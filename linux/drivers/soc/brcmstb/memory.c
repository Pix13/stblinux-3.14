/*
 * Copyright © 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * A copy of the GPL is available at
 * http://www.broadcom.com/licenses/GPLv2.php or from the Free Software
 * Foundation at https://www.gnu.org/licenses/ .
 */

#include <asm/page.h>
#include <asm/setup.h>  /* for meminfo */
#include <linux/device.h>
#include <linux/io.h>
#include <linux/libfdt.h>
#include <linux/memblock.h>
#include <linux/mm.h>   /* for high_memory */
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/vme.h>
#include <linux/brcmstb/bmem.h>
#include <linux/brcmstb/cma_driver.h>
#include <linux/brcmstb/memory_api.h>

/* -------------------- Constants -------------------- */

#define DEFAULT_LOWMEM_PCT	20  /* used if only one membank */

/* Macros to help extract property data */
#define U8TOU32(b, offs) \
	((((u32)b[0+offs] << 0)  & 0x000000ff) | \
	 (((u32)b[1+offs] << 8)  & 0x0000ff00) | \
	 (((u32)b[2+offs] << 16) & 0x00ff0000) | \
	 (((u32)b[3+offs] << 24) & 0xff000000))

#define DT_PROP_DATA_TO_U32(b, offs) (fdt32_to_cpu(U8TOU32(b, offs)))

/* Constants used when retrieving memc info */
#define NUM_BUS_RANGES 10
#define BUS_RANGE_ULIMIT_SHIFT 4
#define BUS_RANGE_LLIMIT_SHIFT 4
#define BUS_RANGE_PA_SHIFT 12

enum {
	BUSNUM_MCP0 = 0x4,
	BUSNUM_MCP1 = 0x5,
	BUSNUM_MCP2 = 0x6,
};

/* -------------------- Shared and local vars -------------------- */

const enum brcmstb_reserve_type brcmstb_default_reserve = BRCMSTB_RESERVE_CMA;
bool brcmstb_memory_override_defaults = false;

static struct {
	struct brcmstb_range range[MAX_BRCMSTB_RESERVED_RANGE];
	int count;
} reserved_init;

/* -------------------- Functions -------------------- */

/*
 * If the DT nodes are handy, determine which MEMC holds the specified
 * physical address.
 */
int brcmstb_memory_phys_addr_to_memc(phys_addr_t pa)
{
	int memc = -1;
	int i;
	struct device_node *np;
	void __iomem *cpubiuctrl = NULL;
	void __iomem *curr;

	np = of_find_compatible_node(NULL, NULL, "brcm,brcmstb-cpu-biu-ctrl");
	if (!np)
		goto cleanup;

	cpubiuctrl = of_iomap(np, 0);
	if (!cpubiuctrl)
		goto cleanup;

	for (i = 0, curr = cpubiuctrl; i < NUM_BUS_RANGES; i++, curr += 8) {
		const u64 ulimit_raw = readl(curr);
		const u64 llimit_raw = readl(curr + 4);
		const u64 ulimit =
			((ulimit_raw >> BUS_RANGE_ULIMIT_SHIFT)
			 << BUS_RANGE_PA_SHIFT) | 0xfff;
		const u64 llimit = (llimit_raw >> BUS_RANGE_LLIMIT_SHIFT)
				   << BUS_RANGE_PA_SHIFT;
		const u32 busnum = (u32)(ulimit_raw & 0xf);

		if (pa >= llimit && pa <= ulimit) {
			if (busnum >= BUSNUM_MCP0 && busnum <= BUSNUM_MCP2) {
				memc = busnum - BUSNUM_MCP0;
				break;
			}
		}
	}

cleanup:
	if (cpubiuctrl)
		iounmap(cpubiuctrl);

	of_node_put(np);

	return memc;
}

static int populate_memc(struct brcmstb_memory *mem, int addr_cells,
		int size_cells)
{
	const void *fdt = initial_boot_params;
	const int mem_offset = fdt_path_offset(fdt, "/memory");
	const struct fdt_property *prop;
	int proplen, cellslen;
	int i;

	if (mem_offset < 0) {
		pr_err("No memory node?\n");
		return -EINVAL;
	}

	prop = fdt_get_property(fdt, mem_offset, "reg", &proplen);
	cellslen = (int)sizeof(u32) * (addr_cells + size_cells);
	if ((proplen % cellslen) != 0) {
		pr_err("Invalid length of reg prop: %d\n", proplen);
		return -EINVAL;
	}

	for (i = 0; i < proplen / cellslen; ++i) {
		u64 addr = 0;
		u64 size = 0;
		int memc_idx;
		int range_idx;
		int j;

		for (j = 0; j < addr_cells; ++j) {
			int offset = (cellslen * i) + (sizeof(u32) * j);
			addr |= (u64)DT_PROP_DATA_TO_U32(prop->data, offset) <<
				((addr_cells - j - 1) * 32);
		}
		for (j = 0; j < size_cells; ++j) {
			int offset = (cellslen * i) +
				(sizeof(u32) * (j + addr_cells));
			size |= (u64)DT_PROP_DATA_TO_U32(prop->data, offset) <<
				((size_cells - j - 1) * 32);
		}

		if ((phys_addr_t)addr != addr) {
			pr_err("phys_addr_t is smaller than provided address 0x%llx!\n",
					addr);
			return -EINVAL;
		}

		memc_idx = brcmstb_memory_phys_addr_to_memc((phys_addr_t)addr);
		if (memc_idx == -1) {
			pr_err("address 0x%llx does not appear to be in any memc\n",
					addr);
			return -EINVAL;
		}

		range_idx = mem->memc[memc_idx].count;
		if (mem->memc[memc_idx].count >= MAX_BRCMSTB_RANGE)
			pr_warn("%s: Exceeded max ranges for memc%d\n",
					__func__, memc_idx);
		else {
			mem->memc[memc_idx].range[range_idx].addr = addr;
			mem->memc[memc_idx].range[range_idx].size = size;
		}
		++mem->memc[memc_idx].count;
	}

	return 0;
}

static int populate_lowmem(struct brcmstb_memory *mem)
{
#ifdef CONFIG_ARM
	mem->lowmem.range[0].addr = __pa(PAGE_OFFSET);
	mem->lowmem.range[0].size = (unsigned long)high_memory - PAGE_OFFSET;
	++mem->lowmem.count;
	return 0;
#else
	return -ENOSYS;
#endif
}

static int populate_bmem(struct brcmstb_memory *mem)
{
#ifdef CONFIG_BRCMSTB_BMEM
	phys_addr_t addr, size;
	int i;

	for (i = 0; i < MAX_BRCMSTB_RANGE; ++i) {
		if (bmem_region_info(i, &addr, &size))
			break;  /* no more regions */
		mem->bmem.range[i].addr = addr;
		mem->bmem.range[i].size = size;
		++mem->bmem.count;
	}
	if (i >= MAX_BRCMSTB_RANGE) {
		while (bmem_region_info(i, &addr, &size) == 0) {
			pr_warn("%s: Exceeded max ranges\n", __func__);
			++mem->bmem.count;
		}
	}

	return 0;
#else
	return -ENOSYS;
#endif
}

static int populate_cma(struct brcmstb_memory *mem)
{
#ifdef CONFIG_BRCMSTB_CMA
	int i;

	for (i = 0; i < CMA_NUM_RANGES; ++i) {
		struct cma_dev *cdev = cma_dev_get_cma_dev(i);
		if (cdev == NULL)
			break;
		if (i >= MAX_BRCMSTB_RANGE)
			pr_warn("%s: Exceeded max ranges\n", __func__);
		else {
			mem->cma.range[i].addr = cdev->range.base;
			mem->cma.range[i].size = cdev->range.size;
		}
		++mem->cma.count;
	}

	return 0;
#else
	return -ENOSYS;
#endif
}

static int populate_reserved(struct brcmstb_memory *mem)
{
#ifdef CONFIG_HAVE_MEMBLOCK
	memcpy(&mem->reserved, &reserved_init, sizeof(reserved_init));
	return 0;
#else
	return -ENOSYS;
#endif
}

/**
 * brcmstb_memory_get_default_reserve() - find default reservation for given ID
 * @bank_nr: bank index
 * @pstart: pointer to the start address (output)
 * @psize: pointer to the size address (output)
 *
 * NOTE: This interface will change in future kernels that do not have meminfo
 *
 * This takes in the bank number and determines the size and address of the
 * default region reserved for refsw within the bank.
 */
int __init brcmstb_memory_get_default_reserve(int bank_nr,
		phys_addr_t *pstart, phys_addr_t *psize)
{
	/* min alignment for mm core */
	const phys_addr_t alignment =
		PAGE_SIZE << max(MAX_ORDER - 1, pageblock_order);
	struct membank *bank = &meminfo.bank[bank_nr];
	phys_addr_t start = bank->start;
	phys_addr_t size = 0;
	phys_addr_t newstart, newsize;
	int i;

	if (!pstart || !psize)
		return -EFAULT;

	if (bank_nr == 0) {
		if (meminfo.nr_banks == 1) {
			u32 rem;
			u64 tmp;

			BUG_ON(bank->highmem);
			if (bank->size < SZ_32M) {
				pr_err("low memory too small for default bmem\n");
				return -EINVAL;
			}

			/* kernel reserves X percent, bmem gets the rest */
			tmp = ((u64)bank->size) * (100 - DEFAULT_LOWMEM_PCT);
			rem = do_div(tmp, 100);
			size = tmp + rem;
			start = bank->start + bank->size - size;
		} else {
			/* If more than one bank, don't use first bank */
			return -EINVAL;
		}
	} else if (bank->start >= VME_A32_MAX && bank->size > SZ_64M) {
		/*
		 * Nexus doesn't use the address extension range yet, just
		 * reserve 64 MiB in these areas until we have a firmer
		 * specification
		 */
		size = SZ_64M;
	} else {
		size = bank->size;
	}

	/*
	 * To keep things simple, we only handle the case where reserved memory
	 * is at the start or end of a region.
	 */
	i = 0;
	while (i < memblock.reserved.cnt) {
		struct memblock_region *region = &memblock.reserved.regions[i];
		newstart = start;
		newsize = size;

		if (start >= region->base &&
				start < region->base + region->size) {
			newstart = region->base + region->size;
			newsize = size - (newstart - start);
		} else if (start < region->base) {
			if (start + size < region->base + region->size) {
				newsize = region->base - start;
			} else if (start + size >=
					region->base + region->size) {
				/* unhandled condition */
				pr_err("%s: Split region %pa@%pa, reserve will fail\n",
						__func__, &size, &start);
				/* enable 'debug' param for dump output */
				memblock_dump_all();
				return -EINVAL;
			}
		}
		/* see if we had any modifications */
		if (newsize != size || newstart != start) {
			pr_debug("%s: moving default region from %pa@%pa to %pa@%pa\n",
					__func__, &size, &start, &newsize,
					&newstart);
			size = newsize;
			start = newstart;
			i = 0; /* start over */
		} else {
			++i;
		}
	}

	/* Fix up alignment */
	newstart = ALIGN(start, alignment);
	if (newstart != start) {
		pr_debug("adjusting start from %pa to %pa\n",
				&start, &newstart);
		start = newstart;
	}
	newsize = round_down(size, alignment);
	if (newsize != size) {
		pr_debug("adjusting size from %pa to %pa\n",
				&size, &newsize);
		size = newsize;
	}

	if (size == 0) {
		pr_debug("size available in bank was 0 - skipping\n");
		return -EINVAL;
	}

	*pstart = start;
	*psize = size;

	return 0;
}

/**
 * brcmstb_memory_reserve() - fill in static brcmstb_memory structure
 *
 * This is a boot-time initialization function used to copy the information
 * stored in the memblock reserve function that is discarded after boot.
 */
void __init brcmstb_memory_reserve(void)
{
#ifdef CONFIG_HAVE_MEMBLOCK
	struct memblock_type *type = &memblock.reserved;
	int i;

	for (i = 0; i < type->cnt; ++i) {
		struct memblock_region *region = &type->regions[i];

		if (i >= MAX_BRCMSTB_RESERVED_RANGE)
			pr_warn_once("%s: Exceeded max ranges\n", __func__);
		else {
			reserved_init.range[i].addr = region->base;
			reserved_init.range[i].size = region->size;
		}
		++reserved_init.count;
	}
#else
	pr_err("No memblock, cannot get reserved range\n");
#endif
}

/*
 * brcmstb_memory_get() - fill in brcmstb_memory structure
 * @mem: pointer to allocated struct brcmstb_memory to fill
 *
 * The brcmstb_memory struct is required by the brcmstb middleware to
 * determine how to set up its memory heaps.  This function expects that the
 * passed pointer is valid.  The struct does not need to have be zeroed
 * before calling.
 */
int brcmstb_memory_get(struct brcmstb_memory *mem)
{
	const void *fdt = initial_boot_params;
	const struct fdt_property *prop;
	int addr_cells = 1, size_cells = 1;
	int proplen;
	int ret;

	if (!mem)
		return -EFAULT;

	if (!fdt) {
		pr_err("No device tree?\n");
		return -EINVAL;
	}

	/* Get root size and address cells if specified */
	prop = fdt_get_property(fdt, 0, "#size-cells", &proplen);
	if (prop)
		size_cells = DT_PROP_DATA_TO_U32(prop->data, 0);
	pr_debug("size_cells = %x\n", size_cells);

	prop = fdt_get_property(fdt, 0, "#address-cells", &proplen);
	if (prop)
		addr_cells = DT_PROP_DATA_TO_U32(prop->data, 0);
	pr_debug("address_cells = %x\n", addr_cells);

	memset(mem, 0, sizeof(*mem));

	ret = populate_memc(mem, addr_cells, size_cells);
	if (ret)
		return ret;

	ret = populate_lowmem(mem);
	if (ret)
		return ret;

	ret = populate_bmem(mem);
	if (ret)
		pr_debug("bmem is disabled\n");

	ret = populate_cma(mem);
	if (ret)
		pr_debug("cma is disabled\n");

	ret = populate_reserved(mem);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL(brcmstb_memory_get);
