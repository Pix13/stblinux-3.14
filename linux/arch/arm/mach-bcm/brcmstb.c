/*
 * Copyright (C) 2013 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk-provider.h>
#include <linux/console.h>
#include <linux/clocksource.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-contiguous.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/smp.h>
#if defined(CONFIG_BRCMSTB)
#include <linux/brcmstb/brcmstb.h>
#include <linux/brcmstb/cma_driver.h>
#endif
#include <linux/clk/clk-brcmstb.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <linux/irqchip.h>
#include <linux/irqchip/arm-gic.h>
#include <asm/cacheflush.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include "brcmstb.h"

/***********************************************************************
 * STB CPU (main application processor)
 ***********************************************************************/

static const char *brcmstb_match[] __initconst = {
	"brcm,bcm7445",
	"brcm,brcmstb",
	NULL
};

#if defined(CONFIG_BRCMSTB)
/*
 * HACK: The following drivers are still using BDEV macros:
 * - PCIe (bridge setup)
 *
 * Once these drivers have migrated over to using 'of_iomap()' and standard
 * register accessors, we can eliminate this static mapping.
 */
static struct map_desc brcmstb_io_map[] __initdata = {
	{
	.virtual = (unsigned long)BRCMSTB_PERIPH_VIRT,
	.pfn     = __phys_to_pfn(BRCMSTB_PERIPH_PHYS),
	.length  = BRCMSTB_PERIPH_LENGTH,
	.type    = MT_DEVICE,
	},
};

static void __init brcmstb_map_io(void)
{
	iotable_init(brcmstb_io_map, ARRAY_SIZE(brcmstb_io_map));
}

static void __init brcmstb_reserve(void)
{
	cma_reserve();
}

#ifdef CONFIG_FIXED_PHY
static int of_add_one_fixed_phy(struct device_node *np)
{
	struct fixed_phy_status status = {};
 	u32 *fixed_link;
	int ret;

	fixed_link  = (u32 *)of_get_property(np, "fixed-link", NULL);
	if (!fixed_link)
		return 1;

	status.link = 1;
	status.duplex = be32_to_cpu(fixed_link[1]);
	status.speed = be32_to_cpu(fixed_link[2]);
	status.pause = be32_to_cpu(fixed_link[3]);
	status.asym_pause = be32_to_cpu(fixed_link[4]);

	ret = fixed_phy_add(PHY_POLL, be32_to_cpu(fixed_link[0]), &status);
	if (ret)
		of_node_put(np);

	return ret;
}

static int __init of_add_fixed_phys(void)
{
	struct device_node *np, *child, *port;
	struct fixed_phy_status status = {};
	unsigned int i = 0;
	u32 phy_type;

	for_each_compatible_node(np, NULL, "brcm,bcm7445-switch-v4.0") {
		for_each_child_of_node(np, child) {
			for_each_child_of_node(child, port) {
				if (of_add_one_fixed_phy(port))
					continue;
			}
		}
	}

	/* SYSTEMPORT Ethernet MAC also uses the 'fixed-link' property */
	for_each_compatible_node(np, NULL, "brcm,systemport-v1.00")
		of_add_one_fixed_phy(np);

	/* For compatibility with the old DT binding, we just match
	 * against our specific Ethernet driver compatible property
	 * and we just parse the speed settings for the fixed-PHY
	 */
	for_each_compatible_node(np, NULL, "brcm,genet-v4") {
		status.link = 1;
		status.duplex = DUPLEX_FULL;
		status.pause = 0;
		status.asym_pause = 0;

		/* Look for the old binding, identified by the 'phy-type'
		 * property existence
		 */
		if (!of_property_read_u32(np, "phy-type", &phy_type)) {
			/* Do not register a fixed PHY for internal PHYs */
			if (phy_type == BRCM_PHY_TYPE_INT)
				continue;

			if (!of_property_read_u32(np, "phy-speed",
						&status.speed)) {
				/* Convention with the old (inflexible) binding
				 * is 0 -> MoCA, 1 -> anything else
				 */
				if (phy_type == BRCM_PHY_TYPE_MOCA)
					i = 0;
				else
					i = 1;
				fixed_phy_add(PHY_POLL, i, &status);
			}
		} else {
			/* Or try the new, standard 'fixed-link' binding */
			of_add_one_fixed_phy(np);
		}
	}

	return 0;
}
#else
static inline void of_add_fixed_phys(void)
{
}
#endif /* CONFIG_FIXED_PHY */

static void __init brcmstb_init_irq(void)
{
	/* Force lazily-disabled IRQs to be masked before suspend */
	gic_arch_extn.flags |= IRQCHIP_MASK_ON_SUSPEND;

	BDEV_WR(BCHP_IRQ0_IRQEN, BCHP_IRQ0_IRQEN_uarta_irqen_MASK
		| BCHP_IRQ0_IRQEN_uartb_irqen_MASK
		| BCHP_IRQ0_IRQEN_uartc_irqen_MASK
	);

	irqchip_init();
}

static void __init brcmstb_init_machine(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	cma_register();
	of_add_fixed_phys();
	brcmstb_hook_fault_code();
	brcmstb_clocks_init();
	brcmstb_pm_init();
}

static void __init brcmstb_init_early(void)
{
	add_preferred_console("ttyS", 0, "115200");
}

#ifdef CONFIG_BRCMSTB_USE_MEGA_BARRIER
static phys_addr_t so_mem_paddr[NR_BANKS];
static void __iomem *so_mem_vaddr[NR_BANKS];

static struct cma_dev *cma_dev_get_by_addr(phys_addr_t start, phys_addr_t end)
{
	int i = 0;
	for (i = 0; i < CMA_DEV_MAX; i++) {
		struct cma_dev *cma_dev = cma_dev_get_cma_dev(i);
		if (!cma_dev)
			continue;

		if (cma_dev->range.base >= start &&
		    (cma_dev->range.base + cma_dev->range.size) <= end)
			return cma_dev;
	}

	return NULL;
}

static int brcmstb_mega_barrier_init(void)
{
	int bank_nr;

	pr_info("brcmstb: setting up mega-barrier workaround\n");

	for_each_bank(bank_nr, &meminfo) {
		struct page *page;
		struct cma_dev *cma_dev;
		const struct membank *bank = &meminfo.bank[bank_nr];
		const int len = PAGE_SIZE;

		cma_dev = cma_dev_get_by_addr(bank_phys_start(bank),
						bank_phys_end(bank));
		if (!cma_dev) {
			phys_addr_t start = bank_phys_start(bank);
			phys_addr_t end = bank_phys_end(bank);
			pr_warn("no cma dev for addr range (%pa,%pa) exists\n",
					&start,
					&end);
			continue;
		}

		page = dma_alloc_from_contiguous(cma_dev->dev,
							len >> PAGE_SHIFT, 0);
		if (!page) {
			pr_err("failed to alloc page for dummy store on bank %d\n",
				bank_nr);
			continue;
		}

		so_mem_paddr[bank_nr] = page_to_phys(page);
		so_mem_vaddr[bank_nr] = cma_dev_kva_map(page, len >> PAGE_SHIFT,
					pgprot_noncached(pgprot_kernel));
	}

	return 0;
}
late_initcall(brcmstb_mega_barrier_init);

/*
 * The suggested workaround requires a dummy store to memory mapped as
 * STRONGLY ORDERED on each MEMC, followed by a data sync barrier.
 *
 * This function should be called following all cache flush operations.
 */
void brcmstb_mega_barrier(void)
{
	int bank_nr;

	__asm__("dsb");

	for (bank_nr = 0; bank_nr < NR_BANKS; bank_nr++) {
		if (so_mem_vaddr[bank_nr])
			writel_relaxed(0, so_mem_vaddr[bank_nr]);
	}

	__asm__("dsb");
}
EXPORT_SYMBOL(brcmstb_mega_barrier);
#endif /* CONFIG_BRCMSTB_USE_MEGA_BARRIER */
#endif /* CONFIG_BRCMSTB */

/***********************************************************************
 * SMP boot
 ***********************************************************************/

#ifdef CONFIG_SMP
static DEFINE_SPINLOCK(boot_lock);

static void brcmstb_secondary_init(unsigned int cpu)
{
	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

static int brcmstb_boot_secondary(unsigned int cpu,
				  struct task_struct *idle)
{
	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/* Bring up power to the core if necessary */
	if (brcmstb_cpu_get_power_state(cpu) == 0)
		brcmstb_cpu_power_on(cpu);

	brcmstb_cpu_boot(cpu);

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return 0;
}

struct smp_operations brcmstb_smp_ops __initdata = {
	.smp_prepare_cpus	= brcmstb_cpu_ctrl_setup,
	.smp_secondary_init	= brcmstb_secondary_init,
	.smp_boot_secondary	= brcmstb_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_kill		= brcmstb_cpu_kill,
	.cpu_die		= brcmstb_cpu_die,
#endif
};
#endif  /* CONFIG_SMP */

DT_MACHINE_START(BRCMSTB, "Broadcom STB (Flattened Device Tree)")
	.dt_compat	= brcmstb_match,
#if defined(CONFIG_BRCMSTB)
	.map_io		= brcmstb_map_io,
	.reserve	= brcmstb_reserve,
	.init_machine	= brcmstb_init_machine,
	.init_early	= brcmstb_init_early,
	.init_irq	= brcmstb_init_irq,
#endif
#ifdef CONFIG_SMP
	.smp		= smp_ops(brcmstb_smp_ops),
#endif
MACHINE_END
