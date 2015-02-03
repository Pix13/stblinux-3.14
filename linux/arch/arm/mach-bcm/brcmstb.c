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
#include <linux/syscore_ops.h>
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
 * - XPT DMA
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
	/* Force full-duplex settings here, since BOLT v0.86 would set MoCA
	 * links to half-duplex, and that might cause packet losses since the
	 * link between GENET or SWITCH and MoCA's ECL is full-duplex.
	 *
	 * BOLT does not support configuring the duplex type, so we can safely
	 * override this to DUPLEX_FULL.
	 */
	status.duplex = DUPLEX_FULL;
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

	/* GENET uses the 'fixed-link' property */
	for_each_compatible_node(np, NULL, "brcm,genet-v4")
		of_add_one_fixed_phy(np);

	return 0;
}
#else
static inline void of_add_fixed_phys(void)
{
}
#endif /* CONFIG_FIXED_PHY */

#define CPU_CREDIT_REG_OFFSET 0x184
#define  CPU_CREDIT_REG_MCPx_WR_PAIRING_EN_MASK 0x70000000

static void __iomem *cpubiuctrl_base;

/*
 * HW7445-1920: Disable MCP write pairing to improve stability on long term
 * stress test.
 */
static int __init disable_mcp_write_pairing(void)
{
	u32 creds = 0;

	if (!cpubiuctrl_base)
		return -1;

	creds = __raw_readl(cpubiuctrl_base + CPU_CREDIT_REG_OFFSET);
	if (creds & CPU_CREDIT_REG_MCPx_WR_PAIRING_EN_MASK) {
		pr_info("MCP: Disabling write pairing\n");
		__raw_writel(creds & ~CPU_CREDIT_REG_MCPx_WR_PAIRING_EN_MASK,
				cpubiuctrl_base + CPU_CREDIT_REG_OFFSET);
	} else {
		pr_info("MCP: Write pairing already disabled\n");
	}

	return 0;
}

static void __init setup_hifcpubiuctrl_regs(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "brcm,brcmstb-cpu-biu-ctrl");
	if (!np)
		pr_err("missing BIU control node\n");

	cpubiuctrl_base = of_iomap(np, 0);
	if (!cpubiuctrl_base)
		pr_err("failed to remap BIU control base\n");

	of_node_put(np);
}

#ifdef CONFIG_PM_SLEEP
static u32 cpu_credit_reg_dump;  /* for save/restore */

static int brcmstb_cpu_credit_reg_suspend(void)
{
	if (cpubiuctrl_base)
		cpu_credit_reg_dump =
			__raw_readl(cpubiuctrl_base + CPU_CREDIT_REG_OFFSET);
	return 0;
}

static void brcmstb_cpu_credit_reg_resume(void)
{
	if (cpubiuctrl_base)
		__raw_writel(cpu_credit_reg_dump,
				cpubiuctrl_base + CPU_CREDIT_REG_OFFSET);
}

static struct syscore_ops brcmstb_cpu_credit_syscore_ops = {
	.suspend = brcmstb_cpu_credit_reg_suspend,
	.resume = brcmstb_cpu_credit_reg_resume,
};
#endif

void brcmstb_irq0_init(void)
{
	BDEV_WR(BCHP_IRQ0_IRQEN, BCHP_IRQ0_IRQEN_uarta_irqen_MASK
		| BCHP_IRQ0_IRQEN_uartb_irqen_MASK
		| BCHP_IRQ0_IRQEN_uartc_irqen_MASK
	);
}

static void __init brcmstb_init_irq(void)
{
	/* Force lazily-disabled IRQs to be masked before suspend */
	gic_arch_extn.flags |= IRQCHIP_MASK_ON_SUSPEND;

	brcmstb_irq0_init();
	irqchip_init();
}

static void __init brcmstb_init_machine(void)
{
	struct platform_device_info devinfo = { .name = "cpufreq-cpu0", };
	int ret;

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	cma_register();
	of_add_fixed_phys();
	brcmstb_hook_fault_code();
	ret = brcmstb_pm_init();
	if (ret)
		pr_warn("PM: initialization failed with code %d\n", ret);
	platform_device_register_full(&devinfo);
#ifdef CONFIG_PM_SLEEP
	register_syscore_ops(&brcmstb_cpu_credit_syscore_ops);
#endif
}

static void __init brcmstb_init_early(void)
{
	setup_hifcpubiuctrl_regs();
	if (disable_mcp_write_pairing())
		pr_err("MCP: Unable to disable write pairing!\n");
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

static void __init brcmstb_init_time(void)
{
	brcmstb_clocks_init();
	clocksource_of_init();
}
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
	.init_time	= brcmstb_init_time,
#endif
#ifdef CONFIG_SMP
	.smp		= smp_ops(brcmstb_smp_ops),
#endif
MACHINE_END
