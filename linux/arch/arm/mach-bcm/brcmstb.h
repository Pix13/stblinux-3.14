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

#ifndef __BRCMSTB_H__
#define __BRCMSTB_H__

#if !defined(__ASSEMBLY__)
#include <linux/smp.h>
#endif

#if !defined(__ASSEMBLY__)
extern void brcmstb_secondary_startup(void);
extern void brcmstb_cpu_boot(unsigned int cpu);
extern void brcmstb_cpu_power_on(unsigned int cpu);
extern int brcmstb_cpu_get_power_state(unsigned int cpu);
extern struct smp_operations brcmstb_smp_ops;
#if defined(CONFIG_HOTPLUG_CPU)
extern void brcmstb_cpu_die(unsigned int cpu);
extern int brcmstb_cpu_kill(unsigned int cpu);
void __init brcmstb_cpu_ctrl_setup(unsigned int max_cpus);
#else
static inline void brcmstb_cpu_die(unsigned int cpu) {}
static inline int brcmstb_cpu_kill(unsigned int cpu)
{
	return 0;
}
static inline void __init brcmstb_cpu_ctrl_setup(unsigned int max_cpus) {}
#endif
#ifdef CONFIG_CMA
extern void __init cma_reserve(void);
extern void __init cma_register(void);
#else
static inline void cma_reserve(void) { }
static inline void cma_register(void) { }
#endif
#endif

#ifdef CONFIG_PM
int brcmstb_regsave_init(void);
int brcmstb_pm_init(void);
#else
static inline int brcmstb_pm_init(void)
{
	return 0;
}

static inline int brcmstb_regsave_init(void)
{
	return 0;
}
#endif /* CONFIG_PM */

extern void brcmstb_hook_fault_code(void);

void brcmstb_irq0_init(void);

#endif /* __BRCMSTB_H__ */
