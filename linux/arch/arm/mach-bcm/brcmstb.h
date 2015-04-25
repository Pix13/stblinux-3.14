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
extern void brcmstb_secondary_startup(void);
#ifdef CONFIG_CMA
extern void __init cma_reserve(void);
extern void __init cma_register(void);
#else
static inline void cma_reserve(void) { }
static inline void cma_register(void) { }
#endif
#endif

#endif /* __BRCMSTB_H__ */
