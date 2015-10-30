/*
 * Nexus Register(s) API
 *
 * Copyright (C) 2015, Broadcom Corporation
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

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/spinlock.h>

#include <linux/brcmstb/brcmstb.h>
#include <linux/brcmstb/reg_api.h>

#include "gpio.h"

static DEFINE_SPINLOCK(brcmstb_update_lock);

static inline void __do_write(void __iomem *addr, uint32_t mask, uint32_t value)
{
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&brcmstb_update_lock, flags);
	reg = readl_relaxed(addr);
	reg &= ~mask;
	reg |= value;
	writel_relaxed(reg, addr);
	spin_unlock_irqrestore(&brcmstb_update_lock, flags);
}

int brcmstb_update32(uint32_t addr, uint32_t mask, uint32_t value)
{
	void __iomem *paddr;
	int gpio, bit, ret = 0;

	paddr = ioremap(BCHP_PHYSICAL_OFFSET + addr, sizeof(u32));
	if (!paddr)
		return -ENOMEM;

	gpio = brcmstb_gpio_find_by_addr(addr, 0);
	if (gpio < 0) {
		pr_err("%s: addr is not in GPIO range\n", __func__);
		ret = -EPERM;
		goto out;
	}

	for (bit = 0; bit < 32; bit++) {
		/* Ignore bits which are not in mask */
		if (!((1 << bit) & mask))
			continue;

		gpio = brcmstb_gpio_find_by_addr(addr, bit);
		if (gpio < 0) {
			pr_err("%s: bit %d in mask is not found\n", __func__, bit);
			goto out;
		}

		ret = brcmstb_gpio_request(gpio);
		if (ret < 0) {
			pr_err("%s: unable to request gpio %d\n", __func__, gpio);
			goto out;
		}
	}

	/* We got full access to the entire mask, do the write */

	pr_info("%s: offset=0x%08x mask=0x%08x, value=0x%08x\n",
		__func__, addr, mask, value);

	__do_write(paddr, mask, value);
out:
	iounmap(paddr);
	return ret;
}
EXPORT_SYMBOL(brcmstb_update32);
