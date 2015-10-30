/*
 * Nexus GPIO(s) resolution API
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
#include <linux/bitmap.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/basic_mmio_gpio.h>

#include <linux/brcmstb/brcmstb.h>
#include <linux/brcmstb/gpio_api.h>

#include "gpio.h"

static const char *brcmstb_gpio_compat = GPIO_DT_COMPAT;

static int brcmstb_gpio_chip_find(struct gpio_chip *chip, void *data)
{
	struct device_node *dn = data;

	if (chip->of_node == dn)
		return 1;

	return 0;
}

/* Keep an additional bitmap of which GPIOs are requested by Nexus to
 * maintain Linux/Nexus GPIO exclusive ownership managed via gpio_request().
 * We do want multiple consecutive calls to brcmstb_gpio_irq() not to fail
 * because the first one claimed ownernship already.
 */
static DECLARE_BITMAP(brcmstb_gpio_requested, CONFIG_ARCH_NR_GPIO);

int brcmstb_gpio_request(unsigned int gpio)
{
	int ret = 0;

	/* Request the GPIO to flag that Nexus now owns it, but
	 * also keep it requested in our local bitmap to avoid
	 * subsequent gpio_request() calls to the same GPIO
	 */
	if (!test_bit(gpio, brcmstb_gpio_requested)) {
		ret = gpio_request(gpio, "nexus");
		if (ret) {
			pr_err("%s: GPIO request failed\n", __func__);
			return ret;
		}

		/* We could get ownership, so flag it now */
		set_bit(gpio, brcmstb_gpio_requested);
	}

	return ret;
}

int brcmstb_gpio_find_by_addr(uint32_t addr, unsigned int shift)
{
	struct device_node *dn;
	struct gpio_chip *gc;
	struct resource res;
	int ret, gpio;
	u32 bank_offset;

	for_each_compatible_node(dn, NULL, brcmstb_gpio_compat) {
		ret = of_address_to_resource(dn, 0, &res);
		if (ret) {
			pr_err("%s: unable to translate resource\n", __func__);
			continue;
		}

		if (res.flags != IORESOURCE_MEM) {
			pr_err("%s: invalid resource type\n", __func__);
			continue;
		}

		/* Mask address with the physical offset, since the address has
		 * already been translated based on the busing hierarchy
		 */
		res.start &= ~BCHP_PHYSICAL_OFFSET;
		res.end &= ~BCHP_PHYSICAL_OFFSET;

		/* Verify address in the resource range, if not, go to the
		 * other GPIO controllers
		 */
		if (addr < res.start || addr >= res.end)
			continue;

		gc = gpiochip_find(dn, brcmstb_gpio_chip_find);
		if (!gc) {
			pr_err("%s: unable to find gpio chip\n", __func__);
			continue;
		}

		/* Now find out what GPIO bank this pin belongs to */
		bank_offset = (addr - res.start) / GIO_BANK_SIZE;

		pr_debug("%s: bank_offset is %d\n", __func__, bank_offset);

		gpio = gc->base + shift + bank_offset * GIO_BANK_SIZE;

		pr_debug("%s: xlate base=%d, offset=%d, shift=%d gpio=%d\n",
			__func__, gc->base, bank_offset, shift, gpio - gc->base);

		return gpio;
	}

	return -ENODEV;
}

int brcmstb_gpio_irq(uint32_t addr, unsigned int shift)
{
	int gpio, ret;

	gpio = brcmstb_gpio_find_by_addr(addr, shift);
	if (gpio < 0)
		return gpio;

	ret = brcmstb_gpio_request(gpio);
	if (ret < 0)
		return ret;

	ret = gpio_to_irq(gpio);
	if (ret < 0) {
		gpio_free(gpio);
		pr_err("%s: unable to map GPIO%d to irq, ret=%d\n",
		       __func__, gpio, ret);
	}

	return ret;
}
EXPORT_SYMBOL(brcmstb_gpio_irq);
