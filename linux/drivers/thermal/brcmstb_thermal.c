/*
 * Broadcom STB AVS TMON thermal sensor driver
 *
 * Copyright (C) 2015 Broadcom Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/thermal.h>

#define DRV_NAME	"brcmstb_thermal"

#define AVS_TMON_STATUS			0x00
 #define AVS_TMON_STATUS_valid_msk	BIT(11)
 #define AVS_TMON_STATUS_data_msk	GENMASK(10, 1)
 #define AVS_TMON_STATUS_data_shift	1

#define AVS_TMON_EN_OVERTEMP_RESET	0x04
 #define AVS_TMON_EN_OVERTEMP_RESET_msk	BIT(0)

#define AVS_TMON_RESET_THRESH		0x08
 #define AVS_TMON_RESET_THRESH_msk	GENMASK(10, 1)
 #define AVS_TMON_RESET_THRESH_shift	1

#define AVS_TMON_INT_IDLE_TIME		0x10

#define AVS_TMON_EN_TEMP_INT_SRCS	0x14
 #define AVS_TMON_EN_TEMP_INT_SRCS_high	BIT(1)
 #define AVS_TMON_EN_TEMP_INT_SRCS_low	BIT(0)

#define AVS_TMON_INT_THRESH		0x18
 #define AVS_TMON_INT_THRESH_high_msk	GENMASK(26, 17)
 #define AVS_TMON_INT_THRESH_high_shift	17
 #define AVS_TMON_INT_THRESH_low_msk	GENMASK(10, 1)
 #define AVS_TMON_INT_THRESH_low_shift	1

#define AVS_TMON_TEMP_INT_CODE		0x1c
#define AVS_TMON_TP_TEST_ENABLE		0x20

struct brcmstb_thermal_priv {
	void __iomem *tmon_base;
	struct thermal_zone_device *thermal;
};

/* Convert a HW code to a temperature reading (millidegree celsius) */
static inline int avs_tmon_code_to_temp(u32 code)
{
	return (4100400U - code * 4870U) / 10U;
}

/* Convert a temperature value (millidegree celsius) to a HW code */
static inline u32 avs_tmon_temp_to_code(unsigned long temp)
{
	return (4100400U - temp * 10U) / 4870U;
}

static int brcmstb_get_temp(struct thermal_zone_device *thermal,
			    unsigned long *temp)
{
	struct brcmstb_thermal_priv *priv = thermal->devdata;
	u32 val;

	val = __raw_readl(priv->tmon_base + AVS_TMON_STATUS);

	if (!(val & AVS_TMON_STATUS_valid_msk)) {
		dev_err(&thermal->device, "reading not valid\n");
		return -EIO;
	}

	val = (val & AVS_TMON_STATUS_data_msk) >> AVS_TMON_STATUS_data_shift;

	*temp = avs_tmon_code_to_temp(val);

	return 0;
}

static struct thermal_zone_device_ops ops = {
	.get_temp	= brcmstb_get_temp,
};

static const struct of_device_id brcmstb_thermal_id_table[] = {
	{ .compatible = "brcm,avs-tmon" },
	{},
};
MODULE_DEVICE_TABLE(of, brcmstb_thermal_id_table);

static int brcmstb_thermal_probe(struct platform_device *pdev)
{
	struct thermal_zone_device *thermal;
	struct brcmstb_thermal_priv *priv;
	struct resource *res;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->tmon_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->tmon_base))
		return PTR_ERR(priv->tmon_base);

	thermal = thermal_zone_device_register("avs_tmon", 0, 0,
					       priv, &ops, NULL, 0, 0);
	if (IS_ERR(thermal)) {
		dev_err(&pdev->dev,
			"Failed to register thermal zone device\n");
		return PTR_ERR(thermal);
	}

	platform_set_drvdata(pdev, thermal);
	priv->thermal = thermal;

	dev_info(&pdev->dev, "registered AVS TMON driver\n");

	return 0;
}

static int brcmstb_thermal_exit(struct platform_device *pdev)
{
	struct thermal_zone_device *thermal = platform_get_drvdata(pdev);

	thermal_zone_device_unregister(thermal);

	return 0;
}

static struct platform_driver brcmstb_thermal_driver = {
	.probe = brcmstb_thermal_probe,
	.remove = brcmstb_thermal_exit,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = brcmstb_thermal_id_table,
	},
};
module_platform_driver(brcmstb_thermal_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Brian Norris");
MODULE_DESCRIPTION("Broadcom STB AVS TMON thermal driver");
