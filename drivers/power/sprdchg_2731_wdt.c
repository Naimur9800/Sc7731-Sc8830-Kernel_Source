/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/errno.h>
#include <linux/regmap.h>
#include "sprd_battery.h"
#include "sprd_2731.h"
#include "sprd_charge_helper.h"

#define SPRDCHG_2731WDT_DEBUG(format, arg...) pr_info("sprd 2731wdt:" format, ## arg)

static unsigned long base_addr;
static struct regmap *reg_map;

static int chg_wdt_adi_read(unsigned long reg)
{
	unsigned int val;

	regmap_read(reg_map, base_addr + reg, &val);
	return val;
}

static int chg_wdt_adi_write(unsigned long reg, unsigned int or_val)
{
	return regmap_write(reg_map, base_addr + reg, or_val);
}

void sprdchg_reset_wdt(int sec)
{
	unsigned int value = sec * 32768;

	if (chg_wdt_adi_read(CHG_WDG_LOCK))
		chg_wdt_adi_write(CHG_WDG_LOCK, 0xE551);
	while (chg_wdt_adi_read(CHG_WDG_RAW_STATUS) & BIT(4))
		;
	chg_wdt_adi_write(CHG_WDG_LOAD_HIGH,
		(uint16_t)(((value) >> 16) & 0xffff));
	chg_wdt_adi_write(CHG_WDG_LOAD_LOW,
		(uint16_t)((value)  & 0xffff));
}

void sprdchg_wdt_stop(void)
{
	chg_wdt_adi_write(CHG_WDG_CLR, 0x3);
	glb_adi_write(ANA_REG_GLB_MODULE_EN1,
		      0x0, BIT_ANA_CHG_WDG_EN);
	glb_adi_write(ANA_REG_GLB_RTC_CLK_EN1,
		      0x0, BIT_RTC_CHG_WDG_EN);
}

static int sprdchg_wdt_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	u32 value = 0;
	int ret = 0;

	reg_map =
		dev_get_regmap(pdev->dev.parent, NULL);
	if (!reg_map) {
		dev_err(&pdev->dev, "%s :NULL regmap for charge 2731\n",
			__func__);
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "reg", &value);
	if (ret) {
		dev_err(&pdev->dev, "%s :no reg of property for pmic fgu\n",
			__func__);
		return -ENODEV;
	}
	base_addr = (unsigned long)value;

	pr_info("sprd2731 chg-wdt probe ok\n");
	return ret;
}

static int sprdchg_wdt_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id sprdchg_wdt_of_match[] = {
	{.compatible = "sprd,sc2731-chg-wdt",},
	{}
};

static struct platform_driver sprdchg_wdt_driver = {
	.driver = {
		   .name = "chg-wdt",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(sprdchg_wdt_of_match),
		   },
	.probe = sprdchg_wdt_probe,
	.remove = sprdchg_wdt_remove
};

static int __init sprdchg_wdt_driver_init(void)
{
	return platform_driver_register(&sprdchg_wdt_driver);
}

static void __exit sprdchg_wdt_driver_exit(void)
{
	platform_driver_unregister(&sprdchg_wdt_driver);
}

subsys_initcall_sync(sprdchg_wdt_driver_init);
module_exit(sprdchg_wdt_driver_exit);

