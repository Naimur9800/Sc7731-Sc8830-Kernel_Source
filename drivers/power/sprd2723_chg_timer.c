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
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/regmap.h>
#include "sprd_battery.h"

#define TIMER_LOAD		(0x0000)
#define TIMER_VALUE		(0x0004)
#define TIMER_CTL		(0x0008)
#define TIMER_INT			(0x000C)

#define PERIOD_MODE			BIT(6)
#define TIMER_DISABLE		(0)
#define TIMER_ENABLE			BIT(7)
#define TIMER_INT_EN			(1)
#define TIMER_INT_STS			BIT(2)
#define TIMER_INT_CLR		BIT(3)
#define TIMER_INT_BUSY		BIT(4)

static uint32_t irq_chg_timer;
static void __iomem *timer_base;

void sprdchg_timer_enable(uint32_t cycles)
{
	writel_relaxed(TIMER_DISABLE | PERIOD_MODE,
		timer_base + TIMER_CTL);
	writel_relaxed(32768 * cycles,
		timer_base + TIMER_LOAD);
	writel_relaxed(TIMER_ENABLE | PERIOD_MODE,
		timer_base  + TIMER_CTL);
	writel_relaxed(TIMER_INT_EN,
		timer_base + TIMER_INT);

}

void sprdchg_timer_disable(void)
{
	writel_relaxed(TIMER_DISABLE | PERIOD_MODE,
		timer_base + TIMER_CTL);
}

static irqreturn_t sprdchg_timer_interrupt(int irq, void *dev_id)
{
	unsigned int value;

	value = readl_relaxed(timer_base + TIMER_INT);
	value |= TIMER_INT_CLR;
	writel_relaxed(value, timer_base + TIMER_INT);
	return IRQ_WAKE_THREAD;
}

int sprdchg_timer_request(irq_handler_t fn_cb, void *data)
{
	int ret = 0;

	ret = request_threaded_irq(irq_chg_timer,
		sprdchg_timer_interrupt, fn_cb,
		IRQF_NO_SUSPEND | __IRQF_TIMER | IRQF_ONESHOT,
		"chg_timer", data);

	if (ret)
		pr_err("request charge timer failed\n");

	return ret;
}

struct sprd_chg_timer_operations sprd_chg_timer_ops = {
	.timer_enable = sprdchg_timer_enable,
	.timer_disable = sprdchg_timer_disable,
	.timer_request = sprdchg_timer_request,
};

static int sprdchg_timer_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct regmap *aon_reg =
	    syscon_regmap_lookup_by_compatible("sprd,sys-aon-apb");

	if (IS_ERR_OR_NULL(aon_reg)) {
		pr_err("%s:failed to find sprd reg\n", __func__);
		return -ENODEV;
	}

	irq_chg_timer = irq_of_parse_and_map(np, 0);
	if (irq_chg_timer <= 0)
		pr_err("%s can't parse timer irq\n", __func__);

	timer_base = of_iomap(np, 0);
	if (!timer_base)
		pr_err("ERROR:%s invalid timer base address\n", __func__);

	regmap_update_bits(aon_reg, REG_AON_APB_APB_EB1,
			   BIT_AON_APB_AP_TMR1_EB, BIT_AON_APB_AP_TMR1_EB);
	sprdchg_timer_disable();

	sprdbat_register_timer_ops(&sprd_chg_timer_ops);
	return 0;
}

static int sprdchg_timer_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id sprdchg_timer_of_match[] = {
	{.compatible = "sprd,chg-timer",},
	{}
};

static struct platform_driver sprdchg_timer_driver = {
	.driver = {
		   .name = "chg-timer",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(sprdchg_timer_of_match),
		   },
	.probe = sprdchg_timer_probe,
	.remove = sprdchg_timer_remove
};

static int __init sprdchg_timer_driver_init(void)
{
	return platform_driver_register(&sprdchg_timer_driver);
}

static void __exit sprdchg_timer_driver_exit(void)
{
	platform_driver_unregister(&sprdchg_timer_driver);
}

subsys_initcall_sync(sprdchg_timer_driver_init);
module_exit(sprdchg_timer_driver_exit);
