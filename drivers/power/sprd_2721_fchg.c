/*
 * Copyright (C) 2018 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/completion.h>

#include "sprd_charge_helper.h"

#undef pr_fmt
#define pr_fmt(fmt) "fchg_2721:" fmt

#define FCHG1_TIME1				0x0
#define FCHG1_TIME2				0x4
#define FCHG1_DELAY				0x8
#define FCHG2_DET_HIGH				0xc
#define FCHG2_DET_LOW				0x10
#define FCHG2_DET_LOW_CV			0x14
#define FCHG2_DET_HIGH_CV			0x18
#define FCHG2_DET_LOW_CC			0x1c
#define FCHG2_ADJ_TIME1				0x20
#define FCHG2_ADJ_TIME2				0x24
#define FCHG2_ADJ_TIME3				0x28
#define FCHG2_ADJ_TIME4				0x2c
#define FCHG_CTRL				0x30
#define FCHG_ADJ_CTRL				0x34
#define FCHG_INT_EN				0x38
#define FCHG_INT_CLR				0x3c
#define FCHG_INT_STS				0x40
#define FCHG_INT_STS0				0x44
#define FCHG_ERR_STS				0x48

#define FCHG_ENABLE_BIT				BIT(0)
#define FCHG_INT_EN_BIT				BIT(1)
#define FCHG_INT_CLR_MASK			BIT(1)
#define FCHG_TIME1_MASK				GENMASK(10, 0)
#define FCHG_TIME2_MASK				GENMASK(11, 0)
#define FCHG_DET_VOL_MASK			GENMASK(1, 0)
#define FCHG_DET_VOL_SHIFT			3

#define FCHG_ERR0_BIT				BIT(1)
#define FCHG_ERR1_BIT				BIT(2)
#define FCHG_ERR2_BIT				BIT(3)
#define FCHG_OUT_OK_BIT				BIT(0)

#define FCHG_INT_STS_DETDONE			BIT(5)

/* FCHG1_TIME1_VALUE is used for detect the time of V > VT1 */
#define FCHG1_TIME1_VALUE			0x514
/* FCHG1_TIME2_VALUE is used for detect the time of V > VT2 */
#define FCHG1_TIME2_VALUE			0x9c4

#define FCHG_VOLTAGE_1				9000
#define FCHG_VOLTAGE_2				12000
#define FCHG_VOLTAGE_3				20000

struct sprd_fchg_data {
	u32 base;
	int irq;
	int detect_ok;
	struct device *dev;
	struct regmap *regmap;
	struct completion completion;
};

static struct sprd_fchg_data *data;

static irqreturn_t sprd_fchg_interrupt(int irq, void *dev_id)
{
	int ret;
	u32 int_sts, int_sts0;

	ret = regmap_read(data->regmap, data->base + FCHG_INT_STS, &int_sts);
	if (ret)
		return IRQ_RETVAL(ret);

	ret = regmap_read(data->regmap, data->base + FCHG_INT_STS0, &int_sts0);
	if (ret)
		return IRQ_RETVAL(ret);

	ret = regmap_update_bits(data->regmap, data->base + FCHG_INT_EN,
				 FCHG_INT_EN_BIT, 0);
	if (ret) {
		dev_err(data->dev, "failed to disable fast charger irq.\n");
		return IRQ_RETVAL(ret);
	}

	ret = regmap_update_bits(data->regmap, data->base + FCHG_INT_CLR,
				 FCHG_INT_CLR_MASK, FCHG_INT_CLR_MASK);
	if (ret) {
		dev_err(data->dev, "failed to clear fast charger interrupts.\n");
		return IRQ_RETVAL(ret);
	}

	if ((int_sts & FCHG_INT_STS_DETDONE) && !(int_sts0 & FCHG_OUT_OK_BIT))
		dev_warn(data->dev, "get some errors, now status = 0x%x, status0 = 0x%x\n",
			 int_sts, int_sts0);

	if ((int_sts & FCHG_INT_STS_DETDONE) && (int_sts0 & FCHG_OUT_OK_BIT))
		data->detect_ok = true;
	else
		data->detect_ok = false;

	complete(&data->completion);

	return IRQ_HANDLED;
}

static int sprd_2721_fchg_init(u32 vol)
{
	int ret;
	u32 value;

	if (vol < FCHG_VOLTAGE_1)
		value = 0;
	else if (vol < FCHG_VOLTAGE_2)
		value = 1;
	else if (vol < FCHG_VOLTAGE_3)
		value = 2;
	else
		value = 3;

	reinit_completion(&data->completion);
	data->detect_ok = 0;

	ret = regmap_update_bits(data->regmap, ANA_REG_GLB_MODULE_EN0,
				 BIT_FAST_CHG_EN, BIT_FAST_CHG_EN);
	if (ret) {
		dev_err(data->dev, "failed to enable fast charger.\n");
		return ret;
	}

	ret = regmap_update_bits(data->regmap, ANA_REG_GLB_RTC_CLK_EN0,
				 BIT_RTC_FAST_CHG_EN, BIT_RTC_FAST_CHG_EN);
	if (ret) {
		dev_err(data->dev, "failed to enable fast charger clock.\n");
		return ret;
	}

	ret = regmap_update_bits(data->regmap, data->base + FCHG1_TIME1,
				 FCHG_TIME1_MASK, FCHG1_TIME1_VALUE);
	if (ret) {
		dev_err(data->dev, "failed to set fast charge time1.\n");
		return ret;
	}

	ret = regmap_update_bits(data->regmap, data->base + FCHG1_TIME2,
				 FCHG_TIME2_MASK, FCHG1_TIME2_VALUE);
	if (ret) {
		dev_err(data->dev, "failed to set fast charge time2.\n");
		return ret;
	}

	ret = regmap_update_bits(data->regmap, data->base + FCHG_CTRL,
				 FCHG_DET_VOL_MASK << FCHG_DET_VOL_SHIFT,
				 (value & FCHG_DET_VOL_MASK) << FCHG_DET_VOL_SHIFT);
	if (ret) {
		dev_err(data->dev, "failed to set fast charger detect voltage.\n");
		return ret;
	}

	ret = regmap_update_bits(data->regmap, data->base + FCHG_CTRL,
				 FCHG_ENABLE_BIT, FCHG_ENABLE_BIT);
	if (ret) {
		dev_err(data->dev, "failed to enable fast charger.\n");
		return ret;
	}

	ret = regmap_update_bits(data->regmap, data->base + FCHG_INT_EN,
				 FCHG_INT_EN_BIT, FCHG_INT_EN_BIT);
	if (ret) {
		dev_err(data->dev, "failed to enable fast charger irq.\n");
		return ret;
	}

	wait_for_completion(&data->completion);

	return data->detect_ok;
}

static void sprd_2721_fchg_deinit(void)
{
	int ret;

	ret = regmap_update_bits(data->regmap, data->base + FCHG_CTRL,
				 FCHG_ENABLE_BIT, 0);
	if (ret) {
		dev_err(data->dev, "failed to disable fast charger.\n");
		return;
	}

	/*
	 * Adding delay is to write the register fchg_ctrl
	 * value successfully firstly, then write module_en0, rtc_clk_en0.
	 */
	msleep(100);

	ret = regmap_update_bits(data->regmap, ANA_REG_GLB_MODULE_EN0,
				 BIT_FAST_CHG_EN, 0);
	if (ret) {
		dev_err(data->dev, "failed to disable fast charger module.\n");
		return;
	}

	ret = regmap_update_bits(data->regmap, ANA_REG_GLB_RTC_CLK_EN0,
				 BIT_RTC_FAST_CHG_EN, 0);
	if (ret) {
		dev_err(data->dev, "failed to disable charger clock.\n");
		return;
	}
}

static struct sprd_fchg_operations sprd_2721_fchg_ops = {
	.fchg_init = sprd_2721_fchg_init,
	.fchg_deinit = sprd_2721_fchg_deinit,
};

static int sprd_2721_fchg_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!data->regmap) {
		dev_err(&pdev->dev, "failed to get regmap.\n");
		return PTR_ERR(data->regmap);
	}

	data->dev = &pdev->dev;
	data->irq = platform_get_irq(pdev, 0);
	if (data->irq < 0) {
		dev_err(&pdev->dev, "no irq resource specified.\n");
		return data->irq;
	}

	ret = of_property_read_u32(np, "reg", &data->base);
	if (ret) {
		dev_err(&pdev->dev, "failed to get address base.\n");
		return -EINVAL;
	}

	ret = devm_request_threaded_irq(data->dev, data->irq, NULL,
					sprd_fchg_interrupt,
					IRQF_NO_SUSPEND | IRQF_ONESHOT,
					"sprdfchg", NULL);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq.\n");
		return ret;
	}

	init_completion(&data->completion);
	sprdbat_register_fchg_ops(&sprd_2721_fchg_ops);

	return 0;
}

static int sprd_2721_fchg_remove(struct platform_device *pdev)
{
	sprdbat_unregister_fchg_ops();
	return 0;
}

static const struct of_device_id sprd_2721_fchg_of_match[] = {
	{ .compatible = "sprd,sc2721-fchg", },
	{ }
};

static struct platform_driver sprd_2721_fchg_driver = {
	.driver = {
		.name = "sc2721-fchg",
		.of_match_table = sprd_2721_fchg_of_match,
	},
	.probe = sprd_2721_fchg_probe,
	.remove = sprd_2721_fchg_remove
};

module_platform_driver(sprd_2721_fchg_driver);

MODULE_DESCRIPTION("Yuanjiang Yu <yuanjiang.yu@unisoc.com>");
MODULE_AUTHOR("Spreadtrum 2721 fast charge driver>");
MODULE_LICENSE("GPL v2");
