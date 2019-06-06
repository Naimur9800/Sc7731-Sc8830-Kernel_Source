/*
 *  Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * Author: Xiaodong Bi <xiaodong.bi@spreadtrum.com>
 *
 * License terms: GNU General Public License (GPL) version 2
 *
 *  sdio MMC power sequence management
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mmc/host.h>
#include <linux/regulator/consumer.h>

#include "pwrseq.h"

#define PWRSEQ_SDIO_VDDWCN_VOLTAGE 1500000
#define PWRSEQ_SDIO_VDDCORE_VOLTAGE 1200000

struct mmc_pwrseq_sdio {
	struct mmc_pwrseq pwrseq;
	struct regulator *vdd_wcn;
	struct regulator *vdd_core;
	int chip_en;
	int reset;
	struct clk *clk_32k;
	struct clk *clk_parent;
	struct clk *clk_enable;
	bool clk_enabled;

	struct mutex power_lock;
	struct mutex power_1v2_lock;
	struct mutex clk_lock;
	struct mutex chip_en_lock;
	struct mutex reset_lock;
	u32 post_power_on_delay_ms;
};

#define to_pwrseq_sdio(p) container_of(p, struct mmc_pwrseq_sdio, pwrseq)

static int mmc_pwrseq_sdio_power_enable(struct mmc_pwrseq_sdio *pwrseq,
					bool enable)
{
	int ret = 0;

	pr_info("pwrseq: %s, enable:%d\n", __func__, enable);
	if (IS_ERR(pwrseq->vdd_wcn)) {
		WARN_ON(1);
		return 0;
	}

	mutex_lock(&pwrseq->power_lock);
	if (enable) {
		regulator_set_voltage(pwrseq->vdd_wcn,
				      PWRSEQ_SDIO_VDDWCN_VOLTAGE,
				      PWRSEQ_SDIO_VDDWCN_VOLTAGE);
		ret = regulator_enable(pwrseq->vdd_wcn);
	} else {
		if (regulator_is_enabled(pwrseq->vdd_wcn))
			ret = regulator_disable(pwrseq->vdd_wcn);
	}
	mutex_unlock(&pwrseq->power_lock);

	return ret;
}
static int mmc_pwrseq_sdio_power_1v2_enable(struct mmc_pwrseq_sdio *pwrseq,
					    bool enable)
{
	int ret = 0;

	pr_info("pwrseq: %s, enable:%d\n", __func__, enable);
	if (IS_ERR(pwrseq->vdd_core)) {
		WARN_ON(1);
		return 0;
	}

	mutex_lock(&pwrseq->power_1v2_lock);
	if (enable) {
		regulator_set_voltage(pwrseq->vdd_core,
				      PWRSEQ_SDIO_VDDCORE_VOLTAGE,
				      PWRSEQ_SDIO_VDDCORE_VOLTAGE);
		ret = regulator_enable(pwrseq->vdd_core);
	} else {
		if (regulator_is_enabled(pwrseq->vdd_core))
			ret = regulator_disable(pwrseq->vdd_core);
	}
	mutex_unlock(&pwrseq->power_1v2_lock);

	return ret;
}


static int mmc_pwrseq_sdio_clk_enable(struct mmc_pwrseq_sdio *pwrseq,
				      bool enable)
{
	pr_info("pwrseq: %s, enable:%d\n", __func__, enable);
	if (IS_ERR(pwrseq->clk_32k) || IS_ERR(pwrseq->clk_enable)) {
		WARN_ON(1);
		return 0;
	}

	mutex_lock(&pwrseq->clk_lock);
	if (enable && !pwrseq->clk_enabled) {
		clk_prepare_enable(pwrseq->clk_32k);
		clk_prepare_enable(pwrseq->clk_enable);
		pwrseq->clk_enabled = true;
	} else if (!enable && pwrseq->clk_enabled) {
		clk_disable_unprepare(pwrseq->clk_enable);
		clk_disable_unprepare(pwrseq->clk_32k);
		pwrseq->clk_enabled = false;
	}
	mutex_unlock(&pwrseq->clk_lock);

	return 0;
}

static void mmc_pwrseq_sdio_chip_en(struct mmc_pwrseq_sdio *pwrseq,
			     bool enable)
{
	mutex_lock(&pwrseq->chip_en_lock);
	if (gpio_is_valid(pwrseq->chip_en)) {
		if (enable)
			gpio_direction_output(pwrseq->chip_en, 1);
		else
			gpio_direction_output(pwrseq->chip_en, 0);

		pr_info("pwrseq: %s, enable:%d\n", __func__, enable);
	}
	mutex_unlock(&pwrseq->chip_en_lock);
}

static void mmc_pwrseq_sdio_reset(struct mmc_pwrseq_sdio *pwrseq,
			   bool enable)
{
	mutex_lock(&pwrseq->reset_lock);
	if (gpio_is_valid(pwrseq->reset)) {
		if (enable)
			gpio_direction_output(pwrseq->reset, 1);
		else
			gpio_direction_output(pwrseq->reset, 0);

		pr_info("pwrseq: %s, enable:%d\n", __func__, enable);
	}
	mutex_unlock(&pwrseq->reset_lock);
}

static void mmc_pwrseq_sdio_lock_init(struct mmc_pwrseq_sdio *pwrseq)
{
	mutex_init(&pwrseq->power_lock);
	mutex_init(&pwrseq->power_1v2_lock);
	mutex_init(&pwrseq->clk_lock);
	mutex_init(&pwrseq->chip_en_lock);
	mutex_init(&pwrseq->reset_lock);
}

static int mmc_pwrseq_sdio_parse_dt(struct device_node *of_node,
				    struct mmc_pwrseq_sdio *pwrseq)
{
	struct device_node *np;
	struct platform_device *sub_pdev;
	struct device *sub_dev;
	int ret;
	char str_err[64] = {0};

	np = of_parse_phandle(of_node, "pwrseq-wcn", 0);
	if (!np) {
		memcpy(str_err, "no pwrseq-wcn node",
		       sizeof("no pwrseq-wcn node"));
		goto err;
	}

	if (of_get_property(np, "pwrseq_disable", NULL)) {
		pr_info("pwrseq: %s sdio pwrseq disabled\n", __func__);
		return 1;
	}

	sub_pdev = of_find_device_by_node(np);
	if (!sub_pdev) {
		memcpy(str_err, "no sub node", sizeof("no sub node"));
		goto err;
	}

	sub_dev = &sub_pdev->dev;

	pwrseq->vdd_wcn = devm_regulator_get(sub_dev, "vddwcn");
	if (IS_ERR(pwrseq->vdd_wcn)) {
		memcpy(str_err, "dt no vdd_wcn!", sizeof("dt no vdd_wcn!"));
		goto err;
	}

	pwrseq->vdd_core = devm_regulator_get(sub_dev, "vdd_marlin2_1v2");
	if (IS_ERR(pwrseq->vdd_core)) {
		memcpy(str_err, "dt no vdd_core!", sizeof("dt no vdd_core!"));
		goto err;
	}

	pwrseq->clk_32k = devm_clk_get(sub_dev, "clk_32k");
	if (IS_ERR(pwrseq->clk_32k)) {
		memcpy(str_err, "dt no clk_32k!", sizeof("dt no clk_32k!"));
		goto err;
	}

	pwrseq->clk_parent = devm_clk_get(sub_dev, "source");
	if (IS_ERR(pwrseq->clk_parent)) {
		memcpy(str_err, "dt no 32k source!",
		       sizeof("dt no 32k source!"));
		goto err;
	}

	clk_set_parent(pwrseq->clk_32k, pwrseq->clk_parent);

	pwrseq->clk_enable = devm_clk_get(sub_dev, "enable");
	if (IS_ERR(pwrseq->clk_enable)) {
		pr_err("pwrseq: parse dt no 32k enable!\n");
		memcpy(str_err, "dt no 32k enable!",
		       sizeof("dt no 32k enable!"));
		goto err;
	}

	pwrseq->chip_en = of_get_named_gpio(np, "chip-en-gpios", 0);
	if (!gpio_is_valid(pwrseq->chip_en))
		pr_info("pwrseq: parse dt no chip_en!\n");

	pwrseq->reset = of_get_named_gpio(np, "rstn-gpios", 0);
	if (!gpio_is_valid(pwrseq->reset))
		pr_info("pwrseq: parse dt no reset!\n");

	ret = gpio_request(pwrseq->reset, "reset");
	if (ret)
		pr_err("pwrseq: gpio reset request err\n");

	ret = gpio_request(pwrseq->chip_en, "chip_en");
	if (ret)
		pr_err("pwrseq: gpio chip_en request err\n");

	ret = of_property_read_u32(np, "post-power-on-delay-ms",
				 &pwrseq->post_power_on_delay_ms);

	return 0;
err:
	pr_err("pwrseq: %s %s\n", __func__, str_err);

	return -1;
}

static void mmc_pwrseq_sdio_pre_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq_sdio *pwrseq = to_pwrseq_sdio(host->pwrseq);

	pr_info("pwrseq: %s entry\n", __func__);

	mmc_pwrseq_sdio_clk_enable(pwrseq, true);

	mmc_pwrseq_sdio_reset(pwrseq, false);
}

static void mmc_pwrseq_sdio_post_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq_sdio *pwrseq = to_pwrseq_sdio(host->pwrseq);

	pr_info("pwrseq: %s entry\n", __func__);

	mmc_pwrseq_sdio_power_enable(pwrseq, true);
	mmc_pwrseq_sdio_power_1v2_enable(pwrseq, true);
	mmc_pwrseq_sdio_chip_en(pwrseq, true);
	mmc_pwrseq_sdio_reset(pwrseq, true);
	if (pwrseq->post_power_on_delay_ms)
		msleep(pwrseq->post_power_on_delay_ms);
}

static void mmc_pwrseq_sdio_power_off(struct mmc_host *host)
{
	struct mmc_pwrseq_sdio *pwrseq = to_pwrseq_sdio(host->pwrseq);

	pr_info("pwrseq: %s entry\n", __func__);

	mmc_pwrseq_sdio_reset(pwrseq, false);
	mmc_pwrseq_sdio_chip_en(pwrseq, false);
	mmc_pwrseq_sdio_power_1v2_enable(pwrseq, false);
	mmc_pwrseq_sdio_power_enable(pwrseq, false);
	mmc_pwrseq_sdio_clk_enable(pwrseq, false);
}

static const struct mmc_pwrseq_ops mmc_pwrseq_sdio_ops = {
	.pre_power_on = mmc_pwrseq_sdio_pre_power_on,
	.post_power_on = mmc_pwrseq_sdio_post_power_on,
	.power_off = mmc_pwrseq_sdio_power_off,
};

static const struct of_device_id mmc_pwrseq_sdio_of_match[] = {
	{ .compatible = "mmc-pwrseq-sprd-sdio",},
};
MODULE_DEVICE_TABLE(of, mmc_pwrseq_sdio_of_match);

static int mmc_pwrseq_sdio_probe(struct platform_device *pdev)
{
	struct mmc_pwrseq_sdio *pwrseq;
	struct device *dev = &pdev->dev;

	pwrseq = devm_kzalloc(dev, sizeof(*pwrseq), GFP_KERNEL);
	if (!pwrseq)
		return -ENOMEM;

	if (mmc_pwrseq_sdio_parse_dt(dev->of_node, pwrseq)) {
		pr_info("pwrseq:%s parse_dt some para not config\n", __func__);
		devm_kfree(dev, pwrseq);
		return -ENODEV;
	}

	mmc_pwrseq_sdio_lock_init(pwrseq);

	pwrseq->pwrseq.dev = dev;
	pwrseq->pwrseq.ops = &mmc_pwrseq_sdio_ops;
	pwrseq->pwrseq.owner = THIS_MODULE;
	platform_set_drvdata(pdev, pwrseq);

	return mmc_pwrseq_register(&pwrseq->pwrseq);
}

static int mmc_pwrseq_sdio_remove(struct platform_device *pdev)
{
	struct mmc_pwrseq_sdio *pwrseq = platform_get_drvdata(pdev);

	mmc_pwrseq_unregister(&pwrseq->pwrseq);

	return 0;
}

static struct platform_driver mmc_pwrseq_sdio_driver = {
	.probe = mmc_pwrseq_sdio_probe,
	.remove = mmc_pwrseq_sdio_remove,
	.driver = {
		.name = "pwrseq_sdio",
		.of_match_table = mmc_pwrseq_sdio_of_match,
	},
};

module_platform_driver(mmc_pwrseq_sdio_driver);
MODULE_LICENSE("GPL v2");
