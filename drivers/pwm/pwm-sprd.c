/*
 * Spreadtrum Pulse Width Modulator driver
 *
 * Copyright (C) 2016 Spreadtrum Inc.
 * infi.chen <infi.chen@spreadtrum.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "pwm-sprd.h"

#define		NUM_PWM			4
#define		NANOSECOND		1000000000
#define		PWM_CLK_PARENT		"sprd_pwm_clk_parent"
#define		PWM_CLK			"clk_pwm"

struct pwm_core_ops *sprd_core_ops;
extern struct pwm_core_ops pwm_r2p0_ops;
extern struct pwm_core_ops pwm_r3p0_ops;

static int sprd_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
				int duty_ns, int period_ns)
{
	struct sprd_pwm_chip *pc = to_sprd_pwm_chip(chip);
	static int config_state;
	static int pwm_scale;
	int led_level;
	u64 clk_rate, div, val;

	led_level = duty_ns * PWM_MOD_MAX / period_ns;
	pr_debug("duty_ns = %d,period_ns = %d,led_level = %d\n",
		duty_ns, period_ns, led_level);

	if (0 == config_state) {
		clk_rate = clk_get_rate(pc->clk);
		/*
		 * Find pv, dc and prescale to suit duty_ns and period_ns.
		 * This is done according to formulas described below:
		 *
		 * period_ns = 10^9 * (PRESCALE + 1) * PV / PWM_CLK_RATE
		 * duty_ns = 10^9 * (PRESCALE + 1) * DC / PWM_CLK_RATE
		 *
		 * PV = (PWM_CLK_RATE * period_ns) / (10^9 * (PRESCALE + 1))
		 * DC = (PWM_CLK_RATE * duty_ns) / (10^9 * (PRESCALE + 1))
		 */
		div = 1000000000;
		div = div * PWM_MOD_MAX;
		val = clk_rate * period_ns;
		pwm_scale = div64_u64(val, div) - 1;
		config_state = 1;
	}

	sprd_core_ops->pwm_early_config(chip, pwm, true);
	sprd_core_ops->pwm_config(chip, pwm, led_level, pwm_scale);

	if (led_level == 0)
		sprd_core_ops->pwm_early_config(chip, pwm, false);

	return 0;
}
static int sprd_pwm_clk_enable(struct sprd_pwm_chip *chip, int enable)
{
	static int current_state;
	int rc = 0;

	if (1 == enable) {
		if (0 == current_state) {
			rc = clk_enable(chip->clk);
			if (rc)
				return rc;
			current_state = 1;
		}
	} else {
		if (1 == current_state) {
			clk_disable(chip->clk);
			current_state = 0;
		}
	}
	return rc;
}
static int sprd_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct sprd_pwm_chip *pc = to_sprd_pwm_chip(chip);
	int rc = 0;

	rc = sprd_pwm_clk_enable(pc, 1);
	if (rc)
		return rc;

	return 0;
}

static void sprd_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct sprd_pwm_chip *pc = to_sprd_pwm_chip(chip);

	sprd_pwm_clk_enable(pc, 0);
}
#ifdef CONFIG_PM_SLEEP
static int pwm_sprd_suspend(struct device *dev)
{
	struct sprd_pwm_chip *pc = dev_get_drvdata(dev);

	sprd_pwm_clk_enable(pc, 0);

	return 0;
}

static int pwm_sprd_resume(struct device *dev)
{
	struct sprd_pwm_chip *pc = dev_get_drvdata(dev);
	int rc = 0;

	rc = sprd_pwm_clk_enable(pc, 1);
	if (rc)
		return rc;

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_sprd_pm_ops, pwm_sprd_suspend,
			 pwm_sprd_resume);
static const struct pwm_ops sprd_pwm_ops = {
	.config = sprd_pwm_config,
	.enable = sprd_pwm_enable,
	.disable = sprd_pwm_disable,
	.owner = THIS_MODULE,
};

static const struct of_device_id sprd_pwm_matches[] = {
	{	.compatible = "sprd,pwm",
		.data = &pwm_r2p0_ops,
	},
	{	.compatible = "sprd,pwm-r3p0",
		.data = &pwm_r3p0_ops,
	},
	{},
};

static int sprd_pwm_probe(struct platform_device *pdev)
{
	struct sprd_pwm_chip *pc;
	struct resource *r;
	struct clk *clk_parent;
	const struct of_device_id *of_id;
	int ret;

	pr_debug("%s:enter\n", __func__);

	of_id = of_match_node(sprd_pwm_matches, pdev->dev.of_node);
	if (of_id)
		sprd_core_ops = (struct pwm_core_ops *)of_id->data;
	else
		pr_err("%s: Not found match of_id\n", __func__);

	pc = devm_kzalloc(&pdev->dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pc->mmio_base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(pc->mmio_base))
		return PTR_ERR(pc->mmio_base);

	clk_parent = devm_clk_get(&pdev->dev, PWM_CLK_PARENT);
	if (IS_ERR(clk_parent))
		return PTR_ERR(clk_parent);

	pc->clk = devm_clk_get(&pdev->dev, PWM_CLK);
	if (IS_ERR(pc->clk))
		return PTR_ERR(pc->clk);

	clk_set_parent(pc->clk, clk_parent);

	platform_set_drvdata(pdev, pc);

	pc->syscon = syscon_regmap_lookup_by_compatible("sprd,sys-aon-apb");
	if (IS_ERR(pc->syscon)) {
		dev_err(&pdev->dev, "pwm syscon get failed\n");
		return PTR_ERR(pc->syscon);
	}
	pc->chip.dev = &pdev->dev;
	pc->chip.ops = &sprd_pwm_ops;
	pc->chip.base = -1;
	pc->chip.npwm = NUM_PWM;

	ret = clk_prepare(pc->clk);
	if (ret)
		return ret;

	ret = pwmchip_add(&pc->chip);
	if (ret < 0) {
		clk_unprepare(pc->clk);
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
	}

	return ret;
}

static int sprd_pwm_remove(struct platform_device *pdev)
{
	struct sprd_pwm_chip *pc = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < NUM_PWM; i++)
		pwm_disable(&pc->chip.pwms[i]);

	/* clk was prepared in probe, hence unprepare it here */
	clk_unprepare(pc->clk);
	return pwmchip_remove(&pc->chip);
}

static struct platform_driver sprd_pwm_driver = {
	.driver = {
		.name = "sprd-pwm",
		.owner = THIS_MODULE,
		.pm = &pwm_sprd_pm_ops,
		.of_match_table = of_match_ptr(sprd_pwm_matches),
	},
	.probe = sprd_pwm_probe,
	.remove = sprd_pwm_remove,
};

module_platform_driver(sprd_pwm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Billy Zhang <billy.zhang@spreadtrum.com>");
MODULE_ALIAS("platform:sprd-pwm");
