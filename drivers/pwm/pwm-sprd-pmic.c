/*
 * Spreadtrum Pulse Width Modulator PMIC driver
 *
 * Copyright (C) 2016 Spreadtrum Inc.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */



#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/regmap.h>

#define NUM_PWM		4

#define PWM_PMIC_PRESCALE	(pwm_data.offset + 0x00)
#define PWM_PMIC_CNT		(pwm_data.offset + 0x04)
#define PWM_PMIC_TONE_DIV	(pwm_data.offset + 0x08)
#define PWM_PMIC_PAT_LOW	(pwm_data.offset + 0x0C)
#define PWM_PMIC_PAT_HIGH	(pwm_data.offset + 0x10)

#define PWM_PMIC_EN			0x0100
#define PWM_PMIC_PAT_LOW_MASK		0xFFFF
#define PWM_PMIC_PAT_HIGH_MASK		0xFFFF
#define PWM_PMIC_REG_MSK		0xFFFF
#define PWM_PMIC_MOD_MAX		0xFF

struct pwm_pmic_data {
	struct regmap *pwm_regmap;
	unsigned int offset;
};

static struct pwm_pmic_data pwm_data;

static int pwm_pmic_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	pr_debug("%s Enter\n", __func__);
	regmap_update_bits(pwm_data.pwm_regmap, PWM_PMIC_PRESCALE, PWM_PMIC_EN,
				PWM_PMIC_EN);
	return 0;
}

static void pwm_pmic_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	pr_debug("%s Enter\n", __func__);
	regmap_update_bits(pwm_data.pwm_regmap, PWM_PMIC_PRESCALE, PWM_PMIC_EN,
				~PWM_PMIC_EN);
}

static int pwm_pmic_config(struct pwm_chip *chip, struct pwm_device *pwm,
						int duty_ns, int period_ns)
{
	int level;

	level = duty_ns * PWM_PMIC_MOD_MAX + period_ns / 2;
	do_div(level, period_ns);

	pr_debug("%s: duty_ns = %d,period_ns = %d,level = %d\n",
			__func__, duty_ns, period_ns, level);

	regmap_update_bits(pwm_data.pwm_regmap, PWM_PMIC_PAT_HIGH,
			PWM_PMIC_PAT_HIGH_MASK, PWM_PMIC_PAT_HIGH_MASK);

	regmap_update_bits(pwm_data.pwm_regmap, PWM_PMIC_PAT_LOW,
			PWM_PMIC_PAT_LOW_MASK, PWM_PMIC_PAT_LOW_MASK);
	regmap_update_bits(pwm_data.pwm_regmap, PWM_PMIC_CNT,
			PWM_PMIC_REG_MSK, level << 8 | PWM_PMIC_MOD_MAX);

	return 0;
}
static const struct pwm_ops pwm_pmic_ops = {
	.enable = pwm_pmic_enable,
	.disable = pwm_pmic_disable,
	.config = pwm_pmic_config,
	.owner = THIS_MODULE,
};

static int pwm_pmic_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct pwm_chip *chip;
	struct device_node *node = pdev->dev.of_node;

	pr_debug("%s Entering\n", __func__);
	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	pwm_data.pwm_regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (IS_ERR(pwm_data.pwm_regmap)) {
		dev_err(&pdev->dev, "%s :dev_get_regmap failed\n", __func__);
		if (chip != NULL)
			devm_kfree(&pdev->dev, chip);
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "reg", &pwm_data.offset);
	if (ret) {
		dev_err(&pdev->dev, "%s :no property of reg\n", __func__);
		if (chip != NULL)
			devm_kfree(&pdev->dev, chip);
		return -ENXIO;
	}

	platform_set_drvdata(pdev, chip);

	chip->dev = &pdev->dev;
	chip->ops = &pwm_pmic_ops;
	chip->base = -1;
	chip->npwm = NUM_PWM;

	ret = pwmchip_add(chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
		if (chip != NULL)
			devm_kfree(&pdev->dev, chip);
	}

	return ret;
}

static int pwm_pmic_remove(struct platform_device *pdev)
{
	struct pwm_chip *chip = platform_get_drvdata(pdev);

	return pwmchip_remove(chip);
}

static const struct of_device_id pwm_pmic_matches[] = {
	{ .compatible = "sprd,sc2723-pwm" },
	{},
};

static struct platform_driver pwm_pmic_driver = {
	.driver = {
		.name = "pwm-pmic",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pwm_pmic_matches),
	},
	.probe = pwm_pmic_probe,
	.remove = pwm_pmic_remove,
};

module_platform_driver(pwm_pmic_driver);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-pmic");
