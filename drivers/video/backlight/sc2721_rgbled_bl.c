 /*
  * linux/drivers/video/backlight/sc2721_rgbled_bl.c
  *
  * simple PWM based backlight control, board code has to setup
  * 1) pin configuration so PWM waveforms can output
  * 2) platform_data being correctly configured
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  */

#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/mfd/sprd/sc2721_glb.h>

#define BLTC_CTRL 0x180

struct sprd_bl_devdata {
	int suspend;
	int brightness_max;
	int brightness_min;
	unsigned int led_level;
	struct clk *clk;
	struct backlight_device *bl;
};

static struct sprd_bl_devdata sprdbl = {
	.brightness_max = 0,
	.brightness_min = 0,
};

static struct regmap *sprd_rgb_backlight_handle;

static void sprd_rgb_led_off(void)
{
	regmap_update_bits(sprd_rgb_backlight_handle,
		ANA_REG_GLB_RGB_CTRL, BIT_RGB_PD_SW | BIT_SLP_RGB_PD_EN,
		BIT_RGB_PD_SW | BIT_SLP_RGB_PD_EN);
	regmap_write(sprd_rgb_backlight_handle,
		BLTC_CTRL, 0x0);
	regmap_update_bits(sprd_rgb_backlight_handle,
		ANA_REG_GLB_RTC_CLK_EN0, BIT_RTC_BLTC_EN, ~BIT_RTC_BLTC_EN);
	regmap_update_bits(sprd_rgb_backlight_handle,
		ANA_REG_GLB_MODULE_EN0, BIT_BLTC_EN, ~BIT_BLTC_EN);
}

static void sprd_rgb_led_set_brightness(u32 level)
{
	if (level < 25)
		level = 0;
	else if (level > 255)
		level = 255;
	else {
		level = (80 * level * 100) / 255 / 3;
		level = (level - 169) / 84;
		level = level << 4;
	}

	regmap_update_bits(sprd_rgb_backlight_handle,
		ANA_REG_GLB_MODULE_EN0, BIT_BLTC_EN, BIT_BLTC_EN);
	regmap_update_bits(sprd_rgb_backlight_handle,
		ANA_REG_GLB_RTC_CLK_EN0, BIT_RTC_BLTC_EN, BIT_RTC_BLTC_EN);
	regmap_write(sprd_rgb_backlight_handle,
		ANA_REG_GLB_RGB_CTRL, level);
	regmap_write(sprd_rgb_backlight_handle,
		BLTC_CTRL, 0xFFF);
}

static int rgbled_backlight_update_status(struct backlight_device *bl)
{
	u32 led_level;

	if ((bl->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
		|| (bl->props.power != FB_BLANK_UNBLANK) ||
		sprdbl.suspend ||
		(bl->props.brightness == 0))
		/* disable backlight */
		sprd_rgb_led_off();
	else {
		led_level = bl->props.brightness & 0xff;
		sprdbl.led_level = led_level;
		sprd_rgb_led_set_brightness(led_level);
	}
	return 0;
}

static int rgbled_backlight_get_brightness(struct backlight_device *bl)
{
	return sprdbl.led_level;
}

static const struct backlight_ops sprd_backlight_lcd_ops = {
	.update_status = rgbled_backlight_update_status,
	.get_brightness = rgbled_backlight_get_brightness,
};

static int rgbled_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct backlight_device *bl;
	const char *backlight_name = "sprd_backlight";

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = 0xff;
	props.type = BACKLIGHT_RAW;
	/*the default brightness = 1/2 max brightness */
	props.brightness = 0xff >> 1;
	props.power = FB_BLANK_UNBLANK;

	bl = backlight_device_register(
			backlight_name, &pdev->dev,
			&sprdbl, &sprd_backlight_lcd_ops, &props);
	if (IS_ERR(bl)) {
		pr_info("Failed to register backlight device\n");
		return -ENODEV;
	}

	sprdbl.bl = bl;
	platform_set_drvdata(pdev, bl);

	sprd_rgb_backlight_handle = dev_get_regmap(pdev->dev.parent, NULL);
	if (!sprd_rgb_backlight_handle) {
		pr_err("Unable to get regmap\n");
		return -EINVAL;
	}

	bl->ops->update_status(bl);

	return 0;
}

static void rgbled_backlight_shutdown(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	bl->props.brightness = 0;

	backlight_update_status(bl);
}

static int rgbled_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_device_unregister(bl);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rgbled_backlight_suspend(struct device *dev)
{
	sprd_rgb_led_off();

	return 0;
}

static int rgbled_backlight_resume(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);

	backlight_update_status(bl);

	return 0;
}
#endif

static const struct dev_pm_ops rgbled_backlight_pm_ops = {
#ifdef CONFIG_PM_SLEEP
	.suspend = rgbled_backlight_suspend,
	.resume = rgbled_backlight_resume,
	.poweroff = rgbled_backlight_suspend,
	.restore = rgbled_backlight_resume,
#endif
};

static const struct of_device_id rgbled_backlight_of_match[] = {
	{.compatible = "sprd,sc2721-bltc-rgb"},
	{ }
};

static struct platform_driver rgbled_backlight_driver = {
	.driver = {
		.name		= "rgbled-backlight",
		.pm		= &rgbled_backlight_pm_ops,
		.of_match_table	= rgbled_backlight_of_match,
	},
	.probe		= rgbled_backlight_probe,
	.remove		= rgbled_backlight_remove,
	.shutdown	= rgbled_backlight_shutdown,
};

module_platform_driver(rgbled_backlight_driver);

MODULE_DESCRIPTION("Spreadtrum PMIC RGBLED backlight Driver");
MODULE_AUTHOR("albert.zhang@spreadtrum.com");
MODULE_LICENSE("GPL");
