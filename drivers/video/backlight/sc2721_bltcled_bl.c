/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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

#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/mfd/sprd/sc2721_glb.h>

/*#define BACKLIGHT_BLTC_DEBUG*/

#ifdef pr_fmt
#undef pr_fmt
#define pr_fmt(__fmt) "[sprd-sc2721-backlight][%20s] "__fmt, __func__
#endif

/*Breathing light control:0xXXXX_X180 ~ 0xXXXX_X19F*/
#define REG_BLTC_BASE		0x180
#define REG_BLTC_R_PRESCL	(REG_BLTC_BASE + 0x004)
#define REG_BLTC_R_DUTY		(REG_BLTC_BASE + 0x008)
#define REG_BLTC_R_CURVE0	(REG_BLTC_BASE + 0x00C)
#define REG_BLTC_R_CURVE1	(REG_BLTC_BASE + 0x010)
#define REG_BLTC_G_PRESCL	(REG_BLTC_BASE + 0x014)
#define REG_BLTC_G_DUTY		(REG_BLTC_BASE + 0x018)
#define REG_BLTC_G_CURVE0	(REG_BLTC_BASE + 0x01C)
#define REG_BLTC_G_CURVE1	(REG_BLTC_BASE + 0x020)
#define REG_BLTC_B_PRESCL	(REG_BLTC_BASE + 0x024)
#define REG_BLTC_B_DUTY		(REG_BLTC_BASE + 0x028)
#define REG_BLTC_B_CURVE0	(REG_BLTC_BASE + 0x02C)
#define REG_BLTC_B_CURVE1	(REG_BLTC_BASE + 0x030)
#define REG_BLTC_STS		(REG_BLTC_BASE + 0x034)


struct sprd_bl_devdata {
	u8 suspend;
	u8 x_duty;
	u8 x_mode;
	u8 output_current;
	u8 brightness_max;
	u8 brightness_min;
	u8 led_level;
	struct backlight_device *bldev;
	struct regmap *bltc_adi;
};

static struct sprd_bl_devdata sprdbl = {
	.suspend = 0,
	.x_duty = 0xFF,
	.x_mode = 0xFE,
	.output_current = 0x1F,
	.brightness_max = 0xFF,
	.brightness_min = 0,
	.led_level = 0x7F,
};


#ifdef BACKLIGHT_BLTC_DEBUG
static void sprd_bltc_whtled_get_brightness(void)
{
	int module_en, rgb_ctrl, bltc_base, rtc_clk_en;
	int r_duty, g_duty, b_duty;

	regmap_read(sprdbl.bltc_adi, ANA_REG_GLB_MODULE_EN0, &module_en);
	regmap_read(sprdbl.bltc_adi, ANA_REG_GLB_RGB_CTRL, &rgb_ctrl);
	regmap_read(sprdbl.bltc_adi, REG_BLTC_BASE, &bltc_base);
	regmap_read(sprdbl.bltc_adi, ANA_REG_GLB_RTC_CLK_EN0, &rtc_clk_en);
	regmap_read(sprdbl.bltc_adi, REG_BLTC_R_DUTY, &r_duty);
	regmap_read(sprdbl.bltc_adi, REG_BLTC_G_DUTY, &g_duty);
	regmap_read(sprdbl.bltc_adi, REG_BLTC_B_DUTY, &b_duty);

	pr_info("GLB_MODULE_EN0 = %x, GLB_RTC_CLK_EN0 = %x\n",
			module_en, rtc_clk_en);
	pr_info("BLTC_BASE = %x, GLB_RGB_CTRL = %x\n",
			bltc_base, rgb_ctrl);
	pr_info("BLTC: R_DUTY = %x, G_DUTY = %x, B_DUTY = %x\n",
			r_duty, g_duty, b_duty);
}
#endif


static void sprd_bltc_whtled_off(void)
{
	int cnt = 3;
	int bltc_base = 0;

	do {
		regmap_write(sprdbl.bltc_adi, REG_BLTC_BASE, 0x0);

		regmap_write(sprdbl.bltc_adi, REG_BLTC_R_DUTY, 0x0);
		regmap_write(sprdbl.bltc_adi, REG_BLTC_G_DUTY, 0x0);
		regmap_write(sprdbl.bltc_adi, REG_BLTC_B_DUTY, 0x0);

		regmap_read(sprdbl.bltc_adi, REG_BLTC_BASE, &bltc_base);
	} while (bltc_base && (--cnt));

	regmap_update_bits(sprdbl.bltc_adi, ANA_REG_GLB_RGB_CTRL,
		BIT_SLP_RGB_PD_EN | BIT_RGB_PD_SW,
		BIT_SLP_RGB_PD_EN | BIT_RGB_PD_SW);

	regmap_update_bits(sprdbl.bltc_adi, ANA_REG_GLB_RTC_CLK_EN0,
		BIT_RTC_BLTC_EN, ~BIT_RTC_BLTC_EN);

	regmap_update_bits(sprdbl.bltc_adi, ANA_REG_GLB_MODULE_EN0,
		BIT_BLTC_EN, ~BIT_BLTC_EN);

#ifdef BACKLIGHT_BLTC_DEBUG
	pr_info("cnt = %d\n", cnt);
	sprd_bltc_whtled_get_brightness();
#endif
}


static void sprd_bltc_whtled_set_brightness(u32 level)
{
	/* rtc_clk = 32KHz, pwm_freq = rtc_clk / x_duty */
	/* duty_rate = x_duty / (x_mode + 1) */
	u32 duty = level * sprdbl.x_duty / 0xFF;

	duty = ((duty ? duty : 1) << 8) | sprdbl.x_mode;

	regmap_update_bits(sprdbl.bltc_adi, ANA_REG_GLB_MODULE_EN0,
		BIT_BLTC_EN, BIT_BLTC_EN);

	regmap_update_bits(sprdbl.bltc_adi, ANA_REG_GLB_RTC_CLK_EN0,
		BIT_RTC_BLTC_EN, BIT_RTC_BLTC_EN);

	regmap_update_bits(sprdbl.bltc_adi, ANA_REG_GLB_RGB_CTRL,
		BIT_SLP_RGB_PD_EN | BIT_RGB_PD_SW,
		~(BIT_SLP_RGB_PD_EN | BIT_RGB_PD_SW));

	regmap_write(sprdbl.bltc_adi, REG_BLTC_BASE, 0x333);

	/*SET BLTC DUTY(duty_counter + mod_counter)*/
	regmap_write(sprdbl.bltc_adi, REG_BLTC_R_DUTY, duty);
	regmap_write(sprdbl.bltc_adi, REG_BLTC_G_DUTY, duty);
	regmap_write(sprdbl.bltc_adi, REG_BLTC_B_DUTY, duty);

#ifdef BACKLIGHT_BLTC_DEBUG
	sprd_bltc_whtled_get_brightness();
#endif
}


static int sprd_bl_whtled_update_status(struct backlight_device *bldev)
{
	if ((bldev->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK)) ||
		bldev->props.power != FB_BLANK_UNBLANK || sprdbl.suspend ||
		bldev->props.brightness == 0) {
		/* disable backlight */
		sprd_bltc_whtled_off();
		pr_info("disabled\n");
	} else {
		sprdbl.led_level = bldev->props.brightness;
		sprd_bltc_whtled_set_brightness(sprdbl.led_level);
		pr_info("brightness = %d\n", bldev->props.brightness);
	}

	return 0;
}


static int sprd_bl_whtled_get_brightness(struct backlight_device *bldev)
{
	return sprdbl.led_level;
}


static const struct backlight_ops sprd_backlight_whtled_ops = {
	.update_status = sprd_bl_whtled_update_status,
	.get_brightness = sprd_bl_whtled_get_brightness,
};


static void whtled_bltc_parse_dt(struct device_node *np)
{
	u32 get_of;

	if (!of_property_read_u32(np, "sprd,bltc-duty", &get_of)) {
		sprdbl.x_duty = (u8)get_of;
		sprdbl.x_mode = sprdbl.x_duty - 1;
		pr_info("bltc-duty = %d\n", sprdbl.x_duty);
	} else
		pr_err("Failed to get bltc-duty in dts\n");

	if (!of_property_read_u32(np, "sprd,bltc-current", &get_of)) {
		sprdbl.output_current = (u8)get_of;
		pr_info("bltc-current = 0x%x\n", sprdbl.output_current);
	} else
		pr_err("Failed to get output_current in dts\n");
}


static int whtled_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct backlight_device *bldev;
	const char *backlight_name = "sprd_backlight";

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = sprdbl.brightness_max;
	props.type = BACKLIGHT_RAW;
	props.brightness = sprdbl.led_level;
	props.power = FB_BLANK_UNBLANK;

	bldev = backlight_device_register(
		backlight_name, &pdev->dev,
		&sprdbl, &sprd_backlight_whtled_ops, &props);
	if (IS_ERR(bldev)) {
		pr_err("Failed to register backlight device\n");
		return -ENOMEM;
	}

	sprdbl.bldev = bldev;
	whtled_bltc_parse_dt(pdev->dev.of_node);

	platform_set_drvdata(pdev, bldev);

	sprdbl.bltc_adi = dev_get_regmap(pdev->dev.parent, NULL);
	if (!sprdbl.bltc_adi) {
		pr_err("Unable to get regmap\n");
		return -EINVAL;
	}

	/*SET BLTC prescale coefficient, no prescl(0 -> 1rtc_clk)*/
	regmap_write(sprdbl.bltc_adi, REG_BLTC_R_PRESCL, 0x0);
	regmap_write(sprdbl.bltc_adi, REG_BLTC_G_PRESCL, 0x0);
	regmap_write(sprdbl.bltc_adi, REG_BLTC_B_PRESCL, 0x0);

	/*SET BLTC Output RISE/FALL Time*/
	regmap_write(sprdbl.bltc_adi, REG_BLTC_R_CURVE0, 0x0);
	regmap_write(sprdbl.bltc_adi, REG_BLTC_G_CURVE0, 0x0);
	regmap_write(sprdbl.bltc_adi, REG_BLTC_B_CURVE0, 0x0);

	/*SET BLTC Output HIGH/LOW Time*/
	regmap_write(sprdbl.bltc_adi, REG_BLTC_R_CURVE1, 0x0);
	regmap_write(sprdbl.bltc_adi, REG_BLTC_G_CURVE1, 0x0);
	regmap_write(sprdbl.bltc_adi, REG_BLTC_B_CURVE1, 0x0);

	/*USE MAX CURRENT 85.71mA= {(1.69+0.84*32)*3}*/
	/*AS THE backlight DEFAULT OUTPUT CURRENT*/
	regmap_update_bits(sprdbl.bltc_adi, ANA_REG_GLB_RGB_CTRL,
		BITS_RGB_V(sprdbl.output_current),
		BITS_RGB_V(sprdbl.output_current));

	bldev->ops->update_status(bldev);

	pr_info("bltc backlight probe success!\n");

	return 0;
}


static int whtled_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bldev;

	bldev = platform_get_drvdata(pdev);

	bldev->props.power = FB_BLANK_UNBLANK;
	bldev->props.brightness = 0;

	backlight_update_status(bldev);

	backlight_device_unregister(bldev);

	platform_set_drvdata(pdev, NULL);

	return 0;
}


static void whtled_backlight_shutdown(struct platform_device *pdev)
{
	struct backlight_device *bldev;

	bldev = platform_get_drvdata(pdev);
	bldev->props.brightness = 0;

	backlight_update_status(bldev);
}


#ifdef CONFIG_PM_SLEEP
static int whtled_backlight_suspend(struct device *pdev)
{
	sprd_bltc_whtled_off();

	return 0;
}

static int whtled_backlight_resume(struct device *pdev)
{
	struct backlight_device *bldev;

	bldev = dev_get_drvdata(pdev);

	backlight_update_status(bldev);
	return 0;
}
#endif

static const struct dev_pm_ops whtled_backlight_pm_ops = {
#ifdef CONFIG_PM_SLEEP
	.suspend = whtled_backlight_suspend,
	.resume = whtled_backlight_resume,
	.poweroff = whtled_backlight_suspend,
	.restore = whtled_backlight_resume,
#endif
};

static const struct of_device_id whtled_of_match[] = {
	{ .compatible = "sprd,sc2721-bltc-rgb", },
	{ }
};

static struct platform_driver whtled_backlight_driver = {
	.probe = whtled_backlight_probe,
	.remove = whtled_backlight_remove,
	.shutdown = whtled_backlight_shutdown,
	.driver = {
		.name = "bltc-whtled-backlight",
		.pm = &whtled_backlight_pm_ops,
		.of_match_table = whtled_of_match,
	},
};


module_platform_driver(whtled_backlight_driver);

MODULE_DESCRIPTION("Spreadtrum backlight Driver");
MODULE_AUTHOR("dong1.wang@spreadtrum.com");
MODULE_LICENSE("GPL");
