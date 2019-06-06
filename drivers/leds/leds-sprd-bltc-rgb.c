/*
* Copyright (C) 2012 Spreadtrum Communications Inc.
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>

#define PRINT_INFO(x...)  pr_info("[SPRD_BLTC_RGB_INFO]" x)

#define BLTC_CTRL			0x0000
#define BLTC_R_PRESCL			0x0004
#define BLTC_G_PRESCL			0x0014
#define BLTC_B_PRESCL			0x0024
#define BLTC_PRESCL_OFFSET		0x0004
#define BLTC_DUTY_OFFSET		0x0004
#define BLTC_CURVE0_OFFSET		0x0008
#define BLTC_CURVE1_OFFSET		0x000C
#define BLTC_R_ISAT			0x0038
#define BLTC_G_ISAT			0x003c
#define BLTC_B_ISAT			0x0040
#define BLTC_W_ISAT			0x0044
#define BLTC_W_PRESCL			0x0048
#define BLTC_PD_CTRL			0x0058

#define BLTC_STS			0x0034

#define R_RUN	(1 << 0)
#define R_TYPE	(1 << 1)
#define G_RUN	(1 << 4)
#define G_TYPE	(1 << 5)
#define B_RUN	(1 << 8)
#define B_TYPE	(1 << 9)
#define W_RUN	(1 << 12)
#define W_TYPE	(1 << 13)

#define PWM_MOD_COUNTER 0xFF

struct sprd_leds_bltc_rgb {
	struct platform_device *dev;
	struct mutex mutex;
	spinlock_t value_lock;
	enum led_brightness value;
	struct led_classdev cdev;
	u16 leds_flag;
	u16 enable;
	unsigned long bltc_addr, bltc_addr_rf, bltc_addr_hl, bltc_addr_onoff;
	unsigned long sprd_bltc_base_addr, sprd_arm_module_en_addr;
	u8 rising_time;
	u8 falling_time;
	u8 high_time;
	u8 low_time;
	u16 on_off;
	int suspend;
};

enum sprd_leds_type {
	SPRD_LED_TYPE_R = 0,
	SPRD_LED_TYPE_G,
	SPRD_LED_TYPE_B,
#ifdef CONFIG_PMIC_SC2720
/*TODO: This macro definition is not the best method,
should removed in next version*/
	SPRD_LED_TYPE_W,
#endif
	SPRD_LED_TYPE_R_BL,
	SPRD_LED_TYPE_G_BL,
	SPRD_LED_TYPE_B_BL,
#ifdef CONFIG_PMIC_SC2720
	SPRD_LED_TYPE_W_BL,
#endif
	SPRD_LED_TYPE_TOTAL
};

enum sprd_leds_bl_param {
	R_TIME = 1,
	F_TIME,
	H_TIME,
	L_TIME,
	ONOFF
};

static char *sprd_leds_rgb_name[SPRD_LED_TYPE_TOTAL] = {
	"red",
	"green",
	"blue",
#ifdef CONFIG_PMIC_SC2720
	"wled",
#endif
	"red_bl",
	"green_bl",
	"blue_bl",
#ifdef CONFIG_PMIC_SC2720
	"wled_bl",
#endif
};

struct sprd_leds_bltc_rgb_platform_data {
	unsigned long sprd_base_addr;
};

#define to_sprd_bltc_rgb(led_cdev) \
	container_of(led_cdev, struct sprd_leds_bltc_rgb, cdev)

static struct regmap *sprd_rgb_handle;

static inline unsigned int sprd_leds_bltc_rgb_read(unsigned long reg)
{
	unsigned int val;

	regmap_read(sprd_rgb_handle, reg, &val);
	return val;
}

static void sprd_leds_bltc_rgb_set_brightness(struct sprd_leds_bltc_rgb *brgb)
{
	unsigned long brightness = brgb->value;
	unsigned long pwm_duty;

	pwm_duty = brightness;
	if (pwm_duty > PWM_MOD_COUNTER)
		pwm_duty = PWM_MOD_COUNTER;
	regmap_update_bits(sprd_rgb_handle, brgb->bltc_addr, 0xffff,
			(pwm_duty << 8) | PWM_MOD_COUNTER);
}

static void sprd_bltc_rgb_init(struct sprd_leds_bltc_rgb *brgb)
{
#ifdef CONFIG_PMIC_SC2721
	/*ARM_MODULE_EN-enable pclk */
	regmap_update_bits(sprd_rgb_handle,
				ANA_REG_GLB_MODULE_EN0,
				BIT_ANA_BLTC_EN, BIT_ANA_BLTC_EN);
	/*RTC_CLK_EN-enable rtc */
	regmap_update_bits(sprd_rgb_handle,
				ANA_REG_GLB_RTC_CLK_EN0,
				BIT_RTC_BLTC_EN, BIT_RTC_BLTC_EN);
	/*SW POWERDOWN DISABLE */
	regmap_update_bits(sprd_rgb_handle,
				ANA_REG_GLB_RGB_CTRL, BIT_RGB_PD_SW,
				~(unsigned int)(BIT_RGB_PD_SW));
	/*CURRENT CONTROL DEFAULT */
	regmap_update_bits(sprd_rgb_handle,
				ANA_REG_GLB_RGB_CTRL,
				/*BITS_RGB_V(0x1f), ~BITS_RGB_V(0x1f));*/
				BITS_RGB_V(0x1f), BITS_RGB_V(0x0c));	
	/* 2712 not support WHTLED */
#elif defined(CONFIG_PMIC_SC2720)
	/*ARM_MODULE_EN-enable pclk */
	regmap_update_bits(sprd_rgb_handle,
				ANA_REG_GLB_MODULE_EN0,
				BIT_ANA_BLTC_EN, BIT_ANA_BLTC_EN);
	/*RTC_CLK_EN-enable rtc */
	regmap_update_bits(sprd_rgb_handle,
				ANA_REG_GLB_RTC_CLK_EN0,
				BIT_RTC_BLTC_EN, BIT_RTC_BLTC_EN);
	/*SW POWERDOWN DISABLE */
	regmap_update_bits(sprd_rgb_handle,
				ANA_REG_GLB_RGB_CTRL, BIT_RGB_PD_SW,
				~(unsigned int)(BIT_RGB_PD_SW));
	/*CURRENT CONTROL DEFAULT */
	regmap_update_bits(sprd_rgb_handle,
				brgb->sprd_bltc_base_addr + BLTC_R_ISAT,
				0x3f, ~0x3f);
	regmap_update_bits(sprd_rgb_handle,
				brgb->sprd_bltc_base_addr + BLTC_G_ISAT,
				0x3f, ~0x3f);
	regmap_update_bits(sprd_rgb_handle,
				brgb->sprd_bltc_base_addr + BLTC_B_ISAT,
				0x3f, ~0x3f);
	regmap_update_bits(sprd_rgb_handle,
				brgb->sprd_bltc_base_addr + BLTC_W_ISAT,
				0x3f, ~0x3f);
	/* 2720  support WHTLED */
#else
	/*ARM_MODULE_EN-enable pclk */
	regmap_update_bits(sprd_rgb_handle,
			   brgb->sprd_arm_module_en_addr + 0x08,
			   BIT_ANA_BLTC_EN, BIT_ANA_BLTC_EN);
	/*RTC_CLK_EN-enable rtc */
	regmap_update_bits(sprd_rgb_handle,
			   brgb->sprd_arm_module_en_addr + 0x18,
			   BIT_RTC_BLTC_EN, BIT_RTC_BLTC_EN);
	/*SW POWERDOWN DISABLE */
	regmap_update_bits(sprd_rgb_handle,
			   brgb->sprd_arm_module_en_addr + 0x2bc, BIT_RGB_PD_SW,
			   ~(unsigned int)(BIT_RGB_PD_SW));
	/*CURRENT CONTROL DEFAULT */
	regmap_update_bits(sprd_rgb_handle,
			   brgb->sprd_arm_module_en_addr + 0x2bc,
			   BITS_RGB_V(0x1f), ~BITS_RGB_V(0x1f));
	/*WHTLED_SERIES_EN=1 */
	regmap_update_bits(sprd_rgb_handle,
			   brgb->sprd_arm_module_en_addr + 0x2c0,
			   BIT_WHTLED_SERIES_EN, BIT_WHTLED_SERIES_EN);
	/*WHTLED POWERDOWN ENABLE */
	regmap_update_bits(sprd_rgb_handle,
			   brgb->sprd_arm_module_en_addr + 0x2c0, BIT_WHTLED_PD,
			   BIT_WHTLED_PD);
#endif
}

static void sprd_leds_bltc_rgb_enable(struct sprd_leds_bltc_rgb *brgb)
{
	sprd_bltc_rgb_init(brgb);

	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_R]) == 0) {
		regmap_update_bits(sprd_rgb_handle,
				brgb->sprd_bltc_base_addr + BLTC_CTRL,
				R_RUN | R_TYPE, R_RUN | R_TYPE);
		brgb->bltc_addr = brgb->sprd_bltc_base_addr
				+ BLTC_R_PRESCL + BLTC_DUTY_OFFSET;
		sprd_leds_bltc_rgb_set_brightness(brgb);
	}
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_G]) == 0) {
		regmap_update_bits(sprd_rgb_handle,
				brgb->sprd_bltc_base_addr + BLTC_CTRL,
				G_RUN | G_TYPE, G_RUN | G_TYPE);
		brgb->bltc_addr = brgb->sprd_bltc_base_addr
				+ BLTC_G_PRESCL + BLTC_DUTY_OFFSET;
		sprd_leds_bltc_rgb_set_brightness(brgb);
	}
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_B]) == 0) {
		regmap_update_bits(sprd_rgb_handle,
				brgb->sprd_bltc_base_addr + BLTC_CTRL,
				B_RUN | B_TYPE, B_RUN | B_TYPE);
		brgb->bltc_addr = brgb->sprd_bltc_base_addr
				+ BLTC_B_PRESCL + BLTC_DUTY_OFFSET;
		sprd_leds_bltc_rgb_set_brightness(brgb);
	}
#ifdef CONFIG_PMIC_SC2720
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_W]) == 0) {
		regmap_update_bits(sprd_rgb_handle,
				brgb->sprd_bltc_base_addr + BLTC_CTRL,
				W_RUN | W_TYPE, W_RUN | W_TYPE);
		brgb->bltc_addr = brgb->sprd_bltc_base_addr
				+ BLTC_W_PRESCL + BLTC_DUTY_OFFSET;
		sprd_leds_bltc_rgb_set_brightness(brgb);
	}
#endif
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_R_BL]) == 0
			|| strcmp(brgb->cdev.name,
			sprd_leds_rgb_name[SPRD_LED_TYPE_G_BL]) == 0
			|| strcmp(brgb->cdev.name,
			sprd_leds_rgb_name[SPRD_LED_TYPE_B_BL]) == 0
	#ifdef CONFIG_PMIC_SC2720
			|| strcmp(brgb->cdev.name,
			sprd_leds_rgb_name[SPRD_LED_TYPE_W_BL]) == 0
	#endif
	) {
		PRINT_INFO("LEDFLAG:%1d\n", brgb->leds_flag);
		PRINT_INFO("VALUE:%1d\n", brgb->value);

		if (brgb->leds_flag == R_TIME
				|| brgb->leds_flag == F_TIME
				|| brgb->leds_flag == H_TIME
				|| brgb->leds_flag == L_TIME) {
			if (brgb->leds_flag == R_TIME)
				brgb->rising_time = brgb->value;
			if (brgb->leds_flag == F_TIME)
				brgb->falling_time = brgb->value;
			if (brgb->leds_flag == H_TIME)
				brgb->high_time = brgb->value;
			if (brgb->leds_flag == L_TIME)
				brgb->low_time = brgb->value;

			brgb->leds_flag = 0;
		}

		if (strcmp(brgb->cdev.name,
				sprd_leds_rgb_name[SPRD_LED_TYPE_R_BL]) == 0) {
			brgb->bltc_addr_rf = brgb->sprd_bltc_base_addr
					+ BLTC_R_PRESCL + BLTC_CURVE0_OFFSET;
			brgb->bltc_addr_hl = brgb->sprd_bltc_base_addr
					+ BLTC_R_PRESCL + BLTC_CURVE1_OFFSET;
			brgb->bltc_addr_onoff =
				    brgb->sprd_bltc_base_addr + BLTC_CTRL;
			if (brgb->leds_flag == ONOFF)
				brgb->on_off = (brgb->value << 0);
			regmap_update_bits(sprd_rgb_handle,
					brgb->sprd_bltc_base_addr +
					BLTC_CTRL, R_RUN | R_TYPE,
					~(R_RUN | R_TYPE));
		}
		if (strcmp(brgb->cdev.name,
				sprd_leds_rgb_name[SPRD_LED_TYPE_G_BL]) == 0) {
			brgb->bltc_addr_rf = brgb->sprd_bltc_base_addr
					+ BLTC_G_PRESCL + BLTC_CURVE0_OFFSET;
			brgb->bltc_addr_hl = brgb->sprd_bltc_base_addr
					+ BLTC_G_PRESCL + BLTC_CURVE1_OFFSET;
			brgb->bltc_addr_onoff =
					brgb->sprd_bltc_base_addr + BLTC_CTRL;
			if (brgb->leds_flag == ONOFF)
				brgb->on_off = (brgb->value << 4);
			regmap_update_bits(sprd_rgb_handle,
					brgb->sprd_bltc_base_addr +
					BLTC_CTRL, G_RUN | G_TYPE,
					~(G_RUN | G_TYPE));
		}
		if (strcmp(brgb->cdev.name,
				sprd_leds_rgb_name[SPRD_LED_TYPE_B_BL]) == 0) {
			brgb->bltc_addr_rf =
					brgb->sprd_bltc_base_addr
					+ BLTC_B_PRESCL + BLTC_CURVE0_OFFSET;
			brgb->bltc_addr_hl =
					brgb->sprd_bltc_base_addr
					+ BLTC_B_PRESCL + BLTC_CURVE1_OFFSET;
			brgb->bltc_addr_onoff =
					brgb->sprd_bltc_base_addr + BLTC_CTRL;
			if (brgb->leds_flag == ONOFF)
				brgb->on_off = (brgb->value << 8);
			regmap_update_bits(sprd_rgb_handle,
					brgb->sprd_bltc_base_addr +
					BLTC_CTRL, B_RUN | B_TYPE,
					~(B_RUN | B_TYPE));
		}
	#ifdef CONFIG_PMIC_SC2720
		if (strcmp(brgb->cdev.name,
				sprd_leds_rgb_name[SPRD_LED_TYPE_W_BL]) == 0) {
			brgb->bltc_addr_rf = brgb->sprd_bltc_base_addr
					+ BLTC_W_PRESCL + BLTC_CURVE0_OFFSET;
			brgb->bltc_addr_hl = brgb->sprd_bltc_base_addr
					+ BLTC_W_PRESCL + BLTC_CURVE1_OFFSET;
			brgb->bltc_addr_onoff =
					brgb->sprd_bltc_base_addr + BLTC_CTRL;
			if (brgb->leds_flag == ONOFF)
				brgb->on_off = (brgb->value << 12);
			regmap_update_bits(sprd_rgb_handle,
					brgb->sprd_bltc_base_addr +
					BLTC_CTRL, W_RUN | W_TYPE,
					~(W_RUN | W_TYPE));
		}
	#endif
		regmap_update_bits(sprd_rgb_handle, brgb->bltc_addr_rf,
				0XFFFF,
				(brgb->falling_time << 8) | brgb->rising_time);
		regmap_update_bits(sprd_rgb_handle, brgb->bltc_addr_hl,
				0XFFFF,
				(brgb->low_time << 8) | brgb->high_time);
		regmap_update_bits(sprd_rgb_handle, brgb->bltc_addr_onoff,
				brgb->on_off, brgb->on_off);
	}

	PRINT_INFO("sprd_leds_bltc_rgb_enable\n");
	brgb->enable = 1;
}

static void sprd_leds_bltc_rgb_disable(struct sprd_leds_bltc_rgb *brgb)
{
	brgb->on_off = brgb->value;
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_R]) == 0
			|| strcmp(brgb->cdev.name,
			sprd_leds_rgb_name[SPRD_LED_TYPE_R_BL]) == 0) {
		regmap_update_bits(sprd_rgb_handle,
				brgb->sprd_bltc_base_addr + BLTC_CTRL, R_RUN,
				~R_RUN);
	}
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_G]) == 0
			|| strcmp(brgb->cdev.name,
			sprd_leds_rgb_name[SPRD_LED_TYPE_G_BL]) == 0) {
		regmap_update_bits(sprd_rgb_handle,
				brgb->sprd_bltc_base_addr + BLTC_CTRL, G_RUN,
				~G_RUN);
	}
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_B]) == 0
			|| strcmp(brgb->cdev.name,
			sprd_leds_rgb_name[SPRD_LED_TYPE_B_BL]) == 0) {
		regmap_update_bits(sprd_rgb_handle,
				brgb->sprd_bltc_base_addr + BLTC_CTRL, B_RUN,
				~B_RUN);
	}
	#ifdef CONFIG_PMIC_SC2720
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_W]) == 0
			|| strcmp(brgb->cdev.name,
			sprd_leds_rgb_name[SPRD_LED_TYPE_W_BL]) == 0) {
		regmap_update_bits(sprd_rgb_handle,
				brgb->sprd_bltc_base_addr + BLTC_CTRL, W_RUN,
				~W_RUN);
	}
	#endif
	PRINT_INFO("sprd_leds_bltc_rgb_disable\n");
}

static void sprd_leds_rgb_work(struct sprd_leds_bltc_rgb *brgb)
{
	unsigned long flags;

	mutex_lock(&brgb->mutex);
	spin_lock_irqsave(&brgb->value_lock, flags);
	if (brgb->value == LED_OFF) {
		spin_unlock_irqrestore(&brgb->value_lock, flags);
		sprd_leds_bltc_rgb_disable(brgb);
		goto out;
	}
	spin_unlock_irqrestore(&brgb->value_lock, flags);
	sprd_leds_bltc_rgb_enable(brgb);
	PRINT_INFO("sprd_leds_bltc_rgb_work_for rgb!\n");

out:
	mutex_unlock(&brgb->mutex);
}

static void sprd_leds_bltc_work(struct sprd_leds_bltc_rgb *brgb)
{
	unsigned long flags;

	mutex_lock(&brgb->mutex);
	spin_lock_irqsave(&brgb->value_lock, flags);
	if (brgb->value == LED_OFF) {
		spin_unlock_irqrestore(&brgb->value_lock, flags);
		if (brgb->leds_flag == ONOFF) {
			sprd_leds_bltc_rgb_disable(brgb);
			goto out;
		} else
			goto enable;
	}
	spin_unlock_irqrestore(&brgb->value_lock, flags);
enable:
	sprd_leds_bltc_rgb_enable(brgb);
	PRINT_INFO("sprd_leds_bltc_rgb_work for bltc!\n");

out:
	mutex_unlock(&brgb->mutex);
}

static void sprd_leds_bltc_rgb_set(struct led_classdev *bltc_rgb_cdev,
				   enum led_brightness value)
{
	struct sprd_leds_bltc_rgb *brgb;
	unsigned long flags;

	brgb = to_sprd_bltc_rgb(bltc_rgb_cdev);
	spin_lock_irqsave(&brgb->value_lock, flags);
	brgb->leds_flag = bltc_rgb_cdev->flags;
	brgb->value = value;
	spin_unlock_irqrestore(&brgb->value_lock, flags);

	if (1 == brgb->suspend) {
		PRINT_INFO("Do NOT change brightness in suspend mode\n");
		return;
	}
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_R]) == 0
			|| strcmp(brgb->cdev.name,
			sprd_leds_rgb_name[SPRD_LED_TYPE_G]) == 0
			|| strcmp(brgb->cdev.name,
			sprd_leds_rgb_name[SPRD_LED_TYPE_B]) == 0)
		sprd_leds_rgb_work(brgb);
	else
		sprd_leds_bltc_work(brgb);
}

static void sprd_leds_bltc_rgb_shutdown(struct platform_device *dev)
{
	struct sprd_leds_bltc_rgb *brgb = platform_get_drvdata(dev);

	mutex_lock(&brgb->mutex);
	sprd_leds_bltc_rgb_disable(brgb);
	mutex_unlock(&brgb->mutex);
}

static u16 r_value, f_value, h_value, l_value, onoff_value;

static ssize_t show_rising_time(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", r_value);
}

static ssize_t store_rising_time(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long state;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &state);
	PRINT_INFO("r_state_value:%1ld\n", state);
	r_value = state;
	led_cdev->flags = R_TIME;
	sprd_leds_bltc_rgb_set(led_cdev, state);
	return size;
}

static ssize_t show_falling_time(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", f_value);
}

static ssize_t store_falling_time(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long state;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &state);
	PRINT_INFO("f_state_value:%1ld\n", state);
	f_value = state;
	led_cdev->flags = F_TIME;
	sprd_leds_bltc_rgb_set(led_cdev, state);
	return size;
}

static ssize_t show_high_time(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", h_value);
}

static ssize_t store_high_time(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long state;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &state);
	PRINT_INFO("h_state_value:%1ld\n", state);
	h_value = state;
	led_cdev->flags = H_TIME;
	sprd_leds_bltc_rgb_set(led_cdev, state);
	return size;
}

static ssize_t show_low_time(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", l_value);
}

static ssize_t store_low_time(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long state;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &state);
	PRINT_INFO("l_state_value:%1ld\n", state);
	l_value = state;
	led_cdev->flags = L_TIME;
	sprd_leds_bltc_rgb_set(led_cdev, state);
	return size;
}

static ssize_t show_on_off(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", onoff_value);
}

static ssize_t store_on_off(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long state;
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &state);
	PRINT_INFO("onoff_state_value:%1ld\n", state);
	onoff_value = state;
	led_cdev->flags = ONOFF;
	sprd_leds_bltc_rgb_set(led_cdev, state);
	return size;
}

static DEVICE_ATTR(rising_time, 0644, show_rising_time, store_rising_time);
static DEVICE_ATTR(falling_time, 0644, show_falling_time, store_falling_time);
static DEVICE_ATTR(high_time, 0644, show_high_time, store_high_time);
static DEVICE_ATTR(low_time, 0644, show_low_time, store_low_time);
static DEVICE_ATTR(on_off, 0644, show_on_off, store_on_off);

static char *soc_name[] = {
	"sc2713",
	"sc2723",
	"sc2731",
	"sc2721",
	"sc2720",
};

static char *pname;

static struct of_device_id sprd_rgb_bltc_of_match[] = {
	{.compatible = "sprd,sc2723t-bltc-rgb", .data = &soc_name[1],},
	{.compatible = "sprd,sc2731-bltc-rgb", .data = &soc_name[2],},
	{.compatible = "sprd,sc2721-bltc-rgb", .data = &soc_name[3],},
	{.compatible = "sprd,sc2720-bltc-rgb", .data = &soc_name[4],},
	{}
};

static int sprd_leds_bltc_rgb_probe(struct platform_device *dev)
{
	struct sprd_leds_bltc_rgb *brgb;
	const struct of_device_id *of_id;
	struct device_node *node = dev->dev.of_node;
	int ret, i;
	int gpio_prechg_led;
	unsigned int pmic_base_addr;

	gpio_prechg_led = of_get_named_gpio(node, "sprd,prechg-led-gpios", 0);
	if (gpio_is_valid(gpio_prechg_led)) {
		ret = devm_gpio_request_one(&dev->dev, gpio_prechg_led,
				GPIOF_OUT_INIT_HIGH, "prechg_led_gpio");
		if (ret) {
			dev_err(&dev->dev, "gpio%d request failed, ret %d\n",
				gpio_prechg_led, ret);
		}
	}

	sprd_rgb_handle = dev_get_regmap(dev->dev.parent, NULL);
	if (!sprd_rgb_handle)
		panic("%s :NULL spi parent property for vibrator!", __func__);

	ret = of_property_read_u32(node, "reg", &pmic_base_addr);
	if (ret)
		panic("%s :no reg of property for rgb-bltc!", __func__);

	of_id = of_match_node(sprd_rgb_bltc_of_match, node);
	if (of_id)
		pname = (char *)of_id->data;

	for (i = 0; i < SPRD_LED_TYPE_TOTAL; i++) {
		brgb = kzalloc(sizeof(struct sprd_leds_bltc_rgb), GFP_KERNEL);

		if (brgb == NULL) {
			dev_err(&dev->dev, "No memory for bltc_device\n");
			ret = -ENOMEM;
			goto err;
		}

		brgb->cdev.brightness_set = sprd_leds_bltc_rgb_set;
		brgb->cdev.name = sprd_leds_rgb_name[i];
		brgb->cdev.brightness_get = NULL;
		brgb->enable = 0;

		/***0x40038200***/
		/***sharkl2  2721:0x403c8180***/
		brgb->sprd_bltc_base_addr = pmic_base_addr;

		/***0x40038c00***/
		/***sharkl2 2721:0x403c8c00***/
		brgb->sprd_arm_module_en_addr = CTL_BASE_ANA_GLB;

		spin_lock_init(&brgb->value_lock);
		mutex_init(&brgb->mutex);

		brgb->value = LED_OFF;
		platform_set_drvdata(dev, brgb);

		ret = led_classdev_register(&dev->dev, &brgb->cdev);
		if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[i]) == 0) {
			ret = device_create_file(brgb->cdev.dev,
					&dev_attr_rising_time);
			ret = device_create_file(brgb->cdev.dev,
					&dev_attr_falling_time);
			ret = device_create_file(brgb->cdev.dev,
					&dev_attr_high_time);
			ret = device_create_file(brgb->cdev.dev,
					&dev_attr_low_time);
			ret = device_create_file(brgb->cdev.dev,
					&dev_attr_on_off);
		}
		if (ret < 0)
			goto err;

		brgb->value = 15;
		brgb->suspend = 0;
		++brgb;
	}

	/*BLTC SOFT RESET */
	if (strcmp(pname, "sc2723") == 0)
		regmap_update_bits(sprd_rgb_handle,
				brgb->sprd_arm_module_en_addr + 0x0c,
				(0x1 << 13), ~(0x1 << 13));
	else if (strcmp(pname, "sc2731") == 0
			|| strcmp(pname, "sc2721") == 0
			|| strcmp(pname, "sc2720") == 0)
		regmap_update_bits(sprd_rgb_handle,
				ANA_REG_GLB_SOFT_RST0,
				BIT_BLTC_SOFT_RST,
				~(unsigned int)(BIT_BLTC_SOFT_RST));

	return 0;

err:
	if (i) {
		for (i = i - 1; i >= 0; i--) {
			if (!brgb)
				continue;
			led_classdev_unregister(&brgb->cdev);
			kfree(brgb);
			brgb = NULL;
			--brgb;
		}
	}

	return ret;
}

static int sprd_leds_bltc_rgb_remove(struct platform_device *dev)
{
	struct sprd_leds_bltc_rgb *brgb = platform_get_drvdata(dev);

	led_classdev_unregister(&brgb->cdev);
	brgb->value = LED_OFF;
	brgb->enable = 1;
	sprd_leds_bltc_rgb_disable(brgb);
	kfree(brgb);

	return 0;
}

static struct platform_driver sprd_leds_bltc_rgb_driver = {
	.driver = {
		   .name = "sprd-leds-bltc-rgb",
		   .owner = THIS_MODULE,
		   .of_match_table = sprd_rgb_bltc_of_match,
		   },
	.probe = sprd_leds_bltc_rgb_probe,
	.remove = sprd_leds_bltc_rgb_remove,
	.shutdown = sprd_leds_bltc_rgb_shutdown,
};

module_platform_driver(sprd_leds_bltc_rgb_driver);

MODULE_AUTHOR("Xiaotong Lu <xiaotong.lu@spreadtrum.com>");
MODULE_DESCRIPTION("Sprd leds bltc rgb driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sprd_leds_bltc_rgb");
