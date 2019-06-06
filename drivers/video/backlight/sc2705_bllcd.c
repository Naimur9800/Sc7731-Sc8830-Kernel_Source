/*
 * LCD pannel and backlight device driver for SC2705
 * Copyright (c) 2017 Dialog Semiconductor.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/lcd.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include "sc2705_bllcd.h"

#define SC2705_IDAC_MAX_BRIGHTNESS	0x7FF
#define SC2705_PWM_MAX_BRIGHTNESS		0x7F
#define SC2705_WLED_NAME	"sprd_backlight"
#define SC2705_LCD_NAME	"sc2705_lcd"
#define SC2705_DISP_REG_MAX	0xFF

/* Default pwm freq to 20kHz and Max 33kHz */
#define SC2705_WLED_FREQ_DEF	20480
#define SC2705_WLED_FREQ_MIN	3072
#define SC2705_WLED_FREQ_MAX	33792
#define SC2705_WLED_PWM_DUTY_MAX 128 /* 100% */
#define SC2705_WLED_PWM_DUTY_DEF (128/2) /* 50% */
#define SC2705_WLED_IDAC_MAX 2047 /* 100% - 20mA */
#define SC2705_WLED_IDAC_DEF 0
#define SC2705_PWM_DUTY_THRESH_MAX 255
#define SC2705_PWM_DUTY_THRESH_DEF 50
#define SC2705_PWM_BRIGHTNESS_MAX 0xFFFF
#define SC2705_PWM_BRIGHTNESS_DEF 100

#define LCD_IRQ_NUM	5

#define POWER_IS_ON(pwr)	((pwr) == FB_BLANK_UNBLANK)
#define POWER_IS_OFF(pwr)	((pwr) == FB_BLANK_POWERDOWN)
#define POWER_IS_NRM(pwr)	((pwr) == FB_BLANK_NORMAL)

#define SPRD_DEVICE_ATTR(name)  \
	static ssize_t name##_show(struct device *dev,\
			       struct device_attribute *attr, char *buf);\
	static ssize_t name##_store(struct device *dev,\
				  struct device_attribute *attr,\
				  const char *buf, size_t count); \
	static DEVICE_ATTR(name, S_IRUGO | S_IWUSR, name##_show, name##_store)


enum {
	SC2705_DISP_OFF,
	SC2705_DISP_ON,
};

enum sc2705_lcd_mode {
	SC2705_TFT = 0,
	SC2705_AMOLED = 1,
	SC2705_LCD_MODE_DEF = SC2705_TFT,
};

enum {
	SC2705_WLED_BYPASS = 0,
	SC2705_WLED_DIRECT = 1,
	SC2705_WLED_DUTY_DET_PWM = 2,
	SC2705_WLED_DUTY_DET_ANALOG = 3,
	SC2705_WLED_DUTY_DET_MIXED = 4,
	SC2705_WLED_MODE_MAX,
};

enum {
	SC2705_IDAC_MODE = 0,
	SC2705_PWM_MODE = 1,
	SC2705_LEVEL_CTRL_DEF = SC2705_PWM_MODE,
};

SPRD_DEVICE_ATTR(i2c_read);
SPRD_DEVICE_ATTR(i2c_write);

struct sc2705_lcd {
	struct device *dev;
	struct delayed_work work;
	struct workqueue_struct *irqthread;
	struct lcd_device	*ld;
	struct backlight_device *wled;
	struct regmap *regmap;
	struct mutex	lock;
	struct pwm_device *pwm_dev;
	bool	legacy;
	int pwm_id;

	unsigned int	power;
	int irq;
	int brightness;

	u8 mode;
	u8 wled_mode;
	u8 level_ctrl;
	uint out_freq_step;
	uint pwm_out_duty;
	uint wled_idac;
	u8 pwm_duty_thresh;
	uint max_brightness;
	int display_ce_gpio;
};
struct data_i2c_t {
	u8 reg;
	u8 value;
};
static struct data_i2c_t data_i2c;
static struct regmap *i2c_regmap;
static int sc2705_power_on(struct sc2705_lcd *lcd);
static int sc2705_power_off(struct sc2705_lcd *lcd);

static ssize_t i2c_read_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	int ret;

	dev_info(dev, "data_i2c.reg 0x%x= 0x%x\n",
		 data_i2c.reg, data_i2c.value);
	ret = snprintf(buf, PAGE_SIZE, "i2c_read reg: 0x%x = 0x%x\n",
		 data_i2c.reg, data_i2c.value);

	return ret;
}

static ssize_t i2c_read_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {

	int ret;

	memset(&data_i2c, 0, sizeof(struct data_i2c_t));
	ret = kstrtou8(buf, 0, &data_i2c.reg);
	dev_info(dev, "data_i2c.reg = 0x%x (%d)\n", data_i2c.reg, ret);
	ret = regmap_read(i2c_regmap, data_i2c.reg,
			  (unsigned int *)&data_i2c.value);
	if (ret)
		dev_err(dev, "i2c_read_store i2c_read fail\n");
	dev_info(dev, "data_i2c.reg 0x%x= 0x%x\n",
		 data_i2c.reg, data_i2c.value);

	return count;
}

static ssize_t i2c_write_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	int ret;

	dev_info(dev, "data_i2c.reg (0x%x 0x%x)\n",
		data_i2c.reg, data_i2c.value);
	ret = snprintf(buf, PAGE_SIZE, "you set i2c_write 0x%x 0x%x\n",
		 data_i2c.reg, data_i2c.value);

	return ret;
}

static ssize_t i2c_write_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {

	int ret;

	memset(&data_i2c, 0, sizeof(struct data_i2c_t));
	ret = sscanf(buf, "%x %x", (unsigned int *)&data_i2c.reg,
	       (unsigned int *)&data_i2c.value);
	if (ret < 0) {
		dev_err(dev, "store error\n");
		return count;
	}
	dev_info(dev, "data_i2c = 0x%x 0x%x\n", data_i2c.reg, data_i2c.value);
	ret = regmap_write(i2c_regmap, data_i2c.reg, data_i2c.value);
	if (ret)
		dev_err(dev, "i2c_read_store i2c_read fail\n");

	return count;
}

static int sc2705_bl_set_pwm(struct sc2705_lcd *lcd, int brightness)
{
	unsigned int pwm_duty;
	int ret;

	/*
	 * The formula to convert level(from host) to pwm duty cycle as below:
	 * pwm_duty range (0 ~ 100%) = (level * pwm_period) / MAXIMUM
	 */
	pwm_duty = (unsigned int)((lcd->pwm_dev->period * brightness)
				/ lcd->max_brightness);

	ret = pwm_config(lcd->pwm_dev, pwm_duty, lcd->pwm_dev->period);
	if (ret) {
		dev_err(lcd->dev, "failed to set pwm duty cycle: %d\n", ret);
		return ret;
	}

	if (brightness == 0)
		pwm_disable(lcd->pwm_dev);
	else {
		ret = pwm_enable(lcd->pwm_dev);
		if (ret) {
			pwm_disable(lcd->pwm_dev);
			dev_err(lcd->dev,
				"failed to enable lcd pwm device: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int sc2705_adjust_brightness(struct sc2705_lcd *lcd, int brightness)
{
	int ret;
	int sc2705_brightness;
	/* No need for AMOLED mode */
	if (lcd->mode == SC2705_AMOLED)
		return 0;

	if (brightness > 255)
		brightness = 255;

	if (lcd->brightness == brightness)
		return 0;

	if (lcd->level_ctrl == SC2705_IDAC_MODE
		&& lcd->wled_mode == SC2705_WLED_DIRECT)
		sc2705_brightness = brightness * SC2705_WLED_IDAC_MAX
				/ lcd->max_brightness;
	else
		/* change to 128 steps */
		sc2705_brightness = brightness / 2;



	dev_info(lcd->dev, "brightness:%d sc2705_brightness:%d\n",
			brightness, sc2705_brightness);

	if (sc2705_brightness == 0) {
		ret = regmap_update_bits(lcd->regmap, SC2705_SUPPLY_ACTIVE,
					SC2705_WLED_EN_MASK, 0);
		if (ret)
			return ret;
	}

	if (lcd->wled_mode != SC2705_WLED_DIRECT)
		ret = sc2705_bl_set_pwm(lcd, sc2705_brightness);
	else {
		if (lcd->level_ctrl == SC2705_IDAC_MODE) {

			/* IDAC control for WLED brigntness */
			ret = regmap_update_bits(lcd->regmap,
				SC2705_WLED_CONFIG2,
				SC2705_IDAC_LINEAR_MASK
				|SC2705_IDAC_TARGET_H_MASK,
				SC2705_IDAC_LINEAR_MASK |
				(sc2705_brightness >> 8 & 0x7));
			if (ret)
				return ret;
			ret = regmap_write(lcd->regmap,
				SC2705_WLED_CONFIG3,
				sc2705_brightness & 0xFF);
		} else {
			/* PWM_OUT_DUTY control for WLED brigntness */
			ret = regmap_write(lcd->regmap,
				SC2705_WLED_CONFIG7,
				sc2705_brightness == 0 ? 1 : (sc2705_brightness & 0x7F));
		}
	}
	if (ret)
		return ret;

	if (sc2705_brightness > 0
		&& lcd->brightness == 0) {
		ret = regmap_update_bits(lcd->regmap, SC2705_SUPPLY_ACTIVE,
			SC2705_WLED_EN_MASK, SC2705_WLED_EN_MASK);
		if (ret)
			return ret;
	}
	lcd->brightness = brightness;

	return 0;
}

static int sc2705_backlight_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;
	struct sc2705_lcd *lcd = bl_get_data(bl);

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.fb_blank != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		brightness = 0;

	return sc2705_adjust_brightness(lcd, brightness);
}

static int sc2705_backlight_get_brightness(struct backlight_device *bl)
{
	struct sc2705_lcd *lcd = bl_get_data(bl);

	return lcd->brightness;
}

static int sc2705_power_on(struct sc2705_lcd *lcd)
{
	int ret = 0;

	if (gpio_is_valid(lcd->display_ce_gpio))
		gpio_set_value(lcd->display_ce_gpio, 1);
	else {
		ret = regmap_update_bits(lcd->regmap,
			SC2705_SUPPLY_ACTIVE,
			SC2705_DISPLAY_EN_MASK, SC2705_DISPLAY_EN_MASK);
		if (ret)
			dev_err(lcd->dev, "Failed to write register: %d\n", ret);
	}
	return ret;
}

static int sc2705_power_off(struct sc2705_lcd *lcd)
{
	int ret = 0;

	if (gpio_is_valid(lcd->display_ce_gpio))
		gpio_set_value(lcd->display_ce_gpio, 0);
	else {
		ret = regmap_update_bits(lcd->regmap,
			SC2705_SUPPLY_ACTIVE,
			SC2705_DISPLAY_EN_MASK, 0);
		if (ret)
			dev_err(lcd->dev, "Failed to write register: %d\n", ret);
	}
	return ret;
}

static int sc2705_set_power(struct lcd_device *ld, int power)
{
	struct sc2705_lcd *lcd = lcd_get_data(ld);
	int ret = 0;

	if (power != FB_BLANK_UNBLANK &&
		power != FB_BLANK_POWERDOWN &&
		power != FB_BLANK_NORMAL) {
		dev_err(lcd->dev, "power value should be 0, 1 or 4.\n");
		return -EINVAL;
	}

	if (POWER_IS_ON(power) && !POWER_IS_ON(lcd->power))
		ret = sc2705_power_on(lcd);
	else if (!POWER_IS_ON(power) && POWER_IS_ON(lcd->power))
		ret = sc2705_power_off(lcd);

	if (!ret)
		lcd->power = power;

	return ret;
}

static int sc2705_get_power(struct lcd_device *ld)
{
	struct sc2705_lcd *lcd = lcd_get_data(ld);

	return lcd->power;
}

static const struct backlight_ops sc2705_backlight_ops = {
	.update_status = sc2705_backlight_update_status,
	.get_brightness = sc2705_backlight_get_brightness,
};

static struct lcd_ops sc2705_lcd_ops = {
	.set_power = sc2705_set_power,
	.get_power = sc2705_get_power,
};

static bool sc2705_lcd_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case SC2705_LCD_STATUS_A ... SC2705_LCD_EVENT_E:
		case SC2705_SUPPLY_ACTIVE:
		case SC2705_SUPPLY_STATUS:
		case SC2705_WLED_CONFIG2:
		case SC2705_WLED_CONFIG3:
			return true;
		default:
			return false;
	}
}

static const struct regmap_config sc2705_lcd_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SC2705_DISP_REG_MAX,
	.volatile_reg = sc2705_lcd_volatile_reg,
};

static u8 sc2705_wled_mode_str(struct device *dev,
	const char *str)
{
	if (!strcmp(str, "bypass-mode"))
		return SC2705_WLED_BYPASS;
	else if (!strcmp(str, "direct-mode"))
		return SC2705_WLED_DIRECT;
	else if (!strcmp(str, "duty-det-pwm"))
		return SC2705_WLED_DUTY_DET_PWM;
	else if (!strcmp(str, "duty-det-analog"))
		return SC2705_WLED_DUTY_DET_ANALOG;
	else if (!strcmp(str, "duty-det-mixed"))
		return SC2705_WLED_DUTY_DET_MIXED;

	dev_warn(dev, "Invalid mode - set to default\n");
	return SC2705_WLED_DUTY_DET_ANALOG;
}

static u8 sc2705_level_ctrl_str(struct device *dev,
	const char *str)
{
	if (!strcmp(str, "idac-control"))
		return SC2705_IDAC_MODE;
	else if (!strcmp(str, "pwm-control"))
		return SC2705_PWM_MODE;

	dev_warn(dev, "Invalid WLED mode - set to default\n");
	return SC2705_LEVEL_CTRL_DEF;
}

static void sc2705_bllcd_of_to_pdata(struct device *dev,
			    struct sc2705_lcd *lcd)
{
	struct device_node *np = dev->of_node;
	u32 of_val32;
	const char *of_str;

	if (lcd->mode == SC2705_AMOLED)
		return;

	/* For WLED when the system is TFT Mode
	*/
	if (!of_property_read_string(np, "sprd,wled-mode", &of_str))
		lcd->wled_mode =
			sc2705_wled_mode_str(dev, of_str);
	else
		lcd->wled_mode = SC2705_WLED_BYPASS;

	/* In case of direct mode, this must be selected */
	if (lcd->wled_mode == SC2705_WLED_DIRECT) {
		if (!of_property_read_string(np, "sprd,level-ctrl-by", &of_str))
			lcd->level_ctrl =
				sc2705_level_ctrl_str(dev, of_str);
		else
			lcd->level_ctrl = SC2705_LEVEL_CTRL_DEF;
	}

	if (of_property_read_u32(np, "sprd,pwm-freq-hz", &of_val32) >= 0) {
		if (of_val32 >= SC2705_WLED_FREQ_MIN
			&& of_val32 <= SC2705_WLED_FREQ_MAX)
			lcd->out_freq_step = (of_val32 - 3000) / 240;
		else
			lcd->out_freq_step
				= (SC2705_WLED_FREQ_DEF - 3000) / 240;
	} else
		lcd->out_freq_step = (SC2705_WLED_FREQ_DEF - 3000) / 240;

	if (of_property_read_u32(np, "sprd,pwm-duty", &of_val32) >= 0) {
		if (of_val32 <= SC2705_WLED_PWM_DUTY_MAX)
			lcd->pwm_out_duty = of_val32;
		else
			lcd->pwm_out_duty = SC2705_WLED_PWM_DUTY_DEF;
	} else
		lcd->pwm_out_duty = SC2705_WLED_PWM_DUTY_DEF;

	if (of_property_read_u32(np, "sprd,wled-idac", &of_val32) >= 0) {
		if (of_val32 <= SC2705_WLED_IDAC_MAX)
			lcd->wled_idac = of_val32;
		else
			lcd->wled_idac = SC2705_WLED_IDAC_DEF;
	} else
		lcd->wled_idac = SC2705_WLED_IDAC_DEF;

	if (of_property_read_u32(np,
			"sprd,pwm-duty-threshold", &of_val32) >= 0) {
		if (of_val32 <= SC2705_PWM_DUTY_THRESH_MAX)
			lcd->pwm_duty_thresh = (u8)(of_val32 & 0xFF);
		else
			lcd->pwm_duty_thresh = SC2705_PWM_DUTY_THRESH_DEF;
	} else
		lcd->pwm_duty_thresh = SC2705_PWM_DUTY_THRESH_DEF;

	/* Set the max_brightness by the wled_mode and control factor */
	if (of_property_read_u32(np, "sprd,dft-brightness", &of_val32) >= 0)
		lcd->brightness = of_val32;

	of_property_read_u32(np, "sprd,max-brightness", &of_val32);
	if (lcd->wled_mode != SC2705_WLED_DIRECT) {
		if (of_property_read_u32(np,
				"sprd,max-brightness", &of_val32) >= 0) {
			if (of_val32 <= SC2705_PWM_BRIGHTNESS_MAX)
				lcd->max_brightness = of_val32;
			else
				lcd->max_brightness
					= SC2705_PWM_BRIGHTNESS_DEF;
		} else
			lcd->max_brightness = SC2705_PWM_BRIGHTNESS_DEF;
	} else {
			lcd->max_brightness = of_val32;
	}

	lcd->display_ce_gpio = of_get_named_gpio(np,
					"sprd,lcm-avdden-gpios", 0);
	if (!gpio_is_valid(lcd->display_ce_gpio))
		dev_info(dev, "sprd,lcm-avdden-gpios is not existed\n");
}

static int sc2705_bllcd_init(struct sc2705_lcd *lcd)
{
	int ret = 0;
	int fix_pwm_out_freq = 0, fix_pwm_out_duty = 0, fix_wled_idac = 0;
	int set_pwm_in_range = 0;
	unsigned int pwm_kHz;
	u8 range = 0;

	if (lcd->mode == SC2705_AMOLED)
		return 0;

	/* WLED init here */
	ret = regmap_update_bits(lcd->regmap,
		SC2705_WLED_CONFIG1,
			SC2705_WLED_MODE_MASK |
			SC2705_WLED_STR1_EN_MASK |
			SC2705_WLED_STR2_EN_MASK,
			lcd->wled_mode << SC2705_WLED_MODE_SHIFT |
			SC2705_WLED_STR1_EN_MASK |
			SC2705_WLED_STR2_EN_MASK);
	if (ret)
		goto error_i2c;

	switch (lcd->wled_mode) {
	case SC2705_WLED_BYPASS:
		fix_wled_idac = 1;
		break;
	case SC2705_WLED_DIRECT:
		if (lcd->level_ctrl == SC2705_IDAC_MODE)
			fix_pwm_out_duty = 1;
		else
			fix_wled_idac = 1;
		break;
	case SC2705_WLED_DUTY_DET_PWM:
		fix_pwm_out_freq = 1;
		fix_wled_idac = 1;
		set_pwm_in_range = 1;
		break;
	case SC2705_WLED_DUTY_DET_ANALOG:
		set_pwm_in_range = 1;
		break;
	case SC2705_WLED_DUTY_DET_MIXED:
		set_pwm_in_range = 1;
		fix_pwm_out_freq = 1;
		ret = regmap_write(lcd->regmap, SC2705_WLED_CONFIG5,
			lcd->pwm_duty_thresh & 0xFF);
		if (ret)
			goto error_i2c;
		break;
	default:
		dev_err(lcd->dev, "Invalid WLED Mode : %d\n", lcd->wled_mode);
		break;
	}

	if (fix_pwm_out_freq) {
		ret = regmap_write(lcd->regmap,
			SC2705_WLED_CONFIG6,
			lcd->out_freq_step);
		if (ret)
			goto error_i2c;
	}

	if (fix_pwm_out_duty) {
		ret = regmap_write(lcd->regmap,
			SC2705_WLED_CONFIG7,
			lcd->pwm_out_duty);
		if (ret)
			goto error_i2c;
	}

	if (fix_wled_idac) {
		/* Fix IDAC and control internal PWM duty */
		ret = regmap_update_bits(lcd->regmap,
			SC2705_WLED_CONFIG2,
			SC2705_IDAC_TARGET_H_MASK,
			lcd->wled_idac >> 8 & 0x7);
		if (ret)
			goto error_i2c;
		ret = regmap_update_bits(lcd->regmap,
			SC2705_WLED_CONFIG3,
			SC2705_IDAC_TARGET_H_MASK,
			lcd->wled_idac & 0xFF);
		if (ret)
			goto error_i2c;
	}

	if (set_pwm_in_range) {
		/* Fix IDAC and control internal PWM duty */
		pwm_kHz = 1000000/lcd->pwm_dev->period;

		if (pwm_kHz > 33) {
			dev_err(lcd->dev, "Invalid pwm input : %dkHz(must be 2~33kHz)\n",
				pwm_kHz);
			return -EINVAL;
		} else if (pwm_kHz > 30)
			range = 3;
		else if (pwm_kHz > 25)
			range = 2;
		else if (pwm_kHz > 14)
			range = 1;

		ret = regmap_update_bits(lcd->regmap,
			SC2705_WLED_CONFIG4,
			SC2705_PWM_IN_FREQ_RANGE_MASK,
			range << SC2705_PWM_IN_FREQ_RANGE_SHIFT);
		if (ret)
			goto error_i2c;
	}

	return 0;

error_i2c:
	dev_err(lcd->dev, "I2C error : %d\n", ret);
	return ret;
}

static irqreturn_t sc2705_lcd_irq_handler(int irq, void *data)
{
	struct sc2705_lcd *lcd = data;
	u8 events[LCD_IRQ_NUM];
	int ret, i;

	/* Check what events have happened */
	ret = regmap_bulk_read(lcd->regmap,
		SC2705_LCD_EVENT_A, events, LCD_IRQ_NUM);
	if (ret)
		goto error_i2c;

	/* Empty check due to shared interrupt */
	if ((events[0] | events[1] | events[2]
		| events[3] | events[4]) == 0x00)
		return IRQ_HANDLED;

	/* Clear events */
	ret = regmap_bulk_write(lcd->regmap,
		SC2705_LCD_EVENT_A, events, LCD_IRQ_NUM);
	if (ret)
		goto error_i2c;

	/* Event handling for SC2705_IRQ_EVENTs */
	for (i = 0; i < LCD_IRQ_NUM; i++) {
		if (events[i])
			dev_info(lcd->dev,
				"sc2705-bllcd event(%d): 0x%x\n", i, events[i]);
	}

	return IRQ_HANDLED;

error_i2c:
	dev_err(lcd->dev, "I2C error : %d\n", ret);
	return IRQ_NONE;
}

static int sc2705_bllcd_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct backlight_properties props;
	struct sc2705_lcd *lcd;
	int ret;
	unsigned int value;

	lcd = devm_kzalloc(&client->dev, sizeof(struct sc2705_lcd),
			     GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;
	lcd->dev = &client->dev;

	lcd->regmap = devm_regmap_init_i2c(client, &sc2705_lcd_regmap);
	if (IS_ERR(lcd->regmap)) {
		ret = PTR_ERR(lcd->regmap);
		dev_err(&client->dev, "fail : allocate reg. map: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(client, lcd);

	ret = regmap_read(lcd->regmap, SC2705_LCD_STATUS_A, &value);
	if (ret)
		goto error_i2c;

	if (value & SC2705_DISPLAY_TYPE_MASK)
		lcd->mode = SC2705_AMOLED;
	else
		lcd->mode = SC2705_TFT;

	/* Handle DT data if provided */
	if (client->dev.of_node)
		sc2705_bllcd_of_to_pdata(&client->dev, lcd);

	if (lcd->wled_mode != SC2705_WLED_DIRECT) {
		/* Get pwm and regulatot for WLED device */
		lcd->pwm_dev = devm_pwm_get(&client->dev, NULL);
		if (IS_ERR(lcd->pwm_dev) && !client->dev.of_node) {
			dev_err(lcd->dev,
				"unable to request PWM, trying legacy API\n");
			lcd->legacy = true;
			lcd->pwm_dev =
			    pwm_request(lcd->pwm_id, "sc-wled");
		}

		if (IS_ERR(lcd->pwm_dev)) {
			dev_err(lcd->dev, "unable to request PWM\n");
			return PTR_ERR(lcd->pwm_dev);
		}
	}

	lcd->ld = devm_lcd_device_register(lcd->dev,
			SC2705_LCD_NAME, lcd->dev, lcd,
			&sc2705_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "failed to register sc2705_lcd_ops\n");
		return PTR_ERR(lcd->ld);
	}

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = lcd->max_brightness;
	lcd->wled = devm_backlight_device_register(lcd->dev,
					SC2705_WLED_NAME, lcd->dev, lcd,
					&sc2705_backlight_ops, &props);
	if (IS_ERR(lcd->wled)) {
		dev_err(lcd->dev, "Failed to register backlight\n");
		return PTR_ERR(lcd->wled);
	}

	/* interrupt enable  : irq 0 is not allowed */
	if (!client->irq) {
		dev_err(lcd->dev, "No IRQ configured\n");
		return -EINVAL;
	}
	ret = devm_request_threaded_irq(lcd->dev, client->irq, NULL,
				sc2705_lcd_irq_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
				"sc2705-bllcd", lcd);
	if (ret != 0) {
		dev_err(lcd->dev, "Failed to request IRQ: %d\n", client->irq);
		return ret;
	}

	/* Panel & backlihgt initialisation */
	ret = sc2705_bllcd_init(lcd);
	if (ret)
		return ret;
	i2c_regmap = lcd->regmap;
	ret = sysfs_create_file(&lcd->ld->dev.kobj, &dev_attr_i2c_read.attr);
	if (ret)
		dev_err(lcd->dev, "Failed create sysfs file i2c_read\n");
	ret = sysfs_create_file(&lcd->ld->dev.kobj, &dev_attr_i2c_write.attr);
	if (ret)
		dev_err(lcd->dev, "Failed create sysfs file i2c_write\n");

	ret = sc2705_adjust_brightness(lcd, lcd->brightness);
	if (ret)
		return ret;

	if (lcd->brightness > 0) {
		if (gpio_is_valid(lcd->display_ce_gpio))
			ret = regmap_update_bits(lcd->regmap,
				SC2705_SUPPLY_ACTIVE,
				SC2705_WLED_EN_MASK,
				SC2705_WLED_EN_MASK);
		else
			ret = regmap_update_bits(lcd->regmap,
				SC2705_SUPPLY_ACTIVE,
				SC2705_WLED_EN_MASK |
				SC2705_DISPLAY_EN_MASK,
				SC2705_WLED_EN_MASK |
				SC2705_DISPLAY_EN_MASK);
		if (ret)
			return ret;
	}
	return 0;

error_i2c:
	dev_err(lcd->dev, "I2C error : %d\n", ret);
	return ret;
}

static int sc2705_bllcd_remove(struct i2c_client *client)
{
	struct sc2705_lcd *lcd = i2c_get_clientdata(client);

	sc2705_adjust_brightness(lcd, 0);
	return sc2705_set_power(lcd->ld, FB_BLANK_POWERDOWN);
}

#ifdef CONFIG_PM_SLEEP
static int sc2705_bllcd_suspend(struct device *dev)
{
	struct sc2705_lcd *lcd = dev_get_drvdata(dev);

	/*
	 * when lcd panel is suspend, lcd panel becomes off
	 * regardless of status.
	 */
	return sc2705_set_power(lcd->ld, FB_BLANK_POWERDOWN);
}

static int sc2705_bllcd_resume(struct device *dev)
{
	struct sc2705_lcd *lcd = dev_get_drvdata(dev);

	return sc2705_set_power(lcd->ld, FB_BLANK_UNBLANK);
}
#endif

static const struct of_device_id sc2705_bllcd_of_match[] = {
	{ .compatible = "sprd,sc2705-bllcd", },
	{ }
};
MODULE_DEVICE_TABLE(of, sc2705_bllcd_of_match);

static const struct i2c_device_id sc2705_lcd_i2c_id[] = {
	{ "sc2705-bllcd", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc2705_lcd_i2c_id);

static SIMPLE_DEV_PM_OPS(sc2705_bllcd_pm_ops,
		sc2705_bllcd_suspend, sc2705_bllcd_resume);

static struct i2c_driver sc2705_bllcd_driver = {
	.driver = {
		.name = "sc2705-bllcd",
		.of_match_table = of_match_ptr(sc2705_bllcd_of_match),
		.pm	= &sc2705_bllcd_pm_ops,
	},
	.probe = sc2705_bllcd_probe,
	.remove = sc2705_bllcd_remove,
	.id_table	= sc2705_lcd_i2c_id,
};

module_i2c_driver(sc2705_bllcd_driver);

MODULE_DESCRIPTION("LCD panel and Backlight driver for SC2705");
MODULE_AUTHOR("Roy Im <Roy.Im.Opensource@diasemi.com>");
MODULE_LICENSE("GPL");
