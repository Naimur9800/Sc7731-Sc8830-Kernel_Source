/*
 * Haptic device driver for SC2705
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

#include <linux/err.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>

#include "sc2705-haptic.h"
#include "sc2705-haptic-helper.h"

#define SC2705_HAPTIC_EXT_PULSE		0
#define SC2705_HAPTIC_INT_PULSE		1

/* Voltage Rating related */
#define SC2705_VOLT_RATE_MAX		6000
#define SC2705_VOLT_STEP_uV		23400
#define SC2705_NOM_VOLT_DFT		0x6B
#define SC2705_OVRDRV_VOLT_DFT		0x78

#define SC2705_CURR_RATE_DFT		0x0E
#define SC2705_CURR_STEP_uA		7200
#define SC2705_CURR_RATE_MAX		382

#define SC2705_TUNED_OUTPUT_DFT		0x7F
#define SC2705_TUNED_OUTPUT_MAX		128

#define SC2705_RESONT_FREQH_DFT		0x39
#define SC2705_RESONT_FREQL_DFT		0x32

#define SC2705_MIN_RESONAT_FREQ		50
#define SC2705_MAX_RESONAT_FREQ		300
#define SC2705_DEF_RESONAT_FREQ		205

#define SC2705_MIN_PWM_FREQ_kHz		10
#define SC2705_MAX_PWM_FREQ_kHz		250
#define SC2705_DEF_PWM_FREQ_kHz		100

#define SC2705_ADC_LSB_uVOLTS	183
#define SC2705_IMPD_Ohm_DFT		10
#define SC2705_IMPD_Ohm_MAX		50
#define SC2705_IMPD_Ohm_MIN		8

#define SC2705_SNP_MEM_SIZE		90
#define IRQ_NUM				3

enum sc2705_haptic_mode {
	SC2705_H_LRA = 0,
	SC2705_H_ERM_BAR = 1,
	SC2705_H_ERM_COIN = 2,
	SC2705_H_MODE_DFT = SC2705_H_LRA,
};

enum sc2705_haptic_op_mode {
	SC2705_H_DRO_MODE = 1,
	SC2705_H_PWM_MODE = 2,
	SC2705_H_WAVE_REG_TRIG = 3,
	SC2705_H_WAVE_RISING_TRIG = 4,
	SC2705_H_WAVE_DUAL_TRIG = 5,
	SC2705_H_OPMODE_DFT = SC2705_H_DRO_MODE,
};

struct sc2705_haptic {
	struct regmap *regmap;
	struct input_dev *input_dev;
	struct device *dev;
	struct i2c_client *client;
	struct pwm_device *pwm_dev;
	bool	legacy;
	int pwm_id;
	struct work_struct work;

	bool suspend_state;
	unsigned int magnitude;

	u8 mode;
	u8 op_mode;
	u32 nom_volt_rating;
	u32 abs_overdrive_volt;
	u32 current_rate;
	u32 tuned_output;
	u32 impd_ohms;
	u32 filter_res_trim;
	u32 resonant_freq_h;
	u32 resonant_freq_l;
	bool mem_update;
	u8 snp_mem[SC2705_SNP_MEM_SIZE];
	u8 acc_en;
};

static bool sc2705_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SC2705_IRQ_EVENT1:
	case SC2705_IRQ_EVENT_WARNING_DIAG:
	case SC2705_IRQ_EVENT_PAT_DIAG:
	case SC2705_IRQ_STATUS1:
	case SC2705_TOP_CTL1:
		return 1;
	default:
		return 0;
	}
}

static const struct regmap_config sc2705_haptic_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SC2705_SNP_MEM_89,
	.volatile_reg = sc2705_volatile_register,
};

static const u8 r_lra_ohms[] = {
/* Ohms, values */
	42,	/* 0 */
	28,	/* 1 */
	19,	/* 2 */
	0,	/* 3 */
};

static int sc2705_haptic_mem_update(struct sc2705_haptic *haptics)
{
	int ret;
	unsigned int val;

	ret = regmap_read(haptics->regmap, SC2705_MEM_CTL1, &val);
	if (ret)
		return ret;
	return regmap_bulk_write(haptics->regmap, val,
		haptics->snp_mem, SC2705_SNP_MEM_89 - val + 1);
}

static int sc2705_haptic_set_pwm(struct sc2705_haptic *haptics)
{
	#define PWM_DUTY_WHOLE_RANGE
	#define MAX_MAGNITUDE_SHIFT	16
	u64 period_mag_multi;
	unsigned int pwm_duty;
	int ret;

	/* The magnitude comes from force-feedback interface.
	 * The formula to convert magnitude to pwm duty cycle as below:
	 * - pwm_duty
	 *	1. Full range (0 ~ 100%)
	 *		F_R = (magnitude * pwm_period)
	 *				/ MAX_MAGNITUDE(0xFFFF)
	 *	2. Half range (50 ~ 100%) for 0 ~ 0xffff
	 *		H_R = (period + F_R(full range)) /2
	 * - For your information,
	 *	if magnitude == 0x8000(32768), then it is 50% dute cycle.
	 */
	period_mag_multi
		= (u64)(haptics->pwm_dev->period * haptics->magnitude);
#ifdef PWM_DUTY_WHOLE_RANGE
	pwm_duty = (unsigned int)(period_mag_multi >> MAX_MAGNITUDE_SHIFT);
#else
	pwm_duty = (unsigned int)(period_mag_multi >> MAX_MAGNITUDE_SHIFT
				+ haptics->pwm_dev->period) / 2;
#endif

	ret = pwm_config(haptics->pwm_dev,
		pwm_duty, haptics->pwm_dev->period);
	if (ret) {
		dev_err(haptics->dev,
			"failed to set pwm duty cycle: %d\n", ret);
		return ret;
	}

	ret = pwm_enable(haptics->pwm_dev);
	if (ret) {
		pwm_disable(haptics->pwm_dev);
		dev_err(haptics->dev,
			"failed to enable haptics pwm device: %d\n", ret);
		return ret;
	}
	return 0;
}

static void sc2705_haptic_enable(struct sc2705_haptic *haptics)
{
	int ret = 0;

	if (haptics->op_mode == SC2705_H_PWM_MODE)
		if (sc2705_haptic_set_pwm(haptics))
			return;

	if (haptics->op_mode == SC2705_H_DRO_MODE) {
		/* If acc_en == 1,
		*	set from 0 ~ 127, control level but not direction
		*   If acc_en == 0,
		*	set from -128 ~ 127, control level & direction
		*/

		/* Set driver level
		* as a % of ACTUATOR_NOMMAX(nom_volt_rating)
		*/
		ret = regmap_write(haptics->regmap, SC2705_TOP_CTL2,
			haptics->magnitude & 0x7F);
	} else {
		if (haptics->op_mode != SC2705_H_PWM_MODE) {
			ret = regmap_write(haptics->regmap,
					SC2705_SEQ_CTL2,
					haptics->magnitude & 0xff);
		}
	}
	if (ret)
		goto error_i2c;

	ret = regmap_update_bits(haptics->regmap, SC2705_TOP_CTL1,
		SC2705_OPERATION_MODE_MASK,
		haptics->op_mode);
	if (ret)
		goto error_i2c;

	if (haptics->op_mode == SC2705_H_PWM_MODE
		|| haptics->op_mode == SC2705_H_WAVE_REG_TRIG) {
		ret = regmap_update_bits(haptics->regmap,
			SC2705_TOP_CTL1, SC2705_SEQ_START_MASK,
			SC2705_SEQ_START_MASK);
		if (ret)
			goto error_i2c;
	}
	return;

error_i2c:
	dev_err(haptics->dev, "SC2705 enable i2c error : %d\n", ret);
}

static void sc2705_haptic_disable(struct sc2705_haptic *haptics)
{
	int ret;

	/* external PWM or GPIO input triggered waveform mode
	* which means rising edge/dual edge mode,
	* would be finished if this functions is performed.
	*/
	ret = regmap_update_bits(haptics->regmap,
		SC2705_TOP_CTL1,
		SC2705_OPERATION_MODE_MASK, 0);
	if (ret)
		goto error_i2c;

	switch (haptics->op_mode) {
	case SC2705_H_DRO_MODE:
		ret = regmap_write(haptics->regmap,
			SC2705_TOP_CTL2, 0);
		if (ret)
			goto error_i2c;
		break;
	case SC2705_H_PWM_MODE:
		pwm_disable(haptics->pwm_dev);
	default:
		break;
	}
	return;

error_i2c:
	pwm_disable(haptics->pwm_dev);
	dev_err(haptics->dev, "SC2705 haptic disable error : %d\n", ret);
}

static void sc2705_haptic_work(struct work_struct *work)
{
	struct sc2705_haptic *haptics =
		container_of(work, struct sc2705_haptic, work);

	if (haptics->magnitude)
		sc2705_haptic_enable(haptics);
	else
		sc2705_haptic_disable(haptics);
}

static int sc2705_haptic_play(struct input_dev *dev, void *data,
				       struct ff_effect *effect)
{
	struct sc2705_haptic *haptics = input_get_drvdata(dev);

	if (effect->u.rumble.strong_magnitude > 0)
		haptics->magnitude = effect->u.rumble.strong_magnitude;
	else if (effect->u.rumble.weak_magnitude > 0)
		haptics->magnitude = effect->u.rumble.weak_magnitude;
	else
		haptics->magnitude = 0;

	schedule_work(&haptics->work);
	return 0;
}

static int sc2705_haptic_open(struct input_dev *dev)
{
	struct sc2705_haptic *haptics = input_get_drvdata(dev);
	int ret;

	ret = regmap_update_bits(haptics->regmap,
		SC2705_TOP_CTL1,
		SC2705_PATTERN_INACTIVE_MODE_MASK,
		SC2705_PATTERN_INACTIVE_MODE_MASK);
	if (ret)
		dev_err(haptics->dev,
			"Failed to open haptic, i2c error: %d\n", ret);
	return ret;
}

static void sc2705_haptic_close(struct input_dev *dev)
{
	struct sc2705_haptic *haptics = input_get_drvdata(dev);
	int ret;

	cancel_work_sync(&haptics->work);

	ret = regmap_update_bits(haptics->regmap,
		SC2705_TOP_CTL1,
		SC2705_OPERATION_MODE_MASK, 0);
	if (ret)
		goto error_i2c;

	if (haptics->op_mode == SC2705_H_DRO_MODE) {
		ret = regmap_write(haptics->regmap,
			SC2705_TOP_CTL2, 0);

		if (ret)
			goto error_i2c;
	}

	ret = regmap_update_bits(haptics->regmap,
		SC2705_TOP_CTL1,
		SC2705_PATTERN_INACTIVE_MODE_MASK, 0);
	if (ret)
		goto error_i2c;

	return;
error_i2c:
	dev_err(haptics->dev, "SC2705-haptic I2C error : %d\n", ret);
}

static ssize_t sc2705_haptic_debug_register(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int reg, value, len;
	char *blank, str[32] = { 0 };
	struct sc2705_haptic *haptics;

	haptics = i2c_get_clientdata(to_i2c_client(dev));
	dev_info(dev, "haptics=%p\n", haptics);
	if (IS_ERR_OR_NULL(haptics)) {
		dev_err(dev, "failed to get haptics\n");
		return count;
	}

	if (!strcmp(attr->attr.name, "read")) {
		if (kstrtouint(buf, 16, &reg) < 0) {
			dev_err(dev,
					"failed to get register address from %s\n",
					buf);
			return count;
		}

		regmap_read(haptics->regmap, reg, &value);
		dev_info(dev, "read reg=0x%02X, value=0x%02X\n", reg, value);
		return count;
	} else if (!strcmp(attr->attr.name, "write")) {
		blank = strchr(buf, ' ');
		if (!blank) {
			dev_err(dev, "failed to parse from %s\n", buf);
			return count;
		}
		len = blank - buf;
		strncpy(str, buf, len);
		if (kstrtouint(str, 16, &reg) < 0) {
			dev_err(dev,
					"failed to get register address from %s\n",
					str);
			return count;
		}
		len = count - len - 1;
		strncpy(str, blank + 1, len);
		str[len] = '\0';
		if (kstrtouint(str, 16, &value) < 0) {
			dev_err(dev,
					"failed to get register value from %s\n",
					str);
			return count;
		}

		dev_info(dev, "write reg=0x%02X, value=0x%02X\n", reg, value);
		regmap_write(haptics->regmap, reg, value);

		return count;
	}

	dev_warn(dev, "impossible to get here!\n");

	return count;
}

static ssize_t sc2705_haptic_debug_dump(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int reg, value;
	struct sc2705_haptic *haptics;
	ssize_t size = 0;

	haptics = i2c_get_clientdata(to_i2c_client(dev));
	dev_info(dev, "haptics=%p\n", haptics);
	if (IS_ERR_OR_NULL(haptics)) {
		dev_err(dev, "failed to get haptics\n");
		return 0;
	}

	size += sprintf(buf, "mode=%d, op_mode=%d\n",
			haptics->mode, haptics->op_mode);

#define DUMP_REG(addr) \
do {\
	reg = addr;\
	regmap_read(haptics->regmap, reg, &value);\
	size += sprintf(buf + size, "reg=0x%02X, value=0x%02X\n", reg, value);\
} while (0)

	DUMP_REG(0x13);
	DUMP_REG(0x21);
	DUMP_REG(0x22);
	DUMP_REG(0x24);
	DUMP_REG(0x03);
	DUMP_REG(0x04);
	DUMP_REG(0x05);
	DUMP_REG(0x06);
	DUMP_REG(0x07);

	return size;
}

static irqreturn_t sc2705_haptic_irq_handler(int irq, void *data);
static ssize_t sc2705_haptic_debug_irq(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sc2705_haptic *haptics;

	haptics = i2c_get_clientdata(to_i2c_client(dev));
	dev_info(dev, "haptics=%p\n", haptics);
	if (IS_ERR_OR_NULL(haptics)) {
		dev_err(dev, "failed to get haptics\n");
		return count;
	}

	sc2705_haptic_irq_handler(0, haptics);

	return count;
}

static DEVICE_ATTR(read, S_IWUSR, NULL, sc2705_haptic_debug_register);
static DEVICE_ATTR(write, S_IWUSR, NULL, sc2705_haptic_debug_register);
static DEVICE_ATTR(dump, S_IRUGO, sc2705_haptic_debug_dump, NULL);
static DEVICE_ATTR(irq, S_IWUSR, NULL, sc2705_haptic_debug_irq);

static struct attribute *sc2705_haptic_debug_attrs[] = {
	&dev_attr_read.attr,
	&dev_attr_write.attr,
	&dev_attr_dump.attr,
	&dev_attr_irq.attr,
	NULL,
};

static struct attribute_group sc2705_haptic_debug_attr_group = {
	.attrs = sc2705_haptic_debug_attrs,
};

static u8 sc2705_haptic_of_mode_str(struct device *dev,
	const char *str)
{
	if (!strcmp(str, "LRA_MODE"))
		return SC2705_H_LRA;
	else if (!strcmp(str, "ERM_BAR"))
		return SC2705_H_ERM_BAR;
	else if (!strcmp(str, "ERM_COIN"))
		return SC2705_H_ERM_COIN;

	dev_warn(dev, "Invalid HAPTIC mode\n");
	return SC2705_H_MODE_DFT;

}

static u8 sc2705_haptic_of_opmode_str(struct device *dev,
	const char *str)
{
	if (!strcmp(str, "DRO-MODE"))
		return SC2705_H_DRO_MODE;
	else if (!strcmp(str, "PWM-MODE"))
		return SC2705_H_PWM_MODE;
	else if (!strcmp(str, "WAVEFORM-Reg-trig"))
		return SC2705_H_WAVE_REG_TRIG;
	else if (!strcmp(str, "WAVEFORM-Rising-trig"))
		return SC2705_H_WAVE_RISING_TRIG;
	else if (!strcmp(str, "WAVEFORM-dual-trig"))
		return SC2705_H_WAVE_DUAL_TRIG;

	dev_warn(dev, "Invalid mode - set to default\n");
	return SC2705_H_OPMODE_DFT;
}

static u8 sc2705_haptic_of_volt_rating_set(u32 val)
{
	u32 voltage;

	if (val < SC2705_VOLT_RATE_MAX)
		voltage = (val * 1000 / SC2705_VOLT_STEP_uV + 1);
	else
		voltage = SC2705_NOM_VOLT_DFT;
	if (voltage > 0xFF)
		return 0xFF;
	return voltage;
}

static void sc2705_haptic_of_to_pdata(struct device *dev,
				      struct sc2705_haptic *haptics)
{
	struct device_node *np = dev->of_node;
	u32 of_val32;
	unsigned int mem[SC2705_SNP_MEM_SIZE];
	const char *of_str;
	int i;

	if (!of_property_read_string(np, "sprd,vib-mode", &of_str))
		haptics->mode = sc2705_haptic_of_mode_str(dev, of_str);
	else
		haptics->mode = SC2705_H_MODE_DFT;

	if (!of_property_read_string(np, "sprd,play-mode", &of_str))
		haptics->op_mode = sc2705_haptic_of_opmode_str(dev, of_str);
	else
		haptics->op_mode = SC2705_H_OPMODE_DFT;

	if (!of_property_read_u32(np, "sprd,nom-volt-rating", &of_val32))
		if (of_val32 < SC2705_VOLT_RATE_MAX)
			haptics->nom_volt_rating =
				sc2705_haptic_of_volt_rating_set(of_val32);
		else
			haptics->nom_volt_rating = SC2705_NOM_VOLT_DFT;
	else
		haptics->nom_volt_rating = SC2705_NOM_VOLT_DFT;

	if (!of_property_read_u32(np, "sprd,abs-max-volt", &of_val32))
		if (of_val32 < SC2705_VOLT_RATE_MAX)
			haptics->abs_overdrive_volt =
				sc2705_haptic_of_volt_rating_set(of_val32);
		else
			haptics->abs_overdrive_volt = SC2705_OVRDRV_VOLT_DFT;
	else
		haptics->abs_overdrive_volt = SC2705_OVRDRV_VOLT_DFT;

	if (!of_property_read_u32(np, "sprd,current-rate", &of_val32))
		if (of_val32 < SC2705_CURR_RATE_MAX)
			haptics->current_rate =
				(of_val32 * 1000 - 28600) / SC2705_CURR_STEP_uA;
		else
			haptics->current_rate = SC2705_CURR_RATE_DFT;
	else
		haptics->current_rate = SC2705_CURR_RATE_DFT;

	if (!of_property_read_u32(np, "sprd,tuned-output", &of_val32))
		if (of_val32 < SC2705_TUNED_OUTPUT_MAX)
			haptics->tuned_output = of_val32;
		else
			haptics->tuned_output = SC2705_TUNED_OUTPUT_DFT;
	else
		haptics->tuned_output = SC2705_TUNED_OUTPUT_DFT;

	if (haptics->mode == SC2705_H_LRA) {
		if (of_property_read_u32(np, "sprd,resonant-freq",
						&of_val32) >= 0) {
			if (of_val32 < SC2705_MAX_RESONAT_FREQ &&
				of_val32 > SC2705_MIN_RESONAT_FREQ) {
				haptics->resonant_freq_h =
				(((1*1000000 / of_val32) * 1000 / 665 / 2)
					>> 7) & 0xFF;
				haptics->resonant_freq_l =
				(((1*1000000 / of_val32) * 1000 / 665 / 2)
					 & 0x7F);
			} else {
				haptics->resonant_freq_h
					= SC2705_RESONT_FREQH_DFT;
				haptics->resonant_freq_l
					= SC2705_RESONT_FREQL_DFT;
			}
		} else {
			haptics->resonant_freq_h = SC2705_RESONT_FREQH_DFT;
			haptics->resonant_freq_l = SC2705_RESONT_FREQL_DFT;
		}
	}

	if (of_property_read_u32(np, "sprd,impedance-ohms", &of_val32) >= 0)
		if (of_val32 < SC2705_IMPD_Ohm_MAX)
			haptics->impd_ohms = of_val32;
		else
			haptics->impd_ohms = SC2705_IMPD_Ohm_DFT;
	else
		haptics->impd_ohms = SC2705_IMPD_Ohm_DFT;

	if (of_property_read_u32(np, "sprd,filter-res-trim", &of_val32) >= 0)
		if (of_val32 <= 3)
			haptics->filter_res_trim = of_val32;
		else
			haptics->filter_res_trim = 0;
	else
		haptics->filter_res_trim = 0;

	if (of_property_read_u32_array(np, "sprd,mem-array",
		&mem[0], SC2705_SNP_MEM_SIZE) >= 0) {
		haptics->mem_update = 1;
		for (i = 0; i < SC2705_SNP_MEM_SIZE; i++)
			if (mem[i] > 0xff)
				haptics->snp_mem[i] = 0x0;
			else
				haptics->snp_mem[i] = (u8)mem[i];
	} else
		haptics->mem_update = 0;
}

static irqreturn_t sc2705_haptic_irq_handler(int irq, void *data)
{
	struct sc2705_haptic *haptics = data;
	u8 events[IRQ_NUM];
	int ret, i;

	/* Check what events have happened */
	ret = regmap_bulk_read(haptics->regmap,
		SC2705_IRQ_EVENT1, events, IRQ_NUM);
	if (ret)
		goto error_i2c;

	/* Empty check due to shared interrupt */
	if ((events[0] | events[1] | events[2]) == 0x00)
		return IRQ_HANDLED;

	if (events[0] & SC2705_E_PAT_FAULT_MASK) {
		/* Stop first if Haptic is working
		 * Otherwise, the fault may happen continually
		* even though the bit is cleared.
		*/
		/* TODO modify SC2705_TOP_CTL1 according to mode */
		ret = regmap_update_bits(haptics->regmap,
			SC2705_TOP_CTL1,
			SC2705_OPERATION_MODE_MASK, 0);
		if (ret)
			goto error_i2c;
	}

	/* Clear events */
	ret = regmap_write(haptics->regmap,
		SC2705_IRQ_EVENT1, events[0]);
	if (ret)
		goto error_i2c;

	/* Event handling for SC2705_IRQ_EVENT1 */
	for (i = 0; i < IRQ_NUM; i++) {
		if (events[i])
			dev_info(haptics->dev,
			"sc2705-haptic event(%d): 0x%x\n", i, events[i]);
	}

	return IRQ_HANDLED;

error_i2c:
	dev_err(haptics->dev, "Haptic I2C error : %d\n", ret);
	return IRQ_NONE;
}

static int sc2705_haptic_init(struct sc2705_haptic *haptics)
{
	int ret, i;
	u32 val = 0;
	unsigned int read_val;

	/* Apply user settings */
	if (haptics->mode == SC2705_H_LRA) {
		/* LRA default value */
		haptics->acc_en = 1;
		ret = regmap_update_bits(haptics->regmap,
			SC2705_TOP_CFG1,
			SC2705_TOP_CFG1_ALL_MASK,
			haptics->mode << SC2705_ACTUATOR_TYPE_SHIFT |
			1 << SC2705_FREQ_TRACK_EN_SHIFT |
			haptics->acc_en << SC2705_ACCELERATION_EN_SHIFT |
			1 << SC2705_RAPID_STOP_EN_SHIFT);
		if (ret)
			goto error_i2c;

		ret = regmap_write(haptics->regmap,
			SC2705_FRQ_LRA_PER_H,
			haptics->resonant_freq_h);
		if (ret)
			goto error_i2c;
		ret = regmap_write(haptics->regmap,
			SC2705_FRQ_LRA_PER_L,
			haptics->resonant_freq_l);
		if (ret)
			goto error_i2c;

	} else if (haptics->mode == SC2705_H_ERM_COIN) {

		if (haptics->op_mode >= SC2705_H_WAVE_REG_TRIG) {

			ret = regmap_write(haptics->regmap,
				SC2705_TOP_CFG1, 0xE);
			if (ret)
				goto error_i2c;

			haptics->acc_en = 1;
			ret = regmap_write(haptics->regmap,
				SC2705_TOP_CFG1, 0x1E);
			if (ret)
				goto error_i2c;

		} else {
			/* Coin type ERM default for DRO and PWM */
			ret = regmap_write(haptics->regmap,
				SC2705_TOP_CFG1, 0);
			if (ret)
				goto error_i2c;
			val = 1 << SC2705_ACTUATOR_TYPE_SHIFT;
			haptics->acc_en = 0;
			ret = regmap_write(haptics->regmap,
				SC2705_TOP_CFG1, val);
			if (ret)
				goto error_i2c;
		}

		ret = regmap_update_bits(haptics->regmap,
			SC2705_TOP_CTL1, SC2705_CALIB_REQ_MASK, 0);
		if (ret)
			goto error_i2c;

		ret = regmap_update_bits(haptics->regmap,
			SC2705_TOP_CFG4,
			SC2705_TST_CALIB_IMPEDANCE_DIS_MASK |
			SC2705_V2I_FACTOR_FREEZE_MASK,
			SC2705_TST_CALIB_IMPEDANCE_DIS_MASK |
			SC2705_TST_CALIB_IMPEDANCE_DIS_MASK);
		if (ret)
			goto error_i2c;

	} else {
		/* Bar type ERM follow default value */
		haptics->acc_en = 1;
		ret = regmap_update_bits(haptics->regmap,
			SC2705_TOP_CFG1,
			SC2705_ACTUATOR_TYPE_MASK
				| SC2705_ACCELERATION_EN_MASK,
			1 << SC2705_ACTUATOR_TYPE_SHIFT |
			haptics->acc_en << SC2705_ACCELERATION_EN_SHIFT);
		if (ret)
			goto error_i2c;
	}

	/* set working voltage */
	ret = regmap_write(haptics->regmap, SC2705_ACTUATOR1,
		haptics->nom_volt_rating & 0xFF);
	if (ret)
		goto error_i2c;

	/* set max voltage */
	ret = regmap_write(haptics->regmap, SC2705_ACTUATOR2,
		haptics->abs_overdrive_volt & 0xFF);
	if (ret)
		goto error_i2c;

	/* set working current */
	ret = regmap_update_bits(haptics->regmap,
		SC2705_ACTUATOR3, SC2705_IMAX_MASK,
		haptics->current_rate);
	if (ret)
		goto error_i2c;

	/* haptic goes to STANDBY rather than OFF when inactive */
	/* TODO: may waste power?? */
	ret = regmap_update_bits(haptics->regmap,
		SC2705_TOP_CTL1, SC2705_PATTERN_INACTIVE_MODE_MASK,
		SC2705_PATTERN_INACTIVE_MODE_MASK);
	if (ret)
		goto error_i2c;

#ifdef SC2705_HAPTIC_AUTO_CALIB
	/* Automatic calibaration if need */
	if (haptics->mode != SC2705_H_ERM_COIN) {
		ret = regmap_update_bits(haptics->regmap,
			SC2705_TOP_CTL1,
			SC2705_CALIB_REQ_MASK, SC2705_CALIB_REQ_MASK);
		if (ret)
			goto error_i2c;
	}
#endif

	if (haptics->mem_update) {
		ret = sc2705_haptic_mem_update(haptics);
		if (ret)
			goto error_i2c;
	}

	/* Setting Impedance related values.
	 * have to receive first the impedance Ohms from DT
	*/
	ret = regmap_read(haptics->regmap,
			SC2705_H_TRIM1, &read_val);
	if (ret)
		goto error_i2c;

	read_val = (read_val >> 4) & 0xf;
	val = (haptics->impd_ohms*(haptics->current_rate+4)*3)*10000/
		(SC2705_ADC_LSB_uVOLTS*4*(58+read_val));
	/* idac_gain */
	ret = regmap_write(haptics->regmap, SC2705_CALIB_V2I_L,
		val & 0xFF);
	if (ret)
		goto error_i2c;
	ret = regmap_write(haptics->regmap, SC2705_CALIB_V2I_H,
		(val >> 8) & 0xFF);
	if (ret)
		goto error_i2c;

	for (i = 0; i < 4; i++) {
		if (r_lra_ohms[i] <= haptics->impd_ohms) {
			val = i;
			break;
		}
	}
	ret = regmap_update_bits(haptics->regmap,
		SC2705_H_TRIM2, 0x0F, val << 2 | haptics->filter_res_trim);
	if (ret)
		goto error_i2c;

	ret = regmap_update_bits(haptics->regmap,
		SC2705_IRQ_MASK1,
		SC2705_PAT_FAULT_M_MASK
		| SC2705_PAT_DONE_M_MASK, 0);
	if (ret)
		goto error_i2c;

	return 0;

error_i2c:
	dev_err(haptics->dev, "haptic init - I2C error : %d\n", ret);
	return ret;
}

static void sc2705_haptic_control(bool onoff, void *priv)
{
	struct sc2705_haptic *haptics;

	if (!priv)
		return;

	haptics = (struct sc2705_haptic *)priv;
	if (onoff) {
		dev_info(haptics->dev, "haptic on\n");
		haptics->magnitude = haptics->tuned_output;
	} else {
		dev_info(haptics->dev, "haptic off\n");
		haptics->magnitude = 0x0;
	}
	sc2705_haptic_enable(haptics);
}

static int sc2705_haptic_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sc2705_haptic *haptics;
	int ret;
	unsigned int period2Freq;

	haptics = devm_kzalloc(dev, sizeof(*haptics), GFP_KERNEL);
	if (!haptics)
		return -ENOMEM;
	haptics->dev = dev;

	dev_dbg(dev, "IRQ=%d\n", client->irq);
	if (!client->irq) {
		dev_err(dev, "No IRQ configured\n");
		return -EINVAL;
	}

	/* Handle DT data if provided */
	if (client->dev.of_node)
		sc2705_haptic_of_to_pdata(&client->dev, haptics);

	/* TODO: we've only tested DRO mode */
	if (haptics->op_mode == SC2705_H_PWM_MODE) {
		/* Get pwm and regulatot for haptics device */
		haptics->pwm_dev = devm_pwm_get(&client->dev, NULL);
		if (IS_ERR(haptics->pwm_dev) && !client->dev.of_node) {
			dev_err(haptics->dev, "unable to request PWM, trying legacy API\n");
			haptics->legacy = true;
			haptics->pwm_dev =
				pwm_request(haptics->pwm_id, "sc-haptic");
		}

		if (IS_ERR(haptics->pwm_dev)) {
			dev_err(dev, "unable to request PWM\n");
			return PTR_ERR(haptics->pwm_dev);
		}

		/* Check PWM Period, should be in 10k~250kHz */
		period2Freq = 1000000/haptics->pwm_dev->period;
		if (period2Freq < 10 || period2Freq > 250) {
			dev_err(dev, "Not supported PWM frequence\n");
			return -EINVAL;
		}
	}

	/* Initialize input device for haptic device */
	haptics->input_dev = devm_input_allocate_device(&client->dev);
	if (!haptics->input_dev) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	haptics->input_dev->name = "sc2705-haptic";
	haptics->input_dev->dev.parent = client->dev.parent;
	haptics->input_dev->open = sc2705_haptic_open;
	haptics->input_dev->close = sc2705_haptic_close;
	input_set_drvdata(haptics->input_dev, haptics);
	input_set_capability(haptics->input_dev, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(haptics->input_dev, NULL,
				sc2705_haptic_play);
	if (ret) {
		dev_err(dev, "failed to create force-feedback\n");
		return ret;
	}

	ret = input_register_device(haptics->input_dev);
	if (ret) {
		dev_err(dev, "failed to register input device\n");
		return ret;
	}

	INIT_WORK(&haptics->work, sc2705_haptic_work);
	haptics->client = client;
	i2c_set_clientdata(client, haptics);

	haptics->regmap = devm_regmap_init_i2c(client,
						&sc2705_haptic_regmap_config);
	if (IS_ERR(haptics->regmap)) {
		ret = PTR_ERR(haptics->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	ret = devm_request_threaded_irq(haptics->dev, client->irq, NULL,
				sc2705_haptic_irq_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
				"sc2705-haptics", haptics);
	if (ret != 0) {
		dev_err(dev,
			"Failed to request IRQ: %d\n", client->irq);
		return ret;
	}

	ret = sysfs_create_group(&dev->kobj, &sc2705_haptic_debug_attr_group);
	if (ret < 0) {
		dev_err(dev, "failed to create sysfs files\n");
		return ret;
	}
#if 0
	ret = sysfs_create_link(NULL, &dev->kobj, "haptic");
	if (ret < 0) {
		dev_err(dev, "failed to create sysfs link\n");
		return ret;
	}
#endif
	ret = timed_vibrator_register(sc2705_haptic_control, haptics);
	if (ret < 0) {
		dev_err(dev, "failed to register timed_vibrator driver\n");
		return ret;
	}

	ret = sc2705_haptic_init(haptics);
	if (ret)
		return ret;

	dev_info(dev, "Haptic successfully probed: %p\n", haptics);
	return 0;

	/* TODO: no free if error?? */
}

static int __maybe_unused sc2705_haptic_suspend(struct device *dev)
{
	struct sc2705_haptic *haptics = dev_get_drvdata(dev);
	int ret;

	if (haptics->suspend_state == false) {
		ret = regmap_update_bits(haptics->regmap,
			SC2705_TOP_CTL1, SC2705_PATTERN_INACTIVE_MODE_MASK, 0);
		if (ret)
			dev_err(haptics->dev, "I2C error : %d\n", ret);

		haptics->suspend_state = true;
	}
	return 0;
}

static int __maybe_unused sc2705_haptic_resume(struct device *dev)
{
	struct sc2705_haptic *haptics = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&haptics->input_dev->mutex);
	if (haptics->suspend_state) {
		ret = regmap_update_bits(haptics->regmap,
			SC2705_TOP_CTL1, SC2705_PATTERN_INACTIVE_MODE_MASK,
			SC2705_PATTERN_INACTIVE_MODE_MASK);
		if (ret)
			dev_err(haptics->dev, "I2C error : %d\n", ret);

		haptics->suspend_state = false;
	}
	mutex_unlock(&haptics->input_dev->mutex);
	return 0;
}

static const struct of_device_id sc2705_haptic_of_match[] = {
	{ .compatible = "sprd,sc2705-haptic", },
	{ }
};
MODULE_DEVICE_TABLE(of, sc2705_haptic_of_match);

static const struct i2c_device_id sc2705_haptic_i2c_id[] = {
	{ "sc2705-haptic", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc2705_lcd_i2c_id);

static SIMPLE_DEV_PM_OPS(sc2705_haptic_pm_ops,
		 sc2705_haptic_suspend, sc2705_haptic_resume);

static struct i2c_driver sc2705_haptic_driver = {
	.driver		= {
		.name	= "sc2705-haptic",
		.of_match_table = of_match_ptr(sc2705_haptic_of_match),
		.pm	= &sc2705_haptic_pm_ops,
	},
	.probe	= sc2705_haptic_probe,
	.id_table	= sc2705_haptic_i2c_id,
};
module_i2c_driver(sc2705_haptic_driver);

MODULE_DESCRIPTION("Haptic driver for SC2705");
MODULE_AUTHOR("Roy Im <Roy.Im.Opensource@diasemi.com>");
MODULE_LICENSE("GPL");
