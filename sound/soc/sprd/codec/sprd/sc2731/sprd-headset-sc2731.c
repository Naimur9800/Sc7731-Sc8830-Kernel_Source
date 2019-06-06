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
#include "sprd-asoc-debug.h"
#define pr_fmt(fmt) pr_sprd_fmt("HDST2731")""fmt

#include <asm/div64.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/iio/consumer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/notifier.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sprd_otp.h>
#include <linux/timer.h>

#include "sprd-codec.h"
#include "sprd-headset.h"

#define ENTER pr_debug("func: %s  line: %04d\n", __func__, __LINE__)

/* In kernel 3.18, the processing of the interrupt handler of
 * pmic eic changes to thread way, which cause the gap between
 * irq triggered and  eic level trigger settings must be less than
 * about 50ms. So the trigger settings must be located
 * in headset_irq_xx_handler().
 */
#define FOR_EIC_CHANGE_DET /* peng.lee debug for eic change */
#define FOR_EIC_CHANGE_BUTTON /* peng.lee debug for eic change */

#define EIC_AUD_HEAD_INST2 312
#define MAX_BUTTON_NUM 6
#define SPRD_HEADSET_JACK_MASK (SND_JACK_HEADSET)
#define SPRD_BUTTON_JACK_MASK (SND_JACK_BTN_0 | SND_JACK_BTN_1 | \
	SND_JACK_BTN_2 | SND_JACK_BTN_3 | SND_JACK_BTN_4)
#define PLUG_CONFIRM_COUNT (1)
#define NO_MIC_RETRY_COUNT (0)
#define ADC_READ_COUNT (3)
#define ADC_READ_LOOP (2)
#define ADC_GND (600)
/* summer debug for adc_get_average 50-->20 */
#define SCI_ADC_GET_VALUE_COUNT (20)

#define HID_CFG0 (0x0080)
#define HID_CFG2 (0x0088)
#define HID_CFG3 (0x008C)
#define HID_CFG4 (0x0090)

#define ABS(x) (((x) < (0)) ? (-(x)) : (x))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))

#define headset_reg_read(reg, val) \
	sci_adi_read(CODEC_REG((reg)), val)

#define headset_reg_write(reg, val, mask) \
	sci_adi_write(CODEC_REG((reg)), (val), (mask))

#define headset_reg_clr_bits(reg, bits) \
	sci_adi_clr(CODEC_REG((reg)), (bits))

#define headset_reg_set_bits(reg, bits) \
	sci_adi_set(CODEC_REG((reg)), (bits))

int dsp_fm_mute_by_set_dg(void)
	__attribute__ ((weak, alias("__dsp_fm_mute_by_set_dg")));

static int __dsp_fm_mute_by_set_dg(void)
{
	pr_err("ERR: dsp_fm_mute_by_set_dg is not defined!\n");
	return -1;
}

static inline int headset_reg_get_bits(unsigned int reg, int bits)
{
	unsigned int temp;
	int ret;

	ret = sci_adi_read(CODEC_REG(reg), &temp);
	if (ret) {
		pr_err("%s: read reg#%#x failed!\n", __func__, reg);
		return ret;
	}
	temp = temp & bits;

	return temp;
}

enum sprd_headset_type {
	HEADSET_4POLE_NORMAL,
	HEADSET_NO_MIC,
	HEADSET_4POLE_NOT_NORMAL,
	HEADSET_APPLE,
	HEADSET_TYPE_ERR = -1,
};

struct sprd_headset_auxadc_cal_l {
	u32 A;
	u32 B;
	u32 E;
	u32 cal_type;
};

#define SPRD_HEADSET_AUXADC_CAL_NO 0
#define SPRD_HEADSET_AUXADC_CAL_DO 1

static struct sprd_headset_auxadc_cal_l adc_cal_headset = {
	0, 0, 0, SPRD_HEADSET_AUXADC_CAL_NO,
};

static struct sprd_headset *sprd_hdst;

/* ========================  audio codec  ======================== */

int vbc_close_fm_dggain(bool mute)
	__attribute__ ((weak, alias("__vbc_close_fm_dggain")));
static int __vbc_close_fm_dggain(bool mute)
{
	pr_err("ERR: vbc_close_fm_dggain is not defined!\n");
	return -1;
}

static void headset_jack_report(struct sprd_headset *hdst,
	struct snd_soc_jack *jack, int status, int mask)
{
	snd_soc_jack_report(jack, status, mask);
}

static enum snd_jack_types headset_jack_type_get(int index)
{
	enum snd_jack_types jack_type_map[MAX_BUTTON_NUM] = {
		SND_JACK_BTN_0, SND_JACK_BTN_1, SND_JACK_BTN_2,
		SND_JACK_BTN_3, SND_JACK_BTN_4, SND_JACK_BTN_5
	};

	return jack_type_map[index];
}

static int sprd_headset_power_get(struct device *dev,
				  struct regulator **regu, const char *id)
{
	if (!*regu) {
		*regu = regulator_get(dev, id);
		if (IS_ERR(*regu)) {
			pr_err("ERR:Failed to request %ld: %s\n",
				PTR_ERR(*regu), id);
			*regu = 0;
			return PTR_ERR(*regu);
		}
	}

	return 0;
}

static int sprd_headset_power_init(struct sprd_headset *hdst)
{
	int ret = 0;
	struct platform_device *pdev = hdst->pdev;
	struct device *dev = NULL;
	struct sprd_headset_power *power = &hdst->power;

	if (!pdev) {
		pr_err("%s: codec is null!\n", __func__);
		return -1;
	}

	dev = &pdev->dev;
	ret = sprd_headset_power_get(dev, &power->head_mic, "HEADMICBIAS");
	if (ret || (power->head_mic == NULL)) {
		power->head_mic = 0;
		return ret;
	}
	regulator_set_voltage(power->head_mic, 950000, 950000);

	ret = sprd_headset_power_get(dev, &power->vcom_buf, "VCOM_BUF");
	if (ret) {
		power->vcom_buf = 0;
		goto __err1;
	}

	goto __ok;

__err1:
	regulator_put(power->head_mic);
__ok:
	return ret;
}

void sprd_headset_power_deinit(void)
{
	struct sprd_headset_power *power = &sprd_hdst->power;

	regulator_put(power->head_mic);
	regulator_put(power->vcom_buf);
}

static int sprd_headset_audio_block_is_running(struct sprd_headset *hdst)
{
	struct sprd_headset_power *power = &hdst->power;

	return regulator_is_enabled(power->vcom_buf);
}

static int sprd_headset_audio_headmic_sleep_disable(
	struct sprd_headset *hdst, int on)
{
	int ret = 0;
	struct sprd_headset_power *power = &hdst->power;

	if (!power->head_mic)
		return -1;

	if (on) {
		ret = regulator_set_mode(
			power->head_mic, REGULATOR_MODE_NORMAL);
	} else {
		if (sprd_headset_audio_block_is_running(hdst))
			ret = regulator_set_mode(
				power->head_mic, REGULATOR_MODE_NORMAL);
		else
			ret = regulator_set_mode(
				power->head_mic, REGULATOR_MODE_STANDBY);
	}

	return ret;
}

static int sprd_headset_headmic_bias_control(
	struct sprd_headset *hdst, int on)
{
	int ret = 0;
	struct sprd_headset_power *power = &hdst->power;

	if (!power->head_mic)
		return -1;

	if (on)
		ret = regulator_enable(power->head_mic);
	else
		ret = regulator_disable(power->head_mic);
	if (!ret) {
		/* Set HEADMIC_SLEEP when audio block closed */
		if (sprd_headset_audio_block_is_running(hdst))
			ret = regulator_set_mode(
				power->head_mic, REGULATOR_MODE_NORMAL);
		else
			ret = regulator_set_mode(
				power->head_mic, REGULATOR_MODE_STANDBY);
	}

	return ret;
}

static BLOCKING_NOTIFIER_HEAD(hp_chain_list);
int headset_register_notifier(struct notifier_block *nb)
{
	struct sprd_headset *hdst = sprd_hdst;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return 0;
	}

	if (pdata->jack_type == JACK_TYPE_NO) {
		nb = NULL;
		return 0;
	}

	return blocking_notifier_chain_register(&hp_chain_list, nb);
}
EXPORT_SYMBOL(headset_register_notifier);

int headset_unregister_notifier(struct notifier_block *nb)
{
	if (nb == NULL)
		return -1;

	return blocking_notifier_chain_unregister(&hp_chain_list, nb);
}
EXPORT_SYMBOL(headset_unregister_notifier);

#if 0
static int hp_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&hp_chain_list, val, NULL)
		== NOTIFY_BAD) ? -EINVAL : 0;
}
#endif

int headset_get_plug_state(void)
{
	struct sprd_headset *hdst = sprd_hdst;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return 0;
	}

	return !!hdst->plug_stat_last;
}
EXPORT_SYMBOL(headset_get_plug_state);
/* ========================  audio codec  ======================== */

static int headset_wrap_sci_adc_get(struct iio_channel *chan)
{
	int count = 0;
	int average = 0;
	int val, ret = 0;
	struct sprd_headset *hdst = sprd_hdst;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return 0;
	}

	if (!chan) {
		pr_err("%s: iio_channel is NULL\n", __func__);
		return 0;
	}

	if (hdst->pdata.jack_type == JACK_TYPE_NO) {
		headset_reg_set_bits(ANA_HDT0, BIT(BUT_DET_PD));
		usleep_range(2000, 4000);
	}

	while (count < SCI_ADC_GET_VALUE_COUNT) {
		ret = iio_read_channel_raw(chan, &val);
		if (ret < 0) {
			pr_err("%s: read adc raw value failed!\n", __func__);
			return 0;
		}
		average += val;
		count++;
	}

	if (hdst->pdata.jack_type == JACK_TYPE_NO)
		headset_reg_clr_bits(ANA_HDT0, BIT(BUT_DET_PD));

	average /= SCI_ADC_GET_VALUE_COUNT;

	pr_debug("%s: adc: %d\n", __func__, average);

	return average;
}

/*  on = 0: open headmic detect circuit */
static void headset_detect_circuit(unsigned on)
{
	if (on) {
		headset_reg_clr_bits(ANA_HDT0, BIT(HEAD_INS_PD));
		headset_reg_clr_bits(ANA_HDT0, BIT(BUT_DET_PD));
	} else {
		headset_reg_set_bits(ANA_HDT0, BIT(HEAD_INS_PD));
		headset_reg_set_bits(ANA_HDT0, BIT(BUT_DET_PD));
	}
}

static void headset_detect_clk_en(void)
{
	sci_adi_set(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_AUD_EN);
	headset_reg_set_bits(AUD_CFGA_CLK_EN,
		BIT(CLK_AUD_HID_EN) | BIT(CLK_AUD_HBD_EN));
}

static void headset_detect_init(void)
{
	unsigned long msk, val;

	headset_detect_clk_en();
	/* set headset detect voltage */
	msk = HEAD_SDET_MASK << HEAD_SDET;
	val = HEAD_SDET_2P5 << HEAD_SDET;
	headset_reg_write(ANA_HDT0, val, msk);
	msk = HEAD_INS_VREF_MASK << HEAD_INS_VREF;
	val = HEAD_INS_VREF_2P0 << HEAD_INS_VREF;
	headset_reg_write(ANA_HDT0, val, msk);
}

static void headmic_sleep_disable(struct sprd_headset *hdst, int on);

static void headset_adc_en(int en)
{
	struct sprd_headset *hdst = sprd_hdst;
	unsigned long msk, val;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return;
	}

	msk = V2AD_SEL_MASK << V2AD_SEL;
	if (en) {
		headset_reg_set_bits(ANA_HDT3, BIT(V2AD_EN));
		val = V2AD_SEL_HEADMIC_IN_DET << V2AD_SEL;
		headset_reg_write(ANA_HDT3, val, msk);
		headmic_sleep_disable(hdst, 1);
	} else {
		if (hdst->debug_level <= 2) {
			headset_reg_clr_bits(ANA_HDT3, BIT(V2AD_EN));
			val = V2AD_SEL_DISABLE << V2AD_SEL;
			headset_reg_write(ANA_HDT3, val, msk);
		}
		headmic_sleep_disable(hdst, 0);
	}
}

/* is_set = 1, headset_mic to AUXADC */
static void headset_set_adc_to_headmic(unsigned is_set)
{
	unsigned long msk, val;

	headset_adc_en(1);

	msk = V2AD_SEL_MASK << V2AD_SEL;
	if (is_set)
		val = V2AD_SEL_HEADMIC_IN_DET << V2AD_SEL;
	else
		val = V2AD_SEL_HEADSET_L_INT << V2AD_SEL;
	headset_reg_write(ANA_HDT3, val, msk);
}

static void headset_button_irq_threshold(int enable)
{
	struct sprd_headset *hdst = sprd_hdst;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);
	int audio_head_sbut = 0;
	unsigned long msk, val;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return;
	}

	audio_head_sbut = pdata->irq_threshold_button;
	msk = HEAD_SBUT_MASK << HEAD_SBUT;
	val = enable ? audio_head_sbut << HEAD_SBUT : 0xf;
	headset_reg_write(ANA_HDT0, val, msk);
}

static void headset_irq_button_enable(int enable, unsigned int irq)
{
	static int current_irq_state = 1;
	struct sprd_headset *hdst = sprd_hdst;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return;
	}

	mutex_lock(&hdst->irq_btn_lock);
	if (enable == 1) {
		if (current_irq_state == 0) {
			enable_irq(irq);
			current_irq_state = 1;
		}
	} else {
		if (current_irq_state == 1) {
			disable_irq_nosync(irq);
			current_irq_state = 0;
		}
	}
	mutex_unlock(&hdst->irq_btn_lock);
}

static void headset_irq_detect_enable(int enable, unsigned int irq)
{
	/* irq is enabled after request_irq() */
	static int current_irq_state = 1;
	struct sprd_headset *hdst = sprd_hdst;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return;
	}

	mutex_lock(&hdst->irq_det_lock);
	if (enable == 1) {
		if (current_irq_state == 0) {
			enable_irq(irq);
			current_irq_state = 1;
		}
	} else {
		if (current_irq_state == 1) {
			disable_irq_nosync(irq);
			current_irq_state = 0;
		}
	}
	mutex_unlock(&hdst->irq_det_lock);
}

static void headset_irq_detect_mic_enable(int enable, unsigned int irq)
{
	static int current_irq_state = 1;
	struct sprd_headset *hdst = sprd_hdst;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return;
	}

	mutex_lock(&hdst->irq_det_mic_lock);
	if (enable == 1) {
		if (current_irq_state == 0) {
			enable_irq(irq);
			current_irq_state = 1;
		}
	} else {
		if (current_irq_state == 1) {
			disable_irq_nosync(irq);
			current_irq_state = 0;
		}
	}
	mutex_unlock(&hdst->irq_det_mic_lock);
}

static void headmic_sleep_disable(struct sprd_headset *hdst, int on)
{
	static int current_power_state;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return;
	}

	if (on == 1) {
		if (current_power_state == 0) {
			sprd_headset_audio_headmic_sleep_disable(hdst, 1);
			current_power_state = 1;
		}
	} else {
		if (current_power_state == 1) {
			sprd_headset_audio_headmic_sleep_disable(hdst, 0);
			current_power_state = 0;
		}
	}
}

static void headmicbias_power_on(struct sprd_headset *hdst, int on)
{
	static int current_power_state;
	struct sprd_headset_platform_data *pdata;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return;
	}

	pdata = &hdst->pdata;

	if (on == 1) {
		if (current_power_state == 0) {
			if (pdata->external_headmicbias_power_on != NULL)
				pdata->external_headmicbias_power_on(1);
			sprd_headset_headmic_bias_control(hdst, 1);
			current_power_state = 1;
		}
	} else {
		if (current_power_state == 1) {
			if (pdata->external_headmicbias_power_on != NULL)
				pdata->external_headmicbias_power_on(0);
			sprd_headset_headmic_bias_control(hdst, 0);
			current_power_state = 0;
		}
	}
}

static int headset_sts0_confirm(u32 jack_type)
{
	if (headset_reg_get_bits(ANA_STS0, BIT(HEAD_INSERT)) == 0) {
		if (jack_type == JACK_TYPE_NC)
			return 0;
		else
			return 1;
	} else {
		return 1;
	}
}

static int headset_adc_compare(int x1, int x2)
{
	int delta = 0;
	int max = 0;

	if ((x1 < 300) && (x2 < 300))
		return 1;

	x1 = ((x1 == 0) ? 1 : (x1*100));
	x2 = ((x2 == 0) ? 1 : (x2*100));

	delta = ABS(x1-x2);
	max = MAX(x1, x2);

	if (delta < ((max*10)/100))
		return 1;
	else
		return 0;
}

static int headset_accumulate_adc(int *adc, u32 jack_type,
				  struct iio_channel *chan,
				  int gpio_num, int gpio_value)
{
	int k;
	int j;
	int success = 1;

	adc[0] = headset_wrap_sci_adc_get(chan);
	for (j = 0; j < ADC_READ_COUNT-1; j++) {
		if (jack_type != JACK_TYPE_NO &&
		    gpio_get_value(gpio_num) != gpio_value) {
			pr_warn("gpio value changed!!! the adc read operation aborted (step3)\n");
			return -1;
		}

		if (headset_sts0_confirm(jack_type) == 0) {
			pr_warn("headset_sts0_confirm failed!!! the adc read operation aborted (step4)\n");
			return -1;
		}

		adc[j+1] = headset_wrap_sci_adc_get(chan);

		if (headset_adc_compare(adc[j], adc[j+1]) == 0) {
			success = 0;
			for (k = 0; k <= j+1; k++)
				pr_debug("adc[%d] = %d\n", k, adc[k]);
			break;
		}
	}

	return success;
}

static int headset_cal_adc_average(int *adc, u32 jack_type,
				   int gpio_num, int gpio_value)
{
	int i;
	int adc_average = 0;

	if (gpio_get_value(gpio_num) != gpio_value) {
		pr_warn("gpio value changed!!! the adc read operation aborted (step5)\n");
		return -1;
	}

	if (headset_sts0_confirm(jack_type) == 0) {
		pr_warn("headset_sts0_confirm failed!!! the adc read operation aborted (step6)\n");
		return -1;
	}

	for (i = 0; i < ADC_READ_COUNT; i++) {
		adc_average += adc[i];
		pr_debug("adc[%d] = %d\n", i, adc[i]);
	}
	adc_average = adc_average / ADC_READ_COUNT;
	pr_debug("%s success, adc_average = %d\n", __func__, adc_average);

	return adc_average;
}

static int headset_get_adc_average(struct iio_channel *chan,
				   int gpio_num, int gpio_value)
{
	int i = 0;
	int success = 1;
	int adc[ADC_READ_COUNT] = {0};
	struct sprd_headset *hdst = sprd_hdst;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);
	u32 jack_type;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return -1;
	}

	jack_type = pdata->jack_type;

	/* ================= debug ===================== */
	if (hdst->debug_level >= 3) {
		int count = 0;
		int adc_val = 0;
		int gpio_button = 0;
		int ana_pmu0 = 0;
		int ana_cfg1 = 0;
		int ana_hdt0 = 0;
		int ana_sts0 = 0;

		while (count < 20) {
			count++;
			adc_val = headset_wrap_sci_adc_get(chan);
			gpio_button = gpio_get_value(
						pdata->gpios[HDST_GPIO_BUTTON]);
			headset_reg_read(ANA_PMU0, &ana_pmu0);
			headset_reg_read(ANA_HDT0, &ana_hdt0);
			headset_reg_read(ANA_STS0, &ana_sts0);

			pr_info("%2d:  gpio_button=%d  adc=%4d  ana_pmu0=0x%08X  ana_cfg1=0x%08X  ana_hdt0=0x%08X  ana_sts0=0x%08X\n",
				count, gpio_button, adc_val, ana_pmu0,
				ana_cfg1, ana_hdt0, ana_sts0);
			usleep_range(800, 1200);
		}
	}
	/* ================= debug ===================== */

	usleep_range(800, 1200);
	for (i = 0; i < ADC_READ_LOOP; i++) {
		if (gpio_get_value(gpio_num) != gpio_value) {
			pr_warn("gpio value changed!!! the adc read operation aborted (step1)\n");
			return -1;
		}
		if (headset_sts0_confirm(jack_type) == 0) {
			pr_warn("headset_sts0_confirm failed!!! the adc read operation aborted (step2)\n");
			return -1;
		}

		success = headset_accumulate_adc(adc, jack_type, chan, gpio_num,
						 gpio_value);
		if (success < 0)
			return success;

		if (success == 1) {
			usleep_range(2800, 3600);
			return headset_cal_adc_average(adc, jack_type, gpio_num,
						       gpio_value);
		} else if (i+1 < ADC_READ_LOOP) {
			pr_info("%s failed, retrying count = %d\n",
				__func__, i+1);
			usleep_range(800, 1200);
		}
	}
	pr_info("%s out\n", __func__);

	return -1;
}

static int headset_irq_set_irq_type(unsigned int irq, unsigned int type)
{
	struct sprd_headset *hdst = sprd_hdst;
	struct irq_desc *irq_desc = NULL;
	unsigned int irq_flags = 0;
	int ret = -1;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return -1;
	}

	ret = irq_set_irq_type(irq, type);
	irq_desc = irq_to_desc(irq);
	irq_flags = irq_desc->action->flags;

	if (irq == hdst->irq_button) {
		if (type == IRQF_TRIGGER_HIGH) {
			pr_debug("IRQF_TRIGGER_HIGH is set for irq_button(%d). irq_flags = 0x%08X, ret = %d\n",
				  hdst->irq_button, irq_flags, ret);
		} else if (type == IRQF_TRIGGER_LOW) {
			pr_debug("IRQF_TRIGGER_LOW is set for irq_button(%d). irq_flags = 0x%08X, ret = %d\n",
				  hdst->irq_button, irq_flags, ret);
		}
	} else if (irq == hdst->irq_detect) {
		if (type == IRQF_TRIGGER_HIGH) {
			pr_debug("IRQF_TRIGGER_HIGH is set for irq_detect(%d). irq_flags = 0x%08X, ret = %d\n",
				  hdst->irq_detect, irq_flags, ret);
		} else if (type == IRQF_TRIGGER_LOW) {
			pr_debug("IRQF_TRIGGER_LOW is set for irq_detect(%d). irq_flags = 0x%08X, ret = %d\n",
				  hdst->irq_detect, irq_flags, ret);
		}
	}

	return 0;
}

static int headset_button_valid(int gpio_detect_value_current)
{
	struct sprd_headset *hdst = sprd_hdst;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);
	int button_is_valid = 0;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return -1;
	}

	if (pdata->irq_trigger_levels[HDST_GPIO_DET_L] == 1) {
		if (gpio_detect_value_current == 1)
			button_is_valid = 1;
		else
			button_is_valid = 0;
	} else {
		if (gpio_detect_value_current == 0)
			button_is_valid = 1;
		else
			button_is_valid = 0;
	}

	return button_is_valid;
}

static int headset_gpio_2_button_state(int gpio_button_value_current)
{
	struct sprd_headset *hdst = sprd_hdst;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);
	int button_state_current = 0;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return -1;
	}

	if (pdata->irq_trigger_levels[HDST_GPIO_BUTTON] == 1) {
		if (gpio_button_value_current == 1)
			button_state_current = 1;
		else
			button_state_current = 0;
	} else {
		if (gpio_button_value_current == 0)
			button_state_current = 1;
		else
			button_state_current = 0;
	}

	return button_state_current; /* 0==released, 1==pressed */
}

static int headset_plug_confirm_by_adc(struct iio_channel *chan,
	int last_gpio_detect_value)
{
	struct sprd_headset *hdst = sprd_hdst;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);
	int adc_last = 0;
	int adc_current = 0;
	int count = 0;
	int adc_read_interval = 10;

	if (!hdst || !chan) {
		pr_err("%s: sprd_hdset(%p) or chan(%p) is NULL!\n",
			__func__, hdst, chan);
		return -1;
	}

	adc_last = headset_get_adc_average(chan, pdata->gpios[HDST_GPIO_DET_L],
		last_gpio_detect_value);
	if (-1 == adc_last) {
		pr_info("headset_plug_confirm_by_adc failed!!!\n");
		return -1;
	}

	while (count < PLUG_CONFIRM_COUNT) {
		msleep(adc_read_interval);
		adc_current = headset_get_adc_average(
			chan, pdata->gpios[HDST_GPIO_DET_L],
			last_gpio_detect_value);
		if (-1 == adc_current) {
			pr_info("headset_plug_confirm_by_adc failed!!!\n");
			return -1;
		}
		if (headset_adc_compare(adc_last, adc_current) == 0) {
			pr_info("headset_plug_confirm_by_adc failed!!!\n");
			return -1;
		}
		adc_last = adc_current;
		count++;
	}
	pr_info("headset_plug_confirm_by_adc success!!!\n");

	return adc_current;
}

static int headset_adc_get_ideal(u32 adc_mic);

static int headset_read_adc_repeatable(struct iio_channel *chan,
	int last_gpio_detect_value)
{
	int retrytimes = 0;
	int adc_value = 0;

	do {
		adc_value = headset_plug_confirm_by_adc(chan,
							last_gpio_detect_value);
		retrytimes++;
	} while ((-1 == adc_value) && (retrytimes < 10));
	pr_info("%s : adc_value  %d retrytimes is %d\n",
		__func__, adc_value, retrytimes);

	return adc_value;
}

static enum sprd_headset_type headset_type_detect(int last_gpio_detect_value)
{
	struct sprd_headset *hdst = sprd_hdst;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);
	int adc_mic_average = 0;
	int adc_left_average = 0;
	int adc_mic_ideal = 0;
	int no_mic_retry_count = NO_MIC_RETRY_COUNT;
	struct iio_channel *adc_chan = hdst->adc_chan;

	ENTER;

	if (!hdst || !adc_chan) {
		pr_err("%s: sprd_hdset(%p) or adc_chan(%p) is NULL!\n",
			__func__, hdst, adc_chan);
		return HEADSET_TYPE_ERR;
	}

	if (pdata->gpio_switch != 0)
		gpio_direction_output(pdata->gpio_switch, 0);
	else
		pr_info("automatic type switch is unsupported\n");

	headset_button_irq_threshold(1);
	headset_detect_init();
	headset_detect_circuit(1);

no_mic_retry:

	/* get adc value of left */
	headset_set_adc_to_headmic(0);
	msleep(100);
	adc_left_average = headset_read_adc_repeatable(
		adc_chan, last_gpio_detect_value);
	if (adc_left_average == -1) {
		headset_adc_en(0);
		return HEADSET_TYPE_ERR;
	}

	/* Get adc value of headmic in. */
	headset_set_adc_to_headmic(1);
	msleep(50);
	adc_mic_average = headset_read_adc_repeatable(
		adc_chan, last_gpio_detect_value);
	headset_adc_en(0);
	if (-1 == adc_mic_average)
		return HEADSET_TYPE_ERR;
	adc_mic_ideal = headset_adc_get_ideal(adc_mic_average);
	if (adc_mic_ideal >= 0)
		adc_mic_average = adc_mic_ideal;
	pr_info("adc average, mic = %d, left = %d\n",
		adc_mic_average, adc_left_average);

	if ((gpio_get_value(pdata->gpios[HDST_GPIO_DET_L])) !=
	    last_gpio_detect_value) {
		pr_info("software debance (gpio check)!!!(headset_type_detect)\n");
		return HEADSET_TYPE_ERR;
	}

	if (headset_sts0_confirm(pdata->jack_type) == 0) {
		pr_info("software debance (headset_sts0_confirm)!!!(headset_type_detect)\n");
		return HEADSET_TYPE_ERR;
	}

	if (adc_mic_average < pdata->adc_threshold_3pole_detect) {
		if (no_mic_retry_count != 0) {
			pr_info("no_mic_retry\n");
			no_mic_retry_count--;
			goto no_mic_retry;
		}
		return HEADSET_NO_MIC;
	} else if ((adc_left_average < ADC_GND) && (adc_mic_average > ADC_GND))
		return HEADSET_4POLE_NORMAL;
	else if ((adc_left_average > ADC_GND) && (adc_mic_average > ADC_GND)
		&& (ABS(adc_mic_average - adc_left_average) < ADC_GND))
		return HEADSET_4POLE_NOT_NORMAL;
	else
		return HEADSET_TYPE_ERR;

	return HEADSET_TYPE_ERR;
}

static void headset_button_release_verify(void)
{
	struct sprd_headset *hdst = sprd_hdst;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return;
	}

	if (hdst->btn_stat_last == 1) {
		headset_jack_report(hdst, &hdst->btn_jack,
			0, hdst->btns_pressed);
		hdst->btn_stat_last = 0;
		pr_info("headset button released by force!!! current button: %#x\n",
			hdst->btns_pressed);
		hdst->btns_pressed &= ~SPRD_BUTTON_JACK_MASK;
		if (hdst->pdata.irq_trigger_levels[HDST_GPIO_BUTTON] == 1)
			headset_irq_set_irq_type(
				hdst->irq_button, IRQF_TRIGGER_HIGH);
		else
			headset_irq_set_irq_type(
				hdst->irq_button, IRQF_TRIGGER_LOW);
	}
}

static enum snd_jack_types headset_adc_to_button(int adc_mic)
{
	int i;
	struct sprd_headset *hdst = sprd_hdst;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);
	struct headset_buttons *hdst_btns =
		(pdata ? pdata->headset_buttons : NULL);
	int nb = (pdata ? pdata->nbuttons : 0);
	enum snd_jack_types j_type = KEY_RESERVED;

	if (!hdst || !hdst_btns) {
		pr_err("%s: sprd_hdst(%p) or hdst_btns(%p) is NULL!\n",
			__func__, sprd_hdst, hdst_btns);
		return KEY_RESERVED;
	}

	for (i = 0; i < nb; i++) {
		if (adc_mic >= hdst_btns[i].adc_min &&
			adc_mic < hdst_btns[i].adc_max) {
			j_type = headset_jack_type_get(i);
			break;
		}
	}

	return j_type;
}

static void headset_button_work_func(struct work_struct *work)
{
	struct sprd_headset *hdst = sprd_hdst;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);
	int gpio_button_value_current = 0;
	int button_state_current = 0;
	int adc_mic_average = 0;
	int adc_ideal = 0;
	struct iio_channel *chan = hdst->adc_chan;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return;
	}

	down(&hdst->sem);
	headset_set_adc_to_headmic(1);
	ENTER;

	gpio_button_value_current =
		gpio_get_value(pdata->gpios[HDST_GPIO_BUTTON]);
	if (gpio_button_value_current != hdst->gpio_btn_val_last) {
		pr_info("software debance (step 1: gpio check)\n");
		goto out;
	}

	button_state_current =
		headset_gpio_2_button_state(gpio_button_value_current);

	if (button_state_current == 1) {/* pressed! */
		if (pdata->nbuttons > 0) {
			adc_mic_average = headset_get_adc_average(chan,
				pdata->gpios[HDST_GPIO_BUTTON],
				hdst->gpio_btn_val_last);
			if (-1 == adc_mic_average) {
				pr_info("software debance (step 3: adc check)!!!(headset_button_work_func)(pressed)\n");
				goto out;
			}
			adc_ideal = headset_adc_get_ideal(adc_mic_average);
			pr_info("adc_mic_average=%d, adc_ideal=%d\n",
				adc_mic_average, adc_ideal);
			if (adc_ideal >= 0)
				adc_mic_average = adc_ideal;
			pr_info("adc_mic_average = %d\n", adc_mic_average);
			hdst->btns_pressed |=
				headset_adc_to_button(adc_mic_average);
		}

		if (hdst->btn_stat_last == 0) {
			headset_jack_report(hdst, &hdst->btn_jack,
				hdst->btns_pressed, hdst->btns_pressed);
			hdst->btn_stat_last = 1;
			pr_info("Reporting headset button press. button: 0x%#x\n",
				hdst->btns_pressed);
		} else {
			pr_err("Headset button has been reported already. button: 0x%#x\n",
				hdst->btns_pressed);
		}

		if (pdata->irq_trigger_levels[HDST_GPIO_BUTTON] == 1)
			headset_irq_set_irq_type(
				hdst->irq_button, IRQF_TRIGGER_LOW);
		else
			headset_irq_set_irq_type(
				hdst->irq_button, IRQF_TRIGGER_HIGH);

		headset_irq_button_enable(1, hdst->irq_button);
	} else { /* released! */
		if (hdst->btn_stat_last == 1) {
			headset_jack_report(hdst, &hdst->btn_jack,
				0, hdst->btns_pressed);
			hdst->btn_stat_last = 0;
			pr_info("Reporting headset button release. button: %#x\n",
				hdst->btns_pressed);
		} else
			pr_err("Headset button has been released already. button: %#x\n",
				hdst->btns_pressed);

		if (pdata->irq_trigger_levels[HDST_GPIO_BUTTON] == 1)
			headset_irq_set_irq_type(
				hdst->irq_button, IRQF_TRIGGER_HIGH);
		else
			headset_irq_set_irq_type(
				hdst->irq_button, IRQF_TRIGGER_LOW);

		headset_irq_button_enable(1, hdst->irq_button);
		hdst->btns_pressed &= ~SPRD_BUTTON_JACK_MASK;
	}
out:
	headset_adc_en(0);
	headset_irq_button_enable(1, hdst->irq_button);
	/* wake_unlock(&hdst->btn_wakelock); */
	up(&hdst->sem);
}

static void headset_process_for_4pole(enum sprd_headset_type headset_type)
{
	struct sprd_headset *hdst = sprd_hdst;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);
	int irq_type;

	if (!pdata) {
		pr_err("%s: pdata is NULL!\n", __func__);
		return;
	}

	headset_button_irq_threshold(1);

	if ((headset_type == HEADSET_4POLE_NOT_NORMAL)
	    && (pdata->gpio_switch == 0)) {
		headset_irq_button_enable(0, hdst->irq_button);
		pr_info("HEADSET_4POLE_NOT_NORMAL is not supported %s\n",
			"by your hardware! so disable the button irq!");
	} else {
		irq_type = pdata->irq_trigger_levels[HDST_GPIO_BUTTON] ?
			IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
		headset_irq_set_irq_type(hdst->irq_button, irq_type);

		headset_irq_button_enable(1, hdst->irq_button);
	}

	hdst->hdst_status = SND_JACK_HEADSET;
	headset_jack_report(hdst, &hdst->hdst_jack,
		hdst->hdst_status, SND_JACK_HEADSET);
}

static void headset_detect_work_func(struct work_struct *work)
{
	struct sprd_headset *hdst = sprd_hdst;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);
	struct sprd_headset_power *power = (hdst ? &hdst->power : NULL);
	enum sprd_headset_type headset_type;
	int plug_state_current = 0;
	int gpio_detect_value_current = 0;
#ifndef FOR_EIC_CHANGE_DET
	int irq_type;
#endif

	down(&hdst->sem);

	ENTER;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return;
	}

	if (power->head_mic == NULL)
		sprd_headset_power_init(hdst);
	if (power->head_mic == NULL) {
		pr_info("sprd_headset_power_init fail 0\n");
		goto out;
	}

	if (hdst->plug_stat_last == 0) {
		/* schedule_timeout_uninterruptible(msecs_to_jiffies(10)); */
		headmicbias_power_on(hdst, 1);
	}

	if (hdst->plug_stat_last == 0) {
		gpio_detect_value_current =
			gpio_get_value(pdata->gpios[HDST_GPIO_DET_L]);
		pr_info("gpio_detect_value_current = %d, gpio_detect_value_last = %d, plug_state_last = %d\n",
			gpio_detect_value_current, hdst->gpio_det_val_last,
			hdst->plug_stat_last);

		if (gpio_detect_value_current != hdst->gpio_det_val_last) {
			pr_info("software debance (step 1)!!!(headset_detect_work_func)\n");
			goto out;
		}

		if (pdata->irq_trigger_levels[HDST_GPIO_DET_L] == 1) {
			if (gpio_detect_value_current == 1)
				plug_state_current = 1;
			else
				plug_state_current = 0;
		} else {
			if (gpio_detect_value_current == 0)
				plug_state_current = 1;
			else
				plug_state_current = 0;
		}
	} else
		plug_state_current = 0;/* no debounce for plug out!!! */

	if (1 == plug_state_current && 0 == hdst->plug_stat_last) {
		if (pdata->do_fm_mute)
			vbc_close_fm_dggain(false);

		headset_type = headset_type_detect(hdst->gpio_det_val_last);
		switch (headset_type) {
		case HEADSET_TYPE_ERR:
			hdst->det_err_cnt++;
			pr_info("headset_type = %d detect_err_count = %d(HEADSET_TYPE_ERR)\n",
				headset_type, hdst->det_err_cnt);
			goto out;
		case HEADSET_4POLE_NOT_NORMAL:
			pr_info("headset_type = %d (HEADSET_4POLE_NOT_NORMAL)\n",
				headset_type);
			if (pdata->gpio_switch != 0)
				gpio_direction_output(pdata->gpio_switch, 1);
			break;
		case HEADSET_4POLE_NORMAL:
			pr_info("headset_type = %d (HEADSET_4POLE_NORMAL)\n",
				headset_type);
			if (pdata->gpio_switch != 0)
				gpio_direction_output(pdata->gpio_switch, 0);
			break;
		case HEADSET_NO_MIC:
			pr_info("headset_type = %d (HEADSET_NO_MIC)\n",
				headset_type);
			if (pdata->gpio_switch != 0)
				gpio_direction_output(pdata->gpio_switch, 0);
			break;
		case HEADSET_APPLE:
			pr_info("headset_type = %d (HEADSET_APPLE)\n",
				headset_type);
			pr_info("we have not yet implemented this in the code\n");
			break;
		default:
			pr_info("headset_type = %d (HEADSET_UNKNOWN)\n",
				headset_type);
			break;
		}

		hdst->det_err_cnt = 0;
		if (headset_type == HEADSET_NO_MIC ||
				headset_type == HEADSET_4POLE_NOT_NORMAL)
			hdst->headphone = 1;
		else
			hdst->headphone = 0;

		if (hdst->headphone) {
			headset_button_irq_threshold(0);
			headset_irq_button_enable(0, hdst->irq_button);

			hdst->hdst_status = SND_JACK_HEADPHONE;
			headset_jack_report(hdst, &hdst->hdst_jack,
				hdst->hdst_status, SND_JACK_HEADPHONE);
			pr_info("headphone plug in (headset_detect_work_func)\n");
		} else {
			headset_process_for_4pole(headset_type);
			pr_info("headset plug in (headset_detect_work_func)\n");
		}

		hdst->plug_stat_last = 1;
		if (pdata->jack_type == JACK_TYPE_NC) {
			headset_irq_detect_enable(0, hdst->irq_detect);
			headset_irq_detect_mic_enable(1, hdst->irq_detect_mic);
		} else {
#ifndef FOR_EIC_CHANGE_DET
			irq_type = pdata->irq_trigger_levels[HDST_GPIO_DET_L] ==
				1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
			headset_irq_set_irq_type(hdst->irq_detect, irq_type);
#endif
			headset_irq_detect_enable(1, hdst->irq_detect);
		}
	} else if (0 == plug_state_current && 1 == hdst->plug_stat_last) {

		/* close the fm advanced because of noise when playing fm
		 * in speaker mode plugging out headset
		 * vbc_close_fm_dggain() for vbc-r2p0
		 * dsp_fm_mute_by_set_dg() for vbc-r3p0
		 */
		if (pdata->do_fm_mute) {
			vbc_close_fm_dggain(true);
			dsp_fm_mute_by_set_dg();
		}

		headset_irq_button_enable(0, hdst->irq_button);
		headset_button_release_verify();

		if (hdst->headphone)
			pr_info("headphone plug out (%s)\n", __func__);
		else
			pr_info("headset plug out (%s)\n", __func__);

		/* hp_notifier_call_chain(hdst->type); */
		hdst->hdst_status &= ~SPRD_HEADSET_JACK_MASK;
		headset_jack_report(hdst, &hdst->hdst_jack,
			0, SPRD_HEADSET_JACK_MASK);
		hdst->plug_stat_last = 0;
		if (pdata->jack_type == JACK_TYPE_NC) {
			headset_irq_detect_mic_enable(0, hdst->irq_detect_mic);
		} else {
#ifndef FOR_EIC_CHANGE_DET
			irq_type = pdata->irq_trigger_levels[HDST_GPIO_DET_L] ==
				1 ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
			headset_irq_set_irq_type(hdst->irq_detect, irq_type);
#endif
		}
		headset_irq_detect_enable(1, hdst->irq_detect);
	} else {
		pr_info("irq_detect must be enabled anyway!!!\n");
		goto out;
	}
out:
	headset_reg_set_bits(ANA_HDT0, BIT(HEAD_L_INT_PU_PD));

	if (hdst->plug_stat_last == 0) {
		if (pdata->jack_type == JACK_TYPE_NC) {
			headset_irq_detect_mic_enable(0, hdst->irq_detect_mic);
			if (hdst->det_err_cnt < 500)
				headset_irq_detect_enable(1, hdst->irq_detect);
		}
		headmicbias_power_on(hdst, 0);
	}

	if (pdata->jack_type == JACK_TYPE_NC) {
		if (hdst->plug_stat_last == 1) {
			headset_irq_detect_mic_enable(1, hdst->irq_detect_mic);
			headset_irq_detect_enable(0, hdst->irq_detect);
		}
	} else {
		headset_irq_detect_enable(1, hdst->irq_detect);
	}
	/* wake_unlock(&hdst->det_wakelock); */
	up(&hdst->sem);
}

static void headset_reg_dump_func(struct work_struct *work)
{
	int adc_mic = 0;

	int gpio_detect = 0;
	int gpio_button = 0;
	int gpio_detect_mic = 0;

	int ana_sts0 = 0;
	int ana_pmu0 = 0;
	int ana_cfg1 = 0;
	int ana_hdt0 = 0;

	int hid_cfg0 = 0;
	int hid_cfg2 = 0;
	int hid_cfg3 = 0;
	int hid_cfg4 = 0;

	int arm_module_en = 0;
	int arm_clk_en = 0;

	struct sprd_headset *hdst = sprd_hdst;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return;
	}

	adc_mic = headset_wrap_sci_adc_get(hdst->adc_chan);

	gpio_detect = gpio_get_value(pdata->gpios[HDST_GPIO_DET_L]);
	gpio_button = gpio_get_value(pdata->gpios[HDST_GPIO_BUTTON]);
	if (pdata->jack_type == JACK_TYPE_NC)
		gpio_detect_mic = gpio_get_value(
			pdata->gpios[HDST_GPIO_DET_MIC]);

	sci_adi_write(ANA_REG_GLB_ARM_MODULE_EN,
		BIT_ANA_AUD_EN, BIT_ANA_AUD_EN);
	headset_reg_read(ANA_PMU0, &ana_pmu0);
	headset_reg_read(ANA_HDT0, &ana_hdt0);
	headset_reg_read(ANA_STS0, &ana_sts0);

	headset_reg_set_bits(AUD_CFGA_CLK_EN, BIT(CLK_AUD_HID_EN));

	headset_reg_read(AUD_CFGA_HID_CFG2, &hid_cfg2);
	headset_reg_read(AUD_CFGA_HID_CFG3, &hid_cfg3);
	headset_reg_read(AUD_CFGA_HID_CFG4, &hid_cfg4);
	headset_reg_read(AUD_CFGA_HID_CFG0, &hid_cfg0);

	sci_adi_read(ANA_REG_GLB_ARM_MODULE_EN, &arm_module_en);
	sci_adi_read(ANA_REG_GLB_ARM_CLK_EN, &arm_clk_en);

	pr_info("GPIO_%03d(det)=%d GPIO_%03d(but)=%d GPIO_%03d(det_mic)=%d adc_mic=%d\n",
		pdata->gpios[HDST_GPIO_DET_L], gpio_detect,
		pdata->gpios[HDST_GPIO_BUTTON], gpio_button,
		pdata->gpios[HDST_GPIO_DET_MIC], gpio_detect_mic,
		adc_mic);
	pr_info("arm_module_en|arm_clk_en|ana_pmu0  |ana_cfg1  |ana_hdt0 |ana_sts0  |hid_cfg2  |hid_cfg3  |hid_cfg4  |hid_cfg0\n");
	pr_info("0x%08X|0x%08X|0x%08X|0x%08X|0x%08X|0x%08X|0x%08X|0x%08X|0x%08X|0x%08X\n",
		arm_module_en, arm_clk_en, ana_pmu0, ana_cfg1, ana_hdt0,
		ana_sts0, hid_cfg2, hid_cfg3, hid_cfg4, hid_cfg0);

	if (hdst->debug_level >= 2)
		queue_delayed_work(hdst->reg_dump_work_q,
			&hdst->reg_dump_work, msecs_to_jiffies(500));
}

static irqreturn_t headset_button_irq_handler(int irq, void *dev)
{
	struct sprd_headset *hdst = dev;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);
	int button_state_current = 0;
	int gpio_button_value_current = 0;
	int val;

	pr_info("headset_button_irq_handler in\n");
	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return IRQ_HANDLED;
	}

	if (headset_button_valid(
	    gpio_get_value(pdata->gpios[HDST_GPIO_DET_L])) == 0) {
		headset_reg_read(ANA_STS0, &val);
		pr_info("%s: button is invalid!!! IRQ_%d(GPIO_%d) = %d, ANA_STS0 = 0x%08X\n",
			__func__, hdst->irq_button,
			pdata->gpios[HDST_GPIO_BUTTON],
			hdst->gpio_btn_val_last, val);

		if (hdst->plug_stat_last == 0)
			headset_irq_button_enable(0, hdst->irq_button);

		return IRQ_HANDLED;
	}

	gpio_button_value_current =
		gpio_get_value(pdata->gpios[HDST_GPIO_BUTTON]);
	button_state_current =
		headset_gpio_2_button_state(gpio_button_value_current);
	if (button_state_current == hdst->btn_stat_last) {
		pr_info("button state check failed!!! maybe the release is too quick. button_state_current=%d, hdst->btn_stat_last=%d\n",
			button_state_current, hdst->btn_stat_last);
		return IRQ_HANDLED;
	}

	headset_irq_button_enable(0, hdst->irq_button);
	wake_lock_timeout(&hdst->btn_wakelock, msecs_to_jiffies(2000));
	hdst->gpio_btn_val_last = gpio_button_value_current;

	headset_reg_read(ANA_STS0, &val);
	pr_debug("%s: IRQ_%d(GPIO_%d) = %d, ANA_STS0 = 0x%08X\n",
		__func__, hdst->irq_button, pdata->gpios[HDST_GPIO_BUTTON],
		hdst->gpio_btn_val_last, val);
	pr_info("headset_button_irq_handler out\n");
	queue_work(hdst->btn_work_q, &hdst->btn_work);

	return IRQ_HANDLED;
}

static irqreturn_t headset_detect_irq_handler(int irq, void *dev)
{
	struct sprd_headset *hdst = dev;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);
	unsigned int val;
	int irq_type;

	pr_info("headset_detect_irq_handler in\n");
	headset_irq_button_enable(0, hdst->irq_button);
	headset_irq_detect_enable(0, hdst->irq_detect);
	if (pdata->jack_type == JACK_TYPE_NC)
		headset_irq_detect_mic_enable(0, hdst->irq_detect_mic);
	wake_lock_timeout(&hdst->det_wakelock, msecs_to_jiffies(2000));
	hdst->gpio_det_val_last = gpio_get_value(pdata->gpios[HDST_GPIO_DET_L]);
	headset_reg_read(ANA_STS0, &val);
	pr_debug("headset_detect_irq_handler: IRQ_%d(GPIO_%d) = %d, ANA_STS0 = 0x%08X\n",
		hdst->irq_detect, pdata->gpios[HDST_GPIO_DET_L],
		hdst->gpio_det_val_last, val);

#ifdef FOR_EIC_CHANGE_DET /* peng.lee debug eic change */
	if (pdata->jack_type != JACK_TYPE_NC) {
		if (hdst->gpio_det_val_last == 1) {
			irq_type = pdata->irq_trigger_levels[HDST_GPIO_DET_L] ==
				1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
		} else {
			irq_type = pdata->irq_trigger_levels[HDST_GPIO_DET_L] ==
				1 ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
		}
		headset_irq_set_irq_type(hdst->irq_detect, irq_type);
	}
#endif
	queue_delayed_work(hdst->det_work_q,
		&hdst->det_work, msecs_to_jiffies(0));
	pr_info("headset_detect_irq_handler out\n");

	return IRQ_HANDLED;
}

static irqreturn_t headset_detect_mic_irq_handler(int irq, void *dev)
{
	struct sprd_headset *hdst = dev;

	pr_info("headset_detect_mic_irq_handler in\n");
	if (hdst->plug_stat_last != 1) {
		headset_irq_detect_mic_enable(0, hdst->irq_detect_mic);
		pr_info("headset_detect_mic_irq_handler out0\n");
		return IRQ_HANDLED;
	}

	headset_irq_detect_mic_enable(0, hdst->irq_detect_mic);
	headset_irq_detect_enable(0, hdst->irq_detect);
	wake_lock_timeout(&hdst->det_wakelock, msecs_to_jiffies(2000));
	queue_delayed_work(hdst->det_work_q,
		&hdst->det_work, msecs_to_jiffies(0));
	pr_info("headset_detect_mic_irq_handler out1\n");

	return IRQ_HANDLED;
}

/* ================= create sys fs for debug =================== */
 /* Used for getting headset type in sysfs. */
static ssize_t headset_state_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buff)
{
	struct sprd_headset *hdst = sprd_hdst;
	int type = 0;

	switch (hdst->hdst_status) {
	case SND_JACK_HEADSET:
		type = 1;
		break;
	case SND_JACK_HEADPHONE:
		type = 2;
		break;
	default:
		type = 0;
		break;
	}

	pr_debug("%s status: %#x, headset_state = %d\n",
		__func__, hdst->hdst_status, type);

	return sprintf(buff, "%d\n", type);
}

static ssize_t headset_state_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buff, size_t len)
{
	return len;
}
/* ============= /sys/kernel/headset/debug_level =============== */

static ssize_t headset_debug_level_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buff)
{
	struct sprd_headset *hdst = sprd_hdst;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return -1;
	}

	pr_info("debug_level = %d\n", hdst->debug_level);

	return sprintf(buff, "%d\n", hdst->debug_level);
}

static ssize_t headset_debug_level_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buff, size_t len)
{
	struct sprd_headset *hdst = sprd_hdst;
	unsigned long level;
	int ret;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return -1;
	}

	ret = kstrtoul(buff, 10, &level);
	if (ret) {
		pr_err("%s kstrtoul failed!(%d)\n", __func__, ret);
		return len;
	}
	hdst->debug_level = level;
	pr_info("debug_level = %d\n", hdst->debug_level);
	if (hdst->debug_level >= 2)
		queue_delayed_work(hdst->reg_dump_work_q,
			&hdst->reg_dump_work, msecs_to_jiffies(500));

	return len;
}

static int headset_debug_sysfs_init(void)
{
	int ret = -1;
	int i;
	static struct kobject *headset_debug_kobj;
	static struct kobj_attribute headset_debug_attr[] = {
		__ATTR(debug_level, 0644,
		headset_debug_level_show,
		headset_debug_level_store),
		__ATTR(state, 0644,
		headset_state_show,
		headset_state_store),
	};

	headset_debug_kobj = kobject_create_and_add("headset", kernel_kobj);
	if (headset_debug_kobj == NULL) {
		ret = -ENOMEM;
		pr_err("register sysfs failed. ret = %d\n", ret);
		return ret;
	}

	for (i = 0; i < sizeof(headset_debug_attr) /
	     sizeof(headset_debug_attr[0]); i++) {
		ret = sysfs_create_file(headset_debug_kobj,
					&headset_debug_attr[i].attr);
		if (ret) {
			pr_err("create sysfs '%s' failed. ret = %d\n",
			       headset_debug_attr[i].attr.name, ret);
			return ret;
		}
	}

	pr_info("headset_debug_sysfs_init success\n");

	return ret;
}
/* ================= create sys fs for debug =================== */

void sprd_headset_set_global_variables(
	struct sprd_headset_global_vars *glb)
{
	arch_audio_codec_set_regmap(glb->regmap);
	arch_audio_codec_set_reg_offset(glb->codec_reg_offset);
}

static void headset_adc_cal_from_efuse(void);
int sprd_headset_soc_probe(struct snd_soc_codec *codec)
{
	int ret = 0, i;
	struct sprd_headset *hdst = sprd_hdst;
	struct sprd_headset_platform_data *pdata = (hdst ? &hdst->pdata : NULL);
	struct device *dev = codec->dev; /* digiatal part device */
	unsigned int adie_chip_id = 0;
	unsigned long irqflags = 0;
	unsigned int val;
	unsigned long msk, value;
	struct snd_soc_card *card = codec->component.card;

	if (!hdst) {
		pr_err("%s: sprd_hdset is NULL!\n", __func__);
		return -1;
	}

	sci_adi_set(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_AUD_EN);
	headset_reg_set_bits(ANA_HDT0, BIT(HEAD_L_INT_PU_PD));
	headset_reg_read(ANA_HDT0, &val);
	pr_info("ANA_HDT1 0x%x\n", val);

	msk = HEDET_INTRES_SEL_MASK << HEDET_INTRES_SEL;
	value = HEDET_INTRES_SEL_30K << HEDET_INTRES_SEL;
	headset_reg_write(ANA_HDT6, value, msk);
	headset_reg_read(ANA_HDT6, &val);
	pr_info("ANA_HDT6 0x%x\n", val);

	adie_chip_id = sci_get_ana_chip_id();
	pr_info("adie chip is 0x%x, 0x%x\n", (adie_chip_id >> 16) & 0xFFFF,
	adie_chip_id & 0xFFFF);

	headset_detect_init();

	ret = sprd_headset_power_init(hdst);
	if (ret) {
		pr_err("sprd_headset_power_init failed\n");
		return ret;
	}

	ret = snd_soc_card_jack_new(card, "Headset Jack",
		SPRD_HEADSET_JACK_MASK, &hdst->hdst_jack, NULL, 0);
	if (ret) {
		pr_err("Failed to create headset jack\n");
		return ret;
	}

	ret = snd_soc_card_jack_new(card, "Headset Keyboard",
		SPRD_BUTTON_JACK_MASK, &hdst->btn_jack, NULL, 0);
	if (ret) {
		pr_err("Failed to create button jack\n");
		return ret;
	}

	if (pdata->nbuttons > MAX_BUTTON_NUM) {
		pr_warn("button number in dts is more than %d!\n",
			MAX_BUTTON_NUM);
		pdata->nbuttons = MAX_BUTTON_NUM;
	}
	for (i = 0; i < pdata->nbuttons; i++) {
		struct headset_buttons *buttons =
			&pdata->headset_buttons[i];

		ret = snd_jack_set_key(hdst->btn_jack.jack,
			headset_jack_type_get(i), buttons->code);
		if (ret) {
			pr_err("%s: Failed to set code for btn-%d\n",
				__func__, i);
			return ret;
		}
	}

	if (pdata->gpio_switch != 0)
		gpio_direction_output(pdata->gpio_switch, 0);
	gpio_direction_input(pdata->gpios[HDST_GPIO_DET_L]);
	gpio_direction_input(pdata->gpios[HDST_GPIO_BUTTON]);
	hdst->irq_detect = gpio_to_irq(pdata->gpios[HDST_GPIO_DET_L]);
	hdst->irq_button = gpio_to_irq(pdata->gpios[HDST_GPIO_BUTTON]);
	if (pdata->jack_type == JACK_TYPE_NC) {
		gpio_direction_input(pdata->gpios[HDST_GPIO_DET_MIC]);
		hdst->irq_detect_mic =
			gpio_to_irq(pdata->gpios[HDST_GPIO_DET_MIC]);
	}

	sema_init(&hdst->sem, 1);

	INIT_WORK(&hdst->btn_work, headset_button_work_func);
	hdst->btn_work_q = create_singlethread_workqueue("headset_button");
	if (hdst->btn_work_q == NULL) {
		pr_err("create_singlethread_workqueue for headset_button failed!\n");
		goto failed_to_micbias_power_off;
	}

	INIT_DELAYED_WORK(&hdst->det_work, headset_detect_work_func);
	hdst->det_work_q = create_singlethread_workqueue("headset_detect");
	if (hdst->det_work_q == NULL) {
		pr_err("create_singlethread_workqueue for headset_detect failed!\n");
		goto failed_to_headset_detect;
	}

	INIT_DELAYED_WORK(&hdst->reg_dump_work, headset_reg_dump_func);
	hdst->reg_dump_work_q =
		create_singlethread_workqueue("headset_reg_dump");
	if (hdst->reg_dump_work_q == NULL) {
		pr_err("create_singlethread_workqueue for headset_reg_dump failed!\n");
		goto failed_to_headset_reg_dump;
	}
	if (hdst->debug_level >= 2)
		queue_delayed_work(hdst->reg_dump_work_q,
			&hdst->reg_dump_work, msecs_to_jiffies(500));

	wake_lock_init(&hdst->det_wakelock,
		WAKE_LOCK_SUSPEND, "headset_detect_wakelock");
	wake_lock_init(&hdst->btn_wakelock,
		WAKE_LOCK_SUSPEND, "headset_button_wakelock");

	mutex_init(&hdst->irq_btn_lock);
	mutex_init(&hdst->irq_det_lock);
	mutex_init(&hdst->irq_det_mic_lock);

	for (i = 0; i < HDST_GPIO_MAX; i++) {
		if (pdata->jack_type == JACK_TYPE_NC) {
			if (i == HDST_GPIO_DET_MIC)
				continue;
		}
		gpio_set_debounce(pdata->gpios[i], pdata->dbnc_times[i] * 1000);
	}

	irqflags = pdata->irq_trigger_levels[HDST_GPIO_BUTTON] ?
		IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
	ret = devm_request_threaded_irq(
		dev, hdst->irq_button, NULL, headset_button_irq_handler,
		irqflags | IRQF_NO_SUSPEND, "headset_button", hdst);
	if (ret) {
		pr_err("failed to request IRQ_%d(GPIO_%d)\n",
			hdst->irq_button, pdata->gpios[HDST_GPIO_BUTTON]);
		goto failed_to_request_irq;
	}
	/* Disable button irq before headset detected. */
	headset_irq_button_enable(0, hdst->irq_button);

	irqflags = pdata->irq_trigger_levels[HDST_GPIO_DET_L] ?
		IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
	ret = devm_request_threaded_irq(
		dev, hdst->irq_detect, NULL, headset_detect_irq_handler,
		irqflags | IRQF_NO_SUSPEND, "headset_detect", hdst);
	if (ret < 0) {
		pr_err("failed to request IRQ_%d(GPIO_%d)\n",
			hdst->irq_detect, pdata->gpios[HDST_GPIO_DET_L]);
		goto failed_to_request_detect_irq;
	}

	if (pdata->jack_type == JACK_TYPE_NC) {
		irqflags = pdata->irq_trigger_levels[HDST_GPIO_DET_MIC] ?
			IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
		ret = devm_request_threaded_irq(dev, hdst->irq_detect_mic, NULL,
			headset_detect_mic_irq_handler,
			irqflags | IRQF_NO_SUSPEND, "headset_detect_mic", hdst);
		if (ret < 0) {
			pr_err("failed to request IRQ_%d(GPIO_%d)\n",
				hdst->irq_detect_mic,
				pdata->gpios[HDST_GPIO_DET_MIC]);
			goto failed_to_request_detect_mic_irq;
		}
	}
	headset_debug_sysfs_init();
	headset_adc_cal_from_efuse();

	return 0;

failed_to_request_detect_mic_irq:
	devm_free_irq(dev, hdst->irq_detect, hdst);
failed_to_request_detect_irq:
	devm_free_irq(dev, hdst->irq_button, hdst);
failed_to_request_irq:
	cancel_delayed_work_sync(&hdst->reg_dump_work);
	destroy_workqueue(hdst->reg_dump_work_q);
failed_to_headset_reg_dump:
	destroy_workqueue(hdst->det_work_q);
failed_to_headset_detect:
	destroy_workqueue(hdst->btn_work_q);
failed_to_micbias_power_off:
	headmicbias_power_on(hdst, 0);

	return ret;
}

#ifdef CONFIG_OF
static int sprd_headset_parse_dt(struct sprd_headset *hdst)
{
	int ret = 0;
	struct sprd_headset_platform_data *pdata;
	struct device_node *np, *buttons_np = NULL;
	struct headset_buttons *buttons_data;
	u32 val = 0;
	int index;

	if (!hdst) {
		pr_err("%s sprd_hdst is NULL!\n", __func__);
		return -EINVAL;
	}

	np = hdst->pdev->dev.of_node;
	if (!np) {
		pr_err("%s No device node for headset!\n", __func__);
		return -ENODEV;
	}

	/* Parse configs for headset & button detecting. */
	pdata = &hdst->pdata;
	ret = of_property_read_u32(np, "jack-type", &val);
	if (ret < 0) {
		pr_err("%s: parse 'jack-type' failed!\n", __func__);
		pdata->jack_type = JACK_TYPE_NO;
	}
	pdata->jack_type = val ? JACK_TYPE_NC : JACK_TYPE_NO;

	/* Parse gpios. */
	/* Parse for the gpio of EU/US jack type switch. */
	index = of_property_match_string(np, "gpio-names", "switch");
	if (index < 0) {
		pr_info("%s :no match found for switch gpio.\n", __func__);
		pdata->gpio_switch = 0;
	} else {
		ret = of_get_gpio_flags(np, index, NULL);
		if (ret < 0) {
			pr_err("%s :get gpio for 'switch' failed!\n", __func__);
			return -ENXIO;
		}
		pdata->gpio_switch = (u32)ret;
	}

	/* Parse for button detecting gpio. */
	index = of_property_match_string(np, "gpio-names", "button");
	if (index < 0) {
		pr_err("%s :no match found for button gpio\n", __func__);
		return -ENXIO;
	}
	ret = of_get_gpio_flags(np, index, NULL);
	if (ret < 0) {
		pr_err("%s :get gpio for 'button' failed!\n", __func__);
		return -ENXIO;
	}
	pdata->gpios[HDST_GPIO_BUTTON] = (u32)ret;
	ret = of_property_read_u32_index(
		np, "gpio-trigger-levels", index, &val);
	if (ret < 0) {
		pr_err("%s :get trigger level for 'button' failed!\n",
			__func__);
		return ret;
	}
	pdata->irq_trigger_levels[HDST_GPIO_BUTTON] = val;
	ret = of_property_read_u32_index(
		np, "gpio-dbnc-intervals", index, &val);
	if (ret < 0) {
		pr_err("%s :get debounce inteval for 'button' failed!\n",
			__func__);
		return ret;
	}
	pdata->dbnc_times[HDST_GPIO_BUTTON] = val;
	pr_debug("use GPIO_%u for button detecting, trigr level: %u, debounce: %u\n",
		pdata->gpios[HDST_GPIO_BUTTON],
		pdata->irq_trigger_levels[HDST_GPIO_BUTTON],
		pdata->dbnc_times[HDST_GPIO_BUTTON]);

	/* Parse for left pin detecting gpio. */
	index = of_property_match_string(np, "gpio-names", "detect_l");
	if (index < 0) {
		pr_err("%s :no match found for detect_l gpio\n", __func__);
		return -ENXIO;
	}
	ret = of_get_gpio_flags(np, index, NULL);
	if (ret < 0) {
		pr_err("%s :get gpio for 'detect_l' failed!\n", __func__);
		return -ENXIO;
	}
	pdata->gpios[HDST_GPIO_DET_L] = (u32)ret;
	ret = of_property_read_u32_index(
		np, "gpio-trigger-levels", index, &val);
	if (ret < 0) {
		pr_err("%s :get trigger level for 'detect_l' failed!\n",
			__func__);
		return ret;
	}
	pdata->irq_trigger_levels[HDST_GPIO_DET_L] = val;
	ret = of_property_read_u32_index(
		np, "gpio-dbnc-intervals", index, &val);
	if (ret < 0) {
		pr_err("%s :get debounce inteval for 'detect_l' failed!\n",
			__func__);
		return ret;
	}
	pdata->dbnc_times[HDST_GPIO_DET_L] = val;
	pr_debug("use GPIO_%u for insert detecting, trigr level: %u, debounce: %u\n",
		pdata->gpios[HDST_GPIO_DET_L],
		pdata->irq_trigger_levels[HDST_GPIO_DET_L],
		pdata->dbnc_times[HDST_GPIO_DET_L]);

	/* Parse for mic pin detecting gpio. */
	if (pdata->jack_type == JACK_TYPE_NC) {
		index = of_property_match_string(
			np, "gpio-names", "detect_mic");
		if (index < 0) {
			pr_err("%s :no match found for detect_mic gpio\n",
				__func__);
			return -ENXIO;
		}
		ret = of_get_gpio_flags(np, index, NULL);
		if (ret < 0) {
			pr_err("%s :get gpio for 'detect_mic' failed!\n",
				__func__);
			return -ENXIO;
		}
		pdata->gpios[HDST_GPIO_DET_MIC] = (u32)ret;
		ret = of_property_read_u32_index(
			np, "gpio-trigger-levels", index, &val);
		if (ret < 0) {
			pr_err("%s :get trigger level for 'detect_mic' failed!\n",
				__func__);
			return ret;
		}
		pdata->irq_trigger_levels[HDST_GPIO_DET_MIC] = val;
		ret = of_property_read_u32_index(
			np, "gpio-dbnc-intervals", index, &val);
		if (ret < 0) {
			pr_err("%s :get debounce inteval for 'detect_mic' failed!\n",
				__func__);
			return ret;
		}
		pdata->dbnc_times[HDST_GPIO_DET_MIC] = val;
		pr_debug("use GPIO_%u for mic detecting, trig level: %u, debounce: %u\n",
			pdata->gpios[HDST_GPIO_DET_MIC],
			pdata->irq_trigger_levels[HDST_GPIO_DET_MIC],
			pdata->dbnc_times[HDST_GPIO_DET_MIC]);
	}

	ret = of_property_read_u32(np, "adc-threshold-3pole-detect",
		&pdata->adc_threshold_3pole_detect);
	if (ret) {
		pr_err("%s: fail to get adc-threshold-3pole-detect\n",
			__func__);
		return -ENXIO;
	}

	ret = of_property_read_u32(
		np, "irq-threshold-button", &pdata->irq_threshold_button);
	if (ret) {
		pr_err("%s: fail to get irq-threshold-button\n", __func__);
		return -ENXIO;
	}

	pdata->do_fm_mute = !of_property_read_bool(np, "sprd,no-fm-mute");

	/* Parse for buttons */
	pdata->nbuttons = of_get_child_count(np);
	buttons_data = devm_kzalloc(&hdst->pdev->dev,
		pdata->nbuttons*sizeof(*buttons_data), GFP_KERNEL);
	if (!buttons_data)
		return -ENOMEM;
	pdata->headset_buttons = buttons_data;

	for_each_child_of_node(np, buttons_np) {
		ret = of_property_read_u32(
			buttons_np, "adc-min", &buttons_data->adc_min);
		if (ret) {
			pr_err("%s: fail to get adc-min\n", __func__);
			return ret;
		}
		ret = of_property_read_u32(
			buttons_np, "adc-max", &buttons_data->adc_max);
		if (ret) {
			pr_err("%s: fail to get adc-min\n", __func__);
			return ret;
		}
		ret = of_property_read_u32(
			buttons_np, "code", &buttons_data->code);
		if (ret) {
			pr_err("%s: fail to get code\n", __func__);
			return ret;
		}
		pr_info("device tree data: adc_min = %d adc_max = %d code = %d\n",
			buttons_data->adc_min,
			buttons_data->adc_max, buttons_data->code);
		buttons_data++;
	};

	return 0;
}
#endif

/* Note: @pdev is the platform_device of headset node in dts. */
int sprd_headset_probe(struct platform_device *pdev)
{
	struct sprd_headset *hdst = NULL;
	struct sprd_headset_platform_data *pdata;
	int ret = -1;
	struct device *dev = (pdev ? &pdev->dev : NULL);

#ifndef CONFIG_OF
	pr_err("%s: Only OF configurations are supported yet!\n", __func__);
	return -1;
#endif
	if (!pdev) {
		pr_err("%s: platform device is NULL!\n", __func__);
		return -1;
	}

	hdst = devm_kzalloc(dev, sizeof(*hdst), GFP_KERNEL);
	if (!hdst)
		return -ENOMEM;

	hdst->pdev = pdev;
	ret = sprd_headset_parse_dt(hdst);
	if (ret < 0) {
		pr_err("Failed to parse dt for headset.\n");
		return ret;
	}
	pdata = &hdst->pdata;

	if (pdata->gpio_switch != 0) {
		ret = devm_gpio_request(dev,
			pdata->gpio_switch, "headset_switch");
		if (ret < 0) {
			pr_err("failed to request GPIO_%d(headset_switch)\n",
				pdata->gpio_switch);
			return ret;
		}
	} else
		pr_info("automatic EU/US type switch is unsupported\n");

	ret = devm_gpio_request(dev,
		pdata->gpios[HDST_GPIO_DET_L], "headset_detect");
	if (ret < 0) {
		pr_err("failed to request GPIO_%d(headset_detect)\n",
			pdata->gpios[HDST_GPIO_DET_L]);
		return ret;
	}
	ret = devm_gpio_request(dev,
		pdata->gpios[HDST_GPIO_BUTTON], "headset_button");
	if (ret < 0) {
		pr_err("failed to request GPIO_%d(headset_button)\n",
			pdata->gpios[HDST_GPIO_BUTTON]);
		return ret;
	}

	if (pdata->jack_type == JACK_TYPE_NC) {
		ret = devm_gpio_request(dev,
			pdata->gpios[HDST_GPIO_DET_MIC], "headset_detect_mic");
		if (ret < 0) {
			pr_err("failed to request GPIO_%d(headset_detect_mic)\n",
				pdata->gpios[HDST_GPIO_DET_MIC]);
			return ret;
		}
	}

	/* Get the adc channels of headset. */
	hdst->adc_chan = iio_channel_get(dev, "headset");
	if (IS_ERR(hdst->adc_chan)) {
		pr_err("failed to get headmic in adc channel!\n");
		return PTR_ERR(hdst->adc_chan);
	}

	sprd_hdst = hdst;

	pr_info("headset_detect_probe success\n");

	return 0;
}
EXPORT_SYMBOL(sprd_headset_probe);

static int headset_adc_get_ideal(u32 adc_mic)
{
	u64 numerator = 0;
	u32 denominator = 0;
	u32 adc_ideal = 0;
	u32 a, b, e;

	if (adc_cal_headset.cal_type != SPRD_HEADSET_AUXADC_CAL_DO) {
		pr_warn("efuse A,B,E hasn't been calculated!\n");
		return adc_mic;
	}

	a = adc_cal_headset.A;
	b = adc_cal_headset.B;
	e = adc_cal_headset.E;

	pr_debug("wangzuo headset_adc_get_ideal in\n");

	if (9*adc_mic + b < 10*a)
		return adc_mic;

	denominator = e*(b-a);
	numerator = 939836*(9*(u64)adc_mic+(u64)b-10*(u64)a);
	pr_debug("denominator=%u, numerator=%llu\n", denominator, numerator);
	do_div(numerator, denominator);
	adc_ideal = (u32)numerator;
	pr_info("adc_mic=%d, adc_ideal=%d\n", adc_mic, adc_ideal);

	return adc_ideal;
}

#define DELTA1_BLOCK20 20
#define DELTA2_BLOCK22 22
#define BITCOUNT 16
#define BLK_WIDTH 16
#define PROTECT_BIT (0)
static void headset_adc_cal_from_efuse(void)
{
	u32 delta[2] = {0};
	u32 block0_bit7 = 128;
	u8 test[3] = {0};

	pr_info("to get calibration data from efuse ...\n");
	if (adc_cal_headset.cal_type != SPRD_HEADSET_AUXADC_CAL_NO) {
		pr_info("efuse A,B,E has been calculated already!\n");
		return;
	}

	delta[0] =
		sprd_pmic_efuse_bits_read(DELTA1_BLOCK20*BLK_WIDTH, BITCOUNT);
	delta[1] =
		sprd_pmic_efuse_bits_read(DELTA2_BLOCK22*BLK_WIDTH, BITCOUNT);

	test[0] = delta[0]&0xff;
	test[1] = (delta[0] >> 8) & 0xff;
	test[2] = delta[1]&0xff;
	pr_info("test[0] 0x%x %d, test[1] 0x%x %d, test[2] 0x%x %d\n",
		test[0], test[0], test[1], test[1], test[2], test[2]);

	block0_bit7 = sprd_pmic_efuse_bits_read(0, BITCOUNT);
	pr_info("block_7 0x%08x\n", block0_bit7);
#if 0
	if (!(block0_bit7&(1<<PROTECT_BIT))) {
		pr_info("block 0 bit 7 set 1 no efuse data\n");
		return;
	}
#endif
	adc_cal_headset.cal_type = SPRD_HEADSET_AUXADC_CAL_DO;
	pr_info("block 0 bit 7 set 0 have efuse data\n");

	pr_info("test[0] %d, test[1] %d, test[2] %d\n",
		test[0], test[1], test[2]);

	adc_cal_headset.A = (test[0]*4)-176;
	adc_cal_headset.B = (test[1]*4)+2840;
	adc_cal_headset.E = (test[2]*2)+2500;
	pr_info("adc_cal_headset.A %d, adc_cal_headset.B %d, adc_cal_headset.E %d\n",
		adc_cal_headset.A, adc_cal_headset.B, adc_cal_headset.E);
}

MODULE_DESCRIPTION("headset & button detect driver v2");
MODULE_AUTHOR("Yaochuan Li <yaochuan.li@spreadtrum.com>");
MODULE_LICENSE("GPL");
