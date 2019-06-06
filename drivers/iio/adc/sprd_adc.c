/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
 * Author: steve.zhan <steve.zhan@spreadtrum.com>
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
#include <linux/hwspinlock.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sort.h>
#include <linux/sprd_otp.h>
#include <linux/sprd_hwspinlock.h>
#include <linux/spi/spi-sprd-adi.h>
#include "sprd_adc.h"

#define SC2731_ADC0_SELECT_BIT	(BIT(11))
#define ADC_CTL		(0x00)
#define ADC_SW_CH_CFG		(0x04)
#define ADC_FAST_HW_CHX_CFG(_X_)		((_X_) * 0x4 + 0x8)
#define ADC_SLOW_HW_CHX_CFG(_X_)		((_X_) * 0x4 + 0x28)
#define ADC_HW_CH_DELAY		(0x48)
#define ADC_DAT		(0x4c)
#define ADC_IRQ_EN		(0x50)
#define ADC_IRQ_CLR		(0x54)
#define ADC_IRQ_STS		(0x58)
#define ADC_IRQ_RAW		(0x5c)
#define ADC_DEBUG		(0x60)

#define ADC_MAX_SAMPLE_NUM			(0x10)
#define BIT_SW_CH_RUN_NUM(_X_)		((((_X_) - 1) & 0xf) << 4)

/* 0: adc in 10bits mode, 1: adc in 12bits mode */

#define BIT_ADC_BIT_MODE(_X_)		(((_X_) & 0x1) << 2)
#define BIT_ADC_BIT_MODE_MASK		(BIT_ADC_BIT_MODE(1))
#define BIT_SW_CH_ON                (BIT(1))
#define BIT_EN_ADC                  (BIT(0))

/* 0: resistance path, 1: capacitance path */

#define BIT_CH_IN_MODE(_X_)			(((_X_) & 0x1) << 8)

/* 0: quick mode, 1: slow mode */

#define BIT_CH_SLOW(_X_)			(((_X_) & 0x1) << 6)

/* 0: little scale, 1: big scale */

#define BIT_CH_SCALE(_X_)			(((_X_) & 0x1) << 5)
#define BIT_CH_ID(_X_)				((_X_) & 0x1f)

/* 0:disable, 1:enable */

#define BIT_CH_DLY_EN(_X_)			(((_X_) & 0x1) << 7)

/* its unit is ADC clock */

#define BIT_HW_CH_DELAY(_X_)		((_X_) & 0xff)

#define SPRD_MODULE_NAME	"sprd_adc"
#define MEASURE_TIMES		(15)
#define ADC_MESURE_NUMBER	15
#define RATIO(_n_, _d_) (_n_ << 16 | _d_)
#define DIV_ROUND(n, d)		(((n) + ((d)/2)) / (d))
#define ADC_DROP_CNT		(DIV_ROUND(MEASURE_TIMES, 5))
#define BITSINDEX(b, o)	((b) * pmic_adc->info->para->blk_width + (o))

#define HWSPINLOCK_TIMEOUT		(5000)
static DEFINE_MUTEX(adc_mutex);

static struct sprd_pmic_adc *pmic_adc;
static u32 ana_chip_id;
static int cal_type = SPRD_AUXADC_CAL_NO;

static struct sprd_adc_cal big_scale_cal = {
	4200, 3310,
	3600, 2832,
	SPRD_AUXADC_CAL_NO,
};

static struct sprd_adc_cal small_sclae_cal = {
	1000, 3413,
	100, 341,
	SPRD_AUXADC_CAL_NO,
};

struct sprd_chip_info {
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	const struct pmic_adc_para *para;
};

static inline int pmic_adc_reg_read(unsigned long reg)
{
	unsigned int val;

	regmap_read(pmic_adc->regmap, pmic_adc->adc_base + reg, &val);
	return val;
}

static inline int pmic_adc_reg_write(unsigned long reg, unsigned long val)
{
	return regmap_write(pmic_adc->regmap, pmic_adc->adc_base + reg, val);
}

static inline int pmic_adc_reg_set(unsigned long reg, unsigned int mask,
				   unsigned int bits)
{
	return regmap_update_bits(pmic_adc->regmap, reg, mask, bits);
}

static inline int pmic_adc_reg_clr(unsigned long reg, unsigned int mask,
				   unsigned int bits)
{
	return regmap_update_bits(pmic_adc->regmap, reg, mask, ~bits);
}

static int pmic_adc_lock(void)
{
	int ret = 0;

	mutex_lock(&adc_mutex);
	ret =
	    hwspin_lock_no_swlock_timeout(pmic_adc->hw_lock,
					  HWSPINLOCK_TIMEOUT);
	if (ret) {
		pr_err("pmic adc:lock the hwlock failed.\n");
		mutex_unlock(&adc_mutex);
		return ret;
	}
	return ret;
}

static void pmic_adc_unlock(void)
{
	hwspin_unlock_no_swlock(pmic_adc->hw_lock);
	mutex_unlock(&adc_mutex);
}

static const struct of_device_id pmic_adc_of_match[] = {
	{.compatible = "sprd,sc2723-adc", .data = (void *)SC2723_ADC,},
	{.compatible = "sprd,sc2731-adc", .data = (void *)SC2731_ADC,},
	{.compatible = "sprd,sc2721-adc", .data = (void *)SC2721_ADC,},
	{}
};

#if defined(CONFIG_OTP_SPRD_PMIC_EFUSE)
static int sprd_get_adc_small_scale_cal(unsigned int *p_cal_data)
{
	unsigned int deta = 0;
	unsigned short adc_data = 0;
	const struct pmic_adc_para *para = pmic_adc->info->para;

	deta =
	    sprd_pmic_efuse_bits_read(BITSINDEX(para->small_cal_efuse_blk, 0),
				      para->small_cal_efuse_bit);
	pr_info("get pmic efuse block %u, deta: 0x%08x\n",
		para->small_cal_efuse_blk, deta);
	deta &= 0xFFFFFF;
	if ((!deta) || (p_cal_data == NULL))
		return -1;

	if (pmic_adc->type == SC2723_ADC) {
		/* adc 0.1V */
		adc_data =
		    ((deta & 0x00FF) + para->small_adc_p1 -
		     para->adc_data_off) * 4;
		pr_info("0.1V adc data 0x%x\n", adc_data);
		p_cal_data[1] = (para->small_vol_p1) | (adc_data << 16);

		/* adc 1.0V */
		adc_data =
		    (((deta >> 8) & 0x00FF) + para->small_adc_p0 -
		     para->adc_data_off) * 4;
		pr_info("1.0V adc data 0x%x\n", adc_data);
		p_cal_data[0] = (para->small_vol_p0) | (adc_data << 16);
	} else if (pmic_adc->type == SC2731_ADC ||
		pmic_adc->type == SC2721_ADC) {

		/* adc 0.1V */
		adc_data =
		    (((deta >> 8) & 0x00FF) + para->small_adc_p1 -
		     para->adc_data_off) * 4;
		pr_info("0.1V adc data 0x%x\n", adc_data);
		p_cal_data[1] = (para->small_vol_p1) | (adc_data << 16);

		/* adc 1.0V */
		adc_data =
		    ((deta & 0x00FF) + para->small_adc_p0 -
		     para->adc_data_off) * 4;
		pr_info("1.0V adc data 0x%x\n", adc_data);
		p_cal_data[0] = (para->small_vol_p0) | (adc_data << 16);
	} else {
		pr_info("adc type is unknown\n");
	}

	return 0;
}

static int sprd_get_adc_big_scale_cal(unsigned int *p_cal_data)
{
	unsigned int deta = 0;
	unsigned int cal_data = 0;
	const struct pmic_adc_para *para = pmic_adc->info->para;

	deta =
	    sprd_pmic_efuse_bits_read(BITSINDEX(para->big_cal_efuse_blk, 0),
				      16);
	deta &= 0xFFFFFF;
	if ((!deta) || (p_cal_data == NULL))
		return -1;

	pr_info("get pmic efuse block %u, deta: 0x%08x\n",
		para->big_cal_efuse_blk, deta);
	/*adc 3.6V */
	cal_data =
	    ((deta >> 8) & 0x00FF) + para->big_adc_p0 - para->adc_data_off;
	p_cal_data[1] = (para->big_vol_p0) | ((cal_data << 2) << 16);
	/*adc 4.2V */
	cal_data = (deta & 0x00FF) + para->big_adc_p1 - para->adc_data_off;
	p_cal_data[0] = (para->big_vol_p1) | ((cal_data << 2) << 16);

	return 0;
}

static int sprd_bigscale_adc_efuse_get(void)
{
	unsigned int efuse_cal_data[2] = { 0 };

	if (!sprd_get_adc_big_scale_cal(efuse_cal_data)) {
		big_scale_cal.p0_vol = efuse_cal_data[0] & 0xffff;
		big_scale_cal.p0_adc = (efuse_cal_data[0] >> 16) & 0xffff;
		big_scale_cal.p1_vol = efuse_cal_data[1] & 0xffff;
		big_scale_cal.p1_adc = (efuse_cal_data[1] >> 16) & 0xffff;
		big_scale_cal.cal_type = SPRDBIG_AUXADC_CAL_CHIP;
		pr_info("adc efuse big cal %d,%d,%d,%d,cal_type:%d\n",
			big_scale_cal.p0_vol, big_scale_cal.p0_adc,
			big_scale_cal.p1_vol, big_scale_cal.p1_adc,
			big_scale_cal.cal_type);
		return 0;
	}
	return -1;
}

static int sprd_small_scale_adc_efuse_get(void)
{
	unsigned int efuse_cal_data[2] = { 0 };

	if (!sprd_get_adc_small_scale_cal(efuse_cal_data)) {
		small_sclae_cal.p0_vol = efuse_cal_data[0] & 0xffff;
		small_sclae_cal.p0_adc = (efuse_cal_data[0] >> 16) & 0xffff;
		small_sclae_cal.p1_vol = efuse_cal_data[1] & 0xffff;
		small_sclae_cal.p1_adc = (efuse_cal_data[1] >> 16) & 0xffff;
		small_sclae_cal.cal_type = SPRDLIT_AUXADC_CAL_CHIP;
		pr_info("adc efuse small cal %d,%d,%d,%d,cal_type:%d\n",
			small_sclae_cal.p0_vol, small_sclae_cal.p0_adc,
			small_sclae_cal.p1_vol, small_sclae_cal.p1_adc,
			small_sclae_cal.cal_type);
		return 0;
	}
	return -1;
}
#endif
static int average_int(int a[], int len)
{
	int i, sum = 0;

	for (i = 0; i < len; i++)
		sum += a[i];
	return DIV_ROUND(sum, len);
}

static int compare_val(const void *a, const void *b)
{
	return *(int *)a - *(int *)b;
}

static void sprd_pmic_adc_enable(struct sprd_pmic_adc *pmic_adc)
{
	pmic_adc_reg_set(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_ADC_EN,
			 BIT_ANA_ADC_EN);
	pmic_adc_reg_set(ANA_REG_GLB_ARM_CLK_EN,
			 BIT_CLK_AUXADC_EN | BIT_CLK_AUXAD_EN,
			 BIT_CLK_AUXADC_EN | BIT_CLK_AUXAD_EN);
	pmic_adc_reg_set(ANA_REG_GLB_XTL_WAIT_CTRL, BIT_XTL_EN, BIT_XTL_EN);
}

static void sprd_adc_dump_register(void)
{
	unsigned long base = 0;
	unsigned long end = base + 0x64;

	pr_info("sci_adc_dump_register begin\n");
	for (; base < end; base += 4) {
		pr_info("_base = 0x%lx, value = 0x%x\n", base,
			pmic_adc_reg_read(base));
	}
	pr_info("sci_adc_dump_register end\n");
}

static void sprd_pmic_adc_init(struct sprd_pmic_adc *pmic_adc)
{
	pmic_adc_reg_write(ADC_IRQ_EN, 0x0);
	pmic_adc_reg_write(ADC_IRQ_CLR, 0x1);
	sprd_pmic_adc_enable(pmic_adc);
	ana_chip_id =
	    ((u32) pmic_adc_reg_read(ANA_REG_GLB_CHIP_ID_HIGH) << 16) |
	    ((u32) pmic_adc_reg_read(ANA_REG_GLB_CHIP_ID_LOW) & 0xFFFF);

#if defined(CONFIG_OTP_SPRD_PMIC_EFUSE)
	if (cal_type == SPRD_AUXADC_CAL_NO) {
		if (!sprd_bigscale_adc_efuse_get()) {
			pmic_adc->bigscale_cal_type = SPRDBIG_AUXADC_CAL_CHIP;
			cal_type = SPRD_AUXADC_CAL_CHIP;
		}
		if (!sprd_small_scale_adc_efuse_get()) {
			pmic_adc->litscale_cal_type = SPRDLIT_AUXADC_CAL_CHIP;
			cal_type = SPRD_AUXADC_CAL_CHIP;
		}
	}
#endif
}

static int sprd_pmic_adc_config(struct adc_sample_data *adc)
{
	unsigned long addr = 0;
	unsigned val = 0;

	val = BIT_CH_IN_MODE(adc->signal_mode);
	val |= BIT_CH_SLOW(adc->sample_speed);
	val |= BIT_CH_SCALE(adc->scale);
	val |= BIT_CH_ID(adc->channel_id);
	val |= BIT_CH_DLY_EN(adc->hw_channel_delay ? 1 : 0);
	pmic_adc_reg_write(ADC_SW_CH_CFG, val);
	if (adc->channel_type > 0) {
		pmic_adc_reg_write(ADC_HW_CH_DELAY,
				   BIT_HW_CH_DELAY(adc->hw_channel_delay));
		if (adc->channel_type == 1)
			addr = (ADC_SLOW_HW_CHX_CFG(adc->channel_id));
		else
			addr = (ADC_FAST_HW_CHX_CFG(adc->channel_id));

		pmic_adc_reg_write(addr, val);
	}
	return 0;
}

static uint32_t sprd_small_scale_to_vol(uint16_t adcvalue)
{
	int32_t vol;

	vol = small_sclae_cal.p0_vol - small_sclae_cal.p1_vol;
	vol = vol * (adcvalue - small_sclae_cal.p0_adc);
	vol = vol / (small_sclae_cal.p0_adc - small_sclae_cal.p1_adc);
	vol = vol + small_sclae_cal.p0_vol;
	if (vol < 0)
		vol = 0;

	return vol;
}

static uint32_t sprd_big_scale_adc_to_vol(uint16_t adcvalue)
{
	int32_t vol;

	vol = big_scale_cal.p0_vol - big_scale_cal.p1_vol;
	vol = vol * (adcvalue - big_scale_cal.p0_adc);
	vol = vol / (big_scale_cal.p0_adc - big_scale_cal.p1_adc);
	vol = vol + big_scale_cal.p0_vol;
	if (vol < 0)
		vol = 0;

	return vol;
}

static int __init sprd_adc_cal_proc(char *str)
{
	unsigned int adc_data[2] = { 0 };
	char *cali_data = &str[1];
	int ret;

	if (str) {
		pr_info("adc_cal%s!\n", str);
		ret = sscanf(cali_data, "%d,%d", &adc_data[0], &adc_data[1]);
		if (ret != 2) {
			pr_err("adc cal proc invalid command\n");
			return -EINVAL;
		}
		big_scale_cal.p0_vol = adc_data[0] & 0xffff;
		big_scale_cal.p0_adc = (adc_data[0] >> 16) & 0xffff;
		big_scale_cal.p1_vol = adc_data[1] & 0xffff;
		big_scale_cal.p1_adc = (adc_data[1] >> 16) & 0xffff;
		big_scale_cal.cal_type = SPRD_AUXADC_CAL_NV;
		pr_info
		    ("adc cal cmdline adc_data[0]: 0x%x, adc_data[1]:0x%x\n",
		     adc_data[0], adc_data[1]);
		cal_type = SPRD_AUXADC_CAL_NV;
	}
	return 1;
}

__setup("adc_cal", sprd_adc_cal_proc);

static int sprd_adc_get_values(struct adc_sample_data *adc)
{
	int cnt = 12;
	unsigned long addr = 0;
	unsigned int val = 0;
	int ret = 0;
	int num = 0;
	int sample_bits_msk = 0;
	int *pbuf = NULL;

	if (!adc)
		return -EINVAL;

	pbuf = adc->pbuf;
	if (!pbuf)
		return -EINVAL;

	num = adc->sample_num;
	if (num > ADC_MAX_SAMPLE_NUM)
		return -EINVAL;

	ret = pmic_adc_lock();
	if (ret)
		return ret;

	sprd_pmic_adc_config(adc);

	addr = ADC_CTL;
	val = pmic_adc_reg_read((unsigned long)addr);
	val &= ~(BIT_EN_ADC | BIT_SW_CH_ON | BIT_ADC_BIT_MODE_MASK);
	pmic_adc_reg_write(addr, val);
	pmic_adc_reg_write(ADC_IRQ_CLR, 0x1);

	val = pmic_adc_reg_read(addr);
	val |= BIT_SW_CH_RUN_NUM(num);
	val |= BIT_EN_ADC;
	val |= BIT_ADC_BIT_MODE(adc->sample_bits);
	val |= BIT_SW_CH_ON;

	pmic_adc_reg_write(addr, val);
	while ((!pmic_adc_reg_read(ADC_IRQ_RAW)) && cnt--)
		udelay(50);

	if (cnt == -1) {
		ret = -1;
		sprd_adc_dump_register();
		goto Exit;
	}

	if (adc->sample_bits)
		sample_bits_msk = ((1 << 12) - 1);
	else
		sample_bits_msk = ((1 << 10) - 1);
	while (num--)
		*pbuf++ = pmic_adc_reg_read(ADC_DAT) & (sample_bits_msk);

Exit:
	val = pmic_adc_reg_read((unsigned long)addr);
	val &= ~BIT_EN_ADC;
	pmic_adc_reg_write(addr, val);
	pmic_adc_unlock();

	return ret;
}

static int sprd_sc2731_adc_ratio(int channel, int scale)
{
	switch (channel) {
	case SC2731ADC_CHANNEL_0:
		return RATIO(1, 1);
	case SC2731ADC_CHANNEL_1:
	case SC2731ADC_CHANNEL_2:
	case SC2731ADC_CHANNEL_3:
	case SC2731ADC_CHANNEL_4:
		return scale ? RATIO(400, 1025) : RATIO(1, 1);
	case SC2731ADC_CHANNEL_VBATSENSE:
		return RATIO(7, 29);
	case SC2731ADC_CHANNEL_VCHGSEN:
		return RATIO(375, 9000);
	case SC2731ADC_CHANNEL_TYPEC_CC1:
	case SC2731ADC_CHANNEL_TYPEC_CC2:
		return scale ? RATIO(100, 125) : RATIO(1, 1);
	case SC2731ADC_CHANNEL_DCDC_ARM0:
	case SC2731ADC_CHANNEL_DCDC_ARM1:
		return scale ? RATIO(4, 5) : RATIO(1, 1);
	case SC2731ADC_CHANNEL_DCDC_MEM:
		return scale ? RATIO(3, 5) : RATIO(3, 4);
	case SC2731ADC_CHANNEL_DCDC_GEN:
		return scale ? RATIO(49, 125) : RATIO(49, 100);
	case SC2731ADC_CHANNEL_DCDC_RF:
		return scale ? RATIO(9, 20) : RATIO(9, 16);
	case SC2731ADC_CHANNEL_DCDC_CORE:
		return scale ? RATIO(4, 5) : RATIO(1, 1);
	case SC2731ADC_CHANNEL_DCDC_WPA:
		return scale ? RATIO(36, 170) : RATIO(9, 34);
	case SC2731ADC_CHANNEL_DCDC_GPU:
		return scale ? RATIO(4, 5) : RATIO(1, 1);
	case SC2731ADC_CHANNEL_SDAVDD:
		return RATIO(1, 3);
	case SC2731ADC_CHANNEL_HEADMIC_HEADMIC_IN:
	case SC2731ADC_CHANNEL_HEADMIC_GND_DET:
	case SC2731ADC_CHANNEL_HEADMIC_HEAD_DRO_L:
	case SC2731ADC_CHANNEL_HEADMIC_HEADSET_L_INT:
		return scale ? RATIO(1, 3) : RATIO(1, 1);
	case SC2731ADC_CHANNEL_HEADMIC_VDDVB:
	case SC2731ADC_CHANNEL_HEADMIC_VDDVO:
	case SC2731ADC_CHANNEL_HEADMIC_VDDPA:
	case SC2731ADC_CHANNEL_HEADMIC_CPVDD:
	case SC2731ADC_CHANNEL_HEADMIC_MICBIAS:
	case SC2731ADC_CHANNEL_HEADMIC_AUXMICBIAS:
	case SC2731ADC_CHANNEL_HEADMIC_HEADMICBIAS:
		return scale ? RATIO(1, 12) : RATIO(1, 4);
	case SC2731ADC_CHANNEL_DCDCLDO_VDDCAMIO:
	case SC2731ADC_CHANNEL_DCDCLDO_VDD1V8:
	case SC2731ADC_CHANNEL_DCDCLDO_RF:
	case SC2731ADC_CHANNEL_DCDCLDO_VDDCAMD0:
	case SC2731ADC_CHANNEL_DCDCLDO_VDDCAMD1:
	case SC2731ADC_CHANNEL_DCDCLDO_VDDCON:
	case SC2731ADC_CHANNEL_DCDCLDO_VDDSRAM:
	case SC2731ADC_CHANNEL_VBATALDO_VDDCAMMOT:
	case SC2731ADC_CHANNEL_VBATALDO_VDD18DCXO:
	case SC2731ADC_CHANNEL_VBATDLDO_VDDSIM1:
	case SC2731ADC_CHANNEL_VBATDLDO_VDDSIM0:
	case SC2731ADC_CHANNEL_VBATDLDO_VDDCAMA1:
	case SC2731ADC_CHANNEL_VBATALDO_VDD2V8:
	case SC2731ADC_CHANNEL_VBATDLDO_VDDCAMA0:
	case SC2731ADC_CHANNEL_VBATBLDO_VDDSD:
	case SC2731ADC_CHANNEL_VBATBLDO_VDDSDIO:
	case SC2731ADC_CHANNEL_VBATBLDO_VDDEMMCCORE:
	case SC2731ADC_CHANNEL_VBATBLDO_VDDUSB33:
	case SC2731ADC_CHANNEL_VBATBLDO_VDDWIFIPA:
	case SC2731ADC_CHANNEL_VBATBLDO_VDDSIM2:
		return RATIO(9, 25);
	case SC2731ADC_CHANNEL_DP:
	case SC2731ADC_CHANNEL_DM:
		return RATIO(1, 1);
	default:
		return RATIO(1, 1);
	}
	return RATIO(1, 1);
}

static int sprd_sc2723_adc_ratio(int channel, int scale)
{
	switch (channel) {
	case SC2723ADC_CHANNEL_0:
		return RATIO(1, 1);
	case SC2723ADC_CHANNEL_1:
	case SC2723ADC_CHANNEL_2:
	case SC2723ADC_CHANNEL_3:
		return scale ? RATIO(400, 1025) : RATIO(1, 1);
	case SC2723ADC_CHANNEL_VCHGSEN:
		return RATIO(77, 1024);
	case SC2723ADC_CHANNEL_VBATSENSE:
	case SC2723ADC_CHANNEL_ISENSE:
		return RATIO(7, 29);
	case SC2723ADC_CHANNEL_DCDC_ARM:
	case SC2723ADC_CHANNEL_DCDC_CORE:
		return scale ? RATIO(36, 55) : RATIO(9, 11);
	case SC2723ADC_CHANNEL_DCDC_MEM:
	case SC2723ADC_CHANNEL_DCDC_RF:
		return scale ? RATIO(12, 25) : RATIO(3, 5);
	case SC2723ADC_CHANNEL_DCDC_GEN:
		return scale ? RATIO(3, 10) : RATIO(3, 8);
	case SC2723ADC_CHANNEL_DCDC_CON:
		return scale ? RATIO(9, 20) : RATIO(9, 16);
	case SC2723ADC_CHANNEL_DCDC_WPA:
		return scale ? RATIO(12, 55) : RATIO(3, 11);
	case SC2723ADC_CHANNEL_LDO_DCXO:
		return scale ? RATIO(4, 5) : RATIO(1, 1);
	case SC2723ADC_CHANNEL_HEADMIC_HEADMIC_IN:
	case SC2723ADC_CHANNEL_HEADMIC_GND_DET:
	case SC2723ADC_CHANNEL_HEADMIC_HEAD_DRO_L:
	case SC2723ADC_CHANNEL_HEADMIC_HEADSET_L_INT:
	case SC2723ADC_CHANNEL_HEADMIC_VDDVB:
	case SC2723ADC_CHANNEL_HEADMIC_VDDVO:
	case SC2723ADC_CHANNEL_HEADMIC_VDDPA:
	case SC2723ADC_CHANNEL_HEADMIC_CPVDD:
	case SC2723ADC_CHANNEL_HEADMIC_MICBIAS:
	case SC2723ADC_CHANNEL_HEADMIC_AUXMICBIAS:
	case SC2723ADC_CHANNEL_HEADMIC_HEADMICBIAS:
		return RATIO(1, 3);
	case SC2723ADC_CHANNEL_DCDCLDO_VDD1V8:
	case SC2723ADC_CHANNEL_DCDCLDO_VDDCAMD:
	case SC2723ADC_CHANNEL_DCDCLDO_VDDCAMIO:
	case SC2723ADC_CHANNEL_DCDCLDO_RF0:
	case SC2723ADC_CHANNEL_DCDCLDO_GEN1:
	case SC2723ADC_CHANNEL_DCDCLDO_GEN0:
		return RATIO(1, 2);
	case SC2723ADC_CHANNEL_VBATBK:
	case SC2723ADC_CHANNEL_VBATDLDO_AVDD28:
	case SC2723ADC_CHANNEL_VBATDLDO_VDDCAMA:
	case SC2723ADC_CHANNEL_VBATDLDO_VDDSIM2:
	case SC2723ADC_CHANNEL_VBATDLDO_VDDSIM1:
	case SC2723ADC_CHANNEL_VBATDLDO_VDDSIM0:
	case SC2723ADC_CHANNEL_VBATALDO_VDDWIFIPA:
	case SC2723ADC_CHANNEL_VBATALDO_VDDCAMMOT:
	case SC2723ADC_CHANNEL_VBATALDO_VDDEMMCCORE:
	case SC2723ADC_CHANNEL_VBATALDO_VDDDCXO:
	case SC2723ADC_CHANNEL_VBATALDO_VDDSDCORE:
	case SC2723ADC_CHANNEL_VBATALDO_VDD2V8:
	case SC2723ADC_CHANNEL_KPLED_VIBR_OUT:
	case SC2723ADC_CHANNEL_KPLED_OUT:
	case SC2723ADC_CHANNEL_LDO_VDDSDIO:
	case SC2723ADC_CHANNEL_LDO_VDDSDUSB:
	case SC2723ADC_CHANNEL_LDO_VDDSD:
	case SC2723ADC_CHANNEL_DP:
	case SC2723ADC_CHANNEL_DM:
		return RATIO(1, 3);
	default:
		return RATIO(1, 1);
	}
	return RATIO(1, 1);
}

static int sprd_sc2721_adc_ratio(int channel, int scale)
{
	switch (channel) {
	case SC2721ADC_CHANNEL_0:
		return RATIO(1, 1);
	case SC2721ADC_CHANNEL_1:
	case SC2721ADC_CHANNEL_2:
	case SC2721ADC_CHANNEL_3:
	case SC2721ADC_CHANNEL_4:
		return scale ? RATIO(400, 1025) : RATIO(1, 1);
	case SC2721ADC_CHANNEL_VBATSENSE:
		return RATIO(7, 29);
	case SC2721ADC_CHANNEL_TYPEC_CC1:
	case SC2721ADC_CHANNEL_TYPEC_CC2:
		return scale ? RATIO(100, 125) : RATIO(1, 1);
	case SC2721ADC_CHANNEL_DCDC_CPU:
		return scale ? RATIO(4, 5) : RATIO(1, 1);
	case SC2721ADC_CHANNEL_DCDC_MEM:
		return scale ? RATIO(3, 5) : RATIO(3, 4);
	case SC2721ADC_CHANNEL_DCDC_GEN:
		return scale ? RATIO(49, 125) : RATIO(49, 100);
	case SC2721ADC_CHANNEL_DCDC_CORE:
		return scale ? RATIO(4, 5) : RATIO(1, 1);
	case SC2721ADC_CHANNEL_DCDC_WPA:
		return scale ? RATIO(36, 170) : RATIO(9, 34);
	case SC2721ADC_CHANNEL_VCHGSEN:
		return RATIO(68, 900);
	case SC2721ADC_CHANNEL_PROG2ADC:
		return RATIO(48, 100);
	case SC2721ADC_CHANNEL_SDAVDD:
		return RATIO(1, 3);
	case SC2721ADC_CHANNEL_HEADMIC_IN_DET:
	case SC2721ADC_CHANNEL_HEADMIC_L_INT:
		return scale ? RATIO(1, 3) : RATIO(1, 1);
	case SC2721ADC_CHANNEL_HEADMIC_VDDVB:
	case SC2721ADC_CHANNEL_HEADMIC_VDDPA:
	case SC2721ADC_CHANNEL_HEADMIC_MICBIAS:
	case SC2721ADC_CHANNEL_HEADMIC_HEADMICBIAS:
		return scale ? RATIO(1, 12) : RATIO(1, 4);
	case SC2721ADC_CHANNEL_DP:
	case SC2721ADC_CHANNEL_DM:
		return RATIO(1, 1);
	default:
		return RATIO(1, 1);
	}
	return RATIO(1, 1);
}

static int sprd_def_adc_ratio(int channel, int scale)
{
	return RATIO(1, 1);
}

static void sprd_adc_get_vol_ratio(uint32_t channel_id, int scale,
				   uint32_t *div_numerators,
				   uint32_t *div_denominators)
{
	unsigned int ratio;

	switch (pmic_adc->type) {
	case SC2723_ADC:
		ratio = sprd_sc2723_adc_ratio(channel_id, scale);
		break;
	case SC2731_ADC:
		ratio = sprd_sc2731_adc_ratio(channel_id, scale);
		break;
	case SC2721_ADC:
		ratio = sprd_sc2721_adc_ratio(channel_id, scale);
		break;
	default:
		ratio = sprd_def_adc_ratio(channel_id, scale);

	}
	*div_numerators = ratio >> 16;
	*div_denominators = ratio << 16 >> 16;
}

#if defined(CONFIG_PMIC_SC2731) || defined(CONFIG_PMIC_SC2723)
static int sprd_adc_set_current(int enable, int isen)
{
	if (enable) {
		isen = (isen * 100 / 250 - 1);
		if (isen > BITS_AUXAD_CURRENT_IBS(-1))
			isen = BITS_AUXAD_CURRENT_IBS(-1);
		pmic_adc_reg_set(ANA_REG_GLB_AUXAD_CTL,
				 (BIT_AUXAD_CURRENTSEN_EN |
				  BITS_AUXAD_CURRENT_IBS(isen)),
				 BIT_AUXAD_CURRENTSEN_EN |
				 BITS_AUXAD_CURRENT_IBS(-1));
	} else {
		pmic_adc_reg_clr(ANA_REG_GLB_AUXAD_CTL,
				 BIT_AUXAD_CURRENTSEN_EN,
				 BIT_AUXAD_CURRENTSEN_EN);
	}
	return 0;
}
#else
static int sprd_adc_set_current(int enable, int isen)
{
	return 0;
}

#endif

/*
 * sprd_get_adc_rawdata_by_isen - read adc value by current sense
 * @channel: adc software channel id;
 * @scale: adc sample scale, 0:little scale, 1:big scale;
 * @current: adc current isense(uA),  1.25uA/step, max 40uA;
 * only external adc channel used
 * returns: adc value
 */

static int sprd_get_adc_rawdata_by_isen(unsigned int channel, int scale,
					int sample_num, int isen)
{
	struct adc_sample_data adc;
	int results[ADC_MESURE_NUMBER + 1] = { 0 };
	int ret = 0, i = 0;

	/* Fixme: only external adc channel used */
	if (channel > 3) {
		ret = -1;
		return ret;
	}

	adc.channel_id = channel;
	adc.channel_type = 0;
	adc.hw_channel_delay = 0;
	adc.pbuf = &results[0];
	adc.sample_bits = 1;
	adc.sample_num = ADC_MESURE_NUMBER;
	adc.sample_speed = 0;
	adc.scale = scale;
	adc.signal_mode = 0;

	sprd_adc_set_current(1, isen);
	if (sprd_adc_get_values(&adc) == 0) {
		ret = average_int(&results[ADC_MESURE_NUMBER / 5],
				  (ADC_MESURE_NUMBER -
				   ADC_MESURE_NUMBER * 2 / 5));
	}
	sprd_adc_set_current(0, 0);

	for (i = 0; i < ARRAY_SIZE(results); i++)
		pr_info("%d\t", results[i]);

	pr_info("\n%s() adc[%d] value: %d\n", __func__, channel, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(sprd_get_adc_rawdata_by_isen);

static int sprd_get_adc_raw_value(unsigned int channel, int scale,
				  int sample_num)
{
	struct adc_sample_data adc;
	int32_t result[1];

	adc.channel_id = channel;
	adc.channel_type = 0;
	adc.hw_channel_delay = 0;
	adc.pbuf = &result[0];
	adc.sample_bits = 1;
	adc.sample_num = sample_num;
	adc.sample_speed = 0;
	adc.scale = scale;
	adc.signal_mode = 0;

	if (sprd_adc_get_values(&adc) != 0)
		pr_err("sci_adc_get_value, return error\n");

	pr_info("adc channel %d,raw value %d\n", channel, result[0]);
	return result[0];
}

static int sprd_get_average_adc_data(unsigned int channel, unsigned int scale,
				     int sample_num)
{
	int adc_val[MEASURE_TIMES] = { 0 }, adc_res = 0;
	struct adc_sample_data adc_sample = {
		.channel_id = channel,
		.channel_type = 0,
		.hw_channel_delay = 0,
		.scale = scale,
		.pbuf = &adc_val[0],
		.sample_num = sample_num,
		.sample_bits = 1,
		.sample_speed = 0,
		.signal_mode = 0,
	};

	if (sprd_adc_get_values(&adc_sample) != 0) {
		pr_err("sprd_adc error occurred in function %s\n", __func__);
		sprd_adc_dump_register();
		return 0;
	}

	sort(adc_val, ARRAY_SIZE(adc_val), sizeof(uint32_t), compare_val, NULL);

	adc_res =
	    average_int(&adc_val[ADC_DROP_CNT],
			MEASURE_TIMES - ADC_DROP_CNT * 2);
	pr_info("adc channel %d, value%d\n", channel, (int)adc_res);
	return (int)adc_res;
}

static uint32_t sprd_chan_big_adc_to_vol(uint16_t channel, int scale,
					 uint16_t adcvalue)
{
	uint32_t result;
	uint32_t vol;
	uint32_t m, n;
	uint32_t bat_numerators, bat_denominators;
	uint32_t numerators, denominators;

	vol = sprd_big_scale_adc_to_vol(adcvalue);
	sprd_adc_get_vol_ratio(ADC_CHANNEL_VBATSENSE, 0,
			       &bat_numerators, &bat_denominators);
	sprd_adc_get_vol_ratio(channel, scale, &numerators, &denominators);

	/* v1 = vbat_vol*0.268 = vol_bat_m * r2 /(r1+r2) */

	n = bat_denominators * numerators;
	m = vol * bat_numerators * (denominators);
	result = (m + n / 2) / n;
	return result;
}

static uint32_t sprd_chan_small_adc_to_vol(uint16_t channel, int scale,
					   uint16_t adcvalue)
{
	uint32_t result;
	uint32_t vol;
	uint32_t m, n;
	uint32_t bat_numerators, bat_denominators;
	uint32_t numerators, denominators;

	vol = sprd_small_scale_to_vol(adcvalue);
	bat_numerators = 1;
	bat_denominators = 1;
	sprd_adc_get_vol_ratio(channel, scale, &numerators, &denominators);

	/* v1 = vbat_vol*0.268 = vol_bat_m * r2 /(r1+r2) */

	n = bat_denominators * numerators;
	m = vol * bat_numerators * (denominators);
	result = (m + n / 2) / n;
	return result;
}

static int sprd_get_processed_adc_vol_value(uint32_t vir_channel,
					    uint32_t phy_channel, int scale,
					    int sample_num)
{
	int chan_adc = 0, chan_vol = 0;

	chan_adc = sprd_get_average_adc_data(phy_channel, scale, sample_num);
	if (phy_channel == 5)
		chan_vol =
		    sprd_chan_big_adc_to_vol(vir_channel, scale, chan_adc);
	else
		chan_vol =
		    sprd_chan_small_adc_to_vol(vir_channel, scale, chan_adc);

	return chan_vol;
}

static int sprd_pmic_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long m)
{
	struct sprd_pmic_adc *st = iio_priv(indio_dev);
	uint32_t vir_chan;

	vir_chan = chan->channel / 2;
	if ((chan->channel < 0)
	    || (chan->channel > st->info->num_channels))
		return -EINVAL;
	switch (m) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		if (chan->channel % 2 == 0)
			*val = sprd_get_adc_raw_value(chan->address, 1, 1);
		else
			*val = sprd_get_adc_raw_value(chan->address, 0, 1);

		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&indio_dev->mlock);
		if (chan->channel % 2 == 0) {
			*val =
			    sprd_get_processed_adc_vol_value(vir_chan,
							     chan->address, 1,
							     ADC_MESURE_NUMBER);
		} else {
			*val =
			    sprd_get_processed_adc_vol_value(vir_chan,
							     chan->address, 0,
							     ADC_MESURE_NUMBER);
		}
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_AVERAGE_RAW:
		mutex_lock(&indio_dev->mlock);
		if (chan->channel % 2 == 0) {
			*val =
			    sprd_get_average_adc_data(chan->address, 1,
						      ADC_MESURE_NUMBER);
		} else {
			*val =
			    sprd_get_average_adc_data(chan->address, 0,
						      ADC_MESURE_NUMBER);
		}
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

/*  _channel: is logical channel, even number representation of a big scale
	and odd number  representation of a small scale.
     _hw_id is hw channel
*/

#define SPRD_ADC_CHANNEL(_channel, _hw_id, _type, _name) {	\
	.type = _type,					\
	.channel = _channel,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |  \
			      BIT(IIO_CHAN_INFO_AVERAGE_RAW) | \
			      BIT(IIO_CHAN_INFO_PROCESSED), \
	.address = _hw_id,	\
	.datasheet_name = _name,			\
	.indexed = 1,					\
}

static const struct iio_chan_spec sprd_sc2723_iio_channels[] = {
	SPRD_ADC_CHANNEL(0, 0, IIO_VOLTAGE, "ADCI0"),
	SPRD_ADC_CHANNEL(1, 0, IIO_VOLTAGE, "ADCI0"),
	SPRD_ADC_CHANNEL(2, 1, IIO_VOLTAGE, "ADCI1"),
	SPRD_ADC_CHANNEL(3, 1, IIO_VOLTAGE, "ADCI1"),
	SPRD_ADC_CHANNEL(4, 2, IIO_VOLTAGE, "ADCI2"),
	SPRD_ADC_CHANNEL(5, 2, IIO_VOLTAGE, "ADCI2"),
	SPRD_ADC_CHANNEL(6, 3, IIO_VOLTAGE, "ADCI3"),
	SPRD_ADC_CHANNEL(7, 3, IIO_VOLTAGE, "ADCI3"),
	SPRD_ADC_CHANNEL(8, 4, IIO_VOLTAGE, "ADCI4"),
	SPRD_ADC_CHANNEL(9, 4, IIO_VOLTAGE, "ADCI4"),
	SPRD_ADC_CHANNEL(10, 5, IIO_VOLTAGE, "VBATSENSE"),
	SPRD_ADC_CHANNEL(11, 5, IIO_VOLTAGE, "VBATSENSE"),
	SPRD_ADC_CHANNEL(12, 6, IIO_VOLTAGE, "VCHGSEN"),
	SPRD_ADC_CHANNEL(13, 6, IIO_VOLTAGE, "VCHGSEN"),
	SPRD_ADC_CHANNEL(14, 7, IIO_VOLTAGE, "VCHGBG"),
	SPRD_ADC_CHANNEL(15, 7, IIO_VOLTAGE, "VCHGBG"),
	SPRD_ADC_CHANNEL(16, 8, IIO_VOLTAGE, "ISENSE"),
	SPRD_ADC_CHANNEL(17, 8, IIO_VOLTAGE, "ISENSE"),
	SPRD_ADC_CHANNEL(18, 9, IIO_VOLTAGE, "FLOATTING_9"),
	SPRD_ADC_CHANNEL(19, 9, IIO_VOLTAGE, "FLOATTING_9"),
	SPRD_ADC_CHANNEL(20, 10, IIO_VOLTAGE, "FLOATTING_10"),
	SPRD_ADC_CHANNEL(21, 10, IIO_VOLTAGE, "FLOATTING_10"),
	SPRD_ADC_CHANNEL(22, 11, IIO_VOLTAGE, "FLOATTING_11"),
	SPRD_ADC_CHANNEL(23, 11, IIO_VOLTAGE, "FLOATTING_11"),
	SPRD_ADC_CHANNEL(24, 12, IIO_VOLTAGE, "FLOATTING_12"),
	SPRD_ADC_CHANNEL(25, 12, IIO_VOLTAGE, "FLOATTING_12"),
	SPRD_ADC_CHANNEL(26, 13, IIO_VOLTAGE, "DCDC_ARM"),
	SPRD_ADC_CHANNEL(27, 13, IIO_VOLTAGE, "DCDC_ARM"),
	SPRD_ADC_CHANNEL(28, 13, IIO_VOLTAGE, "DCDC_CORE"),
	SPRD_ADC_CHANNEL(29, 13, IIO_VOLTAGE, "DCDC_CORE"),
	SPRD_ADC_CHANNEL(30, 13, IIO_VOLTAGE, "DCDC_MEM"),
	SPRD_ADC_CHANNEL(31, 13, IIO_VOLTAGE, "DCDC_MEM"),
	SPRD_ADC_CHANNEL(32, 13, IIO_VOLTAGE, "DCDC_GEN"),
	SPRD_ADC_CHANNEL(33, 13, IIO_VOLTAGE, "DCDC_GEN"),
	SPRD_ADC_CHANNEL(34, 13, IIO_VOLTAGE, "DCDC_RF"),
	SPRD_ADC_CHANNEL(35, 13, IIO_VOLTAGE, "DCDC_RF"),
	SPRD_ADC_CHANNEL(36, 13, IIO_VOLTAGE, "DCDC_CON"),
	SPRD_ADC_CHANNEL(37, 13, IIO_VOLTAGE, "DCDC_CON"),
	SPRD_ADC_CHANNEL(38, 13, IIO_VOLTAGE, "DCDC_WPA"),
	SPRD_ADC_CHANNEL(39, 13, IIO_VOLTAGE, "DCDC_WPA"),
	SPRD_ADC_CHANNEL(40, 14, IIO_VOLTAGE, "LDODCXO"),
	SPRD_ADC_CHANNEL(41, 14, IIO_VOLTAGE, "LDODCXO"),
	SPRD_ADC_CHANNEL(42, 15, IIO_VOLTAGE, "FLOATTING_15"),
	SPRD_ADC_CHANNEL(43, 15, IIO_VOLTAGE, "FLOATTING_15"),
	SPRD_ADC_CHANNEL(44, 16, IIO_VOLTAGE, "FLOATTING_16"),
	SPRD_ADC_CHANNEL(45, 16, IIO_VOLTAGE, "FLOATTING_16"),
	SPRD_ADC_CHANNEL(46, 17, IIO_VOLTAGE, "FLOATTING_17"),
	SPRD_ADC_CHANNEL(47, 17, IIO_VOLTAGE, "FLOATTING_17"),
	SPRD_ADC_CHANNEL(48, 18, IIO_VOLTAGE, "FLOATTING_18"),
	SPRD_ADC_CHANNEL(49, 18, IIO_VOLTAGE, "FLOATTING_18"),
	SPRD_ADC_CHANNEL(50, 19, IIO_VOLTAGE, "FLOATTING_19"),
	SPRD_ADC_CHANNEL(51, 19, IIO_VOLTAGE, "FLOATTING_19"),
	SPRD_ADC_CHANNEL(52, 20, IIO_VOLTAGE, "HEADMIC_HEADMIC_IN"),
	SPRD_ADC_CHANNEL(53, 20, IIO_VOLTAGE, "HEADMIC_HEADMIC_IN"),
	SPRD_ADC_CHANNEL(54, 20, IIO_VOLTAGE, "HEADMIC_GND_DET"),
	SPRD_ADC_CHANNEL(55, 20, IIO_VOLTAGE, "HEADMIC_GND_DET"),
	SPRD_ADC_CHANNEL(56, 20, IIO_VOLTAGE, "HEADMIC_HEAD_DRO_L"),
	SPRD_ADC_CHANNEL(57, 20, IIO_VOLTAGE, "HEADMIC_HEAD_DRO_L"),
	SPRD_ADC_CHANNEL(58, 20, IIO_VOLTAGE, "HEADMIC_HEADSET_L_INT"),
	SPRD_ADC_CHANNEL(59, 20, IIO_VOLTAGE, "HEADMIC_HEADSET_L_INT"),
	SPRD_ADC_CHANNEL(60, 20, IIO_VOLTAGE, "HEADMIC_VDDVB"),
	SPRD_ADC_CHANNEL(61, 20, IIO_VOLTAGE, "HEADMIC_VDDVB"),
	SPRD_ADC_CHANNEL(62, 20, IIO_VOLTAGE, "HEADMIC_VDDVO"),
	SPRD_ADC_CHANNEL(63, 20, IIO_VOLTAGE, "HEADMIC_VDDVO"),
	SPRD_ADC_CHANNEL(64, 20, IIO_VOLTAGE, "HEADMIC_VDDPA"),
	SPRD_ADC_CHANNEL(65, 20, IIO_VOLTAGE, "HEADMIC_VDDPA"),
	SPRD_ADC_CHANNEL(66, 20, IIO_VOLTAGE, "HEADMIC_CPVDD"),
	SPRD_ADC_CHANNEL(67, 20, IIO_VOLTAGE, "HEADMIC_CPVDD"),
	SPRD_ADC_CHANNEL(68, 20, IIO_VOLTAGE, "HEADMIC_MICBIAS"),
	SPRD_ADC_CHANNEL(69, 20, IIO_VOLTAGE, "HEADMIC_MICBIAS"),
	SPRD_ADC_CHANNEL(70, 20, IIO_VOLTAGE, "HEADMIC_AUXMICBIAS"),
	SPRD_ADC_CHANNEL(71, 20, IIO_VOLTAGE, "HEADMIC_AUXMICBIAS"),
	SPRD_ADC_CHANNEL(72, 20, IIO_VOLTAGE, "HEADMIC_HEADMICBIAS"),
	SPRD_ADC_CHANNEL(73, 20, IIO_VOLTAGE, "HEADMIC_HEADMICBIAS"),
	SPRD_ADC_CHANNEL(74, 21, IIO_VOLTAGE, "DCDCLDO_VDD1V8"),
	SPRD_ADC_CHANNEL(75, 21, IIO_VOLTAGE, "DCDCLDO_VDD1V8"),
	SPRD_ADC_CHANNEL(76, 21, IIO_VOLTAGE, "DCDCLDO_VDDCAMD"),
	SPRD_ADC_CHANNEL(77, 21, IIO_VOLTAGE, "DCDCLDO_VDDCAMD"),
	SPRD_ADC_CHANNEL(78, 21, IIO_VOLTAGE, "DCDCLDO_VDDCAMIO"),
	SPRD_ADC_CHANNEL(79, 21, IIO_VOLTAGE, "DCDCLDO_VDDCAMIO"),
	SPRD_ADC_CHANNEL(80, 21, IIO_VOLTAGE, "DCDCLDO_RF0"),
	SPRD_ADC_CHANNEL(81, 21, IIO_VOLTAGE, "DCDCLDO_RF0"),
	SPRD_ADC_CHANNEL(82, 21, IIO_VOLTAGE, "DCDCLDO_GEN1"),
	SPRD_ADC_CHANNEL(83, 21, IIO_VOLTAGE, "DCDCLDO_GEN1"),
	SPRD_ADC_CHANNEL(84, 21, IIO_VOLTAGE, "DCDCLDO_GEN0"),
	SPRD_ADC_CHANNEL(85, 21, IIO_VOLTAGE, "DCDCLDO_GEN0"),
	SPRD_ADC_CHANNEL(86, 22, IIO_VOLTAGE, "VBATDLDO_AVDD28"),
	SPRD_ADC_CHANNEL(87, 22, IIO_VOLTAGE, "VBATDLDO_AVDD28"),
	SPRD_ADC_CHANNEL(88, 22, IIO_VOLTAGE, "VBATDLDO_VDDCAMA"),
	SPRD_ADC_CHANNEL(89, 22, IIO_VOLTAGE, "VBATDLDO_VDDCAMA"),
	SPRD_ADC_CHANNEL(90, 22, IIO_VOLTAGE, "VBATDLDO_VDDSIM2"),
	SPRD_ADC_CHANNEL(91, 22, IIO_VOLTAGE, "VBATDLDO_VDDSIM2"),
	SPRD_ADC_CHANNEL(92, 22, IIO_VOLTAGE, "VBATDLDO_VDDSIM1"),
	SPRD_ADC_CHANNEL(93, 22, IIO_VOLTAGE, "VBATDLDO_VDDSIM1"),
	SPRD_ADC_CHANNEL(94, 22, IIO_VOLTAGE, "VBATDLDO_VDDSIM0"),
	SPRD_ADC_CHANNEL(95, 22, IIO_VOLTAGE, "VBATDLDO_VDDSIM0"),
	SPRD_ADC_CHANNEL(96, 23, IIO_VOLTAGE, "VBATALDO_VDDWIFIPA"),
	SPRD_ADC_CHANNEL(97, 23, IIO_VOLTAGE, "VBATALDO_VDDWIFIPA"),
	SPRD_ADC_CHANNEL(98, 23, IIO_VOLTAGE, "VBATALDO_VDDCAMMOT"),
	SPRD_ADC_CHANNEL(99, 23, IIO_VOLTAGE, "VBATALDO_VDDCAMMOT"),
	SPRD_ADC_CHANNEL(100, 23, IIO_VOLTAGE, "VBATALDO_VDDEMMCCORE"),
	SPRD_ADC_CHANNEL(101, 23, IIO_VOLTAGE, "VBATALDO_VDDEMMCCORE"),
	SPRD_ADC_CHANNEL(102, 23, IIO_VOLTAGE, "VBATALDO_VDDDCXO"),
	SPRD_ADC_CHANNEL(103, 23, IIO_VOLTAGE, "VBATALDO_VDDDCXO"),
	SPRD_ADC_CHANNEL(104, 23, IIO_VOLTAGE, "VBATALDO_VDDSDCORE"),
	SPRD_ADC_CHANNEL(105, 23, IIO_VOLTAGE, "VBATALDO_VDDSDCORE"),
	SPRD_ADC_CHANNEL(106, 23, IIO_VOLTAGE, "VBATALDO_VDD2V8"),
	SPRD_ADC_CHANNEL(107, 23, IIO_VOLTAGE, "VBATALDO_VDD2V8"),
	SPRD_ADC_CHANNEL(108, 24, IIO_VOLTAGE, "KPLED_VIBR_OUT"),
	SPRD_ADC_CHANNEL(109, 24, IIO_VOLTAGE, "KPLED_VIBR_OUT"),
	SPRD_ADC_CHANNEL(110, 24, IIO_VOLTAGE, "KPLED_OUT"),
	SPRD_ADC_CHANNEL(111, 24, IIO_VOLTAGE, "KPLED_OUT"),
	SPRD_ADC_CHANNEL(112, 25, IIO_VOLTAGE, "TEMP_SENSOR"),
	SPRD_ADC_CHANNEL(113, 25, IIO_VOLTAGE, "TEMP_SENSOR"),
	SPRD_ADC_CHANNEL(114, 26, IIO_CURRENT, "LDODCDCIN_TOP"),
	SPRD_ADC_CHANNEL(115, 26, IIO_CURRENT, "LDODCDCIN_TOP"),
	SPRD_ADC_CHANNEL(116, 27, IIO_VOLTAGE, "LDOVBAT_B_TOP"),
	SPRD_ADC_CHANNEL(117, 27, IIO_VOLTAGE, "LDOVBAT_B_TOP"),
	SPRD_ADC_CHANNEL(118, 28, IIO_VOLTAGE, "LDOVBAT_A_TOP"),
	SPRD_ADC_CHANNEL(119, 28, IIO_VOLTAGE, "LDOVBAT_A_TOP"),
	SPRD_ADC_CHANNEL(120, 29, IIO_VOLTAGE, "LDO_VDDSDIO"),
	SPRD_ADC_CHANNEL(121, 29, IIO_VOLTAGE, "LDO_VDDSDIO"),
	SPRD_ADC_CHANNEL(122, 29, IIO_VOLTAGE, "LDO_VDDSDUSB"),
	SPRD_ADC_CHANNEL(123, 29, IIO_VOLTAGE, "LDO_VDDSDUSB"),
	SPRD_ADC_CHANNEL(124, 29, IIO_VOLTAGE, "LDO_VDDSD"),
	SPRD_ADC_CHANNEL(125, 29, IIO_VOLTAGE, "LDO_VDDSD"),
	SPRD_ADC_CHANNEL(126, 30, IIO_VOLTAGE, "DP"),
	SPRD_ADC_CHANNEL(127, 30, IIO_VOLTAGE, "DP"),
	SPRD_ADC_CHANNEL(128, 31, IIO_VOLTAGE, "DM"),
	SPRD_ADC_CHANNEL(129, 31, IIO_VOLTAGE, "DM"),
};

static const struct iio_chan_spec sprd_sc2731_iio_channels[] = {
	SPRD_ADC_CHANNEL(0, 0, IIO_VOLTAGE, "ADCI0"),
	SPRD_ADC_CHANNEL(1, 0, IIO_VOLTAGE, "ADCI0"),
	SPRD_ADC_CHANNEL(2, 1, IIO_VOLTAGE, "ADCI1"),
	SPRD_ADC_CHANNEL(3, 1, IIO_VOLTAGE, "ADCI1"),
	SPRD_ADC_CHANNEL(4, 2, IIO_VOLTAGE, "ADCI2"),
	SPRD_ADC_CHANNEL(5, 2, IIO_VOLTAGE, "ADCI2"),
	SPRD_ADC_CHANNEL(6, 3, IIO_VOLTAGE, "ADCI3"),
	SPRD_ADC_CHANNEL(7, 3, IIO_VOLTAGE, "ADCI3"),
	SPRD_ADC_CHANNEL(8, 4, IIO_VOLTAGE, "ADCI4"),
	SPRD_ADC_CHANNEL(9, 4, IIO_VOLTAGE, "ADCI4"),
	SPRD_ADC_CHANNEL(10, 5, IIO_VOLTAGE, "VBATSENSE"),
	SPRD_ADC_CHANNEL(11, 5, IIO_VOLTAGE, "VBATSENSE"),
	SPRD_ADC_CHANNEL(12, 6, IIO_VOLTAGE, "VCHGSEN"),
	SPRD_ADC_CHANNEL(13, 6, IIO_VOLTAGE, "VCHGSEN"),
	SPRD_ADC_CHANNEL(14, 7, IIO_VOLTAGE, "TYPEC_CC1"),
	SPRD_ADC_CHANNEL(15, 7, IIO_VOLTAGE, "TYPEC_CC1"),
	SPRD_ADC_CHANNEL(16, 8, IIO_VOLTAGE, "FLOATING_8"),
	SPRD_ADC_CHANNEL(17, 8, IIO_VOLTAGE, "FLOATING_8"),
	SPRD_ADC_CHANNEL(18, 9, IIO_VOLTAGE, "FLOATING_9"),
	SPRD_ADC_CHANNEL(19, 9, IIO_VOLTAGE, "FLOATING_9"),
	SPRD_ADC_CHANNEL(20, 10, IIO_VOLTAGE, "FLOATING_10"),
	SPRD_ADC_CHANNEL(21, 10, IIO_VOLTAGE, "FLOATING_10"),
	SPRD_ADC_CHANNEL(22, 11, IIO_VOLTAGE, "FLOATING_11"),
	SPRD_ADC_CHANNEL(23, 11, IIO_VOLTAGE, "FLOATING_11"),
	SPRD_ADC_CHANNEL(24, 12, IIO_VOLTAGE, "FLOATING_12"),
	SPRD_ADC_CHANNEL(25, 12, IIO_VOLTAGE, "FLOATING_12"),
	SPRD_ADC_CHANNEL(26, 13, IIO_VOLTAGE, "DCDC_ARM0"),
	SPRD_ADC_CHANNEL(27, 13, IIO_VOLTAGE, "DCDC_ARM0"),
	SPRD_ADC_CHANNEL(28, 13, IIO_VOLTAGE, "DCDC_ARM1"),
	SPRD_ADC_CHANNEL(29, 13, IIO_VOLTAGE, "DCDC_ARM1"),
	SPRD_ADC_CHANNEL(30, 13, IIO_VOLTAGE, "DCDC_MEM"),
	SPRD_ADC_CHANNEL(31, 13, IIO_VOLTAGE, "DCDC_MEM"),
	SPRD_ADC_CHANNEL(32, 13, IIO_VOLTAGE, "DCDC_GEN"),
	SPRD_ADC_CHANNEL(33, 13, IIO_VOLTAGE, "DCDC_GEN"),
	SPRD_ADC_CHANNEL(34, 13, IIO_VOLTAGE, "DCDC_RF"),
	SPRD_ADC_CHANNEL(35, 13, IIO_VOLTAGE, "DCDC_RF"),
	SPRD_ADC_CHANNEL(36, 13, IIO_VOLTAGE, "DCDC_CORE"),
	SPRD_ADC_CHANNEL(37, 13, IIO_VOLTAGE, "DCDC_CORE"),
	SPRD_ADC_CHANNEL(38, 13, IIO_VOLTAGE, "DCDC_WPA"),
	SPRD_ADC_CHANNEL(39, 13, IIO_VOLTAGE, "DCDC_WPA"),
	SPRD_ADC_CHANNEL(40, 13, IIO_VOLTAGE, "DCDC_GPU"),
	SPRD_ADC_CHANNEL(41, 13, IIO_VOLTAGE, "DCDC_GPU"),
	SPRD_ADC_CHANNEL(42, 14, IIO_VOLTAGE, "FLOATTING_14"),
	SPRD_ADC_CHANNEL(43, 14, IIO_VOLTAGE, "FLOATTING_14"),
	SPRD_ADC_CHANNEL(44, 15, IIO_VOLTAGE, "FLOATTING_15"),
	SPRD_ADC_CHANNEL(45, 15, IIO_VOLTAGE, "FLOATTING_15"),
	SPRD_ADC_CHANNEL(46, 16, IIO_VOLTAGE, "FLOATTING_16"),
	SPRD_ADC_CHANNEL(47, 16, IIO_VOLTAGE, "FLOATTING_16"),
	SPRD_ADC_CHANNEL(48, 17, IIO_VOLTAGE, "FLOATTING_17"),
	SPRD_ADC_CHANNEL(49, 17, IIO_VOLTAGE, "FLOATTING_17"),
	SPRD_ADC_CHANNEL(50, 18, IIO_VOLTAGE, "FLOATTING_18"),
	SPRD_ADC_CHANNEL(51, 18, IIO_VOLTAGE, "FLOATTING_18"),
	SPRD_ADC_CHANNEL(52, 19, IIO_VOLTAGE, "SDAVDD"),
	SPRD_ADC_CHANNEL(53, 19, IIO_VOLTAGE, "SDAVDD"),
	SPRD_ADC_CHANNEL(54, 20, IIO_VOLTAGE, "HEADMIC_IN"),
	SPRD_ADC_CHANNEL(55, 20, IIO_VOLTAGE, "HEADMIC_IN"),
	SPRD_ADC_CHANNEL(56, 20, IIO_VOLTAGE, "HEADMIC_GND_DET"),
	SPRD_ADC_CHANNEL(57, 20, IIO_VOLTAGE, "HEADMIC_GND_DET"),
	SPRD_ADC_CHANNEL(58, 20, IIO_CURRENT, "HEADMIC_HEAD_DRO_L"),
	SPRD_ADC_CHANNEL(59, 20, IIO_VOLTAGE, "HEADMIC_HEAD_DRO_L"),
	SPRD_ADC_CHANNEL(60, 20, IIO_VOLTAGE, "HEADMIC_HEADSET_L_INT"),
	SPRD_ADC_CHANNEL(61, 20, IIO_VOLTAGE, "HEADMIC_HEADSET_L_INT"),
	SPRD_ADC_CHANNEL(62, 20, IIO_VOLTAGE, "HEADMIC_VDDVB"),
	SPRD_ADC_CHANNEL(63, 20, IIO_VOLTAGE, "HEADMIC_VDDVB"),
	SPRD_ADC_CHANNEL(64, 20, IIO_VOLTAGE, "HEADMIC_VDDVO"),
	SPRD_ADC_CHANNEL(65, 20, IIO_VOLTAGE, "HEADMIC_VDDVO"),
	SPRD_ADC_CHANNEL(66, 20, IIO_VOLTAGE, "HEADMIC_VDDPA"),
	SPRD_ADC_CHANNEL(67, 20, IIO_VOLTAGE, "HEADMIC_VDDPA"),
	SPRD_ADC_CHANNEL(68, 20, IIO_VOLTAGE, "HEADMIC_CPVDD"),
	SPRD_ADC_CHANNEL(69, 20, IIO_VOLTAGE, "HEADMIC_CPVDD"),
	SPRD_ADC_CHANNEL(70, 20, IIO_VOLTAGE, "HEADMIC_MICBIAS"),
	SPRD_ADC_CHANNEL(71, 20, IIO_VOLTAGE, "HEADMIC_MICBIAS"),
	SPRD_ADC_CHANNEL(72, 20, IIO_VOLTAGE, "HEADMIC_AUXMICBIAS"),
	SPRD_ADC_CHANNEL(73, 20, IIO_VOLTAGE, "HEADMIC_AUXMICBIAS"),
	SPRD_ADC_CHANNEL(74, 20, IIO_VOLTAGE, "HEADMIC_HEADMICBIAS"),
	SPRD_ADC_CHANNEL(75, 20, IIO_VOLTAGE, "HEADMIC_HEADMICBIAS"),
	SPRD_ADC_CHANNEL(76, 21, IIO_VOLTAGE, "DCDCLDO_VDDCAMIO"),
	SPRD_ADC_CHANNEL(77, 21, IIO_VOLTAGE, "DCDCLDO_VDDCAMIO"),
	SPRD_ADC_CHANNEL(78, 21, IIO_VOLTAGE, "DCDCLDO_VDD1V8"),
	SPRD_ADC_CHANNEL(79, 21, IIO_VOLTAGE, "DCDCLDO_VDD1V8"),
	SPRD_ADC_CHANNEL(80, 21, IIO_VOLTAGE, "DCDCLDO_RF"),
	SPRD_ADC_CHANNEL(81, 21, IIO_VOLTAGE, "DCDCLDO_RF"),
	SPRD_ADC_CHANNEL(82, 21, IIO_VOLTAGE, "DCDCLDO_VDDCAMD0"),
	SPRD_ADC_CHANNEL(83, 21, IIO_VOLTAGE, "DCDCLDO_VDDCAMD0"),
	SPRD_ADC_CHANNEL(84, 21, IIO_VOLTAGE, "DCDCLDO_VDDCAMD1"),
	SPRD_ADC_CHANNEL(85, 21, IIO_VOLTAGE, "DCDCLDO_VDDCAMD1"),
	SPRD_ADC_CHANNEL(86, 21, IIO_VOLTAGE, "DCDCLDO_VDDCON"),
	SPRD_ADC_CHANNEL(87, 21, IIO_VOLTAGE, "DCDCLDO_VDDCON"),
	SPRD_ADC_CHANNEL(88, 21, IIO_VOLTAGE, "DCDCLDO_VDDSRAM"),
	SPRD_ADC_CHANNEL(89, 21, IIO_VOLTAGE, "DCDCLDO_VDDSRAM"),
	SPRD_ADC_CHANNEL(90, 22, IIO_VOLTAGE, "VBATALDO_VDDCAMMOT"),
	SPRD_ADC_CHANNEL(91, 22, IIO_VOLTAGE, "VBATALDO_VDDCAMMOT"),
	SPRD_ADC_CHANNEL(92, 22, IIO_VOLTAGE, "VBATALDO_VDD18DCXO"),
	SPRD_ADC_CHANNEL(93, 22, IIO_VOLTAGE, "VBATALDO_VDD18DCXO"),
	SPRD_ADC_CHANNEL(94, 22, IIO_VOLTAGE, "VBATDLDO_VDDSIM1"),
	SPRD_ADC_CHANNEL(95, 22, IIO_VOLTAGE, "VBATDLDO_VDDSIM1"),
	SPRD_ADC_CHANNEL(96, 22, IIO_VOLTAGE, "VBATDLDO_VDDSIM0"),
	SPRD_ADC_CHANNEL(97, 22, IIO_VOLTAGE, "VBATDLDO_VDDSIM0"),
	SPRD_ADC_CHANNEL(98, 22, IIO_VOLTAGE, "VBATDLDO_VDDCAMA1"),
	SPRD_ADC_CHANNEL(99, 22, IIO_VOLTAGE, "VBATDLDO_VDDCAMA1"),
	SPRD_ADC_CHANNEL(100, 22, IIO_VOLTAGE, "VBATALDO_VDD2V8"),
	SPRD_ADC_CHANNEL(101, 22, IIO_VOLTAGE, "VBATALDO_VDD2V8"),
	SPRD_ADC_CHANNEL(102, 22, IIO_VOLTAGE, "VBATDLDO_VDDCAMA0"),
	SPRD_ADC_CHANNEL(103, 22, IIO_VOLTAGE, "VBATDLDO_VDDCAMA0"),
	SPRD_ADC_CHANNEL(104, 23, IIO_VOLTAGE, "VBATBLDO_VDDSD"),
	SPRD_ADC_CHANNEL(105, 23, IIO_VOLTAGE, "VBATBLDO_VDDSD"),
	SPRD_ADC_CHANNEL(106, 23, IIO_VOLTAGE, "VBATBLDO_VDDSDIO"),
	SPRD_ADC_CHANNEL(107, 23, IIO_VOLTAGE, "VBATBLDO_VDDSDIO"),
	SPRD_ADC_CHANNEL(108, 23, IIO_VOLTAGE, "VBATBLDO_VDDEMMCCORE"),
	SPRD_ADC_CHANNEL(109, 23, IIO_VOLTAGE, "VBATBLDO_VDDEMMCCORE"),
	SPRD_ADC_CHANNEL(110, 23, IIO_VOLTAGE, "VBATBLDO_VDDUSB33"),
	SPRD_ADC_CHANNEL(111, 23, IIO_VOLTAGE, "VBATBLDO_VDDUSB33"),
	SPRD_ADC_CHANNEL(112, 23, IIO_VOLTAGE, "VBATBLDO_VDDWIFIPA"),
	SPRD_ADC_CHANNEL(113, 23, IIO_VOLTAGE, "VBATBLDO_VDDWIFIPA"),
	SPRD_ADC_CHANNEL(114, 23, IIO_VOLTAGE, "VBATBLDO_VDDSIM2"),
	SPRD_ADC_CHANNEL(115, 23, IIO_VOLTAGE, "VBATBLDO_VDDSIM2"),
	SPRD_ADC_CHANNEL(116, 24, IIO_VOLTAGE, "FLOATTING_24"),
	SPRD_ADC_CHANNEL(117, 24, IIO_VOLTAGE, "FLOATTING_24"),
	SPRD_ADC_CHANNEL(118, 25, IIO_VOLTAGE, "FLOATTING_25"),
	SPRD_ADC_CHANNEL(119, 25, IIO_VOLTAGE, "FLOATTING_25"),
	SPRD_ADC_CHANNEL(120, 26, IIO_VOLTAGE, "FLOATTING_26"),
	SPRD_ADC_CHANNEL(121, 26, IIO_VOLTAGE, "FLOATTING_26"),
	SPRD_ADC_CHANNEL(122, 27, IIO_VOLTAGE, "FLOATTING_27"),
	SPRD_ADC_CHANNEL(123, 27, IIO_VOLTAGE, "FLOATTING_27"),
	SPRD_ADC_CHANNEL(124, 28, IIO_CURRENT, "FLOATTING_28"),
	SPRD_ADC_CHANNEL(125, 28, IIO_VOLTAGE, "FLOATTING_28"),
	SPRD_ADC_CHANNEL(126, 29, IIO_VOLTAGE, "FLOATTING_29"),
	SPRD_ADC_CHANNEL(127, 29, IIO_VOLTAGE, "FLOATTING_29"),
	SPRD_ADC_CHANNEL(128, 30, IIO_VOLTAGE, "DP"),
	SPRD_ADC_CHANNEL(129, 30, IIO_VOLTAGE, "DP"),
	SPRD_ADC_CHANNEL(130, 31, IIO_VOLTAGE, "DM"),
	SPRD_ADC_CHANNEL(131, 31, IIO_VOLTAGE, "DM"),
};

static const struct iio_chan_spec sprd_sc2721_iio_channels[] = {
	SPRD_ADC_CHANNEL(0, 0, IIO_VOLTAGE, "ADCI0"),
	SPRD_ADC_CHANNEL(1, 0, IIO_VOLTAGE, "ADCI0"),
	SPRD_ADC_CHANNEL(2, 1, IIO_VOLTAGE, "ADCI1"),
	SPRD_ADC_CHANNEL(3, 1, IIO_VOLTAGE, "ADCI1"),
	SPRD_ADC_CHANNEL(4, 2, IIO_VOLTAGE, "ADCI2"),
	SPRD_ADC_CHANNEL(5, 2, IIO_VOLTAGE, "ADCI2"),
	SPRD_ADC_CHANNEL(6, 3, IIO_VOLTAGE, "ADCI3"),
	SPRD_ADC_CHANNEL(7, 3, IIO_VOLTAGE, "ADCI3"),
	SPRD_ADC_CHANNEL(8, 4, IIO_VOLTAGE, "ADCI4"),
	SPRD_ADC_CHANNEL(9, 4, IIO_VOLTAGE, "ADCI4"),
	SPRD_ADC_CHANNEL(10, 5, IIO_VOLTAGE, "VBATSENSE"),
	SPRD_ADC_CHANNEL(11, 5, IIO_VOLTAGE, "VBATSENSE"),
	SPRD_ADC_CHANNEL(12, 6, IIO_VOLTAGE, "FLOATING_6"),
	SPRD_ADC_CHANNEL(13, 6, IIO_VOLTAGE, "FLOATING_6"),
	SPRD_ADC_CHANNEL(14, 7, IIO_VOLTAGE, "TYPEC_CC1"),
	SPRD_ADC_CHANNEL(15, 7, IIO_VOLTAGE, "TYPEC_CC1"),
	SPRD_ADC_CHANNEL(16, 8, IIO_VOLTAGE, "THM"),
	SPRD_ADC_CHANNEL(17, 8, IIO_VOLTAGE, "THM"),
	SPRD_ADC_CHANNEL(18, 9, IIO_VOLTAGE, "TYPEC_CC2"),
	SPRD_ADC_CHANNEL(19, 9, IIO_VOLTAGE, "TYPEC_CC2"),
	SPRD_ADC_CHANNEL(20, 10, IIO_VOLTAGE, "FLOATING_10"),
	SPRD_ADC_CHANNEL(21, 10, IIO_VOLTAGE, "FLOATING_10"),
	SPRD_ADC_CHANNEL(22, 11, IIO_VOLTAGE, "FLOATING_11"),
	SPRD_ADC_CHANNEL(23, 11, IIO_VOLTAGE, "FLOATING_11"),
	SPRD_ADC_CHANNEL(24, 12, IIO_VOLTAGE, "FLOATING_12"),
	SPRD_ADC_CHANNEL(25, 12, IIO_VOLTAGE, "FLOATING_12"),
	SPRD_ADC_CHANNEL(26, 13, IIO_VOLTAGE, "DCDCOUT_CPU"),
	SPRD_ADC_CHANNEL(27, 13, IIO_VOLTAGE, "DCDCOUT_CPU"),
	SPRD_ADC_CHANNEL(28, 13, IIO_VOLTAGE, "DCDCOUT_MEM"),
	SPRD_ADC_CHANNEL(29, 13, IIO_VOLTAGE, "DCDCOUT_MEM"),
	SPRD_ADC_CHANNEL(30, 13, IIO_VOLTAGE, "DCDCOUT_GEN"),
	SPRD_ADC_CHANNEL(31, 13, IIO_VOLTAGE, "DCDCOUT_GEN"),
	SPRD_ADC_CHANNEL(32, 13, IIO_VOLTAGE, "DCDCOUT_CORE"),
	SPRD_ADC_CHANNEL(33, 13, IIO_VOLTAGE, "DCDCOUT_CORE"),
	SPRD_ADC_CHANNEL(34, 13, IIO_VOLTAGE, "DCDCOUT_WPA"),
	SPRD_ADC_CHANNEL(35, 13, IIO_VOLTAGE, "DCDCOUT_WPA"),
	SPRD_ADC_CHANNEL(36, 14, IIO_VOLTAGE, "VCHGSEN"),
	SPRD_ADC_CHANNEL(37, 14, IIO_VOLTAGE, "VCHGSEN"),
	SPRD_ADC_CHANNEL(38, 15, IIO_VOLTAGE, "VCHG_BG"),
	SPRD_ADC_CHANNEL(39, 15, IIO_VOLTAGE, "VCHG_BG"),
	SPRD_ADC_CHANNEL(40, 16, IIO_VOLTAGE, "PROG2ADC"),
	SPRD_ADC_CHANNEL(41, 16, IIO_VOLTAGE, "PROG2ADC"),
	SPRD_ADC_CHANNEL(42, 17, IIO_VOLTAGE, "FLOATTING_17"),
	SPRD_ADC_CHANNEL(43, 17, IIO_VOLTAGE, "FLOATTING_17"),
	SPRD_ADC_CHANNEL(44, 18, IIO_VOLTAGE, "FLOATTING_18"),
	SPRD_ADC_CHANNEL(45, 18, IIO_VOLTAGE, "FLOATTING_18"),
	SPRD_ADC_CHANNEL(46, 19, IIO_VOLTAGE, "SDAVDD"),
	SPRD_ADC_CHANNEL(47, 19, IIO_VOLTAGE, "SDAVDD"),
	SPRD_ADC_CHANNEL(48, 20, IIO_VOLTAGE, "HEADMIC_IN_DET"),
	SPRD_ADC_CHANNEL(49, 20, IIO_VOLTAGE, "HEADMIC_IN_DET"),
	SPRD_ADC_CHANNEL(50, 20, IIO_CURRENT, "HEADMIC_HEADSET_L_INT"),
	SPRD_ADC_CHANNEL(51, 20, IIO_VOLTAGE, "HEADMIC_HEADSET_L_INT"),
	SPRD_ADC_CHANNEL(52, 20, IIO_VOLTAGE, "HEADMIC_VDDVB"),
	SPRD_ADC_CHANNEL(53, 20, IIO_VOLTAGE, "HEADMIC_VDDVB"),
	SPRD_ADC_CHANNEL(54, 20, IIO_VOLTAGE, "HEADMIC_VDDPA"),
	SPRD_ADC_CHANNEL(55, 20, IIO_VOLTAGE, "HEADMIC_VDDPA"),
	SPRD_ADC_CHANNEL(56, 20, IIO_VOLTAGE, "HEADMIC_MICBIAS"),
	SPRD_ADC_CHANNEL(57, 20, IIO_VOLTAGE, "HEADMIC_MICBIAS"),
	SPRD_ADC_CHANNEL(58, 20, IIO_VOLTAGE, "HEADMIC_HEADMICBIAS"),
	SPRD_ADC_CHANNEL(59, 20, IIO_VOLTAGE, "HEADMIC_HEADMICBIAS"),
	SPRD_ADC_CHANNEL(60, 21, IIO_VOLTAGE, "LDO_CALOUT0"),
	SPRD_ADC_CHANNEL(61, 21, IIO_VOLTAGE, "LDO_CALOUT0"),
	SPRD_ADC_CHANNEL(62, 22, IIO_VOLTAGE, "LDO_CALOUT1"),
	SPRD_ADC_CHANNEL(63, 22, IIO_VOLTAGE, "LDO_CALOUT1"),
	SPRD_ADC_CHANNEL(64, 23, IIO_VOLTAGE, "LDO_CALOUT2"),
	SPRD_ADC_CHANNEL(65, 23, IIO_VOLTAGE, "LDO_CALOUT2"),
	SPRD_ADC_CHANNEL(66, 24, IIO_VOLTAGE, "FLOATTING_24"),
	SPRD_ADC_CHANNEL(67, 24, IIO_VOLTAGE, "FLOATTING_24"),
	SPRD_ADC_CHANNEL(68, 25, IIO_VOLTAGE, "FLOATTING_25"),
	SPRD_ADC_CHANNEL(69, 25, IIO_VOLTAGE, "FLOATTING_25"),
	SPRD_ADC_CHANNEL(70, 26, IIO_VOLTAGE, "FLOATTING_26"),
	SPRD_ADC_CHANNEL(71, 26, IIO_VOLTAGE, "FLOATTING_26"),
	SPRD_ADC_CHANNEL(72, 27, IIO_VOLTAGE, "FLOATTING_27"),
	SPRD_ADC_CHANNEL(73, 27, IIO_VOLTAGE, "FLOATTING_27"),
	SPRD_ADC_CHANNEL(74, 28, IIO_CURRENT, "FLOATTING_28"),
	SPRD_ADC_CHANNEL(75, 28, IIO_VOLTAGE, "FLOATTING_28"),
	SPRD_ADC_CHANNEL(76, 29, IIO_VOLTAGE, "FLOATTING_29"),
	SPRD_ADC_CHANNEL(77, 29, IIO_VOLTAGE, "FLOATTING_29"),
	SPRD_ADC_CHANNEL(78, 30, IIO_VOLTAGE, "DP"),
	SPRD_ADC_CHANNEL(79, 30, IIO_VOLTAGE, "DP"),
	SPRD_ADC_CHANNEL(80, 31, IIO_VOLTAGE, "DM"),
	SPRD_ADC_CHANNEL(81, 31, IIO_VOLTAGE, "DM"),
};

static const struct pmic_adc_para sc2723_adc_para = {
	.small_adc_p0 = 819,
	.small_adc_p1 = 82,
	.small_vol_p0 = 1000,
	.small_vol_p1 = 100,
	.small_cal_efuse_blk = 9,
	.small_cal_efuse_bit = 16,
	.big_adc_p0 = 711,
	.big_adc_p1 = 830,
	.big_vol_p0 = 3600,
	.big_vol_p1 = 4200,
	.big_cal_efuse_blk = 7,
	.big_cal_efuse_bit = 16,
	.blk_width = 8,
	.adc_data_off = 128,
};

static const struct pmic_adc_para sc2731_adc_para = {
	.small_adc_p0 = 838,
	.small_adc_p1 = 84,
	.small_vol_p0 = 1000,
	.small_vol_p1 = 100,
	.small_cal_efuse_blk = 19,
	.small_cal_efuse_bit = 16,
	.big_adc_p0 = 728,
	.big_adc_p1 = 850,
	.big_vol_p0 = 3600,
	.big_vol_p1 = 4200,
	.big_cal_efuse_blk = 18,
	.big_cal_efuse_bit = 16,
	.blk_width = 16,
	.adc_data_off = 128,
};

static const struct sprd_chip_info sprd_chip_info_tbl[] = {
	[SC2723_ADC] = {
			.channels = sprd_sc2723_iio_channels,
			.num_channels = ARRAY_SIZE(sprd_sc2723_iio_channels),
			.para = &sc2723_adc_para,
			},
	[SC2731_ADC] = {
			.channels = sprd_sc2731_iio_channels,
			.num_channels = ARRAY_SIZE(sprd_sc2731_iio_channels),
			.para = &sc2731_adc_para,
			},
	[SC2721_ADC] = {
			.channels = sprd_sc2721_iio_channels,
			.num_channels = ARRAY_SIZE(sprd_sc2721_iio_channels),
			.para = &sc2731_adc_para,
			},
};

static ssize_t sprd_read_adc_info(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sprd_pmic_adc *adc_data = iio_priv(indio_dev);
	int adc_value = 0;
	int adc_vol = 0;

	adc_value =
	    sprd_get_average_adc_data(adc_data->channel, adc_data->scale,
				      adc_data->sample);
	adc_vol =
	    sprd_get_processed_adc_vol_value(adc_data->channel,
					     adc_data->channel, adc_data->scale,
					     adc_data->sample);

	return sprintf(buf,
		       "channel = %d,scale = %d,adc_data = %d,adc_vol =%d,cal_type =%d\n",
		       adc_data->channel, adc_data->scale, adc_value, adc_vol,
		       cal_type);
}

static ssize_t sprd_read_adc_vol_ratio(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	uint32_t div_num = 0, div_den = 0;
	int voltage_ratio;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sprd_pmic_adc *adc_data = iio_priv(indio_dev);

	sprd_adc_get_vol_ratio(adc_data->channel, adc_data->scale,
			       &div_num, &div_den);
	voltage_ratio = (div_num << 16) | (div_den & 0xFFFF);
	return sprintf(buf, "%d\n", voltage_ratio);
}

static ssize_t sprd_set_adc_channel(struct device *dev,
				    struct device_attribute *dev_attr,
				    const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sprd_pmic_adc *adc_data = iio_priv(indio_dev);
	int len = 0, temp_channel = 0;

	len = sscanf(buf, "%d", &temp_channel);
	mutex_lock(&adc_data->lock);
	adc_data->channel = temp_channel;
	mutex_unlock(&adc_data->lock);
	return count;
}

static ssize_t sprd_set_adc_scale(struct device *dev,
				  struct device_attribute *dev_attr,
				  const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sprd_pmic_adc *adc_data = iio_priv(indio_dev);
	int len = 0, temp_scale = 0;

	len = sscanf(buf, "%d", &temp_scale);
	mutex_lock(&adc_data->lock);
	adc_data->scale = temp_scale;
	mutex_unlock(&adc_data->lock);
	return count;
}

static ssize_t sprd_enable_adc_isen(struct device *dev,
				    struct device_attribute *dev_attr,
				    const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sprd_pmic_adc *adc_data = iio_priv(indio_dev);
	int len = 0, temp_isen = 0;

	len = sscanf(buf, "%d", &temp_isen);
	mutex_lock(&adc_data->lock);
	sprd_adc_set_current(1, temp_isen);
	mutex_unlock(&adc_data->lock);
	return count;
}

static ssize_t sprd_disable_adc_isen(struct device *dev,
				     struct device_attribute *dev_attr,
				     const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sprd_pmic_adc *adc_data = iio_priv(indio_dev);
	int len = 0, temp_isen = 0;

	len = sscanf(buf, "%d", &temp_isen);
	mutex_lock(&adc_data->lock);
	sprd_adc_set_current(0, temp_isen);
	mutex_unlock(&adc_data->lock);
	return count;
}

static IIO_DEVICE_ATTR(adc_info,
		       S_IRUGO | S_IWUSR, sprd_read_adc_info, NULL, 0);

static IIO_DEVICE_ATTR(adc_channel_set,
		       S_IRUGO | S_IWUSR, NULL, sprd_set_adc_channel, 0);

static IIO_DEVICE_ATTR(adc_scale_set,
		       S_IRUGO | S_IWUSR, NULL, sprd_set_adc_scale, 0);

static IIO_DEVICE_ATTR(adc_isen_enable,
		       S_IRUGO | S_IWUSR, NULL, sprd_enable_adc_isen, 0);

static IIO_DEVICE_ATTR(adc_isen_disable,
		       S_IRUGO | S_IWUSR, NULL, sprd_disable_adc_isen, 0);

static IIO_DEVICE_ATTR(adc_vol_ratio,
		       S_IRUGO | S_IWUSR, sprd_read_adc_vol_ratio, NULL, 0);

static struct attribute *sprdadc_attributes[] = {
	&iio_dev_attr_adc_info.dev_attr.attr,
	&iio_dev_attr_adc_channel_set.dev_attr.attr,
	&iio_dev_attr_adc_scale_set.dev_attr.attr,
	&iio_dev_attr_adc_isen_enable.dev_attr.attr,
	&iio_dev_attr_adc_isen_disable.dev_attr.attr,
	&iio_dev_attr_adc_vol_ratio.dev_attr.attr,
	NULL,
};

static const struct attribute_group sprdadc_attribute_group = {
	.attrs = sprdadc_attributes,
};

static const struct iio_info sprd_adc_iio_info = {
	.read_raw = &sprd_pmic_read_raw,
	.attrs = &sprdadc_attribute_group,
	.driver_module = THIS_MODULE,
};

static int sprd_pmic_adc_probe(struct platform_device *pdev)
{
	struct sprd_pmic_adc *sprd_adc_data = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct iio_dev *indio_dev = NULL;
	const struct of_device_id *of_id;
	enum sprd_pmic_adc_type adc_type;
	int ret = -ENODEV;
	u32 value = 0;

	dev_info(&pdev->dev, "sprd adc probe start\n");
	if (!np) {
		dev_err(&pdev->dev, "device node not found\n");
		return -ENODEV;
	}
	of_id = of_match_node(pmic_adc_of_match, pdev->dev.of_node);
	if (!of_id) {
		dev_err(&pdev->dev, "Get the adc of device id failed!\n");
		return -ENODEV;
	}
	adc_type = (enum sprd_pmic_adc_type)of_id->data;
	indio_dev =
	    devm_iio_device_alloc(&pdev->dev, sizeof(struct sprd_pmic_adc));
	if (!indio_dev) {
		dev_err(&pdev->dev, "failed allocating iio device\n");
		return -ENOMEM;
	}
	sprd_adc_data = iio_priv(indio_dev);
	ret = of_property_read_u32(np, "reg", &value);
	if (ret) {
		dev_err(&pdev->dev, "sprd get pmic adc reg failed!\n");
		return -EINVAL;
	}

	sprd_adc_data->hw_lock =
	    of_hwspin_lock_request(pdev->dev.of_node, "pmic_adc");
	if (!sprd_adc_data->hw_lock) {
		dev_err(&pdev->dev, "pmic_adc not get hardware spinlock.\n");
		return -ENXIO;
	}

	sprd_adc_data->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!sprd_adc_data->regmap) {
		dev_err(&pdev->dev,
			"%s :fail regmap property for pmic adc .", __func__);
		ret = -ENODEV;
		return ret;
	}

	sprd_adc_data->adc_base = (unsigned long)value;
	sprd_adc_data->channel = 5;
	sprd_adc_data->sample = 15;
	sprd_adc_data->cal_type = SPRD_AUXADC_CAL_NO;
	sprd_adc_data->type = adc_type;
	sprd_adc_data->info = &sprd_chip_info_tbl[adc_type];
	mutex_init(&sprd_adc_data->lock);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = SPRD_MODULE_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &sprd_adc_iio_info;
	indio_dev->channels = sprd_adc_data->info->channels;
	indio_dev->num_channels = sprd_adc_data->info->num_channels;
	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "could not register iio (adc)");
		return ret;
	}

	pmic_adc = sprd_adc_data;
	platform_set_drvdata(pdev, indio_dev);
	sprd_pmic_adc_init(sprd_adc_data);
	dev_info(&pdev->dev, "sprd adc probe end\n");
	return 0;
}

static int sprd_pmic_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	devm_iio_device_unregister(&pdev->dev, indio_dev);
	return 0;
}

static struct platform_driver sprd_adc_driver = {
	.probe = sprd_pmic_adc_probe,
	.remove = sprd_pmic_adc_remove,
	.driver = {
		   .name = SPRD_MODULE_NAME,
		   .of_match_table = of_match_ptr(pmic_adc_of_match),
		   },
};

static int __init sprd_adc_driver_init(void)
{
	return platform_driver_register(&sprd_adc_driver);
}

static void __exit sprd_adc_driver_exit(void)
{
	platform_driver_unregister(&sprd_adc_driver);
}

subsys_initcall(sprd_adc_driver_init);
module_exit(sprd_adc_driver_exit);

MODULE_AUTHOR("Freeman Liu <freeman.liu@spreadtrum.com>");
MODULE_DESCRIPTION("Spreadtrum sc2723-adc Driver");
MODULE_LICENSE("GPL");
