/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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
#include "sprd_adc.h"

#define ADC_CFG_CTL		0x4
#define ADC_SW_CH_CFG		0x8
#define ADC_DAT			0x50
#define ADC_CFG_INT_EN		0x54
#define ADC_CFG_INT_CLR		0x58
#define ADC_CFG_INT_STS		0x5c
#define ADC_CFG_INT_RAW		0x60
#define AUXAD_CTL0		0xb0
#define AUXAD_CTL1		0xb4

/* ADC_CFG_CTL */
#define BIT_EN_ADC			BIT(0)
#define BIT_SW_CH_ON			BIT(1)
#define BIT_ADC_BIT_MODE(_X_)		(((_X_) & 0x1) << 2)
#define BIT_ADC_BIT_MODE_MASK		BIT_ADC_BIT_MODE(1)
#define BIT_SW_CH_RUN_NUM(_X_)		((((_X_) - 1) & GENMASK(3, 0)) << 4)
#define BIT_RG_AUXAD_AVERAGE(_X_)	(((_X_) & GENMASK(2, 0)) << 8)
#define BIT_ADC_OFFSET_CAL_EN		BIT(12)

/* ADC_SW_CH_CFG */
#define BIT_CH_ID(_X_)			((_X_) & GENMASK(4, 0))
#define BIT_CH_SLOW(_X_)		(((_X_) & 0x1) << 6)
#define BIT_CH_SCALE(_X_)		(((_X_) & GENMASK(2, 0)) << 9)

#define ADC_MAX_SAMPLE_NUM		0x10
#define ADC_MESURE_NUMBER		2
#define RATIO(n, d)			(((n) << 16) | (d))
#define HWSPINLOCK_TIMEOUT		5000
#define CAL_DATA_MASK			GENMASK(15, 0)
#define DETA_MASK			GENMASK(23, 0)
#define EFUSE_DATA_MASK			GENMASK(7, 0)
#define SAMPLE_BIT_MASK			GENMASK(11, 0)
#define RATIO_PARA_MASK			GENMASK(11, 0)
#define RATIO_PARA_OFFSET		16
#define EFUSE_DATA_OFFSET		16
#define ADC_CAL_PONIT_SHIFT		8
#define ADC_CAL_DATA_SHIFT		2
#define ADC_REG_END			0xb4

static DEFINE_MUTEX(adc_mutex);
static int cal_type = SPRD_AUXADC_CAL_NO;

struct sprd_chip_info {
	const struct iio_chan_spec *channels;
	const struct pmic_adc_para *para;
	u32 num_channels;
};

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

static const struct pmic_adc_para sc2720_adc_para = {
	.small_adc_p0 = 833,
	.small_adc_p1 = 80,
	.small_vol_p0 = 1000,
	.small_vol_p1 = 100,
	.small_cal_efuse_blk = 19,
	.small_cal_efuse_bit = 16,
	.big_adc_p0 = 856,
	.big_adc_p1 = 733,
	.big_vol_p0 = 4200,
	.big_vol_p1 = 3600,
	.big_cal_efuse_blk = 18,
	.big_cal_efuse_bit = 16,
	.blk_width = 16,
	.adc_data_off = 128,
};

static inline int sprd_adc_reg_read(struct sprd_pmic_adc *padc, u32 reg)
{
	u32 val;

	regmap_read(padc->regmap, padc->adc_base + reg, &val);
	return val;
}

static inline int sprd_adc_reg_write(struct sprd_pmic_adc *padc, u32 reg,
				     u32 val)
{
	return regmap_write(padc->regmap, padc->adc_base + reg, val);
}

static inline int sprd_adc_reg_set(struct sprd_pmic_adc *padc, u32 reg,
				   u32 mask, u32 bits)
{
	return regmap_update_bits(padc->regmap, reg, mask, bits);
}

static inline int sprd_adc_reg_clr(struct sprd_pmic_adc *padc, u32 reg,
				   u32 mask, u32 bits)
{
	return regmap_update_bits(padc->regmap, reg, mask, ~bits);
}

static int sprd_adc_lock(struct sprd_pmic_adc *padc)
{
	int ret;

	mutex_lock(&adc_mutex);
	ret = hwspin_lock_no_swlock_timeout(padc->hw_lock, HWSPINLOCK_TIMEOUT);
	if (ret) {
		dev_err(padc->dev, "pmic adc:lock the hwlock failed.\n");
		mutex_unlock(&adc_mutex);
	}
	return ret;
}

static void sprd_adc_unlock(struct sprd_pmic_adc *padc)
{
	hwspin_unlock_no_swlock(padc->hw_lock);
	mutex_unlock(&adc_mutex);
}

#if defined(CONFIG_OTP_SPRD_PMIC_EFUSE)

static int sprd_adc_small_scale_efuse_cal(struct sprd_pmic_adc *padc)
{
	const struct pmic_adc_para *para = padc->info->para;
	u32 deta, adc_data, sample_cal_dvale = 4;
	u32 bit_index = para->small_cal_efuse_blk * para->blk_width;

	deta = sprd_pmic_efuse_bits_read(bit_index, para->small_cal_efuse_bit);
	deta &= DETA_MASK;
	if (!deta)
		return -EINVAL;

	if (padc->type == SC2720_ADC) {
		/* adc 0.1V */
		adc_data = (((deta >> ADC_CAL_PONIT_SHIFT) & EFUSE_DATA_MASK) +
			    para->small_adc_p1 - para->adc_data_off) *
			    sample_cal_dvale;
		small_sclae_cal.p1_vol = para->small_vol_p1;
		small_sclae_cal.p1_adc = adc_data & CAL_DATA_MASK;

		/* adc 1.0V */
		adc_data = ((deta & EFUSE_DATA_MASK) + para->small_adc_p0 -
			    para->adc_data_off) * sample_cal_dvale;
		small_sclae_cal.p0_vol = para->small_vol_p0;
		small_sclae_cal.p0_adc = adc_data & CAL_DATA_MASK;
		dev_info(padc->dev, "adc efuse small cal %d,%d,%d,%d\n",
		small_sclae_cal.p0_vol, small_sclae_cal.p0_adc,
		small_sclae_cal.p1_vol, small_sclae_cal.p1_adc);
	} else {
		dev_err(padc->dev, "adc type is unknown\n");
	}

	return 0;
}

static int sprd_adc_big_scale_efuse(struct sprd_pmic_adc *padc)
{
	const struct pmic_adc_para *para = padc->info->para;
	u32 bit_index = para->big_cal_efuse_blk * para->blk_width;
	u32 deta, cal_data;

	deta = sprd_pmic_efuse_bits_read(bit_index, para->big_cal_efuse_bit);
	deta &= DETA_MASK;
	if (!deta)
		return -EINVAL;

	/* adc 3.6V */
	cal_data = ((deta >> ADC_CAL_PONIT_SHIFT) & EFUSE_DATA_MASK) +
		para->big_adc_p1 - para->adc_data_off;
	big_scale_cal.p1_vol = para->big_vol_p1;
	big_scale_cal.p1_adc = (cal_data << ADC_CAL_DATA_SHIFT) & CAL_DATA_MASK;

	/* adc 4.2V */
	cal_data = (deta & EFUSE_DATA_MASK) + para->big_adc_p0 -
		para->adc_data_off;
	big_scale_cal.p0_vol = para->big_vol_p0;
	big_scale_cal.p0_adc = (cal_data << ADC_CAL_DATA_SHIFT) & CAL_DATA_MASK;
	dev_info(padc->dev, "adc efuse big cal %d,%d,%d,%d\n",
		 big_scale_cal.p0_vol, big_scale_cal.p0_adc,
		 big_scale_cal.p1_vol, big_scale_cal.p1_adc);
	return 0;
}
#endif

static int __init sprd_adc_cal_proc(char *str)
{
	u32 adc_data[2] = { 0, 0 };
	char *cali_data = &str[1];
	int ret;

	if (!str)
		return -EINVAL;

	pr_info("adc_cal%s!\n", str);
	ret = sscanf(cali_data, "%d,%d", &adc_data[0], &adc_data[1]);
	if (ret != 2) {
		pr_err("adc cal proc invalid command\n");
		return -EINVAL;
	}

	big_scale_cal.p0_vol = adc_data[0] & CAL_DATA_MASK;
	big_scale_cal.p0_adc = (adc_data[0] >> EFUSE_DATA_OFFSET) &
		CAL_DATA_MASK;
	big_scale_cal.p1_vol = adc_data[1] & CAL_DATA_MASK;
	big_scale_cal.p1_adc = (adc_data[1] >> EFUSE_DATA_OFFSET) &
		CAL_DATA_MASK;
	big_scale_cal.cal_type = SPRD_AUXADC_CAL_NV;
	pr_info("adc cal cmdline adc_data[0]: 0x%x, adc_data[1]:0x%x\n",
		 adc_data[0], adc_data[1]);
	cal_type = SPRD_AUXADC_CAL_NV;

	return 1;
}

__setup("adc_cal", sprd_adc_cal_proc);

static void sprd_adc_enable(struct sprd_pmic_adc *padc)
{
	sprd_adc_reg_set(padc, ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_ADC_EN,
			 BIT_ANA_ADC_EN);
	sprd_adc_reg_set(padc, ANA_REG_GLB_ARM_CLK_EN,
			 BIT_CLK_AUXADC_EN | BIT_CLK_AUXAD_EN,
			 BIT_CLK_AUXADC_EN | BIT_CLK_AUXAD_EN);
	sprd_adc_reg_set(padc, ANA_REG_GLB_XTL_WAIT_CTRL, BIT_XTL_EN,
			 BIT_XTL_EN);
}

static void sprd_adc_dump_register(struct sprd_pmic_adc *padc)
{
	u32 base = 0;
	u32 end = base + ADC_REG_END;

	dev_dbg(padc->dev, "sci_adc_dump_register begin\n");
	for (; base < end; base += 0x4) {
		dev_info(padc->dev, "adc base = 0x%x, value = 0x%x\n", base,
			 sprd_adc_reg_read(padc, base));
	}
	dev_dbg(padc->dev, "sci_adc_dump_register end\n");
}

static void sprd_adc_init(struct sprd_pmic_adc *pmic_adc)
{
	sprd_adc_enable(pmic_adc);
	sprd_adc_reg_write(pmic_adc, ADC_CFG_INT_EN, 0x0);
	sprd_adc_reg_write(pmic_adc, ADC_CFG_INT_CLR, 0x1);

#if defined(CONFIG_OTP_SPRD_PMIC_EFUSE)
	if (cal_type == SPRD_AUXADC_CAL_NO) {
		if (!sprd_adc_big_scale_efuse(pmic_adc)) {
			pmic_adc->bigscale_cal_type = SPRDBIG_AUXADC_CAL_CHIP;
			cal_type = SPRD_AUXADC_CAL_CHIP;
		}
		if (!sprd_adc_small_scale_efuse_cal(pmic_adc)) {
			pmic_adc->litscale_cal_type = SPRDLIT_AUXADC_CAL_CHIP;
			cal_type = SPRD_AUXADC_CAL_CHIP;
		}
	}
#endif
}

static int sprd_adc_get_values(struct sprd_pmic_adc *padc,
			       struct adc_sample_data *adc)
{
	int ret, num, cnt = 30;
	u32 val, status;
	int *pbuf;

	if (!adc)
		return -EINVAL;

	pbuf = adc->pbuf;
	if (!pbuf)
		return -EINVAL;

	num = adc->sample_num;
	if (num > ADC_MAX_SAMPLE_NUM)
		return -EINVAL;

	ret = sprd_adc_lock(padc);
	if (ret)
		return ret;

	val = BIT_CH_SLOW(adc->sample_speed);
	val |= BIT_CH_SCALE(adc->scale);
	val |= BIT_CH_ID(adc->channel_id);
	sprd_adc_reg_write(padc, ADC_SW_CH_CFG, val);
	sprd_adc_reg_write(padc, ADC_CFG_INT_CLR, 0x1);

	val = 0;
	val |= BIT_SW_CH_RUN_NUM(num);
	val |= BIT_EN_ADC;
	val |= BIT_ADC_BIT_MODE(adc->sample_bits);
	val |= BIT_RG_AUXAD_AVERAGE(0x3);
	val |= BIT_ADC_OFFSET_CAL_EN;
	val |= BIT_SW_CH_ON;
	sprd_adc_reg_write(padc, ADC_CFG_CTL, val);

	status = sprd_adc_reg_read(padc, ADC_CFG_INT_RAW);
	while ((!sprd_adc_reg_read(padc, ADC_CFG_INT_RAW)) && cnt--)
		udelay(20);

	if (cnt == -1) {
		ret = -ETIMEDOUT;
		sprd_adc_dump_register(padc);
		goto exit;
	}

	while (num--)
		*pbuf++ = sprd_adc_reg_read(padc, ADC_DAT) & SAMPLE_BIT_MASK;

exit:
	val = sprd_adc_reg_read(padc, ADC_CFG_CTL);
	val &= ~BIT_EN_ADC;
	sprd_adc_reg_write(padc, ADC_CFG_CTL, val);
	sprd_adc_unlock(padc);

	return ret;
}

static int sprd_adc_sc2720_ratio(u32 channel, int scale)
{
	if (channel >= ADC_CHANNEL_MAX)
		return RATIO(1, 1);

	switch (channel) {
	case ADC_CHANNEL14:
		switch (scale) {
		case 0:
			return RATIO(68, 900);
		case 1:
			return RATIO(68, 1760);
		case 2:
			return RATIO(68, 2327);
		case 3:
			return RATIO(68, 3654);
		default:
			return RATIO(1, 1);
		}
	case ADC_CHANNEL16:
		switch (scale) {
		case 0:
			return RATIO(48, 100);
		case 1:
			return RATIO(480, 1955);
		case 2:
			return RATIO(480, 2586);
		case 3:
			return RATIO(48, 406);
		default:
			return RATIO(1, 1);
		}
	case ADC_CHANNEL21:
	case ADC_CHANNEL22:
	case ADC_CHANNEL23:
		switch (scale) {
		case 0:
			return RATIO(3, 8);
		case 1:
			return RATIO(375, 1955);
		case 2:
			return RATIO(375, 2586);
		case 3:
			return RATIO(300, 3248);
		default:
			return RATIO(1, 1);
		}
	default:
		switch (scale) {
		case 0:
			return RATIO(1, 1);
		case 1:
			return RATIO(1000, 1955);
		case 2:
			return RATIO(1000, 2586);
		case 3:
			return RATIO(100, 406);
		default:
			return RATIO(1, 1);
		}
	}
	return RATIO(1, 1);
}

static int sprd_adc_def_ratio(u32 channel, int scale)
{
	return RATIO(1, 1);
}

static void sprd_adc_vol_ratio(struct sprd_pmic_adc *padc, u32 channel,
			       int scale, u32 *div_numerators,
			       u32 *div_denominators)
{
	u32 ratio;

	switch (padc->type) {
	case SC2720_ADC:
		ratio = sprd_adc_sc2720_ratio(channel, scale);
		break;
	default:
		ratio = sprd_adc_def_ratio(channel, scale);

	}
	*div_numerators = ratio >> RATIO_PARA_OFFSET;
	*div_denominators = ratio << RATIO_PARA_OFFSET >> RATIO_PARA_OFFSET;
}

static int sprd_adc_raw_value(struct sprd_pmic_adc *padc, u32 channel,
			      int scale, int sample_num)
{
	struct adc_sample_data adc;
	int result;

	adc.channel_id = channel;
	adc.pbuf = &result;
	adc.sample_bits = 1;
	adc.sample_num = sample_num;
	adc.sample_speed = 0;
	adc.scale = scale;

	if (sprd_adc_get_values(padc, &adc) != 0) {
		dev_err(padc->dev, "sci_adc_get_value, return error\n");
		result = 0;
	}

	return result;
}


static u32 sprd_adc_cal_vol(u32 cal_denominators, u32 cal_numerators,
			   u32 denominators, u32 numerators, u32 vol)
{
	u32 result, m, n;

	n = cal_denominators * numerators;
	m = vol * cal_numerators * denominators;
	result = (m + n / 2) / n;
	return result;
}

static int sprd_adc_to_vol(u16 p0_vol, u16 p1_vol,
			   u16 p0_adc, u16 p1_adc, u16 adcval)
{
	int vol;

	vol = p0_vol - p1_vol;
	vol = vol * (adcval - p1_adc);
	vol = vol / (p0_adc - p1_adc);
	vol = vol + p1_vol;
	if (vol < 0)
		vol = 0;

	return vol;
}

static u32 sprd_adc_big_scale_vol(struct sprd_pmic_adc *padc, u32 channel,
				  int scale, u32 adcval)
{
	u32 vol;
	u32 bat_numerators, bat_denominators;
	u32 numerators, denominators;

	vol = sprd_adc_to_vol(big_scale_cal.p0_vol, big_scale_cal.p1_vol,
				    big_scale_cal.p0_adc, big_scale_cal.p1_adc,
				    adcval);
	sprd_adc_vol_ratio(padc, ADC_CHANNEL_VBATSENSE, 3,
			   &bat_numerators, &bat_denominators);
	sprd_adc_vol_ratio(padc, channel, scale, &numerators, &denominators);
	dev_info(padc->dev, "adc chn %d val %d vol %d\n", channel, adcval, vol);

	return sprd_adc_cal_vol(bat_denominators, bat_numerators, denominators,
			       numerators, vol);
}

static u32 sprd_adc_small_scale_vol(struct sprd_pmic_adc *padc, u32 channel,
				    int scale, u32 adcval)
{
	u32 vol;
	u32 cal_numerators, cal_denominators;
	u32 numerators, denominators;

	vol = sprd_adc_to_vol(small_sclae_cal.p0_vol, small_sclae_cal.p1_vol,
			      small_sclae_cal.p0_adc, small_sclae_cal.p1_adc,
			      adcval);
	sprd_adc_vol_ratio(padc, ADC_CHANNEL_SMALL_CAL, 0,
			   &cal_numerators, &cal_denominators);
	sprd_adc_vol_ratio(padc, channel, scale, &numerators, &denominators);
	dev_info(padc->dev, "adc chn %d val %d vol %d\n", channel, adcval, vol);

	return sprd_adc_cal_vol(cal_denominators, cal_numerators, denominators,
				numerators, vol);
}

static int sprd_adc_processed_vol(struct sprd_pmic_adc *padc,
				  u32 chan, int scale, int sample_num)
{
	int adc, vol;

	adc = sprd_adc_raw_value(padc, chan, scale, sample_num);
	if (chan == 5)
		vol = sprd_adc_big_scale_vol(padc, chan, scale, adc);
	else
		vol = sprd_adc_small_scale_vol(padc, chan, scale, adc);

	return vol;
}

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

static const struct iio_chan_spec sprd_common_iio_channels[] = {
	SPRD_ADC_CHANNEL(0, 0, IIO_VOLTAGE, "ADCI0"),
	SPRD_ADC_CHANNEL(1, 1, IIO_VOLTAGE, "ADCI1"),
	SPRD_ADC_CHANNEL(2, 2, IIO_VOLTAGE, "ADCI2"),
	SPRD_ADC_CHANNEL(3, 3, IIO_VOLTAGE, "ADCI3"),
	SPRD_ADC_CHANNEL(4, 4, IIO_VOLTAGE, "ADCI4"),
	SPRD_ADC_CHANNEL(5, 5, IIO_VOLTAGE, "ADCI5"),
	SPRD_ADC_CHANNEL(6, 6, IIO_VOLTAGE, "ADCI6"),
	SPRD_ADC_CHANNEL(7, 7, IIO_VOLTAGE, "ADCI7"),
	SPRD_ADC_CHANNEL(8, 8, IIO_VOLTAGE, "ADCI8"),
	SPRD_ADC_CHANNEL(9, 9, IIO_VOLTAGE, "ADCI9"),
	SPRD_ADC_CHANNEL(10, 10, IIO_VOLTAGE, "ADCI10"),
	SPRD_ADC_CHANNEL(11, 11, IIO_VOLTAGE, "ADCI11"),
	SPRD_ADC_CHANNEL(12, 12, IIO_VOLTAGE, "ADCI12"),
	SPRD_ADC_CHANNEL(13, 13, IIO_VOLTAGE, "ADCI13"),
	SPRD_ADC_CHANNEL(14, 14, IIO_VOLTAGE, "ADCI14"),
	SPRD_ADC_CHANNEL(15, 15, IIO_VOLTAGE, "ADCI15"),
	SPRD_ADC_CHANNEL(16, 16, IIO_VOLTAGE, "ADCI16"),
	SPRD_ADC_CHANNEL(17, 17, IIO_VOLTAGE, "ADCI17"),
	SPRD_ADC_CHANNEL(18, 18, IIO_VOLTAGE, "ADCI18"),
	SPRD_ADC_CHANNEL(19, 19, IIO_VOLTAGE, "ADCI19"),
	SPRD_ADC_CHANNEL(20, 20, IIO_VOLTAGE, "ADCI20"),
	SPRD_ADC_CHANNEL(21, 21, IIO_VOLTAGE, "ADCI21"),
	SPRD_ADC_CHANNEL(22, 22, IIO_VOLTAGE, "ADCI22"),
	SPRD_ADC_CHANNEL(23, 23, IIO_VOLTAGE, "ADCI23"),
	SPRD_ADC_CHANNEL(24, 24, IIO_VOLTAGE, "ADCI24"),
	SPRD_ADC_CHANNEL(25, 25, IIO_VOLTAGE, "ADCI25"),
	SPRD_ADC_CHANNEL(26, 26, IIO_VOLTAGE, "ADCI26"),
	SPRD_ADC_CHANNEL(27, 27, IIO_VOLTAGE, "ADCI27"),
	SPRD_ADC_CHANNEL(28, 28, IIO_VOLTAGE, "ADCI28"),
	SPRD_ADC_CHANNEL(29, 29, IIO_VOLTAGE, "ADCI29"),
	SPRD_ADC_CHANNEL(30, 30, IIO_VOLTAGE, "ADCI30"),
	SPRD_ADC_CHANNEL(31, 31, IIO_VOLTAGE, "ADCI31"),
};

static const struct sprd_chip_info sprd_chip_info_tbl[] = {
	[SC2720_ADC] = {
		.channels = sprd_common_iio_channels,
		.num_channels = ARRAY_SIZE(sprd_common_iio_channels),
		.para = &sc2720_adc_para,
	},
};

static int sprd_adc_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long m)
{
	struct sprd_pmic_adc *padc = iio_priv(indio_dev);
	int scale;

	scale = padc->channel_scale[chan->address];
	if ((chan->channel < 0)
	    || (chan->channel > padc->info->num_channels))
		return -EINVAL;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		*val = sprd_adc_raw_value(padc, chan->address, scale, 1);
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&indio_dev->mlock);
		*val = sprd_adc_processed_vol(padc, chan->address, scale, 1);
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_AVERAGE_RAW:
		mutex_lock(&indio_dev->mlock);
		*val = sprd_adc_raw_value(padc, chan->address, scale, 1);
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;

	}
	return -EINVAL;
}

static ssize_t sprd_adc_read_info(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sprd_pmic_adc *padc = iio_priv(indio_dev);
	int adc_value, adc_vol;

	adc_value = sprd_adc_raw_value(padc, padc->channel, padc->scale,
				       padc->sample);
	adc_vol = sprd_adc_processed_vol(padc, padc->channel, padc->scale,
					 padc->sample);

	return sprintf(buf,
		       "channel = %d,scale = %d,adc_data = %d,adc_vol =%d,cal_type =%d\n",
		       padc->channel, padc->scale, adc_value, adc_vol,
		       cal_type);
}

static ssize_t sprd_adc_read_ratio(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sprd_pmic_adc *padc = iio_priv(indio_dev);
	uint32_t div_num, div_den;
	int voltage_ratio;

	sprd_adc_vol_ratio(padc, padc->channel, padc->scale, &div_num,
			   &div_den);

	voltage_ratio = (div_num << RATIO_PARA_OFFSET) |
		(div_den & RATIO_PARA_MASK);
	return sprintf(buf, "%d\n", voltage_ratio);
}

static ssize_t sprd_adc_set_channel(struct device *dev,
				    struct device_attribute *dev_attr,
				    const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sprd_pmic_adc *padc = iio_priv(indio_dev);
	int temp_channel, err;

	err = kstrtouint(buf, 10, &temp_channel);
	if (err)
		return err;

	mutex_lock(&padc->lock);
	padc->channel = temp_channel;
	mutex_unlock(&padc->lock);
	return count;
}

static ssize_t sprd_adc_set_scale(struct device *dev,
				  struct device_attribute *dev_attr,
				  const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sprd_pmic_adc *padc = iio_priv(indio_dev);
	int temp_scale, err;

	err = kstrtouint(buf, 10, &temp_scale);
	if (err)
		return err;

	mutex_lock(&padc->lock);
	padc->scale = temp_scale;
	mutex_unlock(&padc->lock);
	return count;
}

static IIO_DEVICE_ATTR(adc_info,
		       S_IRUGO | S_IWUSR, sprd_adc_read_info, NULL, 0);

static IIO_DEVICE_ATTR(adc_channel_set,
		       S_IRUGO | S_IWUSR, NULL, sprd_adc_set_channel, 0);

static IIO_DEVICE_ATTR(adc_scale_set,
		       S_IRUGO | S_IWUSR, NULL, sprd_adc_set_scale, 0);

static IIO_DEVICE_ATTR(adc_vol_ratio,
		       S_IRUGO | S_IWUSR, sprd_adc_read_ratio, NULL, 0);

static struct attribute *sprdadc_attributes[] = {
	&iio_dev_attr_adc_info.dev_attr.attr,
	&iio_dev_attr_adc_channel_set.dev_attr.attr,
	&iio_dev_attr_adc_scale_set.dev_attr.attr,
	&iio_dev_attr_adc_vol_ratio.dev_attr.attr,
	NULL,
};

static const struct attribute_group sprdadc_attribute_group = {
	.attrs = sprdadc_attributes,
};

static const struct iio_info sprd_adc_iio_info = {
	.read_raw = &sprd_adc_read_raw,
	.attrs = &sprdadc_attribute_group,
	.driver_module = THIS_MODULE,
};

static void sprd_adc_scale_init(struct sprd_pmic_adc *padc)
{
	int i;

	for (i = 0; i < ADC_CHANNEL_MAX; i++)
		padc->channel_scale[i] = 0;

	padc->channel_scale[5] = 3;
	padc->channel_scale[7] = 2;
	padc->channel_scale[9] = 2;
	padc->channel_scale[13] = 1;
	padc->channel_scale[19] = 3;
	padc->channel_scale[30] = 3;
	padc->channel_scale[31] = 3;
}

static void sprd_adc_scale_parse_dt(struct platform_device *pdev,
				    struct sprd_pmic_adc *padc)
{
	int i, size, chn_cnt;
	const __be32 *list;

	sprd_adc_scale_init(padc);
	list = of_get_property(pdev->dev.of_node, "sprd,channel-scale", &size);
	if (!size || !list) {
		dev_warn(&pdev->dev, "no adc channel-scale property\n");
		return;
	}

	chn_cnt = size / 8;
	if (chn_cnt >= ADC_CHANNEL_MAX)
		return;

	for (i = 0; i < chn_cnt; i++) {
		u32 chan_num = be32_to_cpu(*list++);
		u32 chan_scale = be32_to_cpu(*list++);

		padc->channel_scale[chan_num] = chan_scale;
		dev_info(&pdev->dev, "channel[%d] scale:%d\n", i,
			 padc->channel_scale[i]);
	}
}

static const struct of_device_id pmic_adc_of_match[] = {
	{.compatible = "sprd,sc2720-adc", .data = (void *)SC2720_ADC,},
	{}
};

static int sprd_pmic_adc_probe(struct platform_device *pdev)
{
	struct sprd_pmic_adc *sprd_adc_data;
	struct device_node *np = pdev->dev.of_node;
	struct iio_dev *indio_dev;
	const struct of_device_id *of_id;
	enum sprd_pmic_adc_type adc_type;
	int ret;
	u32 value;

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

	indio_dev = devm_iio_device_alloc(&pdev->dev,
					  sizeof(struct sprd_pmic_adc));
	if (!indio_dev) {
		dev_err(&pdev->dev, "failed allocating iio device\n");
		return -ENOMEM;
	}

	sprd_adc_data = iio_priv(indio_dev);
	ret = of_property_read_u32(np, "reg", &value);
	if (ret) {
		dev_err(&pdev->dev, "sprd get pmic adc reg failed!\n");
		return ret;
	}

	sprd_adc_data->hw_lock = of_hwspin_lock_request(pdev->dev.of_node,
							"pmic_adc");
	if (!sprd_adc_data->hw_lock) {
		dev_err(&pdev->dev, "pmic_adc not get hardware spinlock.\n");
		return -ENXIO;
	}

	sprd_adc_data->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!sprd_adc_data->regmap) {
		dev_err(&pdev->dev,
			"%s :fail regmap property for pmic adc .", __func__);
		return -ENODEV;
	}

	sprd_adc_scale_parse_dt(pdev, sprd_adc_data);
	sprd_adc_data->adc_base = value;
	sprd_adc_data->channel = ADC_CHANNEL_VBATSENSE;
	sprd_adc_data->sample = ADC_MESURE_NUMBER;
	sprd_adc_data->cal_type = SPRD_AUXADC_CAL_NO;
	sprd_adc_data->type = adc_type;
	sprd_adc_data->dev = &pdev->dev;
	sprd_adc_data->info = &sprd_chip_info_tbl[adc_type];
	mutex_init(&sprd_adc_data->lock);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &sprd_adc_iio_info;
	indio_dev->channels = sprd_adc_data->info->channels;
	indio_dev->num_channels = sprd_adc_data->info->num_channels;
	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "could not register iio (adc)");
		return ret;
	}

	platform_set_drvdata(pdev, indio_dev);
	sprd_adc_init(sprd_adc_data);
	dev_info(&pdev->dev, "sprd adc probe end\n");
	return 0;
}

static struct platform_driver sprd_adc_driver = {
	.probe = sprd_pmic_adc_probe,
	.driver = {
		.name = "sprd-adc",
		.of_match_table = pmic_adc_of_match,
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
MODULE_DESCRIPTION("Spreadtrum sc2720-adc Driver");
MODULE_LICENSE("GPL");
