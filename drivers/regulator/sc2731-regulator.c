/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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
 *
 */
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sort.h>
#include <linux/version.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>

#define REGULATOR_ROOT_DIR	"sprd-regulator"
#define REGULAOR_DT_NODE	"regulators"
#define PMIC_2731CA		0xC000

static inline int regu_adi_read(unsigned long reg);
static inline int regu_adi_write(unsigned long reg, unsigned long or_val,
				 unsigned long clear_msk);
#ifdef REGU_DEBUG
#define debug(format, arg...) pr_info("regu: ""@@@%s: " format, __func__, ##arg)
#else
#define debug(format, arg...)
#endif

#ifndef	ANA_REG_OR
#define	ANA_REG_OR(_r, _b)	regu_adi_write(_r, _b, _b)
#endif

#ifndef	ANA_REG_BIC
#define	ANA_REG_BIC(_r, _b)	regu_adi_write(_r, 0, _b)
#endif

#ifndef	ANA_REG_GET
#define	ANA_REG_GET(_r)		regu_adi_read(_r)
#endif

#ifndef	ANA_REG_SET
#define	ANA_REG_SET(_r, _v, _m)	regu_adi_write((_r), ((_v) & (_m)), (_m))
#endif

/*
 * hw auto voltage scaling regs
 */
#define STEP_DELAY(x)			(((x) & 0x3) << 12)
#define STEP_NUM(x)			(((x) & 0xF) << 8)
#define STEP_VOL(x)			(((x) & 0x1F) << 3)
#define VOL_TUNE_START			BIT(2)
#define VOL_TUNE_FLAG			BIT(1)
#define VOL_TUNE_EN			BIT(0)

struct {
	char regu_name[15];
	int index;
	unsigned long pd_reg;
	u32 pd_mask;
	unsigned long vol_reg;
	u32 vol_mask;
	unsigned long adc_reg;
	u32 adc_mask;
	u32 adc_chan;
	u32 auto_reg;
} reg_info_map[] = {
/* regu_name
 * index pd_reg   pd_mask val_reg val_mask adc_reg adc_mask adc_chan
 */
{ "vddarm0",
	0, 0x0C28, 0x10, 0x0C54, 0x1ff, 0x0888, 0x4000, 0xd, 0xF9C},
{ "vddarm1",
	1, 0x0C28, 0x8, 0x0C64, 0x1ff, 0x0888, 0x2000, 0xd, 0xFA0},
{ "vddcore",
	2, 0x0C28, 0x40, 0x0C74, 0x1ff,  0x0888, 0x6000, 0xd, 0xF94},
{ "vddgpu",
	3, 0x0C88, 0x1, 0x0C84, 0x1ff,  0x0888, 0x8000, 0xd, 0xF98},
{ "vddmem",
	4, 0x0C28, 0x80, 0x0C98, 0x3ff,  0x0888, 0xa000, 0xd},
{ "vddgen",
	5, 0x0C28, 0x100, 0x0CA8, 0x3ff,  0x0888, 0xe000, 0xd},
{ "dcdcrf",
	6, 0x0C28, 0x800, 0x0CB8, 0x3ff,  0x0888, 0xc000, 0xd},
{ "vddwpa",
	7, 0x0CD4, 0x4000, 0x0CD8, 0x3ff,  0x0844, 0x4, 0x1},
{ "vddcama0",
	8, 0x0CFC, 0x1, 0x0D00, 0xff,  0x0844, 0x60, 0x1},
{ "vddcama1",
	9, 0x0D04, 0x1, 0x0D08, 0xff,  0x0844, 0x5, 0x1},
{ "vddcammot",
	10, 0x0D0C, 0x1, 0x0D10, 0xff,  0x0844, 0x6, 0x1},
{ "vddsim0",
	11, 0x0D64, 0x1, 0x0D18, 0xff,  0x0844, 0x80, 0x1},
{ "vddsim1",
	12, 0x0D68, 0x1, 0x0D20, 0xff,  0x0844, 0x100, 0x1},
{ "vddvldo",
	13, 0x0D6C, 0x1, 0x0D28, 0xff,  0x0844, 0xc0, 0x1},
{ "vddemmccore",
	14, 0x0D2C, 0x1, 0x0D30, 0xff,  0x0844, 0x1, 0x1},
{ "vddsdcore",
	15, 0x0D74, 0x1, 0x0D38, 0xff,  0x0844, 0x20, 0x1},
{ "vddsdio",
	16, 0x0D70, 0x1, 0x0D40, 0xff,  0x0844, 0xa0, 0x1},
{ "vdd28",
	17, 0x0C28, 0x2, 0x0D48, 0xff,  0x0844, 0x8, 0x1},
{ "vddwifipa",
	18, 0x0D4C, 0x1, 0x0D50, 0xff,  0x0844, 0x500, 0x1},
{ "vdddcxo",
	19, 0x0C28, 0x2000, 0x0D58, 0xff,  0x0844, 0x400, 0x1},
{ "vddusb33",
	20, 0x0D5C, 0x1, 0x0D60, 0xff,  0x0844, 0x300, 0x1},
{ "vddcamd0",
	21, 0x0D7C, 0x1, 0x0D80, 0x7f,  0x0844, 0x200, 0x1},
{ "vddcamd1",
	22, 0x0D84, 0x1, 0x0D88, 0x7f,  0x0844, 0x2, 0x1},
{ "vddcon",
	23, 0x0D8C, 0x1, 0x0D90, 0x7f,  0x0844, 0x3, 0x1},
{ "vddcamio",
	24, 0x0D94, 0x1, 0x0D98, 0x7f,  0x0844, 0x40, 0x1},
{  "vdd18",
	25, 0x0C28, 0x4, 0x0DA0, 0x7f,  0x0844, 0x10, 0x1},
{ "vddrf",
	26, 0x0DA4, 0x1, 0x0DA8, 0x7f,  0x08f8, 0x3000, 0x1},
{ "vddsram",
	27, 0x0C28, 0x1000, 0x0DB0, 0x7f,  0x08f8, 0x2000, 0x1},
{ "vddkpled",
	28, 0x0EC4, 0x100, 0x0EC4, 0xff,  0x08f8, 0x2000, 0x1},
{ "vddvibr",
	29, 0x0EC8, 0x100, 0x0EC8, 0xff,  0x08f8, 0x2000, 0x1},

{}
};

struct sprd_regulator_regs {
	/* BIT4: default on/off(0: off, 1: on);
	 * BIT0~BIT3: dcdc/ldo type(0: ldo; 2: dcdc)
	 */
	int typ;
	int hide_offset;
	unsigned long pd_set;
	u32 pd_set_bit;
	unsigned long vol_trm;
	u32 vol_trm_bits;
	unsigned long cal_ctl;
	u32 cal_ctl_bits, cal_chan;
	int otp_delta;
	u32 vol_def;
	unsigned long vol_ctl;
	u32 vol_ctl_bits;
	u32 vol_sel_cnt;
	u32 auto_reg;
	u32 *vol_sel;
};

struct sprd_regulator_data {
	struct regulator_dev *rdev;
};

struct sprd_regulator_desc {
	struct regulator_desc desc;
	struct regulator_init_data *init_data;
	struct sprd_regulator_regs regs;
	struct sprd_regulator_data data;
#if defined(CONFIG_DEBUG_FS)
	struct dentry *debugfs;
#endif
};

enum {
	VDD_TYP_LDO,
	VDD_TYP_LDO_D,
	VDD_TYP_DCDC,
	VDD_TYP_LPREF,
	VDD_TYP_BOOST,
};

static u32 ana_chip_id;
static u16 ana_mixed_ctl, otp_pwr_sel;

static struct regmap *pmic_regmap;
static struct sprd_regulator_desc *sprd_desc_list;

/* 0: dummy */
static atomic_t idx = ATOMIC_INIT(0);
static struct of_device_id sprd_regulator_of_match[] = {
	{.compatible = "sprd,sc2731-regulator",},
	{}
};

#define REGU_VERIFY_DLY	(1000)
static inline int regu_adi_read(unsigned long reg)
{
	int ret;
	unsigned int value;

	ret = regmap_read(pmic_regmap, reg, &value);
	return ret ? ret : value;
}

static inline int regu_adi_write(unsigned long reg, unsigned long or_val,
				 unsigned long clear_msk)
{
	return regmap_update_bits(pmic_regmap, reg, clear_msk, or_val);
}

static inline int adi_raw_write(unsigned long reg, unsigned long val)
{
	return regmap_write(pmic_regmap, reg, val);
}

static struct sprd_regulator_desc *get_desc(struct regulator_dev *rdev)
{
	return (struct sprd_regulator_desc *)rdev->desc;
}

static inline int __strcmp(const char *cs, const char *ct)
{
	if (!cs || !ct)
		return -1;

	return strcmp(cs, ct);
}

static inline unsigned int get_max_uV(struct regulator_dev *rdev)
{
	return (rdev->desc->n_voltages - 1) * (rdev->desc->uV_step)
		+ rdev->desc->min_uV;
}

static struct regulator_consumer_supply *set_supply_map(struct device *dev,
							const char *supply_name,
							int *num)
{
	char **map = (char **)dev_get_platdata(dev);
	int i, n;
	struct regulator_consumer_supply *consumer_supplies = NULL;

	if (!supply_name || !(map && map[0]))
		return NULL;

	for (i = 0; map[i] || map[i + 1]; i++) {
		if (map[i] && 0 == strcmp(map[i], supply_name))
			break;
	}

	/*
	 * i++;
	 * Do not skip supply name
	 */

	for (n = 0 ; map[i + n] ; n++)
		;

	if (n) {
		debug("supply %s consumers %d - %d\n", supply_name, i, n);
		consumer_supplies =
		devm_kzalloc(dev, n*sizeof(*consumer_supplies), GFP_KERNEL);
			   /* BUG_ON(!consumer_supplies); */
		for (n = 0; map[i]; i++, n++)
			consumer_supplies[n].supply = map[i];

		if (num)
			*num = n;
	}
	return consumer_supplies;
}

static int acquire_reg_info(struct sprd_regulator_desc *desc, int index)
{
	struct sprd_regulator_regs *regs = &desc->regs;
	int err = 0;

	if (__strcmp(desc->desc.name, reg_info_map[index].regu_name) == 0) {
		/* ldo/dcdc power down */
		regs->pd_set = reg_info_map[index].pd_reg;
		regs->pd_set_bit = reg_info_map[index].pd_mask;
		/* ldo/dcdc voltage trim */
		regs->vol_trm = reg_info_map[index].vol_reg;
		regs->vol_trm_bits = reg_info_map[index].vol_mask;
		/* adc channel info */
		regs->cal_ctl = reg_info_map[index].adc_reg;
		regs->cal_ctl_bits = reg_info_map[index].adc_mask;
		regs->cal_chan = reg_info_map[index].adc_chan;
		regs->auto_reg = reg_info_map[index].auto_reg;
		err = 1;
	} else {
		debug("regulator %s does not match with %s !!!\n",
		      desc->desc.name, reg_info_map[index].regu_name);
	}
	return err;

}

static int ldo_turn_on(struct regulator_dev *rdev)
{
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;

	if (regs->pd_set)
		ANA_REG_BIC(regs->pd_set, regs->pd_set_bit);

	debug("regu 0x%p (%s), turn on\n", regs, desc->desc.name);
	return 0;
}

static int ldo_turn_off(struct regulator_dev *rdev)
{
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;

	if (regs->pd_set)
		ANA_REG_OR(regs->pd_set, regs->pd_set_bit);

	debug("regu 0x%p (%s), turn off\n", regs, desc->desc.name);
	return 0;
}

static int ldo_is_on(struct regulator_dev *rdev)
{
	int ret = -EINVAL;
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;

	if (regs->pd_set)
		ret = !(ANA_REG_GET(regs->pd_set) & regs->pd_set_bit);

	debug("regu 0x%p (%s) turn on, return %d\n", regs, desc->desc.name,
	      ret);
	return ret;
}

static int ldo_set_voltage(struct regulator_dev *rdev, int min_uV,
			   int max_uV, unsigned *selector)
{
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;
	/* int mv = min_uV / 1000; */
	int ret = -EINVAL;

	debug("regu 0x%p (%s) set voltage, %d(uV) %d(uV)\n", regs,
	      desc->desc.name, min_uV, max_uV);

	if (min_uV < desc->desc.min_uV) {
		debug
		    ("warning:(%s) lower than min_uV,set to min_uV %d(uV)\n",
		     desc->desc.name, desc->desc.min_uV);
		min_uV = desc->desc.min_uV;
	}

	if (min_uV > get_max_uV(rdev)) {
		debug
		    ("warning:(%s) higher than max_uV,set to max_uV %d(uV)\n",
		     desc->desc.name, get_max_uV(rdev));
		min_uV = get_max_uV(rdev);
	}

	if (regs->vol_trm) {
		int shft = __ffs(regs->vol_trm_bits);
		u32 trim =
		    DIV_ROUND_UP((int)(min_uV - desc->desc.min_uV),
				 desc->desc.uV_step);

		ret = trim > (regs->vol_trm_bits >> shft);
		WARN_ONCE(ret != 0,
		     "warning: regulator (%s) not support %d(uV)\n",
		     desc->desc.name, min_uV);

		if (ret == 0) {
			ANA_REG_SET(regs->vol_trm,
				    trim << shft, regs->vol_trm_bits);
		}
	}

	return ret;
}

static int ldo_get_voltage(struct regulator_dev *rdev)
{
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;
	u32 vol;

	if (regs->vol_trm) {
		int shft = __ffs(regs->vol_trm_bits);
		u32 trim =
		    (ANA_REG_GET(regs->vol_trm) & regs->vol_trm_bits) >> shft;
		vol = desc->desc.min_uV + trim * desc->desc.uV_step;
		return vol;
	}

	return -EFAULT;
}

static int get_trimming_step(struct regulator_dev *rdev, int to_vol)
{
	struct sprd_regulator_desc *desc = get_desc(rdev);

	WARN_ONCE(!desc->desc.uV_step, "not found uV_step for %s\n",
	     desc->desc.name);

	return desc->desc.uV_step;
}

static int match_dcdc_vol(struct sprd_regulator_regs *regs, u32 vol)
{
	int i, j = -1;
	int ds, min_ds = 100 * 1000;
	/* uV, the max range of small voltage */

	for (i = 0; i < regs->vol_sel_cnt; i++) {
		ds = vol - regs->vol_sel[i];
		if (ds >= 0 && ds < min_ds) {
			min_ds = ds;
			j = i;
		}
	}
	return j;
}

static int dcdc_enable_time(struct regulator_dev *rdev, int old_vol)
{
#ifdef LINEAR_DELAY
	int vol = rdev->desc->ops->get_voltage(rdev);

	if (vol > old_vol) {
		/* FIXME: for dcdc, each step (50mV) takes 10us */
		int dly = (vol - old_vol) * 10 / (50 * 1000);

		WARN_ON_ONCE(dly > 1000);
		udelay(dly);
	}
#else
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;

	if (4 == (regs->typ & (BIT(4) - 1)))
		return 0;

	/*
	 * FIXME: for 2731 dcdc, each command need 3*32k cycle to take effect on
	 * analog port.
	 */
	udelay(123);
#endif
	return 0;
}

static int dcdc_set_voltage(struct regulator_dev *rdev, int min_uV,
			    int max_uV, unsigned *selector)
{
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;
	int i = 0;
	/* int mV = min_uV / 1000; */
	int old_vol = rdev->desc->ops->get_voltage(rdev);

	debug("regu 0x%p (%s) %d %d\n", regs, desc->desc.name, min_uV,
	      get_max_uV(rdev));

	if (regs->vol_ctl) {
		/* found the closely vol ctrl bits */
		i = match_dcdc_vol(regs, min_uV);
		if (i < 0)
			return WARN_ONCE(-EINVAL,
				    "not found %s closely ctrl bits for %d(uV)\n",
				    desc->desc.name, min_uV);
	}
	/* dcdc calibration control bits (default 00000),
	 * small adjust voltage: 100/32mv ~= 3.125mv
	 */
	{
		int shft_trm = __ffs(regs->vol_trm_bits);
		int shft_ctl = 0;
		int step = 0;
		int j = 0;

		if (regs->vol_ctl) {
			shft_ctl = __ffs(regs->vol_ctl_bits);
			step = get_trimming_step(rdev, 0);

			j = DIV_ROUND_UP((int)(min_uV - (int)regs->vol_sel[i]),
					 step);

			debug("regu 0x%p (%s) %d = %d %+duV(trim %#x)\n", regs,
			      desc->desc.name, min_uV, regs->vol_sel[i],
			      min_uV - regs->vol_sel[i], j);
		} else {
			j = DIV_ROUND_UP((int)(min_uV - desc->desc.min_uV),
					 desc->desc.uV_step);

			debug("regu 0x%p (%s) %d = %d %+duV(trim %#x)\n", regs,
			      desc->desc.name, min_uV, desc->desc.min_uV,
			      min_uV - desc->desc.min_uV, j);
		}

		WARN_ON_ONCE(j > (regs->vol_trm_bits >> shft_trm));

		if (regs->vol_trm == regs->vol_ctl) {
			ANA_REG_SET(regs->vol_ctl,
				    (j << shft_trm) | (i << shft_ctl),
				    regs->vol_trm_bits | regs->vol_ctl_bits);
		} else {
			if (regs->vol_trm) {
				/* small adjust first */
				ANA_REG_SET(regs->vol_trm, j << shft_trm,
					    regs->vol_trm_bits);
			}

			if (regs->vol_ctl) {
				ANA_REG_SET(regs->vol_ctl, i << shft_ctl,
					    regs->vol_ctl_bits);
			}
		}
	}

	dcdc_enable_time(rdev, old_vol);

	return 0;
}

static int dcdc_set_voltage_step(struct regulator_dev *rdev, int min_uV,
				 int max_uV, unsigned *selector)
{
	int to_vol;
	int step;
	int vol = rdev->desc->ops->get_voltage(rdev);
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;

	to_vol = min_uV;
	/* uV */
	step = 25 * 1000;

	to_vol -= regs->hide_offset * 1000;

	if (to_vol < desc->desc.min_uV) {
		debug
		    ("warning:(%s) lower than min_uV,set to min_uV %d(uV)\n",
		     desc->desc.name, desc->desc.min_uV);
		to_vol = desc->desc.min_uV;
	}

	if (to_vol > get_max_uV(rdev)) {
		debug
		    ("warning:(%s) higher than max_uV,set to max_uV %d(uV)\n",
		     desc->desc.name, get_max_uV(rdev));
		to_vol = get_max_uV(rdev);
	}

	if (vol < to_vol) {
		do {
			vol += step;
			if (vol > to_vol)
				vol = to_vol;
			dcdc_set_voltage(rdev, vol, vol, selector);
		} while (vol < to_vol);
	} else {
		do {
			vol -= step;
			if (vol < to_vol)
				vol = to_vol;
			dcdc_set_voltage(rdev, vol, vol, selector);
		} while (vol > to_vol);
	}
	return 0;
}

static int trigger_hw_voltage_scaling(struct regulator_dev *rdev, int vol,
				      unsigned *selector)
{
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;
	/*
	 *set vol turn enable
	 */
	ANA_REG_OR(regs->auto_reg, VOL_TUNE_EN);
	dcdc_set_voltage(rdev, vol, vol, selector);
	ANA_REG_OR(regs->auto_reg, VOL_TUNE_START);
	/*
	 *  delay 32K *3 cycle
	 */
	udelay(100);

	return 0;
}

static int dcdc_set_voltage_auto_step(struct regulator_dev *rdev, int min_uV,
				 int max_uV, unsigned *selector)
{
	int to_vol;
	int step;
	int vol = rdev->desc->ops->get_voltage(rdev);
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;

	to_vol = min_uV;
	/* uV */
	step = 100 * 1000;

	to_vol -= regs->hide_offset * 1000;

	if (to_vol < desc->desc.min_uV) {
		debug
		    ("warning:(%s) lower than min_uV,set to min_uV %d(uV)\n",
		     desc->desc.name, desc->desc.min_uV);
		to_vol = desc->desc.min_uV;
	}

	if (to_vol > get_max_uV(rdev)) {
		debug
		    ("warning:(%s) higher than max_uV,set to max_uV %d(uV)\n",
		     desc->desc.name, get_max_uV(rdev));
		to_vol = get_max_uV(rdev);
	}


	if (vol < to_vol) {
		do {
			vol += step;
			if (vol > to_vol)
				vol = to_vol;

			trigger_hw_voltage_scaling(rdev, vol, selector);

		} while (vol < to_vol);
	} else {
		do {
			vol -= step;
			if (vol < to_vol)
				vol = to_vol;

			trigger_hw_voltage_scaling(rdev, vol, selector);

		} while (vol > to_vol);
	}

	return 0;
}

static int dcdc_get_voltage(struct regulator_dev *rdev)
{
	struct sprd_regulator_desc *desc = get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;
	u32 uV = 0;
	int i, cal = 0;

	if (regs->vol_ctl) {
		int shft_ctl = __ffs(regs->vol_ctl_bits);
		int shft_trm = __ffs(regs->vol_trm_bits);

		i = (ANA_REG_GET(regs->vol_ctl) & regs->vol_ctl_bits) >>
		    shft_ctl;
		uV = regs->vol_sel[i];

		if (regs->vol_trm) {
			cal =
			    (ANA_REG_GET(regs->vol_trm) & regs->vol_trm_bits) >>
			    shft_trm;
			cal *= get_trimming_step(rdev, 0);
		}

	} else if (regs->vol_trm) {
		int shft_trm = __ffs(regs->vol_trm_bits);
		u32 trim =
		    (ANA_REG_GET(regs->vol_trm) & regs->vol_trm_bits) >>
		    shft_trm;
		uV = desc->desc.min_uV + trim * desc->desc.uV_step;
	}

	uV += regs->hide_offset * 1000;

	return (uV + cal);
}

static struct regulator_ops ldo_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.enable = ldo_turn_on,
	.disable = ldo_turn_off,
	.is_enabled = ldo_is_on,
	.set_voltage = ldo_set_voltage,
	.get_voltage = ldo_get_voltage,
};

static struct regulator_ops dcdc_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.enable = ldo_turn_on,
	.disable = ldo_turn_off,
	.is_enabled = ldo_is_on,
	.set_voltage = dcdc_set_voltage_step,
	.get_voltage = dcdc_get_voltage,
};

static struct regulator_ops dcdc_auto_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.enable = ldo_turn_on,
	.disable = ldo_turn_off,
	.is_enabled = ldo_is_on,
	.set_voltage = dcdc_set_voltage_auto_step,
	.get_voltage = dcdc_get_voltage,
};

#if defined(CONFIG_DEBUG_FS)
static struct dentry *debugfs_root;

static int debugfs_enable_get(void *data, u64 *val)
{
	struct regulator_dev *rdev = data;

	if (rdev && rdev->desc->ops->is_enabled)
		*val = rdev->desc->ops->is_enabled(rdev);
	else
		*val = -1;
	return 0;
}

static int debugfs_enable_set(void *data, u64 val)
{
	struct regulator_dev *rdev = data;

	if (rdev && rdev->desc->ops->enable)
		(val) ? rdev->desc->ops->enable(rdev)
		    : rdev->desc->ops->disable(rdev);
	return 0;
}

static int debugfs_voltage_get(void *data, u64 *val)
{
	struct regulator_dev *rdev = data;

	if (rdev)
		*val = rdev->desc->ops->get_voltage(rdev) / 1000;
	else
		*val = -1;
	return 0;
}

static int debugfs_voltage_set(void *data, u64 val)
{
	struct regulator_dev *rdev = data;
	u32 min_uV;

	if (rdev && rdev->desc->ops->set_voltage) {
		min_uV = (u32)val*1000;
		rdev->desc->ops->set_voltage(rdev, min_uV, min_uV, 0);
	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_enable,
			debugfs_enable_get, debugfs_enable_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_ldo,
			debugfs_voltage_get, debugfs_voltage_set, "%llu\n");

static void rdev_init_debugfs(struct regulator_dev *rdev)
{
	struct sprd_regulator_desc *desc = get_desc(rdev);

	desc->debugfs = debugfs_create_dir(rdev->desc->name, debugfs_root);

	if (IS_ERR_OR_NULL(rdev->debugfs)) {
		pr_warn("Failed to create debugfs directory\n");
		rdev->debugfs = NULL;
		return;
	}

	debugfs_create_file("enable", S_IRUGO | S_IWUSR,
			    desc->debugfs, rdev, &fops_enable);
	debugfs_create_file("voltage", S_IRUGO | S_IWUSR,
			    desc->debugfs, rdev, &fops_ldo);
}
#else
static void rdev_init_debugfs(struct regulator_dev *rdev)
{
}
#endif

static int regulator_parse_dt(struct platform_device *pdev,
				  struct device_node *np,
				  struct sprd_regulator_desc *desc,
				  struct regulator_consumer_supply *supply,
				  int sz)
{
	struct sprd_regulator_regs *regs = &desc->regs;
	int index = -1;
	u32 tmp_val_u32;
	int ret = 0;

	if (!pdev || !np || !desc || !supply)
		return -EINVAL;

	desc->init_data = of_get_regulator_init_data(&pdev->dev, np,
						     &desc->desc);

	if (!desc->init_data) {
		dev_err(&pdev->dev,
			"failed to parse regulator(%s) init data!\n",
			np->name);
		return -EINVAL;
	}

	desc->desc.name = desc->init_data->constraints.name;
	desc->desc.id = (atomic_inc_return(&idx));
	desc->desc.type = REGULATOR_VOLTAGE;
	desc->desc.owner = THIS_MODULE;
	desc->desc.continuous_voltage_range = true;

	supply[0].dev_name = NULL;
	supply[0].supply = desc->init_data->constraints.name;
	desc->init_data->supply_regulator = 0;
	desc->init_data->constraints.valid_modes_mask =
	    REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY;
	desc->init_data->constraints.valid_ops_mask =
	    REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS |
	    REGULATOR_CHANGE_VOLTAGE;
	desc->init_data->num_consumer_supplies = sz;

	desc->init_data->consumer_supplies =
	    set_supply_map(&pdev->dev, desc->desc.name,
			   &desc->init_data->num_consumer_supplies);

	if (!desc->init_data->consumer_supplies)
		desc->init_data->consumer_supplies = supply;

	if (!__strcmp("dcdc", np->name))
		regs->typ = 2;
	else
		regs->typ = 0;
	if (ANA_REG_GET(ANA_REG_GLB_CHIP_ID_LOW) >= PMIC_2731CA) {
		if (of_property_read_bool(np, "sprd,auto_step"))
			regs->typ = 4;
	}

	if (of_property_read_bool(np, "sprd,default-on"))
		regs->typ |= BIT(4);

	desc->desc.min_uV = desc->init_data->constraints.min_uV;

	ret = of_property_read_u32(np, "reg", &tmp_val_u32);
	if (!ret)
		index = tmp_val_u32;

	if (-1 != index) {
		if (!acquire_reg_info(desc, index))
			debug("acquire reg info:%d fail!!!\n", index);
	} else {
		debug("get index:%d fail!!!\n", index);
	}

	ret = of_property_read_u32(np, "sprd,step-microvolt", &tmp_val_u32);
	if (!ret)
		desc->desc.uV_step = tmp_val_u32;

	ret = of_property_read_u32(np, "sprd,default-microvolt", &tmp_val_u32);
	if (!ret)
		regs->vol_def = tmp_val_u32;

	/* base value is 1000 */
	ret = of_property_read_u32(np, "sprd,hide-offset", &tmp_val_u32);
	if (!ret)
		regs->hide_offset = ((int)tmp_val_u32 - 1000);

	desc->desc.n_voltages =
	    (desc->init_data->constraints.max_uV - (desc->desc.min_uV))
	    / (desc->desc.uV_step) + 1;

	return 0;
}

static int regulator_register_dt(struct platform_device *pdev)
{
	struct sprd_regulator_desc *sprd_desc = NULL;

	struct regulator_dev *rdev;
	struct regulator_ops *regs_ops[] = {
		&ldo_ops, NULL, &dcdc_ops, NULL, &dcdc_auto_ops,
		NULL,
	};
	struct regulator_consumer_supply consumer_supplies_default[1];
	struct regulator_config config;
	struct device_node *dev_np;
	struct device_node *child_np;
	int regu_cnt = 0, ret = 0;

	dev_np = of_find_node_by_name(pdev->dev.of_node, REGULAOR_DT_NODE);
	if (!dev_np)
		return -ENODEV;

	regu_cnt = of_get_child_count(dev_np);
	if (!regu_cnt)
		return -EINVAL;

	sprd_desc_list =
	    devm_kzalloc(&pdev->dev,
			 regu_cnt * sizeof(struct sprd_regulator_desc),
			 GFP_KERNEL);
	if (!sprd_desc_list) {
		dev_err(&pdev->dev,
			"failed allocate memory for sprd_regulator_desc list\n");
		return -ENOMEM;
	}
	sprd_desc = sprd_desc_list;

	for_each_available_child_of_node(dev_np, child_np) {
		ret =
		    regulator_parse_dt(pdev, child_np, sprd_desc,
					   consumer_supplies_default,
					   ARRAY_SIZE
					   (consumer_supplies_default));
		if (ret) {
			dev_err(&pdev->dev,
				"failed to parse regulator(%s) dts\n",
				child_np->name);
			continue;
		}

		BUG_ON((sprd_desc->regs.typ & (BIT(4) - 1)) >=
		       ARRAY_SIZE(regs_ops));

		if (!sprd_desc->desc.ops)
			sprd_desc->desc.ops =
			    regs_ops[sprd_desc->regs.typ & (BIT(4) - 1)];

		config.dev = &pdev->dev;
		config.init_data = sprd_desc->init_data;
		config.driver_data = NULL;
		config.of_node = child_np;
		config.regmap = NULL;
		config.ena_gpio = -EINVAL;
		config.ena_gpio_initialized = false;


		rdev = regulator_register(&sprd_desc->desc, &config);

		debug("regulator_desc 0x%p, rdev 0x%p\n", &sprd_desc->desc,
		      rdev);

		if (!IS_ERR_OR_NULL(rdev)) {
			rdev->reg_data = rdev;
			sprd_desc->data.rdev = rdev;
			if (rdev->desc->ops->is_enabled(rdev))
				rdev->use_count = 1;
			rdev_init_debugfs(rdev);
		}

		sprd_desc++;
	}

	return 0;
}

static int sprd_regulator_probe(struct platform_device *pdev)
{

	pmic_regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pmic_regmap) {
		dev_err(&pdev->dev, "NULL spi parent property for regulator.");
		return PTR_ERR(pmic_regmap);
	}

#if defined(CONFIG_DEBUG_FS)
	debugfs_root = debugfs_create_dir(REGULATOR_ROOT_DIR, NULL);
	if (IS_ERR_OR_NULL(debugfs_root)) {
		WARN_ONCE(!debugfs_root,
		     "%s: Failed to create debugfs directory\n",
		     REGULATOR_ROOT_DIR);
		debugfs_root = NULL;
	}

	/* vddarm/vddcore/vddmem common debugfs interface */
	{
		char str[NAME_MAX];
		struct dentry *vol_root = debugfs_create_dir("vol", NULL);

		sprintf(str, "../%s/vddarm/voltage", REGULATOR_ROOT_DIR);
		debugfs_create_symlink("dcdcarm", vol_root, str);
		sprintf(str, "../%s/vddcore/voltage", REGULATOR_ROOT_DIR);
		debugfs_create_symlink("dcdccore", vol_root, str);
		sprintf(str, "../%s/vddmem/voltage", REGULATOR_ROOT_DIR);
		debugfs_create_symlink("dcdcmem", vol_root, str);
	}
#endif
	ana_chip_id = ((u32) ANA_REG_GET(ANA_REG_GLB_CHIP_ID_HIGH) << 16) |
	    ((u32) ANA_REG_GET(ANA_REG_GLB_CHIP_ID_LOW) & 0xFFFF);
	ana_mixed_ctl = ANA_REG_GET(ANA_REG_GLB_MIXED_CTRL0);
	otp_pwr_sel = ANA_REG_GET(ANA_REG_GLB_DCDC_VLG_SEL);
	pr_info("sc2731 ana chipid:(0x%08x), ana_mixed_ctl:(0x%08x),otp_sel:(0x%08x)\n",
		ana_chip_id, ana_mixed_ctl, otp_pwr_sel);

	adi_raw_write(ANA_REG_GLB_PWR_WR_PROT_VALUE,
		      BITS_PWR_WR_PROT_VALUE(0x6e7f));

	regulator_register_dt(pdev);

	return 0;
}

static int sprd_regulator_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver sprd_regulator_driver = {
	.driver = {
		   .name = "sc2731-regulator",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(sprd_regulator_of_match),
		   },
	.probe = sprd_regulator_probe,
	.remove = sprd_regulator_remove
};

static int __init regu_driver_init(void)
{
	return platform_driver_register(&sprd_regulator_driver);
}

subsys_initcall(regu_driver_init);

MODULE_DESCRIPTION("Spreadtrum sc2731 regulator driver");
MODULE_AUTHOR("Chen Junhui <erick.chen@spreadtrum.com>");
MODULE_LICENSE("GPL");
