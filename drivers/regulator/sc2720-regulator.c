/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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
#include <linux/idr.h>
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

#define WR_UNLOCK 0x6e7f
#define REGULATOR_SCALING_STEP (3.125 * 1000)
#define DESC_TYPE_MASK GENMASK(3, 0)
#define MIN_HW_STEP	3125
#define MAX_STEP_NUM	15

static struct {
	char regu_name[15];
	int index;
	unsigned long pd_reg;
	u32 pd_mask;
	unsigned long vol_reg;
	u32 vol_mask;
	unsigned long adc_reg;
} reg_info_map[] = {
/* regu_name, index,
 *	pd_reg, pd_mask
 *	val_reg, val_mask
 */
{ "vddcore", 0,
	ANA_REG_GLB_POWER_PD_SW, BIT_DCDC_CORE_PD,
	ANA_REG_GLB_DCDC_CORE_VOL, BITS_VOSEL_VCORE(~0U)},
{ "vddgen", 1,
	ANA_REG_GLB_POWER_PD_SW, BIT_DCDC_GEN_PD,
	ANA_REG_GLB_DCDC_GEN_VOL, BITS_VOSEL_VGEN(~0U)},
{ "vddwpa", 2,
	ANA_REG_GLB_DCDC_WPA_REG2, BIT_PD_BUCK_VPA,
	ANA_REG_GLB_DCDC_WPA_VOL, BITS_VOSEL_VPA(~0U)},
{ "avdd18", 3,
	ANA_REG_GLB_POWER_PD_SW, BIT_LDO_AVDD18_PD,
	ANA_REG_GLB_LDO_AVDD18_REG1, BITS_LDO_AVDD18_V(~0U)},
{ "vddcamio", 4,
	ANA_REG_GLB_LDO_CAMIO_REG0, BIT_LDO_CAMIO_PD,
	ANA_REG_GLB_LDO_CAMIO_REG1, BITS_LDO_CAMIO_V(~0U)},
{ "vddrf18a", 5,
	ANA_REG_GLB_LDO_RF18A_REG0, BIT_LDO_RF18A_PD,
	ANA_REG_GLB_LDO_RF18A_REG1, BITS_LDO_RF18A_V(~0U)},
{ "vddrf18b", 6,
	ANA_REG_GLB_LDO_RF18B_REG0, BIT_LDO_RF18B_PD,
	ANA_REG_GLB_LDO_RF18B_REG1, BITS_LDO_RF18B_V(~0U)},
{ "vddcamd", 7,
	ANA_REG_GLB_LDO_CAMD_REG0, BIT_LDO_CAMD_PD,
	ANA_REG_GLB_LDO_CAMD_REG1, BITS_LDO_CAMD_V(~0U)},
{ "vddcon", 8,
	ANA_REG_GLB_LDO_CON_REG0, BIT_LDO_CON_PD,
	ANA_REG_GLB_LDO_CON_REG1, BITS_LDO_CON_V(~0U)},
{ "vddmem", 9,
	ANA_REG_GLB_POWER_PD_SW, BIT_LDO_MEM_PD,
	ANA_REG_GLB_LDO_MEM_REG1, BITS_LDO_MEM_V(~0U)},
{ "vddsim0", 10,
	ANA_REG_GLB_LDO_SIM0_PD_REG, BIT_LDO_SIM0_PD,
	ANA_REG_GLB_LDO_SIM0_REG1, BITS_LDO_SIM0_V(~0U)},
{ "vddsim1", 11,
	ANA_REG_GLB_LDO_SIM1_PD_REG, BIT_LDO_SIM1_PD,
	ANA_REG_GLB_LDO_SIM1_REG1, BITS_LDO_SIM1_V(~0U)},
{ "vddsim2", 12,
	ANA_REG_GLB_LDO_SIM2_PD_REG, BIT_LDO_SIM2_PD,
	ANA_REG_GLB_LDO_SIM2_REG1, BITS_LDO_SIM2_V(~0U)},
{ "vddcama", 13,
	ANA_REG_GLB_LDO_CAMA_REG0, BIT_LDO_CAMA_PD,
	ANA_REG_GLB_LDO_CAMA_REG1, BITS_LDO_CAMA_V(~0U)},
{ "vddcammot", 14,
	ANA_REG_GLB_LDO_CAMMOT_REG0, BIT_LDO_CAMMOT_PD,
	ANA_REG_GLB_LDO_CAMMOT_REG1, BITS_LDO_CAMMOT_V(~0U)},
{ "vddemmccore", 15,
	ANA_REG_GLB_LDO_EMMCCORE_PD_REG, BIT_LDO_EMMCCORE_PD,
	ANA_REG_GLB_LDO_EMMCCORE_REG1, BITS_LDO_EMMCCORE_V(~0U)},
{ "vddsdcore", 16,
	ANA_REG_GLB_LDO_SD_PD_REG, BIT_LDO_SDCORE_PD,
	ANA_REG_GLB_LDO_SD_REG1, BITS_LDO_SDCORE_V(~0U)},
{ "vddsdio", 17,
	ANA_REG_GLB_LDO_SDIO_PD_REG, BIT_LDO_SDIO_PD,
	ANA_REG_GLB_LDO_SDIO_REG1, BITS_LDO_SDIO_V(~0U)},
{ "vdd28", 18,
	ANA_REG_GLB_POWER_PD_SW, BIT_LDO_VDD28_PD,
	ANA_REG_GLB_LDO_VDD28_REG1, BITS_LDO_VDD28_V(~0U)},
{ "vddwifipa", 19,
	ANA_REG_GLB_LDO_WIFIPA_REG0, BIT_LDO_WIFIPA_PD,
	ANA_REG_GLB_LDO_WIFIPA_REG1, BITS_LDO_WIFIPA_V(~0U)},
{ "vdddcxo", 20,
	ANA_REG_GLB_POWER_PD_SW, BIT_LDO_DCXO_PD,
	ANA_REG_GLB_LDO_DCXO_REG1, BITS_LDO_DCXO_V(~0U)},
{ "vddusb33", 21,
	ANA_REG_GLB_LDO_USB_PD_REG, BIT_LDO_USB33_PD,
	ANA_REG_GLB_LDO_USB_REG1, BITS_LDO_USB33_V(~0U)},
{}
};

struct sprd_regulator_regs {
	/* BIT4: default on/off(0: off, 1: on);
	 * BIT0~BIT3: dcdc/ldo type(0: ldo; 2: dcdc)
	 */
	u32 typ;
	unsigned long pd_set;
	u32 pd_set_bit;
	unsigned long vol_trm;
	u32 vol_trm_bits;
	u32 vol_def;
	u32 hw_step;
};

struct sprd_regulator_data {
	struct regulator_dev *rdev;
};

struct sprd_regulator_desc {
	struct regulator_desc desc;
	struct regulator_init_data *init_data;
	struct sprd_regulator_regs regs;
	struct sprd_regulator_data data;
	struct dentry *debugfs;
};

static struct regmap *pmic_regmap;
static DEFINE_IDA(regu_ida);
static struct dentry *debugfs_root;

static const struct of_device_id sprd_regulator_of_match[] = {
	{.compatible = "sprd,sc2720-regulator",},
	{}
};

static inline int sprd_regu_read(unsigned long reg)
{
	int ret, value;

	ret = regmap_read(pmic_regmap, reg, (unsigned int *) &value);
	return ret ? ret : value;
}

static inline int sprd_regu_set(unsigned long reg, unsigned long or_val,
				 unsigned long clear_msk)
{
	return regmap_update_bits(pmic_regmap, reg, clear_msk, or_val);
}

static inline int sprd_regu_write(unsigned long reg, unsigned long val)
{
	return regmap_write(pmic_regmap, reg, val);
}

static struct sprd_regulator_desc *sprd_regu_get_desc(struct regulator_dev
						      *rdev)
{
	return (struct sprd_regulator_desc *)rdev->desc;
}

static inline unsigned int sprd_regu_get_max_uV(struct regulator_dev *rdev)
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

	if (!n)
		return NULL;

	dev_info(dev, "supply %s consumers %d - %d\n",
		 supply_name, i, n);
	consumer_supplies =
		devm_kzalloc(dev, n * sizeof(*consumer_supplies),
			     GFP_KERNEL);
	if (!consumer_supplies)
		return NULL;

	for (n = 0; map[i]; i++, n++)
		consumer_supplies[n].supply = map[i];

	if (num)
		*num = n;
	return consumer_supplies;
}

static int acquire_reg_info(struct sprd_regulator_desc *desc, int index)
{
	struct sprd_regulator_regs *regs = &desc->regs;
	struct sprd_regulator_data *data = &desc->data;
	struct regulator_dev *rdev = data->rdev;

	if (strcmp(desc->desc.name, reg_info_map[index].regu_name) == 0) {
		/* ldo/dcdc power down */
		regs->pd_set = reg_info_map[index].pd_reg;
		regs->pd_set_bit = reg_info_map[index].pd_mask;
		/* ldo/dcdc voltage trim */
		regs->vol_trm = reg_info_map[index].vol_reg;
		regs->vol_trm_bits = reg_info_map[index].vol_mask;
	} else {
		dev_err(&rdev->dev, "regulator %s does not match with %s!!!\n",
			desc->desc.name, reg_info_map[index].regu_name);
		return -EINVAL;
	}
	return 0;
}

static int sprd_regu_turn_on(struct regulator_dev *rdev)
{
	struct sprd_regulator_desc *desc = sprd_regu_get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;

	if (regs->pd_set)
		sprd_regu_set(regs->pd_set, 0, regs->pd_set_bit);

	dev_info(&rdev->dev, "regu 0x%p (%s), turn on\n",
		 regs, desc->desc.name);
	return 0;
}

static int sprd_regu_turn_off(struct regulator_dev *rdev)
{
	struct sprd_regulator_desc *desc = sprd_regu_get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;

	if (regs->pd_set)
		sprd_regu_set(regs->pd_set, regs->pd_set_bit,
			      regs->pd_set_bit);

	dev_info(&rdev->dev, "regu 0x%p (%s), turn off\n",
		 regs, desc->desc.name);
	return 0;
}

static int sprd_regu_is_on(struct regulator_dev *rdev)
{
	int ret = -EINVAL;
	struct sprd_regulator_desc *desc = sprd_regu_get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;

	if (regs->pd_set)
		ret = !(sprd_regu_read(regs->pd_set) & regs->pd_set_bit);

	dev_info(&rdev->dev, "regu 0x%p (%s) turn on, return %d\n",
		 regs, desc->desc.name, ret);
	return ret;
}

static int sprd_set_voltage(struct regulator_dev *rdev, int min_uV,
			   int max_uV, unsigned *selector)
{
	struct sprd_regulator_desc *desc = sprd_regu_get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;
	/* int mv = min_uV / 1000; */
	int ret = -EINVAL;

	dev_info(&rdev->dev, "regu 0x%p (%s) set voltage, %d(uV) %d(uV)\n",
		 regs, desc->desc.name, min_uV, max_uV);

	min_uV = (min_uV + max_uV) / 2;

	if (min_uV < desc->desc.min_uV) {
		dev_warn(&rdev->dev, "(%s) overrun,set to min_uV %d(uV)\n",
			 desc->desc.name, desc->desc.min_uV);
		min_uV = desc->desc.min_uV;
	}

	if (min_uV > sprd_regu_get_max_uV(rdev)) {
		dev_warn(&rdev->dev, "(%s) overrun,set to max_uV %d(uV)\n",
			 desc->desc.name, sprd_regu_get_max_uV(rdev));
		min_uV = sprd_regu_get_max_uV(rdev);
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
			sprd_regu_set(regs->vol_trm, trim << shft &
				       regs->vol_trm_bits,
				       regs->vol_trm_bits);
		}
	}

	return ret;
}

static void sprd_hw_regulator(struct regulator_dev *rdev, int present_uV,
			     int target_uV, unsigned *selector)
{
	int n_steps, tune_flag;
	struct sprd_regulator_desc *desc = sprd_regu_get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;

	sprd_regu_set(ANA_REG_GLB_VOL_TUNE_CTRL_CORE,
		      BIT_CORE_VOL_TUNE_EN, BIT_CORE_VOL_TUNE_EN);
	n_steps = DIV_ROUND_UP(abs(present_uV - target_uV), regs->hw_step);
	sprd_regu_set(ANA_REG_GLB_VOL_TUNE_CTRL_CORE,
		      BITS_CORE_STEP_NUM(n_steps),
		      BITS_CORE_STEP_NUM(~0ULL));

	sprd_set_voltage(rdev, target_uV, target_uV, selector);
	sprd_regu_set(ANA_REG_GLB_VOL_TUNE_CTRL_CORE,
		      BIT_CORE_VOL_TUNE_START, BIT_CORE_VOL_TUNE_START);
	do {
		tune_flag = sprd_regu_read(ANA_REG_GLB_VOL_TUNE_CTRL_CORE);
	} while ((tune_flag & BIT_CORE_VOL_TUNE_FLAG) !=  0);
	sprd_regu_set(ANA_REG_GLB_VOL_TUNE_CTRL_CORE, 0, BIT_CORE_VOL_TUNE_EN);
}

static int sprd_hybrid_regulator(struct regulator_dev *rdev, int min_uV,
			   int max_uV, unsigned *selector)
{
	int to_vol, base_vol, sw_step, step_vol;
	struct sprd_regulator_desc *desc = sprd_regu_get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;
	int vol = rdev->desc->ops->get_voltage(rdev);

	sw_step = regs->hw_step * MAX_STEP_NUM;
	to_vol = min_uV;
	step_vol = DIV_ROUND_UP(regs->hw_step, MIN_HW_STEP);
	sprd_regu_set(ANA_REG_GLB_VOL_TUNE_CTRL_CORE,
		      BITS_CORE_STEP_VOL(step_vol),
		      BITS_CORE_STEP_VOL(~0ULL));
	sprd_regu_set(ANA_REG_GLB_VOL_TUNE_CTRL_CORE,
		      BITS_CORE_STEP_DELAY(0xf),
		      BITS_CORE_STEP_DELAY(~0ULL));

	sprd_regu_set(ANA_REG_GLB_VOL_TUNE_CTRL_CORE,
		      BIT_CORE_VOL_TUNE_EN, BIT_CORE_VOL_TUNE_EN);

	if (vol < to_vol) {
		do {
			base_vol = vol;
			vol += sw_step;
			if (vol > to_vol)
				vol = to_vol;
			sprd_hw_regulator(rdev, base_vol, vol, selector);
		} while (vol < to_vol);
	} else if (vol > to_vol) {
		do {
			base_vol = vol;
			vol -= sw_step;
			if (vol < to_vol)
				vol = to_vol;
			sprd_hw_regulator(rdev, base_vol, vol, selector);
		} while (vol > to_vol);
	}

	return 0;
}

static int sprd_dcdc_set_voltage_step(struct regulator_dev *rdev, int min_uV,
				 int max_uV, unsigned *selector)
{
	int to_vol, step;
	int vol = rdev->desc->ops->get_voltage(rdev);

	to_vol = min_uV;
	/* uV */
	step = REGULATOR_SCALING_STEP;

	if (vol < to_vol) {
		do {
			vol += step;
			if (vol > to_vol)
				vol = to_vol;
			sprd_set_voltage(rdev, vol, vol, selector);
		} while (vol < to_vol);
	} else if (vol > to_vol) {
		do {
			vol -= step;
			if (vol < to_vol)
				vol = to_vol;
			sprd_set_voltage(rdev, vol, vol, selector);
		} while (vol > to_vol);
	}

	return 0;
}

static int sprd_get_voltage(struct regulator_dev *rdev)
{
	struct sprd_regulator_desc *desc = sprd_regu_get_desc(rdev);
	struct sprd_regulator_regs *regs = &desc->regs;
	u32 uV = 0;

	if (regs->vol_trm) {
		int shft_trm = __ffs(regs->vol_trm_bits);
		u32 trim =
		    (sprd_regu_read(regs->vol_trm) & regs->vol_trm_bits) >>
		    shft_trm;

		uV = desc->desc.min_uV + trim * desc->desc.uV_step;
	}

	return uV;
}

static struct regulator_ops ldo_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.enable = sprd_regu_turn_on,
	.disable = sprd_regu_turn_off,
	.is_enabled = sprd_regu_is_on,
	.set_voltage = sprd_set_voltage,
	.get_voltage = sprd_get_voltage,
};

static struct regulator_ops dcdc_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.enable = sprd_regu_turn_on,
	.disable = sprd_regu_turn_off,
	.is_enabled = sprd_regu_is_on,
	.set_voltage = sprd_dcdc_set_voltage_step,
	.get_voltage = sprd_get_voltage,
};

static struct regulator_ops hybrid_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.enable = sprd_regu_turn_on,
	.disable = sprd_regu_turn_off,
	.is_enabled = sprd_regu_is_on,
	.set_voltage = sprd_hybrid_regulator,
	.get_voltage = sprd_get_voltage,
};

static int sprd_regu_get_state(void *data, u64 *val)
{
	struct regulator_dev *rdev = data;

	if (rdev && rdev->desc->ops->is_enabled)
		*val = rdev->desc->ops->is_enabled(rdev);
	else
		*val = ~0ULL;
	return 0;
}

static int sprd_regu_set_state(void *data, u64 val)
{
	struct regulator_dev *rdev = data;

	if (rdev && rdev->desc->ops->enable && rdev->desc->ops->enable)
		val ? rdev->desc->ops->enable(rdev)
			: rdev->desc->ops->disable(rdev);
	return 0;
}

static int sprd_regu_get_vol(void *data, u64 *val)
{
	struct regulator_dev *rdev = data;

	if (rdev)
		*val = rdev->desc->ops->get_voltage(rdev) / 1000;
	else
		*val = ~0ULL;
	return 0;
}

static int sprd_regu_set_vol(void *data, u64 val)
{
	struct regulator_dev *rdev = data;
	u32 min_uV;

	if (rdev && rdev->desc->ops->set_voltage) {
		min_uV = (u32)val * 1000;
		rdev->desc->ops->set_voltage(rdev, min_uV, min_uV,
					     (unsigned *) 0);
	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_enable,
			sprd_regu_get_state, sprd_regu_set_state, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_ldo,
			sprd_regu_get_vol, sprd_regu_set_vol, "%llu\n");

static void sprd_regu_create_debugfs(struct regulator_dev *rdev)
{
	struct sprd_regulator_desc *desc = sprd_regu_get_desc(rdev);

	desc->debugfs = debugfs_create_dir(rdev->desc->name, debugfs_root);

	if (IS_ERR_OR_NULL(rdev->debugfs)) {
		dev_err(&rdev->dev, "Failed to create debugfs directory\n");
		rdev->debugfs = NULL;
		return;
	}

	debugfs_create_file("enable", S_IRUGO | S_IWUSR,
			    desc->debugfs, rdev, &fops_enable);
	debugfs_create_file("voltage", S_IRUGO | S_IWUSR,
			    desc->debugfs, rdev, &fops_ldo);
}

static int sprd_regu_parse_dt(struct platform_device *pdev,
			      struct device_node *np,
			      struct sprd_regulator_desc *desc,
			      struct regulator_consumer_supply *supply,
			      int sz)
{
	struct sprd_regulator_regs *regs = &desc->regs;
	int index = -1;
	u32 tmp_val_u32;
	int ret;

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
	desc->desc.id = ida_simple_get(&regu_ida, 0, 0, GFP_KERNEL);
	if (desc->desc.id < 0)
		return desc->desc.id;

	desc->desc.type = REGULATOR_VOLTAGE;
	desc->desc.owner = THIS_MODULE;
	desc->desc.continuous_voltage_range = true;

	supply[0].dev_name = NULL;
	supply[0].supply = desc->init_data->constraints.name;
	desc->init_data->supply_regulator = NULL;
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

	if (!strcmp("dcdc", np->name))
		regs->typ = 2;
	else
		regs->typ = 0;

	if (of_property_read_bool(np, "sprd,hw-regulator"))
		regs->typ = 4;

	ret = of_property_read_u32(np, "sprd,hw-step", &tmp_val_u32);
	if (!ret)
		regs->hw_step = tmp_val_u32;

	if (of_property_read_bool(np, "sprd,default-on"))
		regs->typ |= BIT(4);

	desc->desc.min_uV = desc->init_data->constraints.min_uV;

	ret = of_property_read_u32(np, "reg", &tmp_val_u32);
	if (!ret)
		index = tmp_val_u32;

	if (-1 != index) {
		if (acquire_reg_info(desc, index))
			dev_err(&pdev->dev, "acquire reg info:%d fail!!!\n",
				index);
	} else {
		dev_err(&pdev->dev, "get index:%d fail!!!\n", index);
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "sprd,step-microvolt", &tmp_val_u32);
	if (!ret)
		desc->desc.uV_step = tmp_val_u32;

	ret = of_property_read_u32(np, "sprd,default-microvolt", &tmp_val_u32);
	if (!ret)
		regs->vol_def = tmp_val_u32;

	desc->desc.n_voltages =
	    (desc->init_data->constraints.max_uV - (desc->desc.min_uV))
	    / (desc->desc.uV_step) + 1;

	return 0;
}

static int sprd_regu_register(struct platform_device *pdev)
{
	struct sprd_regulator_desc *sprd_desc = NULL;
	struct regulator_dev *rdev;
	struct regulator_ops *regs_ops[] = {
		&ldo_ops, NULL, &dcdc_ops, NULL, &hybrid_ops,
		NULL,
	};
	struct regulator_consumer_supply consumer_supplies_default[1];
	struct regulator_config config;
	struct device_node *dev_np;
	struct device_node *child_np;
	int regu_cnt, ret;

	dev_np = of_find_node_by_name(pdev->dev.of_node, REGULAOR_DT_NODE);
	if (!dev_np)
		return -ENODEV;

	regu_cnt = of_get_child_count(dev_np);
	if (!regu_cnt)
		return -EINVAL;

	sprd_desc = devm_kzalloc(&pdev->dev,
				 regu_cnt * sizeof(struct sprd_regulator_desc),
				 GFP_KERNEL);
	if (!sprd_desc)
		return -ENOMEM;

	platform_set_drvdata(pdev, sprd_desc);

	for_each_available_child_of_node(dev_np, child_np) {
		ret =
		    sprd_regu_parse_dt(pdev, child_np, sprd_desc,
				       consumer_supplies_default,
				       ARRAY_SIZE(consumer_supplies_default));
		if (ret) {
			dev_err(&pdev->dev,
				"failed to parse regulator(%s) dts\n",
				child_np->name);
			continue;
		}

		WARN_ON((sprd_desc->regs.typ & DESC_TYPE_MASK) >=
		       ARRAY_SIZE(regs_ops));

		if (!sprd_desc->desc.ops)
			sprd_desc->desc.ops =
			    regs_ops[sprd_desc->regs.typ & DESC_TYPE_MASK];

		config.dev = &pdev->dev;
		config.init_data = sprd_desc->init_data;
		config.driver_data = NULL;
		config.of_node = child_np;
		config.regmap = NULL;
		config.ena_gpio = -EINVAL;
		config.ena_gpio_initialized = false;

		rdev = devm_regulator_register(&pdev->dev, &sprd_desc->desc,
					       &config);

		dev_info(&pdev->dev, "regulator_desc 0x%p, rdev 0x%p\n",
			 &sprd_desc->desc, rdev);

		if (!IS_ERR_OR_NULL(rdev)) {
			rdev->reg_data = rdev;
			sprd_desc->data.rdev = rdev;
			if (rdev->desc->ops->is_enabled(rdev))
				rdev->use_count = 1;
			sprd_regu_create_debugfs(rdev);
		}

		sprd_desc++;
	}

	return 0;
}

static int sprd_regulator_probe(struct platform_device *pdev)
{
	int ret;

	pmic_regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pmic_regmap) {
		dev_err(&pdev->dev, "NULL spi parent property for regulator.");
		return -ENODEV;
	}

	debugfs_root = debugfs_create_dir(REGULATOR_ROOT_DIR, NULL);
	if (IS_ERR_OR_NULL(debugfs_root)) {
		WARN_ONCE(!debugfs_root,
		     "%s: Failed to create debugfs directory\n",
		     REGULATOR_ROOT_DIR);
		debugfs_root = NULL;
	}

	sprd_regu_write(ANA_REG_GLB_PWR_WR_PROT_VALUE,
			BITS_PWR_WR_PROT_VALUE(WR_UNLOCK));

	ret = sprd_regu_register(pdev);
	if (ret)
		dev_err(&pdev->dev, "can not register regulator dt\n");

	return ret;
}

static int sprd_regulator_remove(struct platform_device *pdev)
{
	struct sprd_regulator_desc *sprd_desc = platform_get_drvdata(pdev);

	do {
		ida_simple_remove(&regu_ida, sprd_desc->desc.id);
	} while (++sprd_desc != NULL);

	debugfs_remove(debugfs_root);

	return 0;
}

static struct platform_driver sprd_regulator_driver = {
	.driver = {
		   .name = "sc2720-regulator",
		   .of_match_table = sprd_regulator_of_match,
	},
	.probe = sprd_regulator_probe,
	.remove = sprd_regulator_remove
};

static int __init sprd_regulator_init(void)
{
	return platform_driver_register(&sprd_regulator_driver);
}

static void __init sprd_regulator_exit(void)
{
	platform_driver_unregister(&sprd_regulator_driver);
}

subsys_initcall(sprd_regulator_init);
module_exit(sprd_regulator_exit);

MODULE_DESCRIPTION("Spreadtrum sc2720 regulator driver");
MODULE_AUTHOR("Chen Junhui <erick.chen@spreadtrum.com>");
MODULE_LICENSE("GPL");
