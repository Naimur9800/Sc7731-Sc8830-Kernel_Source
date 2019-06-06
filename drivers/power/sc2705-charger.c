/*
 * Charger device driver for SC2705
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

#include <linux/extcon.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/iio/consumer.h>
#include <linux/mfd/sc2705/core.h>
#include <linux/mfd/sc2705/registers.h>
#include <linux/delay.h>
#include "sprd_charge_helper.h"

#define SPRD_EX_DEBUG(format, arg...) pr_info("sc2705_chg: "format, ## arg)
/* The range of cccv value is 3.80v-4.80v, the default vol is 3.80v */
#define DEFAULT_CCCV_VOL 3800

/* Private data */
struct sc2705_extcon_cable {
	struct extcon_dev *edev;
	struct notifier_block nb;
};

struct sc2705_charger {
	struct sc2705 *sc2705;
	struct device *dev;

	struct power_supply *usb;
	struct power_supply *battery;
	struct iio_channel *vbat_chan;
	struct iio_channel *ibat_chan;
	struct iio_channel *vin_chan;
	struct iio_channel *iin_chan;
	struct iio_channel *tjunc_chan;
	struct iio_channel *tbat_chan;

	struct delayed_work wd_work;
	unsigned int wd_interval;
	unsigned int wd_kick_interval;

	bool timeout_safety;
	bool timeout_wd;

	struct sc2705_extcon_cable ext_id;
	struct delayed_work otg_work;
};

#define SC2705_CHG_B_IMIN		500000
#define SC2705_CHG_B_ISTEP		50000
#define SC2705_CHG_B_IMAX		3500000
#define SC2705_CHG_B_VMIN		3800000
#define SC2705_CHG_B_VSTEP		20000
#define SC2705_CHG_B_VMAX		4800000
#define SC2705_VIN_DROP_MIN		4300000
#define SC2705_VIN_DROP_STEP		50000
#define SC2705_VIN_DROP_MAX		12000000
#define SC2705_CHGV_MAX			13500000
#define SC2705_OTG_RETRY_TIMES		10

/* Values in uA */
static unsigned int iin_lim_tbl[] = {
	100000,
	150000,
	500000,
	900000,
	1500000,
	2000000,
	2500000,
	3000000,
};

static struct sc2705_charger *sc2705_chg;

static const struct of_device_id sc2705_charger_of_match[] = {
	{ .compatible = "sprd,sc2705-charger", },
	{ }
};
MODULE_DEVICE_TABLE(of, sc2705_charger_of_match);

static u8 sc2705_charger_of_prop_range(struct device *dev, u32 val, u32 min,
				       u32 max, u32 step, u8 default_val,
				       const char *name)
{
	if ((val < min) || (val > max) || (val % step)) {
		dev_warn(dev, "Invalid %s value\n", name);
		return default_val;
	} else {
		return (val - min) / step;
	}
}

static enum sc2705_ibat_pre
sc2705_charger_of_ibat_pre(struct device *dev, u32 val)
{
	switch (val) {
	case 50000:
		return SC2705_IBAT_PRE_50MA;
	case 100000:
		return SC2705_IBAT_PRE_100MA;
	case 200000:
		return SC2705_IBAT_PRE_200MA;
	case 400000:
		return SC2705_IBAT_PRE_400MA;
	default:
		dev_warn(dev, "Invalid ibat-pre value\n");
		return SC2705_IBAT_PRE_DEFAULT;
	}
}

static enum sc2705_t_eoc sc2705_charger_of_t_eoc(struct device *dev, u32 val)
{
	switch (val) {
	case 0:
		return SC2705_T_EOC_0MIN;
	case 60:
		return SC2705_T_EOC_1MIN;
	case 300:
		return SC2705_T_EOC_5MIN;
	case 600:
		return SC2705_T_EOC_10MIN;
	case 1200:
		return SC2705_T_EOC_20MIN;
	case 1800:
		return SC2705_T_EOC_30MIN;
	case 2700:
		return SC2705_T_EOC_45MIN;
	case 3600:
		return SC2705_T_EOC_60MIN;
	default:
		dev_warn(dev, "Invalid t-eoc value\n");
		return SC2705_T_EOC_DEFAULT;
	}
}

static enum sc2705_vbat_rechg
sc2705_charger_of_vbat_rechg(struct device *dev, u32 val)
{
	switch (val) {
	case 100000:
		return SC2705_VBAT_RECHG_100MA;
	case 160000:
		return SC2705_VBAT_RECHG_160MA;
	case 200000:
		return SC2705_VBAT_RECHG_200MA;
	case 240000:
		return SC2705_VBAT_RECHG_240MA;
	default:
		dev_warn(dev, "Invalid vbat-rechg value\n");
		return SC2705_VBAT_RECHG_DEFAULT;
	}
}

static enum sc2705_timeout_cccv
sc2705_charger_of_timeout_cccv(struct device *dev, u32 val)
{
	switch (val) {
	case 7200:
		return SC2705_TIMEOUT_CCCV_2HOURS;
	case 14400:
		return SC2705_TIMEOUT_CCCV_4HOURS;
	case 21600:
		return SC2705_TIMEOUT_CCCV_6HOURS;
	case 28800:
		return SC2705_TIMEOUT_CCCV_8HOURS;
	case 36000:
		return SC2705_TIMEOUT_CCCV_10HOURS;
	case 43200:
		return SC2705_TIMEOUT_CCCV_12HOURS;
	case 50400:
		return SC2705_TIMEOUT_CCCV_14HOURS;
	case 64800:
		return SC2705_TIMEOUT_CCCV_18HOURS;
	default:
		dev_warn(dev, "Invalid timeout-cccv value\n");
		return SC2705_TIMEOUT_CCCV_DEFAULT;
	}
}

static enum sc2705_iin_rev_lim
sc2705_charger_of_iin_rev_lim(struct device *dev, u32 val)
{
	switch (val) {
	case 500000:
		return SC2705_IIN_REV_LIM_500MA;
	case 900000:
		return SC2705_IIN_REV_LIM_900MA;
	case 1200000:
		return SC2705_IIN_REV_LIM_1200MA;
	case 1500000:
		return SC2705_IIN_REV_LIM_1500MA;
	case 2000000:
		return SC2705_IIN_REV_LIM_2000MA;
	default:
		dev_warn(dev, "Invalid iin-rev-lim value\n");
		return SC2705_IIN_REV_LIM_DEFAULT;
	}
}

static enum sc2705_bat_det_src
sc2705_charger_of_bat_det_src(struct device *dev, const char *str)
{
	if (!strcmp(str, "Off"))
		return SC2705_BAT_DET_SRC_OFF;
	else if (!strcmp(str, "VBAT"))
		return SC2705_BAT_DET_SRC_VBAT;
	else if (!strcmp(str, "BATID"))
		return SC2705_BAT_DET_SRC_BATID;
	else if (!strcmp(str, "TBAT"))
		return SC2705_BAT_DET_SRC_TBAT;

	dev_warn(dev, "Invalid bat-det-src value\n");
	return SC2705_BAT_DET_SRC_DEFAULT;
}

static struct sc2705_charger_pdata *
sc2705_charger_of_to_pdata(struct device *dev)
{
	struct device_node *charger_np = dev->of_node;
	struct sc2705_charger_pdata *charger_pdata;
	u32 of_val32;
	const char *of_str;
	struct extcon_dev *edev;

	charger_pdata = devm_kzalloc(dev, sizeof(*charger_pdata), GFP_KERNEL);
	if (!charger_pdata)
		return NULL;

	if (!of_property_read_u32(charger_np, "sprd,ibat-pre-microamp",
				 &of_val32))
		charger_pdata->ibat_pre =
			sc2705_charger_of_ibat_pre(dev, of_val32);
	else
		charger_pdata->ibat_pre = SC2705_IBAT_PRE_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,vbat-chg-microvolt",
				 &of_val32))
		charger_pdata->vbat_chg =
			sc2705_charger_of_prop_range(dev, of_val32,
				SC2705_CHG_B_VMIN, SC2705_CHG_B_VMAX,
				SC2705_CHG_B_VSTEP,
				SC2705_VBAT_CHG_DEFAULT, "vbat-chg");
	else
		charger_pdata->vbat_chg = SC2705_VBAT_CHG_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,ibat-chg-microamp",
				 &of_val32))
		charger_pdata->ibat_chg =
			sc2705_charger_of_prop_range(dev, of_val32,
				SC2705_CHG_B_IMIN, SC2705_CHG_B_IMAX,
				SC2705_CHG_B_ISTEP,
				SC2705_IBAT_CHG_DEFAULT, "ibat-chg");
	else
		charger_pdata->ibat_chg = SC2705_IBAT_CHG_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,ibat-term-microamp",
				 &of_val32))
		charger_pdata->ibat_term =
			sc2705_charger_of_prop_range(dev, of_val32, 100000,
						     450000, 50000,
						     SC2705_IBAT_TERM_DEFAULT,
						     "ibat-term");
	else
		charger_pdata->ibat_term = SC2705_IBAT_TERM_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,t-eoc-sec", &of_val32))
		charger_pdata->t_eoc =
			sc2705_charger_of_t_eoc(dev, of_val32);
	else
		charger_pdata->t_eoc = SC2705_T_EOC_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,vbat-rechg-microvolt",
				 &of_val32))
		charger_pdata->vbat_rechg =
			sc2705_charger_of_vbat_rechg(dev, of_val32);
	else
		charger_pdata->vbat_rechg = SC2705_VBAT_RECHG_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,dcdc-peak-ilim-microamp",
				 &of_val32))
		charger_pdata->dcdc_peak_ilim =
			sc2705_charger_of_prop_range(dev, of_val32, 6000000,
					     9000000, 1000000,
					     SC2705_DCDC_PEAK_ILIM_DEFAULT,
					     "dcdc-peak-ilim");
	else
		charger_pdata->dcdc_peak_ilim = SC2705_DCDC_PEAK_ILIM_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,vbat-ov-microvolt",
				 &of_val32))
		charger_pdata->vbat_ov =
			sc2705_charger_of_prop_range(dev, of_val32, 4000000,
						     5500000, 100000,
						     SC2705_VBAT_OV_DEFAULT,
						     "vbat-ov");
	else
		charger_pdata->vbat_ov = SC2705_VBAT_OV_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,vbat-uv-microvolt",
				 &of_val32))
		charger_pdata->vbat_uv =
			sc2705_charger_of_prop_range(dev, of_val32, 2200000,
						     2950000, 50000,
						     SC2705_VBAT_UV_DEFAULT,
						     "vbat-uv");
	else
		charger_pdata->vbat_uv = SC2705_VBAT_UV_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,vsys-min-microvolt",
				 &of_val32))
		charger_pdata->vsys_min =
			sc2705_charger_of_prop_range(dev, of_val32, 3000000,
						     3900000, 60000,
						     SC2705_VSYS_MIN_DEFAULT,
						     "vsys-min");
	else
		charger_pdata->vsys_min = SC2705_VSYS_MIN_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,timeout-pre-sec",
				 &of_val32))
		charger_pdata->timeout_pre =
			sc2705_charger_of_prop_range(dev, of_val32, 900, 3600,
						     900,
						     SC2705_TIMEOUT_PRE_DEFAULT,
						     "timeout-pre");
	else
		charger_pdata->timeout_pre = SC2705_TIMEOUT_PRE_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,timeout-cccv-sec",
				 &of_val32))
		charger_pdata->timeout_cccv =
			sc2705_charger_of_timeout_cccv(dev, of_val32);
	else
		charger_pdata->timeout_pre = SC2705_TIMEOUT_CCCV_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,timer-load-sec",
				 &of_val32))
		charger_pdata->timer_load =
			sc2705_charger_of_prop_range(dev, of_val32, 0, 255, 1,
						     0, "timer-load");

	if (!of_property_read_u32(charger_np, "sprd,iin-rev-lim-microamp",
				&of_val32))
		charger_pdata->iin_rev_lim =
			sc2705_charger_of_iin_rev_lim(dev, of_val32);
	else
		charger_pdata->timeout_pre = SC2705_IIN_REV_LIM_DEFAULT;

	if (!of_property_read_string(charger_np, "sprd,bat-det-src", &of_str))
		charger_pdata->bat_det_src =
			sc2705_charger_of_bat_det_src(dev, of_str);
	else
		charger_pdata->bat_det_src = SC2705_BAT_DET_SRC_DEFAULT;

	if (of_property_read_bool(charger_np, "extcon")) {
		edev = extcon_get_edev_by_phandle(dev, 0);
		if (IS_ERR(edev)) {
			if (PTR_ERR(edev) == -EPROBE_DEFER)
				return ERR_CAST(edev);
			dev_warn(dev,
				"Failed to get extcon device: %ld\n",
				PTR_ERR(edev));
		}
		charger_pdata->ext_id = edev;
	}

	if (!of_property_read_u32(charger_np, "sprd,tbat-t3-celsius",
				 &of_val32))
		charger_pdata->tbat_t3 =
			sc2705_charger_of_prop_range(dev, of_val32, 40, 50, 5,
						     SC2705_TBAT_T3_DEFAULT,
						     "tbat-t3");
	else
		charger_pdata->tbat_t3 = SC2705_TBAT_T3_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,tbat-t4-celsius",
				 &of_val32))
		charger_pdata->tbat_t4 =
			sc2705_charger_of_prop_range(dev, of_val32, 55, 65, 5,
						     SC2705_TBAT_T4_DEFAULT,
						     "tbat-t4");
	else
		charger_pdata->tbat_t4 = SC2705_TBAT_T4_DEFAULT;

	if (!of_property_read_u32(charger_np, "sprd,vbat-cold-microvolt",
				 &of_val32))
		charger_pdata->vbat_cold =
			sc2705_charger_of_prop_range(dev, of_val32, 0, 200000,
						     100000,
						     SC2705_VBAT_COLD_DEFAULT,
						     "vbat-cold");
	else
		charger_pdata->vbat_cold = SC2705_VBAT_COLD_DEFAULT;

	charger_pdata->ibat_cold_enable =
		of_property_read_bool(charger_np, "sprd,ibat-cold-enable");

	if (!of_property_read_u32(charger_np, "sprd,vbat-warm-microvolt",
				 &of_val32))
		charger_pdata->vbat_warm =
			sc2705_charger_of_prop_range(dev, of_val32, 0, 200000,
						     100000,
						     SC2705_VBAT_WARM_DEFAULT,
						     "vbat-warm");
	else
		charger_pdata->vbat_warm = SC2705_VBAT_WARM_DEFAULT;

	charger_pdata->ibat_warm_enable =
		of_property_read_bool(charger_np, "sprd,ibat-warm-enable");

	return charger_pdata;
}

static void sc2705_charger_handle_pdata(struct sc2705_charger *charger)
{
	struct device *dev = charger->dev;
	struct regmap *regmap = charger->sc2705->regmap;
	struct sc2705_charger_pdata *charger_pdata =
		dev_get_platdata(charger->dev);
	int ret;

	if (charger_pdata) {
		unsigned int cfg = 0, mask = 0;

		switch (charger_pdata->ibat_pre) {
		case SC2705_IBAT_PRE_50MA:
		case SC2705_IBAT_PRE_100MA:
		case SC2705_IBAT_PRE_200MA:
		case SC2705_IBAT_PRE_400MA:
			cfg |= (charger_pdata->ibat_pre <<
				SC2705_IBAT_PRE_SHIFT);
			mask |= SC2705_IBAT_PRE_MASK;
		}

		if (charger_pdata->ibat_term <= SC2705_IBAT_TERM_MAX) {
			cfg |= (charger_pdata->ibat_term <<
				SC2705_IBAT_TERM_SHIFT);
			mask |= SC2705_IBAT_TERM_MASK;
		}

		switch (charger_pdata->t_eoc) {
		case SC2705_T_EOC_0MIN:
		case SC2705_T_EOC_1MIN:
		case SC2705_T_EOC_5MIN:
		case SC2705_T_EOC_10MIN:
		case SC2705_T_EOC_20MIN:
		case SC2705_T_EOC_30MIN:
		case SC2705_T_EOC_45MIN:
		case SC2705_T_EOC_60MIN:
			cfg |= (charger_pdata->t_eoc << SC2705_T_EOC_SHIFT);
			mask |= SC2705_T_EOC_MASK;
		}

		ret = regmap_update_bits(regmap, SC2705_CHG_CTRL_D, mask, cfg);
		if (ret)
			dev_err(dev, "Failed to write pdata (CHG_CTRL_D): %d\n",
				ret);

		cfg = 0;
		mask = 0;
		if (charger_pdata->vbat_chg <= SC2705_VBAT_CHG_MAX) {
			cfg |= (charger_pdata->vbat_chg <<
				SC2705_VBAT_CHG_SHIFT);
			mask |= SC2705_VBAT_CHG_MASK;
		}

		switch (charger_pdata->vbat_rechg) {
		case SC2705_VBAT_RECHG_100MA:
		case SC2705_VBAT_RECHG_160MA:
		case SC2705_VBAT_RECHG_200MA:
		case SC2705_VBAT_RECHG_240MA:
			cfg |= (charger_pdata->vbat_rechg <<
				SC2705_VBAT_RECHG_SHIFT);
			mask |= SC2705_VBAT_RECHG_MASK;
		}

		ret = regmap_update_bits(regmap, SC2705_CHG_CTRL_A, mask, cfg);
		if (ret)
			dev_err(dev, "Failed to write pdata (CHG_CTRL_A): %d\n",
				ret);

		cfg = 0;
		mask = 0;
		if (charger_pdata->ibat_chg <= SC2705_IBAT_CHG_MAX) {
			cfg |= (charger_pdata->ibat_chg <<
				SC2705_IBAT_CHG_SHIFT);
			mask |= SC2705_IBAT_CHG_MASK;
		}

		ret = regmap_update_bits(regmap, SC2705_CHG_CTRL_B, mask, cfg);
		if (ret)
			dev_err(dev, "Failed to write pdata (CHG_CTRL_B): %d\n",
				ret);

		cfg = 0;
		mask = 0;
		if (charger_pdata->dcdc_peak_ilim <=
					SC2705_DCDC_PEAK_ILIM_MAX) {
			cfg |= (charger_pdata->dcdc_peak_ilim <<
				SC2705_DCDC_PEAK_ILIM_SHIFT);
			mask |= SC2705_DCDC_PEAK_ILIM_MASK;
		}

		switch (charger_pdata->iin_rev_lim) {
		case SC2705_IIN_REV_LIM_500MA:
		case SC2705_IIN_REV_LIM_900MA:
		case SC2705_IIN_REV_LIM_1200MA:
		case SC2705_IIN_REV_LIM_1500MA:
		case SC2705_IIN_REV_LIM_2000MA:
			cfg |= (charger_pdata->iin_rev_lim <<
				SC2705_IIN_REV_LIM_SHIFT);
			mask |= SC2705_IIN_REV_LIM_MASK;
		}

		ret = regmap_update_bits(regmap, SC2705_DCDC_CTRL_B, mask, cfg);
		if (ret)
			dev_err(dev, "Failed to write pdata (DCDC_CTRL_B): %d\n",
				ret);

		cfg = 0;
		mask = 0;
		if (charger_pdata->vbat_uv <= SC2705_VBAT_UV_MAX) {
			cfg |= (charger_pdata->vbat_uv <<
				SC2705_VBAT_UV_SHIFT);
			mask |= SC2705_VBAT_UV_MASK;
		}

		if (charger_pdata->vbat_ov <= SC2705_VBAT_OV_MAX) {
			cfg |= (charger_pdata->vbat_ov <<
				SC2705_VBAT_OV_SHIFT);
			mask |= SC2705_VBAT_OV_MASK;
		}

		ret = regmap_update_bits(regmap, SC2705_VBAT_CTRL_A, mask, cfg);
		if (ret)
			dev_err(dev, "Failed to write pdata (VBAT_CTRL_A): %d\n",
				ret);

		cfg = 0;
		mask = 0;
		if (charger_pdata->vsys_min <= SC2705_VSYS_MIN_MAX) {
			cfg |= (charger_pdata->vsys_min <<
				SC2705_VSYS_MIN_SHIFT);
			mask |= SC2705_VSYS_MIN_MASK;
		}

		ret = regmap_update_bits(regmap, SC2705_CHG_CTRL_C, mask, cfg);
		if (ret)
			dev_err(dev, "Failed to write pdata (CHG_CTRL_C): %d\n",
				ret);

		cfg = 0;
		mask = 0;
		switch (charger_pdata->timeout_cccv) {
		case SC2705_TIMEOUT_CCCV_2HOURS:
		case SC2705_TIMEOUT_CCCV_4HOURS:
		case SC2705_TIMEOUT_CCCV_6HOURS:
		case SC2705_TIMEOUT_CCCV_8HOURS:
		case SC2705_TIMEOUT_CCCV_10HOURS:
		case SC2705_TIMEOUT_CCCV_12HOURS:
		case SC2705_TIMEOUT_CCCV_14HOURS:
		case SC2705_TIMEOUT_CCCV_18HOURS:
			cfg |= (charger_pdata->timeout_cccv <<
				SC2705_TIMEOUT_CCCV_SHIFT);
			mask |= SC2705_TIMEOUT_CCCV_MASK;
		}

		if (charger_pdata->timeout_pre <= SC2705_TIMEOUT_PRE_MAX) {
			cfg |= (charger_pdata->timeout_pre <<
				SC2705_TIMEOUT_PRE_SHIFT);
			mask |= SC2705_TIMEOUT_PRE_MASK;
		}

		ret = regmap_update_bits(regmap, SC2705_CHG_TIMER_CTRL_A, mask,
					 cfg);
		if (ret)
			dev_err(dev,
				"Failed to write pdata (CHG_TIMER_CTRL_A): %d\n",
				ret);

		charger->wd_interval = charger_pdata->timer_load;
		charger->wd_kick_interval = (charger->wd_interval * 1000) / 4;
		ret = regmap_update_bits(regmap, SC2705_CONF_A,
					 SC2705_WD_EN_MASK,
					 (charger->wd_interval) ?
					 SC2705_WD_EN_MASK : 0);
		if (ret)
			dev_err(dev, "Failed to set watchdog: %d\n", ret);

		cfg = 0;
		mask = 0;
		switch (charger_pdata->bat_det_src) {
		case SC2705_BAT_DET_SRC_OFF:
		case SC2705_BAT_DET_SRC_VBAT:
		case SC2705_BAT_DET_SRC_BATID:
		case SC2705_BAT_DET_SRC_TBAT:
			cfg |= (charger_pdata->bat_det_src <<
				SC2705_BAT_DET_SRC_SHIFT);
			mask |= SC2705_BAT_DET_SRC_MASK;
		}

		ret = regmap_update_bits(regmap, SC2705_CONF_A, mask, cfg);
		if (ret)
			dev_err(dev, "Failed to write pdata (CONF_A): %d\n",
				ret);

		charger->ext_id.edev = charger_pdata->ext_id;

		cfg = 0;
		mask = 0;
		if (charger_pdata->tbat_t3 <= SC2705_TBAT_T3_MAX) {
			cfg |= (charger_pdata->tbat_t3 << SC2705_TBAT_T3_SHIFT);
			mask |= SC2705_TBAT_T3_MASK;
		}

		if (charger_pdata->tbat_t4 <= SC2705_TBAT_T4_MAX) {
			cfg |= (charger_pdata->tbat_t4 << SC2705_TBAT_T4_SHIFT);
			mask |= SC2705_TBAT_T4_MASK;
		}

		ret = regmap_update_bits(regmap, SC2705_TBAT_CTRL_A, mask, cfg);
		if (ret)
			dev_err(dev, "Failed to write pdata (TBAT_CTRL_A): %d\n",
				ret);

		cfg = 0;
		mask = 0;
		if (charger_pdata->ibat_cold_enable) {
			cfg |= SC2705_IBAT_COLD_MASK;
			mask |= SC2705_IBAT_COLD_MASK;
		}

		if (charger_pdata->ibat_warm_enable) {
			cfg |= SC2705_IBAT_WARM_MASK;
			mask |= SC2705_IBAT_WARM_MASK;
		}

		if (charger_pdata->vbat_cold <= SC2705_VBAT_COLD_MAX) {
			cfg |= (charger_pdata->vbat_cold <<
				SC2705_VBAT_COLD_SHIFT);
			mask |= SC2705_VBAT_COLD_MASK;
		}

		if (charger_pdata->vbat_warm <= SC2705_VBAT_WARM_MAX) {
			cfg |= (charger_pdata->vbat_warm <<
				SC2705_VBAT_WARM_SHIFT);
			mask |= SC2705_VBAT_WARM_MASK;
		}

		ret = regmap_update_bits(regmap, SC2705_CHG_CTRL_E, mask, cfg);
		if (ret)
			dev_err(dev, "Failed to write pdata (CHG_CTRL_E): %d\n",
				ret);
	}
}

static void sc2705_clr_event(void)
{
	struct regmap *regmap = sc2705_chg->sc2705->regmap;

	regmap_write(regmap, SC2705_EVENT_A, 0xFF);
	regmap_write(regmap, SC2705_EVENT_B, 0xFF);
	regmap_write(regmap, SC2705_EVENT_C,
			0xFF & (~SC2705_E_ADC_DONE_MASK));
	regmap_write(regmap, SC2705_EVENT_D, 0xFF);
}

void sc2705_charger_ic_init(void)
{
	/* Do nothing, init chip in probe function by dts config */
}

void sc2705_charger_start(void)
{
	struct regmap *regmap = sc2705_chg->sc2705->regmap;

	sc2705_clr_event();
	regmap_update_bits(regmap, SC2705_DCDC_CTRL_A,
				  SC2705_CHG_EN_MASK|SC2705_DCDC_EN_MASK,
				  SC2705_CHG_EN_MASK|SC2705_DCDC_EN_MASK);
}

void sc2705_charger_stop(unsigned int flag)
{
	struct regmap *regmap = sc2705_chg->sc2705->regmap;

	if (flag & (SPRDBAT_CHG_END_OVP_BIT | SPRDBAT_CHG_END_UNSPEC
	    | SPRDBAT_CHG_END_FORCE_STOP_BIT | SPRDBAT_CHG_END_CHGR_OTP_BIT))
		regmap_update_bits(regmap, SC2705_DCDC_CTRL_A,
					SC2705_CHG_EN_MASK|SC2705_DCDC_EN_MASK,
					0);
	else
		regmap_update_bits(regmap, SC2705_DCDC_CTRL_A,
					    SC2705_CHG_EN_MASK,
					    0);
}

unsigned int sc2705_charger_cc_current_get(void)
{
	unsigned int chg_ctrl_b;
	int ret;
	struct regmap *regmap = sc2705_chg->sc2705->regmap;

	ret = regmap_read(regmap, SC2705_CHG_CTRL_B, &chg_ctrl_b);
	if (ret)
		return ret;

	SPRD_EX_DEBUG("%s,chg_ctrl_b:%d\n", __func__, chg_ctrl_b);

	return ((chg_ctrl_b & SC2705_IBAT_CHG_MASK) * 50) + 500;
}

void sc2705_termina_vol_set(unsigned int val)
{
	u8 reg = 0;
	struct regmap *regmap = sc2705_chg->sc2705->regmap;

	/* mV to uV */
	val *= 1000;

	if (val < SC2705_CHG_B_VMIN)
		val = SC2705_CHG_B_VMIN;

	if (val > SC2705_CHG_B_VMAX)
		val = SC2705_CHG_B_VMAX;

	reg = (val - SC2705_CHG_B_VMIN) / SC2705_CHG_B_VSTEP;

	SPRD_EX_DEBUG("%s,reg:%d\n", __func__, reg);
	regmap_update_bits(regmap, SC2705_CHG_CTRL_A, SC2705_VBAT_CHG_MASK,
			(reg << SC2705_VBAT_CHG_SHIFT));

}

void sc2705_charger_cc_current_set(unsigned int val)
{
	u8 reg = 0;
	struct regmap *regmap = sc2705_chg->sc2705->regmap;

	/* mA to uA */
	val *= 1000;

	if (val < SC2705_CHG_B_IMIN)
		val = SC2705_CHG_B_IMIN;

	if (val > SC2705_CHG_B_IMAX)
		val = SC2705_CHG_B_IMAX;

	reg = (val - SC2705_CHG_B_IMIN) / SC2705_CHG_B_ISTEP;

	SPRD_EX_DEBUG("%s,reg:%d\n", __func__, reg);
	regmap_update_bits(regmap, SC2705_CHG_CTRL_B, SC2705_IBAT_CHG_MASK,
			(reg << SC2705_IBAT_CHG_SHIFT));
}

void sc2705_charger_input_current_set(unsigned int val)
{
	unsigned int iin_lim0;
	int i;
	struct regmap *regmap = sc2705_chg->sc2705->regmap;

	val *= 1000;
	for (i = 1; i < ARRAY_SIZE(iin_lim_tbl); ++i)
		if (val < iin_lim_tbl[i])
			break;
	iin_lim0 = i - 1;
	SPRD_EX_DEBUG("%s,iin_lim0:%d\n", __func__, iin_lim0);
	regmap_update_bits(regmap, SC2705_DCDC_CTRL_D,
				  SC2705_IIN_LIM0_MASK | SC2705_IIN_LIM1_MASK,
				  iin_lim0 | iin_lim0 << SC2705_IIN_LIM1_SHIFT);
}

static int sc2705_dcdc_output_set_cur(u32 val)
{
	u32 mask = SC2705_IIN_REV_LIM_MASK;
	u32 cfg = val << SC2705_IIN_REV_LIM_SHIFT;
	struct regmap *regmap = sc2705_chg->sc2705->regmap;

	return regmap_update_bits(regmap, SC2705_DCDC_CTRL_B, mask, cfg);
}

void sc2705_charger_otg_enable(int enable)
{
	struct regmap *regmap = sc2705_chg->sc2705->regmap;

	SPRD_EX_DEBUG("%s,enable:0x%x\n", __func__, enable);
	sc2705_clr_event();
	if (enable) {
		sc2705_dcdc_output_set_cur(SC2705_IIN_REV_LIM_500MA);
		regmap_update_bits(regmap, SC2705_DCDC_CTRL_A,
			SC2705_OTG_EN_MASK, (enable) ? SC2705_OTG_EN_MASK : 0);
		schedule_delayed_work(&sc2705_chg->otg_work,
			msecs_to_jiffies(100));
	} else {
		cancel_delayed_work_sync(&sc2705_chg->otg_work);
		regmap_update_bits(regmap, SC2705_DCDC_CTRL_A,
			SC2705_OTG_EN_MASK, 0);
	}
}

int sc2705_charger_status_get(void)
{
	u8 status[4];
	int ret;
	struct regmap *regmap = sc2705_chg->sc2705->regmap;

	ret = regmap_bulk_read(regmap, SC2705_STATUS_A, status,
				   ARRAY_SIZE(status));
	if (ret)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	/* If wall charger not connected, we're definitely discharging */
	if (!(status[0] & SC2705_S_ADP_DET_MASK)) {
		ret = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	SPRD_EX_DEBUG("%s,status:0x%x\n", __func__, *(unsigned int *)status);

	switch (status[3] & SC2705_S_CHG_STAT_MASK) {
	case SC2705_S_CHG_STAT_DCDC_OFF:
	case SC2705_S_CHG_STAT_FAULT_L2:
		ret = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case SC2705_S_CHG_STAT_FAULT_L1:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case SC2705_S_CHG_STAT_PRE:
	case SC2705_S_CHG_STAT_CC:
	case SC2705_S_CHG_STAT_CV:
	case SC2705_S_CHG_STAT_TOP_OFF:
		ret = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case SC2705_S_CHG_STAT_FULL:
		ret = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return ret;
}

void sc2705_charger_vindpm_set(int val)
{
	unsigned int v_val, vin_drop;
	struct regmap *regmap = sc2705_chg->sc2705->regmap;

	val *= 1000;

	/* Limit input value to min or max, or round down to correct units */
	if (val < SC2705_VIN_DROP_MIN)
		v_val = SC2705_VIN_DROP_MIN;
	else if (val > SC2705_VIN_DROP_MAX)
		v_val = SC2705_VIN_DROP_MAX;
	else
		v_val = val - (val % SC2705_VIN_DROP_STEP);

	/* Convert to register value */
	vin_drop = (v_val - SC2705_VIN_DROP_MIN) / SC2705_VIN_DROP_STEP;
	SPRD_EX_DEBUG("%s,vin_drop:%d\n", __func__, vin_drop);
	regmap_write(regmap, SC2705_VIN_CTRL_A, vin_drop);
}

void sc2705_charger_reset_timer(void)
{
	struct regmap *regmap = sc2705_chg->sc2705->regmap;

	regmap_write(regmap, SC2705_CHG_TIMER_CTRL_B,
		sc2705_chg->wd_interval);
	SPRD_EX_DEBUG("%s,sc2705_chg->wd_interval=%x\n",
		__func__, sc2705_chg->wd_interval);
}

static int sc2705_print_otg_regs(void)
{
	struct regmap *regmap = sc2705_chg->sc2705->regmap;
	u32 reg_ctrl_a, reg_event_c;
	int ret;

	ret = regmap_read(regmap, SC2705_DCDC_CTRL_A, &reg_ctrl_a);
	if (ret)
		return ret;
	ret = regmap_read(regmap, SC2705_EVENT_C, &reg_event_c);
	if (ret)
		return ret;
	SPRD_EX_DEBUG("%s, SC2705_DCDC_CTRL_A=%x, SC2705_EVENT_C=%x\n",
		__func__, reg_ctrl_a, reg_event_c);

	return 0;
}

static int sc2705_otg_func_check(void)
{
	struct regmap *regmap = sc2705_chg->sc2705->regmap;
	u32 raw_st, otg_state;
	u32 reg_ctrl_a, reg_event_c;
	int ret;

	ret = regmap_read(regmap, SC2705_DCDC_CTRL_A, &reg_ctrl_a);
	if (ret)
		return ret;
	ret = regmap_read(regmap, SC2705_EVENT_C, &reg_event_c);
	if (ret)
		return ret;

	otg_state = reg_ctrl_a & SC2705_OTG_EN_MASK;
	raw_st = reg_event_c & (SC2705_E_LOWBAT_MASK |
		SC2705_E_IIN_REV_LIM_MAX_MASK | SC2705_E_IIN_REV_LIM_MASK |
		SC2705_E_VIN_REV_SHORT_MASK);

	SPRD_EX_DEBUG("%s, otg_state=%x, raw_st=%x\n",
		__func__, otg_state, raw_st);

	if (!otg_state || raw_st)
		return -EINVAL;

	return 0;
}

static void sc2705_otg_restart(void)
{
	struct regmap *regmap = sc2705_chg->sc2705->regmap;

	regmap_update_bits(regmap, SC2705_DCDC_CTRL_A,
		SC2705_OTG_EN_MASK, 0);
	regmap_update_bits(regmap, SC2705_EVENT_C,
		(SC2705_E_LOWBAT_MASK | SC2705_E_IIN_REV_LIM_MAX_MASK
		| SC2705_E_IIN_REV_LIM_MASK | SC2705_E_VIN_REV_SHORT_MASK),
		(SC2705_E_LOWBAT_MASK | SC2705_E_IIN_REV_LIM_MAX_MASK |
		SC2705_E_IIN_REV_LIM_MASK | SC2705_E_VIN_REV_SHORT_MASK));

	sc2705_dcdc_output_set_cur(SC2705_IIN_REV_LIM_500MA);
	sc2705_print_otg_regs();
	regmap_update_bits(regmap, SC2705_DCDC_CTRL_A,
		SC2705_OTG_EN_MASK, SC2705_OTG_EN_MASK);
}

static void sc2705_otg_work(struct work_struct *work)
{
	int ret = 0;
	int retry_cnt = 0;

	if (sc2705_otg_func_check()) {
		do {
			sc2705_otg_restart();
			/*
			 * sc2705 the interval between two times OTG
			 * enable must be 100ms, otherwise OTG restart
			 * will fail, undetected anomaly.
			 */
			msleep(100);
			ret = sc2705_otg_func_check();
		} while (ret && (retry_cnt++ < SC2705_OTG_RETRY_TIMES));
	}

	if (!ret) {
		SPRD_EX_DEBUG("%s,func ok\n", __func__);
		sc2705_dcdc_output_set_cur(SC2705_IIN_REV_LIM_2000MA);
		schedule_delayed_work(&sc2705_chg->otg_work,
			msecs_to_jiffies(1500));
	} else {
		sc2705_print_otg_regs();
		SPRD_EX_DEBUG("%s, is stop, retry_cnt=%d\n",
			__func__, retry_cnt);
	}
}

static int sc2705_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc2705 *sc2705 = dev_get_drvdata(dev->parent);
	struct sc2705_charger_pdata *charger_pdata = dev_get_platdata(dev);
	struct sc2705_charger *charger;
	struct regmap *regmap = sc2705->regmap;
	unsigned int status_a;
	int ext_id_state = 0, ret;

	SPRD_EX_DEBUG("sc2705 charge probe start!\n");
	charger = devm_kzalloc(dev, sizeof(struct sc2705_charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	platform_set_drvdata(pdev, charger);
	charger->sc2705 = sc2705;
	charger->dev = dev;

	/* Handle DT data if provided */
	if (dev->of_node) {
		charger_pdata = sc2705_charger_of_to_pdata(dev);
		if (IS_ERR(charger_pdata))
			return PTR_ERR(charger_pdata);

		dev->platform_data = charger_pdata;
	}

	sc2705_charger_handle_pdata(charger);
	sc2705_chg = charger;

	/* Enable automatic input power reduction (for low VIN voltage) */
	ret = regmap_update_bits(regmap, SC2705_DCDC_CTRL_B,
				 SC2705_IN_PWR_RED_MASK,
				 SC2705_IN_PWR_RED_MASK);
	if (ret)
		return ret;

	/* Enable automatic temperature regulation/profiling */
	ret = regmap_update_bits(regmap, SC2705_TJUNC_CTRL_A,
				 SC2705_TJUNC_REG_MASK, SC2705_TJUNC_REG_MASK);
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, SC2705_TBAT_CTRL_A,
			 SC2705_TBAT_AUTO_MASK | SC2705_TBAT_L2_EN_MASK |
			 SC2705_TBAT_EXT_MASK,
			 SC2705_TBAT_AUTO_MASK | SC2705_TBAT_L2_EN_MASK |
			 SC2705_TBAT_EXT_MASK);
	if (ret)
		return ret;

	/* Enable charging/boost, as required */
	ret = regmap_read(regmap, SC2705_STATUS_A, &status_a);
	if (ret)
		return ret;

	if (status_a & SC2705_S_ADP_DET_MASK) {
		ret = regmap_update_bits(regmap, SC2705_DCDC_CTRL_A,
					 SC2705_CHG_EN_MASK |
					 SC2705_OTG_EN_MASK,
					 SC2705_CHG_EN_MASK);
		if (ret)
			return ret;
	} else if (ext_id_state > 0) {
		ret = regmap_update_bits(regmap, SC2705_DCDC_CTRL_A,
					 SC2705_CHG_EN_MASK |
					 SC2705_OTG_EN_MASK,
					 SC2705_OTG_EN_MASK);
		if (ret)
			return ret;
	}

	INIT_DELAYED_WORK(&sc2705_chg->otg_work, sc2705_otg_work);
	sprdbat_register_ext_ops(sprd_get_2705_ops());
	SPRD_EX_DEBUG("sc2705 charge probe end!\n");
	return 0;
}

static int sc2705_charger_remove(struct platform_device *pdev)
{
	struct sc2705_charger *charger = platform_get_drvdata(pdev);
	struct regmap *regmap = charger->sc2705->regmap;
	int ret;

	/* Disable charging/boost */
	ret = regmap_update_bits(regmap, SC2705_DCDC_CTRL_A,
				 SC2705_CHG_EN_MASK |
				 SC2705_OTG_EN_MASK, 0);
	if (ret) {
		dev_err(charger->dev, "Failed to disable charging/boost: %d\n",
			ret);
		return ret;
	}

	return 0;
}

static void sc2705_shutdown(struct platform_device *pdev)
{
	sc2705_termina_vol_set(DEFAULT_CCCV_VOL);
	SPRD_EX_DEBUG("sc2705 shutdown\n");
}

static struct platform_driver sc2705_charger_driver = {
	.driver = {
		.name = "sc2705-charger",
		.of_match_table = of_match_ptr(sc2705_charger_of_match),
	},
	.probe = sc2705_charger_probe,
	.shutdown = sc2705_shutdown,
	.remove = sc2705_charger_remove,
};

static int __init sc2705_charger_init(void)
{
	return platform_driver_register(&sc2705_charger_driver);
}

subsys_initcall_sync(sc2705_charger_init);

static void __exit sc2705_charger_exit(void)
{
	platform_driver_unregister(&sc2705_charger_driver);
}
module_exit(sc2705_charger_exit);

MODULE_DESCRIPTION("Charger Driver for SC2705");
MODULE_AUTHOR("Mingwei Zhang <mingwei.zhang@spreadtrum.com>");
MODULE_LICENSE("GPL");
