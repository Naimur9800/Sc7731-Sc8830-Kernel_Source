/* SPDX-License-Identifier: GPL-2.0
 *
 * Charger device driver for SC2703
 *
 * Copyright (c) 2018 Dialog Semiconductor.
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
#include <linux/mfd/sc2703/registers.h>
#include <linux/delay.h>
#include "sprd_charge_helper.h"
#include "sc2703-charger.h"

#undef pr_fmt
#define pr_fmt(fmt)	"sc2703_chg: " fmt

/* The range of cccv value is 3.80v-4.80v, the default vol is 3.80v */
#define DEFAULT_CCCV_VOL 3800

/* Private data */
struct sc2703_extcon_cable {
	struct extcon_dev *edev;
	struct notifier_block nb;
};

struct sc2703_charger {
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
	struct sc2703_extcon_cable ext_id;
	struct delayed_work otg_work;
	struct regmap *regmap;
};

#define SC2703_CHG_B_IMIN		500000
#define SC2703_CHG_B_ISTEP		50000
#define SC2703_CHG_B_IMAX		3500000
#define SC2703_CHG_B_VMIN		3800000
#define SC2703_CHG_B_VSTEP		20000
#define SC2703_CHG_B_VMAX		4800000
#define SC2703_VIN_DROP_MIN		4300000
#define SC2703_VIN_DROP_STEP		50000
#define SC2703_VIN_DROP_MAX		12000000
#define SC2703_CHGV_MAX			13500000
#define SC2703_CC_CURR_MIN		500000
#define SC2703_CC_CURR_MAX		3500000
#define SC2703_EVENT_CLR_MASK		0xff
#define SC2703_OTG_RETRY_TIMES		10
#define SC2703_SPSP_TUNE_ERR_MIN		0x08
#define SC2703_SPSP_TUNE_ERR_MAX		0x10

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

static struct sc2703_charger *sc2703_chg;

static const struct of_device_id sc2703_charger_of_match[] = {
	{ .compatible = "sprd,sc2703-charger", },
	{ }
};
MODULE_DEVICE_TABLE(of, sc2703_charger_of_match);

static u8 sc2703_charger_of_prop_range(struct device *dev, u32 val, u32 min,
				       u32 max, u32 step, u32 default_val,
				       const char *name)
{
	if (val < min || val > max || val % step) {
		dev_warn(dev, "Invalid %s value\n", name);
		return default_val;
	} else
		return (val - min) / step;
}

static enum sc2703_ibat_pre
	sc2703_charger_of_ibat_pre(struct device *dev, u32 val)
{
	switch (val) {
	case 50000:
		return SC2703_IBAT_PRE_50MA;
	case 100000:
		return SC2703_IBAT_PRE_100MA;
	case 200000:
		return SC2703_IBAT_PRE_200MA;
	case 400000:
		return SC2703_IBAT_PRE_400MA;
	default:
		dev_warn(dev, "Invalid ibat-pre value\n");
		return SC2703_IBAT_PRE_DEFAULT;
	}
}

static enum sc2703_t_eoc sc2703_charger_of_t_eoc(struct device *dev, u32 val)
{
	switch (val) {
	case 0:
		return SC2703_T_EOC_0MIN;
	case 60:
		return SC2703_T_EOC_1MIN;
	case 300:
		return SC2703_T_EOC_5MIN;
	case 600:
		return SC2703_T_EOC_10MIN;
	case 1200:
		return SC2703_T_EOC_20MIN;
	case 1800:
		return SC2703_T_EOC_30MIN;
	case 2700:
		return SC2703_T_EOC_45MIN;
	case 3600:
		return SC2703_T_EOC_60MIN;
	default:
		dev_warn(dev, "Invalid t-eoc value\n");
		return SC2703_T_EOC_DEFAULT;
	}
}

static enum sc2703_vbat_rechg
	sc2703_charger_of_vbat_rechg(struct device *dev, u32 val)
{
	switch (val) {
	case 100000:
		return SC2703_VBAT_RECHG_100MA;
	case 160000:
		return SC2703_VBAT_RECHG_160MA;
	case 200000:
		return SC2703_VBAT_RECHG_200MA;
	case 240000:
		return SC2703_VBAT_RECHG_240MA;
	default:
		dev_warn(dev, "Invalid vbat-rechg value\n");
		return SC2703_VBAT_RECHG_DEFAULT;
	}
}

static enum sc2703_timeout_cccv
	sc2703_charger_of_timeout_cccv(struct device *dev, u32 val)
{
	switch (val) {
	case 7200:
		return SC2703_TIMEOUT_CCCV_2HOURS;
	case 14400:
		return SC2703_TIMEOUT_CCCV_4HOURS;
	case 21600:
		return SC2703_TIMEOUT_CCCV_6HOURS;
	case 28800:
		return SC2703_TIMEOUT_CCCV_8HOURS;
	case 36000:
		return SC2703_TIMEOUT_CCCV_10HOURS;
	case 43200:
		return SC2703_TIMEOUT_CCCV_12HOURS;
	case 50400:
		return SC2703_TIMEOUT_CCCV_14HOURS;
	case 64800:
		return SC2703_TIMEOUT_CCCV_18HOURS;
	default:
		dev_warn(dev, "Invalid timeout-cccv value\n");
		return SC2703_TIMEOUT_CCCV_DEFAULT;
	}
}

static enum sc2703_iin_rev_lim
	sc2703_charger_of_iin_rev_lim(struct device *dev, u32 val)
{
	switch (val) {
	case 500000:
		return SC2703_IIN_REV_LIM_500MA;
	case 900000:
		return SC2703_IIN_REV_LIM_900MA;
	case 1200000:
		return SC2703_IIN_REV_LIM_1200MA;
	case 1500000:
		return SC2703_IIN_REV_LIM_1500MA;
	case 2000000:
		return SC2703_IIN_REV_LIM_2000MA;
	default:
		dev_warn(dev, "Invalid iin-rev-lim value\n");
		return SC2703_IIN_REV_LIM_DEFAULT;
	}
}

static enum sc2703_bat_det_src
	sc2703_charger_of_bat_det_src(struct device *dev, const char *str)
{
	if (!strcmp(str, "Off"))
		return SC2703_BAT_DET_SRC_OFF;
	else if (!strcmp(str, "VBAT"))
		return SC2703_BAT_DET_SRC_VBAT;
	else if (!strcmp(str, "BATID"))
		return SC2703_BAT_DET_SRC_BATID;
	else if (!strcmp(str, "TBAT"))
		return SC2703_BAT_DET_SRC_TBAT;

	dev_warn(dev, "Invalid bat-det-src value\n");
	return SC2703_BAT_DET_SRC_DEFAULT;
}

static struct sc2703_charger_pdata
	*sc2703_charger_of_to_pdata(struct device *dev)
{
	struct device_node *charger_np = dev->of_node;
	struct sc2703_charger_pdata *charger_pdata;
	u32 of_val32;
	const char *of_str;
	struct extcon_dev *edev;

	charger_pdata = devm_kzalloc(dev, sizeof(*charger_pdata), GFP_KERNEL);
	if (!charger_pdata)
		return NULL;

	if (of_property_read_u32(charger_np, "sprd,ibat-pre-microamp",
				 &of_val32) >= 0)
		charger_pdata->ibat_pre =
			sc2703_charger_of_ibat_pre(dev, of_val32);
	else
		charger_pdata->ibat_pre = SC2703_IBAT_PRE_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,vbat-chg-microvolt",
				 &of_val32) >= 0)
		charger_pdata->vbat_chg =
			sc2703_charger_of_prop_range(dev, of_val32,
				SC2703_CHG_B_VMIN, SC2703_CHG_B_VMAX,
				SC2703_CHG_B_VSTEP,
				SC2703_VBAT_CHG_DEFAULT, "vbat-chg");
	else
		charger_pdata->vbat_chg = SC2703_VBAT_CHG_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,ibat-chg-microamp",
				 &of_val32) >= 0)
		charger_pdata->ibat_chg =
			sc2703_charger_of_prop_range(dev, of_val32,
				SC2703_CHG_B_IMIN, SC2703_CHG_B_IMAX,
				SC2703_CHG_B_ISTEP,
				SC2703_IBAT_CHG_DEFAULT, "ibat-chg");
	else
		charger_pdata->ibat_chg = SC2703_IBAT_CHG_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,ibat-term-microamp",
				 &of_val32) >= 0)
		charger_pdata->ibat_term =
			sc2703_charger_of_prop_range(dev, of_val32, 100000,
						     450000, 50000,
						     SC2703_IBAT_TERM_DEFAULT,
						     "ibat-term");
	else
		charger_pdata->ibat_term = SC2703_IBAT_TERM_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,t-eoc-sec", &of_val32) >= 0)
		charger_pdata->t_eoc = sc2703_charger_of_t_eoc(dev, of_val32);
	else
		charger_pdata->t_eoc = SC2703_T_EOC_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,vbat-rechg-microvolt",
				 &of_val32) >= 0)
		charger_pdata->vbat_rechg =
			sc2703_charger_of_vbat_rechg(dev, of_val32);
	else
		charger_pdata->vbat_rechg = SC2703_VBAT_RECHG_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,dcdc-peak-ilim-microamp",
				 &of_val32) >= 0)
		charger_pdata->dcdc_peak_ilim =
			sc2703_charger_of_prop_range(dev, of_val32, 6000000,
					     9000000, 1000000,
					     SC2703_DCDC_PEAK_ILIM_DEFAULT,
					     "dcdc-peak-ilim");
	else
		charger_pdata->dcdc_peak_ilim = SC2703_DCDC_PEAK_ILIM_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,dcdc-rev-peak-ilim-microamp",
				 &of_val32) >= 0)
		charger_pdata->dcdc_rev_peak_ilim =
			sc2703_charger_of_prop_range(dev, of_val32, 5000000,
					     8000000, 1000000,
					     SC2703_DCDC_REV_PEAK_ILIM_DEFAULT,
					     "dcdc-rev-peak-ilim");
	else
		charger_pdata->dcdc_rev_peak_ilim =
			SC2703_DCDC_REV_PEAK_ILIM_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,vbat-ov-microvolt",
				 &of_val32) >= 0)
		charger_pdata->vbat_ov =
			sc2703_charger_of_prop_range(dev, of_val32, 4000000,
						     5500000, 100000,
						     SC2703_VBAT_OV_DEFAULT,
						     "vbat-ov");
	else
		charger_pdata->vbat_ov = SC2703_VBAT_OV_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,vbat-uv-microvolt",
				 &of_val32) >= 0)
		charger_pdata->vbat_uv =
			sc2703_charger_of_prop_range(dev, of_val32, 2200000,
						     2950000, 50000,
						     SC2703_VBAT_UV_DEFAULT,
						     "vbat-uv");
	else
		charger_pdata->vbat_uv = SC2703_VBAT_UV_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,vsys-min-microvolt",
				 &of_val32) >= 0)
		charger_pdata->vsys_min =
			sc2703_charger_of_prop_range(dev, of_val32, 3000000,
						     3900000, 60000,
						     SC2703_VSYS_MIN_DEFAULT,
						     "vsys-min");
	else
		charger_pdata->vsys_min = SC2703_VSYS_MIN_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,timeout-pre-sec",
				 &of_val32) >= 0)
		charger_pdata->timeout_pre =
			sc2703_charger_of_prop_range(dev, of_val32, 900, 3600,
						     900,
						     SC2703_TIMEOUT_PRE_DEFAULT,
						     "timeout-pre");
	else
		charger_pdata->timeout_pre = SC2703_TIMEOUT_PRE_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,timeout-cccv-sec",
				 &of_val32) >= 0)
		charger_pdata->timeout_cccv =
			sc2703_charger_of_timeout_cccv(dev, of_val32);
	else
		charger_pdata->timeout_pre = SC2703_TIMEOUT_CCCV_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,timer-load-sec",
				 &of_val32) >= 0)
		charger_pdata->timer_load =
			sc2703_charger_of_prop_range(dev, of_val32, 0, 255, 1,
						     0, "timer-load");

	if (of_property_read_u32(charger_np, "sprd,iin-rev-lim-microamp",
				&of_val32) >= 0)
		charger_pdata->iin_rev_lim =
			sc2703_charger_of_iin_rev_lim(dev, of_val32);
	else
		charger_pdata->timeout_pre = SC2703_IIN_REV_LIM_DEFAULT;

	if (!of_property_read_string(charger_np, "sprd,bat-det-src", &of_str))
		charger_pdata->bat_det_src =
			sc2703_charger_of_bat_det_src(dev, of_str);
	else
		charger_pdata->bat_det_src = SC2703_BAT_DET_SRC_DEFAULT;

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

	if (of_property_read_u32(charger_np, "sprd,tbat-t3-celsius",
				 &of_val32) >= 0)
		charger_pdata->tbat_t3 =
			sc2703_charger_of_prop_range(dev, of_val32, 40, 50, 5,
						     SC2703_TBAT_T3_DEFAULT,
						     "tbat-t3");
	else
		charger_pdata->tbat_t3 = SC2703_TBAT_T3_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,tbat-t4-celsius",
				 &of_val32) >= 0)
		charger_pdata->tbat_t4 =
			sc2703_charger_of_prop_range(dev, of_val32, 55, 65, 5,
						     SC2703_TBAT_T4_DEFAULT,
						     "tbat-t4");
	else
		charger_pdata->tbat_t4 = SC2703_TBAT_T4_DEFAULT;

	if (of_property_read_u32(charger_np, "sprd,vbat-cold-microvolt",
				 &of_val32) >= 0)
		charger_pdata->vbat_cold =
			sc2703_charger_of_prop_range(dev, of_val32, 0, 200000,
						     100000,
						     SC2703_VBAT_COLD_DEFAULT,
						     "vbat-cold");
	else
		charger_pdata->vbat_cold = SC2703_VBAT_COLD_DEFAULT;

	charger_pdata->ibat_cold_enable =
		of_property_read_bool(charger_np, "sprd,ibat-cold-enable");

	if (of_property_read_u32(charger_np, "sprd,vbat-warm-microvolt",
				 &of_val32) >= 0)
		charger_pdata->vbat_warm =
			sc2703_charger_of_prop_range(dev, of_val32, 0, 200000,
						     100000,
						     SC2703_VBAT_WARM_DEFAULT,
						     "vbat-warm");
	else
		charger_pdata->vbat_warm = SC2703_VBAT_WARM_DEFAULT;

	charger_pdata->ibat_warm_enable =
		of_property_read_bool(charger_np, "sprd,ibat-warm-enable");

	charger_pdata->onkey_det_enable =
		of_property_read_bool(charger_np, "sprd,onkey-detect-enable");

	charger_pdata->spsp_en =
		of_property_read_bool(charger_np, "sprd,spsp-enable");

	charger_pdata->spsp_tune_en =
		of_property_read_bool(charger_np, "sprd,spsp-tune-enable");

	if (of_property_read_u32(charger_np, "sprd,spsp-tune",
				 &of_val32) >= 0)
		if ((of_val32 > SC2703_SPSP_TUNE_ERR_MIN &&
			 of_val32 < SC2703_SPSP_TUNE_ERR_MAX) ||
			 (of_val32 > SC2703_SPSP_TUNE_MAX))
			charger_pdata->spsp_tune = SC2703_SPSP_TUNE_DEFAULT;
		else
			charger_pdata->spsp_tune = of_val32;
	else
		charger_pdata->spsp_tune = SC2703_SPSP_TUNE_DEFAULT;

	return charger_pdata;
}

static void sc2703_charger_handle_pdata(struct sc2703_charger *charger)
{
	struct device *dev = charger->dev;
	struct sc2703_charger_pdata *charger_pdata =
		dev_get_platdata(charger->dev);
	int ret;
	u32 cfg = 0, mask = 0;

	if (!charger_pdata)
		return;

	switch (charger_pdata->ibat_pre) {
	case SC2703_IBAT_PRE_50MA:
	case SC2703_IBAT_PRE_100MA:
	case SC2703_IBAT_PRE_200MA:
	case SC2703_IBAT_PRE_400MA:
		cfg |= (charger_pdata->ibat_pre <<
			SC2703_IBAT_PRE_SHIFT);
		mask |= SC2703_IBAT_PRE_MASK;
	}

	if (charger_pdata->ibat_term <= SC2703_IBAT_TERM_MAX) {
		cfg |= (charger_pdata->ibat_term <<
			SC2703_IBAT_TERM_SHIFT);
		mask |= SC2703_IBAT_TERM_MASK;
	}

	switch (charger_pdata->t_eoc) {
	case SC2703_T_EOC_0MIN:
	case SC2703_T_EOC_1MIN:
	case SC2703_T_EOC_5MIN:
	case SC2703_T_EOC_10MIN:
	case SC2703_T_EOC_20MIN:
	case SC2703_T_EOC_30MIN:
	case SC2703_T_EOC_45MIN:
	case SC2703_T_EOC_60MIN:
		cfg |= (charger_pdata->t_eoc << SC2703_T_EOC_SHIFT);
		mask |= SC2703_T_EOC_MASK;
	}

	ret = regmap_update_bits(charger->regmap,
		SC2703_CHG_CTRL_D, mask, cfg);
	if (ret)
		dev_warn(dev, "Failed to write pdata (CHG_CTRL_D): %d\n", ret);

	cfg = 0;
	mask = 0;
	if (charger_pdata->vbat_chg <= SC2703_VBAT_CHG_MAX) {
		cfg |= (charger_pdata->vbat_chg <<
			SC2703_VBAT_CHG_SHIFT);
		mask |= SC2703_VBAT_CHG_MASK;
	}

	switch (charger_pdata->vbat_rechg) {
	case SC2703_VBAT_RECHG_100MA:
	case SC2703_VBAT_RECHG_160MA:
	case SC2703_VBAT_RECHG_200MA:
	case SC2703_VBAT_RECHG_240MA:
		cfg |= (charger_pdata->vbat_rechg <<
			SC2703_VBAT_RECHG_SHIFT);
		mask |= SC2703_VBAT_RECHG_MASK;
	}

	ret = regmap_update_bits(charger->regmap,
		SC2703_CHG_CTRL_A, mask, cfg);
	if (ret)
		dev_warn(dev, "Failed to write pdata (CHG_CTRL_A): %d\n", ret);

	cfg = 0;
	mask = 0;
	if (charger_pdata->ibat_chg <= SC2703_IBAT_CHG_MAX) {
		cfg |= (charger_pdata->ibat_chg <<
			SC2703_IBAT_CHG_SHIFT);
		mask |= SC2703_IBAT_CHG_MASK;
	}

	ret = regmap_update_bits(charger->regmap,
		SC2703_CHG_CTRL_B, mask, cfg);
	if (ret)
		dev_warn(dev, "Failed to write pdata (CHG_CTRL_B): %d\n", ret);

	cfg = 0;
	mask = 0;
	if (charger_pdata->dcdc_peak_ilim <=
				SC2703_DCDC_PEAK_ILIM_MAX) {
		cfg |= (charger_pdata->dcdc_peak_ilim <<
			SC2703_DCDC_PEAK_ILIM_SHIFT);
		mask |= SC2703_DCDC_PEAK_ILIM_MASK;
	}

	if (charger_pdata->dcdc_rev_peak_ilim <=
				SC2703_DCDC_REV_PEAK_ILIM_MAX) {
		cfg |= (charger_pdata->dcdc_rev_peak_ilim <<
			SC2703_DCDC_REV_PEAK_ILIM_SHIFT);
		mask |= SC2703_DCDC_REV_PEAK_ILIM_MASK;
	}

	switch (charger_pdata->iin_rev_lim) {
	case SC2703_IIN_REV_LIM_500MA:
	case SC2703_IIN_REV_LIM_900MA:
	case SC2703_IIN_REV_LIM_1200MA:
	case SC2703_IIN_REV_LIM_1500MA:
	case SC2703_IIN_REV_LIM_2000MA:
		cfg |= (charger_pdata->iin_rev_lim <<
			SC2703_IIN_REV_LIM_SHIFT);
		mask |= SC2703_IIN_REV_LIM_MASK;
	}

	ret = regmap_update_bits(charger->regmap,
		SC2703_DCDC_CTRL_B, mask, cfg);
	if (ret)
		dev_warn(dev, "Failed to write pdata (DCDC_CTRL_B): %d\n", ret);

	cfg = 0;
	mask = 0;
	if (charger_pdata->vbat_uv <= SC2703_VBAT_UV_MAX) {
		cfg |= (charger_pdata->vbat_uv <<
			SC2703_VBAT_UV_SHIFT);
		mask |= SC2703_VBAT_UV_MASK;
	}

	if (charger_pdata->vbat_ov <= SC2703_VBAT_OV_MAX) {
		cfg |= (charger_pdata->vbat_ov <<
			SC2703_VBAT_OV_SHIFT);
		mask |= SC2703_VBAT_OV_MASK;
	}

	ret = regmap_update_bits(charger->regmap,
		SC2703_VBAT_CTRL_A, mask, cfg);
	if (ret)
		dev_warn(dev, "Failed to write pdata (VBAT_CTRL_A): %d\n", ret);

	cfg = 0;
	mask = 0;
	if (charger_pdata->vsys_min <= SC2703_VSYS_MIN_MAX) {
		cfg |= (charger_pdata->vsys_min <<
			SC2703_VSYS_MIN_SHIFT);
		mask |= SC2703_VSYS_MIN_MASK;
	}

	ret = regmap_update_bits(charger->regmap,
		SC2703_CHG_CTRL_C, mask, cfg);
	if (ret)
		dev_warn(dev, "Failed to write pdata (CHG_CTRL_C): %d\n", ret);

	cfg = 0;
	mask = 0;
	switch (charger_pdata->timeout_cccv) {
	case SC2703_TIMEOUT_CCCV_2HOURS:
	case SC2703_TIMEOUT_CCCV_4HOURS:
	case SC2703_TIMEOUT_CCCV_6HOURS:
	case SC2703_TIMEOUT_CCCV_8HOURS:
	case SC2703_TIMEOUT_CCCV_10HOURS:
	case SC2703_TIMEOUT_CCCV_12HOURS:
	case SC2703_TIMEOUT_CCCV_14HOURS:
	case SC2703_TIMEOUT_CCCV_18HOURS:
		cfg |= (charger_pdata->timeout_cccv <<
			SC2703_TIMEOUT_CCCV_SHIFT);
		mask |= SC2703_TIMEOUT_CCCV_MASK;
	}

	if (charger_pdata->timeout_pre <= SC2703_TIMEOUT_PRE_MAX) {
		cfg |= (charger_pdata->timeout_pre <<
			SC2703_TIMEOUT_PRE_SHIFT);
		mask |= SC2703_TIMEOUT_PRE_MASK;
	}

	ret = regmap_update_bits(charger->regmap,
		SC2703_CHG_TIMER_CTRL_A, mask,
				 cfg);
	if (ret)
		dev_warn(dev,
			"Failed to write pdata (CHG_TIMER_CTRL_A): %d\n", ret);

	charger->wd_interval = charger_pdata->timer_load;
	charger->wd_kick_interval = (charger->wd_interval * 1000) / 4;
	ret = regmap_update_bits(charger->regmap, SC2703_CONF_A,
				 SC2703_WD_EN_MASK,
				 (charger->wd_interval) ?
				 SC2703_WD_EN_MASK : 0);
	if (ret)
		dev_warn(dev, "Failed to set watchdog: %d\n", ret);

	cfg = 0;
	mask = 0;
	switch (charger_pdata->bat_det_src) {
	case SC2703_BAT_DET_SRC_OFF:
	case SC2703_BAT_DET_SRC_VBAT:
	case SC2703_BAT_DET_SRC_BATID:
	case SC2703_BAT_DET_SRC_TBAT:
		cfg |= (charger_pdata->bat_det_src <<
			SC2703_BAT_DET_SRC_SHIFT);
		mask |= SC2703_BAT_DET_SRC_MASK;
	}

	if (charger_pdata->onkey_det_enable) {
		cfg |= 1 << SC2703_ONKEY_DET_EN_SHIFT;
		mask |= SC2703_ONKEY_DET_EN_MASK;
	}

	ret = regmap_update_bits(charger->regmap, SC2703_CONF_A, mask, cfg);
	if (ret)
		dev_warn(dev, "Failed to write pdata (CONF_A): %d\n",
			ret);

	charger->ext_id.edev = charger_pdata->ext_id;

	cfg = 0;
	mask = 0;
	if (charger_pdata->tbat_t3 <= SC2703_TBAT_T3_MAX) {
		cfg |= (charger_pdata->tbat_t3 << SC2703_TBAT_T3_SHIFT);
		mask |= SC2703_TBAT_T3_MASK;
	}

	if (charger_pdata->tbat_t4 <= SC2703_TBAT_T4_MAX) {
		cfg |= (charger_pdata->tbat_t4 << SC2703_TBAT_T4_SHIFT);
		mask |= SC2703_TBAT_T4_MASK;
	}

	ret = regmap_update_bits(charger->regmap,
		SC2703_TBAT_CTRL_A, mask, cfg);
	if (ret)
		dev_warn(dev, "Failed to write pdata (TBAT_CTRL_A): %d\n", ret);

	cfg = 0;
	mask = 0;
	if (charger_pdata->ibat_cold_enable) {
		cfg |= SC2703_IBAT_COLD_MASK;
		mask |= SC2703_IBAT_COLD_MASK;
	}

	if (charger_pdata->ibat_warm_enable) {
		cfg |= SC2703_IBAT_WARM_MASK;
		mask |= SC2703_IBAT_WARM_MASK;
	}

	if (charger_pdata->vbat_cold <= SC2703_VBAT_COLD_MAX) {
		cfg |= (charger_pdata->vbat_cold <<
			SC2703_VBAT_COLD_SHIFT);
		mask |= SC2703_VBAT_COLD_MASK;
	}

	if (charger_pdata->vbat_warm <= SC2703_VBAT_WARM_MAX) {
		cfg |= (charger_pdata->vbat_warm <<
			SC2703_VBAT_WARM_SHIFT);
		mask |= SC2703_VBAT_WARM_MASK;
	}

	ret = regmap_update_bits(charger->regmap,
		SC2703_CHG_CTRL_E, mask, cfg);
	if (ret)
		dev_warn(dev, "Failed to write pdata (CHG_CTRL_E): %d\n", ret);

	cfg = 0;
	mask = 0;
	if (charger_pdata->spsp_en) {
		cfg |= SC2703_SPSP_ENABLE_MASK;
		mask |= SC2703_SPSP_ENABLE_MASK;
	}

	if (charger_pdata->spsp_tune_en) {
		cfg |= SC2703_SPSP_TUNE_ENABLE_MASK;
		mask |= SC2703_SPSP_TUNE_ENABLE_MASK;
	}

	if (charger_pdata->spsp_tune <= SC2703_SPSP_TUNE_MAX) {
		cfg |= charger_pdata->spsp_tune
			<< SC2703_SPSP_TUNE_SHIFT;
		mask |= SC2703_SPSP_TUNE_MASK;
	}

	ret = regmap_update_bits(charger->regmap, SC2703_CONF_B, mask, cfg);
	if (ret)
		dev_warn(dev, "Failed to write pdata (SC2703_CONF_B): %d\n",
		ret);
}

void sc2703_charger_ic_init(void)
{
	/* Do nothing, init chip in probe function by dts config */
}

static void sc2703_clr_event(void)
{
	regmap_write(sc2703_chg->regmap, SC2703_EVENT_A, SC2703_EVENT_CLR_MASK);
	regmap_write(sc2703_chg->regmap, SC2703_EVENT_B, SC2703_EVENT_CLR_MASK);
	regmap_write(sc2703_chg->regmap, SC2703_EVENT_C,
		     SC2703_EVENT_CLR_MASK & (~SC2703_E_ADC_DONE_MASK));
	regmap_write(sc2703_chg->regmap, SC2703_EVENT_D, SC2703_EVENT_CLR_MASK);
}

void sc2703_charger_start(void)
{
	sc2703_clr_event();
	regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_A,
			   SC2703_CHG_EN_MASK, SC2703_CHG_EN_MASK);
	regmap_update_bits(sc2703_chg->regmap, SC2703_REG_UNLOCK,
			   SC2703_CHG_REG_MASK_ALL, SC2703_REG_UNLOCK_VAL);
	regmap_update_bits(sc2703_chg->regmap, SC2703_VSYS_UV_SWITCH,
			   SC2703_CHG_REG_MASK_ALL, SC2703_VSYS_UV_SWITCH_VAL);
	regmap_update_bits(sc2703_chg->regmap, SC2703_CHG_BUCK_SWITCH,
			   SC2703_CHG_REG_MASK_ALL, SC2703_CHG_REG_DISABLE_VAL);
	regmap_update_bits(sc2703_chg->regmap, SC2703_REG_UNLOCK,
			   SC2703_CHG_REG_MASK_ALL, SC2703_CHG_REG_DISABLE_VAL);
}

void sc2703_charger_stop(unsigned int flag)
{
	if (flag & SPRDBAT_CHG_END_UNSPEC) {
		regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_A,
				   SC2703_CHG_EN_MASK, 0);
		regmap_update_bits(sc2703_chg->regmap, SC2703_REG_UNLOCK,
				   SC2703_CHG_REG_MASK_ALL,
				   SC2703_REG_UNLOCK_VAL);
		regmap_update_bits(sc2703_chg->regmap, SC2703_VSYS_UV_SWITCH,
				   SC2703_CHG_REG_MASK_ALL,
				   SC2703_VSYS_UV_SWITCH_VAL);
		regmap_update_bits(sc2703_chg->regmap, SC2703_CHG_BUCK_SWITCH,
				   SC2703_CHG_REG_MASK_ALL,
				   SC2703_CHG_BUCK_SWITCH_VAL);
		regmap_update_bits(sc2703_chg->regmap, SC2703_REG_UNLOCK,
				   SC2703_CHG_REG_MASK_ALL,
				   SC2703_CHG_REG_DISABLE_VAL);
	} else if (flag & SPRDBAT_CHG_END_CHGR_OTP_BIT) {
		regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_A,
				   SC2703_CHG_EN_MASK, 0);
	} else {
		regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_A,
				   SC2703_CHG_EN_MASK, 0);
		regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_A,
				   SC2703_DCDC_EN_MASK, 0);
	}
}

void sc2703_termina_vol_set(unsigned int val)
{
	u32 reg;

	/* mV to uV */
	val *= 1000;

	if (val < SC2703_CHG_B_VMIN)
		val = SC2703_CHG_B_VMIN;
	else if (val > SC2703_CHG_B_VMAX)
		val = SC2703_CHG_B_VMAX;

	reg = (val - SC2703_CHG_B_VMIN) / SC2703_CHG_B_VSTEP;
	regmap_update_bits(sc2703_chg->regmap, SC2703_CHG_CTRL_A,
		SC2703_VBAT_CHG_MASK, reg << SC2703_VBAT_CHG_SHIFT);
}

void sc2703_charger_vindpm_set(unsigned int val)
{
	unsigned int v_val, vin_drop;

	val *= 1000;

	/* Limit input value to min or max, or round down to correct units */
	if (val < SC2703_VIN_DROP_MIN)
		v_val = SC2703_VIN_DROP_MIN;
	else if (val > SC2703_VIN_DROP_MAX)
		v_val = SC2703_VIN_DROP_MAX;
	else
		v_val = val - (val % SC2703_VIN_DROP_STEP);

	/* Convert to register value */
	vin_drop = (v_val - SC2703_VIN_DROP_MIN) / SC2703_VIN_DROP_STEP;
	regmap_write(sc2703_chg->regmap, SC2703_VIN_CTRL_A, vin_drop);
}

void sc2703_charger_cc_current_set(unsigned int val)
{
	u32 reg;

	/* mA to uA */
	val *= 1000;

	if (val < SC2703_CHG_B_IMIN)
		val = SC2703_CHG_B_IMIN;
	else if (val > SC2703_CHG_B_IMAX)
		val = SC2703_CHG_B_IMAX;

	reg = (val - SC2703_CHG_B_IMIN) / SC2703_CHG_B_ISTEP;
	regmap_update_bits(sc2703_chg->regmap, SC2703_CHG_CTRL_B,
		SC2703_IBAT_CHG_MASK, reg << SC2703_IBAT_CHG_SHIFT);
}

void sc2703_charger_input_current_set(unsigned int val)
{
	unsigned int iin_lim0;
	int i;

	val *= 1000;
	for (i = 1; i < ARRAY_SIZE(iin_lim_tbl); ++i)
		if (val < iin_lim_tbl[i])
			break;
	iin_lim0 = i - 1;
	regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_D,
			   SC2703_IIN_LIM0_MASK | SC2703_IIN_LIM1_MASK,
			   iin_lim0 | iin_lim0 << SC2703_IIN_LIM1_SHIFT);
}

unsigned int sc2703_charger_cc_current_get(void)
{
	unsigned int chg_ctrl_b;
	int ret;

	ret = regmap_read(sc2703_chg->regmap, SC2703_CHG_CTRL_B, &chg_ctrl_b);
	if (ret)
		return ret;

	/* The value of the SC2703_CHG_CTRL_B register is converted
	 * to the charge current value.
	 * Calculation formula:(x * 50) + 500 -ma
	 */
	return ((chg_ctrl_b & SC2703_IBAT_CHG_MASK) * 50) + 500;
}

int sc2703_charger_status_get(void)
{
	u8 status[4];
	int ret;

	ret = regmap_bulk_read(sc2703_chg->regmap, SC2703_STATUS_A, status,
			       ARRAY_SIZE(status));
	if (ret)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	/* If wall charger not connected, we're definitely discharging */
	if (!(status[0] & SC2703_S_ADP_DET_MASK))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	switch (status[3] & SC2703_S_CHG_STAT_MASK) {
	case SC2703_S_CHG_STAT_DCDC_OFF:
	case SC2703_S_CHG_STAT_FAULT_L2:
		ret = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case SC2703_S_CHG_STAT_FAULT_L1:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case SC2703_S_CHG_STAT_PRE:
	case SC2703_S_CHG_STAT_CC:
	case SC2703_S_CHG_STAT_CV:
	case SC2703_S_CHG_STAT_TOP_OFF:
		ret = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case SC2703_S_CHG_STAT_FULL:
		ret = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return ret;
}

static int sc2703_dcdc_output_set_cur(u32 val)
{
	u32 mask = SC2703_IIN_REV_LIM_MASK;
	u32 cfg = val << SC2703_IIN_REV_LIM_SHIFT;

	return regmap_update_bits(sc2703_chg->regmap,
		SC2703_DCDC_CTRL_B, mask, cfg);
}

void sc2703_charger_otg_enable(int enable)
{
	if (enable) {
		regmap_update_bits(sc2703_chg->regmap, SC2703_REG_UNLOCK,
			   SC2703_CHG_REG_MASK_ALL, SC2703_REG_UNLOCK_VAL);
		regmap_update_bits(sc2703_chg->regmap, SC2703_VSYS_UV_SWITCH,
			   SC2703_CHG_REG_MASK_ALL, SC2703_VSYS_UV_SWITCH_VAL);
		regmap_update_bits(sc2703_chg->regmap, SC2703_CHG_BUCK_SWITCH,
			   SC2703_CHG_REG_MASK_ALL, SC2703_CHG_REG_DISABLE_VAL);
		regmap_update_bits(sc2703_chg->regmap, SC2703_REG_UNLOCK,
			   SC2703_CHG_REG_MASK_ALL, SC2703_CHG_REG_DISABLE_VAL);
		sc2703_dcdc_output_set_cur(SC2703_IIN_REV_LIM_500MA);
		regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_A,
			SC2703_OTG_EN_MASK, enable ? SC2703_OTG_EN_MASK : 0);
		schedule_delayed_work(&sc2703_chg->otg_work,
				      msecs_to_jiffies(100));
	} else {
		cancel_delayed_work_sync(&sc2703_chg->otg_work);
		regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_A,
				   SC2703_OTG_EN_MASK, 0);
	}
}

void sc2703_charger_reset_timer(void)
{
	regmap_write(sc2703_chg->regmap, SC2703_CHG_TIMER_CTRL_B,
		     sc2703_chg->wd_interval);
}

static int sc2703_otg_func_check(void)
{
	u32 raw_st, otg_state;
	u32 reg_ctrl_a, reg_event_c;
	int ret;

	ret = regmap_read(sc2703_chg->regmap, SC2703_DCDC_CTRL_A, &reg_ctrl_a);
	if (ret)
		return ret;
	ret = regmap_read(sc2703_chg->regmap, SC2703_EVENT_C, &reg_event_c);
	if (ret)
		return ret;

	otg_state = reg_ctrl_a & SC2703_OTG_EN_MASK;
	raw_st = reg_event_c & (SC2703_E_LOWBAT_MASK |
		SC2703_E_IIN_REV_LIM_MAX_MASK | SC2703_E_IIN_REV_LIM_MASK |
		SC2703_E_VIN_REV_SHORT_MASK);
	pr_info("otg_state=%x, raw_st=%x\n", otg_state, raw_st);
	if (!otg_state || raw_st)
		return -EINVAL;

	return 0;
}

static void sc2703_otg_restart(void)
{
	regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_A,
			   SC2703_OTG_EN_MASK, 0);
	regmap_update_bits(sc2703_chg->regmap, SC2703_EVENT_C,
		(SC2703_E_LOWBAT_MASK | SC2703_E_IIN_REV_LIM_MAX_MASK
		| SC2703_E_IIN_REV_LIM_MASK | SC2703_E_VIN_REV_SHORT_MASK),
		(SC2703_E_LOWBAT_MASK | SC2703_E_IIN_REV_LIM_MAX_MASK |
		SC2703_E_IIN_REV_LIM_MASK | SC2703_E_VIN_REV_SHORT_MASK));

	sc2703_dcdc_output_set_cur(SC2703_IIN_REV_LIM_500MA);
	regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_A,
		SC2703_OTG_EN_MASK, SC2703_OTG_EN_MASK);
}

static void sc2703_otg_work(struct work_struct *work)
{
	int ret = 0;
	int retry_cnt = 0;

	if (sc2703_otg_func_check()) {
		do {
			sc2703_otg_restart();
			/*
			 * sc2703 the interval between two times OTG
			 * enable must be 100ms, otherwise OTG restart
			 * will fail, undetected anomaly.
			 */
			msleep(100);
			ret = sc2703_otg_func_check();
		} while (ret && retry_cnt++ < SC2703_OTG_RETRY_TIMES);
	}

	if (!ret) {
		sc2703_dcdc_output_set_cur(SC2703_IIN_REV_LIM_2000MA);
		schedule_delayed_work(&sc2703_chg->otg_work,
				      msecs_to_jiffies(1500));
	}
	pr_info("otg restart %s\n", ret ? "fail" : "ok");
}

int sc2703_get_fault_val(u32 *val)
{
	u32 sts, event_a;
	int ret;

	ret = regmap_read(sc2703_chg->regmap, SC2703_STATUS_D, &sts);
	if (ret)
		return ret;

	ret = regmap_read(sc2703_chg->regmap, SC2703_EVENT_A, &event_a);
	if (ret)
		return ret;

	ret = regmap_update_bits(sc2703_chg->regmap, SC2703_EVENT_A,
			   SC2703_E_VBAT_OV_MASK, SC2703_E_VBAT_OV_MASK);
	if (ret)
		return ret;

	if (sts == SC2703_S_CHG_STAT_FULL) {
		sc2703_charger_stop(SPRDBAT_CHG_END_FULL_BIT);
		sc2703_charger_start();
	}

	*val = event_a;
	return 0;
}

bool sc2703_is_support_fchg(void)
{
	u32 version;
	int ret;

	ret = regmap_read(sc2703_chg->regmap, SC2703_VERSION_INFO, &version);
	if (ret)
		return false;

	return version == SC2703_SUPPORT_FCHG;
}

static int sc2703_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc2703_charger_pdata *charger_pdata = dev_get_platdata(dev);
	struct sc2703_charger *charger;
	unsigned int status_a;
	int ext_id_state = 0, ret;

	charger = devm_kzalloc(dev, sizeof(struct sc2703_charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	charger->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!charger->regmap) {
		dev_err(&pdev->dev, "failed to get regmap\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, charger);
	charger->dev = dev;

	/* Handle DT data if provided */
	if (dev->of_node) {
		charger_pdata = sc2703_charger_of_to_pdata(dev);
		if (IS_ERR(charger_pdata))
			return PTR_ERR(charger_pdata);

		dev->platform_data = charger_pdata;
	}

	sc2703_charger_handle_pdata(charger);
	sc2703_chg = charger;

	/* Enable automatic input power reduction (for low VIN voltage) */
	ret = regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_B,
				 SC2703_IN_PWR_RED_MASK,
				 SC2703_IN_PWR_RED_MASK);
	if (ret)
		return ret;

	/* Enable automatic temperature regulation/profiling */
	ret = regmap_update_bits(sc2703_chg->regmap, SC2703_TJUNC_CTRL_A,
				 SC2703_TJUNC_REG_MASK, SC2703_TJUNC_REG_MASK);
	if (ret)
		return ret;

	ret = regmap_update_bits(sc2703_chg->regmap, SC2703_TBAT_CTRL_A,
			 SC2703_TBAT_AUTO_MASK | SC2703_TBAT_L2_EN_MASK |
			 SC2703_TBAT_EXT_MASK,
			 SC2703_TBAT_AUTO_MASK | SC2703_TBAT_L2_EN_MASK |
			 SC2703_TBAT_EXT_MASK);
	if (ret)
		return ret;

	/* Enable charging/boost, as required */
	ret = regmap_read(sc2703_chg->regmap, SC2703_STATUS_A, &status_a);
	if (ret)
		return ret;

	if (status_a & SC2703_S_ADP_DET_MASK) {
		ret = regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_A,
					 SC2703_CHG_EN_MASK |
					 SC2703_OTG_EN_MASK,
					 SC2703_CHG_EN_MASK);
		if (ret)
			return ret;
	} else if (ext_id_state > 0) {
		ret = regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_A,
					 SC2703_CHG_EN_MASK |
					 SC2703_OTG_EN_MASK,
					 SC2703_OTG_EN_MASK);
		if (ret)
			return ret;
	}

	ret = regmap_update_bits(sc2703_chg->regmap, SC2703_REG_UNLOCK,
				 SC2703_CHG_REG_MASK_ALL,
				 SC2703_REG_UNLOCK_VAL);
	if (ret)
		return ret;

	ret = regmap_update_bits(sc2703_chg->regmap, SC2703_VSYS_UV_SWITCH,
				 SC2703_CHG_REG_MASK_ALL,
				 SC2703_VSYS_UV_SWITCH_VAL);
	if (ret)
		return ret;

	ret = regmap_update_bits(sc2703_chg->regmap, SC2703_CHG_BUCK_SWITCH,
				 SC2703_CHG_REG_MASK_ALL,
				 SC2703_CHG_REG_DISABLE_VAL);
	if (ret)
		return ret;

	ret = regmap_update_bits(sc2703_chg->regmap, SC2703_REG_UNLOCK,
				 SC2703_CHG_REG_MASK_ALL,
				 SC2703_CHG_REG_DISABLE_VAL);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&sc2703_chg->otg_work, sc2703_otg_work);
	sprdbat_register_ext_ops(sprd_get_2703_ops());
	return 0;
}

static int sc2703_charger_remove(struct platform_device *pdev)
{
	struct sc2703_charger *charger = platform_get_drvdata(pdev);
	int ret;

	/* Disable charging/boost */
	ret = regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_A,
				 SC2703_CHG_EN_MASK |
				 SC2703_OTG_EN_MASK, 0);
	if (ret) {
		dev_warn(charger->dev, "Failed to disable charging/boost: %d\n",
			ret);
		return ret;
	}

	return 0;
}

static void sc2703_shutdown(struct platform_device *pdev)
{
	int ret;

	cancel_delayed_work_sync(&sc2703_chg->otg_work);
	ret = regmap_update_bits(sc2703_chg->regmap, SC2703_DCDC_CTRL_A,
				 SC2703_OTG_EN_MASK, 0);
	if (ret)
		return;

	ret = regmap_update_bits(sc2703_chg->regmap, SC2703_REG_UNLOCK,
				 SC2703_CHG_REG_MASK_ALL,
				 SC2703_REG_UNLOCK_VAL);
	if (ret)
		return;

	ret = regmap_update_bits(sc2703_chg->regmap, SC2703_CHG_BUCK_SWITCH,
				 SC2703_CHG_REG_MASK_ALL,
				 SC2703_CHG_REG_DISABLE_VAL);
	if (ret)
		return;

	ret = regmap_update_bits(sc2703_chg->regmap, SC2703_REG_UNLOCK,
				 SC2703_CHG_REG_MASK_ALL,
				 SC2703_CHG_REG_DISABLE_VAL);
	if (ret)
		return;

	sc2703_termina_vol_set(DEFAULT_CCCV_VOL);
}

static struct platform_driver sc2703_charger_driver = {
	.driver = {
		.name = "sc2703-charger",
		.of_match_table = sc2703_charger_of_match,
	},
	.probe = sc2703_charger_probe,
	.shutdown = sc2703_shutdown,
	.remove = sc2703_charger_remove,
};

static int __init sc2703_charger_init(void)
{
	return platform_driver_register(&sc2703_charger_driver);
}

static void __exit sc2703_charger_exit(void)
{
	platform_driver_unregister(&sc2703_charger_driver);
}

module_init(sc2703_charger_init);
module_exit(sc2703_charger_exit);

MODULE_DESCRIPTION("Charger Driver for SC2703");
MODULE_AUTHOR("Yuanjiang Yu <yuanjiang.yu@unisoc.com>");
MODULE_LICENSE("GPL v2");
