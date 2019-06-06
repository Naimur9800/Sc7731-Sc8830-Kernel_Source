/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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

#include <linux/reboot.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include "sprd_charge_helper.h"
#include "sprd_2720_charge.h"

#define BITSINDEX(b, o)	((b) * 16 + (o))
#define SPRDCHG_DEBUG(format, arg...) pr_info("sc2720: " format, ## arg)

static struct regmap *reg_map;
static struct sprd2720_platform_data *g_pdata;
static struct sprdbat_drivier_data *bat_drv_data;

static unsigned int chg_adi_read(unsigned long reg)
{
	unsigned int val;

	regmap_read(reg_map, reg, &val);
	return val;
}

static int chg_adi_write(unsigned long reg, unsigned int or_val,
				 unsigned int clear_msk)
{
	return regmap_update_bits(reg_map, reg, clear_msk, or_val);
}

#ifdef CONFIG_OTP_SPRD_PMIC_EFUSE
static void sprdchg_cccv_cal_get(void)
{
	unsigned int data;

	/* cccv cal is on offset 0 of block 13, width is 6bit. */
	data = sprd_pmic_efuse_bits_read(BITSINDEX(13, 0), 6);
	pr_info("sprdchg_cccv_cal_get data:0x%x\n", data);
	g_pdata->cccv_cal = (data) & SPRDBAT_CCCV_MSK;
}
#else
static void sprdchg_cccv_cal_get(void)
{
	g_pdata->cccv_cal = SPRDBAT_CCCV_DEF;
}
#endif

static void sprdchg_set_chg_ovp(uint32_t ovp_vol)
{
	uint32_t temp;

	if (ovp_vol >= SPRDCHG_OVP_9700MV)
		temp = 3;
	else if (ovp_vol >= SPRDCHG_OVP_7000MV)
		temp = 2;
	else if (ovp_vol >= SPRDCHG_OVP_6500MV)
		temp = 1;
	else
		temp = 0;

	chg_adi_write(ANA_REG_GLB_CHGR_CTRL1,
		      BITS_VCHG_OVP_V(temp),
		      BITS_VCHG_OVP_V(~0U));
}

static void sprdchg_set_chg_cur(uint32_t chg_current)
{
	uint32_t temp;

	if (bat_drv_data->bat_info.user_set_input_cur_max != -1 &&
			chg_current > bat_drv_data->bat_info.user_set_input_cur_max) {
		chg_current = bat_drv_data->bat_info.user_set_input_cur_max;
		SPRDCHG_DEBUG("sprdchg_set_chg_cur =%d from user\n",
			bat_drv_data->bat_info.user_set_input_cur_max);
	}

	if (bat_drv_data == NULL)
		return;
	if (bat_drv_data->bat_info.chg_current_type_limit < chg_current)
		bat_drv_data->bat_info.chg_current_type_limit = chg_current;

	if (chg_current > SPRDCHG_CUR_MAX)
		chg_current = SPRDCHG_CUR_MAX;

	if (chg_current < SPRDCHG_CUR_MIN)
		chg_current = SPRDCHG_CUR_MIN;

	if (chg_current < SPRDCHG_CUR_800MA) {
		temp = ((chg_current - SPRDCHG_CUR_MIN) / SPRDCHG_CUR_STEP50);
	} else {
		temp = ((chg_current - SPRDCHG_CUR_800MA)
			/ SPRDCHG_CUR_STEP100);
		temp += SPRDCHG_CUR_800MA_REG;
	}

	chg_adi_write(ANA_REG_GLB_CHGR_CTRL0, 0,
		BIT_CHGR_CC_EN);

	chg_adi_write(ANA_REG_GLB_CHGR_CTRL1,
		      BITS_CHGR_CC_I(temp),
		      BITS_CHGR_CC_I(~0U));

	chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
		BIT_CHGR_CC_EN, BIT_CHGR_CC_EN);
}

static uint32_t sprdchg_get_chg_cur(void)
{
	int rawdata;
	int shft = __ffs(BITS_CHGR_CC_I(~0U));

	if (chg_adi_read(ANA_REG_GLB_CHGR_CTRL0) & BIT_CHGR_CC_EN) {
		rawdata = chg_adi_read(ANA_REG_GLB_CHGR_CTRL1);
		rawdata = (rawdata & BITS_CHGR_CC_I(~0U)) >> shft;

		if (rawdata >= SPRDCHG_CUR_800MA_REG)
			rawdata = SPRDCHG_CUR_800MA
				+ (rawdata - SPRDCHG_CUR_800MA_REG)
				* SPRDCHG_CUR_STEP100;
		else
			rawdata = SPRDCHG_CUR_MIN
				+ rawdata * SPRDCHG_CUR_STEP50;

		SPRDCHG_DEBUG("sprdchg_get_chg_cur =%d\n", rawdata);
		return rawdata;
	} else {
		return 0;
	}
}

static void sprdchg_termina_vol_set(unsigned int vol)
{
	unsigned int big_level, small_level, cv;

	SPRDCHG_DEBUG("%s vol =%d\n", __func__, vol);

	if (vol > SPRDCHG_VOLBIG_MAX)
		vol = SPRDCHG_VOLBIG_MAX;

	if (vol >= SPRDCHG_VOLBIG_MIN) {
		unsigned int temp = vol % SPRDCHG_VOLBIG_STEP;

		big_level = (vol - SPRDCHG_VOLBIG_MIN) / SPRDCHG_VOLBIG_STEP;
		cv = DIV_ROUND_CLOSEST((temp) * 10, ONE_CCCV_STEP_VOL);
		small_level = cv + g_pdata->cccv_cal;
		if (small_level > SPRDBAT_CCCV_MAX) {
			big_level++;
			cv = DIV_ROUND_CLOSEST(
				(SPRDCHG_VOLBIG_STEP - temp) * 10,
				ONE_CCCV_STEP_VOL);
			SPRDCHG_DEBUG
			    ("cccv point is over range small=%d,big=%d\n",
			     small_level, big_level);
			small_level = g_pdata->cccv_cal - cv;
		}
	} else {
		big_level = 0;
		cv = DIV_ROUND_CLOSEST(((SPRDCHG_VOLBIG_MIN - vol) * 10),
			ONE_CCCV_STEP_VOL);
		if (cv > g_pdata->cccv_cal)
			small_level = 0;
		else
			small_level = g_pdata->cccv_cal - cv;
	}

	chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
			  BITS_CHGR_END_V(big_level) |
			  BITS_CHGR_CV_V(small_level),
			  BITS_CHGR_END_V(~0U) | BITS_CHGR_CV_V(~0U));
}

int sprdchg_bc1p2_disable(int disable)
{
	if (disable)
		glb_adi_write(ANA_REG_GLB_CHGR_DET_FGU_CTRL,
				BIT_DP_DM_BC_ENB, BIT_DP_DM_BC_ENB);
	else
		glb_adi_write(ANA_REG_GLB_CHGR_DET_FGU_CTRL,
				0, BIT_DP_DM_BC_ENB);
	return 0;
}

static void sprdchg_stop_charge(unsigned int flag)
{
	chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
		BIT_CHGR_PD, BIT_CHGR_PD);
}

static void sprdchg_start_charge(void)
{
	chg_adi_write(ANA_REG_GLB_CHGR_CTRL0, 0, BIT_CHGR_PD);
}

static void sprdchg_set_eoc_level(int level)
{
	chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
		BITS_CHGR_ITERM(level),
		BITS_CHGR_ITERM(~0U));
}

static int sprdchg_get_cccvstate(void)
{
	SPRDCHG_DEBUG("sprdbat:cv state:0x%x, iterm:0x%x",
		chg_adi_read(ANA_REG_GLB_CHGR_STATUS),
		chg_adi_read(ANA_REG_GLB_CHGR_CTRL0));
	return ((chg_adi_read(ANA_REG_GLB_CHGR_STATUS) &
		BIT_CHGR_CV_STATUS) ? 1 : 0);
}

static int sprdchg_get_charge_fault(void)
{
	return 0;
}

static int sprdchg_get_enable_status(void)
{
	int reg_val = 0;

	reg_val = chg_adi_read(ANA_REG_GLB_CHGR_CTRL0);

	reg_val = !(reg_val & 0x01);

	return reg_val;
}

static void sprdchg_chip_init(struct sprdbat_drivier_data *bdata)
{
	if (bdata == NULL)
		return;
	bat_drv_data = bdata;
	glb_adi_write(ANA_REG_GLB_CHGR_CTRL0,
		BIT_CHGR_CC_EN, BIT_CHGR_CC_EN);
	chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
		      BITS_CHGR_CV_V(0U), BITS_CHGR_CV_V(~0U));

	if (bat_drv_data->pdata->chg_end_vol_pure < SPRDCHG_DPM_4300MV) {
		chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
			BITS_CHGR_DPM(2), BITS_CHGR_DPM(~0U));
	} else {
		chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
			BITS_CHGR_DPM(3), BITS_CHGR_DPM(~0U));
	}

	sprdchg_termina_vol_set(bat_drv_data->pdata->chg_end_vol_pure);
	sprdchg_set_chg_ovp(bat_drv_data->pdata->ovp_stop);
	sprdchg_set_eoc_level(0);
}

static struct sprd2720_platform_data *
sprdchg_2720_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct sprd2720_platform_data *pdata;
	int gpio;

	if (!np)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	gpio = of_get_named_gpio(np, "chg-cv-gpios", 0);
	if (gpio < 0)
		SPRDCHG_DEBUG("no chg-cv-gpios\n");
	else
		pdata->gpio_chg_cv_state = (uint32_t)gpio;

	gpio = of_get_named_gpio(np, "chg-ovi-gpios", 0);
	if (gpio < 0)
		SPRDCHG_DEBUG("no chg-ovi-gpios\n");
	else
		pdata->gpio_vchg_ovi = (uint32_t)gpio;

	return pdata;
}

static struct sprd_ext_ic_operations sprd_2720_op = {
	.ic_init = sprdchg_chip_init,
	.charge_start_ext = sprdchg_start_charge,
	.set_charge_cur = sprdchg_set_chg_cur,
	.charge_stop_ext = sprdchg_stop_charge,
	.get_charging_status = sprdchg_get_cccvstate,
	.get_charging_fault =  sprdchg_get_charge_fault,
	.get_charge_cur_ext = sprdchg_get_chg_cur,
	.set_termina_vol_ext = sprdchg_termina_vol_set,
	.get_charge_status = sprdchg_get_enable_status,
};

static int sprd2720_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	return -EINVAL;
}

static int sprdchg_2720_probe(struct platform_device *pdev)
{
	struct sprd2720_device *sprd_2720;
	struct sprd2720_platform_data *pdata = pdev->dev.platform_data;
	int ret = 0;
	struct power_supply *ret_ptr;
	struct power_supply_desc *charger_desc;

	reg_map = dev_get_regmap(pdev->dev.parent, NULL);
	if (!reg_map) {
		dev_err(&pdev->dev, "%s :NULL regmap for charge 2720\n",
			__func__);
		return -EINVAL;
	}
	sprd_2720 = devm_kzalloc(&pdev->dev, sizeof(*sprd_2720), GFP_KERNEL);
	if (!sprd_2720)
		return -ENOMEM;

	if (!pdata) {
		pdata = sprdchg_2720_parse_dt(&pdev->dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	g_pdata = pdata;
	sprd_2720->dev = &pdev->dev;
	platform_set_drvdata(pdev, sprd_2720);

	if (pdata->gpio_chg_cv_state) {
		ret = devm_gpio_request(&pdev->dev,
			pdata->gpio_chg_cv_state, "chg_cv_state");
		if (ret) {
			dev_err(&pdev->dev,
				"failed to request gpio: %d\n", ret);
			return ret;
		}
		gpio_direction_input(pdata->gpio_chg_cv_state);
	}

	if (pdata->gpio_vchg_ovi) {
		ret = devm_gpio_request(&pdev->dev,
			pdata->gpio_vchg_ovi, "vchg_ovi");
		if (ret) {
			dev_err(&pdev->dev,
				"failed to request gpio: %d\n", ret);
			return ret;
		}
		gpio_direction_input(pdata->gpio_vchg_ovi);
	}
	sprdchg_cccv_cal_get();
	sprdbat_register_ext_ops(&sprd_2720_op);
	charger_desc = devm_kzalloc(&pdev->dev,
		sizeof(*charger_desc), GFP_KERNEL);
	if (!charger_desc)
		return -ENOMEM;

	sprd_2720->charger.desc = charger_desc;
	charger_desc->name = CHG_PSY_INNER;
	charger_desc->type = POWER_SUPPLY_TYPE_UNKNOWN;
	charger_desc->get_property = sprd2720_get_property;
	ret_ptr = power_supply_register(&pdev->dev,
		sprd_2720->charger.desc, NULL);
	if (IS_ERR(ret_ptr)) {
		dev_err(&pdev->dev, "failed to register sprd_2720 psy: %p\n",
			ret_ptr);
		return PTR_ERR(ret_ptr);
	}
	SPRDCHG_DEBUG("%s probe ok\n", __func__);
	return 0;
}

static int sprdchg_2720_remove(struct platform_device *pdev)
{
	struct sprd2720_device *sprd_2720 = platform_get_drvdata(pdev);

	power_supply_unregister(&sprd_2720->charger);
	return 0;
}

static const struct of_device_id sprdchg_2720_of_match[] = {
	{.compatible = "sprd,sc2720-charger",},
	{}
};

static struct platform_driver sprdchg_2720_driver = {
	.driver = {
		.name = "sc2720-charger",
		.of_match_table = of_match_ptr(sprdchg_2720_of_match),
	},
	.probe = sprdchg_2720_probe,
	.remove = sprdchg_2720_remove
};

static int __init sprdchg_2720_driver_init(void)
{
	return platform_driver_register(&sprdchg_2720_driver);
}

static void __exit sprdchg_2720_driver_exit(void)
{
	platform_driver_unregister(&sprdchg_2720_driver);
}

subsys_initcall_sync(sprdchg_2720_driver_init);
module_exit(sprdchg_2720_driver_exit);
