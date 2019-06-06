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
#include "sprd_2723_charge.h"

#define BITSINDEX(b, o)	((b) * 8 + (o))
#define SPRDCHG_DEBUG(format, arg...) pr_info("sprdchg: " format, ## arg)

static struct regmap *reg_map;
static struct sprd2723_platform_data *g_pdata;
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
int sprdchg_cccv_cal_get(unsigned int *p_cal_data)
{
	unsigned int data, blk0;

	blk0 = sprd_pmic_efuse_block_read(0);

	if (blk0 & BIT(7))
		return 0;

	data = sprd_pmic_efuse_bits_read(BITSINDEX(14, 0), 6);
	pr_info("sprdchg_cccv_cal_get data:0x%x\n", data);
	*p_cal_data = (data) & 0x003F;
	return 1;
}
#endif

void sprdchg_set_chg_ovp(uint32_t ovp_vol)
{
	uint32_t temp;

	if (ovp_vol > SPRDBAT_CHG_OVP_LEVEL_MAX)
		ovp_vol = SPRDBAT_CHG_OVP_LEVEL_MAX;

	if (ovp_vol < SPRDBAT_CHG_OVP_LEVEL_MIN)
		ovp_vol = SPRDBAT_CHG_OVP_LEVEL_MIN;

	temp = ((ovp_vol - SPRDBAT_CHG_OVP_LEVEL_MIN) / 100);

	chg_adi_write(ANA_REG_GLB_CHGR_CTRL2, 0,
		BIT_CHGR_CC_EN);

	chg_adi_write(ANA_REG_GLB_CHGR_CTRL1,
		      BITS_VCHG_OVP_V(temp),
		      BITS_VCHG_OVP_V(~0U));

	chg_adi_write(ANA_REG_GLB_CHGR_CTRL2,
		BIT_CHGR_CC_EN,
		BIT_CHGR_CC_EN);
}

void sprdchg_set_chg_cur(uint32_t chg_current)
{
	uint32_t temp;

	if (NULL == bat_drv_data)
		return;
	if (bat_drv_data->bat_info.chg_current_type_limit < chg_current)
		bat_drv_data->bat_info.chg_current_type_limit = chg_current;

	if (chg_current > SPRDBAT_CHG_CUR_LEVEL_MAX)
		chg_current = SPRDBAT_CHG_CUR_LEVEL_MAX;

	if (chg_current < SPRDBAT_CHG_CUR_LEVEL_MIN)
		chg_current = SPRDBAT_CHG_CUR_LEVEL_MIN;

	if (chg_current < 1400) {
		temp = ((chg_current - 300) / 50);
	} else {
		temp = ((chg_current - 1400) / 100);
		temp += 0x16;
	}


	chg_adi_write(ANA_REG_GLB_CHGR_CTRL2, 0,
		BIT_CHGR_CC_EN);

	chg_adi_write(ANA_REG_GLB_CHGR_CTRL1,
		      BITS_CHGR_CC_I(temp),
		      BITS_CHGR_CC_I(~0U));

	chg_adi_write(ANA_REG_GLB_CHGR_CTRL2,
		BIT_CHGR_CC_EN, BIT_CHGR_CC_EN);
}

uint32_t sprdchg_get_chg_cur(void)
{
	int rawdata = 0;

	if (chg_adi_read(ANA_REG_GLB_CHGR_CTRL2) & 0x2) {
		rawdata = chg_adi_read(ANA_REG_GLB_CHGR_CTRL1);
		rawdata = (rawdata >> 10 & 0x1f);
		SPRDCHG_DEBUG("sprdchg_get_chg_cur rawdata * 50+300=%d\n",
			rawdata * 50+300);
		return (rawdata * 50 + 300);
	} else {
		return 0;
	}
}

uint32_t sprdchg_get_cccvpoint(void)
{
	int shft = __ffs(BITS_CHGR_CV_V(~0U));

	return (chg_adi_read(ANA_REG_GLB_CHGR_CTRL0)
		& BITS_CHGR_CV_V(~0U)) >> shft;
}

uint32_t sprdchg_tune_endvol_cccv(uint32_t chg_end_vol, uint32_t cal_cccv)
{
	uint32_t cv;

	chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
		      BITS_CHGR_END_V(0U), BITS_CHGR_END_V(~0U));
	if (chg_end_vol >= 4200) {
		if (chg_end_vol < 4300) {
			cv = (((chg_end_vol - 4200) * 10) +
			      (ONE_CCCV_STEP_VOL >> 1)) / ONE_CCCV_STEP_VOL +
			    cal_cccv;
			if (cv > SPRDBAT_CCCV_MAX) {
				SPRDCHG_DEBUG("sprdchg: cv > CCCV_MAX!\n");
				chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
					      BITS_CHGR_END_V(1),
					      BITS_CHGR_END_V(~0U));
				return (cal_cccv -
					(((4300 - chg_end_vol) * 10) +
					 (ONE_CCCV_STEP_VOL >> 1)) /
					ONE_CCCV_STEP_VOL);
			} else {
				return cv;
			}
		} else {
			cv = (((chg_end_vol - 4300) * 10) +
			      (ONE_CCCV_STEP_VOL >> 1)) / ONE_CCCV_STEP_VOL +
			    cal_cccv;
			if (cv > SPRDBAT_CCCV_MAX) {
				SPRDCHG_DEBUG("sprdchg: cv > CCCV_MAX!\n");
				chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
					      BITS_CHGR_END_V(2),
					      BITS_CHGR_END_V(~0U));
				return (cal_cccv -
					(((4400 - chg_end_vol) * 10) +
					 (ONE_CCCV_STEP_VOL >> 1)) /
					ONE_CCCV_STEP_VOL);
			} else {
				chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
					      BITS_CHGR_END_V(1),
					      BITS_CHGR_END_V(~0U));
				return cv;
			}
		}
	} else {
		cv = (((4200 - chg_end_vol) * 10) +
		      (ONE_CCCV_STEP_VOL >> 1)) / ONE_CCCV_STEP_VOL;
		if (cv > cal_cccv)
			return 0;
		else
			return (cal_cccv - cv);
	}
}

uint32_t sprdchg_cccv_cal(uint32_t chg_end_vol_pure)
{
	uint32_t ret = chg_end_vol_pure;
#ifdef CONFIG_OTP_SPRD_PMIC_EFUSE
	uint32_t cv_point;

	if (sprdchg_cccv_cal_get(&cv_point)) {
		SPRDCHG_DEBUG("cccv_point efuse:%d\n", cv_point);
		ret = sprdchg_tune_endvol_cccv(chg_end_vol_pure, cv_point);
		SPRDCHG_DEBUG("cccv_point new cccv:%d\n", ret);
	} else {
		SPRDCHG_DEBUG("cccv_point default\n");
	}
#endif
	return ret;
}

void sprdchg_set_cccvpoint(unsigned int cvpoint)
{
	int new_cvpoint;

	new_cvpoint = sprdchg_cccv_cal(cvpoint);
	chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
		BITS_CHGR_CV_V(new_cvpoint),
		BITS_CHGR_CV_V(~0U));

}

int sprdchg_bc1p2_disable(int disable)
{
	if (disable)
		glb_adi_write(ANA_REG_GLB_CHGR_DET_FGU_CTRL,
				BIT_DP_DM_SW_EN, BIT_DP_DM_SW_EN);
	else
		glb_adi_write(ANA_REG_GLB_CHGR_DET_FGU_CTRL,
				0, BIT_DP_DM_SW_EN);
	return 0;
}

static void _sprdchg_set_recharge(void)
{
	chg_adi_write(ANA_REG_GLB_CHGR_CTRL2,
		BIT_RECHG, BIT_RECHG);
}

static void _sprdchg_stop_recharge(void)
{
	chg_adi_write(ANA_REG_GLB_CHGR_CTRL2, 0, BIT_RECHG);
}

void sprdchg_stop_charge(unsigned int flag)
{
	chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
		BIT_CHGR_PD, BIT_CHGR_PD);
	_sprdchg_stop_recharge();
}

void sprdchg_start_charge(void)
{
	chg_adi_write(ANA_REG_GLB_CHGR_CTRL0, 0, BIT_CHGR_PD);
	_sprdchg_set_recharge();
}

void sprdchg_set_eoc_level(int level)
{
	chg_adi_write(ANA_REG_GLB_CHGR_CTRL2,
		BITS_CHGR_ITERM(level),
		BITS_CHGR_ITERM(~0U));
}

int sprdchg_get_eoc_level(void)
{
	int shft = __ffs(BITS_CHGR_ITERM(~0U));

	return (chg_adi_read(ANA_REG_GLB_CHGR_CTRL2) &
		BITS_CHGR_ITERM(~0U)) >>
	    shft;
}

int sprdchg_get_cccvstate(void)
{
	SPRDCHG_DEBUG("sprdbat:cv state:0x%x, iterm:0x%x",
		chg_adi_read(ANA_REG_GLB_CHGR_STATUS),
		chg_adi_read(ANA_REG_GLB_CHGR_CTRL2));
	return ((chg_adi_read(ANA_REG_GLB_CHGR_STATUS) &
		BIT_CHGR_CV_STATUS) ? 1 : 0);
}

int sprdchg_get_charge_fault(void)
{
	return 0;
}

void sprdchg_chip_init(struct sprdbat_drivier_data *bdata)
{
	if (NULL == bdata)
		return;
	bat_drv_data = bdata;
	glb_adi_write(ANA_REG_GLB_CHGR_CTRL0,
		BIT_CHGLDO_DIS, BIT_CHGLDO_DIS);
	glb_adi_write(ANA_REG_GLB_CHGR_CTRL2,
		BIT_CHGR_CC_EN, BIT_CHGR_CC_EN);
	chg_adi_write(ANA_REG_GLB_CHGR_CTRL0,
		      BITS_CHGR_CV_V(0U), BITS_CHGR_CV_V(~0U));

	if (bat_drv_data->pdata->chg_end_vol_pure < 4300) {
		chg_adi_write(ANA_REG_GLB_CHGR_CTRL2,
			BITS_CHGR_DPM(2), BITS_CHGR_DPM(~0U));
	} else {
		chg_adi_write(ANA_REG_GLB_CHGR_CTRL2,
			BITS_CHGR_DPM(3), BITS_CHGR_DPM(~0U));
	}

	sprdchg_set_cccvpoint(bat_drv_data->pdata->chg_end_vol_pure);
	sprdchg_set_chg_ovp(bat_drv_data->pdata->ovp_stop);
	sprdchg_set_eoc_level(0);
}

static __used irqreturn_t sprdchg_vchg_ovi_irq(int irq, void *dev_id)
{
	int value;
	struct sprd2723_device *sprd_2723 = dev_id;
	struct sprd2723_platform_data *pdata = sprd_2723->dev->platform_data;

	value = gpio_get_value(pdata->gpio_vchg_ovi);
	if (value) {
		irq_set_irq_type(pdata->irq_vchg_ovi,
				 IRQ_TYPE_LEVEL_LOW);
	} else {
		irq_set_irq_type(pdata->irq_vchg_ovi,
				 IRQ_TYPE_LEVEL_HIGH);
	}
	power_supply_changed(&sprd_2723->charger);
	return IRQ_HANDLED;
}

static irqreturn_t sprdchg_chg_cv_irq(int irq, void *dev_id)
{
	int value;
	struct sprd2723_device *sprd_2723 = dev_id;
	struct sprd2723_platform_data *pdata = sprd_2723->dev->platform_data;

	value = gpio_get_value(pdata->gpio_chg_cv_state);
	power_supply_changed(&sprd_2723->charger);
	return IRQ_HANDLED;
}

static  struct sprd2723_platform_data *
sprdchg_2723_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct sprd2723_platform_data *pdata = NULL;
	int gpio;

	if (!np)
		return ERR_PTR(-ENODEV);
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	gpio = of_get_named_gpio(np, "chg-cv-gpios", 0);
	if (gpio < 0)
		SPRDCHG_DEBUG("chg-cv-gpios parse err\n");
	else
		pdata->gpio_chg_cv_state = (uint32_t)gpio;

	gpio = of_get_named_gpio(np, "chg-ovi-gpios", 0);
	if (gpio < 0)
		SPRDCHG_DEBUG("chg-ovi-gpios parse err\n");
	else
		pdata->gpio_vchg_ovi = (uint32_t)gpio;

	return pdata;
}

struct sprd_ext_ic_operations sprd_2723_op = {
	.ic_init = sprdchg_chip_init,
	.charge_start_ext = sprdchg_start_charge,
	.set_charge_cur = sprdchg_set_chg_cur,
	.charge_stop_ext = sprdchg_stop_charge,
	.get_charging_status = sprdchg_get_cccvstate,
	.get_charging_fault =  sprdchg_get_charge_fault,
	.get_charge_cur_ext = sprdchg_get_chg_cur,
	.set_termina_vol_ext = sprdchg_set_cccvpoint,
};

static int sprd2723_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	return -EINVAL;
}


static int sprdchg_2723_probe(struct platform_device *pdev)
{
	struct sprd2723_device *sprd_2723 = NULL;
	struct sprd2723_platform_data *pdata = pdev->dev.platform_data;
	int ret = 0;
	struct power_supply *ret_ptr = NULL;
	struct power_supply_desc *charger_desc = NULL;

	reg_map = dev_get_regmap(pdev->dev.parent, NULL);
	if (!reg_map)
		dev_err(&pdev->dev, "%s :NULL regmap for charge 2723\n",
			__func__);

	sprd_2723 = devm_kzalloc(&pdev->dev, sizeof(*sprd_2723), GFP_KERNEL);
	if (!sprd_2723)
		return -ENOMEM;

	if (!pdata)
		pdata = sprdchg_2723_parse_dt(&pdev->dev);
	if (IS_ERR_OR_NULL(pdata))
		return PTR_ERR(pdata);

	g_pdata = pdata;
	sprd_2723->dev = &pdev->dev;
	platform_set_drvdata(pdev, sprd_2723);

	if (pdata->gpio_chg_cv_state) {
		ret = devm_gpio_request(&pdev->dev,
			pdata->gpio_chg_cv_state, "chg_cv_state");
		if (ret) {
			dev_err(&pdev->dev,
				"failed to request gpio: %d\n", ret);
			return ret;
		}
		gpio_direction_input(pdata->gpio_chg_cv_state);
		pdata->irq_chg_cv_state = gpio_to_irq(pdata->gpio_chg_cv_state);

		irq_set_status_flags(pdata->irq_chg_cv_state, IRQ_NOAUTOEN);
		ret = devm_request_threaded_irq(&pdev->dev,
			pdata->irq_chg_cv_state, NULL,
			sprdchg_chg_cv_irq,
			IRQF_NO_SUSPEND | IRQF_ONESHOT,
			"sprdbat_chg_cv_state", sprd_2723);
		if (ret) {
			dev_err(&pdev->dev,
				"failed to request chg cv irq: %d\n", ret);
			return ret;
		}
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
		pdata->irq_vchg_ovi = gpio_to_irq(pdata->gpio_vchg_ovi);

		irq_set_status_flags(pdata->irq_vchg_ovi, IRQ_NOAUTOEN);
		ret = devm_request_threaded_irq(&pdev->dev,
			pdata->irq_vchg_ovi, NULL,
			sprdchg_vchg_ovi_irq,
			IRQF_NO_SUSPEND | IRQF_ONESHOT,
			"sprdbat_vchg_ovi", sprd_2723);
		if (ret) {
			dev_err(&pdev->dev,
				"failed to request chg ovi irq: %d\n", ret);
			return ret;
		}
	}
	sprdbat_register_ext_ops(&sprd_2723_op);
	charger_desc = devm_kzalloc(&pdev->dev, sizeof(*charger_desc), GFP_KERNEL);
	if (charger_desc == NULL) {
		ret = -ENOMEM;
		sprd_2723 = NULL;
		return ret;
	}
	sprd_2723->charger.desc = charger_desc;
	charger_desc->name = CHG_PSY_INNER;
	charger_desc->type = POWER_SUPPLY_TYPE_UNKNOWN;
	charger_desc->get_property = sprd2723_get_property;

	ret_ptr = power_supply_register(&pdev->dev, sprd_2723->charger.desc, NULL);
	if (IS_ERR(ret_ptr)) {
		dev_err(&pdev->dev, "failed to register sprd_2723 psy: %p\n",
			ret_ptr);
		return PTR_ERR(ret_ptr);
	}
	SPRDCHG_DEBUG("%s probe ok\n", __func__);
	return 0;
}

static int sprdchg_2723_remove(struct platform_device *pdev)
{
	struct sprd2723_device *sprd_2723 = platform_get_drvdata(pdev);

	power_supply_unregister(&sprd_2723->charger);
	return 0;
}

static struct of_device_id sprdchg_2723_of_match[] = {
	{.compatible = "sprd,sc2723-charger",},
	{}
};

static struct platform_driver sprdchg_2723_driver = {
	.driver = {
		   .name = "sc2723-charger",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(sprdchg_2723_of_match),
		   },
	.probe = sprdchg_2723_probe,
	.remove = sprdchg_2723_remove
};

static int __init sprdchg_2723_driver_init(void)
{
	return platform_driver_register(&sprdchg_2723_driver);
}

static void __exit sprdchg_2723_driver_exit(void)
{
	platform_driver_unregister(&sprdchg_2723_driver);
}

subsys_initcall_sync(sprdchg_2723_driver_init);
module_exit(sprdchg_2723_driver_exit);
