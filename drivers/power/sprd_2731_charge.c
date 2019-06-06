#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include "sprd_battery.h"
#include "sprd_2731.h"
#include "sprd_charge_helper.h"

#define SPRDCHG_2731_DEBUG(format, arg...) pr_info("sprd 2731:" format, ## arg)

#define BITSINDEX(b, o)	((b) * 16 + (o))
#define OTG_RETRY_TIMEOUT 50
#define OTG_RETRY_TIMER 3000

static struct sprdchg_2731_data sprdchg_2731;
static struct sprdbat_drivier_data *bat_drv_data;

static unsigned int chg2731_adi_read(unsigned long reg)
{
	unsigned int val;

	regmap_read(sprdchg_2731.reg_map, sprdchg_2731.base_addr + reg, &val);
	return val;
}

static int chg2731_adi_write(unsigned long reg, unsigned int or_val,
			     unsigned int clear_msk)
{
	return regmap_update_bits(sprdchg_2731.reg_map,
				  sprdchg_2731.base_addr + reg,
				  clear_msk, or_val);
}

void sprd2731_cal_init(void)
{
#ifdef CONFIG_OTP_SPRD_PMIC_EFUSE
	unsigned int cal;

	if (!sprd_pmic_efuse_bits_read(0, 1)) {
		sprdchg_2731.cccv_cal = 0x20;
		SPRDCHG_2731_DEBUG("sprd2731chg:have no efuse data\n");
		return;
	}

	/* bolck16,bit[15:10], cccv cal data in efuse bitmap */
	cal = sprd_pmic_efuse_bits_read(BITSINDEX(16, 10), 6);
	SPRDCHG_2731_DEBUG("cccv_cal:0x%x\n", cal);
	sprdchg_2731.cccv_cal = (cal) & 0x003F;

	/* set current cal by efuse bitmap */
	/* block16[9:5] outcurrent cal data */
	cal = sprd_pmic_efuse_bits_read(BITSINDEX(16, 5), 5);
	SPRDCHG_2731_DEBUG("outcurrent cal = 0x%x\n", cal);
	chg2731_adi_write(CHG_CFG1, BITS_OUTCUR_CAL(cal), BITS_OUTCUR_CAL(~0U));

	/* block17[9:5] incurrent cal data */
	cal = sprd_pmic_efuse_bits_read(BITSINDEX(17, 5), 5);
	SPRDCHG_2731_DEBUG("incurrent cal = 0x%x\n", cal);
	chg2731_adi_write(CHG_CFG1, BITS_INCUR_CAL(cal), BITS_INCUR_CAL(~0U));

	/* block16[4:0] boost cal data */
	cal = sprd_pmic_efuse_bits_read(BITSINDEX(16, 0), 5);
	SPRDCHG_2731_DEBUG("boost cal = 0x%x\n", cal);
	chg2731_adi_write(RG_BST_CFG1, BITS_BST_CAL(cal), BITS_BST_CAL(~0U));
#endif
}

void sprd2731_reset_timer(void)
{
/*
 *	SPRDCHG_2731_DEBUG("sprd2731 reset 30s timer\n");
 *	sprdchg_reset_wdt(30);
 */
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

void sprd2731_otg_enable(int enable)
{
	SPRDCHG_2731_DEBUG("%s enable =%d\n", __func__, enable);
	glb_adi_write(ANA_REG_GLB_MODULE_EN1,
		      BIT_ANA_SWITCH_CHG_EN, BIT_ANA_SWITCH_CHG_EN);
	if (enable) {
		chg2731_adi_write(SW_LED_EN, BIT_OTG_REG_EN, BIT_OTG_REG_EN);
		schedule_delayed_work(&sprdchg_2731.otg_work,
				 msecs_to_jiffies(500));
	} else {
		cancel_delayed_work_sync(&sprdchg_2731.otg_work);
		chg2731_adi_write(SW_LED_EN, 0, BIT_OTG_REG_EN);
	}
}

void sprd2731_disable_chg(unsigned int flag)
{
	sprdchg_wdt_stop();
	chg2731_adi_write(CHG_CFG0, SPRD2731_PD, SPRD2731_PD);
}

unsigned int sprd2731_get_chg_status(void)
{
	return glb_adi_read(ANA_REG_GLB_CHGR_STATUS) & BIT_CHGR_CV_STATUS;
}

void sprd2731_set_chg_current_limit(unsigned int cur)
{
	unsigned char reg_val;

	if (cur <= 100)
		reg_val = 0;
	else if (cur <= 500)
		reg_val = 3;
	else if (cur <= 900)
		reg_val = 2;
	else
		reg_val = 1;
	chg2731_adi_write(CHG_CFG5, BITS_ICRSET(reg_val), BITS_ICRSET(~0U));
}

void sprd2731_set_chg_current(unsigned int cur)
{
	unsigned int reg_val = 0;

	if (cur <= 450)
		reg_val = 0;
	else if (cur >= 2000)
		reg_val = 0x1f;
	else
		reg_val = (cur - 450) / 50;

	/* cur = prechg_rng(450) + x*50 */
	chg2731_adi_write(CHG_CFG0, BITS_PRECHG_RNG(0x3), BITS_PRECHG_RNG(~0U));

	chg2731_adi_write(CHG_CFG1, BITS_CHG_CUR(reg_val), BITS_CHG_CUR(~0U));
}

void sprd2731_set_chgr_cur_limit(unsigned int limit)
{
	sprd2731_set_chg_current_limit(limit);
}

void sprd2731_termina_cur_set(int cur)
{
	unsigned int reg_val = 0x0;

	SPRDCHG_2731_DEBUG("%s cur =%d\n", __func__, cur);
	if (cur <= 90)
		reg_val = 0;
	else if (cur >= 265)
		reg_val = 0x7;
	else
		reg_val = ((cur - 90) / 25) + 1;

	chg2731_adi_write(CHG_CFG2, BITS_TERM_CUR(reg_val), BITS_TERM_CUR(~0U));
}

void sprd2731_termina_vol_set(unsigned int vol)
{
	unsigned int big_level = 0, small_level = 0;
	unsigned int cv = 0;

	SPRDCHG_2731_DEBUG("%s vol =%d\n", __func__, vol);

	if (vol > 4500)
		vol = 4500;

	if (vol >= 4200) {
		unsigned int temp = vol % 100;

		big_level = (vol - 4200) / 100;
		cv = DIV_ROUND_CLOSEST((temp) * 10, ONE_CCCV_STEP_VOL);
		small_level = cv + sprdchg_2731.cccv_cal;
		if (small_level > SPRDBAT_CCCV_MAX) {
			big_level++;
			cv = DIV_ROUND_CLOSEST((100 - temp) * 10,
					       ONE_CCCV_STEP_VOL);
			SPRDCHG_2731_DEBUG
			    ("cccv point is over range small=%d,big=%d\n",
			     small_level, big_level);
			small_level = sprdchg_2731.cccv_cal - cv;
		}
	} else {
		big_level = 0;
		cv = DIV_ROUND_CLOSEST(((4200 - vol) * 10), ONE_CCCV_STEP_VOL);
		if (cv > sprdchg_2731.cccv_cal)
			small_level = 0;
		else
			small_level = sprdchg_2731.cccv_cal - cv;
	}

	chg2731_adi_write(CHG_CFG0,
			  BITS_TERM_VOL_BIG(big_level) |
			  BITS_TERM_VOL_CAL(small_level),
			  BITS_TERM_VOL_BIG(~0U) | BITS_TERM_VOL_CAL(~0U));
}

int sprd2731_get_charge_status(void)
{
	int chg_status =
	    glb_adi_read(ANA_REG_GLB_CHGR_STATUS) & BIT_CHGR_CV_STATUS;

	if (chg_status == BIT_CHGR_CV_STATUS) {
		SPRDCHG_2731_DEBUG("sprd2731 charge full\n");
		return POWER_SUPPLY_STATUS_FULL;
	}
	return POWER_SUPPLY_STATUS_CHARGING;
}

int sprd2731_get_charge_fault(void)
{
	/* TODO: maybe need add later */
	return 0;
}

void sprd2731_set_vindpm(unsigned int vol)
{
	/* TODO: maybe need add later */
}

unsigned int sprd2731_get_chgcur(void)
{
	unsigned int reg_val = chg2731_adi_read(CHG_CFG1);

	SPRDCHG_2731_DEBUG("%s:reg_val =0x%x\n", __func__, reg_val);

	reg_val &= 0x1f;

	return (reg_val * 50 + 450);
}

void sprd2731_init(struct sprdbat_drivier_data *bdata)
{
	if (bdata == NULL)
		return;
	bat_drv_data = bdata;

	chg2731_adi_write(ANA_REG_GLB_MODULE_EN1,
			  BIT_ANA_SWITCH_CHG_EN, BIT_ANA_SWITCH_CHG_EN);
	sprd2731_termina_cur_set(bat_drv_data->pdata->chg_end_cur);
	sprd2731_termina_vol_set(bat_drv_data->pdata->chg_end_vol_pure);
	sprd2731_set_vindpm(bat_drv_data->pdata->chg_end_vol_pure);
}

void sprd2731_start_chg(void)
{
	chg2731_adi_write(CHG_CFG0, SPRD2731_CC_EN, SPRD2731_CC_EN);
	chg2731_adi_write(CHG_CFG0, 0, SPRD2731_PD);

	glb_adi_write(ANA_REG_GLB_MODULE_EN1,
		      0x0, BIT_ANA_CHG_WDG_EN);
	glb_adi_write(ANA_REG_GLB_RTC_CLK_EN1,
		      0x0, BIT_RTC_CHG_WDG_EN);
}

struct sprd_ext_ic_operations sprd_2731_ops = {
	.ic_init = sprd2731_init,
	.charge_start_ext = sprd2731_start_chg,
	.charge_stop_ext = sprd2731_disable_chg,
	.set_charge_cur = sprd2731_set_chg_current,
	.get_charging_status = sprd2731_get_charge_status,
	.get_charging_fault = sprd2731_get_charge_fault,
	.timer_callback_ext = sprd2731_reset_timer,
	.otg_charge_ext = sprd2731_otg_enable,
	.get_charge_cur_ext = sprd2731_get_chgcur,
	.set_termina_vol_ext = sprd2731_termina_vol_set,
	.set_input_cur_limit = sprd2731_set_chgr_cur_limit,
};

static void sprd2731_otg_work(struct work_struct *work)
{
	uint32_t raw_st = chg2731_adi_read(BST_INT_RAW)
				& (BIT_BST_UVI | BIT_OTG_SHTI | BIT_OTG_OCI);
	int retry_cnt = 0;

	if (raw_st) {
		SPRDCHG_2731_DEBUG("otg vol err, status=0x%x\n", raw_st);
		do {
			chg2731_adi_write(SW_LED_EN, 0, BIT_OTG_REG_EN);
			chg2731_adi_write(BST_INT_CLR, raw_st, raw_st);
			usleep_range(200, 220);
			chg2731_adi_write(SW_LED_EN, BIT_OTG_REG_EN,
				BIT_OTG_REG_EN);
			usleep_range(OTG_RETRY_TIMER, OTG_RETRY_TIMER + 200);
			raw_st = chg2731_adi_read(BST_INT_RAW)
				& (BIT_BST_UVI | BIT_OTG_SHTI | BIT_OTG_OCI);
		} while (raw_st && (retry_cnt++ < OTG_RETRY_TIMEOUT));
		SPRDCHG_2731_DEBUG("retry_cnt=%d,status=0x%x\n",
				retry_cnt, raw_st);

		if (retry_cnt >= OTG_RETRY_TIMEOUT)
			SPRDCHG_2731_DEBUG("retry_cnt=%d,status=0x%x\n",
					retry_cnt, raw_st);
	}

	if (!raw_st) {
		SPRDCHG_2731_DEBUG("%s\n", __func__);
		schedule_delayed_work(&sprdchg_2731.otg_work,
			msecs_to_jiffies(1500));
	}
}


static int sprdchg_2731_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	u32 value = 0;
	int ret = 0;

	sprdchg_2731.reg_map = dev_get_regmap(pdev->dev.parent, NULL);
	if (!sprdchg_2731.reg_map) {
		dev_err(&pdev->dev, "%s :NULL regmap for charge 2731\n",
			__func__);
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "reg", &value);
	if (ret) {
		dev_err(&pdev->dev, "%s :no reg of property for pmic fgu\n",
			__func__);
		return -ENODEV;
	}
	sprdchg_2731.base_addr = (unsigned long)value;

	sprd2731_cal_init();
	INIT_DELAYED_WORK(&sprdchg_2731.otg_work,
			  sprd2731_otg_work);

	sprdbat_register_ext_ops(&sprd_2731_ops);

	SPRDCHG_2731_DEBUG("%s probe ok\n", __func__);
	return 0;
}

static int sprdchg_2731_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id sprdchg_2731_of_match[] = {
	{.compatible = "sprd,sc2731-charger",},
	{}
};

static struct platform_driver sprdchg_2731_driver = {
	.driver = {
		   .name = "sc2731-charger",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(sprdchg_2731_of_match),
		   },
	.probe = sprdchg_2731_probe,
	.remove = sprdchg_2731_remove
};

static int __init sprdchg_2731_driver_init(void)
{
	return platform_driver_register(&sprdchg_2731_driver);
}

static void __exit sprdchg_2731_driver_exit(void)
{
	platform_driver_unregister(&sprdchg_2731_driver);
}

subsys_initcall_sync(sprdchg_2731_driver_init);
module_exit(sprdchg_2731_driver_exit);
