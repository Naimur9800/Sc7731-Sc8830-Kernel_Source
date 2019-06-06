#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/param.h>
#include <linux/stat.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include "sprd_battery.h"
#include "sprd_2700.h"

#define SPRCHG_2700_DEBUG(format, arg...) pr_info("sprd 2700: " format, ## arg)
static struct sprdbat_drivier_data *bat_drv_data;

static void sprdchg_usb_aic_set(int usb_aic_vol)
{
	sprd_2700_usb_aic_set(usb_aic_vol);
}

static void sprdchg_cvl_set(unsigned int cvl_vol)
{
	sprd_2700_sprdchg_cvl_set(cvl_vol);
}

static void sprdchg_2700_termina_cur_set(unsigned int cur)
{
	sprd_2700_termina_cur_set(cur);
}

static void sprdchg_2700_reset_timer(void)
{
	sprd_2700_reset_timer();
}

static void sprdchg_2700_otg_enable(int enable)
{
	sprd_2700_otg_enable(enable);
}

static void sprdchg_2700_stop_chg(unsigned int flag)
{
	SPRCHG_2700_DEBUG("%s stop\n", __func__);
	sprd_2700_stop_chg();
}

static int sprdchg_2700_get_charge_status(void)
{
	BYTE chg_status = sprd_2700_get_chg_status();

	chg_status = (chg_status & CHG_STAT_BIT) >> CHG_STAT_SHIFT;

	switch (chg_status) {
	case CHG_SHUTDOWN:
		SPRCHG_2700_DEBUG("sprd_2700 charge shutdown\n");
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case CHG_CHGING:
		SPRCHG_2700_DEBUG("sprd_2700 is charging\n");
		return POWER_SUPPLY_STATUS_CHARGING;
	case CHG_DONE:
		SPRCHG_2700_DEBUG("sprd_2700 charge full\n");
		return POWER_SUPPLY_STATUS_FULL;
	case CHG_FAULT:
		SPRCHG_2700_DEBUG("sprd_2700 charge fault\n");
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	default:
		SPRCHG_2700_DEBUG("sprd_2700 charge default\n");
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}
}

static unsigned int sprdchg_2700_get_chg_cur(void)
{
	unsigned int cur;

	cur = sprd_2700_get_chg_current();
	return cur;
}

static void sprdchg_2700_set_chg_cur(unsigned int cur)
{
	 SPRCHG_2700_DEBUG("%s cur =%d\n", __func__, cur);
	 sprd_2700_set_chg_current(cur);
}

static int sprdchg_2700_get_charge_fault(void)
{
	BYTE fault_val = 0;
	BYTE reg_val = 0;
	int ret = 0;

	reg_val = sprd_2700_get_fault_val();
	fault_val = (reg_val & FAULT_BIT) >> FAULT_SHIFT;
	switch (fault_val) {
	case CHG_NO_FAULT:
		SPRCHG_2700_DEBUG("no fault\n");
		ret = SPRDBAT_CHG_END_NONE_BIT;
		break;
	case CHG_THM_SHUTDOWN:
		SPRCHG_2700_DEBUG("chg hot\n");
		ret = SPRDBAT_CHG_END_OTP_OVERHEAT_BIT;
		break;
	case CHG_VBAT_OVP:
		SPRCHG_2700_DEBUG("chg vbat ovp\n");
		ret = SPRDBAT_CHG_END_BAT_OVP_BIT;
		break;
	default:
		SPRCHG_2700_DEBUG("chg unspec fault\n");
		ret = SPRDBAT_CHG_END_UNSPEC;
	}
	return ret;
}

static void sprdchg_2700_start_chg(void)
{
	SPRCHG_2700_DEBUG("%s start\n", __func__);
	sprd_2700_start_chg();
}

static void sprdchg_2700_init(struct sprdbat_drivier_data *pdata)
{
	if (pdata == NULL)
		return;
	bat_drv_data = pdata;
	sprd_2700_init();
	sprdchg_usb_aic_set(bat_drv_data->pdata->chg_end_vol_pure);
	sprdchg_cvl_set(bat_drv_data->pdata->chg_end_vol_pure);
	sprdchg_2700_termina_cur_set(bat_drv_data->pdata->chg_end_cur);
}

static struct sprd_ext_ic_operations sprd_2700_op = {
	.ic_init = sprdchg_2700_init,
	.charge_start_ext = sprdchg_2700_start_chg,
	.charge_stop_ext = sprdchg_2700_stop_chg,
	.set_charge_cur = sprdchg_2700_set_chg_cur,
	.get_charge_cur_ext = sprdchg_2700_get_chg_cur,
	.get_charging_status = sprdchg_2700_get_charge_status,
	.get_charging_fault = sprdchg_2700_get_charge_fault,
	.timer_callback_ext = sprdchg_2700_reset_timer,
	.set_termina_vol_ext = sprdchg_cvl_set,
	.set_termina_cur_ext = sprdchg_2700_termina_cur_set,
	.otg_charge_ext = sprdchg_2700_otg_enable,
};

struct sprd_ext_ic_operations *sprd_get_2700_ops(void)
{
	return &sprd_2700_op;
}
