#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/param.h>
#include <linux/stat.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include "sprd_battery.h"
#include "bq25896.h"

#define CHG_BQ25896_DEBUG(format, arg...) pr_info("bq25896: " format, ## arg)

static struct sprdbat_drivier_data *bat_drv_data;

void chg_bq25896_set_vindpm(int vin)
{
	unsigned char reg_val = 0x6;

	CHG_BQ25896_DEBUG("%s vin =%d\n", __func__, vin);
	if (vin <= 0)
		reg_val = 0x0;
	else if (vin >= 3100)
		reg_val = 0x1f;
	else
	/* ???why "+1" */
		reg_val = vin / 100 + 1;

	bq25896_set_vindpm(reg_val);
}

void chg_bq25896_termina_cur_set(unsigned int cur)
{
	unsigned char reg_val = 0x3;

	CHG_BQ25896_DEBUG("%s cur =%d\n", __func__, cur);
	if (cur <= 64)
		reg_val = 0x0;
	else if (cur >= 1024)
		reg_val = 0xf;
	else
		reg_val = ((cur - 64) >> 6) + 1;

	bq25896_termina_cur_set(reg_val);
}

void chg_bq25896_termina_vol_set(unsigned int vol)
{
	unsigned char reg_val = 0x17;

	CHG_BQ25896_DEBUG("%s vol =%d\n", __func__, vol);
	if (vol <= 3840)
		reg_val = 0x0;
	else if (vol >= 4608)
		reg_val = 0x3f;
	else
		reg_val = ((vol - 3840) >> 4) + 1;

	CHG_BQ25896_DEBUG("reg_val=0x%x\n", reg_val);
	bq25896_termina_vol_set(reg_val);
}

void chg_bq25896_termina_time_set(int time)
{
	unsigned char reg_val = 0x2;

	CHG_BQ25896_DEBUG("chg_bq25896_termina_time_set time =%d\n", time);
	time = time / 60;

	if (time <= 5)
		reg_val = 0x0;
	else if (time > 5 && time <= 8)
		reg_val = 0x1;
	else if (time > 8 && time <= 12)
		reg_val = 0x2;
	else
		reg_val = 0x3;

	bq25896_termina_time_set(reg_val);
}

void chg_bq25896_reset_timer(void)
{
	bq25896_reset_timer();
}

void chg_bq25896_otg_enable(int enable)
{
	bq25896_otg_enable(enable);
}

void chg_bq25896_stop_charging(unsigned int flag)
{
	bq25896_stop_charging(flag);
}

int chg_bq25896_get_charge_status(void)
{
	unsigned char chg_status = 0;

	chg_status = bq25896_get_sys_status();
	chg_status = (chg_status & CHRG_STAT_BIT) >> CHRG_STAT_SHIFT;
	if (chg_status == CHG_TERMINA_DONE) {
		CHG_BQ25896_DEBUG("bq25896  charge full\n");
		return POWER_SUPPLY_STATUS_FULL;
	} else if (chg_status == CHG_NOT_CHGING) {
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		return POWER_SUPPLY_STATUS_CHARGING;
	}
}

unsigned int chg_bq25896_get_chgcur(void)
{
	int cur = 2048;
	unsigned char reg_val = bq25896_get_chgcur();

	cur = (reg_val * 64);
	return cur;
}

int chg_bq25896_get_charge_fault(void)
{
	unsigned char reg_val = 0, fault_val = 0, ret = 0;

	reg_val = bq25896_get_fault_val();
	fault_val = (reg_val & NTC_FAULT_BIT) >> NTC_FAULT_SHIFT;

	if (fault_val == CHG_COLD_FALUT) {
		CHG_BQ25896_DEBUG("bq25896 cold fault\n");
		ret |= SPRDBAT_CHG_END_OTP_COLD_BIT;
	} else if (fault_val == CHG_HOT_FALUT) {
		CHG_BQ25896_DEBUG("bq25896 hot fault\n");
		ret |= SPRDBAT_CHG_END_OTP_OVERHEAT_BIT;
	} else {
		CHG_BQ25896_DEBUG("bq25896 no ntc fault\n");
	}

	fault_val = (reg_val & CHRG_FAULT_BIT) >> CHRG_FAULT_SHIFT;
	if (fault_val == CHG_SAFETY_TIMER_EXPIRE) {
		CHG_BQ25896_DEBUG("bq25896 safety time expire fault\n");
		ret |= SPRDBAT_CHG_END_TIMEOUT_BIT;
	}
	if (fault_val == INPUT_FAULT) {
		CHG_BQ25896_DEBUG("bq25896 input fault\n");
		ret |= SPRDBAT_CHG_END_UNSPEC;
	}

	fault_val = (reg_val & BAT_FAULT_BIT) >> BAT_FAULT_SHIFT;
	if (fault_val == CHG_VBAT_FAULT) {
		CHG_BQ25896_DEBUG("bq25896 vbat ovpfault\n");
		ret |= SPRDBAT_CHG_END_BAT_OVP_BIT;
	}

	fault_val = (reg_val & WATCHDOG_FAULT_BIT) >> WATCHDOG_FAULT_SHIFT;
	if (fault_val == CHG_WATCHDOG_FAULT) {
		CHG_BQ25896_DEBUG("bq25896 whatch dog fault\n");
		ret |= SPRDBAT_CHG_END_UNSPEC;
	}

	fault_val = (reg_val & BOOST_FAULT_BIT) >> BOOST_FAULT_SHIFT;
	if (fault_val == BOOST_FAULT) {
		CHG_BQ25896_DEBUG("bq25896 boost fault\n");
		ret |= SPRDBAT_CHG_END_UNSPEC;
	}

	if (ret)
		return ret;
	return SPRDBAT_CHG_END_NONE_BIT;

}

void chg_bq25896_set_chg_cur(unsigned int cur)
{
	unsigned char reg_val = 0;

	CHG_BQ25896_DEBUG("%s cur =%d\n", __func__, cur);
	if (cur > 3008)
		cur = 3008;
	reg_val = (cur >> 6);

	bq25896_set_chg_current(reg_val);
}
void chg_bq25896_chgr_cur_limit(unsigned int limit)
{
	bq25896_set_chg_current_limit(limit);
}

void chg_bq25896_enable_chg(void)
{
	CHG_BQ25896_DEBUG("%s enbale\n", __func__);
	bq25896_enable_chg();
}

void chg_bq25896_init(struct sprdbat_drivier_data *bdata)
{
	if (bdata == NULL)
		return;
	bat_drv_data = bdata;
	bq25896_init();
	chg_bq25896_set_vindpm(bat_drv_data->pdata->chg_end_vol_pure);
	chg_bq25896_termina_cur_set(bat_drv_data->pdata->chg_end_cur);
	chg_bq25896_termina_vol_set(bat_drv_data->pdata->chg_end_vol_pure);
}

struct sprd_ext_ic_operations bq25896_op = {
	.ic_init = chg_bq25896_init,
	.charge_start_ext = chg_bq25896_enable_chg,
	.set_charge_cur = chg_bq25896_set_chg_cur,
	.charge_stop_ext = chg_bq25896_stop_charging,
	.get_charge_cur_ext = chg_bq25896_get_chgcur,
	.get_charging_status = chg_bq25896_get_charge_status,
	.get_charging_fault = chg_bq25896_get_charge_fault,
	.timer_callback_ext = chg_bq25896_reset_timer,
	.set_termina_cur_ext = chg_bq25896_termina_cur_set,
	.set_termina_vol_ext = chg_bq25896_termina_vol_set,
	.otg_charge_ext = chg_bq25896_otg_enable,
	.set_input_cur_limit = chg_bq25896_chgr_cur_limit,
};

struct sprd_ext_ic_operations *sprd_get_bq25896_ops(void)
{
	return &bq25896_op;
}
