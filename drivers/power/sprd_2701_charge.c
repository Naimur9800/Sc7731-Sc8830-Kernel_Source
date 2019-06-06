#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/param.h>
#include <linux/stat.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include "sprd_battery.h"
#include "sprd_2701.h"

#define SPRCHG_2701_DEBUG(format, arg...) pr_info("sprd 2701: " format, ## arg)

static struct sprdbat_drivier_data *bat_drv_data;

void sprdchg_2701_set_vindpm(int vin)
{
	unsigned char reg_val = 0x1;

	SPRCHG_2701_DEBUG("%s vin =%d\n", __func__, vin);
	if (vin <= 4300)
		reg_val = 0x1;
	else if (vin >= 5180)
		reg_val = 0xf;
	else
		reg_val = (vin - 4300) / 80 + 1;

	sprd_2701_set_vindpm(reg_val);
}

void sprdchg_2701_termina_cur_set(unsigned int cur)
{
	unsigned char reg_val = 0x1;

	SPRCHG_2701_DEBUG("%s cur =%d\n", __func__, cur);
	if (cur <= 128)
		reg_val = 0x0;
	else if (cur >= 2048)
		reg_val = 0xf;
	else
		reg_val = ((cur - 128) >> 7) + 1;

	sprd_2701_termina_cur_set(reg_val);
}

void sprdchg_2701_termina_vol_set(unsigned int vol)
{
	unsigned char reg_val = 0x2a;

	SPRCHG_2701_DEBUG("%s vol =%d\n", __func__, vol);
	if (vol <= 3520)
		reg_val = 0x0;
	else if (vol >= 4400)
		reg_val = 0x3f;
	else
		reg_val = ((vol - 3520) >> 4) + 1;

	SPRCHG_2701_DEBUG("reg_val=0x%x\n", reg_val);
	sprd_2701_termina_vol_set(reg_val);
}

void sprdchg_2701_termina_time_set(int time)
{
	unsigned char reg_val = 0x1;

	SPRCHG_2701_DEBUG("sprdchg_2701_termina_time_set time =%d\n", time);
	time = time / 60;

	if (time <= 5)
		reg_val = 0x0;
	else if (time > 5 && time <= 8)
		reg_val = 0x1;
	else if (time > 8 && time <= 12)
		reg_val = 0x2;
	else
		reg_val = 0x3;

	sprd_2701_termina_time_set(reg_val);
}

void sprdchg_2701_reset_timer(void)
{
	sprd_2701_reset_timer();
}

void sprdchg_2701_otg_enable(int enable)
{
	sprd_2701_otg_enable(enable);
}

void sprdchg_2701_stop_charging(unsigned int flag)
{
	sprd_2701_stop_charging(flag);
}

int sprdchg_2701_get_charge_status(void)
{
	unsigned char chg_status = 0;

	chg_status = sprd_2701_get_sys_status();
	chg_status = (chg_status & CHG_STAT_BIT) >> CHG_STAT_SHIFT;
	if (chg_status == CHG_TERMINA_DONE) {
		SPRCHG_2701_DEBUG("2701  charge full\n");
		return POWER_SUPPLY_STATUS_FULL;
	}
	return POWER_SUPPLY_STATUS_CHARGING;
}

unsigned int sprdchg_2701_get_chgcur(void)
{
	int cur = 1450;
	unsigned char reg_val = sprd_2701_get_chgcur();

	cur = (reg_val * 64) + 512;
	return cur;
}

int sprd_chg_2701_get_charge_fault(void)
{
	unsigned char reg_val = 0, fault_val = 0, ret = 0;

	reg_val = sprd_2701_get_fault_val();
	fault_val = (reg_val & NTC_FAULT_BIT) >> NTC_FAULT_SHIFT;

	if (fault_val == CHG_COLD_FALUT) {
		SPRCHG_2701_DEBUG("2701 cold fault\n");
		ret |= SPRDBAT_CHG_END_OTP_COLD_BIT;
	} else if (fault_val == CHG_HOT_FALUT) {
		SPRCHG_2701_DEBUG("2701 hot fault\n");
		ret |= SPRDBAT_CHG_END_OTP_OVERHEAT_BIT;
	} else {
		SPRCHG_2701_DEBUG("2701 no ntc fault\n");
	}

	fault_val = (reg_val & CHG_FAULT_BIT) >> CHG_FAULT_SHIFT;
	if (fault_val == CHG_TIME_OUT) {
		SPRCHG_2701_DEBUG("2701 safety time expire fault\n");
		ret |= SPRDBAT_CHG_END_TIMEOUT_BIT;
	}
	if (fault_val == IN_DETECT_FAIL) {
		SPRCHG_2701_DEBUG("2701 in detect fail  fault\n");
		ret |= SPRDBAT_CHG_END_UNSPEC;
	}

	fault_val = (reg_val & BAT_FAULT_BIT) >> BAT_FAULT_SHIFT;
	if (fault_val == CHG_VBAT_FAULT) {
		SPRCHG_2701_DEBUG("2701 vbat ovpfault\n");
		ret |= SPRDBAT_CHG_END_BAT_OVP_BIT;
	}

	fault_val = (reg_val & WHATCH_DOG_FAULT_BIT) >> WHATCH_DOG_FAULT_SHIFT;
	if (fault_val == CHG_WHATCHDOG_FAULT) {
		SPRCHG_2701_DEBUG("2701 whatch dog fault\n");
		ret |= SPRDBAT_CHG_END_UNSPEC;
	}

	fault_val = (reg_val & SYS_FAULT_BIT) >> SYS_FAULT_SHIFT;
	if (fault_val == CHG_SYS_FAULT) {
		SPRCHG_2701_DEBUG("2701 system  fault\n");
		ret |= SPRDBAT_CHG_END_UNSPEC;
	}

	if (!ret)
		return ret;
	return SPRDBAT_CHG_END_NONE_BIT;

}

void sprdchg_2701_set_chg_cur(unsigned int cur)
{
	unsigned char reg_val = 0;

	SPRCHG_2701_DEBUG("%s cur =%d\n", __func__, cur);
	if (cur < 512)
		cur = 512;
	reg_val = (cur - 512) >> 6;

	sprd_2701_set_chg_current(reg_val);
}

void sprdchg_2701_chgr_cur_limit(unsigned int limit)
{
	sprd_2701_set_chg_current_limit(limit);
}

void sprdchg_2701_enable_chg(void)
{
	SPRCHG_2701_DEBUG("%s enbale\n", __func__);
	sprd_2701_enable_chg();
}

void sprdchg_2701_init(struct sprdbat_drivier_data *bdata)
{
	if (bdata == NULL)
		return;
	bat_drv_data = bdata;
	sprd_2701_init();
	sprdchg_2701_set_vindpm(bat_drv_data->pdata->chg_end_vol_pure);
	sprdchg_2701_termina_cur_set(bat_drv_data->pdata->chg_end_cur);
	sprdchg_2701_termina_vol_set(bat_drv_data->pdata->chg_end_vol_pure);
}

struct sprd_ext_ic_operations sprd_2701_op = {
	.ic_init = sprdchg_2701_init,
	.charge_start_ext = sprdchg_2701_enable_chg,
	.set_charge_cur = sprdchg_2701_set_chg_cur,
	.charge_stop_ext = sprdchg_2701_stop_charging,
	.get_charge_cur_ext = sprdchg_2701_get_chgcur,
	.get_charging_status = sprdchg_2701_get_charge_status,
	.get_charging_fault = sprd_chg_2701_get_charge_fault,
	.timer_callback_ext = sprdchg_2701_reset_timer,
	.set_termina_cur_ext = sprdchg_2701_termina_cur_set,
	.set_termina_vol_ext = sprdchg_2701_termina_vol_set,
	.otg_charge_ext = sprdchg_2701_otg_enable,
	.set_input_cur_limit = sprdchg_2701_chgr_cur_limit,
};

struct sprd_ext_ic_operations *sprd_get_2701_ops(void)
{
	return &sprd_2701_op;
}
