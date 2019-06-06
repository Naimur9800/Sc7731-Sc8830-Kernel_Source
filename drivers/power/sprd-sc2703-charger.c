/*
 * Copyright (C) 2018 Spreadtrum Communications Inc.
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

#include <linux/kernel.h>
#include "sprd_battery.h"
#include "sc2703-charger.h"

static struct sprdbat_drivier_data *bat_drv_data;

static void sprdchg_2703_termina_cur_set(unsigned int cur)
{
}

static void sprdchg_2703_termina_vol_set(unsigned int vol)
{
	sc2703_termina_vol_set(vol);
}

static void sprdchg_2703_reset_timer(void)
{
	sc2703_charger_reset_timer();
}

static void sprdchg_2703_otg_enable(int enable)
{
	sc2703_charger_otg_enable(enable);
}

static void sprdchg_2703_stop_charging(unsigned int flag)
{
	sc2703_charger_stop(flag);
}

static int sprdchg_2703_get_charge_status(void)
{
	return sc2703_charger_status_get();
}

static unsigned int sprdchg_2703_get_chgcur(void)
{
	return sc2703_charger_cc_current_get();
}

static int sprdchg_2703_get_charge_fault(void)
{
	u32 fault_val;
	int ret;

	ret = sc2703_get_fault_val(&fault_val);
	if (ret)
		return SPRDBAT_CHG_END_NONE_BIT;

	if (fault_val == SC2703_E_VBAT_OV_MASK)
		return SPRDBAT_CHG_END_BAT_OVP_BIT;

	return SPRDBAT_CHG_END_NONE_BIT;
}

static void sprdchg_2703_set_chg_cur(unsigned int cur)
{
	sc2703_charger_cc_current_set(cur);
}

static void sprdchg_2703_chgr_cur_limit(unsigned int limit)
{
	sc2703_charger_input_current_set(limit);
}

static void sprdchg_2703_enable_chg(void)
{
	sc2703_charger_start();
}

static void sprdchg_2703_init(struct sprdbat_drivier_data *bdata)
{
	if (!bdata)
		return;
	bat_drv_data = bdata;
	sc2703_charger_ic_init();
	sc2703_charger_vindpm_set(4300);
	sprdchg_2703_termina_vol_set(bat_drv_data->pdata->chg_end_vol_pure);
}

static int sprdchg_2703_is_support_fchg(void)
{
	return sc2703_is_support_fchg();
}

struct sprd_ext_ic_operations sprd_2703_op = {
	.ic_init = sprdchg_2703_init,
	.charge_start_ext = sprdchg_2703_enable_chg,
	.set_charge_cur = sprdchg_2703_set_chg_cur,
	.charge_stop_ext = sprdchg_2703_stop_charging,
	.get_charge_cur_ext = sprdchg_2703_get_chgcur,
	.get_charging_status = sprdchg_2703_get_charge_status,
	.get_charging_fault = sprdchg_2703_get_charge_fault,
	.timer_callback_ext = sprdchg_2703_reset_timer,
	.set_termina_cur_ext = sprdchg_2703_termina_cur_set,
	.set_termina_vol_ext = sprdchg_2703_termina_vol_set,
	.otg_charge_ext = sprdchg_2703_otg_enable,
	.set_input_cur_limit = sprdchg_2703_chgr_cur_limit,
	.support_fchg = sprdchg_2703_is_support_fchg,
};

struct sprd_ext_ic_operations *sprd_get_2703_ops(void)
{
	return &sprd_2703_op;
}
