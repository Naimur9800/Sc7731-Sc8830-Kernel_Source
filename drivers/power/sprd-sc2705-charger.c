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

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/param.h>
#include <linux/stat.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include "sprd_battery.h"
#include "sc2705-charger.h"

#define SPRCHG_2705_DEBUG(format, arg...) pr_info("sprd 2705: " format, ## arg)

static struct sprdbat_drivier_data *bat_drv_data;

static void sprdchg_2705_termina_cur_set(unsigned int cur)
{

}

static void sprdchg_2705_termina_vol_set(unsigned int vol)
{
	SPRCHG_2705_DEBUG("%s,vol:%d\n", __func__, vol);
	sc2705_termina_vol_set(vol);
}

static void sprdchg_2705_reset_timer(void)
{
	sc2705_charger_reset_timer();
	SPRCHG_2705_DEBUG("%s\n", __func__);
}

static void sprdchg_2705_otg_enable(int enable)
{
	SPRCHG_2705_DEBUG("%s,enable:%d\n", __func__, enable);
	sc2705_charger_otg_enable(enable);
}

static void sprdchg_2705_stop_charging(unsigned int flag)
{
	SPRCHG_2705_DEBUG("%s,flag:%d\n", __func__, flag);
	sc2705_charger_stop(flag);
}

static int sprdchg_2705_get_charge_status(void)
{
	return sc2705_charger_status_get();
}

static unsigned int sprdchg_2705_get_chgcur(void)
{
	return sc2705_charger_cc_current_get();
}

static int sprdchg_2705_get_charge_fault(void)
{
	return SPRDBAT_CHG_END_NONE_BIT;
}

static void sprdchg_2705_set_chg_cur(unsigned int cur)
{
	SPRCHG_2705_DEBUG("%s,cur:%d\n", __func__, cur);
	sc2705_charger_cc_current_set(cur);
}

static void sprdchg_2705_chgr_cur_limit(unsigned int limit)
{
	SPRCHG_2705_DEBUG("%s,limit:%d\n", __func__, limit);
	sc2705_charger_input_current_set(limit);
}

static void sprdchg_2705_enable_chg(void)
{
	sc2705_charger_start();
}

static void sprdchg_2705_init(struct sprdbat_drivier_data *bdata)
{
	if (bdata == NULL)
		return;
	SPRCHG_2705_DEBUG("%s\n", __func__);
	bat_drv_data = bdata;
	sc2705_charger_ic_init();
	sc2705_charger_vindpm_set(4300);
	sprdchg_2705_termina_vol_set(bat_drv_data->pdata->chg_end_vol_pure);
}

struct sprd_ext_ic_operations sprd_2705_op = {
	.ic_init = sprdchg_2705_init,
	.charge_start_ext = sprdchg_2705_enable_chg,
	.set_charge_cur = sprdchg_2705_set_chg_cur,
	.charge_stop_ext = sprdchg_2705_stop_charging,
	.get_charge_cur_ext = sprdchg_2705_get_chgcur,
	.get_charging_status = sprdchg_2705_get_charge_status,
	.get_charging_fault = sprdchg_2705_get_charge_fault,
	.timer_callback_ext = sprdchg_2705_reset_timer,
	.set_termina_cur_ext = sprdchg_2705_termina_cur_set,
	.set_termina_vol_ext = sprdchg_2705_termina_vol_set,
	.otg_charge_ext = sprdchg_2705_otg_enable,
	.set_input_cur_limit = sprdchg_2705_chgr_cur_limit,
};

struct sprd_ext_ic_operations *sprd_get_2705_ops(void)
{
	return &sprd_2705_op;
}

