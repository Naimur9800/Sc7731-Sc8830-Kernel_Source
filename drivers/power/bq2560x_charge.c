/* SPDX-License-Identifier: GPL-2.0
 *
 * Charger device driver for BQ2560X
 *
 * Copyright (c) 2018 UNISOC.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/param.h>
#include <linux/stat.h>
#include <linux/string.h>
#include "sprd_battery.h"
#include "bq2560x.h"

static struct sprdbat_drivier_data *bat_drv_data;

static void chg_bq2560x_set_vindpm(int vin)
{
	u8 reg_val;

	if (vin < 3900)
		reg_val = 0x0;
	else if (vin > 5400)
		reg_val = 0x0f;
	else
		reg_val = (vin - 3900) / 100;

	bq2560x_set_vindpm(reg_val);
}

static void chg_bq2560x_termina_cur_set(u32 cur)
{
	u8 reg_value;

	if (cur <= 60)
		reg_value = 0x0;
	else if (cur >= 480)
		reg_value = 0x8;
	else
		reg_value = (cur - 60) / 60;

	bq2560x_termina_cur_set(reg_value);
}

static void chg_bq2560x_termina_vol_set(u32 vol)
{
	u8 reg_value;

	if (vol < REG04_VREG_BASE)
		vol = REG04_VREG_BASE;
	else if (vol > REG04_VREG_MAX)
		vol = REG04_VREG_MAX;

	reg_value = (vol - REG04_VREG_BASE) / REG04_VREG_LSB;
	bq2560x_termina_vol_set(reg_value);
}

static void chg_bq2560x_reset_timer(void)
{
	bq2560x_reset_timer();
}

static void chg_bq2560x_otg_enable(int enable)
{
	bq2560x_otg_enable(enable);
}

static void chg_bq2560x_stop_charging(u32 flag)
{
	bq2560x_stop_charging(flag);
}

static int chg_bq2560x_get_charge_status(void)
{
	u8 chg_status;

	chg_status = bq2560x_get_sys_status();

	if (chg_status == REG08_CHRG_STAT_IDLE)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	else if (chg_status == REG08_CHRG_STAT_FASTCHG)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_CHARGING;
}

static u32 chg_bq2560x_get_chgcur(void)
{
	int cur;
	u8 reg_val = bq2560x_get_chgcur();

	cur = (reg_val * 60);
	return cur;
}

static int chg_bq2560x_get_charge_fault(void)
{
	u8 reg_val, fault_val;
	u8 ret = 0;

	reg_val = bq2560x_get_fault_val();

	fault_val = (reg_val & REG09_FAULT_WDT_MASK) >> REG09_FAULT_WDT_SHIFT;

	if (fault_val == REG09_FAULT_WDT)
		ret |= SPRDBAT_CHG_END_UNSPEC;

	fault_val = (reg_val & REG09_FAULT_BOOST_MASK) >>
			REG09_FAULT_BOOST_SHIFT;
	if (fault_val == REG09_FAULT_BOOT)
		ret |= SPRDBAT_CHG_END_OVP_BIT;

	fault_val = (reg_val & REG09_FAULT_BAT_MASK) >> REG09_FAULT_BAT_SHIFT;
	if (fault_val == REG09_FAULT_BAT_OVP)
		ret |= SPRDBAT_CHG_END_BAT_OVP_BIT;

	fault_val = (reg_val & REG09_FAULT_CHRG_MASK) >> REG09_FAULT_CHRG_SHIFT;
	switch (fault_val) {
	case REG09_FAULT_CHRG_TIMER:
		ret |= SPRDBAT_CHG_END_TIMEOUT_BIT;
		break;
	case REG09_FAULT_CHRG_THERMAL:
		ret |= SPRDBAT_CHG_END_UNSPEC;
		break;
	case REG09_FAULT_CHRG_INPUT:
		ret |= SPRDBAT_CHG_END_UNSPEC;
		break;
	default:
		ret |= SPRDBAT_CHG_END_NONE_BIT;
		break;
	}

	return ret;
}

static u8 chg_bq2560x_cur2reg(u32 cur)
{
	u8 reg_val;

	if (cur > 3000) {
		reg_val = 0x32;
	} else if (cur < 0) {
		reg_val = 0x0;
	} else {
		reg_val = cur / REG02_ICHG_LSB;
		reg_val &= REG02_ICHG_MASK;
	}

	return reg_val;
}

static void chg_bq2560x_set_chg_cur(u32 cur)
{
	u8 reg_val;

	reg_val = chg_bq2560x_cur2reg(cur);
	bq2560x_set_chg_current(reg_val);
}

static void chg_bq2560x_chgr_cur_limit(u32 limit)
{
	u8 reg_value;

	if (limit > 3200)
		limit = 3200;

	reg_value = limit / REG00_IINLIM_BASE;
	bq2560x_set_chg_current_limit(reg_value);
}

static void chg_bq2560x_enable_chg(void)
{
	bq2560x_enable_chg();
}

static void chg_bq2560x_init(struct sprdbat_drivier_data *bdata)
{
	if (bdata == NULL)
		return;
	bat_drv_data = bdata;
	bq2560x_init();
	chg_bq2560x_set_vindpm(bat_drv_data->pdata->chg_end_vol_pure);
	chg_bq2560x_termina_cur_set(bat_drv_data->pdata->chg_end_cur);
	chg_bq2560x_termina_vol_set(bat_drv_data->pdata->chg_end_vol_pure);
}

static void chg_bq2560x_set_ship_mode(int enable)
{
	if (enable == 0x0)
		bq2560x_set_ship_mode();
	else
		pr_info("ship_mode counld not parse input value: %d\n", enable);
}

static int chg_bq2560x_get_ship_mode(void)
{
	int ret = 0;

	ret = bq2560x_get_ship_mode();

	return ret;
}

static int chg_bq2560x_get_enable_status(void)
{
	int ret = 0;

	ret = bq2560x_get_charge_status();

	return ret;
}

struct sprd_ext_ic_operations bq2560x_op = {
	.ic_init = chg_bq2560x_init,
	.charge_start_ext = chg_bq2560x_enable_chg,
	.set_charge_cur = chg_bq2560x_set_chg_cur,
	.charge_stop_ext = chg_bq2560x_stop_charging,
	.get_charge_cur_ext = chg_bq2560x_get_chgcur,
	.get_charging_status = chg_bq2560x_get_charge_status,
	.get_charging_fault = chg_bq2560x_get_charge_fault,
	.timer_callback_ext = chg_bq2560x_reset_timer,
	.set_termina_cur_ext = chg_bq2560x_termina_cur_set,
	.set_termina_vol_ext = chg_bq2560x_termina_vol_set,
	.otg_charge_ext = chg_bq2560x_otg_enable,
	.set_input_cur_limit = chg_bq2560x_chgr_cur_limit,
	.set_ship_mode =  chg_bq2560x_set_ship_mode,
	.get_ship_mode =  chg_bq2560x_get_ship_mode,
	.get_charge_status = chg_bq2560x_get_enable_status,
};

struct sprd_ext_ic_operations *sprd_get_bq2560x_ops(void)
{
	return &bq2560x_op;
}
