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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/hrtimer.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/usb.h>
#include <linux/thermal.h>
#include "sprd_charge_helper.h"

#define SPRDBAT_DEBUG(format, arg...) pr_info("sprdbat: " format, ## arg)
#define NORMAL_TEMP 200
#define BAT_NTC_THRESHOLD		50
#define CHGR_TEMP_BUFF_CNT	5
#define VBAT_OVP_CNT_THRESHOLD	20
#define VBAT_OVP_THRESHOLD	50

static int jeita_debug = 200;
static int jeita_debug_enable;
static int adjust_chg_flag;
static __s64 trickle_s_time;
static int trickle_time;
static int current_buff[CUR_BUFF_CNT];
static int chg_vol_buff[VOL_BUFF_CNT] = { 5000, 5000, 5000 };

static struct sprdbat_drivier_data *sprdbat_data;
const struct sprd_ext_ic_operations *sprd_ext_ic_op;
static struct sprd_chg_timer_operations *sprdchg_timer_op;
static struct sprd_fchg_operations *sprd_fchg_op;
extern int enable_to_shutdown;
extern bool charger_policy_get_status(void);
static void sprdbat_change_module_state(uint32_t event);
static int sprdbat_stop_charge(void);
static void sprdbat_charge_prepare(void);
static int sprdbat_fchg_detect(void);

static int sprdbat_get_avgval_from_buff(int value, int *buf, int count,
					int type)
{
	static int cur_pointer, vol_pointer;
	int i = 0, sum = 0;

	SPRDBAT_DEBUG("sprdbat_get_val_from_buff value=%d,type =%d\n", value,
		      type);
	if (type) {
		if (cur_pointer >= count)
			cur_pointer = 0;
		buf[cur_pointer++] = value;
	} else {
		if (vol_pointer >= count)
			vol_pointer = 0;
		buf[vol_pointer++] = value;
	}
	for (i = 0; i < count; i++)
		sum += buf[i];
	return (sum / count);
}

static void sprdbat_clear_buff(int *buf, int count, int value)
{
	int i = 0;

	SPRDBAT_DEBUG("sprdbat_clear_buff\n");
	for (i = 0; i < count; i++)
		buf[i] = value;
}

void sprdbat_chgcurrent_adjust(int state)
{
	int state_cur;

	if (sprdbat_data->chg_cur_adjust_min >
		sprdbat_data->chg_cur_adjust_max)
		sprdbat_data->chg_cur_adjust_min =
			sprdbat_data->chg_cur_adjust_max;

	state_cur = sprdbat_data->chg_cur_adjust_min;

	if (!state)
		state_cur = sprdbat_data->chg_cur_adjust_max;

	SPRDBAT_DEBUG
		("enter state_cur = %d, state:%d\n", state_cur, state);
	if (sprdbat_data->last_temp_status == 0 ||
	    sprdbat_data->last_temp_status >=
	    sprdbat_data->pdata->jeita_tab_size) {
		SPRDBAT_DEBUG("error last_temp_status\n");
		return;

	} else {
		if (state_cur >
		    sprdbat_data->pdata->jeita_tab[sprdbat_data->
						   last_temp_status].y)
			state_cur =
			    sprdbat_data->pdata->jeita_tab[sprdbat_data->
							   last_temp_status].y;
	}
	if (state_cur > sprdbat_data->bat_info.chg_current_type)
		state_cur = sprdbat_data->bat_info.chg_current_type;

	SPRDBAT_DEBUG("state_cur =%d,set cur\n", state_cur);
	if (sprdbat_data->bat_info.chg_stop_flags == SPRDBAT_CHG_END_NONE_BIT) {
		sprd_ext_ic_op->set_charge_cur(state_cur);
	}
}

static int sprdbat_is_chg_timeout(void)
{
	struct timespec64 cur_time;

	cur_time = ktime_to_timespec64(ktime_get_boottime());
	if (cur_time.tv_sec - sprdbat_data->bat_info.chg_start_time >
	    sprdbat_data->bat_info.chg_this_timeout)
		return 1;
	else
		return 0;
}

static int sprdbat_get_status_from_tab(int size,
				       struct sprdbat_jeita_table_data *buf,
				       int data)
{
	int jeita_tab_size = sprdbat_data->pdata->jeita_tab_size;
	static int jeita_status;
	int i;

	for (i = jeita_tab_size - 1; i >= 0; i--) {
		if ((data >= sprdbat_data->pdata->jeita_tab[i].x && i > 0) ||
			(data > sprdbat_data->pdata->jeita_tab[i].x && i == 0)) {
			break;
		}
	}

	switch (i) {
	case 4:
		jeita_status = STATUS_ABOVE_T4;
		break;
	case 3:
		if (jeita_status != STATUS_ABOVE_T4 ||
			data <= sprdbat_data->pdata->jeita_tab[4].w)
			jeita_status = STATUS_T3_TO_T4;
		break;
	case 2:
		if ((jeita_status != STATUS_T3_TO_T4 ||
			data <= sprdbat_data->pdata->jeita_tab[3].w) &&
			(jeita_status != STATUS_T1_TO_T2 ||
			data >= sprdbat_data->pdata->jeita_tab[2].w))
			jeita_status = STATUS_T2_TO_T3;
		break;
	case 1:
		if (jeita_status != STATUS_T0_TO_T1 ||
			data >= sprdbat_data->pdata->jeita_tab[1].w)
			jeita_status = STATUS_T1_TO_T2;
		break;
	case 0:
		if (jeita_status != STATUS_BELOW_T0 ||
			data >= sprdbat_data->pdata->jeita_tab[0].w)
			jeita_status = STATUS_T0_TO_T1;
		break;
	default:
		jeita_status = STATUS_BELOW_T0;
		break;
	}

	SPRDBAT_DEBUG("jeita_state=%d\n", jeita_status);
	return jeita_status;
}

static void sprdbat_current_adjust_by_status(uint32_t status)
{
	int cccv;
	int target_cur;
	int jeita_tab_size = sprdbat_data->pdata->jeita_tab_size;

	if (status > jeita_tab_size)
		status = jeita_tab_size;
	cccv = sprdbat_data->pdata->jeita_tab[status].z;
	if (jeita_tab_size != status)
		target_cur = sprdbat_data->pdata->jeita_tab[status].y;
	else
		target_cur = 0;
	if (target_cur > sprdbat_data->bat_info.chg_current_type)
		target_cur = sprdbat_data->bat_info.chg_current_type;
	else
		SPRDBAT_DEBUG("sprdbat_current_adjust_by_status\n");

	SPRDBAT_DEBUG("status =%d cccv=%d,target_cur=%d\n", status, cccv,
		      target_cur);
	if (status == 0) {
		if (sprdbat_data->bat_info.chg_stop_flags &
			SPRDBAT_CHG_END_OTP_COLD_BIT) {
			SPRDBAT_DEBUG("bat is  already cold return\n");
		} else {
			SPRDBAT_DEBUG("bat turn cold status,stop charge\n");
			sprdbat_change_module_state(SPRDBAT_OTP_COLD_STOP_E);
			sprdbat_data->stop_charge();
		}
		return;
	} else if (jeita_tab_size == status) {
		if (sprdbat_data->bat_info.chg_stop_flags &
			SPRDBAT_CHG_END_OTP_OVERHEAT_BIT) {
			SPRDBAT_DEBUG("bat is  already hot return\n");
		} else {
			SPRDBAT_DEBUG("bat turn hot status,stop charge\n");
			sprdbat_change_module_state
				(SPRDBAT_OTP_OVERHEAT_STOP_E);
			sprdbat_data->stop_charge();
		}
		return;
	}
		SPRDBAT_DEBUG("status change set cccv and cur\n");
		if (sprdbat_data->bat_info.chg_stop_flags &
			SPRDBAT_CHG_END_OTP_COLD_BIT) {
			sprdbat_change_module_state(SPRDBAT_OTP_COLD_RESTART_E);
		/* sprdbat_data->start_charge(); */
		} else if (sprdbat_data->bat_info.chg_stop_flags &
			SPRDBAT_CHG_END_OTP_OVERHEAT_BIT) {
			sprdbat_change_module_state
			    (SPRDBAT_OTP_OVERHEAT_RESTART_E);
		/* sprdbat_data->start_charge(); */
		} else if (sprdbat_data->bat_info.chg_stop_flags !=
		    SPRDBAT_CHG_END_NONE_BIT) {
			SPRDBAT_DEBUG("can't change cur ,something wrong\n");
			return;
		}

	if (cccv > sprdbat_data->pdata->chg_end_vol_pure)
		cccv = sprdbat_data->pdata->chg_end_vol_pure;
	sprd_ext_ic_op->set_termina_vol_ext(cccv);
	if (adjust_chg_flag && sprdbat_data->chg_cur_adjust_min < target_cur)
		target_cur = sprdbat_data->chg_cur_adjust_min;
	sprd_ext_ic_op->set_charge_cur(target_cur);
	sprd_ext_ic_op->charge_start_ext();
}

static void sprdbat_temp_monitor(void)
{
	int chg_cur = sprd_ext_ic_op->get_charge_cur_ext();
	uint32_t cur_temp_status = sprdbat_data->cur_temp_status;

	SPRDBAT_DEBUG
	    ("sprdbat_temp_monitor cur_temp =%d,chg_cur=%d,jeita_debug=%d\n",
	     sprdbat_data->bat_info.cur_temp, chg_cur, jeita_debug);

	cur_temp_status = sprdbat_get_status_from_tab(
		sprdbat_data->pdata->jeita_tab_size,
		sprdbat_data->pdata->jeita_tab,
		sprdbat_data->bat_info.cur_temp);
	sprdbat_data->cur_temp_status = cur_temp_status;

	if (cur_temp_status > sprdbat_data->last_temp_status) {
		sprdbat_data->temp_down_trigger_cnt = 0;
		SPRDBAT_DEBUG("temp really up\n");
		sprdbat_data->temp_up_trigger_cnt++;
		if (sprdbat_data->temp_up_trigger_cnt >
		    SPRDBAT_TEMP_TRIGGER_TIMES) {
			sprdbat_current_adjust_by_status(cur_temp_status);
			sprdbat_data->last_temp_status = cur_temp_status;
		}
	} else if (cur_temp_status < sprdbat_data->last_temp_status) {
		sprdbat_data->temp_up_trigger_cnt = 0;
		SPRDBAT_DEBUG("temp really down\n");
		sprdbat_data->temp_down_trigger_cnt++;
		if (sprdbat_data->temp_down_trigger_cnt >
		    SPRDBAT_TEMP_TRIGGER_TIMES) {
			sprdbat_current_adjust_by_status(cur_temp_status);
			sprdbat_data->last_temp_status = cur_temp_status;
		}
	} else {
		sprdbat_data->temp_up_trigger_cnt = 0;
		sprdbat_data->temp_down_trigger_cnt = 0;
	}
}

static int sprdbat_get_avg_chgr_temp(int temp, bool init)
{
	static u32 p;
	static int temp_buff[CHGR_TEMP_BUFF_CNT];
	int i, sum = 0;

	if (unlikely(init)) {
		for (i = 0; i < CHGR_TEMP_BUFF_CNT; i++)
			temp_buff[i] = temp;
		return temp;
	}

	if (p >= CHGR_TEMP_BUFF_CNT)
		p = 0;
	temp_buff[p++] = temp;

	for (i = 0; i < CHGR_TEMP_BUFF_CNT; i++)
		sum += temp_buff[i];

	return sum / CHGR_TEMP_BUFF_CNT;
}

static void sprdbat_chgr_temp_monitor(void)
{
	int temp;
	struct thermal_zone_device *tz = sprdbat_data->pdata->chgr_thmz;

	if (!tz)
		return;

	if (!thermal_zone_get_temp(tz, &temp)) {
		temp /= 100;
		SPRDBAT_DEBUG("charger cur temp:%d,%s\n", temp, tz->type);
		sprdbat_data->bat_info.chgr_temp
		    = sprdbat_get_avg_chgr_temp(temp, false);
	} else {
		SPRDBAT_DEBUG("chgr temp fail:%s\n", tz->type);
		return;
	}

	SPRDBAT_DEBUG("charger avg temp:%d\n",
		sprdbat_data->bat_info.chgr_temp);
	if (sprdbat_data->bat_info.chg_stop_flags
	    & SPRDBAT_CHG_END_CHGR_OTP_BIT) {
	    if (sprdbat_data->bat_info.chgr_temp
		    < sprdbat_data->pdata->chgr_otp_restart) {
			SPRDBAT_DEBUG("charger otp restart\n");
			sprdbat_change_module_state(SPRDBAT_CHGR_OTP_START_E);
			sprdbat_data->start_charge();
		}
	} else {
	    if (sprdbat_data->bat_info.chgr_temp
		    > sprdbat_data->pdata->chgr_otp_stop) {
			SPRDBAT_DEBUG("charger otp stop\n");
			sprdbat_change_module_state(SPRDBAT_CHGR_OTP_E);
			sprdbat_data->stop_charge();
		}
	}
}

static void sprdbat_chg_timeout_monitor(void)
{
	SPRDBAT_DEBUG("sprdbat_chg_timeout_monitor enter\n");
	if (sprdbat_data->bat_info.chg_stop_flags &
		SPRDBAT_CHG_END_TIMEOUT_BIT) {
		SPRDBAT_DEBUG("sprdbat_chg_timeout_monitor recharge\n");
		sprdbat_change_module_state(SPRDBAT_CHG_TIMEOUT_RESTART_E);
		sprdbat_data->start_charge();
	}
	if (sprdbat_data->bat_info.chg_stop_flags == SPRDBAT_CHG_END_NONE_BIT) {
		if (sprdbat_is_chg_timeout()) {
			SPRDBAT_DEBUG
			    ("sprdbat_chg_timeout_monitor chg timeout\n");
			if (sprdbat_data->bat_info.vbat_ocv >
			    sprdbat_data->pdata->rechg_vol) {
				sprdbat_change_module_state(SPRDBAT_CHG_FULL_E);
				sprdbat_data->stop_charge();
			} else {
				sprdbat_data->bat_info.chg_this_timeout =
				    sprdbat_data->pdata->chg_rechg_timeout;
				sprdbat_change_module_state
				    (SPRDBAT_CHG_TIMEOUT_E);
				sprdbat_data->stop_charge();
			}
		}
	}
}

static void sprdbat_chg_ovp_monitor(void)
{
	int ovp_restart, ovp_stop;

	if (sprdbat_data->fchg_det) {
		ovp_restart = sprdbat_data->pdata->fchg_ovp_restart;
		ovp_stop = sprdbat_data->pdata->fchg_ovp_stop;
	} else {
		ovp_restart = sprdbat_data->pdata->ovp_restart;
		ovp_stop = sprdbat_data->pdata->ovp_stop;
	}
	SPRDBAT_DEBUG("%s chg_vol = %d,ovp_stop =%d,ovp_restart=%d\n",
		__func__, sprdbat_data->bat_info.avg_chg_vol,
		ovp_stop, ovp_restart);

	if (sprdbat_data->bat_info.chg_stop_flags & SPRDBAT_CHG_END_OVP_BIT) {
		if (sprdbat_data->bat_info.avg_chg_vol <= ovp_restart) {
			SPRDBAT_DEBUG("charge vol low restart chg\n");
			sprdbat_change_module_state(SPRDBAT_OVI_RESTART_E);
			sprdbat_data->start_charge();
		} else {
			SPRDBAT_DEBUG("sprdbat_chg_ovp_monitor ovp return ");
		}
	} else if (sprdbat_data->bat_info.avg_chg_vol >= ovp_stop) {
		SPRDBAT_DEBUG("charge vol is too high\n");
		sprdbat_change_module_state(SPRDBAT_OVI_STOP_E);
		sprdbat_data->stop_charge();
	}

	if (sprdbat_data->fchg_det) {
		/*if vbus vol <(vbus - 2000mv),exit*/
		uint32_t fchg_l = sprdbat_data->pdata->fchg_vol - 2000;

		if (sprdbat_data->bat_info.avg_chg_vol <= fchg_l) {
			SPRDBAT_DEBUG("fchg_l_low:%d\n", fchg_l);
			sprdbat_data->fchg_det = 0;
			if ((sprd_fchg_op != NULL)
				&& (sprd_fchg_op->fchg_deinit))
				sprd_fchg_op->fchg_deinit();
			sprdbat_charge_prepare();
			sprdbat_data->start_charge();
			power_supply_changed(sprdbat_data->battery);
		}
	}
}

static inline bool sprdbat_is_force_5v(void)
{
	uint32_t vol = sprdbat_data->bat_info.vchg_vol_max;

	return (vol == 0 || vol > 5000) ? false : true;
}

static void sprdbat_chg_vol_prop_set(uint32_t vol)
{
	mutex_lock(&sprdbat_data->lock);
	sprdbat_data->bat_info.vchg_vol_max = vol;
	if (POWER_SUPPLY_STATUS_DISCHARGING !=
	    sprdbat_data->bat_info.module_state) {
	    if (sprdbat_is_force_5v()) {
			SPRDBAT_DEBUG("exit fast charge!\n");
			if ((sprd_fchg_op != NULL)
			    && (sprd_fchg_op->fchg_deinit)
			    && sprdbat_data->fchg_det) {
				sprdbat_data->fchg_det = 0;
				sprd_fchg_op->fchg_deinit();
			}
			sprdbat_charge_prepare();
			sprdbat_data->start_charge();
		} else {
			if (!sprdbat_data->fchg_det) {
				SPRDBAT_DEBUG("try entry fast charge!\n");
				sprdbat_fchg_detect();
				sprdbat_charge_prepare();
				sprdbat_data->start_charge();
			}
		}
	}
	mutex_unlock(&sprdbat_data->lock);
}

void sprd_extic_otg_power(int enable)
{
	/* disable bc1.2 while enable otg, vice versa */
	sprdchg_bc1p2_disable(enable);
	if (sprd_ext_ic_op->otg_charge_ext != NULL)
		sprd_ext_ic_op->otg_charge_ext(enable);
}
EXPORT_SYMBOL_GPL(sprd_extic_otg_power);

void sprdbat_register_ext_ops(const struct sprd_ext_ic_operations *ops)
{
	sprd_ext_ic_op = ops;
}
EXPORT_SYMBOL_GPL(sprdbat_register_ext_ops);

void sprdbat_unregister_ext_ops(void)
{
	sprd_ext_ic_op = NULL;
}
EXPORT_SYMBOL_GPL(sprdbat_unregister_ext_ops);

void sprdbat_register_timer_ops(struct sprd_chg_timer_operations *ops)
{
	sprdchg_timer_op = ops;
}
EXPORT_SYMBOL_GPL(sprdbat_register_timer_ops);

void sprdbat_unregister_timer_ops(void)
{
	sprdchg_timer_op = NULL;
}
EXPORT_SYMBOL_GPL(sprdbat_unregister_timer_ops);

void sprdbat_register_fchg_ops(struct sprd_fchg_operations *ops)
{
	sprd_fchg_op = ops;
}
EXPORT_SYMBOL_GPL(sprdbat_register_fchg_ops);

void sprdbat_unregister_fchg_ops(void)
{
	sprd_fchg_op = NULL;
}
EXPORT_SYMBOL_GPL(sprdbat_unregister_fchg_ops);

int sprdbat_interpolate(int x, int n, struct sprdbat_table_data *tab)
{
	int index;
	int y;

	if (x >= tab[0].x)
		y = tab[0].y;
	else if (x <= tab[n - 1].x)
		y = tab[n - 1].y;
	else {
		/*  find interval */
		for (index = 1; index < n; index++)
			if (x > tab[index].x)
				break;
		/*  interpolate */
		y = (tab[index - 1].y - tab[index].y) * (x - tab[index].x)
		    * 2 / (tab[index - 1].x - tab[index].x);
		y = (y + 1) / 2;
		y += tab[index].y;
	}
	return y;
}

uint32_t sprd_get_vbat_voltage(void)
{
	return sprdbat_read_vbat_vol();
}

bool sprdbat_is_support_batdet(void)
{
	if (sprdbat_data == NULL) {
		WARN_ONCE(1, "%s: Not init\n", __func__);
		return false;
	}

	if (sprdbat_data->gpio_vbat_detect > 0)
		return true;
	else
		return false;
}

bool sprdbat_is_bat_present(void)
{
	if (sprdbat_data == NULL) {
		WARN_ONCE(1, "%s: Not init\n", __func__);
		return true;
	}

	return sprdbat_data->bat_info.bat_present;
}

static int sprdbat_ac_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct sprdbat_drivier_data *data = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (likely(psy->desc->type == POWER_SUPPLY_TYPE_MAINS))
			val->intval = data->bat_info.ac_online ? 1 : 0;
		else
			ret = -EINVAL;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = data->bat_info.vchg_vol_max * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval =
			sprdbat_data->bat_info.chg_current_type_limit * 1000;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int sprdbat_ac_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		{
			int limit = val->intval / 1000;

			SPRDBAT_DEBUG("set chgr vol:%d", val->intval);
			sprdbat_chg_vol_prop_set(limit);
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int sprdbat_ac_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}


static int sprdbat_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct sprdbat_drivier_data *data = power_supply_get_drvdata(psy);
	int ret = 0;
	int usb_status = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->bat_info.usb_online ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		usb_status = usb_charger_get_state(sprdbat_data->usb_charger);
		if (usb_status == USB_CHARGER_PRESENT)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval =
			sprdbat_data->bat_info.chg_current_type_limit * 1000;
		break;
	/* usb charge voltage is 5000mv */
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = 5000 * 1000;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#define CUTOFF_VOLTAGE_MV 3400
static int sprdbat_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct sprdbat_drivier_data *data = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = data->bat_info.module_state;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = data->bat_info.bat_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = data->bat_info.bat_present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = data->bat_info.capacity;
		if (!enable_to_shutdown || data->bat_info.vbat_vol >= CUTOFF_VOLTAGE_MV) {
			if (data->bat_info.capacity == 0) {
				val->intval = 1;
				SPRDBAT_DEBUG("force report soc 1\n");
			}
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = data->bat_info.vbat_vol * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = data->bat_info.vbat_ocv * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = data->bat_info.user_set_input_cur_max;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = data->bat_info.input_cur_limit * 1000;
		SPRDBAT_DEBUG("get chgr limit:%d", val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = data->bat_info.cur_temp;
		break;
	/* calculate battery remaining capacity UAH */
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = (sprdbat_data->bat_info.capacity *
			sprdbat_data->pdata->cnom * 1000)
			/ 100;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = sprdbat_data->pdata->cnom * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = sprdbat_data->bat_info.bat_current * (-1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = sprdbat_data->bat_info.bat_current * (-1000);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = sprdbat_data->bat_info.bat_present;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval =
			sprdbat_data->bat_info.chg_current_type_limit * (-1000);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = sprdbat_data->pdata->chg_end_vol_pure * 1000;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		if (sprd_ext_ic_op->get_charge_status == NULL)
			val->intval = -EPERM;
		else
			val->intval = sprd_ext_ic_op->get_charge_status();
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		if (sprdbat_data->bat_info.charging_enabled) {
			sprdbat_data->bat_info.usb_input_current = data->bat_info.input_cur_limit * 1000;
		}

		if (sprdbat_data->bat_info.charging_enabled == 0) {
			if (sprd_ext_ic_op->set_input_cur_limit
				&& (data->bat_info.input_cur_limit != 0)) {
				sprdbat_data->bat_info.charging_enabled = 1;
				SPRDBAT_DEBUG("it happened the usb plug and in\n");
				sprdbat_data->bat_info.usb_input_current = data->bat_info.input_cur_limit * 1000;
			}
		}
		val->intval = sprdbat_data->bat_info.charging_enabled;
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		if (sprd_ext_ic_op->get_ship_mode == NULL)
			val->intval = -EPERM;
		else
			val->intval = sprd_ext_ic_op->get_ship_mode();
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = sprdbat_data->pdata->batt_full_design_capacity;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int sprdbat_battery_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct sprdbat_drivier_data *data = power_supply_get_drvdata(psy);
	int ret = 0;
	int limit = -1;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		limit = val->intval;

		data->bat_info.user_set_input_cur_max = limit;

		SPRDBAT_DEBUG("user set chgr limit:%d", val->intval);
		if (sprd_ext_ic_op->set_input_cur_limit)
			sprd_ext_ic_op->set_input_cur_limit(limit);
		else if (sprd_ext_ic_op->set_charge_cur)
			sprd_ext_ic_op->set_charge_cur(limit);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (sprd_ext_ic_op->set_input_cur_limit == NULL)
			return -EPERM;

		{
			int limit = val->intval / 1000;

			SPRDBAT_DEBUG("set chgr limit:%d", val->intval);
			if ((limit <= 0)
			    || limit > data->bat_info.chg_current_type_limit)
				limit = data->bat_info.chg_current_type_limit;

			sprd_ext_ic_op->set_input_cur_limit(limit);
			data->bat_info.input_cur_limit = limit;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		SPRDBAT_DEBUG("battery charging enabled %d\n", val->intval);
		mutex_lock(&sprdbat_data->lock);
		if (val->intval == 0) {
			sprdbat_change_module_state(SPRDBAT_CHG_FORCE_STOP_E);
			sprdbat_data->stop_charge();
		} else {
			sprdbat_change_module_state(SPRDBAT_CHG_FORCE_START_E);
			sprdbat_data->start_charge();
		}
		mutex_unlock(&sprdbat_data->lock);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		SPRDBAT_DEBUG("charging enabled %d\n", val->intval);
		if (sprd_ext_ic_op->set_input_cur_limit == NULL) {
			sprdbat_data->bat_info.charging_enabled = val->intval;
			return -EPERM;
		}
		if (val->intval == 0) {
			sprd_ext_ic_op->set_input_cur_limit(0);
			data->bat_info.input_cur_limit = 0;
		} else {
			int limit = sprdbat_data->bat_info.usb_input_current / 1000;

			SPRDBAT_DEBUG("charging enabled set chgr limit:%d\n", val->intval);
			if ((limit <= 0)
			    || limit > data->bat_info.chg_current_type_limit)
				limit = data->bat_info.chg_current_type_limit;

			sprd_ext_ic_op->set_input_cur_limit(limit);
			data->bat_info.input_cur_limit = limit;
		}
		sprdbat_data->bat_info.charging_enabled = val->intval;
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		if (sprd_ext_ic_op->set_ship_mode == NULL)
			return -EPERM;
		sprd_ext_ic_op->set_ship_mode(val->intval);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int sprdbat_battery_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = 1;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

static enum power_supply_property sprdbat_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_SET_SHIP_MODE,
};

static enum power_supply_property sprdbat_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static enum power_supply_property sprdbat_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static ssize_t sprdbat_store_caliberate(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);
static ssize_t sprdbat_show_caliberate(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);

#define SPRDBAT_CALIBERATE_ATTR(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IRUGO | S_IWUSR | S_IWGRP, },  \
	.show = sprdbat_show_caliberate,                  \
	.store = sprdbat_store_caliberate,                              \
}
#define SPRDBAT_CALIBERATE_ATTR_RO(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IRUGO, },  \
	.show = sprdbat_show_caliberate,                  \
}
#define SPRDBAT_CALIBERATE_ATTR_WO(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IWUSR | S_IWGRP, },  \
	.store = sprdbat_store_caliberate,                              \
}

static struct device_attribute sprd_caliberate[] = {
	SPRDBAT_CALIBERATE_ATTR_RO(real_time_voltage),
	SPRDBAT_CALIBERATE_ATTR_WO(stop_charge),
	SPRDBAT_CALIBERATE_ATTR_RO(real_time_current),
	SPRDBAT_CALIBERATE_ATTR_RO(charger_voltage),
	SPRDBAT_CALIBERATE_ATTR_RO(real_time_vbat_adc),
	SPRDBAT_CALIBERATE_ATTR_WO(save_capacity),
	SPRDBAT_CALIBERATE_ATTR_RO(temp_and_adc),
	SPRDBAT_CALIBERATE_ATTR(adjust_cur_min),
	SPRDBAT_CALIBERATE_ATTR(adjust_cur_max),
	SPRDBAT_CALIBERATE_ATTR(debug_jeita_enable),
	SPRDBAT_CALIBERATE_ATTR(debug_jeita),
	SPRDBAT_CALIBERATE_ATTR(chg_cool_state),
	SPRDBAT_CALIBERATE_ATTR_WO(force_chg_cur),
	SPRDBAT_CALIBERATE_ATTR_WO(force_input_cur),
	SPRDBAT_CALIBERATE_ATTR(temp_tab),
	SPRDBAT_CALIBERATE_ATTR(temp_comp_res),
	SPRDBAT_CALIBERATE_ATTR(temp_support),
	SPRDBAT_CALIBERATE_ATTR(chg_end_cur),
	SPRDBAT_CALIBERATE_ATTR_WO(request_start_charge),
};

static struct attribute *sprd_bat_attrs[] = {
	&sprd_caliberate[0].attr,
	&sprd_caliberate[1].attr,
	&sprd_caliberate[2].attr,
	&sprd_caliberate[3].attr,
	&sprd_caliberate[4].attr,
	&sprd_caliberate[5].attr,
	&sprd_caliberate[6].attr,
	&sprd_caliberate[7].attr,
	&sprd_caliberate[8].attr,
	&sprd_caliberate[9].attr,
	&sprd_caliberate[10].attr,
	&sprd_caliberate[11].attr,
	&sprd_caliberate[12].attr,
	&sprd_caliberate[13].attr,
	&sprd_caliberate[14].attr,
	&sprd_caliberate[15].attr,
	&sprd_caliberate[16].attr,
	&sprd_caliberate[17].attr,
	&sprd_caliberate[18].attr,
	NULL
};

static struct attribute_group sprd_bat_group = {
	.name = NULL,
	.attrs = sprd_bat_attrs,
};

enum sprdbat_attribute {
	BATTERY_VOLTAGE = 0,
	STOP_CHARGE,
	BATTERY_NOW_CURRENT,
	CHARGER_VOLTAGE,
	BATTERY_ADC,
	SAVE_CAPACITY,
	TEMP_AND_ADC,
	ADJUST_CUR_MIN,
	ADJUST_CUR_MAX,
	DEBUG_JEITA_ENABLE,
	DEBUG_JEITA,
	CHG_COOL_STATE,
	FORCE_CHG_CUR,
	FORCE_INPUT_CUR,
	TEMP_TAB,
	TEMP_COMP_RES,
	TEMP_SUPPORT,
	CHG_END_CUR,
	REQUEST_START_CHARGE,
};

static ssize_t sprdbat_store_caliberate(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int set_value;
	const ptrdiff_t off = attr - sprd_caliberate;

	if (off != TEMP_TAB) {
		if (kstrtoint(buf, 10, &set_value))
			return -EINVAL;
		pr_info("battery calibrate value %d\n", set_value);
	}

	mutex_lock(&sprdbat_data->lock);
	switch (off) {
	case STOP_CHARGE:
		if (sprdbat_data->bat_info.module_state
		    == POWER_SUPPLY_STATUS_DISCHARGING) {
			break;
		}

		if (set_value == 0) {
			sprdbat_change_module_state(SPRDBAT_CHG_FORCE_START_E);
			sprdbat_data->start_charge();
		} else {
			sprdbat_change_module_state(SPRDBAT_CHG_FORCE_STOP_E);
			sprdbat_data->stop_charge();
		}
		break;
	case SAVE_CAPACITY:
		{
			int temp = set_value - sprdbat_data->poweron_capacity;

			pr_info("battery temp:%d\n", temp);
			if (abs(temp) >
			    sprdbat_data->pdata->cap_valid_range_poweron
			    || 0 == set_value) {
				pr_info("battery poweron capacity:%d,%d\n",
					set_value,
					sprdbat_data->poweron_capacity);
				sprdbat_data->bat_info.capacity =
				    sprdbat_data->poweron_capacity;
			} else {
				pr_info("battery old capacity:%d,%d\n",
					set_value,
					sprdbat_data->poweron_capacity);
				sprdbat_data->bat_info.capacity = set_value;
			}
			power_supply_changed(sprdbat_data->battery);
		}
		break;
	case ADJUST_CUR_MIN:
		sprdbat_data->chg_cur_adjust_min = set_value;
		break;
	case ADJUST_CUR_MAX:
		if (set_value > sprdbat_data->bat_info.chg_current_type) {
			SPRDBAT_DEBUG("set_value over range %d\n",
				      set_value);
			set_value = sprdbat_data->bat_info.chg_current_type;
		}
		sprdbat_data->chg_cur_adjust_max = set_value;
		break;
	case DEBUG_JEITA_ENABLE:
		jeita_debug_enable = set_value;
		break;
	case DEBUG_JEITA:
		jeita_debug = set_value;
		break;
	case CHG_COOL_STATE:
		adjust_chg_flag = set_value;
		SPRDBAT_DEBUG("set_cur_state state=%d\n", set_value);
		sprdbat_chgcurrent_adjust(set_value);
		break;
	case FORCE_CHG_CUR:
		sprd_ext_ic_op->set_charge_cur(set_value);
		break;
	case FORCE_INPUT_CUR:
		if (sprd_ext_ic_op->set_input_cur_limit)
			sprd_ext_ic_op->set_input_cur_limit(set_value);
		break;
	case TEMP_TAB:
		{
			uint32_t i, vol;

			if (sscanf(buf, "%d,%d", &i, &vol) != 2)
				break;

			SPRDBAT_DEBUG("temp tab set %d,%d\n", i, vol);
			if (i >= sprdbat_data->pdata->temp_tab_size)
				break;

			sprdbat_data->pdata->temp_tab[i].x = vol;
		}
		break;
	case TEMP_COMP_RES:
		sprdbat_data->pdata->temp_comp_res = set_value;
		break;
	case TEMP_SUPPORT:
		sprdbat_data->pdata->temp_support = set_value;
		break;
	case CHG_END_CUR:
		sprdbat_data->pdata->chg_end_cur = set_value;
		break;
	case REQUEST_START_CHARGE:
		if (sprdbat_data->bat_info.module_state
		    == POWER_SUPPLY_STATUS_DISCHARGING)
			break;

		if (set_value) {
			/* already chagre */
			if (sprdbat_data->bat_info.chg_stop_flags
			    == SPRDBAT_CHG_END_NONE_BIT)
				break;
			sprdbat_change_module_state
				(SPRDBAT_CHG_REQUEST_START_E);
			sprdbat_data->start_charge();
		} else {
			sprdbat_change_module_state(SPRDBAT_CHG_FORCE_STOP_E);
			sprdbat_data->stop_charge();
		}
		break;

	default:
		count = -EINVAL;
		break;
	}
	mutex_unlock(&sprdbat_data->lock);
	return count;
}

static ssize_t sprdbat_show_caliberate(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - sprd_caliberate;
	int adc_value, temp_value;
	int voltage;
	int chg_cur = sprd_ext_ic_op->get_charge_cur_ext();

	switch (off) {
	case BATTERY_VOLTAGE:
		voltage = sprdchg_read_vbat_vol();
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", voltage);
		break;
	case BATTERY_NOW_CURRENT:
		if (sprdbat_data->bat_info.module_state ==
		    POWER_SUPPLY_STATUS_CHARGING) {
			i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n",
				       "ext charge ic");
		} else {
			i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n",
				       "discharging");
		}
		break;
	case CHARGER_VOLTAGE:
		if (sprdbat_data->bat_info.module_state ==
		    POWER_SUPPLY_STATUS_CHARGING) {
			voltage = sprdchg_read_vchg_vol();
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", voltage);
		} else {
			i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n",
				       "discharging");
		}

		break;
	case BATTERY_ADC:
		iio_read_channel_processed(sprdbat_data->pdata->channel_vbat,
					   &adc_value);
		if (adc_value < 0)
			adc_value = 0;
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", adc_value);
		break;
	case TEMP_AND_ADC:
		adc_value = sprdbat_read_temp_adc();
		temp_value = sprdbat_read_temp();
		if (adc_value < 0)
			adc_value = 0;
		i += scnprintf(buf + i, PAGE_SIZE - i, "adc:%d,temp:%d\n",
			       adc_value, temp_value);
		break;
	case ADJUST_CUR_MIN:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       sprdbat_data->chg_cur_adjust_min);
		break;
	case ADJUST_CUR_MAX:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       sprdbat_data->chg_cur_adjust_max);
		break;
	case DEBUG_JEITA_ENABLE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       jeita_debug_enable);
		break;
	case DEBUG_JEITA:
		i += scnprintf(buf + i, PAGE_SIZE - i, "temp:%d,cur%d\n",
			       jeita_debug, chg_cur);
		break;
	case CHG_COOL_STATE:
		adc_value = adjust_chg_flag;
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d,%d\n", adc_value,
			       chg_cur);
		break;
	case TEMP_TAB:
		{
			int j;

			i += scnprintf(buf + i, PAGE_SIZE - i,
				"temperature table dump:\n");
			for (j = 0; j < sprdbat_data->pdata->temp_tab_size;
			    j++) {
				i += scnprintf(buf + i, PAGE_SIZE - i,
					"id:%-2d vol:%-4d temp:%-4d;  ",
					j,
					sprdbat_data->pdata->temp_tab[j].x,
					sprdbat_data->pdata->temp_tab[j].y);
				if (!((j + 1) & 0x3))
					i += scnprintf(buf + i, PAGE_SIZE - i,
								"\n");
			}
			i += scnprintf(buf + i, PAGE_SIZE - i,
				"\ntemperature table dump end.\n");
		}
		break;
	case TEMP_COMP_RES:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				   sprdbat_data->pdata->temp_comp_res);
		break;
	case TEMP_SUPPORT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				   sprdbat_data->pdata->temp_support);
		break;
	case CHG_END_CUR:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       sprdbat_data->pdata->chg_end_cur);
		break;
	default:
		i = -EINVAL;
		break;
	}

	return i;
}

static void sprdbat_info_init(struct sprdbat_drivier_data *data)
{
	struct timespec64 cur_time;

	data->bat_info.adp_type = SDP_TYPE;
	if (data->gpio_vbat_detect > 0) {
		if (gpio_get_value(sprdbat_data->gpio_vbat_detect)) {
			SPRDBAT_DEBUG("vbat good!!!\n");
			data->bat_info.bat_present = 1;
			data->bat_info.bat_health = POWER_SUPPLY_HEALTH_GOOD;
			data->bat_info.chg_stop_flags =
			    SPRDBAT_CHG_END_NONE_BIT;
		} else {
			SPRDBAT_DEBUG("vbat unspec!!!\n");
			data->bat_info.bat_present = 0;
			data->bat_info.bat_health =
			    POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			data->bat_info.chg_stop_flags |= SPRDBAT_CHG_END_UNSPEC;
			data->stop_charge();
		}
	} else {
		SPRDBAT_DEBUG("vbat no detected!!!\n");
		data->bat_info.bat_present = 1;
		data->bat_info.bat_health = POWER_SUPPLY_HEALTH_GOOD;
		data->bat_info.chg_stop_flags = SPRDBAT_CHG_END_NONE_BIT;
	}
	data->bat_info.module_state = POWER_SUPPLY_STATUS_DISCHARGING;
	data->bat_info.chg_start_time = 0;
	cur_time = ktime_to_timespec64(ktime_get_boottime());
	sprdbat_data->sprdbat_update_capacity_time = cur_time.tv_sec;
	sprdbat_data->sprdbat_last_query_time = cur_time.tv_sec;
	trickle_s_time = cur_time.tv_sec;
	data->bat_info.capacity = sprdfgu_poweron_capacity();
	sprdbat_data->poweron_capacity = sprdfgu_poweron_capacity();
	data->bat_info.soc = sprdfgu_read_soc();
	data->bat_info.vbat_vol = sprdbat_read_vbat_vol();
	data->bat_info.vbat_ocv = sprdfgu_read_vbat_ocv();
	data->bat_info.cur_temp = sprdbat_read_temp();
	data->bat_info.bat_current = sprdfgu_read_batcurrent();
	data->bat_info.chging_current = 0;
	data->bat_info.chg_current_type = sprdbat_data->pdata->adp_sdp_cur;
	data->chg_cur_adjust_min = data->pdata->adp_sdp_cur;
	data->chg_cur_adjust_max = data->pdata->adp_sdp_cur;
	data->bat_info.chg_current_type_limit = data->pdata->adp_sdp_cur_limit;
	data->bat_info.input_cur_limit = data->pdata->adp_sdp_cur_limit;
	data->bat_info.vchg_vol_max = 0;
	data->bat_info.charging_enabled = 1;
	data->bat_info.user_set_input_cur_max = -1;
}

static int sprdbat_fchg_detect(void)
{
	if (sprdbat_is_force_5v()) {
		SPRDBAT_DEBUG("forbid fast charge!\n");
		sprdbat_data->fchg_det = 0;
		return 0;
	}

	if (sprdbat_data->bat_info.adp_type == DCP_TYPE) {
		if ((sprd_fchg_op != NULL) &&
			sprd_fchg_op->fchg_init) {
			if (sprd_ext_ic_op->support_fchg &&
				!sprd_ext_ic_op->support_fchg()) {
					SPRDBAT_DEBUG("not support fchg!\n");
					return 0;
			}
			sprdbat_data->fchg_det =
				sprd_fchg_op->fchg_init(sprdbat_data->pdata->fchg_vol);
		}
		if (!sprdbat_data->fchg_det) {
			if ((sprd_fchg_op != NULL) && sprd_fchg_op->fchg_deinit)
				sprd_fchg_op->fchg_deinit();
		}
	}

	return sprdbat_data->fchg_det;
}

static void sprdbat_charge_prepare(void)
{
	if (sprdbat_data->bat_info.adp_type == CDP_TYPE) {
		sprdbat_data->bat_info.chg_current_type =
		    sprdbat_data->pdata->adp_cdp_cur;
		sprdbat_data->bat_info.chg_current_type_limit =
		    sprdbat_data->pdata->adp_cdp_cur_limit;
	} else if (sprdbat_data->bat_info.adp_type == DCP_TYPE) {
		if (sprdbat_data->fchg_det) {
			sprdbat_data->bat_info.chg_current_type =
				sprdbat_data->pdata->adp_fchg_cur;
			sprdbat_data->bat_info.chg_current_type_limit =
				sprdbat_data->pdata->adp_fchg_cur_limit;
		} else {
			sprdbat_data->bat_info.chg_current_type =
				sprdbat_data->pdata->adp_dcp_cur;
			sprdbat_data->bat_info.chg_current_type_limit =
				sprdbat_data->pdata->adp_dcp_cur_limit;
		}
	} else if (sprdbat_data->bat_info.adp_type == UNKNOWN_TYPE) {
		sprdbat_data->bat_info.chg_current_type =
		    sprdbat_data->pdata->adp_unknown_cur;
		sprdbat_data->bat_info.chg_current_type_limit =
		    sprdbat_data->pdata->adp_unknown_cur_limit;
	} else {
		sprdbat_data->bat_info.chg_current_type =
		    sprdbat_data->pdata->adp_sdp_cur;
		sprdbat_data->bat_info.chg_current_type_limit =
		    sprdbat_data->pdata->adp_sdp_cur_limit;
	}

	if (!sprdbat_data->bat_info.chg_current_type_limit)
		sprdbat_data->bat_info.chg_current_type_limit =
			sprdbat_data->bat_info.chg_current_type;

	sprdbat_data->bat_info.input_cur_limit =
			sprdbat_data->bat_info.chg_current_type_limit;
}

static int sprdbat_start_charge(void)
{
	struct timespec64 cur_time;
	int cur_temp = sprdbat_read_temp();

	SPRDBAT_DEBUG("sprdbat_start_charge,cur_temp=%d\n", cur_temp);
	if (sprdbat_data->bat_info.chg_stop_flags != SPRDBAT_CHG_END_NONE_BIT) {
		SPRDBAT_DEBUG("can't start charge ,something wrong\n");
		return 0;
	}

	sprd_ext_ic_op->ic_init(sprdbat_data);

	if (sprd_ext_ic_op->set_input_cur_limit) {
		unsigned int limit =
			sprdbat_data->bat_info.input_cur_limit;
		sprd_ext_ic_op->set_input_cur_limit(limit);
	}

	sprdbat_data->cur_temp_status =
	    sprdbat_get_status_from_tab(sprdbat_data->pdata->jeita_tab_size,
					sprdbat_data->pdata->jeita_tab,
					cur_temp);
	SPRDBAT_DEBUG("sprdbat_data->cur_temp_status=%d\n",
		      sprdbat_data->cur_temp_status);
	sprdbat_data->last_temp_status = sprdbat_data->cur_temp_status;
	sprdbat_current_adjust_by_status(sprdbat_data->cur_temp_status);
	if (sprdbat_data->fchg_det) {
		sprdbat_clear_buff(chg_vol_buff, VOL_BUFF_CNT,
				sprdbat_data->pdata->fchg_vol);
		sprdbat_data->bat_info.avg_chg_vol
				= sprdbat_data->pdata->fchg_vol;
	} else {
		sprdbat_clear_buff(chg_vol_buff, VOL_BUFF_CNT, 5000);
		sprdbat_data->bat_info.avg_chg_vol = 5000;
	}

	SPRDBAT_DEBUG("Start charge init vchg vol=%d\n",
		      sprdbat_data->bat_info.avg_chg_vol);

	sprdbat_data->chg_cur_adjust_max =
		sprdbat_data->bat_info.chg_current_type;

	cur_time = ktime_to_timespec64(ktime_get_boottime());
	sprdbat_data->bat_info.chg_start_time = cur_time.tv_sec;

	sprdbat_data->bat_info.chging_on = 1;

	SPRDBAT_DEBUG
		("%s health:%d,chg_start_time:%lld,chg_current_type:%d\n",
		__func__, sprdbat_data->bat_info.bat_health,
		sprdbat_data->bat_info.chg_start_time,
		sprdbat_data->bat_info.chg_current_type);

	return 0;
}

static int sprdbat_stop_charge(void)
{
	sprdbat_data->bat_info.chg_start_time = 0;
	sprd_ext_ic_op->charge_stop_ext(sprdbat_data->bat_info.chg_stop_flags);
	SPRDBAT_DEBUG("sprdbat_stop_charge\n");
	power_supply_changed(sprdbat_data->battery);
	return 0;
}

static inline void _sprdbat_clear_stopflags(uint32_t flag_msk)
{
	sprdbat_data->bat_info.chg_stop_flags &= ~flag_msk;
	SPRDBAT_DEBUG("_sprdbat_clear_stopflags flag_msk:0x%x flag:0x%x\n",
		      flag_msk, sprdbat_data->bat_info.chg_stop_flags);

	if (sprdbat_data->bat_info.chg_stop_flags == SPRDBAT_CHG_END_NONE_BIT) {
		sprdbat_data->bat_info.bat_health = POWER_SUPPLY_HEALTH_GOOD;
		if (sprdbat_data->bat_info.module_state ==
			POWER_SUPPLY_STATUS_NOT_CHARGING) {
			sprdbat_data->bat_info.module_state =
				POWER_SUPPLY_STATUS_CHARGING;
		}
	} else if (sprdbat_data->bat_info.chg_stop_flags &
		SPRDBAT_CHG_END_UNSPEC)
		sprdbat_data->bat_info.bat_health =
			POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	else if (sprdbat_data->bat_info.chg_stop_flags &
		SPRDBAT_CHG_END_OTP_OVERHEAT_BIT)
		sprdbat_data->bat_info.bat_health =
			POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (sprdbat_data->bat_info.chg_stop_flags &
		SPRDBAT_CHG_END_OTP_COLD_BIT)
		sprdbat_data->bat_info.bat_health = POWER_SUPPLY_HEALTH_COLD;
	else if (sprdbat_data->bat_info.chg_stop_flags &
		SPRDBAT_CHG_END_CHGR_OTP_BIT)
		sprdbat_data->bat_info.bat_health =
			POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (sprdbat_data->bat_info.chg_stop_flags &
		SPRDBAT_CHG_END_OVP_BIT)
		sprdbat_data->bat_info.bat_health =
			POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if (sprdbat_data->bat_info.chg_stop_flags &
		SPRDBAT_CHG_END_BAT_OVP_BIT)
		sprdbat_data->bat_info.bat_health =
			POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if (sprdbat_data->bat_info.chg_stop_flags &
		SPRDBAT_CHG_END_TIMEOUT_BIT)
		sprdbat_data->bat_info.bat_health =
			POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
	else if (sprdbat_data->bat_info.chg_stop_flags &
		SPRDBAT_CHG_END_FULL_BIT)
		sprdbat_data->bat_info.bat_health = POWER_SUPPLY_HEALTH_GOOD;
	else
		SPRDBAT_DEBUG("error chg_stop_flags\n");
}

static inline void _sprdbat_set_stopflags(uint32_t flag)
{
	SPRDBAT_DEBUG("_sprdbat_set_stopflags flags:0x%x\n", flag);
	sprdbat_data->bat_info.chg_stop_flags |= flag;
	if (sprdbat_data->bat_info.module_state ==
		POWER_SUPPLY_STATUS_CHARGING) {
		if (flag == SPRDBAT_CHG_END_FULL_BIT)
			sprdbat_data->bat_info.module_state =
			    POWER_SUPPLY_STATUS_FULL;
		else
			sprdbat_data->bat_info.module_state =
			    POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
}

static void sprdbat_change_module_state(uint32_t event)
{
	SPRDBAT_DEBUG("sprdbat_change_module_state event :0x%x\n", event);
	switch (event) {
	case SPRDBAT_ADP_PLUGIN_E:
		sprdbat_data->bat_info.chg_this_timeout =
		    sprdbat_data->pdata->chg_timeout;
		sprdbat_data->bat_info.chg_stop_flags &= SPRDBAT_CHG_END_UNSPEC;
		if (sprdbat_data->bat_info.
		    chg_stop_flags & SPRDBAT_CHG_END_UNSPEC) {
			sprdbat_data->bat_info.module_state =
			    POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else {
			sprdbat_data->bat_info.module_state =
			    POWER_SUPPLY_STATUS_CHARGING;
		}
		queue_delayed_work(sprdbat_data->monitor_wqueue,
				   &sprdbat_data->sprdbat_charge_work, 2 * HZ);
		break;
	case SPRDBAT_ADP_PLUGOUT_E:
		if (sprdbat_data->bat_info.
		    chg_stop_flags & SPRDBAT_CHG_END_UNSPEC) {
			sprdbat_data->bat_info.bat_health =
			    POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		} else {
			sprdbat_data->bat_info.bat_health =
			    POWER_SUPPLY_HEALTH_GOOD;
		}
		sprdbat_data->bat_info.chg_stop_flags &= SPRDBAT_CHG_END_UNSPEC;
		sprdbat_data->bat_info.module_state =
		    POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case SPRDBAT_OVI_STOP_E:
		if (sprdbat_data->bat_info.bat_health ==
		    POWER_SUPPLY_HEALTH_GOOD) {
			sprdbat_data->bat_info.bat_health =
			    POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		}
		_sprdbat_set_stopflags((uint32_t)SPRDBAT_CHG_END_OVP_BIT);
		break;
	case SPRDBAT_OVI_RESTART_E:
		_sprdbat_clear_stopflags((uint32_t)SPRDBAT_CHG_END_OVP_BIT);
		break;
	case SPRDBAT_OTP_COLD_STOP_E:
		if (sprdbat_data->bat_info.bat_health ==
		    POWER_SUPPLY_HEALTH_GOOD) {
			sprdbat_data->bat_info.bat_health =
			    POWER_SUPPLY_HEALTH_COLD;
		}
		_sprdbat_set_stopflags((uint32_t)
			SPRDBAT_CHG_END_OTP_COLD_BIT);
		break;
	case SPRDBAT_OTP_OVERHEAT_STOP_E:
		if (sprdbat_data->bat_info.bat_health ==
		    POWER_SUPPLY_HEALTH_GOOD) {
			sprdbat_data->bat_info.bat_health =
			    POWER_SUPPLY_HEALTH_OVERHEAT;
		}
		_sprdbat_set_stopflags((uint32_t)
			SPRDBAT_CHG_END_OTP_OVERHEAT_BIT);
		break;
	case SPRDBAT_OTP_COLD_RESTART_E:
		_sprdbat_clear_stopflags((uint32_t)
			SPRDBAT_CHG_END_OTP_COLD_BIT);
		break;
	case SPRDBAT_OTP_OVERHEAT_RESTART_E:
		_sprdbat_clear_stopflags((uint32_t)
			SPRDBAT_CHG_END_OTP_OVERHEAT_BIT);
		break;
	case SPRDBAT_CHG_FULL_E:
		sprdbat_data->bat_info.chg_this_timeout =
		    sprdbat_data->pdata->chg_rechg_timeout;
		_sprdbat_set_stopflags((uint32_t)
			SPRDBAT_CHG_END_FULL_BIT);
		break;
	case SPRDBAT_RECHARGE_E:
		_sprdbat_clear_stopflags((uint32_t)
			SPRDBAT_CHG_END_FULL_BIT);
		break;
	case SPRDBAT_CHG_TIMEOUT_E:
		if (sprdbat_data->bat_info.bat_health ==
		    POWER_SUPPLY_HEALTH_GOOD)
			sprdbat_data->bat_info.bat_health =
				POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;

		sprdbat_data->bat_info.chg_this_timeout =
		    sprdbat_data->pdata->chg_rechg_timeout;
		_sprdbat_set_stopflags((uint32_t)
			SPRDBAT_CHG_END_TIMEOUT_BIT);
		break;
	case SPRDBAT_CHG_TIMEOUT_RESTART_E:
		_sprdbat_clear_stopflags((uint32_t)
			SPRDBAT_CHG_END_TIMEOUT_BIT);
		break;
	case SPRDBAT_CHG_UNSPEC_E:
		if (sprdbat_data->bat_info.bat_health ==
		    POWER_SUPPLY_HEALTH_GOOD) {
			sprdbat_data->bat_info.bat_health =
			    POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		}
		_sprdbat_set_stopflags(SPRDBAT_CHG_END_UNSPEC);
		break;
	case SPRDBAT_CHG_UNSPEC_RESTART_E:
		_sprdbat_clear_stopflags(SPRDBAT_CHG_END_UNSPEC);
		break;
	case SPRDBAT_VBAT_OVP_E:
		sprdbat_data->bat_info.bat_health =
		    POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		_sprdbat_set_stopflags((uint32_t)
			SPRDBAT_CHG_END_BAT_OVP_BIT);
		break;
	case SPRDBAT_VBAT_OVP_RESTART_E:
		_sprdbat_clear_stopflags((uint32_t)
			SPRDBAT_CHG_END_BAT_OVP_BIT);
		break;
	case SPRDBAT_FULL_TO_CHARGING_E:
		sprdbat_data->bat_info.module_state =
		    POWER_SUPPLY_STATUS_CHARGING;
		break;
	/* only change module state, but do not close charge current */
	case SPRDBAT_CHARGING_TO_FULL_E:
		sprdbat_data->bat_info.module_state =
		    POWER_SUPPLY_STATUS_FULL;
		break;
	case SPRDBAT_CHG_FORCE_STOP_E:
		_sprdbat_set_stopflags((uint32_t)
			SPRDBAT_CHG_END_FORCE_STOP_BIT);
		break;
	case SPRDBAT_CHG_FORCE_START_E:
		sprdbat_data->bat_info.chg_this_timeout =
		    sprdbat_data->pdata->chg_timeout;
		_sprdbat_clear_stopflags((uint32_t)~SPRDBAT_CHG_END_NONE_BIT);
		break;
	case SPRDBAT_CHG_REQUEST_START_E:
		_sprdbat_clear_stopflags((uint32_t)
			SPRDBAT_CHG_END_FORCE_STOP_BIT);
		break;
	case SPRDBAT_CHGR_OTP_E:
		if (sprdbat_data->bat_info.bat_health ==
		    POWER_SUPPLY_HEALTH_GOOD) {
			sprdbat_data->bat_info.bat_health =
			    POWER_SUPPLY_HEALTH_OVERHEAT;
		}
		_sprdbat_set_stopflags((uint32_t)
			SPRDBAT_CHG_END_CHGR_OTP_BIT);
		break;
	case SPRDBAT_CHGR_OTP_START_E:
		_sprdbat_clear_stopflags((uint32_t)
			SPRDBAT_CHG_END_CHGR_OTP_BIT);
		break;
	default:
		break;
	}
	power_supply_changed(sprdbat_data->battery);
}

static int plugin_callback(void)
{
	SPRDBAT_DEBUG("charger plug in interrupt happen\n");

	mutex_lock(&sprdbat_data->lock);
	sprdbat_data->sprdbat_vbat_ovp_cnt = 0;

	if (sprdbat_data->bat_info.module_state
	    != POWER_SUPPLY_STATUS_DISCHARGING) {
		mutex_unlock(&sprdbat_data->lock);
		return 0;
	}

	sprdbat_data->bat_info.adp_type = sprdchg_charger_is_adapter();
	if ((sprdbat_data->bat_info.adp_type == SDP_TYPE) ||
		(sprdbat_data->bat_info.adp_type == CDP_TYPE)) {
		sprdbat_data->bat_info.usb_online = 1;
		power_supply_changed(sprdbat_data->usb);
	} else {
		sprdbat_data->bat_info.ac_online = 1;
		power_supply_changed(sprdbat_data->ac);
	}

	sprdbat_data->bat_info.chgr_temp
		= sprdbat_get_avg_chgr_temp(NORMAL_TEMP, true);

	sprdbat_change_module_state(SPRDBAT_ADP_PLUGIN_E);
	sprdbat_adp_plug_nodify(1);
	sprdbat_fchg_detect();
	sprdbat_charge_prepare();
	sprdbat_data->start_charge();
	if (sprdchg_timer_op->timer_enable) {
		u32 polling_time = sprdbat_data->pdata->chg_polling_time;

		if (sprdbat_data->pdata->only_vol_mode)
			sprdchg_timer_op->timer_enable(polling_time, ONE_TIME);
		else
			sprdchg_timer_op->timer_enable(polling_time,
						       PERIOD_TIME);
	}

	mutex_unlock(&sprdbat_data->lock);

	SPRDBAT_DEBUG("plugin_callback:adp_type:%d\n",
		      sprdbat_data->bat_info.adp_type);
	SPRDBAT_DEBUG("plugin_callback: end...\n");
	return 0;
}

static int plugout_callback(void)
{
	uint32_t adp_type = sprdbat_data->bat_info.adp_type;

	SPRDBAT_DEBUG("charger plug out interrupt happen\n");

	mutex_lock(&sprdbat_data->lock);

	if (sprdbat_data->bat_info.module_state
	    == POWER_SUPPLY_STATUS_DISCHARGING) {
		mutex_unlock(&sprdbat_data->lock);
		return 0;
	}

	disable_irq_nosync(sprdbat_data->irq_vchg_ovi);

	if (sprdchg_timer_op->timer_disable)
		sprdchg_timer_op->timer_disable();

	sprdbat_change_module_state(SPRDBAT_ADP_PLUGOUT_E);
	sprdbat_data->stop_charge();

	if ((sprd_fchg_op != NULL) && sprd_fchg_op->fchg_deinit)
		sprd_fchg_op->fchg_deinit();

	sprdbat_adp_plug_nodify(0);
	sprdbat_data->bat_info.module_state = POWER_SUPPLY_STATUS_DISCHARGING;

	sprdbat_data->bat_info.adp_type = SDP_TYPE;
	sprdbat_data->bat_info.ac_online = 0;
	sprdbat_data->bat_info.usb_online = 0;
	sprdbat_data->fchg_det = 0;
	mutex_unlock(&sprdbat_data->lock);

	if (sprd_ext_ic_op->set_input_cur_limit) {
		unsigned int limit = sprdbat_data->pdata->adp_sdp_cur_limit;

		sprd_ext_ic_op->set_input_cur_limit(limit);
		sprdbat_data->bat_info.input_cur_limit = limit;
	}

	if ((adp_type == SDP_TYPE) || (adp_type == CDP_TYPE))
		power_supply_changed(sprdbat_data->usb);
	else
		power_supply_changed(sprdbat_data->ac);

	return 0;
}

static int sprdbat_usb_plug_event(struct notifier_block *this,
				  unsigned long limit, void *ptr)
{
	wake_lock_timeout(&(sprdbat_data->charger_wake_lock),
			  SPRDBAT_PLUG_WAKELOCK_TIME_SEC * HZ);

	if (limit != 0)
		SPRDBAT_DEBUG("sprdbat_usb_plug_event plug in\n");
	else
		SPRDBAT_DEBUG("sprdbat_usb_plug_event plug out\n");

	queue_work(sprdbat_data->monitor_wqueue,
		   &sprdbat_data->plug_work);
	return 0;
}

static void sprdbat_plug_works(struct work_struct *work)
{
	if (usb_charger_get_state(sprdbat_data->usb_charger)
	    == USB_CHARGER_PRESENT)
		plugin_callback();
	else
		plugout_callback();
}

static char *supply_list[] = {
	"battery",
};

static char *battery_supply_list[] = {
	"audio-ldo",
	"sprdfgu",
};

static irqreturn_t sprdbat_timer_handler(int irq, void *data)
{
	wake_lock_timeout(&(sprdbat_data->charger_wake_lock), 3 * HZ);
	queue_delayed_work(sprdbat_data->monitor_wqueue,
			   &sprdbat_data->sprdbat_charge_work,
			   msecs_to_jiffies(1000));
	return IRQ_HANDLED;
}

static __used irqreturn_t sprdbat_vbat_detect_irq(int irq, void *dev_id)
{
	disable_irq_nosync(sprdbat_data->irq_vbat_detect);
	SPRDBAT_DEBUG("battery detect handle!!!!\n");
	queue_work(sprdbat_data->monitor_wqueue,
		   &sprdbat_data->vbat_detect_irq_work);
	return IRQ_HANDLED;
}

static void sprdbat_vbat_detect_irq_works(struct work_struct *work)
{
	int value;

	value = gpio_get_value(sprdbat_data->gpio_vbat_detect);
	SPRDBAT_DEBUG("bat_detect value:0x%x\n", value);
	mutex_lock(&sprdbat_data->lock);
	if (value) {
		if (!sprdbat_data->bat_info.bat_present) {
			sprdbat_data->bat_info.bat_present = 1;
			sprdbat_change_module_state
				(SPRDBAT_CHG_UNSPEC_RESTART_E);
			if (POWER_SUPPLY_STATUS_DISCHARGING !=
				sprdbat_data->bat_info.module_state)
				sprdbat_data->start_charge();
			SPRDBAT_DEBUG("vbat_detect-start_charge!!!!\n");
		}
		irq_set_irq_type(sprdbat_data->irq_vbat_detect,
			IRQ_TYPE_LEVEL_LOW);
	} else {
		sprdbat_data->bat_info.bat_present = 0;
		sprdbat_change_module_state(SPRDBAT_CHG_UNSPEC_E);
		sprdbat_data->stop_charge();
		SPRDBAT_DEBUG("vbat_detect-stop_charge!!!!\n");
		irq_set_irq_type(sprdbat_data->irq_vbat_detect,
			IRQ_TYPE_LEVEL_HIGH);
	}
	enable_irq(sprdbat_data->irq_vbat_detect);
	mutex_unlock(&sprdbat_data->lock);
}

static void sprdbat_chg_rechg_monitor(void)
{
	int chg_status = POWER_SUPPLY_STATUS_CHARGING;

	SPRDBAT_DEBUG("sprdbat_chg_rechg_monitor\n");

	if (sprdbat_data->pdata->only_vol_mode) {
		if (sprdbat_data->bat_info.vbat_ocv <=
		    sprdbat_data->pdata->rechg_vol) {
			sprdbat_change_module_state(SPRDBAT_RECHARGE_E);
			sprdbat_data->start_charge();
		}

		return;
	}

	if (sprdbat_data->pdata->chg_full_condition == FROM_EXT_IC) {
		chg_status = sprd_ext_ic_op->get_charging_status();
		if (chg_status == POWER_SUPPLY_STATUS_FULL) {
			SPRDBAT_DEBUG("chg full status  ,need rechg??\n");
			if (sprdbat_data->bat_info.vbat_ocv <=
			    sprdbat_data->pdata->rechg_vol) {
				SPRDBAT_DEBUG("yes,now recharge\n");
				sprdbat_change_module_state(SPRDBAT_RECHARGE_E);
				sprdbat_data->start_charge();
			} else {
				SPRDBAT_DEBUG("no,don't need recharge\n");
			}
		} else if (chg_status == POWER_SUPPLY_STATUS_CHARGING) {
			SPRDBAT_DEBUG("chg restart by status charging\n");
			sprdbat_change_module_state(SPRDBAT_RECHARGE_E);
		} else {
			SPRDBAT_DEBUG("some faults\n");
		}
	} else if (sprdbat_data->pdata->chg_full_condition == VOL_AND_CUR) {
		SPRDBAT_DEBUG("chg stop_flag full  ,need rechg??\n");
		if (sprdbat_data->bat_info.vbat_ocv <=
		    sprdbat_data->pdata->rechg_vol) {
			SPRDBAT_DEBUG("yes, chg restart\n");
		       if (!charger_policy_get_status()) {
				sprdbat_change_module_state(SPRDBAT_RECHARGE_E);
				sprdbat_data->start_charge();
		       } else {
				SPRDBAT_DEBUG("policy: not chg restart\n");
		       }
		} else {
			SPRDBAT_DEBUG("no,don't need recharge\n");
		}
	} else {
		SPRDBAT_DEBUG("need add\n");
	}
}

static void sprdbat_chg_status_monitor(void)
{
	int chg_status = POWER_SUPPLY_STATUS_CHARGING;

	SPRDBAT_DEBUG
		(" %s,ocv=%d, cur=%d,chg_end_vol_l=%d,chg_end_cur=%d\n",
	__func__, sprdbat_data->bat_info.vbat_ocv,
	sprdbat_data->bat_info.bat_current,
	sprdbat_data->pdata->chg_end_vol_l,
	sprdbat_data->pdata->chg_end_cur);

	if (sprdbat_data->pdata->only_vol_mode) {
		if (sprdbat_data->bat_info.vbat_vol >
			sprdbat_data->pdata->chg_end_vol_l) {
			sprdbat_data->chg_full_trigger_cnt++;
			if (sprdbat_data->chg_full_trigger_cnt >= 2) {
				sprdbat_data->chg_full_trigger_cnt = 0;
				if (sprdbat_data->bat_info.capacity >= 99 &&
				    trickle_time >=
				    sprdbat_data->pdata->cap_one_per_time) {
					sprdbat_change_module_state
						(SPRDBAT_CHG_FULL_E);
					sprdbat_data->stop_charge();
				} else {
					sprdfgu_force_set_soc(1000);
				}
			}
		} else {
			sprdbat_data->chg_full_trigger_cnt = 0;
		}
		return;
	}

	if (sprdbat_data->pdata->chg_full_condition == FROM_EXT_IC) {
		chg_status = sprd_ext_ic_op->get_charging_status();
		if (chg_status == POWER_SUPPLY_STATUS_FULL) {
			SPRDBAT_DEBUG("chg full\n");
			/* capacity is high enough, set the status to full */
			if (sprdbat_data->bat_info.capacity >= 99 &&
			    trickle_time >=
			    sprdbat_data->pdata->cap_one_per_time)
				sprdbat_change_module_state(SPRDBAT_CHG_FULL_E);
			else
				sprdfgu_force_set_soc(1000);
		} else {
			SPRDBAT_DEBUG("chging or fault\n");
		}
	} else if (sprdbat_data->pdata->chg_full_condition == VOL_AND_CUR) {
		if ((sprdbat_data->bat_info.vbat_vol >
		     sprdbat_data->pdata->chg_end_vol_l)
		    && (sprdbat_data->bat_info.bat_current <
			sprdbat_data->pdata->chg_end_cur)) {
			sprdbat_data->chg_full_trigger_cnt++;
			if (sprdbat_data->chg_full_trigger_cnt >= 2) {
				SPRDBAT_DEBUG("charge full stop charge\n");
				sprdbat_data->chg_full_trigger_cnt = 0;
				/* cap is high enough, set the status to full */
				if (sprdbat_data->bat_info.capacity >= 99 &&
				    trickle_time >=
				    sprdbat_data->pdata->cap_one_per_time) {
					sprdbat_change_module_state
						(SPRDBAT_CHG_FULL_E);
					sprdbat_data->stop_charge();
				} else {
					sprdfgu_force_set_soc(1000);
				}
			}
		} else {
			sprdbat_data->chg_full_trigger_cnt = 0;
		}
	} else if (sprdbat_data->pdata->chg_full_condition == VOL_AND_STATUS) {
		if ((sprdbat_data->bat_info.vbat_vol >
		     sprdbat_data->pdata->chg_end_vol_l
		     || sprd_ext_ic_op->get_charging_status())
		    && (sprdbat_data->bat_info.bat_current <
			sprdbat_data->pdata->chg_end_cur)) {
			sprdbat_data->chg_full_trigger_cnt++;
			if (sprdbat_data->chg_full_trigger_cnt >= 2) {
				SPRDBAT_DEBUG("charge full stop charge\n");
				sprdbat_data->chg_full_trigger_cnt = 0;
				/* cap is high enough, set the status to full */
				if (sprdbat_data->bat_info.capacity >= 99 &&
				    trickle_time >=
				    sprdbat_data->pdata->cap_one_per_time) {
					sprdbat_change_module_state
						(SPRDBAT_CHG_FULL_E);
					sprdbat_data->stop_charge();
				} else {
					sprdfgu_force_set_soc(1000);
				}
			}
		} else {
			sprdbat_data->chg_full_trigger_cnt = 0;
		}
	} else {
		SPRDBAT_DEBUG("bad chg_full_condition\n");
	}
}

static void sprdbat_fault_recovery_monitor(void)
{
	SPRDBAT_DEBUG("sprdbat_fault_recovery_monitor enter\n");

	if (sprdbat_data->bat_info.chg_stop_flags &
		SPRDBAT_CHG_END_BAT_OVP_BIT) {
		SPRDBAT_DEBUG("chg  not vbat ovp now is  chging\n");
		sprdbat_change_module_state(SPRDBAT_VBAT_OVP_RESTART_E);
	}
}

static void sprdbat_fault_monitor(void)
{
	int chg_fault, status, vbat_ovp, terminal_voltage;

	SPRDBAT_DEBUG("sprdbat_fault_monitor enter\n");
	status = sprdbat_data->cur_temp_status;
	terminal_voltage = sprdbat_data->pdata->jeita_tab[status].z;
	vbat_ovp = terminal_voltage + VBAT_OVP_THRESHOLD;
	chg_fault = sprd_ext_ic_op->get_charging_fault();

	if (chg_fault == SPRDBAT_CHG_END_NONE_BIT)
		sprdbat_fault_recovery_monitor();

	if (chg_fault & SPRDBAT_CHG_END_OTP_COLD_BIT)
		SPRDBAT_DEBUG(" power cold\n");

	if (chg_fault & SPRDBAT_CHG_END_OTP_OVERHEAT_BIT)
		SPRDBAT_DEBUG("power hot\n");

	if (chg_fault & SPRDBAT_CHG_END_TIMEOUT_BIT) {
		SPRDBAT_DEBUG("  safe time expire\n");
		sprdbat_change_module_state(SPRDBAT_CHG_TIMEOUT_E);
	}
	if (chg_fault & SPRDBAT_CHG_END_BAT_OVP_BIT) {
		if (sprdbat_data->sprdbat_vbat_ovp_cnt > VBAT_OVP_CNT_THRESHOLD &&
			sprdbat_data->bat_info.vbat_vol > vbat_ovp) {
			SPRDBAT_DEBUG("fault: vbat ovp\n");
			sprdbat_change_module_state(SPRDBAT_VBAT_OVP_E);
		} else {
			SPRDBAT_DEBUG("warning: vbat ovp\n");
			sprdbat_fchg_detect();
			sprdbat_charge_prepare();
			sprdbat_data->start_charge();
			sprdbat_data->sprdbat_vbat_ovp_cnt++;
		}
	} else {
		sprdbat_data->sprdbat_vbat_ovp_cnt = 0;
	}

	if (chg_fault == SPRDBAT_CHG_END_UNSPEC)
		SPRDBAT_DEBUG(" unspec fault\n");

}
/* TODO: switch polling to interrupt again need open this code */
#if 0
static void sprdbat_charge_fault_event(void)
{
	int chg_fault = 0;
	/* TODO: need modify this when we use ext ic interrupt */
	SPRDBAT_DEBUG("enter sprdbat_charge_fault_event\n");
	chg_fault = sprd_ext_ic_op->get_charging_fault();

	if (chg_fault == SPRDBAT_CHG_END_NONE_BIT)
		sprdbat_fault_recovery_monitor();

	if (chg_fault & SPRDBAT_CHG_END_OTP_COLD_BIT)
		SPRDBAT_DEBUG(" power cold\n");

	if (chg_fault & SPRDBAT_CHG_END_OTP_OVERHEAT_BIT)
		SPRDBAT_DEBUG("power hot\n");

	if (chg_fault & SPRDBAT_CHG_END_TIMEOUT_BIT) {
		SPRDBAT_DEBUG("  safe time expire\n");
		sprdbat_change_module_state(SPRDBAT_CHG_TIMEOUT_E);
	}
	if (chg_fault & SPRDBAT_CHG_END_BAT_OVP_BIT) {
		SPRDBAT_DEBUG(" vbat ovp\n");
		sprdbat_change_module_state(SPRDBAT_VBAT_OVP_E);
	}
	if (chg_fault == SPRDBAT_CHG_END_UNSPEC)
		SPRDBAT_DEBUG(" unspec fault\n");

}
static int sprdbat_chg_event_call(struct notifier_block *nb,
				  unsigned long val, void *v)
{
	int chg_statu, chg_fault;
	struct power_supply *psy = v;

	if (val != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;
	if ((!strcmp(psy->desc->name, "ac"))
			|| (!strcmp(psy->desc->name, "usb")) ||
	    (!strcmp(psy->desc->name, "battery"))) {
		return NOTIFY_OK;
	}
	mutex_lock(&sprdbat_data->lock);
	if (!strcmp(psy->desc->name, CHG_PSY_INNER)) {
		chg_statu = sprd_ext_ic_op->get_charging_status();
		chg_fault = sprd_ext_ic_op->get_charging_fault();
		if (chg_statu)
			SPRDBAT_DEBUG("cccv irq\n");

		if (chg_fault == SPRDBAT_CHG_END_BAT_OVP_BIT) {
			SPRDBAT_DEBUG("chg ovp irq\n");
			if (sprdbat_data->bat_info.bat_health ==
				POWER_SUPPLY_HEALTH_GOOD) {
				sprdbat_data->bat_info.bat_health =
					POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			}
			_sprdbat_set_stopflags((uint32_t)
				SPRDBAT_CHG_END_OVP_BIT);
			sprdbat_data->stop_charge();
		} else if (sprdbat_data->bat_info.chg_stop_flags &
				SPRDBAT_CHG_END_OVP_BIT) {
			SPRDBAT_DEBUG("chg ovp recovery\n");
			_sprdbat_clear_stopflags((uint32_t)
				SPRDBAT_CHG_END_OVP_BIT);
			sprdbat_data->start_charge();
		}

	}
	if (!strcmp(psy->desc->name, CHG_PSY_EXT))
		sprdbat_charge_fault_event();

	if (!strcmp(psy->desc->name, "sprdfgu"))
		SPRDBAT_DEBUG("do nothing ,maybe need add\n");

	mutex_unlock(&sprdbat_data->lock);
	return NOTIFY_OK;
}
#endif
static void sprdbat_chg_print_log(void)
{
	struct timespec64 cur_time;

	cur_time = ktime_to_timespec64(ktime_get_boottime());
#if __BITS_PER_LONG == 64
	SPRDBAT_DEBUG("cur time:%ld\n", cur_time.tv_sec);
#else
	SPRDBAT_DEBUG("cur time:%lld\n", cur_time.tv_sec);
#endif
	SPRDBAT_DEBUG
		("chg_log:health:%d,state:%d,chg_s_time:%lld\n",
		sprdbat_data->bat_info.bat_health,
		sprdbat_data->bat_info.module_state,
		sprdbat_data->bat_info.chg_start_time);
	SPRDBAT_DEBUG
		("stopflags0x:%x,temp:%d\n",
		sprdbat_data->bat_info.chg_stop_flags,
		sprdbat_data->bat_info.cur_temp);
	SPRDBAT_DEBUG
	    ("chg_log:chgcur_type:%d,vchg:%d,adp_type:%d\n",
	    sprdbat_data->bat_info.chg_current_type,
	    sprdchg_read_vchg_vol(),
	    sprdbat_data->bat_info.adp_type);
}

static void sprdbat_print_battery_log(void)
{
	struct timespec64 cur_time;

	cur_time = ktime_to_timespec64(ktime_get_boottime());

#if __BITS_PER_LONG == 64
	SPRDBAT_DEBUG("bat_log:time:%ld\n", cur_time.tv_sec);
#else
	SPRDBAT_DEBUG("bat_log:time:%lld\n", cur_time.tv_sec);
#endif

	SPRDBAT_DEBUG
		("vbat:%d,ocv:%d,current:%d,cap:%d\n",
		sprdbat_data->bat_info.vbat_vol,
		sprdbat_data->bat_info.vbat_ocv,
		sprdbat_data->bat_info.bat_current,
		sprdbat_data->bat_info.capacity);
	SPRDBAT_DEBUG
		("state:%d,auxbat:%d,temp:%d,present:%d\n",
		sprdbat_data->bat_info.module_state,
		sprdchg_read_vbat_vol(),
		sprdbat_data->bat_info.cur_temp,
		sprdbat_data->bat_info.bat_present);
}

#define SPRDBAT_SHUTDOWN_OFSSET 50
static void sprdbat_update_capacty(void)
{
	uint32_t fgu_capacity;
	int flush_time = 0;
	int period_time = 0;
	struct timespec64 cur_time;
	int chging_flag;

	if (sprdbat_data->bat_info.capacity == ~0U)
		return;

	if (sprdbat_data->pdata->only_vol_mode) {
		if (sprdbat_data->bat_info.module_state ==
		    POWER_SUPPLY_STATUS_CHARGING)
			chging_flag = 1;
		else
			chging_flag = 0;

		fgu_capacity = sprdfgu_only_vol_read_capacity(chging_flag);
	} else {
		fgu_capacity = sprdfgu_read_capacity();
	}

	cur_time = ktime_to_timespec64(ktime_get_boottime());

	if (POWER_SUPPLY_STATUS_CHARGING ==
	    sprdbat_data->bat_info.module_state) {
		if (sprdbat_data->bat_info.capacity >= 99) {
			trickle_time = cur_time.tv_sec -
				trickle_s_time;
		} else {
			trickle_s_time = cur_time.tv_sec;
			trickle_time = 0;
		}
	} else {
		trickle_s_time = cur_time.tv_sec;
		trickle_time = sprdbat_data->pdata->trickle_timeout +
			sprdbat_data->pdata->cap_one_per_time + 1;
	}

	SPRDBAT_DEBUG("trickle_s_time: = %lld,trickle_time: = %d\n",
			trickle_s_time, trickle_time);

	flush_time =
		(int)(cur_time.tv_sec -
		sprdbat_data->sprdbat_update_capacity_time);
	period_time =
		(int)(cur_time.tv_sec -
		sprdbat_data->sprdbat_last_query_time);
	sprdbat_data->sprdbat_last_query_time = cur_time.tv_sec;

	SPRDBAT_DEBUG("fgu_cap: = %d,flush: = %d,period:=%d,bat_status:=%d\n",
		      fgu_capacity, flush_time, period_time, sprdbat_data->bat_info.module_state);

	switch (sprdbat_data->bat_info.module_state) {
	case POWER_SUPPLY_STATUS_CHARGING:
		if (fgu_capacity < sprdbat_data->bat_info.capacity) {
			if (sprdfgu_read_batcurrent() >= 0) {
				pr_info("avoid vol jumping\n");
				fgu_capacity = sprdbat_data->bat_info.capacity;
			} else {
				if (period_time <
				    sprdbat_data->pdata->cap_one_per_time) {
					fgu_capacity =
					    sprdbat_data->bat_info.capacity - 1;
					SPRDBAT_DEBUG
						("cap decrease fgu_cap:=%d\n",
						fgu_capacity);
				}
				if ((sprdbat_data->bat_info.capacity -
				     fgu_capacity) >=
				    (flush_time /
				     sprdbat_data->pdata->cap_one_per_time)) {
					fgu_capacity =
					    sprdbat_data->bat_info.capacity -
					    flush_time /
					    sprdbat_data->pdata->
					    cap_one_per_time;
				}

			}
		} else if (fgu_capacity > sprdbat_data->bat_info.capacity) {
			if (period_time < sprdbat_data->
				pdata->cap_one_per_time) {
				fgu_capacity =
				    sprdbat_data->bat_info.capacity + 1;
				SPRDBAT_DEBUG
				    ("avoid  jumping! fgu_cap: = %d\n",
				     fgu_capacity);
			}
			if ((fgu_capacity - sprdbat_data->bat_info.capacity) >=
			    (flush_time /
			     sprdbat_data->pdata->cap_one_per_time)) {
				fgu_capacity =
				    sprdbat_data->bat_info.capacity +
				    flush_time /
				    sprdbat_data->pdata->cap_one_per_time;
			}
		}

		if ((sprdbat_data->bat_info.capacity != 100)
		    && (fgu_capacity >= 100)) {
			fgu_capacity = 99;
		}

		if ((sprdbat_data->bat_info.capacity >= 99) &&
		    (trickle_time >= sprdbat_data->pdata->trickle_timeout) &&
		    (sprdbat_data->pdata->trickle_timeout > 0)) {
			SPRDBAT_DEBUG("cap is full, but charge continue\n");
			sprdbat_change_module_state
				(SPRDBAT_CHARGING_TO_FULL_E);
		}

		if (sprdbat_data->bat_info.vbat_vol <=
		    (sprdbat_data->pdata->soft_vbat_uvlo -
		     SPRDBAT_SHUTDOWN_OFSSET)) {
			fgu_capacity = 0;
			SPRDBAT_DEBUG("soft uvlo, shutdown by kernel.. vol:%d",
				      sprdbat_data->bat_info.vbat_vol);
			orderly_poweroff(true);
		}

		break;
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
	case POWER_SUPPLY_STATUS_DISCHARGING:
		if (fgu_capacity >= sprdbat_data->bat_info.capacity) {
			fgu_capacity = sprdbat_data->bat_info.capacity;
		} else {
			if (period_time < sprdbat_data->
				pdata->cap_one_per_time) {
				fgu_capacity =
				    sprdbat_data->bat_info.capacity - 1;
				SPRDBAT_DEBUG
				    ("avoid jumping! fgu_capacity: = %d\n",
				     fgu_capacity);
			}
			if ((sprdbat_data->bat_info.capacity - fgu_capacity) >=
			    (flush_time /
			     sprdbat_data->pdata->cap_one_per_time)) {
				fgu_capacity =
				    sprdbat_data->bat_info.capacity -
				    flush_time /
				    sprdbat_data->pdata->cap_one_per_time;
			}
		}
		break;
	case POWER_SUPPLY_STATUS_FULL:
		sprdbat_data->sprdbat_update_capacity_time = cur_time.tv_sec;
		if ((sprdbat_data->bat_info.vbat_ocv <
		     (sprdbat_data->pdata->rechg_vol - 50))
		    && sprdfgu_read_batcurrent() < 0) {
			SPRDBAT_DEBUG("vbat_ocv < rechg_vol -50\n");
			if (!charger_policy_get_status()) {
				sprdbat_change_module_state(SPRDBAT_FULL_TO_CHARGING_E);
			} else {
				SPRDBAT_DEBUG("policy: force to not charging\n");
				sprdbat_data->bat_info.module_state = POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
		}
		if (fgu_capacity != 100)
			fgu_capacity = 100;

		if (sprdbat_data->bat_info.vbat_vol <=
		    (sprdbat_data->pdata->soft_vbat_uvlo -
		     SPRDBAT_SHUTDOWN_OFSSET)) {
			fgu_capacity = 0;
			SPRDBAT_DEBUG
				("soft uvlo, shutdown by kernel status full\n");
			SPRDBAT_DEBUG("vol:%d",
				sprdbat_data->bat_info.vbat_vol);
			orderly_poweroff(true);
		}

		break;
	default:
		break;
	}

	if (sprdbat_data->bat_info.vbat_vol <=
	    sprdbat_data->pdata->soft_vbat_uvlo) {
		fgu_capacity = 0;
		SPRDBAT_DEBUG("soft uvlo, vbat very low,level..0.. vol:%d",
			      sprdbat_data->bat_info.vbat_vol);
	}

	if (fgu_capacity != sprdbat_data->bat_info.capacity) {
		sprdbat_data->bat_info.capacity = fgu_capacity;
		sprdbat_data->sprdbat_update_capacity_time = cur_time.tv_sec;
		sprdfgu_record_cap(sprdbat_data->bat_info.capacity);
		power_supply_changed(sprdbat_data->battery);
	} else {
		if (sprdbat_data->bat_info.cur_temp !=
			sprdbat_data->bat_info.last_temp)
			power_supply_changed(sprdbat_data->battery);
	}

}

static void sprdbat_battery_works(struct work_struct *work)
{
	SPRDBAT_DEBUG("sprdbat_battery_works\n");

	mutex_lock(&sprdbat_data->lock);

	if (!sprdbat_data->pdata->only_vol_mode) {
		sprdbat_data->bat_info.vbat_vol = sprdbat_read_vbat_vol();
		sprdbat_data->bat_info.vbat_ocv = sprdfgu_read_vbat_ocv();
	}

	sprdbat_data->bat_info.last_temp =
			sprdbat_data->bat_info.cur_temp;
	if (jeita_debug_enable)
		sprdbat_data->bat_info.cur_temp = jeita_debug;
	else
		sprdbat_data->bat_info.cur_temp = sprdbat_read_temp();
	sprdbat_data->bat_info.bat_current = sprdfgu_read_batcurrent();
	sprdbat_data->bat_info.vchg_vol = sprdchg_read_vchg_vol();

	sprdbat_data->bat_info.avg_chg_vol =
	    sprdbat_get_avgval_from_buff(sprdbat_data->bat_info.vchg_vol,
					 chg_vol_buff, VOL_BUFF_CNT, 0);
	sprdbat_data->bat_info.bat_current_avg =
	    sprdbat_get_avgval_from_buff(sprdbat_data->bat_info.bat_current,
					 current_buff, CUR_BUFF_CNT, 1);
	if (sprdbat_data->pdata->only_vol_mode) {
		if (sprdbat_data->bat_info.module_state ==
		    POWER_SUPPLY_STATUS_DISCHARGING ||
		    sprdbat_data->bat_info.module_state ==
		    POWER_SUPPLY_STATUS_UNKNOWN) {
			sprdbat_data->bat_info.vbat_vol =
				sprdbat_read_vbat_vol();
			sprdbat_data->bat_info.vbat_ocv =
				sprdfgu_read_vbat_ocv();
			sprdbat_update_capacty();
		}
	} else {
		sprdbat_update_capacty();
	}

	mutex_unlock(&sprdbat_data->lock);
	sprdbat_print_battery_log();
	queue_delayed_work(system_power_efficient_wq,
			   &sprdbat_data->battery_work,
			   15 * HZ);
}

static void sprdbat_battery_sleep_works(struct work_struct *work)
{
	SPRDBAT_DEBUG("sprdbat_battery_sleep_works\n");
	if (!queue_delayed_work(system_power_efficient_wq,
	    &sprdbat_data->battery_work, 0)) {
		cancel_delayed_work_sync(&sprdbat_data->battery_work);
		queue_delayed_work(system_power_efficient_wq,
			&sprdbat_data->battery_work, 0);
	}
}

static void sprdbat_charge_works(struct work_struct *work)
{
	SPRDBAT_DEBUG("sprdbat_charge_works----------start\n");
	mutex_lock(&sprdbat_data->lock);

	if (!sprdbat_data->pdata->only_vol_mode) {
		sprdbat_data->bat_info.vbat_vol = sprdbat_read_vbat_vol();
		sprdbat_data->bat_info.vbat_ocv = sprdfgu_read_vbat_ocv();
	}
	sprdbat_data->bat_info.bat_current = sprdfgu_read_batcurrent();

	if (sprd_ext_ic_op->timer_callback_ext)
		sprd_ext_ic_op->timer_callback_ext();
	if (sprdbat_data->bat_info.module_state ==
	    POWER_SUPPLY_STATUS_DISCHARGING) {
		SPRDBAT_DEBUG("not charing return\n");
		mutex_unlock(&sprdbat_data->lock);
		return;
	}

	if (sprdbat_data->pdata->only_vol_mode &&
		!sprdchg_timer_op->timer_enable) {
		mutex_unlock(&sprdbat_data->lock);
		return;
	}

	if (sprdbat_data->pdata->only_vol_mode) {
		unsigned int poll_time_fast =
				sprdbat_data->pdata->chg_polling_time_fast;
		unsigned int poll_time =
				sprdbat_data->pdata->chg_polling_time;
		if (sprdbat_data->bat_info.chg_stop_flags ==
			SPRDBAT_CHG_END_NONE_BIT) {
			if (sprdbat_data->bat_info.chging_on) {
				sprd_ext_ic_op->charge_stop_ext
					(SPRDBAT_CHG_END_NONE_BIT);
				sprdbat_data->bat_info.chging_on = 0;
				sprdchg_timer_op->timer_disable();
				sprdchg_timer_op->timer_enable(poll_time_fast,
							       ONE_TIME);
				mutex_unlock(&sprdbat_data->lock);
				return;
			}
			sprdbat_data->bat_info.vbat_vol =
				sprdbat_read_vbat_vol();
			sprdbat_data->bat_info.vbat_ocv =
				sprdfgu_read_vbat_ocv();
			sprdbat_update_capacty();
			sprdbat_data->bat_info.chging_on = 1;
			sprd_ext_ic_op->charge_start_ext();
			msleep(20);
			sprdchg_timer_op->timer_disable();
			sprdchg_timer_op->timer_enable(poll_time, ONE_TIME);
		} else {
			sprdbat_data->bat_info.vbat_vol =
				sprdbat_read_vbat_vol();
			sprdbat_data->bat_info.vbat_ocv =
				sprdfgu_read_vbat_ocv();
			sprdbat_update_capacty();
			sprdchg_timer_op->timer_disable();
			sprdchg_timer_op->timer_enable(poll_time, ONE_TIME);
		}
	}

	if (sprdbat_data->bat_info.chg_stop_flags & SPRDBAT_CHG_END_FULL_BIT)
		sprdbat_chg_rechg_monitor();

	sprdbat_chg_status_monitor();
	sprdbat_chg_timeout_monitor();
	sprdbat_chg_ovp_monitor();
	sprdbat_temp_monitor();
	sprdbat_chgr_temp_monitor();
	sprdbat_fault_monitor();
	mutex_unlock(&sprdbat_data->lock);

	sprdbat_chg_print_log();
	SPRDBAT_DEBUG("sprdbat_charge_works----------end\n");

}

static void print_pdata(struct sprd_battery_platform_data *pdata)
{
	int i;

	SPRDBAT_DEBUG("chg_end_vol_l:%d\n", pdata->chg_end_vol_l);
	SPRDBAT_DEBUG("chg_end_vol_pure:%d\n", pdata->chg_end_vol_pure);
	SPRDBAT_DEBUG("chg_bat_safety_vol:%d\n", pdata->chg_bat_safety_vol);
	SPRDBAT_DEBUG("rechg_vol:%d\n", pdata->rechg_vol);
	SPRDBAT_DEBUG("adp_cdp_cur:%d\n", pdata->adp_cdp_cur);
	SPRDBAT_DEBUG("adp_dcp_cur:%d\n", pdata->adp_dcp_cur);
	SPRDBAT_DEBUG("adp_sdp_cur:%d\n", pdata->adp_sdp_cur);
	SPRDBAT_DEBUG("adp_unknown_cur:%d\n", pdata->adp_unknown_cur);
	SPRDBAT_DEBUG("adp_fchg_cur:%d\n", pdata->adp_fchg_cur);
	SPRDBAT_DEBUG("ovp_stop:%d\n", pdata->ovp_stop);
	SPRDBAT_DEBUG("ovp_restart:%d\n", pdata->ovp_restart);
	SPRDBAT_DEBUG("fchg-ovp_stop:%d\n", pdata->fchg_ovp_stop);
	SPRDBAT_DEBUG("fchg-ovp_restart:%d\n", pdata->fchg_ovp_restart);
	SPRDBAT_DEBUG("chg_timeout:%d\n", pdata->chg_timeout);
	SPRDBAT_DEBUG("chg_rechg_timeout:%d\n", pdata->chg_rechg_timeout);
	SPRDBAT_DEBUG("chg_end_cur:%d\n", pdata->chg_end_cur);
	SPRDBAT_DEBUG("fchg_vol:%d\n", pdata->fchg_vol);
	SPRDBAT_DEBUG("chg_polling_time:%d\n", pdata->chg_polling_time);
	SPRDBAT_DEBUG("chg_polling_time_fast:%d\n",
		      pdata->chg_polling_time_fast);
	SPRDBAT_DEBUG("trickle_timeout:%d\n", pdata->trickle_timeout);
	SPRDBAT_DEBUG("cap_one_per_time:%d\n", pdata->cap_one_per_time);
	SPRDBAT_DEBUG("cap_one_per_time_fast:%d\n",
		      pdata->cap_one_per_time_fast);
	SPRDBAT_DEBUG("cap_valid_range_poweron:%d\n",
		      pdata->cap_valid_range_poweron);
	SPRDBAT_DEBUG("temp_support:%d\n", pdata->temp_support);
	SPRDBAT_DEBUG("temp_comp_res:%d\n", pdata->temp_comp_res);
	SPRDBAT_DEBUG("temp_tab_size:%d\n", pdata->temp_tab_size);
	SPRDBAT_DEBUG("gpio_vchg_detect:%d\n", pdata->gpio_vchg_detect);
	SPRDBAT_DEBUG("only_vol_mode:%d\n", pdata->only_vol_mode);
	SPRDBAT_DEBUG("fgu_mode:%d\n", pdata->fgu_mode);
	SPRDBAT_DEBUG("alm_soc:%d\n", pdata->alm_soc);
	SPRDBAT_DEBUG("alm_vol:%d\n", pdata->alm_vol);
	SPRDBAT_DEBUG("soft_vbat_uvlo:%d\n", pdata->soft_vbat_uvlo);
	SPRDBAT_DEBUG("soft_vbat_ovp:%d\n", pdata->soft_vbat_ovp);
	SPRDBAT_DEBUG("rint:%d\n", pdata->rint);
	SPRDBAT_DEBUG("cnom:%d\n", pdata->cnom);
	SPRDBAT_DEBUG("rsense_real:%d\n", pdata->rsense_real);
	SPRDBAT_DEBUG("rsense_spec:%d\n", pdata->rsense_spec);
	SPRDBAT_DEBUG("relax_current:%d\n", pdata->relax_current);
	SPRDBAT_DEBUG("fgu_cal_ajust:%d\n", pdata->fgu_cal_ajust);
	SPRDBAT_DEBUG("qmax_update_period:%d\n", pdata->qmax_update_period);

	SPRDBAT_DEBUG("charge_vol_tab_size:%d\n", pdata->charge_vol_tab_size);
	SPRDBAT_DEBUG("discharge_vol_tab_size:%d\n",
		pdata->discharge_vol_tab_size);
	SPRDBAT_DEBUG("ocv_tab_size:%d\n", pdata->ocv_tab_size);

	SPRDBAT_DEBUG("jeita_tab_size:%d\n", pdata->jeita_tab_size);

	SPRDBAT_DEBUG("adp_cdp_cur_limit:%d\n", pdata->adp_cdp_cur_limit);
	SPRDBAT_DEBUG("adp_dcp_cur_limit:%d\n", pdata->adp_dcp_cur_limit);
	SPRDBAT_DEBUG("adp_sdp_cur_limit:%d\n", pdata->adp_sdp_cur_limit);
	SPRDBAT_DEBUG("adp_unknown_cur_limit:%d\n",
			pdata->adp_unknown_cur_limit);
	SPRDBAT_DEBUG("adp_fchg_cur_limit:%d\n", pdata->adp_fchg_cur_limit);
	SPRDBAT_DEBUG("chgr_otp_stop:%d\n", pdata->chgr_otp_stop);
	SPRDBAT_DEBUG("chgr_otp_restart:%d\n", pdata->chgr_otp_restart);
	if (pdata->chgr_thmz)
		SPRDBAT_DEBUG("chgr_thmz:%s\n", pdata->chgr_thmz->type);
	else
		SPRDBAT_DEBUG("No chargr OTP\n");

	for (i = 0; i < pdata->charge_vol_tab_size; i++) {
		SPRDBAT_DEBUG("charge_vol_tab i=%d x:%d,y:%d\n", i,
			pdata->charge_vol_tab[i].x, pdata->charge_vol_tab[i].y);
	}

	for (i = 0; i < pdata->discharge_vol_tab_size; i++) {
		SPRDBAT_DEBUG("discharge_vol_tab i=%d x:%d,y:%d\n",
			i, pdata->discharge_vol_tab[i].x,
			pdata->discharge_vol_tab[i].y);
	}

	for (i = 0; i < pdata->ocv_tab_size; i++) {
		SPRDBAT_DEBUG("ocv_tab i=%d x:%d,y:%d\n", i,
			      pdata->ocv_tab[i].x, pdata->ocv_tab[i].y);
	}

	for (i = 0; i < pdata->temp_tab_size; i++) {
		SPRDBAT_DEBUG("temp_tab_size i=%d x:%d,y:%d\n", i,
			      pdata->temp_tab[i].x, pdata->temp_tab[i].y);
	}

	for (i = 0; i < pdata->cnom_temp_tab_size; i++) {
		SPRDBAT_DEBUG("cnom_temp_tab i=%d x:%d,y:%d\n", i,
			      pdata->cnom_temp_tab[i].x,
			      pdata->cnom_temp_tab[i].y);
	}

	for (i = 0; i < pdata->rint_temp_tab_size; i++) {
		SPRDBAT_DEBUG("rint_temp_tab i=%d x:%d,y:%d\n", i,
			      pdata->rint_temp_tab[i].x,
			      pdata->rint_temp_tab[i].y);
	}

	for (i = 0; i < pdata->jeita_tab_size; i++) {
		SPRDBAT_DEBUG("jeita_tab i=%d,w:%d x:%d,y:%d:z:%d\n", i,
			      pdata->jeita_tab[i].w, pdata->jeita_tab[i].x,
			      pdata->jeita_tab[i].y, pdata->jeita_tab[i].z);
	}
	SPRDBAT_DEBUG("jeita_tab i=%d y:%d:z:%d\n",
		i, pdata->jeita_tab[i].y, pdata->jeita_tab[i].z);
}

static int bat_id_kohm;
module_param_named(
	bat_id_kohm, bat_id_kohm, int, S_IRUSR | S_IWUSR
);

#define BAT_ID_PULL_UP 475
#define BAT_ID_REF_VOL 1850
#define VOL_V_PER_MV 1000

int __weak sprdbat_get_bat_id(struct platform_device *pdev)
{
	struct iio_channel *channel_batt_id;
	int batt_id_vol;
	int batt_id = 0;

	channel_batt_id = iio_channel_get(&pdev->dev, "adc_batt_id");
	if (IS_ERR(channel_batt_id))
		return PTR_ERR(channel_batt_id);
	batt_id_vol = sprdchg_read_ntc_vol(channel_batt_id);
	SPRDBAT_DEBUG("%s batt_id_vol:%d\n", __func__, batt_id_vol);

	iio_channel_release(channel_batt_id);

	batt_id = DIV_ROUND_CLOSEST((batt_id_vol * VOL_V_PER_MV * BAT_ID_PULL_UP),
		(BAT_ID_REF_VOL * VOL_V_PER_MV - batt_id_vol * VOL_V_PER_MV));
	bat_id_kohm = batt_id;

	SPRDBAT_DEBUG("%s bat id:%d\n", __func__, batt_id);
	return batt_id;
}

static int (*sprdbat_bat_fun[4])(struct platform_device *pdev) = {
	sprdbat_get_bat_id,
};

static void sprdbat_chgr_otp_build(struct device_node *np,
			struct sprd_battery_platform_data *pdata)
{
	struct device_node *chg_z;
	struct thermal_zone_device *tzd;

	if (!np || !pdata)
		return;

	if (of_property_read_s32(np, "chgr-otp-stop",
		&pdata->chgr_otp_stop)) {
		SPRDBAT_DEBUG("NOT support chgr otp!\n");
		return;
	}
	/* dts temp must be 1000 + xC */
	pdata->chgr_otp_stop -= 1000;

	if (of_property_read_s32(np, "chgr-otp-restart",
		&pdata->chgr_otp_restart))
		/* if dts don't define restart temp, use stop temp - 5C */
		pdata->chgr_otp_restart = pdata->chgr_otp_stop - 50;
	else
		pdata->chgr_otp_restart -= 1000;

	chg_z = of_parse_phandle(np, "charger-thmzone", 0);
	if (!chg_z) {
		SPRDBAT_DEBUG("Not find charger-thmzone!\n");
		return;
	}
	tzd = thermal_zone_get_zone_by_name(chg_z->name);
	if (IS_ERR(tzd)) {
		SPRDBAT_DEBUG("Not find %s!\n", chg_z->name);
		return;
	}

	pdata->chgr_thmz = tzd;
	SPRDBAT_DEBUG("Get %s OK!\n", chg_z->name);
}

#define DEFAULT_FULL_DESIGN_CAPACITY 3200
static struct sprd_battery_platform_data *sprdbat_parse_dt(struct
							   platform_device
							   *pdev)
{
	struct sprd_battery_platform_data *pdata = NULL;
	struct device_node *np = pdev->dev.of_node;
	int i, temp, len;
	u32 fun_num, bat_adapt;
	int bat_id = 0;
	int bat_id_kohm = 0;
	int bat_id_index = 0;

	pdata = devm_kzalloc(&pdev->dev,
			     sizeof(struct sprd_battery_platform_data),
			     GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	if (!of_property_read_u32(np, "battery-adapt-fun",
		&fun_num) &&
		!of_property_read_u32(np, "battery-adapt-support",
		&bat_adapt)) {
		if (bat_adapt)
			bat_id = sprdbat_bat_fun[fun_num](pdev);
	}

	if (bat_id) {
		struct device_node *child;

		for_each_child_of_node(np, child) {
			u32 r;

			if (!child->name || of_node_cmp(child->name, "battery"))
				continue;

			if (of_property_read_u32(child, "reg", &r) < 0)
				continue;

			if (r == bat_id) {
				np = child;
				break;
			}
		}
		SPRDBAT_DEBUG("Sub-nodes: battery@%d %s.\n", bat_id,
			      np == child ? "enabled" : "failed");
	}

	if (of_get_property(np, "bat-id-list", &len)) {
		len /= sizeof(u32);
		pdata->batt_id_list_size = len;
		pdata->batt_id_list =
			devm_kzalloc(&pdev->dev,
			sizeof(int) * pdata->batt_id_list_size, GFP_KERNEL);
		if (!pdata->batt_id_list)
			return ERR_PTR(-ENOMEM);
		for (i = 0; i < pdata->batt_id_list_size; i++) {
			if (of_property_read_u32_index(np, "bat-id-list",
				i, &pdata->batt_id_list[i])) {
				dev_err(&pdev->dev, "parse bat-id-list err\n");
			}
			SPRDBAT_DEBUG("batt_id_list index=%d, value=%d K\n", i, pdata->batt_id_list[i]);
		}

		bat_id_kohm = sprdbat_get_bat_id(pdev);
		for (i = 0; i < pdata->batt_id_list_size; i++) {
			/* batt id is in + - 20%
			*  and if batt id is 0 komh, we use 0-5k as the scope, so, +5
			*/
			if (bat_id_kohm >= pdata->batt_id_list[i] * 80/100
				&& bat_id_kohm <= pdata->batt_id_list[i] * 120/100 + 5) {
				bat_id_index = i;
				SPRDBAT_DEBUG("near the batt id %d kohm, bat_id_index=%d\n",
					pdata->batt_id_list[i], bat_id_index);
				break;
			}
		}

		if (i >= pdata->batt_id_list_size) {
			SPRDBAT_DEBUG("not find the batt id\n");
			bat_id_index = 0;
		}
	}

	temp = of_get_named_gpio(np, "charger-det-gpios", 0);
	if (gpio_is_valid(temp)) {
		pdata->gpio_vchg_detect = (uint32_t) temp;
		SPRDBAT_DEBUG("gpio_vchg_detect =%d\n",
			      pdata->gpio_vchg_detect);
	} else {
		pdata->gpio_vchg_detect = 0;
		SPRDBAT_DEBUG("gpio_vchg_detect do Not support =%d\n",
			      pdata->gpio_vchg_detect);
	}

	temp = of_get_named_gpio(np, "battery-det-gpios", 0);
	if (gpio_is_valid(temp)) {
		pdata->gpio_vbat_detect = (uint32_t) temp;
		SPRDBAT_DEBUG("gpio_vbat_detect support =%d\n",
			      pdata->gpio_vbat_detect);
	} else {
		pdata->gpio_vbat_detect = 0;
		SPRDBAT_DEBUG("gpio_vbat_detect do Not support =%d\n",
			      pdata->gpio_vbat_detect);
	}

	if (of_property_read_u32(np, "chg-end-vol-check",
		&pdata->chg_end_vol_l))
		dev_err(&pdev->dev, "parse chg-end-vol-check err\n");
	if (of_property_read_u32(np, "chg-end-vol",
		&pdata->chg_end_vol_pure))
		dev_err(&pdev->dev, "parse chg-end-vol err\n");
	if (of_property_read_u32(np, "fchg-vol",
		&pdata->fchg_vol))
		dev_err(&pdev->dev, "parse fchg-vol err\n");

	if (of_property_read_u32(np, "chg-bat-safety-vol",
		&pdata->chg_bat_safety_vol))
		dev_err(&pdev->dev, "parse chg-bat-safety-vol err\n");

	if (of_property_read_u32(np, "rechg-vol", &pdata->rechg_vol))
		dev_err(&pdev->dev, "parse rechg-vol err\n");

	if (of_property_read_u32(np, "adp-cdp-cur", &pdata->adp_cdp_cur))
		dev_err(&pdev->dev, "parse adp-cdp-cur err\n");
	if (of_property_read_u32(np, "adp-dcp-cur", &pdata->adp_dcp_cur))
		dev_err(&pdev->dev, "parse adp-dcp-cur err\n");
	if (of_property_read_u32(np, "adp-sdp-cur", &pdata->adp_sdp_cur))
		dev_err(&pdev->dev, "parse adp-sdp-cur err\n");
	if (of_property_read_u32(np, "adp-unknown-cur",
		&pdata->adp_unknown_cur)) {
		dev_err(&pdev->dev, "parse adp-unknown-cur err\n");
		pdata->adp_unknown_cur = pdata->adp_sdp_cur;
	}
	if (of_property_read_u32(np, "adp-fchg-cur", &pdata->adp_fchg_cur))
		dev_err(&pdev->dev, "parse adp-fchg-cur err\n");

	if (of_property_read_u32(np, "adp-cdp-cur-limit",
				 &pdata->adp_cdp_cur_limit))
		dev_err(&pdev->dev, "parse adp-cdp-cur-limit err\n");
	if (of_property_read_u32(np, "adp-dcp-cur-limit",
				 &pdata->adp_dcp_cur_limit))
		dev_err(&pdev->dev, "parse adp-dcp-cur-limit err\n");
	if (of_property_read_u32(np, "adp-sdp-cur-limit",
				 &pdata->adp_sdp_cur_limit))
		dev_err(&pdev->dev, "parse adp-sdp-cur-limit err\n");
	if (of_property_read_u32(np, "adp-unknown-cur-limit",
		&pdata->adp_unknown_cur_limit)) {
		dev_err(&pdev->dev, "parse adp-unknown-cur-limit err\n");
		pdata->adp_unknown_cur_limit = pdata->adp_dcp_cur_limit;
	}
	if (of_property_read_u32(np, "adp-fchg-cur-limit",
				 &pdata->adp_fchg_cur_limit))
		dev_err(&pdev->dev, "parse adp-fchg-cur err\n");

	if (of_property_read_u32(np, "ovp-stop", &pdata->ovp_stop))
		dev_err(&pdev->dev, "parse ovp-stop err\n");
	if (of_property_read_u32(np, "ovp-restart", &pdata->ovp_restart))
		dev_err(&pdev->dev, "parse ovp-restart err\n");

	if (of_property_read_u32(np, "fchg-ovp-stop", &pdata->fchg_ovp_stop))
		dev_err(&pdev->dev, "parse fchg-ovp-stop err\n");
	if (of_property_read_u32(np, "fchg-ovp-restart",
		&pdata->fchg_ovp_restart))
		dev_err(&pdev->dev, "parse fchg-ovp-restart err\n");

	if (of_property_read_u32(np, "chg-timeout", &pdata->chg_timeout))
		dev_err(&pdev->dev, "parse chg-timeout err\n");
	if (of_property_read_u32(np, "chg-rechg-timeout",
		&pdata->chg_rechg_timeout))
		dev_err(&pdev->dev, "parse chg-rechg-timeout err\n");

	if (of_property_read_u32(np, "trickle-timeout",
		&pdata->trickle_timeout))
		pdata->trickle_timeout = 60 * 25;

	if (of_property_read_u32(np, "chg-end-cur",
		(u32 *) (&pdata->chg_end_cur)))
		dev_err(&pdev->dev, "parse chg-end-cur err\n");

	if (of_property_read_u32(np, "chg-polling-time",
		&pdata->chg_polling_time))
		dev_err(&pdev->dev, "parse chg-polling-time err\n");
	if (of_property_read_u32(np, "chg-polling-time-fast",
		&pdata->chg_polling_time_fast))
		dev_err(&pdev->dev, "parse chg-polling-time-fast err\n");

	if (of_property_read_u32(np, "cap-one-per-time",
		&pdata->cap_one_per_time))
		dev_err(&pdev->dev, "parse cap_one_per_time err\n");
	if (of_property_read_u32(np, "cap-valid-range-poweron",
				 (u32 *) (&pdata->cap_valid_range_poweron)))
		dev_err(&pdev->dev, "parse cap-valid-range-poweron err\n");

	if (of_property_read_u32(np, "temp-support",
		(u32 *) (&pdata->temp_support)))
		dev_err(&pdev->dev, "parse temp-support err\n");
	if (of_property_read_u32(np, "temp-comp-res",
		(u32 *) (&pdata->temp_comp_res)))
		dev_err(&pdev->dev, "parse temp-comp-res err\n");

	if (of_property_read_u32(np, "only-vol-mode", &pdata->only_vol_mode))
		dev_warn(&pdev->dev, "parse only-vol-mode err\n");

	if (of_property_read_u32(np, "fgu-mode", &pdata->fgu_mode))
		dev_err(&pdev->dev, "parse fgu-mode err\n");

	if (of_property_read_u32(np, "chg-full-condition",
		&pdata->chg_full_condition))
		dev_err(&pdev->dev, "parse chg-full-condition err\n");

	if (of_property_read_u32(np, "alm-soc", &pdata->alm_soc))
		dev_err(&pdev->dev, "parse alm-soc err\n");
	if (of_property_read_u32(np, "alm-vol", &pdata->alm_vol))
		dev_err(&pdev->dev, "parse alm-vol err\n");
	if (of_property_read_u32(np, "soft-vbat-uvlo",
		&pdata->soft_vbat_uvlo))
		dev_err(&pdev->dev, "parse soft-vbat-uvlo err\n");
	if (bat_id_index == 0) {
		if (of_property_read_u32(np, "rint", (u32 *) (&pdata->rint)))
			dev_err(&pdev->dev, "parse rint err\n");

		if (of_property_read_u32(np, "cnom", (u32 *) (&pdata->cnom)))
			dev_err(&pdev->dev, "parse cnom err\n");
	} else if (bat_id_index == 1) {
		if (of_property_read_u32(np, "rint-1", (u32 *) (&pdata->rint)))
			dev_err(&pdev->dev, "parse rint err\n");

		if (of_property_read_u32(np, "cnom-1", (u32 *) (&pdata->cnom)))
			dev_err(&pdev->dev, "parse cnom err\n");
	} else if (bat_id_index == 2) {
		if (of_property_read_u32(np, "rint-2", (u32 *) (&pdata->rint)))
			dev_err(&pdev->dev, "parse rint err\n");

		if (of_property_read_u32(np, "cnom-2", (u32 *) (&pdata->cnom)))
			dev_err(&pdev->dev, "parse cnom err\n");
	}

	if (of_property_read_u32(np, "rsense-real",
		(u32 *) (&pdata->rsense_real)))
		dev_err(&pdev->dev, "parse rsense-real err\n");
	if (of_property_read_u32(np, "rsense-spec",
		(u32 *) (&pdata->rsense_spec)))
		dev_err(&pdev->dev, "parse rsense-spec err\n");

	if (of_property_read_u32(np, "relax-current",
		&pdata->relax_current))
		dev_err(&pdev->dev, "parse relax-current err\n");
	if (of_property_read_u32(np, "fgu-cal-ajust",
		(u32 *) (&pdata->fgu_cal_ajust)))
		dev_err(&pdev->dev, "parse fgu-cal-ajust err\n");

	if (of_property_read_u32(np, "cnom-track-support",
		&pdata->cnom_track_support))
		pdata->cnom_track_support = 0;

	if (of_get_property(np, "temp-tab-val", &len)) {
		len /= sizeof(u32);
		pdata->temp_tab_size = len;
		pdata->temp_tab =
			devm_kzalloc(&pdev->dev,
			sizeof(struct sprdbat_table_data) *
			pdata->temp_tab_size, GFP_KERNEL);
		if (!pdata->temp_tab)
			return ERR_PTR(-ENOMEM);
		for (i = 0; i < pdata->temp_tab_size; i++) {
			if (of_property_read_u32_index(np, "temp-tab-val",
				i, &pdata->temp_tab[i].x))
				dev_err(&pdev->dev, "parse temp-tab-val err\n");
			if (of_property_read_u32_index(np, "temp-tab-temp",
				i, &temp))
				dev_err(&pdev->dev, "parse temp-tab-temp err\n");
			else
				pdata->temp_tab[i].y = temp - 1000;
		}
	}

	if (of_get_property(np, "charge-vol-tab", &len)) {
		int tab_size;

		pdata->charge_vol_tab_size = len / sizeof(u32);
		tab_size = pdata->charge_vol_tab_size;
		pdata->charge_vol_tab = devm_kzalloc(&pdev->dev,
			sizeof(struct sprdbat_table_data) * tab_size,
			       GFP_KERNEL);
		if (!pdata->charge_vol_tab)
			return ERR_PTR(-ENOMEM);
		for (i = 0; i < pdata->charge_vol_tab_size; i++) {
			if (of_property_read_u32_index(np, "charge-vol-tab",
				i, (u32 *) (&pdata->charge_vol_tab[i].x)))
				dev_warn(&pdev->dev, "parse charge-vol-tab err\n");
			if (of_property_read_u32_index(np, "charge-vol-tab-cap",
				i, (u32 *) (&pdata->charge_vol_tab[i].y)))
				dev_warn(&pdev->dev, "parse charge-vol-tab-cap err\n");
		}
	}

	if (of_get_property(np, "discharge-vol-tab", &len)) {
		int tab_size;

		pdata->discharge_vol_tab_size = len / sizeof(u32);
		tab_size = pdata->discharge_vol_tab_size;
		pdata->discharge_vol_tab = devm_kzalloc(&pdev->dev,
			sizeof(struct sprdbat_table_data) * tab_size,
				   GFP_KERNEL);
		if (!pdata->discharge_vol_tab)
			return ERR_PTR(-ENOMEM);
		for (i = 0; i < pdata->discharge_vol_tab_size; i++) {
			if (of_property_read_u32_index(np, "discharge-vol-tab",
				i, (u32 *) (&pdata->discharge_vol_tab[i].x)))
				dev_warn(&pdev->dev, "parse discharge-vol-tab err\n");
			if (of_property_read_u32_index(np,
				"discharge-vol-tab-cap", i,
				(u32 *) (&pdata->discharge_vol_tab[i].y)))
				dev_warn(&pdev->dev, "parse discharge-vol-tab-cap err\n");
		}
	}

	if (bat_id_index == 0) {
		if (of_get_property(np, "ocv-tab-vol", &len)) {
			len /= sizeof(u32);
			pdata->ocv_tab_size = len;
			pdata->ocv_tab =
				devm_kzalloc(&pdev->dev,
				sizeof(struct sprdbat_table_data) *
				pdata->ocv_tab_size, GFP_KERNEL);
			if (!pdata->ocv_tab)
				return ERR_PTR(-ENOMEM);
			for (i = 0; i < pdata->ocv_tab_size; i++) {
				if (of_property_read_u32_index(np, "ocv-tab-vol", i,
					(u32 *) (&pdata->ocv_tab[i].x)))
					dev_err(&pdev->dev, "parse ocv-tab-vol err\n");
				if (of_property_read_u32_index(np, "ocv-tab-cap", i,
					(u32 *) (&pdata->ocv_tab[i].y)))
					dev_err(&pdev->dev, "parse ocv-tab-cap err\n");
			}
		}
	} else if (bat_id_index == 1) {
		if (of_get_property(np, "ocv-tab-vol-1", &len)) {
			len /= sizeof(u32);
			pdata->ocv_tab_size = len;
			pdata->ocv_tab =
				devm_kzalloc(&pdev->dev,
				sizeof(struct sprdbat_table_data) *
				pdata->ocv_tab_size, GFP_KERNEL);
			if (!pdata->ocv_tab)
				return ERR_PTR(-ENOMEM);
			for (i = 0; i < pdata->ocv_tab_size; i++) {
				if (of_property_read_u32_index(np, "ocv-tab-vol-1", i,
					(u32 *) (&pdata->ocv_tab[i].x)))
					dev_err(&pdev->dev, "parse ocv-tab-vol err\n");
				if (of_property_read_u32_index(np, "ocv-tab-cap", i,
					(u32 *) (&pdata->ocv_tab[i].y)))
					dev_err(&pdev->dev, "parse ocv-tab-cap err\n");
			}
		}
	} else if (bat_id_index == 2) {
		if (of_get_property(np, "ocv-tab-vol-2", &len)) {
			len /= sizeof(u32);
			pdata->ocv_tab_size = len;
			pdata->ocv_tab =
				devm_kzalloc(&pdev->dev,
				sizeof(struct sprdbat_table_data) *
				pdata->ocv_tab_size, GFP_KERNEL);
			if (!pdata->ocv_tab)
				return ERR_PTR(-ENOMEM);
			for (i = 0; i < pdata->ocv_tab_size; i++) {
				if (of_property_read_u32_index(np, "ocv-tab-vol-2", i,
					(u32 *) (&pdata->ocv_tab[i].x)))
					dev_err(&pdev->dev, "parse ocv-tab-vol err\n");
				if (of_property_read_u32_index(np, "ocv-tab-cap", i,
					(u32 *) (&pdata->ocv_tab[i].y)))
					dev_err(&pdev->dev, "parse ocv-tab-cap err\n");
			}
		}
	}

	if (of_get_property(np, "jeita-temp-tab", &len)) {
		len /= sizeof(u32);
		pdata->jeita_tab_size = len;
		pdata->jeita_tab =
			devm_kzalloc(&pdev->dev,
			sizeof(struct sprdbat_jeita_table_data) *
			(pdata->jeita_tab_size + 1), GFP_KERNEL);
		if (!pdata->jeita_tab)
			return ERR_PTR(-ENOMEM);
		for (i = 0; i < pdata->jeita_tab_size; i++) {
			if (of_property_read_u32_index(np, "jeita-temp-tab", i,
				(u32 *) (&pdata->jeita_tab[i].x)))
				dev_err(&pdev->dev, "parse jeita-temp-tab err\n");
			else
				pdata->jeita_tab[i].x -= 1000;
			if (of_property_read_u32_index(np,
				"jeita-temp-recovery-tab", i,
				(u32 *) (&pdata->jeita_tab[i].w)))
				dev_err(&pdev->dev, "parse jeita-temp-recovery-tab err\n");
			else
				pdata->jeita_tab[i].w -= 1000;
			if (of_property_read_u32_index(np, "jeita-cur-tab", i,
				(u32 *) (&pdata->jeita_tab[i].y)))
				dev_err(&pdev->dev, "parse jeita-cur-tab err\n");
			if (of_property_read_u32_index(np, "jeita-cccv-tab", i,
				(u32 *) (&pdata->jeita_tab[i].z)))
				dev_err(&pdev->dev, "parse jeita-cccv-tab err\n");
		}
		if (of_property_read_u32_index(np, "jeita-cur-tab", i,
			(u32 *) (&pdata->jeita_tab[i].y)))
			dev_err(&pdev->dev, "parse jeita-cur-tab err\n");
		if (of_property_read_u32_index(np, "jeita-cccv-tab", i,
			(u32 *) (&pdata->jeita_tab[i].z)))
			dev_err(&pdev->dev, "parse jeita-cccv-tab err\n");
	}

	if (of_get_property(np, "cnom-temp-tab", &len)) {
		len /= sizeof(u32);
		pdata->cnom_temp_tab_size = len >> 1;
		pdata->cnom_temp_tab =
		devm_kzalloc(&pdev->dev,
			sizeof(struct sprdbat_table_data) *
			pdata->cnom_temp_tab_size, GFP_KERNEL);
		if (!pdata->cnom_temp_tab)
			return ERR_PTR(-ENOMEM);
		for (i = 0; i < len; i++) {
			if (of_property_read_u32_index(np,
				"cnom-temp-tab", i, &temp))
				dev_err(&pdev->dev,
					"parse cnom-temp-tab err\n");
			if (i & 0x1)
				pdata->cnom_temp_tab[i >> 1].y = temp;
			else
				pdata->cnom_temp_tab[i >> 1].x = temp - 1000;
		}
	}

	if (of_get_property(np, "rint-temp-tab", &len)) {
		len /= sizeof(u32);
		pdata->rint_temp_tab_size = len >> 1;
		pdata->rint_temp_tab =
			devm_kzalloc(&pdev->dev,
			sizeof(struct sprdbat_table_data) *
			pdata->rint_temp_tab_size, GFP_KERNEL);
		if (!pdata->rint_temp_tab)
			return ERR_PTR(-ENOMEM);
		for (i = 0; i < len; i++) {
			if (of_property_read_u32_index(np,
				"rint-temp-tab", i, &temp))
				dev_err(&pdev->dev,
					"parse rint-temp-tab err\n");
			if (i & 0x1)
				pdata->rint_temp_tab[i >> 1].y = temp;
			else
				pdata->rint_temp_tab[i >> 1].x = temp - 1000;
		}
	}

	if (of_property_read_u32(np, "batt_full_design_capacity",
		(u32 *) (&pdata->batt_full_design_capacity))) {
		pdata->batt_full_design_capacity = DEFAULT_FULL_DESIGN_CAPACITY;
		dev_err(&pdev->dev, "parse batt_full_design_capacity err\n");
	}

	sprdbat_chgr_otp_build(np, pdata);

	return pdata;

}

static int sprdbat_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	enum usb_charger_state usb_online_state = USB_CHARGER_DEFAULT;
	struct power_supply *ret_ptr = NULL;
	struct sprdbat_drivier_data *data = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct power_supply_desc *battery_desc = NULL,
		*ac_desc = NULL, *usb_desc = NULL;
	struct power_supply_config battery_cfg = {}, ac_cfg = {}, usb_cfg = {};

	if (!np) {
		dev_err(&pdev->dev, "device node not found\n");
		return -EINVAL;
	}
	if (sprd_ext_ic_op == NULL) {
		dev_err(&pdev->dev, "sprd_ext_ic_op not found\n");
		return -EINVAL;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}

	data->pdata = sprdbat_parse_dt(pdev);
	if (IS_ERR_OR_NULL(data->pdata))
		return -ENOMEM;

	data->dev = &pdev->dev;
	platform_set_drvdata(pdev, data);
	sprdbat_data = data;

	data->pdata->channel_temp = iio_channel_get(data->dev, "adc_temp");
	if (IS_ERR(data->pdata->channel_temp)) {
		ret = PTR_ERR(data->pdata->channel_temp);
		goto err_iio_get_temp;
	}
	data->pdata->channel_vbat = iio_channel_get(data->dev, "adc_vbat");
	if (IS_ERR(data->pdata->channel_vbat)) {
		ret = PTR_ERR(data->pdata->channel_vbat);
		goto err_iio_get_vbat;
	}
	data->pdata->channel_vchg = iio_channel_get(data->dev, "adc_vchg");
	if (IS_ERR(data->pdata->channel_vchg)) {
		ret = PTR_ERR(data->pdata->channel_vchg);
		goto err_iio_get_vchg;
	}
	print_pdata(sprdbat_data->pdata);
	battery_desc = devm_kzalloc(&pdev->dev,
		sizeof(struct power_supply_desc), GFP_KERNEL);
	if (battery_desc == NULL) {
		ret = -ENOMEM;
		goto err_desc_alloc_failed;
	}
	battery_desc->properties = sprdbat_battery_props;
	battery_desc->num_properties = ARRAY_SIZE(sprdbat_battery_props);
	battery_desc->get_property = sprdbat_battery_get_property;
	battery_desc->set_property = sprdbat_battery_set_property;
	battery_desc->property_is_writeable =
			sprdbat_battery_property_is_writeable;
	battery_desc->name = "battery";
	battery_desc->type = POWER_SUPPLY_TYPE_BATTERY;
	battery_desc->no_thermal = true;
	battery_cfg.drv_data = sprdbat_data;

	ac_desc = devm_kzalloc(&pdev->dev,
		sizeof(struct power_supply_desc), GFP_KERNEL);
	if (ac_desc == NULL) {
		ret = -ENOMEM;
		goto err_desc_alloc_failed;
	}
	ac_desc->properties = sprdbat_ac_props;
	ac_desc->num_properties = ARRAY_SIZE(sprdbat_ac_props);
	ac_desc->get_property = sprdbat_ac_get_property;
	ac_desc->set_property = sprdbat_ac_set_property;
	ac_desc->property_is_writeable =
			sprdbat_ac_property_is_writeable;

	ac_desc->name = "ac";
	ac_desc->type = POWER_SUPPLY_TYPE_MAINS;
	ac_desc->no_thermal = true;
	ac_cfg.drv_data = sprdbat_data;

	usb_desc = devm_kzalloc(&pdev->dev,
		sizeof(struct power_supply_desc), GFP_KERNEL);
	if (usb_desc == NULL) {
		ret = -ENOMEM;
		goto err_desc_alloc_failed;
	}
	usb_desc->properties = sprdbat_usb_props;
	usb_desc->num_properties = ARRAY_SIZE(sprdbat_usb_props);
	usb_desc->get_property = sprdbat_usb_get_property;
	usb_desc->name = "usb";
	usb_desc->type = POWER_SUPPLY_TYPE_USB;
	usb_desc->no_thermal = true;
	usb_cfg.drv_data = sprdbat_data;

	data->start_charge = sprdbat_start_charge;
	data->stop_charge = sprdbat_stop_charge;

	ret_ptr = power_supply_register(&pdev->dev, battery_desc, &battery_cfg);
	if (IS_ERR(ret_ptr)) {
		goto err_battery_failed;
	} else {
		data->battery = ret_ptr;
		data->battery->supplied_to = battery_supply_list;
		data->battery->num_supplicants =
			ARRAY_SIZE(battery_supply_list);
	}

	ret_ptr = power_supply_register(&pdev->dev, ac_desc, &ac_cfg);
	if (IS_ERR(ret_ptr)) {
		goto err_ac_failed;
	} else {
		data->ac = ret_ptr;
		data->ac->supplied_to = supply_list;
		data->ac->num_supplicants = ARRAY_SIZE(supply_list);
	}

	ret_ptr = power_supply_register(&pdev->dev, usb_desc, &usb_cfg);
	if (IS_ERR(ret_ptr)) {
		goto err_usb_failed;
	} else {
		data->usb = ret_ptr;
		data->usb->supplied_to = supply_list;
		data->usb->num_supplicants = ARRAY_SIZE(supply_list);
	}

	/*
	 * TODO: switch polling to interrupt again need open this code.
	 * data->chg_nb.notifier_call = sprdbat_chg_event_call;
	 * ret = power_supply_reg_notifier(&data->chg_nb);
	 *
	 * if (ret)
	 *	dev_err(data->dev, "failed to reg notifier: %d\n", ret);
	 */
	ret = sysfs_create_group(&data->battery->dev.kobj,
			&sprd_bat_group);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to create sprd_bat sysfs device attributes\n");
		goto err_sysfs_create_gr;
	}

	data->gpio_vbat_detect = data->pdata->gpio_vbat_detect;
	if (data->gpio_vbat_detect > 0) {
		devm_gpio_request(&pdev->dev,
			data->gpio_vbat_detect, "vbat_detect");
		gpio_direction_input(data->gpio_vbat_detect);
		data->irq_vbat_detect = gpio_to_irq(data->gpio_vbat_detect);

		irq_set_status_flags(data->irq_vbat_detect, IRQ_NOAUTOEN);
		ret =
			devm_request_threaded_irq(&pdev->dev,
			data->irq_vbat_detect, NULL,
			sprdbat_vbat_detect_irq,
			IRQ_TYPE_LEVEL_LOW | IRQF_NO_SUSPEND,
			"sprdbat_vbat_detect", data);
		if (ret)
			dev_err(&pdev->dev, "failed to use vbat gpio: %d\n",
				ret);
	}

	data->bat_info.bat_present = 1;

	mutex_init(&data->lock);

	wake_lock_init(&(data->charger_wake_lock), WAKE_LOCK_SUSPEND,
		       "charger_wake_lock");

	INIT_DELAYED_WORK(&data->battery_work, sprdbat_battery_works);
	INIT_DELAYED_WORK(&data->battery_sleep_work,
			  sprdbat_battery_sleep_works);
	INIT_WORK(&data->vbat_detect_irq_work, sprdbat_vbat_detect_irq_works);
	INIT_WORK(&data->plug_work, sprdbat_plug_works);
	INIT_DELAYED_WORK(&sprdbat_data->sprdbat_charge_work,
			  sprdbat_charge_works);
	data->monitor_wqueue = create_freezable_workqueue("sprdbat_monitor");
	if (data->monitor_wqueue == NULL)
		goto err_create_wq;

	sprdchg_init(data->pdata);
	sprdfgu_init(data->pdata);

	if (sprdchg_timer_op->timer_request)
		sprdchg_timer_op->timer_request(sprdbat_timer_handler,
		data->pdata);
	else
		SPRDBAT_DEBUG("warning !!charge timer ops = null\n");

#ifdef CONFIG_LEDS_TRIGGERS
	data->charging_led.name = "sprdbat_charging_led";
	data->charging_led.default_trigger = "battery-charging";
	data->charging_led.brightness_set = sprdchg_led_brightness_set;
	ret = led_classdev_register(&pdev->dev, &data->charging_led);
	if (ret)
		goto err_led_reg;
#endif

	sprd_ext_ic_op->ic_init(sprdbat_data);
	sprdbat_info_init(data);

	SPRDBAT_DEBUG("register_usb_notifier\n");
	sprdbat_data->usb_charger =
		usb_charger_find_by_name("usb-charger.0");
	if (IS_ERR(sprdbat_data->usb_charger)) {
		ret = -EPROBE_DEFER;
		dev_err(&pdev->dev,
			"Failed to find USB gadget: %d\n", ret);
		goto err_usb_find_name;
	}

	sprdbat_data->chg_usb_nb.notifier_call = sprdbat_usb_plug_event;

	ret = usb_charger_register_notify(sprdbat_data->usb_charger,
					  &sprdbat_data->chg_usb_nb);
	if (ret != 0) {
		dev_err(&pdev->dev,
			"Failed to register notifier: %d\n", ret);
		goto err_usb_reg_notify;
	}
	sprdbat_data->usb_charger->get_charger_type =
		sprdchg_charger_is_adapter_for_usb;
	usb_online_state = usb_charger_get_state(sprdbat_data->usb_charger);
	if (usb_online_state == USB_CHARGER_PRESENT)
		queue_work(sprdbat_data->monitor_wqueue,
		   &sprdbat_data->plug_work);

	if (data->gpio_vbat_detect > 0)
		enable_irq(sprdbat_data->irq_vbat_detect);

	queue_delayed_work(system_power_efficient_wq,
			&data->battery_work, 15 * HZ);
	SPRDBAT_DEBUG("sprdbat_probe----------end\n");
	return 0;

err_usb_reg_notify:
err_usb_find_name:
#ifdef CONFIG_LEDS_TRIGGERS
	led_classdev_unregister(&data->charging_led);
err_led_reg:
#endif
	destroy_workqueue(data->monitor_wqueue);
err_create_wq:
	sysfs_remove_group(&data->battery->dev.kobj,
			   &sprd_bat_group);
err_sysfs_create_gr:
	power_supply_unregister(data->usb);
err_usb_failed:
	power_supply_unregister(data->ac);
err_ac_failed:
	power_supply_unregister(data->battery);
err_battery_failed:
	iio_channel_release(data->pdata->channel_vchg);
err_desc_alloc_failed:
	sprdbat_data = NULL;
err_iio_get_vchg:
	iio_channel_release(data->pdata->channel_vbat);
err_iio_get_vbat:
	iio_channel_release(data->pdata->channel_temp);
err_iio_get_temp:
err_data_alloc_failed:
	sprdbat_data = NULL;
	return ret;

}

static int sprdbat_remove(struct platform_device *pdev)
{
	struct sprdbat_drivier_data *data = platform_get_drvdata(pdev);

	sysfs_remove_group(&data->battery->dev.kobj,
		&sprd_bat_group);
	if (sprdbat_data->usb_charger)
		usb_charger_unregister_notify(sprdbat_data->usb_charger,
				      &sprdbat_data->chg_usb_nb);
	flush_workqueue(data->monitor_wqueue);
	destroy_workqueue(data->monitor_wqueue);
	power_supply_unregister(data->battery);
	power_supply_unregister(data->ac);
	power_supply_unregister(data->usb);
	iio_channel_release(data->pdata->channel_vchg);
	iio_channel_release(data->pdata->channel_isense);
	iio_channel_release(data->pdata->channel_vbat);
	iio_channel_release(data->pdata->channel_temp);
	sprdbat_data = NULL;
	return 0;
}

static int sprdbat_resume(struct platform_device *pdev)
{
	schedule_delayed_work(&sprdbat_data->battery_sleep_work, 0);
	sprdfgu_pm_op(0);
	return 0;
}

static int sprdbat_suspend(struct platform_device *pdev, pm_message_t state)
{
	sprdfgu_pm_op(1);
	return 0;
}

static const struct of_device_id battery_of_match[] = {
	{.compatible = "sprd,sprd-battery",},
	{}
};

static struct platform_driver sprdbat_driver = {
	.probe = sprdbat_probe,
	.remove = sprdbat_remove,
	.suspend = sprdbat_suspend,
	.resume = sprdbat_resume,
	.driver = {
		   .name = "sprd-battery",
		   .of_match_table = of_match_ptr(battery_of_match),
		   }
};

static int __init sprd_battery_init(void)
{
	return platform_driver_register(&sprdbat_driver);
}

static void __exit sprd_battery_exit(void)
{
	platform_driver_unregister(&sprdbat_driver);
}

late_initcall(sprd_battery_init);
module_exit(sprd_battery_exit);

MODULE_AUTHOR("xianke.cui@spreadtrum.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Battery and charger driver for ext charge ic");
