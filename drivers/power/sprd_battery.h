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

#ifndef _SPRD_BATTERY_H_
#define _SPRD_BATTERY_H_

#include <linux/types.h>
#include <linux/hrtimer.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/sprd_battery_common.h>
#include <linux/iio/consumer.h>
#include <linux/spi/spi-sprd-adi.h>
#include <linux/usb/charger.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/regmap.h>

#define CHG_PSY_INNER     "chg_inner"
#define CHG_PSY_EXT        "chg_ext"

#define SPRDBAT_AUXADC_CAL_NO         0
#define SPRDBAT_AUXADC_CAL_NV         1
#define SPRDBAT_AUXADC_CAL_CHIP      2

#define SPRDBAT_CHG_END_NONE_BIT			0
#define SPRDBAT_CHG_END_FULL_BIT		BIT(0)
#define SPRDBAT_CHG_END_OTP_OVERHEAT_BIT     BIT(1)
#define SPRDBAT_CHG_END_OTP_COLD_BIT    BIT(2)
#define SPRDBAT_CHG_END_TIMEOUT_BIT		BIT(3)
#define SPRDBAT_CHG_END_OVP_BIT		BIT(4)
#define SPRDBAT_CHG_END_BAT_OVP_BIT	BIT(5)
#define SPRDBAT_CHG_END_UNSPEC		BIT(8)
#define SPRDBAT_CHG_END_FORCE_STOP_BIT	BIT(9)
#define SPRDBAT_CHG_END_CHGR_OTP_BIT	BIT(10)

#define SPRDBAT_AVERAGE_COUNT   8
#define SPRDBAT_PLUG_WAKELOCK_TIME_SEC 3

#define SPRDBAT_AUXADC_CAL_TYPE_NO         0
#define SPRDBAT_AUXADC_CAL_TYPE_NV         1
#define SPRDBAT_AUXADC_CAL_TYPE_EFUSE      2

#define CUR_ADJUST_OFFSET (100)
#define CUR_BUFF_CNT (5)
#define VOL_BUFF_CNT (3)

enum sprdbat_event {
	SPRDBAT_ADP_PLUGIN_E,
	SPRDBAT_ADP_PLUGOUT_E,
	SPRDBAT_OVI_STOP_E,
	SPRDBAT_OVI_RESTART_E,
	SPRDBAT_OTP_COLD_STOP_E,
	SPRDBAT_OTP_OVERHEAT_STOP_E,
	SPRDBAT_OTP_COLD_RESTART_E,
	SPRDBAT_OTP_OVERHEAT_RESTART_E,
	SPRDBAT_CHG_FULL_E,
	SPRDBAT_RECHARGE_E,
	SPRDBAT_CHG_TIMEOUT_E,
	SPRDBAT_CHG_TIMEOUT_RESTART_E,
	SPRDBAT_VBAT_OVP_E,
	SPRDBAT_VBAT_OVP_RESTART_E,
	SPRDBAT_CHG_UNSPEC_E,
	SPRDBAT_CHG_UNSPEC_RESTART_E,
	SPRDBAT_FULL_TO_CHARGING_E,
	SPRDBAT_CHARGING_TO_FULL_E,
	SPRDBAT_CHG_FORCE_STOP_E,
	SPRDBAT_CHG_FORCE_START_E,
	SPRDBAT_CHG_REQUEST_START_E,
	SPRDBAT_CHGR_OTP_E,
	SPRDBAT_CHGR_OTP_START_E,
};


#define sprdbat_read_vbat_vol sprdfgu_read_vbat_vol
#define sprdbat_read_temp sprdchg_read_temp
#define sprdbat_adp_plug_nodify sprdfgu_adp_status_set
#define sprdbat_read_temp_adc sprdchg_read_temp_adc

enum chg_full_condition {
	VOL_AND_CUR = 0,
	VOL_AND_STATUS,
	FROM_EXT_IC,
};

enum time_mode {
	ONE_TIME = 0,
	PERIOD_TIME,
};

struct sprdbat_table_data {
	int x;
	int y;
};

struct sprdbat_jeita_table_data {
	int w;
	int x;
	int y;
	int z;
};

enum sprdbat_jeita_status {
	STATUS_BELOW_T0 = 0,
	STATUS_T0_TO_T1,
	STATUS_T1_TO_T2,
	STATUS_T2_TO_T3,
	STATUS_T3_TO_T4,
	STATUS_ABOVE_T4
};

struct sprd_battery_platform_data {
	uint32_t chg_end_vol_l;
	uint32_t chg_end_vol_pure;
	uint32_t chg_bat_safety_vol;
	uint32_t fchg_vol;
	uint32_t rechg_vol;
	uint32_t adp_cdp_cur;
	uint32_t adp_dcp_cur;
	uint32_t adp_sdp_cur;
	uint32_t adp_unknown_cur;
	uint32_t adp_fchg_cur;
	uint32_t adp_cdp_cur_limit;
	uint32_t adp_dcp_cur_limit;
	uint32_t adp_sdp_cur_limit;
	uint32_t adp_unknown_cur_limit;
	uint32_t adp_fchg_cur_limit;
	uint32_t ovp_stop;
	uint32_t ovp_restart;
	uint32_t fchg_ovp_stop;
	uint32_t fchg_ovp_restart;
	uint32_t chg_timeout;
	uint32_t chgtimeout_show_full;
	uint32_t chg_rechg_timeout;
	uint32_t trickle_timeout;
	int chg_end_cur;
	int otp_high_stop;
	int otp_high_restart;
	int otp_low_stop;
	int otp_low_restart;
	uint32_t chg_polling_time;
	uint32_t chg_polling_time_fast;
	uint32_t cap_one_per_time;
	uint32_t cap_one_per_time_fast;
	int cap_valid_range_poweron;
	uint32_t chg_full_condition;
	struct iio_channel *channel_temp;
	struct iio_channel *channel_vbat;
	struct iio_channel *channel_isense;
	struct iio_channel *channel_vchg;

	struct thermal_zone_device *chgr_thmz;
	int chgr_otp_stop;
	int chgr_otp_restart;
	/* temp param */
	int temp_support;
	int temp_comp_res;
	struct sprdbat_table_data *temp_tab;
	int temp_tab_size;

	int jeita_tab_size;
	struct sprdbat_jeita_table_data  *jeita_tab;


	/* resource */
	uint32_t gpio_vchg_detect;
	uint32_t gpio_vbat_detect;
	uint32_t gpio_cv_state;
	uint32_t gpio_vchg_ovi;
	uint32_t gpio_chg_en;
	unsigned long chg_reg_base;
	unsigned long fgu_reg_base;

	/* fgu param */
	unsigned int only_vol_mode;
	uint32_t fgu_mode;
	uint32_t alm_soc;
	uint32_t alm_vol;
	/* reserved,if vbat voltage lower than it,
	 * should shutdown device
	 */
	uint32_t soft_vbat_uvlo;
	/* reserved,if battery voltage higher
	 * than it maybe shutdown device
	 */
	uint32_t soft_vbat_ovp;
	int rint;
	int cnom;
	int rsense_real;
	int rsense_spec;
	uint32_t relax_current;
	int fgu_cal_ajust;
	uint32_t cnom_track_support;
	int qmax_update_period;
	/* capacity temp table */
	struct sprdbat_table_data *cnom_temp_tab;

	int cnom_temp_tab_size;
	/* impedance temp table */
	struct sprdbat_table_data *rint_temp_tab;
	int rint_temp_tab_size;
	int ocv_type;
	/* OCV curve adjustment */
	struct sprdbat_table_data *ocv_tab;
	int ocv_tab_size;
	struct sprdbat_table_data *charge_vol_tab;
	int charge_vol_tab_size;
	struct sprdbat_table_data *discharge_vol_tab;
	int discharge_vol_tab_size;
	void *ext_data;
	int batt_full_design_capacity;
	int *batt_id_list;
	int batt_id_list_size;
};

struct sprd_chg_timer_operations {
	void (*timer_enable)(unsigned int, unsigned int);
	void (*timer_disable)(void);
	int (*timer_request)(irq_handler_t, void *);
};

struct sprd_fchg_operations {
	int (*fchg_init)(unsigned int);
	void (*fchg_deinit)(void);
};

struct sprdbat_info {
	uint32_t module_state;
	uint32_t bat_health;
	uint32_t chging_current;
	int bat_present;
	int bat_current;
	int bat_current_avg;
	uint32_t chg_current_type;
	uint32_t chg_current_type_limit;
	uint32_t input_cur_limit;
	uint32_t adp_type;
	uint32_t usb_online;
	uint32_t ac_online;
	uint32_t vchg_vol;
	uint32_t vchg_vol_max;
	uint32_t cccv_point;
	__s64 chg_start_time;
	uint32_t chg_stop_flags;
	uint32_t vbat_vol;
	uint32_t vbat_ocv;
	int cur_temp;
	int last_temp;
	int chgr_temp;
	uint32_t capacity;
	uint32_t soc;
	uint32_t chg_this_timeout;
	uint32_t avg_chg_vol;
	unsigned int chging_on;
	int usb_input_current;
	int charging_enabled;
	int user_set_input_cur_max;
};

struct sprdbat_auxadc_cal {
	uint16_t p0_vol;
	uint16_t p0_adc;
	uint16_t p1_vol;
	uint16_t p1_adc;
	uint16_t cal_type;
};

struct sprdbat_drivier_data {
	struct sprd_battery_platform_data *pdata;
	struct sprdbat_info bat_info;
	struct mutex lock;
	struct device *dev;
	struct power_supply *battery;
	struct power_supply *ac;
	struct power_supply *usb;
	struct notifier_block chg_nb;
	struct notifier_block chg_usb_nb;
	uint32_t gpio_charger_detect;
	uint32_t gpio_chg_cv_state;
	uint32_t gpio_vchg_ovi;
	uint32_t gpio_vbat_detect;
	uint32_t irq_charger_detect;
	uint32_t irq_chg_cv_state;
	uint32_t irq_vchg_ovi;
	uint32_t irq_vbat_detect;
	struct wake_lock charger_wake_lock;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work cv_irq_work;
	struct delayed_work battery_work;
	struct delayed_work battery_sleep_work;
	struct work_struct ovi_irq_work;
	struct work_struct vbat_detect_irq_work;
	struct work_struct plug_work;
	struct delayed_work *charge_work;
	int (*start_charge)(void);
	int (*stop_charge)(void);
#ifdef CONFIG_LEDS_TRIGGERS
	struct led_classdev charging_led;
#endif

	uint32_t chg_cur_adjust_min;
	uint32_t chg_cur_adjust_max;
	uint32_t last_temp_status;
	uint32_t cur_temp_status;
	uint32_t temp_up_trigger_cnt;
	uint32_t temp_down_trigger_cnt;
	uint32_t chg_full_trigger_cnt;
	uint32_t sprdbat_average_cnt;
	u32 sprdbat_vbat_ovp_cnt;
	uint32_t sprdbat_trickle_chg;
	uint32_t poweron_capacity;
	uint32_t fchg_det;
	__s64 sprdbat_update_capacity_time;
	__s64 sprdbat_last_query_time;
	struct delayed_work sprdbat_charge_work;
	struct notifier_block sprdbat_notifier;
	struct usb_charger *usb_charger;
};

struct sprd_ext_ic_operations {
	void (*ic_init)(struct sprdbat_drivier_data *);
	void (*charge_start_ext)(void);
	void (*charge_stop_ext)(unsigned int);
	void (*set_charge_cur)(unsigned int);
	void (*set_input_cur_limit)(unsigned int);
	int (*get_charging_status)(void);
	int (*get_charging_fault)(void);
	void (*timer_callback_ext)(void);
	unsigned int (*get_charge_cur_ext)(void);
	void (*set_termina_cur_ext)(unsigned int);
	void (*set_termina_vol_ext)(unsigned int);
	void (*otg_charge_ext)(int);
	int (*get_chgcur_step)(int, int);
	void (*ext_register_notifier)(struct notifier_block *);
	void (*ext_unregster_notifier)(struct notifier_block *);
	int (*support_fchg)(void);
	void (*set_ship_mode)(int);
	int (*get_ship_mode)(void);
	int (*get_charge_status)(void);
};

#if defined(CONFIG_USB_SPRD_DWC) || defined(CONFIG_USB_DWC3_SPRD)
extern int register_usb_notifier(struct notifier_block *nb);
extern int unregister_usb_notifier(struct notifier_block *nb);
#endif

#ifdef CONFIG_BATTERY_SPRD
int sprdbat_interpolate
	(int x, int n, struct sprdbat_table_data *tab);
void sprdbat_register_timer_ops
	(struct sprd_chg_timer_operations *ops);
void sprdbat_unregister_timer_ops(void);
void sprdbat_register_fchg_ops(struct sprd_fchg_operations *ops);
void sprdbat_unregister_fchg_ops(void);
bool sprdbat_is_support_batdet(void);
bool sprdbat_is_bat_present(void);
#else
static inline int sprdbat_interpolate
	(int x, int n, struct sprdbat_table_data *tab)
{
	return 0;
}
static inline void sprdbat_register_timer_ops
	(struct sprd_chg_timer_operations *ops)
{
}
static inline void sprdbat_unregister_timer_ops(void)
{
}
static inline void sprdbat_register_fchg_ops
	(struct sprd_fchg_operations *ops)
{
}
static inline void sprdbat_unregister_fchg_ops(void)
{
}
static inline int sprdfgu_read_batcurrent(void)
{
	return 0;
}
static inline bool sprdbat_is_support_batdet(void)
{
	return 0;
}
static inline bool sprdbat_is_bat_present(void)
{
	return 1;
}
#endif
void sprdbat_register_ext_ops(const struct sprd_ext_ic_operations *ops);
void sprdbat_unregister_ext_ops(void);
void sprdbat_register_timer_ops(struct sprd_chg_timer_operations *ops);
void sprdbat_unregister_timer_ops(void);
void sprdbat_register_fchg_ops(struct sprd_fchg_operations *ops);
void sprdbat_unregister_fchg_ops(void);

struct sprd_ext_ic_operations *sprd_get_54015_ops(void);
struct sprd_ext_ic_operations *sprd_get_2701_ops(void);
struct sprd_ext_ic_operations *sprd_get_bq25896_ops(void);
struct sprd_ext_ic_operations *sprd_get_2700_ops(void);
struct sprd_ext_ic_operations *sprd_get_2705_ops(void);
struct sprd_ext_ic_operations *sprd_get_2703_ops(void);
struct sprd_ext_ic_operations *sprd_get_bq2560x_ops(void);
#endif /* _CHG_DRVAPI_H_ */
