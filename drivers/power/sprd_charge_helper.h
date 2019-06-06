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

#ifndef _SPRD_CHG_HELP_CHARGE_H_
#define _SPRD_CHG_HELP_CHARGE_H_

#include "sprd_battery.h"

#define SPRDBAT_TEMP_TRIGGER_TIMES 2

#define SPRDBAT_FGUADC_CAL_NO         0
#define SPRDBAT_FGUADC_CAL_NV         1
#define SPRDBAT_FGUADC_CAL_CHIP      2

int sprdfgu_init(struct sprd_battery_platform_data *pdata);
int sprdfgu_reset(void);
int sprdfgu_force_set_soc(unsigned int cap);
void sprdfgu_record_cap(u32 cap);
uint32_t sprdfgu_read_capacity(void);
unsigned int sprdfgu_only_vol_read_capacity(int val);
uint32_t sprdfgu_poweron_capacity(void);
int sprdfgu_read_soc(void);
int sprdfgu_read_batcurrent(void);
uint32_t sprdfgu_read_vbat_vol(void);
uint32_t sprdfgu_read_vbat_ocv(void);
void sprdfgu_adp_status_set(int plugin);
void sprdfgu_pm_op(int is_suspend);

int sprdchg_init(struct sprd_battery_platform_data *pdata);
int sprdchg_read_temp(void);
int sprdchg_read_temp_adc(void);
int sprdchg_read_ntc_vol(struct iio_channel *channel_temp);
uint32_t sprdchg_read_vchg_vol(void);
uint32_t sprdchg_read_vbat_vol(void);
void sprdchg_led_brightness_set(struct led_classdev *led_cdev,
					enum led_brightness brightness);

int sprdchg_bc1p2_disable(int disable);
extern unsigned int glb_adi_read(u32 reg);
extern int glb_adi_write(u32 reg, u32 or_val, u32 clear_msk);
enum usb_charger_type sprdchg_charger_is_adapter(void);
extern int sprd_charge_pd_control(bool enable);

#ifdef CONFIG_OTP_SPRD_PMIC_EFUSE
extern u32 sprd_pmic_efuse_block_read(int blk_index);
extern u32 sprd_pmic_efuse_bits_read(int bit_index, int length);
#endif

#endif

