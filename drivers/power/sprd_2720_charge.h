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

#ifndef _SPRD_2720_CHARGE_H_
#define _SPRD_2720_CHARGE_H_

#define SPRDBAT_CCCV_MIN    0x00
#define SPRDBAT_CCCV_MAX   0x3F
#define SPRDBAT_CCCV_DEF   0x20
#define SPRDBAT_CCCV_MSK   0x3F


#define ONE_CCCV_STEP_VOL   75

#define SPRDCHG_OVP_6000MV		6000
#define SPRDCHG_OVP_6500MV		6500
#define SPRDCHG_OVP_7000MV		7000
#define SPRDCHG_OVP_9700MV		9700

#define SPRDCHG_CUR_MIN		300
#define SPRDCHG_CUR_800MA	800
#define SPRDCHG_CUR_MAX		1300
#define SPRDCHG_CUR_STEP50	50
#define SPRDCHG_CUR_STEP100	100
#define SPRDCHG_CUR_800MA_REG	0x0A


#define SPRDCHG_VOLBIG_MIN	4200
#define SPRDCHG_VOLBIG_MAX	4500
#define SPRDCHG_VOLBIG_STEP	100

#define SPRDCHG_DPM_4300MV	4300


#define SPRDBAT_TEMP_TRIGGER_TIMES 2

struct sprd2720_platform_data {
	uint32_t gpio_chg_cv_state;
	uint32_t gpio_vchg_ovi;
	uint32_t irq_vchg_ovi;
	uint32_t irq_chg_cv_state;
	uint32_t cccv_cal;
};

struct sprd2720_device {
	struct device *dev;
	struct power_supply charger;
	struct delayed_work cv_irq_work;
	struct work_struct ovi_irq_work;
	struct sprd_battery_platform_data *pdata;
};

#endif /* _CHG_DRVAPI_H_ */
