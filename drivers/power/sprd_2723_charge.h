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

#ifndef _SPRD_2723_CHARGE_H_
#define _SPRD_2723_CHARGE_H_

#define SPRDBAT_CCCV_MIN    0x00
#define SPRDBAT_CCCV_MAX   0x3F
#define ONE_CCCV_STEP_VOL   75

#define SPRDBAT_CHG_OVP_LEVEL_MIN       5600
#define SPRDBAT_CHG_OVP_LEVEL_MAX   9000

#define SPRDBAT_CHG_CUR_LEVEL_MIN       300
#define SPRDBAT_CHG_CUR_LEVEL_MAX	1300

#define SPRDBAT_TEMP_TRIGGER_TIMES 2

struct sprd2723_platform_data{
	uint32_t gpio_chg_cv_state;
	uint32_t gpio_vchg_ovi;
	uint32_t irq_vchg_ovi;
	uint32_t irq_chg_cv_state;
};

struct sprd2723_device {
	struct device *dev;
	struct power_supply charger;
	struct delayed_work cv_irq_work;
	struct work_struct ovi_irq_work;
	struct sprd_battery_platform_data *pdata;
};

#endif /* _CHG_DRVAPI_H_ */
