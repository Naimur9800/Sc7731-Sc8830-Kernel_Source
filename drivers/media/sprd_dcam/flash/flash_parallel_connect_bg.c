/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#ifndef CONFIG_64BIT
#include <soc/sprd/hardware.h>
#include <soc/sprd/board.h>
#endif
#include <soc/sprd/adi.h>
#include "../common/parse_hwinfo.h"

#define SPRD_PWM0_CTRL_OFST                       0xEC
#define SPRD_PWM0_PATTERN_HIGHT_OFST              0x10
#define SPRD_PWM0_PATTERT_LOW_OFST                0xC
#define SPRD_PWM0_TONE_OFST                       0x8
#define SPRD_PWM0_RATION_OFST                     0x4
#define SPRD_WHTLED_CTRL_OFST                     0xF0


int sprd_flash_on(void)
{
	printk("parallel sprd_flash_on \n");
	/*ENABLE THE PWM0 CONTROLLTER: RTC_PWM0_EN=1 & PWM0_EN=1*/
	sci_adi_set(ANA_CTL_GLB_BASE + SPRD_PWM0_CTRL_OFST, 0xC000);

	/*SET PWM0 PATTERN HIGH*/
	sci_adi_set(ANA_PWM_BASE + SPRD_PWM0_PATTERN_HIGHT_OFST, 0xFFFF);

	/*SET PWM0 PATTERN LOW*/
	sci_adi_set(ANA_PWM_BASE + SPRD_PWM0_PATTERT_LOW_OFST, 0xFFFF);

	/*TONE DIV USE DEFAULT VALUE*/
	sci_adi_clr(ANA_PWM_BASE + SPRD_PWM0_TONE_OFST, 0xFFFF);

	/*SET PWM0 DUTY RATIO = 100%: MOD=FF & DUTY=FF*/
	sci_adi_set(ANA_PWM_BASE + SPRD_PWM0_RATION_OFST, 0xFFFF);

	/*ENABLE PWM0 OUTPUT: PWM0_EN=1*/
	sci_adi_set(ANA_PWM_BASE, 0x100);

	/*SET LOW LIGHT & ENABLE WHTLED*/
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x7F);
	return 0;
}

int sprd_flash_high_light(void)
{
	printk("parallel sprd_flash_high_light \n");
	/*SET HIGH LIGHT*/
	sci_adi_set(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x7E);
	/*ENABLE WHTLED*/
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x1);
	return 0;
}

int sprd_flash_close(void)
{
	printk("parallel sprd_flash_close \n");

	/*DISABLE WHTLED*/
	sci_adi_set(ANA_CTL_GLB_BASE + SPRD_WHTLED_CTRL_OFST, 0x1);

	/*DISABLE THE PWM0 CONTROLLTER: RTC_PWM0_EN=0 & PWM0_EN=0*/
	sci_adi_clr(ANA_CTL_GLB_BASE + SPRD_PWM0_CTRL_OFST, 0xC000);

	/*ENABLE PWM0 OUTPUT: PWM0_EN=0*/
	sci_adi_clr(ANA_PWM_BASE, 0x100);
	return 0;
}
