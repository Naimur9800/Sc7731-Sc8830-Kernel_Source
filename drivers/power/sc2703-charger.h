/*
 * sc2703-charger.h - CHARGER H for SC2703
 * Copyright (c) 2018 Dialog Semiconductor.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __SC2703_CHARGER_H
#define __SC2703_CHARGER_H

enum sc2703_ibat_pre {
	SC2703_IBAT_PRE_50MA = 0,
	SC2703_IBAT_PRE_100MA,
	SC2703_IBAT_PRE_200MA,
	SC2703_IBAT_PRE_400MA,
	SC2703_IBAT_PRE_DEFAULT = SC2703_IBAT_PRE_200MA,
};

enum sc2703_t_eoc {
	SC2703_T_EOC_0MIN = 0,
	SC2703_T_EOC_1MIN,
	SC2703_T_EOC_5MIN,
	SC2703_T_EOC_10MIN,
	SC2703_T_EOC_20MIN,
	SC2703_T_EOC_30MIN,
	SC2703_T_EOC_45MIN,
	SC2703_T_EOC_60MIN,
	SC2703_T_EOC_DEFAULT = SC2703_T_EOC_0MIN,
};

enum sc2703_vbat_rechg {
	SC2703_VBAT_RECHG_100MA = 0,
	SC2703_VBAT_RECHG_160MA,
	SC2703_VBAT_RECHG_200MA,
	SC2703_VBAT_RECHG_240MA,
	SC2703_VBAT_RECHG_DEFAULT = SC2703_VBAT_RECHG_100MA,
};

enum sc2703_timeout_cccv {
	SC2703_TIMEOUT_CCCV_2HOURS = 0,
	SC2703_TIMEOUT_CCCV_4HOURS,
	SC2703_TIMEOUT_CCCV_6HOURS,
	SC2703_TIMEOUT_CCCV_8HOURS,
	SC2703_TIMEOUT_CCCV_10HOURS,
	SC2703_TIMEOUT_CCCV_12HOURS,
	SC2703_TIMEOUT_CCCV_14HOURS,
	SC2703_TIMEOUT_CCCV_18HOURS,
	SC2703_TIMEOUT_CCCV_DEFAULT = SC2703_TIMEOUT_CCCV_2HOURS,
};

enum sc2703_iin_rev_lim {
	SC2703_IIN_REV_LIM_500MA = 0,
	SC2703_IIN_REV_LIM_900MA,
	SC2703_IIN_REV_LIM_1200MA,
	SC2703_IIN_REV_LIM_1500MA,
	SC2703_IIN_REV_LIM_2000MA,
	SC2703_IIN_REV_LIM_DEFAULT = SC2703_IIN_REV_LIM_2000MA,
};

enum sc2703_bat_det_src {
	SC2703_BAT_DET_SRC_OFF = 0,
	SC2703_BAT_DET_SRC_VBAT,
	SC2703_BAT_DET_SRC_BATID,
	SC2703_BAT_DET_SRC_TBAT,
	SC2703_BAT_DET_SRC_DEFAULT = SC2703_BAT_DET_SRC_TBAT,
};

struct sc2703_charger_pdata {
	enum sc2703_ibat_pre ibat_pre;
	u8 vbat_chg;
	u8 ibat_chg;
	u8 ibat_term;
	enum sc2703_t_eoc t_eoc;
	enum sc2703_vbat_rechg vbat_rechg;
	u8 dcdc_peak_ilim;
	u8 dcdc_rev_peak_ilim;
	u8 vbat_ov;
	u8 vbat_uv;
	u8 vsys_min;
	u8 timeout_pre;
	enum sc2703_timeout_cccv timeout_cccv;
	u8 timer_load;
	enum sc2703_iin_rev_lim iin_rev_lim;
	enum sc2703_bat_det_src bat_det_src;
	u8 tbat_t3;
	u8 tbat_t4;
	u8 vbat_cold;
	bool ibat_cold_enable;
	u8 vbat_warm;
	bool ibat_warm_enable;
	bool onkey_det_enable;
	bool spsp_en;
	bool spsp_tune_en;
	u8 spsp_tune;
	struct extcon_dev *ext_id;
};

struct sc2703_pdata {
	int irq_base;
	struct sc2703_charger_pdata *charger_pdata;
};


#define SC2703_VBAT_CHG_DEFAULT			0x14
#define SC2703_VBAT_CHG_MAX			0x32
#define SC2703_IBAT_CHG_DEFAULT			0x00
#define SC2703_IBAT_CHG_MAX			0x3c
#define SC2703_IBAT_TERM_DEFAULT		0x00
#define SC2703_IBAT_TERM_MAX			0x7
#define SC2703_DCDC_PEAK_ILIM_DEFAULT		0x2
#define SC2703_DCDC_PEAK_ILIM_MAX		0x3
#define SC2703_DCDC_REV_PEAK_ILIM_DEFAULT		0x0
#define SC2703_DCDC_REV_PEAK_ILIM_MAX		0x3
#define SC2703_VBAT_OV_DEFAULT			0xf
#define SC2703_VBAT_OV_MAX			0xf
#define SC2703_VBAT_UV_DEFAULT			0x0
#define SC2703_VBAT_UV_MAX			0xf
#define SC2703_VSYS_MIN_DEFAULT			0x8
#define SC2703_VSYS_MIN_MAX			0xf
#define SC2703_TIMEOUT_PRE_DEFAULT		0x0
#define SC2703_TIMEOUT_PRE_MAX			0x3
#define SC2703_TBAT_T3_DEFAULT			0x0
#define SC2703_TBAT_T3_MAX			0x2
#define SC2703_TBAT_T4_DEFAULT			0x0
#define SC2703_TBAT_T4_MAX			0x2
#define SC2703_VBAT_COLD_DEFAULT		0x0
#define SC2703_VBAT_COLD_MAX			0x2
#define SC2703_VBAT_WARM_DEFAULT		0x0
#define SC2703_VBAT_WARM_MAX			0x2
#define SC2703_SPSP_TUNE_DEFAULT	0x0
#define SC2703_SPSP_TUNE_MAX	0x16
#define SC2703_S_CHG_STAT_FULL		0x5
#define SC2703_S_CHG_STAT_FAULT_L1	0x6
#define SC2703_S_CHG_STAT_FAULT_L2	0x7
#define SC2703_E_VBAT_OV_SHIFT         6
#define SC2703_E_VBAT_OV_MASK          BIT(6)
#define SC2703_SUPPORT_FCHG            0x80

void sc2703_charger_ic_init(void);
void sc2703_charger_start(void);
void sc2703_charger_stop(unsigned int flag);
unsigned int sc2703_charger_cc_current_get(void);
void sc2703_charger_cc_current_set(unsigned int val);
void sc2703_charger_input_current_set(unsigned int val);
void sc2703_charger_otg_enable(int enable);
void sc2703_termina_vol_set(unsigned int val);
int sc2703_charger_status_get(void);
void sc2703_charger_vindpm_set(unsigned int val);
void sc2703_charger_reset_timer(void);
int sc2703_get_fault_val(u32 *val);
bool sc2703_is_support_fchg(void);
#endif
