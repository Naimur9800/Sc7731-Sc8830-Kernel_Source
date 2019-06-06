/*
 * Copyright (c) 2017 Dialog Semiconductor.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SC2705_CORE_H
#define __SC2705_CORE_H

#include <linux/device.h>
#include <linux/extcon.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>

/* IRQs */
enum sc2705_irq_defs {
	SC2705_IRQ_ADP_DET = 0,
	SC2705_IRQ_VIN2BAT,
	SC2705_IRQ_VIN_UV,
	SC2705_IRQ_VIN_DROP,
	SC2705_IRQ_VIN_OV,
	SC2705_IRQ_VBAT_UV,
	SC2705_IRQ_VBAT_OV,
	SC2705_IRQ_IN_PWR_BLOCK,
	SC2705_IRQ_TJUNC_WARN,
	SC2705_IRQ_TJUNC_CRIT,
	SC2705_IRQ_TJUNC_POR,
	SC2705_IRQ_TBAT_T1,
	SC2705_IRQ_TBAT_T2,
	SC2705_IRQ_TBAT_T3,
	SC2705_IRQ_TBAT_T4,
	SC2705_IRQ_BATDET,
	SC2705_IRQ_VIN_REV_SHORT,
	SC2705_IRQ_VIN_REV_OV,
	SC2705_IRQ_DCDC_REV_BOOST_FAULT,
	SC2705_IRQ_IIN_REV_LIM,
	SC2705_IRQ_IIN_REV_LIM_MAX,
	SC2705_IRQ_LOWBAT,
	SC2705_IRQ_ADC_DONE,
	SC2705_IRQ_IIN_LIM,
	SC2705_IRQ_CHG_STAT,
	SC2705_IRQ_VSYS_POR,
	SC2705_IRQ_VSYS_OV,
	SC2705_IRQ_WD,
	SC2705_IRQ_TIMEOUT_PRE,
	SC2705_IRQ_TIMEOUT_CCCV,
	SC2705_IRQ_GPI0,
	SC2705_IRQ_GPI1,
	SC2705_IRQ_GPI2,
	SC2705_IRQ_GPI3,
	SC2705_IRQ_VSYS_UV,
	SC2705_IRQ_BOOST_STARTUP_OV,
	SC2705_IRQ_FLASH_LDO_SHORT_CKT,
	SC2705_IRQ_TORCH_CHG_OV,
	SC2705_NUM_IRQS,
};

#define SC2705_NUM_IRQ_REGS	6

/* Platform Data */
enum sc2705_ibat_pre {
	SC2705_IBAT_PRE_50MA = 0,
	SC2705_IBAT_PRE_100MA,
	SC2705_IBAT_PRE_200MA,
	SC2705_IBAT_PRE_400MA,
	SC2705_IBAT_PRE_DEFAULT = SC2705_IBAT_PRE_200MA,
};

enum sc2705_t_eoc {
	SC2705_T_EOC_0MIN = 0,
	SC2705_T_EOC_1MIN,
	SC2705_T_EOC_5MIN,
	SC2705_T_EOC_10MIN,
	SC2705_T_EOC_20MIN,
	SC2705_T_EOC_30MIN,
	SC2705_T_EOC_45MIN,
	SC2705_T_EOC_60MIN,
	SC2705_T_EOC_DEFAULT = SC2705_T_EOC_0MIN,
};

enum sc2705_vbat_rechg {
	SC2705_VBAT_RECHG_100MA = 0,
	SC2705_VBAT_RECHG_160MA,
	SC2705_VBAT_RECHG_200MA,
	SC2705_VBAT_RECHG_240MA,
	SC2705_VBAT_RECHG_DEFAULT = SC2705_VBAT_RECHG_100MA,
};

enum sc2705_timeout_cccv {
	SC2705_TIMEOUT_CCCV_2HOURS = 0,
	SC2705_TIMEOUT_CCCV_4HOURS,
	SC2705_TIMEOUT_CCCV_6HOURS,
	SC2705_TIMEOUT_CCCV_8HOURS,
	SC2705_TIMEOUT_CCCV_10HOURS,
	SC2705_TIMEOUT_CCCV_12HOURS,
	SC2705_TIMEOUT_CCCV_14HOURS,
	SC2705_TIMEOUT_CCCV_18HOURS,
	SC2705_TIMEOUT_CCCV_DEFAULT = SC2705_TIMEOUT_CCCV_2HOURS,
};

enum sc2705_iin_rev_lim {
	SC2705_IIN_REV_LIM_500MA = 0,
	SC2705_IIN_REV_LIM_900MA,
	SC2705_IIN_REV_LIM_1200MA,
	SC2705_IIN_REV_LIM_1500MA,
	SC2705_IIN_REV_LIM_2000MA,
	SC2705_IIN_REV_LIM_DEFAULT = SC2705_IIN_REV_LIM_2000MA,
};

enum sc2705_bat_det_src {
	SC2705_BAT_DET_SRC_OFF = 0,
	SC2705_BAT_DET_SRC_VBAT,
	SC2705_BAT_DET_SRC_BATID,
	SC2705_BAT_DET_SRC_TBAT,
	SC2705_BAT_DET_SRC_DEFAULT = SC2705_BAT_DET_SRC_TBAT,
};

#define SC2705_VBAT_CHG_DEFAULT			0x14
#define SC2705_VBAT_CHG_MAX			0x32
#define SC2705_IBAT_CHG_DEFAULT			0x00
#define SC2705_IBAT_CHG_MAX			0x3C
#define SC2705_IBAT_TERM_DEFAULT		0x00
#define SC2705_IBAT_TERM_MAX			0x7
#define SC2705_DCDC_PEAK_ILIM_DEFAULT		0x2
#define SC2705_DCDC_PEAK_ILIM_MAX		0x3
#define SC2705_VBAT_OV_DEFAULT			0xF
#define SC2705_VBAT_OV_MAX			0xF
#define SC2705_VBAT_UV_DEFAULT			0x0
#define SC2705_VBAT_UV_MAX			0xF
#define SC2705_VSYS_MIN_DEFAULT			0x8
#define SC2705_VSYS_MIN_MAX			0xF
#define SC2705_TIMEOUT_PRE_DEFAULT		0x0
#define SC2705_TIMEOUT_PRE_MAX			0x3
#define SC2705_DCDC_REV_PEAK_ILIM_MAX		0x3
#define SC2705_TBAT_T3_DEFAULT			0x0
#define SC2705_TBAT_T3_MAX			0x2
#define SC2705_TBAT_T4_DEFAULT			0x0
#define SC2705_TBAT_T4_MAX			0x2
#define SC2705_VBAT_COLD_DEFAULT		0x0
#define SC2705_VBAT_COLD_MAX			0x2
#define SC2705_VBAT_WARM_DEFAULT		0x0
#define SC2705_VBAT_WARM_MAX			0x2

struct sc2705_charger_pdata {
	enum sc2705_ibat_pre ibat_pre;
	u8 vbat_chg;
	u8 ibat_chg;
	u8 ibat_term;
	enum sc2705_t_eoc t_eoc;
	enum sc2705_vbat_rechg vbat_rechg;
	u8 dcdc_peak_ilim;
	u8 vbat_ov;
	u8 vbat_uv;
	u8 vsys_min;
	u8 timeout_pre;
	enum sc2705_timeout_cccv timeout_cccv;
	u8 timer_load;
	enum sc2705_iin_rev_lim iin_rev_lim;
	enum sc2705_bat_det_src bat_det_src;

	u8 tbat_t3;
	u8 tbat_t4;
	u8 vbat_cold;
	bool ibat_cold_enable;
	u8 vbat_warm;
	bool ibat_warm_enable;

	struct extcon_dev *ext_id;
};

struct sc2705_pdata {
	int irq_base;

	struct sc2705_charger_pdata *charger_pdata;
};

/* Private Data */
struct sc2705 {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_irq_chip_data *regmap_irq_data;
	struct dentry *debugfs_root;
	int irq;
	int irq_base;
	unsigned int addr;
	unsigned int read_cnt;
};

struct sc2705_irq_type {
	char *name;
	irq_handler_t handler;
};

#endif /* __SC2705_CORE_H */
