/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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
 *
 */
#ifndef __SPRD_PM_DEBUG_ARM_H__
#define __SPRD_PM_DEBUG_ARM_H__

#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/mfd/syscon/sprd/sc9833/sc9833_glb.h>
#include <linux/io.h>
#include "pm_debug.h"

/*******************INT********************/
#define   SPRD_INTC_NUM					10
#define   SPRD_HARD_INTERRUPT_NUM			160
#define   SPRD_HARD_INT_NUM_EACH_INTC			32
#define   SPRD_IRQ_NUM					1024


#define	INTCV_IRQ_MSKSTS	(0x0000)
#define	INTCV_IRQ_RAW		(0x0004)
#define	INTCV_IRQ_EN		(0x0008)
#define	INTCV_IRQ_DIS		(0x000c)
#define	INTCV_IRQ_SOFT		(0x0010)
#define	INTCV_IRQ_TEST_SRC		(0x0014)
#define	INTCV_IRQ_TEST_SEL		(0x0018)

#define	INTCV_FIQ_STS	(0x0020)
#define	INTCV_FIQ_RAW		(0x0024)
#define	INTCV_FIQ_EN		(0x0028)
#define	INTCV_FIQ_DIS		(0x002c)
#define	INTCV_FIQ_SOFT		(0x0030)
#define	INTCV_FIQ_TEST_SRC		(0x0034)
#define	INTCV_FIQ_TEST_SEL		(0x0038)

#define INT_IRQ_MASK	(1<<3)
#define ANA_REG_INTC_BASE		(0x0140)
#define ANA_REG_INT_MASK_STATUS         (ANA_REG_INTC_BASE + 0x0000)
#define ANA_REG_INT_RAW_STATUS          (ANA_REG_INTC_BASE + 0x0004)
#define ANA_REG_INT_EN                  (ANA_REG_INTC_BASE + 0x0008)
#define ANA_REG_INT_MASK_STATUS_SYNC    (ANA_REG_INTC_BASE + 0x000c)

#define ANA_REG_EIC_BASE		(0x0300)
#define ANA_REG_EIC_INT_EVENT           (ANA_REG_EIC_BASE+0x0014)
#define ANA_REG_EIC_INT_EN		(ANA_REG_EIC_BASE+0x0018)
#define ANA_REG_EIC_RAW_INT_STATUS      (ANA_REG_EIC_BASE+0x001c)
#define ANA_REG_EIC_MASK_INT_STATUS     (ANA_REG_EIC_BASE+0x0020)

#define GPIO_GROUP_NUM			16
#define REG_GPIO_MIS			(0x0020)

struct ap_ahb_reg_bak {
	u32 ahb_eb;
	u32 misc_ckg_en;
	u32 ca7_ckg_div_cfg;
	u32 ca7_ckg_sel_cfg;
	u32 force_sleep_cfg;
	u32 auto_sleep_cfg;
	u32 otg_phy_tune;
	u32 otg_phy_test;
	u32 otg_phy_ctrl;
	u32 otg_ctrl0;
	u32 otg_ctrl1;
	u32 m0_lpc;
	u32 m1_lpc;
	u32 m2_lpc;
	u32 m3_lpc;
	u32 m4_lpc;
	u32 m5_lpc;
	u32 m6_lpc;
	u32 m7_lpc;
	u32 m8_lpc;
	u32 m9_lpc;
	u32 main_lpc;
	u32 s0_lpc;
	u32 s1_lpc;
	u32 s2_lpc;
	u32 s3_lpc;
	u32 s4_lpc;
	};

struct ap_apb_reg_bak {
	u32 apb_eb;
	u32 apb_misc_ctrl;
};

struct auto_pd_en {
	u32 magic_header;
	u32 bits;
	u32 magic_end;
	char pd_config_menu[6][16];
};

#define AP_AHB_BITS_CLEAR_CHECK(__flag, __bits, __string) do {\
	uint32_t mid ;\
	regmap_read(deepsleep_syscon_ap_ahb, __flag, &mid); \
	if (mid & __bits) \
		pr_info(__string " not 0\n"); \
	} while (0)
#define AP_APB_BITS_CLEAR_CHECK(__flag, __bits, __string) do {\
	uint32_t mid ;\
	regmap_read(deepsleep_syscon_ap_apb, __flag, &mid); \
	if (mid & __bits) \
		pr_info(__string " not 0\n"); \
	} while (0)
#define AON_APB_BITS_CLEAR_CHECK(__flag, __bits, __string) do {\
	uint32_t mid ;\
	regmap_read(deepsleep_syscon_aon_apb, __flag, &mid); \
	if (mid & __bits) \
		pr_info(__string " not 0\n"); \
	} while (0)

extern struct regmap *deepsleep_syscon_ap_ahb;
extern struct regmap *deepsleep_syscon_pmu_apb;
extern struct regmap *deepsleep_syscon_aon_apb;
extern struct regmap *deepsleep_syscon_ana_apb;
extern struct regmap *deepsleep_syscon_ap_apb;
extern struct regmap *deepsleep_syscon_ap_intc0;
extern struct regmap *deepsleep_syscon_ap_intc1;
extern struct regmap *deepsleep_syscon_ap_intc2;
extern struct regmap *deepsleep_syscon_ap_intc3;
extern struct regmap *deepsleep_syscon_aon_intc;
extern int is_intc_base_vaddr_ok;
extern int is_syscon_base_vaddr_ok;

void deepsleep_condition_check(void);
void bak_last_reg(void);
void print_last_reg(void);
void show_pin_reg(void);
void show_deep_reg_status(void);
void print_int_status(void);
void pm_debug_init(void);
void hard_irq_set(void);
void set_sleep_mode(int sm);
void time_add(unsigned int time, int ret);
void print_hard_irq_inloop(int ret);
void clr_sleep_mode(void);
void time_statisic_begin(void);
int sprd_cpu_deep_sleep(unsigned int cpu);
unsigned int sprd_irq_pending(void);
void irq_wakeup_set(void);
void time_statisic_end(void);
void print_statisic(void);
void __init sc_pm_init(void);

#endif /* __SPRD_PM_DEBUG_ARM32_H__ */
