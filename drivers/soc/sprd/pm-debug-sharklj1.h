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
#ifndef __SPRD_PM_DEBUG_ARM64_H__
#define __SPRD_PM_DEBUG_ARM64_H__

#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/mfd/syscon/sprd/sharklj1/glb.h>
#include <linux/io.h>
#include "pm_debug.h"

/*******************INT********************/
#define   SPRD_INTC_NUM					12
#define   SPRD_HARD_INTERRUPT_NUM			192
#define   SPRD_HARD_INT_NUM_EACH_INTC			32
#define   SPRD_IRQ_NUM					1024


#define	INTCV0_IRQ_MSKSTS	(0x0000)
#define	INTCV0_IRQ_RAW		(0x0004)
#define	INTCV0_IRQ_EN		(0x0008)
#define	INTCV0_IRQ_DIS		(0x000C)
#define	INTCV0_FIQ_STS		(0x0020)
#define	INTCV1_IRQ_MSKSTS	(0x0000)
#define	INTCV1_IRQ_RAW		(0x0004)
#define	INTCV1_IRQ_EN		(0x0008)
#define	INTCV1_IRQ_DIS		(0x000C)
#define	INTCV1_FIQ_STS		(0x0020)
#define	INTCV2_IRQ_MSKSTS	(0x0000)
#define	INTCV2_IRQ_RAW		(0x0004)
#define	INTCV2_IRQ_EN		(0x0008)
#define	INTCV2_IRQ_DIS		(0x000C)
#define	INTCV2_FIQ_STS		(0x0020)
#define	INTCV3_IRQ_MSKSTS	(0x0000)
#define	INTCV3_IRQ_RAW		(0x0004)
#define	INTCV3_IRQ_EN		(0x0008)
#define	INTCV3_IRQ_DIS		(0x000C)
#define	INTCV3_FIQ_STS		(0x0020)
#define	INTCV4_IRQ_MSKSTS	(0x0000)
#define	INTCV4_IRQ_RAW		(0x0004)
#define	INTCV4_IRQ_EN		(0x0008)
#define	INTCV4_IRQ_DIS		(0x000C)
#define	INTCV4_FIQ_STS		(0x0020)
#define	INTCV5_IRQ_MSKSTS	(0x0000)
#define	INTCV5_IRQ_RAW		(0x0004)
#define	INTCV5_IRQ_EN		(0x0008)
#define	INTCV5_IRQ_DIS		(0x000C)
#define	INTCV5_FIQ_STS		(0x0020)

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
	u32 ca7_ckg_cfg;
	u32 ca7_div_cfg;
	u32 misc_ckg_en;
	u32 misc_cfg;
	u32 usb_phy_tune;
};
struct ap_clk_reg_bak {
	u32 ap_apb_cfg;
	u32 usb3_utmi_cfg;
	u32 usb3_pipe_cfg;
	u32 usb3_ref_cfg;
	u32 uart0_cfg;
	u32 uart1_cfg;
	u32 uart2_cfg;
	u32 uart3_cfg;
	u32 uart4_cfg;
	u32 i2c0_cfg;
	u32 i2c1_cfg;
	u32 i2c2_cfg;
	u32 i2c3_cfg;
	u32 i2c4_cfg;
	u32 i2c5_cfg;
	u32 spi0_cfg;
	u32 spi1_cfg;
	u32 spi2_cfg;
	u32 spi3_cfg;
	u32 iis0_cfg;
	u32 iis1_cfg;
	u32 iis2_cfg;
	u32 iis3_cfg;
	u32 bist_192m_cfg;
};
struct ap_apb_reg_bak {
	u32 apb_eb;
	u32 usb_phy_tune;
	u32 usb_phy_ctrl;
	u32 apb_misc_ctrl;
};
struct pub_reg_bak {
	u32 ddr_qos_cfg1;
	u32 ddr_qos_cfg2;
	u32 ddr_qos_cfg3;
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
		pr_info(__string " not 0" "\n"); \
	} while (0)
#define AP_APB_BITS_CLEAR_CHECK(__flag, __bits, __string) do {\
	uint32_t mid ;\
	regmap_read(deepsleep_syscon_ap_apb, __flag, &mid); \
	if (mid & __bits) \
		pr_info(__string " not 0" "\n"); \
	} while (0)
#define AON_APB_BITS_CLEAR_CHECK(__flag, __bits, __string) do {\
	uint32_t mid ;\
	regmap_read(deepsleep_syscon_aon_apb, __flag, &mid); \
	if (mid & __bits) \
		pr_info(__string " not 0" "\n"); \
	} while (0)

#define DEVICE_AHB              (0x1UL << 20)
#define DEVICE_APB              (0x1UL << 21)
#define DEVICE_VIR              (0x1UL << 22)
#define DEVICE_AWAKE            (0x1UL << 23)
#define LIGHT_SLEEP             (0x1UL << 24)
#define DEVICE_TEYP_MASK        (DEVICE_AHB | DEVICE_APB |\
				DEVICE_VIR | DEVICE_AWAKE | LIGHT_SLEEP)

extern struct regmap *deepsleep_syscon_ap_ahb;
extern struct regmap *deepsleep_syscon_pmu_apb;
extern struct regmap *deepsleep_syscon_aon_apb;
#if 0
extern struct regmap *deepsleep_syscon_ana_apb;
#else
extern struct regmap *deepsleep_syscon_anlg_phy_g1;
extern struct regmap *deepsleep_syscon_anlg_phy_g2;
extern struct regmap *deepsleep_syscon_anlg_phy_g3;
extern struct regmap *deepsleep_syscon_anlg_phy_g4;
extern struct regmap *deepsleep_syscon_anlg_phy_g5;
extern struct regmap *deepsleep_syscon_ap_syst;
#endif
extern struct regmap *deepsleep_syscon_ap_apb;
extern struct regmap *deepsleep_syscon_aon_intc0;
extern struct regmap *deepsleep_syscon_aon_intc1;
extern struct regmap *deepsleep_syscon_aon_intc2;
extern struct regmap *deepsleep_syscon_aon_intc3;
extern struct regmap *deepsleep_syscon_aon_intc4;
extern struct regmap *deepsleep_syscon_aon_intc5;
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

#endif /* __SPRD_PM_DEBUG_ARM64_H__ */

