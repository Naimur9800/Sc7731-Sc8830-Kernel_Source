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
#ifndef __SPRD_PM_DEBUG_X86_64_H__
#define __SPRD_PM_DEBUG_X86_64_H__

#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/mfd/syscon/sprd/iwhale2/iwhale2_glb.h>
#include <linux/io.h>

/*******************INT********************/
#define   SPRD_INTC_NUM						8
#define   SPRD_INTC_CONTROLLER_NUM          4
#define   SPRD_HARD_INT_NUM_EACH_INTC		32
#define   SPRD_HARD_INTERRUPT_NUM	(SPRD_INTC_CONTROLLER_NUM * \
									SPRD_HARD_INT_NUM_EACH_INTC)

#define GPIO_GROUP_NUM			16
#define REG_GPIO_MIS			(0x0020)
#define	INTCV0_IRQ_RAW		(0x0004)
#define	INTCV1_IRQ_RAW		(0x0004)
#define	INTCV2_IRQ_RAW		(0x0004)
#define	INTCV3_IRQ_RAW		(0x0004)

/* bia interrupt request register*/
#define IOAPIC_BASE_PADDR (0xfec00000)
#define LINE_INT_STATUS0 (0x2040)
#define LINE_INT_STATUS1 (0x2044)
#define LINE_INT_STATUS2 (0x2048)
#define LINE_INT_STATUS3 (0x204c)
#define LINE_INT_STATUS4 (0x2050)
#define LINE_INT_STATUS5 (0x2054)
#define LINE_INT_STATUS6 (0x2058)
#define LINE_INT_STATUS7 (0x205c)

struct ap_ahb_reg_bak {
	u32 ahb_eb;
	u32 ca7_ckg_cfg;
	u32 ca7_div_cfg;
	u32 misc_ckg_en;
	u32 misc_cfg;
	u32 usb_phy_tune;
	u32 pub0_frc_lslp;
	u32 pub1_frc_lslp;
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
	regmap_read(deep_sys_ap_ahb, __flag, &mid); \
	if (mid & __bits) \
		pr_info(__string " not 0\n"); \
	} while (0)
#define AP_APB_BITS_CLEAR_CHECK(__flag, __bits, __string) do {\
	uint32_t mid ;\
	regmap_read(deep_sys_ap_apb, __flag, &mid); \
	if (mid & __bits) \
		pr_info(__string " not 0\n"); \
	} while (0)
#define AON_APB_BITS_CLEAR_CHECK(__flag, __bits, __string) do {\
	uint32_t mid ;\
	regmap_read(deep_sys_aon_apb, __flag, &mid); \
	if (mid & __bits) \
		pr_info(__string " not 0\n"); \
	} while (0)

#define DEVICE_AHB              (0x1UL << 20)
#define DEVICE_APB              (0x1UL << 21)
#define DEVICE_VIR              (0x1UL << 22)
#define DEVICE_AWAKE            (0x1UL << 23)
#define LIGHT_SLEEP             (0x1UL << 24)
#define DEVICE_TEYP_MASK        (DEVICE_AHB | DEVICE_APB |\
				DEVICE_VIR | DEVICE_AWAKE | LIGHT_SLEEP)

extern struct regmap *deep_sys_ap_ahb;
extern struct regmap *deep_sys_aon_pwu_apb;
extern struct regmap *deep_sys_aon_apb;
extern struct regmap *deep_sys_agcp_ahb;
extern struct regmap *deep_sys_ap_apb;
extern struct regmap *deep_sys_aon_common_apb;
extern struct regmap *deep_sys_com_pmu_apb;
extern struct regmap *deep_sys_bia_intc;
extern struct regmap *deepsleep_syscon_aon_intc0;
extern struct regmap *deepsleep_syscon_aon_intc1;
extern struct regmap *deepsleep_syscon_aon_intc2;
extern struct regmap *deepsleep_syscon_aon_intc3;
extern int is_intc_base_vaddr_ok;
extern int is_syscon_base_vaddr_ok;

void deepsleep_condition_check(void);
void bak_last_reg(void);
void print_last_reg(void);
void show_pin_reg(void);
void show_deep_reg_status(void);
void print_pmu_wakeup_sources(void);
void pm_debug_init(void);
void read_bia_int_req(void);
void set_sleep_mode(int sm);
void time_add(unsigned int time, int ret);
void print_bia_irq_status(void);
void get_hard_irq(void);
void print_hard_irq(void);

#endif /* __SPRD_PM_DEBUG_X86_64_H__ */

