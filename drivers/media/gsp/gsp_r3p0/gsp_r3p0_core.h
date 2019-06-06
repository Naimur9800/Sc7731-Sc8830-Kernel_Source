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
#ifndef _GSP_R3P0_CORE_H
#define _GSP_R3P0_CORE_H

#include <linux/device.h>
#include <linux/list.h>
#include <video/gsp_cfg.h>
#include "gsp_core.h"
#include "gsp_debug.h"

#define R3P0_GSP_CLOCK_PARENT		   ("clk_gsp_parent")
#define R3P0_GSP0_CLOCK_NAME			 ("clk_gsp0")
#define R3P0_GSP1_CLOCK_NAME			 ("clk_gsp1")

#define R3P0_GSP0_MMU_EB_CLOCK_NAME			 ("clk_gsp0_mmu_eb")
#define R3P0_GSP1_MMU_EB_CLOCK_NAME			 ("clk_gsp1_mmu_eb")
#define R3P0_GSP0_EB_CLOCK_NAME			 ("clk_gsp0_eb")
#define R3P0_GSP1_EB_CLOCK_NAME			 ("clk_gsp1_eb")
#define R3P0_GSP0_FORCE_GATE_CLOCK_NAME			 ("clk_gsp0_force_gate")
#define R3P0_GSP1_FORCE_GATE_CLOCK_NAME			 ("clk_gsp1_force_gate")
#define R3P0_GSP0_AUTO_EB_CLOCK_NAME			 ("clk_gsp0_auto_eb")
#define R3P0_GSP1_AUTO_EB_CLOCK_NAME			 ("clk_gsp1_auto_eb")

#define MIN_POOL_SIZE				   (6 * 1024)
#define SCALER_COEF_TAB_LEN_HOR		 48
#define SCALER_COEF_TAB_LEN_VER		 68
#define R3P0_GSP_COEF_CACHE_MAX		 32
#define GSC_COUNT					   64

/* Global config reg */
#define R3P0_GSP_GLB_CFG(base)			 (base)
#define R3P0_GSP_MOD1_CFG(base)			(base + 0x004)
#define R3P0_GSP_MOD2_CFG(base)			(base + 0x008)
#define R3P0_GSP_CMD_STS0(base)			(base + 0x00C)
#define R3P0_GSP_CMD_STS1(base)			(base + 0x010)
#define R3P0_GSP_CMD_ADDR(base)			(base + 0x014)

/*Destination reg 1*/
#define R3P0_DES_DATA_CFG1(base)		   (base + 0x018)
#define R3P0_DES_Y_ADDR1(base)			 (base + 0x01C)
#define R3P0_DES_U_ADDR1(base)			 (base + 0x020)
#define R3P0_DES_V_ADDR1(base)			 (base + 0x024)
#define R3P0_DES_PITCH1(base)			  (base + 0x028)
#define R3P0_BACK_RGB(base)				(base + 0x02C)
#define R3P0_DES_SCL_SIZE(base)			(base + 0x030)
#define R3P0_WORK_DES_START(base)		  (base + 0x034)
#define R3P0_WORK_SCR_SIZE(base)		   (base + 0x038)
#define R3P0_WORK_SCR_START(base)		  (base + 0x03C)

/*Destination reg 2*/
#define R3P0_DES_DATA_CFG2(base)		   (base + 0x040)
#define R3P0_DES_Y_ADDR2(base)			 (base + 0x044)
#define R3P0_DES_U_ADDR2(base)			 (base + 0x048)
#define R3P0_DES_V_ADDR2(base)			 (base + 0x04C)
#define R3P0_DES_PITCH2(base)			  (base + 0x050)
#define R3P0_AXI_DEBUG(base)			   (base + 0x054)
#define R3P0_GSP_INT(base)				 (base + 0x058)

/*LAYER0*/
#define R3P0_LAYER0_CFG(base)			  (base + 0x05C)
#define R3P0_LAYER0_Y_ADDR(base)		   (base + 0x060)
#define R3P0_LAYER0_U_ADDR(base)		   (base + 0x064)
#define R3P0_LAYER0_V_ADDR(base)		   (base + 0x068)
#define R3P0_LAYER0_PITCH(base)			(base + 0x06C)
#define R3P0_LAYER0_CLIP_START(base)	   (base + 0x070)
#define R3P0_LAYER0_CLIP_SIZE(base)		(base + 0x074)
#define R3P0_LAYER0_DES_START(base)		(base + 0x078)
#define R3P0_LAYER0_PALLET_RGB(base)	   (base + 0x07C)
#define R3P0_LAYER0_CK(base)			   (base + 0x080)
#define R3P0_Y2R_Y_PARAM(base)			 (base + 0x084)
#define R3P0_Y2R_U_PARAM(base)			 (base + 0x088)
#define R3P0_Y2R_V_PARAM(base)			 (base + 0x08C)

/*LAYER1*/
#define R3P0_LAYER1_CFG(base)			  (base + 0x090)
#define R3P0_LAYER1_R_ADDR(base)		   (base + 0x094)
#define R3P0_LAYER1_PITCH(base)			(base + 0x098)
#define R3P0_LAYER1_CLIP_START(base)	   (base + 0x09C)
#define R3P0_LAYER1_CLIP_SIZE(base)		(base + 0x0A0)
#define R3P0_LAYER1_DES_START(base)		(base + 0x0A4)
#define R3P0_LAYER1_PALLET_RGB(base)	   (base + 0x0A8)
#define R3P0_LAYER1_CK(base)			   (base + 0x0AC)

/*LAYER2*/
#define R3P0_LAYER2_CFG(base)			  (base + 0x0B0)
#define R3P0_LAYER2_R_ADDR(base)		   (base + 0x0B4)
#define R3P0_LAYER2_PITCH(base)			(base + 0x0B8)
#define R3P0_LAYER2_CLIP_START(base)	   (base + 0x0BC)
#define R3P0_LAYER2_CLIP_SIZE(base)		(base + 0x0C0)
#define R3P0_LAYER2_DES_START(base)		(base + 0x0C4)
#define R3P0_LAYER2_PALLET_RGB(base)	   (base + 0x0C8)
#define R3P0_LAYER2_CK(base)			   (base + 0x0CC)

/*LAYER3*/
#define R3P0_LAYER3_CFG(base)			  (base + 0x0D0)
#define R3P0_LAYER3_R_ADDR(base)		   (base + 0x0D4)
#define R3P0_LAYER3_PITCH(base)			(base + 0x0D8)
#define R3P0_LAYER3_CLIP_START(base)	   (base + 0x0DC)
#define R3P0_LAYER3_CLIP_SIZE(base)		(base + 0x0E0)
#define R3P0_LAYER3_DES_START(base)		(base + 0x0E4)
#define R3P0_LAYER3_PALLET_RGB(base)	   (base + 0x0E8)
#define R3P0_LAYER3_CK(base)			   (base + 0x0EC)

#define R3P0_GSP_MAIN_FSM(base)		 (base + 0x0F0)
#define R3P0_GSP_IOUT_FSM(base)		 (base + 0x0F4)


/*HORIZONTAL COEF TABLE (72*8)*/
#define R3P0_HOR_COEF_1_12(base)		   (base + 0x100)
#define R3P0_HOR_COEF_1_34(base)		   (base + 0x104)
#define R3P0_HOR_COEF_1_56(base)		   (base + 0x108)
#define R3P0_HOR_COEF_1_78(base)		   (base + 0x10C)
#define R3P0_HOR_COEF_2_12(base)		   (base + 0x110)
#define R3P0_HOR_COEF_2_34(base)		   (base + 0x114)
#define R3P0_HOR_COEF_2_56(base)		   (base + 0x118)
#define R3P0_HOR_COEF_2_78(base)		   (base + 0x11C)
#define R3P0_HOR_COEF_3_12(base)		   (base + 0x120)
#define R3P0_HOR_COEF_3_34(base)		   (base + 0x124)
#define R3P0_HOR_COEF_3_56(base)		   (base + 0x128)
#define R3P0_HOR_COEF_3_78(base)		   (base + 0x12C)
#define R3P0_HOR_COEF_4_12(base)		   (base + 0x130)
#define R3P0_HOR_COEF_4_34(base)		   (base + 0x134)
#define R3P0_HOR_COEF_4_56(base)		   (base + 0x138)
#define R3P0_HOR_COEF_4_78(base)		   (base + 0x13C)
#define R3P0_HOR_COEF_5_12(base)		   (base + 0x140)
#define R3P0_HOR_COEF_5_34(base)		   (base + 0x144)
#define R3P0_HOR_COEF_5_56(base)		   (base + 0x148)
#define R3P0_HOR_COEF_5_78(base)		   (base + 0x14C)
#define R3P0_HOR_COEF_6_12(base)		   (base + 0x150)
#define R3P0_HOR_COEF_6_34(base)		   (base + 0x154)
#define R3P0_HOR_COEF_6_56(base)		   (base + 0x158)
#define R3P0_HOR_COEF_6_78(base)		   (base + 0x15C)
#define R3P0_HOR_COEF_7_12(base)		   (base + 0x160)
#define R3P0_HOR_COEF_7_34(base)		   (base + 0x164)
#define R3P0_HOR_COEF_7_56(base)		   (base + 0x168)
#define R3P0_HOR_COEF_7_78(base)		   (base + 0x16C)
#define R3P0_HOR_COEF_8_12(base)		   (base + 0x170)
#define R3P0_HOR_COEF_8_34(base)		   (base + 0x174)
#define R3P0_HOR_COEF_8_56(base)		   (base + 0x178)
#define R3P0_HOR_COEF_8_78(base)		   (base + 0x17C)

/*HORIZONTAL COEF TABLE (72*8)*/
#define R3P0_VER_COEF_1_12(base)		   (base + 0x180)
#define R3P0_VER_COEF_1_34(base)		   (base + 0x184)
#define R3P0_VER_COEF_2_12(base)		   (base + 0x188)
#define R3P0_VER_COEF_2_34(base)		   (base + 0x18C)
#define R3P0_VER_COEF_3_12(base)		   (base + 0x190)
#define R3P0_VER_COEF_3_34(base)		   (base + 0x194)
#define R3P0_VER_COEF_4_12(base)		   (base + 0x198)
#define R3P0_VER_COEF_4_34(base)		   (base + 0x19C)
#define R3P0_VER_COEF_5_12(base)		   (base + 0x1A0)
#define R3P0_VER_COEF_5_34(base)		   (base + 0x1A4)
#define R3P0_VER_COEF_6_12(base)		   (base + 0x1A8)
#define R3P0_VER_COEF_6_34(base)		   (base + 0x1AC)
#define R3P0_VER_COEF_7_12(base)		   (base + 0x1B0)
#define R3P0_VER_COEF_7_34(base)		   (base + 0x1B4)
#define R3P0_VER_COEF_8_12(base)		   (base + 0x1B8)
#define R3P0_VER_COEF_8_34(base)		   (base + 0x1BC)

/*ROT*/
#define R3P0_ROT_MOD_CFG(base)			 (base + 0x200)
#define R3P0_ROT_SRC_ADDR(base)			(base + 0x204)
#define R3P0_ROT_DES_ADDR(base)			(base + 0x208)
#define R3P0_ROT_PITCH(base)			   (base + 0x20C)
#define R3P0_ROT_CLIP_START(base)		  (base + 0x210)
#define R3P0_ROT_CLIP_SIZE(base)		   (base + 0x214)

#define R3P0_GSP_HOR_COEF_BASE(base)		   (base + 0x100)
#define R3P0_GSP_VER_COEF_BASE(base)		   (base + 0x180)

struct gsp_r3p0_core {
	struct gsp_core common;
	struct list_head coef_list;
	struct COEF_ENTRY_T coef_cache[R3P0_GSP_COEF_CACHE_MAX];

	ulong gsp_coef_force_calc;
	uint32_t cache_coef_init_flag;
	int32_t coef_calc_buf[SCALER_COEF_TAB_LEN_HOR+SCALER_COEF_TAB_LEN_VER];
	char coef_buf_pool[MIN_POOL_SIZE];

	/* module ctl reg base, virtual	0x63500000 */
	void *gsp_ctl_reg_base;
	/* 0x63100000 : GSP0, GSP0_MMU Eb */
	uint32_t *ahb_en_reg_base;
	uint32_t *cgm_gsp_clk_reg_base;
	struct regmap *disp_ahb;

	struct clk *gsp_clk;
	struct clk *gsp_clk_parent;
	struct clk *gsp_mmu_eb_clk;
	struct clk *gsp_eb_clk;
	struct clk *gsp_force_gate_clk;
	struct clk *gsp_auto_gate_clk;
};

#define MEM_OPS_ADDR_ALIGN_MASK (0x7UL)

int gsp_r3p0_core_parse_dt(struct gsp_core *core);

int gsp_r3p0_core_copy_cfg(struct gsp_kcfg *kcfg, void *arg, int index);

int gsp_r3p0_core_init(struct gsp_core *core);

int gsp_r3p0_core_alloc(struct gsp_core **core, struct device_node *node);

int gsp_r3p0_core_trigger(struct gsp_core *core);

int gsp_r3p0_core_enable(struct gsp_core *core);

void gsp_r3p0_core_disable(struct gsp_core *core);

int gsp_r3p0_core_release(struct gsp_core *core);

int __user *gsp_r3p0_core_intercept(void __user *arg, int index);
void gsp_r3p0_core_reset(struct gsp_core *core);
void gsp_r3p0_core_dump(struct gsp_core *core);

#endif
