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
#ifndef _GSP_R1P1_CORE_H
#define _GSP_R1P1_CORE_H

#include <linux/device.h>
#include <linux/list.h>
#include <video/gsp_cfg.h>
#include "gsp_core.h"
#include "scaler_coef_cal.h"

#define CORE_CLK ("clk_gsp")
#define CORE_PARENT_CLK ("clk_gsp_parent")
#define CORE_EMC_CLK ("clk_emc_gsp")
#define CORE_EN_CLK ("clk_gsp_eb")


#define GSP_CFG(base)  (base)
#define GSP_INT_CFG(base)  ((base) + 0x4)
#define GSP_CMD_ADDR(base)  ((base) + 0x8)
#define GSP_CMD_CFG(base)  ((base) + 0xc)
#define DES_DATA_CFG(base)  ((base) + 0x10)
#define DES_Y_ADDR(base)  ((base) + 0x14)
#define DES_UV_ADDR(base)  ((base) + 0x18)
#define DES_V_ADDR(base)  ((base) + 0x1c)
#define DES_PITCH_REG(base)  ((base) + 0x20)
#define DES_DATA_ENDIAN(base)  ((base) + 0x24)
#define LAYER0_DES_SIZE(base)  ((base) + 0x28)


/* LAYER0 register */
#define LAYER0_CFG(base)  ((base) + 0x30)
#define LAYER0_Y_ADDR(base)  ((base) + 0x34)
#define LAYER0_UV_ADDR(base)  ((base) + 0x38)
#define LAYER0_VA_ADDR(base)  ((base) + 0x3c)
#define LAYER0_PITCH(base)  ((base) + 0x40)
#define LAYER0_CLIP_START(base)  ((base) + 0x44)
#define LAYER0_CLIP_SIZE(base)  ((base) + 0x48)
#define LAYER0_DES_START(base)  ((base) + 0x4c)
#define LAYER0_GREY_RGB(base)  ((base) + 0x50)
#define LAYER0_ENDIAN(base)  ((base) + 0x54)
#define LAYER0_ALPHA(base)  ((base) + 0x58)
#define LAYER0_CK(base)  ((base) + 0x5c)

/* LAYER1 register */
#define LAYER1_CFG(base)  ((base) + 0x60)
#define LAYER1_Y_ADDR(base)  ((base) + 0x64)
#define LAYER1_UV_ADDR(base)  ((base) + 0x68)
#define LAYER1_VA_ADDR(base)  ((base) + 0x6c)
#define LAYER1_PITCH(base)  ((base) + 0x70)
#define LAYER1_CLIP_START(base)  ((base) + 0x74)
#define LAYER1_CLIP_SIZE(base)  ((base) + 0x78)
#define LAYER1_DES_START(base)  ((base) + 0x7c)
#define LAYER1_GREY_RGB(base)  ((base) + 0x80)
#define LAYER1_ENDIAN(base)  ((base) + 0x84)
#define LAYER1_ALPHA(base)  ((base) + 0x88)
#define LAYER1_CK(base)  ((base) + 0x8c)

/* GSP CFG OFFSET */
#define DIST_RB_OFFSET  (24)
#define L1_EN_OFFSET  (17)
#define L0_EN_OFFSET  (16)
#define SCL_CLR_OFFSET  (15)
#define BND_BYPASS_OFFSET  (14)
#define Y2R_OPT_OFFSET  (13)
#define SCALE_EN_OFFSET  (12)
#define PMARGB_EN_OFFSET  (11)
#define PMARGB_MOD_OFFSET  (9)
#define DITHER_EN_OFFSET  (8)
#define ERR_CODE_OFFSET  (3)
#define ERR_FLG_OFFSET  (2)
#define GSP_BUSY_OFFSET  (1)
#define GSP_RUN_OFFSET  (0)

/* GSP CFG */
#define DIST_RB  (0xff << DIST_RB_OFFSET)
#define L1_EN  (0x1 << L1_EN_OFFSET)
#define L0_EN  (0x1 << L0_EN_OFFSET)
#define SCL_CLR  (0x1 << SCL_CLR_OFFSET)
#define BND_BYPASS  (0x01 << BND_BYPASS_OFFSET)
#define Y2R_OPT  (0x01 << Y2R_OPT_OFFSET)
#define SCALE_EN  (0x1 << SCALE_EN_OFFSET)
#define PMARGB_EN  (0x1 << PMARGB_EN_OFFSET)
#define PMARGB_MOD  (0x3 << PMARGB_MOD_OFFSET)
#define DITHER_EN  (0x1 << DITHER_EN_OFFSET)
#define ERR_CODE  (0x1f << ERR_CODE_OFFSET)
#define ERR_FLG  (0x1 << ERR_FLG_OFFSET)
#define GSP_BUSY  (0x1 << GSP_BUSY_OFFSET)
#define GSP_RUN  (0x1 << GSP_RUN_OFFSET)

/* GSP INT GSP OFFSET */
#define INT_CLR_OFFSET  (16)
#define INT_MOD_OFFSET  (8)
#define INT_EN_OFFSET  (0)

/* GSP INT GSP */
#define INT_CLR  (0x1 << INT_CLR_OFFSET)
#define INT_MOD  (0x1 << INT_MOD_OFFSET)
#define INT_EN  (0x1 << INT_EN_OFFSET)

/* GSP CMD ADDR OFFSET */
#define CMD_BASE_ADDR_OFFSET  (0)

/* GSP CMD ADDR */
#define CMD_BASE_ADDR  (0xffff << CMD_BASE_ADDR_OFFSET)

/* GSP CMD CFG OFFSET */
#define CMD_ENDIAN_MOD_OFFSET  (24)
#define CMD_EN_OFFSET  (16)
#define CMD_NUM_OFFSET  (0)

/* GSP CMD CFG */
#define CMD_ENDIAN_MOD  (0x7 << CMD_ENDIAN_MOD_OFFSET)
#define CMD_EN  (0x1 << CMD_EN_OFFSET)
#define CMD_NUM  (0xff << CMD_NUM_OFFSET)

/* DES DATA CFG OFFSET */
#define COMPRESS_R8_OFFSET  (8)
#define DES_IMG_FORMAT_OFFSET  (0)

/* DES DATA CFG */
#define COMPRESS_R8  (0x1 << COMPRESS_R8_OFFSET)
#define DES_IMG_FORMAT  (0x7 << DES_IMG_FORMAT_OFFSET)

/* DES Y ADDR OFFSET */
#define DES_Y_BASE_ADDR_OFFSET  (0)

/* DES Y ADDR */
#define DES_Y_BASE_ADDR  (0xffffffff << DES_Y_BASE_ADDR_OFFSET)

/* DES UV ADDR OFFSET */
#define DES_UV_BASE_ADDR_OFFSET  (0)

/* DES UV ADDR */
#define DES_UV_BASE_ADDR  (0xffffffff << DES_UV_BASE_ADDR_OFFSET)

/* DES V ADDR OFFSET */
#define DES_V_BASE_ADDR_OFFSET  (0)

/* DES V ADDR */
#define DES_V_BASE_ADDR  (0xffffffff << DES_V_BASE_ADDR_OFFSET)

/* DES PITCH OFFSET */
#define DES_PITCH_OFFSET  (0)

/* DES PITCH */
#define DES_PITCH  (0xfff << DES_PITCH_OFFSET)

/* DES DATA ENDIAN OFFSET */
#define A_SWAP_MOD_OFFSET  (12)
#define RGB_SWAP_MOD_OFFSET  (9)
#define V_ENDIAN_MOD_OFFSET  (6)
#define UV_ENDIAN_MOD_OFFSET  (3)
#define Y_ENDIAN_MOD_OFFSET  (0)

/* DES DATA ENDIAN */
#define A_SWAP_MOD  (0x1 << A_SWAP_MOD_OFFSET)
#define RGB_SWAP_MOD  (0x7 << RGB_SWAP_MOD_OFFSET)
#define V_ENDIAN_MOD  (0x7 << V_ENDIAN_MOD_OFFSET)
#define UV_ENDIAN_MOD  (0x7 << UV_ENDIAN_MOD_OFFSET)
#define Y_ENDIAN_MOD  (0x07 << Y_ENDIAN_MOD_OFFSET)

/* LAYER0 DES SIZE OFFSET */
#define DES_SIZE_Y_L0_OFFSET  (16)
#define DES_SIZE_X_L0_OFFSET  (0)

/* LAYER0 DES SIZE */
#define DES_SIZE_Y_L0  (0xfff << DES_SIZE_Y_L0_OFFSET)
#define DES_SIZE_X_L0  (0xfff << DES_SIZE_X_L0_OFFSET)

/* LAYER0 CFG OFFSET*/
#define COL_TAP_MOD_OFFSET  (26)
#define ROW_TAP_MOD_OFFSET  (24)
#define PALLET_EN_L0_OFFSET  (20)
#define CK_EN_L0_OFFSET  (19)
#define ROT_MOD_L0_OFFSET  (16)
#define IMG_FORMAT_L0_OFFSET  (0)

/* LAYER0 CFG */
#define COL_TAP_MOD  (0x3 << COL_TAP_MOD_OFFSET)
#define ROW_TAP_MOD  (0x3 << ROW_TAP_MOD_OFFSET)
#define PALLET_EN_L0  (0x1 << PALLET_EN_L0_OFFSET)
#define CK_EN_L0  (0x1 << CK_EN_L0_OFFSET)
#define ROT_MOD_L0  (0x7 << ROT_MOD_L0_OFFSET)
#define IMG_FORMAT_L0  (0xf << IMG_FORMAT_L0_OFFSET)

/* LAYER0 Y ADDR OFFSET */
#define Y_BASE_ADDR_L0_OFFSET  (0)

/* LAYER0 Y ADDR */
#define Y_BASE_ADDR_L0  (0xffffffff << Y_BASE_ADDR_L0_OFFSET)

/* LAYER0 UV ADDR OFFSET */
#define UV_BASE_ADDR_L0_OFFSET  (0)

/* LAYER0 UV ADDR */
#define UV_BASE_ADDR_L0  (0xffffffff << UV_BASE_ADDR_L0_OFFSET)

/* LAYER0 VA ADDR OFFSET */
#define VA_BASE_ADDR_L0_OFFSET  (0)

/* LAYER0 VA ADDR */
#define VA_BASE_ADDR_L0  (0xffffffff << VA_BASE_ADDR_L0_OFFSET)

/* LAYER0 PITCH OFFSET*/
#define PITCH0_OFFSET  (0)

/* LAYER0 PITCH*/
#define PITCH0  (0xfff << PITCH0_OFFSET)

/* LAYER0 CLIP START OFFSET */
#define CLIP_START_Y_L0_OFFSET  (16)
#define CLIP_START_X_L0_OFFSET  (0)

/* LAYER0 CLIP START */
#define CLIP_START_Y_L0  (0xfff << CLIP_START_Y_L0_OFFSET)
#define CLIP_START_X_L0  (0xfff << CLIP_START_X_L0_OFFSET)

/* LAYER0 CLIP SIZE OFFSET */
#define CLIP_SIZE_Y_L0_OFFSET  (16)
#define CLIP_SIZE_X_L0_OFFSET  (0)

/* LAYER0 CLIP SIZE */
#define CLIP_SIZE_Y_L0  (0xfff << 16)
#define CLIP_SIZE_X_L0  (0xfff)

/* LAYER0 DES START OFFSET */
#define DES_START_Y_L0_OFFSET  (16)
#define DES_START_X_L0_OFFSET  (0)

/* LAYER0 DES START */
#define DES_START_Y_L0  (0xfff << DES_START_Y_L0_OFFSET)
#define DES_START_X_L0  (0xfff << DES_START_X_L0_OFFSET)

/* LAYER0 GREY RGB OFFSET */
#define LAYER0_GREY_R_OFFSET  (16)
#define LAYER0_GREY_G_OFFSET  (8)
#define LAYER0_GREY_B_OFFSET  (0)

/* LAYER0 GREY RGB */
#define LAYER0_GREY_R  (0xff << LAYER0_GREY_R_OFFSET)
#define LAYER0_GREY_G  (0xff << LAYER0_GREY_G_OFFSET)
#define LAYER0_GREY_B  (0xff << LAYER0_GREY_B_OFFSET)

/* LAYER0 ENDIAN OFFSET */
#define A_SWAP_MOD_L0_OFFSET  (12)
#define RGB_SWAP_MOD_L0_OFFSET  (9)
#define VA_ENDIAN_MOD_L0_OFFSET  (6)
#define UV_ENDIAN_MOD_L0_OFFSET  (3)
#define Y_ENDIAN_MOD_L0_OFFSET  (0)

/* LAYER0 ENDIAN */
#define A_SWAP_MOD_L0  (0x1 << A_SWAP_MOD_L0_OFFSET)
#define RGB_SWAP_MOD_L0  (0x7 << RGB_SWAP_MOD_L0_OFFSET)
#define VA_ENDIAN_MOD_L0  (0x7 << VA_ENDIAN_MOD_L0_OFFSET)
#define UV_ENDIAN_MOD_L0  (0x7 << UV_ENDIAN_MOD_L0_OFFSET)
#define Y_ENDIAN_MOD_L0  (0x7 << Y_ENDIAN_MOD_L0_OFFSET)

/* LAYER0 ALPHA OFFSET */
#define ALPHA_L0_OFFSET  (0)

/* LAYER0 ALPHA */
#define ALPHA_L0  (0xff << ALPHA_L0_OFFSET)

/* LAYER0 CK OFFSET */
#define CK_R_L0_OFFSET  (16)
#define CK_G_L0_OFFSET  (8)
#define CK_B_L0_OFFSET  (0)

/* LAYER0 CK */
#define CK_R_L0  (0xff << CK_R_L0_OFFSET)
#define CK_G_L0  (0xff << CK_G_L0_OFFSET)
#define CK_B_L0  (0xff << CK_B_L0_OFFSET)

/* LAYER1 CFG OFFSET */
#define PALLET_EN_L1_OFFSET  (20)
#define CK_EN_L1_OFFSET  (19)
#define ROT_MOD_L1_OFFSET  (16)
#define IMG_FORMAT_L1_OFFSET  (0)

/* LAYER1 CFG */
#define PALLET_EN_L1  (0x1 << PALLET_EN_L1_OFFSET)
#define CK_EN_L1  (0x1 << CK_EN_L1_OFFSET)
#define ROT_MOD_L1  (0x7 << ROT_MOD_L1_OFFSET)
#define IMG_FORMAT_L1  (0xf << IMG_FORMAT_L1_OFFSET)

/* LAYER1 Y ADDR OFFSET */
#define Y_BASE_ADDR_L1_OFFSET  (0)

/* LAYER1 Y ADDR */
#define Y_BASE_ADDR_L1  (0xffffffff << Y_BASE_ADDR_L1_OFFSET)

/* LAYER1 UV ADDR OFFSET */
#define UV_BASE_ADDR_L1_OFFSET  (0)

/* LAYER1 UV ADDR */
#define UV_BASE_ADDR_L1  (0xffffffff << UV_BASE_ADDR_L1_OFFSET)

/* LAYER1 VA ADDR OFFSET */
#define VA_BASE_ADDR_L1_OFFSET  (0)

/* LAYER1 VA ADDR */
#define VA_BASE_ADDR_L1  (0xffffffff << VA_BASE_ADDR_L1_OFFSET)

/* LAYER1 PITCH OFFSET */
#define PITCH1_OFFSET  (0)

/* LAYER1 PITCH */
#define PITCH1  (0xfff << PITCH1_OFFSET)

/* LAYER1 CLIP START OFFSET */
#define CLIP_START_Y_L1_OFFSET  (16)
#define CLIP_START_X_L1_OFFSET  (0)

/* LAYER1 CLIP START */
#define CLIP_START_Y_L1  (0xfff << CLIP_START_Y_L1_OFFSET)
#define CLIP_START_X_L1  (0xfff << CLIP_START_X_L1_OFFSET)

/* LAYER1 CLIP SIZE OFFSET */
#define CLIP_SIZE_Y_L1_OFFSET  (16)
#define CLIP_SIZE_X_L1_OFFSET  (0)

/* LAYER1 CLIP SIZE */
#define CLIP_SIZE_Y_L1  (0xfff << CLIP_SIZE_Y_L1_OFFSET)
#define CLIP_SIZE_X_L1  (0xfff << CLIP_SIZE_X_L1_OFFSET)

/* LAYER1 DES START OFFSET */
#define DES_START_Y_L1_OFFSET (16)
#define DES_START_X_L1_OFFSET (0)

/* LAYER1 DES START */
#define DES_START_Y_L1 (0xfff << DES_START_Y_L1_OFFSET)
#define DES_START_X_L1 (0xfff << DES_START_X_L1_OFFSET)

/* LAYER1 GREY RGB OFFSET */
#define GREY_R_L1_OFFSET  (16)
#define GREY_G_L1_OFFSET  (8)
#define GREY_B_L1_OFFSET  (0)

/* LAYER1 GREY RGB */
#define GREY_R_L1  (0xff << GREY_R_L1_OFFSET)
#define GREY_G_L1  (0xff << GREY_G_L1_OFFSET)
#define GREY_B_L1  (0xff << GREY_B_L1_OFFSET)

/* LAYER1 ENDIAN OFFSET */
#define A_SWAP_MOD_L1_OFFSET  (12)
#define RGB_SWAP_MOD_L1_OFFSET  (9)
#define VA_ENDIAN_MOD_L1_OFFSET  (6)
#define UV_ENDIAN_MOD_L1_OFFSET  (3)
#define Y_ENDIAN_MOD_L1_OFFSET  (0)

/* LAYER1 ENDIAN */
#define A_SWAP_MOD_L1  (0x1 << A_SWAP_MOD_L1_OFFSET)
#define RGB_SWAP_MOD_L1  (0x7 << RGB_SWAP_MOD_L1_OFFSET)
#define VA_ENDIAN_MOD_L1  (0x7 << VA_ENDIAN_MOD_L1_OFFSET)
#define UV_ENDIAN_MOD_L1  (0x7 << UV_ENDIAN_MOD_L1_OFFSET)
#define Y_ENDIAN_MOD_L1  (0x7 << Y_ENDIAN_MOD_L1_OFFSET)

/* LAYER1 ALPHA OFFSET */
#define ALPHA_L1_OFFSET  (0)

/* LAYER1 ALPHA */
#define ALPHA_L1  (0xff)

/* LAYER1 CK OFFSET */
#define CK_R_L1_OFFSET  (16)
#define CK_G_L1_OFFSET  (8)
#define CK_B_L1_OFFSET  (0)

/* LAYER1 CK OFFSET */
#define CK_R_L1  (0xff << CK_R_L1_OFFSET)
#define CK_G_L1  (0xff << CK_G_L1_OFFSET)
#define CK_B_L1  (0xff << CK_B_L1_OFFSET)

#define COEF_CACHE_MAX  32

struct gsp_r1p1_core {
	struct gsp_core common;

	struct clk *core_clk;
	struct clk *core_pclk;
	struct clk *core_emc_clk;
	struct clk *core_en_clk;

	struct coef_entry coef_cache[COEF_CACHE_MAX];
	struct list_head coef_list;
};


int gsp_r1p1_core_parse_dt(struct gsp_core *core);

int gsp_r1p1_core_copy_cfg(struct gsp_kcfg *kcfg,
			void *arg, int index);
int __user *gsp_r1p1_core_intercept(void __user *arg, int index);

int gsp_r1p1_core_init(struct gsp_core *core);
int gsp_r1p1_core_alloc(struct gsp_core **core, struct device_node *node);

int gsp_r1p1_core_trigger(struct gsp_core *core);
int gsp_r1p1_core_release(struct gsp_core *core);

int gsp_r1p1_core_enable(struct gsp_core *core);
void gsp_r1p1_core_disable(struct gsp_core *core);

void gsp_r1p1_core_reset(struct gsp_core *core);
void gsp_r1p1_core_dump(struct gsp_core *core);
#endif
