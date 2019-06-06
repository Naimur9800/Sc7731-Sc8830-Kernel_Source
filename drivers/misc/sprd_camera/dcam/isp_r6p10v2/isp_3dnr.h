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

#ifndef _ISP_3DNR_H_
#define _ISP_3DNR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <video/sprd_mm.h>
#include <video/sprd_img.h>
#include <video/sprd_isp_r6p10v2.h>
#include "isp_reg.h"

#ifdef WIN32
#define DEBUG_NR3
#endif

#define SLICE_DIV	1500
#define SLICE_ROW_NUM	1
#define SLICE_COL_NUM	3
#define SLICE_NUM	(SLICE_ROW_NUM * SLICE_COL_NUM)

#define YUV_OVERLAP_UP			46
#define YUV_OVERLAP_DOWN		68
#define YUV_OVERLAP_LEFT		74
#define YUV_OVERLAP_RIGHT		126

struct slice_pos {
	unsigned int start_col;
	unsigned int start_row;
	unsigned int end_col;
	unsigned int end_row;
};

struct isp_3dnr_slice_param {
	unsigned int cur_img_width_fetch;
	unsigned int cur_img_height_fetch;
	unsigned int cur_img_addr_fetch[2];

	unsigned int ref_img_width_fetch[2];
	unsigned int ref_img_height_fetch[2];
	unsigned int ref_img_addr_fetch[2];
	unsigned int global_start_row;
	unsigned int global_start_col;
	unsigned int first_line_mode;
	unsigned int last_line_mode;

	unsigned int ref_img_width_store;
	unsigned int ref_img_height_store;
	unsigned int ref_img_addr_store[2];
	unsigned int ref_img_overlap_up;
	unsigned int ref_img_overlap_down;
	unsigned int ref_img_overlap_left;
	unsigned int ref_img_overlap_right;

	unsigned int trim1_size_x;
	unsigned int trim1_size_y;
	unsigned int comb2scl_hblank_num;
};

struct _3dnr_param {
	signed char  mv_x;
	signed char  mv_y;
	unsigned int fetch_cur_addr[2];
	unsigned int fetch_ref_addr[2];
	unsigned int store_ref_addr[2];
	unsigned int fetch_cur_endian;
	unsigned int fetch_ref_endian;
	unsigned int store_ref_endian;

	int image_width;
	int image_height;
	int blending_no;

	unsigned int isp_reg_base;

	unsigned int *fmcu_cmd_vbuf;
	unsigned int fmcu_cmd_pbuf;
	unsigned int fmcu_cmd_num;

	struct isp_3dnr_slice_param isp_3dnr_slice_param[SLICE_NUM];
	struct slice_overlap slice_overlap[SLICE_NUM];
	struct slice_pos slice_pos[SLICE_NUM];
	unsigned int slice_row_num;
	unsigned int slice_col_num;
	unsigned int slice_width;
	unsigned int slice_height;

};

struct isp_3dnr_info {
	signed char  mv_x;
	signed char  mv_y;
	unsigned int fetch_cur_addr[2];
	unsigned int fetch_ref_addr[2];
	unsigned int store_ref_addr[2];
	unsigned int fetch_cur_endian;
	unsigned int fetch_ref_endian;
	unsigned int store_ref_endian;

	int image_width;
	int image_height;
	int blending_no;

	unsigned int isp_reg_base;

	unsigned int *fmcu_cmd_vbuf;
	unsigned int fmcu_cmd_pbuf;
};

/************************************************************/
int isp_3dnr_process_one_frame(struct isp_3dnr_info *param);
int isp_3dnr_stop_preview(void);
/************************************************************/
#ifdef DEBUG_NR3
void REG_OR(unsigned int reg, unsigned int val);
void REG_AND(unsigned int reg, unsigned int val);
void REG_SET(unsigned int reg, unsigned int val);
unsigned REG_RD(unsigned int reg);


#define reg_or(reg, val)		REG_OR(reg, val)
#define reg_and(reg, val)		REG_AND(reg, val)
#define reg_set(reg, val)		REG_SET(reg, val)
#define reg_rd(reg)			REG_RD(reg)
#else
#define reg_or(reg, val)		ISP_REG_OWR(0, reg, (val))
#define reg_and(reg, val)		ISP_REG_WR(0, reg, (ISP_REG_RD(0, reg) \
							    & (val)))
#define reg_set(reg, val)		ISP_REG_WR(0, reg, (val))
#define reg_rd(reg)			ISP_REG_RD(0, reg)

#endif


#define ISP_REG_LEN					0x40000

#define ISP_STORE_B_CTRL				0x0214

#define ISP_COMMON_DISPATCH_SEL				0x0764
#define ISP_COMMON_PATH_SEL				0x0768

#define ISP_NR3_COMMON_PARAM0				0x9014
#define ISP_NR3_COMMON_CAP_SOF				0x9038
#define ISP_NR3_CAP_SHADOW_CLR				0x9040
#define ISP_NR3_FRM_CNT_CLR				0x9044
#define ISP_NR3_BLEND_CONTROL0				0x9214
#define ISP_NR3_BLEND_CFG0				0x9218
#define ISP_NR3_BLEND_CFG1				0x921C
#define ISP_NR3_BLEND_CFG2				0x9220
#define ISP_NR3_BLEND_CFG3				0x9224
#define ISP_NR3_BLEND_CFG4				0x9228
#define ISP_NR3_BLEND_CFG5				0x922C
#define ISP_NR3_BLEND_CFG6				0x9230
#define ISP_NR3_BLEND_CFG7				0x9234
#define ISP_NR3_BLEND_CFG8				0x9238
#define ISP_NR3_BLEND_CFG9				0x923C
#define ISP_NR3_BLEND_CFG10				0x9240
#define ISP_NR3_BLEND_CFG11				0x9244
#define ISP_NR3_BLEND_CFG12				0x9248
#define ISP_NR3_BLEND_CFG13				0x924C
#define ISP_NR3_BLEND_CFG14				0x9250
#define ISP_NR3_BLEND_CFG15				0x9254
#define ISP_NR3_BLEND_CFG16				0x9258
#define ISP_NR3_BLEND_CFG17				0x925C
#define ISP_NR3_BLEND_CFG18				0x9260
#define ISP_NR3_BLEND_CFG19				0x9264
#define ISP_NR3_BLEND_CFG20				0x9268
#define ISP_NR3_BLEND_CFG21				0x926C
#define ISP_NR3_BLEND_CFG22				0x9270
#define ISP_NR3_BLEND_CFG23				0x9274
#define ISP_NR3_BLEND_CFG24				0x9278
#define ISP_NR3_BLEND_CAP_GLOBAL_START_POS		0x927C

#define ISP_NR3_FAST_ME_PARAM0				0x9314
#define ISP_NR3_FAST_ME_PARAM1				0x9318

#define ISP_MEM_CTRL_PRE_PARAM1				0x9418
#define ISP_MEM_CTRL_PRE_FT_CUR_LUMA_ADDR0		0x9424
#define ISP_MEM_CTRL_PRE_FT_CUR_CHROMA_ADDR0		0x9428
#define ISP_MEM_CTRL_PRE_FT_REF_LUMA_ADDR0		0x942C
#define ISP_MEM_CTRL_PRE_FT_REF_CHROMA_ADDR0		0x9430
#define ISP_MEM_CTRL_PRE_PITCH				0x9434
#define ISP_MEM_CTRL_PRE_FRAME_CNT_CLR			0x945C

#define ISP_NR3_MEM_CTRL_CAP_PARAM0			0x9514
#define ISP_NR3_MEM_CTRL_CAP_FT_CUR_SIZE		0x9518
#define ISP_NR3_MEM_CTRL_CAP_HBLANK			0x951C
#define ISP_NR3_MEM_CTRL_CAP_ST_REF_ADDR0		0x9520
#define ISP_NR3_MEM_CTRL_CAP_ST_REF_ADDR1		0x9524
#define ISP_NR3_MEM_CTRL_CAP_FT_CUR_ADDR0		0x9528
#define ISP_NR3_MEM_CTRL_CAP_FT_CUR_ADDR1		0x952C
#define ISP_NR3_MEM_CTRL_CAP_FT_REF_ADDR0		0x9530
#define ISP_NR3_MEM_CTRL_CAP_FT_REF_ADDR1		0x9534
#define ISP_NR3_MEM_FT_CTRL_PITCH			0x9538
#define ISP_NR3_MEM_ST_CTRL_PITCH			0x953C
#define ISP_NR3_MEM_CTRL_CAP_FT_REF_SIZE0		0x9540
#define ISP_NR3_MEM_CTRL_CAP_FT_REF_SIZE1		0x9544
#define ISP_NR3_MEM_CTRL_CAP_ST_REF_BORDER		0x9548
#define ISP_NR3_MEM_CTRL_CAP_ST_REF_SIZE		0x954C
#define ISP_NR3_MEM_CTRL_CAP_PARAM7			0x9554
#define ISP_NR3_MEM_CTRL_CAP_GLOBAL_START_POS		0x9568
#define ISP_NR3_MEM_CTRL_CAP_PARAM13			0x956C
#define ISP_NR3_MEM_CTRL_CAP_LINE_MODE			0x9570

#define ISP_3DNR_STORE_PARAM				0x9614
#define ISP_NR3_SCL_CFG					0xA014
#define ISP_NR3_SCL_TRIM1_SIZE				0xA044

#define ISP_STORE_PRE_CTRL				0xC114
#define ISP_STORE_VID_CTRL				0xD114
#define ISP_STORE_CAP_CTRL				0xE114

#define ISP_FMCU_ADDR					0xF018

#ifdef __cplusplus
}
#endif

#endif
