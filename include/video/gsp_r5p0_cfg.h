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

#ifndef _GSP_R5P0_CFG_H
#define _GSP_R5P0_CFG_H
/**---------------------------------------------------------------------------*
 **                             Indepence                                     *
 **---------------------------------------------------------------------------*
*/
#include <linux/types.h>
#include <uapi/video/gsp_cfg.h>
#include <uapi/video/gsp_r5p0_cfg.h>
#include <video/gsp_cfg.h>

struct gsp_r5p0_img_layer {
	struct gsp_layer		common;
	struct gsp_r5p0_img_layer_params	params;
};

struct gsp_r5p0_osd_layer {
	struct gsp_layer		common;
	struct gsp_r5p0_osd_layer_params	params;
};

struct gsp_r5p0_des_layer {
	struct gsp_layer		common;
	struct gsp_r5p0_des_layer_params	params;
};

struct gsp_r5p0_misc_cfg {
	uint8_t gsp_gap;
	uint8_t split_pages;
	uint8_t cmd_cnt;
	uint8_t run_mod;
	uint8_t scale_seq;
	uint8_t pmargb_en;
	struct gsp_rect		workarea1_src_rect;
	struct gsp_rect		workarea2_src_rect;
	struct gsp_pos		workarea2_des_pos;
	struct gsp_scale_para	scale_para[R5P0_IMGL_NUM];
};

struct gsp_r5p0_cfg {
	struct gsp_cfg common;
	struct gsp_r5p0_img_layer limg[R5P0_IMGL_NUM];
	struct gsp_r5p0_osd_layer losd[R5P0_OSDL_NUM];
	struct gsp_r5p0_des_layer ld1;
	struct gsp_r5p0_misc_cfg misc;
};

#endif
