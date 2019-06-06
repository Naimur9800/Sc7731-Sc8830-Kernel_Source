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

#ifndef _GSP_R3P0_CFG_H
#define _GSP_R3P0_CFG_H
/**---------------------------------------------------------------------------*
 **                             Indepence                                     *
 **---------------------------------------------------------------------------*/
#include <linux/types.h>
#include <uapi/video/gsp_cfg.h>
#include <uapi/video/gsp_r3p0_cfg.h>
#include <video/gsp_cfg.h>

struct gsp_r3p0_img_layer {
	struct gsp_layer		common;
	struct gsp_r3p0_img_layer_params	params;
};

struct gsp_r3p0_osd_layer {
	struct gsp_layer		common;
	struct gsp_r3p0_osd_layer_params	params;
};

struct gsp_r3p0_des_layer {
	struct gsp_layer		common;
	struct gsp_r3p0_des_layer_params	params;
};

struct gsp_r3p0_misc_cfg {
	uint8_t gsp_gap;
	uint8_t split_pages;
	uint8_t cmd_cnt;
	uint8_t run_mod;
	uint8_t scale_seq;
	struct gsp_rect		workarea_src_rect;
	struct gsp_rect		workarea_dst_rect;
	struct gsp_scale_para	scale_para;
};

struct gsp_r3p0_cfg {
	struct gsp_cfg common;
	struct gsp_r3p0_img_layer l0;
	struct gsp_r3p0_osd_layer l1;
	struct gsp_r3p0_osd_layer l2;
	struct gsp_r3p0_osd_layer l3;
	struct gsp_r3p0_des_layer ld1;
	struct gsp_r3p0_des_layer ld2;
	struct gsp_r3p0_misc_cfg misc;
};

#endif
