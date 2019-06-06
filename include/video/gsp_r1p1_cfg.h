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

#ifndef _VIDEO_gsp_r1p1_CFG_H
#define _VIDEO_gsp_r1p1_CFG_H


#include <linux/types.h>
#include <uapi/video/gsp_cfg.h>
#include <uapi/video/gsp_r1p1_cfg.h>
#include <video/gsp_cfg.h>

struct gsp_r1p1_img_layer {
	struct gsp_layer		common;
	struct gsp_r1p1_img_layer_params	params;
};

struct gsp_r1p1_osd_layer {
	struct gsp_layer		common;
	struct gsp_r1p1_osd_layer_params	params;
};

struct gsp_r1p1_des_layer {
	struct gsp_layer		common;
	struct gsp_r1p1_des_layer_params	params;
};

struct gsp_r1p1_cfg {
	struct gsp_cfg common;
	struct gsp_r1p1_img_layer l0;
	struct gsp_r1p1_osd_layer l1;
	struct gsp_r1p1_des_layer ld;
	struct gsp_r1p1_misc_cfg misc;
};


#endif
