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

#ifndef _ISP_POST_YNR_H_
#define _ISP_POST_YNR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "isp_3dnr.h"
#include "gen_scale_coef.h"
#include <video/sprd_mm.h>
#include <video/sprd_img.h>
#include <video/sprd_isp_r6p10.h>
#include "isp_reg.h"

struct isp_drv_ynr_info {
	unsigned int src_img_w;
	unsigned int src_img_h;
	unsigned int dst_img_w;
	unsigned int dst_img_h;
	unsigned long src_buf_addr;
	unsigned long dst_buf_addr;
	struct ynr_param ynr_param;
	unsigned int isp_reg_base;
};

int isp_ynr_process_one_frame(struct isp_drv_ynr_info *param);

#ifndef NULL
	#ifdef __cplusplus
		#define NULL    0
	#else
		#define NULL    ((void *)0)
	#endif
#endif

#define COUNT 256
#define PI 3.14159265357
#define SIGN2(input, p) {if (p >= 0) input = 1; if (p < 0) input = -1; }

#define YUV422			0
#define YUV420			1

#define SEMI_PLANAR	    0
#define PLANAR		    1

#define		LUMA			1
#define		CB				2
#define		CR				3


#define ROTATE0			0
#define ROTATE90		1 /*CCW 90	counterclockwise*/
#define ROTATE270		2 /*CW 90*/
#define ROTATE180		3 /*MIRROR and FLIP*/
#define MIRROR			4 /*horizontal mirror*/
#define FLIP			5 /*vertical*/

#define MIRROR_NO		0
#define MIRROR_HOR		1
#define MIRROR_VER		2
#define MIRROR_HOR_VER	3

#define YUVSCALER_OVERLAP_UP    32
#define YUVSCALER_OVERLAP_DOWN  52
#define YUVSCALER_OVERLAP_LEFT  16
#define YUVSCALER_OVERLAP_RIGHT 68

struct coef_info {
	unsigned int     y_hor_coef[48];
	unsigned int     y_ver_coef[132];
	unsigned int     c_ver_coef[132];
};

struct decimation_info {
	unsigned char	deci_x_en;
	unsigned char	deci_y_en;
	unsigned char   deci_x_parse;
	unsigned char   deci_y_parse;
	unsigned char	deci_x;
	unsigned char	deci_y;
	unsigned char	deci_cut_first_y;
	unsigned char	deci_option;
	unsigned short  reserved0;
};

struct initial_phase_info {
	signed short	scaler_init_phase_int[2][2];
	unsigned short	scaler_init_phase_rmd[2][2];
	signed int   scaler_initial_phase[2];
};

struct scaler_info {
	unsigned char	scaler_bypass;
	unsigned char	scaler_en;
	unsigned char	scaler_tap;
	unsigned char   scaler_y_ver_tap;
	unsigned char   scaler_uv_ver_tap;
	unsigned char	scaling2yuv420;
	unsigned char   reserved0;
	unsigned short  scaler_in_width;
	unsigned short	scaler_in_height;
	unsigned short	scaler_out_width;
	unsigned short	scaler_out_height;
	unsigned short	scaler_factor_in_hor;
	unsigned short	scaler_factor_out_hor;
	unsigned short	scaler_factor_in_ver;
	unsigned short	scaler_factor_out_ver;
	unsigned short  reserved1;

	struct initial_phase_info init_phase_info;
	struct coef_info scaler_coef_info;
};

struct trim_info {
	unsigned char	trim_eb;
	unsigned short	trim_start_x;
	unsigned short	trim_start_y;
	unsigned short	trim_size_x;
	unsigned short	trim_size_y;
};

struct rotation_info {
	unsigned char	rot_parse;
	unsigned char	rot_en;
	unsigned char	rot_dir;
	unsigned char	uv_average_deci;
	unsigned char   reserved0;
};

struct regular_yuv_info {
	unsigned char	shrink_y_range;
	unsigned char	shrink_uv_range;
	unsigned char	shrink_y_offset;
	unsigned char	shrink_uv_offset;
	unsigned char	shrink_uv_dn_th;
	unsigned char	shrink_uv_up_th;
	unsigned char	shrink_y_dn_th;
	unsigned char	shrink_y_up_th;
	unsigned char	effect_v_th;
	unsigned char	effect_u_th;
	unsigned char	effect_y_th;
	unsigned char	reserved0;
};

struct yuv_path_info {
	unsigned char			yuv_scaler_bypass;
	unsigned char     yuv_output_format;
	unsigned char			outdata_mode;
	unsigned char			outdata_format;
	unsigned char			regular_mode;
	unsigned short			src_size_x;
	unsigned short			src_size_y;
	struct trim_info		trim0_info;
	struct decimation_info		deci_info;
	struct scaler_info	scaler_info;
	struct trim_info		trim1_info;
	struct rotation_info rotation_info;
	struct regular_yuv_info   regular_yuv_info;
};

struct yuv_info {
	unsigned short       slice_mode;
	int                  scaler_id;
	struct yuv_path_info yuv_path;
};

#define MAX(_x, _y) (((_x) > (_y)) ? (_x) : (_y))

#define MIN(_x, _y) (((_x) < (_y)) ? (_x) : (_y))

#define	SAFE_FREE(x) {if (x) {vfree(x); x = NULL; } }

#ifdef __cplusplus
}
#endif

#endif
