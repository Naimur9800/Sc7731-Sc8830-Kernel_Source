/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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

#ifndef _ISP_SLICE_HEADER_
#define _ISP_SLICE_HEADER_

#include "isp_drv.h"

#define SLICE_NUM_MAX			9

enum isp_channel_idx {
	ISP_CH_0,
	ISP_CH_1,
	ISP_CH_MAX,
};

enum slice_path_index {
	SLICE_PATH_PRE,
	SLICE_PATH_VID,
	SLICE_PATH_CAP,
	SLICE_PATH_MAX,
};

enum isp_fetchYUV_format {
	ISP_FETCHYUV_YUV422_3FRAME = 0,
	ISP_FETCHYUV_YUYV,
	ISP_FETCHYUV_UYVY,
	ISP_FETCHYUV_YVYU,
	ISP_FETCHYUV_VYUY,
	ISP_FETCHYUV_YUV422_2FRAME,
	ISP_FETCHYUV_YVU422_2FRAME,
	ISP_FETCHYUV_YUV420_2FRAME = 10,
	ISP_FETCHYUV_YVU420_2FRAME,
	ISP_FETCHYUV_FORMAT_MAX
};

enum isp_store_format {
	ISP_STORE_UYVY = 0x00,
	ISP_STORE_YUV422_2FRAME,
	ISP_STORE_YVU422_2FRAME,
	ISP_STORE_YUV422_3FRAME,
	ISP_STORE_YUV420_2FRAME,
	ISP_STORE_YVU420_2FRAME,
	ISP_STORE_YUV420_3FRAME,
	ISP_STORE_RAW10,
	ISP_STORE_FULL_RGB8,
	ISP_STORE_FORMAT_MAX
};

struct slice_img_size {
	unsigned int width;
	unsigned int height;
};

struct slice_addr {
	unsigned int chn0;
	unsigned int chn1;
	unsigned int chn2;
};

struct slice_pitch {
	unsigned int chn0;
	unsigned int chn1;
	unsigned int chn2;
};

struct slice_border {
	unsigned int up_border;
	unsigned int down_border;
	unsigned int left_border;
	unsigned int right_border;
};

struct slice_pos_info {
	unsigned int start_col;
	unsigned int start_row;
	unsigned int end_col;
	unsigned int end_row;
};

struct slice_overlap_info {
	unsigned int overlap_up;
	unsigned int overlap_down;
	unsigned int overlap_left;
	unsigned int overlap_right;
};

struct slice_base_info {
	struct slice_pos_info slice_pos_array[SLICE_NUM_MAX];
	struct slice_overlap_info slice_overlap_array[SLICE_NUM_MAX];
	struct slice_pitch pitch;
	unsigned int cur_slice_id;
	unsigned int slice_row_num;
	unsigned int slice_col_num;
	unsigned int slice_num;
	unsigned int slice_height;
	unsigned int slice_width;
	unsigned int img_width;
	unsigned int img_height;
	unsigned int store_width;
	unsigned int store_height;
	unsigned int yuv_overlap_up;
	unsigned int yuv_overlap_down;
	unsigned int yuv_overlap_left;
	unsigned int yuv_overlap_right;
	unsigned int yuvscaler_overlap_right;
	unsigned int isp_jpg_cowork;
};

struct slice_fetchYUV_info {
	struct slice_img_size size;
	struct slice_addr addr;
};

struct slice_store_info {
	struct slice_img_size size;
	struct slice_border border;
	struct slice_addr addr;
};

struct slice_dispatchYUV_info {
	struct slice_img_size size;
};

struct slice_scaler_info {
	unsigned int trim0_size_x;
	unsigned int trim0_size_y;
	unsigned int trim0_start_x;
	unsigned int trim0_start_y;
	unsigned int trim1_size_x;
	unsigned int trim1_size_y;
	unsigned int trim1_start_x;
	unsigned int trim1_start_y;
	unsigned int scaler_ip_int;
	unsigned int scaler_ip_rmd;
	unsigned int scaler_cip_int;
	unsigned int scaler_cip_rmd;
	unsigned int scaler_factor_in;
	unsigned int scaler_factor_out;
	unsigned int scaler_ip_int_ver;
	unsigned int scaler_ip_rmd_ver;
	unsigned int scaler_cip_int_ver;
	unsigned int scaler_cip_rmd_ver;
	unsigned int scaler_factor_in_ver;
	unsigned int scaler_factor_out_ver;
	unsigned int src_size_x;
	unsigned int src_size_y;
	unsigned int dst_size_x;
	unsigned int dst_size_y;
	unsigned int scaler_in_width;
	unsigned int scaler_in_height;
	unsigned int scaler_out_width;
	unsigned int scaler_out_height;
	unsigned int chk_sum_clr;
};

struct slice_yuv_param {
	unsigned int id;
	unsigned int width;
	unsigned int height;
	unsigned int start_col;
	unsigned int start_row;
	unsigned int end_col;
	unsigned int end_row;
	unsigned int overlap_hor_left;
	unsigned int overlap_hor_right;
	unsigned int overlap_ver_up;
	unsigned int overlap_ver_down;
};

struct slice_postcnr_info {
	unsigned int start_row_mod4;
};

struct slice_ynr_info {
	unsigned int start_row;
	unsigned int start_col;
};

struct slice_noisefilter_info {
	unsigned int seed0;
	unsigned int seed1;
	unsigned int seed2;
	unsigned int seed3;
	unsigned int seed_int;
};

struct slice_context_info {
	struct slice_base_info base_info;
	struct slice_fetchYUV_info fetchYUV_info[SLICE_NUM_MAX];
	struct slice_store_info store_info[SLICE_PATH_MAX][SLICE_NUM_MAX];
	struct slice_dispatchYUV_info dispatchYUV_info[SLICE_NUM_MAX];
	struct slice_scaler_info scaler_info[SLICE_PATH_MAX][SLICE_NUM_MAX];
	struct slice_postcnr_info postcnr_info[SLICE_NUM_MAX];
	struct slice_ynr_info ynr_info[SLICE_NUM_MAX];
	struct slice_noisefilter_info noisefilter_info[SLICE_NUM_MAX];
};

struct slice_store_path {
	unsigned int format;
	struct slice_addr addr;
	struct slice_img_size size;
};

struct slice_scaler_path {
	unsigned int trim0_size_x;
	unsigned int trim0_size_y;
	unsigned int trim0_start_x;
	unsigned int trim0_start_y;
	unsigned int deci_x;
	unsigned int deci_y;
	unsigned int odata_mode;
	unsigned int scaler_bypass;
	unsigned int scaler_factor_in;
	unsigned int scaler_factor_out;
	unsigned int scaler_out_width;
	unsigned int scaler_out_height;
	unsigned int scaler_ver_factor_in;
	unsigned int scaler_ver_factor_out;
	unsigned int scaler_y_ver_tap;
	unsigned int scaler_uv_ver_tap;
};

struct slice_param_in {
	enum isp_id idx;
	unsigned int is_raw_capture;
	unsigned int fetchYUV_format;
	unsigned int pre_slice_need;
	unsigned int vid_slice_need;
	unsigned int cap_slice_need;
	unsigned int *fmcu_addr_vir;
	struct slice_addr fetchYUV_addr;
	struct slice_img_size img_size;
	struct slice_scaler_path scaler_frame[SLICE_PATH_MAX];
	struct slice_store_path store_frame[SLICE_PATH_MAX];
};

int isp_fmcu_slice_cfg(enum isp_id idx, void *fmcu_handler,
		       struct slice_param_in *in_ptr, unsigned int *fmcu_num);
int isp_fmcu_slice_init(void **fmcu_handler);
int isp_fmcu_slice_deinit(void *fmcu_handler);

#endif
