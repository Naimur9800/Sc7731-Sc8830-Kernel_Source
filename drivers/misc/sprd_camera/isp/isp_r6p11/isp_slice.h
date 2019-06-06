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
#include "isp_cfg.h"

#define SLICE_NUM_MAX	3

enum slice_path_index {
	SLICE_PATH_PRE,
	SLICE_PATH_VID,
	SLICE_PATH_CAP,
	SLICE_PATH_MAX,
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
	unsigned int overlap_up;
	unsigned int overlap_down;
	unsigned int overlap_left;
	unsigned int overlap_right;
	unsigned int isp_jpg_cowork;
};

struct slice_lsc_2d_info {
	uint32_t start_col;
	uint32_t start_row;
	uint32_t relative_x;
	uint32_t relative_y;
	uint32_t slice_width;
	uint32_t slice_height;
	uint32_t grid_x_num;
	uint32_t grid_y_num;
	uint16_t q_val[5][2];
	uint32_t grid_num_t;
	uint16_t *grid_buf;
};

struct slice_fetch_info {
	struct slice_img_size size;
	struct slice_addr addr;
	uint32_t mipi_word_num;
	uint32_t mipi_byte_rel_pos;
};

struct slice_store_info {
	struct slice_img_size size;
	struct slice_border border;
	struct slice_addr addr;
};

struct slice_dispatch_info {
	struct slice_img_size size;
	uint32_t bayer_mode;
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

struct slice_cfa_info {
	unsigned int gbuf_addr_max;
};

struct slice_context_info {
	struct slice_base_info base_info;
	struct slice_lsc_2d_info lsc_2d_info[SLICE_NUM_MAX];
	struct slice_fetch_info fetch_info[SLICE_NUM_MAX];
	struct slice_store_info store_info[SLICE_PATH_MAX][SLICE_NUM_MAX];
	struct slice_dispatch_info dispatch_info[SLICE_NUM_MAX];
	struct slice_scaler_info scaler_info[SLICE_PATH_MAX][SLICE_NUM_MAX];
	struct slice_postcnr_info postcnr_info[SLICE_NUM_MAX];
	struct slice_ynr_info ynr_info[SLICE_NUM_MAX];
	struct slice_noisefilter_info noisefilter_info[SLICE_NUM_MAX];
	struct slice_cfa_info cfa_info[SLICE_NUM_MAX];
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
	enum isp_id iid;
	enum isp_scene_id sid;
	enum isp_work_mode mid;
	unsigned int is_raw_capture;
	unsigned int fetch_format;
	unsigned int bayer_mode;
	unsigned int pre_slice_need;
	unsigned int vid_slice_need;
	unsigned int cap_slice_need;
	unsigned int *fmcu_addr_vir;
	struct isp_pipe_dev *isp_dev;
	struct slice_addr fetch_addr;
	struct slice_img_size img_size;
	struct slice_scaler_path scaler_frame[SLICE_PATH_MAX];
	struct slice_store_path store_frame[SLICE_PATH_MAX];
};

int isp_fmcu_slice_cfg(void *fmcu_handler,
		       struct slice_param_in *in_ptr, unsigned int *fmcu_num);
int isp_fmcu_slice_init(void **fmcu_handler);
int isp_fmcu_slice_deinit(void *fmcu_handler);

#endif
