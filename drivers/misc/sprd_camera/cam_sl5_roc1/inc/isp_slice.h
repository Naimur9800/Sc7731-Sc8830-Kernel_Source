/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
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

#ifndef _ISP_SLICE_H_
#define _ISP_SLICE_H_


#define SLICE_NUM_MAX 4
#define SLICE_W_NUM_MAX 4
#define SLICE_H_NUM_MAX 1
#define SLICE_OVERLAP_W_MAX    64


#define RAW_OVERLAP_UP                 62
#define RAW_OVERLAP_DOWN               82
#define RAW_OVERLAP_LEFT               90
#define RAW_OVERLAP_RIGHT              142
#define YUV_OVERLAP_UP                 46
#define YUV_OVERLAP_DOWN               68
#define YUV_OVERLAP_LEFT               74
#define YUV_OVERLAP_RIGHT              126

#define YUVSCALER_OVERLAP_UP           32
#define YUVSCALER_OVERLAP_DOWN         52
#define YUVSCALER_OVERLAP_LEFT         16
#define YUVSCALER_OVERLAP_RIGHT        68


struct slice_scaler_info {
	uint32_t out_of_range;
	uint32_t scaler_bypass;
	uint32_t odata_mode;
	uint32_t trim0_size_x;
	uint32_t trim0_size_y;
	uint32_t trim0_start_x;
	uint32_t trim0_start_y;
	uint32_t trim1_size_x;
	uint32_t trim1_size_y;
	uint32_t trim1_start_x;
	uint32_t trim1_start_y;
	uint32_t scaler_ip_int;
	uint32_t scaler_ip_rmd;
	uint32_t scaler_cip_int;
	uint32_t scaler_cip_rmd;
	uint32_t scaler_factor_in;
	uint32_t scaler_factor_out;
	uint32_t scaler_ip_int_ver;
	uint32_t scaler_ip_rmd_ver;
	uint32_t scaler_cip_int_ver;
	uint32_t scaler_cip_rmd_ver;
	uint32_t scaler_factor_in_ver;
	uint32_t scaler_factor_out_ver;
	uint32_t scaler_in_width;
	uint32_t scaler_in_height;
	uint32_t scaler_out_width;
	uint32_t scaler_out_height;
	uint32_t src_size_x;
	uint32_t src_size_y;
	uint32_t dst_size_x;
	uint32_t dst_size_y;
	uint32_t chk_sum_clr;
};

struct slice_cfa_info {
	uint32_t gbuf_addr_max;
};

struct slice_pos_info {
	uint32_t start_col;
	uint32_t start_row;
	uint32_t end_col;
	uint32_t end_row;
};

struct slice_overlap_info {
	uint32_t overlap_up;
	uint32_t overlap_down;
	uint32_t overlap_left;
	uint32_t overlap_right;
};

struct slice_border_info {
	uint32_t up_border;
	uint32_t down_border;
	uint32_t left_border;
	uint32_t right_border;
};

struct slice_store_info {
	uint32_t store_bypass;
	struct img_size size;
	struct img_addr addr;
	struct img_pitch pitch;
	struct slice_border_info border;
};

struct slice_fetch_info {
	struct img_size size;
	struct img_addr addr;
	uint32_t mipi_byte_rel_pos;
	uint32_t mipi_word_num;
};

struct isp_slice_desc {
	uint32_t valid;
	uint32_t x;
	uint32_t y;
	uint32_t path_en[ISP_SPATH_NUM];
	/* original slice position without overlap*/
	struct slice_pos_info slice_pos_orig;
	struct slice_pos_info slice_pos;    /* slice position with overlap*/
	struct slice_overlap_info slice_overlap;
	struct slice_fetch_info slice_fetch;
	struct slice_store_info slice_store[ISP_SPATH_NUM];
	struct slice_scaler_info slice_scaler[ISP_SPATH_NUM];
	struct slice_cfa_info slice_cfa;
};

struct isp_slice_context {
	struct isp_slice_desc slices[SLICE_NUM_MAX];
	uint32_t slice_row_num;
	uint32_t slice_col_num;
	uint32_t slice_num;
	uint32_t slice_height;
	uint32_t slice_width;
	uint32_t img_width;
	uint32_t img_height;
	uint32_t overlap_up;
	uint32_t overlap_down;
	uint32_t overlap_left;
	uint32_t overlap_right;
};


int isp_cfg_slice_fetch_info(
			void *cfg_in, struct isp_slice_context *slc_ctx);

int isp_cfg_slice_store_info(
		void *cfg_in, struct isp_slice_context *slc_ctx);

int isp_cfg_slices(void *cfg_in, struct isp_slice_context *slc_ctx);

void *get_isp_slice_ctx(void);
int put_isp_slice_ctx(void **slc_ctx);

int isp_set_slices_fmcu_cmds(
		void *fmcu_handle,
		void *slc_handle,
		uint32_t ctx_idx,
		uint32_t wmode);

#endif
