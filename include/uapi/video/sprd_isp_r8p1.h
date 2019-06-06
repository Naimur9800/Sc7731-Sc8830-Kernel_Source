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

#ifndef _SPRD_ISP_R8P1_H_
#define _SPRD_ISP_R8P1_H_

#define BUF_ALIGN(w) ((((w) + 512 + 1024 - 1) >> 10) << 10)

#define ISP_PDAF_STATIS_BUF_SIZE                BUF_ALIGN(0x43600)

#define PDAF_PPI_NUM                            64


#define ISP_MAX_WIDTH  4672
#define ISP_MAX_HEIGHT  3504


#define ISP_LENS_BUF_SIZE  0x1000
#define ISP_AEM_BUF_SIZE   0x1000
#define ISP_AFM_BUF_SIZE   0x1000
#define ISP_AFL_BUF_SIZE    0x1000
#define ISP_PDAF_BUF_SIZE  0x1000
#define ISP_HIST_BUF_SIZE   0x1000
#define ISP_3DNR_BUF_SIZE  0x1000
#define ISP_STATIS_BUF_CNT  5


enum isp_irq_done_id {
	IRQ_DCAM_SOF,
	IRQ_RAW_CAP_DONE,
	IRQ_AEM_STATIS,
	IRQ_AFL_STATIS,
	IRQ_AFM_STATIS,
	IRQ_PDAF_STATIS,
	IRQ_MAX_DONE,
};

enum isp_statis_buf_type {
	STATIS_INIT = 0,
	STATIS_AEM,
	STATIS_AFM,
	STATIS_AFL,
	STATIS_HIST,
	STATIS_PDAF,
};


struct isp_statis_buf_input {
	enum isp_statis_buf_type type;
	union {
		struct init {
			int mfd;
			uint32_t  buf_size;
		} init_data;
		struct block {
			uint32_t hw_addr[2];
		} block_data;
	} u;
	uint32_t vir_addr[2];
	uint32_t kaddr[2];
};




struct isp_io_param {
	uint32_t isp_id;
	uint32_t scene_id;
	uint32_t sub_block;
	uint32_t property;
	uint32_t param_uaddr[2];
};

struct isp_addr {
	unsigned long	chn0;
	unsigned long	chn1;
	unsigned long	chn2;
};

struct isp_img_size {
	uint32_t width;
	uint32_t height;
};

struct isp_coord {
	uint32_t start_x;
	uint32_t start_y;
	uint32_t end_x;
	uint32_t end_y;
};


/*anti	flicker */
struct isp_dev_anti_flicker_new_info {
	unsigned int bypass;
	unsigned int mode;
	unsigned int skip_frame_num;
	unsigned int afl_stepx;
	unsigned int afl_stepy;
	unsigned int frame_num;
	unsigned int start_col;
	unsigned int end_col;
	unsigned int mem_init_addr;
	unsigned int step_x_region;
	unsigned int step_y_region;
	unsigned int step_x_start_region;
	unsigned int step_x_end_region;
	unsigned int mem_init_addr_region;
	unsigned int skip_num_clr;
	unsigned int afl_glb_total_num;
	unsigned int afl_region_total_num;
	struct isp_img_size img_size;
};

struct isp_dev_anti_flicker_info {
	unsigned int bypass;
	unsigned int mode;
	unsigned int skip_frame_num;
	unsigned int line_step;
	unsigned int frame_num;
	unsigned int vheight;
	unsigned int start_col;
	unsigned int end_col;
	unsigned int afl_total_num;
	struct isp_img_size img_size;
};


struct isp_rrgb {
	uint32_t r;
	uint32_t b;
	uint32_t gr;
	uint32_t gb;
};


struct isp_dev_pdaf_info {
	uint32_t bypass;
	uint32_t corrector_bypass;
	uint32_t phase_map_corr_en;
	struct isp_img_size block_size;
	uint32_t grid_mode;
	struct isp_coord win;
	struct isp_coord block;
	struct isp_rrgb gain_upperbound;
	uint32_t phase_txt_smooth;
	uint32_t phase_gfilter;
	uint32_t phase_flat_smoother;
	uint32_t hot_pixel_th[3];
	uint32_t dead_pixel_th[3];
	uint32_t flat_th;
	uint32_t edge_ratio_hv;
	uint32_t edge_ratio_rd;
	uint32_t edge_ratio_hv_rd;
	uint32_t phase_left_addr;
	uint32_t phase_right_addr;
	uint32_t phase_pitch;
	uint32_t pattern_pixel_is_right[PDAF_PPI_NUM];
	uint32_t pattern_pixel_row[PDAF_PPI_NUM];
	uint32_t pattern_pixel_col[PDAF_PPI_NUM];
	uint32_t gain_ori_left[2];
	uint32_t gain_ori_right[2];
	uint32_t extractor_bypass;
	uint32_t mode_sel;
	uint32_t skip_num;
	uint32_t phase_data_dword_num;
	struct isp_rrgb pdaf_blc;
	uint32_t data_ptr_left[2];
	uint32_t data_ptr_right[2];
};

struct pdaf_ppi_info {
	struct isp_img_size block_size;
	struct isp_coord block;
	unsigned int pattern_pixel_is_right[PDAF_PPI_NUM];
	unsigned int pattern_pixel_row[PDAF_PPI_NUM];
	unsigned int pattern_pixel_col[PDAF_PPI_NUM];
};

struct pdaf_roi_info {
	struct isp_coord win;
	unsigned int phase_data_write_num;
};



enum SCINFO_COLOR_ORDER {
	COLOR_ORDER_RG = 0,
	COLOR_ORDER_GR,
	COLOR_ORDER_GB,
	COLOR_ORDER_BG
};


struct isp_raw_proc_info {
	struct isp_img_size in_size;
	struct isp_img_size out_size;
	uint32_t img_fd;
};

/************  for test only below ************** */
enum ch_property {
	PROP_PRE,
	PROP_CAP,
	PROP_VIDEO,
	PROP_FD,
	PROP_MAX
};

struct dev_test_info {
	uint32_t  dev; /* 0: isp, 1: dcam0, 2: dcam1 */

	/* channel desc  */
	uint32_t in_fmt;  /* forcc */
	enum ch_property prop;
	struct isp_img_size input_size;
	struct isp_img_size output_size;

	/* buffer desc */
	uint32_t iommu_en;
	uint32_t inbuf_fd;
	uint32_t inbuf_kaddr[2];
	uint32_t outbuf_fd;
	uint32_t outbuf_kaddr[2];
};

#endif
