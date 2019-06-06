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

#ifndef _DCAM_CORE_H_
#define _DCAM_CORE_H_

#include <linux/of.h>
#include <linux/platform_device.h>
#include <video/sprd_img.h>
#include <linux/sprd_ion.h>

#include "cam_types.h"
#include "cam_queue.h"

#define DCAM_IN_Q_LEN  1
#define DCAM_PROC_Q_LEN  2

#define DCAM_RESULT_Q_LEN 3
#define DCAM_OUT_BUF_Q_LEN 16
#define DCAM_RESERVE_BUF_Q_LEN 3

#define DCAM_INTERNAL_RES_BUF_SIZE  0x40000

enum dcam_path_id {
	DCAM_PATH_FULL = 0,
	DCAM_PATH_BIN,
	DCAM_PATH_PDAF,
	DCAM_PATH_VCH2,
	DCAM_PATH_VCH3,
	DCAM_PATH_AEM,
	DCAM_PATH_AFM,
	DCAM_PATH_AFL,
	DCAM_PATH_HIST,
	DCAM_PATH_3DNR,
	DCAM_PATH_BPC,
	DCAM_PATH_MAX,
};

enum dcam_scaler_type {
	DCAM_SCALER_BINNING = 0,
	DCAM_SCALER_RAW_DOWNSISER,
	DCAM_SCALER_BYPASS,
	DCAM_SCALER_MAX,
};





struct statis_path_buf_info {
	enum dcam_path_id path_id;
	size_t buf_size;
	size_t buf_cnt;
};

struct dcam_mipi_info {
	uint32_t sensor_if;  /* MIPI CSI-2 */
	uint32_t format;  /* input color format */
	uint32_t mode;   /* single or multi mode. */
	uint32_t data_bits;
	uint32_t pattern; /* bayer mode for rgb, yuv pattern for yuv */
	uint32_t href;
	uint32_t frm_deci;
	uint32_t frm_skip;
	uint32_t x_factor;
	uint32_t y_factor;
	uint32_t is_4in1;
	struct img_trim cap_size;
};

struct dcam_fetch_info {
	uint32_t is_loose;
	uint32_t endian;
	uint32_t pattern;
	struct img_size size;
	struct img_trim trim;
	struct img_addr addr;
};

struct dcam_path_desc {
	atomic_t user_cnt;
	struct mutex param_mutex;
	enum dcam_path_id path_id;
	uint32_t updated;

	struct img_endian endian;
	struct img_size in_size;
	struct img_trim in_trim;
	struct img_size out_size;

	uint32_t out_fmt;
	uint32_t is_loose;

	/* full path source sel */
	uint32_t src_sel;

	/* for bin path */
	uint32_t is_slw;
	uint32_t slw_frm_num;
	uint32_t bin_ratio;
	uint32_t scaler_sel; /* 0: bining, 1: RDS, 2&3: bypass */
	void *rds_coeff_buf;
	uint32_t rds_coeff_size;

	uint32_t frm_deci;
	uint32_t frm_deci_cnt;

	uint32_t frm_skip;
	uint32_t frm_cnt;

	atomic_t set_frm_cnt;
	struct camera_queue reserved_buf_queue;
	struct camera_queue out_buf_queue;
	struct camera_queue result_queue;
};


struct dcam_pipe_dev {
	int irq_no;
	uint32_t idx;
	uint32_t err_status;
	atomic_t enable;
	atomic_t started;

	uint32_t is_4in1;
	uint32_t is_3dnr;
	uint32_t is_pdaf;
	uint32_t pdaf_type;

	uint32_t iommu_enable;
	struct dcam_mipi_info cap_info;

	struct camera_buf *statis_buf;
	void *internal_reserved_buf; /* for statis path output */

	dcam_dev_callback dcam_cb_func;
	void *cb_priv_data;

	struct dcam_path_desc path[DCAM_PATH_MAX];

	struct dcam_fetch_info fetch;
	struct camera_queue in_queue;
	struct camera_queue proc_queue;
	struct cam_offline_thread_info thread;

	struct sprd_cam_hw_info *dcam_hw;
};

#endif
