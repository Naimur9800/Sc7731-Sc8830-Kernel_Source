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

#ifndef _ISP_CORE_H_
#define _ISP_CORE_H_

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sprd_ion.h>
#include <video/sprd_img.h>

#include "cam_types.h"
#include "cam_queue.h"

#define ISP_LINE_BUFFER_W      2560

#define ISP_IN_Q_LEN  1
#define ISP_PROC_Q_LEN  2
#define ISP_RESULT_Q_LEN 2
#define ISP_OUT_BUF_Q_LEN 16
#define ISP_RESERVE_BUF_Q_LEN 3


#define ODATA_YUV420   1
#define ODATA_YUV422   0

#define ISP_SC_COEFF_BUF_SIZE          (24 << 10)
#define ISP_SC_COEFF_COEF_SIZE         (1 << 10)
#define ISP_SC_COEFF_TMP_SIZE          (21 << 10)

#define ISP_SC_H_COEF_SIZE             0xC0
#define ISP_SC_V_COEF_SIZE             0x210
#define ISP_SC_V_CHROM_COEF_SIZE       0x210

#define ISP_SC_COEFF_H_NUM             (ISP_SC_H_COEF_SIZE / 6)
#define ISP_SC_COEFF_H_CHROMA_NUM      (ISP_SC_H_COEF_SIZE / 12)
#define ISP_SC_COEFF_V_NUM             (ISP_SC_V_COEF_SIZE / 4)
#define ISP_SC_COEFF_V_CHROMA_NUM      (ISP_SC_V_CHROM_COEF_SIZE / 4)

#define ISP_PIXEL_ALIGN_WIDTH          4
#define ISP_PIXEL_ALIGN_HEIGHT         2


enum isp_work_mode {
	ISP_CFG_MODE,
	ISP_AP_MODE,
	ISP_WM_MAX
};

enum isp_fetch_format {
	ISP_FETCH_YUV422_3FRAME = 0,
	ISP_FETCH_YUYV,
	ISP_FETCH_UYVY,
	ISP_FETCH_YVYU,
	ISP_FETCH_VYUY,
	ISP_FETCH_YUV422_2FRAME,
	ISP_FETCH_YVU422_2FRAME,
	ISP_FETCH_RAW10,
	ISP_FETCH_CSI2_RAW10,  /* MIPI RAW10*/
	ISP_FETCH_FULL_RGB10,
	ISP_FETCH_YUV420_2FRAME,
	ISP_FETCH_YVU420_2FRAME,
	ISP_FETCH_FORMAT_MAX
};

enum isp_store_format {
	ISP_STORE_UYVY = 0x00,
	ISP_STORE_YUV422_2FRAME,
	ISP_STORE_YVU422_2FRAME,
	ISP_STORE_YUV422_3FRAME,
	ISP_STORE_YUV420_2FRAME,
	ISP_STORE_YVU420_2FRAME,
	ISP_STORE_YUV420_3FRAME,
	ISP_STORE_FORMAT_MAX
};

enum isp_sub_path_id {
	ISP_SPATH_CP = 0,
	ISP_SPATH_VID,
	ISP_SPATH_FD,
	ISP_SPATH_NUM,
};

enum isp_context_id {
	ISP_CONTEXT_P0,
	ISP_CONTEXT_C0,
	ISP_CONTEXT_P1,
	ISP_CONTEXT_C1,
	ISP_CONTEXT_NUM
};

enum isp_scene_id {
	ISP_SCENE_PRE,
	ISP_SCENE_CAP,
	ISP_SCENE_NUM
};


enum isp_path_wk_status {
	PATH_STATUS_IDLE,
	PATH_STATUS_READY,
	PATH_STATUS_RUNNING,
};


struct isp_pipe_dev;
struct isp_pipe_context;

struct isp_fetch_info {
	enum isp_fetch_format fetch_fmt;
	struct img_size size;
	struct img_addr addr;
	struct img_addr trim_off;
	struct img_pitch pitch;
	uint32_t mipi_byte_rel_pos;
	uint32_t mipi_word_num;
};


struct isp_regular_info {
	uint32_t regular_mode;
	uint32_t shrink_uv_dn_th;
	uint32_t shrink_uv_up_th;
	uint32_t shrink_y_dn_th;
	uint32_t shrink_y_up_th;
	uint32_t effect_v_th;
	uint32_t effect_u_th;
	uint32_t effect_y_th;
	uint32_t shrink_c_range;
	uint32_t shrink_c_offset;
	uint32_t shrink_y_range;
	uint32_t shrink_y_offset;
};

struct isp_scaler_info {
	uint32_t scaler_bypass;
	uint32_t odata_mode;
	uint32_t scaler_y_ver_tap;
	uint32_t scaler_uv_ver_tap;
	uint32_t scaler_ip_int;
	uint32_t scaler_ip_rmd;
	uint32_t scaler_cip_int;
	uint32_t scaler_cip_rmd;
	uint32_t scaler_factor_in;
	uint32_t scaler_factor_out;
	uint32_t scaler_ver_ip_int;
	uint32_t scaler_ver_ip_rmd;
	uint32_t scaler_ver_cip_int;
	uint32_t scaler_ver_cip_rmd;
	uint32_t scaler_ver_factor_in;
	uint32_t scaler_ver_factor_out;
	uint32_t scaler_out_width;
	uint32_t scaler_out_height;
	uint32_t coeff_buf[ISP_SC_COEFF_BUF_SIZE];
};

struct isp_store_info {
	uint32_t bypass;
	uint32_t endian;
	uint32_t speed_2x;
	uint32_t mirror_en;
	uint32_t max_len_sel;
	uint32_t shadow_clr;
	uint32_t store_res;
	uint32_t rd_ctrl;
	uint32_t shadow_clr_sel;
	enum isp_store_format color_fmt; /* output color format */
	struct img_size size;
	struct img_addr addr;
	struct img_pitch pitch;
};


struct slice_cfg_input {
	struct img_size frame_in_size;
	struct img_size *frame_out_size[ISP_SPATH_NUM];
	struct isp_fetch_info *frame_fetch;
	struct isp_store_info *frame_store[ISP_SPATH_NUM];
	struct isp_scaler_info *frame_scaler[ISP_SPATH_NUM];
	struct img_deci_info *frame_deci[ISP_SPATH_NUM];
	struct img_trim *frame_trim0[ISP_SPATH_NUM];
	struct img_trim *frame_trim1[ISP_SPATH_NUM];
};

/*
struct isp_offline_thead_info {
	atomic_t thread_stop;
	void *ctx_handle;
	int (*proc_func)(void *param);
	struct completion thread_com;
	struct task_struct *thread_task;
};
*/

struct isp_path_desc {
	atomic_t user_cnt;
	atomic_t store_cnt;
	enum isp_sub_path_id spath_id;
	uint32_t base_frm_id;
	uint32_t frm_cnt;
	uint32_t skip_pipeline;
	uint32_t uv_sync_v;
	uint32_t frm_deci;
	uint32_t out_fmt;  /* forcc */
	uint32_t shared; /* can be shared in same context. */
	struct isp_pipe_context *attach_ctx;

	struct isp_regular_info regular_info;
	struct isp_store_info store;
	struct isp_scaler_info scaler;

	struct img_deci_info deci;
	struct img_trim in_trim;
	struct img_trim out_trim;
	struct img_size src;
	struct img_size dst;
	struct img_endian data_endian;
	struct camera_queue reserved_buf_queue;
	/*struct camera_frame *reserved_out_buf;*/
	struct camera_queue out_buf_queue;
	struct camera_queue result_queue;
};


struct isp_pipe_context {
	atomic_t user_cnt;
	uint32_t ctx_id;
	uint32_t in_fmt; /* forcc */
	uint32_t shared; /* can be shared in same context for multi paths */
	enum camera_id attach_cam_id;

	uint32_t updated;
	uint32_t en_3dnr;
	uint32_t dispatch_color;
	uint32_t dispatch_bayer_mode; /* RAWRGB_GR, RAWRGB_Gb, RAWRGB_R...*/
	uint32_t fetch_path_sel; /* fbd or normal */
	struct mutex ctx_mutex; /* lock path param(size) updated from zoom*/

	struct img_size input_size;
	struct img_trim input_trim;
	struct isp_fetch_info fetch;
	struct isp_path_desc isp_path[ISP_SPATH_NUM];
	struct isp_pipe_dev *dev;
	void *slice_ctx;
	void *fmcu_handle;
	struct camera_queue in_queue;
	struct camera_queue proc_queue;
	struct cam_offline_thread_info thread;
	struct completion shadow_com;
	struct completion fmcu_com;

	isp_dev_callback isp_cb_func;
	void *cb_priv_data;
};


struct isp_pipe_dev {
	uint32_t irq_no[2];
	atomic_t user_cnt;
	atomic_t enable;
	struct mutex path_mutex;  /* lock path resource management */
	enum isp_work_mode wmode;
	void *cfg_handle;
	struct isp_pipe_context ctx[ISP_CONTEXT_NUM];
	struct sprd_cam_hw_info *isp_hw;
};
#endif
