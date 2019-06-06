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

#ifndef _ISP_SLW_HEADER_
#define _ISP_SLW_HEADER_

#include "isp_drv.h"

#define ISP_SLW_FRM_NUM			4
#define ISP_SLW_BUF_NUM			4
#define FMCU_ALIGN			8

enum isp_slw_rtn {
	ISP_RTN_SLW_SUCCESS = 0,
	ISP_RTN_SLW_DQ_ERR,
	ISP_RTN_SLW_EQ_ERR,
	ISP_RTN_SLW_FRM_ERR,
};

struct isp_int_info {
	unsigned int sdw_done_skip_en;
	unsigned int sdw_done_int_cnt_num;
	unsigned int sdw_done_skip_cnt_num;
	unsigned int sdw_done_skip_cnt_clr;
	unsigned int vid_done_skip_en;
	unsigned int vid_done_int_cnt_num;
	unsigned int vid_done_skip_cnt_num;
	unsigned int vid_done_skip_cnt_clr;
};

struct isp_fmcu_cmd_frm {
	unsigned int yaddr_frm;
	unsigned int yaddr_reg;
	unsigned int uaddr_frm;
	unsigned int uaddr_reg;
	unsigned int vaddr_frm;
	unsigned int vaddr_reg;
	unsigned int shadow_frm;
	unsigned int shadow_reg;
};

struct isp_slw_cmd {
	struct isp_fmcu_cmd_frm cmd_frm[ISP_SLW_FRM_NUM];
};

struct isp_slw_info {
	unsigned int *fmcu_addr_vir;
	unsigned int fmcu_addr_phy;
	unsigned int fmcu_num;
	unsigned int is_reserved;
	struct isp_frm_queue slw_queue;
};

struct isp_slw_queue {
	struct isp_slw_info slw_array[ISP_SLW_BUF_NUM];
	unsigned int valid_cnt;
};

struct isp_fmcu_slw_info {
	struct isp_slw_queue empty_queue;
	struct isp_slw_queue embed_queue;
	struct isp_slw_queue insert_queue;
	struct isp_slw_info slw_reserved;
	struct isp_int_info sdw_done_info;
	struct isp_slw_info slw_info;
	struct isp_slw_info p_from_embed;
	struct isp_slw_info p_from_queue;
	struct isp_slw_info p_from_empty;

};

int slowmotion_frame_enqueue(struct isp_slw_queue *queue,
			      struct isp_slw_info *slw);
int slowmotion_frame_dequeue(struct isp_slw_queue *queue,
			      struct isp_slw_info *slw);
int set_isp_fmcu_cmd_reg(enum isp_scl_id path_id, void *isp_handle);
int set_isp_fmcu_slw_cmd(void *isp_handle,
		enum isp_scl_id path_id, unsigned int buf_reserved);
int set_fmcu_slw_cfg(void *handle);
int get_slw_status(void *isp_handle);
void isp_slw_clear(void *handle);
int isp_slw_flags_init(void *isp_handle, struct isp_path_info *info);
int isp_fmcu_slw_start(enum isp_scl_id path_id, void *isp_handle);
int isp_fmcu_slw_init(void **fmcu_handler);
int isp_fmcu_slw_deinit(void *fmcu_handler);
#endif
