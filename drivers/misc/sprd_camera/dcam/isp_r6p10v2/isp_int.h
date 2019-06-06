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

#ifndef _ISP_INT_HEADER_
#define _ISP_INT_HEADER_

#include <linux/platform_device.h>

#include "isp_drv.h"

extern struct isp_ch_irq s_isp_irq[ISP_MAX_COUNT];

enum isp_irq0_id {
	ISP_INT_ISP_ALL_DONE,
	ISP_INT_SHADOW_DONE,
	ISP_INT_DISPATCH_SLICE_DONE,
	ISP_INT_ISP_START,
	ISP_INT_FETCH_RAW_DONE,
	ISP_INT_DISPATCH_YUV_SLICE_DONE,
	ISP_INT_ISP2DCAM_AFIFO_FULL,
	ISP_INT_LENS_LOAD,
	ISP_INT_AEM_START,
	ISP_INT_AEM_SHADOW_DONE,
	ISP_INT_AEM_DONE,
	ISP_INT_AEM_ERROR,
	ISP_INT_AFL_START,
	ISP_INT_AFL_SHADOW_DONE,
	ISP_INT_AFL_DONE,
	ISP_INT_AFL_ERROR,
	ISP_INT_BINNING_START,
	ISP_INT_BINNING_SHADOW_DONE,
	ISP_INT_BINNING_DONE,
	ISP_INT_BINNING_ERROR,
	ISP_INT_PDAF_DDR_START,
	ISP_INT_PDAF_MEM_LEFT_DONE,
	ISP_INT_PDAF_MEM_LEFT_NOT_EMPTY_ERR,
	ISP_INT_PDAF_MEM_LEFT_STORE_ERR,
	ISP_INT_PDAF_MEM_RIGHT_DONE,
	ISP_INT_PDAF_MEM_RIGHT_NOT_EMPTY_ERR,
	ISP_INT_PDAF_MEM_RIGHT_STORE_ERR,
	ISP_INT_PDAF_SHADOW_DONE,
	ISP_INT_DCAM_SOF,
	ISP_INT_DCAM_EOF,
	ISP_INT_STORE_DONE,
	ISP_INT_AFL_LAST_SOF,
	ISP_INT_NUMBER0,
};

enum isp_irq1_id {
	ISP_INT_STORE_DONE_OUT,
	ISP_INT_SHADOW_DONE_OUT,
	ISP_INT_STORE_DONE_CAP,
	ISP_INT_SHADOW_DONE_CAP,
	ISP_INT_STORE_DONE_VID,
	ISP_INT_SHADOW_DONE_VID,
	ISP_INT_STORE_DONE_PRE,
	ISP_INT_SHADOW_DONE_PRE,
	ISP_INT_STORE_DONE_CCE,
	ISP_INT_SHADOW_DONE_CCE,
	ISP_INT_NULL10,
	ISP_INT_NULL11,
	ISP_INT_NULL12,
	ISP_INT_NULL13,
	ISP_INT_NULL14,
	ISP_INT_NULL15,
	ISP_INT_NULL16,
	ISP_INT_NULL17,
	ISP_INT_HIST_SHADOW_DONE,
	ISP_INT_HIST2_SHADOW_DONE,
	ISP_INT_HIST_START,
	ISP_INT_HIST_DONE,
	ISP_INT_HIST2_START,
	ISP_INT_HIST2_DONE,
	ISP_INT_BPC_STORE_MEM_DONE,
	ISP_INT_BPC_STORE_ERROR,
	ISP_INT_BPC_ERR2,
	ISP_INT_BPC_ERR1,
	ISP_INT_BPC_ERR0,
	ISP_INT_BPC_STORE_NOT_EMPTY_ERROR,
	ISP_INT_JPG_LINE_DONE,
	ISP_INT_JPG_FRAME_DONE,
	ISP_INT_NUMBER1,
};

enum isp_irq2_id {
	ISP_INT_AFM_RGB_Win0,
	ISP_INT_AFM_RGB_Win1,
	ISP_INT_AFM_RGB_Win2,
	ISP_INT_AFM_RGB_Win3,
	ISP_INT_AFM_RGB_Win4,
	ISP_INT_AFM_RGB_Win5,
	ISP_INT_AFM_RGB_Win6,
	ISP_INT_AFM_RGB_Win7,
	ISP_INT_AFM_RGB_Win8,
	ISP_INT_AFM_RGB_Win9,
	ISP_INT_AFM_RGB_START,
	ISP_INT_AFM_RGB_DONE,
	ISP_INT_AFM_RGB_SHADOW_DONE,
	ISP_INT_NR3_FAST_ME_DONE,
	ISP_INT_NR3_DONE,
	ISP_INT_NR3_SHADOW_DONE,
	ISP_INT_NULL23,
	ISP_INT_NULL24,
	ISP_INT_NULL25,
	ISP_INT_NULL26,
	ISP_INT_NULL27,
	ISP_INT_FMCU_LOAD_DONE,
	ISP_INT_FMCU_CONFIG_DONE,
	ISP_INT_FMCU_SHADOW_DONE,
	ISP_INT_FMCU_CMD_X,
	ISP_INT_FMCU_TIMEOUT,
	ISP_INT_FMCU_CMD_ERROR,
	ISP_INT_FMCU_STOP_DONE,
	ISP_INT_NUMBER2,
};

enum isp_irq3_id {
	ISP_INT_YUV_DONE,
	ISP_INT_YUV_START,
	ISP_INT_FRGB_DONE,
	ISP_INT_FRGB_START,
	ISP_INT_RRGB_DONE,
	ISP_INT_RRGB_START,
	ISP_INT_NUMBER3,
};

int isp_irq_callback(enum isp_id idx, enum isp_irq_id irq_id,
	isp_isr_func user_func, void *user_data);
int isp_irq_request(struct device *p_dev, struct isp_ch_irq *irq,
	struct isp_pipe_dev *ispdev);
int isp_irq_free(struct isp_ch_irq *irq, struct isp_pipe_dev *ispdev);
int isp_set_next_statis_buf(enum isp_id idx, struct isp_statis_module *module,
			    enum isp_3a_block_id block_index);
int isp_storecce_buf_reset(struct isp_store_cce_desc *store_cce);

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
void isp_path_done_set_cpp_param(struct isp_module *module);
void isp_shadow_done_set_path_cpp(struct isp_module *module);
void isp_handle_cpp_done(struct isp_pipe_dev *isp_dev,
	int path_id, struct camera_frame *frame);

#endif
#endif
