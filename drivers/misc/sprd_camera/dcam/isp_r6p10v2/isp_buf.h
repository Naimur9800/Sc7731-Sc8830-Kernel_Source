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

#ifndef _ISP_BUF_HEADER_
#define _ISP_BUF_HEADER_

#include "isp_drv.h"

extern struct platform_device *s_isp_pdev;
extern struct platform_device *s_isp1_pdev;
extern spinlock_t isp_mod_lock[ISP_MAX_COUNT];

int isp_irq_done_init(enum isp_id isp_idx);
int isp_irq_done_read(enum isp_id isp_idx, enum isp_irq_done_id idx);
int isp_irq_done_write(enum isp_id isp_idx, enum isp_irq_done_id idx);

int isp_statis_queue_init(struct isp_statis_buf_queue *queue);
int isp_statis_queue_read(struct isp_statis_buf_queue *queue,
	struct isp_statis_buf *buf);
void isp_statis_frm_queue_clear(struct isp_statis_frm_queue *queue);
void isp_statis_reset_all_queue(struct isp_pipe_dev *dev);
int isp_statis_enqueue(struct isp_statis_frm_queue *queue,
	struct isp_statis_buf *frame);
int isp_statis_dequeue(struct isp_statis_frm_queue *queue,
	struct isp_statis_buf *frame);
int sprd_isp_set_statis_addr(struct isp_pipe_dev *dev,
	struct isp_statis_buf_input *parm);
int sprd_isp_cfg_statis_buf(struct isp_pipe_dev *dev,
	struct isp_statis_buf_input *parm);
void isp_buf_queue_init(struct isp_buf_queue *queue);
int isp_frame_enqueue(struct isp_frm_queue *queue,
	struct camera_frame *frame);
int isp_frame_dequeue(struct isp_frm_queue *queue,
	struct camera_frame *frame);
int isp_frame_dequeue_withtime(struct isp_frm_queue *queue,
	struct camera_frame *frame,  int64_t  timestamp);

int isp_queue_init(struct isp_queue *queue);
int32_t isp_queue_read(struct isp_queue *queue, struct isp_node *node);
int isp_buf_queue_read(struct isp_buf_queue *queue,
	struct camera_frame *frame);
int isp_buf_queue_write(struct isp_buf_queue *queue,
	struct camera_frame *frame);
void isp_frm_clear(struct isp_pipe_dev *dev, enum isp_path_index path_index);
int isp_fmcu_get_buf(struct isp_fmcu_slice_desc *fmcu, enum isp_id idx);
int isp_fmcu_clear_buf(struct isp_fmcu_slice_desc *fmcu, enum isp_id idx);
int isp_fmcu_iommu_map(struct isp_fmcu_slice_desc *fmcu, enum isp_id idx);
int isp_fmcu_iommu_unmap(struct isp_fmcu_slice_desc *fmcu, enum isp_id idx);
int isp_storecce_get_buf(struct isp_store_cce_desc *store_cce,
	enum isp_id idx);
int isp_storecce_clear_buf(struct isp_store_cce_desc *store_cce);
int isp_storecce_buf_iommu_map(struct isp_store_cce_desc *store_cce,
	enum isp_id idx);
int isp_storecce_buf_iommu_unmap(struct isp_store_cce_desc *store_cce,
	enum isp_id idx);
void isp_frm_queue_clear(struct isp_frm_queue *queue);
void isp_coeff_queue_init(struct isp_sc_coeff_queue *queue);
int isp_coeff_get_new_node(struct isp_sc_coeff_queue *queue,
			   struct isp_sc_coeff **coeff, int type);
int isp_coeff_get_valid_node(struct isp_sc_coeff_queue *queue,
			     struct isp_sc_coeff **coeff, int type);
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT

int isp_cpp_get_buf(struct isp_cpp_desc *isp_cpp_dev,
		int idx);
int isp_ion_cpp_buf_iommu_unmap(
	struct device *dev, struct camera_frame  *frame,
	unsigned int ch_id);

int isp_ion_cpp_buf_iommu_map(
	struct device *dev, struct camera_frame  *frame);

int isp_cpp_clear_buf(int idx, struct isp_cpp_desc *isp_cpp_dev);
#endif
#endif

