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

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <video/sprd_mm.h>

#include "cam_hw.h"
#include "cam_types.h"
#include "cam_queue.h"
#include "cam_buf.h"

#include "isp_reg.h"
#include "isp_int.h"
#include "isp_core.h"
#include "isp_interface.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_INT: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__



typedef void(*isp_isr)(enum isp_context_id idx, void *param);


static const uint32_t isp_irq_process[] = {
	ISP_INT_ISP_ALL_DONE,
	ISP_INT_SHADOW_DONE,
	ISP_INT_DISPATCH_DONE,
	ISP_INT_STORE_DONE_PRE,
	ISP_INT_STORE_DONE_VID,
	ISP_INT_FMCU_LOAD_DONE,
	ISP_INT_FMCU_STORE_DONE,
};


static int irq_done[4][32];


static void isp_all_done(enum isp_context_id idx, void *isp_handle)
{
	int i;
	struct isp_pipe_context *pctx;
	struct isp_pipe_dev *dev;
	struct camera_frame *pframe;
	struct isp_path_desc *path;
	struct timespec cur_ts;
	ktime_t boot_time;

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[idx];

	/* pop bufferq in dispatch_done & strore_done */
	return;

	boot_time = ktime_get_boottime();
	ktime_get_ts(&cur_ts);

	/* return buffer to cam channel shared buffer queue. */
	pframe = camera_dequeue(&pctx->proc_queue);
	cambuf_iommu_unmap(&pframe->buf);

	pr_debug("ctx %d, ret src buf: %p,  priv %p\n",
			pctx->ctx_id, pframe,  pctx->cb_priv_data);

	pctx->isp_cb_func(ISP_CB_RET_SRC_BUF, pframe, pctx->cb_priv_data);

	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		if (atomic_read(&path->user_cnt) <= 0)
			continue;

		pframe = camera_dequeue(&path->result_queue);
		if (!pframe) {
			pr_err("error: no frame from queue. cxt:%d, path:%d\n",
						pctx->ctx_id, i);
			continue;
		}
		pr_debug("ctx %d path %d, ret out buf: %p,  priv %p\n",
			pctx->ctx_id, path->spath_id,
			pframe,  pctx->cb_priv_data);

		path->frm_cnt++;
		if (unlikely(pframe->is_reserved)) {
			camera_enqueue(&path->reserved_buf_queue, pframe);
		} else {
			pframe->time.tv_sec = cur_ts.tv_sec;
			pframe->time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;
			pframe->boot_time = boot_time;
			pframe->fid = path->base_frm_id + path->frm_cnt;
			cambuf_iommu_unmap(&pframe->buf);
			pctx->isp_cb_func(ISP_CB_RET_DST_BUF,
						pframe, pctx->cb_priv_data);
		}
	}
	pr_debug("all don. cxt_id:%d\n", idx);
}


static int isp_err_pre_proc(enum isp_context_id idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_context *pctx;

	pr_err("isp cxt_id:%d error happened\n", idx);
	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[idx];

	/*pctx->isp_cb_func(ISP_CB_DEV_ERR, dev, pctx->cb_priv_data);*/
	return 0;
}

static void isp_shadow_done(enum isp_context_id idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_context *pctx;

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[idx];

	complete(&pctx->shadow_com);
	pr_debug("cxt_id:%d shadow done.\n", idx);
}


static void isp_dispatch_done(enum isp_context_id idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_context *pctx;
	struct camera_frame *pframe;

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[idx];

	if (pctx->fmcu_handle) {
		pr_err("fmcu started. skip dispatch done.\n ");
		return;
	}
	/* return buffer to cam channel shared buffer queue. */
	pframe = camera_dequeue(&pctx->proc_queue);
	cambuf_iommu_unmap(&pframe->buf);

	pr_debug("ctx %d, ret src buf: %p,  priv %p\n",
			pctx->ctx_id, pframe,  pctx->cb_priv_data);

	pctx->isp_cb_func(ISP_CB_RET_SRC_BUF, pframe, pctx->cb_priv_data);

	pr_debug("cxt_id:%d dispatch done.\n", pctx->ctx_id);
}


static void isp_path_store_done(
		struct isp_pipe_dev *dev,
		struct isp_pipe_context *pctx,
		struct isp_path_desc *path)
{
	struct camera_frame *pframe;
	struct timespec cur_ts;

	if (atomic_read(&path->user_cnt) <= 0) {
		pr_err("path %p not in use.\n", path);
		return;
	}

	pframe = camera_dequeue(&path->result_queue);
	if (!pframe) {
		pr_err("error: no frame from queue. cxt:%d, path:%d\n",
					pctx->ctx_id, path->spath_id);
		return;
	}
	atomic_dec(&path->store_cnt);
	pr_debug("path %d store_cnt %d\n",
			path->spath_id, atomic_read(&path->store_cnt));

	pr_debug("ctx %d path %d, ret out buf: %p,  priv %p\n",
		pctx->ctx_id, path->spath_id,
		pframe,  pctx->cb_priv_data);

	path->frm_cnt++;
	if (unlikely(pframe->is_reserved)) {
		camera_enqueue(&path->reserved_buf_queue, pframe);
	} else {
		ktime_get_ts(&cur_ts);
		pframe->boot_time = ktime_get_boottime();
		pframe->time.tv_sec = cur_ts.tv_sec;
		pframe->time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;

		pframe->fid = path->base_frm_id + path->frm_cnt;
		cambuf_iommu_unmap(&pframe->buf);
		pctx->isp_cb_func(ISP_CB_RET_DST_BUF,
					pframe, pctx->cb_priv_data);
	}
}

static void isp_pre_store_done(enum isp_context_id idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_context *pctx;
	struct isp_path_desc *path;

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[idx];

	if (pctx->fmcu_handle) {
		pr_err("fmcu started. skip pre done.\n ");
		return;
	}

	path = &pctx->isp_path[ISP_SPATH_CP];
	isp_path_store_done(dev, pctx, path);

	pr_debug("cxt_id:%d done.\n", pctx->ctx_id);
}


static void isp_vid_store_done(enum isp_context_id idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_context *pctx;
	struct isp_path_desc *path;

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[idx];

	if (pctx->fmcu_handle) {
		pr_err("fmcu started. skip vid done.\n ");
		return;
	}

	path = &pctx->isp_path[ISP_SPATH_VID];
	isp_path_store_done(dev, pctx, path);

	pr_debug("cxt_id:%d done.\n", pctx->ctx_id);
}

static void isp_fmcu_store_done(enum isp_context_id idx, void *isp_handle)
{
	int i;
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_context *pctx;
	struct isp_path_desc *path;
	struct camera_frame *pframe;

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[idx];

	/* return buffer to cam channel shared buffer queue. */
	pframe = camera_dequeue(&pctx->proc_queue);
	cambuf_iommu_unmap(&pframe->buf);

	pr_debug("ctx %d, ret src buf: %p,  priv %p\n",
			pctx->ctx_id, pframe,  pctx->cb_priv_data);

	pctx->isp_cb_func(ISP_CB_RET_SRC_BUF, pframe, pctx->cb_priv_data);

	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		if (atomic_read(&path->user_cnt) <= 0)
			continue;
		isp_path_store_done(dev, pctx, path);
	}

	pr_debug("fmuc done cxt_id:%d\n", idx);
}


static void isp_fmcu_load_done(enum isp_context_id idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_context *pctx;

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[idx];

	complete(&pctx->fmcu_com);
	pr_debug("fmuc load done cxt_id:%d\n", idx);
}


static isp_isr isp_isr_handler[32] = {
	[ISP_INT_ISP_ALL_DONE] = isp_all_done,
	[ISP_INT_SHADOW_DONE] = isp_shadow_done,
	[ISP_INT_DISPATCH_DONE] = isp_dispatch_done,
	[ISP_INT_STORE_DONE_PRE] = isp_pre_store_done,
	[ISP_INT_STORE_DONE_VID] = isp_vid_store_done,
	[ISP_INT_FMCU_LOAD_DONE] = isp_fmcu_load_done,
	[ISP_INT_FMCU_STORE_DONE] = isp_fmcu_store_done,
};

struct isp_int_ctx {
	unsigned long reg_offset;
	uint32_t err_mask;
	uint32_t irq_numbers;
	const uint32_t *irq_vect;
} isp_int_ctxs[4] = {
		{/* P0 */
			ISP_P0_INT_BASE,
			ISP_INT_LINE_MASK_ERR,
			(uint32_t)ARRAY_SIZE(isp_irq_process),
			isp_irq_process,
		},
		{/* C0 */
			ISP_C0_INT_BASE,
			ISP_INT_LINE_MASK_ERR,
			(uint32_t)ARRAY_SIZE(isp_irq_process),
			isp_irq_process,
		},
		{/* P1 */
			ISP_P1_INT_BASE,
			ISP_INT_LINE_MASK_ERR,
			(uint32_t)ARRAY_SIZE(isp_irq_process),
			isp_irq_process,
		},
		{/* C1 */
			ISP_C1_INT_BASE,
			ISP_INT_LINE_MASK_ERR,
			(uint32_t)ARRAY_SIZE(isp_irq_process),
			isp_irq_process,
		},
};

static irqreturn_t isp_isr_root(int irq, void *priv)
{
	unsigned long irq_offset;
	uint32_t iid;
	enum isp_context_id c_id;
	uint32_t sid, k;
	uint32_t err_mask;
	uint32_t irq_line = 0;
	uint32_t irq_numbers = 0;
	const uint32_t *irq_vect = NULL;
	struct isp_pipe_dev *isp_handle = (struct isp_pipe_dev *)priv;

	if (!isp_handle) {
		pr_err("error: null dev\n");
		return IRQ_HANDLED;
	}
	pr_debug("isp irq %d, %p\n", irq, priv);

	if (irq == isp_handle->irq_no[0]) {
		iid = 0;
	} else if (irq == isp_handle->irq_no[1]) {
		iid = 1;
	} else {
		pr_err("error irq %d mismatched\n", irq);
		return IRQ_NONE;
	}

	for (sid = 0; sid < 2; sid++) {
		c_id = (iid << 1) | sid;
		irq_offset = isp_int_ctxs[c_id].reg_offset;
		err_mask = isp_int_ctxs[c_id].err_mask;
		irq_numbers = isp_int_ctxs[c_id].irq_numbers;
		irq_vect = isp_int_ctxs[c_id].irq_vect;

		pr_debug("offset %lx,  num %d, vect: %p\n",
			 irq_offset, irq_numbers,  irq_vect);
		irq_line = ISP_HREG_RD(irq_offset + ISP_INT_INT0);
		pr_debug("cid: %d,  irq_line: %08x\n", c_id,  irq_line);
		if (unlikely(irq_line == 0))
			continue;
		for (k = 0; k < 32; k++) {
			if (irq_line & (1 << k))
				irq_done[c_id][k]++;
		}

		/*clear the interrupt*/
		ISP_HREG_WR(irq_offset + ISP_INT_CLR0, irq_line);

		pr_debug("isp ctx %d irqno %d, INT: 0x%x\n",
					      c_id, irq, irq_line);

		if (atomic_read(&isp_handle->ctx[c_id].user_cnt) < 1) {
			pr_err("error irq: contex %d not started.\n", c_id);
			return IRQ_HANDLED;
		}

		if (unlikely(err_mask & irq_line)) {
			pr_err("error irq: isp ctx %d, INT:0x%x\n",
					c_id, irq_line);
			/*handle the error here*/
			if (isp_err_pre_proc(c_id, isp_handle))
				return IRQ_HANDLED;
		}

		for (k = 0; k < irq_numbers; k++) {
			uint32_t irq_id = irq_vect[k];

			if (irq_line & (1 << irq_id)) {
				if (isp_isr_handler[irq_id]) {
					isp_isr_handler[irq_id](
						c_id, isp_handle);
				}
			}
			irq_line  &= ~(1 << irq_id);
			if (!irq_line)
				break;
		}
	}

	return IRQ_HANDLED;
}


int isp_irq_request(struct device *p_dev,
		uint32_t *irq_no, void *isp_handle)
{
	int ret = 0;
	uint32_t  id;
	char dev_name[32];
	struct isp_pipe_dev *ispdev;

	if (!p_dev || !isp_handle || !irq_no) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}
	ispdev = (struct isp_pipe_dev *)isp_handle;

	for (id = 0; id < ISP_LOGICAL_COUNT; id++) {
		sprintf(dev_name, "ISP%d", id);
		ispdev->irq_no[id] = irq_no[id];
		ret = devm_request_irq(p_dev,
				ispdev->irq_no[id], isp_isr_root,
				IRQF_SHARED, dev_name, (void *)ispdev);
		if (ret) {
			pr_err("fail to install isp%d irq_no %d\n",
					id, ispdev->irq_no[id]);
			if (id == 1)
				free_irq(ispdev->irq_no[0], (void *)ispdev);
			return -EFAULT;
		}
		pr_info("install isp%d irq_no %d\n", id, ispdev->irq_no[id]);
	}

	memset(irq_done, 0, sizeof(irq_done));

	return ret;
}

int reset_isp_irq_cnt(int ctx_id)
{
	memset(irq_done[ctx_id], 0, sizeof(irq_done[ctx_id]));
	return 0;
}

int trace_isp_irq_cnt(int ctx_id)
{
	int i;

	for (i = 0; i < 32; i++)
		pr_debug("done %d %d :   %d\n", ctx_id, i, irq_done[ctx_id][i]);

	return 0;
}

int isp_irq_free(struct device *p_dev, void *isp_handle)
{
	struct isp_pipe_dev *ispdev;

	ispdev = (struct isp_pipe_dev *)isp_handle;
	if (!ispdev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	devm_free_irq(p_dev, ispdev->irq_no[0], (void *)ispdev);
	devm_free_irq(p_dev, ispdev->irq_no[1], (void *)ispdev);

	return 0;
}
