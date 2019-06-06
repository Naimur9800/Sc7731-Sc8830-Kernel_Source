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


#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <video/sprd_mm.h>

#include "cam_hw.h"
#include "cam_types.h"
#include "cam_queue.h"
#include "cam_buf.h"

#include "dcam_reg.h"
#include "dcam_int.h"
#include "dcam_core.h"
#include "dcam_path.h"
#include "dcam_interface.h"


/* Macro Definitions */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_INT: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__


typedef void (*dcam_isr) (void *param);

static uint32_t dcam0_irq_processed[] = {
	DCAM_CAP_SOF,
	DCAM_FULL_PATH_TX_DONE,
	DCAM_BIN_PATH_TX_DONE,
	DCAM_AEM_PATH_TX_DONE,
	DCAM_PDAF_PATH_TX_DONE,
	DCAM_VCH2_PATH_TX_DONE,
	DCAM_VCH3_PATH_TX_DONE,
	DCAM_AFM_INTREQ1,
	DCAM_AFL_TX_DONE,
	DCAM_NR3_TX_DONE,
};

static uint32_t dcam1_irq_processed[] = {
	DCAM_CAP_SOF,
	DCAM_FULL_PATH_TX_DONE,
	DCAM_BIN_PATH_TX_DONE,
	DCAM_AEM_PATH_TX_DONE,
	DCAM_AFM_INTREQ1,
	DCAM_AFL_TX_DONE,
	DCAM_NR3_TX_DONE,
};

static uint32_t dcam2_irq_processed[] = {
	DCAM_CAP_SOF,
	DCAM_FULL_PATH_TX_DONE,
};

static int irq_done[3][32];


#ifdef TEST_SHARKL3
static void dcam_auto_copy(enum dcam_id idx)
{
	uint32_t mask = BIT_4 | BIT_6 | BIT_8 | BIT_10 | BIT_12 | BIT_14 | BIT_16 | BIT_18;
	mask <<= 1;
	pr_info("DCAM%d: auto copy 0x%0x:\n", idx, mask);

	/* auto copy all*/
	DCAM_REG_MWR(idx, DCAM_CONTROL, mask, mask);
}
#else
static void dcam_auto_copy(enum dcam_id idx)
{
	uint32_t mask = BIT_1 | BIT_5 | BIT_7 | BIT_9 | BIT_11 | BIT_13 | BIT_15 | BIT_17;

	pr_info("DCAM%d: auto copy 0x%0x:\n", idx, mask);

	/* auto copy all*/
	DCAM_REG_MWR(idx, DCAM_CONTROL, mask, mask);
}
#endif


static void dcam_path_tx_done(
	struct dcam_pipe_dev *dev, enum dcam_path_id path_id)
{
	struct dcam_path_desc *path;
	struct camera_frame *frame;
	struct timespec cur_ts;

	path = &dev->path[path_id];
	pr_debug("dev%d %p path %d\n", dev->idx, dev, path_id);

	if (atomic_read(&path->set_frm_cnt) <= 1) {
		pr_warn("dcam %d should not output. deci %d\n",
				dev->idx, path->frm_deci);
		return;
	}

	frame = camera_dequeue(&path->result_queue);
	if (frame) {
		atomic_dec(&path->set_frm_cnt);

		if (unlikely(frame->is_reserved)) {
			camera_enqueue(&path->reserved_buf_queue, frame);
		} else {
			pr_debug("outframe %p, time %06d.%06d\n", frame,
				(int)frame->time.tv_sec, (int)(frame->time.tv_usec));
			ktime_get_ts(&cur_ts);
			frame->boot_time = ktime_get_boottime();
			frame->time.tv_sec = cur_ts.tv_sec;
			frame->time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;
			pr_debug("outframe %p, time %06d.%06d\n", frame,
				(int)frame->time.tv_sec, (int)(frame->time.tv_usec));
			cambuf_iommu_unmap(&frame->buf);

			frame->fid = path->frm_cnt;
			if (path_id == DCAM_PATH_FULL || path_id == DCAM_PATH_BIN)
				dev->dcam_cb_func(DCAM_CB_DATA_DONE,
						frame, dev->cb_priv_data);
		}
	} else {
		pr_err("error: no output buffer from dcam%d path %d\n",
					dev->idx, path_id);
	}
}

static void dcam_cap_sof(void *param)
{
	int ret = 0;
	int i;
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)param;
	struct dcam_path_desc *path;

	for (i  = 0; i < DCAM_PATH_MAX; i++) {
		path = &dev->path[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;

		path->frm_cnt++;
		if (path->frm_cnt <= path->frm_skip)
			continue;

		path->frm_deci_cnt++;
		if (path->frm_deci_cnt >= path->frm_deci) {
			path->frm_deci_cnt = 0;
			ret = dcam_path_set_store_frm(dev, path);
			/* set_frm_cnt should be larger than done_frm_cnt. */
			if (ret == 0) {
				atomic_inc(&path->set_frm_cnt);
				pr_debug("store cnt %d\n",
					atomic_read(&path->set_frm_cnt));
			}
		}
	}

	dcam_auto_copy(dev->idx);
}


static void dcam_full_path_done(void *param)
{
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)param;

	dcam_path_tx_done(dev, DCAM_PATH_FULL);
	pr_debug("full path done. dev %p\n", dev);
}

static void dcam_bin_path_done(void *param)
{
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)param;

	dcam_path_tx_done(dev, DCAM_PATH_BIN);
	pr_debug("bin path done. dev %p\n", dev);
}


static void dcam_aem_done(void *param)
{
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)param;

	dcam_path_tx_done(dev, DCAM_PATH_AEM);
}


static void dcam_pdaf_done(void *param)
{
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)param;

	dcam_path_tx_done(dev, DCAM_PATH_PDAF);
}


static void dcam_vch2_done(void *param)
{
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)param;

	pr_debug("vch2 done.\n");
	dcam_path_tx_done(dev, DCAM_PATH_VCH2);
}

static void dcam_vch3_done(void *param)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)param;
	struct dcam_path_desc *path;
	path = &dev->path[DCAM_PATH_VCH3];

	path->frm_cnt++;
	path->frm_deci_cnt++;
	if (path->frm_deci_cnt >= path->frm_deci) {
		path->frm_deci_cnt = 0;
		ret = dcam_path_set_store_frm(dev, path);
		/* set_frm_cnt should be larger than done_frm_cnt. */
		if (ret == 0) {
			atomic_inc(&path->set_frm_cnt);
			pr_debug("store cnt %d\n",
				atomic_read(&path->set_frm_cnt));
		}
	}

	dcam_path_tx_done(dev, DCAM_PATH_VCH3);
}

static void dcam_afm_done(void *param)
{
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)param;

	dcam_path_tx_done(dev, DCAM_PATH_AFM);
}


static void dcam_afl_done(void *param)
{
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)param;

	dcam_path_tx_done(dev, DCAM_PATH_AFL);
}


static void dcam_nr3_done(void *param)
{
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)param;

	dcam_path_tx_done(dev, DCAM_PATH_3DNR);
}


static int dcam_err_pre_proc(void *param, uint32_t status)
{
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)param;

	pr_err("dcam%d error 0x%x happened.\n", dev->idx, status);

	if (dev->err_status)
		return 0;
	dev->err_status = 1;

	dev->dcam_cb_func(DCAM_CB_DEV_ERR,
						dev, dev->cb_priv_data);
	return 0;
}


static const dcam_isr isr_list[DCAM_IRQ_NUMBER] = {
	[DCAM_CAP_SOF] = dcam_cap_sof,
	[DCAM_FULL_PATH_TX_DONE] = dcam_full_path_done,
	[DCAM_BIN_PATH_TX_DONE] = dcam_bin_path_done,
	[DCAM_AEM_PATH_TX_DONE] = dcam_aem_done,
	[DCAM_PDAF_PATH_TX_DONE] = dcam_pdaf_done,
	[DCAM_VCH2_PATH_TX_DONE] = dcam_vch2_done,
	[DCAM_VCH3_PATH_TX_DONE] = dcam_vch3_done,
	[DCAM_AFM_INTREQ1] = dcam_afm_done,
	[DCAM_AFL_TX_DONE] = dcam_afl_done,
	[DCAM_NR3_TX_DONE] = dcam_nr3_done,
};


struct dcam_int_ctx {
	uint32_t irq_numbers;
	uint32_t *irq_vects;
} dcam_int_ctxs[DCAM_ID_MAX] = {
	{
		ARRAY_SIZE(dcam0_irq_processed),
		dcam0_irq_processed,
	},
	{
		ARRAY_SIZE(dcam1_irq_processed),
		dcam1_irq_processed,
	},
	{
		ARRAY_SIZE(dcam2_irq_processed),
		dcam2_irq_processed,
	},
};

static irqreturn_t dcam_isr_root(int irq, void *priv)
{
	int i;
	enum dcam_id idx;
	uint32_t irq_mmu = 0, irq_line = 0, status = 0, vect = 0;
	int irq_numbers;
	uint32_t *irq_vects;
	struct dcam_pipe_dev *dcam_dev = (struct dcam_pipe_dev *)priv;

	if (irq != dcam_dev->irq_no) {
		pr_err("error irq %d, mismatched to dcam irq %d\n",
					irq, dcam_dev->irq_no);
		return IRQ_NONE;
	}
	status = atomic_read(&dcam_dev->started);
	pr_debug("dcam%d started %d\n", dcam_dev->idx, status);

	if (status == 0) {
		pr_err("dcam %d is stopped.\n", dcam_dev->idx);
		return IRQ_HANDLED;
	}

	idx = dcam_dev->idx;
	status = DCAM_REG_RD(idx, DCAM_INT_MASK);
	irq_mmu = DCAM_REG_RD(idx, DCAM_INT_RAW);
	pr_debug("dcam %d. irq %x %x.  dev %p\n", idx, status, irq_mmu, dcam_dev);

	status &= DCAM_IRQ_LINE_MASK;
	if (unlikely(status == 0))
		return IRQ_NONE;
	DCAM_REG_WR(idx, DCAM_INT_CLR, status);

	irq_line = status;
	for (i = 0; i < 32; i++) {
		if (irq_line & (1 << i))
			irq_done[idx][i]++;
	}

	if (unlikely(DCAM_IRQ_ERR_MASK & status)) {
		pr_err("dcam%d error interrupts: %x\n", idx, status);
		if (dcam_err_pre_proc(priv, status))
			return IRQ_HANDLED;
	}

	irq_numbers = dcam_int_ctxs[idx].irq_numbers;
	irq_vects = dcam_int_ctxs[idx].irq_vects;

	for (i = 0; i < irq_numbers; i++) {
		vect = irq_vects[i];
		if (irq_line & (1 << (uint32_t)vect)) {
			if (isr_list[vect])
				isr_list[vect](priv);
		}
		irq_line &= ~(uint32_t)(1 << (uint32_t)vect);
		if (!irq_line)
			break;
	}

	return IRQ_HANDLED;
}



int dcam_irq_request(struct device *p_dev, int irq, void *param)
{
	int ret = 0;
	struct dcam_pipe_dev *dcam_dev;
	char dev_name[32];

	if (!p_dev || !param) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}
	dcam_dev = (struct dcam_pipe_dev *)param;
	dcam_dev->irq_no = irq;

	sprintf(dev_name, "DCAM%d", dcam_dev->idx);
	ret = devm_request_irq(p_dev, dcam_dev->irq_no, dcam_isr_root,
						IRQF_SHARED, dev_name, (void *)dcam_dev);
	if (ret) {
		pr_err("fail to install DCAM%d irq_no %d\n",
				dcam_dev->idx, dcam_dev->irq_no);
		return -EFAULT;
	}
	memset(irq_done[dcam_dev->idx], 0, sizeof(irq_done[dcam_dev->idx]));
	return 0;
}


int reset_dcam_irq_cnt(int idx)
{
	memset(irq_done[idx], 0, sizeof(irq_done[idx]));
	return 0;
}

int trace_dcam_irq_cnt(int idx)
{
	int i;

	for (i = 0; i < 32; i++)
		pr_debug("done %d %d :   %d\n", idx, i, irq_done[idx][i]);

	return 0;
}

int dcam_irq_free(struct device *p_dev, void *param)
{
	struct dcam_pipe_dev *dcam_dev;

	if (!param) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

	dcam_dev = (struct dcam_pipe_dev *)param;
	devm_free_irq(p_dev, dcam_dev->irq_no, dcam_dev);

	return 0;
}
