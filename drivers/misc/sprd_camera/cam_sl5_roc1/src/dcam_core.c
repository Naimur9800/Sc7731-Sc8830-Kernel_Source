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


#include <linux/of.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/sprd_ion.h>

#include <video/sprd_isp_r8p1.h>
#include <video/sprd_img.h>
#include <video/sprd_mm.h>


#include "cam_pw_domain.h"
#include "cam_hw.h"
#include "cam_types.h"
#include "cam_queue.h"
#include "cam_buf.h"

#include "dcam_reg.h"
#include "dcam_int.h"

#include "dcam_interface.h"
#include "dcam_core.h"
#include "dcam_path.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_CORE: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__


static DEFINE_MUTEX(dcam_pipe_dev_mutex);
static struct dcam_pipe_dev *s_dcam_dev[DCAM_ID_MAX];

struct statis_path_buf_info s_statis_path_info_all[6] = {
	{DCAM_PATH_PDAF,    ISP_PDAF_BUF_SIZE, 8},
	{DCAM_PATH_AEM,     ISP_AEM_BUF_SIZE, 8},
	{DCAM_PATH_AFM,     ISP_AFM_BUF_SIZE, 8},
	{DCAM_PATH_AFL,      ISP_AFL_BUF_SIZE, 8},
	{DCAM_PATH_HIST,    ISP_HIST_BUF_SIZE, 8},
	{DCAM_PATH_3DNR,   ISP_3DNR_BUF_SIZE, 8},
};


static int set_dcam_cap_info(
	struct dcam_pipe_dev *dev,
	struct dcam_mipi_info  *cap_info)
{
	int ret = 0;
	uint32_t idx = dev->idx;
	uint32_t reg_val;

	/* 1. set mipi interface  */
	if (cap_info->sensor_if != DCAM_CAP_IF_CSI2) {
		pr_err("error: unsupported sensor if: %d", cap_info->sensor_if);
		return -EINVAL;
	}

	/* 2  data format */
	reg_val = (cap_info->format == DCAM_CAP_MODE_RAWRGB) ? 1 : 0;

	if (cap_info->format == DCAM_CAP_MODE_RAWRGB) {
		DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_1, 1 << 1);

		DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG,
						BIT_17 | BIT_16, cap_info->pattern << 16);

		/* x/y deci should be disable for raw-rgb */
		DCAM_REG_MWR(idx, DCAM_MIPI_CAP_FRM_CTRL,
						BIT_9 | BIT_8 | BIT_5 | BIT_4, 0);
	} else {
		DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_1,  0 << 1);

		DCAM_REG_MWR(idx, DCAM_MIPI_CAP_FRM_CTRL,
						BIT_1 | BIT_0, cap_info->pattern);

		DCAM_REG_MWR(idx, DCAM_MIPI_CAP_FRM_CTRL,
				BIT_9 | BIT_8 | BIT_5 | BIT_4,
				(cap_info->y_factor << 8) | (cap_info->x_factor << 4));
	}
	DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_2, cap_info->mode << 2);

	DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_3, cap_info->href << 3);

	reg_val = (cap_info->data_bits == DCAM_CAP_12_BITS) ? 2 :
			 ((cap_info->data_bits == DCAM_CAP_10_BITS) ? 1 : 0);
	DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_5 | BIT_4, reg_val << 4);

	DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG,
					BIT_7 | BIT_6, cap_info->frm_deci << 6);

	reg_val = (cap_info->cap_size.start_y << 16);
	reg_val |= cap_info->cap_size.start_x;
	DCAM_REG_WR(idx, DCAM_MIPI_CAP_START, reg_val);
	reg_val = (cap_info->cap_size.start_y +
				cap_info->cap_size.size_y - 1) << 16;
	reg_val |= (cap_info->cap_size.start_x +
				cap_info->cap_size.size_x - 1);
	DCAM_REG_WR(idx, DCAM_MIPI_CAP_END, reg_val);

	DCAM_REG_MWR(idx, DCAM_MIPI_CAP_FRM_CLR, BIT_6, (1 << 6));

	DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG,
					0xF << 8, cap_info->frm_skip << 8);

	/* bypass 4in1 */
	DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_12,
					(!cap_info->is_4in1) << 12);

	pr_info("cap size : %d %d %d %d\n",
		cap_info->cap_size.start_x, cap_info->cap_size.start_y,
		cap_info->cap_size.size_x, cap_info->cap_size.size_y);
	pr_info("cap: frm %d, mode %d, bits %d, pattern %d, href %d\n",
		cap_info->format, cap_info->mode, cap_info->data_bits,
		cap_info->pattern, cap_info->href);
	pr_info("cap: deci %d, skip %d, x %d, y %d, 4in1 %d\n",
		cap_info->frm_deci, cap_info->frm_skip, cap_info->x_factor,
		cap_info->y_factor, cap_info->is_4in1);

	return ret;
}

void dcam_ret_src_frame(void *param)
{
	struct camera_frame *frame;
	struct dcam_pipe_dev *dev;

	if (!param) {
		pr_err("error: null input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	dev = (struct dcam_pipe_dev *)frame->priv_data;
	pr_debug("frame %p, ch_id %d, buf_fd %d\n",
		frame, frame->channel_id, frame->buf.mfd[0]);

	cambuf_iommu_unmap(&frame->buf);
	dev->dcam_cb_func(
		DCAM_CB_RET_SRC_BUF,
		frame, dev->cb_priv_data);
}

void dcam_ret_out_frame(void *param)
{
	struct camera_frame *frame;
	struct dcam_pipe_dev *dev;
	struct dcam_path_desc *path;

	if (!param) {
		pr_err("error: null input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;

	if (frame->is_reserved) {
		path =  (struct dcam_path_desc *)frame->priv_data;
		camera_enqueue(&path->reserved_buf_queue, frame);
	} else {
		cambuf_iommu_unmap(&frame->buf);
		dev = (struct dcam_pipe_dev *)frame->priv_data;
		dev->dcam_cb_func(DCAM_CB_DATA_DONE, frame, dev->cb_priv_data);
	}
}


void dcam_destroy_reserved_buf(void *param)
{
	struct camera_frame *frame;

	if (!param) {
		pr_err("error: null input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;

	if (unlikely(frame->is_reserved == 0)) {
		pr_err("error: frame has no reserved buffer.");
		return;
	}
	/* is_reserved:
	  *  1:  basic mapping reserved buffer;
	  *  2:  copy of reserved buffer.
	  */
	if (frame->is_reserved == 1) {
		cambuf_iommu_unmap(&frame->buf);
		cambuf_put_ionbuf(&frame->buf);
	}
	put_empty_frame(frame);
}

void dcam_destroy_statis_buf(void *param)
{
	struct camera_frame *frame;

	if (!param) {
		pr_err("error: null input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	put_empty_frame(frame);
}


static struct camera_buf *get_reserved_buffer(struct dcam_pipe_dev *dev)
{
	int ret = 0;
	int iommu_enable = 0;    /* todo: get from dev dts config value */
	size_t size;
	struct camera_buf *ion_buf = NULL;

	ion_buf = kzalloc(sizeof(*ion_buf), GFP_KERNEL);
	if (!ion_buf) {
		pr_err("fail to alloc buffer.\n");
		goto nomem;
	}

	if (sprd_iommu_attach_device(&dev->dcam_hw->pdev->dev) == 0)
		iommu_enable = 1;

	size = DCAM_INTERNAL_RES_BUF_SIZE;
	ret = cambuf_alloc(ion_buf, size, 0, iommu_enable);
	if (ret) {
		pr_err("fail to get dcam reserverd buffer\n");
		goto ion_fail;
	}

	ret = cambuf_kmap(ion_buf);
	if (ret) {
		pr_err("fail to kmap dcame reserved buffer\n");
		goto kmap_fail;
	}

	ret = cambuf_iommu_map(ion_buf,
				&dev->dcam_hw->pdev->dev);
	if (ret) {
		pr_err("fail to map dcam reserved buffer to iommu\n");
		goto hwmap_fail;
	}
	pr_info("dcam%d done. ion %p\n", dev->idx, ion_buf);
	return ion_buf;

hwmap_fail:
	cambuf_kunmap(ion_buf);
kmap_fail:
	cambuf_free(ion_buf);
ion_fail:
	kfree(ion_buf);
nomem:
	return NULL;
}

static int put_reserved_buffer(struct dcam_pipe_dev *dev)
{
	struct camera_buf *ion_buf = NULL;

	ion_buf = (struct camera_buf *)dev->internal_reserved_buf;
	if (!ion_buf) {
		pr_info("no reserved buffer.\n");
		return 0;
	}
	pr_info("ionbuf %p\n", ion_buf);

	cambuf_iommu_unmap(ion_buf);
	cambuf_kunmap(ion_buf);
	cambuf_free(ion_buf);
	kfree(ion_buf);
	dev->internal_reserved_buf = NULL;

	return 0;
}



static void init_reserved_statis_bufferq(struct dcam_pipe_dev *dev)
{
	int i, j;
	struct camera_frame *newfrm;
	enum dcam_path_id path_id;
	struct camera_buf *ion_buf;
	struct dcam_path_desc *path;

	if (dev->internal_reserved_buf == NULL) {
		ion_buf =  get_reserved_buffer(dev);
		if (IS_ERR_OR_NULL(ion_buf)) {
			return;
		}
		dev->internal_reserved_buf = ion_buf;
	}

	for (i = 0; i < (int)ARRAY_SIZE(s_statis_path_info_all); i++) {
		path_id = s_statis_path_info_all[i].path_id;
		path = &dev->path[path_id];

		j  = 0;
		while (j < DCAM_RESERVE_BUF_Q_LEN) {
			newfrm = get_empty_frame();
			if (newfrm) {
				newfrm->is_reserved = 1;
				memcpy(&newfrm->buf, ion_buf, sizeof(struct camera_buf));
				camera_enqueue(&path->reserved_buf_queue, newfrm);
				j++;
			}
			pr_debug("path%d reserved buffer %d\n", path->path_id, j);
		}
	}

	pr_info("init statis reserver bufq done: %p\n", ion_buf);
}

static int init_statis_bufferq(struct dcam_pipe_dev *dev)
{
	int ret = 0;
	int i;
	int count = 0;
	size_t used_size = 0, total_size;
	size_t buf_size;
	unsigned long kaddr;
	unsigned long paddr;
	unsigned long uaddr;
	enum dcam_path_id path_id;
	struct camera_buf *ion_buf;
	struct camera_frame *pframe;
	struct dcam_path_desc *path;

	for (i = 0; i < 6; i++) {
		path_id = s_statis_path_info_all[i].path_id;
		path = &dev->path[path_id];
		camera_queue_init(&path->out_buf_queue,
				DCAM_OUT_BUF_Q_LEN, 0, dcam_destroy_statis_buf);
		camera_queue_init(&path->result_queue,
				DCAM_RESULT_Q_LEN, 0, dcam_destroy_statis_buf);
		camera_queue_init(&path->reserved_buf_queue,
				DCAM_RESERVE_BUF_Q_LEN, 0, dcam_destroy_statis_buf);
	}

	init_reserved_statis_bufferq(dev);

	ion_buf = dev->statis_buf;
	if (ion_buf == NULL) {
		pr_info("dcam%d no init statis buf.\n", dev->idx);
		return ret;
	}

	kaddr = ion_buf->addr_k[0];
	paddr = ion_buf->iova[0];
	uaddr = ion_buf->addr_vir[0];
	total_size = ion_buf->size[0];

	pr_info("size %d  addr %p %p,  %08x\n", (int)total_size,
			(void *)kaddr, (void *)uaddr, (uint32_t)paddr);

	do {
		for (i = 0; i < 6; i++) {
			path_id = s_statis_path_info_all[i].path_id;
			buf_size = s_statis_path_info_all[i].buf_size;
			path = &dev->path[path_id];
			used_size += buf_size;
			if (used_size >= total_size)
				break;

			pframe = get_empty_frame();
			if (pframe) {
				pframe->channel_id = path_id;
				pframe->buf.addr_vir[0] = uaddr;
				pframe->buf.addr_k[0] = kaddr;
				pframe->buf.iova[0] = paddr;
				pframe->buf.size[0] = buf_size;
				camera_enqueue(&path->out_buf_queue, pframe);
				uaddr += buf_size;
				kaddr += buf_size;
				paddr += buf_size;
			}
		}
		count++;
	} while (used_size < total_size);

	pr_info("done. count %d\n", count);
	return ret;
}

static int deinit_statis_bufferq(struct dcam_pipe_dev *dev)
{
	int ret = 0;
	int i;
	enum dcam_path_id path_id;
	struct dcam_path_desc *path;

	for (i = 0; i < 6; i++) {
		path_id = s_statis_path_info_all[i].path_id;
		path = &dev->path[path_id];
		camera_queue_clear(&path->out_buf_queue);
		camera_queue_clear(&path->result_queue);
		camera_queue_clear(&path->reserved_buf_queue);
	}
	pr_info("done.\n");

	return ret;
}


static int unmap_statis_buffer(struct dcam_pipe_dev *dev)
{
	struct camera_buf *ion_buf = NULL;

	ion_buf = dev->statis_buf;
	if (!ion_buf) {
		pr_info("no statis buffer.\n");
		return 0;
	}
	pr_info("%p\n", ion_buf);

	cambuf_iommu_unmap(ion_buf);
	kfree(ion_buf);
	dev->statis_buf = NULL;

	pr_info("done %p\n", ion_buf);

	return 0;
}

static int dcam_cfg_statis_buffer(
		struct dcam_pipe_dev *dev,
		struct isp_statis_buf_input *input)
{
	int ret = 0;
	unsigned long kaddr;
	unsigned long uaddr;
	unsigned long hw_addr;
	struct camera_buf *ion_buf = NULL;

	kaddr = input->kaddr[1];
	kaddr <<= 32;
	kaddr |= input->kaddr[0];
	uaddr = input->vir_addr[1];
	uaddr <<= 32;
	uaddr |= input->vir_addr[0];

	if (input->type == STATIS_INIT) {
		ion_buf = kzalloc(sizeof(*ion_buf), GFP_KERNEL);
		if (IS_ERR_OR_NULL(ion_buf)) {
			pr_err("fail to alloc memory for dcam%d statis buf.\n",
					dev->idx);
			ret = -ENOMEM;
			goto exit;
		}

		ion_buf->mfd[0] = input->u.init_data.mfd;
		ion_buf->size[0] = input->u.init_data.buf_size;
		ion_buf->addr_vir[0] = uaddr;
		ion_buf->addr_k[0] = kaddr;
		ion_buf->type = CAM_BUF_USER;

		ret = cambuf_iommu_map(ion_buf,
				&dev->dcam_hw->pdev->dev);
		if (ret) {
			pr_err("fail to map dcam statis buffer to iommu\n");
			kfree(ion_buf);
			ret = -EINVAL;
			goto exit;
		}
		dev->statis_buf = ion_buf;

		hw_addr = ion_buf->iova[0];
		pr_info("map dcam statis buffer. mfd: %d, size: 0x%x\n",
							 ion_buf->mfd[0], (int)ion_buf->size[0]);
		pr_info("uaddr: %p, kaddr: %p,  iova: 0x%x\n",
				(void *)uaddr, (void *)kaddr, (uint32_t)hw_addr);
	} else {
		pr_debug("to do.\n");
		pr_debug("to do.\n");
	}
exit:
	return ret;
}

static int dcam_offline_start_frame(void *param)
{
	int ret = 0;
	int i, loop;
	struct dcam_pipe_dev *dev = NULL;
	struct camera_frame *pframe = NULL;
	struct dcam_path_desc *path;
	struct dcam_fetch_info *fetch;

	pr_debug("enter.\n");

	dev = (struct dcam_pipe_dev *)param;
	fetch = &dev->fetch;

	pframe = camera_dequeue(&dev->in_queue);
	if (pframe == NULL) {
		pr_warn("no frame from in_q. dcam%d\n", dev->idx);
		return 0;
	}

	pr_debug("frame %p, ctx %d  ch_id %d.  buf_fd %d\n", pframe,
		dev->idx, pframe->channel_id, pframe->buf.mfd[0]);

	ret = cambuf_iommu_map(&pframe->buf,
				&dev->dcam_hw->pdev->dev);
	if (ret) {
		pr_err("fail to map buf to dcam%d iommu.\n", dev->idx);
		goto map_err;
	}

	loop = 0;
	do {
		ret = camera_enqueue(&dev->proc_queue, pframe);
		if (ret == 0)
			break;
		pr_info("wait for proc queue. loop %d\n", loop);

		/* wait for previous frame proccessed done. */
		msleep(1);
	} while (loop++ < 500);

	if (ret) {
		pr_err("error: input frame queue tmeout.\n");
		ret = -EINVAL;
		goto inq_overflow;
	}

	for (i  = 0; i < DCAM_PATH_MAX; i++) {
		path = &dev->path[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;
		ret = dcam_path_set_store_frm(dev, path);
		if (ret == 0) {
			atomic_inc(&path->set_frm_cnt);
			dcam_start_path(dev, path);
		}
	}

	/* todo - need to cfg fetch param from input or frame. */
	fetch->is_loose = 0;
	fetch->endian = ENDIAN_LITTLE;
	fetch->pattern = COLOR_ORDER_GB;
	fetch->size.w = pframe->width;
	fetch->size.h = pframe->height;
	fetch->trim.start_x = 0;
	fetch->trim.start_y = 0;
	fetch->trim.size_x = pframe->width;
	fetch->trim.size_y = pframe->height;
	fetch->addr.addr_ch0 = (uint32_t)pframe->buf.iova[0];

	/* todo - wait last offline frame completion done here. */
	ret = dcam_start_fetch(dev, fetch);

	return ret;

inq_overflow:
	cambuf_iommu_unmap(&pframe->buf);
map_err:
	/* return buffer to cam channel shared buffer queue. */
	dev->dcam_cb_func(DCAM_CB_RET_SRC_BUF, pframe, dev->cb_priv_data);
	return ret;
}

static int dcam_offline_thread_loop(void *arg)
{
	struct dcam_pipe_dev *dev = NULL;
	struct cam_offline_thread_info *thrd;

	if (!arg) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}

	thrd = (struct cam_offline_thread_info *)arg;
	dev = (struct dcam_pipe_dev *)thrd->ctx_handle;

	while (1) {
		if (wait_for_completion_interruptible(
			&thrd->thread_com) == 0) {
			if (atomic_cmpxchg(
					&thrd->thread_stop, 1, 0) == 1) {
				pr_info("dcam%d offline thread stop.\n",
						dev->idx);
				break;
			}
			pr_debug("thread com done.\n");

			if (thrd->proc_func(dev)) {
				pr_err("fail to start dcam pipe to proc. exit thread\n");
				dev->dcam_cb_func(
					DCAM_CB_DEV_ERR, dev,
					dev->cb_priv_data);
				break;
			}
		} else {
			pr_debug("offline thread exit!");
			break;
		}
	}

	return 0;
}

static int dcam_stop_offline_thread(void *param)
{
	int cnt = 0;
	struct cam_offline_thread_info *thrd;

	thrd = (struct cam_offline_thread_info *)param;

	if (thrd->thread_task) {
		atomic_set(&thrd->thread_stop, 1);
		complete(&thrd->thread_com);
		while (cnt < 1000) {
			cnt++;
			if (atomic_read(&thrd->thread_stop) == 0)
				break;
			udelay(1000);
		}
		thrd->thread_task = NULL;
		pr_info("offline thread stopped. wait %d ms\n", cnt);
	}

	return 0;
}

static int dcam_create_offline_thread(void *param)
{
	struct dcam_pipe_dev *dev;
	struct cam_offline_thread_info *thrd;
	char thread_name[32] = { 0 };

	dev = (struct dcam_pipe_dev *)param;
	thrd = &dev->thread;
	thrd->ctx_handle = dev;
	thrd->proc_func = dcam_offline_start_frame;
	atomic_set(&thrd->thread_stop, 0);
	init_completion(&thrd->thread_com);

	sprintf(thread_name, "dcam%d_offline", dev->idx);
	thrd->thread_task = kthread_run(
						dcam_offline_thread_loop,
					      thrd, thread_name);
	if (IS_ERR_OR_NULL(thrd->thread_task)) {
		pr_err("fail to start offline thread for dcam%d\n",
				dev->idx);
		return -EFAULT;
	}

	pr_info("dcam%d offline thread created.\n", dev->idx);
	return 0;
}

static int sprd_dcam_get_path(void *dcam_handle,
				void *param)
{
	int path_id = -1;
	struct dcam_pipe_dev *dev;
	struct dcam_path_desc *path = NULL;
	struct cam_channel_desc *ch_desc;

	if (!dcam_handle || !param) {
		pr_err("error input param: %p, %p\n",
				dcam_handle, param);
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	ch_desc = (struct cam_channel_desc *)param;

	if ((ch_desc->prop == CH_PROP_CAP) &&
		(ch_desc->output_size.w == ch_desc->input_trim.size_x) &&
		(ch_desc->output_size.h == ch_desc->input_trim.size_y)) {
		path = &dev->path[DCAM_PATH_FULL];
		if (atomic_inc_return(&path->user_cnt) == 1) {
			path_id = DCAM_PATH_FULL;
		} else {
			atomic_dec(&path->user_cnt);
			pr_err("error: dcam%d full path in use.\n", dev->idx);
		}
	} else {
		path = &dev->path[DCAM_PATH_BIN];
		if (atomic_inc_return(&path->user_cnt) == 1) {
			path_id = DCAM_PATH_BIN;
		} else {
			atomic_dec(&path->user_cnt);
			pr_err("error: dcam%d bin path in use.\n", dev->idx);
		}
	}

	if (path) {
		path->in_size = ch_desc->input_size;
		path->in_trim = ch_desc->input_trim;
		path->out_size = ch_desc->output_size;
		camera_queue_init(&path->result_queue, DCAM_RESULT_Q_LEN,
							0, dcam_ret_out_frame);
		camera_queue_init(&path->out_buf_queue, DCAM_OUT_BUF_Q_LEN,
							0, dcam_ret_out_frame);
		camera_queue_init(&path->reserved_buf_queue, DCAM_RESERVE_BUF_Q_LEN,
							0, dcam_destroy_reserved_buf);
	}

	return path_id;
}

static int sprd_dcam_put_path(void *dcam_handle, uint32_t path_id)
{
	int ret = 0;
	struct dcam_pipe_dev *dev;
	struct dcam_path_desc *path = NULL;

	if (!dcam_handle || (path_id >= DCAM_PATH_MAX)) {
		pr_err("error input param: %p, path %d\n",
				dcam_handle, path_id);
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	path = &dev->path[path_id];
	atomic_set(&path->user_cnt, 0);

	camera_queue_clear(&path->result_queue);
	camera_queue_clear(&path->out_buf_queue);
	camera_queue_clear(&path->reserved_buf_queue);

	pr_info("put path %d done\n", path_id);
	return ret;
}


static int sprd_dcam_cfg_path(void *dcam_handle,
				enum dcam_path_cfg_cmd cfg_cmd,
				uint32_t path_id, void *param)
{
	int ret = 0;
	struct dcam_pipe_dev *dev;
	struct dcam_path_desc *path;
	struct camera_frame *pframe;

	if (!dcam_handle || !param || (path_id > DCAM_PATH_MAX)) {
		pr_err("error input param: %p, %p, %d\n",
				dcam_handle, param, path_id);
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	path = &dev->path[path_id];

	if (atomic_read(&path->user_cnt) == 0) {
		pr_err("dcam%d, path %d is not in use.\n",
				dev->idx, path_id);
		return -EFAULT;
	}

	switch (cfg_cmd) {
	case DCAM_PATH_CFG_OUTPUT_RESERVED_BUF:
	case DCAM_PATH_CFG_OUTPUT_BUF:
		pframe = (struct camera_frame *)param;
		ret = cambuf_iommu_map(
				&pframe->buf, &dev->dcam_hw->pdev->dev);
		if (ret)
			goto exit;

		/* is_reserved:
		  *  1:  basic mapping reserved buffer;
		  *  2:  copy of reserved buffer.
		  */
		if (cfg_cmd == DCAM_PATH_CFG_OUTPUT_RESERVED_BUF) {
			int i = 1;
			struct camera_frame *newfrm;

			pframe->is_reserved = 1;
			pframe->priv_data = path;
			camera_enqueue(&path->reserved_buf_queue, pframe);

			pr_info("config dcam output reserverd buffer.\n");

			while (i < DCAM_RESERVE_BUF_Q_LEN) {
				newfrm = get_empty_frame();
				if (newfrm) {
					newfrm->is_reserved = 2;
					newfrm->priv_data = path;
					memcpy(&newfrm->buf,
							&pframe->buf,
							sizeof(pframe->buf));
					camera_enqueue(&path->reserved_buf_queue, newfrm);
					i++;
				}
			}
		} else {
			pframe->is_reserved = 0;
			pframe->priv_data = dev;
			ret = camera_enqueue(&path->out_buf_queue, pframe);
			if (ret) {
				pr_err("dcam path %d output buffer en queue failed\n",
							path_id);
				cambuf_iommu_unmap(&pframe->buf);
				goto exit;
			}
			pr_debug("config dcam output buffer.\n");
		}
		break;

	case DCAM_PATH_CFG_COMMON:
		/*cfg_param = (struct dcam_path_cfg_param *)param;*/
		mutex_lock(&path->param_mutex);
		ret = dcam_cfg_path(dev, path, param);
		path->updated = 1;
		mutex_unlock(&path->param_mutex);
		break;
	default:
		pr_warn("unsupported command: %d\n", cfg_cmd);
		break;
	}
exit:
	return ret;
}

/* offline process frame */
static int sprd_dcam_proc_frame(
		void *dcam_handle,  void *param)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct camera_frame *pframe;

	if (!dcam_handle || !param) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	dev = (struct dcam_pipe_dev *)dcam_handle;

	pr_debug("dcam%d offline proc frame!\n", dev->idx);
	if (atomic_read(&dev->started) > 0) {
		pr_err("dcam%d started for online.\n", dev->idx);
		return -EFAULT;
	}

	pframe = (struct camera_frame *)param;
	pframe->priv_data = dev;
	ret = camera_enqueue(&dev->in_queue, pframe);
	if (ret == 0)
		complete(&dev->thread.thread_com);

	return ret;
}

static int sprd_dcam_ioctrl(void *dcam_handle,
	enum dcam_ioctrl_cmd cmd, void *param)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_mipi_info *cap;

	if (!dcam_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	dev = (struct dcam_pipe_dev *)dcam_handle;

	if (atomic_read(&dev->enable) < 1) {
		pr_err("error: dcam%d is not enable.\n", dev->idx);
		return -EFAULT;
	}

	switch (cmd) {
	case DCAM_IOCTL_CFG_CAP:
		cap = &dev->cap_info;
		memcpy(cap, param, sizeof(struct dcam_mipi_info));
		break;
	case DCAM_IOCTL_CFG_STATIS_BUF:
		dcam_cfg_statis_buffer(dev, param);
		break;
	case DCAM_IOCTL_INIT_STATIS_Q:
		init_statis_bufferq(dev);
		break;
	case DCAM_IOCTL_DEINIT_STATIS_Q:
		deinit_statis_bufferq(dev);
		break;
	default:
		pr_err("error: unknown cmd: %d\n", cmd);
		ret = -EFAULT;
		break;
	}

	return ret;
}


static int sprd_dcam_set_cb(void *dcam_handle,
		dcam_dev_callback cb, void *priv_data)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;

	if (!dcam_handle || !cb || !priv_data) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	dev->dcam_cb_func = cb;
	dev->cb_priv_data = priv_data;

	return ret;
}


static int sprd_dcam_dev_start(void *dcam_handle)
{
	int ret = 0;
	int i;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_path_desc *path;

	if (!dcam_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	if ((atomic_read(&dev->started) > 0) ||
		(atomic_read(&dev->enable) == 0)) {
		pr_err("error: dcam%d status start %d, enable %d\n",
			dev->idx, atomic_read(&dev->started),
			atomic_read(&dev->enable));
		return -EINVAL;
	}

	pr_info("dev %d start: %p\n", dev->idx, dev);
#if 0
	atomic_set(&dev->path[DCAM_PATH_AEM].user_cnt, 1);
	atomic_set(&dev->path[DCAM_PATH_AFM].user_cnt, 1);
	atomic_set(&dev->path[DCAM_PATH_AFL].user_cnt, 1);
	atomic_set(&dev->path[DCAM_PATH_HIST].user_cnt, 1);
	atomic_set(&dev->path[DCAM_PATH_BPC].user_cnt, 1);
#endif
	if (dev->is_3dnr)
		atomic_set(&dev->path[DCAM_PATH_3DNR].user_cnt, 1);
	if (dev->is_pdaf)
		atomic_set(&dev->path[DCAM_PATH_PDAF].user_cnt, 1);

	ret = set_dcam_cap_info(dev, &dev->cap_info);

	for (i  = 0; i < DCAM_PATH_MAX; i++) {
		path = &dev->path[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;
		ret = dcam_path_set_store_frm(dev, path);
		/* If output frame set failed, the path will not be started. */
		if (ret == 0) {
			atomic_set(&path->set_frm_cnt, 1);
			dcam_start_path(dev, path);
		}
	}
	reset_dcam_irq_cnt(dev->idx);
	dev->dcam_hw->ops->start(dev->dcam_hw, dev);

	atomic_set(&dev->started, 1);
	pr_info("start dcam pipe dev[%d]!\n", dev->idx);
	return ret;
}


static int sprd_dcam_dev_stop(void *dcam_handle)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;

	if (!dcam_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	if (atomic_read(&dev->started) == 0) {
		pr_info("dcam%d not start\n", dev->idx);
		return -EINVAL;
	}
	atomic_set(&dev->started, 0);
	pr_info("stop dcam %d.\n", dev->idx);

	dev->dcam_hw->ops->stop(dev->dcam_hw, dev);
	dev->dcam_hw->ops->reset(dev->dcam_hw, dev);

	trace_dcam_irq_cnt(dev->idx);

	dev->err_status = 0;
	pr_info("stop dcam pipe dev[%d]!\n", dev->idx);
	return ret;
}


static int sprd_dcam_dev_open(void *dcam_handle, void *param)
{
	int ret = 0;
	int i;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_path_desc *path;
	struct sprd_cam_hw_info *hw;

	if (!dcam_handle || !param) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	dev = (struct dcam_pipe_dev *)dcam_handle;
	if (atomic_read(&dev->enable) > 0) {
		pr_err("error: dcam %d already inited.\n", dev->idx);
		return -EFAULT;
	}

	dev->dcam_hw = (struct sprd_cam_hw_info *)param;
	hw = dev->dcam_hw;
	if (hw->ops == NULL) {
		pr_err("error: no hw ops.\n");
		return -EFAULT;
	}

	for (i  = 0; i < DCAM_PATH_MAX; i++) {
		path = &dev->path[i];
		path->path_id = i;
		atomic_set(&path->user_cnt, 0);
		atomic_set(&path->set_frm_cnt, 0);
		mutex_init(&path->param_mutex);
	}

	ret = sprd_cam_pw_on();
	ret = sprd_cam_domain_eb();

	ret = hw->ops->enable_clk(hw, NULL);
	if (ret)
		goto clk_fail;

	ret = hw->ops->reset(hw, NULL);
	if (ret)
		goto reset_fail;

	ret = hw->ops->init(hw, dev);
	if (ret)
		goto reset_fail;

	ret = dcam_create_offline_thread(dev);
	if (ret)
		goto reset_fail;

	camera_queue_init(&dev->in_queue, DCAM_IN_Q_LEN,
						0, dcam_ret_src_frame);
	camera_queue_init(&dev->proc_queue, DCAM_PROC_Q_LEN,
						0, dcam_ret_src_frame);

	atomic_set(&dev->started, 0);
	atomic_set(&dev->enable, 1);
	pr_info("open dcam pipe dev[%d]!\n", dev->idx);
	return 0;

reset_fail:
	hw->ops->disable_clk(hw, NULL);
clk_fail:
	sprd_cam_domain_disable();
	sprd_cam_pw_off();

	pr_info("fail to open dcam pipe dev[%d]!\n", dev->idx);
	return ret;

}


int sprd_dcam_dev_close(void *dcam_handle)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct sprd_cam_hw_info *hw;

	if (!dcam_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	if (atomic_read(&dev->enable) == 0) {
		pr_err("error: dcam%d not enable.", dev->idx);
		return -EFAULT;
	}

	dcam_stop_offline_thread(&dev->thread);
	camera_queue_clear(&dev->in_queue);
	camera_queue_clear(&dev->proc_queue);

	put_reserved_buffer(dev);
	unmap_statis_buffer(dev);

	hw = dev->dcam_hw;

	ret = hw->ops->reset(hw, dev);
	ret = hw->ops->deinit(hw, dev);
	ret = hw->ops->disable_clk(hw, dev);

	sprd_cam_domain_disable();
	sprd_cam_pw_off();

	atomic_set(&dev->enable, 0);
	pr_info("close dcam pipe dev[%d]!\n", dev->idx);

	return ret;
}


static struct dcam_pipe_ops dcam_ops = {
	.open = sprd_dcam_dev_open,
	.close = sprd_dcam_dev_close,
	.start = sprd_dcam_dev_start,
	.stop = sprd_dcam_dev_stop,
	.get_path = sprd_dcam_get_path,
	.put_path = sprd_dcam_put_path,
	.cfg_path = sprd_dcam_cfg_path,
	.ioctl = sprd_dcam_ioctrl,
	.proc_frame = sprd_dcam_proc_frame,
	.set_callback = sprd_dcam_set_cb,
};


struct dcam_pipe_ops *get_dcam_ops(void)
{
	return &dcam_ops;
}


void *get_dcam_pipe_dev(int idx)
{
	struct dcam_pipe_dev *dev = NULL;

	if (idx >= DCAM_ID_MAX) {
		pr_err("error dcam index: %d\n", idx);
		return NULL;
	}

	mutex_lock(&dcam_pipe_dev_mutex);
	if (s_dcam_dev[idx]) {
		pr_err("dcam %d already in use. pipe dev: %p\n",
					idx, s_dcam_dev[idx]);
		goto exit;
	}

	dev = vzalloc(sizeof(struct dcam_pipe_dev));
	if (!dev) {
		pr_err("no memory for dcam%d.\n", idx);
		goto exit;
	}

	dev->idx = idx;
	atomic_set(&dev->enable, 0);
	s_dcam_dev[idx] = dev;

exit:
	mutex_unlock(&dcam_pipe_dev_mutex);

	if (dev == NULL)
		pr_err("fail to get dcam pipe dev %d\n", idx);
	else
		pr_info("get dcam pipe dev: %p, idx:%d\n", dev, idx);

	return dev;
}


int put_dcam_pipe_dev(void *dcam_handle)
{
	int ret = 0;
	int idx;
	struct dcam_pipe_dev *dev = NULL;

	if (!dcam_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	idx = dev->idx;
	if (idx >= DCAM_ID_MAX) {
		pr_err("error dcam index: %d\n", idx);
		return -EINVAL;
	}

	mutex_lock(&dcam_pipe_dev_mutex);
	if (dev != s_dcam_dev[idx]) {
		pr_err("error: mismatched dev: %p, %p\n", dev, s_dcam_dev[idx]);
		mutex_unlock(&dcam_pipe_dev_mutex);
		return -EFAULT;
	}
	if (atomic_read(&dev->enable) > 0)
		pr_err("error: dcam %d is not disable before free.\n", idx);

	vfree(dev);
	s_dcam_dev[idx] = NULL;

	mutex_unlock(&dcam_pipe_dev_mutex);

	pr_info("put dcam pipe dev: %p, idx:%d\n", dev, idx);
	return ret;
}

