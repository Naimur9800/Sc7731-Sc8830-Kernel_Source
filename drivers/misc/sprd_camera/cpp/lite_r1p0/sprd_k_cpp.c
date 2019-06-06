/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/semaphore.h>
#include <linux/vmalloc.h>
#include <video/sprd_cpp.h>
#include "../../common/cam_common.h"
#include "cpp_reg.h"
#include "cpp_core.h"
#include "scale_drv.h"
#include "scaler_coef_gen.h"
#include "../../dcam/isp_r6p10v2/isp_buf.h"
#include "../../dcam/isp_r6p10v2/isp_int.h"
#include "../../dcam/dcam_if_r3p0v2/dcam_drv.h"
#include <sprd_k_cpp.h>

static DEFINE_SPINLOCK(isp_cpp_q_lock);
static int s_cpp_cur_path_id;
static void get_cpp_mapped_addr(struct isp_cpp_desc *isp_cpp_dev,
		struct camera_frame *frame)
{
	int i = 0;
	struct path_cpp_ion_buf *ion_buf = NULL;

	while (i < PATH_CPP_ION_BUF_NUM) {
		ion_buf = &isp_cpp_dev->ion_buf[i];
		if (frame->pfinfo.table[0] == ion_buf->sg_table) {
			frame->yaddr = ion_buf->cpp_addr.yaddr;
			frame->uaddr = ion_buf->cpp_addr.uaddr;
			break;
		}
		i++;
	}
}

static void get_isp_mapped_addr(struct isp_cpp_desc *isp_cpp_dev,
		struct camera_frame *frame)
{
	int i = 0;
	struct path_cpp_ion_buf *ion_buf = NULL;

	while (i < PATH_CPP_ION_BUF_NUM) {
		ion_buf = &isp_cpp_dev->ion_buf[i];
		if (frame->pfinfo.table[0] == ion_buf->sg_table) {
			frame->yaddr = ion_buf->isp_addr.yaddr;
			frame->uaddr = ion_buf->isp_addr.uaddr;
			break;
		}
		i++;
	}
}

static void construct_data_cpp(struct sprd_dcam_img_frm *img_frm,
		struct isp_module *module, int is_dst,
		enum isp_scl_id path_id, struct camera_frame *frame)
{
	struct isp_path_desc *path = NULL;
	struct camera_size *out_size;

	if (IS_ERR(path)) {
		pr_err("isp path error\n");
		return;
	}

	path = &module->isp_path[path_id];
	out_size = &path->out_size;
	if (is_dst) {
		img_frm->addr_phy.y =  frame->pfinfo.iova[0] + frame->yaddr;
		img_frm->addr_phy.u =  frame->pfinfo.iova[0] + frame->uaddr;
		img_frm->rect.w = out_size->w;
		img_frm->rect.h = out_size->h;
		img_frm->rect.x = 0;
		img_frm->rect.y = 0;
		img_frm->size.w = out_size->w;
		img_frm->size.h = out_size->h;
	} else {
		img_frm->size.w = frame->width;
		img_frm->size.h = frame->height;
		img_frm->addr_phy.y =  frame->yaddr;
		img_frm->addr_phy.u =  frame->uaddr;
		img_frm->rect.w = frame->width;
		img_frm->rect.h = frame->height;
		img_frm->rect.x = 0;
		img_frm->rect.y = 0;
	}
	img_frm->data_end.y_endian = SCALE_ENDIAN_BIG;
	img_frm->data_end.uv_endian = SCALE_ENDIAN_HALFBIG;
	img_frm->fmt = SCALE_YUV420;
}


static void handle_cpp_done(struct isp_pipe_dev *isp_dev)
{
	int  ret = 0;
	struct camera_frame frame;
	struct isp_path_desc *pre;
	struct isp_path_desc *vid;
	struct isp_module *module = NULL;
	struct device *dev;

	module = &isp_dev->module_info;

	sprd_cpp_get_cpp_dev(&dev);
	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];

	if (pre->path_mode == ISP_PRE_ONLINE_CPP ||
		pre->path_mode == ISP_PRE_VID_ONLINE_CPP) {
		if (!pre->valid) {
			pr_debug("handle cpp done, pre is not valid\n");
			return;
		}
		ret = isp_frame_dequeue(&pre->cpp_frame_queue, &frame);
		isp_handle_cpp_done(isp_dev, ISP_SCL_PRE, &frame);
	}

	if (vid->path_mode == ISP_VID_ONLINE_CPP ||
		vid->path_mode == ISP_PRE_VID_ONLINE_CPP) {
		if (!vid->valid) {
			pr_debug("handle cpp done, vid is not valid\n");
			return;
		}
		ret = isp_frame_dequeue(&vid->cpp_frame_queue, &frame);
		isp_handle_cpp_done(isp_dev, ISP_SCL_VID, &frame);
	}
}

static void isp_cpp_dst_frame_cfg(struct isp_module *module,
		enum isp_scl_id path_id,
		struct camera_frame *frame)
{

	struct isp_path_desc *path;
	struct camera_frame *reserved_frame;
	int use_reserve_buf = 0;
	struct device *dev = NULL;

	path = &module->isp_path[path_id];
	reserved_frame =
		&module->path_reserved_frame[path_id];

	sprd_cpp_get_cpp_dev(&dev);
	if (isp_buf_queue_read(&path->buf_queue, frame)) {
		use_reserve_buf = 1;
		pr_debug("cpp pre use reserve frame %d\n",
				use_reserve_buf);
	}
	if ((use_reserve_buf == 0) &&
			(frame->pfinfo.mfd[0] ==
			 reserved_frame->pfinfo.mfd[0])) {
		use_reserve_buf = 1;
		if (reserved_frame->pfinfo.iova[0] == 0) {
			reserved_frame->pfinfo.dev = dev;
			pfiommu_get_addr(&reserved_frame->pfinfo);
		}
	}

	if (use_reserve_buf) {
		if (reserved_frame->pfinfo.mfd[0] == 0) {
			pr_info("ISP: No need to cfg frame buffer\n");
			return;
		}
		memcpy(frame, reserved_frame, sizeof(struct camera_frame));
	} else {

		frame->pfinfo.dev = dev;
		if (pfiommu_get_addr(&frame->pfinfo))
			pr_err("ISP cpp get frame iommu map address failed!\n");

	}
}

static int isp_cpp_deinit_rest(struct isp_pipe_dev *isp_dev)
{
	struct isp_module *module = NULL;
	struct camera_frame frame;
	struct camera_frame *res_frame;
	struct isp_cpp_desc *isp_cpp_dev;
	struct isp_path_desc *pre;
	struct isp_path_desc *vid;
	unsigned int ch_id = 0;
	int idx;
	struct device *dev;

	pr_info("isp_cpp_deinit_rest\n");
	module = &isp_dev->module_info;
	idx = module->idx;
	vid = &module->isp_path[ISP_SCL_VID];
	pre = &module->isp_path[ISP_SCL_PRE];
	isp_cpp_dev  = &module->isp_cpp_dev;
	isp_buf_queue_init(&isp_cpp_dev->cpp_buf_queue);

	/* we may get into situation where we have mapped addresses in
	 * the queue, so we need to unmap them before we clear
	 * the queues
	 */


	/*we open only preview path for this flow so we just need to check
	 * for preview ch_id bits while releasing any left over buffers
	 */
	ch_id = (BIT_15 | BIT_16 | BIT_17);

	if (idx == ISP_ID_0)
		dev = &s_isp_pdev->dev;
	else
		dev = &s_isp1_pdev->dev;

	if (pre->path_mode == ISP_PRE_ONLINE_CPP ||
			pre->path_mode == ISP_PRE_VID_ONLINE_CPP) {

		res_frame = &module->path_reserved_frame[ISP_SCL_PRE];
		while (!isp_frame_dequeue(&pre->cpp_frame_queue, &frame)) {
			if (frame.pfinfo.mfd[0] != res_frame->pfinfo.mfd[0]) {
				pr_info("free pre queue\n");
				pfiommu_free_addr(&frame.pfinfo);
			}
		}
		if (res_frame->pfinfo.mfd[0] != 0 && res_frame->pfinfo.iova[0])
			pfiommu_free_addr(&res_frame->pfinfo);
		memset((void *)res_frame, 0, sizeof(struct camera_frame));
	}
	isp_buf_queue_init(&pre->buf_queue);
	if (vid->path_mode == ISP_PRE_VID_ONLINE_CPP) {
		res_frame = &module->path_reserved_frame[ISP_SCL_VID];
		while (!isp_frame_dequeue(&vid->cpp_frame_queue, &frame)) {
			if (frame.pfinfo.mfd[0] != res_frame->pfinfo.mfd[0]) {
				pr_err("free vid queue\n");
				pfiommu_free_addr(&frame.pfinfo);
			}
		}
		if (res_frame->pfinfo.mfd[0] != 0 && res_frame->pfinfo.iova[0])
			pfiommu_free_addr(&res_frame->pfinfo);
		memset((void *)res_frame, 0, sizeof(struct camera_frame));
	}

	isp_buf_queue_init(&vid->buf_queue);
	isp_frm_queue_clear(&isp_cpp_dev->isp_cpp_queue);
	isp_frm_queue_clear(&isp_cpp_dev->isp_path_queue);
	isp_frm_queue_clear(&pre->cpp_frame_queue);
	isp_frm_queue_clear(&vid->cpp_frame_queue);
	isp_cpp_clear_buf(module->idx, &module->isp_cpp_dev);
	module->isp_cpp_dev.is_valid = 0;
	return 0;
}

static void isp_cpp_err_proc(struct isp_module *module)
{
	sprd_dcam_cpp_error_notify(module->idx);
}

static int isp_cpp_thrd_handler(void *data)
{

	struct isp_pipe_dev *isp_dev;
	struct isp_cpp_desc *isp_cpp_dev = NULL;
	struct isp_module *module = NULL;
	enum isp_id idx = 0;
	struct device *dev = NULL;
	struct isp_path_desc *pre = NULL;
	struct isp_path_desc *vid = NULL;
	struct sprd_dcam_img_frm src_img = {0};
	struct sprd_dcam_img_frm pre_dst = {0};
	struct sprd_dcam_img_frm vid_dst = {0};
	int   cpp_path_id;
	int ret = 0;
	struct camera_frame cpp_frame;
	struct camera_frame pre_frame;
	struct camera_frame vid_frame;
	unsigned long flags;

	isp_dev = (struct isp_pipe_dev *)data;
	module = &isp_dev->module_info;
	idx = module->idx;
	isp_cpp_dev = &module->isp_cpp_dev;
	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];

	sprd_cpp_get_cpp_dev(&dev);

	while (1) {

		ret = wait_for_completion_interruptible(
				&isp_dev->cpp_info.cpp_thread_com);

		if (ret == -ERESTARTSYS) {
			pr_err("cpp_wait com restart");
			continue;
		}
		if (isp_dev->cpp_info.is_cpp_thread_stop) {
			pr_info("cpp thread_loop stop\n");
			break;
		}


		spin_lock_irqsave(&isp_cpp_q_lock, flags);
		if (isp_frame_dequeue(&isp_cpp_dev->isp_cpp_queue, &cpp_frame))
			pr_err("cpp frame error dq\n");
		spin_unlock_irqrestore(&isp_cpp_q_lock, flags);

		get_cpp_mapped_addr(isp_cpp_dev, &cpp_frame);
		construct_data_cpp(&src_img, module, 0, ISP_SCL_PRE,
				&cpp_frame);

		if (pre->valid && (pre->path_mode == ISP_PRE_ONLINE_CPP ||
				pre->path_mode == ISP_PRE_VID_ONLINE_CPP)) {

			isp_cpp_dst_frame_cfg(module, ISP_SCL_PRE, &pre_frame);


			construct_data_cpp(&pre_dst, module, 1, ISP_SCL_PRE,
					&pre_frame);
			if (isp_frame_enqueue(&pre->cpp_frame_queue,
						&pre_frame))
				pr_err("cpp pre frm enq error\n");
		}

		if (vid->valid && (vid->path_mode == ISP_VID_ONLINE_CPP ||
			vid->path_mode == ISP_PRE_VID_ONLINE_CPP)) {

			isp_cpp_dst_frame_cfg(module, ISP_SCL_VID, &vid_frame);
			construct_data_cpp(&vid_dst, module, 1,
					ISP_SCL_VID, &vid_frame);
			if (isp_frame_enqueue(&vid->cpp_frame_queue,
						&vid_frame)) {
				pr_err("cpp vid frm enq error\n");
			}
		}

		if (pre->valid && pre->path_mode == ISP_PRE_ONLINE_CPP) {
			cpp_path_id = CPP_SENARIO_PATH2;
			s_cpp_cur_path_id  = CPP_SENARIO_PATH2;
			isp_dev->cpp_info.is_wait_cpp  = 1;
			if (cpp_k_start_scale(&src_img, &pre_dst, NULL,
					&cpp_path_id))
				goto err;

		}

		if (vid->valid && vid->path_mode == ISP_VID_ONLINE_CPP) {
			cpp_path_id = CPP_SENARIO_PATH2;
			s_cpp_cur_path_id  = CPP_SENARIO_PATH2;
			isp_dev->cpp_info.is_wait_cpp  = 1;
			if (cpp_k_start_scale(&src_img, &vid_dst, NULL,
						&cpp_path_id))
				goto err;
		}

		if ((vid->valid && vid->path_mode == ISP_PRE_VID_ONLINE_CPP) &&
			(pre->valid && pre->path_mode ==
				 ISP_PRE_VID_ONLINE_CPP)) {
			cpp_path_id = CPP_SENARIO_PATH2_PATH3;
			s_cpp_cur_path_id  = CPP_SENARIO_PATH2_PATH3;
			isp_dev->cpp_info.is_wait_cpp  = 1;
			if (cpp_k_start_scale(&src_img, &pre_dst, &vid_dst,
					&cpp_path_id))
				goto err;
		}
		ret = cpp_k_stop_scale(cpp_path_id);
		isp_dev->cpp_info.is_wait_cpp  = 0;

		if (isp_buf_queue_write(&isp_cpp_dev->cpp_buf_queue,
					&cpp_frame))
			pr_err("cpp q buf error\n");

		if (isp_dev->cpp_info.is_cpp_thread_stop) {
			pr_info("cppline_thread_loop stop\n");
			break;
		}

		handle_cpp_done(isp_dev);
	}
	isp_cpp_deinit_rest(isp_dev);
	isp_dev->cpp_info.is_cpp_thread_stop = 0;
	pr_info("isp cpp_thread stopping thread\n");
	return 0;
err:
	isp_dev->cpp_info.isp_cpp_thread = NULL;
	isp_cpp_deinit_rest(isp_dev);
	isp_cpp_err_proc(module);
	pr_err("isp cpp_thread error\n");
	return 0;
}

void isp_cpp_get_cpp_dev(struct device **dev)
{
	sprd_cpp_get_cpp_dev(dev);
}

void isp_path_done_cpp_cfg(struct isp_pipe_dev *isp_dev,
		enum isp_scl_id path_id)
{
	struct camera_frame frame;
	struct isp_cpp_desc *isp_cpp_dev = NULL;
	enum isp_id idx = 0;
	struct isp_module *module;
	unsigned int ch_id = 0;
	struct device *dev;
	unsigned long flags;

	memset(&frame, 0, sizeof(frame));
	module = &isp_dev->module_info;
	idx = module->idx;
	isp_cpp_dev = &module->isp_cpp_dev;

	if (isp_dev->cpp_info.isp_cpp_thread == NULL)
		return;
	if (isp_frame_dequeue(&isp_cpp_dev->isp_path_queue, &frame)) {
		pr_err("%s isp_path_q dq error\n", __func__);
		return;
	}
	if (path_id == ISP_SCL_PRE)
		ch_id = (BIT_15 | BIT_16 | BIT_17);
	else if (path_id == ISP_SCL_VID)
		ch_id = (BIT_18 | BIT_19 | BIT_20);

	if (idx == ISP_ID_0)
		dev = &s_isp_pdev->dev;
	else
		dev = &s_isp1_pdev->dev;

	if (frame.pfinfo.table[0] ==
			isp_cpp_dev->cpp_reserved_frame.pfinfo.table[0]) {
		pr_err("isp path done cpp cfg use reserve frame\n");
		return;
	}
	spin_lock_irqsave(&isp_cpp_q_lock, flags);
	if (isp_frame_enqueue(&isp_cpp_dev->isp_cpp_queue, &frame)) {
		pr_err("%s isp_path_q enq error\n", __func__);
		spin_unlock_irqrestore(&isp_cpp_q_lock, flags);
		return;
	}
	spin_unlock_irqrestore(&isp_cpp_q_lock, flags);

	complete(&isp_dev->cpp_info.cpp_thread_com);
}

int isp_cpp_set_next_frame_for_path(struct isp_module *module,
		enum isp_scl_id path_id)
{
	struct camera_frame frame;
	unsigned long yuv_reg[3] = {0};
	unsigned int yuv_addr[3] = {0};
	enum isp_id idx = 0;
	struct isp_cpp_desc *isp_cpp_dev;
	unsigned int val = 0;
	unsigned int store_addr = 0;
	int ret = 0;
	struct device *dev;

	isp_cpp_dev = &module->isp_cpp_dev;
	idx = module->idx;
	memset((void *)&frame, 0, sizeof(frame));
	if (isp_buf_queue_read(&isp_cpp_dev->cpp_buf_queue, &frame)) {
		pr_err("%s cpp buf queue is error use reserve\n", __func__);
		memcpy(&frame, &isp_cpp_dev->cpp_reserved_frame,
				sizeof(struct camera_frame));
	}

	if (path_id == ISP_SCL_PRE) {
		store_addr = ISP_STORE1_BASE;
		yuv_reg[0] = ISP_STORE_PREVIEW_Y_ADDR;
		yuv_reg[1] = ISP_STORE_PREVIEW_U_ADDR;
		yuv_reg[2] = ISP_STORE_PREVIEW_V_ADDR;

	} else if (path_id == ISP_SCL_VID) {
		store_addr = ISP_STORE2_BASE;
		yuv_reg[0] = ISP_STORE_VIDEO_Y_ADDR;
		yuv_reg[1] = ISP_STORE_VIDEO_U_ADDR;
		yuv_reg[2] = ISP_STORE_VIDEO_V_ADDR;
	} else if (path_id == ISP_SCL_CAP) {
		store_addr = ISP_STORE3_BASE;
		yuv_reg[0] = ISP_STORE_CAPTURE_Y_ADDR;
		yuv_reg[1] = ISP_STORE_CAPTURE_U_ADDR;
		yuv_reg[2] = ISP_STORE_CAPTURE_V_ADDR;

	} else {
		pr_err("Not valid path index\n");
		return -EINVAL;
	}
	val = ISP_REG_RD(idx, store_addr + ISP_STORE_SLICE_SIZE);
	frame.width = val & 0xFFFF;
	frame.height = (val >> 16) & 0xFFFF;

	if (idx == ISP_ID_0)
		dev = &s_isp_pdev->dev;
	else
		dev = &s_isp1_pdev->dev;
	get_isp_mapped_addr(isp_cpp_dev, &frame);

	if (isp_frame_enqueue(&isp_cpp_dev->isp_path_queue, &frame)) {
		pr_err("%s cpp path queue enq error\n", __func__);
		return -EINVAL;
	}

	yuv_addr[0] = frame.yaddr;
	yuv_addr[1] = frame.uaddr;
	ISP_REG_WR(idx, yuv_reg[0], yuv_addr[0]);
	ISP_REG_WR(idx, yuv_reg[1], yuv_addr[1]);
	return ret;
}



static int isp_cpp_create_thread(void *param)
{
	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)param;
	char thread_name[20] = { 0 };

	if (dev == NULL) {
		pr_err("dev is NULL\n");
		return -1;
	}

	init_completion(&dev->cpp_info.cpp_thread_com);
	sprintf(thread_name, "isp%d_cpp_thread", dev->idx);
	dev->cpp_info.is_cpp_thread_stop = 0;
	dev->cpp_info.isp_cpp_thread = kthread_run(isp_cpp_thrd_handler,
					  param, thread_name);
	if (IS_ERR(dev->cpp_info.isp_cpp_thread)) {
		pr_err("isp_create_offline_thread error!\n");
		return -1;
	}

	return 0;
}


static int isp_cpp_stop_thread(void *param)
{

	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)param;
	int cnt = 0;

	if (dev == NULL) {
		pr_err("dev is NULL\n");
		return -1;
	}

	if (dev->cpp_info.isp_cpp_thread) {
		pr_info("isp cpp stop thread idx %d\n", dev->idx);
		dev->cpp_info.is_cpp_thread_stop = 1;
		complete(&dev->cpp_info.cpp_thread_com);
		if (dev->cpp_info.is_wait_cpp == 1)
			sprd_cpp_force_stop(s_cpp_cur_path_id);

		if (dev->cpp_info.is_cpp_thread_stop != 0) {
			while (cnt < 500) {
				cnt++;
				if (dev->cpp_info.is_cpp_thread_stop == 0 &&
					 dev->cpp_info.is_wait_cpp == 0) {
					break;
				}
				udelay(1000);
			}
		}
		dev->cpp_info.isp_cpp_thread = NULL;
	}

	return 0;
}

void isp_cpp_init(void *dev)
{
	struct isp_pipe_dev *isp_dev = (struct isp_pipe_dev *)dev;
	struct isp_module *module = NULL;
	struct isp_path_desc *pre;
	struct isp_path_desc *vid;

	module = &isp_dev->module_info;
	vid = &module->isp_path[ISP_SCL_VID];
	pre = &module->isp_path[ISP_SCL_PRE];
	isp_cpp_create_thread(dev);
	isp_buf_queue_init(&module->isp_cpp_dev.cpp_buf_queue);
	isp_frm_queue_clear(&module->isp_cpp_dev.isp_cpp_queue);
	isp_frm_queue_clear(&module->isp_cpp_dev.isp_path_queue);
	isp_frm_queue_clear(&pre->cpp_frame_queue);
	isp_frm_queue_clear(&vid->cpp_frame_queue);
	isp_cpp_get_buf(&module->isp_cpp_dev, module->idx);
}

void isp_cpp_deinit(void *isp_handle)
{
	struct isp_pipe_dev *isp_dev = (struct isp_pipe_dev *)isp_handle;
	struct isp_module *module = NULL;
	struct isp_cpp_desc *isp_cpp_dev;

	pr_info("isp_cpp deinit\n");
	module = &isp_dev->module_info;
	isp_cpp_dev  = &module->isp_cpp_dev;
	if (!isp_cpp_dev->is_valid)
		return;
	isp_cpp_stop_thread(isp_dev);
}
