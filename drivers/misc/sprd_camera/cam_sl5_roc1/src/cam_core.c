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
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>

#include <video/sprd_mm.h>
#include <video/sprd_isp_r8p1.h>
#include <video/sprd_img.h>


#include "cam_pw_domain.h"
#include "cam_types.h"
#include "cam_buf.h"
#include "cam_queue.h"
#include "cam_hw.h"
#include "cam_flash.h"

#include "isp_interface.h"
#include "dcam_interface.h"

#include "sprd_sensor_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_CORE: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define IMG_DEVICE_NAME			"sprd_image"
#define CAMERA_TIMEOUT			5000

#define  CAM_COUNT  CAM_ID_MAX
#define  CAM_SHARED_BUF_NUM  4
#define  CAM_FRAME_Q_LEN   16
#define  CAM_IRQ_Q_LEN        16
#define  CAM_STATIS_Q_LEN   16


/* Static Variables Declaration */
static struct camera_format output_img_fmt[] = {
	{  /*ISP_STORE_UYVY = 0 */
		.name = "4:2:2, packed, UYVY",
		.fourcc = IMG_PIX_FMT_UYVY,
		.depth = 16,
	},
	{/* ISP_STORE_YUV422_3FRAME,*/
		.name = "YUV 4:2:2, planar, (Y-Cb-Cr)",
		.fourcc = IMG_PIX_FMT_YUV422P,
		.depth = 16,
	},
	{  /*ISP_STORE_YUV420_2FRAME*/
		.name = "YUV 4:2:0 planar (Y-CbCr)",
		.fourcc = IMG_PIX_FMT_NV12,
		.depth = 12,
	},
	{ /* ISP_STORE_YVU420_2FRAME,*/
		.name = "YVU 4:2:0 planar (Y-CrCb)",
		.fourcc = IMG_PIX_FMT_NV21,
		.depth = 12,
	},
	{/*ISP_STORE_YUV420_3FRAME,*/
		.name = "YUV 4:2:0 planar (Y-Cb-Cr)",
		.fourcc = IMG_PIX_FMT_YUV420,
		.depth = 12,
	},
	{
		.name = "RawRGB",
		.fourcc = IMG_PIX_FMT_GREY,
		.depth = 8,
	},
};


struct camera_group;

/* user set information for camera module */
struct camera_uchannel {
	uint32_t sn_fmt;
	uint32_t dst_fmt;
	uint32_t scene_mode;
	uint32_t slowmotion;
	enum camera_cap_type  cap_type;
	struct sprd_img_size src_size;
	struct sprd_img_rect src_crop;
	struct sprd_img_size dst_size;

	struct dcam_regular_desc regular_desc;
};

struct camera_uinfo {
	/* cap info */
	struct sprd_img_sensor_if sensor_if;
	struct sprd_img_size sn_size;
	struct sprd_img_size sn_max_size;
	struct sprd_img_rect sn_rect;
	uint32_t capture_mode;
	uint32_t capture_skip;

	uint32_t is_4in1;
	uint32_t is_3dnr;
};

struct channel_context {
	enum cam_ch_id ch_id;
	uint32_t enable;
	uint32_t frm_base_id;
	uint32_t frm_cnt;
	atomic_t err_status;

	int32_t dcam_path_id;
	int32_t isp_path_id;

	int32_t reserved_buf_fd;

	struct camera_uchannel ch_uinfo;
	struct img_size swap_size;

	struct completion alloc_com;
	struct sprd_cam_work alloc_buf_work;

	struct completion cfg_com;
	struct sprd_cam_work cfg_work;

	/* dcam/isp shared frame buffer for full path */
	struct camera_queue share_buf_queue;
};

struct camera_module {
	uint32_t idx;
	struct camera_group *grp;

	atomic_t stream_on;
	atomic_t run_flag;
	atomic_t user_cnt;
	int attach_sensor_id;
	uint32_t iommu_enable;
	enum camera_cap_status cap_status;

	void *isp_dev_handle;
	void *dcam_dev_handle;
	uint32_t dcam_idx;

	struct camera_uinfo cam_uinfo;

	uint32_t last_channel_id;
	struct channel_context channel[CAM_CH_MAX];

	struct completion frm_com;
	struct camera_queue frm_queue;     /* frame message queue for user*/
	struct camera_queue irq_queue;     /* IRQ message queue for user*/
	struct camera_queue statis_queue;    /* statis data queue or user*/

	struct timer_list cam_timer;
	struct workqueue_struct *workqueue;
	struct sprd_cam_work work;
	struct flash_led_task *flash_task;

	/*for test below.*/
	struct camera_queue  mysrc_queue;
	struct completion test_com;

	uint32_t debug_stream_on;
};

struct camera_group {
	atomic_t camera_opened;

	spinlock_t module_lock;
	uint32_t  module_used;
	struct camera_module *module[CAM_COUNT];

	uint32_t dcam_count;	/*dts cfg dcam count*/
	uint32_t isp_count;	/*dts cfg isp count*/
	struct sprd_cam_hw_info *dcam[DCAM_ID_MAX];	/* dcam hw dev from dts */
	struct sprd_cam_hw_info *isp[2];	       /* isp hw dev from dts */

	struct miscdevice *md;
	struct platform_device *pdev;
	struct camera_queue empty_frm_q;

	struct cam_mem_dbg_info mem_dbg;
};

struct cam_ioctl_cmd {
	unsigned int cmd;
	int (*cmd_proc)(struct camera_module *module,
		       unsigned long arg);
};


/* mem debug info/ops starts */

struct cam_mem_dbg_info *g_mem_dbg;
struct camera_queue *g_empty_frm_q;

static struct isp_pipe_ops *isp_ops;
static struct dcam_pipe_ops *dcam_ops;


static int img_ioctl_stream_off(
			struct camera_module *module,
			unsigned long arg);


int mdbg_init(void)
{
	if (!g_mem_dbg)
		return -1;

	atomic_set(&g_mem_dbg->ion_alloc_cnt, 0);
	atomic_set(&g_mem_dbg->ion_kmap_cnt, 0);
	atomic_set(&g_mem_dbg->ion_dma_cnt, 0);
	atomic_set(&g_mem_dbg->iommu_map_cnt, 0);
	atomic_set(&g_mem_dbg->empty_frm_cnt, 0);
	pr_info("reset to 0\n");
	return 0;
}

int mdbg_check(void)
{
	int val[5];

	if (!g_mem_dbg) {
		pr_err("NULL g_mem_dbg\n");
		return -1;
	}

	val[0] = atomic_read(&g_mem_dbg->ion_alloc_cnt);
	val[1] = atomic_read(&g_mem_dbg->ion_kmap_cnt);
	val[2] = atomic_read(&g_mem_dbg->ion_dma_cnt);
	val[3] = atomic_read(&g_mem_dbg->iommu_map_cnt);
	val[4] = atomic_read(&g_mem_dbg->empty_frm_cnt);
	pr_info("mdbg info: %d, %d, %d, %d, %d\n",
			val[0], val[1], val[2], val[3], val[4]);
	return 0;
}
/* mem debug info/ops ends */
/*****************************************************/


static void put_k_frame(void *param)
{
	int ret = 0;
	struct camera_frame *frame;

	if (!param) {
		pr_err("error: null input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	cambuf_kunmap(&frame->buf);
	cambuf_free(&frame->buf);

	ret = put_empty_frame(frame);
}

static void camera_put_empty_frame(void *param)
{
	int ret = 0;
	struct camera_frame *frame;

	if (!param) {
		pr_err("error: null input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	cambuf_put_ionbuf(&frame->buf);
	ret = put_empty_frame(frame);
}


static void update_channel_param(struct work_struct *work)
{
	int ret = 0;
	uint32_t shift;
	struct sprd_cam_work *this_work;
	struct camera_module *module;
	struct channel_context *channel;
	struct camera_uchannel *ch_uinfo;

	struct img_size swap_size;
	struct img_size src_size;
	struct img_size crop_size;
	struct img_size dst_size;

	struct img_size dcam_src_size;
	struct img_trim dcam_src_trim;
	struct img_size dcam_dst_size;

	pr_info("enter.\n");


	this_work = container_of(work, struct sprd_cam_work, work);
	atomic_set(&this_work->status, CAM_WORK_RUNNING);

	channel = container_of(this_work, struct channel_context, cfg_work);
	module = (struct camera_module *)this_work->priv_data;
	ch_uinfo = &channel->ch_uinfo;
	pr_info("orig ch%d swap size %d , %d\n",
		channel->ch_id,  channel->swap_size.w, channel->swap_size.h);

	src_size.w = ch_uinfo->src_size.w;
	src_size.h = ch_uinfo->src_size.h;
	crop_size.w = ch_uinfo->src_crop.w;
	crop_size.h = ch_uinfo->src_crop.h;
	dst_size.w = ch_uinfo->dst_size.w;
	dst_size.h = ch_uinfo->dst_size.h;

	if ((crop_size.w <= dst_size.w) ||
		(crop_size.h <= dst_size.h) ||
		(ch_uinfo->scene_mode == DCAM_SCENE_MODE_CAPTURE) ||
		(ch_uinfo->scene_mode == DCAM_SCENE_MODE_CAPTURE_CALLBACK)) {
		shift = 0;
		swap_size = src_size;
	} else {
		shift = 1;
		while ((1 << shift) <= DCAM_SCALE_DOWN_MAX) {
			swap_size.w = crop_size.w >> shift;
			swap_size.h = crop_size.h >> shift;
			if ((swap_size.w >= dst_size.w) &&
				(swap_size.h >= dst_size.h)) {
				shift++;
				continue;
			} else {
				break;
			}
		}
		shift--;
		swap_size.w = crop_size.w >> shift;
		while (swap_size.w > ISP_MAX_LINE_WIDTH) {
			shift++;
			swap_size.w = crop_size.w >> shift;
		}

		swap_size.w = src_size.w >> shift;
		swap_size.h = src_size.h >> shift;
		pr_info("shift: %d. swap_size %d %d. dst_size %d %d\n", shift,
			swap_size.w, swap_size.h, dst_size.w, dst_size.h);
	}

	dcam_src_size = src_size;
	dcam_src_trim.start_x = 0;
	dcam_src_trim.start_y = 0;
	dcam_src_trim.size_x = dcam_src_size.w;
	dcam_src_trim.size_y = dcam_src_size.h;
	dcam_dst_size = swap_size;

	if (channel->dcam_path_id >= 0) {
		struct dcam_path_cfg_param ch_desc = {0};

		ch_desc.is_raw = (ch_uinfo->cap_type == CAM_CAP_NORMAL) ? 0 : 1;
		ch_desc.is_loose = module->cam_uinfo.sensor_if.if_spec.mipi.is_loose;
		ch_desc.frm_deci = 0;
		ch_desc.frm_skip = 0;
		ch_desc.endian.y_endian = ENDIAN_LITTLE;
		ch_desc.input_size = dcam_src_size;
		ch_desc.input_trim = dcam_src_trim;
		ch_desc.output_size = dcam_dst_size;
		ret = dcam_ops->cfg_path(module->dcam_dev_handle,
								DCAM_PATH_CFG_COMMON,
								channel->dcam_path_id, &ch_desc);
	}

	if (channel->isp_path_id >= 0) {
		struct isp_path_cfg_param ch_desc;

		ch_desc.endian.y_endian = ENDIAN_LITTLE;
		ch_desc.input_size = dcam_dst_size;
		ret = isp_ops->cfg_path(module->isp_dev_handle,
								ISP_PATH_CFG_COMMON,
								channel->isp_path_id, &ch_desc);
	}
	complete(&channel->cfg_com);
	atomic_set(&this_work->status, CAM_WORK_DONE);
	pr_info("done.\n");
}


static void alloc_shared_buffers(struct work_struct *work)
{
	int ret = 0;
	int i, count, iommu_enable;
	uint32_t width, height;
	ssize_t  size;
	struct sprd_cam_work *alloc_work;
	struct camera_module *module;
	struct camera_frame *pframe;
	struct channel_context *channel;

	pr_info("enter.\n");

	alloc_work = container_of(work, struct sprd_cam_work, work);
	channel = container_of(alloc_work, struct channel_context, alloc_buf_work);
	atomic_set(&alloc_work->status, CAM_WORK_RUNNING);

	module = (struct camera_module *)alloc_work->priv_data;
	iommu_enable = module->iommu_enable;

	pr_info("module %p, channel %d %p\n", module, channel->ch_id, channel);
	pr_info("orig ch%d swap size %d , %d\n",
		channel->ch_id,  channel->swap_size.w, channel->swap_size.h);

	if (channel->enable) {
		width = channel->swap_size.w;
		height = channel->swap_size.h;
		if (channel->ch_uinfo.sn_fmt == IMG_PIX_FMT_GREY)
			size = width * height * 10 / 8;
		else
			size = width * height * 3;
		size = ALIGN(size, CAM_BUF_ALIGN_SIZE);
		pr_info("channel %d alloc shared buffer size: %d.\n", channel->ch_id, (int)size);

		for (i = 0, count = 0; i < CAM_SHARED_BUF_NUM; i++) {
			do {
				pframe = get_empty_frame();
				if (pframe) {
					pframe->width = width;
					pframe->height = height;
					pframe->channel_id = channel->ch_id;
					pframe->is_reserved = 0;
					pframe->fid = 0;
					ret = cambuf_alloc(
							&pframe->buf, size,
							0, iommu_enable);
					if (ret) {
						pr_err("fail to alloc shared buf: %d ch %d\n",
								i, channel->ch_id);
						put_empty_frame(pframe);
						atomic_inc(&channel->err_status);
						break;
					}
					cambuf_kmap(&pframe->buf);
					ret = camera_enqueue(&channel->share_buf_queue, pframe);
					if (ret) {
						pr_err("fail to enqueue shared buf: %d ch %d\n",
								i, channel->ch_id);
						cambuf_kunmap(&pframe->buf);
						cambuf_free(&pframe->buf);
						put_empty_frame(pframe);
						/* no break here and will retry */
					} else {
						count++;
						pr_debug("alloc shared buffer %p, ch %d idx %d, cnt %d, w %d h %d size: %d\n",
									pframe, channel->ch_id, i, count, width, height, (uint32_t)size);
						break;
					}
				}
			} while (1);
		}
	}
	complete(&channel->alloc_com);
	atomic_set(&alloc_work->status, CAM_WORK_DONE);
	pr_info("done.");
}

static int set_cap_info(struct camera_module *module)
{
	int ret = 0;
	struct camera_uinfo *info = &module->cam_uinfo;
	struct sprd_img_sensor_if *sensor_if = &info->sensor_if;
	struct dcam_cap_cfg cap_info = { 0 };

	cap_info.mode = info->capture_mode;
	cap_info.frm_skip = info->capture_skip;
	cap_info.is_4in1 = info->is_4in1;
	cap_info.sensor_if = sensor_if->if_type;
	cap_info.format =  sensor_if->img_fmt;
	cap_info.pattern = sensor_if->img_ptn;
	cap_info.frm_deci = sensor_if->frm_deci;
	if (cap_info.sensor_if == DCAM_CAP_IF_CSI2) {
		cap_info.href = sensor_if->if_spec.mipi.use_href;
		cap_info.data_bits = sensor_if->if_spec.mipi.bits_per_pxl;
	}
	cap_info.cap_size.start_x = info->sn_rect.x;
	cap_info.cap_size.start_y = info->sn_rect.y;
	cap_info.cap_size.size_x = info->sn_rect.w;
	cap_info.cap_size.size_y = info->sn_rect.h;

	ret = dcam_ops->ioctl(module->dcam_dev_handle,
				DCAM_IOCTL_CFG_CAP, &cap_info);

	return ret;
}

int isp_callback(enum isp_cb_type type, void *param, void *priv_data)
{
	int ret = 0;
	uint32_t ch_id;
	struct camera_frame *pframe;
	struct camera_module *module;
	struct channel_context *channel;

	if (!param || !priv_data) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

	module = (struct camera_module *)priv_data;

	if (type == ISP_CB_DEV_ERR) {
		pr_err("ISP error happens. camera %d\n", module->idx);
		pframe = get_empty_frame();
		pframe->evt = IMG_TX_ERR;
		pframe->irq_type = CAMERA_IRQ_IMG;
		ret = camera_enqueue(&module->frm_queue, pframe);
		complete(&module->frm_com);
		return 0;
	}

	pframe = (struct camera_frame *)param;
	channel = &module->channel[pframe->channel_id];

	if ((pframe->fid & 0x3F) == 0)
		pr_debug("cam %d, module %p, frame %p, ch %d\n",
		module->idx, module, pframe, pframe->channel_id);

	switch (type) {
	case ISP_CB_RET_SRC_BUF:
		if ((atomic_read(&module->stream_on) == 0) ||
			(channel->dcam_path_id < 0)) {
			/* stream off or test_isp_only */
			pr_info("isp ret src frame %p\n", pframe);
			camera_enqueue(&channel->share_buf_queue, pframe);
		} else {
			/* return offline buffer to dcam available queue. */
			pr_debug("isp reset dcam path out %d\n", channel->dcam_path_id);
			ret = dcam_ops->cfg_path(module->dcam_dev_handle,
										DCAM_PATH_CFG_OUTPUT_BUF,
										channel->dcam_path_id, pframe);
		}
		break;

	case ISP_CB_RET_DST_BUF:
		if (atomic_read(&module->stream_on) == 1) {
			pframe->evt = IMG_TX_DONE;
			pframe->irq_type = CAMERA_IRQ_IMG;
			ch_id = pframe->channel_id;
			ret = camera_enqueue(&module->frm_queue, pframe);
			complete(&module->frm_com);
			pr_debug("ch %d get out frame: %p, evt %d\n",
					ch_id, pframe, pframe->evt);
		} else {
			cambuf_put_ionbuf(&pframe->buf);
			put_empty_frame(pframe);
		}
		break;

	default:
		pr_err("unsupported cb cmd: %d\n", type);
		break;
	}

	return ret;
}

#ifndef TEST_DCAM_ONLY
int dcam_callback(enum dcam_cb_type type, void *param, void *priv_data)
{
	int ret = 0;
	struct camera_frame *pframe;
	struct camera_module *module;
	struct channel_context *channel;

	if (!param || !priv_data) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

	module = (struct camera_module *)priv_data;
	if (type == DCAM_CB_DEV_ERR) {
		pr_err("DCAM error happens. camera %d\n", module->idx);
		dcam_ops->stop(module->dcam_dev_handle);

		pframe = get_empty_frame();
		pframe->evt = IMG_TX_ERR;
		pframe->irq_type = CAMERA_IRQ_IMG;
		ret = camera_enqueue(&module->frm_queue, pframe);
		complete(&module->frm_com);
		return 0;
	}

	pframe = (struct camera_frame *)param;
	channel = &module->channel[pframe->channel_id];

	pr_debug("module %p, cam%d ch %d.  scene %d,  cb cmd %d, frame %p\n",
		module, module->idx, pframe->channel_id,
		channel->ch_uinfo.scene_mode, type, pframe);

	switch (type) {
	case DCAM_CB_DATA_DONE:
		if ((pframe->fid & 0x3F) == 0)
		if (pframe->buf.addr_k[0]) {
			uint32_t *ptr = (uint32_t *)pframe->buf.addr_k[0];

			pr_debug("dcam path %d. outdata: %08x %08x %08x %08x\n",
					channel->dcam_path_id, ptr[0], ptr[1], ptr[2], ptr[3]);
		}
		if (channel->ch_uinfo.cap_type != CAM_CAP_NORMAL) {
			/* RAW capture or test_dcam only */
			pframe->evt = IMG_TX_DONE;
			pframe->irq_type = CAMERA_IRQ_IMG;
			ret = camera_enqueue(&module->frm_queue, pframe);
			complete(&module->frm_com);
			pr_info("get out raw frame: %p\n", pframe);

		} else if (atomic_read(&module->stream_on) == 0) {
			pr_info("stream off. put frame %p\n", pframe);
			camera_enqueue(&channel->share_buf_queue, pframe);

		} else if ((channel->ch_uinfo.scene_mode == DCAM_SCENE_MODE_PREVIEW) ||
			(channel->ch_uinfo.scene_mode == DCAM_SCENE_MODE_RECORDING) ||
			(module->cap_status == CAM_CAPTURE_START)) {

			if (channel->ch_uinfo.scene_mode == DCAM_SCENE_MODE_CAPTURE &&
				pframe->fid < (channel->frm_cnt + 3)) {
				pr_info("skip cap %d %d\n", channel->frm_cnt, pframe->fid);
				ret = -1;
			} else {
				pr_debug("proc isp path %d\n", channel->isp_path_id);
				ret = isp_ops->proc_frame(module->isp_dev_handle,
								pframe, channel->isp_path_id);
			}

			if (ret) {
				pr_debug("error: isp proc frame failed.\n");
				ret = dcam_ops->cfg_path(module->dcam_dev_handle,
									DCAM_PATH_CFG_OUTPUT_BUF,
									channel->dcam_path_id, pframe);
			} else {
				channel->frm_cnt = pframe->fid;
				if (module->cap_status == CAM_CAPTURE_START &&
					channel->ch_uinfo.scene_mode == DCAM_SCENE_MODE_CAPTURE) {
					pr_info("should stop capture here. ch %d\n", channel->ch_id);
					module->cap_status = CAM_CAPTURE_STOP;
				}
			}
		} else {
			pr_debug("reset dcam path out %d\n", channel->dcam_path_id);
			ret = dcam_ops->cfg_path(module->dcam_dev_handle,
									DCAM_PATH_CFG_OUTPUT_BUF,
									channel->dcam_path_id, pframe);
		}
		break;
	case DCAM_CB_RET_SRC_BUF:

		pr_info("dcam ret src frame %p\n", pframe);
		camera_enqueue(&channel->share_buf_queue, pframe);
		break;

	default:
		break;
	}

	return ret;
}
#else
int dcam_callback(enum dcam_cb_type type, void *param, void *priv_data)
{
	int ret = 0;
	struct camera_frame *pframe;
	struct camera_module *module;
	struct channel_context *channel;

	if (!param || !priv_data) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

	module = (struct camera_module *)priv_data;
	pframe = (struct camera_frame *)param;
	channel = &module->channel[pframe->channel_id];

	pr_debug("module %p, cam%d ch %d.  scene %d,  cb cmd %d, frame %p\n",
		module, module->idx, pframe->channel_id,
		channel->ch_uinfo.scene_mode, type, pframe);

	switch (type) {
	case DCAM_CB_DATA_DONE:
		if (atomic_read(&module->stream_on) == 0) {
			pr_info("stream off. put frame %p\n", pframe);
			pframe->evt = IMG_TX_STOP;
			pframe->irq_type = CAMERA_IRQ_IMG;
			ret = camera_enqueue(&module->frm_queue, pframe);
			complete(&module->frm_com);
		} else {
			/* Return dcam RAW frame */
			pframe->evt = IMG_TX_DONE;
			pframe->irq_type = CAMERA_IRQ_IMG;
			ret = camera_enqueue(&module->frm_queue, pframe);
			complete(&module->frm_com);
			pr_debug("get out raw frame: %p\n", pframe);
		}
		break;

	default:
		break;
	}

	return ret;
}
#endif

static int img_ioctl_isp(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;

	/*ret = sprd_isp_k_ioctl(dev->isp_dev_handle, cmd, arg);*/

	pr_debug("isp ioctl, ret %d\n", ret);
	return ret;
}


static void cam_timer_callback(unsigned long data)
{
	struct camera_module *module = (struct camera_module *)data;
	struct camera_frame *frame;
	int ret = 0;

	if (!module || atomic_read(&module->stream_on) == 0) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	if (atomic_read(&module->run_flag) == 0) {
		pr_err("CAM%d timeout.\n", module->idx);
		frame = get_empty_frame();
		if (frame) {
			frame->evt = IMG_TIMEOUT;
			frame->irq_type = CAMERA_IRQ_IMG;
			frame->irq_property = IRQ_MAX_DONE;
			ret = camera_enqueue(&module->frm_queue, frame);
			complete(&module->frm_com);
			if (ret)
				pr_err("fail to enqueue.\n");
		} else {
			/* error case, inform read thread exception happens. */
			complete(&module->frm_com);
		}
	}
}

static void sprd_init_timer(struct timer_list *cam_timer,
			unsigned long data)
{
	setup_timer(cam_timer, cam_timer_callback, data);
}

static int sprd_start_timer(struct timer_list *cam_timer,
			uint32_t time_val)
{
	int ret = 0;

	pr_debug("starting timer %ld\n", jiffies);
	ret = mod_timer(cam_timer, jiffies + msecs_to_jiffies(time_val));
	if (ret)
		pr_err("fail to start in mod_timer %d\n", ret);

	return ret;
}

static int sprd_stop_timer(struct timer_list *cam_timer)
{
	pr_debug("stop timer\n");
	del_timer_sync(cam_timer);
	return 0;
}


static int camera_module_init(struct camera_module *module)
{
	int ret = 0;
	int ch;
	struct channel_context *channel;

	pr_info("sprd_img: camera dev %d init start!\n", module->idx);

	atomic_set(&module->run_flag, 0);
	atomic_set(&module->stream_on, 0);
	atomic_set(&module->user_cnt, 0);
	init_completion(&module->frm_com);
	module->cap_status = CAM_CAPTURE_STOP;

	/*init_completion(&module->test_com);*/
	for (ch = 0; ch < CAM_CH_MAX; ch++) {
		channel = &module->channel[ch];
		channel->ch_id = ch;
		channel->dcam_path_id = -1;
		channel->isp_path_id = -1;
		init_completion(&channel->alloc_com);
		init_completion(&channel->cfg_com);
	}

	ret  = cam_create_flash_task(&module->flash_task, module->idx);
	if (unlikely(ret != 0)) {
		pr_err("fail to create flash service\n");
		ret = -EINVAL;
		goto flash_exit;
	}

	sprd_init_timer(&module->cam_timer, (unsigned long)module);
	module->attach_sensor_id = SPRD_SENSOR_ID_MAX + 1;

	pr_info("module[%d] init OK %p!\n", module->idx, module);
	return 0;

flash_exit:
	return ret;
}

static void camera_module_deinit(
			struct camera_module *module)
{
	complete(&module->frm_com);
	atomic_set(&module->run_flag, 0);
	atomic_set(&module->stream_on, 0);
	atomic_set(&module->user_cnt, 0);

	cam_destroy_flash_task(module->flash_task);
	sprd_stop_timer(&module->cam_timer);

	pr_info("module [%d] clear queue!\n", module->idx);
	camera_queue_clear(&module->frm_queue);
	camera_queue_clear(&module->irq_queue);
	camera_queue_clear(&module->statis_queue);

	pr_info("module [%d] deinit OK!\n", module->idx);
}


/*---------------  Misc interface start --------------- */

static int img_ioctl_get_time(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	struct sprd_img_time utime;
	struct timespec ts;

	ktime_get_ts(&ts);
	utime.sec = ts.tv_sec;
	utime.usec = ts.tv_nsec / NSEC_PER_USEC;
	pr_debug("get_time %d.%06d\n", utime.sec, utime.usec);

	ret = copy_to_user((void __user *)arg, &utime,
			   sizeof(struct sprd_img_time));
	if (unlikely(ret)) {
		pr_err("fail to put user info, ret %d\n", ret);
		return -EFAULT;
	}

	return 0;
}

static int img_ioctl_set_flash(
			struct camera_module *module,
			unsigned long arg)
{
	return 0;
}

static int img_ioctl_cfg_flash(
			struct camera_module *module,
			unsigned long arg)
{
	return 0;
}


static int img_ioctl_get_iommu_status(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	unsigned int iommu_enable;
	struct device	*dcam_dev;

	dcam_dev = &module->grp->pdev->dev;
	if (sprd_iommu_attach_device(dcam_dev) == 0)
		iommu_enable = 1;
	else
		iommu_enable = 0;
	module->iommu_enable = iommu_enable;

	ret = copy_to_user((void __user *)arg, &iommu_enable,
			   sizeof(unsigned char));

	if (unlikely(ret)) {
		pr_err("fail to copy to user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	pr_info("iommu_enable:%d\n", iommu_enable);
exit:
	return ret;
}


static int img_ioctl_set_statis_buf(
			struct camera_module *module,
			unsigned long arg)
{
	return 0;
}

static int img_ioctl_cfg_param(
			struct camera_module *module,
			unsigned long arg)
{
	return 0;
}


static int img_ioctl_set_function_mode(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	/*struct sprd_img_sensor_if *dst;*/
	struct sprd_img_function_mode __user *uparam;

	uparam = (struct sprd_img_function_mode __user *)arg;
	ret |= get_user(module->cam_uinfo.is_4in1, &uparam->need_4in1);
	ret |= get_user(module->cam_uinfo.is_3dnr, &uparam->need_3dnr);

	pr_info("4in1: %d, 3dnr %d\n", module->cam_uinfo.is_4in1,
				module->cam_uinfo.is_3dnr);

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}


/*---------------  Misc interface end --------------- */




/*---------------  capture(sensor input) config interface start --------------- */

static int img_ioctl_set_sensor_if(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	struct sprd_img_sensor_if *dst;

	dst = &module->cam_uinfo.sensor_if;

	ret = copy_from_user(dst,
			     (void __user *)arg,
			     sizeof(struct sprd_img_sensor_if));
	pr_info("sensor_if %d %x %x, %d.....mipi %d %d %d %d\n",
		dst->if_type, dst->img_fmt, dst->img_ptn, dst->frm_deci,
		dst->if_spec.mipi.use_href, dst->if_spec.mipi.bits_per_pxl,
		dst->if_spec.mipi.is_loose, dst->if_spec.mipi.lane_num);

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int img_ioctl_set_mode(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;

	ret = copy_from_user(&module->cam_uinfo.capture_mode,
			     (void __user *)arg,
			     sizeof(uint32_t));

	pr_info("mode %d\n", module->cam_uinfo.capture_mode);
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int img_ioctl_set_sensor_size(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	struct sprd_img_size *dst;

	dst = &module->cam_uinfo.sn_size;

	ret = copy_from_user(dst,
			     (void __user *)arg,
			     sizeof(struct sprd_img_size));

	pr_info("sensor_size %d %d\n", dst->w, dst->h);
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int img_ioctl_set_sensor_trim(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	struct sprd_img_rect *dst;

	dst = &module->cam_uinfo.sn_rect;

	ret = copy_from_user(dst,
			     (void __user *)arg,
			     sizeof(struct sprd_img_rect));
	pr_info("sensor_trim %d %d %d %d\n", dst->x, dst->y, dst->w, dst->h);
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int img_ioctl_set_sensor_max_size(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	struct sprd_img_size *dst;

	dst = &module->cam_uinfo.sn_max_size;

	ret = copy_from_user(dst,
			     (void __user *)arg,
			     sizeof(struct sprd_img_size));
	pr_info("sensor_max_size %d %d\n", dst->w, dst->h);

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int img_ioctl_set_cap_skip_num(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	uint32_t *dst;

	dst = &module->cam_uinfo.capture_skip;

	ret = copy_from_user(dst,
			     (void __user *)arg,
			     sizeof(uint32_t));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		return -EFAULT;
	}

	pr_debug("set cap skip frame %d\n", *dst);
	return 0;
}


/*---------------  capture(sensor input) config interface end --------------- */



/*---------------  Channel config interface start --------------- */

static int img_ioctl_set_output_size(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	uint32_t scene_mode;
	uint32_t cap_type;
	struct channel_context *channel = NULL;
	struct camera_uchannel *dst;
	struct sprd_img_parm __user *uparam;

	module->last_channel_id = CAM_CH_MAX;
	uparam = (struct sprd_img_parm __user *)arg;

	ret |= get_user(scene_mode, &uparam->scene_mode);
	ret |= get_user(cap_type, &uparam->need_isp_tool);

	pr_info("scene %d  cap_type %d\n", scene_mode, cap_type);
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		goto exit;
	}

	if ((cap_type == CAM_CAP_RAW_FULL) &&
		(module->channel[CAM_CH_RAW].enable == 0)) {
		channel = &module->channel[CAM_CH_RAW];
		channel->enable = 1;
	} else if ((scene_mode == DCAM_SCENE_MODE_PREVIEW)  &&
			(module->channel[CAM_CH_PRE].enable == 0)) {
		channel = &module->channel[CAM_CH_PRE];
		channel->enable = 1;
	} else if ((scene_mode == DCAM_SCENE_MODE_RECORDING)  &&
			(module->channel[CAM_CH_VID].enable == 0)) {
		channel = &module->channel[CAM_CH_VID];
		channel->enable = 1;
	} else if ((scene_mode == DCAM_SCENE_MODE_CAPTURE)  &&
			(module->channel[CAM_CH_CAP].enable == 0)) {
		channel = &module->channel[CAM_CH_CAP];
		channel->enable = 1;
	} else if (module->channel[CAM_CH_PRE].enable == 0) {
		channel = &module->channel[CAM_CH_PRE];
		channel->enable = 1;
	} else if (module->channel[CAM_CH_CAP].enable == 0) {
		channel = &module->channel[CAM_CH_CAP];
		channel->enable = 1;
	}

	if (channel == NULL) {
		pr_err("fail to get valid channel\n");
		goto exit;
	}

	pr_info("get ch %d\n", channel->ch_id);

	module->last_channel_id = channel->ch_id;

	dst = &channel->ch_uinfo;
	dst->cap_type = cap_type;
	dst->scene_mode = scene_mode;
	ret |= get_user(dst->sn_fmt, &uparam->sn_fmt);
	ret |= get_user(dst->dst_fmt, &uparam->pixel_fmt);
	ret |= get_user(dst->slowmotion, &uparam->slowmotion);
	ret |= copy_from_user(&dst->src_crop,
			&uparam->crop_rect, sizeof(struct sprd_img_rect));
	ret |= copy_from_user(&dst->dst_size,
			&uparam->dst_size, sizeof(struct sprd_img_size));
	pr_info("fmt %x %x. slw %d crop %d %d %d %d.  dst size %d %d\n",
		dst->sn_fmt, dst->dst_fmt, dst->slowmotion,
		dst->src_crop.x, dst->src_crop.y, dst->src_crop.w, dst->src_crop.h,
		dst->dst_size.w, dst->dst_size.h);

exit:
	return ret;
}


static int img_ioctl_get_ch_id(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	uint32_t shift;
	struct channel_context *channel;
	struct camera_uchannel *ch_uinfo;
	struct img_size swap_size;
	struct img_size src_size;
	struct img_size crop_size;
	struct img_size dst_size;

	if (module->last_channel_id >= CAM_CH_MAX) {
		ret = -EINVAL;
		goto exit;
	}

	channel = &module->channel[module->last_channel_id];
	ch_uinfo = &channel->ch_uinfo;

	ch_uinfo->src_size.w = module->cam_uinfo.sn_rect.w;
	ch_uinfo->src_size.h = module->cam_uinfo.sn_rect.h;
	src_size.w = ch_uinfo->src_size.w;
	src_size.h = ch_uinfo->src_size.h;
	crop_size.w = ch_uinfo->src_crop.w;
	crop_size.h = ch_uinfo->src_crop.h;
	dst_size.w = ch_uinfo->dst_size.w;
	dst_size.h = ch_uinfo->dst_size.h;

	pr_info("stream on %d. cnt %d\n",
		atomic_read(&module->stream_on),
		module->debug_stream_on);
	pr_info("ch %d. src %d %d, crop %d %d , dst %d %d\n",
		channel->ch_id, src_size.w, src_size.h,
		crop_size.w, crop_size.h, dst_size.w, dst_size.h);

	if ((crop_size.w <= dst_size.w) ||
		(crop_size.h <= dst_size.h) ||
		(ch_uinfo->scene_mode == DCAM_SCENE_MODE_CAPTURE) ||
		(ch_uinfo->scene_mode == DCAM_SCENE_MODE_CAPTURE_CALLBACK)) {
		shift = 0;
		swap_size = src_size;
	} else {
		shift = 1;
		while ((1 << shift) <= DCAM_SCALE_DOWN_MAX) {
			swap_size.w = crop_size.w >> shift;
			swap_size.h = crop_size.h >> shift;
			if ((swap_size.w >= dst_size.w) &&
				(swap_size.h >= dst_size.h)) {
				shift++;
				continue;
			} else {
				break;
			}
		}
		shift--;
		swap_size.w = crop_size.w >> shift;
		while (swap_size.w > ISP_MAX_LINE_WIDTH) {
			shift++;
			swap_size.w = crop_size.w >> shift;
		}

		swap_size.w = src_size.w >> shift;
		swap_size.h = src_size.h >> shift;
		pr_info("shift: %d. swap_size %d %d. dst_size %d %d\n", shift,
			swap_size.w, swap_size.h, dst_size.w, dst_size.h);
	}
	channel->swap_size = swap_size;

	if (dcam_ops->get_path) {
		struct cam_channel_desc ch_desc;

		ch_desc.attach_cam_id = module->idx;
		ch_desc.in_fmt = ch_uinfo->sn_fmt;
		if (ch_uinfo->scene_mode == DCAM_SCENE_MODE_PREVIEW)
			ch_desc.prop = CH_PROP_PRE;
		else if (ch_uinfo->scene_mode == DCAM_SCENE_MODE_RECORDING)
			ch_desc.prop = CH_PROP_VIDEO;
		else
			ch_desc.prop = CH_PROP_CAP;
		ch_desc.input_size = src_size;
		ch_desc.input_trim.start_x = 0;
		ch_desc.input_trim.start_y = 0;
		ch_desc.input_trim.size_x = src_size.w;
		ch_desc.input_trim.size_y = src_size.h;
		ch_desc.output_size = swap_size;
		ret = dcam_ops->get_path(module->dcam_dev_handle, &ch_desc);
		if (ret < 0) {
			pr_err("fail to get dcam path for size %d %d.\n",
					swap_size.w, swap_size.h);
		} else {
			channel->dcam_path_id = (int32_t)ret;
			pr_info("get dcam path : %d\n", channel->dcam_path_id);
		}
	}

	if (isp_ops->get_path) {
		struct cam_channel_desc ch_desc;

		ch_desc.attach_cam_id = module->idx;
		ch_desc.in_fmt = ch_uinfo->sn_fmt;
		if (ch_uinfo->scene_mode == DCAM_SCENE_MODE_PREVIEW) {
			ch_desc.prop = CH_PROP_PRE;
			ch_desc.shared = 1;
		} else if (ch_uinfo->scene_mode == DCAM_SCENE_MODE_RECORDING) {
			ch_desc.prop = CH_PROP_VIDEO;
			ch_desc.shared = 1;
		} else {
			ch_desc.prop = CH_PROP_CAP;
			ch_desc.shared = 0;
		}

		ch_desc.input_size = swap_size;
		ch_desc.input_trim.start_x = ch_uinfo->src_crop.x >> shift;
		ch_desc.input_trim.start_y = ch_uinfo->src_crop.y >> shift;
		ch_desc.input_trim.size_x = ch_uinfo->src_crop.w >> shift;
		ch_desc.input_trim.size_y = ch_uinfo->src_crop.h >> shift;
		ch_desc.output_size = dst_size;
		ret = isp_ops->get_path(module->isp_dev_handle, &ch_desc);
		if (ret < 0) {
			pr_err("fail to get isp path for size %d %d.\n",
					dst_size.w, dst_size.h);
		} else {
			channel->isp_path_id = (int32_t)ret;
			pr_info("get isp path : %d\n", channel->isp_path_id);
			ret = isp_ops->set_callback(module->isp_dev_handle,
						(uint32_t)channel->isp_path_id,
						isp_callback, module);
			if (ret)
				pr_err("fail to set cam%d callback for isp.\n",
						module->idx);
		}
	}
	camera_queue_init(&channel->share_buf_queue,
			CAM_SHARED_BUF_NUM, 0, put_k_frame);

	ret = copy_to_user((void __user *)arg, &module->last_channel_id,
			   sizeof(uint32_t));
	if (unlikely(ret))
		pr_err("fail to copy to user. ret %d\n", ret);

exit:
	return ret;
}


static int img_ioctl_dcam_path_size(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	struct sprd_dcam_path_size param;

	ret = copy_from_user(
			&param, (void __user *)arg,
			sizeof(struct sprd_dcam_path_size));
	param.dcam_out_w = param.dcam_in_w;
	param.dcam_out_h = param.dcam_in_h;

	if (atomic_read(&module->run_flag) == 0)
		pr_info("in %d  %d. pre %d %d, vid %d, %d\n",
			param.dcam_in_w, param.dcam_in_h,
			param.pre_dst_w, param.pre_dst_h,
			param.vid_dst_w, param.vid_dst_h);

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

	ret = copy_to_user(
			(void __user *)arg, &param,
			sizeof(struct sprd_dcam_path_size));

exit:
	return ret;
}



static int img_ioctl_set_shrink(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	uint32_t channel_id;
	struct sprd_img_parm __user *uparam;

	uparam = (struct sprd_img_parm __user *)arg;

	ret = get_user(channel_id, &uparam->channel_id);
	if (ret  == 0 && channel_id < CAM_CH_MAX) {
		ret = copy_from_user(
				&module->channel[channel_id].ch_uinfo.regular_desc,
				(void __user *)&uparam->regular_desc,
				sizeof(struct dcam_regular_desc));
	}

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}



static int img_ioctl_pdaf_control(
			struct camera_module *module,
			unsigned long arg)
{
	return 0;
}

static int img_ioctl_path_frm_deci(
			struct camera_module *module,
			unsigned long arg)
{
	return 0;
}

static int img_ioctl_set_path_skip_num(
			struct camera_module *module,
			unsigned long arg)
{
	return 0;
}



static int img_ioctl_set_crop(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	uint32_t channel_id;
	struct camera_uchannel *dst;
	struct sprd_img_parm __user *uparam;

	uparam = (struct sprd_img_parm __user *)arg;

	ret = get_user(channel_id, &uparam->channel_id);
	pr_info("ch %d\n", channel_id);
	if (ret  == 0 && channel_id < CAM_CH_MAX) {
		dst = &module->channel[channel_id].ch_uinfo;
		ret = copy_from_user(
				&dst->src_crop,
				(void __user *)&uparam->crop_rect,
				sizeof(struct sprd_img_rect));

		pr_info("crop %d %d %d %d.\n",
				dst->src_crop.x, dst->src_crop.y,
				dst->src_crop.w, dst->src_crop.h);
		if (unlikely(ret)) {
			pr_err("fail to copy from user, ret %d\n", ret);
			goto exit;
		}
	}

exit:
	return ret;
}



static int img_ioctl_get_fmt(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	struct sprd_img_get_fmt fmt_desc;
	struct camera_format *fmt = NULL;

	ret = copy_from_user(&fmt_desc, (void __user *)arg,
			     sizeof(struct sprd_img_get_fmt));
	if (unlikely(ret)) {
		pr_err("fail to copy from user ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	if (unlikely(fmt_desc.index >= ARRAY_SIZE(output_img_fmt))) {
		pr_err("fail to get valid index > arrar size\n");
		ret = -EINVAL;
		goto exit;
	}

	fmt = &output_img_fmt[fmt_desc.index];
	fmt_desc.fmt = fmt->fourcc;

	ret = copy_to_user((void __user *)arg,
			   &fmt_desc,
			   sizeof(struct sprd_img_get_fmt));
	if (unlikely(ret)) {
		pr_err("fail to put user info, GET_FMT, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
exit:
	return ret;
}


static int img_ioctl_check_fmt(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	uint32_t channel_id;
	struct sprd_img_format img_format;
	struct channel_context *channel;

	pr_debug("check fmt\n");
	ret = copy_from_user(&img_format,
				(void __user *)arg,
				sizeof(struct sprd_img_format));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to get img_format\n");
		goto exit;
	}

	channel_id = img_format.channel_id;
	channel = &module->channel[channel_id];

	/* to do: check & set channel format / param */
	/* if stream on , may update channel size for zoom */

	pr_info("ch %d\n", channel_id);

	channel->cfg_work.priv_data = module;
	atomic_set(&channel->cfg_work.status, CAM_WORK_PENDING);
	INIT_WORK(&channel->cfg_work.work, update_channel_param);
	queue_work(module->workqueue, &channel->cfg_work.work);

	if ((atomic_read(&module->stream_on) == 0) &&
		(channel->ch_uinfo.cap_type == CAM_CAP_NORMAL)) {
		/* This is the last step for channel, should cfg/alloc buffer here. */
		channel->alloc_buf_work.priv_data = module;
		atomic_set(&channel->alloc_buf_work.status, CAM_WORK_PENDING);
		INIT_WORK(&channel->alloc_buf_work.work, alloc_shared_buffers);
		pr_info("module %p, channel %d %p\n", module, channel->ch_id, channel);
		queue_work(module->workqueue, &channel->alloc_buf_work.work);
	}


	img_format.need_binning = 0;
	ret = copy_to_user((void __user *)arg,
			&img_format,
			sizeof(struct sprd_img_format));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy to user\n");
		goto exit;
	}
exit:
	return ret;
}

static int img_ioctl_set_frm_id_base(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;

	return ret;
}


static int img_ioctl_set_frame_addr(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	uint32_t i;
	struct sprd_img_parm param;
	struct channel_context *ch;
	struct camera_frame *pframe;

	ret = copy_from_user(&param, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to copy from user. ret %d\n", ret);
		return -EFAULT;
	}
	if ((param.channel_id >= CAM_CH_MAX) ||
		(param.buffer_count == 0) ||
		(module->channel[param.channel_id].enable == 0)) {
		pr_err("error: invalid channel id %d. buf cnt %d\n",
				param.channel_id,  param.buffer_count);
		return -EFAULT;
	}
	pr_debug("ch %d, buffer_count %d\n", param.channel_id, param.buffer_count);

	ch = &module->channel[param.channel_id];
	for (i = 0; i < param.buffer_count; i++) {
		pframe = get_empty_frame();
		if (pframe == NULL) {
			pr_err("fail to get empty frame node\n");
			ret = -EFAULT;
			break;
		}
		pframe->buf.type = CAM_BUF_USER;
		pframe->buf.mfd[0] = param.fd_array[i];
		pframe->channel_id = ch->ch_id;
		pr_debug("frame %p,  mfd %d, reserved %d\n",
				pframe, pframe->buf.mfd[0], param.is_reserved_buf);

		ret = cambuf_get_ionbuf(&pframe->buf);
		if (ret) {
			put_empty_frame(pframe);
			ret = -EFAULT;
			break;
		}

		if (param.is_reserved_buf) {
			ch->reserved_buf_fd = pframe->buf.mfd[0];
			ret = dcam_ops->cfg_path(module->dcam_dev_handle,
								DCAM_PATH_CFG_OUTPUT_RESERVED_BUF,
								ch->dcam_path_id, pframe);
		} else if (ch->ch_uinfo.cap_type == CAM_CAP_NORMAL) {
#ifndef TEST_DCAM_ONLY
			ret = isp_ops->cfg_path(module->isp_dev_handle,
								ISP_PATH_CFG_OUTPUT_BUF,
								ch->isp_path_id, pframe);
#else
			ret = dcam_ops->cfg_path(module->dcam_dev_handle,
								DCAM_PATH_CFG_OUTPUT_BUF,
								ch->dcam_path_id, pframe);
#endif
		} else {
			ret = dcam_ops->cfg_path(module->dcam_dev_handle,
								DCAM_PATH_CFG_OUTPUT_BUF,
								ch->dcam_path_id, pframe);
		}

		if (ret) {
			pr_err("fail to set output buffer for ch%d.\n", ch->ch_id);
			cambuf_put_ionbuf(&pframe->buf);
			put_empty_frame(pframe);
			ret = -EFAULT;
			break;
		}
	}

	return ret;
}


/*---------------  Channel config interface end --------------- */




/*--------------- Core controlling interface start --------------- */

static int img_ioctl_get_cam_res(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	int dcam_idx;
	struct sprd_img_res res = {0};
	struct camera_group *grp = module->grp;
	void *dcam = NULL;
	void *isp = NULL;

	ret = copy_from_user(&res, (void __user *)arg,
			     sizeof(struct sprd_img_res));
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	dcam_idx = -1;
#ifdef TEST_ON_HAPS
	if (res.sensor_id == 0)
		dcam_idx = 0;
	else
		dcam_idx = 1;
#else
	if (res.sensor_id < SPRD_SENSOR_ID_MAX) {
		/* get a preferred dcam dev */
		dcam_idx = sprd_sensor_find_dcam_id(res.sensor_id);
	}
#endif
	if (dcam_idx == -1) {
		pr_err("fail to get dcam id for sensor: %d\n", res.sensor_id);
		return -EFAULT;
	}

	dcam = module->dcam_dev_handle;
	if (dcam == NULL) {
		dcam = get_dcam_pipe_dev(dcam_idx);
		if (IS_ERR_OR_NULL(dcam)) {
			pr_err("fail to get dcam%d\n", dcam_idx);
			ret = -EINVAL;
			goto no_dcam;
		}
		module->dcam_dev_handle = dcam;
		module->dcam_idx = dcam_idx;
	}

	ret = dcam_ops->open(dcam, grp->dcam[dcam_idx]);
	if (ret) {
		put_dcam_pipe_dev(dcam);
		ret = -EINVAL;
		goto dcam_fail;
	}

	ret = dcam_ops->set_callback(dcam, dcam_callback, module);
	if (ret) {
		pr_err("fail to set cam%d callback for dcam.\n", dcam_idx);
		goto dcam_cb_fail;
	}

	isp = module->isp_dev_handle;
	if (isp == NULL) {
		isp = get_isp_pipe_dev();
		if (IS_ERR_OR_NULL(isp)) {
			pr_err("fail to get isp\n");
			module->isp_dev_handle = NULL;
			ret = -EINVAL;
			goto no_isp;
		}
		module->isp_dev_handle = isp;
	}

	ret = isp_ops->open(isp, grp->isp[0]);
	if (ret) {
		pr_err("faile to enable isp module.\n");
		ret = -EINVAL;
		goto isp_fail;
	}


	module->attach_sensor_id = res.sensor_id;
	module->workqueue = create_workqueue("sprd_camera_module");
	if (!module->workqueue) {
		pr_err("fail to create camera dev wq\n");
		ret = -EINVAL;
		goto wq_fail;
	}
	atomic_set(&module->work.status, CAM_WORK_DONE);


	if (dcam_idx == 0)
		res.flag = DCAM_RES_DCAM0_CAP | DCAM_RES_DCAM0_PATH;
	else if (dcam_idx == 1)
		res.flag = DCAM_RES_DCAM1_CAP | DCAM_RES_DCAM1_PATH;
	else
		res.flag = DCAM_RES_DCAM2_CAP | DCAM_RES_DCAM2_PATH;


	pr_debug("sensor %d w %u h %u, cam [%d]\n",
		res.sensor_id, res.width, res.height, module->idx);

	pr_info("get camera res for sensor %d res %x done.",
					res.sensor_id, res.flag);

	ret = copy_to_user((void __user *)arg, &res,
			   sizeof(struct sprd_img_res));
	if (ret) {
		pr_err("fail to copy_to_user\n");
		ret = -EFAULT;
		goto copy_fail;
	}
	atomic_set(&module->user_cnt, 1);

	module->debug_stream_on = 0;
	return 0;

copy_fail:
	destroy_workqueue(module->workqueue);
	module->workqueue  = NULL;

wq_fail:
	isp_ops->close(isp);

isp_fail:
	put_isp_pipe_dev(isp);
	module->isp_dev_handle = NULL;

no_isp:
dcam_cb_fail:
	dcam_ops->close(dcam);

dcam_fail:
	put_dcam_pipe_dev(dcam);
	module->dcam_dev_handle = NULL;
no_dcam:
	pr_err("fail to get camera res for sensor: %d\n", res.sensor_id);

	return ret;
}


static int img_ioctl_put_cam_res(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	uint32_t idx;
	struct sprd_img_res res = {0};

	ret = copy_from_user(&res, (void __user *)arg,
			     sizeof(struct sprd_img_res));
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	idx = module->idx;

	if (module->attach_sensor_id != res.sensor_id) {
		pr_warn("warn: mismatch sensor id: %d, %d for cam %d\n",
				module->attach_sensor_id, res.sensor_id, module->idx);
	}

	destroy_workqueue(module->workqueue);
	module->workqueue  = NULL;
	module->attach_sensor_id = -1;

	ret = img_ioctl_stream_off(module, arg);

	if (module->dcam_dev_handle) {
		dcam_ops->close(module->dcam_dev_handle);
		put_dcam_pipe_dev(module->dcam_dev_handle);
		module->dcam_dev_handle = NULL;
	}
	if (module->isp_dev_handle) {
		isp_ops->close(module->isp_dev_handle);
		put_isp_pipe_dev(module->isp_dev_handle);
		module->isp_dev_handle = NULL;
	}

	atomic_set(&module->user_cnt, 0);

	pr_debug("sensor %d w %u h %u, cam [%d]\n",
		res.sensor_id, res.width, res.height, module->idx);

	pr_info("put camera res for sensor %d res %x done.",
					res.sensor_id, res.flag);
	ret = copy_to_user((void __user *)arg, &res,
			sizeof(struct sprd_img_res));
	if (ret)
		pr_err("fail to copy_to_user!\n");

	return ret;
}



static int img_ioctl_stream_on(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	uint32_t i;

	if (atomic_read(&module->stream_on) > 0) {
		pr_err("cam%d already stream on.\n", module->idx);
		return -EFAULT;
	}
	atomic_set(&module->run_flag, 0);

	camera_queue_init(&module->frm_queue,
		CAM_FRAME_Q_LEN, 0, camera_put_empty_frame);
	camera_queue_init(&module->irq_queue,
		CAM_IRQ_Q_LEN, 0, camera_put_empty_frame);
	camera_queue_init(&module->statis_queue,
		CAM_STATIS_Q_LEN, 0, camera_put_empty_frame);

	ret = set_cap_info(module);
	if (ret) {
		pr_err("fail to set capture info.\n");
		goto exit;
	}

	ret = dcam_ops->ioctl(module->dcam_dev_handle,
				DCAM_IOCTL_INIT_STATIS_Q, NULL);

	for (i = 0;  i < CAM_CH_MAX; i++) {
		struct channel_context *ch = &module->channel[i];

		if (ch->enable) {
			ret = wait_for_completion_interruptible(&ch->cfg_com);
			if (ret != 0) {
				pr_err("error: config channel/path param work %d\n", ret);
				goto exit;
			}

			pr_info("update channel %d done. cap type %d, dcam %d, isp %d\n",
					ch->ch_id, ch->ch_uinfo.cap_type,
					ch->dcam_path_id, ch->isp_path_id);
			if (ch->ch_uinfo.cap_type == CAM_CAP_NORMAL) {
				ret = wait_for_completion_interruptible(&ch->alloc_com);
				if (ret != 0) {
					pr_err("error: config channel/path param work %d\n", ret);
					goto exit;
				}
				pr_info("alloc buffer done.\n");

#ifndef TEST_DCAM_ONLY
				/* set shared frame for dcam output */
				while (1) {
					struct camera_frame *pframe = NULL;

					pframe = camera_dequeue(&ch->share_buf_queue);
					if (pframe == NULL)
						break;

					ret = dcam_ops->cfg_path(module->dcam_dev_handle,
										DCAM_PATH_CFG_OUTPUT_BUF,
										ch->dcam_path_id, pframe);
					if (ret) {
						pr_err("fail to config dcam output buffer.\n");
						camera_enqueue(&ch->share_buf_queue, pframe);
						ret = -EINVAL;
						goto exit;
					}
				}
#endif
			}
		}
	}

	pr_info("wait for wq done.\n");
	flush_workqueue(module->workqueue);

	ret = dcam_ops->start(module->dcam_dev_handle);

	atomic_set(&module->stream_on, 1);
	module->debug_stream_on++;

	ret = sprd_start_timer(&module->cam_timer, CAMERA_TIMEOUT);
exit:
	return ret;
}


static int img_ioctl_stream_off(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	uint32_t i;

	pr_info("cam %d stream off.\n", module->idx);
	if (atomic_read(&module->stream_on) > 0) {
		ret = dcam_ops->stop(module->dcam_dev_handle);
		sprd_stop_timer(&module->cam_timer);
	}
	atomic_set(&module->stream_on, 0);

	if (module->dcam_dev_handle)
		ret = dcam_ops->ioctl(module->dcam_dev_handle,
				DCAM_IOCTL_DEINIT_STATIS_Q, NULL);

	pr_info("clear path.\n");
	for (i = 0;  i < CAM_CH_MAX; i++) {
		struct channel_context *ch = &module->channel[i];

		if (ch->enable) {
			pr_info("clear ch %d, dcam path %d, isp path %d\n",
					ch->ch_id,
					ch->dcam_path_id,
					ch->isp_path_id);
			if (ch->dcam_path_id >= 0)
				dcam_ops->put_path(module->dcam_dev_handle,
										ch->dcam_path_id);
			if (ch->isp_path_id >= 0)
				isp_ops->put_path(module->isp_dev_handle,
										ch->isp_path_id);
			camera_queue_clear(&ch->share_buf_queue);
			ch->enable = 0;
		}
	}
	atomic_set(&module->run_flag, 0);

	camera_queue_clear(&module->frm_queue);
	camera_queue_clear(&module->irq_queue);
	camera_queue_clear(&module->statis_queue);

	ret = mdbg_check();
	pr_info("cam %d stream off done.\n", module->idx);

	return ret;
}


static int img_ioctl_start_capture(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	 struct sprd_img_capture_param param;

	ret = copy_from_user(&param, (void __user *)arg,
			sizeof(struct sprd_img_capture_param));
	if (ret) {
		pr_err("fail to copy user info\n");
		goto exit;
	}

	if (atomic_read(&module->stream_on) == 0) {
		pr_err("fail to start capture for stream not on.\n");
		ret = -EINVAL;
		goto exit;
	}

	if (param.type != DCAM_CAPTURE_STOP)
		module->cap_status = CAM_CAPTURE_START;

	pr_info("cam %d start capture.\n", module->idx);
exit:
	return ret;
}


static int img_ioctl_stop_capture(
			struct camera_module *module,
			unsigned long arg)
{
	module->cap_status = CAM_CAPTURE_STOP;
	pr_info("cam %d stop capture.\n", module->idx);
	return 0;
}


static int img_ioctl_raw_cap(
			struct camera_module *module,
			unsigned long arg)
{
	return 0;
}
/*--------------- Core controlling interface end --------------- */



/* for user test isp/dcam/fd directly. */
static int test_dcam(struct camera_module *module,
			struct dev_test_info *test_info)
{
	int ret = 0;
	int dcam_idx = 0;
	int iommu_enable;
	int i;
	size_t size;
	struct channel_context *channel;
	struct cam_channel_desc *ch_desc, desc = { 0 };
	struct camera_frame *pframe;
	struct camera_frame *user_frame, *user_frame1;
	void *dcam;

	if (test_info->dev == 1)
		dcam_idx = 0;
	else
		dcam_idx = 1;

	pr_info("test dcam %d\n", dcam_idx);

	/* test dcam only */
	dcam = module->dcam_dev_handle;
	if (dcam == NULL) {
		dcam = get_dcam_pipe_dev(dcam_idx);
		if (IS_ERR_OR_NULL(dcam)) {
			pr_err("fail to get dcam%d\n", dcam_idx);
			ret = -EINVAL;
			goto exit;
		}
		ret = dcam_ops->open(dcam, module->grp->dcam[dcam_idx]);
		if (ret) {
			put_dcam_pipe_dev(dcam);
			goto exit;
		}

		ret = dcam_ops->set_callback(dcam, dcam_callback, module);
		if (ret) {
			pr_err("fail to set cam%d callback for dcam.\n", dcam_idx);
			dcam_ops->close(dcam);
			put_dcam_pipe_dev(dcam);
			goto exit;
		}
		pr_info("dcam module open done: %p\n", dcam);
		module->dcam_dev_handle = dcam;
	}

	for (i = 0; i < 2; i++) {
		channel = &module->channel[i];
		channel->ch_uinfo.cap_type = CAM_CAP_RAW_FULL;
		if (dcam_ops->get_path) {
			ch_desc = &desc;
			ch_desc->attach_cam_id = module->idx;
			ch_desc->in_fmt = test_info->in_fmt;
			ch_desc->prop = i;
			ch_desc->input_size.w = test_info->input_size.width;
			ch_desc->input_size.h = test_info->input_size.height;
			ch_desc->input_trim.size_x = ch_desc->input_size.w;
			ch_desc->input_trim.size_y = ch_desc->input_size.h;
			ch_desc->output_size.w = test_info->output_size.width;
			ch_desc->output_size.h = test_info->output_size.height;
			ret = dcam_ops->get_path(module->dcam_dev_handle, ch_desc);
			if (ret < 0) {
				pr_err("fail to get dcam path for cam%d %d size %d %d.\n",
					module->idx, ch_desc->attach_cam_id,
					ch_desc->output_size.w, ch_desc->output_size.h);
				goto exit;
			} else {
				channel->dcam_path_id = (int32_t)ret;
				pr_info("cam %d get dcam path : %d\n",
						module->idx, channel->dcam_path_id);
			}
		}

		if (channel->dcam_path_id >= 0) {
			struct dcam_path_cfg_param dcam_ch_desc = {0};

			dcam_ch_desc.is_raw = 0;
			dcam_ch_desc.is_loose = 0;
			dcam_ch_desc.frm_deci = 0;
			dcam_ch_desc.frm_skip = 0;
			dcam_ch_desc.endian.y_endian = ENDIAN_LITTLE;
			dcam_ch_desc.input_size = ch_desc->input_size;
			dcam_ch_desc.input_trim = ch_desc->input_trim;
			dcam_ch_desc.output_size = ch_desc->output_size;
			ret = dcam_ops->cfg_path(module->dcam_dev_handle,
									DCAM_PATH_CFG_COMMON,
									channel->dcam_path_id, &dcam_ch_desc);
		}
	}

	iommu_enable = test_info->iommu_en;
	pframe = get_empty_frame();
	if (pframe) {
		pframe->is_reserved = 0;
		pframe->fid = 0;
		size = ch_desc->input_size.w + 4;
		size *= ch_desc->input_size.h * 10 / 8;
		ret = cambuf_alloc(
				&pframe->buf, size,
				0, iommu_enable);
		if (ret) {
			pr_err("fail to alloc buffer.\n");
			put_empty_frame(pframe);
			goto nobuf;
		}
		cambuf_kmap(&pframe->buf);
	}

	user_frame = get_empty_frame();
	if (user_frame) {
		user_frame->buf.type = CAM_BUF_USER;
		user_frame->buf.mfd[0] = test_info->inbuf_fd;
		user_frame->buf.addr_k[0] = test_info->inbuf_kaddr[1];
		user_frame->buf.addr_k[0] <<= 32;
		user_frame->buf.addr_k[0] |= test_info->inbuf_kaddr[0];
		cambuf_get_ionbuf(&user_frame->buf);
		pr_info("src buf kaddr %p.\n",
				(void *)user_frame->buf.addr_k[0]);
	}
	pr_info("copy source image.\n");
	memcpy((void *)pframe->buf.addr_k[0],
			(void *)user_frame->buf.addr_k[0], size);
	user_frame1 = user_frame;

	user_frame = get_empty_frame();
	if (user_frame) {
		user_frame->buf.type = CAM_BUF_USER;
		user_frame->buf.mfd[0] = test_info->outbuf_fd;
		cambuf_get_ionbuf(&user_frame->buf);
	}
	user_frame->channel_id = module->channel[0].ch_id;
	user_frame1->channel_id = module->channel[1].ch_id;
	ret = dcam_ops->cfg_path(module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_BUF,
				module->channel[0].dcam_path_id, user_frame);
	ret = dcam_ops->cfg_path(module->dcam_dev_handle,
				DCAM_PATH_CFG_OUTPUT_BUF,
				module->channel[1].dcam_path_id, user_frame1);

	pframe->channel_id = module->channel[0].ch_id;
	camera_queue_init(&module->channel[0].share_buf_queue,
			CAM_SHARED_BUF_NUM, 0, put_k_frame);
	camera_queue_init(&module->frm_queue,
			CAM_FRAME_Q_LEN, 0, camera_put_empty_frame);

	atomic_set(&module->stream_on, 1);

	ret = dcam_ops->proc_frame(module->dcam_dev_handle, pframe);

	pr_info("start wait for DCAM callback.\n");
	ret = wait_for_completion_interruptible(&module->frm_com);
	pr_info("wait done.\n");
	user_frame = camera_dequeue(&module->frm_queue);

	pr_info("start wait for DCAM callback.\n");
	ret = wait_for_completion_interruptible(&module->frm_com);
	pr_info("wait done.\n");
	user_frame1 = camera_dequeue(&module->frm_queue);

	cambuf_put_ionbuf(&user_frame->buf);
	put_empty_frame(user_frame);

	cambuf_put_ionbuf(&user_frame1->buf);
	put_empty_frame(user_frame1);

	camera_queue_clear(&module->frm_queue);

	atomic_set(&module->stream_on, 0);

nobuf:
	dcam_ops->put_path(module->dcam_dev_handle, channel[0].dcam_path_id);
	dcam_ops->put_path(module->dcam_dev_handle, channel[1].dcam_path_id);
exit:
	return ret;
}

static int test_isp(struct camera_module *module,
			struct dev_test_info *test_info)
{
	int ret = 0;
	int iommu_enable;
	size_t size;
	struct channel_context *channel;
	struct cam_channel_desc *ch_desc, desc = { 0 };
	struct camera_frame *pframe;
	struct camera_frame *user_frame;
	void *isp;

	isp = module->isp_dev_handle;
	if (isp == NULL) {
		isp = get_isp_pipe_dev();
		if (IS_ERR_OR_NULL(isp)) {
			pr_err("fail to get isp\n");
			module->isp_dev_handle = NULL;
			ret = -EINVAL;
			goto exit;
		}

		ret = isp_ops->open(isp, module->grp->isp[0]);
		if (ret) {
			pr_err("faile to enable isp module.\n");
			put_isp_pipe_dev(isp);
			ret = -EINVAL;
			goto exit;
		}
		module->isp_dev_handle = isp;
		pr_info("isp module open done: %p\n", isp);
	}

	channel = &module->channel[0];

	if (isp_ops->get_path) {
		ch_desc = &desc;
		ch_desc->attach_cam_id = module->idx;
		ch_desc->in_fmt = test_info->in_fmt;
		ch_desc->prop = test_info->prop;
		ch_desc->input_size.w = test_info->input_size.width;
		ch_desc->input_size.h = test_info->input_size.height;
		ch_desc->input_trim.size_x = ch_desc->input_size.w;
		ch_desc->input_trim.size_y = ch_desc->input_size.h;
		ch_desc->output_size.w = test_info->output_size.width;
		ch_desc->output_size.h = test_info->output_size.height;
		ret = isp_ops->get_path(module->isp_dev_handle, ch_desc);
		if (ret < 0) {
			pr_err("fail to get isp path for cam%d %d size %d %d.\n",
				module->idx, ch_desc->attach_cam_id,
				ch_desc->output_size.w, ch_desc->output_size.h);
			goto exit;
		} else {
			channel->isp_path_id = (int32_t)ret;
			pr_info("cam %d get isp path : %d\n",
					module->idx, channel->isp_path_id);
			if (isp_ops->set_callback) {
				ret = isp_ops->set_callback(module->isp_dev_handle,
							(uint32_t)channel->isp_path_id,
							isp_callback, module);
				if (ret)
					pr_err("fail to set cam%d callback for isp.\n",
							module->idx);
			}
		}
	}
	if (channel->isp_path_id >= 0) {
		struct isp_path_cfg_param isp_ch_desc;

		ret = isp_ops->cfg_path(module->isp_dev_handle,
				ISP_PATH_CFG_COMMON,
				channel->isp_path_id, &isp_ch_desc);
	}

	iommu_enable = test_info->iommu_en;

	pframe = get_empty_frame();
	if (pframe) {
		pframe->channel_id = channel->ch_id;
		pframe->is_reserved = 0;
		pframe->fid = 0;
		size = ch_desc->input_size.w + 4;
		size *= ch_desc->input_size.h * 10 / 8;
		ret = cambuf_alloc(
				&pframe->buf, size,
				0, iommu_enable);
		if (ret) {
			pr_err("fail to alloc buffer.\n");
			put_empty_frame(pframe);
			goto nobuf;
		}
		cambuf_kmap(&pframe->buf);
	}

	user_frame = get_empty_frame();
	if (user_frame) {
		user_frame->buf.type = CAM_BUF_USER;
		user_frame->buf.mfd[0] = test_info->inbuf_fd;
		user_frame->channel_id = channel->ch_id;
		user_frame->buf.addr_k[0] = test_info->inbuf_kaddr[1];
		user_frame->buf.addr_k[0] <<= 32;
		user_frame->buf.addr_k[0] |= test_info->inbuf_kaddr[0];
		cambuf_get_ionbuf(&user_frame->buf);
		pr_info("src buf kaddr %p.\n",
				(void *)user_frame->buf.addr_k[0]);
	}
	pr_info("copy source image.\n");
	memcpy((void *)pframe->buf.addr_k[0],
			(void *)user_frame->buf.addr_k[0], size);

	ret = isp_ops->cfg_path(module->isp_dev_handle,
					ISP_PATH_CFG_OUTPUT_BUF,
					channel->isp_path_id, user_frame);

	camera_queue_init(&channel->share_buf_queue,
			CAM_SHARED_BUF_NUM, 0, put_k_frame);
	camera_queue_init(&module->frm_queue,
			CAM_FRAME_Q_LEN, 0, camera_put_empty_frame);

	atomic_set(&module->stream_on, 1);

	ret = isp_ops->proc_frame(module->isp_dev_handle,
						pframe, channel->isp_path_id);

	pr_info("start wait for ISP callback.\n");
	ret = wait_for_completion_interruptible(&module->frm_com);
	pr_info("wait done.\n");

	user_frame = camera_dequeue(&module->frm_queue);

	pr_info("get dst frame %p. mfd %d\n",
		user_frame, user_frame->buf.mfd[0]);

	cambuf_put_ionbuf(&user_frame->buf);
	put_empty_frame(user_frame);

	camera_queue_clear(&channel->share_buf_queue);
	camera_queue_clear(&module->frm_queue);

	atomic_set(&module->stream_on, 0);

nobuf:
	isp_ops->put_path(module->isp_dev_handle,
						 channel->isp_path_id);
exit:
	pr_info("done. ret: %d\n", ret);
	return ret;
}

static int ioctl_test_dev(
			struct camera_module *module,
			unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm param;
	struct dev_test_info *test_info = (struct dev_test_info *)&param;

	ret = copy_from_user(&param, (void __user *)arg,
			     sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}
	atomic_set(&module->user_cnt, 1);

	if (test_info->dev == 0) {
		ret = test_isp(module, test_info);
		goto exit;
	}
	ret = test_dcam(module, test_info);

exit:
	if (ret >= 0)
		ret = copy_to_user((void __user *)arg, &param,
			     sizeof(struct sprd_img_parm));
	pr_info("done. ret: %d\n", ret);
	return ret;
}


static struct cam_ioctl_cmd ioctl_cmds_table[60] = {
	[_IOC_NR(SPRD_IMG_IO_SET_MODE)]                = {SPRD_IMG_IO_SET_MODE,		       img_ioctl_set_mode},
	[_IOC_NR(SPRD_IMG_IO_SET_CAP_SKIP_NUM)] = {SPRD_IMG_IO_SET_CAP_SKIP_NUM,	img_ioctl_set_cap_skip_num},
	[_IOC_NR(SPRD_IMG_IO_SET_SENSOR_SIZE)]    = {SPRD_IMG_IO_SET_SENSOR_SIZE,	img_ioctl_set_sensor_size},
	[_IOC_NR(SPRD_IMG_IO_SET_SENSOR_TRIM)]   = {SPRD_IMG_IO_SET_SENSOR_TRIM,	img_ioctl_set_sensor_trim},
	[_IOC_NR(SPRD_IMG_IO_SET_FRM_ID_BASE)]    = {SPRD_IMG_IO_SET_FRM_ID_BASE,	img_ioctl_set_frm_id_base},
	[_IOC_NR(SPRD_IMG_IO_SET_CROP)]                 = {SPRD_IMG_IO_SET_CROP,		       img_ioctl_set_crop},
	[_IOC_NR(SPRD_IMG_IO_SET_FLASH)]                = {SPRD_IMG_IO_SET_FLASH,		img_ioctl_set_flash},
	[_IOC_NR(SPRD_IMG_IO_SET_OUTPUT_SIZE)]     = {SPRD_IMG_IO_SET_OUTPUT_SIZE,	img_ioctl_set_output_size},
	[_IOC_NR(SPRD_IMG_IO_SET_ZOOM_MODE)]       = {SPRD_IMG_IO_SET_ZOOM_MODE,		NULL},
	[_IOC_NR(SPRD_IMG_IO_SET_SENSOR_IF)]         = {SPRD_IMG_IO_SET_SENSOR_IF,		img_ioctl_set_sensor_if},
	[_IOC_NR(SPRD_IMG_IO_SET_FRAME_ADDR)]      = {SPRD_IMG_IO_SET_FRAME_ADDR,	img_ioctl_set_frame_addr},
	[_IOC_NR(SPRD_IMG_IO_PATH_FRM_DECI)]         = {SPRD_IMG_IO_PATH_FRM_DECI,		img_ioctl_path_frm_deci},
/*	[_IOC_NR(SPRD_IMG_IO_PATH_PAUSE)]               = {SPRD_IMG_IO_PATH_PAUSE,		NULL},*/
	[_IOC_NR(SPRD_IMG_IO_PATH_RESUME)]            = {SPRD_IMG_IO_PATH_RESUME,		NULL},
	[_IOC_NR(SPRD_IMG_IO_STREAM_ON)]               = {SPRD_IMG_IO_STREAM_ON,		img_ioctl_stream_on},
	[_IOC_NR(SPRD_IMG_IO_STREAM_OFF)]              = {SPRD_IMG_IO_STREAM_OFF,		img_ioctl_stream_off},
	[_IOC_NR(SPRD_IMG_IO_GET_FMT)]                    = {SPRD_IMG_IO_GET_FMT,		img_ioctl_get_fmt},
	[_IOC_NR(SPRD_IMG_IO_GET_CH_ID)]                = {SPRD_IMG_IO_GET_CH_ID,		img_ioctl_get_ch_id},
	[_IOC_NR(SPRD_IMG_IO_GET_TIME)]                   = {SPRD_IMG_IO_GET_TIME,		img_ioctl_get_time},
	[_IOC_NR(SPRD_IMG_IO_CHECK_FMT)]                = {SPRD_IMG_IO_CHECK_FMT,		img_ioctl_check_fmt},
	[_IOC_NR(SPRD_IMG_IO_SET_SHRINK)]               = {SPRD_IMG_IO_SET_SHRINK,		img_ioctl_set_shrink},
	[_IOC_NR(SPRD_IMG_IO_SET_FREQ_FLAG)]          = {SPRD_IMG_IO_SET_FREQ_FLAG,		NULL},
	[_IOC_NR(SPRD_IMG_IO_CFG_FLASH)]                  = {SPRD_IMG_IO_CFG_FLASH,		img_ioctl_cfg_flash},
	[_IOC_NR(SPRD_IMG_IO_PDAF_CONTROL)]           = {SPRD_IMG_IO_PDAF_CONTROL,		img_ioctl_pdaf_control},
	[_IOC_NR(SPRD_IMG_IO_GET_IOMMU_STATUS)]   = {SPRD_IMG_IO_GET_IOMMU_STATUS,	img_ioctl_get_iommu_status},
	[_IOC_NR(SPRD_IMG_IO_DISABLE_MODE)]           = {SPRD_IMG_IO_DISABLE_MODE,		NULL},
	[_IOC_NR(SPRD_IMG_IO_ENABLE_MODE)]             = {SPRD_IMG_IO_ENABLE_MODE,		NULL},
	[_IOC_NR(SPRD_IMG_IO_START_CAPTURE)]          = {SPRD_IMG_IO_START_CAPTURE,		img_ioctl_start_capture},
	[_IOC_NR(SPRD_IMG_IO_STOP_CAPTURE)]            = {SPRD_IMG_IO_STOP_CAPTURE,		img_ioctl_stop_capture},
	[_IOC_NR(SPRD_IMG_IO_SET_PATH_SKIP_NUM)]   = {SPRD_IMG_IO_SET_PATH_SKIP_NUM,	img_ioctl_set_path_skip_num},
	[_IOC_NR(SPRD_IMG_IO_SBS_MODE)]                    = {SPRD_IMG_IO_SBS_MODE,		NULL},
	[_IOC_NR(SPRD_IMG_IO_DCAM_PATH_SIZE)]         = {SPRD_IMG_IO_DCAM_PATH_SIZE,	img_ioctl_dcam_path_size},
	[_IOC_NR(SPRD_IMG_IO_SET_SENSOR_MAX_SIZE)] = {SPRD_IMG_IO_SET_SENSOR_MAX_SIZE,	img_ioctl_set_sensor_max_size},
	[_IOC_NR(SPRD_ISP_IO_IRQ)]                                  = {SPRD_ISP_IO_IRQ,			img_ioctl_isp},
	[_IOC_NR(SPRD_ISP_IO_READ)]                                = {SPRD_ISP_IO_READ,			img_ioctl_isp},
	[_IOC_NR(SPRD_ISP_IO_WRITE)]                              = {SPRD_ISP_IO_WRITE,			img_ioctl_isp},
	[_IOC_NR(SPRD_ISP_IO_RST)] 	                                 = {SPRD_ISP_IO_RST,			img_ioctl_isp},
	[_IOC_NR(SPRD_ISP_IO_STOP)]                                 = {SPRD_ISP_IO_STOP,			img_ioctl_isp},
	[_IOC_NR(SPRD_ISP_IO_INT)]                                    = {SPRD_ISP_IO_INT,			img_ioctl_isp},
	[_IOC_NR(SPRD_ISP_IO_SET_STATIS_BUF)]               = {SPRD_ISP_IO_SET_STATIS_BUF,	img_ioctl_set_statis_buf},
	[_IOC_NR(SPRD_ISP_IO_CFG_PARAM)]                       = {SPRD_ISP_IO_CFG_PARAM,		img_ioctl_cfg_param},
	[_IOC_NR(SPRD_ISP_REG_READ)]                               = {SPRD_ISP_REG_READ,			img_ioctl_isp},
	[_IOC_NR(SPRD_ISP_IO_POST_3DNR)]                        = {SPRD_ISP_IO_POST_3DNR,		img_ioctl_isp},
	[_IOC_NR(SPRD_STATIS_IO_CFG_PARAM)]                 = {SPRD_STATIS_IO_CFG_PARAM,		NULL},
	[_IOC_NR(SPRD_ISP_IO_RAW_CAP)]                           = {SPRD_ISP_IO_RAW_CAP,		img_ioctl_raw_cap},

	[_IOC_NR(SPRD_IMG_IO_GET_DCAM_RES)]       = {SPRD_IMG_IO_GET_DCAM_RES,		img_ioctl_get_cam_res},
	[_IOC_NR(SPRD_IMG_IO_PUT_DCAM_RES)]       = {SPRD_IMG_IO_PUT_DCAM_RES,		img_ioctl_put_cam_res},

	[_IOC_NR(SPRD_ISP_IO_SET_PULSE_LINE)]                = {SPRD_ISP_IO_SET_PULSE_LINE,	NULL},
	[_IOC_NR(SPRD_ISP_IO_CFG_START)]                        = {SPRD_ISP_IO_CFG_START,		img_ioctl_isp},
	[_IOC_NR(SPRD_ISP_IO_POST_YNR)]                          = {SPRD_ISP_IO_POST_YNR,		img_ioctl_isp},
	[_IOC_NR(SPRD_ISP_IO_SET_NEXT_VCM_POS)]          = {SPRD_ISP_IO_SET_NEXT_VCM_POS,	NULL},
	[_IOC_NR(SPRD_ISP_IO_SET_VCM_LOG)]                    = {SPRD_ISP_IO_SET_VCM_LOG,		NULL},
	[_IOC_NR(SPRD_IMG_IO_SET_3DNR)]                         = {SPRD_IMG_IO_SET_3DNR,		NULL},
	[_IOC_NR(SPRD_IMG_IO_SET_FUNCTION_MODE)]       = {SPRD_IMG_IO_SET_FUNCTION_MODE,	img_ioctl_set_function_mode},
	[_IOC_NR(SPRD_IMG_IO_GET_FLASH_INFO)]              = {SPRD_IMG_IO_GET_FLASH_INFO,	NULL},
	[_IOC_NR(SPRD_ISP_IO_MASK_3A)]                            = {SPRD_ISP_IO_MASK_3A,		NULL},

	[_IOC_NR(SPRD_IMG_IO_PATH_PAUSE)]               = {SPRD_IMG_IO_PATH_PAUSE,		ioctl_test_dev},
};


static long sprd_img_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	int ret = 0;
	struct camera_module *module = NULL;
	struct cam_ioctl_cmd *ioctl_cmd_p = NULL;
	int nr = _IOC_NR(cmd);

	pr_debug("cam ioctl, cmd:0x%x, cmdnum %d\n", cmd, nr);

	module = (struct camera_module *)file->private_data;
	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (unlikely(!(nr >= 0 && nr < ARRAY_SIZE(ioctl_cmds_table)))) {
		pr_info("invalid cmd: 0x%xn", cmd);
		return -EINVAL;
	}

	ioctl_cmd_p = &ioctl_cmds_table[nr];
	if (unlikely((ioctl_cmd_p->cmd != cmd) ||
			(ioctl_cmd_p->cmd_proc == NULL))) {
		pr_debug("unsupported cmd_k: 0x%x, cmd_u: 0x%x, nr: %d\n",
			ioctl_cmd_p->cmd, cmd, nr);
		return 0;
	}

	ret = ioctl_cmd_p->cmd_proc(module, arg);
	if (ret) {
		pr_err("fail to ioctl cmd:%x, nr:%d, func %ps\n",
			cmd, nr, ioctl_cmd_p->cmd_proc);
		return -EFAULT;
	}

	if (atomic_read(&module->run_flag) == 0)
		pr_debug("cam id:%d, %ps, successfully!\n",
			   module->idx,
			   ioctl_cmd_p->cmd_proc);
	return 0;
}

static ssize_t sprd_img_read(struct file *file, char __user *u_data,
			     size_t cnt, loff_t *cnt_ret)
{
	int ret = 0;
	struct sprd_img_read_op read_op;
	struct camera_module *module = NULL;
	struct camera_frame *pframe;
	struct channel_context *pchannel;
	struct sprd_img_path_capability *cap;

	module = (struct camera_module *)file->private_data;
	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (cnt != sizeof(struct sprd_img_read_op)) {
		pr_err("fail to img read, cnt %zd read_op %d\n", cnt,
		       (int32_t)sizeof(struct sprd_img_read_op));
		return -EIO;
	}

	if (copy_from_user(&read_op, (void __user *)u_data, cnt)) {
		pr_err("fail to get user info\n");
		return -EFAULT;
	}

	pr_debug("cam %d read cmd %d\n", module->idx, read_op.cmd);

	switch (read_op.cmd) {
	case SPRD_IMG_GET_SCALE_CAP:
		read_op.parm.reserved[0] = 4672;
		read_op.parm.reserved[1] = 4;
		read_op.parm.reserved[2] = 4672;
		pr_debug("line threshold %d, sc factor %d, scaling %d.\n",
			   read_op.parm.reserved[0], read_op.parm.reserved[1],
			   read_op.parm.reserved[2]);
		break;
	case SPRD_IMG_GET_FRM_BUFFER:
rewait:
		memset(&read_op, 0, sizeof(struct sprd_img_read_op));
		while (1) {
			ret = wait_for_completion_interruptible(&module->frm_com);
			if (ret == 0) {
				break;
			} else if (ret == -ERESTARTSYS) {
				read_op.evt = IMG_SYS_BUSY;
				ret = 0;
				goto read_end;
			} else {
				pr_err("read frame buf, fail to down, %d\n", ret);
				return -EPERM;
			}
		}

		pframe = camera_dequeue(&module->frm_queue);
		if (!pframe) {
			/* any exception happens. */
			pr_err("fail to read frame buffer queue\n");
			read_op.evt = IMG_TX_STOP;
		} else {
			read_op.evt = pframe->evt;
			cambuf_put_ionbuf(&pframe->buf);
		}

		if (read_op.evt == IMG_TX_STOP) {
			pr_info("sprd_img_read: tx stop\n");
		} else if (read_op.evt == IMG_TX_DONE) {
			atomic_set(&module->run_flag, 1);
			pchannel = &module->channel[pframe->channel_id];
			if (pframe->buf.mfd[0] == pchannel->reserved_buf_fd) {
				pr_info("get output buffer with reserved frame fd %d\n",
							pchannel->reserved_buf_fd);
				put_empty_frame(pframe);
				goto rewait;
			}

			read_op.parm.frame.channel_id = pframe->channel_id;
			read_op.parm.frame.irq_type = pframe->irq_type;
			read_op.parm.frame.length = pframe->width;
			read_op.parm.frame.height = pframe->height;
			read_op.parm.frame.index = pchannel->frm_base_id;
			read_op.parm.frame.frm_base_id = pchannel->frm_base_id;
			read_op.parm.frame.real_index = pframe->fid;
			read_op.parm.frame.sec = pframe->time.tv_sec;
			read_op.parm.frame.usec = pframe->time.tv_usec;
			read_op.parm.frame.monoboottime = pframe->boot_time.tv64;
			read_op.parm.frame.yaddr_vir = (uint32_t)pframe->buf.addr_vir[0];
			read_op.parm.frame.uaddr_vir = (uint32_t)pframe->buf.addr_vir[1];
			read_op.parm.frame.vaddr_vir = (uint32_t)pframe->buf.addr_vir[2];
			read_op.parm.frame.mfd = pframe->buf.mfd[0];
		} else {
			pr_err("error event %d\n", read_op.evt);
			read_op.parm.frame.irq_type = pframe->irq_type;
			read_op.parm.frame.irq_property = pframe->irq_property;
		}

		if (pframe)
			put_empty_frame(pframe);

		pr_debug("read frame, evt 0x%x irq %d ch 0x%x index 0x%x mfd %d\n",
			   read_op.evt,
			   read_op.parm.frame.irq_type,
			   read_op.parm.frame.channel_id,
			   read_op.parm.frame.real_index,
			   read_op.parm.frame.mfd);
		break;

	case SPRD_IMG_GET_PATH_CAP:
		pr_debug("get path capbility\n");
		cap = &read_op.parm.capability;
		memset(cap, 0, sizeof(struct sprd_img_path_capability));
		cap->support_3dnr_mode = 0;
		cap->count = 5;
		cap->path_info[CAM_CH_RAW].support_yuv = 0;
		cap->path_info[CAM_CH_RAW].support_raw = 1;
		cap->path_info[CAM_CH_RAW].support_jpeg = 0;
		cap->path_info[CAM_CH_RAW].support_scaling = 0;
		cap->path_info[CAM_CH_RAW].support_trim = 1;
		cap->path_info[CAM_CH_RAW].is_scaleing_path = 0;
		cap->path_info[CAM_CH_RAW_BIN].support_yuv = 0;
		cap->path_info[CAM_CH_RAW_BIN].support_raw = 1;
		cap->path_info[CAM_CH_RAW_BIN].support_jpeg = 0;
		cap->path_info[CAM_CH_RAW_BIN].support_scaling = 1;
		cap->path_info[CAM_CH_RAW_BIN].support_trim = 1;
		cap->path_info[CAM_CH_RAW_BIN].is_scaleing_path = 0;
		cap->path_info[CAM_CH_PRE].line_buf = ISP_MAX_WIDTH;
		cap->path_info[CAM_CH_PRE].support_yuv = 1;
		cap->path_info[CAM_CH_PRE].support_raw = 0;
		cap->path_info[CAM_CH_PRE].support_jpeg = 0;
		cap->path_info[CAM_CH_PRE].support_scaling = 1;
		cap->path_info[CAM_CH_PRE].support_trim = 1;
		cap->path_info[CAM_CH_PRE].is_scaleing_path = 0;
		cap->path_info[CAM_CH_CAP].line_buf = ISP_MAX_WIDTH;
		cap->path_info[CAM_CH_CAP].support_yuv = 1;
		cap->path_info[CAM_CH_CAP].support_raw = 0;
		cap->path_info[CAM_CH_CAP].support_jpeg = 0;
		cap->path_info[CAM_CH_CAP].support_scaling = 1;
		cap->path_info[CAM_CH_CAP].support_trim = 1;
		cap->path_info[CAM_CH_CAP].is_scaleing_path = 0;
		cap->path_info[CAM_CH_VID].line_buf = ISP_MAX_WIDTH;
		cap->path_info[CAM_CH_VID].support_yuv = 1;
		cap->path_info[CAM_CH_VID].support_raw = 0;
		cap->path_info[CAM_CH_VID].support_jpeg = 0;
		cap->path_info[CAM_CH_VID].support_scaling = 1;
		cap->path_info[CAM_CH_VID].support_trim = 1;
		cap->path_info[CAM_CH_VID].is_scaleing_path = 0;
		break;

	default:
		pr_err("fail to get valid cmd\n");
		return -EINVAL;
	}

read_end:
	if (copy_to_user((void __user *)u_data, &read_op, cnt))
		ret = -EFAULT;

	if (ret)
		cnt = ret;

	return cnt;
}

static ssize_t sprd_img_write(struct file *file, const char __user *u_data,
			      size_t cnt, loff_t *cnt_ret)
{
	int ret = 0;
	struct sprd_img_write_op write_op;
	struct camera_module *module = NULL;

	module = (struct camera_module *)file->private_data;
	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (cnt != sizeof(struct sprd_img_write_op)) {
		pr_err("fail to write, cnt %zd  write_op %d\n", cnt,
				(uint32_t)sizeof(struct sprd_img_write_op));
		return -EIO;
	}

	if (copy_from_user(&write_op, (void __user *)u_data, cnt)) {
		pr_err("fail to get user info\n");
		return -EFAULT;
	}

	switch (write_op.cmd) {
	case SPRD_IMG_STOP_DCAM:
		pr_info("user stop camera %d\n", module->idx);
		complete(&module->frm_com);
		break;

	default:
		pr_err("error: unsupported write cmd %d\n", write_op.cmd);
		ret = -EINVAL;
		break;
	}

	ret =  copy_to_user((void __user *)u_data, &write_op, cnt);
	if (ret) {
		pr_err("fail to get user info\n");
		return -EFAULT;
	}

	if (ret)
		cnt = ret;

	return cnt;
}


static int sprd_img_open(struct inode *node, struct file *file)
{
	int ret = 0;
	unsigned long flag;
	struct camera_module *module = NULL;
	struct camera_group *grp = NULL;
	struct miscdevice *md = file->private_data;
	uint32_t i, idx, count = 0;

	grp = md->this_device->platform_data;
	count = grp->dcam_count;

	if (count == 0 || count > CAM_COUNT) {
		pr_err("error: invalid dts configured dcam count\n");
		return -ENODEV;
	}

	if (atomic_inc_return(&grp->camera_opened) > count) {
		pr_err("sprd_img: all %d cameras opened already.", count);
		atomic_dec(&grp->camera_opened);
		return -EMFILE;
	}

	pr_info("sprd_img: the camera opened count %d\n",
			atomic_read(&grp->camera_opened));

	spin_lock_irqsave(&grp->module_lock, flag);
	for (i = 0, idx = count; i < count; i++) {
		if ((grp->module_used & (1 << i)) == 0) {
			if (grp->module[i] != NULL) {
				pr_err("fatal: un-release camera module:  %p, idx %d\n",
						grp->module[i], i);
				spin_unlock_irqrestore(&grp->module_lock, flag);
				ret = -EMFILE;
				goto exit;
			}
			idx = i;
			grp->module_used |= (1 << i);
			break;
		}
	}
	spin_unlock_irqrestore(&grp->module_lock, flag);

	if (idx == count) {
		pr_err("error: no available camera module.\n");
		ret = -EMFILE;
		goto exit;
	}

	pr_debug("kzalloc. size of module %x, group %x\n",
		(int)sizeof(struct camera_module),
		(int) sizeof(struct camera_group));

	module = vzalloc(sizeof(struct camera_module));
	if (!module) {
		pr_err("fail to alloc camera module %d\n", idx);
		ret = -ENOMEM;
		goto alloc_fail;
	}

	module->idx = idx;
	ret = camera_module_init(module);
	if (ret) {
		pr_err("fail to init camera module %d\n", idx);
		ret = -ENOMEM;
		goto init_fail;
	}

	if (atomic_read(&grp->camera_opened) == 1) {
		isp_ops = get_isp_ops();
		dcam_ops = get_dcam_ops();
		if (isp_ops == NULL || dcam_ops == NULL) {
			pr_err("error:  isp ops %p, dcam ops %p\n",
					isp_ops, dcam_ops);
			goto init_fail;
		}
		/* should check all needed interface here. */

		g_empty_frm_q = &grp->empty_frm_q;
		camera_queue_init(g_empty_frm_q,
				CAM_EMP_Q_LEN_MAX, 0,
				free_empty_frame);

		g_mem_dbg = &grp->mem_dbg;
		pr_info("init %p, %p\n", g_empty_frm_q, g_mem_dbg);
		mdbg_init();
	}

	module->idx = idx;
	module->grp = grp;
	grp->module[idx] = module;
	file->private_data = (void *)module;
	pr_info("sprd_img: open end! %d, %p, %p, grp %p\n",
		idx, module, grp->module[idx], grp);

	return 0;

init_fail:
	vfree(module);

alloc_fail:
	spin_lock_irqsave(&grp->module_lock, flag);
	grp->module_used &= ~(1 << idx);
	grp->module[idx] = NULL;
	spin_unlock_irqrestore(&grp->module_lock, flag);

exit:
	atomic_dec(&grp->camera_opened);

	pr_err("open camera failed: %d\n", ret);
	return ret;
}

static int sprd_img_release(struct inode *node, struct file *file)
{
	int ret = 0;
	int idx = 0;
	unsigned long flag;
	struct camera_group *group = NULL;
	struct camera_module *module;
	struct dcam_pipe_dev *dcam_dev = NULL;
	struct isp_pipe_dev *isp_dev = NULL;

	pr_info("sprd_img: cam release start.\n");

	module = (struct camera_module *)file->private_data;
	if (!module || !module->grp) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	group = module->grp;
	idx = module->idx;

	pr_info("cam %d, used: %d, module %p, %p, grp %p\n",
		idx, group->module_used, module, group->module[idx], group);

	spin_lock_irqsave(&group->module_lock, flag);
	if (((group->module_used & (1 << idx)) == 0) ||
		(group->module[idx] != module)) {
		pr_err("fatal error to release camera %d. used:%x, module:%p\n",
					idx, group->module_used, module);
		spin_unlock_irqrestore(&group->module_lock, flag);
		return -EFAULT;
	}
	spin_unlock_irqrestore(&group->module_lock, flag);

	if (atomic_read(&module->user_cnt) > 0) {
		module->attach_sensor_id = -1;

		if (module->workqueue)
			destroy_workqueue(module->workqueue);
		module->workqueue  = NULL;

		ret = img_ioctl_stream_off(module, 0L);

		dcam_dev = module->dcam_dev_handle;
		isp_dev = module->isp_dev_handle;

		if (dcam_dev) {
			pr_info("force close dcam %p\n", dcam_dev);
			dcam_ops->close(dcam_dev);
			put_dcam_pipe_dev(dcam_dev);
			module->dcam_dev_handle = NULL;
		}

		if (isp_dev) {
			pr_info("force close isp %p\n", dcam_dev);
			isp_ops->close(isp_dev);
			put_isp_pipe_dev(isp_dev);
			module->isp_dev_handle = NULL;
		}

		atomic_set(&module->user_cnt, 0);
	}

	camera_module_deinit(module);

	spin_lock_irqsave(&group->module_lock, flag);
	group->module_used &= ~(1 << idx);
	group->module[idx] = NULL;
	spin_unlock_irqrestore(&group->module_lock, flag);

	vfree(module);
	file->private_data = NULL;

	if (atomic_dec_return(&group->camera_opened) == 0) {

		pr_info("release %p, %p\n", g_empty_frm_q, g_mem_dbg);

		/* g_leak_debug_cnt should be 0 after clr, or else memory leak. */
		ret = camera_queue_clear(g_empty_frm_q);
		g_empty_frm_q = NULL;

		ret = mdbg_check();
		g_mem_dbg = NULL;

		dcam_ops = NULL;
		isp_ops = NULL;
	}

	pr_info("sprd_img: cam %d release end.\n", idx);

	return ret;
}

#ifdef CONFIG_COMPAT
static long compat_sprd_img_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	long ret = 0L;

	ret = file->f_op->unlocked_ioctl(file, cmd, arg);
	return ret;
}
#endif

static const struct file_operations image_fops = {
	.open = sprd_img_open,
	.unlocked_ioctl = sprd_img_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_sprd_img_ioctl,
#endif
	.release = sprd_img_release,
	.read = sprd_img_read,
	.write = sprd_img_write,
};

static struct miscdevice image_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = IMG_DEVICE_NAME,
	.fops = &image_fops,
};

static int sprd_img_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct camera_group *group = NULL;

	if (!pdev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	pr_info("Start camera img probe\n");
	group = kzalloc(sizeof(struct camera_group), GFP_KERNEL);
	if (group == NULL) {
		pr_err("alloc memory fail.");
		return -ENOMEM;
	}

	ret = misc_register(&image_dev);
	if (ret) {
		pr_err("fail to register misc devices, ret %d\n", ret);
		kfree(group);
		return -EACCES;
	}

	image_dev.this_device->of_node = pdev->dev.of_node;
	image_dev.this_device->platform_data = (void *)group;
	group->md = &image_dev;
	group->pdev = pdev;
	atomic_set(&group->camera_opened, 0);
	spin_lock_init(&group->module_lock);

	pr_info("sprd img probe pdev name %s\n", pdev->name);
	ret = sprd_cam_pw_domain_init(pdev);
	if (ret) {
		pr_err("fail to init pw domain\n");
		goto probe_pw_fail;
	}

	pr_info("sprd dcam dev name %s\n", pdev->dev.init_name);
	ret = sprd_dcam_parse_dt(pdev->dev.of_node,
						&group->dcam[0],
						&group->dcam_count);
	if (ret) {
		pr_err("fail to parse dcam dts\n");
		goto probe_pw_fail;
	}

	pr_info("sprd isp dev name %s\n", pdev->dev.init_name);
	ret = sprd_isp_parse_dt(pdev->dev.of_node,
						&group->isp[0],
						&group->isp_count);
	if (ret) {
		pr_err("fail to parse isp dts\n");
		goto probe_pw_fail;
	}

	/*ret = sprd_dcam_debugfs_init();*/
	if (ret)
		pr_err("fail to init dcam debugfs\n");

	ret = sprd_isp_debugfs_init();
	if (ret)
		pr_err("fail to init isp debugfs\n");

	return 0;

probe_pw_fail:
	misc_deregister(&image_dev);
	kfree(group);

	return ret;
}

static int sprd_img_remove(struct platform_device *pdev)
{
	struct camera_group *group = NULL;

	if (!pdev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	group = image_dev.this_device->platform_data;

	kfree(image_dev.this_device->platform_data);
	misc_deregister(&image_dev);

	return 0;
}



static const struct of_device_id sprd_dcam_of_match[] = {
	{ .compatible = "sprd,dcam", },
	{},
};

static struct platform_driver sprd_img_driver = {
	.probe = sprd_img_probe,
	.remove = sprd_img_remove,
	.driver = {
		.name = IMG_DEVICE_NAME,
		.of_match_table = of_match_ptr(sprd_dcam_of_match),
	},
};

module_platform_driver(sprd_img_driver);

MODULE_DESCRIPTION("Sprd CAM Driver");
MODULE_AUTHOR("Multimedia_Camera@Spreadtrum");
MODULE_LICENSE("GPL");
