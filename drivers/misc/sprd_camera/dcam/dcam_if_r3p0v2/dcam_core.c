/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
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
#include <linux/vmalloc.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include <video/sprd_img.h>
#include <video/sprd_mm.h>
#include "flash_drv.h"
#include "dcam_drv.h"
#include "isp_drv.h"
#include "isp_reg.h"
#include "isp_int.h"
#include "isp_buf.h"
#include "cam_pw_domain.h"
#include "isp_slw.h"
#include "sprd_sensor_drv.h"
#include "compat_isp_drv.h"

/* Macro Definitions */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_CORE: %d: %d " fmt, current->pid, __LINE__

#define IMG_DEVICE_NAME			"sprd_image"
#define CAMERA_INVALID_FOURCC		0xFFFFFFFF
#define CAMERA_QUEUE_LENGTH		96
#define CAMERA_TIMING_LEN		16

#define CAMERA_ZOOM_LEVEL_MAX		4
#define CAMERA_ZOOM_STEP(x, y)		(((x) - (y)) / CAMERA_ZOOM_LEVEL_MAX)
#define CAMERA_PIXEL_ALIGNED		4
#define CAMERA_WIDTH(w)			((w) & ~(CAMERA_PIXEL_ALIGNED - 1))
#define CAMERA_HEIGHT(h)		((h) & ~(CAMERA_PIXEL_ALIGNED - 1))

#define DISCARD_FRAME_TIME		10000
#ifdef CONFIG_SC_FPGA
#define DCAM_TIMEOUT			(2000*100)
#else
#define DCAM_TIMEOUT			1500
#endif
#define DCAM_VALID_FRAME_COUNT		3

#define CAP_IN_SIZE_5M_WIDTH		2592
#define CAP_IN_SIZE_2M_WIDTH		1600

/* Structure Definitions */

enum {
	PATH_IDLE = 0x00,
	PATH_RUN,
};

struct camera_format {
	char *name;
	unsigned int fourcc;
	int depth;
};

struct camera_node {
	unsigned int irq_flag;
	unsigned int irq_type;
	unsigned int f_type;
	unsigned int index;
	unsigned int height;
	unsigned int yaddr;
	unsigned int uaddr;
	unsigned int vaddr;
	unsigned int yaddr_vir;
	unsigned int uaddr_vir;
	unsigned int vaddr_vir;
	unsigned int phy_addr;
	unsigned int vir_addr;
	unsigned int kaddr[2];
	unsigned int buf_size;
	unsigned int irq_property;
	unsigned int invalid_flag;
	unsigned int mfd[3];
	unsigned int reserved[2];
	struct timeval time;
	ktime_t boot_time;
};

struct camera_queue {
	struct camera_node node[CAMERA_QUEUE_LENGTH];
	struct camera_node *write;
	struct camera_node *read;
	unsigned int wcnt;
	unsigned int rcnt;
	spinlock_t lock;
};

struct camera_img_buf_addr {
	struct camera_addr frm_addr;
	struct camera_addr frm_addr_vir;
};

struct camera_img_buf_queue {
	struct camera_img_buf_addr buf_addr[DCAM_FRM_CNT_MAX];
	struct camera_img_buf_addr *write;
	struct camera_img_buf_addr *read;
	unsigned int wcnt;
	unsigned int rcnt;
};

struct camera_path_spec {
	unsigned int is_work;
	unsigned int is_from_isp;
	unsigned int rot_mode;
	unsigned int status;
	enum isp_path_mode path_mode;
	struct camera_size in_size;
	struct camera_path_dec img_deci;
	struct camera_rect in_rect;
	struct camera_rect in_rect_current;
	struct camera_rect in_rect_backup;
	struct camera_size out_size;
	enum dcam_fmt out_fmt;
	struct camera_endian_sel end_sel;
	unsigned int fourcc;
	unsigned int pixel_depth;
	unsigned int frm_id_base;
	unsigned int frm_type;
	unsigned int index[DCAM_FRM_CNT_MAX];
	struct camera_img_buf_queue buf_queue;
	struct camera_img_buf_queue tmp_buf_queue;
	struct camera_addr frm_reserved_addr;
	struct camera_addr frm_reserved_addr_vir;
	struct camera_frame *frm_ptr[DCAM_FRM_CNT_MAX];
	unsigned int frm_cnt_act;
	unsigned int path_frm_deci;
	unsigned int skip_num;
	struct dcam_regular_desc regular_desc;
	struct sprd_pdaf_control pdaf_ctrl;
};

struct camera_info {
	unsigned int if_mode;
	unsigned int sn_mode;
	unsigned int yuv_ptn;
	unsigned int data_bits;
	unsigned int is_loose;
	unsigned int lane_num;
	unsigned int bps_per_lane;
	struct dcam_cap_sync_pol sync_pol;
	unsigned int frm_deci;
	struct dcam_cap_dec img_deci;
	struct camera_size cap_in_size;
	struct camera_rect cap_in_rect;
	struct camera_size cap_out_size;
	struct camera_size dst_size;
	struct camera_size sn_max_size;
	unsigned int pxl_fmt;
	unsigned int need_isp_tool;
	unsigned int need_isp;
	unsigned int rt_refocus;
	struct camera_rect path_input_rect;
	struct camera_path_spec dcam_path[CAMERA_MAX_PATH];
	unsigned int capture_mode;
	unsigned int skip_number;
	struct sprd_img_set_flash set_flash;
	unsigned int after_af;
	unsigned int is_smooth_zoom;
	struct timeval timestamp;
	unsigned int scene_mode;
	unsigned int is_slow_motion;
	unsigned int sensor_id;
	struct sprd_flash_capacity capacity;
	unsigned int raw_cap_flag;
	unsigned int slow_motion_flag;
};

struct camera_dev {
	struct mutex dcam_mutex;
	struct completion irq_com;
	atomic_t users;
	struct camera_info dcam_cxt;
	atomic_t stream_on;
	struct camera_queue queue;
	struct timer_list dcam_timer;
	atomic_t run_flag;
	struct completion flash_thread_com;
	struct task_struct *flash_thread;
	unsigned int is_flash_thread_stop;
	unsigned int frame_skipped;
	struct completion zoom_thread_com;
	struct task_struct *zoom_thread;
	unsigned int is_zoom_thread_stop;
	unsigned int zoom_level;
	unsigned int channel_id;
	unsigned int use_path;
	enum dcam_id idx;
	void *isp_dev_handle;
	struct isp_if_context *ispif;
	unsigned int cap_flag;
	unsigned int cap_scene;
	unsigned int raw_cap;
};

struct camera_group {
	unsigned int dev_inited;
	unsigned int mode_inited;
	unsigned int dcam_res_used;
	unsigned int camera_output_info;
	unsigned int dcam_count;
	unsigned int isp_count;
	unsigned int sensor_id;
	atomic_t camera_opened;
	struct camera_dev *dev[DCAM_ID_MAX];
	struct miscdevice *md;
	struct platform_device *pdev;
};

struct camera_file {
	int idx;
	struct camera_group *grp;
};

/* Static Variables Declaration */
static struct camera_format dcam_img_fmt[] = {
	{
		.name = "4:2:2, packed, YUYV",
		.fourcc = IMG_PIX_FMT_YUYV,
		.depth = 16,
	},
	{
		.name = "4:2:2, packed, YVYU",
		.fourcc = IMG_PIX_FMT_YVYU,
		.depth = 16,
	},
	{
		.name = "4:2:2, packed, UYVY",
		.fourcc = IMG_PIX_FMT_UYVY,
		.depth = 16,
	},
	{
		.name = "4:2:2, packed, VYUY",
		.fourcc = IMG_PIX_FMT_VYUY,
		.depth = 16,
	},
	{
		.name = "YUV 4:2:2, planar, (Y-Cb-Cr)",
		.fourcc = IMG_PIX_FMT_YUV422P,
		.depth = 16,
	},
	{
		.name = "YUV 4:2:0 planar (Y-CbCr)",
		.fourcc = IMG_PIX_FMT_NV12,
		.depth = 12,
	},
	{
		.name = "YVU 4:2:0 planar (Y-CrCb)",
		.fourcc = IMG_PIX_FMT_NV21,
		.depth = 12,
	},

	{
		.name = "YUV 4:2:0 planar (Y-Cb-Cr)",
		.fourcc = IMG_PIX_FMT_YUV420,
		.depth = 12,
	},
	{
		.name = "YVU 4:2:0 planar (Y-Cr-Cb)",
		.fourcc = IMG_PIX_FMT_YVU420,
		.depth = 12,
	},
	{
		.name = "RGB565 (LE)",
		.fourcc = IMG_PIX_FMT_RGB565,
		.depth = 16,
	},
	{
		.name = "RGB565 (BE)",
		.fourcc = IMG_PIX_FMT_RGB565X,
		.depth = 16,
	},
	{
		.name = "RawRGB",
		.fourcc = IMG_PIX_FMT_GREY,
		.depth = 8,
	},
	{
		.name = "JPEG",
		.fourcc = IMG_PIX_FMT_JPEG,
		.depth = 8,
	},
};

static int sprd_camera_stream_off(struct file *file);
extern void isp_statis_reset_all_queue(struct isp_pipe_dev *dev);

/* Internal Function Implementation */
static int img_get_timestamp(struct timeval *tv)
{
	struct timespec ts;

	ktime_get_ts(&ts);
	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = ts.tv_nsec / NSEC_PER_USEC;

	return 0;
}

static int sprd_img_buf_queue_init(struct camera_img_buf_queue *queue)
{
	if (queue == NULL)
		return -EINVAL;

	memset(queue, 0, sizeof(*queue));
	queue->write = &queue->buf_addr[0];
	queue->read = &queue->buf_addr[0];

	return 0;
}

static int sprd_img_setflash(enum dcam_id idx,
			     struct sprd_img_set_flash *set_flash)
{
	sprd_flash_ctrl(set_flash);
	DCAM_TRACE("%d set flash\n", idx);

	return 0;
}

static int sprd_img_opt_flash(struct camera_frame *frame, void *param)
{
	struct camera_dev *dev = (struct camera_dev *)param;
	struct camera_info *info = NULL;
	unsigned int led0_ctrl;
	unsigned int led1_ctrl;
	unsigned int led0_status;
	unsigned int led1_status;

	if (dev == NULL) {
		DCAM_TRACE("dev is NULL\n");
		return 0;
	}

	info = &dev->dcam_cxt;
	led0_ctrl = info->set_flash.led0_ctrl;
	led1_ctrl = info->set_flash.led1_ctrl;
	led0_status = info->set_flash.led0_status;
	led1_status = info->set_flash.led1_status;

	if ((led0_ctrl && led0_status < FLASH_STATUS_MAX) ||
	    (led1_ctrl && led1_status < FLASH_STATUS_MAX)) {
		DCAM_TRACE("led0_status %d led1_status %d\n",
			   led0_status, led1_status);
		if (led0_status == FLASH_CLOSE_AFTER_AUTOFOCUS ||
		    led1_status == FLASH_CLOSE_AFTER_AUTOFOCUS) {
			img_get_timestamp(&info->timestamp);
			info->after_af = 1;
			DCAM_TRACE("time, %d %d\n",
				   (int)info->timestamp.tv_sec,
				   (int)info->timestamp.tv_usec);
		}
		sprd_img_setflash(dev->idx, &info->set_flash);
		info->set_flash.led0_ctrl = 0;
		info->set_flash.led1_ctrl = 0;
		info->set_flash.led0_status = FLASH_STATUS_MAX;
		info->set_flash.led1_status = FLASH_STATUS_MAX;
	}

	return 0;
}

static int sprd_img_start_flash(struct camera_frame *frame, void *param)
{
	struct camera_dev *dev = (struct camera_dev *)param;
	struct camera_info *info = NULL;
	unsigned int need_light = 1;
	unsigned int led0_ctrl;
	unsigned int led1_ctrl;
	unsigned int led0_status;
	unsigned int led1_status;

	if (dev == NULL) {
		DCAM_TRACE("dev is NULL\n");
		return -1;
	}

	info = &dev->dcam_cxt;
	led0_ctrl = info->set_flash.led0_ctrl;
	led1_ctrl = info->set_flash.led1_ctrl;
	led0_status = info->set_flash.led0_status;
	led1_status = info->set_flash.led1_status;

	if ((led0_ctrl && led0_status < FLASH_STATUS_MAX) ||
		(led1_ctrl && led1_status < FLASH_STATUS_MAX)) {
		if ((led0_ctrl && FLASH_HIGH_LIGHT == led0_status) ||
			(led1_ctrl && FLASH_HIGH_LIGHT == led1_status)) {
			dev->frame_skipped++;
			if (dev->frame_skipped >= info->skip_number) {
				/* flash lighted at the last SOF before
				* the right capture frame
				*/
				DCAM_TRACE("waiting finished\n");
			} else {
				need_light = 0;
				DCAM_TRACE("wait for the next SOF, %d %d\n",
					dev->frame_skipped,
					info->skip_number);
			}
		}
		if (need_light)
			complete(&dev->flash_thread_com);
	}

	return 0;
}

static int flash_thread_loop(void *arg)
{
	struct camera_dev *dev = (struct camera_dev *)arg;
	struct sprd_img_set_flash set_flash;

	if (dev == NULL) {
		DCAM_TRACE("flash_thread_loop, dev is NULL\n");
		return -1;
	}
	while (1) {
		if (wait_for_completion_interruptible(
			&dev->flash_thread_com) == 0) {
			if (dev->is_flash_thread_stop) {
				set_flash.led0_ctrl = 1;
				set_flash.led1_ctrl = 1;
				set_flash.led0_status = FLASH_CLOSE;
				set_flash.led1_status = FLASH_CLOSE;
				set_flash.flash_index = 0;
				sprd_img_setflash(dev->idx, &set_flash);
				set_flash.flash_index = 1;
				sprd_img_setflash(dev->idx, &set_flash);
				DCAM_TRACE("flash_thread_loop stop\n");
				break;
			}
			sprd_img_opt_flash(NULL, arg);
		} else {
			DCAM_TRACE("flash int!");
			break;
		}
	}
	dev->is_flash_thread_stop = 0;

	return 0;
}

static int dcam_create_flash_thread(void *param)
{
	struct camera_dev *dev = (struct camera_dev *)param;
	char thread_name[20] = { 0 };

	if (dev == NULL) {
		DCAM_TRACE("dev is NULL\n");
		return -1;
	}
	dev->is_flash_thread_stop = 0;
	init_completion(&dev->flash_thread_com);
	sprintf(thread_name, "dcam%d_flash_thread", dev->idx);
	dev->flash_thread = kthread_run(flash_thread_loop, param, thread_name);
	if (IS_ERR(dev->flash_thread)) {
		pr_err("dcam_create_flash_thread error!\n");
		return -1;
	}

	return 0;
}

static int dcam_stop_flash_thread(void *param)
{
	struct camera_dev *dev = (struct camera_dev *)param;
	int cnt = 0;

	if (dev == NULL) {
		DCAM_TRACE("dev is NULL\n");
		return -1;
	}

	if (dev->flash_thread) {
		dev->is_flash_thread_stop = 1;
		complete(&dev->flash_thread_com);
		if (dev->is_flash_thread_stop != 0) {
			while (cnt < 500) {
				cnt++;
				if (dev->is_flash_thread_stop == 0)
					break;
				udelay(1000);
			}
		}
		dev->flash_thread = NULL;
	}

	return 0;
}

static int sprd_img_get_path_index(unsigned int channel_id)
{
	int path_index;

	switch (channel_id) {
	case CAMERA_PRE_PATH:
		path_index = ISP_PATH_IDX_PRE;
		break;
	case CAMERA_VID_PATH:
		path_index = ISP_PATH_IDX_VID;
		break;
	case CAMERA_CAP_PATH:
		path_index = ISP_PATH_IDX_CAP;
		break;
	default:
		path_index = ISP_PATH_IDX_ALL;
		pr_info("failed to get path index, channel %d\n", channel_id);
	}

	return path_index;
}

static int sprd_img_queue_init(struct camera_queue *queue)
{
	unsigned long flags;

	if (queue == NULL)
		return -EINVAL;
	spin_lock_irqsave(&queue->lock, flags);
	memset(queue, 0, sizeof(*queue));
	queue->write = &queue->node[0];
	queue->read = &queue->node[0];
	queue->wcnt = 0;
	queue->rcnt = 0;
	spin_unlock_irqrestore(&queue->lock, flags);
	return 0;
}

static int sprd_img_local_deinit(struct camera_dev *dev)
{
	int ret = 0;
	int i;
	struct camera_path_spec *path;

	for (i = 0; i < CAMERA_MAX_PATH; i++) {
		path = &dev->dcam_cxt.dcam_path[i];
		DCAM_TRACE("local_deinit, path %d cnt %d\n",
			   i, path->frm_cnt_act);
		if (unlikely(NULL == dev || NULL == path))
			return -EINVAL;

		path->is_work = 0;
		path->frm_cnt_act = 0;
		sprd_img_buf_queue_init(&path->buf_queue);
		sprd_img_buf_queue_init(&path->tmp_buf_queue);
	}

	ret = sprd_img_queue_init(&dev->queue);
	if (unlikely(ret != 0)) {
		pr_err("failed to init queue\n");
		return -EINVAL;
	}

	DCAM_TRACE("local_deinit, frm_cnt_act %d\n",
		   path->frm_cnt_act);

	return 0;
}

static int sprd_dcam_cfg_path0(struct camera_path_spec *path0, enum dcam_id idx)
{
	int ret = 0;
	struct camera_img_buf_addr *cur_node;
	struct camera_img_buf_queue *queue;
	struct camera_addr frm_addr;
	unsigned int param;

	if (path0 == NULL) {
		ret = -EINVAL;
		goto exit;
	}

	ret = set_dcam_path0_cfg(idx, DCAM_PATH_FRM_DECI,
		&path0->path_frm_deci);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_dcam_path0_cfg(idx, DCAM_PATH_DATA_ENDIAN, &path0->end_sel);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_dcam_path0_cfg(idx, DCAM_PATH_INPUT_RECT, &path0->in_rect);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	queue = &path0->buf_queue;
	for (cur_node = queue->read; cur_node != queue->write; cur_node++) {
		if (cur_node > &queue->buf_addr[DCAM_FRM_CNT_MAX - 1])
			cur_node = &queue->buf_addr[0];

		frm_addr.yaddr = cur_node->frm_addr.yaddr;
		frm_addr.uaddr = cur_node->frm_addr.uaddr;
		frm_addr.vaddr = cur_node->frm_addr.vaddr;
		frm_addr.yaddr_vir = cur_node->frm_addr_vir.yaddr;
		frm_addr.uaddr_vir = cur_node->frm_addr_vir.uaddr;
		frm_addr.vaddr_vir = cur_node->frm_addr_vir.vaddr;
		frm_addr.mfd_y = cur_node->frm_addr.mfd_y;
		frm_addr.mfd_u = cur_node->frm_addr.mfd_u;
		frm_addr.mfd_v = cur_node->frm_addr.mfd_v;
		ret = set_dcam_path0_cfg(idx, DCAM_PATH0_OUTPUT_ADDR,
			&frm_addr);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	}

	ret = set_dcam_path0_cfg(idx, DCAM_PATH_OUTPUT_RESERVED_ADDR,
		&path0->frm_reserved_addr);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	param = 1;
	ret = set_dcam_path0_cfg(idx, DCAM_PATH_ENABLE, &param);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

exit:
	return ret;
}

static int sprd_dcam_cfg_path_pdaf(struct camera_path_spec *path_pdaf,
	enum dcam_id idx)
{
	int ret = 0;
	struct camera_img_buf_addr *cur_node;
	struct camera_img_buf_queue *queue;
	struct camera_addr frm_addr;
	unsigned int param;

	if (path_pdaf == NULL) {
		ret = -EINVAL;
		goto exit;
	}

	queue = &path_pdaf->buf_queue;
	for (cur_node = queue->read; cur_node != queue->write; cur_node++) {
		if (cur_node > &queue->buf_addr[DCAM_FRM_CNT_MAX - 1])
			cur_node = &queue->buf_addr[0];

		frm_addr.yaddr = cur_node->frm_addr.yaddr;
		frm_addr.uaddr = cur_node->frm_addr.uaddr;
		frm_addr.vaddr = cur_node->frm_addr.vaddr;
		frm_addr.yaddr_vir = cur_node->frm_addr_vir.yaddr;
		frm_addr.uaddr_vir = cur_node->frm_addr_vir.uaddr;
		frm_addr.vaddr_vir = cur_node->frm_addr_vir.vaddr;
		frm_addr.mfd_y = cur_node->frm_addr.mfd_y;
		frm_addr.mfd_u = cur_node->frm_addr.mfd_u;
		frm_addr.mfd_v = cur_node->frm_addr.mfd_v;
		ret = set_dcam_path_pdaf_cfg(idx, DCAM_PATH_PDAF_OUTPUT_ADDR,
			&frm_addr);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	}

	ret = set_dcam_path_pdaf_cfg(idx, DCAM_PATH_OUTPUT_RESERVED_ADDR,
		&path_pdaf->frm_reserved_addr);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_dcam_path_pdaf_cfg(idx, DCAM_PDAF_CONTROL,
		&path_pdaf->pdaf_ctrl);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	param = 1;
	ret = set_dcam_path_pdaf_cfg(idx, DCAM_PATH_ENABLE, &param);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

exit:
	return ret;
}

static int set_isp_path_buf_cfg(void *handle, enum isp_cfg_id cfg_id,
	enum isp_path_index path_index, struct camera_img_buf_queue *queue)
{
	int ret = 0;
	struct camera_img_buf_addr *cur_node;
	struct camera_addr frm_addr;

	if (!queue || !handle) {
		ret = -EINVAL;
		goto exit;
	}

	for (cur_node = queue->read; cur_node != queue->write; cur_node++) {
		if (cur_node > &queue->buf_addr[DCAM_FRM_CNT_MAX - 1])
			cur_node = &queue->buf_addr[0];

		frm_addr.yaddr = cur_node->frm_addr.yaddr;
		frm_addr.uaddr = cur_node->frm_addr.uaddr;
		frm_addr.vaddr = cur_node->frm_addr.vaddr;
		frm_addr.yaddr_vir = cur_node->frm_addr_vir.yaddr;
		frm_addr.uaddr_vir = cur_node->frm_addr_vir.uaddr;
		frm_addr.vaddr_vir = cur_node->frm_addr_vir.vaddr;
		frm_addr.mfd_y = cur_node->frm_addr.mfd_y;
		frm_addr.mfd_u = cur_node->frm_addr.mfd_u;
		frm_addr.mfd_v = cur_node->frm_addr.mfd_v;
		ret = set_isp_path_cfg(handle, path_index, cfg_id, &frm_addr);
	}

exit:
	return ret;
}

static int sprd_isp_path_cfg_block(struct camera_path_spec *path,
	void *handle, enum isp_path_index path_index)
{
	int ret = 0;
	unsigned int param = 0;
	struct isp_endian_sel endian;
	struct isp_regular_info regular_info;

	memset(&endian, 0, sizeof(struct isp_endian_sel));

	if (!path || !handle) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}

	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_INPUT_SIZE,
		&path->in_size);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_INPUT_RECT,
		&path->in_rect);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_OUTPUT_SIZE,
		&path->out_size);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_OUTPUT_FORMAT,
		&path->out_fmt);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_isp_path_buf_cfg(handle, ISP_PATH_OUTPUT_ADDR, path_index,
		&path->buf_queue);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_isp_path_buf_cfg(handle, ISP_PATH_STORE_CCE_ADDR, path_index,
		&path->tmp_buf_queue);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_isp_path_cfg(handle, path_index,
		ISP_PATH_OUTPUT_RESERVED_ADDR, &path->frm_reserved_addr);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_FRM_DECI,
		&path->path_frm_deci);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_MODE,
		&path->path_mode);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}
	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_SKIP_NUM,
				&path->skip_num);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	memset(&regular_info, 0, sizeof(regular_info));
	regular_info.regular_mode = path->regular_desc.regular_mode;
	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_SHRINK,
			       &regular_info);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	param = 1;
	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_ENABLE, &param);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	endian.y_endian = path->end_sel.y_endian;
	endian.uv_endian = path->end_sel.uv_endian;
	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_DATA_ENDIAN,
		&endian);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}
exit:
	return ret;
}

static int sprd_img_check_path0_cap(unsigned int fourcc,
				    struct sprd_img_format *f,
				    struct camera_info *info)
{
	struct camera_path_spec *path0 = &info->dcam_path[CAMERA_PATH0];

	DCAM_TRACE("check format for path0\n");

	path0->frm_type = f->channel_id;
	path0->is_work = 0;

	switch (fourcc) {
	case IMG_PIX_FMT_GREY:
		path0->out_fmt = DCAM_RAWRGB;
		path0->pixel_depth = info->is_loose ? 16 : 10;
		path0->end_sel.y_endian = DCAM_ENDIAN_BIG;
		break;

	default:
		pr_info("unsupported image format for path0 0x%x\n", fourcc);
		return -EINVAL;
	}
	path0->fourcc = fourcc;

	DCAM_TRACE("check format for path0: out_fmt=%d, is_loose=%d\n",
		   path0->out_fmt, info->is_loose);
	path0->out_size.w = f->width;
	path0->out_size.h = f->height;

	path0->is_work = 1;

	return 0;
}

static int sprd_img_check_path_pdaf_cap(unsigned int fourcc,
				    struct sprd_img_format *f,
				    struct camera_info *info)
{
	struct camera_path_spec *path_pdaf = &info->dcam_path[CAMERA_PDAF_PATH];

	DCAM_TRACE("check format for path pdaf\n");

	path_pdaf->frm_type = f->channel_id;
	path_pdaf->is_work = 0;
	/* need add pdaf path necessary info here*/
	path_pdaf->is_work = 1;

	return 0;
}

static struct camera_format *sprd_img_get_format(unsigned int fourcc)
{
	struct camera_format *fmt;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dcam_img_fmt); i++) {
		fmt = &dcam_img_fmt[i];
		if (fmt->fourcc == fourcc)
			break;
	}

	if (unlikely(i == ARRAY_SIZE(dcam_img_fmt)))
		return NULL;

	return &dcam_img_fmt[i];
}

static int sprd_img_check_binning(struct sprd_img_format *f,
				  struct camera_info *info,
				  struct camera_path_spec *path)
{
	unsigned int tempw;

	if (info->cap_out_size.w > DCAM_ISP_LINE_BUF_LENGTH) {
		if (info->if_mode == DCAM_CAP_IF_CCIR) {
			/* CCIR CAP, no bining */
			pr_err("CCIR CAP, no bining\n");
			return -EINVAL;
		} else if (info->if_mode == DCAM_CAP_IF_CSI2) {
			/* MIPI CAP,
			 * support 1/2 binning
			 */
			DCAM_TRACE("Need Binning\n");
			tempw = path->in_rect.w;
			tempw = tempw >> 1;
			if (tempw > DCAM_ISP_LINE_BUF_LENGTH)
				return -EINVAL;

			info->img_deci.x_factor = 1;
			f->need_binning = 1;
			path->in_size.w = path->in_size.w >> 1;
			path->in_rect.x = path->in_rect.x >> 1;
			path->in_rect.w = path->in_rect.w >> 1;
			path->in_rect.w = path->in_rect.w & (~3);
		}
	}

	return 0;
}

static int sprd_img_check_scaling(struct sprd_img_format *f,
				  struct camera_info *info,
				  struct camera_path_spec *path,
				  unsigned int line_buf_size)
{
	unsigned int maxw, maxh, tempw, temph;

	tempw = path->in_rect.w;
	temph = path->in_rect.h;

	/* no need to scale */
	if (tempw == f->width && temph == f->height)
		return 0;

	/* scaling needed */
	switch (info->sn_mode) {
	case DCAM_CAP_MODE_RAWRGB:
		maxw = f->width * CAMERA_SC_COEFF_DOWN_MAX;
		maxw = maxw * (1 << CAMERA_PATH_DECI_FAC_MAX);
		maxh = f->height * CAMERA_SC_COEFF_DOWN_MAX;
		maxh = maxh * (1 << CAMERA_PATH_DECI_FAC_MAX);
		if (unlikely(tempw > maxw || temph > maxh)) {
			/* out of scaling capbility */
			pr_err("out of scaling capbility\n");
			return -EINVAL;
		}

		if (unlikely(f->width > line_buf_size)) {
			/* out of scaling capbility. TBD */
			pr_debug("out of scaling capbility\n");
		}

		maxw = tempw * CAMERA_SC_COEFF_UP_MAX;
		maxh = temph * CAMERA_SC_COEFF_UP_MAX;
		if (unlikely(f->width > maxw || f->height > maxh)) {
			/* out of scaling capbility */
			pr_err("out of scaling capbility\n");
			return -EINVAL;
		}
		break;

	default:
		break;
	}

	return 0;
}

static void sprd_img_endian_sel(unsigned int fourcc,
					struct camera_path_spec *path)
{
	if (fourcc == IMG_PIX_FMT_YUV422P ||
	    fourcc == IMG_PIX_FMT_RGB565 || fourcc == IMG_PIX_FMT_RGB565X) {
		if (fourcc == IMG_PIX_FMT_YUV422P) {
			path->out_fmt = DCAM_YUV422;
		} else {
			path->out_fmt = DCAM_RGB565;
			if (fourcc == IMG_PIX_FMT_RGB565)
				path->end_sel.y_endian = DCAM_ENDIAN_HALFBIG;
			else
				path->end_sel.y_endian = DCAM_ENDIAN_BIG;
		}
	} else {
		if (fourcc == IMG_PIX_FMT_YUV420 ||
		    fourcc == IMG_PIX_FMT_YVU420) {
			path->out_fmt = DCAM_YUV420_3FRAME;
		} else {
			if (fourcc == IMG_PIX_FMT_NV12) {
				path->out_fmt = DCAM_YVU420;
				path->end_sel.uv_endian = DCAM_ENDIAN_LITTLE;
			} else {
				path->out_fmt = DCAM_YUV420;
				path->end_sel.uv_endian = DCAM_ENDIAN_LITTLE;
			}

		}
	}

}

static int sprd_img_check_path_cap(enum dcam_id idx, unsigned int fourcc,
				struct sprd_img_format *f,
				struct camera_info *info,
				enum camera_path_id path_id)
{
	int ret = 0;
	unsigned int tempw, temph;
	unsigned int line_buf_size;
	struct camera_path_spec *path;

	DCAM_TRACE("check format for path%d\n", path_id);

	switch (path_id) {
	case CAMERA_PRE_PATH:
		if (idx == DCAM_ID_1)
			line_buf_size = ISP1_PATH1_LINE_BUF_LENGTH;
		else
			line_buf_size = ISP_PATH1_LINE_BUF_LENGTH;
		break;
	case CAMERA_VID_PATH:
		if (idx == DCAM_ID_1)
			line_buf_size = ISP1_PATH2_LINE_BUF_LENGTH;
		else
			line_buf_size = ISP_PATH2_LINE_BUF_LENGTH;
		break;
	case CAMERA_CAP_PATH:
		if (idx == DCAM_ID_1)
			line_buf_size = ISP1_PATH3_LINE_BUF_LENGTH;
		else
			line_buf_size = ISP_PATH3_LINE_BUF_LENGTH;
		break;
	default:
		return -EINVAL;
	}

	path = &info->dcam_path[path_id];
	path->frm_type = f->channel_id;
	path->is_from_isp = f->need_isp;
	path->rot_mode = f->flip_on;
	path->end_sel.y_endian = DCAM_ENDIAN_LITTLE;
	path->end_sel.uv_endian = DCAM_ENDIAN_LITTLE;
	path->is_work = 0;
	path->pixel_depth = 0;
	path->img_deci.x_factor = 0;
	path->img_deci.y_factor = 0;
	tempw = path->in_rect.w;
	temph = path->in_rect.h;
	info->img_deci.x_factor = 0;
	f->need_binning = 0;
	/* app should fill in this field(fmt.pix.priv) to set the base index
	 * of frame buffer, and lately this field will return the flag
	 * whether ISP is needed for this work path
	 */
	switch (fourcc) {
	case IMG_PIX_FMT_GREY:
	case IMG_PIX_FMT_JPEG:
	case IMG_PIX_FMT_YUYV:
	case IMG_PIX_FMT_YVYU:
	case IMG_PIX_FMT_UYVY:
	case IMG_PIX_FMT_VYUY:
		if (unlikely(f->width != tempw || f->height != temph)) {
			/* need need scaling or triming */
			pr_err("can't scaling, src %d %d, dst %d %d\n",
			       tempw, temph, f->width, f->height);
			return -EINVAL;
		}

		if (fourcc == IMG_PIX_FMT_GREY) {
			if (unlikely(info->sn_mode != DCAM_CAP_MODE_RAWRGB)) {
				/* the output of sensor is not RawRGB
				 * which is needed by app
				 */
				pr_err("It's not RawRGB sensor\n");
				return -EINVAL;
			}

			path->out_fmt = DCAM_RAWRGB;
			path->end_sel.y_endian = DCAM_ENDIAN_BIG;
		} else if (fourcc == IMG_PIX_FMT_JPEG) {
			if (unlikely(info->sn_mode != DCAM_CAP_MODE_JPEG)) {
				/* the output of sensor is not JPEG
				 * which is needed by app
				 */
				pr_err("It's not JPEG sensor\n");
				return -EINVAL;
			}
			path->out_fmt = DCAM_JPEG;
		}
		break;
	case IMG_PIX_FMT_YUV422P:
	case IMG_PIX_FMT_YUV420:
	case IMG_PIX_FMT_YVU420:
	case IMG_PIX_FMT_NV12:
	case IMG_PIX_FMT_NV21:
	case IMG_PIX_FMT_RGB565:
	case IMG_PIX_FMT_RGB565X:
		if (info->sn_mode == DCAM_CAP_MODE_RAWRGB) {
			if (path->is_from_isp) {
				/* check binning */
				ret = sprd_img_check_binning(f, info, path);
				if (ret)
					return ret;

				/* check scaling */
				ret = sprd_img_check_scaling(f, info, path,
							     line_buf_size);
				if (ret)
					return ret;
			} else {
				/* no isp, only RawRGB data can be sampled */
				pr_err("RawRGB sensor, no ISP, format 0x%x\n",
				       fourcc);
				return -EINVAL;
			}
		} else {
			pr_err("sensor mode is 0x%x not support\n",
				info->sn_mode);
			return -EINVAL;
		}

		sprd_img_endian_sel(fourcc, path);
		break;
	default:
		pr_info("unsupported image format for path %d 0x%x\n",
			path_id, fourcc);
		return -EINVAL;
	}

	path->fourcc = fourcc;
	path->out_size.w = f->width;
	path->out_size.h = f->height;
	path->is_work = 1;

	return 0;
}

static int sprd_img_queue_write(struct camera_queue *queue,
				struct camera_node *node)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_node *ori_node;
	unsigned long flags;

	if (NULL == queue || NULL == node)
		return -EINVAL;
	spin_lock_irqsave(&queue->lock, flags);
	ori_node = queue->write;
	queue->wcnt++;
	*queue->write++ = *node;
	if (queue->write > &queue->node[CAMERA_QUEUE_LENGTH - 1])
		queue->write = &queue->node[0];

	if (queue->write == queue->read) {
		queue->write = ori_node;
		pr_info("warning, queue is full\n");
		ret = -EINVAL;
	}
	spin_unlock_irqrestore(&queue->lock, flags);
	return ret;
}

static int sprd_img_queue_read(struct camera_queue *queue, struct camera_node *
node)
{
	int ret = DCAM_RTN_SUCCESS;
	int flag = 0;
	unsigned long flags;

	if (NULL == queue || NULL == node)
		return -EINVAL;

	spin_lock_irqsave(&queue->lock, flags);
	if (queue->read != queue->write) {
		flag = 1;
		*node = *queue->read;
		queue->read->yaddr = 0;
		queue->read->yaddr_vir = 0;
		queue->read++;
		queue->rcnt++;
		if (queue->read > &queue->node[CAMERA_QUEUE_LENGTH - 1])
			queue->read = &queue->node[0];
	}

	if (!flag)
		ret = EAGAIN;

	spin_unlock_irqrestore(&queue->lock, flags);
	return ret;
}

static int sprd_img_buf_queue_write(struct camera_img_buf_queue *queue,
				    struct camera_img_buf_addr *buf_addr)
{
	struct camera_img_buf_addr *ori_node;

	if (NULL == queue || NULL == buf_addr)
		return -EINVAL;

	ori_node = queue->write;
	*queue->write++ = *buf_addr;
	queue->wcnt++;
	if (queue->write > &queue->buf_addr[DCAM_FRM_CNT_MAX - 1])
		queue->write = &queue->buf_addr[0];

	if (queue->write == queue->read) {
		queue->write = ori_node;
		pr_info("warning, queue is full\n");
	}

	return 0;
}

static int sprd_img_tx_error(struct camera_frame *frame, void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_dev *dev = (struct camera_dev *)param;
	struct camera_node node;

	if (NULL == param || 0 == atomic_read(&dev->stream_on))
		return -EINVAL;

	memset((void *)&node, 0, sizeof(struct camera_node));
	atomic_set(&dev->run_flag, DCAM_VALID_FRAME_COUNT);
	node.irq_flag = IMG_TX_ERR;
	node.irq_type = CAMERA_IRQ_IMG;
	node.irq_property = IRQ_MAX_DONE;
	if (frame != NULL) {
		node.f_type = frame->type;
		node.index = frame->fid;
		node.height = frame->height;
		node.yaddr = frame->yaddr;
		node.uaddr = frame->uaddr;
		node.vaddr = frame->vaddr;
		node.yaddr_vir = frame->yaddr_vir;
		node.uaddr_vir = frame->uaddr_vir;
		node.vaddr_vir = frame->vaddr_vir;
	}
	ret = sprd_img_queue_write(&dev->queue, &node);
	if (ret)
		return ret;

	complete(&dev->irq_com);

	pr_info("tx error\n");

	return ret;
}

static int sprd_img_tx_done(struct camera_frame *frame, void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_dev *dev = (struct camera_dev *)param;
	struct camera_path_spec *path;
	struct camera_node node;

	if (NULL == frame || NULL == param || 0 == atomic_read(&dev->stream_on))
		return -EINVAL;

	if (atomic_read(&dev->run_flag)  <=  DCAM_VALID_FRAME_COUNT)
		atomic_inc(&dev->run_flag);

	memset((void *)&node, 0, sizeof(struct camera_node));

	if (frame->irq_type == CAMERA_IRQ_IMG) {
		path = &dev->dcam_cxt.dcam_path[frame->type];
		if (path->status == PATH_IDLE) {
			pr_debug("%s path idle\n", __func__);
			return ret;
		}

		node.irq_flag = IMG_TX_DONE;
		node.irq_type = CAMERA_IRQ_IMG;
		node.irq_property = IRQ_MAX_DONE;
		node.f_type = frame->type;
		node.index = frame->fid;
		node.height = frame->height;
		node.yaddr = frame->yaddr;
		node.uaddr = frame->uaddr;
		node.vaddr = frame->vaddr;
		node.yaddr_vir = frame->yaddr_vir;
		node.uaddr_vir = frame->uaddr_vir;
		node.vaddr_vir = frame->vaddr_vir;
		memcpy(node.mfd, frame->pfinfo.mfd, sizeof(unsigned int) * 3);
		pr_debug("%s CAMERA_IRQ_IMG mfd: %d, %d, %d\n", __func__,
				node.mfd[0], node.mfd[1], node.mfd[2]);
	} else if (frame->irq_type == CAMERA_IRQ_STATIS) {
		node.irq_flag = IMG_TX_DONE;
		node.irq_type = CAMERA_IRQ_STATIS;
		node.irq_property = frame->irq_property;
		node.phy_addr = frame->phy_addr;
		node.vir_addr = frame->vir_addr;
		node.kaddr[0] = frame->kaddr[0];
		node.kaddr[1] = frame->kaddr[1];
		node.buf_size = frame->buf_size;
		memcpy(node.mfd, frame->pfinfo.mfd, sizeof(unsigned int) * 3);
		pr_debug("%s CAMERA_IRQ_STATIS mfd: %d, %d, %d\n", __func__,
				node.mfd[0], node.mfd[1], node.mfd[2]);
	} else if (frame->irq_type == CAMERA_IRQ_DONE) {
		node.irq_flag = IMG_TX_DONE;
		node.irq_type = CAMERA_IRQ_DONE;
		node.irq_property = frame->irq_property;
		pr_debug("%s CAMERA_IRQ_DONE, irq_property:%d\n", __func__,
				node.irq_property);
	} else if (frame->irq_type == CAMERA_IRQ_3DNR_DONE) {
		node.irq_flag = IMG_TX_DONE;
		node.irq_type = CAMERA_IRQ_3DNR_DONE;
		node.irq_property = frame->irq_property;
	} else {
		pr_err("%s Not support irq_type %d\n", __func__,
				frame->irq_type);
		return -EINVAL;
	}
	node.boot_time = ktime_get_boottime();
	img_get_timestamp(&node.time);

	DCAM_TRACE("flag 0x%x type 0x%x\n",
		   node.irq_flag, node.irq_type);

	ret = sprd_img_queue_write(&dev->queue, &node);
	if (ret)
		return ret;

	DCAM_TRACE("sprd_img %d %d %p\n", dev->idx, frame->type, dev);
	if (node.irq_property == IRQ_RAW_CAP_DONE)
		pr_info("dcam_core: RAW CAP tx done flag %d type %d\n",
			node.irq_flag, node.irq_type);

	complete(&dev->irq_com);

	return ret;
}

static int sprd_img_tx_stop(void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_dev *dev = (struct camera_dev *)param;
	struct camera_node node;

	memset((void *)&node, 0, sizeof(struct camera_node));
	node.irq_flag = IMG_TX_STOP;
	ret = sprd_img_queue_write(&dev->queue, &node);
	if (ret)
		return ret;

	complete(&dev->irq_com);

	pr_info("tx stop\n");

	return ret;
}

static int img_get_zoom_rect(struct camera_rect *src_rect,
			     struct camera_rect *dst_rect,
			     struct camera_rect *output_rect,
			     unsigned int zoom_level)
{
	unsigned int trim_width = 0;
	unsigned int trim_height = 0;
	unsigned int zoom_step_w = 0, zoom_step_h = 0;

	if (NULL == src_rect || NULL == dst_rect || NULL == output_rect) {
		pr_err("%p, %p, %p\n", src_rect, dst_rect, output_rect);
		return -EINVAL;
	}

	if (0 == dst_rect->w || 0 == dst_rect->h) {
		pr_err("dst rect, w %d, h %d\n", dst_rect->w, dst_rect->h);
		return -EINVAL;
	}

	if (src_rect->w > dst_rect->w && src_rect->h > dst_rect->h) {
		zoom_step_w = CAMERA_ZOOM_STEP(src_rect->w, dst_rect->w);
		zoom_step_w &= ~1;
		zoom_step_w *= zoom_level;
		trim_width = src_rect->w - zoom_step_w;

		zoom_step_h = CAMERA_ZOOM_STEP(src_rect->h, dst_rect->h);
		zoom_step_h &= ~1;
		zoom_step_h *= zoom_level;
		trim_height = src_rect->h - zoom_step_h;

		output_rect->x =
			src_rect->x + ((src_rect->w - trim_width) >> 1);
		output_rect->y =
			src_rect->y + ((src_rect->h - trim_height) >> 1);
	} else if (src_rect->w < dst_rect->w && src_rect->h < dst_rect->h) {
		zoom_step_w = CAMERA_ZOOM_STEP(dst_rect->w, src_rect->w);
		zoom_step_w &= ~1;
		zoom_step_w *= zoom_level;
		trim_width = src_rect->w + zoom_step_w;

		zoom_step_h = CAMERA_ZOOM_STEP(dst_rect->h, src_rect->h);
		zoom_step_h &= ~1;
		zoom_step_h *= zoom_level;
		trim_height = src_rect->h + zoom_step_h;

		output_rect->x =
			src_rect->x - ((trim_width - src_rect->w) >> 1);
		output_rect->y =
			src_rect->y - ((trim_height - src_rect->h) >> 1);
	} else {
		pr_err("parm error\n");
		return -EINVAL;
	}

	output_rect->x = CAMERA_WIDTH(output_rect->x);
	output_rect->y = CAMERA_HEIGHT(output_rect->y);
	output_rect->w = CAMERA_WIDTH(trim_width);
	output_rect->h = CAMERA_HEIGHT(trim_height);
	DCAM_TRACE("zoom_level %d, trim rect, %d %d %d %d\n",
		   zoom_level,
		   output_rect->x,
		   output_rect->y, output_rect->w, output_rect->h);

	return 0;
}


static int sprd_img_zoom_thread_loop(void *arg)
{
	struct camera_dev *dev = (struct camera_dev *)arg;
	int ret = DCAM_RTN_SUCCESS;
	struct camera_rect zoom_rect = { 0 };
	struct camera_path_spec *path = NULL;
	enum isp_path_index path_index;

	if (dev == NULL) {
		DCAM_TRACE("dev is NULL\n");
		return -1;
	}

	while (1) {
		/* zoom_thread_com up not added*/
		if (wait_for_completion_interruptible(
					&dev->zoom_thread_com) == 0) {
			DCAM_TRACE("zoom thread level %d\n", dev->zoom_level);
			if (dev->is_zoom_thread_stop) {
				DCAM_TRACE("stop zoom thread\n");
				break;
			}
			if (dev->zoom_level > CAMERA_ZOOM_LEVEL_MAX)
				continue;

			mutex_lock(&dev->dcam_mutex);
			path = &dev->dcam_cxt.dcam_path[dev->channel_id];
			path_index = sprd_img_get_path_index(dev->channel_id);
			if (dev->zoom_level < CAMERA_ZOOM_LEVEL_MAX) {
				ret = img_get_zoom_rect(&path->in_rect_backup,
							&path->in_rect,
							&zoom_rect,
							dev->zoom_level);
				if (!ret) {
					memcpy((void *)&path->in_rect_current,
						(void *)&zoom_rect,
						sizeof(struct camera_rect));
					sprd_isp_update_path(
						dev->isp_dev_handle,
						path_index, &path->in_size,
						&zoom_rect, &path->out_size);
				}
			} else {
				sprd_isp_update_path(dev->isp_dev_handle,
						path_index,
						&path->in_size,
						&path->in_rect,
						&path->out_size);
				memcpy((void *)&path->in_rect_backup,
						(void *)&path->in_rect,
						sizeof(struct camera_rect));
				memcpy((void *)&path->in_rect_current,
						(void *)&path->in_rect_backup,
						sizeof(struct camera_rect));
			}
			dev->zoom_level++;
			mutex_unlock(&dev->dcam_mutex);
			DCAM_TRACE("zoom thread level %d end\n",
				   dev->zoom_level);

		} else {
			pr_info("zoom int!");
			break;
		}
	}
	dev->is_zoom_thread_stop = 0;


	return 0;
}

static int sprd_img_create_zoom_thread(void *param)
{
	struct camera_dev *dev = (struct camera_dev *)param;

	if (dev == NULL) {
		DCAM_TRACE("dev is NULL\n");
		return -1;
	}

	dev->is_zoom_thread_stop = 0;
	dev->zoom_level = CAMERA_ZOOM_LEVEL_MAX + 1;
	init_completion(&dev->zoom_thread_com);
	dev->zoom_thread = kthread_run(sprd_img_zoom_thread_loop, param,
				       "img_zoom_thread");
	if (IS_ERR(dev->zoom_thread)) {
		pr_err("failed to create zoom thread\n");
		return -1;
	}

	return 0;
}

static int sprd_img_stop_zoom_thread(void *param)
{
	struct camera_dev *dev = (struct camera_dev *)param;
	int cnt = 0;

	if (dev == NULL) {
		DCAM_TRACE("dev is NULL\n");
		return -1;
	}

	DCAM_TRACE("stop zoom thread\n");
	if (dev->zoom_thread) {
		dev->is_zoom_thread_stop = 1;
		complete(&dev->zoom_thread_com);
		if (dev->is_zoom_thread_stop != 0) {
			while (cnt < 500) {
				cnt++;
				if (dev->is_zoom_thread_stop == 0)
					break;
				udelay(1000);
			}
		}
		dev->zoom_thread = NULL;
	}

	return 0;
}

static void sprd_timer_callback(unsigned long data)
{
	struct camera_dev *dev = (struct camera_dev *)data;
	struct camera_node node;
	struct camera_path_spec *path_0 = NULL;
	int ret = 0;

	path_0 = &dev->dcam_cxt.dcam_path[CAMERA_PATH0];
	memset((void *)&node, 0, sizeof(node));
	if (0 == data || 0 == atomic_read(&dev->stream_on)) {
		pr_err("timer cb error\n");
		return;
	}

	if ((atomic_read(&dev->run_flag) == 0) && (path_0->is_work == 0)) {
		pr_info("DCAM timeout.\n");
		node.irq_flag = IMG_TIMEOUT;
		node.irq_type = CAMERA_IRQ_IMG;
		node.irq_property = IRQ_MAX_DONE;
		node.invalid_flag = 0;
		ret = sprd_img_queue_write(&dev->queue, &node);
		if (ret)
			pr_err("timer cb write queue error\n");

		complete(&dev->irq_com);
	}
}

static void sprd_init_timer(struct timer_list *dcam_timer, unsigned long data)
{
	setup_timer(dcam_timer, sprd_timer_callback, data);
}

static int sprd_start_timer(struct timer_list *dcam_timer,
			    unsigned int time_val)
{
	int ret = 0;

	DCAM_TRACE("starting timer %ld\n", jiffies);
	ret = mod_timer(dcam_timer, jiffies + msecs_to_jiffies(time_val));
	if (ret)
		pr_err("Error in mod_timer %d\n", ret);

	return ret;
}

static int sprd_stop_timer(struct timer_list *dcam_timer)
{
	DCAM_TRACE("stop timer\n");
	del_timer_sync(dcam_timer);

	return 0;
}

static int sprd_init_handle(struct camera_dev *dev)
{
	struct camera_info *info = &dev->dcam_cxt;
	struct camera_path_spec *path;
	unsigned int i = 0;

	if (info == NULL) {
		pr_err("info is null\n");
		return -EINVAL;
	}

	info->set_flash.led0_ctrl = 0;
	info->set_flash.led1_ctrl = 0;
	info->set_flash.led0_status = FLASH_STATUS_MAX;
	info->set_flash.led1_status = FLASH_STATUS_MAX;
	info->set_flash.flash_index = 0;
	info->after_af = 0;

	for (i = 0; i < CAMERA_MAX_PATH; i++) {
		path = &info->dcam_path[i];
		if (path == NULL) {
			pr_err("init path %d fail\n", i);
			return -EINVAL;
		}
		memset((void *)path->frm_ptr,
		       0,
		       (unsigned int) (DCAM_FRM_CNT_MAX *
				       sizeof(struct camera_frame *)));
		path->frm_cnt_act = 0;
		sprd_img_buf_queue_init(&path->buf_queue);
		sprd_img_buf_queue_init(&path->tmp_buf_queue);
		path->status = PATH_IDLE;
	}
	atomic_set(&dev->stream_on, 0);

	pr_info("sprd_img: init handle end!\n");

	return 0;
}

static void sprd_camera_dev_deinit(struct camera_group *group, enum dcam_id idx)
{
	struct camera_dev *dev = NULL;

	dev = group->dev[idx];

	mutex_lock(&dev->dcam_mutex);
	atomic_set(&dev->stream_on, 0);

	sprd_isp_dev_deinit(dev->isp_dev_handle, idx);
	sprd_img_stop_zoom_thread(dev);
	dcam_stop_flash_thread(dev);
	sprd_stop_timer(&dev->dcam_timer);
	atomic_dec(&dev->users);
	mutex_unlock(&dev->dcam_mutex);

	vfree(dev);
	group->dev[idx] = NULL;

	if (group->dev[DCAM_ID_0] == NULL
		&& group->dev[DCAM_ID_1] == NULL)
		pfiommu_put_sg_table();

	pr_info("%s: dev[%d] deinit OK!\n", __func__, idx);
}

static int sprd_camera_dev_init(struct camera_group *group, enum dcam_id idx)
{
	int ret = 0;
	struct camera_dev *dev = NULL;

	if (!group) {
		pr_err("Input ptr is NULL!\n");
		return -EFAULT;
	}

	if (group->dev_inited & (1 << (int)idx)) {
		pr_info("sprd_img: dev%d already inited\n", idx);
		return 0;
	}

	pr_info("sprd_img: camera dev init start!\n");
	dev = vzalloc(sizeof(*dev));
	if (!dev) {
		pr_err("%s fail alloc dcam%d\n", __func__, idx);
		return -ENOMEM;
	}
	atomic_set(&dev->run_flag, DCAM_VALID_FRAME_COUNT);

	dev->idx = idx;
	mutex_init(&dev->dcam_mutex);
	init_completion(&dev->irq_com);
	spin_lock_init(&dev->queue.lock);
	if (unlikely(atomic_inc_return(&dev->users) > 1)) {
		vfree(dev);
		return -EBUSY;
	}

	ret = sprd_img_queue_init(&dev->queue);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to init queue\n", __func__);
		ret = -EINVAL;
		goto users_exit;
	}

	ret = sprd_init_handle(dev);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to init queue\n", __func__);
		ret = -EINVAL;
		goto users_exit;
	}

	sprd_init_timer(&dev->dcam_timer, (unsigned long)dev);

	ret = dcam_create_flash_thread(dev);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to create flash thread\n", __func__);
		ret = -EINVAL;
		goto flash_exit;
	}

	ret = sprd_img_create_zoom_thread(dev);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to create zoom thread\n", __func__);
		ret = -EINVAL;
		goto zoom_exit;
	}

	ret = sprd_isp_dev_init(&dev->isp_dev_handle, idx);
	if (unlikely(ret != 0)) {
		pr_err("sprd_img: ISP_ID_%d dev init fail\n", idx);
		ret = -EINVAL;
		goto isp_exit;
	}

	group->dev[idx] = dev;
	group->dev_inited |= 1 << idx;
	group->mode_inited = 0;

	pr_info("%s: dev[%d] init OK %p!\n", __func__, idx, dev);
	return ret;

isp_exit:
	sprd_img_stop_zoom_thread(dev);
zoom_exit:
	dcam_stop_flash_thread(dev);
flash_exit:
	sprd_stop_timer(&dev->dcam_timer);
users_exit:
	atomic_dec(&dev->users);
	vfree(dev);

	return ret;
}


static int sprd_img_get_res(struct camera_group *group,
			    struct sprd_img_res *res)
{
	int ret = 0;
	int dcam_id = 0;

	dcam_id = sprd_sensor_find_dcam_id(res->sensor_id);
	group->sensor_id = res->sensor_id;
	/*normal mode*/
	if (dcam_id == DCAM_ID_0) {
		if (0 == ((DCAM_RES_DCAM0_CAP |
			DCAM_RES_DCAM0_PATH)
			& group->dcam_res_used)) {
			pr_info("real sensor:dcam0\n");
			res->flag = DCAM_RES_DCAM0_CAP |
				DCAM_RES_DCAM0_PATH;
			group->dcam_res_used |=
				DCAM_RES_DCAM0_CAP |
				DCAM_RES_DCAM0_PATH;
		} else {
			pr_err("Err: no valid dcam for real sensor!\n");
			ret = -1;
			goto exit;
		}
	} else if (dcam_id == DCAM_ID_1) {
		if (0 == ((DCAM_RES_DCAM1_CAP |
			DCAM_RES_DCAM1_PATH) &
			group->dcam_res_used)) {
			pr_info("front sensor:dcam1\n");
			res->flag = DCAM_RES_DCAM1_CAP |
				DCAM_RES_DCAM1_PATH;
			group->dcam_res_used |=
				DCAM_RES_DCAM1_CAP |
				DCAM_RES_DCAM1_PATH;
		} else {
			pr_err("Err: no valid dcam for front sensor!\n");
			ret = -1;
			goto exit;
		}
	}
exit:
	if (ret)
		res->flag = 0;
	return ret;
}

static int sprd_img_put_res(struct camera_group *group,
			    struct sprd_img_res *res)
{
	int ret = 0;
	int dcam_id = 0;

	dcam_id = sprd_sensor_find_dcam_id(res->sensor_id);

	if (dcam_id == DCAM_ID_0) {
		if ((DCAM_RES_DCAM0_CAP |
			DCAM_RES_DCAM0_PATH) ==
			(res->flag &
			group->dcam_res_used)) {
			pr_info("put dcam0 for rear sensor\n");
			group->dcam_res_used &= ~(DCAM_RES_DCAM0_CAP |
				DCAM_RES_DCAM0_PATH);
		} else if (DCAM_RES_DCAM0_CAP ==
			(res->flag & group->dcam_res_used)) {
			pr_info("put dcam0 top for rear sensor\n");
			group->dcam_res_used &= ~DCAM_RES_DCAM0_CAP;
			goto exit;
		} else {
			pr_err("Err: can't put dcam0 for rear sensor!\n");
			ret = -1;
			goto exit;
		}
	} else if (dcam_id == DCAM_ID_1) {
		if ((DCAM_RES_DCAM1_CAP |
			DCAM_RES_DCAM1_PATH) ==
			(res->flag & group->dcam_res_used)) {
			pr_info("put dcam1 for front sensor\n");
			group->dcam_res_used &= ~(DCAM_RES_DCAM1_CAP |
				DCAM_RES_DCAM1_PATH);
		} else if (DCAM_RES_DCAM1_CAP ==
			(res->flag &
			group->dcam_res_used)) {
			pr_info("put dcam1 top for front sensor\n");
			group->dcam_res_used &= ~DCAM_RES_DCAM1_CAP;
			goto exit;
		} else {
				pr_err("Err: can't put dcam1 for front sensor!\n");
				ret = -1;
				goto exit;
		}
	}

exit:
	if (ret)
		res->flag = 0;
	return ret;
}

static int sprd_img_k_open(struct inode *node, struct file *file)
{
	int ret = 0;
	struct camera_file *camerafile = NULL;
	struct miscdevice *md = file->private_data;
	unsigned int count = 0;

	camerafile = vzalloc(sizeof(struct camera_file));
	if (camerafile == NULL)
		return -ENOMEM;

	camerafile->grp = md->this_device->platform_data;
	file->private_data = (void *)camerafile;
	count = camerafile->grp->dcam_count;

	if (atomic_inc_return(&camerafile->grp->camera_opened) > 1) {
		pr_info("sprd_img: the camera has been inited!\n");
		return 0;
	}

	if (count == 0 || count > DCAM_ID_MAX) {
		pr_err("dts tree config count error\n");
		return -ENODEV;
	}

	ret = sprd_camera_dev_init(camerafile->grp, DCAM_ID_0);
	if (unlikely(ret != 0)) {
		pr_err("camera 0 init fail\n");
		vfree(camerafile);
		return -EINVAL;
	}

	if (count == DCAM_ID_MAX)
		ret = sprd_camera_dev_init(camerafile->grp, DCAM_ID_1);

	if (unlikely(ret != 0)) {
		pr_err("camera 1 init fail\n");
		sprd_camera_dev_deinit(camerafile->grp, DCAM_ID_0);
		vfree(camerafile);
		return -EINVAL;
	}

	pr_info("sprd_img: open end!\n");

	return 0;
}

static int sprd_img_k_release(struct inode *node, struct file *file)
{
	struct camera_file *camerafile = NULL;
	struct camera_group *group = NULL;
	int idx;

	camerafile = file->private_data;
	if (!camerafile)
		goto exit;
	group = camerafile->grp;

	if (group->dev_inited & (1 << 0))
		complete(&((struct camera_dev *)group->dev[0])->irq_com);
	idx = camerafile->idx;
	pr_info("sprd_img: release start. idx %d\n", idx);

	if (!(group->dev_inited & (1 << idx))) {
		pr_info("sprd_img: dev%d already deinited\n",
				idx);
		return -1;
	}
	isp_statis_reset_all_queue(group->dev[idx]->isp_dev_handle);
	sprd_camera_stream_off(file);
	sprd_isp_module_dis(group->dev[idx]->isp_dev_handle, idx);
	sprd_dcam_module_dis(idx);
	sprd_camera_dev_deinit(group, idx);
	group->dev_inited &= ~(1<<idx);
	vfree(camerafile);
	file->private_data = NULL;
	if (atomic_dec_return(&group->camera_opened) == 0) {
		group->mode_inited = 0;
		group->dcam_res_used &= ~(DCAM_RES_DCAM0_CAP |
				DCAM_RES_DCAM0_PATH);
		group->dcam_res_used &= ~(DCAM_RES_DCAM1_CAP |
				DCAM_RES_DCAM1_PATH);
	}
	pr_info("sprd_img: release end!\n");
exit:
	return 0;
}

static const char * const ioctl_str[] = {
	"SPRD_IMG_IO_SET_MODE",
	"SPRD_IMG_IO_SET_CAP_SKIP_NUM",
	"SPRD_IMG_IO_SET_SENSOR_SIZE",
	"SPRD_IMG_IO_SET_SENSOR_TRIM",
	"SPRD_IMG_IO_SET_FRM_ID_BASE",
	"SPRD_IMG_IO_SET_CROP",
	"SPRD_IMG_IO_SET_FLASH",
	"SPRD_IMG_IO_SET_OUTPUT_SIZE",
	"SPRD_IMG_IO_SET_ZOOM_MODE",
	"SPRD_IMG_IO_SET_SENSOR_IF",
	"SPRD_IMG_IO_SET_FRAME_ADDR",
	"SPRD_IMG_IO_PATH_FRM_DECI",
	"SPRD_IMG_IO_PATH_PAUSE",
	"SPRD_IMG_IO_PATH_RESUME",
	"SPRD_IMG_IO_STREAM_ON",
	"SPRD_IMG_IO_STREAM_OFF",
	"SPRD_IMG_IO_GET_FMT",
	"SPRD_IMG_IO_GET_CH_ID",
	"SPRD_IMG_IO_GET_TIME",
	"SPRD_IMG_IO_CHECK_FMT",
	"SPRD_IMG_IO_SET_SHRINK",
	"SPRD_IMG_IO_SET_FREQ_FLAG",
	"SPRD_IMG_IO_CFG_FLASH",
	"SPRD_IMG_IO_PDAF_CONTROL",
	"SPRD_IMG_IO_DISABLE_MODE",
	"SPRD_IMG_IO_ENABLE_MODE",
	"SPRD_ISP_IO_CAPABILITY",
	"SPRD_ISP_IO_IRQ",
	"SPRD_ISP_IO_READ",
	"SPRD_ISP_IO_WRITE",
	"SPRD_ISP_IO_RST",
	"SPRD_ISP_IO_STOP",
	"SPRD_ISP_IO_INT",
	"SPRD_ISP_IO_CFG_PARAM",
	"SPRD_ISP_IO_SET_STSTIS_BUF",
	"SPRD_ISP_REG_READ",
	"SPRD_IMG_IO_GET_DCAM_RES",
	"SPRD_IMG_IO_PUT_DCAM_RES",
	"SPRD_ISP_IO_POST_3DNR",
	"SPRD_IMG_IO_GET_FLASH_INFO",
};

static int sprd_dcam_cfg_block(struct camera_info *info, enum dcam_id idx)
{
	int ret = DCAM_RTN_SUCCESS;
	unsigned int param = 0;

	if (info == NULL) {
		ret = -EINVAL;
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_INTERFACE, &info->if_mode);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_SENSOR_MODE, &info->sn_mode);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_SYNC_POL, &info->sync_pol);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	if ((info->dcam_path[CAMERA_PRE_PATH].is_work) ||
	    (info->dcam_path[CAMERA_VID_PATH].is_work) ||
	    (info->dcam_path[CAMERA_CAP_PATH].is_work))
		param = 1;
	else
		param = 0;

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_TO_ISP, &param);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_DATA_BITS, &info->data_bits);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	if (info->sn_mode == DCAM_CAP_MODE_RAWRGB &&
	    info->if_mode == DCAM_CAP_IF_CSI2) {
		ret = set_dcam_cap_cfg(idx, DCAM_CAP_DATA_PACKET,
					&info->is_loose);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_FRM_DECI, &info->frm_deci);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_INPUT_RECT, &info->cap_in_rect);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_FRM_COUNT_CLR, NULL);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_PRE_SKIP_CNT, &info->skip_number);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_SAMPLE_MODE, &info->capture_mode);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_IMAGE_XY_DECI, &info->img_deci);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

exit:
	return ret;
}


static int sprd_dcam_block_reg_isr(struct camera_dev *param)
{
	sprd_dcam_reg_isr(param->idx, DCAM_PATH0_AXIM_DONE,
		sprd_img_tx_done, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_FRM_ERR,
		sprd_img_tx_error, param);
	sprd_dcam_reg_isr(param->idx, DCAM_ISP_FIFO_OVF,
		sprd_img_tx_error, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_LINE_ERR,
		sprd_img_tx_error, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_MIPI_OVF,
		sprd_img_tx_error, param);
	sprd_dcam_reg_isr(param->idx, DCAM_PATH0_OVF,
		sprd_img_tx_error, param);
	sprd_dcam_reg_isr(param->idx, DCAM_PDAF_BUF_OVF,
		sprd_img_tx_error, param);
	sprd_dcam_reg_isr(param->idx, DCAM_SN_EOF,
		sprd_img_start_flash, param);
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	sprd_dcam_reg_isr(param->idx, DCAM_CPP_ERR,
		sprd_img_tx_error, param);
#endif
	return 0;
}

static int sprd_dcam_block_unreg_isr(struct camera_dev *param)
{
	sprd_dcam_reg_isr(param->idx, DCAM_PATH0_AXIM_DONE, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_FRM_ERR, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_ISP_FIFO_OVF, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_LINE_ERR, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_MIPI_OVF, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_PATH0_OVF, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_PDAF_BUF_OVF, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_SN_EOF, NULL, param);
	return 0;
}


static int sprd_dcam_cfg(struct camera_dev *dev)
{
	int ret = 0;
	struct camera_info *info = NULL;
	struct camera_path_spec *path0 = NULL;
	struct camera_path_spec *path_pdaf = NULL;
	enum dcam_id idx = 0;

	if (!dev) {
		ret = -EFAULT;
		pr_err("dev is NULL\n");
		goto exit;
	}

	mutex_lock(&dev->dcam_mutex);
	info = &dev->dcam_cxt;
	idx = dev->idx;
	path0 = &info->dcam_path[CAMERA_PATH0];
	path_pdaf = &info->dcam_path[CAMERA_PDAF_PATH];
	memset((void *)path0->frm_ptr, 0,
	       DCAM_FRM_CNT_MAX * sizeof(struct camera_frame *));
	memset((void *)path_pdaf->frm_ptr, 0,
	       DCAM_FRM_CNT_MAX * sizeof(struct camera_frame *));

	ret = sprd_dcam_module_init(idx);
	if (unlikely(ret)) {
		pr_err("failed to init dcam module\n");
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}

	ret = sprd_dcam_block_reg_isr(dev);
	if (unlikely(ret)) {
		pr_err("failed to register dcam isr\n");

		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}

	/* config cap sub-module */
	ret = sprd_dcam_cfg_block(info, idx);
	if (unlikely(ret)) {
		pr_err("failed to config cap");
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}

	/* config dcam_if path0 */
	if (path0->is_work) {
		ret = sprd_dcam_cfg_path0(path0, idx);
		if (unlikely(ret)) {
			pr_err("failed to config path0 cap");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		path0->status = PATH_RUN;
	}

	/* config dcam_if path pdaf*/
	if (path_pdaf->is_work) {
		ret = sprd_dcam_cfg_path_pdaf(path_pdaf, idx);
		if (unlikely(ret)) {
			pr_err("failed to config path pdaf cap");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
	}

	mutex_unlock(&dev->dcam_mutex);
	pr_debug("end dcam cfg.\n");
exit:
	return ret;
}

static int sprd_isp_path_mode_cfg(struct camera_info *info)
{
	int ret = 0;
	unsigned int tempw;
	struct camera_path_spec *path_pre = NULL;
	struct camera_path_spec *path_vid = NULL;
	struct camera_path_spec *path_cap = NULL;

	if (!info) {
		ret = -EFAULT;
		pr_err("info is NULL\n");
		goto exit;
	}

	path_pre = &info->dcam_path[CAMERA_PRE_PATH];
	path_vid = &info->dcam_path[CAMERA_VID_PATH];
	path_cap = &info->dcam_path[CAMERA_CAP_PATH];

	if (info->is_slow_motion) {
		path_pre->path_mode = ISP_PRE_ONLINE;
		path_vid->path_mode = ISP_VID_ONLINE;
		path_cap->path_mode = ISP_CAP_OFFLINE;
	} else {
		tempw = path_pre->out_size.w * ISP_ONLINE_ZFACTOR_MAX;
		if (path_pre->in_size.w >= tempw) {
			path_pre->path_mode = ISP_PRE_ONLINE;
			path_vid->path_mode = ISP_VID_ONLINE;
			tempw = path_vid->out_size.w * ISP_ONLINE_ZFACTOR_MAX;
			if (path_pre->in_size.w < tempw)
				path_vid->path_mode = ISP_VID_ONLINE_CPP;
		} else {
			path_pre->path_mode = ISP_PRE_OFFLINE;
			path_vid->path_mode = ISP_VID_OFFLINE;
		}
		path_pre->path_mode = ISP_PRE_ONLINE;
		path_vid->path_mode = ISP_VID_ONLINE;
		path_cap->path_mode = ISP_CAP_OFFLINE;
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (path_pre->is_work && !path_cap->is_work)
			path_pre->path_mode = ISP_PRE_ONLINE_CPP;

		if (path_pre->is_work && path_vid->is_work &&
				!path_cap->is_work) {
			path_pre->path_mode = ISP_PRE_VID_ONLINE_CPP;
			path_vid->path_mode = ISP_PRE_VID_ONLINE_CPP;
		}
#endif
	}

	pr_err("prv path_mode: %d, cap path_mode %d, vid path mode %d\n",
	       path_pre->path_mode, path_cap->path_mode, path_vid->path_mode);
exit:
	return ret;
}

static int sprd_img_isp_reg_isr(struct camera_dev *dev)
{
	enum isp_id idx = ISP_ID_0;

	if (!dev) {
		pr_err("Input dev ptr is NULL\n");
		return -EFAULT;
	}

	if (dev->idx == DCAM_ID_0)
		idx = ISP_ID_0;
	else if (dev->idx == DCAM_ID_1)
		idx = ISP_ID_1;
	else {
		pr_err("dev idx %d not support", dev->idx);
		return -EFAULT;
	}
	sprd_isp_reg_isr(idx, ISP_PATH_PRE_DONE,
		sprd_img_tx_done, dev);
	sprd_isp_reg_isr(idx, ISP_PATH_VID_DONE,
		sprd_img_tx_done, dev);
	sprd_isp_reg_isr(idx, ISP_PATH_CAP_DONE,
		sprd_img_tx_done, dev);
	sprd_isp_reg_isr(idx, ISP_AEM_DONE,
		sprd_img_tx_done, dev);
	sprd_isp_reg_isr(idx, ISP_AFL_DONE,
		sprd_img_tx_done, dev);
	sprd_isp_reg_isr(idx, ISP_AFM_DONE,
		sprd_img_tx_done, dev);
	sprd_isp_reg_isr(idx, ISP_BINNING_DONE,
		sprd_img_tx_done, dev);
	sprd_isp_reg_isr(idx, ISP_PDAF_LEFT_DONE,
		sprd_img_tx_done, dev);
	sprd_isp_reg_isr(idx, ISP_PDAF_RIGHT_DONE,
		sprd_img_tx_done, dev);
	sprd_isp_reg_isr(idx, ISP_SHADOW_DONE,
		sprd_img_tx_done, dev);
	sprd_isp_reg_isr(idx, ISP_DCAM_SOF,
		sprd_img_tx_done, dev);
	sprd_isp_reg_isr(idx, ISP_3DNR_CAP_DONE,
		sprd_img_tx_done, dev);

	return 0;
}

static int sprd_img_isp_unreg_isr(struct camera_dev *dev)
{
	enum isp_id idx = ISP_ID_0;

	if (!dev) {
		pr_err("Input dev ptr is NULL\n");
		return -EFAULT;
	}

	if (dev->idx == DCAM_ID_0)
		idx = ISP_ID_0;
	else if (dev->idx == DCAM_ID_1)
		idx = ISP_ID_1;
	else {
		pr_err("dev idx %d not support", dev->idx);
		return -EFAULT;
	}
	sprd_isp_reg_isr(idx, ISP_PATH_PRE_DONE,
			NULL, dev);
	sprd_isp_reg_isr(idx, ISP_PATH_VID_DONE,
			NULL, dev);
	sprd_isp_reg_isr(idx, ISP_PATH_CAP_DONE,
			 NULL, dev);
	sprd_isp_reg_isr(idx, ISP_AEM_DONE,
			 NULL, dev);
	sprd_isp_reg_isr(idx, ISP_AFL_DONE,
			 NULL, dev);
	sprd_isp_reg_isr(idx, ISP_AFM_DONE,
			 NULL, dev);
	sprd_isp_reg_isr(idx, ISP_BINNING_DONE,
			 NULL, dev);
	sprd_isp_reg_isr(idx, ISP_PDAF_LEFT_DONE,
			 NULL, dev);
	sprd_isp_reg_isr(idx, ISP_PDAF_RIGHT_DONE,
			 NULL, dev);
	sprd_isp_reg_isr(idx, ISP_SHADOW_DONE,
			 NULL, dev);
	sprd_isp_reg_isr(idx, ISP_DCAM_SOF,
			 NULL, dev);
	sprd_isp_reg_isr(idx, ISP_3DNR_CAP_DONE,
			 NULL, dev);

	return 0;
}

static int sprd_isp_path_cfg(struct camera_dev *dev)
{
	int ret = 0;
	int path_not_work = 0;
	struct camera_info *info = NULL;
	struct isp_path_info path_info;
	struct camera_path_spec *path_pre = NULL;
	struct camera_path_spec *path_vid = NULL;
	struct camera_path_spec *path_cap = NULL;
	enum dcam_id idx = 0;

	if (!dev) {
		ret = -EFAULT;
		pr_err("dev is NULL\n");
		goto exit;
	}

	mutex_lock(&dev->dcam_mutex);
	info = &dev->dcam_cxt;
	idx = dev->idx;
	path_pre = &info->dcam_path[CAMERA_PRE_PATH];
	path_vid = &info->dcam_path[CAMERA_VID_PATH];
	path_cap = &info->dcam_path[CAMERA_CAP_PATH];

	path_info.is_slow_motion = info->is_slow_motion;
	memset((void *)path_pre->frm_ptr, 0,
	       DCAM_FRM_CNT_MAX * sizeof(struct camera_frame *));
	memset((void *)path_vid->frm_ptr, 0,
	       DCAM_FRM_CNT_MAX * sizeof(struct camera_frame *));
	memset((void *)path_cap->frm_ptr, 0,
	       DCAM_FRM_CNT_MAX * sizeof(struct camera_frame *));

	/* need config pre&vid onlie/offline/use cpp or not here*/
	ret = sprd_isp_path_mode_cfg(info);
	if (unlikely(ret)) {
		pr_err("failed to config isp path mode\n");
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}

	ret = sprd_img_isp_reg_isr(dev);
	if (unlikely(ret)) {
		pr_err("failed to register isp isr\n");
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}

	ret = set_isp_path_cfg(dev->isp_dev_handle, ISP_PATH_IDX_PRE,
		ISP_PATH_ZOOM_MODE, &info->is_smooth_zoom);
	ret = set_isp_path_cfg(dev->isp_dev_handle, ISP_PATH_IDX_PRE,
		ISP_PATH_SN_MAX_SIZE, &info->sn_max_size);

	do {
		/* config isp pre path */
		if (dev->use_path && path_pre->is_work) {
			ret = sprd_isp_path_cfg_block(path_pre,
				dev->isp_dev_handle, ISP_PATH_IDX_PRE);
			if (unlikely(ret)) {
				pr_err("failed to config path_pre");
				break;
			}
			path_pre->status = PATH_RUN;
		} else {
			ret = set_isp_path_cfg(dev->isp_dev_handle,
				ISP_PATH_IDX_PRE, ISP_PATH_ENABLE,
		&path_not_work);
			if (unlikely(ret)) {
				pr_err("failed to config isp path pre\n");
				break;
			}
		}

		/* config isp vid path*/
		if (dev->use_path && path_vid->is_work) {
			ret = sprd_isp_path_cfg_block(path_vid,
				dev->isp_dev_handle, ISP_PATH_IDX_VID);
			if (unlikely(ret)) {
				pr_err("failed to config path_vid");
				break;
			}
			ret = isp_slw_flags_init(dev->isp_dev_handle,
						&path_info);
			if (unlikely(ret)) {
				pr_err("failed to config slow motion");
				break;
			}
			path_vid->status = PATH_RUN;
		} else {
			ret = set_isp_path_cfg(dev->isp_dev_handle,
				ISP_PATH_IDX_VID, ISP_PATH_ENABLE,
		&path_not_work);
			if (unlikely(ret)) {
				pr_err("failed to config isp path pre\n");
				break;
			}
		}

		/* config isp cap path*/
		if (dev->use_path && path_cap->is_work) {
			ret = sprd_isp_path_cfg_block(path_cap,
				dev->isp_dev_handle, ISP_PATH_IDX_CAP);
			if (unlikely(ret)) {
				pr_err("failed to config path_cap");
				break;
			}
			path_cap->status = PATH_RUN;
		} else {
			ret = set_isp_path_cfg(dev->isp_dev_handle,
				ISP_PATH_IDX_CAP, ISP_PATH_ENABLE,
		&path_not_work);
			if (unlikely(ret)) {
				pr_err("failed to config isp path pre\n");
				break;
			}
		}

	} while (0);

	mutex_unlock(&dev->dcam_mutex);
	pr_debug("end isp path cfg\n");
exit:
	return ret;
}

static int sprd_camera_update_clk(struct camera_dev *dev,
	struct device_node *dn, int device_num)
{
	int ret = 0;
	unsigned int isp_clk_index = ISP_CLK_307M2_INDEX;
	unsigned int dcam_clk_index = DCAM_CLK_307M2_INDEX;
	unsigned int width = 0;
	unsigned int height = 0;
	unsigned int width_max = 0;
	unsigned int height_max = 0;
	struct camera_info *info = NULL;
	struct camera_path_spec *path_cap = NULL;

	if (!dev || !dn) {
		ret = -EFAULT;
		pr_err("input ptr is NULL\n");
		goto exit;
	}

	info = &dev->dcam_cxt;
	if (!info) {
		ret = -EFAULT;
		pr_err("dcam_cxt is NULL\n");
		goto exit;
	}
	path_cap = &info->dcam_path[CAMERA_CAP_PATH];
	width_max = info->sn_max_size.w;
	height_max = info->sn_max_size.h;
	width = info->cap_in_size.w;
	height = info->cap_in_size.h;

	if (info->bps_per_lane == 0xffff) {
		pr_info("dcam and isp not support power optimization\n");
		goto exit;
	}

	if (width <= CAP_IN_SIZE_2M_WIDTH)
		dcam_clk_index = DCAM_CLK_256M_INDEX;

	if ((width_max > CAP_IN_SIZE_5M_WIDTH &&
		path_cap->is_work) ||
		info->raw_cap_flag ||
		info->slow_motion_flag ||
		device_num >= 2)
		dcam_clk_index = DCAM_CLK_307M2_INDEX;

	pr_info("dcamclk index :%d", dcam_clk_index);
	ret = sprd_dcam_update_clk(dcam_clk_index, dn);

	if (unlikely(ret != 0)) {
		pr_err("failed to update dcam_if clk\n");
		goto exit;
	}

	if (width > CAP_IN_SIZE_5M_WIDTH)
		isp_clk_index = ISP_CLK_576M_INDEX;
	else if (width <= CAP_IN_SIZE_2M_WIDTH)
		isp_clk_index = ISP_CLK_256M_INDEX;

	if ((width_max > CAP_IN_SIZE_5M_WIDTH &&
		path_cap->is_work) ||
		info->raw_cap_flag ||
		info->slow_motion_flag ||
		device_num >= 2)
		isp_clk_index = ISP_CLK_576M_INDEX;

	pr_info("ispclk index :%d", isp_clk_index);
	ret = sprd_isp_update_clk(isp_clk_index, dn);
	if (unlikely(ret != 0)) {
		pr_err("failed to update isp clk\n");
		goto exit;
	}

exit:
	return ret;
}

static int sprd_camera_stream_on(struct file *file)
{
	int ret = 0;
	struct camera_dev *dev = NULL;
	struct camera_file *camerafile = NULL;
	struct camera_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	int device_num;

	pr_info("stream on in\n");
	camerafile = (struct camera_file *)file->private_data;
	if (!camerafile) {
		ret = -EFAULT;
		pr_err("camerafile is NULL\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	device_num = atomic_read(&group->camera_opened);
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("dev[%d] is NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("dev[%d] hasn't been inited!\n", idx);
		goto exit;
	}

	mutex_lock(&dev->dcam_mutex);
	ret = sprd_camera_update_clk(dev,
		group->pdev->dev.of_node, device_num);
	if (unlikely(ret != 0)) {
		mutex_unlock(&dev->dcam_mutex);
		pr_err("failed to update clk\n");
		goto exit;
	}
	mutex_unlock(&dev->dcam_mutex);

	ret = sprd_img_queue_init(&dev->queue);
	if (unlikely(ret != 0)) {
		pr_err("failed to init queue\n");
		goto exit;
	}

	ret = sprd_dcam_cfg(dev);
	if (unlikely(ret)) {
		pr_err("failed to config dcam param\n");
		goto exit;
	}

	ret = sprd_isp_path_cfg(dev);
	if (unlikely(ret)) {
		pr_err("failed to config isp path\n");
		goto exit;
	}

	dev->frame_skipped = 0;

	if ((dev->dcam_cxt.set_flash.led0_ctrl &&
		dev->dcam_cxt.set_flash.led0_status == FLASH_HIGH_LIGHT) ||
		(dev->dcam_cxt.set_flash.led1_ctrl &&
		dev->dcam_cxt.set_flash.led1_status == FLASH_HIGH_LIGHT)) {
		if (dev->dcam_cxt.skip_number == 0)
			sprd_img_start_flash(NULL, dev);
	}

	ret = sprd_isp_start(dev->isp_dev_handle);
	if (unlikely(ret)) {
		pr_err("failed to start isp path\n");
		goto exit;
	}

	mutex_lock(&dev->dcam_mutex);
	ret = sprd_dcam_start(idx);
	if (unlikely(ret)) {
		pr_err("failed to start dcam\n");
		ret = sprd_img_isp_unreg_isr(dev);
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	} else {
		atomic_set(&dev->run_flag, 0);
		sprd_start_timer(&dev->dcam_timer, DCAM_TIMEOUT);
	}

	atomic_set(&dev->stream_on, 1);
	mutex_unlock(&dev->dcam_mutex);

	pr_info("DCAM%d, stream on end.\n", idx);
exit:
	return ret;
}

static int sprd_camera_stream_off(struct file *file)
{
	int ret = 0;
	struct camera_dev *dev = NULL;
	struct camera_file *camerafile = NULL;
	struct camera_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_path_spec *path_0 = NULL;

	camerafile = (struct camera_file *)file->private_data;
	if (!camerafile) {
		ret = -EFAULT;
		pr_err("camerafile is NULL\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("dev[%d] is NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("dev[%d] hasn't been inited!\n", idx);
		goto exit;
	}

	mutex_lock(&dev->dcam_mutex);

	path_0 = &dev->dcam_cxt.dcam_path[CAMERA_PATH0];

	if (unlikely(atomic_read(&dev->stream_on) == 0)) {
		pr_info("stream is off, idx: %d\n", idx);
		ret = sprd_img_local_deinit(dev);
		if (unlikely(ret))
			pr_err("failed to local deinit\n");

		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}
	do {
		ret = sprd_stop_timer(&dev->dcam_timer);
		if (unlikely(ret)) {
			pr_err("failed to stop timer\n");
			break;
		}
		ret = sprd_dcam_stop(idx, 0);
		if (unlikely(ret)) {
			pr_err("failed to stop dcam\n");
			break;
		}
		ret = sprd_dcam_block_unreg_isr(dev);
		if (unlikely(ret)) {
			pr_err("failed to unregister isr\n");
			break;
		}

		/* check if stop dcam first, then isp, or not? */
		ret = sprd_isp_stop(dev->isp_dev_handle, 0);
		if (unlikely(ret)) {
			pr_err("failed to stop isp\n");
			break;
		}

		ret = sprd_img_isp_unreg_isr(dev);
		if (unlikely(ret)) {
			pr_err("failed to unregister isp isr\n");
			break;
		}

		if (path_0->is_work) {
			path_0->status = PATH_IDLE;
			path_0->is_work = 0;
		}

		atomic_set(&dev->stream_on, 0);

		ret = sprd_dcam_module_deinit(idx);
		if (unlikely(ret)) {
			pr_err("failed to deinit dcam module\n");
			break;
		}

		ret = sprd_img_local_deinit(dev);
		if (unlikely(ret)) {
			pr_err("failed to local deinit\n");
			break;
		}
	} while (0);

	mutex_unlock(&dev->dcam_mutex);

	pr_info("camera idx=%d stream off end.\n", idx);
exit:
	return ret;
}

static int sprd_img_update_video(struct camera_dev *dev,
		unsigned int channel_id)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_path_spec *path = NULL;
	enum isp_path_index path_index;

	DCAM_TRACE("update video, channel %d\n", channel_id);

	mutex_lock(&dev->dcam_mutex);

	path = &dev->dcam_cxt.dcam_path[channel_id];
	path_index = sprd_img_get_path_index(channel_id);

	if (dev->dcam_cxt.is_smooth_zoom && CAMERA_PATH0 != channel_id) {
		dev->zoom_level = 1;
		dev->channel_id = channel_id;
		if (path->in_rect_backup.w == 0 ||
		    path->in_rect_backup.h == 0) {
			path->in_rect_backup.x = 0;
			path->in_rect_backup.y = 0;
			path->in_rect_backup.w = path->in_size.w;
			path->in_rect_backup.h = path->in_size.h;
			memcpy((void *)&path->in_rect_current,
			       (void *)&path->in_rect_backup,
			       sizeof(struct camera_rect));
		} else {
			memcpy((void *)&path->in_rect_backup,
			       (void *)&path->in_rect_current,
			       sizeof(struct camera_rect));
		}

		DCAM_TRACE("in_size{%d %d}, in_rect{%d %d %d %d}\n",
			   path->in_size.w, path->in_size.h, path->in_rect.x,
			   path->in_rect.y, path->in_rect.w, path->in_rect.h);
		DCAM_TRACE("in_rect_backup{%d %d %d %d}, out_size{%d %d}\n",
			   path->in_rect_backup.x, path->in_rect_backup.y,
			   path->in_rect_backup.w, path->in_rect_backup.h,
			   path->out_size.w, path->out_size.h);
	} else {
		DCAM_TRACE("in_size{%d %d}, in_rect{%d %d %d %d}\n",
			   path->in_size.w, path->in_size.h, path->in_rect.x,
			   path->in_rect.y, path->in_rect.w, path->in_rect.h);
		DCAM_TRACE("out_size{%d %d}\n",
			   path->out_size.w, path->out_size.h);
		ret = sprd_isp_update_path(dev->isp_dev_handle,
					   path_index,
					   &path->in_size,
					   &path->in_rect,
					   &path->out_size);
	}

	mutex_unlock(&dev->dcam_mutex);
	DCAM_TRACE("update video 0x%x\n", ret);

	if (ret)
		pr_err("failed to update video 0x%x\n", ret);

	return ret;
}

static int sprd_img_check_fmt(struct file *file,
			      struct sprd_img_format *img_format)
{
	int ret = 0;
	unsigned int channel_id;
	struct camera_dev *dev = NULL;
	struct camera_file *camerafile = NULL;
	struct camera_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_format *fmt;

	camerafile = (struct camera_file *)file->private_data;
	if (!camerafile) {
		ret = -EFAULT;
		pr_err("camerafile is NULL\n");
		goto exit;
	}

	if (!img_format) {
		ret = -EFAULT;
		pr_err("img format parm is NULL\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("dev[%d] is NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("dev[%d] hasn't been inited!\n", idx);
		goto exit;
	}
	dev->use_path = img_format->buffer_cfg_isp;

	fmt = sprd_img_get_format(img_format->fourcc);
	if (unlikely(!fmt)) {
		pr_err("fourcc format (0x%08x) invalid.\n",
		       img_format->fourcc);
		return -EINVAL;
	}

	if (img_format->channel_id == CAMERA_PATH0) {
		mutex_lock(&dev->dcam_mutex);
		ret = sprd_img_check_path0_cap(fmt->fourcc,
					       img_format,
					       &dev->dcam_cxt);
		mutex_unlock(&dev->dcam_mutex);
		channel_id = CAMERA_PATH0;
	} else if (img_format->channel_id == CAMERA_PDAF_PATH) {
		mutex_lock(&dev->dcam_mutex);
		ret = sprd_img_check_path_pdaf_cap(fmt->fourcc,
					       img_format,
					       &dev->dcam_cxt);
		mutex_unlock(&dev->dcam_mutex);
		channel_id = CAMERA_PDAF_PATH;
	} else {
		mutex_lock(&dev->dcam_mutex);
		ret = sprd_img_check_path_cap(idx, fmt->fourcc, img_format,
			&dev->dcam_cxt, img_format->channel_id);
		mutex_unlock(&dev->dcam_mutex);
		channel_id = img_format->channel_id;
	}

	if (channel_id < CAMERA_PDAF_PATH) {
		img_format->endian.y_endian =
			dev->dcam_cxt.dcam_path[channel_id].end_sel.y_endian;
		img_format->endian.uv_endian =
			dev->dcam_cxt.dcam_path[channel_id].end_sel.uv_endian;
	}

	if ((ret == 0) && (atomic_read(&dev->stream_on) != 0)) {
		if (channel_id == CAMERA_PRE_PATH
			|| channel_id == CAMERA_VID_PATH
			|| channel_id == CAMERA_CAP_PATH) {
			ret = sprd_img_update_video(dev, channel_id);
		}
	}

exit:
	return ret;
}


static int sprd_img_set_crop(struct file *file, struct sprd_img_parm *p)
{
	int ret = 0;
	struct camera_dev *dev = NULL;
	struct camera_file *camerafile = NULL;
	struct camera_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_rect *input_rect;
	struct camera_size *input_size;

	camerafile = (struct camera_file *)file->private_data;
	if (!camerafile) {
		ret = -EFAULT;
		pr_err("camerafile is NULL\n");
		goto exit;
	}

	if (!p) {
		ret = -EFAULT;
		pr_err("img parm is NULL\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("dev[%d] is NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("dev[%d] hasn't been inited!\n", idx);
		goto exit;
	}

	if (p->crop_rect.x + p->crop_rect.w > dev->dcam_cxt.cap_in_size.w ||
	    p->crop_rect.y + p->crop_rect.h > dev->dcam_cxt.cap_in_size.h) {
		ret = -EINVAL;
		goto exit;
	}

	pr_debug("dcam%d: sprd_img_set_crop, window %d %d %d %d\n", idx,
		   p->crop_rect.x, p->crop_rect.y,
		   p->crop_rect.w, p->crop_rect.h);

	switch (p->channel_id) {
	case CAMERA_PATH0:
	case CAMERA_PRE_PATH:
	case CAMERA_VID_PATH:
	case CAMERA_CAP_PATH:
		input_size = &dev->dcam_cxt.dcam_path[p->channel_id].in_size;
		input_rect = &dev->dcam_cxt.dcam_path[p->channel_id].in_rect;
		break;
	default:
		pr_info("dcam%d: wrong channel id %d\n", idx, p->channel_id);
		ret = -EINVAL;
		goto exit;
	}

	input_size->w = dev->dcam_cxt.cap_out_size.w;
	input_size->h = dev->dcam_cxt.cap_out_size.h;
	input_rect->x = p->crop_rect.x;
	input_rect->y = p->crop_rect.y;
	input_rect->w = p->crop_rect.w;
	input_rect->h = p->crop_rect.h;

	DCAM_TRACE("path %d, rect %d %d %d %d, size %d %d\n",
		   p->channel_id, input_rect->x, input_rect->y,
		   input_rect->w, input_rect->h, input_size->w,
		   input_size->h);

exit:
	return ret;
}

static int sprd_img_set_sensor_if(struct file *file,
				  struct sprd_img_sensor_if *sensor_if)
{
	int ret = 0;
	struct camera_dev *dev = NULL;
	struct camera_file *camerafile = NULL;
	struct camera_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_info *info = NULL;

	camerafile = (struct camera_file *)file->private_data;
	if (!camerafile) {
		ret = -EFAULT;
		pr_err("camerafile is NULL\n");
		goto exit;
	}

	if (!sensor_if) {
		ret = -EFAULT;
		pr_err("sensor_if parm is NULL\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("dev[%d] is NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("dev[%d] hasn't been inited!\n", idx);
		goto exit;
	}

	if (sensor_if->res[0] != IF_OPEN)
		goto exit;

	info = &dev->dcam_cxt;
	info->if_mode = sensor_if->if_type;
	info->sn_mode = sensor_if->img_fmt;
	info->yuv_ptn = sensor_if->img_ptn;
	info->frm_deci = sensor_if->frm_deci;
	info->sensor_id = group->sensor_id;
	pr_debug("dcam%d: if %d mode %d deci %d\n",
		   idx, info->if_mode, info->sn_mode,
		   info->frm_deci);

	if (info->if_mode == DCAM_CAP_IF_CCIR) {
		/* CCIR interface */
		info->sync_pol.vsync_pol = sensor_if->if_spec.ccir.v_sync_pol;
		info->sync_pol.hsync_pol = sensor_if->if_spec.ccir.h_sync_pol;
		info->sync_pol.pclk_pol = sensor_if->if_spec.ccir.pclk_pol;
		info->data_bits = 8;
	} else {
		info->sync_pol.need_href = sensor_if->if_spec.mipi.use_href;
		info->is_loose = sensor_if->if_spec.mipi.is_loose;
		info->data_bits = sensor_if->if_spec.mipi.bits_per_pxl;
		info->lane_num = sensor_if->if_spec.mipi.lane_num;
		info->bps_per_lane = sensor_if->if_spec.mipi.pclk;
	}

exit:
	return ret;
}

static int sprd_img_set_frame_addr(struct file *file,
				   struct sprd_img_parm *p)
{
	int ret = 0;
	unsigned int i = 0;
	struct camera_dev *dev = NULL;
	struct camera_file *camerafile = NULL;
	struct camera_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_path_spec *path = NULL;
	enum isp_path_index path_index;

	camerafile = (struct camera_file *)file->private_data;
	if (!camerafile) {
		ret = -EFAULT;
		pr_err("camerafile is NULL\n");
		goto exit;
	}

	if (!p) {
		ret = -EFAULT;
		pr_err("img parm is NULL\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("dev[%d] is NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("dev[%d] hasn't been inited!\n", idx);
		goto exit;
	}

	switch (p->channel_id) {
	case CAMERA_PATH0:
		path = &dev->dcam_cxt.dcam_path[CAMERA_PATH0];
		break;
	case CAMERA_PRE_PATH:
		path = &dev->dcam_cxt.dcam_path[CAMERA_PRE_PATH];
		break;
	case CAMERA_VID_PATH:
		path = &dev->dcam_cxt.dcam_path[CAMERA_VID_PATH];
		break;
	case CAMERA_CAP_PATH:
		path = &dev->dcam_cxt.dcam_path[CAMERA_CAP_PATH];
		break;
	case CAMERA_PDAF_PATH:
		path = &dev->dcam_cxt.dcam_path[CAMERA_PDAF_PATH];
		break;
	default:
		pr_info("invalid channel %d\n", p->channel_id);
		return -EINVAL;
	}

	DCAM_TRACE("set path%d frame addr,status %d cnt %d reserved_buf %d\n",
		   p->channel_id, path->status, path->frm_cnt_act,
		   p->is_reserved_buf);

	if (unlikely(p->frame_addr_vir_array[0].y == 0)) {
		pr_info("no yaddr\n");
		ret = -EINVAL;
		goto exit;
	}

	if (p->is_reserved_buf == 1) {
		path->frm_reserved_addr.yaddr = p->frame_addr_array[0].y;
		path->frm_reserved_addr.uaddr = p->frame_addr_array[0].u;
		path->frm_reserved_addr.vaddr = p->frame_addr_array[0].v;
		path->frm_reserved_addr_vir.yaddr =
						p->frame_addr_vir_array[0].y;
		path->frm_reserved_addr_vir.uaddr =
						p->frame_addr_vir_array[0].u;
		path->frm_reserved_addr_vir.vaddr =
						p->frame_addr_vir_array[0].v;
		path->frm_reserved_addr.mfd_y = p->fd_array[0];
		path->frm_reserved_addr.mfd_u = p->fd_array[0];
		path->frm_reserved_addr.mfd_v = p->fd_array[0];
	} else {
		struct camera_addr frame_addr;
		struct camera_img_buf_addr buf_addr;

		if (atomic_read(&dev->stream_on) == 1 &&
			path->status == PATH_RUN) {
			if (p->channel_id == CAMERA_PATH0) {
				ret = set_dcam_path0_cfg(idx,
					DCAM_PATH0_OUTPUT_ADDR, &frame_addr);
				if (unlikely(ret)) {
					pr_err("%s err, code %d",
						__func__, ret);
					goto exit;
				}
			} else if (p->channel_id == CAMERA_PDAF_PATH) {
				ret = set_dcam_path_pdaf_cfg(idx,
					DCAM_PATH_PDAF_OUTPUT_ADDR,
					&frame_addr);
				if (unlikely(ret)) {
					pr_err("%s err, code %d",
						__func__, ret);
					goto exit;
				}
			} else {
				path_index =
					sprd_img_get_path_index(p->channel_id);
				for (i = 0; i < p->buffer_count; i++) {
					if (p->frame_addr_vir_array[i].y == 0) {
						pr_info("no yaddr\n");
						ret = -EINVAL;
						goto exit;
					}
					frame_addr.yaddr =
						p->frame_addr_array[i].y;
					frame_addr.uaddr =
						p->frame_addr_array[i].u;
					frame_addr.vaddr =
						p->frame_addr_array[i].v;
					frame_addr.yaddr_vir =
						p->frame_addr_vir_array[i].y;
					frame_addr.uaddr_vir =
						p->frame_addr_vir_array[i].u;
					frame_addr.vaddr_vir =
						p->frame_addr_vir_array[i].v;
					frame_addr.mfd_y = p->fd_array[i];
					frame_addr.mfd_u = p->fd_array[i];
					frame_addr.mfd_v = p->fd_array[i];
					ret = set_isp_path_cfg(
						dev->isp_dev_handle,
						path_index,
						ISP_PATH_OUTPUT_ADDR,
						&frame_addr);
					if (unlikely(ret)) {
						pr_err("%s err, code %d",
							__func__, ret);
						goto exit;
					}
				}
			}
		} else {

			for (i = 0; i < p->buffer_count; i++) {
				if (unlikely(p->frame_addr_vir_array[i].y
						== 0)) {
					pr_info("no yaddr\n");
					ret = -EINVAL;
					goto exit;
				}
				buf_addr.frm_addr.yaddr =
						p->frame_addr_array[i].y;
				buf_addr.frm_addr.uaddr =
						p->frame_addr_array[i].u;
				buf_addr.frm_addr.vaddr =
						p->frame_addr_array[i].v;
				buf_addr.frm_addr_vir.yaddr =
						p->frame_addr_vir_array[i].y;
				buf_addr.frm_addr_vir.uaddr =
						p->frame_addr_vir_array[i].u;
				buf_addr.frm_addr_vir.vaddr =
						p->frame_addr_vir_array[i].v;
				buf_addr.frm_addr.mfd_y = p->fd_array[i];
				buf_addr.frm_addr.mfd_u = p->fd_array[i];
				buf_addr.frm_addr.mfd_v = p->fd_array[i];
				ret = sprd_img_buf_queue_write(
					&path->buf_queue,
					&buf_addr);
			}
		}
	}

	DCAM_TRACE("dcam idx=%d: success to set frame addr\n", idx);

exit:
	return ret;
}

static int sprd_img_get_free_channel(struct file *file,
	unsigned int *channel_id, unsigned int scene_mode)
{
	int ret = 0;
	struct camera_dev *dev = NULL;
	struct camera_file *camerafile = NULL;
	struct camera_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_path_spec *path_0 = NULL;
	struct camera_path_spec *path_1 = NULL;
	struct camera_path_spec *path_2 = NULL;
	struct camera_path_spec *path_3 = NULL;
	struct camera_get_path_id path_id;

	camerafile = (struct camera_file *)file->private_data;
	if (!camerafile) {
		ret = -EFAULT;
		pr_err("camerafile is NULL\n");
		goto exit;
	}

	if (!channel_id) {
		ret = -EFAULT;
		pr_err("parm is NULL\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("dev[%d] is NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("dev[%d] hasn't been inited!\n", idx);
		goto exit;
	}

	path_0 = &dev->dcam_cxt.dcam_path[CAMERA_PATH0];
	path_1 = &dev->dcam_cxt.dcam_path[CAMERA_PRE_PATH];
	path_2 = &dev->dcam_cxt.dcam_path[CAMERA_VID_PATH];
	path_3 = &dev->dcam_cxt.dcam_path[CAMERA_CAP_PATH];

	memset((void *)&path_id, 0,
	       sizeof(struct camera_get_path_id));
	path_id.input_size.w = dev->dcam_cxt.cap_in_rect.w;
	path_id.input_size.h = dev->dcam_cxt.cap_in_rect.h;
	path_id.output_size.w = dev->dcam_cxt.dst_size.w;
	path_id.output_size.h = dev->dcam_cxt.dst_size.h;
	path_id.fourcc = dev->dcam_cxt.pxl_fmt;
	path_id.need_isp_tool = dev->dcam_cxt.need_isp_tool;
	path_id.need_isp = dev->dcam_cxt.need_isp;
	path_id.rt_refocus = dev->dcam_cxt.rt_refocus;
	path_id.input_trim.x = dev->dcam_cxt.path_input_rect.x;
	path_id.input_trim.y = dev->dcam_cxt.path_input_rect.y;
	path_id.input_trim.w = dev->dcam_cxt.path_input_rect.w;
	path_id.input_trim.h = dev->dcam_cxt.path_input_rect.h;
	DCAM_TRACE("get parm, path work %d %d %d %d\n",
		   path_0->is_work, path_1->is_work,
		   path_2->is_work, path_3->is_work);
	path_id.is_path_work[CAMERA_PATH0] = path_0->is_work;
	path_id.is_path_work[CAMERA_PRE_PATH] = path_1->is_work;
	path_id.is_path_work[CAMERA_VID_PATH] = path_2->is_work;
	path_id.is_path_work[CAMERA_CAP_PATH] = path_3->is_work;
	ret = sprd_camera_get_path_id(idx, &path_id, channel_id, scene_mode);

	DCAM_TRACE("get channel %d\n", *channel_id);

exit:
	return ret;
}

static long sprd_img_k_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	int ret = 0;
	unsigned int mode;
	unsigned int channel_id;
	unsigned int skip_num;
	unsigned int zoom;
	unsigned int led0_ctrl;
	unsigned int led1_ctrl;
	unsigned int led0_status;
	unsigned int led1_status;
	unsigned int iommu_enable;
	ktime_t boot_time;
	int64_t timestamp;
	struct camera_dev *dev = NULL;
	struct camera_info *info = NULL;
	struct camera_file *camerafile = NULL;
	struct camera_group *group = NULL;
	struct camera_path_spec *path = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_format *fmt;
	struct sprd_img_format img_format;
	struct sprd_img_get_fmt fmt_desc;
	struct sprd_img_size size;
	struct sprd_img_rect rect;
	struct sprd_img_parm parm;
	struct sprd_img_sensor_if sensor;
	struct sprd_img_time utime;
	struct sprd_flash_cfg_param cfg_parm;
	struct timeval time;
	struct sprd_img_res res = {0};
	struct sprd_img_capture_param capture_param;

	/*add this log to help debug an open camera error problem*/
	if (cmd == SPRD_IMG_IO_GET_DCAM_RES) {
		pr_info("%s cmd SPRD_IMG_IO_GET_DCAM_RES in\n", __func__);
	}

	camerafile = (struct camera_file *)file->private_data;
	if (!camerafile) {
		ret = -EFAULT;
		pr_err("camerafile is NULL\n");
		goto exit;
	}

	group = camerafile->grp;

	if ((_IOC_NR(cmd) >= _IOC_NR(SPRD_ISP_IO_CAPABILITY)) &&
		    (_IOC_NR(cmd) <= _IOC_NR(SPRD_ISP_IO_RAW_CAP))) {
		DCAM_TRACE("ispcompat isp ioctl cmd: 0x%x, %d\n",
				cmd, _IOC_NR(cmd));
	}

	if ((_IOC_NR(cmd) >= _IOC_NR(SPRD_IMG_IO_SET_MODE)) &&
	    (_IOC_NR(cmd) <= _IOC_NR(SPRD_IMG_IO_PUT_DCAM_RES))) {
		if ((_IOC_NR(cmd) >= _IOC_NR(SPRD_IMG_IO_SET_MODE)) &&
		    (_IOC_NR(cmd) <= _IOC_NR(SPRD_ISP_IO_RAW_CAP))) {
			idx = camerafile->idx;
			if (group->dev_inited & (1 << (int)idx)) {
				dev = group->dev[idx];
				if (!dev) {
					ret = -EFAULT;
					pr_err("ispcompat dev[%d] is NULL\n",
							idx);
					goto exit;
				}
				info = &dev->dcam_cxt;
			} else {
				ret = -EFAULT;
				pr_err("ispcompat dev[%d] not inited!\n", idx);
				goto exit;
			}
		}
	} else {
		pr_info("ispcompat invalid cmd 0x%x\n", cmd);
		goto exit;
	}

	DCAM_TRACE("dcam ioctl: %d, cmd: 0x%x, %d\n",
			   idx, cmd, _IOC_NR(cmd));
	switch (cmd) {
	case SPRD_IMG_IO_GET_DCAM_RES:
		ret = copy_from_user(&res, (struct sprd_img_res *)arg,
				     sizeof(struct sprd_img_res));

		if (group->dev[idx] == NULL) {
			pr_err("dev%d is NULL\n", idx);
			ret = -EINVAL;
			break;
		}
		ret = sprd_img_get_res(group, &res);
		if (ret) {
			pr_err("get res failed!\n");
			break;
		}

		idx = sprd_sensor_find_dcam_id(res.sensor_id);
		if (group->mode_inited & (1 << (int)idx)) {
			pr_info("dcam%d has been enabled!\n", idx);
			break;
		}
		ret = sprd_dcam_module_en(idx);
		if (unlikely(ret != 0)) {
			pr_err("%s: Failed to enable dcam module %d\n",
				__func__, idx);
			break;
		}
		ret = sprd_isp_module_en(group->dev[idx]->isp_dev_handle, idx);
		if (unlikely(ret != 0)) {
			pr_err("%s: Failed to enable isp module %d\n",
				__func__, idx);
			break;
		}
		camerafile->idx = idx;
		group->mode_inited |= 1 << idx;
		pr_info("%s: dcam dev[%d] init OK: 0x%lx\n", __func__, idx,
				(unsigned long)group->dev[idx]);
		pr_info("sprd_img idx %d inited %d res_used %d\n",
				idx, group->dev_inited, group->dcam_res_used);

		ret = copy_to_user((struct sprd_img_res *)arg, &res,
				   sizeof(struct sprd_img_res));
		break;
	case SPRD_IMG_IO_PUT_DCAM_RES:
		ret = copy_from_user(&res, (struct sprd_img_res *)arg,
				     sizeof(struct sprd_img_res));

		idx = sprd_sensor_find_dcam_id(res.sensor_id);
		pr_info("dev_inited:%d, idx: %d\n", group->dev_inited, idx);
		if (!(group->mode_inited & (1 << (int)idx))) {
			pr_err("dcam%d has been already disabled!\n", idx);
			break;
		}
		ret = sprd_dcam_module_dis(idx);
		if (unlikely(ret != 0)) {
			pr_err("SPRD_IMG%d: Failed to disable dcam module\n",
				idx);
		}
		ret = sprd_isp_module_dis(group->dev[idx]->isp_dev_handle, idx);
		if (unlikely(ret != 0)) {
			pr_err("SPRD_IMG%d: Failed to disable isp module\n",
				idx);
		}
		group->mode_inited &= ~(1<<idx);

		ret = sprd_img_put_res(group, &res);
		if (ret) {
			pr_err("put res failed!\n");
			break;
		}
		ret = copy_to_user((struct sprd_img_res *)arg, &res,
				   sizeof(struct sprd_img_res));
		break;
	case SPRD_IMG_IO_SET_MODE:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&mode, (unsigned int *) arg,
				     sizeof(unsigned int));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		info->capture_mode = mode;
		mutex_unlock(&dev->dcam_mutex);
		pr_debug("%s: capture mode %d\n", __func__,
			   dev->dcam_cxt.capture_mode);
		break;
	case SPRD_IMG_IO_SET_CAP_SKIP_NUM:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&skip_num, (unsigned int *) arg,
				     sizeof(unsigned int));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		info->skip_number = skip_num;
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("%d: cap skip number %d\n", idx,
			   dev->dcam_cxt.skip_number);
		break;
	case SPRD_IMG_IO_SET_PATH_SKIP_NUM:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (void __user *)arg,
				     sizeof(struct sprd_img_parm));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}

		switch (parm.channel_id) {
		case CAMERA_PATH0:
		case CAMERA_PRE_PATH:
		case CAMERA_VID_PATH:
		case CAMERA_CAP_PATH:
			info->dcam_path[parm.channel_id].skip_num =
				parm.skip_num;
			break;
		default:
			pr_info("%d: wrong channel ID, %d\n", idx,
				parm.channel_id);
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("%d: channel %d, skip_num %d\n",
			   idx, parm.channel_id, parm.skip_num);
		break;

	case SPRD_IMG_IO_SET_SENSOR_SIZE:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&size, (struct sprd_img_size *)arg,
				     sizeof(struct sprd_img_size));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		info->cap_in_size.w = size.w;
		info->cap_in_size.h = size.h;
		mutex_unlock(&dev->dcam_mutex);
		pr_debug("%s: sensor size %d %d\n", __func__,
			   info->cap_in_size.w, info->cap_in_size.h);
		break;
	case SPRD_IMG_IO_SET_SENSOR_TRIM:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&rect, (struct sprd_img_rect *)arg,
				     sizeof(struct sprd_img_rect));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		info->cap_in_rect.x = rect.x;
		info->cap_in_rect.y = rect.y;
		info->cap_in_rect.w = rect.w;
		info->cap_in_rect.h = rect.h;

		info->cap_out_size.w = info->cap_in_rect.w;
		info->cap_out_size.h = info->cap_in_rect.h;
		mutex_unlock(&dev->dcam_mutex);
		pr_debug("%s: sensor trim x y w h %d %d %d %d\n", __func__,
			   info->cap_in_rect.x,
			   info->cap_in_rect.y,
			   info->cap_in_rect.w,
			   info->cap_in_rect.h);
		break;
	case SPRD_IMG_IO_SET_FRM_ID_BASE:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (struct sprd_img_parm *)arg,
				     sizeof(struct sprd_img_parm));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}

		switch (parm.channel_id) {
		case CAMERA_PATH0:
		case CAMERA_PRE_PATH:
		case CAMERA_VID_PATH:
		case CAMERA_CAP_PATH:
			info->dcam_path[parm.channel_id].frm_id_base =
				parm.frame_base_id;
			break;
		default:
			pr_info("%d: wrong channel ID, %d\n", idx,
				parm.channel_id);
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("%d: channel %d, base id 0x%x\n",
			   idx, parm.channel_id, parm.frame_base_id);
		break;
	case SPRD_IMG_IO_SET_CROP:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (struct sprd_img_parm *)arg,
				     sizeof(struct sprd_img_parm));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}

		ret = sprd_img_set_crop(file, &parm);
		mutex_unlock(&dev->dcam_mutex);
		break;
	case SPRD_IMG_IO_GET_FLASH_INFO:
		sprd_flash_get_info(info->set_flash.flash_index,
			SPRD_FLASH_LED_ALL, &info->capacity);

		ret = copy_to_user((struct sprd_flash_capacity *)arg,
			&info->capacity, sizeof(struct sprd_flash_capacity));

		DCAM_TRACE("get flash info, ret %d\n", ret);
		break;
	case SPRD_IMG_IO_SET_FLASH:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&info->set_flash,
				     (struct sprd_img_set_flash *)arg,
				     sizeof(struct sprd_img_set_flash));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}

		led0_ctrl = info->set_flash.led0_ctrl;
		led1_ctrl = info->set_flash.led1_ctrl;
		led0_status = info->set_flash.led0_status;
		led1_status = info->set_flash.led1_status;

		mutex_unlock(&dev->dcam_mutex);
		if ((led0_ctrl &&
		     (led0_status == FLASH_CLOSE_AFTER_OPEN ||
		      led0_status == FLASH_CLOSE ||
		      led0_status == FLASH_CLOSE_AFTER_AUTOFOCUS)) ||
		    (led1_ctrl &&
		     (led1_status == FLASH_CLOSE_AFTER_OPEN ||
		      led1_status == FLASH_CLOSE ||
		      led1_status == FLASH_CLOSE_AFTER_AUTOFOCUS))) {
			complete(&dev->flash_thread_com);
		}

		DCAM_TRACE("led0_ctrl %d led0_status %d\n",
			   info->set_flash.led0_ctrl,
			   info->set_flash.led0_status);
		DCAM_TRACE("led1_ctrl %d led1_status %d\n",
			   info->set_flash.led1_ctrl,
			   info->set_flash.led1_status);
		break;
	case SPRD_IMG_IO_SET_OUTPUT_SIZE:
		DCAM_TRACE("%d: set output size\n", idx);
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (struct sprd_img_parm *)arg,
				     sizeof(struct sprd_img_parm));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		info->dst_size.w = parm.dst_size.w;
		info->dst_size.h = parm.dst_size.h;
		info->pxl_fmt = parm.pixel_fmt;
		info->need_isp_tool = parm.need_isp_tool;
		info->need_isp = parm.need_isp;
		info->rt_refocus = parm.rt_refocus;
		info->path_input_rect.x = parm.crop_rect.x;
		info->path_input_rect.y = parm.crop_rect.y;
		info->path_input_rect.w = parm.crop_rect.w;
		info->path_input_rect.h = parm.crop_rect.h;
		info->scene_mode = parm.scene_mode;
		info->raw_cap_flag = parm.need_isp_tool;
		info->slow_motion_flag = parm.slowmotion;

		if (parm.slowmotion)
			info->is_slow_motion = parm.slowmotion;
		mutex_unlock(&dev->dcam_mutex);

		pr_debug("%s input_rect, x:%d, y:%d, w:%d, h:%d\n", __func__,
					info->path_input_rect.x,
					info->path_input_rect.y,
					info->path_input_rect.w,
					info->path_input_rect.h);


		break;
	case SPRD_IMG_IO_SET_ZOOM_MODE:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&zoom, (unsigned int *) arg,
				     sizeof(unsigned int));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		info->is_smooth_zoom = zoom;
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("%d: set zoom mode %d\n", idx, info->is_smooth_zoom);
		break;
	case SPRD_IMG_IO_SET_SENSOR_IF:
		DCAM_TRACE("%d: set sensor if\n", idx);
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&sensor,
				     (struct sprd_img_sensor_if *)arg,
				     sizeof(struct sprd_img_sensor_if));
		if (unlikely(ret)) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}

		ret = sprd_img_set_sensor_if(file, &sensor);
		mutex_unlock(&dev->dcam_mutex);
		break;
	case SPRD_IMG_IO_SET_FRAME_ADDR:
		DCAM_TRACE("dcam idx=%d: set frame addr\n", idx);
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (struct sprd_img_parm *)arg,
				     sizeof(struct sprd_img_parm));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}

		ret = sprd_img_set_frame_addr(file, &parm);
		mutex_unlock(&dev->dcam_mutex);
		break;
	case SPRD_IMG_IO_PATH_FRM_DECI:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (struct sprd_img_parm *)arg,
				     sizeof(struct sprd_img_parm));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		path = &info->dcam_path[parm.channel_id];
		path->path_frm_deci = parm.deci;
		mutex_unlock(&dev->dcam_mutex);
		break;
	case SPRD_IMG_IO_SET_SHRINK:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (struct sprd_img_parm *)arg,
				     sizeof(struct sprd_img_parm));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		path = &info->dcam_path[parm.channel_id];
		path->regular_desc = parm.regular_desc;
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("channel %d, regular mode %d\n",
			   parm.channel_id, path->regular_desc.regular_mode);
		break;
	case SPRD_IMG_IO_PDAF_CONTROL:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (struct sprd_img_parm *)arg,
				     sizeof(struct sprd_img_parm));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		path = &info->dcam_path[parm.channel_id];
		path->pdaf_ctrl.mode = parm.pdaf_ctrl.mode;
		path->pdaf_ctrl.phase_data_type =
			parm.pdaf_ctrl.phase_data_type;
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("channel %d, pdaf mode %d type %d\n",
			   parm.channel_id, path->pdaf_ctrl.mode,
			   path->pdaf_ctrl.phase_data_type);
		break;
	case SPRD_IMG_IO_PATH_PAUSE:
		pr_debug("DCAM_CORE: Not support ioctrl %d\n", _IOC_NR(cmd));
		break;
	case SPRD_IMG_IO_PATH_RESUME:
		pr_debug("DCAM_CORE: Not support ioctrl %d\n", _IOC_NR(cmd));
		break;
	case SPRD_IMG_IO_STREAM_ON:
		pr_info("stream on\n");
		ret = sprd_camera_stream_on(file);
		break;
	case SPRD_IMG_IO_STREAM_OFF:
		pr_info("stream off\n");
		ret = sprd_camera_stream_off(file);
		break;
	case SPRD_IMG_IO_GET_FMT:
		DCAM_TRACE("get fmt\n");
		ret = copy_from_user(&fmt_desc,
				     (struct sprd_img_get_fmt *)arg,
				     sizeof(struct sprd_img_get_fmt));
		if (ret) {
			pr_err("failed to get user info\n");
			goto exit;
		}
		if (unlikely(fmt_desc.index >= ARRAY_SIZE(dcam_img_fmt)))
			return -EINVAL;

		fmt = &dcam_img_fmt[fmt_desc.index];
		fmt_desc.fmt = fmt->fourcc;

		ret = copy_to_user((struct sprd_img_get_fmt *)arg,
				   &fmt_desc,
				   sizeof(struct sprd_img_get_fmt));
		break;
	case SPRD_IMG_IO_GET_CH_ID:
		DCAM_TRACE("get free channel\n");
		sprd_img_get_free_channel(file, &channel_id, info->scene_mode);
		ret = copy_to_user((unsigned int *) arg, &channel_id,
				   sizeof(unsigned int));
		break;
	case SPRD_IMG_IO_GET_TIME:
		DCAM_TRACE("get time\n");
		img_get_timestamp(&time);
		utime.sec = time.tv_sec;
		utime.usec = time.tv_usec;
		ret = copy_to_user((struct sprd_img_time *)arg, &utime,
				   sizeof(struct sprd_img_time));
		break;
	case SPRD_IMG_IO_CHECK_FMT:
		DCAM_TRACE("check fmt\n");
		ret = copy_from_user(&img_format,
				     (struct sprd_img_format *)arg,
				     sizeof(struct sprd_img_format));
		if (ret) {
			pr_err("failed to get user info\n");
			goto exit;
		}

		ret = sprd_img_check_fmt(file, &img_format);
		if (ret)
			goto exit;

		ret = copy_to_user((struct sprd_img_format *)arg,
				   &img_format,
				   sizeof(struct sprd_img_format));
		break;
	case SPRD_IMG_IO_CFG_FLASH:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&cfg_parm,
				     (struct sprd_flash_cfg_param *)arg,
				     sizeof(struct sprd_flash_cfg_param));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		/*need add flash cfg here*/
		 ret = sprd_flash_cfg(&cfg_parm);
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("config flash, ret %d\n", ret);
		break;
	case SPRD_IMG_IO_START_CAPTURE:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&capture_param,
					(void __user *)arg,
					sizeof(capture_param));
		if (ret) {
			pr_err("failed to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
#ifdef ISP_PW_SAVE
		if (dev->cap_flag == 0) {
			boot_time = ktime_get_boottime();
			timestamp = boot_time.tv64;
			pr_info("DCAM_CORE: start capture =%d, %lld, %lld\n",
				capture_param.type,
				capture_param.timestamp, timestamp);
			dev->cap_scene = capture_param.type;
			sprd_isp_store_cce_bypass(dev->isp_dev_handle,
				dev->cap_scene, 0);
			dev->cap_flag = 1;
		} else {
			pr_info("Capture already running, please stop first\n");
		}
#else
		if (dev->cap_flag == 0) {
			boot_time = ktime_get_boottime();
			timestamp = boot_time.tv64;
			pr_info("DCAM_CORE: start capture =%d, %lld, %lld\n",
				capture_param.type,
				capture_param.timestamp, timestamp);

			ret = sprd_isp_start_fmcu(dev->isp_dev_handle,
					capture_param, 0);
			if (ret) {
				mutex_unlock(&dev->dcam_mutex);
				pr_err("DCAM_CORE: start offline err\n");
				goto exit;
			}
			dev->cap_flag = 1;
		}
#endif
		mutex_unlock(&dev->dcam_mutex);
		break;
	case SPRD_IMG_IO_STOP_CAPTURE:
		mutex_lock(&dev->dcam_mutex);
		pr_info("DCAM_CORE: stop capture\n");
#ifdef ISP_PW_SAVE
		sprd_isp_store_cce_bypass(dev->isp_dev_handle,
			dev->cap_scene, 1);
#endif
		if (dev->cap_flag == 1)
			dev->cap_flag = 0;
		ret = sprd_isp_fmcu_slice_stop(dev->isp_dev_handle);
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("DCAM_CORE: stop offline err\n");
			goto exit;
		}
		mutex_unlock(&dev->dcam_mutex);
		break;
	case SPRD_IMG_IO_GET_IOMMU_STATUS:
		ret = copy_from_user(&iommu_enable, (unsigned char *)arg,
				     sizeof(unsigned char));
		if (ret) {
			pr_err("copy_from_user failed\n");
			goto exit;
		}

		if (sprd_iommu_attach_device(&group->pdev->dev) == 0)
			iommu_enable = 1;
		else
			iommu_enable = 0;

		ret = copy_to_user((unsigned char *)arg, &iommu_enable,
				   sizeof(unsigned char));
		break;
	case SPRD_IMG_IO_SET_SENSOR_MAX_SIZE:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&size, (struct sprd_img_size *)arg,
				     sizeof(struct sprd_img_size));
		if (ret || !size.w || !size.h) {
			pr_err("failed to get right user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		info->sn_max_size.w = size.w;
		info->sn_max_size.h = size.h;
		mutex_unlock(&dev->dcam_mutex);
		pr_debug("%d: sensor max size %d %d\n", idx,
			   info->sn_max_size.w, info->sn_max_size.h);
		break;
	case SPRD_ISP_IO_CAPABILITY:
	case SPRD_ISP_IO_IRQ:
	case SPRD_ISP_IO_READ:
	case SPRD_ISP_IO_WRITE:
	case SPRD_ISP_IO_RST:
	case SPRD_ISP_IO_STOP:
	case SPRD_ISP_IO_INT:
	case SPRD_ISP_IO_CFG_PARAM:
	case SPRD_ISP_IO_SET_STATIS_BUF:
	case SPRD_ISP_REG_READ:
	case SPRD_ISP_IO_RAW_CAP:
	case SPRD_ISP_IO_POST_3DNR:
		mutex_lock(&dev->dcam_mutex);
		DCAM_TRACE("start isp raw cap ioctl cmd%d dev %p\n",
			_IOC_NR(cmd), dev);
		ret = sprd_isp_k_ioctl(dev->isp_dev_handle, cmd, arg);
		mutex_unlock(&dev->dcam_mutex);
		break;
#ifdef ISP_COMPAT_SUPPORT
	case COMPAT_SPRD_ISP_IO_SET_STATIS_BUF:
	case COMPAT_SPRD_ISP_IO_CFG_PARAM:
	case COMPAT_SPRD_ISP_IO_CAPABILITY:
		mutex_lock(&dev->dcam_mutex);
		DCAM_TRACE("ispcompat start isp compat ioctl cmd%d dev %p\n",
				_IOC_NR(cmd), dev);
		ret = compat_sprd_isp_k_ioctl(dev->isp_dev_handle, cmd, arg);
		mutex_unlock(&dev->dcam_mutex);
		break;
	case COMPAT_SPRD_ISP_IO_RAW_CAP:
		pr_info("ispcompat start isp compat ioctl cmd%d dev %p,stream_on:%d\n",
				_IOC_NR(cmd),
				dev,
				atomic_read(&dev->stream_on));
		mutex_lock(&dev->dcam_mutex);
		if (unlikely(atomic_read(&dev->stream_on) == 0)) {
			pr_info("%s raw_cap, idx: %d\n", __func__, idx);
			dev->raw_cap = 1;
			ret = sprd_img_queue_init(&dev->queue);
			if (unlikely(ret != 0)) {
				mutex_unlock(&dev->dcam_mutex);
				pr_err("failed to init queue\n");
				goto exit;
			}
			ret = sprd_img_isp_reg_isr(dev);
			if (unlikely(ret)) {
				pr_err("failed to register isp isr\n");
				mutex_unlock(&dev->dcam_mutex);
				goto exit;
			}
			atomic_set(&dev->stream_on, 1);
		}
		ret = compat_sprd_isp_k_ioctl(dev->isp_dev_handle, cmd, arg);
		mutex_unlock(&dev->dcam_mutex);
		break;
#endif /* ISP_COMPAT_SUPPORT */
	default:
		pr_info("unsupported cmd %d, 0x%x\n", _IOC_NR(cmd), cmd);
		break;
	}
	DCAM_TRACE("dcam ioctl end: %d, cmd: 0x%x, %d\n",
			   idx, cmd, _IOC_NR(cmd));

exit:
	return ret;
}

static ssize_t sprd_img_read(struct file *file, char __user *u_data,
			     size_t cnt, loff_t *cnt_ret)
{
	int ret = 0;
	int i = 0;
	struct camera_file *camerafile = file->private_data;
	struct camera_group *group = camerafile->grp;
	enum dcam_id idx = camerafile->idx;
	struct camera_dev *dev = group->dev[idx];
	struct camera_path_spec *path;
	struct sprd_img_read_op read_op;
	struct camera_node node;
	struct dcam_path_info *info;
	struct sprd_img_path_info *info_ret;
	struct dcam_path_capability path_capability;

	if (cnt < sizeof(struct sprd_img_read_op)) {
		pr_err("err, cnt %zd read_op %ld\n", cnt,
		       sizeof(struct sprd_img_read_op));
		return -EIO;
	}

	if (copy_from_user(&read_op, (void __user *)u_data, cnt)) {
		pr_err("failed to get user info\n");
		return -EFAULT;
	}

	if (!dev) {
		pr_info("dev is not init %p\n", dev);
		return -EFAULT;
	}

	switch (read_op.cmd) {
	case SPRD_IMG_GET_SCALE_CAP:
		DCAM_TRACE("get scale capbility\n");
		if (idx == DCAM_ID_1) {
			read_op.parm.reserved[0] = ISP1_PATH2_LINE_BUF_LENGTH;
			read_op.parm.reserved[1] = CAMERA_SC_COEFF_UP_MAX;
			read_op.parm.reserved[2] = DCAM1_SCALING_THRESHOLD;
		} else {
			read_op.parm.reserved[0] = ISP_PATH2_LINE_BUF_LENGTH;
			read_op.parm.reserved[1] = CAMERA_SC_COEFF_UP_MAX;
			read_op.parm.reserved[2] = DCAM_SCALING_THRESHOLD;
		}
		DCAM_TRACE("cap %d, threshold %d, factor %d, scaling %d.\n",
			   idx,
			   read_op.parm.reserved[0],
			   read_op.parm.reserved[1],
			   read_op.parm.reserved[2]);
		break;
	case SPRD_IMG_GET_FRM_BUFFER:
		idx = sprd_sensor_find_dcam_id(read_op.sensor_id);
		dev = group->dev[idx];
		memset(&read_op, 0, sizeof(struct sprd_img_read_op));
		while (1) {
			ret = wait_for_completion_interruptible(&dev->irq_com);
			if (ret == 0) {
				break;
			} else if (ret == -ERESTARTSYS) {
				read_op.evt = IMG_SYS_BUSY;
				ret = DCAM_RTN_SUCCESS;
				goto read_end;
			} else {
				pr_err("read frame buf, failed to down, %d\n",
				       ret);

				return -EPERM;
			}
		}

		if (sprd_img_queue_read(&dev->queue, &node)) {
			DCAM_TRACE("read frame buffer, queue is null\n");
			read_op.evt = IMG_SYS_BUSY;
			ret = DCAM_RTN_SUCCESS;
			goto read_end;
		} else {
			if (node.invalid_flag) {
				DCAM_TRACE("read frame buffer, invalid node\n");
				read_op.evt = IMG_SYS_BUSY;
				ret = DCAM_RTN_SUCCESS;
				goto read_end;
			}
		}

		read_op.evt = node.irq_flag;
		if (read_op.evt == IMG_TX_DONE) {
			read_op.parm.frame.channel_id = node.f_type;
			path =
				&dev->dcam_cxt.dcam_path[read_op.parm.frame.
				channel_id];
			DCAM_TRACE("node, %p %d 0x%x\n",
				   (void *)&node, (node.index),
				   path->frm_id_base);
			read_op.parm.frame.index = path->frm_id_base;
			read_op.parm.frame.height = node.height;
			read_op.parm.frame.length = node.reserved[0];
			read_op.parm.frame.sec = node.time.tv_sec;
			read_op.parm.frame.usec = node.time.tv_usec;
			read_op.parm.frame.monoboottime = node.boot_time.tv64;
			read_op.parm.frame.frm_base_id = path->frm_id_base;
			read_op.parm.frame.img_fmt = path->fourcc;
			read_op.parm.frame.yaddr = node.yaddr;
			read_op.parm.frame.uaddr = node.uaddr;
			read_op.parm.frame.vaddr = node.vaddr;
			read_op.parm.frame.yaddr_vir = node.yaddr_vir;
			read_op.parm.frame.uaddr_vir = node.uaddr_vir;
			read_op.parm.frame.vaddr_vir = node.vaddr_vir;
			read_op.parm.frame.phy_addr = node.phy_addr;
			read_op.parm.frame.vir_addr = node.vir_addr;
			read_op.parm.frame.kaddr[0] = node.kaddr[0];
			read_op.parm.frame.kaddr[1] = node.kaddr[1];
			read_op.parm.frame.buf_size = node.buf_size;
			read_op.parm.frame.irq_type = node.irq_type;
			read_op.parm.frame.irq_property = node.irq_property;
			read_op.parm.frame.mfd = node.mfd[0];

			if (read_op.parm.frame.irq_type == CAMERA_IRQ_DONE)
				isp_irq_done_read(idx, node.irq_property);

			pr_debug("get frame channel_id:%d, height:%d, irq_type:0x%x, yaddr_vir:0x%x, yaddr:0x%x, sec:%u, usec:%u, mfd:0x%x\n",
					read_op.parm.frame.channel_id,
					read_op.parm.frame.height,
					node.irq_type,
				   read_op.parm.frame.yaddr_vir,
				   read_op.parm.frame.yaddr,
				   read_op.parm.frame.sec,
				   read_op.parm.frame.usec,
				   node.mfd[0]);
		} else {
			if (read_op.evt == IMG_TIMEOUT ||
			    read_op.evt == IMG_TX_ERR) {
				read_op.parm.frame.irq_type = node.irq_type;
				pr_info("DCAM_CORE: error evt %d\n",
					read_op.evt);
				read_op.parm.frame.irq_property
					= node.irq_property;

			}
		}
		DCAM_TRACE("read frame buf, evt 0x%x channel 0x%x index 0x%x\n",
			   read_op.evt, read_op.parm.frame.channel_id,
			   read_op.parm.frame.index);

		if (read_op.parm.frame.irq_property == IRQ_RAW_CAP_DONE) {
			pr_info("sprd_img_read: raw cap\n");
			if (dev->raw_cap == 1) {
				dev->raw_cap = 0;
				ret = sprd_img_queue_init(&dev->queue);
				if (unlikely(ret != 0))
					pr_err("failed to init queue\n");
				ret = sprd_img_isp_unreg_isr(dev);
				if (unlikely(ret))
					pr_err("failed to register isp isr\n");
				atomic_set(&dev->stream_on, 0);
			}
		}
		if (read_op.evt == IMG_TX_STOP)
			pr_info("sprd_img_read: tx stop\n");

		break;
	case SPRD_IMG_GET_PATH_CAP:
		DCAM_TRACE("get path capbility\n");
		sprd_dcam_get_path_capability(&path_capability);
		read_op.parm.capability.count = path_capability.count;
		read_op.parm.capability.support_3dnr_mode =
			path_capability.support_3dnr_mode;
		for (i = 0; i < path_capability.count; i++) {
			info = &path_capability.path_info[i];
			info_ret = &read_op.parm.capability.path_info[i];
			info_ret->line_buf = info->line_buf;
			info_ret->support_yuv = info->support_yuv;
			info_ret->support_raw = info->support_raw;
			info_ret->support_jpeg = info->support_jpeg;
			info_ret->support_scaling = info->support_scaling;
			info_ret->support_trim = info->support_trim;
			info_ret->is_scaleing_path = info->is_scaleing_path;
		}
		break;
	default:
		pr_err("invalid cmd\n");
		return -EINVAL;
	}

read_end:
	if (copy_to_user((void __user *)u_data, &read_op, cnt)) {
		pr_debug("%s err\n", __func__);
		ret = -EFAULT;
	}

	if (ret)
		cnt = ret;

	pr_debug("%s out, cnt:%ld\n", __func__, cnt);
	return cnt;
}

static ssize_t sprd_img_write(struct file *file, const char __user *u_data,
			      size_t cnt, loff_t *cnt_ret)
{
	struct camera_file *camerafile = file->private_data;
	struct camera_group *group = camerafile->grp;
	enum dcam_id idx = camerafile->idx;
	struct camera_dev *dev = group->dev[idx];
	struct camera_info *info = &dev->dcam_cxt;
	struct sprd_img_write_op write_op;
	struct camera_path_spec *path;
	int ret = 0;
	unsigned int index;

	if (cnt < sizeof(struct sprd_img_write_op)) {
		pr_err("err, cnt %zd read_op %ld\n", cnt,
		       sizeof(struct sprd_img_write_op));
		return -EIO;
	}

	if (copy_from_user(&write_op, (void __user *)u_data, cnt)) {
		pr_err("failed to get user info\n");
		return -EFAULT;
	}

	if (!dev)
		return -EFAULT;

	switch (write_op.cmd) {
	case SPRD_IMG_STOP_DCAM:
		idx = sprd_sensor_find_dcam_id(write_op.sensor_id);
		dev = group->dev[idx];
		if (!dev) {
			pr_err("sprd_img_write dev NULL\n");
			return -EFAULT;
		}
		mutex_lock(&dev->dcam_mutex);
		ret = sprd_img_tx_stop(dev);
		mutex_unlock(&dev->dcam_mutex);
		break;
	case SPRD_IMG_FREE_FRAME:
		if (atomic_read(&dev->stream_on) == 0) {
			pr_info("dev close, no need free!");
			break;
		}

		switch (write_op.channel_id) {
		case CAMERA_PATH0:
		case CAMERA_PRE_PATH:
		case CAMERA_VID_PATH:
		case CAMERA_CAP_PATH:
			path = &info->dcam_path[write_op.channel_id];
			break;
		default:
			pr_err("failed to free frame buf, channel %d\n",
			       write_op.channel_id);
			return -EINVAL;
		}

		if (path->status == PATH_IDLE) {
			DCAM_TRACE("failed to free frame buf, channel %d\n",
				   write_op.channel_id);
			return -EINVAL;
		}

		if (write_op.index >
		    (path->frm_id_base + path->frm_cnt_act - 1)) {
			pr_err("err, index %d, frm_id_base %d frm_cnt_act %d\n",
			       write_op.index, path->frm_id_base,
			       path->frm_cnt_act);
			ret = -EINVAL;
		} else if (write_op.index < path->frm_id_base) {
			pr_err("err, index %d, frm_id_base %d\n",
			       write_op.index, path->frm_id_base);
			ret = -EINVAL;
		} else {
			index = write_op.index - path->frm_id_base;

			DCAM_TRACE("free frame buf, channel %d, index 0x%x\n",
				   write_op.channel_id, write_op.index);
		}
		break;
	default:
		pr_err("cmd error!\n");
		ret = -EINVAL;
		break;
	}

	if (ret)
		cnt = ret;

	return cnt;
}

static const struct file_operations image_fops = {
	.open = sprd_img_k_open,
	.unlocked_ioctl = sprd_img_k_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = sprd_img_k_ioctl,
#endif
	.release = sprd_img_k_release,
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
		pr_err("Input pdev is NULL\n");
		return -EFAULT;
	}

	pr_info("Start camera img probe\n");
	group = vzalloc(sizeof(struct camera_group));
	if (group == NULL)
		return -ENOMEM;

	ret = misc_register(&image_dev);
	if (ret) {
		pr_err("failed to register misc devices, ret %d\n", ret);
		return -EACCES;
	}

	image_dev.this_device->of_node = pdev->dev.of_node;
	image_dev.this_device->platform_data = (void *)group;
	group->md = &image_dev;
	group->pdev = pdev;
	atomic_set(&group->camera_opened, 0);

	pr_info("sprd img probe pdev name %s\n", pdev->name);

	ret = sprd_cam_pw_domain_init(pdev);
	if (ret) {
		pr_err("failed to init pw domain\n");
		goto exit;
	}

	pr_info("sprd dcam dev name %s\n", pdev->dev.init_name);
	ret = sprd_dcam_parse_dt(pdev->dev.of_node, &group->dcam_count);
	if (ret) {
		pr_err("failed to parse dcam dts\n");
		goto exit;
	}

	pr_info("sprd isp dev name %s\n", pdev->dev.init_name);
	ret = sprd_isp_parse_dt(pdev->dev.of_node, &group->isp_count);
	if (ret) {
		pr_err("failed to parse isp dts\n");
		goto exit;
	}

	if (group->dcam_count != group->isp_count) {
		pr_err("dts tree config is error\n");
		ret = -EINVAL;
		goto exit;
	}

	if (sprd_dcam_drv_init(pdev)) {
		pr_err("failed to call sprd_dcam_drv_init\n");
		ret = -EINVAL;
		goto exit;
	}

	if (sprd_isp_drv_init(pdev)) {
		pr_err("failed to call sprd_isp_drv_init\n");
		misc_deregister(&image_dev);
		sprd_dcam_drv_deinit();
		return -EINVAL;
	}

	return ret;

exit:
	misc_deregister(&image_dev);
	return ret;
}

static int sprd_img_remove(struct platform_device *pdev)
{
	struct camera_group *group = NULL;

	if (!pdev) {
		pr_err("Input pdev is NULL\n");
		return -EFAULT;
	}

	group = image_dev.this_device->platform_data;

	sprd_isp_drv_deinit();
	sprd_dcam_drv_deinit();
	vfree(image_dev.this_device->platform_data);
	misc_deregister(&image_dev);

	return 0;
}

static void sprd_img_shutdown(struct platform_device *pdev)
{
	struct sprd_img_set_flash set_flash;

	set_flash.led0_ctrl = 1;
	set_flash.led1_ctrl = 1;
	set_flash.led0_status = FLASH_CLOSE;
	set_flash.led1_status = FLASH_CLOSE;
	set_flash.flash_index = 0;
	sprd_img_setflash(DCAM_ID_0, &set_flash);
	set_flash.flash_index = 1;
	sprd_img_setflash(DCAM_ID_0, &set_flash);
}

static const struct of_device_id sprd_dcam_of_match[] = {
	{ .compatible = "sprd,dcam", },
	{},
};

static struct platform_driver sprd_img_driver = {
	.probe = sprd_img_probe,
	.remove = sprd_img_remove,
	.shutdown = sprd_img_shutdown,
	.driver = {
		.owner = THIS_MODULE,
		.name = IMG_DEVICE_NAME,
		.of_match_table = of_match_ptr(sprd_dcam_of_match),
	},
};

module_platform_driver(sprd_img_driver);

MODULE_DESCRIPTION("Sprd DCAM Driver");
MODULE_AUTHOR("Multimedia_Camera@Spreadtrum");
MODULE_LICENSE("GPL");
