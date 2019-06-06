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
#include "cam_pw_domain.h"
#include "dcam_drv.h"
#include "flash_drv.h"
#include "isp_drv.h"
#include "isp_int.h"
#include "isp_reg.h"
#include "isp_slw.h"
#include "sprd_sensor_drv.h"
#ifdef CONFIG_COMPAT
#include "compat_isp_drv.h"
#endif
#include "csi_api.h"

/* Macro Definitions */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_CORE: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define IMG_DEVICE_NAME			"sprd_image"
#define CAMERA_QUEUE_LENGTH		96

#ifdef CONFIG_SC_FPGA
#define DCAM_TIMEOUT			(2000*100)
#else
#define DCAM_TIMEOUT			1500
#endif

#define CAP_IN_SIZE_13M_WIDTH		4224
#define CAP_IN_SIZE_8M_WIDTH		3264
#define CAP_IN_SIZE_5M_WIDTH		2592
#define CAP_IN_SIZE_2M_WIDTH		1600
#define CAP_IN_SIZE_8MBIN_WIDTH		1632
#define CAP_IN_SIZE_VGA_WIDTH		640

/* Structure Definitions */

enum {
	PATH_IDLE = 0x00,
	PATH_RUN,
};

enum {
	CAMERA_CAPTURE_STOP,
	CAMERA_CAPTURE_START,
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
	unsigned int addr_offset;
	unsigned int kaddr[2];
	unsigned int buf_size;
	unsigned int irq_property;
	unsigned int invalid_flag;
	unsigned int frame_id;
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
	unsigned int count;
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
	unsigned int sn_fmt;
	unsigned int sensor_id;
	unsigned int need_isp_tool;
	unsigned int need_isp;
	unsigned int rt_refocus;
	struct camera_rect path_input_rect;
	struct camera_path_spec dcam_path[CAMERA_MAX_PATH];
	unsigned int capture_mode;
	unsigned int skip_number;
	struct sprd_img_set_flash set_flash;
	struct dcam_sbs_info sbs_info;
	unsigned int after_af;
	unsigned int is_smooth_zoom;
	struct timeval timestamp;
	struct isp_raw_proc_info raw_cap;
	unsigned int scene_mode;
	unsigned int is_slow_motion;
	unsigned int raw_cap_flag;
	unsigned int slow_motion_flag;
	struct sprd_flash_capacity capacity;
};

struct camera_dev {
	struct mutex dcam_mutex;
	struct completion irq_com;
	struct completion irq_com_sub;
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
	unsigned int channel_id;
	unsigned int use_path;
	enum dcam_id idx;
	void *isp_dev_handle;
	struct isp_if_context *ispif;
	unsigned int cap_flag;
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
	void *camerafile;
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
	if (queue == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

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
		DCAM_TRACE("fail to get valid input ptr\n");
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
		DCAM_TRACE("fail to get valid input ptr\n");
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
		DCAM_TRACE("fail to get valid input ptr\n");
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
		DCAM_TRACE("fail to get valid input ptr\n");
		return -1;
	}
	dev->is_flash_thread_stop = 0;
	init_completion(&dev->flash_thread_com);
	sprintf(thread_name, "dcam%d_flash_thread", dev->idx);
	dev->flash_thread = kthread_run(flash_thread_loop, param, thread_name);
	if (IS_ERR(dev->flash_thread)) {
		pr_err("fail to create flash thread\n");
		return -1;
	}

	return 0;
}

static int dcam_stop_flash_thread(void *param)
{
	struct camera_dev *dev = (struct camera_dev *)param;

	if (dev == NULL) {
		DCAM_TRACE("fail to get valid input ptr\n");
		return -1;
	}

	if (dev->flash_thread) {
		dev->is_flash_thread_stop = 1;
		complete(&dev->flash_thread_com);
		if (dev->is_flash_thread_stop != 0) {
			while (dev->is_flash_thread_stop)
				udelay(1000);
		}
		dev->flash_thread = NULL;
	}

	return 0;
}

static int sprd_img_get_path_index(unsigned int channel_id)
{
	int path_index = 0;

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
		pr_info("fail to get path index, channel %d\n", channel_id);
	}

	return path_index;
}

static int sprd_img_queue_init(struct camera_queue *queue)
{
	unsigned long flags;

	if (queue == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flags);
	memset(queue->node, 0,
	       sizeof(struct camera_node) * CAMERA_QUEUE_LENGTH);
	queue->write = &queue->node[0];
	queue->read = &queue->node[0];
	queue->wcnt = 0;
	queue->rcnt = 0;
	queue->count = 0;
	spin_unlock_irqrestore(&queue->lock, flags);
	pr_info("End sprd_img_queue_init\n");

	return 0;
}

static int sprd_img_local_deinit(struct camera_dev *dev)
{
	int ret = 0;
	int i;
	struct camera_path_spec *path;

	if (dev == NULL) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	for (i = 0; i < CAMERA_MAX_PATH; i++) {
		path = &dev->dcam_cxt.dcam_path[i];
		DCAM_TRACE("local_deinit, path %d cnt %d\n",
			   i, path->frm_cnt_act);
		if (path == NULL) {
			pr_err("fail to get valid path\n");
			return -EINVAL;
		}

		path->is_work = 0;
		path->frm_cnt_act = 0;
		sprd_img_buf_queue_init(&path->buf_queue);
		sprd_img_buf_queue_init(&path->tmp_buf_queue);
	}

	ret = sprd_img_queue_init(&dev->queue);
	if (unlikely(ret != 0)) {
		pr_err("fail to init queue\n");
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
		pr_err("fail to get valid input ptr\n");
		ret = -EINVAL;
		goto exit;
	}

	ret = set_dcam_path0_cfg(idx, DCAM_PATH_FRM_DECI,
		&path0->path_frm_deci);
	if (unlikely(ret)) {
		pr_err("fail to cfg path0 frame deci\n");
		goto exit;
	}

	ret = set_dcam_path0_cfg(idx, DCAM_PATH_DATA_ENDIAN, &path0->end_sel);
	if (unlikely(ret)) {
		pr_err("fail to cfg path0 data endian\n");
		goto exit;
	}

	ret = set_dcam_path0_cfg(idx, DCAM_PATH_INPUT_RECT, &path0->in_rect);
	if (unlikely(ret)) {
		pr_err("fail to cfg path0 input rect\n");
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
			pr_err("fail to cfg path0 output addr\n");
			goto exit;
		}
	}

	ret = set_dcam_path0_cfg(idx, DCAM_PATH_OUTPUT_RESERVED_ADDR,
		&path0->frm_reserved_addr);
	if (unlikely(ret)) {
		pr_err("fail to cfg path0 output reserved addr\n");
		goto exit;
	}

	param = 1;
	ret = set_dcam_path0_cfg(idx, DCAM_PATH_ENABLE, &param);
	if (unlikely(ret)) {
		pr_err("fail to enable path0\n");
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
		pr_err("fail to get valid input ptr\n");
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
			pr_err("fail to cfg pdaf path output addr\n");
			goto exit;
		}
	}

	ret = set_dcam_path_pdaf_cfg(idx, DCAM_PATH_OUTPUT_RESERVED_ADDR,
		&path_pdaf->frm_reserved_addr);
	if (unlikely(ret)) {
		pr_err("fail to cfg pdaf path output reserved addr\n");
		goto exit;
	}

	ret = set_dcam_path_pdaf_cfg(idx, DCAM_PDAF_CONTROL,
		&path_pdaf->pdaf_ctrl);
	if (unlikely(ret)) {
		pr_err("fail to cfg pdaf path control\n");
		goto exit;
	}

	param = 1;
	ret = set_dcam_path_pdaf_cfg(idx, DCAM_PATH_ENABLE, &param);
	if (unlikely(ret)) {
		pr_err("fail to enable pdaf path\n");
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
		pr_err("fail to get valid input ptr\n");
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
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_INPUT_SIZE,
		&path->in_size);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d input size\n", path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_INPUT_RECT,
		&path->in_rect);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d input rect\n", path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_OUTPUT_SIZE,
		&path->out_size);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d output size\n", path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_OUTPUT_FORMAT,
		&path->out_fmt);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d output format\n", path_index);
		goto exit;
	}

	ret = set_isp_path_buf_cfg(handle, ISP_PATH_OUTPUT_ADDR, path_index,
		&path->buf_queue);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d output addr\n", path_index);
		goto exit;
	}

	ret = set_isp_path_buf_cfg(handle, ISP_PATH_STORE_CCE_ADDR, path_index,
		&path->tmp_buf_queue);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d store cce addr\n", path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(handle, path_index,
		ISP_PATH_OUTPUT_RESERVED_ADDR, &path->frm_reserved_addr);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d output reserved addr\n",
			path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_FRM_DECI,
		&path->path_frm_deci);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d frame deci\n", path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_MODE,
		&path->path_mode);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d mode\n", path_index);
		goto exit;
	}

	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_SKIP_NUM,
			       &path->skip_num);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d skip num\n", path_index);
		goto exit;
	}

	memset(&regular_info, 0, sizeof(regular_info));
	regular_info.regular_mode = path->regular_desc.regular_mode;
	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_SHRINK,
			       &regular_info);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d shrink\n", path_index);
		goto exit;
	}

	param = 1;
	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_ENABLE, &param);
	if (unlikely(ret)) {
		pr_err("fail to enable path %d\n", path_index);
		goto exit;
	}

	endian.y_endian = path->end_sel.y_endian;
	endian.uv_endian = path->end_sel.uv_endian;
	ret = set_isp_path_cfg(handle, path_index, ISP_PATH_DATA_ENDIAN,
		&endian);
	if (unlikely(ret)) {
		pr_err("fail to cfg path %d data endian\n", path_index);
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
		pr_info("fail to get valid image format for path0 0x%x\n",
			fourcc);
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

	if (unlikely(i == ARRAY_SIZE(dcam_img_fmt))) {
		pr_err("fail to get valid image format\n");
		return NULL;
	}

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
			if (tempw > DCAM_ISP_LINE_BUF_LENGTH) {
				pr_err("fail to get valid input rect_w %d\n",
					path->in_rect.w);
				return -EINVAL;
			}

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
	if (tempw == f->width && temph == f->height) {
		pr_info("Do not need scale\n");
		return 0;
	}

	/* scaling needed */
	switch (info->sn_mode) {
	case DCAM_CAP_MODE_RAWRGB:
		maxw = f->width * CAMERA_SC_COEFF_DOWN_MAX;
		maxw = maxw * (1 << CAMERA_PATH_DECI_FAC_MAX);
		maxh = f->height * CAMERA_SC_COEFF_DOWN_MAX;
		maxh = maxh * (1 << CAMERA_PATH_DECI_FAC_MAX);
		if (unlikely(tempw > maxw || temph > maxh)) {
			/* out of scaling capbility */
			pr_err("fail to scale:out of scaling capbility\n");
			return -EINVAL;
		}

		if (unlikely(f->width > line_buf_size)) {
			/* out of scaling capbility. TBD */
			pr_err("fail to scale:out of scaling capbility\n");
			/*return -EINVAL;*/
		}

		maxw = tempw * CAMERA_SC_COEFF_UP_MAX;
		maxh = temph * CAMERA_SC_COEFF_UP_MAX;
		if (unlikely(f->width > maxw || f->height > maxh)) {
			/* out of scaling capbility */
			pr_err("fail to scale:out of scaling capbility\n");
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

static int sprd_img_check_path_cap(unsigned int fourcc,
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
		line_buf_size = ISP_PATH1_LINE_BUF_LENGTH;
		break;
	case CAMERA_VID_PATH:
		line_buf_size = ISP_PATH2_LINE_BUF_LENGTH;
		break;
	case CAMERA_CAP_PATH:
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
			pr_err("fail to scale, src %d %d, dst %d %d\n",
			       tempw, temph, f->width, f->height);
			return -EINVAL;
		}

		if (fourcc == IMG_PIX_FMT_GREY) {
			if (unlikely(info->sn_mode != DCAM_CAP_MODE_RAWRGB)) {
				/* the output of sensor is not RawRGB
				 * which is needed by app
				 */
				pr_err("fail to get RawRGB sensor\n");
				return -EINVAL;
			}

			path->out_fmt = DCAM_RAWRGB;
			path->end_sel.y_endian = DCAM_ENDIAN_BIG;
		} else if (fourcc == IMG_PIX_FMT_JPEG) {
			if (unlikely(info->sn_mode != DCAM_CAP_MODE_JPEG)) {
				/* the output of sensor is not JPEG
				 * which is needed by app
				 */
				pr_err("fail to get JPEG sensor\n");
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
				pr_err("fail to get valid format 0x%x raw sensor\n",
					fourcc);
				return -EINVAL;
			}
		} else {
			pr_err("fail to get valid sensor mode 0x%x\n",
				info->sn_mode);
			return -EINVAL;
		}

		sprd_img_endian_sel(fourcc, path);
		break;
	default:
		pr_info("fail to get valid image format for path %d 0x%x\n",
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
	unsigned long flags;
	struct camera_node *ori_node;
	int ret = 0;

	if (NULL == queue || NULL == node) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flags);
	ori_node = queue->write;
	queue->wcnt++;
	queue->count++;
	*queue->write++ = *node;
	if (queue->write > &queue->node[CAMERA_QUEUE_LENGTH - 1])
		queue->write = &queue->node[0];

	if (queue->write == queue->read) {
		queue->write = ori_node;
		queue->count--;
		pr_info("fail to get valid queue : full\n");
	}

	spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

static int sprd_img_queue_read(struct camera_queue *queue,
				struct camera_node *node)
{
	unsigned long flags;
	int ret = DCAM_RTN_SUCCESS;
	int flag = 0;

	if (NULL == queue || NULL == node) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flags);
	if (queue->read != queue->write) {
		flag = 1;
		*node = *queue->read;
		queue->read->yaddr = 0;
		queue->read->yaddr_vir = 0;
		queue->read++;
		queue->rcnt++;
		queue->count--;
		if (queue->read > &queue->node[CAMERA_QUEUE_LENGTH - 1])
			queue->read = &queue->node[0];
	}

	if (!flag)
		ret = -EAGAIN;
	spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

static int sprd_img_buf_queue_write(struct camera_img_buf_queue *queue,
				    struct camera_img_buf_addr *buf_addr)
{
	struct camera_img_buf_addr *ori_node;

	if (NULL == queue || NULL == buf_addr) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	ori_node = queue->write;
	*queue->write++ = *buf_addr;
	queue->wcnt++;
	if (queue->write > &queue->buf_addr[DCAM_FRM_CNT_MAX - 1])
		queue->write = &queue->buf_addr[0];

	if (queue->write == queue->read) {
		queue->write = ori_node;
		pr_info("fail to get valid queue : full\n");
	}

	return 0;
}

static int sprd_img_tx_error(struct camera_frame *frame, void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_dev *dev = (struct camera_dev *)param;
	struct camera_node node;

	if (NULL == param || 0 == atomic_read(&dev->stream_on)) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	memset((void *)&node, 0, sizeof(node));
	atomic_set(&dev->run_flag, 1);
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
	if (ret) {
		pr_err("fail to write to queue\n");
		return ret;
	}

	if (dev->dcam_cxt.sensor_id == SPRD_SENSOR_MAIN2_ID_E
		|| dev->dcam_cxt.sensor_id == SPRD_SENSOR_SUB2_ID_E)
		complete(&dev->irq_com_sub);
	else
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

	if (NULL == frame || NULL == param ||
		atomic_read(&dev->stream_on) == 0) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	atomic_set(&dev->run_flag, 1);

	memset((void *)&node, 0, sizeof(node));

	if (frame->irq_type == CAMERA_IRQ_IMG) {
		path = &dev->dcam_cxt.dcam_path[frame->type];

		if (path->status == PATH_IDLE) {
			pr_info("DCAM: path id %d idle\n", frame->type);
			return ret;
		}

		node.irq_flag = IMG_TX_DONE;
		node.irq_type = frame->irq_type;
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
		node.frame_id = frame->frame_id;
		memcpy(node.mfd, frame->pfinfo.mfd, sizeof(unsigned int) * 3);
	} else if (frame->irq_type == CAMERA_IRQ_STATIS) {
		node.irq_flag = IMG_TX_DONE;
		node.irq_type = frame->irq_type;
		node.irq_property = frame->irq_property;
		node.phy_addr = frame->phy_addr;
		node.vir_addr = frame->vir_addr;
		node.kaddr[0] = frame->kaddr[0];
		node.kaddr[1] = frame->kaddr[1];
		node.addr_offset = frame->addr_offset;
		node.buf_size = frame->buf_size;
		node.frame_id = frame->frame_id;
		memcpy(node.mfd, frame->pfinfo.mfd, sizeof(unsigned int) * 3);
	} else if (frame->irq_type == CAMERA_IRQ_DONE) {
		node.irq_flag = IMG_TX_DONE;
		node.irq_type = frame->irq_type;
		node.irq_property = frame->irq_property;
		node.frame_id = frame->frame_id;
	} else if (frame->irq_type == CAMERA_IRQ_3DNR_DONE) {
		node.irq_flag = IMG_TX_DONE;
		node.irq_type = frame->irq_type;
		node.irq_property = frame->irq_property;
	} else if (frame->irq_type == CAMERA_IRQ_POST_YNR_DONE) {
		node.irq_flag = IMG_TX_DONE;
		node.irq_type = frame->irq_type;
		node.irq_property = frame->irq_property;
	} else {
		pr_err("fail to get valid irq_type %d\n", frame->irq_type);
		return -EINVAL;
	}
	node.boot_time = ktime_get_boottime();
	img_get_timestamp(&node.time);

	DCAM_TRACE("flag 0x%x type 0x%x\n",
		   node.irq_flag, node.irq_type);

	DCAM_TRACE("sprd_img %d %d %p\n", dev->idx, frame->type, dev);
	if (node.irq_property == IRQ_RAW_CAP_DONE)
		pr_info("dcam_core: RAW CAP tx done flag %d type %d\n",
			node.irq_flag, node.irq_type);

	if (dev->queue.count > 30 &&
		(node.irq_property == IRQ_DCAM_SOF ||
		node.irq_property == IRQ_SHADOW_DONE)) {
		ret = -EINVAL;
		goto exit;
	}

	ret = sprd_img_queue_write(&dev->queue, &node);
	if (ret) {
		pr_err("fail to write to queue\n");
		goto exit;
	}

	if (dev->dcam_cxt.sensor_id == SPRD_SENSOR_MAIN2_ID_E
		|| dev->dcam_cxt.sensor_id == SPRD_SENSOR_SUB2_ID_E)
		complete(&dev->irq_com_sub);
	else
		complete(&dev->irq_com);

exit:
	return ret;
}

static int sprd_img_tx_stop(void *param, uint32_t sensor_id)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_dev *dev = (struct camera_dev *)param;
	struct camera_node node;

	memset((void *)&node, 0, sizeof(node));
	node.irq_flag = IMG_TX_STOP;
	ret = sprd_img_queue_write(&dev->queue, &node);
	if (ret) {
		pr_err("fail to write to queue\n");
		return ret;
	}

	if (sensor_id == SPRD_SENSOR_MAIN2_ID_E
	    || sensor_id == SPRD_SENSOR_SUB2_ID_E)
		complete(&dev->irq_com_sub);
	else
		complete(&dev->irq_com);

	pr_info("tx stop %d %d\n", dev->irq_com.done, dev->irq_com_sub.done);

	return ret;
}

static void sprd_timer_callback(unsigned long data)
{
	struct camera_dev *dev = (struct camera_dev *)data;
	struct camera_node node;
	int ret = 0;

	memset((void *)&node, 0, sizeof(node));
	if (data == 0 || atomic_read(&dev->stream_on) == 0) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	if (atomic_read(&dev->run_flag) == 0) {
		pr_info("DCAM timeout.\n");
		node.irq_flag = IMG_TIMEOUT;
		node.irq_type = CAMERA_IRQ_IMG;
		node.irq_property = IRQ_MAX_DONE;
		node.invalid_flag = 0;
		ret = sprd_img_queue_write(&dev->queue, &node);
		if (ret)
			pr_err("fail to write queue error\n");

		if (dev->dcam_cxt.sensor_id == SPRD_SENSOR_MAIN2_ID_E
		|| dev->dcam_cxt.sensor_id == SPRD_SENSOR_SUB2_ID_E)
			complete(&dev->irq_com_sub);
		else
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
		pr_err("fail to start in mod_timer %d\n", ret);

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
		pr_err("fail to get valid input ptr\n");
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
			pr_err("fail to init path %d\n", i);
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
	dcam_stop_flash_thread(dev);
	sprd_stop_timer(&dev->dcam_timer);
	atomic_dec(&dev->users);
	mutex_unlock(&dev->dcam_mutex);
	mutex_destroy(&dev->dcam_mutex);

	vfree(dev);
	group->dev[idx] = NULL;

	pr_info("dev[%d] deinit OK!\n", idx);
}

static int sprd_camera_dev_init(struct camera_group *group, enum dcam_id idx)
{
	int ret = 0;
	struct camera_dev *dev = NULL;

	if (!group) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (group->dev_inited & (1 << (int)idx)) {
		pr_info("sprd_img: dev%d already inited\n", idx);
		return 0;
	}

	pr_info("sprd_img: camera dev init start!\n");
	dev = vzalloc(sizeof(*dev));
	if (!dev) {
		ret = -ENOMEM;
		goto vzalloc_exit;
	}
	atomic_set(&dev->run_flag, 1);

	dev->idx = idx;
	mutex_init(&dev->dcam_mutex);
	init_completion(&dev->irq_com);
	init_completion(&dev->irq_com_sub);
	spin_lock_init(&dev->queue.lock);

	if (unlikely(atomic_inc_return(&dev->users) > 1)) {
		vfree(dev);
		return -EBUSY;
	}

	ret = sprd_img_queue_init(&dev->queue);
	if (unlikely(ret != 0)) {
		pr_err("fail to init queue\n");
		ret = -EINVAL;
		goto users_exit;
	}

	ret = sprd_init_handle(dev);
	if (unlikely(ret != 0)) {
		pr_err("fail to init queue\n");
		ret = -EINVAL;
		goto users_exit;
	}

	sprd_init_timer(&dev->dcam_timer, (unsigned long)dev);

	ret = dcam_create_flash_thread(dev);
	if (unlikely(ret != 0)) {
		pr_err("fail to create flash thread\n");
		ret = -EINVAL;
		goto flash_exit;
	}

	ret = sprd_isp_dev_init(&dev->isp_dev_handle, idx);
	if (unlikely(ret != 0)) {
		pr_err("fail to init ISP_ID_%d dev\n", idx);
		ret = -EINVAL;
		goto isp_exit;
	}

	group->dev[idx] = dev;
	group->dev_inited |= 1 << idx;
	group->mode_inited = 0;

	pr_info("dev[%d] init OK %p!\n", idx, dev);
	return ret;

vzalloc_exit:
	pr_err("fail to alloc camera dev\n");
	return ret;

isp_exit:
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
		pr_info("real sensor:dcam0\n");
		res->flag = DCAM_RES_DCAM0_CAP |
			DCAM_RES_DCAM0_PATH;
		group->dcam_res_used |=
			DCAM_RES_DCAM0_CAP |
			DCAM_RES_DCAM0_PATH;
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
			pr_err("fail to get valid dcam for front sensor!\n");
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
		pr_info("put dcam0 for rear sensor\n");
		group->dcam_res_used &= ~(DCAM_RES_DCAM0_CAP |
				DCAM_RES_DCAM0_PATH);
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
				pr_err("fail to put dcam1 for front sensor!\n");
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
	struct camera_group *grp = NULL;
	struct miscdevice *md = file->private_data;
	unsigned int count = 0;

	grp = md->this_device->platform_data;

	if (atomic_read(&grp->camera_opened) == 0) {
		camerafile = vzalloc(sizeof(struct camera_file));
		if (camerafile == NULL) {
			ret = -ENOMEM;
			goto vzalloc_exit;
		}

		grp->camerafile = (void *)camerafile;
		camerafile->grp = grp;
	} else {
		camerafile = (struct camera_file *)grp->camerafile;
	}
	file->private_data = (void *)camerafile;

	if (atomic_inc_return(&grp->camera_opened) > 1) {
		pr_info("sprd_img: the camera has been inited %d\n",
			atomic_read(&grp->camera_opened));
		return 0;
	}

	count = camerafile->grp->dcam_count;

	if (count == 0 || count > DCAM_ID_MAX) {
		pr_err("fail to get valid dts tree config count\n");
		vfree(camerafile);
		return -ENODEV;
	}

	ret = sprd_camera_dev_init(camerafile->grp, DCAM_ID_0);
	if (unlikely(ret != 0)) {
		pr_err("fail to init camera 0\n");
		atomic_dec_return(&camerafile->grp->camera_opened);
		vfree(camerafile);
		return -EINVAL;
	}

	if (count == DCAM_ID_MAX)
		ret = sprd_camera_dev_init(camerafile->grp, DCAM_ID_1);

	if (unlikely(ret != 0)) {
		pr_err("fail to init camera 1\n");
		sprd_camera_dev_deinit(camerafile->grp, DCAM_ID_0);
		atomic_dec_return(&camerafile->grp->camera_opened);
		vfree(camerafile);
		return -EINVAL;
	}

	pr_info("sprd_img: open end!\n");

	return ret;

vzalloc_exit:
	pr_err("fail to alloc memory for camerafile\n");

	return ret;
}

static int sprd_img_k_release(struct inode *node, struct file *file)
{
	int i = 0;
	struct camera_file *camerafile = NULL;
	struct camera_group *group = NULL;

	camerafile = file->private_data;
	if (!camerafile) {
		pr_err("fail to get valid camerafile\n");
		goto exit;
	}
	group = camerafile->grp;

	pr_info("sprd_img: release start.\n");
	if (atomic_dec_return(&group->camera_opened) == 0) {
		for (i = 0; i < group->dcam_count; i++) {
			if (!(group->dev_inited & (1 << i))) {
				pr_info("sprd_img: dev%d already deinited\n",
					i);
				return -1;
			}
			if (group->dev_inited & (1 << 0)) {
				complete(&((struct camera_dev *)
					   group->dev[0])->irq_com);
				complete(&((struct camera_dev *)
					   group->dev[0])->irq_com_sub);
			}
			sprd_camera_stream_off(file);
			sprd_isp_module_dis(group->dev[i]->isp_dev_handle, i);
			sprd_dcam_module_dis(i);
			/*For multi camera*/
			sprd_isp_module_dis(group->dev[i]->isp_dev_handle, i);
			sprd_dcam_module_dis(i);
			sprd_camera_dev_deinit(group, i);

			group->dcam_res_used &= ~(DCAM_RES_DCAM0_CAP |
						  DCAM_RES_DCAM0_PATH);
			group->dcam_res_used &= ~(DCAM_RES_DCAM1_CAP |
						  DCAM_RES_DCAM1_PATH);

			group->dev_inited &= ~(1<<i);
			group->mode_inited = 0;
		}
		vfree(camerafile);
		file->private_data = NULL;
	}

	pr_info("sprd_img: release end!\n");
exit:
	return 0;
}

static int sprd_dcam_cfg_block(struct camera_info *info, enum dcam_id idx)
{
	int ret = DCAM_RTN_SUCCESS;
	unsigned int param = 0;

	if (info == NULL) {
		pr_err("fail to get valid input ptr\n");
		ret = -EINVAL;
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_SBS_MODE, &info->sbs_info);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_INTERFACE, &info->if_mode);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap interface\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_SENSOR_MODE, &info->sn_mode);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap sensor mode\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_SYNC_POL, &info->sync_pol);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap sync pol\n");
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
		pr_err("fail to cfg dcam cap to isp\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_DATA_BITS, &info->data_bits);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap data bits\n");
		goto exit;
	}

	if (info->sn_mode == DCAM_CAP_MODE_RAWRGB &&
	    info->if_mode == DCAM_CAP_IF_CSI2) {
		ret = set_dcam_cap_cfg(idx, DCAM_CAP_DATA_PACKET,
					&info->is_loose);
		if (unlikely(ret)) {
			pr_err("fail to cfg dcam cap data packet\n");
			goto exit;
		}
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_FRM_DECI, &info->frm_deci);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap frame deci\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_INPUT_RECT, &info->cap_in_rect);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap input rect\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_INPUT_SIZE, &info->cap_in_size);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap frame count clear\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_FRM_COUNT_CLR, NULL);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap pre skip count\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_PRE_SKIP_CNT, &info->skip_number);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap sample mode\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_SAMPLE_MODE, &info->capture_mode);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap image xy deci\n");
		goto exit;
	}

	ret = set_dcam_cap_cfg(idx, DCAM_CAP_IMAGE_XY_DECI, &info->img_deci);
	if (unlikely(ret)) {
		pr_err("fail to cfg dcam cap sbs mode\n");
		goto exit;
	}

exit:
	return ret;
}

static int sprd_dcam_block_reg_isr(struct camera_dev *param)
{
	sprd_dcam_reg_isr(param->idx, DCAM_PATH0_AXIM_DONE,
		sprd_img_tx_done, param);
	sprd_dcam_reg_isr(param->idx, DCAM_PDAF_AXIM_DONE,
		sprd_img_tx_done, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_FRM_ERR,
		sprd_img_tx_error, param);
	sprd_dcam_reg_isr(param->idx, DCAM_ISP_FIFO_OVF,
		sprd_img_tx_error, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_LINE_ERR,
		sprd_img_tx_error, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_MIPI_OVF,
		sprd_img_tx_error, param);
	sprd_dcam_reg_isr(param->idx, DCAM_SN_EOF,
		sprd_img_start_flash, param);

	return 0;
}

static int sprd_dcam_block_unreg_isr(struct camera_dev *param)
{
	sprd_dcam_reg_isr(param->idx, DCAM_PATH0_AXIM_DONE, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_PDAF_AXIM_DONE, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_FRM_ERR, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_ISP_FIFO_OVF, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_LINE_ERR, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_CAP_MIPI_OVF, NULL, param);
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
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

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
		pr_err("fail to init dcam module\n");
		goto exit;
	}

	ret = sprd_dcam_block_reg_isr(dev);
	if (unlikely(ret)) {
		pr_err("fail to register dcam isr\n");
		goto exit;
	}

	/* config cap sub-module */
	ret = sprd_dcam_cfg_block(info, idx);
	if (unlikely(ret)) {
		pr_err("fail to config cap");
		goto exit;
	}

	/* config dcam_if path0 */
	if (path0->is_work) {
		ret = sprd_dcam_cfg_path0(path0, idx);
		if (unlikely(ret)) {
			pr_err("fail to config path0 cap");
			goto exit;
		}
		path0->status = PATH_RUN;
	}

	/* config dcam_if path pdaf*/
	if (path_pdaf->is_work) {
		if (info->sn_fmt == IMG_PIX_FMT_YUV422P) {
			path_pdaf->pdaf_ctrl.mode = 1;
			path_pdaf->pdaf_ctrl.phase_data_type = 0x1E;
		}
		ret = sprd_dcam_cfg_path_pdaf(path_pdaf, idx);
		if (unlikely(ret)) {
			pr_err("fail to config path pdaf cap");
			goto exit;
		}
		path_pdaf->status = PATH_RUN;
	}

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
		pr_err("fail to get valid input ptr\n");
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
	}

	pr_debug("prv path_mode: %d, cap path_mode %d, vid path mode %d\n",
	       path_pre->path_mode, path_cap->path_mode, path_vid->path_mode);
exit:
	return ret;
}

static int sprd_img_isp_reg_isr(struct camera_dev *dev)
{
	enum isp_id idx = ISP_ID_0;

	if (!dev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (dev->idx == DCAM_ID_0)
		idx = ISP_ID_0;
	else if (dev->idx == DCAM_ID_1)
		idx = ISP_ID_1;
	else {
		pr_err("fail to get valid dev idx %d\n", dev->idx);
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
	sprd_isp_reg_isr(idx, ISP_POST_YNR_DONE,
		sprd_img_tx_done, dev);


	return 0;
}

static int sprd_img_isp_unreg_isr(struct camera_dev *dev)
{
	enum isp_id idx = ISP_ID_0;

	if (!dev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (dev->idx == DCAM_ID_0)
		idx = ISP_ID_0;
	else if (dev->idx == DCAM_ID_1)
		idx = ISP_ID_1;
	else {
		pr_err("fail to get valid dev idx %d\n", dev->idx);
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
	sprd_isp_reg_isr(idx, ISP_POST_YNR_DONE,
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
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	info = &dev->dcam_cxt;
	idx = dev->idx;
	path_pre = &info->dcam_path[CAMERA_PRE_PATH];
	path_vid = &info->dcam_path[CAMERA_VID_PATH];
	path_cap = &info->dcam_path[CAMERA_CAP_PATH];
	memset((void *)&path_info, 0, sizeof(path_info));
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
		pr_err("fail to config isp path mode\n");
		goto exit;
	}

	ret = sprd_img_isp_reg_isr(dev);
	if (unlikely(ret)) {
		pr_err("fail to register isp isr\n");
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
				pr_err("fail to config path_pre\n");
				break;
			}
			path_pre->status = PATH_RUN;
		} else {
			ret = set_isp_path_cfg(dev->isp_dev_handle,
				ISP_PATH_IDX_PRE, ISP_PATH_ENABLE,
		&path_not_work);
			if (unlikely(ret)) {
				pr_err("fail to config isp path pre\n");
				break;
			}
		}

		/* config isp vid path*/
		if (dev->use_path && path_vid->is_work) {
			ret = sprd_isp_path_cfg_block(path_vid,
				dev->isp_dev_handle, ISP_PATH_IDX_VID);
			if (unlikely(ret)) {
				pr_err("fail to config path_vid\n");
				break;
			}
			ret = isp_slw_flags_init(dev->isp_dev_handle,
						&path_info);
			if (unlikely(ret)) {
				pr_err("fail to config slow motion\n");
				break;
			}
			path_vid->status = PATH_RUN;
		} else {
			ret = set_isp_path_cfg(dev->isp_dev_handle,
				ISP_PATH_IDX_VID, ISP_PATH_ENABLE,
		&path_not_work);
			if (unlikely(ret)) {
				pr_err("fail to config isp path pre\n");
				break;
			}
		}

		/* config isp cap path*/
		if (dev->use_path && path_cap->is_work) {
			ret = sprd_isp_path_cfg_block(path_cap,
				dev->isp_dev_handle, ISP_PATH_IDX_CAP);
			if (unlikely(ret)) {
				pr_err("fail to config path_cap");
				break;
			}
			path_cap->status = PATH_RUN;
		} else {
			ret = set_isp_path_cfg(dev->isp_dev_handle,
				ISP_PATH_IDX_CAP, ISP_PATH_ENABLE,
		&path_not_work);
			if (unlikely(ret)) {
				pr_err("fail to config isp path pre\n");
				break;
			}
		}

	} while (0);

	pr_debug("end isp path cfg\n");
exit:
	return ret;
}

static int sprd_camera_update_clk(struct camera_dev *dev,
	struct device_node *dn)
{
	int ret = 0;
	unsigned int dcam_clk_index = DCAM_CLK_307M2_INDEX;
	unsigned int isp_clk_index = ISP_CLK_307M2_INDEX;
	unsigned int cap_in_size_width = 0;
	unsigned int cap_in_size_height = 0;
	unsigned int cap_out_size_width = 0;
	unsigned int cap_out_size_height = 0;
	unsigned int sn_width_max = 0;
	unsigned int sn_height_max = 0;
	struct camera_info *info = NULL;
	struct camera_path_spec *path_cap = NULL;
	unsigned int is_optimization_flag = 1;

	if (!dev || !dn) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	info = &dev->dcam_cxt;
	if (info == NULL) {
		ret = -EFAULT;
		pr_err("fail to get valid dcam_cxt:NULL\n");
		goto exit;
	}
	path_cap = &info->dcam_path[CAMERA_CAP_PATH];
	sn_width_max = info->sn_max_size.w;
	sn_height_max = info->sn_max_size.h;
	cap_in_size_width = info->cap_in_size.w;
	cap_in_size_height = info->cap_in_size.h;
	cap_out_size_width = info->cap_out_size.w;
	cap_out_size_height = info->cap_out_size.h;

	if (info->bps_per_lane == 0xffff) {
		pr_info("dcam and isp not support power optimization\n");
		goto exit;
	}

	if (cap_in_size_width <= CAP_IN_SIZE_VGA_WIDTH)
		dcam_clk_index = DCAM_CLK_128M_INDEX;
	else if (cap_in_size_width <= CAP_IN_SIZE_8MBIN_WIDTH)
		dcam_clk_index = DCAM_CLK_256M_INDEX;
	else
		dcam_clk_index = DCAM_CLK_307M2_INDEX;

	if ((sn_width_max > CAP_IN_SIZE_8M_WIDTH && path_cap->is_work) ||
		info->raw_cap_flag ||
		info->slow_motion_flag) {
		if (sn_width_max > CAP_IN_SIZE_13M_WIDTH)
			dcam_clk_index = DCAM_CLK_384M_INDEX;
		else
			dcam_clk_index = DCAM_CLK_307M2_INDEX;
	}

	ret = sprd_dcam_update_clk(dcam_clk_index, dn);
	if (unlikely(ret != 0)) {
		pr_err("fail to update dcam_if clk\n");
		goto exit;
	}

	if (cap_out_size_width <= CAP_IN_SIZE_VGA_WIDTH)
		isp_clk_index = ISP_CLK_128M_INDEX;
	else if (cap_out_size_width <= CAP_IN_SIZE_8MBIN_WIDTH)
		isp_clk_index = ISP_CLK_256M_INDEX;
	else if (cap_out_size_width <= CAP_IN_SIZE_8M_WIDTH)
		isp_clk_index = ISP_CLK_307M2_INDEX;
	else
		isp_clk_index = ISP_CLK_MAX_INDEX;

	if ((sn_width_max > CAP_IN_SIZE_8M_WIDTH && path_cap->is_work) ||
		info->raw_cap_flag ||
		info->slow_motion_flag)
		isp_clk_index = ISP_CLK_MAX_INDEX;

	if (isp_clk_index == ISP_CLK_MAX_INDEX &&
		sn_width_max > CAP_IN_SIZE_13M_WIDTH)
		is_optimization_flag = 0;

	ret = sprd_isp_update_clk(is_optimization_flag,
		isp_clk_index, dn);
	if (unlikely(ret != 0)) {
		pr_err("fail to update isp clk\n");
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

	pr_info("stream on in\n");
	camerafile = (struct camera_file *)file->private_data;
	if (!camerafile) {
		ret = -EFAULT;
		pr_err("fail to get valid camerafile:NULL\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("fail to get valid dev[%d]:NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("fail to get valid dev[%d] hasn't been inited\n",
			idx);
		goto exit;
	}

	mutex_lock(&dev->dcam_mutex);
	ret = sprd_camera_update_clk(dev,
		group->pdev->dev.of_node);
	if (unlikely(ret != 0)) {
		mutex_unlock(&dev->dcam_mutex);
		pr_err("fail to update clk\n");
		goto exit;
	}

	ret = sprd_img_queue_init(&dev->queue);
	if (unlikely(ret != 0)) {
		mutex_unlock(&dev->dcam_mutex);
		pr_err("fail to init queue\n");
		goto exit;
	}

	ret = sprd_dcam_cfg(dev);
	if (unlikely(ret)) {
		mutex_unlock(&dev->dcam_mutex);
		pr_err("fail to config dcam param\n");
		goto exit;
	}

	ret = sprd_isp_path_cfg(dev);
	if (unlikely(ret)) {
		mutex_unlock(&dev->dcam_mutex);
		pr_err("fail to config isp path\n");
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
		mutex_unlock(&dev->dcam_mutex);
		pr_err("fail to start isp path\n");
		goto exit;
	}

	ret = sprd_dcam_start(idx);
	if (unlikely(ret)) {
		pr_err("fail to start dcam\n");
		ret = sprd_img_isp_unreg_isr(dev);
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	} else {
		atomic_set(&dev->run_flag, 0);
		sprd_start_timer(&dev->dcam_timer, DCAM_TIMEOUT);
	}

	atomic_set(&dev->stream_on, 1);
	mutex_unlock(&dev->dcam_mutex);

	pr_info("stream on end\n");
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
	struct camera_path_spec *path_pdaf = NULL;

	camerafile = (struct camera_file *)file->private_data;
	if (!camerafile) {
		ret = -EFAULT;
		pr_err("fail to get valid camerafile:NULL\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("fail to get valid dev[%d]:NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("fail to get valid dev[%d] hasn't been inited\n",
			idx);
		goto exit;
	}

	mutex_lock(&dev->dcam_mutex);

	path_0 = &dev->dcam_cxt.dcam_path[CAMERA_PATH0];
	path_pdaf = &dev->dcam_cxt.dcam_path[CAMERA_PDAF_PATH];

	if (unlikely(atomic_read(&dev->stream_on) == 0)) {
		pr_info("stream not on, idx: %d\n", idx);
		ret = sprd_img_local_deinit(dev);
		if (unlikely(ret))
			pr_err("fail to local deinit\n");

		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}
	do {
		ret = sprd_stop_timer(&dev->dcam_timer);
		if (unlikely(ret)) {
			pr_err("fail to stop timer\n");
			break;
		}
		ret = sprd_dcam_stop(idx, 0);
		if (unlikely(ret)) {
			pr_err("fail to stop dcam\n");
			break;
		}
		ret = sprd_dcam_block_unreg_isr(dev);
		if (unlikely(ret)) {
			pr_err("fail to unregister isr\n");
			break;
		}

		/* check if stop dcam first, then isp, or not? */
		ret = sprd_isp_stop(dev->isp_dev_handle, 0);
		if (unlikely(ret)) {
			pr_err("fail to stop isp\n");
			break;
		}

		ret = sprd_img_isp_unreg_isr(dev);
		if (unlikely(ret)) {
			pr_err("fail to unregister isp isr\n");
			break;
		}

		if (path_0->is_work) {
			path_0->status = PATH_IDLE;
			path_0->is_work = 0;
		}

		if (path_pdaf->is_work) {
			path_pdaf->status = PATH_IDLE;
			path_pdaf->is_work = 0;
		}

		atomic_set(&dev->stream_on, 0);

		ret = sprd_dcam_module_deinit(idx);
		if (unlikely(ret)) {
			pr_err("fail to deinit dcam module\n");
			break;
		}

		ret = sprd_img_local_deinit(dev);
		if (unlikely(ret)) {
			pr_err("fail to local deinit\n");
			break;
		}
	} while (0);

	mutex_unlock(&dev->dcam_mutex);

	pr_info("camera stream off end.\n");
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

	pr_debug("in_size{%d %d}, in_rect{%d %d %d %d}\n",
		   path->in_size.w, path->in_size.h, path->in_rect.x,
		   path->in_rect.y, path->in_rect.w, path->in_rect.h);
	pr_debug("out_size{%d %d}\n",
		   path->out_size.w, path->out_size.h);
	ret = sprd_isp_update_zoom_param(dev->isp_dev_handle,
					 path_index,
					 &path->in_size,
					 &path->in_rect,
					 &path->out_size);

	mutex_unlock(&dev->dcam_mutex);
	DCAM_TRACE("update video 0x%x\n", ret);

	if (ret)
		pr_err("fail to update video 0x%x\n", ret);

	return ret;
}

static int sprd_img_io_check_fmt(struct file *file,
			      unsigned long arg)
{
	int ret = 0;
	unsigned int channel_id;
	struct camera_dev *dev = NULL;
	struct camera_file *camerafile = NULL;
	struct camera_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_format *fmt;
	struct sprd_img_format img_format;

	DCAM_TRACE("check fmt\n");
	ret = copy_from_user(&img_format,
				(void __user *)arg,
				sizeof(struct sprd_img_format));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to get img_format\n");
		goto exit;
	}

	camerafile = (struct camera_file *)file->private_data;
	if (!camerafile) {
		ret = -EFAULT;
		pr_err("fail to get valid camerafile:NULL\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("fail to get valid dev[%d]:NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("fail to get inited dev[%d]\n", idx);
		goto exit;
	}
	dev->use_path = img_format.buffer_cfg_isp;

	fmt = sprd_img_get_format(img_format.fourcc);
	if (unlikely(!fmt)) {
		ret = -EFAULT;
		pr_err("fail to get valid fourcc format (0x%08x)\n",
		       img_format.fourcc);
		goto exit;
	}

	if (img_format.channel_id == CAMERA_PATH0) {
		mutex_lock(&dev->dcam_mutex);
		ret = sprd_img_check_path0_cap(fmt->fourcc,
					       &img_format,
					       &dev->dcam_cxt);
		mutex_unlock(&dev->dcam_mutex);
		channel_id = CAMERA_PATH0;
	} else if (img_format.channel_id == CAMERA_PDAF_PATH) {
		mutex_lock(&dev->dcam_mutex);
		ret = sprd_img_check_path_pdaf_cap(fmt->fourcc,
					       &img_format,
					       &dev->dcam_cxt);
		mutex_unlock(&dev->dcam_mutex);
		channel_id = CAMERA_PDAF_PATH;
	} else {
		mutex_lock(&dev->dcam_mutex);
		ret = sprd_img_check_path_cap(fmt->fourcc, &img_format,
			&dev->dcam_cxt, img_format.channel_id);
		mutex_unlock(&dev->dcam_mutex);
		channel_id = img_format.channel_id;
	}

	if (channel_id < CAMERA_PDAF_PATH) {
		img_format.endian.y_endian =
			dev->dcam_cxt.dcam_path[channel_id].end_sel.y_endian;
		img_format.endian.uv_endian =
			dev->dcam_cxt.dcam_path[channel_id].end_sel.uv_endian;
	}

	if ((ret == 0) && (atomic_read(&dev->stream_on) != 0)) {
		if (channel_id == CAMERA_PRE_PATH
			|| channel_id == CAMERA_VID_PATH
			|| channel_id == CAMERA_CAP_PATH) {
			ret = sprd_img_update_video(dev, channel_id);
		}
	}

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

static int sprd_img_set_crop(struct file *file, struct sprd_img_parm *p)
{
	int ret = 0;
	struct camera_dev *dev = NULL;
	struct camera_file *camerafile = NULL;
	struct camera_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct camera_rect *input_rect = NULL;
	struct camera_size *input_size = NULL;

	camerafile = (struct camera_file *)file->private_data;
	if (!camerafile) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	if (!p) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("fail to get inited dev[%d] : NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("fail to get inited dev[%d]\n", idx);
		goto exit;
	}

	if (p->crop_rect.x + p->crop_rect.w > dev->dcam_cxt.cap_in_size.w ||
	    p->crop_rect.y + p->crop_rect.h > dev->dcam_cxt.cap_in_size.h) {
		ret = -EINVAL;
		goto exit;
	}

	DCAM_TRACE("dcam%d: set crop, window %d %d %d %d\n", idx,
		   p->crop_rect.x, p->crop_rect.y,
		   p->crop_rect.w, p->crop_rect.h);

	switch (p->channel_id) {
	case CAMERA_PATH0:
	case CAMERA_PRE_PATH:
	case CAMERA_VID_PATH:
	case CAMERA_CAP_PATH:
	case CAMERA_PDAF_PATH:
		input_size = &dev->dcam_cxt.dcam_path[p->channel_id].in_size;
		input_rect = &dev->dcam_cxt.dcam_path[p->channel_id].in_rect;
		break;
	default:
		pr_info("dcam%d: fail to get right channel id %d\n",
			idx, p->channel_id);
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
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	if (!sensor_if) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("fail to get valid dev[%d] : NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("fail to get inited dev[%d]\n", idx);
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

	DCAM_TRACE("dcam%d: if %d mode %d deci %d\n",
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
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	if (!p) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("fail to get valid dev[%d] : NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("fail to get inited dev[%d]\n", idx);
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
		pr_info("fail to get valid channel %d\n", p->channel_id);
		return -EINVAL;
	}

	DCAM_TRACE("set path%d frame addr,status %d cnt %d reserved_buf %d\n",
		   p->channel_id, path->status, path->frm_cnt_act,
		   p->is_reserved_buf);

	if (unlikely(p->fd_array[0] == 0)) {
		pr_info("fail to get fd\n");
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
				for (i = 0; i < p->buffer_count; i++) {
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
				}
				ret = set_dcam_path0_cfg(idx,
					DCAM_PATH0_OUTPUT_ADDR, &frame_addr);
				if (unlikely(ret)) {
					pr_err("fail to cfg path0 output addr\n");
					goto exit;
				}
			} else if (p->channel_id == CAMERA_PDAF_PATH) {
				for (i = 0; i < p->buffer_count; i++) {
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
					ret = set_dcam_path_pdaf_cfg(idx,
						DCAM_PATH_PDAF_OUTPUT_ADDR,
						&frame_addr);
					if (unlikely(ret)) {
						pr_err("fail to cfg path0 output addr\n");
						goto exit;
					}
				}
			} else {
				path_index =
					sprd_img_get_path_index(p->channel_id);
				for (i = 0; i < p->buffer_count; i++) {
					if (p->fd_array[0] == 0) {
						pr_info("fail to get valid fd\n");
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
						pr_err("fail to cfg path0 output addr\n");
						goto exit;
					}
				}
			}
		} else {

			for (i = 0; i < p->buffer_count; i++) {
				if (unlikely(p->fd_array[0] == 0)) {
					pr_info("fail to get valid fd\n");
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

	DCAM_TRACE("%d: success to set frame addr\n", idx);

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
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	if (!channel_id) {
		ret = -EFAULT;
		pr_err("fail to get valid param:channel id\n");
		goto exit;
	}

	group = camerafile->grp;
	idx = camerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		dev = group->dev[idx];
		if (!dev) {
			ret = -EFAULT;
			pr_err("fail to get valid dev[%d]:NULL\n", idx);
			goto exit;
		}
	} else {
		ret = -EFAULT;
		pr_err("fail to get inited dev[%d]\n", idx);
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
	path_id.sn_fmt = dev->dcam_cxt.sn_fmt;
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
	ret = sprd_camera_get_path_id(&path_id, channel_id, scene_mode);
	DCAM_TRACE("get channel %d\n", *channel_id);

exit:
	return ret;
}

static int sprd_img_io_get_fmt(struct file *file, unsigned long arg)
{
	int ret = 0;
	struct camera_format *fmt = NULL;
	struct sprd_img_get_fmt fmt_desc;

	DCAM_TRACE("get fmt\n");
	ret = copy_from_user(&fmt_desc,
				(void __user *)arg,
				sizeof(struct sprd_img_get_fmt));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy from user\n");
		goto exit;
	}
	if (unlikely(fmt_desc.index >= ARRAY_SIZE(dcam_img_fmt)))
		return -EINVAL;

	fmt = &dcam_img_fmt[fmt_desc.index];
	fmt_desc.fmt = fmt->fourcc;

	ret = copy_to_user((void __user *)arg,
			&fmt_desc,
			sizeof(struct sprd_img_get_fmt));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy to user\n");
		goto exit;
	}
exit:
	return ret;
}

static int sprd_img_io_set_sensor_max_size(struct camera_dev *dev,
	struct camera_info *info, enum dcam_id idx, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_size size;

	mutex_lock(&dev->dcam_mutex);
	ret = copy_from_user(&size, (void __user *)arg,
				sizeof(struct sprd_img_size));
	if (ret || !size.w || !size.h) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->dcam_mutex);
		return ret;
	}
	info->sn_max_size.w = size.w;
	info->sn_max_size.h = size.h;
	mutex_unlock(&dev->dcam_mutex);
	DCAM_TRACE("%d: sensor max size %d %d\n", idx,
			info->sn_max_size.w, info->sn_max_size.h);
	return ret;
}

static int sprd_img_io_set_sensor_size(struct camera_dev *dev,
	struct camera_info *info, enum dcam_id idx, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_size size;

	mutex_lock(&dev->dcam_mutex);
	ret = copy_from_user(&size, (void __user *)arg,
				sizeof(struct sprd_img_size));
	if (ret) {
		pr_err("fail to get user info %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->dcam_mutex);
		return ret;
	}
	info->cap_in_size.w = size.w;
	info->cap_in_size.h = size.h;
	mutex_unlock(&dev->dcam_mutex);
	DCAM_TRACE("%d: sensor size %d %d\n", idx,
			info->cap_in_size.w, info->cap_in_size.h);
	return ret;
}

static int sprd_img_io_set_sensor_trim(struct camera_dev *dev,
	struct camera_info *info, enum dcam_id idx, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_rect rect;

	mutex_lock(&dev->dcam_mutex);
	ret = copy_from_user(&rect, (void __user *)arg,
				sizeof(struct sprd_img_rect));
	if (ret) {
		pr_err("fail to get user info %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->dcam_mutex);
		return ret;
	}
	info->cap_in_rect.x = rect.x;
	info->cap_in_rect.y = rect.y;
	info->cap_in_rect.w = rect.w;
	info->cap_in_rect.h = rect.h;

	info->cap_out_size.w = info->cap_in_rect.w;
	info->cap_out_size.h = info->cap_in_rect.h;
	mutex_unlock(&dev->dcam_mutex);
	DCAM_TRACE("%d: sensor trim x y w h %d %d %d %d\n", idx,
			info->cap_in_rect.x,
			info->cap_in_rect.y,
			info->cap_in_rect.w,
			info->cap_in_rect.h);
	return ret;
}

static int sprd_img_io_get_time(unsigned long arg)
{
	int ret = 0;
	struct sprd_img_time utime;
	struct timeval time;

	DCAM_TRACE("get time\n");
	img_get_timestamp(&time);
	utime.sec = time.tv_sec;
	utime.usec = time.tv_usec;
	ret = copy_to_user((void __user *)arg, &utime,
				sizeof(struct sprd_img_time));
	if (ret) {
		pr_err("fail to get time info %d\n", ret);
		ret = -EFAULT;
	}
	return ret;
}

static int sprd_img_io_set_sensor_if(struct file *file,
	struct camera_dev *dev, enum dcam_id idx, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_sensor_if sensor;

	DCAM_TRACE("%d: set sensor if\n", idx);
	mutex_lock(&dev->dcam_mutex);
	ret = copy_from_user(&sensor,
				(void __user *)arg,
				sizeof(struct sprd_img_sensor_if));
	if (unlikely(ret)) {
		pr_err("fail to get user info %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->dcam_mutex);
		return ret;
	}

	ret = sprd_img_set_sensor_if(file, &sensor);
	mutex_unlock(&dev->dcam_mutex);
	return ret;
}

static int sprd_img_io_cfg_flash(struct camera_dev *dev,
	unsigned long arg)
{
	int ret = 0;
	struct sprd_flash_cfg_param cfg_parm;

	mutex_lock(&dev->dcam_mutex);
	ret = copy_from_user(&cfg_parm,
				(void __user *)arg,
				sizeof(struct sprd_flash_cfg_param));
	if (ret) {
		pr_err("fail to get user info %d\n", ret);
		mutex_unlock(&dev->dcam_mutex);
		ret = -EFAULT;
		return ret;
	}
	ret = sprd_flash_cfg(&cfg_parm);
	mutex_unlock(&dev->dcam_mutex);
	DCAM_TRACE("config flash, ret %d\n", ret);
	return ret;
}

static int sprd_img_io_set_path_skip_num(struct camera_dev *dev,
	struct camera_info *info, enum dcam_id idx, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;

	mutex_lock(&dev->dcam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}

	switch (parm.channel_id) {
	case CAMERA_PATH0:
	case CAMERA_PRE_PATH:
	case CAMERA_VID_PATH:
	case CAMERA_CAP_PATH:
	case CAMERA_PDAF_PATH:
		info->dcam_path[parm.channel_id].skip_num =
			parm.skip_num;
		break;
	default:
		pr_info("fail to get valid channel ID, %d\n",
			parm.channel_id);
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}
	mutex_unlock(&dev->dcam_mutex);
	DCAM_TRACE("%d: channel %d, skip_num %d\n",
			idx, parm.channel_id, parm.skip_num);
exit:
	return ret;
}

static int sprd_img_io_set_frm_id_base(struct camera_dev *dev,
	struct camera_info *info, enum dcam_id idx, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;

	mutex_lock(&dev->dcam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}

	switch (parm.channel_id) {
	case CAMERA_PATH0:
	case CAMERA_PRE_PATH:
	case CAMERA_VID_PATH:
	case CAMERA_CAP_PATH:
	case CAMERA_PDAF_PATH:
		info->dcam_path[parm.channel_id].frm_id_base =
			parm.frame_base_id;
		break;
	default:
		pr_info("fail to get valid channel ID, %d\n",
			parm.channel_id);
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}
	mutex_unlock(&dev->dcam_mutex);
	DCAM_TRACE("%d: channel %d, base id 0x%x\n",
			idx, parm.channel_id, parm.frame_base_id);
exit:
	return ret;
}

static int sprd_img_io_set_crop(struct file *file,
	struct camera_dev *dev, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;

	mutex_lock(&dev->dcam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				 sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}

	ret = sprd_img_set_crop(file, &parm);
	mutex_unlock(&dev->dcam_mutex);
exit:
	return ret;
}

static int sprd_img_io_set_output_size(struct camera_dev *dev,
	struct camera_info *info, enum dcam_id idx, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;

	DCAM_TRACE("%d: set output size\n", idx);
	mutex_lock(&dev->dcam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				 sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}
	info->dst_size.w = parm.dst_size.w;
	info->dst_size.h = parm.dst_size.h;
	info->pxl_fmt = parm.pixel_fmt;
	info->sn_fmt = parm.sn_fmt;
	info->sensor_id = parm.sensor_id;
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
exit:
	return ret;
}

static int sprd_img_io_set_frame_addr(struct file *file,
	struct camera_dev *dev, enum dcam_id idx, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;

	DCAM_TRACE("%d: set frame addr\n", idx);
	mutex_lock(&dev->dcam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}

	ret = sprd_img_set_frame_addr(file, &parm);
	mutex_unlock(&dev->dcam_mutex);
exit:
	return ret;
}

static int sprd_img_io_path_frm_deci(struct camera_dev *dev,
	struct camera_info *info, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;
	struct camera_path_spec *path = NULL;

	mutex_lock(&dev->dcam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}
	path = &info->dcam_path[parm.channel_id];
	path->path_frm_deci = parm.deci;
	mutex_unlock(&dev->dcam_mutex);
exit:
	return ret;
}

static int sprd_img_io_set_shrink(struct camera_dev *dev,
	struct camera_info *info, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;
	struct camera_path_spec *path = NULL;

	mutex_lock(&dev->dcam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}
	path = &info->dcam_path[parm.channel_id];
	path->regular_desc = parm.regular_desc;
	mutex_unlock(&dev->dcam_mutex);
	DCAM_TRACE("channel %d, regular mode %d\n",
		parm.channel_id, path->regular_desc.regular_mode);
exit:
	return ret;
}

static int sprd_img_io_pdaf_control(struct camera_dev *dev,
	struct camera_info *info, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_parm parm;
	struct camera_path_spec *path = NULL;

	mutex_lock(&dev->dcam_mutex);
	ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		ret = -EFAULT;
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
exit:
	return ret;
}

static int sprd_img_io_get_dcam_res(struct camera_file *camerafile,
	struct camera_group *group, struct camera_dev *dev,
	enum dcam_id idx, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_res res = {0};

	ret = copy_from_user(&res, (void __user *)arg,
				sizeof(struct sprd_img_res));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy from user!\n");
		goto exit;
	}
	ret = sprd_img_get_res(group, &res);
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to get res!\n");
		goto exit;
	}

	idx = sprd_sensor_find_dcam_id(res.sensor_id);
	if (idx == -1) {
		ret = -EFAULT;
		pr_err("fail to find attach dcam id!\n");
		goto exit;
	}
	if (group->mode_inited & (1 << (int)idx)) {
		pr_info("dcam%d has been enabled!\n", idx);
		/*break;*/
	}
	ret = sprd_dcam_module_en(idx);
	if (unlikely(ret != 0)) {
		pr_err("fail to enable dcam module %d\n", idx);
		goto exit;
	}
	ret = sprd_isp_module_en(group->dev[idx]->isp_dev_handle,
		(enum isp_id)idx);
	if (unlikely(ret != 0)) {
		pr_err("fail to enable isp module %d\n", idx);
		ret = -EFAULT;
		goto exit;
	}
	camerafile->idx = idx;
	group->mode_inited |= 1 << idx;
	pr_info("dcam dev[%d] init OK: 0x%lx\n", idx,
			(unsigned long)group->dev[idx]);
	pr_info("sprd_img idx %d inited %d res_used %d\n",
			idx, group->dev_inited, group->dcam_res_used);

	ret = copy_to_user((void __user *)arg, &res,
				sizeof(struct sprd_img_res));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy_to_user\n");
		goto exit;
	}
exit:
	return ret;
}

static int sprd_img_io_put_dcam_res(struct camera_group *group,
	struct file *file, struct camera_dev *dev,
	enum dcam_id idx, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_res res = {0};

	ret = copy_from_user(&res, (void __user *)arg,
				sizeof(struct sprd_img_res));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy_from_user!\n");
		goto exit;
	}

	idx = sprd_sensor_find_dcam_id(res.sensor_id);
	pr_info("dev_inited:%d, idx: %d\n", group->dev_inited, idx);
	if (idx == -1) {
		ret = -EFAULT;
		pr_err("fail to find attach dcam id!\n");
		goto exit;
	}
	if (!(group->mode_inited & (1 << (int)idx))) {
		pr_err("dcam%d has been already disabled!\n", idx);
		/*break;*/
	}

	sprd_camera_stream_off(file);

	ret = sprd_dcam_module_dis(idx);
	if (unlikely(ret != 0)) {
		pr_err("fail to disable dcam%d module\n", idx);
		ret = -EFAULT;
	}
	ret = sprd_isp_module_dis(group->dev[idx]->isp_dev_handle,
		(enum isp_id)idx);
	if (unlikely(ret != 0)) {
		pr_err("SPRD_IMG%d: fail to disable isp module\n",
			idx);
		ret = -EFAULT;
	}
	group->mode_inited &= ~(1<<idx);

	ret = sprd_img_put_res(group, &res);
	if (ret) {
		pr_err("fail to put res %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	ret = copy_to_user((void __user *)arg, &res,
			sizeof(struct sprd_img_res));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy_to_user!\n");
		goto exit;
	}
exit:
	return ret;
}

static int sprd_img_get_dcam_dev(struct camera_file *pcamerafile,
	struct camera_dev **ppdev, struct camera_info **ppinfo)
{
	int ret = 0;
	int idx = 0;
	struct camera_group *group = NULL;

	if (!pcamerafile) {
		ret = -EFAULT;
		pr_err("fail to get camerafile\n");
		goto exit;
	}
	group = pcamerafile->grp;
	if (!group) {
		ret = -EFAULT;
		pr_err("fail to get group\n");
		goto exit;
	}

	idx = pcamerafile->idx;
	if (group->dev_inited & (1 << (int)idx)) {
		*ppdev = group->dev[idx];
		if (!(*ppdev)) {
			ret = -EFAULT;
			pr_err("fail to get dcam dev[%d]\n", idx);
			goto exit;
		}
		*ppinfo = &(*ppdev)->dcam_cxt;
	} else {
		ret = -EFAULT;
		pr_err("fail to get dcam dev[%d] and info\n", idx);
		goto exit;
	}

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
	struct camera_dev *dev = NULL;
	struct camera_info *info = NULL;
	struct camera_file *camerafile = NULL;
	struct camera_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	unsigned int cap_flag = 0;
	struct sprd_img_sbs_info sbs_info;
	struct sprd_img_capture_param capture_param;

	camerafile = (struct camera_file *)file->private_data;
	if (!camerafile) {
		ret = -EFAULT;
		pr_err("fail to get valid input ptr\n");
		goto exit;
	}

	idx = camerafile->idx;
	group = camerafile->grp;

	DCAM_TRACE("dcam ioctl: %d, cmd: 0x%x, %d\n",
			   idx, cmd, _IOC_NR(cmd));
	switch (cmd) {
	case SPRD_IMG_IO_GET_DCAM_RES:
		ret = sprd_img_io_get_dcam_res(camerafile,
			group, dev, idx, arg);
		if (ret) {
			pr_err("fail to get dcam res\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_PUT_DCAM_RES:
		ret = sprd_img_io_put_dcam_res(group, file, dev, idx, arg);
		if (ret) {
			pr_err("fail to put dcam res\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_SET_MODE:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&mode, (void __user *) arg,
				     sizeof(unsigned int));
		if (ret) {
			pr_err("fail to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		info->capture_mode = mode;
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("%d: capture mode %d\n", idx,
			   dev->dcam_cxt.capture_mode);
		break;
	case SPRD_IMG_IO_SET_CAP_SKIP_NUM:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&skip_num, (void __user *)arg,
				     sizeof(unsigned int));
		if (ret) {
			pr_err("fail to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		info->skip_number = skip_num;
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("%d: cap skip number %d\n", idx,
			   dev->dcam_cxt.skip_number);
		break;
	case SPRD_IMG_IO_SET_PATH_SKIP_NUM:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		ret = sprd_img_io_set_path_skip_num(dev, info, idx, arg);
		if (ret) {
			pr_err("fail to set path skip num\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_SET_SENSOR_SIZE:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		ret = sprd_img_io_set_sensor_size(dev, info, idx, arg);
		if (ret) {
			pr_err("fail to set sensor size\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_SET_SENSOR_TRIM:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		ret = sprd_img_io_set_sensor_trim(dev, info, idx, arg);
		if (ret) {
			pr_err("fail to set sensor trim\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_SET_FRM_ID_BASE:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		ret = sprd_img_io_set_frm_id_base(dev, info, idx, arg);
		if (ret) {
			pr_err("fail to set frm id base\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_SET_CROP:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		ret = sprd_img_io_set_crop(file, dev, arg);
		if (ret) {
			pr_err("fail to set crop\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_GET_FLASH_INFO:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		sprd_flash_get_info(info->set_flash.flash_index,
			SPRD_FLASH_LED_ALL, &info->capacity);

		ret = copy_to_user((struct sprd_flash_capacity *)arg,
			&info->capacity, sizeof(struct sprd_flash_capacity));

		DCAM_TRACE("get flash info, ret %d\n", ret);
		break;
	case SPRD_IMG_IO_SET_FLASH:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&info->set_flash,
				     (void __user *)arg,
				     sizeof(struct sprd_img_set_flash));
		if (ret) {
			pr_err("fail to get user info\n");
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
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		ret = sprd_img_io_set_output_size(dev, info, idx, arg);
		if (ret) {
			pr_err("fail to set output size\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_SET_ZOOM_MODE:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&zoom, (void __user *)arg,
				     sizeof(unsigned int));
		if (ret) {
			pr_err("fail to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		info->is_smooth_zoom = zoom;
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("%d: set zoom mode %d\n", idx, info->is_smooth_zoom);
		break;
	case SPRD_IMG_IO_SET_SENSOR_IF:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		ret = sprd_img_io_set_sensor_if(file, dev, idx, arg);
		if (ret) {
			pr_err("fail to set sensor if\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_SET_FRAME_ADDR:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		ret = sprd_img_io_set_frame_addr(file, dev, idx, arg);
		if (ret) {
			pr_err("fail to set frame addr\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_PATH_FRM_DECI:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		ret = sprd_img_io_path_frm_deci(dev, info, arg);
		if (ret) {
			pr_err("fail to path frm deci\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_SET_SHRINK:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		ret = sprd_img_io_set_shrink(dev, info, arg);
		if (ret) {
			pr_err("fail to set shrink\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_PDAF_CONTROL:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		ret = sprd_img_io_pdaf_control(dev, info, arg);
		if (ret) {
			pr_err("fail to pdaf control\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_PATH_PAUSE:
		pr_debug("fail to get valid ioctrl cmd %d\n",
			_IOC_NR(cmd));
		break;
	case SPRD_IMG_IO_PATH_RESUME:
		pr_debug("fail to get valid ioctrl cmd %d\n",
			_IOC_NR(cmd));

		break;
	case SPRD_IMG_IO_STREAM_ON:
		DCAM_TRACE("stream on\n");
		ret = sprd_camera_stream_on(file);
		break;
	case SPRD_IMG_IO_STREAM_OFF:
		pr_info("stream off\n");
		ret = sprd_camera_stream_off(file);
		break;
	case SPRD_IMG_IO_GET_FMT:
		ret = sprd_img_io_get_fmt(file, arg);
		if (ret) {
			pr_err("fail to check fmt\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_GET_CH_ID:
		DCAM_TRACE("get free channel\n");
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		sprd_img_get_free_channel(file, &channel_id, info->scene_mode);
		ret = copy_to_user((void __user *)arg, &channel_id,
				   sizeof(unsigned int));
		break;
	case SPRD_IMG_IO_GET_TIME:
		sprd_img_io_get_time(arg);
		if (ret) {
			pr_err("fail to get time\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_CHECK_FMT:
		ret = sprd_img_io_check_fmt(file, arg);
		if (ret) {
			pr_err("fail to check fmt\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_CFG_FLASH:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		sprd_img_io_cfg_flash(dev, arg);
		if (ret) {
			pr_err("fail to cfg_parm\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_START_CAPTURE:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&capture_param, (void __user *)arg,
				     sizeof(struct sprd_img_capture_param));
		if (ret) {
			pr_err("fail to get user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		if (unlikely(atomic_read(&dev->stream_on) == 0)) {
			pr_info("stream not on\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		if (dev->cap_flag == CAMERA_CAPTURE_STOP) {
			cap_flag = capture_param.type;
			pr_info("start capture\n");
			ret = sprd_isp_start_fmcu(dev->isp_dev_handle,
				cap_flag, 0);
			if (ret) {
				mutex_unlock(&dev->dcam_mutex);
				pr_err("fail to start offline\n");
				goto exit;
			}
			dev->cap_flag = CAMERA_CAPTURE_START;
		}
		mutex_unlock(&dev->dcam_mutex);
		break;
	case SPRD_IMG_IO_STOP_CAPTURE:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		mutex_lock(&dev->dcam_mutex);
		pr_info("stop capture\n");
		if (dev->cap_flag == CAMERA_CAPTURE_START) {
			dev->cap_flag = CAMERA_CAPTURE_STOP;
			ret = sprd_isp_fmcu_slice_stop(dev->isp_dev_handle);
			if (ret) {
				mutex_unlock(&dev->dcam_mutex);
				pr_err("fail to stop offline\n");
				goto exit;
			}
		}
		mutex_unlock(&dev->dcam_mutex);
		break;
	case SPRD_IMG_IO_GET_IOMMU_STATUS:
		ret = copy_from_user(&iommu_enable, (void __user *)arg,
				     sizeof(unsigned char));
		if (ret) {
			pr_err("fail to copy from user\n");
			goto exit;
		}

		if (sprd_iommu_attach_device(&group->pdev->dev) == 0)
			iommu_enable = 1;
		else
			iommu_enable = 0;

		ret = copy_to_user((void __user *)arg, &iommu_enable,
				   sizeof(unsigned char));
		break;
	case SPRD_IMG_IO_SET_SENSOR_MAX_SIZE:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		ret = sprd_img_io_set_sensor_max_size(dev, info, idx, arg);
		if (ret) {
			pr_err("fail to set sensor max size\n");
			goto exit;
		}
		break;
	case SPRD_IMG_IO_SBS_MODE:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&sbs_info, (void __user *)arg,
				     sizeof(struct sprd_img_sbs_info));
		if (ret) {
			pr_err("fail to get right user info\n");
			mutex_unlock(&dev->dcam_mutex);
			goto exit;
		}
		info->sbs_info.sbs_mode = sbs_info.sbs_mode;
		mutex_unlock(&dev->dcam_mutex);
		pr_info("%d: sbs_mode %d\n", idx,
			   info->sbs_info.sbs_mode);
		break;
	case SPRD_ISP_IO_CAPABILITY:
	case SPRD_ISP_IO_CFG_START:
	case SPRD_ISP_IO_CFG_PARAM:
	case SPRD_ISP_IO_SET_STATIS_BUF:
	case SPRD_ISP_IO_POST_3DNR:
	case SPRD_ISP_IO_POST_YNR:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		mutex_lock(&dev->dcam_mutex);
		DCAM_TRACE("start isp ioctl cmd%d dev %p\n",
			_IOC_NR(cmd), dev);
		ret = sprd_isp_k_ioctl(dev->isp_dev_handle, cmd, arg);
		mutex_unlock(&dev->dcam_mutex);
		break;
	case SPRD_ISP_IO_RAW_CAP:
		ret = sprd_img_get_dcam_dev(camerafile, &dev, &info);
		if (ret) {
			pr_err("fail to get dcam dev\n");
			goto exit;
		}
		mutex_lock(&dev->dcam_mutex);
		DCAM_TRACE("start isp raw cap ioctl cmd%d dev %p\n",
			_IOC_NR(cmd), dev);
		if (unlikely(atomic_read(&dev->stream_on) == 0)) {
			pr_info("raw_cap, idx: %d\n", idx);
			memset((void *)&info->raw_cap, 0x0,
				sizeof(struct isp_raw_proc_info));
			ret = copy_from_user(&info->raw_cap,
					     (void __user *)arg,
					     sizeof(struct isp_raw_proc_info));
			if (ret != 0) {
				pr_err("DCAM_CORE: fail to copy from user\n");
				mutex_unlock(&dev->dcam_mutex);
				goto exit;
			}

			dev->raw_cap = 1;
			ret = sprd_img_queue_init(&dev->queue);
			if (unlikely(ret != 0)) {
				mutex_unlock(&dev->dcam_mutex);
				pr_err("fail to init queue\n");
				goto exit;
			}
			ret = sprd_img_isp_reg_isr(dev);
			if (unlikely(ret)) {
				pr_err("fail to register isp isr\n");
				mutex_unlock(&dev->dcam_mutex);
				goto exit;
			}
			info->sensor_id = info->raw_cap.sensor_id;
			atomic_set(&dev->stream_on, 1);
		}
		ret = sprd_isp_k_ioctl(dev->isp_dev_handle, cmd, arg);
		mutex_unlock(&dev->dcam_mutex);
		break;
	default:
		pr_info("fail to get valid cmd 0x%x\n", cmd);
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
	struct completion *irq_com = NULL;

	if (cnt < sizeof(struct sprd_img_read_op)) {
		pr_err("fail to img read, cnt %zd read_op %d\n", cnt,
		       (int32_t)sizeof(struct sprd_img_read_op));
		return -EIO;
	}

	if (copy_from_user(&read_op, (void __user *)u_data, cnt)) {
		pr_err("fail to get user info\n");
		return -EFAULT;
	}

	if (!dev) {
		pr_info("fail to inited dev %p\n", dev);
		return -EFAULT;
	}

	switch (read_op.cmd) {
	case SPRD_IMG_GET_SCALE_CAP:
		DCAM_TRACE("get scale capbility\n");
		read_op.parm.reserved[0] = ISP_PATH2_LINE_BUF_LENGTH;
		read_op.parm.reserved[1] = CAMERA_SC_COEFF_UP_MAX;
		read_op.parm.reserved[2] = DCAM_SCALING_THRESHOLD;
		DCAM_TRACE("line threshold %d, sc factor %d, scaling %d.\n",
			   read_op.parm.reserved[0], read_op.parm.reserved[1],
			   read_op.parm.reserved[2]);
		break;
	case SPRD_IMG_GET_FRM_BUFFER:
		idx = sprd_sensor_find_dcam_id(read_op.sensor_id);
		if (idx == -1) {
			pr_err("fail to get attch dcam id\n");
			return -EFAULT;
		}
		dev = group->dev[idx];
		if (read_op.sensor_id == SPRD_SENSOR_MAIN2_ID_E
		    || read_op.sensor_id == SPRD_SENSOR_SUB2_ID_E)
			irq_com = &dev->irq_com_sub;
		else
			irq_com = &dev->irq_com;
		memset(&read_op, 0, sizeof(struct sprd_img_read_op));
		while (1) {
			ret = wait_for_completion_interruptible(irq_com);
			if (ret == 0) {
				break;
			} else if (ret == -ERESTARTSYS) {
				read_op.evt = IMG_SYS_BUSY;
				ret = DCAM_RTN_SUCCESS;
				goto read_end;
			} else {
				pr_err("read frame buf, fail to down, %d\n",
				       ret);
				WARN_ON(1);
				return -EPERM;
			}
		}

		if (sprd_img_queue_read(&dev->queue, &node)) {
			DCAM_TRACE("fail to read frame buffer queue\n");
			read_op.evt = IMG_SYS_BUSY;
			ret = DCAM_RTN_SUCCESS;
			goto read_end;
		} else {
			if (node.invalid_flag) {
				DCAM_TRACE("fail to get valid node\n");
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
			read_op.parm.frame.addr_offset = node.addr_offset;
			read_op.parm.frame.kaddr[0] = node.kaddr[0];
			read_op.parm.frame.kaddr[1] = node.kaddr[1];
			read_op.parm.frame.buf_size = node.buf_size;
			read_op.parm.frame.irq_type = node.irq_type;
			read_op.parm.frame.irq_property = node.irq_property;
			read_op.parm.frame.frame_id = node.frame_id;
			read_op.parm.frame.mfd = node.mfd[0];
			DCAM_TRACE("index %d real_index %d frm_id_base %d\n",
				   read_op.parm.frame.index,
				   read_op.parm.frame.real_index,
				   read_op.parm.frame.frm_base_id);
		} else {
			if (read_op.evt == IMG_TIMEOUT ||
			    read_op.evt == IMG_TX_ERR) {
				pr_info("fail to get valid evt %d\n",
					read_op.evt);
				read_op.parm.frame.irq_type = node.irq_type;
				read_op.parm.frame.irq_property
					= node.irq_property;
				csi_api_reg_trace();
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
					pr_err("fail to init queue\n");
				ret = sprd_img_isp_unreg_isr(dev);
				if (unlikely(ret))
					pr_err("fail to register isp isr\n");
				atomic_set(&dev->stream_on, 0);
			}
		}
		if (read_op.evt == IMG_TX_STOP)
			pr_info("sprd_img_read: tx stop\n");
		if (read_op.parm.frame.channel_id == CAMERA_PDAF_PATH)
			pr_info("sprd_img_read: pdaf\n");

		break;
	case SPRD_IMG_GET_PATH_CAP:
		DCAM_TRACE("get path capbility\n");
		sprd_dcam_get_path_capability(&path_capability);
		read_op.parm.capability.count = path_capability.count;
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
		pr_err("fail to write, cnt %zd read_op %d\n", cnt,
		       (int32_t)sizeof(struct sprd_img_write_op));
		return -EIO;
	}

	if (copy_from_user(&write_op, (void __user *)u_data, cnt)) {
		pr_err("fail to get user info\n");
		return -EFAULT;
	}

	if (!dev)
		return -EFAULT;

	switch (write_op.cmd) {
	case SPRD_IMG_STOP_DCAM:
		idx = sprd_sensor_find_dcam_id(write_op.sensor_id);
		if (idx == -1) {
			pr_err("fail to get attch dcam id\n");
			return -EFAULT;
		}
		dev = group->dev[idx];
		if (!dev) {
			pr_err("fail to get valid dev:NULL\n");
			return -EFAULT;
		}
		mutex_lock(&dev->dcam_mutex);
		ret = sprd_img_tx_stop(dev, write_op.sensor_id);
		mutex_unlock(&dev->dcam_mutex);
		break;
	case SPRD_IMG_FREE_FRAME:
		if (atomic_read(&dev->stream_on) == 0) {
			pr_info("fail to free frame, dev close!");
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
			pr_err("fail to free frame buf, channel %d\n",
			       write_op.channel_id);
			return -EINVAL;
		}

		if (path->status == PATH_IDLE) {
			DCAM_TRACE("fail to free frame buf, channel %d\n",
				   write_op.channel_id);
			return -EINVAL;
		}

		if (write_op.index >
		    (path->frm_id_base + path->frm_cnt_act - 1)) {
			pr_err("fail to write, index %d, frm_id_base %d frm_cnt_act %d\n",
			       write_op.index, path->frm_id_base,
			       path->frm_cnt_act);
			ret = -EINVAL;
		} else if (write_op.index < path->frm_id_base) {
			pr_err("fail to write, index %d, frm_id_base %d\n",
			       write_op.index, path->frm_id_base);
			ret = -EINVAL;
		} else {
			index = write_op.index - path->frm_id_base;

			DCAM_TRACE("free frame buf, channel %d, index 0x%x\n",
				   write_op.channel_id, write_op.index);
		}
		break;
	default:
		pr_err("fail to get valid cmd!\n");
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
	.compat_ioctl = compat_sprd_img_k_ioctl,
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
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	pr_info("Start camera img probe\n");
	group = vzalloc(sizeof(struct camera_group));
	if (group == NULL)
		return -ENOMEM;

	ret = misc_register(&image_dev);
	if (ret) {
		pr_err("fail to register misc devices, ret %d\n", ret);
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
		pr_err("fail to init pw domain\n");
		goto exit;
	}

	pr_info("sprd dcam dev name %s\n", pdev->dev.init_name);
	ret = sprd_dcam_parse_dt(pdev->dev.of_node, &group->dcam_count);
	if (ret) {
		pr_err("fail to parse dcam dts\n");
		goto exit;
	}

	pr_info("sprd isp dev name %s\n", pdev->dev.init_name);
	ret = sprd_isp_parse_dt(pdev->dev.of_node, &group->isp_count);
	if (ret) {
		pr_err("fail to parse isp dts\n");
		goto exit;
	}

	if (group->dcam_count != group->isp_count) {
		pr_err("fail to config dts tree\n");
		ret = -EINVAL;
		goto exit;
	}

	if (sprd_dcam_drv_init(pdev)) {
		pr_err("fail to call sprd_dcam_drv_init\n");
		ret = -EINVAL;
		goto exit;
	}

	if (sprd_isp_drv_init(pdev)) {
		pr_err("fail to call sprd_isp_drv_init\n");
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
		pr_err("fail to get valid input ptr\n");
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
