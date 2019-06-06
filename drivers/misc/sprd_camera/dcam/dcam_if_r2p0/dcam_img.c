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
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/vmalloc.h>
#include <linux/regmap.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include <video/sprd_img.h>
#include <video/sprd_mm.h>
#include <video/sprd_cpp.h>
#include "flash_drv.h"
#include "csi_driver.h"
#include "csi_api.h"
#include "isp_drv.h"
#include "dcam_drv.h"
#include "dcam2isp.h"
#include "cam_pw_domain.h"
#include "cpp_core.h"
#include "cam_util.h"

/* Macro Definitions */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "[sprd_img]: line=%d: " fmt, \
	__LINE__


#define IMG_DEVICE_NAME			"sprd_image"
#define DCAM_INVALID_FOURCC		0xFFFFFFFF
#define DCAM_QUEUE_LENGTH		48
#define DCAM_TIMING_LEN			16
#ifdef CONFIG_SC_FPGA
#define DCAM_TIMEOUT			(2000*100)
#else
#define DCAM_TIMEOUT			1500
#endif

#define DCAM_ZOOM_LEVEL_MAX		4
#define DCAM_ZOOM_STEP(x, y)		(((x) - (y)) / DCAM_ZOOM_LEVEL_MAX)

#define DCAM_PIXEL_XOFFSET_ALIGNED		2
#define DCAM_PIXEL_YOFFSET_ALIGNED		2
#define DCAM_PIXEL_WIDTH_ALIGNED		4
#define DCAM_PIXEL_HEIGHT_ALIGNED		2

#define DCAM_WIDTH(w)			((w) & ~(DCAM_PIXEL_WIDTH_ALIGNED - 1))
#define DCAM_HEIGHT(h)			((h) & ~(DCAM_PIXEL_HEIGHT_ALIGNED - 1))

#define ALIGN_SIZE_DOWN(size, align_size) ((size) & ~(align_size - 1))

#define DISCARD_FRAME_TIME		10000

#define IMG_PRINT_IF_ERR(n) \
	do { \
		if (unlikely(n)) { \
			pr_err("%s err, code %d", __func__, n); \
		} \
	} while (0)


/* Structure Definitions */
static struct platform_device *s_pdev;
typedef int (*path_cfg_func)(enum dcam_id, enum dcam_path_index,
			     enum dcam_cfg_id, void *);

enum {
	PATH_IDLE = 0x00,
	PATH_RUN,
};

struct dcam_format {
	char *name;
	unsigned int fourcc;
	int depth;
};

struct dcam_node {
	unsigned int irq_flag;
	unsigned int f_type;
	unsigned int index;
	unsigned int height;
	unsigned int yaddr;
	unsigned int uaddr;
	unsigned int vaddr;
	unsigned int yaddr_vir;
	unsigned int uaddr_vir;
	unsigned int vaddr_vir;
	unsigned int invalid_flag;
	unsigned int mfd[3];
	unsigned int reserved[2];
	struct timeval time;
	ktime_t boot_time;
	unsigned int irq_type;
	unsigned int length;
	unsigned int frm_base_id;
	unsigned int channel_id;
	unsigned int format;
};

struct dcam_queue {
	struct dcam_node node[DCAM_QUEUE_LENGTH];
	struct dcam_node *write;
	struct dcam_node *read;
	unsigned int wcnt;
	unsigned int rcnt;
	spinlock_t lock;
};

struct dcam_img_buf_addr {
	struct dcam_addr frm_addr;
	struct dcam_addr frm_addr_vir;
};

struct dcam_img_buf_queue {
	struct dcam_img_buf_addr buf_addr[DCAM_FRM_CNT_MAX];
	struct dcam_img_buf_addr *write;
	struct dcam_img_buf_addr *read;
	unsigned int wcnt;
	unsigned int rcnt;
};

struct dcam_path_spec {
	unsigned int is_work;
	unsigned int is_from_isp;
	unsigned int rot_mode;
	unsigned int status;
	struct dcam_size in_size;
	struct dcam_path_dec img_deci;
	struct dcam_rect in_rect;
	struct dcam_rect in_rect_current;
	struct dcam_rect in_rect_backup;
	struct dcam_size isp_out_size;
	struct dcam_rect cpp_in_rect;
	struct dcam_size out_size;
	enum dcam_fmt out_fmt;
	struct dcam_endian_sel end_sel;
	unsigned int fourcc;
	unsigned int pixel_depth;
	unsigned int frm_id_base;
	unsigned int frm_type;
	unsigned int index[DCAM_FRM_CNT_MAX];
	struct dcam_img_buf_queue buf_queue;
	struct dcam_addr frm_reserved_addr;
	struct dcam_addr frm_reserved_addr_vir;
	struct dcam_frame *frm_ptr[DCAM_FRM_CNT_MAX];
	unsigned int frm_cnt_act;
	unsigned int path_frm_deci;
	struct dcam_regular_desc regular_desc;
	struct dcam_jpegls_desc jpegls_desc;
	struct sprd_pdaf_control pdaf_ctrl;
};

struct dcam_info {
	unsigned int if_mode;
	unsigned int sn_mode;
	unsigned int yuv_ptn;
	unsigned int data_bits;
	unsigned int is_loose;
	unsigned int lane_num;
	unsigned int pclk;
	struct dcam_cap_sync_pol sync_pol;
	unsigned int frm_deci;
	struct dcam_cap_dec img_deci;
	struct dcam_size cap_in_size;
	struct dcam_rect cap_in_rect;
	struct dcam_size cap_out_size;
	struct dcam_size dst_size;
	unsigned int pxl_fmt;
	unsigned int need_isp_tool;
	unsigned int need_isp;
	unsigned int rt_refocus;
	struct dcam_rect path_input_rect;
	unsigned int path_input_rect_change;

	struct dcam_path_spec dcam_path[DCAM_PATH_NUM];

	unsigned int capture_mode;
	unsigned int skip_number;
	struct sprd_img_set_flash set_flash;
	unsigned int after_af;
	unsigned int is_smooth_zoom;
	struct timeval timestamp;
	unsigned int scene_mode;
};

struct dcam_dev {
	struct mutex dcam_mutex;
	atomic_t users;
	struct dcam_info dcam_cxt;
	atomic_t stream_on;
	atomic_t run_flag;
	struct proc_dir_entry *proc_file;
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
	void *driver_data;
	enum dcam_id idx;
};

struct cpp_reqnode {
	struct dcam_file *dcamfile;
	unsigned int isp_id;
	unsigned int img_id;
};

struct cpp_dev {
	struct completion zoom_thread_com;
	struct task_struct *zoom_thread;
	unsigned int is_zoom_thread_stop;
	unsigned int is_zoom_thread_pause;
	unsigned int zoom_level;

	struct cam_queue cpp_queue;
	unsigned int cpp_id;
};

struct dcam_group {
	unsigned int dev_inited;
	unsigned int mode_inited;
	unsigned int dcam_res_used;
	unsigned int cowork_mod;
	atomic_t dcam_opened;
	struct dcam_dev *dev[DCAM_ID_MAX];
	struct miscdevice *md;
	struct platform_device *pdev;
	struct wake_lock wakelock;
	struct cpp_dev *cpp_dev[DCAM_ID_MAX];
};

struct dcam_file {
	int idx;
	struct dcam_group *grp;
	int isp_id;   /* attached isp id */
	struct completion irq_com;
	struct dcam_queue queue;
	struct timer_list dcam_timer;
};

/* Static Variables Declaration */
static unsigned char dcam_cowork_flag;
static DEFINE_MUTEX(global_dcam_mutex);
static atomic_t global_dcam_flag = ATOMIC_INIT(0);
static atomic_t global_zoom_thread_flag[2] = {ATOMIC_INIT(0), ATOMIC_INIT(0)};

enum dcam_flag {
	DCAM_IDLE = 0,
	DCAM_RUNNING,
	DCAM_RELEASE
};
static void get_dcam_lock(enum dcam_flag flag)
{
	if (flag == DCAM_RUNNING) {
		while (atomic_read(&global_dcam_flag) == 0xaa)
			usleep_range(500, 1000);
		atomic_inc(&global_dcam_flag);
	} else if (flag == DCAM_RELEASE) {
		while (atomic_read(&global_dcam_flag) > 0)
			usleep_range(500, 1000);
		atomic_set(&global_dcam_flag, 0xaa);
	} else
		pr_err("invalid flag!\n");
}

static void put_dcam_lock(enum dcam_flag flag)
{
	if (flag == DCAM_RUNNING)
		atomic_dec(&global_dcam_flag);
	else if (flag == DCAM_RELEASE)
		atomic_set(&global_dcam_flag, 0);
	else
		pr_err("invalid flag!\n");
}

static inline struct dcam_dev *get_dcam_dev(struct dcam_file *dcam_file)
{
	struct dcam_group *group = NULL;
	struct dcam_dev *dev = NULL;

	if (dcam_file == NULL) {
		pr_err("dcam_file is NULL\n");
		return NULL;
	}

	group = dcam_file->grp;
	if (group == NULL) {
		pr_err("group is NULL\n");
		return NULL;
	}

	dev = group->dev[dcam_file->idx];
	if (dev == NULL) {
		pr_err("dev%d is NULL\n", dcam_file->idx);
		return NULL;
	}
	return dev;
}

static struct dcam_format dcam_img_fmt[] = {
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
		.name = "PDAF_TYPE3",
		.fourcc = IMG_PIX_FMT_PDA3,
		.depth = 8,
	},
	{
		.name = "JPEG",
		.fourcc = IMG_PIX_FMT_JPEG,
		.depth = 8,
	},
};

/* Internal Function Implementation */

static int img_get_timestamp(struct timeval *tv)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);
	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = ts.tv_nsec / NSEC_PER_USEC;

	return 0;
}

static int sprd_img_setflash(enum dcam_id idx,
			     struct sprd_img_set_flash *set_flash)
{
	sprd_flash_ctrl(set_flash);
	DCAM_TRACE("%d set flash\n", idx);

	return 0;
}

static int sprd_img_opt_flash(struct dcam_frame *frame, void *param)
{
	struct dcam_dev *dev = (struct dcam_dev *)param;
	struct dcam_info *info = NULL;
	unsigned int led0_ctrl;
	unsigned int led1_ctrl;
	unsigned int led0_status;
	unsigned int led1_status;

	if (dev == NULL) {
		pr_err("dev is NULL\n");
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
		info->set_flash.flash_index = 0;
		sprd_img_setflash(dev->idx, &info->set_flash);
		info->set_flash.led0_ctrl = 0;
		info->set_flash.led1_ctrl = 0;
		info->set_flash.led0_status = FLASH_STATUS_MAX;
		info->set_flash.led1_status = FLASH_STATUS_MAX;
	}

	return 0;
}

static int sprd_img_start_flash(struct dcam_frame *frame, void *param)
{
	struct dcam_file *dcam_file = (struct dcam_file *)param;
	struct dcam_dev *dev = NULL;
	struct dcam_info *info = NULL;
	unsigned int need_light = 1;
	unsigned int led0_ctrl;
	unsigned int led1_ctrl;
	unsigned int led0_status;
	unsigned int led1_status;

	dev = get_dcam_dev(dcam_file);
	if (dev == NULL)
		return -EINVAL;

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
	struct dcam_dev *dev = (struct dcam_dev *)arg;
	struct sprd_img_set_flash set_flash;
	int ret = 0;

	if (dev == NULL) {
		pr_err("flash_thread_loop, dev is NULL\n");
		return -1;
	}
	while (1) {
		ret = wait_for_completion_interruptible(&dev->flash_thread_com);
		if (ret == 0) {
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
	struct dcam_dev *dev = (struct dcam_dev *)param;
	char thread_name[20] = { 0 };

	if (dev == NULL) {
		pr_err("dev is NULL\n");
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
	struct dcam_dev *dev = (struct dcam_dev *)param;

	if (dev == NULL) {
		pr_err("dev is NULL\n");
		return -1;
	}

	if (dev->flash_thread) {
		dev->is_flash_thread_stop = 1;
		complete(&dev->flash_thread_com);
		if (dev->is_flash_thread_stop != 0) {
			while (dev->is_flash_thread_stop)
				usleep_range(1000, 1500);
		}
		dev->flash_thread = NULL;
	}

	return 0;
}

static struct dcam_format *sprd_img_get_format(unsigned int fourcc)
{
	struct dcam_format *fmt;
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

static int sprd_img_check_path0_cap(unsigned int fourcc,
				    struct sprd_img_format *f,
				    struct dcam_info *info)
{
	struct dcam_path_spec *path = &info->dcam_path[DCAM_PATH0];

	DCAM_TRACE("check format for path0\n");

	path->frm_type = f->channel_id;
	path->is_work = 0;

	switch (fourcc) {
	case IMG_PIX_FMT_GREY:
		path->out_fmt = DCAM_RAWRGB;
		path->pixel_depth = info->is_loose ? 16 : 10;
		path->end_sel.y_endian = DCAM_ENDIAN_BIG;
		break;

	case IMG_PIX_FMT_PDA3:
		path->out_fmt = DCAM_RAWRGB;
		path->pixel_depth = info->is_loose ? 16 : 10;
		path->end_sel.y_endian = DCAM_ENDIAN_LITTLE;
		break;

	case IMG_PIX_FMT_JPEG:
		path->out_fmt = DCAM_JPEG;
		path->end_sel.y_endian = DCAM_ENDIAN_LITTLE;
		break;

	default:
		pr_info("unsupported image format for path0 0x%x\n", fourcc);
		return -EINVAL;
	}
	path->fourcc = fourcc;

	DCAM_TRACE("check format for path0: out_fmt=%d, is_loose=%d\n",
		   path->out_fmt, info->is_loose);
	path->out_size.w = f->width;
	path->out_size.h = f->height;

	path->is_work = 1;

	return 0;
}

static int sprd_img_cap_cfg(struct dcam_dev *dev)
{
	int ret = DCAM_RTN_SUCCESS;
	unsigned int param = 0;
	struct dcam_info *info = NULL;
	enum dcam_id idx = 0;

	if (dev == NULL)
		return -EINVAL;

	info = &dev->dcam_cxt;
	idx = dev->idx;

	ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_INTERFACE, &info->if_mode);

	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_SENSOR_MODE, &info->sn_mode);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_SYNC_POL, &info->sync_pol);

	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	if (info->dcam_path[DCAM_PATH1].is_work ||
	    info->dcam_path[DCAM_PATH2].is_work ||
	    info->dcam_path[DCAM_PATH3].is_work)
		param = 1;
	else
		param = 0;

	ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_TO_ISP, &param);

	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_YUV_TYPE, &info->yuv_ptn);

	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_DATA_BITS, &info->data_bits);

	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	if (info->sn_mode == DCAM_CAP_MODE_RAWRGB &&
	    info->if_mode == DCAM_CAP_IF_CSI2) {
		ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_DATA_PACKET,
					&info->is_loose);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	}

	ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_FRM_DECI, &info->frm_deci);

	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_INPUT_RECT, &info->cap_in_rect);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_FRM_COUNT_CLR, NULL);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_PRE_SKIP_CNT, &info->skip_number);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_SAMPLE_MODE, &info->capture_mode);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_ZOOM_MODE, &info->is_smooth_zoom);

	ret = sprd_dcam_cap_cfg(idx, DCAM_CAP_IMAGE_XY_DECI, &info->img_deci);

exit:
	return ret;
}

static uint32_t sprd_img_get_isp_img_id(uint32_t channel_id)
{
	uint32_t img_id = ISP_PATH_MAX_NUM;

	switch (channel_id) {
	case DCAM_PATH1:
		img_id = ISP_PATH_PREIVEW;
		break;
	case DCAM_PATH2:
		img_id = ISP_PATH_VIDEO;
		break;
	case DCAM_PATH3:
		img_id = ISP_PATH_STILL;
		break;
	default:
		/*pr_err("No isp img id mapped to %d\n", channel_id);*/
		break;
	}
	return img_id;
}

static uint32_t sprd_img_get_channel_id(uint32_t img_id)
{
	uint32_t channel_id = DCAM_PATH_MAX;

	switch (img_id) {
	case ISP_PATH_PREIVEW:
		channel_id = DCAM_PATH1;
		break;
	case ISP_PATH_VIDEO:
		channel_id = DCAM_PATH2;
		break;
	case ISP_PATH_STILL:
		channel_id = DCAM_PATH3;
		break;
	default:
		/*pr_err("No channel mapped to %d\n", img_id);*/
		break;
	}
	return channel_id;
}

static int sprd_img_queue_init(struct dcam_queue *queue)
{
	if (queue == NULL)
		return -EINVAL;

	memset(queue, 0, sizeof(*queue));
	queue->write = &queue->node[0];
	queue->read = &queue->node[0];
	spin_lock_init(&queue->lock);

	return 0;
}

static int sprd_img_queue_clr(struct dcam_queue *queue)
{
	unsigned long flags;

	if (queue == NULL)
		return -EINVAL;

	spin_lock_irqsave(&queue->lock, flags);
	memset(queue->node, 0, sizeof(struct dcam_node) * DCAM_QUEUE_LENGTH);
	queue->write = &queue->node[0];
	queue->read = &queue->node[0];
	spin_unlock_irqrestore(&queue->lock, flags);

	return 0;
}

static int sprd_img_queue_write(struct dcam_queue *queue,
				struct dcam_node *node)
{
	struct dcam_node *ori_node;
	unsigned long flags;

	if (queue == NULL || node == NULL)
		return -EINVAL;

	spin_lock_irqsave(&queue->lock, flags);
	ori_node = queue->write;
	queue->wcnt++;
	*queue->write++ = *node;
	if (queue->write > &queue->node[DCAM_QUEUE_LENGTH - 1])
		queue->write = &queue->node[0];

	if (queue->write == queue->read) {
		queue->write = ori_node;
		pr_info("warning, queue is full\n");
	}
	spin_unlock_irqrestore(&queue->lock, flags);
	return 0;
}

static int sprd_img_queue_read(struct dcam_queue *queue, struct dcam_node *node)
{
	int ret = DCAM_RTN_SUCCESS;
	int flag = 0;
	unsigned long flags;

	if (queue == NULL || node == NULL)
		return -EINVAL;

	spin_lock_irqsave(&queue->lock, flags);
	if (queue->read != queue->write) {
		flag = 1;
		*node = *queue->read;
		queue->read++;
		queue->rcnt++;
		if (queue->read > &queue->node[DCAM_QUEUE_LENGTH - 1])
			queue->read = &queue->node[0];
	}
	if (!flag)
		ret = -EAGAIN;
	spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

static int sprd_img_queue_disable(struct dcam_queue *queue,
				  unsigned int channel_id)
{
	struct dcam_node *cur_node;

	if (queue == NULL)
		return -EINVAL;

	cur_node = queue->read;
	while (cur_node != queue->write) {
		if (channel_id == cur_node->f_type)
			cur_node->invalid_flag = 1;

		if (cur_node >= &queue->node[DCAM_QUEUE_LENGTH - 1])
			cur_node = &queue->node[0];
		else
			cur_node++;
	}
	return 0;
}

static int sprd_img_queue_enable(struct dcam_queue *queue,
				 unsigned int channel_id)
{
	unsigned int i;

	if (queue == NULL)
		return -EINVAL;

	for (i = 0; i < DCAM_QUEUE_LENGTH; i++)
		queue->node[i].invalid_flag = 0;

	return 0;
}

static int sprd_img_buf_queue_init(struct dcam_img_buf_queue *queue)
{
	if (queue == NULL)
		return -EINVAL;

	memset(queue, 0, sizeof(*queue));
	queue->write = &queue->buf_addr[0];
	queue->read = &queue->buf_addr[0];

	return 0;
}

static int sprd_img_buf_queue_write(struct dcam_img_buf_queue *queue,
				    struct dcam_img_buf_addr *buf_addr)
{
	struct dcam_img_buf_addr *ori_node;

	if (queue == NULL || buf_addr == NULL)
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

static int sprd_img_tx_error(struct dcam_frame *frame,
		void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct dcam_file *dcam_file = (struct dcam_file *)param;
	struct dcam_dev *dev = NULL;
	struct dcam_node node;

	if (param == NULL)
		return -EINVAL;

	dev = get_dcam_dev(dcam_file);
	if (dev == NULL || atomic_read(&dev->stream_on) == 0)
		return -EINVAL;

	memset((void *)&node, 0, sizeof(struct dcam_node));
	atomic_set(&dev->run_flag, 1);
	node.irq_flag = IMG_TX_ERR;
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
	ret = sprd_img_queue_write(&dcam_file->queue, &node);
	if (ret)
		return ret;

	complete(&dcam_file->irq_com);

	pr_err("%s: tx error\n", __func__);

	return ret;
}

static int sprd_img_tx_done(struct dcam_frame *frame,
		void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct dcam_file *dcam_file = (struct dcam_file *)param;
	struct dcam_dev *dev = NULL;
	struct dcam_path_spec *path;
	struct dcam_node node;

	if (frame == NULL || param == NULL)
		return -EINVAL;

	dev = get_dcam_dev(dcam_file);
	if (dev == NULL || atomic_read(&dev->stream_on) == 0)
		return -EINVAL;

	path = &dev->dcam_cxt.dcam_path[frame->type];
	if (path->status == PATH_IDLE)
		return ret;

	memset((void *)&node, 0, sizeof(struct dcam_node));
	node.irq_flag = IMG_TX_DONE;
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

	node.boot_time = ktime_get_boottime();
	img_get_timestamp(&node.time);

	pr_debug("flag 0x%x type 0x%x index 0x%x\n",
		   node.irq_flag, node.f_type, node.index);

	if (frame->type == DCAM_PATH0 &&
	    dev->dcam_cxt.sn_mode == DCAM_CAP_MODE_JPEG) {
		sprd_dcam_cap_get_info(dev->idx, DCAM_CAP_JPEG_GET_LENGTH,
				       &node.reserved[0]);
		if (node.reserved[0] < DCAM_JPEG_LENGTH_MIN)
			return sprd_img_tx_error(frame, param);
	}

	ret = sprd_img_queue_write(&dcam_file->queue, &node);
	if (ret)
		return ret;

	pr_debug("sprd_img %d %d %p\n", dev->idx, frame->type, dev);

	complete(&dcam_file->irq_com);
	return ret;
}

static int sprd_img_no_mem(struct dcam_frame *frame,
		void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct dcam_file *dcam_file = (struct dcam_file *)param;
	struct dcam_dev *dev = NULL;
	struct dcam_node node;

	if (param == NULL)
		return -EINVAL;

	dev = get_dcam_dev(dcam_file);
	if (dev == NULL || atomic_read(&dev->stream_on) == 0)
		return -EINVAL;

	atomic_set(&dev->run_flag, 1);

	memset((void *)&node, 0, sizeof(struct dcam_node));
	node.irq_flag = IMG_NO_MEM;
	node.f_type = frame->type;
	node.index = frame->fid;
	node.height = frame->height;
	node.yaddr = frame->yaddr;
	node.uaddr = frame->uaddr;
	node.vaddr = frame->vaddr;
	node.yaddr_vir = frame->yaddr_vir;
	node.uaddr_vir = frame->uaddr_vir;
	node.vaddr_vir = frame->vaddr_vir;

	ret = sprd_img_queue_write(&dcam_file->queue, &node);
	if (ret)
		return ret;

	complete(&dcam_file->irq_com);

	pr_info("%s: no mem\n", __func__);

	return ret;
}

static int sprd_img_tx_stop(void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct dcam_file *dcamfile  = (struct dcam_file *)param;
	struct dcam_node node;

	DCAM_TRACE("sprd_img_tx_stop\n");
	memset((void *)&node, 0, sizeof(struct dcam_node));
	node.irq_flag = IMG_TX_STOP;

	ret = sprd_img_queue_write(&dcamfile->queue, &node);
	if (ret)
		return ret;

	complete(&dcamfile->irq_com);
	return ret;
}

static int sprd_img_dcam_reg_isr(struct dcam_file *param, enum dcam_id dcam_id)
{
	if (param == NULL) {
		pr_err("param is NULL\n");
		return -EINVAL;
	}
	sprd_dcam_reg_isr(dcam_id, DCAM_PATH0_DONE, sprd_img_tx_done, param);
	sprd_dcam_reg_isr(dcam_id, DCAM_PATH0_OV, sprd_img_tx_error, param);
	sprd_dcam_reg_isr(dcam_id, DCAM_ISP_OV, sprd_img_tx_error, param);
	sprd_dcam_reg_isr(dcam_id, DCAM_MIPI_OV, sprd_img_tx_error, param);
	sprd_dcam_reg_isr(dcam_id, DCAM_SN_LINE_ERR,
			  sprd_img_tx_error, param);
	sprd_dcam_reg_isr(dcam_id, DCAM_SN_FRAME_ERR,
			  sprd_img_tx_error, param);
	sprd_dcam_reg_isr(dcam_id, DCAM_JPEG_BUF_OV, sprd_img_no_mem, param);
	sprd_dcam_reg_isr(dcam_id, DCAM_SN_EOF, sprd_img_start_flash, param);

	return 0;
}

static inline int sprd_img_isp_tx_done(unsigned int isp_id, unsigned int img_id,
	void *param)
{
	int ret = DCAM_RTN_SUCCESS;
	struct dcam_file *dcamfile = NULL;
	struct dcam_dev *dcam_dev = NULL;
	struct dcam_group *cpp_grp = NULL;
	struct isp_if_context *ispif = NULL;
	struct isp_pipe_dev *isp_pipeline = NULL;
	struct cpp_dev *cpp_dev = NULL;
	struct cpp_reqnode cpp_node;

	dcamfile = (struct dcam_file *)param;
	if (dcamfile == NULL) {
		pr_err("dcamfile is NULL!\n");
		ret = -EFAULT;
		goto exit;
	}

	dcam_dev = get_dcam_dev(dcamfile);
	if (dcam_dev == NULL) {
		pr_err("dcam_dev is NULL\n");
		return -EFAULT;
	}

	ispif = sprd_isp_get_isp_handle((enum isp_dev_id)isp_id);
	if (ispif == NULL) {
		pr_err("ispif is NULL.\n");
		return -EFAULT;
	}

	isp_pipeline = ispif->isp_pipe;
	if (isp_pipeline == NULL) {
		pr_err("isp_pipeline is null error\n");
		return -EINVAL;
	}

	if (isp_pipeline->scenario_id ==
		ISP_SCENARIO_PREVIEW_STILL_SS &&
		img_id == ISP_IMG_STILL_CAPTURE) {
		atomic_set(&dcam_dev->run_flag, 1);
		goto exit;
	}


	cpp_grp = dcamfile->grp;
	if (cpp_grp != NULL) {
		if (img_id == ISP_PATH_PREIVEW || img_id == ISP_PATH_VIDEO)
			cpp_dev = cpp_grp->cpp_dev[0];
		else
			cpp_dev = cpp_grp->cpp_dev[1];

		/*cpp_dev = cpp_grp->cpp_dev[dcamfile->idx];*/
		/*cpp_dev->dcamfile = dcamfile;  */ /*attach dcam file*/
		/*cpp_dev->isp_id = isp_id;*/
		/*cpp_dev->img_id = img_id;*/
		cpp_dev->zoom_level = 1;

		cpp_node.dcamfile = dcamfile;   /*attach dcam file*/
		cpp_node.isp_id = isp_id;
		cpp_node.img_id = img_id;
		sprd_cam_queue_write(&cpp_dev->cpp_queue, &cpp_node);
		pr_debug("isp done cpp_id=%d isp_id=%d img_id=%d\n",
			cpp_dev->cpp_id, isp_id, img_id);
		complete(&cpp_dev->zoom_thread_com);
	} else {
		pr_err("get cpp_grp is NULL.\n");
		ret = -EFAULT;
	}
exit:
	return ret;
}

static int sprd_img_isp_reg_isr(struct dcam_file *param, enum isp_dev_id isp_id)
{
	sprd_isp_reg_callback(isp_id, ISP_PATH_PREIVEW,
		sprd_img_isp_tx_done, param);
	sprd_isp_reg_callback(isp_id, ISP_PATH_VIDEO,
		sprd_img_isp_tx_done, param);
	sprd_isp_reg_callback(isp_id, ISP_PATH_STILL,
		sprd_img_isp_tx_done, param);
	return 0;
}

static int sprd_img_isp_unreg_isr(enum isp_dev_id isp_id)
{
	sprd_isp_reg_callback(isp_id, ISP_PATH_PREIVEW,
		NULL, NULL);
	sprd_isp_reg_callback(isp_id, ISP_PATH_VIDEO,
		NULL, NULL);
	sprd_isp_reg_callback(isp_id, ISP_PATH_STILL,
		NULL, NULL);
	return 0;
}

static int sprd_img_unreg_isr(struct dcam_file *param)
{
	sprd_dcam_reg_isr(param->idx, DCAM_PATH0_DONE, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_PATH0_OV, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_ISP_OV, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_MIPI_OV, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_SN_LINE_ERR, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_SN_FRAME_ERR, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_JPEG_BUF_OV, NULL, param);
	sprd_dcam_reg_isr(param->idx, DCAM_SN_EOF, NULL, param);

	return 0;
}

static int sprd_img_path_cfg_output_addr(enum dcam_id idx,
					 enum dcam_path_index path_index,
					 path_cfg_func path_cfg,
					 struct dcam_path_spec *path_spec)
{
	int ret = DCAM_RTN_SUCCESS;
	struct dcam_img_buf_addr *cur_node;
	struct dcam_img_buf_queue *queue;
	struct dcam_addr frm_addr;

	if (path_cfg == NULL || path_spec == NULL)
		return -EINVAL;

	queue = &path_spec->buf_queue;

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
		ret = path_cfg(idx, path_index, DCAM_PATH_OUTPUT_ADDR,
			       &frm_addr);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	}

exit:
	return ret;
}

static int sprd_img_path0_cfg(enum dcam_id idx, path_cfg_func path_cfg,
			      struct dcam_path_spec *path_spec)
{
	int ret = DCAM_RTN_SUCCESS;
	unsigned int param;
	enum dcam_path_index path_index = DCAM_PATH_IDX_0;

	if (path_cfg == NULL || path_spec == NULL)
		return -EINVAL;

	if (path_spec->is_from_isp)
		param = DCAM_PATH_FROM_ISP;
	else
		param = DCAM_PATH_FROM_CAP;

	ret = path_cfg(idx, path_index, DCAM_PATH_INPUT_SIZE,
		       &path_spec->in_size);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = path_cfg(idx, path_index, DCAM_PATH_OUTPUT_SIZE,
			   &path_spec->out_size);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}
	ret = path_cfg(idx, path_index, DCAM_PATH_INPUT_RECT,
		       &path_spec->in_rect);

	ret = path_cfg(idx, path_index, DCAM_PATH_OUTPUT_FORMAT,
		       &path_spec->out_fmt);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = path_cfg(idx, path_index, DCAM_PATH_FRAME_BASE_ID,
		       &path_spec->frm_id_base);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = path_cfg(idx, path_index, DCAM_PATH_FRAME_TYPE,
		       &path_spec->frm_type);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = path_cfg(idx, path_index, DCAM_PATH_DATA_ENDIAN,
		       &path_spec->end_sel);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = path_cfg(idx, path_index, DCAM_PATH_FRM_DECI,
		       &path_spec->path_frm_deci);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = path_cfg(idx, path_index, DCAM_PDAF_CONTROL,
		       &path_spec->pdaf_ctrl);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = sprd_img_path_cfg_output_addr(idx, path_index, path_cfg,
					    path_spec);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = path_cfg(idx, path_index, DCAM_PATH_OUTPUT_RESERVED_ADDR,
		       &path_spec->frm_reserved_addr);

	param = 1;
	ret = path_cfg(idx, path_index, DCAM_PATH_ENABLE, &param);

exit:
	return ret;
}

static int sprd_img_path_cfg(enum dcam_id idx, enum dcam_path_index path_index,
			     path_cfg_func path_cfg,
			     struct dcam_path_spec *path_spec)
{
	int ret = DCAM_RTN_SUCCESS;
	unsigned int param;

	if (path_cfg == NULL || path_spec == NULL)
		return -EINVAL;

	DCAM_TRACE("%s, idx: %d path_index: %d\n", __func__, idx, path_index);

	if (dcam_cowork_flag) {
		param = DCAM_PATH_FROM_ISP;
		ret = path_cfg(DCAM_ID_0, path_index, DCAM_PATH_SRC_SEL,
					&param);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
		ret = path_cfg(DCAM_ID_1, path_index, DCAM_PATH_SRC_SEL,
					&param);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	} else {
		if (path_spec->is_from_isp)
			param = DCAM_PATH_FROM_ISP;
		else
			param = DCAM_PATH_FROM_CAP;

		ret = path_cfg(idx, path_index, DCAM_PATH_SRC_SEL, &param);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	}

	if (dcam_cowork_flag) {
		ret = path_cfg(DCAM_ID_0, path_index, DCAM_PATH_ROT_MODE,
			&path_spec->rot_mode);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
		ret = path_cfg(DCAM_ID_1, path_index, DCAM_PATH_ROT_MODE,
		       &path_spec->rot_mode);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	} else {
		ret = path_cfg(idx, path_index, DCAM_PATH_ROT_MODE,
		       &path_spec->rot_mode);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	}

	if (dcam_cowork_flag) {
		ret = path_cfg(DCAM_ID_0, path_index, DCAM_PATH_INPUT_SIZE,
			       &path_spec->in_size);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
		ret = path_cfg(DCAM_ID_1, path_index, DCAM_PATH_INPUT_SIZE,
			       &path_spec->in_size);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	} else {
		ret = path_cfg(idx, path_index, DCAM_PATH_INPUT_SIZE,
			       &path_spec->in_size);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	}

	if (dcam_cowork_flag) {
		ret = path_cfg(DCAM_ID_0, path_index, DCAM_PATH_INPUT_RECT,
			       &path_spec->in_rect);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
		ret = path_cfg(DCAM_ID_1, path_index, DCAM_PATH_INPUT_RECT,
			       &path_spec->in_rect);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	} else {
		ret = path_cfg(idx, path_index, DCAM_PATH_INPUT_RECT,
			       &path_spec->in_rect);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	}

	if (dcam_cowork_flag) {
		ret = path_cfg(DCAM_ID_0, path_index,
			       DCAM_PATH_OUTPUT_SIZE,
			       &path_spec->out_size);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
		ret = path_cfg(DCAM_ID_1, path_index,
			       DCAM_PATH_OUTPUT_SIZE,
			       &path_spec->out_size);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	} else {
		ret = path_cfg(idx, path_index, DCAM_PATH_OUTPUT_SIZE,
			       &path_spec->out_size);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	}

	if (dcam_cowork_flag) {
		ret = path_cfg(DCAM_ID_0, path_index,
			       DCAM_PATH_OUTPUT_FORMAT,
			       &path_spec->out_fmt);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
		ret = path_cfg(DCAM_ID_1, path_index,
			       DCAM_PATH_OUTPUT_FORMAT,
			       &path_spec->out_fmt);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	} else {
		ret = path_cfg(idx, path_index, DCAM_PATH_OUTPUT_FORMAT,
		       &path_spec->out_fmt);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	}

	ret = path_cfg(idx, path_index, DCAM_PATH_FRAME_BASE_ID,
		       &path_spec->frm_id_base);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = path_cfg(idx, path_index, DCAM_PATH_FRAME_TYPE,
		       &path_spec->frm_type);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	if (dcam_cowork_flag) {
		ret = path_cfg(DCAM_ID_0, path_index, DCAM_PATH_DATA_ENDIAN,
			       &path_spec->end_sel);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
		ret = path_cfg(DCAM_ID_1, path_index, DCAM_PATH_DATA_ENDIAN,
			       &path_spec->end_sel);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	} else {
		ret = path_cfg(idx, path_index, DCAM_PATH_DATA_ENDIAN,
			       &path_spec->end_sel);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	}

	if (dcam_cowork_flag) {
		ret = path_cfg(DCAM_ID_0, path_index, DCAM_PATH_FRM_DECI,
			       &path_spec->path_frm_deci);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
		ret = path_cfg(DCAM_ID_1, path_index, DCAM_PATH_FRM_DECI,
			       &path_spec->path_frm_deci);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	} else {
		ret = path_cfg(idx, path_index, DCAM_PATH_FRM_DECI,
			       &path_spec->path_frm_deci);
		if (unlikely(ret)) {
			pr_err("%s err, code %d", __func__, ret);
			goto exit;
		}
	}

	ret = sprd_img_path_cfg_output_addr(idx, path_index, path_cfg,
					    path_spec);
	if (unlikely(ret)) {
		pr_err("%s err, code %d", __func__, ret);
		goto exit;
	}

	ret = path_cfg(idx, path_index, DCAM_PATH_OUTPUT_RESERVED_ADDR,
		       &path_spec->frm_reserved_addr);

	if (dcam_cowork_flag) {
		ret = path_cfg(DCAM_ID_0, path_index, DCAM_PATH_SHRINK,
			       &path_spec->regular_desc);
		ret = path_cfg(DCAM_ID_1, path_index, DCAM_PATH_SHRINK,
			       &path_spec->regular_desc);
	} else {
		ret = path_cfg(idx, path_index, DCAM_PATH_SHRINK,
			       &path_spec->regular_desc);
	}

	if (dcam_cowork_flag) {
		ret = path_cfg(DCAM_ID_0, path_index, DCAM_PATH_JPEGLS,
		       &path_spec->jpegls_desc);
		ret = path_cfg(DCAM_ID_1, path_index, DCAM_PATH_JPEGLS,
		       &path_spec->jpegls_desc);
	} else {
		ret = path_cfg(idx, path_index, DCAM_PATH_JPEGLS,
			       &path_spec->jpegls_desc);
	}

	param = 1;
	if (dcam_cowork_flag) {
		ret = path_cfg(DCAM_ID_0, path_index, DCAM_PATH_ENABLE, &param);
		ret = path_cfg(DCAM_ID_1, path_index, DCAM_PATH_ENABLE, &param);
	} else {
		ret = path_cfg(idx, path_index, DCAM_PATH_ENABLE, &param);
	}

exit:
	return ret;
}

static int sprd_img_local_deinit(struct dcam_dev *dev)
{
	int i;
	struct dcam_path_spec *path;

	for (i = 0; i < DCAM_PATH_MAX; i++) {
		path = &dev->dcam_cxt.dcam_path[i];
		DCAM_TRACE("local_deinit, path %d cnt %d\n",
			   i, path->frm_cnt_act);
		if (unlikely(NULL == dev || NULL == path))
			return -EINVAL;

		path->is_work = 0;
		path->frm_cnt_act = 0;
		sprd_img_buf_queue_init(&path->buf_queue);
	}

	DCAM_TRACE("local_deinit, frm_cnt_act %d\n",
		   path->frm_cnt_act);

	return 0;
}

static int sprd_img_get_path_index(unsigned int channel_id)
{
	int path_index;

	switch (channel_id) {
	case DCAM_PATH0:
		path_index = DCAM_PATH_IDX_0;
		break;
	case DCAM_PATH1:
		path_index = DCAM_PATH_IDX_1;
		break;
	case DCAM_PATH2:
		path_index = DCAM_PATH_IDX_2;
		break;
	case DCAM_PATH3:
		path_index = DCAM_PATH_IDX_3;
		break;
	default:
		path_index = DCAM_PATH_IDX_NONE;
		pr_err("failed to get path index, channel %d\n", channel_id);
	}

	return path_index;
}

#if defined(DUMP_ISP_OUT_YUV) || defined(DUMP_CPP_OUT_YUV)
void sprd_img_write_image_to_file(unsigned int buffer,
	unsigned int size, const char *file)
{
	struct file *wfp = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;
	int err = 0;
	unsigned int i = 0;
	unsigned int write_size = 0, start_addr = 0;
	char *pmap = NULL;

	/* Write imag to a file.*/
	pr_debug("write to image:%s, buf=0x%x, size=%d",
		file, buffer, size);

	wfp = filp_open(file, O_CREAT|O_RDWR, 0666);
	if (IS_ERR(wfp)) {
		pr_err("error occurred while opening file %s.\n", file);
		return;
	}
	old_fs = get_fs();
	set_fs(get_ds());
	for (i = 0; i < size; i += 0x1000) {
		write_size =
			((size - i) > 0x1000) ? 0x1000 : (size - i);
		start_addr = buffer + i;
		offset = wfp->f_pos;
		pmap  = ioremap_nocache(start_addr, 0x1000);
		err = vfs_write(wfp, (char *)pmap, write_size, &offset);
		if (err < 0)
			pr_err("error vfs_write\n");
		wfp->f_pos = offset;
		iounmap(pmap);
	}
	set_fs(old_fs);
	if (wfp)
		filp_close(wfp, NULL);
	wfp = NULL;
}
#endif

int zoom_thread_flag_get(int index)
{
	return atomic_read(&global_zoom_thread_flag[index]);
}

void zoom_thread_flag_set(int index, int val)
{
	atomic_set(&global_zoom_thread_flag[index], val);
}

static int sprd_img_zoom_thread_loop(void *arg)
{
	struct dcam_file *dcamfile = NULL;
	struct cpp_dev *cpp_dev = NULL;
	struct dcam_dev *dcam_dev = NULL;
	struct isp_if_context *ispif = NULL;
	struct isp_pipe_dev *isp_pipeline = NULL;
	struct isp_path_info *isp_path = NULL;
	struct isp_img_info *img_info = NULL;
	struct isp_img_buf src_img_buf;
	struct isp_img_buf dst_img_buf1;
	struct isp_img_buf dst_img_buf2;
	struct isp_img_buf *p_dst_img_buf1 = NULL;
	struct isp_img_buf *p_dst_img_buf2 = NULL;

	struct dcam_node node;
	struct cpp_reqnode cpp_node;

	struct sprd_dcam_img_frm src_img_frm;
	struct sprd_dcam_img_frm dst_img_frm1;
	struct sprd_dcam_img_frm dst_img_frm2;
	struct sprd_dcam_img_frm *p_dst_img_frm1 = NULL;
	struct sprd_dcam_img_frm *p_dst_img_frm2 = NULL;

	struct dcam_path_spec *dcam_path = NULL;
	struct dcam_path_spec *dcam_path1 = NULL;
	struct dcam_path_spec *dcam_path2 = NULL;

	int cpp_path_id = 0;
	int ret = DCAM_RTN_SUCCESS;
	unsigned int isp_id = 0;
	unsigned int img_id = 0;
	unsigned int is_reserved_buffer1 = 0;
	unsigned int is_reserved_buffer2 = 0;
	unsigned int dst_img_num = 0;
	unsigned int cpp_index = 0;

#if defined(DUMP_ISP_OUT_YUV) || defined(DUMP_CPP_OUT_YUV)
	char s_out_file_name[128];
#endif

#if defined(DUMP_ISP_OUT_YUV)
	static unsigned int index0[PIPE_COUNT_MAX] = {0};
	static unsigned int index01[PIPE_COUNT_MAX] = {0};
#endif

#if defined(DUMP_CPP_OUT_YUV)
	static unsigned int index1;
	static unsigned int index2;
	static unsigned int index3;
#endif

	pr_info("%s enter\n", __func__);

	cpp_dev = (struct cpp_dev *)arg;
	if (cpp_dev == NULL) {
		pr_err("error cpp dev is NULL.\n");
		return -EFAULT;
	}

	while (1) {
		if (wait_for_completion_interruptible(
					&cpp_dev->zoom_thread_com) == 0) {
			if (cpp_dev->is_zoom_thread_stop) {
				pr_info("stop zoom thread\n");
				break;
			}

			if (cpp_dev->is_zoom_thread_pause) {
				usleep_range(9000, 10000);
				continue;
			}

			if (sprd_cam_queue_read(&cpp_dev->cpp_queue,
				&cpp_node)) {
				pr_info("cpp_queue empty\n");
				continue;
			}

			dcamfile = cpp_node.dcamfile;
			if (dcamfile == NULL) {
				pr_err("error dcamfile is NULL.\n");
				return -EFAULT;
			}
			dcam_dev = get_dcam_dev(dcamfile);
			if (dcam_dev == NULL) {
				pr_err("dcam_dev is NULL\n");
				return -EFAULT;
			}

			dcam_path = &dcam_dev->dcam_cxt.dcam_path
				[sprd_img_get_channel_id(cpp_node.img_id)];
			if (dcam_path == NULL) {
				pr_err("dcam_path is NULL\n");
				return -EFAULT;
			}
			dcam_path1 = &dcam_dev->dcam_cxt.dcam_path[DCAM_PATH1];
			dcam_path2 = &dcam_dev->dcam_cxt.dcam_path[DCAM_PATH2];

			ispif = sprd_isp_get_isp_handle(cpp_node.isp_id);
			if (ispif == NULL) {
				pr_err("ispif is NULL.\n");
				return -EFAULT;
			}

			if (atomic_read(&ispif->isp_if_existed) == 0)  {
				/*isp_if will release*/
				pr_info("isp is not existed, don't start cpp!\n");
				goto zoom_end;
			}
			zoom_thread_flag_set(cpp_index, 1);
			isp_pipeline = ispif->isp_pipe;
			if (isp_pipeline == NULL) {
				pr_err("isp_pipeline is null error\n");
				zoom_thread_flag_set(cpp_index, 0);
				return -EINVAL;
			}
			isp_id = cpp_node.isp_id;
			img_id = cpp_node.img_id;
			isp_path = &isp_pipeline->isp_path;
			img_info = &isp_path->img_info[img_id];
			memset(&src_img_buf, 0,
				sizeof(struct isp_img_buf));
			memset(&dst_img_buf1, 0,
				sizeof(struct isp_img_buf));
			memset(&dst_img_buf2, 0,
				sizeof(struct isp_img_buf));
			memset(&src_img_frm, 0,
				sizeof(struct sprd_dcam_img_frm));
			memset(&dst_img_frm1, 0,
				sizeof(struct sprd_dcam_img_frm));
			memset(&dst_img_frm2, 0,
				sizeof(struct sprd_dcam_img_frm));

			ret = _isp_img_buf_queue_read(&img_info->frame_queue,
				&src_img_buf);
			if (ret) {
				pr_err("isp%d img_id%d read frame_queue error!\n",
					isp_id, img_id);
				goto zoom_end;
			} else {
				/* set source img entity */
				src_img_frm.addr_phy.mfd[0] =
					src_img_buf.img_y_fd;
				src_img_frm.addr_phy.mfd[1] =
					src_img_buf.img_u_fd;
				src_img_frm.addr_phy.mfd[2] =
					src_img_buf.img_v_fd;

				src_img_frm.addr_phy.y = src_img_buf.iova_yaddr;
				src_img_frm.addr_phy.u = src_img_buf.iova_uaddr;
				src_img_frm.addr_phy.v = src_img_buf.iova_vaddr;

				/* need fixed */
				src_img_frm.size.w = img_info->out_size.w;
				src_img_frm.size.h = img_info->out_size.h;
				src_img_frm.format_pattern = SCALE_YUV420;

				src_img_frm.rect.x = dcam_path->cpp_in_rect.x;
				src_img_frm.rect.y = dcam_path->cpp_in_rect.y;
				src_img_frm.rect.w = dcam_path->cpp_in_rect.w;
				src_img_frm.rect.h = dcam_path->cpp_in_rect.h;

				src_img_frm.data_end.y_endian =
					SCALE_ENDIAN_LITTLE;
				src_img_frm.data_end.uv_endian =
					SCALE_ENDIAN_HALFBIG;
			}

			if (ispif->last_isp_write_buf[img_id] !=
			    src_img_buf.img_y_fd) {
				pr_info("discard this frm1:%d 0x%x!\n", img_id,
					src_img_buf.img_y_fd);
				goto zoom_end;
			}

			dst_img_num = 0;
			is_reserved_buffer1 = 0;
			is_reserved_buffer2 = 0;
			p_dst_img_frm1 = NULL;
			p_dst_img_frm2 = NULL;
			p_dst_img_buf1 = NULL;
			p_dst_img_buf2 = NULL;
			ret = _isp_img_buf_queue_read(&img_info->in_buff_queue,
				&dst_img_buf1);
			if (ret) {
				pr_info("isp%d img%d q empty\n",
				isp_id, img_id);
				is_reserved_buffer1 = 1;
			} else {
				if (dst_img_buf1.img_y_fd ==
					img_info->reserved_buf[0].img_y_fd) {
					is_reserved_buffer1 = 1;
				}
			}

			/*set destination img entity*/
			if (is_reserved_buffer1 == 0) {
				dst_img_frm1.addr_phy.mfd[0] =
					dst_img_buf1.img_y_fd;
				dst_img_frm1.addr_phy.mfd[1] =
					dst_img_buf1.img_u_fd;
				dst_img_frm1.addr_phy.mfd[2] =
					dst_img_buf1.img_v_fd;

				dst_img_frm1.addr_phy.y =
					dst_img_buf1.iova_yaddr;
				dst_img_frm1.addr_phy.u =
					dst_img_buf1.iova_uaddr;
				dst_img_frm1.addr_phy.v =
					dst_img_buf1.iova_vaddr;

				dst_img_frm1.size.w = dcam_path->out_size.w;
				dst_img_frm1.size.h = dcam_path->out_size.h;

				dst_img_frm1.format_pattern = SCALE_YUV420;
				dst_img_frm1.rect.x = 0;
				dst_img_frm1.rect.y = 0;
				dst_img_frm1.rect.w = dcam_path->out_size.w;
				dst_img_frm1.rect.h = dcam_path->out_size.h;
				dst_img_frm1.data_end.y_endian =
					SCALE_ENDIAN_LITTLE;
				dst_img_frm1.data_end.uv_endian =
					SCALE_ENDIAN_HALFBIG;

				p_dst_img_frm1 = &dst_img_frm1;
				p_dst_img_buf1 = &dst_img_buf1;
				dst_img_num++;
			}

			/*dump isp output yuv data*/
			#if defined(DUMP_ISP_OUT_YUV)
			if (img_info->frm_id_base == 0x4000
				&& ((index0[isp_id]++ % 10) == 5
				&& index0[isp_id] < 10)) {
				pr_info("save snapshot isp%d output data to file e.\n",
					isp_id);
				snprintf(s_out_file_name,
						sizeof(s_out_file_name),
						"%s.%dx%d_%02d.y",
						"/data/mlog/isp_snaptshot",
						src_img_frm.size.w,
						src_img_frm.size.h,
						index0[isp_id]);
				sprd_img_write_image_to_file(
						src_img_frm.addr_phy.y,
						src_img_frm.size.w *
						src_img_frm.size.h,
						s_out_file_name);
				snprintf(s_out_file_name,
						sizeof(s_out_file_name),
						"%s.%dx%d_%02d.uv",
						"/data/mlog/isp_snaptshot",
						src_img_frm.size.w,
						src_img_frm.size.h,
						index0[isp_id]);
				sprd_img_write_image_to_file(
						src_img_frm.addr_phy.u,
						src_img_frm.size.w *
						src_img_frm.size.h/2,
						s_out_file_name);
				pr_info("save snapshot isp%d output data to file x.\n",
					isp_id);
			}

			if (img_info->frm_id_base == 0x1000
				&& ((index01[isp_id]++ % 10) == 3
				&& index01[isp_id] < 10)) {
				pr_info("save preview isp%d output data to file e.\n",
					isp_id);
				snprintf(s_out_file_name,
						sizeof(s_out_file_name),
						"%s.%dx%d_%02d.y",
						"/data/mlog/isp_preview",
						src_img_frm.size.w,
						src_img_frm.size.h,
						index01[isp_id]);
				sprd_img_write_image_to_file(
						src_img_frm.addr_phy.y,
						src_img_frm.size.w *
						src_img_frm.size.h,
						s_out_file_name);
				snprintf(s_out_file_name,
						sizeof(s_out_file_name),
						"%s.%dx%d_%02d.uv",
						"/data/mlog/isp_preview",
						src_img_frm.size.w,
						src_img_frm.size.h,
						index01[isp_id]);
				sprd_img_write_image_to_file(
						src_img_frm.addr_phy.u,
						src_img_frm.size.w *
						src_img_frm.size.h/2,
						s_out_file_name);

				pr_info("save preiew isp%d output data to file x.\n",
					isp_id);
			}
			#endif

			if (cpp_node.img_id == ISP_PATH_VIDEO &&
					dcam_path2->is_work &&
					dcam_path1->is_work) {
				is_reserved_buffer2 = 0;
				img_info = &isp_path->
					img_info[ISP_PATH_PREIVEW];
				ret = _isp_img_buf_queue_read(
						&img_info->in_buff_queue,
						&dst_img_buf2);
				if (ret) {
					pr_info("q empty\n");
					is_reserved_buffer2 = 1;
				} else {
					if (dst_img_buf2.img_y_fd ==
							img_info->
							reserved_buf[0].
							img_y_fd) {
						is_reserved_buffer2 = 1;
					}
				}

				/*set destination img entity*/
				if (is_reserved_buffer2 == 0) {
					if (dst_img_num == 0) {
						src_img_frm.rect.x =
						dcam_path1->cpp_in_rect.x;
						src_img_frm.rect.y =
						dcam_path1->cpp_in_rect.y;
						src_img_frm.rect.w =
						dcam_path1->cpp_in_rect.w;
						src_img_frm.rect.h =
						dcam_path1->cpp_in_rect.h;
					} else {
						src_img_frm.rect2.x =
						dcam_path1->cpp_in_rect.x;
						src_img_frm.rect2.y =
						dcam_path1->cpp_in_rect.y;
						src_img_frm.rect2.w =
						dcam_path1->cpp_in_rect.w;
						src_img_frm.rect2.h =
						dcam_path1->cpp_in_rect.h;
					}

					dst_img_frm2.addr_phy.mfd[0] =
						dst_img_buf2.img_y_fd;
					dst_img_frm2.addr_phy.mfd[1] =
						dst_img_buf2.img_u_fd;
					dst_img_frm2.addr_phy.mfd[2] =
						dst_img_buf2.img_v_fd;

					dst_img_frm2.addr_phy.y =
						dst_img_buf2.iova_yaddr;
					dst_img_frm2.addr_phy.u =
						dst_img_buf2.iova_uaddr;
					dst_img_frm2.addr_phy.v =
						dst_img_buf2.iova_vaddr;

					dst_img_frm2.size.w =
						dcam_path1->out_size.w;
					dst_img_frm2.size.h =
						dcam_path1->out_size.h;

					dst_img_frm2.format_pattern =
						SCALE_YUV420;
					dst_img_frm2.rect.x = 0;
					dst_img_frm2.rect.y = 0;
					dst_img_frm2.rect.w =
						dcam_path1->out_size.w;
					dst_img_frm2.rect.h =
						dcam_path1->out_size.h;
					dst_img_frm2.data_end.y_endian =
						SCALE_ENDIAN_LITTLE;
					dst_img_frm2.data_end.uv_endian =
						SCALE_ENDIAN_HALFBIG;

					p_dst_img_frm2 = &dst_img_frm2;
					p_dst_img_buf2 = &dst_img_buf2;
					dst_img_num++;
				}
				if (dst_img_num == 2)
					cpp_path_id = CPP_SENARIO_PATH2_PATH3;
				else if (dst_img_num == 1)
					cpp_path_id = CPP_SENARIO_PATH2;
			} else {
				cpp_path_id = CPP_SENARIO_SCALE;
			}

			if (dst_img_num == 0)
				goto zoom_end;

			if (dst_img_num == 1 &&
				p_dst_img_frm1 == NULL &&
				p_dst_img_frm2 != NULL) {
				p_dst_img_frm1 = p_dst_img_frm2;
				p_dst_img_frm2 = NULL;
				p_dst_img_buf1 = p_dst_img_buf2;
				p_dst_img_buf2 = NULL;
			}

			ret = cpp_k_start_scale(&src_img_frm, p_dst_img_frm1,
					p_dst_img_frm2, &cpp_path_id);
			if (!ret) {
				ret = cpp_k_stop_scale(cpp_path_id);
				if (ret)
					pr_err("error cpp_k_stop_scale failed.\n");
			} else {
				pr_err("error cpp_k_start_scale failed.\n");
				goto zoom_end;
			}

			if (ispif->cur_isp_write_buf[img_id] ==
				src_img_buf.img_y_fd) {
				if (is_reserved_buffer1 == 0) {
					pr_info("discard this frm2:%d 0x%x!\n",
						img_id, src_img_buf.img_y_fd);
					img_info = &isp_path->img_info[img_id];
					_isp_img_buf_queue_unread(
						&img_info->in_buff_queue);
				}
				if (cpp_node.img_id == ISP_PATH_VIDEO &&
					dcam_path2->is_work &&
					dcam_path1->is_work &&
					is_reserved_buffer2 == 0) {
					img_info = &isp_path->
						img_info[ISP_PATH_PREIVEW];
					ret = _isp_img_buf_queue_unread(
						&img_info->in_buff_queue);
				}
				goto zoom_end;
			}

			#if defined(DUMP_CPP_OUT_YUV)
			if (p_dst_img_frm2 != NULL &&
				p_dst_img_buf2 != NULL &&
				p_dst_img_buf2->base_id == 0x8000 &&
					(index1 < 1)) {
				pr_info("save video cpp output data to file e.\n");
				snprintf(s_out_file_name,
						sizeof(s_out_file_name),
						"%s.%dx%d_%02d.y",
						"/data/mlog/cpp_video",
						p_dst_img_frm2->size.w,
						p_dst_img_frm2->size.h, index1);
				sprd_img_write_image_to_file(
						p_dst_img_frm2->addr_phy.y,
						p_dst_img_frm2->size.w *
						p_dst_img_frm2->size.h,
						s_out_file_name);
				snprintf(s_out_file_name,
						sizeof(s_out_file_name),
						"%s.%dx%d_%02d.uv",
						"/data/mlog/cpp_video",
						p_dst_img_frm2->size.w,
						p_dst_img_frm2->size.h,
						index1++);
				sprd_img_write_image_to_file(
						p_dst_img_frm2->addr_phy.u,
						p_dst_img_frm2->size.w *
						p_dst_img_frm2->size.h/2,
						s_out_file_name);
				 pr_info("save snapshot cpp output data to file x.\n");
			}

			if (img_info->frm_id_base == 0x4000 &&
				(index2 < 10)) {
				pr_info("save snapshot cpp output data to file e.\n");
				snprintf(s_out_file_name,
						sizeof(s_out_file_name),
						"%s.%dx%d_%02d.y",
						"/data/mlog/cpp_snapshot",
						dst_img_frm1.size.w,
						dst_img_frm1.size.h, index2);
				sprd_img_write_image_to_file(
						dst_img_frm1.addr_phy.y,
						dst_img_frm1.size.w *
						dst_img_frm1.size.h,
						s_out_file_name);
				snprintf(s_out_file_name,
						sizeof(s_out_file_name),
						"%s.%dx%d_%02d.uv",
						"/data/mlog/cpp_snapshot",
						dst_img_frm1.size.w,
						dst_img_frm1.size.h, index2++);
				sprd_img_write_image_to_file(
						dst_img_frm1.addr_phy.u,
						 dst_img_frm1.size.w *
						 dst_img_frm1.size.h/2,
						 s_out_file_name);
				 pr_info("save snapshot cpp output data to file x.\n");
			}

			if (img_info->frm_id_base == 0x1000 &&
					(index3 < 10)) {
				pr_info("save preview cpp output data to file e.\n");
				snprintf(s_out_file_name,
						sizeof(s_out_file_name),
						"%s.%dx%d_%02d.y",
						"/data/mlog/cpp_preview",
						dst_img_frm1.size.w,
						dst_img_frm1.size.h, index3);
				sprd_img_write_image_to_file(dst_img_frm1.
						 addr_phy.y,
						 dst_img_frm1.size.w *
						 dst_img_frm1.size.h,
						 s_out_file_name);
				 snprintf(s_out_file_name,
						 sizeof(s_out_file_name),
						 "%s.%dx%d_%02d.uv",
						 "/data/mlog/cpp_preview",
						 dst_img_frm1.size.w,
						 dst_img_frm1.size.h, index3++);
				sprd_img_write_image_to_file(dst_img_frm1.
						 addr_phy.u,
						 dst_img_frm1.size.w *
						 dst_img_frm1.size.h/2,
						 s_out_file_name);
				pr_info("save preview cpp output data to file x.\n");
			}
			#endif
			/* pr_info("dst_img_num=%d\n", dst_img_num); */
			if (p_dst_img_frm1 != NULL) {
				memset((void *)&node, 0,
					sizeof(struct dcam_node));
				node.irq_type = ISP_IRQ_IMG;
				node.irq_flag = ISP_IMG_TX_DONE;
				node.index = 0;
				node.height = p_dst_img_buf1->buf_size.h;
				node.length = 0;

				node.frm_base_id = p_dst_img_buf1->base_id;
				node.channel_id = p_dst_img_buf1->img_id;
				node.format = p_dst_img_buf1->format;
				node.yaddr = p_dst_img_buf1->yaddr;
				node.uaddr = p_dst_img_buf1->uaddr;
				node.vaddr = p_dst_img_buf1->vaddr;
				node.yaddr_vir = p_dst_img_buf1->yaddr_vir;
				node.uaddr_vir = p_dst_img_buf1->uaddr_vir;
				node.vaddr_vir = p_dst_img_buf1->vaddr_vir;
				node.mfd[0] = p_dst_img_buf1->img_y_fd;
				node.mfd[1] = p_dst_img_buf1->img_u_fd;
				node.mfd[2] = p_dst_img_buf1->img_v_fd;

				img_get_timestamp(&node.time);

				node.boot_time = ktime_get_boottime();
				atomic_set(&dcam_dev->run_flag, 1);
				ret = sprd_img_queue_write(
					&dcamfile->queue,
					&node);
				if (ret) {
					pr_err("write isp irq queue error\n");
					zoom_thread_flag_set(cpp_index, 0);
					return -EINVAL;
				}
				complete(&dcamfile->irq_com);
			}

			if (p_dst_img_frm2 != NULL) {
				memset((void *)&node, 0,
					sizeof(struct dcam_node));
				node.irq_type = ISP_IRQ_IMG;
				node.irq_flag = ISP_IMG_TX_DONE;
				node.index = 0;
				node.height = p_dst_img_buf2->buf_size.h;
				node.length = 0;
				node.frm_base_id = p_dst_img_buf2->base_id;
				node.channel_id = p_dst_img_buf2->img_id;
				node.format = p_dst_img_buf2->format;
				node.yaddr = p_dst_img_buf2->yaddr;
				node.uaddr = p_dst_img_buf2->uaddr;
				node.vaddr = p_dst_img_buf2->vaddr;
				node.yaddr_vir = p_dst_img_buf2->yaddr_vir;
				node.uaddr_vir = p_dst_img_buf2->uaddr_vir;
				node.vaddr_vir = p_dst_img_buf2->vaddr_vir;
				node.mfd[0] = p_dst_img_buf2->img_y_fd;
				node.mfd[1] = p_dst_img_buf2->img_u_fd;
				node.mfd[2] = p_dst_img_buf2->img_v_fd;

				img_get_timestamp(&node.time);

				node.boot_time = ktime_get_boottime();
				atomic_set(&dcam_dev->run_flag, 1);
				ret = sprd_img_queue_write(
					&dcamfile->queue,
					&node);
				if (ret) {
					pr_err("write isp irq queue error\n");
					zoom_thread_flag_set(cpp_index, 0);
					return -EINVAL;
				}
				complete(&dcamfile->irq_com);
			}

		}

zoom_end:
		zoom_thread_flag_set(cpp_index, 0);
		continue;
	}
#if defined(DUMP_ISP_OUT_YUV)
	index0[0] = 0;
	index01[0] = 0;
	index0[1] = 0;
	index01[1] = 0;
#endif

	cpp_dev->is_zoom_thread_stop = 0;
	pr_info("exit zoom thread loop: %d\n", cpp_index);
	return 0;
}

static int sprd_img_create_zoom_thread(void *param)
{
	struct cpp_dev *dev = (struct cpp_dev *)param;

	if (dev == NULL) {
		pr_err("cpp dev is NULL\n");
		return -EFAULT;
	}

	dev->is_zoom_thread_stop = 0;
	dev->is_zoom_thread_pause = 0;
	dev->zoom_level = DCAM_ZOOM_LEVEL_MAX + 1;
	init_completion(&dev->zoom_thread_com);

	dev->zoom_thread = kthread_run(sprd_img_zoom_thread_loop,
			param,
			"img_zoom_thread");
	if (IS_ERR(dev->zoom_thread)) {
		pr_err("failed to create zoom thread\n");
		return -EINVAL;
	}

	pr_info("%s exit\n", __func__);
	return 0;
}

static int sprd_img_stop_zoom_thread(void *param)
{
	struct cpp_dev *dev = (struct cpp_dev *)param;

	if (dev == NULL) {
		pr_err("dev is NULL\n");
		return -1;
	}

	DCAM_TRACE("stop zoom thread\n");
	if (dev->zoom_thread) {
		dev->is_zoom_thread_stop = 1;
		complete(&dev->zoom_thread_com);
		if (dev->is_zoom_thread_stop != 0) {
			while (dev->is_zoom_thread_stop)
				usleep_range(1000, 1500);
		}
		dev->zoom_thread = NULL;
	}

	return 0;
}

/* calculate out rect */
static int sprd_img_calc_crop_rect(struct dcam_size in_size,
		struct dcam_rect in_rect, struct dcam_size out_size,
		struct dcam_rect *out_rect)
{
	struct dcam_rect equal_ratio_rect_for_in_size;
	struct dcam_rect equal_ratio_rect_for_out_size;

	pr_info("original in_rect[%d,%d,%d,%d]\n",
			in_rect.x, in_rect.y,
			in_rect.w, in_rect.h);

	/* eg:4:3 < 16:9*/
	if ((in_rect.w*in_size.h) != (in_size.w*in_rect.h)) {
		if ((in_rect.w*in_size.h) >
			(in_size.w*in_rect.h)) {
			equal_ratio_rect_for_in_size.x = 0;
			equal_ratio_rect_for_in_size.w = in_size.w;
			equal_ratio_rect_for_in_size.h = (
				in_size.w * in_rect.h / in_rect.w);
			equal_ratio_rect_for_in_size.y = (in_size.h-
				equal_ratio_rect_for_in_size.h)/2;
		} else {
			equal_ratio_rect_for_in_size.y = 0;
			equal_ratio_rect_for_in_size.h = in_size.h;
			equal_ratio_rect_for_in_size.w = (in_size.h *
					in_rect.w / in_rect.h);
			equal_ratio_rect_for_in_size.x =
					(in_size.w-
					equal_ratio_rect_for_in_size.w)/2;
		}
	} else {
		equal_ratio_rect_for_in_size.x = 0;
		equal_ratio_rect_for_in_size.y = 0;
		equal_ratio_rect_for_in_size.w = in_size.w;
		equal_ratio_rect_for_in_size.h = in_size.h;
	}
	pr_info("equal_ratio_rect_for_in_size x=%d, y=%d, w=%d, h=%d\n",
				equal_ratio_rect_for_in_size.x,
				equal_ratio_rect_for_in_size.y,
				equal_ratio_rect_for_in_size.w,
				equal_ratio_rect_for_in_size.h);

	if ((in_rect.w*out_size.h) != (out_size.w*in_rect.h)) {
		if ((in_rect.w*out_size.h) >
			(out_size.w*in_rect.h)) {
			equal_ratio_rect_for_out_size.x = 0;
			equal_ratio_rect_for_out_size.w = out_size.w;
			equal_ratio_rect_for_out_size.h = (
				out_size.w * in_rect.h / in_rect.w);
			equal_ratio_rect_for_out_size.y = (out_size.h-
				equal_ratio_rect_for_out_size.h)/2;
		} else {
			equal_ratio_rect_for_out_size.y = 0;
			equal_ratio_rect_for_out_size.h = out_size.h;
			equal_ratio_rect_for_out_size.w = (
				out_size.h * in_rect.w / in_rect.h);
			equal_ratio_rect_for_out_size.x = (out_size.w-
				equal_ratio_rect_for_out_size.w)/2;
		}
	} else {
		equal_ratio_rect_for_out_size.x = 0;
		equal_ratio_rect_for_out_size.y = 0;
		equal_ratio_rect_for_out_size.w = out_size.w;
		equal_ratio_rect_for_out_size.h = out_size.h;
	}
	pr_info("equal_ratio_rect_for_out_size x=%d, y=%d, w=%d, h=%d\n",
				equal_ratio_rect_for_out_size.x,
				equal_ratio_rect_for_out_size.y,
				equal_ratio_rect_for_out_size.w,
				equal_ratio_rect_for_out_size.h);

	if (in_rect.x < equal_ratio_rect_for_in_size.x) {
		out_rect->x = ALIGN_SIZE_DOWN(equal_ratio_rect_for_out_size.x,
			DCAM_PIXEL_XOFFSET_ALIGNED);
	} else {
		out_rect->x = ALIGN_SIZE_DOWN((equal_ratio_rect_for_out_size.x +
			((in_rect.x - equal_ratio_rect_for_in_size.x) *
			equal_ratio_rect_for_out_size.w /
			equal_ratio_rect_for_in_size.w)),
			DCAM_PIXEL_XOFFSET_ALIGNED);
	}

	if (in_rect.y < equal_ratio_rect_for_in_size.y) {
		out_rect->y = ALIGN_SIZE_DOWN(equal_ratio_rect_for_out_size.y,
			DCAM_PIXEL_XOFFSET_ALIGNED);
	} else {
		out_rect->y = ALIGN_SIZE_DOWN((equal_ratio_rect_for_out_size.y +
			((in_rect.y - equal_ratio_rect_for_in_size.y) *
			equal_ratio_rect_for_out_size.h /
			equal_ratio_rect_for_in_size.h)),
			DCAM_PIXEL_YOFFSET_ALIGNED);
	}

	if (equal_ratio_rect_for_in_size.w == in_rect.w) {
		out_rect->w = ALIGN_SIZE_DOWN(
			equal_ratio_rect_for_out_size.w,
			DCAM_PIXEL_WIDTH_ALIGNED);
	} else {
		out_rect->w = ALIGN_SIZE_DOWN(((in_rect.w *
			equal_ratio_rect_for_out_size.w)/
			equal_ratio_rect_for_in_size.w),
			DCAM_PIXEL_WIDTH_ALIGNED);
	}

	if (equal_ratio_rect_for_in_size.h == in_rect.h) {
		out_rect->h = ALIGN_SIZE_DOWN(
			equal_ratio_rect_for_out_size.h,
			DCAM_PIXEL_HEIGHT_ALIGNED);
	} else {
		out_rect->h = ALIGN_SIZE_DOWN(((in_rect.h *
			equal_ratio_rect_for_out_size.h)/
			equal_ratio_rect_for_in_size.h),
			DCAM_PIXEL_HEIGHT_ALIGNED);
	}

	pr_info("after out_rect[%d,%d,%d,%d]\n",
				out_rect->x,
				out_rect->y,
				out_rect->w,
				out_rect->h);
	return 0;
}

static int sprd_img_path_pre_proc(unsigned int channel_id,
		struct dcam_path_spec *path,
		struct dcam_info *dcaminfo)
{
	struct dcam_path_spec *path1 = NULL;
	struct dcam_path_spec *path2 = NULL;
	pr_info("original channel_id=%d in_rect[%d,%d,%d,%d]\n",
		channel_id, path->in_rect.x, path->in_rect.y,
		path->in_rect.w, path->in_rect.h);

	if (dcaminfo == NULL) {
		pr_err("dcaminfo is NULL.\n");
		return 0;
	}
	path1 = &dcaminfo->dcam_path[DCAM_PATH1];
	path2 = &dcaminfo->dcam_path[DCAM_PATH2];
	switch (channel_id) {
	case DCAM_PATH3:
		sprd_img_calc_crop_rect(path->in_size,
			path->in_rect,
			path->isp_out_size,
			&path->cpp_in_rect);
		break;

	case DCAM_PATH2:
		if (path1->is_work && path2->is_work) {
			;
		} else {
			sprd_img_calc_crop_rect(path->in_size,
				path->in_rect,
				path->isp_out_size,
				&path->cpp_in_rect);
		}
		break;

	case DCAM_PATH1:
		if (path1->is_work && path2->is_work) {
			if (path1->isp_out_size.w >
				path2->isp_out_size.w) {
				sprd_img_calc_crop_rect(path2->in_size,
					path2->in_rect,
					path->isp_out_size,
					&path2->cpp_in_rect);
				sprd_img_calc_crop_rect(path->in_size,
					path->in_rect,
					path->isp_out_size,
					&path->cpp_in_rect);
			} else {
				sprd_img_calc_crop_rect(path2->in_size,
					path2->in_rect,
					path2->isp_out_size,
					&path2->cpp_in_rect);
				sprd_img_calc_crop_rect(path->in_size,
					path->in_rect,
					path2->isp_out_size,
					&path->cpp_in_rect);
			}
		} else {
			sprd_img_calc_crop_rect(path->in_size,
			path->in_rect,
			path->isp_out_size,
			&path->cpp_in_rect);
		}
		break;
	default:
		break;
	}
	return 0;
}

static int sprd_img_update_video(struct dcam_file *dcamfile,
		unsigned int channel_id)
{
	int ret = DCAM_RTN_SUCCESS;
	struct dcam_path_spec *path = NULL;
	struct dcam_dev *dev = NULL;
	struct dcam_group *cpp_grp = NULL;
	struct cpp_dev *cpp_dev = NULL;
	enum dcam_path_index path_index;

	DCAM_TRACE("update video, channel %d\n", channel_id);

	dev = get_dcam_dev(dcamfile);
	if (dev == NULL) {
		pr_err("error dcam dev is NULL\n");
		return -EFAULT;
	}

	cpp_grp = dcamfile->grp;
	if (cpp_grp == NULL) {
		pr_err("err cpp grp is NULL\n");
		return -EFAULT;
	}

	if (channel_id == DCAM_PATH1 || channel_id == DCAM_PATH2)
		cpp_dev = cpp_grp->cpp_dev[0];
	else
		cpp_dev = cpp_grp->cpp_dev[1];

	mutex_lock(&dev->dcam_mutex);

	path = &dev->dcam_cxt.dcam_path[channel_id];
	path_index = sprd_img_get_path_index(channel_id);

	if (dev->dcam_cxt.is_smooth_zoom && DCAM_PATH0 != channel_id) {
		cpp_dev->zoom_level = 1;
		dev->channel_id = channel_id;
		if (path->in_rect_backup.w == 0 ||
		    path->in_rect_backup.h == 0) {
			path->in_rect_backup.x = 0;
			path->in_rect_backup.y = 0;
			path->in_rect_backup.w = path->in_size.w;
			path->in_rect_backup.h = path->in_size.h;
			memcpy((void *)&path->in_rect_current,
			       (void *)&path->in_rect_backup,
			       sizeof(struct dcam_rect));
		} else {
			memcpy((void *)&path->in_rect_backup,
			       (void *)&path->in_rect_current,
			       sizeof(struct dcam_rect));
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

		ret = sprd_dcam_update_path(dev->idx, path_index,
					    &path->in_size, &path->in_rect,
					    &path->out_size);
		sprd_img_path_pre_proc(channel_id, path,
				&dev->dcam_cxt);
	}

	mutex_unlock(&dev->dcam_mutex);
	DCAM_TRACE("update video 0x%x\n", ret);

	if (ret)
		pr_err("failed to update video 0x%x\n", ret);

	return ret;
}

static int sprd_img_streampause(struct dcam_file *dcamfile,
	unsigned int channel_id, unsigned int reconfig_flag)
{
	struct dcam_dev *dev = get_dcam_dev(dcamfile);
	struct dcam_path_spec *path = NULL;
	int ret = 0;
	enum dcam_path_index path_index;

	DCAM_TRACE("pause, channel %d ,recfg flag %d\n", channel_id,
		   reconfig_flag);

	path = &dev->dcam_cxt.dcam_path[channel_id];
	path_index = sprd_img_get_path_index(channel_id);

	if (path->status == PATH_RUN) {
		path->status = PATH_IDLE;
		ret = sprd_dcam_stop_path(dev->idx, path_index);
		IMG_PRINT_IF_ERR(ret);
		if ((reconfig_flag) /* && (DCAM_PATH2 == channel_id) */) {
			/* path->is_work = 0; */
			path->frm_cnt_act = 0;
			sprd_img_buf_queue_init(&path->buf_queue);
			sprd_img_queue_disable(&dcamfile->queue, channel_id);
		}

		DCAM_TRACE("pause, channel %d done\n", channel_id);
	} else {
		DCAM_TRACE("pause, path %d not running, status %d\n",
			channel_id, path->status);
	}

	return ret;
}

static int sprd_img_streamresume(struct dcam_file *dcamfile,
	unsigned int channel_id)
{
	struct dcam_dev *dev = get_dcam_dev(dcamfile);
	struct dcam_path_spec *path = NULL;
	enum dcam_path_index path_index;
	path_cfg_func path_cfg;
	int ret = 0;
	int on_flag = 1;

	DCAM_TRACE("resume, channel %d\n", channel_id);

	if (!dev) {
		pr_err("dev is NULL\n");
		return -EFAULT;
	}

	path = &dev->dcam_cxt.dcam_path[channel_id];
	path_index = sprd_img_get_path_index(channel_id);

	if (unlikely(atomic_read(&dev->stream_on) == 0)) {
		pr_info("resume stream not on\n");
		ret = sprd_img_local_deinit(dev);
		if (unlikely(ret))
			pr_err("%s error", __func__);

		on_flag = 0;
	}

	if (path->status == PATH_IDLE && on_flag) {
		if (path->is_work) {
			if (channel_id == DCAM_PATH0) {
				path_cfg = sprd_dcam_path_cfg;
			} else if (channel_id == DCAM_PATH1) {
				path_cfg = sprd_dcam_path_cfg;
			} else if (channel_id == DCAM_PATH2) {
				IMG_PRINT_IF_ERR(ret);
				path_cfg = sprd_dcam_path_cfg;
			} else if (channel_id == DCAM_PATH3) {
				path_cfg = sprd_dcam_path_cfg;
			} else {
				pr_info("resume, invalid channel %d\n",
					channel_id);
				return -EINVAL;
			}

			if (channel_id == DCAM_PATH0) {
				ret = sprd_img_path0_cfg(dev->idx, path_cfg,
							 path);
			} else {
				ret = sprd_img_path_cfg(dev->idx, path_index,
							path_cfg, path);
			}
			sprd_img_queue_enable(&dcamfile->queue, channel_id);
			if (unlikely(ret)) {
				pr_err("%s err, code %d", __func__, ret);
				goto exit;
			}
			ret = sprd_dcam_start_path(dev->idx, path_index);
			IMG_PRINT_IF_ERR(ret);

			path->status = PATH_RUN;
		} else {
			DCAM_TRACE("resume, path %d is_work %d, can't resume\n",
				   channel_id, path->is_work);
		}
	} else {
		DCAM_TRACE("resume, path %d status %d, can't resume\n",
			   channel_id, path->status);
	}
exit:
	if (ret) {
		pr_err("failed to resume, path %d, ret 0x%x\n",
			   channel_id, ret);
	}

	return ret;
}

static void sprd_img_print_reg(enum dcam_id idx)
{
	unsigned int *reg_buf = NULL;
	unsigned int reg_buf_len = 0x400 * 4;
	int ret;
	unsigned int print_len = 0, print_cnt = 0;

	reg_buf = vzalloc(reg_buf_len);
	if (reg_buf == NULL)
		return;

	ret = sprd_dcam_read_registers(idx, reg_buf, &reg_buf_len);
	if (ret) {
		vfree(reg_buf);
		return;
	}

	while (print_len < reg_buf_len) {
		pr_info("offset 0x%03x : 0x%08x, 0x%08x, 0x%08x, 0x%08x\n",
			print_len,
			reg_buf[print_cnt],
			reg_buf[print_cnt + 1],
			reg_buf[print_cnt + 2], reg_buf[print_cnt + 3]);
		print_cnt += 4;
		print_len += 16;
	}

	udelay(1);
	vfree(reg_buf);
}

static void sprd_timer_callback(unsigned long data)
{
	struct dcam_file *dcamfile  = (struct dcam_file *)data;

	struct dcam_dev *dev = NULL;
	struct dcam_node node;
	int ret = 0;

	dev = get_dcam_dev(dcamfile);
	if (data == 0 || atomic_read(&dev->stream_on) == 0) {
		pr_err("timer cb error\n");
		return;
	}

	if (atomic_read(&dev->run_flag) == 0) {
		pr_err("DCAM%d timeout.\n", dev->idx);
		node.irq_flag = IMG_TIMEOUT;
		node.invalid_flag = 0;
		ret = sprd_img_queue_write(&dcamfile->queue, &node);
		if (ret)
			pr_err("timer cb write queue error\n");

		complete(&dcamfile->irq_com);
	}
}

static int sprd_init_timer(struct timer_list *dcam_timer, unsigned long data)
{
	setup_timer(dcam_timer, sprd_timer_callback, data);

	return 0;
}

static int sprd_start_timer(struct timer_list *dcam_timer,
			    unsigned int time_val)
{

	DCAM_TRACE("starting timer %ld\n", jiffies);
	mod_timer(dcam_timer, jiffies + msecs_to_jiffies(time_val));

	return 0;
}

static int sprd_stop_timer(struct timer_list *dcam_timer)
{
	DCAM_TRACE("stop timer\n");
	del_timer_sync(dcam_timer);

	return 0;
}

static int sprd_init_handle(struct dcam_dev *dev)
{
	struct dcam_info *info = &dev->dcam_cxt;
	struct dcam_path_spec *path;
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
	for (i = 0; i < DCAM_PATH_MAX; i++) {
		path = &info->dcam_path[i];
		if (path == NULL) {
			pr_err("init path %d fail\n", i);
			return -EINVAL;
		}
		memset((void *)path->frm_ptr,
		       0,
		       (unsigned int) (DCAM_FRM_CNT_MAX *
				       sizeof(struct dcam_frame *)));
		path->frm_cnt_act = 0;
		sprd_img_buf_queue_init(&path->buf_queue);
		path->status = PATH_IDLE;
	}
	atomic_set(&dev->stream_on, 0);

	return 0;
}

static int sprd_img_dev_init(struct dcam_group *group, enum dcam_id idx)
{
	int ret = 0;
	struct dcam_dev *dev = NULL;

	if (group->dev_inited & (1 << (int)idx)) {
		pr_info("sprd_img: dev%d already inited\n", idx);
		return ret;
	}

	dev = vzalloc(sizeof(*dev));
	if (!dev) {
		ret = -ENOMEM;
		pr_err("%s fail alloc dcam%d\n", __func__, idx);
		goto init_exit;
	}
	atomic_set(&dev->run_flag, 1);

	dev->idx = idx;
	mutex_init(&dev->dcam_mutex);

	if (unlikely(atomic_inc_return(&dev->users) > 1)) {
		ret = -EBUSY;
		goto init_exit;
	}

	ret = sprd_init_handle(dev);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to init handle\n", __func__);
		ret = -EIO;
		goto init_exit;
	}

	ret = dcam_create_flash_thread(dev);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to create flash thread\n", __func__);
		ret = -EIO;
		goto init_exit;
	}

	group->dev[idx] = dev;
	group->dev_inited |= 1 << idx;
	group->mode_inited = 0;

	DCAM_TRACE("%s: dev[%d] init OK %p!\n", __func__, idx, dev);

init_exit:
	if (unlikely(ret)) {
		atomic_dec(&dev->users);
		vfree(dev);
		pr_err("%s: dev[%d] init failed!\n", __func__, idx);
	}
	return ret;
}

static int sprd_img_dev_deinit(struct dcam_group *group, enum dcam_id idx)
{
	struct dcam_dev *dev = NULL;

	if (!(group->dev_inited & (1 << (int)idx))) {
		DCAM_TRACE("sprd_img: dev%d already deinited\n", idx);
		return 0;
	}

	dev = group->dev[idx];

	mutex_lock(&dev->dcam_mutex);

	atomic_set(&dev->stream_on, 0);
	sprd_dcam_module_deinit(idx);
	sprd_img_local_deinit(dev);

	atomic_dec(&dev->users);
	dcam_stop_flash_thread(dev);
	mutex_unlock(&dev->dcam_mutex);

	vfree(dev);
	group->dev[idx] = NULL;
	group->dev_inited &= ~(1<<idx);
	group->mode_inited = 0;
	DCAM_TRACE("%s: dev[%d] deinit OK!\n", __func__, idx);

	return 0;
}

static int sprd_img_get_res(struct dcam_group *group, struct sprd_img_res *res)
{
	int ret = 0;
	int dcam_id = 0;

	dcam_id = sprd_sensor_find_dcam_id(res->sensor_id);
	/*normal mode*/
	if (res->width < DCAM_SIGNLE_MAX_LENGTH) {
		/*rear sensor*/
		if (dcam_id == DCAM_ID_0) {
			if (res->flag == 0 && 0 == ((DCAM_RES_DCAM0_CAP |
				DCAM_RES_DCAM0_PATH)
				& group->dcam_res_used)) {
				pr_info("rear sensor => dcam0\n");
				res->flag = DCAM_RES_DCAM0_CAP |
					DCAM_RES_DCAM0_PATH;
				group->dcam_res_used |=
					DCAM_RES_DCAM0_CAP |
					DCAM_RES_DCAM0_PATH;
				group->cowork_mod = 0;
			} else if (res->flag && (DCAM_RES_DCAM0_CAP
				& group->dcam_res_used) == 0) {
				DCAM_TRACE("rear sensor => dcam0 top\n");
				res->flag = DCAM_RES_DCAM0_CAP;
				group->dcam_res_used |=
					DCAM_RES_DCAM0_CAP;
			} else {
				pr_err("Err: res_used 0x%x no valid dcam for rear sensor!\n",
						group->dcam_res_used);
				ret = -1;
				goto exit;
			}
		} else if (dcam_id == DCAM_ID_1) {
			if (res->flag == 0 && 0 == ((DCAM_RES_DCAM1_CAP |
				DCAM_RES_DCAM1_PATH) &
				group->dcam_res_used)) {
				DCAM_TRACE("front sensor => dcam1\n");
				res->flag = DCAM_RES_DCAM1_CAP |
					DCAM_RES_DCAM1_PATH;
				group->dcam_res_used |=
					DCAM_RES_DCAM1_CAP |
					DCAM_RES_DCAM1_PATH;
				group->cowork_mod = 0;
			} else if (res->flag && 0 == (DCAM_RES_DCAM1_CAP &
				group->dcam_res_used)) {
				DCAM_TRACE("front sensor => dcam1 top\n");
				res->flag = DCAM_RES_DCAM1_CAP;
				group->dcam_res_used |=
					DCAM_RES_DCAM1_CAP;
			} else {
				pr_err("Err:res_used 0x%x no valid dcam for front sensor!\n",
					group->dcam_res_used);
				ret = -1;
				goto exit;
			}
		}
	} else {
		if (0 == ((DCAM_RES_DCAM0_CAP |
			DCAM_RES_DCAM0_PATH |
			DCAM_RES_DCAM1_PATH) & group->dcam_res_used)) {
			pr_info("rear sensor => dcam0,dcam1_path\n");
			res->flag =
				DCAM_RES_DCAM0_CAP |
				DCAM_RES_DCAM0_PATH |
				DCAM_RES_DCAM1_PATH;
			group->dcam_res_used |=
				DCAM_RES_DCAM0_CAP |
				DCAM_RES_DCAM0_PATH |
				DCAM_RES_DCAM1_PATH;
			group->cowork_mod = 1;
		} else {
			pr_err("Err: res_used 0x%x no enough dcam for rear sensor!\n",
				group->dcam_res_used);
			ret = -1;
			goto exit;
		}
	}
exit:
	if (ret)
		res->flag = 0;
	return ret;
}

static int sprd_img_put_res(struct dcam_group *group, struct sprd_img_res *res)
{
	int ret = 0;
	int dcam_id = 0;

	dcam_id = sprd_sensor_find_dcam_id(res->sensor_id);
	if (group->cowork_mod == 0) {
		if (dcam_id == DCAM_ID_0) {
			if ((DCAM_RES_DCAM0_CAP |
				DCAM_RES_DCAM0_PATH) ==
				(res->flag &
				group->dcam_res_used)) {
				pr_info("put dcam0 for rear sensor\n");
				group->dcam_res_used &= ~(DCAM_RES_DCAM0_CAP |
					DCAM_RES_DCAM0_PATH);
			} else if ((res->flag & group->dcam_res_used) ==
					DCAM_RES_DCAM0_CAP) {
				DCAM_TRACE("put dcam0 top for rear sensor\n");
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
				(res->flag &
				group->dcam_res_used)) {
				DCAM_TRACE("put dcam1 for front sensor\n");
				group->dcam_res_used &= ~(DCAM_RES_DCAM1_CAP |
					DCAM_RES_DCAM1_PATH);
			} else if ((res->flag & group->dcam_res_used) ==
					DCAM_RES_DCAM1_CAP) {
				DCAM_TRACE("put dcam1 top for front sensor\n");
				group->dcam_res_used &= ~DCAM_RES_DCAM1_CAP;
				goto exit;
			} else {
					pr_err("Err: can't put dcam1 for front sensor!\n");
					ret = -1;
					goto exit;
				}
		}
	} else {
		if (dcam_id == DCAM_ID_0) {
			if ((DCAM_RES_DCAM0_CAP |
				DCAM_RES_DCAM0_PATH |
				DCAM_RES_DCAM1_PATH) ==
				(res->flag & group->dcam_res_used)) {
				DCAM_TRACE("put dcam0 and dcam1_path\n");
				group->dcam_res_used &= ~(DCAM_RES_DCAM0_CAP |
						DCAM_RES_DCAM0_PATH |
						DCAM_RES_DCAM1_PATH);
				group->cowork_mod = 0;
			} else {
				pr_err("can't put dcam rear sensor!\n");
				ret = -1;
				goto exit;
			}
		} else if (dcam_id == DCAM_ID_1) {
			if ((res->flag &
				group->dcam_res_used)
				== DCAM_RES_DCAM1_CAP) {
				DCAM_TRACE("put dcam1 top\n");
				group->dcam_res_used &=
					~DCAM_RES_DCAM1_CAP;
				} else {
					pr_err("Err: can't put dcam1_path!\n");
					ret = -1;
					goto exit;
				}
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
	struct dcam_file *dcamfile = NULL;
	struct miscdevice *md = file->private_data;
	struct dcam_group *group = NULL;
	struct cpp_dev *cpp_dev = NULL;

	get_dcam_lock(DCAM_RUNNING);
	group = md->this_device->platform_data;

	dcamfile = vzalloc(sizeof(struct dcam_file));
	if (dcamfile == NULL) {
		ret = -ENOMEM;
		goto exit;
	}

	init_completion(&dcamfile->irq_com);
	ret = sprd_img_queue_init(&dcamfile->queue);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to init queue\n", __func__);
		ret = -EIO;
		vfree(dcamfile);
		goto exit;
	}

	ret = sprd_init_timer(&dcamfile->dcam_timer, (unsigned long)dcamfile);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to init timer\n", __func__);
		ret = -EIO;
		vfree(dcamfile);
		goto exit;
	}

	dcamfile->grp = group;
	file->private_data = (void *)dcamfile;

	if (atomic_inc_return(&group->dcam_opened) > 1) {
		DCAM_TRACE("sprd_img: the dcam has been inited!\n");
		ret = 0;
		goto exit;
	}

	ret = sprd_img_dev_init(group, DCAM_ID_0);
	if (unlikely(ret != 0)) {
		pr_err("sprd_img: DCAM_ID_0 dev init fail\n");
		ret = -EINVAL;
		goto dcam_file_exit;
	}

	ret = sprd_img_dev_init(group, DCAM_ID_1);
	if (unlikely(ret != 0)) {
		pr_err("sprd_img: DCAM_ID_1 dev init fail\n");
		ret = -EINVAL;
		goto dev_init_exit0;
	}

	wake_lock(&group->wakelock);

	ret = cpp_k_open_device();
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to open cpp device\n", __func__);
		ret = -EINVAL;
		goto dev_init_exit1;
	}

	cpp_dev = vzalloc(sizeof(struct cpp_dev));
	if (!cpp_dev) {
		ret = -ENOMEM;
		pr_err("%s fail alloc cpp dev0\n", __func__);
		goto cpp_grp_exit;
	}
	cpp_dev->cpp_id = 0;
	group->cpp_dev[0] = cpp_dev;
	sprd_cam_queue_init(&cpp_dev->cpp_queue, sizeof(struct cpp_reqnode),
		DCAM_QUEUE_LENGTH);
	ret = sprd_img_create_zoom_thread(cpp_dev);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to create zoom thread 0\n", __func__);
		ret = -EINVAL;
		goto cpp_dev_exit0;
	}

	cpp_dev = vzalloc(sizeof(struct cpp_dev));
	if (!cpp_dev) {
		ret = -ENOMEM;
		pr_err("%s fail alloc cpp dev1\n", __func__);
		goto create_zoom_thread0_exit;
	}
	cpp_dev->cpp_id = 1;
	group->cpp_dev[1] = cpp_dev;
	sprd_cam_queue_init(&cpp_dev->cpp_queue, sizeof(struct cpp_reqnode),
		DCAM_QUEUE_LENGTH);
	ret = sprd_img_create_zoom_thread(cpp_dev);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to create zoom thread 1\n", __func__);
		ret = -EINVAL;
		goto cpp_dev_exit1;
	}
	goto exit;

cpp_dev_exit1:
	vfree(group->cpp_dev[1]);
create_zoom_thread0_exit:
	sprd_img_stop_zoom_thread(group->cpp_dev[0]);
cpp_dev_exit0:
	vfree(group->cpp_dev[0]);
cpp_grp_exit:
	cpp_k_close_device();
dev_init_exit1:
	sprd_img_dev_deinit(group, DCAM_ID_1);
dev_init_exit0:
	sprd_img_dev_deinit(group, DCAM_ID_0);
dcam_file_exit:
	vfree(dcamfile);
	atomic_dec(&group->dcam_opened);
exit:
	put_dcam_lock(DCAM_RUNNING);
	return ret;
}

static int sprd_img_k_release(struct inode *node, struct file *file)
{
	enum dcam_id idx = DCAM_ID_0;
	struct dcam_file *dcamfile = NULL;
	struct dcam_group *group = NULL;

	dcamfile = file->private_data;
	if (!dcamfile)
		return -EFAULT;

	group = dcamfile->grp;

	if (atomic_dec_return(&group->dcam_opened) == 0) {
		pr_info("begin to release dcam_img!\n");
		sprd_dcam_module_existed_set(0, 0);
		sprd_dcam_module_existed_set(1, 0);
		get_dcam_lock(DCAM_RELEASE);
		sprd_img_stop_zoom_thread(group->cpp_dev[0]);
		sprd_img_stop_zoom_thread(group->cpp_dev[1]);
		sprd_cam_queue_deinit(&group->cpp_dev[0]->cpp_queue);
		vfree(group->cpp_dev[0]);
		sprd_cam_queue_deinit(&group->cpp_dev[1]->cpp_queue);
		vfree(group->cpp_dev[1]);
		cpp_k_close_device();

		for (idx = DCAM_ID_0; idx < DCAM_ID_MAX; idx++) {
			sprd_dcam_stop(idx);
			sprd_img_dev_deinit(group, idx);
			sprd_dcam_module_dis(idx);
		}
		group->dcam_res_used &= ~(DCAM_RES_DCAM0_CAP |
				DCAM_RES_DCAM0_PATH);
		group->dcam_res_used &= ~(DCAM_RES_DCAM1_CAP |
				DCAM_RES_DCAM1_PATH);
		dcam_cowork_flag = 0;
		group->cowork_mod = 0;
		wake_unlock(&group->wakelock);
		put_dcam_lock(DCAM_RELEASE);
	}

	sprd_img_isp_unreg_isr(dcamfile->idx);
	sprd_stop_timer(&dcamfile->dcam_timer);
	sprd_img_queue_init(&dcamfile->queue);

	vfree(dcamfile);
	file->private_data = NULL;

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
	"SPRD_IMG_IO_GET_IOMMU_STATUS",
	"SPRD_IMG_IO_DISABLE_MODE",
	"SPRD_IMG_IO_ENABLE_MODE",
	"SPRD_IMG_IO_GET_DCAM_RES",
	"SPRD_IMG_IO_PUT_DCAM_RES",
};

static int sprd_img_stream_on(struct file *file)
{
	int ret = 0;
	int isp_id = 0;
	struct dcam_dev *dev = NULL;
	struct dcam_file *dcamfile = NULL;
	struct dcam_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct dcam_path_spec *path_0 = NULL;
	struct dcam_path_spec *path_1 = NULL;
	struct dcam_path_spec *path_2 = NULL;
	struct dcam_path_spec *path_3 = NULL;

	dcamfile = (struct dcam_file *)file->private_data;
	if (!dcamfile) {
		ret = -EFAULT;
		pr_err("dcamfile is NULL\n");
		goto exit;
	}

	group = dcamfile->grp;
	idx = dcamfile->idx;
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
	path_0 = &dev->dcam_cxt.dcam_path[DCAM_PATH0];
	path_1 = &dev->dcam_cxt.dcam_path[DCAM_PATH1];
	path_2 = &dev->dcam_cxt.dcam_path[DCAM_PATH2];
	path_3 = &dev->dcam_cxt.dcam_path[DCAM_PATH3];
	memset((void *)path_0->frm_ptr, 0,
	       DCAM_FRM_CNT_MAX * sizeof(struct dcam_frame *));

	do {
		/* dcam driver module initialization */
		ret = sprd_dcam_module_init(idx);
		if (unlikely(ret)) {
			pr_err("failed to init dcam module\n");
			break;
		}
		sprd_dcam_module_existed_set(idx, 1);
		pr_info("use_path %d\n", dev->use_path);

		ret = sprd_img_queue_clr(&dcamfile->queue);
		if (unlikely(ret != 0)) {
			pr_err("failed to init queue\n");
			break;
		}
		ret = sprd_img_dcam_reg_isr(dcamfile, idx);
		if (unlikely(ret)) {
			pr_err("failed to register isr\n");
			break;
		}

		/* config cap sub-module */
		ret = sprd_img_cap_cfg(dev);
		if (unlikely(ret)) {
			pr_err("failed to config cap");
			break;
		}

		/* config path1 sub-module if necessary */
		if (path_1->is_work)
			path_1->status = PATH_RUN;

		/* config path2 sub-module if necessary */
		if (path_2->is_work)
			path_2->status = PATH_RUN;

		/* config path3 sub-module if necessary */
		if (path_3->is_work)
			path_3->status = PATH_RUN;

		DCAM_TRACE("path_0->is_work=%d\n", path_0->is_work);
		if (path_0->is_work) {
			ret = sprd_img_path0_cfg(idx,
						 sprd_dcam_path_cfg,
						 path_0);
			if (unlikely(ret)) {
				pr_err("failed to config dcam path 0\n");
				break;
			}
			path_0->status = PATH_RUN;
		} else {
			ret = sprd_dcam_path_cfg(idx,
						 DCAM_PATH_IDX_0,
						 DCAM_PDAF_MODE_CONTROL,
						 &path_0->pdaf_ctrl);
			if (unlikely(ret)) {
				pr_err("failed to config dcam path 0\n");
				break;
			}
			ret = sprd_dcam_path_cfg(idx,
						 DCAM_PATH_IDX_0,
						 DCAM_PATH_ENABLE,
						 &path_0->is_work);
			if (unlikely(ret)) {
				pr_err("failed to config dcam path 0\n");
				break;
			}
		}
	} while (0);

	/*start isp path*/
	if (path_1->is_work || path_2->is_work || path_3->is_work) {
		isp_id = dcamfile->isp_id;

		ret = sprd_img_isp_reg_isr(dcamfile, isp_id);
		if (unlikely(ret))
			pr_err("failed to register isp isr\n");

		if (sprd_isp_stream_on(isp_id))
			pr_err("failed to sprd_isp_stream_on isp%d\n", isp_id);
	}

	dev->frame_skipped = 0;

	if ((dev->dcam_cxt.set_flash.led0_ctrl &&
	     dev->dcam_cxt.set_flash.led0_status == FLASH_HIGH_LIGHT) ||
	    (dev->dcam_cxt.set_flash.led1_ctrl &&
	     dev->dcam_cxt.set_flash.led1_status == FLASH_HIGH_LIGHT)) {
		if (dev->dcam_cxt.skip_number == 0)
			sprd_img_start_flash(NULL, dcamfile);
	}

	ret = sprd_dcam_start(idx);
	if (unlikely(ret))
		pr_err("failed to start path ixd %d\n", idx);

	atomic_set(&dev->stream_on, 1);

	if (ret) {
		sprd_img_unreg_isr(dcamfile);
		pr_err("failed to start stream 0x%x\n", ret);
	} else {
		atomic_set(&dev->run_flag, 0);
		sprd_start_timer(&dcamfile->dcam_timer, DCAM_TIMEOUT);
	}

	mutex_unlock(&dev->dcam_mutex);
exit:
	return ret;
}

static int sprd_img_stream_off(struct file *file)
{
	int ret = 0;
	struct dcam_dev *dev = NULL;
	struct dcam_file *dcamfile = NULL;
	struct dcam_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct dcam_path_spec *path_0 = NULL;
	struct dcam_path_spec *path_1 = NULL;
	struct dcam_path_spec *path_2 = NULL;
	struct dcam_path_spec *path_3 = NULL;

	dcamfile = (struct dcam_file *)file->private_data;
	if (!dcamfile) {
		ret = -EFAULT;
		pr_err("dcamfile is NULL\n");
		goto exit;
	}

	group = dcamfile->grp;
	idx = dcamfile->idx;
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

	path_0 = &dev->dcam_cxt.dcam_path[DCAM_PATH0];
	path_1 = &dev->dcam_cxt.dcam_path[DCAM_PATH1];
	path_2 = &dev->dcam_cxt.dcam_path[DCAM_PATH2];
	path_3 = &dev->dcam_cxt.dcam_path[DCAM_PATH3];

	if (unlikely(atomic_read(&dev->stream_on) == 0)) {
		DCAM_TRACE("stream not on, idx: %d\n", idx);
		ret = sprd_img_local_deinit(dev);
		if (unlikely(ret))
			pr_err("failed to local deinit\n");

		mutex_unlock(&dev->dcam_mutex);
		goto exit;
	}

	do {
		sprd_dcam_module_existed_set(idx, 0);
		ret = sprd_stop_timer(&dcamfile->dcam_timer);
		if (unlikely(ret)) {
			pr_err("failed to stop timer\n");
			break;
		}
		ret = sprd_dcam_stop(idx);
		if (unlikely(ret)) {
			pr_err("failed to stop dcam\n");
			break;
		}

		ret = sprd_img_unreg_isr(dcamfile);
		if (unlikely(ret)) {
			pr_err("failed to unregister isr\n");
			break;
		}

		if (path_1->is_work) {
			path_1->status = PATH_IDLE;
			path_1->is_work = 0;
		}

		if (path_2->is_work) {
			path_2->status = PATH_IDLE;
			path_2->is_work = 0;
		}

		if (path_3->is_work) {
			path_3->status = PATH_IDLE;
			path_3->is_work = 0;
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

exit:
	return ret;
}

static int sprd_img_check_fmt(struct file *file,
			      struct sprd_img_format *img_format)
{
	int ret = 0;
	unsigned int channel_id;
	struct dcam_dev *dev = NULL;
	struct dcam_file *dcamfile = NULL;
	struct dcam_group *group = NULL;
	struct dcam_path_spec *path = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct dcam_format *fmt;
	struct isp_cfg_img_param isp_path_param;
	struct isp_cfg_img_param isp_path_param1;
	struct dcam_info *dcaminfo = NULL;
	unsigned int is_work = 0;

	dcamfile = (struct dcam_file *)file->private_data;
	if (!dcamfile) {
		ret = -EFAULT;
		pr_err("dcamfile is NULL\n");
		goto exit;
	}

	if (!img_format) {
		ret = -EFAULT;
		pr_err("img format parm is NULL\n");
		goto exit;
	}

	group = dcamfile->grp;
	idx = dcamfile->idx;
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
	dcaminfo = &dev->dcam_cxt;
	if (unlikely(!dcaminfo)) {
		pr_err("dcaminfo is null\n");
		return -EINVAL;
	}
	fmt = sprd_img_get_format(img_format->fourcc);
	if (unlikely(!fmt)) {
		pr_err("fourcc format (0x%08x) invalid.\n",
		       img_format->fourcc);
		return -EINVAL;
	}

	channel_id = img_format->channel_id;

	if (channel_id == DCAM_PATH1) {
		if (atomic_read(&dev->stream_on) == 0) {
			mutex_lock(&dev->dcam_mutex);
			memset(&isp_path_param, 0,
				sizeof(struct isp_cfg_img_param));
			memset(&isp_path_param1, 0,
				sizeof(struct isp_cfg_img_param));
			isp_path_param.format = ISP_OUT_IMG_NV12;
			isp_path_param.buf_num = 4;
			isp_path_param.dram_eb = 0;
			isp_path_param.width = img_format->width;
			isp_path_param.height = img_format->height;
			isp_path_param.img_id = ISP_PATH_PREIVEW;
			isp_path_param.line_offset = img_format->width;
			sprd_isp_path_set(dcamfile->isp_id, ISP_PATH_PREIVEW,
					ISP_CFG_PATH_SET_PARAM,
					&isp_path_param);

			sprd_isp_path_get(dcamfile->isp_id, ISP_PATH_VIDEO,
				ISP_CFG_PATH_GET_WORK_STATUS, &is_work);
			if (is_work) {
				sprd_isp_path_get(dcamfile->isp_id,
						ISP_PATH_VIDEO,
						ISP_CFG_PATH_GET_PARAM,
						&isp_path_param1);
				if (isp_path_param.width >
					isp_path_param1.width) {
					isp_path_param.img_id = ISP_PATH_VIDEO;
					sprd_isp_path_set(dcamfile->isp_id,
							ISP_PATH_VIDEO,
							ISP_CFG_PATH_SET_PARAM,
							&isp_path_param);
				}
				is_work = 0;
			} else
				is_work = 1;

			sprd_isp_path_set(dcamfile->isp_id, ISP_PATH_PREIVEW,
				ISP_CFG_PATH_SET_WORK_STATUS, &is_work);

			path = &dev->dcam_cxt.dcam_path[DCAM_PATH1];
			path->is_work = 1;
			/*hard code for test*/
			path->isp_out_size.w = isp_path_param.width;
			path->isp_out_size.h = isp_path_param.height;
			path->out_size.w = img_format->width;
			path->out_size.h = img_format->height;
			mutex_unlock(&dev->dcam_mutex);
		}
	} else if (channel_id == DCAM_PATH2) {
		if (atomic_read(&dev->stream_on) == 0) {
			mutex_lock(&dev->dcam_mutex);
			isp_path_param.format = ISP_OUT_IMG_NV12;
			isp_path_param.buf_num = 4;
			isp_path_param.dram_eb = 0;
			isp_path_param.width = img_format->width;
			isp_path_param.height = img_format->height;
			isp_path_param.img_id = ISP_PATH_VIDEO;
			isp_path_param.line_offset = img_format->width;
			sprd_isp_path_set(dcamfile->isp_id, ISP_PATH_VIDEO,
				ISP_CFG_PATH_SET_PARAM, &isp_path_param);
			is_work = 1;
			sprd_isp_path_set(dcamfile->isp_id, ISP_PATH_VIDEO,
				ISP_CFG_PATH_SET_WORK_STATUS, &is_work);

			path = &dev->dcam_cxt.dcam_path[DCAM_PATH2];
			path->is_work = 1;
			/*hard code for test*/
			path->isp_out_size.w = isp_path_param.width;
			path->isp_out_size.h = isp_path_param.height;
			path->out_size.w = img_format->width;
			path->out_size.h = img_format->height;
			mutex_unlock(&dev->dcam_mutex);
		}
	} else if (channel_id == DCAM_PATH3) {
		if (atomic_read(&dev->stream_on) == 0) {
			mutex_lock(&dev->dcam_mutex);
			isp_path_param.format = ISP_OUT_IMG_NV12;
			isp_path_param.buf_num = 4;
			isp_path_param.dram_eb = 0;
			isp_path_param.width = img_format->width;
			isp_path_param.height = img_format->height;
			isp_path_param.img_id = ISP_PATH_STILL;
			isp_path_param.line_offset = img_format->width;
			sprd_isp_path_set(dcamfile->isp_id, ISP_PATH_STILL,
				ISP_CFG_PATH_SET_PARAM, &isp_path_param);
			is_work = 1;
			sprd_isp_path_set(dcamfile->isp_id, ISP_PATH_STILL,
				ISP_CFG_PATH_SET_WORK_STATUS, &is_work);
			path = &dev->dcam_cxt.dcam_path[DCAM_PATH3];
			path->is_work = 1;
			/*hard code for test*/
			path->isp_out_size.w = isp_path_param.width;
			path->isp_out_size.h = isp_path_param.height;
			path->out_size.w = img_format->width;
			path->out_size.h = img_format->height;
			mutex_unlock(&dev->dcam_mutex);
		}
	} else if (channel_id == DCAM_PATH0) {
		path = &dev->dcam_cxt.dcam_path[DCAM_PATH0];
		mutex_lock(&dev->dcam_mutex);
		ret = sprd_img_check_path0_cap(fmt->fourcc,
					       img_format,
					       &dev->dcam_cxt);
		mutex_unlock(&dev->dcam_mutex);
	} else {
		pr_err("buf type invalid\n");
		return -EINVAL;
	}
	memcpy((void *)&img_format->reserved[0],
	       (void *)&dev->dcam_cxt.dcam_path[channel_id].end_sel,
	       sizeof(struct dcam_endian_sel));

	path = &dev->dcam_cxt.dcam_path[channel_id];
	pr_info("channel_id %d out size w=%d, h=%d\n",
		channel_id,
		path->out_size.w,
		path->out_size.h);

	if (ret == 0 && (atomic_read(&dev->stream_on) != 0)) {
		if (channel_id == DCAM_PATH1 ||
		    channel_id == DCAM_PATH2 ||
		    channel_id == DCAM_PATH3) {
			ret = sprd_img_update_video(dcamfile, channel_id);
		}
	} else {
		mutex_lock(&dev->dcam_mutex);
		ret = sprd_img_path_pre_proc(channel_id,
				path,
				dcaminfo);
		mutex_unlock(&dev->dcam_mutex);
	}

exit:
	return ret;
}

static int sprd_img_set_frame_addr(struct file *file,
				   struct sprd_img_parm *p)
{
	int ret = 0;
	struct dcam_file *dcamfile = NULL;
	struct dcam_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct dcam_dev *dev = NULL;
	struct isp_if_context *ispif = NULL;
	struct isp_img_info *img_info = NULL;
	struct isp_img_buf  temp_img_buf;
	struct dcam_path_spec *path = NULL;
	int isp_id = 0;
	int img_id = 0;

	dcamfile = (struct dcam_file *)file->private_data;
	if (!dcamfile) {
		ret = -EFAULT;
		pr_err("dcamfile is NULL\n");
		goto exit;
	}

	if (!p) {
		ret = -EFAULT;
		pr_err("img parm is NULL\n");
		goto exit;
	}

	group = dcamfile->grp;
	idx = dcamfile->idx;
	isp_id = dcamfile->isp_id;
	img_id = sprd_img_get_isp_img_id(p->channel_id);
	if (group->dev_inited & (1 << (int)idx)) {
		ispif = sprd_isp_get_isp_handle(dcamfile->isp_id);
		if (!ispif) {
			ret = -EFAULT;
			pr_err("ispif[%d] is NULL\n", dcamfile->isp_id);
			goto exit;
		}
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

	/*need fixed, hard code*/
	switch (p->channel_id) {
	case DCAM_PATH0:
		path = &dev->dcam_cxt.dcam_path[DCAM_PATH0];
		break;
	case DCAM_PATH1:
	case DCAM_PATH2:
	case DCAM_PATH3:
		img_info =
			&ispif->isp_pipe->isp_path.img_info[img_id];
		break;
	default:
		pr_info("invalid channel %d\n", p->channel_id);
		return -EINVAL;
	}

	if (unlikely(p->frame_addr_vir.y == 0)) {
		pr_err("no yaddr\n");
		ret = -EINVAL;
		goto exit;
	}

	if (p->channel_id == DCAM_PATH0) {
		if (p->is_reserved_buf == 1) {
			path->frm_reserved_addr.yaddr = p->frame_addr.y;
			path->frm_reserved_addr.uaddr = p->frame_addr.u;
			path->frm_reserved_addr.vaddr = p->frame_addr.v;
			path->frm_reserved_addr_vir.yaddr = p->frame_addr_vir.y;
			path->frm_reserved_addr_vir.uaddr = p->frame_addr_vir.u;
			path->frm_reserved_addr_vir.vaddr = p->frame_addr_vir.v;
			path->frm_reserved_addr.mfd_y = p->reserved[1];
			path->frm_reserved_addr.mfd_u = p->reserved[2];
			path->frm_reserved_addr.mfd_v = p->reserved[3];
		} else {
			struct dcam_addr frame_addr;
			struct dcam_img_buf_addr buf_addr;

			frame_addr.yaddr = p->frame_addr.y;
			frame_addr.uaddr = p->frame_addr.u;
			frame_addr.vaddr = p->frame_addr.v;
			frame_addr.yaddr_vir = p->frame_addr_vir.y;
			frame_addr.uaddr_vir = p->frame_addr_vir.u;
			frame_addr.vaddr_vir = p->frame_addr_vir.v;
			frame_addr.mfd_y = p->reserved[1];
			frame_addr.mfd_u = p->reserved[2];
			frame_addr.mfd_v = p->reserved[3];

			if (atomic_read(&dev->stream_on) == 1 &&
			    path->status == PATH_RUN) {
				ret = sprd_dcam_path_cfg(idx,
					DCAM_PATH_IDX_0,
					DCAM_PATH_OUTPUT_ADDR,
					&frame_addr);
			} else {
				buf_addr.frm_addr.yaddr = p->frame_addr.y;
				buf_addr.frm_addr.uaddr = p->frame_addr.u;
				buf_addr.frm_addr.vaddr = p->frame_addr.v;
				buf_addr.frm_addr_vir.yaddr =
					p->frame_addr_vir.y;
				buf_addr.frm_addr_vir.uaddr =
					p->frame_addr_vir.u;
				buf_addr.frm_addr_vir.vaddr =
					p->frame_addr_vir.v;
				buf_addr.frm_addr.mfd_y = p->reserved[1];
				buf_addr.frm_addr.mfd_u = p->reserved[2];
				buf_addr.frm_addr.mfd_v = p->reserved[3];
				ret = sprd_img_buf_queue_write(&path->buf_queue,
							       &buf_addr);
			}
		}
	} else {
		if (p->is_reserved_buf == 1) {
			/*to be fixed*/
			temp_img_buf.yaddr = p->frame_addr.y;
			temp_img_buf.uaddr = p->frame_addr.u;
			temp_img_buf.vaddr = p->frame_addr.v;
			temp_img_buf.yaddr_vir = p->frame_addr_vir.y;
			temp_img_buf.uaddr_vir = p->frame_addr_vir.u;
			temp_img_buf.vaddr_vir = p->frame_addr_vir.v;
			temp_img_buf.img_y_fd = p->reserved[1];
			temp_img_buf.img_u_fd = p->reserved[2];
			temp_img_buf.img_v_fd = p->reserved[3];

			DCAM_TRACE("set reserved dcam%d img_id=%d yfd=0x%x\n",
				dcamfile->idx, img_id, p->reserved[1]);

			ret = sprd_isp_path_set(isp_id,
					img_id,
					ISP_CFG_PATH_OUTPUT_RESERVED_ADDR,
					&temp_img_buf);
		} else {
			struct isp_output_addr output_addr;

			output_addr.yaddr = p->frame_addr.y;
			output_addr.uaddr = p->frame_addr.u;
			output_addr.vaddr = p->frame_addr.v;
			output_addr.yaddr_vir = p->frame_addr_vir.y;
			output_addr.uaddr_vir = p->frame_addr_vir.u;
			output_addr.vaddr_vir = p->frame_addr_vir.v;
			output_addr.mfd_y = p->reserved[1];
			output_addr.mfd_u = p->reserved[2];
			output_addr.mfd_v = p->reserved[3];
			pr_debug("set dcam%d img_id=%d yfd=0x%x\n",
					dcamfile->idx, img_id, p->reserved[1]);
			ret = sprd_isp_path_set(isp_id,
				img_id,
				ISP_CFG_PATH_OUTPUT_ADDR,
				&output_addr);
		}
	}
exit:
	return ret;
}

static int sprd_img_get_free_channel(struct file *file,
				     unsigned int *channel_id,
				     unsigned int scene_mode)
{
	int ret = 0;
	struct dcam_dev *dev = NULL;
	struct dcam_file *dcamfile = NULL;
	struct dcam_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct dcam_path_spec *path_0 = NULL;
	struct dcam_path_spec *path_1 = NULL;
	struct dcam_path_spec *path_2 = NULL;
	struct dcam_path_spec *path_3 = NULL;
	struct dcam_get_path_id path_id;

	dcamfile = (struct dcam_file *)file->private_data;
	if (!dcamfile) {
		ret = -EFAULT;
		pr_err("dcamfile is NULL\n");
		goto exit;
	}

	if (!channel_id) {
		ret = -EFAULT;
		pr_err("parm is NULL\n");
		goto exit;
	}

	group = dcamfile->grp;
	idx = dcamfile->idx;
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

	path_0 = &dev->dcam_cxt.dcam_path[DCAM_PATH0];
	path_1 = &dev->dcam_cxt.dcam_path[DCAM_PATH1];
	path_2 = &dev->dcam_cxt.dcam_path[DCAM_PATH2];
	path_3 = &dev->dcam_cxt.dcam_path[DCAM_PATH3];

	memset((void *)&path_id, 0,
	       sizeof(struct dcam_get_path_id));
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
	path_id.is_path_work[DCAM_PATH0] = path_0->is_work;
	path_id.is_path_work[DCAM_PATH1] = path_1->is_work;
	path_id.is_path_work[DCAM_PATH2] = path_2->is_work;
	path_id.is_path_work[DCAM_PATH3] = path_3->is_work;
	ret = sprd_dcam_get_path_id(&path_id, channel_id, scene_mode);

	DCAM_TRACE("get channel %d\n", *channel_id);

exit:
	return ret;
}

static int sprd_img_set_crop(struct file *file, struct sprd_img_parm *p)
{
	int ret = 0;
	struct dcam_dev *dev = NULL;
	struct dcam_file *dcamfile = NULL;
	struct dcam_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct dcam_rect *input_rect;
	struct dcam_size *input_size;

	dcamfile = (struct dcam_file *)file->private_data;
	if (!dcamfile) {
		ret = -EFAULT;
		pr_err("dcamfile is NULL\n");
		goto exit;
	}

	if (!p) {
		ret = -EFAULT;
		pr_err("img parm is NULL\n");
		goto exit;
	}

	group = dcamfile->grp;
	idx = dcamfile->idx;
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

	DCAM_TRACE("dcam%d: set crop, window %d %d %d %d\n", idx,
		   p->crop_rect.x, p->crop_rect.y,
		   p->crop_rect.w, p->crop_rect.h);

	switch (p->channel_id) {
	case DCAM_PATH0:
	case DCAM_PATH1:
	case DCAM_PATH2:
	case DCAM_PATH3:
		input_size = &dev->dcam_cxt.dcam_path[p->channel_id].in_size;
		input_rect = &dev->dcam_cxt.dcam_path[p->channel_id].in_rect;
		break;
	default:
		pr_err("dcam%d: wrong channel id %d\n", idx, p->channel_id);
		ret = -EINVAL;
		goto exit;
	}

	input_rect->x = p->crop_rect.x;
	input_rect->y = p->crop_rect.y;
	input_rect->w = p->crop_rect.w;
	input_rect->h = p->crop_rect.h;
	if (dev->dcam_cxt.path_input_rect_change) {
		input_size->w = p->reserved[1];
		input_size->h = p->reserved[2];
	} else {
		input_size->w = dev->dcam_cxt.cap_out_size.w;
		input_size->h = dev->dcam_cxt.cap_out_size.h;
	}
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
	struct dcam_dev *dev = NULL;
	struct dcam_file *dcamfile = NULL;
	struct dcam_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct dcam_info *info = NULL;

	dcamfile = (struct dcam_file *)file->private_data;
	if (!dcamfile) {
		ret = -EFAULT;
		pr_err("dcamfile is NULL\n");
		goto exit;
	}

	if (!sensor_if) {
		ret = -EFAULT;
		pr_err("sensor_if parm is NULL\n");
		goto exit;
	}

	group = dcamfile->grp;
	idx = dcamfile->idx;
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
		info->pclk = sensor_if->if_spec.mipi.pclk;
	}

exit:
	return ret;
}

static long sprd_img_k_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	int ret = 0;
	int isp_id = 0;
	struct dcam_dev *dev = NULL;
	struct dcam_info *info = NULL;
	unsigned int channel_id;
	unsigned int mode;
	unsigned int skip_num;
	unsigned int zoom;
	unsigned int led0_ctrl;
	unsigned int led1_ctrl;
	unsigned int led0_status;
	unsigned int led1_status;
	unsigned int iommu_enable;
	struct sprd_img_parm parm;
	struct sprd_img_size size;
	struct sprd_img_rect rect;
	struct dcam_path_spec *path = NULL;
	struct dcam_file *dcamfile = NULL;
	struct dcam_group *group = NULL;
	enum dcam_id idx = DCAM_ID_0;
	struct sprd_img_sensor_if sensor;
	struct dcam_format *fmt;
	struct sprd_img_format img_format;
	struct sprd_img_get_fmt fmt_desc;
	struct timeval time;
	struct sprd_img_time utime;
	struct sprd_flash_cfg_param cfg_parm;
	struct sprd_img_res res = {0};
	struct isp_if_context *ispif = NULL;
	struct isp_img_info  *isp_img = NULL;

	get_dcam_lock(DCAM_RUNNING);
	dcamfile = (struct dcam_file *)file->private_data;
	if (!dcamfile) {
		ret = -EFAULT;
		pr_err("dcamfile is NULL\n");
		goto exit;
	}

	group = dcamfile->grp;

	if ((_IOC_NR(cmd) >= _IOC_NR(SPRD_IMG_IO_SET_MODE)) &&
	    (_IOC_NR(cmd) <= _IOC_NR(SPRD_IMG_IO_PUT_DCAM_RES))) {
		if (_IOC_NR(cmd) <= _IOC_NR(SPRD_IMG_IO_DISABLE_MODE)) {
			idx = dcamfile->idx;
			if (group->dev_inited & (1 << (int)idx)) {
				dev = group->dev[idx];
				if (!dev) {
					ret = -EFAULT;
					pr_err("dev[%d] is NULL\n", idx);
					goto exit;
				}
				info = &dev->dcam_cxt;
			} else {
				ret = -EFAULT;
				pr_err("dev[%d] hasn't been inited!\n", idx);
				goto exit;
			}
		}
		pr_debug("sprd_img_k_ioctl:dcam%d, cmd: 0x%x, %s\n",
			idx, cmd, ioctl_str[_IOC_NR(cmd)]);

	} else {
		pr_err("invalid cmd 0x%x\n", cmd);
		goto exit;
	}

	switch (cmd) {
	case SPRD_IMG_IO_GET_DCAM_RES:
		ret = copy_from_user(&res, (void __user *)arg,
				sizeof(struct sprd_img_res));
		if (ret) {
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}
		ret = sprd_img_get_res(group, &res);
		if (ret) {
			pr_err("get res failed!\n");
			break;
		}

		isp_id = get_unused_isp_id(&res);
		if (isp_id >= ISP_DEV_NUM || isp_id < ISP_DEV0) {
			pr_err("get wrong isp_id %d\n", isp_id);
			ret = -EFAULT;
			break;
		}
		dcamfile->isp_id = isp_id;

		idx = sprd_sensor_find_dcam_id(res.sensor_id);
		if (group->mode_inited & (1 << (int)idx)) {
			DCAM_TRACE("dcam%d has been enabled!\n", idx);
			break;
		}
		sprd_dcam_module_en(idx);
		dcamfile->idx = idx;
		group->mode_inited |= 1 << idx;
		DCAM_TRACE("%s: dcam dev[%d] init: 0x%lx\n", __func__,
				idx, (unsigned long)group->dev[idx]);

		if (group->cowork_mod == 1 && idx == DCAM_ID_0) {
			if ((group->dev_inited & (1 << DCAM_ID_1)) &&
				(DCAM_RES_DCAM1_PATH &
				group->dcam_res_used)) {
				DCAM_TRACE("cowork mod init dcam1!\n");
				sprd_dcam_module_en(DCAM_ID_1);
				group->mode_inited |= 1 << DCAM_ID_1;
				dcam_cowork_flag = 1;
			}
		}
		ret = copy_to_user((void __user *)arg, &res,
				sizeof(struct sprd_img_res));
		if (ret) {
			pr_err("failed to set user info\n");
			ret = -EFAULT;
		}
		break;

	case SPRD_IMG_IO_PUT_DCAM_RES:
		ret = copy_from_user(&res, (void __user *)arg,
				sizeof(struct sprd_img_res));
		if (ret) {
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}

		idx = sprd_sensor_find_dcam_id(res.sensor_id);
		DCAM_TRACE("cowork: %d, dev_inited:%d, idx: %d\n",
			group->cowork_mod,
			group->dev_inited,
			idx);
		if (!(group->mode_inited & (1 << (int)idx))) {
			pr_err("dcam%d has been already disabled!\n", idx);
			break;
		}
		sprd_dcam_module_dis(idx);
		group->mode_inited &= ~(1<<idx);
		if (group->cowork_mod == 1 && idx == DCAM_ID_0) {
			if ((group->dev_inited & (1 << DCAM_ID_1)) &&
				(DCAM_RES_DCAM1_PATH &
				      group->dcam_res_used)) {
				sprd_dcam_module_dis(DCAM_ID_1);
				dcam_cowork_flag = 0;
				group->mode_inited &= ~(1<<DCAM_ID_1);
			}
		}

		put_used_isp_id(dcamfile->isp_id);
		ret = sprd_img_put_res(group, &res);
		if (ret) {
			pr_err("put res failed!\n");
			break;
		}
		ret = copy_to_user((void __user *)arg, &res,
				sizeof(struct sprd_img_res));
		if (ret) {
			pr_err("failed to set user info\n");
			ret = -EFAULT;
		}
		break;

	case SPRD_IMG_IO_ENABLE_MODE:
	case SPRD_IMG_IO_DISABLE_MODE:
		break;

	case SPRD_IMG_IO_SET_MODE:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&mode, (void __user *) arg,
				sizeof(unsigned int));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}
		info->capture_mode = mode;
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("%d: capture mode %d\n", idx,
			   dev->dcam_cxt.capture_mode);
		break;

	case SPRD_IMG_IO_SET_CAP_SKIP_NUM:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&skip_num, (void __user *) arg,
				sizeof(unsigned int));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}
		info->skip_number = skip_num;
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("%d: cap skip number %d\n", idx,
			   dev->dcam_cxt.skip_number);
		break;

	case SPRD_IMG_IO_SET_SENSOR_SIZE:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&size, (void __user *)arg,
				sizeof(struct sprd_img_size));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}
		info->cap_in_size.w = size.w;
		info->cap_in_size.h = size.h;
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("%d: sensor size %d %d\n", idx,
			   info->cap_in_size.w, info->cap_in_size.h);
		break;

	case SPRD_IMG_IO_SET_SENSOR_TRIM:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&rect, (void __user *)arg,
				sizeof(struct sprd_img_rect));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
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
		break;

	case SPRD_IMG_IO_SET_FRM_ID_BASE:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}

		switch (parm.channel_id) {
		case DCAM_PATH0:
			info->dcam_path[parm.channel_id].frm_id_base =
				parm.frame_base_id;
			break;
		case DCAM_PATH1:
		case DCAM_PATH2:
		case DCAM_PATH3:
			ispif = sprd_isp_get_isp_handle(dcamfile->isp_id);
			if (!ispif) {
				mutex_unlock(&dev->dcam_mutex);
				pr_err("isp %d handle is null.\n",
						dcamfile->isp_id);
				goto exit;
			}

			isp_img = &ispif->isp_pipe->isp_path.img_info
				[sprd_img_get_isp_img_id(parm.channel_id)];
			isp_img->frm_id_base = parm.frame_base_id;
			break;
		default:
			mutex_unlock(&dev->dcam_mutex);
			pr_err("%d: wrong channel ID, %d\n", idx,
				parm.channel_id);
			ret = -EFAULT;
			goto exit;
		}
		mutex_unlock(&dev->dcam_mutex);
		break;

	case SPRD_IMG_IO_SET_CROP:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}
		info->path_input_rect_change = parm.reserved[0];
		ret = sprd_img_set_crop(file, &parm);
		mutex_unlock(&dev->dcam_mutex);
		break;

	case SPRD_IMG_IO_SET_FLASH:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&info->set_flash,
				(void __user *)arg,
				sizeof(struct sprd_img_set_flash));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
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
		DCAM_TRACE("flash index %d\n",
			   info->set_flash.flash_index);
		break;

	case SPRD_IMG_IO_SET_OUTPUT_SIZE:
		DCAM_TRACE("%d: set output size\n", idx);
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}
		info->dst_size.w = parm.dst_size.w;
		info->dst_size.h = parm.dst_size.h;
		DCAM_TRACE("dst_size.w=%d, dst_size.h=%d\n",
			parm.dst_size.w, parm.dst_size.h);
		info->pxl_fmt = parm.pixel_fmt;
		info->need_isp_tool = parm.need_isp_tool;
		info->need_isp = parm.need_isp;
		info->rt_refocus = parm.rt_refocus;
		info->path_input_rect.x = parm.crop_rect.x;
		info->path_input_rect.y = parm.crop_rect.y;
		info->path_input_rect.w = parm.crop_rect.w;
		info->path_input_rect.h = parm.crop_rect.h;
		info->scene_mode = parm.scene_mode;
		pr_info("scene_mode = %d\n", parm.scene_mode);
		mutex_unlock(&dev->dcam_mutex);
		break;

	case SPRD_IMG_IO_SET_ZOOM_MODE:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&zoom, (void __user *) arg,
				sizeof(unsigned int));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}
		info->is_smooth_zoom = zoom;
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("%d: set zoom mode %d\n", idx, info->is_smooth_zoom);
		break;

	case SPRD_IMG_IO_SET_SENSOR_IF:
		DCAM_TRACE("%d: set sensor if\n", idx);
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&sensor, (void __user *)arg,
				sizeof(struct sprd_img_sensor_if));
		if (unlikely(ret)) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}

		ret = sprd_img_set_sensor_if(file, &sensor);
		mutex_unlock(&dev->dcam_mutex);
		break;

	case SPRD_IMG_IO_SET_FRAME_ADDR:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}

		ret = sprd_img_set_frame_addr(file, &parm);
		mutex_unlock(&dev->dcam_mutex);
		break;

	case SPRD_IMG_IO_PATH_FRM_DECI:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}
		path = &info->dcam_path[parm.channel_id];
		path->path_frm_deci = parm.deci;
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("channel %d, frm_deci %d\n",
			   parm.channel_id, path->path_frm_deci);
		break;

	case SPRD_IMG_IO_SET_SHRINK:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
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
		ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}
		path = &info->dcam_path[parm.channel_id];
		path->pdaf_ctrl.isp_tool_mode = parm.pdaf_ctrl.isp_tool_mode;
		path->pdaf_ctrl.mode = parm.pdaf_ctrl.mode;
		path->pdaf_ctrl.phase_data_type =
			parm.pdaf_ctrl.phase_data_type;
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("channel %d, pdaf mode %d type %d\n",
			   parm.channel_id, path->pdaf_ctrl.mode,
			   path->pdaf_ctrl.phase_data_type);
		break;

	case SPRD_IMG_IO_PATH_PAUSE:
		ret = copy_from_user(&parm, (void __user *)arg,
				sizeof(struct sprd_img_parm));
		if (ret) {
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}
		sprd_img_streampause(dcamfile, parm.channel_id,
			parm.reserved[0]);
		break;

	case SPRD_IMG_IO_PATH_RESUME:
		ret = copy_from_user(&channel_id, (void __user *) arg,
				sizeof(unsigned int));
		if (ret) {
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}
		if (dev->use_path)
			sprd_img_streamresume(dcamfile, channel_id);
		break;

	case SPRD_IMG_IO_STREAM_ON:
		DCAM_TRACE("stream on\n");
		ret = sprd_img_stream_on(file);
		break;

	case SPRD_IMG_IO_STREAM_OFF:
		DCAM_TRACE("stream off\n");
		ret = sprd_img_stream_off(file);
		break;

	case SPRD_IMG_IO_GET_FMT:
		ret = copy_from_user(&fmt_desc, (void __user *)arg,
				sizeof(struct sprd_img_get_fmt));
		if (ret) {
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}
		if (unlikely(fmt_desc.index >= ARRAY_SIZE(dcam_img_fmt))) {
			ret = -EINVAL;
			goto exit;
		}

		fmt = &dcam_img_fmt[fmt_desc.index];
		fmt_desc.fmt = fmt->fourcc;

		ret = copy_to_user((void __user *)arg, &fmt_desc,
				sizeof(struct sprd_img_get_fmt));
		if (ret) {
			pr_err("failed to set user info\n");
			ret = -EFAULT;
		}
		break;

	case SPRD_IMG_IO_GET_CH_ID:
		DCAM_TRACE("get free channel\n");
		sprd_img_get_free_channel(file, &channel_id, info->scene_mode);
		ret = copy_to_user((void __user *) arg, &channel_id,
				sizeof(unsigned int));
		if (ret) {
			pr_err("failed to set user info\n");
			ret = -EFAULT;
		}
		break;

	case SPRD_IMG_IO_GET_TIME:
		DCAM_TRACE("get time\n");
		img_get_timestamp(&time);
		utime.sec = time.tv_sec;
		utime.usec = time.tv_usec;
		ret = copy_to_user((void __user *)arg, &utime,
				sizeof(struct sprd_img_time));
		if (ret) {
			pr_err("failed to set user info\n");
			ret = -EFAULT;
		}
		break;

	case SPRD_IMG_IO_CHECK_FMT:
		DCAM_TRACE("check fmt\n");
		ret = copy_from_user(&img_format, (void __user *)arg,
				sizeof(struct sprd_img_format));
		if (ret) {
			pr_err("failed to get user info\n");
			ret = -EFAULT;
			goto exit;
		}

		ret = sprd_img_check_fmt(file, &img_format);
		if (ret)
			goto exit;

		ret = copy_to_user((void __user *)arg, &img_format,
				sizeof(struct sprd_img_format));
		if (ret) {
			pr_err("failed to set user info\n");
			ret = -EFAULT;
		}
		break;

	case SPRD_IMG_IO_CFG_FLASH:
		mutex_lock(&dev->dcam_mutex);
		ret = copy_from_user(&cfg_parm, (void __user *)arg,
				sizeof(struct sprd_flash_cfg_param));
		if (ret) {
			mutex_unlock(&dev->dcam_mutex);
			ret = -EFAULT;
			goto exit;
		}
		ret = sprd_flash_cfg(&cfg_parm);
		mutex_unlock(&dev->dcam_mutex);
		DCAM_TRACE("config flash, ret %d\n", ret);
		break;

	case SPRD_IMG_IO_GET_IOMMU_STATUS:
		ret = copy_from_user(&iommu_enable, (void __user *)arg,
				sizeof(unsigned char));
		if (ret) {
			pr_err("copy_from_user failed\n");
			ret = -EFAULT;
			goto exit;
		}

		if (sprd_iommu_attach_device(&s_pdev->dev) == 0)
			iommu_enable = 1;
		else
			iommu_enable = 0;

		ret = copy_to_user((void __user *)arg, &iommu_enable,
				sizeof(unsigned char));
		if (ret) {
			pr_err("failed to set user info\n");
			ret = -EFAULT;
		}
		break;

	default:
		pr_info("invalid cmd 0x%x\n", cmd);
		ret = -EFAULT;
		break;
	}

exit:
	put_dcam_lock(DCAM_RUNNING);
	return ret;
}

static ssize_t sprd_img_read(struct file *file, char __user *u_data,
			     size_t cnt, loff_t *cnt_ret)
{
	int i;
	int ret = 0;
	uint32_t channel_id = 0;
	struct dcam_file *dcamfile = file->private_data;
	struct dcam_dev *dev = NULL;
	struct dcam_node node;
	struct dcam_path_spec *path;
	struct sprd_img_read_op read_op;
	struct dcam_path_capability path_capability;
	struct dcam_path_info *info;
	struct sprd_img_path_info *info_ret;

	if (cnt != sizeof(struct sprd_img_read_op)) {
		pr_err("err, cnt %ld read_op %ld\n", cnt,
		       sizeof(struct sprd_img_read_op));
		return -EIO;
	}

	if (copy_from_user(&read_op, (void __user *)
				u_data, cnt)) {
		pr_err("failed to get user info\n");
		return -EFAULT;
	}

	get_dcam_lock(DCAM_RUNNING);

	switch (read_op.cmd) {
	case SPRD_IMG_GET_SCALE_CAP:
		/*DCAM_TRACE("get scale capbility\n");*/
		read_op.parm.reserved[0] = DCAM_PATH2_LINE_BUF_LENGTH;
		read_op.parm.reserved[1] = DCAM_SC_COEFF_UP_MAX;
		read_op.parm.reserved[2] = DCAM_SCALING_THRESHOLD;
		DCAM_TRACE("line threshold %d, sc factor %d, scaling %d.\n",
			   read_op.parm.reserved[0], read_op.parm.reserved[1],
			   read_op.parm.reserved[2]);
		break;

	case SPRD_IMG_GET_FRM_BUFFER:
		memset(&node, 0, sizeof(struct dcam_node));
		memset(&read_op, 0, sizeof(struct sprd_img_read_op));
		while (1) {
			ret = wait_for_completion_interruptible(
				&dcamfile->irq_com);
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
		memset(&node, 0, sizeof(struct dcam_node));
		memset(&read_op, 0, sizeof(struct sprd_img_read_op));
		if (sprd_img_queue_read(&dcamfile->queue, &node)) {
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

		dev = get_dcam_dev(dcamfile);
		if (!dev) {
			pr_err("not init dev %p\n", dev);
			return -EFAULT;
		}

		channel_id = sprd_img_get_channel_id(node.channel_id);
		if (node.irq_flag == ISP_IMG_TX_DONE) {
			read_op.evt = IMG_TX_DONE;
			read_op.parm.frame.channel_id = channel_id;
			read_op.parm.frame.index = node.frm_base_id;
			read_op.parm.frame.height = node.height;
			read_op.parm.frame.length = node.length;
			read_op.parm.frame.sec = node.time.tv_sec;
			read_op.parm.frame.usec = node.time.tv_usec;
			read_op.parm.frame.monoboottime = node.boot_time.tv64;
			read_op.parm.frame.frm_base_id = node.frm_base_id;
			read_op.parm.frame.img_fmt = node.format;
			read_op.parm.frame.yaddr = node.yaddr;
			read_op.parm.frame.uaddr = node.uaddr;
			read_op.parm.frame.vaddr = node.vaddr;
			read_op.parm.frame.yaddr_vir = node.yaddr_vir;
			read_op.parm.frame.uaddr_vir = node.uaddr_vir;
			read_op.parm.frame.vaddr_vir = node.vaddr_vir;
			read_op.parm.frame.reserved[0] = node.mfd[0];
			read_op.parm.frame.reserved[1] = node.mfd[1];
			read_op.parm.frame.reserved[2] = node.mfd[2];
			#if 1
			pr_debug("yfd:0x%x, channel_id:%d base id:0x%x\n",
					node.mfd[0],
					channel_id,
					node.frm_base_id);
			#endif
		} else if (node.irq_flag == IMG_TX_DONE) {
			read_op.evt = IMG_TX_DONE;
			read_op.parm.frame.channel_id = node.f_type;
			path = &dev->dcam_cxt.dcam_path[read_op.parm.frame.
				channel_id];
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
			read_op.parm.frame.reserved[0] = node.mfd[0];
			read_op.parm.frame.reserved[1] = node.mfd[1];
			read_op.parm.frame.reserved[2] = node.mfd[2];
			/*DCAM_TRACE("dcam driver done\n");*/
		} else if (node.irq_flag == ISP_IMG_NO_MEM ||
			node.irq_flag == IMG_NO_MEM) {
			read_op.evt = IMG_NO_MEM;
			DCAM_TRACE("isp_read:dev%d no memory error", dev->idx);
		} else if (node.irq_flag == ISP_IMG_TX_ERR ||
			node.irq_flag == IMG_TX_ERR) {
			read_op.evt = IMG_TX_ERR;
			DCAM_TRACE("isp_read:dev%d tx error", dev->idx);
		} else if (node.irq_flag == ISP_IMG_TIMEOUT ||
			node.irq_flag == IMG_TIMEOUT) {
			read_op.evt = IMG_TIMEOUT;
			DCAM_TRACE("isp_read:dev%d time out", dev->idx);
		} else if (node.irq_flag == ISP_IMG_SYS_BUSY) {
			read_op.evt = IMG_SYS_BUSY;
			DCAM_TRACE("isp_read:dev%d sys busy", dev->idx);
		} else if (node.irq_flag == IMG_TX_STOP) {
			read_op.evt = IMG_TX_STOP;
			DCAM_TRACE("isp_read:dev%d img tx stop", dev->idx);
		}

		if (read_op.evt == IMG_TIMEOUT || read_op.evt == IMG_TX_ERR) {
			unsigned int val = 0;
			struct regmap *syscon_gpr;

			syscon_gpr =
				syscon_regmap_lookup_by_compatible(
				"sprd,iwhale2-aon-apb");
			if (!IS_ERR(syscon_gpr)) {
				regmap_read(syscon_gpr,
					REG_AON_APB_AON_CGM_CFG, &val);
				pr_err("APB_AON_CGM_CFG 0x%x", val);
			}
			csi_dump_reg();
			isp_dump_reg_info();
			sprd_dcam2isp_dump_reg();
			sprd_img_print_reg(dcamfile->idx);
		}
		break;

	case SPRD_IMG_GET_PATH_CAP:
		/*DCAM_TRACE("get path capbility\n");*/
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
		pr_err("invalid cmd\n");
		put_dcam_lock(DCAM_RUNNING);
		return -EINVAL;
	}

read_end:

	if (copy_to_user((void __user *)u_data, &read_op, cnt))
		ret = -EFAULT;

	if (ret)
		cnt = ret;

	put_dcam_lock(DCAM_RUNNING);
	return cnt;
}

static ssize_t sprd_img_write(struct file *file, const char __user *u_data,
			      size_t cnt, loff_t *cnt_ret)
{
	struct dcam_file *dcamfile = file->private_data;
	struct dcam_dev *dev = NULL;
	struct dcam_info *info = &dev->dcam_cxt;
	struct dcam_path_spec *path = NULL;
	struct sprd_img_write_op write_op;
	unsigned int index;
	int ret = 0;

	if (cnt != sizeof(struct sprd_img_write_op)) {
		pr_err("err, cnt %ld read_op %ld\n", cnt,
		       sizeof(struct sprd_img_write_op));
		return -EIO;
	}

	if (copy_from_user(&write_op, (void __user *)u_data, cnt)) {
		pr_err("failed to get user info\n");
		return -EFAULT;
	}

	dev = get_dcam_dev(dcamfile);
	if (!dev)
		return -EFAULT;

	get_dcam_lock(DCAM_RUNNING);
	switch (write_op.cmd) {
	case SPRD_IMG_STOP_DCAM:
		ret = sprd_img_tx_stop(dcamfile);
		break;

	case SPRD_IMG_FREE_FRAME:
		if (atomic_read(&dev->stream_on) == 0) {
			DCAM_TRACE("dev close, no need free!");
			break;
		}

		switch (write_op.channel_id) {
		case DCAM_PATH0:
		case DCAM_PATH1:
		case DCAM_PATH2:
		case DCAM_PATH3:
			path = &info->dcam_path[write_op.channel_id];
			break;
		default:
			pr_err("failed to free frame buf, channel %d\n",
			       write_op.channel_id);
			ret = -EINVAL;
			break;
		}

		if (path->status == PATH_IDLE) {
			DCAM_TRACE("failed to free frame buf, channel %d\n",
				   write_op.channel_id);
			ret = -EINVAL;
			break;
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

	put_dcam_lock(DCAM_RUNNING);
	return cnt;
}

static const struct file_operations image_fops = {
	.owner = THIS_MODULE,
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
	struct dcam_group *group = NULL;

	if (sprd_dcam_drv_init(pdev)) {
		pr_err("failed to call sprd_dcam_drv_init\n");
		return -1;
	}

	ret = misc_register(&image_dev);
	if (ret) {
		pr_err("failed to register misc devices, ret %d\n", ret);
		ret = -EACCES;
		goto exit;
	}
	image_dev.this_device->of_node = pdev->dev.of_node;
	sprd_dcam_parse_regbase(pdev);
	sprd_dcam_parse_clk(pdev);
	sprd_dcam_parse_irq(pdev);

	group = vzalloc(sizeof(struct dcam_group));
	if (group == NULL)
		return -ENOMEM;

	image_dev.this_device->platform_data = (void *)group;
	group->md = &image_dev;
	group->pdev = pdev;
	atomic_set(&group->dcam_opened, 0);
	s_pdev = pdev;

	sprd_cam_pw_domain_init(pdev);
	wake_lock_init(&group->wakelock, WAKE_LOCK_SUSPEND,
			"pm_message_wakelock_dcam");

exit:
	return ret;
}

static int sprd_img_remove(struct platform_device *pdev)
{
	struct dcam_group *group = image_dev.this_device->platform_data;

	if (group == NULL)
		return -ENOMEM;

	wake_lock_destroy(&group->wakelock);
	vfree(image_dev.this_device->platform_data);
	misc_deregister(&image_dev);
	sprd_dcam_drv_deinit();

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
	sprd_img_setflash(DCAM_ID_1, &set_flash);
}

static const struct of_device_id sprd_dcam_of_match[] = {
	{ .compatible = "sprd,dcam-r2p0", },
	{},
};

static struct platform_driver sprd_img_driver = {
	.probe = sprd_img_probe,
	.remove = sprd_img_remove,
	.shutdown = sprd_img_shutdown,
	.driver = {
		.name = IMG_DEVICE_NAME,
		.of_match_table = of_match_ptr(sprd_dcam_of_match),
	},
};

module_platform_driver(sprd_img_driver);

MODULE_DESCRIPTION("Sprd DCAM Driver");
MODULE_AUTHOR("Multimedia_Camera@Spreadtrum");
MODULE_LICENSE("GPL");
