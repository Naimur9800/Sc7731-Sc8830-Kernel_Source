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

#ifndef _DCAM_CORE_HEADER_
#define _DCAM_CORE_HEADER_

#include <linux/wakelock.h>
#include <video/sprd_img.h>
#include <video/sprd_mm.h>
#include "sprd_isp_hw.h"
#include "flash_drv.h"
#include "dcam_block.h"
#include "dcam_drv.h"
#include "isp_drv.h"
#include "isp_statis_buf.h"

#define IMG_DEVICE_NAME		"sprd_image"
#define CAMERA_INVALID_FOURCC	0xFFFFFFFF
#define CAMERA_QUEUE_LENGTH	96
#define CAMERA_TIMING_LEN	16

#define CAMERA_ZOOM_LEVEL_MAX	4
#define CAMERA_ZOOM_STEP(x, y)	(((x) - (y)) / CAMERA_ZOOM_LEVEL_MAX)
#define CAMERA_PIXEL_ALIGNED	4
#define CAMERA_WIDTH(w)		((w) & ~(CAMERA_PIXEL_ALIGNED - 1))
#define CAMERA_HEIGHT(h)	((h) & ~(CAMERA_PIXEL_ALIGNED - 1))

#define DEF_DUMP2LOG_MAX_WORD_SIZE 0x200
#define SBLK_NAME(name) #name

enum {
	PATH_IDLE = 0x00,
	PATH_RUN,
};

/* Structure Definitions */

struct isp_sub_blk_base {
	const char *name;
	unsigned long base_addr;
};

/* TODO: add new items */
struct dcam_dbg_info {
	bool dbg_on;
};

/* TODO: add new items */
struct isp_dbg_info {
	bool dbg_on;
	bool fmcu_dbg_on;

	/*
	 * Use this value to record the max word size
	 * to dump the isp regs, and the remainings will
	 * be collected by another thread, now in todo list.
	 */
	unsigned int dump2log_max_word_size;

	/* start and end address, total 3 sets */
	unsigned int dump_range[6];

	/*
	 * used to save the bypass/work configs from sysfs,
	 * and map sub-blocks of isp, each bit for one
	 * sub-block.
	 */
	unsigned int sblk_maps[ISP_SBLK_MAP_CNT];
	unsigned int sblk_cnt;
	struct isp_sub_blk_base *sblk_base;
};

struct camera_dev_dbg_info {
	struct dcam_dbg_info dcam_dbg;
	struct isp_dbg_info isp_dbg;
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
	struct sprd_img_vcm_dac_info dac_info;
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
	unsigned int is_loose;
	unsigned int rot_mode;
	unsigned int status;
	enum isp_path_mode path_mode;
	struct camera_size in_size;
	struct camera_path_dec img_deci;
	struct camera_rect in_rect;
	struct camera_size out_size;
	struct camera_size isp_out_size;
	enum dcam_fmt out_fmt;
	struct camera_endian_sel end_sel;
	unsigned int fourcc;
	unsigned int frm_id_base;
	unsigned int frm_type;
	unsigned int index[DCAM_FRM_CNT_MAX];
	struct camera_img_buf_queue buf_queue;
	struct camera_img_buf_queue tmp_buf_queue;
	struct camera_addr frm_reserved_addr;
	struct camera_addr frm_reserved_addr_vir;
	unsigned int frm_cnt_act;
	unsigned int path_frm_deci;
	unsigned int path_skip_num;
	struct dcam_regular_desc regular_desc;
	struct sprd_pdaf_control pdaf_ctrl;
	struct sprd_ebd_control ebd_ctrl;
	unsigned int bin_ratio;
	unsigned int isp_fetch_fmt;
};

struct camera_pulse_queue {
	struct sprd_img_vcm_param node[CAMERA_QUEUE_LENGTH];
	struct sprd_img_vcm_param *write;
	struct sprd_img_vcm_param *read;
	unsigned int wcnt;
	unsigned int rcnt;
	spinlock_t lock;
};

struct camera_pulse_type {
	struct workqueue_struct *pulse_work_queue;
	struct work_struct pulse_work;
	uint32_t pulse_line;
	int32_t last_vcm_pos;
	uint32_t enable_debug_info;
	struct sprd_img_vcm_dac_info dac_info;
	struct camera_pulse_queue vcm_queue;
	struct mutex pulse_mutex;
};

struct camera_info {
	unsigned int if_mode;
	unsigned int sn_mode;
	unsigned int img_ptn;
	unsigned int data_bits;
	unsigned int is_loose;
	struct dcam_cap_sync_pol sync_pol;
	unsigned int frm_deci;
	struct dcam_cap_dec img_deci;
	struct camera_size cap_in_size;
	struct camera_rect cap_in_rect;
	struct camera_size cap_out_size;
	struct camera_size small_size;
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
	struct camera_pulse_type pulse_info;
	unsigned int is_3dnr;
	unsigned int is_hdr;
	struct sprd_flash_capacity capacity;
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
	unsigned int channel_id;
	unsigned int use_path;
	enum dcam_id idx;
	struct isp_statis_buf_input init_inptr;
	void *isp_dev_handle;
	unsigned int cap_flag;
	struct dcam_statis_module statis_module_info;
	struct camera_frame *frame;
	unsigned int is_simulation_mode;
	unsigned int frame_id;
	unsigned int bin_frame_id;
	unsigned int afm_delta_frame;
};

struct camera_group {
	unsigned int dev_inited;
	unsigned int mode_inited;
	unsigned int dcam_res_used;
	unsigned int dcam_count;
	unsigned int isp_count;
	atomic_t camera_opened;
	struct camera_dev *dev[DCAM_ID_MAX];
	struct ion_client *cam_ion_client[DCAM_ID_MAX];
	struct platform_device *pdev;
	struct camera_dev_dbg_info dbg_info;
	struct wake_lock wakelock;
};

struct camera_file {
	int idx;
	struct camera_group *grp;
};

int img_get_timestamp(struct timeval *tv);

#endif /* _DCAM_CORE_H_ */

