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

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/sprd_ion.h>
#include <linux/vmalloc.h>
#include <linux/videodev2.h>
#include <linux/wakelock.h>
#include <video/sprd_mm.h>
#include "dcam_drv.h"
#include "gen_scale_coef.h"
#include "getyuvinput.h"
#include "cam_pw_domain.h"
#include "isp_drv.h"

/* Macro Definitions */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_DRV: %d: %d " fmt, current->pid, __LINE__

#define DCAM_DRV_DEBUG

#define DCAM_SC1_H_TAB_OFFSET			0x400
#define DCAM_SC1_V_TAB_OFFSET			0x4F0
#define DCAM_SC1_V_CHROMA_TAB_OFFSET		0x8F0

#define DCAM_SC2_H_TAB_OFFSET			0x1400
#define DCAM_SC2_V_TAB_OFFSET			0x14F0
#define DCAM_SC2_V_CHROMA_TAB_OFFSET		0x18F0

#define DCAM_SC3_H_TAB_OFFSET			0x2400
#define DCAM_SC3_V_TAB_OFFSET			0x24F0
#define DCAM_SC3_V_CHROMA_TAB_OFFSET		0x28F0

#define DCAM_SC_COEFF_COEF_SIZE			(1 << 10)
#define DCAM_SC_COEFF_TMP_SIZE			(21 << 10)

#define DCAM_SC_H_COEF_SIZE			0xC0
#define DCAM_SC_V_COEF_SIZE			0x210
#define DCAM_SC_V_CHROM_COEF_SIZE		0x210

#define DCAM_SC_COEFF_H_NUM			(DCAM_SC_H_COEF_SIZE/4)
#define DCAM_SC_COEFF_V_NUM			(DCAM_SC_V_COEF_SIZE/4)
#define DCAM_SC_COEFF_V_CHROMA_NUM		(DCAM_SC_V_CHROM_COEF_SIZE/4)

#define DCAM_CLK_NUM			4

#define DCAM_AXI_STOP_TIMEOUT			1000

#ifdef CONFIG_SC_FPGA
#define DCAM_PATH_TIMEOUT			msecs_to_jiffies(500*10)
#else
#define DCAM_PATH_TIMEOUT			msecs_to_jiffies(500)
#endif

#define DCAM_FRM_QUEUE_LENGTH			4
#define DCAM_STATE_QUICKQUIT			0x01
#define DCAM_MAX_COUNT				2

#define DCAM_IRQ_ERR_MASK \
	((1 << DCAM_PATH0_OVF) | (1 << DCAM_PDAF_BUF_OVF) | \
	(1 << DCAM_CAP_MIPI_OVF) | (1 << DCAM_CAP_LINE_ERR) | \
	(1 << DCAM_CAP_FRM_ERR) | (1 << DCAM_JPG_BUF_OVF) | \
	(1 << DCAM_ISP_FIFO_OVF))

#define DCAM_IRQ_LINE_MASK \
	((1 << DCAM_CAP_SOF) | (1 << DCAM_PATH0_SOF) | \
	(1 << DCAM_PDAF_SN_SOF) | (1 << DCAM_PDAF_SN_EOF) | \
	(1 << DCAM_PDAF_CAP_SOF) | (1 << DCAM_SN_EOF) | \
	(1 << DCAM_CAP_EOF) | (1 << DCAM_PATH0_TX_DONE) | \
	(1 << DCAM_PDAF_CAP_EOF) | (1 << DCAM_PDAF_TX_DONE) | \
	(1 << DCAM_PATH0_AXIM_DONE) | DCAM_IRQ_ERR_MASK)

/* Structure Definitions */

typedef void (*dcam_isr) (enum dcam_id idx);

enum {
	DCAM_FRM_UNLOCK = 0,
	DCAM_FRM_LOCK_WRITE = 0x10011001,
	DCAM_FRM_LOCK_READ = 0x01100110
};

enum {
	DCAM_ST_STOP = 0,
	DCAM_ST_START,
};

static const struct dcam_if_clk_tag dcam_if_clk_tab[DCAM_CLK_NUM] = {
	{77, "dcam_clk_76m8"},
	{256, "dcam_clk_256m"},
	{307, "dcam_clk_307m2"},
	{384, "dcam_clk_384m"},
};

struct dcam_cap_desc {
	enum dcam_cap_if_mode cam_if;
	enum dcam_cap_sensor_mode input_format;
	enum dcam_capture_mode cap_mode;
	struct camera_rect cap_rect;
};

struct dcam_path_valid {
	unsigned int input_rect:1;
	unsigned int output_format:1;
	unsigned int src_sel:1;
	unsigned int data_endian:1;
	unsigned int frame_deci:1;
	unsigned int v_deci:1;
	unsigned int pdaf_ctrl:1;
};

struct dcam_frm_queue {
	struct camera_frame frm_array[DCAM_FRM_QUEUE_LENGTH];
	unsigned int valid_cnt;
};

struct dcam_buf_queue {
	struct camera_frame frame[DCAM_FRM_CNT_MAX];
	struct camera_frame *write;
	struct camera_frame *read;
	int w_index;
	int r_index;
	spinlock_t lock;
};

struct dcam_path_desc {
	enum camera_path_id id;
	struct camera_size input_size;
	struct camera_rect input_rect;
	struct camera_size sc_input_size;
	struct camera_size output_size;
	struct dcam_frm_queue frame_queue;
	struct dcam_buf_queue buf_queue;
	struct camera_endian_sel data_endian;
	struct camera_deci deci_val;
	struct dcam_path_valid valid_param;
	unsigned int frame_base_id;
	unsigned int output_frame_count;
	unsigned int output_format;
	enum dcam_path_src_sel src_sel;
	unsigned int frame_deci;
	unsigned int valid;
	unsigned int status;
	struct completion tx_done_com;
	struct completion sof_com;
	unsigned int wait_for_done;
	unsigned int is_update;
	unsigned int wait_for_sof;
	unsigned int need_stop;
	unsigned int need_wait;
	int sof_cnt;
	int done_cnt;
};

struct dcam_path_pdaf {
	struct sprd_pdaf_control pdaf_ctrl;
	struct dcam_buf_queue buf_queue;
	struct dcam_frm_queue frame_queue;
	unsigned int valid;
	unsigned int output_frame_count;
	unsigned int frame_base_id;
	unsigned int status;
};

struct dcam_module {
	struct dcam_cap_desc dcam_cap;
	struct dcam_path_desc dcam_path0;
	struct dcam_path_pdaf dcam_pdaf;
	struct camera_frame path0_reserved_frame;
	struct camera_frame pdaf_reserved_frame;
	struct camera_frame path0_frame;
	struct camera_frame pdaf_frame;
	unsigned int path0_framecnt;
	unsigned int pdaf_framecnt;

	unsigned int wait_resize_done;
	unsigned int err_happened;
	unsigned int state;
};

/* Static Variables Declaration */
static unsigned int s_dcam_count;
static struct platform_device *s_dcam_pdev;
static struct platform_device *s_dcam1_pdev;
static atomic_t s_dcam_users[DCAM_MAX_COUNT];
static struct dcam_module *s_p_dcam_mod[DCAM_MAX_COUNT];
static int s_dcam_irq[DCAM_MAX_COUNT];
static dcam_isr_func s_user_func[DCAM_MAX_COUNT][DCAM_IRQ_NUMBER];
static void *s_user_data[DCAM_MAX_COUNT][DCAM_IRQ_NUMBER];
static struct wake_lock dcam_wakelock[DCAM_MAX_COUNT];
static char dcam_wakelock_name[DCAM_MAX_COUNT][30];
static unsigned long s_dcam_regbase[DCAM_MAX_COUNT];
static struct mutex dcam_module_sema[DCAM_MAX_COUNT];

static spinlock_t dcam_mod_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_cfg_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_control_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_mask_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_clr_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_ahbm_sts_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_endian_lock[DCAM_MAX_COUNT];

static struct clk *dcam_clk[DCAM_MAX_COUNT];
static struct clk *dcam_clk_parent[DCAM_MAX_COUNT];
static struct clk *dcam_clk_default[DCAM_MAX_COUNT];
static struct clk *dcam_mg_eb[DCAM_MAX_COUNT];
static struct clk *dcam_d2i_mg_eb[DCAM_MAX_COUNT];
static struct clk *dcam_eb[DCAM_MAX_COUNT];

static struct regmap *cam_ahb_gpr;

const unsigned int s_irq_vect[] = {
	DCAM_SN_SOF,
	DCAM_SN_EOF,
	DCAM_CAP_SOF,
	DCAM_CAP_EOF,
	DCAM_PATH0_TX_DONE,
	DCAM_PATH0_OVF,
	DCAM_PDAF_SN_SOF,
	DCAM_PDAF_SN_EOF,
	DCAM_PDAF_CAP_SOF,
	DCAM_PDAF_CAP_EOF,
	DCAM_CAP_LINE_ERR,
	DCAM_CAP_FRM_ERR,
	DCAM_JPG_BUF_OVF,
	DCAM_ISP_FIFO_OVF,
	DCAM_CAP_MIPI_OVF,
	DCAM_PDAF_TX_DONE,
	DCAM_PDAF_BUF_OVF,
	DCAM_PDAF_END,
	DCAM_RESERVED,
	DCAM_PDAF_SOF,
	DCAM_PATH0_SOF,
	DCAM_PATH0_END,
	DCAM_PATH0_AXIM_DONE,
	DCAM_PDAF_AXIM_DONE,
};

static unsigned int dcam_err_count;
static unsigned long g_dcam_err_irq_cnt[DCAM_MAX_COUNT][DCAM_IRQ_NUMBER];

/* Internal Function Implementation */
static void dcam_buf_queue_init(struct dcam_buf_queue *queue)
{
	if (DCAM_ADDR_INVALID(queue)) {
		pr_err("invalid heap %p\n", queue);
		return;
	}

	memset((void *)queue, 0, sizeof(struct dcam_buf_queue));
	queue->write = &queue->frame[0];
	queue->read = &queue->frame[0];
	spin_lock_init(&queue->lock);
}

static int dcam_buf_queue_read(struct dcam_buf_queue *queue,
			       struct camera_frame *frame)
{
	int ret = DCAM_RTN_SUCCESS;
	unsigned long flags;

	if (DCAM_ADDR_INVALID(queue) || DCAM_ADDR_INVALID(frame)) {
		pr_err("deq, invalid parm %p, %p\n", queue, frame);
		return -EINVAL;
	}

	DCAM_TRACE("read buf queue\n");

	spin_lock_irqsave(&queue->lock, flags);
	if (queue->read != queue->write) {
		*frame = *queue->read++;
		queue->r_index++;
		if (queue->read > &queue->frame[DCAM_FRM_CNT_MAX - 1]) {
			queue->read = &queue->frame[0];
			queue->r_index = 0;
		}
	} else {
		ret = -EAGAIN;
		DCAM_TRACE("warning, read wait new node write\n");
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	DCAM_TRACE("read buf queue %d index %x\n",
		   frame->type, queue->r_index);

	return ret;
}

static int dcam_buf_queue_write(struct dcam_buf_queue *queue,
				struct camera_frame *frame)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_frame *ori_frame;
	unsigned long flags;

	if (DCAM_ADDR_INVALID(queue) || DCAM_ADDR_INVALID(frame)) {
		pr_err("enq, invalid parm %p, %p\n", queue, frame);
		return -EINVAL;
	}

	DCAM_TRACE("write buf queue\n");
	spin_lock_irqsave(&queue->lock, flags);

	ori_frame = queue->write;
	*queue->write++ = *frame;
	queue->w_index++;
	if (queue->write > &queue->frame[DCAM_FRM_CNT_MAX - 1]) {
		queue->write = &queue->frame[0];
		queue->w_index = 0;
	}

	if (queue->write == queue->read) {
		queue->write = ori_frame;
		pr_warn("warning, queue is full, can't write 0x%x\n",
			frame->yaddr);
		ret = -EAGAIN;
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	DCAM_TRACE("write buf queue type %d index %x\n",
		   frame->type, queue->w_index);

	return ret;
}

static void dcam_frm_queue_clear(struct dcam_frm_queue *queue)
{
	if (DCAM_ADDR_INVALID(queue)) {
		pr_err("invalid heap %p\n", queue);
		return;
	}

	memset((void *)queue, 0, sizeof(struct dcam_frm_queue));
}

static int dcam_frame_enqueue(struct dcam_frm_queue *queue,
			      struct camera_frame *frame)
{
	if (DCAM_ADDR_INVALID(queue) || DCAM_ADDR_INVALID(frame)) {
		pr_err("enq, invalid parm %p, %p\n", queue, frame);
		return -1;
	}

	if (queue->valid_cnt >= DCAM_FRM_QUEUE_LENGTH) {
		pr_info("q over flow\n");
		return -1;
	}

	memcpy(&queue->frm_array[queue->valid_cnt], frame,
	       sizeof(struct camera_frame));
	queue->valid_cnt++;
	DCAM_TRACE("en queue, %d, %d, 0x%x, 0x%x\n",
		   (0xF & frame->fid),
		   queue->valid_cnt, frame->yaddr, frame->uaddr);

	return 0;
}

static int dcam_frame_dequeue(struct dcam_frm_queue *queue,
			      struct camera_frame *frame)
{
	unsigned int i = 0;

	if (DCAM_ADDR_INVALID(queue) || DCAM_ADDR_INVALID(frame)) {
		pr_err("deq, invalid parm %p, %p\n", queue, frame);
		return -1;
	}

	if (queue->valid_cnt == 0) {
		DCAM_TRACE("q under flow\n");
		return -1;
	}

	memcpy(frame, &queue->frm_array[0], sizeof(struct camera_frame));
	queue->valid_cnt--;
	for (i = 0; i < queue->valid_cnt; i++) {
		memcpy(&queue->frm_array[i], &queue->frm_array[i + 1],
		       sizeof(struct camera_frame));
	}
	DCAM_TRACE("de queue, %d, %d\n",
		   (0xF & (frame)->fid), queue->valid_cnt);

	return 0;
}

static int dcam_path0_set_next_frm(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct camera_frame frame;
	struct camera_frame *reserved_frame = NULL;
	struct dcam_path_desc *path0 = NULL;
	unsigned long reg = 0;
	unsigned int addr = 0;
	unsigned int path_max_frm_cnt;
	struct dcam_frm_queue *p_heap = NULL;
	struct dcam_buf_queue *p_buf_queue = NULL;
	unsigned int output_frame_count = 0;
	int use_reserve_frame = 0;
	struct dcam_module *module;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	module = s_p_dcam_mod[idx];
	reserved_frame = &module->path0_reserved_frame;
	path0 = &module->dcam_path0;
	reg = DCAM_FRM_ADDR0;
	path_max_frm_cnt = DCAM_PATH_0_FRM_CNT_MAX;
	p_heap = &path0->frame_queue;
	p_buf_queue = &path0->buf_queue;
	output_frame_count = path0->output_frame_count;

	if (dcam_buf_queue_read(p_buf_queue, &frame) == 0 &&
	    (frame.pfinfo.mfd[0] != 0)) {
		path0->output_frame_count--;
	} else {
		pr_info("DCAM%d: No freed frame id %d path0\n",
			idx, frame.fid);
		if (reserved_frame->pfinfo.mfd[0] == 0) {
			pr_info("DCAM%d: No need to cfg frame buffer", idx);
			return -1;
		}
		memcpy(&frame, reserved_frame, sizeof(struct camera_frame));
		use_reserve_frame = 1;
	}

	DCAM_TRACE("DCAM%d: reserved %d y 0x%x mfd 0x%x path0\n",
		   idx, use_reserve_frame, frame.yaddr,
		   frame.pfinfo.mfd[0]);

	if (frame.pfinfo.dev == NULL)
		pr_info("DCAM%d next dev NULL %p\n", idx, frame.pfinfo.dev);
	if (pfiommu_get_addr(&frame.pfinfo) != 0) {
		pr_err("DCAM%d path0 get frame iommu address failed!\n", idx);
		rtn = DCAM_RTN_PATH_ADDR_ERR;
		return rtn;
	}

	addr = frame.pfinfo.iova[0] + frame.yaddr;

	if (use_reserve_frame)
		memcpy(reserved_frame, &frame, sizeof(struct camera_frame));

	DCAM_REG_WR(idx, reg, addr);

	if (dcam_frame_enqueue(p_heap, &frame) == 0)
		DCAM_TRACE("success to enq frame buf\n");
	else
		rtn = DCAM_RTN_PATH_FRAME_LOCKED;

	return -rtn;
}

static int dcam_pdaf_set_next_frm(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct camera_frame frame;
	struct camera_frame *reserved_frame = NULL;
	struct dcam_path_pdaf *pdaf = NULL;
	unsigned long reg = 0;
	unsigned int addr = 0;
	unsigned int path_max_frm_cnt;
	struct dcam_frm_queue *p_heap = NULL;
	struct dcam_buf_queue *p_buf_queue = NULL;
	unsigned int output_frame_count = 0;
	int use_reserve_frame = 0;
	struct dcam_module *module;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	module = s_p_dcam_mod[idx];
	reserved_frame = &module->pdaf_reserved_frame;
	pdaf = &module->dcam_pdaf;
	reg = FRM_ADDR_PDAF;
	path_max_frm_cnt = DCAM_PATH_0_FRM_CNT_MAX;
	p_heap = &pdaf->frame_queue;
	p_buf_queue = &pdaf->buf_queue;
	output_frame_count = pdaf->output_frame_count;

	if (dcam_buf_queue_read(p_buf_queue, &frame) == 0 &&
	    (frame.pfinfo.mfd[0] != 0)) {
		pdaf->output_frame_count--;
	} else {
		pr_info("DCAM%d: No freed frame id %d path0\n",
			idx, frame.fid);
		if (reserved_frame->pfinfo.mfd[0] == 0) {
			pr_info("DCAM%d: No need to cfg frame buffer", idx);
			return -1;
		}
		memcpy(&frame, reserved_frame, sizeof(struct camera_frame));
		use_reserve_frame = 1;
	}

	DCAM_TRACE("DCAM%d: reserved %d y 0x%x mfd 0x%x path0\n",
		   idx, use_reserve_frame, frame.yaddr,
		   frame.pfinfo.mfd[0]);

	if (frame.pfinfo.dev == NULL)
		pr_info("DCAM%d next dev NULL %p\n", idx, frame.pfinfo.dev);
	if (pfiommu_get_addr(&frame.pfinfo) != 0) {
		pr_err("DCAM%d pdaf get frame iommu address failed!\n", idx);
		rtn = DCAM_RTN_PATH_ADDR_ERR;
		return rtn;
	}

	addr = frame.pfinfo.iova[0];

	if (use_reserve_frame)
		memcpy(reserved_frame, &frame, sizeof(struct camera_frame));

	DCAM_REG_WR(idx, reg, addr);

	if (dcam_frame_enqueue(p_heap, &frame) == 0)
		DCAM_TRACE("success to enq frame buf\n");
	else
		rtn = DCAM_RTN_PATH_FRAME_LOCKED;

	return -rtn;
}

static void dcam_force_copy(enum dcam_id idx)
{
	sprd_dcam_glb_reg_mwr(idx, DCAM_CONTROL, BIT_0, 1 << 0,
				      DCAM_CONTROL_REG);
	sprd_dcam_glb_reg_mwr(idx, DCAM_CONTROL, BIT_8, 1 << 8,
				      DCAM_CONTROL_REG);
}

static void dcam_auto_copy(enum dcam_id idx)
{
	sprd_dcam_glb_reg_mwr(idx, DCAM_CONTROL, BIT_1, 1 << 1,
				      DCAM_CONTROL_REG);
	sprd_dcam_glb_reg_mwr(idx, DCAM_CONTROL, BIT_9, 1 << 9,
				      DCAM_CONTROL_REG);
}
static void dcam_reg_trace(enum dcam_id idx)
{
#ifdef DCAM_DRV_DEBUG
	unsigned long addr = 0;

	if (likely(dcam_err_count < 10)) {
		pr_info("DCAM%d: Register list", idx);
		for (addr = DCAM_CFG; addr <= FRM_ADDR_PDAF; addr += 16) {
			pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
				addr,
				DCAM_REG_RD(idx, addr),
				DCAM_REG_RD(idx, addr + 4),
				DCAM_REG_RD(idx, addr + 8),
				DCAM_REG_RD(idx, addr + 12));
		}
	}

#endif
}
static void dcam_path0_done_notice(enum dcam_id idx)
{
	struct dcam_path_desc *p_path0 = NULL;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	p_path0 = &s_p_dcam_mod[idx]->dcam_path0;

	DCAM_TRACE("DCAM%d: path done notice %d, %d\n", idx,
		   p_path0->wait_for_done, p_path0->tx_done_com.done);
	if (p_path0->wait_for_done) {
		complete(&p_path0->tx_done_com);
		pr_info("release tx_done_com: %d\n",
			p_path0->tx_done_com.done);
		p_path0->wait_for_done = 0;
	}
}

static void dcam_sensor_sof(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_SN_SOF];
	void *data = s_user_data[idx][DCAM_SN_SOF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_sensor_eof(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_SN_EOF];
	void *data = s_user_data[idx][DCAM_SN_EOF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_cap_sof(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_CAP_SOF];
	void *data = s_user_data[idx][DCAM_CAP_SOF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer, idx:%d\n", idx);
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_cap_eof(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_CAP_EOF];
	void *data = s_user_data[idx][DCAM_CAP_EOF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_path0_overflow(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_PATH0_OVF];
	void *data = s_user_data[idx][DCAM_PATH0_OVF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	pr_info("DCAM%d: dcam path0 overflow\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_pdaf_sensor_sof(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_PDAF_SN_SOF];
	void *data = s_user_data[idx][DCAM_PDAF_SN_SOF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_pdaf_sensor_eof(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_PDAF_SN_EOF];
	void *data = s_user_data[idx][DCAM_PDAF_SN_EOF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_pdaf_cap_sof(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_PDAF_CAP_SOF];
	void *data = s_user_data[idx][DCAM_PDAF_CAP_SOF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_pdaf_cap_eof(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_PDAF_CAP_EOF];
	void *data = s_user_data[idx][DCAM_PDAF_CAP_EOF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_cap_line_err(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_CAP_LINE_ERR];
	void *data = s_user_data[idx][DCAM_CAP_LINE_ERR];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	pr_info("DCAM%d: line err\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_cap_frm_err(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_CAP_FRM_ERR];
	void *data = s_user_data[idx][DCAM_CAP_FRM_ERR];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	pr_info("DCAM%d: frame err\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_isp_overflow(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_ISP_FIFO_OVF];
	void *data = s_user_data[idx][DCAM_ISP_FIFO_OVF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	pr_info("DCAM%d: isp overflow\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_mipi_overflow(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_CAP_MIPI_OVF];
	void *data = s_user_data[idx][DCAM_CAP_MIPI_OVF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	pr_info("DCAM%d: mipi overflow\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_pdaf_overflow(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_PDAF_BUF_OVF];
	void *data = s_user_data[idx][DCAM_PDAF_BUF_OVF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	pr_info("DCAM%d: pdaf overflow\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_pdaf_end(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_PDAF_END];
	void *data = s_user_data[idx][DCAM_PDAF_END];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_pdaf_sof(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	dcam_isr_func user_func;
	void *data;
	struct dcam_path_pdaf *pdaf = NULL;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx]))
		return;

	user_func = s_user_func[idx][DCAM_PDAF_SOF];
	data = s_user_data[idx][DCAM_PDAF_SOF];

	if (s_p_dcam_mod[idx]->dcam_pdaf.status == DCAM_ST_START) {
		pdaf = &s_p_dcam_mod[idx]->dcam_pdaf;
		if (pdaf->valid == 0) {
			pr_info("DCAM%d: path pdaf not valid\n", idx);
			return;
		}

		rtn = dcam_pdaf_set_next_frm(idx);
		dcam_auto_copy(idx);

		if (user_func)
			(*user_func) (NULL, data);
	}
}

static void dcam_path0_end(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_PATH0_END];
	void *data = s_user_data[idx][DCAM_PATH0_END];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_pdaf_done(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_PDAF_AXIM_DONE];
	void *data = s_user_data[idx][DCAM_PDAF_AXIM_DONE];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_path0_done(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	dcam_isr_func user_func;
	void *data;
	struct dcam_path_desc *path0;
	struct dcam_module *module;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx]))
		return;

	module = s_p_dcam_mod[idx];
	path0 = &module->dcam_path0;
	if (path0->valid == 0) {
		pr_info("DCAM%d: path0 not valid\n", idx);
		return;
	}

	if (path0->frame_deci > 1) {
		if (path0->done_cnt == 0) {
			path0->done_cnt = 1;
		} else {
			path0->done_cnt = 0;
			DCAM_TRACE("DCAM%d: path0 dummy done, drop\n",
					idx);
			return;
		}
	} else {
		path0->done_cnt = 1;
	}

	user_func = s_user_func[idx][DCAM_PATH0_AXIM_DONE];
	data = s_user_data[idx][DCAM_PATH0_AXIM_DONE];
	DCAM_TRACE("dcam%d %p\n", idx, data);

	if (path0->need_stop) {
		sprd_dcam_glb_reg_awr(idx, DCAM_CFG, ~(1 << 0),
					      DCAM_CFG_REG);
		path0->need_stop = 0;
	}
	dcam_path0_done_notice(idx);

	if (path0->need_wait) {
		path0->need_wait = 0;
	} else {
		struct camera_frame frame;

		rtn = dcam_frame_dequeue(&path0->frame_queue, &frame);
		if (rtn)
			return;
		if (frame.pfinfo.dev == NULL)
			pr_info("DCAM%d done dev NULL %p\n",
				idx, frame.pfinfo.dev);
		pfiommu_free_addr(&frame.pfinfo);
		if (frame.pfinfo.mfd[0] !=
		    module->path0_reserved_frame.pfinfo.mfd[0]) {
			frame.width = path0->output_size.w;
			frame.height = path0->output_size.h;
			frame.irq_type = CAMERA_IRQ_IMG;

			DCAM_TRACE("DCAM%d: path0 frame %p\n",
				   idx, &frame);
			DCAM_TRACE("y uv, 0x%x 0x%x, mfd = 0x%x,0x%x\n",
				   frame.yaddr, frame.uaddr,
				   frame.pfinfo.mfd[0], frame.pfinfo.mfd[1]);

			if (user_func)
				(*user_func) (&frame, data);
		} else {
			DCAM_TRACE("DCAM%d: use reserved path0\n", idx);
		}
	}
}

static void dcam_path0_sof(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	dcam_isr_func user_func;
	void *data;
	struct dcam_path_desc *path0 = NULL;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx]))
		return;

	user_func = s_user_func[idx][DCAM_PATH0_SOF];
	data = s_user_data[idx][DCAM_PATH0_SOF];

	if (s_p_dcam_mod[idx]->dcam_path0.status == DCAM_ST_START) {
		path0 = &s_p_dcam_mod[idx]->dcam_path0;
		if (path0->valid == 0) {
			pr_info("DCAM%d: path0 not valid\n", idx);
			return;
		}

		if (path0->sof_cnt == 1 + path0->frame_deci)
			path0->sof_cnt = 0;

		path0->sof_cnt++;
		if (path0->sof_cnt == 1) {
			DCAM_TRACE("DCAM%d: path0 sof %d frame_deci %d\n",
				idx, path0->sof_cnt, path0->frame_deci);
		} else {
			DCAM_TRACE("DCAM%d: path0 invalid sof, cnt %d\n",
				idx, path0->sof_cnt);
			return;
		}

		rtn = dcam_path0_set_next_frm(idx);
		dcam_auto_copy(idx);

		if (rtn) {
			path0->need_wait = 1;
			pr_info("DCAM%d: path0\n", idx);
			return;
		}
		if (user_func)
			(*user_func) (NULL, data);
	}
}

static void save_err_irq_cnt(enum dcam_id idx, unsigned int irq_status)
{
	int i;

	for (i = 0; i < DCAM_IRQ_NUMBER; i++) {
		if ((1 << i) & irq_status) {
			g_dcam_err_irq_cnt[idx][i]++;
		}
	}
}

static int dcam_err_pre_proc(enum dcam_id idx, unsigned int irq_status)
{
	unsigned long flag;

	spin_lock_irqsave(&dcam_mod_lock[idx], flag);

	save_err_irq_cnt(idx, irq_status);
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		printk_ratelimited(KERN_ERR "zero pointer, irq_status:0x%x\n", irq_status);
		spin_unlock_irqrestore(&dcam_mod_lock[idx], flag);
		return -1;
	}

	DCAM_TRACE("DCAM%d: state in err_pre_proc 0x%x\n", idx,
			s_p_dcam_mod[idx]->state);

	if (s_p_dcam_mod[idx]->state & DCAM_STATE_QUICKQUIT) {
		spin_unlock_irqrestore(&dcam_mod_lock[idx], flag);
		return -1;
	}

	s_p_dcam_mod[idx]->err_happened = 1;

	pr_info("DCAM%d: err, 0x%x\n", idx, irq_status);

	spin_unlock_irqrestore(&dcam_mod_lock[idx], flag);

	isp_reg_trace(idx);
	dcam_reg_trace(idx);

	sprd_dcam_glb_reg_mwr(idx, DCAM_CONTROL, BIT_2, 0, DCAM_CONTROL_REG);

	spin_lock_irqsave(&dcam_mod_lock[idx], flag);
	sprd_dcam_stop(idx, 1);
	spin_unlock_irqrestore(&dcam_mod_lock[idx], flag);

	return 0;
}

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
void sprd_dcam_cpp_error_notify(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_CPP_ERR];
	void *data = s_user_data[idx][DCAM_CPP_ERR];

	pr_err("DCAM_DRV cpp error idx %d\n", idx);
	dcam_err_pre_proc(idx, 0);


	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);

}
#endif

static dcam_isr isr_list[DCAM_MAX_COUNT][DCAM_IRQ_NUMBER] = {
	{
		dcam_sensor_sof,
		dcam_sensor_eof,
		dcam_cap_sof,
		dcam_cap_eof,
		NULL,
		dcam_path0_overflow,
		dcam_pdaf_sensor_sof,
		dcam_pdaf_sensor_eof,
		dcam_pdaf_cap_sof,
		dcam_pdaf_cap_eof,
		dcam_cap_line_err,
		dcam_cap_frm_err,
		NULL,
		dcam_isp_overflow,
		dcam_mipi_overflow,
		NULL,
		dcam_pdaf_overflow,
		dcam_pdaf_end,
		NULL,
		dcam_pdaf_sof,
		dcam_path0_sof,
		dcam_path0_end,
		dcam_path0_done,
		dcam_pdaf_done,
		NULL,
		NULL,
		NULL,
		NULL,
	},
};

static irqreturn_t dcam_isr_root(int irq, void *priv)
{
	unsigned int irq_line, status;
	unsigned long flag;
	unsigned long flag_mod;
	int i;
	enum dcam_id idx = DCAM_ID_0;
	int irq_numbers = ARRAY_SIZE(s_irq_vect);
	unsigned int vect = 0;

	if (s_dcam_irq[DCAM_ID_0] == irq)
		idx = DCAM_ID_0;
	else if (s_dcam_irq[DCAM_ID_1] == irq)
		idx = DCAM_ID_1;
	else
		return IRQ_NONE;

	status = DCAM_REG_RD(idx, DCAM_INT_STS) & DCAM_IRQ_LINE_MASK;
	if (unlikely(status == 0))
		return IRQ_NONE;
	DCAM_REG_WR(idx, DCAM_INT_CLR, status);

	irq_line = status;
	if (unlikely(DCAM_IRQ_ERR_MASK & status))
		if (dcam_err_pre_proc(idx, status))
			return IRQ_HANDLED;

	spin_lock_irqsave(&dcam_lock[idx], flag);
	spin_lock_irqsave(&dcam_mod_lock[idx], flag_mod);

	for (i = 0; i < irq_numbers; i++) {
		vect = s_irq_vect[i];
		if (irq_line & (1 << (unsigned int)vect)) {
			if (isr_list[idx][vect])
				isr_list[idx][vect](idx);
		}
		irq_line &= ~(unsigned int)(1 << (unsigned int)vect);
		if (!irq_line)
			break;
	}

	spin_unlock_irqrestore(&dcam_mod_lock[idx], flag_mod);
	spin_unlock_irqrestore(&dcam_lock[idx], flag);

	return IRQ_HANDLED;
}

static int dcam_internal_init(enum dcam_id idx)
{
	int ret = 0;

	s_p_dcam_mod[idx] = vzalloc(sizeof(struct dcam_module));
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	s_p_dcam_mod[idx]->dcam_path0.id = 0;
	init_completion(&s_p_dcam_mod[idx]->dcam_path0.tx_done_com);
	init_completion(&s_p_dcam_mod[idx]->dcam_path0.sof_com);
	dcam_buf_queue_init(&s_p_dcam_mod[idx]->dcam_path0.buf_queue);

	return ret;
}

int sprd_dcam_reg_isr(enum dcam_id idx, enum dcam_irq_id id,
		      dcam_isr_func user_func, void *user_data)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	unsigned long flag;

	if (id >= DCAM_IRQ_NUMBER) {
		rtn = DCAM_RTN_ISR_ID_ERR;
	} else {
		spin_lock_irqsave(&dcam_lock[idx], flag);
		s_user_func[idx][id] = user_func;
		s_user_data[idx][id] = user_data;
		spin_unlock_irqrestore(&dcam_lock[idx], flag);
	}

	return -rtn;
}

static int sprd_dcam_reset(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	unsigned int time_out = 0;
	unsigned int flag = 0;

	pr_info("DCAM%d: reset:\n", idx);

	if (atomic_read(&s_dcam_users[idx])) {
		/* firstly, stop AXI writing */
		sprd_dcam_glb_reg_owr(idx, DCAM_AMBA_STS, BIT_6,
			DCAM_AHBM_STS_REG);
	}

	/* then wait for AHB busy cleared */
	while (++time_out < DCAM_AXI_STOP_TIMEOUT) {
		if (0 == (DCAM_REG_RD(idx, DCAM_AXIM_DBG_STS) & BIT_2))
			break;
	}

	if (time_out >= DCAM_AXI_STOP_TIMEOUT) {
		pr_info("DCAM%d: reset timeout %d\n", idx, time_out);
		return DCAM_RTN_TIMEOUT;
	}

	if (idx == 0) {
		regmap_update_bits(cam_ahb_gpr,
			REG_MM_AHB_MODULE_RST, BIT_0, BIT_0);
	} else {
		regmap_update_bits(cam_ahb_gpr,
			REG_MM_AHB_MODULE_RST, BIT_1, BIT_1);
	}

	if (idx == 0)
		flag = BIT(0) | BIT(8) | BIT(18) | BIT(24);
	else
		flag = BIT(1) | BIT(12) | BIT(26);

	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, flag);
	udelay(1);
	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, ~flag);

	if (idx == 0) {
		regmap_update_bits(cam_ahb_gpr,
		  REG_MM_AHB_MODULE_RST, BIT_0, ~BIT_0);
	} else {
		flag = BIT(1) | BIT(12) | BIT(26);
		regmap_update_bits(cam_ahb_gpr,
		  REG_MM_AHB_MODULE_RST, BIT_1, ~BIT_1);
	}

	sprd_dcam_glb_reg_owr(idx, DCAM_INT_CLR,
		DCAM_IRQ_LINE_MASK, DCAM_INIT_CLR_REG);
	sprd_dcam_glb_reg_owr(idx, DCAM_INT_MASK,
		DCAM_IRQ_LINE_MASK, DCAM_INIT_MASK_REG);

	/* the end, enable AXI writing */
	sprd_dcam_glb_reg_awr(idx, DCAM_AMBA_STS, ~BIT_6,
		DCAM_AHBM_STS_REG);

	pr_info("DCAM%d: reset end\n", idx);

	return -rtn;
}

static void dcam_frm_clear(enum dcam_id idx)
{
	struct camera_frame frame, *res_frame;
	struct dcam_path_desc *path0;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	path0 = &s_p_dcam_mod[idx]->dcam_path0;
	while (!dcam_frame_dequeue(&path0->frame_queue, &frame))
		pfiommu_free_addr(&frame.pfinfo);

	dcam_frm_queue_clear(&path0->frame_queue);
	dcam_buf_queue_init(&path0->buf_queue);
	res_frame = &s_p_dcam_mod[idx]->path0_reserved_frame;
	pfiommu_free_addr(&res_frame->pfinfo);
	memset((void *)res_frame, 0, sizeof(struct camera_frame));
}

static void dcam_quickstop_path0(enum dcam_id idx)
{
	unsigned int ret = 0;
	int time_out = 5000;
	struct dcam_path_desc *path0 = NULL;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	path0 = &s_p_dcam_mod[idx]->dcam_path0;

	sprd_dcam_glb_reg_owr(idx, DCAM_AMBA_STS, BIT_3, DCAM_AHBM_STS_REG);
	udelay(1000);
	sprd_dcam_glb_reg_mwr(idx, DCAM_CFG, BIT_0, ~BIT_0, DCAM_CFG_REG);

	/* wait for AHB path busy cleared */
	while (path0->valid && time_out) {
		ret = DCAM_REG_RD(idx, DCAM_AMBA_STS) & BIT_5;
		if (!ret)
			break;
		time_out--;
	}

}

static void dcam_quickstop_pdaf(enum dcam_id idx)
{
	unsigned int ret = 0;
	int time_out = 5000;
	struct dcam_path_pdaf *pdaf = NULL;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	pdaf = &s_p_dcam_mod[idx]->dcam_pdaf;

	sprd_dcam_glb_reg_owr(idx, DCAM_AMBA_STS, BIT_2, DCAM_AHBM_STS_REG);
	udelay(1000);
	sprd_dcam_glb_reg_mwr(idx, DCAM_CFG, BIT_1, ~BIT_1, DCAM_CFG_REG);

	/* wait for AHB path busy cleared */
	while (pdaf->valid && time_out) {
		ret = DCAM_REG_RD(idx, DCAM_AMBA_STS) & BIT_4;
		if (!ret)
			break;
		time_out--;
	}

}

static void dcam_quickstop_isptop(enum dcam_id idx)
{
	sprd_dcam_glb_reg_mwr(idx, DCAM_CFG, BIT_7, ~BIT_7, DCAM_CFG_REG);
}

static void dcam_quickstop(enum dcam_id idx)
{
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return;
	}

	DCAM_TRACE("DCAM%d: state before stop 0x%x\n", idx,
		   s_p_dcam_mod[idx]->state);

	dcam_quickstop_path0(idx);
	dcam_quickstop_pdaf(idx);
	dcam_quickstop_isptop(idx);

	DCAM_TRACE("DCAM%d: exit, state: 0x%x\n", idx,
		   s_p_dcam_mod[idx]->state);
}

int sprd_dcam_stop(enum dcam_id idx, int is_irq)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	unsigned long flag;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		dcam_err_count++;
		if (dcam_err_count > 10) {
			goto rst;
		}
		return -EFAULT;
	}

	s_p_dcam_mod[idx]->state |= DCAM_STATE_QUICKQUIT;

	if (!is_irq)
		spin_lock_irqsave(&dcam_lock[idx], flag);

	dcam_quickstop(idx);

	s_p_dcam_mod[idx]->dcam_path0.status = DCAM_ST_STOP;
	s_p_dcam_mod[idx]->dcam_path0.valid = 0;
	s_p_dcam_mod[idx]->dcam_path0.sof_cnt = 0;
	s_p_dcam_mod[idx]->dcam_path0.done_cnt = 0;

	s_p_dcam_mod[idx]->dcam_pdaf.status = DCAM_ST_STOP;
	s_p_dcam_mod[idx]->dcam_pdaf.valid = 0;

	dcam_frm_clear(idx);

	if (!is_irq)
		spin_unlock_irqrestore(&dcam_lock[idx], flag);
rst:
	sprd_dcam_reset(idx);

	pr_info("dcam stop\n");
	return -rtn;
}

static void dcam_internal_deinit(enum dcam_id idx)
{
	unsigned long flag;

	spin_lock_irqsave(&dcam_mod_lock[idx], flag);
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_info("DCAM%d: invalid addr, %p", idx, s_p_dcam_mod[idx]);
	} else {
		vfree(s_p_dcam_mod[idx]);
		s_p_dcam_mod[idx] = NULL;
	}
	spin_unlock_irqrestore(&dcam_mod_lock[idx], flag);
}

void sprd_dcam_glb_reg_awr(enum dcam_id idx, unsigned long addr,
			   unsigned int val, unsigned int reg_id)
{
	unsigned long flag;

	switch (reg_id) {
	case DCAM_CFG_REG:
		spin_lock_irqsave(&dcam_glb_reg_cfg_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_cfg_lock[idx], flag);
		break;
	case DCAM_CONTROL_REG:
		spin_lock_irqsave(&dcam_glb_reg_control_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_control_lock[idx], flag);
		break;
	case DCAM_INIT_MASK_REG:
		spin_lock_irqsave(&dcam_glb_reg_mask_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_mask_lock[idx], flag);
		break;
	case DCAM_INIT_CLR_REG:
		spin_lock_irqsave(&dcam_glb_reg_clr_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_clr_lock[idx], flag);
		break;
	case DCAM_AHBM_STS_REG:
		spin_lock_irqsave(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		break;
	case DCAM_ENDIAN_REG:
		spin_lock_irqsave(&dcam_glb_reg_endian_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_endian_lock[idx], flag);
		break;
	default:
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		break;
	}
}

void sprd_dcam_glb_reg_owr(enum dcam_id idx, unsigned long addr,
			   unsigned int val, unsigned int reg_id)
{
	unsigned long flag;

	switch (reg_id) {
	case DCAM_CFG_REG:
		spin_lock_irqsave(&dcam_glb_reg_cfg_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_cfg_lock[idx], flag);
		break;
	case DCAM_CONTROL_REG:
		spin_lock_irqsave(&dcam_glb_reg_control_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_control_lock[idx], flag);
		break;
	case DCAM_INIT_MASK_REG:
		spin_lock_irqsave(&dcam_glb_reg_mask_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_mask_lock[idx], flag);
		break;
	case DCAM_INIT_CLR_REG:
		spin_lock_irqsave(&dcam_glb_reg_clr_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_clr_lock[idx], flag);
		break;
	case DCAM_AHBM_STS_REG:
		spin_lock_irqsave(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		break;
	case DCAM_ENDIAN_REG:
		spin_lock_irqsave(&dcam_glb_reg_endian_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_endian_lock[idx], flag);
		break;
	default:
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		break;
	}
}

void sprd_dcam_glb_reg_mwr(enum dcam_id idx, unsigned long addr,
			   unsigned int mask, unsigned int val,
			   unsigned int reg_id)
{
	unsigned long flag;
	unsigned int tmp = 0;

	switch (reg_id) {
	case DCAM_CFG_REG:
		spin_lock_irqsave(&dcam_glb_reg_cfg_lock[idx], flag);
		{
			tmp = DCAM_REG_RD(idx, addr);
			tmp &= ~(mask);
			DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		}
		spin_unlock_irqrestore(&dcam_glb_reg_cfg_lock[idx], flag);
		break;
	case DCAM_CONTROL_REG:
		spin_lock_irqsave(&dcam_glb_reg_control_lock[idx], flag);
		{
			tmp = DCAM_REG_RD(idx, addr);
			tmp &= ~(mask);
			DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		}
		spin_unlock_irqrestore(&dcam_glb_reg_control_lock[idx], flag);
		break;
	case DCAM_INIT_MASK_REG:
		spin_lock_irqsave(&dcam_glb_reg_mask_lock[idx], flag);
		{
			tmp = DCAM_REG_RD(idx, addr);
			tmp &= ~(mask);
			DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		}
		spin_unlock_irqrestore(&dcam_glb_reg_mask_lock[idx], flag);
		break;
	case DCAM_INIT_CLR_REG:
		spin_lock_irqsave(&dcam_glb_reg_clr_lock[idx], flag);
		{
			tmp = DCAM_REG_RD(idx, addr);
			tmp &= ~(mask);
			DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		}
		spin_unlock_irqrestore(&dcam_glb_reg_clr_lock[idx], flag);
		break;
	case DCAM_AHBM_STS_REG:
		spin_lock_irqsave(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		{
			tmp = DCAM_REG_RD(idx, addr);
			tmp &= ~(mask);
			DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		}
		spin_unlock_irqrestore(&dcam_glb_reg_ahbm_sts_lock[idx], flag);
		break;
	case DCAM_ENDIAN_REG:
		spin_lock_irqsave(&dcam_glb_reg_endian_lock[idx], flag);
		{
			tmp = DCAM_REG_RD(idx, addr);
			tmp &= ~(mask);
			DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		}
		spin_unlock_irqrestore(&dcam_glb_reg_endian_lock[idx], flag);
		break;
	default:
		{
			tmp = DCAM_REG_RD(idx, addr);
			tmp &= ~(mask);
			DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		}
		break;
	}
}

int sprd_camera_get_path_id(enum dcam_id idx,
			struct camera_get_path_id *path_id,
			unsigned int *channel_id,
			unsigned int scene_mode)
{
	int ret = DCAM_RTN_SUCCESS;
	unsigned int isp_path1_line_max = 0;
	unsigned int isp_path2_line_max = 0;
	unsigned int isp_path3_line_max = 0;


	if (NULL == path_id || NULL == channel_id)
		return -1;

	pr_info("DCAM: fourcc 0x%x input %d %d output %d %d\n",
			path_id->fourcc,
			path_id->input_size.w, path_id->input_size.h,
			path_id->output_size.w, path_id->output_size.h);
	pr_info("DCAM: input_trim %d %d %d %d need_isp %d rt_refocus %d\n",
			path_id->input_trim.x, path_id->input_trim.y,
			path_id->input_trim.w, path_id->input_trim.h,
			path_id->need_isp, path_id->rt_refocus);
	pr_info("DCAM: is_path_work %d %d %d %d\n",
			path_id->is_path_work[CAMERA_PATH0],
			path_id->is_path_work[CAMERA_PRE_PATH],
			path_id->is_path_work[CAMERA_VID_PATH],
			path_id->is_path_work[CAMERA_CAP_PATH]);
	pr_info("DCAM: scene_mode %d, need_isp_tool %d\n", scene_mode,
			path_id->need_isp_tool);
	if (idx == DCAM_ID_0) {
		isp_path1_line_max = ISP_PATH1_LINE_BUF_LENGTH;
		isp_path2_line_max = ISP_PATH2_LINE_BUF_LENGTH;
		isp_path3_line_max = ISP_PATH3_LINE_BUF_LENGTH;
	} else if (idx == DCAM_ID_1) {
		isp_path1_line_max = ISP1_PATH1_LINE_BUF_LENGTH;
		isp_path2_line_max = ISP1_PATH2_LINE_BUF_LENGTH;
		isp_path3_line_max = ISP1_PATH3_LINE_BUF_LENGTH;

	}

	if (path_id->need_isp_tool) {
		*channel_id = CAMERA_PATH0;
	} else if (path_id->fourcc == V4L2_PIX_FMT_GREY &&
			!path_id->is_path_work[CAMERA_PATH0]) {
		*channel_id = CAMERA_PATH0;
	} else if (path_id->fourcc == V4L2_PIX_FMT_JPEG &&
			!path_id->is_path_work[CAMERA_PATH0]) {
		*channel_id = CAMERA_PATH0;
	} else if (path_id->rt_refocus &&
			!path_id->is_path_work[CAMERA_PATH0]) {
		*channel_id = CAMERA_PATH0;
	} else if (scene_mode == DCAM_SCENE_MODE_PREVIEW) {
		*channel_id = CAMERA_PRE_PATH;
	} else if (scene_mode == DCAM_SCENE_MODE_RECORDING) {
		*channel_id = CAMERA_VID_PATH;
	} else if (scene_mode == DCAM_SCENE_MODE_CAPTURE) {
		*channel_id = CAMERA_CAP_PATH;
	} else if (scene_mode == DCAM_SCENE_MODE_CAPTURE_CALLBACK) {
		if ((path_id->output_size.w <= isp_path2_line_max
				&& !path_id->is_path_work[CAMERA_VID_PATH])
				|| (idx == DCAM_ID_1))
			*channel_id = CAMERA_VID_PATH;
		else
			*channel_id = CAMERA_CAP_PATH;
	} else {
		*channel_id = CAMERA_PATH0;
		pr_info("path select error\n");
	}

	pr_info("path id %d\n", *channel_id);

	return ret;
}

static int dcam_enable_clk(enum dcam_id idx)
{
	int ret = 0;

	pr_info("[%s] enter, idx=%d\n", __func__, idx);

	/*set dcam_if clock to max value*/
	ret = clk_set_parent(dcam_clk[idx], dcam_clk_parent[idx]);
	if (ret) {
		pr_err("dcam_clk_parent error.\n");
		clk_set_parent(dcam_clk[idx], dcam_clk_default[idx]);
		clk_disable_unprepare(dcam_eb[idx]);
		goto exit;
	}

	ret = clk_prepare_enable(dcam_clk[idx]);
	if (ret) {
		pr_err("dcam_clk error.\n");
		clk_set_parent(dcam_clk[idx], dcam_clk_default[idx]);
		clk_disable_unprepare(dcam_eb[idx]);
		goto exit;
	}


	/*dcam enable*/
	ret = clk_prepare_enable(dcam_mg_eb[idx]);
	if (ret) {
		pr_info("dcam module gate enable error.\n");
		goto exit;
	}

	ret = clk_prepare_enable(dcam_d2i_mg_eb[idx]);
	if (ret) {
		pr_info("clk dcam_if enable in isp main error.\n");
		goto exit;
	}

	ret = clk_prepare_enable(dcam_eb[idx]);
	if (ret) {
		pr_info("dcam_enable error.\n");
		goto exit;
	}

	pr_info("dcam_enable_clk end.\n");
exit:
	return ret;
}

static int dcam_disable_clk(enum dcam_id idx)
{
	pr_info("dcm %s enter, idx=%d\n", __func__, idx);

	/*cut off the dcam_if colck source*/
	clk_disable_unprepare(dcam_eb[idx]);

	/*set dcam_if clock to default value before power off*/
	clk_set_parent(dcam_clk[idx], dcam_clk_default[idx]);
	clk_disable_unprepare(dcam_clk[idx]);

	clk_disable_unprepare(dcam_mg_eb[idx]);
	clk_disable_unprepare(dcam_d2i_mg_eb[idx]);

	pr_info("dcam_disable_clk end.\n");

	return 0;
}

int set_dcam_cap_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_cap_desc *cap_desc = NULL;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}
	cap_desc = &s_p_dcam_mod[idx]->dcam_cap;

	switch (id) {
	case DCAM_CAP_INTERFACE:
		{
			enum dcam_cap_if_mode cap_if_mode =
				*(unsigned int *)param;

			if (cap_if_mode == DCAM_CAP_IF_CSI2)
				cap_desc->cam_if = cap_if_mode;
			else
				rtn = DCAM_RTN_CAP_IF_MODE_ERR;
			break;
		}

	case DCAM_CAP_SENSOR_MODE:
		{
			enum dcam_cap_sensor_mode cap_sensor_mode =
				*(unsigned int *)param;

			if (cap_sensor_mode >= DCAM_CAP_MODE_MAX) {
				rtn = DCAM_RTN_CAP_SENSOR_MODE_ERR;
			} else {
				DCAM_REG_MWR(idx, CAP_MIPI_CFG, BIT_1,
					cap_sensor_mode << 1);
				cap_desc->input_format = cap_sensor_mode;
			}
			break;
		}

	case DCAM_CAP_SYNC_POL:
		{
			struct dcam_cap_sync_pol *sync_pol =
				(struct dcam_cap_sync_pol *)param;

			if (sync_pol->need_href)
				DCAM_REG_MWR(idx, CAP_MIPI_CFG, BIT_5, 1 << 5);
			else
				DCAM_REG_MWR(idx, CAP_MIPI_CFG, BIT_5, 0 << 5);
			break;
		}

	case DCAM_CAP_DATA_BITS:
		{
			enum dcam_cap_data_bits bits = *(unsigned int *)param;

			if (bits == DCAM_CAP_12_BITS)
				DCAM_REG_MWR(idx, CAP_MIPI_CFG,
				BIT_4 | BIT_3, 2 << 3);
			else if (bits == DCAM_CAP_10_BITS)
				DCAM_REG_MWR(idx, CAP_MIPI_CFG,
				BIT_4 | BIT_3, 1 << 3);
			else if (bits == DCAM_CAP_8_BITS)
				DCAM_REG_MWR(idx, CAP_MIPI_CFG,
				BIT_4 | BIT_3, 0 << 3);
			else
				rtn = DCAM_RTN_CAP_IN_BITS_ERR;
			break;
		}

	case DCAM_CAP_PRE_SKIP_CNT:
		{
			unsigned int skip_num = *(unsigned int *)param;

			if (skip_num > DCAM_CAP_SKIP_FRM_MAX)
				rtn = DCAM_RTN_CAP_SKIP_FRAME_ERR;
			else
				DCAM_REG_MWR(idx, CAP_MIPI_FRM_CTRL,
					BIT_3 | BIT_2 | BIT_1 | BIT_0,
					skip_num);
			break;
		}

	case DCAM_CAP_FRM_DECI:
		{
			unsigned int deci_factor = *(unsigned int *)param;

			if (deci_factor < DCAM_FRM_DECI_FAC_MAX)
				DCAM_REG_MWR(idx, CAP_MIPI_FRM_CTRL,
				BIT_5 | BIT_4,
					deci_factor << 4);
			else
				rtn = DCAM_RTN_CAP_FRAME_DECI_ERR;
			break;
		}

	case DCAM_CAP_FRM_COUNT_CLR:
		DCAM_REG_MWR(idx, CAP_MIPI_FRM_CTRL, BIT_22, 1 << 22);
		break;

	case DCAM_CAP_INPUT_RECT:
		{
			struct camera_rect *rect = (struct camera_rect *)param;
			unsigned int tmp = 0;

			if (rect->x > DCAM_CAP_FRAME_WIDTH_MAX ||
			    rect->y > DCAM_CAP_FRAME_HEIGHT_MAX ||
			    rect->w > DCAM_CAP_FRAME_WIDTH_MAX ||
			    rect->h > DCAM_CAP_FRAME_HEIGHT_MAX) {
				rtn = DCAM_RTN_CAP_FRAME_SIZE_ERR;
				return -rtn;
			}
			cap_desc->cap_rect = *rect;
			tmp = rect->x | (rect->y << 16);
			DCAM_REG_WR(idx, CAP_MIPI_START, tmp);
			tmp = (rect->x + rect->w - 1);
			tmp |= (rect->y + rect->h - 1) << 16;
			DCAM_REG_WR(idx, CAP_MIPI_END, tmp);
			break;
		}

	case DCAM_CAP_IMAGE_XY_DECI:
		{
			struct dcam_cap_dec *cap_dec =
				(struct dcam_cap_dec *)param;

			if (cap_dec->x_factor > DCAM_CAP_X_DECI_FAC_MAX ||
			    cap_dec->y_factor > DCAM_CAP_Y_DECI_FAC_MAX) {
				rtn = DCAM_RTN_CAP_XY_DECI_ERR;
			} else {
				if (DCAM_CAP_MODE_RAWRGB ==
				    cap_desc->input_format) {
					if (cap_dec->x_factor > 1
					    || cap_dec->y_factor > 1)
						rtn = DCAM_RTN_CAP_XY_DECI_ERR;
				}
				if (DCAM_CAP_MODE_RAWRGB ==
				    cap_desc->input_format) {
					DCAM_REG_MWR(idx, CAP_MIPI_IMG_DECI,
					BIT_1, cap_dec->x_factor << 1);
				} else {
					DCAM_REG_MWR(idx, CAP_MIPI_IMG_DECI,
						BIT_1 | BIT_0,
						cap_dec->x_factor);
					DCAM_REG_MWR(idx, CAP_MIPI_IMG_DECI,
						BIT_3 | BIT_2,
						cap_dec->y_factor << 2);
				}
			}
			break;
		}

	case DCAM_CAP_TO_ISP:
		{
			unsigned int need_isp = *(unsigned int *)param;

			if (need_isp) {
				sprd_dcam_glb_reg_mwr(idx,
				DCAM_CFG,
				BIT_7,
				1 << 7,
				DCAM_CFG_REG);
			} else
				sprd_dcam_glb_reg_mwr(idx, DCAM_CFG, BIT_7,
						      0 << 7,
						      DCAM_CFG_REG);
			break;
		}

	case DCAM_CAP_DATA_PACKET:
		{
			unsigned int is_loose = *(unsigned int *)param;

			if (cap_desc->cam_if == DCAM_CAP_IF_CSI2 &&
			    cap_desc->input_format == DCAM_CAP_MODE_RAWRGB) {
				if (is_loose)
					DCAM_REG_MWR(idx, CAP_MIPI_CFG,
					BIT_0, 1);
				else
					DCAM_REG_MWR(idx, CAP_MIPI_CFG,
					BIT_0, 0);
			} else
				rtn = DCAM_RTN_MODE_ERR;
			break;
		}

	case DCAM_CAP_SAMPLE_MODE:
		{
			enum dcam_capture_mode samp_mode =
				*(enum dcam_capture_mode *)param;

			if (samp_mode >= DCAM_CAPTURE_MODE_MAX) {
				rtn = DCAM_RTN_MODE_ERR;
			} else {
				DCAM_REG_MWR(idx, CAP_MIPI_CFG, BIT_6,
					samp_mode << 6);
				s_p_dcam_mod[idx]->dcam_cap.cap_mode =
					samp_mode;
			}
			break;
		}
	default:
		rtn = DCAM_RTN_IO_ID_ERR;
		break;

	}

	return -rtn;
}

int set_dcam_path0_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *path0 = NULL;
	struct camera_addr *p_addr;
	struct dcam_module *module;
	unsigned int base_id;
	struct camera_endian_sel *endian;
	struct camera_rect *rect = NULL;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	if (0 == (unsigned long)(param)) {
		pr_err("parm is null");
		return -DCAM_RTN_PARA_ERR;
	}

	module = s_p_dcam_mod[idx];
	path0 = &module->dcam_path0;

	switch (id) {
	case DCAM_PATH0_OUTPUT_ADDR:
		p_addr = (struct camera_addr *)param;

		if (DCAM_YUV_ADDR_INVALID(p_addr->yaddr, p_addr->uaddr,
					  p_addr->vaddr) &&
		    p_addr->mfd_y == 0) {
			rtn = DCAM_RTN_PATH_ADDR_ERR;
		} else {
			struct camera_frame frame;

			memset((void *)&frame, 0,
			       sizeof(struct camera_frame));
			frame.yaddr = p_addr->yaddr;
			frame.uaddr = p_addr->uaddr;
			frame.vaddr = p_addr->vaddr;
			frame.yaddr_vir = p_addr->yaddr_vir;
			frame.uaddr_vir = p_addr->uaddr_vir;
			frame.vaddr_vir = p_addr->vaddr_vir;

			frame.type = CAMERA_PATH0;
			frame.fid = path0->frame_base_id;

			if (idx == DCAM_ID_0)
				frame.pfinfo.dev = &s_dcam_pdev->dev;
			else
				frame.pfinfo.dev = &s_dcam1_pdev->dev;
			frame.pfinfo.mfd[0] = p_addr->mfd_y;
			frame.pfinfo.mfd[1] = p_addr->mfd_u;
			frame.pfinfo.mfd[2] = p_addr->mfd_u;
			/*may need update iommu here*/
			rtn = pfiommu_get_sg_table(&frame.pfinfo);
			if (rtn) {
				pr_err("cfg output addr failed!\n");
				rtn = DCAM_RTN_PATH_ADDR_ERR;
				break;
			}

			frame.pfinfo.offset[0] = 0;
			frame.pfinfo.offset[1] = 0;
			frame.pfinfo.offset[2] = 0;

			if (!dcam_buf_queue_write(&path0->buf_queue,
						  &frame))
				path0->output_frame_count++;

			DCAM_TRACE("DCAM%d: Path0 set output addr, i %d\n",
				   idx, path0->output_frame_count);
			DCAM_TRACE("y=0x%x u=0x%x v=0x%x mfd=0x%x 0x%x\n",
				   p_addr->yaddr, p_addr->uaddr,
				   p_addr->vaddr, frame.pfinfo.mfd[0],
				   frame.pfinfo.mfd[1]);
		}
		break;

	case DCAM_PATH_OUTPUT_RESERVED_ADDR:
		p_addr = (struct camera_addr *)param;

		if (DCAM_YUV_ADDR_INVALID(p_addr->yaddr,
					  p_addr->uaddr,
					  p_addr->vaddr) &&
		    p_addr->mfd_y == 0) {
			rtn = DCAM_RTN_PATH_ADDR_ERR;
		} else {
			unsigned int output_frame_count = 0;
			struct camera_frame *frame = NULL;

			frame = &module->path0_reserved_frame;
			output_frame_count = path0->output_frame_count;
			frame->yaddr = p_addr->yaddr;
			frame->uaddr = p_addr->uaddr;
			frame->vaddr = p_addr->vaddr;
			frame->yaddr_vir = p_addr->yaddr_vir;
			frame->uaddr_vir = p_addr->uaddr_vir;
			frame->vaddr_vir = p_addr->vaddr_vir;

			if (idx == DCAM_ID_0)
				frame->pfinfo.dev = &s_dcam_pdev->dev;
			else
				frame->pfinfo.dev = &s_dcam1_pdev->dev;
			frame->pfinfo.mfd[0] = p_addr->mfd_y;
			frame->pfinfo.mfd[1] = p_addr->mfd_u;
			frame->pfinfo.mfd[2] = p_addr->mfd_u;
			/*may need update iommu here*/
			rtn = pfiommu_get_sg_table(&frame->pfinfo);
			if (rtn) {
				pr_err("cfg reserved output addr failed!\n");
				rtn = DCAM_RTN_PATH_ADDR_ERR;
				break;
			}

			frame->pfinfo.offset[0] = 0;
			frame->pfinfo.offset[1] = 0;
			frame->pfinfo.offset[2] = 0;
		}
		break;

	case DCAM_PATH_FRAME_BASE_ID:
		base_id = *(unsigned int *)param;

		DCAM_TRACE("DCAM%d: set frame base id 0x%x\n",
			   idx, base_id);
		path0->frame_base_id = base_id;
		break;

	case DCAM_PATH_DATA_ENDIAN:
		endian = (struct camera_endian_sel *)param;

		if (endian->y_endian >= DCAM_ENDIAN_MAX) {
			rtn = DCAM_RTN_PATH_ENDIAN_ERR;
		} else {
			path0->data_endian.y_endian = endian->y_endian;
			path0->valid_param.data_endian = 1;
		}
		break;

	case DCAM_PATH_INPUT_RECT:
		rect = (struct camera_rect *)param;
		memcpy((void *)&path0->input_rect, (void *)rect,
			sizeof(struct camera_rect));
		path0->valid_param.input_rect = 0;
		break;

	case DCAM_PATH_FRM_DECI:
		{
			unsigned int deci_factor = *(unsigned int *) param;

			if (deci_factor >= DCAM_FRM_DECI_FAC_MAX) {
				rtn = DCAM_RTN_PATH_FRM_DECI_ERR;
			} else {
				path0->frame_deci = deci_factor;
				path0->valid_param.frame_deci = 1;
			}
			break;
		}

	case DCAM_PATH_ENABLE:
		path0->valid = *(unsigned int *)param;
		break;

	default:
		pr_err("%s error", __func__);
		break;
	}

	return -rtn;
}

int set_dcam_path_pdaf_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_pdaf *path_pdaf = NULL;
	struct camera_addr *p_addr;
	struct dcam_module *module;
	unsigned int base_id;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	if (0 == (unsigned long)(param)) {
		pr_err("parm is null");
		return -DCAM_RTN_PARA_ERR;
	}

	module = s_p_dcam_mod[idx];
	path_pdaf = &module->dcam_pdaf;

	switch (id) {
	case DCAM_PATH_PDAF_OUTPUT_ADDR:
		p_addr = (struct camera_addr *)param;

		if (DCAM_YUV_ADDR_INVALID(p_addr->yaddr, p_addr->uaddr,
					  p_addr->vaddr) &&
		    p_addr->mfd_y == 0) {
			rtn = DCAM_RTN_PATH_ADDR_ERR;
		} else {
			struct camera_frame frame;

			memset((void *)&frame, 0,
			       sizeof(struct camera_frame));
			frame.yaddr = p_addr->yaddr;
			frame.uaddr = p_addr->uaddr;
			frame.vaddr = p_addr->vaddr;
			frame.yaddr_vir = p_addr->yaddr_vir;
			frame.uaddr_vir = p_addr->uaddr_vir;
			frame.vaddr_vir = p_addr->vaddr_vir;

			frame.type = CAMERA_PATH0;
			frame.fid = path_pdaf->frame_base_id;

			if (idx == DCAM_ID_0)
				frame.pfinfo.dev = &s_dcam_pdev->dev;
			else
				frame.pfinfo.dev = &s_dcam1_pdev->dev;
			frame.pfinfo.mfd[0] = p_addr->mfd_y;
			frame.pfinfo.mfd[1] = p_addr->mfd_u;
			frame.pfinfo.mfd[2] = p_addr->mfd_u;
			/*may need update iommu here*/

			/*need add path0 size info here*/

			frame.pfinfo.offset[0] = frame.yaddr;
			frame.pfinfo.offset[1] = frame.uaddr;
			frame.pfinfo.offset[2] = frame.vaddr;

			if (!dcam_buf_queue_write(&path_pdaf->buf_queue,
						  &frame))
				path_pdaf->output_frame_count++;

			DCAM_TRACE("DCAM%d: Path0 set output addr, i %d\n",
				   idx, path_pdaf->output_frame_count);
			DCAM_TRACE("y=0x%x u=0x%x v=0x%x mfd=0x%x 0x%x\n",
				   p_addr->yaddr, p_addr->uaddr,
				   p_addr->vaddr, frame.pfinfo.mfd[0],
				   frame.pfinfo.mfd[1]);
		}
		break;

	case DCAM_PATH_OUTPUT_RESERVED_ADDR:
		p_addr = (struct camera_addr *)param;

		if (DCAM_YUV_ADDR_INVALID(p_addr->yaddr,
					  p_addr->uaddr,
					  p_addr->vaddr) &&
		    p_addr->mfd_y == 0) {
			rtn = DCAM_RTN_PATH_ADDR_ERR;
		} else {
			unsigned int output_frame_count = 0;
			struct camera_frame *frame = NULL;

			frame = &module->path0_reserved_frame;
			output_frame_count = path_pdaf->output_frame_count;
			frame->yaddr = p_addr->yaddr;
			frame->uaddr = p_addr->uaddr;
			frame->vaddr = p_addr->vaddr;
			frame->yaddr_vir = p_addr->yaddr_vir;
			frame->uaddr_vir = p_addr->uaddr_vir;
			frame->vaddr_vir = p_addr->vaddr_vir;

			if (idx == DCAM_ID_0)
				frame->pfinfo.dev = &s_dcam_pdev->dev;
			else
				frame->pfinfo.dev = &s_dcam1_pdev->dev;
			frame->pfinfo.mfd[0] = p_addr->mfd_y;
			frame->pfinfo.mfd[1] = p_addr->mfd_u;
			frame->pfinfo.mfd[2] = p_addr->mfd_u;
			/*may need update iommu here*/

			/*need add path0 size info here*/

			frame->pfinfo.offset[0] = frame->yaddr;
			frame->pfinfo.offset[1] = frame->uaddr;
			frame->pfinfo.offset[2] = frame->vaddr;
		}
		break;

	case DCAM_PATH_FRAME_BASE_ID:
		base_id = *(unsigned int *)param;

		DCAM_TRACE("DCAM%d: set frame base id 0x%x\n",
			   idx, base_id);
		path_pdaf->frame_base_id = base_id;
		break;

	case DCAM_PDAF_CONTROL:
		memcpy(&path_pdaf->pdaf_ctrl, param,
		       sizeof(struct sprd_pdaf_control));
		break;

	case DCAM_PATH_ENABLE:
		path_pdaf->valid = *(unsigned int *)param;
		break;

	default:
		pr_err("%s error", __func__);
		break;
	}

	return -rtn;
}

int sprd_dcam_start(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *path0 = NULL;
	struct dcam_path_pdaf *path_pdaf = NULL;
	unsigned int cap_en = 0;
	unsigned int reg_val = 0;
	unsigned long addr = 0;
	struct camera_rect *rect = NULL;

	dcam_err_count = 0;
	memset(g_dcam_err_irq_cnt, 0, sizeof(g_dcam_err_irq_cnt));
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	DCAM_TRACE("DCAM%d: dcam start %x\n", idx,
		   s_p_dcam_mod[idx]->dcam_cap.cap_mode);
	path0 = &s_p_dcam_mod[idx]->dcam_path0;
	path_pdaf = &s_p_dcam_mod[idx]->dcam_pdaf;

	if (path0->valid) {
		if (path0->valid_param.frame_deci) {
			sprd_dcam_glb_reg_mwr(idx, DCAM_PATH0_CFG,
					      BIT_1 | BIT_0,
					      path0->frame_deci << 0,
					      DCAM_REG_MAX);
		}

		if (path0->valid_param.data_endian) {
			sprd_dcam_glb_reg_mwr(idx, DCAM_ENDIAN_SEL,
					      BIT_5 | BIT_4,
					      path0->data_endian.y_endian << 4,
					      DCAM_ENDIAN_REG);
			sprd_dcam_glb_reg_mwr(idx, DCAM_ENDIAN_SEL,
					      BIT_18, BIT_18,
					      DCAM_ENDIAN_REG);
			DCAM_TRACE("path 0: data_endian y=0x%x\n",
				   path0->data_endian.y_endian);
		}

		if (path0->valid_param.input_rect) {
			DCAM_REG_MWR(idx, CAP_MIPI_CFG, BIT_7, BIT_7);
			pr_info("%s, set path0 crop_eb\n", __func__);

			addr = PACK_CROP_START;
			rect = &path0->input_rect;
			reg_val = (rect->x & 0xFFFF) |
				((rect->y & 0xFFFF) << 16);
			DCAM_REG_WR(idx, addr, reg_val);

			addr = PACK_CROP_END;
			reg_val = ((rect->x + rect->w - 1) & 0xFFFF) |
				(((rect->y + rect->h - 1) & 0xFFFF) << 16);
			DCAM_REG_WR(idx, addr, reg_val);
		}

		rtn = dcam_path0_set_next_frm(idx);
		if (rtn) {
			pr_err("%s err, code %d", __func__, rtn);
			return -(rtn);
		}

		path0->need_wait = 0;
		path0->status = DCAM_ST_START;
		path0->sof_cnt = 0;
		path0->done_cnt = 0;
		sprd_dcam_glb_reg_owr(idx, DCAM_CFG, BIT_0, DCAM_CFG_REG);
	}

	if (path_pdaf->valid) {
		sprd_dcam_glb_reg_mwr(idx, DCAM_PDAF_CTRL,
					      BIT_1 | BIT_0,
					      path_pdaf->pdaf_ctrl.mode << 0,
					      DCAM_REG_MAX);
		sprd_dcam_glb_reg_mwr(idx, DCAM_PDAF_CTRL,
				      BIT_13 | BIT_12 | BIT_11 |
				      BIT_10 | BIT_9 | BIT_8,
				      path_pdaf->pdaf_ctrl.phase_data_type << 8,
				      DCAM_REG_MAX);

		rtn = dcam_pdaf_set_next_frm(idx);
		if (rtn) {
			pr_err("%s err, code %d", __func__, rtn);
			return -(rtn);
		}

		sprd_dcam_glb_reg_owr(idx, DCAM_CFG, BIT_1, DCAM_CFG_REG);
	}

	cap_en = DCAM_REG_RD(idx, DCAM_CONTROL) & BIT_2;
	DCAM_TRACE("DCAM%d: cap_eb %d\n", idx, cap_en);

	if (cap_en == 0) {
		/* Cap force copy */
		dcam_force_copy(idx);
		/* Cap Enable */
		sprd_dcam_glb_reg_mwr(idx, DCAM_CONTROL, BIT_2, 1 << 2,
				      DCAM_CONTROL_REG);
	}

	return -rtn;
}

int sprd_dcam_module_en(enum dcam_id idx)
{
	int ret = 0;

	pr_info("DCAM%d: enable dcam module, in %d\n", idx,
		atomic_read(&s_dcam_users[idx]));

	mutex_lock(&dcam_module_sema[idx]);
	if (atomic_inc_return(&s_dcam_users[idx]) == 1) {

		wake_lock(&dcam_wakelock[idx]);

		ret = sprd_cam_pw_on();
		if (ret != 0) {
			pr_err("%s : sprd cam_sys power on failed\n", __func__);
			mutex_unlock(&dcam_module_sema[idx]);
			return ret;
		}

		sprd_cam_domain_eb();

		ret = dcam_enable_clk(idx);
		if (ret) {
			pr_err("failed to enable dcam clk %d\n", ret);
			mutex_unlock(&dcam_module_sema[idx]);
			return ret;
		}
		ret = sprd_dcam_reset(idx);
		if (ret) {
			pr_err("failed to dcam reset %d\n", ret);
			mutex_unlock(&dcam_module_sema[idx]);
			return ret;
		}

		ret = request_irq(s_dcam_irq[idx], dcam_isr_root,
				  IRQF_SHARED, "DCAM",
				  (void *)&s_dcam_irq[idx]);
		if (ret) {
			pr_err("failed to install IRQ %d\n", ret);
			mutex_unlock(&dcam_module_sema[idx]);
			return ret;
		}

		memset((void *)&s_user_func[idx][0], 0,
		       sizeof(void *) * DCAM_IRQ_NUMBER);
		memset((void *)&s_user_data[idx][0], 0,
		       sizeof(void *) * DCAM_IRQ_NUMBER);
		pr_info("DCAM%d: register isr, 0x%x\n", idx,
			DCAM_REG_RD(idx, DCAM_INT_MASK));
	}
	mutex_unlock(&dcam_module_sema[idx]);

	pr_info("DCAM%d: enable dcam module, end %d\n", idx,
		atomic_read(&s_dcam_users[idx]));

	return ret;
}

int sprd_dcam_module_init(enum dcam_id idx)
{
	int ret = 0;

	if (atomic_read(&s_dcam_users[idx]) < 1) {
		pr_err("s_dcam_users[%d] equal to %d", idx,
		       atomic_read(&s_dcam_users[idx]));
		return -EIO;
	}
	ret = dcam_internal_init(idx);

	return ret;
}

int sprd_dcam_module_deinit(enum dcam_id idx)
{
	dcam_internal_deinit(idx);

	if (s_p_dcam_mod[DCAM_ID_0] == NULL
		&& s_p_dcam_mod[DCAM_ID_1] == NULL)
		pfiommu_put_sg_table();

	return -DCAM_RTN_SUCCESS;
}

int sprd_dcam_drv_init(struct platform_device *pdev)
{
	int i, j;

	s_dcam_pdev = pdev;

	for (i = 0; i < s_dcam_count; i++) {
		atomic_set(&s_dcam_users[i], 0);
		s_p_dcam_mod[i] = NULL;

		mutex_init(&dcam_module_sema[i]);

		dcam_mod_lock[i] = __SPIN_LOCK_UNLOCKED(dcam_mod_lock);
		dcam_lock[i] = __SPIN_LOCK_UNLOCKED(dcam_lock);
		dcam_glb_reg_cfg_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_cfg_lock);
		dcam_glb_reg_control_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_control_lock);
		dcam_glb_reg_mask_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_mask_lock);
		dcam_glb_reg_clr_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_clr_lock);
		dcam_glb_reg_ahbm_sts_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_ahbm_sts_lock);
		dcam_glb_reg_endian_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_endian_lock);
		if (i > 0) {
			for (j = 0; j < DCAM_IRQ_NUMBER; j++)
				isr_list[i][j] = isr_list[0][j];
		}

		sprintf(dcam_wakelock_name[i], "sprd_cam_wakelock_dcam%d", i);
		wake_lock_init(&dcam_wakelock[i], WAKE_LOCK_SUSPEND,
			   dcam_wakelock_name[i]);
	}

	return 0;
}

void sprd_dcam_drv_deinit(void)
{
	int i;

	for (i = 0; i < s_dcam_count; i++) {
		atomic_set(&s_dcam_users[i], 0);
		s_p_dcam_mod[i] = NULL;
		s_dcam_irq[i] = 0;

		mutex_init(&dcam_module_sema[i]);

		dcam_mod_lock[i] = __SPIN_LOCK_UNLOCKED(dcam_mod_lock);
		dcam_lock[i] = __SPIN_LOCK_UNLOCKED(dcam_lock);
		dcam_glb_reg_cfg_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_cfg_lock);
		dcam_glb_reg_control_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_control_lock);
		dcam_glb_reg_mask_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_mask_lock);
		dcam_glb_reg_clr_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_clr_lock);
		dcam_glb_reg_ahbm_sts_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_ahbm_sts_lock);
		dcam_glb_reg_endian_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_endian_lock);

		wake_lock_destroy(&dcam_wakelock[i]);
	}
}

int sprd_dcam_module_dis(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;

	pr_info("DCAM%d: disable dcam module, in %d\n", idx,
		   atomic_read(&s_dcam_users[idx]));

	if (atomic_read(&s_dcam_users[idx]) == 0)
		return rtn;

	mutex_lock(&dcam_module_sema[idx]);
	if (atomic_dec_return(&s_dcam_users[idx]) == 0) {
		dcam_disable_clk(idx);

		sprd_cam_domain_disable();

		rtn = sprd_cam_pw_off();
		if (rtn != 0) {
			pr_err("%s : sprd cam_sys power off failed\n",
					__func__);
			free_irq(s_dcam_irq[idx], (void *)&s_dcam_irq[idx]);
			wake_unlock(&dcam_wakelock[idx]);
			mutex_unlock(&dcam_module_sema[idx]);
			return rtn;
		}

		free_irq(s_dcam_irq[idx], (void *)&s_dcam_irq[idx]);

		wake_unlock(&dcam_wakelock[idx]);

	}

	mutex_unlock(&dcam_module_sema[idx]);

	return rtn;
}

int sprd_dcam_get_path_capability(struct dcam_path_capability *capacity)
{
	if (capacity == NULL)
		return -1;

	capacity->count = 4;
	capacity->support_3dnr_mode = SPRD_3DNR_SW;
	capacity->path_info[CAMERA_PATH0].line_buf = 0;
	capacity->path_info[CAMERA_PATH0].support_yuv = 0;
	capacity->path_info[CAMERA_PATH0].support_raw = 1;
	capacity->path_info[CAMERA_PATH0].support_jpeg = 1;
	capacity->path_info[CAMERA_PATH0].support_scaling = 0;
	capacity->path_info[CAMERA_PATH0].support_trim = 0;
	capacity->path_info[CAMERA_PATH0].is_scaleing_path = 0;

	capacity->path_info[CAMERA_PRE_PATH].line_buf = 0;
	capacity->path_info[CAMERA_PRE_PATH].support_yuv = 1;
	capacity->path_info[CAMERA_PRE_PATH].support_raw = 0;
	capacity->path_info[CAMERA_PRE_PATH].support_jpeg = 0;
	capacity->path_info[CAMERA_PRE_PATH].support_scaling = 1;
	capacity->path_info[CAMERA_PRE_PATH].support_trim = 1;
	capacity->path_info[CAMERA_PRE_PATH].is_scaleing_path = 0;

	capacity->path_info[CAMERA_VID_PATH].line_buf = 0;
	capacity->path_info[CAMERA_VID_PATH].support_yuv = 1;
	capacity->path_info[CAMERA_VID_PATH].support_raw = 0;
	capacity->path_info[CAMERA_VID_PATH].support_jpeg = 0;
	capacity->path_info[CAMERA_VID_PATH].support_scaling = 1;
	capacity->path_info[CAMERA_VID_PATH].support_trim = 1;
	capacity->path_info[CAMERA_VID_PATH].is_scaleing_path = 0;

	capacity->path_info[CAMERA_CAP_PATH].line_buf = 0;
	capacity->path_info[CAMERA_CAP_PATH].support_yuv = 1;
	capacity->path_info[CAMERA_CAP_PATH].support_raw = 0;
	capacity->path_info[CAMERA_CAP_PATH].support_jpeg = 0;
	capacity->path_info[CAMERA_CAP_PATH].support_scaling = 1;
	capacity->path_info[CAMERA_CAP_PATH].support_trim = 1;
	capacity->path_info[CAMERA_CAP_PATH].is_scaleing_path = 0;

	return 0;
}

int sprd_dcam_update_clk(int clk_index, struct device_node *dn)
{
	int ret = 0;
	unsigned int clk_dcam_if = 0;
	char *clk_name = NULL;

	if (!dn) {
		pr_err("Input ptr is NULL!\n");
		return -EINVAL;
	}

	clk_dcam_if = dcam_if_clk_tab[clk_index].clock;
	clk_name = dcam_if_clk_tab[clk_index].clk_name;

	if (clk_name != NULL) {
		dcam_clk_parent[0] = of_clk_get_by_name(dn, clk_name);
		if (IS_ERR(dcam_clk_parent[0])) {
			pr_err("failed to get dcam0_clk_parent\n");
			return PTR_ERR(dcam_clk_parent[0]);
		}
	} else {
		pr_err("clk_name is NULL & clk_dcam_if %d\n", clk_dcam_if);
	}

	ret = clk_set_parent(dcam_clk[0], dcam_clk_parent[0]);
	if (ret) {
		pr_err("dcam0_clk_parent error.\n");
		clk_set_parent(dcam_clk[0], dcam_clk_default[0]);
		clk_disable_unprepare(dcam_eb[0]);
	}

	pr_info("dcam_clk_name is %s\n", clk_name);

	return ret;
}

int sprd_dcam_parse_dt(struct device_node *dn, unsigned int *dcam_count)
{
	int i = 0;
	unsigned int count = 0;
	void __iomem *reg_base;

	if (!dn || !dcam_count) {
		pr_err("Input ptr is NULL!\n");
		return -EINVAL;
	}

	pr_info("Start dcam dts parse\n");
	if (of_property_read_u32_index(dn, "sprd,dcam-count", 0, &count)) {
		pr_err("failed to parse the property of sprd,dcam-count\n");
		return -EINVAL;
	}

	s_dcam_count = count;
	*dcam_count = count;

	pr_info("dcam dt %s dcam_count = %d\n", __func__, count);
	dcam_mg_eb[0] = of_clk_get_by_name(dn, "dcam0_mg_eb");
	if (IS_ERR(dcam_mg_eb[0])) {
		pr_err("failed to get dcam0_mg_eb\n");
		return PTR_ERR(dcam_mg_eb[0]);
	}

	dcam_d2i_mg_eb[0] = of_clk_get_by_name(dn, "dcam0_d2ia_mg_eb");
	if (IS_ERR(dcam_d2i_mg_eb[0])) {
		pr_err("failed to get dcam0_d2ia_eb\n");
		return PTR_ERR(dcam_d2i_mg_eb[0]);
	}

	dcam_eb[0] = of_clk_get_by_name(dn, "dcam0_eb");
	if (IS_ERR(dcam_eb[0])) {
		pr_err("failed to get dcam0_eb\n");
		return PTR_ERR(dcam_eb[0]);
	}

	dcam_clk[0] = of_clk_get_by_name(dn, "dcam0_clk");
	if (IS_ERR(dcam_clk[0])) {
		pr_err("failed to get dcam0_clk\n");
		return PTR_ERR(dcam_clk[0]);
	}

	dcam_clk_parent[0] = of_clk_get_by_name(dn, "dcam0_clk_parent");
	if (IS_ERR(dcam_clk_parent[0])) {
		pr_err("failed to get dcam0_clk_parent\n");
		return PTR_ERR(dcam_clk_parent[0]);
	}

	dcam_clk_default[0] = clk_get_parent(dcam_clk[0]);
	if (IS_ERR(dcam_clk_default[0])) {
		pr_err("failed to get dcam0_clk_default\n");
		return PTR_ERR(dcam_clk_default[0]);
	}

	cam_ahb_gpr = syscon_regmap_lookup_by_phandle(dn,
		"sprd,cam-ahb-syscon");
	if (IS_ERR(cam_ahb_gpr))
		return PTR_ERR(cam_ahb_gpr);

	if (count == DCAM_MAX_COUNT) {
		dcam_mg_eb[1] = of_clk_get_by_name(dn, "dcam1_mg_eb");
		if (IS_ERR(dcam_mg_eb[1])) {
			pr_err("failed to get dcam1_mg_eb\n");
			return PTR_ERR(dcam_mg_eb[1]);
		}

		dcam_d2i_mg_eb[1] = of_clk_get_by_name(dn, "dcam1_d2ib_mg_eb");
		if (IS_ERR(dcam_d2i_mg_eb[1])) {
			pr_err("failed to get dcam1_d2ib_eb\n");
			return PTR_ERR(dcam_d2i_mg_eb[1]);
		}
		dcam_eb[1] = of_clk_get_by_name(dn, "dcam1_eb");
		if (IS_ERR(dcam_eb[1])) {
			pr_err("failed to get dcam1_eb\n");
			return PTR_ERR(dcam_eb[1]);
		}

		dcam_clk[1] = of_clk_get_by_name(dn, "dcam1_clk");
		if (IS_ERR(dcam_clk[1])) {
			pr_err("failed to get dcam0_clk\n");
			return PTR_ERR(dcam_clk[1]);
		}

		dcam_clk_parent[1] = of_clk_get_by_name(dn, "dcam1_clk_parent");
		if (IS_ERR(dcam_clk_parent[1])) {
			pr_err("failed to get dcam0_clk_parent\n");
			return PTR_ERR(dcam_clk_parent[1]);
		}

		dcam_clk_default[1] = clk_get_parent(dcam_clk[1]);
		if (IS_ERR(dcam_clk_default[1])) {
			pr_err("failed to get dcam0_clk_default\n");
			return PTR_ERR(dcam_clk_default[1]);
		}

	}

	for (i = 0; i < count; i++) {
		s_dcam_irq[i] = irq_of_parse_and_map(dn, i);
		if (s_dcam_irq[i] <= 0) {
			pr_err("failed to get dcam irq %d\n", i);
			return -EFAULT;
		}

		reg_base = of_iomap(dn, i);
		if (!reg_base) {
			pr_err("failed to get dcam reg_base %d\n", i);
			return -ENXIO;
		}
		s_dcam_regbase[i] = (unsigned long)reg_base;

		pr_info("Dcam dts OK ! base %lx, irq %d\n",
			s_dcam_regbase[i], s_dcam_irq[i]);
	}

	if (count == DCAM_MAX_COUNT) {
		struct device_node *dcam1_node = NULL;

		dcam1_node = of_parse_phandle(dn, "sprd,dcam1", 0);
		if (dcam1_node == NULL) {
			pr_err("failed to parse the property of sprd,dcam1\n");
			s_dcam1_pdev = NULL;
		} else {
			pr_info("parse dcam1 device node %s, full name %s\n",
				dcam1_node->name, dcam1_node->full_name);
			s_dcam1_pdev = of_find_device_by_node(dcam1_node);
			pr_info("sprd s_dcam1_pdev name %s\n",
				s_dcam1_pdev->name);
		}
	}
	return 0;
}
