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
#include <linux/printk.h>
#include <linux/regmap.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/sprd_ion.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <linux/wakelock.h>
#include <linux/sprd_iommu.h>
#include <video/sprd_mm.h>
#include "dcam_drv.h"
#include "gen_scale_coef.h"
#include "getyuvinput.h"
#include "cam_pw_domain.h"
#include "isp_drv.h"
#include "dcam_buf.h"
#include "dcam_core.h"

/* Macro Definitions */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_DRV: %d %d %s : " \
	fmt, current->pid, __LINE__, __func__

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

#define DCAM_AXI_STOP_TIMEOUT			1000

#ifdef CONFIG_SPRD_CAMERA_HAPS_VERIFICATION
#define DCAM_PATH_TIMEOUT			msecs_to_jiffies(500*10)
#else
#define DCAM_PATH_TIMEOUT			msecs_to_jiffies(500)
#endif

#define DCAM_FRM_QUEUE_LENGTH			4

#define DCAM_STATE_QUICKQUIT			BIT(0)
#define DCAM_STATE_FIRSTQUIT			BIT(1)

#define DCAM_AXI_DBG_STS_BUSY			(BIT(0)|BIT(1))
#define DCAM_MMU_BUSY					BIT(2)

/* n should be 2,4,8,16,32.n=2^x */
#define DCAM_SIZE_ALIGN(x, a)			((x) & ~((1<<(a))-1))

#define DCAM_IRQ_ERR_MASK \
	((1 << DCAM_OVF) | \
	 (1 << DCAM_AEM_HOLD_OVF) | \
	 (1 << DCAM_CAP_LINE_ERR) | \
	 (1 << DCAM_CAP_FRM_ERR) | \
	 (1 << DCAM_MMU_INT))

#define DCAM_IRQ_LINE_MASK \
	((1 << DCAM_SN_EOF) | \
	 (1 << DCAM_CAP_SOF) | \
	 (1 << DCAM_ISP_ENABLE_PULSE) | \
	 (1 << DCAM_FULL_PATH_TX_DONE) | \
	 (1 << DCAM_BIN_PATH_TX_DONE) | \
	 (1 << DCAM_AEM_PATH_TX_DONE) | \
	 (1 << DCAM_PDAF_PATH_TX_DONE) | \
	 (1 << DCAM_VCH2_PATH_TX_DONE) | \
	 (1 << DCAM_VCH3_PATH_TX_DONE) | \
	 (DCAM_IRQ_ERR_MASK))

#define DCAM_OTHER_ID(idx)	((idx == DCAM_ID_0) ? DCAM_ID_1 : DCAM_ID_0)

typedef void (*dcam_isr) (enum dcam_id idx);

atomic_t dcam_full_path_time_flag = ATOMIC_INIT(DCAM_TIME_NULL);

enum {
	DCAM_FRM_UNLOCK = 0,
	DCAM_FRM_LOCK_WRITE = 0x10011001,
	DCAM_FRM_LOCK_READ = 0x01100110
};

enum {
	DCAM_ST_STOP = 0,
	DCAM_ST_START,
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
	struct timeval t;
	int bin_ratio; /* (w,h) >> this),0: no binning 1: 1/2 binning */
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

struct dcam_path_aem {
	unsigned int valid;
};

struct dcam_path_ebd {
	struct sprd_ebd_control ebd_ctrl;
	unsigned int valid;
};

struct dcam_module {
	struct dcam_cap_desc dcam_cap;
	struct dcam_path_desc dcam_raw_path;
	struct dcam_path_desc dcam_full_path;
	struct dcam_path_desc dcam_binning_path;
	struct dcam_path_pdaf dcam_pdaf;
	struct camera_frame raw_reserved_frame;
	struct camera_frame full_reserved_frame;
	struct camera_frame bin_reserved_frame;
	struct dcam_path_aem dcam_aem;
	struct dcam_path_ebd dcam_ebd;
	struct camera_frame pdaf_reserved_frame;
	struct camera_frame raw_frame;
	struct camera_frame pdaf_frame;
	unsigned int raw_framecnt;
	unsigned int pdaf_framecnt;

	unsigned int wait_resize_done;
	unsigned int err_happened;
	unsigned int state;
	void *dev_handle;
};

/* Static Variables Declaration */
static unsigned int s_dcam_count;
static atomic_t s_dcam_users[DCAM_MAX_COUNT];
static struct dcam_module *s_p_dcam_mod[DCAM_MAX_COUNT];
static int s_dcam_irq[DCAM_MAX_COUNT];
static dcam_isr_func s_user_func[DCAM_MAX_COUNT][DCAM_IRQ_NUMBER];
static void *s_user_data[DCAM_MAX_COUNT][DCAM_IRQ_NUMBER];
static unsigned long s_lsc_grid_base[DCAM_MAX_COUNT];
static struct mutex dcam_module_sema[DCAM_MAX_COUNT];

static unsigned long s_dcam_axi_base;
static spinlock_t dcam_mod_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_cfg_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_control_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_mask_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_clr_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_axi_sts_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_endian_lock[DCAM_MAX_COUNT];
static spinlock_t dcam_glb_reg_axi_endian_lock;
static spinlock_t dcam_glb_reg_path_stop_lock[DCAM_MAX_COUNT];

static struct clk *dcam0_clk;
static struct clk *dcam0_clk_parent;
static struct clk *dcam0_clk_default;
static struct clk *dcam0_mm_eb;
static struct clk *dcam0_eb;
/*
*static struct clk *dcam1_mm_eb;
*static struct clk *dcam1_eb;
*/
static struct regmap *cam_ahb_gpr;

/* type3, sync left and right */
static atomic_t pdaf_left, pdaf_right;

/* SoF timestamp */
static struct dcam_sof_time_queue dcam_t_sof;

struct platform_device *s_dcam_pdev;

unsigned long s_dcam_regbase[DCAM_MAX_COUNT];
#define _LOG_FRAME_TIMES 0
static struct log_frame_cnt log_frame;

const unsigned int s_irq_err_vect[] = {
	DCAM_OVF,
	DCAM_AEM_HOLD_OVF,
	DCAM_CAP_LINE_ERR,
	DCAM_CAP_FRM_ERR,
	DCAM_MMU_INT,
};

const unsigned int s_irq_vect[] = {
	DCAM_BIN_PATH_TX_DONE,
	DCAM_FULL_PATH_TX_DONE,
	DCAM_SN_SOF,
	DCAM_SN_EOF,
	DCAM_CAP_SOF,
	DCAM_CAP_EOF,
	DCAM_OVF,
	DCAM_AEM_HOLD_OVF,
	DCAM_ISP_ENABLE_PULSE,
	DCAM_IRQ_RESERVE_1,
	DCAM_IRQ_RESERVE_2,
	DCAM_IRQ_RESERVE_3,
	DCAM_CAP_LINE_ERR,
	DCAM_CAP_FRM_ERR,
	DCAM_FULL_PATH_END,
	DCAM_BIN_PATH_END,
	DCAM_AEM_PATH_END,
	DCAM_PDAF_PATH_END,
	DCAM_VCH2_PATH_END,
	DCAM_VCH3_PATH_END,
	DCAM_AEM_PATH_TX_DONE,
	DCAM_PDAF_PATH_TX_DONE,
	DCAM_VCH2_PATH_TX_DONE,
	DCAM_VCH3_PATH_TX_DONE,
	DCAM_MMU_INT,
};

static const int s_dcam_time_logger;

static inline void dcam_print_time(char *str, int type)
{
	struct timeval t;
	static struct timeval old_t;
	struct tm broken;
	long total, old_total;

	if (s_dcam_time_logger == 0)
		return;

	do_gettimeofday(&t);

	if (type == DCAM_TIME_SOF)
		old_t = t;
	else if (type == DCAM_TIME_TX_DONE) {
		total = t.tv_sec * 1000000 + t.tv_usec;
		old_total = old_t.tv_sec * 1000000 + old_t.tv_usec;
		if (total - old_total > 33000 + 1000 ||
			total - old_total < 33000 - 1000)
			pr_err("debug shaking: out of range\n");
	}

	time_to_tm(t.tv_sec, 0, &broken);

	pr_info("%s: %02d:%02d:%02d:%03ld.%03ld\n",
		str, broken.tm_hour, broken.tm_min,
		broken.tm_sec, t.tv_usec/1000, t.tv_usec%1000);
}

/* Internal Function Implementation */
static void dcam_buf_queue_init(struct dcam_buf_queue *queue)
{
	if (DCAM_ADDR_INVALID(queue)) {
		pr_err("fail to get heap %p\n", queue);
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
		pr_err("fail to get queue frm %p, %p\n", queue, frame);
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

		DCAM_TRACE("read buf queue %d index %x\n",
			frame->type, queue->r_index);
	} else {
		ret = -EAGAIN;
		DCAM_TRACE("warning, read wait new node write\n");
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

static int dcam_buf_queue_write(struct dcam_buf_queue *queue,
				struct camera_frame *frame)
{
	int ret = DCAM_RTN_SUCCESS;
	struct camera_frame *ori_frame;
	unsigned long flags;

	if (DCAM_ADDR_INVALID(queue) || DCAM_ADDR_INVALID(frame)) {
		pr_err("fail to get queue frm %p, %p\n", queue, frame);
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

static void dcam_time_queue_init(struct dcam_path_time_queue *queue)
{
	memset((void *)queue, 0, sizeof(struct dcam_path_time_queue));
	queue->write = &queue->t[0];
	queue->read = &queue->t[0];
	spin_lock_init(&queue->lock);
}

static int dcam_time_queue_read(struct dcam_path_time_queue *queue,
					struct timeval *t)
{
	int ret = DCAM_RTN_SUCCESS;
	unsigned long flags;

	DCAM_TRACE("read time queue\n");

	spin_lock_irqsave(&queue->lock, flags);
	if (queue->read != queue->write) {
		*t = *queue->read++;
		queue->r_index++;
		if (queue->read > &queue->t[DCAM_SOF_TIME_QUEUE_LENGTH - 1]) {
			queue->read = &queue->t[0];
			queue->r_index = 0;
		}

		DCAM_TRACE("read time queue index %x\n",
			queue->r_index);
	} else {
		ret = -EAGAIN;
		DCAM_TRACE("warning, read wait new node write\n");
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

static int dcam_time_queue_write(struct dcam_path_time_queue *queue,
					struct timeval *t)
{
	int ret = DCAM_RTN_SUCCESS;
	struct timeval *ori_t;
	unsigned long flags;
	int old_index;

	DCAM_TRACE("write buf queue\n");
	spin_lock_irqsave(&queue->lock, flags);

	ori_t = queue->write;
	old_index = queue->w_index;
	*queue->write++ = *t;
	queue->w_index++;
	if (queue->write > &queue->t[DCAM_SOF_TIME_QUEUE_LENGTH - 1]) {
		queue->write = &queue->t[0];
		queue->w_index = 0;
	}

	if (queue->write == queue->read) {
		queue->write = ori_t;
		queue->w_index = old_index;
		pr_warn("warning, queue is full, can't write 0x%x\n",
			queue->w_index);
		ret = -EAGAIN;
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	DCAM_TRACE("write time queue index %x\n",
		queue->w_index);

	return ret;
}

static void dcam_frm_queue_clear(struct dcam_frm_queue *queue)
{
	if (DCAM_ADDR_INVALID(queue)) {
		pr_err("fail to get heap %p\n", queue);
		return;
	}

	memset((void *)queue, 0, sizeof(struct dcam_frm_queue));
}

static int dcam_frame_enqueue(struct dcam_frm_queue *queue,
			      struct camera_frame *frame)
{
	if (DCAM_ADDR_INVALID(queue) || DCAM_ADDR_INVALID(frame)) {
		pr_err("fail to get queue frm %p, %p\n", queue, frame);
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
		pr_err("fail to get queue frm %p, %p\n", queue, frame);
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

static int dcam_raw_path_set_next_frm(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct camera_frame frame = {0};
	struct camera_frame *reserved_frame = NULL;
	struct dcam_path_desc *raw_path = NULL;
	unsigned long reg = 0;
	unsigned int addr = 0;
	unsigned int path_max_frm_cnt;
	struct dcam_frm_queue *p_heap = NULL;
	struct dcam_buf_queue *p_buf_queue = NULL;
	unsigned int output_frame_count = 0;
	int use_reserve_frame = 0;
	struct dcam_module *module;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	module = s_p_dcam_mod[idx];
	reserved_frame = &module->raw_reserved_frame;
	raw_path = &module->dcam_raw_path;
	pr_debug("raw capture mode:%d\n", raw_path->valid);

	if (raw_path->valid == FULL_RAW_CAPTURE)
		reg = DCAM0_FULL_BASE_WADDR;
	else if (raw_path->valid == BIN_RAW_CAPTURE)
		reg = DCAM0_BIN_BASE_WADDR0;
	else
		pr_err("fail to get valid raw capture mode\n");

	path_max_frm_cnt = DCAM_PATH_0_FRM_CNT_MAX;
	p_heap = &raw_path->frame_queue;
	p_buf_queue = &raw_path->buf_queue;
	output_frame_count = raw_path->output_frame_count;

	if (dcam_buf_queue_read(p_buf_queue, &frame) == 0 &&
	    (frame.pfinfo.mfd[0] != 0)) {
		raw_path->output_frame_count--;
	} else {
		pr_info("DCAM%d: No freed frame id %d raw path\n",
			idx, frame.fid);
		if (reserved_frame->pfinfo.mfd[0] == 0) {
			pr_info("DCAM%d: No need to cfg frame buffer", idx);
			return -1;
		}
		memcpy(&frame, reserved_frame, sizeof(struct camera_frame));
		use_reserve_frame = 1;
	}

	DCAM_TRACE("DCAM%d: reserved %d y 0x%x mfd 0x%x raw path\n",
		   idx, use_reserve_frame, frame.yaddr,
		   frame.pfinfo.mfd[0]);

	if (frame.pfinfo.dev == NULL)
		pr_info("DCAM%d next dev NULL %p\n", idx, frame.pfinfo.dev);
	rtn = pfiommu_get_addr(&frame.pfinfo);
	if (rtn) {
		pr_err("fail to get dcam path0 addr\n");
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

static inline unsigned int dcam_get_frm_addr(struct camera_frame *frame)
{
	pr_debug("iova %x , called from %pS",
		frame->uaddr_vir, __builtin_return_address(0));

	return frame->uaddr_vir;
}

static int dcam_full_path_set_next_frm(enum dcam_id idx,
	struct camera_frame *frame)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	unsigned int addr = 0;

	/* processed by ISP */
	if (DCAM_PHYADDR_INVALID(frame->yaddr_vir)) {
		rtn = DCAM_RTN_PATH_ADDR_ERR;
		goto exit;
	}

	addr = dcam_get_frm_addr(frame);

	DCAM_TRACE_INT("full addr 0x%x\n", addr);
	DCAM_REG_WR(idx, DCAM0_FULL_BASE_WADDR, addr);

	dcam_print_time("debug shaking: write dcam address (full)",
		DCAM_TIME_NULL);

	if (log_frame.tx_done_full) {
		pr_info("tx_done full[%d], frm 0x%x\n",
			log_frame.tx_done_full, addr);
		log_frame.tx_done_full--;
	}

exit:
	return -rtn;
}

static int dcam_bin_path_set_next_frm(enum dcam_id idx,
			struct camera_frame *frame)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	unsigned int addr = 0;

	/* processed by ISP */
	if (DCAM_PHYADDR_INVALID(frame->yaddr_vir)) {
		rtn = DCAM_RTN_PATH_ADDR_ERR;
		goto exit;
	}

	addr = dcam_get_frm_addr(frame);

	DCAM_TRACE_INT("binning addr 0x%x\n", addr);
	DCAM_REG_WR(idx, DCAM0_BIN_BASE_WADDR0, addr);
	dcam_print_time("debug shaking: write dcam address (binning)",
		DCAM_TIME_NULL);

	if (log_frame.tx_done_bin) {
		pr_info("tx_done bin[%d], frm 0x%x\n",
			log_frame.tx_done_bin, addr);
		log_frame.tx_done_bin--;
	}

exit:
	return -rtn;
}

int dcam_pdaf_set_next_frm(void *statis_module,
	enum dcam_id idx, enum isp_3a_block_id block_index)
{
	enum dcam_drv_rtn ret = DCAM_RTN_SUCCESS;
	unsigned int statis_addr = 0;
	struct dcam_path_pdaf *pdaf = NULL;
	struct dcam_module *module;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}
	module = s_p_dcam_mod[idx];
	pdaf = &module->dcam_pdaf;
	if (unlikely(block_index != DCAM_PDAF_BLOCK))
		return -1;
	ret = dcam_set_next_statis_buf(statis_module,
				block_index, &statis_addr);
	if (unlikely(ret))
		return -ret;

	if (!statis_addr) {
		pr_err("fail to get pdaf statis buf\n");
		return -1;
	}

	pr_debug("phy_addr=%x, index=%d\n", statis_addr, block_index);

	DCAM_REG_WR(idx, DCAM0_PDAF_BASE_WADDR, statis_addr);
	if (E_PDAF_TYPE3 == (pdaf->pdaf_ctrl.mode & 0x03)) {
		/* type3: 2buffers,pdaf,vch2 path */
		atomic_set(&pdaf_left, 0);
		atomic_set(&pdaf_right, 0);
		DCAM_REG_WR(idx, DCAM0_VCH2_BASE_WADDR,
				statis_addr + ISP_PDAF_STATIS_BUF_SIZE/2);
		sprd_dcam_glb_reg_mwr(idx, DCAM0_CONTROL,
				BIT(16), BIT(16), DCAM_CONTROL_REG);
	}
	sprd_dcam_glb_reg_mwr(idx, DCAM0_CONTROL,
				BIT(14), BIT(14), DCAM_CONTROL_REG);
	return -ret;
}

int dcam_ebd_set_next_frm(void *statis_module,
	enum dcam_id idx, enum isp_3a_block_id block_index)
{
	enum dcam_drv_rtn ret = DCAM_RTN_SUCCESS;
	unsigned int statis_addr = 0;

	ret = dcam_set_next_statis_buf(statis_module,
		block_index, &statis_addr);

	pr_debug("phy_addr=%x, index=%d\n", statis_addr, block_index);
	if (ret) {
		pr_err("fail to set stats buff\n");
		return ret;
	}

	DCAM_REG_WR(idx, DCAM0_VCH3_BASE_WADDR, statis_addr);
	sprd_dcam_glb_reg_mwr(idx, DCAM0_CONTROL,
			BIT(18), BIT(18), DCAM_CONTROL_REG);
	return ret;
}

static void dcam_force_copy(enum dcam_id idx)
{
	unsigned int tmp = 0u;

	if (s_p_dcam_mod[idx]->dcam_full_path.valid)
		tmp |= BIT(8); /* enable full path force copy */
	if (s_p_dcam_mod[idx]->dcam_binning_path.valid)
		tmp |= BIT(10); /* enable binning path force copy */
	if (s_p_dcam_mod[idx]->dcam_aem.valid)
		tmp |= BIT(12);
	if (s_p_dcam_mod[idx]->dcam_pdaf.valid)
		tmp |= BIT(14); /* enable pdaf path force copy */
	if (s_p_dcam_mod[idx]->dcam_raw_path.valid == FULL_RAW_CAPTURE)
		tmp |= BIT(8); /* enable raw path force copy */
	if (s_p_dcam_mod[idx]->dcam_raw_path.valid == BIN_RAW_CAPTURE)
		tmp |= BIT(10); /* enable bin path force copy */

	tmp |= BIT(0); /* enable mipi cap force copy */
	sprd_dcam_glb_reg_mwr(idx, DCAM0_CONTROL,
				BIT(0) | BIT(8) | BIT(10) | BIT(12) | BIT(14),
				tmp, DCAM_CONTROL_REG);
	DCAM_TRACE("force copy 0x%x\n", tmp);
}

static void dcam_auto_copy(enum dcam_id idx)
{
	unsigned int tmp = 0u;

	if (s_p_dcam_mod[idx]->dcam_full_path.valid)
		tmp |= BIT(9); /* enable full path auto copy */
	if (s_p_dcam_mod[idx]->dcam_binning_path.valid)
		tmp |= BIT(11);
	if (s_p_dcam_mod[idx]->dcam_aem.valid)
		tmp |= BIT(13);
	if (s_p_dcam_mod[idx]->dcam_pdaf.valid)
		tmp |= BIT(15);
	if (s_p_dcam_mod[idx]->dcam_raw_path.valid == FULL_RAW_CAPTURE)
		tmp |= BIT(9); /* enable raw path auto copy */
	if (s_p_dcam_mod[idx]->dcam_raw_path.valid == BIN_RAW_CAPTURE)
		tmp |= BIT(11); /* enable bin path auto copy */

	tmp |= BIT(1); /* enable mipi cap auto copy */
	sprd_dcam_glb_reg_mwr(idx, DCAM0_CONTROL,
				BIT(1) | BIT(9) | BIT(11) | BIT(13) | BIT(15),
				tmp, DCAM_CONTROL_REG);
}

void dcam_reg_trace(enum dcam_id idx, unsigned long reg_start,
				unsigned long reg_end)
{
#ifdef DCAM_DRV_DEBUG
#define DCAM0_BASE 0x60800000
	unsigned long addr = 0;
	unsigned int p;

	pr_info("DCAM%d: Register list", idx);
	p = DCAM0_BASE + 0x1000 * idx;
	for (addr = reg_start; addr <= reg_end; addr += 16) {
		pr_info("0x%x: 0x%x 0x%x 0x%x 0x%x\n",
			p,
			DCAM_REG_RD(idx, addr),
			DCAM_REG_RD(idx, addr + 4),
			DCAM_REG_RD(idx, addr + 8),
			DCAM_REG_RD(idx, addr + 12));
		p += 16;
	}
#endif
}

#if 0
static void dcam_raw_path_done_notice(enum dcam_id idx)
{
	struct dcam_path_desc *path = NULL;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	path = &s_p_dcam_mod[idx]->dcam_raw_path;

	DCAM_TRACE("DCAM%d: path done notice %d, %d\n", idx,
		   path->wait_for_done, path->tx_done_com.done);
	if (path->wait_for_done) {
		complete(&path->tx_done_com);
		pr_info("release tx_done_com: %d\n",
			path->tx_done_com.done);
		path->wait_for_done = 0;
	}
}
#endif

static void dcam_sensor_sof(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_SN_SOF];
	void *data = s_user_data[idx][DCAM_SN_SOF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
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
		pr_err("fail to get ptr\n");
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_bin_path_sof(enum dcam_id idx)
{
	struct camera_frame frame = {0};
	dcam_isr_func user_func = s_user_func[idx][DCAM_CAP_SOF];
	void *data = s_user_data[idx][DCAM_CAP_SOF];

	dcam_time_queue_read(&dcam_t_sof.tq[idx].bin_t, &frame.t);

	/* TODO: doesn't consider skip frame */
	if (s_p_dcam_mod[idx]->dcam_binning_path.valid) {
		frame.irq_type = CAMERA_IRQ_DONE;
		frame.irq_property = IRQ_DCAM_SOF;
		frame.flags = ISP_OFF_BUF_BIN;
		frame.fid = s_p_dcam_mod[idx]->dcam_binning_path.sof_cnt++;

		pr_debug("DCAM%d : dcam_bin_path_sof\n",
			idx);

		if (user_func)
			(*user_func) (&frame, data);

		dcam_print_time("debug shaking: Get offline buffer (binning)",
				DCAM_TIME_NULL);

		dcam_bin_path_set_next_frm(idx, &frame);
	}
}

#define timeval_after(a, b) \
	(((a).tv_sec > (b).tv_sec) || \
	(((a).tv_sec = (b).tv_sec) && (((a).tv_usec > (b).tv_usec))))

/* delta indicates the difference of SoF count between paired frames */
static inline void find_delta_helper(enum dcam_id idx, int *delta)
{
	enum dcam_id idx_other = DCAM_OTHER_ID(idx);
	struct dcam_path_desc *path =
		&s_p_dcam_mod[idx]->dcam_full_path;
	struct dcam_path_desc *path_other =
		DCAM_ADDR_INVALID(s_p_dcam_mod[idx_other]) ? NULL :
			&s_p_dcam_mod[idx_other]->dcam_full_path;

	if (!path_other || !delta)
		return;

	if (path->sof_cnt == 1) {
		/* only need to check the first frame */
		if (path_other->sof_cnt >= 1) {
			/* the fisrt frame of other frame has come already */
			if (timeval_after(path->t, path_other->t))
				/* after the other dcam */
				*delta = path_other->sof_cnt - 1;
			else
				/* before the other dcam */
				*delta = path_other->sof_cnt;
			pr_info("[%d] delta = %d\n", idx, *delta);
		}
	}
}

static void dcam_full_path_sof(enum dcam_id idx)
{
	struct camera_frame frame = {0};
	dcam_isr_func user_func = s_user_func[idx][DCAM_CAP_SOF];
	void *data = s_user_data[idx][DCAM_CAP_SOF];
	struct camera_dev *cam_dev = NULL;
	struct isp_pipe_dev *isp_dev = NULL;

	dcam_time_queue_read(&dcam_t_sof.tq[idx].full_t, &frame.t);

	/* TODO: doesn't consider skip frame */
	if (s_p_dcam_mod[idx]->dcam_full_path.valid) {
		cam_dev = (struct camera_dev *)s_p_dcam_mod[idx]->dev_handle;
		isp_dev = (struct isp_pipe_dev *)cam_dev->isp_dev_handle;

		frame.irq_type = CAMERA_IRQ_DONE;
		frame.irq_property = IRQ_DCAM_SOF;
		frame.flags = ISP_OFF_BUF_FULL;
		s_p_dcam_mod[idx]->dcam_full_path.t = frame.t;
		frame.fid = s_p_dcam_mod[idx]->dcam_full_path.sof_cnt++;

		if (is_dual_cam)
			find_delta_helper(idx, &isp_dev->delta_full);

		pr_debug("[%d] SoF frm cnt = %d, time = %ld.%06ld\n",
			idx, frame.fid, frame.t.tv_sec, frame.t.tv_usec);

		if (user_func)
			(*user_func) (&frame, data);

		dcam_print_time("debug shaking: Get offline buffer (full)",
				DCAM_TIME_NULL);

		dcam_full_path_set_next_frm(idx, &frame);

		if (isp_dev->wait_full_tx_done == WAIT_CLEAR)
			atomic_set(&dcam_full_path_time_flag, DCAM_TIME_SOF);
	}
}

static void dcam_cap_sof_handle(enum dcam_id idx)
{
	struct camera_frame frame = {0};
	dcam_isr_func user_func = s_user_func[idx][DCAM_CAP_SOF];
	void *data = s_user_data[idx][DCAM_CAP_SOF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	frame.irq_type = CAMERA_IRQ_DONE;
	frame.irq_property = IRQ_DCAM_SOF;
	frame.flags = ISP_OFF_BUF_NONE;
	if (user_func)
		(*user_func) (&frame, data);
}

static void dcam_cap_sof(enum dcam_id idx)
{
	dcam_cap_sof_handle(idx);
	dcam_bin_path_sof(idx);
	dcam_full_path_sof(idx);

	dcam_auto_copy(idx);
	dcam_print_time("debug shaking: auto_copy", DCAM_TIME_SOF);
}

static void dcam_cap_eof(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_CAP_EOF];
	void *data = s_user_data[idx][DCAM_CAP_EOF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	if (user_func)
		(*user_func) (NULL, data);

	DCAM_TRACE_INT("cap eof\n");
}

static void dcam_ovf(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_OVF];
	enum dcam_id idx_other = DCAM_OTHER_ID(idx);
	void *data = s_user_data[idx][DCAM_OVF];
	void *data_other = s_user_data[idx_other][DCAM_OVF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	pr_err("DCAM%d: fail to work cause ovf\n", idx);

	if (user_func)
		(*user_func) (NULL, data);

	if (is_dual_cam) {
		if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx_other])) {
			pr_err("fail to get ptr for other dcam\n");
			return;
		}

		pr_err("DCAM%d: notify tx error for other idx.\n", idx);

		if (user_func)
			(*user_func) (NULL, data_other);
	}
}

static void dcam_aem_hold_ovf(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_AEM_HOLD_OVF];
	void *data = s_user_data[idx][DCAM_AEM_HOLD_OVF];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	pr_err("DCAM%d: fail to work aem hold overflow\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_isp_enable_pulse(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_ISP_ENABLE_PULSE];
	void *data = s_user_data[idx][DCAM_ISP_ENABLE_PULSE];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	DCAM_TRACE_INT("DCAM%d: isp enable pulse\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_full_path_end(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_FULL_PATH_END];
	void *data = s_user_data[idx][DCAM_FULL_PATH_END];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	DCAM_TRACE_INT("DCAM%d: full path end\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_bin_path_end(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_BIN_PATH_END];
	void *data = s_user_data[idx][DCAM_BIN_PATH_END];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	DCAM_TRACE_INT("DCAM%d: bin path end\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_aem_path_end(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_AEM_PATH_END];
	void *data = s_user_data[idx][DCAM_AEM_PATH_END];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	DCAM_TRACE_INT("DCAM%d: aem path end\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_pdaf_path_end(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_PDAF_PATH_END];
	void *data = s_user_data[idx][DCAM_PDAF_PATH_END];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	DCAM_TRACE_INT("DCAM%d: pdaf path end\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_vch2_path_end(enum dcam_id idx)
{
	/* use the same as pdaf, for type3 of pdaf */
	dcam_isr_func user_func = s_user_func[idx][DCAM_PDAF_PATH_END];
	void *data = s_user_data[idx][DCAM_PDAF_PATH_END];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	DCAM_TRACE_INT("DCAM%d: vch2 path end\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_vch3_path_end(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_VCH3_PATH_END];
	void *data = s_user_data[idx][DCAM_VCH3_PATH_END];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	DCAM_TRACE_INT("DCAM%d: vch3 path end\n", idx);

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_cap_line_err(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_CAP_LINE_ERR];
	enum dcam_id idx_other = DCAM_OTHER_ID(idx);
	void *data = s_user_data[idx][DCAM_CAP_LINE_ERR];
	void *data_other = s_user_data[idx_other][DCAM_CAP_LINE_ERR];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	DCAM_TRACE_INT("DCAM%d: line err\n", idx);

	if (user_func)
		(*user_func) (NULL, data);

	if (is_dual_cam) {
		if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx_other])) {
			pr_err("fail to get ptr for other dcam\n");
			return;
		}

		pr_err("DCAM%d: notify tx error for other idx.\n", idx);

		if (user_func)
			(*user_func) (NULL, data_other);
	}
}

static void dcam_cap_frm_err(enum dcam_id idx)
{
	dcam_isr_func user_func = s_user_func[idx][DCAM_CAP_FRM_ERR];
	enum dcam_id idx_other = DCAM_OTHER_ID(idx);
	void *data = s_user_data[idx][DCAM_CAP_FRM_ERR];
	void *data_other = s_user_data[idx_other][DCAM_CAP_FRM_ERR];

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	pr_err("DCAM%d: fail to work frame err\n", idx);

	if (user_func)
		(*user_func) (NULL, data);

	if (is_dual_cam) {
		if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx_other])) {
			pr_err("fail to get ptr for other dcam\n");
			return;
		}

		pr_err("DCAM%d: notify tx error for other idx.\n", idx);

		if (user_func)
			(*user_func) (NULL, data_other);
	}
}

static void dcam_mmu_int(enum dcam_id idx)
{
	unsigned long addr = 0;
	int mmu_sts;

	if (DCAM_ADDR_INVALID(s_dcam_axi_base)) {
		pr_err("fail to get ptr\n");
		return;
	}

	pr_err("DCAM%d: fail to mmu int\n", idx);
	mmu_sts = DCAM_AXI_REG_RD(DCAM_MMU_STS);
	if (mmu_sts & 0x20) {
		/* VA OUT OF RANGE READ ERR */
		pr_err("fail to read dcam iommu va out of range, addr: 0x%x\n",
			DCAM_AXI_REG_RD(DCAM_MMU_VAOR_ADDR_RD));
	} else if (mmu_sts & 0x10) {
		/* VA OUT OF RANGE WRITE ERR */
		pr_err("fail to write dcam iommu va out of range, addr: 0x%x\n",
			DCAM_AXI_REG_RD(DCAM_MMU_VAOR_ADDR_WR));
	} else if (mmu_sts & 0x8) {
		/* INVALID READ ERR */
		pr_err("fail to read dcam iommu invalid, addr: 0x%x\n",
			DCAM_AXI_REG_RD(DCAM_MMU_INV_ADDR_RD));
	} else if (mmu_sts & 0x4) {
		/* INVALID WRITE ERR */
		pr_err("fail to write dcam iommu invalid, addr: 0x%x\n",
			DCAM_AXI_REG_RD(DCAM_MMU_INV_ADDR_WR));
	} else {
		pr_err("fail to get type dcam iommu other error\n");
	}

	pr_info("DCAM%d:mmu Register list", idx);
	for (addr = DCAM_MMU_EN; addr <= DCAM_MMU_STS; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_AXI_REG_RD(addr),
			DCAM_AXI_REG_RD(addr + 4),
			DCAM_AXI_REG_RD(addr + 8),
			DCAM_AXI_REG_RD(addr + 12));
	}
}

static void dcam_raw_path_done(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	dcam_isr_func user_func;
	void *data;
	struct dcam_path_desc *path;
	struct dcam_module *module;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx]))
		return;

	module = s_p_dcam_mod[idx];
	path = &module->dcam_raw_path;
	if (path->valid == 0) {
		pr_info("DCAM%d: raw path not valid\n", idx);
		return;
	}

	if (path->frame_deci > 1) {
		if (path->done_cnt == 0) {
			path->done_cnt = 1;
		} else {
			path->done_cnt = 0;
			DCAM_TRACE("DCAM%d: raw path dummy done, drop\n",
					idx);
			return;
		}
	} else {
		path->done_cnt = 1;
	}

	user_func = s_user_func[idx][DCAM_FULL_PATH_TX_DONE];
	data = s_user_data[idx][DCAM_FULL_PATH_TX_DONE];
	DCAM_TRACE("dcam%d %p\n", idx, data);
	sprd_dcam_glb_reg_awr(idx, DCAM0_CFG, ~(1 << 0),
				DCAM_CFG_REG);
#if 0
	if (path->need_stop) {
		sprd_dcam_glb_reg_awr(idx, DCAM_CFG, ~(1 << 0),
					      DCAM_CFG_REG);
		path->need_stop = 0;
	}
	dcam_raw_path_done_notice(idx);
#endif
	if (path->need_wait) {
		path->need_wait = 0;
	} else {
		struct camera_frame frame;

		rtn = dcam_frame_dequeue(&path->frame_queue, &frame);
		if (rtn)
			return;
		if (frame.pfinfo.dev == NULL)
			pr_info("DCAM%d done dev NULL %p\n",
				idx, frame.pfinfo.dev);
		pfiommu_free_addr(&frame.pfinfo);
		if (frame.pfinfo.mfd[0] !=
		    module->raw_reserved_frame.pfinfo.mfd[0]) {
			frame.width = path->output_size.w;
			frame.height = path->output_size.h;
			frame.irq_type = CAMERA_IRQ_IMG;

			DCAM_TRACE("DCAM%d: raw path frame %p\n",
				   idx, &frame);
			DCAM_TRACE("y uv, 0x%x 0x%x, mfd = 0x%x,0x%x\n",
				   frame.yaddr, frame.uaddr,
				   frame.pfinfo.mfd[0], frame.pfinfo.mfd[1]);

			if (user_func)
				(*user_func) (&frame, data);
		} else {
			DCAM_TRACE("DCAM%d: use reserved path\n", idx);
		}
	}
}

static void dcam_full_path_tx_done(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct camera_frame frame;
	dcam_isr_func user_func;
	void *data;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx]))
		return;

	user_func = s_user_func[idx][DCAM_FULL_PATH_TX_DONE];
	data = s_user_data[idx][DCAM_FULL_PATH_TX_DONE];

	if (s_p_dcam_mod[idx]->dcam_raw_path.valid == FULL_RAW_CAPTURE) {
		dcam_raw_path_done(idx);
		/*
		 * rtn = dcam_raw_path_set_next_frm(idx);
		 */
		return;
	}

	if (user_func) {
		rtn = (*user_func) (&frame, data);
		if (rtn)
			pr_warn("WARNING: callback function return %d\n", rtn);
	}

	atomic_set(&dcam_full_path_time_flag, DCAM_TIME_TX_DONE);

	dcam_print_time("debug shaking: set offline buffer (full)",
		DCAM_TIME_TX_DONE);

}

static void dcam_binning_path_tx_done(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	dcam_isr_func user_func;
	void *data;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx]))
		return;

	user_func = s_user_func[idx][DCAM_BIN_PATH_TX_DONE];
	data = s_user_data[idx][DCAM_BIN_PATH_TX_DONE];

	if (s_p_dcam_mod[idx]->dcam_raw_path.valid == BIN_RAW_CAPTURE) {
		dcam_raw_path_done(idx);
		/*
		 * rtn = dcam_raw_path_set_next_frm(idx);
		 */
		return;
	}

	if (user_func) {
		rtn = (*user_func) (NULL, data);
		if (rtn)
			pr_warn("WARNING: callback function return %d\n", rtn);
	}

	dcam_print_time("debug shaking: set offline buffer (binning)",
		DCAM_TIME_TX_DONE);
}

static void dcam_pdaf_path_tx_done(enum dcam_id idx)
{
	dcam_isr_func user_func;
	void *data;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx]))
		return;

	user_func = s_user_func[idx][DCAM_PDAF_PATH_TX_DONE];
	data = s_user_data[idx][DCAM_PDAF_PATH_TX_DONE];

	atomic_inc(&pdaf_left);
	if (s_p_dcam_mod[idx]->dcam_pdaf.pdaf_ctrl.mode != E_PDAF_TYPE3)
		atomic_inc(&pdaf_right);

	if (atomic_read(&pdaf_left) > 0 && atomic_read(&pdaf_right) > 0) {
		atomic_set(&pdaf_left, 0);
		atomic_set(&pdaf_right, 0);
		if (user_func)
			(*user_func) (NULL, data);
	}
}

static void dcam_vch2_path_tx_done(enum dcam_id idx)
{
	dcam_isr_func user_func;
	void *data;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx]))
		return;

	user_func = s_user_func[idx][DCAM_VCH2_PATH_TX_DONE];
	data = s_user_data[idx][DCAM_VCH2_PATH_TX_DONE];

	atomic_inc(&pdaf_right);
	if (atomic_read(&pdaf_left) > 0 &&
	    atomic_read(&pdaf_right) > 0) {
		atomic_set(&pdaf_left, 0);
		atomic_set(&pdaf_right, 0);
		if (user_func)
			(*user_func) (NULL, data);
	}
}

static void dcam_vch3_path_tx_done(enum dcam_id idx)
{
	dcam_isr_func user_func;
	void *data;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx]))
		return;

	user_func = s_user_func[idx][DCAM_VCH3_PATH_TX_DONE];
	data = s_user_data[idx][DCAM_VCH3_PATH_TX_DONE];

	if (user_func)
		(*user_func) (NULL, data);
}

static void dcam_aem_path_tx_done(enum dcam_id idx)
{
	dcam_isr_func user_func;
	void *data;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx]))
		return;

	user_func = s_user_func[idx][DCAM_AEM_PATH_TX_DONE];
	data = s_user_data[idx][DCAM_AEM_PATH_TX_DONE];

	if (user_func)
		(*user_func) (NULL, data);
}

static dcam_isr isr_list[DCAM_MAX_COUNT][DCAM_IRQ_NUMBER] = {
	[0][DCAM_SN_SOF] = dcam_sensor_sof,
	[0][DCAM_SN_EOF] = dcam_sensor_eof,
	[0][DCAM_CAP_SOF] = dcam_cap_sof,
	[0][DCAM_CAP_EOF] = dcam_cap_eof,
	[0][DCAM_OVF] = dcam_ovf,
	[0][DCAM_AEM_HOLD_OVF] = dcam_aem_hold_ovf,
	[0][DCAM_ISP_ENABLE_PULSE] = dcam_isp_enable_pulse,
	[0][DCAM_CAP_LINE_ERR] = dcam_cap_line_err,
	[0][DCAM_CAP_FRM_ERR] = dcam_cap_frm_err,
	[0][DCAM_FULL_PATH_END] = dcam_full_path_end,
	[0][DCAM_BIN_PATH_END] = dcam_bin_path_end,
	[0][DCAM_AEM_PATH_END] = dcam_aem_path_end,
	[0][DCAM_PDAF_PATH_END] = dcam_pdaf_path_end,
	[0][DCAM_VCH2_PATH_END] = dcam_vch2_path_end,
	[0][DCAM_VCH3_PATH_END] = dcam_vch3_path_end,
	[0][DCAM_FULL_PATH_TX_DONE] = dcam_full_path_tx_done,
	[0][DCAM_BIN_PATH_TX_DONE] = dcam_binning_path_tx_done,
	[0][DCAM_AEM_PATH_TX_DONE] = dcam_aem_path_tx_done,
	[0][DCAM_PDAF_PATH_TX_DONE] = dcam_pdaf_path_tx_done,
	[0][DCAM_VCH2_PATH_TX_DONE] = dcam_vch2_path_tx_done,
	[0][DCAM_VCH3_PATH_TX_DONE] = dcam_vch3_path_tx_done,
	[0][DCAM_MMU_INT] = dcam_mmu_int,

	[1][DCAM_SN_SOF] = dcam_sensor_sof,
	[1][DCAM_SN_EOF] = dcam_sensor_eof,
	[1][DCAM_CAP_SOF] = dcam_cap_sof,
	[1][DCAM_CAP_EOF] = dcam_cap_eof,
	[1][DCAM_OVF] = dcam_ovf,
	[1][DCAM_AEM_HOLD_OVF] = dcam_aem_hold_ovf,
	[1][DCAM_ISP_ENABLE_PULSE] = dcam_isp_enable_pulse,
	[1][DCAM_CAP_LINE_ERR] = dcam_cap_line_err,
	[1][DCAM_CAP_FRM_ERR] = dcam_cap_frm_err,
	[1][DCAM_FULL_PATH_END] = dcam_full_path_end,
	[1][DCAM_BIN_PATH_END] = dcam_bin_path_end,
	[1][DCAM_AEM_PATH_END] = dcam_aem_path_end,
	[1][DCAM_PDAF_PATH_END] = dcam_pdaf_path_end,
	[1][DCAM_VCH2_PATH_END] = dcam_vch2_path_end,
	[1][DCAM_VCH3_PATH_END] = dcam_vch3_path_end,
	[1][DCAM_FULL_PATH_TX_DONE] = dcam_full_path_tx_done,
	[1][DCAM_BIN_PATH_TX_DONE] = dcam_binning_path_tx_done,
	[1][DCAM_AEM_PATH_TX_DONE] = dcam_aem_path_tx_done,
	[1][DCAM_PDAF_PATH_TX_DONE] = dcam_pdaf_path_tx_done,
	[1][DCAM_VCH2_PATH_TX_DONE] = dcam_vch2_path_tx_done,
	[1][DCAM_VCH3_PATH_TX_DONE] = dcam_vch3_path_tx_done,
	[1][DCAM_MMU_INT] = dcam_mmu_int,
};

static int dcam_err_pre_proc(enum dcam_id idx, unsigned int irq_status)
{
	unsigned long flag;
	void *data = s_user_data[idx][DCAM_OVF];
	unsigned int i;
	unsigned int vect = 0;
	unsigned int irq_line = irq_status;
	unsigned int ret = 0;

	spin_lock_irqsave(&dcam_mod_lock[idx], flag);
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err_ratelimited("fail to get ptr, null irq_status:0x%x\n",
			irq_status);
		goto exit;
	}

	pr_info_ratelimited("DCAM%d: state in err_pre_proc 0x%x, err 0x%x\n",
			idx, s_p_dcam_mod[idx]->state, irq_status);

	pr_info("is_dual_cam %d\n", is_dual_cam);

	if (s_p_dcam_mod[idx]->state & DCAM_STATE_QUICKQUIT)
		goto exit;

	s_p_dcam_mod[idx]->err_happened = 1;
	sprd_dcam_stop(idx, 1, NULL, NULL);
	spin_unlock_irqrestore(&dcam_mod_lock[idx], flag);

	isp_handle_dcam_err(data);

	spin_lock_irqsave(&dcam_mod_lock[idx], flag);
	for (i = 0; i < ARRAY_SIZE(s_irq_err_vect); i++) {
		vect = s_irq_err_vect[i];
		if (irq_line & (1 << (unsigned int)vect)) {
			if (isr_list[idx][vect]) {
				DCAM_TRACE_INT("idx 0x%x %pF\n",
					idx, isr_list[idx][vect]);
				isr_list[idx][vect](idx);
				goto exit;
			}
		}
		irq_line &= ~(unsigned int)(1 << (unsigned int)vect);
		if (!irq_line)
			break;
	}

	pr_err("fail to handle irq 0x%x\n", irq_status);
	ret = -1;
exit:
	spin_unlock_irqrestore(&dcam_mod_lock[idx], flag);
	return ret;
}

static irqreturn_t dcam_isr_root(int irq, void *priv)
{
	unsigned int irq_line, status;
	unsigned long flag;
	int i;
	enum dcam_id idx = DCAM_ID_0;
	int irq_numbers = ARRAY_SIZE(s_irq_vect);
	unsigned int vect = 0;
	struct timeval t;

	if (s_dcam_irq[DCAM_ID_0] == irq)
		idx = DCAM_ID_0;
	else if (s_dcam_irq[DCAM_ID_1] == irq)
		idx = DCAM_ID_1;
	else
		return IRQ_NONE;

	status = DCAM_REG_RD(idx, DCAM0_INT_MASK) & DCAM_IRQ_LINE_MASK;
	if (unlikely(status == 0))
		return IRQ_NONE;
	DCAM_REG_WR(idx, DCAM0_INT_CLR, status);

	irq_line = status;
	if (unlikely(DCAM_IRQ_ERR_MASK & status))
		if (dcam_err_pre_proc(idx, status) == 0)
			return IRQ_HANDLED;
	spin_lock_irqsave(&dcam_lock[idx], flag);

	if (unlikely(irq_line & (1 << (DCAM_CAP_SOF)))) {
		img_get_timestamp(&t);
		dcam_time_queue_write(&dcam_t_sof.tq[idx].bin_t, &t);
		dcam_time_queue_write(&dcam_t_sof.tq[idx].full_t, &t);
	}

	for (i = 0; i < irq_numbers; i++) {
		vect = s_irq_vect[i];
		if (irq_line & (1 << (unsigned int)vect)) {
			if (isr_list[idx][vect]) {
				DCAM_TRACE_INT("idx 0x%x %pF\n",
					idx, isr_list[idx][vect]);
				isr_list[idx][vect](idx);
			}
		}
		irq_line &= ~(unsigned int)(1 << (unsigned int)vect);
		if (!irq_line)
			break;
	}

	spin_unlock_irqrestore(&dcam_lock[idx], flag);

	return IRQ_HANDLED;
}

static int dcam_internal_init(enum dcam_id idx, void *dev_handle)
{
	int ret = 0;

	s_p_dcam_mod[idx] = vzalloc(sizeof(struct dcam_module));
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	s_p_dcam_mod[idx]->dcam_raw_path.id = 0;
	init_completion(&s_p_dcam_mod[idx]->dcam_raw_path.tx_done_com);
	init_completion(&s_p_dcam_mod[idx]->dcam_raw_path.sof_com);
	dcam_buf_queue_init(&s_p_dcam_mod[idx]->dcam_raw_path.buf_queue);
	dcam_time_queue_init(&dcam_t_sof.tq[idx].bin_t);
	dcam_time_queue_init(&dcam_t_sof.tq[idx].full_t);

	s_p_dcam_mod[idx]->dcam_full_path.id = 0;
	init_completion(&s_p_dcam_mod[idx]->dcam_full_path.tx_done_com);
	init_completion(&s_p_dcam_mod[idx]->dcam_full_path.sof_com);
	dcam_buf_queue_init(&s_p_dcam_mod[idx]->dcam_full_path.buf_queue);

	s_p_dcam_mod[idx]->dev_handle = dev_handle;

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

void sprd_dcam_enable_int(enum dcam_id idx)
{
	DCAM_TRACE("Dcam%d,INT 0x%x\n", idx, DCAM_IRQ_LINE_MASK);
	sprd_dcam_glb_reg_owr(idx, DCAM0_INT_EN,
			DCAM_IRQ_LINE_MASK, DCAM_INIT_MASK_REG);
}

void sprd_dcam_disable_int(enum dcam_id idx)
{
	DCAM_TRACE("Dcam%d,INT 0x%x\n", idx, DCAM_IRQ_LINE_MASK);
	sprd_dcam_glb_reg_mwr(idx, DCAM0_CONTROL, BIT_2, 0, DCAM_CONTROL_REG);
	sprd_dcam_glb_reg_owr(idx, DCAM0_INT_CLR,
		DCAM_IRQ_LINE_MASK, DCAM_INIT_CLR_REG);
	sprd_dcam_glb_reg_awr(idx, DCAM0_INT_EN,
		0, DCAM_INIT_MASK_REG);
}

static int sprd_dcam_reset(enum dcam_id idx,
	void *unmap_func, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	unsigned int time_out = 0;
	unsigned int flag = 0;
	enum dcam_id idx_other = DCAM_OTHER_ID(idx);

	pr_info("DCAM%d: reset:\n", idx);

	if (atomic_read(&s_dcam_users[idx])) {
		/* firstly, stop AXI writing */
		sprd_dcam_glb_reg_owr(idx, DCAM_AXIM_CTRL,
			BIT(20) | BIT(19), DCAM_AXI_STS_REG);
	}

	/* then wait for AHB busy cleared */
	while (++time_out < DCAM_AXI_STOP_TIMEOUT) {
		if (0 == (DCAM_AXI_REG_RD(DCAM_AXIM_DBG_STS) &
				DCAM_AXI_DBG_STS_BUSY))
			break;
	}

	if (time_out >= DCAM_AXI_STOP_TIMEOUT) {
		pr_err("DCAM%d: fail to work axi busy timeout %d, %x\n", idx,
			time_out, DCAM_AXI_REG_RD(DCAM_AXIM_DBG_STS));
		rtn = DCAM_RTN_TIMEOUT;
		/*return -rtn;*/
	}

	/* wait for MMU busy too */
	if (sprd_iommu_attach_device(&s_dcam_pdev->dev) == 0) {
		time_out = 0;
		while (++time_out < DCAM_AXI_STOP_TIMEOUT) {
			if (0 == (DCAM_AXI_REG_RD(DCAM_MMU_EN) &
				  DCAM_MMU_BUSY))
				break;
		}

		if (time_out >= DCAM_AXI_STOP_TIMEOUT) {
			pr_err("DCAM%d: fail to work mmu busy timeout %d, %x\n",
			       idx, time_out, DCAM_AXI_REG_RD(DCAM_MMU_EN));
			rtn = DCAM_RTN_TIMEOUT;
			/*return -rtn;*/
		}
	}

	if (unmap_func)
		((dcam_unmap_func)(unmap_func))(param);

	if (idx == DCAM_ID_0)
		flag = BIT_CAM_AHB_DCAM0_SOFT_RST;
	else if (idx == DCAM_ID_1)
		flag = BIT_CAM_AHB_DCAM1_SOFT_RST;

	/* single dcam || the later stopped one of dual-cam */
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx_other]) ||
		(s_p_dcam_mod[idx_other]->state & DCAM_STATE_FIRSTQUIT)) {
		flag |= BIT_CAM_AHB_AXIM_SOFT_RST;
	}

	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, flag);
	udelay(1);
	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, ~flag);

	sprd_dcam_glb_reg_owr(idx, DCAM0_INT_CLR,
		DCAM_IRQ_LINE_MASK, DCAM_INIT_CLR_REG);
	sprd_dcam_glb_reg_awr(idx, DCAM0_INT_EN,
		0, DCAM_INIT_MASK_REG);

	/* the end, enable AXI writing */
	sprd_dcam_glb_reg_awr(idx, DCAM_AXIM_CTRL, ~BIT_19,
		DCAM_AXI_STS_REG);

	sprd_iommu_restore(&s_dcam_pdev->dev);

	pr_info("DCAM%d: reset end\n", idx);

	return -rtn;
}

static void dcam_frm_clear(enum dcam_id idx)
{
	struct camera_frame frame, *res_frame;
	struct dcam_path_desc *raw_path;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	raw_path = &s_p_dcam_mod[idx]->dcam_raw_path;
	while (!dcam_frame_dequeue(&raw_path->frame_queue, &frame))
		pfiommu_free_addr(&frame.pfinfo);

	dcam_frm_queue_clear(&raw_path->frame_queue);
	dcam_buf_queue_init(&raw_path->buf_queue);
	dcam_time_queue_init(&dcam_t_sof.tq[idx].bin_t);
	dcam_time_queue_init(&dcam_t_sof.tq[idx].full_t);

	res_frame = &s_p_dcam_mod[idx]->raw_reserved_frame;
	pfiommu_free_addr(&res_frame->pfinfo);
	memset((void *)res_frame, 0, sizeof(struct camera_frame));
}

#if 0 /* no need as I think */
static void dcam_quickstop_raw(enum dcam_id idx)
{
	unsigned int ret = 0;
	int time_out = 5000;
	struct dcam_path_desc *raw_path = NULL;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	raw_path = &s_p_dcam_mod[idx]->dcam_raw_path;

	sprd_dcam_glb_reg_owr(idx, DCAM0_PATH_STOP, BIT_0, DCAM_PATH_STOP_REG);
	udelay(1000);
	sprd_dcam_glb_reg_mwr(idx, DCAM0_CFG, BIT_0, ~BIT_0, DCAM_CFG_REG);

	/* wait for AHB path busy cleared */
	while (raw_path->valid && time_out) {
		ret = DCAM_REG_RD(idx, DCAM0_PATH_BUSY) & BIT_0;
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
		pr_err("fail to get ptr\n");
		return;
	}

	pdaf = &s_p_dcam_mod[idx]->dcam_pdaf;

	sprd_dcam_glb_reg_owr(idx, DCAM0_PATH_STOP, BIT_3, DCAM_PATH_STOP_REG);
	udelay(1000);
	sprd_dcam_glb_reg_mwr(idx, DCAM0_CFG, BIT_3, ~BIT_3, DCAM_CFG_REG);

	/* wait for AHB path busy cleared */
	while (pdaf->valid && time_out) {
		ret = DCAM_REG_RD(idx, DCAM0_PATH_BUSY) & BIT_3;
		if (!ret)
			break;
		time_out--;
	}
}
#endif

static void dcam_quickstop(enum dcam_id idx)
{
	unsigned int val;
	int time_out = 5000;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return;
	}

	DCAM_TRACE("DCAM%d: state before stop 0x%x\n", idx,
		   s_p_dcam_mod[idx]->state);
/*
 *	dcam_quickstop_raw(idx);
 *	dcam_quickstop_pdaf(idx);
 */
	val = DCAM_REG_RD(idx, DCAM0_CFG);
	DCAM_REG_WR(idx, DCAM0_PATH_STOP, val);
	/* wait not busy */
	while (--time_out) {
		if ((DCAM_REG_RD(idx, DCAM0_PATH_BUSY) & val) == 0)
			break;
	}
	if (!time_out)
		pr_info("dcam quickstop time out %d\n", time_out);

}

static void sprd_dcam_stop_locked(enum dcam_id idx)
{
	dcam_quickstop(idx);

	s_p_dcam_mod[idx]->dcam_raw_path.status = DCAM_ST_STOP;
	s_p_dcam_mod[idx]->dcam_raw_path.valid = 0;
	s_p_dcam_mod[idx]->dcam_raw_path.sof_cnt = 0;
	s_p_dcam_mod[idx]->dcam_raw_path.done_cnt = 0;

	s_p_dcam_mod[idx]->dcam_pdaf.status = DCAM_ST_STOP;
	s_p_dcam_mod[idx]->dcam_pdaf.valid = 0;

	dcam_frm_clear(idx);
}

int sprd_dcam_stop(enum dcam_id idx, int is_irq, void *func, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	unsigned long flag;
	enum dcam_id idx_other = DCAM_OTHER_ID(idx);

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	s_p_dcam_mod[idx]->state |=
		(DCAM_STATE_QUICKQUIT | DCAM_STATE_FIRSTQUIT);

	if (!is_irq)
		spin_lock_irqsave(&dcam_lock[idx], flag);

	if (!DCAM_ADDR_INVALID(s_p_dcam_mod[idx_other])) {
		sprd_dcam_disable_int(idx);
		sprd_dcam_disable_int(idx_other);
	}

	sprd_dcam_stop_locked(idx);

	if (!is_irq)
		spin_unlock_irqrestore(&dcam_lock[idx], flag);

	/* always need to stop both dcam */
	if (!DCAM_ADDR_INVALID(s_p_dcam_mod[idx_other])) {
		unsigned long flag_other;

		s_p_dcam_mod[idx_other]->state |= DCAM_STATE_QUICKQUIT;

		if (!is_irq)
			spin_lock_irqsave(&dcam_lock[idx_other], flag_other);

		sprd_dcam_stop_locked(idx_other);

		if (!is_irq)
			spin_unlock_irqrestore(&dcam_lock[idx_other],
				flag_other);
	}

	sprd_dcam_reset(idx, func, param);

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
	case DCAM_AXI_STS_REG:
		spin_lock_irqsave(&dcam_glb_reg_axi_sts_lock[0], flag);
		DCAM_AXI_REG_WR(addr, DCAM_AXI_REG_RD(addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_axi_sts_lock[0], flag);
		break;
	case DCAM_ENDIAN_REG:
		spin_lock_irqsave(&dcam_glb_reg_endian_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_endian_lock[idx], flag);
		break;
	case DCAM_AXI_ENDIAN_REG:
		spin_lock_irqsave(&dcam_glb_reg_axi_endian_lock, flag);
		DCAM_AXI_REG_WR(addr, DCAM_AXI_REG_RD(addr) & (val));
		spin_unlock_irqrestore(&dcam_glb_reg_axi_endian_lock, flag);
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
	case DCAM_AXI_STS_REG:
		spin_lock_irqsave(&dcam_glb_reg_axi_sts_lock[0], flag);
		DCAM_AXI_REG_WR(addr, DCAM_AXI_REG_RD(addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_axi_sts_lock[0], flag);
		break;
	case DCAM_ENDIAN_REG:
		spin_lock_irqsave(&dcam_glb_reg_endian_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_endian_lock[idx], flag);
		break;
	case DCAM_AXI_ENDIAN_REG:
		spin_lock_irqsave(&dcam_glb_reg_axi_endian_lock, flag);
		DCAM_AXI_REG_WR(addr, DCAM_AXI_REG_RD(addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_axi_endian_lock, flag);
		break;
	case DCAM_PATH_STOP_REG:
		spin_lock_irqsave(&dcam_glb_reg_path_stop_lock[idx], flag);
		DCAM_REG_WR(idx, addr, DCAM_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&dcam_glb_reg_path_stop_lock[idx], flag);
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
		tmp = DCAM_REG_RD(idx, addr);
		tmp &= ~(mask);
		DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_cfg_lock[idx], flag);
		break;
	case DCAM_CONTROL_REG:
		spin_lock_irqsave(&dcam_glb_reg_control_lock[idx], flag);
		tmp = DCAM_REG_RD(idx, addr);
		tmp &= ~(mask);
		DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_control_lock[idx], flag);
		break;
	case DCAM_INIT_MASK_REG:
		spin_lock_irqsave(&dcam_glb_reg_mask_lock[idx], flag);
		tmp = DCAM_REG_RD(idx, addr);
		tmp &= ~(mask);
		DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_mask_lock[idx], flag);
		break;
	case DCAM_INIT_CLR_REG:
		spin_lock_irqsave(&dcam_glb_reg_clr_lock[idx], flag);
		tmp = DCAM_REG_RD(idx, addr);
		tmp &= ~(mask);
		DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_clr_lock[idx], flag);
		break;
	case DCAM_AXI_STS_REG:
		spin_lock_irqsave(&dcam_glb_reg_axi_sts_lock[0], flag);
		tmp = DCAM_AXI_REG_RD(addr);
		tmp &= ~(mask);
		DCAM_AXI_REG_WR(addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_axi_sts_lock[0], flag);
		break;
	case DCAM_ENDIAN_REG:
		spin_lock_irqsave(&dcam_glb_reg_endian_lock[idx], flag);
		tmp = DCAM_REG_RD(idx, addr);
		tmp &= ~(mask);
		DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_endian_lock[idx], flag);
		break;
	case DCAM_AXI_ENDIAN_REG:
		spin_lock_irqsave(&dcam_glb_reg_axi_endian_lock, flag);
		tmp = DCAM_AXI_REG_RD(addr);
		tmp &= ~(mask);
		DCAM_AXI_REG_WR(addr, tmp | ((mask) & (val)));
		spin_unlock_irqrestore(&dcam_glb_reg_axi_endian_lock, flag);
		break;
	default:
		tmp = DCAM_REG_RD(idx, addr);
		tmp &= ~(mask);
		DCAM_REG_WR(idx, addr, tmp | ((mask) & (val)));
		break;
	}
}

int sprd_camera_get_path_id(struct camera_get_path_id *path_id,
	unsigned int *channel_id, unsigned int scene_mode)
{
	int ret = DCAM_RTN_SUCCESS;

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
		path_id->is_path_work[CAMERA_RAW_PATH],
		path_id->is_path_work[CAMERA_PRE_PATH],
		path_id->is_path_work[CAMERA_VID_PATH],
		path_id->is_path_work[CAMERA_CAP_PATH]);
	pr_info("DCAM: scene_mode %d, need_isp_tool %d\n", scene_mode,
		path_id->need_isp_tool);
	/* TODO: which is PDAF/AEM PATH */
	if (path_id->need_isp_tool)
		*channel_id = CAMERA_RAW_PATH;
	else if (path_id->fourcc == V4L2_PIX_FMT_GREY &&
		 !path_id->is_path_work[CAMERA_RAW_PATH])
		*channel_id = CAMERA_RAW_PATH;
	else if (path_id->fourcc == V4L2_PIX_FMT_JPEG &&
		 !path_id->is_path_work[CAMERA_RAW_PATH])
		*channel_id = CAMERA_RAW_PATH;
	else if (path_id->rt_refocus &&
		 !path_id->is_path_work[CAMERA_RAW_PATH])
		*channel_id = CAMERA_RAW_PATH;
	else if (scene_mode == DCAM_SCENE_MODE_PREVIEW)
		if (unlikely(path_id->output_size.w >
		    ISP_PATH1_LINE_BUF_LENGTH &&
		    !path_id->is_path_work[CAMERA_CAP_PATH]))
			*channel_id = CAMERA_CAP_PATH;
		else
			*channel_id = CAMERA_PRE_PATH;
	else if (scene_mode == DCAM_SCENE_MODE_RECORDING)
		*channel_id = CAMERA_VID_PATH;
	else if (scene_mode == DCAM_SCENE_MODE_CAPTURE)
		*channel_id = CAMERA_CAP_PATH;
	else if (scene_mode == DCAM_SCENE_MODE_CAPTURE_CALLBACK) {
		if (path_id->output_size.w <= ISP_PATH2_LINE_BUF_LENGTH
		    && !path_id->is_path_work[CAMERA_VID_PATH])
			*channel_id = CAMERA_VID_PATH;
		else
			*channel_id = CAMERA_CAP_PATH;
	} else {
		*channel_id = CAMERA_RAW_PATH;
		pr_info("path select error\n");
	}

	pr_info("path id %d\n", *channel_id);

	return ret;
}

static int dcam_enable_clk(void)
{
	int ret = 0;

	/*set dcam_if clock to max value*/
	ret = clk_set_parent(dcam0_clk, dcam0_clk_parent);
	if (ret) {
		pr_err("fail to set dcam0_clk_parent\n");
		clk_set_parent(dcam0_clk, dcam0_clk_default);
		clk_disable_unprepare(dcam0_eb);
		goto exit;
	}

	ret = clk_prepare_enable(dcam0_clk);
	if (ret) {
		pr_err("fail to set dcam0_clk\n");
		clk_set_parent(dcam0_clk, dcam0_clk_default);
		clk_disable_unprepare(dcam0_eb);
		goto exit;
	}

	/*dcam enable*/
	ret = clk_prepare_enable(dcam0_eb);
	if (ret) {
		pr_err("fail to set dcam0_enable\n");
		goto exit;
	}

	/* if (s_dcam_count == DCAM_MAX_COUNT) {
	* clk_prepare_enable(dcam0_eb);
	* clk_prepare_enable(dcam0_mm_eb);
	* //Add more if support dual camera
	* }
	*/
exit:
	pr_info("dcam_enable_clk end.\n");

	return ret;
}

static int dcam_disable_clk(enum dcam_id idx)
{
	/*cut off the dcam_if colck source*/
	clk_disable_unprepare(dcam0_eb);

	/*set dcam_if clock to default value before power off*/
	clk_set_parent(dcam0_clk, dcam0_clk_default);
	clk_disable_unprepare(dcam0_clk);

/*
*	if (s_dcam_count == DCAM_MAX_COUNT) {
*		clk_disable_unprepare(dcam0_eb);
*		clk_disable_unprepare(dcam0_mm_eb);
*		// Add more if support dual camera
*	}
*/
	pr_info("dcam_disable_clk end.\n");

	return 0;
}

int set_dcam_cap_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_cap_desc *cap_desc = NULL;
	struct camera_rect *rect;
	unsigned int tmp = 0;


	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
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
				DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_CFG, BIT_7,
					cap_sensor_mode << 7);
				cap_desc->input_format = cap_sensor_mode;
			}
			break;
		}

	case DCAM_CAP_BAYER_PATTERN:
		{
			uint32_t bayer_pattern = *(uint32_t *)param;

			DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_CFG, BIT_2 | BIT_1,
				     bayer_pattern << 1);
			break;
		}
	case DCAM_CAP_YUV_TYPE:
		{
			uint32_t yuv_type = *(uint32_t *)param;

			DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_CFG, BIT_9 | BIT_8,
				     yuv_type << 8);
			break;
		}
	case DCAM_CAP_SYNC_POL:
		{
			struct dcam_cap_sync_pol *sync_pol =
				(struct dcam_cap_sync_pol *)param;

			if (sync_pol->need_href)
				DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_CFG,
					BIT_5, 1 << 5);
			else
				DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_CFG,
					BIT_5, 0 << 5);
			break;
		}

	case DCAM_CAP_DATA_BITS:
		{
			enum dcam_cap_data_bits bits = *(unsigned int *)param;

			if (bits == DCAM_CAP_12_BITS)
				DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_CFG,
				BIT_4 | BIT_3, 2 << 3);
			else if (bits == DCAM_CAP_10_BITS)
				DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_CFG,
				BIT_4 | BIT_3, 1 << 3);
			else if (bits == DCAM_CAP_8_BITS)
				DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_CFG,
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
				DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_FRM_CTRL,
					BIT_3 | BIT_2 | BIT_1 | BIT_0,
					skip_num);
			break;
		}

	case DCAM_CAP_FRM_DECI:
		{
			unsigned int deci_factor = *(unsigned int *)param;

			if (deci_factor < DCAM_FRM_DECI_FAC_MAX)
				DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_FRM_CTRL,
				BIT_5 | BIT_4,
					deci_factor << 4);
			else
				rtn = DCAM_RTN_CAP_FRAME_DECI_ERR;
			break;
		}

	case DCAM_CAP_FRM_PULSE_LINE:
		{
			uint32_t line_num = *(uint32_t *)param;

			if ((cap_desc->cap_rect.y < line_num) &&
			    (line_num <=
			     (cap_desc->cap_rect.y +
			      cap_desc->cap_rect.h - 1)))
				DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_FRM_CTRL,
					     0x00fff000,
					     line_num << 12);
			else if (line_num != 0)
				pr_err("fail to set pulse line %d y h %d %d\n",
				       line_num,
				       cap_desc->cap_rect.y,
				       cap_desc->cap_rect.h);
			break;
		}

	case DCAM_CAP_FRM_COUNT_CLR:
		DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_FRM_CLR, BIT_0, 1);
		break;

	case DCAM_CAP_INPUT_RECT:
		rect = (struct camera_rect *)param;
		/* check align, start:2, end:4 */
		rect->x &= (~BIT(0));
		rect->y &= (~BIT(0));
		rect->w = ((rect->x + rect->w) & (~(BIT(0)|BIT(1)))) - rect->x;
		rect->h = ((rect->y + rect->h) & (~(BIT(0)|BIT(1)))) - rect->y;
		if (rect->x > DCAM_CAP_FRAME_WIDTH_MAX ||
		    rect->y > DCAM_CAP_FRAME_HEIGHT_MAX ||
		    rect->w > DCAM_CAP_FRAME_WIDTH_MAX ||
		    rect->h > DCAM_CAP_FRAME_HEIGHT_MAX ||
		    (rect->x + rect->w) < 4 ||
		    (rect->y + rect->h) < 2) { /* add lower limit */
			rtn = DCAM_RTN_CAP_FRAME_SIZE_ERR;
			return -rtn;
		}
		cap_desc->cap_rect = *rect;
		/* start */
		tmp = (rect->x) | (rect->y << 16);
		DCAM_REG_WR(idx, DCAM0_MIPI_CAP_START, tmp);
		/* end - 1*/
		tmp = (rect->x + rect->w);
		tmp -= 1;
		tmp |= (rect->y + rect->h - 1) << 16;
		DCAM_REG_WR(idx, DCAM0_MIPI_CAP_END, tmp);
		break;

	case DCAM_CAP_IMAGE_XY_DECI:
		{
			struct dcam_cap_dec *cap_dec =
				(struct dcam_cap_dec *)param;

			if (cap_dec->x_factor > DCAM_CAP_X_DECI_FAC_MAX ||
			    cap_dec->y_factor > DCAM_CAP_Y_DECI_FAC_MAX) {
				rtn = DCAM_RTN_CAP_XY_DECI_ERR;
			} else {
				if (cap_desc->input_format !=
					DCAM_CAP_MODE_YUV) {
					pr_warn("Only yuv can x,y-deci\n");
					rtn = DCAM_RTN_SUCCESS;
				} else { /* only for yuv */
					unsigned int tmp = 0;

					tmp = cap_dec->x_factor << 16;
					tmp |= (cap_dec->y_factor << 18);
					DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_CFG,
						(0xF << 16), tmp);
				}
			}
			break;
		}

	case DCAM_CAP_DATA_PACKET:
		{
			unsigned int is_loose = *(unsigned int *)param;

			if (cap_desc->cam_if == DCAM_CAP_IF_CSI2 &&
			    cap_desc->input_format == DCAM_CAP_MODE_RAWRGB) {
				/*[todo] full or bin*/
				int value = is_loose ? 1 : 0;

				/* full & bin path use the same pack mode. */
				DCAM_REG_MWR(idx, DCAM0_FULL_CFG,
					BIT_0, value);
				DCAM_REG_MWR(idx, DCAM0_CAM_BIN_CFG,
					BIT_0, value);
			} else
				rtn = DCAM_RTN_MODE_ERR;
			break;
		}

	case DCAM_CAP_SAMPLE_MODE:
		{ /* single frame or multiframe */
			enum dcam_capture_mode samp_mode =
				*(enum dcam_capture_mode *)param;

			if (samp_mode >= DCAM_CAPTURE_MODE_MAX) {
				rtn = DCAM_RTN_MODE_ERR;
			} else {
				DCAM_REG_MWR(idx, DCAM0_MIPI_CAP_CFG, BIT_6,
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

int set_dcam_raw_path_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *raw_path = NULL;
	struct camera_addr *p_addr;
	struct dcam_module *module;
	unsigned int base_id;
	struct camera_endian_sel *endian;
	struct size_transfer *tmp;
	struct camera_size size;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	if (0 == (unsigned long)(param)) {
		pr_err("fail to get parm null\n");
		return -DCAM_RTN_PARA_ERR;
	}

	module = s_p_dcam_mod[idx];
	raw_path = &module->dcam_raw_path;

	switch (id) {
	case DCAM_PATH_OUTPUT_ADDR:
		p_addr = (struct camera_addr *)param;

		if (DCAM_YUV_ADDR_INVALID(p_addr->yaddr, p_addr->uaddr,
			p_addr->vaddr) && p_addr->mfd_y == 0) {

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

			frame.type = CAMERA_RAW_PATH;
			frame.fid = raw_path->frame_base_id;

			frame.pfinfo.dev = &s_dcam_pdev->dev;
			frame.pfinfo.mfd[0] = p_addr->mfd_y;
			frame.pfinfo.mfd[1] = p_addr->mfd_u;
			frame.pfinfo.mfd[2] = p_addr->mfd_u;
			/*may need update iommu here*/
			rtn = pfiommu_get_sg_table(&frame.pfinfo);
			if (rtn) {
				pr_err("fail to cfg output addr\n");
				rtn = DCAM_RTN_PATH_ADDR_ERR;
				break;
			}

			frame.pfinfo.offset[0] = 0;
			frame.pfinfo.offset[1] = 0;
			frame.pfinfo.offset[2] = 0;

			if (!dcam_buf_queue_write(&raw_path->buf_queue,
						  &frame))
				raw_path->output_frame_count++;

			DCAM_TRACE("DCAM%d: raw path set output addr, i %d\n",
				   idx, raw_path->output_frame_count);
			DCAM_TRACE("y=0x%x u=0x%x v=0x%x mfd=0x%x 0x%x\n",
				   p_addr->yaddr, p_addr->uaddr,
				   p_addr->vaddr, frame.pfinfo.mfd[0],
				   frame.pfinfo.mfd[1]);
		}
		break;

	case DCAM_PATH_OUTPUT_RESERVED_ADDR:
		p_addr = (struct camera_addr *)param;

		if (DCAM_YUV_ADDR_INVALID(p_addr->yaddr, p_addr->uaddr,
			p_addr->vaddr) && p_addr->mfd_y == 0) {

			rtn = DCAM_RTN_PATH_ADDR_ERR;
		} else {
			unsigned int output_frame_count = 0;
			struct camera_frame *frame = NULL;
			/* struct xx_spec. --> struct xx_desc. */
			frame = &module->raw_reserved_frame;
			output_frame_count = raw_path->output_frame_count;
			frame->yaddr = p_addr->yaddr;
			frame->uaddr = p_addr->uaddr;
			frame->vaddr = p_addr->vaddr;
			frame->yaddr_vir = p_addr->yaddr_vir;
			frame->uaddr_vir = p_addr->uaddr_vir;
			frame->vaddr_vir = p_addr->vaddr_vir;

			frame->pfinfo.dev = &s_dcam_pdev->dev;
			frame->pfinfo.mfd[0] = p_addr->mfd_y;
			frame->pfinfo.mfd[1] = p_addr->mfd_u;
			frame->pfinfo.mfd[2] = p_addr->mfd_u;
			/*may need update iommu here*/
			rtn = pfiommu_get_sg_table(&frame->pfinfo);
			if (rtn) {
				pr_err("fail to cfg reserved output addr\n");
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
		raw_path->frame_base_id = base_id;
		break;

	case DCAM_PATH_DATA_ENDIAN:
		endian = (struct camera_endian_sel *)param;

		if (endian->y_endian >= DCAM_ENDIAN_MAX) {
			rtn = DCAM_RTN_PATH_ENDIAN_ERR;
		} else {
			raw_path->data_endian.y_endian = endian->y_endian;
			raw_path->valid_param.data_endian = 1;
		}
		break;
	case DCAM_PATH_BIN_RATIO:
		raw_path->bin_ratio = *(unsigned int *)param;
		if (raw_path->bin_ratio > 2) {
			pr_warn("raw bin ratio erro, use 0\n");
			raw_path->bin_ratio = 0;
		}
		break;
	case DCAM_PATH_INPUT_RECT:
		tmp = (struct size_transfer *)param;
		size = *(tmp->pin);
		pr_debug("tmp->prect->x,y,w,h:%d, %d, %d, %d\n",
					tmp->prect->x,
					tmp->prect->y,
					tmp->prect->w,
					tmp->prect->h);

		if ((tmp->prect->x + tmp->prect->w) > tmp->pin->w ||
			(tmp->prect->y + tmp->prect->h) > tmp->pin->h) {
			pr_err("fail to input raw input rect\n");
			rtn = DCAM_RTN_PATH_TRIM_SIZE_ERR;
			goto exit;
		} else if (raw_path->valid == FULL_RAW_CAPTURE) {
			raw_path->input_rect = *(tmp->prect); /* crop */
			if (tmp->prect->w == tmp->pin->w &&
				tmp->prect->h == tmp->pin->h) {
				/* not need crop */
				raw_path->valid_param.input_rect = 0;
				pr_debug("not need crop\n");
			} else {
				raw_path->valid_param.input_rect = 1;
				/* crop align w:4,h:2 */
				raw_path->input_rect.w =
					DCAM_SIZE_ALIGN(tmp->prect->w, 2);
				raw_path->input_rect.h =
					DCAM_SIZE_ALIGN(tmp->prect->h, 1);
				size.w  = raw_path->input_rect.w;
				size.h  = raw_path->input_rect.h;

			}
		}
		if (raw_path->valid == BIN_RAW_CAPTURE) {
			if ((tmp->prect->x + tmp->prect->w) > size.w ||
				(tmp->prect->y + tmp->prect->h) > size.h ||
				(0 == (tmp->prect->x + tmp->prect->y +
				tmp->prect->w + tmp->prect->h))) {
				/* all 0 or to large */
				pr_err("fail to input bin input rect\n");
				rtn = DCAM_RTN_PATH_TRIM_SIZE_ERR;
				goto exit;
			}

			raw_path->valid_param.input_rect = 0;
			raw_path->input_rect.x = 0;
			raw_path->input_rect.y = 0;
			raw_path->input_rect.w = tmp->pin->w;
			raw_path->input_rect.h = tmp->pin->h;
			/* bypass binning */
			if (raw_path->bin_ratio == 0) {
				DCAM_REG_MWR(idx, DCAM0_CAM_BIN_CFG,
					       BIT(2), BIT(2));
				DCAM_REG_MWR(idx, DCAM0_BIN_CORE_CTRL,
					       BIT(1), BIT(1));
				*(tmp->pout) = size;
				pr_info("raw 1/1 binning\n");
			} else{
				/* not bypass */
				DCAM_REG_MWR(idx, DCAM0_CAM_BIN_CFG,
						BIT(2), 0);
				align_for_bin(&size, raw_path->bin_ratio);
				if (raw_path->bin_ratio == 1) {
					 /* 1/2 binning */
					DCAM_REG_MWR(idx, DCAM0_BIN_CORE_CTRL,
							BIT(1), ~BIT(1));
					pr_info("1/2 binning\n");

				} else {
					 /* 1/4 binning */
					DCAM_REG_MWR(idx, DCAM0_BIN_CORE_CTRL,
							BIT(1),	BIT(1));
					pr_info("1/4 binning\n");
				}
				tmp->pout->w = size.w >> raw_path->bin_ratio;
				tmp->pout->h = size.h >> raw_path->bin_ratio;
				/* If not align for binning, need crop */
				if (raw_path->input_rect.w != size.w ||
					raw_path->input_rect.h != size.h) {
					raw_path->valid_param.input_rect = 1;
					raw_path->input_rect.x = 0;
					raw_path->input_rect.y = 0;
					raw_path->input_rect.w = size.w;
					raw_path->input_rect.h = size.h;
					/* isp trim0 size can't larger than
					 * the size of dcam crop
					 */
					if (tmp->prect->w > size.w)
						tmp->prect->w = size.w;
					if (tmp->prect->h > size.h)
						tmp->prect->h = size.h;
					/* refresh trim size for isp */
				}
				refresh_bin_trim_size(tmp->prect,
							raw_path->bin_ratio);

			}
		}
		break;
	case DCAM_PATH_FRM_DECI:
		{
			unsigned int deci_factor = *(unsigned int *) param;

			if (deci_factor >= DCAM_FRM_DECI_FAC_MAX) {
				rtn = DCAM_RTN_PATH_FRM_DECI_ERR;
			} else {
				raw_path->frame_deci = deci_factor;
				raw_path->valid_param.frame_deci = 1;
			}
			break;
		}

	case DCAM_PATH_ENABLE:
		raw_path->valid = *(unsigned int *)param;
		break;

	default:
		pr_err("fail to get type\n");
		break;
	}
exit:
	return -rtn;
}

int set_dcam_full_path_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *path = NULL;
	struct dcam_module *module;
	unsigned int base_id;
	struct camera_endian_sel *endian;
	struct size_transfer *tmp;


	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	if (0 == (unsigned long)(param)) {
		pr_err("fail to get parm null\n");
		return -DCAM_RTN_PARA_ERR;
	}

	module = s_p_dcam_mod[idx];
	path = &module->dcam_full_path;

	switch (id) {
	case DCAM_PATH_FRAME_BASE_ID:
		base_id = *(unsigned int *)param;

		DCAM_TRACE("DCAM%d: set frame base id 0x%x\n",
			   idx, base_id);
		path->frame_base_id = base_id;
		break;

	case DCAM_PATH_DATA_ENDIAN:
		endian = (struct camera_endian_sel *)param;

		if (endian->y_endian >= DCAM_ENDIAN_MAX) {
			rtn = DCAM_RTN_PATH_ENDIAN_ERR;
		} else {
			path->data_endian.y_endian = endian->y_endian;
			path->valid_param.data_endian = 1;
		}
		break;
	case DCAM_PATH_BIN_RATIO:
		path->bin_ratio = *(unsigned int *)param;
		/* full path not support binning */
		path->bin_ratio = 0;
		break;
	case DCAM_PATH_INPUT_RECT:
		tmp = (struct size_transfer *)param;
		if ((tmp->prect->x + tmp->prect->w) > tmp->pin->w ||
			(tmp->prect->y + tmp->prect->h) > tmp->pin->h) {
			pr_err("fail to input full input rect\n");
			rtn = DCAM_RTN_PATH_TRIM_SIZE_ERR;
		} else {
			/*dcam not crop */
			path->valid_param.input_rect = 0;
			tmp->pout->w = tmp->pin->w;
			tmp->pout->h = tmp->pin->h;
			path->input_rect.x = 0;
			path->input_rect.y = 0;
			path->input_rect.w = tmp->pin->w;
			path->input_rect.h = tmp->pin->h;
		}
		break;
	case DCAM_PATH_FRM_DECI:
		{
			unsigned int deci_factor = *(unsigned int *) param;

			if (deci_factor >= DCAM_FRM_DECI_FAC_MAX) {
				rtn = DCAM_RTN_PATH_FRM_DECI_ERR;
			} else {
				path->frame_deci = deci_factor;
				path->valid_param.frame_deci = 1;
			}
			break;
		}

	case DCAM_PATH_ENABLE:
		path->valid = *(unsigned int *)param;
		break;
	case DCAM_PATH_OUTPUT_ADDR: /* offline, no need */
	case DCAM_PATH_OUTPUT_RESERVED_ADDR:
		break;
	default:
		pr_err("fail to get type\n");
		break;
	}

	return -rtn;
}

int set_dcam_bin_path_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *path = NULL;
	struct dcam_module *module;
	unsigned int base_id;
	struct camera_endian_sel *endian;
	struct size_transfer *tmp;
	struct camera_size size;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	if (0 == (unsigned long)(param)) {
		pr_err("fail to get parm null\n");
		return -DCAM_RTN_PARA_ERR;
	}

	module = s_p_dcam_mod[idx];
	path = &module->dcam_binning_path;

	switch (id) {
	case DCAM_PATH_FRAME_BASE_ID:
		base_id = *(unsigned int *)param;

		DCAM_TRACE("DCAM%d: set frame base id 0x%x\n",
			   idx, base_id);
		path->frame_base_id = base_id;
		break;

	case DCAM_PATH_DATA_ENDIAN:
		endian = (struct camera_endian_sel *)param;

		if (endian->y_endian >= DCAM_ENDIAN_MAX) {
			rtn = DCAM_RTN_PATH_ENDIAN_ERR;
		} else {
			path->data_endian.y_endian = endian->y_endian;
			path->valid_param.data_endian = 1;
		}
		break;
	case DCAM_PATH_BIN_RATIO:
		path->bin_ratio = *(unsigned int *)param;
		if (path->bin_ratio > 2) {
			pr_warn("bin path bin ratio erro, use 0\n");
			path->bin_ratio = 0;
		}
		break;
	case DCAM_PATH_INPUT_RECT:
		tmp = (struct size_transfer *)param;
		size = *(tmp->pin);
		if ((tmp->prect->x + tmp->prect->w) > size.w ||
			(tmp->prect->y + tmp->prect->h) > size.h ||
			(0 == (tmp->prect->x + tmp->prect->y +
			tmp->prect->w + tmp->prect->h))) {
			/* all 0 or to large */
			pr_err("fail to input bin input rect\n");
			rtn = DCAM_RTN_PATH_TRIM_SIZE_ERR;
			goto exit;
		}
		/* default dcam do not crop */
		path->valid_param.input_rect = 0;
		path->input_rect.x = 0;
		path->input_rect.y = 0;
		path->input_rect.w = tmp->pin->w;
		path->input_rect.h = tmp->pin->h;

		/* bypass binning */
		if (path->bin_ratio == 0) {
			DCAM_REG_MWR(idx, DCAM0_CAM_BIN_CFG, BIT(2), BIT(2));
			*(tmp->pout) = size;
			pr_info("1/1 binning\n");
		} else{
			/* not bypass */
			DCAM_REG_MWR(idx, DCAM0_CAM_BIN_CFG, BIT(2), 0);
			align_for_bin(&size, path->bin_ratio);
			if (path->bin_ratio == 1) { /* 1/2 binning */
				DCAM_REG_MWR(idx, DCAM0_BIN_CORE_CTRL, BIT(1),
						~BIT(1));
				pr_info("1/2 binning\n");

			} else { /* 1/4 binning */
				DCAM_REG_MWR(idx, DCAM0_BIN_CORE_CTRL, BIT(1),
						BIT(1));
				pr_info("1/4 binning\n");
			}
			tmp->pout->w = size.w >> path->bin_ratio;
			tmp->pout->h = size.h >> path->bin_ratio;
			/* If not align for binning, need crop */
			if (path->input_rect.w != size.w ||
				path->input_rect.h != size.h) {
				path->valid_param.input_rect = 1;
				path->input_rect.x = 0;
				path->input_rect.y = 0;
				path->input_rect.w = size.w;
				path->input_rect.h = size.h;
				/* isp trim0 size can't larger than
				 * the size of dcam crop
				 */
				if (tmp->prect->w > size.w)
					tmp->prect->w = size.w;
				if (tmp->prect->h > size.h)
					tmp->prect->h = size.h;
				pr_info("Not align for bin,crop: %d %d %d %d\n",
					path->input_rect.x,
					path->input_rect.y,
					path->input_rect.w,
					path->input_rect.h);
			}
			/* refresh trim size for isp */
			refresh_bin_trim_size(tmp->prect, path->bin_ratio);
		}
		pr_info("dcam in:%d %d, out:%d %d,isp trim:%d %d %d %d\n",
				tmp->pin->w, tmp->pin->h,
				tmp->pout->w, tmp->pout->h,
				tmp->prect->x, tmp->prect->y,
				tmp->prect->w, tmp->prect->h);

		break;

	case DCAM_PATH_FRM_DECI:
		{
			unsigned int deci_factor = *(unsigned int *) param;

			if (deci_factor >= DCAM_FRM_DECI_FAC_MAX) {
				rtn = DCAM_RTN_PATH_FRM_DECI_ERR;
			} else {
				path->frame_deci = deci_factor;
				path->valid_param.frame_deci = 1;
			}
			break;
		}

	case DCAM_PATH_ENABLE:
		path->valid = *(unsigned int *)param;
		break;
	case DCAM_PATH_OUTPUT_ADDR: /* offline, no need */
	case DCAM_PATH_OUTPUT_RESERVED_ADDR:
		break;
	default:
		pr_err("fail to get type\n");
		break;
	}
exit:
	return -rtn;
}

int set_dcam_pdaf_path_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_pdaf *path_pdaf = NULL;
	struct camera_addr *p_addr;
	struct dcam_module *module;
	unsigned int base_id;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	if (0 == (unsigned long)(param)) {
		pr_err("fail to get parm null\n");
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

			frame.type = CAMERA_RAW_PATH;
			frame.fid = path_pdaf->frame_base_id;

			frame.pfinfo.dev = &s_dcam_pdev->dev;
			frame.pfinfo.mfd[0] = p_addr->mfd_y;
			frame.pfinfo.mfd[1] = p_addr->mfd_u;
			frame.pfinfo.mfd[2] = p_addr->mfd_u;
			/* may need update iommu here */
			rtn = pfiommu_get_sg_table(&frame.pfinfo);
			if (rtn) {
				pr_err("fail to cfg output addr\n");
				rtn = DCAM_RTN_PATH_ADDR_ERR;
				break;
			}

			frame.pfinfo.offset[0] = frame.yaddr;
			frame.pfinfo.offset[1] = frame.uaddr;
			frame.pfinfo.offset[2] = frame.vaddr;

			if (!dcam_buf_queue_write(&path_pdaf->buf_queue,
						  &frame))
				path_pdaf->output_frame_count++;

			DCAM_TRACE("DCAM%d: pdaf path set output addr, i %d\n",
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

			frame = &module->pdaf_reserved_frame;
			output_frame_count = path_pdaf->output_frame_count;
			frame->yaddr = p_addr->yaddr;
			frame->uaddr = p_addr->uaddr;
			frame->vaddr = p_addr->vaddr;
			frame->yaddr_vir = p_addr->yaddr_vir;
			frame->uaddr_vir = p_addr->uaddr_vir;
			frame->vaddr_vir = p_addr->vaddr_vir;

			frame->pfinfo.dev = &s_dcam_pdev->dev;
			frame->pfinfo.mfd[0] = p_addr->mfd_y;
			frame->pfinfo.mfd[1] = p_addr->mfd_u;
			frame->pfinfo.mfd[2] = p_addr->mfd_u;
			/* may need update iommu here */
			rtn = pfiommu_get_sg_table(&frame->pfinfo);
			if (rtn) {
				pr_err("fail to cfg output addr\n");
				rtn = DCAM_RTN_PATH_ADDR_ERR;
				break;
			}
			/* need add raw path size info here */

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
	case DCAM_PDAF_EXTR:
		{ /* now set pdaf extr reg as constant */
			struct pdaf_extr_ctrl extr_ctrl = {
				3, 3};
			struct pdaf_skip_frm skip_frm = {
				1, 0, 1, 0};
			struct pdaf_extr_roi_start roi_start = {
				0, 0};
			struct pdaf_extr_roi_size roi_size = {
				1280, 720}; /* TBD */
			DCAM_REG_MWR(idx, DCAM0_PDAF_EXTR_CTRL,
				0x1E, /* bit 4,3,2,1 */
				(extr_ctrl.size_x << 1) |
				(extr_ctrl.size_y << 3));
			DCAM_REG_MWR(idx, DCAM0_PDAF_SKIP_FRM,
				0x7F, /* bit 6-0 */
				(skip_frm.skip_num << 3) |
				(skip_frm.mul_en << 2) |
				(skip_frm.sgl_start << 1) |
				(skip_frm.frm_mode));
			DCAM_REG_MWR(idx, DCAM0_PDAF_EXTR_ROI_ST,
				(0x1FFF << 13) | 0x1FFF,
				(roi_start.y << 13) | roi_start.x);
			DCAM_REG_MWR(idx, DCAM0_PDAF_EXTR_ROI_SIZE,
				(0x1FFF << 13) | 0x1FFF,
				(roi_size.h << 13) | roi_size.w);

		}
		break;
	default:
		pr_err("fail to get type\n");
		break;
	}

	return -rtn;
}

int set_dcam_aem_path_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_aem *aem_path = NULL;
	struct dcam_module *module;
	unsigned int endian = 0;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	if (0 == (unsigned long)(param)) {
		pr_err("fail to get parm null\n");
		return -DCAM_RTN_PARA_ERR;
	}

	module = s_p_dcam_mod[idx];
	aem_path = &module->dcam_aem;

	switch (id) {
	case DCAM_PATH_ENABLE:
		aem_path->valid = *(unsigned int *)param;
		break;
	case DCAM_PATH_DATA_ENDIAN:
		endian = *(unsigned int *)param;
		if (idx == 0) {
			sprd_dcam_glb_reg_mwr(idx, DCAM0_PATH_ENDIAN,
					      BIT_13 | BIT_12,
					      endian << 12,
					      DCAM_ENDIAN_REG);
		} else if (idx == 1) {
			sprd_dcam_glb_reg_mwr(idx, DCAM0_PATH_ENDIAN,
					      BIT_9 | BIT_8,
					      endian << 8,
					      DCAM_ENDIAN_REG);
		} else {
			pr_err("fail to set dcam%d endian\n", idx);
		}
		break;

	default:
		pr_err("fail to get type\n");
		break;
	}

	return -rtn;
}

int set_dcam_ebd_path_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_ebd *path = NULL;
	struct dcam_module *module;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	if (0 == (unsigned long)(param)) {
		pr_err("fail to get parm null\n");
		return -DCAM_RTN_PARA_ERR;
	}

	module = s_p_dcam_mod[idx];
	path = &module->dcam_ebd;

	switch (id) {
	case DCAM_PATH_ENABLE:
		path->valid = *(unsigned int *)param;
		break;
	case DCAM_EBD_CONTROL:
		memcpy(&path->ebd_ctrl, param,
		       sizeof(path->ebd_ctrl));
		break;

	default:
		pr_err("fail to get type\n");
		break;
	}

	return -rtn;
}

int dcam_aem_set_next_frm(void *statis_module,
	enum dcam_id idx, enum isp_3a_block_id block_index)
{
	enum dcam_drv_rtn ret = DCAM_RTN_SUCCESS;
	unsigned int statis_addr = 0;

	ret = dcam_set_next_statis_buf(statis_module,
		block_index, &statis_addr);

	pr_debug("phy_addr=%x, index=%d, ret=%d\n",
		statis_addr, block_index, ret);

	if (likely(!ret)) {
		DCAM_REG_WR(idx, DCAM0_AEM_BASE_WADDR, statis_addr);
		DCAM_REG_MWR(idx, DCAM0_AEM_CFG_READY, BIT_0, 1);
	}

	return ret;
}

static int dcam_full_path_start(enum dcam_id idx,
				struct dcam_path_desc *path,
				struct camera_frame *frame)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct camera_rect *rect = NULL;
	unsigned int reg_val = 0;
	unsigned long addr = 0;
	unsigned long image_vc = 0;
	unsigned long image_data_type = 0x2b;
	unsigned long image_mode = 1;

	if (path->valid_param.frame_deci) {
		sprd_dcam_glb_reg_mwr(idx, DCAM0_MIPI_CAP_FRM_CTRL,
			BIT_5 | BIT_4,
			path->frame_deci << 4,
			DCAM_REG_MAX);
	}

	if (path->valid_param.data_endian) {
		sprd_dcam_glb_reg_mwr(idx, DCAM0_PATH_ENDIAN,
			BIT_3 | BIT_2,
			path->data_endian.y_endian << 2,
			DCAM_ENDIAN_REG);
		sprd_dcam_glb_reg_mwr(idx, DCAM_AXIM_WORD_ENDIAN,
			BIT_0, 0,
			DCAM_AXI_ENDIAN_REG);
		DCAM_TRACE("full_path: data_endian y=0x%x\n",
			   path->data_endian.y_endian);
	}
	/* full path crop */
	if (path->valid_param.input_rect) {
		DCAM_REG_MWR(idx, DCAM0_FULL_CFG, BIT(1), BIT(1));
		pr_info("full path crop enable\n");

		addr = DCAM0_FULL_CROP_START;
		rect = &path->input_rect;
		reg_val = (rect->x & 0xFFFF) |
			((rect->y & 0xFFFF) << 16);
		DCAM_REG_WR(idx, addr, reg_val);

		addr = DCAM0_FULL_CROP_SIZE;
		reg_val = (rect->w & 0xFFFF) |
			((rect->h & 0xFFFF) << 16);
		DCAM_REG_WR(idx, addr, reg_val);
		DCAM_TRACE("full path crop: x %d, y %d, w %d h %d\n",
			rect->x, rect->y, rect->w, rect->h);
	}

	if (s_p_dcam_mod[idx]->dcam_cap.input_format ==
	    DCAM_CAP_MODE_YUV)
		image_data_type = 0x1e;
	/* setup data type */
	reg_val = ((image_vc & 0x3) << 16)  |
		((image_data_type & 0x3F) << 8) | (image_mode & 0x3);

	if (idx == DCAM_ID_0)
		addr = DCAM0_IMAGE_DT_VC_CONTROL;
	else if (idx == DCAM_ID_1)
		addr = DCAM1_IMAGE_DT_VC_CONTROL;
	DCAM_REG_WR(idx, addr, reg_val);

	rtn = dcam_full_path_set_next_frm(idx, frame+1);
	if (rtn) {
		pr_err("fail to set full next frm rtn %d, %x\n",
			rtn, (frame+1)->yaddr_vir);
		return -(rtn);
	}

	path->need_wait = 0;
	path->status = DCAM_ST_START;
	path->sof_cnt = 0;
	path->done_cnt = 0;
	sprd_dcam_glb_reg_owr(idx, DCAM0_CFG, BIT_0, DCAM_CFG_REG);

	return -rtn;
}

static int dcam_bin_path_start(enum dcam_id idx,
				struct dcam_path_desc *path,
				struct camera_frame *frame)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct camera_rect *rect = NULL;
	unsigned int reg_val = 0;
	unsigned long addr = 0;
	unsigned long image_vc = 0;
	unsigned long image_data_type = 0x2b;
	unsigned long image_mode = 1;

	/*
	 * don't set reg, deci in ISP
	if (binning_path->valid_param.frame_deci) {
		sprd_dcam_glb_reg_mwr(idx, DCAM0_MIPI_CAP_FRM_CTRL,
			BIT_5 | BIT_4,
			binning_path->frame_deci << 4,
			DCAM_REG_MAX);
	}
	*/

	if (path->valid_param.data_endian) {
		sprd_dcam_glb_reg_mwr(idx, DCAM0_PATH_ENDIAN,
			BIT_3 | BIT_2,
			path->data_endian.y_endian << 2,
			DCAM_ENDIAN_REG);
		sprd_dcam_glb_reg_mwr(idx, DCAM_AXIM_WORD_ENDIAN,
				      BIT_0, 0,
				      DCAM_AXI_ENDIAN_REG);
		DCAM_TRACE("binning_path: data_endian y=0x%x\n",
			   path->data_endian.y_endian);
	}

	/* bin path crop rect */
	if (path->valid_param.input_rect) {
		/* crop: binning or bypass
		 * first crop, then binning
		 * crop size align[x,y,w,h:2,2,4,2],bypass,1/2,1/4
		 * Attention: if package data is not align uint32,
		 * the image data dump by trace32 can't be show by
		 * tools.Because add some zero data. ISP can ignore it.
		 */
		rect = &path->input_rect;
		DCAM_REG_MWR(idx, DCAM0_CAM_BIN_CFG, BIT(1), BIT(1));
		pr_info("bin path crop enable\n");

		addr = DCAM0_BIN_CROP_START;
		/* crop align start:2,size:4:2 */
		rect->x = DCAM_SIZE_ALIGN(rect->x, 1);
		rect->y = DCAM_SIZE_ALIGN(rect->y, 1);
		rect->w = DCAM_SIZE_ALIGN(rect->w, 2);
		rect->h = DCAM_SIZE_ALIGN(rect->h, 1);

		reg_val = rect->x | (rect->y << 16);
		DCAM_REG_WR(idx, addr, reg_val);

		addr = DCAM0_BIN_CROP_SIZE;
		reg_val = rect->w | (rect->h << 16);
		DCAM_REG_WR(idx, addr, reg_val);
		DCAM_TRACE("bin crop:x %d, y %d, w %d, h %d\n",
			rect->x, rect->y, rect->w, rect->h);
	}

	if (s_p_dcam_mod[idx]->dcam_cap.input_format ==
		DCAM_CAP_MODE_YUV)
		image_data_type = 0x1e;
	/* setup data type */
	reg_val = ((image_vc & 0x3) << 16)  |
		((image_data_type & 0x3F) << 8) | (image_mode & 0x3);

	if (idx == DCAM_ID_0)
		addr = DCAM0_IMAGE_DT_VC_CONTROL;
	else if (idx == DCAM_ID_1)
		addr = DCAM1_IMAGE_DT_VC_CONTROL;
	DCAM_REG_WR(idx, addr, reg_val);
	rtn = dcam_bin_path_set_next_frm(idx, frame);
	if (rtn) {
		pr_err("fail to set bin next frm rtn%d,%x",
			rtn, frame->yaddr_vir);
		return -(rtn);
	}
	path->need_wait = 0;
	path->status = DCAM_ST_START;
	path->sof_cnt = 0;
	path->done_cnt = 0;
	sprd_dcam_glb_reg_owr(idx, DCAM0_CFG, BIT_1, DCAM_CFG_REG);

	return -rtn;
}

static int dcam_full_raw_path_start(enum dcam_id idx,
				struct dcam_path_desc *path)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct camera_rect *rect = NULL;
	unsigned int reg_val = 0;
	unsigned long addr = 0;
	unsigned long image_vc = 0;
	unsigned long image_data_type = 0x2b;
	unsigned long image_mode = 1;

	if (path->valid_param.frame_deci) {
		sprd_dcam_glb_reg_mwr(idx, DCAM0_MIPI_CAP_FRM_CTRL,
			BIT_5 | BIT_4,
			path->frame_deci << 4,
			DCAM_REG_MAX);
	}

	if (path->valid_param.data_endian) {
		sprd_dcam_glb_reg_mwr(idx, DCAM0_PATH_ENDIAN,
			BIT_3 | BIT_2,
			path->data_endian.y_endian << 2,
			DCAM_ENDIAN_REG);
		sprd_dcam_glb_reg_mwr(idx, DCAM_AXIM_WORD_ENDIAN,
			BIT_0, 0,
			DCAM_AXI_ENDIAN_REG);
		DCAM_TRACE("raw_path: data_endian y=0x%x\n",
			path->data_endian.y_endian);
	}
	/* raw path crop */
	if (path->valid_param.input_rect) {
		DCAM_REG_MWR(idx, DCAM0_FULL_CFG, BIT(1), BIT(1));
		pr_info("raw path crop enable\n");

		addr = DCAM0_FULL_CROP_START;
		rect = &path->input_rect;
		reg_val = (rect->x & 0xFFFF) |
			((rect->y & 0xFFFF) << 16);
		DCAM_REG_WR(idx, addr, reg_val);

		addr = DCAM0_FULL_CROP_SIZE;
		reg_val = (rect->w & 0xFFFF) |
			((rect->h & 0xFFFF) << 16);
		DCAM_REG_WR(idx, addr, reg_val);
		DCAM_TRACE("full path crop: x %d, y %d, w %d h %d\n",
			rect->x, rect->y, rect->w, rect->h);
	}


	/* setup data type raw10 */
	reg_val = ((image_vc & 0x3) << 16)  |
		((image_data_type & 0x3F) << 8) | (image_mode & 0x3);
	if (idx == DCAM_ID_0)
		addr = DCAM0_IMAGE_DT_VC_CONTROL;
	else if (idx == DCAM_ID_1)
		addr = DCAM1_IMAGE_DT_VC_CONTROL;
	DCAM_REG_WR(idx, addr, reg_val);
#if 0
	rtn = dcam_full_path_set_next_frm(idx);
	if (rtn) {
		pr_err("fail to set frm err, code %d", rtn);
		return -(rtn);
	}
#endif
	path->need_wait = 0;
	path->status = DCAM_ST_START;
	path->sof_cnt = 0;
	path->done_cnt = 0;
	sprd_dcam_glb_reg_owr(idx, DCAM0_CFG, BIT_0, DCAM_CFG_REG);

	rtn = dcam_raw_path_set_next_frm(idx);
	if (rtn) {
		pr_err("fail to set raw path next frm %d\n", rtn);
		return -(rtn);
	}

	path->need_wait = 0;
	path->status = DCAM_ST_START;
	path->sof_cnt = 0;
	path->done_cnt = 0;
	sprd_dcam_glb_reg_owr(idx, DCAM0_CFG, BIT_0, DCAM_CFG_REG);

	return -rtn;
}

static int dcam_bin_raw_path_start(enum dcam_id idx,
				struct dcam_path_desc *path)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct camera_rect *rect = NULL;
	unsigned int reg_val = 0;
	unsigned long addr = 0;
	unsigned long image_vc = 0;
	unsigned long image_data_type = 0x2b;
	unsigned long image_mode = 1;

	if (path->valid_param.frame_deci) {
		sprd_dcam_glb_reg_mwr(idx, DCAM0_MIPI_CAP_FRM_CTRL,
			BIT_5 | BIT_4,
			path->frame_deci << 4,
			DCAM_REG_MAX);
	}

	if (path->valid_param.data_endian) {
		sprd_dcam_glb_reg_mwr(idx, DCAM0_PATH_ENDIAN,
			BIT_3 | BIT_2,
			path->data_endian.y_endian << 2,
			DCAM_ENDIAN_REG);
		sprd_dcam_glb_reg_mwr(idx, DCAM_AXIM_WORD_ENDIAN,
			BIT_0, 0,
			DCAM_AXI_ENDIAN_REG);
		DCAM_TRACE("raw_path: data_endian y=0x%x\n",
			path->data_endian.y_endian);
	}
	/* raw path crop */
	if (path->valid_param.input_rect) {
		/* crop: binning or bypass,
		 * first crop, then binning
		 * Attention: if package data is not align uint32,
		 * the image data dump by trace32 can't be show by
		 * tools.Because add some zero data. ISP can ignore it.
		 */
		rect = &path->input_rect;
		DCAM_REG_MWR(idx, DCAM0_CAM_BIN_CFG, BIT(1), 1);

		pr_info("raw path crop enable\n");

		addr = DCAM0_BIN_CROP_START;
		rect->x = DCAM_SIZE_ALIGN(rect->x, 1);
		rect->y = DCAM_SIZE_ALIGN(rect->y, 1);
		rect->w = DCAM_SIZE_ALIGN(rect->w, 2);
		rect->h = DCAM_SIZE_ALIGN(rect->h, 1);

		reg_val = rect->x | (rect->y << 16);
		DCAM_REG_WR(idx, addr, reg_val);

		addr = DCAM0_BIN_CROP_SIZE;
		reg_val = (rect->w & 0xFFFF) |
		((rect->h & 0xFFFF) << 16);
		DCAM_REG_WR(idx, addr, reg_val);
		DCAM_TRACE("raw bin path crop: x %d, y %d, w %d h %d\n",
			rect->x, rect->y, rect->w, rect->h);
	}

	/* setup data type raw10 */
	reg_val = ((image_vc & 0x3) << 16)  |
		((image_data_type & 0x3F) << 8) | (image_mode & 0x3);
	if (idx == DCAM_ID_0)
		addr = DCAM0_IMAGE_DT_VC_CONTROL;
	else if (idx == DCAM_ID_1)
		addr = DCAM1_IMAGE_DT_VC_CONTROL;
	DCAM_REG_WR(idx, addr, reg_val);

	path->need_wait = 0;
	path->status = DCAM_ST_START;
	path->sof_cnt = 0;
	path->done_cnt = 0;
	sprd_dcam_glb_reg_owr(idx, DCAM0_CFG, BIT_1, DCAM_CFG_REG);

	rtn = dcam_raw_path_set_next_frm(idx);
	if (rtn) {
		pr_err("fail to set raw path next frm %d\n", rtn);
		return -(rtn);
	}

	path->need_wait = 0;
	path->status = DCAM_ST_START;
	path->sof_cnt = 0;
	path->done_cnt = 0;
	sprd_dcam_glb_reg_owr(idx, DCAM0_CFG, BIT_1, DCAM_CFG_REG);

	return -rtn;
}

static int dcam_pdaf_path_start(enum dcam_id idx,
				struct dcam_path_pdaf *path,
				void *statis_module)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;

	sprd_dcam_glb_reg_mwr(idx, DCAM0_PDAF_CONTROL,
			      BIT_1 | BIT_0,
			      path->pdaf_ctrl.mode << 0,
			      DCAM_REG_MAX);
	/* channel */
	sprd_dcam_glb_reg_mwr(idx, DCAM0_PDAF_CONTROL,
			      BIT(17) | BIT(16),
			      path->pdaf_ctrl.image_vc << 16,
			      DCAM_REG_MAX);
	/* data type */
	sprd_dcam_glb_reg_mwr(idx, DCAM0_PDAF_CONTROL,
			      BIT(13) | BIT(12) | BIT(11) |
			      BIT(10) | BIT(9) | BIT(8),
			      path->pdaf_ctrl.phase_data_type << 8,
			      DCAM_REG_MAX);

	rtn = dcam_pdaf_set_next_frm(statis_module,
				     idx, DCAM_PDAF_BLOCK);
	if (rtn) {
		pr_err("fail to set pdaf next frm %d\n", rtn);
		return -(rtn);
	}

	sprd_dcam_glb_reg_owr(idx, DCAM0_CFG, BIT_3, DCAM_CFG_REG);
	sprd_dcam_glb_reg_owr(idx, DCAM0_CFG, BIT_4, DCAM_CFG_REG);
	sprd_dcam_glb_reg_mwr(idx, DCAM0_CONTROL,
			      BIT(14) | BIT(16),
			      BIT(14) | BIT(16), DCAM_CONTROL_REG);

	return -rtn;
}

static int dcam_vch2_path_stop(enum dcam_id idx)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;

	sprd_dcam_glb_reg_mwr(idx, DCAM0_VC2_CONTROL,
			      BIT_1 | BIT_0,
			      0 << 0,
			      DCAM_REG_MAX);
	/* channel */
	sprd_dcam_glb_reg_mwr(idx, DCAM0_VC2_CONTROL,
			      BIT(17) | BIT(16),
			      0 << 16,
			      DCAM_REG_MAX);
	/* data type */
	sprd_dcam_glb_reg_mwr(idx, DCAM0_VC2_CONTROL,
			      BIT(13) | BIT(12) | BIT(11) |
			      BIT(10) | BIT(9) | BIT(8),
			      0 << 8,
			      DCAM_REG_MAX);

	sprd_dcam_glb_reg_mwr(idx, DCAM0_CFG,
			      BIT(4),
			      0 << 4,
			      DCAM_CFG_REG);

	return -rtn;
}

static int dcam_vch3_path_start(enum dcam_id idx,
				void *statis_module)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;


	rtn = dcam_ebd_set_next_frm(statis_module,
				    idx, DCAM_EBD_BLOCK);
	if (rtn) {
		pr_err("fail to set embed line next frm %d\n", rtn);
		return -(rtn);
	}

	sprd_dcam_glb_reg_owr(idx, DCAM0_CFG, BIT_5, DCAM_CFG_REG);
	sprd_dcam_glb_reg_mwr(idx, DCAM0_CONTROL,
			      BIT(18),
			      BIT(18), DCAM_CONTROL_REG);

	return -rtn;
}

static int dcam_aem_path_start(enum dcam_id idx, void *statis_module)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;

	rtn = dcam_aem_set_next_frm(statis_module, idx, DCAM_AEM_BLOCK);
	if (unlikely(rtn)) {
		pr_err("fail to set aem next frm %d\n", rtn);
		return -(rtn);
	}
	sprd_dcam_glb_reg_owr(idx, DCAM0_CFG, BIT_2, DCAM_CFG_REG);

	return -rtn;
}

int sprd_dcam_start(enum dcam_id idx, struct camera_frame *frame,
			void *statis_module)
{
	enum dcam_drv_rtn rtn = DCAM_RTN_SUCCESS;
	struct dcam_path_desc *raw_path = NULL;
	struct dcam_path_desc *full_path = NULL;
	struct dcam_path_desc *binning_path = NULL;
	struct dcam_path_pdaf *pdaf_path = NULL;
	struct dcam_path_aem *aem_path = NULL;
	struct dcam_path_ebd *ebd_path = NULL;
	unsigned int cap_en = 0;
	unsigned int reg_val = 0;

	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	DCAM_TRACE("DCAM%d: dcam start %x\n", idx,
		   s_p_dcam_mod[idx]->dcam_cap.cap_mode);

	full_path = &s_p_dcam_mod[idx]->dcam_full_path;
	binning_path = &s_p_dcam_mod[idx]->dcam_binning_path;
	raw_path = &s_p_dcam_mod[idx]->dcam_raw_path;
	pdaf_path = &s_p_dcam_mod[idx]->dcam_pdaf;
	aem_path = &s_p_dcam_mod[idx]->dcam_aem;
	ebd_path = &s_p_dcam_mod[idx]->dcam_ebd;

	/* Disable crop as default */
	DCAM_REG_MWR(idx, DCAM0_CAM_BIN_CFG, BIT(1), 0);
	DCAM_REG_MWR(idx, DCAM0_FULL_CFG, BIT(1), 0);
	if (full_path->valid) {
		rtn = dcam_full_path_start(idx, full_path, frame);
		if (rtn) {
			pr_err("fail to start full path %d\n", rtn);
			return -(rtn);
		}
	}
	if (binning_path->valid) {
		rtn = dcam_bin_path_start(idx, binning_path, frame);
		if (rtn) {
			pr_err("fail to start bin path %d\n", rtn);
			return -(rtn);
		}
	}
	if (raw_path->valid == FULL_RAW_CAPTURE) {
		rtn = dcam_full_raw_path_start(idx, raw_path);
		if (rtn) {
			pr_err("fail to start full raw path %d\n", rtn);
			return -(rtn);
		}
	}
	if (raw_path->valid == BIN_RAW_CAPTURE) {
		rtn = dcam_bin_raw_path_start(idx, raw_path);
		if (rtn) {
			pr_err("fail to start bin raw path %d\n", rtn);
			return -(rtn);
		}
	}
	if (pdaf_path->valid) {
		rtn = dcam_pdaf_path_start(idx, pdaf_path, statis_module);
		if (rtn) {
			pr_err("fail to start pdaf path %d\n", rtn);
			return -(rtn);
		}
	}
	if (ebd_path->valid) {
		if (pdaf_path->valid &&
		    pdaf_path->pdaf_ctrl.mode == E_PDAF_TYPE3) {
			rtn = dcam_vch3_path_start(idx, statis_module);
			if (rtn) {
				pr_err("fail to start vch3 path %d\n", rtn);
				return -(rtn);
			}
		} else {
			dcam_vch2_path_stop(idx);
			rtn = dcam_vch3_path_start(idx, statis_module);
			if (rtn) {
				pr_err("fail to start vch3 path %d\n", rtn);
				return -(rtn);
			}
		}
	}
	if (aem_path->valid) {
		rtn = dcam_aem_path_start(idx, statis_module);
		if (rtn) {
			pr_err("fail to start ebd path %d\n", rtn);
			return -(rtn);
		}
	}

	cap_en = DCAM_REG_RD(idx, DCAM0_CONTROL) & BIT_2;
	DCAM_TRACE("DCAM%d: cap_eb %d\n", idx, cap_en);

	if (cap_en == 0) {
		/* Cap force copy */
		dcam_force_copy(idx);
		/* Cap Enable */
		sprd_dcam_glb_reg_mwr(idx, DCAM0_CONTROL, BIT_2, 1 << 2,
				      DCAM_CONTROL_REG);
	}

	reg_val = (0xB << 21) | (0x3 << 16) | (0xE << 8) | 0xB;
	sprd_dcam_glb_reg_mwr(idx,
			      DCAM_AXIM_CTRL,
			      DCAM_AXIM_AQOS_MASK,
			      reg_val,
			      DCAM_AXI_STS_REG);
	reg_val = 0xE;
	sprd_dcam_glb_reg_mwr(idx,
			      DCAM_MMU_PT_UPDATE_QOS,
			      DCAM_MMU_PT_UPDATE_QOS_MASK,
			      reg_val,
			      DCAM_AXI_STS_REG);

	return -rtn;
}

int sprd_dcam_module_en(enum dcam_id idx)
{
	int ret = 0;

	pr_info("DCAM%d: enable dcam module, in %d\n", idx,
		atomic_read(&s_dcam_users[idx]));

	mutex_lock(&dcam_module_sema[idx]);
	if (atomic_inc_return(&s_dcam_users[idx]) == 1) {
		ret = sprd_cam_pw_on();
		if (ret != 0) {
			pr_err("fail to power on camera\n");
			mutex_unlock(&dcam_module_sema[idx]);
			return ret;
		}
		sprd_cam_domain_eb();
		dcam_enable_clk();
		sprd_dcam_reset(idx, NULL, NULL);

		ret = request_irq(s_dcam_irq[idx], dcam_isr_root,
				  IRQF_SHARED, "DCAM",
				  (void *)&s_dcam_irq[idx]);
		if (ret) {
			pr_err("fail to install IRQ %d\n", ret);
			mutex_unlock(&dcam_module_sema[idx]);
			return ret;
		}

		memset((void *)&s_user_func[idx][0], 0,
		       sizeof(void *) * DCAM_IRQ_NUMBER);
		memset((void *)&s_user_data[idx][0], 0,
		       sizeof(void *) * DCAM_IRQ_NUMBER);
		pr_info("DCAM%d: register isr, 0x%x\n", idx,
			DCAM_REG_RD(idx, DCAM0_INT_MASK));
	}
	mutex_unlock(&dcam_module_sema[idx]);

	pr_info("DCAM%d: enable dcam module, end %d\n", idx,
		atomic_read(&s_dcam_users[idx]));

	return ret;
}

int sprd_dcam_module_init(enum dcam_id idx, void *dev_handle)
{
	int ret = 0;

	if (atomic_read(&s_dcam_users[idx]) < 1) {
		pr_info("s_dcam_users[%d] equal to %d", idx,
		       atomic_read(&s_dcam_users[idx]));
		return -EIO;
	}
	ret = dcam_internal_init(idx, dev_handle);

	return ret;
}

int sprd_dcam_module_deinit(enum dcam_id idx)
{
	dcam_internal_deinit(idx);

	/* release all dmabufs only after all bufs are h/w unmapped */
	if (s_p_dcam_mod[DCAM_ID_0] == NULL
		&& s_p_dcam_mod[DCAM_ID_1] == NULL) {
		pr_info("Now put all dmabufs at dcam %d\n", idx);
		pfiommu_put_sg_table();
	}

	return -DCAM_RTN_SUCCESS;
}

int sprd_dcam_drv_init(struct platform_device *pdev)
{
	int i = 0;

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
		dcam_glb_reg_axi_sts_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_axi_sts_lock);
		dcam_glb_reg_endian_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_endian_lock);
		dcam_glb_reg_path_stop_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_path_stop_lock);
	}
	dcam_glb_reg_axi_endian_lock =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_axi_endian_lock);
	return 0;
}

void sprd_dcam_drv_deinit(void)
{
	int i;

	for (i = 0; i < s_dcam_count; i++) {
		atomic_set(&s_dcam_users[i], 0);
		s_p_dcam_mod[i] = NULL;
		s_dcam_irq[i] = 0;

		mutex_destroy(&dcam_module_sema[i]);

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
		dcam_glb_reg_axi_sts_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_axi_sts_lock);
		dcam_glb_reg_endian_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_endian_lock);
		dcam_glb_reg_path_stop_lock[i] =
			__SPIN_LOCK_UNLOCKED(dcam_glb_reg_path_stop_lock);
	}
	dcam_glb_reg_axi_endian_lock =
		__SPIN_LOCK_UNLOCKED(dcam_glb_reg_axi_endian_lock);
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
			pr_err("fail to power off camera\n");
			mutex_unlock(&dcam_module_sema[idx]);
			return rtn;
		}

		free_irq(s_dcam_irq[idx], (void *)&s_dcam_irq[idx]);
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
	capacity->path_info[CAMERA_RAW_PATH].line_buf = 0;
	capacity->path_info[CAMERA_RAW_PATH].support_yuv = 0;
	capacity->path_info[CAMERA_RAW_PATH].support_raw = 1;
	capacity->path_info[CAMERA_RAW_PATH].support_jpeg = 1;
	capacity->path_info[CAMERA_RAW_PATH].support_scaling = 0;
	capacity->path_info[CAMERA_RAW_PATH].support_trim = 0;
	capacity->path_info[CAMERA_RAW_PATH].is_scaleing_path = 0;

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

int sprd_dcam_parse_dt(struct device_node *dn, unsigned int *dcam_count)
{
	int i = 0;
	unsigned int count = 0;
	void __iomem *reg_base;
	int j = 0;

	if (!dn || !dcam_count) {
		pr_err("fail to get ptr dn %p dcam_count %p\n",
		       dn, dcam_count);
		return -EINVAL;
	}

	pr_info("Start dcam dts parse\n");
	if (of_property_read_u32_index(dn, "sprd,dcam-count", 0, &count)) {
		pr_err("fail to parse the property of sprd,dcam-count\n");
		return -EINVAL;
	}

	s_dcam_count = count;
	*dcam_count = count;
	dcam0_mm_eb = of_clk_get_by_name(dn, "clk_mm_eb");
	if (IS_ERR(dcam0_mm_eb)) {
		pr_err("fail to get dcam0_mm_eb\n");
		return PTR_ERR(dcam0_mm_eb);
	}

	dcam0_eb = of_clk_get_by_name(dn, "dcam_eb");
	if (IS_ERR(dcam0_eb)) {
		pr_err("fail to get dcam0_eb\n");
		return PTR_ERR(dcam0_eb);
	}

	dcam0_clk = of_clk_get_by_name(dn, "dcam_clk");
	if (IS_ERR(dcam0_clk)) {
		pr_err("fail to get dcam0_clk\n");
		return PTR_ERR(dcam0_clk);
	}

	dcam0_clk_parent = of_clk_get_by_name(dn, "dcam_clk_parent");
	if (IS_ERR(dcam0_clk_parent)) {
		pr_err("fail to get dcam0_clk_parent\n");
		return PTR_ERR(dcam0_clk_parent);
	}

	dcam0_clk_default = clk_get_parent(dcam0_clk);
	if (IS_ERR(dcam0_clk_default)) {
		pr_err("fail to get dcam0_clk_default\n");
		return PTR_ERR(dcam0_clk_default);
	}

	cam_ahb_gpr = syscon_regmap_lookup_by_phandle(dn,
		"sprd,cam-ahb-syscon");
	if (IS_ERR(cam_ahb_gpr))
		return PTR_ERR(cam_ahb_gpr);
/*
 * TBD
	if (count == DCAM_MAX_COUNT) {
		dcam1_mm_eb = of_clk_get_by_name(dn, "clk_mm_eb1");
		if (IS_ERR(dcam1_mm_eb)) {
			pr_err("fail to get dcam1_mm_eb\n");
			return PTR_ERR(dcam1_mm_eb);
		}

		dcam1_eb = of_clk_get_by_name(dn, "dcam1_eb");
		if (IS_ERR(dcam1_eb)) {
			pr_err("fail to get dcam0_eb\n");
			return PTR_ERR(dcam1_eb);
		}
		// Add more if support dual camera
	}
*/
	for (i = 0; i < count; i++) {
		s_dcam_irq[i] = irq_of_parse_and_map(dn, i);
		if (s_dcam_irq[i] <= 0) {
			pr_err("fail to get dcam irq %d\n", i);
			return -EFAULT;
		}

		reg_base = of_iomap(dn, i);
		if (!reg_base) {
			pr_err("fail to get dcam reg_base %d\n", i);
			return -ENXIO;
		}
		s_dcam_regbase[i] = (unsigned long)reg_base;

		pr_info("Dcam dts OK! base %lx, irq %d\n", s_dcam_regbase[i],
			s_dcam_irq[i]);
	}
	/* dcam axi */
	i = 2;
	reg_base = of_iomap(dn, i);
	if (!reg_base) {
		pr_err("fail to get dcam axi base\n");
		return -ENXIO;
	}
	s_dcam_axi_base = (unsigned long)reg_base;

	/* skip axi[0x2000], [0x3000] */
	i += 2;

	/* additional I/O memory for LSC buffer */
	for (j = 0; j < count; j++) {
		reg_base = of_iomap(dn, i);
		i++;
		if (!reg_base) {
			pr_err("fail to get dcam LSC buffer base %d\n", j);
			return -ENXIO;
		}

		s_lsc_grid_base[j] = (unsigned long)reg_base;

		pr_info("lsc base %lx\n", s_lsc_grid_base[j]);
	}

	return 0;
}

unsigned int cal_bin_ratio(struct camera_size src,
			struct camera_size dst)
{
/* less than this width(<2592), bypass binning */
#define	_WIDTH_NO_BINNING	1600
#define _MAX_SCALER_DOWN	16
#define _WIDTH_NO_1_4_BINNING	176
	unsigned int tmp_ratio = 0;
	struct camera_size size;

	/* as normal, binning ratio: 0,1,2,3*/
	if (src.w == 0 || src.h == 0 || dst.w == 0 ||
		dst.h == 0)
		return 0;

	size.w = src.w >> 1;
	size.h = src.h >> 1;
	while (tmp_ratio < 4 && size.w >= dst.w &&
		size.h >= dst.h) {
		tmp_ratio++;
		size.w >>= 1;
		size.h >>= 1;
	}
	pr_debug("bin ratio %d\n", tmp_ratio);

	/* try to reduce binning ratio */
	if (tmp_ratio > 0 && tmp_ratio < 4) {
		size.w = src.w >> (tmp_ratio - 1);
		size.h = src.h >> (tmp_ratio - 1);
		if (size.w <= ISP_PATH1_LINE_BUF_LENGTH)
			tmp_ratio--;
	}

	/* if src.w > _BUF_LENGTH, do some binning */
	size.w = src.w >> tmp_ratio;
	while (size.w > ISP_PATH1_LINE_BUF_LENGTH) {
		tmp_ratio++;
		size.w = src.w >> tmp_ratio;
	}

	/* for feature phone(640x480), no binning,
	 * src.w <= 1600 && src / dst <= 16
	 */
	if (src.w <= _WIDTH_NO_BINNING &&
		src.w <= (dst.w * _MAX_SCALER_DOWN) &&
		src.h <= (dst.h * _MAX_SCALER_DOWN)) {
		tmp_ratio = 0;
		pr_debug("bypass binning, w:%d %d\n", src.w, src.h);
	}

	/* for QCIF, no dcam 1/4 binning because it may result in
	 * overall 1/8 binning, that may not be supported by any
	 * tuning parameters
	 */
	if (dst.w <= _WIDTH_NO_1_4_BINNING && tmp_ratio == 2) {
		tmp_ratio = 1;
		pr_warning("no 1/4 binning, (%d, %d) -> (%d, %d)\n",
			src.w, src.h, dst.w, dst.h);
	}

	/* check binning ratio */
	if (tmp_ratio > 2)
		tmp_ratio = 2;
	pr_debug("cal bin ratio %d, src %d %d dst %d %d\n",
		tmp_ratio, src.w, src.h, dst.w, dst.h);

	return tmp_ratio;
}

/* aligned bin path input size with bin ratio
 * r: bin_ration
 */
void align_for_bin(struct camera_size *p, unsigned int r)
{
	if ((!p) || (!r))
		return;
	if (r == 1) {
		/* binning need align w = 8 or h = 4 */
		p->w = DCAM_SIZE_ALIGN(p->w, 3);
		p->h = DCAM_SIZE_ALIGN(p->h, 2);
	} else {
		/* binning need align w = 16 or h = 8 */
		p->w = DCAM_SIZE_ALIGN(p->w, 4);
		p->h = DCAM_SIZE_ALIGN(p->h, 3);
	}
}

/* get bin_ratio from path(struct dcam_path_desc)
 * Only binning path has this. Other path return 0
 */
static struct dcam_path_desc *get_bin_ratio(enum dcam_id idx,
				unsigned int channel_id)
{
	if (DCAM_ADDR_INVALID(s_p_dcam_mod[idx])) {
		pr_err("fail to get ptr\n");
		return NULL;
	}
	/* only bin path need update trime size
	 * preview, video use bin path
	 */
	if (channel_id == CAMERA_PRE_PATH ||
		channel_id == CAMERA_VID_PATH)
		return &s_p_dcam_mod[idx]->dcam_binning_path;

	return NULL;
}

/* refresh trim size (bin) for isp trim
 * 1: trim size different with different bin ratio
 * 2: isp trim0 need align[x,y,w,h~2:1:2:1]
 */
void refresh_bin_trim_size(struct camera_rect *p, unsigned int ratio)
{
	if (1 == ratio || 2 == ratio) {
		p->x >>= ratio;
		p->y >>= ratio;
		p->w >>= ratio;
		p->h >>= ratio;
	}
	p->x &= DCAM_SIZE_ALIGN(p->x, 1);
	p->w &= DCAM_SIZE_ALIGN(p->w, 1);
/*	y, h no need align
 *	p->y &= DCAM_SIZE_ALIGN(p->y, 1);
 *	p->h &= DCAM_SIZE_ALIGN(p->h, 1);
 */
}

void update_trim_size(enum dcam_id idx, struct camera_rect *p,
	unsigned int channel_id)
{
	unsigned int ratio;
	struct dcam_path_desc *bin_path;

	bin_path = get_bin_ratio(idx, channel_id);
	if (!bin_path) {
		/* If capture path enable, will to here, it's right */
		DCAM_TRACE("It's not bin path, channel_id %d\n", channel_id);
		return;
	}
	ratio = bin_path->bin_ratio;
	DCAM_TRACE("update crop size, bin ratio= %d\n", ratio);
	/* If Dcam crop enable for align
	 * isp trim0 size can't larger than the size of dcam crop
	 * use the larger one as crop
	 */
	if (bin_path->valid_param.input_rect) {
		if (p->w > bin_path->input_rect.w)
			p->w = bin_path->input_rect.w;
		if (p->h > bin_path->input_rect.h)
			p->h = bin_path->input_rect.h;
	}
	refresh_bin_trim_size(p, ratio);
}

/* log_frame_init
 * output log firt n frames per stream on
 */
void log_frame_init(void)
{
	log_frame.tx_done_bin = _LOG_FRAME_TIMES;
	log_frame.tx_done_full = _LOG_FRAME_TIMES;
}
