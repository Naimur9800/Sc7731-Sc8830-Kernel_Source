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

#include <video/sprd_img.h>
#include <video/sprd_mm.h>
#include <linux/sprd_ion.h>

#include "cam_pw_domain.h"
#include "cam_hw.h"
#include "cam_types.h"
#include "cam_queue.h"
#include "cam_buf.h"

#include "isp_int.h"
#include "isp_reg.h"

#include "isp_interface.h"
#include "isp_core.h"
#include "isp_path.h"
#include "isp_slice.h"

#include "isp_cfg.h"
#include "isp_fmcu.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_CORE: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__




enum  isp_context_id prefer_cxt_id[CH_PROP_MAX][ISP_CONTEXT_NUM] = {
	[CH_PROP_PRE][0] = ISP_CONTEXT_P0,
	[CH_PROP_PRE][1] = ISP_CONTEXT_P1,
	[CH_PROP_PRE][2] = ISP_CONTEXT_C0,
	[CH_PROP_PRE][3] = ISP_CONTEXT_C1,

	[CH_PROP_CAP][0] = ISP_CONTEXT_C0,
	[CH_PROP_CAP][1] = ISP_CONTEXT_C1,
	[CH_PROP_CAP][2] = ISP_CONTEXT_P0,
	[CH_PROP_CAP][3] = ISP_CONTEXT_P1,

	[CH_PROP_VIDEO][0] = ISP_CONTEXT_P0,
	[CH_PROP_VIDEO][1] = ISP_CONTEXT_P1,
	[CH_PROP_VIDEO][2] = ISP_CONTEXT_C0,
	[CH_PROP_VIDEO][3] = ISP_CONTEXT_C1,

	[CH_PROP_FD][0] = ISP_CONTEXT_P0,
	[CH_PROP_FD][1] = ISP_CONTEXT_P1,
	[CH_PROP_FD][2] = ISP_CONTEXT_C0,
	[CH_PROP_FD][3] = ISP_CONTEXT_C1,
};

enum isp_sub_path_id prefer_path_id[CH_PROP_MAX][ISP_SPATH_NUM] = {
	[CH_PROP_PRE][0] = ISP_SPATH_CP,
	[CH_PROP_PRE][1] = ISP_SPATH_VID,
	[CH_PROP_PRE][2] = ISP_SPATH_FD,

	[CH_PROP_CAP][0] = ISP_SPATH_CP,
	[CH_PROP_CAP][1] = ISP_SPATH_VID,
	[CH_PROP_CAP][2] = ISP_SPATH_FD,

	[CH_PROP_VIDEO][0] = ISP_SPATH_VID,
	[CH_PROP_VIDEO][1] = ISP_SPATH_CP,
	[CH_PROP_VIDEO][2] = ISP_SPATH_FD,

	[CH_PROP_FD][0] = ISP_SPATH_FD,
	[CH_PROP_FD][1] = ISP_SPATH_VID,
	[CH_PROP_FD][2] = ISP_SPATH_CP,
};

uint32_t line_buffer_len;
static uint32_t set_line_buffer_len;
static int s_work_mode = ISP_CFG_MODE;
unsigned long *isp_cfg_poll_addr[ISP_CONTEXT_MAX];


static DEFINE_MUTEX(isp_pipe_dev_mutex);
static struct isp_pipe_dev *s_isp_dev;


static int sprd_isp_put_path(void *isp_handle,
				uint32_t put_path_id);


/* debug fs starts */

#ifdef DBG_REGISTER
static uint32_t common_reg[ISP_CONTEXT_MAX][512];
static uint32_t debug_ctx_id[4] = {0, 1, 2, 3};

static int read_common_reg(uint32_t  ctx_id)
{
	uint32_t  *ptr = &common_reg[ctx_id][0];
	unsigned long addr;
	unsigned long int_base[4] = {
		ISP_P0_INT_BASE,
		ISP_C0_INT_BASE,
		ISP_P1_INT_BASE,
		ISP_C1_INT_BASE,
	};

	pr_info("ctx_id %d,  ptr %p\n", ctx_id, ptr);

	for (addr = int_base[ctx_id]; addr <= (int_base[ctx_id] + ISP_INT_ALL_DONE_SRC_CTRL); addr += 4)
		*ptr++ = ISP_HREG_RD(addr);

	for (addr = ISP_COMMON_VERSION; addr <= ISP_BLOCK_MODE; addr += 4)
		*ptr++ = ISP_HREG_RD(addr);

	for (addr = ISP_CFG_STATUS0; addr <= ISP_CFG_STATUS4; addr += 4)
		*ptr++ = ISP_HREG_RD(addr);

	for (addr = ISP_ARBITER_WR_STATUS; addr <= ISP_ARBITER_CHK_SUM0; addr += 4)
		*ptr++ = ISP_HREG_RD(addr);

	for (addr = ISP_AXI_WR_MASTER_STATUS; addr <= ISP_AXI_PARAM3; addr += 4)
		*ptr++ = ISP_HREG_RD(addr);

	for (addr = ISP_CFG0_BUF; addr <= ISP_CFG0_BUF + 220; addr += 4)
		*ptr++ = ISP_HREG_RD(addr);

	for (addr = ISP_CFG1_BUF; addr <= ISP_CFG1_BUF + 220; addr += 4)
		*ptr++ = ISP_HREG_RD(addr);

	addr = (unsigned long)ptr - (unsigned long)&common_reg[ctx_id][0];
	pr_info("read total num %d\n", (uint32_t)addr/4);

	return 0;
}


static int common_reg_show(struct seq_file *s, void *unused)
{
	uint32_t *ptr = (uint32_t *)s->private;
	uint32_t addr;

	pr_info("ptr %p\n",  ptr);

	seq_puts(s, "---------dump regsigters of ISP----------------\n");

	seq_puts(s, "=== INTERRUPT ====\n");
	for (addr = ISP_INT_STATUS; addr <= ISP_INT_ALL_DONE_SRC_CTRL; addr += 4)
		seq_printf(s, "%04x:  %08x\n", addr,  *ptr++);

	seq_puts(s, "===common====\n");
	for (addr = ISP_COMMON_VERSION; addr <= ISP_BLOCK_MODE; addr += 4)
		seq_printf(s, "%04x:  %08x\n", addr,  *ptr++);

	seq_puts(s, "=== CFG MODULE ====\n");
	for (addr = ISP_CFG_STATUS0; addr <= ISP_CFG_STATUS4; addr += 4)
		seq_printf(s, "%04x:  %08x\n", addr,  *ptr++);

	seq_puts(s, "=== ARBITER ====\n");
	for (addr = ISP_ARBITER_WR_STATUS; addr <= ISP_ARBITER_CHK_SUM0; addr += 4)
		seq_printf(s, "%04x:  %08x\n", addr,  *ptr++);

	seq_puts(s, "=== AXI ====\n");
	for (addr = ISP_AXI_WR_MASTER_STATUS; addr <= ISP_AXI_PARAM3; addr += 4)
		seq_printf(s, "%04x:  %08x\n", addr,  *ptr++);

	seq_puts(s, "=== CFG0 BUF ====\n");
	for (addr = ISP_CFG0_BUF; addr <= ISP_CFG0_BUF + 220; addr += 4)
		seq_printf(s, "%04x:  %08x\n", addr,  *ptr++);

	seq_puts(s, "=== CFG1 BUF ====\n");
	for (addr = ISP_CFG1_BUF; addr <= ISP_CFG1_BUF + 220; addr += 4)
		seq_printf(s, "%04x:  %08x\n", addr,  *ptr++);

	seq_puts(s, "----------------------------------------------------\n");


	return 0;
}

static int common_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, common_reg_show, inode->i_private);
}

static const struct file_operations common_reg_ops = {
	.owner =	THIS_MODULE,
	.open = common_reg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int reg_buf_show(struct seq_file *s, void *unused)
{
	debug_show_ctx_reg_buf((void *)s);
	return 0;
}

static int reg_buf_open(struct inode *inode, struct file *file)
{
	return single_open(file, reg_buf_show, inode->i_private);
}

static const struct file_operations reg_buf_ops = {
	.owner =	THIS_MODULE,
	.open = reg_buf_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#else
static int read_common_reg(uint32_t  ctx_id)
{
	return 0;
}
#endif


#define WORK_MODE_SLEN  2
#define LBUF_LEN_SLEN  8

static struct dentry *debugfs_base;

static ssize_t work_mode_show(
		struct file *filp, char __user *buffer,
		size_t count, loff_t *ppos)
{
	char buf[16];

	snprintf(buf, sizeof(buf), "%d\n", s_work_mode);

	return simple_read_from_buffer(
			buffer, count, ppos,
			buf, strlen(buf));
}

static ssize_t work_mode_write(
		struct file *filp, const char __user *buffer,
		size_t count, loff_t *ppos)
{
	int ret = 0;
	char msg[8];
	long val = 0L;

	if (count > WORK_MODE_SLEN)
		return -EINVAL;

	ret = copy_from_user(
			msg, (void __user *)buffer, count);
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	msg[WORK_MODE_SLEN-1] = '\0';
	ret = kstrtol(msg, 0, &val);
	if (val == 0L)
		s_work_mode = ISP_CFG_MODE;
	else if (val == 1L)
		s_work_mode = ISP_AP_MODE;
	else
		pr_err("error: invalid work mode: %d", (int)val);

	return count;
}

static const struct file_operations work_mode_ops = {
	.owner =	THIS_MODULE,
	.open = simple_open,
	.read = work_mode_show,
	.write = work_mode_write,
};

static ssize_t lbuf_len_show(
			struct file *filp, char __user *buffer,
			size_t count, loff_t *ppos)
{
	char buf[16];

	snprintf(buf, sizeof(buf), "%d\n", set_line_buffer_len);

	return simple_read_from_buffer(
			buffer, count, ppos,
			buf, strlen(buf));
}

static ssize_t lbuf_len_write(struct file *filp,
			const char __user *buffer,
			size_t count, loff_t *ppos)
{
	int ret = 0;
	char msg[8];
	long val;

	if (count > LBUF_LEN_SLEN)
		return -EINVAL;

	ret = copy_from_user(
			msg, (void __user *)buffer, count);
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	msg[LBUF_LEN_SLEN - 1] = '\0';
	ret = kstrtol(msg, 0, &val);
	set_line_buffer_len = (uint32_t)val;
	pr_info("set line buf len %d.  %lx, %s\n",
			set_line_buffer_len, val, msg);

	return count;
}

static const struct file_operations lbuf_len_ops = {
	.owner =	THIS_MODULE,
	.open = simple_open,
	.read = lbuf_len_show,
	.write = lbuf_len_write,
};

int sprd_isp_debugfs_init(void)
{
	char dirname[32] = {0};

	snprintf(dirname, sizeof(dirname), "sprd_isp");
	debugfs_base = debugfs_create_dir(dirname, NULL);
	if (!debugfs_base) {
		pr_err("fail to create debugfs dir\n");
		return -ENOMEM;
	}

	if (!debugfs_create_file("work_mode", S_IRUGO | S_IWUSR,
			debugfs_base, NULL, &work_mode_ops))
		return -ENOMEM;

	if (!debugfs_create_file("line_buf_len", S_IRUGO | S_IWUSR,
			debugfs_base, NULL, &lbuf_len_ops))
		return -ENOMEM;

#ifdef DBG_REGISTER
	if (!debugfs_create_file("pre0", S_IRUGO,
			debugfs_base, &common_reg[0][0], &common_reg_ops))
		return -ENOMEM;
	if (!debugfs_create_file("cap0", S_IRUGO,
			debugfs_base, &common_reg[1][0], &common_reg_ops))
		return -ENOMEM;
	if (!debugfs_create_file("pre1", S_IRUGO,
			debugfs_base, &common_reg[2][0], &common_reg_ops))
		return -ENOMEM;
	if (!debugfs_create_file("cap1", S_IRUGO,
			debugfs_base, &common_reg[3][0], &common_reg_ops))
		return -ENOMEM;

	if (!debugfs_create_file("pre0_buf", S_IRUGO,
			debugfs_base, &debug_ctx_id[0], &reg_buf_ops))
		return -ENOMEM;
	if (!debugfs_create_file("cap0_buf", S_IRUGO,
			debugfs_base, &debug_ctx_id[1], &reg_buf_ops))
		return -ENOMEM;
	if (!debugfs_create_file("pre1_buf", S_IRUGO,
			debugfs_base, &debug_ctx_id[2], &reg_buf_ops))
		return -ENOMEM;
	if (!debugfs_create_file("cap1_buf", S_IRUGO,
			debugfs_base, &debug_ctx_id[3], &reg_buf_ops))
		return -ENOMEM;

#endif

	return 0;
}
/* debug fs end */


void isp_ret_out_frame(void *param)
{
	struct camera_frame *frame;
	struct isp_pipe_context *pctx;
	struct isp_path_desc *path;

	if (!param) {
		pr_err("error: null input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	path = (struct isp_path_desc *)frame->priv_data;

	pr_debug("frame %p, ch_id %d, buf_fd %d\n",
		frame, frame->channel_id, frame->buf.mfd[0]);

	if (frame->is_reserved)
		camera_enqueue(
			&path->reserved_buf_queue, frame);
	else {
		pctx = path->attach_ctx;
		cambuf_iommu_unmap(&frame->buf);
		pctx->isp_cb_func(
			ISP_CB_RET_DST_BUF,
			frame, pctx->cb_priv_data);
	}
}

void isp_ret_src_frame(void *param)
{
	struct camera_frame *frame;
	struct isp_pipe_context *pctx;

	if (!param) {
		pr_err("error: null input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	pctx = (struct isp_pipe_context *)frame->priv_data;
	pr_debug("frame %p, ch_id %d, buf_fd %d\n",
		frame, frame->channel_id, frame->buf.mfd[0]);

	cambuf_iommu_unmap(&frame->buf);
	pctx->isp_cb_func(
		ISP_CB_RET_SRC_BUF,
		frame, pctx->cb_priv_data);
}

void isp_destroy_reserved_buf(void *param)
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

void isp_set_ctx_common(struct isp_pipe_context *pctx)
{
	uint32_t idx = pctx->ctx_id;
	uint32_t bypass = 0;
	struct isp_fetch_info *fetch = &pctx->fetch;

	pr_info("enter.\n");

	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL,
			BIT_1 | BIT_0, pctx->dispatch_color);
	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL,
			BIT_3 | BIT_2, 3 << 2);  /* 11b: close store_dbg module */
	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL, BIT_4, 0 << 4);

#ifdef TEST_SHARKL3
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_8,  0 << 8); /* 3dnr off */
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_7 | BIT_6, 3 << 6);  /* store out path off */
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_3 | BIT_2, 3 << 2);  /* vid path off */
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_1 | BIT_0, 3 << 0);  /* pre/cap path off */
#else
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_10, pctx->fetch_path_sel  << 10);
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_8, pctx->en_3dnr << 8); /* 3dnr off */
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_5 | BIT_4, 3 << 4);  /* thumb path off */
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_3 | BIT_2, 3 << 2); /* vid path off */
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_1 | BIT_0, 3 << 0);  /* pre/cap path off */
	if (pctx->fmcu_handle) {
		unsigned long reg_offset;
		struct isp_fmcu_ctx_desc *fmcu;

		fmcu = (struct isp_fmcu_ctx_desc *)pctx->fmcu_handle;
		reg_offset = (fmcu->fid == 0) ?
					ISP_COMMON_FMCU0_PATH_SEL :
					ISP_COMMON_FMCU1_PATH_SEL;
		ISP_HREG_MWR(reg_offset, BIT_1 | BIT_0, pctx->ctx_id);
	}
#endif

	ISP_REG_MWR(idx, ISP_FETCH_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_FETCH_PARAM,
			(0xF << 4), fetch->fetch_fmt << 4);
	ISP_REG_WR(idx, ISP_FETCH_MEM_SLICE_SIZE,
			fetch->size.w | (fetch->size.h << 16));

	ISP_REG_WR(idx, ISP_FETCH_SLICE_Y_PITCH, fetch->pitch.pitch_ch0);
	ISP_REG_WR(idx, ISP_FETCH_SLICE_U_PITCH, fetch->pitch.pitch_ch1);
	ISP_REG_WR(idx, ISP_FETCH_SLICE_V_PITCH, fetch->pitch.pitch_ch2);
	ISP_REG_WR(idx, ISP_FETCH_LINE_DLY_CTRL, 0x200);
	ISP_REG_WR(idx, ISP_FETCH_MIPI_INFO,
		fetch->mipi_word_num | (fetch->mipi_byte_rel_pos << 16));

	ISP_REG_WR(idx, ISP_DISPATCH_DLY,  0x1D3C);
#ifdef TEST_SHARKL3
	ISP_REG_WR(idx, ISP_DISPATCH_HW_CTRL_CH0,  0x080004);
#endif
	ISP_REG_WR(idx, ISP_DISPATCH_LINE_DLY1,  0x280001C);
	ISP_REG_WR(idx, ISP_DISPATCH_PIPE_BUF_CTRL_CH0,  0x64043C);
	ISP_REG_WR(idx, ISP_DISPATCH_CH0_SIZE,
		pctx->fetch.size.w | (pctx->fetch.size.h << 16));
	ISP_REG_WR(idx, ISP_DISPATCH_CH0_BAYER, pctx->dispatch_bayer_mode);

	/*CFA*/
	ISP_REG_MWR(idx, ISP_CFAE_NEW_CFG0, BIT_0, 0);
	ISP_REG_WR(idx, ISP_CFAE_INTP_CFG0, 0x1F4 | 0x1F4 << 16);
	ISP_REG_WR(idx, ISP_CFAE_INTP_CFG1,
		(0x1 << 31) | (0x14 << 12) | (0x7F << 4) | 0x4);
	ISP_REG_WR(idx, ISP_CFAE_INTP_CFG2, 0x8 | (0x0 << 8));
	ISP_REG_WR(idx, ISP_CFAE_INTP_CFG3,
		(0x8 << 20) | (0x8 << 12) | 0x118);
	ISP_REG_WR(idx, ISP_CFAE_INTP_CFG4, 0x64 | (0x64 << 16));
	ISP_REG_WR(idx, ISP_CFAE_INTP_CFG5, 0xC8 | (0xC8 << 16));

	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG0, 0x64 | (0x96 << 16));
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG1, 0x14 | (0x5 << 16));
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG2,
		(0x28 << 20) | (0x1E << 10) | 0xF);
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG3, 0xC8);
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG4, (0x5 << 10));
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG5, (0x50 << 9) | 0x78);
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG6,
		(0x32 << 18) | (0x32 << 9));
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG7, (0x64 << 18));
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG8, 0x3C | (0x8 << 17));
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG9,
		(0x1FC << 20) | (0x134 << 10) | 0x27C);
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG10,
		(0x214 << 20) | (0x1FF << 10) | 0x1CD);
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG11, 0x22D << 10 | 0x205);

	/*CCE*/
	ISP_REG_MWR(idx, ISP_CCE_PARAM, BIT_0, 0);
	ISP_REG_WR(idx, ISP_CCE_MATRIX0, (150 << 11) | 77);
	ISP_REG_WR(idx, ISP_CCE_MATRIX1, ((-43) << 11) | 29);
	ISP_REG_WR(idx, ISP_CCE_MATRIX2, 0x407AB);
	ISP_REG_WR(idx, ISP_CCE_MATRIX3, ((-107) << 11) | 128);
	ISP_REG_WR(idx, ISP_CCE_MATRIX4, (-21));
	ISP_REG_WR(idx, ISP_CCE_SHIFT, 0);

	ISP_REG_WR(idx, ISP_YDELAY_STEP, 0x144);
	ISP_REG_WR(idx, ISP_SCALER_PRE_CAP_BASE
		+ ISP_SCALER_HBLANK, 0x4040);
	ISP_REG_WR(idx, ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_RES, 0xFF);
	ISP_REG_WR(idx, ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_DEBUG, 1);

	ISP_REG_MWR(idx, ISP_STORE_DEBUG_BASE + ISP_STORE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_STORE_PRE_CAP_BASE + ISP_STORE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_STORE_VID_BASE + ISP_STORE_PARAM, BIT_0, 1);

	pr_info("end\n");
}

#ifdef TEST_SHARKL3
void isp_set_ctx_default(struct isp_pipe_context *pctx)
{
	uint32_t idx = pctx->ctx_id;
	uint32_t bypass = 1;

	ISP_REG_MWR(idx, ISP_STORE_DEBUG_BASE + ISP_STORE_PARAM, BIT_0, bypass);

	/* default bypass all blocks */
	ISP_REG_MWR(idx, ISP_NLM_PARA, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_VST_PARA, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_IVST_PARA, BIT_0, bypass);

	ISP_REG_MWR(idx, ISP_CMC10_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_GAMMA_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_PSTRZ_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_CCE_PARAM, BIT_0, bypass);

	ISP_REG_MWR(idx, ISP_UVD_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0, BIT_0|BIT_1, 0x3);
	ISP_REG_MWR(idx, ISP_BRIGHT_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_CONTRAST_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST_CFG_READY, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST2_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST2_CFG_RDY, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_CDN_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_EE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_CSA_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HUA_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_POSTCDN_COMMON_CTRL, BIT_0|BIT_1, 0x3);
	ISP_REG_MWR(idx, ISP_YGAMMA_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_IIRCNR_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM1, BIT_0, 1);

	/* 3DNR bypass */
	ISP_REG_MWR(idx, ISP_3DNR_MEM_CTRL_PARAM0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_NR3_BLEND_CONTROL0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_STORE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_MEM_CTRL_PRE_PARAM0, BIT_0, 1);

	pr_info("end\n");
}
#else
void isp_set_ctx_default(struct isp_pipe_context *pctx)
{
	uint32_t idx = pctx->ctx_id;
	uint32_t bypass = 1;

	ISP_REG_MWR(idx, ISP_STORE_DEBUG_BASE + ISP_STORE_PARAM, BIT_0, bypass);

	/* default bypass all blocks */
	ISP_REG_MWR(idx, ISP_NLM_PARA, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_VST_PARA, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_IVST_PARA, BIT_0, bypass);
	/* 3DNR bypass */
	ISP_REG_MWR(idx, ISP_3DNR_MEM_CTRL_PARAM0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_NR3_BLEND_CONTROL0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_STORE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_MEM_CTRL_PRE_PARAM0, BIT_0, 1);
	pr_info("end\n");
}
#endif

static int isp_offline_start_frame(void *ctx)
{
	int ret = 0;
	int i, loop, kick_fmcu = 0;
	struct isp_pipe_dev *dev = NULL;
	struct camera_frame *pframe = NULL;
	struct camera_frame *out_frame = NULL;
	struct isp_pipe_context *pctx;
	struct isp_path_desc *path;
	struct isp_cfg_ctx_desc *cfg_desc;
	struct isp_fmcu_ctx_desc *fmcu;

	pr_debug("enter.\n");

	pctx = (struct isp_pipe_context *)ctx;
	if (atomic_read(&pctx->user_cnt) < 1) {
		pr_err("isp cxt %d is not inited.\n", pctx->ctx_id);
		return -EINVAL;
	}

	dev = pctx->dev;
	cfg_desc = (struct isp_cfg_ctx_desc *)dev->cfg_handle;
	fmcu = (struct isp_fmcu_ctx_desc *)pctx->fmcu_handle;

	pframe = camera_dequeue(&pctx->in_queue);
	if (pframe == NULL) {
		pr_warn("no frame from input queue. cxt:%d\n",
				pctx->ctx_id);
		return 0;
	}

	pr_debug("frame %p, ctx %d  ch_id %d.  buf_fd %d\n", pframe,
		pctx->ctx_id, pframe->channel_id, pframe->buf.mfd[0]);

	ret = cambuf_iommu_map(&pframe->buf,
				&dev->isp_hw->pdev->dev);
	if (ret) {
		pr_err("fail to map buf to ISP iommu. cxt %d\n", pctx->ctx_id);
		ret = -EINVAL;
		goto map_err;
	}

	loop = 0;
	do {
		ret = camera_enqueue(&pctx->proc_queue, pframe);
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

	/*
	* ctx_mutex to avoid ctx/all paths param
	* updated when set to register.
	*/
	mutex_lock(&pctx->ctx_mutex);

	/* the context/path maybe init/updated after dev start. */
	if (pctx->updated)
		isp_set_ctx_common(pctx);

	/* config fetch address */
	isp_path_set_fetch_frm(pctx, pframe, &pctx->fetch.addr);

	/* config all paths output */
	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;

		if (pctx->updated)
			ret = isp_set_path(path);

		out_frame = camera_dequeue(&path->out_buf_queue);
		if (out_frame == NULL)
			out_frame =
				camera_dequeue(&path->reserved_buf_queue);

		if (out_frame == NULL) {
			pr_debug("fail to get available output buffer.\n");
			ret = 0;
			goto unlock;
		}

		/* config store buffer */
		isp_path_set_store_frm(path, out_frame);

		/*
		* context proc_queue frame number
		* should be equal to path result queue.
		* if ctx->proc_queue enqueue OK,
		* path result_queue enqueue should be OK.
		*/
		loop = 0;
		do {
			ret = camera_enqueue(&path->result_queue, out_frame);
			if (ret == 0)
				break;
			pr_info("wait for output queue. loop %d\n", loop);

			/* wait for previous frame output queue done. */
			msleep(1);
		} while (loop++ < 500);

		if (ret) {
			trace_isp_irq_cnt(pctx->ctx_id);
			pr_err("fatal: should not be here. path %d, store %d\n",
					path->spath_id, atomic_read(&path->store_cnt));
			/* ret frame to original queue */
			if (out_frame->is_reserved)
				camera_enqueue(
					&path->reserved_buf_queue, out_frame);
			else
				camera_enqueue(
					&path->out_buf_queue, out_frame);
			ret = -EINVAL;
			goto unlock;
		}
		atomic_inc(&path->store_cnt);
	}

	if (fmcu) {
		ret = wait_for_completion_interruptible_timeout(
						&pctx->fmcu_com,
						ISP_CONTEXT_TIMEOUT);
		if (ret == ERESTARTSYS) {
			pr_err("interrupt when isp cfg wait\n");
			ret = -EFAULT;
			goto unlock;
		} else if (ret == 0) {
			pr_err("error: isp context %d timeout.\n",
					pctx->ctx_id);
			ret = -EFAULT;
			goto unlock;
		}
		fmcu->ops->ctx_reset(fmcu);
	}

	if (pctx->slice_ctx) {
		struct slice_cfg_input slc_cfg;

		memset(&slc_cfg, 0, sizeof(slc_cfg));
		for (i = 0; i < ISP_SPATH_NUM; i++) {
			path = &pctx->isp_path[i];
			if (atomic_read(&path->user_cnt) < 1)
				continue;
			slc_cfg.frame_store[i] = &path->store;
		}
		slc_cfg.frame_fetch = &pctx->fetch;
		isp_cfg_slice_fetch_info(&slc_cfg, pctx->slice_ctx);
		isp_cfg_slice_store_info(&slc_cfg, pctx->slice_ctx);

		if (!fmcu) {
			/* should not be here */
			pr_err("error: no fmcu to support slices.\n");
		} else {
			ret = isp_set_slices_fmcu_cmds((void *)fmcu,
				pctx->slice_ctx,
				pctx->ctx_id,
				(uint32_t)pctx->dev->wmode);
			if (ret == 0)
				kick_fmcu = 1;
		}
	}

	pctx->updated = 0;
	mutex_unlock(&pctx->ctx_mutex);

	/* start to prepare/kickoff cfg buffer. */
	if (likely(dev->wmode == ISP_CFG_MODE)) {
		pr_debug("cfg enter.");
		ret = wait_for_completion_interruptible_timeout(
						&pctx->shadow_com,
						ISP_CONTEXT_TIMEOUT);
		if (ret == ERESTARTSYS) {
			pr_err("interrupt when isp cfg wait\n");
			ret = -EFAULT;
			goto deq_output;
		} else if (ret == 0) {
			pr_err("error: isp context %d timeout.\n",
					pctx->ctx_id);
			ret = -EFAULT;
			goto deq_output;
		}
		pr_debug("cfg start.");
		ret = cfg_desc->ops->hw_cfg(
				cfg_desc, pctx->ctx_id, kick_fmcu);

		if (kick_fmcu) {
			pr_debug("fmcu start.");
			ret = fmcu->ops->hw_start(fmcu);
		} else {
			pr_debug("cfg start.");
			ret = cfg_desc->ops->hw_start(
					cfg_desc, pctx->ctx_id);
		}
		read_common_reg(pctx->ctx_id);
	} else {
		if (kick_fmcu) {
			ret = fmcu->ops->hw_start(fmcu);
			pr_debug("fmcu start.");
		} else {
			pr_debug("fetch start.");
			ISP_HREG_WR(ISP_FETCH_START, 1);
		}
	}

	pr_debug("done.\n");
	return 0;

unlock:
	mutex_unlock(&pctx->ctx_mutex);
deq_output:
	for (i = i - 1; i >= 0; i--) {
		path = &pctx->isp_path[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;
		pframe = camera_dequeue_tail(&path->result_queue);
		/* ret frame to original queue */
		if (out_frame->is_reserved)
			camera_enqueue(
				&path->reserved_buf_queue, out_frame);
		else
			camera_enqueue(
				&path->out_buf_queue, out_frame);
		atomic_dec(&path->store_cnt);
	}

	pframe = camera_dequeue_tail(&pctx->proc_queue);
inq_overflow:
	cambuf_iommu_unmap(&pframe->buf);
map_err:
	/* return buffer to cam channel shared buffer queue. */
	pctx->isp_cb_func(ISP_CB_RET_SRC_BUF, pframe, pctx->cb_priv_data);
	return ret;
}


static int isp_offline_thread_loop(void *arg)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_context *pctx;
	struct cam_offline_thread_info *thrd;

	if (!arg) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}

	thrd = (struct cam_offline_thread_info *)arg;
	pctx = (struct isp_pipe_context *)thrd->ctx_handle;
	dev = pctx->dev;

	while (1) {
		if (wait_for_completion_interruptible(
			&thrd->thread_com) == 0) {
			if (atomic_cmpxchg(
					&thrd->thread_stop, 1, 0) == 1) {
				pr_info("isp context %d thread stop.\n",
						pctx->ctx_id);
				break;
			}
			pr_debug("thread com done.\n");

			if (thrd->proc_func(pctx)) {
				pr_err("fail to start isp pipe to proc. exit thread\n");
				pctx->isp_cb_func(
					ISP_CB_DEV_ERR, dev,
					pctx->cb_priv_data);
				break;
			}
		} else {
			pr_debug("offline thread exit!");
			break;
		}
	}

	return 0;
}


static int isp_stop_offline_thread(void *param)
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


static int isp_create_offline_thread(void *param)
{
	struct isp_pipe_context *pctx;
	struct cam_offline_thread_info *thrd;
	char thread_name[32] = { 0 };

	pctx = (struct isp_pipe_context *)param;
	thrd = &pctx->thread;
	thrd->ctx_handle = pctx;
	thrd->proc_func = isp_offline_start_frame;
	atomic_set(&thrd->thread_stop, 0);
	init_completion(&thrd->thread_com);

	sprintf(thread_name, "isp_ctx%d_offline", pctx->ctx_id);
	thrd->thread_task = kthread_run(
						isp_offline_thread_loop,
					      thrd, thread_name);
	if (IS_ERR_OR_NULL(thrd->thread_task)) {
		pr_err("fail to start offline thread for isp ctx%d\n",
				pctx->ctx_id);
		return -EFAULT;
	}

	pr_info("isp ctx %d offline thread created.\n", pctx->ctx_id);
	return 0;
}


static int isp_context_init(struct isp_pipe_dev *dev)
{
	int ret = 0;
	int i, j;
	struct isp_cfg_ctx_desc *cfg_desc = NULL;
	struct isp_pipe_context *pctx;
	enum isp_context_id cid[4] = {
		ISP_CONTEXT_P0,
		ISP_CONTEXT_C0,
		ISP_CONTEXT_P1,
		ISP_CONTEXT_C1
	};

	pr_info("isp contexts init start!\n");
	memset(&dev->ctx[0], 0, sizeof(dev->ctx));
	for (i = 0; i < ISP_CONTEXT_NUM; i++) {
		pctx  = &dev->ctx[i];
		pctx->ctx_id = cid[i];
		pctx->dev = dev;
		pctx->attach_cam_id = CAM_ID_MAX;
		atomic_set(&pctx->user_cnt, 0);
		init_completion(&pctx->shadow_com);
		init_completion(&pctx->fmcu_com);
		/* complete for first frame/slice config */
		complete(&pctx->shadow_com);
		complete(&pctx->fmcu_com);
		mutex_init(&pctx->ctx_mutex);

		for (j = 0; j < ISP_SPATH_NUM; j++)
			atomic_set(&pctx->isp_path[j].user_cnt, 0);

		pr_debug("isp context %d init done!\n", cid[i]);
	}

	/* CFG module init */
	if (dev->wmode == ISP_AP_MODE) {

		pr_info("isp ap mode.\n");
		for (i = 0; i < ISP_CONTEXT_NUM; i++)
			isp_cfg_poll_addr[i] = &s_isp_regbase[0];

	} else {
		cfg_desc = get_isp_cfg_ctx_desc();
		if (!cfg_desc || !cfg_desc->ops) {
			pr_err("error: get isp cfg ctx failed.\n");
			ret = -EINVAL;
			goto cfg_null;
		}
		pr_info("cfg_init start.\n");

		ret = cfg_desc->ops->ctx_init(
				cfg_desc, dev->isp_hw->pdev);
		if (ret) {
			pr_err("error: cfg ctx init failed.\n");
			goto ctx_fail;
		}
		pr_info("cfg_init done.\n");

		ret = cfg_desc->ops->hw_init(cfg_desc);
		if (ret)
			goto hw_fail;
	}
	dev->cfg_handle = cfg_desc;

	pr_info("done!\n");
	return 0;

hw_fail:
	cfg_desc->ops->ctx_deinit(cfg_desc);
ctx_fail:
	put_isp_cfg_ctx_desc(cfg_desc);
cfg_null:
	pr_err("failed!\n");
	return ret;
}

static int isp_context_deinit(struct isp_pipe_dev *dev)
{
	int ret = 0;
	int i, j;
	uint32_t path_id;
	struct isp_cfg_ctx_desc *cfg_desc = NULL;
	struct isp_pipe_context *pctx;
	struct isp_path_desc *path;

	pr_info("enter.\n");

	for (i = 0; i < ISP_CONTEXT_NUM; i++) {
		pctx  = &dev->ctx[i];

		/* free all used path here if user did not call put_path  */
		for (j = 0; j < ISP_SPATH_NUM; j++) {
			path_id = (pctx->ctx_id << 2) | j;
			path = &pctx->isp_path[j];
			if (atomic_read(&path->user_cnt) > 0)
				sprd_isp_put_path(dev, path_id);
		}

		isp_stop_offline_thread(&pctx->thread);

		camera_queue_clear(&pctx->in_queue);
		camera_queue_clear(&pctx->proc_queue);

		mutex_destroy(&pctx->ctx_mutex);
	}
	pr_info("isp contexts deinit done!\n");

	cfg_desc = (struct isp_cfg_ctx_desc *)dev->cfg_handle;
	if (cfg_desc) {
		cfg_desc->ops->ctx_deinit(cfg_desc);
		put_isp_cfg_ctx_desc(cfg_desc);
	}
	dev->cfg_handle = NULL;

	pr_info("done.\n");
	return ret;
}


static int isp_check_spath_available(
			struct isp_path_desc *path,
			struct cam_channel_desc *path_desc)
{
	int ret = 0;

	if (atomic_inc_return(&path->user_cnt) > 1) {
		atomic_dec(&path->user_cnt);
		return -EFAULT;
	}

	/* todo check path scaler capbility  */
	return ret;
}



static int isp_slice_ctx_init(struct isp_pipe_context *pctx)
{
	int ret = 0;
	int j;
	struct isp_path_desc *path;
	struct slice_cfg_input slc_cfg_in;

	if (pctx->fmcu_handle == NULL) {
		pr_debug("no need slices.\n");
		return ret;
	}

	if (pctx->slice_ctx == NULL) {
		pctx->slice_ctx = get_isp_slice_ctx();
		if (IS_ERR_OR_NULL(pctx->slice_ctx)) {
			pr_err("fail to get memory for slice_ctx.\n");
			pctx->slice_ctx = NULL;
			ret = -ENOMEM;
			goto exit;
		}
	}

	memset(&slc_cfg_in, 0, sizeof(struct slice_cfg_input));
	slc_cfg_in.frame_in_size.w = pctx->input_trim.size_x;
	slc_cfg_in.frame_in_size.h = pctx->input_trim.size_y;
	slc_cfg_in.frame_fetch = &pctx->fetch;
	for (j = 0; j < ISP_SPATH_NUM; j++) {
		path = &pctx->isp_path[j];
		if (atomic_read(&path->user_cnt) <= 0)
			continue;
		slc_cfg_in.frame_out_size[j] = &path->dst;
		slc_cfg_in.frame_store[j] = &path->store;
		slc_cfg_in.frame_scaler[j] = &path->scaler;
		slc_cfg_in.frame_deci[j] = &path->deci;
		slc_cfg_in.frame_trim0[j] = &path->in_trim;
		slc_cfg_in.frame_trim1[j] = &path->out_trim;
	}

	isp_cfg_slices(&slc_cfg_in, pctx->slice_ctx);

exit:
	return ret;
}

/* offline process frame */
static int sprd_isp_proc_frame(void *isp_handle,
			void *param, uint32_t in_path_id)
{
	int ret = 0;
	struct camera_frame *pframe;
	enum  isp_context_id  ctx_id;
	struct isp_pipe_context *pctx;
	struct isp_pipe_dev *dev;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pframe = (struct camera_frame *)param;

	ctx_id = (in_path_id >> 2);
	pctx = &dev->ctx[ctx_id];
	pframe->priv_data = pctx;

	pr_debug("frame %p, ctx %d  path %d, ch_id %d.  buf_fd %d\n", pframe,
		ctx_id, in_path_id&3, pframe->channel_id, pframe->buf.mfd[0]);
	ret = camera_enqueue(&pctx->in_queue, pframe);
	if (ret == 0)
		complete(&pctx->thread.thread_com);

	return ret;
}

static int sprd_isp_get_path(void *isp_handle, void *param)
{
	int ret = 0;
	int i, j;
	int sel_path_id = -1;
	enum  isp_context_id  ctx_id;
	enum isp_sub_path_id path_id;
	struct isp_pipe_context *pctx;
	struct isp_pipe_dev *dev;
	struct isp_path_desc *path;
	struct cam_channel_desc *ch_desc;
	struct isp_fmcu_ctx_desc *fmcu;
	struct sprd_cam_hw_info *hw;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	pr_debug("start.\n");

	dev = (struct isp_pipe_dev *)isp_handle;
	ch_desc = (struct cam_channel_desc *)param;

	pr_info("get path for cam %d, prop %d, size %d %d  dst %d  %d\n",
		ch_desc->attach_cam_id, ch_desc->prop,
		ch_desc->input_size.w, ch_desc->input_size.h,
		ch_desc->output_size.w, ch_desc->output_size.h);

	mutex_lock(&dev->path_mutex);

	if (unlikely(dev->wmode == ISP_AP_MODE)) {
		ctx_id = ISP_CONTEXT_P0;
		pctx = &dev->ctx[ctx_id];

		pr_info("AP mode. ctx0 user %d\n",
				atomic_read(&pctx->user_cnt));

		pr_info("new ctx: %d\n", ctx_id);
		for (j = ISP_SPATH_CP; j < ISP_SPATH_NUM; j++) {
			path_id = prefer_path_id[ch_desc->prop][j];
			path = &pctx->isp_path[path_id];
			if (isp_check_spath_available(path, ch_desc) == 0) {
				sel_path_id = (pctx->ctx_id << 2) + path_id;
				path->spath_id = path_id;
				pr_info("new ctx %d, path %d\n",
						pctx->ctx_id, path_id);
				break;
			}
		}
		if (sel_path_id == -1)
			goto exit;
		else if (atomic_read(&pctx->user_cnt) == 0)
			goto path_found;
		else
			goto new_ctx;
	}

	if (ch_desc->shared == 0) {
		pr_info("channel can not shared.\n");
		goto get_freed;
	}

	for (i = 0; i < ISP_CONTEXT_NUM; i++) {
		ctx_id = prefer_cxt_id[ch_desc->prop][i];
		pctx = &dev->ctx[ctx_id];
		if ((atomic_read(&pctx->user_cnt) == 0) ||
			(pctx->shared == 0) ||
			(pctx->attach_cam_id != ch_desc->attach_cam_id) ||
			(pctx->in_fmt != ch_desc->in_fmt) ||
			(pctx->input_size.w != ch_desc->input_size.w) ||
			(pctx->input_size.h != ch_desc->input_size.h)) {
			pr_debug("input not match. user %d\n",
				atomic_read(&pctx->user_cnt));
			continue;
		}

		for (j = ISP_SPATH_CP; j < ISP_SPATH_NUM; j++) {
			path_id = prefer_path_id[ch_desc->prop][j];
			path = &pctx->isp_path[path_id];
			if (isp_check_spath_available(path, ch_desc) == 0) {
				path->spath_id = path_id;
				sel_path_id = (pctx->ctx_id << 2) + path_id;
				pr_info("ctx %d, path %d\n",
						pctx->ctx_id, path_id);
				goto path_found;
			}
			pr_debug("path%d not avail.\n", j);
		}
	}

get_freed:
	/* get freed context and path. */
	for (i = 0; i < ISP_CONTEXT_NUM; i++) {
		ctx_id = prefer_cxt_id[ch_desc->prop][i];
		pctx = &dev->ctx[ctx_id];
		pr_debug("ctx%d user_cnt %d\n", ctx_id,
			atomic_read(&pctx->user_cnt));
		if (atomic_read(&pctx->user_cnt) > 0)
			continue;

		pr_info("get new ctx: %d\n", ctx_id);
		for (j = ISP_SPATH_CP; j < ISP_SPATH_NUM; j++) {
			path_id = prefer_path_id[ch_desc->prop][j];
			path = &pctx->isp_path[path_id];
			if (isp_check_spath_available(path, ch_desc) == 0) {
				sel_path_id = (pctx->ctx_id << 2) + path_id;
				path->spath_id = path_id;
				pr_info("get available path %d\n", path_id);
				goto new_ctx;
			}
		}
	}

	if (sel_path_id == -1)
		goto exit;

new_ctx:
	ret = isp_create_offline_thread(pctx);
	if (unlikely(ret != 0)) {
		pr_err("fail to create offline thread for isp cxt:%d\n",
				pctx->ctx_id);
		goto thrd_err;
	}

	pctx->attach_cam_id = ch_desc->attach_cam_id;
	pctx->in_fmt = ch_desc->in_fmt;
	pctx->input_size = ch_desc->input_size;
	/* fetch whole image. trim before path scaler.*/
	pctx->input_trim.start_x = 0;
	pctx->input_trim.start_y = 0;
	pctx->input_trim.size_x = pctx->input_size.w;
	pctx->input_trim.size_y = pctx->input_size.h;

	camera_queue_init(&pctx->in_queue, ISP_IN_Q_LEN,
						0, isp_ret_src_frame);
	camera_queue_init(&pctx->proc_queue, ISP_PROC_Q_LEN,
						0, isp_ret_src_frame);

	isp_set_ctx_default(pctx);
	reset_isp_irq_cnt(pctx->ctx_id);

	hw = dev->isp_hw;
	ret = hw->ops->enable_irq(hw, &pctx->ctx_id);

path_found:
	path->attach_ctx = pctx;
	path->src = ch_desc->input_size;
	path->dst = ch_desc->output_size;
	path->in_trim = ch_desc->input_trim;
	path->frm_cnt = 0;
	atomic_set(&path->store_cnt, 0);

	camera_queue_init(&path->result_queue, ISP_RESULT_Q_LEN,
						0, isp_ret_out_frame);
	camera_queue_init(&path->out_buf_queue, ISP_OUT_BUF_Q_LEN,
						0, isp_ret_out_frame);
	camera_queue_init(&path->reserved_buf_queue, ISP_RESERVE_BUF_Q_LEN,
						0, isp_destroy_reserved_buf);

	fmcu = (struct isp_fmcu_ctx_desc *)pctx->fmcu_handle;

	if (ch_desc->input_size.w > line_buffer_len  ||
		ch_desc->output_size.w > line_buffer_len) {
		if (fmcu == NULL) {
			fmcu = get_isp_fmcu_ctx_desc();
			pr_info("ctx get fmcu %p\n", fmcu);
			if (fmcu == NULL) {
				pr_err("error: no fmcu for cam %d, img %d %d, out %d, %d\n",
						ch_desc->attach_cam_id,
						ch_desc->input_size.w,
						ch_desc->input_size.h,
						ch_desc->output_size.w,
						ch_desc->output_size.h);
				goto no_fmcu;
			} else if (fmcu->ops) {
				ret = fmcu->ops->ctx_init(
						fmcu, dev->isp_hw->pdev);
				if (ret) {
					pr_err("error: fmcu ctx init failed.\n");
					goto fmcu_err;
				}
			} else {
				pr_err("error: no fmcu ops.\n");
				goto fmcu_err;
			}
			pctx->fmcu_handle = fmcu;
		}
	}

	pctx->shared = ch_desc->shared;
	atomic_inc(&pctx->user_cnt);
	goto exit;

fmcu_err:
	if (fmcu)
		put_isp_fmcu_ctx_desc(fmcu);

no_fmcu:
thrd_err:
	atomic_dec(&path->user_cnt); /* free path */
	sel_path_id = -1;

exit:
	mutex_unlock(&dev->path_mutex);
	pr_info("done, ret path_id: %d\n", sel_path_id);
	return sel_path_id;
}


static int sprd_isp_put_path(void *isp_handle,
				uint32_t put_path_id)
{
	int ret = 0;
	enum  isp_context_id  ctx_id;
	enum isp_sub_path_id path_id;
	struct isp_pipe_context *pctx;
	struct isp_pipe_dev *dev;
	struct isp_path_desc *path;
	struct isp_fmcu_ctx_desc *fmcu;
	struct sprd_cam_hw_info *hw;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	ctx_id = (put_path_id >> 2);
	path_id = (put_path_id & 3);

	pctx = &dev->ctx[ctx_id];
	path = &pctx->isp_path[path_id];

	mutex_lock(&dev->path_mutex);
	if (atomic_read(&path->user_cnt) == 0) {
		mutex_unlock(&dev->path_mutex);
		pr_err("isp cxt %d, path %d is not in use.\n",
					ctx_id, path_id);
		return -EFAULT;
	}

	if (atomic_dec_return(&path->user_cnt) != 0) {
		pr_warn("isp cxt %d, path %d has multi users.\n",
					ctx_id, path_id);
		atomic_set(&path->user_cnt, 0);
	}

	/* reserved buffer queue should be cleared at last. */
	camera_queue_clear(&path->result_queue);
	camera_queue_clear(&path->out_buf_queue);
	camera_queue_clear(&path->reserved_buf_queue);
	path->attach_ctx = NULL;

	if (atomic_dec_return(&pctx->user_cnt) == 0) {
		pr_info("free context %d without users.\n", pctx->ctx_id);

		hw = dev->isp_hw;
		ret = hw->ops->irq_clear(hw, &pctx->ctx_id);
		ret = hw->ops->disable_irq(hw, &pctx->ctx_id);

		isp_stop_offline_thread(&pctx->thread);
		camera_queue_clear(&pctx->in_queue);
		camera_queue_clear(&pctx->proc_queue);

		fmcu = (struct isp_fmcu_ctx_desc *)pctx->fmcu_handle;
		if (fmcu) {
			fmcu->ops->ctx_deinit(fmcu);
			put_isp_fmcu_ctx_desc(fmcu);
			pctx->fmcu_handle = NULL;
		}
		if (pctx->slice_ctx)
			put_isp_slice_ctx(&pctx->slice_ctx);

		pctx->attach_cam_id = CAM_ID_MAX;

		trace_isp_irq_cnt(pctx->ctx_id);
	}

	mutex_unlock(&dev->path_mutex);
	pr_info("done, put path_id: %d\n", put_path_id);
	return ret;
}


static int sprd_isp_cfg_path(void *isp_handle,
				enum isp_path_cfg_cmd cfg_cmd,
				uint32_t cfg_path_id,
				void *param)
{
	int ret = 0;
	enum  isp_context_id  ctx_id;
	enum isp_sub_path_id path_id;
	struct isp_pipe_context *pctx;
	struct isp_pipe_dev *dev;
	struct isp_path_desc *path;
	struct camera_frame *pframe;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	ctx_id = (cfg_path_id >> 2);
	path_id = (cfg_path_id & 3);

	pctx = &dev->ctx[ctx_id];
	path = &pctx->isp_path[path_id];
	if (atomic_read(&path->user_cnt) == 0) {
		pr_err("isp cxt %d, path %d is not in use.\n",
				ctx_id, path_id);
		return -EFAULT;
	}

	switch (cfg_cmd) {
	case ISP_PATH_CFG_OUTPUT_RESERVED_BUF:
	case ISP_PATH_CFG_OUTPUT_BUF:
		pr_debug("cfg buf path %d, %p\n",
			path->spath_id, pframe);
		pframe = (struct camera_frame *)param;
		ret = cambuf_iommu_map(
				&pframe->buf, &dev->isp_hw->pdev->dev);
		if (ret) {
			pr_err("isp buf iommu map failed.\n");
			ret = -EINVAL;
			goto exit;
		}

		/* is_reserved:
		  *  1:  basic mapping reserved buffer;
		  *  2:  copy of reserved buffer.
		  */
		if (cfg_cmd == ISP_PATH_CFG_OUTPUT_RESERVED_BUF) {
			int i = 1;
			struct camera_frame *newfrm;

			pframe->is_reserved = 1;
			pframe->priv_data = path;
			pr_debug("reserved buf\n");
			camera_enqueue(
					&path->reserved_buf_queue, pframe);
			while (i < ISP_RESERVE_BUF_Q_LEN) {
				newfrm = get_empty_frame();
				if (newfrm) {
					newfrm->is_reserved = 2;
					newfrm->priv_data = path;
					memcpy(&newfrm->buf,
						&pframe->buf, sizeof(pframe->buf));
					camera_enqueue(
						&path->reserved_buf_queue, newfrm);
					i++;
				}
			}
		} else {
			pr_debug("output buf\n");
			pframe->is_reserved = 0;
			pframe->priv_data = path;
			ret = camera_enqueue(
					&path->out_buf_queue, pframe);
			if (ret) {
				pr_err("isp path %d output buffer en queue failed.\n",
						cfg_path_id);
				cambuf_iommu_unmap(&pframe->buf);
				goto exit;
			}
		}
		break;

	case ISP_PATH_CFG_COMMON:
		/*cfg_param = (struct isp_path_cfg_param *)param;*/
		pr_info("cfg path %d\n", path->spath_id);
		mutex_lock(&pctx->ctx_mutex);
		ret = isp_cfg_ctx_fetch_info(pctx);
		ret = isp_cfg_path(path);
		ret = isp_slice_ctx_init(pctx);
		pctx->updated = 1;
		mutex_unlock(&pctx->ctx_mutex);
		break;
	default:
		pr_warn("unsupported cmd: %d\n", cfg_cmd);
		break;
	}
exit:
	pr_debug("cfg path %d done. ret %d\n", path->spath_id, ret);
	return ret;
}


static int sprd_isp_set_sb(void *isp_handle, uint32_t path_id,
		isp_dev_callback cb, void *priv_data)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_context *pctx;
	enum  isp_context_id  ctx_id;

	if (!isp_handle || !cb || !priv_data) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	ctx_id = (path_id >> 2) & 3;
	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[ctx_id];
	if (pctx->isp_cb_func == NULL) {
		pctx->isp_cb_func = cb;
		pctx->cb_priv_data = priv_data;
		pr_info("ctx %d,  %p,  set_cb  %p, %p\n",
			pctx->ctx_id, pctx, cb, priv_data);
	}

	return ret;
}


static int sprd_isp_dev_open(void *isp_handle, void *param)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct sprd_cam_hw_info *hw;

	pr_info("enter.\n");
	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	dev = (struct isp_pipe_dev *)isp_handle;
	hw = (struct sprd_cam_hw_info *)param;
	if (hw->ops == NULL) {
		pr_err("error: no hw ops.\n");
		return -EFAULT;
	}

	if (atomic_inc_return(&dev->enable) == 1) {

		pr_info("isp dev init start.\n");

		/* line_buffer_len for debug */
		if (set_line_buffer_len > 0 &&
			set_line_buffer_len < ISP_LINE_BUFFER_W)
			line_buffer_len = set_line_buffer_len;
		else
			line_buffer_len = ISP_LINE_BUFFER_W;

		pr_info("work mode: %d,  line_buf_len: %d\n",
			s_work_mode, line_buffer_len);

		dev->wmode = s_work_mode;
		dev->isp_hw = param;
		mutex_init(&dev->path_mutex);

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

		ret = hw->ops->start(dev->isp_hw, dev);

		ret = isp_context_init(dev);
		if (ret) {
			pr_err("error: isp context init failed.\n");
			ret = -EFAULT;
			goto err_init;
		}
	}

	pr_info("open isp pipe dev done!\n");
	return 0;

err_init:
	hw->ops->stop(hw, dev);
	hw->ops->deinit(hw, dev);
reset_fail:
	hw->ops->disable_clk(hw, NULL);
clk_fail:
	sprd_cam_domain_disable();
	sprd_cam_pw_off();

	atomic_dec(&dev->enable);
	pr_err("fail to open isp dev!\n");
	return ret;
}



int sprd_isp_dev_close(void *isp_handle)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct sprd_cam_hw_info *hw;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	hw = dev->isp_hw;
	if (atomic_dec_return(&dev->enable) == 0) {

		ret = hw->ops->stop(hw, dev);

		ret = isp_context_deinit(dev);
		mutex_destroy(&dev->path_mutex);

		ret = hw->ops->reset(hw, NULL);
		ret = hw->ops->deinit(hw, dev);
		ret = hw->ops->disable_clk(hw, NULL);

		sprd_cam_domain_disable();
		sprd_cam_pw_off();
	}

	pr_info("isp dev disable done\n");
	return ret;

}


static int sprd_isp_dev_reset(void *isp_handle, void *param)
{
	int ret = 0;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	return ret;
}

static struct isp_pipe_ops isp_ops = {
	.open = sprd_isp_dev_open,
	.close = sprd_isp_dev_close,
	.reset = sprd_isp_dev_reset,
	.get_path = sprd_isp_get_path,
	.put_path = sprd_isp_put_path,
	.cfg_path = sprd_isp_cfg_path,
	.proc_frame = sprd_isp_proc_frame,
	.set_callback = sprd_isp_set_sb,
};

struct isp_pipe_ops *get_isp_ops(void)
{
	return &isp_ops;
}

void *get_isp_pipe_dev(void)
{
	struct isp_pipe_dev *dev = NULL;

	mutex_lock(&isp_pipe_dev_mutex);

	if (s_isp_dev) {
		atomic_inc(&s_isp_dev->user_cnt);
		dev = s_isp_dev;
		goto exit;
	}

	dev = vzalloc(sizeof(struct isp_pipe_dev));
	if (!dev)
		goto exit;

	atomic_set(&dev->user_cnt, 1);
	atomic_set(&dev->enable, 0);
	s_isp_dev = dev;

exit:
	mutex_unlock(&isp_pipe_dev_mutex);

	if (dev)
		pr_info("get isp pipe dev: %p, users %d\n",
			dev, atomic_read(&dev->user_cnt));
	else
		pr_err("error: no memory for isp pipe dev.\n");

	return dev;
}


int put_isp_pipe_dev(void *isp_handle)
{
	int ret = 0;
	int user_cnt, en_cnt;
	struct isp_pipe_dev *dev = NULL;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;

	pr_info("put isp pipe dev: %p, %p, users: %d\n",
		dev, s_isp_dev, atomic_read(&dev->user_cnt));

	mutex_lock(&isp_pipe_dev_mutex);

	if (dev != s_isp_dev) {
		mutex_unlock(&isp_pipe_dev_mutex);
		pr_err("error: mismatched dev: %p, %p\n",
					dev, s_isp_dev);
		return -EINVAL;
	}

	user_cnt = atomic_read(&dev->user_cnt);
	en_cnt = atomic_read(&dev->enable);
	if (user_cnt != (en_cnt + 1))
		pr_err("error: mismatched %d %d\n",
				user_cnt, en_cnt);

	if (atomic_dec_return(&dev->user_cnt) == 0) {
		vfree(dev);
		s_isp_dev = NULL;
		pr_info("free isp pipe dev %p\n", dev);
	}
	mutex_unlock(&isp_pipe_dev_mutex);

	pr_info("put isp pipe dev: %p\n", dev);

	return ret;
}
