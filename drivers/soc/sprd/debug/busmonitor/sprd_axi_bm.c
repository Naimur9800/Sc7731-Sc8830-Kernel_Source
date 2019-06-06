/*copyright (C) 2015 Spreadtrum Communications Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>

#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include "busmonitor.h"

#define SPRD_AXI_BM_OFFSET	 0x10000
#define SPRD_AHB_BM_OFFSET	 0x100000

#define PUB_APB_BM_CNT_START	(0x0)
#define PUB_APB_BM_CFG		(0x4)

static struct sprd_bm_dev bm_dev;
static const struct of_device_id bm_axi_of_match[];

static void __iomem *sprd_get_bm_base(u32 bm_index)
{
	return bm_dev.axi_bm_base + bm_index * SPRD_AXI_BM_OFFSET;
}

static void sprd_axi_bm_init(void)
{
	u32 bm_index;
	void __iomem *bm_reg;
	u32 offset = 0;

	for (bm_index = AXI_BM0; bm_index < AHB_BM0; bm_index++) {
		bm_reg = sprd_get_bm_base(bm_index);
		for (offset = 0; offset <= AXI_BM_MATCH_ID_REG; offset += 4)
			writel_relaxed(0, bm_reg + offset);
	}
}

static inline void sprd_axi_bm_chn_en(int chn)
{
	ulong val;
	void __iomem *base_reg;

	base_reg = sprd_get_bm_base(chn);
	val = readl_relaxed(base_reg + AXI_BM_INTC_REG);
	val |= (BM_CHN_EN);
	writel_relaxed(val, base_reg + AXI_BM_INTC_REG);
}

static inline void sprd_axi_bm_chn_int_clr(int chn)
{
	ulong val;
	void __iomem *base_reg;

	base_reg = sprd_get_bm_base(chn);
	val = readl_relaxed(base_reg + AXI_BM_INTC_REG);
	val |= (BM_INT_CLR);
	writel_relaxed(val, base_reg + AXI_BM_INTC_REG);
}

static inline void sprd_axi_bm_chn_int_disable(int chn)
{
	ulong val;
	void __iomem *base_reg;

	base_reg = sprd_get_bm_base(chn);
	val = readl_relaxed(base_reg + AXI_BM_INTC_REG);
	val &= ~(BM_INT_EN);
	val |= (BM_INT_CLR);
	writel_relaxed(val, base_reg + AXI_BM_INTC_REG);
}

static inline void sprd_axi_bm_chn_int_enable(int chn)
{
	ulong val;
	void __iomem *base_reg;

	base_reg = sprd_get_bm_base(chn);
	val = readl_relaxed(base_reg + AXI_BM_INTC_REG);
	val |= (BM_INT_CLR | BM_INT_EN);
	writel_relaxed(val, base_reg + AXI_BM_INTC_REG);
}

static inline ulong sprd_axi_bm_chn_cnt_bw(int chn)
{
	ulong rbw, wbw;
	void __iomem *base_reg;

	base_reg = sprd_get_bm_base(chn);
	rbw = readl_relaxed(base_reg
		+ AXI_BM_RBW_IN_WIN_REG);
	wbw = readl_relaxed(base_reg
		+ AXI_BM_WBW_IN_WIN_REG);
	if (rbw || wbw)
		pr_info("chn:%d,rbw:%lu,wbw:%lu\n", chn, rbw, wbw);

	return (rbw+wbw);
}

static void sprd_axi_bm_cnt_start(void)
{
	ulong bm_index, val;
	void __iomem *base_reg;

	for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++) {
		base_reg = sprd_get_bm_base(bm_index);
		val = readl_relaxed(base_reg
			+ AXI_BM_INTC_REG);
		val |= (BM_CHN_EN | BM_CNT_EN |
			BM_CNT_START | BM_INT_EN);
		writel_relaxed(val, base_reg
			+ AXI_BM_INTC_REG);
	}
}

static void sprd_axi_bm_cnt_stop(void)
{
	ulong bm_index, val;
	void __iomem *base_reg;

	for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++) {
		base_reg = sprd_get_bm_base(bm_index);
		val = readl_relaxed(base_reg
			+ AXI_BM_INTC_REG);
		val &= ~(BM_CNT_START | BM_INT_EN);
		writel_relaxed(val, base_reg
			+ AXI_BM_INTC_REG);
	}
}

static void sprd_axi_bm_int_clr(void)
{
	int bm_index;

	for (bm_index = AXI_BM0; bm_index < AHB_BM0; bm_index++)
		sprd_axi_bm_chn_int_clr(bm_index);
}

static void sprd_axi_bm_int_disable(void)
{
	int bm_index;

	for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++)
		sprd_axi_bm_chn_int_disable(bm_index);
}

static void sprd_axi_bm_int_enable(void)
{
	int bm_index;

	for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++)
		sprd_axi_bm_chn_int_enable(bm_index);
}

/* performance count clear */
static void sprd_axi_bm_cnt_clr(void)
{
	ulong bm_index, val;
	void __iomem *base_reg;

	for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++) {
		base_reg = sprd_get_bm_base(bm_index);
		val = readl_relaxed(base_reg);
		val |= (BM_CHN_EN | BM_CNT_EN);
		writel_relaxed(val, base_reg
			+ AXI_BM_INTC_REG);
		val &= ~(BM_CNT_START);
		writel_relaxed(val, base_reg
			+ AXI_BM_INTC_REG);
		val |= (BM_CNT_CLR);
		writel_relaxed(val, base_reg
			+ AXI_BM_INTC_REG);
	}
}

static void sprd_axi_bm_set_winlen(void)
{
	ulong bm_index, axi_clk, win_len;
	void __iomem *base_reg;

	/* the win len is 10ms */
	axi_clk = 640;
	/* the win_len = (axi_clk / 1000) * 10 */
	win_len = axi_clk * 10000;
	for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++) {
		base_reg = sprd_get_bm_base(bm_index);
		writel_relaxed(win_len, base_reg
			+ AXI_BM_CNT_WIN_LEN_REG);
	}
}

static void sprd_axi_bm_clr_winlen(void)
{
	ulong bm_index;
	void __iomem *base_reg;

	for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++) {
		base_reg = sprd_get_bm_base(bm_index);
		writel_relaxed(0x0, base_reg
			+ AXI_BM_CNT_WIN_LEN_REG);
	}
}

static int sprd_bm_glb_reset_and_enable(u32 bm_index, bool is_enable)
{
	ulong reg_en, reg_rst, bit_en, bit_rst;
	u32 temp, temp_set, temp_clr;

	reg_en = PUB_APB_BM_CFG;
	reg_rst = PUB_APB_BM_CFG;
	bit_en = BIT(16 + bm_index);
	bit_rst = BIT(bm_index);
	temp = readl_relaxed(bm_dev.pub_glb_base
			+ reg_rst);
	temp_set = temp | bit_rst;
	temp_clr = temp & (~bit_rst);
	writel_relaxed(temp_set, bm_dev.pub_glb_base
			+ reg_rst);
	writel_relaxed(temp_clr, bm_dev.pub_glb_base
			+ reg_rst);
	if (is_enable) {
		u32 temp;

		temp = readl_relaxed(
			bm_dev.pub_glb_base + reg_en);
		temp |= bit_en;
		writel_relaxed(temp,
				bm_dev.pub_glb_base + reg_en);
	} else {
		u32 temp;

		temp = readl_relaxed(bm_dev.pub_glb_base
			+ reg_en);
		temp &= ~bit_en;
		writel_relaxed(temp, bm_dev.pub_glb_base
			+ reg_en);
	}
	return SPRD_BM_SUCCESS;
}

static void sprd_bm_glb_count_enable(bool is_enable)
{
	ulong reg_val = 0;

	reg_val = readl_relaxed(bm_dev.pub_glb_base
		+ PUB_APB_BM_CNT_START);
	if (is_enable) {
		writel_relaxed(reg_val|BIT(0), bm_dev.pub_glb_base
			+ PUB_APB_BM_CNT_START);
	} else {
		writel_relaxed(reg_val&~(BIT(0)), bm_dev.pub_glb_base
			+ PUB_APB_BM_CNT_START);
	}
}

static u32 sprd_bm_get_chn_sel(u32 bm_index)
{
	return bm_index;
}

static void sprd_bm_store_int_info(u32 bm_index)
{
	void __iomem *reg_addr;
	uint mask_id;

	reg_addr = sprd_get_bm_base(bm_index);
	if (bm_dev.bm_ctn_dbg.bm_continue_dbg == false) {
		bm_dev.debug_bm_int_info[bm_index].bm_index = bm_index;
		bm_dev.debug_bm_int_info[bm_index].msk_addr =
		readl_relaxed(reg_addr + AXI_BM_MATCH_ADDR_REG);
		bm_dev.debug_bm_int_info[bm_index].msk_cmd =
		readl_relaxed(reg_addr + AXI_BM_MATCH_CMD_REG);
		bm_dev.debug_bm_int_info[bm_index].msk_data_l =
		readl_relaxed(reg_addr + AXI_BM_MATCH_DATA_L_REG);
		bm_dev.debug_bm_int_info[bm_index].msk_data_h =
		readl_relaxed(reg_addr + AXI_BM_MATCH_DATA_H_REG);
		if (bm_index <= AXI_BM9) {
			mask_id = readl_relaxed(reg_addr
				+ AXI_BM_MATCH_ID_REG);
			bm_dev.debug_bm_int_info[bm_index].msk_id
				= mask_id >> 2 ? 0 : mask_id;
		}
		pr_cont("bm axi int info:\nBM CHN:	%d\n"
			"Overlap ADDR:	0x%X\n"
			"Overlap CMD:	0x%X\n"
			"Overlap DATA:	0x%X0x%X\n"
			"Overlap ID:	%s--%u\n",
			bm_dev.debug_bm_int_info[bm_index].bm_index,
			bm_dev.debug_bm_int_info[bm_index].msk_addr,
			bm_dev.debug_bm_int_info[bm_index].msk_cmd,
			bm_dev.debug_bm_int_info[bm_index].msk_data_l,
			bm_dev.debug_bm_int_info[bm_index].msk_data_h,
			bm_dev.bm_chn_name[sprd_bm_get_chn_sel(bm_index)],
			mask_id);
	} else {
		bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev.bm_ctn_dbg.current_cnt]
		.bm_index = bm_index;
		bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev.bm_ctn_dbg.current_cnt]
		.msk_addr = readl_relaxed(reg_addr
				+ AXI_BM_MATCH_ADDR_REG);
		bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev.bm_ctn_dbg.current_cnt]
		.msk_cmd = readl_relaxed(reg_addr
				+ AXI_BM_MATCH_CMD_REG);
		bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev.bm_ctn_dbg.current_cnt]
		.msk_data_l = readl_relaxed(reg_addr
				+ AXI_BM_MATCH_DATA_L_REG);
		bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev.bm_ctn_dbg.current_cnt]
		.msk_data_h = readl_relaxed(reg_addr
				+ AXI_BM_MATCH_DATA_H_REG);
		if (bm_index <= AXI_BM9) {
			mask_id = readl_relaxed(reg_addr
				+ AXI_BM_MATCH_ID_REG);
			bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev
		.bm_ctn_dbg.current_cnt].msk_id = mask_id >> 2 ? 0 : mask_id;
		}
		pr_cont("bm axi ctn info:\n"
		"BM CHN:	%d\n"
		"Overlap ADDR:	0x%X\n"
		"Overlap CMD:	0x%X\n"
		"Overlap DATA:	0x%X0x%X\n"
		"Overlap ID:	%s--%u\n",
			bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev
				.bm_ctn_dbg.current_cnt].bm_index,
			bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev
				.bm_ctn_dbg.current_cnt].msk_addr,
			bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev
				.bm_ctn_dbg.current_cnt].msk_cmd,
			bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev
				.bm_ctn_dbg.current_cnt].msk_data_l,
			bm_dev.bm_ctn_dbg.bm_ctn_info[bm_dev
				.bm_ctn_dbg.current_cnt].msk_data_h,
			bm_dev.bm_chn_name[sprd_bm_get_chn_sel(bm_index)],
			mask_id);
		bm_dev.bm_ctn_dbg.current_cnt++;
	}
}

static irqreturn_t sprd_bm_isr(int irq_num, void *dev)
{
	ulong bm_index, rwbw_cnt, bm_chn, bm_int;
	void __iomem *bm_reg;
	struct bm_per_info *bm_info;
	ktime_t kt;
	struct rtc_time tm;

	bm_info = (struct bm_per_info *)bm_dev.per_buf;
	bm_reg = sprd_get_bm_base(AXI_BM0);
	if (readl_relaxed(bm_reg + AXI_BM_INTC_REG) & BM_CNT_EN) {
		if (bm_info == NULL) {
			pr_err("BM irq ERR, trigger info: int 0x%x"
				",min addr 0x%x, max addr 0x%x, cnt len 0x%x\n",
				readl_relaxed(bm_reg
						+ AXI_BM_INTC_REG),
				readl_relaxed(bm_reg
						+ AXI_BM_ADDR_MIN_REG),
				readl_relaxed(bm_reg
						+ AXI_BM_ADDR_MAX_REG),
				readl_relaxed(bm_reg
						+ AXI_BM_CNT_WIN_LEN_REG));
			return IRQ_NONE;
		}
		sprd_axi_bm_cnt_stop();
		rwbw_cnt = 0x0;
		/* count stop time stamp */
		bm_info[bm_dev.buf_write_index].t_stop = jiffies;
		bm_info[bm_dev.buf_write_index].count = bm_dev.bm_count;
		for (bm_chn = AXI_BM0; bm_chn <= AXI_BM9; bm_chn++) {
			bm_reg = sprd_get_bm_base(bm_chn);
			bm_info[bm_dev.buf_write_index].per_data[bm_chn][0] =
				readl_relaxed(bm_reg
				+ AXI_BM_RTRANS_IN_WIN_REG);
			bm_info[bm_dev.buf_write_index].per_data[bm_chn][1] =
				readl_relaxed(bm_reg
				+ AXI_BM_RBW_IN_WIN_REG);
			bm_info[bm_dev.buf_write_index].per_data[bm_chn][2] =
				readl_relaxed(bm_reg
				+ AXI_BM_RLATENCY_IN_WIN_REG);
			bm_info[bm_dev.buf_write_index].per_data[bm_chn][3] =
				readl_relaxed(bm_reg
				+ AXI_BM_WTRANS_IN_WIN_REG);
			bm_info[bm_dev.buf_write_index].per_data[bm_chn][4] =
				readl_relaxed(bm_reg
				+ AXI_BM_WBW_IN_WIN_REG);
			bm_info[bm_dev.buf_write_index].per_data[bm_chn][5] =
				readl_relaxed(bm_reg
				+ AXI_BM_WLATENCY_IN_WIN_REG);
			rwbw_cnt += bm_info[bm_dev.buf_write_index]
				.per_data[bm_chn][1];
			rwbw_cnt += bm_info[bm_dev.buf_write_index]
				.per_data[bm_chn][4];
		}
		bm_dev.buf_write_index++;
		/* wake up the thread to output log per 4 second */
		if (bm_dev.buf_write_index == PER_COUNT_RECORD_SIZE) {
				bm_dev.buf_write_index = 0;
				up(&bm_dev.bm_seam);
		}
		if (bm_dev.buf_write_index == (PER_COUNT_RECORD_SIZE >> 1))
				up(&bm_dev.bm_seam);
		sprd_axi_bm_int_clr();
		sprd_axi_bm_cnt_clr();
		sprd_axi_bm_set_winlen();
		/* count start time stamp */
		kt = ktime_get();
		tm = rtc_ktime_to_tm(kt);
		tm.tm_hour = tm.tm_hour < 16 ? (tm.tm_hour + 8)
				: (tm.tm_hour - 16);
		bm_info[bm_dev.buf_write_index].t_start = jiffies;
		bm_info[bm_dev.buf_write_index].tmp1 = (tm.tm_hour * 10000)
			+ (tm.tm_min * 100) + tm.tm_sec;
		bm_info[bm_dev.buf_write_index].tmp2 = 640;
		sprd_axi_bm_cnt_start();

		return IRQ_HANDLED;
	}
	for (bm_index = AXI_BM0; bm_index < AHB_BM0; bm_index++) {
		bm_reg = sprd_get_bm_base(bm_index);
		bm_int = readl_relaxed(bm_reg
			+ AXI_BM_INTC_REG);
		if (bm_int & BM_INT_MSK_STS) {
			sprd_bm_store_int_info(bm_index);
			if ((bm_dev.bm_ctn_dbg.bm_continue_dbg == true)
				&& (bm_dev.bm_ctn_dbg.current_cnt
					<= BM_CONTINUE_DEBUG_SIZE)) {
				bm_int &= ~BM_INT_EN;
				bm_int |= (BM_INT_CLR | BM_INT_EN);
			} else {
				bm_int &= ~BM_INT_EN;
			}
			writel_relaxed(bm_int, bm_reg);
			if (bm_dev.bm_st_info.bm_stack_st == true) {
				pr_err("Bus Monitor output stack!\n");
				dump_stack();
			}
			if (true == bm_dev.bm_st_info.bm_panic_st) {
				pr_err("Bus Monitor enter panic!\n");
				BUG();
			}
		}
	}
	return IRQ_HANDLED;
}

void sprd_bm_noirq_ctrl(void)
{
	ulong  rwbw_cnt, bm_chn;
	void __iomem  *bm_reg;
	struct bm_per_info *bm_info;
	struct timeval tv;
	struct rtc_time tm;

	if (!bm_dev.noirq_en_flag)
		return;
	bm_info = (struct bm_per_info *)bm_dev.per_buf;
	bm_reg = sprd_get_bm_base(AXI_BM0);
	if (bm_info == NULL) {
		pr_err("BM irq ERR, trigger info: int 0x%x, min addr 0x%x\
			,max addr 0x%x, cnt len 0x%x\n",
			readl_relaxed(bm_reg + AXI_BM_INTC_REG),
			readl_relaxed(bm_reg + AXI_BM_ADDR_MIN_REG),
			readl_relaxed(bm_reg + AXI_BM_ADDR_MAX_REG),
			readl_relaxed(bm_reg
					+ AXI_BM_CNT_WIN_LEN_REG));
		return;
	}
	sprd_axi_bm_cnt_stop();
	rwbw_cnt = 0x0;
	/* count stop time stamp */
	bm_info[bm_dev.buf_write_index].t_stop = jiffies*1000/HZ;
	bm_info[bm_dev.buf_write_index].count = bm_dev.bm_count;
	for (bm_chn = AXI_BM0; bm_chn <= AXI_BM9; bm_chn++) {
		bm_reg = sprd_get_bm_base(bm_chn);
		bm_info[bm_dev.buf_write_index].per_data[bm_chn][0] =
			readl_relaxed(bm_reg
				+ AXI_BM_RTRANS_IN_WIN_REG);
		bm_info[bm_dev.buf_write_index].per_data[bm_chn][1] =
			readl_relaxed(bm_reg + AXI_BM_RBW_IN_WIN_REG);
		bm_info[bm_dev.buf_write_index].per_data[bm_chn][2] =
			readl_relaxed(bm_reg
				+ AXI_BM_RLATENCY_IN_WIN_REG);
		bm_info[bm_dev.buf_write_index].per_data[bm_chn][3] =
			readl_relaxed(bm_reg
				+ AXI_BM_WTRANS_IN_WIN_REG);
		bm_info[bm_dev.buf_write_index].per_data[bm_chn][4] =
			readl_relaxed(bm_reg
				+ AXI_BM_WBW_IN_WIN_REG);
		bm_info[bm_dev.buf_write_index].per_data[bm_chn][5] =
			readl_relaxed(bm_reg
				+ AXI_BM_WLATENCY_IN_WIN_REG);

		rwbw_cnt += bm_info[bm_dev.buf_write_index].per_data[bm_chn][1];
		rwbw_cnt += bm_info[bm_dev.buf_write_index].per_data[bm_chn][4];
	}
	bm_dev.buf_write_index++;
		/* wake up the thread to output log per 4 second */
	if (bm_dev.buf_write_index == PER_COUNT_RECORD_SIZE) {
			bm_dev.buf_write_index = 0;
			up(&bm_dev.bm_seam);
	}
	if (bm_dev.buf_write_index == (PER_COUNT_RECORD_SIZE >> 1))
			up(&bm_dev.bm_seam);
	sprd_axi_bm_int_clr();
	sprd_axi_bm_cnt_clr();
	sprd_axi_bm_clr_winlen();
	/* count start time stamp */
	do_gettimeofday(&tv);
	rtc_time_to_tm(tv.tv_sec, &tm);
	tm.tm_hour = tm.tm_hour < 16 ? (tm.tm_hour + 8) : (tm.tm_hour - 16);
	bm_info[bm_dev.buf_write_index].t_start = jiffies*1000/HZ;
	bm_info[bm_dev.buf_write_index].tmp1 = (tm.tm_hour * 10000)
		+ (tm.tm_min * 100) + tm.tm_sec;
	bm_info[bm_dev.buf_write_index].tmp2 = 640;
	sprd_axi_bm_cnt_start();
}
EXPORT_SYMBOL_GPL(sprd_bm_noirq_ctrl);

unsigned int sprd_dmc_mon_cnt_bw(void)
{
	int chn;
	ulong cnt = 0;

	if (bm_dev.bm_st_info.bm_dbg_st != true) {
		if (true == bm_dev.bm_st_info.bm_dfs_off_st)
			return 0xFFFFFFFF;
		for (chn = AXI_BM0; chn <= AXI_BM9; chn++)
			cnt += sprd_axi_bm_chn_cnt_bw(chn);
		return cnt;
	} else {
		return 0xFFFFFFFF;
	}
}
EXPORT_SYMBOL_GPL(sprd_dmc_mon_cnt_bw);

void sprd_dmc_mon_cnt_clr(void)
{
	if (bm_dev.bm_st_info.bm_dbg_st != true)
		sprd_axi_bm_cnt_clr();
}
EXPORT_SYMBOL_GPL(sprd_dmc_mon_cnt_clr);

void sprd_dmc_mon_cnt_start(void)
{
	if (bm_dev.bm_st_info.bm_dbg_st != true) {
		sprd_axi_bm_cnt_start();
		sprd_bm_glb_count_enable(true);
		sprd_axi_bm_cnt_start();
	}
}
EXPORT_SYMBOL_GPL(sprd_dmc_mon_cnt_start);

void sprd_dmc_mon_cnt_stop(void)
{
	if (bm_dev.bm_st_info.bm_dbg_st != true) {
		sprd_bm_glb_count_enable(false);
		sprd_axi_bm_cnt_stop();
	}
}
EXPORT_SYMBOL_GPL(sprd_dmc_mon_cnt_stop);

void sprd_dmc_mon_resume(void)
{
}
EXPORT_SYMBOL_GPL(sprd_dmc_mon_resume);

static int sprd_bm_set_point(enum sci_bm_index bm_index, enum sci_bm_chn chn,
	const struct sci_bm_cfg *cfg, void(*call_back)(void *), void *data)
{
	int ret;
	u32 ddr_size = 0;
	struct device_node *np = NULL;
	struct resource res;
	void __iomem *bm_reg;

	if (0 == bm_dev.axi_bm_base)
		return	-ENODEV;
	bm_reg = sprd_get_bm_base(bm_index);
	/* clean the irq status */
	writel_relaxed(readl_relaxed(bm_reg + AXI_BM_INTC_REG)
				| BM_INT_CLR, bm_reg
				+ AXI_BM_INTC_REG);
	writel_relaxed(0, bm_reg + AXI_BM_INTC_REG);
	np = of_find_node_by_name(NULL, "memory");
	if (np) {
		ret = of_address_to_resource(np, 0, &res);
		if (!ret)
			ddr_size = res.end - res.start;
	}
	/* if monitor ddr addr, it needs addr wrap */
	if (cfg->addr_min >= 0x80000000 &&
		(strcmp(bm_dev.bm_chn_name[bm_index], "ERROR_NAME")))
		if (ddr_size == 0x1fffffff) /* 512M */
			bm_store_vale[bm_index].bm_mask = 0xE0000000;
		else if (ddr_size == 0x3fffffff) /* 1G */
			bm_store_vale[bm_index].bm_mask = 0xC0000000;
		else /* >= 4G, do not need mask */
			bm_store_vale[bm_index].bm_mask = 0x0;
	else
		bm_store_vale[bm_index].bm_mask = 0x0;
	if (strcmp(bm_dev.bm_chn_name[bm_index], "ERROR_NAME")) {
		if (bm_index < AHB_BM0) {
			writel_relaxed(
				cfg->addr_min &
				(~bm_store_vale[bm_index].bm_mask), bm_reg +
				AXI_BM_ADDR_MIN_REG);
			writel_relaxed(
				cfg->addr_max &
				(~bm_store_vale[bm_index].bm_mask), bm_reg +
				AXI_BM_ADDR_MAX_REG);
			writel_relaxed(bm_store_vale[bm_index].bm_mask, bm_reg +
				AXI_BM_ADDR_MSK_REG);
			/* the interrupt just trigger by addr range for axi
			* busmonitor, so set the data range difficult to
			* access.
			*/
			writel_relaxed(0x0fffffff, bm_reg +
				AXI_BM_DATA_MIN_L_REG);
			writel_relaxed(0x0fffffff, bm_reg +
				AXI_BM_DATA_MIN_H_REG);
			writel_relaxed(0x0, bm_reg +
				AXI_BM_DATA_MAX_L_REG);
			writel_relaxed(0x0, bm_reg +
				AXI_BM_DATA_MAX_H_REG);
			writel_relaxed(0x0, bm_reg +
				AXI_BM_DATA_MSK_L_REG);
			writel_relaxed(0x0, bm_reg +
				AXI_BM_DATA_MAX_H_REG);
			switch (cfg->bm_mode) {
			case R_MODE:
				writel_relaxed(0x1, bm_reg +
				AXI_BM_CFG_REG);
				break;
			case W_MODE:
				writel_relaxed(0x3, bm_reg +
				AXI_BM_CFG_REG);
				break;
			case RW_MODE:
				writel_relaxed(0x0, bm_reg +
				AXI_BM_CFG_REG);
				break;
			default:
				return -EINVAL;
			}
			writel_relaxed(readl_relaxed(bm_reg + AXI_BM_INTC_REG)
				| BM_INT_EN | BM_CHN_EN, bm_reg
				+ AXI_BM_INTC_REG);
		}
	}

	return SPRD_BM_SUCCESS;
}

int sprd_bm_monitor_cp(unsigned start_addr, unsigned end_addr)
{
	struct sci_bm_cfg bm_cfg;
	u32 rd_wt;
	int ret;
	u32 bm_chn;

	if (NULL == bm_dev.pub_glb_base)
		return -EINVAL;
	rd_wt = RW_MODE;
	bm_cfg.addr_min = (u32)start_addr;
	bm_cfg.addr_max = (u32)end_addr;
	bm_cfg.bm_mode = rd_wt;
	for (bm_chn = AXI_BM0; bm_chn <= AXI_BM9; bm_chn++) {
		if (!strcmp(bm_dev.bm_chn_name[bm_chn], "CPU") ||
			!strcmp(bm_dev.bm_chn_name[bm_chn], "DISP") ||
			!strcmp(bm_dev.bm_chn_name[bm_chn], "GPU") ||
			!strcmp(bm_dev.bm_chn_name[bm_chn], "AP/ZIP") ||
			!strcmp(bm_dev.bm_chn_name[bm_chn], "ZIP") ||
			!strcmp(bm_dev.bm_chn_name[bm_chn], "AP") ||
			!strcmp(bm_dev.bm_chn_name[bm_chn], "MM")) {
			bm_dev.bm_st_info.bm_dbg_st = true;
			bm_dev.bm_st_info.bm_dfs_off_st = true;
			ret = sprd_bm_set_point(bm_chn,
					CHN0, &bm_cfg, NULL, NULL);
			if (SPRD_BM_SUCCESS != ret)
				return -EINVAL;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sprd_bm_monitor_cp);

int sprd_bm_monitor_ap(unsigned start_addr, unsigned end_addr)
{
	struct sci_bm_cfg bm_cfg;
	u32 rd_wt;
	int ret;
	u32 bm_chn;

	if (NULL == bm_dev.pub_glb_base)
		return -EINVAL;
	rd_wt = RW_MODE;
	bm_cfg.addr_min = (u32)start_addr;
	bm_cfg.addr_max = (u32)end_addr;
	bm_cfg.bm_mode = rd_wt;
	for (bm_chn = AXI_BM0; bm_chn <= AXI_BM9; bm_chn++) {
		if (!strcmp(bm_dev.bm_chn_name[bm_chn], "CP0") ||
			!strcmp(bm_dev.bm_chn_name[bm_chn], "CP0 DSP") ||
			!strcmp(bm_dev.bm_chn_name[bm_chn], "CP1 LTEACC/HARQ") ||
			!strcmp(bm_dev.bm_chn_name[bm_chn], "CP1 DSP") ||
			!strcmp(bm_dev.bm_chn_name[bm_chn], "CP1 A5") ||
			!strcmp(bm_dev.bm_chn_name[bm_chn], "CP1 LTEACC") ||
			!strcmp(bm_dev.bm_chn_name[bm_chn], "CP1 HARQ")) {
			bm_dev.bm_st_info.bm_dbg_st = true;
			bm_dev.bm_st_info.bm_dfs_off_st = true;
			ret = sprd_bm_set_point(bm_chn,
					CHN0, &bm_cfg, NULL, NULL);
			if (SPRD_BM_SUCCESS != ret)
				return -EINVAL;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sprd_bm_monitor_ap);

void sprd_bm_set_perform_point(void)
{
	sprd_axi_bm_init();
	sprd_axi_bm_cnt_clr();
	sprd_axi_bm_int_clr();
	sprd_axi_bm_set_winlen();
	sprd_bm_glb_count_enable(false);
	sprd_axi_bm_cnt_start();
	sprd_bm_glb_count_enable(true);
}

void sprd_bm_set_perform_point_nowinlen(void)
{
	sprd_axi_bm_init();
	sprd_axi_bm_cnt_clr();
	sprd_axi_bm_int_clr();
	sprd_bm_glb_count_enable(false);
	sprd_axi_bm_cnt_start();
	sprd_bm_glb_count_enable(true);
}

static void sprd_bm_def_val_set(void)
{
	u32 bm_chn, ret;
	struct sci_bm_cfg bm_cfg;

	for (bm_chn = AXI_BM0; bm_chn < AHB_BM0; bm_chn++) {
		if ((0x0 == bm_def_value[bm_chn].str_addr)
			&& (0x0 == bm_def_value[bm_chn].end_addr))
			continue;
		bm_cfg.addr_min = bm_def_value[bm_chn].str_addr;
		bm_cfg.addr_max = bm_def_value[bm_chn].end_addr;
		bm_cfg.bm_mode = bm_def_value[bm_chn].mode;
		ret = sprd_bm_set_point(bm_chn, bm_def_value[bm_chn].chn_sel,
			&bm_cfg, NULL, NULL);
		if (SPRD_BM_SUCCESS != ret)
			return;
	}
}

static int sprd_bm_output_log(void *p)
{
	mm_segment_t old_fs;
	int ret;
	int read_index;

	while (1) {
		down(&bm_dev.bm_seam);
		if (!bm_dev.log_file) {
			bm_dev.log_file = filp_open(LOG_FILE_PATH, O_RDWR
					| O_CREAT | O_TRUNC, 0644);
			if (IS_ERR(bm_dev.log_file) || !bm_dev.log_file
				|| !bm_dev.log_file->f_dentry) {
				pr_err("file_open(%s) for create failed\n",
				LOG_FILE_PATH);
				return -ENODEV;
			}
		}
		if (bm_dev.buf_write_index >= PER_COUNT_RECORD_SIZE >> 1)
			read_index = 0;
		else
			read_index = PER_COUNT_RECORD_SIZE >> 1;
		old_fs = get_fs();
		set_fs(get_ds());
		ret = vfs_write(bm_dev.log_file,
			(const char *)(bm_dev.per_buf + read_index *
			sizeof(struct bm_per_info)),
			sizeof(struct bm_per_info)
			*(PER_COUNT_RECORD_SIZE >> 1),
			&bm_dev.log_file->f_pos);
		set_fs(old_fs);
		/* raw back file write */
		if (bm_dev.log_file->f_pos >= (LOG_FILE_MAX_RECORDS)) {
			bm_dev.log_file->f_pos = 0x0;
			filp_close(bm_dev.log_file, NULL);
			bm_dev.log_file = 0;
		}
	}
	if (bm_dev.log_file)
		filp_close(bm_dev.log_file, NULL);

	return 0;
}

static enum hrtimer_restart sprd_hrtimer_putlog_handler(struct hrtimer *timer)
{
	ulong  rwbw_cnt, bm_chn;
	void __iomem *bm_reg;
	struct bm_per_info *bm_info;
	ktime_t kt;
	struct timespec64 now;
	struct rtc_time tm;

	bm_info = (struct bm_per_info *)bm_dev.per_buf;
	bm_reg = sprd_get_bm_base(AXI_BM0);
	if (readl_relaxed((bm_reg + AXI_BM_INTC_REG)) & BM_CNT_EN) {
		if (bm_info == NULL) {
			pr_err("BM irq ERR, trigger info: int 0x%x,\
				min addr 0x%x, max addr 0x%x, cnt len 0x%x\n",
				readl_relaxed(bm_reg
						+ AXI_BM_INTC_REG),
				readl_relaxed(bm_reg
						+ AXI_BM_ADDR_MIN_REG),
				readl_relaxed(bm_reg
						+ AXI_BM_ADDR_MAX_REG),
				readl_relaxed(bm_reg
						+ AXI_BM_CNT_WIN_LEN_REG));
			return HRTIMER_NORESTART;
		}
		rwbw_cnt = 0x0;
		/* count stop time stamp */
		getnstimeofday64(&now);
		bm_info[bm_dev.buf_write_index].t_stop =
			now.tv_nsec/1000;
		bm_info[bm_dev.buf_write_index].count = bm_dev.bm_count;
		for (bm_chn = AXI_BM0; bm_chn <= AXI_BM9; bm_chn++) {
			bm_reg = sprd_get_bm_base(bm_chn);
			bm_info[bm_dev.buf_write_index].per_data[bm_chn][0] =
			readl_relaxed(bm_reg
					+ AXI_BM_RTRANS_IN_WIN_REG);
			bm_info[bm_dev.buf_write_index].per_data[bm_chn][1] =
			readl_relaxed(bm_reg
					+ AXI_BM_RBW_IN_WIN_REG);
			bm_info[bm_dev.buf_write_index].per_data[bm_chn][2] =
			readl_relaxed(bm_reg
					+ AXI_BM_RLATENCY_IN_WIN_REG);
			bm_info[bm_dev.buf_write_index].per_data[bm_chn][3] =
			readl_relaxed(bm_reg
					+ AXI_BM_WTRANS_IN_WIN_REG);
			bm_info[bm_dev.buf_write_index].per_data[bm_chn][4] =
			readl_relaxed(bm_reg + AXI_BM_WBW_IN_WIN_REG);
			bm_info[bm_dev.buf_write_index].per_data[bm_chn][5] =
			readl_relaxed(bm_reg
					+ AXI_BM_WLATENCY_IN_WIN_REG);
			rwbw_cnt += bm_info[bm_dev.buf_write_index]
				.per_data[bm_chn][1];
			rwbw_cnt += bm_info[bm_dev.buf_write_index]
				.per_data[bm_chn][4];
		}
		bm_dev.buf_write_index++;
		/* wake up the thread to output log per 4 second */
		if (bm_dev.buf_write_index == PER_COUNT_RECORD_SIZE) {
			bm_dev.buf_write_index = 0;
			up(&bm_dev.bm_seam);
		}
		if (bm_dev.buf_write_index == (PER_COUNT_RECORD_SIZE >> 1))
			up(&bm_dev.bm_seam);
		/* count start time stamp */
		getnstimeofday64(&now);
		kt = ktime_get();
		tm = rtc_ktime_to_tm(kt);
		tm.tm_hour = tm.tm_hour < 16 ? (tm.tm_hour + 8)
				: (tm.tm_hour - 16);
		bm_info[bm_dev.buf_write_index].t_start =
			now.tv_nsec/1000;
		bm_info[bm_dev.buf_write_index].tmp1 = (tm.tm_hour * 10000)
			+ (tm.tm_min * 100) + tm.tm_sec;
		bm_info[bm_dev.buf_write_index].tmp2  = 640;
		hrtimer_forward_now(timer,
				ktime_set(0, bm_dev.timer_interval*1000000));
		return HRTIMER_RESTART;
	}

	return HRTIMER_NORESTART;
}

static ssize_t state_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	if (bm_dev.bm_st_info.bm_dbg_st == true)
		return sprintf(buf, "Bus Monitor in debug mode!\n");
	else
		return sprintf(buf, "Bus Monitor in bandwidth mode!\n");
}

static DEVICE_ATTR_RO(state);

static ssize_t chn_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	u32 bm_index;
	char occ_info[30] = {0};
	char chn_info[30 * BM_CHANNEL_SIZE] = {0};

	for (bm_index = AXI_BM0; bm_index < AHB_BM0; bm_index++) {
		sprintf(occ_info, "%s\n", bm_dev.bm_chn_name[bm_index]);
		strcat(chn_info, occ_info);
	}

	return sprintf(buf, "%s\n", chn_info);
}

static DEVICE_ATTR_RO(chn);

static ssize_t axi_dbg_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	u32 bm_index;
	ulong str_addr, end_addr;
	void __iomem *reg_addr;
	char chn_info[58] = {0};
	char info_buf[58 * 10] = {0};

	for (bm_index = AXI_BM0; bm_index < AHB_BM0; bm_index++) {
		reg_addr = sprd_get_bm_base(bm_index);
		str_addr = readl_relaxed(reg_addr
			+ AXI_BM_ADDR_MIN_REG)
			+ (0x80000000 & bm_store_vale[bm_index].bm_mask);
		end_addr = readl_relaxed(reg_addr
			+ AXI_BM_ADDR_MAX_REG)
			+ (0x80000000 & bm_store_vale[bm_index].bm_mask);
		sprintf(chn_info, "%d	0x%08lX	0x%08lX	%s\n"
			, bm_index, str_addr,
			end_addr, bm_dev.bm_chn_name[bm_index]);
		strcat(info_buf, chn_info);
	}
	pr_info("%s\n", info_buf);

	return sprintf(buf, "%s\n", info_buf);
}

static ssize_t axi_dbg_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned char start[12], end[12], chn[6], mod[3];
	unsigned long start_addr, end_addr, channel;
	struct sci_bm_cfg bm_cfg;
	u32 rd_wt;
	int ret, i;

	ret = sscanf(buf, "%s %s %s %s", chn, start, end, mod);
	if (ret < 4)
		return -EINVAL;
	ret = kstrtoul(start, 0, &start_addr);
	if (ret)
		pr_err("start %s is not in hex or decimal form.\n", buf);
	ret = kstrtoul(end, 0, &end_addr);
	if (ret)
		pr_err("end %s is not in hex or decimal form.\n", buf);
	if ((chn[0] >= '0') && (chn[0] <= '9')) {
		channel = chn[0] - '0';
	} else if (strcmp(chn, "all") == 0) {
		channel = BM_DEBUG_ALL_CHANNEL;
	} else {
		pr_err("please input a legal channel number! e.g: 0-9,all.");
		return -EINVAL;
	}
	if ((strcmp(mod, "r") == 0) || (strcmp(mod, "R") == 0)) {
		rd_wt = R_MODE;
	} else if ((strcmp(mod, "w") == 0)
		|| (strcmp(mod, "W") == 0)) {
		rd_wt = W_MODE;
	} else if ((strcmp(mod, "rw") == 0)
		|| (strcmp(mod, "RW") == 0)) {
		rd_wt = RW_MODE;
	} else {
		pr_err("please input a legal channel mode! e.g: r,w,rw");
		return -EINVAL;
	}

	pr_info("str addr 0x%lx end addr 0x%lx  chn %ld rw %d\n",
		start_addr, end_addr, channel, rd_wt);
	if (((channel > AXI_BM9) && (channel != BM_DEBUG_ALL_CHANNEL))
		|| (rd_wt > RW_MODE) || (start_addr > end_addr))
		return -EINVAL;
	bm_dev.bm_st_info.bm_dbg_st = true;
	bm_dev.bm_st_info.bm_dfs_off_st = true;
	if (channel != BM_DEBUG_ALL_CHANNEL) {
		bm_cfg.addr_min = (u32)start_addr;
		bm_cfg.addr_max = (u32)end_addr;
		bm_cfg.bm_mode = rd_wt;
		ret = sprd_bm_set_point(channel, CHN0, &bm_cfg, NULL, NULL);
		if (SPRD_BM_SUCCESS != ret)
			return ret;
	} else {
		bm_cfg.addr_min = (u32)start_addr;
		bm_cfg.addr_max = (u32)end_addr;
		bm_cfg.bm_mode = rd_wt;
		for (i = AXI_BM0; i < AHB_BM0; i++) {
			ret = sprd_bm_set_point(i, CHN0, &bm_cfg, NULL, NULL);
			if (SPRD_BM_SUCCESS != ret)
				return ret;
		}
	}
	return count;
}

static DEVICE_ATTR_RW(axi_dbg);

static ssize_t bandwidth_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	u32 bw_en, bm_index, ret;
	struct task_struct *t;
	struct bm_per_info *bm_info;
	ktime_t kt;
	struct rtc_time tm;

	bm_dev.bm_st_info.bm_dfs_off_st = false;
	ret = kstrtou32(buf, 0, &bw_en);
	if (ret)
		return -EINVAL;
	if (bw_en) {
		pr_info("bm bandwidth mode enable!!!\n");
		bm_dev.bm_st_info.bm_dbg_st = false;
		if (bm_dev.per_buf == NULL) {
			bm_dev.per_buf = kmalloc(PER_COUNT_BUF_SIZE,
							GFP_KERNEL);
			bm_info = (struct bm_per_info *)bm_dev.per_buf;
			sema_init(&bm_dev.bm_seam, 0);
			for (bm_index = AXI_BM0; bm_index <= AXI_BM9;
				bm_index++) {
				sprd_bm_glb_reset_and_enable(bm_index, true);
				bm_store_vale[bm_index].bm_mask = 0;
			}
			sprd_bm_set_perform_point();
			kt = ktime_get();
			tm = rtc_ktime_to_tm(kt);
			tm.tm_hour = tm.tm_hour < 16 ? (tm.tm_hour + 8)
				: (tm.tm_hour - 16);
			bm_info[bm_dev.buf_write_index].t_start
				= jiffies;
			bm_info[bm_dev.buf_write_index].tmp1
				= (tm.tm_hour * 10000)
				+ (tm.tm_min * 100) + tm.tm_sec;
			bm_info[bm_dev.buf_write_index].tmp2  = 640;
			t = kthread_run(sprd_bm_output_log, NULL, "%s",
				"bm_per_log");
			if (IS_ERR_OR_NULL(t)) {
				pr_err("bm probe: Failed to run"
					" thread bm_per_log\n");
				kfree(bm_dev.per_buf);
				return -EINVAL;
			}
		}
	} else {
		pr_info("bm bandwidth mode disable!!!\n");
		bm_dev.bm_st_info.bm_dbg_st = true;
		if (bm_dev.per_buf != NULL) {
			kfree(bm_dev.per_buf);
			bm_dev.per_buf = NULL;
			if (bm_dev.log_file != NULL) {
				filp_close(bm_dev.log_file, NULL);
				bm_dev.log_file = NULL;
				bm_dev.buf_write_index = 0;
				bm_dev.bm_count = 0;
			}
		}
		sprd_axi_bm_cnt_clr();
		sprd_axi_bm_int_clr();
		sprd_bm_glb_count_enable(false);
		for (bm_index = AXI_BM0; bm_index <= AHB_BM2; bm_index++) {
			sprd_bm_glb_reset_and_enable(bm_index, false);
			bm_store_vale[bm_index].bm_mask = 0;
		}
		sprd_bm_glb_count_enable(false);
	}
	return count;
}

static DEVICE_ATTR_WO(bandwidth);

static ssize_t bandwidth_timer_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	u32 bm_index, ret;
	struct task_struct *t;
	struct bm_per_info *bm_info;
	struct timespec64 now;
	ktime_t kt;
	struct rtc_time tm;

	bm_dev.bm_st_info.bm_dfs_off_st = false;
	ret = kstrtou32(buf, 0, &bm_dev.timer_interval);
	if (ret)
		return -EINVAL;
	if (bm_dev.timer_interval) {
		pr_info("bm bandwidth mode enable!!!\n");
		bm_dev.bm_st_info.bm_dbg_st = false;
		if (bm_dev.per_buf == NULL) {
			bm_dev.per_buf = kmalloc(PER_COUNT_BUF_SIZE,
						GFP_KERNEL);
			bm_info = (struct bm_per_info *)bm_dev.per_buf;
			sema_init(&bm_dev.bm_seam, 0);
			for (bm_index = AXI_BM0; bm_index <= AXI_BM9;
				bm_index++) {
				sprd_bm_glb_reset_and_enable(bm_index, true);
				bm_store_vale[bm_index].bm_mask = 0;
			}
			sprd_bm_set_perform_point_nowinlen();
			msleep(100);
			hrtimer_init(&bm_dev.timer, CLOCK_REALTIME,
				HRTIMER_MODE_ABS);
			getnstimeofday64(&now);
			kt = ktime_get();
			tm = rtc_ktime_to_tm(kt);
			tm.tm_hour = tm.tm_hour < 16 ? (tm.tm_hour + 8)
				: (tm.tm_hour - 16);
			bm_info[bm_dev.buf_write_index].t_start =
				now.tv_nsec/1000;
			bm_info[bm_dev.buf_write_index].tmp1
				= (tm.tm_hour * 10000)
				+ (tm.tm_min * 100) + tm.tm_sec;
			bm_info[bm_dev.buf_write_index].tmp2 = 640;
			bm_dev.timer.function = sprd_hrtimer_putlog_handler;
			hrtimer_start(&bm_dev.timer,
				ktime_set(0, bm_dev.timer_interval*1000000),
				HRTIMER_MODE_ABS);
			t = kthread_run(sprd_bm_output_log, NULL, "%s",
				"bm_per_log");
			if (IS_ERR_OR_NULL(t)) {
				pr_err("bm probe: Failed to run"
					" thread bm_per_log\n");
				kfree(bm_dev.per_buf);
				return -EINVAL;
			}
		}
	} else {
		pr_info("bm bandwidth mode disable!!!\n");
		bm_dev.bm_st_info.bm_dbg_st = true;
		if (bm_dev.per_buf != NULL) {
			kfree(bm_dev.per_buf);
			bm_dev.per_buf = NULL;
			if (bm_dev.log_file != NULL) {
				filp_close(bm_dev.log_file, NULL);
				bm_dev.log_file = NULL;
				bm_dev.buf_write_index = 0;
				bm_dev.bm_count = 0;
			}
		}
		sprd_axi_bm_cnt_clr();
		sprd_axi_bm_int_clr();
		sprd_bm_glb_count_enable(false);
		for (bm_index = AXI_BM0; bm_index <= AXI_BM0; bm_index++) {
			sprd_bm_glb_reset_and_enable(bm_index, false);
			bm_store_vale[bm_index].bm_mask = 0;
		}
		sprd_bm_glb_count_enable(false);
		hrtimer_cancel(&bm_dev.timer);
	}
	return count;
}

static DEVICE_ATTR_WO(bandwidth_timer);

static ssize_t occur_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	u32 bm_index;
	char occ_info[116] = {0};

	for (bm_index = AXI_BM0; bm_index < AHB_BM0; bm_index++) {
		if (bm_dev.debug_bm_int_info[bm_index].msk_addr != 0) {
			sprintf(occ_info, " %s\n addr: 0x%X\n CMD: 0x%X\n"
				"msk_data_l: 0x%X\n msk_data_h: 0x%X\n"
				"id 0x%X\n",
				bm_dev.
				bm_chn_name[sprd_bm_get_chn_sel(bm_index)],
				bm_dev.debug_bm_int_info[bm_index].msk_addr,
				bm_dev.debug_bm_int_info[bm_index].msk_cmd,
				bm_dev.debug_bm_int_info[bm_index].msk_data_l,
				bm_dev.debug_bm_int_info[bm_index].msk_data_h,
				bm_dev.debug_bm_int_info[bm_index].msk_id);
			strcat(buf, occ_info);
		}
	}
	if (occ_info[1] == 0)
		sprintf(buf, ":-) ! No action was monitored by BM!!!\n");

	return strlen(buf);
}

static DEVICE_ATTR_RO(occur);

static ssize_t continue_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	u32 bm_index;
	char occ_info[96] = {0};

	for (bm_index = 0; bm_index < bm_dev.bm_ctn_dbg.loop_cnt; bm_index++) {
		if (bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index].msk_addr != 0) {
			sprintf(occ_info, " %d\n addr: 0x%X\n CMD: 0x%X\n\
				msk_data_l: 0x%X\n"
				"msk_data_h: 0x%X\n id 0x%X\n",
				bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index]
					.bm_index,
				bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index]
					.msk_addr,
				bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index]
					.msk_cmd,
				bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index]
					.msk_data_l,
				bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index]
					.msk_data_h,
				bm_dev.bm_ctn_dbg.bm_ctn_info[bm_index]
					.msk_id);
			strcat(buf, occ_info);
		}
	}
	if (occ_info[1] == 0)
		sprintf(buf, ":-) ! No action was monitored by BM!!!\n");

	return strlen(buf);
}

static ssize_t continue_store(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	u32 ctn_num, ret;

	ret = kstrtou32(buf, 0, &ctn_num);
	if (ret)
		return -EINVAL;
	if (ctn_num) {
		pr_info("Set BM support continue debug!!!\n");
		bm_dev.bm_ctn_dbg.bm_continue_dbg = true;
		if (ctn_num > BM_CONTINUE_DEBUG_SIZE)
			bm_dev.bm_ctn_dbg.loop_cnt = BM_CONTINUE_DEBUG_SIZE;
		else
			bm_dev.bm_ctn_dbg.loop_cnt = ctn_num;
	} else {
		pr_info("Set BM do not support continue debug!!!\n");
		bm_dev.bm_ctn_dbg.bm_continue_dbg = false;
		bm_dev.bm_ctn_dbg.loop_cnt = 0;
	}
	return count;
}

static DEVICE_ATTR_RW(continue);

static ssize_t dfs_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	if (bm_dev.bm_st_info.bm_dfs_off_st == true)
		return sprintf(buf, "The BM DFS is closed.\n");
	else
		return sprintf(buf, "The BM DFS is open.\n");

}

static ssize_t dfs_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	u32 dfs_flg, ret;

	ret = kstrtou32(buf, 0, &dfs_flg);
	if (ret)
		return -EINVAL;
	if (dfs_flg) {
		pr_info("Disable BM DFS.\n");
		bm_dev.bm_st_info.bm_dfs_off_st = true;
	} else {
		pr_info("Reopen BM DFS.\n");
		bm_dev.bm_st_info.bm_dfs_off_st = false;
	}

	return count;
}

static DEVICE_ATTR_RW(dfs);

static ssize_t panic_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	if (bm_dev.bm_st_info.bm_panic_st == true)
		return sprintf(buf, "The BM panic is open.\n");
	else
		return sprintf(buf, "The BM panic is close.\n");
}

static ssize_t panic_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	u32 panic_flg, ret;

	ret = kstrtou32(buf, 0, &panic_flg);
	if (ret)
		return -EINVAL;
	if (panic_flg) {
		pr_info("Reopen BM panic.\n");
		bm_dev.bm_st_info.bm_panic_st = true;
	} else {
		pr_info("Disable BM panic.\n");
		bm_dev.bm_st_info.bm_panic_st = false;
	}
	return count;
}

static DEVICE_ATTR_RW(panic);

static ssize_t stack_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	if (bm_dev.bm_st_info.bm_stack_st == true)
		return sprintf(buf, "The BM stack is open.\n");
	else
		return sprintf(buf, "The BM stack is close.\n");
}

static ssize_t stack_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	u32 panic_flg, ret;

	ret = kstrtou32(buf, 0, &panic_flg);
	if (ret)
		return -EINVAL;
	if (panic_flg) {
		pr_info("Reopen BM DFS.\n");
		bm_dev.bm_st_info.bm_stack_st = true;
	} else {
		pr_info("Disable BM DFS.\n");
		bm_dev.bm_st_info.bm_stack_st = false;
	}
	return count;
}

static DEVICE_ATTR_RW(stack);

static ssize_t disable_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	u32 dis_val, ret;

	ret = kstrtou32(buf, 0, &dis_val);
	if (ret)
		return -EINVAL;
	if (dis_val) {
		pr_info("Set BM disable count %d\n", bm_dev.bm_count);
		writel_relaxed(0x0, bm_dev.pub_glb_base
			+ PUB_APB_BM_CFG);
	} else {
		bm_dev.bm_count++;
		pr_info("Set BM enable count %d\n", bm_dev.bm_count);
		writel_relaxed(BM_CHANNEL_ENABLE_FLAGE,
		bm_dev.pub_glb_base+PUB_APB_BM_CFG);
	}
	return count;
}

static DEVICE_ATTR_WO(disable);

static ssize_t bus_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u32 bm_index;
	ulong axi_val;
	void __iomem *reg_addr;
	char  axi_info[20] = {0}, axi_buf[10 * 20] = {0};

	for (bm_index = AXI_BM0; bm_index < AHB_BM0; bm_index++) {
		reg_addr = sprd_get_bm_base(bm_index);
		axi_val = readl_relaxed(reg_addr
			+ AXI_BM_BUS_STATUS_REG);
		sprintf(axi_info, "0x%08lX\n", axi_val);
		strcat(axi_buf, axi_info);
	}
	pr_info("%s\n", axi_buf);

	return sprintf((char *)buf, "%s\n", axi_buf);
}

static DEVICE_ATTR_RO(bus_status);

static ssize_t noirqctrl_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	u32 bw_en, bm_index, ret;
	struct task_struct *t;

	bm_dev.bm_st_info.bm_dfs_off_st = false;
	ret = kstrtou32(buf, 0, &bw_en);
	if (ret)
		return -EINVAL;
	if (bw_en) {
		pr_info("bm bandwidth mode enable!!!\n");
		bm_dev.noirq_en_flag = 1;
		bm_dev.bm_st_info.bm_dbg_st = false;
		if (bm_dev.per_buf == NULL) {
			bm_dev.per_buf = kmalloc(PER_COUNT_BUF_SIZE,
				GFP_KERNEL);
			sema_init(&bm_dev.bm_seam, 0);
			t = kthread_run(sprd_bm_output_log, NULL, "%s",
				"bm_per_log");
			if (IS_ERR_OR_NULL(t)) {
				pr_err("bm probe: Failed to run"
					" thread bm_per_log\n");
				kfree(bm_dev.per_buf);
				return -EINVAL;
			}
		}
		for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++) {
			sprd_bm_glb_reset_and_enable(bm_index, true);
			bm_store_vale[bm_index].bm_mask = 0;
		}
		sprd_axi_bm_init();
		sprd_axi_bm_cnt_clr();
		sprd_axi_bm_int_clr();
		sprd_axi_bm_clr_winlen();
		sprd_bm_glb_count_enable(false);
		sprd_axi_bm_cnt_start();
		sprd_bm_glb_count_enable(true);
		msleep(100);
	} else {
		pr_info("bm bandwidth mode disable!!!\n");
		bm_dev.noirq_en_flag = 0;
		bm_dev.bm_st_info.bm_dbg_st = true;
		if (bm_dev.per_buf != NULL) {
			kfree(bm_dev.per_buf);
			bm_dev.per_buf = NULL;
			if (bm_dev.log_file != NULL) {
				filp_close(bm_dev.log_file, NULL);
				bm_dev.log_file = NULL;
				bm_dev.buf_write_index = 0;
				bm_dev.bm_count = 0;
			}
		}
		sprd_axi_bm_cnt_clr();
		sprd_axi_bm_int_clr();
		sprd_bm_glb_count_enable(false);
		for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++) {
			sprd_bm_glb_reset_and_enable(bm_index, false);
			bm_store_vale[bm_index].bm_mask = 0;
		}
		sprd_bm_glb_count_enable(false);
	}
	return count;
}

static DEVICE_ATTR_WO(noirqctrl);

static struct attribute *bm_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_chn.attr,
	&dev_attr_axi_dbg.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_bandwidth_timer.attr,
	&dev_attr_occur.attr,
	&dev_attr_continue.attr,
	&dev_attr_dfs.attr,
	&dev_attr_panic.attr,
	&dev_attr_stack.attr,
	&dev_attr_disable.attr,
	&dev_attr_bus_status.attr,
	&dev_attr_noirqctrl.attr,
	NULL,
};

ATTRIBUTE_GROUPS(bm);

static void sprd_bm_init_name_info(const struct of_device_id *lock_of_id)
{
	u32 n;

	for (n = 0; n < MEMLAYOUT_COUNT; n++) {
			if (!strncmp(lock_of_id->compatible, board_name_gat[n],
				11)) {
				bm_dev.bm_chn_name = chn_name_gat[n];
				break;
			}
	}
}

static void sprd_bm_get_hw_info(const struct of_device_id *lock_of_id)
{
	struct bm_status *this_status = NULL;
	u32 n;

	for (n = 0; n < MEMLAYOUT_COUNT; n++) {
		if (!strncmp(lock_of_id->compatible, board_name_gat[n], 11)) {
			this_status = bmstatus_gat[n];
			break;
		}
	}
	if (n == MEMLAYOUT_COUNT)
		panic("busmonitor has no match hw_info");
	bm_dev.bm_st_info.bm_def_mode = this_status->bm_status;
	pr_info("Bus Monitor mode: 0 - Pref; 1 - Debug. Current mode %d\n",
			bm_dev.bm_st_info.bm_def_mode);
	bm_dev.bm_st_info.axi_bm_cnt = this_status->bm_count0;
	sprd_bm_init_name_info(lock_of_id);
}

static int sprd_bm_axi_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	u32 bm_chn, bm_mode;
	void __iomem *reg_addr;

	if (true == bm_dev.bm_st_info.bm_dbg_st) {
		for (bm_chn = AXI_BM0; bm_chn < AHB_BM0; bm_chn++) {
			reg_addr = sprd_get_bm_base(bm_chn);
			bm_store_vale[bm_chn].str_addr
				= readl_relaxed(reg_addr
				+ AXI_BM_ADDR_MIN_REG) + (0x80000000
				& bm_store_vale[bm_chn].bm_mask);
			bm_store_vale[bm_chn].end_addr
				= readl_relaxed(reg_addr
				+ AXI_BM_ADDR_MAX_REG) + (0x80000000
				& bm_store_vale[bm_chn].bm_mask);
			bm_mode = readl_relaxed(reg_addr
				+ AXI_BM_CFG_REG) & 0x3;
			if (0 == bm_mode)
				bm_store_vale[bm_chn].mode = RW_MODE;
			else if (1 == bm_mode)
				bm_store_vale[bm_chn].mode = R_MODE;
			else if (3 == bm_mode)
				bm_store_vale[bm_chn].mode = W_MODE;
			if (bm_chn >= AHB_BM0) {
				bm_store_vale[bm_chn].min_data
				= readl_relaxed(reg_addr
					+ AXI_BM_DATA_MIN_L_REG);
				bm_store_vale[bm_chn].max_data
				= readl_relaxed(reg_addr
					+ AXI_BM_DATA_MAX_L_REG);
				regmap_read(bm_dev.ap_ahb_gpr,
					REG_AP_AHB_MISC_CFG, &bm_mode);
				switch (bm_chn) {
				case AHB_BM0:
					bm_store_vale[bm_chn].chn_sel
						= (bm_mode >> 4) & 0x3;
					break;
				case AHB_BM1:
					bm_store_vale[bm_chn].chn_sel
						= (bm_mode >> 8) & 0x3;
					break;
				case AHB_BM2:
					bm_store_vale[bm_chn].chn_sel
						= (bm_mode >> 10) & 0x3;
					break;
				default:
					break;
				}
			}
		}
	}

	return 0;
}

static int sprd_bm_axi_resume(struct platform_device *pdev)
{
	u32 bm_chn, ret;
	struct sci_bm_cfg bm_cfg;

	sprd_axi_bm_init();
	sprd_axi_bm_int_clr();
	if (true == bm_dev.bm_st_info.bm_dbg_st) {
		for (bm_chn = AXI_BM0; bm_chn < AHB_BM0; bm_chn++) {
			if (((0x0 == bm_store_vale[bm_chn].str_addr)
				&& (0x0 == bm_store_vale[bm_chn]
				.end_addr)) || !strcmp(bm_dev
				.bm_chn_name[bm_chn], "ERROR_NAME"))
				continue;
			bm_cfg.addr_min = bm_store_vale[bm_chn].str_addr;
			bm_cfg.addr_max = bm_store_vale[bm_chn].end_addr;
			bm_cfg.bm_mode = bm_store_vale[bm_chn].mode;
			if (bm_chn >= AHB_BM0) {
				bm_cfg.data_min_l = bm_store_vale[bm_chn]
							.min_data;
				bm_cfg.data_min_h = 0;
				bm_cfg.data_max_l = bm_store_vale[bm_chn]
							.max_data;
				bm_cfg.data_max_h = 0;
			}
			ret = sprd_bm_set_point(bm_chn, bm_store_vale[bm_chn]
				.chn_sel, &bm_cfg, NULL, NULL);
			if (SPRD_BM_SUCCESS != ret)
				return ret;
		}
	}

	return 0;
}

static int sprd_bm_axi_probe(struct platform_device *pdev)
{
	int ret;
	u32 bm_index;
	u32 irq_axi;
	struct resource *res = NULL;
	const struct of_device_id *lock_of_id = NULL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "sprd_bm get io resource failed!\n");
		return -ENODEV;
	}
	bm_dev.axi_bm_base = devm_ioremap_nocache(&pdev->dev,
		res->start, resource_size(res));
	if (!bm_dev.axi_bm_base) {
		dev_err(&pdev->dev, "sprd_bm get axi base address failed!\n");
		return -ENODEV;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "sprd_bm get io resource failed!\n");
		return -ENODEV;
	}
	bm_dev.pub_glb_base = devm_ioremap_nocache(&pdev->dev,
		res->start, resource_size(res));
	if (!bm_dev.axi_bm_base) {
		dev_err(&pdev->dev, "sprd_bm get axi base address failed!\n");
		return -ENODEV;
	}
	irq_axi = platform_get_irq(pdev, 0);
	if (irq_axi < 0) {
		pr_err("Error: Can't get the axi irq number!\n");
		return -EBUSY;
	}
	ret = devm_request_irq(&pdev->dev, irq_axi, sprd_bm_isr,
			IRQF_TRIGGER_NONE, pdev->name, pdev);
	if (ret)
		return ret;
	lock_of_id = of_match_node(bm_axi_of_match, pdev->dev.of_node);
	sprd_bm_get_hw_info(lock_of_id);
	for (bm_index = AXI_BM0; bm_index < BM_SIZE; bm_index++)
		sprd_bm_glb_reset_and_enable(bm_index, true);
	sprd_bm_glb_count_enable(true);
	sprd_axi_bm_int_clr();
	sprd_axi_bm_init();
	dev_set_name(&bm_dev.dev, "busmonitor");
	bm_dev.dev.parent = &pdev->dev;
	bm_dev.dev.groups = bm_groups;
	dev_set_drvdata(&bm_dev.dev, &bm_dev);
	ret = device_register(&bm_dev.dev);
	if (ret) {
		pr_err("Unable to device_register\n");
		return ret;
	}
	if (bm_dev.bm_st_info.bm_def_mode > 0) {
		sprd_bm_def_val_set();
		bm_dev.bm_st_info.bm_dbg_st = true;
		bm_dev.bm_st_info.bm_stack_st = true;
		bm_dev.bm_st_info.bm_panic_st = true;
		bm_dev.bm_st_info.bm_dfs_off_st = true;
		pr_info("Bus Monitor default config set success!!!\n");
	}
	pr_info("sprd_axi_bm_probe done!\n");
	return SPRD_BM_SUCCESS;
}

static int sprd_bm_axi_remove(struct platform_device *pdev)
{
	u32 bm_index;

	for (bm_index = AXI_BM0; bm_index < AHB_BM0; bm_index++)
		sprd_bm_glb_reset_and_enable(bm_index, false);
	sprd_bm_glb_count_enable(false);
	device_unregister(&pdev->dev);
	return SPRD_BM_SUCCESS;
}

static int sprd_bm_open(struct inode *inode, struct file *filp)
{
	pr_info("%s!\n", __func__);
	return 0;
}

static int sprd_bm_release(struct inode *inode, struct file *filp)
{
	pr_info("%s!\n", __func__);
	return 0;
}

static long sprd_bm_ioctl(struct file *filp, unsigned int cmd,
	unsigned long args)
{
	struct sci_bm_cfg bm_cfg;
	struct task_struct *t;
	static struct timespec64 now;
	void __iomem *bm_reg;
	unsigned long bm_user;
	unsigned char bm_name[18] = {0};
	u32 i, ret, bm_index = 0;

	if (cmd >= (BM_CHANNELS << BM_CHN_NAME_CMD_OFFSET)) {
		if (BM_CHANNELS == (cmd >> BM_CHN_NAME_CMD_OFFSET)) {
			bm_index = cmd & 0xf;
			cmd = cmd >> BM_CHN_NAME_CMD_OFFSET;
		}
	}
	if (_IOC_TYPE(cmd) >= BM_CMD_MAX)
		return -EINVAL;

	switch (cmd) {
	case BM_STATE:
		if (bm_dev.bm_st_info.bm_dbg_st)
			bm_user = 1;
		if (put_user(bm_user, (unsigned long __user *)args))
			return -EFAULT;
		break;
	case BM_CHANNELS:
		for (i = 0; i < strlen(bm_dev.bm_chn_name[bm_index]); i++)
			bm_name[i] = bm_dev.bm_chn_name[bm_index][i];
		bm_name[i] = '\0';
		if (copy_to_user((unsigned char __user *)args,
				&bm_name, strlen(bm_name)))
				return -EFAULT;
		break;
	case BM_AXI_DEBUG_SET:/* set axi debug point */
		if (copy_from_user(&bm_cfg, (struct sci_bm_cfg __user *)args,
			sizeof(struct sci_bm_cfg)))
			return -EFAULT;
		ret = sprd_bm_set_point(bm_cfg.chn, CHN0, &bm_cfg, NULL, NULL);
		if (SPRD_BM_SUCCESS != ret)
			return ret;
		bm_dev.bm_st_info.bm_dbg_st = true;
		bm_dev.bm_st_info.bm_dfs_off_st = true;
		break;
	case BM_PERFORM_SET:/* set performance point */
		if (bm_dev.per_buf == NULL) {
			bm_dev.per_buf = kmalloc(PER_COUNT_BUF_SIZE,
						GFP_KERNEL);
			sema_init(&bm_dev.bm_seam, 0);
			t = kthread_run(sprd_bm_output_log, NULL, "%s",
				"bm_per_log");
			if (IS_ERR(t)) {
				pr_err("bm probe: Failed to run"
					" thread bm_per_log\n");
				kthread_stop(t);
				return -EFAULT;
			}
		}
		for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++)
			sprd_bm_glb_reset_and_enable(bm_index, true);
		sprd_bm_set_perform_point();
		msleep(100);
		bm_dev.bm_st_info.bm_dbg_st = false;
		bm_dev.bm_st_info.bm_dfs_off_st = false;
		break;
	case BM_PERFORM_UNSET:/* unset performance point */
		if (bm_dev.per_buf != NULL)
			kfree(bm_dev.per_buf);
		for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++)
			sprd_bm_glb_reset_and_enable(bm_index, false);
		 sprd_bm_glb_count_enable(false);
		break;
	case BM_OCCUR:/* read dbg info */
		if (copy_to_user((struct bm_debug_info __user *)args,
			bm_dev.debug_bm_int_info, sizeof(struct bm_debug_info)))
			return -EFAULT;
		break;
	case BM_CONTINUE_SET:/* set continue statue */
		if (copy_from_user(&bm_dev.bm_ctn_dbg,
			(struct bm_continue_debug __user *)args,
			sizeof(struct bm_continue_debug)))
			return -EFAULT;
		break;
	case BM_CONTINUE_UNSET:
		bm_dev.bm_ctn_dbg.bm_continue_dbg = false;
		break;
	case BM_DFS_SET:/* set DFS statue */
		bm_dev.bm_st_info.bm_dfs_off_st = true;
		break;
	case BM_DFS_UNSET:
		bm_dev.bm_st_info.bm_dfs_off_st = false;
		break;
	case BM_PANIC_SET:/* set DFS statue */
		bm_dev.bm_st_info.bm_panic_st = true;
		break;
	case BM_PANIC_UNSET:
		bm_dev.bm_st_info.bm_panic_st = false;
		break;
	case BM_BW_CNT_START:
		sprd_dmc_mon_cnt_start();
		break;
	case BM_BW_CNT_STOP:
		sprd_dmc_mon_cnt_stop();
		break;
	case BM_BW_CNT_RESUME:
		break;
	case BM_BW_CNT:
		bm_user = sprd_dmc_mon_cnt_bw();
		if (put_user(bm_user, (unsigned long __user *)args))
			return -EFAULT;
	case BM_BW_CNT_CLR:
		sprd_dmc_mon_cnt_clr();
		break;
	case BM_DBG_INT_CLR:
		sprd_axi_bm_int_disable();
		break;
	case BM_DBG_INT_SET:
		sprd_axi_bm_int_enable();
		break;
	case BM_DISABLE:
		writel_relaxed(0x0, bm_dev.pub_glb_base
			+ PUB_APB_BM_CFG);
		break;
	case BM_ENABLE:
		writel_relaxed(BM_CHANNEL_ENABLE_FLAGE,
			bm_dev.pub_glb_base + PUB_APB_BM_CFG);
		break;
	case BM_PROF_SET:
		for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++)
			sprd_bm_glb_reset_and_enable(bm_index, true);
		sprd_axi_bm_init();
		sprd_axi_bm_cnt_clr();
		sprd_axi_bm_int_clr();
		sprd_bm_glb_count_enable(false);
		sprd_axi_bm_cnt_start();
		sprd_bm_glb_count_enable(true);
		break;
	case BM_PROF_CLR:
		for (bm_index = AXI_BM0; bm_index <= AXI_BM9; bm_index++)
			sprd_bm_glb_reset_and_enable(bm_index, false);
		sprd_bm_glb_count_enable(false);
		break;
	case BM_CHN_CNT:
		if (put_user(bm_dev.bm_st_info.axi_bm_cnt,
			(unsigned long __user *)args))
			return -EFAULT;
		break;
	case BM_RD_CNT:
		ret = copy_from_user(&bm_user,
			(unsigned long __user *)args, sizeof(unsigned long));
		if (bm_user > AXI_BM9 || bm_user < AXI_BM0)
			return -EFAULT;
		bm_reg = sprd_get_bm_base(bm_user);
		bm_user = readl_relaxed(bm_reg + AXI_BM_RTRANS_IN_WIN_REG);
		if (copy_to_user((unsigned long __user *)args,
			&bm_user, sizeof(unsigned long)))
			return -EFAULT;
		break;
	case BM_WR_CNT:
		ret = copy_from_user(&bm_user,
			(unsigned long __user *)args, sizeof(unsigned long));
		if (bm_user > AXI_BM9 || bm_user < AXI_BM0)
			return -EFAULT;
		bm_reg = sprd_get_bm_base(bm_user);
		bm_user = readl_relaxed(bm_reg + AXI_BM_WTRANS_IN_WIN_REG);
		if (copy_to_user((unsigned long __user *)args,
			&bm_user, sizeof(unsigned long)))
			return -EFAULT;
		break;
	case BM_RD_BW:
		ret = copy_from_user(&bm_user,
			(unsigned long __user *)args, sizeof(unsigned long));
		if (bm_user > AXI_BM9 || bm_user < AXI_BM0)
			return -EFAULT;
		bm_reg = sprd_get_bm_base(bm_user);
		bm_user = readl_relaxed(bm_reg + AXI_BM_RBW_IN_WIN_REG);
		if (copy_to_user((unsigned long __user *)args,
			&bm_user, sizeof(unsigned long)))
			return -EFAULT;
		break;
	case BM_WR_BW:
		ret = copy_from_user(&bm_user, (unsigned long __user *)args,
			sizeof(unsigned long));
		if (bm_user > AXI_BM9 || bm_user < AXI_BM0)
			return -EFAULT;
		bm_reg = sprd_get_bm_base(bm_user);
		bm_user = readl_relaxed(bm_reg + AXI_BM_WBW_IN_WIN_REG);
		if (copy_to_user((unsigned long __user *)args, &bm_user,
			sizeof(unsigned long)))
			return -EFAULT;
		break;
	case BM_RD_LATENCY:
		ret = copy_from_user(&bm_user, (unsigned long __user *)args,
			sizeof(unsigned long));
		if (bm_user > AXI_BM9 || bm_user < AXI_BM0)
			return -EFAULT;
		bm_reg = sprd_get_bm_base(bm_user);
		bm_user = readl_relaxed(bm_reg + AXI_BM_RLATENCY_IN_WIN_REG);
		if (copy_to_user((unsigned long __user *)args, &bm_user,
			sizeof(unsigned long)))
			return -EFAULT;
		break;
	case BM_WR_LATENCY:
		ret = copy_from_user(&bm_user, (unsigned long __user *)args,
			sizeof(unsigned long));
		if (bm_user > AXI_BM9 || bm_user < AXI_BM0)
			return -EFAULT;
		bm_reg = sprd_get_bm_base(bm_user);
		bm_user = readl_relaxed(bm_reg + AXI_BM_WLATENCY_IN_WIN_REG);
		if (copy_to_user((unsigned long __user *)args, &bm_user,
			sizeof(unsigned long)))
			return -EFAULT;
		break;
	case BM_KERNEL_TIME:
		getnstimeofday64(&now);
		if (copy_to_user((u64 __user *)args,
			&now.tv_sec, sizeof(u64)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct file_operations bm_fops = {
	.owner = THIS_MODULE,
	.open = sprd_bm_open,
	.release = sprd_bm_release,
	.unlocked_ioctl = sprd_bm_ioctl,
};

static struct miscdevice bm_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sprd_axi_bm",
	.fops = &bm_fops,
};

static const struct of_device_id bm_axi_of_match[] = {
	{ .compatible = "sprd,sc9838-pub-busmon", },
	{ .compatible = "sprd,sc9830-pub-busmon", },
	{ }
};

static struct platform_driver sprd_bm_axi_driver = {
	.probe    = sprd_bm_axi_probe,
	.remove   = sprd_bm_axi_remove,
	.suspend  = sprd_bm_axi_suspend,
	.resume   = sprd_bm_axi_resume,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd_axi_bm",
		.of_match_table = bm_axi_of_match,
	},
};

static int __init sprd_busmonitor_init(void)
{
	misc_register(&bm_misc);
	return platform_driver_register(&sprd_bm_axi_driver);
}

static void __exit sprd_busmonitor_exit(void)
{
	platform_driver_unregister(&sprd_bm_axi_driver);
	misc_deregister(&bm_misc);
}

module_init(sprd_busmonitor_init);
module_exit(sprd_busmonitor_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aiden Cheng<aiden.cheng@spreadtrum.com>");
MODULE_DESCRIPTION("spreadtrum platform busmonitor driver");



