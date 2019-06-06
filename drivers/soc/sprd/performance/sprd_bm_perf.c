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
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/debugfs.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dma/sprd_dma.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/jiffies.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include "sprd_bm_perf.h"

static const struct of_device_id sprd_bm_match[];
struct platform_device *pdev_internal;
static bool internal;
u64 *bandwidth_data;

static int sprd_bm_dma_request(struct device *dev,
		struct sprd_bm_dev *sdev)
{
	int ret;
	size_t dma_buf_size;

	dma_buf_size = BM_DMA_BUF_LEN(sdev->bm_dma_info.cir,
					sdev->bm_cnt);
	sdev->bm_dma_info.dst_buf_v = dma_zalloc_coherent(dev,
				dma_buf_size * 2,
				&(sdev->bm_dma_info.dst_buf_p),
				GFP_KERNEL);
	if (!sdev->bm_dma_info.dst_buf_v) {
		dev_err(sdev->dev.parent, "alloc dst buf mem failed!\n");
		return -ENOMEM;
	}

	sdev->bm_dma_info.llist_v = dma_zalloc_coherent(dev,
		sizeof(struct sprd_dma_cfg) * BM_DMA_LLINK_SIZE,
		&(sdev->bm_dma_info.llist_p), GFP_KERNEL);
	if (!sdev->bm_dma_info.llist_v) {
		dev_err(sdev->dev.parent, "alloc llist buf mem failed!\n");
		ret = -ENOMEM;
		goto err_llist;
	}

	sdev->bm_dma_info.dma = dma_request_slave_channel(dev, "bm_chn");
	if (!sdev->bm_dma_info.dma) {
		dev_err(sdev->dev.parent, "request BM DMDA channel failed!\n");
		ret = -ENXIO;
		goto err_request;
	}
	return 0;

err_request:
	dma_free_coherent(dev,
		sizeof(struct sprd_dma_cfg) * BM_DMA_LLINK_SIZE,
		sdev->bm_dma_info.llist_v, sdev->bm_dma_info.llist_p);
err_llist:
	dma_free_coherent(dev, dma_buf_size * 2, sdev->bm_dma_info.dst_buf_v,
		sdev->bm_dma_info.dst_buf_p);
	return ret;
}

static int sprd_bm_dma_config(struct sprd_bm_dev *sdev)
{
	int i, ret;
	static struct sprd_dma_cfg config[BM_DMA_LLINK_SIZE];
	struct dma_async_tx_descriptor *dma_des;
	struct dma_device *dma_dev = sdev->bm_dma_info.dma->device;

	for (i = 0; i < BM_DMA_LLINK_SIZE; i++) {
		memset(&config[i], 0, sizeof(struct sprd_dma_cfg));
		config[i].link_cfg_v =
			(unsigned long)sdev->bm_dma_info.llist_v;
		config[i].link_cfg_p =
			(unsigned long)sdev->bm_dma_info.llist_p;
		config[i].dev_id = sdev->bm_dma_info.dma_id;
		config[i].datawidth = WORD_WIDTH;
		config[i].src_step = WORD_STEP;
		config[i].des_step = WORD_STEP;
		config[i].src_addr = sdev->bm_dma_info.src_buf_p;
		config[i].des_addr = sdev->bm_dma_info.dst_buf_p +
			i * BM_DMA_FRAG_LEN * sdev->bm_cnt;
		config[i].fragmens_len = BM_DMA_FRAG_LEN;
		config[i].block_len = BM_DMA_FRAG_LEN;
		config[i].src_blk_step = BM_DMA_BLOCK_STEP;
		config[i].transcation_len = BM_DMA_FRAG_LEN * sdev->bm_cnt;
		config[i].req_mode = TRANS_REQ_MODE;
		config[i].irq_mode = NO_INT;
	}
	config[BM_DMA_LLINK_SIZE - 1].is_end = 2;
	ret = dmaengine_slave_config(sdev->bm_dma_info.dma,
		&config[0].config);
	if (ret < 0) {
		dev_err(sdev->dev.parent, "sprd bm dma config error!\n");
		return ret;
	}

	dma_des = dma_dev->device_prep_dma_memcpy(sdev->bm_dma_info.dma,
			0, 0, 0, DMA_CFG_FLAG | DMA_HARDWARE_FLAG);
	if (!dma_des) {
		dev_err(sdev->dev.parent, "bm dma chn ID %d memcpy failed!\n",
			sdev->bm_dma_info.dma_id);
		return -ENODEV;
	}

	sdev->bm_dma_info.cookie = dmaengine_submit(dma_des);
	return 0;
}

static enum hrtimer_restart sprd_bm_hrtimer_handler(struct hrtimer *timer)
{
	struct sprd_bm_dev *sdev = container_of(timer, struct sprd_bm_dev,
			bm_dma_info.timer);

	sdev->bm_buf_write_cnt++;
	up(&sdev->bm_seam);
	hrtimer_forward_now(timer, ktime_set(0, NSEC_PER_MSEC * 2));
	return HRTIMER_RESTART;
}

static void sprd_bm_hrtimer_start(struct sprd_bm_dev *sdev)
{
	hrtimer_init(&sdev->bm_dma_info.timer,
			CLOCK_REALTIME, HRTIMER_MODE_ABS);
	sdev->bm_dma_info.timer.function = sprd_bm_hrtimer_handler;
	hrtimer_start(&sdev->bm_dma_info.timer,
			ktime_set(0, NSEC_PER_MSEC * 2), HRTIMER_MODE_ABS);
}

static void sprd_bm_reset_and_enable(struct sprd_bm_dev *sdev,
	bool is_enable)
{
	u32 pub0_bit_en, pub0_bit_rst, val0;
	u32 pub1_bit_en = 0, val1 = 0, pub1_bit_rst = 0;

	close_smart_lightsleep();
	val0 = readl_relaxed(sdev->bm_pub0_glb_base
		+ REG_PUB0_APB_BUSMON_CFG);
	if (sdev->double_ddr)
		val1 = readl_relaxed(sdev->bm_pub1_glb_base
			+ REG_PUB1_APB_BUSMON_CFG);

	/* Reset bus monitor */
	pub0_bit_rst = val0 | BM_RESET_VALUE;
	writel_relaxed(pub0_bit_rst, sdev->bm_pub0_glb_base
		+ REG_PUB0_APB_BUSMON_CFG);
	if (sdev->double_ddr) {
		pub1_bit_rst = val1 | BM_RESET_VALUE;
		writel_relaxed(pub1_bit_rst, sdev->bm_pub1_glb_base
			+ REG_PUB1_APB_BUSMON_CFG);
	}
	/* Clear reset bus monitor bit */
	pub0_bit_rst = val0 & (~BM_RESET_VALUE);
	writel_relaxed(pub0_bit_rst, sdev->bm_pub0_glb_base
		+ REG_PUB0_APB_BUSMON_CFG);
	if (sdev->double_ddr) {
		pub1_bit_rst = val1 & (~BM_RESET_VALUE);
		writel_relaxed(pub1_bit_rst, sdev->bm_pub1_glb_base
			+ REG_PUB1_APB_BUSMON_CFG);
	}

	if (is_enable) {
		pub0_bit_en = val0 | BM_ENABLE_VALUE;
		writel_relaxed(pub0_bit_en, sdev->bm_pub0_glb_base
			+ REG_PUB0_APB_BUSMON_CFG);
		if (sdev->double_ddr) {
			pub1_bit_en = val1 | BM_ENABLE_VALUE;
			writel_relaxed(pub1_bit_en, sdev->bm_pub1_glb_base
				+ REG_PUB1_APB_BUSMON_CFG);
		}
	} else {
		pub0_bit_en = val0 & (~BM_ENABLE_VALUE);
		writel_relaxed(pub0_bit_en, sdev->bm_pub0_glb_base
		+ REG_PUB0_APB_BUSMON_CFG);
		if (sdev->double_ddr) {
			pub1_bit_en = val1 & (~BM_ENABLE_VALUE);
			writel_relaxed(pub1_bit_en, sdev->bm_pub1_glb_base
				+ REG_PUB1_APB_BUSMON_CFG);
		}
	}
	open_smart_lightsleep();
}

static void sprd_bm_init(struct sprd_bm_dev *sdev)
{
	u32 chn;
	void * __iomem i;

	close_smart_lightsleep();
	for (chn = 0; chn < sdev->bm_cnt; chn++) {
		for (i = SPRD_BM_CHN_REG(sdev->bm_pub0_base, chn);
			i <= SPRD_BM_CHN_REG(sdev->bm_pub0_base, chn)
			+ sizeof(struct sprd_bm_reg);
			i += 4)
			writel_relaxed(0, i);
		if (!sdev->double_ddr)
			continue;
		for (i = SPRD_BM_CHN_REG(sdev->bm_pub1_base, chn);
			i <= SPRD_BM_CHN_REG(sdev->bm_pub1_base, chn)
			+ sizeof(struct sprd_bm_reg);
			i += 4)
			writel_relaxed(0, i);
	}
	open_smart_lightsleep();
}

static void sprd_bm_chn_enable(struct sprd_bm_dev *sdev, int index)
{
	struct sprd_bm_reg *bm_reg = NULL;
	u32 chn;

	close_smart_lightsleep();
	for (chn = 0; chn < sdev->bm_cnt; chn++) {
		if (!index)
			bm_reg = (struct sprd_bm_reg *)
			SPRD_BM_CHN_REG(sdev->bm_pub0_base, chn);
		else
			bm_reg = (struct sprd_bm_reg *)
			SPRD_BM_CHN_REG(sdev->bm_pub1_base, chn);
		writel_relaxed(readl_relaxed(
			(void * __iomem)&bm_reg->chn_cfg) | 0x1,
			(void * __iomem)&bm_reg->chn_cfg);
	}
	open_smart_lightsleep();
}

static void sprd_bm_chn_int_clr(struct sprd_bm_dev *sdev, int index)
{
	struct sprd_bm_reg *bm_reg = NULL;
	u32 chn, val;

	close_smart_lightsleep();
	for (chn = 0; chn < sdev->bm_cnt; chn++) {
		if (!index)
			bm_reg = (struct sprd_bm_reg *)
			SPRD_BM_CHN_REG(sdev->bm_pub0_base, chn);
		else
			bm_reg = (struct sprd_bm_reg *)
			SPRD_BM_CHN_REG(sdev->bm_pub1_base, chn);
		/* this is the ASIC bug, need to set 0 to INT CLR bit */
		val = readl_relaxed((void * __iomem)&bm_reg->chn_cfg);
		val &= ~(BM_INT_CLR);
		writel_relaxed(val, (void * __iomem)&bm_reg->chn_cfg);
		udelay(10);
		val = readl_relaxed((void * __iomem)&bm_reg->chn_cfg);
		val |= (BM_INT_CLR);
		writel_relaxed(val, (void * __iomem)&bm_reg->chn_cfg);
	}
	open_smart_lightsleep();
}

static void sprd_bm_perf_int_enable(struct sprd_bm_dev *sdev, int index)
{
	struct sprd_bm_reg *bm_reg = NULL;
	u32 chn, val;

	close_smart_lightsleep();
	for (chn = 0; chn < sdev->bm_cnt; chn++) {
		if (!index)
			bm_reg = (struct sprd_bm_reg *)
			SPRD_BM_CHN_REG(sdev->bm_pub0_base, chn);
		else
			bm_reg = (struct sprd_bm_reg *)
			SPRD_BM_CHN_REG(sdev->bm_pub1_base, chn);
		val = readl_relaxed((void * __iomem)&bm_reg->chn_cfg);
		val |= (BM_INT_CLR | BM_PERFOR_INT_EN);
		writel_relaxed(val, (void * __iomem)&bm_reg->chn_cfg);
	}
	open_smart_lightsleep();
}

static void sprd_bm_mode_set(struct sprd_bm_dev *sdev,
	int index, enum sprd_bm_mode mode)
{
	struct sprd_bm_reg *bm_reg = NULL;
	u32 chn, val, up_rbw, up_rl, up_wbw, up_wl;

	close_smart_lightsleep();
	switch (mode) {
	case BM_CIRCLE_MODE:
		for (chn = 0; chn < sdev->bm_cnt; chn++) {
			if (!index)
				bm_reg = (struct sprd_bm_reg *)
				SPRD_BM_CHN_REG(sdev->bm_pub0_base,
				chn);
			else
				bm_reg = (struct sprd_bm_reg *)
				SPRD_BM_CHN_REG(sdev->bm_pub1_base,
				chn);
			val = readl_relaxed
				((void * __iomem)&bm_reg->chn_cfg);
			val |= (BM_CHN_EN | BM_AUTO_MODE_EN | BM_RBW_EN
				| BM_WBW_EN | BM_WLATENCY_EN |
				BM_RLATENCY_EN | BM_PERFOR_REQ_EN);
			writel_relaxed
				(val, (void * __iomem)&bm_reg->chn_cfg);
			writel_relaxed
				(0x100,
				(void * __iomem)&bm_reg->peak_win_len);
		}
		break;
	case BM_NORMAL_MODE:
		for (chn = 0; chn < sdev->bm_cnt; chn++) {
			if (!index)
				bm_reg = (struct sprd_bm_reg *)
				SPRD_BM_CHN_REG(sdev->bm_pub0_base,
				chn);
			else
				bm_reg = (struct sprd_bm_reg *)
				SPRD_BM_CHN_REG(sdev->bm_pub1_base,
				chn);
			val = readl_relaxed
				((void * __iomem)&bm_reg->chn_cfg);
			val |= (BM_CHN_EN | BM_PERFOR_REQ_EN);
			writel_relaxed
				(val, (void * __iomem)&bm_reg->chn_cfg);
			writel_relaxed
				(0x100,
				(void * __iomem)&bm_reg->peak_win_len);
		}
		break;
	case BM_FREQ_MODE:
		for (chn = 0; chn < sdev->bm_cnt; chn++) {
			if (!index)
				bm_reg = (struct sprd_bm_reg *)
				SPRD_BM_CHN_REG(sdev->bm_pub0_base,
				chn);
			else
				bm_reg = (struct sprd_bm_reg *)
				SPRD_BM_CHN_REG(sdev->bm_pub1_base,
				chn);
			up_rbw = readl_relaxed
				((void * __iomem)&bm_reg->f_up_rbw_set);
			up_rl = readl_relaxed
				((void * __iomem)&bm_reg->f_up_rl_set);
			up_wbw = readl_relaxed
				((void * __iomem)&bm_reg->f_up_wbw_set);
			up_wl = readl_relaxed
				((void * __iomem)&bm_reg->f_up_wl_set);
			if ((!up_rbw) && (!up_rl) &&
				(!up_wbw) && (!up_wl)) {
				pr_err(
			"Config error, up freq has not been setted!\n");
				break;
			}
			val = readl_relaxed
				((void * __iomem)&bm_reg->chn_cfg);
			val |= (BM_CHN_EN | BM_RBW_EN | BM_WBW_EN |
				BM_WLATENCY_EN | BM_RLATENCY_EN
				| BM_F_UP_REQ_EN | BM_F_DN_REQ_EN);
			writel_relaxed
				(val, (void * __iomem)&bm_reg->chn_cfg);
			writel_relaxed
				(0x100,
				(void * __iomem)&bm_reg->peak_win_len);
		}
		break;
	default:
		break;
	}
	open_smart_lightsleep();
}

static void sprd_bm_tmr_enable(struct sprd_bm_dev *sdev)
{
	if (sdev->pdata->chip == SHARKL2_CHIP
		|| sdev->pdata->chip == PIKE2_CHIP) {
		regmap_update_bits(sdev->aon_apb, REG_AON_APB_EB2,
		BIT_REG_AON_APB_BST_TMR_EN, BIT_REG_AON_APB_BST_TMR_EN);
	} else {
		regmap_update_bits(sdev->aon_apb, REG_AON_APB_CGM_REG1,
			BIT_AON_APB_CGM_SP_TMR_EN, BIT_AON_APB_CGM_SP_TMR_EN);
		regmap_update_bits(sdev->aon_apb, REG_AON_APB_EB_AON_ADD1,
			BIT_AON_APB_BSM_TMR_EB, BIT_AON_APB_BSM_TMR_EB);
	}
}

static void sprd_bm_tmr_disable(struct sprd_bm_dev *sdev)
{
	if (sdev->pdata->chip == SHARKL2_CHIP
		|| sdev->pdata->chip == PIKE2_CHIP) {
		regmap_update_bits(sdev->aon_apb, REG_AON_APB_EB2,
		BIT_REG_AON_APB_BST_TMR_EN, 0);
	} else {
		regmap_update_bits(sdev->aon_apb, REG_AON_APB_CGM_REG1,
			BIT_AON_APB_CGM_SP_TMR_EN, 0);
		regmap_update_bits(sdev->aon_apb, REG_AON_APB_EB_AON_ADD1,
			BIT_AON_APB_BSM_TMR_EB, 0);
	}
}

static void sprd_bm_tmr_start(struct sprd_bm_dev *sdev, int index)
{
	struct sprd_bm_tmr_reg *bm_tmr_reg =
		(struct sprd_bm_tmr_reg *)sdev->bm_tmr_base;
	u32 val;

	if (!index) {
		val = readl_relaxed
			((void * __iomem)&bm_tmr_reg->tmr_ctrl);
		val |= BM_TMR1_EN;
		writel_relaxed(val, (void * __iomem)&bm_tmr_reg->tmr_ctrl);
	} else {
		val = readl_relaxed
			((void * __iomem)&bm_tmr_reg->tmr_ctrl);
		val |= BM_TMR2_EN;
		writel_relaxed(val, (void * __iomem)&bm_tmr_reg->tmr_ctrl);
	}
}

static void sprd_bm_tmr_stop(struct sprd_bm_dev *sdev, int index)
{
	struct sprd_bm_tmr_reg *bm_tmr_reg =
		(struct sprd_bm_tmr_reg *)sdev->bm_tmr_base;
	u32 val;

	if (!index) {
		val = readl_relaxed
			((void * __iomem)&bm_tmr_reg->tmr_ctrl);
		val &= ~BM_TMR1_EN;
		writel_relaxed(val, (void * __iomem)&bm_tmr_reg->tmr_ctrl);
	} else {
		val = readl_relaxed
			((void * __iomem)&bm_tmr_reg->tmr_ctrl);
		val &= ~BM_TMR2_EN;
		writel_relaxed(val, (void * __iomem)&bm_tmr_reg->tmr_ctrl);
	}
}

static void sprd_bm_tmr_set(struct sprd_bm_dev *sdev,
	int index, u32 val_h, u32 val_l)
{
	struct sprd_bm_tmr_reg *bm_tmr_reg = (struct sprd_bm_tmr_reg *)
		sdev->bm_tmr_base;

	if (!index) {
		writel_relaxed(val_h, (void * __iomem)&bm_tmr_reg->high_len_t1);
		writel_relaxed(val_l, (void * __iomem)&bm_tmr_reg->low_len_t1);
	} else {
		writel_relaxed(val_h, (void * __iomem)&bm_tmr_reg->high_len_t2);
		writel_relaxed(val_l, (void * __iomem)&bm_tmr_reg->low_len_t2);
	}
}

static void sprd_bm_tmr_cnt_clr(struct sprd_bm_dev *sdev, int index)
{
	struct sprd_bm_tmr_reg *bm_tmr_reg = (struct sprd_bm_tmr_reg *)
		sdev->bm_tmr_base;
	u32 val;

	if (!index) {
		val = readl_relaxed
			((void * __iomem)&bm_tmr_reg->tmr_ctrl);
		val |= BM_TMR1_CNT_CLR;
		writel_relaxed(val, (void * __iomem)&bm_tmr_reg->tmr_ctrl);
	} else {
		val = readl_relaxed
			((void * __iomem)&bm_tmr_reg->tmr_ctrl);
		val |= BM_TMR2_CNT_CLR;
		writel_relaxed(val, (void * __iomem)&bm_tmr_reg->tmr_ctrl);
	}
}

static void sprd_bm_tmr_sel_f(struct sprd_bm_dev *sdev,
	enum sprd_bm_tmr_sel mode)
{
	struct sprd_bm_tmr_reg *bm_tmr_reg =
		(struct sprd_bm_tmr_reg *)sdev->bm_tmr_base;

	if (mode == ALL_FROM_TIMER1)
		writel_relaxed(0x0, (void * __iomem)&bm_tmr_reg->tmr_out_sel);
	else
		writel_relaxed(0x1, (void * __iomem)&bm_tmr_reg->tmr_out_sel);
}

static irqreturn_t sprd_bm_isr(int irq_num, void *dev)
{
	struct sprd_bm_reg *bm_reg = NULL;
	struct sprd_bm_dev *sdev = (struct sprd_bm_dev *)dev;
	struct bm_per_info *bm_info;
	ktime_t tv;
	struct rtc_time tm;
	int chn = 0;
	int idx = 0;
	static u32 num;

	spin_lock(&sdev->bm_lock);
	bm_info = (struct bm_per_info *)sdev->per_buf;
	if (bm_info == NULL) {
		pr_err("BM irq ERR, BM dev err, can get perf buf!\n");
		/* discer pub bm who trigger the isr */
		sprd_bm_chn_int_clr(sdev, 0);
		if (sdev->double_ddr)
			sprd_bm_chn_int_clr(sdev, 1);
		spin_unlock(&sdev->bm_lock);
		return IRQ_NONE;
	}

	/* abandon the last irq, because disable bm tmr will trigger the BM. */
	if (!sdev->bm_perf_st) {
		pr_err("BM TMR last trigger, abandon! %d\n", irq_num);
		/* discer pub bm who trigger the isr */
		sprd_bm_chn_int_clr(sdev, 0);
		if (sdev->double_ddr)
			sprd_bm_chn_int_clr(sdev, 1);
		spin_unlock(&sdev->bm_lock);
		return IRQ_NONE;
	}
	close_smart_lightsleep();
	/* count stop time stamp */
	bm_info[sdev->bm_buf_write_cnt].t_stop = jiffies;
	num++;
	bm_info[sdev->bm_buf_write_cnt].count = num;
	for (chn = 0; chn < sdev->bm_cnt; chn++) {
		u32 cfg;
		bm_reg = (struct sprd_bm_reg *)SPRD_BM_CHN_REG
			(sdev->bm_pub0_base, chn);
		cfg = !(readl_relaxed((void * __iomem)&bm_reg->chn_cfg)
			& BM_PERFOR_INT_MASK);
		bm_info[sdev->bm_buf_write_cnt].perf_data[chn][0] = cfg ? 0 :
			readl_relaxed((void * __iomem)&bm_reg->rtrns_in_win);
		bm_info[sdev->bm_buf_write_cnt].perf_data[chn][1] = cfg ? 0 :
			readl_relaxed((void * __iomem)&bm_reg->rbw_in_win);
		bm_info[sdev->bm_buf_write_cnt].perf_data[chn][2] = cfg ? 0 :
			readl_relaxed((void * __iomem)&bm_reg->rl_in_win);
		bm_info[sdev->bm_buf_write_cnt].perf_data[chn][3] = cfg ? 0 :
			readl_relaxed((void * __iomem)&bm_reg->wtrns_in_win);
		bm_info[sdev->bm_buf_write_cnt].perf_data[chn][4] = cfg ? 0 :
			readl_relaxed((void * __iomem)&bm_reg->wbw_in_win);
		bm_info[sdev->bm_buf_write_cnt].perf_data[chn][5] = cfg ? 0 :
			readl_relaxed((void * __iomem)&bm_reg->wl_in_win);
		if (!sdev->double_ddr)
			continue;
		bm_reg = (struct sprd_bm_reg *)
			SPRD_BM_CHN_REG(sdev->bm_pub1_base, chn);
		cfg = !(readl_relaxed((void * __iomem)&bm_reg->chn_cfg)
			& BM_PERFOR_INT_MASK);
		bm_info[sdev->bm_buf_write_cnt].perf_data[chn][0] += cfg ? 0 :
			readl_relaxed((void * __iomem)&bm_reg->rtrns_in_win);
		bm_info[sdev->bm_buf_write_cnt].perf_data[chn][1] += cfg ? 0 :
			readl_relaxed((void * __iomem)&bm_reg->rbw_in_win);
		bm_info[sdev->bm_buf_write_cnt].perf_data[chn][2] += cfg ? 0 :
			readl_relaxed((void * __iomem)&bm_reg->rl_in_win);
		bm_info[sdev->bm_buf_write_cnt].perf_data[chn][3] += cfg ? 0 :
			readl_relaxed((void * __iomem)&bm_reg->wtrns_in_win);
		bm_info[sdev->bm_buf_write_cnt].perf_data[chn][4] += cfg ? 0 :
			readl_relaxed((void * __iomem)&bm_reg->wbw_in_win);
		bm_info[sdev->bm_buf_write_cnt].perf_data[chn][5] += cfg ? 0 :
			readl_relaxed((void * __iomem)&bm_reg->wl_in_win);
	}
	open_smart_lightsleep();
	/* discer pub bm who trigger the isr */
	sprd_bm_chn_int_clr(sdev, 0);
	if (sdev->double_ddr)
		sprd_bm_chn_int_clr(sdev, 1);
	/** accumulate counts **/
	if (internal && bandwidth_data) {
		for (chn = 0; chn < sdev->bm_cnt; chn++) {
			for (idx = 0; idx < 6; idx++)
				bandwidth_data[(chn * 6) + idx] += bm_info[sdev->bm_buf_write_cnt].perf_data[chn][idx];
		}
	}
	if (++sdev->bm_buf_write_cnt == BM_PER_CNT_RECORD_SIZE)
		sdev->bm_buf_write_cnt = 0;
	/* wake up the thread to output log per 4 second */
	if ((sdev->bm_buf_write_cnt == 0) ||
		sdev->bm_buf_write_cnt == (BM_PER_CNT_RECORD_SIZE >> 1))
		up(&sdev->bm_seam);
	/* count start time stamp */
	tv = ktime_get();
	tm = rtc_ktime_to_tm(tv);
	tm.tm_hour = tm.tm_hour < 16 ? (tm.tm_hour + 8) : (tm.tm_hour - 16);
	bm_info[sdev->bm_buf_write_cnt].t_start = jiffies;
	bm_info[sdev->bm_buf_write_cnt].tmp1 = 0;
	bm_info[sdev->bm_buf_write_cnt].tmp2 = 0;
	spin_unlock(&sdev->bm_lock);

	return IRQ_HANDLED;
}

static void sprd_bm_circle_mod(struct sprd_bm_dev *sdev, int ms)
{
	struct sprd_bm_reg *bm_reg = NULL;
	u32 value = 0;

	sprd_bm_tmr_enable(sdev);
	sprd_bm_tmr_stop(sdev, 0);
	sprd_bm_tmr_cnt_clr(sdev, 0);
	if (sdev->bm_dma_info.cir)
		sprd_bm_tmr_set(sdev, 0, ms * BM_TMR_1_MS / 1000, BM_TMR_L_LEN);
	else
		sprd_bm_tmr_set(sdev, 0, ms * BM_TMR_1_MS, BM_TMR_L_LEN);
	bm_reg = (struct sprd_bm_reg *)
	SPRD_BM_CHN_REG(sdev->bm_pub0_base, 0);
	value = readl_relaxed((void * __iomem)&bm_reg->chn_cfg);
	if (!(value & 0x1)) {
		sprd_bm_reset_and_enable(sdev, true);
		sprd_bm_init(sdev);
	}
	/* config pub0 bm */
	sprd_bm_mode_set(sdev, 0, BM_CIRCLE_MODE);
	sprd_bm_chn_int_clr(sdev, 0);
	sprd_bm_perf_int_enable(sdev, 0);
	sprd_bm_chn_enable(sdev, 0);
	/* config pub1 bm */
	if (sdev->double_ddr) {
		sprd_bm_mode_set(sdev, 1, BM_CIRCLE_MODE);
		sprd_bm_chn_int_clr(sdev, 1);
		sprd_bm_perf_int_enable(sdev, 1);
		sprd_bm_chn_enable(sdev, 1);
	}
	/* config both of the bm timer */
	sprd_bm_tmr_sel_f(sdev, TRIGER_BY_EACH);
}
ssize_t sprd_bm_internal_start(void **data)
{
	u32 ret;
	u32 chn;
	u32 idx;
	struct sprd_bm_dev *sdev;
	struct device *dev = &pdev_internal->dev;

	if (!dev)
		return -EINVAL;

	sdev = dev_get_drvdata(dev);

	if (!sdev || sdev->bm_perf_st)
		return -EINVAL;

	/* bm request irq handle */
	ret = devm_request_irq(dev, sdev->bm_pub0_irq,
		sprd_bm_isr, IRQF_TRIGGER_NONE, "bm pub0 irq", sdev);
	if (ret)
		return ret;

	if (bandwidth_data == NULL) {
		bandwidth_data = devm_kzalloc(dev, BM_CHN_BUF_MAX * 6 * sizeof(u64),
			GFP_KERNEL);
		if (!bandwidth_data)
			return -ENOMEM;
	}

	for (chn = 0; chn < sdev->bm_cnt; chn++) {
		for (idx = 0; idx < 6; idx++)
			bandwidth_data[(chn * 6) + idx] = 0;
	}

	/* enable both of the bm */
	sprd_bm_circle_mod(sdev, 10);/*10 ms interval*/
	sdev->bm_perf_st = true;
	internal = true;
	sprd_bm_tmr_start(sdev, 0);
	*data = (void *) bandwidth_data;
	return 0;
}
EXPORT_SYMBOL(sprd_bm_internal_start);

ssize_t sprd_bm_internal_stop(void)
{
	struct sprd_bm_dev *sdev;
	struct device *dev = &pdev_internal->dev;

	if (!dev)
		return -EINVAL;

	sdev = dev_get_drvdata(dev);

	if (!sdev || !sdev->bm_perf_st)
		return -EINVAL;

	/* disable both of the bm */
	sdev->bm_perf_st = false;
	sprd_bm_tmr_stop(sdev, 0);
	sprd_bm_tmr_disable(sdev);
	internal = false;
	devm_free_irq(dev, sdev->bm_pub0_irq, sdev);
	return 0;
}
EXPORT_SYMBOL(sprd_bm_internal_stop);

static int sprd_bm_perf_thread(void *data)
{
	struct sprd_bm_dev *sdev = (struct sprd_bm_dev *)data;
	struct file *bm_perf_file = NULL;
	mm_segment_t old_fs;
	u32 bm_read_cnt = 0;
	int rval;
	size_t dma_buf_size;

	pr_info("Enter BM perf thread!\n");
	while (1) {
		down(&sdev->bm_seam);
		dma_buf_size = BM_DMA_BUF_LEN(sdev->bm_dma_info.cir,
							sdev->bm_cnt);
		if (!bm_perf_file) {
			bm_perf_file = filp_open(BM_LOG_FILE_PATH,
				O_RDWR | O_CREAT | O_TRUNC, 0644);
			if (IS_ERR(bm_perf_file) || !bm_perf_file ||
				!bm_perf_file->f_path.dentry) {
				pr_err("file_open(%s) for create failed\n",
					BM_LOG_FILE_PATH);
				return -ENODEV;
			}
		}
		if (sdev->bm_buf_write_cnt >= (BM_PER_CNT_RECORD_SIZE >> 1))
			bm_read_cnt = 0x0;
		else
			bm_read_cnt = BM_PER_CNT_RECORD_SIZE >> 1;
		old_fs = get_fs();
		set_fs(get_ds());
		if (sdev->bm_dma_info.cir) {
			if (sdev->bm_buf_write_cnt % 2)
				bm_read_cnt = dma_buf_size;
			else
				bm_read_cnt = 0x0;

			rval = vfs_write(bm_perf_file,
				(const char *)sdev->bm_dma_info.dst_buf_v +
				bm_read_cnt, dma_buf_size,
				&bm_perf_file->f_pos);

			set_fs(old_fs);
			/* raw back file write */
			if (sdev->bm_buf_write_cnt >= BM_DMA_LOG_SIZE)
				bm_perf_file->f_pos = 0x0;
			continue;
		} else {
			rval = vfs_write(bm_perf_file,
				(const char *)(sdev->per_buf + bm_read_cnt *
				sizeof(struct bm_per_info)/4),
				sizeof(struct bm_per_info) *
				(BM_PER_CNT_RECORD_SIZE >> 1),
				&bm_perf_file->f_pos);

			set_fs(old_fs);
			/* raw back file write */
			if (bm_perf_file->f_pos >= (sizeof(struct bm_per_info) *
				BM_LOG_FILE_MAX_RECORDS))
				bm_perf_file->f_pos = 0x0;
		}
	}
	filp_close(bm_perf_file, NULL);

	return 0;
}

static ssize_t bandwidth_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct sprd_bm_dev *sdev = dev_get_drvdata(dev);
	u32 ret;
	unsigned long ms;

	ret = kstrtoul(buf, 0, &ms);
	if (ret)
		return -EINVAL;
	if (ms > 0 && !sdev->bm_perf_st) {
		/* bm request irq handle */
		ret = devm_request_irq(dev, sdev->bm_pub0_irq,
			sprd_bm_isr, IRQF_TRIGGER_NONE, "bm pub0 irq", sdev);
		if (ret) {
			dev_err(dev,
			"Unable to request bm pub0 irq!\n");
			return ret;
		}
		/* enable both of the bm */
		sprd_bm_circle_mod(sdev, ms);
		sdev->bm_perf_st = true;
		sprd_bm_tmr_start(sdev, 0);
		pr_info("bm bandwidth mode enable!!!\n");
	} else if (sdev->bm_perf_st) {
		/* disable both of the bm */
		sdev->bm_perf_st = false;
		sprd_bm_tmr_stop(sdev, 0);
		sprd_bm_tmr_disable(sdev);
		devm_free_irq(dev, sdev->bm_pub0_irq, sdev);
		pr_info("bm bandwidth mode disable!!!\n");
	}
	return strnlen(buf, count);
}

static ssize_t bandwidth_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	struct sprd_bm_dev *sdev = dev_get_drvdata(dev);

	if (sdev->bm_perf_st) {
		pr_info("bm bandwidth mode enable!!!\n");
		return sprintf(buf, "bm bandwidth mode enable!\n");
	} else {
		pr_info("bm bandwidth mode disable!!!\n");
		return sprintf(buf, "bm bandwidth mode disable!\n");
	}
}
static DEVICE_ATTR(bandwidth, 0660, bandwidth_show,
	bandwidth_store);

static ssize_t dma_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct sprd_bm_dev *sdev = dev_get_drvdata(dev);
	int ret;
	u32 dma_mode;

	ret = kstrtou32(buf, 0, &dma_mode);
	if (ret)
		return -EINVAL;
	if (dma_mode && sdev->bm_dma_info.cir == 0) {
		sprd_bm_chn_int_clr(sdev, 0);
		sdev->bm_dma_info.cir = USEC_PER_MSEC * 2 / BM_CHN_DEF_WINLEN;
		ret = sprd_bm_dma_request(sdev->dev.parent, sdev);
		if (ret < 0)
			return ret;
		ret = sprd_bm_dma_config(sdev);
		if (ret < 0) {
			dma_release_channel(sdev->bm_dma_info.dma);
			return ret;
		}
		sprd_bm_hrtimer_start(sdev);
		sprd_bm_circle_mod(sdev, BM_CHN_DEF_WINLEN);
		sprd_bm_tmr_start(sdev, 0);
		dev_dbg(dev, "bm dma mode enable!!!\n");
	} else if (sdev->bm_dma_info.cir > 0) {
		sdev->bm_dma_info.cir = 0;
		dma_release_channel(sdev->bm_dma_info.dma);
		sprd_bm_tmr_stop(sdev, 0);
		sprd_bm_tmr_disable(sdev);
		hrtimer_cancel(&sdev->bm_dma_info.timer);
		dev_dbg(dev, "bm dma mode disable!!!\n");
	}
	return strnlen(buf, count);
}

static ssize_t dma_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sprd_bm_dev *sdev = dev_get_drvdata(dev);

	if (sdev->bm_dma_info.cir) {
		dev_dbg(dev, "bm dma mode enable!\n");
	} else {
		dev_err(dev, "bm dma mode disable!\n");
		return sprintf(buf, "bm dma mode disable!\n");
	}

	return sprintf(buf, "bm dma mode enable!\n");
}

static DEVICE_ATTR_RW(dma_mode);

static ssize_t chn_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	struct sprd_bm_dev *sdev = dev_get_drvdata(dev);
	int chn, cnt = 0;

	for (chn = 0; chn < sdev->bm_cnt; chn++)
		cnt += sprintf(buf + cnt, "%s\n", sdev->name_list[chn]);
	return cnt;
}
static DEVICE_ATTR_RO(chn);

static ssize_t data_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	struct sprd_bm_dev *sdev = dev_get_drvdata(dev);
	struct bm_per_info *bm_info;
	int n;
	int cnt;

	bm_info = (struct bm_per_info *)sdev->per_buf;
	for (n = 1; n <= BM_DATA_COUNT; n++) {
		cnt = sdev->bm_buf_write_cnt - n;
		cnt = (cnt + BM_PER_CNT_RECORD_SIZE) % BM_PER_CNT_RECORD_SIZE;
		memcpy(buf + (n - 1) * sizeof(bm_info[cnt]),
			(void *)&bm_info[cnt], sizeof(bm_info[cnt]));
	}
	return BM_DATA_COUNT * sizeof(bm_info[cnt]);
}
static DEVICE_ATTR_RO(data);

static struct attribute *sprd_bm_attrs[] = {
	&dev_attr_bandwidth.attr,
	&dev_attr_chn.attr,
	&dev_attr_data.attr,
	NULL,
};

static struct attribute *bm_dma_attrs[] = {
	&dev_attr_dma_mode.attr,
	NULL,
};

static struct attribute_group bm_dma_group = {
	.attrs = bm_dma_attrs,
	.name = "bm-performance",
};

static struct attribute_group perf_attr_group = {
	.attrs = sprd_bm_attrs,
	.name = "bm-performance",
};

static int sprd_bm_probe(struct platform_device *pdev)
{
	struct sprd_bm_dev *sdev = NULL;
	struct resource *res;
	struct device_node *np = NULL;
	u32 *var_array;
	u32 double_ddr_v;
	int count, i;
	bool double_ddr = false;
	void __iomem *bm_pub0_base = NULL;
	void __iomem *bm_pub1_base = NULL;
	void __iomem *bm_pub0_glb_base = NULL;
	void __iomem *bm_pub1_glb_base = NULL;
	void __iomem *bm_tmr_base = NULL;
	const struct perf_data *bm_data;
	int bm_pub0_irq, ret;
	dma_addr_t src_buf_p;

	bm_data = of_device_get_match_data(&pdev->dev);
	if (!bm_data || bm_data->channel_num > BM_CHN_BUF_MAX)
		return -EINVAL;

	if (bm_data->chip == WHALE2_CHIP) {
		np = of_find_node_by_name(NULL, "memory");
		if (!np) {
			dev_err(&pdev->dev, "not find memory node!\n");
			double_ddr = false;
			goto begin;
		}
		count = of_property_count_u32_elems(np, "reg");
		var_array = devm_kzalloc(&pdev->dev,
			count * sizeof(u32), GFP_KERNEL);
		if (!var_array)
			return -ENOMEM;
		if (of_property_read_u32_array(np, "reg", var_array, count)) {
			dev_err(&pdev->dev, "read mem info failed!\n");
			return -ENODEV;
		}
		for (i = 0; i < count; i = i + 2)
			if (var_array[i] > 0) {
				double_ddr = true;
				break;
			}
	}
	if (bm_data->chip == IWHALE2_CHIP) {
		if (of_property_read_u32_index(pdev->dev.of_node,
					       "sprd,doubleddr",
					       0, &double_ddr_v)) {
			dev_err(&pdev->dev, "read ddr-interleaved failed!\n");
			return -ENODEV;
		}
		if (double_ddr_v == 1)
			double_ddr = true;
	}
begin:
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "sprd pub0 bm get io resource failed!\n");
		return -ENODEV;
	}
	src_buf_p = res->start;
	bm_pub0_base = devm_ioremap_nocache(&pdev->dev, res->start,
		resource_size(res));
	if (!bm_pub0_base) {
		dev_err(&pdev->dev, "sprd pub0 bm get base address failed!\n");
		return -ENODEV;
	}
	if (double_ddr) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (!res) {
			dev_err(&pdev->dev,
				"sprd pub1 bm get io resource failed!\n");
			return -ENODEV;
		}
		bm_pub1_base = devm_ioremap_nocache(&pdev->dev, res->start,
			resource_size(res));
		if (!bm_pub1_base) {
			dev_err(&pdev->dev,
				"sprd pub1 bm get base address failed!\n");
			return -ENODEV;
		}
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dev_err(&pdev->dev, "sprd bm timer get io resource failed!\n");
		return -ENODEV;
	}
	bm_tmr_base = devm_ioremap_nocache(&pdev->dev, res->start,
		resource_size(res));
	if (!bm_tmr_base) {
		dev_err(&pdev->dev, "sprd bm timer get base address failed!\n");
		return -ENODEV;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (!res) {
		dev_err(&pdev->dev, "sprd bm pub0 glb get io resource failed!\n");
		return -ENODEV;
	}
	bm_pub0_glb_base = devm_ioremap_nocache(&pdev->dev, res->start,
		resource_size(res));
	if (!bm_pub0_glb_base) {
		dev_err(&pdev->dev, "sprd bm pub0 glb get base address failed!\n");
		return -ENODEV;
	}
	if (double_ddr) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 4);
		if (!res) {
			dev_err(&pdev->dev,
				"sprd bm pub1 glb get io resource failed!\n");
			return -ENODEV;
		}
		bm_pub1_glb_base = devm_ioremap_nocache(&pdev->dev, res->start,
			resource_size(res));
		if (!bm_pub1_glb_base) {
			dev_err(&pdev->dev,
				"sprd bm pub1 glb get base address failed!\n");
			return -ENODEV;
		}
	}
	/* get bm interrupts */
	bm_pub0_irq = platform_get_irq(pdev, 0);
	if (bm_pub0_irq < 0) {
		dev_err(&pdev->dev, "Can't get the pub0 bm irq number!\n");
		return -EIO;
	}
	/* bm device momery alloc */
	sdev = devm_kzalloc(&pdev->dev, sizeof(*sdev), GFP_KERNEL);
	if (!sdev)
		return -ENOMEM;

	sdev->aon_apb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
				"sprd,syscon-aon-glb");
	if (IS_ERR(sdev->aon_apb)) {
		pr_err("sprd bm probe failed!\n");
		return -ENODEV;
	}
	if (of_property_read_u32(pdev->dev.of_node,
		"sprd,bm-dma", &sdev->bm_dma_info.dma_id))
		dev_warn(&pdev->dev, "get dma id fail, disable dma mode!\n");
	sdev->dev.parent = &pdev->dev;
	sdev->bm_pub0_base = bm_pub0_base;
	sdev->bm_pub1_base = bm_pub1_base;
	sdev->bm_tmr_base = bm_tmr_base;
	sdev->bm_pub0_irq = bm_pub0_irq;
	sdev->bm_pub0_glb_base = bm_pub0_glb_base;
	sdev->bm_pub1_glb_base = bm_pub1_glb_base;
	sdev->bm_cnt = bm_data->channel_num;
	sdev->name_list = bm_data->name_list;
	sdev->double_ddr = double_ddr;
	sdev->pdata = bm_data;
	sdev->bm_dma_info.src_buf_p = BM_REG_IN_WINDOW(src_buf_p);
	spin_lock_init(&sdev->bm_lock);
	sema_init(&sdev->bm_seam, 0);
	dev_set_name(&pdev->dev, "bm-performance");
	ret = sysfs_create_group(&pdev->dev.kobj, &perf_attr_group);
	if (ret)
		return ret;
	if (sdev->bm_dma_info.dma_id) {
		ret = sysfs_merge_group(&pdev->dev.kobj, &bm_dma_group);
		if (ret)
			goto err_merge;
	}
	dev_set_drvdata(&pdev->dev, sdev);
	sdev->bm_thl = kthread_create(sprd_bm_perf_thread, sdev, "bm_perf");
	if (IS_ERR(sdev->bm_thl)) {
		dev_err(&pdev->dev, "Failed to create kthread: bm perf\n");
		sysfs_remove_group(&pdev->dev.kobj, &perf_attr_group);
		ret = PTR_ERR(sdev->bm_thl);
		goto err_kthread;
	}
	wake_up_process(sdev->bm_thl);
	sdev->per_buf = devm_kzalloc(&pdev->dev, BM_PER_CNT_BUF_SIZE,
		GFP_KERNEL);
	if (!sdev->per_buf) {
		ret = -ENOMEM;
		goto err_kzalloc;
	}
	/* save platform_device location */
	pdev_internal = pdev;
	pr_info("sprd bm probe done!\n");
	return 0;

err_kzalloc:
	kthread_stop(sdev->bm_thl);
err_kthread:
	sysfs_unmerge_group(&pdev->dev.kobj, &bm_dma_group);
err_merge:
	sysfs_remove_group(&pdev->dev.kobj, &perf_attr_group);
	return ret;
}

static int sprd_bm_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &perf_attr_group);
	return 0;
}

static const struct of_device_id sprd_bm_match[] = {
	{ .compatible = "sprd,bm-perf-whale2", .data = &whale2_perfdata },
	{ .compatible = "sprd,bm-perf-whale", .data = &whale_perfdata },
	{ .compatible = "sprd,bm-perf-iwhale2", .data = &iwhale2_perfdata },
	{ .compatible = "sprd,bm-perf-isharkl2", .data = &isharkl2_perfdata },
	{ .compatible = "sprd,bm-perf-sharkl2", .data = &sharkl2_perfdata },
	{ .compatible = "sprd,bm-perf-sharklj1", .data = &sharklj1_perfdata },
	{ .compatible = "sprd,bm-perf-pike2", .data = &pike2_perfdata },
	{ },
};

static struct platform_driver sprd_bm_driver = {
	.probe    = sprd_bm_probe,
	.remove   = sprd_bm_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd_bm_perf",
		.of_match_table = sprd_bm_match,
	},
};

module_platform_driver(sprd_bm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aiden Cheng<aiden.cheng@spreadtrum.com>");
MODULE_DESCRIPTION("Spreadtrum platform Performance driver");
