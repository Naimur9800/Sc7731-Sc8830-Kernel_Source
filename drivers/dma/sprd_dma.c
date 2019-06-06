/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_dma.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/dma/sprd_dma.h>
#include <linux/pm_runtime.h>
#include "dmaengine.h"

/* #define SPRD_DMA_DEBUG */
#ifdef SPRD_DMA_DEBUG
#define pr_dma(fmt, ...)	pr_emerg(fmt, ##__VA_ARGS__)
#else
#define pr_dma(fmt, ...)
#endif

#define DMA_TAG				"SPRD_DMA "
#define SPRD_DMA_DESCRIPTORS		(64)
#define SPRD_DMA_CHN_OFFSET			(0x40)
#define SPRD_DMA_MEMCPY_MIN_SIZE	(64)
#define SPRD_DMA_CFG_COUNT			(SPRD_DMA_DESCRIPTORS)
#define SPRD_DMA_REQ_CID(base, uid)	\
				((unsigned long)base + \
				0x2000 + 0x4 * ((uid) - 1))

/* dma channel register definition */
struct sprd_dma_chn_reg {
	u32 pause;
	u32 req;
	u32 cfg;
	u32 intc;
	u32 src_addr;
	u32 des_addr;
	u32 frg_len;
	u32 blk_len;
	/* only full chn have following regs */
	u32 trsc_len;
	u32 trsf_step;
	u32 wrap_ptr;
	u32 wrap_to;
	u32 llist_ptr;
	u32 frg_step;
	u32 src_blk_step;
	u32 des_blk_step;
};

/* dma request description */
struct sprd_dma_desc {
	struct dma_async_tx_descriptor	desc;
	struct sprd_dma_chn_reg		*dma_chn_reg;
	struct list_head		node;
	struct list_head		next_node;
	enum dma_flags			dma_flags;
	int				done;
	int				cycle;
};

/* dma channel description */
struct sprd_dma_chn {
	struct dma_chan			chan;
	struct list_head		free;
	struct list_head		prepared;
	struct list_head		queued;
	struct list_head		active;
	struct list_head		completed;
	spinlock_t			chn_lock;
	int				chan_num;
	u32				dev_id;
	enum dma_chn_status		chan_status;
	void __iomem			*dma_chn_base;
	void __iomem			*dma_desc;
	dma_addr_t			dma_desc_paddr;
	enum dma_chn_type		chn_type;
	enum request_mode		re_mode;
	int				irq_handle_enable;
};

/* DMA controller type */
enum sprd_dma_controller_type {
	UNKNOWN_DMA,
	AP_DMA_V0_0,
	AP_DMA_V1_0,
	AP_DMA_V2_0,
	AP_DMA_V3_0,
	AP_DMA_V4_0,
	AON_DMA_V0_0,
	AON_DMA_V1_0,
	AON_DMA_V2_0,
	AON_DMA_V3_0,
	AGCP_DMA_V0_0,
	AGCP_DMA_V1_0,
	AGCP_DMA_V2_0,
	AGCP_DMA_V3_0,
};

/* dma device */
struct sprd_dma_dev {
	enum sprd_dma_controller_type	dma_type;
	struct dma_device		dma_dev;
	spinlock_t			dma_lock;
	void __iomem			*dma_glb_base;
	struct regmap			*dma_glb_syscon;
	struct clk			*clk;
	struct clk			*ashb_clk;
	int				irq;
	struct tasklet_struct		tasklet;
	struct kmem_cache		*dma_desc_node_cachep;
	u32				dma_chn_cnt;
	u32				full_chn_cnt;
	u32				std_chn_cnt;
	struct sprd_dma_chn		channels[0];
};

struct dma_cfg_group_t {
	struct semaphore		cfg_sema;
	int				dma_cfg_cnt;
	struct sprd_dma_cfg		dma_cfg[SPRD_DMA_CFG_COUNT];
};

struct dma_cfg_group_t dma_cfg_group;

static const struct of_device_id sprd_dma_match[];
bool sprd_dma_filter_fn(struct dma_chan *chan, void *param);
static struct of_dma_filter_info sprd_dma_info = {
	.filter_fn = sprd_dma_filter_fn,
};

static inline struct sprd_dma_chn *to_sprd_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct sprd_dma_chn, chan);
}

static inline struct sprd_dma_dev *to_sprd_dma_dev(struct dma_chan *c)
{
	struct sprd_dma_chn *mchan = to_sprd_dma_chan(c);

	return container_of(mchan, struct sprd_dma_dev, channels[c->chan_id]);
}

static inline struct sprd_dma_desc *
to_sprd_dma_desc(struct dma_async_tx_descriptor *tx)
{
	return container_of(tx, struct sprd_dma_desc, desc);
}

/* dma debug function */
static int sprd_dma_cfg_check_regs(void __iomem *dma_reg_addr)
{
#ifdef SPRD_DMA_DEBUG
	print_hex_dump(KERN_EMERG, "DMA reg: ", DUMP_PREFIX_ADDRESS,
			   4, 4, dma_reg_addr, 64, true);
#endif
	return 0;
}

static void sprd_dma_glb_state_check(struct sprd_dma_chn *mchan)
{
	struct sprd_dma_dev *sdev = to_sprd_dma_dev(&mchan->chan);
	volatile struct sprd_dma_glb_reg *dma_glb_reg = sdev->dma_glb_base;

	pr_warn(DMA_TAG "Gbl dbg reg: pause=0x%x, raw_sts=0x%x, msk_sts=0x%x,\n"
		"req_sts=0x%x, en_sts=0x%x, debug sts=0x%x, arb_sel_sts=0x%x!\n",
		dma_glb_reg->pause, dma_glb_reg->int_raw_sts,
		dma_glb_reg->int_msk_sts, dma_glb_reg->req_sts,
		dma_glb_reg->en_sts, dma_glb_reg->debug_sts,
		dma_glb_reg->arb_sel_sts);
}

static int sprd_dma_enable(struct sprd_dma_dev *sdev)
{
	clk_prepare_enable(sdev->clk);

	if (!IS_ERR(sdev->ashb_clk))
		clk_prepare_enable(sdev->ashb_clk);

	return 0;
}

static int sprd_dma_disable(struct sprd_dma_dev *sdev)
{
	clk_disable_unprepare(sdev->clk);

	if (!IS_ERR(sdev->ashb_clk))
		clk_disable_unprepare(sdev->ashb_clk);

	return 0;
}

static void sprd_dma_softreset(struct sprd_dma_dev *sdev)
{
	unsigned int reg, mask;
	enum sprd_dma_controller_type type = sdev->dma_type;
	struct regmap *syscon = sdev->dma_glb_syscon;

	switch (type) {
	case AP_DMA_V0_0:
	case AP_DMA_V1_0:
	case AP_DMA_V2_0:
	case AP_DMA_V3_0:
		reg = REG_AP_AHB_AHB_RST;
		mask = BIT_AP_AHB_DMA_SOFT_RST;
		break;
	/*
	 * reset bit do not be separated in normal and trusty sys,
	 * do not set this bit to avoid competition.
	 */
	case AP_DMA_V4_0:
		return;
	case AON_DMA_V0_0:
	case AON_DMA_V1_0:
	case AON_DMA_V2_0:
	case AON_DMA_V3_0:
		reg = REG_AON_APB_APB_RST1;
		mask = BIT_AON_APB_AON_DMA_SOFT_RST;
		break;
	case AGCP_DMA_V0_0:
	case AGCP_DMA_V1_0:
	case AGCP_DMA_V2_0:
	case AGCP_DMA_V3_0:
		reg = REG_AGCP_AHB_MODULE_DMA_RST;
		mask = BIT_AGCP_AP_DMA_SOFT_RST;
		break;
	default:
		pr_err(DMA_TAG "Can not identify the dma controller type!\n");
		return;
	}

	if (reg == REG_AP_AHB_AHB_EB) {
		regmap_hwlock_update_bits(syscon, reg, mask, mask);
		udelay(1);
		regmap_hwlock_update_bits(syscon, reg, mask, ~mask);
	} else {
		regmap_update_bits(syscon, reg, mask, mask);
		udelay(1);
		regmap_update_bits(syscon, reg, mask, ~mask);
	}

}

static int sprd_dma_int_enable(struct sprd_dma_dev *sdev)
{
	unsigned int reg, mask, val;
	enum sprd_dma_controller_type type = sdev->dma_type;
	struct regmap *syscon = sdev->dma_glb_syscon;
	int ret;

	switch (type) {
	case AP_DMA_V0_0:
	case AP_DMA_V1_0:
	case AP_DMA_V2_0:
	case AP_DMA_V3_0:
	case AP_DMA_V4_0:
	case AGCP_DMA_V0_0:
	case AGCP_DMA_V1_0:
	case AGCP_DMA_V2_0:
	case AGCP_DMA_V3_0:
	/* this version does not configure sub sys int */
	case AON_DMA_V3_0:
		return 0;
	case AON_DMA_V0_0:
	case AON_DMA_V2_0:
		reg = REG_AON_APB_AON_DMA_INT_EN;
		mask = BIT_AON_APB_AON_DMA_INT_AP_EN;
		break;
	default:
		pr_err(DMA_TAG "Can not identify the dma controller type!\n");
		return -EINVAL;
	}

	ret = regmap_read(syscon, reg , &val);
	if (ret)
		return ret;

	if (!(val & mask))
		regmap_update_bits(syscon, reg, mask, mask);

	return 0;
}

static int sprd_dma_int_disable(struct sprd_dma_dev *sdev)
{
	unsigned int reg, mask, val;
	enum sprd_dma_controller_type type = sdev->dma_type;
	struct regmap *syscon = sdev->dma_glb_syscon;
	int ret;

	switch (type) {
	case AP_DMA_V0_0:
	case AP_DMA_V1_0:
	case AP_DMA_V2_0:
	case AP_DMA_V3_0:
	case AP_DMA_V4_0:
	case AGCP_DMA_V0_0:
	case AGCP_DMA_V1_0:
	case AGCP_DMA_V2_0:
	case AGCP_DMA_V3_0:
	/* this version does not configure sub sys int */
	case AON_DMA_V3_0:
		return 0;
	case AON_DMA_V0_0:
	case AON_DMA_V2_0:
		reg = REG_AON_APB_AON_DMA_INT_EN;
		mask = BIT_AON_APB_AON_DMA_INT_AP_EN;
		break;
	default:
		pr_err(DMA_TAG "Can not identify the dma controller type!\n");
		return -EINVAL;
	}

	ret = regmap_read(syscon, reg , &val);
	if (ret)
		return ret;

	if (val & mask)
		regmap_update_bits(syscon, reg, mask, ~mask);

	return 0;
}

static void
sprd_dma_set_uid(struct sprd_dma_dev *sdev,
		 struct sprd_dma_chn *mchan,
		 u32 dev_id)
{
	if (DMA_UID_SOFTWARE != dev_id) {
		writel_relaxed((mchan->chan_num + 1),
				(void __iomem *)SPRD_DMA_REQ_CID(
				sdev->dma_glb_base, dev_id));
	}
}

static void
sprd_dma_unset_uid(struct sprd_dma_dev *sdev,
		   struct sprd_dma_chn *mchan,
		   u32 dev_id)
{
	if (DMA_UID_SOFTWARE != dev_id) {
		writel_relaxed(0x0,
			       (void __iomem *)SPRD_DMA_REQ_CID(
			       sdev->dma_glb_base,
			       dev_id));
	}
}

static void __inline sprd_dma_int_clr(struct sprd_dma_chn *mchan)
{
	volatile struct sprd_dma_chn_reg *dma_reg =
			(struct sprd_dma_chn_reg *)mchan->dma_chn_base;

	dma_reg->intc |= 0x1f << 24;
}

static void __inline sprd_dma_int_dis(struct sprd_dma_chn *mchan)
{
	volatile struct sprd_dma_chn_reg *dma_reg =
			(struct sprd_dma_chn_reg *)mchan->dma_chn_base;

	dma_reg->intc |= 0x1f << 24;
	dma_reg->intc &= ~0x1f;
}

static void __inline sprd_dma_chn_enable(struct sprd_dma_chn *mchan)
{
	volatile struct sprd_dma_chn_reg *dma_reg =
			(struct sprd_dma_chn_reg *)mchan->dma_chn_base;

	dma_reg->cfg |= 0x1;
}

static void __inline sprd_dma_soft_request(struct sprd_dma_chn *mchan)
{
	volatile struct sprd_dma_chn_reg *dma_reg =
			(struct sprd_dma_chn_reg *)mchan->dma_chn_base;

	dma_reg->req |= 0x1;
}

static void sprd_dma_stop_and_disable(struct sprd_dma_chn *mchan)
{
	volatile struct sprd_dma_chn_reg *dma_reg =
			(struct sprd_dma_chn_reg *)mchan->dma_chn_base;
	struct sprd_dma_dev *sdev = to_sprd_dma_dev(&mchan->chan);
	enum sprd_dma_controller_type dma_type = sdev->dma_type;
	u32 timeout = 0x2000;
	unsigned int pause_bit;

	if (!(dma_reg->cfg & 0x1))
		return;

	dma_reg->pause |= 0x1;

	/* fixme, need to deal with timeout */
	if ((dma_type == AP_DMA_V0_0) || (dma_type == AON_DMA_V0_0)
		|| (dma_type == AGCP_DMA_V0_0))
		pause_bit = DMA_OLD_GLB_PAUSE;
	else
		pause_bit = DMA_NEW_GLB_PAUSE;

	while (!(dma_reg->pause & pause_bit)) {
		if (--timeout == 0) {
			sprd_dma_glb_state_check(mchan);
			sprd_dma_softreset(sdev);
			pr_err(DMA_TAG "pause DMA[%d] failed!\n",
				mchan->chan_num);
			break;
		}
		cpu_relax();
	}

	dma_reg->cfg &= ~0x1;
	dma_reg->pause = 0x0;
}

/* get dma source reg addr */
static unsigned long sprd_dma_get_src_addr(struct dma_chan *dma_chn)
{
	struct sprd_dma_chn *mchan = to_sprd_dma_chan(dma_chn);
	unsigned long addr = (unsigned long)mchan->dma_chn_base + 0x10;
	unsigned long addr_val;

	addr_val = readl_relaxed((void __iomem *)addr);
	addr_val |= (readl_relaxed((void __iomem *)(addr + 0x28))
		& 0xF0000000) << 4;
	return addr_val;
}

/* get dma dest reg addr */
static unsigned long sprd_dma_get_dst_addr(struct dma_chan *dma_chn)
{
	struct sprd_dma_chn *mchan = to_sprd_dma_chan(dma_chn);
	unsigned long addr = (unsigned long)mchan->dma_chn_base + 0x14;
	unsigned long addr_val;

	addr_val = readl_relaxed((void __iomem *)addr);
	addr_val |= (readl_relaxed((void __iomem *)(addr + 0x2C))
		& 0xF0000000) << 4;
	return addr_val;
}

static int
sprd_dma_config(struct dma_chan *chan,
		struct sprd_dma_desc *mdesc,
		struct sprd_dma_cfg *cfg_list,
		struct sprd_dma_chn_reg *dma_reg_addr,
		enum config_type type)
{
	volatile struct sprd_dma_chn_reg *dma_reg;
	struct sprd_dma_chn *mchan = to_sprd_dma_chan(chan);
	struct sprd_dma_cfg *dma_cfg_tmp = cfg_list;
	struct sprd_dma_dev *sdev = to_sprd_dma_dev(&mchan->chan);
	enum sprd_dma_controller_type dma_type = sdev->dma_type;
	u32 fix_mode = 0, llist_en = 0, wrap_en = 0;
	u32 list_end = 0, fix_en = 0, irq_mode = 0, wrap_mode = 0;
	int chn_type;

	/* check dma fix mode */
	if (dma_cfg_tmp->src_step != 0 && dma_cfg_tmp->des_step != 0) {
		fix_en = 0x0;
	} else {
		if ((dma_cfg_tmp->src_step | dma_cfg_tmp->des_step) == 0) {
			fix_en = 0x0;
		} else {
			fix_en = 0x1;
			if (dma_cfg_tmp->src_step)
				fix_mode = 0x1;
			else
				fix_mode = 0x0;
		}
	}

	/* check dma wrap mode */
	if (dma_cfg_tmp->wrap_ptr && dma_cfg_tmp->wrap_to) {
		wrap_en = 0x1;
		if (dma_cfg_tmp->wrap_to == dma_cfg_tmp->src_addr) {
			wrap_mode = 0x0;
		} else {
			if (dma_cfg_tmp->wrap_to == dma_cfg_tmp->des_addr)
				wrap_mode = 0x1;
			else {
				pr_err(DMA_TAG "check wrap config failed\n");
				return -EINVAL;
			}
		}
	}

	/* linklist configuration */
	if (dma_cfg_tmp->linklist_ptr) {
		llist_en = 0x1;
		if (dma_cfg_tmp->is_end == 1)
			list_end = 0x1;
		else
			list_end = 0;
	}

	chn_type = mchan->chn_type;
	irq_mode = dma_cfg_tmp->irq_mode;

	/* check dma mode */
	if ((chn_type == STANDARD_DMA) &&
		(irq_mode == TRANS_DONE || irq_mode == LIST_DONE)) {
		pr_err(DMA_TAG "Irq type isn't compatible with channel type!");
		return -EINVAL;
	}

	/* aligned src/des step with datawith, both of their uint is byte */
	if (!IS_ALIGNED(dma_cfg_tmp->src_step, (1 << dma_cfg_tmp->datawidth))) {
		pr_err(DMA_TAG "Source step is not aligned!");
		return -EINVAL;
	}

	if (!IS_ALIGNED(dma_cfg_tmp->des_step, (1 << dma_cfg_tmp->datawidth))) {
		pr_err(DMA_TAG "Destination step is not aligned!");
		return -EINVAL;
	}

	if (!mchan->dev_id)
		mchan->dev_id = dma_cfg_tmp->dev_id;

	if (type == CONFIG_DESC)
		dma_reg = mdesc->dma_chn_reg;
	else if (type == CONFIG_LINKLIST)
		dma_reg = dma_reg_addr;
	else {
		pr_err(DMA_TAG "check type failed, tpye : %d\n", type);
		return -EINVAL;
	}

	dma_reg->pause = 0x0;
	dma_reg->req = 0x0;

	/* set default priority = 1 */
	dma_reg->cfg = dma_cfg_tmp->chn_pri << CHN_PRIORITY_OFFSET |
	    llist_en << LLIST_EN_OFFSET |
	    DMA_DONOT_WAIT_BDONE << CHN_WAIT_BDONE;

	if (wrap_en) {
		if ((dma_type == AP_DMA_V1_0) || (dma_type == AON_DMA_V1_0)
			|| (dma_type == AGCP_DMA_V1_0))
			dma_reg->wrap_ptr = (dma_cfg_tmp->wrap_ptr - 1)
				& WRAP_DATA_MASK;
		else
			dma_reg->wrap_ptr = dma_cfg_tmp->wrap_ptr
				& WRAP_DATA_MASK;
		/* the hight 4 bits are reserved in 32 bit DMA SoC */
		dma_reg->wrap_to = dma_cfg_tmp->wrap_to & WRAP_DATA_MASK;
	}

	/* src and des addr */
	dma_reg->wrap_ptr |= (u32)((dma_cfg_tmp->src_addr >> 4)
		& SRC_DEST_HADDR_MASK);
	dma_reg->wrap_to |= (u32)((dma_cfg_tmp->des_addr >> 4)
		& SRC_DEST_HADDR_MASK);

	dma_reg->src_addr = (u32)dma_cfg_tmp->src_addr;
	dma_reg->des_addr = (u32)dma_cfg_tmp->des_addr;

	/* frag len */
	dma_reg->frg_len =
	    (dma_cfg_tmp->datawidth << SRC_DATAWIDTH_OFFSET) |
	    (dma_cfg_tmp->datawidth << DES_DATAWIDTH_OFFSET) |
	    (dma_cfg_tmp->swt_mode << SWT_MODE_OFFSET) |
	    (dma_cfg_tmp->req_mode << REQ_MODE_OFFSET) |
	    (wrap_mode << ADDR_WRAP_SEL_OFFSET) |
	    (wrap_en << ADDR_WRAP_EN_OFFSET) |
	    (fix_mode << ADDR_FIX_SEL_OFFSET) |
	    (fix_en << ADDR_FIX_SEL_EN) |
	    (list_end << LLIST_END_OFFSET) |
		(dma_cfg_tmp->fragmens_len & FRG_LEN_MASK);

	/* blk len */
	dma_reg->blk_len = dma_cfg_tmp->block_len & BLK_LEN_MASK;

	/* set interrupt type*/
	if (type == CONFIG_DESC) {
		if (irq_mode == NO_INT)
			mchan->irq_handle_enable = 0;
		else
			mchan->irq_handle_enable = 1;

		dma_reg->intc &= ~0x1f;
		dma_reg->intc |= 0x1 << 4;
		switch (irq_mode) {
		case NO_INT:
			break;
		case FRAG_DONE:
			dma_reg->intc |= 0x1;
			break;
		case BLK_DONE:
			dma_reg->intc |= 0x2;
			break;
		case BLOCK_FRAG_DONE:
			dma_reg->intc |= 0x3;
			break;
		case TRANS_DONE:
			dma_reg->intc |= 0x4;
			break;
		case TRANS_FRAG_DONE:
			dma_reg->intc |= 0x5;
			break;
		case TRANS_BLOCK_DONE:
			dma_reg->intc |= 0x6;
			break;
		case LIST_DONE:
			dma_reg->intc |= 0x8;
			break;
		case CONFIG_ERR:
			dma_reg->intc |= 0x10;
			break;
		default:
			pr_err(DMA_TAG "irq mode %d failed!\n", irq_mode);
			return -EINVAL;
		}
	} else {
		dma_reg->intc = 0;
	}

	if (chn_type == STANDARD_DMA)
		return 0;

	/* full dma config */
	if (0x0 == dma_cfg_tmp->transcation_len)
		dma_reg->trsc_len = dma_cfg_tmp->block_len
				    & TRSC_LEN_MASK;
	else
		dma_reg->trsc_len = dma_cfg_tmp->transcation_len
				    & TRSC_LEN_MASK;

	dma_reg->trsf_step =
	    (dma_cfg_tmp->des_step & TRSF_STEP_MASK)
		<< DEST_TRSF_STEP_OFFSET |
	    (dma_cfg_tmp->src_step & TRSF_STEP_MASK)
		<< SRC_TRSF_STEP_OFFSET;

	dma_reg->frg_step =
	    (dma_cfg_tmp->dst_frag_step & FRAG_STEP_MASK)
		<< DEST_FRAG_STEP_OFFSET |
	    (dma_cfg_tmp->src_frag_step & FRAG_STEP_MASK)
		<< SRC_FRAG_STEP_OFFSET;

	dma_reg->src_blk_step = dma_cfg_tmp->src_blk_step;
	dma_reg->des_blk_step = dma_cfg_tmp->dst_blk_step;

	dma_reg->src_blk_step |=
		(u32)((dma_cfg_tmp->linklist_ptr >> 4) & 0xF0000000);
	dma_reg->llist_ptr = (u32)dma_cfg_tmp->linklist_ptr;

	return 0;
}

/* config dma linklist */
static int
sprd_dma_config_linklist(struct dma_chan *chan,
			 struct sprd_dma_desc *mdesc,
			 struct sprd_dma_cfg *cfg_list,
			 u32 node_size)
{
	struct sprd_dma_chn_reg *dma_reg_list;
	struct sprd_dma_cfg list_cfg;
	dma_addr_t cfg_p;
	int ret, i;

	if (node_size < 2) {
		pr_err(DMA_TAG "linklist config node less than 2!\n");
		return -EINVAL;
	}

	/* check dma linklist node memory */
	if (cfg_list[0].link_cfg_v == 0 || cfg_list[0].link_cfg_p == 0) {
		pr_err(DMA_TAG "Haven't allocated memory for list node!\n");
		return -EINVAL;
	}

	/* get linklist node virtual addr and physical addr */
	dma_reg_list = (struct sprd_dma_chn_reg *)cfg_list[0].link_cfg_v;
	cfg_p = (dma_addr_t)cfg_list[0].link_cfg_p;

	pr_dma(DMA_TAG "Linklist:alloc addr virt:0x%lx, phys addr: 0x%lx\n",
		   (unsigned long)dma_reg_list, (unsigned long)cfg_p);

	/* linklist configuration */
	for (i = 0; i < node_size; i++) {
		cfg_list[i].linklist_ptr = (cfg_p +
					((i + 1) % node_size) *
					sizeof(struct sprd_dma_chn_reg) +
					0x10);

		ret = sprd_dma_config(chan,
				      NULL,
				      cfg_list + i,
				      dma_reg_list + i,
				      CONFIG_LINKLIST);
		if (ret < 0) {
			pr_err(DMA_TAG "Linklist configuration error!\n");
			return -EINVAL;
		}

		pr_dma("Configuration the link list!\n");
		sprd_dma_cfg_check_regs(dma_reg_list + i);
	}

	memset((void *)&list_cfg, 0x0, sizeof(list_cfg));
	list_cfg.linklist_ptr = cfg_p + 0x10;
	list_cfg.irq_mode = cfg_list[0].irq_mode;
	list_cfg.src_addr = cfg_list[0].src_addr;
	list_cfg.des_addr = cfg_list[0].des_addr;

	/* support for audio */
	if (cfg_list[node_size - 1].is_end > 1)
		mdesc->cycle = 1;

	ret = sprd_dma_config(chan, mdesc, &list_cfg, NULL, CONFIG_DESC);

	return ret;
}

static dma_int_type sprd_dma_check_int_type(u32 intc_reg)
{
	if (intc_reg & 0x1000)
		return CONFIG_ERR;
	else if (intc_reg & 0x800)
		return LIST_DONE;
	else if (intc_reg & 0x400)
		return TRANS_DONE;
	else if (intc_reg & 0x200)
		return BLK_DONE;
	else if (intc_reg & 0x100)
		return FRAG_DONE;
	else
		return NO_INT;
}

static dma_request_mode sprd_dma_check_req_type(u32 frag_reg)
{
	u32 frag_reg_t = frag_reg >> 24;

	if ((frag_reg_t & 0x3) == 0)
		return	FRAG_REQ_MODE;
	else if ((frag_reg_t & 0x3) == 0x1)
		return	BLOCK_REQ_MODE;
	else if ((frag_reg_t & 0x3) == 0x2)
		return	TRANS_REQ_MODE;
	else if ((frag_reg_t & 0x3) == 0x3)
		return	LIST_REQ_MODE;
	else
		return FRAG_REQ_MODE;
}

/* check if the dma request desc is done */
static void
sprd_dma_check_mdesc_done(struct sprd_dma_desc *mdesc,
			  dma_int_type int_type,
			  dma_request_mode req_mode)
{
	if (mdesc->cycle == 1) {
		mdesc->done = 0;
		return;
	}

	if ((unsigned int)int_type >= ((unsigned int)req_mode + 1))
		mdesc->done = 1;
	else
		mdesc->done = 0;
}

static void sprd_dma_check_int(struct sprd_dma_dev *sdev)
{
	struct sprd_dma_chn *mchan = NULL;
	struct sprd_dma_chn_reg *dma_reg = NULL;
	struct sprd_dma_desc *mdesc = NULL;
	struct dma_async_tx_descriptor *desc = NULL;
	u32 irq_status, i;
	dma_int_type int_type;
	dma_request_mode req_type;
	volatile struct sprd_dma_glb_reg *dma_glb_reg = sdev->dma_glb_base;

	irq_status = dma_glb_reg->int_msk_sts;

	while (irq_status) {
		i = __ffs(irq_status);
		irq_status &= (irq_status - 1);

		mchan = &sdev->channels[i];
		spin_lock(&mchan->chn_lock);
		dma_reg = (struct sprd_dma_chn_reg *)(mchan->dma_chn_base);
		int_type = sprd_dma_check_int_type(dma_reg->intc);
		req_type = sprd_dma_check_req_type(dma_reg->frg_len);
		pr_dma("DMA channel [%d] interrupt,intc=0x%x,int_type=%d,"
			   "req_type=%d!\n", i, dma_reg->intc,
			   int_type, (req_type + 1));

		/* check if the dma request desc is done */
		dma_reg->intc |= 0x1f << 24;
		if (!list_empty(&mchan->active)) {
			mdesc = list_first_entry(&mchan->active,
						 struct sprd_dma_desc, node);
			sprd_dma_check_mdesc_done(mdesc, int_type, req_type);
			if (mdesc->done == 1)
				list_splice_tail_init(&mchan->active,
					&mchan->completed);

			if (mdesc->cycle == 1) {
				desc = &mdesc->desc;
				if (desc->callback)
					desc->callback(desc->callback_param);
				spin_unlock(&mchan->chn_lock);
				return;
			}
		}
		if (!list_empty(&mchan->queued)) {
			mdesc = list_first_entry(&mchan->queued,
				struct sprd_dma_desc, node);
			sprd_dma_check_mdesc_done(mdesc,int_type,req_type);
			if (mdesc->done == 1)
				list_splice_tail_init(&mchan->queued, &mchan->completed);

			/* only for dst chn callback. */
			if ((mdesc->cycle == 1) &&
				(mdesc->dma_flags &
				(DMA_GROUP1_DST | DMA_GROUP2_DST))) {
				desc = &mdesc->desc;
				if (desc->callback)
					desc->callback(desc->callback_param);
			}
		}
		spin_unlock(&mchan->chn_lock);
	}
}

static int sprd_dma_start(struct sprd_dma_dev *sdev,
			  struct sprd_dma_chn *mchan,
			  struct sprd_dma_desc *mdesc,
			  u32 dev_id)
{
	enum dma_flags dma_flags = mdesc->dma_flags;

	sprd_dma_set_uid(sdev, mchan, dev_id);
	sprd_dma_chn_enable(mchan);

	if (DMA_UID_SOFTWARE == dev_id)
		if (!(dma_flags & (DMA_GROUP1_DST | DMA_GROUP2_DST)))
			sprd_dma_soft_request(mchan);

	/* for debug info */
	if (dev_id > DMA_UID_SOFTWARE) {
		pr_dma("sprd_dma_start,dev_id=%d,req_cid=%d!\n",
			dev_id,
			readl_relaxed((void __iomem *)
			SPRD_DMA_REQ_CID(sdev->dma_glb_base,
			dev_id)));
	}

	return 0;
}

static int sprd_dma_stop(struct dma_chan *chan)
{
	struct sprd_dma_chn *mchan =
			container_of(chan, struct sprd_dma_chn, chan);
	struct sprd_dma_dev *sdev = to_sprd_dma_dev(&mchan->chan);

	sprd_dma_unset_uid(sdev, mchan, mchan->dev_id);
	sprd_dma_stop_and_disable(mchan);
	sprd_dma_int_clr(mchan);

	return 0;
}

static void sprd_copy_to_chn(void __iomem *dma_chn_base,
			     struct sprd_dma_chn_reg *first)
{
	struct sprd_dma_chn_reg *dma_chn_reg =
		(struct sprd_dma_chn_reg *)dma_chn_base;
	struct sprd_dma_chn_reg *first_reg = first;

	dma_chn_reg->pause = first_reg->pause;
	dma_chn_reg->cfg = first_reg->cfg;
	dma_chn_reg->intc = first_reg->intc;
	dma_chn_reg->src_addr = first_reg->src_addr;
	dma_chn_reg->des_addr = first_reg->des_addr;
	dma_chn_reg->frg_len = first_reg->frg_len;
	dma_chn_reg->blk_len = first_reg->blk_len;
	dma_chn_reg->trsc_len = first_reg->trsc_len;
	dma_chn_reg->trsf_step = first_reg->trsf_step;
	dma_chn_reg->wrap_ptr = first_reg->wrap_ptr;
	dma_chn_reg->wrap_to = first_reg->wrap_to;
	dma_chn_reg->llist_ptr = first_reg->llist_ptr;
	dma_chn_reg->frg_step = first_reg->frg_step;
	dma_chn_reg->src_blk_step = first_reg->src_blk_step;
	dma_chn_reg->des_blk_step = first_reg->des_blk_step;
	dma_chn_reg->req = first_reg->req;
}

static int
sprd_dma_chn_start_chn_cfg(struct sprd_dma_chn *mchan,
		int chn, enum dma_flags flag)
{
	struct sprd_dma_dev *sdev = to_sprd_dma_dev(&mchan->chan);
	struct sprd_dma_glb_reg *dma_glb_reg = sdev->dma_glb_base;
	int start_mode = 0;

	if (chn < 0) {
		pr_err(DMA_TAG "dma slv chn %d flag 0x%x\n", chn, flag);
		return -EINVAL;
	}

	if (flag & (DMA_GROUP1_SRC | DMA_GROUP2_SRC)) {
		switch (flag &
			(DMA_MUTL_FRAG_DONE | DMA_MUTL_BLK_DONE |
			DMA_MUTL_TRANS_DONE | DMA_MUTL_LIST_DONE)) {
		case DMA_MUTL_FRAG_DONE:
			start_mode = 0;
			break;
		case DMA_MUTL_BLK_DONE:
			start_mode = 1;
			break;
		case DMA_MUTL_TRANS_DONE:
			start_mode = 2;
			break;
		case DMA_MUTL_LIST_DONE:
			start_mode = 3;
			break;
		default:
			pr_err(DMA_TAG "chn stat chn mode flag %d failed\n",
				flag);
			return -EINVAL;
		}
	}

	switch (flag &
		(DMA_GROUP1_SRC | DMA_GROUP2_SRC |
		DMA_GROUP1_DST | DMA_GROUP2_DST)) {
	case DMA_GROUP1_SRC:
		dma_glb_reg->cfg_group1 |= (chn << SRC_CHN_OFFSET);
		dma_glb_reg->cfg_group1 |= ((1 << start_mode) << START_MODE_OFFSET);
		dma_glb_reg->cfg_group1 |= CHN_START_CHN;
		break;
	case DMA_GROUP2_SRC:
		dma_glb_reg->cfg_group2 |= (chn << SRC_CHN_OFFSET);
		dma_glb_reg->cfg_group2 |= ((1 << start_mode) << START_MODE_OFFSET);
		dma_glb_reg->cfg_group2 |= CHN_START_CHN;
		break;
	case DMA_GROUP1_DST:
		dma_glb_reg->cfg_group1 |= (chn << DST_CHN_OFFSET);
		dma_glb_reg->cfg_group1 |= CHN_START_CHN;
		break;
	case DMA_GROUP2_DST:
		dma_glb_reg->cfg_group2 |= (chn << DST_CHN_OFFSET);
		dma_glb_reg->cfg_group2 |= CHN_START_CHN;
		break;
	default:
		pr_err(DMA_TAG "chn stat chn addr flag %d failed\n", flag);
		return -EINVAL;
	}

	return 0;
}

static int sprd_dma_execute(struct dma_chan *chan)
{
	struct sprd_dma_chn *mchan =
			container_of(chan, struct sprd_dma_chn, chan);
	struct sprd_dma_desc *first = NULL;
	struct sprd_dma_dev *sdev = to_sprd_dma_dev(&mchan->chan);

	if (!list_empty(&mchan->active))
		first = list_first_entry(&mchan->active,
					 struct sprd_dma_desc, node);
	else
		return 0;

	pr_dma("Before copy data to DMA reg!\n");
	sprd_dma_cfg_check_regs(first->dma_chn_reg);

	sprd_copy_to_chn(mchan->dma_chn_base, first->dma_chn_reg);

	pr_dma("After copy data to DMA reg!\n");
	sprd_dma_cfg_check_regs(mchan->dma_chn_base);

	mb();
	sprd_dma_start(sdev, mchan, first, mchan->dev_id);

	pr_dma("After start the DMA!\n");
	sprd_dma_cfg_check_regs(mchan->dma_chn_base);

	return 0;
}

static dma_cookie_t sprd_desc_submit(struct dma_async_tx_descriptor *tx)
{
	struct sprd_dma_chn *mchan = to_sprd_dma_chan(tx->chan);
	struct sprd_dma_desc *mdesc = to_sprd_dma_desc(tx);
	struct sprd_dma_desc *first = NULL;
	unsigned long flags;
	dma_cookie_t cookie;

	/* assign cookie */
	spin_lock_irqsave(&mchan->chn_lock, flags);
	cookie = dma_cookie_assign(tx);
	list_move_tail(&mdesc->node, &mchan->queued);
	if (!list_empty(&mdesc->next_node)) {
		list_splice_tail_init(&mdesc->next_node, &mchan->queued);
		pr_dma("Submitting has next node!\n");
	}

	/* config one channel start another chnannel */
	if (mdesc->dma_flags &
		(DMA_GROUP1_DST | DMA_GROUP2_DST)) {
		sprd_dma_chn_start_chn_cfg(mchan, mchan->chan_num + 1,
			mdesc->dma_flags);
		sprd_copy_to_chn(mchan->dma_chn_base, mdesc->dma_chn_reg);
		mb();
		sprd_dma_chn_enable(mchan);
	} else if (mdesc->dma_flags &
		(DMA_GROUP1_SRC | DMA_GROUP2_SRC)) {
		sprd_dma_chn_start_chn_cfg(mchan, mchan->chan_num + 1,
			mdesc->dma_flags);
	}

	/* execute the dma desc */
	if (!(mdesc->dma_flags &
		(DMA_GROUP1_DST | DMA_GROUP2_DST)) &&
		list_empty(&mchan->active)) {
		first = list_first_entry(&mchan->queued,
					struct sprd_dma_desc, node);
		list_move_tail(&first->node, &mchan->active);
		sprd_dma_execute(&mchan->chan);
	}
	spin_unlock_irqrestore(&mchan->chn_lock, flags);

	return cookie;
}

static irqreturn_t dma_irq_handle(int irq, void *dev_id)
{
	struct sprd_dma_dev *sdev = (struct sprd_dma_dev *)dev_id;

	/* spin_lock(&sdev->dma_lock); */
	sprd_dma_check_int(sdev);
	/* spin_unlock(&sdev->dma_lock); */

	tasklet_schedule(&sdev->tasklet);

	return IRQ_HANDLED;
}

static int sprd_dma_process_completed(struct sprd_dma_dev *sdev)
{
	struct sprd_dma_chn *mchan;
	struct sprd_dma_desc *mdesc;
	struct sprd_dma_desc *first;
	dma_cookie_t last_cookie = 0;
	struct dma_async_tx_descriptor *desc;
	unsigned long flags;
	LIST_HEAD(list);
	u32 dma_chn_cnt = sdev->dma_chn_cnt;
	int i;

	for (i = 0; i < dma_chn_cnt; i++) {
		mchan = &sdev->channels[i];

		/* deal the compelete dma request */
		spin_lock_irqsave(&mchan->chn_lock, flags);
		if (!list_empty(&mchan->completed))
			list_splice_tail_init(&mchan->completed, &list);
		spin_unlock_irqrestore(&mchan->chn_lock, flags);

		if (list_empty(&list))
			continue;

		list_for_each_entry(mdesc, &list, node) {
			pr_dma("Channel [%d] complete list have node!\n", i);
			desc = &mdesc->desc;

			if (desc->callback)
				desc->callback(desc->callback_param);

			/* submit desc->next */
			dma_run_dependencies(desc);
			last_cookie = desc->cookie;
		}

		spin_lock_irqsave(&mchan->chn_lock, flags);
		list_splice_tail_init(&list, &mchan->free);

		/* continue to process new adding queued request */
		if (!list_empty(&mchan->queued)) {
			list_for_each_entry(mdesc, &mchan->queued, node) {
				pr_dma("Channel [%d] queued list have node!\n", i);
				/* 2 stage mode will always in queued list */
				/* but donot exe them */
				if (!(mdesc->dma_flags &
					(DMA_GROUP1_DST | DMA_GROUP2_DST)) &&
					(list_empty(&mchan->active))) {
					first = list_first_entry(&mchan->queued,
							struct sprd_dma_desc, node);
					list_move_tail(&first->node, &mchan->active);
					sprd_dma_execute(&mchan->chan);
					break;
				}
			}
		} else {
			mchan->chan.completed_cookie = last_cookie;
			pr_dma("Chan [%d] list is NULL,transfer done!\n", i);
		}

		spin_unlock_irqrestore(&mchan->chn_lock, flags);
	}

	return 0;
}

static void sprd_dma_tasklet(unsigned long data)
{
	struct sprd_dma_dev *sdev = (void *)data;

	sprd_dma_process_completed(sdev);
}

static int sprd_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct sprd_dma_chn *mchan = to_sprd_dma_chan(chan);
	struct sprd_dma_dev *mdev = to_sprd_dma_dev(chan);
	struct sprd_dma_desc *mdesc;
	struct sprd_dma_chn_reg *chn_reg;
	unsigned long i, flags;
	int ret = 0;
	LIST_HEAD(descs);

	/* enable dma */
	if (mchan->chan_num < mdev->dma_chn_cnt) {
#ifdef CONFIG_PM
		ret = pm_runtime_get_sync(chan->device->dev);
		if (ret < 0) {
			pr_err(DMA_TAG "pm_runtime_get_sync is err:%d\n", ret);
			return ret;
		}
#else
		sprd_dma_enable(mdev);
		sprd_dma_int_enable(mdev);
#endif
	}
	chn_reg = devm_kzalloc(mdev->dma_dev.dev,
				      SPRD_DMA_DESCRIPTORS *
				      sizeof(struct sprd_dma_chn_reg),
				      GFP_KERNEL);
	if (!chn_reg) {
		pm_runtime_put_sync(chan->device->dev);
		pr_err(DMA_TAG "%s chan_num = %d alloc memory failed!\n",
			dma_chan_name(chan), mchan->chan_num);
		return -ENOMEM;
	}

	/* init the dma desc */
	for (i = 0; i < SPRD_DMA_DESCRIPTORS; i++) {
		mdesc = (struct sprd_dma_desc *)kmem_cache_zalloc(
					mdev->dma_desc_node_cachep,
					GFP_ATOMIC);
		if (!mdesc) {
			pr_err(DMA_TAG "Alloc only %ld descriptors\n", i);
			break;
		}

		dma_async_tx_descriptor_init(&mdesc->desc, chan);
		mdesc->desc.flags = DMA_CTRL_ACK;
		mdesc->desc.tx_submit = sprd_desc_submit;
		mdesc->dma_chn_reg = &chn_reg[i];
		mdesc->done = 0;
		mdesc->cycle = 0;
		mdesc->dma_flags = 0;
		INIT_LIST_HEAD(&mdesc->node);
		INIT_LIST_HEAD(&mdesc->next_node);
		list_add_tail(&mdesc->node, &descs);
	}

	if (i == 0) {
		pm_runtime_put_sync(chan->device->dev);
		devm_kfree(mdev->dma_dev.dev, chn_reg);
		return -ENOMEM;
	}
	spin_lock_irqsave(&mchan->chn_lock, flags);
	mchan->dma_desc = chn_reg;
	list_splice_tail_init(&descs, &mchan->free);
	spin_unlock_irqrestore(&mchan->chn_lock, flags);
	mchan->dev_id = 0;
	mchan->chan_status = USED;

	pr_dma(DMA_TAG "%s Alloc chan[%d] resources is OK, chn_reg=0x%lx,\n",
		dma_chan_name(chan), mchan->chan_num, (unsigned long)chn_reg);

	return ret;
}

static void sprd_dma_free_chan_resources(struct dma_chan *chan)
{
	struct sprd_dma_chn *mchan = to_sprd_dma_chan(chan);
	struct sprd_dma_dev *mdev = to_sprd_dma_dev(chan);
	struct sprd_dma_desc *mdesc, *tmp;
	struct sprd_dma_chn_reg *chn_reg;
	struct sprd_dma_chn *tchan = tchan;
	u32 dma_chn_cnt;
	unsigned long flags;
	LIST_HEAD(descs);
	int i, ret;

	i = ret = 0;
	dma_chn_cnt = mdev->dma_chn_cnt;
	/* delete list */
	spin_lock_irqsave(&mchan->chn_lock, flags);
	list_splice_tail_init(&mchan->prepared, &mchan->free);
	list_splice_tail_init(&mchan->queued, &mchan->free);
	list_splice_tail_init(&mchan->active, &mchan->free);
	list_splice_tail_init(&mchan->completed, &mchan->free);

	list_splice_tail_init(&mchan->free, &descs);
	chn_reg = mchan->dma_desc;
	spin_unlock_irqrestore(&mchan->chn_lock, flags);

	devm_kfree(mdev->dma_dev.dev, chn_reg);

	list_for_each_entry_safe(mdesc, tmp, &descs, node)
		kmem_cache_free(mdev->dma_desc_node_cachep, mdesc);
	mchan->chan_status = NO_USED;
	/* stop and disable dma */
	sprd_dma_stop(&mchan->chan);
	pr_dma(DMA_TAG "%s release chan [%d] resources is OK!\n",
		dma_chan_name(chan), mchan->chan_num);
#ifdef CONFIG_PM
	ret = pm_runtime_put_sync(chan->device->dev);
	if (ret < 0) {
		pr_err(DMA_TAG "pm_runtime_put_sync is err:%d\n", ret);
		return;
	}
#else
	for (i = 0; i < dma_chn_cnt; i++) {
		tchan = &mdev->channels[i];
		if (tchan->chan_status == USED)
			break;
	}

	sprd_dma_disable(mdev);
	if (i == dma_chn_cnt) {
		sprd_dma_int_disable(mdev);
		sprd_dma_softreset(mdev);
	}
#endif
}

static enum dma_status
sprd_dma_tx_status(struct dma_chan *chan,
		   dma_cookie_t cookie,
		   struct dma_tx_state *txstate)
{
	enum dma_status ret;
	int residue = txstate->residue;

	/* spin_lock(&mchan->chn_lock); */
	ret = dma_cookie_status(chan, cookie, txstate);
	/* spin_unlock(&mchan->chn_lock); */

	/* get dma sour&dest addr */
	if (residue == SPRD_SRC_ADDR)
		txstate->residue = sprd_dma_get_src_addr(chan);
	else if (residue == SPRD_DST_ADDR)
		txstate->residue = sprd_dma_get_dst_addr(chan);
	else
		txstate->residue = 0;

	pr_dma("%s cookie=%d, residue=0x%x!\n", __func__,
		cookie, txstate->residue);
	return ret;
}

static void sprd_dma_issue_pending(struct dma_chan *chan)
{
	/*
	 * We are posting descriptors to the hardware as soon as
	 * they are ready, so this function does nothing.
	 */
}

struct dma_async_tx_descriptor *
sprd_dma_prep_dma_memcpy(struct dma_chan *chan,
			 dma_addr_t dest, dma_addr_t src,
			 size_t len, unsigned long flags)
{
	struct sprd_dma_dev *sdev = to_sprd_dma_dev(chan);
	struct sprd_dma_chn *mchan = to_sprd_dma_chan(chan);
	struct sprd_dma_desc *mdesc = NULL;
	struct sprd_dma_cfg *dma_cfg = NULL;
	unsigned long irq_flags;
	u32 datawidth = 0, src_step = 0, des_step = 0;
	int dma_cfg_cnt = dma_cfg_group.dma_cfg_cnt;
	int ret;

	/* for configuration */
	if (flags & DMA_CFG_FLAG) {
		if (dma_cfg_cnt < 1 || dma_cfg_cnt > SPRD_DMA_CFG_COUNT) {
			dma_cfg_group.dma_cfg_cnt = 0;
			up(&dma_cfg_group.cfg_sema);
			pr_err(DMA_TAG "DMA wrong configuration number!\n");
			return NULL;
		}

		dma_cfg = (struct sprd_dma_cfg *)kzalloc(
			sizeof(struct sprd_dma_cfg) * dma_cfg_cnt,
			GFP_KERNEL);
		memcpy(dma_cfg, dma_cfg_group.dma_cfg,
			sizeof(struct sprd_dma_cfg)*dma_cfg_cnt);

		dma_cfg_group.dma_cfg_cnt = 0;
		memset(dma_cfg_group.dma_cfg, 0,
			sizeof(struct sprd_dma_cfg) * SPRD_DMA_CFG_COUNT);
		up(&dma_cfg_group.cfg_sema);
		goto configured;
	} else {
		dma_cfg = (struct sprd_dma_cfg *)kzalloc(
				sizeof(struct sprd_dma_cfg), GFP_KERNEL);
	}

	if (len > BLK_LEN_MASK && mchan->chn_type != FULL_DMA) {
		kfree(dma_cfg);
		pr_err(DMA_TAG "Channel type isn't support!\n");
		return NULL;
	}

	/* set step automatically */
	if ((len & 0x3) == 0) {
		datawidth = 2;
		src_step = 4;
		des_step = 4;
	} else {
		if ((len & 0x1) == 0) {
			datawidth = 1;
			src_step = 2;
			des_step = 2;
		} else {
			datawidth = 0;
			src_step = 1;
			des_step = 1;
		}
	}

	/* dma reg setup */
	memset(&dma_cfg[0], 0, sizeof(struct sprd_dma_cfg));
	dma_cfg[0].src_addr = src;
	dma_cfg[0].des_addr = dest;
	dma_cfg[0].datawidth = datawidth;
	dma_cfg[0].src_step = src_step;
	dma_cfg[0].des_step = src_step;
	dma_cfg[0].fragmens_len = SPRD_DMA_MEMCPY_MIN_SIZE;
	if (len <= BLK_LEN_MASK) {
		dma_cfg[0].block_len = len;
		dma_cfg[0].req_mode = BLOCK_REQ_MODE;
		dma_cfg[0].irq_mode = BLK_DONE;
	} else {
		dma_cfg[0].block_len = SPRD_DMA_MEMCPY_MIN_SIZE;
		dma_cfg[0].transcation_len = len;
		dma_cfg[0].req_mode = TRANS_REQ_MODE;
		dma_cfg[0].irq_mode = TRANS_DONE;
	}
	dma_cfg_cnt = 1;

configured:
	/* get a free dma desc */
	spin_lock_irqsave(&mchan->chn_lock, irq_flags);
	if (!list_empty(&mchan->free)) {
		mdesc = list_first_entry(&mchan->free,
					struct sprd_dma_desc, node);
		list_del(&mdesc->node);
	}
	spin_unlock_irqrestore(&mchan->chn_lock, irq_flags);

	if (!mdesc) {
		pr_err(DMA_TAG "Get mdesc failed!\n");
		sprd_dma_process_completed(sdev);
		kfree(dma_cfg);
		return NULL;
	}

	/* config into dma reg */
	mdesc->dma_flags = flags;
	if (dma_cfg_cnt == 1)
		ret = sprd_dma_config(chan, mdesc, &dma_cfg[0], NULL, CONFIG_DESC);
	else
		/* dma_cfg_cnt > 1 */
		ret = sprd_dma_config_linklist(chan, mdesc, &dma_cfg[0],
					dma_cfg_cnt);

	kfree(dma_cfg);

	if (ret < 0) {
		spin_lock_irqsave(&mchan->chn_lock, irq_flags);
		list_add_tail(&mdesc->node, &mchan->free);
		spin_unlock_irqrestore(&mchan->chn_lock, irq_flags);
		pr_err(DMA_TAG "Configuration is error!\n");
		return NULL;
	}

	/* support hardware request */
	if (flags & DMA_HARDWARE_FLAG) {
		mchan->re_mode = HARDWARE_REQ;
	} else {
		mchan->re_mode = SOFTWARE_REQ;
		mchan->dev_id =  DMA_UID_SOFTWARE;
	}

	/* add the prepare list */
	spin_lock_irqsave(&mchan->chn_lock, irq_flags);
	list_add_tail(&mdesc->node, &mchan->prepared);
	spin_unlock_irqrestore(&mchan->chn_lock, irq_flags);
	pr_dma(DMA_TAG "DMA chan[%d] flag %ld configuration ok!\n",
		mchan->chan_num, flags);
	return &mdesc->desc;
}

struct dma_async_tx_descriptor *
sprd_prep_dma_sg(struct dma_chan *chan,
		 struct scatterlist *dst_sg, unsigned int dst_nents,
		 struct scatterlist *src_sg, unsigned int src_nents,
		 unsigned long flags)
{
	struct sprd_dma_dev *sdev = to_sprd_dma_dev(chan);
	struct sprd_dma_chn *mchan = to_sprd_dma_chan(chan);
	struct sprd_dma_desc *mdesc = NULL;
	struct sprd_dma_desc *first_mdesc = NULL;
	struct scatterlist *sg_d;
	struct scatterlist *sg_s;
	unsigned int scatterlist_entry, src_dma_len, dst_dma_len;
	u32 datawidth, src_step, des_step;
	struct sprd_dma_cfg dma_cfg_t;
	dma_addr_t dst_dma_addr, src_dma_addr;
	unsigned long irq_flags, len;
	int i, ret;

	/* security check */
	if (dst_nents != src_nents) {
		pr_err(DMA_TAG "DMA scatterlist entry count is not equal!\n");
		return NULL;
	}

	scatterlist_entry = src_nents;

	if (scatterlist_entry > SPRD_DMA_DESCRIPTORS) {
		pr_err(DMA_TAG "DMA scatterlist is overrun!\n");
		return NULL;
	}

	if (flags & DMA_HARDWARE_FLAG) {
		pr_err(DMA_TAG "DMA scatterlist do't support hw request!\n");
		return NULL;
	}

	/* for scatter list */
	for (i = 0, sg_d = dst_sg, sg_s = src_sg; i < scatterlist_entry;
		 i++, sg_d = sg_next(sg_d), sg_s = sg_next(sg_s)) {
		dst_dma_addr = sg_dma_address(sg_d);
		dst_dma_len = sg_dma_len(sg_d);
		src_dma_addr = sg_dma_address(sg_s);
		src_dma_len = sg_dma_len(sg_s);

		if (dst_dma_len != src_dma_len)
			continue;
		else
			len = src_dma_len;

		if ((len & 0x3) == 0) {
			datawidth = 2;
			src_step = 4;
			des_step = 4;
		} else {
			if ((len & 0x1) == 0) {
				datawidth = 1;
				src_step = 2;
				des_step = 2;
			} else {
				datawidth = 0;
				src_step = 1;
				des_step = 1;
			}
		}

		memset(&dma_cfg_t, 0, sizeof(struct sprd_dma_cfg));
		dma_cfg_t.src_addr = src_dma_addr;
		dma_cfg_t.des_addr = dst_dma_addr;
		dma_cfg_t.datawidth = datawidth;
		dma_cfg_t.src_step = src_step;
		dma_cfg_t.des_step = src_step;
		dma_cfg_t.fragmens_len = SPRD_DMA_MEMCPY_MIN_SIZE;
		if (len <= BLK_LEN_MASK) {
			dma_cfg_t.block_len = len;
			dma_cfg_t.req_mode = BLOCK_REQ_MODE;
			dma_cfg_t.irq_mode = BLK_DONE;
		} else {
			dma_cfg_t.block_len = SPRD_DMA_MEMCPY_MIN_SIZE;
			dma_cfg_t.transcation_len = len;
			dma_cfg_t.req_mode = TRANS_REQ_MODE;
			dma_cfg_t.irq_mode = TRANS_DONE;
		}

		spin_lock_irqsave(&mchan->chn_lock, irq_flags);
		if (!list_empty(&mchan->free)) {
			mdesc = list_first_entry(&mchan->free,
							struct sprd_dma_desc,
							node);
			list_del(&mdesc->node);
		}
		spin_unlock_irqrestore(&mchan->chn_lock, irq_flags);

		if (!mdesc) {
			sprd_dma_process_completed(sdev);
			pr_err(DMA_TAG "There are not enough mdesc for scatterlist!\n");
		}

		ret = sprd_dma_config(chan, mdesc, &dma_cfg_t, NULL, CONFIG_DESC);
		if (ret < 0) {
			pr_err(DMA_TAG "Configuration is error!\n");
			spin_lock_irqsave(&mchan->chn_lock, irq_flags);
			list_add_tail(&mdesc->node, &mchan->free);
			spin_unlock_irqrestore(&mchan->chn_lock, irq_flags);
			continue;
		}

		if (!first_mdesc) {
			first_mdesc = mdesc;
			spin_lock_irqsave(&mchan->chn_lock, irq_flags);
			list_add_tail(&mdesc->node, &mchan->prepared);
			spin_unlock_irqrestore(&mchan->chn_lock, irq_flags);
		} else {
			spin_lock_irqsave(&mchan->chn_lock, irq_flags);
			list_add_tail(&mdesc->node, &first_mdesc->next_node);
			spin_unlock_irqrestore(&mchan->chn_lock, irq_flags);
		}
	}

	mchan->re_mode = SOFTWARE_REQ;

	if (first_mdesc)
		return &first_mdesc->desc;
	else
		return NULL;
}

/* copy dma configuration */
static void sprd_dma_copy(struct sprd_dma_chn *mchan,
	struct sprd_dma_cfg *cfg)
{
	unsigned int i = 0;

	do {
		memcpy(&dma_cfg_group.dma_cfg[i], cfg,
			sizeof(struct sprd_dma_cfg));
		cfg++;
	} while (dma_cfg_group.dma_cfg[i++].is_end == 0
		 && i < (SPRD_DMA_CFG_COUNT - 1));

	dma_cfg_group.dma_cfg_cnt = i;
}

/* terminate dma channel */
static int sprd_terminate_all(struct dma_chan *chan)
{
	sprd_dma_free_chan_resources(chan);
	return 0;
}

/* dma control method */
static int sprd_dma_control(struct dma_chan *chan,
	struct dma_slave_config *config)
{
	struct sprd_dma_chn *mchan = to_sprd_dma_chan(chan);
	struct sprd_dma_cfg *cfg = NULL;

	cfg = container_of(config, struct sprd_dma_cfg, config);
	if (cfg == NULL)
		return -ENXIO;
	if (down_trylock(&dma_cfg_group.cfg_sema)) {
		pr_err(DMA_TAG "DMA resource is busy, try again...\n");
		return -ENXIO;
	}
	sprd_dma_copy(mchan, cfg);

	return 0;
}

/* for dma channel request filter*/
bool sprd_dma_filter_fn(struct dma_chan *chan, void *param)
{
	struct sprd_dma_chn *mchan = to_sprd_dma_chan(chan);
	struct of_phandle_args *dma_spec = (struct of_phandle_args *)param;
	unsigned int req = (unsigned int)dma_spec->args[0];

	if (chan->device->dev->of_node == dma_spec->np)
		return req == (mchan->chan_num + 1);
	else
		return false;
}

struct dma_chan *sprd_dma_simple_xlate(struct of_phandle_args *dma_spec,
						struct of_dma *ofdma)
{
	int count = dma_spec->args_count;
	struct of_dma_filter_info *info = ofdma->of_dma_data;

	if (!info || !info->filter_fn)
		return NULL;

	if (count != 1)
		return NULL;

	return dma_request_channel(info->dma_cap, info->filter_fn,
			dma_spec);
}

/* for dma debug */
int sprd_dma_check_register(struct dma_chan *c)
{
	volatile struct sprd_dma_chn_reg *dma_reg;
	struct sprd_dma_chn *mchan = to_sprd_dma_chan(c);

	dma_reg = (struct sprd_dma_chn_reg *)mchan->dma_chn_base;
	sprd_dma_cfg_check_regs((void __iomem *)dma_reg);

	return 0;
}

static int sprd_dma_probe(struct platform_device *pdev)
{
	void __iomem *dma_base;
	struct sprd_dma_dev *sdev;
	struct sprd_dma_chn *dma_chn;
	struct clk *clk = NULL;
	struct clk *ashb_clk = NULL;
	struct resource *res;
	const struct of_device_id *lock_of_id;
	enum sprd_dma_controller_type dma_type;
	static u64 dma_dmamask = DMA_BIT_MASK(BITS_PER_LONG);
	struct regmap *dma_gpr;
	u32 dma_chn_cnt, full_type_offset;
	u32 full_chn_cnt, std_chn_cnt;
	int ret, i, dma_irq = -ENXIO;

	if (!pdev->dev.of_node) {
		pr_err(DMA_TAG "Can't find the %s dma node!\n", pdev->name);
		return -ENODEV;
	}

	lock_of_id = of_match_node(sprd_dma_match,
				   pdev->dev.of_node);
	if (!lock_of_id) {
		pr_err(DMA_TAG "Get the dma of device id failed!\n");
		return -ENODEV;
	}

	dma_type = (enum sprd_dma_controller_type)lock_of_id->data;

	if (of_property_read_u32(pdev->dev.of_node, "#dma-channels",
				 &dma_chn_cnt)) {
		pr_err(DMA_TAG "Can't get total dma channel number infor!\n");
		return -ENODEV;
	}

	if (of_property_read_u32_index(pdev->dev.of_node,
				       "sprd,full-type-offset",
				       0, &full_type_offset)) {
		pr_err(DMA_TAG "Can't get ap dma full type offset!\n");
		return -ENODEV;
	}

	if (full_type_offset > dma_chn_cnt) {
		pr_err(DMA_TAG "dma full type channel count is incorrect!\n");
		return -1;
	}

	full_chn_cnt = dma_chn_cnt - full_type_offset;
	std_chn_cnt = full_type_offset;

	pr_err(DMA_TAG "DMA total chn: %d, full chn cnt: %d, std chn cnt: %d\n",
		dma_chn_cnt, full_chn_cnt, std_chn_cnt);

	dma_gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
				"sprd,syscon-dma-glb");
	if (IS_ERR(dma_gpr)) {
		pr_err(DMA_TAG "Get the dma gloabl syscon failed!\n");
		return -ENODEV;
	}

	clk = of_clk_get_by_name(pdev->dev.of_node, "enable");
	if (IS_ERR(clk))
		pr_err(DMA_TAG "Can't get the clock dts config: enable!\n");

	ashb_clk = of_clk_get_by_name(pdev->dev.of_node, "ashb_eb");
	if (IS_ERR(ashb_clk))
		pr_err(DMA_TAG "Can't get ashb eb, only means it's not agcp dma.\n");

	if ((of_find_property(pdev->dev.of_node, "interrupts", NULL))) {
		dma_irq = platform_get_irq(pdev, 0);
		if (dma_irq < 0) {
			pr_err(DMA_TAG "Can't get the dma irq number!\n");
			return dma_irq;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err(DMA_TAG "Can't get ap dma registers resource!\n");
		return -EBUSY;
	}

	dma_base = devm_ioremap_nocache(&pdev->dev, res->start,
					resource_size(res));
	if (!dma_base) {
		pr_err(DMA_TAG "Can't get the dma base addr!\n");
		return -ENOMEM;
	}

	pr_dma(DMA_TAG "DMA base is 0x%lx, irq number is %d!\n",
		(unsigned long)dma_base, dma_irq);

	/* dma device momery alloc */
	sdev = devm_kzalloc(&pdev->dev, (sizeof(*sdev) +
			    (sizeof(struct sprd_dma_chn) * dma_chn_cnt)),
			    GFP_KERNEL);
	if (!sdev)
		return -ENOMEM;

	/* initialize dma device */
	dma_cap_set(DMA_MEMCPY | DMA_SG, sdev->dma_dev.cap_mask);
	sdev->dma_dev.chancnt = dma_chn_cnt;
	INIT_LIST_HEAD(&sdev->dma_dev.channels);
	INIT_LIST_HEAD(&sdev->dma_dev.global_node);
	spin_lock_init(&sdev->dma_lock);
	pdev->dev.dma_mask = &dma_dmamask;
	pdev->dev.coherent_dma_mask = dma_dmamask;
	sdev->dma_dev.dev = &pdev->dev;
	sdev->dma_type = dma_type;

	sdev->dma_dev.device_alloc_chan_resources =
				sprd_dma_alloc_chan_resources;
	sdev->dma_dev.device_free_chan_resources = sprd_dma_free_chan_resources;
	sdev->dma_dev.device_tx_status = sprd_dma_tx_status;
	sdev->dma_dev.device_issue_pending = sprd_dma_issue_pending;
	sdev->dma_dev.device_prep_dma_memcpy = sprd_dma_prep_dma_memcpy;
	sdev->dma_dev.device_prep_dma_sg = sprd_prep_dma_sg;
	sdev->dma_dev.device_config = sprd_dma_control;
	sdev->dma_dev.device_pause = sprd_dma_stop;
	sdev->dma_dev.device_resume = sprd_dma_execute;
	sdev->dma_dev.device_terminate_all = sprd_terminate_all;

	/* initialize dma chn */
	for (i = 0; i < dma_chn_cnt; i++) {
		dma_chn = &sdev->channels[i];
		dma_chn->chan.device = &sdev->dma_dev;
		dma_cookie_init(&dma_chn->chan);
		list_add_tail(&dma_chn->chan.device_node,
			      &sdev->dma_dev.channels);

		dma_chn->chan_num = i;
		dma_chn->chan_status = NO_USED;
		dma_chn->irq_handle_enable = 0;

		if (i < std_chn_cnt)
			dma_chn->chn_type = STANDARD_DMA;
		else
			dma_chn->chn_type = FULL_DMA;

		dma_chn->dma_chn_base = (void __iomem *)
					((unsigned long)dma_base +
					0x1000 + SPRD_DMA_CHN_OFFSET * (i));

		pr_info(DMA_TAG "dma chn [%d] base: 0x%lx!\n", i,
			(unsigned long)dma_chn->dma_chn_base);

		spin_lock_init(&dma_chn->chn_lock);
		INIT_LIST_HEAD(&dma_chn->free);
		INIT_LIST_HEAD(&dma_chn->prepared);
		INIT_LIST_HEAD(&dma_chn->queued);
		INIT_LIST_HEAD(&dma_chn->active);
		INIT_LIST_HEAD(&dma_chn->completed);
	}

	sdev->dma_glb_base = dma_base;
	sdev->irq = dma_irq;
	sdev->dma_chn_cnt = dma_chn_cnt;
	sdev->full_chn_cnt = full_chn_cnt;
	sdev->std_chn_cnt = std_chn_cnt;
	sdev->dma_glb_syscon = dma_gpr;
	sdev->clk = clk;
	sdev->ashb_clk = ashb_clk;

	pr_info(DMA_TAG "dma_glb_base = 0x%lx!\n",
		(unsigned long)sdev->dma_glb_base);

	sdev->dma_desc_node_cachep = kmem_cache_create("dma_desc_node",
				sizeof(struct sprd_dma_desc), 0,
				SLAB_HWCACHE_ALIGN, NULL);
	if (!sdev->dma_desc_node_cachep) {
		pr_err(DMA_TAG "DMA alloc cache failed!\n");
		return -ENOMEM;
	}

	/* irq request, if there is no irq, do not request */
	if (dma_irq != -ENXIO) {
		ret = devm_request_irq(&pdev->dev, dma_irq, dma_irq_handle,
				       0, "sprd_dma", (void *)sdev);
		if (ret < 0) {
			pr_err(DMA_TAG "Request dma irq failed %d\n", ret);
			return ret;
		}
		/* initialize the tasklet */
		tasklet_init(&sdev->tasklet, sprd_dma_tasklet,
			(unsigned long)sdev);
	}

	/* save the sdev as private data */
	platform_set_drvdata(pdev, sdev);
	/* enable pm runtime */
	pm_runtime_enable(&pdev->dev);
	/* dma device register */
	ret = dma_async_device_register(&sdev->dma_dev);
	if (ret < 0) {
		pr_err(DMA_TAG "failed to register slave device: %d\n", ret);
		goto register_fail;
	}

	/* Device-tree DMA controller registration */
	sprd_dma_info.dma_cap = sdev->dma_dev.cap_mask;
	ret = of_dma_controller_register(pdev->dev.of_node,
				sprd_dma_simple_xlate, &sprd_dma_info);
	if (ret) {
		pr_warn(DMA_TAG "failed to register of DMA controller\n");
		goto of_register_fail;
	}

	pr_notice(DMA_TAG "SPRD DMA engine driver probe OK!\n");
	return 0;

of_register_fail:
	dma_async_device_unregister(&sdev->dma_dev);
register_fail:
	pm_runtime_disable(&pdev->dev);
	kmem_cache_destroy(sdev->dma_desc_node_cachep);
	return ret;
}

static int sprd_dma_remove(struct platform_device *pdev)
{
	struct sprd_dma_dev *sdev = platform_get_drvdata(pdev);

	dma_async_device_unregister(&sdev->dma_dev);
	kmem_cache_destroy(sdev->dma_desc_node_cachep);
	pm_runtime_disable(&pdev->dev);
	pr_notice(DMA_TAG "SPRD DMA engine driver remove OK!\n");

	return 0;
}

static const struct of_device_id sprd_dma_match[] = {
	{ .compatible = "sprd,ap-dma-v0.0", .data = (void *)AP_DMA_V0_0},
	{ .compatible = "sprd,ap-dma-v1.0", .data = (void *)AP_DMA_V1_0},
	{ .compatible = "sprd,ap-dma-v2.0", .data = (void *)AP_DMA_V2_0},
	{ .compatible = "sprd,ap-dma-v3.0", .data = (void *)AP_DMA_V3_0},
	{ .compatible = "sprd,ap-dma-v4.0", .data = (void *)AP_DMA_V4_0},
	{ .compatible = "sprd,aon-dma-v0.0", .data = (void *)AON_DMA_V0_0},
	{ .compatible = "sprd,aon-dma-v1.0", .data = (void *)AON_DMA_V1_0},
	{ .compatible = "sprd,aon-dma-v2.0", .data = (void *)AON_DMA_V2_0},
	{ .compatible = "sprd,aon-dma-v3.0", .data = (void *)AON_DMA_V3_0},
	{ .compatible = "sprd,agcp-dma-v1.0", .data = (void *)AGCP_DMA_V1_0},
	{ .compatible = "sprd,agcp-dma-v2.0", .data = (void *)AGCP_DMA_V2_0},
	{ .compatible = "sprd,agcp-dma-v3.0", .data = (void *)AGCP_DMA_V3_0},
	{},
};

#ifdef CONFIG_PM
static int sprd_dma_runtime_suspend(struct device *dev)
{
	struct sprd_dma_dev *mdev  = dev_get_drvdata(dev);

	sprd_dma_int_disable(mdev);
	sprd_dma_softreset(mdev);
	sprd_dma_disable(mdev);
	return 0;
}

static int sprd_dma_runtime_resume(struct device *dev)
{
	struct sprd_dma_dev *mdev  = dev_get_drvdata(dev);

	sprd_dma_enable(mdev);
	sprd_dma_int_enable(mdev);
	return 0;
}
#endif

static const struct dev_pm_ops sprd_dma_pm_ops = {
	SET_RUNTIME_PM_OPS(sprd_dma_runtime_suspend, sprd_dma_runtime_resume,
				NULL)
};

static struct platform_driver sprd_dma_driver = {
	.probe = sprd_dma_probe,
	.remove = sprd_dma_remove,
	.driver = {
		.name = "sprd_dma",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sprd_dma_match),
		.pm = &sprd_dma_pm_ops,
	},
};

int __init sprd_dma_init(void)
{
	sema_init(&dma_cfg_group.cfg_sema, 1);
	dma_cfg_group.dma_cfg_cnt = 0;
	memset(dma_cfg_group.dma_cfg, 0,
	   sizeof(struct sprd_dma_cfg) * SPRD_DMA_CFG_COUNT);

	return platform_driver_register(&sprd_dma_driver);
}

void __exit sprd_dma_exit(void)
{
	platform_driver_unregister(&sprd_dma_driver);
}

arch_initcall_sync(sprd_dma_init);
module_exit(sprd_dma_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DMA driver for Spreadtrum");
MODULE_AUTHOR("Baolin Wang <baolin.wang@spreadtrum.com>");
MODULE_AUTHOR("Eric Long <eric.long@spreadtrum.com>");
