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

#include <linux/types.h>
#include <linux/debugfs.h>
#include <video/sprd_mm.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>

#include "isp_reg.h"

#include "isp_core.h"
#include "isp_cfg.h"


#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_CFG: %d: %d %s:" \
	fmt, current->pid, __LINE__, __func__

#ifdef TEST_SHARKL3
static uint32_t ISP_CFG_MAP[] __aligned(8) = {
		0x00080710, /*0x0710  - 0x0714 , 2   , common path sel*/
		0x00041C10, /*0x1C10  - 0x1C10 , 1   , VST*/
		0x01702010, /*0x2010  - 0x217C , 92  , NLM*/
		0x00041E10, /*0x1E10  - 0x1E10 , 1   , IVST*/
		0x00503010, /*0x3010  - 0x305C , 20  , CFA_NEW*/
		0x00183110, /*0x3110  - 0x3124 , 6   , CMC10*/
		0x00043210, /*0x3210  - 0x3210 , 1   , GAMC_NEW*/
		0x00403310, /*0x3310  - 0x334C , 16  , HSV*/
		0x00243410, /*0x3410  - 0x3430 , 9   , PSTRZ*/
		0x001C3510, /*0x3510  - 0x3528 , 7   , CCE*/
		0x001C3610, /*0x3610  - 0x3628 , 7   , UVD*/
		0x004C5010, /*0x5010  - 0x5058 , 19  , PRECDN*/
		0x00845110, /*0x5110  - 0x5190 , 33  , YNR*/
		0x00045210, /*0x5210  - 0x5210 , 1   , BRTA*/
		0x00045310, /*0x5310  - 0x5310 , 1   , CNTA*/
		0x000C5410, /*0x5410  - 0x5418 , 3   , HISTS*/
		0x00145510, /*0x5510  - 0x5520 , 5   , HISTS2*/
		0x00485610, /*0x5610  - 0x5654 , 18  , CDN*/
		0x00745710, /*0x5710  - 0x5780 , 29  , NEW_EE*/
		0x00045810, /*0x5810  - 0x5810 , 1   , CSA*/
		0x00045910, /*0x5910  - 0x5910 , 1   , HUA*/
		0x00745A10, /*0x5A10  - 0x5A80 , 29  , POST_CDN*/
		0x00045B10, /*0x5B10  - 0x5B10 , 1   , YGAMMA*/
		0x00085C10, /*0x5C10  - 0x5C14 , 2   , YUVDELAY*/
		0x00C85D10, /*0x5D10  - 0x5DD4 , 50  , IIRCNR*/
		0x00185E10, /*0x5E10  - 0x5E24 , 6   , YRANDOM*/
		0x00449010, /*0x9010  - 0x9050 , 17   , 3DNR mem ctrl*/
		0x00649110, /*0x9110  - 0x9170 , 25   , 3DNR blend*/
		0x00189210, /*0x9210  - 0x9224 , 6   , 3DNR store*/
		0x00109310, /*0x9310  - 0x931C , 4   , 3DNR crop*/
		0x0050D010, /*0xD010  - 0xD05C , 20  , SCL_VID*/
		0x0034D110, /*0xD110  - 0xD140 , 13  , SCL_VID_store*/
		0x0044C010, /*0xC010  - 0xC050 , 17  , SCL_CAP*/
		0x0034C110, /*0xC110  - 0xC140 , 13  , SCL_CAP_store*/
		0x00640110, /*0x110   - 0x170  , 25  , FETCH*/
		0x00300210, /*0x210   - 0x23C  , 12  , STORE*/
		0x00180310, /*0x310   - 0x324  , 6   , DISPATCH*/
		0x05A18000, /*0x18000 - 0x1859C, 360 , ISP_HSV_BUF0_CH0*/
		0x10019000, /*0x19000 - 0x19FFC, 1024, ISP_VST_BUF0_CH0*/
		0x1001A000, /*0x1A000 - 0x1AFFC, 1024, ISP_IVST_BUF0_CH0*/
		0x0401B000, /*0x1B000 - 0x1B3FC, 256 , ISP_FGAMMA_R_BUF0_CH0*/
		0x0401C000, /*0x1C000 - 0x1C3FC, 256 , ISP_FGAMMA_G_BUF0_CH0*/
		0x0401D000, /*0x1D000 - 0x1D3FC, 256 , ISP_FGAMMA_B_BUF0_CH0*/
		0x0205E000, /*0x1E000 - 0x1E200, 129 , ISP_YGAMMA_BUF0_CH0*/
		0x007F9100, /*0x39100 - 0x39178, 31  , CAP_HOR_CORF_Y_BUF0_CH0*/
		0x003F9300, /*0x39300 - 0x39338, 15  , CAP_HOR_CORF_UV_BUF0*/
		0x020F94F0, /*0x394F0 - 0x396F8, 131 , CAP_VER_CORF_Y_BUF0_CH0*/
		0x020F9AF0, /*0x39AF0 - 0x39CF8, 131 , CAP_VER_CORF_UV_BUF0*/
		0x007F8100, /*0x38100 - 0x38178, 31  , VID_HOR_CORF_Y_BUF0_CH0*/
		0x003F8300, /*0x38300 - 0x38338, 15  , VID_HOR_CORF_UV_BUF0*/
		0x020F84F0, /*0x384F0 - 0x386F8, 131 , VID_VER_CORF_Y_BUF0_CH0*/
		0x020F8AF0, /*0x38AF0 - 0x38CF8, 131 , VID_VER_CORF_UV_BUF0*/
};
#else  /* SharkL5/ROC1 */
static uint32_t ISP_CFG_MAP[] __aligned(8) = {
		0x00080710, /*0x0710  - 0x0714 , 2   , common path sel*/
};
#endif

unsigned long cfg_cmd_addr_reg[ISP_CONTEXT_NUM] = {
	ISP_CFG_PRE0_CMD_ADDR,
	ISP_CFG_CAP0_CMD_ADDR,
	ISP_CFG_PRE1_CMD_ADDR,
	ISP_CFG_CAP1_CMD_ADDR
};

unsigned long isp_cfg_ctx_addr[ISP_CONTEXT_NUM] = { 0 };

static DEFINE_MUTEX(buf_mutex);

struct isp_dev_cfg_info {
	uint32_t bypass;
	uint32_t tm_bypass;
	uint32_t sdw_mode;
	uint32_t num_of_mod;
	uint32_t *isp_cfg_map;

	uint32_t cfg_main_sel;
	uint32_t bp_pre0_pixel_rdy;
	uint32_t bp_pre1_pixel_rdy;
	uint32_t bp_cap0_pixel_rdy;
	uint32_t bp_cap1_pixel_rdy;

	/* 0: cfg trigger start. 1: fmcu trigger start */
	uint32_t pre0_cmd_ready_mode;
	uint32_t pre1_cmd_ready_mode;
	uint32_t cap0_cmd_ready_mode;
	uint32_t cap1_cmd_ready_mode;

	uint32_t tm_set_number;
	uint32_t cap0_th;
	uint32_t cap1_th;
} s_cfg_settings = {
	0, 1, 1, ARRAY_SIZE(ISP_CFG_MAP), ISP_CFG_MAP,
	0, 1, 1, 1, 1,
	0, 0, 0, 0,
	0, 0, 0
};


int debug_show_ctx_reg_buf(void *param)
{
	struct seq_file *s = (struct seq_file *)param;
	uint32_t ctx_id = *(uint32_t *)s->private;
	uint32_t i, item, start, count;
	uint32_t addr;
	unsigned long datap;

	mutex_lock(&buf_mutex);
	datap =  isp_cfg_ctx_addr[ctx_id];
	if (datap == 0UL) {
		mutex_unlock(&buf_mutex);
		seq_printf(s,
			"----ctx %d cfg buf freed.---\n", ctx_id);
		return 0;
	}

	seq_printf(s,
		"dump regsigters buf of ISP context %d - kaddr: 0x%lx\n",
		ctx_id, datap);

	for (i = 0; i < ARRAY_SIZE(ISP_CFG_MAP); i++) {
		seq_puts(s, "---------------\n");
		item = ISP_CFG_MAP[i];
		start = item & 0xffff;
		count = (item >> 16) & 0xffff;
		for (addr = start; addr < (start + count); addr += 4)
			seq_printf(s, "%04x:  %08x\n",
				addr,  *(uint32_t *)(datap + addr));
	}
	mutex_unlock(&buf_mutex);
	seq_puts(s, "------------------------\n");
	return 0;
}


static void cctx_init_page_buf_addr(
				struct isp_cfg_ctx_desc *cfg_ctx,
				void *sw_addr,
				unsigned long hw_addr)
{
	unsigned long ctx_offset, shadow_offset;
	unsigned long offset;
	struct isp_cfg_buf *cfg_buf;
	int  c_id, bid;

	for (c_id = 0; c_id < ISP_CONTEXT_NUM; c_id++) {
		ctx_offset = c_id * ISP_CFG_BUF_SIZE;
		cfg_buf = &cfg_ctx->cfg_buf[c_id];
		for (bid = 0; bid < CFG_BUF_NUM; bid++) {
			shadow_offset = bid * ISP_REG_SIZE;
			offset = ctx_offset + shadow_offset;
			cfg_buf->reg_buf[bid].sw_addr = sw_addr + offset;
			cfg_buf->reg_buf[bid].hw_addr = hw_addr + offset;
			pr_debug("isp ctx %d  buf %d: sw=%p, hw:0x%lx\n",
				c_id, bid, cfg_buf->reg_buf[bid].sw_addr,
				cfg_buf->reg_buf[bid].hw_addr);
		}
	}
}

static void cctx_deinit_page_buf_addr(struct isp_cfg_ctx_desc *cfg_ctx)
{
	struct isp_cfg_buf *cfg_buf;
	int c_id, bid;

	for (c_id = 0; c_id < ISP_CONTEXT_NUM; c_id++) {
		cfg_buf = &cfg_ctx->cfg_buf[c_id];
		for (bid = 0; bid < CFG_BUF_NUM; bid++) {
			cfg_buf->reg_buf[bid].sw_addr = NULL;
			cfg_buf->reg_buf[bid].hw_addr = 0UL;
		}
	}
}

static void cctx_init_regbuf_addr(struct isp_cfg_ctx_desc *cfg_ctx)
{
	struct isp_cfg_buf *cfg_buf;
	struct regfile_buf_info *cur_regbuf_p;
	enum cfg_buf_id cur_regbuf_id;
	int c_id;

	/*
	 * Init context reg buf for each isp context.
	 * Each context has two buf, shadow and work.
	 * Using shadow buf as initial buf.
	 * @isp_cfg_ctx_addr will be used when using ISP_BASE_ADDR.
	 */
	for (c_id = 0; c_id < ISP_CONTEXT_NUM; c_id++) {
		cfg_buf = &cfg_ctx->cfg_buf[c_id];
		cfg_buf->cur_buf_id = cur_regbuf_id = CFG_BUF_SHADOW;
		cur_regbuf_p = &cfg_buf->reg_buf[cur_regbuf_id];
		isp_cfg_ctx_addr[c_id] =
				(unsigned long)cur_regbuf_p->sw_addr;
		pr_debug("init cctx_buf[%d] sw=%p, hw=0x%lx\n",
					c_id, cur_regbuf_p->sw_addr,
					cur_regbuf_p->hw_addr);
	}

}

static void cctx_deinit_regbuf_addr(struct isp_cfg_ctx_desc *cfg_ctx)
{
	int c_id;

	mutex_lock(&buf_mutex);
	for (c_id = 0; c_id < ISP_CONTEXT_NUM; c_id++)
		isp_cfg_ctx_addr[c_id] = 0UL;
	mutex_unlock(&buf_mutex);
}

static int cctx_reset_page_buf(struct isp_cfg_ctx_desc *cfg_ctx)
{
	int c_id, bid;
	void *sw_addr;
	struct isp_cfg_buf *cfg_buf;
	struct regfile_buf_info *regbuf_p;

	if (!cfg_ctx) {
		pr_err("null cfg_ctx pointer\n");
		return -EFAULT;
	}
	pr_debug("Enter\n");
	for (c_id = 0; c_id < ISP_CONTEXT_NUM; c_id++) {
		cfg_buf = &cfg_ctx->cfg_buf[c_id];

		bid = cfg_buf->cur_buf_id;
		regbuf_p = &cfg_buf->reg_buf[bid];
		sw_addr = regbuf_p->sw_addr;
		if (!IS_ERR_OR_NULL(sw_addr)) {
			memset(sw_addr, 0x0, ISP_REG_SIZE);
			pr_debug("ctx %d reset page buf%d\n", c_id, bid);
		}
	}
	pr_debug("Done\n");

	return 0;
}


static void cctx_page_buf_aligned(struct isp_cfg_ctx_desc *cfg_ctx,
				  void **sw_addr, unsigned long *hw_addr)
{
	void *kaddr;
	unsigned long ofst_align = 0;
	unsigned long iova, aligned_iova;
	struct camera_buf *ion_buf = &cfg_ctx->ion_pool;

	kaddr = (void *)ion_buf->addr_k[0];
	iova = ion_buf->iova[0];

	if (!IS_ERR_OR_NULL(kaddr) && iova) {
		if (IS_ALIGNED(iova, ALIGN_PADDING_SIZE)) {
			*hw_addr = iova;
			*sw_addr = kaddr;
		} else {
			aligned_iova = ALIGN(iova, ALIGN_PADDING_SIZE);
			ofst_align = aligned_iova - iova;
			*sw_addr = kaddr + ofst_align;
			*hw_addr = aligned_iova;
		}
		pr_debug("aligned sw: %p, hw: 0x%lx, ofs: 0x%lx",
					*sw_addr, *hw_addr, ofst_align);
	}
}


static int cctx_buf_init(struct isp_cfg_ctx_desc *cfg_ctx)
{
	int ret = 0;
	int iommu_enable = 0;
	size_t size;
	void *sw_addr = NULL;
	unsigned long hw_addr = 0;
	struct camera_buf *ion_buf = NULL;

	/*alloc cfg context buffer*/
	ion_buf = &cfg_ctx->ion_pool;
	memset(ion_buf, 0, sizeof(cfg_ctx->ion_pool));
	sprintf(ion_buf->name, "isp_cfg_ctx");

	if (sprd_iommu_attach_device(&cfg_ctx->owner->dev) == 0) {
		pr_debug("isp iommu enable\n");
		iommu_enable = 1;
	} else {
		pr_debug("isp iommu disable\n");
		iommu_enable = 0;
	}
	size = ISP_CFG_BUF_SIZE_ALL_PADDING;
	ret = cambuf_alloc(ion_buf, size, 0, iommu_enable);
	if (ret) {
		pr_err("fail to get cfg buffer\n");
		ret = -EFAULT;
		goto err_alloc_cfg;
	}

	ret = cambuf_kmap(ion_buf);
	if (ret) {
		pr_err("fail to kmap cfg buffer\n");
		ret = -EFAULT;
		goto err_kmap_cfg;
	}

	ret = cambuf_iommu_map(ion_buf,
				&cfg_ctx->owner->dev);
	if (ret) {
		pr_err("fail to map cfg buffer\n");
		ret = -EFAULT;
		goto err_hwmap_cfg;
	}

	cctx_page_buf_aligned(cfg_ctx, &sw_addr, &hw_addr);
	cctx_init_page_buf_addr(cfg_ctx, sw_addr, hw_addr);
	cctx_init_regbuf_addr(cfg_ctx);
	cctx_reset_page_buf(cfg_ctx);

	pr_debug("cmd sw: %p, hw: 0x%lx, size:0x%x\n",
			sw_addr, hw_addr, (int)ion_buf->size[0]);

	return 0;

err_hwmap_cfg:
	cambuf_kunmap(ion_buf);
err_kmap_cfg:
	cambuf_free(ion_buf);
err_alloc_cfg:
	return ret;
}


static int cctx_buf_deinit(struct isp_cfg_ctx_desc *cfg_ctx)
{
	struct camera_buf *ion_buf = NULL;

	ion_buf = &cfg_ctx->ion_pool;

	cctx_deinit_regbuf_addr(cfg_ctx);
	cctx_deinit_page_buf_addr(cfg_ctx);

	cambuf_iommu_unmap(ion_buf);
	cambuf_kunmap(ion_buf);
	cambuf_free(ion_buf);

	return 0;
}


/*  Interface */
static int isp_cfg_map_init(struct isp_cfg_ctx_desc *cfg_ctx)
{
	uint32_t i = 0;
	uint32_t cfg_map_size = 0;
	uint32_t *cfg_map = NULL;
	uint32_t val;

	pr_debug("enter.");
	if (atomic_inc_return(&cfg_ctx->map_cnt) == 1) {
		pr_info("isp cfg map init\n");
		cfg_map_size = s_cfg_settings.num_of_mod;
		cfg_map = s_cfg_settings.isp_cfg_map;
		for (i = 0; i < cfg_map_size; i++) {
			ISP_HREG_WR(ISP_CFG0_BUF + i * 4,
				cfg_map[i]);
			ISP_HREG_WR(ISP_CFG1_BUF + i * 4,
				cfg_map[i]);
		}
	}

	val = (s_cfg_settings.pre1_cmd_ready_mode << 27)|
		(s_cfg_settings.pre0_cmd_ready_mode << 26)|
		(s_cfg_settings.cap1_cmd_ready_mode << 25)|
		(s_cfg_settings.cap0_cmd_ready_mode << 24)|
		(s_cfg_settings.bp_cap1_pixel_rdy << 23) |
		(s_cfg_settings.bp_cap0_pixel_rdy << 22) |
		(s_cfg_settings.bp_pre1_pixel_rdy << 21) |
		(s_cfg_settings.bp_pre0_pixel_rdy << 20) |
		(s_cfg_settings.cfg_main_sel << 16) |
		(s_cfg_settings.num_of_mod << 8) |
		(s_cfg_settings.sdw_mode << 5) |
		(s_cfg_settings.tm_bypass << 4) |
		(s_cfg_settings.bypass);

	ISP_HREG_WR(ISP_CFG_PAMATER, val);

	if (!s_cfg_settings.tm_bypass) {
		ISP_HREG_WR(ISP_CFG_TM_NUM,
			    s_cfg_settings.tm_set_number);
		ISP_HREG_WR(ISP_CFG_CAP0_TH,
			    s_cfg_settings.cap0_th);
		ISP_HREG_WR(ISP_CFG_CAP1_TH,
			    s_cfg_settings.cap1_th);
	}

	ISP_HREG_MWR(ISP_ARBITER_ENDIAN_COMM, BIT_0, 0x1);
	return 0;
}

static int isp_cfg_start_isp(
			struct isp_cfg_ctx_desc *cfg_ctx,
			enum isp_context_id ctx_id)
{
	unsigned long reg_addr[ISP_CONTEXT_NUM] = {
		ISP_CFG_PRE0_START,
		ISP_CFG_CAP0_START,
		ISP_CFG_PRE1_START,
		ISP_CFG_CAP1_START,
	};

	pr_debug("isp cfg start:  context_id %d, P0_addr 0x%x\n", ctx_id,
		ISP_HREG_RD(ISP_CFG_PRE0_CMD_ADDR));

	ISP_HREG_WR(reg_addr[ctx_id], 1);
	return 0;
}


static int isp_cfg_config_block(
			struct isp_cfg_ctx_desc *cfg_ctx,
			enum isp_context_id ctx_id,
			uint32_t  fmcu_enable)
{
	int ret = 0;
	uint32_t val = 0;
	unsigned long flag;
	void *shadow_buf_vaddr = NULL;
	void *work_buf_vaddr = NULL;
	unsigned long hw_addr = 0;
	enum cfg_buf_id buf_id;
	struct isp_cfg_buf *cfg_buf_p;
	uint32_t ready_mode[ISP_CONTEXT_NUM] = {
		BIT_26,  /* pre0_cmd_ready_mode */
		BIT_24,  /* cap0_cmd_ready_mode */
		BIT_27,  /* pre1_cmd_ready_mode */
		BIT_25   /* cap1_cmd_ready_mode */
	};

	if (!cfg_ctx) {
		pr_err("null cfg_ctx pointer\n");
		return -EFAULT;
	}

	pr_debug("cfg isp ctx %d\n", ctx_id);

	shadow_buf_vaddr = (void *)isp_cfg_ctx_addr[ctx_id];

	if (ctx_id < ISP_CONTEXT_NUM) {
		cfg_buf_p = &cfg_ctx->cfg_buf[ctx_id];
		buf_id = CFG_BUF_WORK;
		hw_addr = (unsigned long)cfg_buf_p->reg_buf[buf_id].hw_addr;
		work_buf_vaddr = cfg_buf_p->reg_buf[buf_id].sw_addr;

		pr_debug("ctx %d cmd buf %p, %p\n",
				ctx_id, work_buf_vaddr, shadow_buf_vaddr);
		memcpy(work_buf_vaddr, shadow_buf_vaddr, ISP_REG_SIZE);
	} else {
		pr_err("error: invalid isp ctx_id %d\n", ctx_id);
		return -EINVAL;
	}

	FLUSH_DCACHE(work_buf_vaddr, ISP_REG_SIZE);

	pr_debug("shadow: 0x%p, work: 0x%p, hw_addr: 0x%lx\n",
			shadow_buf_vaddr, work_buf_vaddr, hw_addr);

	if (fmcu_enable)
		val = ready_mode[ctx_id];
	else
		val = 0;

	spin_lock_irqsave(&cfg_ctx->lock, flag);
	ISP_HREG_MWR(ISP_CFG_PAMATER, ready_mode[ctx_id], val);
	spin_unlock_irqrestore(&cfg_ctx->lock, flag);

	ISP_HREG_WR(cfg_cmd_addr_reg[ctx_id], hw_addr);

	pr_debug("ctx %d,  reg %08x  %08x, hw_addr %lx, val %08x\n",
		ctx_id,
		(uint32_t)cfg_cmd_addr_reg[ctx_id],
		(uint32_t)ISP_GET_REG(cfg_cmd_addr_reg[ctx_id]),
		hw_addr,
		ISP_HREG_RD(cfg_cmd_addr_reg[ctx_id]));

	pr_debug("Done\n");
	return ret;
}


static int isp_cfg_ctx_init(struct isp_cfg_ctx_desc *cfg_ctx, void *arg)
{
	int ret = 0;
	int i;

	if (!cfg_ctx || !arg) {
		pr_err("null cfg_ctx pointer\n");
		return -EFAULT;
	}
	pr_debug("Enter\n");

	if (atomic_inc_return(&cfg_ctx->user_cnt) > 1) {
		pr_info("already done.\n");
		goto exit;
	}

	cfg_ctx->owner = (struct platform_device *)arg;
	cfg_ctx->lock = __SPIN_LOCK_UNLOCKED(&cfg_ctx->lock);
	atomic_set(&cfg_ctx->map_cnt, 0);

	ret = cctx_buf_init(cfg_ctx);
	if (ret) {
		pr_err("fail to init isp cfg ctx buffer.\n");
		return -EFAULT;
	}

	for (i = 0; i < ISP_CONTEXT_NUM; i++)
		isp_cfg_poll_addr[i] = &isp_cfg_ctx_addr[i];

exit:
	pr_info("Done. ret %d\n", ret);

	return ret;
}

static int isp_cfg_ctx_deinit(struct isp_cfg_ctx_desc *cfg_ctx)
{
	int ret = 0;

	if (!cfg_ctx) {
		pr_err("null cfg_ctx pointer\n");
		return -EFAULT;
	}

	pr_info("Enter\n");
	if (atomic_dec_return(&cfg_ctx->user_cnt) > 0) {
		pr_info("isp cfg still have user.\n");
		goto exit;
	}

	atomic_set(&cfg_ctx->map_cnt, 0);
	ret = cctx_buf_deinit(cfg_ctx);
	if (ret)
		pr_err("fail to deinit isp cfg ctx buffer.\n");
exit:
	pr_info("Done\n");

	return ret;
}


struct isp_cfg_ops cfg_ops = {
	.ctx_init = isp_cfg_ctx_init,
	.ctx_deinit = isp_cfg_ctx_deinit,
	.hw_init = isp_cfg_map_init,
	.hw_cfg = isp_cfg_config_block,
	.hw_start = isp_cfg_start_isp,
};

struct isp_cfg_ctx_desc s_ctx_desc = {
		.ops = &cfg_ops,
};

struct isp_cfg_ctx_desc *get_isp_cfg_ctx_desc()
{
	return &s_ctx_desc;
}

int put_isp_cfg_ctx_desc(struct isp_cfg_ctx_desc *param)
{
	if (&s_ctx_desc == param)
		return 0;
	pr_err("error: mismatched param %p, %p\n",
			param, &s_ctx_desc);
	return -EINVAL;
}
