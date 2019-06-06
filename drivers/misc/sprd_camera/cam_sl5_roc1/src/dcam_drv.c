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

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/pagemap.h>
#include <linux/sprd_iommu.h>
#include <video/sprd_mm.h>
#include <video/sprd_img.h>

#include "cam_types.h"
#include "cam_hw.h"

#include "dcam_reg.h"
#include "dcam_int.h"
#include "dcam_interface.h"


#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_DRV: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__


#define DCAM_CLK_NUM 4
#define DCAM_AXI_STOP_TIMEOUT			2000
#define DCAM_STATE_QUICKQUIT			0x01
#define DCAM_AXIM_AQOS_MASK		(0x30FFFF)


unsigned long s_dcam_aximbase;
unsigned long s_dcam_regbase[DCAM_ID_MAX];


static DEFINE_MUTEX(clk_domain_mutex);
static int clk_domain_user;

struct dcam_if_clk_tag {
	char *clock;
	char *clk_name;
};
static const struct dcam_if_clk_tag dcam_if_clk_tab[DCAM_CLK_NUM] = {
	{"128", "dcam_clk_128m"},
	{"256", "dcam_clk_256m"},
	{"307", "dcam_clk_307m2"},
	{"384", "dcam_clk_384m"},
};

static int dcam_reg_trace(struct sprd_cam_hw_info *hw, void *arg)
{
#ifdef DCAM_DRV_DEBUG
	unsigned long addr = 0;
	int idx = hw->idx;

	pr_info("DCAM%d: Register list", idx);
	for (addr = DCAM_IP_REVISION; addr <= 0x400;
		addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_REG_RD(idx, addr),
			DCAM_REG_RD(idx, addr + 4),
			DCAM_REG_RD(idx, addr + 8),
			DCAM_REG_RD(idx, addr + 12));
	}

	pr_info("AXIM: Register list");
	for (addr = DCAM_AXIM_CTRL; addr <= DCAM_LBUF_SHARE_MODE;
		addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_AXIM_RD(addr),
			DCAM_AXIM_RD(addr + 4),
			DCAM_AXIM_RD(addr + 8),
			DCAM_AXIM_RD(addr + 12));
	}
#endif
	return 0;
}


static int dcam_init(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;

	ret = dcam_irq_request(&hw->pdev->dev, hw->irq_no, arg);
	return ret;
}

static int dcam_deinit(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;

	ret = dcam_irq_free(&hw->pdev->dev, arg);
	return ret;
}


#ifdef TEST_SHARKL3
static void dcam_force_copy(enum dcam_id idx)
{
	uint32_t mask = BIT_4 | BIT_6 | BIT_8 | BIT_10 | BIT_12 | BIT_14 | BIT_16 | BIT_18;

	pr_info("DCAM%d: force copy 0x%0x:\n", idx, mask);

	/* force copy all*/
	DCAM_REG_MWR(idx, DCAM_CONTROL, mask, mask);
}

static void set_default(struct sprd_cam_hw_info *hw, void *arg)
{
	int i;
	int idx = hw->idx;
	uint32_t bypass, eb, val;

	/* seems sharkl3 hw bug handling.*/
	for (i = 0x200; i < 0x400; i += 4)
		DCAM_REG_WR(idx, i, 0);

	DCAM_REG_WR(idx, DCAM_CFG, 0);   /* disable all path */
	DCAM_REG_WR(idx, DCAM_IMAGE_CONTROL, 0x2b << 8 | 0x01);

	eb = 0;
	DCAM_REG_MWR(idx, DCAM_PDAF_CONTROL, BIT_1 | BIT_0, eb);
	DCAM_REG_MWR(idx, DCAM_VCH2_CONTROL, BIT_1 | BIT_0, eb);
	DCAM_REG_MWR(idx, DCAM_CROP0_START, BIT_31, eb << 31);

	/* default bypass all blocks */
	bypass = 1;
	DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_18, bypass << 18);
	DCAM_REG_MWR(idx, ISP_RGBG_PARAM, BIT_0, bypass);
	DCAM_REG_MWR(idx, ISP_RGBG_YRANDOM_PARAM0, BIT_0, bypass);
	DCAM_REG_MWR(idx, DCAM_LENS_LOAD_EB, BIT_0, bypass);
	DCAM_REG_MWR(idx, ISP_AWBC_PARAM, BIT_0, bypass);
	DCAM_REG_MWR(idx, ISP_BPC_PARAM, BIT_0, bypass);

	/* 3A statistic */
	DCAM_REG_MWR(idx, DCAM_AEM_FRM_CTRL0, BIT_0, bypass);
	DCAM_REG_MWR(idx, ISP_AFL_FRM_CTRL0, BIT_0, bypass);
	DCAM_REG_MWR(idx, ISP_AFM_FRM_CTRL0, BIT_0, bypass);
	DCAM_REG_MWR(idx, NR3_FAST_ME_PARAM, BIT_0, bypass);

	if (0)
	DCAM_REG_MWR(idx, ISP_BPC_GC_CFG,
				0x07 << 0, 0x06 << 0);

	if (idx == 1) {
		/* set blc : 64 */
		DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_18, 0 << 18);
		val = 0x00010040;
		DCAM_REG_WR(idx, DCAM_BLC_PARA_R_B, val);
		DCAM_REG_WR(idx, DCAM_BLC_PARA_G, val);

		/* set rgb gain */
		DCAM_REG_MWR(idx, ISP_RGBG_PARAM, BIT_0, 0);
		val = 0xFFFF0000;
		DCAM_REG_MWR(idx, ISP_RGBG_PARAM, 0xFFFF0000, val);
		val = 0x10001000;
		DCAM_REG_WR(idx, ISP_RGBG_RB, val);
		val = 0x1000;
		DCAM_REG_WR(idx, ISP_RGBG_G, val);
	}

}
static int dcam_reset(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;
	enum dcam_id idx = hw->idx;
	uint32_t reg_val;
	uint32_t time_out = 0, flag = 0;
	uint32_t reset_bit[DCAM_ID_MAX] = {
		BIT_MM_AHB_DCAM0_SOFT_RST,
		BIT_MM_AHB_DCAM1_SOFT_RST,
		BIT_MM_AHB_DCAM2_SOFT_RST
	};

	pr_info("DCAM%d: reset.\n", idx);

	/* firstly, stop AXI writing.
	 *  todo: should consider other dcamX work status here.
	 */
	DCAM_AXIM_MWR(DCAM_AXIM_CTRL, BIT_24 | BIT_23, (0x3 << 23));

	/* then wait for AHB busy cleared */
	while (++time_out < DCAM_AXI_STOP_TIMEOUT) {
		if (0 == (DCAM_AXIM_RD(DCAM_AXIM_DBG_STS) & 0x0F))
			break;
	}

	if (time_out >= DCAM_AXI_STOP_TIMEOUT) {
		pr_info("DCAM%d: reset timeout, axim status 0x%x\n", idx,
			DCAM_AXIM_RD(DCAM_AXIM_DBG_STS));
	}

	flag = reset_bit[idx];

	regmap_update_bits(hw->cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, flag);
	udelay(1);
	regmap_update_bits(hw->cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, ~flag);


	DCAM_REG_MWR(idx, DCAM_INT_CLR,
		DCAM_IRQ_LINE_MASK, DCAM_IRQ_LINE_MASK);
	DCAM_REG_MWR(idx, DCAM_INT_EN,
		DCAM_IRQ_LINE_MASK, DCAM_IRQ_LINE_MASK);

	/* the end, enable AXI writing */
	DCAM_AXIM_MWR(DCAM_AXIM_CTRL, BIT_24 | BIT_23, (0x0 << 23));

	reg_val = (0x0 << 20) | (0xA << 12) | (0x8 << 8) |
			(0xD << 4) | 0xA;
	DCAM_AXIM_MWR(DCAM_AXIM_CTRL,
			DCAM_AXIM_AQOS_MASK, reg_val);

	set_default(hw, NULL);

	pr_info("DCAM%d: reset end\n", idx);
	return ret;
}

static int dcam_start(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;
	uint32_t idx = hw->idx;

	dcam_force_copy(idx);

	/* enable internal logic access sram */
	DCAM_REG_MWR(hw->idx, DCAM_APB_SRAM_CTRL, BIT_0, 1);

	/* trigger cap_en*/
	DCAM_REG_MWR(idx, DCAM_CFG, BIT_0, 1);

	return ret;
}

static int dcam_quickstop(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;
	int time_out = 3000;
	uint32_t idx = hw->idx;
	uint32_t regval;
	unsigned long addr;

	for (addr = DCAM_IP_REVISION; addr < ISP_ANTI_FLICKER_NEW_SUM2; addr += 4) {
		regval = DCAM_REG_RD(idx, addr);
		pr_debug("%04x: 0x%08x\n", (uint32_t)addr, regval);
	}

	/* reset  cap_en*/
	DCAM_REG_MWR(idx, DCAM_CFG, BIT_0, 0);
	DCAM_REG_WR(idx, DCAM_PATH_STOP, 0xFF);

	/* wait for AHB path busy cleared */
	while (time_out) {
		ret = DCAM_REG_RD(idx, DCAM_PATH_BUSY) & 0xFFF;
		if (!ret)
			break;
		udelay(1000);
		time_out--;
	}

	if (time_out == 0)
		pr_err("DCAM%d: stop timeout for 3s\n", idx);

	pr_info("dcam stop\n");
	return ret;
}

static int dcam_enable_clk(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;
	uint32_t rst_bits;

	/* dcam_if share same clk domain */
	mutex_lock(&clk_domain_mutex);
	clk_domain_user++;
	if (clk_domain_user > 1) {
		mutex_unlock(&clk_domain_mutex);
		return 0;
	}

	/*set dcam clock to max value*/
	ret = clk_set_parent(hw->clk, hw->clk_parent);
	if (ret) {
		pr_err("fail to set clk_parent for DCAM%d .\n", hw->idx);
		ret =  -EINVAL;
		goto err_set_parent;
	}

	ret = clk_prepare_enable(hw->clk);
	if (ret) {
		pr_info("fail to enable dcam_clk.\n");
		ret = -EINVAL;
		goto err_prepare;
	}

	ret = clk_set_parent(hw->bpc_clk, hw->bpc_clk_parent);
	if (ret) {
		pr_err("fail to set bpc_clk_parent for DCAM%d .\n", hw->idx);
		ret =  -EINVAL;
		goto err_set_bpc_parent;
	}
	ret = clk_prepare_enable(hw->bpc_clk);
	if (ret) {
		pr_info("fail to enable dcam_clk.\n");
		ret = -EINVAL;
		goto err_prepare_bpc;
	}

	ret = clk_prepare_enable(hw->core_eb);
	if (ret) {
		pr_err("fail to enable dcam_eb.\n");
		ret = -EINVAL;
		goto err_core_eb;
	}

	ret = clk_prepare_enable(hw->axi_eb);
	if (ret) {
		pr_err("fail to enable dcam_axi_eb.\n");
		ret = -EINVAL;
		goto err_axi_eb;
	}

	rst_bits = BIT_MM_AHB_DCAM_AXIM_SOFT_RST |
			BIT_MM_AHB_DCAM_ALL_SOFT_RST;
	regmap_update_bits(hw->cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, rst_bits,  rst_bits);
	udelay(1);
	regmap_update_bits(hw->cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, rst_bits, ~rst_bits);

	mutex_unlock(&clk_domain_mutex);
	pr_info("dcam_enable_clk end.\n");
	return 0;

err_axi_eb:
	clk_disable_unprepare(hw->core_eb);
err_core_eb:
	clk_disable_unprepare(hw->bpc_clk);
err_prepare_bpc:
err_set_bpc_parent:
	clk_disable_unprepare(hw->clk);
	clk_set_parent(hw->bpc_clk, hw->bpc_clk_default);
err_prepare:
err_set_parent:
	clk_set_parent(hw->clk, hw->clk_default);

	clk_domain_user--;
	mutex_unlock(&clk_domain_mutex);

	return ret;
}


static int dcam_disable_clk(struct sprd_cam_hw_info *hw, void *arg)
{
	mutex_lock(&clk_domain_mutex);
	clk_domain_user--;
	if (clk_domain_user > 0)
		goto exit;

	clk_disable_unprepare(hw->axi_eb);
	clk_disable_unprepare(hw->core_eb);
	clk_disable_unprepare(hw->bpc_clk);
	clk_disable_unprepare(hw->clk);
	clk_set_parent(hw->bpc_clk, hw->bpc_clk_default);
	clk_set_parent(hw->clk, hw->clk_default);
	pr_info("done.\n");

exit:
	mutex_unlock(&clk_domain_mutex);
	return 0;
}

static int dcam_update_clk(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;
	int clk_index;
	char *clk_dcam_if = NULL;
	char *clk_name = NULL;
	struct device_node *dn;

	dn = hw->pdev->dev.of_node;
	if (!dn) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	clk_index = *(int *)arg;
	clk_dcam_if = dcam_if_clk_tab[clk_index].clock;
	clk_name = dcam_if_clk_tab[clk_index].clk_name;

	if (clk_name != NULL) {
		hw->clk_parent = of_clk_get_by_name(dn, clk_name);
		if (IS_ERR_OR_NULL(hw->clk_parent)) {
			pr_err("fail to get dcam%d clk_parent\n", hw->idx);
			return PTR_ERR(hw->clk_parent);
		}
	} else {
		pr_err("fail to get valid clk_name &clk_dcam_if %s\n",
			clk_dcam_if);
		return -EINVAL;
	}

	ret = clk_set_parent(hw->clk, hw->clk_parent);
	if (ret) {
		pr_err("fail to set dcam%d clk_parent\n", hw->idx);
		clk_set_parent(hw->clk, hw->clk_default);
		clk_disable_unprepare(hw->core_eb);
	} else {
		pr_info("DCAM%d, update clk to %s\n", hw->idx, clk_name);
	}

	return ret;
}

#else /* ROC1/SHARKL5 */
static void dcam_force_copy(enum dcam_id idx)
{
	uint32_t mask = BIT_0 | BIT_4 | BIT_6 | BIT_8 | BIT_10 | BIT_12 | BIT_14 | BIT_16;

	pr_info("DCAM%d: force copy 0x%0x:\n", idx, mask);

	/* force copy all*/
	DCAM_REG_MWR(idx, DCAM_CONTROL, mask, mask);
}

static void set_default(struct sprd_cam_hw_info *hw, void *arg)
{
	int idx = hw->idx;
	uint32_t bypass, eb;

	DCAM_REG_WR(idx, DCAM_CFG, 0);   /* disable all path */
	DCAM_REG_WR(idx, DCAM_IMAGE_CONTROL, 0x2b << 8 | 0x01);

	eb = 0;
	DCAM_REG_MWR(idx, DCAM_PDAF_CONTROL, BIT_1 | BIT_0, eb);
	DCAM_REG_MWR(idx, DCAM_VCH2_CONTROL, BIT_1 | BIT_0, eb);
	DCAM_REG_MWR(idx, DCAM_CROP0_START, BIT_31, eb << 31);

	/* default bypass all blocks */
	bypass = 1;
	DCAM_REG_MWR(idx, DCAM_BLC_PARA_R_B, BIT_31, bypass << 31);
	DCAM_REG_MWR(idx, ISP_RGBG_YRANDOM_PARAM0, BIT_0, bypass);
	DCAM_REG_MWR(idx, ISP_PPI_PARAM, BIT_0, bypass);
	DCAM_REG_MWR(idx, DCAM_LENS_LOAD_EB, BIT_0, bypass);
	DCAM_REG_MWR(idx, ISP_AWBC_GAIN0, BIT_31, bypass << 31);
	DCAM_REG_MWR(idx, ISP_BPC_PARAM, BIT_0, bypass);

	/* 3A statistic */
	DCAM_REG_MWR(idx, DCAM_AEM_FRM_CTRL0, BIT_0, bypass);
	DCAM_REG_MWR(idx, ISP_AFL_FRM_CTRL0, BIT_0, bypass);
	DCAM_REG_MWR(idx, ISP_AFM_FRM_CTRL0, BIT_0, bypass);
	DCAM_REG_MWR(idx, NR3_FAST_ME_PARAM, BIT_0, bypass);

	/* todo: set other default registers */
}

static int dcam_reset(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;
	enum dcam_id idx = hw->idx;
	uint32_t reg_val;
	uint32_t time_out = 0, flag = 0;
	uint32_t reset_bit[DCAM_ID_MAX] = {
		BIT_MM_AHB_DCAM0_SOFT_RST,
		BIT_MM_AHB_DCAM1_SOFT_RST,
		BIT_MM_AHB_DCAM2_SOFT_RST
	};

	pr_info("DCAM%d: reset.\n", idx);

	/* firstly, stop AXI writing.
	 *  todo: should consider other dcamX work status here.
	 */
	DCAM_AXIM_MWR(DCAM_AXIM_CTRL, BIT_24 | BIT_23, (0x3 << 23));

	/* then wait for AHB busy cleared */
	while (++time_out < DCAM_AXI_STOP_TIMEOUT) {
		if (0 == (DCAM_AXIM_RD(DCAM_AXIM_DBG_STS) & 0x0F))
			break;
	}

	if (time_out >= DCAM_AXI_STOP_TIMEOUT) {
		pr_info("DCAM%d: reset timeout, axim status 0x%x\n", idx,
			DCAM_AXIM_RD(DCAM_AXIM_DBG_STS));
	}

	pr_info("hw %p, cam_ahb_gpr %p\n",
			hw, hw->cam_ahb_gpr);

	flag = reset_bit[idx];
	/* todo: reset DCAMx here from mm sys */
	regmap_update_bits(hw->cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, flag);
	udelay(1);
	pr_info("hw %p, cam_ahb_gpr %p\n",
			hw, hw->cam_ahb_gpr);

	regmap_update_bits(hw->cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, ~flag);
	pr_info("hw %p, cam_ahb_gpr %p\n",
			hw, hw->cam_ahb_gpr);

	DCAM_REG_MWR(idx, DCAM_INT_CLR,
		DCAM_IRQ_LINE_MASK, DCAM_IRQ_LINE_MASK);
	DCAM_REG_MWR(idx, DCAM_INT_EN,
		DCAM_IRQ_LINE_MASK, DCAM_IRQ_LINE_MASK);

	pr_info("set AXIM.\n");
	/* the end, enable AXI writing */
	DCAM_AXIM_MWR(DCAM_AXIM_CTRL, BIT_24 | BIT_23, (0x0 << 23));

	reg_val = (0x0 << 20) | (0xA << 12) | (0x8 << 8) |
			(0xD << 4) | 0xA;
	DCAM_AXIM_MWR(DCAM_AXIM_CTRL,
			DCAM_AXIM_AQOS_MASK, reg_val);

	pr_info("set default regs.\n");
	set_default(hw, NULL);

	pr_info("DCAM%d: reset end\n", idx);
	return ret;
}

static int dcam_start(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;
	uint32_t idx = hw->idx;

	dcam_force_copy(idx);

	/* enable internal logic access sram */
	DCAM_REG_MWR(hw->idx, DCAM_APB_SRAM_CTRL, BIT_0, 1);

	/* trigger cap_en*/
	DCAM_REG_MWR(idx, DCAM_CFG, BIT_0, 1);

	return ret;
}

static int dcam_quickstop(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;
	int time_out = 3000;
	uint32_t idx = hw->idx;

	/* reset  cap_en*/
	DCAM_REG_MWR(idx, DCAM_CFG, BIT_0, 0);
	DCAM_REG_WR(idx, DCAM_PATH_STOP, 0xFF);

	/* wait for AHB path busy cleared */
	while (time_out) {
		ret = DCAM_REG_RD(idx, DCAM_PATH_BUSY) & 0xFFF;
		if (!ret)
			break;
		udelay(1000);
		time_out--;
	}

	if (time_out == 0)
		pr_err("DCAM%d: stop timeout for 3s\n", idx);

	pr_info("dcam stop\n");
	return ret;
}

static int dcam_enable_clk(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;

	/* dcam_if share same clk domain */
	mutex_lock(&clk_domain_mutex);
	clk_domain_user++;
	if (clk_domain_user > 1) {
		mutex_unlock(&clk_domain_mutex);
		return 0;
	}

#ifdef TEST_ON_HAPS
	pr_info("skip on haps.\n");
#else
	pr_info("todo here.\n");
#endif

	clk_domain_user--;
	mutex_unlock(&clk_domain_mutex);

	return ret;
}


static int dcam_disable_clk(struct sprd_cam_hw_info *hw, void *arg)
{
	mutex_lock(&clk_domain_mutex);
	clk_domain_user--;
	if (clk_domain_user > 0)
		goto exit;

#ifdef TEST_ON_HAPS
	pr_info("skip on haps.\n");
#else
	pr_info("todo here.\n");
#endif

exit:
	pr_info("done.\n");
	mutex_unlock(&clk_domain_mutex);
	return 0;
}

static int dcam_update_clk(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;

	/* todo: update dcam clock here */
#ifdef TEST_ON_HAPS
	pr_info("skip on haps.\n");
#else
	pr_info("todo here.\n");
#endif
	return ret;
}
#endif

static struct sprd_cam_hw_ops dcam_ops = {
	.init = dcam_init,
	.deinit = dcam_deinit,
	.start = dcam_start,
	.stop = dcam_quickstop,
	.reset = dcam_reset,
	.enable_clk = dcam_enable_clk,
	.disable_clk = dcam_disable_clk,
	.update_clk = dcam_update_clk,
	.trace_reg = dcam_reg_trace,
};

static struct sprd_cam_hw_info dcam_hw_dev[DCAM_ID_MAX];


#ifdef TEST_SHARKL3
int sprd_dcam_parse_dt(struct device_node *dn,
		struct sprd_cam_hw_info **dcam_hw,
		uint32_t *dcam_count)
{
	int i = 0;
	uint32_t count = 0;
	void __iomem *reg_base;
	struct resource res = {0};
	struct sprd_cam_hw_info *dcam_cur;

	pr_info("start dcam dts parse\n");

	if (of_property_read_u32_index(dn, "sprd,dcam-count", 0, &count)) {
		pr_err("fail to parse the property of sprd,dcam-count\n");
		return -EINVAL;
	}

	if (count == 0 || count > DCAM_ID_MAX) {
		pr_err("error dcam count: %d", count);
		return -EINVAL;
	}

	pr_info("dcam dev device node %s, full name %s, count: %d\n",
		dn->name, dn->full_name, count);

	*dcam_count = 0;

	for (i = 0; i < count; i++) {
		dcam_cur = &dcam_hw_dev[i];
		dcam_cur->ops = &dcam_ops;
		dcam_hw[i] = dcam_cur;
		dcam_cur->idx = i;

		dcam_cur->pdev = of_find_device_by_node(dn);
		pr_info("sprd dcam name %s\n", dcam_cur->pdev->name);

		dcam_cur->core_eb = of_clk_get_by_name(dn, "dcam_eb");
		if (IS_ERR_OR_NULL(dcam_cur->core_eb)) {
			pr_err("fail to get dcam_eb.\n");
			return PTR_ERR(dcam_cur->core_eb);
		}

		dcam_cur->axi_eb = of_clk_get_by_name(dn, "dcam_axi_eb");

		dcam_cur->clk = of_clk_get_by_name(dn, "dcam_clk");

		dcam_cur->clk_parent = of_clk_get_by_name(dn, "dcam_clk_parent");

		dcam_cur->clk_default = clk_get_parent(dcam_cur->clk);

		dcam_cur->bpc_clk = of_clk_get_by_name(dn, "dcam_bpc_clk");

		dcam_cur->bpc_clk_parent = of_clk_get_by_name(dn, "dcam_bpc_clk_parent");

		dcam_cur->bpc_clk_default = clk_get_parent(dcam_cur->bpc_clk);


		dcam_cur->cam_ahb_gpr = syscon_regmap_lookup_by_phandle(dn,
			"sprd,cam-ahb-syscon");
		if (IS_ERR_OR_NULL(dcam_cur->cam_ahb_gpr)) {
			pr_err("fail to get sprd,cam-ahb-syscon");
			return PTR_ERR(dcam_cur->cam_ahb_gpr);
		}

		dcam_cur->aon_apb_gpr = syscon_regmap_lookup_by_phandle(dn,
			"sprd,aon-apb-syscon");
		if (IS_ERR_OR_NULL(dcam_cur->aon_apb_gpr)) {
			pr_err("fail to get sprd,aon-apb-syscon");
			return PTR_ERR(dcam_cur->aon_apb_gpr);
		}
		if (of_address_to_resource(dn, i, &res)) {
			pr_err("fail to get dcam phys addr\n");
			return -ENXIO;
		}

		dcam_cur->phy_base = (unsigned long)res.start;

		reg_base = of_iomap(dn, i);
		if (!reg_base) {
			pr_err("fail to get dcam reg_base %d\n", i);
			return -ENXIO;
		}
		s_dcam_regbase[i] = (unsigned long)reg_base;
		dcam_cur->reg_base = (unsigned long)reg_base;

		dcam_cur->irq_no = irq_of_parse_and_map(dn, i);
		if (dcam_cur->irq_no <= 0) {
			pr_err("fail to get dcam%d irq\n", i);
			return -EFAULT;
		}

		pr_info("dcam%d dts OK! regbase %lx, irq %d", i,
				dcam_cur->reg_base,  dcam_cur->irq_no);

	}
	reg_base = of_iomap(dn, i);
	s_dcam_aximbase = (unsigned long)reg_base;
	*dcam_count = count;

	return 0;
}
#else
int sprd_dcam_parse_dt(struct device_node *dn,
		struct sprd_cam_hw_info **dcam_hw,
		uint32_t *dcam_count)
{
	int i = 0;
	uint32_t count = 0;
	void __iomem *reg_base;
	struct resource res = {0};
	struct sprd_cam_hw_info *dcam_cur;

	/* todo: should update according to SharkL5/ROC1 dts
	 * or set value for required variables with hard-code
	 * for quick bringup */

	pr_info("start dcam dts parse\n");

	if (of_property_read_u32_index(dn, "sprd,dcam-count", 0, &count)) {
		pr_err("fail to parse the property of sprd,dcam-count\n");
		return -EINVAL;
	}

	if (count == 0 || count > DCAM_ID_MAX) {
		pr_err("error dcam count: %d", count);
		return -EINVAL;
	}

	pr_info("dcam dev device node %s, full name %s, count: %d\n",
		dn->name, dn->full_name, count);

	*dcam_count = 0;

	for (i = 0; i < count; i++) {
		dcam_cur = &dcam_hw_dev[i];
		dcam_cur->ops = &dcam_ops;
		dcam_hw[i] = dcam_cur;
		dcam_cur->idx = i;

		dcam_cur->pdev = of_find_device_by_node(dn);
		pr_info("sprd dcam name %s\n", dcam_cur->pdev->name);

#ifdef TEST_ON_HAPS
		pr_info("skip parse clock tree on haps.\n");
#else
		pr_info("todo here parse clock tree\n");
#endif

		dcam_cur->cam_ahb_gpr = syscon_regmap_lookup_by_phandle(dn,
			"sprd,cam-ahb-syscon");
		if (IS_ERR_OR_NULL(dcam_cur->cam_ahb_gpr)) {
			pr_err("fail to get sprd,cam-ahb-syscon");
			return PTR_ERR(dcam_cur->cam_ahb_gpr);
		}
		pr_info("hw %p, cam_ahb_gpr %p\n",
			dcam_cur, dcam_cur->cam_ahb_gpr);

		if (of_address_to_resource(dn, i, &res)) {
			pr_err("fail to get dcam phys addr\n");
			return -ENXIO;
		}

		dcam_cur->phy_base = (unsigned long)res.start;

		reg_base = of_iomap(dn, i);
		if (!reg_base) {
			pr_err("fail to get dcam reg_base %d\n", i);
			return -ENXIO;
		}
		s_dcam_regbase[i] = (unsigned long)reg_base;
		dcam_cur->reg_base = (unsigned long)reg_base;

		dcam_cur->irq_no = irq_of_parse_and_map(dn, i);
		if (dcam_cur->irq_no <= 0) {
			pr_err("fail to get dcam%d irq\n", i);
			return -EFAULT;
		}

		pr_info("dcam%d dts OK! regbase %lx, irq %d", i,
				dcam_cur->reg_base,  dcam_cur->irq_no);

	}
	reg_base = of_iomap(dn, i);
	s_dcam_aximbase = (unsigned long)reg_base;
	*dcam_count = count;

	return 0;
}
#endif
