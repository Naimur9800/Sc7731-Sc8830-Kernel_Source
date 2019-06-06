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
#include <linux/spinlock.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <video/sprd_mm.h>

#include "isp_reg.h"
#include "isp_int.h"
#include "isp_interface.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_DRV: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__


#define ISP_AXI_STOP_TIMEOUT			1000
#define ISP_CLK_NUM                    5
#define CLK_CPPLL                      468000000


struct isp_clk_tag {
	char *clock;
	char *clk_name;
};


static uint32_t  s_isp_irq_no[ISP_LOGICAL_COUNT];
unsigned long s_isp_regbase[ISP_MAX_COUNT];
unsigned long isp_phys_base[ISP_MAX_COUNT];

static unsigned long irq_base[4] = {
	ISP_P0_INT_BASE,
	ISP_C0_INT_BASE,
	ISP_P1_INT_BASE,
	ISP_C1_INT_BASE
};

static const struct isp_clk_tag isp_clk_tab[ISP_CLK_NUM] = {
	{"128", "isp_clk_128m"},
	{"256", "isp_clk_256m"},
	{"307", "isp_clk_307m2"},
	{"384", "isp_clk_384m"},
	{"max", "isp_clk_max"},
};

static int isp_reg_trace(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;
#ifdef ISP_DRV_DEBUG
	unsigned long addr = 0;

	pr_info("ISP%d: Register list:\n", hw->idx);
	for (addr = ISP_INT_EN0; addr <= ISP_ALL_DONE_SRC_CTRL; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));
	}
#endif
	return ret;
}

static int isp_init(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;

	if (!hw || !arg) {
		pr_err("error: null input ptr\n");
		return -EFAULT;
	}

	ret = isp_irq_request(
			&hw->pdev->dev, s_isp_irq_no, arg);
	return ret;
}

static int isp_deinit(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;

	if (!hw || !arg) {
		pr_err("error: null input ptr\n");
		return -EFAULT;
	}

	ret = isp_irq_free(&hw->pdev->dev, arg);

	return ret;
}

static int isp_irq_disable(struct sprd_cam_hw_info *hw, void *arg)
{
	uint32_t ctx_id;

	if (!hw || !arg) {
		pr_err("error: null input ptr.\n");
		return -EFAULT;
	}

	ctx_id = *(uint32_t *)arg;
	if (ctx_id >= 4) {
		pr_err("error ctx id %d\n", ctx_id);
		return -EFAULT;
	}

	ISP_HREG_WR(irq_base[ctx_id] + ISP_INT_EN0, 0);
	ISP_HREG_WR(irq_base[ctx_id] + ISP_INT_EN1, 0);

	return 0;
}

static int isp_irq_clear(struct sprd_cam_hw_info *hw, void *arg)
{
	uint32_t ctx_id;

	if (!hw || !arg) {
		pr_err("error: null input ptr.\n");
		return -EFAULT;
	}

	ctx_id = *(uint32_t *)arg;
	if (ctx_id >= 4) {
		pr_err("error ctx id %d\n", ctx_id);
		return -EFAULT;
	}

	ISP_HREG_WR(irq_base[ctx_id] + ISP_INT_CLR0, 0xFFFFFFFF);
	ISP_HREG_WR(irq_base[ctx_id] + ISP_INT_CLR1, 0xFFFFFFFF);

	return 0;
}

static int isp_irq_enable(struct sprd_cam_hw_info *hw, void *arg)
{
	uint32_t ctx_id;
	uint32_t mask = ~0;

	if (!hw || !arg) {
		pr_err("error: null input ptr.\n");
		return -EFAULT;
	}

	ctx_id = *(uint32_t *)arg;
	if (ctx_id >= 4) {
		pr_err("error ctx id %d\n", ctx_id);
		return -EFAULT;
	}

	ISP_HREG_MWR(irq_base[ctx_id] + ISP_INT_EN0, mask, mask);

	return 0;
}


#ifdef TEST_SHARKL3
static void set_common(void)
{
	uint32_t wqos_val = 0;
	uint32_t rqos_val = 0;

	ISP_HREG_MWR(ISP_AXI_ITI2AXIM_CTRL, 0xFF00, (1 << 8));
	wqos_val = (0x1 << 13) | (0x0 << 12) |
			(0x4 << 8) | (0x6 << 4) | 0x6;
	rqos_val = (0x0 << 8) | (0x6 << 4) | 0x6;
	ISP_HREG_MWR(ISP_AXI_ARBITER_WQOS,
					0x37FF,
					wqos_val);
	ISP_HREG_MWR(ISP_AXI_ARBITER_RQOS,
					0x1FF,
					rqos_val);
	ISP_HREG_WR(ISP_CORE_PMU_EN, 0xFFFF0000);


	ISP_HREG_WR(ISP_COMMON_GCLK_CTRL_0, 0xFFFFFFFF);
	ISP_HREG_WR(ISP_COMMON_GCLK_CTRL_1, 0xFFFFFFFF);
	ISP_HREG_WR(ISP_COMMON_GCLK_CTRL_2, 0xFFFF0000);
	ISP_HREG_WR(ISP_COMMON_GCLK_CTRL_3, 0x7F00);

	ISP_HREG_MWR(ISP_AXI_ISOLATION, BIT_0, 0);
	ISP_HREG_MWR(ISP_ARBITER_ENDIAN0, BIT_0, 0);
	ISP_HREG_WR(ISP_ARBITER_CHK_SUM_CLR, 0xF10);
	ISP_HREG_WR(ISP_ARBITER_CHK_SUM0, 0x0);

	/* to be defined. */
	ISP_HREG_MWR(
		ISP_COMMON_SHADOW_CTRL_CH0, BIT_16, (1 << 16));
	ISP_HREG_MWR(
		ISP_COMMON_SHADOW_CTRL_CH0, BIT_21, (0 << 21));
	ISP_HREG_MWR(ISP_COMMON_PMU_RAM_MASK, BIT_0, 1);
	ISP_HREG_MWR(ISP_BLOCK_MODE, 0xF, 0);

	/* dispatch_done should be disable? */
	ISP_HREG_MWR(ISP_INT_ALL_DONE_CTRL, 0x1F, 0x1C);

	ISP_HREG_WR(ISP_COMMON_SOFT_RST_CTRL, 0);

	/* bypass config mode by default */
	ISP_HREG_MWR(ISP_CFG_PAMATER, BIT_0, 1);

	pr_debug("end\n");
}

static int isp_start(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;

	set_common();
	return ret;
}

static int isp_quickstop(struct sprd_cam_hw_info *hw, void *arg)
{
	uint32_t id;
	uint32_t cid;

	id = hw->idx;

	ISP_HREG_MWR(ISP_AXI_ITI2AXIM_CTRL, BIT_26, 1<<26);

	pr_info("ISP%d:ISP_AXI_AXIM_CTRL 0x%x INT STATUS 0x%x  0x%x 0x%x 0x%x\n",
		id, ISP_HREG_RD(ISP_AXI_ITI2AXIM_CTRL),
		ISP_HREG_RD(ISP_P0_INT_BASE+ISP_INT_STATUS),
		ISP_HREG_RD(ISP_C0_INT_BASE+ISP_INT_STATUS),
		ISP_HREG_RD(ISP_P1_INT_BASE+ISP_INT_STATUS),
		ISP_HREG_RD(ISP_C1_INT_BASE+ISP_INT_STATUS));
	udelay(10);

	for (cid = 0; cid < 4; cid++)
		isp_irq_clear(hw, &cid);

	return 0;
}

static int isp_reset(struct sprd_cam_hw_info *hw, void *arg)
{
	int rtn = 0;
	uint32_t cid;
	uint32_t time_out = 0;
	uint32_t flag = 0;

	pr_info("ISP%d: reset:\n", hw->idx);

	/* then wait for AHB busy cleared */
	while (++time_out < ISP_AXI_STOP_TIMEOUT) {
		if (1 == ((ISP_HREG_RD(ISP_INT_STATUS) & BIT_3) >> 3))
			break;
	}
	if (time_out >= ISP_AXI_STOP_TIMEOUT) {
		pr_info("ISP reset timeout %d\n", time_out);
		return -EFAULT;
	}

	flag = BIT_MM_AHB_ISP_LOG_SOFT_RST;
	regmap_update_bits(hw->cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, flag);
	udelay(1);
	regmap_update_bits(hw->cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, ~flag);

	ISP_HREG_MWR(ISP_AXI_ITI2AXIM_CTRL, BIT_26, 0);

	for (cid = 0; cid < 4; cid++) {
		isp_irq_clear(hw, &cid);
		isp_irq_disable(hw, &cid);
	}

	pr_info("ISP rest end\n");

	return rtn;
}

static int isp_enable_clk(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;
	uint32_t flag = 0;

	/*set isp clock to max value*/
	ret = clk_set_parent(hw->clk, hw->clk_parent);
	if (ret) {
		pr_err("fail to set clk_parent for ISP%d .\n", hw->idx);
		ret =  -EINVAL;
		goto err_set_parent;
	}

	ret = clk_prepare_enable(hw->clk);
	if (ret) {
		pr_info("fail to enable isp0_clk.\n");
		ret = -EINVAL;
		goto err_prepare;
	}

	/*isp enable*/
	ret = clk_prepare_enable(hw->core_eb);
	if (ret) {
		pr_err("fail to enable isp_eb.\n");
		ret = -EINVAL;
		goto err_core_eb;
	}

	ret = clk_prepare_enable(hw->axi_eb);
	if (ret) {
		pr_err("fail to enable isp_axi_eb.\n");
		ret = -EINVAL;
		goto err_axi_eb;
	}

	flag = BIT_MM_AHB_ISP_EB;
	regmap_update_bits(hw->cam_ahb_gpr,
		REG_MM_AHB_AHB_EB, flag, flag);

	return 0;

err_axi_eb:
	clk_disable_unprepare(hw->core_eb);
err_core_eb:
	clk_disable_unprepare(hw->clk);
err_prepare:
err_set_parent:
	clk_set_parent(hw->clk, hw->clk_default);

	return ret;
}

static int isp_disable_clk(struct sprd_cam_hw_info *hw, void *arg)
{
	uint32_t flag = 0;

	/*cut off the isp colck source*/
	clk_disable_unprepare(hw->core_eb);
	clk_disable_unprepare(hw->axi_eb);

	/*set isp clock to default value before power off*/
	clk_set_parent(hw->clk, hw->clk_default);
	clk_disable_unprepare(hw->clk);

	flag = BIT_MM_AHB_ISP_EB;
	regmap_update_bits(hw->cam_ahb_gpr,
		REG_MM_AHB_AHB_EB, flag, ~flag);
	pr_info("done.\n");
	return 0;
}

static int isp_update_clk(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;
	int clk_index;
	char *clk_name = NULL;
	char *clk_isp_if = NULL;
	struct device_node *isp_node = NULL;

	if (!hw || !arg) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	clk_index = *(int *)arg;
	if (clk_index >= ISP_CLK_NUM) {
		pr_err("fail to get isp_clk_parent\n");
		return  -EINVAL;
	}

	clk_isp_if = isp_clk_tab[clk_index].clock;
	clk_name = isp_clk_tab[clk_index].clk_name;

	pr_info("isp_clk_name %s, clock %sM\n", clk_name, clk_isp_if);
	if (clk_name != NULL) {
		isp_node = hw->pdev->dev.of_node;
		hw->clk_parent = of_clk_get_by_name(isp_node, clk_name);
		if (IS_ERR_OR_NULL(hw->clk_parent)) {
			pr_err("fail to get isp_clk_parent\n");
			return  -EINVAL;
		}
	} else {
		pr_err("clk_name is NULL & clk_isp_if %s\n", clk_isp_if);
	}

	if (clk_index == ISP_CLK_NUM) {
		const char *value = NULL;

		value = __clk_get_name(hw->clk_parent);
		if (!strcmp(value, "clk_cppll")) {
			clk_set_rate(hw->clk_parent, CLK_CPPLL);
			pr_info("isp_clk_parent : %s\n",
				__clk_get_name(hw->clk_parent));
		}
	}

	ret = clk_set_parent(hw->clk, hw->clk_parent);
	if (ret) {
		pr_info("fail to set isp_clk_parent.\n");
		clk_set_parent(hw->clk, hw->clk_parent);
		clk_disable_unprepare(hw->clk);
	}

	return ret;
}
#else  /* SharkL5/ROC1 */
static void set_common(void)
{
	uint32_t wqos_val = 0;
	uint32_t rqos_val = 0;

	ISP_HREG_MWR(ISP_AXI_ITI2AXIM_CTRL, 0xFF00, (1 << 8));
	wqos_val = (0x1 << 13) | (0x0 << 12) |
			(0x4 << 8) | (0x6 << 4) | 0x6;
	rqos_val = (0x0 << 8) | (0x6 << 4) | 0x6;
	ISP_HREG_MWR(ISP_AXI_ARBITER_WQOS,
					0x37FF,
					wqos_val);
	ISP_HREG_MWR(ISP_AXI_ARBITER_RQOS,
					0x1FF,
					rqos_val);
	ISP_HREG_WR(ISP_CORE_PMU_EN, 0xFFFF0000);


	ISP_HREG_WR(ISP_COMMON_GCLK_CTRL_0, 0xFFFFFFFF);
	ISP_HREG_WR(ISP_COMMON_GCLK_CTRL_1, 0xFFFFFFFF);
	ISP_HREG_WR(ISP_COMMON_GCLK_CTRL_2, 0xFFFF0000);
	ISP_HREG_WR(ISP_COMMON_GCLK_CTRL_3, 0x7F00);

	ISP_HREG_MWR(ISP_AXI_ISOLATION, BIT_0, 0);
	ISP_HREG_MWR(ISP_ARBITER_ENDIAN0, BIT_0, 0);
	ISP_HREG_WR(ISP_ARBITER_CHK_SUM_CLR, 0xF10);
	ISP_HREG_WR(ISP_ARBITER_CHK_SUM0, 0x0);

	/* to be defined. */
	ISP_HREG_MWR(
		ISP_COMMON_SHADOW_CTRL_CH0, BIT_16, (1 << 16));
	ISP_HREG_MWR(
		ISP_COMMON_SHADOW_CTRL_CH0, BIT_21, (0 << 21));
	ISP_HREG_MWR(ISP_COMMON_PMU_RAM_MASK, BIT_0, 1);
	ISP_HREG_MWR(ISP_BLOCK_MODE, 0xF, 0);

	/* dispatch_done should be disable? */
	ISP_HREG_MWR(ISP_INT_ALL_DONE_CTRL, 0x1F, 0x1C);

	/* bypass config mode by default */
	ISP_HREG_MWR(ISP_CFG_PAMATER, BIT_0, 1);

	pr_debug("end\n");
}

struct isp_work_ctrl {
	uint32_t work_en_sw;
	uint32_t work_start;
	uint32_t work_mode;
};

static int isp_start(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;
	struct isp_work_ctrl work_ctrl;

	/* by SW config (for DVFS)*/
	work_ctrl.work_mode = 1;
	work_ctrl.work_en_sw = 0;
	work_ctrl.work_start = 1;

	ISP_HREG_MWR(ISP_WORK_CTRL, BIT_0, work_ctrl.work_mode);
	ISP_HREG_MWR(ISP_WORK_CTRL, BIT_1, (work_ctrl.work_start << 1));
	ISP_HREG_MWR(ISP_WORK_CTRL, BIT_2, (work_ctrl.work_en_sw << 2));

	set_common();
	return ret;
}

static int isp_quickstop(struct sprd_cam_hw_info *hw, void *arg)
{
	uint32_t id;
	uint32_t cid;

	id = hw->idx;

	ISP_HREG_MWR(ISP_AXI_ITI2AXIM_CTRL, BIT_26, 1<<26);

	pr_info("ISP%d:ISP_AXI_AXIM_CTRL 0x%x INT STATUS 0x%x  0x%x 0x%x 0x%x\n",
		id, ISP_HREG_RD(ISP_AXI_ITI2AXIM_CTRL),
		ISP_HREG_RD(ISP_P0_INT_BASE+ISP_INT_STATUS),
		ISP_HREG_RD(ISP_C0_INT_BASE+ISP_INT_STATUS),
		ISP_HREG_RD(ISP_P1_INT_BASE+ISP_INT_STATUS),
		ISP_HREG_RD(ISP_C1_INT_BASE+ISP_INT_STATUS));
	udelay(10);

	for (cid = 0; cid < 4; cid++)
		isp_irq_clear(hw, &cid);

	return 0;
}

static int isp_reset(struct sprd_cam_hw_info *hw, void *arg)
{
	int rtn = 0;
	uint32_t cid;
	uint32_t time_out = 0;
	uint32_t flag = 0;

	pr_info("ISP%d: reset:\n", hw->idx);

	/* then wait for AHB busy cleared */
	while (++time_out < ISP_AXI_STOP_TIMEOUT) {
		if (1 == ((ISP_HREG_RD(ISP_INT_STATUS) & BIT_3) >> 3))
			break;
	}
	if (time_out >= ISP_AXI_STOP_TIMEOUT) {
		pr_info("ISP reset timeout %d\n", time_out);
		return -EFAULT;
	}

	flag = BIT_MM_AHB_ISP_SOFT_RST;
	/* todo: reset ISP here from mm sys */
	regmap_update_bits(hw->cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, flag);
	udelay(1);
	regmap_update_bits(hw->cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, ~flag);

	ISP_HREG_MWR(ISP_AXI_ITI2AXIM_CTRL, BIT_26, 0);

	for (cid = 0; cid < 4; cid++) {
		isp_irq_clear(hw, &cid);
		isp_irq_disable(hw, &cid);
	}

	pr_info("ISP rest end\n");

	return rtn;
}
static int isp_enable_clk(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;

#ifdef TEST_ON_HAPS
	pr_info("skip on haps.\n");
#else
	pr_info("todo here.\n");
#endif
	/* todo: enable isp clock here */
	return ret;
}

static int isp_disable_clk(struct sprd_cam_hw_info *hw, void *arg)
{
	/* todo: disable isp clock here */
#ifdef TEST_ON_HAPS
	pr_info("skip on haps.\n");
#else
	pr_info("todo here.\n");
#endif
	return 0;
}

static int isp_update_clk(struct sprd_cam_hw_info *hw, void *arg)
{
	int ret = 0;

	/* todo: update isp clock here */
#ifdef TEST_ON_HAPS
	pr_info("skip on haps.\n");
#else
	pr_info("todo here.\n");
#endif
	return ret;
}
#endif


static struct sprd_cam_hw_ops isp_ops = {
	.init = isp_init,
	.deinit = isp_deinit,
	.start = isp_start,
	.stop = isp_quickstop,
	.reset = isp_reset,
	.enable_irq = isp_irq_enable,
	.disable_irq = isp_irq_disable,
	.irq_clear = isp_irq_clear,
	.enable_clk = isp_enable_clk,
	.disable_clk = isp_disable_clk,
	.update_clk = isp_update_clk,
	.trace_reg = isp_reg_trace,
};
static struct sprd_cam_hw_info s_isp_hw_dev;


#ifdef TEST_SHARKL3
int sprd_isp_parse_dt(struct device_node *dn,
		struct sprd_cam_hw_info **isp_hw_dev,
		uint32_t *isp_count)
{
	int i = 0;
	uint32_t count = 0;
	void __iomem *reg_base;
	struct device_node *isp_node = NULL;
	struct resource res = {0};
	struct sprd_cam_hw_info *isp_hw = &s_isp_hw_dev;

	pr_info("isp dev device node %s, full name %s\n",
				dn->name, dn->full_name);
	isp_node = of_parse_phandle(dn, "sprd,isp", 0);
	if (isp_node == NULL) {
		pr_err("fail to parse the property of sprd,isp\n");
		return -EFAULT;
	}

	pr_info("after isp dev device node %s, full name %s\n",
		isp_node->name, isp_node->full_name);
	isp_hw->pdev = of_find_device_by_node(isp_node);
	pr_info("sprd s_isp_pdev name %s\n", isp_hw->pdev->name);

	if (of_device_is_compatible(isp_node, "sprd,isp")) {
		if (of_property_read_u32_index(isp_node,
			"sprd,isp-count", 0, &count)) {
			pr_err("fail to parse the property of sprd,isp-count\n");
			return -EINVAL;
		}

		if (count > 1) {
			pr_err("error isp count: %d", count);
			return -EINVAL;
		}
		*isp_count = count;

		isp_hw->core_eb = of_clk_get_by_name(isp_node, "isp_eb");
		if (IS_ERR_OR_NULL(isp_hw->core_eb)) {
			pr_err("fail to get isp0_eb\n");
			return PTR_ERR(isp_hw->core_eb);
		}

		isp_hw->axi_eb = of_clk_get_by_name(isp_node,
			"isp_axi_eb");
		if (IS_ERR_OR_NULL(isp_hw->axi_eb)) {
			pr_err("fail to get isp0_axi_eb\n");
			return PTR_ERR(isp_hw->axi_eb);
		}

		isp_hw->clk = of_clk_get_by_name(isp_node, "isp_clk");
		if (IS_ERR_OR_NULL(isp_hw->clk)) {
			pr_err("fail to get isp_clk\n");
			return PTR_ERR(isp_hw->clk);
		}

		isp_hw->clk_parent = of_clk_get_by_name(isp_node,
			"isp_clk_parent");
		if (IS_ERR_OR_NULL(isp_hw->clk_parent)) {
			pr_err("fail to get isp_clk_parent\n");
			return PTR_ERR(isp_hw->clk_parent);
		}

		isp_hw->cam_ahb_gpr = syscon_regmap_lookup_by_phandle(isp_node,
			"sprd,cam-ahb-syscon");
		if (IS_ERR_OR_NULL(isp_hw->cam_ahb_gpr))
			return PTR_ERR(isp_hw->cam_ahb_gpr);

		isp_hw->aon_apb_gpr = syscon_regmap_lookup_by_phandle(isp_node,
			"sprd,aon-apb-syscon");
		if (IS_ERR_OR_NULL(isp_hw->aon_apb_gpr))
			return PTR_ERR(isp_hw->aon_apb_gpr);

		isp_hw->clk_default = clk_get_parent(isp_hw->clk);
		if (IS_ERR_OR_NULL(isp_hw->clk_default)) {
			pr_err("fail to get isp_clk_default\n");
			return PTR_ERR(isp_hw->clk_default);
		}

		if (of_address_to_resource(isp_node, i, &res))
			pr_err("fail to get isp phys addr\n");

		isp_hw->phy_base = (unsigned long)res.start;
		isp_phys_base[0] = isp_hw->phy_base;
		pr_info("isp phys reg base is %lx\n", isp_phys_base[0]);
		reg_base = of_iomap(isp_node, i);
		if (!reg_base) {
			pr_err("fail to get isp reg_base %d\n", i);
			return -ENXIO;
		}

		isp_hw->reg_base = (unsigned long)reg_base;
		s_isp_regbase[0] = isp_hw->reg_base;

		for (i = 0; i < ISP_LOGICAL_COUNT; i++) {
			s_isp_irq_no[i] = irq_of_parse_and_map(isp_node, i);
			if (s_isp_irq_no[i] <= 0) {
				pr_err("fail to get isp irq %d\n", i);
				return -EFAULT;
			}

			pr_info("ISP%d dts OK! regbase %lx, irq %d\n", i,
				s_isp_regbase[0],  s_isp_irq_no[i]);
		}
	} else {
		pr_err("fail to match isp device node\n");
		return -EINVAL;
	}

	isp_hw->ops = &isp_ops;
	*isp_hw_dev = isp_hw;

	return 0;
}
#else
int sprd_isp_parse_dt(struct device_node *dn,
		struct sprd_cam_hw_info **isp_hw_dev,
		uint32_t *isp_count)
{
	int i = 0;
	uint32_t count = 0;
	void __iomem *reg_base;
	struct device_node *isp_node = NULL;
	struct resource res = {0};
	struct sprd_cam_hw_info *isp_hw = &s_isp_hw_dev;

	/* todo: should update according to SharkL5/ROC1 dts
	 * or set value for required variables with hard-code
	 * for quick bringup */

	pr_info("isp dev device node %s, full name %s\n",
				dn->name, dn->full_name);
	isp_node = of_parse_phandle(dn, "sprd,isp", 0);
	if (isp_node == NULL) {
		pr_err("fail to parse the property of sprd,isp\n");
		return -EFAULT;
	}

	pr_info("after isp dev device node %s, full name %s\n",
		isp_node->name, isp_node->full_name);
	isp_hw->pdev = of_find_device_by_node(isp_node);
	pr_info("sprd s_isp_pdev name %s\n", isp_hw->pdev->name);

	if (of_device_is_compatible(isp_node, "sprd,isp")) {
		if (of_property_read_u32_index(isp_node,
			"sprd,isp-count", 0, &count)) {
			pr_err("fail to parse the property of sprd,isp-count\n");
			return -EINVAL;
		}

		if (count > 1) {
			pr_err("error isp count: %d", count);
			return -EINVAL;
		}
		*isp_count = count;

#ifdef TEST_ON_HAPS
		pr_info("skip parse clock tree on haps.\n");
#else
		pr_info("todo here parse clock tree\n");
#endif

		isp_hw->cam_ahb_gpr = syscon_regmap_lookup_by_phandle(isp_node,
			"sprd,cam-ahb-syscon");
		if (IS_ERR_OR_NULL(isp_hw->cam_ahb_gpr)) {
			pr_err("fail to get sprd,cam-ahb-syscon");
			return PTR_ERR(isp_hw->cam_ahb_gpr);
		}

		if (of_address_to_resource(isp_node, i, &res))
			pr_err("fail to get isp phys addr\n");

		isp_hw->phy_base = (unsigned long)res.start;
		isp_phys_base[0] = isp_hw->phy_base;
		pr_info("isp phys reg base is %lx\n", isp_phys_base[0]);
		reg_base = of_iomap(isp_node, i);
		if (!reg_base) {
			pr_err("fail to get isp reg_base %d\n", i);
			return -ENXIO;
		}

		isp_hw->reg_base = (unsigned long)reg_base;
		s_isp_regbase[0] = isp_hw->reg_base;

		for (i = 0; i < ISP_LOGICAL_COUNT; i++) {
			s_isp_irq_no[i] = irq_of_parse_and_map(isp_node, i);
			if (s_isp_irq_no[i] <= 0) {
				pr_err("fail to get isp irq %d\n", i);
				return -EFAULT;
			}

			pr_info("ISP%d dts OK! regbase %lx, irq %d\n", i,
				s_isp_regbase[0],  s_isp_irq_no[i]);
		}
	} else {
		pr_err("fail to match isp device node\n");
		return -EINVAL;
	}

	isp_hw->ops = &isp_ops;
	*isp_hw_dev = isp_hw;

	return 0;
}
#endif
