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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include "cam_common.h"
#include "dcam2isp.h"

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "[dcam2isp]: line=%d: " fmt, __LINE__

#define DCAM2ISP_DEVICE_NAME			"dcam2isp"

static const unsigned int mipi_word_num_start[16] = {
	0, 1, 1, 1,
	1, 2, 2, 2,
	3, 3, 3, 4,
	4, 4, 5, 5,
};

static const unsigned int mipi_word_num_end[16] = {
	0, 2, 2, 2,
	2, 3, 3, 3,
	3, 4, 4, 4,
	4, 5, 5, 5,
};

struct dump_info {
	unsigned int addr;
	unsigned int length;
};

static struct dump_info dcam2isp_dump_info[] = {
	{DCAM2ISP_INT_STATUS, 3},
	{DCAM2ISP_MERGE_STATUS0, 4},
	{ISP_AXI0_WR_MASTER_STATUS, 2},
	{ISP_AXI1_WR_MASTER_STATUS, 2},
	{ISP_FETCH0_MEM_STATUS0, 5},
	{ISP_FETCH1_MEM_STATUS0, 5},
	{DCAM2ISP_ISP0_STATUS, 2},
	{DCAM2ISP_ISP1_STATUS, 2},
	{DCAM2ISP_ISP2_STATUS, 2},
};

/* Structure Definitions */
struct dcam2isp_device {
	struct dcam2isp_cfg cfg_parm;
	void __iomem *io_base;
	struct regmap *cam_ahb_gpr;

	struct clk *d2i_if_eb;
	struct clk *d2i_if_axi_eb;
	struct clk *d0if_in_d2i_eb;
	struct clk *d1if_in_d2i_eb;
	struct clk *i0_in_d2i_eb;
	struct clk *i1_in_d2i_eb;
	struct clk *i2_in_d2i_eb;
};

static struct dcam2isp_device *d2iif;

/* Internal Function Implementation */

static void dcam2isp_int_clr(unsigned int mask)
{
	reg_owr(d2iif, DCAM2ISP_INT_CLR0, mask);
	udelay(50);
	reg_awr(d2iif, DCAM2ISP_INT_CLR0, ~mask);
}

static void dcam2isp_int_enable(unsigned int mask)
{
	reg_owr(d2iif, DCAM2ISP_INT_EN0, mask);
}

static void dcam2isp_parm_set(struct dcam2isp_device *d2iif)
{
	unsigned int value = 0;
	struct dcam2isp_cfg *cfg_parm = &d2iif->cfg_parm;

	if (cfg_parm->ctrl_parm.merge_bypass == 1)
		value = 1;
	else
		value = (cfg_parm->ctrl_parm.merge_mode & 0x3) << 4;

	if (cfg_parm->ctrl_parm.isp0_eb == 1)
		value = value | ((cfg_parm->ctrl_parm.isp0_src & 0x3) << 6);
	else
		value = value | (0x3 << 6);

	if (cfg_parm->ctrl_parm.isp1_eb == 1)
		value = value | ((cfg_parm->ctrl_parm.isp1_src & 0x3) << 8);
	else
		value = value | (0x3 << 8);

	if (cfg_parm->ctrl_parm.isp2_eb == 1)
		value = value | ((cfg_parm->ctrl_parm.isp2_src & 0x3) << 10);
	else
		value = value | (0x3 << 10);

	if (cfg_parm->ctrl_parm.fetch0_eb == FETCH_MEM)
		value = value | (1 << 12);

	if (cfg_parm->ctrl_parm.fetch1_eb == FETCH_MEM)
		value = value | (1 << 13);

	reg_wr(d2iif, DCAM2ISP_PARAM, value);
}

static void dcam2isp_fetch_parm_set(struct dcam2isp_device *d2iif)
{
	struct dcam2isp_fetch_parm *fetch_parm = NULL;
	struct dcam2isp_cfg *cfg_parm = &d2iif->cfg_parm;
	uint32_t value = 0x1;
	uint32_t mipi_word_info = 0;

	if (cfg_parm->ctrl_parm.fetch0_eb) {
		fetch_parm = &cfg_parm->fetch_parm[0];
		reg_wr(d2iif, ISP_FETCH0_PARAM,
		       (fetch_parm->color_fmt & 0xf) << 4);

		reg_wr(d2iif, ISP_FETCH0_SLICE_Y_ADDR, fetch_parm->y_address);
		reg_wr(d2iif, ISP_FETCH0_SLICE_U_ADDR, fetch_parm->u_address);
		reg_wr(d2iif, ISP_FETCH0_Y_PITCH, fetch_parm->y_pitch);
		reg_wr(d2iif, ISP_FETCH0_U_PITCH, fetch_parm->u_pitch);

		reg_wr(d2iif, ISP_FETCH0_DATA_ENDIAN, fetch_parm->data_endian);
		reg_wr(d2iif, ISP_FETCH0_MEM_SLICE_SIZE,
		       ((fetch_parm->fetch_size.height & 0xffff) << 16) |
		       ((fetch_parm->fetch_size.width & 0xffff) << 0));
		mipi_word_info = (fetch_parm->fetch_size.width >> 4) * 5 +
			mipi_word_num_end[fetch_parm->fetch_size.width&0xf] -
			mipi_word_num_start[1] +
			1;
		reg_wr(d2iif, ISP_FETCH0_MIPI_WORD_INFO, mipi_word_info);
		reg_wr(d2iif, ISP_FETCH0_START, value);
	}

	if (cfg_parm->ctrl_parm.fetch1_eb) {
		fetch_parm = &cfg_parm->fetch_parm[1];
		reg_wr(d2iif, ISP_FETCH1_PARAM,
		       (fetch_parm->color_fmt & 0xf) << 4);

		reg_wr(d2iif, ISP_FETCH1_SLICE_Y_ADDR, fetch_parm->y_address);
		reg_wr(d2iif, ISP_FETCH1_SLICE_U_ADDR, fetch_parm->u_address);
		reg_wr(d2iif, ISP_FETCH1_Y_PITCH, fetch_parm->y_pitch);
		reg_wr(d2iif, ISP_FETCH1_U_PITCH, fetch_parm->u_pitch);

		reg_wr(d2iif, ISP_FETCH1_DATA_ENDIAN, fetch_parm->data_endian);
		reg_wr(d2iif, ISP_FETCH1_MEM_SLICE_SIZE,
		       ((fetch_parm->fetch_size.height & 0xffff) << 16) |
		       ((fetch_parm->fetch_size.width & 0xffff) << 0));
		reg_wr(d2iif, ISP_FETCH1_START, value);
	}
}

static void dcam2isp_isp_set_slice_size(struct dcam2isp_device *d2iif)
{
	struct dcam2isp_cfg *cfg_parm = &d2iif->cfg_parm;

	if (cfg_parm->ctrl_parm.isp0_eb) {
		reg_wr(d2iif, DCAM2ISP_ISP0_SLICE_SIZE,
		       ((cfg_parm->isp_slice_size[0].height & 0xffff) << 16) |
		       ((cfg_parm->isp_slice_size[0].width & 0xffff) << 0));
	}

	if (cfg_parm->ctrl_parm.isp1_eb) {
		reg_wr(d2iif, DCAM2ISP_ISP1_SLICE_SIZE,
		       ((cfg_parm->isp_slice_size[1].height & 0xffff) << 16) |
		       ((cfg_parm->isp_slice_size[1].width & 0xffff) << 0));
	}

	if (cfg_parm->ctrl_parm.isp2_eb) {
		reg_wr(d2iif, DCAM2ISP_ISP2_SLICE_SIZE,
		       ((cfg_parm->isp_slice_size[2].height & 0xffff) << 16) |
		       ((cfg_parm->isp_slice_size[2].width & 0xffff) << 0));
	}
}

void sprd_dcam2isp_dump_reg(void)
{
	int i = 0, j = 0;
	int ret = 0;
	unsigned int offset = 0;
	unsigned int val = 0;

	ret = regmap_read(d2iif->cam_ahb_gpr,
					REG_CAM_AHB_AHB_EB, &val);
	if (ret) {
		pr_err("%s, read cam_ahb_gpr failed!\n", __func__);
		return;
	}
	pr_info("%s, enter in, cam_ahb_eb: 0x%.8x\n", __func__, val);

	if (0 == (val&BIT_CAM_AHB_DCAM0_IF_EB)) {
		pr_err("%s, dcam0_if not enable!\n", __func__);
		return;
	}

	pr_info("Begin to dump dcam2isp regs:\n");

	for (j = 0; j < ARRAY_SIZE(dcam2isp_dump_info); j++) {
		if ((j == 3 || j == 5) && (0 == (val&BIT_CAM_AHB_DCAM1_EB)))
			continue;
		if (j == 6 && 0 == (val&BIT_CAM_AHB_ISP0_EB))
			continue;
		if (j == 7 && 0 == (val&BIT_CAM_AHB_ISP1_EB))
			continue;
		if (j == 8 && 0 == (val&BIT_CAM_AHB_ISP2_EB))
			continue;
		for (i = 0; i < dcam2isp_dump_info[j].length; i++) {
			offset = dcam2isp_dump_info[j].addr + 0x10 * i;
			pr_info("0x%lx: 0x%.8x 0x%.8x 0x%.8x 0x%.8x\n",
				(unsigned long)d2iif->io_base + offset,
				reg_rd(d2iif, offset + 0x00),
				reg_rd(d2iif, offset + 0x04),
				reg_rd(d2iif, offset + 0x08),
				reg_rd(d2iif, offset + 0x0c)
				);
		}
	}
}
EXPORT_SYMBOL(sprd_dcam2isp_dump_reg);

int sprd_dcam2isp_param_cfg(struct dcam2isp_cfg *cfg)
{
	int i;
	int ret = 0;
	struct dcam2isp_cfg *parm = &d2iif->cfg_parm;

	if (!parm || !cfg) {
		ret = -EFAULT;
		pr_err("parm or cfg is null.\n");
		return ret;
	}

	memset(parm, 0, sizeof(struct dcam2isp_cfg));
	memcpy((void *)parm,
	       (void *)cfg,
	       sizeof(struct dcam2isp_cfg));

	for (i = 0; i < 3; i++) {
		pr_debug("cfg w: %d, h:%d\n",
				cfg->isp_slice_size[i].width,
				cfg->isp_slice_size[i].height);
	}
	pr_debug("cfg ctrl parm: %d, %d, %d, %d, %d, %d\n",
			cfg->ctrl_parm.isp0_eb,
			cfg->ctrl_parm.isp0_src,
			cfg->ctrl_parm.isp1_eb,
			cfg->ctrl_parm.isp1_src,
			cfg->ctrl_parm.isp2_eb,
			cfg->ctrl_parm.isp2_src);

	return ret;
}
EXPORT_SYMBOL(sprd_dcam2isp_param_cfg);

int sprd_dcam2isp_start(void)
{

	dcam2isp_int_clr(0xffffffff);
	dcam2isp_int_enable(0xffffffff);
	dcam2isp_parm_set(d2iif);
	dcam2isp_isp_set_slice_size(d2iif);
	dcam2isp_fetch_parm_set(d2iif);

	return 0;
}
EXPORT_SYMBOL(sprd_dcam2isp_start);


void sprd_dcam2isp_enable(void)
{
	clk_prepare_enable(d2iif->d2i_if_eb);
	clk_prepare_enable(d2iif->d2i_if_axi_eb);
	clk_prepare_enable(d2iif->d0if_in_d2i_eb);
	clk_prepare_enable(d2iif->d1if_in_d2i_eb);
	clk_prepare_enable(d2iif->i0_in_d2i_eb);
	clk_prepare_enable(d2iif->i1_in_d2i_eb);
	clk_prepare_enable(d2iif->i2_in_d2i_eb);
}
EXPORT_SYMBOL(sprd_dcam2isp_enable);

void sprd_dcam2isp_disable(void)
{
	clk_disable_unprepare(d2iif->i2_in_d2i_eb);
	clk_disable_unprepare(d2iif->i1_in_d2i_eb);
	clk_disable_unprepare(d2iif->i0_in_d2i_eb);
	clk_disable_unprepare(d2iif->d1if_in_d2i_eb);
	clk_disable_unprepare(d2iif->d0if_in_d2i_eb);
	clk_disable_unprepare(d2iif->d2i_if_axi_eb);
	clk_disable_unprepare(d2iif->d2i_if_eb);
}
EXPORT_SYMBOL(sprd_dcam2isp_disable);


static int sprd_dcam2isp_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;

	d2iif = devm_kzalloc(&pdev->dev, sizeof(*d2iif), GFP_KERNEL);
	if (!d2iif)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	d2iif->io_base = devm_ioremap_nocache(&pdev->dev, res->start,
					      resource_size(res));
	if (IS_ERR_OR_NULL(d2iif->io_base))
		return -EFAULT;

	d2iif->cam_ahb_gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						      "sprd,syscon-cam-ahb");
	if (IS_ERR_OR_NULL(d2iif->cam_ahb_gpr))
		return -EFAULT;

	d2iif->d2i_if_eb = devm_clk_get(&pdev->dev, "dcam2isp_if_eb");
	if (IS_ERR_OR_NULL(d2iif->d2i_if_eb))
		return -EFAULT;

	d2iif->d2i_if_axi_eb = devm_clk_get(&pdev->dev, "d2i_if_axi_gate");
	if (IS_ERR_OR_NULL(d2iif->d2i_if_axi_eb))
		return -EFAULT;

	d2iif->d0if_in_d2i_eb = devm_clk_get(&pdev->dev, "d0if_in_d2i_eb");
	if (IS_ERR_OR_NULL(d2iif->d0if_in_d2i_eb))
		return -EFAULT;

	d2iif->d1if_in_d2i_eb = devm_clk_get(&pdev->dev, "d1if_in_d2i_eb");
	if (IS_ERR_OR_NULL(d2iif->d1if_in_d2i_eb))
		return -EFAULT;

	d2iif->i0_in_d2i_eb = devm_clk_get(&pdev->dev, "i0_in_d2i_eb");
	if (IS_ERR_OR_NULL(d2iif->i0_in_d2i_eb))
		return -EFAULT;

	d2iif->i1_in_d2i_eb = devm_clk_get(&pdev->dev, "i1_in_d2i_eb");
	if (IS_ERR_OR_NULL(d2iif->i1_in_d2i_eb))
		return -EFAULT;

	d2iif->i2_in_d2i_eb = devm_clk_get(&pdev->dev, "i2_in_d2i_eb");
	if (IS_ERR_OR_NULL(d2iif->i2_in_d2i_eb))
		return -EFAULT;

	return 0;
}

static int sprd_dcam2isp_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id sprd_dcam2isp_of_match[] = {
	{ .compatible = "sprd,dcam2isp", },
	{},
};

static struct platform_driver sprd_dcam2isp_driver = {
	.probe = sprd_dcam2isp_probe,
	.remove = sprd_dcam2isp_remove,
	.driver = {
		.name = DCAM2ISP_DEVICE_NAME,
		.of_match_table = of_match_ptr(sprd_dcam2isp_of_match),
	},
};

module_platform_driver(sprd_dcam2isp_driver);

MODULE_DESCRIPTION("Sprd Dcam2isp Driver");
MODULE_LICENSE("GPL");
