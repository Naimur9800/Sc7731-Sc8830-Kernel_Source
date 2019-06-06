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

#include "cam_common.h"
#include "isp2dcam.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP2DCAM: %d: %d " fmt, current->pid, __LINE__

#define ISP2DCAM_DEVICE_NAME			"isp2dcam"

/* Structure Definitions */

struct dump_info {
	unsigned int addr;
	unsigned int length;
};

struct dump_info isp2dcam_dump_info[] = {
	{ISP2DCAM_INT_STATUS, 3},
	{ISP2DCAM0_STATUS0, 3},
};

struct isp2dcam_device {
	struct isp2dcam_cfg_parm cfg_parm;
	void __iomem *io_base;

	struct clk *i2d_clk;
	struct clk *i2d_clk_default;
	struct clk *i2d_clk_parent;
	struct clk *i2d_if_eb;
	struct clk *i2d_if_axi_eb;
};

static struct isp2dcam_device *i2dif;

/* Internal Function Implementation */

static void isp2dcam_int_clr(unsigned int mask)
{
	reg_owr(i2dif, ISP2DCAM_INT_CLR0, mask);
	udelay(50);
	reg_awr(i2dif, ISP2DCAM_INT_CLR0, ~mask);
}

static void isp2dcam_int_enable(unsigned int mask)
{
	reg_owr(i2dif, ISP2DCAM_INT_EN0, mask);
}
#if 0
static void isp2dcam_parm_init(struct isp2dcam_cfg_parm *cfg_parm)
{
	struct isp2dcam_size slice_size = { 1920, 1080 };

	cfg_parm->isp0_eb = 1;
	cfg_parm->isp1_eb = 0;
	memcpy(&cfg_parm->slice_size[0],
	       &slice_size, sizeof(struct isp2dcam_size));
}
#endif


void sprd_isp2dcam_dump_reg(void)
{
	int i = 0, j = 0;
	unsigned int offset = 0;

	pr_info("Begin to dump isp2dcam regs:\n");

	for (j = 0; j < ARRAY_SIZE(isp2dcam_dump_info); j++) {
		for (i = 0; i < isp2dcam_dump_info[j].length; i++) {
			offset = isp2dcam_dump_info[j].addr + 0x10 * i;
			pr_info("0x%lx: 0x%.8x 0x%.8x 0x%.8x 0x%.8x\n",
				(unsigned long)i2dif->io_base + offset,
				reg_rd(i2dif, offset + 0x00),
				reg_rd(i2dif, offset + 0x04),
				reg_rd(i2dif, offset + 0x08),
				reg_rd(i2dif, offset + 0x0c)
				);
		}
	}
}
EXPORT_SYMBOL(sprd_isp2dcam_dump_reg);

static void isp2dcam_parm_set(struct isp2dcam_cfg_parm *cfg_parm)
{
	struct isp2dcam_size *slice_size = NULL;

	if (cfg_parm->dcam0_eb) {
		slice_size = &cfg_parm->slice_size[0];
		reg_wr(i2dif,
		       ISP2DCAM0_SLICE_SIZE,
		       slice_size->width | (slice_size->height << 16));
	}

	if (cfg_parm->dcam1_eb) {
		slice_size = &cfg_parm->slice_size[1];
		reg_wr(i2dif,
		       ISP2DCAM1_SLICE_SIZE,
		       slice_size->width | (slice_size->height << 16));
	}
	/*set yuv endian*/
	reg_wr(i2dif, ISP2DCAM_RESERVED, 2);

	pr_info("cfg w %d h %d, w %d h %d, %d %d\n",
		cfg_parm->slice_size[0].width,
		cfg_parm->slice_size[0].height,
		cfg_parm->slice_size[1].width,
		cfg_parm->slice_size[1].height,
		cfg_parm->dcam0_eb,
		cfg_parm->dcam1_eb);

}

void sprd_isp2dcam_config_parm(struct isp2dcam_cfg_parm *cfg_parm)
{
	isp2dcam_int_clr(0xffffffff);
	isp2dcam_int_enable(0xffffffff);
	isp2dcam_parm_set(cfg_parm);
}
EXPORT_SYMBOL(sprd_isp2dcam_config_parm);

void sprd_isp2dcam_enable(void)
{
	clk_prepare_enable(i2dif->i2d_if_eb);
	clk_prepare_enable(i2dif->i2d_if_axi_eb);

	clk_set_parent(i2dif->i2d_clk, i2dif->i2d_clk_parent);
	clk_prepare_enable(i2dif->i2d_clk);
}
EXPORT_SYMBOL(sprd_isp2dcam_enable);

void sprd_isp2dcam_disable(void)
{
	clk_set_parent(i2dif->i2d_clk, i2dif->i2d_clk_default);
	clk_disable_unprepare(i2dif->i2d_clk);

	clk_disable_unprepare(i2dif->i2d_if_axi_eb);
	clk_disable_unprepare(i2dif->i2d_if_eb);
}
EXPORT_SYMBOL(sprd_isp2dcam_disable);

static int sprd_isp2dcam_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;

	i2dif = devm_kzalloc(&pdev->dev, sizeof(*i2dif), GFP_KERNEL);
	if (!i2dif)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	i2dif->io_base = devm_ioremap_nocache(&pdev->dev, res->start,
					      resource_size(res));
	if (IS_ERR(i2dif->io_base))
		return PTR_ERR(i2dif->io_base);

	i2dif->i2d_clk = devm_clk_get(&pdev->dev, "clk_isp2dcam");
	if (IS_ERR(i2dif->i2d_clk))
		return PTR_ERR(i2dif->i2d_clk);

	i2dif->i2d_clk_parent = devm_clk_get(&pdev->dev, "clk_isp2dcam_parent");
	if (IS_ERR(i2dif->i2d_clk_parent))
		return PTR_ERR(i2dif->i2d_clk_parent);

	i2dif->i2d_if_eb = devm_clk_get(&pdev->dev, "isp2dcam_if_eb");
	if (IS_ERR(i2dif->i2d_if_eb))
		return PTR_ERR(i2dif->i2d_if_eb);

	i2dif->i2d_if_axi_eb = devm_clk_get(&pdev->dev, "i2d_if_axi_gate");
	if (IS_ERR(i2dif->i2d_if_axi_eb))
		return PTR_ERR(i2dif->i2d_if_axi_eb);

	i2dif->i2d_clk_default = clk_get_parent(i2dif->i2d_clk);
	if (IS_ERR(i2dif->i2d_clk_default))
		return PTR_ERR(i2dif->i2d_clk_default);

	return 0;
}

static int sprd_isp2dcam_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id of_match_table_isp2dcam[] = {
	{ .compatible = "sprd,isp2dcam", },
	{},
};

static struct platform_driver sprd_isp2dcam_driver = {
	.probe = sprd_isp2dcam_probe,
	.remove = sprd_isp2dcam_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = ISP2DCAM_DEVICE_NAME,
		.of_match_table = of_match_ptr(of_match_table_isp2dcam),
	},
};

static int __init sprd_isp2dcam_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&sprd_isp2dcam_driver);
	pr_info("init ret %d\n", ret);

	return ret;
}

static void sprd_isp2dcam_exit(void)
{
	platform_driver_unregister(&sprd_isp2dcam_driver);
}

module_init(sprd_isp2dcam_init);
module_exit(sprd_isp2dcam_exit);

MODULE_DESCRIPTION("Isp2dcam Driver");
MODULE_LICENSE("GPL");
