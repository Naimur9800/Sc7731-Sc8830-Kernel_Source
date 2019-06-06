/*
 *Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 *This software is licensed under the terms of the GNU General Public
 *License version 2, as published by the Free Software Foundation, and
 *may be copied, distributed, and modified under those terms.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqreturn.h>
#include <linux/kgdb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_graph.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include "sprd_dsi.h"
#include "disp_notify.h"
#include "dsi/mipi_dsi_api.h"
#include "sysfs/sysfs_display.h"

#define DSI_PHY_REF_CLOCK (26*1000)

#define MAX_TIME_HS2LP       120 /* unit: ns */
#define MAX_TIME_LP2HS       500 /* unit: ns */

LIST_HEAD(dsi_core_head);
LIST_HEAD(dsi_glb_head);

static uint32_t bpp_to_coding(uint32_t bits, bool compression)
{
	uint32_t code = 0;

	switch (bits) {
	case 16:
		code = COLOR_CODE_16BIT_CONFIG1;
		break;
	case 18:
		code = COLOR_CODE_18BIT_CONFIG1;
		break;
	case 24:
		if (compression)
			code = COLOR_CODE_COMPRESSTION;
		else
			code = COLOR_CODE_24BIT;
		break;
	default:
		pr_err("invalid bus width\n");
		break;
	}

	return code;
}

static int32_t dsi_dpi_video(struct sprd_dsi *dsi)
{
	struct dpi_video_param param = {};
	uint32_t hs2lp, lp2hs;
	struct panel_info *panel = dsi->panel;
	struct rgb_timing *timing = &panel->rgb_timing;

	param.burst_mode = panel->burst_mode;
	param.lanes = panel->lane_num;
	param.byte_clk = panel->phy_freq / 8;
	param.pixel_clk = panel->pixel_clk / 1000;

	hs2lp = MAX_TIME_HS2LP * panel->phy_freq / 8000000;
	hs2lp += (MAX_TIME_HS2LP * panel->phy_freq % 8000000) < 4000000 ? 0 : 1;
	lp2hs = MAX_TIME_LP2HS * panel->phy_freq / 8000000;
	lp2hs += (MAX_TIME_LP2HS * panel->phy_freq % 8000000) < 4000000 ? 0 : 1;
	param.data_hs2lp = hs2lp;
	param.data_lp2hs = lp2hs;
	param.clk_hs2lp = 4;
	param.clk_lp2hs = 15;
	param.coding = bpp_to_coding(panel->bpp, panel->dsc_en);

	param.hact = panel->width >> panel->bv3_en;
	param.hsync = timing->hsync;
	param.hbp = timing->hbp;
	param.hline = panel->width
		+ timing->hsync + timing->hbp + timing->hfp;

	param.vact = panel->height;
	param.vsync = timing->vsync;
	param.vbp = timing->vbp;
	param.vfp = timing->vfp;

	param.color_mode_pol = 0;
	param.shut_down_pol = 0;
	param.frame_ack_en = 0;
	param.channel = 0;
	param.is_18_loosely = 1;

	mipi_dsi_dpi_video(dsi, &param);

	return 0;
}

static int32_t dsi_edpi_video(struct sprd_dsi *dsi)
{
	struct edpi_video_param param = {};
	struct panel_info *panel = dsi->panel;

	param.coding = bpp_to_coding(panel->bpp, panel->dsc_en);
	param.hact = panel->width;

	mipi_dsi_edpi_video(dsi, &param);

	return 0;
}

static irqreturn_t sprd_dsi_isr0(int irq, void *data)
{
	struct sprd_dsi *dsi = data;
	u32 status;

	if (!dsi) {
		pr_err("dsi pointer is NULL\n");
		return IRQ_HANDLED;
	}

	status = mipi_dsi_int_status(dsi, 0);

	if (status & DSI_INT_STS_NEED_SOFT_RESET)
		mipi_dsi_state_reset(dsi);

	return IRQ_HANDLED;
}

static irqreturn_t sprd_dsi_isr1(int irq, void *data)
{
	struct sprd_dsi *dsi = data;
	u32 status;

	if (!dsi) {
		pr_err("dsi pointer is NULL\n");
		return IRQ_HANDLED;
	}

	status = mipi_dsi_int_status(dsi, 1);

	if (status & DSI_INT_STS_NEED_SOFT_RESET)
		mipi_dsi_state_reset(dsi);

	return IRQ_HANDLED;
}

static int sprd_dsi_init(struct sprd_dsi *dsi)
{
	if (dsi->glb && dsi->glb->power)
		dsi->glb->power(&dsi->ctx, true);
	if (dsi->glb && dsi->glb->enable)
		dsi->glb->enable(&dsi->ctx);

	return 0;
}

static int sprd_dsi_resume(struct sprd_dsi *dsi)
{
	struct panel_info *panel = dsi->panel;

	if (dsi->glb && dsi->glb->power)
		dsi->glb->power(&dsi->ctx, true);
	if (dsi->glb && dsi->glb->enable)
		dsi->glb->enable(&dsi->ctx);
	if (dsi->glb && dsi->glb->reset)
		dsi->glb->reset(&dsi->ctx);

	mipi_dsi_init(dsi);

	if (panel->work_mode == SPRD_MIPI_MODE_VIDEO)
		dsi_dpi_video(dsi);
	else
		dsi_edpi_video(dsi);

	pr_info("dsi resume OK\n");
	return 0;
}

static int sprd_dsi_suspend(struct sprd_dsi *dsi)
{
	mipi_dsi_uninit(dsi);

	if (dsi->glb && dsi->glb->disable)
		dsi->glb->disable(&dsi->ctx);
	if (dsi->glb && dsi->glb->power)
		dsi->glb->power(&dsi->ctx, false);

	pr_info("dsi suspend OK\n");
	return 0;
}

static int dsi_notify_callback(struct notifier_block *nb,
			unsigned long action, void *data)
{
	struct sprd_dsi *dsi = container_of(nb, struct sprd_dsi, nb);
	struct panel_info *panel = dsi->panel;
	struct device *dev = dsi->dev.parent;
	struct dpi_video_param param = {};
	struct rgb_timing *timing;
	unsigned long enable;

	if (!panel) {
		pr_err("panel pointer is null\n");
		return NOTIFY_DONE;
	}

	switch (action) {
	case DISP_EVENT_ESD_RECOVER:
		enable = (unsigned long)data;
		if (enable)
			pm_runtime_get_sync(dev);
		else
			pm_runtime_put_sync(dev);
		break;

	case DISP_EVENT_CLOCK_CHANGE:
		timing = &panel->rgb_timing;

		param.byte_clk = panel->phy_freq / 8;
		param.pixel_clk = panel->pixel_clk / 1000;
		param.hsync = timing->hsync;
		param.hbp = timing->hbp;
		param.hline = panel->width
			+ timing->hsync + timing->hbp + timing->hfp;

		mipi_dsi_htime_update(dsi, &param);

		break;
	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static int dsi_notifier_register(struct sprd_dsi *dsi)
{
	dsi->nb.notifier_call = dsi_notify_callback;

	return disp_notifier_register(&dsi->nb);
}

static int dsi_device_register(struct sprd_dsi *dsi,
				struct device *parent)
{
	int ret;

	dsi->dev.class = display_class;
	dsi->dev.parent = parent;
	dsi->dev.of_node = parent->of_node;
	dev_set_name(&dsi->dev, "dsi%d", dsi->ctx.id);
	dev_set_drvdata(&dsi->dev, dsi);

	ret = device_register(&dsi->dev);
	if (ret)
		pr_err("dsi device register failed\n");

	return ret;
}

static int dsi_context_init(struct sprd_dsi *dsi, struct device_node *np)
{
	struct panel_info *panel = dsi->panel;
	struct dsi_context *ctx = &dsi->ctx;
	struct resource r;
	uint32_t tmp;

	if (dsi->glb && dsi->glb->parse_dt)
		dsi->glb->parse_dt(&dsi->ctx, np);

	if (of_address_to_resource(np, 0, &r)) {
		pr_err("parse dsi ctrl reg base failed\n");
		return -ENODEV;
	}
	ctx->base = (unsigned long)
	    ioremap_nocache(r.start, resource_size(&r));
	if (ctx->base == 0) {
		pr_err("dsi ctrl reg base ioremap failed\n");
		return -ENODEV;
	}

	if (!of_property_read_u32(np, "dev-id", &tmp))
		ctx->id = tmp;

	if (!of_property_read_u32(np, "sprd,max-read-time", &tmp))
		ctx->max_rd_time = tmp;
	else
		ctx->max_rd_time = 0x6000;

	if (!of_property_read_u32(np, "sprd,int0_mask", &tmp))
		ctx->int0_mask = tmp;
	else
		ctx->int0_mask = 0xffffffff;

	if (!of_property_read_u32(np, "sprd,int1_mask", &tmp))
		ctx->int1_mask = tmp;
	else
		ctx->int1_mask = 0xffffffff;

	ctx->freq = panel->phy_freq;
	ctx->lanes = panel->lane_num;
	ctx->dln_timer = panel->dln_timer;
	ctx->nc_clk_en = panel->nc_clk_en;

	return 0;
}

static int dsi_irq_init(struct sprd_dsi *dsi)
{
	int ret;
	int irq0, irq1;

	irq0 = irq_of_parse_and_map(dsi->dev.of_node, 0);
	if (irq0) {
		pr_info("dsi irq0 num = %d\n", irq0);
		ret = request_irq(irq0, sprd_dsi_isr0, 0, "DSI_INT0", dsi);
		if (ret) {
			pr_err("dsi failed to request irq int0!\n");
			return -EINVAL;
		}
	}

	irq1 = irq_of_parse_and_map(dsi->dev.of_node, 1);
	if (irq1) {
		pr_info("dsi irq1 num = %d\n", irq1);
		ret = request_irq(irq1, sprd_dsi_isr1, 0, "DSI_INT1", dsi);
		if (ret) {
			pr_err("dsi failed to request irq int1!\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int sprd_dsi_probe(struct platform_device *pdev)
{
	const char *str;
	struct device *dispc_dev;
	struct sprd_dsi *dsi;
	struct device_node *np = pdev->dev.of_node;

	dsi = kzalloc(sizeof(struct sprd_dsi), GFP_KERNEL);
	if (dsi == NULL)
		return -ENOMEM;

	dsi->panel = platform_get_drvdata(pdev);
	if (dsi->panel == NULL) {
		pr_err("error: dsi->panel is null\n");
		goto err;
	}
	dispc_dev = dev_get_prev(&pdev->dev);
	if (dispc_dev)
		dev_set_drvdata(dispc_dev, dsi->panel);

	if (!of_property_read_string(np, "sprd,ip", &str))
		dsi->core = dsi_core_ops_attach(str);
	else
		pr_err("error: \"sprd,ip\" was not found\n");

	if (!of_property_read_string(np, "sprd,soc", &str))
		dsi->glb = dsi_glb_ops_attach(str);
	else
		pr_err("error: \"sprd,soc\" was not found\n");

	if (dsi_context_init(dsi, np))
		goto err;

	dsi_device_register(dsi, &pdev->dev);
	sprd_dsi_sysfs_init(&dsi->dev);
	dsi_notifier_register(dsi);
	platform_set_drvdata(pdev, dsi);

	sprd_dsi_init(dsi);
	dsi_irq_init(dsi);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	pr_info("dsi driver probe success\n");

	return 0;

err:
	kfree(dsi);
	return -ENODEV;
}

static int dsi_runtime_resume(struct device *dev)
{
	struct sprd_dsi *dsi = dev_get_drvdata(dev);
	struct device *next = dev_get_next(dev);

	sprd_dsi_resume(dsi);
	pm_runtime_get_sync(next);

	return 0;
}

static int dsi_runtime_suspend(struct device *dev)
{
	struct sprd_dsi *dsi = dev_get_drvdata(dev);
	struct device *next = dev_get_next(dev);

	pm_runtime_put_sync(next);
	sprd_dsi_suspend(dsi);

	return 0;
}

static const struct dev_pm_ops dsi_pm_ops = {
	.runtime_suspend = dsi_runtime_suspend,
	.runtime_resume = dsi_runtime_resume,
};

static const struct of_device_id dt_ids[] = {
	{ .compatible = "sprd,dsi-controller", },
	{}
};

static struct platform_driver sprd_dsi_driver = {
	.probe = sprd_dsi_probe,
	.driver = {
		.name = "sprd-dsi",
		.of_match_table = of_match_ptr(dt_ids),
		.pm = &dsi_pm_ops,
	},
};

module_platform_driver(sprd_dsi_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leon.he@spreadtrum.com");
MODULE_AUTHOR("infi.chen@spreadtrum.com");
MODULE_DESCRIPTION("SPRD DSI Controller Platform Driver");
