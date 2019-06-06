/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#include <linux/clk.h>
#include "sprd_chip_common.h"
#include <video/disp_pw_domain.h>
#include <linux/mfd/syscon/sprd/sc9833/sc9833_glb.h>

#define REG_DISP_AHB_AHB_RST					0x0004

#define BIT_DISP_AHB_DISPC0_SOFT_RST				BIT(1)
#define BIT_DISP_AHB_DSI0_SOFT_RST				BIT(0)

static struct dispc_clk_context {
	uint32_t clk_src[DISPC_CLK_ID_MAX];
	struct clk *clk_dispc_core_parent;
	struct clk *clk_dispc_dpi_parent;
	struct clk *clk_dispc_core;
	struct clk *clk_dispc_dpi;
} dispc_clk_ctx[2];

static struct dispc_glb_context {
	struct regmap *disp_ahb;
	struct clk *clk_dispc_ahb_eb;
} dispc_glb_ctx[2];

static struct dsi_glb_context {
	struct regmap *disp_ahb;
	struct clk *clk_dsi0_ahb_eb;
} dsi_glb_ctx[2];


/******************* DISPC global interface start *****************/

static int dispc_clk_parse_dt(struct dispc_context *ctx,
				struct device_node *np)
{
	int ret = 0;
	int status = 0;
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx[ctx->id];

	clk_ctx->clk_dispc_core_parent =
		of_clk_get_by_name(np, "clk_dispc_core_parent");
	clk_ctx->clk_dispc_dpi_parent =
		of_clk_get_by_name(np, "clk_dispc_dpi_parent");
	clk_ctx->clk_dispc_core =
		of_clk_get_by_name(np, "clk_dispc_core");
	clk_ctx->clk_dispc_dpi =
		of_clk_get_by_name(np, "clk_dispc_dpi");

	ret = of_property_read_u32_array(np,
			 "clock-src", clk_ctx->clk_src, 2);
	if (ret) {
		PERROR("read clock-src fail (%d)\n", ret);
		status |= -1;
	}

	ctx->dpi_clk_src = clk_ctx->clk_src[1];
	PRINT("the dpi clock source from dts is %d\n", ctx->dpi_clk_src);

	if (IS_ERR_OR_NULL(clk_ctx->clk_dispc_core_parent)) {
		PERROR("read clk_dispc_core_parent failed\n");
		clk_ctx->clk_dispc_core_parent = NULL;
		status |= -1;
	}

	if (IS_ERR_OR_NULL(clk_ctx->clk_dispc_dpi_parent)) {
		PERROR("read clk_dispc_dpi_parent failed\n");
		clk_ctx->clk_dispc_dpi_parent = NULL;
		status |= -1;
	}

	if (IS_ERR_OR_NULL(clk_ctx->clk_dispc_core)) {
		PERROR("read clk_dispc_core failed\n");
		clk_ctx->clk_dispc_core = NULL;
		status |= -1;
	}

	if (IS_ERR_OR_NULL(clk_ctx->clk_dispc_dpi)) {
		PERROR("read clk_dispc_dpi failed\n");
		clk_ctx->clk_dispc_dpi = NULL;
		status |= -1;
	}

	return status;
}


static int dispc_clk_init(struct dispc_context *ctx)
{
	int ret = 0;
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx[ctx->id];

	ret = clk_set_parent(clk_ctx->clk_dispc_core,
			clk_ctx->clk_dispc_core_parent);
	if (ret)
		PERROR("dispc set clk parent fail\n");

	ret = clk_set_parent(clk_ctx->clk_dispc_dpi,
			clk_ctx->clk_dispc_dpi_parent);
	if (ret)
		PERROR("dispc set dpi clk parent fail\n");

	return ret;
}


static int dispc_clk_enable(struct dispc_context *ctx)
{
	int ret = -1;
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx[ctx->id];

	ret = clk_prepare_enable(clk_ctx->clk_dispc_core);
	if (ret) {
		PERROR("enable clk_dispc_core error\n");
		goto ERR_CLK_DISPC_CORE;
	}
	ret = clk_prepare_enable(clk_ctx->clk_dispc_dpi);
	if (ret) {
		PERROR("enable clk_dispc_dpi error\n");
		goto ERR_CLK_DISPC_DPI;
	}

	return 0;

ERR_CLK_DISPC_DPI:
	clk_disable_unprepare(clk_ctx->clk_dispc_dpi);
ERR_CLK_DISPC_CORE:
	clk_disable_unprepare(clk_ctx->clk_dispc_core);

	return ret;
}

static int dispc_clk_disable(struct dispc_context *ctx)
{
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx[ctx->id];

	clk_disable_unprepare(clk_ctx->clk_dispc_dpi);
	clk_disable_unprepare(clk_ctx->clk_dispc_core);

	clk_set_parent(clk_ctx->clk_dispc_dpi, NULL);
	clk_set_parent(clk_ctx->clk_dispc_core, NULL);

	return 0;
}


static int dispc_clk_update(struct dispc_context *ctx, int clk_id, int val)
{
	int ret = -1;
	struct dispc_clk_context *clk_ctx = &dispc_clk_ctx[ctx->id];

	switch (clk_id) {
	case DISPC_CLK_ID_CORE:
		PERROR("dispc core clk doesn't support update\n");
		break;

	case DISPC_CLK_ID_DPI:
		PRINT("dpi_clk = %d\n", val);
		ret = clk_set_rate(clk_ctx->clk_dispc_dpi, val);
		if (ret)
			PERROR("dispc update dbi clk rate fail\n");
		break;

	default:
		PERROR("clk id %d doesn't support\n", clk_id);
		break;
	}

	return ret;
}

static int dispc_glb_parse_dt(struct dispc_context *ctx,
				struct device_node *np)
{
	int status = 0;
	struct dispc_glb_context *glb_ctx = &dispc_glb_ctx[ctx->id];

	glb_ctx->disp_ahb = syscon_regmap_lookup_by_compatible(
					    "sprd,sys-ap-ahb");
	glb_ctx->clk_dispc_ahb_eb =
		of_clk_get_by_name(np, "clk_dispc_ahb_eb");

	if (IS_ERR_OR_NULL(glb_ctx->clk_dispc_ahb_eb)) {
		PERROR("read clk_dispc_ahb_eb failed\n");
		glb_ctx->clk_dispc_ahb_eb = NULL;
		status |= -1;
	}
	return status;
}

static void dispc_glb_enable(struct dispc_context *ctx)
{
	int ret = 0;
	struct dispc_glb_context *glb_ctx = &dispc_glb_ctx[ctx->id];

	ret = clk_prepare_enable(glb_ctx->clk_dispc_ahb_eb);
	if (ret) {
		PERROR("enable clk_dispc_ahb_eb failed!\n");
		goto ERR_CLK_DISPC_AHB_EB;
	}
	return;
ERR_CLK_DISPC_AHB_EB:
	clk_disable_unprepare(glb_ctx->clk_dispc_ahb_eb);
}

static void dispc_glb_disable(struct dispc_context *ctx)
{
	struct dispc_glb_context *glb_ctx = &dispc_glb_ctx[ctx->id];

	clk_disable_unprepare(glb_ctx->clk_dispc_ahb_eb);
}

static void dispc_reset(struct dispc_context *ctx)
{
	struct dispc_glb_context *glb_ctx = &dispc_glb_ctx[ctx->id];

	regmap_update_bits(glb_ctx->disp_ahb,
		    REG_DISP_AHB_AHB_RST, BIT_DISP_AHB_DISPC0_SOFT_RST,
		    BIT_DISP_AHB_DISPC0_SOFT_RST);
	udelay(10);
	regmap_update_bits(glb_ctx->disp_ahb,
		    REG_DISP_AHB_AHB_RST, BIT_DISP_AHB_DISPC0_SOFT_RST,
		    (unsigned int)(~BIT_DISP_AHB_DISPC0_SOFT_RST));
}

static void dispc_power_domain(struct dispc_context *ctx, int enable)
{
}

static void dispc_noc_ctrl(struct dispc_context *ctx, uint32_t mode)
{
}

/******************* DSI global interface start *****************/

static int dsi_glb_parse_dt(struct dsi_context *ctx,
				struct device_node *np)
{
	int status = 0;
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx[ctx->id];

	glb_ctx->disp_ahb = syscon_regmap_lookup_by_compatible(
					    "sprd,sys-ap-ahb");

	glb_ctx->clk_dsi0_ahb_eb =
		of_clk_get_by_name(np, "clk_dsi0_ahb_eb");

	if (IS_ERR_OR_NULL(glb_ctx->clk_dsi0_ahb_eb)) {
		PERROR("read clk_dsi0_ahb_eb failed\n");
		glb_ctx->clk_dsi0_ahb_eb = NULL;
		status |= -1;
	}
	return status;
}

static void dsi_glb_enable(struct dsi_context *ctx)
{
	int ret = 0;
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx[ctx->id];

	ret = clk_prepare_enable(glb_ctx->clk_dsi0_ahb_eb);
	if (ret) {
		PERROR("enable clk_dsi0_ahb_eb failed!\n");
		goto ERR_CLK_DSI_AHB_EB;
	}

	return;
ERR_CLK_DSI_AHB_EB:
	clk_disable_unprepare(glb_ctx->clk_dsi0_ahb_eb);
}

static void dsi_glb_disable(struct dsi_context *ctx)
{
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx[ctx->id];

	clk_disable_unprepare(glb_ctx->clk_dsi0_ahb_eb);
}

static void dsi_reset(struct dsi_context *ctx)
{
	struct dsi_glb_context *glb_ctx = &dsi_glb_ctx[ctx->id];

	regmap_update_bits(glb_ctx->disp_ahb,
		    REG_DISP_AHB_AHB_RST, BIT_DISP_AHB_DSI0_SOFT_RST,
		    BIT_DISP_AHB_DSI0_SOFT_RST);
	udelay(10);
	regmap_update_bits(glb_ctx->disp_ahb,
		    REG_DISP_AHB_AHB_RST, BIT_DISP_AHB_DSI0_SOFT_RST,
		    (unsigned int)(~BIT_DISP_AHB_DSI0_SOFT_RST));
}


static void dsi_power_domain(struct dsi_context *ctx, int enable)
{
}

/* DISPC module */
static struct dispc_clk_ops dispc_clk_ops = {
	.parse_dt = dispc_clk_parse_dt,
	.init = dispc_clk_init,
	.enable = dispc_clk_enable,
	.disable = dispc_clk_disable,
	.update = dispc_clk_update,
};

static struct dispc_glb_ops dispc_glb_ops = {
	.parse_dt = dispc_glb_parse_dt,
	.reset = dispc_reset,
	.enable = dispc_glb_enable,
	.disable = dispc_glb_disable,
	.power = dispc_power_domain,
	.noc = dispc_noc_ctrl,
};

static struct dispc_ops sc9835_dispc_ops = {
	.core = &dpu_lite_r1p0_ops,
	.clk = &dispc_clk_ops,
	.glb = &dispc_glb_ops,
};


/* DSI module */
static struct dsi_glb_ops dsi_glb_ops = {
	.parse_dt = dsi_glb_parse_dt,
	.reset = dsi_reset,
	.enable = dsi_glb_enable,
	.disable = dsi_glb_disable,
	.power = dsi_power_domain,
};

static struct dsi_ops sc9835_dsi_ops = {
	.core = NULL,
	.clk = NULL,
	.glb = &dsi_glb_ops,
};

struct sprd_hw_operations sc9835_hw_ops = {
	.platform_id = 0x9835,
	.dispc = &sc9835_dispc_ops,
	.dsi = &sc9835_dsi_ops,
};
