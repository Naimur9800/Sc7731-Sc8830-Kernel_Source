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
#include <asm/cacheflush.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/dma-buf.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include "ion.h"
#include "sprd_adf_adapter.h"
#include "sprd_dispc.h"
#include "../sprd_spi_intf.h"

static bool evt_te;
unsigned int flag_mask_refresh;
struct sprd_adf_hwlayer *hwlayer_lastest;

static int32_t swdispc_wait_te(struct dispc_context *ctx)
{
	int rc;

	evt_te = false;
	/*wait for reg update done interrupt*/
	rc = wait_event_interruptible_timeout(ctx->wait_queue, evt_te,
					       msecs_to_jiffies(500));
	if (!rc) {
		/* time out */
		pr_err("dpu wait for reg update done time out!\n");
		return -1;
	}

	return 0;
}

static int swdispc_disable_te(struct dispc_context *ctx)
{
	struct panel_info *panel;
	struct panel_device *pd;

	panel = ctx->panel;
	pd = panel->pd;
	pd->ops->disable_te(pd);
	pr_info("TE disable\n");

	return 0;
}

static int swdispc_enable_te(struct dispc_context *ctx)
{
	struct panel_info *panel;
	struct panel_device *pd;

	panel = ctx->panel;
	pd = panel->pd;
	pd->ops->enable_te(pd);
	pr_info("TE enable\n");

	return 0;
}

static u32 swdispc_get_version(struct dispc_context *ctx)
{
	return 0x0100;
}

static u32 swdispc_isr(struct dispc_context *ctx)
{
	u32 reg_val = 0;

	reg_val = DISPC_INT_TE_MASK;
	evt_te = true;
	wake_up_interruptible_all(&ctx->wait_queue);
	return reg_val;
}

static void swdispc_layer(struct dispc_context *ctx,
		    struct sprd_adf_hwlayer *hwlayer)
{
	if (!flag_mask_refresh) {
		u8 bg_flag = 0;

		sprd_spi_refresh(hwlayer, bg_flag);
	}
}

static void swdispc_intf_init(struct dispc_context *ctx)
{
	pr_info("swdispc intf inited");
}

static void swdispc_flip(struct dispc_context *ctx,
			struct sprd_restruct_config *config)
{
	struct sprd_adf_hwlayer *hwlayer;

	swdispc_wait_te(ctx);
	hwlayer = &config->hwlayers[0];
	hwlayer_lastest = hwlayer;
	swdispc_layer(ctx, hwlayer);
}

static void swdispc_run(struct dispc_context *ctx)
{
	pr_info("swdispc run now\n");
}

static void swdispc_stop(struct dispc_context *ctx)
{
	pr_info("swdispc stop now\n");
}

static void swdispc_bg_color(struct dispc_context *ctx, uint32_t color)
{
	if (!flag_mask_refresh) {
		u8 bg_flag = 1;

		swdispc_wait_te(ctx);
		sprd_spi_refresh(NULL, bg_flag);
	}
}

static int swdispc_init(struct dispc_context *ctx)
{
	pr_info("swdispc init\n");
	return 0;
}

static void swdispc_uninit(struct dispc_context *ctx)
{
	pr_info("swdispc uninit\n");

}

static struct dispc_core_ops swdispc_ops = {
	.version = swdispc_get_version,
	.init = swdispc_init,
	.run = swdispc_run,
	.stop = swdispc_stop,
	.isr = swdispc_isr,
	.flip = swdispc_flip,
	.ifconfig = swdispc_intf_init,
	.bg_color = swdispc_bg_color,
	.disable_vsync = swdispc_disable_te,
	.enable_vsync = swdispc_enable_te,
	.uninit = swdispc_uninit,
};

static struct ops_entry entry = {
	.ver = "swdispc",
	.ops = &swdispc_ops,
};

static int __init swdispc_core_register(void)
{
	return dispc_core_ops_register(&entry);
}

subsys_initcall(swdispc_core_register);
