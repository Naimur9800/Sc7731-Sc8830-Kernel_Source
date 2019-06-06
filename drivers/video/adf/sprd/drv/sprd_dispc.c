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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>
#include <linux/sprd_ion.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <video/adf.h>
#include <video/adf_format.h>

#include "ion.h"
#include "sprd_adf_adapter.h"
#include "sprd_dispc.h"
#include "disp_notify.h"
#include "sw_sync.h"
#include "sysfs/sysfs_display.h"
#include "zte_lcd_common.h"

LIST_HEAD(dispc_core_head);
LIST_HEAD(dispc_clk_head);
LIST_HEAD(dispc_glb_head);
LIST_HEAD(dispc_enc_head);

static unsigned long logo_base_from_uboot;
struct sprd_restruct_config *config_saved;

static int __init logo_base_get(char *str)
{
	int len = 0;

	if (str != NULL)
		len = kstrtoul(str, 16, &logo_base_from_uboot);

	pr_info("logo base from uboot: 0x%lx\n", logo_base_from_uboot);
	return 0;
}
__setup("lcd_base=", logo_base_get);

static void dispc_encoder_init(struct sprd_dispc *dispc)
{
	struct dispc_encoder *dsc = dispc->dsc;
	struct dispc_encoder *bv3 = dispc->bv3;
	struct panel_info *panel = dispc->ctx.panel;

	if (panel->dsc_en && dsc) {
		dsc->ops->init(&dsc->ctx);
		dsc->ops->enable(&dsc->ctx);
	}

	if (panel->bv3_en && bv3) {
		bv3->ops->init(&bv3->ctx);
		bv3->ops->enable(&bv3->ctx);
	}
}

static struct dispc_encoder *dispc_encoder_create(
				struct sprd_dispc *dispc, int id)
{
	struct resource r;
	struct device_node *endpoint;
	struct device_node *enc_node;
	struct dispc_encoder *encoder;
	struct device_node *np = dispc->dev.of_node;
	struct panel_info *panel = dispc->ctx.panel;
	const char *str;

	encoder = kzalloc(sizeof(struct dispc_encoder), GFP_KERNEL);
	if (!encoder)
		return NULL;
	encoder->ctx.panel = panel;

	endpoint = of_graph_get_endpoint_by_regs(np, 0, id);
	if (!endpoint) {
		pr_err("error: encoder endpoint was not found\n");
		goto err;
	}

	enc_node = of_graph_get_remote_port_parent(endpoint);
	if (!enc_node) {
		pr_err("error: get encoder device node failed\n");
		goto err;
	}

	if (of_address_to_resource(enc_node, 0, &r)) {
		pr_err("parse dt base address failed\n");
		goto err;
	}
	encoder->ctx.base = (unsigned long)ioremap_nocache(r.start,
					resource_size(&r));
	if (encoder->ctx.base == 0) {
		pr_err("ioremap base address failed\n");
		goto err;
	}

	if (!of_property_read_string(enc_node, "compatible", &str)) {
		encoder->ops = dispc_enc_ops_attach(str);
		if (encoder->ops == NULL)
			goto err;
	}

	return encoder;

err:
	kfree(encoder);
	return NULL;
}

static int dispc_encoder_probe(struct sprd_dispc *dispc)
{
	int ret = 0;
	struct dispc_encoder *encoder;
	struct panel_info *panel = dispc->ctx.panel;

	if (!panel)
		return -ENODEV;

	if (panel->dsc_en) {
		encoder = dispc_encoder_create(dispc, 1);
		if (!encoder)
			ret = -1;
		dispc->dsc = encoder;
	}

	if (panel->bv3_en) {
		encoder = dispc_encoder_create(dispc, 2);
		if (!encoder)
			ret = -1;
		dispc->bv3 = encoder;
	}

	return ret;
}

static int dispc_notify_callback(struct notifier_block *nb,
			unsigned long action, void *data)
{
	struct sprd_dispc *dispc = container_of(nb, struct sprd_dispc, nb);

	if (!dispc) {
		pr_err("dispc pointer is null\n");
		return NOTIFY_DONE;
	}

	switch (action) {
	case DISP_EVENT_DISPC_RUN:
		sprd_dispc_run(dispc);
		break;

	case DISP_EVENT_DISPC_BLACK:
		sprd_dispc_black(dispc);
		break;

	case DISP_EVENT_DISPC_STOP:
		sprd_dispc_stop(dispc);
		break;

	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static int dispc_notifier_register(struct sprd_dispc *dispc)
{
	dispc->nb.notifier_call = dispc_notify_callback;

	return disp_notifier_register(&dispc->nb);
}

static int dispc_device_register(struct sprd_dispc *dispc,
				struct device *parent)
{
	int ret;

	dispc->dev.class = display_class;
	dispc->dev.parent = parent;
	dispc->dev.of_node = parent->of_node;
	dev_set_name(&dispc->dev, "dispc%d", dispc->ctx.id);
	dev_set_drvdata(&dispc->dev, dispc);

	ret = device_register(&dispc->dev);
	if (ret)
		pr_err("dispc device register failed\n");

	return ret;
}

static int calc_dpi_clk(struct sprd_dispc *dispc,
			       u32 *new_pclk, u32 pclk_src,
			       u32 new_val, int type)
{
	int divider;
	u32 hpixels, vlines, pclk, fps;
	struct panel_info *panel = dispc->ctx.panel;

	pr_debug("%s: enter\n", __func__);
	if (!panel) {
		pr_err("No panel is specified!\n");
		return -ENXIO;
	}

	if (dispc->ctx.if_type == SPRD_DISPC_IF_EDPI) {
		pr_err("panel interface should be DPI\n");
		return -EINVAL;
	}

	if ((new_val == 0) || (new_pclk == NULL) || (pclk_src == 0)) {
		pr_err("input parameter is invalid\n");
		return -EINVAL;
	}

	if (panel->type == SPRD_PANEL_TYPE_MIPI) {
		struct rgb_timing *timing = &panel->rgb_timing;

		hpixels = panel->width + timing->hsync +
		    timing->hbp + timing->hfp;
		vlines = panel->height + timing->vsync +
		    timing->vbp + timing->vfp;
	} else if (panel->type == SPRD_PANEL_TYPE_RGB
		   || panel->type == SPRD_PANEL_TYPE_LVDS) {
		struct rgb_timing *timing = &panel->rgb_timing;

		hpixels = panel->width + timing->hsync +
		    timing->hbp + timing->hfp;
		vlines = panel->height + timing->vsync +
		    timing->vbp + timing->vfp;
	} else {
		pr_err("[%s] unexpected panel type (%d)\n",
		       __func__, panel->type);
		return -EINVAL;
	}

	switch (type) {
	case SPRD_FORCE_FPS:
	case SPRD_DYNAMIC_FPS:
		if (new_val < LCD_MIN_FPS || new_val > LCD_MAX_FPS) {
			pr_err
			    ("Unsupported FPS. fps range should be [%d, %d]\n",
			     LCD_MIN_FPS, LCD_MAX_FPS);
			return -EINVAL;
		}
		pclk = hpixels * vlines * new_val;
		divider = ROUND(pclk_src, pclk);
		*new_pclk = pclk_src / divider;
		if (pclk_src % divider)
			*new_pclk += 1;
		/* Save the updated fps */
		panel->fps = new_val;
		break;

	case SPRD_DYNAMIC_PCLK:
		divider = ROUND(pclk_src, new_val);
		pclk = pclk_src / divider;
		fps = pclk / (hpixels * vlines);
		if (fps < LCD_MIN_FPS || fps > LCD_MAX_FPS) {
			pr_err
			    ("Unsupported FPS. fps range should be [%d, %d]\n",
			     LCD_MIN_FPS, LCD_MAX_FPS);
			return -EINVAL;
		}
		*new_pclk = pclk;
		/* Save the updated fps */
		panel->fps = fps;
		break;

	case SPRD_FORCE_PCLK:
		*new_pclk = new_val;
		break;

	default:
		pr_err("This checked type is unsupported.\n");
		*new_pclk = 0;
		return -EINVAL;
	}
	return 0;
}

static int dispc_clk_update(struct sprd_dispc *dispc,
				u32 new_val, int howto)
{
	int ret;
	uint32_t pclk;
	struct dispc_context *ctx = &dispc->ctx;
	struct panel_info *panel = ctx->panel;

	if (!dispc->clk || !dispc->clk->update)
		return -EINVAL;

	ret = calc_dpi_clk(dispc, &pclk,
			ctx->dpi_clk_src, new_val, howto);
	if (ret) {
		pr_err("calc dpi clk failed!\n");
		return -EINVAL;
	}

	ret = dispc->clk->update(ctx, DISPC_CLK_ID_DPI, pclk);
	if (ret) {
		pr_err("Failed to set pixel clock.\n");
		return ret;
	}
	pr_info("dpi clock is switched from %dHz to %dHz\n",
		panel->pixel_clk, pclk);

	panel->pixel_clk = pclk;

	return 0;
}


static irqreturn_t sprd_dispc_isr(int irq, void *data)
{
	struct sprd_dispc *dispc = data;
	struct dispc_context *ctx = &dispc->ctx;
	struct panel_info *panel = ctx->panel;
	u32 int_mask;

	int_mask = dispc->core->isr(ctx);

	if (int_mask & DISPC_INT_ERR_MASK)
		pr_err("Warning: dispc underflow (0x%x)!\n", int_mask);

	if (int_mask & (DISPC_INT_DPI_VSYNC_MASK)) {
		if (ctx->disable_flip && ctx->disable_timeout) {
			if (ctx->disable_timeout > 0) {
				ctx->disable_timeout--;
				if (ctx->disable_timeout == 0)
					ctx->disable_flip = false;
			}
			if (!ctx->disable_flip)
				ctx->disable_timeout = 0;
		}
		ctx->vsync_ratio_to_panel--;
		if (ctx->vsync_ratio_to_panel == 0) {
			ctx->vsync_ratio_to_panel =
				panel->fps / ctx->vsync_report_rate;
			sprd_adf_report_vsync(dispc->adf,
				ADF_INTF_FLAG_PRIMARY);
			vsync_notifier_call_chain(ADF_EVENT_VSYNC,
				(void *)(unsigned long)ctx->vsync_report_rate);
		}
	}

	if (int_mask & DISPC_INT_TE_MASK) {
		if (panel->work_mode == SPRD_MIPI_MODE_CMD) {
			ctx->vsync_ratio_to_panel--;
			if (ctx->vsync_ratio_to_panel == 0) {
				ctx->vsync_ratio_to_panel =
					panel->fps / ctx->vsync_report_rate;
				sprd_adf_report_vsync(dispc->adf,
					ADF_INTF_FLAG_PRIMARY);
				vsync_notifier_call_chain(ADF_EVENT_VSYNC,
					(void *)(unsigned long)
					ctx->vsync_report_rate);
			}
		}
		if (panel->esd_check_en == ESD_MODE_WAIT_TE &&
		    panel->te_esd_waiter) {
			panel->te_esd_flag = true;
			wake_up_interruptible_all(&panel->wq_te_esd);
		}
	}
	if (int_mask & DISPC_INT_FENCE_SIGNAL_REQUEST)
		sprd_adf_fence_signal(dispc->adf);

	return IRQ_HANDLED;
}

void sprd_dispc_run(struct sprd_dispc *dispc)
{
	struct dispc_context *ctx = &dispc->ctx;

	down(&ctx->refresh_lock);

	if (!ctx->is_inited) {
		pr_err("dispc is not initialized!\n");
		up(&ctx->refresh_lock);
		return;
	}

	dispc->core->run(ctx);

	up(&ctx->refresh_lock);
}

void sprd_dispc_black(struct sprd_dispc *dispc)
{
	struct dispc_context *ctx = &dispc->ctx;

	down(&ctx->refresh_lock);

	if (!ctx->is_inited) {
		pr_err("dispc is not initialized!\n");
		up(&ctx->refresh_lock);
		return;
	}

	dispc->core->bg_color(ctx, 0);

	up(&ctx->refresh_lock);
}

void sprd_dispc_stop(struct sprd_dispc *dispc)
{
	struct dispc_context *ctx = &dispc->ctx;

	down(&ctx->refresh_lock);

	if (!ctx->is_inited) {
		pr_err("dispc is not initialized!\n");
		up(&ctx->refresh_lock);
		return;
	}

	dispc->core->stop(ctx);

	up(&ctx->refresh_lock);
}

static int32_t sprd_dispc_uninit(struct sprd_dispc *dispc)
{
	struct dispc_context *ctx = &dispc->ctx;

	if (dispc->core)
		dispc->core->uninit(ctx);
	if (dispc->clk)
		dispc->clk->disable(ctx);
	if (dispc->glb) {
		dispc->glb->disable(ctx);
		dispc->glb->power(ctx, false);
	}

	ctx->is_inited = false;

	return 0;
}

void sprd_dispc_iommu_map(struct sprd_dispc *dispc,
			  struct sprd_adf_hwlayer *layer)
{
	int i;
	bool fb_reserved;
	unsigned long phys_addr;
	size_t size;
	int ret;
	struct dma_buf *dmabuf;
	struct sprd_iommu_map_data iommu_data = {};

	for (i = 0; i < layer->n_planes; i++) {
		dmabuf = layer->base.dma_bufs[i];
		sprd_ion_is_reserved(-1, dmabuf, &fb_reserved);
		if (fb_reserved) {
			ret = sprd_ion_get_phys_addr(-1, dmabuf,
						&phys_addr, &size);
			if (ret)
				pr_err("get physical address error\n");
			else
				layer->iova_plane[i] =
					layer->iova_plane[i] + phys_addr;
		} else {
			iommu_data.iova_size = dmabuf->size;
			iommu_data.table = layer->mapping.sg_tables[i];
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;

			ret = sprd_iommu_get_kaddr(&dispc->dev, &iommu_data);
			if (ret) {
				pr_err("iommu addr error\n");
				goto map_fail;
			}
			if (iommu_data.iova_addr == 0) {
				ret = -1;
				pr_err("iommu iova_addr is 0\n");
				goto map_fail;
			}
			layer->iova_space[i].iova_addr =
				iommu_data.iova_addr;
			layer->iova_space[i].iova_size =
				iommu_data.iova_size;
			layer->iova_space[i].ch_type = iommu_data.ch_type;
			layer->iova_plane[i] =
				layer->iova_plane[i] + iommu_data.iova_addr;
		}
	}

	return;

map_fail:
	pr_err("iommu map failed\n");
}

void sprd_dispc_iommu_unmap(struct sprd_dispc *dispc,
			    struct sprd_adf_hwlayer *hwlayer)
{
	int i;
	bool fb_reserved;
	struct dma_buf *dmabuf;
	struct dispc_context *ctx = &dispc->ctx;

	if (!ctx->is_inited) {
		pr_err("dispc is not initialized from unmap!\n");
		return;
	}

	pr_debug("iommu unmap\n");
	for (i = 0; i < hwlayer->n_planes; i++) {
		dmabuf = hwlayer->base.dma_bufs[i];
		sprd_ion_is_reserved(-1, dmabuf, &fb_reserved);
		if (!fb_reserved)
			sprd_iommu_free_kaddr(&dispc->dev,
					      &hwlayer->iova_space[i]);
	}
}

int sprd_dispc_wb_buf_alloc(struct sprd_dispc *dispc,
			    int heap_type, size_t *size, u32 *buffer)
{
	struct dispc_context *ctx = &dispc->ctx;
	struct sprd_iommu_map_data iommu_data = {};
	ion_phys_addr_t vaddr = {0};
	void *kvaddr;
	int ret;

	if (!ctx->buffer_client)
		ctx->buffer_client =
			sprd_ion_client_create("sprd-dispc");

	if (!ctx->buffer_client) {
		pr_err("ion client create fail\n");
		return -ENOMEM;
	}

	ctx->handle = ion_alloc(ctx->buffer_client, (size_t)(*size), 0,
			heap_type, 0);
	if (IS_ERR(ctx->handle)) {
		pr_err("ion_alloc handle fail\n");
		return -ENOMEM;
	}

	if (heap_type == ION_HEAP_ID_MASK_SYSTEM) {
		iommu_data.iova_size = *size;
		iommu_data.table =
			ion_sg_table(ctx->buffer_client, ctx->handle);
		iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		ret = sprd_iommu_get_kaddr(&dispc->dev, &iommu_data);
		if (ret) {
			pr_err("iommu addr error\n");
			return -ENOMEM;
		}
		if (iommu_data.iova_addr == 0) {
			ret = -ENOMEM;
			return ret;
		}
		vaddr = iommu_data.iova_addr;
	} else {
		ret = ion_phys(ctx->buffer_client, ctx->handle, &vaddr, size);
		if (!vaddr || ret) {
			pr_err("Failed to allocate write back buffer\n");
			return -ENOMEM;
		}
	}
	pr_info("allocated for write back buffer:0x%x type %d\n",
	      (u32)vaddr, heap_type);
	*buffer = (u32)vaddr;

	/*clear memory*/
	kvaddr = ion_map_kernel(ctx->buffer_client, ctx->handle);
	memset(kvaddr, 0, *size);
	ion_unmap_kernel(ctx->buffer_client, ctx->handle);

	return 0;
}

static int32_t sprd_swdispc_init(struct sprd_dispc *dispc)
{
	struct dispc_context *ctx = &dispc->ctx;
	struct panel_info *panel = ctx->panel;

	if (panel == NULL)
		return -ENODEV;
	pr_info("sprd_swdispc_init success\n");
	ctx->is_inited = true;
	return 0;
}

static int32_t sprd_dispc_init(struct sprd_dispc *dispc)
{
	int32_t ret = 0;
	struct dispc_context *ctx = &dispc->ctx;
	struct panel_info *panel = ctx->panel;

	dispc->glb->power(ctx, true);
	dispc->glb->enable(ctx);

	if (panel == NULL) {
		pr_err("Calibration Mode, disable DISP_EB bit\n");
		dispc->glb->disable(ctx);
		dispc->glb->power(ctx, false);
		return -ENODEV;
	}

	dispc->clk->init(ctx);
	dispc->clk->enable(ctx);
	dispc_clk_update(dispc, panel->fps, SPRD_FORCE_FPS);

	dispc->core->init(ctx);
	dispc->core->ifconfig(ctx);

	dispc_encoder_init(dispc);

	/* for zebu/vdk, refresh immediately */
	dispc->core->run(ctx);

	ctx->is_inited = true;

	return ret;
}

static int32_t sprd_dispc_flip(struct sprd_dispc *dispc,
		struct sprd_restruct_config *config)
{
	int i;
	static bool logo2animation = true;
	struct dispc_context *ctx = &dispc->ctx;

	down(&ctx->refresh_lock);

	config_saved = config;
	if (!ctx->is_inited) {
		pr_err("dispc is not initialized!\n");
		up(&ctx->refresh_lock);
		return -1;
	}

	/*do iommu map when buffer is not reserved memory*/
	for (i = 0; i < config->number_hwlayer; i++)
		sprd_dispc_iommu_map(dispc, &config->hwlayers[i]);

	if (ctx->disable_flip) {
		pr_debug("dispc flip was disabled\n");
		up(&ctx->refresh_lock);
		return -1;
	}

	dispc->core->flip(ctx, config);

	if (unlikely(logo2animation)) {
		/*free logo buffer*/
		if (ctx->logo_vaddr) {
			pr_info("unmap logo buffer dma pages\n");
			dma_unmap_page(dispc->dev.parent, ctx->dma_handle,
				ctx->logo_size, DMA_TO_DEVICE);
			pr_info("free logo buffer!\n");
			free_pages((unsigned long)ctx->logo_vaddr,
						get_order(ctx->logo_size));
			ctx->logo_vaddr = NULL;
		}

		logo2animation = false;
	}

	/* update shadow registers and run if it is not in ESD Recovery */
	if (!ctx->is_stopped || (dispc->ctx.if_type == SPRD_DISPC_IF_EDPI))
		dispc->core->run(ctx);
	up(&ctx->refresh_lock);

	return 0;
}

static void sprd_dispc_enable_vsync(struct sprd_dispc *dispc)
{
	struct dispc_context *ctx = &dispc->ctx;

	down(&ctx->refresh_lock);

	if (!ctx->is_inited) {
		pr_err("dispc is not initialized!\n");
		up(&ctx->refresh_lock);
		return;
	}
	if (dispc->core->enable_vsync)
		dispc->core->enable_vsync(ctx);

	up(&ctx->refresh_lock);
}

static void sprd_dispc_disable_vsync(struct sprd_dispc *dispc)
{
	struct dispc_context *ctx = &dispc->ctx;

	down(&ctx->refresh_lock);

	if (!ctx->is_inited) {
		pr_err("dispc is not initialized!\n");
		up(&ctx->refresh_lock);
		return;
	}
	if (dispc->core->disable_vsync)
		dispc->core->disable_vsync(ctx);

	up(&ctx->refresh_lock);
}

static int sprd_dispc_modeset(struct sprd_dispc *dispc,
			struct drm_mode_modeinfo *mode)
{
	struct dispc_context *ctx = &dispc->ctx;
	int ret = 0;

	down(&ctx->refresh_lock);

	if (!ctx->is_inited) {
		pr_err("dispc is not initialized!\n");
		up(&ctx->refresh_lock);
		return -ENODEV;
	}
	if (dispc->core->modeset)
		ret = dispc->core->modeset(ctx, mode);

	up(&ctx->refresh_lock);
	return ret;
}

static int32_t sprd_dispc_suspend(struct sprd_dispc *dispc)
{
	struct dispc_context *ctx = &dispc->ctx;

	down(&ctx->refresh_lock);

	if (!ctx->is_inited) {
		pr_err("dispc is not initialized\n");
		up(&ctx->refresh_lock);
		return -1;
	}

	sprd_adf_fence_signal(dispc->adf);
	sprd_dispc_uninit(dispc);

	up(&ctx->refresh_lock);

	pr_info("dispc suspend OK\n");

	return 0;
}

static int32_t sprd_dispc_resume(struct sprd_dispc *dispc)
{
	struct dispc_context *ctx = &dispc->ctx;
	struct panel_info *panel = ctx->panel;

	down(&ctx->refresh_lock);

	if (ctx->is_inited) {
		pr_info("dispc has already initialized\n");
		up(&ctx->refresh_lock);
		return -1;
	}

	if (dispc->glb) {
		dispc->glb->power(ctx, true);
		dispc->glb->enable(ctx);
		dispc->glb->reset(ctx);
	}

	if (dispc->clk) {
		dispc->clk->init(ctx);
		dispc->clk->enable(ctx);
		dispc_clk_update(dispc, panel->fps, SPRD_FORCE_FPS);
	}

	if (dispc->core) {
		dispc->core->init(ctx);
		dispc->core->ifconfig(ctx);
	}

	dispc_encoder_init(dispc);

	ctx->is_inited = true;
	up(&ctx->refresh_lock);

	sprd_iommu_restore(&dispc->dev);

	pr_info("dispc resume OK\n");
	return 0;
}

int sprd_dispc_dynamic_clk(struct sprd_dispc *dispc, int type, u32 new_val)
{
	int ret = 0;
	struct dispc_context *ctx = &dispc->ctx;

	down(&ctx->refresh_lock);

	if (!ctx->is_inited) {
		pr_info("dispc is not initialized!\n");
		ret = -EINVAL;
		goto exit;
	}

	/* Now let's do update dpi clock */
	switch (type) {
	case SPRD_DYNAMIC_PCLK:
		if (ctx->if_type == SPRD_DISPC_IF_EDPI) {
			pr_err("current dispc interface isn't DPI\n");
			ret = -EINVAL;
			break;
		}
		dispc->core->stop(ctx);
		ret = dispc_clk_update(dispc, new_val,
						SPRD_DYNAMIC_PCLK);
		if (ret)
			pr_err("Failed to update pclk!\n");
		break;

	case SPRD_DYNAMIC_FPS:
		if (ctx->if_type == SPRD_DISPC_IF_EDPI) {
			/* sprd_panel_change_fps(dispc, new_val); */
			pr_err("CMD panel doesn't support this feature!\n");
			ret = -EINVAL;
			break;
		}
		dispc->core->stop(ctx);
		ret = dispc_clk_update(dispc, new_val,
						SPRD_DYNAMIC_FPS);
		if (ret)
			pr_err("Failed to update fps!\n");
		break;

	default:
		pr_err("invalid CMD type\n");
		ret = -EINVAL;
		break;
	}

	disp_notifier_call_chain(DISP_EVENT_CLOCK_CHANGE, NULL);
exit:
	dispc->core->run(ctx);
	up(&ctx->refresh_lock);

	return ret;

}

static void sprd_dispc_logo_proc(struct sprd_dispc *dispc)
{
	uint32_t logo_size;
	void *logo_src_v = NULL;
	void *logo_src_p = NULL;
	void *logo_dst_v = NULL;
	void *logo_dst_p = NULL;
	struct device *parent = NULL;
	struct dispc_context *ctx = &dispc->ctx;
	struct panel_info *panel = ctx->panel;
	struct sprd_restruct_config *config;
	struct page *page;

	if (!panel)
		return;

	if (logo_base_from_uboot == 0) {
		pr_err("logo_base from uboot is 0\n");
		return;
	}

	if (ctx->if_type == SPRD_DISPC_IF_EDPI) {
		pr_err("cmd panel doesn't need logo copy\n");
		return;
	}

	/* should be rgb565 */
	logo_size = panel->width * panel->height * 2;
#ifdef CONFIG_ZONE_DMA32
	logo_dst_v = (void *)__get_free_pages(GFP_DMA32 |
			GFP_ATOMIC | __GFP_ZERO, get_order(logo_size));
#else
	logo_dst_v = (void *)__get_free_pages(GFP_DMA |
			GFP_ATOMIC | __GFP_ZERO, get_order(logo_size));
#endif
	if (!logo_dst_v) {
		pr_err("Failed to allocate logo proc buffer\n");
		return;
	}

	pr_info("allocated %d bytes for logo buffer, address: %p\n",
						logo_size, logo_dst_v);
	logo_src_v = __va(logo_base_from_uboot);
	logo_src_p = (void *)logo_base_from_uboot;

	parent = dispc->dev.parent;
	page = virt_to_page(logo_dst_v);

#ifdef CONFIG_DISP_LITE_R2P0
	dma_set_mask(parent, DMA_BIT_MASK(64));
#endif
	ctx->dma_handle = dma_map_page(parent, page, 0, logo_size,
							DMA_TO_DEVICE);
	if (dma_mapping_error(parent, ctx->dma_handle)) {
		pr_err("dma map failed for address %p\n", logo_dst_v);
		free_pages((unsigned long)logo_dst_v, get_order(logo_size));
		return;
	}

	memcpy(logo_dst_v, logo_src_v, logo_size);
	dma_sync_single_for_device(parent, ctx->dma_handle,
				logo_size, DMA_TO_DEVICE);
	logo_dst_p = (void *)ctx->dma_handle;

	pr_debug("logo_src_p:0x%p, logo_src_v:0x%p\n", logo_src_p, logo_src_v);
	pr_debug("logo_dst_p:0x%p, logo_dst_v:0x%p\n", logo_dst_p, logo_dst_v);

	config = kzalloc(sizeof(struct sprd_restruct_config) +
			sizeof(struct sprd_adf_hwlayer), GFP_KERNEL);
	if (!config)
		return;

	config->number_hwlayer = 1;
	config->hwlayers[0].hwlayer_id = 0;
#ifdef CONFIG_DISP_LITE_R2P0
	config->hwlayers[0].iova_plane[0] = ctx->dma_handle - 0x80000000;
#else
	config->hwlayers[0].iova_plane[0] = ctx->dma_handle;
#endif
	config->hwlayers[0].n_planes = 1;
	config->hwlayers[0].alpha = 0xff;
	config->hwlayers[0].pitch[0] = panel->width * 2;
	config->hwlayers[0].dst_w = panel->width;
	config->hwlayers[0].dst_h = panel->height;
	config->hwlayers[0].format = DRM_FORMAT_RGB565;
	config->hwlayers[0].blending = HWC_BLENDING_NONE;
	config->hwlayers[0].compression = 0;

	dispc->core->flip(ctx, config);
	dispc->core->run(ctx);
	ctx->logo_size = logo_size;
	ctx->logo_vaddr = logo_dst_v;

	kfree(config);
}

static struct display_ctrl dispc_ctrl = {
	.name = "dispc",
	.flip = sprd_dispc_flip,
	.logo_proc = sprd_dispc_logo_proc,
	.enable_hw_vsync = sprd_dispc_enable_vsync,
	.disable_hw_vsync = sprd_dispc_disable_vsync,
	.modeset = sprd_dispc_modeset,
};

static int swdispc_context_init(struct sprd_dispc *dispc,
				struct device_node *np)
{

	uint32_t temp;
	struct dispc_context *ctx = &dispc->ctx;
	struct panel_info *panel = ctx->panel;

	if (dispc->clk && dispc->clk->parse_dt)
		dispc->clk->parse_dt(&dispc->ctx, np);
	if (dispc->glb && dispc->glb->parse_dt)
		dispc->glb->parse_dt(&dispc->ctx, np);

	if (!of_property_read_u32(np, "dev-id", &temp))
		ctx->id = temp;

	ctx->is_stopped = true;
	ctx->disable_flip = false;
	sema_init(&ctx->refresh_lock, 1);
	init_waitqueue_head(&ctx->wait_queue);

	if (!panel) {
		pr_err("error: panel is null, maybe Calibration Mode\n");
		return 0;
	}

	ctx->if_type = SPRD_DISPC_IF_EDPI;
	ctx->vsync_report_rate = panel->fps;
	ctx->vsync_ratio_to_panel =
		panel->fps / ctx->vsync_report_rate;

	return 0;
}

static int dispc_context_init(struct sprd_dispc *dispc,
				struct device_node *np)
{
	uint32_t temp;
	struct resource r;
	struct dispc_context *ctx = &dispc->ctx;
	struct panel_info *panel = ctx->panel;

	if (dispc->core && dispc->core->parse_dt)
		dispc->core->parse_dt(&dispc->ctx, np);
	if (dispc->clk && dispc->clk->parse_dt)
		dispc->clk->parse_dt(&dispc->ctx, np);
	if (dispc->glb && dispc->glb->parse_dt)
		dispc->glb->parse_dt(&dispc->ctx, np);

	if (!of_property_read_u32(np, "dev-id", &temp))
		ctx->id = temp;

	if (of_address_to_resource(np, 0, &r)) {
		pr_err("parse dt base address failed\n");
		return -ENODEV;
	}
	ctx->base = (unsigned long)ioremap_nocache(r.start,
					resource_size(&r));
	if (ctx->base == 0) {
		pr_err("ioremap base address failed\n");
		return -EFAULT;
	}

	ctx->is_stopped = true;
	ctx->disable_flip = false;
	sema_init(&ctx->refresh_lock, 1);
	init_waitqueue_head(&ctx->wait_queue);

	if (!panel) {
		pr_err("error: panel is null, maybe Calibration Mode\n");
		return 0;
	}

	switch (panel->type) {
	case SPRD_PANEL_TYPE_RGB:
	case SPRD_PANEL_TYPE_LVDS:
		ctx->if_type = SPRD_DISPC_IF_DPI;
		break;
	case SPRD_PANEL_TYPE_MIPI:
		if (panel->work_mode == SPRD_MIPI_MODE_VIDEO)
			ctx->if_type = SPRD_DISPC_IF_DPI;
		else
			ctx->if_type = SPRD_DISPC_IF_EDPI;
		break;
	case SPRD_PANEL_TYPE_MCU:
		ctx->if_type = SPRD_DISPC_IF_EDPI;
		break;
	default:
		ctx->if_type = SPRD_DISPC_IF_DPI;
		break;
	}

	ctx->vsync_report_rate = panel->fps;
	ctx->vsync_ratio_to_panel =
		panel->fps / ctx->vsync_report_rate;

	return 0;
}

static int dispc_irq_init(struct sprd_dispc *dispc)
{
	int ret;
	int irq_num;
	struct dispc_context *ctx = &dispc->ctx;

	irq_num = irq_of_parse_and_map(dispc->dev.of_node, 0);
	if (!irq_num) {
		pr_err("error: dispc parse irq num failed\n");
		return -EINVAL;
	}
	pr_info("dispc irq_num = %d\n", irq_num);
	ret = request_irq(irq_num, sprd_dispc_isr, 0, "DISPC", dispc);
	if (ret) {
		pr_err("error: dispc request irq failed\n");
		return -EINVAL;
	}

	ctx->irq_num = irq_num;
	ctx->dispc_isr = sprd_dispc_isr;

	return 0;
}

static int swdispc_irq_init(struct sprd_dispc *dispc)
{
	int ret;
	struct dispc_context *ctx = &dispc->ctx;
	struct panel_info *panel = ctx->panel;
	int irq_num;
	int te_gpio;

	te_gpio = panel->spi_te_gpio;
	gpio_request(te_gpio, "GPIO_TE");
	gpio_direction_input(te_gpio);
	irq_num = gpio_to_irq(te_gpio);
	ret = request_irq(irq_num, sprd_dispc_isr,
		IRQF_TRIGGER_FALLING,
		"SW-DISPC-TE", dispc);
	if (ret) {
		pr_err("sprd: swdispc failed to request irq!\n");
		return -EINVAL;
	}
	ctx->irq_num = irq_num;
	ctx->dispc_isr = sprd_dispc_isr;

	return 0;
}

int sprd_dispc_bgcolor(struct sprd_dispc *dispc, u32 color)
{
	struct dispc_context *ctx = &dispc->ctx;

	if (!dispc->core->bg_color)
		return -1;

	down(&ctx->refresh_lock);
	if (!ctx->is_inited) {
		pr_err("dispc is not initialized\n");
		up(&ctx->refresh_lock);
		return -1;
	}
	ctx->disable_flip = true;
	dispc->core->bg_color(ctx, color);
	dispc->core->run(ctx);
	up(&ctx->refresh_lock);

	return 0;
}

void sprd_dispc_disable_flip(struct sprd_dispc *dispc, bool disable)
{
	dispc->ctx.disable_flip = disable;
}

void sprd_dispc_disable_timeout(struct sprd_dispc *dispc, u32 timeout)
{
	dispc->ctx.disable_timeout = timeout / 17;
}

int sprd_dispc_refresh_restore(struct sprd_dispc *dispc)
{
	struct dispc_context *ctx = &dispc->ctx;

	ctx->disable_flip = false;
	down(&ctx->refresh_lock);

	if (!ctx->is_inited) {
		pr_err("dispc is not initialized!\n");
		up(&ctx->refresh_lock);
		return -1;
	}

	dispc->core->flip(ctx, config_saved);

	if (!ctx->is_stopped ||
			(dispc->ctx.if_type == SPRD_DISPC_IF_EDPI))
		dispc->core->run(ctx);
	up(&ctx->refresh_lock);

	return 0;
}

static int sprd_dispc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct sprd_dispc *dispc;
	const char *str;
	struct panel_info *panel;

	dispc = kzalloc(sizeof(struct sprd_dispc), GFP_KERNEL);
	if (!dispc)
		return -ENOMEM;

	dispc->ctrl = &dispc_ctrl;

	dispc->ctx.panel = platform_get_drvdata(pdev);
	if (dispc->ctx.panel == NULL) {
		pr_err("error: dispc->ctx.panel is null\n");
		goto err;
	}
	panel = dispc->ctx.panel;

	if (!of_property_read_string(np, "sprd,ip", &str)) {
		dispc->core = dispc_core_ops_attach(str);
		dispc->ctx.version = str;
	} else
		pr_err("error: \"sprd,ip\" was not found\n");

	if (!of_property_read_string(np, "sprd,soc", &str)) {
		dispc->clk = dispc_clk_ops_attach(str);
		dispc->glb = dispc_glb_ops_attach(str);
	} else {
		pr_err("error: \"sprd,soc\" was not found\n");
		dispc->clk = NULL;
		dispc->glb = NULL;
	}

	if (panel->type == SPRD_PANEL_TYPE_MIPI) {
		if (dispc_context_init(dispc, np))
			goto err;
		dispc_encoder_probe(dispc);
	} else if (panel->type == SPRD_PANEL_TYPE_SPI) {
		swdispc_context_init(dispc, np);
	}

	dispc_device_register(dispc, &pdev->dev);
	sprd_dispc_sysfs_init(&dispc->dev);
	#ifdef CONFIG_ZTE_LCD_COMMON_FUNCTION
	zte_lcd_common_func(dispc, panel);
	#endif
	dispc_notifier_register(dispc);
	platform_set_drvdata(pdev, dispc);

	if (panel->type == SPRD_PANEL_TYPE_SPI) {
		sprd_swdispc_init(dispc);
		swdispc_irq_init(dispc);
	} else {
		sprd_dispc_init(dispc);
		dispc_irq_init(dispc);
	}

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	pr_info("dispc driver probe success\n");

	return 0;

err:
	kfree(dispc);
	return -ENODEV;
}

static int dispc_runtime_resume(struct device *dev)
{
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct device *next = dev_get_next(dev);

	sprd_dispc_resume(dispc);
	pm_runtime_get_sync(next);

	return 0;
}

static int dispc_runtime_suspend(struct device *dev)
{
	struct sprd_dispc *dispc = dev_get_drvdata(dev);
	struct device *next = dev_get_next(&dispc->dev);

	pm_runtime_put_sync(next);
	sprd_dispc_suspend(dispc);

	return 0;
}

static const struct dev_pm_ops dispc_pm_ops = {
	.runtime_suspend = dispc_runtime_suspend,
	.runtime_resume = dispc_runtime_resume,
};

static const struct of_device_id dt_ids[] = {
	{ .compatible = "sprd,display-controller", },
	{}
};

static struct platform_driver sprd_dispc_driver = {
	.probe = sprd_dispc_probe,
	.driver = {
		.name = "sprd-dispc",
		.of_match_table = of_match_ptr(dt_ids),
		.pm = &dispc_pm_ops,
	},
};

module_platform_driver(sprd_dispc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leon.he@spreadtrum.com");
MODULE_AUTHOR("infi.chen@spreadtrum.com");
MODULE_DESCRIPTION("SPRD Display Controller Platform Driver");
