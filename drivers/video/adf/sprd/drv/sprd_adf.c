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

#include <linux/compat.h>
#include <linux/export.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sprd_iommu.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <video/adf_notifier.h>

#include "sprd_adf_adapter.h"
#include "sprd_adf.h"

#ifdef CONFIG_ZTE_LCD_COMMON_FUNCTION
#include "zte_lcd_common.h"
extern struct sprd_dispc *g_zte_ctrl_pdata;
#endif
#define DISPLAY_CFG_PATH "/mnt/vendor/enhance/display.conf"


static struct adf_notifier_event evdata;
static bool dpms_state = true;

static BLOCKING_NOTIFIER_HEAD(adf_notifier_list);

/**
 *	adf_register_client - register a client notifier
 *	@nb: notifier block to callback on events
 */
int adf_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&adf_notifier_list, nb);
}
EXPORT_SYMBOL(adf_register_client);

/**
 *	adf_unregister_client - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
int adf_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&adf_notifier_list, nb);
}
EXPORT_SYMBOL(adf_unregister_client);

/**
 * adf_notifier_call_chain - notify clients of adf_events
 *
 */
static int adf_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&adf_notifier_list, val, v);
}

struct sprd_restruct_config *sprd_adf_restruct_post_config(struct adf_post
							   *post)
{
	struct sprd_restruct_config *config;
	struct sprd_adf_post_custom_data *data;
	struct adf_buffer *bufs;
	struct sprd_adf_hwlayer *dst;
	struct sprd_adf_hwlayer_custom_data *cust_src;
	size_t n_bufs;
	size_t i, j;
	size_t kzalloc_size;
	size_t custom_data_size;
	size_t display_mode;

	if (!post || !post->custom_data) {
		pr_warn("parameter is ilegal\n");
		goto error_out;
	}

	custom_data_size = (size_t) post->custom_data_size;
	if (custom_data_size < FLIP_CUSTOM_DATA_MIN_SIZE) {
		pr_warn("data size is ilegal data_size = %zd;\n",
			       custom_data_size);
		goto error_out;
	}

	display_mode = SPRD_SINGLE_DISPC;
	data = (struct sprd_adf_post_custom_data *)post->custom_data;
	bufs = post->bufs;
	n_bufs = post->n_bufs;

	kzalloc_size =
	    sizeof(struct sprd_restruct_config) +
	    n_bufs * sizeof(struct sprd_adf_hwlayer);
	if (display_mode == SPRD_DOUBLE_DISPC_MASTER_AND_SLAVE
	    || display_mode == SPRD_DOUBLE_DISPC_SAME_CONTENT) {
		kzalloc_size = kzalloc_size << 1;
		n_bufs = n_bufs << 1;
	}

	config = kzalloc(kzalloc_size, GFP_KERNEL);
	if (!config)
		goto error_out;

	config->number_hwlayer = n_bufs;

	for (i = 0; i < post->n_bufs; i++) {
		dst = &config->hwlayers[i];
		cust_src = &data->hwlayers[i];
		dst->mapping = post->mappings[i];
		dst->base = bufs[i];
		dst->hwlayer_id = i;
		dst->interface_id = cust_src->interface_id;
		dst->n_planes = bufs[i].n_planes;
		dst->alpha = cust_src->alpha;
		dst->start_x = cust_src->start_x;
		dst->start_y = cust_src->start_y;
		dst->start_w = cust_src->start_w;
		dst->start_h = cust_src->start_h;

		dst->dst_x = cust_src->dst_x;
		dst->dst_y = cust_src->dst_y;
		dst->dst_w = cust_src->dst_w;
		dst->dst_h = cust_src->dst_h;
		dst->format = bufs[i].format;
		dst->blending = cust_src->blending;
		dst->rotation = cust_src->rotation;
		dst->scale = cust_src->scale;
		dst->compression = cust_src->compression;
		dst->height = cust_src->height;
		dst->header_size_r = cust_src->header_size_r;
		dst->header_size_y = cust_src->header_size_y;
		dst->header_size_uv = cust_src->header_size_uv;

		if (display_mode == SPRD_DOUBLE_DISPC_MASTER_AND_SLAVE ||
		    display_mode == SPRD_DOUBLE_DISPC_SAME_CONTENT) {
			memcpy((dst + post->n_bufs), dst,
			       sizeof(struct sprd_adf_hwlayer));
			(dst + post->n_bufs)->hwlayer_id =
			    cust_src->hwlayer_id + (SPRD_N_LAYER >> 1);
		}

		if (display_mode == SPRD_DOUBLE_DISPC_MASTER_AND_SLAVE) {
			dst->dst_h = (data->hwlayers[i].dst_h >> 1);
			(dst + post->n_bufs)->dst_y =
			    cust_src->dst_y + (cust_src->dst_h >> 1);
			(dst + post->n_bufs)->dst_h = (cust_src->dst_h >> 1);
		}

		for (j = 0; j < bufs[i].n_planes; j++) {
			dst->pitch[j] = bufs[i].pitch[j];
			dst->iova_plane[j] = bufs[i].offset[j];
			if (display_mode
				== SPRD_DOUBLE_DISPC_MASTER_AND_SLAVE) {
				(dst + post->n_bufs)->pitch[j] =
				    bufs[i].pitch[j];
				(dst + post->n_bufs)->iova_plane[j] =
				    dst->iova_plane[j] +
				    bufs[i].pitch[j] * (cust_src->dst_h >> 1);
			} else if (display_mode ==
				   SPRD_DOUBLE_DISPC_SAME_CONTENT) {
				(dst + post->n_bufs)->pitch[j] =
				    bufs[i].pitch[j];
				(dst + post->n_bufs)->iova_plane[j] =
				    dst->iova_plane[j];
			}
		}
	}

	return config;

error_out:
	return NULL;
}

void sprd_adf_free_restruct_config(struct sprd_adf_interface *adf_intf,
				   struct sprd_restruct_config *config)
{
	struct sprd_dispc *dispc;
	struct pipe *pipe = (struct pipe *)(adf_intf->data);
	int i, j;

	for (i = 0; i < pipe->n_sdev; i++) {
		dispc = pipe->sdev[i];
		for (j = 0; j < config->number_hwlayer; j++)
			sprd_dispc_iommu_unmap(dispc, &config->hwlayers[j]);
	}

	kfree(config);
}

void sprd_adf_wait_flip_done(struct sprd_adf_interface *adf_intf)
{
}

static void dump_config_info(struct sprd_restruct_config *config)
{
	size_t i, j;

	pr_debug("flip number_hwlayer = 0x%x\n",
			  config->number_hwlayer);
	for (i = 0; i < config->number_hwlayer; i++) {
		pr_debug("flip hwlayer_id = 0x%x;index = %zx;\n",
				  config->hwlayers[i].hwlayer_id, i);
		pr_debug("flip n_planes = 0x%x\n",
				  config->hwlayers[i].n_planes);
		pr_debug("flip alpha = 0x%x\n",
				  config->hwlayers[i].alpha);
		pr_debug("flip dst_x = 0x%x\n",
				  config->hwlayers[i].dst_x);
		pr_debug("flip dst_y = 0x%x\n",
				  config->hwlayers[i].dst_y);
		pr_debug("flip dst_w = 0x%x\n",
				  config->hwlayers[i].dst_w);
		pr_debug("flip dst_h = 0x%x\n",
				  config->hwlayers[i].dst_h);
		pr_debug("flip format = 0x%x\n",
				  config->hwlayers[i].format);
		pr_debug("flip blending = 0x%x\n",
				  config->hwlayers[i].blending);
		pr_debug("flip rotation = 0x%x\n",
				  config->hwlayers[i].rotation);
		pr_debug("flip scale = 0x%x\n",
				  config->hwlayers[i].scale);
		pr_debug("flip compression = 0x%x\n",
				  config->hwlayers[i].compression);

		for (j = 0; j < config->hwlayers[i].n_planes; j++) {
			pr_debug("flip pitch = 0x%x;index = %zx;\n",
					  config->hwlayers[i].pitch[j], j);
			pr_debug("flip iova_plane = 0x%x\n",
					  config->hwlayers[i].iova_plane[j]);
		}
	}
}

void sprd_adf_device_flip(struct sprd_adf_interface *adf_intf,
			  struct sprd_restruct_config *config)
{
	struct sprd_dispc *dispc;
	struct pipe *pipe = (struct pipe *)(adf_intf->data);
	int index;
	int num_dev = pipe->n_sdev;

	dump_config_info(config);
	for (index = 0; index < num_dev; index++) {
		dispc = pipe->sdev[index];
		dispc->ctrl->flip(dispc, config);
	}
}

int sprd_adf_get_screen_size(struct sprd_adf_interface *intf,
			     u16 *width_mm, u16 *height_mm)
{
	struct pipe *pipe;
	struct panel_info *panel;
	struct sprd_dispc *dispc;

	pipe = intf->data;
	dispc = pipe->sdev[0];
	panel = dispc->ctx.panel;

	if (!panel) {
		pr_err("panel is null, maybe Calibration Mode\n");
		return -EINVAL;
	}

	if (!panel->width_mm || !panel->height_mm) {
		*width_mm = 68;
		*height_mm = 121;
	} else {
		*width_mm = panel->width_mm;
		*height_mm = panel->height_mm;
	}
	pr_debug("width_mm = %d, height_mm = %d\n",
			*width_mm, *height_mm);
	return 0;
}

int sprd_adf_interface_get_modes(struct sprd_adf_interface *intf,
			struct drm_mode_modeinfo *mode, size_t *n_modes)
{
	struct pipe *pipe;
	struct panel_info *panel;
	struct sprd_dispc *dispc;
	uint16_t *res;
	int i, j, ret, count;

	pipe = intf->data;
	dispc = pipe->sdev[0];
	panel = dispc->ctx.panel;

	if (!panel) {
		pr_err("panel is null, maybe Calibration Mode\n");
		return -EINVAL;
	}

	if (!mode) {
		count = of_property_count_u16_elems(dispc->adf->of_node,
					"sprd,low-res-support");
		if (count < 0)
			count = 0;
		else
			count = count / 2;
		*n_modes = count + 1;
		return 0;
	}

	if (panel->low_res_simu && panel->simu_width && panel->simu_height) {
		mode[0].hdisplay = panel->simu_width;
		mode[0].vdisplay = panel->simu_height;
	} else {
		mode[0].hdisplay = panel->width;
		mode[0].vdisplay = panel->height;
	}
	mode[0].vrefresh = panel->fps * 1000;

	if (*n_modes > 1) {
		count = (*n_modes - 1) * 2;
		res = kcalloc(count, sizeof(*res), GFP_KERNEL);
		if (!res)
			return -ENOMEM;

		ret = of_property_read_u16_array(dispc->adf->of_node,
					"sprd,low-res-support", res, count);
		if (ret) {
			pr_err("get sprd,low-res-support failed\n");
			kfree(res);
			*n_modes = 1;
			return ret;
		}

		for (i = 1, j = 0; j < count; j++) {
			mode[i].hdisplay = res[j];
			j++;
			mode[i].vdisplay = res[j];

			if (panel->width < mode[i].hdisplay
				|| panel->height < mode[i].vdisplay
				|| (panel->width == mode[i].hdisplay
					&& panel->height == mode[i].vdisplay))
				continue;

			mode[i++].vrefresh = panel->fps * 1000;
		}
		kfree(res);
		*n_modes = i;
	}

	return 0;
}

int sprd_adf_interface_set_mode(struct sprd_adf_interface *intf,
				struct drm_mode_modeinfo *mode)
{
	struct sprd_dispc *dispc;
	struct pipe *pipe = (struct pipe *)(intf->data);
	int index, ret;
	int num_dev = pipe->n_sdev;
	struct panel_info *panel;
	uint32_t size, cfgindex;

	for (cfgindex = 0; cfgindex < intf->base.n_modes; cfgindex++) {
		if (intf->base.modelist[cfgindex].hdisplay == mode->hdisplay
		&& intf->base.modelist[cfgindex].vdisplay == mode->vdisplay) {
			size = (mode->vdisplay << 16) | mode->hdisplay;
			break;
		}
	}

	pr_info("cfgindex:%d\n", cfgindex);
	for (index = 0; index < num_dev; index++) {
		dispc = pipe->sdev[index];
		panel = dispc->ctx.panel;

		ret = file_write(DISPLAY_CFG_PATH, O_WRONLY | O_CREAT, 0644,
				(char *)&cfgindex, sizeof(cfgindex));
		if (ret < 0) {
			pr_err("save index fail\n");
			return ret;
		}

		if (panel && panel->low_res_simu) {
			adf_notifier_call_chain(ADF_EVENT_MODE_SET, &size);
			continue;
		}

		dispc->ctrl->modeset(dispc, mode);
	}

	return 0;
}

int sprd_adf_interface_get_mode(struct sprd_adf_interface *intf, u32 *index)
{
	struct sprd_dispc *dispc;
	struct pipe *pipe = (struct pipe *)(intf->data);
	int num_dev = pipe->n_sdev;
	struct panel_info *panel;
	int ret, i;
	bool enable = 0;

	ret = file_read(DISPLAY_CFG_PATH, O_RDONLY, 0,
			(char *)index, sizeof(*index));
	if (ret < 0) {
		if (-ENOENT == ret)
			*index = 0;
		else {
			pr_err("read index fail\n");
			return ret;
		}
	}

	ret = file_read(LOW_RES_SIMU_EN_PATH, O_RDONLY | O_CREAT, 0644,
			(char *)&enable, sizeof(enable));
	if (ret < 0) {
		pr_err("read fail\n");
		return 0;
	}

	for (i = 0; i < num_dev; i++) {
		dispc = pipe->sdev[i];
		panel = dispc->ctx.panel;
		panel->low_res_simu = enable;
	}

	pr_info("index:%d enable:%d\n", *index, enable);
	return 0;
}


int sprd_adf_interface_dpms_on(struct sprd_adf_interface *intf)
{
	static int adf_blank = DRM_MODE_DPMS_ON;
	struct pipe *pipe = intf->data;
	struct sprd_dispc *dispc;

	if (dpms_state) {
		pr_info("dpms enable\n");
		return 0;
	}

	dispc = pipe->sdev[0];
	pm_runtime_get_sync(dispc->adf);

	evdata.info = intf;
	evdata.data = &adf_blank;
	adf_notifier_call_chain(ADF_EVENT_BLANK, &evdata);

	dpms_state = true;

	return 0;
}

int sprd_adf_interface_dpms_off(struct sprd_adf_interface *intf)
{
	static int adf_blank = DRM_MODE_DPMS_OFF;
	struct pipe *pipe = intf->data;
	struct sprd_dispc *dispc;

	if (!dpms_state) {
		pr_info("dpms off\n");
		return 0;
	}

	dispc = pipe->sdev[0];
	pm_runtime_put_sync(dispc->adf);

	evdata.info = intf;
	evdata.data = &adf_blank;
	adf_notifier_call_chain(ADF_EVENT_BLANK, &evdata);

	dpms_state = false;

	return 0;
}

void sprd_adf_enable_vsync_irq(struct sprd_adf_interface *intf)
{
	struct sprd_dispc *dispc;
	struct pipe *pipe = (struct pipe *)(intf->data);
	int index;
	int num_dev = pipe->n_sdev;

	for (index = 0; index < num_dev; index++) {
		dispc = pipe->sdev[index];
		dispc->ctrl->enable_hw_vsync(dispc);
	}
}

void sprd_adf_disable_vsync_irq(struct sprd_adf_interface *intf)
{
	struct sprd_dispc *dispc;
	struct pipe *pipe = (struct pipe *)(intf->data);
	int index;
	int num_dev = pipe->n_sdev;

	for (index = 0; index < num_dev; index++) {
		dispc = pipe->sdev[index];
		dispc->ctrl->disable_hw_vsync(dispc);
	}
}

int32_t sprd_adf_interface_init(struct sprd_adf_interface *adf_intf)
{
	pr_debug("entern\n");
	return 0;
}

int32_t sprd_adf_interface_uninit(struct sprd_adf_interface *adf_intf)
{
	pr_debug("entern\n");
	return 0;
}

static struct sprd_dispc *sprd_device_init(struct platform_device *pdev,
					    int index)
{
	struct platform_device *dispc_pdev;
	struct device_node *np;
	struct sprd_dispc *dispc;

	np = of_parse_phandle(pdev->dev.of_node, "sprd,dispc", 0);
	dispc_pdev = of_find_device_by_node(np);
	if (!dispc_pdev) {
		pr_err("error: get dispc platform device failed\n");
		return NULL;
	}
	dispc = platform_get_drvdata(dispc_pdev);
	dispc->adf = &pdev->dev;

	return dispc;
}

int pipe_config(struct pipe *pipe, int display_mode, int interface_id,
		struct platform_device *pdev)
{
	switch (display_mode) {
	case SPRD_SINGLE_DISPC:
		pr_info("interface_id = %d\n", interface_id);
		pipe->n_sdev = 1;
		pipe->sdev[0] = sprd_device_init(pdev, 0);
		break;
	case SPRD_DOUBLE_DISPC_MASTER_AND_SLAVE:
		pr_info("interface_id = %d\n", interface_id);
		pipe->n_sdev = 2;
		pipe->sdev[0] = sprd_device_init(pdev, 0);
		pipe->sdev[1] = sprd_device_init(pdev, 1);
		break;
	case SPRD_DOUBLE_DISPC_SAME_CONTENT:
		pr_info("interface_id = %d\n", interface_id);
		pipe->n_sdev = 2;
		pipe->sdev[0] = sprd_device_init(pdev, 0);
		pipe->sdev[1] = sprd_device_init(pdev, 1);
		break;
	case SPRD_DOUBLE_DISPC_INDEPENDENT_CONTENT:
		pr_info("interface_id = %d\n", interface_id);
		pipe->n_sdev = 1;
		pipe->sdev[interface_id] = sprd_device_init(pdev, interface_id);
		break;
	default:
		return -1;
	}
	return 0;
}

static const struct of_device_id sprd_adf_dt_ids[] = {
	{ .compatible = "sprd-adf", },
	{}
};

static int adf_get_dispc(struct platform_device *pdev)
{
	struct platform_device *dispc_pdev;
	struct device_node *np;
	struct sprd_dispc *dispc;

	np = of_parse_phandle(pdev->dev.of_node, "sprd,dispc", 0);
	dispc_pdev = of_find_device_by_node(np);
	if (!dispc_pdev) {
		pr_err("error: get dispc platform device failed\n");
		return -ENODEV;
	}
	dispc = platform_get_drvdata(dispc_pdev);
	if (dispc == NULL)
		return -ENODEV;

	return 0;
}

static int sprd_adf_probe(struct platform_device *pdev)
{
	struct sprd_adf_context *adf_context;
	struct sprd_adf_interface *interfaces;
	struct sprd_dispc *dispc;
	struct pipe *pipe;

	if (adf_get_dispc(pdev))
		return -ENODEV;

	adf_context = sprd_adf_context_create(pdev);
	if (!adf_context) {
		pr_warn("sprd_adf_context_create faile\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, adf_context);

	interfaces = adf_context->interfaces;
	pipe = (struct pipe *)interfaces->data;
	dispc = pipe->sdev[0];

	dispc->ctrl->logo_proc(dispc);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return 0;
}

static void sprd_adf_shutdown(struct platform_device *pdev)
{
	pr_info("adf shutdown\n");

	#ifdef CONFIG_ZTE_LCD_COMMON_FUNCTION
	g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_powerdown_for_shutdown = true;
	#endif
	/*
	 * The force suspend will disable pm runtime, then
	 * the pm resume can not be active after shutdown.
	 */
	pm_runtime_force_suspend(&pdev->dev);
	#ifdef CONFIG_ZTE_LCD_COMMON_FUNCTION
	g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_powerdown_for_shutdown = false;
	#endif

}

static int adf_runtime_suspend(struct device *dev)
{
	struct sprd_adf_context *adf_context = dev_get_drvdata(dev);
	struct sprd_adf_interface *interfaces = adf_context->interfaces;
	struct pipe *pipe = interfaces->data;
	int num_dev = pipe->n_sdev;
	struct sprd_dispc *dispc;
	int index;

	pr_info("adf suspend start\n");
	for (index = 0; index < num_dev; index++) {
		dispc = pipe->sdev[index];
		pm_runtime_put_sync(dispc->dev.parent);
	}
	pr_info("adf suspend end\n");

	return 0;
};

static int adf_runtime_resume(struct device *dev)
{
	struct sprd_adf_context *adf_context = dev_get_drvdata(dev);
	struct sprd_adf_interface *interfaces = adf_context->interfaces;
	struct pipe *pipe = interfaces->data;
	int num_dev = pipe->n_sdev;
	struct sprd_dispc *dispc;
	int index;

	pr_info("adf resume start\n");
	for (index = 0; index < num_dev; index++) {
		dispc = pipe->sdev[index];
		pm_runtime_get_sync(dispc->dev.parent);
	}
	pr_info("adf resume end\n");

	return 0;
};

static const struct dev_pm_ops sprd_adf_pm_ops = {
	.runtime_suspend = adf_runtime_suspend,
	.runtime_resume = adf_runtime_resume,
};

static struct platform_driver sprd_adf_driver = {
	.probe = sprd_adf_probe,
	.shutdown = sprd_adf_shutdown,
	.driver = {
		.name = "sprd-adf",
		.of_match_table = of_match_ptr(sprd_adf_dt_ids),
		.pm = &sprd_adf_pm_ops,
	},
};

module_platform_driver(sprd_adf_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xinglong.zhu@spreadtrum.com");
MODULE_AUTHOR("infi.chen@spreadtrum.com");
MODULE_DESCRIPTION("SPRD ADF Platform Driver");
