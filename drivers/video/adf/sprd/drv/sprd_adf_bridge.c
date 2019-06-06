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

#include <linux/of_platform.h>
#include "sprd_adf_bridge.h"
#include "sprd_adf.h"

LIST_HEAD(display_config_head);

/**
 * sprd_adf_get_config_entry - get drv's callback entry
 *
 * @pdev: platform device pointer of adf device
 *
 * sprd_adf_get_config_entry according to the dispc of node,
 * get the dispc hw config
 */
struct sprd_display_config_entry *sprd_adf_get_config_entry(
			struct platform_device *pdev)
{
	const char *str;
	struct device_node *np;
	struct sprd_display_config_entry *config = NULL;

	np = of_parse_phandle(pdev->dev.of_node, "sprd,dispc", 0);
	pdev = of_find_device_by_node(np);
	if (!pdev) {
		pr_warn("error: get dispc platform device failed\n");
		return NULL;
	}

	if (!of_property_read_string(pdev->dev.of_node, "sprd,ip", &str))
		config = disp_ops_attach(str, &display_config_head);

	if (!config)
		pr_warn("get display config entry failed\n");

	return config;
}

static struct sprd_adf_device_ops device_ops = {
	.restruct_post_config = sprd_adf_restruct_post_config,
	.free_restruct_config = sprd_adf_free_restruct_config,
	.flip = sprd_adf_device_flip,
	.wait_flip_done = sprd_adf_wait_flip_done,
};

static struct sprd_adf_interface_ops dsi0_ops = {
	.init = sprd_adf_interface_init,
	.uninit = sprd_adf_interface_uninit,
	.dpms_on = sprd_adf_interface_dpms_on,
	.dpms_off = sprd_adf_interface_dpms_off,
	.enable_vsync_irq = sprd_adf_enable_vsync_irq,
	.disable_vsync_irq = sprd_adf_disable_vsync_irq,
	.get_screen_size = sprd_adf_get_screen_size,
	.get_modes = sprd_adf_interface_get_modes,
	.set_mode = sprd_adf_interface_set_mode,
	.get_mode = sprd_adf_interface_get_mode,
};

/**
 * sprd_adf_get_device_ops - get sprd device callback function
 *
 * @index: device index
 *
 * sprd_adf_get_device_ops() get the target device's callback
 * functions
 */
struct sprd_adf_device_ops *sprd_adf_get_device_ops(size_t index)
{
	pr_debug("entern\n");
	if (index != 0) {
		pr_warn("index is illegal\n");
		return NULL;
	}

	return &device_ops;
}

/**
 * sprd_adf_get_device_private_data - get device some private data
 *
 * @pdev: parent device
 * @index: device index
 *
 * sprd_adf_get_device_private_data() get the target device's private
 * data.
 */
void *sprd_adf_get_device_private_data(struct platform_device *pdev,
			size_t index)
{
	pr_debug("entern\n");

	return NULL;
}

/**
 * sprd_adf_destroy_device_private_data - free some resource
 *
 * @data: private data ptr
 *
 * sprd_adf_destroy_device_private_data() free private data.
 */
void sprd_adf_destroy_device_private_data(void *data)
{
	pr_debug("entern\n");
	kfree(data);
}

/**
 * sprd_adf_get_interface_ops - get sprd interface callback function
 *
 * @index: interface index
 *
 * sprd_adf_get_interface_ops() get the target interface's callback
 * functions
 */
struct sprd_adf_interface_ops *sprd_adf_get_interface_ops(size_t index)
{
	pr_debug("entern\n");
	return &dsi0_ops;
}

/**
 * sprd_adf_get_interface_private_data - get interface some private data
 *
 * @pdev: parent device
 * @index: device index
 *
 * sprd_adf_get_interface_private_data() get the target interface's private
 * data.
 */
void *sprd_adf_get_interface_private_data(struct platform_device *pdev,
			size_t index)
{
	int display_mode;
	struct pipe *pipe;
	int rc;

	pr_debug("entern\n");
	if (!pdev) {
		pr_warn("index is illegal\n");
		return NULL;
	}

	rc = of_property_read_u32(pdev->dev.of_node,
				  "sprd,display-mode", &display_mode);
	if (rc) {
		pr_warn("read display mode fail\n");
		pr_warn("use single dispc default\n");
		display_mode = SPRD_SINGLE_DISPC;
	}
	pipe = kzalloc(sizeof(struct pipe), GFP_KERNEL);
	if (!pipe)
		return NULL;

	pipe_config(pipe, display_mode, index, pdev);

	return (void *)pipe;
}

/**
 * sprd_adf_destroy_interface_private_data - free some resource
 *
 * @data: private data ptr
 *
 * sprd_adf_destroy_interface_private_data() free private data.
 */
void sprd_adf_destroy_interface_private_data(void *data)
{
	pr_debug("entern\n");
	kfree(data);
}
