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

#include "sprd_adf_bridge.h"

static struct sprd_display_attachment sharkl2_allowed_attachments[] = {
	[0] = {
		.interface_id = SPRD_INTERFACE_DSI0,
		.overlayengine_id = SPRD_OVERLAYENGINE_DISPC0,
	},
};

static struct sprd_display_config sharkl2_display_config = {
	.id = 0x1,
	.n_devices = 1,
	.n_interfaces = 1,
	.n_overlayengines = 1,
	.n_allowed_attachments = ARRAY_SIZE(sharkl2_allowed_attachments),
	.allowed_attachments = sharkl2_allowed_attachments,
};

static struct sprd_adf_device_capability sharkl2_device_capability = {
	.device_id = 0x1,

};

static struct sprd_adf_interface_capability sharkl2_interface_capability = {
	.interface_id = 0x11,
	.fb_count = 3,
	.fb_format = ADF_PIXEL_FORMAT_RGBA_8888,
};

static struct sprd_adf_hwlayer_capability sharkl2_hwlayer_capability[] = {
	/* Image layer*/
	[0] = {
	       .hwlayer_id = 0,
	       .format = ADF_PIXEL_FORMAT_RGBA_8888 |
	       ADF_PIXEL_FORMAT_RGBX_8888 |
	       ADF_PIXEL_FORMAT_RGB_888 |
	       ADF_PIXEL_FORMAT_RGB_565 |
	       ADF_PIXEL_FORMAT_BGRA_8888 |
	       ADF_PIXEL_FORMAT_YV12 |
	       ADF_PIXEL_FORMAT_YCrCb_422_SP |
	       ADF_PIXEL_FORMAT_YCbCr_422_SP |
	       ADF_PIXEL_FORMAT_YCrCb_420_SP |
	       ADF_PIXEL_FORMAT_YCbCr_420_SP | ADF_PIXEL_FORMAT_YCbCr_422_I,
	       .rotation = ADF_TRANSFORM_NONE,
	       .scale = ADF_SCALE_NONE,
	       .blending = ADF_BLENDING_PREMULT,
	       },
	/* OSD layer */
	[1] = {
	       .hwlayer_id = 1,
	       .format = ADF_PIXEL_FORMAT_RGBA_8888 |
	       ADF_PIXEL_FORMAT_RGBX_8888 |
	       ADF_PIXEL_FORMAT_RGB_888 |
	       ADF_PIXEL_FORMAT_RGB_565 | ADF_PIXEL_FORMAT_BGRA_8888,

	       .rotation = ADF_TRANSFORM_NONE,
	       .scale = ADF_SCALE_NONE,
	       .blending = ADF_BLENDING_PREMULT,
	       },
};

static struct sprd_adf_overlayengine_capability
sharkl2_overlayengine_capability = {
	.number_hwlayer = 4,

	.format = ADF_PIXEL_FORMAT_RGBA_8888 |
	    ADF_PIXEL_FORMAT_RGBX_8888 |
	    ADF_PIXEL_FORMAT_RGB_888 |
	    ADF_PIXEL_FORMAT_RGB_565 |
	    ADF_PIXEL_FORMAT_BGRA_8888 |
	    ADF_PIXEL_FORMAT_YV12 |
	    ADF_PIXEL_FORMAT_YCrCb_422_SP |
	    ADF_PIXEL_FORMAT_YCbCr_422_SP |
	    ADF_PIXEL_FORMAT_YCrCb_420_SP |
	    ADF_PIXEL_FORMAT_YCbCr_420_SP | ADF_PIXEL_FORMAT_YCbCr_422_I,
	.rotation = ADF_TRANSFORM_NONE,
	.scale = ADF_SCALE_NONE,
	.blending = ADF_BLENDING_PREMULT,

	.hwlayer_ptr[0] = &sharkl2_hwlayer_capability[0],
	.hwlayer_ptr[1] = &sharkl2_hwlayer_capability[1],
	.hwlayer_ptr[2] = &sharkl2_hwlayer_capability[1],
	.hwlayer_ptr[3] = &sharkl2_hwlayer_capability[1],
	.hwlayer_ptr[4] = &sharkl2_hwlayer_capability[1],
	.hwlayer_ptr[5] = &sharkl2_hwlayer_capability[1],

};

static struct sprd_adf_device_capability *sharkl2_get_device_capability(int
								      dev_id)
{
	struct sprd_adf_device_capability *capability;

	ADF_DEBUG_GENERAL("entern\n");
	if (dev_id < 0) {
		ADF_DEBUG_WARN("parameter dev_id is illegal\n");
		return NULL;
	}

	capability = &sharkl2_device_capability;

	return capability;
}

static struct sprd_adf_interface_capability
*sharkl2_get_interface_capability(int dev_id, int intf_id)
{
	struct sprd_adf_interface_capability *capability;

	ADF_DEBUG_GENERAL("entern\n");
	if (dev_id < 0 || intf_id < 0) {
		ADF_DEBUG_WARN("parameter dev_id or intf_id is illegal\n");
		return NULL;
	}

	capability = &sharkl2_interface_capability;

	return capability;
}

static struct sprd_adf_overlayengine_capability
*sharkl2_get_overlayengine_capability(int dev_id, int eng_id)
{
	struct sprd_adf_overlayengine_capability *capability;

	ADF_DEBUG_GENERAL("entern\n");
	if (dev_id < 0 || eng_id < 0) {
		ADF_DEBUG_WARN("parameter dev_id or eng_id is illegal\n");
		return NULL;
	}

	capability = &sharkl2_overlayengine_capability;
	return capability;
}

static struct sprd_display_config *sharkl2_get_display_config(void)
{
	struct sprd_display_config *display_config = NULL;

	ADF_DEBUG_GENERAL("entern\n");

	display_config = &sharkl2_display_config;

	return display_config;
}

static struct sprd_display_config_entry sharkl2_display_config_entry = {
	.get_device_capability = sharkl2_get_device_capability,
	.get_interface_capability = sharkl2_get_interface_capability,
	.get_overlayengine_capability = sharkl2_get_overlayengine_capability,
	.get_display_config = sharkl2_get_display_config,
};

static struct ops_entry entry = {
	.ver = "dispc-lite-r1p0",
	.ops = &sharkl2_display_config_entry,
};

static int __init sprd_display_config_register(void)
{
	return disp_ops_register(&entry, &display_config_head);
}

subsys_initcall(sprd_display_config_register);
