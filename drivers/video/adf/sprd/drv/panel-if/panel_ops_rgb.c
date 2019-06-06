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

#include "sprd_panel.h"

static int32_t rgb_panel_init(struct panel_device *pd)
{
	return 0;
}

static int32_t rgb_panel_esd_check(struct panel_device *pd)
{
	return 0;
}

static uint32_t rgb_panel_read_id(struct panel_device *pd)
{
	return 0;
}

static int32_t rgb_panel_sleep_in(struct panel_device *pd)
{
	return 0;
}

static int32_t rgb_panel_send_cmd(struct panel_device *pd, int id)
{
	return 0;
}

static int32_t rgb_panel_set_brightness(struct panel_device *pd, int level)
{
	return 0;
}

struct panel_ops panel_ops_rgb = {
	.init = rgb_panel_init,
	.sleep_in = rgb_panel_sleep_in,
	.read_id = rgb_panel_read_id,
	.esd_check = rgb_panel_esd_check,
	.send_cmd = rgb_panel_send_cmd,
	.set_brightness = rgb_panel_set_brightness,
};
