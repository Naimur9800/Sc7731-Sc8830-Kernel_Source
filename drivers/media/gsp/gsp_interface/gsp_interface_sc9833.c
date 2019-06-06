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


#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd/sc9833/sc9833_glb.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include "gsp_interface_sc9833.h"
#include "../gsp_debug.h"
#include "../gsp_interface.h"


int gsp_interface_sc9833_parse_dt(struct gsp_interface *intf,
				  struct device_node *node)
{
	struct regmap *gpr = NULL;
	struct gsp_interface_sc9833 *interface = NULL;

	interface = (struct gsp_interface_sc9833 *)intf;

	/*get global enable register*/
	gpr = syscon_regmap_lookup_by_phandle(node, "sprd,sys-ap-ahb");
	if (IS_ERR(gpr)) {
		GSP_ERR("lookup global module enable regmap failed\n");
		return -1;
	}
	interface->module_en_regmap = gpr;

	/*get global soft reset register*/
	gpr = syscon_regmap_lookup_by_phandle(node, "sprd,sys-ap-ahb");
	if (IS_ERR(gpr)) {
		GSP_ERR("lookup global soft-reset regmap failed\n");
		return -1;
	}
	interface->reset_regmap = gpr;

	return 0;
}

int gsp_interface_sc9833_init(struct gsp_interface *intf)
{
	return 0;
}

int gsp_interface_sc9833_deinit(struct gsp_interface *intf)
{
	return 0;
}

int gsp_interface_sc9833_prepare(struct gsp_interface *intf)
{
	return 0;
}

int gsp_interface_sc9833_unprepare(struct gsp_interface *intf)
{
	return 0;
}

int gsp_interface_sc9833_reset(struct gsp_interface *intf)
{
	int ret = -1;
	struct gsp_interface_sc9833 *interface = NULL;

	if (IS_ERR_OR_NULL(intf)) {
		GSP_ERR("interface params error\n");
		return ret;
	}

	interface = (struct gsp_interface_sc9833 *)intf;

	ret = regmap_update_bits(interface->reset_regmap,
				 REG_AP_AHB_AHB_RST,
				 BIT_AP_AHB_GSP_SOFT_RST,
				 BIT_AP_AHB_GSP_SOFT_RST);

	mdelay(1);

	ret = regmap_update_bits(interface->reset_regmap,
				 REG_AP_AHB_AHB_RST,
				 BIT_AP_AHB_GSP_SOFT_RST,
				 0);

	if (ret) {
		GSP_ERR("interface[%s] reset failed\n",
			gsp_interface_to_name(intf));
		return ret;
	}

	GSP_DEBUG("interface[%s] reset success\n",
		  gsp_interface_to_name(intf));
	return ret;
}

void gsp_interface_sc9833_dump(struct gsp_interface *intf)
{

}
