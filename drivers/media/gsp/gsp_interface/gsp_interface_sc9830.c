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


#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd/sc9830/sc9830_glb.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include "gsp_interface_sc9830.h"
#include "../gsp_debug.h"
#include "../gsp_interface.h"


int gsp_interface_sc9830_parse_dt(struct gsp_interface *intf,
				  struct device_node *node)
{
	struct regmap *gpr = NULL;
	struct gsp_interface_sc9830 *interface = NULL;

	interface = (struct gsp_interface_sc9830*)intf;

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

	/*get global auto/gate clock register*/
	gpr = syscon_regmap_lookup_by_phandle(node, "sprd,sys-ap-ahb");
	if (IS_ERR(gpr)) {
		GSP_ERR("lookup global auot-gate regmap failed\n");
		return -1;
	}
	interface->auto_gate_regmap = gpr;

	/*get gsp emc enable register*/
	gpr = syscon_regmap_lookup_by_phandle(node, "sprd,sys-aon-apb");
	if (IS_ERR(gpr)) {
		GSP_ERR("lookup gsp emc regmap failed\n");
		return -1;
	}
	interface->gsp_emc_regmap = gpr;

	return 0;
}

int gsp_interface_sc9830_init(struct gsp_interface *intf)
{
	return 0;
}

int gsp_interface_sc9830_deinit(struct gsp_interface *intf)
{
	return 0;
}

int gsp_interface_sc9830_prepare(struct gsp_interface *intf)
{
	int ret = -1;
	struct gsp_interface_sc9830 *interface = NULL;

	if (IS_ERR_OR_NULL(intf)) {
		GSP_ERR("interface params error\n");
		return ret;
	}

	interface = (struct gsp_interface_sc9830*)intf;
	/* gsp module enable */
	ret = regmap_update_bits(interface->module_en_regmap,
				 REG_AP_AHB_AHB_EB,
				 BIT_AP_AHB_GSP_EB,
				 BIT_AP_AHB_GSP_EB);
	if (ret) {
		GSP_ERR("interface[%s] module enable failed\n",
			gsp_interface_to_name(intf));
		return ret;
	}

	/* gsp emc enable */
	ret = regmap_update_bits(interface->gsp_emc_regmap,
				 REG_AON_APB_APB_EB1,
				 BIT_AON_APB_GSP_EMC_EB,
				 BIT_AON_APB_GSP_EMC_EB);
	if (ret) {
		GSP_ERR("interface[%s] emc enable failed\n",
			gsp_interface_to_name(intf));
		return ret;
	}

	/* gsp auto gate enable */
	ret = regmap_update_bits(interface->auto_gate_regmap,
				 REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
				 BIT_AP_AHB_GSP_AUTO_GATE_EN,
				 BIT_AP_AHB_GSP_AUTO_GATE_EN);
	if (ret) {
		GSP_ERR("interface[%s] auto gate enable failed\n",
			gsp_interface_to_name(intf));
		return ret;
	}

	GSP_DEBUG("interface[%s] prepare success\n",
		  gsp_interface_to_name(intf));
	return ret;
}

int gsp_interface_sc9830_unprepare(struct gsp_interface *intf)
{
	int ret = -1;
	struct gsp_interface_sc9830 *interface = NULL;

	if (IS_ERR_OR_NULL(intf)) {
		GSP_ERR("interface params error\n");
		return ret;
	}

	interface = (struct gsp_interface_sc9830*)intf;
	/* gsp module enable */
	ret = regmap_update_bits(interface->module_en_regmap,
				 REG_AP_AHB_AHB_EB,
				 BIT_AP_AHB_GSP_EB,
				 0);
	if (ret) {
		GSP_ERR("interface[%s] module disable failed\n",
			gsp_interface_to_name(intf));
		return ret;
	}

	/* gsp emc enable */
	ret = regmap_update_bits(interface->gsp_emc_regmap,
				 REG_AON_APB_APB_EB1,
				 BIT_AON_APB_GSP_EMC_EB,
				 0);
	if (ret) {
		GSP_ERR("interface[%s] emc disable failed\n",
			gsp_interface_to_name(intf));
		return ret;
	}

	/* gsp auto gate enable */
	ret = regmap_update_bits(interface->auto_gate_regmap,
				 REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG,
				 BIT_AP_AHB_GSP_AUTO_GATE_EN,
				 0);
	if (ret) {
		GSP_ERR("interface[%s] auto gate disable failed\n",
			gsp_interface_to_name(intf));
		return ret;
	}

	GSP_DEBUG("interface[%s] prepare success\n",
		  gsp_interface_to_name(intf));
	return ret;
}

int gsp_interface_sc9830_reset(struct gsp_interface *intf)
{
	int ret = -1;
	struct gsp_interface_sc9830 *interface = NULL;

	if (IS_ERR_OR_NULL(intf)) {
		GSP_ERR("interface params error\n");
		return ret;
	}

	interface = (struct gsp_interface_sc9830*)intf;

	ret = regmap_update_bits(interface->reset_regmap,
				 REG_AP_AHB_AHB_RST,
				 BIT_AP_AHB_GSP_SOFT_RST,
				 BIT_AP_AHB_GSP_SOFT_RST);
	if (ret) {
		GSP_ERR("interface[%s] reset failed\n",
			gsp_interface_to_name(intf));
		return ret;
	}

	GSP_DEBUG("interface[%s] reset success\n", gsp_interface_to_name(intf));
	return ret;
}

void gsp_interface_sc9830_dump(struct gsp_interface *intf)
{
	int ret = -1;
	unsigned int val = 0;
	int tmp = 0;
	struct gsp_interface_sc9830 *interface = NULL;

	if (IS_ERR_OR_NULL(intf)) {
		GSP_ERR("interface dump params error\n");
		return;
	}

	interface = (struct gsp_interface_sc9830*)intf;
	/* print gsp module enable */
	ret = regmap_read(interface->module_en_regmap, REG_AP_AHB_AHB_EB, &val);
	if (ret) {
		GSP_ERR("module regmap enable print failed\n");
		return;
	}
	tmp = (val & BIT_AP_AHB_GSP_EB) > 0 ? 1 : 0;
	GSP_DEBUG("module regmap enable: %d\n", tmp);

	/* print gsp emc enable */
	ret = regmap_read(interface->module_en_regmap, REG_AON_APB_APB_EB1,
			  &val);
	if (ret) {
		GSP_ERR("emc regmap enable print failed\n");
		return;
	}
	tmp = (val & BIT_AON_APB_GSP_EMC_EB) > 0 ? 1 : 0;
	GSP_DEBUG("emc regmap enable: %d\n", tmp);

	/* print gsp auto gate enable */
	ret = regmap_read(interface->auto_gate_regmap,
			  REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG, &val);
	if (ret) {
		GSP_ERR("auto gate enable print failed\n");
		return;
	}
	tmp = (val & BIT_AP_AHB_GSP_AUTO_GATE_EN) > 0 ? 1 : 0;
	GSP_DEBUG("auto gate regmap enable: %d\n", tmp);
}
