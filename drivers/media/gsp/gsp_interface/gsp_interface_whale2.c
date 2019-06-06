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
#include <linux/regmap.h>
#include <linux/of.h>
#include <video/disp_pw_domain.h>
#include "gsp_interface_whale2.h"
#include "../gsp_debug.h"
#include "../gsp_interface.h"


int gsp_interface_whale2_parse_dt(struct gsp_interface *intf,
				  struct device_node *node)
{
	int status = 0;
	struct gsp_interface_whale2 *gsp_interface = NULL;

	gsp_interface = (struct gsp_interface_whale2 *)intf;

	gsp_interface->gsp_ahb_disp_clk = of_clk_get_by_name(node,
		WHALE2_GSP_AHB_DISP_CLOCK_NAME);
	gsp_interface->gsp_ahb_disp_parent_clk = of_clk_get_by_name(node,
		WHALE2_GSP_AHB_DISP_PARENT_NAME);
	gsp_interface->disp_ckg_eb_clk = of_clk_get_by_name(node,
		WHALE2_GSP_DISP_CKG_EB_CLOCK_NAME);
	gsp_interface->disp_sys_eb_clk = of_clk_get_by_name(node,
		WHALE2_GSP_DISPSYS_EB_CLOCK_NAME);
	gsp_interface->gsp_mtx_eb_clk = of_clk_get_by_name(node,
		WHALE2_GSP_MTX_EB_CLOCK_NAME);
	gsp_interface->gsp_mtx_force_gate_clk = of_clk_get_by_name(node,
		WHALE2_GSP_MTX_FORCE_GATE_CLOCK_NAME);
	gsp_interface->gsp_noc_force_gate_clk = of_clk_get_by_name(node,
		WHALE2_GSP_NOC_FORCE_GATE_CLOCK_NAME);
	gsp_interface->module_en_regmap =
		 syscon_regmap_lookup_by_compatible("sprd,sys-disp-ahb");

	if (IS_ERR_OR_NULL(gsp_interface->gsp_ahb_disp_clk)
		|| IS_ERR_OR_NULL(gsp_interface->gsp_ahb_disp_parent_clk)) {
		GSP_ERR("r3p0 interface read gsp gsp_ahb_disp_clk failed\n");
		status = -1;
	}
	if (IS_ERR_OR_NULL(gsp_interface->disp_ckg_eb_clk)) {
		GSP_ERR("r3p0 interface read gsp disp_ckg_eb_clk failed\n");
		status |= -1;
	}
	if (IS_ERR_OR_NULL(gsp_interface->disp_sys_eb_clk)) {
		GSP_ERR("r3p0 interface read gsp disp_sys_eb_clk  failed\n");
		status |= -1;
	}
	if (IS_ERR_OR_NULL(gsp_interface->gsp_mtx_eb_clk)) {
		GSP_ERR("r3p0 interface read gsp gsp_mtx_eb_clk  failed\n");
		status |= -1;
	}
	if (IS_ERR_OR_NULL(gsp_interface->gsp_mtx_force_gate_clk)) {
		GSP_ERR("r3p0 interface read gsp mtx_force_gate_clk  failed\n");
		status |= -1;
	}
	if (IS_ERR_OR_NULL(gsp_interface->gsp_noc_force_gate_clk)) {
		GSP_ERR("r3p0 interface read gsp noc_force_gate_clk  failed\n");
		status |= -1;
	}
	if (IS_ERR(gsp_interface->module_en_regmap)) {
		GSP_ERR("r3p0 interface map disp domain reg failed\n");
		status |= -1;
	}

	return status;
}

int gsp_interface_whale2_init(struct gsp_interface *intf)
{
	disp_pw_on(DISP_PW_DOMAIN_GSP);
	return 0;
}

int gsp_interface_whale2_deinit(struct gsp_interface *intf)
{
	disp_pw_off(DISP_PW_DOMAIN_GSP);
	return 0;
}

int gsp_interface_whale2_prepare(struct gsp_interface *intf)
{
	int ret = -1;
	struct gsp_interface_whale2 *gsp_interface = NULL;

	if (IS_ERR_OR_NULL(intf)) {
		GSP_ERR("interface params error\n");
		return ret;
	}

	gsp_interface = (struct gsp_interface_whale2 *)intf;

	ret = clk_prepare_enable(gsp_interface->disp_sys_eb_clk);
	if (ret) {
		GSP_ERR("enable interface[%s] disp_sys_eb_clkclk failed\n",
			gsp_interface_to_name(intf));
		goto disp_sys_eb_clk_disable;
	}

	clk_set_parent(gsp_interface->gsp_ahb_disp_clk, NULL);
	clk_set_parent(gsp_interface->gsp_ahb_disp_clk,
		gsp_interface->gsp_ahb_disp_parent_clk);

	ret = clk_prepare_enable(gsp_interface->gsp_ahb_disp_clk);
	if (ret) {
		GSP_ERR("enable interface[%s] gsp_ahb_disp_clk failed\n",
			gsp_interface_to_name(intf));
		goto gsp_ahb_disp_clk_disable;
	}

	ret = clk_prepare_enable(gsp_interface->disp_ckg_eb_clk);
	if (ret) {
		GSP_ERR("enable interface[%s] disp_ckg_eb_clk failed\n",
			gsp_interface_to_name(intf));
		goto disp_ckg_eb_clk_disable;
	}

	ret = clk_prepare_enable(gsp_interface->gsp_mtx_eb_clk);
	if (ret) {
		GSP_ERR("enable interface[%s] gsp_mtx_eb_clk clk failed\n",
			gsp_interface_to_name(intf));
		goto gsp_mtx_eb_clk_disable;
	}

	ret = clk_prepare_enable(gsp_interface->gsp_mtx_force_gate_clk);
	if (ret) {
		GSP_ERR("enable interface[%s] gsp_mtx_force_gate_clk failed\n",
			gsp_interface_to_name(intf));
		goto gsp_mtx_force_gate_clk_disable;
	}

	ret = clk_prepare_enable(gsp_interface->gsp_noc_force_gate_clk);
	if (ret) {
		GSP_ERR("enable interface[%s] gsp_noc_force_clk clk failed\n",
			gsp_interface_to_name(intf));
		goto gsp_noc_force_gate_clk_disable;
	}

	goto exit;

gsp_noc_force_gate_clk_disable:
	clk_disable_unprepare(gsp_interface->gsp_noc_force_gate_clk);
gsp_mtx_force_gate_clk_disable:
	clk_disable_unprepare(gsp_interface->gsp_mtx_force_gate_clk);
gsp_mtx_eb_clk_disable:
	clk_disable_unprepare(gsp_interface->gsp_mtx_eb_clk);
disp_ckg_eb_clk_disable:
	clk_disable_unprepare(gsp_interface->disp_ckg_eb_clk);
gsp_ahb_disp_clk_disable:
	clk_disable_unprepare(gsp_interface->gsp_ahb_disp_clk);
disp_sys_eb_clk_disable:
	clk_disable_unprepare(gsp_interface->disp_sys_eb_clk);
	GSP_ERR("interface[%s] prepare ERR !\n",
			  gsp_interface_to_name(intf));


exit:
	GSP_DEBUG("interface[%s] prepare success\n",
			  gsp_interface_to_name(intf));
	return ret;
}

int gsp_interface_whale2_unprepare(struct gsp_interface *intf)
{
	struct gsp_interface_whale2 *gsp_interface = NULL;

	if (IS_ERR_OR_NULL(intf)) {
		GSP_ERR("interface params error\n");
		return -1;
	}

	gsp_interface = (struct gsp_interface_whale2 *)intf;

	clk_disable_unprepare(gsp_interface->gsp_noc_force_gate_clk);
	clk_disable_unprepare(gsp_interface->gsp_mtx_force_gate_clk);
	clk_disable_unprepare(gsp_interface->gsp_mtx_eb_clk);
	clk_disable_unprepare(gsp_interface->disp_sys_eb_clk);
	clk_disable_unprepare(gsp_interface->disp_ckg_eb_clk);
	clk_disable_unprepare(gsp_interface->gsp_ahb_disp_clk);

	GSP_DEBUG("interface[%s] unprepare success\n",
		  gsp_interface_to_name(intf));
	return 0;
}

int gsp_interface_whale2_reset(struct gsp_interface *intf)
{
	int ret = 0;

	GSP_INFO("interface[%s] reset not implemented yet\n",
			gsp_interface_to_name(intf));
	return ret;
}

void gsp_interface_whale2_dump(struct gsp_interface *intf)
{
	struct WHALE2_AHB_EB_REG whale2_ahb_reg;
	struct WHALE2_GEN_CKG_CFG_REG  whale2_gen_clk_cfg_reg;
	struct gsp_interface_whale2 *gsp_interface = NULL;

	gsp_interface = (struct gsp_interface_whale2 *)intf;

	regmap_read(gsp_interface->module_en_regmap,
				 0x0, &whale2_ahb_reg.value);
	regmap_read(gsp_interface->module_en_regmap,
				 0x8, &whale2_gen_clk_cfg_reg.value);

	GSP_INFO("[Whale2] 0x63100000: 0x%x, 0x63100008: 0x%x\n",
			 whale2_ahb_reg.value,
			 whale2_gen_clk_cfg_reg.value);
	GSP_INFO("GSP0_EB: %d,GSP0_MMU_EB: %d,GSP1_EB: %d,GSP1_MMU_EB: %d\n",
			 whale2_ahb_reg.GSP0_EB, whale2_ahb_reg.GSP0_MMU_EB,
			 whale2_ahb_reg.GSP1_EB, whale2_ahb_reg.GSP1_MMU_EB);
	GSP_INFO("CKG_EB: 0x%x, GSP_MTX_EB: 0x%x\n",
		 whale2_ahb_reg.CKG_EB, whale2_ahb_reg.GSP_MTX_EB);
	GSP_INFO("GSP0_AUTO_CKG_EN: 0x%x, GSP0_FORCE_CKG_EN: 0x%x\n",
			 whale2_gen_clk_cfg_reg.GSP0_AUTO_CKG_EN,
			 whale2_gen_clk_cfg_reg.GSP0_FORCE_CKG_EN);
	GSP_INFO("GSP1_AUTO_CKG_EN: 0x%x, GSP1_FORCE_CKG_EN: 0x%x\n",
			 whale2_gen_clk_cfg_reg.GSP1_AUTO_CKG_EN,
			 whale2_gen_clk_cfg_reg.GSP1_FORCE_CKG_EN);
	GSP_INFO("GSP_MTX_AUTO_CKG_EN: 0x%x, GSP_MTX_FORCE_CKG_EN: 0x%x\n",
			 whale2_gen_clk_cfg_reg.GSP_MTX_AUTO_CKG_EN,
			 whale2_gen_clk_cfg_reg.GSP_MTX_FORCE_CKG_EN);
	GSP_INFO("GSP_NOC_AUTO_CKG_EN: 0x%x, GSP_NOC_FORCE_CKG_EN: 0x%x\n",
			 whale2_gen_clk_cfg_reg.GSP_NOC_AUTO_CKG_EN,
			 whale2_gen_clk_cfg_reg.GSP_NOC_FORCE_CKG_EN);
}
