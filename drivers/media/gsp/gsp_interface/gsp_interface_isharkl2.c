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
#include <linux/of_address.h>
#include "gsp_interface_isharkl2.h"
#include "../gsp_debug.h"
#include "../gsp_interface.h"
#include <linux/delay.h>
#include <video/sprd_vsp_pw_domain.h>

int gsp_interface_isharkl2_parse_dt(struct gsp_interface *intf,
				  struct device_node *node)
{
	int status = 0;
	struct gsp_interface_isharkl2 *gsp_interface = NULL;
	struct resource r;

	gsp_interface = (struct gsp_interface_isharkl2 *)intf;

	gsp_interface->aon_vsp_eb_clk = of_clk_get_by_name(node,
		ISHARKL2_GSP_AON_VSP_EB_NAME);
	gsp_interface->gsp_emc_eb_clk = of_clk_get_by_name(node,
		ISHARKL2_GSP_EMC_EB_NAME);
	gsp_interface->vsp_ahb_ckg_eb_clk = of_clk_get_by_name(node,
		ISHARKL2_GSP_VSP_AHB_CKG_EB_NAME);

	gsp_interface->ahb_vsp_cfg_clk = of_clk_get_by_name(node,
		ISHARKL2_GSP_AHB_VSP_CLOCK_NAME);
	gsp_interface->ahb_vsp_cfg_parent_clk = of_clk_get_by_name(node,
		ISHARKL2_GSP_AHB_VSP_PARENT_NAME);
	gsp_interface->vsys_mtx_cfg_clk = of_clk_get_by_name(node,
		ISHARKL2_VSYS_MTX_CLOCK_NAME);
	gsp_interface->vsys_mtx_cfg_parent_clk = of_clk_get_by_name(node,
		ISHARKL2_VSYS_MTX_PARENT_NAME);

	gsp_interface->aon_pwu_regmap =
		syscon_regmap_lookup_by_compatible("sprd,iwhale2-aon-pwu-apb");
	if (IS_ERR(gsp_interface->aon_pwu_regmap)) {
		GSP_ERR("r5p0 interface map aon pwu domain reg failed\n");
		status |= -1;
	}
	gsp_interface->aon_apb_regmap =
		syscon_regmap_lookup_by_compatible("sprd,iwhale2-aon-apb");
	if (IS_ERR(gsp_interface->aon_apb_regmap)) {
		GSP_ERR("r5p0 interface map aon apb domain reg failed\n");
		status |= -1;
	}

	if (of_address_to_resource(node, 2, &r)) {
		GSP_ERR("Line(%d), parse vsp sys regmap fail\n", __LINE__);
		status |= -1;
		return status;
	}
	gsp_interface->vsp_clk_regmap =
		ioremap_nocache(r.start, resource_size(&r));
	if (IS_ERR(gsp_interface->vsp_clk_regmap)) {
		GSP_ERR("r5p0 interface map vsp_clk_regmap reg failed\n");
		status |= -1;
	}
	GSP_INFO("vsp_sys_regmap(0xD1000000): 0x%p\n",
		gsp_interface->vsp_clk_regmap);

	if (IS_ERR_OR_NULL(gsp_interface->aon_vsp_eb_clk)) {
		GSP_ERR("r5p0 interface read gsp aon_vsp_eb_clk  failed\n");
		status |= -1;
	}
	if (IS_ERR_OR_NULL(gsp_interface->gsp_emc_eb_clk)) {
		GSP_ERR("r5p0 interface read gsp gsp_emc_eb_clk failed\n");
		status |= -1;
	}
	if (IS_ERR_OR_NULL(gsp_interface->vsp_ahb_ckg_eb_clk)) {
		GSP_ERR("r5p0 interface read gsp vsp_ahb_ckg_eb_clk  failed\n");
		status |= -1;
	}

	if (IS_ERR_OR_NULL(gsp_interface->ahb_vsp_cfg_clk)
		|| IS_ERR_OR_NULL(gsp_interface->ahb_vsp_cfg_parent_clk)) {
		GSP_ERR("r5p0 read gsp ahb_vsp_cfg or parent_clk failed\n");
		status = -1;
	}
	if (IS_ERR_OR_NULL(gsp_interface->vsys_mtx_cfg_clk)
		|| IS_ERR_OR_NULL(gsp_interface->vsys_mtx_cfg_parent_clk)) {
		GSP_ERR("r5p0 get gsp vsys_mtx_cfg_clk or parent_clk failed\n");
		status = -1;
	}

	return status;
}

int gsp_interface_isharkl2_init(struct gsp_interface *intf)
{
#ifdef	CONFIG_SPRD_VSP
	vsp_pw_on(VSP_PW_DOMAIN_VSP_GSP);
#endif
	return 0;
}

int gsp_interface_isharkl2_deinit(struct gsp_interface *intf)
{
	struct gsp_interface_isharkl2 *gsp_interface = NULL;

	gsp_interface = (struct gsp_interface_isharkl2 *)intf;
#ifdef	CONFIG_SPRD_VSP
	vsp_pw_off(VSP_PW_DOMAIN_VSP_GSP);
#endif
	return 0;
}

int gsp_interface_isharkl2_prepare(struct gsp_interface *intf)
{
	int ret = -1;
	struct gsp_interface_isharkl2 *gsp_interface = NULL;

	if (IS_ERR_OR_NULL(intf)) {
		GSP_ERR("interface params error\n");
		return ret;
	}

	gsp_interface = (struct gsp_interface_isharkl2 *)intf;


	ret = clk_prepare_enable(gsp_interface->aon_vsp_eb_clk);
	if (ret) {
		GSP_ERR("enable interface[%s] aon_vsp_eb_clk failed\n",
			gsp_interface_to_name(intf));
		goto aon_vsp_eb_clk_disable;
	}

	ret = clk_prepare_enable(gsp_interface->gsp_emc_eb_clk);
	if (ret) {
		GSP_ERR("enable interface[%s] gsp_emc_eb_clk failed\n",
			gsp_interface_to_name(intf));
		goto gsp_emc_eb_clk_disable;
	}

	ret = clk_prepare_enable(gsp_interface->vsp_ahb_ckg_eb_clk);
	if (ret) {
		GSP_ERR("enable interface[%s] vsp_ahb_ckg_eb_clk failed\n",
			gsp_interface_to_name(intf));
		goto vsp_ahb_ckg_eb_clk_disable;
	}

	clk_set_parent(gsp_interface->ahb_vsp_cfg_clk, NULL);
	clk_set_parent(gsp_interface->ahb_vsp_cfg_clk,
		gsp_interface->ahb_vsp_cfg_parent_clk);

	ret = clk_prepare_enable(gsp_interface->ahb_vsp_cfg_clk);
	if (ret) {
		GSP_ERR("enable interface[%s] ahb_vsp_cfg_clk clk failed\n",
			gsp_interface_to_name(intf));
		goto ahb_vsp_cfg_clk_disable;
	}

	clk_set_parent(gsp_interface->vsys_mtx_cfg_clk, NULL);
	clk_set_parent(gsp_interface->vsys_mtx_cfg_clk,
		gsp_interface->vsys_mtx_cfg_parent_clk);

	ret = clk_prepare_enable(gsp_interface->vsys_mtx_cfg_clk);
	if (ret) {
		GSP_ERR("enable interface[%s] vsys_mtx_cfg_clk clk failed\n",
			gsp_interface_to_name(intf));
		goto vsys_mtx_cfg_clk_disable;
	}

	goto exit;

vsys_mtx_cfg_clk_disable:
	clk_disable_unprepare(gsp_interface->vsys_mtx_cfg_clk);
ahb_vsp_cfg_clk_disable:
	clk_disable_unprepare(gsp_interface->ahb_vsp_cfg_clk);
vsp_ahb_ckg_eb_clk_disable:
	clk_disable_unprepare(gsp_interface->vsp_ahb_ckg_eb_clk);
gsp_emc_eb_clk_disable:
	clk_disable_unprepare(gsp_interface->gsp_emc_eb_clk);
aon_vsp_eb_clk_disable:
	clk_disable_unprepare(gsp_interface->aon_vsp_eb_clk);
	GSP_ERR("interface[%s] prepare ERR !\n",
			  gsp_interface_to_name(intf));

exit:
	GSP_DEBUG("interface[%s] prepare success\n",
			  gsp_interface_to_name(intf));
	return ret;
}

int gsp_interface_isharkl2_unprepare(struct gsp_interface *intf)
{
	struct gsp_interface_isharkl2 *gsp_interface = NULL;

	if (IS_ERR_OR_NULL(intf)) {
		GSP_ERR("interface params error\n");
		return -1;
	}

	gsp_interface = (struct gsp_interface_isharkl2 *)intf;

	clk_disable_unprepare(gsp_interface->vsys_mtx_cfg_clk);
	clk_disable_unprepare(gsp_interface->ahb_vsp_cfg_clk);
	clk_disable_unprepare(gsp_interface->vsp_ahb_ckg_eb_clk);
	clk_disable_unprepare(gsp_interface->gsp_emc_eb_clk);
	clk_disable_unprepare(gsp_interface->aon_vsp_eb_clk);

	GSP_DEBUG("interface[%s] unprepare success\n",
		  gsp_interface_to_name(intf));
	return 0;
}

int gsp_interface_isharkl2_reset(struct gsp_interface *intf)
{
	GSP_INFO("interface[%s] reset not implemented yet\n",
			gsp_interface_to_name(intf));
	return 0;
}

void gsp_interface_isharkl2_dump(struct gsp_interface *intf)
{
	uint32_t aon_pwu_vsp, aon_apb_vsp, vsp_sysmtx_clk;
	uint32_t vsp_ahb_clk, gsp0_clk;
	struct gsp_interface_isharkl2 *gsp_interface = NULL;

	gsp_interface = (struct gsp_interface_isharkl2 *)intf;

	regmap_read(gsp_interface->aon_pwu_regmap,
				 0x4C, &aon_pwu_vsp);
	regmap_read(gsp_interface->aon_apb_regmap,
				 0x4, &aon_apb_vsp);

	vsp_sysmtx_clk =
		gsp_core_reg_read((gsp_interface->vsp_clk_regmap + 0x2C));
	vsp_ahb_clk =
		gsp_core_reg_read((gsp_interface->vsp_clk_regmap + 0x20));
	gsp0_clk =
		gsp_core_reg_read((gsp_interface->vsp_clk_regmap + 0x28));

	GSP_DUMP("0xE42B004C[0x%x], 0xE42E0004[0x%x]\n",
		aon_pwu_vsp, aon_apb_vsp);

	GSP_DUMP("0xD1000020[0x%x],0xD1000028[0x%x],0xD100002C[0x%x]\n",
		vsp_ahb_clk, gsp0_clk, vsp_sysmtx_clk);
}
