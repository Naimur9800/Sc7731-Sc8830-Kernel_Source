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

#ifndef _GSP_INTERFACE_ISHARKL2_H
#define _GSP_INTERFACE_ISHARKL2_H

#include <linux/of.h>
#include <linux/regmap.h>
#include "../gsp_interface.h"


#define GSP_ISHARKL2 "isharkl2"

#define ISHARKL2_GSP_EMC_EB_NAME		  ("clk_gsp_emc_eb")
#define ISHARKL2_GSP_AON_VSP_EB_NAME		 ("clk_aon_vsp_eb")
#define ISHARKL2_GSP_VSP_AHB_CKG_EB_NAME		 ("clk_vsp_ahb_ckg_eb")


#define ISHARKL2_GSP_AHB_VSP_CLOCK_NAME   ("clk_gsp_ahb_vsp")
#define ISHARKL2_GSP_AHB_VSP_PARENT_NAME   ("clk_gsp_ahb_vsp_parent")
#define ISHARKL2_VSYS_MTX_CLOCK_NAME   ("clk_vsys_mtx")
#define ISHARKL2_VSYS_MTX_PARENT_NAME   ("clk_vsys_mtx_parent")

struct ISHARKL2_AHB_EB_REG {
	union {
		struct {
			uint32_t VSP_1080P_EN	  :1;
			uint32_t CKG_EB	  :1;
			uint32_t Reserved1	  :3;
			uint32_t GSP0_EB	  :1;
			uint32_t Reserved2	  :4;
			uint32_t SYS_MTX_EB	  :1;
			uint32_t Reserved3	  :1;
			uint32_t VSP_Busmon_EB	  :1;
			uint32_t GSP_Busmon_EB	  :1;
			uint32_t VSP_Memfw_EB	  :1;
			uint32_t Reserved4		 :17;
		};
		uint32_t	   value;
	};
};

struct ISHARKL2_AHB_RST_REG {
	union {
		struct {
			uint32_t VSP_1080P_SOFT_RST	  :1;
			uint32_t Reserved1	  :1;
			uint32_t CKG_SOFT_RST	  :1;
			uint32_t Reserved2	  :3;
			uint32_t GSP0_SOFT_RST	  :1;
			uint32_t Reserved3	  :1;
			uint32_t MEM_FW_SOFT_RST	  :1;
			uint32_t Reserved4 :23;
		};
		uint32_t	   value;
	};
};

struct gsp_interface_isharkl2 {
	struct gsp_interface common;

	struct clk *aon_vsp_eb_clk;
	struct clk *gsp_emc_eb_clk;

	struct clk *vsp_ahb_ckg_eb_clk;

	struct clk *ahb_vsp_cfg_clk;
	struct clk *ahb_vsp_cfg_parent_clk;
	struct clk *vsys_mtx_cfg_clk;
	struct clk *vsys_mtx_cfg_parent_clk;

	struct regmap *aon_pwu_regmap;
	struct regmap *aon_apb_regmap;

	void __iomem *vsp_clk_regmap;
};



int gsp_interface_isharkl2_parse_dt(struct gsp_interface *intf,
				  struct device_node *node);

int gsp_interface_isharkl2_init(struct gsp_interface *intf);
int gsp_interface_isharkl2_deinit(struct gsp_interface *intf);

int gsp_interface_isharkl2_prepare(struct gsp_interface *intf);
int gsp_interface_isharkl2_unprepare(struct gsp_interface *intf);

int gsp_interface_isharkl2_reset(struct gsp_interface *intf);

void gsp_interface_isharkl2_dump(struct gsp_interface *inf);

#endif
