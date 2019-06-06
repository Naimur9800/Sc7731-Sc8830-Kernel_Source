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

#ifndef _GSP_INTERFACE_IWHALE2_H
#define _GSP_INTERFACE_IWHALE2_H

#include <linux/of.h>
#include <linux/regmap.h>
#include "../gsp_interface.h"


#define GSP_IWHALE2 "iwhale2"

#define IWHALE2_GSP_EMC_EB_NAME		  ("clk_gsp_emc_eb")
#define IWHALE2_GSP_AON_VSP_EB_NAME		 ("clk_aon_vsp_eb")
#define IWHALE2_GSP_VSP_AHB_CKG_EB_NAME		 ("clk_vsp_ahb_ckg_eb")
#define IWHALE2_GSP_MMU_EB_NAME	("clk_gsp_mmu_eb")
#define IWHALE2_GSP_MTX_EB_NAME	("clk_gsp_mtx_eb")


#define IWHALE2_GSP_AHB_VSP_CLOCK_NAME   ("clk_gsp_ahb_vsp")
#define IWHALE2_GSP_AHB_VSP_PARENT_NAME   ("clk_gsp_ahb_vsp_parent")
#define IWHALE2_GSP_MTX_CLOCK_NAME   ("clk_gsp_mtx")
#define IWHALE2_GSP_MTX_PARENT_NAME   ("clk_gsp_mtx_parent")
#define IWHALE2_VSYS_MTX_CLOCK_NAME   ("clk_vsys_mtx")
#define IWHALE2_VSYS_MTX_PARENT_NAME   ("clk_vsys_mtx_parent")

struct IWHALE2_AON_PWU_VSP_REG {
	union {
		struct {
			uint32_t PD_VSP_SYS_ISO_ON_DLY	  :8;
			uint32_t PD_VSP_SYS_PWR_ON_SEQ_DLY	  :8;
			uint32_t PD_VSP_SYS_PWR_ON_DLY	  :8;
			uint32_t PD_VSP_SYS_AUTO_SHUTDOWN_EN	  :1;
			uint32_t PD_VSP_SYS_FORCE_SHUTDOWN	  :1;
			uint32_t Reserved0	  :2;
			uint32_t PD_VSP_SYS_DBG_SHUTDOWN_EN	  :1;
			uint32_t Reserved1	  :3;
		};
		uint32_t	   value;
	};
};

struct IWHALE2_AON_APB_EB1_REG {
	union {
		struct {
			uint32_t PMU_EB	  :1;
			uint32_t THM_EB	  :1;
			uint32_t AUX0_EB	  :1;
			uint32_t AUX1_EB	  :1;
			uint32_t AUX2_EB	  :1;
			uint32_t PROBE_EB	  :1;
			uint32_t AVS_GPU0_EB	  :1;
			uint32_t AVS_GPU1_EB	  :1;
			uint32_t AP_WDG_EB	  :1;
			uint32_t AP_TMR1_EB	  :1;
			uint32_t AP_TMR2_EB	  :1;
			uint32_t DISP_EMC_EB	  :1;
			uint32_t ZIP_EMC_EB	  :1;
			uint32_t GSP_EMC_EB	  :1;
			uint32_t OSC_AON_TOP_EB	  :1;
			uint32_t BIA_THM0_EB	  :1;
			uint32_t BIA_THM1_EB	  :1;
			uint32_t MDAR_EB	  :1;
			uint32_t CP_APB_EB	  :1;
			uint32_t RC100M_CAL_EB	  :1;
			uint32_t DJTAG_EB	  :1;
			uint32_t MBOX_EB	  :1;
			uint32_t IO_APB_EB	  :1;
			uint32_t THM1_EB	  :1;
			uint32_t LVDS_PLL_DIV_EN	  :1;
			uint32_t DEF_EB	  :1;
			uint32_t ANALOG_APB_EB	  :1;
			uint32_t ORP_JTAG_EB	  :1;
			uint32_t AON_VSP_EB	  :1;
			uint32_t AON_CAM_EB	  :1;
			uint32_t AON_DISP_EB	  :1;
			uint32_t DBG_AXI_IF_EB	  :1;
		};
		uint32_t	   value;
	};
};

struct IWHALE2_AHB_EB_REG {
	union {
		struct {
			uint32_t VSP_CODEC_EB	  :1;
			uint32_t CKG_EB	  :1;
			uint32_t VSP_MMU_EB	  :1;
			uint32_t VSP_ENC_EB	  :1;
			uint32_t VPP_EB	  :1;
			uint32_t GSP0_EB	  :1;
			uint32_t GSP1_EB	  :1;
			uint32_t GSP_MMU_EB	  :1;
			uint32_t Reserved1	  :1;
			uint32_t GSP_MTX_EB	  :1;
			uint32_t SYS_MTX_EB	  :1;
			uint32_t Reserved2	  :1;
			uint32_t VSP_Busmon_EB	  :1;
			uint32_t GSP_Busmon_EB	  :1;
			uint32_t VSP_Memfw_EB	  :1;
			uint32_t Reserved3		 :17;
		};
		uint32_t	   value;
	};
};

struct IWHALE2_AHB_RST_REG {
	union {
		struct {
			uint32_t VSP_HEVC_SOFT_RST	  :1;
			uint32_t GSP_MMU_SOFT_RST	  :1;
			uint32_t CKG_SOFT_RST	  :1;
			uint32_t VSP_MMU_SOFT_RST	  :1;
			uint32_t VSP_ENC_SOFT_RST	  :1;
			uint32_t VPP_SOFT_RST	  :1;
			uint32_t GSP0_SOFT_RST	  :1;
			uint32_t GSP1_SOFT_RST	  :1;
			uint32_t MEM_FW_SOFT_RST	  :1;
			uint32_t Reserved1 :1;
			uint32_t GSP_MTX_SOFT_RST	  :1;
			uint32_t Reserved2	  :21;
		};
		uint32_t	   value;
	};
};

struct gsp_interface_iwhale2 {
	struct gsp_interface common;

	struct clk *aon_vsp_eb_clk;
	struct clk *gsp_emc_eb_clk;

	struct clk *vsp_ahb_ckg_eb_clk;
	struct clk *gsp_mmu_eb_clk;
	struct clk *gsp_mtx_eb_clk;

	struct clk *ahb_vsp_cfg_clk;
	struct clk *ahb_vsp_cfg_parent_clk;
	struct clk *gsp_mtx_cfg_clk;
	struct clk *gsp_mtx_cfg_parent_clk;
	struct clk *vsys_mtx_cfg_clk;
	struct clk *vsys_mtx_cfg_parent_clk;

	struct regmap *aon_pwu_regmap;
	struct regmap *aon_apb_regmap;

	void __iomem *vsp_clk_regmap;
};

int gsp_interface_iwhale2_parse_dt(struct gsp_interface *intf,
				  struct device_node *node);

int gsp_interface_iwhale2_init(struct gsp_interface *intf);
int gsp_interface_iwhale2_deinit(struct gsp_interface *intf);

int gsp_interface_iwhale2_prepare(struct gsp_interface *intf);
int gsp_interface_iwhale2_unprepare(struct gsp_interface *intf);

int gsp_interface_iwhale2_reset(struct gsp_interface *intf);

void gsp_interface_iwhale2_dump(struct gsp_interface *inf);

#endif
