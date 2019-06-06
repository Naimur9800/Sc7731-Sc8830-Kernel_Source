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

#ifndef _GSP_INTERFACE_WHALE2_H
#define _GSP_INTERFACE_WHALE2_H

#include <linux/of.h>
#include <linux/regmap.h>
#include "../gsp_interface.h"


#define GSP_SC9850 "whale2"

#define WHALE2_GSP_MTX_EB_CLOCK_NAME		  ("clk_gsp_mtx_eb")
#define WHALE2_GSP_DISPSYS_EB_CLOCK_NAME		 ("clk_disp_sys_eb")
#define WHALE2_GSP_DISP_CKG_EB_CLOCK_NAME		 ("clk_disp_ckg_eb")
#define WHALE2_GSP_AHB_DISP_CLOCK_NAME   ("clk_gsp_ahb_disp")
#define WHALE2_GSP_AHB_DISP_PARENT_NAME   ("clk_gsp_ahb_disp_parent")

#define WHALE2_GSP_MTX_FORCE_GATE_CLOCK_NAME	("clk_gsp_mtx_force_gate")
#define WHALE2_GSP_NOC_FORCE_GATE_CLOCK_NAME	("clk_gsp_noc_force_gate")



struct WHALE2_AHB_EB_REG {
	union {
		struct {
			uint32_t DISPC0_EB	  :1;
			uint32_t DISPC1_EB	  :1;
			uint32_t DISPC_MMU_EB	  :1;
			uint32_t GSP0_EB	  :1;
			uint32_t GSP1_EB	  :1;
			uint32_t GSP0_MMU_EB	  :1;
			uint32_t GSP1_MMU_EB	  :1;
			uint32_t DSI0_EB	  :1;
			uint32_t DSI1_EB	  :1;
			uint32_t CKG_EB	  :1;
			uint32_t GPU_EB	  :1;
			uint32_t VPP_MMU_EB	  :1;
			uint32_t VPP_EB	  :1;
			uint32_t GPU_MTX_EB	  :1;
			uint32_t GSP_MTX_EB	  :1;
			uint32_t TMC_MTX_EB	  :1;
			uint32_t DISPC_MTX_EB	  :1;
			uint32_t Reserved1		 :15;
		};
		uint32_t	   value;
	};
};


struct WHALE2_AHB_RST_REG {
	union {
		struct {
			uint32_t DISPC0_SOFT_RST	  :1;
			uint32_t DISPC1_SOFT_RST	  :1;
			uint32_t DISPC_MMU_SOFT_RST	  :1;
			uint32_t GSP0_SOFT_RST	  :1;
			uint32_t GSP1_SOFT_RST	  :1;
			uint32_t GSP0_MMU_SOFT_RST	  :1;
			uint32_t GSP1_MMU_SOFT_RST	  :1;
			uint32_t DSI0_SOFT_RST	  :1;
			uint32_t DSI1_SOFT_RST	  :1;
			uint32_t DISPC0_LVDS_SOFT_RST :1;
			uint32_t CKG_SOFT_RST	  :1;
			uint32_t DISPC_MTX_SOFT_RST	  :1;
			uint32_t DISPC0_ENC_SOFT_RST	  :1;
			uint32_t VPP_MMU_SOFT_RST	  :1;
			uint32_t VPP_SOFT_RST	  :1;
			uint32_t GSP_MTX_SOFT_RST	  :1;
			uint32_t Reserved1		 :16;

		};
		uint32_t	   value;
	};
};

struct WHALE2_GEN_CKG_CFG_REG {
	union {
		struct {
			uint32_t DPHY0_CFG_CKG_EN	  :1;
			uint32_t DPHY1_CFG_CKG_EN	  :1;
			uint32_t GSP0_AUTO_CKG_EN	  :1;
			uint32_t GSP1_AUTO_CKG_EN	  :1;
			uint32_t GSP0_FORCE_CKG_EN	  :1;
			uint32_t GSP1_FORCE_CKG_EN	  :1;
			uint32_t DISPC_MTX_FORCE_CKG_EN	  :1;
			uint32_t DISPC_MTX_AUTO_CKG_EN	  :1;
			uint32_t DISPC_NOC_FORCE_CKG_EN	  :1;
			uint32_t DISPC_NOC_AUTO_CKG_EN :1;
			uint32_t GSP_MTX_FORCE_CKG_EN	  :1;
			uint32_t GSP_MTX_AUTO_CKG_EN	  :1;
			uint32_t GSP_NOC_FORCE_CKG_EN	  :1;
			uint32_t GSP_NOC_AUTO_CKG_EN	  :1;
			uint32_t Reserved1		 :18;

		};
		uint32_t	   value;
	};
};

struct WHALE2_DISPC_QOS_CFG_REG {
	union {
		struct {
			uint32_t QOS_W_DISPC0	  :4;
			uint32_t QOS_R_DISPC0	  :4;
			uint32_t QOS_W_DISPC1	  :4;
			uint32_t QOS_R_DISPC1	  :4;
			uint32_t QOS_W_GSP0	  :4;
			uint32_t QOS_R_GSP0	  :4;
			uint32_t QOS_W_GSP1	  :4;
			uint32_t QOS_R_GSP1	  :4;
		};
		uint32_t	   value;
	};
};

struct WHALE2_MISC_CTRL_REG {
	union {
		struct {
			uint32_t DISPC_DDR_ADDR_BIT32	  :1;
			uint32_t GSP_DDR_ADDR_BIT32	  :1;
			uint32_t Reserved1	  :2;
			uint32_t DISPC_NIU_AW_QOS	  :4;
			uint32_t DISPC_NIU_AR_QOS	  :4;
			uint32_t GSP_NIU_AW_QOS	  :4;
			uint32_t GSP_NIU_AR_QOS	  :4;
			uint32_t Reserved2	  :12;
		};
		uint32_t	   value;
	};
};

struct WHALE2_APB_EB1_REG {
	union {
		struct {
			uint32_t Reserved1	  :13;
			uint32_t GSP_EMC_EB	  :1;
			uint32_t Reserved2		 :18;
		};
		uint32_t	   value;
	};
};

struct WHALE2_CGM_GSP0_CFG_REG {
	union {
		struct {
			uint32_t CGM_GSP0_SEL	  :2;
			uint32_t Reserved1		 :6;
			uint32_t Reserved2		 :8;
			uint32_t Reserved3		 :8;
			uint32_t Reserved4		 :8;
		};
		uint32_t	   value;
	};
};

struct WHALE2_CGM_GSP1_CFG_REG {
	union {
		struct {
			uint32_t CGM_GSP1_SEL	  :2;
			uint32_t Reserved1		 :6;
			uint32_t Reserved2		 :8;
			uint32_t Reserved3		 :8;
			uint32_t Reserved4		 :8;
		};
		uint32_t	   value;
	};
};

struct WHALE2_MMU_CTRL_REG {
	union {
		struct {
			uint32_t MMU_EN	  :1;
			uint32_t TLB_EN		 :1;
			uint32_t RAMCLK_DIV2_EN		 :1;
			uint32_t Reserved1		 :17;
			uint32_t START_MB_ADDR		 :12;
		};
		uint32_t	   value;
	};
};

struct gsp_interface_whale2 {
	struct gsp_interface common;
	struct regmap *auto_gate_regmap;
	struct regmap *gsp_emc_regmap;
	struct regmap *module_en_regmap;
	struct regmap *reset_regmap;
	struct clk *gsp_mtx_eb_clk;
	struct clk *disp_sys_eb_clk;
	struct clk *disp_ckg_eb_clk;
	struct clk *gsp_ahb_disp_clk;
	struct clk *gsp_ahb_disp_parent_clk;
	struct clk *gsp_mtx_force_gate_clk;
	struct clk *gsp_noc_force_gate_clk;
};



int gsp_interface_whale2_parse_dt(struct gsp_interface *intf,
				  struct device_node *node);

int gsp_interface_whale2_init(struct gsp_interface *intf);
int gsp_interface_whale2_deinit(struct gsp_interface *intf);

int gsp_interface_whale2_prepare(struct gsp_interface *intf);
int gsp_interface_whale2_unprepare(struct gsp_interface *intf);

int gsp_interface_whale2_reset(struct gsp_interface *intf);

void gsp_interface_whale2_dump(struct gsp_interface *inf);

#endif
