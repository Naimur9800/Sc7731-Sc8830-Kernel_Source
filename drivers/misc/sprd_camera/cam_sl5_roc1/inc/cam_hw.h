/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
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

#ifndef _CAM_HW_H_
#define _CAM_HW_H_


#include <linux/of.h>
#include <linux/platform_device.h>


struct sprd_cam_hw_ops;

struct sprd_cam_hw_info {
	uint32_t idx;
	uint32_t irq_no;
	atomic_t status;
	atomic_t user_cnt;
	struct platform_device *pdev;
	struct regmap *cam_ahb_gpr;
	struct regmap *aon_apb_gpr;
	struct clk *clk;
	struct clk *clk_parent;
	struct clk *clk_default;
	struct clk *bpc_clk;
	struct clk *bpc_clk_parent;
	struct clk *bpc_clk_default;
	struct clk *core_eb;
	struct clk *axi_eb;
	struct clk *mm_eb;

	unsigned long  phy_base;
	unsigned long  reg_base;
	struct sprd_cam_hw_ops *ops;
};


struct sprd_cam_hw_ops {
	int (*init)(struct sprd_cam_hw_info *hw, void *arg);
	int (*deinit)(struct sprd_cam_hw_info *hw, void *arg);
	int (*start)(struct sprd_cam_hw_info *hw, void *arg);
	int (*stop)(struct sprd_cam_hw_info *hw, void *arg);
	int (*reset)(struct sprd_cam_hw_info *hw, void *arg);
	int (*enable_irq)(struct sprd_cam_hw_info *hw, void *arg);
	int (*disable_irq)(struct sprd_cam_hw_info *hw, void *arg);
	int (*irq_clear)(struct sprd_cam_hw_info *hw, void *arg);
	int (*enable_clk)(struct sprd_cam_hw_info *hw, void *arg);
	int (*disable_clk)(struct sprd_cam_hw_info *hw, void *arg);
	int (*update_clk)(struct sprd_cam_hw_info *hw, void *arg);
	int (*trace_reg)(struct sprd_cam_hw_info *hw, void *arg);
};


#endif
