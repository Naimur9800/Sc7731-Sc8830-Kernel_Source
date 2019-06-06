/*
 * Spreadtrum Clock support
 *
 * Copyright (C) 2015 Spreadtrum, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include "clk-sprd.h"

static u8 sprd_clk_mux_get_parent(struct clk_hw *hw)
{
	/* check power status, if no power check use default value */
	return clk_mux_ops.get_parent(hw);
}

static int sprd_clk_mux_set_parent(struct clk_hw *hw, u8 index)
{
	/* check power status, if no power use default value*/
	return clk_mux_ops.set_parent(hw, index);
}

static unsigned long sprd_clk_divider_recalc_rate(struct clk_hw *hw,
				unsigned long parent_rate)
{
	/* check power status, if no power use default value */
	/* if no need round up, should not use the clk_divider_ops */
	return clk_divider_ops.recalc_rate(hw, parent_rate);
}

static long sprd_clk_divider_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *prate)
{
	/* actually, sprd chip no need use this function now */
	return rate;
}

static int sprd_clk_divider_set_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	/* check power status, if no power use default value */
	return clk_divider_ops.set_rate(hw, rate, parent_rate);
}

struct clk_ops sprd_mux_ops = {
	.set_parent = sprd_clk_mux_set_parent,
	.get_parent = sprd_clk_mux_get_parent,
};

struct clk_ops sprd_divider_ops = {
	.recalc_rate = sprd_clk_divider_recalc_rate,
	.round_rate = sprd_clk_divider_round_rate,
	.set_rate = sprd_clk_divider_set_rate,
};

void __init of_sprd_composite_clk_setup(struct device_node *node)
{
	struct clk *clk;
	struct clk_mux *mux = NULL;
	const struct clk_ops *mux_ops = NULL;
	struct clk_divider *div = NULL;
	const struct clk_ops *div_ops = NULL;
	const char *clk_name = node->name;
	const char **parent_names;
	unsigned long flags = 0;
	u32 msk;
	int num_parents = 0;
	int i = 0;
	int index = 0;

	of_property_read_string(node, "clock-output-names", &clk_name);

	num_parents = of_clk_get_parent_count(node);
	if (num_parents < 0) {
		pr_err("clk: sprd: get mux clock %s's parent num failed!\n",
			clk_name);
		return;
	}

	parent_names = kzalloc((sizeof(char *) * num_parents), GFP_KERNEL);
	if (!parent_names)
		return;

	while (i < num_parents &&
			(parent_names[i] =
			 of_clk_get_parent_name(node, i)) != NULL)
		i++;

	if (!of_property_read_u32(node, "sprd,mux-msk", &msk)) {
		mux = kzalloc(sizeof(struct clk_mux), GFP_KERNEL);
		if (mux == NULL) {
			clk_err("register composite clk %s failed!\n",
				clk_name);
			goto kfree_parent_names;
		}

		mux->reg = of_iomap(node, index);
		if (!mux->reg)
			goto kfree_mux;
		index++;
		mux->shift = __ffs(msk);
		mux->mask = msk >> (mux->shift);
		mux_ops = &sprd_mux_ops;
		clk_debug("mux %u, %x\n", (u32) mux->shift, mux->mask);
	}

	if (!of_property_read_u32(node, "sprd,div-msk", &msk)) {
		div = kzalloc(sizeof(struct clk_divider), GFP_KERNEL);
		if (div == NULL)
			goto iounmap_mux_reg;

		div->reg = of_iomap(node, index);
		if (!div->reg)
			div->reg = mux->reg;

		div->shift = __ffs(msk);
		div->width = __ffs((msk >> div->shift) + 1);
		div_ops = &sprd_divider_ops;
		clk_debug("div %u, %u\n", (u32) div->shift, (u32) div->width);
	}

	flags |= CLK_IGNORE_UNUSED;
	/* TODO: later maybe register as platform_dev */
	clk_debug("composite clock %s\n", clk_name);
	clk = clk_register_composite(NULL, clk_name, parent_names, num_parents,
				&mux->hw, mux_ops, &div->hw, div_ops, NULL,
				NULL, flags);
	if (!IS_ERR(clk)) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		clk_register_clkdev(clk, clk_name, NULL);
		return;
	}

	if (div->reg)
		if (!mux || (div->reg != mux->reg))
			iounmap(div->reg);

	kfree(div);

iounmap_mux_reg:
	if (mux->reg)
		iounmap(mux->reg);

kfree_mux:
	kfree(mux);

kfree_parent_names:
	kfree(parent_names);
}

CLK_OF_DECLARE(composite_clock, "sprd,composite-clock",
		of_sprd_composite_clk_setup);
