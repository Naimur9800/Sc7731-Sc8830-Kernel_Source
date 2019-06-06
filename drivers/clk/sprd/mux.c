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

void __init of_sprd_mux_clk_setup(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	const char **parent_names;
	int   num_parents = 0;
	void __iomem *reg;
	u32  msk = 0;
	u8   shift = 0;
	u8   width = 0;
	int i = 0;

	of_property_read_string(node, "clock-output-names", &clk_name);

	if (of_property_read_u32(node, "sprd,mux-msk", &msk)) {
		pr_err("clk: sprd: get %s's mux-msk failed!\n", clk_name);
		return;
	}
	shift = __ffs(msk);
	width = __ffs((msk >> shift) + 1);

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

	reg = of_iomap(node, 0);
	if (!reg) {
		pr_err("clk: sprd: mux-clock %s remap register failed!\n",
			clk_name);
		goto kfree_parent_names;
	}

	clk_debug("mux clock %s\n", clk_name);
	clk = clk_register_mux(NULL, clk_name, parent_names, num_parents,
			       CLK_SET_RATE_NO_REPARENT | CLK_GET_RATE_NOCACHE,
			       reg, shift, width, 0, NULL);
	if (clk) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		clk_register_clkdev(clk, clk_name, NULL);
		return;
	}

	iounmap(reg);

kfree_parent_names:
	kfree(parent_names);
}

CLK_OF_DECLARE(muxed_clock, "sprd,muxed-clock", of_sprd_mux_clk_setup);
