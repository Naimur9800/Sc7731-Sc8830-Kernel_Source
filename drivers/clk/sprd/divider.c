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

void __init of_sprd_divider_clk_setup(struct device_node *node)
{
	struct clk *clk, *pclk;
	struct clk_divider *clk_div;
	struct clk_composite *clk_composite;
	const char *clk_name = node->name;
	const char *parent;
	void __iomem *reg;
	u32 msk = 0;
	u8 shift = 0;
	u8 width = 0;

	of_property_read_string(node, "clock-output-names", &clk_name);
	parent = of_clk_get_parent_name(node, 0);
	if (of_property_read_bool(node, "reg"))
		reg = of_iomap(node, 0);
	else {
		pclk = __clk_lookup(parent);
		if (!pclk) {
			pr_err("clk: sprd: %s has no reg and parent!\n",
				clk_name);
			return;
		}

		clk_composite = container_of(__clk_get_hw(pclk),
					     struct clk_composite, hw);
		clk_div = container_of(clk_composite->rate_hw,
				       struct clk_divider, hw);
		reg = clk_div->reg;
	}
	if (!reg) {
		pr_err("clk: sprd: clock %s remap register failed!\n",
			clk_name);
		return;
	}

	if (of_property_read_u32(node, "sprd,div-msk", &msk)) {
		pr_err("clk: sprd: get %s's div-msk failed!\n", clk_name);
		goto iounmap_reg;
	}
	shift = __ffs(msk);
	width = __ffs((msk >> shift) + 1);

	clk = clk_register_divider(NULL, clk_name, parent,
			0, reg, shift, width, 0, NULL);
	if (!IS_ERR(clk)) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		clk_register_clkdev(clk, clk_name, NULL);
		return;
	}

iounmap_reg:
	iounmap(reg);

	pr_err("clk: sprd: register divider clock %s failed!\n", clk_name);
}

CLK_OF_DECLARE(divider_clock, "sprd,divider-clock", of_sprd_divider_clk_setup);
