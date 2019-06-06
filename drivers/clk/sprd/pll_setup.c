/*
 * Spreatrum Clock support
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

#include "pll_cfg.h"

struct sprd_pll_config *g_sprd_pll_config;

static struct sprd_pll_config *get_pll_config(struct clk_hw *hw,
				struct sprd_pll_config *ppll_config)
{
	for (; ppll_config->name != NULL; ppll_config++)
		if (!strcmp(ppll_config->name, __clk_get_name(hw->clk)))
			break;

	if (ppll_config->name == NULL)
		return NULL;

	return ppll_config;
}

static unsigned long sprd_sc9838_adjustable_pll_recalc_rate(struct clk_hw *hw,
						    unsigned long parent_rate)
{
	struct sprd_pll_hw *pll = to_sprd_pll_hw(hw);

	return sprd_adjustable_pll_recalc_rate(pll, g_sprd_pll_config,
					       parent_rate);
}

static unsigned long sprd_sc9850_adjustable_pll_recalc_rate(struct clk_hw *hw,
						    unsigned long parent_rate)
{
	struct sprd_pll_config *ppll_config;
	struct sprd_pll_hw *pll = to_sprd_pll_hw(hw);

	ppll_config = get_pll_config(hw, g_sprd_pll_config);

	return sprd_adjustable_pll_recalc_rate(pll, ppll_config,
					       parent_rate);
}

static int sprd_sc9838_adjustable_pll_set_rate(struct clk_hw *hw,
						unsigned long rate,
						unsigned long parent_rate)
{
	struct sprd_pll_hw *pll = to_sprd_pll_hw(hw);

	return sprd_adjustable_pll_set_rate(pll, g_sprd_pll_config, rate,
					    parent_rate);
}

static int sprd_sc9850_adjustable_pll_set_rate(struct clk_hw *hw,
						unsigned long rate,
						unsigned long parent_rate)
{
	struct sprd_pll_config *ppll_config;
	struct sprd_pll_hw *pll = to_sprd_pll_hw(hw);

	ppll_config = get_pll_config(hw, g_sprd_pll_config);

	return sprd_adjustable_pll_set_rate(pll, ppll_config, rate,
					    parent_rate);
}

const struct clk_ops sprd_clk_sc9838_adjustable_pll_ops = {
	.round_rate = sprd_adjustable_pll_round_rate,
	.set_rate = sprd_sc9838_adjustable_pll_set_rate,
	.recalc_rate = sprd_sc9838_adjustable_pll_recalc_rate,
};

const struct clk_ops sprd_clk_sc9850_adjustable_pll_ops = {
	.round_rate = sprd_adjustable_pll_round_rate,
	.set_rate = sprd_sc9850_adjustable_pll_set_rate,
	.recalc_rate = sprd_sc9850_adjustable_pll_recalc_rate,
};

static void __init of_sc9838_adjustable_pll_clk_setup(struct device_node *node)
{
	g_sprd_pll_config = sc9838_pll_config;
	of_sprd_pll_clk_setup(node, &sprd_clk_sc9838_adjustable_pll_ops);
}

static void __init of_sc9850_adjustable_pll_clk_setup(struct device_node *node)
{
	g_sprd_pll_config = sc9850_pll_config;
	of_sprd_pll_clk_setup(node, &sprd_clk_sc9850_adjustable_pll_ops);
}

static void __init of_sc9861_adjustable_pll_clk_setup(struct device_node *node)
{
	g_sprd_pll_config = sc9861_pll_config;
	of_sprd_pll_clk_setup(node, &sprd_clk_sc9850_adjustable_pll_ops);
}

static void __init of_sc9833_adjustable_pll_clk_setup(struct device_node *node)
{
	g_sprd_pll_config = sc9833_pll_config;
	of_sprd_pll_clk_setup(node, &sprd_clk_sc9850_adjustable_pll_ops);
}

static void __init of_sc9853_adjustable_pll_clk_setup(struct device_node *node)
{
	g_sprd_pll_config = sc9853_pll_config;
	of_sprd_pll_clk_setup(node, &sprd_clk_sc9850_adjustable_pll_ops);
}

static void __init of_sc9850kh_adjustable_pll_clk_setup(struct device_node *node)
{
	g_sprd_pll_config = sc9850kh_pll_config;
	of_sprd_pll_clk_setup(node, &sprd_clk_sc9850_adjustable_pll_ops);
}

static void __init of_sc9835_adjustable_pll_clk_setup(struct device_node *node)
{
	g_sprd_pll_config = sc9835_pll_config;
	of_sprd_pll_clk_setup(node, &sprd_clk_sc9850_adjustable_pll_ops);
}

static void __init of_sc7731e_adjustable_pll_clk_setup(struct device_node *node)
{
	g_sprd_pll_config = sc7731e_pll_config;
	of_sprd_pll_clk_setup(node, &sprd_clk_sc9850_adjustable_pll_ops);
}

static void __init of_sc9863a_adjustable_pll_clk_setup(struct device_node *node)
{
	g_sprd_pll_config = sc9863a_pll_config;
	of_sprd_pll_clk_setup(node, &sprd_clk_sc9850_adjustable_pll_ops);
}

CLK_OF_DECLARE(sc9838_adjustable_pll_clock, "sprd,sc9838-adjustable-pll-clock",
		of_sc9838_adjustable_pll_clk_setup);
CLK_OF_DECLARE(sc9850_adjustable_pll_clock, "sprd,sc9850-adjustable-pll-clock",
		of_sc9850_adjustable_pll_clk_setup);
CLK_OF_DECLARE(sc9861_adjustable_pll_clock, "sprd,sc9861-adjustable-pll-clock",
		of_sc9861_adjustable_pll_clk_setup);
CLK_OF_DECLARE(sc9833_adjustable_pll_clock, "sprd,sc9833-adjustable-pll-clock",
		of_sc9833_adjustable_pll_clk_setup);
CLK_OF_DECLARE(sc9853_adjustable_pll_clock, "sprd,sc9853-adjustable-pll-clock",
		of_sc9853_adjustable_pll_clk_setup);
CLK_OF_DECLARE(sc9850kh_adjustable_pll_clock, "sprd,sc9850kh-adjustable-pll-clock",
		of_sc9850kh_adjustable_pll_clk_setup);
CLK_OF_DECLARE(sc9835_adjustable_pll_clock, "sprd,sc9835-adjustable-pll-clock",
		of_sc9835_adjustable_pll_clk_setup);
CLK_OF_DECLARE(sc7731e_adjustable_pll_clock,
		"sprd,sc7731e-adjustable-pll-clock",
		of_sc7731e_adjustable_pll_clk_setup);
CLK_OF_DECLARE(sc9863a_adjustable_pll_clock,
		"sprd,sc9863a-adjustable-pll-clock",
		of_sc9863a_adjustable_pll_clk_setup);
