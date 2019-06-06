/*
 * Spreadtrum Clock support
 *
 * Copyright (C) 2015 spreadtrum, Inc.
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
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/spinlock.h>

#include "pll.h"

DEFINE_SPINLOCK(gate_lock);

#define to_clk_gate(_hw) container_of(_hw, struct clk_gate, hw)

static void sprd_clk_sc_gate_endisable(struct clk_hw *hw, int enable,
					unsigned int offset)
{
	struct clk_gate *gate = to_clk_gate(hw);
	int set = gate->flags & CLK_GATE_SET_TO_DISABLE ? 1 : 0;
	unsigned long uninitialized_var(flags);
	int __maybe_unused val;

	set ^= enable;

	if (gate->lock)
		spin_lock_irqsave(gate->lock, flags);
	else
		__acquire(gate->lock);

	if (set)
		clk_writel(BIT(gate->bit_idx), gate->reg + offset);
	else
		clk_writel(BIT(gate->bit_idx), gate->reg + 2 * offset);

	if (gate->lock)
		spin_unlock_irqrestore(gate->lock, flags);
	else
		__release(gate->lock);
}

static int sprd_clk_sc100_gate_enable(struct clk_hw *hw)
{
	sprd_clk_sc_gate_endisable(hw, 1, 0x100);

	return 0;
}

static void sprd_clk_sc100_gate_disable(struct clk_hw *hw)
{
	sprd_clk_sc_gate_endisable(hw, 0, 0x100);
}

static int sprd_clk_sc1000_gate_enable(struct clk_hw *hw)
{
	sprd_clk_sc_gate_endisable(hw, 1, 0x1000);

	return 0;
}

/* high performance plls' prepare & enable interface */
static int sprd_clk_sc1000_hppll_gate_enable(struct clk_hw *hw)
{
	sprd_clk_sc_gate_endisable(hw, 1, 0x1000);
	udelay(SPRD_DELAY_240);

	return 0;
}

/* low performance plls' prepare & enable interface */
static int sprd_clk_sc1000_lppll_gate_enable(struct clk_hw *hw)
{
	sprd_clk_sc_gate_endisable(hw, 1, 0x1000);
	udelay(SPRD_DELAY_1000);

	return 0;
}

static void sprd_clk_sc1000_gate_disable(struct clk_hw *hw)
{
	sprd_clk_sc_gate_endisable(hw, 0, 0x1000);
}

static int sprd_clk_gate_is_enabled(struct clk_hw *hw)
{
	u32 reg;
	struct clk_gate *gate = to_clk_gate(hw);

	reg = clk_readl(gate->reg);

	/* if a set bit disables this clk, flip it before masking */
	if (gate->flags & CLK_GATE_SET_TO_DISABLE)
		reg ^= BIT(gate->bit_idx);

	reg &= BIT(gate->bit_idx);

	return reg ? 1 : 0;
}

const struct clk_ops sprd_clk_sc100_gate_ops = {
	.enable = sprd_clk_sc100_gate_enable,
	.disable = sprd_clk_sc100_gate_disable,
	.is_enabled = sprd_clk_gate_is_enabled,
};

const struct clk_ops sprd_clk_sc1000_gate_ops = {
	.enable = sprd_clk_sc1000_gate_enable,
	.disable = sprd_clk_sc1000_gate_disable,
	.is_enabled = sprd_clk_gate_is_enabled,
};

/* high performance plls' ops */
const struct clk_ops sprd_clk_sc1000_hppll_gate_ops = {
	.prepare = sprd_clk_sc1000_hppll_gate_enable,
	.unprepare = sprd_clk_sc1000_gate_disable,
};

/* low performance plls' ops */
const struct clk_ops sprd_clk_sc1000_lppll_gate_ops = {
	.prepare = sprd_clk_sc1000_lppll_gate_enable,
	.unprepare = sprd_clk_sc1000_gate_disable,
};

static struct clk *sprd_clk_register_gate(struct device *dev,
		const char *name, const char *parent_name,
		unsigned long flags, void __iomem *reg,
		u8 bit_idx, u8 clk_gate_flags,
		spinlock_t *lock, const struct clk_ops *ops)
{
	struct clk_gate *gate;
	struct clk *clk;
	struct clk_init_data init;

	/* allocate the gate */
	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = ops;
	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	/* struct clk_gate assignments */
	gate->reg = reg;
	gate->bit_idx = bit_idx;
	gate->flags = clk_gate_flags;
	gate->lock = lock;
	gate->hw.init = &init;

	clk = clk_register(dev, &gate->hw);

	if (IS_ERR(clk))
		kfree(gate);

	return clk;
}

static void __init of_sprd_gates_clk_setup_with_ops(struct device_node *node,
						const struct clk_ops *ops)
{
	const char *clk_name = NULL;
	void __iomem *reg;
	const char *parent_name;
	unsigned long flags = CLK_IGNORE_UNUSED;
	u8 clk_total_num = 0;
	u8 gate_flags = 0;
	u32 msk, index;
	int i = 0, j = 0, count = 0;
	struct resource res;
	struct clk_onecell_data *clk_data;

	/* TODO: need to use of_iomap */
	if (of_address_to_resource(node, 0, &res)) {
		pr_err("clk: sprd:[%s] gates clock setup failed!\n",
			node->full_name);
		return;
	}

	/* TODO: need to change the reg add in dt */
	if (res.start & 1) {
		res.start &= ~3;
		gate_flags |= CLK_GATE_SET_TO_DISABLE;
	}
	reg = ioremap(res.start, resource_size(&res));
	if (!reg) {
		pr_err("clk: sprd:[%s] gates clock remap register failed!\n",
			node->full_name);
		return;
	}

	parent_name = of_clk_get_parent_name(node, 0);

	if (of_property_read_u32(node, "sprd,gates-msk", &msk) < 0) {
		pr_err("sprd: clk:[%s] gates clock no gates_msk property!\n",
			node->full_name);
		goto iounmap_reg;
	}

	index = msk;
	for (i = 0; i < BITS_PER_BYTE * sizeof(msk); i++) {
		if (index & 0x1)
			count++;
		index >>= 1;
	}

	clk_total_num = fls(msk);

	clk_data = kmalloc(sizeof(struct clk_onecell_data), GFP_KERNEL);
	if (!clk_data)
		goto iounmap_reg;

	clk_data->clks = kcalloc(clk_total_num, sizeof(struct clk *),
			GFP_KERNEL);
	if (!clk_data->clks)
		goto kfree_clk_data;

	for (i = 0; i < clk_total_num; i++) {
		if (0 == (msk & (1U << i)))
			continue;
		of_property_read_string_index(node, "clock-output-names",
					j, &clk_name);

		/* Fix me: MM domain and GPU domain need power */
		pr_debug("sprd: clk:%s, clk:%s,msk:0x%x i:%d j:%d num:%d\n",
			__func__, clk_name, msk, i, j, clk_total_num);
		clk_data->clks[i] = sprd_clk_register_gate(NULL, clk_name,
				parent_name, flags, reg, i,
				gate_flags, &gate_lock, ops);
		WARN_ON(IS_ERR(clk_data->clks[i]));
		clk_register_clkdev(clk_data->clks[i], clk_name, NULL);

		j++;
	}

	if (j != count) {
		pr_info("sprd: clk:[%s] msk:%x count:%d,j:%d can't match! num:%d\n",
		node->full_name, msk, count, j, clk_total_num);
	}

	clk_data->clk_num = clk_total_num;
	of_clk_add_provider(node, of_clk_src_onecell_get, clk_data);
	return;

kfree_clk_data:
	kfree(clk_data);

iounmap_reg:
	iounmap(reg);
}

static void __init of_sprd_sc100_gates_clk_setup(struct device_node *node)
{
	of_sprd_gates_clk_setup_with_ops(node, &sprd_clk_sc100_gate_ops);
}

static void __init of_sprd_sc1000_gates_clk_setup(struct device_node *node)
{
	of_sprd_gates_clk_setup_with_ops(node, &sprd_clk_sc1000_gate_ops);
}

static void __init
of_sprd_sc1000_hppll_gates_clk_setup(struct device_node *node)
{
	of_sprd_gates_clk_setup_with_ops(node, &sprd_clk_sc1000_hppll_gate_ops);
}

static void __init
of_sprd_sc1000_lppll_gates_clk_setup(struct device_node *node)
{
	of_sprd_gates_clk_setup_with_ops(node, &sprd_clk_sc1000_lppll_gate_ops);
}

static void __init of_sprd_gates_clk_setup(struct device_node *node)
{
	of_sprd_gates_clk_setup_with_ops(node, &clk_gate_ops);
}

CLK_OF_DECLARE(sc100_gates_clock, "sprd,sc100-gates-clock",
				of_sprd_sc100_gates_clk_setup);
CLK_OF_DECLARE(sc1000_gates_clock, "sprd,sc1000-gates-clock",
				of_sprd_sc1000_gates_clk_setup);
CLK_OF_DECLARE(sc1000_hppll_gates_clock, "sprd,sc1000-hppll-gates-clock",
				of_sprd_sc1000_hppll_gates_clk_setup);
CLK_OF_DECLARE(sc1000_lppll_gates_clock, "sprd,sc1000-lppll-gates-clock",
				of_sprd_sc1000_lppll_gates_clk_setup);
CLK_OF_DECLARE(gates_clock, "sprd,gates-clock", of_sprd_gates_clk_setup);
