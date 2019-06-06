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

#include <linux/delay.h>

#include "pll.h"

static void __pllreg_write(void __iomem *reg, u32 val, u32 msk)
{
	writel_relaxed((readl_relaxed(reg) & ~msk) | val, reg);
}

static unsigned long __pll_get_refin_rate(struct sprd_pll_hw *pll,
			struct sprd_pll_config *ppll_config)
{
	u32 i;
	void __iomem *reg;
	const unsigned long refin[4] = { 2000000, 4000000, 13000000, 26000000 };

	if (ppll_config->refin_msk.refin_msk == 0)
		i = 3;
	else {
		reg = pll->reg[ppll_config->refin_msk.rindex];
		i = (readl_relaxed(reg) & ppll_config->refin_msk.refin_msk)
			>> __ffs(ppll_config->refin_msk.refin_msk);
	}

	return refin[i]/1000000;
}

static u8 __sprd_get_ibias_from_ibias_table(unsigned long rate,
		struct sprd_ibias_table *table)
{
	if (!table)
		return 0;

	for (; table->rate <= SPRD_PLL_MAX_RATE; table++) {
		if (rate <= table->rate)
			break;
	}
	return table->ibias;
}

struct sprd_pll_config *get_sprd_pll_config(struct clk_hw *hw,
		struct sprd_pll_config *ppll_config)
{
	for (; ppll_config->name != NULL; ppll_config++) {
		if (!strcmp(ppll_config->name, __clk_get_name(hw->clk)))
			break;
	}

	if (ppll_config->name == NULL)
		return NULL;

	return ppll_config;
}

static int __wait_pll_ready(struct sprd_pll_config *ppll_config)
{
	udelay(ppll_config->udelay);

	return 0;
}

long sprd_adjustable_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *prate)
{
	/**
	 * FixMe:
	 * It maybe support real round_rate later, but not now!
	 */
	return rate;
}

static inline int sprd_adjustable_pll_check(struct sprd_pll_hw *pll,
					struct sprd_pll_config *ppll_config)
{
	if ((ppll_config->lock_done.rindex >= pll->reg_num) ||
		(ppll_config->div_s.rindex >= pll->reg_num) ||
		(ppll_config->mod_en.rindex >= pll->reg_num) ||
		(ppll_config->sdm_en.rindex >= pll->reg_num) ||
		(ppll_config->refin_msk.rindex >= pll->reg_num) ||
		(ppll_config->ibias_msk.rindex >= pll->reg_num) ||
		(ppll_config->pll_n_msk.rindex >= pll->reg_num) ||
		(ppll_config->nint_msk.rindex >= pll->reg_num) ||
		(ppll_config->kint_msk.rindex >= pll->reg_num) ||
		(ppll_config->prediv_msk.rindex >= pll->reg_num)) {
		pr_err("%s pll:%s exceed max:%d\n", __func__,
			__clk_get_name(pll->hw.clk), pll->reg_num);
		return -EINVAL;
	}

	return 0;
}

unsigned long sprd_adjustable_pll_recalc_rate(struct sprd_pll_hw *pll,
					struct sprd_pll_config *ppll_config,
					unsigned long parent_rate)
{
	unsigned long rate, refin;
	unsigned long kint = 0, nint, cfg[SPRD_PLL_MAX_REGNUM], n;
	int index;

	if (ppll_config == NULL) {
		pr_err("clk: %s cannot get pll %s\n", __func__,
			__clk_get_name(pll->hw.clk));
		return -EINVAL;
	}

	if (sprd_adjustable_pll_check(pll, ppll_config))
		return -EINVAL;

	for (index = 0; index < pll->reg_num; index++)
		cfg[index] = readl_relaxed(pll->reg[index]);

	refin = __pll_get_refin_rate(pll, ppll_config);

	if (ppll_config->prediv_msk.prediv_msk) {
		if (cfg[ppll_config->prediv_msk.rindex] &
		    ppll_config->prediv_msk.prediv_msk)
			refin = refin * 2;
	}

	if (ppll_config->postdiv_msk.postdiv_msk) {
		if (((ppll_config->postdiv_msk.fvco_threshold->flag == 1) &&
		     ((cfg[ppll_config->postdiv_msk.rindex]) &
		       ppll_config->postdiv_msk.postdiv_msk)) ||
		    ((ppll_config->postdiv_msk.fvco_threshold->flag == 0) &&
		     !((cfg[ppll_config->postdiv_msk.rindex]) &
		       ppll_config->postdiv_msk.postdiv_msk))) {
			refin = refin / 2;
		}
	}

	if (!(ppll_config->div_s.div_s & cfg[ppll_config->div_s.rindex])) {
		n = (cfg[ppll_config->pll_n_msk.rindex] &
		     ppll_config->pll_n_msk.pll_n_msk) >>
		    __ffs(ppll_config->pll_n_msk.pll_n_msk);
		rate = refin * n * 1000000;
		pr_debug("sprd: clk: %s refin = %lu, n = %lu, rate = %lu\n",
			  __func__, refin, n, rate);
	} else {
		nint = (cfg[ppll_config->nint_msk.rindex] &
			ppll_config->nint_msk.nint_msk) >>
			__ffs(ppll_config->nint_msk.nint_msk);
		if (cfg[ppll_config->sdm_en.rindex] &
		    ppll_config->sdm_en.sdm_en)
			kint = (cfg[ppll_config->kint_msk.rindex] &
				ppll_config->kint_msk.kint_msk) >>
				__ffs(ppll_config->kint_msk.kint_msk);
	/*
	 * improve the accuracy of clk through sprd_adjustable_pll_recalc_rate.
	 */
#ifdef CONFIG_64BIT
		rate = refin * (nint) * 1000000 +
			DIV_ROUND_CLOSEST(refin * kint * 1000,
				((ppll_config->kint_msk.kint_msk >>
				__ffs(ppll_config->kint_msk.kint_msk)) + 1))
									* 1000;
#else
		index = __ffs((ppll_config->kint_msk.kint_msk >>
				__ffs(ppll_config->kint_msk.kint_msk)) + 1);
		if (index <= 20)
			rate = refin * (nint) * 1000000 +
				DIV_ROUND_CLOSEST(refin * kint * 100,
				((ppll_config->kint_msk.kint_msk >>
				__ffs(ppll_config->kint_msk.kint_msk)) + 1))
								* 10000;
		else
			rate = refin * nint * 1000000 +
				DIV_ROUND_CLOSEST(refin * (kint >>
				(index - 20)) * 100,
				((ppll_config->kint_msk.kint_msk >>
				 (__ffs(ppll_config->kint_msk.kint_msk) +
				(index - 20))) + 1)) * 10000;
#endif

		pr_debug("sprd: clk: %s refin = %lu, kint = %lu, nint =%lu\n",
			__clk_get_name(pll->hw.clk), refin, kint, nint);
	}

	return rate;
}

struct reg_cfg {
	u32 val;
	u32 msk;
};

int sprd_adjustable_pll_set_rate(struct sprd_pll_hw *pll,
				 struct sprd_pll_config *ppll_config,
				 unsigned long rate,
				 unsigned long parent_rate)
{
	u8 ibias;
	unsigned long kint, nint;
	unsigned long refin, old_rate, val, fvco = rate;
	struct reg_cfg cfg[SPRD_PLL_MAX_REGNUM] = {{},};
	int index;
#ifndef CONFIG_64BIT
	int first_one_idx;
#endif
	int ret;

	if (ppll_config == NULL) {
		pr_err("clk: %s cannot get pll %s\n", __func__,
					__clk_get_name(pll->hw.clk));
		return -EINVAL;
	}

	if (sprd_adjustable_pll_check(pll, ppll_config))
		return -EINVAL;

	old_rate = sprd_adjustable_pll_recalc_rate(pll, ppll_config,
						   parent_rate);

	/* calc the pll refin */
	refin = __pll_get_refin_rate(pll, ppll_config);

	if (ppll_config->prediv_msk.prediv_msk) {
		index = ppll_config->prediv_msk.rindex;
		val = readl_relaxed(pll->reg[index]);
		if (val & ppll_config->prediv_msk.prediv_msk)
			refin = refin * 2;
	}

	if (ppll_config->postdiv_msk.postdiv_msk &&
		(ppll_config->postdiv_msk.fvco_threshold->flag == 1)) {
		if (fvco <= ppll_config->postdiv_msk.fvco_threshold->rate) {
			cfg[ppll_config->postdiv_msk.rindex].val |=
				ppll_config->postdiv_msk.postdiv_msk;
			fvco = fvco * 2;
		} else
			cfg[ppll_config->postdiv_msk.rindex].val &=
				~ppll_config->postdiv_msk.postdiv_msk;
	} else if (ppll_config->postdiv_msk.postdiv_msk &&
			(ppll_config->postdiv_msk.fvco_threshold->flag == 0)) {
		if (fvco <= ppll_config->postdiv_msk.fvco_threshold->rate) {
			cfg[ppll_config->postdiv_msk.rindex].val &=
				~ppll_config->postdiv_msk.postdiv_msk;
			fvco = fvco * 2;
		} else
			cfg[ppll_config->postdiv_msk.rindex].val |=
				ppll_config->postdiv_msk.postdiv_msk;
	}
	cfg[ppll_config->postdiv_msk.rindex].msk =
		ppll_config->postdiv_msk.postdiv_msk;

	/* calc the nint and kint */
	nint  = fvco/(refin * 1000000);
#ifdef CONFIG_64BIT
	kint  = DIV_ROUND_CLOSEST(((fvco - refin * nint * 1000000)/10000) *
		 ((ppll_config->kint_msk.kint_msk >>
		   __ffs(ppll_config->kint_msk.kint_msk)) + 1),
		 refin * 100);
#else
	first_one_idx = __ffs((ppll_config->kint_msk.kint_msk >>
			__ffs(ppll_config->kint_msk.kint_msk)) + 1);
	if (first_one_idx <= 20)
		kint  =
		DIV_ROUND_CLOSEST(((fvco - refin * nint * 1000000)/10000) *
		((ppll_config->kint_msk.kint_msk >>
		__ffs(ppll_config->kint_msk.kint_msk)) + 1),
		refin * 100);
	else
		kint =
		DIV_ROUND_CLOSEST(((fvco - refin * nint * 1000000)/10000) *
		((ppll_config->kint_msk.kint_msk >>
		(__ffs(ppll_config->kint_msk.kint_msk) +
		(first_one_idx - 20))) + 1), refin * 100)
		<< (first_one_idx - 20);
#endif
	cfg[ppll_config->nint_msk.rindex].val |=
		((nint << __ffs(ppll_config->nint_msk.nint_msk)) &
		 ppll_config->nint_msk.nint_msk);
	cfg[ppll_config->nint_msk.rindex].msk |= ppll_config->nint_msk.nint_msk;

	/* get kint */
	cfg[ppll_config->div_s.rindex].val |= ppll_config->div_s.div_s;
	cfg[ppll_config->div_s.rindex].msk |= ppll_config->div_s.div_s;

	cfg[ppll_config->sdm_en.rindex].val |= ppll_config->sdm_en.sdm_en;
	cfg[ppll_config->sdm_en.rindex].msk |= ppll_config->sdm_en.sdm_en;

	cfg[ppll_config->kint_msk.rindex].val |=
			((kint << __ffs(ppll_config->kint_msk.kint_msk)) &
			 ppll_config->kint_msk.kint_msk);
	cfg[ppll_config->kint_msk.rindex].msk |=
					ppll_config->kint_msk.kint_msk;

	/* get the ibias */
	ibias = __sprd_get_ibias_from_ibias_table(fvco, ppll_config->itable);
	cfg[ppll_config->ibias_msk.rindex].val |=
		(ibias << __ffs(ppll_config->ibias_msk.ibias_msk) &
		 ppll_config->ibias_msk.ibias_msk);
	cfg[ppll_config->ibias_msk.rindex].msk |=
				ppll_config->ibias_msk.ibias_msk;

	for (index = 0; index < pll->reg_num; index++)
		if (cfg[index].msk)
			__pllreg_write(pll->reg[index],
				cfg[index].val, cfg[index].msk);

	ret = __wait_pll_ready(ppll_config);
	if (ret)
		return ret;

	return 0;
}

static const char *of_clk_get_pll_parent_name(struct device_node *np, int index)
{
	struct of_phandle_args clkspec;
	const char *clk_name = NULL;
	int rc, i = 0, gates_index, temp;
	u32 msk;
	u32 msk_size = BITS_PER_BYTE * sizeof(msk);

	rc = of_parse_phandle_with_args(np, "clocks", "#clock-cells", index,
					&clkspec);
	if (rc)
		return NULL;

	gates_index = clkspec.args_count ? clkspec.args[0] : 0;
	if (of_property_read_u32(clkspec.np, "sprd,gates-msk", &msk) < 0) {
		pr_debug("sprd: clk:[%s] parent clock isn't a gate clock!\n",
			np->full_name);
		goto get_output_name;
	}

	temp =  find_first_bit((const unsigned long *)&msk, msk_size);
	for (i = 0; i < msk_size; i++) {
		if (temp == gates_index)
			break;
		temp = find_next_bit((const unsigned long *)&msk, msk_size,
				     temp + 1);
	}
	if (msk_size == i) {
		pr_err("sprd: clk:[%s] gates_index:%d cann't match!\n",
			np->full_name, gates_index);
		goto out;
	}

get_output_name:
	if (of_property_read_string_index(clkspec.np, "clock-output-names",
					i, &clk_name) < 0) {
		pr_err("sprd: clk:[%s] gates cann't find clock names!\n",
			np->full_name);
		goto out;
	}

out:
	pr_debug("sprd: clk:[%s] parent_names=%s gates_index:%d i:%d msk:%x\n",
		np->full_name, clk_name, gates_index, i, msk);
	of_node_put(clkspec.np);
	return clk_name;
}

void __init of_sprd_pll_clk_setup(struct device_node *node,
				const struct clk_ops *clk_ops)
{
	struct clk *clk = NULL;
	const char *parent_names[1];
	struct sprd_pll_hw *pll;
	int reg_num, index;
	struct clk_init_data init = {
		.ops = clk_ops,
		.flags = CLK_IGNORE_UNUSED,
		.num_parents = 1,
	};

	parent_names[0] = of_clk_get_pll_parent_name(node, 0);
	if (!parent_names[0]) {
		pr_err("%s cann't get parent_names\n", __func__);
		return;
	}
	init.parent_names = parent_names;

	of_property_read_string(node, "clock-output-names",
				(const char **)&init.name);

	pll = kzalloc(sizeof(struct sprd_pll_hw), GFP_KERNEL);
	if (!pll) {
		pr_err("%s cann't alloc\n", __func__);
		return;
	}

	reg_num = of_property_count_u32_elems(node, "reg");
	reg_num = reg_num / (of_n_addr_cells(node) + of_n_size_cells(node));
	if (reg_num > SPRD_PLL_MAX_REGNUM) {
		pr_err("%s reg_num:%d exceed max number\n", __func__, reg_num);
		goto kfree_pll;
	}
	pll->reg_num = reg_num;

	for (index = 0; index < reg_num; index++) {
		pll->reg[index] = of_iomap(node, index);
		if (!pll->reg[index])
			goto kfree_pll;
	}

	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (!IS_ERR(clk)) {
		clk_register_clkdev(clk, init.name, 0);
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		pr_info("%s clk:%s parent:%s reg_num:%d setup ok\n", __func__,
				__clk_get_name(clk), parent_names[0],
				reg_num);
		return;
	}

kfree_pll:
	kfree(pll);
}

