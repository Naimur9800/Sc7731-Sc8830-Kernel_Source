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

#ifndef __SPRD_PLL_H__
#define __SPRD_PLL_H__

#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define SPRD_PLL_MAX_RATE		(0xFFFFFFFF)
#define SPRD_PLL_MAX_REGNUM		(3)
#define SPRD_DELAY_240			(240)
#define SPRD_DELAY_1000			(1000)

struct sprd_ibias_table {
	unsigned long rate;
	u8 ibias;
};

struct lock_done_struct {
	u32 lock_done;
	u8  rindex;
};

struct div_s_struct {
	u32 div_s;
	u8  rindex;
};

struct mod_en_struct {
	u32 mod_en;
	u8  rindex;
};

struct sdm_en_struct {
	u32 sdm_en;
	u8  rindex;
};

struct refin_msk_struct {
	u32 refin_msk;
	u8  rindex;
};

struct ibias_msk_struct {
	u32 ibias_msk;
	u8  rindex;
};

struct pll_n_msk_struct {
	u32 pll_n_msk;
	u8  rindex;
};

struct nint_msk_struct {
	u32 nint_msk;
	u8  rindex;
};

struct kint_msk_struct {
	u32 kint_msk;
	u8  rindex;
};

struct fvco_threshold_struct {
	unsigned long rate;
	int flag;
};

struct prediv_msk_struct {
	u32 prediv_msk;
	u8  rindex;
	struct fvco_threshold_struct *fvco_threshold;
};

struct postdiv_msk_struct {
	u32 postdiv_msk;
	u8  rindex;
	struct fvco_threshold_struct *fvco_threshold;
};

struct sprd_pll_config {
	const char *name;
	struct lock_done_struct lock_done;
	struct div_s_struct  div_s;
	struct mod_en_struct mod_en;
	struct sdm_en_struct sdm_en;
	struct refin_msk_struct refin_msk;
	struct ibias_msk_struct ibias_msk;
	struct pll_n_msk_struct pll_n_msk;
	struct nint_msk_struct nint_msk;
	struct kint_msk_struct kint_msk;
	struct prediv_msk_struct prediv_msk;
	struct postdiv_msk_struct postdiv_msk;
	struct sprd_ibias_table *itable;
	u32 udelay;
};

struct sprd_pll_hw {
	struct clk_hw hw;
	void __iomem *reg[SPRD_PLL_MAX_REGNUM];
	int reg_num;
};

#define to_sprd_pll_hw(_hw) container_of(_hw, struct sprd_pll_hw, hw)

extern struct sprd_pll_config *g_sprd_pll_config;

struct sprd_pll_config *get_sprd_pll_config(struct clk_hw *hw,
				struct sprd_pll_config *ppll_config);
int sprd_pll_clk_prepare(struct clk_hw *hw);
void sprd_clk_unprepare(struct clk_hw *hw);
long sprd_adjustable_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *prate);
unsigned long sprd_adjustable_pll_recalc_rate(struct sprd_pll_hw *pll,
					struct sprd_pll_config *ppll_config,
					unsigned long parent_rate);
int sprd_adjustable_pll_set_rate(struct sprd_pll_hw *pll,
				struct sprd_pll_config *ppll_config,
				unsigned long rate,
				unsigned long parent_rate);
void __init of_sprd_pll_clk_setup(struct device_node *node,
				const struct clk_ops *clk_ops);
#endif
