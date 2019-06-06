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
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "clk-sprd.h"

struct sprd_clock_func {
	void (*exit)(void);
};

static inline unsigned int sprd_reg_read(struct regmap *map, u32 offset)
{
	unsigned int val;

	regmap_read(map, offset, &val);
	return val;
}

static void __init sprd_clock_exit(void)
{
	struct device_node *np;
	struct regmap *aon_apb_gpr, *pmu_apb_gpr;
	struct clk	 *clk;
	int i, num;
	u32 offset, msk, value;

	np = of_find_node_by_name(NULL, "clk-default");
	if (!np) {
		pr_err("%s:failed to find clk-default node\n", __func__);
		return;
	}

	pmu_apb_gpr = syscon_regmap_lookup_by_phandle(np,
				"sprd,syscon-pmu-apb");
	if (IS_ERR(pmu_apb_gpr)) {
		pr_err("%s:failed to find pmu_apb_gpr\n", __func__);
		return;
	}

	aon_apb_gpr = syscon_regmap_lookup_by_phandle(np,
				"sprd,syscon-aon-apb");
	if (IS_ERR(aon_apb_gpr)) {
		pr_err("%s:failed to find aon_apb_gpr\n", __func__);
		return;
	}

	num = of_clk_get_parent_count(np);
	if (num < 0)
		return;
	for (i = 0; i < num; i++) {
		clk = of_clk_get(np, i);
		clk_prepare_enable(clk);
		clk_disable_unprepare(clk);
	}
	clk_debug("eb0=0x%x,eb1=0x%x", sprd_reg_read(aon_apb_gpr, 0x0),
		sprd_reg_read(aon_apb_gpr, 0x4));

	num = of_property_count_elems_of_size(np, "pmu-pwd-list",
					3 * sizeof(u32));
	if (num < 0)
		return;
	for (i = 0; i < num; i++) {
		of_property_read_u32_index(np, "pmu-pwd-list", 3 * i,
					   &offset);
		of_property_read_u32_index(np, "pmu-pwd-list", 3 * i + 1,
					   &msk);
		of_property_read_u32_index(np, "pmu-pwd-list", 3 * i + 2,
					   &value);
		regmap_update_bits(pmu_apb_gpr, offset, msk, value);
		clk_debug("reg:0x%x,offset:0x%x,msk:0x%x,value:0x%x\n",
			sprd_reg_read(pmu_apb_gpr, offset),
			offset, msk, value);
	}
}

static struct sprd_clock_func sprd_clock_func __initdata = {
	.exit = sprd_clock_exit,
};

static struct of_device_id sprd_clock_dt_ids[] __initdata = {
	{ .compatible = "sprd,clk-default", .data = &sprd_clock_func },
};

int __init sprd_clock_post_init(void)
{
	int i;
	struct device_node *np;
	struct sprd_clock_func *pdata = NULL;

	for (i = 0; i < ARRAY_SIZE(sprd_clock_dt_ids); i++) {
		np = of_find_compatible_node(NULL, NULL,
			sprd_clock_dt_ids[i].compatible);
		if (np) {
			pdata = (struct sprd_clock_func *)
				sprd_clock_dt_ids[i].data;
			break;
		}
	}

	if (pdata && pdata->exit)
		pdata->exit();

	return 0;
}
arch_initcall_sync(sprd_clock_post_init);
