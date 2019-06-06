/*
 * Spreadtrum Generic power domain support.
 *
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#ifdef pr_fmt
#undef pr_fmt
#define pr_fmt(__fmt) "[disp-pm-domain][adf][%20s] "__fmt, __func__
#endif

struct disp_pm_domain {
	struct generic_pm_domain pd;
	struct regmap *pmu_apb;
};

static int disp_power_on(struct generic_pm_domain *domain)
{
	struct disp_pm_domain *pd;

	pd = container_of(domain, struct disp_pm_domain, pd);

	regmap_update_bits(pd->pmu_apb,
		REG_PMU_APB_PD_DISP_CFG,
		BIT_PMU_APB_PD_DISP_FORCE_SHUTDOWN,
		(unsigned int)~BIT_PMU_APB_PD_DISP_FORCE_SHUTDOWN);

	mdelay(10);

	pr_info("disp power domain on\n");
	return 0;
}

static int disp_power_off(struct generic_pm_domain *domain)
{
	struct disp_pm_domain *pd;

	pd = container_of(domain, struct disp_pm_domain, pd);

	mdelay(10);

	regmap_update_bits(pd->pmu_apb,
		REG_PMU_APB_PD_DISP_CFG,
		BIT_PMU_APB_PD_DISP_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_DISP_FORCE_SHUTDOWN);

	pr_info("disp power domain off\n");
	return 0;
}

static __init int disp_pm_domain_init(void)
{
	struct disp_pm_domain *pd;
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "sprd,sharkl3-disp-domain");
	if  (!np) {
		pr_err("Error: sprd,sharkl3-disp-domain not found\n");
		return -ENODEV;
	}

	pd = kzalloc(sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		of_node_put(np);
		return -ENOMEM;
	}

	pd->pd.name = kstrdup(np->name, GFP_KERNEL);
	pd->pd.power_off = disp_power_off;
	pd->pd.power_on = disp_power_on;

	pd->pmu_apb = syscon_regmap_lookup_by_compatible("sprd,sys-pmu-apb");
	if (IS_ERR(pd->pmu_apb)) {
		kfree(pd);
		return PTR_ERR(pd->pmu_apb);
	}

	pm_genpd_init(&pd->pd, NULL, true);
	of_genpd_add_provider_simple(np, &pd->pd);

	pr_info("display power domain init ok!\n");

	return 0;
}

core_initcall(disp_pm_domain_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leon.he@spreadtrum.com");
MODULE_DESCRIPTION("sprd sharkl3 display pm generic domain");
