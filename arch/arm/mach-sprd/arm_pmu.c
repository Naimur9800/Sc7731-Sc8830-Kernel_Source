/* arch/arm/mach-sprd/arm_pmu.c
 *
 *  Copyright (C) 2017 SPREADTRUM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>



#define AP_INTC2_PMU_OFFSET		0xC
#define AP_INTC2_PMU_CORE0_BIT		BIT(28)
#define AP_INTC2_PMU_CORE1_BIT		BIT(29)
#define AP_INTC2_PMU_CORE2_BIT		BIT(30)
#define AP_INTC2_PMU_CORE3_BIT		BIT(31)


static bool is_chip_true(void)
{
	struct device_node *np = NULL;

	np = of_find_compatible_node(NULL, NULL, "sprd,sc9850");
	if (!np) {
		pr_info("[%s]chip does not need to disable/enable pmu irq\n",
			__func__);
		return false;
	}

	return true;
}


static int get_pmu_irq(unsigned int cpu)
{
	struct device_node *np = NULL;

	np = of_find_compatible_node(NULL, NULL, "arm,cortex-a7-pmu");
	if (!np) {
		pr_err("[%s]can not find arm pmu node\n", __func__);
		return -ENODEV;
	}

	return irq_of_parse_and_map(np, cpu);
}


void sprd_pmu_enable_irq(unsigned int cpu)
{
	int irq;

	if (!is_chip_true())
		return;

	irq = get_pmu_irq(cpu);
	if (irq > 0) {
		pr_info("enable cpu%d pmu irq\n", cpu);
		enable_irq(irq);
	}
}

void sprd_pmu_disable_irq(unsigned int cpu)
{
	struct device_node *deepsleep_np = NULL;
	struct regmap *deepsleep_syscon_ap_intc2 = NULL;
	int irq;

	if (!is_chip_true())
		return;

	irq = get_pmu_irq(cpu);
	if (irq > 0) {
		pr_info("disable cpu%d pmu irq\n", cpu);
		disable_irq(irq);
	} else {
		pr_err("[%s]can not find proper pmu irq of cpu%d\n", __func__,
		       cpu);
		return;
	}

	deepsleep_np = of_find_node_by_name(NULL, "deep-sleep");
	if (!deepsleep_np) {
		pr_err("[%s]failed to find deepsleep_np\n", __func__);
		return;
	}

	deepsleep_syscon_ap_intc2 = syscon_regmap_lookup_by_phandle(
			deepsleep_np,
			"sprd,sys-ap-intc2");
	if ((!deepsleep_syscon_ap_intc2) || IS_ERR(deepsleep_syscon_ap_intc2)) {
		pr_err("[%s]failed to find deepsleep_syscon_ap_intc2\n",
			 __func__);
		return;
	}

	switch (cpu) {
	case 0:
		regmap_update_bits(deepsleep_syscon_ap_intc2,
				   AP_INTC2_PMU_OFFSET,
				   AP_INTC2_PMU_CORE0_BIT,
				   AP_INTC2_PMU_CORE0_BIT);
		break;

	case 1:
		regmap_update_bits(deepsleep_syscon_ap_intc2,
				   AP_INTC2_PMU_OFFSET,
				   AP_INTC2_PMU_CORE1_BIT,
				   AP_INTC2_PMU_CORE1_BIT);
		break;

	case 2:
		regmap_update_bits(deepsleep_syscon_ap_intc2,
				   AP_INTC2_PMU_OFFSET,
				   AP_INTC2_PMU_CORE2_BIT,
				   AP_INTC2_PMU_CORE2_BIT);
		break;

	case 3:
		regmap_update_bits(deepsleep_syscon_ap_intc2,
				   AP_INTC2_PMU_OFFSET,
				   AP_INTC2_PMU_CORE3_BIT,
				   AP_INTC2_PMU_CORE3_BIT);
		break;

	default:
		pr_err("[%s]invalid cpu number cpu%d\n", __func__, cpu);
		break;
	}
}

