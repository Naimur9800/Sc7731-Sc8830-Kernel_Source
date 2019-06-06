/*
 * Spreadtrum ARM CPU idle driver.
 *
 * Copyright (C) 2017 Spreadtrum Ltd.
 * Author: Jingchao Ye <jingchao.ye@spreadtrum.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/cpuidle.h>
#include <asm/cpuidle.h>
#include <asm/suspend.h>
#include <linux/psci.h>


static DEFINE_PER_CPU_READ_MOSTLY(u32 *, psci_power_state);

static int psci_suspend_finisher(unsigned long index)
{
	u32 *state = __this_cpu_read(psci_power_state);

	return psci_ops.cpu_suspend(state[index - 1],
				    virt_to_phys(cpu_resume));
}

static int __maybe_unused sprd_cpuidle_suspend(int cpu, unsigned long index)
{
	int ret;
	u32 *state = __this_cpu_read(psci_power_state);
	/*
	 * idle state index 0 corresponds to wfi, should never be called
	 * from the cpu_suspend operations
	 */
	if (WARN_ON_ONCE(!index))
		return -EINVAL;

	if (!psci_power_state_loses_context(state[index - 1]))
		ret = psci_ops.cpu_suspend(state[index - 1], 0);
	else
		ret = cpu_suspend(index, psci_suspend_finisher);

	return ret;
}

static int __maybe_unused sprd_cpuidle_init(struct device_node *cpu_node, int cpu)
{
	int i, ret, count = 0;
	u32 *psci_states;
	struct device_node *state_node;


	/*
	 * If the PSCI cpu_suspend function hook has not been initialized
	 * idle states must not be enabled, so bail out
	 */
	if (!psci_ops.cpu_suspend)
		return -EOPNOTSUPP;

	/* Count idle states */
	while ((state_node = of_parse_phandle(cpu_node, "cpu-idle-states",
					      count))) {
		count++;
		of_node_put(state_node);
	}

	if (!count)
		return -ENODEV;

	psci_states = kcalloc(count, sizeof(*psci_states), GFP_KERNEL);
	if (!psci_states)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		u32 state;

		state_node = of_parse_phandle(cpu_node, "cpu-idle-states", i);

		ret = of_property_read_u32(state_node,
					   "arm,psci-suspend-param",
					   &state);
		if (ret) {
			pr_warn(" * %s missing arm,psci-suspend-param property\n",
				state_node->full_name);
			of_node_put(state_node);
			goto free_mem;
		}

		of_node_put(state_node);
		pr_debug("psci-power-state %#x index %d\n", state, i);
		if (!psci_power_state_is_valid(state)) {
			pr_warn("Invalid PSCI power state %#x\n", state);
			ret = -EINVAL;
			goto free_mem;
		}
		psci_states[i] = state;
	}
	/* Idle states parsed correctly, initialize per-cpu pointer */
	per_cpu(psci_power_state, cpu) = psci_states;
	return 0;

free_mem:
	kfree(psci_states);
	return ret;
}

static struct cpuidle_ops sprd_cpuidle_ops __initdata = {
	.suspend = sprd_cpuidle_suspend,
	.init = sprd_cpuidle_init,
};

CPUIDLE_METHOD_OF_DECLARE(sprd_idle, "psci", &sprd_cpuidle_ops);
