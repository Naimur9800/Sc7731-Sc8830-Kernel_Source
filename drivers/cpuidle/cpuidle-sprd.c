/*
 * Spreadtrum ARM CPU idle driver.
 *
 * Copyright (C) 2014 Spreadtrum Ltd.
 * Author: Icy Zhu <icy.zhu@spreadtrum.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpuidle.h>
#include <linux/cpumask.h>
#include <linux/cpu_pm.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/time.h>

#ifdef CONFIG_ARM
#include <asm/proc-fns.h>
#endif
#include <asm/suspend.h>

#include "dt_idle_states.h"
#include "cpuidle-sprd.h"

static int cpuidle_debug;
module_param_named(cpuidle_debug, cpuidle_debug, int, S_IRUGO | S_IWUSR);
static void core_pd_en(void)
{
}
static void core_pd_dis(void)
{
}

#ifdef CONFIG_ARM

static int real_suspend_fn(unsigned long cpu)
{
	cpu_do_idle();
	return 0;
}
/*
 * arm_enter_idle_state - Programs CPU to enter the specified state
 *
 * @dev: cpuidle device
 * @drv: cpuidle driver
 * @idx: state index
 *
 * Called from the CPUidle framework to program the device to the
 * specified target state selected by the governor.
 */
static int arm_enter_idle_state(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int idx)
{
	int ret = 0;
	struct timeval start_time, end_time;
	long usec_elapsed;
	struct device_node *np = of_find_node_by_name(NULL , "idle-states");


	if (cpuidle_debug)
		do_gettimeofday(&start_time);

	switch (idx) {
	case STANDBY:
		cpu_do_idle();
		break;
	case L_SLEEP:
		light_sleep_en(np);
		cpu_do_idle();
		light_sleep_dis(np);
		break;
	case H_SLEEP:
		light_doze_sleep_en(np);
		cpu_do_idle();
		light_doze_sleep_dis(np);
		break;
	case CORE_PD:
		cpu_pm_enter();
		core_pd_en();
		ret = cpu_suspend(idx, real_suspend_fn);
		core_pd_dis();
		cpu_pm_exit();
		break;

	default:
		cpu_do_idle();
		WARN(1, "[CPUIDLE]: NO THIS IDLE LEVEL!!!");
	}
	if (cpuidle_debug) {
		do_gettimeofday(&end_time);
		usec_elapsed = (end_time.tv_sec - start_time.tv_sec) * 1000000 +
			(end_time.tv_usec - start_time.tv_usec);
		pr_info("[CPUIDLE] Enter idle state: %d ,usec_elapsed = %ld \n",
			idx, usec_elapsed);
	}
	return ret ? -1 : idx;
}
#endif

#ifdef CONFIG_ARM64
/*
 * arm_enter_idle_state - Programs CPU to enter the specified state
 *
 * @dev: cpuidle device
 * @drv: cpuidle driver
 * @idx: state index
 *
 * Called from the CPUidle framework to program the device to the
 * specified target state selected by the governor.
 */
static int arm_enter_idle_state(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int idx)
{
	int ret = 0;
	struct timeval start_time, end_time;
	long usec_elapsed;
	struct device_node *np = of_find_node_by_name(NULL , "idle-states");

	if (cpuidle_debug)
		do_gettimeofday(&start_time);

	switch (idx) {
	case STANDBY:
		cpu_do_idle();
		break;
	case L_SLEEP:
		light_sleep_en(np);
		cpu_do_idle();
		light_sleep_dis(np);
		break;
	case CORE_PD:
		light_sleep_en(np);
		cpu_pm_enter();
		ret = cpu_suspend(idx);
		cpu_pm_exit();
		light_sleep_dis(np);
		break;
	case CLUSTER_PD:
		light_sleep_en(np);
		cpu_pm_enter();
		cpu_cluster_pm_enter();
		ret = cpu_suspend(idx);
		cpu_cluster_pm_exit();
		cpu_pm_exit();
		light_sleep_dis(np);
		break;
	case TOP_PD:
		light_sleep_en(np);
		cpu_pm_enter();
		cpu_cluster_pm_enter();
		ret = cpu_suspend(idx);
		cpu_cluster_pm_exit();
		cpu_pm_exit();
		light_sleep_dis(np);
		break;
	default:
		cpu_do_idle();
		WARN(1, "[CPUIDLE]: NO THIS IDLE LEVEL!!!");
	}
	if (cpuidle_debug) {
		do_gettimeofday(&end_time);
		usec_elapsed = (end_time.tv_sec - start_time.tv_sec) * 1000000 +
			(end_time.tv_usec - start_time.tv_usec);
		pr_info("[CPUIDLE] Enter idle state: %d ,usec_elapsed = %ld \n",
			idx, usec_elapsed);
	}
	return ret ? -1 : idx;
}
#endif

static struct cpuidle_driver arm_idle_driver = {
	.name = "arm_idle_sprd",
	.owner = THIS_MODULE,
	/*
	 * State at index 0 is standby wfi and considered standard
	 * on all ARM platforms. If in some platforms simple wfi
	 * can't be used as "state 0", DT bindings must be implemented
	 * to work around this issue and allow installing a special
	 * handler for idle state index 0.
	 */
	.states[0] = {
		.enter                  = arm_enter_idle_state,
		.exit_latency           = 1,
		.target_residency       = 1,
		.power_usage		= UINT_MAX,
		.name                   = "WFI",
		.desc                   = "ARM WFI",
	}
};

static const struct of_device_id arm_idle_state_match[] __initconst = {
	{ .compatible = "arm,idle-state",
	  .data = arm_enter_idle_state },
	{ },
};

/*
 * arm64_idle_init
 *
 * Registers the arm64 specific cpuidle driver with the cpuidle
 * framework. It relies on core code to parse the idle states
 * and initialize them using driver data structures accordingly.
 */
static int __init arm_idle_sprd_init(void)
{
	int  ret;
	struct cpuidle_driver *drv = &arm_idle_driver;

	/*
	 * Initialize idle states data, starting at index 1.
	 * This driver is DT only, if no DT idle states are detected (ret == 0)
	 * let the driver initialization fail accordingly since there is no
	 * reason to initialize the idle driver if only wfi is supported.
	 */
	ret = dt_init_idle_driver(drv, arm_idle_state_match, 1);
	if (ret <= 0) {
		if (ret)
			pr_err("failed to initialize idle states\n");
		return ret ? : -ENODEV;
	}

#ifdef CONFIG_ARM64
	int cpu;
	/*
	 * Call arch CPU operations in order to initialize
	 * idle states suspend back-end specific data
	 */
	for_each_possible_cpu(cpu) {
		ret = cpu_init_idle(cpu);
		if (ret) {
			pr_err("CPU %d failed to init idle CPU ops\n", cpu);
			return ret;
		}
	}
#endif
	ret = cpuidle_register(drv, NULL);
	if (ret) {
		pr_err("failed to register cpuidle driver\n");
		return ret;
	}

	return 0;
}

device_initcall(arm_idle_sprd_init);
