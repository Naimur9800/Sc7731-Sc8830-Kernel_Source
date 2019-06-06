/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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

#include <linux/kernel.h>
#include <linux/cpuidle.h>
#include <linux/pm_qos.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <trace/events/power.h>

struct sprd_device {
	int		last_state_idx;
	int		base_idx;

	unsigned int	next_timer_us;
	struct hrtimer recovery_htimer;
	bool set_as_base_idx;
};

static DEFINE_PER_CPU(struct sprd_device, sprd_devices);

static void recovery_htimer_start(uint32_t expires_time_us, struct cpuidle_device *dev);
static int recovery_htimer_cancel(struct sprd_device *data);

/**
 * sprd_select - selects the next idle state to enter
 * @drv: cpuidle driver containing state data
 * @dev: the CPU
 */
static int sprd_select(struct cpuidle_driver *drv, struct cpuidle_device *dev)
{
	struct sprd_device *data = this_cpu_ptr(&sprd_devices);
	int latency_req = pm_qos_request(PM_QOS_CPU_DMA_LATENCY);
	int i;
	bool need_recovery = false;

	data->last_state_idx = data->base_idx = CPUIDLE_DRIVER_STATE_START - 1;

	/* Special case when user has set very strict latency requirement */
	if (unlikely(latency_req == 0))
		return 0;

	/* determine the expected residency time, round up */
	data->next_timer_us = ktime_to_us(tick_nohz_get_sleep_length());
	/* Find the base idle state with next_timer_us and latency_req_qos */
	for (i = CPUIDLE_DRIVER_STATE_START; i < drv->state_count; i++) {
		struct cpuidle_state *s = &drv->states[i];
		struct cpuidle_state_usage *su = &dev->states_usage[i];

		if (s->disabled || su->disable)
			continue;
		if (s->target_residency > data->next_timer_us)
			continue;
		if (s->exit_latency > latency_req)
			continue;

		data->base_idx = i;
		if ((data->base_idx > CPUIDLE_DRIVER_STATE_START) && (data->next_timer_us > 100 + 2 * s->target_residency))
			need_recovery = true;
		else
			need_recovery = false;
	}

	if (data->base_idx > CPUIDLE_DRIVER_STATE_START)
		data->last_state_idx = data->base_idx - 1;
	else
		data->last_state_idx = data->base_idx;

	if (data->set_as_base_idx) {
		data->set_as_base_idx = false;
		data->last_state_idx = data->base_idx;
	} else if (need_recovery)
		recovery_htimer_start(drv->states[data->base_idx].target_residency, dev);

	return data->last_state_idx;
}

/**
 * sprd_reflect - records that data structures need update
 * @dev: the CPU
 * @index: the index of actual entered state
 *
 * NOTE: it's important to be fast here because this operation will add to
 *       the overall exit latency.
 */
static void sprd_reflect(struct cpuidle_device *dev, int index)
{
	struct sprd_device *data = this_cpu_ptr(&sprd_devices);
	data->last_state_idx = index;
	recovery_htimer_cancel(data);
}

static int recovery_htimer_cancel(struct sprd_device *data)
{
	return hrtimer_try_to_cancel(&data->recovery_htimer);
}

static enum hrtimer_restart recovery_htimer_fn(struct hrtimer *h)
{
	struct cpuidle_device *dev = __this_cpu_read(cpuidle_devices);
	struct sprd_device *data = &per_cpu(sprd_devices, dev->cpu);

	data->set_as_base_idx = true;

	return HRTIMER_NORESTART;
}

static void recovery_htimer_start(uint32_t expires_time_us, struct cpuidle_device *dev)
{
	struct sprd_device *data = &per_cpu(sprd_devices, dev->cpu);
	uint64_t expires_time_ns = expires_time_us * NSEC_PER_USEC;
	ktime_t expires_ktime = ns_to_ktime(expires_time_ns);

	data->recovery_htimer.function = recovery_htimer_fn;
	hrtimer_start(&data->recovery_htimer, expires_ktime, HRTIMER_MODE_REL_PINNED);
}

/**
 * sprd_enable_device - scans a CPU's states and does setup
 * @drv: cpuidle driver
 * @dev: the CPU
 */
static int sprd_enable_device(struct cpuidle_driver *drv,
				struct cpuidle_device *dev)
{
	struct sprd_device *data = &per_cpu(sprd_devices, dev->cpu);

	memset(data, 0, sizeof(struct sprd_device));
	hrtimer_init(&data->recovery_htimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	return 0;
}

static struct cpuidle_governor sprd_governor = {
	.name =		"sprd",
	.rating =	30,
	.enable =	sprd_enable_device,
	.select =	sprd_select,
	.reflect =	sprd_reflect,
	.owner =	THIS_MODULE,
};

/**
 * init_sprd - initializes the governor
 */
static int __init init_sprd(void)
{
	return cpuidle_register_governor(&sprd_governor);
}

postcore_initcall(init_sprd);
