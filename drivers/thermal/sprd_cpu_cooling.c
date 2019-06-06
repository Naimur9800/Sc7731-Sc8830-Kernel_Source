/*
 * Copyright (C) 2012-2015 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/rculist.h>
#include <linux/rcupdate.h>
#include <linux/debugfs.h>
#include <linux/suspend.h>

#include <linux/cpu.h>
#include <linux/sprd_cpu_cooling.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/sprd_otp.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#endif
#include <trace/events/thermal.h>

#ifdef SPRDCPU_DEBUG__
#define SPRDCPU_DEBUG(format, arg...) pr_info("cpu-cooling: " format, ## arg)
#else
#define SPRDCPU_DEBUG(format, arg...) pr_debug("cpu-cooling: " format, ## arg)
#endif

#define MAX_CPU_STATE	8
#define MAX_LIT_CPU_MUN	4

static atomic_t in_suspend;

enum cluster_type {
	CLUSTER_LITTLE = 0,
	CLUSTER_BIG,
	NUM_CLUSTERS
};

struct cooling_state {
	int max_freq;
	int max_core;
};

struct freq_vddarm {
	unsigned int freq;
	unsigned int vddarm_mv;
};

struct vddarm_update {
	unsigned long state;
	struct freq_vddarm *freq_vddarm;
};

struct cpu_cooling_param_t {
	unsigned long state;
	unsigned long max_core;
	unsigned long limit_freq;
};

struct thermal_cooling_info_t {
	char type[THERMAL_NAME_LENGTH];
	unsigned long cooling_state;
	struct thermal_cooling_device *cdev;
	int max_state;
	int enable;
	int cluster;
	struct cooling_state cpu_state[MAX_CPU_STATE];
	struct vddarm_update *vddarm_update;
	int state_num;
	void *devdata;
	struct cpu_cooling_param_t param;
	int binning;
};

/*
 * Cooling state <-> CPUFreq frequency
 *
 * Cooling states are translated to frequencies throughout this driver and this
 * is the relation between them.
 *
 * Highest cooling state corresponds to lowest possible frequency.
 *
 * i.e.
 *	level 0 --> 1st Max Freq
 *	level 1 --> 2nd Max Freq
 *	...
 */

/**
 * struct power_table - frequency to power conversion
 * @frequency:	frequency in KHz
 * @power:	power in mW
 *
 * This structure is built when the cooling device registers and helps
 * in translating frequency to power and viceversa.
 */
struct power_table {
	u32 frequency;
	u32 power;
};

struct assassin_data {
	unsigned int cpu;
	bool pending_kill;
};

struct online_data {
	u32 target_online_cpus;
	u32 rounds;
};

/**
 * struct cpufreq_cooling_device - data for cooling device with cpufreq
 * @id: unique integer value corresponding to each cpufreq_cooling_device
 *	registered.
 * @cool_dev: thermal_cooling_device pointer to keep track of the
 *	registered cooling device.
 * @cpufreq_state: integer value representing the current state of cpufreq
 *	cooling	devices.
 * @clipped_freq: integer value representing the absolute value of the clipped
 *	frequency.
 * @max_level: maximum cooling level. One less than total number of valid
 *	cpufreq frequencies.
 * @allowed_cpus: all the cpus involved for this cpufreq_cooling_device.
 * @node: list_head to link all cpufreq_cooling_device together.
 * @last_load: load measured by the latest call to cpufreq_get_actual_power()
 * @time_in_idle: previous reading of the absolute time that this cpu was idle
 * @time_in_idle_timestamp: wall time of the last invocation of
 *	get_cpu_idle_time_us()
 * @dyn_power_table: array of struct power_table for frequency to power
 *	conversion, sorted in ascending order.
 * @dyn_power_table_entries: number of entries in the @dyn_power_table array
 * @cpu_dev: the first cpu_device from @allowed_cpus that has OPPs registered
 * @plat_get_static_power: callback to calculate the static power
 *
 * This structure is required for keeping information of each registered
 * cpufreq_cooling_device.
 */
struct cpufreq_cooling_device {
	int id;
	struct thermal_cooling_device *cool_dev;
	unsigned int cpufreq_state;
	unsigned int cpufreq_val;
	unsigned int max_level;
	unsigned int *freq_table;	/* In descending order */
	struct cpumask allowed_cpus;
	struct cpumask hotplug_cpus;
	struct list_head node;
	u32 last_load;
	u64 *time_in_idle;
	u64 *time_in_idle_timestamp;
	struct power_table *dyn_power_table;
	struct power_table *dyn_l2_power_table;
	int dyn_power_table_entries;
	struct device *cpu_dev;
	u32 hotplug_refractory_period;
	get_static_t plat_get_static_power;
	struct work_struct assassin;
	struct assassin_data assassin_data;
	struct online_data online_data;
	struct cpu_cooling_param_t param;
	struct thermal_cooling_info_t *info;
};

/*
 * Tscale = aT^3 + bT^2 + cT + d
 * Vscale = aV^3 + bV^2 + cV + d
*/
struct scale_coeff {
	int scale_a;
	int scale_b;
	int scale_c;
	int scale_d;
};

/* Pdyn = dynperghz * freq * (V/Vbase)^2 */
struct dyn_power_coeff {
	int dynperghz;
	int freq;
	int voltage_base;
};

struct cluster_power_coefficients {
	u32 hotplug_period;
	int leak_core_base;
	int leak_cluster_base;
	struct scale_coeff temp_scale;
	struct scale_coeff voltage_scale;
	struct dyn_power_coeff core_coeff;
	struct dyn_power_coeff cluster_coeff;
	int weight;
	void *devdata;
};

struct cluster_power_coefficients cluster_data[] = {
	[CLUSTER_LITTLE] = {
		.hotplug_period = 0,
		.leak_core_base = 27,
		.leak_cluster_base = 38,
	},
	[CLUSTER_BIG] = {
		.hotplug_period = 10,
		.leak_core_base = 324,
		.leak_cluster_base = 311,
	},
};

static DEFINE_IDR(cpufreq_idr);
static DEFINE_MUTEX(cooling_cpufreq_lock);

static LIST_HEAD(cpufreq_dev_list);

#define SPRD_CPU_STORE(_name) \
	static ssize_t sprd_cpu_store_##_name(struct device *dev, \
			struct device_attribute *attr, \
			const char *buf, size_t count);
#define SPRD_CPU_SHOW(_name) \
	static ssize_t sprd_cpu_show_##_name(struct device *dev, \
			struct device_attribute *attr, \
			char *buf);

/* sys I/F for cooling device */
#define to_cooling_device(_dev)	\
	container_of(_dev, struct thermal_cooling_device, device)

#define SPRD_CPU_ATTR(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IRUGO | S_IWUSR | S_IWGRP,},  \
	.show = sprd_cpu_show_##_name,                  \
	.store = sprd_cpu_store_##_name,                              \
}
#define SPRD_CPU_ATTR_RO(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IRUGO, },  \
	.show = sprd_cpu_show_##_name,                  \
}
#define SPRD_CPU_ATTR_WO(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IWUSR | S_IWGRP, },  \
	.store = sprd_cpu_store_##_name,                              \
}

SPRD_CPU_SHOW(cur_ctrl_param);
SPRD_CPU_STORE(cur_ctrl_param);
static struct device_attribute sprd_cpu_atrr[] = {
	SPRD_CPU_ATTR(cur_ctrl_param),
};

static int sprd_cpu_creat_attr(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(sprd_cpu_atrr); i++) {
		rc = device_create_file(dev, &sprd_cpu_atrr[i]);
		if (rc)
			goto sprd_attrs_failed;
	}
	goto sprd_attrs_succeed;

sprd_attrs_failed:
	while (i--)
		device_remove_file(dev, &sprd_cpu_atrr[i]);

sprd_attrs_succeed:
	return rc;
}

static int sprd_cpu_remove_attr(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sprd_cpu_atrr); i++) {
		device_remove_file(dev, &sprd_cpu_atrr[i]);
	}
	return 0;
}

static ssize_t sprd_cpu_show_cur_ctrl_param(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i = 0;
	int offset = 0;
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	struct cpufreq_cooling_device *cpufreq_dev = cdev->devdata;
	struct thermal_cooling_info_t *info = cpufreq_dev->info;
	unsigned long *data = (unsigned long *)&info->param;
	int len = sizeof(info->param) / sizeof(info->param.state);

	for (i = 0; i < len; i++)
		offset += sprintf(buf + offset, "%ld,", data[i]);
	buf[offset - 1] = '\n';

	SPRDCPU_DEBUG("dev_attr_cur_ctrl_param: %s\n", buf);
	return offset;
}

#define BUF_LEN	128
static int sprd_cpu_core_limit(struct cpufreq_cooling_device *cpufreq_dev, unsigned int target_online_cpus);
int cpufreq_table_thermal_update(struct cpufreq_cooling_device *cpufreq_dev,
		unsigned long freq, unsigned long voltage);

static ssize_t sprd_cpu_store_cur_ctrl_param(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int i = 0;
	char *str = NULL;
	char *pbuf;
	char *after = NULL;
	char buffer[BUF_LEN];
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	struct cpufreq_cooling_device *cpufreq_dev= cdev->devdata;
	struct thermal_cooling_info_t *info = cpufreq_dev->info;
	unsigned long *data = (unsigned long *)&info->param;
	int len = sizeof(info->param) / sizeof(info->param.state);
	unsigned int cpu = cpumask_any(&cpufreq_dev->allowed_cpus);
	long low_freq, low_vol;


	if (count > BUF_LEN){
		count = BUF_LEN;
	}
	strncpy(buffer, buf, count);
	buffer[count] = '\0';
	SPRDCPU_DEBUG("buf param form usespace:%s\n", buf);
	SPRDCPU_DEBUG("buffer parm:%s\n", buffer);

	pbuf = buffer;
	while ((pbuf!= NULL) && (i < len))
	{
		str = strsep(&pbuf, ",");
		data[i++] = simple_strtoul(str, &after, 0);
	}

	if (i < len) {
		SPRDCPU_DEBUG("buffer parm is too shot!\n");
		return count;
	}
	SPRDCPU_DEBUG("param.status: %ld\n", info->param.state);
	SPRDCPU_DEBUG("param.core_num: %ld\n", info->param.max_core);
	SPRDCPU_DEBUG("param.freq: %ld\n", info->param.limit_freq);

	cpufreq_dev->cpufreq_state = info->param.state;
	cpufreq_dev->cpufreq_val = info->param.limit_freq;
	cpufreq_update_policy(cpu);

	sprd_cpu_core_limit(cpufreq_dev, info->param.max_core);

	for (str = strsep(&pbuf, ","); str != NULL; str = strsep(&pbuf, ",")){
		low_freq = simple_strtoul(str, &after, 0);
		str = strsep(&pbuf, ",");
		low_vol = simple_strtoul(str, &after, 0);
		SPRDCPU_DEBUG("change %ld voltage to %ld\n", low_freq, low_vol);
		if (info->binning == 1)
			cpufreq_table_thermal_update(cpufreq_dev, low_freq, low_vol);
	}

	return count;
}

int cpufreq_table_thermal_update(
		struct cpufreq_cooling_device *cpufreq_dev,
		unsigned long freq, unsigned long voltage)
{
	return 0;
}

static int sprd_cpu_core_limit(struct cpufreq_cooling_device *cpufreq_dev,
		unsigned int target_online_cpus)
{
	int ret, cpu;
	struct cpumask our_online_cpus;
	u32 current_online_cpus;

	cpumask_and(&our_online_cpus, &cpufreq_dev->allowed_cpus,
			cpu_online_mask);
	current_online_cpus = cpumask_weight(&our_online_cpus);

	SPRDCPU_DEBUG("target_online_cpus: %d, current_online_cpus: %d\n",
			target_online_cpus, current_online_cpus);

	for_each_cpu(cpu, &cpufreq_dev->allowed_cpus) {
		if (target_online_cpus == current_online_cpus) {
			break;
		} else if (target_online_cpus > current_online_cpus) {
			if (!cpu_online(cpu)) {
				SPRDCPU_DEBUG("we gonna plugin cpu%d\n", cpu);
				ret = cpu_up(cpu);
				if (!ret || cpu_online(cpu))
					current_online_cpus++;
			}
		} else {
			/*
			 * Don't offline the first cpu of a cluster until you
			 * have offlined all the other ones.
			 */
			if (cpu == 0 && current_online_cpus > 1)
				continue;

			if ((!atomic_read(&in_suspend)) && (cpu_online(cpu))) {
				ret = cpu_down(cpu);
				if (ret)
					pr_warn("failed to offline\n");
				current_online_cpus--;
			}
		}
	}

	/*
	 * Note that target_online_cpus may not be current_online_cpus
	 * due to a variety of reasons: cpu0 can't be hotplugged out,
	 * a previous hotplug operation may be racing with us,... We
	 * can't use get_online_cpus/put_online_cpus() because we
	 * *are* changing the number of online cpus
	 */
	return 0;
}

/* return (leak * 10)  */
static int get_cpu_static_power_coeff(enum cluster_type cluster)
{
	return cluster_data[cluster].leak_core_base;
}

/* return (leak * 10)  */
static int get_cache_static_power_coeff(enum cluster_type cluster)
{
	return cluster_data[cluster].leak_cluster_base;
}

static u64 get_core_dyn_power(
int cluster, unsigned int freq_mhz, unsigned int voltage_mv)
{
	u64 power = 0;
	int voltage_base = cluster_data[cluster].core_coeff.voltage_base;
	int dyn_base = cluster_data[cluster].core_coeff.dynperghz;

	power = (u64)dyn_base * freq_mhz * voltage_mv * voltage_mv;
	power = power / (voltage_base * voltage_base);
	do_div(power, 10000);

	return power;
}

static u64 get_cluster_dyn_power(
int cluster, unsigned int freq_mhz, unsigned int voltage_mv)
{
	u64 power = 0;
	int voltage_base = cluster_data[cluster].cluster_coeff.voltage_base;
	int dyn_base = cluster_data[cluster].cluster_coeff.dynperghz;

	power = (u64)dyn_base * freq_mhz * voltage_mv * voltage_mv;
	power = power / (voltage_base * voltage_base);
	do_div(power, 10000);

	return power;
}

/**
 * get_idr - function to get a unique id.
 * @idr: struct idr * handle used to create a id.
 * @id: int * value generated by this function.
 *
 * This function will populate @id with an unique
 * id, using the idr API.
 *
 * Return: 0 on success, an error code on failure.
 */
static int get_idr(struct idr *idr, int *id)
{
	int ret;

	mutex_lock(&cooling_cpufreq_lock);
	ret = idr_alloc(idr, NULL, 0, 0, GFP_KERNEL);
	mutex_unlock(&cooling_cpufreq_lock);
	if (unlikely(ret < 0))
		return ret;
	*id = ret;

	return 0;
}

/**
 * release_idr - function to free the unique id.
 * @idr: struct idr * handle used for creating the id.
 * @id: int value representing the unique id.
 */
static void release_idr(struct idr *idr, int id)
{
	mutex_lock(&cooling_cpufreq_lock);
	idr_remove(idr, id);
	mutex_unlock(&cooling_cpufreq_lock);
}

/* Below code defines functions to be used for cpufreq as cooling device */

/**
 * is_cpufreq_valid - function to check frequency transitioning capability.
 * @cpu: cpu for which check is needed.
 *
 * This function will check the current state of the system if
 * it is capable of changing the frequency for a given @cpu.
 *
 * Return: 0 if the system is not currently capable of changing
 * the frequency of given cpu. !0 in case the frequency is changeable.
 */
static int is_cpufreq_valid(int cpu)
{
	struct cpufreq_policy policy;

	return !cpufreq_get_policy(&policy, cpu);
}

enum cpufreq_cooling_property {
	GET_LEVEL,
	GET_FREQ,
	GET_MAXL,
};

/**
 * get_property - fetch a property of interest for a give cpu.
 * @cpu: cpu for which the property is required
 * @input: query parameter
 * @output: query return
 * @property: type of query (frequency, level, max level)
 *
 * This is the common function to
 * 1. get maximum cpu cooling states
 * 2. translate frequency to cooling state
 * 3. translate cooling state to frequency
 * Note that the code may be not in good shape
 * but it is written in this way in order to:
 * a) reduce duplicate code as most of the code can be shared.
 * b) make sure the logic is consistent when translating between
 *    cooling states and frequencies.
 *
 * Return: 0 on success, -EINVAL when invalid parameters are passed.
 */
static int get_property(unsigned int cpu, unsigned long input,
		unsigned int *output,
		enum cpufreq_cooling_property property)
{
	int i;
	unsigned long max_level = 0, level = 0;
	unsigned int freq = CPUFREQ_ENTRY_INVALID;
	int descend = -1;
	struct cpufreq_frequency_table *pos, *table =
		cpufreq_frequency_get_table(cpu);

	if (!output)
		return -EINVAL;

	if (!table)
		return -EINVAL;

	cpufreq_for_each_valid_entry(pos, table) {
		/* ignore duplicate entry */
		if (freq == pos->frequency)
			continue;

		/* get the frequency order */
		if (freq != CPUFREQ_ENTRY_INVALID && descend == -1)
			descend = freq > pos->frequency;

		freq = pos->frequency;
		max_level++;
	}

	/* No valid cpu frequency entry */
	if (max_level == 0)
		return -EINVAL;

	/* max_level is an index, not a counter */
	max_level--;

	/* get max level */
	if (property == GET_MAXL) {
		*output = (unsigned int)max_level;
		return 0;
	}

	if (property == GET_FREQ)
		level = descend ? input : (max_level - input);

	i = 0;
	cpufreq_for_each_valid_entry(pos, table) {
		/* ignore duplicate entry */
		if (freq == pos->frequency)
			continue;

		/* now we have a valid frequency entry */
		freq = pos->frequency;

		if (property == GET_LEVEL && (unsigned int)input == freq) {
			/* get level by frequency */
			*output = descend ? i : (max_level - i);
			return 0;
		}
		if (property == GET_FREQ && level == i) {
			/* get frequency by level */
			*output = freq;
			return 0;
		}
		i++;
	}

	return -EINVAL;
}

/**
 * cpufreq_cooling_get_level - for a give cpu, return the cooling level.
 * @cpu: cpu for which the level is required
 * @freq: the frequency of interest
 *
 * This function will match the cooling level corresponding to the
 * requested @freq and return it.
 *
 * Return: The matched cooling level on success or THERMAL_CSTATE_INVALID
 * otherwise.
 */
unsigned long cpufreq_cooling_get_level(unsigned int cpu, unsigned int freq)
{
	unsigned int val;

	if (get_property(cpu, (unsigned long)freq, &val, GET_LEVEL))
		return THERMAL_CSTATE_INVALID;

	return (unsigned long)val;
}
EXPORT_SYMBOL_GPL(cpufreq_cooling_get_level);

/**
 * get_cpu_frequency - get the absolute value of frequency from level.
 * @cpu: cpu for which frequency is fetched.
 * @level: cooling level
 *
 * This function matches cooling level with frequency. Based on a cooling level
 * of frequency, equals cooling state of cpu cooling device, it will return
 * the corresponding frequency.
 *	e.g level=0 --> 1st MAX FREQ, level=1 ---> 2nd MAX FREQ, .... etc
 *
 * Return: 0 on error, the corresponding frequency otherwise.
 */
static unsigned int get_cpu_frequency(unsigned int cpu, unsigned long level)
{
	int ret = 0;
	unsigned int freq;

	ret = get_property(cpu, level, &freq, GET_FREQ);
	if (ret)
		return 0;

	return freq;
}

/**
 * cpufreq_apply_cooling - function to apply frequency clipping.
 * @cpufreq_device: cpufreq_cooling_device pointer containing frequency
 *	clipping data.
 * @cooling_state: value of the cooling state.
 *
 * Function used to make sure the cpufreq layer is aware of current thermal
 * limits. The limits are applied by updating the cpufreq policy.
 *
 * Return: 0 on success, an error code otherwise (-EINVAL in case wrong
 * cooling state).
 */
static int cpufreq_apply_cooling(struct cpufreq_cooling_device *cpufreq_device,
		unsigned long cooling_state)
{
	unsigned int cpuid, clip_freq;
	struct cpumask *mask = &cpufreq_device->allowed_cpus;
	unsigned int cpu = cpumask_any(mask);

	/* Check if the old cooling action is same as new cooling action */
	if (cpufreq_device->cpufreq_state == cooling_state)
		return 0;

	clip_freq = get_cpu_frequency(cpu, cooling_state);
	if (!clip_freq)
		return -EINVAL;

	cpufreq_device->cpufreq_state = cooling_state;
	cpufreq_device->cpufreq_val = clip_freq;

	for_each_cpu(cpuid, mask) {
		if (is_cpufreq_valid(cpuid)) {
			cpufreq_update_policy(cpuid);
			break;
		}
	}

	return 0;
}

/**
 * cpufreq_thermal_notifier - notifier callback for cpufreq policy change.
 * @nb:	struct notifier_block * with callback info.
 * @event: value showing cpufreq event for which this function invoked.
 * @data: callback-specific data
 *
 * Callback to hijack the notification on cpufreq policy transition.
 * Every time there is a change in policy, we will intercept and
 * update the cpufreq policy with thermal constraints.
 *
 * Return: 0 (success)
 */
static int cpufreq_thermal_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned long max_freq = 0;
	struct cpufreq_cooling_device *cpufreq_dev;

	if (event != CPUFREQ_ADJUST)
		return 0;

	mutex_lock(&cooling_cpufreq_lock);
	list_for_each_entry(cpufreq_dev, &cpufreq_dev_list, node) {
		if (!cpumask_test_cpu(policy->cpu,
					&cpufreq_dev->allowed_cpus))
			continue;

		if (!cpufreq_dev->cpufreq_val)
			cpufreq_dev->cpufreq_val = get_cpu_frequency(
					cpumask_any(&cpufreq_dev->allowed_cpus),
					cpufreq_dev->cpufreq_state);

		max_freq = cpufreq_dev->cpufreq_val;

		if (policy->max != max_freq) {
			cpufreq_verify_within_limits(policy, 0, max_freq);
			SPRDCPU_DEBUG("cpu: %d, new max_freq: %d\n",
				policy->cpu, policy->max);
		}
	}
	mutex_unlock(&cooling_cpufreq_lock);

	return 0;
}

/**
 * build_dyn_power_table() - create a dynamic power to frequency table
 * @cpufreq_device:	the cpufreq cooling device in which to store the table
 * @capacitance: dynamic power coefficient for these cpus
 *
 * Build a dynamic power to frequency table for this cpu and store it
 * in @cpufreq_device.  This table will be used in cpu_power_to_freq() and
 * cpu_freq_to_power() to convert between power and frequency
 * efficiently.  Power is stored in mW, frequency in KHz.  The
 * resulting table is in ascending order.
 *
 * Return: 0 on success, -EINVAL if there are no OPPs for any CPUs,
 * -ENOMEM if we run out of memory or -EAGAIN if an OPP was
 * added/enabled while the function was executing.
 */
static int build_dyn_power_table(struct cpufreq_cooling_device *cpufreq_device,
				u32 cluster)
{
	struct power_table *power_table = NULL;
	struct power_table *l2_power_table = NULL;
	struct dev_pm_opp *opp;
	struct device *dev = NULL;
	int num_opps = 0, cpu, i, ret = 0;
	unsigned long freq;

	for_each_cpu(cpu, &cpufreq_device->allowed_cpus) {
		dev = get_cpu_device(cpu);
		if (!dev) {
			dev_warn(&cpufreq_device->cool_dev->device,
							"No cpu device for cpu %d\n", cpu);
			continue;
		}

		num_opps = dev_pm_opp_get_opp_count(dev);
		if (num_opps > 0)
			break;
		else if (num_opps < 0)
			return num_opps;
	}

	if (num_opps == 0)
		return -EINVAL;

	power_table = kcalloc(num_opps, sizeof(*power_table), GFP_KERNEL);
	if (!power_table)
		return -ENOMEM;
	l2_power_table = kcalloc(num_opps, sizeof(*l2_power_table), GFP_KERNEL);
	if (!l2_power_table)
		goto free_power_table;

	rcu_read_lock();

	for (freq = 0, i = 0;
				opp = dev_pm_opp_find_freq_ceil(dev, &freq), !IS_ERR(opp);
				freq++, i++) {
		u32 freq_mhz, voltage_mv;
		u64 power, l2_power;

		if (i >= num_opps) {
			rcu_read_unlock();
			ret = -EAGAIN;
			goto free_l2_power_table;
		}

		freq_mhz = freq / 1000000;
		voltage_mv = dev_pm_opp_get_voltage(opp) / 1000;

		power = get_core_dyn_power(cluster, freq_mhz, voltage_mv);
		l2_power = get_cluster_dyn_power(cluster, freq_mhz, voltage_mv);

		/* frequency is stored in power_table in KHz */
		power_table[i].frequency = freq / 1000;
		l2_power_table[i].frequency = freq / 1000;

		/* power is stored in mW */
		power_table[i].power = power;
		l2_power_table[i].power = l2_power;
	}

	rcu_read_unlock();

	if (i != num_opps) {
		ret = PTR_ERR(opp);
		goto free_l2_power_table;
	}

	cpufreq_device->cpu_dev = dev;
	cpufreq_device->dyn_power_table = power_table;
	cpufreq_device->dyn_l2_power_table = l2_power_table;
	cpufreq_device->dyn_power_table_entries = i;

	return 0;

free_l2_power_table:
	kfree(l2_power_table);

free_power_table:
	kfree(power_table);

	return ret;
}

static u32 cpu_freq_to_power(struct cpufreq_cooling_device *cpufreq_device,
				u32 freq)
{
	int i;
	struct power_table *pt = cpufreq_device->dyn_power_table;

	for (i = 1; i < cpufreq_device->dyn_power_table_entries; i++)
		if (freq < pt[i].frequency)
			break;

	return pt[i - 1].power;
}

static u32 l2_freq_to_power(struct cpufreq_cooling_device *cpufreq_device,
				u32 freq)
{
	int i;
	struct power_table *pt = cpufreq_device->dyn_l2_power_table;

	for (i = 1; i < cpufreq_device->dyn_power_table_entries; i++)
		if (freq < pt[i].frequency)
			break;

	return pt[i - 1].power;
}

static u32 cpu_power_to_freq(struct cpufreq_cooling_device *cpufreq_device,
				u32 power)
{
	int i;
	struct power_table *pt = cpufreq_device->dyn_power_table;

	for (i = 1; i < cpufreq_device->dyn_power_table_entries; i++)
		if (power < pt[i].power)
			break;

	return pt[i - 1].frequency;
}

/**
 * get_load() - get load for a cpu since last updated
 * @cpufreq_device:	&struct cpufreq_cooling_device for this cpu
 * @cpu:	cpu number
 *
 * Return: The average load of cpu @cpu in percentage since this
 * function was last called.
 */
static u32 get_load(struct cpufreq_cooling_device *cpufreq_device, int cpu)
{
	u32 load;
	u64 now = 0, now_idle, delta_time, delta_idle;

	now_idle = get_cpu_idle_time(cpu, &now, 0);
	delta_idle = now_idle - cpufreq_device->time_in_idle[cpu];
	delta_time = now - cpufreq_device->time_in_idle_timestamp[cpu];

	if (delta_time <= delta_idle)
		load = 0;
	else
		load = div64_u64(100 * (delta_time - delta_idle), delta_time);

	cpufreq_device->time_in_idle[cpu] = now_idle;
	cpufreq_device->time_in_idle_timestamp[cpu] = now;

	return load;
}

/**
 * get_static_power() - calculate the static power consumed by the cpus
 * @cpufreq_device:	struct &cpufreq_cooling_device for this cpu cdev
 * @tz:		thermal zone device in which we're operating
 * @freq:	frequency in KHz
 * @power:	pointer in which to store the calculated static power
 *
 * Calculate the static power consumed by the cpus described by
 * @cpu_actor running at frequency @freq.  This function relies on a
 * platform specific function that should have been provided when the
 * actor was registered.  If it wasn't, the static power is assumed to
 * be negligible.  The calculated static power is stored in @power.
 *
 * Return: 0 on success, -E* on failure.
 */
static int get_static_power(struct cpufreq_cooling_device *cpufreq_device,
				struct thermal_zone_device *tz, unsigned long freq,
				u32 *power)
{
	struct dev_pm_opp *opp;
	unsigned long voltage, temperature;
	struct cpumask our_online_cpus;
	unsigned long freq_hz = freq * 1000;

	cpumask_and(&our_online_cpus,
		&cpufreq_device->allowed_cpus, cpu_online_mask);

	if (!cpufreq_device->plat_get_static_power ||
					!cpufreq_device->cpu_dev ||
					cpumask_empty(&our_online_cpus)) {
		*power = 0;
		return 0;
	}

	rcu_read_lock();

	opp = dev_pm_opp_find_freq_exact(cpufreq_device->cpu_dev, freq_hz,
					true);
	voltage = dev_pm_opp_get_voltage(opp);

	rcu_read_unlock();

	if (voltage == 0) {
		dev_warn_ratelimited(cpufreq_device->cpu_dev,
						"Failed to get voltage for frequency %lu: %ld\n",
						freq_hz, IS_ERR(opp) ? PTR_ERR(opp) : 0);
		return -EINVAL;
	}

	/*
	 * We can call thermal_zone_get_temp() here because this
	 * function may be called with tz->lock held.  Use the latest
	 * read temperature instead.
	 */
	temperature = (unsigned long)tz->temperature;

	*power = cpufreq_device->plat_get_static_power(&our_online_cpus,
							voltage, temperature);

	return 0;
}

/**
 * get_dynamic_power() - calculate the dynamic power
 * @cpufreq_device:	&cpufreq_cooling_device for this cdev
 * @freq:	current frequency
 *
 * Return: the dynamic power consumed by the cpus described by
 * @cpufreq_device.
 */
static u32 get_dynamic_power(struct cpufreq_cooling_device *cpufreq_device,
				unsigned long freq)
{
	u32 raw_cpu_power;

	raw_cpu_power = cpu_freq_to_power(cpufreq_device, freq);
	return (raw_cpu_power * cpufreq_device->last_load) / 100;
}

static u32 get_dynamic_l2_power(struct cpufreq_cooling_device *cpufreq_device,
				unsigned long freq)
{
	u32 raw_cpu_power, load;
	struct cpumask our_online_cpus;
	unsigned int num_our_online_cpus;

	cpumask_and(&our_online_cpus,
			&cpufreq_device->allowed_cpus, cpu_online_mask);
	num_our_online_cpus = cpumask_weight(&our_online_cpus);
	if (num_our_online_cpus == 0)
		load = 0;
	else
		load = cpufreq_device->last_load / num_our_online_cpus;
	raw_cpu_power = l2_freq_to_power(cpufreq_device, freq);
	return (raw_cpu_power * load) / 100;
}

static void order_killing_cpu(struct cpufreq_cooling_device *cpufreq_device, unsigned int cpu)
{
	cpufreq_device->assassin_data.cpu = cpu;
	cpufreq_device->assassin_data.pending_kill = true;
	schedule_work(&cpufreq_device->assassin);
}

static void kill_cpu(struct work_struct *work)
{
	int ret;
	struct cpufreq_cooling_device *cpufreq_device = container_of(work, struct cpufreq_cooling_device, assassin);
	unsigned int cpu = cpufreq_device->assassin_data.cpu;
	SPRDCPU_DEBUG("we gonna unplug cpu: %d\n", cpu);

	ret = cpu_down(cpu);
	if (ret)
		pr_warn("failed to offline\n");

	cpufreq_device->assassin_data.pending_kill = false;
}

static int is_hotplug_cpu(int cpu, const struct cpumask *cpumask)
{
	return cpumask_test_cpu(cpu, cpumask);
}

/*
 * We only change the online cpus after being called
 * hotplug_refractory_period times and then we average all the rounds.
 * This is to smooth the hotplug curve and prevent pingponging cpus
 */
static void set_online_cpus(struct cpufreq_cooling_device *cpufreq_dev,
			unsigned int target_online_cpus)
{
	int ret, cpu;
	struct cpumask our_online_cpus;
	u32 current_online_cpus;
	struct online_data *online_data = &cpufreq_dev->online_data;

	online_data->target_online_cpus += target_online_cpus;
	online_data->rounds++;

	if (online_data->rounds < cpufreq_dev->hotplug_refractory_period)
		return;

	target_online_cpus = online_data->target_online_cpus / online_data->rounds;
	online_data->target_online_cpus = 0;
	online_data->rounds = 0;
	cpumask_and(&our_online_cpus, &cpufreq_dev->allowed_cpus,
		    cpu_online_mask);
	current_online_cpus = cpumask_weight(&our_online_cpus);

	for_each_cpu(cpu, &cpufreq_dev->allowed_cpus) {
		if (target_online_cpus == current_online_cpus) {
			break;
		} else if (target_online_cpus > current_online_cpus) {
			if (!cpu_online(cpu) && is_hotplug_cpu(cpu, &cpufreq_dev->hotplug_cpus)) {
				SPRDCPU_DEBUG("we gonna plugin cpu%d\n", cpu);
				ret = cpu_up(cpu);
				if (!ret || cpu_online(cpu)) {
					current_online_cpus++;
					cpumask_clear_cpu(cpu, &cpufreq_dev->hotplug_cpus);
				}
			}
		} else {
			/*
			 * Don't offline the first cpu of a cluster until you
			 * have offlined all the other ones.
			 */
			if (cpu == cpumask_first(&cpufreq_dev->allowed_cpus) &&
			    current_online_cpus > 1)
				continue;

			if ((!atomic_read(&in_suspend)) && (cpu_online(cpu))) {
				SPRDCPU_DEBUG("we gonna down cpu%d\n", cpu);
				order_killing_cpu(cpufreq_dev, cpu);
				current_online_cpus--;
				cpumask_set_cpu(cpu, &cpufreq_dev->hotplug_cpus);

				/*
				 * order_killing_cpu() currently can
				 * only kill one cpu
				 */
				break;
			}
		}
	}

	/*
	 * Note that target_online_cpus may not be current_online_cpus
	 * due to a variety of reasons: cpu0 can't be hotplugged out,
	 * a previous hotplug operation may be racing with us,... We
	 * can't use get_online_cpus/put_online_cpus() because we
	 * *are* changing the number of online cpus
	 */
}

static void hotplug_out_cpus(struct cpufreq_cooling_device *cpufreq_device,
			u32 target_power, unsigned int target_freq)
{
	int i;
	u32 raw_cpu_power, estimated_power, per_cpu_load;
	unsigned int target_online_cpus, num_our_online_cpus;
	struct cpumask our_online_cpus;

	BUG_ON(target_freq != cpufreq_device->dyn_power_table[0].frequency);

	if (cpufreq_device->assassin_data.pending_kill)
		return;

	raw_cpu_power = cpu_freq_to_power(cpufreq_device, target_freq);
	cpumask_and(&our_online_cpus, &cpufreq_device->allowed_cpus, cpu_online_mask);
	num_our_online_cpus = cpumask_weight(&our_online_cpus);
	per_cpu_load = cpufreq_device->last_load / num_our_online_cpus;

	estimated_power = 0;
	for (i = 0; i < num_our_online_cpus; i++) {
		if (estimated_power >= target_power)
			break;

		estimated_power += (raw_cpu_power * per_cpu_load) / 100;
	}

	target_online_cpus = max(i - 1, 0);

	set_online_cpus(cpufreq_device, target_online_cpus);
}

static void hotplug_in_cpus(struct cpufreq_cooling_device *cpufreq_device)
{
	unsigned int target_online_cpus, num_our_online_cpus;
	struct cpumask our_online_cpus;

	cpumask_and(&our_online_cpus, &cpufreq_device->allowed_cpus, cpu_online_mask);
	num_our_online_cpus = cpumask_weight(&our_online_cpus);
	target_online_cpus = num_our_online_cpus + 1;

	set_online_cpus(cpufreq_device, target_online_cpus);
}

static void hotplug_keep_cpus(struct cpufreq_cooling_device *cpufreq_device)
{
	struct cpumask our_online_cpus;

	cpumask_and(&our_online_cpus,
		&cpufreq_device->allowed_cpus, cpu_online_mask);
	set_online_cpus(cpufreq_device, cpumask_weight(&our_online_cpus));
}

/* cpufreq cooling device callback functions are defined below */

/**
 * cpufreq_get_max_state - callback function to get the max cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: fill this variable with the max cooling state.
 *
 * Callback for the thermal cooling device to return the cpufreq
 * max cooling state.
 *
 * Return: 0 on success, an error code otherwise.
 */
static int cpufreq_get_max_state(struct thermal_cooling_device *cdev,
		unsigned long *state)
{
	struct cpufreq_cooling_device *cpufreq_device = cdev->devdata;
	struct cpumask *mask = &cpufreq_device->allowed_cpus;
	unsigned int cpu;
	unsigned int count = 0;
	int ret;

	cpu = cpumask_any(mask);

	ret = get_property(cpu, 0, &count, GET_MAXL);

	if (count > 0)
		*state = count;

	return ret;
}

/**
 * cpufreq_get_cur_state - callback function to get the current cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: fill this variable with the current cooling state.
 *
 * Callback for the thermal cooling device to return the cpufreq
 * current cooling state.
 *
 * Return: 0 on success, an error code otherwise.
 */
static int cpufreq_get_cur_state(struct thermal_cooling_device *cdev,
		unsigned long *state)
{
	struct cpufreq_cooling_device *cpufreq_device = cdev->devdata;

	*state = cpufreq_device->cpufreq_state;

	return 0;
}

/**
 * cpufreq_set_cur_state - callback function to set the current cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: set this variable to the current cooling state.
 *
 * Callback for the thermal cooling device to change the cpufreq
 * current cooling state.
 *
 * Return: 0 on success, an error code otherwise.
 */
static int cpufreq_set_cur_state(struct thermal_cooling_device *cdev,
		unsigned long state)
{
	struct cpufreq_cooling_device *cpufreq_device = cdev->devdata;

	return cpufreq_apply_cooling(cpufreq_device, state);
}

/**
 * cpufreq_get_requested_power() - get the current power
 * @cdev:	&thermal_cooling_device pointer
 * @tz:		a valid thermal zone device pointer
 * @power:	pointer in which to store the resulting power
 *
 * Calculate the current power consumption of the cpus in milliwatts
 * and store it in @power.  This function should actually calculate
 * the requested power, but it's hard to get the frequency that
 * cpufreq would have assigned if there were no thermal limits.
 * Instead, we calculate the current power on the assumption that the
 * immediate future will look like the immediate past.
 *
 * We use the current frequency and the average load since this
 * function was last called.  In reality, there could have been
 * multiple opps since this function was last called and that affects
 * the load calculation.  While it's not perfectly accurate, this
 * simplification is good enough and works.  REVISIT this, as more
 * complex code may be needed if experiments show that it's not
 * accurate enough.
 *
 * Return: 0 on success, -E* if getting the static power failed.
 */
static int cpufreq_get_requested_power(struct thermal_cooling_device *cdev,
				struct thermal_zone_device *tz,
				u32 *power)
{
	unsigned long freq;
	int i = 0, cpu, ret;
	u32 static_power, dynamic_power, total_load = 0;
	struct cpufreq_cooling_device *cpufreq_device = cdev->devdata;
	u32 *load_cpu = NULL;

	get_online_cpus();
	cpu = cpumask_any_and(&cpufreq_device->allowed_cpus, cpu_online_mask);

	/*
	 * All the CPUs are offline, thus the requested power by
	 * the cdev is 0
	 */
	if (cpu >= nr_cpu_ids) {
		*power = 0;
		put_online_cpus();
		return 0;
	}
	freq = cpufreq_quick_get(cpu);
	put_online_cpus();
	if (trace_thermal_power_cpu_get_power_enabled()) {
		u32 ncpus = cpumask_weight(&cpufreq_device->allowed_cpus);

		load_cpu = devm_kcalloc(&cdev->device, ncpus, sizeof(*load_cpu),
						GFP_KERNEL);
	}

	for_each_cpu(cpu, &cpufreq_device->allowed_cpus) {
		u32 load;

		if (cpu_online(cpu))
			load = get_load(cpufreq_device, cpu);
		else
			load = 0;

		total_load += load;
		if (trace_thermal_power_cpu_limit_enabled() && load_cpu)
			load_cpu[i] = load;
		i++;
	}

	cpufreq_device->last_load = total_load;

	dynamic_power = get_dynamic_power(cpufreq_device, freq);
	dynamic_power += get_dynamic_l2_power(cpufreq_device, freq);

	ret = get_static_power(cpufreq_device, tz, freq, &static_power);
	if (ret) {
		if (load_cpu)
				devm_kfree(&cdev->device, load_cpu);
		return ret;
	}
	if (load_cpu) {
		trace_thermal_power_cpu_get_power(
						&cpufreq_device->allowed_cpus,
						freq, load_cpu, i, dynamic_power, static_power);

		devm_kfree(&cdev->device, load_cpu);
	}

	*power = static_power + dynamic_power;

	return 0;
}

/**
 * cpufreq_state2power() - convert a cpu cdev state to power consumed
 * @cdev:	&thermal_cooling_device pointer
 * @tz:		a valid thermal zone device pointer
 * @state:	cooling device state to be converted
 * @power:	pointer in which to store the resulting power
 *
 * Convert cooling device state @state into power consumption in
 * milliwatts assuming 100% load.  Store the calculated power in
 * @power.
 *
 * Return: 0 on success, -EINVAL if the cooling device state could not
 * be converted into a frequency or other -E* if there was an error
 * when calculating the static power.
 */
static int cpufreq_state2power(struct thermal_cooling_device *cdev,
				   struct thermal_zone_device *tz,
				   unsigned long state, u32 *power)
{
	unsigned int freq, num_cpus;
	u32 static_power, dynamic_power;
	int ret;
	struct cpufreq_cooling_device *cpufreq_device = cdev->devdata;

	num_cpus = cpumask_weight(&cpufreq_device->allowed_cpus);

	freq = cpufreq_device->freq_table[state];
	if (!freq)
		return -EINVAL;

	dynamic_power = cpu_freq_to_power(cpufreq_device, freq) * num_cpus;
	ret = get_static_power(cpufreq_device, tz, freq, &static_power);
	if (ret)
		return ret;

	*power = static_power + dynamic_power;
	return 0;
}

/**
 * cpufreq_power2state() - convert power to a cooling device state
 * @cdev:	&thermal_cooling_device pointer
 * @tz:		a valid thermal zone device pointer
 * @power:	power in milliwatts to be converted
 * @state:	pointer in which to store the resulting state
 *
 * Calculate a cooling device state for the cpus described by @cdev
 * that would allow them to consume at most @power mW and store it in
 * @state.  Note that this calculation depends on external factors
 * such as the cpu load or the current static power.  Calling this
 * function with the same power as input can yield different cooling
 * device states depending on those external factors.
 *
 * Return: 0 on success, -ENODEV if no cpus are online or -EINVAL if
 * the calculated frequency could not be converted to a valid state.
 * The latter should not happen unless the frequencies available to
 * cpufreq have changed since the initialization of the cpu cooling
 * device.
 */
static int cpufreq_power2state(struct thermal_cooling_device *cdev,
			       struct thermal_zone_device *tz, u32 power,
			       unsigned long *state)
{
	unsigned int cpu, target_freq, num_our_online_cpus;
	int ret;
	u32 last_load, normalised_power;
	struct cpufreq_cooling_device *cpufreq_device = cdev->devdata;
	struct cpumask our_online_cpus;

	get_online_cpus();
	cpu = cpumask_any_and(&cpufreq_device->allowed_cpus, cpu_online_mask);

	last_load = cpufreq_device->last_load ?: 1;

	/* None of our cpus are online */
	if (cpu < nr_cpu_ids) {
		unsigned int cur_freq;
		s32 dyn_power;
		u32 static_power;
		u32 l2_power;

		cur_freq = cpufreq_quick_get(cpu);
		put_online_cpus();
		ret = get_static_power(cpufreq_device, tz, cur_freq, &static_power);
		if (ret)
			return ret;

		l2_power = get_dynamic_l2_power(cpufreq_device, cur_freq);

        dyn_power = power - static_power - l2_power;

		dyn_power = dyn_power > 0 ? dyn_power : 0;
		normalised_power = (dyn_power * 100) / last_load;
		target_freq = cpu_power_to_freq(cpufreq_device,
						normalised_power);

		*state = cpufreq_cooling_get_level(cpu, target_freq);
		if (*state == THERMAL_CSTATE_INVALID) {
			dev_warn_ratelimited(&cdev->device,
					     "Failed to convert %dKHz for cpu %d into a cdev state\n",
					     target_freq, cpu);
			return -EINVAL;
		}
	} else {
		put_online_cpus();
		normalised_power = (power * 100) / last_load;
		target_freq = cpu_power_to_freq(cpufreq_device,
						normalised_power);
	}

	cpumask_and(&our_online_cpus, &cpufreq_device->allowed_cpus, cpu_online_mask);
	num_our_online_cpus = cpumask_weight(&our_online_cpus);

	if (cpufreq_device->hotplug_refractory_period) {
		if (normalised_power < cpu_freq_to_power(cpufreq_device, target_freq))
			hotplug_out_cpus(cpufreq_device, normalised_power, target_freq);
		else if (num_our_online_cpus < cpumask_weight(&cpufreq_device->allowed_cpus))
			hotplug_in_cpus(cpufreq_device);
		else
			hotplug_keep_cpus(cpufreq_device);
	}

	trace_thermal_power_cpu_limit(&cpufreq_device->allowed_cpus,
				      target_freq, *state, power);
	return 0;
}

/*
 *Tscale = 0.0000825T^3 - 0.0117T^2 + 0.608T - 8.185
 * return Tscale * 1000
*/
static u64 get_temperature_scale(int cluster, unsigned long temp)
{
	u64 t_scale = 0;
	struct scale_coeff *coeff = &cluster_data[cluster].temp_scale;

	t_scale = coeff->scale_a * temp * temp * temp
			+ coeff->scale_b * temp * temp
			+ coeff->scale_c * temp
			+ coeff->scale_d;

	return t_scale / 10000;
}

/*
 * Vscale = 33.31V^3 - 73.25V^2 + 54.44V - 12.81
 * return Vscale * 1000
 */
static u64 get_voltage_scale(int cluster, unsigned long u_volt)
{
	unsigned long m_volt = u_volt / 1000;
	u64 v_scale = 0;
	struct scale_coeff *coeff = &cluster_data[cluster].voltage_scale;

	v_scale = coeff->scale_a * m_volt * m_volt * m_volt / 1000
			+ coeff->scale_b * m_volt * m_volt
			+ coeff->scale_c * m_volt * 1000
			+ coeff->scale_d * 1000 * 1000;

	return v_scale / 100000;
}

/* voltage in uV and temperature in mC */
static u32 platform_get_static_power(cpumask_t *cpumask, unsigned long u_volt,
			    unsigned long milli_temp)
{
	u64 t_scale, v_scale;
	unsigned long mw_leakage;
	int cpu_coeff;
	u32 nr_cpus = cpumask_weight(cpumask);
	enum cluster_type cluster =
		topology_physical_package_id(cpumask_any(cpumask));

	if (cluster >= NUM_CLUSTERS)
		return 0;

	/* get coeff * 10 */
	cpu_coeff = get_cpu_static_power_coeff(cluster);
	/* get Tscale * 1000 */
	t_scale = get_temperature_scale(cluster, milli_temp / 1000);
	/* get Vscale * 1000 */
	v_scale = get_voltage_scale(cluster, u_volt);

	mw_leakage = nr_cpus * (cpu_coeff * t_scale * v_scale) / 10000000;

	if (nr_cpus) {
		int cache_coeff = get_cache_static_power_coeff(cluster);
		/* cache leakage */
		mw_leakage += (cache_coeff * v_scale * t_scale) / 10000000;
	}

	return mw_leakage;
}

int cpufreq_get_static_power(cpumask_t *cpumask, int interval,unsigned long voltage, u32 *power)
{
	*power = 0;
	return 0;
}

static int cpufreq_online_everything(struct thermal_cooling_device *cdev)
{
	struct cpufreq_cooling_device *cpufreq_device = cdev->devdata;
	unsigned int num_cpus = cpumask_weight(&cpufreq_device->allowed_cpus);

	if (cpufreq_device->hotplug_refractory_period) {
		cpufreq_device->online_data.rounds = cpufreq_device->hotplug_refractory_period;
		cpufreq_device->online_data.target_online_cpus = num_cpus *
			cpufreq_device->hotplug_refractory_period;
		set_online_cpus(cpufreq_device, num_cpus);
	}

	return 0;
}

/* Bind cpufreq callbacks to thermal cooling device ops */
static struct thermal_cooling_device_ops cpufreq_cooling_ops = {
	.get_max_state = cpufreq_get_max_state,
	.get_cur_state = cpufreq_get_cur_state,
	.set_cur_state = cpufreq_set_cur_state,
	.online_everything = cpufreq_online_everything,
};

/* Notifier for cpufreq policy change */
static struct notifier_block thermal_cpufreq_notifier_block = {
	.notifier_call = cpufreq_thermal_notifier,
};

static unsigned int find_next_max(struct cpufreq_frequency_table *table,
				  unsigned int prev_max)
{
	struct cpufreq_frequency_table *pos;
	unsigned int max = 0;

	cpufreq_for_each_valid_entry(pos, table) {
		if (pos->frequency > max && pos->frequency < prev_max)
			max = pos->frequency;
	}

	return max;
}

static void create_hotplug_debugfs(struct cpufreq_cooling_device *cpufreq_dev,
				   char *dev_name)
{
	struct dentry *cpu_cdev_d, *dentry_f;

	cpu_cdev_d = debugfs_create_dir(dev_name, NULL);
	if (IS_ERR_OR_NULL(cpu_cdev_d)) {
		pr_warn("unable to create debugfs directory for the cpu cooling device %s\n",
			dev_name);
		return;
	}

	dentry_f = debugfs_create_u32("hotplug_period", S_IWUSR | S_IRUGO,
				      cpu_cdev_d,
				      &cpufreq_dev->hotplug_refractory_period);
	if (IS_ERR_OR_NULL(dentry_f))
		pr_warn("Unable to create debugfs file: hotplug_period\n");
}

/**
 * __cpufreq_cooling_register - helper function to create cpufreq cooling device
 * @np: a valid struct device_node to the cooling device device tree node
 * @clip_cpus: cpumask of cpus where the frequency constraints will happen.
 * Normally this should be same as cpufreq policy->related_cpus.
 * @hotplug_period: periods to wait before hotplugging CPUs
 * @capacitance: dynamic power coefficient for these cpus
 * @plat_static_func: function to calculate the static power consumed by these
 *                    cpus (optional)
 *
 * This interface function registers the cpufreq cooling device with the name
 * "thermal-cpufreq-%x". This api can support multiple instances of cpufreq
 * cooling devices. It also gives the opportunity to link the cooling device
 * with a device tree node, in order to bind it via the thermal DT code.
 *
 * Return: a valid struct thermal_cooling_device pointer on success,
 * on failure, it returns a corresponding ERR_PTR().
 */
static struct thermal_cooling_device *
__cpufreq_cooling_register(struct device_node *np,
			   const struct cpumask *clip_cpus, u32 policy,
			   u32 cluster, get_static_t plat_static_func)
{
	struct thermal_cooling_device *cool_dev;
	struct cpufreq_cooling_device *cpufreq_dev;
	char dev_name[THERMAL_NAME_LENGTH];
	struct cpufreq_frequency_table *pos, *table;
	unsigned int freq, i, num_cpus;
	int ret;

	table = cpufreq_frequency_get_table(cpumask_first(clip_cpus));
	if (!table) {
		pr_debug("%s: CPUFreq table not found\n", __func__);
		return ERR_PTR(-EPROBE_DEFER);
	}

	cpufreq_dev = kzalloc(sizeof(*cpufreq_dev), GFP_KERNEL);
	if (!cpufreq_dev)
		return ERR_PTR(-ENOMEM);

	num_cpus = cpumask_weight(clip_cpus);
	cpufreq_dev->time_in_idle = kcalloc(num_cpus,
					    sizeof(*cpufreq_dev->time_in_idle),
					    GFP_KERNEL);
	if (!cpufreq_dev->time_in_idle) {
		cool_dev = ERR_PTR(-ENOMEM);
		goto free_cdev;
	}

	cpufreq_dev->time_in_idle_timestamp =
		kcalloc(num_cpus, sizeof(*cpufreq_dev->time_in_idle_timestamp),
			GFP_KERNEL);
	if (!cpufreq_dev->time_in_idle_timestamp) {
		cool_dev = ERR_PTR(-ENOMEM);
		goto free_time_in_idle;
	}

	/* Find max levels */
	cpufreq_for_each_valid_entry(pos, table)
		cpufreq_dev->max_level++;

	cpufreq_dev->freq_table = kmalloc(sizeof(*cpufreq_dev->freq_table) *
					  cpufreq_dev->max_level, GFP_KERNEL);
	if (!cpufreq_dev->freq_table) {
		cool_dev = ERR_PTR(-ENOMEM);
		goto free_time_in_idle_timestamp;
	}

	/* max_level is an index, not a counter */
	cpufreq_dev->max_level--;

	cpumask_copy(&cpufreq_dev->allowed_cpus, clip_cpus);

	if (policy) {
		cpufreq_cooling_ops.get_requested_power =
			cpufreq_get_requested_power;
		cpufreq_cooling_ops.state2power = cpufreq_state2power;
		cpufreq_cooling_ops.power2state = cpufreq_power2state;
		cpufreq_dev->plat_get_static_power = platform_get_static_power;

		ret = build_dyn_power_table(cpufreq_dev, cluster);
		if (ret) {
			cool_dev = ERR_PTR(ret);
			goto free_table;
		}
	}

	ret = get_idr(&cpufreq_idr, &cpufreq_dev->id);
	if (ret) {
		cool_dev = ERR_PTR(ret);
		goto free_table;
	}

	snprintf(dev_name, sizeof(dev_name), "thermal-cpufreq-%d",
		 cpufreq_dev->id);

	cool_dev = thermal_of_cooling_device_register(np, dev_name, cpufreq_dev,
						      &cpufreq_cooling_ops);
	if (IS_ERR(cool_dev))
		goto remove_idr;

	/* Fill freq-table in descending order of frequencies */
	for (i = 0, freq = -1; i <= cpufreq_dev->max_level; i++) {
		freq = find_next_max(table, freq);
		cpufreq_dev->freq_table[i] = freq;

		/* Warn for duplicate entries */
		if (!freq)
			pr_warn("%s: table has duplicate entries\n", __func__);
		else
			pr_debug("%s: freq:%u KHz\n", __func__, freq);
	}

	cpufreq_dev->cpufreq_val = cpufreq_dev->freq_table[0];
	cpufreq_dev->cool_dev = cool_dev;
	cpufreq_dev->hotplug_refractory_period = cluster_data[cluster].hotplug_period;
	INIT_WORK(&cpufreq_dev->assassin, kill_cpu);
	cpufreq_dev->assassin_data.pending_kill = false;

	create_hotplug_debugfs(cpufreq_dev, dev_name);

	mutex_lock(&cooling_cpufreq_lock);

	/* Register the notifier for first cpufreq cooling device */
	if (list_empty(&cpufreq_dev_list))
		cpufreq_register_notifier(&thermal_cpufreq_notifier_block,
					  CPUFREQ_POLICY_NOTIFIER);
	list_add(&cpufreq_dev->node, &cpufreq_dev_list);

	mutex_unlock(&cooling_cpufreq_lock);

	return cool_dev;

remove_idr:
	release_idr(&cpufreq_idr, cpufreq_dev->id);
free_table:
	kfree(cpufreq_dev->freq_table);
free_time_in_idle_timestamp:
	kfree(cpufreq_dev->time_in_idle_timestamp);
free_time_in_idle:
	kfree(cpufreq_dev->time_in_idle);
free_cdev:
	kfree(cpufreq_dev);

	return cool_dev;
}

/**
 * cpufreq_cooling_register - function to create cpufreq cooling device.
 * @clip_cpus: cpumask of cpus where the frequency constraints will happen.
 *
 * This interface function registers the cpufreq cooling device with the name
 * "thermal-cpufreq-%x". This api can support multiple instances of cpufreq
 * cooling devices.
 *
 * Return: a valid struct thermal_cooling_device pointer on success,
 * on failure, it returns a corresponding ERR_PTR().
 */
struct thermal_cooling_device *
cpufreq_cooling_register(const struct cpumask *clip_cpus)
{
	return __cpufreq_cooling_register(NULL, clip_cpus, false, 0, NULL);
}
EXPORT_SYMBOL_GPL(cpufreq_cooling_register);

/**
 * of_cpufreq_cooling_register - function to create cpufreq cooling device.
 * @np: a valid struct device_node to the cooling device device tree node
 * @clip_cpus: cpumask of cpus where the frequency constraints will happen.
 *
 * This interface function registers the cpufreq cooling device with the name
 * "thermal-cpufreq-%x". This api can support multiple instances of cpufreq
 * cooling devices. Using this API, the cpufreq cooling device will be
 * linked to the device tree node provided.
 *
 * Return: a valid struct thermal_cooling_device pointer on success,
 * on failure, it returns a corresponding ERR_PTR().
 */
struct thermal_cooling_device *
of_cpufreq_cooling_register(struct device_node *np,
			    const struct cpumask *clip_cpus)
{
	if (!np)
		return ERR_PTR(-EINVAL);

	return __cpufreq_cooling_register(np, clip_cpus, false, 0, NULL);
}
EXPORT_SYMBOL_GPL(of_cpufreq_cooling_register);

/**
 * cpufreq_power_cooling_register() - create cpufreq cooling device with power extensions
 * @clip_cpus:	cpumask of cpus where the frequency constraints will happen
 * @capacitance:	dynamic power coefficient for these cpus
 * @hotplug_period: periods to wait before hotplugging CPUs
 * @plat_static_func:	function to calculate the static power consumed by these
 *			cpus (optional)
 *
 * This interface function registers the cpufreq cooling device with
 * the name "thermal-cpufreq-%x".  This api can support multiple
 * instances of cpufreq cooling devices.  Using this function, the
 * cooling device will implement the power extensions by using a
 * simple cpu power model.  The cpus must have registered their OPPs
 * using the OPP library.
 *
 * An optional @plat_static_func may be provided to calculate the
 * static power consumed by these cpus.  If the platform's static
 * power consumption is unknown or negligible, make it NULL.
 *
 * Return: a valid struct thermal_cooling_device pointer on success,
 * on failure, it returns a corresponding ERR_PTR().
 */
struct thermal_cooling_device *
cpufreq_power_cooling_register(const struct cpumask *clip_cpus,
			       u32 hotplug_period, u32 capacitance,
			       get_static_t plat_static_func)
{
	return __cpufreq_cooling_register(NULL, clip_cpus, hotplug_period,
					  capacitance, plat_static_func);
}
EXPORT_SYMBOL(cpufreq_power_cooling_register);

/**
 * of_cpufreq_power_cooling_register() - create cpufreq cooling device with power extensions
 * @np:	a valid struct device_node to the cooling device device tree node
 * @clip_cpus:	cpumask of cpus where the frequency constraints will happen
 * @hotplug_period: periods to wait before hotplugging CPUs
 * @capacitance:	dynamic power coefficient for these cpus
 * @plat_static_func:	function to calculate the static power consumed by these
 *			cpus (optional)
 *
 * This interface function registers the cpufreq cooling device with
 * the name "thermal-cpufreq-%x".  This api can support multiple
 * instances of cpufreq cooling devices.  Using this API, the cpufreq
 * cooling device will be linked to the device tree node provided.
 * Using this function, the cooling device will implement the power
 * extensions by using a simple cpu power model.  The cpus must have
 * registered their OPPs using the OPP library.
 *
 * An optional @plat_static_func may be provided to calculate the
 * static power consumed by these cpus.  If the platform's static
 * power consumption is unknown or negligible, make it NULL.
 *
 * Return: a valid struct thermal_cooling_device pointer on success,
 * on failure, it returns a corresponding ERR_PTR().
 */
struct thermal_cooling_device *
of_cpufreq_power_cooling_register(struct device_node *np,
				  const struct cpumask *clip_cpus,
				  u32 hotplug_period, u32 capacitance,
				  get_static_t plat_static_func)
{
	if (!np)
		return ERR_PTR(-EINVAL);

	return __cpufreq_cooling_register(np, clip_cpus, hotplug_period,
					  capacitance, plat_static_func);
}
EXPORT_SYMBOL(of_cpufreq_power_cooling_register);

/**
 * cpufreq_cooling_unregister - function to remove cpufreq cooling device.
 * @cdev: thermal cooling device pointer.
 *
 * This interface function unregisters the "thermal-cpufreq-%x" cooling device.
 */
void cpufreq_cooling_unregister(struct thermal_cooling_device *cdev)
{
	struct cpufreq_cooling_device *cpufreq_dev;

	if (!cdev)
		return;

	cpufreq_dev = cdev->devdata;
	mutex_lock(&cooling_cpufreq_lock);
	list_del(&cpufreq_dev->node);

	/* Unregister the notifier for the last cpufreq cooling device */
	if (list_empty(&cpufreq_dev_list))
		cpufreq_unregister_notifier(&thermal_cpufreq_notifier_block,
					    CPUFREQ_POLICY_NOTIFIER);
	mutex_unlock(&cooling_cpufreq_lock);

	thermal_cooling_device_unregister(cpufreq_dev->cool_dev);
	release_idr(&cpufreq_idr, cpufreq_dev->id);
	kfree(cpufreq_dev->time_in_idle_timestamp);
	kfree(cpufreq_dev->time_in_idle);
	kfree(cpufreq_dev->freq_table);
	kfree(cpufreq_dev);
}
EXPORT_SYMBOL_GPL(cpufreq_cooling_unregister);

struct cpufreq_cooling_device *cooling_devices_get_zone_by_name(const char *name)
{
	struct cpufreq_cooling_device *cpufreq_dev= NULL, *ref = ERR_PTR(-EINVAL);
	unsigned int found = 0;

	if (!name)
		goto exit;

	mutex_lock(&cooling_cpufreq_lock);
	list_for_each_entry(cpufreq_dev, &cpufreq_dev_list, node) {
		if (!strncasecmp(name, cpufreq_dev->info->type, THERMAL_NAME_LENGTH)) {
			found++;
			ref = cpufreq_dev;
		}
	}
	mutex_unlock(&cooling_cpufreq_lock);

	/* nothing has been found, thus an error code for it */
	if (found == 0)
		ref = ERR_PTR(-ENODEV);
	else if (found > 1)
		/* Success only when an unique zone is found */
		ref = ERR_PTR(-EEXIST);

exit:
	return ref;
}

#if defined(CONFIG_OTP_SPRD_AP_EFUSE)
/* return (leakage * 10) */
static u64 get_leak_base(int cluster, int val, int *coeff)
{
	int i;
	u64 leak_base;

	if (cluster)
		leak_base = ((val>>16) & 0x1F) + 1;
	else
		leak_base = ((val>>11) & 0x1F) + 1;

	/* (LIT_LEAK[4:0]+1) x 2mA x 0.85V x 18.69% */
	for (i = 0; i < 3; i++)
		leak_base = leak_base * coeff[i];

	leak_base = leak_base / 100000;

	return leak_base;
}
#endif

static int sprd_get_power_model_coeff(struct device_node *np,
	struct cluster_power_coefficients *power_coeff, int cluster)
{
	int ret;
#if defined(CONFIG_OTP_SPRD_AP_EFUSE)
	int val = 0;
	int efuse_block = -1;
	u32 coeff[3];
#endif
	if (!np) {
		pr_err("device node not found\n");
		return -EINVAL;
	}

#if defined(CONFIG_OTP_SPRD_AP_EFUSE)
	ret = of_property_read_s32(np, "sprd,efuse-block15", &efuse_block);
	if (ret) {
		pr_err("fail to get cooling devices efuse_block\n");
		efuse_block = -1;
	}

	if (efuse_block >= 0)
		val = sprd_ap_efuse_read(efuse_block);

	SPRDCPU_DEBUG("sci_efuse_leak --val : %x\n", val);
	if (val) {
		ret = of_property_read_u32_array(np,
			"sprd,leak-core", (u32 *)coeff, 3);
		if (ret) {
			pr_err("fail to get cooling devices leak-core-coeff\n");
			return -EINVAL;
		}

		power_coeff->leak_core_base =
			get_leak_base(cluster, val, (int *)coeff);

		ret = of_property_read_u32_array(np,
			"sprd,leak-cluster", (u32 *)coeff, 3);
		if (ret) {
			pr_err("fail to get cooling devices leak-cluster-coeff\n");
			return -EINVAL;
		}

		power_coeff->leak_cluster_base =
			get_leak_base(cluster, val, (int *)coeff);
	}
#endif

	ret = of_property_read_u32_array(np, "sprd,temp-scale",
				(u32 *)&power_coeff->temp_scale,
				sizeof(struct scale_coeff) / sizeof(int));
	if (ret) {
		pr_err("fail to get cooling devices temp-scale\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np, "sprd,volt-scale",
				(u32 *)&power_coeff->voltage_scale,
				sizeof(struct scale_coeff) / sizeof(int));
	if (ret) {
		pr_err("fail to get cooling devices voltage-scale\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np, "sprd,dynamic-core",
				(u32 *)&power_coeff->core_coeff,
				sizeof(struct dyn_power_coeff) / sizeof(int));
	if (ret) {
		pr_err("fail to get cooling devices dynamic-core-coeff\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np, "sprd,dynamic-cluster",
				(u32 *)&power_coeff->cluster_coeff,
				sizeof(struct dyn_power_coeff) / sizeof(int));
	if (ret) {
		pr_err("fail to get cooling devices dynamic-cluster-coeff\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "sprd,hotplug-period",
				&power_coeff->hotplug_period);
	if (ret)
		pr_err("fail to get cooling devices efuse_block\n");

	return 0;
}

static int get_cooling_devices_dt_data(struct device_node *np,
	struct thermal_cooling_info_t *info)
{
	int ret;
#if defined(CONFIG_OTP_SPRD_AP_EFUSE)
	int val = 0;
	int efuse_block = -1;
#endif

	if (!np) {
		pr_err("device node not found\n");
		return -EINVAL;
	}

	info->cluster = of_alias_get_id(np, "cooling-device");
	if (info->cluster == -ENODEV) {
		pr_err("fail to get cooling devices id\n");
		info->cluster = 0;
	}

#if defined(CONFIG_OTP_SPRD_AP_EFUSE)
	ret = of_property_read_s32(np, "sprd,efuse-block7", &efuse_block);
	if(ret){
		pr_err("fail to get cooling devices efuse_block\n");
		efuse_block = -1;
	}

	if (efuse_block >= 0)
		val = sprd_ap_efuse_read(efuse_block);

	SPRDCPU_DEBUG("sci_efuse_Dhryst_binning_get--val : %d\n", val);
	if ((val < 28) && (val >= 22)){
		info->binning = 0;
	}else{
		info->binning = 1;
	}
	SPRDCPU_DEBUG("binning : %d\n", info->binning);
#else
	info->binning = 0;
#endif

	ret = sprd_get_power_model_coeff(np,
		&cluster_data[info->cluster], info->cluster);
	if (ret) {
		pr_err("fail to get power model coeff !\n");
		return -EINVAL;
	}
	return 0;
}

static int cpu_cooling_pm_notify(struct notifier_block *nb,
				unsigned long mode, void *_unused)
{
	switch (mode) {
	case PM_HIBERNATION_PREPARE:
	case PM_RESTORE_PREPARE:
	case PM_SUSPEND_PREPARE:
		atomic_set(&in_suspend, 1);
		break;
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		atomic_set(&in_suspend, 0);
		break;
	default:
		break;
	}
	return 0;
}

static struct notifier_block cpu_cooling_pm_nb = {
	.notifier_call = cpu_cooling_pm_notify,
};

int __init parse_cooling_devices_zones(void)
{
	struct device_node *np, *child;
	int ret;
	int cpu = 0;
	struct device *dev = NULL;
	int policy = 0;
	int result;
	struct thermal_cooling_device *cool_dev = NULL;
	struct cpufreq_cooling_device *cpufreq_dev = NULL;

	np = of_find_node_by_name(NULL, "cooling-devices");
	if (!np) {
		pr_err("unable to find thermal zones\n");
		return 0; /* Run successfully on systems without thermal DT */
	}

	/*ipa is only policy*/
	ret = of_property_match_string(np, "policy", "ipa");
	if (ret == 0) {
		policy = 1;
	} else {
		pr_err("fail to get policy\n");
		policy = 0;
	}

	for_each_child_of_node(np, child) {
		struct thermal_cooling_info_t *info = NULL;
		struct cpumask clip_cpus;
		cpumask_clear(&clip_cpus);

		/* Check whether child is enabled or not */
		if (!of_device_is_available(child))
			continue;

		info = kzalloc(sizeof(*info), GFP_KERNEL);
		if (!info) {
			pr_err("could not allocate memory for info data\n");
			return -1;
		}

		ret = get_cooling_devices_dt_data(child, info);
		if (ret){
			pr_err("Fail to get cpu cooling dts data\n");
			kfree(info);
			info = NULL;
			continue;
		}
		strlcpy(info->type, child->name ? : "", sizeof(info->type));
		info->cooling_state = 0;
		info->enable = 0;
		info->param.state = 0;
		info->param.max_core = 0;
		info->param.limit_freq = 0;

		for_each_possible_cpu(cpu) {
			if (((cpu >= 0) && (cpu < MAX_LIT_CPU_MUN) && (info->cluster == CLUSTER_LITTLE )) ||
					((cpu >= MAX_LIT_CPU_MUN) && (cpu < MAX_CPU_STATE) && (info->cluster == CLUSTER_BIG))) {
				cpumask_set_cpu(cpu, &clip_cpus);
			} else {
				SPRDCPU_DEBUG("cpu id: %d > %d\n",
						cpu, MAX_CPU_STATE);
			}
		}

		for_each_cpu(cpu, &clip_cpus) {
			dev = get_cpu_device(cpu);
			if (!dev) {
				pr_err("No cpu device for cpu %d\n", cpu);
				continue;
			}
			if (dev_pm_opp_get_opp_count(dev) > 0)
				break;
		}

		cool_dev = __cpufreq_cooling_register(child,
								  &clip_cpus,
								  policy,
								  info->cluster,
								  NULL);
		if (IS_ERR(cool_dev)) {
			pr_err("Error registering cooling device\n");
			kfree(info);
			info = NULL;
			continue;
		}

		info->cdev = cool_dev;
		cpufreq_dev = cool_dev->devdata;
		cpufreq_dev->cpu_dev = dev;
		cpufreq_dev->info = info;
		cpumask_clear(&cpufreq_dev->hotplug_cpus);

		sprd_cpu_creat_attr(&cool_dev->device);
	}

	result = register_pm_notifier(&cpu_cooling_pm_nb);
	if (result)
		pr_warn("Thermal: Can not register suspend notifier, return %d\n",
			result);

	return 0;
}

int __init destroy_cooling_devices_zones(void)
{
	struct device_node *np, *child;

	unregister_pm_notifier(&cpu_cooling_pm_nb);

	np = of_find_node_by_name(NULL, "cooling-devices");
	if (!np) {
		pr_err("unable to find thermal zones\n");
		return -ENODEV;
	}

	for_each_child_of_node(np, child) {
		struct cpufreq_cooling_device *cpufreq_dev;

		/* Check whether child is enabled or not */
		if (!of_device_is_available(child))
			continue;

		cpufreq_dev = cooling_devices_get_zone_by_name(child->name);
		if (IS_ERR(cpufreq_dev))
			continue;

		sprd_cpu_remove_attr(&cpufreq_dev->cool_dev->device);

		cpufreq_cooling_unregister(cpufreq_dev->cool_dev);
		kfree(cpufreq_dev->info);
		cpufreq_dev->info = NULL;
	}

	return 0;
}

static int __init sprd_cooling_devices_init(void)
{
	return parse_cooling_devices_zones();
}

static void __exit sprd_cooling_devices_exit(void)
{
	destroy_cooling_devices_zones();
}

late_initcall(sprd_cooling_devices_init);
module_exit(sprd_cooling_devices_exit);
