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
#include <linux/devfreq_cooling.h>
#include <linux/thermal.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sprd_otp.h>
#include <linux/debugfs.h>
#include <linux/printk.h>
#include <linux/sprd_gpu_device.h>

#define FALLBACK_STATIC_TEMPERATURE 55000
#define GPU_CLUSTER 0
#define NP_NAME_LEN 20

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
	struct thermal_cooling_device *gpudev;
	char devname[NP_NAME_LEN];
	int weight;
	void *devdata;
};

static struct cluster_power_coefficients *cluster_data;
static u64 *gpu_mask;

/* return (leak * 10)  */
static int get_gpu_static_power_coeff(int cluster_id)
{
	return cluster_data[cluster_id].leak_core_base;
}

/* return (leak * 10)  */
static int get_cache_static_power_coeff(int cluster_id)
{
	return cluster_data[cluster_id].leak_cluster_base;
}

static u64 get_core_dyn_power(int cluster_id,
	unsigned int freq_mhz, unsigned int voltage_mv)
{
	u64 power = 0;
	int voltage_base = cluster_data[cluster_id].core_coeff.voltage_base;
	int dyn_base = cluster_data[cluster_id].core_coeff.dynperghz;

	power = (u64)dyn_base * freq_mhz * voltage_mv * voltage_mv;
	do_div(power, voltage_base * voltage_base);
	do_div(power, 10000);

	return power;
}

static u64 get_cluster_dyn_power(int cluster_id,
	unsigned int freq_mhz, unsigned int voltage_mv)
{
	u64 power = 0;
	int voltage_base = cluster_data[cluster_id].cluster_coeff.voltage_base;
	int dyn_base = cluster_data[cluster_id].cluster_coeff.dynperghz;

	power = (u64)dyn_base * freq_mhz * voltage_mv * voltage_mv;
	do_div(power, voltage_base * voltage_base);
	do_div(power, 10000);

	pr_debug("cluster:%d cluster_dyn_power:%u freq:%u voltage:%u voltage_base:%d\n",
		cluster_id, (u32)power, freq_mhz, voltage_mv, voltage_base);

	return power;
}

/*
 *Tscale = 0.0000825T^3 - 0.0117T^2 + 0.608T - 8.185
 * return Tscale * 1000
 */
static u64 get_temperature_scale(int cluster_id, unsigned long temp)
{
	u64 t_scale = 0;
	struct scale_coeff *coeff = &cluster_data[cluster_id].temp_scale;

	t_scale = coeff->scale_a * temp * temp * temp
		+ coeff->scale_b * temp * temp
		+ coeff->scale_c * temp
		+ coeff->scale_d;

	do_div(t_scale, 10000);

	return t_scale;
}

/*
 * Vscale = 33.31V^3 - 73.25V^2 + 54.44V - 12.81
 * return Vscale * 1000
 */
static u64 get_voltage_scale(int cluster_id, unsigned long u_volt)
{
	unsigned long m_volt = u_volt / 1000;
	u64 v_scale = 0;
	struct scale_coeff *coeff = &cluster_data[cluster_id].voltage_scale;

	v_scale = coeff->scale_a * m_volt * m_volt * m_volt / 1000
		+ coeff->scale_b * m_volt * m_volt
		+ coeff->scale_c * m_volt * 1000
		+ coeff->scale_d * 1000 * 1000;

	do_div(v_scale, 100000);

	return v_scale;
}

/* voltage in uV and temperature in mC */
static unsigned long get_static_power(unsigned long u_volt)
{
	struct thermal_zone_device *thmzone;
	unsigned long t_scale, v_scale;
	u32 gpu_coeff;
	unsigned long power;
	int gpu_cluster = GPU_CLUSTER;
	int gpu_cores = 1;
	int cache_coeff = 0;
	int temperature = 0;
	int err;
	/* todo: get gpu cluster id*/

	thmzone = thermal_zone_get_zone_by_name("gpu-thmzone");

	if (thmzone) {
		err = thmzone->ops->get_temp(thmzone, &temperature);
		if (err) {
			pr_warn("Error reading temperature:%d\n", err);
			temperature = FALLBACK_STATIC_TEMPERATURE;
		}
	} else {
		temperature = FALLBACK_STATIC_TEMPERATURE;
	}

	if (gpu_mask != NULL)
		gpu_cores = hweight64(*gpu_mask);

	/* get coeff * 10 */
	gpu_coeff = get_gpu_static_power_coeff(gpu_cluster);
	/* get Tscale * 1000 */
	t_scale = get_temperature_scale(gpu_cluster, temperature / 1000);
	/* get Vscale * 1000 */
	v_scale = get_voltage_scale(gpu_cluster, u_volt);

	power = gpu_cores * (gpu_coeff * t_scale * v_scale) / 10000000;

	if (gpu_cores) {
		cache_coeff = get_cache_static_power_coeff(gpu_cluster);
		power += (cache_coeff * v_scale * t_scale) /
			10000000; /* cache leakage */
	}

	pr_debug("cluster:%d gpus:%d static_power:%lu gpu_coeff:%u\n",
		gpu_cluster, gpu_cores, power, gpu_coeff);
	pr_debug(" cache_coeff:%d t_scale:%lu v_scale:%lu\n",
		cache_coeff, t_scale, v_scale);

	return power;
}

#if defined(CONFIG_OTP_SPRD_AP_EFUSE_I)
/* return (leakage * 10) */
static u64 get_leak_base(int cluster_id, int val, int *coeff)
{
	int i;
	u64 leak_base;

	if (cluster_id)
		leak_base = ((val>>16) & 0x1F) + 1;
	else
		leak_base = ((val>>11) & 0x1F) + 1;

	/* (LIT_LEAK[4:0]+1) x 2mA x 0.85V x 18.69% */
	for (i = 0; i < 3; i++)
		leak_base = leak_base * coeff[i];
	do_div(leak_base, 100000);

	return leak_base;
}
#endif

struct thermal_cooling_device *
cluster_data_get_dev_by_name(const char *name)
{
	int i = 0;

	while (&cluster_data[i] != NULL) {
		if (!strncmp(name, cluster_data[i].devname, strlen(name)))
			return cluster_data[i].gpudev;
		i++;
	}

	return NULL;
}

static unsigned long get_dynamic_power(unsigned long freq,
		unsigned long voltage)
{
	u64 power;
	int gpu_cluster = GPU_CLUSTER;

	power = get_core_dyn_power(gpu_cluster, freq, voltage);
	power += get_cluster_dyn_power(gpu_cluster, freq, voltage);
	return power;
}

struct devfreq_cooling_power power_model_ops = {
	.get_static_power = get_static_power,
	.get_dynamic_power = get_dynamic_power,
};

static int sprd_get_power_model_coeff(struct device_node *np,
		struct cluster_power_coefficients *power_coeff, int cluster_id)
{
	int ret;
	int val = 0;
#if defined(CONFIG_OTP_SPRD_AP_EFUSE_I)
	int efuse_block = -1;
	int coeff[3];
#endif
	if (!np) {
		pr_err("device node not found\n");
		return -EINVAL;
	}

#if defined(CONFIG_OTP_SPRD_AP_EFUSE_I)
	ret = of_property_read_u32(np, "sprd,efuse-block15", &efuse_block);
	if (ret) {
		pr_err("fail to get cooling devices efuse_block\n");
		efuse_block = -1;
	}

	if (efuse_block >= 0)
		val = sprd_ap_efuse_read(efuse_block);

	pr_debug("sci_efuse_leak --val : %x\n", val);
	if (val) {
		ret = of_property_read_u32_array(np,
				"sprd,leak-core", coeff, 3);
		if (ret) {
			pr_err("fail to get cooling devices leak-core-coeff\n");
			return -EINVAL;
		}

		power_coeff->leak_core_base =
			get_leak_base(cluster_id, val, coeff);

		ret = of_property_read_u32_array(np,
				"sprd,leak-cluster", coeff, 3);
		if (ret) {
			pr_err("fail to get cooling devices leak-cluster-coeff\n");
			return -EINVAL;
		}

		power_coeff->leak_cluster_base =
			get_leak_base(cluster_id, val, coeff);
	}
#endif

	if (!val) {
		ret = of_property_read_u32(np, "sprd,core-base",
				&power_coeff->leak_core_base);
		if (ret) {
			pr_err("fail to get def cool-dev leak-core-base\n");
			return -EINVAL;
		}

		ret = of_property_read_u32(np, "sprd,cluster-base",
				&power_coeff->leak_cluster_base);
		if (ret) {
			pr_err("fail to get def cool-dev leak-cluster-base\n");
			return -EINVAL;
		}
	}

	ret = of_property_read_u32_array(np, "sprd,temp-scale",
			(int *)&power_coeff->temp_scale,
			sizeof(struct scale_coeff) / sizeof(int));
	if (ret) {
		pr_err("fail to get cooling devices temp-scale\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np, "sprd,voltage-scale",
			(int *)&power_coeff->voltage_scale,
			sizeof(struct scale_coeff) / sizeof(int));
	if (ret) {
		pr_err("fail to get cooling devices voltage-scale\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np, "sprd,dynamic-core",
			(int *)&power_coeff->core_coeff,
			sizeof(struct dyn_power_coeff) / sizeof(int));
	if (ret) {
		pr_err("fail to get cooling devices dynamic-core-coeff\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np, "sprd,dynamic-cluster",
			(int *)&power_coeff->cluster_coeff,
			sizeof(struct dyn_power_coeff) / sizeof(int));
	if (ret) {
		pr_err("fail to get cooling devices dynamic-cluster-coeff\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np,
		"sprd,hotplug-period", &power_coeff->hotplug_period);
	if (ret)
		pr_err("fail to get cooling devices efuse_block\n");

	return 0;
}

int create_gpu_cooling_device(struct devfreq *gpudev, u64 *mask)
{
	struct device_node *np, *child;
	struct thermal_cooling_device *devfreq_cooling;
	int cluster_count;
	int ret = 0;

	if (gpudev == NULL || mask == NULL) {
		pr_err("params is not complete!\n");
		return -ENOMEM;
	}

	gpu_mask = mask;

	np = of_find_node_by_name(NULL, "gpu-cooling-devices");
	if (!np) {
		pr_err("unable to find thermal zones\n");
		return 0; /* Run successfully on systems without thermal DT */
	}

	cluster_count = of_get_child_count(np);

	cluster_data = kcalloc(cluster_count,
		sizeof(struct cluster_power_coefficients), GFP_KERNEL);
	if (cluster_data == NULL) {
		ret = -ENOMEM;
		goto ERR_RET;
	}

	for_each_child_of_node(np, child) {
		int cluster_id;

		if (!of_device_is_compatible(child, "sprd,mali-power-model")) {
			pr_err("power_model incompatible\n");
			goto free_cluster;
		}

		/* Check whether child is enabled or not */
		if (!of_device_is_available(child))
			continue;

		cluster_id = of_alias_get_id(child, "gpu-cooling");
		if (cluster_id == -ENODEV) {
			pr_err("fail to get cooling devices id\n");
			goto free_cluster;
		}

		ret = sprd_get_power_model_coeff(child,
			&cluster_data[cluster_id], cluster_id);
		if (ret) {
			pr_err("fail to get power model coeff !\n");
			goto free_cluster;
		}

		devfreq_cooling = of_devfreq_cooling_register_power(child,
			gpudev, &power_model_ops);

		strlcpy(cluster_data[cluster_id].devname,
			child->name, strlen(child->name));
		cluster_data[cluster_id].gpudev = devfreq_cooling;

		if (IS_ERR_OR_NULL(devfreq_cooling)) {
			ret = PTR_ERR(devfreq_cooling);
			pr_err("Failed to register cool-dev (%d)\n", ret);
				goto free_cluster;
		} else
			ret = 0;
	}

	return ret;

free_cluster:
	kfree(cluster_data);
	cluster_data = NULL;
ERR_RET:
	return ret;
}
EXPORT_SYMBOL_GPL(create_gpu_cooling_device);

int destroy_gpu_cooling_device(void)
{
	struct device_node *np, *child;

	np = of_find_node_by_name(NULL, "gpu-cooling-devices");
	if (!np) {
		pr_err("unable to find thermal zones\n");
		return -ENODEV;
	}

	for_each_child_of_node(np, child) {
		struct thermal_cooling_device *cdev;

		cdev = cluster_data_get_dev_by_name(child->name);
		if (IS_ERR(cdev))
			continue;
		devfreq_cooling_unregister(cdev);
	}

	kfree(cluster_data);
	cluster_data = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(destroy_gpu_cooling_device);
