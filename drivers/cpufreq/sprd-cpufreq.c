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
#undef DEBUG_SPRD_CPUFREQ
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sprd_otp.h>
#include <linux/of_platform.h>
#include <linux/sprd-cpufreq.h>
#include <linux/sprd-cpufreqhw.h>

#ifdef P_DBG
#undef P_DBG
#endif
#ifdef P_INF
#undef P_INF
#endif
#ifdef P_WARN
#undef P_WARN
#endif
#ifdef P_ERR
#undef P_ERR
#endif

#ifdef DEBUG_SPRD_CPUFREQ
#define P_DBG(fmt, ...) pr_emerg("%s: " fmt, __func__, ##__VA_ARGS__)
#define P_INF pr_emerg
#define P_WARN pr_emerg
#define P_ERR pr_emerg
#else
#define P_DBG(fmt, ...) pr_debug("%s: " fmt, __func__, ##__VA_ARGS__)
#define P_INF pr_info
#define P_WARN pr_warn
#define P_ERR pr_err
#endif

/* Default voltage_tolerance */
#define DEF_VOLT_TOL		0
/* Regulator Supply */
#define CORE_SUPPLY		"cpu"
/* Core Clocks */
#define CORE_CLK	"core_clk"
#define LOW_FREQ_CLK_PARENT	"low_freq_clk_parent"
#define HIGH_FREQ_CLK_PARENT	"high_freq_clk_parent"
/*0-cluster0; 1-custer1;  2/3-cci or fcm*/
#define SPRD_CPUFREQ_MAX_MODULE  4
/*0-cluster0; 1-custer1*/
#define SPRD_CPUFREQ_MAX_CLUSTER  2
#define SPRD_MAX_CPUS_EACH_CLUSTER  4
#define SPRD_CPUFREQ_MAX_FREQ_VOLT 10
#define SPRD_CPUFREQ_MAX_TEMP  4
#define SPRD_CPUFREQ_TEMP_FALL_HZ  (2*HZ)
#define SPRD_CPUFREQ_TEMP_UPDATE_HZ  (HZ/2)
#define SPRD_CPUFREQ_TEMP_MAX  200
#define SPRD_CPUFREQ_TEMP_MIN  (-200)
#define SPRD_CPUFREQ_DRV_BOOST_DURATOIN	(60ul*HZ)

#define sprd_cpufreq_data(cpu) \
	cpufreq_datas[topology_physical_package_id(cpu)]
#define is_big_cluster(cpu) (topology_physical_package_id(cpu) != 0)
#ifndef REG_AON_APB_RES_REG4
#define REG_AON_APB_RES_REG4	(0x3098)
#endif
#ifndef REG_AON_APB_RES_REG5
#define REG_AON_APB_RES_REG5	(0x309C)
#endif
static int cpu_gpu_flag;
static struct regmap *aon_apb_reg_base;
struct regulator *gpu_cpu_reg;

/* Sprd bining surpport */
#define SPRD_BINNING_MAX    4
#define SPRD_BINNING_MIN    0

struct sprd_cpufreq_group {
	unsigned long freq;/*HZ*/
	unsigned long volt;/*uV*/
};
struct sprd_cpufreq_driver_data {
	unsigned int cluster;
	unsigned int online;
	struct mutex *volt_lock;
	struct device *cpu_dev;
	struct regulator *reg;
	/*const means clk_high is constant clk source*/
	unsigned int clk_high_freq_const;
	unsigned int clk_en;
	/*first cpu clk*/
	struct clk *clk;
	/*all cpus clk in cluster*/
	struct clk *clks[SPRD_MAX_CPUS_EACH_CLUSTER];
	struct clk *clk_low_freq_p;
	struct clk *clk_high_freq_p;
	unsigned long clk_low_freq_p_max;
	unsigned long clk_high_freq_p_max;
	/* voltage tolerance in percentage */
	unsigned int volt_tol;
	unsigned int volt_tol_var;
	/*bitX = 1 means sub_cluster exists*/
	unsigned int sub_cluster_bits;
	/*volt requested by this cluster*/
	unsigned long volt_req;/*uv*/
	unsigned int volt_share_hosts_bits;
	unsigned int volt_share_masters_bits;
	unsigned int volt_share_slaves_bits;
	unsigned long freq_req;/*hz*/
	unsigned int freq_sync_hosts_bits;
	unsigned int freq_sync_slaves_bits;
	unsigned int freqvolts;
	/*max freq is on freqvolt[0]*/
	struct sprd_cpufreq_group
		freqvolt[SPRD_CPUFREQ_MAX_FREQ_VOLT];
	unsigned int temp_max_freq;
	int temp_list[SPRD_CPUFREQ_MAX_TEMP];
	int temp_max;
	int temp_top;
	int temp_now;
	int temp_bottom;
	unsigned long temp_fall_time;
};

static struct sprd_cpufreq_driver_data
	*cpufreq_datas[SPRD_CPUFREQ_MAX_MODULE];
static unsigned long boot_done_timestamp;
static int boost_mode_flag = 1;
static struct cpufreq_driver sprd_cpufreq_driver;
static int sprd_cpufreq_set_boost(int state);
static int sprd_cpufreq_set_target(
	struct sprd_cpufreq_driver_data *cpufreq_data,
	unsigned int idx, bool force);

DEFINE_SEMAPHORE(cpu_gpu_sync_volt_sem);

void down_voltage_sem(void)
{
    down(&cpu_gpu_sync_volt_sem);
}
EXPORT_SYMBOL_GPL(down_voltage_sem);

void up_voltage_sem(void)
{
    up(&cpu_gpu_sync_volt_sem);
}
EXPORT_SYMBOL_GPL(up_voltage_sem);

struct regulator *get_cpu_gpu_regulator(void)
{
	if (gpu_cpu_reg == NULL)
		pr_err("%s: failed to get regulator\n", __func__);
	return gpu_cpu_reg;
}
EXPORT_SYMBOL_GPL(get_cpu_gpu_regulator);

static u32 get_gpu_target_voltage(void)
{
	u32	gpu_vdd = 0;
	if (cpu_gpu_flag == 1)
		regmap_read(aon_apb_reg_base, REG_AON_APB_RES_REG4, &gpu_vdd);
	pr_debug("%s: gpu_vdd is %dmv, cpu_gpu_flag is %d\n", __func__, gpu_vdd/1000, cpu_gpu_flag);
	return gpu_vdd;
}

static void set_cpu_target_voltage(unsigned long cpu_vdd)
{
	pr_debug("%s: cpu_vdd is %ldmv cpu_gpu_flag is %d\n", __func__, cpu_vdd/1000, cpu_gpu_flag);
	if (cpu_gpu_flag == 1)
		regmap_write(aon_apb_reg_base, REG_AON_APB_RES_REG5, cpu_vdd);
}

static unsigned int get_cpu_target_voltage(void)
{
	unsigned int	cpu_vdd = 0;
	if (cpu_gpu_flag == 1)
		regmap_read(aon_apb_reg_base, REG_AON_APB_RES_REG5, &cpu_vdd);
	pr_debug("%s: cpu_vdd is %dmv, cpu_gpu_flag is %d\n", __func__, cpu_vdd/1000, cpu_gpu_flag);
	return cpu_vdd;
}

static int efuse_uid_waferid_get(struct device *dev,
	struct device_node *np_cpufreq_data, u32 *p_binning_data)
{
	char uid[50];
	const struct property *prop = NULL, *prop1 = NULL;
	struct device_node *dn_cpufreq_data;
	char *p1, *p2;

	if ((dev == NULL && np_cpufreq_data == NULL) ||
	p_binning_data == NULL) {
		P_INF("%s: inputs are NULL\n", __func__);
		return -ENOENT;
	}

	if (dev && !of_node_get(dev->of_node)) {
		dev_info(dev, "sprd_cpufreq: %s not found cpu node\n",
			__func__);
		return -ENOENT;
	}

	if (dev)
		prop = of_find_property(dev->of_node,
			"sprd,ss-waferid-names", NULL);
	prop1 = of_find_property(np_cpufreq_data,
		"sprd,ss-waferid-names", NULL);

	if (!prop && !prop1)
		return -ENODEV;
	if (prop && !prop->value)
		return -ENODATA;
	if (prop1 && !prop1->value)
		return -ENODATA;
	if (prop)
		dn_cpufreq_data = dev->of_node;
	if (prop1)
		dn_cpufreq_data = np_cpufreq_data;

	memset(uid, 0, sizeof(uid));
	sprd_get_chip_uid(uid);
	/*find second '_', and set 0, strcpy(uid, "T8F465_9_12_28");*/
	p1 = strchr(uid, '_');
	if (!p1)
		return -ENODATA;
	p1++;
	p2 = strchr(p1, '_');
	if (!p2)
		return -ENODATA;
	*p2 = 0;
	if (of_property_match_string(dn_cpufreq_data,
				"sprd,ss-waferid-names", uid) >= 0) {
		*p_binning_data = 3;
	} else {
		*p_binning_data = 4;
	}

	P_INF("waferid BIN[%u] uid[%s]\n", *p_binning_data, uid);
	return 0;
}

#if defined(CONFIG_OTP_SPRD_AP_PUBLIC_EFUSE)
static int efuse_binning_low_volt_get(struct device *dev,
	struct device_node *np_cpufreq_data, u32 *p_binning_data)
{
	const struct property *prop, *prop1;
	int nLSB;
	u32 efuse_blk_binning;
	u32 efuse_blk_binning_bits;
	u32 efuse_blk_binning_data;
	struct device_node *cpu_np;
	const __be32 *val;

	if (!dev || !np_cpufreq_data || !p_binning_data)
		return -ENOENT;

	cpu_np = of_node_get(dev->of_node);
	if (!cpu_np) {
		dev_info(dev, "sprd_cpufreq: efuse_binning_get failed to find cpu node\n");
		return -ENOENT;
	}

	prop = of_find_property(dev->of_node,
					"sprd,efuse-blk-binning-low-volt", NULL);
	prop1 = of_find_property(np_cpufreq_data,
					"sprd,efuse-blk-binning-low-volt", NULL);
	if (!prop && !prop1)
		return -ENODEV;
	if (prop && !prop->value)
		return -ENODATA;
	if (prop1 && !prop1->value)
		return -ENODATA;
	if (prop1)
		prop = prop1;

	if (prop->length  != (2*sizeof(u32))) {
		dev_err(dev, "sprd_cpufreq: %s: Invalid efuse_blk_binning(prop->length %d)\n",
				__func__, prop->length);
		return -EINVAL;
	}
	val = prop->value;
	efuse_blk_binning = be32_to_cpup(val++);
	efuse_blk_binning_bits = be32_to_cpup(val);
	nLSB = __ffs(efuse_blk_binning_bits);

	efuse_blk_binning_data = sprd_efuse_double_read(efuse_blk_binning, 1);
	efuse_blk_binning_data = efuse_blk_binning_data & efuse_blk_binning_bits;
	if (nLSB != 0) {
		efuse_blk_binning_data = efuse_blk_binning_data>>nLSB;
	}
	dev_info(dev, "sprd_cpufreq: blk 0x%x, bits 0x%x, nLSB %d, BIN %u\n",
			efuse_blk_binning, efuse_blk_binning_bits,
			nLSB, efuse_blk_binning_data);
	if (efuse_blk_binning_data == 0) {
		*p_binning_data = 0;
		return -1;
	}
	*p_binning_data = efuse_blk_binning_data;

	return 0;
}
#endif

static int efuse_binning_get(struct device *dev,
struct device_node *np_cpufreq_data, u32 *p_binning_data)
{
	const struct property *prop = NULL, *prop1 = NULL;
	int nLSB;
	u32 efuse_blk_binning;
	u32 efuse_blk_binning_bits;
	u32 efuse_blk_binning_data;
	u32 default_efuse_blk_binning;
	const __be32 *val;

	if ((dev == NULL && np_cpufreq_data == NULL) ||
	p_binning_data == NULL) {
		P_INF("%s: inputs are NULL\n", __func__);
		return -ENOENT;
	}

	if (dev && !of_node_get(dev->of_node)) {
		dev_info(dev, "sprd_cpufreq: %s not found cpu node\n",
			__func__);
		return -ENOENT;
	}

	if (dev)
		prop = of_find_property(dev->of_node,
			"sprd,efuse-blk-binning", NULL);
	prop1 = of_find_property(np_cpufreq_data,
					"sprd,efuse-blk-binning", NULL);
	if (!prop && !prop1)
		return -ENODEV;
	if (prop && !prop->value)
		return -ENODATA;
	if (prop1 && !prop1->value)
		return -ENODATA;
	if (prop1)
		prop = prop1;

	if (prop->length  != (2*sizeof(u32))) {
		P_ERR("%s: Invalid efuse_blk_binning(prop->length %d)\n",
				__func__, prop->length);
		return -EINVAL;
	}
	val = prop->value;
	efuse_blk_binning = be32_to_cpup(val++);
	efuse_blk_binning_bits = be32_to_cpup(val);
	nLSB = __ffs(efuse_blk_binning_bits);
	default_efuse_blk_binning = 0;
	if (of_property_read_u32(np_cpufreq_data, "default-efuse-blk-binning", &default_efuse_blk_binning))
		default_efuse_blk_binning = 0;

#if defined(CONFIG_OTP_SPRD_AP_EFUSE) || defined(CONFIG_OTP_SPRD_AP_IEFUSE)
	efuse_blk_binning_data = sprd_ap_efuse_read(efuse_blk_binning);
#elif defined(CONFIG_OTP_SPRD_AP_PUBLIC_EFUSE)
	efuse_blk_binning_data = sprd_efuse_double_read(efuse_blk_binning, 1);
#else
	efuse_blk_binning_data = 0;
#endif

	efuse_blk_binning_data = efuse_blk_binning_data & efuse_blk_binning_bits;
	if (nLSB != 0)
		efuse_blk_binning_data = efuse_blk_binning_data>>nLSB;

	P_INF("blk 0x%x, bits 0x%x, nLSB %d, BIN %u\n",
			efuse_blk_binning, efuse_blk_binning_bits,
			nLSB, efuse_blk_binning_data);
	if (default_efuse_blk_binning == 1) {
		if (efuse_blk_binning_data > SPRD_BINNING_MAX) {
			*p_binning_data = 0;
			return -EINVAL;
		}
	} else{
		if (efuse_blk_binning_data == SPRD_BINNING_MIN || efuse_blk_binning_data > SPRD_BINNING_MAX) {
			*p_binning_data = 0;
			return -EINVAL;
		}
	}
	*p_binning_data = efuse_blk_binning_data;

	return 0;
}

static int temp_binning_get(struct device *dev,
	struct device_node *np_cpufreq_data,
	struct sprd_cpufreq_driver_data *cpufreq_data,
	char *opp_temp)
{
	const struct property *prop = NULL, *prop1 = NULL;
	int i = 0;
	u32 temp_threshold;
	const __be32 *val;
	int temp_index;

	if ((dev == NULL && np_cpufreq_data == NULL) ||
	cpufreq_data == NULL ||
	opp_temp == NULL) {
		P_INF("%s: inputs are NULL\n", __func__);
		return -ENOENT;
	}

	if (dev && !of_node_get(dev->of_node)) {
		dev_info(dev, "sprd_cpufreq: %s not found cpu node\n",
			__func__);
		return -ENOENT;
	}

	if (dev)
		prop = of_find_property(dev->of_node,
			"sprd,cpufreq-temp-threshold", NULL);
	prop1 = of_find_property(np_cpufreq_data,
		"sprd,cpufreq-temp-threshold", NULL);
	if (!prop && !prop1)
		return -ENODEV;
	if (prop && !prop->value)
		return -ENODATA;
	if (prop1 && !prop1->value)
		return -ENODATA;
	if (prop1)
		prop = prop1;

	if (prop->length < sizeof(u32)) {
		P_ERR("Invalid temp_binning_get(prop->length %d)\n",
				prop->length);
		return -EINVAL;
	}

	cpufreq_data->temp_max_freq = 0;
	cpufreq_data->temp_max = 0;
	temp_index = -1;
	val = prop->value;
	for (i = 0;
	i < (prop->length/sizeof(u32)) && i < SPRD_CPUFREQ_MAX_TEMP;
	i++) {
		/*TODO: need to compatible with negative degree celsius*/
		temp_threshold = be32_to_cpup(val++);
		if (cpufreq_data->temp_now >= temp_threshold) {
			temp_index = i;
			sprintf(opp_temp, "-%d", temp_threshold);
		}
		cpufreq_data->temp_list[i] = temp_threshold;
		cpufreq_data->temp_max++;
		P_DBG("found temp %u\n", temp_threshold);
	}

	cpufreq_data->temp_bottom = temp_index < 0 ?
				SPRD_CPUFREQ_TEMP_MIN :
				cpufreq_data->temp_list[temp_index];
	cpufreq_data->temp_top = (temp_index + 1) >= cpufreq_data->temp_max ?
				SPRD_CPUFREQ_TEMP_MAX :
				cpufreq_data->temp_list[temp_index + 1];

	P_DBG("temp_binning_get max num=%d bottom=%d top=%d\n",
		cpufreq_data->temp_max,
		cpufreq_data->temp_bottom,
		cpufreq_data->temp_top);

	P_INF("temp_binning_get[%s] by temp %d\n",
		opp_temp, cpufreq_data->temp_now);
	return 0;
}

/* Initializes OPP tables based on old-deprecated bindings */
static int dev_pm_opp_of_add_table_binning(int cluster,
	struct device *dev,
	struct device_node *np_cpufreq_data,
	struct sprd_cpufreq_driver_data *cpufreq_data)
{
	struct device_node *cpu_np = NULL;
	struct device_node *np = NULL, *np1 = NULL;
	const struct property *prop = NULL, *prop1 = NULL;
	const __be32 *val;
	int nr;
	u32 p_binning_data = 0;
	int index = 0, count = 0;
	char opp_string[30] = "operating-points";
	char temp_string[20] = "";
#if defined(CONFIG_OTP_SPRD_AP_PUBLIC_EFUSE)
	u32 p_binning_low_volt = 0;
#endif

	if ((dev == NULL && np_cpufreq_data == NULL) ||
	cpufreq_data == NULL) {
		P_INF("%s: inputs are NULL\n", __func__);
		return -ENOENT;
	}

	if (dev && np_cpufreq_data == NULL) {
		cpu_np = of_node_get(dev->of_node);
		if (!cpu_np) {
			dev_err(dev, "sprd_cpufreq: failed to find cpu node\n");
			return -ENOENT;
		}

		np = of_parse_phandle(cpu_np, "cpufreq-data", 0);
		np1 = of_parse_phandle(cpu_np, "cpufreq-data-v1", 0);
		if (!np && !np1) {
			dev_err(dev, "sprd_cpufreq: failed to find cpufreq-data\n");
			of_node_put(cpu_np);
			return -ENOENT;
		}
		if (np1)
			np = np1;
		np_cpufreq_data = np;
	}

#if defined(CONFIG_OTP_SPRD_AP_EFUSE) || defined(CONFIG_OTP_SPRD_AP_IEFUSE)
	if (!efuse_binning_get(dev, np_cpufreq_data, &p_binning_data))
#elif defined(CONFIG_OTP_SPRD_AP_PUBLIC_EFUSE)
	if (efuse_binning_get(dev, np_cpufreq_data, &p_binning_data) >= 0)
#endif
	{
		P_INF("p_binning_data=0x%x by BIN\n",
				p_binning_data);

		index = strlen(opp_string);
		opp_string[index++] = '-';
		opp_string[index++] = '0' + p_binning_data;
		opp_string[index] = '\0';

	#if defined(CONFIG_OTP_SPRD_AP_PUBLIC_EFUSE)
		efuse_binning_low_volt_get(dev, np_cpufreq_data, &p_binning_low_volt);

		opp_string[index++] = '-';
		opp_string[index++] = '0' + p_binning_low_volt;
		opp_string[index] = '\0';
	#endif

		/*select dvfs table by temp only if bin is not zero*/
		if (!temp_binning_get(dev,
		np_cpufreq_data, cpufreq_data, temp_string))
		strcat(opp_string, temp_string);

	} else {
		efuse_uid_waferid_get(dev, np_cpufreq_data, &p_binning_data);
		P_INF("p_binning_data=0x%x by UID\n",
				p_binning_data);

		if (p_binning_data > 0) {
			index = strlen(opp_string);
			opp_string[index++] = '-';
			opp_string[index++] = '0' + p_binning_data;
			opp_string[index] = '\0';
			/*select dvfs table by temp only if bin is not zero*/
			if (!temp_binning_get(dev,
			np_cpufreq_data, cpufreq_data, temp_string))
				strcat(opp_string, temp_string);
		}
	}

	P_INF("opp_string[%s]\n", opp_string);

	if (dev)
		prop = of_find_property(dev->of_node, opp_string, NULL);
	prop1 = of_find_property(np_cpufreq_data, opp_string, NULL);
	if (!prop && !prop1)
		return -ENODEV;
	if (prop && !prop->value)
		return -ENODATA;
	if (prop1 && !prop1->value)
		return -ENODATA;
	if (prop1)
		prop = prop1;
	/*
	 * Each OPP is a set of tuples consisting of frequency and
	 * voltage like <freq-kHz vol-uV>.
	 */
	nr = prop->length / sizeof(u32);
	if (nr % 2) {
		P_ERR("%s: Invalid OPP list\n", __func__);
		return -EINVAL;
	}

	cpufreq_data->freqvolts = 0;
	val = prop->value;
	while (nr) {
		unsigned long freq = be32_to_cpup(val++) * 1000;
		unsigned long volt = be32_to_cpup(val++);

		if (dev)
			dev_pm_opp_remove(dev, freq);
		if (dev && dev_pm_opp_add(dev, freq, volt))
			dev_warn(dev, "sprd_cpufreq:dev_pm Failed to add OPP %ld\n",
				freq);
		else {
			if (freq/1000 > cpufreq_data->temp_max_freq)
				cpufreq_data->temp_max_freq = freq/1000;
			P_INF("cluster%d opp_add [freq-%lu & volt-%lu]\n",
					cluster,
					freq, volt);
		}
		if (count < SPRD_CPUFREQ_MAX_FREQ_VOLT) {
			cpufreq_data->freqvolt[count].freq = freq;
			cpufreq_data->freqvolt[count].volt = volt;
			cpufreq_data->freqvolts++;
		}

		count++;
		nr -= 2;
	}

	if (sprd_cpufreqhw_probed(cluster)) {
		while (count-- > 0)
			sprd_cpufreqhw_opp_add(cluster,
				cpufreq_data->freqvolt[count].freq,
				cpufreq_data->freqvolt[count].volt,
				cpufreq_data->freqvolts - 1 - count);
	}

	return 0;
}

static int dev_pm_opp_of_add_table_binning_slave(
	struct sprd_cpufreq_driver_data *c_host,
	int temp_now)
{
	struct device *cpu_dev;
	struct device_node *cpu_np = NULL;
	struct device_node *np = NULL, *np1 = NULL, *np_host = NULL;
	unsigned int cluster;
	int ret = 0, i;

	if (c_host == NULL) {
		P_ERR("c_host is NULL\n");
		return -ENODEV;
	}

	/*if no sub clusters ,return ok*/
	if (c_host->sub_cluster_bits == 0)
		return 0;

	cpu_dev = c_host->cpu_dev;
	if (IS_ERR(cpu_dev)) {
		P_ERR("dev_host is ERR\n");
		return -ENODEV;
	}
	cpu_np = of_node_get(cpu_dev->of_node);
	if (!cpu_np) {
		P_ERR("failed to find cpu node\n");
		return -ENOENT;
	}

	np = of_parse_phandle(cpu_np, "cpufreq-data", 0);
	np1 = of_parse_phandle(cpu_np, "cpufreq-data-v1", 0);
	if (!np && !np1) {
		P_ERR("failed to find cpufreq-data\n");
		of_node_put(cpu_np);
		return -ENOENT;
	}
	if (np1)
		np = np1;

	np_host = np;
	for (i = 0; i < SPRD_CPUFREQ_MAX_MODULE; i++) {
		np = of_parse_phandle(np_host, "cpufreq-sub-clusters", i);
		if (!np) {
			P_DBG("index %d not found sub-clusters\n", i);
			goto free_np;
		}
		P_DBG("slave index %d name is found %s\n",
			i, np->full_name);
		cluster = SPRD_CPUFREQ_MAX_MODULE;
		if (of_property_read_u32(np, "cpufreq-cluster-id", &cluster)) {
			P_ERR("index %d not found cpufreq_custer_id\n", i);
			ret = -ENODEV;
			goto free_np;
		}
		if (cluster >= SPRD_CPUFREQ_MAX_MODULE ||
		cpufreq_datas[cluster] == NULL) {
			P_ERR("index %d cluster %u is NULL\n", i, cluster);
			continue;
		}
		cpufreq_datas[cluster]->temp_now = temp_now;
		ret = dev_pm_opp_of_add_table_binning(cluster,
			NULL,
			np,
			cpufreq_datas[cluster]);
		if (ret)
			goto free_np;
	}

free_np:
	if (np)
		of_node_put(np);
	if (np_host)
		of_node_put(np_host);
	return ret;
}
static int sprd_verify_opp_with_regulator(struct device *cpu_dev,
		struct regulator *cpu_reg, unsigned int volt_tol)
{
	unsigned long opp_freq = 0;
	unsigned long min_uV = ~0, max_uV = 0;

	while (1) {
		struct dev_pm_opp *opp;
		unsigned long opp_uV, tol_uV;

		rcu_read_lock();
		opp = dev_pm_opp_find_freq_ceil(cpu_dev, &opp_freq);
		if (IS_ERR(opp)) {
			/* We dont have more freq in opp table,
			 * break the loop
			*/
			rcu_read_unlock();
			P_DBG("invalid opp freq %lu\n", opp_freq);
			break;
		}
		opp_uV = dev_pm_opp_get_voltage(opp);
		rcu_read_unlock();
		tol_uV = opp_uV * volt_tol / 100;
		if (regulator_is_supported_voltage(cpu_reg, opp_uV,
							   opp_uV + tol_uV)) {
			if (opp_uV < min_uV)
				min_uV = opp_uV;
			if (opp_uV > max_uV)
				max_uV = opp_uV;
		} else {
			P_WARN("disabling unsupported opp_freq %lu\n",
					opp_freq);
			dev_pm_opp_disable(cpu_dev, opp_freq);
		}
	opp_freq++;
	}
	P_DBG("regulator min volt %lu, max volt %lu\n", min_uV, max_uV);

	return regulator_set_voltage_time(cpu_reg, min_uV, max_uV);
}

/**
 * sprd_cpufreq_update_opp() - returns the max freq of a cpu
 * and update dvfs table by temp_now
 * @cpu: which cpu you want to update dvfs table
 * @temp_now: current temperature on this cpu, mini-degree.
 *
 * Return:
 * 1.cluster is not working, then return 0
 * 2.succeed to update dvfs table
 * then return max freq(KHZ) of this cluster
 */
unsigned int sprd_cpufreq_update_opp(int cpu, int temp_now)
{
	unsigned int max_freq = 0;
	struct sprd_cpufreq_driver_data *c;
	int cluster;

	temp_now = temp_now/1000;
	if (temp_now <= SPRD_CPUFREQ_TEMP_MIN ||
	temp_now >= SPRD_CPUFREQ_TEMP_MAX)
		return 0;

	cluster = topology_physical_package_id(cpu);
	if (cluster > SPRD_CPUFREQ_MAX_CLUSTER) {
		P_ERR("cpu%d is overflowd %d\n", cpu,
			SPRD_CPUFREQ_MAX_CLUSTER);
		return 0;
	}

	c = cpufreq_datas[cluster];

	if (c != NULL &&
	c->online &&
	c->temp_max > 0) {
		/*Never block IPA thread*/
		if (mutex_trylock(c->volt_lock) != 1)
			return 0;
		c->temp_now = temp_now;
		if (temp_now < c->temp_bottom && !c->temp_fall_time)
			c->temp_fall_time = jiffies + SPRD_CPUFREQ_TEMP_FALL_HZ;
		if (temp_now >= c->temp_bottom)
			c->temp_fall_time = 0;
		if (temp_now >= c->temp_top ||
		(c->temp_fall_time &&
		time_after(jiffies, c->temp_fall_time))) {
			/*if fails to update slave dvfs table,
			  *never update any more this time,
			  *try to update slave and host dvfs table next time,
			  *because once host dvfs table is updated,
			  *slave dvfs table can not be update here any more.
			  */
			if (!dev_pm_opp_of_add_table_binning_slave(
			c,
			temp_now)) {
				c->temp_fall_time = 0;
				if (!dev_pm_opp_of_add_table_binning(
					c->cluster, c->cpu_dev,
					NULL, c))
					max_freq = c->temp_max_freq;
				dev_info(c->cpu_dev,
					"update temp_max_freq %u\n", max_freq);
			}
		}
		mutex_unlock(c->volt_lock);
	}

	return max_freq;
}
EXPORT_SYMBOL_GPL(sprd_cpufreq_update_opp);

static int sprd_cpufreq_set_clock(struct sprd_cpufreq_driver_data *c,
						unsigned long freq)
{
	struct clk *clk_low_freq_p;
	struct clk *clk_high_freq_p;
	unsigned long clk_low_freq_p_max = 0;
	int ret = 0, i;

	if (c == NULL || freq == 0)
		return -ENODEV;

	clk_low_freq_p = c->clk_low_freq_p;
	clk_high_freq_p = c->clk_high_freq_p;
	clk_low_freq_p_max = c->clk_low_freq_p_max;

	for (i = 0; i < SPRD_MAX_CPUS_EACH_CLUSTER; i++) {
		if (!IS_ERR(c->clks[i])) {
			ret = clk_set_parent(c->clks[i], clk_low_freq_p);
			if (ret) {
				P_ERR("set clk_low_freq_p as parent\n");
				return ret;
			}
		}
	}
	if (freq != clk_low_freq_p_max) {
		if (clk_set_rate(clk_high_freq_p, freq))
			P_ERR("set clk_high_freq_p freq %luKHz\n",
				freq / 1000);
		for (i = 0; i < SPRD_MAX_CPUS_EACH_CLUSTER; i++) {
			if (!IS_ERR(c->clks[i])) {
				ret = clk_set_parent(c->clks[i], clk_high_freq_p);
				if (ret) {
					P_ERR("set clk_high_freq_p as parent\n");
					return ret;
				}
			}
		}
	}
	return 0;
}

static int sprd_cpufreq_set_clock_v1(struct sprd_cpufreq_driver_data *c,
						unsigned long freq)
{
	int ret = 0, i;

	if (c == NULL || freq == 0)
		return -ENODEV;

	if (c->freq_req == freq)
		return 0;

	P_DBG("%s freq=%lu\n", __func__, freq);

	if (freq > c->clk_low_freq_p_max) {
		if (c->clk_high_freq_const == 0) {
			for (i = 0; i < SPRD_MAX_CPUS_EACH_CLUSTER; i++) {
				if (!IS_ERR(c->clks[i]))
					ret = clk_set_parent(c->clks[i],
						c->clk_low_freq_p);
				if (ret)
					break;
			}
			if (ret) {
				P_ERR("pre-set clk_low_freq_p as parent\n");
				goto exit_err;
			}
			ret = clk_set_rate(c->clk_high_freq_p, freq);
			if (ret) {
				P_ERR("set clk_high_freq_p freq %luKHz\n",
					freq / 1000);
				goto exit_err;
			}
		}
		for (i = 0; i < SPRD_MAX_CPUS_EACH_CLUSTER; i++) {
			if (!IS_ERR(c->clks[i]))
				ret = clk_set_parent(c->clks[i], c->clk_high_freq_p);
			if (ret)
				break;
		}
		if (ret) {
			P_ERR("in setting clk_high_freq_p as parent\n");
			goto exit_err;
		}
		P_DBG("set cluster%u setting clk_high as parent\n",
			c->cluster);
	} else if (freq == c->clk_low_freq_p_max) {
		for (i = 0; i < SPRD_MAX_CPUS_EACH_CLUSTER; i++) {
			if (!IS_ERR(c->clks[i]))
				ret = clk_set_parent(c->clks[i],
					c->clk_low_freq_p);
			if (ret)
				break;
		}
		if (ret) {
			P_ERR("error in setting clk_low_freq_p as parent\n");
			goto exit_err;
		}
		P_DBG("set cluster%u setting clk_low as parent\n",
			c->cluster);
	} else {
		P_ERR("error in setting clk_low_freq_p as parent\n");
		goto exit_err;
	}

	P_DBG("set cluster%u freq %luHZ\n", c->cluster, freq);
	c->freq_req = freq;
	return 0;
exit_err:
	return ret;
}

static struct regulator *sprd_volt_share_reg(struct sprd_cpufreq_driver_data *c)
{
	int cluster;
	struct regulator *reg = NULL;

	if (c == NULL)
		return NULL;

	for (cluster = 0; cluster < SPRD_CPUFREQ_MAX_MODULE; cluster++)
		if ((c->volt_share_masters_bits >> cluster) & 0x1) {
			P_DBG("check master cluster%u\n", cluster);
			if (cpufreq_datas[cluster] != NULL &&
			cpufreq_datas[cluster]->reg) {
				reg = cpufreq_datas[cluster]->reg;
				break;
			}
		}

	if (reg == NULL)
		for (cluster = 0; cluster < SPRD_CPUFREQ_MAX_MODULE; cluster++)
			if ((c->volt_share_hosts_bits >> cluster) & 0x1) {
				P_DBG("check host cluster%u\n", cluster);
				if (cpufreq_datas[cluster] != NULL &&
				cpufreq_datas[cluster]->reg) {
					reg = cpufreq_datas[cluster]->reg;
					break;
				}
			}

	P_DBG("cluster %u masters 0x%x hosts 0x%x reg %p\n",
		c->cluster,
		c->volt_share_masters_bits,
		c->volt_share_hosts_bits,
		reg);

	return reg;
}
static struct mutex *sprd_volt_share_lock(struct sprd_cpufreq_driver_data *c)
{
	int cluster;
	struct mutex *volt_lock = NULL;

	if (c == NULL)
		return NULL;

	for (cluster = 0; cluster < SPRD_CPUFREQ_MAX_MODULE; cluster++)
		if ((c->volt_share_masters_bits >> cluster) & 0x1) {
			P_DBG("check master cluster%u\n", cluster);
			if (cpufreq_datas[cluster] != NULL &&
			cpufreq_datas[cluster]->volt_lock) {
				volt_lock = cpufreq_datas[cluster]->volt_lock;
				break;
			}
		}

	if (volt_lock == NULL)
		for (cluster = 0; cluster < SPRD_CPUFREQ_MAX_MODULE; cluster++)
			if ((c->volt_share_hosts_bits >> cluster) & 0x1) {
				P_DBG("check host cluster%u\n", cluster);
				if (cpufreq_datas[cluster] != NULL &&
				cpufreq_datas[cluster]->volt_lock) {
					volt_lock =
					cpufreq_datas[cluster]->volt_lock;
					break;
				}
			}

	P_DBG("cluster %u masters 0x%x hosts 0x%x volt_lock %p\n",
		c->cluster,
		c->volt_share_masters_bits,
		c->volt_share_hosts_bits,
		volt_lock);

	return volt_lock;
}

static unsigned int sprd_volt_tol_min(struct sprd_cpufreq_driver_data *c,
	bool online)
{
	unsigned int cluster, volt_tol_min = 0;

	if (c == NULL)
		return 0;

	volt_tol_min = c->volt_tol_var;
	for (cluster = 0; cluster < SPRD_CPUFREQ_MAX_MODULE; cluster++)
		if ((c->volt_share_masters_bits >> cluster) & 0x1) {
			P_DBG("check master cluster%u\n", cluster);
			if (cpufreq_datas[cluster] != NULL &&
			(online ? cpufreq_datas[cluster]->online : 1) &&
			cpufreq_datas[cluster]->volt_tol_var < volt_tol_min) {
				volt_tol_min =
					cpufreq_datas[cluster]->volt_tol_var;
		}
	}

	P_DBG("cluster %u volt_share_masters_bits %u volt_tol_min %u\n",
		c->cluster,
		c->volt_share_masters_bits,
		volt_tol_min);

	return volt_tol_min;
}
/**
 * sprd_volt_req_max()  - get volt_req_max
 * @c:        cluster
 * @volt_max_aim: aimed max between *volt_max_p
 *	and max volt of online or all clusters.
 * @online: 0-just search all clusters; 1-just search online clusters.
 * @except_self: 0-just search all clusters; 1-just search online clusters.
 * @return: current max volt of online/all clusters.
 */
static unsigned long sprd_volt_req_max(struct sprd_cpufreq_driver_data *c,
	unsigned long *volt_max_aim, bool online, bool except_self)
{
	int cluster;
	unsigned long volt_max_req = 0, ret = 0;

	if (c == NULL)
		return 0;

	for (cluster = 0; cluster < SPRD_CPUFREQ_MAX_MODULE; cluster++)
		if ((c->volt_share_masters_bits >> cluster) & 0x1) {
			P_DBG("check master cluster%u\n", cluster);
			if ((except_self ? (cluster != c->cluster) : 1) &&
			(cpufreq_datas[cluster] != NULL) &&
			(online ? cpufreq_datas[cluster]->online : 1) &&
			(cpufreq_datas[cluster]->volt_req > volt_max_req)) {
				volt_max_req = cpufreq_datas[cluster]->volt_req;
			}
		}

	if (volt_max_req > *volt_max_aim)
		*volt_max_aim = volt_max_req;
	else
		ret = volt_max_req;

	P_DBG("cluster %u volt_share_masters_bits %u volt_max %lu\n",
		c->cluster,
		c->volt_share_masters_bits,
		*volt_max_aim);

	return ret;
}

static int sprd_freq_sync_by_volt(
	struct sprd_cpufreq_driver_data *c,
	unsigned long volt)
{
	int i = 0, ret = -ENODEV;

	while (i < c->freqvolts) {
		if (c->freqvolt[i].volt > 0 &&
		volt >= c->freqvolt[i].volt)
			break;
		i++;
	}
	if (i < c->freqvolts) {
		ret = sprd_cpufreq_set_clock_v1(c, c->freqvolt[i].freq);
		if (!ret)
			c->volt_req = c->freqvolt[i].volt;
	} else
		P_INF("%s not found more than volt%lu\n", __func__, volt);

	return ret;
}

static int sprd_volt_share_slaves_notify(
	struct sprd_cpufreq_driver_data *host,
	unsigned long volt)
{
	unsigned int cluster;
	int ret;

	P_DBG("%s volt_share_slaves_bits%u, volt %lu\n",
		__func__, host->volt_share_slaves_bits, volt);

	for (cluster = 0; cluster < SPRD_CPUFREQ_MAX_MODULE; cluster++)
		if ((host->volt_share_slaves_bits >> cluster) & 0x1) {
			if (cpufreq_datas[cluster] != NULL &&
			cpufreq_datas[cluster]->online &&
			cpufreq_datas[cluster]->volt_share_hosts_bits) {
				ret = sprd_freq_sync_by_volt(
					cpufreq_datas[cluster], volt);
				if (ret)
					goto EXIT_ERR;
			}
		}

	return 0;
EXIT_ERR:
	P_INF("%s cluster%u,EXIT_ERR!\n", __func__, cluster);
	return ret;
}
/**
 * sprd_freq_sync_slaves_notify()  - sprd_freq_sync_slaves_notify
 * @idx:        0 points to min freq, ascending order
 */
static int sprd_freq_sync_slaves_notify(
	struct sprd_cpufreq_driver_data *host,
	const unsigned int idx, bool force)
{
	struct sprd_cpufreq_driver_data *c;
	unsigned int cluster;
	int ret;

	if (!host) {
		P_DBG("invalid host cpufreq_data\n");
		return -ENODEV;
	}

	for (cluster = 0; cluster < SPRD_CPUFREQ_MAX_MODULE; cluster++)
		if ((host->freq_sync_slaves_bits >> cluster) & 0x1) {
			if (cpufreq_datas[cluster] != NULL &&
				cpufreq_datas[cluster]->online &&
				cpufreq_datas[cluster]->freq_sync_hosts_bits) {
				c = cpufreq_datas[cluster];
				if (sprd_cpufreqhw_probed(
					c->cluster)) {
					/*dvfs and dvfs table need mutex lock*/
					mutex_lock(c->volt_lock);
					ret = sprd_cpufreqhw_set_target(
						c->cluster,
						idx);
					mutex_unlock(c->volt_lock);
				} else
					ret = sprd_cpufreq_set_target(
						c,
						idx, force);
				if (ret)
					goto EXIT_ERR;
			}
		}

	return 0;
EXIT_ERR:
	P_INF("%s sub-cluster%u EXIT_ERR!\n", __func__, cluster);
	return ret;
}
/**
 * sprd_cpufreq_set_target()  - cpufreq_set_target
 * @idx:        0 points to min freq, ascending order
 */
static int sprd_cpufreq_set_target(
	struct sprd_cpufreq_driver_data *cpufreq_data,
	unsigned int idx, bool force)
{
	struct dev_pm_opp *opp;
	unsigned long volt_new = 0, volt_new_req = 0, volt_old = 0;
	unsigned long old_freq_hz, new_freq_hz, freq_Hz, opp_freq_hz;
	unsigned int volt_tol = 0;
	struct regulator *cpu_reg;
	struct clk *cpu_clk;
	struct device *cpu_dev;
	int cluster;
	int ret = 0;

	if (!cpufreq_data ||
	(cpufreq_data && idx >= cpufreq_data->freqvolts)) {
		P_ERR("invalid cpufreq_data/idx%u, returning\n", idx);
		return -ENODEV;
	}

	P_DBG("setting target for cluster %d, freq idx %u freqvolts %u\n",
		cpufreq_data->cluster,
		idx,
		cpufreq_data->freqvolts);

	if (IS_ERR(cpufreq_data->clk) ||
	cpufreq_data->clk == NULL ||
	IS_ERR(cpufreq_data->reg) ||
	cpufreq_data->reg == NULL) {
		P_ERR("no regulator/clk for cluster %d\n",
			cpufreq_data->cluster);
		return -ENODEV;
	}

	if (idx > (cpufreq_data->freqvolts - 1))
		idx = 0;
	else
		idx = (cpufreq_data->freqvolts - 1 - idx);

	cpu_dev = cpufreq_data->cpu_dev;
	cpu_clk = cpufreq_data->clk;
	cpu_reg = cpufreq_data->reg;
	cluster = cpufreq_data->cluster;
	freq_Hz = clk_round_rate(cpu_clk, cpufreq_data->freqvolt[idx].freq);
	if (freq_Hz <= 0)
		freq_Hz = cpufreq_data->freqvolt[idx].freq;
	new_freq_hz = freq_Hz;
	old_freq_hz = clk_get_rate(cpu_clk);

	P_DBG("freq idx %u, freq %lu in freqvolt\n",
		idx, cpufreq_data->freqvolt[idx].freq);
	if (cpu_dev) {
		/*for cpu device, get freq&volt from opp*/
		rcu_read_lock();
		opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq_Hz);
		if (IS_ERR(opp)) {
			rcu_read_unlock();
			P_ERR("failed to find OPP for %luKhz\n",
					freq_Hz / 1000);
			return PTR_ERR(opp);
		}
		volt_new = dev_pm_opp_get_voltage(opp);
		opp_freq_hz = dev_pm_opp_get_freq(opp);
		rcu_read_unlock();
	} else {
		/*for sub device, get freq&volt from table*/
		volt_new = cpufreq_data->freqvolt[idx].volt;
		opp_freq_hz = cpufreq_data->freqvolt[idx].freq;
	}
	volt_tol = cpufreq_data->volt_tol;

	if (force)
		mutex_lock(cpufreq_data->volt_lock);
	else {
		if (mutex_trylock(cpufreq_data->volt_lock) != 1) {
			P_INF("cannot acquire lock for cluster %d\n",
				cpufreq_data->cluster);
			return -EBUSY;
		}
	}

	/*must get real volt_old in mutex_lock domain*/
	volt_old = regulator_get_voltage(cpu_reg);
	P_DBG("Found OPP: %ld kHz, %ld uV, tolerance: %uuV\n",
		opp_freq_hz / 1000, volt_new, volt_tol);

	volt_new_req = volt_new;
	sprd_volt_req_max(cpufreq_data, &volt_new, true, true);
	if (!volt_new) {
		goto EXIT_ERR;
		P_DBG("fail to get volt_new=0\n");
	}

	P_DBG("cluster%u scaling from %lu MHz, %ld mV, --> %lu MHz, %ld mV\n",
		cpufreq_data->cluster,
		old_freq_hz / 1000000,
		(volt_old > 0) ? volt_old / 1000 : -1,
		new_freq_hz / 1000000,
		volt_new ? volt_new / 1000 : -1);

	if (volt_new < volt_old) {
		ret  = sprd_volt_share_slaves_notify(
			cpufreq_data,
			volt_new);
		if (ret)
			goto EXIT_ERR;
	}

	/* scaling up?  scale voltage before frequency */
	if (new_freq_hz > old_freq_hz) {
		P_DBG("scaling up voltage to %lu\n", volt_new);
		if (cpu_gpu_flag == 1) {
			down_voltage_sem();
			set_cpu_target_voltage(volt_new);
			volt_new = volt_new < get_gpu_target_voltage()?get_gpu_target_voltage():volt_new;
			pr_debug("scaling from %lu MHz, %ld mV, --> %lu MHz, %ld mV, cpu_vdd:%dmV, gpu_vdd:%dmV\n",\
				old_freq_hz / 1000000, (volt_old > 0) ? volt_old / 1000 : -1,\
				new_freq_hz / 1000000, volt_new ? volt_new / 1000 : -1,\
				get_cpu_target_voltage() ? get_cpu_target_voltage() / 1000 : -1, get_gpu_target_voltage() ? get_gpu_target_voltage()/1000 : -1);
			ret = regulator_set_voltage_tol(cpu_reg, volt_new, volt_tol);
			up_voltage_sem();
		} else {
			ret = regulator_set_voltage_tol(cpu_reg, volt_new, volt_tol);
		}

		if (ret) {
			if (cpu_gpu_flag == 1) {
				down_voltage_sem();
				set_cpu_target_voltage(volt_old);
				pr_err("failed to scale voltage %lu %u up: %d, cpu_vdd:%dmV, gpu_vdd:%dmV\n",\
					volt_new, volt_tol, ret,\
					get_cpu_target_voltage() ? get_cpu_target_voltage() / 1000 : -1, get_gpu_target_voltage() ? get_gpu_target_voltage()/1000 : -1);
				up_voltage_sem();
			}
			P_ERR("failed to scale voltage %lu %u up: %d\n",
				volt_new, volt_tol, ret);
			goto EXIT_ERR;
		}
	}

	ret = sprd_cpufreq_set_clock(cpufreq_data, new_freq_hz);
	if (ret) {
		P_ERR("failed to set clock %luMhz rate: %d\n",
					new_freq_hz / 1000000, ret);
		if ((volt_old > 0) && (new_freq_hz > old_freq_hz)) {
			P_INF("scaling to old voltage %lu\n", volt_old);
			if (cpu_gpu_flag == 1) {
				down_voltage_sem();
				set_cpu_target_voltage(volt_old);
				volt_old = volt_old < get_gpu_target_voltage()?get_gpu_target_voltage():volt_old;
				pr_debug("scaling from %lu MHz, %ld mV, --> %lu MHz, %ld mV,\
					cpu_vdd:%dmV, gpu_vdd:%dmV\n",\
					old_freq_hz / 1000000, (volt_old > 0) ? volt_old / 1000 : -1,\
					new_freq_hz / 1000000, volt_new ? volt_new / 1000 : -1,\
					get_cpu_target_voltage() ? get_cpu_target_voltage() / 1000 : -1, get_gpu_target_voltage() ? get_gpu_target_voltage()/1000 : -1);
				ret = regulator_set_voltage_tol(cpu_reg, volt_old, volt_tol);
				up_voltage_sem();
			} else {
				ret = regulator_set_voltage_tol(cpu_reg, volt_old, volt_tol);
			}
		}
		goto EXIT_ERR;
	}

	/* scaling down?  scale voltage after frequency */
	if (new_freq_hz < old_freq_hz) {
		P_DBG("scaling down voltage to %lu\n", volt_new);
		if (cpu_gpu_flag == 1) {
			down_voltage_sem();
			set_cpu_target_voltage(volt_new);
			volt_new = volt_new < get_gpu_target_voltage()?get_gpu_target_voltage():volt_new;
			pr_debug("cpu_gpu_cync:scaling from %lu MHz, %ld mV, --> %lu MHz, %ld mV, cpu_vdd:%dmV, gpu_vdd:%dmV\n",\
				old_freq_hz / 1000000, (volt_old > 0) ? volt_old / 1000 : -1,\
				new_freq_hz / 1000000, volt_new ? volt_new / 1000 : -1,\
				get_cpu_target_voltage() ? get_cpu_target_voltage() / 1000 : -1, get_gpu_target_voltage() ? get_gpu_target_voltage()/1000 : -1);
			ret = regulator_set_voltage_tol(cpu_reg, volt_new, volt_tol);
			up_voltage_sem();
		} else {
			ret = regulator_set_voltage_tol(cpu_reg, volt_new, volt_tol);
		}

		if (ret) {
			P_WARN("failed to scale volt %lu %u down: %d\n",
				volt_new, volt_tol, ret);
			if (cpu_gpu_flag == 1) {
				down_voltage_sem();
				set_cpu_target_voltage(volt_old);
				pr_err("failed to scale voltage %lu %u up: %d, cpu_vdd:%dmV, gpu_vdd:%dmV\n",\
					volt_new, volt_tol, ret,\
					get_cpu_target_voltage() ? get_cpu_target_voltage() / 1000 : -1, get_gpu_target_voltage() ? get_gpu_target_voltage()/1000 : -1);
				up_voltage_sem();
			}
			ret = sprd_cpufreq_set_clock(cpufreq_data, old_freq_hz);
			goto EXIT_ERR;
		}
	}

	if (volt_new >= volt_old) {
		ret  = sprd_volt_share_slaves_notify(
			cpufreq_data,
			volt_new);
	}
	cpufreq_data->volt_tol_var = volt_tol;
	cpufreq_data->volt_req = volt_new_req;
	cpufreq_data->freq_req = new_freq_hz;

	P_DBG("cluster%u After transition, new clk rate %luMhz, volt %dmV\n",
		cpufreq_data->cluster,
		clk_get_rate(cpufreq_data->clk) / 1000000,
		regulator_get_voltage(cpu_reg) / 1000);

	mutex_unlock(cpufreq_data->volt_lock);
	return ret;
EXIT_ERR:
	sprd_volt_share_slaves_notify(
		cpufreq_data,
		volt_old);

	mutex_unlock(cpufreq_data->volt_lock);
	return ret;
}
/**
 * sprd_cpufreq_set_target_index()  - cpufreq_set_target
 * @idx:        0 points to min freq, ascending order
 */
static int sprd_cpufreq_set_target_index(struct cpufreq_policy *policy,
				unsigned int idx)
{
	int ret, cluster;

	/*never dvfs until boot_done_timestamp*/
	if (unlikely(boot_done_timestamp &&
		time_after(jiffies, boot_done_timestamp))) {
		sprd_cpufreq_set_boost(0);
		sprd_cpufreq_driver.boost_enabled = false;
		P_INF("Disables boost it is %lu seconds after boot up\n",
			SPRD_CPUFREQ_DRV_BOOST_DURATOIN/HZ);
	}

	/*
	  *boost_mode_flag is true and cpu is on max freq
	  *so return 0 here, reject changing freq
	*/
	if (unlikely(boost_mode_flag)) {
		if (policy->max >= policy->cpuinfo.max_freq) {
			ret = 0;
			goto EXIT;
		} else {
			sprd_cpufreq_set_boost(0);
			sprd_cpufreq_driver.boost_enabled = false;
			P_INF("Disables boost due to policy max(%d<%d)\n",
				policy->max, policy->cpuinfo.max_freq);
		}
	}

	if (sprd_cpufreqhw_probed(topology_physical_package_id(policy->cpu))) {
		cluster = topology_physical_package_id(policy->cpu);
		/*dvfs and hw dvfs table regs need mutex lock*/
		mutex_lock(cpufreq_datas[cluster]->volt_lock);
		ret = sprd_cpufreqhw_set_target(
			topology_physical_package_id(policy->cpu),
			idx);
		mutex_unlock(cpufreq_datas[cluster]->volt_lock);
	}
	else
		ret = sprd_cpufreq_set_target(policy->driver_data,
			idx,
			true);

	if (!ret)
		ret = sprd_freq_sync_slaves_notify(policy->driver_data,
			idx,
			true);

EXIT:
	return ret;
}

static int sprd_cpufreq_init_slaves(
	struct sprd_cpufreq_driver_data *c_host,
	struct device_node *np_host)
{
	int ret = 0, i, ic;
	struct device_node *np;
	struct sprd_cpufreq_driver_data *c;
	unsigned int cluster;
	char coreclk[15] = "core*_clk";

	if (np_host == NULL)
		return -ENODEV;

	for (i = 0; i < SPRD_CPUFREQ_MAX_MODULE; i++) {
		np = of_parse_phandle(np_host, "cpufreq-sub-clusters", i);
		if (!np) {
			P_DBG("index %d not found sub-clusters\n", i);
			return 0;
		}

		P_INF("slave index %d name is found %s\n",
			i, np->full_name);

		if (of_property_read_u32(np, "cpufreq-cluster-id", &cluster)) {
			P_ERR("index %d not found cpufreq_custer_id\n", i);
			ret = -ENODEV;
			goto free_np;
		}

		if (cluster < SPRD_CPUFREQ_MAX_CLUSTER ||
		cluster >= SPRD_CPUFREQ_MAX_MODULE) {
			P_ERR("slave index %d custer %u is overflowed\n",
				i, cluster);
			ret = -ENODEV;
			goto free_np;
		}

		if (cpufreq_datas[cluster] == NULL) {
			c = kzalloc(sizeof(*c), GFP_KERNEL);
			if (!c) {
				ret = -ENOMEM;
				goto free_np;
			}
		} else
			c = cpufreq_datas[cluster];

		c_host->sub_cluster_bits |= (0x1 << cluster);

		c->cluster = cluster;
		c->clk = of_clk_get_by_name(np, "clk");
		if (IS_ERR(c->clk)) {
			P_ERR("slave index %d failed to get clk, %ld\n", i,
					PTR_ERR(c->clk));
			ret = PTR_ERR(c->clk);
			goto free_mem;
		}
		c->clks[0] = c->clk;
		for (ic = 1; ic < SPRD_MAX_CPUS_EACH_CLUSTER; ic++) {
			sprintf(coreclk, "core%d_clk", ic);
			c->clks[ic] = of_clk_get_by_name(np, coreclk);
		}
		c->clk_low_freq_p =
			of_clk_get_by_name(np, "clk_low");
		if (IS_ERR(c->clk_low_freq_p)) {
			P_INF("slave index %d clk_low is not defined\n", i);
			ret = PTR_ERR(c->clk_low_freq_p);
			goto free_clk;
		} else {
			c->clk_low_freq_p_max =
				clk_get_rate(c->clk_low_freq_p);
			P_DBG("index %d clk_low_freq_p_max[%lu]Khz\n", i,
				c->clk_low_freq_p_max / 1000);
		}

		of_property_read_u32(np, "clk-high-freq-const",
					&c->clk_high_freq_const);
		c->clk_high_freq_p =
			of_clk_get_by_name(np, "clk_high");
		if (IS_ERR(c->clk_high_freq_p)) {
			P_INF("slave index %d failed in getting clk_high%ld\n",
				i, PTR_ERR(c->clk_high_freq_p));
			ret = PTR_ERR(c->clk_high_freq_p);
			goto free_clk;
		} else {
			if (c->clk_high_freq_const)
				c->clk_high_freq_p_max =
					clk_get_rate(c->clk_high_freq_p);
			P_DBG("index %d clk_high_freq_p_max[%lu]Khz\n", i,
				c->clk_high_freq_p_max / 1000);
		}

		if (!c->clk_en) {
			ret = clk_prepare_enable(c->clk);
			if (ret) {
				P_INF("slave index %d cluster%d clk_en NG\n",
						i, cluster);
					goto free_clk;
			}
			for (ic = 1;
			ic < SPRD_MAX_CPUS_EACH_CLUSTER && !IS_ERR(c->clks[ic]);
			ic++) {
				ret = clk_prepare_enable(c->clks[ic]);
				if (ret) {
					P_ERR("slave index %d clus%d clk NG\n",
							i, cluster);
					goto free_clk;
				}
			}
			c->clk_en = 1;
		}
		c->freq_req = clk_get_rate(c->clk);

		if (of_property_read_u32(np, "voltage-tolerance",
			&c->volt_tol))
			c->volt_tol = DEF_VOLT_TOL;
		of_property_read_u32(np, "volt-share-hosts-bits",
			&c->volt_share_hosts_bits);
		of_property_read_u32(np, "volt-share-masters-bits",
			&c->volt_share_masters_bits);
		of_property_read_u32(np, "volt-share-slaves-bits",
			&c->volt_share_slaves_bits);
		of_property_read_u32(np, "freq-sync-hosts-bits",
			&c->freq_sync_hosts_bits);
		of_property_read_u32(np, "freq-sync-slaves-bits",
			&c->freq_sync_slaves_bits);

		c->volt_lock = sprd_volt_share_lock(c);
		if (c->volt_lock == NULL) {
			P_INF("slave index %d can find host volt_lock!\n", i);
			ret = -ENOMEM;
			goto free_clk;
		}

		c->reg = sprd_volt_share_reg(c);
		if (c->reg == NULL ||
		IS_ERR(c->reg)) {
			P_ERR("failed to get regulator, %ld\n",
				PTR_ERR(c->reg));
			ret = -ENODEV;
			goto free_clk;
		}
		c->volt_req = regulator_get_voltage(c->reg);

		ret = dev_pm_opp_of_add_table_binning(cluster,
			NULL,
			np,
			c);
		if (ret)
			goto free_clk;

		of_node_put(np);

		if (sprd_cpufreqhw_probed(c->cluster))
			sprd_cpufreqhw_enable(c->cluster, true);

		c->online = 1;
		cpufreq_datas[cluster] = c;

		P_DBG("slave index %d cpu_dev=%p\n",
			i, c->cpu_dev);
		P_DBG("slave index %d cpufreq_custer_id=%u\n",
			i, c->cluster);
		P_DBG("slave index %d online=%u\n",
			i, c->online);
		P_DBG("slave index %d volt_lock=%p\n",
			i, c->volt_lock);
		P_DBG("slave index %d reg=%p\n",
			i, c->reg);
		P_DBG("slave index %d clk_high_freq_const=%u\n",
			i, c->clk_high_freq_const);
		P_DBG("slave index %d clk=%p\n",
			i, c->clk);
		P_DBG("slave index %d clk_low_freq_p=%p\n",
			i, c->clk_low_freq_p);
		P_DBG("slave index %d clk_high_freq_p=%p\n",
			i, c->clk_high_freq_p);
		P_DBG("slave index %d clk_low_freq_p_max=%lu\n",
			i, c->clk_low_freq_p_max);
		P_DBG("slave index %d clk_high_freq_p_max=%lu\n",
			i, c->clk_high_freq_p_max);
		P_DBG("slave index %d freq_req=%lu\n",
			i, c->freq_req);
		P_DBG("slave index %d volt_tol=%u\n",
			i, c->volt_tol);
		P_DBG("slave index %d volt_req=%lu\n",
			i, c->volt_req);
		P_DBG("slave index %d volt_share_hosts_bits=%u\n",
			i, c->volt_share_hosts_bits);
		P_DBG("slave index %d volt_share_masters_bits=%u\n",
			i, c->volt_share_masters_bits);
		P_DBG("slave index %d volt_share_slaves_bits=%u\n",
			i, c->volt_share_slaves_bits);
		P_DBG("slave index %d freq_sync_hosts_bits=%u\n",
			i, c->freq_sync_hosts_bits);
		P_DBG("slave index %d freq_sync_slaves_bits=%u\n",
			i, c->freq_sync_slaves_bits);
	}

	return ret;
free_clk:
	if (!IS_ERR(c->clk)) {
		clk_put(c->clk);
		c->clk = ERR_PTR(-ENOENT);
		c->clks[0] = ERR_PTR(-ENOENT);
		for (ic = 1; ic < SPRD_MAX_CPUS_EACH_CLUSTER; ic++) {
			if (!IS_ERR(c->clks[ic]))
				clk_put(c->clks[ic]);
			c->clks[ic] = ERR_PTR(-ENOENT);
		}
	}
	if (!IS_ERR(c->clk_low_freq_p))
		clk_put(c->clk_low_freq_p);
	if (!IS_ERR(c->clk_high_freq_p))

		clk_put(c->clk_high_freq_p);
free_mem:
	kfree(c);
	cpufreq_datas[cluster] = NULL;
free_np:
	if (np)
		of_node_put(np);
	return ret;
}

static int sprd_cpufreq_init(struct cpufreq_policy *policy)
{
	struct dev_pm_opp *opp;
	unsigned long freq_Hz = 0;
	int ret = 0, ic = 0;
	char coreclk[15] = "core*_clk";
	struct device *cpu_dev;
	struct regulator *cpu_reg = NULL;
	struct device_node *cpu_np;
	struct device_node *np, *np1;
	struct clk *cpu_clk;
	struct sprd_cpufreq_driver_data *c;
	unsigned int volt_tol = 0;
	unsigned int transition_latency = CPUFREQ_ETERNAL;
	unsigned long clk_low_freq_p_max = 0;
	struct cpufreq_frequency_table *freq_table = NULL;
	int cpu = 0;

	if (!policy) {
		P_ERR("invalid cpufreq_policy\n");
		return -ENODEV;
	}

	if (topology_physical_package_id(policy->cpu)
	>= SPRD_CPUFREQ_MAX_CLUSTER) {
		P_ERR("cpu%d in invalid cluster %d\n", policy->cpu,
			topology_physical_package_id(policy->cpu));
		return -ENODEV;
	}

	cpu = policy->cpu;
	P_DBG("going to get cpu%d device\n", cpu);
	cpu_dev = get_cpu_device(cpu);
	if (IS_ERR(cpu_dev)) {
		P_ERR("failed to get cpu%d device\n", cpu);
		return -ENODEV;
	}

	cpu_np = of_node_get(cpu_dev->of_node);
	if (!cpu_np) {
		P_ERR("failed to find cpu%d node\n", cpu);
		return -ENOENT;
	}

	np = of_parse_phandle(cpu_np, "cpufreq-data", 0);
	np1 = of_parse_phandle(cpu_np, "cpufreq-data-v1", 0);
	if (!np && !np1) {
		P_ERR("failed to find cpufreq-data for cpu%d\n", cpu);
		of_node_put(cpu_np);
		return -ENOENT;
	}
	if (np1)
		np = np1;

	if (sprd_cpufreq_data(cpu))
		c = sprd_cpufreq_data(cpu);
	else {
		c = kzalloc(sizeof(*c), GFP_KERNEL);
		if (!c) {
			ret = -ENOMEM;
			goto free_np;
		}
	}

	of_property_read_u32(np, "volt-share-masters-bits",
		&c->volt_share_masters_bits);
	of_property_read_u32(np, "volt-share-slaves-bits",
		&c->volt_share_slaves_bits);
	of_property_read_u32(np, "freq-sync-hosts-bits",
		&c->freq_sync_hosts_bits);
	of_property_read_u32(np, "freq-sync-slaves-bits",
		&c->freq_sync_slaves_bits);
	P_DBG("read dts, masters_bits %u, slaves_bits %u\n",
		c->volt_share_masters_bits,
		c->volt_share_slaves_bits);

	if (!c->volt_lock) {
		c->volt_lock = sprd_volt_share_lock(c);
		if (!c->volt_lock) {
			c->volt_lock =
				kzalloc(sizeof(struct mutex), GFP_KERNEL);
			if (!c->volt_lock) {
				ret = -ENOMEM;
				goto free_mem;
			}
			mutex_init(c->volt_lock);
		}
	}

	mutex_lock(c->volt_lock);

	c->cluster = topology_physical_package_id(cpu);

	cpu_clk = of_clk_get_by_name(np, CORE_CLK);
	if (IS_ERR(cpu_clk)) {
		P_ERR("failed to get cpu clock, %ld\n",
				PTR_ERR(cpu_clk));
		ret = PTR_ERR(cpu_clk);
		goto free_mem;
	}
	c->clks[0] = cpu_clk;
	for (ic = 1; ic < SPRD_MAX_CPUS_EACH_CLUSTER; ic++) {
		sprintf(coreclk, "core%d_clk", ic);
		c->clks[ic] = of_clk_get_by_name(np, coreclk);
		if (!IS_ERR(c->clks[ic]))
			P_DBG("got %s\n", coreclk);
	}
	c->clk_low_freq_p =
		of_clk_get_by_name(np, LOW_FREQ_CLK_PARENT);
	if (IS_ERR(c->clk_low_freq_p)) {
		P_INF("clk_low_freq_p is not defined\n");
		clk_low_freq_p_max = 0;
	} else {
		clk_low_freq_p_max =
			clk_get_rate(c->clk_low_freq_p);
		P_DBG("clk_low_freq_p_max[%lu]Khz\n",
			clk_low_freq_p_max / 1000);
	}

	c->clk_high_freq_p =
		of_clk_get_by_name(np, HIGH_FREQ_CLK_PARENT);
	if (IS_ERR(c->clk_high_freq_p)) {
		P_ERR("failed in getting clk_high_freq_p %ld\n",
			PTR_ERR(c->clk_high_freq_p));
		ret = PTR_ERR(c->clk_high_freq_p);
		goto free_clk;
	}

	if (of_property_read_u32(np, "clock-latency", &transition_latency))
		transition_latency = CPUFREQ_ETERNAL;
	if (of_property_read_u32(np, "voltage-tolerance", &volt_tol))
		volt_tol = DEF_VOLT_TOL;
	of_property_read_u32(np, "cpu-gpu-vdd", &cpu_gpu_flag);
	if (cpu_gpu_flag == 1)
		aon_apb_reg_base = syscon_regmap_lookup_by_phandle(np, "sprd,syscon-aon-apb");
	P_DBG("value of transition_latency %u, voltage_tolerance %u, cpu-gpu-vdd %d\n",
			transition_latency, volt_tol, cpu_gpu_flag);

	cpu_reg = sprd_volt_share_reg(c);
	if (cpu_reg == NULL)
		cpu_reg = devm_regulator_get(cpu_dev, CORE_SUPPLY);
	if (cpu_reg == NULL || IS_ERR(cpu_reg)) {
		P_ERR("failed to get regulator, %ld\n",
			PTR_ERR(cpu_reg));
		ret = PTR_ERR(cpu_reg);
		goto free_clk;
	}

	if (cpu_gpu_flag == 1)
		gpu_cpu_reg = cpu_reg;

	/*TODO: need to get new temperature from thermal zone after hotplug*/
	if (cpufreq_datas[0] != NULL)
		c->temp_now = cpufreq_datas[0]->temp_now;
	ret = dev_pm_opp_of_add_table_binning(is_big_cluster(cpu),
		cpu_dev, np, c);

	if (ret < 0) {
		P_ERR("failed to init opp table, %d\n", ret);
		goto free_reg;
	}

	if (cpu_reg != NULL && !IS_ERR(cpu_reg)) {
		P_DBG("going to verify opp with regulator\n");
		ret = sprd_verify_opp_with_regulator(cpu_dev,
				cpu_reg, volt_tol);
		if (ret > 0)
			transition_latency += ret * 1000;
	}

	P_DBG("going to initialize freq_table\n");
	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		P_ERR("%d in initializing freq_table\n", ret);
		goto free_opp;
	}

	ret = cpufreq_table_validate_and_show(policy, freq_table);
	if (ret) {
		P_ERR("invalid frequency table: %d\n", ret);
		goto free_table;
	}
	P_DBG("going to prepare clock\n");

	if (!c->clk_en) {
		ret = clk_prepare_enable(cpu_clk);
		if (ret) {
			P_ERR("CPU%d clk_prepare_enable failed\n",
				policy->cpu);
				goto free_table;
		}
		for (ic = 1;
		ic < SPRD_MAX_CPUS_EACH_CLUSTER && !IS_ERR(c->clks[ic]);
		ic++) {
			P_DBG("going to prepare clock core%d\n", ic);
			ret = clk_prepare_enable(c->clks[ic]);
			if (ret) {
				P_ERR("CPU%d clk_enable core%d failed\n",
					policy->cpu, ic);
					goto free_table;
			}
		}
		c->clk_en = 1;
	}
	#ifdef CONFIG_SMP
	/* CPUs in the same cluster share a clock and power domain. */
	cpumask_or(policy->cpus, policy->cpus, cpu_coregroup_mask(policy->cpu));
	#endif

	c->online = 1;
	c->cpu_dev = cpu_dev;
	c->reg = cpu_reg;
	c->clk = cpu_clk;
	c->volt_tol = volt_tol;
	c->clk_low_freq_p_max = clk_low_freq_p_max;

	if (c->cluster < SPRD_CPUFREQ_MAX_CLUSTER &&
	cpufreq_datas[c->cluster] == NULL) {
		cpufreq_datas[c->cluster] = c;
		P_DBG("cpu%d got new cpufreq_data\n", cpu);
	}

	ret = sprd_cpufreq_init_slaves(c, np);
	if (ret)
		goto free_table;

	mutex_unlock(c->volt_lock);

	policy->driver_data = c;
	policy->clk = cpu_clk;
	policy->suspend_freq = freq_table[0].frequency;
	if (sprd_cpufreqhw_probed(c->cluster))
		policy->cur = sprd_cpufreqhw_get(c->cluster);
	else
		policy->cur = clk_get_rate(cpu_clk) / 1000;
	policy->cpuinfo.transition_latency = transition_latency;
	freq_Hz = policy->cur*1000;
	rcu_read_lock();
	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq_Hz);
	c->volt_req = dev_pm_opp_get_voltage(opp);
	c->freq_req = dev_pm_opp_get_freq(opp);
	rcu_read_unlock();


	if (sprd_cpufreqhw_probed(c->cluster)) {
		sprd_cpufreqhw_enable(c->cluster, true);
	} else {
		if (freq_Hz != policy->cur*1000)
			sprd_cpufreq_set_target(c, 0, true);
	}

	if (cpu_gpu_flag == 1) {
		down_voltage_sem();
		set_cpu_target_voltage(c->volt_req);
		up_voltage_sem();
	}

	P_INF("init cpu%d is ok, freq=%ld, freq_req=%ld, volt_req=%ld\n",
		cpu, freq_Hz, c->freq_req, c->volt_req);

	goto free_np;

free_table:
	if (policy->freq_table != NULL)
		dev_pm_opp_free_cpufreq_table(cpu_dev,
			&policy->freq_table);
free_opp:
	dev_pm_opp_of_remove_table(cpu_dev);
free_reg:
	if (!IS_ERR(cpu_reg))
		devm_regulator_put(cpu_reg);
free_clk:
	if (!IS_ERR(cpu_clk)) {
		clk_put(cpu_clk);
		policy->clk = ERR_PTR(-ENOENT);
		c->clks[0] = ERR_PTR(-ENOENT);
		for (ic = 1; ic < SPRD_MAX_CPUS_EACH_CLUSTER; ic++) {
			if (!IS_ERR(c->clks[ic]))
				clk_put(c->clks[ic]);
			c->clks[ic] = ERR_PTR(-ENOENT);
		}
	}
	if (!IS_ERR(c->clk_low_freq_p))
		clk_put(c->clk_low_freq_p);
	if (!IS_ERR(c->clk_high_freq_p))
		clk_put(c->clk_high_freq_p);
free_mem:
	if (c->volt_lock) {
		mutex_unlock(c->volt_lock);
		mutex_destroy(c->volt_lock);
		kfree(c->volt_lock);
	}
	kfree(c);
	sprd_cpufreq_data(cpu) = NULL;
free_np:
	if (np)
		of_node_put(np);
	if (np1)
		of_node_put(np1);
	if (cpu_np)
		of_node_put(cpu_np);
	return ret;
}

static int sprd_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct sprd_cpufreq_driver_data *c;
	int ic;

	if (!policy)
		return -ENODEV;

	P_INF("releasing resources for cpu %d\n", policy->cpu);
	c = policy->driver_data;
	if (c == NULL)
		return 0;

	mutex_lock(c->volt_lock);

	if (policy->freq_table != NULL)
		dev_pm_opp_free_cpufreq_table(c->cpu_dev,
			&policy->freq_table);

	sprd_cpufreqhw_enable(topology_physical_package_id(policy->cpu), false);

	if (!IS_ERR(policy->clk)) {
		clk_put(policy->clk);
		policy->clk = ERR_PTR(-ENOENT);
		c->clk = ERR_PTR(-ENOENT);
		c->clks[0] = ERR_PTR(-ENOENT);
		for (ic = 1; ic < SPRD_MAX_CPUS_EACH_CLUSTER; ic++) {
			if (!IS_ERR(c->clks[ic]))
				clk_put(c->clks[ic]);
			c->clks[ic] = ERR_PTR(-ENOENT);
		}
	}
	if (!IS_ERR(c->clk_low_freq_p)) {
		clk_put(c->clk_low_freq_p);
		c->clk_low_freq_p = ERR_PTR(-ENOENT);
	}
	if (!IS_ERR(c->clk_high_freq_p)) {
		clk_put(c->clk_high_freq_p);
		c->clk_high_freq_p = ERR_PTR(-ENOENT);
	}

	policy->driver_data = NULL;
	if (cpu_gpu_flag == 1) {
		regmap_exit(aon_apb_reg_base);
		aon_apb_reg_base = NULL;
	}
	mutex_unlock(c->volt_lock);
	return 0;
}

static int sprd_cpufreq_table_verify(struct cpufreq_policy *policy)
{
	return cpufreq_generic_frequency_table_verify(policy);
}


static unsigned int sprd_cpufreq_get(unsigned int cpu)
{
	if (sprd_cpufreqhw_probed(topology_physical_package_id(cpu)))
		return sprd_cpufreqhw_get(topology_physical_package_id(cpu));
	else
		return cpufreq_generic_get(cpu);
}

static int sprd_cpufreq_suspend(struct cpufreq_policy *policy)
{
	/*do not change freq in userpace mode*/
	if (policy && !strcmp(policy->governor->name, "userspace")) {
		P_DBG("%s: do nothing for governor->name %s\n",
			__func__, policy->governor->name);
		return 0;
	}

	return cpufreq_generic_suspend(policy);
}

static int sprd_cpufreq_resume(struct cpufreq_policy *policy)
{
	/*do not change freq in userpace mode*/
	if (policy && !strcmp(policy->governor->name, "userspace")) {
		P_DBG("%s: do nothing for governor->name %s\n",
			__func__, policy->governor->name);
		return 0;
	}

	return cpufreq_generic_suspend(policy);
}

/**
 * sprd_cpufreq_set_boost: set cpufreq driver to be boost mode.
 *
 * @state: 0->disable boost mode;1->enable boost mode;
 *
 * Return: zero on success, otherwise non-zero on failure.
 */
static int sprd_cpufreq_set_boost(int state)
{
	/*
	  *if (boost_mode_flag != state && state) {
	  *TODO: if cpufreq.c set boost mode,
	  *dvfs driver should set max freq on all CPUs here
	  *}
	  */

	/*
	  *cpufreq.c caller takes control of cpufreq driver boost
	  *disalbe boot_done_timestamp function at once
	  */
	boot_done_timestamp = 0;
	boost_mode_flag = state;
	P_INF("sprd_cpufreq_set_boost=%d\n", boost_mode_flag);

	return 0;
}

static struct cpufreq_driver sprd_cpufreq_driver = {
	.name = "sprd-cpufreq",
	.flags = CPUFREQ_STICKY
			| CPUFREQ_NEED_INITIAL_FREQ_CHECK
			| CPUFREQ_HAVE_GOVERNOR_PER_POLICY,
	.init = sprd_cpufreq_init,
	.exit = sprd_cpufreq_exit,
	.verify = sprd_cpufreq_table_verify,
	.target_index = sprd_cpufreq_set_target_index,
	.get = sprd_cpufreq_get,
	.suspend = sprd_cpufreq_suspend,
	.resume = sprd_cpufreq_resume,
	.attr = cpufreq_generic_attr,
	/* platform specific boost support code */
	.boost_supported = true,
	.boost_enabled = true,
	.set_boost = sprd_cpufreq_set_boost,
};
static int sprd_cpufreq_cpu_callback(struct notifier_block *nfb,
	unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;
	unsigned int olcpu = 0;
	unsigned int volt_tol_min = 0, index = 0;
	unsigned long volt_max_online = 0;
	struct sprd_cpufreq_driver_data *c;

	c = sprd_cpufreq_data(cpu);

	if (c == NULL || (c && c->volt_share_masters_bits == 0))
		return NOTIFY_OK;
	if (!is_big_cluster(cpu))
		return NOTIFY_OK;

	/* Big is online then dont proceed */
	for_each_online_cpu(olcpu)
		if (is_big_cluster(olcpu))
			return NOTIFY_OK;

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_UP_PREPARE:
		P_INF("BIG CLUSTER ON: vol to old_vol of cpu %u\n", cpu);

		/*For HW DVFS, no need reovering volt*/
		if (sprd_cpufreqhw_probed(c->cluster))
			break;

		mutex_lock(c->volt_lock);
		volt_tol_min = sprd_volt_tol_min(c, true);
		sprd_volt_req_max(c, &volt_max_online, true, true);

		if (c->volt_req > volt_max_online) {
			if (regulator_set_voltage_tol(c->reg,
				c->volt_req,
				volt_tol_min))
				P_ERR("fail to set voltage %lu\n",
					c->volt_req);
			else
				sprd_volt_share_slaves_notify(
					sprd_cpufreq_data(cpu),
					c->volt_req);
		}
		c->online = 1;
		mutex_unlock(c->volt_lock);

		P_DBG("volt_req %lu, volt_max_online %lu, new volt=%lu\n",
			c->volt_req, volt_max_online,
			c->volt_req > volt_max_online ?
			c->volt_req : volt_max_online);
		/*notify slaves recover freq*/
		for (index = 0; index < c->freqvolts; index++)
			if (c->volt_req >= c->freqvolt[index].volt)
				break;
		if (index < c->freqvolts)
			sprd_freq_sync_slaves_notify(c,
				c->freqvolts - 1 - index, true);
		break;
	case CPU_UP_CANCELED:
	case CPU_POST_DEAD:
		P_INF("BIG CLUSTER OFF: vol to 0 of cpu %u\n", cpu);
		mutex_lock(c->volt_lock);
		c->online = 0;
		mutex_unlock(c->volt_lock);

		/*
		 * For HW DVFS, set master core lowest freq& low volt
		 *   notify slaves set min freq
		 */
		sprd_freq_sync_slaves_notify(c, 0, true);

		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block __refdata sprd_cpufreq_cpu_notifier = {
	.notifier_call = sprd_cpufreq_cpu_callback,
};

static int  sprd_cpufreq_probe(struct platform_device *pdev)
{
	struct device *cpu_dev = NULL;
	struct clk *cpu_clk = NULL;
	struct regulator *cpu_reg = NULL;
	struct device_node *cpu_np;
	struct device_node *np = NULL, *np1 = NULL;
	int ret = 0, i;
	/* doing probe only for cpu 0 */
	int cpu = 0;

	boot_done_timestamp = jiffies + SPRD_CPUFREQ_DRV_BOOST_DURATOIN;

	P_DBG("cpufreq probe begins\n");
	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev) {
		P_ERR("failed to get cpu%d device\n", cpu);
		return -ENODEV;
	}

	cpu_np = of_node_get(cpu_dev->of_node);
	if (!cpu_np) {
		P_ERR("failed to find cpu node\n");
		return -ENODEV;
	}

	np = of_parse_phandle(cpu_np, "cpufreq-data", 0);
	np1 = of_parse_phandle(cpu_np, "cpufreq-data-v1", 0);
	if (!np && !np1) {
		P_ERR("failed to find cpufreq-data for cpu%d\n", cpu);
		of_node_put(cpu_np);
		return -ENOENT;
	}
	if (np1)
		np = np1;

	cpu_clk = of_clk_get_by_name(np, CORE_CLK);
	if (IS_ERR(cpu_clk)) {
		P_ERR("failed in clock getting, %ld\n",
				PTR_ERR(cpu_clk));
		ret = PTR_ERR(cpu_clk);
		goto put_np;
	}

	cpu_reg = devm_regulator_get(cpu_dev, CORE_SUPPLY);
	if (IS_ERR(cpu_reg)) {
		P_ERR("failed  in regulator getting, %ld\n",
			PTR_ERR(cpu_reg));
		ret = PTR_ERR(cpu_reg);
		goto put_clk;
	}

/* Put regulator and clock here, before registering the driver
 * we will get them again while per cpu initialization in cpufreq_init
*/
	if (!IS_ERR(cpu_reg)) {
		P_DBG("putting regulator\n");
		devm_regulator_put(cpu_reg);
	}

	for (i = 0; i < SPRD_CPUFREQ_MAX_MODULE; i++)
		cpufreq_datas[i] = NULL;

put_clk:
	if (!IS_ERR(cpu_clk)) {
		P_DBG("putting clk\n");
		clk_put(cpu_clk);
	}

put_np:
	if (np)
		of_node_put(np);
	if (np1)
		of_node_put(np1);
	if (cpu_np)
		of_node_put(cpu_np);

	/* ret is not zero? we encountered an error.
	 * return failure/probe deferred
	*/
	if (ret)
		return ret;

	P_INF("going to register cpufreq driver\n");
	ret = cpufreq_register_driver(&sprd_cpufreq_driver);
	if (ret)
		P_INF("cpufreq driver register failed %d\n", ret);
	else
		P_INF("cpufreq driver register succcess\n");
	register_hotcpu_notifier(&sprd_cpufreq_cpu_notifier);

	return ret;
}

static int sprd_cpufreq_remove(struct platform_device *pdev)
{
	return cpufreq_unregister_driver(&sprd_cpufreq_driver);
}

static struct platform_driver sprd_cpufreq_platdrv = {
	.driver = {
		.name	= "sprd-cpufreq",
		.owner	= THIS_MODULE,
	},
	.probe		= sprd_cpufreq_probe,
	.remove		= sprd_cpufreq_remove,
};

module_platform_driver(sprd_cpufreq_platdrv);

/* If required, we can move device registration code in other file */
static struct platform_device sprd_cpufreq_pdev = {
	.name = "sprd-cpufreq",
};

static int  __init sprd_cpufreq_init_pdev(void)
{
	return platform_device_register(&sprd_cpufreq_pdev);
}

device_initcall(sprd_cpufreq_init_pdev);

MODULE_DESCRIPTION("sprd cpufreq driver");
MODULE_LICENSE("GPL");
