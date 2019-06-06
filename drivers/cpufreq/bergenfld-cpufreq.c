/*
 *  Performance States Driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  Author: Subin Gangadharan <subin.k.gangadharan@intel.com>
 */

#include <linux/regulator/consumer.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/freq-vid.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/pm_opp.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/regmap.h>
#include <asm/msr.h>
#include <asm/processor.h>

#ifdef CONFIG_MOBILEVISOR
#include <asm/mv/mobilevisor.h>
#include <asm/mv/pm/mv_svc_pm.h>
#endif

#define RATIO_TO_KHZ(v)	(v * 1000 * host_bus_freq())
#define FRAC_BITS 8
#define int_tofp(X) ((int64_t)(X) << FRAC_BITS)
#define fp_toint(X) ((X) >> FRAC_BITS)

#define ATOM_TURBO_RATIOS	0x66c

#define MAX_NUM_FREQS	6
#define MAX_NUM_MODULES	2
#define CORES_PER_MOD	4

struct pstate_data {
	u8 lfm_index;
	u8 hfm_index;
	u32 freqs[MAX_NUM_FREQS];
	u16 ctrl_vals[MAX_NUM_FREQS];
	u32 boost_freq;
	u8 boost_index;
	u8 boost_supported;
	u8 no_of_pstates;
};

struct bfd_freq_desc {
	u8 x86_family;
	u8 x86_model;
	u8 x86_mask;
	u8 sku;
	struct pstate_data pd[MAX_NUM_MODULES];
};

static struct bfd_freq_desc bfd_freq_desc_tables[] = {
	/* BTR3 */
	{
		.x86_family = 6,
		.x86_model = 0x75,
		.x86_mask = 0x8,
		.sku = 0,
		.pd[0] = {
			.lfm_index = 0,
			.hfm_index = 5,
			.freqs = { 0, 936, 1248, 1560, 1872, 0 },
			.ctrl_vals = {
				VID_V2, VID_V3, VID_V4, VID_V5, VID_V6, VID_V7
			},
			.boost_freq = 0,
			.boost_index = 0,
			.no_of_pstates = NO_OF_PSTATES,
			.boost_supported = 0,
		},
		.pd[1] = {
			.lfm_index = 0,
			.hfm_index = 5,
			.freqs = { 0, 936, 1248, 1560, 1872, 0 },
			.ctrl_vals = {
				VID_V2, VID_V3, VID_V4, VID_V5, VID_V6, VID_V7
			},
			.boost_freq = 0,
			.boost_index = 0,
			.no_of_pstates = NO_OF_PSTATES,
			.boost_supported = 0,
		},
	},
	/* BTR2R SKU HPM */
	{
		.x86_family = 6,
		.x86_model = 0x75,
		.x86_mask = 0xa,
		.sku = 0,
		.pd[0] = {
			.lfm_index = 0,
			.hfm_index = 4,
			.freqs = { 0, 936, 1248, 1560, 0, 0 },
			.ctrl_vals = {
				VID_V2, VID_V3, VID_V4, VID_V5, VID_V6, VID_V7
			},
			.boost_freq = 0,
			.boost_index = 5,
			.boost_supported = 1,
			.no_of_pstates = NO_OF_PSTATES,
		},
		.pd[1] = {
			.lfm_index = 0,
			.hfm_index = 4,
			.freqs = { 0, 936, 1248, 1560, 0},
			.ctrl_vals = {VID_V2, VID_V3, VID_V4, VID_V5, VID_V6},
			.boost_freq = 0,
			.boost_index = 0,
			.boost_supported = 0,
			.no_of_pstates = NO_OF_PSTATES - 1,
		},
	},
	/* BTR2R SKU 4 core */
	{
		.x86_family = 6,
		.x86_model = 0x75,
		.x86_mask = 0xa,
		.sku = 1,
		.pd[0] = {
			.lfm_index = 0,
			.hfm_index = 3,
			.freqs = { 0, 936, 1248, 0, 0 },
			.ctrl_vals = {VID_V2, VID_V3, VID_V4, VID_V5, VID_V7},
			.boost_freq = 0,
			.boost_index = 4,
			.boost_supported = 1,
			.no_of_pstates = NO_OF_PSTATES - 1,
		},
	},
	/* BTR2R SKU LPM */
	{
		.x86_family = 6,
		.x86_model = 0x75,
		.x86_mask = 0xa,
		.sku = 2,
		.pd[0] = {
			.lfm_index = 0,
			.hfm_index = 3,
			.freqs = { 0, 936, 1248, 0, 0},
			.ctrl_vals = {VID_V2, VID_V3, VID_V4, VID_V5, VID_V7},
			.boost_freq = 0,
			.boost_index = 4,
			.boost_supported = 1,
			.no_of_pstates = NO_OF_PSTATES - 1,
		},
		.pd[1] = {
			.lfm_index = 0,
			.hfm_index = MAX_NUM_FREQS,
			 /* TODO how to read module 1 limit freq */
			.freqs = { 0, 1092},
			.ctrl_vals = {VID_V2, VID_V3},
			.boost_freq = 0,
			.boost_index = 0,
			.boost_supported = 0,
			.no_of_pstates = NO_OF_PSTATES - 4,
		},
	}
};

static int match_cpu(u8 family, u8 model, u8 mask, u8 sku)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bfd_freq_desc_tables); i++) {
		if ((family == bfd_freq_desc_tables[i].x86_family) &&
			(model == bfd_freq_desc_tables[i].x86_model) &&
			(mask == bfd_freq_desc_tables[i].x86_mask) &&
			(sku == bfd_freq_desc_tables[i].sku))
			return i;
	}

	return -1;
}

static struct bfd_freq_desc *bfd_freq_table;

static struct cpufreq_frequency_table		*freq_table[MAX_NUM_MODULES] = {
	NULL, NULL
};
static struct pmic_vid __iomem			*vt_mod0, *vt_mod1;
static void __iomem				*dvfs_mod0, *dvfs_mod1;

static struct freq_attr *bfd_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,   /* for boost-attr if needed */
	NULL,
};

/* Butter3 configuration document	*/
/* static int dbg_freq_ratios[] = {4, 8, 10, 12, 14, 16, 18,\
*					20, 22, 24, 26, 28, 30, 32, 0};
*/

#if defined(CONFIG_SOC_IWHALE2)
static u32 NO_OF_PSTATES_VAR = NO_OF_PSTATES;
#endif
static u32 vol_table[] = {800000, 840000, 930000, 1020000, 1110000, 1200000};

#define CPUFREQ_BOOT_TIME	(50*HZ)
static unsigned long boot_done;

struct regulator *vddcpu0, *vddcpu1;

static bool pmic_wa_enabled;

static int __init pmic_wa_enable(char *str)
{
	pmic_wa_enabled = true;
	return 1;
}
__setup("pmic2731_wa", pmic_wa_enable);

static unsigned int boost_fuse_wa;

static int __init parse_boost_fuse_wa(char *str)
{
	int ret;
	unsigned long val;

	if (!str)
		return -EINVAL;

	ret = kstrtoul(str, 0, &val);
	if (ret)
		return ret;

	boost_fuse_wa = val;
	pr_info("boost_fuse_wa set to 0x%x\n", boost_fuse_wa);

	return 1;
}
__setup("boost_fuse_wa=", parse_boost_fuse_wa);

static inline int32_t mul_fp(int32_t x, int32_t y)
{
	return ((int64_t)x * (int64_t)y) >> FRAC_BITS;
}

static inline int32_t div_fp(s64 x, s64 y)
{
	return div64_s64((int64_t)x << FRAC_BITS, y);
}

static inline int ceiling_fp(int32_t x)
{
	int mask, ret;

	ret = fp_toint(x);
	mask = (1 << FRAC_BITS) - 1;
	if (x & mask)
		ret += 1;
	return ret;
}

/** Host Bus Frequency
 *  In intel platforms MSR 0xcd is used to calculate the FSB.
 *  However in butter platform this MSR is not avaialable so
 *  hard coding it.
 */

static inline int host_bus_freq(void)
{
	return FSB_FREQ_MHZ;
}

static inline int get_guar_ratio(void)
{
#if defined(CONFIG_SOC_IWHALE2)
	/*workaround,set max freq to be 2028*/
	return 26;
#else
	u32 low, high;
	union ia_core_ratios ratios;

	rdmsr(MSR_CORE_TO_BUS_RATIOS, low, high);
	ratios.raw = low;
#ifdef VP
	ratios.field.guar_ratio = 0x25;
#endif
	return ratios.field.guar_ratio;
#endif
}

static inline int get_lfm_ratio(void)
{

	u32 low, high;
	union ia_core_ratios ratios;

	rdmsr(MSR_CORE_TO_BUS_RATIOS, low, high);
	ratios.raw = low;
#ifdef VP
	ratios.field.lfm_ratio = 8;
#endif
	return ratios.field.lfm_ratio;
}

static inline int get_boost_ratio(void)
{
	u32 low, high;
	int ratio;

	if (boost_fuse_wa) {
		ratio = boost_fuse_wa;
	} else {
		rdmsr(ATOM_TURBO_RATIOS, low, high);
		ratio = low & 0x3F; /* bit 5:0 MAX_RATIO_1C */
	}
#ifdef VP
	ratio = 0x25;
#endif
	return ratio;
}

static int pstate_msrs_not_valid(void)
{
	if (!get_guar_ratio())
		return -ENODEV;
	return 0;
}

static u8 get_sku(void)
{
	int guar_ratio;

	if (boot_cpu_data.x86 == 0x6 &&
			boot_cpu_data.x86_model == 0x75 &&
			boot_cpu_data.x86_mask == 0xa) { /*BTR2R*/
		guar_ratio = get_guar_ratio();
		if (guar_ratio >= 0x18) {
			return 0; /* HPM */
		} else if (setup_max_cpus == 4) {
			/* TODO how to identify 4C sku */
			return 1; /* 4C */
		} else {
			return 2; /* LPM */
		}
	} else {
		return 0;
	}
}

static int select_freq_table(void)
{
	int index;
	u8 sku;

	sku = get_sku();
	index = match_cpu(boot_cpu_data.x86, boot_cpu_data.x86_model,
			boot_cpu_data.x86_mask, sku);
	if (index < 0)
		return -ENODEV;
	bfd_freq_table = &bfd_freq_desc_tables[index];
	return 0;
}

static inline int is_eist_supported(void)
{
	u32 eax = 0, ebx = 0, ecx = 0, edx = 0;

	eax = 1;
	native_cpuid(&eax, &ebx, &ecx, &edx);
	if (ecx && (1 << 7))
		return 1;
	return 0;
}

/*
* pmic formula to calculate voltage in mv
* mv = 400 + ctrl[9:5] * 100 + cal[4:0] * 3.125
*/
static int get_pmic_voltage(int module, int index)
{
	int cal, ctrl, mv;
	struct pmic_vid *v;

	v = (module == MODULE0) ? (vt_mod0 + index) : (vt_mod1 + index);
	/* cal[4:0] * 3.125	*/
	cal = mul_fp(int_tofp(v->cal), PMIC_CAL_PRECISION);
	/* v->ctrl * 100 */
	ctrl = mul_fp(int_tofp(v->ctrl), int_tofp(PMIC_CTRL_MUL));
	/*
	 * pmic formula to calculate voltage in mv
	 * mv = 400 + ctrl[9:5] * 100 + cal[4:0] * 3.125
	 */
	mv = int_tofp(PMIC_VLT_INTERCEPT) + cal + ctrl;
	mv = ceiling_fp(mv);

#ifdef VP
	mv = ((index == VID_V2) ? 750 : 1200);
#endif
	return mv;

}


static unsigned int bergenfld_cpufreq_get(unsigned int cpu)
{

	u32 lo, hi;
	preempt_disable();
	if (cpu == smp_processor_id()) {
		rdmsr(MSR_IA32_PERF_STATUS, lo, hi);
		preempt_enable();
	} else {
		preempt_enable();
		rdmsr_on_cpu(cpu, MSR_IA32_PERF_STATUS, &lo, &hi);
	}
	lo >>= 0x8;
	lo &= 0xFF;

#ifdef VP
	{
		u32 ret;
		struct cpufreq_policy policy;

		ret = cpufreq_get_policy(&policy, cpu);
		if (ret) {
			pr_err("%s %d failed\n", __func__, __LINE__);
			return ret;
		}
		if (unlikely(!policy.cur))
			return policy.min;
		return policy.cur;
	}
#else

	return RATIO_TO_KHZ(lo);
#endif
}

#ifdef VP
static int bergenfld_cpufreq_target(struct cpufreq_policy *policy,
		unsigned int index)
{
	u32 next_pstate; /* Index into perf table */
	u32 lo;
	u16 ctrl_val;
	int module;

	next_pstate = policy->freq_table[index].driver_data;
	module = (policy->cpu < 4) ? MODULE0 : MODULE1;
	ctrl_val = bfd_freq_table->pd[module].ctrl_vals[next_pstate];
	lo = ((u32) ctrl_val &
		 INTEL_PERF_CTL_MASK);
	lo >>= 8;
	lo &= 0xFF;
	lo = RATIO_TO_KHZ(lo);

	pr_debug("Programmed freq %7u KHz core %d\n", lo, policy->cpu);
	pr_debug("control %x cpu %d\n", ctrl_val, policy->cpu);

	return 0;

}
#else
struct pstate_info {
	struct cpufreq_policy *policy;
	unsigned int index;
	struct timer_list *timer;
};
static void bergenfld_pstate_write_new(void *info);
static void pstate_write(unsigned long data)
{
	struct pstate_info *pinfo = (struct pstate_info *)data;

	bergenfld_pstate_write_new(pinfo);
}
static struct pstate_info info0, info1;
static DEFINE_TIMER(pstate_timer0, pstate_write, 0, (unsigned long)&info0);
static DEFINE_TIMER(pstate_timer1, pstate_write, 0, (unsigned long)&info1);

static void bergenfld_pstate_write_new(void *info)
{
	struct cpufreq_policy *policy;
	struct pstate_info *pinfo = (struct pstate_info *)info;
	u32 next_pstate; /* Index into perf table */
	u32 lo, hi;
	u32 req;
	u16 ctrl_val;
	int module;
	int cpu;
	static int count[2];

	preempt_disable();
	cpu = smp_processor_id();
	preempt_enable();
	policy = pinfo->policy;
	if (cpu != policy->cpu) {
		pr_err("notmatch cur cpu %d, target cpu %d, retrigger it\n",
			cpu, policy->cpu);
		smp_call_function_single(policy->cpu,
			bergenfld_pstate_write_new, (void *)pinfo, 0);
		return;
	}
	rdmsr(MSR_IA32_PERF_CTL, lo, hi);
	req = lo;
	req >>= 8;
	req &= 0xFF;
	req = RATIO_TO_KHZ(req);
	module = (policy->cpu < 4) ? MODULE0 : MODULE1;
	if (req != 0 && req != bergenfld_cpufreq_get(policy->cpu))  {
		mod_timer_pinned(pinfo->timer, jiffies+msecs_to_jiffies(1));
		count[module]++;
		if (count[module] > 100) {
			count[module] = 0;
			pr_err("cpufreq limited due to thermal?\n");
		}
		return;
	}
	count[module] = 0;
	next_pstate = policy->freq_table[pinfo->index].driver_data;
	ctrl_val = bfd_freq_table->pd[module].ctrl_vals[next_pstate];
	lo = (lo & ~INTEL_PERF_CTL_MASK) |
		((u32) ctrl_val &
		INTEL_PERF_CTL_MASK);
	wrmsr(MSR_IA32_PERF_CTL, lo, hi);
	lo >>= 8;
	lo &= 0xFF;
	lo = RATIO_TO_KHZ(lo);
	pr_debug("Programmed freq %7u KHz core %d\n", lo, policy->cpu);
	pr_debug("control %x cpu %d\n", ctrl_val, policy->cpu);
	pr_debug("Probing real freq %7u KHz core %d\n",
		bergenfld_cpufreq_get(policy->cpu), policy->cpu);
}

static void bergenfld_pstate_write(void *info)
{
	struct cpufreq_policy *policy;
	struct pstate_info *pinfo = (struct pstate_info *)info;
	u32 next_pstate; /* Index into perf table */
	u32 lo, hi, tmp_stat;
	u32 retry = 10000;
	u32 last_req = 0;
	unsigned long flags;
	u16 ctrl_val;
	int module;

	local_irq_save(flags);
	preempt_disable();
	policy = pinfo->policy;
	rdmsr(MSR_IA32_PERF_CTL, lo, hi);
	lo >>= 8;
	lo &= 0xFF;
	lo = RATIO_TO_KHZ(lo);
	last_req = lo;
	while (last_req != 0 && ((tmp_stat = bergenfld_cpufreq_get(policy->cpu))
		!= last_req) && --retry) {
		udelay(10);
	}
	if (retry == 0) {
		pr_err("HW cur not aligned with SW cur\n");
		pr_err("req %7u cur stat %7u,cpu %d\n", last_req,
			tmp_stat, policy->cpu);
		WARN_ON(1);
	}
	next_pstate = policy->freq_table[pinfo->index].driver_data;
	rdmsr(MSR_IA32_PERF_CTL, lo, hi);
	module = (policy->cpu < 4) ? MODULE0 : MODULE1;
	ctrl_val = bfd_freq_table->pd[module].ctrl_vals[next_pstate];
	lo = (lo & ~INTEL_PERF_CTL_MASK) |
		((u32) ctrl_val &
		 INTEL_PERF_CTL_MASK);
	wrmsr(MSR_IA32_PERF_CTL, lo, hi);
	lo >>= 8;
	lo &= 0xFF;
	lo = RATIO_TO_KHZ(lo);
	/* Final driver this will be removed */
	retry = 1000;
	while (((tmp_stat = bergenfld_cpufreq_get(policy->cpu)) != lo)
		&& --retry) {
		udelay(10);
	}
	if (retry == 0) {
		pr_err("curr freq req not finished\n");
		pr_err("req %7u cur stat %7u,cpu %d\n", lo,
			tmp_stat, policy->cpu);
		WARN_ON(1);
	}

	pr_debug("Programmed freq %7u KHz core %d\n", lo, policy->cpu);
	pr_debug("control %x cpu %d\n", ctrl_val, policy->cpu);
	pr_debug("Probing real freq %7u KHz core %d\n",
		bergenfld_cpufreq_get(policy->cpu), policy->cpu);
	preempt_enable();
	local_irq_restore(flags);
}

static int bergenfld_cpufreq_target(struct cpufreq_policy *policy,
		unsigned int index)
{
	u32 next_pstate; /* Index into perf table */
	u32 next_freq;
	u32 vol_increase_flag = 0;
	struct regulator *vddcpu;
	struct pstate_info info;
	int ret, module;

	if (time_before(jiffies, boot_done)) {
		return 0;
	}

	info.policy = policy;
	info.index = index;

	vddcpu = (policy->cpu < 4) ? vddcpu0 : vddcpu1;
	if (vddcpu != NULL) {
		next_pstate = policy->freq_table[index].driver_data;
		module = (policy->cpu < 4) ? MODULE0 : MODULE1;
		next_freq =
			bfd_freq_table->pd[module].ctrl_vals[next_pstate] >> 8;
		next_freq &= 0xFF;
		next_freq = RATIO_TO_KHZ(next_freq);
		if (next_freq > policy->cur)
			vol_increase_flag = 1;
		if (vol_increase_flag) {
			ret = regulator_set_voltage_tol(vddcpu,
				vol_table[next_pstate], 0);
			if (ret) {
				pr_err("failed to scale up voltage to: %u, ret: %d\n",
						vol_table[next_pstate], ret);
				return ret;
			}
		}
	}
	if (vddcpu != NULL) {
		smp_call_function_single(policy->cpu, bergenfld_pstate_write,
			(void *)&info, 1);
	} else {
		struct pstate_info *pinfo;

		if (policy->cpu < 4)
			pinfo = &info0;
		else
			pinfo = &info1;
		if (timer_pending(pinfo->timer))
			del_timer_sync(pinfo->timer);
		pinfo->policy = policy;
		pinfo->index = index;
		smp_call_function_single(policy->cpu,
			bergenfld_pstate_write_new, (void *)pinfo, 1);
	}

	if (vddcpu != NULL) {
		if (vol_increase_flag == 0)
			regulator_set_voltage_tol(vddcpu,
				vol_table[next_pstate], 0);
	}

	return 0;
}
#endif

#define SINGLE_PCTL_ENABLE	(1<<11)

static int bergenfld_cpufreq_cpu_init(struct cpufreq_policy *policy)
{

	int ret, module;
	u32 lo, hi, i, cpu_id, freq;
	struct device *cpu = NULL;
	u8 no_of_pstates;


	pr_debug("%s cpu %d\n", __func__, policy->cpu);

	module = (policy->cpu < 4) ? MODULE0 : MODULE1;
	/* Single PCTL register */
	rdmsr_on_cpu(policy->cpu, MSR_NHM_SNB_PKG_CST_CFG_CTL, &lo, &hi);
	lo = lo | SINGLE_PCTL_ENABLE;
	wrmsr_on_cpu(policy->cpu, MSR_NHM_SNB_PKG_CST_CFG_CTL, lo, hi);

	rdmsr_on_cpu(policy->cpu, MSR_IA32_MISC_ENABLE, &lo, &hi);
	lo = lo | (1 << MSR_IA32_MISC_ENABLE_ENHANCED_SPEEDSTEP_BIT);

#ifdef VP
	pr_debug("do nothing\n");
#else
	wrmsr_on_cpu(policy->cpu, MSR_IA32_MISC_ENABLE, lo, hi);
#endif

	policy->shared_type = CPUFREQ_SHARED_TYPE_HW;
	policy->cpuinfo.transition_latency = 100000;	/* 100us */

	cpu = get_cpu_device(policy->cpu);
	if (IS_ERR(cpu)) {
		pr_err("cpu %d not found %d\n",
			policy->cpu, (int) PTR_ERR(cpu));
		return -ENODEV;
	}

	no_of_pstates = bfd_freq_table->pd[module].no_of_pstates;
	for (i = 0; i < no_of_pstates; i++) {
		int uv = get_pmic_voltage(module,
				bfd_freq_table->pd[module].ctrl_vals[i] & 0xFF)
			* 1000;

		freq = bfd_freq_table->pd[module].freqs[i];
		ret = dev_pm_opp_add(cpu, freq * 1000000, uv);
		if (ret) {
			pr_err("Adding OPP(%d MHz uV %6d cpu %d) failed %d\n",
					freq, uv, policy->cpu, ret);
			return ret;
		}
		pr_debug("Adding OPP(%d MHz uV %6d cpu %d) success %d\n",
					freq, uv, policy->cpu, ret);
	}

	cpu_id = module * CORES_PER_MOD;

	for (i = 0; i < CORES_PER_MOD; i++)
		cpumask_test_and_set_cpu(cpu_id++, policy->cpus);

	return cpufreq_table_validate_and_show(policy, freq_table[module]);
}

static int bergenfld_cpufreq_resume(struct cpufreq_policy *policy)
{
	u32 lo, hi;

	rdmsr_on_cpu(policy->cpu, MSR_IA32_MISC_ENABLE, &lo, &hi);
	lo = lo | (1 << MSR_IA32_MISC_ENABLE_ENHANCED_SPEEDSTEP_BIT);
	wrmsr_on_cpu(policy->cpu, MSR_IA32_MISC_ENABLE, lo, hi);

	return 0;
}

static void calculate_ctrl_values(void)
{
	int i, j;
	u8 no_of_pstates;

	int fsb = host_bus_freq();

	for (i = 0; i < MAX_NUM_MODULES; i++) {
		no_of_pstates = bfd_freq_table->pd[i].no_of_pstates;
		for (j = 0; j < no_of_pstates; j++) {
			bfd_freq_table->pd[i].ctrl_vals[j] |=
				(bfd_freq_table->pd[i].freqs[j] / fsb) << 8;
			pr_debug("ctrl val[%d][%d] = %x\n", i, j,
					bfd_freq_table->pd[i].ctrl_vals[j]);
		}
#ifdef CONFIG_MOBILEVISOR
		if (is_x86_mobilevisor())
			mv_svc_pm_set_freq_table(i,
				bfd_freq_table->pd[i].ctrl_vals, no_of_pstates,
				bfd_freq_table->pd[i].boost_supported ?
				bfd_freq_table->pd[i].boost_index : 0xFF);
#endif
	}

}

#ifdef VP
static bool is_table_online(int module)
{
	return 1;
}

#else
static bool is_table_online(int module)
{
	int i;
	bool ret = true;
	struct pmic_vid *v;

	v = (module == MODULE0) ? vt_mod0 : vt_mod1;
	for (i = VID_V2; i <= VID_V7; i++) {
		if (!((v + i)->avail)) {
			ret = false;
			break;
		}
	}
	return ret;
}
#endif

static void init_vid_table(void)
{
	int i;
	u8 lfm_index, hfm_index, boost_index;
	u32 lfm_freq, hfm_freq, boost_freq;

	lfm_freq = get_lfm_ratio() * host_bus_freq();
	hfm_freq = get_guar_ratio() * host_bus_freq();
	pr_debug("LFM freq %7d MHz HFM freq %7d MHz\n", lfm_freq,
		hfm_freq);

	for (i = 0; i < MAX_NUM_MODULES; i++) {
		if (bfd_freq_table->pd[i].boost_supported) {
			boost_index = bfd_freq_table->pd[i].boost_index;
			if (boost_index >= 0 && boost_index < MAX_NUM_FREQS) {
				boost_freq = get_boost_ratio()
					* host_bus_freq();
				bfd_freq_table->pd[i].freqs[boost_index]
					= boost_freq;
				bfd_freq_table->pd[i].boost_freq = boost_freq;
				pr_debug("boost freq %7d MHz\n", boost_freq);
			}
		}
		lfm_index = bfd_freq_table->pd[i].lfm_index;
		if (lfm_index >= 0 && lfm_index < MAX_NUM_FREQS)
			bfd_freq_table->pd[i].freqs[lfm_index] = lfm_freq;

		hfm_index = bfd_freq_table->pd[i].hfm_index;
		if (hfm_index >= 0 && hfm_index < MAX_NUM_FREQS)
			bfd_freq_table->pd[i].freqs[hfm_index] = hfm_freq;
	}

}

static int setup_cpufreq_table(void)
{
	int i, j;
	u8 no_of_pstates;
	struct cpufreq_frequency_table *ft;
	struct pstate_data *pd;

	for (j = 0; j < MAX_NUM_MODULES; j++) {
		pd = &bfd_freq_table->pd[j];
		no_of_pstates = pd->no_of_pstates;
		if (no_of_pstates <= 0)
			continue;
		ft = kcalloc((no_of_pstates + 1), sizeof(*ft), GFP_KERNEL);
		if (!ft) {
			pr_err("Memory allocation for freq table failed\n");
			return -ENOMEM;
		}

		freq_table[j] = ft;

		for (i = 0; i < no_of_pstates; i++) {
			ft->driver_data = i;
			ft->frequency = pd->freqs[i] * 1000;

			if (pd->boost_supported
					&& i == pd->boost_index)
				ft->flags |= CPUFREQ_BOOST_FREQ;

			ft++;
		}
		ft->frequency = CPUFREQ_TABLE_END;
	}
	return 0;
}

#if defined(CONFIG_SOC_IWHALE2)
#define AON_VER_ID_A0 0
#define AON_VER_ID_A1 1
static int get_chipid(void)
{
	unsigned int chipid;
	struct regmap *aon_reg =
		 syscon_regmap_lookup_by_compatible("sprd,sys-aon-apb");
	if (IS_ERR_OR_NULL(aon_reg)) {
		pr_err("%s:failed to find sprd reg\n", __func__);
		return -ENODEV;
	}
	regmap_read(aon_reg, REG_AON_APB_AON_VER_ID, &chipid);
	return (int)BIT_AON_APB_AON_VER_ID(chipid);
}
#endif

static struct cpufreq_driver bergenfld_cpufreq_driver = {
	.flags		= CPUFREQ_CONST_LOOPS |
			CPUFREQ_HAVE_GOVERNOR_PER_POLICY,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= bergenfld_cpufreq_target,
	.get		= bergenfld_cpufreq_get,
	.init		= bergenfld_cpufreq_cpu_init,
	.resume		= bergenfld_cpufreq_resume,
	.name		= "bgnfld-cpufreq",
	.attr		= bfd_cpufreq_attr,
};

static int cpu_hotplug_notify(struct notifier_block *n,
				unsigned long action, void *hcpu)
{
	int hotcpu = (unsigned long)hcpu;

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_ONLINE:
		break;
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		if (info0.policy != NULL && hotcpu == info0.policy->cpu) {
			pr_err("hotcpu %d\n", hotcpu);
			del_timer(info0.timer);
		} else if (info1.policy != NULL &&
					hotcpu == info1.policy->cpu) {
			pr_err("hotcpu %d\n", hotcpu);
			del_timer(info1.timer);
		}
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block cpu_hotplug_notifier = {
	.notifier_call = cpu_hotplug_notify,
};

static int __init bergenfld_cpufreq_init(void)
{
	int ret = 0;
	struct device *cpu0, *cpu4;
	int i, j;
	u8 no_of_pstates, boost_supported;

#if defined(CONFIG_SOC_IWHALE2)
	i = get_chipid();
	if (i == AON_VER_ID_A0)
		NO_OF_PSTATES_VAR = NO_OF_PSTATES > 0x4 ? 0x4 : NO_OF_PSTATES;
	pr_info("set num of freq array to be %u for chipid %d\n",
		NO_OF_PSTATES_VAR, i);
#endif
	info0.timer = &pstate_timer0;
	info1.timer = &pstate_timer1;
	__register_cpu_notifier(&cpu_hotplug_notifier);

	boot_done = jiffies + CPUFREQ_BOOT_TIME;

	dvfs_mod0 = ioremap_nocache(ADDR_DVFS_EN_MOD0, PAGE_SIZE);
	dvfs_mod1 = ioremap_nocache(ADDR_DVFS_EN_MOD1, PAGE_SIZE);

	if (IS_ERR(dvfs_mod0) || IS_ERR(dvfs_mod1)) {
		ret = EIO;
		goto err;
	}

	pr_debug("dvfs_mod0 [%p] dvfs_mod1 [%p]\n",
					dvfs_mod0, dvfs_mod1);

	vt_mod0 = (struct pmic_vid *) (dvfs_mod0 + VT_BASE_OFF);
	vt_mod1 = (struct pmic_vid *) (dvfs_mod1 + VT_BASE_OFF);
	pr_debug("vt_mod0 [%p] vt_mod1 [%p]\n",
					vt_mod0, vt_mod1);
	ret = is_table_online(MODULE0) && is_table_online(MODULE1);
	if (!ret) {
		pr_err("Voltage table is not online\n");
		ret = -EIO;
		goto err;
	}

	if (!is_eist_supported()) {
		pr_err("The core doesn't support EIST\n");
		ret = -ENODEV;
		goto err;
	}

	ret = pstate_msrs_not_valid();
	if (ret) {
		pr_err("pstate: invalid msrs\n");
		goto err;
	}

	ret = select_freq_table();
	if (ret) {
		pr_err("cannot select frequency table\n");
		goto err;
	}

#if defined(CONFIG_SOC_IWHALE2)
	for (i = 0; i < MAX_NUM_MODULES; i++)
		bfd_freq_table->pd[i].no_of_pstates = NO_OF_PSTATES_VAR;
#endif

	init_vid_table();

	calculate_ctrl_values();

	ret = setup_cpufreq_table();
	if (ret)
		goto err;

	if (pmic_wa_enabled == true) {
		cpu0 = get_cpu_device(0);
		vddcpu0 = devm_regulator_get(cpu0, "cpu0");
		pr_debug("vddcpu0: %p\n", vddcpu0);
		if (IS_ERR(vddcpu0)) {
			pr_err("no vddcpu0\n");
			WARN_ON(1);
		} else {
			pr_debug("vddcpu0 : 0x%p\n", vddcpu0);
			if (regulator_is_enabled(vddcpu0) > 0)
				pr_debug("vddcpu0 enabled\n");
			else
				pr_err("vddcpu0 disabled\n");
		}
		cpu4 = get_cpu_device(4);
		vddcpu1 = devm_regulator_get(cpu4, "cpu1");
		pr_debug("vddcpu1: %p\n", vddcpu1);
		if (IS_ERR(vddcpu1)) {
			pr_err("no vddcpu1\n");
			WARN_ON(1);
		} else {
			pr_debug("vddcpu1 : 0x%p\n", vddcpu1);
			if (regulator_is_enabled(vddcpu1) > 0)
				pr_debug("vddcpu1 enabled\n");
			else
				pr_err("vddcpu1 disabled\n");
		}
		for (i = 0; i < MAX_NUM_MODULES; i++) {
			no_of_pstates = bfd_freq_table->pd[i].no_of_pstates;
			for (j = 0; j < no_of_pstates; j++) {
				bfd_freq_table->pd[i].ctrl_vals[j] &= 0xff00;
				bfd_freq_table->pd[i].ctrl_vals[j] |= VID_V2;
			}
		}
	} else {
		vddcpu0 = vddcpu1 = NULL;
	}

	boost_supported = 0;
	for (i = 0; i < MAX_NUM_MODULES; i++) {
		if (bfd_freq_table->pd[i].boost_supported) {
			boost_supported = 1;
			break;
		}
	}
	if (boost_supported) {
		bergenfld_cpufreq_driver.boost_supported = true;
		bfd_cpufreq_attr[1] = &cpufreq_freq_attr_scaling_boost_freqs;
	}

	ret = cpufreq_register_driver(&bergenfld_cpufreq_driver);
	if (!ret)
		return ret;

err:
	for (i = 0; i < MAX_NUM_MODULES; i++)
		kfree(freq_table[i]);

	iounmap(dvfs_mod0);
	iounmap(dvfs_mod1);

	return ret;
}

device_initcall(bergenfld_cpufreq_init);


MODULE_AUTHOR("Subin Kollassery Gangadharan <subin.k.gangadharan@intel.com>");
MODULE_DESCRIPTION("Performance-States Driver");
MODULE_LICENSE("GPL");
