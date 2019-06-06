/*
 * Spreadtrum x86 iSoC platform specific setup code
 *
 * (C) Copyright 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/smp.h>
#include <linux/of_platform.h>
#include <asm/mv/mobilevisor.h>
#include <asm/i8259.h>
#include <asm/hpet.h>
#include <asm/tsc.h>

#include "sleep.h"

static struct of_device_id sprd_isoc_bus_ids[] __initdata = {
	{ .compatible = "simple-bus", },
	{},
};

static void dummy_get_wallclock(struct timespec *ts)
{
	ts->tv_sec = 0;
	ts->tv_nsec = 0;
}

static int dummy_set_wallclock(const struct timespec *ts)
{
	return -ENODEV;
}

static void isoc_wallclock_init(void)
{
	x86_platform.get_wallclock = dummy_get_wallclock;
	x86_platform.set_wallclock = dummy_set_wallclock;
}

#define MAX_HPET_PERIOD 100000000

static unsigned long quick_hpet_calibrate(void)
{
	u64 tsc1, tsc2, dtsc;
	u64 ref1, ref2;
	u64 delay = 10;
	unsigned long hpet_period;

	tsc1 = tsc_read_refs(&ref1, 1);
	ref2 = ref1;

	hpet_period = hpet_readl(HPET_PERIOD);
	if (hpet_period == 0 || hpet_period > MAX_HPET_PERIOD) {
		pr_err("HPET calibration failed\n");
		return 0;
	}

	delay *= 1000000000000LL;
	delay /= hpet_period;

	while (ref2 - ref1 < delay)
		tsc2 = tsc_read_refs(&ref2, 1);

	dtsc = (tsc2 - tsc1) * 1000000LL;

	pr_info("Fast TSC calibration using HPET\n");
	return calc_hpet_ref(dtsc, ref1, ref2);
}

static unsigned long hpet_calibrate_tsc(void)
{
	unsigned long flags, fast_calibrate;

	local_irq_save(flags);
	fast_calibrate = quick_hpet_calibrate();
	local_irq_restore(flags);

	return fast_calibrate;
}

/* Overwrite x86 soc specific init function here */
void __init x86_soc_early_setup(void)
{
	pr_info("%s: isoc init!\n", __func__);
	x86_init.timers.wallclock_init = isoc_wallclock_init;

#ifdef CONFIG_X86_DISABLE_LEGACY_PIC
	legacy_pic = &null_legacy_pic;
#endif

	/* ISOC tsc use hpet calibrate */
	x86_platform.calibrate_tsc = hpet_calibrate_tsc;
}

static void x86_bhl_play_dead(void)
{
	unsigned long long msr_bits;

	/* C-State Configuration Control */
	rdmsrl(MSR_NHM_SNB_PKG_CST_CFG_CTL, msr_bits);
	msr_bits |= BIA_EN_MANUAL_PERFCTR
		 |  BIA_DYN_L2_REDUCTION
		 |  BIA_LOWER_CST_CONFIG;
	wrmsrl(MSR_NHM_SNB_PKG_CST_CFG_CTL, msr_bits);

	native_play_dead();
}

static void __init x86_sprd_isoc_early_init(void)
{
	pr_info("%s\n", __func__);
#ifdef CONFIG_SMP
	/*
	 * Only override play_dead for native case
	 * Mobilevisor is handling play_dead via hypercall
	 */
	if (!is_x86_mobilevisor())
		smp_ops.play_dead = x86_bhl_play_dead;
#endif
}

static int __init x86_sprd_isoc_bus_probe(void)
{
	if (!of_have_populated_dt())
		return 0;

	x86_sprd_isoc_early_init();

	return of_platform_populate(NULL, sprd_isoc_bus_ids, NULL, NULL);
}
arch_initcall(x86_sprd_isoc_bus_probe);
