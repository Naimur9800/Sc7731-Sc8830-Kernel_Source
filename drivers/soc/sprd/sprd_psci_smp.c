/*
 * UNISOC ARM CPU SMP/hotplug psci ops.
 *
 * Copyright (C) 2018 unisoc Ltd.
 * Author: Jingchao Ye <jingchao.ye@unisoc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/smp.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/psci.h>
#include <linux/sprd_arm_pmu.h>

#include <uapi/linux/psci.h>

#include <asm/psci.h>
#include <asm/smp_plat.h>
#include <asm/cacheflush.h>

/*
 * psci_smp assumes that the following is true about PSCI:
 *
 * cpu_suspend   Suspend the execution on a CPU
 * @state        we don't currently describe affinity levels, so just pass 0.
 * @entry_point  the first instruction to be executed on return
 * returns 0  success, < 0 on failure
 *
 * cpu_off       Power down a CPU
 * @state        we don't currently describe affinity levels, so just pass 0.
 * no return on successful call
 *
 * cpu_on        Power up a CPU
 * @cpuid        cpuid of target CPU, as from MPIDR
 * @entry_point  the first instruction to be executed on return
 * returns 0  success, < 0 on failure
 *
 * migrate       Migrate the context to a different CPU
 * @cpuid        cpuid of target CPU, as from MPIDR
 * returns 0  success, < 0 on failure
 *
 */

extern void sci_secondary_startup(void);
extern volatile int pen_release;

static DEFINE_SPINLOCK(boot_lock);

#define SPRD_UP_FLAG0	(0x63507530)
#define SPRD_UP_FLAG1	(0x63507531)
#define SPRD_UP_FLAG2	(0x63507532)
#define SPRD_UP_FLAG3	(0x63507533)

static unsigned int sprd_boot_magnum = 0xbadf1a90;

static unsigned int g_sprd_up_flag[4] = {
	SPRD_UP_FLAG0,
	SPRD_UP_FLAG1,
	SPRD_UP_FLAG2,
	SPRD_UP_FLAG3
};

static void write_pen_release(int val)
{
	pen_release = val;
	/* memory barrier */
	smp_wmb();
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}

static int psci_boot_ops(int cpu_id, u32 paddr)
{
	if (cpu_id < 1 || cpu_id > (num_possible_cpus() - 1))
		return -EINVAL;

	if (psci_ops.cpu_on) {
		int ret;

		ret = psci_ops.cpu_on(cpu_logical_map(cpu_id), paddr);
		sprd_pmu_enable_irq(cpu_id);
		return ret;
	}
	return -ENODEV;
}

static void psci_secondary_init(unsigned int cpu)
{
	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	write_pen_release(g_sprd_up_flag[cpu]);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

static int psci_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;
	int ret;
	unsigned int val = sprd_boot_magnum;

	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * The secondary processor is waiting to be released from
	 * the holding pen - release it, then wait for it to flag
	 * that it has been released by resetting pen_release.
	 *
	 */
	val |= (cpu_logical_map(cpu) & 0x0000000f);
	write_pen_release((int)val);

	ret = psci_boot_ops(cpu, virt_to_phys(sci_secondary_startup));
	if (ret < 0)
		pr_warn("SMP: boot_secondary(%u) error\n", cpu);

	dsb_sev();
	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		/* memory barrier */
		smp_rmb();
		if (pen_release == g_sprd_up_flag[cpu])
			break;

		udelay(10);
		dsb_sev();
	}

	spin_unlock(&boot_lock);

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	if (pen_release != g_sprd_up_flag[cpu])
		pr_err("[%s]pen release error\n", __func__);

	return pen_release != g_sprd_up_flag[cpu] ? -ENOSYS : 0;
}

#ifdef CONFIG_HOTPLUG_CPU
int psci_cpu_disable(unsigned int cpu)
{
	/* Fail early if we don't have CPU_OFF support */
	if (!psci_ops.cpu_off)
		return -EOPNOTSUPP;

	/* Trusted OS will deny CPU_OFF */
	if (psci_tos_resident_on(cpu))
		return -EPERM;

	return 0;
}

void psci_cpu_die(unsigned int cpu)
{
	u32 state = PSCI_POWER_STATE_TYPE_POWER_DOWN <<
		    PSCI_0_2_POWER_STATE_TYPE_SHIFT;

	if (psci_ops.cpu_off) {
		sprd_pmu_disable_irq(cpu);
		psci_ops.cpu_off(state);
	}

	/* We should never return */
	panic("psci: cpu %d failed to shutdown\n", cpu);
}

int psci_cpu_kill(unsigned int cpu)
{
	int err, i;

	if (!psci_ops.affinity_info)
		return 1;
	/*
	 * cpu_kill could race with cpu_die and we can
	 * potentially end up declaring this cpu undead
	 * while it is dying. So, try again a few times.
	 */

	for (i = 0; i < 20; i++) {
		err = psci_ops.affinity_info(cpu_logical_map(cpu), 0);
		if (err == PSCI_0_2_AFFINITY_LEVEL_OFF) {
			pr_info("CPU%d killed.\n", cpu);
			return 1;
		}

		udelay(100);
		pr_info("Retrying again to check for CPU kill\n");
	}

	pr_warn("CPU%d may not have shut down cleanly (AFFINITY_INFO reports %d)\n",
			cpu, err);
	/* Make platform_cpu_kill() fail. */
	return 0;
}

#endif

bool __init psci_smp_available(void)
{
	/* is cpu_on available at least? */
	return (psci_ops.cpu_on != NULL);
}

struct smp_operations __initdata psci_smp_ops = {
	.smp_secondary_init = psci_secondary_init,
	.smp_boot_secondary	= psci_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_disable		= psci_cpu_disable,
	.cpu_die		= psci_cpu_die,
	.cpu_kill		= psci_cpu_kill,
#endif
};
