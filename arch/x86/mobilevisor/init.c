/*
 * Copyright (C) 2015 Intel Corporation
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
#include <linux/irqchip.h>
#include <linux/smp.h>
#include <linux/of_platform.h>

#include <asm/apic.h>
#include <asm/i8259.h>
#include <asm/irq.h>
#include <asm/time.h>

#include <asm/mv/mv_hypercalls.h>
#include <asm/mv/mv_gal.h>
#include <asm/mv/mv_vpic.h>
#include <asm/mv/irq_vectors.h>
#include <asm/mv/mobilevisor.h>
#include <asm/mv/vtimer/mv_svc_vtimer.h>

static int __init x86_mv_noop(void)
{
	return 0;
}

static void __init x86_mv_init_IRQ(void)
{
	native_init_IRQ();

	/*
	 * the cpu bitmap does not matter here as it's local interrupt
	 */
	mv_virq_request(MV_LOCAL_TIMER_VECTOR, 1);
	mv_virq_unmask(MV_LOCAL_TIMER_VECTOR);
#ifdef CONFIG_SMP
	mv_virq_request(MV_RESCHEDULE_VECTOR, 1);
	mv_virq_unmask(MV_RESCHEDULE_VECTOR);
	mv_virq_request(MV_CALL_FUNCTION_VECTOR, 1);
	mv_virq_unmask(MV_CALL_FUNCTION_VECTOR);
	mv_virq_request(MV_CALL_FUNCTION_SINGLE_VECTOR, 1);
	mv_virq_unmask(MV_CALL_FUNCTION_SINGLE_VECTOR);
	mv_virq_request(MV_REBOOT_VECTOR, 1);
	mv_virq_unmask(MV_REBOOT_VECTOR);
#endif
}

static __init void x86_mv_default_banner(void)
{
	pr_info("Booting Mobilevisor kernel\n");
}

#ifdef CONFIG_HOTPLUG_CPU
static void mv_play_dead(void)
{
	play_dead_common();
	mv_vcpu_stop(smp_processor_id());
	/* should never come back here */
	BUG();
}
#else
static void mv_play_dead(void)
{
	BUG();
}
#endif

static void __init x86_mv_time_init(void)
{
	pr_info("Intel x86 mv using lapic only\n");
	hpet_time_init();

	x86_init.timers.setup_percpu_clockev = setup_boot_APIC_clock;
	x86_cpuinit.setup_percpu_clockev = setup_secondary_APIC_clock;

	mv_init();
}

static unsigned long x86_mv_calibrate_tsc(void)
{
	return mv_svc_timestamp_counter_frequency() / 1000;
}

/* VMM specific x86_init function overrides and early setup calls. */
void __init x86_mv_early_init(void)
{
	pr_info("x86_mv_early_init\n");

	/* x86_init */
	x86_init.resources.probe_roms = x86_init_noop;
	x86_init.resources.reserve_resources = x86_init_noop;

	x86_init.mpparse.find_smp_config = x86_init_noop;
	x86_init.mpparse.get_smp_config = x86_init_uint_noop;

	x86_init.irqs.pre_vector_init = x86_init_noop;
	x86_init.irqs.intr_init = x86_mv_init_IRQ;

	x86_init.oem.banner = x86_mv_default_banner;

	x86_init.timers.timer_init = x86_mv_time_init,
	x86_init.timers.setup_percpu_clockev = x86_init_noop;

	x86_init.pci.init = x86_mv_noop,
	x86_init.pci.init_irq = x86_init_noop;
	x86_init.pci.fixup_irqs = x86_init_noop;

	/* x86_cpuinit */
	x86_cpuinit.setup_percpu_clockev = x86_init_noop;

	/* x86_platform */
	x86_platform.i8042_detect = x86_mv_noop;

	x86_platform.calibrate_tsc = x86_mv_calibrate_tsc;

#ifdef CONFIG_SMP
	x86_cpuinit.early_percpu_clock_init = (void *)mv_init_secondary;

	smp_ops.play_dead = mv_play_dead;
#endif

	/* Others */
	legacy_pic = &null_legacy_pic;
}

int __init mv_init_machine(void)
{
	if (!is_x86_mobilevisor())
		return 0;

	pr_info("mv_init_machine\n");

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);

	return 0;
}
arch_initcall(mv_init_machine);
