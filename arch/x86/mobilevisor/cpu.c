/* Copyright (C) 2014 Intel Mobile Communications GmbH
 * *
 * * This software is licensed under the terms of the GNU General Public
 * * License version 2, as published by the Free Software Foundation, and
 * * may be copied, distributed, and modified under those terms.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * * GNU General Public License for more details.
 * */


#include <asm/mv/mv_gal.h>
#include <asm/mv/mv_hypercalls.h>
#include <asm/mv/cpu.h>

#include <linux/sched.h>
#include <linux/of.h>

#define MAX_MV_VCPUS	(8)

static struct cpumask _cpumask_shared = { CPU_BITS_NONE };
static struct cpumask *cpumask_shared = &_cpumask_shared;

static unsigned linux_irq_guest_control;
static unsigned apic_vcpu_map[CONFIG_NR_CPUS];
static uint32_t linux_vectors[MAX_MV_VCPUS];

bool mv_is_irq_bypass(void)
{
	/* magic number 0xdeadbeaf */
	if (linux_irq_guest_control == 0xdeadbeaf)
		return true;
	else
		return false;
}

bool mv_is_cpu_shared(unsigned cpu)
{
	return !mv_is_cpu_exclusive(cpu);
}

bool mv_is_cpu_exclusive(unsigned cpu)
{
	if (!mv_is_irq_bypass())
		return false;

	if (cpumask_test_cpu(cpu, cpumask_shared))
		return false;
	else
		return true;
}

unsigned int mv_get_shared_cpu(void)
{
	return cpumask_first(cpumask_shared);
}

bool mv_is_guest_vector(unsigned vector)
{
	if (!mv_is_irq_bypass())
		return true;

	return (linux_vectors[vector >> 5] & (1 << (vector % 32)));
}

unsigned mv_cpu_get_apicid(unsigned cpu)
{
	if (mv_is_irq_bypass())
		return apic_vcpu_map[cpu];
	else
		return cpu;
}

static int mv_cpu_mapping_init(void)
{
	unsigned vcpu, cpu;
	unsigned vcpu_map, shared_cpu_map, apic_map, num_of_cpu_map;

	cpumask_clear(cpumask_shared);
	mv_get_cpu_map(&vcpu_map, &shared_cpu_map, &apic_map, &num_of_cpu_map);

	for (vcpu = 0; vcpu < MAX_MV_VCPUS; vcpu++) {
		unsigned phys_cpu;
		bool shared;

		phys_cpu = (vcpu_map >> (vcpu * 4)) & 0xF;

		if (shared_cpu_map & BIT(phys_cpu))
			shared = true;
		else
			shared = false;

		apic_vcpu_map[vcpu] = (apic_map >> (phys_cpu * 4)) & 0xF;

		pr_debug("shared: %s vcpu %x phys %x\n",
				shared ? "yes" : "no",
				vcpu,
				phys_cpu);
		if (shared)
			cpumask_set_cpu(vcpu, cpumask_shared);

	}

	for_each_cpu(cpu, cpumask_shared)
		pr_info("CPU %d is shared\n", cpu);

	return 0;
}
EXPORT_SYMBOL(mv_cpu_mapping_init);

bool mv_cpu_get_settings(void)
{
	mv_get_os_irq_guest_control(&linux_irq_guest_control);
	pr_info("Guest irq_bypass : %s\n", (mv_is_irq_bypass()) ? "yes" : "no");

	if (mv_is_irq_bypass()) {
		mv_get_os_vectors(linux_vectors, MAX_MV_VCPUS);
		mv_cpu_mapping_init();
	}

	return true;
}
