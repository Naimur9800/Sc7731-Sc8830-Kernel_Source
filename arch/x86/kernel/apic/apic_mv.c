/*
 * Copyright (C) 2015 Intel Mobile Communications GmbH
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

#include <linux/smp.h>
#include <linux/percpu.h>
#include <linux/cpumask.h>
#include <linux/syscore_ops.h>
#include <asm/apic.h>
#include <asm/irq_vectors.h>
#include <asm/ipi.h>

#include <asm/mv/mv_hypercalls.h>
#include <asm/mv/mv_svc_hypercalls.h>
#include <asm/mv/reg_access/mv_svc_reg_access.h>
#include <asm/mv/mobilevisor.h>
#include <asm/mv/cpu.h>

#define LAPIC_PADDR_BASE   0xFEE00000

/* mv_apic_map format: (apic_id << (4 * i)) */
static uint32_t mv_apic_map;

static struct {
	/* r/w apic fields */
	unsigned int apic_id;
	unsigned int apic_taskpri;
	unsigned int apic_ldr;
	unsigned int apic_dfr;
	unsigned int apic_spiv;
	unsigned int apic_lvtt;
	unsigned int apic_lvtpc;
	unsigned int apic_lvt0;
	unsigned int apic_lvt1;
	unsigned int apic_lvterr;
	unsigned int apic_tmict;
	unsigned int apic_tdcr;
	unsigned int apic_thmr;
} apic_state;

static u32 mv_apic_mem_read(u32 reg);
static void mv_apic_mem_write(u32 reg, u32 v);

static void mv_apic_check_vector(int vector)
{
	if ((vector != RESCHEDULE_VECTOR) &&
		(vector != REBOOT_VECTOR) &&
		(vector != NMI_VECTOR) &&
		(vector != CALL_FUNCTION_SINGLE_VECTOR) &&
		(vector != CALL_FUNCTION_VECTOR) &&
		(vector != LOCAL_TIMER_VECTOR) &&
		(vector != IRQ_MOVE_CLEANUP_VECTOR) &&
		(vector != IRQ_WORK_VECTOR)) {
		pr_err("%s: Unknown vector %x\n", __func__, vector);
		BUG();
	}
}

static void mv_apic_send_ipi_mask(const struct cpumask *mask, int vector)
{
	unsigned int this_cpu;

	mv_apic_check_vector(vector);
	vector = x86_mv_map_vector(vector);

	this_cpu = get_cpu();

	if (vector == NMI_VECTOR)
		mv_set_nmi_ipi_func(cpumask_bits(mask)[0]);

	if (mv_is_cpu_exclusive(this_cpu) && mv_is_irq_bypass())
		default_send_IPI_mask_sequence_phys(mask, vector);
	else
		mv_ipi_post(vector, cpumask_bits(mask)[0]);
	put_cpu();
}

static void mv_send_ipi_allbutself(int vector)
{
	cpumask_t mask;
	unsigned int this_cpu;

	if (num_online_cpus() <= 1)
		return;

	this_cpu = get_cpu();

	cpumask_clear(&mask);
	cpumask_copy(&mask, cpu_online_mask);
	cpumask_clear_cpu(this_cpu, &mask);

	mv_apic_send_ipi_mask(&mask, vector);

	put_cpu();
}

static void mv_send_IPI_all(int vector)
{
	mv_apic_send_ipi_mask(cpu_online_mask, vector);
}

static int mv_wakeup_secondary_cpu(int apicid, unsigned long start_ip)
{
	int cpu;
	/* FIXME: Not so good to highjack the parameter.. */
#ifdef CONFIG_X86_32
	unsigned long hack_start_ip = (unsigned long)__pa(startup_32_smp);
#else
	unsigned long hack_start_ip =
		(unsigned long)__pa(mv_secondary_startup_64);
#endif

	if (mv_is_irq_bypass()) {
		for_each_possible_cpu(cpu) {
			if (per_cpu(x86_cpu_to_apicid, cpu) == apicid) {
				pr_info("start cpu 0x%x for apicid 0x%x 0x%lx\n",
					cpu, apicid, hack_start_ip);
				mv_vcpu_start_64bit(cpu, hack_start_ip);
			}
		}
	} else
		mv_vcpu_start_64bit(apicid, hack_start_ip);

	return 0;
}

static int mv_cpu_present_to_apicid(int cpu)
{
	int ret = cpu;

	if (mv_is_irq_bypass())
		ret = default_cpu_present_to_apicid(cpu);

	return ret;
}

static int mv_check_phys_apicid_present(int apicid)
{
	int ret = true;

	if (mv_is_irq_bypass())
		ret = default_check_phys_apicid_present(apicid);

	return ret;
}

static int mv_apic_id_valid(int apicid)
{
	int mv_apic_id;

	if (mv_is_irq_bypass())
		return default_apic_id_valid(apicid);

	/* BSP always valid */
	if (!apicid)
		return true;

	mv_apic_id = (mv_apic_map >> (4 * apicid)) & 0xF;

	return mv_apic_id ? true : false;
}

static int mv_apic_suspend(void)
{
	unsigned int v;
	int maxlvt;

	/* 1, save apic state */
	maxlvt = lapic_get_maxlvt();

	apic_state.apic_id = apic_read(APIC_ID);
	apic_state.apic_taskpri = apic_read(APIC_TASKPRI);
	apic_state.apic_ldr = apic_read(APIC_LDR);
	apic_state.apic_dfr = apic_read(APIC_DFR);
	apic_state.apic_spiv = apic_read(APIC_SPIV);
	apic_state.apic_lvtt = apic_read(APIC_LVTT);
	if (maxlvt >= 4)
		apic_state.apic_lvtpc = apic_read(APIC_LVTPC);
	apic_state.apic_lvt0 = apic_read(APIC_LVT0);
	apic_state.apic_lvt1 = apic_read(APIC_LVT1);
	apic_state.apic_lvterr = apic_read(APIC_LVTERR);
	apic_state.apic_tmict = apic_read(APIC_TMICT);
	apic_state.apic_tdcr = apic_read(APIC_TDCR);
#ifdef CONFIG_X86_THERMAL_VECTOR
	if (maxlvt >= 5)
		apic_state.apic_thmr = apic_read(APIC_LVTTHMR);
#endif

	/* 2, mask lvts */
	v = apic_read(APIC_LVTT);
	apic_write(APIC_LVTT, v | APIC_LVT_MASKED);
	v = apic_read(APIC_LVT0);
	apic_write(APIC_LVT0, v | APIC_LVT_MASKED);
	v = apic_read(APIC_LVT1);
	apic_write(APIC_LVT1, v | APIC_LVT_MASKED);
	if (maxlvt >= 4) {
		v = apic_read(APIC_LVTPC);
		apic_write(APIC_LVTPC, v | APIC_LVT_MASKED);
	}

#ifdef CONFIG_X86_THERMAL_VECTOR
	if (maxlvt >= 5) {
		v = apic_read(APIC_LVTTHMR);
		apic_write(APIC_LVTTHMR, v | APIC_LVT_MASKED);
	}
#endif
#ifdef CONFIG_X86_MCE_INTEL
	if (maxlvt >= 6) {
		v = apic_read(APIC_LVTCMCI);
		if (!(v & APIC_LVT_MASKED))
			apic_write(APIC_LVTCMCI, v | APIC_LVT_MASKED);
	}
#endif
	/* 3, other ops... */

	return 0;
}

static void mv_apic_resume(void)
{
	int maxlvt;

	/* 1, restore apic state */
	maxlvt = lapic_get_maxlvt();
	apic_write(APIC_LVTERR, x86_mv_map_vector(ERROR_APIC_VECTOR)
			| APIC_LVT_MASKED);
	apic_write(APIC_ID, apic_state.apic_id);
	apic_write(APIC_DFR, apic_state.apic_dfr);
	apic_write(APIC_LDR, apic_state.apic_ldr);
	apic_write(APIC_TASKPRI, apic_state.apic_taskpri);
	apic_write(APIC_SPIV, apic_state.apic_spiv);
	apic_write(APIC_LVT0, apic_state.apic_lvt0);
	apic_write(APIC_LVT1, apic_state.apic_lvt1);
#if defined(CONFIG_X86_MCE_INTEL)
	if (maxlvt >= 5)
		apic_write(APIC_LVTTHMR, apic_state.apic_thmr);
#endif
	if (maxlvt >= 4)
		apic_write(APIC_LVTPC, apic_state.apic_lvtpc);
	apic_write(APIC_LVTT, apic_state.apic_lvtt);
	apic_write(APIC_TDCR, apic_state.apic_tdcr);
	apic_write(APIC_TMICT, apic_state.apic_tmict);
	apic_write(APIC_ESR, 0);
	apic_read(APIC_ESR);
	apic_write(APIC_LVTERR, apic_state.apic_lvterr);
	apic_write(APIC_ESR, 0);
	apic_read(APIC_ESR);

	/* 2, other ops... */
}

static struct syscore_ops mv_apic_syscore_ops = {
	.resume		= mv_apic_resume,
	.suspend	= mv_apic_suspend,
};

static int mv_apic_probe(void)
{
	uint32_t vcpu_map, shared_cpu_map, num_of_cpus;

	if (!is_x86_mobilevisor())
		return 0;

	register_syscore_ops(&mv_apic_syscore_ops);

	/* Get allocated apic map from VMM host */
	mv_get_cpu_map(&vcpu_map, &shared_cpu_map,
			&mv_apic_map, &num_of_cpus);

	return 1;
}

static unsigned mv_default_get_apic_id(unsigned long x)
{
	unsigned apic_id;

	if (mv_is_irq_bypass())
		apic_id = default_get_apic_id(x);
	else
		apic_id = mv_vcpu_id();

	return apic_id;
}

static unsigned long mv_set_apic_id(unsigned int id)
{
	return ((id & 0xFFu)<<24);
}

static u32 mv_apic_mem_read(u32 reg)
{
	unsigned addr = LAPIC_PADDR_BASE + reg;
	unsigned mask = 0xFFFFFFFF;
	unsigned val;
	unsigned ret;
	int this_cpu = get_cpu();

	if (mv_is_cpu_exclusive(this_cpu)) {
		val = native_apic_mem_read(reg);
		goto out;
	}

	ret = mv_svc_reg_read(addr, &val, mask);
	if (ret)
		panic("%s: Read %x APIC register failed\n",
				__func__, addr);

out:
	put_cpu();
	return val;
}

static void mv_apic_mem_write(u32 reg, u32 v)
{
	unsigned addr = LAPIC_PADDR_BASE + reg;
	unsigned mask = 0xFFFFFFFF;
	int ret;
	int this_cpu = get_cpu();

	if (mv_is_cpu_exclusive(this_cpu)) {
		native_apic_mem_write(reg, v);
		goto out;
	}

	ret = mv_svc_reg_write(addr, v, mask);
	if (ret)
		panic("%s: Write %x APIC register failed\n",
				__func__, addr);

out:
	put_cpu();
}

#ifdef CONFIG_X86_32
static int mv_early_logical_apicid(int cpu)
{
	return BIT(cpu);
}
#endif

static void mv_apic_eoi_write(u32 reg, u32 value)
{
	int this_cpu = get_cpu();

	if (mv_is_cpu_exclusive(this_cpu) && mv_is_irq_bypass())
		native_apic_mem_write(reg, value);
	else
		mv_virq_eoi(value);

	put_cpu();
}

static int mv_apic_id_registered(void)
{
	return physid_isset(read_apic_id(), phys_cpu_present_map);
}

static inline int mv_default_phys_pkg_id(int cpuid_apic, int index_msb)
{
	return cpuid_apic >> index_msb;
}

static void mv_init_lapic_ldr(void) { }

static void mv_apic_ipi_self(int vector)
{
	unsigned int vcpus;
	unsigned int this_cpu = get_cpu();

	mv_apic_check_vector(vector);
	vector = x86_mv_map_vector(vector);
	if (mv_is_cpu_exclusive(this_cpu)) {
		__default_send_IPI_shortcut(APIC_DEST_SELF, vector, 0);
	} else {
		vcpus = 1 << this_cpu;
		mv_ipi_post(vector, vcpus);
	}

	put_cpu();
}

static struct apic apic_mv = {

	.name				= MV_APIC_NAME,
	.probe				= mv_apic_probe,
	.acpi_madt_oem_check		= NULL,
	.apic_id_valid			= mv_apic_id_valid,
	.apic_id_registered		= mv_apic_id_registered,

	.irq_delivery_mode		= 0,
	.irq_dest_mode			= 1,

	.target_cpus			= default_target_cpus,
	.disable_esr			= 0,
	.dest_logical			= 0,
	.check_apicid_used		= NULL,

	.vector_allocation_domain	= flat_vector_allocation_domain,
	.init_apic_ldr			= mv_init_lapic_ldr,

	.ioapic_phys_id_map		= NULL,
	.setup_apic_routing		= NULL,
	.cpu_present_to_apicid		= mv_cpu_present_to_apicid,
	.apicid_to_cpu_present		= NULL,
	.check_phys_apicid_present	= mv_check_phys_apicid_present,
	.phys_pkg_id			= mv_default_phys_pkg_id,

	.get_apic_id			= mv_default_get_apic_id,
	.set_apic_id			= mv_set_apic_id,
	.apic_id_mask			= 0x0F << 24,

	.cpu_mask_to_apicid_and		= flat_cpu_mask_to_apicid_and,

	.send_IPI_mask			= mv_apic_send_ipi_mask,
	.send_IPI_mask_allbutself	= NULL,
	.send_IPI_allbutself		= mv_send_ipi_allbutself,
	.send_IPI_all			= mv_send_IPI_all,
	.send_IPI_self			= mv_apic_ipi_self,

	.wakeup_secondary_cpu		= mv_wakeup_secondary_cpu,

	.inquire_remote_apic		= NULL,

	.read				= mv_apic_mem_read,
	.write				= mv_apic_mem_write,
	.eoi_write			= mv_apic_eoi_write,

	.icr_read			= native_apic_icr_read,
	.icr_write			= native_apic_icr_write,
	.wait_icr_idle			= native_apic_wait_icr_idle,
	.safe_wait_icr_idle		= native_safe_apic_wait_icr_idle,

#ifdef CONFIG_X86_32
	.x86_32_early_logical_apicid	= mv_early_logical_apicid,
#endif
};

apic_driver(apic_mv);
