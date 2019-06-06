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

#include <linux/module.h>
#include <linux/of.h>
#include <asm/div64.h>
#include <asm/x86_init.h>
#include <asm/hypervisor.h>

#include <asm/mv/irq_vectors.h>
#include <asm/mv/mobilevisor.h>
#include <asm/mv/cpu.h>
#include <asm/mv/mv_gal.h>

static int x86_mobilevisor_mode;

int is_x86_mobilevisor(void)
{
	return x86_mobilevisor_mode;
}

int x86_mv_map_vector(int vector)
{
	int mv_vector;

	if (!is_x86_mobilevisor())
		return vector;

	switch (vector) {
	case IRQ_WORK_VECTOR:
		mv_vector = MV_IRQ_WORK_VECTOR;
		break;
	case RESCHEDULE_VECTOR:
		mv_vector = MV_RESCHEDULE_VECTOR;
		break;
	case CALL_FUNCTION_VECTOR:
		mv_vector = MV_CALL_FUNCTION_VECTOR;
		break;
	case CALL_FUNCTION_SINGLE_VECTOR:
		mv_vector = MV_CALL_FUNCTION_SINGLE_VECTOR;
		break;
	case REBOOT_VECTOR:
		mv_vector = MV_REBOOT_VECTOR;
		break;
	case LOCAL_TIMER_VECTOR:
		mv_vector = MV_LOCAL_TIMER_VECTOR;
		break;
	case NMI_VECTOR: /* 1:1 map */
		mv_vector = NMI_VECTOR;
		break;
	case ERROR_APIC_VECTOR: /* 1:1 map */
		mv_vector = ERROR_APIC_VECTOR;
		break;
	case IRQ_MOVE_CLEANUP_VECTOR: /* 1:1 map */
		mv_vector = IRQ_MOVE_CLEANUP_VECTOR;
		break;
	default:
		mv_vector = vector; /* default to 1:1 */
		pr_err("mv: vector 0x%x is not supoorted\n",
			vector);
	}

	return mv_vector;
}

static void __init mv_platform_setup(void)
{
	x86_mv_early_init();

	pic_mode = 0; /* pic mode must be 0 when vm is present */
	disable_apic = 1;
}

static int __init parse_mobilevisor(char *arg)
{
	struct apic **drv;

	/* set default apic to mv */
	for (drv = __apicdrivers; drv < __apicdrivers_end; drv++) {
		if (!strcmp((*drv)->name, MV_APIC_NAME)) {
			apic = *drv;
			pr_info("Set default APIC to Mobilevisor\n");
			x86_mobilevisor_mode = 1;
			pr_info("Mobilevisor enabled\n");
			if (mv_map_vcpu_shmem() < 0)
				panic("VMM, unable to map vcpu shared mem\n");
			mv_cpu_get_settings();
		}
	}

	return 0;
}
early_param("mobilevisor", parse_mobilevisor);

static uint32_t __init mv_detect(void)
{
	return x86_mobilevisor_mode;
}

static void mv_set_cpu_features(struct cpuinfo_x86 *c)
{
}

/* Checks if hypervisor supports x2apic without VT-D interrupt remapping. */
static bool __init mv_legacy_x2apic_available(void)
{
	return true;
}

const __refconst struct hypervisor_x86 x86_hyper_mobilevisor = {
	.name			= "Mobilevisor",
	.detect			= mv_detect,
	.set_cpu_features	= mv_set_cpu_features,
	.init_platform		= mv_platform_setup,
	.x2apic_available	= mv_legacy_x2apic_available,
};
EXPORT_SYMBOL(x86_hyper_mobilevisor);
