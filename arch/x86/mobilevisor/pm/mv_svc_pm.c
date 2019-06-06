/* ----------------------------------------------------------------------------
   Copyright (C) 2014 Intel Mobile Communications GmbH

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

  ---------------------------------------------------------------------------*/

/*
 * NOTES:
 * 1) This source file is included in guests including Linux and purposely
 * kept in line with Linux kernel coding guidelines for ease of portability and
 * maintainability. See linux/Documentation/CodingStyle for coding guidelines.
 * Please check all further changes are compliant with Linux coding standard
 * and clear all commands reported by Linux checkpatch.pl
 * Use command: linux/scripts/checkpatch.pl -f <filename>
 * Clear ALL warnings and errors reported.
 *
 * 2) Use only C99 fixed width types for definition as this header needs to be
 * both 32bit/64bit portable.
 * Avoid the use of pointers/enum in structure as that make the structure
 * variable size based on 32/64bit toolchain used.
*/
/*
 * WARNING:
 * If any files in pal/platforms/<platform>/service are modified,
 * ensure the regenerated lib_mobilevisor_platform_service.a into
 * pal/release/$(PROJECTFOLDER)/lib_guest/ folder for commit
 * Use command: "make release"
*/

/*
 * WARNING:
  * Always use portable C99 fixed width types for 64bit/32bit compatibility.
*/
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <asm/mv/mv_hypercalls.h>
#include <asm/mv/mv_svc_hypercalls.h>
#include <asm/mv/mv_service_list.h>
#include <asm/mv/pm/mv_svc_pm.h>
#include <asm/mv/sysprofile.h>
#include <linux/percpu.h>
#else
#include <mv_gal.h>
#include <mv_service_list.h>
#include <mv_svc_hypercalls.h>
#include <mv_hypercalls.h>
#include <pm/mv_svc_pm.h>
#endif

static uint32_t offset;
static uint32_t initialized;

void mv_svc_pm_init(void)
{
	uint32_t current_version, min_compatible_version, shared_mem_size;

	if (!mv_svc_get_service_info(VMM_PM_SERVICE, &current_version,
		&min_compatible_version, &offset, &shared_mem_size))
		return;

	if (min_compatible_version > PM_API_VER) {
		panic("attempt to invoke a non-available service: pm\n");
		return;
	}

	if (current_version > PM_API_VER) {
		pr_warn("mismatched version (guest v%d, host v%d) for pm service, consider upgrade\n",
			current_version, PM_API_VER);
		return;
	}

	initialized = 1;
}

static void pm_service_ensure_init(void)
{
	if (!initialized)
		panic("pm service is not initialized\n");
}

struct pm_control_shared_data *mv_svc_pm_get_shared_data(uint32_t cpu)
{
	pm_service_ensure_init();
	return (struct pm_control_shared_data *)((unsigned char *)
		(mv_shared_data[cpu]->pal_shared_mem_data) + offset);
}

uint32_t mv_svc_pm_control(uint32_t pm_opcode,
	uint32_t arg1,
	uint32_t arg2,
	uint32_t arg3)
{
	pm_service_ensure_init();
	return mv_platform_service(OPCODE_PM, pm_opcode, arg1, arg2, 0,
		(uint32_t *)(uintptr_t)arg3,  0, 0, 0, 0);
}

void mv_svc_vm_enter_idle(struct pm_control_shared_data
		*pm_control_shared_data_p, uint32_t state)
{
        sysprof_set_idle_state(state);
        pm_control_shared_data_p->target_power_state = state;
        native_halt();
        pm_control_shared_data_p->target_power_state = PM_S0;
        sysprof_idle_exit(0);
}

void mv_svc_vm_enter_suspend(struct pm_control_shared_data
		*pm_control_shared_data_p, unsigned int state)
{
        sysprof_sleep_enter(state);
        pm_control_shared_data_p->target_power_state = state;
        native_halt();
        pm_control_shared_data_p->target_power_state = PM_S0;
        sysprof_sleep_exit(0);
}

int32_t mv_svc_pm_idle_get_num_cpu_idle_states(unsigned int cpuid)
{
        return mv_svc_pm_control(PM_IDLE_GET_NUM_CPU_IDLE_STATES, cpuid, 0, 0);
}

int32_t mv_svc_pm_idle_get_cpu_idle_table(struct mv_svc_cpu_idle_table *
                                          mv_svc_cpu_idle_table,
                                          unsigned int size,
                                          unsigned int cpuid)
{
        return mv_svc_pm_control(PM_IDLE_GET_CPU_IDLE_TABLE,
                                 (uint32_t) __pa(mv_svc_cpu_idle_table),
                                 (cpuid << 16) | size, 0);
}

int32_t mv_svc_pm_set_freq_table(unsigned int modid,
				u16 *freq_table,
				unsigned int size,
				unsigned int boost_index)
{
	return mv_svc_pm_control(PM_CPU_SET_FREQ_STEPS,
				(modid << 16) | (boost_index << 8) | size,
				(uint32_t) __pa(freq_table),
				0);
}


int32_t mv_svc_pm_get_default_sleep_state(void)
{
        return mv_svc_pm_control(PM_IDLE_GET_DEFAULT_SLEEP_STATE, 0, 0, 0);
}

void mv_svc_modem_2g_sleep_time(struct pm_control_shared_data
		*pm_control_shared_data_p, uint32_t duration)
{
	pm_service_ensure_init();
	pm_control_shared_data_p->vm_pow_state_param[2] = duration;
}

void mv_svc_modem_3g_sleep_time(struct pm_control_shared_data
		*pm_control_shared_data_p, uint32_t duration)
{
	pm_service_ensure_init();
	pm_control_shared_data_p->vm_pow_state_param[3] = duration;
}

void mv_svc_modem_next_timeout(struct pm_control_shared_data
		*pm_control_shared_data_p, uint32_t duration)
{
	pm_service_ensure_init();
	pm_control_shared_data_p->vm_pow_state_param[4] = duration;
}
