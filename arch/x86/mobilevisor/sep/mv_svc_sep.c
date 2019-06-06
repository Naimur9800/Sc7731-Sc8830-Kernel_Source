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
#include <asm/mv/mv_gal.h>
#include <asm/mv/sep/mv_svc_sep.h>
#else
#include <mv_gal.h>
#include <mv_service_list.h>
#include <mv_hypercalls.h>
#include <mv_svc_hypercalls.h>
#include <sep/mv_svc_sep.h>
#endif

static int initialized;
void mv_svc_sep_init(void)
{
	uint32_t current_version, min_compatible_version;
	uint32_t shared_mem_offset, shared_mem_size;

	if (!mv_svc_get_service_info(VMM_SEP_SERVICE, &current_version,
		&min_compatible_version, &shared_mem_offset, &shared_mem_size))
		return;

	if (min_compatible_version > SEP_API_VER) {
		mv_gal_panic("attempt to invoke a non-available service: sep\n");
		return;
	}

	initialized = 1;
}

static void sep_service_ensure_init(void)
{
	if (!initialized)
		mv_gal_panic("sep service is not initialized\n");
}

uint32_t mv_svc_sep_config(struct sep_buffer_info *p_buffer_info)
{
	sep_service_ensure_init();
	mv_platform_service(OPCODE_SEP, SEP_CONFIG,
		(uint32_t)(uintptr_t)p_buffer_info, 0, 0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_config);
#endif

uint32_t mv_svc_sep_guest_context(struct sep_guest_context *guest_context)
{
	sep_service_ensure_init();
	mv_platform_service(OPCODE_SEP, SEP_SET_GUEST_CONTEXT,
			(uint32_t)(uintptr_t)guest_context, 0, 0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_guest_context);
#endif

uint32_t mv_svc_sep_run_control(uint32_t run_control, uint32_t cpu)
{
	sep_service_ensure_init();
	mv_platform_service(OPCODE_SEP, SEP_RUN_CONTROL, run_control,
		cpu, 0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_run_control);
#endif

uint32_t mv_svc_sep_read_counters(struct sep_counter *buffer_pointer)
{
	sep_service_ensure_init();
	mv_platform_service(OPCODE_SEP, SEP_READ_COUNTER_LIST,
		(uint32_t)(uintptr_t)buffer_pointer, 0, 0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_read_counters);
#endif

uint32_t mv_svc_sep_write_counters(struct sep_counter *buffer_pointer)
{
	sep_service_ensure_init();
	mv_platform_service(OPCODE_SEP, SEP_WRITE_COUNTER_LIST,
		(uint32_t)(uintptr_t)buffer_pointer, 0, 0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_write_counters);
#endif

uint32_t mv_svc_sep_pmi_msr_list(struct sep_msr_control *entry_list,
	struct sep_msr_control *exit_list)
{
	sep_service_ensure_init();
	mv_platform_service(OPCODE_SEP, SEP_CONTROL_PMI_MSR_LIST,
		(uint32_t)(uintptr_t)entry_list, (uint32_t)(uintptr_t)exit_list,
		0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_pmi_msr_list);
#endif

uint32_t mv_svc_sep_vmswitch_msr_list(struct sep_msr_control *vmentry_list,
	struct sep_msr_control *vmexit_list)
{
	sep_service_ensure_init();
	mv_platform_service(OPCODE_SEP, SEP_CONTROL_VMSW_MSR_LIST,
		(uint32_t)(uintptr_t)vmentry_list, (uint32_t)(uintptr_t)vmexit_list,
		0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_vmswitch_msr_list);
#endif

uint32_t mv_svc_sep_version_number(void)
{
	uint32_t ver_num;
	sep_service_ensure_init();
	if (mv_platform_service(OPCODE_SEP, SEP_VERSION_NUMBER,
		0, 0, 0, 0, 0, 0, 0, &ver_num) == 0)
		return ver_num;
	else
		return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_version_number);
#endif
