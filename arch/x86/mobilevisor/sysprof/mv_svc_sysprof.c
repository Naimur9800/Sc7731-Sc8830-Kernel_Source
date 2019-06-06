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
#include <asm/mv/sysprof/mv_svc_sysprof.h>
#else
#include <mv_gal.h>
#include <mv_service_list.h>
#include <mv_hypercalls.h>
#include <mv_svc_hypercalls.h>
#include <sysprof/mv_svc_sysprof.h>
#endif

static int initialized;
void mv_svc_sysprof_init(void)
{
	uint32_t current_version, min_compatible_version;
	uint32_t shared_mem_offset, shared_mem_size;

	if (!mv_svc_get_service_info(VMM_SYSPROF_SERVICE, &current_version,
		&min_compatible_version, &shared_mem_offset, &shared_mem_size))
		return;

	if (min_compatible_version > SYSPROF_API_VER) {
		mv_gal_panic(
		"attempt to invoke a non-available service: sysprof\n");
		return;
	}

	initialized = 1;
}

static void sysprof_service_ensure_init(void)
{
	if (!initialized)
		mv_gal_panic("sysprof service is not initialized\n");
}

/* Legacy SYSPROF requirement */
void mv_svc_sysprof_trace_start(uint32_t *swt_paddr, uint32_t mask)
{
	sysprof_service_ensure_init();
	mv_platform_service(OPCODE_SYSPROF, SYSPROF_TRACE_START,
		(uint32_t)(uintptr_t)swt_paddr, mask, 0, 0, 0, 0, 0, 0);
}
/* Legacy SYSPROF requirement */
void mv_svc_sysprof_trace_stop(void)
{
	sysprof_service_ensure_init();
	mv_platform_service(OPCODE_SYSPROF, SYSPROF_TRACE_STOP,
		0, 0, 0, 0, 0, 0, 0, 0);
}
/* Legacy SYSPROF requirement */
void mv_svc_sysprof_get_vcore_map(struct sysprof_vcore_info *vcore_map)
{
	sysprof_service_ensure_init();
	mv_platform_service(OPCODE_SYSPROF, SYSPROF_VCORE_MAP_REQ,
		(uint32_t)(uintptr_t)vcore_map, 0, 0, 0, 0, 0, 0, 0);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sysprof_get_vcore_map);
#endif

/* Legacy SYSPROF requirement */
void mv_svc_sysprof_perf_count_enable(uint32_t cnt_config)
{
	sysprof_service_ensure_init();
	mv_platform_service(OPCODE_SYSPROF, SYSPROF_PERF_COUNT_ENABLE,
		cnt_config, 0, 0, 0, 0, 0, 0, 0);
}

void mv_svc_sysprof_target_initialized(uint8_t context, uint8_t initialized)
{
	sysprof_service_ensure_init();
	mv_platform_service(OPCODE_SYSPROF, SYSPROF_TARGET_INITIALIZED,
		(uint32_t)context, (uint32_t)initialized, 0, 0, 0, 0, 0, 0);
}

void mv_svc_sysprof_target_version(uint8_t context, uint8_t version)
{
	sysprof_service_ensure_init();
	mv_platform_service(OPCODE_SYSPROF, SYSPROF_TARGET_VERSION,
		(uint32_t)context, (uint32_t)version, 0, 0, 0, 0, 0, 0);
}

uint32_t mv_svc_sysprof_manager_version_target(uint8_t context)
{
	sysprof_service_ensure_init();
	return mv_platform_service(OPCODE_SYSPROF,
		SYSPROF_MANAGER_VERSION_TARGET,
		(uint32_t)context, 0, 0, 0, 0, 0, 0, 0);
}

uint32_t mv_svc_sysprof_manager_version_control(void)
{
	sysprof_service_ensure_init();
	return mv_platform_service(OPCODE_SYSPROF,
		SYSPROF_MANAGER_VERSION_CONTROL,
		0, 0, 0, 0, 0, 0, 0, 0);
}

uint32_t mv_svc_sysprof_manager_target_cores(uint8_t context)
{
	sysprof_service_ensure_init();
	return mv_platform_service(OPCODE_SYSPROF, SYSPROF_MANAGER_TARGET_CORES,
		(uint32_t)context, 0, 0, 0, 0, 0, 0, 0);
}

void mv_svc_sysprof_manager_target_trace_data(uint8_t context, void *trace_data)
{
	sysprof_service_ensure_init();
	mv_platform_service(OPCODE_SYSPROF, SYSPROF_MANAGER_TARGET_TRACE_DATA,
		(uint32_t)context, (uint32_t)(uintptr_t)trace_data,
		0, 0, 0, 0, 0, 0);
}

void mv_svc_sysprof_target_classmask(uint8_t context, uint32_t *class_mask)
{
	sysprof_service_ensure_init();
	mv_platform_service(OPCODE_SYSPROF, SYSPROF_TARGET_CLASS_MASK,
		(uint32_t)context, (uint32_t)(uintptr_t)class_mask,
		0, 0, 0, 0, 0, 0);
}

void mv_svc_sysprof_target_shared_data(uint8_t context, void *shared_data)
{
	sysprof_service_ensure_init();
	mv_platform_service(OPCODE_SYSPROF, SYSPROF_TARGET_SHARED_DATA,
		(uint32_t)context, (uint32_t)(uintptr_t)shared_data,
		0, 0, 0, 0, 0, 0);
}

void mv_svc_sysprof_target_confirm(uint8_t context, uint8_t command)
{
	sysprof_service_ensure_init();
	mv_platform_service(OPCODE_SYSPROF, SYSPROF_TARGET_CONFIRM,
		(uint32_t)context, (uint32_t)command, 0, 0, 0, 0, 0, 0);
}

int32_t mv_svc_sysprof_control_configure(uint16_t mid, void *exproConfigParams)
{
	sysprof_service_ensure_init();
	return mv_platform_service(OPCODE_SYSPROF, SYSPROF_CONTROL_CONFIGURE,
		(uint32_t)mid, (uint32_t)(uintptr_t)exproConfigParams,
		0, 0, 0, 0, 0, 0);
}

void mv_svc_sysprof_control_query(void *exproQueryParams)
{
	sysprof_service_ensure_init();
	mv_platform_service(OPCODE_SYSPROF, SYSPROF_CONTROL_QUERY,
		(uint32_t)(uintptr_t)exproQueryParams, 0, 0, 0, 0, 0, 0, 0);
}

