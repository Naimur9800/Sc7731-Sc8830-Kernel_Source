/* ----------------------------------------------------------------------------
    Copyright (C) 2013 Intel Mobile Communications GmbH

        Sec Class: Intel Confidential (IC)

    All rights reserved.
  ----------------------------------------------------------------------------
    This document contains proprietary information belonging to IMC.
    Passing on and copying of this document, use
    and communication of its contents is not permitted without prior written
    authorisation.
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
#include <linux/sched.h>
#include <asm/mv/mv_hypercalls.h>
#include <asm/mv/mv_svc_hypercalls.h>
#include <asm/mv/mv_service_list.h>
#include <asm/mv/mv_gal.h>
#include <asm/mv/logging/mv_svc_log.h>
#else
#include <mv_gal.h>
#include <mv_service_list.h>
#include <mv_hypercalls.h>
#include <mv_svc_hypercalls.h>
#include <logging/mv_svc_log.h>
#endif

static bool initialized;
void mv_svc_log_init(void)
{
	uint32_t current_version, min_compatible_version;
	uint32_t shared_mem_offset, shared_mem_size;

	if (!mv_svc_get_service_info(VMM_LOG_SERVICE, &current_version,
		&min_compatible_version, &shared_mem_offset, &shared_mem_size)) {
		return;
	}

	if (min_compatible_version > VMMLOG_API_VER) {
		mv_gal_panic("attempt to invoke a non-available service: vmmlog\n");
		return;
	}

	initialized = true;
	mv_svc_vmm_logs_sync_timestamp(sched_clock());
}

bool mv_svc_vmm_logs_is_initialized(void)
{
	return initialized;
}

static void log_service_ensure_init(void)
{
	if (!initialized)
		mv_gal_panic("vmmlog service is not initialized\n");
}

/**
 @brief Mobilevisor logs print
 @param os msg needs to be printed in mobilevisor buffer
 @param os msg length
 @return 0 if success
**/
int32_t mv_svc_vmm_logs_print(char *msg, int msg_len)
{
	log_service_ensure_init();
	return mv_platform_service(OPCODE_LOG, VMM_PRINT_LOG,
		(uint32_t)((long)msg), (uint32_t)msg_len, 0, 0,	0, 0, 0, 0);
}

#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_vmm_logs_print);
#endif

/**
 @brief Mobilevisor log connect
 @param os log params need to pass to mobilevior log service
 @return 0 if success
**/
int32_t mv_svc_vmm_logs_connect(uint32_t parm)
{
	log_service_ensure_init();
	return mv_platform_service(OPCODE_LOG, VMM_CONNECT_LOG,
		parm, 0, 0, 0, 0, 0, 0, 0);
}

#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_vmm_logs_connect);
#endif

/**
 @brief Mobilevisor log disconnect
 @param os log params need to pass to mobilevior log service
 @return 0 if success
**/
int32_t mv_svc_vmm_logs_disconnect(uint32_t parm)
{
	log_service_ensure_init();
	return mv_platform_service(OPCODE_LOG, VMM_DISCONNECT_LOG,
		parm, 0, 0, 0, 0, 0, 0, 0);
}

#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_vmm_logs_disconnect);
#endif

/**
 @brief Mobilevisor log level
 @param1/2 are for uart level/kernel level/device controlS
    os log params need to pass to mobilevior log service
 @return 0 if success
**/
int32_t mv_svc_vmm_logs_level(uint32_t uart_level, uint32_t kernel_level)
{
	log_service_ensure_init();
	return mv_platform_service(OPCODE_LOG, VMM_LOG_LEVEL,
		uart_level, kernel_level, 0, 0, 0, 0, 0, 0);
}

#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_vmm_logs_level);
#endif

/**
 @brief sync Kernel and Mobilevisor log timestamp
 @param1 are 64 bits of ts_nsec(could be gotten by sched_clock())
 @return 0 if success
**/
int32_t mv_svc_vmm_logs_sync_timestamp(uint64_t ts_nsec)
{
	if (initialized)
		return mv_platform_service(OPCODE_LOG, VMM_LOG_SYNC_TIMESTAMP,
			(uint32_t)(ts_nsec >> 32), (uint32_t)ts_nsec,
			0, 0, 0, 0, 0, 0);
	else
		return -1;
}

#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_vmm_logs_sync_timestamp);
#endif
