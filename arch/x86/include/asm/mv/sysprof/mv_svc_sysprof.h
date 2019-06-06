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
#ifndef _MV_SVC_SYSPROF_H
#define _MV_SVC_SYSPROF_H

#define SYSPROF_API_VER		1
#define SYSPROF_API_MIN_VER	1

/**
  @typedef sysprof_op_code
  @brief   enumeration containing the operation of System Profiling service
**/
enum sysprof_op_code {
	/* Legacy SYSPROF opcodes */
	SYSPROF_TRACE_START = 0,
	SYSPROF_TRACE_STOP = 1,
	SYSPROF_TASK_LIST_REQ = 2,
	SYSPROF_ENTITY_INFO = 3,
	SYSPROF_ENTITY_SENT = 4,
	SYSPROF_IRQ_LIST_REQ = 5,
	SYSPROF_VCORE_MAP_REQ = 6,
	SYSPROF_PERF_COUNT_ENABLE = 7,
	/* New SYSPROF opcodes */
	SYSPROF_TARGET_INITIALIZED = 8,
	SYSPROF_TARGET_VERSION = 9,
	SYSPROF_MANAGER_VERSION_TARGET = 10,
	SYSPROF_MANAGER_VERSION_CONTROL = 11,
	SYSPROF_MANAGER_TARGET_CORES = 12,
	SYSPROF_MANAGER_TARGET_TRACE_DATA = 13,
	SYSPROF_TARGET_CLASS_MASK = 14,
	SYSPROF_TARGET_SHARED_DATA = 15,
	SYSPROF_TARGET_CONFIRM = 16,
	SYSPROF_CONTROL_CONFIGURE = 17,
	SYSPROF_CONTROL_QUERY = 18
};

/**
  @typedef sysprof_vmctxt_id - Legacy SYSPROF requirement
  @brief   enumeration containing the VM Context ID used in System Profiling
**/
enum sysprof_vmctxt_id {
	SYSPROF_VMCTXT_MEX = 0x00,
	SYSPROF_VMCTXT_SECVM = 0x01,
	SYSPROF_VMCTXT_LINUX = 0x10,
	SYSPROF_VMCTXT_WINDOWS = 0x20,
	SYSPROF_VMCTXT_VMM = 0x30,
	SYSPROF_VMCTXT_INVALID = 0xff
};

/*
 * Maximum number of vCores supported by System Profiling
 * Legacy SYSPROF requirement
 */
#define SYSPROF_MAX_VCORES	16

/**
  @brief   Information of vCore needed by System Profiling
	   Legacy SYSPROF requirement
**/
struct sysprof_vcore_info {
	uint8_t pcore;          /* physical core number */
	uint8_t context;        /* any of sysprof_vmctxt_id */
};

/**
 @brief  MobileVisor System Profiling service to start trace
	 Legacy SYSPROF requirement
 @param  swt_paddr	physical address for software trace channels
			(one per physical core)
 @param  mask		mask for the classes of events
**/
void mv_svc_sysprof_trace_start(uint32_t *swt_paddr, uint32_t mask);

/**
 @brief  MobileVisor System Profiling service to stop trace
	 Legacy SYSPROF requirement
**/
void mv_svc_sysprof_trace_stop(void);

/**
 @brief  MobileVisor System Profiling service to get vCore mapping
	 Legacy SYSPROF requirement
 @param  vcore_map	physical address for storing the vCore mapping
			(support up to SYSPROF_MAX_VCORES vCores)
**/
void mv_svc_sysprof_get_vcore_map(struct sysprof_vcore_info *vcore_map);

/**
 @brief  MobileVisor System Profiling service to enable performance counters
	 Legacy SYSPROF requirement
 @param  cnt_config	configuration of performance counters
**/
void mv_svc_sysprof_perf_count_enable(uint32_t cnt_config);

/**
 @brief  MobileVisor System Profiling service to report the Profiling Target
	 init state to the Profiling Manager
 @param  context	context value of the target reporting the init state
 @param  initialized	initialized state
**/
void mv_svc_sysprof_target_initialized(uint8_t context, uint8_t initialized);
/**
 @brief  MobileVisor System Profiling service to report the Profiling Target
	 version to the Profiling Manager
 @param  context	context value of the target reporting the version
 @param  version	Profiling Target version
**/
void mv_svc_sysprof_target_version(uint8_t context, uint8_t version);
/**
 @brief  MobileVisor System Profiling service for the Profiling Target to
	 request the version of the Profiling Manager
 @param  context	context value of the target requesting the manager
			version
 @return version	Profiling Manager version
**/
uint32_t mv_svc_sysprof_manager_version_target(uint8_t context);
/**
 @brief  MobileVisor System Profiling service for the Profiling Control to
	 request the version of the Profiling Manager
 @return version	Profiling Manager version
**/
uint32_t mv_svc_sysprof_manager_version_control(void);
/**
 @brief  MobileVisor System Profiling service for the Profiling Target to
	 request the number of Target cores as seen by the Profiling Manager
 @param  context	context value of the target requesting the nr of cores
 @return nr_cores	number of Target cores as seen by the Profiling Manager
**/
uint32_t mv_svc_sysprof_manager_target_cores(uint8_t context);
/**
 @brief  MobileVisor System Profiling service to report the Profiling Target
		 trace_data pointer to the Profiling Manager
 @param  context	    context value of the target reporting the trace data
 @param  class_mask     trace data pointer from the Profiling Target
**/
void mv_svc_sysprof_manager_target_trace_data(uint8_t context,
		void *trace_data);
/**
 @brief  MobileVisor System Profiling service to report the Profiling Target
	 class_mask pointer to the Profiling Manager
 @param  context	context value of the target reporting the class mask
 @param  class_mask	class mask pointer of the Profiling Target
**/
void mv_svc_sysprof_target_classmask(uint8_t context, uint32_t *class_mask);
/**
 @brief  MobileVisor System Profiling service to report the Profiling Target
	 shared_data pointer to the Profiling Manager
 @param  context	context value of the target reporting the shared_data
 @param  shared_data	shared data pointer of the Profiling Target
**/
void mv_svc_sysprof_target_shared_data(uint8_t context, void *shared_data);
/**
 @brief  MobileVisor System Profiling service to confirm the Profiling Target
	 execution of a particular command sent previously by the Profiling
	 Manager
 @param  context	context value of the target confirming the command
			execution
 @param  command	what command is confirmed by the Profiling Target
**/
void mv_svc_sysprof_target_confirm(uint8_t context, uint8_t command);
/**
 @brief  MobileVisor System Profiling service to provide a control configuration
	 from the Profiling Control to the Profiling Manager
 @param  mid		 mid trace value for the provided configuration
			(currently valid only for T&D profiling)
 @param  exproConfigParams	profiling configuration parameters
**/
int32_t mv_svc_sysprof_control_configure(uint16_t mid, void *exproConfigParams);
/**
 @brief  MobileVisor System Profiling service to provide a control query command
	 from the Profiling Control to the Profiling Manager
 @param  exproQueryParams	query parameters
**/
void mv_svc_sysprof_control_query(void *exproQueryParams);

#endif /* _MV_SVC_SYSPROF_H */
