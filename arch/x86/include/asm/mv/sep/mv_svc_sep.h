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
#ifndef _MV_SVC_SEP_H
#define _MV_SVC_SEP_H

#define SEP_API_VER     2
#define SEP_API_MIN_VER	0

/**
  @typedef sep_op_code
  @brief   enumeration containing the operation of sep service
**/
enum sep_op_code {
	SEP_CONFIG = 0,
	SEP_SET_GUEST_CONTEXT,
	SEP_RUN_CONTROL,
	SEP_READ_COUNTER_LIST,
	SEP_WRITE_COUNTER_LIST,
	SEP_CONTROL_PMI_MSR_LIST,
	SEP_CONTROL_VMSW_MSR_LIST,
	SEP_VERSION_NUMBER = 100
};


enum sep_data_type {
	SEP_CORE_SAMPLE = 0,
	SEP_CORE_COUNT,
	SEP_LBR_DATA,
	SEP_UNCORE_COUNT
};

struct sep_core_sample {
	/** context where PMI is triggered */
	uint32_t os_id;
	/** instruction pointer */
	uint64_t rip;
	/** the task id */
	uint32_t task_id;
	/** the task name */
	char task[16];
	/** physical core ID */
	uint32_t cpu_id;
	/** the process id */
	uint32_t process_id;
	/** perf global status msr value (for overflow status) */
	uint64_t overflow_status;
	/** rflags */
	uint32_t rflags;
	/** code segment */
	uint32_t cs;
} __attribute__((packed));

#define SEP_CORE_SAMPLE_SIZE (sizeof(struct sep_core_sample))


struct sep_lbr_data {
	/** LBR TOS */
	uint64_t lbr_tos;
	/** LBR FROM IP */
	uint64_t lbr_from_ip[8];
	/** LBR TO IP */
	uint64_t lbr_to_ip[8];
} __attribute__((packed));

#define SEP_LBR_DATA_SIZE (sizeof(struct sep_lbr_data))


/**
 * The main sep_packet_header structure.
 * This struct is used by SEP.
 * Changes to this struct without corresponging changes
 * in the SEP driver/binary will break SEP.
 */
struct sep_packet_header {
	/** length of payload message in bytes (not including this header)
	 * represented by p_data. */
	uint16_t data_len;
	/** physical core number (0 or 1) */
	uint16_t cpu_id;
	/** type of data collected. Bit OR of 'sep_data_type' enum */
	uint32_t data_type;
	/** STM based (26MHz) count. TSC of message. */
	uint64_t timestamp;
} __attribute__((packed));

#define SEP_PKT_HEADER_SIZE (sizeof(struct sep_packet_header))


struct sep_counter {
	/** counter to read/write; last entry will have value of -1 */
	int32_t msr_id;
	/** value to write or location to write into */
	uint64_t value;
} __attribute__((packed));

struct sep_msr_control {
	/** msr to read/write; last entry will have value of -1 */
	int32_t msr_id;
	/** value to write or location to write into */
	uint64_t value;
	/** parameter; usage depends on operation */
	uint32_t param;
} __attribute__((packed));


/* Maximum number of MSRs in a list, including last entry with msr_id of -1 */
#define SEP_MAX_MSR_LIST        16

/* Define this for debug info on total and dropped samples of SEP packet */
#define SEP_LOGGING_DEBUG       1

/**
  @typedef sep_buffer_status
  @brief   defines the buffer status
**/
enum sep_buffer_status {
	SEP_BUFFER_INVALID = 0,
	SEP_BUFFER_VALID,
	SEP_BUFFER_CONSUMED,
};

/**
  @brief sep buffer information
**/
struct sep_buffer_info {
	uint32_t num_buffers_allocated;
	/** buffer size in Kbytes (1024)
	 * 16 means a 16384 byte buffer */
	uint32_t buffer_length;
	/** struct user must check for NULL values
	 * when num_buffers_allocated is less than 4,
	 * some array values will be NULL
	 * buffer_start values must be 64bit physical addresses
	 * pointing to contiguous memory;
	 * use the low 32 bits if 32 bit addresses are  used */
	uint64_t buffer_start[16];
	/** 64bit physical address of buffer to be processed
	 * use the current processor index
	 * to determine which array index to use */
	uint64_t buffer_delivered[8];
	/** the size of the data contained within the buffer
	 * when it is delivered from the VMM to the Linux kernel driver */
	uint32_t buffer_data_size[8];
	/**
0: buffer is invalid
1: buffer is valid and not yet consumed
2: buffer has been consumed
*/
	uint32_t buffer_status[8];

#if defined(SEP_LOGGING_DEBUG)
	uint32_t total_samples_logged[8];
	uint32_t total_samples_dropped[8];
	uint32_t modem_samples_logged[8];
	uint32_t modem_samples_dropped[8];
#endif
} __attribute__((packed));

/**
  @brief sep ring buffer information
**/
struct sep_ring_buffer_info {
	/** buffer size in number of sep_packets */
	uint32_t buffer_size;
	/** addresses of buffer in a ring buffer implementation
	 * buffer_start values must be 64bit physical addresses
	 * pointing to contiguous memory;
	 * use the low 32 bits if 32 bit addresses are  used */
	uint64_t buffer_start;
	/** index to next entry in buffer for writing */
	uint32_t buffer_wridx;
	/** index to next entry in buffer for reading */
	uint32_t buffer_rdidx;
	/** flag indicating if buffer is to be actively filled */
	uint32_t buffer_active;
#if defined(SEP_LOGGING_DEBUG)
	uint32_t samples_logged;
	uint32_t samples_dropped;
#endif
} __attribute__((packed));


struct sep_guest_context {
	uint32_t thread_id;
	uint32_t process_id;
} __attribute__((packed));

/**
 @brief  MobileVisor sep configuration
 @param  p_buffer_info physical address of buffer information
 @return 0 if success
**/
uint32_t mv_svc_sep_config(struct sep_buffer_info *p_buffer_info);

/**
 @brief  MobileVisor sep config guest context
 @param  task_pointer linear address of the guest's current task pointer
 @param  task_name_offset offset of the current task name from the task pointer
 @return 0 if success.
**/
uint32_t mv_svc_sep_guest_context(struct sep_guest_context *guest_context);

/**
 @brief  MobileVisor sep run control
 @param  run_control 0 to stop, 1 to start
 @return 0 if success.
**/
uint32_t mv_svc_sep_run_control(uint32_t run_control, uint32_t cpu);

/**
 @brief  MobileVisor sep read counter control
 @param  buffer_pointer linear address of the guest's current list of
			counters to read
 @return 0 if success.
**/
uint32_t mv_svc_sep_read_counters(struct sep_counter *buffer_pointer);

/**
 @brief  MobileVisor sep write counter control
 @param  buffer_pointer linear address of the guest's current list of
			counters and values to write
 @return 0 if success.
**/
uint32_t mv_svc_sep_write_counters(struct sep_counter *buffer_pointer);

/**
 @brief  MobileVisor sep PMI handler entry and exit MSR list
 @param  entry_list list of MSR to be written on entry of PMI handler
 @param  exit_list  list of MSR to be written on exit of PMI handler
 @return 0 if success.
**/
uint32_t mv_svc_sep_pmi_msr_list(struct sep_msr_control *entry_list,
					struct sep_msr_control *exit_list);

/**
 @brief  MobileVisor sep VM entry and exit MSR list
 @param  vmentry_list list of MSR to be written on VM entry
 @param  vmexit_list  list of MSR to be written on VM exit
 @return 0 if success.
**/
uint32_t mv_svc_sep_vmswitch_msr_list(struct sep_msr_control *vmentry_list,
					struct sep_msr_control *vmexit_list);

/**
 @brief  MobileVisor sep version number
 @return sep version number
**/
uint32_t mv_svc_sep_version_number(void);

#endif /* _MV_SVC_SEP_H */
