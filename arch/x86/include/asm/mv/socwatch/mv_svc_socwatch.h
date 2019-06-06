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
#ifndef _MV_SVC_SOCWATCH_H
#define _MV_SVC_SOCWATCH_H

#define SOCWATCH_API_VER		1
#define SOCWATCH_API_MIN_VER		1

#define SOCWATCH_LOGGING_DEBUG

#define SOCWATCH_BUFFER_COUNT_PER_CORE 2

/**
  @typedef socwatch_op_code
  @brief   enumeration containing the operation of socwatch service
**/
enum socwatch_op_code {
	SOCWATCH_SET_CONFIG = 0,
	SOCWATCH_RUN_CONTROL
};

/**
  @typedef socwatch_buffer_status
  @brief   defines the buffer status
**/
enum socwatch_buffer_status {
	SOCWATCH_BUFFER_INVALID = 0,
	SOCWATCH_BUFFER_VALID,
	SOCWATCH_BUFFER_CONSUMED,
};

/**
  @brief socwatch buffer information
**/
struct socwatch_buffer_info {
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
	uint64_t buffer_start[SOCWATCH_BUFFER_COUNT_PER_CORE];
	/** 64bit physical address of buffer to be processed
	 * use the current processor index
	 * to determine which array index to use */
	uint64_t buffer_delivered;
	/** the size of the data contained within the buffer
	 * when it is delivered from the VMM to the Linux kernel driver */
	uint32_t buffer_data_size;
	/**
	 0: buffer is invalid
	 1: buffer is valid and not yet consumed
	 2: buffer has been consumed
     */
	uint32_t buffer_status;
#if defined(SOCWATCH_LOGGING_DEBUG)
	uint32_t samples_logged;
	uint32_t samples_dropped;
	uint32_t buffers_generated;
	uint64_t first_tsc;
	uint64_t last_tsc;
#endif
} __attribute((packed));
/*
 * We are setting a 2 byte boundary for PWCollector_msg because
 * that is how it is implemented in SoCWatch for all the other systems
 */
#pragma pack(push)	/* Store current alignment */
#pragma pack(2)		/* Set new alignment -- 2 byte boundaries */

/**
 * The main PWCollector_msg structure.
 * This struct is used by SoCWatch.
 * Changes to this struct without corresponging changes
 * in the SoCWatch driver/binary will break SoCWatch.
 */
struct PWCollector_msg {
	/** STM based (26MHz) count. TSC of message. */
	uint64_t tsc;
	/** length of payload message in bytes (not including this header)
	 * represented by p_data. */
	uint16_t data_len;
	 /** physical core number (0 or 1) */
	uint16_t cpuidx;
	/** type of payload encoded by 'p_data': one of 'sofia_msg_t' enum */
	uint8_t data_type;
	/** The compiler would have inserted it anyway! */
	uint8_t padding;
	/** For SW1 file, this is the payload: one of *_msg_t corresponding to
	 * data_type (inline memory). For SoCWatch internal data, this field is
	 * a pointer to the non-contiguous payload memory (not inline). */
	uint64_t p_data;
} __attribute((packed));

#define PW_MSG_HEADER_SIZE (sizeof(struct PWCollector_msg) - sizeof(u64))

#pragma pack(pop)	/* Restore previous alignment */

/**
 @brief  MobileVisor socwatch configuration
 @param  events socwatch events to be enabled
 @param  buffer_info phy address to buffer information
 @return 0 if success
**/
uint32_t mv_svc_socwatch_config(uint32_t events,
			struct socwatch_buffer_info *buffer_info);

/**
 @brief  MobileVisor socwatch run control
 @param  run_control 0 to stop, 1 to start
 @return for start, 0 if success. For stop, returns number of dropped packets
**/
uint32_t mv_svc_socwatch_run_control(uint32_t run_control);
#endif /* _MV_SVC_SOCWATCH_H */
