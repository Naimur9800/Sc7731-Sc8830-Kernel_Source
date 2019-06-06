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
#ifndef _MV_SVC_VTIMER_H
#define _MV_SVC_VTIMER_H

#define VTIMER_API_VER			1
#define VTIMER_API_MIN_VER		1

/**
 @brief  Vtimer start service
 @param  num_of_vcycles
 @return 0 if success, -1 if access disallowed
**/
int32_t mv_svc_vtimer_start(uint32_t num_of_vcycles);

/**
 @brief  Vtimer stop service
 @return 0 if success, -1 if access disallowed
**/
int32_t mv_svc_vtimer_stop(void);

/**
 @brief  Vtimer get frequency service
 @return virtual frequency value
**/
int32_t mv_svc_vtimer_get_freq(void);

/**
  @typedef timestamp_counter_info_op_code
  @brief   enumeration containing the operation of timestamp
  counter info service
**/
enum timestamp_counter_info_op_code {
	TIMESTAMP_COUNTER_FREQ = 0,
	TIMESTAMP_COUNTER_SIZE
};

/**
 @brief  MobileVisor timestamp counter frequency
 @return timestamp counter frequency in Hz
**/
uint32_t mv_svc_timestamp_counter_frequency(void);

/**
 @brief  MobileVisor timestamp counter size
 @return timestamp counter size in number of bits
**/
uint32_t mv_svc_timestamp_counter_size(void);

#endif /* _MV_SVC_VTIMER_H */
