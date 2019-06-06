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

#ifndef _MV_SVC_COREDUMP_H
#define _MV_SVC_COREDUMP_H

#define COREDUMP_API_VER		1
#define COREDUMP_API_MIN_VER		1

/**
  @typedef cd_op_code
  @brief   enumeration containing the operation of core dump service
**/
enum cd_op_code {
	CD_SET_CONFIG = 0,
	CD_ADD_REGION,
	CD_CONFIG_SHAREMEM,
	CD_ALLOW_CONFIG,
	CD_CONFIG_OCT2,
	CD_FILL_SHAREMEM
};

/**
 @brief  MobileVisor    core dump service
 @param  cd_opcode core dump operation code
 @param  cd_data        physical address of share data
**/
void mv_svc_cd_service(uint32_t cd_opcode, void *cd_data);

#endif /* _MV_SVC_COREDUMP_H */
