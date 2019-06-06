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
#ifndef _MV_SVC_REG_ACCESS_H
#define _MV_SVC_REG_ACCESS_H

#define REG_ACCESS_API_VER		1
#define REG_ACCESS_API_MIN_VER		1

/**
 @brief  MobileVisor platform 32bit register access service
 @param  address physical address map
 @param  p_reg_val read value (-1 if access disallowed)
 @param  mask bit to modify
 @return 0 if success, -1 if access disallowed
**/
int32_t mv_svc_reg_read(uint32_t address, uint32_t *p_reg_val, uint32_t mask);

/**
 @brief  MobileVisor platform 32bit register access service
 @param  address physical address map
 @param  reg_val value to be written
 @param  mask bit to modify
 @return 0 if success, -1 if access disallowed
**/
int32_t mv_svc_reg_write(uint32_t address, uint32_t reg_val, uint32_t mask);

/**
 @brief  MobileVisor platform 32bit register access service
 @param  address physical address map
 @param  reg_val value to be written
 @param  mask bit to modify
 @return 0 if success, -1 if access disallowed
**/
int32_t mv_svc_reg_write_only(
	uint32_t address,
	uint32_t reg_val,
	uint32_t mask);
#endif /* _MV_SVC_REG_ACCESS_H */
