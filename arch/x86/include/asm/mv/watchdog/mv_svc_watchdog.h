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
#ifndef _MV_SVC_WATCHDOG_H
#define _MV_SVC_WATCHDOG_H

#define WATCHDOG_API_VER		1
#define WATCHDOG_API_MIN_VER		1

/**
  @typedef watchdog_op_code
  @brief   enumeration containing the operation of watchdog service
**/
enum watchdog_op_code {
	WATCHDOG_ENABLE = 0,
	WATCHDOG_DISABLE,
	WATCHDOG_PET,
	VMM_HW_WATCHDOG_SWITCH
};

/**
 @brief  Enable watchdog with the timeout specified
 @param  timeout period (in seconds) that the watchdog must be serviced
 @return 0 if success.
**/
uint32_t mv_svc_watchdog_enable(uint32_t timeout);

/**
 @brief  Pet the watchdog
 @return 0 if success.
**/
uint32_t mv_svc_watchdog_pet(void);

/**
 @brief  Disable the watchdog
 @return 0 if success.
**/
uint32_t mv_svc_watchdog_disable(void);

/**
 @brief enable/disable vmm scu watchdog
 @return 0 if success.
 **/
uint32_t mv_svc_scu_watchdog_switch(int on);

#endif /* _MV_SVC_WATCHDOG_H */
