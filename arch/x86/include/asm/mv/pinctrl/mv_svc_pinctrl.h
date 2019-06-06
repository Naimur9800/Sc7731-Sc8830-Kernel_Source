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
#ifndef _MV_SVC_PINCTRL_H
#define _MV_SVC_PINCTRL_H

#define PINCTRL_API_VER		1
#define PINCTRL_API_MIN_VER	1

#define MAX_NUMBER_OF_PADS_IN_PINCTRL_GROUP 64
/**
  @brief pinctrl group information
**/
struct pinctrl_group_info {
	uint32_t pinctrl_pad; /*!< PCL pad index */
	uint32_t pinctrl_pad_value; /*!< pad value */
};

#define PINCTRL_ABB_IDX(x)      (x|(0x8000))

/**
  @typedef pinctrl_service_op_code
  @brief   enumeration containing the operation of pinctrl service
**/
enum pinctrl_service_op_code {
	PINCTRL_OPEN = 0,
	PINCTRL_GET,
	PINCTRL_SET,
	PINCTRL_CONFIG_FUNC,
	PINCTRL_CLOSE,
	PINCTRL_SET_CONTROL,
	PINCTRL_GET_CONTROL,
	PINCTRL_GET_PIN,
	PINCTRL_GET_GROUP,
	PINCTRL_SET_GROUP
};

/**
  @typedef pinctrl_oper
  @brief   enumeration containing port configuration
**/
enum pinctrl_oper {
	PINCTRL_OPER_ACTIVE		= 0,
	PINCTRL_OPER_SLEEP		= 1,
	PINCTRL_OPER_DEAVTIVE	= 2
};

/**
  @typedef pinctrl_funcs
  @brief	 enumeration containing function ID
  **/
enum pinctrl_funcs {
	PINCTRL_EMMC	= 0x80000000,
	PINCTRL_SDMMC,
	PINCTRL_NAND,
	PINCTRL_KEYPAD,
	PINCTRL_I2C1,
	PINCTRL_I2C2,
	PINCTRL_USIF1,
	PINCTRL_USIF2,
	PINCTRL_CC1,
	PINCTRL_CC2,
	PINCTRL_LAST	= 0x8FFFFFFF
};

struct pinctrl_group_info *mv_svc_pinctrl_get_shared_data(uint32_t cpu);
/**
 @brief  MobileVisor platform pin control service
 <b> Note, Use PINCTRL_ABB_IDX(agr1) for AGOLD PCL ,<b>
 @param  pinctrl_opcode  operation service
 @param  agr1  physical index number of PCL
	(GET/SET) functional ID (FUNC_CONFIG)
 @param  arg2  register value to be written (SET) operation mode (FUNC_CONFIG)
 @param  arg3  return register value read (GET)
 @return return 0 if success, -1 otherwise
**/
uint32_t mv_svc_pinctrl_service(
			uint32_t pinctrl_opcode,
			uint32_t arg1, uint32_t arg2, uint32_t *arg3);
#endif /* _MV_SVC_PINCTRL */
