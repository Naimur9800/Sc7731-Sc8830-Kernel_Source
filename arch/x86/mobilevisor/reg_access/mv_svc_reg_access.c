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
#include <asm/mv/reg_access/mv_svc_reg_access.h>
#else
#include <mv_gal.h>
#include <mv_service_list.h>
#include <mv_hypercalls.h>
#include <mv_svc_hypercalls.h>
#include <reg_access/mv_svc_reg_access.h>
#endif

void mv_svc_reg_access_init(void)
{
	uint32_t current_version, min_compatible_version;
	uint32_t shared_mem_offset, shared_mem_size;

	if (!mv_svc_get_service_info(VMM_REG_ACCESS_SERVICE, &current_version,
		&min_compatible_version, &shared_mem_offset, &shared_mem_size))
		return;

	if (min_compatible_version > REG_ACCESS_API_VER) {
		mv_gal_panic("attempt to invoke a non-available service: reg_access\n");
		return;
	}
}

int32_t mv_svc_reg_read(uint32_t address, uint32_t *p_reg_val, uint32_t mask)
{
	return mv_platform_service(OPCODE_REG_RD, address, mask,
		0, 0, 0, 0, 0, 0, p_reg_val);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_reg_read);
#endif

int32_t mv_svc_reg_write(uint32_t address, uint32_t reg_val, uint32_t mask)
{
	return mv_platform_service(OPCODE_REG_WR, address, reg_val, mask,
		0, 0, 0, 0, 0, 0);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_reg_write);
#endif

int32_t mv_svc_reg_write_only(uint32_t address, uint32_t reg_val, uint32_t mask)
{
	return mv_platform_service(OPCODE_REG_WR_ONLY, address, reg_val, mask,
		0, 0, 0, 0, 0, 0);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_reg_write_only);
#endif
