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
#include <linux/preempt.h>
#include <linux/percpu.h>
#include <linux/smp.h>
#include <linux/string.h>
#include <asm/mv/mv_hypercalls.h>
#include <asm/mv/mv_svc_hypercalls.h>
#include <asm/mv/mv_service_list.h>
#include <asm/mv/pinctrl/mv_svc_pinctrl.h>
#else
#include <mv_gal.h>
#include <mv_service_list.h>
#include <mv_hypercalls.h>
#include <mv_svc_hypercalls.h>
#include <pinctrl/mv_svc_pinctrl.h>
#include <bastypes.h>
#endif

static uint32_t offset;
static uint32_t initialized;

void mv_svc_pinctrl_init(void)
{
	uint32_t current_version, min_compatible_version, shared_mem_size;

	if (!mv_svc_get_service_info(VMM_PINCTRL_SERVICE, &current_version,
		&min_compatible_version, &offset, &shared_mem_size))
		return;

	if (min_compatible_version > PINCTRL_API_VER) {
		panic("attempt to invoke a non-available service: pinctrl\n");
		return;
	}

	if (current_version > PINCTRL_API_VER) {
		pr_warn("mismatched version (guest v%d, host v%d) for pinctrl service, consider upgrade\n",
			current_version, PINCTRL_API_VER);
		return;
	}

	initialized = 1;
}

struct pinctrl_group_info *mv_svc_pinctrl_get_shared_data(uint32_t cpu)
{
	if (!initialized) {
		panic("pinctrl service is not initialized\n");
		return NULL;
	}

	return (struct pinctrl_group_info *)((unsigned char *)
		(mv_shared_data[cpu]->pal_shared_mem_data) + offset);
}

uint32_t mv_svc_pinctrl_service(
	uint32_t opcode,
	uint32_t arg1,
	uint32_t arg2,
	uint32_t *arg3)
{
	uint32_t result;

	if (!initialized) {
		panic("pinctrl service is not initialized\n");
		return -1;
	}

	switch (opcode) {
	case PINCTRL_OPEN:
	case PINCTRL_SET:
	case PINCTRL_CONFIG_FUNC:
	case PINCTRL_CLOSE:
	case PINCTRL_SET_CONTROL:
		return mv_platform_service(OPCODE_PINCTRL, opcode,
			arg1, arg2, 0, 0, 0, 0, 0, 0);
	break;

	case PINCTRL_GET:
	case PINCTRL_GET_CONTROL:
	case PINCTRL_GET_PIN:
		return mv_platform_service(OPCODE_PINCTRL, opcode,
			arg1, 0, 0, 0, 0, 0, 0, arg3);
	break;

	case PINCTRL_SET_GROUP:
		if (arg2 == 0)
			return 0xFFFFFFFF;
		/*check is required to avoid memory corruption */
		if (arg2 > MAX_NUMBER_OF_PADS_IN_PINCTRL_GROUP)
			arg2 = (uint32_t)
				MAX_NUMBER_OF_PADS_IN_PINCTRL_GROUP;
		preempt_disable();
		memcpy(mv_svc_pinctrl_get_shared_data(
			smp_processor_id()),
			(void *)(uintptr_t)arg1,
			(sizeof(struct pinctrl_group_info) * arg2));
		result = mv_platform_service(OPCODE_PINCTRL, opcode,
			0, arg2, 0, 0, 0, 0, 0, 0);
		preempt_enable();
		return result;
	break;

	case PINCTRL_GET_GROUP:
		if (arg2 == 0)
			return 0xFFFFFFFF;
		/*check is required to avoid memory corruption */
		if (arg2 > MAX_NUMBER_OF_PADS_IN_PINCTRL_GROUP)
			arg2 = (uint32_t)
				MAX_NUMBER_OF_PADS_IN_PINCTRL_GROUP;
		preempt_disable();
		memcpy(mv_svc_pinctrl_get_shared_data(
			smp_processor_id()),
			(void *)(uintptr_t)arg1,
			(sizeof(struct pinctrl_group_info) * arg2));
		result = mv_platform_service(OPCODE_PINCTRL, opcode, 0,
			arg2, 0, 0, 0, 0, 0, 0);
		memcpy((void *)(uintptr_t)arg1, mv_svc_pinctrl_get_shared_data(
			smp_processor_id()),
			(sizeof(struct pinctrl_group_info) * arg2));
		preempt_enable();
	break;

	default:
	break;
	}

	return -1;
}

#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_pinctrl_service);
#endif
