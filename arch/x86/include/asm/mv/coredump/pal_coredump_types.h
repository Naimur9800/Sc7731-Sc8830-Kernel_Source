/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _PAL_COREDUMP_TYPES_H
#define _PAL_COREDUMP_TYPES_H

#define CD_MEMORY_RANGE_MAX 30

enum cd_dev {
	CD_DEV_DISABLED  = 0,
	CD_DEV_USIF1     = 1,
	CD_DEV_USIF2     = 2,
	CD_DEV_USIF3     = 3,
	CD_DEV_USBHS     = 4,
	CD_DEV_SDCARD    = 5,
	CD_DEV_MIPIPTI   = 6,  /* Supports PTI1 and PTI2 */
	CD_DEV_SHAREMEM  = 7   /* Support modem coredump via share memory */
};

struct cd_config {
	uint32_t device;
	uint32_t device_baud;
	uint32_t debug_device;
	uint32_t debug_device_baud;
	uint32_t usb_config;
	unsigned char failsafe_imei[8];
};

struct cd_ram {
	uint32_t logical_start;
	uint32_t physical_start;
	uint32_t length;
	uint32_t osid;
};

struct cd_sharemem {
	uint32_t status;       /* status ==1, modem panic */
	uint32_t number_of_ranges;
	struct cd_ram memory_range[CD_MEMORY_RANGE_MAX];
	struct sys_trap trap_ptr;
	struct sys_vm_dump vm_dump;
};

#endif /* _PAL_COREDUMP_TYPES_H*/
