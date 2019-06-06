
/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#ifndef _DISP_LIB_H_
#define _DISP_LIB_H_

#include <linux/list.h>

#define LOW_RES_SIMU_EN_PATH "/mnt/vendor/enhance/low_res_simu_en"

struct ops_entry {
	const char *ver;
	void *ops;
};

struct ops_list {
	struct list_head head;
	struct ops_entry *entry;
};

int str_to_u32_array(const char *p, unsigned int base, u32 array[]);
int str_to_u8_array(const char *p, unsigned int base, u8 array[]);

void *disp_ops_attach(const char *str, struct list_head *head);
int disp_ops_register(struct ops_entry *entry, struct list_head *head);

struct device *dev_get_prev(struct device *dev);
struct device *dev_get_next(struct device *dev);

int file_write(const char *filename, int flags, umode_t mode,
		char *val, size_t count);
int file_read(const char *filename, int flags, umode_t mode,
		char *val, size_t count);

#endif
