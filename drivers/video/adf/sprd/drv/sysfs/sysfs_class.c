/*
 * Copyright (C) 2014 Spreadtrum Communications Inc.
 *
 * Author: Haibing.Yang <haibing.yang@spreadtrum.com>
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

#define pr_fmt(__fmt) "[sprd-adf][%20s] "__fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>

#include "sysfs_display.h"

struct class *display_class;

static int __init display_class_init(void)
{
	pr_info("display class register\n");

	display_class = class_create(THIS_MODULE, "display");
	if (IS_ERR(display_class)) {
		pr_err("Unable to create display class\n");
		return PTR_ERR(display_class);
	}

	return 0;
}

postcore_initcall(display_class_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leon.he@spreadtrum.com");
MODULE_DESCRIPTION("Provide display class for hardware driver");
