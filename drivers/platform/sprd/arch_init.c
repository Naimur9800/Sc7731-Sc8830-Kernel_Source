/*
 * Copyright (C) 2014 Spreadtrum Communications Inc.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <soc/sprd/adi.h>
#include <soc/sprd/adc.h>
#include <soc/sprd/arch_lock.h>

static int __init arch_init(void)
{
	hwspinlocks_init();
	early_init_hwlocks();
	sci_adi_init();
	sci_adc_init();

	return 0;
}

postcore_initcall_sync(arch_init);
