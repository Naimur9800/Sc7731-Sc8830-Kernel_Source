/* Copyright (c) 2010-2015 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>

#include <asm/mach/arch.h>

static const char * const sc9830_dt_match[] __initconst = {
	"sprd,sp9830a-ga1",
	NULL
};

static const char * const sc9833_dt_match[] __initconst = {
	"sprd,sp9833-fpga",
	"sprd,sp9833-vp",
	"sprd,sp9833-zebu",
	"sprd,sp9833ka-1h10-native",
	NULL
};

static const char * const sc9850_dt_match[] __initconst = {
	"sprd,sp9850ka-1h10-native",
	"sprd,sp9850ka-2c10-native",
	"sprd,sp9850ka-2c20-native",
	"sprd,sp9850ka-2c30-native",
	NULL
};

static const char * const sc7731e_dt_match[] __initconst = {
	"sprd,sp7731e-fpga",
	NULL
};

DT_MACHINE_START(SC9830_DT, "sc9830")
	.dt_compat = sc9830_dt_match,
MACHINE_END

DT_MACHINE_START(SC9833_DT, "sc9833")
	.dt_compat = sc9833_dt_match,
MACHINE_END

DT_MACHINE_START(SC9850_DT, "sc9850k")
	.dt_compat = sc9850_dt_match,
MACHINE_END

DT_MACHINE_START(SC7731e_DT, "sc7731e")
	.dt_compat = sc7731e_dt_match,
MACHINE_END
