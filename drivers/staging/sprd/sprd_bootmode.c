/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

static bool calibration_mode;
static bool charging_mode;
static bool autotest_mode;

static int __init calibration_start(char *str)
{
	int calibration_device = 0;
	int mode = 0, freq = 0, device = 0;
	int ret;

	if (str) {
		pr_info("modem calibartion:%s\n", str);
		ret = sscanf(str, "%d,%d,%d", &mode, &freq, &device);
		if (ret != 3)
			pr_err("calibration params = %d\n", ret);
	}
	if (device & 0x80) {
		calibration_device = device & 0xf0;
		calibration_mode = true;
		pr_info("cali device = 0x%x\n", calibration_device);
	}

	return 0;
}

__setup("calibration=", calibration_start);

static __init int charger_mode_start(char *str)
{
	if (strcmp(str, "charger"))
		charging_mode = false;
	else
		charging_mode = true;

	return 0;
}

__setup("androidboot.mode=", charger_mode_start);

static int __init autotest_start(char *str)
{
	autotest_mode = true;

	return 0;
}

__setup("autotest=", autotest_start);

bool in_calibration(void)
{
	return calibration_mode;
}
EXPORT_SYMBOL(in_calibration);

bool in_charger(void)
{
	return charging_mode;
}
EXPORT_SYMBOL(in_charger);

bool in_autotest(void)
{
	return autotest_mode;
}
EXPORT_SYMBOL(in_autotest);
