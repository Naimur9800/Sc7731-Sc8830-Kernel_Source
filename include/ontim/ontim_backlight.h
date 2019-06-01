/*
 * Backlight Lowlevel Control Abstraction
 *
 * Copyright (C) 2003,2004 Hewlett-Packard Company
 *
 */

#ifndef _LINUX_ONTIM_BACKLIGHT_H
#define _LINUX_ONTIM_BACKLIGHT_H

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/backlight.h>

enum{
	ONTIM_BACKLIGHT_CTRL_GPIO_ONOFF =0
	,ONTIM_BACKLIGHT_CTRL_PWM
	,ONTIM_BACKLIGHT_CTRL_LCDC
};

struct ontim_bl_platform_data{
	const char *name;
	int max_intensity;
	int default_intensity;
	int limit_mask;
	void (*set_bl_intensity)(int intensity);
	void (*kick_battery)(void);
	int ctrl_type;
	int bl_gpio;
	int pwm_id;
	int pwm_clk;
};

#endif
