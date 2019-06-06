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

#ifndef _DISP_NOTIFY_H_
#define _DISP_NOTIFY_H_

#include <linux/notifier.h>

enum {
	DISP_EVENT_CLOCK_CHANGE,
	DISP_EVENT_ESD_RECOVER,
	DISP_EVENT_DISPC_RUN,
	DISP_EVENT_DISPC_BLACK,
	DISP_EVENT_DISPC_STOP,
};

int vsync_notifier_register(struct notifier_block *nb);
int vsync_notifier_unregister(struct notifier_block *nb);
int vsync_notifier_call_chain(unsigned long val, void *v);

int disp_notifier_register(struct notifier_block *nb);
int disp_notifier_unregister(struct notifier_block *nb);
int disp_notifier_call_chain(unsigned long val, void *v);

#endif
