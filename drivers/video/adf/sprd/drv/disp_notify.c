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

#include <linux/notifier.h>
#include "disp_notify.h"

static ATOMIC_NOTIFIER_HEAD(vsync_notifier_list);
static BLOCKING_NOTIFIER_HEAD(disp_notifier_list);

int vsync_notifier_register(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&vsync_notifier_list, nb);
}
EXPORT_SYMBOL(vsync_notifier_register);

int vsync_notifier_unregister(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&vsync_notifier_list, nb);
}
EXPORT_SYMBOL(vsync_notifier_unregister);

int vsync_notifier_call_chain(unsigned long val, void *v)
{
	return atomic_notifier_call_chain(&vsync_notifier_list, val, v);
}
EXPORT_SYMBOL(vsync_notifier_call_chain);


int disp_notifier_register(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&disp_notifier_list, nb);
}
EXPORT_SYMBOL(disp_notifier_register);

int disp_notifier_unregister(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&disp_notifier_list, nb);
}
EXPORT_SYMBOL(disp_notifier_unregister);

int disp_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&disp_notifier_list, val, v);
}
EXPORT_SYMBOL(disp_notifier_call_chain);
