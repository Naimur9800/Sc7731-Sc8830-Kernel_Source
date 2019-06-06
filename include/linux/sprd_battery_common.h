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
#ifndef _SPRD_BATTERY_COMMON_H_
#define _SPRD_BATTERY_COMMON_H_

#include <linux/types.h>
#include <linux/power_supply.h>
#include <uapi/linux/usb/charger.h>
#include <linux/usb/charger.h>

extern enum usb_charger_type
sprdchg_charger_is_adapter_for_usb(struct usb_charger *p);

static inline int sprdpsy_get_property(char *name,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	struct power_supply *psy;

	if (!name)
		return -EINVAL;

	psy = power_supply_get_by_name(name);

	if (!psy) {
		val->intval = 0;
		pr_err("Not find psy %s\n", name);
		return -ENXIO;
	}

	if (!psy->desc->get_property) {
		val->intval = 0;
		pr_err("None get_property\n");
		return -ENXIO;
	}

	ret = psy->desc->get_property(psy, psp, val);

	if (ret)
		val->intval = 0;

	return ret;
}

static inline int sprdpsy_set_property(char *name,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	struct power_supply *psy;

	if (!name)
		return -EINVAL;

	psy = power_supply_get_by_name(name);

	if (!psy) {
		val->intval = 0;
		pr_err("Fail to find psy %s\n", name);
		return -ENXIO;
	}

	if (!psy->desc->set_property) {
		val->intval = 0;
		pr_err("None set_property\n");
		return -ENXIO;
	}

	ret = psy->desc->set_property(psy, psp, val);

	return ret;
}

#endif
