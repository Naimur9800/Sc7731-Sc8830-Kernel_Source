/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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


#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include "gsp_interface_pike2.h"
#include "../gsp_debug.h"
#include "../gsp_interface.h"

static void gsp_interface_pike2_qos_setting(struct gsp_interface_pike2 *intf)
{
	/* write channel latency regulator */
	gsp_core_reg_update(intf->gsp_qos_base + 0x130, 256, 0xfff);
	gsp_core_reg_update(intf->gsp_qos_base + 0x134, 4, 0x7);
	gsp_core_reg_update(intf->gsp_qos_base + 0x138, 0, 0xf);
	gsp_core_reg_update(intf->gsp_qos_base + 0x138, 2 << 8, 0xf << 8);
	gsp_core_reg_update(intf->gsp_qos_base + 0x10c, 0 << 16, 0x1 << 16);
	gsp_core_reg_update(intf->gsp_qos_base + 0x10c, 1 << 3, 0x1 << 3);

	/* read channel latency regulator */
	gsp_core_reg_update(intf->gsp_qos_base + 0x130, 256 << 16, 0xfff << 16);
	gsp_core_reg_update(intf->gsp_qos_base + 0x134, 4 << 8, 0x7 << 8);
	gsp_core_reg_update(intf->gsp_qos_base + 0x138, 0 << 16, 0xf << 16);
	gsp_core_reg_update(intf->gsp_qos_base + 0x138, 2 << 24, 0xf << 24);
	gsp_core_reg_update(intf->gsp_qos_base + 0x10c, 0 << 20, 0x1 << 20);
	gsp_core_reg_update(intf->gsp_qos_base + 0x10c, 1 << 4, 0x1 << 4);
}

int gsp_interface_pike2_parse_dt(struct gsp_interface *intf,
				  struct device_node *node)
{
	int status = 0;
	struct gsp_interface_pike2 *gsp_interface = NULL;
	struct resource res;

	gsp_interface = (struct gsp_interface_pike2 *)intf;
	if (of_address_to_resource(node, 1, &res)) {
		GSP_ERR("r2p0 interface failed to read qos base address\n");
		status |= -1;
	} else {
		gsp_interface->gsp_qos_base = ioremap(res.start,
				resource_size(&res));
	}

	return status;
}

int gsp_interface_pike2_init(struct gsp_interface *intf)
{
	return 0;
}

int gsp_interface_pike2_deinit(struct gsp_interface *intf)
{
	return 0;
}

int gsp_interface_pike2_prepare(struct gsp_interface *intf)
{
	int status = 0;
	struct gsp_interface_pike2 *gsp_interface = NULL;

	gsp_interface = (struct gsp_interface_pike2 *)intf;
	gsp_interface_pike2_qos_setting(gsp_interface);

	return status;
}

int gsp_interface_pike2_unprepare(struct gsp_interface *intf)
{
	return 0;
}

int gsp_interface_pike2_reset(struct gsp_interface *intf)
{
	return 0;
}

void gsp_interface_pike2_dump(struct gsp_interface *intf)
{

}
