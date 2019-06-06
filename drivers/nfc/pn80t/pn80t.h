/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */


/*
 * PN547 power control via ioctl
 * PN547_SET_PWR(0): power off
 * PN547_SET_PWR(1): power on
 * PN547_SET_PWR(2): reset and power on with firmware download enabled
 */

#define PN544_MAGIC	0xE9
#define PN544_SET_PWR	_IOW(PN544_MAGIC, 0x01, unsigned int)


#define PN544_NAME "pn544"
#define PN544_ADDRESS 0x28

struct pn544_i2c_platform_data {
	unsigned int irq_gpio;
	unsigned int ven_gpio;
	unsigned int firm_gpio;
};
