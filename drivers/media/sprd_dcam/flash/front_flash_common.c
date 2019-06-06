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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#ifndef CONFIG_64BIT
#include <soc/sprd/board.h>
#include <soc/sprd/hardware.h>
#include <soc/sprd/adi.h>
#endif

#include <linux/gpio.h>

#ifdef SPRD_FRONT_FLASH_CUSTOMER_CTRL
#define SPRD_FLASH_ON		1
#define SPRD_FLASH_OFF		0

#undef GPIO_CAM_FLASH_EN

#define GPIO_CAM_FLASH_EN 136
#endif

int sprd_front_flash_on(void)
{
#ifdef SPRD_FRONT_FLASH_CUSTOMER_CTRL
	gpio_request(GPIO_CAM_FLASH_EN, "cam_flash_en");
	gpio_direction_output(GPIO_CAM_FLASH_EN, SPRD_FLASH_ON);
	gpio_free(GPIO_CAM_FLASH_EN);
#endif
	return 0;
}

int sprd_front_flash_high_light(void)
{
#ifdef SPRD_FRONT_FLASH_CUSTOMER_CTRL
	gpio_request(GPIO_CAM_FLASH_EN, "cam_flash_en");
	gpio_direction_output(GPIO_CAM_FLASH_EN, SPRD_FLASH_ON);
	gpio_free(GPIO_CAM_FLASH_EN);
#endif
	return 0;
}

int sprd_front_flash_close(void)
{
#ifdef SPRD_FRONT_FLASH_CUSTOMER_CTRL
	gpio_request(GPIO_CAM_FLASH_EN, "cam_flash_en");
	gpio_direction_output(GPIO_CAM_FLASH_EN, SPRD_FLASH_OFF);
	gpio_free(GPIO_CAM_FLASH_EN);
#endif
	return 0;
}

int sprd_front_flash_cfg(struct sprd_flash_cfg_param *param, void *arg)
{
	return 0;
}
