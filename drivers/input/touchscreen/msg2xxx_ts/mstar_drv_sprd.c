/*
 *
 * Copyright (c) 2006-2014 MStar Semiconductor, Inc.
 * All rights reserved.
 *
 * Unless otherwise stipulated in writing, any and all information contained
 * herein regardless in any format shall remain the sole proprietary of
 * MStar Semiconductor Inc. and be kept in strict confidence
 * (??MStar Confidential Information??) by the recipient.
 * Any unauthorized act including without limitation unauthorized disclosure,
 * copying, use, reproduction, sale, distribution, modification, disassembling,
 * reverse engineering and compiling of the contents of MStar Confidential
 * Information is unlawful and strictly prohibited. MStar hereby reserves the
 * rights to any and all damages, losses, costs and expenses resulting therefrom.
 *
 */

/**
 *
 * @file	mstar_drv_sprd.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
/*  INCLUDE FILE */
/*=============================================================*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */
#include <linux/i2c.h>
#include <linux/kobject.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "mstar_drv_platform_interface.h"

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
/* #include <mach/regulator.h> */
#include <soc/sprd/regulator.h>
#include <linux/regulator/consumer.h>
#endif /* CONFIG_ENABLE_REGULATOR_POWER_ON */

#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
/*=============================================================*/
/*  CONSTANT VALUE DEFINITION */
/*=============================================================*/
 /* Please define the mstar touch ic name based on the mutual-capacitive ic or self capacitive ic that you are using */
/* #define MSG_TP_IC_NAME "msg2xxx" //"msg21xxA" or "msg22xx" or "msg26xxM" or "msg28xx" */

/*=============================================================*/
/*  VARIABLE DEFINITION */
/*=============================================================*/

struct i2c_client *g_I2cClient;
int TOUCH_SCREEN_X_MAX;
int TOUCH_SCREEN_Y_MAX;
int MS_TS_MSG_IC_GPIO_RST;
int MS_TS_MSG_IC_GPIO_INT;

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
struct regulator *g_ReguVdd;
struct regulator *g_ReguVcc_i2c;
#endif /* CONFIG_ENABLE_REGULATOR_POWER_ON */

#ifdef TP_HAVE_PROX
int is_incall;
struct input_dev *proximity_input_dev_msg;
struct class *firmware_class_msg;
struct device *firmware_cmd_dev_msg;
#endif
#ifdef WAKE_LOCK_LPSENSOR
struct wake_lock pls_delayed_work_wake_lock_msg;
#endif
#ifdef CONFIG_OF
static struct msg2xxx_ts_platform_data *pixcir_ts_parse_dt(struct device *dev)
{
	struct msg2xxx_ts_platform_data *pdata = NULL;
	struct device_node *np = dev->of_node;
	int ret;
	int buf[12];
	int i;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct msg2138_ts_platform_data");
		return NULL;
	}
	pdata->reset_gpio_number = of_get_gpio(np, 0);
	if (pdata->reset_gpio_number < 0) {
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	pdata->irq_gpio_number = of_get_gpio(np, 1);
	if (pdata->irq_gpio_number < 0) {
		dev_err(dev, "fail to get irq_gpio_number\n");
		goto fail;
	}
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
	ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
	if (ret) {
		dev_err(dev, "fail to get vdd_name\n");
		goto fail;
	}
#endif /* CONFIG_ENABLE_REGULATOR_POWER_ON */
	ret = of_property_read_u32_array(np, "virtualkeys", buf, 12);
	if (ret) {
		dev_err(dev, "fail to get virtualkeys\n");
		goto fail;
	}
	for (i = 0; i < 12; i++) {
		pdata->virtualkeys[i] = buf[i];
	}
	ret = of_property_read_u32(np, "TP_MAX_X", &pdata->TP_MAX_X);
	if (ret) {
		dev_err(dev, "fail to get TP_MAX_X\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->TP_MAX_Y);
	if (ret) {
		dev_err(dev, "fail to get TP_MAX_Y\n");
		goto fail;
	}

	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

/*=============================================================*/
/*  FUNCTION DEFINITION */
/*=============================================================*/

/* probe function is used for matching and initializing input device */
static int /*__devinit*/ touch_driver_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct msg2xxx_ts_platform_data *pdata = client->dev.platform_data;
	int err = 0;/* i = 0; */
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
	int ret = 0;
/* 	const char *vdd_name = "vdd"; */
/* 	const char *vcc_i2c_name = "vcc_i2c"; */
#endif /* CONFIG_ENABLE_REGULATOR_POWER_ON */
	struct device_node *np;

	printk("*** %s ***\n", __FUNCTION__);

	if (client == NULL) {
		printk("i2c client is NULL\n");
		return -1;
	}
	g_I2cClient = client;

#ifdef CONFIG_OF
	np = client->dev.of_node;
	if (np && !pdata) {
		pdata = pixcir_ts_parse_dt(&client->dev);
		if (pdata) {
			client->dev.platform_data = pdata;
		} else {
			err = -ENOMEM;
			goto exit_alloc_platform_data_failed;
		}
	}
#endif
	g_I2cClient = client;
	MS_TS_MSG_IC_GPIO_RST = pdata->reset_gpio_number;
	MS_TS_MSG_IC_GPIO_INT = pdata->irq_gpio_number;
	TOUCH_SCREEN_X_MAX = pdata->TP_MAX_X;
	TOUCH_SCREEN_Y_MAX = pdata->TP_MAX_Y;

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
	g_ReguVdd = regulator_get(&g_I2cClient->dev, pdata->vdd_name);

	ret = regulator_set_voltage(g_ReguVdd, 1800000, 1800000);
	if (ret) {
		printk("Could not set to 2800mv.\n");
	}
/*
	g_ReguVcc_i2c = regulator_get(&g_I2cClient->dev, g_I2cClient->dev.platform_data->vcc_i2c_name);

	ret = regulator_set_voltage(g_ReguVcc_i2c, 1800000, 1800000);
	if (ret) {
		printk("Could not set to 1800mv.\n");
	}
*/
#endif /* CONFIG_ENABLE_REGULATOR_POWER_ON */

#if (defined(CONFIG_I2C_SPRD) || defined(CONFIG_I2C_SPRD_V2))
	/* sprd_i2c_ctl_chg_clk(client->adapter->nr, 50000); */
#endif

	return MsDrvInterfaceTouchDeviceProbe(g_I2cClient, id);
exit_alloc_platform_data_failed:
	return err;
}

/* remove function is triggered when the input device is removed from input sub-system */
static int /*__devexit*/ touch_driver_remove(struct i2c_client *client)
{
	printk("*** %s ***\n", __FUNCTION__);

	return MsDrvInterfaceTouchDeviceRemove(client);
}
#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops msg2xxx_ts_dev_pmops = {
	SET_RUNTIME_PM_OPS(MsDrvInterfaceTouchDeviceSuspend, MsDrvInterfaceTouchDeviceResume, NULL)
};
#endif
/* The I2C device list is used for matching I2C device and I2C device driver. */
static const struct i2c_device_id touch_device_id[] = {
	{MSG_TP_IC_NAME, 0},
	{}, /* should not omitted */
};

MODULE_DEVICE_TABLE(i2c, touch_device_id);

static const struct of_device_id touch_dt_match_table[] = {
	{ .compatible = "Mstar, msg2138_ts",},
	{},
};

MODULE_DEVICE_TABLE(of, touch_dt_match_table);

static struct i2c_driver touch_device_driver = {
	.driver = {
		.name = MSG_TP_IC_NAME,
		.owner = THIS_MODULE,
		.of_match_table = touch_dt_match_table,
#ifdef CONFIG_PM_SLEEP
		.pm = &msg2xxx_ts_dev_pmops,
#endif
	},
	.probe = touch_driver_probe,
	.remove = touch_driver_remove,
	.id_table = touch_device_id,
};

static int /*__init*/ touch_driver_init(void)
{
	int ret;

	/* register driver */
	ret = i2c_add_driver(&touch_device_driver);
	if (ret < 0) {
		printk("add MStar touch device driver i2c driver failed.\n");
		return -ENODEV;
	}
	printk("add MStar touch device driver i2c driver.\n");

	return ret;
}

static void /*__exit*/ touch_driver_exit(void)
{
	printk("remove MStar touch device driver i2c driver.\n");

	i2c_del_driver(&touch_device_driver);
}

module_init(touch_driver_init);
module_exit(touch_driver_exit);
MODULE_LICENSE("GPL");
