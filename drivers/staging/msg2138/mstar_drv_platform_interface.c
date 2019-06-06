/*==============================================================================
 *
 * Copyright (c) 2006-2012 MStar Semiconductor, Inc.
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
 * rights to any and all damages, losses, costs and expenses resulting
 therefrom.
 *
==============================================================================*/

/**
 *
 * @file    mstar_drv_platform_interface.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

/*=============================================================*/
/* INCLUDE FILE */
/*=============================================================*/

#include "mstar_drv_platform_interface.h"
#include "mstar_drv_main.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_porting_layer.h"


/*=============================================================*/
/* GLOBAL FUNCTION DEFINITION */
/*=============================================================*/

#ifdef CONFIG_HAS_EARLYSUSPEND
void interface_device_suspend(struct early_suspend *p_suspend)
{
	LOGTP_FUNC();

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
/*    g_gesture_wakeup_mode = 0x1FFF; // Enable all gesture wakeup mode for
 testing */

	if (g_gesture_wakeup_mode != 0x0000) {
		fwic_open_gesture_wakeup(g_gesture_wakeup_mode);
		return;
	}
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

	/* Send touch end for clearing point touch */
	platform_finger_touch_released(0, 0);
	input_sync(g_input_device);

	platform_disable_finger_touchreport();
	platform_device_poweroff();
}

void interface_device_resume(struct early_suspend *p_suspend)
{
	LOGTP_FUNC();

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	if (g_gesture_wakeup_flag == 1)
		fwic_close_getsture_wakeup();
	else
		platform_enable_finger_touchreport();

#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

	platform_device_poweron();
/*
    platform_finger_touch_released(0, 0);
    input_sync(g_input_device);
*/
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
	fwic_restore_firmwaremode_tologdata();
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
	platform_enable_finger_touchreport();
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */
}
#endif

#ifdef TOUCH_VIRTUAL_KEYS
/*static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute
 *attr, char *buf)
{
	return sprintf(buf,
	__stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":192:1000:64:60"
	":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":256:1000:64:60"
	":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":128:1000:64:60"
	":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":64:1000:64:60"
	"\n");
}*/
static ssize_t virtual_keys_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
struct msg2138_ts_platform_data *pdata = gi2c_client->dev.platform_data;
	return sprintf(buf,
	"%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d\n",
	__stringify(EV_KEY), __stringify(KEY_APPSELECT),
	pdata->virtualkeys[0], pdata->virtualkeys[1],
	pdata->virtualkeys[2], pdata->virtualkeys[3]
	, __stringify(EV_KEY), __stringify(KEY_HOMEPAGE),
	pdata->virtualkeys[4], pdata->virtualkeys[5],
	pdata->virtualkeys[6], pdata->virtualkeys[7]
	, __stringify(EV_KEY), __stringify(KEY_BACK),
	pdata->virtualkeys[8], pdata->virtualkeys[9],
	pdata->virtualkeys[10], pdata->virtualkeys[11]);
}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
	.name = "virtualkeys.msg2138_ts",
	.mode = S_IRUGO,
	},
	.show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
	&virtual_keys_attr.attr,
	NULL
};

static struct attribute_group properties_attr_group = {
	.attrs = properties_attrs,
};

static void pixcir_ts_virtual_keys_init(void)
{
	int ret;
	struct kobject *properties_kobj;

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
	&properties_attr_group);
	if (!properties_kobj || ret)
		LOGTP_ERRO("failed to create board_properties\n");
}
#endif

/* probe function is used for matching and initializing input device */
s32  interface_device_probe(struct i2c_client *p_client,/*__devinit*/
	const struct i2c_device_id
	*p_device_id)
{
	s32 n_retval = 0;

	LOGTP_FUNC();

	platform_device_initialize(p_client);

	n_retval = platform_device_request_gpio();
	if (n_retval < 0) {
		LOGTP_ERRO("[Mstar] platform_device_request_gpio failed %d\n",
				n_retval);
		return n_retval;
	}
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
	platform_device_regulator_poweron();
#endif/* CONFIG_ENABLE_REGULATOR_POWER_ON */

	platform_device_poweron();

	n_retval = main_device_initialize();
	if (n_retval < 0) {
		LOGTP_ERRO("[Mstar] main_device_initialize failed %d\n",
				n_retval);
		return n_retval;
	}
#ifdef TOUCH_VIRTUAL_KEYS
	pixcir_ts_virtual_keys_init();
#endif

	platform_device_register_interrupt_handler();

#ifdef CONFIG_HAS_EARLYSUSPEND
	platform_device_register_earlysuspend();
#endif

	LOGTP_INFO("*** MStar touch driver registered ***\n");

	return n_retval;
}

/* remove function is triggered when the input device is removed from input
 sub-system */
s32  ms_interface_device_remove(struct i2c_client *p_client)/*__devexit*/
{
	LOGTP_FUNC();

	return platform_device_remove(p_client);
}

void interface_device_set_i2c_rate(struct i2c_client *p_client,
	u32 n_i2c_rate)
{
	LOGTP_FUNC();

	platform_set_i2crate(p_client, n_i2c_rate);
}
