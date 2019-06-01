////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_sprd.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.3.0.0
 *
 */

/*=============================================================*/
// INCLUDE FILE
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
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/kobject.h>
#include <asm/irq.h>
#include <asm/io.h>
#if defined(CONFIG_I2C_SPRD) || defined(CONFIG_I2C_SPRD_V1)
#include <soc/sprd/i2c-sprd.h>
#endif

#include "mstar_drv_platform_interface.h"

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
#include <soc/sprd/regulator.h>
#include <linux/regulator/consumer.h>
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON
#include <ontim/ontim_dev_dgb.h>

struct msg2xxx_cfg_info *msg2xxx_info = NULL;

static char msg2xxx_version[25];
static char msg2xxx_vendor_name[50];
static char msg2xxx_info_vendor_ID[10] = "Null";
static char msg2xxx_main_vendor_ID[10] = "Null";
static u32  msg2xxx_wakeup_support = 0;

DEV_ATTR_DECLARE(touch_screen)
DEV_ATTR_DEFINE("version",msg2xxx_version)
DEV_ATTR_DEFINE("vendor",msg2xxx_vendor_name)
DEV_ATTR_DEFINE("vendor_id",msg2xxx_main_vendor_ID)
DEV_ATTR_DEFINE("info_vendor_id",msg2xxx_info_vendor_ID)
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
DEV_ATTR_VAL_DEFINE("wakeup_support",&msg2xxx_wakeup_support,ONTIM_DEV_ARTTR_TYPE_VAL_RO)
DEV_ATTR_EXEC_DEFINE_PARAM("wakeup_mode",DrvPlatformLyrGestureWakeupMode)
#endif
DEV_ATTR_DECLARE_END;
ONTIM_DEBUG_DECLARE_AND_INIT(touch_screen,touch_screen,8);

/*=============================================================*/
// CONSTANT VALUE DEFINITION
/*=============================================================*/

#define MSG_TP_IC_NAME "MSG2XXX" 
/*=============================================================*/
// VARIABLE DEFINITION
/*=============================================================*/
//struct i2c_client *g_I2cClient = NULL;

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
struct regulator *g_ReguVdd = NULL;
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

extern bool tp_probe_ok;//add by liuwei
/*=============================================================*/
// FUNCTION DEFINITION
/*=============================================================*/

/* probe function is used for matching and initializing input device */
static int /*__devinit*/ touch_driver_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
//    const char *vdd_name = "vdd";
    s32 retval = 0 ;

    printk("[kernel][%s %d]/-/-/-/-/-/start MStar IC CTP INIT/-/-/-/-/-/\n",
        __func__,__LINE__);

    if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
        return -1 ;
       
    if (client == NULL)
    {
        DBG("i2c client is NULL\n");
        return -1;
    }
    
    msg2xxx_info = kzalloc(sizeof(struct msg2xxx_cfg_info), GFP_KERNEL);
    if (msg2xxx_info == NULL)
        return -ENOMEM;
    mutex_init(&msg2xxx_info->ts_lock);
    mutex_init(&msg2xxx_info->ts_suspend_lock);
	wake_lock_init(&msg2xxx_info->wlock, WAKE_LOCK_SUSPEND, "ps_wakelock");

	i2c_set_clientdata(client, msg2xxx_info);

    msg2xxx_info->i2c = client;
    msg2xxx_info->i2c_default_clk = 300000;
    msg2xxx_info->i2c_update_clk  = 100000;
    msg2xxx_info->pdata = (struct touchpanel_platform_data *)client->dev.platform_data;
    if (msg2xxx_info->pdata == NULL)
    {
        printk("[kernel][%s() %d]/-/-/-/-/-/error/-/-/-/-/-/\n", __func__, __LINE__);
        goto free_mem ; ;
    }

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    g_ReguVdd = regulator_get(&msg2xxx_info->i2c->dev, msg2xxx_info->i2c->dev.platform_data->vdd_name);
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

#if defined(CONFIG_I2C_SPRD) || defined(CONFIG_I2C_SPRD_V1)
    sprd_i2c_ctl_chg_clk(msg2xxx_info->i2c->adapter->nr, msg2xxx_info->i2c_default_clk);
#endif

    retval = MsDrvInterfaceTouchDeviceProbe(msg2xxx_info->i2c, id);
    switch(retval)
    {
        case -1 :
        case -2 :
            goto free_mem ;
        case -3 :
        case -4 :
        case -5 :
            goto free_gpio;
        case -6 :
            goto free_inputdevice;
        case -7 :
            goto free_irq;
        default :
            break ;
    }
    DrvPlatformLyrGetVerInfo(msg2xxx_version, msg2xxx_vendor_name, 
            msg2xxx_main_vendor_ID, msg2xxx_info_vendor_ID,&msg2xxx_wakeup_support) ;

    REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
    
    if(!tp_probe_ok)//add by liuwei
        tp_probe_ok = 1;//add by liuwei
    printk("[kernel][%s %d ]/-/-/-/-/-/ MStar touch driver register Succsess/-/-/-/-/-/\n",
        __func__,__LINE__);
    
    return retval ;
        
free_irq:
    free_irq(client->irq, msg2xxx_info);
free_inputdevice:    
    DrvPlatformLyrInputDeviceUnInitialize();
free_gpio:
    DrvPlatformLyrTouchDeviceFreeGPIO();
free_mem:
    i2c_set_clientdata(client, NULL);
    mutex_destroy(&msg2xxx_info->ts_lock);
    mutex_destroy(&msg2xxx_info->ts_suspend_lock);
	wake_lock_destroy(&msg2xxx_info->wlock);
    kfree(msg2xxx_info);
    msg2xxx_info = NULL;
    return retval ;
}

/* remove function is triggered when the input device is removed from input sub-system */
static int /*__devexit*/ touch_driver_remove(struct i2c_client *client)
{
    DBG("*** %s ***\n", __FUNCTION__);

    return MsDrvInterfaceTouchDeviceRemove(client);
}

static int touch_driver_resume(struct i2c_client *client)
{
    MsDrvInterfaceTouchDeviceResume(NULL);
    return 0;
}

static int touch_driver_suspend(struct i2c_client *client, pm_message_t mesg)
{
    MsDrvInterfaceTouchDeviceSuspend(NULL);
    return 0;
}

/* The I2C device list is used for matching I2C device and I2C device driver. */
static const struct i2c_device_id touch_device_id[] =
{
    {MSG_TP_IC_NAME, 0}, //SLAVE_I2C_ID_DWI2C
    {}, /* should not omitted */ 
};

MODULE_DEVICE_TABLE(i2c, touch_device_id);

static struct i2c_driver touch_device_driver =
{
    .driver = {
        .name = MSG_TP_IC_NAME,
        .owner = THIS_MODULE,
    },
    .probe   = touch_driver_probe,
    .remove  = touch_driver_remove,
    //.suspend = touch_driver_suspend,
    //.resume  = touch_driver_resume,
/*    .remove = __devexit_p(touch_driver_remove), */
    .id_table = touch_device_id,
};

static int /*__init*/ touch_driver_init(void)
{
    int ret;

    /* register driver */
    ret = i2c_add_driver(&touch_device_driver);
    if (ret < 0)
    {
        DBG("add touch device driver i2c driver failed.\n");
        return -ENODEV;
    }
    DBG("add touch device driver i2c driver.\n");

    return ret;
}

static void /*__exit*/ touch_driver_exit(void)
{
    DBG("remove touch device driver i2c driver.\n");

    i2c_del_driver(&touch_device_driver);
}

module_init(touch_driver_init);
module_exit(touch_driver_exit);
MODULE_LICENSE("GPL");
