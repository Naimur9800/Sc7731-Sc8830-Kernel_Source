////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2012 MStar Semiconductor, Inc.
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
 * @version v2.1.1.0
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
#include <soc/sprd/board.h>

#include <soc/sprd/regulator.h>
#include <linux/regulator/consumer.h>

#include "mstar_drv_platform_interface.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_porting_layer.h"

#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_I2C_SPRD
#include <soc/sprd/i2c-sprd.h>
#endif

/*=============================================================*/
// CONSTANT VALUE DEFINITION
/*=============================================================*/

#define MSG_TP_IC_NAME "msg2xxx" //"msg21xxA" or "msg22xx" or "msg26xxM" /* Please define the mstar touch ic name based on the mutual-capacitive ic or self capacitive ic that you are using */

/*=============================================================*/
// VARIABLE DEFINITION
/*=============================================================*/

struct i2c_client *g_I2cClient = NULL;
struct regulator *g_ReguVdd = NULL;

/*=============================================================*/
// FUNCTION DEFINITION
/*=============================================================*/

struct msg_ts_platform_data{
	int irq_gpio_number;
	int reset_gpio_number;
	const char *vdd_name;
	int virtualkeys[12];
	int TP_MAX_X;
	int TP_MAX_Y;
};

#ifdef CONFIG_OF
static struct msg_ts_platform_data *msg2133_ts_parse_dt(struct device *dev)
{
	struct msg_ts_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret,i=0;
///DJT Jinlili modify begin
#if defined(CONFIG_LCMSIZE_WVGA)
	u32 data[12]  = {60, 900, 30, 30,
                         180, 900, 30, 30,
                         300, 900, 30, 30};
	int TP_MAX_X = 480;
	int TP_MAX_Y = 800;
#elif defined(CONFIG_LCMSIZE_HVGA)
	u32 data[12]  = {60, 540, 30, 30,
                         120, 540, 30, 30,
                         240, 540, 30, 30};
	int TP_MAX_X = 320;
	int TP_MAX_Y = 480;
#elif defined(CONFIG_LCMSIZE_FWVGA)
	u32 data[12]  = {100, 1020, 80, 65,
                         280, 1020, 80, 65,
                         470, 1020, 80, 65};
	int TP_MAX_X = 480;
	int TP_MAX_Y = 854;
#elif defined(CONFIG_LCMSIZE_QHD)
	u32 data[12]  = {100, 1100, 80, 65,
                         280, 1100, 80, 65,
                         470, 1100, 80, 65};
	int TP_MAX_X  = 540;
	int TP_MAX_Y  = 960;
#elif defined(CONFIG_LCMSIZE_HD)
	u32 data[12]  = {100, 1320, 80, 65,
                         280, 1320, 80, 65,
                         470, 1320, 80, 65};
	int TP_MAX_X  = 720;
	int TP_MAX_Y  = 1280;
#endif
///DJT Jinlili modify end
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct ft5x0x_ts_platform_data");
		return NULL;
	}
	pdata->reset_gpio_number = of_get_gpio(np, 0);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	pdata->irq_gpio_number = of_get_gpio(np, 1);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
	if(ret){
		dev_err(dev, "fail to get vdd_name\n");
		goto fail;
	}

///DJT Jinlili modify begin
#if defined(CONFIG_LCMSIZE_WVGA) ||defined(CONFIG_LCMSIZE_FWVGA)||defined(CONFIG_LCMSIZE_QHD)||defined(CONFIG_LCMSIZE_HD)|| defined(CONFIG_LCMSIZE_HVGA) 
	for(i = 0; i < 12; i ++){
		pdata->virtualkeys[i] = data[i];
	}
	pdata->TP_MAX_X = TP_MAX_X;
	pdata->TP_MAX_Y = TP_MAX_Y;
#else ///DJT Jinlili modify 
	ret = of_property_read_u32_array(np, "virtualkeys", &pdata->virtualkeys,12);
	if(ret){
		dev_err(dev, "fail to get virtualkeys\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_X", &pdata->TP_MAX_X);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_X\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->TP_MAX_Y);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_Y\n");
		goto fail;
	}
#endif ///DJT Jinlili modify 
	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif



#if VIRTUAL_KEYS
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
#if defined(CONFIG_DJT_PRJ_V131_DJT_001)
	   return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":390:850:15:20"  ///90
			":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":330:850:15:20"  ///150
			":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":455:850:15:5" ///25
			//":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":220:%d:15:5"
			"\n");
#elif defined(CONFIG_DJT_PRJ_V110_JTV_011)
	   return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":90:850:15:20"  ///90
			":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":25:850:15:20"  ///150
			":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":150:850:15:5" ///25
			//":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":220:%d:15:5"
			"\n");
#elif defined(CONFIG_DJT_PRJ_V125_FNG)
		return sprintf(buf, 				   
				   __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":25:904:50:50"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":90:904:50:50"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":150:904:60:50"
			   "\n");	
	
#elif defined(CONFIG_LCMSIZE_QHD)
		return sprintf(buf, 				   
				   __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":25:1020:50:50"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":90:1020:50:50"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":150:1020:60:50"
			   "\n");	
#elif defined(CONFIG_LCMSIZE_HD)
		return sprintf(buf, 				   
				   __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":25:1320:50:50"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":90:1320:50:50"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":150:1320:60:50"
			   "\n");
#elif defined(CONFIG_LCMSIZE_WVGA)&&defined(CONFIG_DJT_PRJ_C390_HWF_042)
		return sprintf(buf, 				   
				   __stringify(EV_KEY) ":" __stringify(KEY_APPSELECT) ":25:904:30:30"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":90:904:30:30"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":150:904:30:30"
			   "\n");	
#elif defined(CONFIG_LCMSIZE_WVGA)&&defined(CONFIG_DJT_PRJ_C390G_HWF_042)
		return sprintf(buf, 				   
				   __stringify(EV_KEY) ":" __stringify(KEY_APPSELECT) ":25:904:30:30"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":90:904:30:30"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":150:904:30:30"
			   "\n");
#elif defined(CONFIG_LCMSIZE_WVGA)&&defined(CONFIG_DJT_PRJ_C390_HLG_0R3)
		return sprintf(buf, 				   
				   __stringify(EV_KEY) ":" __stringify(KEY_APPSELECT) ":25:904:30:30"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":90:904:30:30"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":150:904:30:30"
			   "\n");

#elif defined(CONFIG_LCMSIZE_WVGA)
		return sprintf(buf, 				   
				   __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":25:904:30:30"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":90:904:30:30"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":150:904:30:30"
			   "\n");	

#elif defined(CONFIG_LCMSIZE_HVGA)
		return sprintf(buf, 				   
				   __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":60:540:30:30"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":120:540:30:30"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":240:540:30:30"
			   "\n");	

#elif defined(CONFIG_DJT_PRJ_C177_YMT) && defined(CONFIG_LCMSIZE_FWVGA)
		return sprintf(buf, 				   
			   __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":25:1020:50:50"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":90:1020:50:50"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":150:1020:50:50"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_F1) ":300:1020:50:50"
			   "\n");

#elif defined(CONFIG_LCMSIZE_FWVGA) && defined(CONFIG_DJT_PRJ_C325_HLG_152)
	return sprintf(buf, 				   
			   __stringify(EV_KEY) ":" __stringify(KEY_APPSELECT) ":25:1020:50:50"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":90:1020:50:50"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":150:1020:50:50"
		   "\n");
           
#elif defined(CONFIG_LCMSIZE_FWVGA) && defined(CONFIG_DJT_PRJ_C325_HLG)
	return sprintf(buf, 				   
			   __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":25:1020:50:50"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":90:1020:50:50"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":150:1020:50:50"
		   "\n");
#elif defined(CONFIG_LCMSIZE_FWVGA) && defined(CONFIG_DJT_PRJ_C325V_HLG)
	return sprintf(buf, 				   
			   __stringify(EV_KEY) ":" __stringify(KEY_APPSELECT) ":25:1020:50:50"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":90:1020:50:50"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":150:1020:50:50"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_F1) ":300:1020:50:50"
		   "\n");			   
#elif defined(CONFIG_LCMSIZE_FWVGA)&& defined(CONFIG_DJT_PRJ_C168_XLL_051)
	return sprintf(buf, 				   
			   __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":25:1020:50:50"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":90:1020:50:50"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":150:1020:50:50"
			 ":" __stringify(EV_KEY) ":" __stringify(KEY_F1) ":300:1020:50:50"
		   "\n");
			   			   			   	
#elif defined(CONFIG_LCMSIZE_FWVGA)
	return sprintf(buf, 				   
			   __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":25:1020:50:50"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":90:1020:50:50"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":150:1020:50:50"
		   "\n");

#elif defined(CONFIG_DJT_PRJ_V173_WKT)
		return sprintf(buf,
				   __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":25:1020:50:50"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":90:1020:50:50"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":150:1020:60:50"
			   "\n");
#else
		return sprintf(buf, 				   
			   __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":25:904:50:50"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":90:904:50:50"
			   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":150:904:60:50"
			   "\n");	
#endif
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.msg2133_ts", //virtualkeys.msg2133
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static struct kobject *properties_kobj = NULL;

static void virtual_keys_init(void)
{
    int ret;

    DBG("%s()\n", __FUNCTION__);

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
    {
        ret = sysfs_create_group(properties_kobj, &properties_attr_group);
    }

    if (!properties_kobj || ret)
    {
        pr_err("failed to create board_properties\n");
    }
}
#endif



/* probe function is used for matching and initializing input device */
static int /*__devinit*/ touch_driver_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{

	struct msg_ts_platform_data *pdata = client->dev.platform_data;
	int err = 0;

#ifdef CONFIG_OF
	struct device_node *np = client->dev.of_node;
	if (np && !pdata){
		pdata = msg2133_ts_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
		}
		else{
			err = -ENOMEM;
			return err;
		}
	}
#endif


    DBG("*** %s ***\n", __FUNCTION__);

    if (client == NULL)
    {
        DBG("i2c client is NULL\n");
        return -1;
    }


    g_I2cClient = client;

    g_ReguVdd = regulator_get(&g_I2cClient->dev, pdata->vdd_name);


#ifdef CONFIG_I2C_SPRD
	sprd_i2c_ctl_chg_clk(client->adapter->nr, 100000);
#endif

	//for geature by chenxioage
 	//regulator_set_mode(g_ReguVdd, REGULATOR_MODE_STANDBY);//change for geature
	if (DrvIcFwLyrGetChipType()!=0)
	{
#if VIRTUAL_KEYS
    virtual_keys_init();
#endif
		return MsDrvInterfaceTouchDeviceProbe(g_I2cClient, id);
	}
	else
	{
		return -ENODEV;
	}

}

/* remove function is triggered when the input device is removed from input sub-system */
static int /*__devexit*/ touch_driver_remove(struct i2c_client *client)
{
    DBG("*** %s ***\n", __FUNCTION__);

    return MsDrvInterfaceTouchDeviceRemove(client);
}

/* The I2C device list is used for matching I2C device and I2C device driver. */
static const struct i2c_device_id touch_device_id[] =
{
    {MSG_TP_IC_NAME, 0}, //SLAVE_I2C_ID_DWI2C
    {}, /* should not omitted */ 
};

MODULE_DEVICE_TABLE(i2c, touch_device_id);

static const struct of_device_id focaltech_of_match[] = {
       { .compatible = "msg2133,msg2133_ts", },
       { }
};
MODULE_DEVICE_TABLE(of, focaltech_of_match);

static struct i2c_driver touch_device_driver =
{
    .driver = {
        .name = MSG_TP_IC_NAME,
        .owner = THIS_MODULE,
        .of_match_table = focaltech_of_match,
    },
    .probe = touch_driver_probe,
    .remove = touch_driver_remove,
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
