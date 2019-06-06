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
 * @file    mstar_drv_platform_interface.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.1.1.0
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_interface.h"
#include "mstar_drv_main.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_porting_layer.h"
#include <linux/irq.h>


/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u16 g_GestureWakeupMode;
extern u8 g_GestureWakeupFlag;
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

#if defined(TPD_PROXIMITY)
extern int msg_proximity_flag;
extern int TP_face_mode_switch(int on);
#endif

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

extern struct input_dev *g_InputDevice;
#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM 
extern struct regulator *g_ReguVdd;
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

void MsDrvInterfaceTouchDeviceSuspend(struct early_suspend *pSuspend)
{
    DBG("*** %s() ***\n", __func__);
	
	////for geture by chenxiaoge
   // __raw_writel(BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE, CTL_PIN_BASE + 0x00ec);//for geture by chen

#ifdef TPD_PROXIMITY
	if(msg_proximity_flag == 1)
	return;	
#endif

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
  // g_GestureWakeupMode = 0x3FFF; // Enable all gesture wakeup mode for testing 

    if (g_GestureWakeupMode != 0x0000)
    {
	g_GestureWakeupMode = 0x3FFF; //gxm modify  只用一个总开关
        DrvIcFwLyrOpenGestureWakeup(g_GestureWakeupMode);

	irq_set_irq_type(gpio_to_irq(MS_TS_MSG_IC_GPIO_INT), IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND);
	enable_irq(gpio_to_irq(MS_TS_MSG_IC_GPIO_INT));
        return;
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

    DrvPlatformLyrFingerTouchReleased(0, 0); // Send touch end for clearing point touch
    input_sync(g_InputDevice);

    DrvPlatformLyrDisableFingerTouchReport();
    DrvPlatformLyrTouchDevicePowerOff(); 
}


void MsDrvInterfaceTouchDeviceResume(struct early_suspend *pSuspend)
{
    DBG("*** %s() ***\n", __func__);
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    if (g_GestureWakeupFlag == 1)
    {
        DrvIcFwLyrCloseGestureWakeup();
    }
    else
    {
        DrvPlatformLyrEnableFingerTouchReport(); 
    }
    	irq_set_irq_type(gpio_to_irq(MS_TS_MSG_IC_GPIO_INT),IRQF_TRIGGER_FALLING);

#endif //CONFIG_ENABLE_GESTURE_WAKEUP

    DrvPlatformLyrTouchDevicePowerOn();

#ifdef TPD_PROXIMITY
	if(msg_proximity_flag == 1)
	{
		if (DrvIcFwLyrGetChipType()!=0)
		{
			printk("jinlili msg2133_read_id ok\n");
		}
		else
		{
			DrvPlatformLyrTouchDevicePowerOn();
			msleep(100);
			printk("jinlili msg2133_read_id failed reset IC\n");
		}
		TP_face_mode_switch(1);
	}
#endif
/*
    DrvPlatformLyrFingerTouchReleased(0, 0);
    input_sync(g_InputDevice);
*/    
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    DrvIcFwLyrRestoreFirmwareModeToLogDataMode();
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
    DrvPlatformLyrEnableFingerTouchReport(); 
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
}

/* probe function is used for matching and initializing input device */
s32 /*__devinit*/ MsDrvInterfaceTouchDeviceProbe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId)
{
    s32 nRetVal = 0;

    printk("*** %s() ***\n", __func__);
  
   nRetVal =  DrvPlatformLyrInputDeviceInitialize(pClient);
  
    nRetVal = DrvPlatformLyrTouchDeviceRequestGPIO();

   if(nRetVal<0)									//gxm add for tp Compatible
   {
		return nRetVal;
   }

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM 
    DrvPlatformLyrTouchDeviceRegulatorPowerOn(g_ReguVdd);
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM

    DrvPlatformLyrTouchDevicePowerOn();

    DrvMainTouchDeviceInitialize();

    DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler();

    DrvPlatformLyrTouchDeviceRegisterEarlySuspend();

    DBG("*** MStar touch driver registered ***\n");
    
    return nRetVal;
}

/* remove function is triggered when the input device is removed from input sub-system */
s32 /*__devexit*/ MsDrvInterfaceTouchDeviceRemove(struct i2c_client *pClient)
{
    DBG("*** %s() ***\n", __func__);

    return DrvPlatformLyrTouchDeviceRemove(pClient);
}

void MsDrvInterfaceTouchDeviceSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate)
{
    DBG("*** %s() ***\n", __func__);

    DrvPlatformLyrSetIicDataRate(pClient, nIicDataRate);
}    
