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
 * @file    mstar_drv_platform_interface.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_interface.h"
#include "mstar_drv_main.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_utility_adaption.h"
#include <linux/irq.h>


#ifdef CONFIG_ENABLE_HOTKNOT
#include "mstar_drv_hotknot.h"
#endif //CONFIG_ENABLE_HOTKNOT

/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u32 g_GestureWakeupMode[2];
extern u8 g_GestureWakeupFlag;

extern int _gIrq;

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
extern u8 g_GestureDebugFlag;
extern u8 g_GestureDebugMode;
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE

#endif //CONFIG_ENABLE_GESTURE_WAKEUP

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
extern u8 g_EnableTpProximity;
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#ifdef CONFIG_ENABLE_GLOVE_MODE
extern u8 g_IsEnableGloveMode;
extern u8 g_IsGloveModeOpen;
#endif //CONFIG_ENABLE_GLOVE_MODE

extern u8 g_IsUpdateFirmware;
extern u8 *_gFwVersion;

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

extern struct input_dev *g_InputDevice;

#ifdef CONFIG_ENABLE_HOTKNOT
extern u8 g_HotKnotState;
extern u32 SLAVE_I2C_ID_DWI2C;
#endif //CONFIG_ENABLE_HOTKNOT

extern u8 IS_FIRMWARE_DATA_LOG_ENABLED;

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

#ifdef CONFIG_ENABLE_HOTKNOT
static u8 _gAMStartCmd[4] = {HOTKNOT_SEND, ADAPTIVEMOD_BEGIN, 0, 0};
#endif //CONFIG_ENABLE_HOTKNOT

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

#ifdef CONFIG_ENABLE_NOTIFIER_FB
int MsDrvInterfaceTouchDeviceFbNotifierCallback(struct notifier_block *pSelf, unsigned long nEvent, void *pData)
{
    struct fb_event *pEventData = pData;
    int *pBlank;

    if (pEventData && pEventData->data && nEvent == FB_EVENT_BLANK)
    {
        pBlank = pEventData->data;

        if (*pBlank == FB_BLANK_UNBLANK)
        {
            DBG("*** %s() TP Resume ***\n", __func__);

            if (g_IsUpdateFirmware != 0) // Check whether update frimware by sw id is finished
            {
                DBG("Not Allow to power on/off touch ic while update firmware by sw id.\n");
                return 0;
            }

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
            if (g_EnableTpProximity == 1)
            {
                DBG("g_EnableTpProximity = %d\n", g_EnableTpProximity);
                return 0;
            }
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION
            
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
            if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT
            {
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
                if (g_GestureDebugMode == 1)
                {
                    DrvIcFwLyrCloseGestureDebugMode();
                }
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE

                if (g_GestureWakeupFlag == 1)
                {
                    DrvIcFwLyrCloseGestureWakeup();
                }
                else
                {
                    DrvPlatformLyrEnableFingerTouchReport(); 
                }
            }
#ifdef CONFIG_ENABLE_HOTKNOT
            else    // Enable touch in hotknot transfer mode
            {
                DrvPlatformLyrEnableFingerTouchReport();     
            }
#endif //CONFIG_ENABLE_HOTKNOT
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
    
#ifdef CONFIG_ENABLE_HOTKNOT
            if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT        
            {
                DrvPlatformLyrTouchDevicePowerOn(); 
            }   
    
#ifdef CONFIG_ENABLE_GLOVE_MODE
            if (g_IsEnableGloveMode == 1)
            {
                if (g_IsGloveModeOpen == 0)
                {
                    DrvIcFwCtrlOpenGloveMode();
                }
            }
#endif //CONFIG_ENABLE_GLOVE_MODE

            if (IS_FIRMWARE_DATA_LOG_ENABLED)    
            {
                DrvIcFwLyrRestoreFirmwareModeToLogDataMode(); // Mark this function call for avoiding device driver may spend longer time to resume from suspend state.
            } //IS_FIRMWARE_DATA_LOG_ENABLED

#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
            DrvPlatformLyrEnableFingerTouchReport(); 
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
        }
        else if (*pBlank == FB_BLANK_POWERDOWN)
        {
            DBG("*** %s() TP Suspend ***\n", __func__);
            
            if (g_IsUpdateFirmware != 0) // Check whether update frimware by sw id is finished
            {
                DBG("Not Allow to power on/off touch ic while update firmware by sw id.\n");
                return 0;
            }

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
            if (g_EnableTpProximity == 1)
            {
                DBG("g_EnableTpProximity = %d\n", g_EnableTpProximity);
                return 0;
            }
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
            if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT
            {
                if (g_GestureWakeupMode[0] != 0x00000000 || g_GestureWakeupMode[1] != 0x00000000)
                {
                    DrvIcFwLyrOpenGestureWakeup(&g_GestureWakeupMode[0]);
                    return 0;
                }
            }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

#ifdef CONFIG_ENABLE_HOTKNOT
            if (g_HotKnotState == HOTKNOT_BEFORE_TRANS_STATE || g_HotKnotState == HOTKNOT_TRANS_STATE || g_HotKnotState == HOTKNOT_AFTER_TRANS_STATE)
            {
                IicWriteData(SLAVE_I2C_ID_DWI2C, &_gAMStartCmd[0], 4); 
            }
#endif //CONFIG_ENABLE_HOTKNOT 

#ifdef CONFIG_ENABLE_GLOVE_MODE
            if (g_IsEnableGloveMode == 1)
            {
                if (g_IsGloveModeOpen == 1)
                {
                    DrvIcFwCtrlCloseGloveMode();
                }
            }
#endif //CONFIG_ENABLE_GLOVE_MODE

            DrvPlatformLyrFingerTouchReleased(0, 0); // Send touch end for clearing point touch
            input_sync(g_InputDevice);

            DrvPlatformLyrDisableFingerTouchReport();

#ifdef CONFIG_ENABLE_HOTKNOT
            if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT        
            {
                DrvPlatformLyrTouchDevicePowerOff(); 
            }    
        }
    }

    return 0;
}

#else

void MsDrvInterfaceTouchDeviceSuspend(struct early_suspend *pSuspend)
{
    DBG("*** %s() ***\n", __func__);

    if (g_IsUpdateFirmware != 0) // Check whether update frimware by sw id is finished
    {
        DBG("Not Allow to power on/off touch ic while update firmware by sw id.\n");
        return;
    }

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
    if (g_EnableTpProximity == 1)
    {
        DBG("g_EnableTpProximity = %d\n", g_EnableTpProximity);
        return;
    }
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	irq_set_irq_type(_gIrq,IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND | IRQF_ONESHOT);

#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT
    {
        if (g_GestureWakeupMode[0] != 0x00000000 || g_GestureWakeupMode[1] != 0x00000000)
        {
            DrvIcFwLyrOpenGestureWakeup(&g_GestureWakeupMode[0]);
            return;
        }
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState == HOTKNOT_BEFORE_TRANS_STATE || g_HotKnotState == HOTKNOT_TRANS_STATE || g_HotKnotState == HOTKNOT_AFTER_TRANS_STATE)
    {
        IicWriteData(SLAVE_I2C_ID_DWI2C, &_gAMStartCmd[0], 4); 
    }
#endif //CONFIG_ENABLE_HOTKNOT  

#ifdef CONFIG_ENABLE_GLOVE_MODE
    if (g_IsEnableGloveMode == 1)
    {
        if (g_IsGloveModeOpen == 1)
        {
            DrvIcFwCtrlCloseGloveMode();
        }
    }
#endif //CONFIG_ENABLE_GLOVE_MODE

    DrvPlatformLyrFingerTouchReleased(0, 0); // Send touch end for clearing point touch
    input_sync(g_InputDevice);

    DrvPlatformLyrDisableFingerTouchReport();

#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT        
    {
        DrvPlatformLyrTouchDevicePowerOff(); 
    }    
}

void MsDrvInterfaceTouchDeviceResume(struct early_suspend *pSuspend)
{
    DBG("*** %s() ***\n", __func__);

    if (g_IsUpdateFirmware != 0) // Check whether update frimware by sw id is finished
    {
        DBG("Not Allow to power on/off touch ic while update firmware by sw id.\n");
        return;
    }

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
    if (g_EnableTpProximity == 1)
    {
        DBG("g_EnableTpProximity = %d\n", g_EnableTpProximity);
        return;
    }
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP

	irq_set_irq_type(_gIrq,IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
	
#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT
    {
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
        if (g_GestureDebugMode == 1)
        {
            DrvIcFwLyrCloseGestureDebugMode();
        }
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE

        if (g_GestureWakeupFlag == 1)
        {
            DrvIcFwLyrCloseGestureWakeup();
        }
        else
        {
            DrvPlatformLyrEnableFingerTouchReport(); 
        }
    }
#ifdef CONFIG_ENABLE_HOTKNOT
    else    // Enable touch in hotknot transfer mode
    {
        DrvPlatformLyrEnableFingerTouchReport();     
    }
#endif //CONFIG_ENABLE_HOTKNOT
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
    
#ifdef CONFIG_ENABLE_HOTKNOT
    if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif //CONFIG_ENABLE_HOTKNOT        
    {
        DrvPlatformLyrTouchDevicePowerOn();
    }   
    
#ifdef CONFIG_ENABLE_GLOVE_MODE
    if (g_IsEnableGloveMode == 1)
    {
        if (g_IsGloveModeOpen == 0)
        {
            DrvIcFwCtrlOpenGloveMode();
        }
    }
#endif //CONFIG_ENABLE_GLOVE_MODE

    if (IS_FIRMWARE_DATA_LOG_ENABLED)    
    {
        DrvIcFwLyrRestoreFirmwareModeToLogDataMode(); // Mark this function call for avoiding device driver may spend longer time to resume from suspend state.
    } //IS_FIRMWARE_DATA_LOG_ENABLED

#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
    DrvPlatformLyrEnableFingerTouchReport(); 
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

}

#endif //CONFIG_ENABLE_NOTIFIER_FB

/* probe function is used for matching and initializing input device */
s32 /*__devinit*/ MsDrvInterfaceTouchDeviceProbe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId)
{
    s32 nRetVal = 0;
	u16 nMajor = 0, nMinor = 0;

    DBG("*** %s() ***\n", __func__);
  
    DrvPlatformLyrInputDeviceInitialize(pClient);
  
    nRetVal = DrvPlatformLyrTouchDeviceRequestGPIO();
    if(nRetVal < 0 )
		return nRetVal;

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    DrvPlatformLyrTouchDeviceRegulatorPowerOn();
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

    DrvPlatformLyrTouchDevicePowerOn();

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
    DmaAlloc();
#endif //CONFIG_ENABLE_DMA_IIC
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

    nRetVal = DrvMainTouchDeviceInitialize();
    if (nRetVal == -ENODEV)
    {
        DrvPlatformLyrTouchDeviceRemove(pClient);
        return nRetVal;
    }

    DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler();

    DrvPlatformLyrTouchDeviceRegisterEarlySuspend();

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
    DrvIcFwLyrCheckFirmwareUpdateBySwId();
#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID

    DBG("*** MStar touch driver registered ***\n");

    //DrvIcFwLyrGetCustomerFirmwareVersion(&nMajor, &nMinor, &_gFwVersion);
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
