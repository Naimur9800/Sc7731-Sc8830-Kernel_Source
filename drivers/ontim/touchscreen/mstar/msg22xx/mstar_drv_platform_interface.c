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
 * @version v2.3.0.0
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_interface.h"
#include "mstar_drv_main.h"
#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_utility_adaption.h"
#include "mstar_drv_self_fw_control.h"
#include <ontim/touchscreen/touchpanel.h>

/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/


/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/
#ifdef CONFIG_HAS_EARLYSUSPEND
void MsDrvInterfaceTouchDeviceSuspend(struct early_suspend *pSuspend)
{
    int  suspend=0;

    DBG("*** %s() ***\n", __func__);

    mutex_lock(&msg2xxx_info->ts_suspend_lock);
    
 #ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
    mutex_lock(&msg2xxx_info->ts_lock);
	msg2xxx_info->suspend_state=1;	
    if (msg2xxx_info->ps_onoff == 0)	
    {       
        printk(KERN_INFO "[kernel][%s]: [FTS]msg2xxx suspend[1]\n",__func__);      
        msg2xxx_info->suspend_state=2;        
        suspend =1; 
    }
    mutex_unlock(&msg2xxx_info->ts_lock);
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

    if(suspend)
    {
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    	msg2xxx_info->wakeup_enable = g_GestureWakeupMode & msg2xxx_info->wakeup_support ;
        if ( msg2xxx_info->wakeup_enable && (msg2xxx_info->pdata->irq_gpio >= 0) )
        {
    		struct irq_desc * tp_irq_desc;
    		
    		printk("[kernel][%s %d]:/-/-/-/-/-/edge_wakeup_gpio gpio %d wakeup mode %d/-/-/-/-/-\n",
    			    __func__, __LINE__, msg2xxx_info->pdata->irq_gpio,g_GestureWakeupMode);
    		disable_irq(msg2xxx_info->irq);
            DrvFwCtrlOpenGestureWakeup(g_GestureWakeupMode);
    		tp_irq_desc = irq_to_desc(msg2xxx_info->irq);
    		tp_irq_desc->action->flags = /*IRQF_TRIGGER_RISING*/IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND;	
            irq_set_irq_type(msg2xxx_info->irq, IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND );
    		enable_irq(msg2xxx_info->irq);
    		enable_irq_wake(msg2xxx_info->irq);
        }else
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
        {
            DrvPlatformLyrFingerTouchReleased(0, 0); // Send touch end for clearing point touch
            input_sync(msg2xxx_info->idev);

            DrvPlatformLyrDisableFingerTouchReport();
            //msg2xxx_info->run_state=0;
            DrvPlatformLyrTouchDevicePowerOff(); 
        }
    }
    mutex_unlock(&msg2xxx_info->ts_suspend_lock);
}

void MsDrvInterfaceTouchDeviceResume(struct early_suspend *pSuspend)
{
    int  resume=0;

    DBG("*** %s() ***\n", __func__);
	
	mutex_lock(&msg2xxx_info->ts_suspend_lock);
	mutex_lock(&msg2xxx_info->ts_lock);

    if(msg2xxx_info->suspend_state == 2)
    {
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
        if (g_GestureWakeupFlag)
        {
    		struct irq_desc * tp_irq_desc;
    		
    		printk("[kernel][%s %d]:/-/-/-/-/-/edge_wakeup_gpio gpio %d/-/-/-/-\n", 
    			    __func__, __LINE__, msg2xxx_info->pdata->irq_gpio);
    		disable_irq_wake(msg2xxx_info->irq);
    		disable_irq(msg2xxx_info->irq);
    		tp_irq_desc = irq_to_desc(msg2xxx_info->i2c->irq);
    		tp_irq_desc->action->flags = IRQF_TRIGGER_RISING ; //msg2xxx_info->pdata->irqflags;
            irq_set_irq_type(msg2xxx_info->irq, tp_irq_desc->action->flags );
    		msg2xxx_info->wakeup_enable = 0 ;
    		DrvFwCtrlCloseGestureWakeup() ;
            //DrvPlatformLyrTouchDevicePowerOn();
    		enable_irq(msg2xxx_info->irq) ;
        }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

        DrvPlatformLyrTouchDevicePowerOn();
        enable_irq(msg2xxx_info->irq) ;
    }
    
    msg2xxx_info->suspend_state=0;
    mutex_unlock(&msg2xxx_info->ts_lock);
    /*if(resume)
    {
        msg2xxx_info->run_state=1;
        enable_irq(msg2xxx_info->irq) ;
    }*/
   mutex_unlock(&msg2xxx_info->ts_suspend_lock);
}

#endif
/* probe function is used for matching and initializing input device */
s32 /*__devinit*/ MsDrvInterfaceTouchDeviceProbe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId)
{
    s32 nRetVal = 0;

    if ( !i2c_check_functionality(pClient->adapter, I2C_FUNC_I2C) ) 
        return -1 ;
	
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    DrvPlatformLyrTouchDeviceRegulatorPowerOn();
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

    if ( DrvPlatformLyrTouchDeviceRequestGPIO() < 0 )
        return -2 ;

    if ( DrvPlatformLyrTouchDevicePowerOn() < 0 )
        return -3 ;

	if ( DrvFwCtrlCheckFirmwareUpdateBySwId() < 0 )
		return -4 ;

    if ( DrvPlatformLyrInputDeviceInitialize() < 0 )
		return -5 ;
	
    if ( DrvPlatformLyrRegisterInterruptHandler() < 0 )
		return -6 ;
	
	if ( DrvMainTouchDeviceInitialize() < 0 )
		return -7 ;
    

#ifdef CONFIG_HAS_EARLYSUSPEND
    DrvPlatformLyrTouchDeviceRegisterEarlySuspend();
#endif

    return nRetVal;
}

/* remove function is triggered when the input device is removed from input sub-system */
s32 /*__devexit*/ MsDrvInterfaceTouchDeviceRemove(struct i2c_client *pClient)
{
    DBG("*** %s() ***\n", __func__);

    return DrvPlatformLyrTouchDeviceRemove(pClient);
}


