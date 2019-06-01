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
 * @file    mstar_drv_platform_porting_layer.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.3.0.0
 *
 */
 
/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_self_fw_control.h"
#include "mstar_drv_platform_interface.h"
#include <ontim/ontim_dev_dgb.h>

/*=============================================================*/
// EXTREN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern struct kset *g_TouchKSet;
extern struct kobject *g_TouchKObj;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG


#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
extern struct tpd_device *tpd;
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
extern struct regulator *g_ReguVdd;
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON
#endif

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
static struct early_suspend _gEarlySuspend;
#endif

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifndef CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
static DECLARE_WAIT_QUEUE_HEAD(_gWaiter);
static struct task_struct *_gThread = NULL;
static int _gTpdFlag = 0;
#endif //CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

#ifdef CONFIG_TP_HAVE_KEY
const int g_TpVirtualKey[] = {TOUCH_KEY_MENU, TOUCH_KEY_HOME, TOUCH_KEY_BACK, TOUCH_KEY_SEARCH};

#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
#define BUTTON_W (100)
#define BUTTON_H (100)

const int g_TpVirtualKeyDimLocal[MAX_KEY_NUM][4] = {{BUTTON_W/2*1,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H},{BUTTON_W/2*3,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H},{BUTTON_W/2*5,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H},{BUTTON_W/2*7,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H}};
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
#endif //CONFIG_TP_HAVE_KEY

/*=============================================================*/
// LOCAL FUNCTION DEFINITION
/*=============================================================*/

/* read data through I2C then report data to input sub-system when interrupt occurred */
static void _DrvPlatformLyrFingerTouchDoWork(struct work_struct *pWork)
{
    DBG("*** %s() ***\n", __func__);
    
    mutex_lock(&msg2xxx_info->ts_lock); 

    DrvFwCtrlHandleFingerTouch();

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
//    enable_irq(MS_TS_MSG_IC_GPIO_INT);
    enable_irq(msg2xxx_info->irq);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
#endif

    mutex_unlock(&msg2xxx_info->ts_lock);  
}

static void _DrvPlatformLyrPSDoWork(struct work_struct *pWork)
{
    static unsigned char v = 0xFF ;
    mutex_lock(&msg2xxx_info->ts_suspend_lock);
	mutex_lock(&msg2xxx_info->ts_lock);
	if (msg2xxx_info->ps_onoff)
	{
		v ^= 0x1 ;
		msg2xxx_info->ps_state = v;
		if (msg2xxx_info->suspend_state == 0) 
			DrvPlatformLyrTpPsEnable(1);

		input_report_abs(msg2xxx_info->ps_input_dev, 
					ABS_DISTANCE, msg2xxx_info->ps_state);
		input_sync(msg2xxx_info->ps_input_dev);
	}
	mutex_unlock(&msg2xxx_info->ts_lock);
	mutex_unlock(&msg2xxx_info->ts_suspend_lock);
}


#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
/* The interrupt service routine will be triggered when interrupt occurred */
static irqreturn_t _DrvPlatformLyrFingerTouchInterruptHandler(s32 nIrq, void *pDeviceId)
{    
    disable_irq_nosync(msg2xxx_info->irq);
    schedule_work(&msg2xxx_info->work);
    return IRQ_HANDLED;
}
#endif

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
void DrvPlatformLyrTouchDeviceRegulatorPowerOn(void)
{
#ifdef (CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);
    
    nRetVal = regulator_set_voltage(g_ReguVdd, 2800000, 2800000); // For specific SPRD BB chip(ex. SC7715) or QCOM BB chip(ex. MSM8610), need to enable this function call for correctly power on Touch IC.

    if (nRetVal)
    {
        DBG("Could not set to 2800mv.\n");
    }
    regulator_enable(g_ReguVdd);

    mdelay(20); //mdelay(100);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_2800, "TP"); // For specific MTK BB chip(ex. MT6582), need to enable this function call for correctly power on Touch IC.
#endif
}
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

s32 DrvPlatformLyrTouchDevicePowerOn(void)
{
    DBG("*** %s() ***\n", __func__);

    if(msg2xxx_info->pdata->power_ic)
    {
        msg2xxx_info->pdata->power_ic ;
        mdelay(10);
    }
    if(msg2xxx_info->pdata->reset_gpio)
    {
        gpio_direction_output(msg2xxx_info->pdata->reset_gpio, 1);
        gpio_set_value(msg2xxx_info->pdata->reset_gpio, 0);
        mdelay(20);
        gpio_set_value(msg2xxx_info->pdata->reset_gpio, 1);
        mdelay(40);
    }    
    else
    {
        gpio_direction_output(MS_TS_MSG_IC_GPIO_RST, 1);
        gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 0);
        mdelay(30);
        gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1);
        mdelay(30);
    }
    return 0 ;
}

void DrvPlatformLyrTouchDevicePowerOff(void)
{
    DBG("*** %s() ***\n", __func__);
    
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
//    gpio_direction_output(MS_TS_MSG_IC_GPIO_RST, 0);
    if(msg2xxx_info->pdata->reset_gpio)
        gpio_set_value(msg2xxx_info->pdata->reset_gpio, 0);
    else
        gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 0);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ZERO);  
#ifdef TPD_CLOSE_POWER_IN_SLEEP
    hwPowerDown(TPD_POWER_SOURCE, "TP");
#endif //TPD_CLOSE_POWER_IN_SLEEP
#endif    
}

void DrvPlatformLyrTouchDeviceResetHw(void)
{
    DBG("*** %s() ***\n", __func__);
    
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
    if(msg2xxx_info->pdata->reset_gpio)
    {
        gpio_direction_output(msg2xxx_info->pdata->reset_gpio, 1);
    //    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1); 
        gpio_set_value(msg2xxx_info->pdata->reset_gpio, 0);
        mdelay(100); 
        gpio_set_value(msg2xxx_info->pdata->reset_gpio, 1);
        mdelay(100); 
    }
    else
    {
        gpio_direction_output(MS_TS_MSG_IC_GPIO_RST, 1);
    //    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1); 
        gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 0);
        mdelay(100); 
        gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1);
        mdelay(100); 
    }
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ONE);
    mdelay(10);
    mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ZERO);  
    mdelay(50);
    mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ONE);
    mdelay(50); 
#endif
}

void DrvPlatformLyrDisableFingerTouchReport(void)
{
    DBG("*** %s() ***\n", __func__);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
//    disable_irq(MS_TS_MSG_IC_GPIO_RST);
    disable_irq(msg2xxx_info->irq);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
}

void DrvPlatformLyrEnableFingerTouchReport(void)
{
    DBG("*** %s() ***\n", __func__);

    if(msg2xxx_info->pdata->irq_gpio > 0 )
        enable_irq(msg2xxx_info->irq);
    else
        enable_irq(MS_TS_MSG_IC_GPIO_RST);
}

void DrvPlatformLyrFingerTouchPressed(s32 nX, s32 nY, s32 nPressure, s32 nId)
{
    DBG("*** %s() ***\n", __func__);
    DBG("point touch pressed\n");

    input_report_key(msg2xxx_info->idev, BTN_TOUCH, 1);
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    input_report_abs(msg2xxx_info->idev, ABS_MT_TRACKING_ID, nId);
#endif //CONFIG_ENABLE_CHIP_MSG26XXM
    input_report_abs(msg2xxx_info->idev, ABS_MT_TOUCH_MAJOR, 1);
    input_report_abs(msg2xxx_info->idev, ABS_MT_WIDTH_MAJOR, 1);
    input_report_abs(msg2xxx_info->idev, ABS_MT_POSITION_X, nX);
    input_report_abs(msg2xxx_info->idev, ABS_MT_POSITION_Y, nY);

    input_mt_sync(msg2xxx_info->idev);
}

void DrvPlatformLyrFingerTouchReleased(s32 nX, s32 nY)
{
    DBG("*** %s() ***\n", __func__);
    DBG("point touch released\n");

    input_report_key(msg2xxx_info->idev, BTN_TOUCH, 0);
    input_mt_sync(msg2xxx_info->idev);

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_TP_HAVE_KEY 
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {   
       tpd_button(nX, nY, 0); 
//       tpd_button(0, 0, 0); 
    }            
#endif //CONFIG_TP_HAVE_KEY    

    TPD_EM_PRINT(nX, nY, nX, nY, 0, 0);
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
}

void DrvPlatformLyrGetVerInfo(char* version, char* vendor_name, char* main, char* info,u32* wakeup_support)
{
    sprintf(version,"%2d.%2d",
        msg2xxx_info->major_version,msg2xxx_info->minor_version);
    if(msg2xxx_info->vendor_name)
        sprintf(vendor_name,"%s",msg2xxx_info->vendor_name);
    sprintf(info ,"%d",msg2xxx_info->info_vendor);
    sprintf(main ,"%d",msg2xxx_info->main_vendor);
    *wakeup_support = msg2xxx_info->wakeup_support ;
}

s32 DrvPlatformLyrInputDeviceInitialize(void)
{
    s32 nRetVal = 0;
    u32 i = 0 ;
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION      
    struct ontim_debug ontim_debug_als_prox;  //wanggang
    int err = 0;
#endif
    DBG("*** %s() ***\n", __func__);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
    /* allocate an input device */
    msg2xxx_info->idev = input_allocate_device();
    if (msg2xxx_info->idev == NULL)
    {
        DBG("*** input device allocation failed ***\n");
        return -1;
    }

    msg2xxx_info->idev->name = "msg2xxx";
    msg2xxx_info->idev->phys = "I2C";
    msg2xxx_info->idev->dev.parent = &msg2xxx_info->i2c->dev;
    msg2xxx_info->idev->id.bustype = BUS_I2C;
    
    /* set the supported event type for input device */
    set_bit(EV_ABS, msg2xxx_info->idev->evbit);
    set_bit(EV_SYN, msg2xxx_info->idev->evbit);
    set_bit(EV_KEY, msg2xxx_info->idev->evbit);
    set_bit(BTN_TOUCH, msg2xxx_info->idev->keybit);
    set_bit(INPUT_PROP_DIRECT, msg2xxx_info->idev->propbit);

    if (msg2xxx_info->wakeup_event_num)
    {
        for(i=0;i<msg2xxx_info->wakeup_event_num;i++)
        {
            msg2xxx_info->wakeup_support |= msg2xxx_info->wakeup_event[i].wakeup_event;
            set_bit(msg2xxx_info->wakeup_event[i].key_code, msg2xxx_info->idev->keybit);
        }
    }

#ifdef CONFIG_TP_HAVE_KEY
    {
        u32 i;
        if(msg2xxx_info->panel_parm->virtual_key)    
        {        
            for(i=0; i<msg2xxx_info->panel_parm->virtual_key_num; i++)        
            { 
                if ( msg2xxx_info->panel_parm->virtual_key[i].key_code )            
                {                
                    set_bit(msg2xxx_info->panel_parm->virtual_key[i].key_code, msg2xxx_info->idev->evbit);    
                    printk(KERN_INFO "[kernel][%s %d]:Set key %d\n",
                        __func__,__LINE__,msg2xxx_info->panel_parm->virtual_key[i].key_code); 
                }
                else            
                {               
                    break;            
                }        
            }   
        }
        else
        {
            for (i = 0; i < MAX_KEY_NUM; i ++)
            {
                input_set_capability(msg2xxx_info->idev, EV_KEY, g_TpVirtualKey[i]);
            }
        }
    }
#endif

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    input_set_capability(msg2xxx_info->idev, EV_KEY, KEY_POWER);
    input_set_capability(msg2xxx_info->idev, EV_KEY, KEY_UP);
    input_set_capability(msg2xxx_info->idev, EV_KEY, KEY_DOWN);
    input_set_capability(msg2xxx_info->idev, EV_KEY, KEY_LEFT);
    input_set_capability(msg2xxx_info->idev, EV_KEY, KEY_RIGHT);
    input_set_capability(msg2xxx_info->idev, EV_KEY, KEY_W);
    input_set_capability(msg2xxx_info->idev, EV_KEY, KEY_Z);
    input_set_capability(msg2xxx_info->idev, EV_KEY, KEY_V);
    input_set_capability(msg2xxx_info->idev, EV_KEY, KEY_O);
    input_set_capability(msg2xxx_info->idev, EV_KEY, KEY_M);
    input_set_capability(msg2xxx_info->idev, EV_KEY, KEY_C);
    input_set_capability(msg2xxx_info->idev, EV_KEY, KEY_E);
    input_set_capability(msg2xxx_info->idev, EV_KEY, KEY_S);
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    input_set_abs_params(msg2xxx_info->idev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif //CONFIG_ENABLE_CHIP_MSG26XXM
    input_set_abs_params(msg2xxx_info->idev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    //input_set_abs_params(msg2xxx_info->idev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);

    if(msg2xxx_info->panel_parm->x_max_res && msg2xxx_info->panel_parm->x_max_res)
    {
        input_set_abs_params(msg2xxx_info->idev, ABS_MT_POSITION_X, 
            TOUCH_SCREEN_X_MIN, msg2xxx_info->panel_parm->x_max_res, 0, 0);
        input_set_abs_params(msg2xxx_info->idev, ABS_MT_POSITION_Y, 
            TOUCH_SCREEN_Y_MIN, msg2xxx_info->panel_parm->y_max_res, 0, 0);
    }else
    {
        input_set_abs_params(msg2xxx_info->idev, ABS_MT_POSITION_X, 
            TOUCH_SCREEN_X_MIN, TOUCH_SCREEN_X_MAX, 0, 0);
        input_set_abs_params(msg2xxx_info->idev, ABS_MT_POSITION_Y, 
            TOUCH_SCREEN_Y_MIN, TOUCH_SCREEN_Y_MAX, 0, 0);
    }
    input_set_abs_params(msg2xxx_info->idev, ABS_DISTANCE, 0, 32, 0, 0);
    /* register the input device to input sub-system */
    nRetVal = input_register_device(msg2xxx_info->idev);
    if (nRetVal < 0)
    {
        DBG("*** Unable to register touch input device ***\n");
        return nRetVal ;
    }
#endif

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION     
    ontim_debug_als_prox.dev_name="als_prox"; 
    if ((msg2xxx_info->pdata->prox_enable) && (ontim_dev_debug_file_exist(&ontim_debug_als_prox)<0))       
    {              
        msg2xxx_info->ps_state = 10;    
        msg2xxx_info->ps_onoff = 0;    
        msg2xxx_info->ps_input_dev = NULL;        
        
        msg2xxx_info->ps_input_dev = input_allocate_device();        
        if (!msg2xxx_info->ps_input_dev) 
        {        
            printk("[kernel][%s %d]failed to allocate ps input device\n",__func__,__LINE__);            
            return -1;        
        }        
        set_bit(EV_ABS,msg2xxx_info->ps_input_dev->evbit);        /*proximity*/        
        input_set_abs_params(msg2xxx_info->ps_input_dev, ABS_DISTANCE, 0, 2048, 0, 0);        
        msg2xxx_info->ps_input_dev->name = "als_prox";        
        err = input_register_device(msg2xxx_info->ps_input_dev);        
        if (err) 
        {           
            printk(KERN_ERR "[kernel][%s]: Unable to register input device: %s\n", 
                           __func__,msg2xxx_info->ps_input_dev->name);    
            kfree(msg2xxx_info->ps_input_dev);
            msg2xxx_info->ps_input_dev = NULL ;
            return -1;        
        }        
        printk("[kernel][%s %d]register ps input succsse \n",__func__,__LINE__);
    }     
    else      
    {              
        printk(KERN_INFO "[kernel][%s]:PS sensor exist\n",__func__);     
    }
#endif
    return nRetVal;    
}
void DrvPlatformLyrInputDeviceUnInitialize(void)
{
    if (msg2xxx_info->idev)
    {
        input_unregister_device(msg2xxx_info->idev);
        input_free_device(msg2xxx_info->idev);
    }
    if (msg2xxx_info->ps_input_dev)
    {
        input_unregister_device(msg2xxx_info->ps_input_dev);
        input_free_device(msg2xxx_info->ps_input_dev);
    }
}

s32 DrvPlatformLyrTouchDeviceRequestGPIO(void)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);
    
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) ||defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)

    if(msg2xxx_info->pdata->reset_gpio)
        nRetVal = gpio_request(msg2xxx_info->pdata->reset_gpio, "C_TP_RST");     
    else
        nRetVal = gpio_request(MS_TS_MSG_IC_GPIO_RST, "C_TP_RST");     
    if (nRetVal < 0)
    {
        DBG("*** Failed to request GPIO %d, error %d ***\n", MS_TS_MSG_IC_GPIO_RST, nRetVal);
        return nRetVal ;
    }
    if(msg2xxx_info->pdata->irq_gpio)
        nRetVal = gpio_request(msg2xxx_info->pdata->irq_gpio, "C_TP_INT");
    else
        nRetVal = gpio_request(MS_TS_MSG_IC_GPIO_INT, "C_TP_INT");   
    
    if (nRetVal < 0)
    {
        DBG("*** Failed to request GPIO %d, error %d ***\n", MS_TS_MSG_IC_GPIO_INT, nRetVal);
    }
#endif

    return nRetVal;    
}
void DrvPlatformLyrTouchDeviceFreeGPIO(void)
{
    if (msg2xxx_info->pdata->irq_gpio)
        gpio_free(msg2xxx_info->pdata->irq_gpio);

    if (msg2xxx_info->pdata->reset_gpio)
        gpio_free(msg2xxx_info->pdata->reset_gpio);
}


s32 DrvPlatformLyrRegisterInterruptHandler(void)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);

    if ( 1 )
    {        
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) 
        /* initialize the finger touch work queue */ 
        INIT_WORK(&msg2xxx_info->work, _DrvPlatformLyrFingerTouchDoWork);
		INIT_DELAYED_WORK(&msg2xxx_info->ps_work, _DrvPlatformLyrPSDoWork);

        if(msg2xxx_info->pdata->irq_gpio)
            msg2xxx_info->i2c->irq = gpio_to_irq(msg2xxx_info->pdata->irq_gpio);
        else
            msg2xxx_info->i2c->irq = gpio_to_irq(MS_TS_MSG_IC_GPIO_INT);
        
        msg2xxx_info->irq = msg2xxx_info->i2c->irq;
        /* request an irq and register the isr */
        nRetVal = request_irq(msg2xxx_info->irq, _DrvPlatformLyrFingerTouchInterruptHandler,
                      IRQF_TRIGGER_RISING /* | IRQF_NO_SUSPEND *//* IRQF_TRIGGER_FALLING */,
                      "msg2xxx", NULL);
        if (nRetVal != 0)
        {
            DBG("*** Unable to claim irq %d; error %d ***\n", MS_TS_MSG_IC_GPIO_INT, nRetVal);
            return -1;
        }
        if (msg2xxx_info->wakeup_support)
           device_init_wakeup(&msg2xxx_info->i2c->dev, 1);

#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
        mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_INT, GPIO_CTP_EINT_PIN_M_EINT);
        mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_INT, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(MS_TS_MSG_IC_GPIO_INT, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(MS_TS_MSG_IC_GPIO_INT, GPIO_PULL_UP);

        mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
        mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_RISING, _DrvPlatformLyrFingerTouchInterruptHandler, 1);

        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

#ifdef CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
        /* initialize the finger touch work queue */ 
        INIT_WORK(&msg2xxx_info->work, _DrvPlatformLyrFingerTouchDoWork);
#else
        _gThread = kthread_run(_DrvPlatformLyrFingerTouchHandler, 0, TPD_DEVICE);
        if (IS_ERR(_gThread))
        { 
            nRetVal = PTR_ERR(_gThread);
            DBG("Failed to create kernel thread: %d\n", nRetVal);
            return -1 ;
        }
#endif //CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
#endif
    }
    return nRetVal;    
}    
#ifdef CONFIG_HAS_EARLYSUSPEND
void DrvPlatformLyrTouchDeviceRegisterEarlySuspend(void)
{
    DBG("*** %s() ***\n", __func__);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
    //_gEarlySuspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
    _gEarlySuspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1 ,
    _gEarlySuspend.suspend = MsDrvInterfaceTouchDeviceSuspend;
    _gEarlySuspend.resume = MsDrvInterfaceTouchDeviceResume;
    register_early_suspend(&_gEarlySuspend);
#endif    
}
#endif
/* remove function is triggered when the input device is removed from input sub-system */
s32 DrvPlatformLyrTouchDeviceRemove(struct i2c_client *pClient)
{
    DBG("*** %s() ***\n", __func__);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
//    free_irq(MS_TS_MSG_IC_GPIO_INT, msg2xxx_info->idev);
    free_irq(msg2xxx_info->irq, msg2xxx_info->idev);
    if(msg2xxx_info->pdata->irq_gpio)
        gpio_free(msg2xxx_info->pdata->irq_gpio);
    else
        gpio_free(MS_TS_MSG_IC_GPIO_INT);

    if(msg2xxx_info->pdata->reset_gpio)
        gpio_free(msg2xxx_info->pdata->reset_gpio);
    else
        gpio_free(MS_TS_MSG_IC_GPIO_RST);
    input_unregister_device(msg2xxx_info->idev);

#endif    
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    kset_unregister(g_TouchKSet);
    kobject_put(g_TouchKObj);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG
	wake_lock_destroy(&msg2xxx_info->wlock);

    return 0;
}

void DrvPlatformLyrSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate)
{
    DBG("*** %s() nIicDataRate = %d ***\n", __func__, nIicDataRate);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
    // TODO : Please FAE colleague to confirm with customer device driver engineer for how to set i2c data rate on SPRD platform
    //sprd_i2c_ctl_chg_clk(pClient->adapter->nr, nIicDataRate); 
    //mdelay(100);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
    // TODO : Please FAE colleague to confirm with customer device driver engineer for how to set i2c data rate on QCOM platform
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    pClient->timing = nIicDataRate/1000;
#endif
}

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
void DrvPlatformLyrGestureWakeupMode( char *pBuf )
{
    u32 nWakeupMode ;
   
    if ( pBuf != NULL )
    {
        sscanf(pBuf, "%x", &nWakeupMode);   

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG) == GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG);
        }
        
        if ((nWakeupMode & GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG);
        }
       
        if ((nWakeupMode & GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG);
        }

        printk("[kernel][%s]/-/-/-/-/-/-/wakeup_mode 0x%x/-/-/-/-/-/\n",__func__,g_GestureWakeupMode );
    }
}
#endif

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION

void DrvPlatformLyrTpPsEnable(int nEnable)
{
    DBG("*** %s() nEnable = %d ***\n", __func__, nEnable);

    if (nEnable)
    {
        DrvFwCtrlEnableProximity();
    }
    else
    {
        DrvFwCtrlDisableProximity();
    }
}
#endif


















