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
 * @file	mstar_drv_platform_interface.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
/*  INCLUDE FILE */
/*=============================================================*/

#include <linux/irq.h>
#include<linux/module.h>
#include "mstar_drv_platform_interface.h"
#include "mstar_drv_main.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_utility_adaption.h"

#ifdef CONFIG_ENABLE_HOTKNOT
#include "mstar_drv_hotknot.h"
#endif /* CONFIG_ENABLE_HOTKNOT */

/*=============================================================*/
/*  EXTERN VARIABLE DECLARATION */
/*=============================================================*/

extern int _gIrq;
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u32 g_GestureWakeupMode[2];
extern u8 g_GestureWakeupFlag;

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
extern u8 g_GestureDebugFlag;
extern u8 g_GestureDebugMode;
#endif /* CONFIG_ENABLE_GESTURE_DEBUG_MODE */

#endif /* CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
extern u8 g_EnableTpProximity;
#endif /* CONFIG_ENABLE_PROXIMITY_DETECTION */

#ifdef CONFIG_ENABLE_GLOVE_MODE
extern u8 g_IsEnableGloveMode;
#endif /* CONFIG_ENABLE_GLOVE_MODE */

extern u8 g_IsUpdateFirmware;

extern struct input_dev *g_InputDevice;
extern struct i2c_client *g_I2cClient;

#ifdef CONFIG_ENABLE_HOTKNOT
extern u8 g_HotKnotState;
extern u32 SLAVE_I2C_ID_DWI2C;
#endif /* CONFIG_ENABLE_HOTKNOT */

#ifdef CONFIG_ENABLE_CHARGER_DETECTION
extern u8 g_ForceUpdate;
#endif /* CONFIG_ENABLE_CHARGER_DETECTION */

#ifdef CONFIG_ENABLE_ESD_PROTECTION
extern int g_IsEnableEsdCheck;
extern struct delayed_work g_EsdCheckWork;
extern struct workqueue_struct *g_EsdCheckWorkqueue;
#endif /* CONFIG_ENABLE_ESD_PROTECTION */

extern u8 IS_FIRMWARE_DATA_LOG_ENABLED;


/*=============================================================*/
/*  GLOBAL VARIABLE DEFINITION */
/*=============================================================*/

extern struct input_dev *g_InputDevice;
extern u8 *_gFwVersion; /*  customer firmware version */
/*=============================================================*/
/*  LOCAL VARIABLE DEFINITION */
/*=============================================================*/

#ifdef CONFIG_ENABLE_HOTKNOT
static u8 _gAMStartCmd[4] = {HOTKNOT_SEND, ADAPTIVEMOD_BEGIN, 0, 0};
#endif /* CONFIG_ENABLE_HOTKNOT */

/*=============================================================*/
/*  GLOBAL FUNCTION DEFINITION */
/*=============================================================*/

#ifdef CONFIG_ENABLE_NOTIFIER_FB
int MsDrvInterfaceTouchDeviceFbNotifierCallback(struct notifier_block *pSelf, unsigned long nEvent, void *pData)
{
	struct fb_event *pEventData = pData;
	int *pBlank;

	if (pEventData && pEventData->data && nEvent == FB_EVENT_BLANK) {
		pBlank = pEventData->data;

		if (*pBlank == FB_BLANK_UNBLANK) {
			DBG(&g_I2cClient->dev, "*** %s() TP Resume ***\n", __func__);

			if (g_IsUpdateFirmware != 0) {
				/*  Check whether update frimware is finished */
				DBG(&g_I2cClient->dev, "Not allow to power on/off touch ic while update firmware.\n");
				return 0;
			}

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
			if (g_EnableTpProximity == 1) {
				DBG(&g_I2cClient->dev, "g_EnableTpProximity = %d\n", g_EnableTpProximity);
				return 0;
			}
#endif /* CONFIG_ENABLE_PROXIMITY_DETECTION */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
			if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /* CONFIG_ENABLE_HOTKNOT */
			{
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
				if (g_GestureDebugMode == 1) {
					DrvIcFwLyrCloseGestureDebugMode();
					disable_irq_wake(_gIrq);
				}
#endif /* CONFIG_ENABLE_GESTURE_DEBUG_MODE */

				if (g_GestureWakeupFlag == 1) {
					DrvIcFwLyrCloseGestureWakeup();
					disable_irq_wake(_gIrq);
				} else {
					DrvPlatformLyrEnableFingerTouchReport();
				}
			}
#ifdef CONFIG_ENABLE_HOTKNOT
			else {
				/*  Enable touch in hotknot transfer mode */
				DrvPlatformLyrEnableFingerTouchReport();
			}
#endif /* CONFIG_ENABLE_HOTKNOT */
#endif /* CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_HOTKNOT
			if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /* CONFIG_ENABLE_HOTKNOT */
			{
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
				DrvPlatformLyrTouchDeviceRegulatorPowerOn(true);
#endif /* CONFIG_ENABLE_REGULATOR_POWER_ON */
				DrvPlatformLyrTouchDevicePowerOn();
			}

#ifdef CONFIG_ENABLE_CHARGER_DETECTION
			{
				u8 szChargerStatus[20] = {0};

				DrvCommonReadFile(POWER_SUPPLY_BATTERY_STATUS_PATCH, szChargerStatus, 20);

				DBG(&g_I2cClient->dev, "*** Battery Status : %s ***\n", szChargerStatus);

				g_ForceUpdate = 1; /*  Set flag to force update charger status */

				if (strstr(szChargerStatus, "Charging") != NULL || strstr(szChargerStatus, "Full") != NULL || strstr(szChargerStatus, "Fully charged") != NULL) {
					/*  Charging */
					DrvFwCtrlChargerDetection(1); /*  charger plug-in */
				} else {
					/*  Not charging */
					DrvFwCtrlChargerDetection(0); /*  charger plug-out */
				}

				g_ForceUpdate = 0; /*  Clear flag after force update charger status */
			}
#endif /* CONFIG_ENABLE_CHARGER_DETECTION */

#ifdef CONFIG_ENABLE_GLOVE_MODE
			if (g_IsEnableGloveMode == 1) {
				DrvIcFwLyrOpenGloveMode();
			}
#endif /* CONFIG_ENABLE_GLOVE_MODE */

			if (IS_FIRMWARE_DATA_LOG_ENABLED) {
				DrvIcFwLyrRestoreFirmwareModeToLogDataMode(); /*  Mark this function call for avoiding device driver may spend longer time to resume from suspend state. */
			} /* IS_FIRMWARE_DATA_LOG_ENABLED */

#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
			DrvPlatformLyrEnableFingerTouchReport();
#endif /* CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_ESD_PROTECTION
			g_IsEnableEsdCheck = 1;
			queue_delayed_work(g_EsdCheckWorkqueue, &g_EsdCheckWork, ESD_PROTECT_CHECK_PERIOD);
#endif /* CONFIG_ENABLE_ESD_PROTECTION */
		} else if (*pBlank == FB_BLANK_POWERDOWN) {
			DBG(&g_I2cClient->dev, "*** %s() TP Suspend ***\n", __func__);

#ifdef CONFIG_ENABLE_ESD_PROTECTION
			g_IsEnableEsdCheck = 0;
			cancel_delayed_work_sync(&g_EsdCheckWork);
#endif /* CONFIG_ENABLE_ESD_PROTECTION */

			if (g_IsUpdateFirmware != 0) {
				/*  Check whether update frimware is finished */
				DBG(&g_I2cClient->dev, "Not allow to power on/off touch ic while update firmware.\n");
				return 0;
			}

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
			if (g_EnableTpProximity == 1) {
				DBG(&g_I2cClient->dev, "g_EnableTpProximity = %d\n", g_EnableTpProximity);
				return 0;
			}
#endif /* CONFIG_ENABLE_PROXIMITY_DETECTION */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
			if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /* CONFIG_ENABLE_HOTKNOT */
			{
				if (g_GestureWakeupMode[0] != 0x00000000 || g_GestureWakeupMode[1] != 0x00000000) {
					DrvIcFwLyrOpenGestureWakeup(&g_GestureWakeupMode[0]);
					enable_irq_wake(_gIrq);
					return 0;
				}
			}
#endif /* CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_HOTKNOT
			if (g_HotKnotState == HOTKNOT_BEFORE_TRANS_STATE || g_HotKnotState == HOTKNOT_TRANS_STATE || g_HotKnotState == HOTKNOT_AFTER_TRANS_STATE) {
				IicWriteData(SLAVE_I2C_ID_DWI2C, &_gAMStartCmd[0], 4);
			}
#endif /* CONFIG_ENABLE_HOTKNOT */

			DrvPlatformLyrFingerTouchReleased(0, 0, 0); /*  Send touch end for clearing point touch */
			input_sync(g_InputDevice);

			DrvPlatformLyrDisableFingerTouchReport();

#ifdef CONFIG_ENABLE_HOTKNOT
			if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /* CONFIG_ENABLE_HOTKNOT */
			{
				DrvPlatformLyrTouchDevicePowerOff();
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
				DrvPlatformLyrTouchDeviceRegulatorPowerOn(false);
#endif /* CONFIG_ENABLE_REGULATOR_POWER_ON */
			}
		}
	}

	return 0;
}

#else

#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
void MsDrvInterfaceTouchDeviceSuspend(struct device *pDevice)
#else
#if defined(CONFIG_HAS_EARLYSUSPEND)
void MsDrvInterfaceTouchDeviceSuspend(struct early_suspend *pSuspend)
#else
int MsDrvInterfaceTouchDeviceSuspend(struct device *pSuspend)
#endif
#endif /* CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD */
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);
	/* printk("*** %s() ***\n", __func__); */

#ifdef CONFIG_ENABLE_ESD_PROTECTION
	g_IsEnableEsdCheck = 0;
	cancel_delayed_work_sync(&g_EsdCheckWork);
#endif /* CONFIG_ENABLE_ESD_PROTECTION */

	if (g_IsUpdateFirmware != 0) {
		/*  Check whether update frimware is finished */
		DBG(&g_I2cClient->dev, "Not allow to power on/off touch ic while update firmware.\n");
#ifdef CONFIG_PM_SLEEP
		return 0;
#else
		return;
#endif
	}

#ifdef TP_HAVE_PROX
	if (is_incall == 1) {
		printk("prox suspend is_incall\n");
		/* enable_irq_wake(_gIrq); */
#ifdef CONFIG_PM_SLEEP
		return 0;
#else
		return;
#endif
	}
#endif

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
	if (g_EnableTpProximity == 1) {
		DBG(&g_I2cClient->dev, "g_EnableTpProximity = %d\n", g_EnableTpProximity);
		return;
	}
#endif /* CONFIG_ENABLE_PROXIMITY_DETECTION */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /* CONFIG_ENABLE_HOTKNOT */
	{
		if (g_GestureWakeupMode[0] != 0x00000000 || g_GestureWakeupMode[1] != 0x00000000) {
			DrvIcFwLyrOpenGestureWakeup(&g_GestureWakeupMode[0]);
			enable_irq_wake(_gIrq);
			irq_set_irq_type(_gIrq, IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND | IRQF_ONESHOT);
#ifdef CONFIG_PM_SLEEP
			return 0;
#else
			return;
#endif
		}
	}
#endif /* CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState == HOTKNOT_BEFORE_TRANS_STATE || g_HotKnotState == HOTKNOT_TRANS_STATE || g_HotKnotState == HOTKNOT_AFTER_TRANS_STATE) {
		IicWriteData(SLAVE_I2C_ID_DWI2C, &_gAMStartCmd[0], 4);
	}
#endif /* CONFIG_ENABLE_HOTKNOT */

	DrvPlatformLyrFingerTouchReleased(0, 0, 0); /*  Send touch end for clearing point touch */
	input_sync(g_InputDevice);

	DrvPlatformLyrDisableFingerTouchReport();

#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /* CONFIG_ENABLE_HOTKNOT */
	{
		DrvPlatformLyrTouchDevicePowerOff();
#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
		DrvPlatformLyrTouchDeviceRegulatorPowerOn(false);
#endif /* CONFIG_ENABLE_REGULATOR_POWER_ON */
#endif /* CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD */
	}
#ifdef CONFIG_PM_SLEEP
	return 0;
#else
	return;
#endif
}

#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
void MsDrvInterfaceTouchDeviceResume(struct device *pDevice)
#else
#if defined(CONFIG_HAS_EARLYSUSPEND)
void MsDrvInterfaceTouchDeviceResume(struct early_suspend *pSuspend)
#else
int MsDrvInterfaceTouchDeviceResume(struct device *pSuspend)
#endif
#endif /* CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD */
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);
	/* printk("*** %s() ***\n", __func__); */

	if (g_IsUpdateFirmware != 0) {
		/*  Check whether update frimware is finished */
		DBG(&g_I2cClient->dev, "Not allow to power on/off touch ic while update firmware.\n");
#ifdef CONFIG_PM_SLEEP
		return 0;
#else
		return;
#endif
	}

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
	if (g_EnableTpProximity == 1) {
		DBG(&g_I2cClient->dev, "g_EnableTpProximity = %d\n", g_EnableTpProximity);
#ifdef CONFIG_PM_SLEEP
		return 0;
#else
		return;
#endif
	}
#endif /* CONFIG_ENABLE_PROXIMITY_DETECTION */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /* CONFIG_ENABLE_HOTKNOT */
	{
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
		if (g_GestureDebugMode == 1) {
			DrvIcFwLyrCloseGestureDebugMode();
		}
#endif /* CONFIG_ENABLE_GESTURE_DEBUG_MODE */

		if (g_GestureWakeupFlag == 1) {
			DrvIcFwLyrCloseGestureWakeup();
			disable_irq_wake(_gIrq);
			irq_set_irq_type(_gIrq, IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
		} else {
			DrvPlatformLyrEnableFingerTouchReport();
		}
	}
#ifdef CONFIG_ENABLE_HOTKNOT
	else {
		/*  Enable touch in hotknot transfer mode */
		DrvPlatformLyrEnableFingerTouchReport();
	}
#endif /* CONFIG_ENABLE_HOTKNOT */
#endif /* CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /* CONFIG_ENABLE_HOTKNOT */
	{
#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
		DrvPlatformLyrTouchDeviceRegulatorPowerOn(true);
#endif /* CONFIG_ENABLE_REGULATOR_POWER_ON */
#endif /* CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD */
		DrvPlatformLyrTouchDevicePowerOn();
	}

#ifdef CONFIG_ENABLE_CHARGER_DETECTION
	{
		u8 szChargerStatus[20] = {0};

		DrvCommonReadFile(POWER_SUPPLY_BATTERY_STATUS_PATCH, szChargerStatus, 20);

		DBG(&g_I2cClient->dev, "*** Battery Status : %s ***\n", szChargerStatus);

		g_ForceUpdate = 1; /*  Set flag to force update charger status */

		if (strstr(szChargerStatus, "Charging") != NULL || strstr(szChargerStatus, "Full") != NULL || strstr(szChargerStatus, "Fully charged") != NULL) {
			/*  Charging */
			DrvFwCtrlChargerDetection(1); /*  charger plug-in */
		} else {
			/*  Not charging */
			DrvFwCtrlChargerDetection(0); /*  charger plug-out */
		}

		g_ForceUpdate = 0; /*  Clear flag after force update charger status */
	}
#endif /* CONFIG_ENABLE_CHARGER_DETECTION */

#ifdef CONFIG_ENABLE_GLOVE_MODE
	if (g_IsEnableGloveMode == 1) {
		DrvIcFwLyrOpenGloveMode();
	}
#endif /* CONFIG_ENABLE_GLOVE_MODE */

	if (IS_FIRMWARE_DATA_LOG_ENABLED) {
		DrvIcFwLyrRestoreFirmwareModeToLogDataMode(); /*  Mark this function call for avoiding device driver may spend longer time to resume from suspend state. */
	} /* IS_FIRMWARE_DATA_LOG_ENABLED */

#ifdef TP_HAVE_PROX
	if (is_incall == 1) {
		printk("prox resume is_incall\n");
		mstar_prox_ctl(1);
		/* disable_irq_wake(_gIrq); */
	}
#endif

#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
	DrvPlatformLyrEnableFingerTouchReport();
#endif /* CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_ESD_PROTECTION
	g_IsEnableEsdCheck = 1;
	queue_delayed_work(g_EsdCheckWorkqueue, &g_EsdCheckWork, ESD_PROTECT_CHECK_PERIOD);
#endif /* CONFIG_ENABLE_ESD_PROTECTION */
#ifdef CONFIG_PM_SLEEP
	return 0;
#else
	return;
#endif
}
#ifdef CONFIG_PM_SLEEP
int MsDrvInterfaceTouchDeviceAdfNotifierCallback(struct notifier_block *pSelf, unsigned long nEvent, void *pData)
{
	struct adf_notifier_event *pEventData = pData;
	int *pBlank;
	struct device *pDev;

	if (pEventData && pEventData->data && nEvent == ADF_EVENT_BLANK) {
		pBlank = pEventData->data;

		if (*pBlank == DRM_MODE_DPMS_ON) {
			MsDrvInterfaceTouchDeviceResume(pDev);
		} else if (*pBlank == DRM_MODE_DPMS_OFF) {
			MsDrvInterfaceTouchDeviceSuspend(pDev);
		}
	}
	return 0;
}
#endif
#endif /* CONFIG_ENABLE_NOTIFIER_FB */

#if 0/* def TOUCH_VIRTUAL_KEYS //my */
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct msg2xxx_ts_platform_data *pdata = g_I2cClient->dev.platform_data;
	return sprintf(buf, "%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d\n"
		, __stringify(EV_KEY), __stringify(KEY_MENU), pdata->virtualkeys[0], pdata->virtualkeys[1], pdata->virtualkeys[2], pdata->virtualkeys[3]
		, __stringify(EV_KEY), __stringify(KEY_HOMEPAGE), pdata->virtualkeys[4], pdata->virtualkeys[5], pdata->virtualkeys[6], pdata->virtualkeys[7]
		, __stringify(EV_KEY), __stringify(KEY_BACK), pdata->virtualkeys[8], pdata->virtualkeys[9], pdata->virtualkeys[10], pdata->virtualkeys[11]);
}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.msg2138_ts", /* msg2xxx */
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
		pr_err("failed to create board_properties\n");
}
#endif

#ifdef CONFIG_SPRD_PH_INFO
extern char SPRD_TPInfo[];
#endif
/* probe function is used for matching and initializing input device */
s32 /*__devinit*/ MsDrvInterfaceTouchDeviceProbe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId)
{
	s32 nRetVal = 0;
	u16 nMajor = 0, nMinor = 0;
	u8 chipid = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	DrvPlatformLyrVariableInitialize();

	nRetVal = DrvPlatformLyrTouchDeviceRequestGPIO(pClient);
	if (nRetVal) {
		printk("*** MStar touch request GPIO error***\n");
		return nRetVal;
	}

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
	DrvPlatformLyrTouchDeviceRegulatorPowerOn(true);
#endif /* CONFIG_ENABLE_REGULATOR_POWER_ON */

	DrvPlatformLyrTouchDevicePowerOn();
	chipid = DrvIcFwLyrGetChipType();
	if (!chipid) {
		nRetVal = -EINVAL;
		printk("*** MStar touch no detect ***\n");
		goto err_out;
	}

	nRetVal = DrvMainTouchDeviceInitialize();
	if (nRetVal == -ENODEV) {
		DrvPlatformLyrTouchDeviceRemove(pClient);
		/* return nRetVal; */
		goto err_out;
	}

	DrvPlatformLyrInputDeviceInitialize(pClient);
#if 0 /* def TOUCH_VIRTUAL_KEYS //my */
/* 	pixcir_ts_virtual_keys_init(); */
#endif

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	DmaAlloc(); /*  DmaAlloc() shall be called after DrvPlatformLyrInputDeviceInitialize() */
#endif /* CONFIG_ENABLE_DMA_IIC */
#endif /* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler();

	if ((0x85 == chipid) || (0xbf == chipid))
		DrvFwCtrlGetCustomerFirmwareVersionByDbBus(EMEM_MAIN, &nMajor, &nMinor, &_gFwVersion);
	else
		DrvIcFwLyrGetCustomerFirmwareVersion(&nMajor, &nMinor, &_gFwVersion);
	DrvPlatformLyrTouchDeviceRegisterEarlySuspend();

	DrvPlatformLyrTouchDeviceResetHw();
#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
	DrvIcFwLyrCheckFirmwareUpdateBySwId();
#endif /* CONFIG_UPDATE_FIRMWARE_BY_SW_ID */

#ifdef CONFIG_ENABLE_ESD_PROTECTION
	INIT_DELAYED_WORK(&g_EsdCheckWork, DrvPlatformLyrEsdCheck);
	g_EsdCheckWorkqueue = create_workqueue("esd_check");
	queue_delayed_work(g_EsdCheckWorkqueue, &g_EsdCheckWork, ESD_PROTECT_CHECK_PERIOD);
#endif /* CONFIG_ENABLE_ESD_PROTECTION */

#ifdef CONFIG_SPRD_PH_INFO
	memset((void *)SPRD_TPInfo, 0, 100);
	memcpy(SPRD_TPInfo, "msg2238", 100); /* MSG_TP_IC_NAME */
#endif
	/* printk(&g_I2cClient->dev, "*** MStar touch driver registered ***\n"); */

	return nRetVal;
err_out:
	gpio_free(MS_TS_MSG_IC_GPIO_INT);
	gpio_free(MS_TS_MSG_IC_GPIO_RST);
	return nRetVal;
}

/* remove function is triggered when the input device is removed from input sub-system */
s32 /*__devexit*/ MsDrvInterfaceTouchDeviceRemove(struct i2c_client *pClient)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	return DrvPlatformLyrTouchDeviceRemove(pClient);
}

void MsDrvInterfaceTouchDeviceSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	DrvPlatformLyrSetIicDataRate(pClient, nIicDataRate);
}
