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
 * @file    mstar_drv_platform_porting_layer.h
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

#ifndef __MSTAR_DRV_PLATFORM_PORTING_LAYER_H__
#define __MSTAR_DRV_PLATFORM_PORTING_LAYER_H__

/*--------------------------------------------------------------------------*/
/* INCLUDE FILE                                                             */
/*--------------------------------------------------------------------------*/

#include "mstar_drv_common.h"

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)

#include <linux/gpio.h>

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/consumer.h>
#endif/* CONFIG_ENABLE_REGULATOR_POWER_ON */

#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
#include <linux/regulator/consumer.h>
#endif/* CONFIG_ENABLE_REGULATOR_POWER_ON */

#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/hwmsen_helper.h>
/* #include <linux/hw_module_info.h> */

#include <linux/fs.h>
/* #include <asm/uaccess.h> */
#include <linux/uaccess.h>
#include <linux/namei.h>
#include <linux/vmalloc.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>

#include <cust_eint.h>
#include "tpd.h"
#include "cust_gpio_usage.h"

#endif

/*--------------------------------------------------------------------------*/
/* PREPROCESSOR CONSTANT DEFINITION                                         */
/*--------------------------------------------------------------------------*/

/*
 * Note.
 * Please change the below GPIO pin setting to follow the platform that you are
 using(EX. MediaTek, Spreadtrum, Qualcomm).
 */
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)

/* TODO : Please FAE colleague to confirm with customer device driver engineer
 about the value of RST and INT GPIO setting */
/* #define MS_TS_MSG_IC_GPIO_RST   GPIO_TOUCH_RESET //53 //35 */
/* #define MS_TS_MSG_IC_GPIO_INT   GPIO_TOUCH_IRQ   //52 //37 */

#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)

/* TODO : Please FAE colleague to confirm with customer device driver engineer
 about the value of RST and INT GPIO setting */
#define MS_TS_MSG_IC_GPIO_RST   0
#define MS_TS_MSG_IC_GPIO_INT   1
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
#define MS_TS_MSG_IC_GPIO_RST   (GPIO_CTP_RST_PIN)
#define MS_TS_MSG_IC_GPIO_INT   (GPIO_CTP_EINT_PIN)
#endif

/*--------------------------------------------------------------------------*/
/* GLOBAL FUNCTION DECLARATION                                              */
/*--------------------------------------------------------------------------*/

extern void platform_disable_finger_touchreport(void);
extern void platform_enable_finger_touchreport(void);
extern void platform_finger_pressed(s32 n_x, s32 n_y, s32 n_pressure,
	s32 n_id);
extern void platform_finger_touch_released(s32 n_x, s32 n_y);
extern s32 platform_device_initialize(struct i2c_client *p_client);
extern void platform_set_i2crate(struct i2c_client *p_client,
	u32 n_i2c_rate);
extern void platform_device_poweroff(void);
extern void platform_device_poweron(void);
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
extern void platform_device_regulator_poweron(void);
#endif/* CONFIG_ENABLE_REGULATOR_POWER_ON */
#ifdef CONFIG_HAS_EARLYSUSPEND
extern void platform_device_register_earlysuspend(void);
#endif
extern s32 platform_device_register_interrupt_handler(void);
extern s32 platform_device_remove(struct i2c_client *p_client);
extern s32 platform_device_request_gpio(void);
extern void platform_device_reset_hw(void);


/*=============================================================*/
/* EXTREN VARIABLE DECLARATION */
/*=============================================================*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern struct kset *g_touch_kset;
extern struct kobject *g_touch_kobj;
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
extern struct tpd_device *tpd;
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
extern struct regulator *g_reg_vdd;
#endif



/*=============================================================*/
/* EXTREN VARIABLE DECLARATION */
/*=============================================================*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern struct kset *g_touch_kset;
extern struct kobject *g_touch_kobj;
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
extern struct tpd_device *tpd;
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
extern struct regulator *g_reg_vdd;
#endif


#endif/* __MSTAR_DRV_PLATFORM_PORTING_LAYER_H__ */
