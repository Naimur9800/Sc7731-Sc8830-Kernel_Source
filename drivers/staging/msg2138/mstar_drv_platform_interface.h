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
 * @file    mstar_drv_platform_interface.h
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

#ifndef __MSTAR_DRV_PLATFORM_INTERFACE_H__
#define __MSTAR_DRV_PLATFORM_INTERFACE_H__

/*--------------------------------------------------------------------------*/
/* INCLUDE FILE                                                             */
/*--------------------------------------------------------------------------*/

#include "mstar_drv_common.h"

/*--------------------------------------------------------------------------*/
/* GLOBAL FUNCTION DECLARATION                                              */
/*--------------------------------------------------------------------------*/

extern s32  interface_device_probe(struct i2c_client  *p_client, /*__devinit*/
				const struct i2c_device_id *p_device_id);
extern s32  ms_interface_device_remove(struct i2c_client/*__devexit*/
	*p_client);
#ifdef CONFIG_HAS_EARLYSUSPEND
extern void interface_device_resume(struct early_suspend *p_suspend);
extern void interface_device_suspend(struct early_suspend *p_suspend);
#endif
extern void interface_device_set_i2c_rate(struct i2c_client *p_client,
	u32 n_i2c_rate);

/*=============================================================*/
/* EXTERN VARIABLE DECLARATION */
/*=============================================================*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u16 g_gesture_wakeup_mode;
extern u8 g_gesture_wakeup_flag;
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

/*=============================================================*/
/* GLOBAL VARIABLE DEFINITION */
/*=============================================================*/

extern struct input_dev *g_input_device;


/*=============================================================*/
/* EXTERN VARIABLE DECLARATION */
/*=============================================================*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u16 g_gesture_wakeup_mode;
extern u8 g_gesture_wakeup_flag;
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

/*=============================================================*/
/* GLOBAL VARIABLE DEFINITION */
/*=============================================================*/

extern struct input_dev *g_input_device;

#endif/* __MSTAR_DRV_PLATFORM_INTERFACE_H__ */
