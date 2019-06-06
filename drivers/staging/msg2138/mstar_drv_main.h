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
 * @file    mstar_drv_main.h
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

#ifndef __MSTAR_DRV_MAIN_H__
#define __MSTAR_DRV_MAIN_H__

/*--------------------------------------------------------------------------*/
/* INCLUDE FILE                                                             */
/*--------------------------------------------------------------------------*/

#include "mstar_drv_common.h"

/*--------------------------------------------------------------------------*/
/* PREPROCESSOR CONSTANT DEFINITION                                         */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/* PREPROCESSOR MACRO DEFINITION                                            */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/* DATA TYPE DEFINITION                                                     */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/* GLOBAL FUNCTION DECLARATION                                              */
/*--------------------------------------------------------------------------*/

extern ssize_t main_firmware_datashow(struct device *p_device,
	struct device_attribute *p_attr,
	char *p_buf);
extern ssize_t main_firmware_data_store(struct device *p_device,
					struct device_attribute *p_attr,
					const char *p_buf, size_t n_size);
extern ssize_t main_firmware_update_show(struct device *p_device,
	struct device_attribute *p_attr,
	char *p_buf);
extern ssize_t main_firmware_update_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size);
extern ssize_t main_firmware_versionshow(struct device *p_device,
	struct device_attribute *p_attr,
	char *p_buf);
extern ssize_t main_firmware_version_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size);

#ifdef CONFIG_ENABLE_ITO_MP_TEST
extern ssize_t main_firmware_test_show(struct device *p_device,
	struct device_attribute *p_attr,
	char *p_buf);
extern ssize_t main_firmware_test_store(struct device *p_device,
					struct device_attribute *p_attr,
					const char *p_buf, size_t n_size);
#endif/* CONFIG_ENABLE_ITO_MP_TEST */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern ssize_t main_firmware_gesture_wakeup_modeshow(struct device *p_device,
	struct device_attribute
	*p_attr, char *p_buf);
extern ssize_t main_firmware_gesture_wakeupmode_store(struct device *p_device,
	struct device_attribute
	*p_attr, const char *p_buf,
	size_t n_size);
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

extern ssize_t main_firmware_debug_show(struct device *p_device,
					struct device_attribute *p_attr,
					char *p_buf);
extern ssize_t main_firmware_debug_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size);
extern ssize_t main_firmware_platform_version_show(struct device *p_device,
	struct device_attribute
	*p_attr, char *p_buf);
extern ssize_t main_firmware_platform_version_store(struct device *p_device,
	struct device_attribute
	*p_attr, const char *p_buf,
	size_t n_size);
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern ssize_t main_firmware_headershow(struct device *p_device,
	struct device_attribute *p_attr,
	char *p_buf);
extern ssize_t main_firmware_header_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size);
extern ssize_t main_firmware_modeshow(struct device *p_device,
	struct device_attribute *p_attr,
	char *p_buf);
extern ssize_t main_firmwaremode_store(struct device *p_device,
					struct device_attribute *p_attr,
					const char *p_buf, size_t n_size);
/* extern ssize_t DrvMainFirmwarePacketShow(struct device *p_device, struct
 device_attribute *p_attr, char *p_buf); */
/* extern ssize_t DrvMainFirmwarePacketStore(struct device *p_device, struct
 device_attribute *p_attr, const char *p_buf, size_t n_size); */
extern ssize_t main_firmware_sensor_show(struct device *p_device,
	struct device_attribute *p_attr,
	char *p_buf);
extern ssize_t main_firmware_sensor_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size);
extern ssize_t main_kobject_packet_show(struct kobject *p_kobj,
					struct kobj_attribute *p_attr,
					char *p_buf);
extern ssize_t main_kobj_packet_store(struct kobject *p_kobj,
	struct kobj_attribute *p_attr,
	const char *p_buf, size_t n_count);
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */
extern s32 main_device_initialize(void);


/*=============================================================*/
/* EXTERN VARIABLE DECLARATION */
/*=============================================================*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern struct firmware_info_t g_firmware_info;
extern u8 *g_logmode_packet;
extern u16 g_firmware_mode;
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u16 g_gesture_wakeup_mode;
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

extern u8 g_chiptype;



#endif/* __MSTAR_DRV_MAIN_H__ */
