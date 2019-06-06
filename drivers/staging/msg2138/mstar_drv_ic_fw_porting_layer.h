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
 * @file    mstar_drv_ic_fw_porting_layer.h
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

#ifndef __MSTAR_DRV_IC_FW_PORTING_LAYER_H__
#define __MSTAR_DRV_IC_FW_PORTING_LAYER_H__

/*--------------------------------------------------------------------------*/
/* INCLUDE FILE                                                             */
/*--------------------------------------------------------------------------*/

#include "mstar_drv_common.h"
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
#include "mstar_drv_mutual_fw_control.h"
#ifdef CONFIG_ENABLE_ITO_MP_TEST
#include "mstar_drv_mutual_mp_test.h"
#endif/* CONFIG_ENABLE_ITO_MP_TEST */
#elif (defined(CONFIG_ENABLE_CHIP_MSG21XXA) \
	|| defined(CONFIG_ENABLE_CHIP_MSG22XX))
#include "mstar_drv_self_fw_control.h"
#ifdef CONFIG_ENABLE_ITO_MP_TEST
#include "mstar_drv_self_mp_test.h"
#endif/* CONFIG_ENABLE_ITO_MP_TEST */
#endif

/*--------------------------------------------------------------------------*/
/* PREPROCESSOR CONSTANT DEFINITION                                         */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/* GLOBAL FUNCTION DECLARATION                                              */
/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern void fwic_open_gesture_wakeup(u16 n_wakeup_mode);
extern void fwic_close_getsture_wakeup(void);
extern u16 g_gesture_wakeup_mode;
extern u8 g_gesture_wakeup_flag;
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern u16 fwic_change_firmwaremode(u16 n_mode);
extern void fwic_getfirmware_info(struct firmware_info_t *p_info);
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
extern u16 fwic_getfirmware_mode(void);
#endif/* CONFIG_ENABLE_CHIP_MSG26XXM */
extern void fwic_restore_firmwaremode_tologdata(void);
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
extern void fwic_checkfirmware_byswid(void);
#endif/* CONFIG_UPDATE_FIRMWARE_BY_SW_ID */

extern u8 fwic_get_chiptype(void);
extern void fwic_getcustomer_firmware_version(u16 *p_major, u16 *p_minor,
	u8 **pp_version);
extern void fwic_getplatform_firmware_version(u8 **pp_version);
extern void fwic_handle_finger_touch(u8 *p_packet, u16 n_len);
extern u32 fwic_register_touch_interrupthandle(void);
extern s32 fwic_update_firmware(u8 sz_fw_data[][1024],
		enum emem_type_e e_mem_type);

#ifdef CONFIG_ENABLE_ITO_MP_TEST
extern void fwic_create_workqueue(void);
extern void fwic_schedule_mptest_work(enum ito_testmode_e e_ito_testmode);
extern void fw_get_mptest_datalog(enum ito_testmode_e e_ito_testmode,
	u8 *p_datalog, u32 *p_len);
extern void fw_get_testfail_channel(enum ito_testmode_e e_ito_testmode,
	u8 *p_fail_channel,
	u32 *p_failchannel_count);
extern s32 fw_get_mptest_result(void);
#endif/* CONFIG_ENABLE_ITO_MP_TEST */

/*=============================================================*/
/* EXTERN VARIABLE DECLARATION */
/*=============================================================*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u16 g_gesture_wakeup_mode;
extern u8 g_gesture_wakeup_flag;
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */


#endif/* __MSTAR_DRV_IC_FW_PORTING_LAYER_H__ */
