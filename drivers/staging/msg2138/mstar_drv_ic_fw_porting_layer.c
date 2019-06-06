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
 * @file    mstar_drv_ic_fw_porting_layer.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

/*=============================================================*/
/* INCLUDE FILE */
/*=============================================================*/

#include "mstar_drv_ic_fw_porting_layer.h"

/*=============================================================*/
/* GLOBAL FUNCTION DEFINITION */
/*=============================================================*/

u8 fwic_get_chiptype(void)
{
/*    LOGTP_FUNC(); */

	return fw_ctrl_get_chiptype();
}

void fwic_getcustomer_firmware_version(u16 *p_major, u16 *p_minor,
	u8 **pp_version)
{
/*    LOGTP_FUNC(); */

	fw_get_customer_firmware_version(p_major, p_minor, pp_version);
}

void fwic_getplatform_firmware_version(u8 **pp_version)
{
/*    LOGTP_FUNC(); */

	fw_get_plattform_firmwareversion(pp_version);
}

s32 fwic_update_firmware(u8 sz_fw_data[][1024], enum emem_type_e e_mem_type)
{
/*    LOGTP_FUNC(); */

	return fw_update_firmware(sz_fw_data, e_mem_type);
}

u32 fwic_register_touch_interrupthandle(void)
{
	LOGTP_FUNC();

return 1;
}

void fwic_handle_finger_touch(u8 *p_packet, u16 n_len)
{
/*    LOGTP_FUNC(); */

	fw_handle_finger_touch();
}

/*
 ------------------------------------------------------------------------------/
/ */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP

void fwic_open_gesture_wakeup(u16 n_wakeup_mode)
{
/*    LOGTP_FUNC(); */

	fw_open_getsture_wakeup(n_wakeup_mode);
}

void fwic_close_getsture_wakeup(void)
{
/*    LOGTP_FUNC(); */

	fw_close_getsture_wakeup();
}

#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

/*
 ------------------------------------------------------------------------------/
/ */

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
u16 fwic_getfirmware_mode(void)
{
/*    LOGTP_FUNC(); */

	return fw_ctrl_getfirmware_mode();
}
#endif/* CONFIG_ENABLE_CHIP_MSG26XXM */

u16 fwic_change_firmwaremode(u16 n_mode)
{
/*    LOGTP_FUNC(); */

	return fw_change_firmware_mode(n_mode);
}

void fwic_getfirmware_info(struct firmware_info_t *p_info)
{
/*    LOGTP_FUNC(); */

	fw_ctrl_getfirmware_info(p_info);
}

void fwic_restore_firmwaremode_tologdata(void)
{
/*    LOGTP_FUNC(); */

	fw_restore_firmwaremode_tologdata();
}

#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

/*
 ------------------------------------------------------------------------------/
/ */

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
void fwic_checkfirmware_byswid(void)
{
/*    LOGTP_FUNC(); */

	fw_checkfirmware_byswid();
}
#endif/* CONFIG_UPDATE_FIRMWARE_BY_SW_ID */

/*
 ------------------------------------------------------------------------------/
/ */

#ifdef CONFIG_ENABLE_ITO_MP_TEST

void fwic_create_workqueue(void)
{
/*    LOGTP_FUNC(); */

	mptest_create_workqueue();
}

void fwic_schedule_mptest_work(enum ito_testmode_e e_ito_testmode)
{
/*    LOGTP_FUNC(); */

	mptest_schedule_work(e_ito_testmode);
}

s32 fw_get_mptest_result(void)
{
/*    LOGTP_FUNC(); */

	return mp_get_test_result();
}

void fw_get_testfail_channel(enum ito_testmode_e e_ito_testmode,
	u8 *p_fail_channel, u32 *p_failchannel_count)
{
/*    LOGTP_FUNC(); */

	return mptest_get_failchannel(e_ito_testmode, p_fail_channel,
	p_failchannel_count);
}

void fw_get_mptest_datalog(enum ito_testmode_e e_ito_testmode, u8 *p_datalog,
				u32 *p_len)
{
/*    LOGTP_FUNC(); */

	return mptest_get_datalog(e_ito_testmode, p_datalog, p_len);
}
#endif/* CONFIG_ENABLE_ITO_MP_TEST */
