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
 * @file    mstar_drv_self_fw_control.h
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

#ifndef __MSTAR_DRV_SELF_FW_CONTROL_H__
#define __MSTAR_DRV_SELF_FW_CONTROL_H__

/*--------------------------------------------------------------------------*/
/* INCLUDE FILE                                                             */
/*--------------------------------------------------------------------------*/

#include "mstar_drv_common.h"

/*--------------------------------------------------------------------------*/
/* COMPILE OPTION DEFINITION                                                */
/*--------------------------------------------------------------------------*/

/* #define CONFIG_SWAP_X_Y */

/* #define CONFIG_REVERSE_X */
/* #define CONFIG_REVERSE_Y */

/*--------------------------------------------------------------------------*/
/* PREPROCESSOR CONSTANT DEFINITION                                         */
/*--------------------------------------------------------------------------*/

#define DEMO_MODE_PACKET_LENGTH    (8)
#define MAX_TOUCH_NUM           (2)

#define MSG21XXA_BLOCKSIZE (32)/* 32K */
#define MSG21XXA_FIRMWARE_INFO_BLOCK_SIZE (1)/* 1K */
#define MSG21XXA_FIRMWARE_WHOLE_SIZE (MSG21XXA_BLOCKSIZE + \
		MSG21XXA_FIRMWARE_INFO_BLOCK_SIZE)

#define MSG22XX_FM_BLOCKSIZE (48)/* 48K */
#define MSG22XX_FIRMWARE_INFO_BLOCK_SIZE (512)/* 512Byte */

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
#define FIRMWARE_MODE_DEMO_MODE      (0x00)
#define FIRMWARE_MODE_DEBUG_MODE     (0x01)
#define FIRMWARE_MODE_RAW_DATA_MODE  (0x02)
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
#define UPDATE_FIRMWARE_RETRY_COUNT (2)
#endif/* CONFIG_UPDATE_FIRMWARE_BY_SW_ID */

/*--------------------------------------------------------------------------*/
/* DATA TYPE DEFINITION                                                     */
/*--------------------------------------------------------------------------*/

struct touchpoint_t {
	u16 n_x;
	u16 n_y;
};

struct touchinfo_t {
	u8 n_touch_keymode;
	u8 n_touch_keycode;
	u8 n_fingernum;
	struct touchpoint_t t_point[MAX_TOUCH_NUM];
};

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG

struct firmware_info_t {
	u8 n_firmware_mode;
	u8 n_logmode_packet_header;
	u16 n_logmode_packet_len;
	u8 n_iscan_change_firmwaremode;
};

#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
/*
 * Note.
 * The following is sw id enum definition for MSG22XX.
 * 0x0000 and 0xFFFF are not allowed to be defined as SW ID.
 * SW_ID_UNDEFINED is a reserved enum value, do not delete it or modify it.
 * Please modify the SW ID of the below enum value depends on the TP vendor
 that you are using.
 */
enum Msg22xxSwId_e {
	MSG22XX_SW_ID_XXXX = 0x0001,
	MSG22XX_SW_ID_YYYY = 0x0002,
	MSG22XX_SW_ID_UNDEFINED
};

/*
 * Note.
 * The following is sw id enum definition for MSG21XXA.
 * SW_ID_UNDEFINED is a reserved enum value, do not delete it or modify it.
 * Please modify the SW ID of the below enum value depends on the TP vendor
 that you are using.
 */
enum Msg21xxaSwId_e {
	MSG21XXA_SW_ID_XXXX = 0x0C,
	MSG21XXA_SW_ID_YYYY,
	MSG21XXA_SW_ID_UNDEFINED
};
#endif/* CONFIG_UPDATE_FIRMWARE_BY_SW_ID */

/*--------------------------------------------------------------------------*/
/* GLOBAL FUNCTION DECLARATION                                              */
/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern void fw_open_getsture_wakeup(u16 n_mode);
extern void fw_close_getsture_wakeup(void);
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern u16 fw_change_firmware_mode(u16 n_mode);
extern void fw_ctrl_getfirmware_info(struct firmware_info_t *p_info);
extern void fw_restore_firmwaremode_tologdata(void);
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
extern void fw_checkfirmware_byswid(void);
#endif/* CONFIG_UPDATE_FIRMWARE_BY_SW_ID */

extern u8 fw_ctrl_get_chiptype(void);
extern void fw_get_customer_firmware_version(u16 *p_major, u16 *p_minor,
						u8 **pp_version);
extern void fw_get_plattform_firmwareversion(u8 **pp_version);
extern void fw_handle_finger_touch(void);
extern s32 fw_update_firmware(u8 sz_fw_data[][1024],
		enum emem_type_e e_mem_type);

extern struct input_dev *g_input_device;

extern u8 g_fwdata[94][1024];
extern u32 g_fwdata_count;

extern struct mutex g_mutex;

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern struct kobject *g_touch_kobj;
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */


#endif/* __MSTAR_DRV_SELF_FW_CONTROL_H__ */
