#ifndef __TOUCHSCREEN_FW__
#define __TOUCHSCREEN_FW__
/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
/*
* if a TP is used by different module, you want to firmware upgrade by config_id you can
* open ONE_TP_MULTI_MODULE
*/
#define ONE_TP_MULTI_MODULE
#ifdef ONE_TP_MULTI_MODULE
/*
* if a TP is used by different module, you only want to firmware upgrade  by panel name, you can
* open FW_UPGRADE_BY_PANEL_NAME and ONE_TP_MULTI_MODULE
*/
#define FW_UPGRADE_BY_PANEL_NAME
#define SYNA_VENDOR_ID1 0x30
#define SYNA_VENDOR_ID2 0x35
#define SYNA_BL_ID1 0xA7
#define SYNA_BL_ID2 0xA8
#define VENDOR_ID1_NAME "skyworth"
#define VENDOR_ID2_NAME "lead"
#define SYN_VENDOR1_FW_NAME "Syna_firmware_Ofilm_Skyworth.img"
#define SYN_VENDOR2_FW_NAME "Syna_firmware_Ofilm_Lead.img"
#endif
#define WAKEUP_GESTURE false
#define GLOVE_MODE 0
#define SMOOTHING_MODE 0
#define SMART_COVER 0
#define REPORTTING_CONTROL_8BIT

/*****************************************************************************
* TP module firmware name define
*****************************************************************************/
#define SYN_TPK_FW_NAME			""
#define SYN_TURLY_FW_NAME			""
#define SYN_SUCCESS_FW_NAME		""
#define SYN_OFILM_FW_NAME			""
#define SYN_LEAD_FW_NAME			""
#define SYN_WINTEK_FW_NAME		""
#define SYN_LAIBAO_FW_NAME		""
#define SYN_CMI_FW_NAME			""
#define SYN_ECW_FW_NAME			""
#define SYN_GOWORLD_FW_NAME		""
#define SYN_BAOMING_FW_NAME		""
#define SYN_EACHOPTO_FW_NAME		""
#define SYN_MUTTO_FW_NAME		""
#define SYN_JUNDA_FW_NAME		""
#define SYN_BOE_FW_NAME			""
#define SYN_TIANMA_FW_NAME		""
#define SYN_SAMSUNG_FW_NAME		""
#define SYN_DIJING_FW_NAME		""
#define SYN_LCE_FW_NAME			""
#define SYN_GUOXIAN_FW_NAME		""
#define SYN_HELITECH_FW_NAME		""
#define SYN_JDI_FW_NAME			""
#define SYN_HUAXINGDA_FW_NAME	""
#define SYN_TOPTOUCH_FW_NAME		""
#define SYN_GVO_FW_NAME			""
#define SYN_WALLY_PANEL_FW_NAME	""

#define SYN_MOUDLE_NUM_MAX 30


#endif
