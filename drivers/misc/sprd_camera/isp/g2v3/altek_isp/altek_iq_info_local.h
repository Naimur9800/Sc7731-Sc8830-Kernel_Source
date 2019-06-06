/*
* File: altek_iq_info_local.h                                             *
* Description: declaration of iq info local api                           *
*                                                                           *
* (C)Copyright altek Corporation 2015                                       *
*                                                                           *
* History:                                                                  *
*   2016/03/14; Joshua Chao; Initial version                                *
*/
#ifndef _ALTEK_IQ_INFO_LOCAL
#define _ALTEK_IQ_INFO_LOCAL

#include <video/sprd_isp_altek.h>

#include "altek_iq_info.h"
#include "altek_iq_info_func.h"

#pragma pack(push)
#pragma pack(4)
struct iq_addr_buf {
	u32 iva;
	u8 *kva;
};
#pragma pack()

/*
 * Reserved mem's index.
 * tbl_tmp: updated iq info data on ap side.
 * tbl_cur: current table used on fw side.
 * tbl_eff: effective table updated each sof on fw side.
 */
enum iq_info_e_func_tbl {
	IQ_INFO_E_FUNC_TBL_TMP,
	IQ_INFO_E_FUNC_TBL_CUR,
	IQ_INFO_E_FUNC_TBL_EFF,
	IQ_INFO_E_FUNC_TBL_MAX
};

#pragma pack(push)
#pragma pack(4)
struct iq_info_func_info_sns {
	/* each sensor's iq_info_func_info_isp address */
	struct iq_addr_buf info_addr;
	struct iq_addr_buf ap_tbl_addr[IQ_INFO_FUNC_ID_MAX];
};
#pragma pack(pop)

#pragma pack(push)
#pragma pack(4)
struct iq_info_func_tbl {
	u8 dirty;
	u32 size;
	u32 tbl_iva[IQ_INFO_E_FUNC_TBL_MAX];
};
#pragma pack(pop)

/* ---------------------
 * iq_info_func_info_isp:
 * - u32 updated func_id
 * - u8  dirty flag
 * - u32 table size (struct size)
 * - u32 working table's iva addr
 * - u32 current table's iva addr
 * - u32 effective table's iva addr
 * ---------------------
 */
#pragma pack(push)
#pragma pack(4)
struct iq_info_func_info_isp {
	u32 func_id; /* record which id is updated */
	u32 tbl_iva; /* address iq_info_func_tbl[fid_cnt] */
};
#pragma pack(pop)

void iq_info_set_rgb2yuv_iva(u8 sensor_id, u32 rgb2yuv_iva);
u32 iq_info_get_rgb2yuv_iva(u8 sensor_id);
u32 *share_buff_iq_info_get_rgb2yuv_iva(u8 sensor_id);

void iq_info_set_rgb2yuv(u8 sensor_id, struct ispdrv_rgb2yuv_martix *rgb2yuv);
struct ispdrv_rgb2yuv_martix *iq_info_get_rgb2yuv(u8 sensor_id);
void **share_buff_iq_info_get_rgb2yuv(u8 sensor_id);


void iq_info_set_tonemap_iva(u8 sensor_id, u32 tonemap_iva);
u32 iq_info_get_tonemap_iva(u8 sensor_id);
u32 *share_buff_iq_info_get_tonemap_iva(u8 sensor_id);

void iq_info_set_tonemap(u8 sensor_id, struct ispdrv_tonemap_curve *tonemap);
struct ispdrv_tonemap_curve *iq_info_get_tonemap(u8 sensor_id);
void **share_buff_iq_info_get_tonemap(u8 sensor_id);

void iq_info_set_brightness_setting_iva(u8 sensor_id,
	u32 brightness_setting_iva);
u32 iq_info_get_brightness_setting_iva(u8 sensor_id);
u32 *share_buff_iq_info_get_brightness_setting_iva(u8 sensor_id);

void iq_info_set_brightness_setting(u8 sensor_id,
	struct ispdrv_brightness_gain_setting *brightness_setting);
struct ispdrv_brightness_gain_setting *iq_info_get_brightness_setting(
	u8 sensor_id);
void **share_buff_iq_info_brightness_setting(u8 sensor_id);


void iq_info_set_ccm_addr_iva(u8 sensor_id, u32 ccm_iva);
u32 iq_info_get_ccm_addr_iva(u8 sensor_id);
u32 *share_buff_iq_info_get_ccm_addr_iva(u8 sensor_id);

void iq_info_set_ccm_addr_kva(u8 sensor_id, struct iq_ccm_info *ccm_addr_kva);
struct iq_ccm_info *iq_info_get_ccm_addr_kva(u8 sensor_id);
void **share_buff_iq_info_get_ccm_addr_kva(u8 sensor_id);

void iq_info_set_otp_addr_iva(u8 sensor_id, u32 otp_iva);
u32 iq_info_get_otp_addr_iva(u8 sensor_id);
u32 *share_buff_iq_info_get_otp_addr_iva(u8 sensor_id);

void iq_info_set_otp_addr_kva(u8 sensor_id, struct iq_otp_info *otp_addr_kva);
struct iq_otp_info *iq_info_get_otp_addr_kva(u8 sensor_id);
void **share_buff_iq_info_get_otp_addr_kva(u8 sensor_id);

void iq_info_set_isp_d_gain_addr_iva(u8 sensor_id, u32 d_gain_iva);
u32 iq_info_get_isp_d_gain_addr_iva(u8 sensor_id);
u32 *share_buff_iq_info_get_isp_d_gain_addr_iva(u8 sensor_id);

void iq_info_set_isp_d_gain_addr_kva(u8 sensor_id,
	struct isp_d_gain_info *isp_d_gain_addr_kva);
struct isp_d_gain_info *iq_info_get_isp_d_gain_addr_kva(u8 sensor_id);
void **share_buff_iq_info_get_isp_d_gain_addr_kva(u8 sensor_id);

#if 0
void iq_info_set_raw_info_addr_iva(u8 sensor_id, u32 raw_iva);
u32 iq_info_get_raw_info_addr_iva(u8 sensor_id);
void iq_info_set_raw_info_addr_kva(u8 sensor_id, u8 *raw_info_kva);
u8 *iq_info_get_raw_info_addr_kva(u8 sensor_id);
#endif
void iq_info_set_proc_still_arg_iva(u8 sensor_id, u32 arg_iva);
u32 iq_info_get_proc_still_arg_iva(u8 sensor_id);
u32 *share_buff_iq_info_get_proc_still_arg_iva(u8 sensor_id);

void iq_info_set_proc_still_arg_kva(u8 sensor_id, u8 *pref);
u8 *iq_info_get_proc_still_arg_kva(u8 sensor_id);
void **share_buff_iq_info_get_proc_still_arg_kva(u8 sensor_id);

u32 *share_buff_iq_function_addr(void);

u32 iq_info_init_func_info(u32 isp_base_iva, u8 *isp_base_kva);

void iq_info_init(void *pri_data);
void iq_info_deinit(void *pri_data);
#endif
