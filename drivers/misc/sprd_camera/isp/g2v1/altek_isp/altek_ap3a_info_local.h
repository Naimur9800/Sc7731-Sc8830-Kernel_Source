/*
* File: altek_ap3a_info_local.h                                             *
* Description: declaration of ap3a_info local api                           *
*                                                                           *
* (C)Copyright altek Corporation 2015                                       *
*                                                                           *
* History:                                                                  *
*   2016/03/12; Caedmon Lai; Initial version                                *
*/
#ifndef _ALTEK_AP3A_INFO_LOCAL
#define _ALTEK_AP3A_INFO_LOCAL

#ifdef _SPRD_PLATFORM_G2V1
#include <video/sprd_isp_altek.h>
#endif

void set_shm_cfg_3a_info_ptr(u8 sensor_id, struct cfg_3a_info *ptr);

struct cfg_3a_info *get_shm_cfg_3a_info_ptr(u8 sensor_id);
void **share_buff_get_shm_cfg_3a_info_ptr_addr(u8 sensor_id);

void set_shm_dld_seq_info_ptr(u8 sensor_id, struct dld_sequence *ptr);

struct dld_sequence *get_shm_dld_seq_info_ptr(u8 sensor_id);
void **share_buff_get_shm_dld_seq_info_ptr_addr(u8 sensor_id);

#endif
