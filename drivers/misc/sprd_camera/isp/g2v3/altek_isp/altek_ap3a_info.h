/*
* File: Altek_AP3AInfo.h                                                    *
* Description: Define 3A data structure and API for AP-side 3A library      *
*                                                                           *
* (C)Copyright altek Corporation 2015                                       *
*                                                                           *
* History:                                                                  *
*   2015/09/09; Caedmon Lai; Initial version                                *
*/
#ifndef _ALTEK_AP3AINFO
#define _ALTEK_AP3AINFO

#include "altek_ap3a_info_shared_struct.h"
#include "altek_os_info.h"
#include <video/sprd_isp_altek.h>


#define AP3A_VER_TYPEID (0)
#define AP3A_VER_YEAR   (17)
#define AP3A_VER_MONTH  (01)
#define AP3A_VER_DAY    (19)

#define AP3A_VER_TYPEID_SHIFT	(24)
#define AP3A_VER_YEAR_SHIFT	(16)
#define AP3A_VER_MONTH_SHIFT	(8)
#define AP3A_VER_DAY_SHIFT	(0)

#define AP3A_VER_TYPEID_MASK	(0xFF << AP3A_VER_TYPEID_SHIFT)
#define AP3A_VER_YEAR_MASK	(0xFF << AP3A_VER_YEAR_SHIFT)
#define AP3A_VER_MONTH_MASK	(0xFF << AP3A_VER_MONTH_SHIFT)
#define AP3A_VER_DAY_MASK	(0xFF << AP3A_VER_DAY_SHIFT)


/*
 *@typedef ispdrv_fast_converge_mode
 *@brief Define fast converge  mode (0:AEAF, 1:AE, 2:AF, 3:None)
 *@see V4L2_CID_SHADINGCORRECT_MODE
 */
enum ispdrv_fast_converge_mode {
	V4L2_ISP_FASTCON_OFF,
	V4L2_ISP_FASTCON_ON,
	V4L2_ISP_FASTCON_TOTAL
};

/*
 *\brief Set fast converge mode
 *\param a_uc_sensor [in], Sensor ID
 *\param a_uc_mode   [in], mode, refer to ispdrv_fast_converge_mode
 *\return error code
 */
extern u32 ispdrv_set_fast_converge_mode(u8 a_uc_sensor, u8 a_uc_mode);


/*
*\brief Set download sequence
*\param sensor_id [in], Sensor ID
*\param dld_sequence   [in],
*       download sequence information
*\return error code
*/
extern u32 ispdrv_set_dld_sequence(u8 sensor_id,
				struct dld_sequence *dld_sequence);

/*
*\brief Set 3A configuration
*\param sensor_id [in], Sensor ID
*\param 3a_info   [in], 3A configuration
*\return error code
*/
extern u32 ispdrv_set_3a_cfg(u8 sensor_id,
				struct cfg_3a_info *a3_info);

/*
*\brief Set AE configuration
*\param sensor_id [in], Sensor ID
*\param ae_info   [in], AE configuration
*\return error code
*/
extern u32 ispdrv_set_ae_cfg(u8 sensor_id,
				struct ae_cfg_info *ae_info);

/*
*\brief Set AF configuration
*\param sensor_id [in], Sensor ID
*\param af_info   [in], AF configuration
*\return error code
*/
extern u32 ispdrv_set_af_cfg(u8 sensor_id,
			struct af_cfg_info *af_info);

/*
*\brief Set AWB configuration
*\param sensor_id [in], Sensor ID
*\param awb_info   [in], AWB configuration
*\return error code
*/
extern u32 ispdrv_set_awb_cfg(u8 sensor_id,
			struct awb_cfg_info *awb_info);

/*
*\brief Set YHis configuration
*\param sensor_id [in], Sensor ID
*\param yhis_info   [in], YHis configuration
*\return error code
*/
extern u32 ispdrv_set_yhis_cfg(u8 sensor_id,
			struct yhis_cfg_info *yhis_info);

/*
*\brief Set SubSample configuration
*\param sensor_id [in], Sensor ID
*\param subsample_info [in], SubSample configuration
*\return error code
*/
extern u32 ispdrv_set_subsample_cfg(u8 sensor_id,
				struct subsample_cfg_info *subsample_info);

/*
*\brief Set antiflicker configuration
*\param sensor_id [in], Sensor ID
*\param antiflicker_info [in], AntiFlicker configuration
*\return error code
*/
extern u32 ispdrv_set_antiflicker_cfg(u8 sensor_id,
				struct antiflicker_cfg_info *antiflicker_info);

/*
*\brief Set download sequence of basic preview
*\param sensor_id [in], Sensor ID
*\param preview_baisc_dld_seq_length [in],
*       length of the sequence
*\param preview_baisc_dld_seq [in], the sequence
*\return error code
*/
extern u32 ispdrv_set_dld_sequence_basic_preview(u8 sensor_id,
					u8 preview_baisc_dld_seq_length,
					u8 *preview_baisc_dld_seq);

/*
*\brief Set download sequence of advanced preview
*\param sensor_id [in], Sensor ID
*\param preview_adv_dld_seq_length [in],
*       length of the sequence
*\param preview_adv_dld_seq [in], the sequence
*\return error code
*/
extern u32 ispdrv_set_dld_sequence_adv_preview(u8 sensor_id,
					u8 preview_adv_dld_seq_length,
					u8 *preview_adv_dld_seq);

/*
*\brief Set download sequence of basic fast converge
*\param sensor_id [in], Sensor ID
*\param fast_converge_baisc_dld_seq_length [in],
*       length of the sequence
*\param fast_converge_baisc_dld_seq [in], the sequence
*\return error code
*/
extern u32 ispdrv_set_dld_sequence_basic_fast_converge(u8 sensor_id,
				u8 fast_converge_baisc_dld_seq_length,
				u8 *fast_converge_baisc_dld_seq);




#endif /*ALTEK_3AINFO*/
