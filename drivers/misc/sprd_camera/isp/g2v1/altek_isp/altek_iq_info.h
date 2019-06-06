/*
 * File: altek_iq_info.h                                                     *
 * Description: definition of iq info structure and api                      *
 *                                                                           *
 * (C)Copyright altek Corporation 2016                                       *
 *                                                                           *
 * History:                                                                  *
 *   2016/03/012; Caedmon Lai; Initial version                               *
 */
#ifndef _ALTEK_IQ_INFO
#define _ALTEK_IQ_INFO

#include "altek_sc_info.h"

/* tone table size for each color */
#define TONESIZE 259

/* Brightness array size */
#define BRIGHTNESS_ARRAY_SIZE 7

/*
*@typedef ispdrv_tonemap_curve
*@brief User defined tone map curve
*       (259 elements in each curve)
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct ispdrv_tonemap_curve {
	u8 r[TONESIZE];
	u8 g[TONESIZE];
	u8 b[TONESIZE];
};
#pragma pack()

/*
*@struct ISPDRV_RGB2YUV_MARTIX
*@brief Define RGB to YUV martix elements
*       Y    [c00, c01, c02] R   o00
*       Cb = [c10, c11, c12] G + o01
*       Cr   [c20, c21, c22] B   o02
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct ispdrv_rgb2yuv_martix {
	u16 c00;
	u16 c01;
	u16 c02;
	u16 c10;
	u16 c11;
	u16 c12;
	u16 c20;
	u16 c21;
	u16 c22;
	u16 o00;
	u16 o01;
	u16 o02;
};
#pragma pack(pop)

/*
 *@typedef ispdrv_shading_mode
 *@brief Define shading mode.(0:Off, 1:On)
 *@see V4L2_CID_SHADINGCORRECT_MODE
 */
enum ispdrv_shading_mode {
	V4L2_ISP_SHADING_OFF,
	V4L2_ISP_SHADING_ON,
	V4L2_ISP_SHADING_TOTAL
};


/*
 *@typedef ispdrv_rgb2yuv_format
 *@brief Define RGB to YHV format.(0:BT601, 1:BT709, 2:User define)
 *@see V4L2_CID_RGB2YUV_FORMAT_MODE
 */
enum ispdrv_rgb2yuv_format {
	V4L2_ISP_RGB2YUV_BT601,
	V4L2_ISP_RGB2YUV_BT709,
	V4L2_ISP_RGB2YUV_USER,
	V4L2_ISP_RGB2YUV_TOTAL
};

/*
 *@typedef ispdrv_tonemap_info
 *@brief Define tone map info.(0:Curve blue, 1:Curve green, 2:Curve red)
 *@see V4L2_CID_SET_TONEMAP_CURVE
 */
enum ispdrv_tonemap_info {
	V4L2_ISP_TONEMAP_CURVE_BLUE,
	V4L2_ISP_TONEMAP_CURVE_GREEN,
	V4L2_ISP_TONEMAP_CURVE_RED,
	V4L2_ISP_TONEMAP_CURVE_TOTAL
};


/*
 *@typedef ispdrv_tonemap_mode
 *@brief Define tone map mode.(0:Curve fix, 1:Curve user define)
 *@see V4L2_CID_TONEMAP_MODE
 */
enum ispdrv_tonemap_mode {
	V4L2_ISP_TONEMAP_CURVE_FIX,
	V4L2_ISP_TONEMAP_CURVE_USER_DEFINE,
	V4L2_ISP_TONEMAP_CURVE_MODE_TOTAL
};

/*
 * *@typedef ispdrv_sharpness_mode
 * *@brief Define sharpness mode
 * *@see V4L2_CID_SHARPNESS_MODE
 */
enum ispdrv_sharpness_mode {
	V4L2_ISP_SHARPNESS_LV0, /* -2 */
	V4L2_ISP_SHARPNESS_LV1, /* -1.5 */
	V4L2_ISP_SHARPNESS_LV2, /* -1 */
	V4L2_ISP_SHARPNESS_LV3, /* -0.5 */
	V4L2_ISP_SHARPNESS_LV4, /* 0 */
	V4L2_ISP_SHARPNESS_LV5, /* 0.5 */
	V4L2_ISP_SHARPNESS_LV6, /* 1 */
	V4L2_ISP_SHARPNESS_LV7, /* 1.5 */
	V4L2_ISP_SHARPNESS_LV8, /* 2 */
	V4L2_ISP_SHARPNESS_TOTAL
	/* V4L2_ISP_SHARPNESS_NORMAL, */
};

/*
 *@typedef ispdrv_saturation_mode
 *@brief Define saturation mode
 *@see V4L2_CID_SATURATION_MODE
*/
enum ispdrv_saturation_mode {
	V4L2_ISP_SATURATION_LV0, /* -2 */
	V4L2_ISP_SATURATION_LV1, /* -1.5 */
	V4L2_ISP_SATURATION_LV2, /* -1 */
	V4L2_ISP_SATURATION_LV3, /* -0.5 */
	V4L2_ISP_SATURATION_LV4, /* 0 */
	V4L2_ISP_SATURATION_LV5, /* 0.5 */
	V4L2_ISP_SATURATION_LV6, /* 1 */
	V4L2_ISP_SATURATION_LV7, /* 1.5 */
	V4L2_ISP_SATURATION_LV8, /* 2 */
	/* V4L2_ISP_SATURATION_NORMAL, */
	/* V4L2_ISP_SATURATION_LOW, */
	/* V4L2_ISP_SATURATION_HIGH, */
	V4L2_ISP_SATURATION_TOTAL
};

/*
 *@typedef ispdrv_contrast_mode
 *@brief Define contrast mode
 *@see V4L2_CID_CONTRAST_MODE
*/
enum ispdrv_contrast_mode {
	V4L2_ISP_CONTRAST_LV0, /* -2 */
	V4L2_ISP_CONTRAST_LV1, /* -1.5 */
	V4L2_ISP_CONTRAST_LV2, /* -1 */
	V4L2_ISP_CONTRAST_LV3, /* -0.5 */
	V4L2_ISP_CONTRAST_LV4, /* 0 */
	V4L2_ISP_CONTRAST_LV5, /* 0.5 */
	V4L2_ISP_CONTRAST_LV6, /* 1 */
	V4L2_ISP_CONTRAST_LV7, /* 1.5 */
	V4L2_ISP_CONTRAST_LV8, /* 2 */

	/* V4L2_ISP_CONTRAST_NORMAL, */
	/* V4L2_ISP_CONTRAST_LOW, */
	/* V4L2_ISP_CONTRAST_HIGH, */
	V4L2_ISP_CONTRAST_TOTAL
};

/*
 *@typedef ispdrv_special_effect_mode
 *@brief Define special effect mode.
 *@(0:Off, 1:Negative, 2:Solarize, 3:Grayscale,
 *@ 4: Sepia, 5:Red points, 6:Green points,
 *@ 7: Blue points 8:Red yellow points,
 *@ 9:WARMVINTAGE 10:COLDVINTAGE 11:WASHOUT,
 *@ 12:POSTERISE, 13:User defined)
 *@see V4L2_CID_SPECIAL_EFFECT_MODE
*/
enum ispdrv_special_effect_mode {
	V4L2_ISP_SPECIAL_EFFECT_OFF,
	V4L2_ISP_SPECIAL_EFFECT_NEGATIVE,
	V4L2_ISP_SPECIAL_EFFECT_SOLARIZE,
	V4L2_ISP_SPECIAL_EFFECT_GRAYSCALE,
	V4L2_ISP_SPECIAL_EFFECT_SEPIA,
	V4L2_ISP_SPECIAL_EFFECT_REDPOINTS,
	V4L2_ISP_SPECIAL_EFFECT_GREENPOINTS,
	V4L2_ISP_SPECIAL_EFFECT_BLUEPOINTS,
	V4L2_ISP_SPECIAL_EFFECT_REDYELLOWPOINTS,
	V4L2_ISP_SPECIAL_EFFECT_WARMVINTAGE,
	V4L2_ISP_SPECIAL_EFFECT_COLDVINTAGE,
	V4L2_ISP_SPECIAL_EFFECT_WASHOUT,
	V4L2_ISP_SPECIAL_EFFECT_POSTERISE,
	V4L2_ISP_SPECIAL_EFFECT_USERDEFINED,
	V4L2_ISP_SPECIAL_EFFECT_TOTAL
};



/*
*@struct ispdrv_brightness_gain_setting
*@brief User defined birghtness gain setting
*/
#pragma pack(4)
struct ispdrv_brightness_gain_setting {
	u16 gain[BRIGHTNESS_ARRAY_SIZE];
};
#pragma pack()


/*
*@typedef awb_gain_info
*@brief AWB gain information
*/
#pragma pack(push)
#pragma pack(4)
struct awb_gain_info {
	/* 256 base, updated WB for ISP processing */
	u16 detected_awb_r;
	/* 256 base, updated WB for ISP processing */
	u16 detected_awb_gr;
	/* 256 base, updated WB for ISP processing */
	u16 detected_awb_gb;
	/* 256 base, updated WB for ISP processing */
	u16 detected_awb_b;
};
#pragma pack(pop)

/*
*\brief Set AWB gain value
*\param sensor_id [in], Sensor ID
*\param awb_gain_info [in], AWB gain value
*\return error code
*/
extern u32 ispdrv_set_awb_gain(u8 sensor_id,
		struct awb_gain_info *awb_gain_info);

/*
*\brief Set Balanced AWB gain value
*\param sensor_id [in], Sensor ID
*\param awb_gain_info [in], AWB gain value
*\return error code
*/
extern u32 ispdrv_set_balanced_awb_gain(u8 sensor_id,
		struct awb_gain_info *awb_gain_info);

/*
*\brief Set detected ISO value
*\param sensor_id [in], Sensor ID
*\param detected_iso [in], detected ISO value
*\return error code
*/
extern u32 ispdrv_set_iso_speed(u8 sensor_id,
				u16 detected_iso);

/*
 *\brief Set shading correct mode
 *\param sensor_id [in], Sensor ID
 *\param mode [in], mode (ispdrv_shading_mode)
 *\return error code
 */
extern u32 ispdrv_set_shading_correct_mode(u8 sensor_id, u8 mode);


/*
 *\brief Set RGB2YUY format mode
 *\param sensor_id [in], Sensor ID
 *\param mode [in], RGB2YUV mode
 *       (BT601/BT709/User define)
 *\return error code
 */
extern u32 ispdrv_set_rgb2yuv_format_mode(u8 sensor_id, u8 mode);


/*
 *\brief Set RGB2YUV user define matrix
 *\param sensor_id [in], Sensor ID
 *\param a_pt_rgb2yuv_matrix [in], RGB2YUV martix array
 *\return EINVAL.(Incorrect sensor ID)
 */
extern u32 ispdrv_set_rgb2yuv_user_defined_matrix(u8 sensor_id,
		struct ispdrv_rgb2yuv_martix *rgb2yuv_matrix);

/*
 *\brief Set Tone map Curve (gamma tone)
 *\param sensor_id [in], Sensor ID
 *\param a_pt_tone_addr  [in], address of the tone map
 *\return EINVAL.(Incorrect sensor ID)
 */
extern u32 ispdrv_set_tonemap_curve(u8 sensor_id,
		struct ispdrv_tonemap_curve *tone_addr);


/*
 *\brief Set Tone map mode
 *\param sensor_id [in], Sensor ID
 *\param mode [in], mode (ispdrv_tonemap_mode)
 *\return error code
 */
extern u32 ispdrv_set_tonemap_mode(u8 sensor_id, u8 mode);

/*
 *\brief Set Sharpness mode
 *\param sensor_id [in], Sensor ID
 *\param mode   [in], mode
 *\return error code
 */
extern u32 ispdrv_set_sharpness_mode(u8 sensor_id, u8 mode);

/*
 *\brief Set Saturation Mode
 *\param sensor_id [in], Sensor ID
 *\param mode   [in], mode, refer to ispdrv_saturation_mode
 *\return error code
 */
extern u32 ispdrv_set_saturation_mode(u8 sensor_id, u8 mode);

/*
 *\brief Set Contrast Mode
 *\param sensor_id [in], Sensor ID
 *\param mode   [in], mode, refer to ispdrv_contrast_mode
 *\return error code
 */
extern u32 ispdrv_set_contrast_mode(u8 sensor_id, u8 mode);

/*
 *\brief Set Special Effect Mode
 *\param sensor_id [in], Sensor ID
 *\param mode [in], mode, refer to ispdrv_special_effect_mode
 *\return error code
 */
extern u32 ispdrv_set_special_effect_mode(u8 sensor_id, u8 mode);

/*
*\brief Set brightness gain setting
*	AP could set 7 brightness gains to ISP
*       The value range is 0 ~ 200
*	Value 150 means gain 1.50
*\param a_uc_sensor [in], Sensor ID
*\param a_pt_brghtness_gain_addr [in],
*       address of the brightness gain array
*\return EINVAL.(Incorrect sensor ID)
*/
extern u32 ispdrv_set_brightness_gain_setting(u8 sensor_id,
	struct ispdrv_brightness_gain_setting *brghtness_gain_addr);

/*
*\brief Set Brightness Mode.
*\Function ispdrv_set_brightness_gain_setting
*\should be called before call this function.
*\param a_uc_sensor [in], Sensor ID
*\param a_uc_mode   [in], mode 0 ~ 6,
*	gain was defined at btightness gain setting
*\return error code
*/
extern u32 ispdrv_set_brightness_mode(u8 sensor_id, u8 mode);

/*
*\brief Set overlap pixel number
*\param a_uw_overlap_pixel_num [In],
*      the number of overlap pixels
*\return error code
*/
extern u32 ispdrv_set_overlap_pixel(u16 overlap_pixel_num);

extern u32 ispdrv_set_color_temperature(u8 sensor_id, u32 color_tmp);

/*total length = 9, an array for hardware, each has 32 bits*/
extern u32 ispdrv_set_ccm(u8 sensor_id, struct iq_ccm_info *ccm);

extern u32 ispdrv_set_iq_otp_info(u8 sensor_id,
		struct iq_otp_info *p_uc_otp_info);

extern u32 ispdrv_set_valid_ad_gain(u8 sensor_id, u32 a_uw_gain);
extern u32 ispdrv_set_valid_exp_time(u8 sensor_id, u32 a_uw_exptime);

extern u32 ispdrv_set_iq_info(u8 sensor_id, u32 func_id,
		void *in_data, u32 length);
extern u32 ispdrv_set_isp_d_gain(u8 sensor_id,
		struct isp_d_gain_info *isp_d_gain);
extern u32 ispdrv_set_y_offset(u8 sensor_id, u32 y_offset);
extern void ispdrv_set_iq_buffer_mem_info(u32 base_iva, u64 buf_size);
extern u32 ispdrv_set_iq_func_info(u32 fid_cnt, u32 *fid_size);
extern u32 ispdrv_set_iq_param_index(u8 sensor_id,
	u8 change_flag,
	u8 *iq_param_index);
#endif
