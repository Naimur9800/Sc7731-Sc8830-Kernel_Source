/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _ISP_DRV_KERNEL_H_
#define _ISP_DRV_KERNEL_H_
#include <linux/time.h>

#define ISP_FW_BUF_SIZE                         0x500000
#define ISP_SHARE_BUFF_SIZE                     0x100000
#define ISP_WORKING_BUF_SIZE                    0xE00000
/* statistics buffer size 200KB * 5 * 2, 2 IRP bin and 2 Shading bin */
#define ISP_STATISTICS_BUF_SIZE                 (230 * 1024)
#define ISP_AF_STATISTICS_BUF_SIZE		(20 * 1024)
#define ISP_IRP_BIN_BUF_SIZE                    0x100000
#define ISP_SHADING_BIN_BUF_SIZE                0x4b000
#define ISP_CBC_BIN_BUF_SIZE                    0x80000
/*modified macro to simplify buffer offset codes*/
#define ISP_WORKING_BUF_BASE \
	(ISP_FW_BUF_SIZE + ISP_SHARE_BUFF_SIZE)
#define ISP_STATISTICS_BUF_BASE \
	(ISP_WORKING_BUF_BASE + ISP_WORKING_BUF_SIZE)
#define ISP_AF_STATISTICS_BUF_BASE \
	(ISP_STATISTICS_BUF_BASE + (ISP_STATISTICS_BUF_SIZE * 5 * 3))
#define ISP_IRP_BIN_BASE \
	(ISP_AF_STATISTICS_BUF_BASE + (ISP_AF_STATISTICS_BUF_SIZE * 5 * 3))
#define ISP_SHADING_BIN_BASE \
	(ISP_IRP_BIN_BASE + ISP_IRP_BIN_BUF_SIZE * 3)
#define ISP_CBC_BIN_BASE \
	(ISP_SHADING_BIN_BASE + ISP_SHADING_BIN_BUF_SIZE * 3)
#define ISP_BUF_SIZE \
	(ISP_CBC_BIN_BASE + ISP_CBC_BIN_BUF_SIZE * 3)

#define IMG_BUF_NUM_MAX                         4
#define MAXSEQLEN                               20
#define BRIGHTNESS_SIZE                         7
#define IQ_CCM_INFO                             9
#define ISP_LSC_MAX_LENGTH                      1800
#define ISP_RAWBUF_NUM				5
/* Define AE IQ info reserved size follow the user space */
#define ISP_AE_IQ_INFO_SIZE            (16)

#define IRP_TUNING_DEBUG_IDX               (3)
#define IRP_TUNING_DEBUG_INFO              (6)
#define IRP_TUNING_DEBUG_BYPASS            (5)
#define IRP_TUNING_DEBUG_CCM               (9)
#define IRP_TUNING_DEBUG_PARA_ADDR         (56)
#define IRP_TUNING_DEBUG_RESERVED          (5)
#define IRP_TUNING_DEBUG_BVTH              (4)

enum isp_img_output_id {
	ISP_IMG_PREVIEW = 0,
	ISP_IMG_VIDEO,
	ISP_IMG_STILL_CAPTURE,
	ISP_IMG_STATISTICS,
	ISP_IMG_AF_STATISTICS,
	ISP_IMG_RAW,
	ISP_OUTPUT_IMG_TOTAL,
};

enum isp_rtn {
	ISP_IMG_TX_DONE = 0x20,
	ISP_IMG_NO_MEM,
	ISP_IMG_TX_ERR,
	ISP_IMG_SYS_BUSY,
	ISP_IMG_TIMEOUT,
	ISP_IMG_TX_STOP,
};

enum isp_irq_type {
	ISP_IRQ_3A_SOF = 0x20,
	ISP_IRQ_STATIS,
	ISP_IRQ_AF_STATIS,
	ISP_IRQ_IMG,
	ISP_IRQ_CFG_BUF,
	ISP_IRQ_ALTEK_RAW,
};

enum isp_output_img_format {
	ISP_OUT_IMG_RAW10 = 0,
	ISP_OUT_IMG_NV12,
	ISP_OUT_IMG_YUY2,
	ISP_OUT_IMG_FORMAT_TOTAL,
};

enum {
	ISP_SHARPNESS_LV0 = 0, /* -2 */
	ISP_SHARPNESS_LV1, /* -1.5 */
	ISP_SHARPNESS_LV2, /* -1 */
	ISP_SHARPNESS_LV3, /* -0.5 */
	ISP_SHARPNESS_LV4, /* 0 */
	ISP_SHARPNESS_LV5, /* 0.5 */
	ISP_SHARPNESS_LV6, /* 1 */
	ISP_SHARPNESS_LV7, /* 1.5 */
	ISP_SHARPNESS_LV8, /* 2 */
	ISP_SHARPNESS_TOTAL
};

enum {
	ISP_SATURATION_LV0, /* -2 */
	ISP_SATURATION_LV1, /* -1.5 */
	ISP_SATURATION_LV2, /* -1 */
	ISP_SATURATION_LV3, /* -0.5 */
	ISP_SATURATION_LV4, /* 0 */
	ISP_SATURATION_LV5, /* 0.5 */
	ISP_SATURATION_LV6, /* 1 */
	ISP_SATURATION_LV7, /* 1.5 */
	ISP_SATURATION_LV8, /* 2 */
	/* ISP_SATURATION_NORMAL, */
	/* ISP_SATURATION_LOW, */
	/* ISP_SATURATION_HIGH, */
	ISP_SATURATION_TOTAL
};

enum {
	ISP_SPECIAL_EFFECT_OFF,
	ISP_SPECIAL_EFFECT_NEGATIVE,
	ISP_SPECIAL_EFFECT_SOLARIZE,
	ISP_SPECIAL_EFFECT_GRAYSCALE,
	ISP_SPECIAL_EFFECT_SEPIA,
	ISP_SPECIAL_EFFECT_REDPOINTS,
	ISP_SPECIAL_EFFECT_GREENPOINTS,
	ISP_SPECIAL_EFFECT_BLUEPOINTS,
	ISP_SPECIAL_EFFECT_REDYELLOWPOINTS,
	ISP_SPECIAL_EFFECT_WARMVINTAGE,
	ISP_SPECIAL_EFFECT_COLDVINTAGE,
	ISP_SPECIAL_EFFECT_WASHOUT,
	ISP_SPECIAL_EFFECT_POSTERISE,
	ISP_SPECIAL_EFFECT_USERDEFINED,
	ISP_SPECIAL_EFFECT_TOTAL
};

enum {
	ISP_CONTRAST_LV0, /* -2 */
	ISP_CONTRAST_LV1, /* -1.5 */
	ISP_CONTRAST_LV2, /* -1 */
	ISP_CONTRAST_LV3, /* -0.5 */
	ISP_CONTRAST_LV4, /* 0 */
	ISP_CONTRAST_LV5, /* 0.5 */
	ISP_CONTRAST_LV6, /* 1 */
	ISP_CONTRAST_LV7, /* 1.5 */
	ISP_CONTRAST_LV8, /* 2 */
	/* ISP_CONTRAST_NORMAL, */
	/* ISP_CONTRAST_LOW, */
	/* ISP_CONTRAST_HIGH, */
	ISP_CONTRAST_TOTAL
};

enum isp_cfg_param {
	ISP_CFG_SET_ISO_SPEED,
	ISP_CFG_SET_AWB_GAIN,
	ISP_CFG_SET_DLD_SEQUENCE,
	ISP_CFG_SET_3A_CFG,
	ISP_CFG_SET_AE_CFG,
	ISP_CFG_SET_AF_CFG,
	ISP_CFG_SET_AWB_CFG,
	ISP_CFG_SET_YHIS_CFG,
	ISP_CFG_SET_SUB_SAMP_CFG,
	ISP_CFG_SET_AFL_CFG,
	ISP_CFG_SET_DLD_SEQ_BASIC_PREV,
	ISP_CFG_SET_DLD_SEQ_ADV_PREV,
	ISP_CFG_SET_DLD_SEQ_BASIC_FAST_CONV,
	ISP_CFG_SET_SCENARIO_INFO,
	ISP_CFG_SET_SHARPNESS,
	ISP_CFG_SET_SATURATION,
	ISP_CFG_SET_CONTRAST,
	ISP_CFG_SET_SPECIAL_EFFECT,
	ISP_CFG_SET_AWB_GAIN_BALANCED,
	ISP_CFG_SET_BRIGHTNESS_GAIN,
	ISP_CFG_SET_BRIGHTNESS_MODE,
	ISP_CFG_SET_COLOR_TEMPERATURE,
	ISP_CFG_SET_CCM,
	ISP_CFG_SET_VALID_ADGAIN,
	ISP_CFG_SET_VALID_EXP_TIME,
	ISP_CFG_SET_IQ_OTP_INFO,
	ISP_CFG_SET_SOF_PARAM,
	ISP_CFG_SET_D_GAIN,
	ISP_CFG_SET_Y_OFFSET,
	ISP_CFG_SET_TOTAL
};

enum isp_dev_capability {
	ISP_GET_FW_BUF_SIZE,
	ISP_GET_STATIS_BUF_SIZE,
	ISP_GET_DRAM_BUF_SIZE,
	ISP_GET_HIGH_ISO_BUF_SIZE,
	ISP_GET_CONTINUE_SIZE,
	ISP_GET_SINGLE_SIZE,
};

enum isp_dev_read_id {
	ISP_IMG_GET_STATISTICS_FRAME = 0,
	ISP_IMG_GET_FRAME,
};

enum isp_dev_write_id {
	ISP_IMG_STOP_ISP = 0,
};

enum scinfo_sensor_mod_type {
	SENSOR_MODULE_TYPE_IMX214 = 0,
};

enum scinfo_color_order {
	COLOR_ORDER_RG = 0,
	COLOR_ORDER_GR,
	COLOR_ORDER_GB,
	COLOR_ORDER_BG
};

enum {
	ISP_OUTPUT_RAW10,
	ISP_OUTPUT_NV12,
	ISP_OUTPUT_YUY2,/* default */
	ISP_OUTPUT_ALTEK_RAW10,
	ISP_OUTPUT_TYPE_TOTAL
};

enum isp_tuning_index {
	ISP_INDEX_PREVIEW_FULL = 0,
	ISP_INDEX_PREVIEW_BINING,
	ISP_INDEX_STILL_FULL,
	ISP_INDEX_STILL_BINING,
	ISP_INDEX_VIDEO_SLOW_MOTION,
	ISP_INDEX_MAX
};

/*3A related parameters, provided by Altek_AP3AInfo.h*/
typedef uint16_t u16;
typedef int16_t s16;
typedef uint32_t u32;
typedef uint8_t u8;
typedef int8_t s8;

struct isp_awb_gain_info {
	uint16_t r;
	uint16_t g;
	uint16_t b;
};

struct isp_brightness_gain {
	uint16_t uw_gain[BRIGHTNESS_SIZE];
};

struct isp_iq_ccm_info {
	int32_t ad_ccm[IQ_CCM_INFO];
};

struct isp_iq_otp_info {
	uint8_t cali_status;
	uint16_t iso;
	uint16_t r_gain;
	uint16_t g_gain;
	uint16_t b_gain;
	uint8_t lsc[ISP_LSC_MAX_LENGTH];
	uint16_t lsc_length;
};

struct isp_func_ae_info {
	uint32_t update_iso;
	uint32_t update_mean;
	int32_t update_bv;
	int32_t update_bgbv;
	uint8_t  valid_flg;  /* 0:AE Not Process yet,1:AE already process */
	uint8_t  reserved[ISP_AE_IQ_INFO_SIZE];  /*reserved 16 bytes*/
};

struct isp_sof_cfg_info {
	struct isp_awb_gain_info awb_gain;
	struct isp_awb_gain_info awb_b_gain;
	struct isp_func_ae_info iq_ae_info;
	uint32_t color_temp;
	uint32_t iso_val;
	uint32_t is_update;
};

/*
 *brief mid mode
 */
enum mid_mode {
	/* 5x5 mask */
	MF_51_MODE,
	/* 3x3 mask */
	MF_31_MODE,
	MF_DISABLE
};

/*
 *@typedef statistics_dld_region
 *@brief Statistics download region
 */
#pragma pack(push)
#pragma pack(4)
struct statistics_dld_region {
	/* Used to control ROI of width,
	* if set 50 means use half image to get HW3A stats data, suggest 100
	*/
	u16 border_ratio_x;
	/* Used to control ROI of height,
	*  if set 50 means use half image to get HW3A stats data, suggest 100
	*/
	u16 border_ratio_y;
	/* Block number of horizontal direction of RAW iamge */
	u16 blk_num_x;
	/* Block number of vertical direction of RAW image */
	u16 blk_num_y;
	/* Used to control ROI shift position ratio of width,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_x;
	/* Used to control ROI shift position ratio of height,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_y;
};
#pragma pack(pop)

/*
 *@typedef ae_cfg_info
 *@brief AE configuration information
 */
#pragma pack(push)
#pragma pack(4)
struct ae_cfg_info {
	u16 token_id;
	struct statistics_dld_region ae_region;
};
#pragma pack(pop)


/*
 *@structawbhis
 *@brief AWB histogram Parameters setting
 */
#pragma pack(push)
#pragma pack(4)
struct awbhis {
	/* Enable HW3A range histogram, Suggest set = TRUE */
	/* bool  enable;*/
	u16 enable;
	/* Cr Range Start, Suggest set =  -46 */
	s8 cr_start;
	/* Cr Range End, Suggest set = 110 */
	s8 cr_end;
	/* Offset Range Up, Suggest set = 10 */
	s8 offset_up;
	/* Offset Range Down, Suggest set = -90 */
	s8 offset_down;
	/* Purple compensation, Suggest set = 0 */
	s8 cr_purple;
	/* Purple compensation, Suggest set = 2 */
	u8 offset_purple;
	/* Grass compensation--offset, Suggest set = -22 */
	s8 grass_offset;
	/* Grass compensation--cr left, Suggest set = -30 */
	s8 grass_start;
	/* Grass compensation--cr right, Suggest set = 25 */
	s8 grass_end;
	/* Grass compensation, Suggest set = 4 */
	u8 damp_grass;
	/* Weighting around bbr, Suggest set = -2 */
	s8 offset_bbr_w_start;
	/* Weighting around bbr, Suggest set = 2 */
	s8 offset_bbr_w_end;
	/* Weighting around bbr, Suggest set = 2 */
	u8 yfac_w;
	/* HisInterp=(ucOffsetPurPle - cOffsetUp)*1024/(cCrPurple-cCrStart),
	*  Note: (cCrPurple-cCrStart) !=0, Suggest set = -178
	*/
	u32 his_interp;
};
#pragma pack(pop)

/*
 *@typedef awb_cfg_info
 *@brief AWB configuration information
 */
#pragma pack(push)
#pragma pack(4)
struct awb_cfg_info {
	u16 token_id;
	struct statistics_dld_region awb_region;
	/* Y factor weighting table,
	*{0,0,0,4,7,10,12,14,15,15,15,15,14,13,10,5};
	*/
	u8 y_factor[16];
	/* BBR curve table,
	*{22,20,18,16,15,13,11,10,8,8,6,5,3,1,-1,-3,-4,-5,-6,
	*-7,-8,-9,-10,-11,-12,-13,-14,-15,-16,-18,-18,-18,-18};
	*/
	s8 bbr_factor[33];
	/* Calib R Gain w/ Altek format */
	u16  r_gain;
	/* Calib G Gain w/ Altek format, Suggest set = 0 */
	u16  g_gain;
	/* Calib B Gain w/ Altek format */
	u16  b_gain;
	/* Data-shift for fitting 8bit, Suggest set = 100 */
	u8   cr_shift;
	/* Data-shift for fitting 8bit, Suggest set = 100 */
	u8   offset_shift;
	/* HW parameter, suggest 0 */
	u8   quantize;
	/* HW parameter, suggest 7 */
	u8   damp;
	/* HW parameter, suggest 5 */
	u8   sum_shift;
	struct awbhis t_his;
	/* Calib R Gain w/ format 7bit. */
	u16  r_linear_gain;
	/* Calib B Gain w/ format 7bit. */
	u16  b_linear_gain;
};
#pragma pack(pop)

/*
 *@typedef af_statistics_dld_region
 *@brief AF Statistics download region
 */
#pragma pack(push)
#pragma pack(4)
struct af_statistics_dld_region {
       /* Used to control ROI of width,
	*if set 50 means use half image to get HW3A stats data
	*/
	u16 size_ratio_x;
       /* Used to control ROI of height,
	* if set 50 means use half image to get HW3A stats data
	*/
	u16 size_ratio_y;
	/* Block number of horizontal direction of ROI */
	u16 blk_num_x;
	/* Block number of vertical direction of ROI */
	u16 blk_num_y;
	/* Used to control ROI shift position ratio of width,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_x;
	/* Used to control ROI shift position ratio of height,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_y;
};
#pragma pack(pop)

/*
 *@typedef af_cfg_info
 *@brief AF configuration information
 */
#pragma pack(push)
#pragma pack(4)
struct af_cfg_info {
	u16 token_id;
	/* AF ROI configuration */
	struct af_statistics_dld_region af_region;
	/* Enable flag for tone mapping, set false would disable auwAFLUT */
	/* bool enable_af_lut;*/
	u16 enable_af_lut;
	/* Tone mapping for common 3A, no longer used (default bypassing) */
	u16 lut[259];
	/* Tone mapping for AF only */
	u16 af_lut[259];
	/* Weight values when calculate the frequency. */
	u8 weight[6];
	/* Bit for shift */
	u16 sh;
	/* Configure 2 modes. Mode 1: First 82 blocks refer to Th,
	*  Tv from thelookup array. Mode 2: Every 4 blocks reference one element
	*/
	u8 th_mode;
	/* Locate index in an 82 elements array and
	*  reference one of 4 stages Th,
	*  Tv
	*/
	u8 index[82];
	/* Thresholds for calculate the horizontal contrast. */
	u16 th[4];
	/* Thresholds for calculate the vertical contrast. */
	u16 tv[4];
	/* Input data offset */
	u32 af_offset;
	/* Turn pseudo Y on or off */
	/*bool af_py_enable;*/
	u16 af_py_enable;
	/* Turn low pass filter on or off */
	/*bool af_lpf_enable;*/
	u16 af_lpf_enable;
	/* Median filter mode, suggest use MF_DISABLE */
	enum mid_mode filter_mode;
	/* Median filter device ID, for AF, use 1 */
	u8 filter_id;
	/* AF timing control anchor point */
	u16 line_cnt;
};
#pragma pack(pop)

/*
 *@typedef yhis_cfg_info
 *@brief YHIS configuration information
 */
#pragma pack(push)
#pragma pack(4)
struct yhis_cfg_info {
	u16 token_id;
	struct statistics_dld_region yhis_region;
};
#pragma pack(pop)


/*
 *@typedef antiflicker_cfg_info
 *@brief AntiFlicker configuration information
 */
#pragma pack(push)
#pragma pack(4)
struct antiflicker_cfg_info {
	u16 token_id;
	/* Used to control ROI shift position ratio of width,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_x;
	/* Used to control ROI shift position ratio of height,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_y;
};
#pragma pack(pop)

/*
 *@typedef subsample_cfg_info
 *@brief SubSample configuration information
 */
#pragma pack(push)
#pragma pack(4)
struct subsample_cfg_info {
	u16 token_id;
	/* Subsample engine buffer size in bytes */
	u32 buffer_image_size;
	/* Used to control ROI shift position ratio of width,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_x;
	/* Used to control ROI shift position ratio of height,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_y;
};
#pragma pack(pop)

/*
 *@typedef dld_sequence
 *@brief Download sequence structure
 */
#pragma pack(push)
#pragma pack(4)
struct dld_sequence {
	u8 preview_baisc_dld_seq_length;
	u8 preview_adv_dld_seq_length;
	u8 fast_converge_baisc_dld_seq_length;
	u8 preview_baisc_dld_seq[MAXSEQLEN];
	u8 preview_adv_dld_seq[MAXSEQLEN];
	u8 fast_converge_baisc_dld_seq[MAXSEQLEN];
};
#pragma pack(pop)

/*
 *@typedef cfg_3a_info
 *@brief 3A configuration
 */
#pragma pack(push)
#pragma pack(4)
struct cfg_3a_info {
	/* For sync between ISP firmware and 3A's lib */
	u32 magic_num;
	struct ae_cfg_info ae_info;
	struct awb_cfg_info awb_info;
	struct af_cfg_info af_info;
	struct yhis_cfg_info yhis_info;
	struct antiflicker_cfg_info antiflicker_info;
	struct subsample_cfg_info subsample_info;
};
#pragma pack(pop)



struct scinfo_mode_info_isp {
	u8                 ucSensorMode; /*FR,  binning_1, binning_2*/
	enum scinfo_sensor_mod_type   ucSensorMouduleType; /* IMX_219, OV4688 */
	u16                uwOriginalWidth; /* Original Width of Raw image */
	u16                uwOriginalHeight; /* Original Height of Raw image */
	u16                uwCropStartX;         /* Crop start X of Raw image */
	u16                uwCropStartY;         /* Crop start Y of Raw image */
	u16                uwCropEndX;         /* Crop end X of Raw image */
	u16                uwCropEndY;         /* Crop end Y of Raw image */
	u16                uwWidth; /* Width of Raw image */
	u16                uwHeight; /*Height of Raw image*/
	u16                uwFrameRate; /* FPS*/
	u32                udLineTime; /*line time x100*/
	enum scinfo_color_order  nColorOrder; /*color order*/
	u16                uwClampLevel; /*sensor's clamp level*/
	/* altek RAW10, android RAW10, packed10, unpacted10 */
	u8                 ucFormat;
	u8                 ucBitNumber; /* 8bit or 10 bit */
	u8                 ucMirrorFlip; /* no, mirror, flip or mirror+flip */
	u8                 cbc_enabled;
};

struct scinfo_out_bypassflg {
	u8    bBypassLV;
	u8    bBypassVideo;
	u8    bBypassStill;
	u8    bBypassMetaData;
};

struct scinfo_bayerscl_out_info {
	u16     uwBayerSCLOutWidth;
	u16     uwBayerSCLOutHeight;
};

struct scinfo_iq_param_idx_info {
	u8 iq_param_idx_lv;
	u8 iq_param_idx_video;
	u8 iq_param_idx_still;
};

struct scenario_info_ap {
	struct scinfo_mode_info_isp    tSensorInfo; /* tModeInfo */
	struct scinfo_out_bypassflg    tScenarioOutBypassFlag;
	struct scinfo_bayerscl_out_info tBayerSCLOutInfo;
	struct scinfo_iq_param_idx_info tIqParamIdxInfo;
};

enum e_alek_scinfo_color_order {
	E_ALEK_SCINFO_COLOR_ORDER_RG = 0,
	E_ALEK_SCINFO_COLOR_ORDER_GR,
	E_ALEK_SCINFO_COLOR_ORDER_GB,
	E_ALEK_SCINFO_COLOR_ORDER_BG
};

/*
*@typedef S_RAW_INFO
*@brief Info about raw image from sensor
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct altek_raw_info {
	u16 width;
	u16 height;
	u8 format;
	u8 bit_number;
	u8 mirror_flip;
	enum e_alek_scinfo_color_order n_color_order;
};
#pragma pack()

/*
*@typedef S_SHADING_INFO
*@brief Info about shading
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct altek_shading_info {
	u16 balanced_awb_gain_r;
	u16 balanced_awb_gain_g;
	u16 balanced_awb_gain_b;
	u8 orientation;
	u8 sensorid;
	u8 otpstatus;
	u8 otp_version;
	u16 shading_w;
	u16 shading_h;
	u16 h_r_gain;
	u16 m_r_gain;
	u16 l_r_gain;
	u16 cur_r_gain;
	u16 cur_b_gain;
	u16 cur_proj_x;
	u16 cur_proj_y;
	u8 r_p_run;
	u8 b_p_run;
	u8 ext_orientation;
	u8 ext_sensorid;
	u16 ext_h_r_gain;
	u16 ext_m_r_gain;
	u16 ext_l_r_gain;
	u16 ext_cur_proj_x;
	u16 ext_cur_proj_y;
	u8 ext_otpstatus;
	u8 ext_r_p_run;
	u8 ext_b_p_run;
	u8 shadtableflag;
	u8 ver_number[12];
	u32 tuning_version;
	u16 iso_step;
	u8 reserved[18];
};
#pragma pack()


/*
*@typedef S_IRP_INFO
*@brief Info about IRP tuning tool
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct altek_irp_info {
	u32 qmerge_ver;
	u32 tool_ver;
	u32 tuning_ver;
	u8 idx[IRP_TUNING_DEBUG_IDX];
	u8 sensor_id;
	u16 sensor_type;
	u8 sensor_mode;
	u8 quality_path;
	u16 bayer_scalar_w;
	u16 bayer_scalar_h;
	u16 isp_output_w;
	u16 isp_output_h;
	u8 verify_debug[IRP_TUNING_DEBUG_INFO];
	u8 func_bypass[IRP_TUNING_DEBUG_BYPASS];
	u16 awb_gain_r;
	u16 awb_gain_g;
	u16 awb_gain_b;
	u16 black_offset_r;
	u16 black_offset_g;
	u16 black_offset_b;
	u16 iso_speed;
	u32 color_tempature;
	s16 ae_bv;
	s16 bv_th_tone[IRP_TUNING_DEBUG_BVTH];
	s16 db_th_ccm[IRP_TUNING_DEBUG_BVTH];
	s16 bv_th_3dlut[IRP_TUNING_DEBUG_BVTH];
	s16 ccm[IRP_TUNING_DEBUG_CCM];
	u32 para_addr[IRP_TUNING_DEBUG_PARA_ADDR];
	u32 reserved[IRP_TUNING_DEBUG_RESERVED];
};
#pragma pack()

/*
*@typedef S_IRP_GAMMA_TONE
*@brief Debug info about IRP tuning tool
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct altek_irp_gamma_tone {
	u16 auwGamaTone[1027];
};
#pragma pack()


/*
*@typedef S_SW_INFO
*@brief Debug info about SW or UI setting
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct altek_sw_info {
	u8 sharpness_level;
	u8 saturation_level;
	u8 contrast_level;
	u8 brightness_value;
	u8 special_effect;
	u8 reserver[59];
};
#pragma pack()

/*
*@typedef S_IQ_INFO_1
*@brief Info about IQ 1( RAW, Shading, IRP, and SW)
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct altek_iq_info_1 {
	struct altek_raw_info raw_info;
	struct altek_shading_info shading_info;
	struct altek_irp_info irp_info;
	struct altek_sw_info sw_info;
};
#pragma pack()

/*
*@typedef S_IQ_INFO_2
*@brief Info about IQ 2( Gamma tone)
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct altek_iq_info_2 {
	struct altek_irp_gamma_tone tGammaTone;
};
#pragma pack()


/*
*@typedef altek_iq_info
*@brief Info about IQ( RAW, Shading, IRP, and SW)
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct altek_iq_info {
	struct altek_iq_info_1 iq_info_1;
	struct altek_iq_info_2 iq_info_2;
};
#pragma pack()

/*
*@struct isp_d_gain_info
*@brief D Gain of ISP
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct altek_d_gain_info {
	u32 isp_d_gain_r;
	u32 isp_d_gain_gr;
	u32 isp_d_gain_gb;
	u32 isp_d_gain_b;
};
#pragma pack()

struct sprd_isp_time {
	uint32_t sec;
	uint32_t usec;
};

/*After one statistics buffer has been used by 3A library.
**user need return this buffer to kernel.
*/
struct isp_statis_buf {
	uint32_t                         buf_size;
	unsigned long                    phy_addr;
	unsigned long                    vir_addr;
};

struct isp_cfg_img_buf {
	uint32_t                         format;
	uint32_t                         img_id;
	uint32_t                         width;
	uint32_t                         height;
	unsigned long                    yaddr;
	unsigned long                    uaddr;
	unsigned long                    vaddr;
	unsigned long                    yaddr_vir;
	unsigned long                    uaddr_vir;
	unsigned long                    vaddr_vir;
};

struct isp_statis_frame_output {
	uint32_t                         format;
	uint32_t                         buf_size;
	unsigned long                    phy_addr;
	unsigned long                    vir_addr;
	struct sprd_isp_time             time_stamp;
};

struct isp_statis_frame {
	uint32_t                         format;
	uint32_t                         evt;
	uint32_t                         buf_size;
	unsigned long                    phy_addr;
	unsigned long                    vir_addr;
	struct sprd_isp_time             time_stamp;
};

struct isp_addr {
	unsigned long                    chn0;
	unsigned long                    chn1;
	unsigned long                    chn2;
};

struct isp_cfg_img_param {
	/* img_id : 0-preview, 1-video, 2-still capture 3-statistics */
	uint32_t                    img_id;
	uint32_t                    dram_eb;
	uint32_t                    format;
	uint32_t                    width;
	uint32_t                    height;
	uint32_t                    buf_num;
	struct isp_addr             addr[IMG_BUF_NUM_MAX];
	struct isp_addr             addr_vir[IMG_BUF_NUM_MAX];
	int32_t                     addr_mfd[IMG_BUF_NUM_MAX]; /* iommu fd */
	uint32_t                    line_offset;
};

struct isp_img_frame_output {
	uint32_t                    format;
	uint32_t                    fid;
	uint32_t                    channel_id;
	uint32_t                    base_id;
	uint32_t                    img_id;
	uint32_t                    irq_id;
	uint32_t                    width;
	uint32_t                    height;
	uint32_t                    yaddr;
	uint32_t                    uaddr;
	uint32_t                    vaddr;
	uint32_t                    yaddr_vir;
	uint32_t                    uaddr_vir;
	uint32_t                    vaddr_vir;
	uint32_t                    img_y_fd;
	uint32_t                    img_u_fd;
	uint32_t                    img_v_fd;
	struct sprd_isp_time        time_stamp;
};

struct isp_img_size {
	uint32_t width;
	uint32_t height;
};

struct isp_irq_info {
	uint32_t irq_type;
	uint32_t irq_flag;
	uint32_t format;
	uint32_t channel_id;
	uint32_t base_id;
	uint32_t img_id;
	uint32_t irq_id;
	uint32_t sensor_id;
	uint32_t statis_cnt;
	unsigned long yaddr;
	unsigned long uaddr;
	unsigned long vaddr;
	unsigned long yaddr_vir;
	unsigned long uaddr_vir;
	unsigned long vaddr_vir;
	uint32_t img_y_fd;
	uint32_t img_u_fd;
	uint32_t img_v_fd;
	unsigned long length;
	struct isp_img_size buf_size;
	struct sprd_isp_time time_stamp;
	uint32_t frm_index;
};

struct isp_awb_match_gain_info {
	uint16_t r;
	uint16_t g;
	uint16_t b;
};

struct isp_awb_match_data {
	struct isp_awb_match_gain_info gain;
	struct isp_awb_match_gain_info gain_balanced;
	struct isp_awb_match_gain_info gain_flash_off;
	struct isp_awb_match_gain_info gain_capture;
	uint32_t ct;
	uint32_t ct_flash_off;
	uint32_t ct_capture;
	uint32_t is_update;
	uint16_t light_source;
	uint32_t awb_states;
	uint16_t awb_decision;
};

struct isp_ae_match_data {
	uint32_t iso;
	uint32_t exif_iso;
	uint32_t exposure_time;
	uint32_t exposure_line;
	uint32_t sensor_ad_gain;
	uint32_t isp_d_gain;
	uint16_t uw_cur_fps;
	int32_t bv_val;
	uint8_t uc_sensor_mode;
};

enum isp_match_data_op {
	ISP_GET_MATCH_AE_DATA,
	ISP_SET_MATCH_AE_DATA,
	ISP_GET_MATCH_AWB_DATA,
	ISP_SET_MATCH_AWB_DATA,
};

struct isp_match_data_param {
	enum isp_match_data_op op;
	union {
		struct isp_awb_match_data awb_data;
		struct isp_ae_match_data ae_data;
	};
};

struct isp_img_read_op {
	uint32_t cmd;
	uint32_t evt;
	union {
		struct isp_statis_frame_output    statis_frame;
		struct isp_img_frame_output       img_frame;
		struct isp_irq_info               irq_info;
	} param;
};

struct isp_img_write_op {
	uint32_t cmd;
	uint32_t reserved;
};

struct isp_init_mem_param {
	uint32_t                     fw_buf_size;
	signed int                   fw_buf_mfd;
	signed int                   fw_buf_dev_fd;
	unsigned long                fw_buf_vir_addr;
	unsigned long long           fw_buf_phy_addr; /* full mode channel */
	uint32_t                     shading_bin_offset;
	unsigned long                dram_buf_vir_addr;
	unsigned long                dram_buf_phy_addr; /* full mode channel */
	uint32_t                     irp_bin_offset;
	uint32_t                     cbc_bin_offset;
	uint32_t                     pdaf_supported;
	unsigned long                high_iso_buf_vir_addr;
	unsigned long                high_iso_phy_addr; /* full mode channel */
	uint32_t                     af_stats_independence;
};

struct isp_dev_init_param {
	uint32_t                     camera_id;
	uint32_t                     width;
	uint32_t                     height;
	uint32_t                     raw_mode;
};

struct isp_io_param {
	uint32_t                     sub_id;
	void  __user                 *property_param;
	uint32_t                     reserved;
};

struct isp_capability {
	uint32_t                     index;
	void  __user                 *property_param;
};

struct isp_raw_data {
	int32_t                      fd[ISP_RAWBUF_NUM];
	uint32_t                     phy_addr[ISP_RAWBUF_NUM];
	uint64_t                     virt_addr[ISP_RAWBUF_NUM];
	uint32_t                     size;
	uint32_t                     width;
	uint32_t                     height;
	uint32_t                     fmt;
	uint32_t                     cnt;
};

struct isp_hiso_data {
	int32_t                      fd;
	unsigned int                 phy_addr;
	unsigned int                 virt_addr;
	uint32_t                     size;
};

struct isp_img_mem {
	uint32_t                    img_fmt;
	uint32_t                    channel_id;
	uint32_t                    base_id;
	uint32_t                    width;
	uint32_t                    height;
	uint32_t                    is_reserved_buf;
	uint32_t                    img_y_fd;
	uint32_t                    img_u_fd;
	uint32_t                    img_v_fd;
	unsigned long               yaddr;
	unsigned long               uaddr;
	unsigned long               vaddr;
	unsigned long               yaddr_vir;
	unsigned long               uaddr_vir;
	unsigned long               vaddr_vir;
};

struct isp_frm_info {
	uint32_t                             channel_id;
	uint32_t                             frame_id;
	uint32_t                             frame_real_id;
	uint32_t                             height;
	uint32_t                             sec;
	uint32_t                             usec;
	uint32_t                             length;
	uint32_t                             base;
	uint32_t                             fmt;
	unsigned long                        yaddr;
	unsigned long                        uaddr;
	unsigned long                        vaddr;
	unsigned long                        yaddr_vir;
	unsigned long                        uaddr_vir;
	unsigned long                        vaddr_vir;
	uint32_t                             zsl_private;
	uint32_t                             fd;
	uint32_t                             monoboottime;
};


#define ISP_IO_MAGIC               'R'
#define ISP_IO_LOAD_FW             _IOW(ISP_IO_MAGIC, \
					0, struct isp_init_mem_param)
#define ISP_IO_IRQ                 _IOR(ISP_IO_MAGIC, \
					1, struct isp_irq_info)
#define ISP_IO_SET_STATIS_BUF      _IOW(ISP_IO_MAGIC, \
					2, struct isp_statis_buf)
#define ISP_IO_SET_IMG_BUF         _IOW(ISP_IO_MAGIC, \
					3, struct isp_cfg_img_buf)
#define ISP_IO_SET_IMG_PARAM       _IOW(ISP_IO_MAGIC, \
					4, struct isp_cfg_img_param)
#define ISP_IO_STREAM_ON           _IOW(ISP_IO_MAGIC, \
					5, uint32_t)
#define ISP_IO_STREAM_OFF          _IOW(ISP_IO_MAGIC, \
					6, uint32_t)
#define ISP_IO_SET_INIT_PARAM      _IOW(ISP_IO_MAGIC, \
					7, struct isp_dev_init_param)
#define ISP_IO_STOP                _IOW(ISP_IO_MAGIC, \
					8, uint32_t)
#define ISP_IO_CAPABILITY          _IOR(ISP_IO_MAGIC, \
					9, struct isp_capability)
#define ISP_IO_CFG_PARAM           _IOWR(ISP_IO_MAGIC, \
					10, struct isp_io_param)
#define ISP_IO_GET_TIME            _IOR(ISP_IO_MAGIC, \
					11, struct sprd_isp_time)
#define ISP_IO_GET_STATIS_BUF      _IOR(ISP_IO_MAGIC, \
					12, struct isp_irq_info)
#define ISP_IO_GET_ISP_ID          _IOR(ISP_IO_MAGIC, \
					13, uint32_t)
#define ISP_IO_GET_IQ_PARAM        _IOR(ISP_IO_MAGIC, \
					14, struct altek_iq_info)
#define ISP_IO_SET_RAW10           _IOW(ISP_IO_MAGIC, \
					15, struct isp_raw_data)
#define ISP_IO_SET_POST_PROC_YUV   _IOW(ISP_IO_MAGIC, \
					16, struct isp_img_mem)
#define ISP_IO_SET_FETCH_SRC_BUF   _IOW(ISP_IO_MAGIC, \
					17, struct isp_img_mem)
#define ISP_IO_SET_CAP_MODE        _IOW(ISP_IO_MAGIC, \
					18, uint32_t)
#define ISP_IO_SET_SKIP_NUM        _IOW(ISP_IO_MAGIC, \
					19, uint32_t)
#define ISP_IO_SET_HISO            _IOW(ISP_IO_MAGIC, \
					20, struct isp_hiso_data)
#define ISP_IO_CFG_CAP_BUF         _IOW(ISP_IO_MAGIC, \
					21, struct isp_img_mem)
#define ISP_IO_GET_USER_CNT        _IOR(ISP_IO_MAGIC, \
					22, int32_t)
#define ISP_IO_MATCH_DATA_CTRL     _IOW(ISP_IO_MAGIC, \
					23, struct isp_match_data_param)
#define ISP_IO_PROC_STILL          _IOW(ISP_IO_MAGIC, \
					24, uint32_t)
#define ISP_IO_SET_DECI_NUM        _IOW(ISP_IO_MAGIC, \
					25, uint32_t)
#define ISP_IO_SEL_TUNING_IQ       _IOW(ISP_IO_MAGIC, \
					26, uint32_t)
#define ISP_IO_GET_IMG_IQ_PARAM    _IOR(ISP_IO_MAGIC, \
					27, struct altek_iq_info)
#endif
