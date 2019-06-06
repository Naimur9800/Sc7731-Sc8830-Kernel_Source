/*
 *@file SCInfo.h
 *@brief for scenario information
 *@author Johnson's Scenario Tool
 *@version 1.1.1.0 2014/11/13
 */
#ifndef ALTEK_SCINFO
#define ALTEK_SCINFO

#include <video/sprd_isp_altek.h>
/*
 * //////////////////////////////////////////////////////////////////////////
 * File: SCInfo.h                                                            *
 * Description: Scenario information                                         *
 * //////////////////////////////////////////////////////////////////////////
 */
#define SCINFO_MAGICNUM 0x20160205

#define PIPE_COUNT    3

#define MAX_BUFFER_NUM 4

#define LSC_MAX_LENGTH 1800
#define S_OUTPUT_IMAGE_TOTAL 6


enum e_scinfo_color_order {
	E_SCINFO_COLOR_ORDER_RG = 0,
	E_SCINFO_COLOR_ORDER_GR,
	E_SCINFO_COLOR_ORDER_GB,
	E_SCINFO_COLOR_ORDER_BG
};


enum e_scinfo_sensor_module_type {
	E_SCINFO_SENSOR_MODULE_TYPE_IMX214 = 0,
};

enum e_selftest_mode {
	E_SELFTEST_NONE = 0,
	E_SELFTEST_BIST,
	E_SELFTEST_DUMP_RAW_AND_RESULT,
};

/*
 *@struct s_scinfo_out_bypassflg
 *@brief Scenario output bypass flag
 */
#pragma pack(push)
#pragma pack(4)
struct s_scinfo_out_bypassflg {
	bool bypass_lv;
	bool bypass_video;
	bool bypass_still;
	bool bypass_metadata;
};
#pragma pack(pop)

#pragma pack(push)
#pragma pack(4)
struct s_scinfo_bayerscl_out_info {
	u16 bayerscl_out_width;
	u16 bayerscl_out_height;
};
#pragma pack(pop)

#pragma pack(push)
#pragma pack(4)
struct s_scinfo_iq_param_idx_info {
	u8 iq_param_idx_lv;
	u8 iq_param_idx_video;
	u8 iq_param_idx_still;
};
#pragma pack(pop)

#pragma pack(push)
#pragma pack(4)
struct s_scinfo_self_test_info {
	u8 sensor_id_1st;
	u8 sensor_id_2nd;
	u8 frame_count_1stsensor;
	u8 frame_count_2ndsensor;
	u8 max_frame_cnt;
	u8 self_test_mode;
	u8 download_raw_mode;
	bool is_dual_sensor_mode;
	bool is_independent;
	bool is_frame_done_1stsensor;
	bool is_frame_done_2ndsensor;
};
#pragma pack(pop)

#pragma pack(push)
#pragma pack(4)
struct otp_data {
	bool has_data;
	u32  otp_addr;
};
#pragma pack(pop)

#pragma pack(push)
#pragma pack(4)
/* S_SCINFO_MODE_INFO for ISP. ISP */
struct s_scinfo_mode_info_isp {
	/* FR,  binning_1, binning_2,... */
	/* 0: FR, 1: BINING */
	u8 sensor_mode;
	/* IMX_219, OV4688,... */
	/* Reserved */
	enum e_scinfo_sensor_module_type sensor_module_type;
	/* Original Width of Raw image */
	u16 original_width;
	/* Original Height of Raw image */
	u16 original_height;
	/* Crop start X of Raw image */
	u16 crop_start_x;
	/* Crop start Y of Raw image */
	u16 crop_start_y;
	/* Crop end X of Raw image */
	u16 crop_end_x;
	/* Crop end Y of Raw image */
	u16 crop_end_y;
	/* Width of Raw image */
	u16 width;
	/* Height of Raw image */
	u16 height;
	/* Frame Rate x100. for example, 3000 means 30 fps */
	u16 frame_rate;
	/* line time(us) x100. for example, 1315 means 13.15 us */
	u32 line_time;
	/* color order */
	enum e_scinfo_color_order  n_color_order;
	/* sensor's clamp level */
	u16 clamp_level;
	/* altek RAW10, android RAW10, packed10, unpacted10 */
	/* Reserved */
	u8 format;
	/* 8bit or 10 bit */
	/* Reserved */
	u8 bit_number;
	/* no, mirror, flip or mirror+flip */
	/* 0: No Mirro and Flip, 1: Mirror, 2: Flip, 3:Mirror and Flip */
	u8 mirror_flip;
	/* CBC enabled */
	u8 cbc_enabled;
};
#pragma pack(pop)


#pragma pack(push)
#pragma pack(4)
struct s_scenario_info_isp {
	u32 ul_magic_num;
	struct s_scinfo_mode_info_isp mode_info[PIPE_COUNT];
	struct s_scinfo_out_bypassflg out_bypass_flag[PIPE_COUNT];
	struct s_scinfo_bayerscl_out_info bayerscl_out_info[PIPE_COUNT];
	struct s_scinfo_iq_param_idx_info iq_param_idx_info[PIPE_COUNT];
	struct s_scinfo_self_test_info self_test_info;
};
#pragma pack(pop)


#pragma pack(push)
#pragma pack(4)
struct s_scenario_info_ap {
	/*  tModeInfo */
	struct s_scinfo_mode_info_isp sensor_info;
	struct s_scinfo_out_bypassflg out_bypass_flag;
	struct s_scinfo_bayerscl_out_info bayerscl_out_info;
	struct s_scinfo_iq_param_idx_info iq_param_idx_info;
};
#pragma pack(pop)

/*
 *@typedef s_scenario_size
 *@brief size structure
 */
#pragma pack(push)
#pragma pack(4)
struct s_scenario_size {
	u16 width;
	u16 height;
};
#pragma pack()



/*
 *@typedef s_scenario_dzoom_info
 *@brief digital room information of a sensor
 */
#pragma pack(push)
#pragma pack(4)
struct s_scenario_dzoom_info {
	struct s_scenario_size base_size[PIPE_COUNT];
	bool crop_size_changed[PIPE_COUNT];
	struct s_scenario_size crop_size[PIPE_COUNT];
	struct s_scenario_size eff_crop_size[PIPE_COUNT][MAX_BUFFER_NUM];
};
#pragma pack()


/*
 *@typedef s_raw_info
 *@brief Info about raw image from sensor
 */
#pragma pack(push)
#pragma pack(4)
struct s_raw_info {
	u16 width;
	u16 height;
	u8 format;
	u8 bit_number;
	u8 mirror_flip;
	enum e_scinfo_color_order n_color_order;
};
#pragma pack()

/*
 *@typedef s_shading_info
 *@brief Info about shading
 */
#pragma pack(push)
#pragma pack(4)
struct s_shading_info {
	u16 balanced_awb_gain_r;
	u16 balanced_awb_gain_g;
	u16 balanced_awb_gain_b;
	u8 input_data;
	u8 applied_sensor_id;
	u8 otp_info;
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
	u8 ext_input_data;
	u8 ext_applied_sensor_id;
	u8 ext_otp_info;
	u16 ext_h_r_gain;
	u16 ext_m_r_gain;
	u16 ext_l_r_gain;
	u16 ext_cur_proj_x;
	u16 ext_cur_proj_y;
	u8 ext_r_p_run;
	u8 ext_b_p_run;
	u8 orientation;
	u8 ver_number[12];
	u16 tuning_version;
};
#pragma pack()


/*
 *@typedef s_irp_info
 *@brief Info about IRP tuning tool
 */
#pragma pack(push)
#pragma pack(4)
struct s_irp_info {
	u32 qmerge_ver;
	u32 tool_ver;
	u32 tuning_ver;
	u8 sensor_id;
	u16 sensor_type;
	u8 sensor_mode;
	u8 tuning_id;
	u8 level_type;
	u8 quality_path;
	u16 bayer_scalar_w;
	u16 bayer_scalar_h;
	bool debug_info[6];
	u16 awb_gain_r;
	u16 awb_gain_g;
	u16 awb_gain_b;
	u16 black_offset_r;
	u16 black_offset_g;
	u16 black_offset_b;
	u32 color_tempature;
	int ccm[9];
	u32 para_addr[56];
	u32 reserved[5];
};
#pragma pack()

/*
 *@typedef s_irp_gamma_tone
 *@brief Debug info about IRP tuning tool
 */
#pragma pack(push)
#pragma pack(4)
struct s_irp_gamma_tone {
	u16 gama_tone[1027];
};
#pragma pack()

/*
 *@typedef iq_otp_info
 *@brief Debug info about OTP_REPORT
 */
#pragma pack(push)
#pragma pack(4)
struct iq_otp_info {
	u8 cali_status;
	u16 iso;
	u16 r_gain;
	u16 g_gain;
	u16 b_gain;
	u8  lsc[LSC_MAX_LENGTH];
	u16 lsc_length;
};
#pragma pack()

/*
 *@typedef iq_ccm_info
 *@brief Debug info about ccm
 */
#pragma pack(push)
#pragma pack(4)
struct iq_ccm_info {
	s32 adCCM[9];
};
#pragma pack()


/*
*@struct isp_d_gain_info
*@brief D Gain of ISP
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct isp_d_gain_info {
	u32 isp_d_gain_r;
	u32 isp_d_gain_gr;
	u32 isp_d_gain_gb;
	u32 isp_d_gain_b;
};
#pragma pack(pop)

/*
 *@typedef s_sw_info
 *@brief Debug info about SW or UI setting
 */
#pragma pack(push)
#pragma pack(4)
struct s_sw_info {
	u8 sharpness_level;
	u8 saturation_level;
	u8 contrast_level;
	u8 brightness_value;
	u8 special_effect;
	u8 reserver[59];
};
#pragma pack()

/*
 *@typedef s_iq_info_1
 *@brief information about IQ (RAW, Shading, IRP, and SW)
 */
#pragma pack(push)
#pragma pack(4)
struct s_iq_info_1 {
	struct s_raw_info raw_info;
	struct s_shading_info shading_info;
	struct s_irp_info irp_info;
	struct s_sw_info sw_info;
};
#pragma pack()

/*
 *@typedef s_iq_info_2
 *@brief information about IQ(Gamma tone)
 */
#pragma pack(push)
#pragma pack(4)
struct s_iq_info_2 {
	struct s_irp_gamma_tone gamma_tone;
};
#pragma pack()



/*
 *@typedef s_iq_info
 *@brief Info about IQ( RAW, Shading, IRP, and SW)
 */
#pragma pack(push)
#pragma pack(4)
struct s_iq_info {
	struct s_raw_info raw_info;
	struct s_shading_info shading_info;
	struct s_irp_info irp_info;
	struct s_sw_info sw_info;
	struct s_irp_gamma_tone gamma_tone;
};
#pragma pack()


/*
 *@typedef s_iq_info_addr
 *@brief Info address about IQ( RAW, Shading, IRP, and SW)
 */
#pragma pack(push)
#pragma pack(4)
struct s_iq_info_addr {
	u32 raw_info_addr[S_OUTPUT_IMAGE_TOTAL][MAX_BUFFER_NUM];
	u32 shading_info_addr[S_OUTPUT_IMAGE_TOTAL][MAX_BUFFER_NUM];
	u32 irp_info_addr[S_OUTPUT_IMAGE_TOTAL][MAX_BUFFER_NUM];
	u32 sw_info_addr[S_OUTPUT_IMAGE_TOTAL][MAX_BUFFER_NUM];
	u32 gamma_tone_addr[S_OUTPUT_IMAGE_TOTAL][MAX_BUFFER_NUM];
#if 0
	u32 color_temperature_addr[S_OUTPUT_IMAGE_TOTAL][MAX_BUFFER_NUM];
	u32 ccm_addr[S_OUTPUT_IMAGE_TOTAL][MAX_BUFFER_NUM];
	u32 iq_opt_info_addr[S_OUTPUT_IMAGE_TOTAL][MAX_BUFFER_NUM];
	u32 valid_ad_gain_addr[S_OUTPUT_IMAGE_TOTAL][MAX_BUFFER_NUM];
	u32 valid_exp_time_addr[S_OUTPUT_IMAGE_TOTAL][MAX_BUFFER_NUM];
#endif
};
#pragma pack()
#if 0
#pragma pack(push)
#pragma pack(4)
struct s_raw_info_setting {
	u8 sensor_id;
	u8 irp_sensor_mode;
	u32 irp_sensor_type;
	u8 irp_param_id;
	u16 irp_iso_speed;
	u8 hdr_type;
	u8 quality_path;
	/* w:16, h:16 */
	u32 bayer_scl_wh;
	u16 temperature;
	/* contrast */
	u8 cont_level;
	/* saturation */
	u8 sat_level;
	/* sharpness */
	u8 sharp_level;
	/* special effect */
	u8 spec_eff;
	u32 weight_ori;
	u32 height_ori;
	u16 detected_awb_r;
	u16 detected_awb_gr;
	u16 detected_awb_gb;
	u16 detected_awb_b;
};
#pragma pack()

/* firmware's address of s_raw_info_settings */
struct s_raw_info_setting_addr {
	u32 raw_info_addr_iva[MAX_BUFFER_NUM];
};
#endif

/* void SCInfo_SetScenarioInfo(S_SCENARIO_INFO *a_ptScenarioInfo); */
/* S_SCENARIO_INFO* SCInfo_GetScenarioInfo(); */


#endif /* ALTEK_SCINFO */
