/*
* File: altek_bist.h                                                    *
* Description: delcalration of command info API for BIST                *
*                                                                       *
* (C)Copyright altek Corporation 2016                                   *
*                                                                       *
* History:                                                              *
*	2016/10/04; Initial version                                     *
*/

void isp_cmd_bist_set_bist_address_setting(u32 value);

void isp_cmd_bist_set_test_case_number_setting(void);

void isp_cmd_bist_set_scenario_info_sensor_info_setting(u8 entry, u32 value);

void isp_cmd_bist_set_scenario_info_bypass_flag_setting(u8 entry, u32 value);

void isp_cmd_bist_set_scenario_info_bayerscl_info_setting(u8 entry, u32 value);

void isp_cmd_bist_set_scenario_info_iq_param_idx_setting(u8 entry, u32 value);

void isp_cmd_bist_set_output_buff_format_lv_setting(u8 entry, u32 value);

void isp_cmd_bist_set_output_buff_format_video_setting(u8 entry, u32 value);

void isp_cmd_bist_set_output_buff_format_still_setting(u8 entry, u32 value);

void isp_cmd_bist_set_output_buff_format_metadata_setting(u8 entry, u32 value);

void isp_cmd_bist_set_output_buff_format_metadata_of_af_setting(u8 entry,
	u32 value);

void isp_cmd_bist_set_output_buff_format_raw_setting(u8 entry, u32 value);

void isp_cmd_bist_set_raw_frame_rate_setting(
		enum ispdrv_raw_frame_rate raw_frame_rate);

void isp_cmd_bist_set_bist_info_scenario_id_setting(u8 scenario_id);

void isp_cmd_bist_set_bist_info_independent_flag_setting(u8 independent_flag);
void isp_cmd_bist_set_writing_mode(
	enum ispdrv_sl_output_writing_mode writing_mode);

void isp_cmd_bist_set_hq_buff_size_setting(u32 size);

void isp_cmd_bist_set_raw_buff_size_setting(u32 size);

void isp_cmd_bist_set_bist_info_max_frame_count_setting(u32 max_frame_count);

void isp_cmd_bist_set_iso_speed_setting(u32 value);

void isp_cmd_bist_set_independent_order_setting(u8 order);

void isp_cmd_bist_set_golden_name_setting(u8 img_type, char *name, u8 size);

void isp_cmd_bist_set_debug_mode_setting(u8 value);

void isp_cmd_bist_load_variable_setting(void);

void isp_cmd_bist_reset_variable_setting(void);

