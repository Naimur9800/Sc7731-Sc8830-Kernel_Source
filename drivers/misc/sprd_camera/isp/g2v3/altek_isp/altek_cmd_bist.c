/*
 * File: altek_cmd_bist.c						*
 * Description: command info. for BIST					*
 *									*
 * (C)Copyright Altek Corporation 2016					*
 *									*
 * History:								*
 *   2016/10/04; ; Initial version					*
 */


#include <linux/ctype.h>
#include "altek_ahb_drv.h"
#include "altek_log_local.h"
#include "altek_isp_drv.h"
#include "altek_bist.h"
#include "altek_sc_info.h"


/*
* ////////////////////////////////////////////////////
*	Private Variables
* ///////////////////////////////////////////////////
*/

static u32 g_cmd_bist_iva;
static u64 g_cmd_bist_kva;
static bool g_cmd_debug_mode;
static u8 g_cmd_test_case_number;
static u8 g_cmd_independent_order;
static struct s_scenario_info_ap
	g_cmd_scenario_info[MAX_TEST_CASE_NUMBER][MAX_USED_SENSOR_NUMBER];
static struct s_output_buff_format_info
	g_cmd_output_buff_format[MAX_TEST_CASE_NUMBER][MAX_USED_SENSOR_NUMBER];
static struct s_scinfo_self_test_info g_cmd_self_test_info[
	MAX_TEST_CASE_NUMBER];
static struct s_bist_info g_cmd_bist_info[MAX_TEST_CASE_NUMBER];
static enum ispdrv_raw_frame_rate
	g_cmd_raw_frame_rate[MAX_TEST_CASE_NUMBER][MAX_RAW_FR_RATE_CHANGE];
static u32 g_cmd_iso_speed[MAX_TEST_CASE_NUMBER];
static char *g_cmd_golden_name
	[MAX_TEST_CASE_NUMBER][MAX_USED_SENSOR_NUMBER][MAX_GOLDEN_IMG_NUMBER];
static enum ispdrv_sl_output_writing_mode g_cmd_writing_mode[
	MAX_TEST_CASE_NUMBER];
static u32 g_cmd_hq_buffer_size[MAX_TEST_CASE_NUMBER];
static u32 g_cmd_raw_buffer_size[MAX_TEST_CASE_NUMBER];
static int g_raw_fr_rate_idx;

/*
 * ////////////////////////////////////////////////////
 *	Private Function Prototype		 *
 * ////////////////////////////////////////////////////
*/
static bool alloc_memory(void);

void isp_cmd_bist_set_bist_address_setting(u32 value)
{
	g_cmd_bist_iva = value;
	g_cmd_bist_kva = (unsigned long)__va(value);
}

void isp_cmd_bist_set_test_case_number_setting(void)
{
	g_cmd_test_case_number++;
	if (g_cmd_test_case_number > MAX_TEST_CASE_NUMBER)
		g_cmd_test_case_number = MAX_TEST_CASE_NUMBER;

	g_cmd_self_test_info[g_cmd_test_case_number-1].self_test_mode =
		E_SELFTEST_BIST;

	/* first run. initialize variables. */
	if (g_cmd_test_case_number == 1)
		if (false == alloc_memory())
			return;

	g_cmd_independent_order = 0;
	g_raw_fr_rate_idx = 0;
}

void isp_cmd_bist_set_scenario_info_sensor_info_setting(u8 entry, u32 value)
{
	switch (entry) {
	case 0:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.sensor_mode = (u8) value;
		break;
	case 1:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.sensor_module_type =
			(enum e_scinfo_sensor_module_type) value;
		break;
	case 2:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.original_width = (u16) value;
		break;
	case 3:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.original_height = (u16) value;
		break;
	case 4:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.crop_start_x = (u16) value;
		break;
	case 5:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.crop_start_y = (u16) value;
		break;
	case 6:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.crop_end_x = (u16) value;
		break;
	case 7:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.crop_end_y = (u16) value;
		break;
	case 8:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.width = (u16) value;
		break;
	case 9:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.height = (u16) value;
		break;
	case 10:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.frame_rate = (u16) value;
		break;
	case 11:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.line_time = value;
		break;
	case 12:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.n_color_order = (enum e_scinfo_color_order) value;
		break;
	case 13:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.clamp_level = (u16) value;
		break;
	case 14:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.format = (u8) value;
		break;
	case 15:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.bit_number = (u8) value;
		break;
	case 16:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.mirror_flip = (u8) value;
		break;
	case 17:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].sensor_info
			.cbc_enabled = (u8) value;
		break;
	default:
		break;
	}
}

void isp_cmd_bist_set_scenario_info_bypass_flag_setting(u8 entry, u32 value)
{
	switch (entry) {
	case 0:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].out_bypass_flag
			.bypass_lv = (bool) value;
		break;
	case 1:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].out_bypass_flag
			.bypass_video = (bool) value;
		break;
	case 2:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].out_bypass_flag
			.bypass_still = (bool) value;
		break;
	case 3:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].out_bypass_flag
			.bypass_metadata = (bool) value;
		break;
	default:
		break;
	}
}

void isp_cmd_bist_set_scenario_info_bayerscl_info_setting(u8 entry, u32 value)
{
	switch (entry) {
	case 0:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].bayerscl_out_info
			.bayerscl_out_width = (u16) value;
		break;
	case 1:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].bayerscl_out_info
			.bayerscl_out_height = (u16) value;
		break;
	default:
		break;
	}
}

void isp_cmd_bist_set_scenario_info_iq_param_idx_setting(u8 entry, u32 value)
{
	switch (entry) {
	case 0:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].iq_param_idx_info
			.iq_param_idx_lv = (u8) value;
		break;
	case 1:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].iq_param_idx_info
			.iq_param_idx_video = (u8) value;
		break;
	case 2:
		g_cmd_scenario_info[g_cmd_test_case_number-1]
			[g_cmd_independent_order].iq_param_idx_info
			.iq_param_idx_still = (u8) value;
		break;
	default:
		break;
	}
}

void isp_cmd_bist_set_output_buff_format_lv_setting(u8 entry, u32 value)
{
	switch (entry) {
	case 0:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.enable[V4L2_ISP_IMG1] = (u8) value;
		break;
	case 1:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.format[V4L2_ISP_IMG1] = (u8) value;
		break;
	case 2:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.width[V4L2_ISP_IMG1] = value;
		break;
	case 3:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.height[V4L2_ISP_IMG1] = value;
		break;
	case 4:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.buffer_number[V4L2_ISP_IMG1] = (u8) value;
		break;
	case 5:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.line_offset[V4L2_ISP_IMG1] = (u16) value;
		break;
	default:
		break;
	}
}

void isp_cmd_bist_set_output_buff_format_video_setting(u8 entry, u32 value)
{
	switch (entry) {
	case 0:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.enable[V4L2_ISP_IMG2] = (u8) value;
		break;
	case 1:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.format[V4L2_ISP_IMG2] = (u8) value;
		break;
	case 2:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.width[V4L2_ISP_IMG2] = value;
		break;
	case 3:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.height[V4L2_ISP_IMG2] = value;
		break;
	case 4:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.buffer_number[V4L2_ISP_IMG2] = (u8) value;
		break;
	case 5:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.line_offset[V4L2_ISP_IMG2] = (u16) value;
		break;
	default:
		break;
	}
}

void isp_cmd_bist_set_output_buff_format_still_setting(u8 entry, u32 value)
{
	switch (entry) {
	case 0:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.enable[V4L2_ISP_IMG3] = (u8) value;
		break;
	case 1:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.format[V4L2_ISP_IMG3] = (u8) value;
		break;
	case 2:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.width[V4L2_ISP_IMG3] = value;
		break;
	case 3:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.height[V4L2_ISP_IMG3] = value;
		break;
	case 4:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.buffer_number[V4L2_ISP_IMG3] = (u8) value;
		break;
	case 5:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.line_offset[V4L2_ISP_IMG3] = (u16) value;
		break;
	default:
		break;
	}
}

void isp_cmd_bist_set_output_buff_format_metadata_setting(u8 entry, u32 value)
{
	switch (entry) {
	case 0:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.enable[V4L2_ISP_STATITISTICS] = (u8) value;
		break;
	case 1:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.format[V4L2_ISP_STATITISTICS] = (u8) value;
		break;
	case 2:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.width[V4L2_ISP_STATITISTICS] = value;
		break;
	case 3:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.height[V4L2_ISP_STATITISTICS] = value;
		break;
	case 4:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.buffer_number[V4L2_ISP_STATITISTICS] = (u8) value;
		break;
	case 5:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.line_offset[V4L2_ISP_STATITISTICS] = (u16) value;
		break;
	default:
		break;
	}
}

void isp_cmd_bist_set_output_buff_format_metadata_of_af_setting(u8 entry,
	u32 value)
{
	switch (entry) {
	case 0:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.enable[V4L2_ISP_STATITISTICS_OF_AF] = (u8) value;
		break;
	case 1:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.format[V4L2_ISP_STATITISTICS_OF_AF] = (u8) value;
		break;
	case 2:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.width[V4L2_ISP_STATITISTICS_OF_AF] = value;
		break;
	case 3:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.height[V4L2_ISP_STATITISTICS_OF_AF] = value;
		break;
	case 4:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.buffer_number[V4L2_ISP_STATITISTICS_OF_AF] =
			(u8) value;
		break;
	case 5:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.line_offset[V4L2_ISP_STATITISTICS_OF_AF] = (u16) value;
		break;
	default:
		break;
	}
}


void isp_cmd_bist_set_output_buff_format_raw_setting(u8 entry, u32 value)
{
	switch (entry) {
	case 0:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.enable[V4L2_ISP_RAW] = (u8) value;
		break;
	case 1:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.format[V4L2_ISP_RAW] = (u8) value;
		break;
	case 2:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.width[V4L2_ISP_RAW] = value;
		break;
	case 3:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.height[V4L2_ISP_RAW] = value;
		break;
	case 4:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.buffer_number[V4L2_ISP_RAW] = (u8) value;
		break;
	case 5:
		g_cmd_output_buff_format[g_cmd_test_case_number-1]
			[g_cmd_independent_order]
			.line_offset[V4L2_ISP_RAW] = (u16) value;
		break;
	default:
		break;
	}
}

void isp_cmd_bist_set_raw_frame_rate_setting(
		enum ispdrv_raw_frame_rate raw_frame_rate)
{
	g_cmd_raw_frame_rate[g_cmd_test_case_number-1][g_raw_fr_rate_idx]
		= raw_frame_rate;

	g_raw_fr_rate_idx =
		(g_raw_fr_rate_idx+1 >= MAX_RAW_FR_RATE_CHANGE)
		? MAX_RAW_FR_RATE_CHANGE : (g_raw_fr_rate_idx + 1);
}

void isp_cmd_bist_set_bist_info_scenario_id_setting(u8 scenario_id)
{
	g_cmd_bist_info[g_cmd_test_case_number-1]
		.scenario_id[g_cmd_independent_order] = scenario_id;
}

void isp_cmd_bist_set_bist_info_independent_flag_setting(u8 independent_flag)
{
	g_cmd_self_test_info[g_cmd_test_case_number-1]
		.is_independent = independent_flag;
}

void isp_cmd_bist_set_writing_mode(
	enum ispdrv_sl_output_writing_mode writing_mode)
{
	g_cmd_writing_mode[g_cmd_test_case_number-1] = writing_mode;
}

void isp_cmd_bist_set_hq_buff_size_setting(u32 size)
{
	g_cmd_hq_buffer_size[g_cmd_test_case_number-1] = size;
}

void isp_cmd_bist_set_raw_buff_size_setting(u32 size)
{
	g_cmd_raw_buffer_size[g_cmd_test_case_number-1] = size;
}

void isp_cmd_bist_set_bist_info_max_frame_count_setting(u32 max_frame_count)
{
	g_cmd_self_test_info[g_cmd_test_case_number-1]
		.max_frame_cnt = max_frame_count;
}

void isp_cmd_bist_set_iso_speed_setting(u32 iso_speed)
{
	g_cmd_iso_speed[g_cmd_test_case_number-1] = iso_speed;
}

void isp_cmd_bist_set_independent_order_setting(u8 order)
{
	/* Currently, the software
	 * only support two path activating in one time
	 */
	g_cmd_independent_order = 1;
}

void isp_cmd_bist_set_golden_name_setting(u8 img_type, char *name, u8 size)
{
	/* Check img_type vaild */
	if (img_type >= MAX_GOLDEN_IMG_NUMBER) {
		isp_err_lo_combo_desc_tag("Wrong img_type: %d, MAX: %d",
			img_type, MAX_GOLDEN_IMG_NUMBER-1);
		return;
	}

	/* Check the size of the file path. */
	if (size > MAX_FILE_PATH_SIZE) {
		isp_err_lo_combo_desc_tag(
			"the size of the file path is too long: %d, MAX: %d",
			img_type, MAX_FILE_PATH_SIZE);
		return;
	}

	memcpy(g_cmd_golden_name[g_cmd_test_case_number-1][
		g_cmd_independent_order][img_type],
		name, size);
}


void isp_cmd_bist_set_debug_mode_setting(u8 value)
{
	if (value == 0)
		g_cmd_debug_mode = false;
	else
		g_cmd_debug_mode = true;
}

void isp_cmd_bist_load_variable_setting(void)
{
	ispdrv_set_bist_addr(g_cmd_bist_iva, g_cmd_bist_kva);
	bist_set_hq_buffer_size_setting(g_cmd_hq_buffer_size);
	bist_set_raw_buffer_size_setting(g_cmd_raw_buffer_size);
	bist_set_test_case_number_setting(g_cmd_test_case_number);
	bist_set_scenario_info_setting(g_cmd_scenario_info);
	bist_set_output_buff_format_setting(g_cmd_output_buff_format);
	bist_set_self_test_info_setting(g_cmd_self_test_info);
	bist_set_bist_info_setting(g_cmd_bist_info);
	bist_set_raw_frame_rate_setting(g_cmd_raw_frame_rate);
	bist_set_isp_speed_setting(g_cmd_iso_speed);
	bist_set_ss_opt_writing_mode_setting(g_cmd_writing_mode);
	bist_set_golden_name_setting(g_cmd_golden_name);
	bist_set_debug_mode_setting(g_cmd_debug_mode);
}

void isp_cmd_bist_reset_variable_setting(void)
{
	int i = 0, j = 0, k = 0;

	g_cmd_test_case_number = 0;
	g_cmd_independent_order = 0;
	g_raw_fr_rate_idx = 0;

	memset(g_cmd_scenario_info, 0, sizeof(g_cmd_scenario_info));

	memset(g_cmd_output_buff_format, 0,
		sizeof(g_cmd_output_buff_format));

	memset(g_cmd_self_test_info, 0, sizeof(g_cmd_self_test_info));
	memset(g_cmd_bist_info, 0, sizeof(g_cmd_bist_info));
	memset(g_cmd_iso_speed, 0, sizeof(g_cmd_iso_speed));

	for (i = 0; i < MAX_TEST_CASE_NUMBER; i++)
		for (j = 0; j < MAX_RAW_FR_RATE_CHANGE; j++)
			g_cmd_raw_frame_rate[i][j] = ISP_RAW_FR_MAX;

	/* free memory */
	for (i = 0; i < MAX_TEST_CASE_NUMBER; i++) {
		for (j = 0; j < MAX_USED_SENSOR_NUMBER; j++) {
			for (k = 0; k < MAX_GOLDEN_IMG_NUMBER; k++) {
				if (g_cmd_golden_name[i][j][k] != NULL)
					kfree((void *) g_cmd_golden_name[
						i][j][k]);
				g_cmd_golden_name[i][j][k] = NULL;
			}
		}
	}

}


static bool alloc_memory(void)
{
	int test = 0, order = 0, type = 0;

	for (test = 0; test < MAX_TEST_CASE_NUMBER; test++) {
		for (order = 0; order < MAX_USED_SENSOR_NUMBER; order++) {
			for (type = 0; type < MAX_GOLDEN_IMG_NUMBER; type++) {
				g_cmd_golden_name[test][order][type] =
					kmalloc(
						MAX_FILE_PATH_SIZE, GFP_KERNEL);
				if (NULL ==
					g_cmd_golden_name[test][order][type]) {
					isp_err_lo_combo_desc_tag(
						"Err: Unable to alloc. mem.");
					return false;
				}

				memset(g_cmd_golden_name[test][order][type], 0,
					MAX_FILE_PATH_SIZE);
			}
		}
	}

	return true;
}
