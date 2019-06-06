/*
 * File: altek_cmd.c							*
 * Description: ISP Device Node Cmd Parser				*
 *									*
 * (C)Copyright Altek Corporation 2016					*
 *									*
 * History:								*
 *   2016/06/17; ; Initial version					*
 */

#include <linux/ctype.h>

#include "altek_ahb_drv.h"
#include "altek_isp_drv.h"
#include "altek_dump_utility.h"
#include "altek_cmd.h"
#include "altek_common.h"
#include "altek_log_local.h"
#include "altek_cmd_bist.h"

struct cmd_tree_s {
	char *token;
	ssize_t (*action)(char *cmd, struct cmd_tree_s *tree);
	struct cmd_tree_s *sub;
	char *help;
};

#define isspace2(ch) ((ch) == ' ' || (ch) == '\f' || \
		      (ch) == '\t' || (ch) == '\v')
#define iscrlf(ch) ((ch) == '\r' || (ch) == '\n')

static char *last;
static u64 strtoull(const char *cp, char **endp, u32 base);
static u32 strtoul(const char *cp, char **endp, u32 base);
static char *expected_token(u32 *len, char **end);
static char *expected_newline(char **end);

static struct cmd_tree_s root[];
static struct cmd_tree_s root_ahb[];
static struct cmd_tree_s root_drv[];
static struct cmd_tree_s root_drv_log[];
static struct cmd_tree_s root_drv_auto_dump[];
static struct cmd_tree_s root_fw[];
static struct cmd_tree_s root_fw_log[];
static struct cmd_tree_s root_isp[];
static struct cmd_tree_s root_vmem[];
static struct cmd_tree_s root_pmem[];

static struct cmd_tree_s root_bist[];
static struct cmd_tree_s root_bist_set[];
static struct cmd_tree_s root_bist_set_scenario_info[];
static struct cmd_tree_s root_bist_set_output_buff_format[];
static struct cmd_tree_s root_bist_set_bist_info[];
static struct cmd_tree_s root_bist_set_ss_opt[];
static struct cmd_tree_s root_bist_set_mem_info[];

static ssize_t isp_cmd_subtree(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_help(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_ahb_dump(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_ahb_get(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_ahb_set(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_drv_auto_dump(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_drv_bist(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_drv_log_threshold(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_drv_version(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_fw_dump(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_fw_log_threshold(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_fw_log_force_switch(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_fw_version(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_isp_dump(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_isp_get(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_isp_set(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_vmem_dump(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_pmem_dump(char *cmd, struct cmd_tree_s *tree);

static ssize_t isp_cmd_bist_load_variables(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_address(char *cmd, struct cmd_tree_s *tree);

static ssize_t isp_cmd_bist_set_test_case_number(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_scenario_info_sensor_info(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_scenario_info_bypass_flag(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_scenario_info_bayerscl_info(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_scenario_info_iq_param_idx(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_lv_output_buff_format(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_video_output_buff_format(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_still_output_buff_format(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_metadata_output_buff_format(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_metadata_of_af_output_buff_format(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_raw_output_buff_format(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_raw_frame_rate(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_iso_speed(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_independent_setting_order(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_golden_name(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_debug_mode(char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_info_set_scenario_id(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_info_set_max_frame_count(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_info_set_independent_flag(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_ss_opt_writing_mode(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_hq_buffer_size(
	char *cmd, struct cmd_tree_s *tree);
static ssize_t isp_cmd_bist_set_raw_buffer_size(
	char *cmd, struct cmd_tree_s *tree);


static struct cmd_tree_s root[] = {
	{"help", isp_cmd_help, root, "This help"},
	/* */
	{"ahb", isp_cmd_subtree, root_ahb, "AHB registers ..."},
	{"driver", isp_cmd_subtree, root_drv, "ISP driver ..."},
	{"drv", isp_cmd_subtree, root_drv, "ISP driver ..."},
	{"firmware", isp_cmd_subtree, root_fw, "ISP firmware ..."},
	{"fw", isp_cmd_subtree, root_fw, "ISP firmware ..."},
	{"isp", isp_cmd_subtree, root_isp, "ISP internal registers ..."},
	{"mem", isp_cmd_subtree, root_vmem, "Virtual memory ..."},
	{"vmem", isp_cmd_subtree, root_vmem, "Virtual memory ..."},
	{"pmem", isp_cmd_subtree, root_pmem, "Physical memory ..."},
	{"bist", isp_cmd_subtree, root_bist, "BIST ..."},
	/* */
	{}
};

static struct cmd_tree_s root_ahb[] = {
	{"help", isp_cmd_help, root_ahb, "This help"},
	/* */
	{"dump", isp_cmd_ahb_dump, NULL,
		"Dump registers to " DEFAULT_DUMP_PATH},
	{"get", isp_cmd_ahb_get, NULL, "get REGISTER_OFFSET"},
	{"set", isp_cmd_ahb_set, NULL, "set REGISTER_OFFSET VALUE"},
	/* */
	{}
};

static struct cmd_tree_s root_drv[] = {
	{"help", isp_cmd_help, root_drv, "This help"},
	/* */
	/*{"auto-dump", isp_cmd_drv_auto_dump, NULL,
	 *	"auto-dump none|on-error|on-success|always"},
	 */
	 /* TODO: */
	{"auto-dump", isp_cmd_drv_auto_dump, NULL, "auto-dump none|on-error"},
	{"bist", isp_cmd_drv_bist, NULL, "Built-In-Self-Test ..."},
	{"log", isp_cmd_subtree, root_drv_log, "Log facility ..."},
	{"version", isp_cmd_drv_version, NULL, "Version"},
	/* */
	{}
};

static struct cmd_tree_s root_drv_log[] = {
	{"help", isp_cmd_help, root_drv_log, "This help"},
	/* */
	{"threshold", isp_cmd_drv_log_threshold, NULL, "0..7"},
	/* */
	{}
};

static struct cmd_tree_s root_fw[] = {
	{"help", isp_cmd_help, root_fw, "This help"},
	/* */
	{"dump", isp_cmd_fw_dump, NULL, "Dump firmware to " DEFAULT_DUMP_PATH},
	{"log", isp_cmd_subtree, root_fw_log, "Log facility ..."},
	{"version", isp_cmd_fw_version, NULL, "Version"},
	/* */
	{}
};


static struct cmd_tree_s root_fw_log[] = {
	{"help", isp_cmd_help, root_fw_log, "This help"},
	/* */
	{"threshold", isp_cmd_fw_log_threshold, NULL, "0..7"},
	{"force-switch", isp_cmd_fw_log_force_switch, NULL, "0 or 1"},
	/* */
	{}
};

static struct cmd_tree_s root_isp[] = {
	{"help", isp_cmd_help, root_isp, "This help"},
	/* */
	{"dump", isp_cmd_isp_dump, NULL,
		"Dump registers to " DEFAULT_DUMP_PATH},
	{"get", isp_cmd_isp_get, NULL, "get ADDRESS"},
	{"set", isp_cmd_isp_set, NULL, "set ADDRESS VALUE"},
	/* */
	{}
};

static struct cmd_tree_s root_vmem[] = {
	{"help", isp_cmd_help, root_vmem, "This help"},
	/* */
	{"dump", isp_cmd_vmem_dump, NULL,
		"dump firmware (to " DEFAULT_DUMP_PATH ")"},
	{"dump", isp_cmd_vmem_dump, NULL,
		"dump irpbin (to " DEFAULT_DUMP_PATH ")"},
	{"dump", isp_cmd_vmem_dump, NULL,
		"dump shadingbin (to " DEFAULT_DUMP_PATH ")"},
	{"dump", isp_cmd_vmem_dump, NULL,
		"dump START_ADDRESS LENGTH FULL_PATH"},
	/* */
	{}
};

static struct cmd_tree_s root_pmem[] = {
	{"help", isp_cmd_help, root_pmem, "This help"},
	/* */
	{"dump", isp_cmd_pmem_dump, NULL,
		"dump START_ADDRESS LENGTH FULL_PATH"},
	/* */
	{}
};

static struct cmd_tree_s root_bist[] = {
	{"help", isp_cmd_help, root_bist, "This help"},
	/* */
	{"set", isp_cmd_subtree, root_bist_set, "BIST sets ..."},
	{"load", isp_cmd_bist_load_variables, NULL, "BIST load variables ..."},
	/* */
	{}

};

static struct cmd_tree_s root_bist_set[] = {
	{"help", isp_cmd_help, root_bist_set, "This help"},
	/* */
	{"bist-addr", isp_cmd_bist_set_address, NULL, "bist address ..."},
	{"test-case-number", isp_cmd_bist_set_test_case_number, NULL,
		"Accumulate the number of test case ..."},
	{"scenario-info", isp_cmd_subtree, root_bist_set_scenario_info,
		"Scenario info ..."},
	{"output-buff-format", isp_cmd_subtree,
		root_bist_set_output_buff_format,
		"output buff format ..."},
	{"raw-frame-rate", isp_cmd_bist_set_raw_frame_rate, NULL, "1..4"},
	{"iso-speed", isp_cmd_bist_set_iso_speed, NULL, "100..1000"},
	{"independent-setting-order",
		isp_cmd_bist_set_independent_setting_order,
		NULL,
		"Indicate the order in independent mode ..."},
	{"bist-info", isp_cmd_subtree, root_bist_set_bist_info,
		"bist info ..."},
	{"golden-name", isp_cmd_bist_set_golden_name, NULL, "golden name ..."},
	{"ss-opt", isp_cmd_subtree, root_bist_set_ss_opt, "ss hq opt ..."},
	{"debug-mode", isp_cmd_bist_set_debug_mode, NULL, "debug mode ..."},
	{"mem-info", isp_cmd_subtree, root_bist_set_mem_info, "mem-info ..."},
	/* */
	{}
};

static struct cmd_tree_s root_bist_set_scenario_info[] = {
	{"help", isp_cmd_help, root_bist_set_scenario_info, "This help"},
	/* */
	{"sensor-info", isp_cmd_bist_set_scenario_info_sensor_info, NULL,
		"sensor info ..."},
	{"bypass-flag", isp_cmd_bist_set_scenario_info_bypass_flag, NULL,
		"bypass flag ..."},
	{"bayerscl-info", isp_cmd_bist_set_scenario_info_bayerscl_info, NULL,
		"bayer scalar info ..."},
	{"iq-param-idx", isp_cmd_bist_set_scenario_info_iq_param_idx, NULL,
		"iq parameter index ..."},
	/* */
	{}
};

static struct cmd_tree_s root_bist_set_output_buff_format[] = {
	{"help", isp_cmd_help, root_bist_set_output_buff_format, "This help"},
	/* */
	{"lv", isp_cmd_bist_set_lv_output_buff_format, NULL,
		"LV output buffer format ..."},
	{"video", isp_cmd_bist_set_video_output_buff_format, NULL,
		"Video output buffer format ..."},
	{"still", isp_cmd_bist_set_still_output_buff_format, NULL,
		"Still output buffer format ..."},
	{"metadata", isp_cmd_bist_set_metadata_output_buff_format, NULL,
		"Metadata output buffer format ..."},
	{"metadata_of_af", isp_cmd_bist_set_metadata_of_af_output_buff_format,
		NULL, "Metadata of AF output buffer format ..."},
	{"raw", isp_cmd_bist_set_raw_output_buff_format, NULL,
		"Raw output buffer format ..."},
	/* */
	{}
};

static struct cmd_tree_s root_bist_set_bist_info[] = {
	{"help", isp_cmd_help, root_bist_set_bist_info, "This help"},
	/* */
	{"scenario-id", isp_cmd_bist_info_set_scenario_id, NULL,
		"scenario id ..."},
	{"max-frame-count", isp_cmd_bist_info_set_max_frame_count, NULL,
		"max frame count ..."},
	{"independent-flag", isp_cmd_bist_info_set_independent_flag, NULL,
		"independent flag ..."},
	/* */
	{}
};

static struct cmd_tree_s root_bist_set_ss_opt[] = {
	{"help", isp_cmd_help, root_bist_set_ss_opt, "This help"},
	/* */
	{"writing-mode", isp_cmd_bist_set_ss_opt_writing_mode, NULL,
		"writing mode ..."},
	/* */
	{}
};

static struct cmd_tree_s root_bist_set_mem_info[] = {
	{"help", isp_cmd_help, root_bist_set_mem_info, "This help"},
	/* */
	{"hq-buffer-size", isp_cmd_bist_set_hq_buffer_size, NULL,
		"hq buffer size"},
	{"raw-buffer-size", isp_cmd_bist_set_raw_buffer_size, NULL,
		"raw buffer size"},
	/* */
	{}
};

static ssize_t isp_cmd_bist_load_variables(char *cmd, struct cmd_tree_s *tree)
{
	isp_cmd_bist_load_variable_setting();
	return 0;
}
static ssize_t isp_cmd_bist_set_address(char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u32 value;

	last = cmd;

	if (expected_newline(&last)) {
		isp_dev_append_response(
			"[%s] not parameter : %s\n", __func__, cmd);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	value = strtoul(start, &start, 0);

	isp_cmd_bist_set_bist_address_setting(value);
	isp_dev_append_response(
		"[%s] set bist address : 0x%x\n",
		__func__, value);

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;

}

static ssize_t isp_cmd_bist_set_test_case_number(
	char *cmd, struct cmd_tree_s *tree)
{
	isp_cmd_bist_set_test_case_number_setting();
	return 0;
}

/*echo bist set raw-frame-rate 1>/dev/isp_dev*/
static ssize_t isp_cmd_bist_set_raw_frame_rate(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u32 value;

	last = cmd;
	if (expected_newline(&last)) {
		isp_dev_append_response(
			"[%s] not parameter : %s\n",
			__func__, cmd);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	value = strtoul(start, &start, 0);

	if (value >= ISP_RAW_FR_OFF && value < ISP_RAW_FR_MAX) {
		isp_cmd_bist_set_raw_frame_rate_setting(value);
		isp_dev_append_response(
			"[%s] set raw frame rate:%d\n",
			__func__, value);
	} else
		isp_dev_append_response(
			"[%s] out of range:%d\n",
			__func__, value);

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

/*echo bist set iso-speed 100>/dev/isp_dev*/
static ssize_t isp_cmd_bist_set_iso_speed(char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u32 value;

	last = cmd;
	if (expected_newline(&last)) {
		isp_dev_append_response(
			"[%s] not parameter : %s\n",
			__func__, cmd);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	value = strtoul(start, &start, 0);

	if (value >= 0 && value <= 1000) {
		isp_cmd_bist_set_iso_speed_setting(value);
		isp_dev_append_response(
			"[%s] set iso speed:%d\n",
			__func__, value);
	} else
		isp_dev_append_response(
			"[%s] out of range:%d\n", __func__, value);

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_bist_set_independent_setting_order(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u32 value;

	last = cmd;
	if (expected_newline(&last)) {
		isp_dev_append_response(
			"[%s] not parameter : %s\n",
			__func__, cmd);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	value = strtoul(start, &start, 0);

	isp_cmd_bist_set_independent_order_setting((u8)value);
	isp_dev_append_response(
		"[%s] set independent setting order:%d\n",
		__func__, value);

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static char *g_golden_info_entry[] = {
	"lv",
	"video",
	"still",
};
static u8 g_golden_info_entry_size = 3;

/* echo bist set golden-name lv a.raw > /dev/isp_dev */
static ssize_t isp_cmd_bist_set_golden_name(char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u8 i = 0;

	last = cmd;
	/* 1. parse first token as an entry */
	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	for (i = 0; i < g_golden_info_entry_size; i++) {
		if (strncasecmp(start, g_golden_info_entry[i], len) != 0)
			continue;

		if (expected_newline(&last)) {
			isp_dev_append_response(
				"[%s] not parameter : %s\n",
				__func__, cmd);
			goto end_ok;
		}

		/* 2. parse the second token as a value */
		start = expected_token(&len, &last);
		if (!start)
			goto end_with_unknown_command;

		isp_cmd_bist_set_golden_name_setting(i, start, len);
		isp_dev_append_response("[%s] set golden name : %s %s\n",
			__func__, g_golden_info_entry[i], start);
	}

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}


static ssize_t isp_cmd_bist_set_debug_mode(char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u32 value;

	last = cmd;
	if (expected_newline(&last)) {
		isp_dev_append_response(
			"[%s] not parameter : %s\n",
			__func__, cmd);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	value = strtoul(start, &start, 0);

	isp_cmd_bist_set_debug_mode_setting(value);
	isp_dev_append_response("[%s] set debug mode:%d\n", __func__, value);

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}


static char *g_sensor_info_entry[] = {
	"sensor-mode",
	"sensor-module-type",
	"original-width",
	"original-height",
	"crop-start-x",
	"crop-start-y",
	"crop-end-x",
	"crop-end-y",
	"width",
	"height",
	"frame-rate",
	"line-time",
	"n-color-order",
	"clamp-level",
	"format",
	"bit-number",
	"mirror-flip",
	"cbc-enable",
};
static u8 g_sensor_info_entry_size = 18;

/* echo bist set sensor-info width 1920>/dev/isp_dev */
static ssize_t isp_cmd_bist_set_scenario_info_sensor_info(char *cmd,
	struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u8 i = 0;
	u32 value;

	last = cmd;
	/* 1. parse first token as an entry */
	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	/* 2. check the entry in the pre-defined array */
	for (i = 0; i < g_sensor_info_entry_size; i++) {
		if (strncasecmp(start, g_sensor_info_entry[i], len) != 0)
			continue;

		if (expected_newline(&last)) {
			isp_dev_append_response(
				"[%s] not parameter : %s\n", __func__, cmd);
			goto end_ok;
		}

		/* 3. parse the second token as a value */
		start = expected_token(&len, &last);
		if (!start)
			goto end_with_unknown_command;
		value = strtoul(start, &start, 0);

		isp_cmd_bist_set_scenario_info_sensor_info_setting(i, value);
		isp_dev_append_response(
			"[%s] set scenario-info sensor-info %s %d\n",
			__func__, g_sensor_info_entry[i], value);

	}


end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static char *g_image_type_entry[] = {
	"lv",
	"video",
	"still",
	"metadata",
	"raw",
};
static u8 g_image_type_entry_size = 5;
static ssize_t isp_cmd_bist_set_scenario_info_bypass_flag(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u8 i = 0;
	u32 value;

	last = cmd;
	/* 1. parse first token as an entry */
	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	/* 2. check the entry in the pre-defined array */
	for (i = 0; i < g_image_type_entry_size; i++) {
		if (strncasecmp(start, g_image_type_entry[i], len) != 0)
			continue;

		if (expected_newline(&last)) {
			isp_dev_append_response(
				"[%s] not parameter : %s\n", __func__, cmd);
			goto end_ok;
		}

		/* 3. parse the second token as a value */
		start = expected_token(&len, &last);
		if (!start)
			goto end_with_unknown_command;
		value = strtoul(start, &start, 0);

		isp_cmd_bist_set_scenario_info_bypass_flag_setting(i, value);
		isp_dev_append_response(
			"[%s] set scenario-info bypass-flag %s %d\n",
			__func__, g_image_type_entry[i], value);
	}

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}


static char *g_size_info_entry[] = {
	"width",
	"height",
};
static u8 g_size_info_entry_size = 2;

static ssize_t isp_cmd_bist_set_scenario_info_bayerscl_info(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u8 i = 0;
	u32 value;

	last = cmd;
	/* 1. parse first token as an entry */
	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	/* 2. check the entry in the pre-defined array */
	for (i = 0; i < g_size_info_entry_size; i++) {
		if (strncasecmp(start, g_size_info_entry[i], len) != 0)
			continue;

		if (expected_newline(&last)) {
			isp_dev_append_response(
				"[%s] not parameter : %s\n", __func__, cmd);
			goto end_ok;
		}

		/* 3. parse the second token as a value */
		start = expected_token(&len, &last);
		if (!start)
			goto end_with_unknown_command;
		value = strtoul(start, &start, 0);

		isp_cmd_bist_set_scenario_info_bayerscl_info_setting(i, value);
		isp_dev_append_response(
			"[%s] set scenario-info bayerscl-info :%s %d\n",
			__func__, g_size_info_entry[i], value);
	}

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_bist_set_scenario_info_iq_param_idx(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u8 i = 0;
	u32 value;

	last = cmd;
	/* 1. parse first token as an entry */
	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	/* 2. check the entry in the pre-defined array */
	for (i = 0; i <= 2; i++) {
		if (strncasecmp(start, g_image_type_entry[i], len) != 0)
			continue;

		if (expected_newline(&last)) {
			isp_dev_append_response(
				"[%s] not parameter : %s\n", __func__, cmd);
			goto end_ok;
		}

		/* 3. parse the second token as a value */
		start = expected_token(&len, &last);
		if (!start)
			goto end_with_unknown_command;
		value = strtoul(start, &start, 0);

		isp_cmd_bist_set_scenario_info_iq_param_idx_setting(i, value);
		isp_dev_append_response(
			"[%s] set scenario-info iq-param-idx %s %d\n",
			__func__, g_image_type_entry[i], value);
	}

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}



static char *g_output_buff_format_entry[] = {
	"enable",
	"format",
	"width",
	"height",
	"buffer-number",
	"line-offset",
};
static u8 g_output_buff_format_entry_size = 6;
static ssize_t isp_cmd_bist_set_lv_output_buff_format(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u8 i = 0;
	u32 value;

	last = cmd;
	/* 1. parse first token as an entry */
	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	/* 2. check the entry in the pre-defined array */
	for (i = 0; i < g_output_buff_format_entry_size; i++) {
		if (strncasecmp(start, g_output_buff_format_entry[i],
			len) != 0)
			continue;

		if (expected_newline(&last)) {
			isp_dev_append_response(
				"[%s] not parameter : %s\n", __func__, cmd);
			goto end_ok;
		}

		/* 3. parse the second token as a value */
		start = expected_token(&len, &last);
		if (!start)
			goto end_with_unknown_command;
		value = strtoul(start, &start, 0);

		isp_cmd_bist_set_output_buff_format_lv_setting(i, value);
		isp_dev_append_response(
			"[%s] set lv output buff format %s %d\n",
			__func__, g_output_buff_format_entry[i], value);
	}

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_bist_set_video_output_buff_format(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u8 i = 0;
	u32 value;

	last = cmd;
	/* 1. parse first token as an entry */
	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	/* 2. check the entry in the pre-defined array */
	for (i = 0; i < g_output_buff_format_entry_size; i++) {
		if (strncasecmp(start, g_output_buff_format_entry[i],
			len) != 0)
			continue;

		if (expected_newline(&last)) {
			isp_dev_append_response(
				"[%s] not parameter : %s\n", __func__, cmd);
			goto end_ok;
		}

		/* 3. parse the second token as a value */
		start = expected_token(&len, &last);
		if (!start)
			goto end_with_unknown_command;
		value = strtoul(start, &start, 0);

		isp_cmd_bist_set_output_buff_format_video_setting(i, value);
		isp_dev_append_response(
			"[%s] set video output buff format %s %d\n",
			__func__, g_output_buff_format_entry[i], value);
	}

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_bist_set_still_output_buff_format(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u8 i = 0;
	u32 value;

	last = cmd;
	/* 1. parse first token as an entry */
	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	/* 2. check the entry in the pre-defined array */
	for (i = 0; i < g_output_buff_format_entry_size; i++) {
		if (strncasecmp(start, g_output_buff_format_entry[i],
			len) != 0)
			continue;

		if (expected_newline(&last)) {
			isp_dev_append_response(
				"[%s] not parameter : %s\n", __func__, cmd);
			goto end_ok;
		}

		/* 3. parse the second token as a value */
		start = expected_token(&len, &last);
		if (!start)
			goto end_with_unknown_command;
		value = strtoul(start, &start, 0);

		isp_cmd_bist_set_output_buff_format_still_setting(i, value);
		isp_dev_append_response(
			"[%s] set still output buff format :%s %d\n",
			__func__, g_output_buff_format_entry[i], value);
	}

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_bist_set_metadata_output_buff_format(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u8 i = 0;
	u32 value;

	last = cmd;
	/* 1. parse first token as an entry */
	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	/* 2. check the entry in the pre-defined array */
	for (i = 0; i < g_output_buff_format_entry_size; i++) {
		if (strncasecmp(start, g_output_buff_format_entry[i],
			len) != 0)
			continue;

		if (expected_newline(&last)) {
			isp_dev_append_response(
				"[%s] not parameter : %s\n", __func__, cmd);
			goto end_ok;
		}

		/* 3. parse the second token as a value */
		start = expected_token(&len, &last);
		if (!start)
			goto end_with_unknown_command;
		value = strtoul(start, &start, 0);

		isp_cmd_bist_set_output_buff_format_metadata_setting(i, value);
		isp_dev_append_response(
			"[%s] set metadata output buff format :%s %d\n",
			__func__, g_output_buff_format_entry[i], value);
	}

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_bist_set_metadata_of_af_output_buff_format(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u8 i = 0;
	u32 value;

	last = cmd;
	/* 1. parse first token as an entry */
	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	/* 2. check the entry in the pre-defined array */
	for (i = 0; i < g_output_buff_format_entry_size; i++) {
		if (strncasecmp(start, g_output_buff_format_entry[i],
			len) != 0)
			continue;

		if (expected_newline(&last)) {
			isp_dev_append_response(
				"[%s] not parameter : %s\n", __func__, cmd);
			goto end_ok;
		}

		/* 3. parse the second token as a value */
		start = expected_token(&len, &last);
		if (!start)
			goto end_with_unknown_command;
		value = strtoul(start, &start, 0);

		isp_cmd_bist_set_output_buff_format_metadata_of_af_setting(i,
			value);
		isp_dev_append_response(
			"[%s] set metadata output buff format :%s %d\n",
			__func__, g_output_buff_format_entry[i], value);
	}

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}


static ssize_t isp_cmd_bist_set_raw_output_buff_format(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u8 i = 0;
	u32 value;

	last = cmd;
	/* 1. parse first token as an entry */
	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	/* 2. check the entry in the pre-defined array */
	for (i = 0; i < g_output_buff_format_entry_size; i++) {
		if (strncasecmp(start, g_output_buff_format_entry[i],
			len) != 0)
			continue;

		if (expected_newline(&last)) {
			isp_dev_append_response(
				"[%s] not parameter : %s\n", __func__, cmd);
			goto end_ok;
		}

		/* 3. parse the second token as a value */
		start = expected_token(&len, &last);
		if (!start)
			goto end_with_unknown_command;
		value = strtoul(start, &start, 0);

		isp_cmd_bist_set_output_buff_format_raw_setting(i, value);
		isp_dev_append_response(
			"[%s] set raw output buff format %s %d\n",
			__func__, g_output_buff_format_entry[i], value);
	}

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_bist_info_set_scenario_id(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u32 value;

	last = cmd;

	if (expected_newline(&last)) {
		isp_dev_append_response(
			"[%s] not parameter : %s\n", __func__, cmd);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	value = strtoul(start, &start, 0);

	isp_cmd_bist_set_bist_info_scenario_id_setting((u8)value);
	isp_dev_append_response("[%s] set scenario id : %d\n",
		__func__, value);

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_bist_info_set_max_frame_count(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u32 value;

	last = cmd;

	if (expected_newline(&last)) {
		isp_dev_append_response(
			"[%s] not parameter : %s\n", __func__, cmd);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	value = strtoul(start, &start, 0);

	isp_cmd_bist_set_bist_info_max_frame_count_setting((u8)value);
	isp_dev_append_response("[%s] set max frame count : %d\n",
		__func__, value);

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_bist_info_set_independent_flag(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u32 value;

	last = cmd;

	if (expected_newline(&last)) {
		isp_dev_append_response(
			"[%s] not parameter : %s\n", __func__, cmd);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	value = strtoul(start, &start, 0);

	isp_cmd_bist_set_bist_info_independent_flag_setting((u8)value);
	isp_dev_append_response(
		"[%s] set independent flag : %d\n", __func__, value);

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_bist_set_ss_opt_writing_mode(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u32 value;

	last = cmd;

	if (expected_newline(&last)) {
		isp_dev_append_response(
			"[%s] not parameter : %s\n", __func__, cmd);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	value = strtoul(start, &start, 0);

	isp_cmd_bist_set_writing_mode(value);
	isp_dev_append_response(
		"[%s] set writing mode : %d\n", __func__, value);

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}


static ssize_t isp_cmd_bist_set_hq_buffer_size(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u32 value;

	last = cmd;

	if (expected_newline(&last)) {
		isp_dev_append_response(
			"[%s] not parameter : %s\n", __func__, cmd);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	value = strtoul(start, &start, 0);

	isp_cmd_bist_set_hq_buff_size_setting(value);
	isp_dev_append_response(
		"[%s] set hq buffer size : 0x%x\n",
		__func__, value);

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_bist_set_raw_buffer_size(
	char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;
	u32 value;

	last = cmd;

	if (expected_newline(&last)) {
		isp_dev_append_response(
			"[%s] not parameter : %s\n", __func__, cmd);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	value = strtoul(start, &start, 0);

	isp_cmd_bist_set_raw_buff_size_setting(value);
	isp_dev_append_response(
		"[%s] set hq buffer size : %d\n",
		__func__, value);

end_ok:__attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

/*****************************************************************************/

static u64 strtoull(const char *cp, char **endp, u32 base)
{
	u64 result;
	u32 rv;

	cp = _parse_integer_fixup_radix(cp, &base);
	rv = _parse_integer(cp, base, &result);
	cp += (rv & ~KSTRTOX_OVERFLOW);

	if (endp)
		*endp = (char *)cp;

	return result;
}

static u32 strtoul(const char *cp, char **endp, u32 base)
{
	return strtoull(cp, endp, base);
}

/*****************************************************************************/

static char *expected_token(u32 *len, char **end)
{
	char *start = *end;

	if (!len || !end || !start || !*start)
		return NULL;

	while (isspace(*start)) { /* skip leading white space */
		if (!*++start) {
			*end = start;
			return NULL;
		}
	}

	for (*end = start + 1 ; **end && !isspace(**end) ; ++*end)
		; /* till next white space */
	*len = *end - start;

	return start;
}

static char *expected_newline(char **end)
{
	char *start = *end;

	if (!end || !start || !*start)
		return NULL;

	while (isspace2(*start)) { /* skip leading white space except CR/LF */
		if (!*++start) {
			*end = start;
			return NULL;
		}
	}

	if (!iscrlf(*start))
		return NULL;

	*end = start + 1;

	return start;
}

/*****************************************************************************/

static ssize_t isp_cmd_subtree(char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;

	last = cmd;
	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	for (; tree->token && *tree->token ; tree++) {
		if (strncasecmp(start, tree->token, len) != 0)
			continue;
		if (tree->action(last, tree->sub) >= 0)
			goto end_ok;
		goto end_with_unknown_command;
	}
	if (!tree->token)
		goto end_with_unknown_command;

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_help(char *cmd, struct cmd_tree_s *tree)
{
	isp_dev_append_response("help\n");
	for (; tree->token && *tree->token ; tree++)
		isp_dev_append_response_raw("%-16s%s\n", tree->token,
					    tree->help);

	return 0;
}

static ssize_t isp_cmd_ahb_dump(char *cmd, struct cmd_tree_s *tree)
{
	if (expected_newline(&last)) {
		ispdrv_dump_ahb_reg();
		isp_dev_append_response("ahb dump: OK\n");
	} else {
		/* TODO: user-defined file path */
		goto end_with_unknown_command;
	}

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_ahb_get(char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len, ofs, value;

	last = cmd;

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	ofs = strtoul(start, &start, 0);
	if (start != last)
		goto end_with_unknown_command;

	if (!expected_newline(&last))
		goto end_with_unknown_command;

	value = 0; /* TODO: to get AHB register value */

	isp_dev_append_response("ahb get 0x%08x: 0x%08x\n",
				ofs, value);

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_ahb_set(char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len, ofs, value;

	last = cmd;

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	ofs = strtoul(start, &start, 0);
	if (start != last)
		goto end_with_unknown_command;

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	value = strtoul(start, &start, 0);
	if (start != last)
		goto end_with_unknown_command;

	if (!expected_newline(&last))
		goto end_with_unknown_command;

	/* TODO: to set AHB register value */

	isp_dev_append_response("ahb set 0x%08x 0x%08x: TODO\n", ofs, value);

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_drv_auto_dump(char *cmd, struct cmd_tree_s *tree)
{
	char *start, **s;
	u32 len;
	u32 i;

	last = cmd;

	if (expected_newline(&last)) {
		/* TODO: to get status */
		isp_dev_append_response("drv auto-dump: %s\n",
					str_auto_dump[auto_dump]);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	if (!expected_newline(&last))
		goto end_with_unknown_command;

	for (i = 0, s = str_auto_dump; *s && **s ; i++, s++) {
		if (strncasecmp(start, *s, len) != 0)
			continue;
		auto_dump = i;
		break;
	}
	if (!s || !*s)
		goto end_with_unknown_command;

	isp_dev_append_response("drv auto-dump %s: OK\n", *s);

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_drv_bist(char *cmd, struct cmd_tree_s *tree)
{
	isp_info_lo_start_tag();
	isp_info_item_tag("debug #%u", __LINE__);
	isp_info_lo_end_tag();
	return 0;
}

static ssize_t isp_cmd_drv_log_threshold(char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;

	last = cmd;

	if (expected_newline(&last)) {
		/* TODO: to get ISP driver log threshold */
		isp_dev_append_response("drv log threshold: %u\n",
					drvlog_threshold);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	drvlog_threshold = strtoul(start, &start, 0);
	if (start != last)
		goto end_with_unknown_command;
	if (!expected_newline(&last))
		goto end_with_unknown_command;

	if (drvlog_threshold > 7)
		drvlog_threshold = 7; /* TODO: should define a macro */

	isp_dev_append_response("drv log threshold %u: OK\n", drvlog_threshold);

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_drv_version(char *cmd, struct cmd_tree_s *tree)
{
	if (!expected_newline(&last))
		goto end_with_unknown_command;

	isp_dev_append_response("drv version: " VERSION "\n");

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_fw_dump(char *cmd, struct cmd_tree_s *tree)
{
	if (!expected_newline(&last))
		goto end_with_unknown_command;

	ispdrv_dump_firmware_memory();

	isp_dev_append_response("fw dump: OK\n");

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_fw_log_threshold(char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;

	last = cmd;

	if (expected_newline(&last)) {
		/* TODO: to get ISP firmware log threshold */
		isp_dev_append_response("fw log threshold: %u\n",
					fwlog_threshold);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	fwlog_threshold = strtoul(start, &start, 0);
	if (start != last)
		goto end_with_unknown_command;
	if (!expected_newline(&last))
		goto end_with_unknown_command;

	if (fwlog_threshold > 7)
		fwlog_threshold = 7; /* TODO: should define a macro */

#if 1 /* FIXME: temporary *//* see log_task_func() in altek_isp_drv.c */
	if (fwlog_threshold == 7)
		set_isplog_flag(1);
	else
		set_isplog_flag(0);
#endif

	/* TODO: change ISP firmware log threshold */

	isp_dev_append_response("fw log threshold %u: OK\n", fwlog_threshold);

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_fw_log_force_switch(char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len;

	last = cmd;

	if (expected_newline(&last)) {
		/* TODO: to get ISP firmware log force-switch */
		isp_dev_append_response("fw log force-switch: %u\n",
					fwlog_force_switch);
		goto end_ok;
	}

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	fwlog_force_switch = strtoul(start, &start, 0);
	if (start != last)
		goto end_with_unknown_command;
	if (!expected_newline(&last))
		goto end_with_unknown_command;

	if (fwlog_force_switch > 1)
		fwlog_force_switch = 1; /* TODO: should define a macro */

	isp_dev_append_response("fw log force-switch %u: OK\n",
				fwlog_force_switch);

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_fw_version(char *cmd, struct cmd_tree_s *tree)
{
	isp_info_lo_start_tag();
	isp_info_item_tag("debug #%u", __LINE__);
	isp_info_lo_end_tag();
	return 0;
}

static ssize_t isp_cmd_isp_get(char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len, addr, value;

	last = cmd;

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	addr = strtoul(start, &start, 0);
	if (start != last)
		goto end_with_unknown_command;

	if (!expected_newline(&last))
		goto end_with_unknown_command;

	value = get_ahb_indirect(addr);

	isp_dev_append_response("isp get 0x%08x => 0x%08x\n",
				addr, value);

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_isp_set(char *cmd, struct cmd_tree_s *tree)
{
	char *start;
	u32 len, addr, value;

	last = cmd;

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	addr = strtoul(start, &start, 0);
	if (start != last)
		goto end_with_unknown_command;

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	value = strtoul(start, &start, 0);
	if (start != last)
		goto end_with_unknown_command;

	if (!expected_newline(&last))
		goto end_with_unknown_command;

	set_ahb_indirect(addr, value);

	isp_dev_append_response("isp set 0x%08x 0x%08x\n", addr, value);

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_isp_dump(char *cmd, struct cmd_tree_s *tree)
{
	if (expected_newline(&last)) {
		ispdrv_dump_isp_internal_reg();
		isp_dev_append_response("isp dump: OK\n");
	} else {
		/* TODO: user-defined file path */
		goto end_with_unknown_command;
	}

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_vmem_dump(char *cmd, struct cmd_tree_s *tree)
{
	char *start, *from, *path;
	u32 len;
	u64 range;

	last = cmd;

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;

	if (strncasecmp(start, "firmware", len) == 0 ||
	    strncasecmp(start, "fw", len) == 0) {
		if (expected_newline(&last)) {
			ispdrv_dump_firmware_memory();
			isp_dev_append_response("vmem dump fw: OK\n");
		} else {
			/* TODO: user-defined file path */
			goto end_with_unknown_command;
		}
	} else if (strncasecmp(start, "irpbin", len) == 0) {
		if (expected_newline(&last)) {
			ispdrv_dump_irpbin();
			isp_dev_append_response("vmem dump ispbin: OK\n");
		} else {
			/* TODO: user-defined file path */
			goto end_with_unknown_command;
		}
	} else if (strncasecmp(start, "shadingbin", len) == 0) {
		if (expected_newline(&last)) {
			ispdrv_dump_shadingbin();
			isp_dev_append_response("vmem dump shadingbin: OK\n");
		} else {
			/* TODO: user-defined file path */
			goto end_with_unknown_command;
		}
	} else { /* User specified address & range */
		start = expected_token(&len, &last);
		if (!start)
			goto end_with_unknown_command;
		from = (char *)strtoull(start, &start, 0);
		if (start != last)
			goto end_with_unknown_command;

		start = expected_token(&len, &last);
		if (!start)
			goto end_with_unknown_command;
		range = strtoull(start, &start, 0);
		if (start != last)
			goto end_with_unknown_command;

		path = expected_token(&len, &last);
		if (!path)
			goto end_with_unknown_command;

		if (!expected_newline(&last))
			goto end_with_unknown_command;

		dump_kva(path, (u64 *)from, range);

		isp_dev_append_response("vmem dump 0x%p %llu \"%s\": OK\n",
					from, range, path);
	}

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

static ssize_t isp_cmd_pmem_dump(char *cmd, struct cmd_tree_s *tree)
{
	char *start, *from, *path;
	u32 len;
	u64 range;

	last = cmd;

	/* User specified address & range */
	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	from = (char *)strtoull(start, &start, 0);
	if (start != last)
		goto end_with_unknown_command;

	start = expected_token(&len, &last);
	if (!start)
		goto end_with_unknown_command;
	range = strtoull(start, &start, 0);
	if (start != last)
		goto end_with_unknown_command;

	path = expected_token(&len, &last);
	if (!path)
		goto end_with_unknown_command;

	if (!expected_newline(&last))
		goto end_with_unknown_command;

	/* TODO: dump physical memory */
	/*dump_physical_memory(path, (u64)from, range);*/

	isp_dev_append_response("pmem dump 0x%p %llu \"%s\": TODO\n",
				from, range, path);

end_ok: __attribute__((unused));
	return last - cmd;

end_with_unknown_command: __attribute__((unused));
	return cmd - last;
}

ssize_t isp_cmd(char *cmd)
{
	char *start;
	u32 len;
	ssize_t total = strlen(cmd);
	struct cmd_tree_s *tree;

	last = cmd;

	while (*last) {
		start = expected_token(&len, &last);
		if (!start)
			break;
		for (tree = root; tree->token && *tree->token ; tree++) {
			if (strncasecmp(start, tree->token, len) != 0)
				continue;
			if (tree->action(last, tree->sub) >= 0)
				break; /* Do next root command */
			goto end_with_unknown_command;
		}
		if (!tree->token)
			break; /* Unknown command or trailer garbage */
	}

	if (!*last)
		goto end_ok; /* Ignore trailer garbage */

end_with_unknown_command: __attribute__((unused));
	isp_dev_append_response("Unknown command at %lu/%u in\n"
				"======\n%s\n======\n", last - cmd, total, cmd);

end_ok: __attribute__((unused));
	return total;
}
