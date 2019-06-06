/*
* File: altek_bist.c							    *
* Description: implementation of bist api				    *
*									    *
* (C)Copyright altek Corporation 2016					    *
*									    *
* History:								    *
*   2016/03/012; Caedmon Lai; Initial version				    *
*/
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <asm/cacheflush.h>

#include "altek_ahb_drv.h"
#include "altek_ap3a_info.h"
#include "altek_isp_drv.h"
#include "altek_sensor_info.h"
#include "altek_common.h"
#include "altek_isp_drv_local.h"
#include "altek_iq_info.h"
#include "altek_bist.h"
#include "altek_dump_utility.h"
#include "altek_log_local.h"

static u64 g_bist_ahb_addr;
static u32 g_bist_ahb_size;
static void *g_bist_v4l2_device;
static u8 g_bist_scenario_id;
/* BIST*/
static u32 g_bist_1stsensor_img1_iva,
g_bist_1stsensor_img2_iva,
g_bist_1stsensor_img3_iva,
g_bist_1stsensor_statistics_iva,
g_bist_1stsensor_raw_img_iva,
g_bist_2ndsensor_img1_iva,
g_bist_2ndsensor_img2_iva,
g_bist_2ndsensor_img3_iva,
g_bist_2ndsensor_statistics_iva,
g_bist_2ndsensor_raw_img_iva;

static u32 g_bist_buff_iva;

static u64 g_bist_buff_kva,
g_bist_direct_kva,
g_bist_dram_lv_video_kva,
g_bist_dram_normal_quality_still_kva,
g_bist_dram_high_quality_still_kva,
g_bist_front_kva,
g_bist_proc_still_kva;

static u8 g_bist_proc_still_start[2];
static u8 g_bist_proc_still_end[2];
static u8 g_bist_proc_still_fw_start[2];

static u32 g_bist_proc_still_src_raw_iva[2];
static struct ispdrv_output_addr_still g_bist_proc_still_iva[2];
static struct ispdrv_output_addr_still g_bist_output_still_iva[2];
static struct s_scenario_size g_ss_raw_crop_size[2];
static struct s_scenario_size g_ss_still_crop_size[2];
static struct s_iq_info_1 g_ss_raw_iq_info_1[2];
static struct s_iq_info_1 g_ss_still_iq_info_1[2];
static struct s_iq_info_2 g_ss_raw_iq_info_2[2];
static struct s_iq_info_2 g_ss_still_iq_info_2[2];
static struct ispdrv_proc_still_opt g_ss_opt[2];
/* static struct s_raw_info_setting g_ss_raw_info_setting[2]; */

static u8 g_is_selftest_addr_set;

/* Self Test variables for 1st sensor and 2nd sensor*/
static u8 g_bist_1stsensor_id;
static u8 g_bist_2ndsensor_id;
static u16 g_bist_1stsensor_input_width;
static u16 g_bist_1stsensor_input_height;
static u16 g_bist_2ndsensor_input_width;
static u16 g_bist_2ndsensor_input_height;
static bool g_is_independent_mode;
static bool g_is_reverse_close_sequence;

/* static variables for build-in self test*/
static enum ispdrv_scenario_mode g_self_test_scenario[ISP_SELF_TEST_TOTAL] = {
	V4L2_ISP_SCENARIO_PREVIEW_SENSOR_1,
	V4L2_ISP_SCENARIO_PREVIEW_STRIPE,
	V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT,
	V4L2_ISP_SCENARIO_PREVIEW_SENSOR_LITE,
	V4L2_ISP_SCENARIO_PREVIEW_STILL_SS,
	V4L2_ISP_SCENARIO_PREVIEW_SENSOR_2
};

/*********************** New BIST Variables *************************/
/* IRQ handling*/
#define BIST_IRQ_MASK	 \
	(V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT | \
	 V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT | \
	 V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT | \
	 V4L2_ISP_SENSOR1_STILL_PROC_START_INT | \
	 V4L2_ISP_SENSOR1_RAW_DONE_INT | \
	 V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT | \
	 V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT | \
	 V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT | \
	 V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT)

static u64 g_firmware_addr;
static struct s_scenario_info_isp *g_shared_scenario_info;

/* Global variable for testing all test cases*/
static u8 g_test_case_number;
static u32 g_image_memory_start_addr;
static u32 g_image_memory_offset;
static u8 g_current_test_case;
static struct s_scenario_info_ap g_scenario_info[
	MAX_TEST_CASE_NUMBER][MAX_USED_SENSOR_NUMBER];
static struct s_output_buff_format_info g_output_buff_format[
	MAX_TEST_CASE_NUMBER][MAX_USED_SENSOR_NUMBER];
static struct s_scinfo_self_test_info g_self_test_info[MAX_TEST_CASE_NUMBER];
static struct s_bist_info g_bist_info[MAX_TEST_CASE_NUMBER];
static enum ispdrv_raw_frame_rate g_raw_frame_rate[MAX_TEST_CASE_NUMBER][
	MAX_RAW_FR_RATE_CHANGE];
static u32 g_iso_speed[MAX_TEST_CASE_NUMBER];
static char *g_golden_name[MAX_TEST_CASE_NUMBER][MAX_USED_SENSOR_NUMBER][
	MAX_GOLDEN_IMG_NUMBER];
static enum ispdrv_sl_output_writing_mode g_ss_opt_writing_mode[
	MAX_TEST_CASE_NUMBER];
static u32 g_hq_buffer_size[MAX_TEST_CASE_NUMBER];
static u32 g_raw_buffer_size[MAX_TEST_CASE_NUMBER];

static struct output_addr_setting g_image_addr[MAX_USED_SENSOR_NUMBER][
	V4L2_ISP_OUTPUT_IMAGE_TOTAL];
static u32 g_golden_addr[MAX_USED_SENSOR_NUMBER][MAX_GOLDEN_IMG_NUMBER];
static bool g_is_compare[MAX_USED_SENSOR_NUMBER][MAX_GOLDEN_IMG_NUMBER];
static u32 g_reserve_raw_addr[2];
static u32 g_proc_still_raw_addr[2];

/* For dynamic multi-layer mode */
static struct s_iq_info_1 g_raw_iq_info_1[2];
static struct s_iq_info_2 g_raw_iq_info_2[2];
static struct ispdrv_proc_still_opt g_proc_still_opt[2];
static u8 g_still_index;
static u8 g_raw_index;
static u32 g_golden_sum, g_golden_xor, g_output_sum, g_output_xor;
static bool g_debug_mode;
/*********************** New BIST Variables ************************/



/*
 * ////////////////////////////////////////////////////
 *	Private Function Prototype		 *
 * ////////////////////////////////////////////////////
*/
#if 0
static void self_test_fill_parameter_ss(u8 image_type,
		    u8 proc_index, u32 image_addr,
		    struct s_scenario_size *img_size);
#endif
static void self_test_fill_ss_hq_opt_param(
	struct ispdrv_proc_still_opt *ss_opt);
static u32 self_test_proc_still(void);
static void self_test_check_sum(u64 addr, u32 size,
	u32 *result_sum, u32 *result_xor);
static void self_test_check_sum_ss(u64 addr, u32 size,
	const u32 image_width, const u32 image_height,
	u32 *result_sum, u32 *result_xor);

static u32 self_test_set_awb_gain_settings(u8 sensor_id);

static u32 self_test_set_3a_settings(u8 sensor_id, u16 sensor_height);

static u32 self_test_set_dld_sequence_settings(u8 sensor_id);

static u32 self_test_set_3a_cfg(u8 sensor_id, u8 scenario_id,
	u16 sensor_height);

static void self_test_activate_test_pattern(u8 sensor_id, u16 width,
	u16 height);

/************ New BIST Private Function Prototype *****************/
static void bist_load_firmware(void);

static bool meet_max_frame_count(u8 sensor_id,
	bool *bypass_flag);

static void self_test_lv_done(u32 irq_flag);
static void self_test_video_done(u32 irq_flag);
static void self_test_still_done(u32 irq_flag);
static void self_test_raw_done(u8 sensor_id, u8 image_type);
static u32 self_test_start_proc_still(void);
static void self_test_clear_global_variables(void);
static u8 self_test_get_sensor_id(
	struct s_bist_info *bist_info,
	u8 test_case_number, u8 order);
static void self_test_allocate_image_addr(u8 scenario_id,
	u8 image_type,
	struct output_addr_setting *output_addr,
	struct s_output_buff_format_info *output_buff_format,
	u32 *memory_start_addr,
	u32 *memory_offset);
static u32 self_test_load_qmerge(void);
static void self_test_load_golden_image(
	u32 *golden_addr,
	struct s_output_buff_format_info *output_buff_format,
	bool *bypass_flag,
	char **golden_name,
	bool *is_compare);
static void self_test_set_bypass_flag(u8 scenario_id,
	bool *bypass_flag,
	struct s_scinfo_out_bypassflg *out_bypass_flag);

static u64 self_test_get_kva(u32 iva);

/************ New BIST Private Function Prototype *****************/


/***********************************************************
*		bist api		  *
*************************************************************/
/*
*\isp memory configuration, t_his function must be called before ispdrv_open()
*\param iva_self_test_img1_addr[in], Self Test frame done address for IMG1
*\param iva_self_test_img2_addr[in], Self Test frame done address for IMG2
*\param iva_self_test_img3_addr[in], Self Test frame done address for IMG3
*\param iva_self_test_metadata_addr[in]
*\param , Self Test frame done address for Metadata
*\return null
*/
void ispdrv_set_self_test_addr(u32 bist_buffer_iva, u64 bist_buffer_kva)
{
	g_bist_buff_iva = bist_buffer_iva;
	g_bist_1stsensor_img1_iva = g_bist_buff_iva;

	isp_info_lo_start_tag();

	memset((u8 *) &g_bist_proc_still_iva[0], 0,
		sizeof(g_bist_proc_still_iva));
	memset((u8 *) &g_bist_output_still_iva[0], 0,
		sizeof(g_bist_output_still_iva));
	g_bist_1stsensor_raw_img_iva =
		g_bist_1stsensor_img1_iva + SS_LV_BUFFER_SIZE * 4;
	g_bist_proc_still_iva[0].buffer_addr =
		g_bist_1stsensor_raw_img_iva + SS_RAW_BUFFER_SIZE * 6;
	g_bist_proc_still_iva[1].buffer_addr =
		g_bist_proc_still_iva[0].buffer_addr + SS_STILL_BUFFER_SIZE;

	g_bist_1stsensor_img2_iva =
	g_bist_1stsensor_img1_iva + MAX_SELF_TEST_LV_SIZE;
	g_bist_1stsensor_img3_iva =
	g_bist_1stsensor_img2_iva + MAX_SELF_TEST_VIDEO_SIZE;
	g_bist_1stsensor_statistics_iva =
	g_bist_1stsensor_img3_iva + MAX_SELF_TEST_STILL_SIZE;
	g_bist_2ndsensor_img1_iva =
	g_bist_1stsensor_img1_iva +
	BUF_SIZE_ISP_SELF_TEST_SENSOR_FD_BUFF;
	g_bist_2ndsensor_img2_iva =
	g_bist_2ndsensor_img1_iva + MAX_SELF_TEST_LV_SIZE;
	g_bist_2ndsensor_img3_iva =
	g_bist_2ndsensor_img2_iva + MAX_SELF_TEST_VIDEO_SIZE;
	g_bist_2ndsensor_statistics_iva =
	g_bist_2ndsensor_img3_iva + MAX_SELF_TEST_STILL_SIZE;

	g_bist_buff_kva = bist_buffer_kva;


	if ((SS_LV_BUFFER_SIZE * 4 +
		SS_RAW_BUFFER_SIZE * 6 + SS_STILL_BUFFER_SIZE * 2) >
		(BUF_SIZE_ISP_SELF_TEST_SENSOR_FD_BUFF * 2))
		g_bist_direct_kva = g_bist_buff_kva +
					(SS_LV_BUFFER_SIZE * 4 +
					SS_RAW_BUFFER_SIZE * 6 +
					SS_STILL_BUFFER_SIZE * 2);
	else
		g_bist_direct_kva = g_bist_buff_kva +
			BUF_SIZE_ISP_SELF_TEST_SENSOR_FD_BUFF * 2;

	g_bist_dram_lv_video_kva =
		g_bist_direct_kva + MAX_SELF_TEST_GOLDEN_SIZE_960x720;
	g_bist_dram_normal_quality_still_kva =
		g_bist_dram_lv_video_kva + MAX_SELF_TEST_GOLDEN_SIZE_640x360;
	g_bist_dram_high_quality_still_kva =
		g_bist_dram_normal_quality_still_kva +
		MAX_SELF_TEST_GOLDEN_SIZE_1920x1080;
	g_bist_front_kva =
		g_bist_dram_high_quality_still_kva +
		MAX_SELF_TEST_GOLDEN_SIZE_1920x1080;
	g_bist_proc_still_kva = g_bist_front_kva +
		MAX_SELF_TEST_GOLDEN_SIZE_960x720;

	bist_set_self_test_addr_set();
	isp_info_item_tag("iva addr: 0x%08x\n", g_bist_buff_iva);
	isp_info_item_tag("1st_sensor_img1_addr 0x%x\n",
		g_bist_1stsensor_img1_iva);

	isp_info_item_tag("1st_sensor_raw_addr 0x%x\n",
		g_bist_1stsensor_raw_img_iva);
	isp_info_item_tag("1st_sensor_proc_still_addr 1 0x%x\n",
		g_bist_proc_still_iva[0].buffer_addr);
	isp_info_item_tag("1st_sensor_proc_still_addr 2 0x%x\n",
		g_bist_proc_still_iva[1].buffer_addr);

	isp_info_item_tag("1st_sensor_img2_addr 0x%x\n",
		g_bist_1stsensor_img2_iva);
	isp_info_item_tag("1st_sensor_img3_addr 0x%x\n",
		g_bist_1stsensor_img3_iva);
	isp_info_item_tag("1st_sensor_metadata_addr 0x%x\n",
		g_bist_1stsensor_statistics_iva);

	isp_info_item_tag("2nd_sensor_img1_addr 0x%x\n",
		g_bist_2ndsensor_img1_iva);
	isp_info_item_tag("2nd_sensor_img2_addr 0x%x\n",
		g_bist_2ndsensor_img2_iva);
	isp_info_item_tag("2nd_sensor_img3_addr 0x%x\n",
		g_bist_2ndsensor_img3_iva);
	isp_info_item_tag("2nd_sensor_metadata_addr 0x%x\n",
		g_bist_2ndsensor_statistics_iva);

	isp_info_item_tag("kva addr:%#llx\n", g_bist_buff_kva);

	isp_info_item_tag("golden_direct_addr 0x%llx\n",
		g_bist_direct_kva);
	isp_info_item_tag("golden_dram_LV_VIDEO_addr 0x%llx\n",
		g_bist_dram_lv_video_kva);
	isp_info_item_tag("golden_dram_NQ_STILL_addr 0x%llx\n",
		g_bist_dram_normal_quality_still_kva);
	isp_info_item_tag("golden_dram_HQ_STILL_addr 0x%llx\n",
		g_bist_dram_high_quality_still_kva);
	isp_info_item_tag("golden_front_addr 0x%llx\n",
		g_bist_front_kva);
	isp_info_item_tag("golden_proc_still_addr 0x%llx\n",
		g_bist_proc_still_kva);

	isp_info_lo_end_tag();
}
EXPORT_SYMBOL(ispdrv_set_self_test_addr);

/*
*\brief ISP Build In Self Test function
*\param scenario_id [In], Scenario ID
*\return error code
*/
u32 ispdrv_self_test(u8 scenario_id)
{
	u32 err = 0;
	u8 i = 0;
	u8 loop = 0, temp_index = 0;
	u8 format = 2;	/* 0: Raw 10 unpack (only Sensor 1), 1: NV12, 2: YUY2*/
	u8 total_test = 0;
	u8 scenario_index = 0;
	u16 bayerscl_width_1stsensor = 0;
	u16 bayerscl_height_1stsensor = 0;
	u16 bayerscl_width_2ndsensor = 0;
	u16 bayerscl_height_2ndsensor = 0;
	u16 out_width_1stsensor[V4L2_ISP_OUTPUT_IMAGE_TOTAL];
	u16 out_height_1stsensor[V4L2_ISP_OUTPUT_IMAGE_TOTAL];
	u16 out_width_2ndsensor[V4L2_ISP_OUTPUT_IMAGE_TOTAL];
	u16 out_height_2ndsensor[V4L2_ISP_OUTPUT_IMAGE_TOTAL];

	u64 golden_addr_1stsensor[V4L2_ISP_OUTPUT_IMAGE_TOTAL - 1];
	u64 golden_addr_2ndsensor[V4L2_ISP_OUTPUT_IMAGE_TOTAL - 1];

	u32 frame_done_addr_1stsensor[V4L2_ISP_OUTPUT_IMAGE_TOTAL] = {
		g_bist_1stsensor_img1_iva,
		g_bist_1stsensor_img2_iva,
		g_bist_1stsensor_img3_iva,
		g_bist_1stsensor_statistics_iva,
		g_bist_1stsensor_raw_img_iva
	};
	u32 frame_done_addr_2ndsensor[V4L2_ISP_OUTPUT_IMAGE_TOTAL] = {
		g_bist_2ndsensor_img1_iva,
		g_bist_2ndsensor_img2_iva,
		g_bist_2ndsensor_img3_iva,
		g_bist_2ndsensor_statistics_iva,
		g_bist_2ndsensor_raw_img_iva
	};
	u32 sum_golden = 0, xor_golden = 0;
	u32 sum_frame_done_1stsensor = 0, xor_frame_done_1stsensor = 0;
	u32 sum_frame_done_2ndsensor = 0, xor_frame_done_2ndsensor = 0;
	bool bypass_lv_1stsensor = false;
	bool bypass_video_1stsensor = false;
	bool bypass_still_1stsensor = false;
	bool bypass_statistics_1stsensor = false;
	bool bypass_lv_2ndsensor = false;
	bool bypass_video_2ndsensor = false;
	bool bypass_still_2ndsensor = false;
	bool bypass_statistics_2ndsensor = false;
	/* Indicate the IMG needs to be stream on, LV/VIDEO/STILL/METADATA*/
	bool stream_on_1stsensor[V4L2_ISP_OUTPUT_IMAGE_TOTAL] = {
		false, false, false, false, false };
	bool stream_on_2ndsensor[V4L2_ISP_OUTPUT_IMAGE_TOTAL] = {
		false, false, false, false, false };
	struct ispdrv_self_test_comp_info comp_info;
	struct s_scinfo_self_test_info self_test_info;
	struct s_scenario_info_ap scenario_info_1stsensor;
	struct s_scenario_info_ap scenario_info_2ndsensor;
	enum ispdrv_scenario_mode pipe_2nd_scenario_id;
	u8 temp_sensor_id;
#ifdef DEBUG_ALTEK_ISP
	static char scenario_string[ISP_SELF_TEST_TOTAL][25] = {
		"Single Direct", "Stripe Direct", "Dram Normal Quality",
		"Dram High Quality", "Dual", "Front" };
#endif
	struct output_addr_setting output_addr;
	struct s_scenario_info_isp *shared_scenario_info = NULL;

	isp_info_lo_start_tag();

	/* Check required qmerge bin address of all used sensors,
	 * Sensor 1 and Sensor Lite
	 */
	if (!common_is_qmerge_data_addr_set()) {
		isp_info_desc_tag("Invalid Qmerge Address\n");
		err = EINVAL;
		goto self_test_end;
	}

	/* Check self test IMGs address*/
	if (g_bist_1stsensor_img1_iva == 0 ||
		g_bist_1stsensor_img2_iva == 0 ||
		g_bist_1stsensor_img3_iva == 0 ||
		g_bist_1stsensor_statistics_iva == 0) {
		isp_info_desc_tag("Invalid FD Img Addr for 1st Sensor\n");
		err = EINVAL;
		goto self_test_end;
	}
	/* Check self test IMGs address for second sensor*/
	if (scenario_id == V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT ||
		scenario_id == V4L2_ISP_SCENARIO_TOTAL) {
		if (g_bist_2ndsensor_img1_iva == 0 ||
			g_bist_2ndsensor_img2_iva == 0 ||
			g_bist_2ndsensor_img3_iva ||
			g_bist_2ndsensor_statistics_iva == 0) {
			isp_info_desc_tag(
				"Invalid FD Img Addr for 2nd Sensor\n");
			err = EINVAL;
			goto self_test_end;
		}
	}

	if (g_bist_direct_kva == 0 ||
		g_bist_dram_lv_video_kva == 0 ||
		g_bist_dram_normal_quality_still_kva == 0 ||
		g_bist_dram_high_quality_still_kva == 0 ||
		g_bist_front_kva == 0 ||
		g_bist_proc_still_kva == 0) {
		isp_info_desc_tag("Invalid Golden Address\n");
		err = EINVAL;
		goto self_test_end;
	}

	/* g_bist_proc_still_iva[0].buffer_addr =
	 *	common_get_qmerge_data_addr_by_sensor_id(SENSOR3) + 0x20000;
	 *isp_info("Reassigned ProcStill_iva 0x%x\n",
	 *	g_bist_proc_still_iva[0].buffer_addr);
	 */

	isp_info_desc_tag("BIST START\n");

	/* Test all paths*/
	if (scenario_id == V4L2_ISP_SCENARIO_TOTAL) {
		total_test = 4;
		scenario_index = 0;
	} else {
		total_test = 1;
		/* Get Scenario index with related Scenario ID*/
		for (loop = 0; loop < ISP_SELF_TEST_TOTAL; loop++) {
			if (g_self_test_scenario[loop] == scenario_id) {
				scenario_index = loop;
				break;
			}
		}
	}

	/* Record the testing scenario id */
	g_bist_scenario_id = g_self_test_scenario[scenario_index];
	shared_scenario_info =
		(struct s_scenario_info_isp *)
		ispdrv_get_shared_scenario_info_ptr();


	for (i = 0; i < total_test; i++, scenario_index++) {
		/* Reset the scenario info*/
		memset((u8 *) &output_addr, 0,
			(sizeof(struct output_addr_setting)));
		memset((u8 *) &scenario_info_1stsensor, 0,
			sizeof(struct s_scenario_info_ap));
		memset((u8 *) &scenario_info_2ndsensor, 0,
			sizeof(struct s_scenario_info_ap));
		memset((u8 *) &comp_info, 0,
			sizeof(struct ispdrv_self_test_comp_info));
		memset((u8 *) &out_width_1stsensor, 0,
			sizeof(out_width_1stsensor));
		memset((u8 *) &out_height_1stsensor, 0,
			sizeof(out_height_1stsensor));
		memset((u8 *) &out_width_2ndsensor, 0,
			sizeof(out_width_2ndsensor));
		memset((u8 *) &out_height_2ndsensor, 0,
			sizeof(out_height_2ndsensor));
		memset((u8 *) &golden_addr_1stsensor, 0,
			sizeof(golden_addr_1stsensor));
		memset((u8 *) &golden_addr_2ndsensor, 0,
			sizeof(golden_addr_2ndsensor));
		memset((u8 *) &self_test_info, 0,
			sizeof(struct s_scinfo_self_test_info));
		memset((u8 *) &g_ss_raw_crop_size[0], 0,
			sizeof(struct s_scenario_size) * 2);
		memset((u8 *) &g_ss_still_crop_size[0], 0,
			sizeof(struct s_scenario_size) * 2);
		memset((u8 *) &g_ss_raw_iq_info_1[0], 0,
			sizeof(struct s_iq_info_1) * 2);
		memset((u8 *) &g_ss_raw_iq_info_2[0], 0,
			sizeof(struct s_iq_info_2) * 2);
		memset((u8 *) &g_ss_still_iq_info_1[0], 0,
			sizeof(struct s_iq_info_1) * 2);
		memset((u8 *) &g_ss_still_iq_info_2[0], 0,
			sizeof(struct s_iq_info_2) * 2);

		/* memset((u8 *) &g_ss_raw_info_setting[0], 0,
		 *  sizeof(struct s_raw_info_setting) * 2);
		 */
		memset((u8 *) &g_ss_opt[0], 0,
			sizeof(struct ispdrv_proc_still_opt) * 2);
		memset((u8 *)&g_bist_proc_still_start[0], 0, sizeof(u8) * 2);
		memset((u8 *)&g_bist_proc_still_end[0], 0, sizeof(u8) * 2);
		memset((u8 *)&g_bist_proc_still_fw_start[0], 0,
			sizeof(u8) * 2);
		memset((u8 *) &g_bist_proc_still_src_raw_iva[0], 0,
			sizeof(u32) * 2);

		temp_index = 0;

		/*  Step 1. Set scenario info according to scenario ID*/
		switch (g_self_test_scenario[scenario_index]) {
		/* Single direct*/
		case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_1:
		/* Stripe direct*/
		case V4L2_ISP_SCENARIO_PREVIEW_STRIPE:
		/* Single direct for Sensor 2*/
		case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_2:
			g_bist_1stsensor_input_width = 3264;
			g_bist_1stsensor_input_height = 2464;
			bayerscl_width_1stsensor = 0;
			bayerscl_height_1stsensor = 0;
			bypass_lv_1stsensor = true;
			bypass_video_1stsensor = true;
			bypass_still_1stsensor = false;
			bypass_statistics_1stsensor = false;
			/* Assign the address of golden image
			 * used to compare with result
			 */
			golden_addr_1stsensor[V4L2_ISP_IMG1] =
				g_bist_direct_kva;
			golden_addr_1stsensor[V4L2_ISP_IMG2] =
				g_bist_direct_kva;
			golden_addr_1stsensor[V4L2_ISP_IMG3] =
				g_bist_direct_kva;

			break;
			/* front*/
		case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_LITE:
			g_bist_1stsensor_input_width = 1920;
			g_bist_1stsensor_input_height = 1080;
			bayerscl_width_1stsensor = 0;
			bayerscl_height_1stsensor = 0;
			bypass_lv_1stsensor = false;
			bypass_video_1stsensor = true;
			bypass_still_1stsensor = true;
			bypass_statistics_1stsensor = false;
			/* Assign the address of golden image
			 * used to compare with result
			 */
			golden_addr_1stsensor[V4L2_ISP_IMG1] =
				g_bist_front_kva;

			break;
		case V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT:
			g_bist_1stsensor_input_width = 3264;
			g_bist_1stsensor_input_height = 2464;
			bayerscl_width_1stsensor = 0;
			bayerscl_height_1stsensor = 0;
			bypass_lv_1stsensor = true;
			bypass_video_1stsensor = true;
			bypass_still_1stsensor = false;
			bypass_statistics_1stsensor = false;
			g_bist_2ndsensor_input_width = 3264;
			g_bist_2ndsensor_input_height = 2464;
			bayerscl_width_2ndsensor = 0;
			bayerscl_height_2ndsensor = 0;
			bypass_lv_2ndsensor = true;
			bypass_video_2ndsensor = true;
			bypass_still_2ndsensor = false;
			bypass_statistics_2ndsensor = false;
			/* Assign the address of golden image
			 * used to compare with result
			 */
			golden_addr_1stsensor[V4L2_ISP_IMG1] =
				g_bist_direct_kva;
			golden_addr_1stsensor[V4L2_ISP_IMG2] =
				g_bist_direct_kva;
			golden_addr_1stsensor[V4L2_ISP_IMG3] =
				g_bist_direct_kva;
			golden_addr_2ndsensor[V4L2_ISP_IMG1] =
				g_bist_direct_kva;
			golden_addr_2ndsensor[V4L2_ISP_IMG2] =
				g_bist_direct_kva;
			golden_addr_2ndsensor[V4L2_ISP_IMG3] =
				g_bist_direct_kva;

			break;
		case V4L2_ISP_SCENARIO_PREVIEW_STILL_SS:

			g_bist_1stsensor_input_width = 3456; /* 4920; 960; */
			g_bist_1stsensor_input_height = 2304; /* 360; 720; */
			bayerscl_width_1stsensor = 960;
			bayerscl_height_1stsensor = 720;
			bypass_lv_1stsensor = false;
			bypass_video_1stsensor = true;
			bypass_still_1stsensor = false;
			bypass_statistics_1stsensor = false;
			golden_addr_1stsensor[V4L2_ISP_IMG1] =
				g_bist_proc_still_kva;
			golden_addr_1stsensor[V4L2_ISP_IMG3] =
				g_bist_proc_still_kva;

			break;
		default:
			err = EINVAL;
			goto self_test_end;
		}

		/* Update scenario info*/
		scenario_info_1stsensor.sensor_info.sensor_mode = 0;
		scenario_info_1stsensor.sensor_info.sensor_module_type = 0;
		scenario_info_1stsensor.sensor_info.original_width =
			g_bist_1stsensor_input_width;
		scenario_info_1stsensor.sensor_info.original_height =
			g_bist_1stsensor_input_height;
		scenario_info_1stsensor.sensor_info.crop_start_x = 0;
		scenario_info_1stsensor.sensor_info.crop_start_y = 0;
		scenario_info_1stsensor.sensor_info.crop_end_x =
			g_bist_1stsensor_input_width - 1;
		scenario_info_1stsensor.sensor_info.crop_end_y =
			g_bist_1stsensor_input_height - 1;
		scenario_info_1stsensor.sensor_info.width =
			g_bist_1stsensor_input_width;
		scenario_info_1stsensor.sensor_info.height =
			g_bist_1stsensor_input_height;
		scenario_info_1stsensor.sensor_info.frame_rate = 2400;
		scenario_info_1stsensor.sensor_info.line_time = 1315;
		scenario_info_1stsensor.sensor_info.n_color_order =
			E_SCINFO_COLOR_ORDER_RG;
		scenario_info_1stsensor.sensor_info.clamp_level = 64;
		scenario_info_1stsensor.out_bypass_flag.bypass_lv =
			bypass_lv_1stsensor;
		scenario_info_1stsensor.out_bypass_flag.bypass_video =
			bypass_video_1stsensor;
		scenario_info_1stsensor.out_bypass_flag.bypass_still =
			bypass_still_1stsensor;
		scenario_info_1stsensor.out_bypass_flag.bypass_metadata =
			bypass_statistics_1stsensor;
		scenario_info_1stsensor.bayerscl_out_info.bayerscl_out_width =
			bayerscl_width_1stsensor;
		scenario_info_1stsensor.bayerscl_out_info.bayerscl_out_height =
			bayerscl_height_1stsensor;

		if (V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT ==
			g_self_test_scenario[scenario_index]) {
			scenario_info_2ndsensor.sensor_info.sensor_mode = 0;
			scenario_info_2ndsensor.sensor_info
				.sensor_module_type = 0;
			scenario_info_2ndsensor.sensor_info.width =
				g_bist_2ndsensor_input_width;
			scenario_info_2ndsensor.sensor_info.height =
				g_bist_2ndsensor_input_height;
			scenario_info_2ndsensor.sensor_info
				.frame_rate = 2400;
			scenario_info_2ndsensor.sensor_info
				.line_time = 1315;
			scenario_info_2ndsensor.sensor_info
				.n_color_order = E_SCINFO_COLOR_ORDER_RG;
			scenario_info_2ndsensor.sensor_info
				.clamp_level = 64;
			scenario_info_2ndsensor.out_bypass_flag
				.bypass_lv = bypass_lv_2ndsensor;
			scenario_info_2ndsensor.out_bypass_flag
				.bypass_video = bypass_video_2ndsensor;
			scenario_info_2ndsensor.out_bypass_flag
				.bypass_still = bypass_still_2ndsensor;
			scenario_info_2ndsensor.out_bypass_flag
				.bypass_metadata = bypass_statistics_2ndsensor;
			scenario_info_2ndsensor.bayerscl_out_info
				.bayerscl_out_width = bayerscl_width_2ndsensor;
			scenario_info_2ndsensor.bayerscl_out_info
				.bayerscl_out_height =
				bayerscl_height_2ndsensor;
		}

		isp_info_desc_tag("selftest chk2\n");

		/* Set Sensor ID and ScenarioInfo for front path*/
		if (V4L2_ISP_SCENARIO_PREVIEW_SENSOR_LITE ==
			g_self_test_scenario[scenario_index]) {
			err = ispdrv_set_scenario_info(
				g_self_test_scenario[scenario_index], NULL,
				NULL, &scenario_info_1stsensor);
			if (err != 0) {
				isp_info_desc_tag(
					"BIST:SetScenarioInfo Error\n");
				goto self_test_end;
			}
			g_bist_1stsensor_id = SENSOR3;
			/* assign the size of output image */
			out_width_1stsensor[V4L2_ISP_IMG1] = 1920;
			out_height_1stsensor[V4L2_ISP_IMG1] = 1080;
		} else if (V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT ==
			   g_self_test_scenario[scenario_index]) {
			err = ispdrv_set_scenario_info(
				g_self_test_scenario[scenario_index],
				&scenario_info_1stsensor,
				&scenario_info_2ndsensor, NULL);
			if (err != 0) {
				isp_info_desc_tag(
					"BIST:SetScenarioInfo Error\n");
				goto self_test_end;
			}
			g_bist_1stsensor_id = SENSOR1;
			g_bist_2ndsensor_id = SENSOR2;
			/* assign the size of output image */
			out_width_1stsensor[V4L2_ISP_IMG1] = 960;
			out_width_1stsensor[V4L2_ISP_IMG2] = 960;
			out_width_1stsensor[V4L2_ISP_IMG3] = 960;
			out_height_1stsensor[V4L2_ISP_IMG1] = 720;
			out_height_1stsensor[V4L2_ISP_IMG2] = 720;
			out_height_1stsensor[V4L2_ISP_IMG3] = 720;
			out_width_2ndsensor[V4L2_ISP_IMG1] = 960;
			out_width_2ndsensor[V4L2_ISP_IMG2] = 960;
			out_width_2ndsensor[V4L2_ISP_IMG3] = 960;
			out_height_2ndsensor[V4L2_ISP_IMG1] = 720;
			out_height_2ndsensor[V4L2_ISP_IMG2] = 720;
			out_height_2ndsensor[V4L2_ISP_IMG3] = 720;
		} else if (V4L2_ISP_SCENARIO_PREVIEW_STILL_SS ==
			   g_self_test_scenario[scenario_index]) {
			err = ispdrv_set_scenario_info(
				g_self_test_scenario[scenario_index],
				&scenario_info_1stsensor,
				NULL, NULL);
			if (err != 0) {
				isp_info_desc_tag(
					"BIST:SetScenarioInfo Error\n");
				goto self_test_end;
			}
			g_bist_1stsensor_id = SENSOR1;

			/* 1920; 960; */
			out_width_1stsensor[V4L2_ISP_IMG1] = 960;

			/* //4224;//960;//4920; //960; */
			out_width_1stsensor[V4L2_ISP_IMG3] = 3456;

			/* 4224;//960;//4920; //960; */
			out_width_1stsensor[V4L2_ISP_RAW] = 3456;

			/* 360; //720; */
			out_height_1stsensor[V4L2_ISP_IMG1] = 720;

			/* 2880;//720;//360; //720; */
			out_height_1stsensor[V4L2_ISP_IMG3] = 2304;

			/* 2880;//720;//360; //720; */
			out_height_1stsensor[V4L2_ISP_RAW] = 2304;
		} else {
			if (V4L2_ISP_SCENARIO_PREVIEW_SENSOR_2 ==
				g_self_test_scenario[scenario_index]) {
				err = ispdrv_set_scenario_info(
					g_self_test_scenario[scenario_index],
					NULL, &scenario_info_1stsensor, NULL);
				g_bist_1stsensor_id = SENSOR2;
			} else {
				err = ispdrv_set_scenario_info(
					g_self_test_scenario[scenario_index],
					&scenario_info_1stsensor,
					NULL, NULL);
				g_bist_1stsensor_id = SENSOR1;
			}
			if (err != 0) {
				isp_info_desc_tag(
					"BIST:SetScenarioInfo Error\n");
				goto self_test_end;
			}
			/* assign the size of output image */
			if (g_self_test_scenario[scenario_index] ==
				V4L2_ISP_SCENARIO_PREVIEW_SENSOR_1 ||
				g_self_test_scenario[scenario_index] ==
				V4L2_ISP_SCENARIO_PREVIEW_STRIPE ||
				g_self_test_scenario[scenario_index] ==
				V4L2_ISP_SCENARIO_PREVIEW_SENSOR_2) {
				out_width_1stsensor[V4L2_ISP_IMG1] = 960;
				out_width_1stsensor[V4L2_ISP_IMG2] = 960;
				out_width_1stsensor[V4L2_ISP_IMG3] = 960;
				out_height_1stsensor[V4L2_ISP_IMG1] = 720;
				out_height_1stsensor[V4L2_ISP_IMG2] = 720;
				out_height_1stsensor[V4L2_ISP_IMG3] = 720;
			} else {
				out_width_1stsensor[V4L2_ISP_IMG1] = 640;
				out_width_1stsensor[V4L2_ISP_IMG2] = 640;
				out_width_1stsensor[V4L2_ISP_IMG3] = 1920;
				out_height_1stsensor[V4L2_ISP_IMG1] = 360;
				out_height_1stsensor[V4L2_ISP_IMG2] = 360;
				out_height_1stsensor[V4L2_ISP_IMG3] = 1080;
			}
		}

		/* Indicate the image needed to stream on
		 * if the bypass flag is false
		 */
		stream_on_1stsensor[V4L2_ISP_IMG1] =
			!bypass_lv_1stsensor;
		stream_on_1stsensor[V4L2_ISP_IMG2] =
			!bypass_video_1stsensor;
		stream_on_1stsensor[V4L2_ISP_IMG3] =
			!bypass_still_1stsensor;
		stream_on_1stsensor[V4L2_ISP_STATITISTICS] =
			!bypass_statistics_1stsensor;

		if (g_self_test_scenario[scenario_index] ==
			V4L2_ISP_SCENARIO_PREVIEW_STILL_SS)
			stream_on_1stsensor[V4L2_ISP_RAW] = true;

		if (g_self_test_scenario[scenario_index] ==
			V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT) {
			stream_on_2ndsensor[V4L2_ISP_IMG1] =
				!bypass_lv_2ndsensor;
			stream_on_2ndsensor[V4L2_ISP_IMG2] =
				!bypass_video_2ndsensor;
			stream_on_2ndsensor[V4L2_ISP_IMG3] =
				!bypass_still_2ndsensor;
			stream_on_2ndsensor[V4L2_ISP_STATITISTICS] =
				!bypass_statistics_2ndsensor;
		}
		/* Set the bist info */
		self_test_info.self_test_mode = E_SELFTEST_BIST;
		self_test_info.max_frame_cnt = SS_SECOND_RAW_FRAME_IDX;

		self_test_info.sensor_id_1st = g_bist_1stsensor_id;
		self_test_info.is_frame_done_1stsensor = false;
		self_test_info.frame_count_1stsensor = 0;
		self_test_info.is_independent = bist_get_independent_flag();
		if (g_self_test_scenario[scenario_index] ==
			V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT) {
			self_test_info.is_dual_sensor_mode = true;
			self_test_info.sensor_id_2nd = g_bist_2ndsensor_id;
			self_test_info.is_frame_done_2ndsensor = false;
			self_test_info.frame_count_2ndsensor = 0;
		}

		/* Get the IMG index for comparing, only IMG1, IMG2, and IMG3*/
		for (loop = 0; loop < 3; loop++) {
			if (stream_on_1stsensor[loop]) {
				comp_info.uc_comp_number =
					comp_info.uc_comp_number + 1;
				comp_info.auc_comp_idx[temp_index++] =
					loop;
			}
		}

		/* Step 2. Set 3A settings*/
		err = self_test_set_3a_cfg(g_bist_1stsensor_id,
				g_self_test_scenario[scenario_index],
				g_bist_1stsensor_input_height);
		if (err != 0)
			goto self_test_end;

		/*  Step 3. Set output format*/
		if (g_self_test_scenario[scenario_index] ==
			V4L2_ISP_SCENARIO_PREVIEW_STILL_SS) {

			format = V4L2_ISP_OUTPUT_YUY2;
			/* Set output addr for IMG1 */
			loop = V4L2_ISP_IMG1;
			output_addr.ud_addr1 =
				frame_done_addr_1stsensor[loop];
			output_addr.ud_addr2 =
				output_addr.ud_addr1 + SS_LV_BUFFER_SIZE;
			output_addr.ud_addr3 =
				output_addr.ud_addr2 + SS_LV_BUFFER_SIZE;
			output_addr.ud_addr4 =
				output_addr.ud_addr3 + SS_LV_BUFFER_SIZE;

			err = ispdrv_set_output_buff_format(
					g_bist_1stsensor_id,
					loop,
					false,
					format,
					out_width_1stsensor[loop],
					out_height_1stsensor[loop],
					4, &output_addr,
					out_width_1stsensor[loop] * 2);
			if (err != 0) {
				isp_info_desc_tag(
					"BIST:SetOutputBuffFormat SS LV\n");
				goto self_test_end;
			}

			/* Set output addr for Still */
			loop = V4L2_ISP_IMG3;
			output_addr.ud_addr1 =
				g_bist_proc_still_iva[0].buffer_addr;
			output_addr.ud_addr2 = 0;
			output_addr.ud_addr3 = 0;
			output_addr.ud_addr4 = 0;
			err = ispdrv_set_output_buff_format(
					g_bist_1stsensor_id,
					loop,
					false,
					format,
					out_width_1stsensor[loop],
					out_height_1stsensor[loop],
					1, &output_addr,
					out_width_1stsensor[loop] * 2);
			if (err != 0) {
				isp_info_desc_tag(
					"BIST: SetOutBufFormat SS STILL\n");
				goto self_test_end;
			}

			/* Set output addr for RAW */
			loop = V4L2_ISP_RAW;
			output_addr.ud_addr1 =
				frame_done_addr_1stsensor[loop];
			output_addr.ud_addr2 =
				output_addr.ud_addr1 + SS_RAW_BUFFER_SIZE;
			output_addr.ud_addr3 =
				output_addr.ud_addr2 + SS_RAW_BUFFER_SIZE;
			output_addr.ud_addr4 =
				output_addr.ud_addr3 + SS_RAW_BUFFER_SIZE;

			format = V4L2_ISP_OUTPUT_ALTEK_RAW10;
			err = ispdrv_set_output_buff_format(
					g_bist_1stsensor_id,
					loop,
					false,
					format,
					out_width_1stsensor[loop],
					out_height_1stsensor[loop],
					4, &output_addr,
					out_width_1stsensor[loop] * 2);
			if (err != 0) {
				isp_info_desc_tag(
					"BIST:SetOutputBuffFormat SS RAW\n");
				goto self_test_end;
			}
		} else {
			for (loop = 0; loop < V4L2_ISP_OUTPUT_IMAGE_TOTAL - 1;
				loop++) {

				output_addr.ud_addr1 =
					frame_done_addr_1stsensor[loop];
				err = ispdrv_set_output_buff_format(
					g_bist_1stsensor_id, loop, true,
					format, out_width_1stsensor[loop],
					out_height_1stsensor[loop], 1,
					&output_addr,
					out_width_1stsensor[loop] * 2);
				if (err != 0) {
					isp_info_desc_tag(
						"BIST: SetOutBufFormat Err\n");
					goto self_test_end;
				}
			}
		}

		/* Partial setting for 2nd sensor in dual sensor mode */
		if (g_self_test_scenario[scenario_index] ==
			V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT) {
			for (loop = 0;
				loop < V4L2_ISP_OUTPUT_IMAGE_TOTAL - 1;
				loop++) {

				output_addr.ud_addr1 = 0;
				err = ispdrv_set_output_buff_format(
					g_bist_2ndsensor_id, loop, false,
					format, out_width_2ndsensor[loop],
					out_height_2ndsensor[loop], 0,
					&output_addr,
					out_width_2ndsensor[loop] * 2);
				if (err != 0) {
					isp_info_desc_tag(
						"BIST: SetOutBufFormat DUAL Err\n");
					goto self_test_end;
				}
			}
		}

		/* Step 4. Enable Self Test Mode and clear frame done flag*/
		MEMCPY((u8 *) &shared_scenario_info->self_test_info,
			(u8 *) &self_test_info,
			sizeof(struct s_scinfo_self_test_info));

		/* Wait 120ms for HW update*/
		SLEEPRANGE(120000, 125000);

		/* Step 5. Set preview mode*/
		err = ispdrv_set_preview_mode(
			g_self_test_scenario[scenario_index]);
		if (err != 0) {
			isp_info_desc_tag("BIST:SetPreviewMode Error\n");
			goto self_test_end;
		}

		if (g_self_test_scenario[scenario_index] ==
			V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT) {

			err = self_test_set_3a_cfg(g_bist_2ndsensor_id,
				g_self_test_scenario[scenario_index],
				g_bist_2ndsensor_input_height);
			if (err != 0)
				goto self_test_end;

			/* Full setting for Rear2*/
			for (loop = 0; loop < V4L2_ISP_OUTPUT_IMAGE_TOTAL;
				loop++) {

				output_addr.ud_addr1 =
					frame_done_addr_2ndsensor[loop];
				err = ispdrv_set_output_buff_format(
					g_bist_2ndsensor_id, loop, false,
					format, out_width_2ndsensor[loop],
					out_height_2ndsensor[loop], 1,
					&output_addr,
					out_width_2ndsensor[loop] * 2);
				if (err != 0) {
					isp_info_item_tag(
						"BIST:SetBufFmt Err\n");
					goto self_test_end;
				}
			}

			for (loop = 0; loop < V4L2_ISP_OUTPUT_IMAGE_TOTAL;
				loop++) {

				if (stream_on_2ndsensor[loop]) {
					err = ispdrv_preview_stream_on(
						g_bist_2ndsensor_id, loop);
					if (err != 0) {
						isp_info_desc_tag(
							"BIST:StreamOn Err\n");
						goto self_test_end;
					}
				}
			}

		}

		if (g_self_test_scenario[scenario_index] ==
			V4L2_ISP_SCENARIO_PREVIEW_STILL_SS) {
			err = ispdrv_preview_stream_on(g_bist_1stsensor_id,
							V4L2_ISP_IMG1);
			if (err != 0) {
				isp_info_desc_tag("BIST:Stream On Failed\n");
				goto self_test_end;
			}
			err = ispdrv_set_raw_frame_rate(g_bist_1stsensor_id,
							ISP_RAW_FR_1);
			if (err != 0) {
				isp_info_desc_tag(
					"BIST:Set Raw Frame Rate Err\n");
				goto self_test_end;
			}
		}
		/* Initial the frame done count */
		common_reset_bist_frame_done_cnt();

		/* Step 6. Turn on sensor with test pattern*/
		if (g_self_test_scenario[scenario_index] ==
			V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT) {

			self_test_activate_test_pattern(
				g_bist_1stsensor_id,
				g_bist_1stsensor_input_width,
				g_bist_1stsensor_input_height);
			self_test_activate_test_pattern(
				g_bist_2ndsensor_id,
				g_bist_2ndsensor_input_width,
				g_bist_2ndsensor_input_height);
		} else
			self_test_activate_test_pattern(
				g_bist_1stsensor_id,
				g_bist_1stsensor_input_width,
				g_bist_1stsensor_input_height);


		/* Get output buffer and set output buffer for RAW */
		if (g_self_test_scenario[scenario_index] ==
			V4L2_ISP_SCENARIO_PREVIEW_STILL_SS) {

			err = self_test_proc_still();
			if (err != 0) {
				isp_info_desc_tag("BIST:Proc_Still Failed\n");
				goto self_test_end;
			}

			isp_info_desc_tag(
				"BIST:Proc_Still Twice Finished\n\n");
			isp_info_desc_tag("BIST: Stop Raw Frame done\n");
			err = ispdrv_set_raw_frame_rate(g_bist_1stsensor_id,
							ISP_RAW_FR_OFF);
			if (err != 0) {
				isp_info_desc_tag(
					"BIST:Set Raw Frame Rate Err\n");
				goto self_test_end;
			}

			while (1)
				SLEEPRANGE(42000, 50000);

			err = ispdrv_preview_stream_off(g_bist_1stsensor_id,
							V4L2_ISP_IMG1);
			if (err != 0) {
				isp_info_desc_tag("BIST:Stream On Failed\n");
				goto self_test_end;
			}
			err = ispdrv_set_raw_frame_rate(g_bist_1stsensor_id,
							ISP_RAW_FR_OFF);
			if (err != 0) {
				isp_info_desc_tag(
					"BIST:Set Raw Frame Rate Err\n");
				goto self_test_end;
			}
		} else if (true ==
			shared_scenario_info->self_test_info.is_independent) {

			/* Start second sensor for independent mode */
			if (g_self_test_scenario[scenario_index] ==
				V4L2_ISP_SCENARIO_PREVIEW_SENSOR_1) {
				pipe_2nd_scenario_id =
					V4L2_ISP_SCENARIO_PREVIEW_SENSOR_2;
				g_bist_2ndsensor_id = SENSOR2;
			} else {
				pipe_2nd_scenario_id =
					V4L2_ISP_SCENARIO_PREVIEW_SENSOR_1;
				g_bist_2ndsensor_id = SENSOR1;
			}
			g_bist_2ndsensor_input_width = 3264;
			g_bist_2ndsensor_input_height = 2464;
			bayerscl_width_2ndsensor = 0;
			bayerscl_height_2ndsensor = 0;
			bypass_lv_2ndsensor = true;
			bypass_video_2ndsensor = true;
			bypass_still_2ndsensor = false;
			bypass_statistics_2ndsensor = false;
			/* Assign the address of golden image
			 * used to compare with result
			 */
			golden_addr_2ndsensor[V4L2_ISP_IMG1] =
				g_bist_direct_kva;
			golden_addr_2ndsensor[V4L2_ISP_IMG2] =
				g_bist_direct_kva;
			golden_addr_2ndsensor[V4L2_ISP_IMG3] =
				g_bist_direct_kva;

			/* Update scenario info*/
			scenario_info_2ndsensor.
				sensor_info.sensor_mode = 0;
			scenario_info_2ndsensor.
				sensor_info.sensor_module_type = 0;
			scenario_info_2ndsensor.sensor_info.width =
				g_bist_2ndsensor_input_width;
			scenario_info_2ndsensor.sensor_info.height =
				g_bist_2ndsensor_input_height;
			scenario_info_2ndsensor.sensor_info.frame_rate = 2400;
			scenario_info_2ndsensor.sensor_info.line_time = 1315;
			scenario_info_2ndsensor.sensor_info.n_color_order =
				E_SCINFO_COLOR_ORDER_RG;
			scenario_info_2ndsensor.sensor_info.clamp_level = 64;
			scenario_info_2ndsensor.out_bypass_flag.bypass_lv =
				bypass_lv_2ndsensor;
			scenario_info_2ndsensor.out_bypass_flag.bypass_video =
				bypass_video_2ndsensor;
			scenario_info_2ndsensor.out_bypass_flag.bypass_still =
				bypass_still_2ndsensor;
			scenario_info_2ndsensor.
				out_bypass_flag.bypass_metadata =
				bypass_statistics_2ndsensor;
			scenario_info_2ndsensor.
				bayerscl_out_info.bayerscl_out_width =
				bayerscl_width_2ndsensor;
			scenario_info_2ndsensor.
				bayerscl_out_info.bayerscl_out_height =
				bayerscl_height_2ndsensor;
			isp_info_desc_tag("Preview2:SetScenarioInfo\n");

			/* Step 5. Get Self Test Mode*/
			MEMCPY((u8 *) &self_test_info,
				(u8 *)&shared_scenario_info->self_test_info,
				sizeof(struct s_scinfo_self_test_info));

			/* Set the scenario info for the seconde used sensor */
			if (g_self_test_scenario[scenario_index] ==
				V4L2_ISP_SCENARIO_PREVIEW_SENSOR_1)

				err = ispdrv_set_scenario_info(
					pipe_2nd_scenario_id,
					NULL, &scenario_info_2ndsensor, NULL);
			else
				err = ispdrv_set_scenario_info(
					pipe_2nd_scenario_id,
					&scenario_info_2ndsensor,
					NULL, NULL);


			out_width_2ndsensor[V4L2_ISP_IMG1] = 960;
			out_width_2ndsensor[V4L2_ISP_IMG2] = 960;
			out_width_2ndsensor[V4L2_ISP_IMG3] = 960;
			out_height_2ndsensor[V4L2_ISP_IMG1] = 720;
			out_height_2ndsensor[V4L2_ISP_IMG2] = 720;
			out_height_2ndsensor[V4L2_ISP_IMG3] = 720;

			stream_on_2ndsensor[V4L2_ISP_IMG1] =
				!bypass_lv_2ndsensor;
			stream_on_2ndsensor[V4L2_ISP_IMG2] =
				!bypass_video_2ndsensor;
			stream_on_2ndsensor[V4L2_ISP_IMG3] =
				!bypass_still_2ndsensor;
			stream_on_2ndsensor[V4L2_ISP_STATITISTICS] =
				!bypass_statistics_2ndsensor;
			isp_info_desc_tag("Preview2:Set3ASetting\n");

			/* Step 2. Set 3A settings*/
			err = self_test_set_3a_cfg(g_bist_2ndsensor_id,
				pipe_2nd_scenario_id,
				g_bist_2ndsensor_input_height);
			if (err != 0)
				goto self_test_end;

			isp_info_desc_tag("Preview2:SetBuffFormat\n");
			/*  Step 4. Set output format*/
			for (loop = 0; loop < V4L2_ISP_OUTPUT_IMAGE_TOTAL - 1;
				loop++) {

				output_addr.ud_addr1 =
					frame_done_addr_2ndsensor[loop];
				if (loop == V4L2_ISP_STATITISTICS) {
					err = ispdrv_set_output_buff_format(
						g_bist_2ndsensor_id, loop,
						false, format,
						out_width_2ndsensor[loop],
						out_height_2ndsensor[loop],
						1, &output_addr,
						out_width_2ndsensor[loop] * 2);
				} else {
					err = ispdrv_set_output_buff_format(
						g_bist_2ndsensor_id, loop,
						true, format,
						out_width_2ndsensor[loop],
						out_height_2ndsensor[loop],
						1, &output_addr,
						out_width_2ndsensor[loop] * 2);
				}
				if (err != 0) {
					isp_info_desc_tag(
						"BIST:SetOutputBuffFormat Error\n");
					goto self_test_end;
				}
			}
			/* Step 5. Enable Self Test Mode and clear */
			/* frame done flag */

			self_test_info.is_dual_sensor_mode = true;
			self_test_info.sensor_id_2nd = g_bist_2ndsensor_id;
			self_test_info.is_frame_done_2ndsensor = false;
			self_test_info.frame_count_2ndsensor = 0;
			MEMCPY((u8 *) &shared_scenario_info->self_test_info,
				(u8 *) &self_test_info,
				sizeof(struct s_scinfo_self_test_info));

			/* Wait 120ms for HW update*/
			SLEEPRANGE(120000, 125000);

			isp_info_desc_tag("Preview2:SetPreviewMode\n");
			/* Step 5. Set preview mode*/
			err = ispdrv_set_preview_mode(
				pipe_2nd_scenario_id);
			if (err != 0) {
				isp_info_desc_tag(
					"BIST:SetPreviewMode Error\n");
				goto self_test_end;
			}

			isp_info_desc_tag("Preview2:StartSensor2\n");
			self_test_activate_test_pattern(
				g_bist_2ndsensor_id,
				g_bist_2ndsensor_input_width,
				g_bist_2ndsensor_input_height);
			isp_info_desc_tag("Preview2:WaitSensor2FrameDone\n");

			/* Step 8. Wait until frame done*/
			while (shared_scenario_info->
				self_test_info.is_frame_done_1stsensor ==
				false ||
				shared_scenario_info->
				self_test_info.is_frame_done_2ndsensor ==
				false) {

				SLEEPRANGE(1000000, 1005000);
				isp_info_desc_tag(
					"BIST:Wait two sensors frame done\n");
			}

			isp_info_desc_tag("sensor 2 frame done\n");
			isp_info_desc_tag("BIST: frame done success\n");
		} else {
			/* Step 8. Wait until frame done*/
			while (shared_scenario_info->self_test_info.
				is_frame_done_1stsensor == false) {
				SLEEPRANGE(1000000, 1005000);
				isp_info_desc_tag("BIST:Wait frame done\n");
			}
			isp_info_desc_tag("BIST: frame done success\n");
		}

		/* Wait 42ms for HW idle*/
		SLEEPRANGE(42000, 50000);

		while (1)
			SLEEPRANGE(42000, 50000);

		/* Step 10. Stop preview mode*/

		/* switch 2 sensors for change the closing sequence */
		if (bist_get_reverse_flag() == true) {
			temp_sensor_id = g_bist_1stsensor_id;
			g_bist_1stsensor_id = g_bist_2ndsensor_id;
			g_bist_2ndsensor_id = temp_sensor_id;
		}


		err = ispdrv_stop_preview_mode(g_bist_1stsensor_id);
		if (err != 0) {
			isp_info_desc_tag("BIST:StopPreviewMode Error\n");
			goto self_test_end;
		}

		/* Step 11. Release soft reset*/
		if (g_bist_1stsensor_id == SENSOR1)
			set_ahb_indirect(0xfffa4000, 0x1);
		else if (g_bist_1stsensor_id == SENSOR2)
			set_ahb_indirect(0xfffa5000, 0x1);
		else
			set_ahb_indirect(0xfffa3000, 0x1);

		/* Dual Rear*/
		if (g_self_test_scenario[scenario_index] ==
			V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT) {

			err = ispdrv_stop_preview_mode(g_bist_2ndsensor_id);
			if (err != 0) {
				isp_info_desc_tag(
					"BIST:StopPreviewMode Error\n");
				goto self_test_end;
			}

			if (g_bist_2ndsensor_id == SENSOR1) {
				set_ahb_indirect(0xfffa4000, 0x1);
				set_ahb_indirect(0xfffa4000, 0x0);
			} else if (g_bist_2ndsensor_id == SENSOR2) {
				set_ahb_indirect(0xfffa5000, 0x1);
				set_ahb_indirect(0xfffa5000, 0x0);
			} else {
				set_ahb_indirect(0xfffa3000, 0x1);
				set_ahb_indirect(0xfffa3000, 0x0);
			}
		} else if (
			shared_scenario_info->self_test_info.is_independent ==
			true) {

			/* Step 10. Stop preview mode*/
			err = ispdrv_stop_preview_mode(g_bist_2ndsensor_id);

			isp_info_desc_tag(
				"ispdrv_stop_preview_mode 2nd sensor for PIPE\n");
			if (err != 0) {
				isp_info_desc_tag(
					"BIST:StopPreview Mode Error\n");
				/* goto self_test_end; */
			}

			/* Step 11. Release soft reset*/
			if (g_bist_2ndsensor_id == SENSOR1)
				set_ahb_indirect(0xfffa4000, 0x1);
			else if (g_bist_2ndsensor_id == SENSOR2)
				set_ahb_indirect(0xfffa5000, 0x1);
			else
				set_ahb_indirect(0xfffa3000, 0x1);
		}

#ifdef DEBUG_ALTEK_ISP
		isp_info_desc_tag("%s test result\n",
			scenario_string[scenario_index]);
#endif
		isp_info_item_tag("BIST: comp_number %d\n",
			comp_info.uc_comp_number);

		/* Step 12. Compare with Golden*/
		for (loop = 0; loop < comp_info.uc_comp_number; loop++) {

#ifdef DEBUG_ALTEK_ISP
			u8 *data =
				(u8 *) golden_addr_1stsensor[
				comp_info.auc_comp_idx[loop]];
			isp_info_item_tag(
				"GoldenData:  %x %x %x %x %x %x %x %x\n",
				*(data), *(data + 1), *(data + 2), *(data + 3),
				*(data + 4), *(data + 5), *(data + 6),
				*(data + 7));
#endif

			isp_info_item_tag("\nGolden, IMG%d Addr%llx\n",
				comp_info.auc_comp_idx[loop] + 1,
				golden_addr_1stsensor[
				comp_info.auc_comp_idx[loop]]);

			self_test_check_sum(golden_addr_1stsensor[
				comp_info.auc_comp_idx[loop]],
				2 * out_width_1stsensor[
				comp_info.auc_comp_idx[loop]] *
				out_height_1stsensor[
				comp_info.auc_comp_idx[loop]],
				&sum_golden, &xor_golden);

			isp_info_item_tag("Sum: 0x%08x, Xor: 0x%08x\n",
				sum_golden,
				xor_golden);

			isp_info_item_tag(
				"Sensor %d IMG%d ivaAddr%x, kvaAddr%llX\n",
				g_bist_1stsensor_id,
				comp_info.auc_comp_idx[loop] + 1,
				frame_done_addr_1stsensor[
				comp_info.auc_comp_idx[loop]],
				g_bist_buff_kva +
				(u64) (frame_done_addr_1stsensor[
				comp_info.auc_comp_idx[loop]] -
				g_bist_buff_iva));

			self_test_check_sum(g_bist_buff_kva +
				(u64)(frame_done_addr_1stsensor[
				comp_info.auc_comp_idx[loop]] -
				g_bist_buff_iva),
				2 * out_width_1stsensor[
				comp_info.auc_comp_idx[loop]] *
				out_height_1stsensor[
				comp_info.auc_comp_idx[loop]],
				&sum_frame_done_1stsensor,
				&xor_frame_done_1stsensor);

			isp_info_item_tag("Sum: 0x%08x, Xor: 0x%08x\n",
				sum_frame_done_1stsensor,
				xor_frame_done_1stsensor);

			if (g_self_test_scenario[scenario_index] ==
				V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT) {

				isp_info_item_tag(
					"Sensor %d IMG%d Addr%llx\n",
					g_bist_2ndsensor_id,
					comp_info.auc_comp_idx[loop] + 1,
					golden_addr_1stsensor[
					comp_info.auc_comp_idx[loop]]);

				self_test_check_sum(frame_done_addr_2ndsensor[
					comp_info.auc_comp_idx[loop]],
					2 * out_width_2ndsensor[
					comp_info.auc_comp_idx[loop]] *
					out_height_2ndsensor[
					comp_info.auc_comp_idx[loop]],
					&sum_frame_done_2ndsensor,
					&xor_frame_done_2ndsensor);
				if (sum_golden ==
					sum_frame_done_1stsensor &&
					xor_golden ==
					xor_frame_done_1stsensor)
					isp_info_desc_tag(
						"1st Sensor IMG %d, Match\n",
						comp_info.auc_comp_idx[loop] +
						1);
				else
					isp_info_desc_tag(
						"1st Sensor IMG %d, NoMatch\n",
						comp_info.auc_comp_idx[loop] +
						1);
				if (sum_golden ==
					sum_frame_done_2ndsensor &&
					xor_golden == xor_frame_done_2ndsensor)
					isp_info_desc_tag(
						"2nd Sensor IMG %d, Match\n",
						comp_info.auc_comp_idx[loop] +
						1);
				else
					isp_info_desc_tag(
						"2nd Sensor IMG %d, Not Match\n",
						comp_info.auc_comp_idx[loop] +
						1);
			} else {
				if (sum_golden ==
					sum_frame_done_1stsensor &&
					xor_golden == xor_frame_done_1stsensor)

					isp_info_desc_tag(
						"IMG %d, Match!\n\n",
						comp_info.auc_comp_idx[loop] +
						1);
				else
					isp_info_desc_tag(
						"IMG %d, Not Match!\n\n",
						comp_info.auc_comp_idx[loop] +
						1);
			}
		}

		isp_info_desc_tag("BIST: Compare Finish\n");

		/* Close ISP*/
		ispdrv_close();

		/* Wait 42ms for HW idle by Caedmon*/
		SLEEPRANGE(42000, 50000);

		/* Open ISP for testing the next Scenario ID*/
		if ((total_test - 1) != i)
			ispdrv_open(bist_get_ahb_addr(), bist_get_ahb_size(),
				bist_get_v4l2_device());
		else
			isp_info_desc_tag("BIST END\n");

	}

self_test_end:

	isp_info_lo_end_tag();
	return err;

}
EXPORT_SYMBOL(ispdrv_self_test);

/***********************************************************
*		public function		  *
*************************************************************/
/*
 *\brief Set the starting address of ahb register
 *\param ahb_addr [In], the starting address of ahb register
 *\return null
 */
void bist_set_ahb_addr(u64 ahb_addr)
{
	g_bist_ahb_addr = ahb_addr;
}

/*
 *\brief Get the starting address of ahb register
 *\param ahb_addr [Out], the starting address of ahb register
 *\return null
 */
u64 bist_get_ahb_addr(void)
{
	return g_bist_ahb_addr;
}

/*
 *\brief Set the size of ahb register
 *\param ahb_size [In], the size of ahb register
 *\return null
 */
void bist_set_ahb_size(u32 ahb_size)
{
	g_bist_ahb_size = ahb_size;
}

/*
 *\brief Get the size of ahb register
 *\param ahb_size [Out], the size of ahb register
 *\return null
 */
u32 bist_get_ahb_size(void)
{
	return g_bist_ahb_size;
}

/*
 *\brief Set the device info
 *\param bist_v4l2_device [In], the device info
 *\return null
 */
void bist_set_v4l2_device(void *bist_v4l2_device)
{
	g_bist_v4l2_device = bist_v4l2_device;
}

/*
 *\brief Get the device info
 *\param bist_v4l2_device [Out], the device info
 *\return null
 */
void *bist_get_v4l2_device(void)
{
	return g_bist_v4l2_device;
}

/*
 *\brief Get the input size
 *\param sensor_index [In], the sensor index, 1st sensor = 1, 2nd sensor = 2
 *\param input_width [Out], the input width
 *\param input_height [Out], the input height
 *\return null
 */
void bist_get_input_size(u8 sensor_index,
		u16 *input_width, u16 *input_height)
{
	if (sensor_index == 1) {
		*input_width = g_bist_1stsensor_input_width;
		*input_height = g_bist_1stsensor_input_height;
	} else if (sensor_index == 2) {
		*input_width = g_bist_2ndsensor_input_width;
		*input_height = g_bist_2ndsensor_input_height;
	}
}

/*
 *\brief Get the input size
 *\param sensor_index [In], the sensor index, 1st sensor = 1, 2nd sensor = 2
 *\param input_width [In], the input width
 *\param input_height [In], the input height
 *\return null
 */
void bist_set_input_size(u8 sensor_index,
		u16 input_width, u16 input_height)
{
	if (sensor_index == 1) {
		g_bist_1stsensor_input_width = input_width;
		g_bist_1stsensor_input_height = input_height;
	} else if (sensor_index == 2) {
		g_bist_2ndsensor_input_width = input_width;
		g_bist_2ndsensor_input_height = input_height;
	}
}

/*
 *\brief check the sensor id
 *\param sensor_index [In], the sensor index, 1st sensor = 1, 2nd sensor = 2
 *\param sensor_id [In], the sensor id
 *\return result
 */
bool bist_check_sensor_id(u8 sensor_index, u8 sensor_id)
{
	bool result = false;

	if (sensor_index == 1) {
		if (sensor_id == g_bist_1stsensor_id)
			result = true;
		else
			result = false;
	} else if (sensor_index == 2) {
		if (sensor_id == g_bist_2ndsensor_id)
			result = true;
		else
			result = false;
	}
	return result;
}

/*
 *\brief get the addresses of golden images
 *\param direct_addr [In],
 *		the address of single direct mode and stripe direct mode
 *\param dram_lv_video_addr [In],
 *		the address of the lv and video images of dram mode
 *\param dram_normal_quality_still_addr [In],
 *		the address of still image in normal quality dram mode
 *\param dram_high_quality_still_addr [In],
 *		the address of still image in high quality dram mode
 *\param front_addr [In], the address of front mode
*\return result
 */
void bist_get_golden_addr(u64 *direct_addr, u64 *dram_lv_video_addr,
		u64 *dram_normal_quality_still_addr,
		u64 *dram_high_quality_still_addr,
		u64 *front_addr)
{
	*direct_addr = g_bist_direct_kva;
	*dram_lv_video_addr = g_bist_dram_lv_video_kva;
	*dram_normal_quality_still_addr = g_bist_dram_normal_quality_still_kva;
	*dram_high_quality_still_addr = g_bist_dram_high_quality_still_kva;
	*front_addr = g_bist_front_kva;
}

/*
 *\brief check the addresses used in bist is set
 *\param none
*\return the flag, 0 = not set, 1 = set
 */
u8 bist_get_self_test_addr_set(void)
{
	return g_is_selftest_addr_set;
}

/*
 *\brief set the flag indicated the addresses used in bist is set
 *\param none
*\return none
 */
void bist_set_self_test_addr_set(void)
{
	g_is_selftest_addr_set = 1;
}

/*
 *\brief reset the flag indicated whether the addresses used in bist is set
 *\param none
*\return none
 */
void bist_reset_self_test_addr_set(void)
{
	g_is_selftest_addr_set = 0;
}

/*
 *\brief reset the first sensor id and the second sensor id
 *\param none
*\return none
 */
void bist_reset_sensor_id(void)
{
	g_bist_1stsensor_id = 0;
	g_bist_2ndsensor_id = 0;
}
/*
 *\brief add the frame done count
 *	    and set frame done flag if the count is satisfied
 *\param scenario_info [In], the scenario info
 *\param frame_done_cnt [In], the frame dont count
 *\param sensor_idx [In], the sensor index,
 *		1 = first sensor, 2 = second sensor
*\return none
 */
void bist_set_frame_done_count(
		struct s_scenario_info_isp *scenario_info,
		u8 *frame_done_cnt,
		u8 sensor_idx)
{

	isp_info_lo_start_tag();

#if 0
	(*frame_done_cnt)++;
	if (sensor_idx == 1) {
		if (false ==
			scenario_info->self_test_info.is_frame_done_1stsensor &&
			scenario_info->self_test_info.max_frame_cnt ==
			*frame_done_cnt)

			scenario_info->self_test_info.is_frame_done_1stsensor =
			true;

		isp_info_item_tag(
			"BIST: 1st sensor, cnt %d/%d, isTrue %d\n",
			*frame_done_cnt, common_get_1stsensor_frame_done_cnt(),
			scenario_info->self_test_info.is_frame_done_1stsensor);
	} else {
		if (false ==
		    scenario_info->self_test_info.is_frame_done_2ndsensor &&
		    scenario_info->self_test_info.max_frame_cnt ==
		    *frame_done_cnt)
			scenario_info->self_test_info.is_frame_done_2ndsensor =
			true;
		isp_info_item_tag(
			"BIST: 2nd sensor, cnt %d/%d, isTrue %d\n",
			*frame_done_cnt, common_get_2ndsensor_frame_done_cnt(),
			scenario_info->self_test_info.is_frame_done_2ndsensor);
	}
#endif

	if (sensor_idx == 1) {
		isp_info_item_tag("BIST: FD Cnt %d\n",
			scenario_info->self_test_info.frame_count_1stsensor);

		if (scenario_info->self_test_info.max_frame_cnt <=
			scenario_info->self_test_info.frame_count_1stsensor) {

			scenario_info->
				self_test_info.is_frame_done_1stsensor = true;
			isp_info_desc_tag("BIST: Set FD CNT True\n");
		}
	} else {
		if (scenario_info->self_test_info.max_frame_cnt <=
			scenario_info->self_test_info.frame_count_2ndsensor) {

			scenario_info->
				self_test_info.is_frame_done_2ndsensor = true;
		}
	}

	isp_info_lo_end_tag();
}

/*
 *\brief set wait flag if not frame done
 *\param scenario_id [In], the scenario id
 *\param scenario_info [In], the scenario_info
*\return true/false
 */
bool bist_check_frame_done(u8 scenario_id,
		struct s_scenario_info_isp *scenario_info)
{
	bool result = false;

	if (scenario_id == V4L2_ISP_SCENARIO_PREVIEW_STILL_SS) {
		/* do nothing */
	} else if (scenario_id == V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT) {
		if (false ==
			scenario_info->
			self_test_info.is_frame_done_1stsensor ||
			false ==
			scenario_info->self_test_info.is_frame_done_2ndsensor)

			result = true;
	} else {
		if (false ==
			scenario_info->self_test_info.is_frame_done_1stsensor)

			result = true;
	}

	return result;
}

/*
 *\brief get the dual sensor mode
 *\param scenario_info [In], the scenario_info
*\return true/false
 */
bool bist_is_dual_sensor_mode(struct s_scenario_info_isp *scenario_info)
{
	return scenario_info->self_test_info.is_dual_sensor_mode;
}

bool bist_is_independent_mode(struct s_scenario_info_isp *scenario_info)
{
	return scenario_info->self_test_info.is_independent;
}

/*
 *\brief get the self test mode
 *\param scenario_info [In], the scenario_info
*\return true/false
 */
u8 bist_get_self_test_mode(struct s_scenario_info_isp *scenario_info)
{
	return scenario_info->self_test_info.self_test_mode;
}

/*
 *\brief set the flag of indicating independent mode
 *\param is_enable [In], the flag
*\return none
 */
void bist_set_independent_flag(bool is_enable)
{
	/* Set independent mode */
	g_is_independent_mode = is_enable;
}

/*
 *\brief get the flag of indicating independent mode
 *\param none
*\return the flag
 */
bool bist_get_independent_flag(void)
{
	return g_is_independent_mode;
}

/*
 *\brief set the flag of reversing the closing sequence in independent mode
 *\param is_reverse_close_sequence [In], the flag
*\return none
 */
void bist_set_reverse_flag(bool is_reverse_close_sequence)
{
	g_is_reverse_close_sequence = is_reverse_close_sequence;
}

/*
 *\brief get the flag of reversing the closing sequence in independent mode
 *\param None
*\return the flag
 */
bool bist_get_reverse_flag(void)
{
	return g_is_reverse_close_sequence;
}

/*
 *\brief analysis the interrupted flags of related frame done INT for BIST
 *\param flags [In], the interrupted flags
 *\param scenario_info [Out], the scenario info
 *\param frame_done_count_1stsensor [Out],
	     the frame done count of first sensor
 *\param frame_done_count_2ndsensor [Out],
	     the frame done count of second sensor
 *\return None
 */
u32 bist_irq_interrupt(u32 flags,
			  struct s_scenario_info_isp *scenario_info,
			  u8 *frame_done_count_1stsensor,
			  u8 *frame_done_count_2ndsensor)
{
	u32 err = 0;
	u8 sensor_id = SENSOR_TOTAL;
	static u8 fw_proc_still_start_int_index;
	static u8 proc_still_frame_done_int_index;

	isp_info_lo_start_tag();

	/* Sensor 1 still image frame done*/
	if (flags & V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT) {
		sensor_id = SENSOR1;
		isp_info_desc_tag("BIST: sensor 1 still FD\n");
		if (g_bist_scenario_id != V4L2_ISP_SCENARIO_PREVIEW_STILL_SS)
			bist_interrupt_handle(sensor_id,
				scenario_info,
				frame_done_count_1stsensor,
				frame_done_count_2ndsensor);
		else if (g_bist_proc_still_fw_start[
				proc_still_frame_done_int_index] == true) {
			isp_info_desc_tag("BIST: proc still FD\n");
			proc_still_frame_done_int_index++;
			sensor_id = SENSOR1;
			err = bist_proc_still_frame_done_irq_proc(
					sensor_id, V4L2_ISP_IMG3);
			if (err != 0) {
				isp_info_desc_tag("BIST: raw irq proc err\n");
				err = EINVAL;
				goto irq_interrupt_end;
			}
		}
	}

	/* Sensor 2 still image frame done*/
	if (flags & V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT) {
		sensor_id = SENSOR2;
		bist_interrupt_handle(sensor_id,
				scenario_info,
				frame_done_count_1stsensor,
				frame_done_count_2ndsensor);
	}

	/* Sensor 3 LV image frame done*/
	if (flags & V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT) {
		sensor_id = SENSOR3;
		bist_interrupt_handle(sensor_id,
				scenario_info,
				frame_done_count_1stsensor,
				frame_done_count_2ndsensor);
	}

	/* Sensor 1 raw image frame done for SS path */
	if (flags & V4L2_ISP_SENSOR1_RAW_DONE_INT) {
		/* isp_info("BIST: SS Raw Frame Done\n"); */
		sensor_id = SENSOR1;
		err = bist_proc_still_frame_done_irq_proc(
				sensor_id, V4L2_ISP_RAW);
		if (err != 0) {
			isp_info_desc_tag("BIST: raw irq proc err\n");
			err = EINVAL;
			goto irq_interrupt_end;
		}
	}
	if (flags & V4L2_ISP_SENSOR1_STILL_PROC_START_INT) {
		isp_info_desc_tag("BIST: fw start proc still No.%d\n",
			fw_proc_still_start_int_index);
		g_bist_proc_still_fw_start[
			fw_proc_still_start_int_index++] = true;
	}


irq_interrupt_end:

	isp_info_lo_end_tag();
	return err;
}

/*
 *\brief handle the interrupt of related frame done INT for BIST
 *\param sensor_id [In], the sensor id
 *\param scenario_info [Out], the scenario info
 *\param frame_done_count_1stsensor [Out],
	     the frame done count of first sensor
 *\param frame_done_count_2ndsensor [Out],
	     the frame done count of second sensor
 *\return None
 */
void bist_interrupt_handle(u8 sensor_id,
			     struct s_scenario_info_isp *scenario_info,
			     u8 *frame_done_count_1stsensor,
			     u8 *frame_done_count_2ndsensor)
{
	u8 sensor_index = 0;
	/* Get sensor index */
	if (bist_check_sensor_id(1, sensor_id))
		sensor_index = 1;
	else if (true ==
		bist_is_dual_sensor_mode(scenario_info) &&
		bist_check_sensor_id(2, sensor_id))
		sensor_index = 2;
	else if (true ==
		bist_is_independent_mode(scenario_info) &&
		bist_check_sensor_id(2, sensor_id))
		sensor_index = 2;
	else
		goto bist_check_irq_flag_end;

	/* check the conditions of waiting frame done are met */
	if (sensor_index == 1)
		bist_set_frame_done_count(scenario_info,
				frame_done_count_1stsensor, sensor_index);
	else
		bist_set_frame_done_count(scenario_info,
				frame_done_count_2ndsensor, sensor_index);
bist_check_irq_flag_end:
	return;
}

u32 bist_proc_still_frame_done_irq_proc(u8 sensor_id, u8 image_type)
{
	u32 err = 0;
	static u32 raw_frame_done_count;
	static u8 proc_still_index_raw;
	static u8 proc_still_index_still;

	bool is_more_img_ready = false;
	u32 get_out_image_addr = 0;
	u32 set_back_image_addr = 0;
	u8 temp_index = 0;
	struct s_iq_info_1 *iq_info_1;
	struct s_iq_info_2 *iq_info_2;
	struct s_scenario_size img_size;
	/* struct s_raw_info_setting raw_info_setting; */

	isp_info_lo_start_tag();

	iq_info_1 = kmalloc(sizeof(struct s_iq_info_1), GFP_KERNEL);
	memset((u8 *) iq_info_1, 0, sizeof(struct s_iq_info_1));
	iq_info_2 = kmalloc(sizeof(struct s_iq_info_2), GFP_KERNEL);
	memset((u8 *) iq_info_2, 0, sizeof(struct s_iq_info_2));

	memset((u8 *) &img_size, 0, sizeof(struct s_scenario_size));

	if (image_type == V4L2_ISP_RAW &&
		raw_frame_done_count >= SS_SECOND_RAW_FRAME_IDX) {
		/* isp_info("BIST: Raw FD %d\n", raw_frame_done_count); */
		goto bist_raw_irq_proc_end;
	}
	if (image_type == V4L2_ISP_RAW) {
		/* check  there is more image ready to be gotten*/
		err = ispdrv_is_more_img_ready(
			sensor_id, image_type, &is_more_img_ready);
		if (!is_more_img_ready) {
			isp_info_desc_tag("BIST: No more img ready\n");
			err = EINVAL;
			goto bist_raw_irq_proc_end;
		}
	}

	/* isp_info("BIST:FD irq proc Get Buffer\n"); */
	if (image_type == V4L2_ISP_RAW) {
		/* isp_info("BIST: Raw FD %d\n", raw_frame_done_count);
		 * get_out_image_addr =
		 *	&g_bist_proc_still_src_raw_iva[proc_still_index_raw];
		 */
		set_back_image_addr = g_bist_1stsensor_raw_img_iva +
			    SS_RAW_BUFFER_SIZE * (3 + 1 + proc_still_index_raw);
		/* iq_info_1 = &g_ss_raw_iq_info_1[proc_still_index_raw]; */
		/* iq_info_2 = &g_ss_raw_iq_info_2[proc_still_index_raw]; */
		/* img_size = &g_ss_raw_crop_size[proc_still_index_raw]; */
		temp_index = proc_still_index_raw;
		/* Count raw frame done interrupt */
		raw_frame_done_count++;
	} else if (image_type == V4L2_ISP_IMG3) {
		/* get_out_image_addr =
		 *	&g_bist_output_still_iva[
		 *	proc_still_index_raw].buffer_addr;
		 */
		/* iq_info_1 = &g_ss_still_iq_info_1[proc_still_index_raw]; */
		/* iq_info_2 = &g_ss_still_iq_info_2[proc_still_index_raw]; */
		/* img_size = &g_ss_still_crop_size[proc_still_index_raw]; */
		temp_index = proc_still_index_still;
	}

	err = ispdrv_get_output_buffer(sensor_id, image_type,
				   &get_out_image_addr, NULL, NULL, NULL,
				   &img_size, iq_info_1,
				   iq_info_2);
	if (err != 0) {
		isp_info_desc_tag("BIST: get output buff err\n");
		err = EINVAL;
		goto bist_raw_irq_proc_end;
	}

	if (image_type == V4L2_ISP_IMG3) {
		g_bist_output_still_iva[temp_index].buffer_addr =
			get_out_image_addr;
		MEMCPY((u8 *)&g_ss_still_iq_info_1[temp_index],
			(u8 *)iq_info_1, sizeof(struct s_iq_info_1));
		MEMCPY((u8 *)&g_ss_still_iq_info_2[temp_index],
			(u8 *)iq_info_2, sizeof(struct s_iq_info_2));
		MEMCPY((u8 *)&g_ss_still_crop_size[temp_index],
			(u8 *)&img_size, sizeof(struct s_scenario_size));
		g_bist_proc_still_end[temp_index] = true;
		if (temp_index < 1)
			temp_index++;

		goto bist_raw_irq_proc_update_index_end;
	} else if (image_type == V4L2_ISP_RAW &&
			(raw_frame_done_count == SS_FIRST_RAW_FRAME_IDX ||
			raw_frame_done_count == SS_SECOND_RAW_FRAME_IDX)) {

		isp_info_desc_tag("BIST:FD irq proc Set Buffer\n");
		err = ispdrv_set_output_buffer(sensor_id, image_type,
				set_back_image_addr, 0, 0, 0);
		if (err != 0) {
			isp_info_desc_tag("BIST: set output buff err\n");
			err = EINVAL;
			goto bist_raw_irq_proc_update_index_end;
	}
	g_bist_proc_still_src_raw_iva[temp_index] = get_out_image_addr;
	MEMCPY((u8 *)&g_ss_raw_iq_info_1[temp_index],
		(u8 *)iq_info_1, sizeof(struct s_iq_info_1));
	MEMCPY((u8 *)&g_ss_raw_iq_info_2[temp_index],
		(u8 *)iq_info_2, sizeof(struct s_iq_info_2));
	MEMCPY((u8 *)&g_ss_raw_crop_size[temp_index],
		(u8 *)&img_size, sizeof(struct s_scenario_size));

	/* Set the flag after copy the parameters for proc_still API */
	g_bist_proc_still_start[temp_index] = true;

	if (temp_index < 1)
		temp_index++;

	} else {
		err = ispdrv_set_output_buffer(sensor_id, image_type,
			get_out_image_addr, 0, 0, 0);
		if (err != 0) {
			isp_info_desc_tag("BIST: set output buff err\n");
			err = EINVAL;
			goto bist_raw_irq_proc_update_index_end;
		}
	}

bist_raw_irq_proc_update_index_end:

	if (image_type == V4L2_ISP_RAW)
		proc_still_index_raw = temp_index;
	else if (image_type == V4L2_ISP_IMG3)
		proc_still_index_still = temp_index;


bist_raw_irq_proc_end:

	kfree((void *)iq_info_1);
	kfree((void *)iq_info_2);

	isp_info_lo_end_tag();
	return err;
}

void bist_init(void *pri_data)
{
	bist_reset_sensor_id();
	bist_reset_self_test_addr_set();
	bist_set_input_size(1, 0, 0);
	bist_set_input_size(2, 0, 0);
}

void bist_deinit(void *pri_data)
{

}
/***********************************************************
*		private function		  *
*************************************************************/
#if 0
static void self_test_fill_parameter_ss(u8 image_type,
		    u8 proc_index)
{
	bool *destination_state = NULL;
	struct s_scenario_size *destination_crop_size = NULL;

	isp_info_lo_start_tag();

	if (image_type == V4L2_ISP_RAW) {
		destination_crop_size = &g_ss_raw_crop_size[proc_index];
		destination_state = &g_bist_proc_still_start[proc_index];
		g_bist_proc_still_src_raw_iva[proc_index] = image_addr;
	} else if (image_type == V4L2_ISP_IMG3) {
		destination_crop_size = &g_ss_still_crop_size[proc_index];
		destination_state = &g_bist_proc_still_end[proc_index];
		g_bist_output_still_iva[proc_index].buffer_addr = image_addr;
	} else
		goto self_test_fill_param_end;

	MEMCPY((u8 *) destination_crop_size, (u8 *)img_size,
		sizeof(struct s_scenario_size));
	*destination_state = true;

	isp_info_item_tag("BIST: Raw FD %d, Still FD %d\n",
		g_bist_proc_still_start[proc_index],
		g_bist_proc_still_end[proc_index]);

self_test_fill_param_end:

	isp_info_lo_end_tag();
	return;

}
#endif

static void self_test_fill_ss_hq_opt_param(struct ispdrv_proc_still_opt *ss_opt)
{
	ss_opt->quality_path = ISP_HIGH_QUALITY;
	/* Inner format in proc_still, */
	/* it must be YCC_420 if the cfr is enable */
	ss_opt->hq_setting.ycc_format = ISP_YCC_422;
	ss_opt->hq_setting.cfr_enabled = 0;
	/* 20160523, YNRE cannot disable since */
	/* Joen's code is depended on YNRE */
	ss_opt->hq_setting.y_nr_enabled = 1;
	ss_opt->hq_setting.sharpness_enabled = 1;
	ss_opt->hq_setting.writing_mode = ISP_LINE_WRITING_MODE;
}

static u32 self_test_proc_still(void)
{
	u32 err = 0;
	u8 proc_still_index = 0;

	isp_info_lo_start_tag();

	for (proc_still_index = 0; proc_still_index < 2; proc_still_index++) {
		while (g_bist_proc_still_start[proc_still_index] == false) {
			isp_info_desc_tag("BIST:wait no.%d raw frame done\n",
				proc_still_index);
			SLEEPRANGE(1000000, 1005000);
		}
		isp_info_desc_tag("BIST:Raw frame done Success\n");

		self_test_fill_ss_hq_opt_param(&g_ss_opt[proc_still_index]);

		#if 0

		if (proc_still_index == 0) {
			isp_info_desc_tag("BIST:Change raw frame rate\n");
			err = ispdrv_set_raw_frame_rate(
					g_bist_1stsensor_id, ISP_RAW_FR_4);
			if (err != 0) {
				isp_info_desc_tag(
					"BIST:Set Raw Frame Rate Err\n");
				goto self_test_proc_still_end;
			}
		}
		#endif

		/* Proc still for 1st Raw */
		err = ispdrv_proc_still(g_bist_1stsensor_id,
				g_bist_proc_still_src_raw_iva[proc_still_index],
				g_bist_proc_still_iva[proc_still_index],
				&g_ss_opt[proc_still_index],
				&g_ss_raw_iq_info_1[proc_still_index],
				&g_ss_raw_iq_info_2[proc_still_index]);
		if (err != 0) {
			isp_info_desc_tag("BIST:Proc Still no.%d Err\n",
				proc_still_index);
			goto self_test_proc_still_end;
		}
		while (g_bist_proc_still_end[proc_still_index] == false) {
			isp_info_desc_tag("BIST:wait no.%d still frame done\n",
				proc_still_index);
			SLEEPRANGE(1000000, 1005000);
		}
		isp_info_item_tag("BIST:output addr 0x%x\n",
			g_bist_output_still_iva[proc_still_index].buffer_addr);

		isp_info_desc_tag("BIST:Proc_Still Success %d\n",
			proc_still_index);

	}

self_test_proc_still_end:

	isp_info_lo_end_tag();
	return err;
}

static void self_test_check_sum(u64 addr, u32 size,
		u32 *result_sum, u32 *result_xor)
{
	u32 i;
	u64 start_addr;
	u32 tmp_size = size;
	u64 img_addr = addr;
	u32 tmp_sum = 0;
	u32 tmp_xor = 0;

	isp_info_lo_start_tag();

	*result_sum = 0;
	*result_xor = 0;

	for (i = 0; i < tmp_size; i += PAGE_SIZE_4K) {
		start_addr = img_addr + i;
		if ((i + PAGE_SIZE_4K) <= tmp_size)
			get_sum_xor_value((u8 *) start_addr, PAGE_SIZE_4K,
			&tmp_sum, &tmp_xor);
		else
			get_sum_xor_value((u8 *) start_addr, tmp_size - i,
			&tmp_sum, &tmp_xor);

		*result_sum += tmp_sum;
		*result_xor ^= tmp_xor;
	}

	isp_info_lo_end_tag();
}

static void self_test_check_sum_ss(u64 addr, u32 size,
	const u32 image_width, const u32 image_height,
	u32 *result_sum, u32 *result_xor)
{
	u32 i;
	u64 start_addr;
	u32 tmp_size = size;
	u32 shift_offset;
	u32 compare_offset;
	u32 tmp_dummy;
	u64 img_addr = addr;
	u32 tmp_sum = 0;
	u32 tmp_xor = 0;

	isp_info_lo_start_tag();

	*result_sum = 0;
	*result_xor = 0;

	isp_info_desc_tag("BIST: width %d, height %d",
			image_width, image_height);

	/* Y */
	tmp_dummy = 8;
	compare_offset = image_width;
	shift_offset = compare_offset + tmp_dummy;
	isp_info_desc_tag("BIST: Y - compare %d, shift %d, dummy %d",
		compare_offset, shift_offset, tmp_dummy);
	for (i = 0; i < (shift_offset * image_height); i += shift_offset) {
		start_addr = img_addr + i;
		if ((i + compare_offset) <=
			(shift_offset * image_height - tmp_dummy))
			get_sum_xor_value((u8 *) start_addr, compare_offset,
			&tmp_sum, &tmp_xor);
		else if ((shift_offset * image_height - i) > tmp_dummy)
			get_sum_xor_value((u8 *) start_addr,
			shift_offset * image_height - tmp_dummy - i,
			&tmp_sum, &tmp_xor);
		else
			break;
		*result_sum += tmp_sum;
		*result_xor ^= tmp_xor;
	}

	tmp_size = i = i + tmp_dummy;
	isp_info_desc_tag("BIST: SS Y size 0x%x", tmp_size);
	isp_info_desc_tag("BIST: CheckSum i 0x%x", i);

	tmp_dummy = ALIGN_CEIL((image_width / 2), 8) - image_width / 2;
	compare_offset = image_width / 2;
	shift_offset = compare_offset + tmp_dummy;
	isp_info_desc_tag("BIST: CbCr - compare %d, shift %d, dummy %d",
		compare_offset, shift_offset, tmp_dummy);
	tmp_size += ((shift_offset * image_height / 2) * 2);
	isp_info_desc_tag("BIST: SS Total size 0x%x", tmp_size);

	for (; i < tmp_size; i += shift_offset) {
		start_addr = img_addr + i;
		if ((i + compare_offset) <= tmp_size)
			get_sum_xor_value((u8 *) start_addr, compare_offset,
			&tmp_sum, &tmp_xor);
		else if ((i + tmp_dummy) < tmp_size)
			get_sum_xor_value((u8 *) start_addr,
				tmp_size - tmp_dummy - i,
				&tmp_sum, &tmp_xor);
		else
			break;
		*result_sum += tmp_sum;
		*result_xor ^= tmp_xor;
	}

	isp_info_lo_end_tag();
}

static u32 self_test_set_awb_gain_settings(u8 sensor_id)
{
	u32 err = 0;
	struct awb_gain_info t_self_test_awb_gain_info;

	memset((u8 *) &t_self_test_awb_gain_info, 0,
	sizeof(struct awb_gain_info));
	t_self_test_awb_gain_info.detected_awb_r = 500; /* 457; */
	t_self_test_awb_gain_info.detected_awb_gr = 256;
	t_self_test_awb_gain_info.detected_awb_gb = 256;
	t_self_test_awb_gain_info.detected_awb_b = 500; /* 368; */

	err = ispdrv_set_awb_gain(sensor_id, &t_self_test_awb_gain_info);

	return err;
}

static u32 self_test_set_3a_settings(u8 sensor_id, u16 sensor_height)
{
	u32 err = 0;
	struct cfg_3a_info *t_self_test_3a_info;

	u8 t_self_test_af_info_auc_weight[6] = { 0, 2, 6, 4, 1, 2 };
	u8 t_self_test_af_info_auc_index[82] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0
	};
	u16 t_self_test_af_info_auw_th[4] = { 0, 0, 0, 0 };
	u16 t_self_test_af_info_pw_tv[4] = { 0, 0, 0, 0 };
	u8 t_self_test_awb_info_uc_y_factor[16] = {
		0, 0, 0, 4, 7, 10, 12, 14, 15, 15, 15, 15, 14, 13, 10, 5 };
	s8 t_self_test_awb_info_bbr_factor[33] = {
		22, 20, 18, 16, 15, 13, 11, 10, 8, 8, 6, 5, 3, 1, -1, -3,
		-4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -15,
		-16, -18, -18, -18, -18
	};

	t_self_test_3a_info = kmalloc(sizeof(struct cfg_3a_info), GFP_KERNEL);
	memset((u8 *) t_self_test_3a_info, 0, sizeof(struct cfg_3a_info));

	/* Example 3A cfg*/
	/*  AF	*/
	t_self_test_3a_info->af_info.token_id = 0x100;
	t_self_test_3a_info->af_info.af_region.size_ratio_x = 33;
	t_self_test_3a_info->af_info.af_region.size_ratio_y = 33;
	t_self_test_3a_info->af_info.af_region.offset_ratio_x = 33;
	t_self_test_3a_info->af_info.af_region.offset_ratio_y = 33;
	t_self_test_3a_info->af_info.af_region.blk_num_x = 3;
	t_self_test_3a_info->af_info.af_region.blk_num_y = 3;
	t_self_test_3a_info->af_info.enable_af_lut = false;
	/* t3AInfo.ae_info.auwLUT[0] = NULL;*/
	/* t3AInfo.ae_info.auwAFLUT[0] = NULL;*/
	MEMCPY((u8 *) t_self_test_3a_info->af_info.weight,
		(u8 *) &t_self_test_af_info_auc_weight, 6);
	t_self_test_3a_info->af_info.sh = 0;
	t_self_test_3a_info->af_info.th_mode = 0;
	MEMCPY((u8 *) t_self_test_3a_info->af_info.index,
		(u8 *) &t_self_test_af_info_auc_index, 82);
	MEMCPY((u8 *) t_self_test_3a_info->af_info.th,
		(u8 *) &t_self_test_af_info_auw_th, 4 * 2);
	MEMCPY((u8 *) t_self_test_3a_info->af_info.tv,
		(u8 *) &t_self_test_af_info_pw_tv, 4 * 2);
	/*  AWB*/
	t_self_test_3a_info->af_info.af_offset = 0;
	t_self_test_3a_info->af_info.af_py_enable = false;
	t_self_test_3a_info->af_info.af_lpf_enable = false;
	t_self_test_3a_info->af_info.filter_mode = 2;
	t_self_test_3a_info->af_info.filter_id = 0;
	t_self_test_3a_info->af_info.line_cnt = sensor_height/2;
	#if 0
	t_self_test_3a_info->af_info.af_hw3aa_iir[0].af_hpf_initbyuser = false;
	t_self_test_3a_info->af_info.af_hw3aa_iir[1].af_hpf_initbyuser = false;
	t_self_test_3a_info->af_info.af_hw3aa_lpf.af_lpf_b0 = 0x1ac;
	t_self_test_3a_info->af_info.af_hw3aa_lpf.af_lpf_b1 = 0x12e;
	t_self_test_3a_info->af_info.af_hw3aa_lpf.af_lpf_b2 = 0x1dc;
	t_self_test_3a_info->af_info.af_hw3aa_lpf.af_lpf_b3 = 0x248;
	t_self_test_3a_info->af_info.af_hw3aa_lpf.af_lpf_b4 = 0x262;
	t_self_test_3a_info->af_info.af_hw3aa_lpf.af_lpf_b5 = 0x259;
	t_self_test_3a_info->af_info.af_hw3aa_lpf.af_lpf_b6 = 0x16e;
	t_self_test_3a_info->af_info.af_hw3aa_lpf.af_lpf_shift = 0x159;
	#endif
	/*  AWB*/
	t_self_test_3a_info->awb_info.token_id = 0x100;
	t_self_test_3a_info->awb_info.awb_region.border_ratio_x = 100;
	t_self_test_3a_info->awb_info.awb_region.border_ratio_y = 100;
	t_self_test_3a_info->awb_info.awb_region.blk_num_x = 64;
	t_self_test_3a_info->awb_info.awb_region.blk_num_y = 48;
	t_self_test_3a_info->awb_info.awb_region.offset_ratio_x = 0;
	t_self_test_3a_info->awb_info.awb_region.offset_ratio_y = 0;
	MEMCPY((u8 *) t_self_test_3a_info->awb_info.y_factor,
		(u8 *) t_self_test_awb_info_uc_y_factor, 16);
	MEMCPY((u8 *) t_self_test_3a_info->awb_info.bbr_factor,
		(u8 *) t_self_test_awb_info_bbr_factor, 33);
	t_self_test_3a_info->awb_info.r_gain = 1103;
	t_self_test_3a_info->awb_info.g_gain = 0;
	t_self_test_3a_info->awb_info.b_gain = 402;
	t_self_test_3a_info->awb_info.cr_shift = 100;
	t_self_test_3a_info->awb_info.offset_shift = 100;
	t_self_test_3a_info->awb_info.quantize = 0;
	t_self_test_3a_info->awb_info.damp = 7;
	t_self_test_3a_info->awb_info.sum_shift = 5;
	t_self_test_3a_info->awb_info.t_his.enable = true;
	t_self_test_3a_info->awb_info.t_his.cr_start = -46;
	t_self_test_3a_info->awb_info.t_his.cr_end = 110;
	t_self_test_3a_info->awb_info.t_his.offset_up = 10;
	t_self_test_3a_info->awb_info.t_his.offset_down = -90;
	t_self_test_3a_info->awb_info.t_his.cr_purple = 0;
	t_self_test_3a_info->awb_info.t_his.offset_purple = 2;
	t_self_test_3a_info->awb_info.t_his.grass_offset = -22;
	t_self_test_3a_info->awb_info.t_his.grass_start = -30;
	t_self_test_3a_info->awb_info.t_his.grass_end = 25;
	t_self_test_3a_info->awb_info.t_his.damp_grass = 4;
	t_self_test_3a_info->awb_info.t_his.offset_bbr_w_start = -2;
	t_self_test_3a_info->awb_info.t_his.offset_bbr_w_end = 2;
	t_self_test_3a_info->awb_info.t_his.yfac_w = 2;
	t_self_test_3a_info->awb_info.t_his.his_interp = -178;
	t_self_test_3a_info->awb_info.r_linear_gain = 270;
	t_self_test_3a_info->awb_info.b_linear_gain = 168;
	/*  AE*/
	t_self_test_3a_info->ae_info.token_id = 0x111;
	t_self_test_3a_info->ae_info.ae_region.border_ratio_x = 100;
	t_self_test_3a_info->ae_info.ae_region.border_ratio_y = 100;
	t_self_test_3a_info->ae_info.ae_region.blk_num_x = 16;
	t_self_test_3a_info->ae_info.ae_region.blk_num_y = 16;
	t_self_test_3a_info->ae_info.ae_region.offset_ratio_x = 0;
	t_self_test_3a_info->ae_info.ae_region.offset_ratio_y = 0;
	/*  SubSample*/
	t_self_test_3a_info->subsample_info.token_id = 0x100;
	t_self_test_3a_info->subsample_info.buffer_image_size = 320 * 240;
	t_self_test_3a_info->subsample_info.offset_ratio_x = 0;
	t_self_test_3a_info->subsample_info.offset_ratio_y = 0;
	/* Antiflicker */
	t_self_test_3a_info->antiflicker_info.token_id = 0x130;
	t_self_test_3a_info->antiflicker_info.offset_ratio_x = 0;
	t_self_test_3a_info->antiflicker_info.offset_ratio_y = 0;
	/*  Yhis configuration:*/
	t_self_test_3a_info->yhis_info.token_id = 0x120;
	t_self_test_3a_info->yhis_info.yhis_region.border_ratio_x = 100;
	t_self_test_3a_info->yhis_info.yhis_region.border_ratio_y = 100;
	t_self_test_3a_info->yhis_info.yhis_region.blk_num_x = 16;
	t_self_test_3a_info->yhis_info.yhis_region.blk_num_y = 16;
	t_self_test_3a_info->yhis_info.yhis_region.offset_ratio_x = 0;
	t_self_test_3a_info->yhis_info.yhis_region.offset_ratio_y = 0;

	err = ispdrv_set_3a_cfg(sensor_id, t_self_test_3a_info);

	return err;
}

static u32 self_test_set_dld_sequence_settings(u8 sensor_id)
{
	u32 err = 0;
	struct dld_sequence t_self_test_dld_seq;

	memset((u8 *) &t_self_test_dld_seq, 0, sizeof(struct dld_sequence));
	t_self_test_dld_seq.preview_baisc_dld_seq_length = 4;
	t_self_test_dld_seq.preview_baisc_dld_seq[0] = 1;
	t_self_test_dld_seq.preview_baisc_dld_seq[1] = 1;
	t_self_test_dld_seq.preview_baisc_dld_seq[2] = 1;
	t_self_test_dld_seq.preview_baisc_dld_seq[3] = 1;
	t_self_test_dld_seq.preview_adv_dld_seq_length = 3;
	t_self_test_dld_seq.preview_adv_dld_seq[0] = 1;
	t_self_test_dld_seq.preview_adv_dld_seq[1] = 1;
	t_self_test_dld_seq.preview_adv_dld_seq[2] = 1;
	t_self_test_dld_seq.fast_converge_baisc_dld_seq_length = 2;
	t_self_test_dld_seq.fast_converge_baisc_dld_seq[0] = 3;
	t_self_test_dld_seq.fast_converge_baisc_dld_seq[1] = 3;

	err = ispdrv_set_dld_sequence(sensor_id, &t_self_test_dld_seq);

	return err;

}

static u32 self_test_set_3a_cfg(u8 sensor_id, u8 scenario_id, u16 sensor_height)
{
	u32 err = 0;
	struct awb_gain_info balanced_awb_gain_info;

	isp_info_lo_start_tag();

	/* Set ISO Speed*/
	err = ispdrv_set_iso_speed(sensor_id, 100);
	if (err != 0) {
		isp_info_desc_tag("BIST:SetISOSpeed Error\n");
		goto test_set_3a_cfg_end;
	}

	/* AWB Gain*/
	err = self_test_set_awb_gain_settings(sensor_id);
	if (err != 0)
		goto test_set_3a_cfg_end;

	if (scenario_id == V4L2_ISP_SCENARIO_PREVIEW_STILL_SS) {
		balanced_awb_gain_info.detected_awb_r = 256;
		balanced_awb_gain_info.detected_awb_gr = 256;
		balanced_awb_gain_info.detected_awb_gb = 256;
		balanced_awb_gain_info.detected_awb_b = 256;
		ispdrv_set_balanced_awb_gain(
			sensor_id, &balanced_awb_gain_info);
		ispdrv_set_shading_correct_mode(sensor_id, true);
	}
	/* Bypass 3A setting for rear2 in dual rear sensor mode*/
	if (scenario_id == V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT &&
		g_bist_2ndsensor_id == sensor_id)
		goto test_set_3a_cfg_end;

	/* Fill the settings for 3A Cfg*/
	err = self_test_set_3a_settings(sensor_id, sensor_height);
	if (err != 0)
		goto test_set_3a_cfg_end;
	err = self_test_set_dld_sequence_settings(sensor_id);
	if (err != 0)
		goto test_set_3a_cfg_end;

test_set_3a_cfg_end:
	isp_info_lo_end_tag();
	return err;

}

static void self_test_activate_test_pattern(u8 sensor_id, u16 width,
		u16 height)
{
	switch (sensor_id) {
	case SENSOR1:
		/*  Switching pattern;*/
		set_ahb_indirect(0xfffa4004, 0);

		/*  Image width;*/
		set_ahb_indirect(0xfffa4008, width);

		/*  Image height*/
		set_ahb_indirect(0xfffa400c, height);

		/*  H blanking*/
		set_ahb_indirect(0xfffa4010, 13440);

		/*  V blanking*/
		set_ahb_indirect(0xfffa4014, 100);

		set_ahb_indirect(0xfffa4018, 0x10);

		/*  Release soft reset*/
		set_ahb_indirect(0xfffa4000, 0x2);
		break;

	case SENSOR2:
		/*  Switching pattern;*/
		set_ahb_indirect(0xfffa5004, 0);

		/*  Image width;*/
		set_ahb_indirect(0xfffa5008, width);

		/*  Image height*/
		set_ahb_indirect(0xfffa500c, height);

		/*  H blanking*/
		set_ahb_indirect(0xfffa5010, 13440);

		/*  V blanking*/
		set_ahb_indirect(0xfffa5014, 100);

		set_ahb_indirect(0xfffa5018, 0x10);

		/*  Release soft reset*/
		set_ahb_indirect(0xfffa5000, 0x2);

		break;

	case SENSOR3:
		/*  Switching pattern;*/
		set_ahb_indirect(0xfffa3004, 0);

		/*  Image width;*/
		set_ahb_indirect(0xfffa3008, width);

		/*  Image height*/
		set_ahb_indirect(0xfffa300c, height);

		/*  H blanking*/
		set_ahb_indirect(0xfffa3010, 13440);

		/*  V blanking*/
		set_ahb_indirect(0xfffa3014, 100);

		set_ahb_indirect(0xfffa3018, 0x10);

		/*  Release soft reset*/
		set_ahb_indirect(0xfffa3000, 0x2);
		break;
	default:
		break;
	}
}

/*********************** New BIST Start ************************************/

/*********************** New BIST Public Function ***************************/
void bist_irq_handle(u32 irq)
{
	u32 irq_flags = irq;

	if (BIST_IRQ_MASK & irq_flags) {
		/* handle raw image done */
		if (irq_flags & V4L2_ISP_SENSOR1_RAW_DONE_INT)
			self_test_raw_done(SENSOR1, V4L2_ISP_RAW);

		/* handle still image done */
		if (irq_flags & (V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT |
			V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT)) {
			isp_alert("BIST: handle still done");
			self_test_still_done(irq_flags);
		}
		/* handle video image done */
		if (irq_flags & (V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT |
			V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT))
			self_test_video_done(irq_flags);
		/* handle lv image done */
		if (irq_flags & (V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT |
			V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT |
			V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT))
			self_test_lv_done(irq_flags);

	}
}


void ispdrv_set_bist_addr(u32 bist_buffer_iva, u64 bist_buffer_kva)
{
	g_bist_buff_iva = bist_buffer_iva;
	g_bist_buff_kva = bist_buffer_kva;

	isp_info_lo_start_tag();

	isp_info_item_tag("iva addr: 0x%08x\n", g_bist_buff_iva);
	isp_info_item_tag("kva addr:0x%llx\n", g_bist_buff_kva);

	isp_info_lo_end_tag();
}
EXPORT_SYMBOL(ispdrv_set_bist_addr);

u32 ispdrv_bist(void)
{
	u32 err = 0;
	u8 index_test_case, index_order;
	u8 bist_test_case_number = 0;
	struct s_scenario_info_ap scenario_info;
	bool bist_bypass_flag[MAX_USED_SENSOR_NUMBER][
		V4L2_ISP_OUTPUT_IMAGE_TOTAL];
	/* start address of bist buffer*/
	u32 memory_start_address = g_bist_buff_iva;
	u32 memory_offset = 0;

	isp_info_pu_start_tag();

	/* Prepare 1. Get test case number */
	bist_test_case_number = g_test_case_number;
	/* Prepare 2. Assign the default data into global variables
	* TODO: fill the structures with default data
	*/
	/* Prepare 3. Change IRP handling Callback and Qmerge */
	bist_set_preparation(&memory_start_address, &memory_offset);
	g_image_memory_start_addr = memory_start_address;
	g_image_memory_offset = memory_offset;
	/* Prepare 4. Get share buffer */
	g_shared_scenario_info =
		(struct s_scenario_info_isp *)
		ispdrv_get_shared_scenario_info_ptr();

	for (index_test_case = 0; index_test_case < bist_test_case_number;
	index_test_case++) {
		g_current_test_case = index_test_case;
		if (index_test_case > 0) {
			memory_start_address = g_image_memory_start_addr;
			memory_offset = g_image_memory_offset;
		}
		if (g_hq_buffer_size[index_test_case] != 0) {
			ispdrv_set_hq_buffer_mem_info(memory_start_address +
				memory_offset,
				(u64)g_hq_buffer_size[index_test_case]);
			memory_start_address = memory_start_address +
				memory_offset;
			memory_offset = g_hq_buffer_size[index_test_case];
		}
		if (g_raw_buffer_size[index_test_case] != 0) {
			ispdrv_set_raw_buffer_mem_info(memory_start_address +
				memory_offset,
				g_raw_buffer_size[index_test_case]);
			memory_start_address = memory_start_address +
				memory_offset;
			memory_offset = g_raw_buffer_size[index_test_case];
		}

		/* Initial all variable */
		memset(g_image_addr, 0, sizeof(g_image_addr));
		memset(g_golden_addr, 0, sizeof(g_golden_addr));
		memset((u8 *) g_raw_iq_info_1, 0,
		    sizeof(struct s_iq_info_1) * 2);
		memset((u8 *) g_raw_iq_info_2, 0,
		    sizeof(struct s_iq_info_2) * 2);
		memset((u8 *) g_proc_still_opt, 0,
		    sizeof(struct ispdrv_proc_still_opt) * 2);
		memset((u8 *)g_bist_proc_still_start, 0,
		    sizeof(bool) * 2);
		memset((u8 *)g_bist_proc_still_end, 0,
		    sizeof(bool) * 2);
		memset((u8 *) &scenario_info, 0,
			sizeof(struct s_scenario_info_ap));
		memset((u8 *) g_is_compare, 0,
			sizeof(bool) * MAX_USED_SENSOR_NUMBER * 3);
		g_still_index = 0;
		g_raw_index = 0;

		memset((u8 *) &(g_bist_info[index_test_case].frame_count), 0,
			sizeof(u16) * MAX_USED_SENSOR_NUMBER *
			S_OUTPUT_IMAGE_TOTAL);

		for (index_order = 0; index_order < MAX_USED_SENSOR_NUMBER;
		index_order++) {

			/* Step 1. Prepare the usable information */
			/* Get sensor id by scenrario id */
			g_bist_info[index_test_case].sensor_id[index_order] =
				self_test_get_sensor_id(g_bist_info,
				g_current_test_case, index_order);
			/* Get the bypass flag from scneario_info */
			self_test_set_bypass_flag(
				g_bist_info[index_test_case].
				scenario_id[index_order],
				bist_bypass_flag[index_order],
				&(g_scenario_info[index_test_case][
				index_order].out_bypass_flag));

			/* Step 2. Allocate memory for output image*/
			bist_set_image_address(
				g_bist_info[index_test_case].
				scenario_id[index_order],
				g_image_addr[index_order],
				g_reserve_raw_addr,
				bist_bypass_flag[index_order],
				&(g_output_buff_format[index_test_case][
				index_order]),
				&memory_start_address,
				&memory_offset);
			/* Allocate memory for golden image */
			bist_set_golden_address(g_golden_addr[index_order],
				bist_bypass_flag[index_order],
				&(g_output_buff_format[index_test_case][
				index_order]),
				&memory_start_address,
				&memory_offset);

			/* Step 3. Set Scenario info */
			MEMCPY((u8 *) &scenario_info,
				(u8 *) &(g_scenario_info[index_test_case][
				index_order]),
				sizeof(struct s_scenario_info_ap));

			err = bist_set_scenario_info(
				g_bist_info[index_test_case].
				sensor_id[index_order],
				g_bist_info[index_test_case].
				scenario_id[index_order],
				&scenario_info);
			if (err != 0)
				goto bist_end;
			/* Step 4. Set 3a */
			err = self_test_set_3a_cfg(
				g_bist_info[index_test_case].
				sensor_id[index_order],
				g_bist_info[index_test_case].
				scenario_id[index_order],
				g_scenario_info[index_test_case][index_order].
				sensor_info.original_height);
			if (err != 0)
				goto bist_end;

			/* Step 5. Set output buffer format */
			err = bist_set_output_buff_format(
				g_bist_info[index_test_case].
				sensor_id[index_order],
				g_image_addr[index_order],
				&(g_output_buff_format[index_test_case][
				index_order]),
				bist_bypass_flag[index_order]);
			if (err != 0)
				goto bist_end;
			if (index_order == 0)
				MEMCPY((u8 *) &g_shared_scenario_info->
					self_test_info,
					(u8 *) &(
					g_self_test_info[index_test_case]),
					sizeof(
					struct s_scinfo_self_test_info));

			/* Wait 120ms for HW update*/
			mdelay(120);

			/* Step 6. Set preview mode */
			err = ispdrv_set_preview_mode(g_bist_info[
				index_test_case].scenario_id[index_order]);
			if (err != 0)
				goto bist_end;

			/* Step 7. Stream on */
			err = bist_preview_stream_on(
				g_bist_info[index_test_case].
				sensor_id[index_order],
				g_bist_info[g_current_test_case].
				scenario_id[index_order],
				bist_bypass_flag[index_order],
				g_raw_frame_rate[index_test_case]);
			if (err != 0)
				goto bist_end;

			/* Step 8. Activate test pattern */
			self_test_activate_test_pattern(
				g_bist_info[index_test_case].
				sensor_id[index_order],
				g_scenario_info[index_test_case][index_order].
				sensor_info.original_width,
				g_scenario_info[index_test_case][index_order].
				sensor_info.original_height);
			/* Break if it is not independent mode */
			if (g_self_test_info[index_test_case].is_independent ==
				false)
				break;
		}
		/* Step 10. Dynamic multi-layer mode */
		if (g_bist_info[index_test_case].scenario_id[0] ==
			V4L2_ISP_SCENARIO_PREVIEW_STILL_SS) {
			err = self_test_start_proc_still();
			if (err != 0)
				goto bist_end;

		}

		/* Step 10. Wait frame done */
		if (g_self_test_info[index_test_case].is_independent == false)
			while (false == meet_max_frame_count(
				g_bist_info[index_test_case].sensor_id[0],
				bist_bypass_flag[0])) {
				mdelay(1000);
			}
		else
			while ((meet_max_frame_count(
				g_bist_info[index_test_case].sensor_id[0],
				bist_bypass_flag[0]) == false) &&
				(meet_max_frame_count(
				g_bist_info[index_test_case].sensor_id[1],
				bist_bypass_flag[1]) == false)) {
				mdelay(1000);
			}

		/* Wait 42ms for HW idle*/
		mdelay(42);

		for (index_order = 0; index_order < MAX_USED_SENSOR_NUMBER;
		index_order++) {
			/* Step 11. Stream off */
			err = bist_preview_stream_off(
				g_bist_info[index_test_case].
				sensor_id[index_order],
				bist_bypass_flag[index_order],
				g_raw_frame_rate[index_test_case]);
			/* Wait 60ms for HW idle*/
			mdelay(60);

			/* Step 12. Stop test pattern */
			bist_inactivate_test_pattern(g_bist_info[
				index_test_case].sensor_id[index_order]);
			/* Step 13. Stop preview */
			err = ispdrv_stop_preview_mode(g_bist_info[
				index_test_case].sensor_id[index_order]);

			/* Break if it is not independent mode */
			if (g_self_test_info[index_test_case].is_independent ==
				false)
				break;
		}

		/* Step 14. Load Golden image */
		for (index_order = 0; index_order < MAX_USED_SENSOR_NUMBER;
		index_order++) {
			self_test_load_golden_image(
				g_golden_addr[index_order],
				&(g_output_buff_format[index_test_case][
				index_order]),
				bist_bypass_flag[index_order],
				g_golden_name[index_test_case][
				index_order],
				g_is_compare[index_order]);
			/* Break if it is not independent mode */
			if (g_self_test_info[index_test_case].is_independent ==
				false)
				break;
		}
		/* Step 15. Calculate CRC and compare */
		bist_check_sum(&g_golden_sum, &g_golden_xor,
			&g_output_sum, &g_output_xor);
	}

bist_end:
	isp_info_desc_tag("BIST: Debug mode:%d for stopping cmd",
		g_debug_mode);
	while (g_debug_mode)
		mdelay(1000);

	/* Clear all global variables */
	self_test_clear_global_variables();
	/* TODO: Clear all variables in [altek_cmd_bist.c] */
	isp_info_pu_end_tag();
	return err;
}
EXPORT_SYMBOL(ispdrv_bist);

bool ispdrv_is_bist_mode(void)
{
	bool is_bist_mode = false;

	if (g_self_test_info[0].self_test_mode == E_SELFTEST_BIST)
		is_bist_mode = true;

	return is_bist_mode;
}
EXPORT_SYMBOL(ispdrv_is_bist_mode);

void bist_set_image_address(u8 scenario_id,
	struct output_addr_setting *output_addr,
	u32 *reserve_raw_addr,
	bool *bypass_flag,
	struct s_output_buff_format_info *output_buff_format,
	u32 *memory_start_addr,
	u32 *memory_offset)
{
	u32 start_addr = *memory_start_addr;
	u32 offset = *memory_offset;
	u8 index;

	/* For LV */
	for (index = 0; index < V4L2_ISP_OUTPUT_IMAGE_TOTAL; index++) {
		if (index == V4L2_ISP_RAW) {
			if (scenario_id ==
				V4L2_ISP_SCENARIO_PREVIEW_STILL_SS) {
				self_test_allocate_image_addr(scenario_id,
					index,
					&(output_addr[index]),
					output_buff_format,
					&start_addr,
					&offset);
				reserve_raw_addr[0] = start_addr + offset;
				start_addr = reserve_raw_addr[0];
				reserve_raw_addr[1] = start_addr + offset;
				start_addr = reserve_raw_addr[1];
			}
		} else {
			if (bypass_flag[index] == NOT_BYPASS)
				self_test_allocate_image_addr(scenario_id,
					index,
					&(output_addr[index]),
					output_buff_format,
					&start_addr,
					&offset);
		}
	}
	*memory_start_addr = start_addr;
	*memory_offset = offset;
}

void bist_set_golden_address(u32 *golden_addr,
	bool *bypass_flag,
	struct s_output_buff_format_info *output_buff_format,
	u32 *memory_start_addr,
	u32 *memory_offset)
{
	u32 start_addr = *memory_start_addr;
	u32 offset = *memory_offset;
	u8 index;

	for (index = 0; index < 3; index++) {
		if (bypass_flag[index] == NOT_BYPASS) {
			golden_addr[index] = start_addr + offset;
			offset = output_buff_format->width[index] *
				output_buff_format->height[index] * 2;
			start_addr = golden_addr[index];
		}
	}
	*memory_start_addr = start_addr;
	*memory_offset = offset;

}

u32 bist_set_scenario_info(u8 sensor_id,
	u8 scenario_id,
	struct s_scenario_info_ap *scenario_info)
{
	u32 err = 0;

	isp_info_lo_start_tag();
	isp_info_desc_tag("Sensor %d, scenario %d",
		sensor_id, scenario_id);

	if (sensor_id == SENSOR1)
		ispdrv_set_scenario_info(
			scenario_id,
			scenario_info,
			NULL, NULL);
	else if (sensor_id == SENSOR2)
		ispdrv_set_scenario_info(
			scenario_id,
			NULL,
			scenario_info, NULL);
	else if (sensor_id == SENSOR3)
		ispdrv_set_scenario_info(
			scenario_id,
			NULL, NULL,
			scenario_info);
	if (err)
		isp_info_err("%d", err);

	isp_info_lo_end_tag();

	return err;
}

u32 bist_set_output_buff_format(u8 sensor_id,
	struct output_addr_setting *image_addr,
	struct s_output_buff_format_info *output_buff_format,
	bool *bypass_flag)
{
	u32 err = 0;
	u8 index;

	isp_info_lo_start_tag();

	for (index = 0; index < V4L2_ISP_OUTPUT_IMAGE_TOTAL; index++) {
		if (bypass_flag[index] == NOT_BYPASS) {
			err = ispdrv_set_output_buff_format(sensor_id, index,
				output_buff_format->enable[index],
				output_buff_format->format[index],
				output_buff_format->width[index],
				output_buff_format->height[index],
				output_buff_format->buffer_number[index],
				&(image_addr[index]),
				output_buff_format->line_offset[index]);
			if (err)
				isp_info_err("%d", err);

		}
	}

	isp_info_lo_end_tag();

	return err;
}

u32 bist_preview_stream_on(u8 sensor_id,
	u8 scenario_id,
	bool *bypass_flag,
	enum ispdrv_raw_frame_rate *raw_frame_rate)
{
	u32 err = 0;
	u8 index;

	isp_info_lo_start_tag();

	for (index = 0; index < V4L2_ISP_OUTPUT_IMAGE_TOTAL; index++) {
		/* Dynamic multi-layer, do not stream on still image flow */
		if (sensor_id == SENSOR1 &&
			scenario_id == V4L2_ISP_SCENARIO_PREVIEW_STILL_SS) {
			if (index == V4L2_ISP_RAW)
				err = ispdrv_set_raw_frame_rate(sensor_id,
					raw_frame_rate[0]);
			else if (index != V4L2_ISP_IMG3 &&
				bypass_flag[index] == NOT_BYPASS)
				err = ispdrv_preview_stream_on(sensor_id,
					index);
			if (err)
				isp_info_err("%d", err);
		} else {
			if (bypass_flag[index] == NOT_BYPASS) {
				/* Statistic of AF stream is configured
				* with V4L2_ISP_STATITISTICS
				*/
				if (index == V4L2_ISP_STATITISTICS_OF_AF)
					continue;
				else {
					err = ispdrv_preview_stream_on(
						sensor_id, index);
					if (err)
						isp_info_err("%d", err);
				}
			}
		}
	}

	isp_info_lo_end_tag();

	return err;
}


u32 bist_preview_stream_off(u8 sensor_id,
	bool *bypass_flag,
	enum ispdrv_raw_frame_rate *raw_frame_rate)
{
	u32 err = 0;
	u8 index;

	isp_info_lo_start_tag();

	for (index = 0; index < V4L2_ISP_OUTPUT_IMAGE_TOTAL; index++) {
		/* Dynamic multi-layer */
		if (sensor_id == SENSOR1 &&
			g_bist_info[g_current_test_case].
			scenario_id[0] ==
			V4L2_ISP_SCENARIO_PREVIEW_STILL_SS) {
			if (index != V4L2_ISP_IMG3 &&
				index != V4L2_ISP_RAW &&
				bypass_flag[index] == NOT_BYPASS)
				err = ispdrv_preview_stream_off(sensor_id,
					index);
			if (err)
				isp_info_err("%d", err);
		} else {
			if (bypass_flag[index] == NOT_BYPASS) {
				/* Statistic of AF stream is configured
				* with V4L2_ISP_STATITISTICS
				*/
				if (index == V4L2_ISP_STATITISTICS_OF_AF)
					continue;
				else {
					err = ispdrv_preview_stream_off(
						sensor_id, index);
					if (err)
						isp_info_err("%d", err);
				}
			}
		}
	}

	isp_info_lo_end_tag();

	return err;

}

void bist_inactivate_test_pattern(u8 sensor_id)
{
	/* Step 11. Release soft reset*/
	if (sensor_id == SENSOR1) {
		set_ahb_indirect(0xfffa4000, 0x1);
		set_ahb_indirect(0xfffa4000, 0x0);
	} else if (sensor_id == SENSOR2) {
		set_ahb_indirect(0xfffa5000, 0x1);
		set_ahb_indirect(0xfffa5000, 0x0);
	} else {
		set_ahb_indirect(0xfffa3000, 0x1);
		set_ahb_indirect(0xfffa3000, 0x0);
	}
}

void bist_check_sum(u32 *golden_sum, u32 *golden_xor,
	u32 *output_sum, u32 *output_xor)
{
	u8 order, image_type;
	u32 image_size;

	isp_info_lo_start_tag();

	for (order = 0; order < MAX_USED_SENSOR_NUMBER; order++) {
		for (image_type = 0; image_type < 3; image_type++) {
			if (g_is_compare[order][image_type] == false)
				continue;
			isp_info_desc_tag("Compare for image %d", image_type);

			if (g_bist_info[g_current_test_case].
				scenario_id[order] ==
				V4L2_ISP_SCENARIO_PREVIEW_STILL_SS &&
				image_type == V4L2_ISP_IMG3 &&
				g_ss_opt_writing_mode[g_current_test_case] ==
				ISP_TILE_WRITING_MODE)
				/* Not Reformat */
				image_size = (u32)(g_output_buff_format[
					g_current_test_case][order].
					width[image_type] *
					g_output_buff_format[
					g_current_test_case][order].
					height[image_type] * 3 / 2);
			else
				image_size = g_output_buff_format[
					g_current_test_case][order].
					width[image_type] *
					g_output_buff_format[
					g_current_test_case][order].
					height[image_type] * 2;
			isp_info_desc_tag(
				"Size: 0x%x", image_size);

			isp_info_desc_tag(
				"Golden: iva 0x%x, kva 0x%llx",
				g_golden_addr[order][image_type],
				self_test_get_kva(
				g_golden_addr[order][image_type]));

			/* Get the CRC of golden image */
			self_test_check_sum(
				self_test_get_kva(
				g_golden_addr[order][image_type]),
				image_size, golden_sum, golden_xor);

			isp_info_desc_tag("Golden: sum 0x%x, xor 0x%x",
				*golden_sum, *golden_xor);
			isp_info_desc_tag(
				"output image: iva 0x%x, kva 0x%llx",
				g_image_addr[order][image_type].ud_addr1,
				self_test_get_kva(
				g_image_addr[order][image_type].ud_addr1));

			/* Get the CRC of output image */
			/* Dynamic multi-layer, still image, width > 4900,
			* tile writing mode
			*/
			if (g_bist_info[g_current_test_case].
				scenario_id[order] ==
				V4L2_ISP_SCENARIO_PREVIEW_STILL_SS &&
				image_type == V4L2_ISP_IMG3 &&
				g_output_buff_format[g_current_test_case][
				order].width[image_type] > 4900 &&
				g_ss_opt_writing_mode[g_current_test_case] ==
				ISP_TILE_WRITING_MODE) {
				self_test_check_sum_ss(
					self_test_get_kva(
					g_image_addr[order][image_type].
					ud_addr1),
					image_size,
					g_output_buff_format[
					g_current_test_case][order].
					width[image_type],
					g_output_buff_format[
					g_current_test_case][order].
					height[image_type],
					output_sum, output_xor);

				isp_info_desc_tag(
					"Output image 1: sum 0x%x, xor 0x%x",
					*output_sum,
					*output_xor);

				if ((*golden_sum == *output_sum) &&
					(*golden_xor == *output_xor))
					isp_info_desc_tag(
						"Output image 1: Match");
				else {
					isp_info_desc_tag(
						"Output image 1: Not Match");
				}

				isp_info_desc_tag(
					"output image: iva 0x%x, kva 0x%llx",
					g_image_addr[order][image_type].
					ud_addr2,
					self_test_get_kva(
					g_image_addr[order][image_type].
					ud_addr2));

				self_test_check_sum_ss(
					self_test_get_kva(
					g_image_addr[order][image_type].
					ud_addr2),
					image_size,
					g_output_buff_format[
					g_current_test_case][order].
					width[image_type],
					g_output_buff_format[
					g_current_test_case][order].
					height[image_type],
					output_sum, output_xor);

				isp_info_desc_tag(
					"Output image 2: sum 0x%x, xor 0x%x",
					*output_sum,
					*output_xor);

				if ((*golden_sum == *output_sum) &&
					(*golden_xor == *output_xor))
					isp_info_desc_tag(
						"Output image 2: Match");
				else {
					isp_info_desc_tag(
						"Output image 2: Not Match");
				}

			} else {
				self_test_check_sum(
					self_test_get_kva(
					g_image_addr[order][image_type].
					ud_addr1),
					image_size, output_sum, output_xor);

				isp_info_desc_tag(
					"Output image: sum 0x%x, xor 0x%x",
					*output_sum,
					*output_xor);

				if ((*golden_sum == *output_sum) &&
					(*golden_xor == *output_xor))
					isp_info_desc_tag(
					"Output image: Match");
				else {
					isp_info_desc_tag(
						"Output image: Not Match");
				}
			}
		}

		/* Break if it is not independent mode */
		if (g_self_test_info[g_current_test_case].is_independent ==
			false)
			break;
	}

	isp_info_lo_end_tag();
}

void bist_set_test_case_number_setting(u8 test_case_number)
{
	isp_info_lo_start_tag();
	if (test_case_number > MAX_TEST_CASE_NUMBER || test_case_number < 0)
		g_test_case_number = 0;
	else
		g_test_case_number = test_case_number;
	isp_info_item_tag("BIST test case number: %d,", g_test_case_number);

	isp_info_lo_end_tag();
}

void bist_set_scenario_info_setting(
	struct s_scenario_info_ap scenario_info[][MAX_USED_SENSOR_NUMBER])
{
	isp_info_lo_start_tag();

	MEMCPY((u8 *)g_scenario_info, (u8 *)scenario_info,
		sizeof(struct s_scenario_info_ap) *
			MAX_TEST_CASE_NUMBER *
			MAX_USED_SENSOR_NUMBER);

	isp_info_lo_end_tag();

}

void bist_set_output_buff_format_setting(
	struct s_output_buff_format_info
	output_buff_format[][MAX_USED_SENSOR_NUMBER])
{
	isp_info_lo_start_tag();

	MEMCPY((u8 *)g_output_buff_format, (u8 *)output_buff_format,
		sizeof(struct s_output_buff_format_info) *
			g_test_case_number *
			MAX_USED_SENSOR_NUMBER);

	isp_info_lo_end_tag();
}

void bist_set_self_test_info_setting(
	struct s_scinfo_self_test_info *self_test_info)
{
	isp_info_lo_start_tag();

	MEMCPY((u8 *) g_self_test_info, (u8 *) self_test_info,
		sizeof(struct s_scinfo_self_test_info) *
			MAX_TEST_CASE_NUMBER);

	isp_info_lo_end_tag();
}

void bist_set_bist_info_setting(
	struct s_bist_info *bist_info)
{
	isp_info_lo_start_tag();

	isp_info_item_tag("BIST INPUT");

	isp_info_item_tag("BIST scenario ID: %d,",
		bist_info[0].sensor_id[0]);
	MEMCPY((u8 *) g_bist_info, (u8 *) bist_info,
		sizeof(struct s_bist_info) * MAX_TEST_CASE_NUMBER);

	isp_info_item_tag("BIST ASSIGN");

	isp_info_item_tag("BIST scenario ID: %d,",
		g_bist_info[0].scenario_id[0]);

	isp_info_lo_end_tag();
}

void bist_set_raw_frame_rate_setting(
	enum ispdrv_raw_frame_rate raw_frame_rate[][MAX_RAW_FR_RATE_CHANGE])
{
	isp_info_lo_start_tag();

	MEMCPY((u8 *) g_raw_frame_rate, (u8 *)raw_frame_rate,
		sizeof(enum ispdrv_raw_frame_rate) * MAX_TEST_CASE_NUMBER * 2);

	isp_info_lo_end_tag();
}

void bist_set_isp_speed_setting(u32 *iso_speed)
{
	isp_info_lo_start_tag();

	MEMCPY((u8 *) g_iso_speed, (u8 *)iso_speed,
		sizeof(u32) * MAX_TEST_CASE_NUMBER);

	isp_info_lo_end_tag();
}

void bist_set_golden_name_setting(
	char *(*golden_name)[MAX_USED_SENSOR_NUMBER][3])
{
	int test = 0, order = 0, type = 0;

	isp_info_lo_start_tag();

	for (test = 0; test < MAX_TEST_CASE_NUMBER; test++) {
		for (order = 0; order < MAX_USED_SENSOR_NUMBER; order++) {
			for (type = 0; type < MAX_GOLDEN_IMG_NUMBER; type++) {
				g_golden_name[test][order][type] =
					kmalloc(
						MAX_FILE_PATH_SIZE, GFP_KERNEL);
				if (NULL ==
					g_golden_name[test][order][type]) {
					isp_err_lo_combo_desc_tag(
						"Err: Unable to alloc. mem.");
					goto bist_set_golden_name_setting_end;
				}

				memset(g_golden_name[test][order][type], 0,
					MAX_FILE_PATH_SIZE);
			}
		}
	}

	MEMCPY((u8 *)g_golden_name, (u8 *) golden_name,
		sizeof(char *) * MAX_TEST_CASE_NUMBER *
		MAX_USED_SENSOR_NUMBER * MAX_GOLDEN_IMG_NUMBER);

bist_set_golden_name_setting_end:

	isp_info_lo_end_tag();
}


void bist_set_debug_mode_setting(bool debug_mode)
{
	g_debug_mode = debug_mode;
}

void bist_set_ss_opt_writing_mode_setting(
	enum ispdrv_sl_output_writing_mode *writing_mode)
{
	isp_info_lo_start_tag();

	MEMCPY((u8 *)g_ss_opt_writing_mode, (u8 *)writing_mode,
		sizeof(enum ispdrv_sl_output_writing_mode) *
		MAX_TEST_CASE_NUMBER);

	isp_info_lo_end_tag();
}

void bist_set_hq_buffer_size_setting(u32 *hq_buff_size)
{
	isp_info_lo_start_tag();

	MEMCPY((u8 *)g_hq_buffer_size, (u8 *)hq_buff_size,
		sizeof(u32) * MAX_TEST_CASE_NUMBER);
	isp_alert("BIST: hq buff size 0x%x", g_hq_buffer_size[0]);
	isp_info_lo_end_tag();
}

void bist_set_raw_buffer_size_setting(u32 *raw_buff_size)
{
	isp_info_lo_start_tag();

	MEMCPY((u8 *)g_raw_buffer_size, (u8 *)raw_buff_size,
		sizeof(u32) * MAX_TEST_CASE_NUMBER);

	isp_info_lo_end_tag();
}

u32 bist_set_preparation(u32 *start_addr, u32 *offset)
{
	u32 err = 0;
	u32 memory_start_address = *start_addr;
	u32 memory_offset = *offset;

	isp_info_lo_start_tag();

	/*  Set rear handle */
	isp_register_irq_CB_func(bist_irq_handle, 1);
	/* Set front handle */
	isp_register_irq_CB_func(bist_irq_handle, 2);
	/* Set qmerge address */
	ispdrv_set_qmerge_addr(memory_start_address+memory_offset, 0, 0);
	memory_start_address = memory_start_address + memory_offset;
	memory_offset = MAX_QMERGE_BIN_SIZE;
	ispdrv_set_qmerge_addr(0, memory_start_address+memory_offset, 0);
	memory_start_address = memory_start_address + memory_offset;
	memory_offset = MAX_QMERGE_BIN_SIZE;
	ispdrv_set_qmerge_addr(0, 0, memory_start_address+memory_offset);
	memory_start_address = memory_start_address + memory_offset;
	memory_offset = MAX_QMERGE_BIN_SIZE;
	/* Load qmerge into memory */
	err = self_test_load_qmerge();

	*start_addr = memory_start_address;
	*offset = memory_offset;

	isp_info_lo_end_tag();

	return err;

}

void bist_set_firmware_addr(u64 firmware_addr)
{
	g_firmware_addr = firmware_addr;
	if (g_self_test_info[0].self_test_mode == E_SELFTEST_BIST)
		bist_load_firmware();
}
/*********************** New BIST Public Function ***************************/

/*********************** New BIST Private Function **************************/

static void bist_load_firmware(void)
{
	static bool is_loaded;
	const struct firmware *fw;

	isp_info_lo_start_tag();

	if (is_loaded == false) {
		/* TO DO ---- Upload firmware.*/
		if (request_firmware
		    (&fw, "TBM_G2v1DDR.bin",
		    (struct device *)bist_get_v4l2_device())) {
			isp_err_pu_combo_desc_tag("get firmware failed.");
		} else {
			isp_info_item_tag(
				"fw: 0x%llx, fw->data: 0x%p, fw->size: 0x%lx",
				g_firmware_addr, fw->data, fw->size);

			MEMCPY((u8 *) g_firmware_addr,
				(void *)fw->data, fw->size);
			dma_flush_range((void *)g_firmware_addr,
					  (void *)(g_firmware_addr +
					  fw->size));

			isp_info_desc_tag("firmware load ok.\n");
			release_firmware(fw);
		}
		is_loaded = true;
	}

	isp_info_lo_end_tag();
}

static bool meet_max_frame_count(u8 sensor_id,
	bool *bypass_flag)
{
	bool condition_met = false;
	bool image_frame_count_met[V4L2_ISP_OUTPUT_IMAGE_TOTAL] = {
		true, true, true, true, true, true};
	u8 image_type;

	isp_info_lo_start_tag();

	if (g_bist_info[g_current_test_case].sensor_id[0] == sensor_id) {
		for (image_type = 0; image_type < V4L2_ISP_OUTPUT_IMAGE_TOTAL;
	image_type++) {
			if (g_bist_info[g_current_test_case].
				scenario_id[0] ==
				V4L2_ISP_SCENARIO_PREVIEW_STILL_SS &&
				bypass_flag[image_type] == NOT_BYPASS &&
				image_type == V4L2_ISP_IMG3) {
				if (g_bist_info[g_current_test_case].
					frame_count[0][image_type] < 2)
					image_frame_count_met[image_type] =
					false;
			} else if (bypass_flag[image_type] == NOT_BYPASS &&
				image_type != V4L2_ISP_STATITISTICS &&
				image_type != V4L2_ISP_STATITISTICS_OF_AF)
				if (g_bist_info[g_current_test_case].
					frame_count[0][image_type] <
					g_self_test_info[g_current_test_case].
					max_frame_cnt) {
					image_frame_count_met[image_type] =
					false;
				}
		}
		condition_met = (image_frame_count_met[V4L2_ISP_IMG1] &
			image_frame_count_met[V4L2_ISP_IMG2] &
			image_frame_count_met[V4L2_ISP_IMG3] &
			image_frame_count_met[V4L2_ISP_STATITISTICS] &
			image_frame_count_met[V4L2_ISP_STATITISTICS_OF_AF] &
			image_frame_count_met[V4L2_ISP_RAW]);
		g_bist_info[g_current_test_case].meet_max_frame_count[0] =
			condition_met;
	} else if (g_bist_info[g_current_test_case].
		sensor_id[1] == sensor_id) {
		for (image_type = 0; image_type < V4L2_ISP_OUTPUT_IMAGE_TOTAL;
		image_type++) {
			if (bypass_flag[image_type] == NOT_BYPASS &&
				image_type != V4L2_ISP_STATITISTICS &&
				image_type != V4L2_ISP_STATITISTICS_OF_AF)
				if (g_bist_info[g_current_test_case].
					frame_count[1][image_type] <
					g_self_test_info[g_current_test_case].
					max_frame_cnt)
					image_frame_count_met[image_type] =
						false;
		}
		condition_met = (image_frame_count_met[V4L2_ISP_IMG1] &
			image_frame_count_met[V4L2_ISP_IMG2] &
			image_frame_count_met[V4L2_ISP_IMG3] &
			image_frame_count_met[V4L2_ISP_STATITISTICS] &
			image_frame_count_met[V4L2_ISP_STATITISTICS_OF_AF] &
			image_frame_count_met[V4L2_ISP_RAW]);
		g_bist_info[g_current_test_case].meet_max_frame_count[1] =
			condition_met;

	}
	isp_alert_lo_combo_item_tag("Meet max frame count: %s",
		((condition_met == true)?("True"):("False")));

	isp_info_lo_end_tag();
	return condition_met;
}

static void self_test_lv_done(u32 irq_flag)
{
	isp_info_lo_start_tag();
	if (irq_flag & V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT) {
		if (g_bist_info[g_current_test_case].sensor_id[0] ==
			SENSOR1) {
			g_bist_info[g_current_test_case].
				frame_count[0][V4L2_ISP_IMG1]++;
		} else if (g_bist_info[g_current_test_case].sensor_id[1] ==
			SENSOR1) {
			g_bist_info[g_current_test_case].
				frame_count[1][V4L2_ISP_IMG1]++;
		}
	}

	if (irq_flag & V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT) {
		if (g_bist_info[g_current_test_case].sensor_id[0] ==
			SENSOR2) {
			g_bist_info[g_current_test_case].
				frame_count[0][V4L2_ISP_IMG1]++;
		} else if (g_bist_info[g_current_test_case].sensor_id[1] ==
			SENSOR2) {
			g_bist_info[g_current_test_case].
				frame_count[1][V4L2_ISP_IMG1]++;
		}
	}

	if (irq_flag &  V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT) {
		if (g_bist_info[g_current_test_case].sensor_id[0] ==
			SENSOR3) {
			g_bist_info[g_current_test_case].
				frame_count[0][V4L2_ISP_IMG1]++;
		} else if (g_bist_info[g_current_test_case].sensor_id[1] ==
			SENSOR3) {
			g_bist_info[g_current_test_case].
				frame_count[1][V4L2_ISP_IMG1]++;
		}
	}
	isp_info_lo_end_tag();
}

static void self_test_video_done(u32 irq_flag)
{
	isp_info_lo_start_tag();
	if (irq_flag & V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT) {
		if (g_bist_info[g_current_test_case].sensor_id[0] ==
			SENSOR1) {
			g_bist_info[g_current_test_case].
				frame_count[0][V4L2_ISP_IMG2]++;
		} else if (g_bist_info[g_current_test_case].sensor_id[1] ==
			SENSOR1) {
			g_bist_info[g_current_test_case].
				frame_count[1][V4L2_ISP_IMG2]++;
		}
	}

	if (irq_flag & V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT) {
		if (g_bist_info[g_current_test_case].sensor_id[0] ==
			SENSOR2) {
			g_bist_info[g_current_test_case].
				frame_count[0][V4L2_ISP_IMG2]++;
		} else if (g_bist_info[g_current_test_case].sensor_id[1] ==
			SENSOR2) {
			g_bist_info[g_current_test_case].
				frame_count[1][V4L2_ISP_IMG2]++;
		}
	}
	isp_info_lo_end_tag();
}

static void self_test_still_done(u32 irq_flag)
{
	isp_info_lo_start_tag();
	if (irq_flag & V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT) {
		if (g_bist_info[g_current_test_case].sensor_id[0] ==
			SENSOR1) {
			g_bist_info[g_current_test_case].
				frame_count[0][V4L2_ISP_IMG3]++;
		} else if (g_bist_info[g_current_test_case].sensor_id[1] ==
			SENSOR1) {
			g_bist_info[g_current_test_case].
				frame_count[1][V4L2_ISP_IMG3]++;
		}

		if (g_bist_info[g_current_test_case].scenario_id[0] ==
			V4L2_ISP_SCENARIO_PREVIEW_STILL_SS) {
			g_bist_proc_still_end[g_still_index] = true;
			g_still_index = (g_still_index + 1) % 2;
		}
	}

	if (irq_flag & V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT) {
		if (g_bist_info[g_current_test_case].sensor_id[0] ==
			SENSOR2) {
			g_bist_info[g_current_test_case].
				frame_count[0][V4L2_ISP_IMG3]++;
		} else if (g_bist_info[g_current_test_case].sensor_id[1] ==
			SENSOR2) {
			g_bist_info[g_current_test_case].
				frame_count[1][V4L2_ISP_IMG3]++;
		}
	}

	isp_info_lo_end_tag();
}

static void self_test_raw_done(u8 sensor_id, u8 image_type)
{
	u32 err = 0;
	u32 output_image_addr = 0;
	bool image_ready = false;
	struct s_iq_info_1 *iq_info_1;
	struct s_iq_info_2 *iq_info_2;
	struct s_scenario_size img_size;
	static bool change_raw_frame_rate;

	isp_info_lo_start_tag();

	iq_info_1 = kmalloc(sizeof(struct s_iq_info_1), GFP_KERNEL);
	memset((u8 *) iq_info_1, 0, sizeof(struct s_iq_info_1));
	iq_info_2 = kmalloc(sizeof(struct s_iq_info_2), GFP_KERNEL);
	memset((u8 *) iq_info_2, 0, sizeof(struct s_iq_info_2));

	memset((u8 *) &img_size, 0, sizeof(struct s_scenario_size));

	if (g_bist_info[g_current_test_case].
		frame_count[0][V4L2_ISP_RAW] ==
		g_self_test_info[g_current_test_case].max_frame_cnt)
		goto self_test_raw_done_end;

	/* Count the frame done of raw image */
	g_bist_info[g_current_test_case].
		frame_count[0][V4L2_ISP_RAW]++;

	/* check  there is more image ready to be gotten*/
	err = ispdrv_is_more_img_ready(
		sensor_id, image_type, &image_ready);
	if (!image_ready) {
		isp_info_desc_tag("BIST: No more img ready\n");
		err = EINVAL;
		goto self_test_raw_done_end;
	}
	/* Get the output image and the information */
	err = ispdrv_get_output_buffer(sensor_id, image_type,
				   &output_image_addr, NULL, NULL, NULL,
				   &img_size, iq_info_1,
				   iq_info_2);

	if (err != 0) {
		isp_info_desc_tag("BIST: get output buff err\n");
		err = EINVAL;
		goto self_test_raw_done_end;
	}

	/* Set back the raw buffer by reserve buffer */
	if (g_bist_info[g_current_test_case].
		frame_count[0][V4L2_ISP_RAW] == SS_FIRST_RAW_FRAME_IDX ||
		g_bist_info[g_current_test_case].
		frame_count[0][V4L2_ISP_RAW] ==
		g_self_test_info[g_current_test_case].max_frame_cnt) {
		isp_info_desc_tag("BIST: Set reserve buffer back");
		err = ispdrv_set_output_buffer(sensor_id, image_type,
				g_reserve_raw_addr[g_raw_index], 0, 0, 0);
		if (err != 0) {
			isp_info_desc_tag("BIST: set output buff err\n");
			err = EINVAL;
			goto self_test_raw_done_end;
		}
		g_proc_still_raw_addr[g_raw_index] = output_image_addr;
		MEMCPY((u8 *)&g_raw_iq_info_1[g_raw_index],
			(u8 *)iq_info_1, sizeof(struct s_iq_info_1));
		MEMCPY((u8 *)&g_raw_iq_info_2[g_raw_index],
			(u8 *)iq_info_2, sizeof(struct s_iq_info_2));
		g_bist_proc_still_start[g_raw_index] = true;
		g_raw_index = (g_raw_index + 1) % 2;
	} else {
		err = ispdrv_set_output_buffer(sensor_id, image_type,
			output_image_addr, 0, 0, 0);
		if (err != 0) {
			isp_info_desc_tag("BIST: set output buff err\n");
			goto self_test_raw_done_end;
		}
	}
	/* Change the raw frame rate, if needed*/
	if (g_bist_info[g_current_test_case].
		frame_count[0][V4L2_ISP_RAW] ==
		(g_self_test_info[g_current_test_case].max_frame_cnt / 2)) {
		if (change_raw_frame_rate == false) {
			if (g_raw_frame_rate[g_current_test_case][1] !=
				ISP_RAW_FR_OFF &&
				g_raw_frame_rate[g_current_test_case][1] !=
				ISP_RAW_FR_MAX &&
				g_raw_frame_rate[g_current_test_case][1] !=
				g_raw_frame_rate[g_current_test_case][0]) {
				isp_info_desc_tag(
					"BIST: Change the raw frame rate");
				err = ispdrv_set_raw_frame_rate(
					g_bist_info[g_current_test_case].
					scenario_id[0],
					g_raw_frame_rate[
					g_current_test_case][1]);
				if (err) {
					isp_info_desc_tag(
						"BIST: Set Raw FR Err\n");
				}
			}
			change_raw_frame_rate = true;
		}
	} else if (g_bist_info[g_current_test_case].
		frame_count[0][V4L2_ISP_RAW] ==
		g_self_test_info[g_current_test_case].max_frame_cnt) {
		err = ispdrv_set_raw_frame_rate(sensor_id, ISP_RAW_FR_OFF);
		if (err) {
			isp_info_desc_tag(
				"BIST: SetRawFrameRate Off Err\n");
		}


	}
self_test_raw_done_end:
	kfree((void *)iq_info_1);
	kfree((void *)iq_info_2);

	isp_info_lo_end_tag();

}

static u32 self_test_start_proc_still(void)
{
	u32 err = 0;
	u8 proc_still_index = 0;
	struct ispdrv_output_addr_still out_addr;

	isp_info_lo_start_tag();

	for (proc_still_index = 0; proc_still_index < 2; proc_still_index++) {
		while (g_bist_proc_still_start[proc_still_index] == false) {
			isp_info_desc_tag("BIST: Wait no.%d raw frame done\n",
				proc_still_index);
			mdelay(1000);
		}
		isp_info_desc_tag("BIST: Raw image is ready");

		self_test_fill_ss_hq_opt_param(
			&g_proc_still_opt[proc_still_index]);
		/* Update writing mode */
		g_proc_still_opt[proc_still_index].hq_setting.writing_mode =
			g_ss_opt_writing_mode[g_current_test_case];
		isp_info_desc_tag("BIST: writing mode %d",
			g_proc_still_opt[proc_still_index].
			hq_setting.writing_mode);

		if (proc_still_index == 0)
			out_addr.buffer_addr =
				g_image_addr[0][V4L2_ISP_IMG3].ud_addr1;
		else
			out_addr.buffer_addr =
				g_image_addr[0][V4L2_ISP_IMG3].ud_addr2;

		isp_info_desc_tag(
			"BIST: Current TestCase %d, index %d, Sensor %d",
			g_current_test_case, proc_still_index,
			g_bist_info[g_current_test_case].sensor_id[0]);

		isp_info_desc_tag("BIST: rawAddr 0x%x, outAddr 0x%x",
			g_reserve_raw_addr[proc_still_index],
			out_addr.buffer_addr);

		isp_info_desc_tag("BIST: ISO Speed 0x%x",
			g_raw_iq_info_1[proc_still_index].
			irp_info.reserved[4]);
		/* Proc still for 1st Raw */
		err = ispdrv_proc_still(
			g_bist_info[g_current_test_case].sensor_id[0],
			g_proc_still_raw_addr[proc_still_index],
			out_addr,
			&g_proc_still_opt[proc_still_index],
			&g_raw_iq_info_1[proc_still_index],
			&g_raw_iq_info_2[proc_still_index]);
		if (err != 0) {
			isp_info_desc_tag("BIST: Proc Still no.%d Err 0x%x\n",
				proc_still_index, err);
			goto self_test_start_proc_still_end;
		}
		while (g_bist_proc_still_end[proc_still_index] == false) {
			isp_info_desc_tag(
				"BIST: Wait no.%d still frame done\n",
				proc_still_index);
			mdelay(1000);
		}
	}

self_test_start_proc_still_end:

	isp_info_lo_end_tag();
	return err;

}

static void self_test_clear_global_variables(void)
{
	u8 test, order, type;

	isp_info_lo_start_tag();

	g_test_case_number = 0;
	g_debug_mode = false;
	memset((u8 *) g_scenario_info, 0,
		sizeof(struct s_scenario_info_ap) *
		MAX_TEST_CASE_NUMBER * MAX_USED_SENSOR_NUMBER);
	memset((u8 *) g_output_buff_format, 0,
		sizeof(struct s_output_buff_format_info) *
		MAX_TEST_CASE_NUMBER * MAX_USED_SENSOR_NUMBER);
	memset((u8 *) g_self_test_info, 0,
		sizeof(struct s_output_buff_format_info) *
		MAX_TEST_CASE_NUMBER);
	memset((u8 *) g_bist_info, 0, sizeof(struct s_bist_info) *
		MAX_TEST_CASE_NUMBER);
	memset((u8 *) g_raw_frame_rate, 0,
		sizeof(enum ispdrv_raw_frame_rate) *
		MAX_TEST_CASE_NUMBER * 2);
	memset((u8 *) g_iso_speed, 0,
		sizeof(u32) * MAX_TEST_CASE_NUMBER);
	memset((u8 *) g_ss_opt_writing_mode, 0,
		sizeof(enum ispdrv_sl_output_writing_mode) *
		MAX_TEST_CASE_NUMBER);
	/* free memory */
	for (test = 0; test < MAX_TEST_CASE_NUMBER; test++) {
		for (order = 0; order < MAX_USED_SENSOR_NUMBER; order++) {
			for (type = 0; type < MAX_GOLDEN_IMG_NUMBER; type++) {
				if (g_golden_name[test][order][type] != NULL)
					kfree((void *) g_golden_name[
						test][order][type]);
				g_golden_name[test][order][type] = NULL;
			}
		}
	}

	isp_info_lo_end_tag();

}

static u8 self_test_get_sensor_id(
	struct s_bist_info *bist_info,
	u8 test_case_number, u8 order)
{
	u8 sensor_id;

	isp_info_lo_start_tag();

	if (bist_info[test_case_number].
		scenario_id[order] ==
		V4L2_ISP_SCENARIO_PREVIEW_SENSOR_2)
		sensor_id = SENSOR2;
	else if (bist_info[test_case_number].
		scenario_id[order] ==
		V4L2_ISP_SCENARIO_PREVIEW_SENSOR_LITE)
		sensor_id = SENSOR3;
	else
		sensor_id = SENSOR1;

	isp_info_desc_tag("Order:%d, SensorID:%d",
		order, sensor_id);
	isp_info_lo_end_tag();
	return sensor_id;
}


static void self_test_allocate_image_addr(u8 scenario_id,
	u8 image_type,
	struct output_addr_setting *output_addr,
	struct s_output_buff_format_info *output_buff_format,
	u32 *memory_start_addr,
	u32 *memory_offset)
{
	u32 start_addr = *memory_start_addr;
	u32 offset = *memory_offset;
	u32 image_size = 0;
	u32 line_offset = output_buff_format->width[image_type];

	isp_info_lo_start_tag();

	if (image_type == V4L2_ISP_STATITISTICS)
		image_size = MAX_SELF_TEST_METADATA_SIZ;
	else if (image_type == V4L2_ISP_STATITISTICS_OF_AF)
		image_size = MAX_SELF_TEST_METADATA_OF_AF_SIZ;
	else if (scenario_id == V4L2_ISP_SCENARIO_PREVIEW_STILL_SS &&
		image_type == V4L2_ISP_IMG3)
		image_size = ((output_buff_format->width[image_type] + 8) *
			output_buff_format->height[image_type]) * 2;
	else
		image_size = output_buff_format->width[image_type] *
			output_buff_format->height[image_type] * 2;

	/* Allocate memory address according to the buffer  number */
	switch (output_buff_format->buffer_number[image_type]) {
	case 1:
		output_addr->ud_addr1 = start_addr + offset;
		offset = image_size;
		start_addr = output_addr->ud_addr1;

		if (scenario_id == V4L2_ISP_SCENARIO_PREVIEW_STRIPE &&
			image_type != V4L2_ISP_STATITISTICS &&
			image_type != V4L2_ISP_STATITISTICS_OF_AF)
			output_addr->ud_addr1_isp1 =
				output_addr->ud_addr1 + line_offset;
		break;
	case 2:
		output_addr->ud_addr1 = start_addr + offset;
		/* Update offset and start address */
		offset = image_size;
		start_addr = output_addr->ud_addr1;

		output_addr->ud_addr2 = start_addr + offset;
		/* Update start address */
		start_addr = output_addr->ud_addr2;

		if (scenario_id == V4L2_ISP_SCENARIO_PREVIEW_STRIPE &&
			image_type != V4L2_ISP_STATITISTICS &&
			image_type != V4L2_ISP_STATITISTICS_OF_AF) {
			output_addr->ud_addr1_isp1 =
				output_addr->ud_addr1 + line_offset;
			output_addr->ud_addr2_isp1 =
				output_addr->ud_addr2 + line_offset;
		}
		break;
	case 3:
		output_addr->ud_addr1 = start_addr + offset;
		/* Update offset and start address */
		offset = image_size;
		start_addr = output_addr->ud_addr1;

		output_addr->ud_addr2 = start_addr + offset;
		/* Update start address */
		start_addr = output_addr->ud_addr2;

		output_addr->ud_addr3 = start_addr + offset;
		/* Update start address */
		start_addr = output_addr->ud_addr3;

		if (scenario_id == V4L2_ISP_SCENARIO_PREVIEW_STRIPE &&
			image_type != V4L2_ISP_STATITISTICS &&
			image_type != V4L2_ISP_STATITISTICS_OF_AF) {
			output_addr->ud_addr1_isp1 =
				output_addr->ud_addr1 + line_offset;
			output_addr->ud_addr2_isp1 =
				output_addr->ud_addr2 + line_offset;
			output_addr->ud_addr3_isp1 =
				output_addr->ud_addr3 + line_offset;
		}
		break;
	case 4:
		output_addr->ud_addr1 = start_addr + offset;
		/* Update offset and start address */
		offset = image_size;
		start_addr = output_addr->ud_addr1;

		output_addr->ud_addr2 = start_addr + offset;
		/* Update start address */
		start_addr = output_addr->ud_addr2;

		output_addr->ud_addr3 = start_addr + offset;
		/* Update start address */
		start_addr = output_addr->ud_addr3;

		output_addr->ud_addr4 = start_addr + offset;
		/* Update start address */
		start_addr = output_addr->ud_addr4;

		if (scenario_id == V4L2_ISP_SCENARIO_PREVIEW_STRIPE &&
			image_type != V4L2_ISP_STATITISTICS &&
			image_type != V4L2_ISP_STATITISTICS_OF_AF) {
			output_addr->ud_addr1_isp1 =
				output_addr->ud_addr1 + line_offset;
			output_addr->ud_addr2_isp1 =
				output_addr->ud_addr2 + line_offset;
			output_addr->ud_addr3_isp1 =
				output_addr->ud_addr3 + line_offset;
			output_addr->ud_addr4_isp1 =
				output_addr->ud_addr4 + line_offset;
		}
		break;
	}
	/* Store back */
	*memory_start_addr = start_addr;
	*memory_offset = offset;

	isp_info_lo_end_tag();
}

static u32 self_test_load_qmerge(void)
{
	u32 err = 0;
	const struct firmware *qmerge_ptr;

	isp_info_lo_start_tag();

	if (request_firmware
		(&qmerge_ptr, "TBM_Qmerge_Sensor1.bin",
		(struct device *)bist_get_v4l2_device())) {
		isp_err_lo_combo_desc_tag("get Sensor 1 Qmerge failed.");
		err = EINVAL;
	} else {
		if (qmerge_ptr->size > MAX_QMERGE_BIN_SIZE) {
			isp_err_pu_combo_desc_tag(
				"Sensor 1 Qmerge size is too big.");
			err = EINVAL;
		} else {
			isp_info_desc_tag("BIST:Qmerge 1 iva 0x%x\n",
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR1));
			isp_info_desc_tag("BIST:Qmerge 1 kva 0x%llx\n",
				self_test_get_kva(
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR1)));
			MEMCPY((u8 *) self_test_get_kva(
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR1)),
				(void *)qmerge_ptr->data, qmerge_ptr->size);
			dma_flush_range(
				(void *)self_test_get_kva(
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR1)),
				(void *)(self_test_get_kva(
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR1)) +
				qmerge_ptr->size));
		}
		release_firmware(qmerge_ptr);
	}

	if (request_firmware
		(&qmerge_ptr, "TBM_Qmerge_Sensor2.bin",
		(struct device *)bist_get_v4l2_device())) {
		isp_err_pu_combo_desc_tag("get Sensor 2 Qmerge failed.");
		err = EINVAL;
	} else {
		if (qmerge_ptr->size > MAX_QMERGE_BIN_SIZE) {
			isp_err_pu_combo_desc_tag(
				"Sensor 2 Qmerge size is too big.");
			err = EINVAL;
		} else {
			isp_info_desc_tag("BIST:Qmerge 2 iva 0x%x\n",
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR2));
			isp_info_desc_tag("BIST:Qmerge 2 kva 0x%llx\n",
				self_test_get_kva(
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR2)));
			MEMCPY((u8 *) self_test_get_kva(
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR2)),
				(void *)qmerge_ptr->data, qmerge_ptr->size);
			dma_flush_range(
				(void *)self_test_get_kva(
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR2)),
				(void *)(self_test_get_kva(
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR2)) +
				qmerge_ptr->size));
		}
		release_firmware(qmerge_ptr);
	}

	if (request_firmware
		(&qmerge_ptr, "TBM_Qmerge_Sensor3.bin",
		(struct device *)bist_get_v4l2_device())) {
		isp_err_pu_combo_desc_tag("get Sensor 3 Qmerge failed.");
		err = EINVAL;
	} else {
		if (qmerge_ptr->size > MAX_QMERGE_BIN_SIZE) {
			isp_err_pu_combo_desc_tag(
				"Sensor 3 Qmerge size is too big.");
			err = EINVAL;
		} else {
			isp_info_desc_tag("BIST:Qmerge 3 iva 0x%x\n",
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR3));
			isp_info_desc_tag("BIST:Qmerge 3 kva 0x%llx\n",
				self_test_get_kva(
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR3)));
			MEMCPY((u8 *) self_test_get_kva(
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR3)),
				(void *)qmerge_ptr->data, qmerge_ptr->size);
			dma_flush_range(
				(void *)self_test_get_kva(
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR3)),
				(void *)(self_test_get_kva(
				common_get_qmerge_data_addr_by_sensor_id(
				SENSOR3)) +
				qmerge_ptr->size));

		}
		release_firmware(qmerge_ptr);
	}

	isp_info_lo_end_tag();
	return err;
}

static void self_test_load_golden_image(
	u32 *golden_addr,
	struct s_output_buff_format_info *output_buff_format,
	bool *bypass_flag,
	char **golden_name,
	bool *is_compare)
{
	u32 err = 0;
	const struct firmware *golden_ptr;
	u8 image_type;
	u32 image_size;

	isp_info_lo_start_tag();

	for (image_type = 0; image_type < 3; image_type++) {
		image_size = output_buff_format->width[image_type] *
			output_buff_format->height[image_type] * 2;
		isp_info_desc_tag("BIST: GoldenName:%s",
			golden_name[image_type]);
		isp_info_desc_tag("BIST: GoldenIVA:0x%x",
			golden_addr[image_type]);
		isp_info_desc_tag("BIST: GoldenKVA:0x%llx",
			self_test_get_kva(golden_addr[image_type]));
		is_compare[image_type] = false;
		if (bypass_flag[image_type] == BYPASS)
			continue;

		err = request_firmware(&golden_ptr, golden_name[image_type],
			(struct device *)bist_get_v4l2_device());

		if (err) {
			isp_err_pu_combo_desc_tag(
				"BIST: get golden image%d faled. Err0x%x",
				image_type, err);
		} else {
			if (golden_ptr->size > image_size) {
				isp_err_pu_combo_desc_tag(
					"BIST: Golden image%d is too big.",
					image_type);
			} else {
				MEMCPY((u8 *) self_test_get_kva(
					golden_addr[image_type]),
					(void *)golden_ptr->data,
					golden_ptr->size);
				dma_flush_range(
					(void *) self_test_get_kva(
					golden_addr[image_type]),
					(void *)(self_test_get_kva(
					golden_addr[image_type]) +
					golden_ptr->size));
				is_compare[image_type] = true;
			}
			release_firmware(golden_ptr);
		}
	}

	isp_info_lo_end_tag();
}

static void self_test_set_bypass_flag(u8 scenario_id,
	bool *bypass_flag,
	struct s_scinfo_out_bypassflg *out_bypass_flag)
{
	bypass_flag[V4L2_ISP_IMG1] = out_bypass_flag->bypass_lv;
	bypass_flag[V4L2_ISP_IMG2] = out_bypass_flag->bypass_video;
	bypass_flag[V4L2_ISP_IMG3] = out_bypass_flag->bypass_still;
	bypass_flag[V4L2_ISP_STATITISTICS] = out_bypass_flag->bypass_metadata;
	bypass_flag[V4L2_ISP_RAW] = (
		(scenario_id ==
		V4L2_ISP_SCENARIO_PREVIEW_STILL_SS)?(NOT_BYPASS):(BYPASS));
}

static u64 self_test_get_kva(u32 iva)
{
	return g_bist_buff_kva + (u64)(iva - g_bist_buff_iva);
}
/*********************** New BIST Private Function ***************************/

/*********************** New BIST End ************************************/

