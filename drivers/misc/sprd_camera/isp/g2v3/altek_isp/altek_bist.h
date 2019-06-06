/*
* File: altek_bist.h                                                *
* Description: delcalration of bist api                             *
*                                                                           *
* (C)Copyright altek Corporation 2016                                       *
*                                                                           *
* History:                                                                  *
*   2016/03/012; Caedmon Lai; Initial version                               *
*/

#ifndef _ALTEK_BIST
#define _ALTEK_BIST

#include "altek_sc_info.h"

/* Self test*/
#define MAX_SELF_TEST_LV_SIZE		0x400000
#define MAX_SELF_TEST_VIDEO_SIZE	0x160000
#define MAX_SELF_TEST_STILL_SIZE	0x400000
#define MAX_SELF_TEST_METADATA_SIZ	0x50000
#define MAX_SELF_TEST_METADATA_OF_AF_SIZ	0x5000
#define MAX_SELF_TEST_GOLDEN_SIZE_960x720	0x160000
#define MAX_SELF_TEST_GOLDEN_SIZE_640x360	0x80000
#define MAX_SELF_TEST_GOLDEN_SIZE_1920x1080	0x400000

#define MAX_SELF_TEST_RAW_SIZE_960x720  0x160000
#define SS_MAX_HQ_BUFFER_SIZE   0x2dd0000
#define SS_LV_BUFFER_SIZE       0x152000
#define SS_RAW_BUFFER_SIZE      0xf78000
#define SS_STILL_BUFFER_SIZE    0x1734000
#define SS_FIRST_RAW_FRAME_IDX      1
#define SS_SECOND_RAW_FRAME_IDX     30

#define MAX_QMERGE_BIN_SIZE		0x60000

#define BYPASS				true
#define NOT_BYPASS			false

#define MAX_TEST_CASE_NUMBER		8
#define MAX_USED_SENSOR_NUMBER		2
#define MAX_RAW_FR_RATE_CHANGE		2
#define MAX_GOLDEN_IMG_NUMBER		3
#define MAX_FILE_PATH_SIZE		256

#pragma pack(push)
#pragma pack(4)
struct s_output_buff_format_info {
	u8 enable[V4L2_ISP_OUTPUT_IMAGE_TOTAL];
	u8 format[V4L2_ISP_OUTPUT_IMAGE_TOTAL];
	u32 width[V4L2_ISP_OUTPUT_IMAGE_TOTAL];
	u32 height[V4L2_ISP_OUTPUT_IMAGE_TOTAL];
	u8 buffer_number[V4L2_ISP_OUTPUT_IMAGE_TOTAL];
	u16 line_offset[V4L2_ISP_OUTPUT_IMAGE_TOTAL];
};
#pragma pack(pop)

#pragma pack(push)
#pragma pack(4)
struct s_bist_info {
	u8 scenario_id[MAX_USED_SENSOR_NUMBER];
	u8 sensor_id[MAX_USED_SENSOR_NUMBER];
	u16 frame_count[MAX_USED_SENSOR_NUMBER][S_OUTPUT_IMAGE_TOTAL];
	bool meet_max_frame_count[MAX_USED_SENSOR_NUMBER];
};
#pragma pack(pop)

extern void ispdrv_set_bist_addr(u32 bist_buffer_iva, u64 bist_buffer_kva);
extern u32 ispdrv_bist(void);
extern bool ispdrv_is_bist_mode(void);
void bist_set_test_case_number_setting(u8 test_case_number);
void bist_set_scenario_info_setting(
	struct s_scenario_info_ap scenario_info[][MAX_USED_SENSOR_NUMBER]);
void bist_set_output_buff_format_setting(
	struct s_output_buff_format_info
	output_buff_format[][MAX_USED_SENSOR_NUMBER]);
void bist_set_self_test_info_setting(
	struct s_scinfo_self_test_info *self_test_info);
void bist_set_bist_info_setting(
	struct s_bist_info *bist_info);
void bist_set_raw_frame_rate_setting(
	enum ispdrv_raw_frame_rate raw_frame_rate[][MAX_RAW_FR_RATE_CHANGE]);
void bist_set_isp_speed_setting(u32 *iso_speed);
void bist_set_golden_name_setting(
	char *(*golden_name)[MAX_USED_SENSOR_NUMBER][3]);
void bist_set_debug_mode_setting(bool debug_mode);

void bist_set_ss_opt_writing_mode_setting(
	enum ispdrv_sl_output_writing_mode *writing_mode);

void bist_set_hq_buffer_size_setting(u32 *hq_buff_size);

void bist_set_raw_buffer_size_setting(u32 *raw_buff_size);

u32 bist_set_preparation(u32 *start_addr, u32 *offset);
void bist_set_firmware_addr(u64 firmware_addr);
void bist_set_image_address(u8 scenario_id,
	struct output_addr_setting *output_addr,
	u32 *reserve_raw_addr,
	bool *bypass_flag,
	struct s_output_buff_format_info *output_buff_format,
	u32 *memory_start_addr,
	u32 *memory_offset);

void bist_set_golden_address(u32 *golden_addr,
	bool *bypass_flag,
	struct s_output_buff_format_info *output_buff_format,
	u32 *memory_start_addr,
	u32 *memory_offset);

u32 bist_set_scenario_info(u8 sensor_id,
	u8 scenario_id,
	struct s_scenario_info_ap *scenario_info);

u32 bist_set_output_buff_format(u8 sensor_id,
	struct output_addr_setting *image_addr,
	struct s_output_buff_format_info *output_buff_format,
	bool *bypass_flag);
u32 bist_preview_stream_on(u8 sensor_id,
	u8 scenario_id,
	bool *bypass_flag,
	enum ispdrv_raw_frame_rate *raw_frame_rate);
u32 bist_preview_stream_off(u8 sensor_id,
	bool *bypass_flag,
	enum ispdrv_raw_frame_rate *raw_frame_rate);
void bist_inactivate_test_pattern(u8 sensor_id);
void bist_check_sum(u32 *golden_sum, u32 *golden_xor,
	u32 *output_sum, u32 *output_xor);


/*
 *\isp memory configuration, this function must be called before ispdrv_open()
 *\param iva_self_test_img1_addr[in], Self Test frame done address for IMG1
 *\param iva_self_test_img2_addr[in], Self Test frame done address for IMG2
 *\param iva_self_test_img3_addr[in], Self Test frame done address for IMG3
 *\param iva_self_test_metadata_addr[in]
 *\, Self Test frame done address for Metadata
 *\return null
 */
void ispdrv_set_self_test_addr(u32 iva_bist_buffer, u64 kva_bist_buffer);

/*
 *\brief ISP Build In Self Test function
 *\param scenario_id [In], Scenario ID
 *\return error code
 */
extern u32 ispdrv_self_test(u8 scenario_id);

/*
 *\brief Set the starting address of ahb register
 *\param ahb_addr [In], the starting address of ahb register
 *\return null
 */
void bist_set_ahb_addr(u64 ahb_addr);

/*
 *\brief Get the starting address of ahb register
 *\param ahb_addr [Out], the starting address of ahb register
 *\return null
 */
u64 bist_get_ahb_addr(void);

/*
 *\brief Set the size of ahb register
 *\param ahb_size [In], the size of ahb register
 *\return null
 */
void bist_set_ahb_size(u32 ahb_size);

/*
 *\brief Get the size of ahb register
 *\param ahb_size [Out], the size of ahb register
 *\return null
 */
u32 bist_get_ahb_size(void);

/*
 *\brief Set the device info
 *\param bist_v4l2_device [In], the device info
 *\return null
 */
void bist_set_v4l2_device(void *bist_v4l2_device);

/*
 *\brief Get the device info
 *\param bist_v4l2_device [Out], the device info
 *\return null
 */
void *bist_get_v4l2_device(void);

/*
 *\brief Get the input size
 *\param sensor_index [In], the sensor index, 1st sensor = 1, 2nd sensor = 2
 *\param input_width [Out], the input width
 *\param input_height [Out], the input height
 *\return null
 */
void bist_get_input_size(u8 sensor_index
	, u16 *input_width, u16 *input_height);

/*
 *\brief Set the input size
 *\param sensor_index [In], the sensor index, 1st sensor = 1, 2nd sensor = 2
 *\param input_width [In], the input width
 *\param input_height [In], the input height
 *\return null
 */
void bist_set_input_size(u8 sensor_index
	, u16 input_width, u16 input_height);

/*
 *\brief check the sensor id
 *\param sensor_index [In], the sensor index, 1st sensor = 1, 2nd sensor = 2
 *\param sensor_id [In], the sensor id
 *\return result
 */
bool bist_check_sensor_id(u8 sensor_index, u8 sensor_id);

/*
 *\brief get the addresses of golden images
 *\param direct_addr [In],
 *	        the address of single direct mode and stripe direct mode
 *\param dram_lv_video_addr [In],
 *	        the address of the lv and video images of dram mode
 *\param dram_normal_quality_still_addr [In],
 *	        the address of still image in normal quality dram mode
 *\param dram_high_quality_still_addr [In],
 *	        the address of still image in high quality dram mode
 *\param front_addr [In], the address of front mode
*\return result
 */
void bist_get_golden_addr(u64 *direct_addr, u64 *dram_lv_video_addr,
			      u64 *dram_normal_quality_still_addr,
			      u64 *dram_high_quality_still_addr,
			      u64 *front_addr);

/*
 *\brief check the addresses used in bist is set
 *\param none
*\return the flag, 0 = not set, 1 = set
 */
u8 bist_get_self_test_addr_set(void);

/*
 *\brief set the flag indicated the addresses used in bist is set
 *\param none
*\return none
 */
void bist_set_self_test_addr_set(void);

/*
 *\brief reset the flag indicated whether the addresses used in bist is set
 *\param none
*\return none
 */
void bist_reset_self_test_addr_set(void);

/*
 *\brief reset the first sensor id and the second sensor id
 *\param none
*\return none
 */
void bist_reset_sensor_id(void);
/*
 *\brief add the frame done count
 *	    and set frame done flag if the count is satisfied
 *\param scenario_info_isp [In], the scenario info
 *\param frame_done_cnt [In], the frame dont count
 *\param sensor_idx [In], the sensor index,
 *	        1 = first sensor, 2 = second sensor
*\return none
 */
void bist_set_frame_done_count(
		    struct s_scenario_info_isp *scenario_info_isp,
		    u8 *frame_done_cnt,
		    u8 sensor_idx);

/*
 *\brief set wait flag if not frame done
 *\param scenario_id [In], the scenario id
 *\param scenario_info [In], the scenario_info
*\return true/false
 */
bool bist_check_frame_done(u8 scenario_id,
		    struct s_scenario_info_isp *scenario_info);

/*
 *\brief get the dual sensor mode
 *\param scenario_info [In], the scenario_info
*\return true/false
 */
bool bist_is_dual_sensor_mode(struct s_scenario_info_isp *scenario_info);

bool bist_is_independent_mode(struct s_scenario_info_isp *scenario_info);
/*
 *\brief get the self test mode
 *\param scenario_info [In], the scenario_info
*\return true/false
 */
u8 bist_get_self_test_mode(struct s_scenario_info_isp *scenario_info);

/*
 *\brief set the flag of indicating independent mode
 *\param is_enable [In], the flag
*\return none
 */
void bist_set_independent_flag(bool is_enable);

/*
 *\brief get the flag of indicating independent mode
 *\param none
*\return the flag
 */
bool bist_get_independent_flag(void);

/*
 *\brief set the flag of reversing the closing sequence in independent mode
 *\param is_reverse_close_sequence [In], the flag
*\return none
 */
void bist_set_reverse_flag(bool is_reverse_close_sequence);

/*
 *\brief get the flag of reversing the closing sequence in independent mode
 *\param None
*\return the flag
 */
bool bist_get_reverse_flag(void);

/*
 *\brief analysis the interrupted flags of related frame done INT for BIST
 *\param flags [In], the interrupted flags
 *\param scenario_info [Out], the scenario info
 *\param frame_done_count_1stsensor [Out],
 *            the frame done count of first sensor
 *\param frame_done_count_2ndsensor [Out],
 *            the frame done count of second sensor
 *\return None
 */
u32 bist_irq_interrupt(u32 flags,
		struct s_scenario_info_isp *scenario_info,
		u8 *frame_done_count_1stsensor,
		u8 *frame_done_count_2ndsensor);

/*
 *\brief handle the interrupt of related frame done INT for BIST
 *\param sensor_id [In], the sensor id
 *\param scenario_info [Out], the scenario info
 *\param frame_done_count_1stsensor [Out],
 *            the frame done count of first sensor
 *\param frame_done_count_2ndsensor [Out],
 *            the frame done count of second sensor
 *\return None
 */
void bist_interrupt_handle(u8 sensor_id,
		struct s_scenario_info_isp *scenario_info,
		u8 *frame_done_count_1stsensor,
		u8 *frame_done_count_2ndsensor);

u32 bist_proc_still_frame_done_irq_proc(u8 sensor_id, u8 image_type);

void bist_init(void *pri_data);
void bist_deinit(void *pri_data);
#endif
