/*
* File: altek_isp_drv_local.h                                             *
* Description: local constant definition                            *
*                                                                           *
* (C)Copyright altek Corporation 2015                                       *
*                                                                           *
* History:                                                                  *
* 2016/03/18; Chavenal Tang; Initial version                                *
*/
#ifndef _ALTEK_ISP_DRV_LOCAL
#define _ALTEK_ISP_DRV_LOCAL

/* #define DEBUG_LOAD_QMERGE_AND_SHADING_BINARY */
/* #define DEBUG_LOAD_FW_BINARY */

/* Buff size definition*/
#define BUFF_SIZE_ISP_SHARE_BUFF	0x100000
#ifdef DEBUG_LOAD_QMERGE_AND_SHADING_BINARY
#define BUFF_SIZE_ISP_FW		0x200000
#define BUFF_SIZE_ISP_SHADING	0x160000
#define BUFF_SIZE_ISP_QMERGE	(0x300000 - BUFF_SIZE_ISP_SHADING)
#else
#define BUFF_SIZE_ISP_FW		0x500000
#endif
/* 14M bytes*/
#define BUFF_SIZE_ISP_WORKING	0xE00000

/* 10M byte for saving one sensor's frame done img*/
#define BUF_SIZE_ISP_SELF_TEST_SENSOR_FD_BUFF	0xA00000
/* 14M byte for remaining golden img*/
#define BUF_SIZE_ISP_SELF_TEST_GOLDEN_BUFF	0xE00000
/* 66M bytes for 6792x3816*/
#define BUF_SIZE_ISP_RAW_BUF		0x4200000
/* 33M bytes for 6792x3816*/
#define BUF_SIZE_ISP_HQ_MODE_RAW_BUF	0x2100000
/* 64M bytes for 6792x3816*/
#define BUF_SIZE_ISP_HQ_MODE_HQ_BUF	 0x4000000

/* ISP Log buffer size*/
#define LOG_BUFFER_SIZE	  0x1000

/* Log task buffer size*/
#define BUFFER_SIZE	220

#ifdef DEBUG_LOAD_QMERGE_AND_SHADING_BINARY
#define MAX_QMERGE_BIN_SIZE	0x50000
#define MAX_SHADING_BIN_SIZE 0x72000
#endif

enum ispdrv_sensor1_path_mode {
	ISP_S1_PATH_MODE_DIRECT,
	ISP_S1_PATH_MODE_DRAM,
	ISP_S1_PATH_MODE_DIRECT_STRIPE,
	ISP_S1_PATH_MODE_DRAM_STRIPE,
	ISP_S1_PATH_MODE_DRAM_STRIPE_BUF,
	ISP_S1_PATH_MODE_HQ,
	ISP_S1_PATH_MODE_SS,
	ISP_S1_PATH_MODE_MAX
};

enum ispdrv_share_buffer_index {
	/* Index 0: Firmware Log Buffer */
	ISP_SHARE_BUFF_INDEX_FIRMWARE_LOG_BUFF,
	/* Index 1: Manual Data of Sensor 1 */
	ISP_SHARE_BUFF_INDEX_SENSOR_1_MANUAL_DATA,
	/* Index 2: Manual Data of Sensor 2 */
	ISP_SHARE_BUFF_INDEX_SENSOR_2_MANUAL_DATA,
	/* Index 3: Manual Data of Sensor 3 */
	ISP_SHARE_BUFF_INDEX_SENSOR_3_MANUAL_DATA,
	/* Index 4: RGB2YUV Matrix of Sensor 1 */
	ISP_SHARE_BUFF_INDEX_SENSOR_1_RGB2YUV,
	/* Index 5: RGB2YUV Matrix of Sensor 2 */
	ISP_SHARE_BUFF_INDEX_SENSOR_2_RGB2YUV,
	/* Index 6: RGB2YUV Matrix of Sensor 3 */
	ISP_SHARE_BUFF_INDEX_SENSOR_3_RGB2YUV,
	/* Index 7: Tone Mapping of Sensor 1 */
	ISP_SHARE_BUFF_INDEX_SENSOR_1_TONEMAP,
	/* Index 8: Tone Mapping of Sensor 2 */
	ISP_SHARE_BUFF_INDEX_SENSOR_2_TONEMAP,
	/* Index 9: Tone Mapping of Sensor 3 */
	ISP_SHARE_BUFF_INDEX_SENSOR_3_TONEMAP,
	/* Index 10: Motion of Sensor 1 */
	ISP_SHARE_BUFF_INDEX_SENSOR_1_MOTION,
	/* Index 11: Motion of Sensor 2 */
	ISP_SHARE_BUFF_INDEX_SENSOR_2_MOTION,
	/* Index 12: Motion of Sensor 3 */
	ISP_SHARE_BUFF_INDEX_SENSOR_3_MOTION,
	/* Index 13: Brightness Gain of Sensor 1 */
	ISP_SHARE_BUFF_INDEX_SENSOR_1_BRIGHTNESS,
	/* Index 14: Brightness Gain of Sensor 2 */
	ISP_SHARE_BUFF_INDEX_SENSOR_2_BRIGHTNESS,
	/* Index 15: Brightness Gain of Sensor 3 */
	ISP_SHARE_BUFF_INDEX_SENSOR_3_BRIGHTNESS,
	/* Index 16: ISP D-Gain of Sensor 1 */
	ISP_SHARE_BUFF_INDEX_SENSOR_1_DGAIN,
	/* Index 17: ISP D-Gain of Sensor 2 */
	ISP_SHARE_BUFF_INDEX_SENSOR_2_DGAIN,
	/* Index 18: ISP D-Gain of Sensor 3 */
	ISP_SHARE_BUFF_INDEX_SENSOR_3_DGAIN,
	/* Index 19: Download Sequence Info of Sensor 1 */
	ISP_SHARE_BUFF_INDEX_SENSOR_1_DLD_SEQ_INFO,
	/* Index 20: Download Sequence Info of Sensor 2 */
	ISP_SHARE_BUFF_INDEX_SENSOR_2_DLD_SEQ_INFO,
	/* Index 21: Download Sequence Info of Sensor 3 */
	ISP_SHARE_BUFF_INDEX_SENSOR_3_DLD_SEQ_INFO,
	/* Index 22: Configuration of 3A Info of Sensor 1 */
	ISP_SHARE_BUFF_INDEX_SENSOR_1_CFG_3A_INFO,
	/* Index 23: Configuration of 3A Info of Sensor 2 */
	ISP_SHARE_BUFF_INDEX_SENSOR_2_CFG_3A_INFO,
	/* Index 24: Configuration of 3A Info of Sensor 3 */
	ISP_SHARE_BUFF_INDEX_SENSOR_3_CFG_3A_INFO,
	/* Index 25: Digital Zoom Info of Sensor 1 */
	ISP_SHARE_BUFF_INDEX_SENSOR_1_DZOOM_INFO,
	/* Index 26: Digital Zoom Info of Sensor 2 */
	ISP_SHARE_BUFF_INDEX_SENSOR_2_DZOOM_INFO,
	/* Index 27: Digital Zoom Info of Sensor 3 */
	ISP_SHARE_BUFF_INDEX_SENSOR_3_DZOOM_INFO,
	/* Index 28: Scenario Info */
	ISP_SHARE_BUFF_INDEX_SCENARIO_INFO,
	/* Index 29: Debug IQ Info of Sensor 1 */
	ISP_SHARE_BUFF_INDEX_SENSOR_1_DEBUG_IQ_INFO,
	/* Index 30: Debug IQ Info of Sensor 2 */
	ISP_SHARE_BUFF_INDEX_SENSOR_2_DEBUG_IQ_INFO,
	/* Index 31: Debug IQ Info of Sensor 3 */
	ISP_SHARE_BUFF_INDEX_SENSOR_3_DEBUG_IQ_INFO,
	/* Index 32: CCM Info of Sensor 1 */
	ISP_SHARE_BUFF_INDEX_SENSOR_1_CCM_INFO,
	/* Index 33: CCM Info of Sensor 2 */
	ISP_SHARE_BUFF_INDEX_SENSOR_2_CCM_INFO,
	/* Index 34: CCM Info of Sensor 3 */
	ISP_SHARE_BUFF_INDEX_SENSOR_3_CCM_INFO,
	/* Index 35: OTP Info of Sensor 1 */
	ISP_SHARE_BUFF_INDEX_SENSOR_1_OTP_INFO,
	/* Index 36: OTP Info of Sensor 2 */
	ISP_SHARE_BUFF_INDEX_SENSOR_2_OTP_INFO,
	/* Index 37: OTP Info of Sensor 3 */
	ISP_SHARE_BUFF_INDEX_SENSOR_3_OTP_INFO,
	/* Index 38: Argument of Processing Still of Sensor 1 */
	ISP_SHARE_BUFF_INDEX_SENSOR_1_PROC_STILL_ARG,
	/* Index 39: Argument of Processing Still of Sensor 2 */
	ISP_SHARE_BUFF_INDEX_SENSOR_2_PROC_STILL_ARG,
	/* Index 40: Argument of Processing Still of Sensor 3 */
	ISP_SHARE_BUFF_INDEX_SENSOR_3_PROC_STILL_ARG,
	/* Index 41: IQ Function */
	ISP_SHARE_BUFF_INDEX_IQ_FUNC,
	/* Index 42: Extended Buffer Info */
	ISP_SHARE_BUFF_INDEX_EXTEND_BUFF_INFO,
	/* Index 43: The Total Index of Share Buff */
	ISP_SHARE_BUFF_INDEX_MAX
};

u32 common_get_iva_fw_addr(void);

u64 common_get_ahb_addr(void);

u32 common_get_ahb_size(void);

void common_get_format(u8 sensor, u8 img_type, u8 *format);

void common_get_buff_number(u8 *buff_ptr);

void common_get_sensor_id(u8 *sensor_id);

void common_get_out_field_table(u32 *output_field_ptr);

bool common_is_qmerge_data_addr_set(void);

void common_reset_bist_frame_done_cnt(void);

u32 common_get_qmerge_data_addr_by_sensor_id(u8 sensor_id);

u32 common_get_shading_data_addr_by_sensor_id(u8 sensor_id);

u32 common_get_1stsensor_frame_done_cnt(void);

u32 common_get_2ndsensor_frame_done_cnt(void);

void isp_drv_init(void *pri_data);
void isp_drv_deinit(void *pri_data);
#endif
