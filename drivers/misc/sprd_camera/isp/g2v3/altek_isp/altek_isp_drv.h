/*
 * File: Altek_ispdrv.h                                                      *
 * Description: The structure and API definition about altek ISP driver      *
 * (C)Copyright Altek Digital Inc. 2014                                      *
 *                                                                           *
 * History:                                                                  *
 *   2014/08/01; Bruce Chung; Initial version                                *
 */


/*
 * \file     Altek_ispdrv.h
 * \brief    altek ISP driver structure and API define
 * \version  0.01
 * \author   Bruce Chung
 * \date     2014/08/01
 * \see      Altek_ispdrv.h
 */


#ifndef ALTEK_ISP
#define ALTEK_ISP

#include "altek_sc_info.h"
#include "altek_os_info.h"

/*
 * //////////////////////////////////////////////////////////////////////////
*                          Public Flag Declaration                          *
 * /////////////////////////////////////////////////////////////////////////
 */
#define SP_FW_V_1
/* #define SP_FW_V_1_TEST */
/* #define ENABLE_TEST_PATTERN_SENSOR_1 */
/* #define ENABLE_TEST_PATTERN_SENSOR_2 */
/* #define ENABLE_TEST_PATTERN_SENSOR_LITE */
/* #define TESTISP */
/* #define SIMULATE_NEW_CMD_FLOW */
#ifdef SIMULATE_NEW_CMD_FLOW
/* #define SIMULATE_CFG_3A */
/* #define DEBUG_LOAD_SHADING_BINARY */
/* #define DEBUG_LOAD_QMERGE_BINARY */
/* #define DEBUG_BUILD_IN_SELF_TEST */
#endif



/*
 * //////////////////////////////////////////////////////////////////////////
*                          Public Type Declaration                          *
 * //////////////////////////////////////////////////////////////////////////
 */


/*
 *@typedef ispdrv_motion_input
 *@brief User defined tone map curve.(259 elements in each curve)
 */
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct ispdrv_motion_input {
	u8 x;
	u8 y;
};
#pragma pack(pop)

/*
 *@typedef ispdrv_scenario_mode
 *@brief Define scenario modes
 *@see V4L2_CID_SCENARIO_MODE
 */
enum ispdrv_scenario_mode {
	V4L2_ISP_SCENARIO_PREVIEW_SENSOR_1 = 2,
	V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT, /* obsolete */
	V4L2_ISP_SCENARIO_PREVIEW_SENSOR_LITE,
	V4L2_ISP_SCENARIO_PREVIEW_STRIPE,
	V4L2_ISP_SCENARIO_PREVIEW_STILL_SS,
	V4L2_ISP_SCENARIO_PREVIEW_SENSOR_2,
	V4L2_ISP_SCENARIO_TOTAL
};

#define ISP_SELF_TEST_TOTAL    6

struct ispdrv_self_test_buff_addr_info {
	u32 ud1st_sensor_img1_addr;
	u32 ud1st_sensor_img2_addr;
	u32 ud1st_sensor_img3_addr;
	u32 ud1st_sensor_metadata_addr;
	u32 ud2nd_sensor_img1_addr;
	u32 ud2nd_sensor_img2_addr;
	u32 ud2nd_sensor_img3_addr;
	u32 ud2nd_sensor_metadata_addr;
	u32 udgolden_direct_addr;
	u32 udgolden_dram_LV_VIDEO_addr;
	u32 udgolden_dram_NQ_STILL_addr;
	u32 udgolden_dram_HQ_STILL_addr;
	u32 udgolden_front_addr;
};

struct ispdrv_self_test_comp_info {
	u8 uc_comp_number;
	u8 auc_comp_idx[3];
};

/*
 *@struct ISPDRV_EXBUFFER_INFO
 *@brief Buffer info of DRAM normal mode or DRAM High quality mode
 */
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct ispdrv_exbuffer_info {
	u32 udExBufAddr;
	u32 udExBufSize;
	u32 udHighQaulityBufAddr;
	u32 udHighQaulityBufSize;
};
#pragma pack(pop)


/*
 *@typedef ISPDRV_OUTPUT_TYPE
 *@brief Define ISP output type.(0:RAW10, 1:NV12, 2:YUY2)
 *       3:ALTEK RAW10
 */
enum ispdrv_output_type {
	V4L2_ISP_OUTPUT_RAW10,
	V4L2_ISP_OUTPUT_NV12,
	V4L2_ISP_OUTPUT_YUY2,/* default */
	V4L2_ISP_OUTPUT_ALTEK_RAW10,
	V4L2_ISP_OUTPUT_TYPE_TOTAL
};

/*
 *@struct ISPDRV_OUTPUT_FORMAT
 *@brief Define ISP output format
 *  output format setting
 *  isp_lite only need to filled img1 and statistics fields.
 *  set height and width to 0 if the output image doens't used
 */
struct ispdrv_output_format {
	u8 output_type;	/* ISPDRV_OUTPUT_TYPE */
	/* Img1, Img2, Img3 size info */
	u16 img1_width;
	u16 img1_height;
	u16 img2_width;
	u16 img2_height;
	u16 img3_width;
	u16 img3_height;
	/* Img1, Img2, Img3, 3A statistic  buffer number */
	u8 img1_buffer_number;
	u8 img2_buffer_number;
	u8 img3_buffer_number;
	u8 statistic_buffer_number;
	/* Img1, Img2, Img3, 3A statistics addr info */
	u32 img1_addr1;
	u32 img1_addr2;
	u32 img1_addr3;
	u32 img1_addr4;
	u32 img2_addr1;
	u32 img2_addr2;
	u32 img2_addr3;
	u32 img2_addr4;
	u32 img3_addr1;
	u32 img3_addr2;
	u32 img3_addr3;
	u32 img3_addr4;
	u32 statistics_addr1;
	u32 statistics_addr2;
	u32 statistics_addr3;
	u32 statistics_addr4;
	/* Img1, Img2, Img3, 3A statistics buffer size */
	u32 img1_buff_size;
	u32 img2_buff_size;
	u32 img3_buff_size;
	u32 statistics_buff_size;
};


/*
 *@ISPDRV_OUTPUT_IMAGE
 *@brief Define ISP output image
 */
enum ispdrv_output_image {
	V4L2_ISP_IMG1,
	V4L2_ISP_IMG2,
	V4L2_ISP_IMG3,
	V4L2_ISP_STATITISTICS,
	V4L2_ISP_STATITISTICS_OF_AF,
	V4L2_ISP_RAW,
	V4L2_ISP_OUTPUT_IMAGE_TOTAL = S_OUTPUT_IMAGE_TOTAL
};

/*
 *@ISPDRV_IMAGE_INDEX
 *@brief Define ISP image buffer index
 */
enum ispdrv_image_index {
	V4L2_ISP_INDEX1,
	V4L2_ISP_INDEX2,
	V4L2_ISP_INDEX3,
	V4L2_ISP_INDEX4,
	V4L2_ISP_INDEX_TOTAL,
	V4L2_ISP_INDEX_MAX = 255
};


/*
 *@struct ISPDRV_MANUAL_DATA
 *@brief Define ISP manual data for manual setting
 */
struct ispdrv_manual_data {
	u8 reserved_0;
	u8 reserved_1;
	u16 reserved_2;
};


/*
 *@struct ISPDRV_SENSOR_INFO
 *@brief Define Sensor Information
 */
struct ispdrv_sensor_info {
	u16 width;
	u16 height;
	u16 framerate;
	enum e_scinfo_color_order color_order;
};

enum ispdrv_int_sys_shift {
	SHIFT_SYS_BOOT_FINISH = 0,
	SHIFT_SYS_CMD_FINISH,
	SHIFT_SYS_IP_ERR,
	SHIFT_SYS_MAX
};

enum ispdrv_int_sensor1_shift {
	SHIFT_SENSOR1_OUT_IMG1_DONE = SHIFT_SYS_MAX,
	SHIFT_SENSOR1_OUT_IMG2_DONE,
	SHIFT_SENSOR1_OUT_IMG3_DONE,
	SHIFT_SENSOR1_3A_DONE,
	SHIFT_SENSOR1_AF_DONE,
	SHIFT_SENSOR1_3A_SOF,
	SHIFT_SENSOR1_3A_LINEMEET,
	SHIFT_SENSOR1_STILL_PROC_START,
	SHIFT_SENSOR1_RAW_DONE,
	SHIFT_SENSOR1_MAX
};

enum ispdrv_int_sensor2_shift {
	SHIFT_SENSOR2_OUT_IMG1_DONE = SHIFT_SENSOR1_MAX,
	SHIFT_SENSOR2_OUT_IMG2_DONE,
	SHIFT_SENSOR2_OUT_IMG3_DONE,
	SHIFT_SENSOR2_3A_DONE,
	SHIFT_SENSOR2_AF_DONE,
	SHIFT_SENSOR2_3A_SOF,
	SHIFT_SENSOR2_3A_LINEMEET,
	SHIFT_SENSOR2_MAX
};

enum ispdrv_int_sensor3_shift {
	SHIFT_SENSOR3_OUT_IMG_DONE = SHIFT_SENSOR2_MAX,
	SHIFT_SENSOR3_3A_DONE,
	SHIFT_SENSOR3_AF_DONE,
	SHIFT_SENSOR3_3A_SOF,
	SHIFT_SENSOR3_3A_LINEMEET,
	SHIFT_SENSOR3_MAX,
};

#define ASIC_REVID_GENESIS	0x0
#define ASIC_REVID_G2V1		0x4
#define ASIC_REVID_G2V3		0x5

enum ispdrv_asic_chip_id {
	ASIC_CHIP_UNKNOW = 0,
	ASIC_CHIP_ID_GENESIS,
	ASIC_CHIP_ID_G2V1,
	ASIC_CHIP_ID_G2V3,
	ASIC_CHIP_ID_MAX,
	ASIC_CHIP_ID_UNDEFINED = 0xFFFF,
};

/*
 *@typedef ispdrv_int
 *@brief Define ISP interrupt type for System, Sensor1, Sensor2 and Sensor3
 */
enum ispdrv_int {
	V4L2_ISP_INT_NONE                     = 0,
	/* System INT Boot finish,Cmd finish, IP error */
	/* 0x00000001 */
	V4L2_ISP_SYS_BOOT_FINISH_INT          =
			(0x00000001 << SHIFT_SYS_BOOT_FINISH),
	/* 0x00000002 */
	V4L2_ISP_SYS_CMD_FINISH_INT           =
			(0x00000001 << SHIFT_SYS_CMD_FINISH),
	/* 0x00000004 */
	V4L2_ISP_SYS_IP_ERROR_INT             =
			(0x00000001 << SHIFT_SYS_IP_ERR),
	/* Sensor1 output img1, Img2, Img3 done */
	/* 0x00000008 */
	V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT    =
			(0x00000001 << SHIFT_SENSOR1_OUT_IMG1_DONE),
	/* 0x00000010 */
	V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT    =
			(0x00000001 << SHIFT_SENSOR1_OUT_IMG2_DONE),
	/* 0x00000020 */
	V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT    =
			(0x00000001 << SHIFT_SENSOR1_OUT_IMG3_DONE),
	/* Sensor1 3A done, AF lock, AE lock */
	/* 0x00000040 */
	V4L2_ISP_SENSOR1_3A_DONE_INT          =
			(0x00000001 << SHIFT_SENSOR1_3A_DONE),
    /* 0x00000080 */
	V4L2_ISP_SENSOR1_AF_DONE_INT          =
			(0x00000001 << SHIFT_SENSOR1_AF_DONE),
    /* 0x00000100 */
	V4L2_ISP_SENSOR1_3A_SOF_INT          =
			(0x00000001 << SHIFT_SENSOR1_3A_SOF),
    /* 0x00000200 */
	V4L2_ISP_SENSOR1_3A_LINEMEET_INT          =
			(0x00000001 << SHIFT_SENSOR1_3A_LINEMEET),
    /* 0x00000400 */
	V4L2_ISP_SENSOR1_STILL_PROC_START_INT =
			(0x00000001 << SHIFT_SENSOR1_STILL_PROC_START),
    /* 0x00000800 */
	V4L2_ISP_SENSOR1_RAW_DONE_INT =
			(0x00000001 << SHIFT_SENSOR1_RAW_DONE),
	/* Sensor2 output img1, img2, img3 done */
	/* 0x00001000 */
	V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT    =
			(0x00000001 << SHIFT_SENSOR2_OUT_IMG1_DONE),
    /* 0x00002000 */
	V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT    =
			(0x00000001 << SHIFT_SENSOR2_OUT_IMG2_DONE),
    /* 0x00004000 */
	V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT    =
			(0x00000001 << SHIFT_SENSOR2_OUT_IMG3_DONE),
	/* Sensor2 3A done, AF lock, AE lock */
	/* 0x00008000 */
	V4L2_ISP_SENSOR2_3A_DONE_INT          =
			(0x00000001 << SHIFT_SENSOR2_3A_DONE),
    /* 0x00010000 */
	V4L2_ISP_SENSOR2_AF_DONE_INT          =
			(0x00000001 << SHIFT_SENSOR2_AF_DONE),
    /* 0x00020000 */
	V4L2_ISP_SENSOR2_3A_SOF_INT          =
			(0x00000001 << SHIFT_SENSOR2_3A_SOF),
    /* 0x00040000 */
	V4L2_ISP_SENSOR2_3A_LINEMEET_INT          =
			(0x00000001 << SHIFT_SENSOR2_3A_LINEMEET),
	/* Sensor3 output img done */
	/* 0x00080000 */
	V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT     =
			(0x00000001 << SHIFT_SENSOR3_OUT_IMG_DONE),
	/* Sensor3 3A done, AF lock, AE lock */
	/* 0x00100000 */
	V4L2_ISP_SENSOR3_3A_DONE_INT          =
			(0x00000001 << SHIFT_SENSOR3_3A_DONE),
    /* 0x00200000 */
	V4L2_ISP_SENSOR3_AF_DONE_INT          =
			(0x00000001 << SHIFT_SENSOR3_AF_DONE),
    /* 0x00400000 */
	V4L2_ISP_SENSOR3_3A_SOF_INT          =
			(0x00000001 << SHIFT_SENSOR3_3A_SOF),
    /* 0x00800000 */
	V4L2_ISP_SENSOR3_3A_LINEMEET_INT          =
			(0x00000001 << SHIFT_SENSOR3_3A_LINEMEET),
};

/*
 *@struct output_addr_setting
 *@brief output addresses of images
 */
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
struct output_addr_setting {
	u32 ud_addr1;
	u32 ud_addr2;
	u32 ud_addr3;
	u32 ud_addr4;
	u32 ud_addr1_isp1;
	u32 ud_addr2_isp1;
	u32 ud_addr3_isp1;
	u32 ud_addr4_isp1;
	u32 ud_addr1_nv12_uv;
	u32 ud_addr2_nv12_uv;
	u32 ud_addr3_nv12_uv;
	u32 ud_addr4_nv12_uv;
	u32 ud_addr1_nv12_uv_isp1;
	u32 ud_addr2_nv12_uv_isp1;
	u32 ud_addr3_nv12_uv_isp1;
	u32 ud_addr4_nv12_uv_isp1;
};
#pragma pack(pop)

#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
enum ispdrv_raw_format {
	/* 0 reserved */
	ISP_RAW_FORMAT_UNPACK_RAW10 = 1,
	/* 2 reserved */
	ISP_RAW_FORMAT_ALTEK_RAW10  = 3,
	ISP_RAW_FORMAT_MAX
};
#pragma pack(pop)

/*
 *@brief frame rate control on Raw top's DRAM.
 */
enum ispdrv_raw_frame_rate {
	/* All off */
	ISP_RAW_FR_OFF,
	/* All on, taking per 1 frame */
	ISP_RAW_FR_1,
	/* taking per 2 frames */
	ISP_RAW_FR_2,
	/* taking per 3 frames */
	ISP_RAW_FR_3,
	/* taking per 4 frames */
	ISP_RAW_FR_4,
	ISP_RAW_FR_MAX,
};

struct ispdrv_output_addr_still {
	/* ycbcr or y */
	u32 buffer_addr;
	u32 buffer_addr_nv12uv;
};

enum ispdrv_quality_path {
	ISP_NORMAL_QUALITY,
	ISP_HIGH_QUALITY
};

enum ispdrv_ycc_format {
	ISP_YCC_420,
	ISP_YCC_422
};

enum ispdrv_sl_output_writing_mode {
	ISP_LINE_WRITING_MODE,
	ISP_TILE_WRITING_MODE,
};

#pragma pack(push)
#pragma pack(4)
struct ispdrv_hq_setting {
	/* YCC420 or YCC422 */
	u8 ycc_format;
	u8 cfr_enabled;
	u8 y_nr_enabled;
	u8 sharpness_enabled;
	/* line or tile mode */
	u8 writing_mode;
};
#pragma pack(pop)

#pragma pack(push)
#pragma pack(4)
struct ispdrv_proc_still_opt {
	/* HQ or NQ */
	u8 quality_path;
	struct ispdrv_hq_setting hq_setting;
};
#pragma pack(pop)

#pragma pack(push)
#pragma pack(4)
struct ispdrv_proc_still_pref {
	u32 src_addr;
	struct ispdrv_output_addr_still out_addr;
	struct ispdrv_proc_still_opt opt;
	struct s_iq_info_1 iq_info_1;
	struct s_iq_info_2 iq_info_2;
};
#pragma pack()

/* V4L2 irq handle define */
typedef void (*h8_v4l2_irq_func)(unsigned int flag);

#define ALIGN_CEIL(x, n)	(((x)+((n)-1))/(n)*(n))
/*
 * ///////////////////////////////////////////////////////////
 *                  Boot Sequence API                         *
 * ///////////////////////////////////////////////////////////
 */
/*
 *\isp memory configuration, this function must be called before ispdrv_open()
 *\param iva_fw_addr[in], firmware base address
 *\param kva_fw_kaddr[in], kerneal address of firmware
 *\return null
 */

void ispdrv_set_firmware_mem_info(u32 iva_fw_addr, u64 fw_kaddr);

/*
 *\isp memory configuration, this function must be called before ispdrv_open()
 *\param iva_raw_buffer_addr[in], the base address of raw buffer
 *\param isp_raw_buf_size[in], Size of raw buffer
 *\return null
 */
void ispdrv_set_raw_buffer_mem_info(u32 iva_raw_buffer_addr,
	u32 isp_raw_buf_size);

/*
 *\isp memory configuration, this function must be called before ispdrv_open()
 *\param iva_high_quality_buffer_addr[in],
 *	        the base address of high quality buffer
 *\param isp_high_quality_buf_size[in], Size of high quality mode buffer
 *\return null
 */
void ispdrv_set_hq_buffer_mem_info(u32 iva_high_quality_buffer_addr,
	u64 isp_high_quality_buf_size);

/*
 *\isp memory configuration, this function must be called before ispdrv_open()
 *\param iva_shading_sensor_1_addr[in], Shading address for sensor 1
 *\param iva_shading_sensor_2_addr[in], Shading address for sensor 2
 *\param iva_shading_sensor_3_addr[in], Shading address for sensor 3
 *\return null
 */
void ispdrv_set_shading_addr(u32 iva_shading_sensor_1_addr,
	u32 iva_shading_sensor_2_addr, u32 iva_shading_sensor_3_addr);

/*
 *\isp memory configuration, this function must be called before ispdrv_open()
 *\param iva_qmerge_sensor_1_addr[in], Qmerge address for sensor 1
 *\param iva_qmerge_sensor_2_addr[in], Qmerge address for sensor 2
 *\param iva_qmerge_sensor_3_addr[in], Qmerge address for sensor 3
 *\return null
 */
void ispdrv_set_qmerge_addr(u32 iva_qmerge_sensor_1_addr,
	u32 iva_qmerge_sensor_2_addr, u32 iva_qmerge_sensor_3_addr);

/*
*\brief initial all used global variables
*\param none
*\return nond
*/
extern void ispdrv_initialize_global_variable(void);

/*
 *\brief Open ISP driver to init AHB config then start ISP firmware
 *\param a_udAHBAddr [in], ahb base address
 *\param a_udAHBSize [in], the range of ahb addr for isp
 *\param v4l2_device [in], v4l2 device
 *\return EINVAL(DRAM size is too small),
 *	       PTR_ERR(Creat a thread of log-task fail)
 */
extern u32 ispdrv_open(u64 a_udAHBAddr, u32 a_udAHBSize, void *v4l2_device);

/*
 *\brief Stop log task, Set AHB in reset state
 *\return No error code, always return 0
 */
extern u32 ispdrv_close(void);

extern void ispdrv_set_buf_status(int status);

/*
*\brief Get the chip id of ASIC
*\param chip_id [Out], the chip id
*\return null
*/
extern void ispdrv_get_isp_chip_id(u16 *chip_id);

extern bool init_isp_interrupt(unsigned int irq, unsigned long flags,
		    const char *device);

extern void isp_register_irq_CB_func(h8_v4l2_irq_func pFunc, int type);


/*
 * ///////////////////////////////////////////////////////////
 *             Mode change / Preview on sequence API          *
 * ///////////////////////////////////////////////////////////
 */

/*
 *\brief Set scenario Information of each sensor of the senario ID
 *\param a_uc_scenario_id [in], Scenario ID (ispdrv_scenario_mode)
 *\param a_pt_sensor1_info[in], Scenario Info of Sensor1
 *\param a_pt_sensor2_info[in], Scenario Info of Sensor2
 *\param a_pt_sensor3_info[in], Scenario Info of Sensor3
 *\return EINVAL(Invalid scenario ID)
 */
u32 ispdrv_set_scenario_info(u8 a_uc_scenario_id,
			struct s_scenario_info_ap *a_pt_sensor1_info,
			struct s_scenario_info_ap *a_pt_sensor2_info,
			struct s_scenario_info_ap *a_pt_sensor3_info);

/*
 *\brief Set specific bypass flags out of scenario info.
 *       This must be called before ispdrv_set_preview_mode.
 *\param a_pt_sns1_bypassflg[in], bypass flag of sensor 1
 *\param a_pt_sns2_bypassflg[in], bypass flag of sensor 2
 *\param a_pt_sns3_bypassflg[in], bypass flag of sensor 3
 */
u32 ispdrv_set_scinfo_out_bypass_flag(
		struct s_scinfo_out_bypassflg *a_pt_sns1_bypassflg,
		struct s_scinfo_out_bypassflg *a_pt_sns2_bypassflg,
		struct s_scinfo_out_bypassflg *a_pt_sns3_bypassflg);
/*
 *\brief Set Preview Mode (for ZSL mode)
 *\param a_uc_scenario_id [in], Scenario ID (ispdrv_scenario_mode)
 *\return EINVAL(Invalid scenario ID)
 *\bug Under construct
 */
extern u32 ispdrv_set_preview_mode(u8 a_uc_scenario_id);

/*
 *\brief stop stream path
 *\param a_uc_sensor [in], sensor ID
 *\return error code
 */
extern u32 ispdrv_stop_preview_mode(u8 a_uc_sensor);

/*
 *\brief Preview stream on for sensor1/sensor2/sensor3 (1: on , 0: ignore)
 *	    Enable sensor status INT, Set op start
 *\param a_uc_sensor [in], sensor ID
 *\param a_uc_output_image [in],
 *	        Img1 for Preview,Img2 for video , Img3 for still,
 *	        refert to ISPDRV_OUTPUT_IMAGE
 *\return ETIMEDOUT.(Without AHB command finish INT)
 */
extern u32 ispdrv_preview_stream_on(u8 a_uc_sensor, u8 a_uc_output_image);

/*
 *\brief Preview stream off for sensor1/sensor2/sensor3 (1: off , 0: ignore)
 *       Disable sensor status INT, Clear op start
 *\param a_uc_sensor [in], sensor ID
 *\param a_uc_output_image [in],
 *	        Img1 for Preview,Img2 for video , Img3 for still,
 *	        refert to ISPDRV_OUTPUT_IMAGE
 *\return ETIMEDOUT.(Without AHB command finish INT)
 */
u32 ispdrv_preview_stream_off(u8 a_uc_sensor, u8 a_uc_output_image);

/*
 *\brief Set the output buffer format
 *\param a_uc_sensor [in], sensor ID
 *\param a_uc_output_image [in],
 *	        Img1 for Preview,Img2 for video , Img3 for still,
 *	        refert to ISPDRV_OUTPUT_IMAGE
 *\param a_uc_enable [in], disable: 0,  enable: 1, enable the output to dram
 *\param a_uc_format[in], output format, refer to ISPDRV_OUTPUT_TYPE
 *\param a_ud_width  [in], output width
 *\param a_ud_height [in], output height
 *\param a_uc_buffer_number [in], buffer number
 *\param a_pt_output_addr_setting [in], structure of image addresses
 *\param a_uw_line_offset [in], image line offset of Y, should be alinged to 8
 *\return No error code, always return 0
 */
extern u32 ispdrv_set_output_buff_format(u8 a_uc_sensor, u8 a_uc_output_image,
		   u8 a_uc_enable, u8 a_uc_format,
		   u32 a_ud_width, u32 a_ud_height,
		   u8 a_uc_buffer_number,
		   struct output_addr_setting *a_pt_output_addr_setting,
		   u16 a_uw_line_offset);

/*
 *\brief Get the output buffer info
 *	    by output image index and output image buffer index
 *\param a_uc_sensor [in], sensor ID. (Sensor1/Sensor2/Sensor3)
 *\param a_uc_output_image [in], output source, refer to ISPDRV_OUTPUT_IMAGE
 *\param a_pud_output_buffer [out], dequeue the buffer.(0/1/2/3)
 *\param a_pud_output_buffer_isp1[out],
 *	       dequeue the buffer for ISP1 in stripe mode.(0/1/2/3)
 *\param a_pud_output_buffer_nv12uv[out],
 *	       dequeue the buffer for UV of NV12 format.(0/1/2/3)
 *\param a_pud_output_buffer_nv12uv_isp1 [out],
 *	       dequeue the buffer for UV of NV12 format
 *	        for ISP1 in stripe mode.(0/1/2/3)
 *\param a_pt_crop_size [out], the crop size of the output image
 *\param a_pt_iq_info_1 [out], the IQ information of the frame in JPEG exif
 *\param a_pt_iq_info_2 [out], the IQ information  of the frame in JPEG debug
 *\return error code
 */
extern u32 ispdrv_get_output_buffer(u8 a_uc_sensor, u8 a_uc_output_image,
		   u32 *a_pud_output_buffer,
		   u32 *a_pud_output_buffer_isp1,
		   u32 *a_pud_output_buffer_nv12uv,
		   u32 *a_pud_output_buffer_nv12uv_isp1,
		   struct s_scenario_size *a_pt_crop_size,
		   struct s_iq_info_1 *a_pt_iq_info_1,
		   struct s_iq_info_2 *a_pt_iq_info_2);

/*
 *\brief Check if there si more image ready
 *\param a_uc_sensor [in], sensor ID
 *\param a_uc_output_image [in], output source
 *\param a_pb_more_img [out], true if there is more imgs that are ready
 *\return Error code
 */
extern u32 ispdrv_is_more_img_ready(u8 a_uc_sensor,
		   u8 a_uc_output_image,
		   bool *a_pb_more_img);


/*
 *\brief Set the output buffer
 *\param a_uc_sensor [in], sensor ID
 *\param a_uc_output_image [in], output source
 *\param a_ud_buffer_addr [in], enqueue the buffer
 *\param a_ud_buffer_addr_isp1 [in], enqueue the buffer for ISP1 in stripe mode
 *\param a_ud_buffer_addr_nv12uv [in], enqueue the buffer for UV of NV12 format
 *\param a_ud_buffer_addr_nv12uv_isp1 [in],
 *	        enqueue the buffer for UV of NV12 format for ISP1 in stripe mode
 *\return Error code
 */
extern u32 ispdrv_set_output_buffer(u8 a_uc_sensor, u8 a_uc_output_image,
		   u32 a_ud_buffer_addr,
		   u32 a_ud_buffer_addr_isp1,
		   u32 a_ud_buffer_addr_nv12uv,
		   u32 a_ud_buffer_addr_nv12uv_isp1);




/*
 *\brief Get base size for digital zoom
*\param a_uc_sensor [In], Sensor1/ Sensor2
*\param a_uc_output_image [In], LV/ Video/ Still
 *\param a_puw_base_width [out], the base width
 *\param a_puw_base_height [out], the base height
 */
extern  u32 ispdrv_get_base_size_for_digital_zoom(u8 a_uc_sensor,
	u8 a_uc_output_image,
	u16 *a_puw_base_width, u16 *a_puw_base_height);


/*
 *\brief Set crop size for digital zoom
 *\param a_uc_sensor [In], the sensor ID
 *\param a_pt_lv [In], the crop size of LV img
 *\param a_pt_video [In], the crop size of video img
 */
extern u32 ispdrv_set_crop_size_for_digital_zoom(u8 a_uc_sensor,
	struct s_scenario_size *a_pt_lv,
	struct s_scenario_size *a_pt_video,
	struct s_scenario_size *a_pt_still);

/*
 *\brief Get the address of the scenario info
 *	   that is shared between ISP driver and firmware for bist
 *\param scenario_info_ptr [Out], the scenario info
 *\return none
 */
#if 0
extern void ispdrv_get_shared_scenario_info_ptr(
	struct s_scenario_info_isp *scenario_info_ptr);
#endif
extern void *ispdrv_get_shared_scenario_info_ptr(void);

/*
 *\brief Skip image
 *\param sensor [in], sensor ID
 *\param output_img [in], output source
 *\return Error code
 */
extern u32 ispdrv_skip_img(u8 sensor, u8 output_img);

/*
 * \brief frame rate control on Raw_top's DRAM output raw path
 * \param sensor_id [in], sensor id
 * \param raw_fr [in], see ispdrv_raw_frame_rate
 * \return Error code
 */
extern u32 ispdrv_set_raw_frame_rate(u8 sensor_id, u8 raw_fr);

/*
 * \param sensor_id [in]
 * \param src_addr [in], raw's address
 * \param out_addr [in], expected YCC output address
 * \param opt [in], HQ/NQ path and other preferences
 * \param a_pt_iq_info_1[in], got from function
 *                      "ispdrv_get_output_buffer"
 * \param a_pt_iq_info_2[in], got from function
 *                      "ispdrv_get_output_buffer"
 */
extern u32 ispdrv_proc_still(u8 a_uc_sensor,
		u32 src_addr,
		struct ispdrv_output_addr_still out_addr,
		struct ispdrv_proc_still_opt *opt,
		struct s_iq_info_1 *a_pt_iq_info_1,
		struct s_iq_info_2 *a_pt_iq_info_2);

/**
 *\brief Set RAW format in dump RAW scenarios.
 *       e.g. V4L2_ISP_SCENARIO_DUMP_RAW10_W_NORMAL_QUALITY,
 *            V4L2_ISP_SCENARIO_DUMP_RAW10_W_HIGH_QUALITY
 *\param sensor_id [in], sensor id
 *\param format, See ispdrv_raw_format.
 *               Default is ISP_RAW_FORMAT_ALTEK_RAW10
 *\return Error code
 */
extern u32 ispdrv_set_raw_format(u8 sensor_id, u8 format);

/**
 *\brief Set CBC map addresses.
 *\param sns1_iva[in], the address of the CBC Map addr for sensor1.
 *\param sns2_iva[in], the address of the CBC Map addr for sensor2.
 *\param sns3_iva[in], the address of the CBC Map addr for sensor3.
 *\return Error code
 */
extern u32 ispdrv_set_cbc_map_addr(u32 sns1_iva, u32 sns2_iva, u32 sns3_iva);


/*
*\brief Enable independent AF interrupt mode.
*\param None
*\return EINVAL
*/
extern u32 ispdrv_set_ind_af_int(void);


/*
* This workround whale2 ISP(ARC) access register bug.
*/
extern int altek_isp_opened(void);


/*
 * ///////////////////////////////////////////////////////////
 *                        Others                              *
 * //////////////////////////////////////////////////////////
 */

/* Triple Tone R channel, 259 elements */
#define _LVHWIRPCTRL_TRIPLE_R_TONE_    {\
	      0, 0, 1, 1, 3, 3, 4, 4, 6, 6, 6, 7, 7, 8, 8, 10, 10, 11, 11,\
	      13, 13, 14, 14, 14, 16, 16, 17, 17, 18, 18, 20, 20,\
	      21, 21, 23, 23, 24, 24, 25, 27, 27, 28, 28, 30, 30,\
	      31, 31, 32, 34, 34, 35, 35, 37, 38, 38, 40, 40, 41,\
	      42, 42, 44, 45, 45, 47, 48, 48, 49, 51, 52, 52, 54,\
	      55, 57, 57, 58, 59, 61, 61, 62, 64, 65, 67, 68, 68,\
	      69, 71, 72, 74, 75, 77, 78, 79, 81, 82, 84, 85, 87,\
	      88, 89, 91, 92, 94, 95, 97, 98, 99, 101, 102, 104,\
	      105, 108, 109, 111, 112, 114, 115, 118, 119, 120,\
	      122, 125, 126, 127, 129, 131, 133, 134, 136, 138,\
	      140, 141, 144, 145, 148, 149, 150, 153, 154, 157,\
	      158, 159, 162, 163, 165, 167, 169, 170, 173, 174,\
	      176, 177, 180, 181, 183, 186, 187, 189, 190, 192,\
	      194, 195, 197, 198, 200, 202, 203, 205, 207, 208,\
	      210, 211, 213, 215, 215, 217, 218, 220, 221, 222,\
	      223, 224, 226, 226, 228, 228, 230, 230, 232, 232,\
	      233, 234, 234, 235, 236, 237, 238, 238, 238, 239,\
	      240, 240, 241, 241, 242, 242, 243, 243, 244, 244,\
	      244, 245, 245, 245, 246, 246, 246, 247, 247, 247,\
	      248, 248, 248, 248, 249, 249, 249, 250, 250, 250,\
	      250, 250, 251, 251, 251, 251, 251, 252, 252, 252,\
	      252, 252, 253, 253, 253, 253, 253, 253, 254, 254,\
	      254, 254, 254, 255, 255, 255, 255, 255, 255, 255,\
}

/* Triple Tone G channel, 259 elements */
#define _LVHWIRPCTRL_TRIPLE_G_TONE_    {\
	      0, 0, 2, 2, 4, 4, 6, 6, 8, 8, 8, 10, 10, 12, 12, 14, 14,\
	      16, 16, 18, 18, 20, 20, 20, 22, 22, 24, 24, 26, 26,\
	      28, 28, 30, 30, 32, 32, 34, 34, 36, 38, 38, 40, 40,\
	      42, 42, 43, 43, 45, 47, 47, 49, 49, 51, 53, 53, 55,\
	      55, 56, 58, 58, 60, 62, 62, 64, 65, 65, 67, 69, 71,\
	      71, 72, 74, 76, 76, 77, 79, 81, 81, 82, 84, 85, 87,\
	      88, 88, 90, 91, 93, 94, 96, 97, 99, 100, 101, 103, \
	      104, 105, 107, 108, 109, 111, 112, 113, 114, 116,\
	      117, 118, 119, 120, 121, 123, 125, 126, 127, 128,\
	      129, 130, 132, 133, 134, 135, 137, 138, 139, 140,\
	      142, 143, 143, 144, 146, 147, 148, 149, 150, 152,\
	      153, 153, 155, 156, 157, 158, 159, 160, 161, 162,\
	      163, 164, 165, 166, 167, 168, 169, 170, 171, 172,\
	      173, 174, 175, 175, 176, 178, 178, 179, 180, 181,\
	      182, 183, 184, 185, 186, 187, 187, 188, 189, 190,\
	      191, 192, 193, 194, 195, 196, 196, 197, 198, 199,\
	      200, 201, 202, 203, 203, 205, 205, 206, 207, 208,\
	      209, 210, 211, 211, 213, 213, 214, 215, 216, 217,\
	      218, 219, 219, 220, 221, 222, 222, 223, 224, 225,\
	      225, 226, 227, 228, 229, 229, 230, 231, 232, 233,\
	      234, 234, 235, 236, 237, 237, 238, 239, 239, 240,\
	      241, 241, 242, 243, 244, 245, 245, 245, 246, 247,\
	      248, 248, 249, 250, 251, 251, 252, 252, 253, 253,\
	      254, 255, 255, 255, 255,\
}

 /* Triple Tone B channel, 259 elements */
#define _LVHWIRPCTRL_TRIPLE_B_TONE_    {\
	      0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 4, 4, 4, 4,\
	      5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 9, 9, 9, 9, 10,\
	      10, 11, 11, 11, 12, 12, 13, 13, 13, 13, 14, 14,\
	      15, 15, 15, 16, 16, 17, 18, 18, 18, 18, 19, 20,\
	      20, 21, 21, 21, 22, 23, 23, 24, 24, 25, 25, 26,\
	      27, 28, 28, 29, 30, 30, 30, 31, 32, 33, 34, 35,\
	      35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 46, 47,\
	      48, 49, 50, 52, 53, 54, 55, 57, 58, 59, 61, 62,\
	      63, 65, 66, 69, 71, 72, 74, 75, 77, 80, 81, 83,\
	      84, 88, 89, 91, 93, 96, 98, 99, 101, 104, 106,\
	      108, 111, 113, 117, 118, 120, 124, 125, 129,\
	      131, 132, 136, 138, 141, 143, 146, 148, 152,\
	      153, 157, 159, 162, 164, 167, 170, 172, 175,\
	      177, 180, 183, 185, 188, 190, 193, 195, 197,\
	      200, 203, 204, 207, 208, 211, 213, 214, 217,\
	      218, 220, 223, 224, 226, 227, 229, 230, 232,\
	      233, 234, 235, 237, 238, 239, 240, 241, 242,\
	      243, 244, 245, 245, 246, 247, 248, 248, 249,\
	      249, 250, 250, 251, 251, 251, 252, 252, 252,\
	      252, 253, 253, 253, 253, 253, 254, 254, 254,\
	      254, 254, 254, 255, 255, 255, 255, 255, 255,\
	      255, 255, 255, 255, 255, 255, 255, 255, 255,\
	      255, 255, 255, 255, 255, 255, 255, 255, 255,\
	      255, 255, 255, 255, 255, 255, 255, 255, 255,\
	      255, 255, 255,\
}

#endif
