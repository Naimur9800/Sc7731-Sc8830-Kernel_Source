/*
 * File: Altek_isp_drv.c				  *
 * Description: ISP driver			 *
 *					 *
 * (C)Copyright altek Corporation 2014			 *
 *					 *
 * History:				  *
 *   2014/07/17; Bruce Chung; Initial version		 *
*/

/*
 * ////////////////////////////////////////////////
 *		  Include File		  *
 * ///////////////////////////////////////////////
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <asm/segment.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

/* Altek local*/
#include "altek_ahb_drv.h"
#include "altek_isp_drv_local.h"
#include "altek_isp_drv.h"
#include "altek_ap3a_info.h"
#include "altek_os_info.h"
#include "altek_sensor_info.h"
#include "altek_bist.h"
#include "altek_common.h"
#include "altek_ap3a_info_local.h"
#include "altek_iq_info_local.h"
#include "altek_dump_utility.h"
#include "altek_log_local.h"

#include <linux/crc32.h>
#include <asm/cacheflush.h>
#include <video/sprd_mm.h>


/*
 * ////////////////////////////////////////////////////
 *	 Private Constant Definition	  *
 * ///////////////////////////////////////////////////
*/

/* IRQ handling*/
#define ISP_IRQ_MASK	 \
	(V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT | \
	 V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT | \
	 V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT | \
	 V4L2_ISP_SENSOR1_3A_DONE_INT | \
	 V4L2_ISP_SENSOR1_3A_SOF_INT | \
	 V4L2_ISP_SENSOR1_STILL_PROC_START_INT | \
	 V4L2_ISP_SENSOR1_RAW_DONE_INT | \
	 V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT | \
	 V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT | \
	 V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT | \
	 V4L2_ISP_SENSOR2_3A_DONE_INT | \
	 V4L2_ISP_SENSOR2_3A_SOF_INT | \
	 V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT | \
	 V4L2_ISP_SENSOR3_3A_DONE_INT | \
	 V4L2_ISP_SENSOR3_3A_SOF_INT | \
	 V4L2_ISP_SENSOR1_AF_DONE_INT | \
	 V4L2_ISP_SENSOR2_AF_DONE_INT | \
	 V4L2_ISP_SENSOR3_AF_DONE_INT)

/* record the index of raw in ready buffer */
#define IDX_READY_BUF_SENSOR1_RAW (V4L2_ISP_RAW)

struct isp_irq_work {
	uint32_t irq_status;
	struct workqueue_struct *irq_work_queue;
	struct work_struct isq_work;
};
static struct isp_irq_work isr_work;
static struct isp_irq_work isr_work_2nd;
static struct isp_irq_work isr_work_3a;
static struct isp_irq_work isr_work_3a_2nd;
static struct isp_irq_work isr_work_af;
static struct isp_irq_work isr_work_af_2nd;
static struct isp_irq_work isr_work_sof;
static struct isp_irq_work isr_work_sof_2nd;
static struct isp_irq_work isr_work_lv;
static struct isp_irq_work isr_work_lv_2nd;
static struct isp_irq_work isr_work_video;
static struct isp_irq_work isr_work_video_2nd;
static struct isp_irq_work isr_work_still;
static struct isp_irq_work isr_work_still_2nd;
static struct isp_irq_work isr_work_raw;
static struct isp_irq_work isr_work_raw_2nd;
static char irq_work_name[30] = "sprd_isp_irq";
static char irq_work_name_2nd[30] = "sprd_isp_irq_2nd";
static char irq_work_name_3a[30] = "sprd_isp_irq_3a";
static char irq_work_name_3a_2nd[30] = "sprd_isp_irq_3a_2nd";
static char irq_work_name_af[30] = "sprd_isp_irq_af";
static char irq_work_name_af_2nd[30] = "sprd_isp_irq_af_2nd";
static char irq_work_name_sof[30] = "sprd_isp_irq_sof";
static char irq_work_name_sof_2nd[30] = "sprd_isp_irq_sof_2nd";
static char irq_work_name_lv[30] = "sprd_isp_irq_lv";
static char irq_work_name_lv_2nd[30] = "sprd_isp_irq_lv_2nd";
static char irq_work_name_video[30] = "sprd_isp_irq_video";
static char irq_work_name_video_2nd[30] = "sprd_isp_irq_video_2nd";
static char irq_work_name_still[30] = "sprd_isp_irq_still";
static char irq_work_name_still_2nd[30] = "sprd_isp_irq_still_2nd";
static char irq_work_name_raw[30] = "sprd_isp_raw_still";
static char irq_work_name_raw_2nd[30] = "sprd_isp_irq_raw_2nd";

static uint32_t irq_cnt;
static uint32_t irq_3a_cnt;
static uint32_t irq_af_cnt;
static uint32_t irq_sof_cnt;
static uint32_t irq_lv_cnt;
static uint32_t irq_video_cnt;
static uint32_t irq_still_cnt;
static uint32_t irq_raw_cnt;

char *altek_isp_driver_version __attribute__((unused)) =
	"Altek ISP Driver " VERSION;
/*
 * ////////////////////////////////////////////////////
 *	 Private Global Variable		  *
 * ////////////////////////////////////////////////////
*/

/* The start address of firmware for ARC access*/
u32 g_iva_isp_fw_buf_addr;
/* The base address of kernel space.
 * It must be saved at ispdrv_set_firmware_mem_info()
 */
u64 g_kva_isp_fw_buff_addr;

/* The start address of working buffer used by firmware */
static unsigned int g_iva_isp_working_buf_addr;

/* The start address of shared buffer
 * used for commnunication between isp driver and firmware
 */
static unsigned int g_iva_isp_share_buf_addr;

#ifdef DEBUG_LOAD_QMERGE_AND_SHADING_BINARY
/* The buffer for store qmerge and shading bin for debug mode and bist*/
static u32 g_iva_isp_qmerge_buf_addr;
static u32 g_iva_isp_shading_buf_addr;
#endif

static u32 g_iva_u32_addr_isp_extend_buff,
g_iva_u32_addr_isp_high_quality_mode_buff;

/* Isp driver shared memory addr (physical addr offset)*/
static u32 g_iva_firmware_log_addr,
g_iva_manual_data_addr[SENSOR_TOTAL],
g_iva_motion_addr[SENSOR_TOTAL],
/* Scenario Info*/
g_iva_ap_t_scenarioInfo_addr,
/* 3A setting*/
g_iva_ap_t_dld_seq_info_addr[SENSOR_TOTAL],
g_iva_ap_t_cfg_3a_info_addr[SENSOR_TOTAL],
/* Digital room*/
g_iva_ap_t_dzoom_info_addr[SENSOR_TOTAL],
/* Extended buffer*/
g_iva_exbuf_info_addr,
/* Shading*/
g_iva_sensor1_shading_data_addr,
g_iva_sensor2_shading_data_addr, g_iva_sensor3_shading_data_addr,
/* Qmerge*/
g_iva_sensor1_qmerge_data_addr,
g_iva_sensor2_qmerge_data_addr, g_iva_sensor3_qmerge_data_addr,
/* Debug Info*/
g_iva_debug_iq_info_addr[SENSOR_TOTAL],
/* CBC Map */
g_iva_cbc_map_addr[SENSOR_TOTAL];

static u8 *g_kva_share_buf_ptr;
static u8 *g_kva_drv_buff_ptr;

static u32 g_share_buff_size_array[ISP_SHARE_BUFF_INDEX_MAX];
static u32 *g_share_buff_iva_array[ISP_SHARE_BUFF_INDEX_MAX];
static void **g_share_buff_kva_array[ISP_SHARE_BUFF_INDEX_MAX];
static u8 g_share_buff_mask[ISP_SHARE_BUFF_INDEX_MAX];


/* Isp driver working buffer addr (virtual addr)*/
/* Isp manual data addr*/
static struct ispdrv_manual_data *g_kva_manual_ptr[SENSOR_TOTAL];
static struct ispdrv_motion_input *g_kva_motion_ptr[SENSOR_TOTAL];

/* Ptr to scenario info*/
static struct s_scenario_info_isp *g_kva_scenario_info_ptr;

/* Ptr to digital zoom info */
static struct s_scenario_dzoom_info *g_kva_dzoom_info_ptr[SENSOR_TOTAL];

/* Debug info buffer addr for LV, VIDEO, and STILL*/
static struct s_iq_info_addr *g_kva_debug_iq_info_ptr[SENSOR_TOTAL];

/* Tonemap for testing*/
#if 0
static struct ispdrv_tonemap_curve g_tonemap = { _LVHWIRPCTRL_TRIPLE_R_TONE_,
	_LVHWIRPCTRL_TRIPLE_G_TONE_,
	_LVHWIRPCTRL_TRIPLE_B_TONE_
};
#endif

static struct ispdrv_exbuffer_info *g_kva_exbuf_info_ptr;
static u32 g_isp_extend_buf_size, g_isp_high_quality_mode_buf_size;

static DECLARE_WAIT_QUEUE_HEAD(WAITQ);

static struct task_struct *log_task;

/* Firmware address*/
static u32 g_iva_fw_addr;
/* Ahb addr*/
static u64 g_ahb_addr;
/* Ahb size*/
static u32 g_ahb_size;

/* Store the INT reg*/
static u32 g_latest_system_int;

/* The spin lock used to prevent multi core race condition*/
static spinlock_t system_int_lock;

/* Record if the output buffer is ready to get*/
/* 15 = sensor_num * img_type_total,
 *  raw not used in sensor2/3, no img3 & stats in sensor 3
 */
static u8 g_ready_buffer[SENSOR_TOTAL * V4L2_ISP_OUTPUT_IMAGE_TOTAL];

/* Record currrent get output img of ap,
 *  15 = sensor_num * img_type_total,
 *  raw not used in sensor2/3, no img3 & stats in sensor 3
 */
/*  For dequeue,  isp1 / isp2: 4   isp lite:2*/
/* image type total * sensor num(3) */
static u8 g_dequeue_buff_index[SENSOR_TOTAL * V4L2_ISP_OUTPUT_IMAGE_TOTAL];
/*  For enqueue*/
static u8 g_enqueue_buff_index[SENSOR_TOTAL * V4L2_ISP_OUTPUT_IMAGE_TOTAL];
/* Number of buffer for each image output*/
static u8 g_buffer_number[SENSOR_TOTAL][S_OUTPUT_IMAGE_TOTAL];

/* Used to prevent multiple open/close*/
static u8 g_open_count;

/* Error codes for report*/
static u32 errcode;

/* Flag for enabling irq*/
static bool g_enable_irq_flag;

/* Record the path to stop*/
static u32 g_stop_path_field[SENSOR_TOTAL] = { 0, 0, 0 };

/* Current used scenario id*/
static u8 g_uc_sensor_1_scenario_id;
static u8 g_uc_sensor_2_scenario_id;
static u8 g_uc_sensor_3_scenario_id;

/* for v4l2 tool*/
static unsigned int isp_irq;
static bool isp_irq_enable;
static h8_v4l2_irq_func irq_rear_handle;
static h8_v4l2_irq_func irq_front_handle;

/* image type for downloading*/
static unsigned int g_sensor_id_to_download_img;

/* for sync scenario id of
 *  ispdrv_set_scenario_info and ispdrv_set_preview_mode
 */
static u8 g_sensor_1_scenario_id_check;
static u8 g_sensor_2_scenario_id_check;
static u8 g_sensor_3_scenario_id_check;

/* Frame done count*/
static u8 g_fd_cnt_1stsensor;
static u8 g_fd_cnt_2ndsensor;

static int g_log_task_working;
static int g_kva_share_buf_valid;

static bool pull_int2ahb_low_by_ap;

/*Flag to record indicate if the raw output img is streamed on*/
static u8 g_uc_raw_stream_on;

static u16 isp_chip_id = ASIC_CHIP_ID_UNDEFINED;

/* Current used output format*/
static u8 g_ucFormat[SENSOR_TOTAL][PIPE_COUNT] = {
	{V4L2_ISP_OUTPUT_YUY2, V4L2_ISP_OUTPUT_YUY2, V4L2_ISP_OUTPUT_YUY2},
	{V4L2_ISP_OUTPUT_YUY2, V4L2_ISP_OUTPUT_YUY2, V4L2_ISP_OUTPUT_YUY2},
	{V4L2_ISP_OUTPUT_YUY2, V4L2_ISP_OUTPUT_YUY2, V4L2_ISP_OUTPUT_YUY2} };

/* AHB item table for dequeue/euqueue buffer*/
static u32 g_output_field_table[] = {
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR1,
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR2,
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR3,
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR4,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR1,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR2,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR3,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR4,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR1,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR2,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR3,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR4,
	AHB_ITEM_SENSOR1_OUT_STATISTICS_ADDR1,
	AHB_ITEM_SENSOR1_OUT_STATISTICS_ADDR2,
	AHB_ITEM_SENSOR1_OUT_STATISTICS_ADDR3,
	AHB_ITEM_SENSOR1_OUT_STATISTICS_ADDR4,
	AHB_ITEM_SENSOR1_OUT_AF_STATISTICS_ADDR1,
	AHB_ITEM_SENSOR1_OUT_AF_STATISTICS_ADDR2,
	AHB_ITEM_SENSOR1_OUT_AF_STATISTICS_ADDR3,
	AHB_ITEM_SENSOR1_OUT_AF_STATISTICS_ADDR4,
	AHB_ITEM_SENSOR1_OUT_RAW_ADD1,
	AHB_ITEM_SENSOR1_OUT_RAW_ADD2,
	AHB_ITEM_SENSOR1_OUT_RAW_ADD3,
	AHB_ITEM_SENSOR1_OUT_RAW_ADD4,
	AHB_ITEM_SENSOR2_OUT_IMG1_ADDR1,
	AHB_ITEM_SENSOR2_OUT_IMG1_ADDR2,
	AHB_ITEM_SENSOR2_OUT_IMG1_ADDR3,
	AHB_ITEM_SENSOR2_OUT_IMG1_ADDR4,
	AHB_ITEM_SENSOR2_OUT_IMG2_ADDR1,
	AHB_ITEM_SENSOR2_OUT_IMG2_ADDR2,
	AHB_ITEM_SENSOR2_OUT_IMG2_ADDR3,
	AHB_ITEM_SENSOR2_OUT_IMG2_ADDR4,
	AHB_ITEM_SENSOR2_OUT_IMG3_ADDR1,
	AHB_ITEM_SENSOR2_OUT_IMG3_ADDR2,
	AHB_ITEM_SENSOR2_OUT_IMG3_ADDR3,
	AHB_ITEM_SENSOR2_OUT_IMG3_ADDR4,
	AHB_ITEM_SENSOR2_OUT_STATISTICS_ADDR1,
	AHB_ITEM_SENSOR2_OUT_STATISTICS_ADDR2,
	AHB_ITEM_SENSOR2_OUT_STATISTICS_ADDR3,
	AHB_ITEM_SENSOR2_OUT_STATISTICS_ADDR4,
	AHB_ITEM_SENSOR2_OUT_AF_STATISTICS_ADDR1,
	AHB_ITEM_SENSOR2_OUT_AF_STATISTICS_ADDR2,
	AHB_ITEM_SENSOR2_OUT_AF_STATISTICS_ADDR3,
	AHB_ITEM_SENSOR2_OUT_AF_STATISTICS_ADDR4,
	/* No sensor2 raw add */
	0,
	0,
	0,
	0,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR1,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR2,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR3,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR4,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR1,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR2,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR3,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR4,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR1,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR2,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR3,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR4,
	AHB_ITEM_SENSOR3_OUT_STATISTICS_ADDR1,
	AHB_ITEM_SENSOR3_OUT_STATISTICS_ADDR2,
	AHB_ITEM_SENSOR3_OUT_STATISTICS_ADDR3,
	AHB_ITEM_SENSOR3_OUT_STATISTICS_ADDR4,
	AHB_ITEM_SENSOR3_OUT_AF_STATISTICS_ADDR1,
	AHB_ITEM_SENSOR3_OUT_AF_STATISTICS_ADDR2,
	AHB_ITEM_SENSOR3_OUT_AF_STATISTICS_ADDR3,
	AHB_ITEM_SENSOR3_OUT_AF_STATISTICS_ADDR4,
	/* no sensor 3 raw add */
	0,
	0,
	0,
	0
};

/* AHB item table for dequeue/euqueue buffer for isp1 in stripe mode*/
static u32 g_output_field_table_isp1[] = {
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR1_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR2_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR3_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR4_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR1_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR2_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR3_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR4_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR1_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR2_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR3_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR4_ISP1,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

static u32 g_output_field_table_nv12_uv[] = {
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR1_NV12_UV,
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR2_NV12_UV,
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR3_NV12_UV,
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR4_NV12_UV,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR1_NV12_UV,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR2_NV12_UV,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR3_NV12_UV,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR4_NV12_UV,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR1_NV12_UV,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR2_NV12_UV,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR3_NV12_UV,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR4_NV12_UV,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	AHB_ITEM_SENSOR2_OUT_IMG1_ADDR1_NV12_UV,
	AHB_ITEM_SENSOR2_OUT_IMG1_ADDR2_NV12_UV,
	AHB_ITEM_SENSOR2_OUT_IMG1_ADDR3_NV12_UV,
	AHB_ITEM_SENSOR2_OUT_IMG1_ADDR4_NV12_UV,
	AHB_ITEM_SENSOR2_OUT_IMG2_ADDR1_NV12_UV,
	AHB_ITEM_SENSOR2_OUT_IMG2_ADDR2_NV12_UV,
	AHB_ITEM_SENSOR2_OUT_IMG2_ADDR3_NV12_UV,
	AHB_ITEM_SENSOR2_OUT_IMG2_ADDR4_NV12_UV,
	AHB_ITEM_SENSOR2_OUT_IMG3_ADDR1_NV12_UV,
	AHB_ITEM_SENSOR2_OUT_IMG3_ADDR2_NV12_UV,
	AHB_ITEM_SENSOR2_OUT_IMG3_ADDR3_NV12_UV,
	AHB_ITEM_SENSOR2_OUT_IMG3_ADDR4_NV12_UV,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR1_NV12_UV,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR2_NV12_UV,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR3_NV12_UV,
	AHB_ITEM_SENSOR3_OUT_IMG1_ADDR4_NV12_UV,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

/* AHB item table for dequeue/euqueue buffer
 *  for UV of NV12 format for isp1 in stripe mode
 */
static u32 g_output_field_table_nv12_uv_isp1[] = {
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR1_NV12_UV_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR2_NV12_UV_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR3_NV12_UV_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG1_ADDR4_NV12_UV_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR1_NV12_UV_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR2_NV12_UV_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR3_NV12_UV_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG2_ADDR4_NV12_UV_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR1_NV12_UV_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR2_NV12_UV_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR3_NV12_UV_ISP1,
	AHB_ITEM_SENSOR1_OUT_IMG3_ADDR4_NV12_UV_ISP1,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

/* AHB item table for output image index*/
static u32 g_output_img_table[] = {
	AHB_ITEM_SENSOR1_OUT_IMG1_INDEX,
	AHB_ITEM_SENSOR1_OUT_IMG2_INDEX,
	AHB_ITEM_SENSOR1_OUT_IMG3_INDEX,
	AHB_ITEM_SENSOR1_OUT_STATISTICS_INDEX,
	AHB_ITEM_SENSOR1_OUT_AF_STATISTICS_INDEX,
	AHB_ITEM_SENSOR1_OUT_RAW_INDEX,
	AHB_ITEM_SENSOR2_OUT_IMG1_INDEX,
	AHB_ITEM_SENSOR2_OUT_IMG2_INDEX,
	AHB_ITEM_SENSOR2_OUT_IMG3_INDEX,
	AHB_ITEM_SENSOR2_OUT_STATISTICS_INDEX,
	AHB_ITEM_SENSOR2_OUT_AF_STATISTICS_INDEX,
	/* just fill all sensors' raw index with
	 * what sensor1's done to avoid crash
	 */
	AHB_ITEM_SENSOR1_OUT_RAW_INDEX,
	AHB_ITEM_SENSOR3_OUT_IMG1_INDEX,
	AHB_ITEM_SENSOR3_OUT_IMG1_INDEX,
	AHB_ITEM_SENSOR3_OUT_IMG1_INDEX,
	AHB_ITEM_SENSOR3_OUT_STATISTICS_INDEX,
	AHB_ITEM_SENSOR3_OUT_AF_STATISTICS_INDEX,
	AHB_ITEM_SENSOR1_OUT_RAW_INDEX,
};

/*
 * ////////////////////////////////////////////////////
 *	Private Function Prototype		 *
 * ////////////////////////////////////////////////////
*/

/*check irq flag*/
static u32 check_irq_flag(void);
/* ISP log*/
static int log_task_func(void *data);

/* Cmd flow*/
static void turn_on_ready_buffer(int flags);


#ifdef TESTISP
static void test_isp(void);
#endif



/*  Memory allocation*/
static u32 request_ex_buffer(u8 a_uc_scenario_id);
static void release_ex_buffer(void);

/* ISP jtag*/
#ifdef DEBUG_JTAG
static void isp_arc_jtag_cfg(void);
#endif
static void print_scenario_info(struct s_scenario_info_ap *scn_info);
static void print_out_bypass_flag(
		u8 sensor_id, struct s_scinfo_out_bypassflg *bypassflg);

static u8 set_output_format(u8 sensor, u8 img_type, u8 format);
static u32 set_output_buff_raw_format(u8 a_uc_sensor,
		u8 a_uc_output_image,
		u8 a_uc_format,
		u8 a_uc_buffer_number,
		struct output_addr_setting
				*a_pt_output_addr_setting);
static void clear_ahb_int_status(void);
static u8 check_raw_out_valid(u8 sensor_id, u8 out_img_type);
static u8 chk_output_format(u8 img_type, u8 format);

static u8 set_scenario_id_check(u8 scenario_id);
static bool chk_scenario_id(u8 scenario_id);
static void clear_scenario_id_check(u8 sensor_id);
static bool is_preview_stream_on(u8 sensor_id, u8 img_type);

static u32 init_queue_work(void);

static void set_isp_chip_id(u16 chip_id);
static u16 get_isp_chip_id(void);

static u32 allocate_shared_buffer_useage(u32 block_size);

#ifdef DEBUG_LOAD_QMERGE_AND_SHADING_BINARY
static void overwrite_shading_and_qmerge_addr(void);
static u32 overwrite_shading_and_qmerge_bin(void *v4l2_device);
#endif

#ifdef DEBUG_LOAD_FW_BINARY
static u32 overwrite_firmware(void *v4l2_device);
#endif


/*
 * \brief Memory allocation for shared buffer
 * \param share_buff_start_iva [in]. the start address
 *		of the memory of shared buffer
 * \param share_buff_start_kva [in]. the start kernel address
 *		of the memory of shared buffer
 * \param iva_array [in]. Array of pointers
 *		The pointers point to the memory address
 *		of the really used global variables
 * \param kva_array [in]. Array of pointers of kernal addresses
 *		The pointers point to the memory address
 *		of the really used global kernal addresses
 * \param size_array [in]. Array of size of each item in the shared buffer
 * \param share_buff_total_size [in]. the total size of memory of shared buffer
 * \param mask [out]. to check the memory of the index of the shared buffer
 *		is allocated or not.
 * \return Error code
 */
static u32 memory_allocate_share_buff(u32 share_buff_start_iva,
	u8 **share_buff_start_kva,
	u32 *iva_array[ISP_SHARE_BUFF_INDEX_MAX],
	void **kva_array[ISP_SHARE_BUFF_INDEX_MAX],
	u32 *size_array,
	const u32 share_buff_total_size,
	u8 *mask);

/*
 * \brief Allocate physical memory for the specific shared buffer
 * \param start_addr [in]. the start address of the memory of shared buffer
 * \param addr_array [in]. Array of pointers
 *		The pointers point to the memory address
 *		of the really used global variables
 * \param size_array [in]. Array of size of each item in the shared buffer
 * \param index [in]. the index of shared buffer.
 * \param acc_size [out]. count for the total allocated size of memory
 *		to check if over used
 * \param share_buff_total_size [in]. the total size of memory of shared buffer
 * \param mask [out]. to check the memory of the index of the shared buffer
 *		is allocated or not.
 * \return Error code
 */
static u32 allocate_share_buff_iva(u32 start_addr,
	u32 *addr_array[ISP_SHARE_BUFF_INDEX_MAX],
	u32 *size_array,
	u32 index,
	u32 *acc_size,
	const u32 share_buff_total_size,
	u8 *mask);

/*
 * \brief Allocate kernel memory for specific shared buffer
 * \param start_addr [in]. the start physical address
 *		of the memory of shared buffer
 * \param start_kva [in]. the start kernel address
 *		of the memory of shared buffer
 * \param addr_array [in]. Array of pointers
 *		The pointers point to the memory address
 *		of the really used global variables
 * \param kva_array [in]. Array of pointers of kernal addresses
 *		The pointers point to the memory address
 *		of the really used global kernal addresses
 * \param size_array [in]. Array of size of each item in the shared buffer
 * \param index [in]. the index of shared buffer.
 * \return Error code
 */
static void allocate_share_buff_kva(u32 start_iva, u8 **start_kva,
	u32 *addr_array[ISP_SHARE_BUFF_INDEX_MAX],
	void **kva_array[ISP_SHARE_BUFF_INDEX_MAX],
	u32 *size_array, u32 index);

/*
 * \brief Set the address into AHB register
 * \param index [in]. the index of shared buffer.
 * \param addr_array [in]. Array of pointers
 *		The pointers point to the memory address
 *		of the really used global variables
 * \param size_array [in]. Array of size of each item in the shared buffer
 * \param enable [in]. The flag which is used to
 *		indicate to save the firmware log or not
 * \return None
 */
static void set_share_buff_ahb(u32 index,
	u32 *addr_array[],
	u32 *size_array,
	bool enable);
/*
 * \brief Set the initial values to the memory of shared buffer
 * \param index [in]. the index of shared buffer.
 * \param kva_array [in]. Array of pointers of kernal addresses
 *		The pointers point to the memory address
 *		of the really used global kernal addresses
 * \param size_array [in]. Array of size of each item in the shared buffer
 * \return None
 */
static void initial_share_buff_info(u8 index,
	void **kva_array[ISP_SHARE_BUFF_INDEX_MAX],
	u32 *size_array);

/*
 * \brief Allocate the size, memory address of physical global variables,
 *		memory address of kernel global variables
 * \param None
 * \return None
 */
static void init_share_buff_info_array(void);
/*
 * /////////////////////////////////////////////////////
 *		 ap private functions		 *
 * /////////////////////////////////////////////////////
*/
static irqreturn_t isp_irq_handle(int irq, void *priv)
{
	unsigned int flags = 0;
	unsigned int error_code = 0;

	if (g_open_count == 0)
		return IRQ_HANDLED;

	INTDISABLE(isp_irq);

	if (pull_int2ahb_low_by_ap)
		isp_set_reg_value(0xb8, 0);

	/* isp_info("kmd: -- disable isp irq.\n");*/
	flags = check_irq_flag();
	/* turn on the ready buffer flag*/
	turn_on_ready_buffer(flags);

	if (flags & ISP_IRQ_MASK) {
		if (flags &
				(V4L2_ISP_SENSOR1_3A_SOF_INT
				| V4L2_ISP_SENSOR2_3A_SOF_INT
				| V4L2_ISP_SENSOR3_3A_SOF_INT)) {
			if (irq_sof_cnt % 2 == 0) {
				isr_work_sof.irq_status =
						flags &
						(V4L2_ISP_SENSOR1_3A_SOF_INT
						| V4L2_ISP_SENSOR2_3A_SOF_INT
						| V4L2_ISP_SENSOR3_3A_SOF_INT);
				flags &=
						~(V4L2_ISP_SENSOR1_3A_SOF_INT
						| V4L2_ISP_SENSOR2_3A_SOF_INT
						| V4L2_ISP_SENSOR3_3A_SOF_INT);
				error_code =
					queue_work(
						isr_work_sof.irq_work_queue,
						&isr_work_sof.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add sof irq_work_queue fail: 0x%x,"
				, isr_work_sof.irq_status);
			} else {
				isr_work_sof_2nd.irq_status =
						flags &
						(V4L2_ISP_SENSOR1_3A_SOF_INT
						| V4L2_ISP_SENSOR2_3A_SOF_INT
						| V4L2_ISP_SENSOR3_3A_SOF_INT);
				flags &=
						~(V4L2_ISP_SENSOR1_3A_SOF_INT
						| V4L2_ISP_SENSOR2_3A_SOF_INT
						| V4L2_ISP_SENSOR3_3A_SOF_INT);
				error_code =
					queue_work(
						isr_work_sof_2nd.irq_work_queue,
						&isr_work_sof_2nd.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add 2nd sof irq_work_queue fail: 0x%x,"
				, isr_work_sof_2nd.irq_status);
			}
			irq_sof_cnt++;
		}

		if (flags & (V4L2_ISP_SENSOR1_3A_DONE_INT |
				V4L2_ISP_SENSOR2_3A_DONE_INT |
				V4L2_ISP_SENSOR3_3A_DONE_INT)) {
			if (irq_3a_cnt % 2 == 0) {
				isr_work_3a.irq_status =
						flags &
						(V4L2_ISP_SENSOR1_3A_DONE_INT
						| V4L2_ISP_SENSOR2_3A_DONE_INT
						| V4L2_ISP_SENSOR3_3A_DONE_INT);
				flags &=
						~(V4L2_ISP_SENSOR1_3A_DONE_INT
						| V4L2_ISP_SENSOR2_3A_DONE_INT
						| V4L2_ISP_SENSOR3_3A_DONE_INT);
				error_code =
					queue_work(
						isr_work_3a.irq_work_queue,
						&isr_work_3a.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add 3a irq_work_queue fail: 0x%x,"
				, isr_work_3a.irq_status);
			} else {
				isr_work_3a_2nd.irq_status =
						flags &
						(V4L2_ISP_SENSOR1_3A_DONE_INT
						| V4L2_ISP_SENSOR2_3A_DONE_INT
						| V4L2_ISP_SENSOR3_3A_DONE_INT);
				flags &=
						~(V4L2_ISP_SENSOR1_3A_DONE_INT
						| V4L2_ISP_SENSOR2_3A_DONE_INT
						| V4L2_ISP_SENSOR3_3A_DONE_INT);
				error_code =
					queue_work(
						isr_work_3a_2nd.irq_work_queue,
						&isr_work_3a_2nd.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add 2nd 3a irq_work_queue fail: 0x%x,"
				, isr_work_3a_2nd.irq_status);
			}
			irq_3a_cnt++;
		}

		if (flags & (V4L2_ISP_SENSOR1_AF_DONE_INT |
				V4L2_ISP_SENSOR2_AF_DONE_INT |
				V4L2_ISP_SENSOR3_AF_DONE_INT)) {
			if (irq_af_cnt % 2 == 0) {
				isr_work_af.irq_status =
						flags &
						(V4L2_ISP_SENSOR1_AF_DONE_INT
						| V4L2_ISP_SENSOR2_AF_DONE_INT
						| V4L2_ISP_SENSOR3_AF_DONE_INT);
				flags &=
						~(V4L2_ISP_SENSOR1_AF_DONE_INT
						| V4L2_ISP_SENSOR2_3A_DONE_INT
						| V4L2_ISP_SENSOR3_AF_DONE_INT);
				error_code =
					queue_work(
						isr_work_af.irq_work_queue,
						&isr_work_af.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add af irq_work_queue fail: 0x%x,"
				, isr_work_af.irq_status);
			} else {
				isr_work_af_2nd.irq_status =
						flags &
						(V4L2_ISP_SENSOR1_AF_DONE_INT
						| V4L2_ISP_SENSOR2_AF_DONE_INT
						| V4L2_ISP_SENSOR3_AF_DONE_INT);
				flags &=
						~(V4L2_ISP_SENSOR1_AF_DONE_INT
						| V4L2_ISP_SENSOR2_AF_DONE_INT
						| V4L2_ISP_SENSOR3_AF_DONE_INT);
				error_code =
					queue_work(
						isr_work_af_2nd.irq_work_queue,
						&isr_work_af_2nd.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add 2nd af irq_work_queue fail: 0x%x,"
				, isr_work_af_2nd.irq_status);
			}
			irq_af_cnt++;
		}

		if (flags & (V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT |
				V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT |
				V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT)) {
			if (irq_lv_cnt % 2 == 0) {
				isr_work_lv.irq_status =
					flags &
					(V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT
					| V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT
					| V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT);
				flags &=
					~(V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT
					| V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT
					| V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT);
				error_code =
					queue_work(
						isr_work_lv.irq_work_queue,
						&isr_work_lv.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add lv irq_work_queue fail: 0x%x,"
				, isr_work_lv.irq_status);
			} else {
				isr_work_lv_2nd.irq_status =
					flags &
					(V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT
					| V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT
					| V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT);
				flags &=
					~(V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT
					| V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT
					| V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT);
				error_code =
					queue_work(
					isr_work_lv_2nd.irq_work_queue,
					&isr_work_lv_2nd.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add 2nd lv irq_work_queue fail: 0x%x,"
				, isr_work_lv_2nd.irq_status);
			}
			irq_lv_cnt++;
		}

		if (flags & (V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT |
				V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT)) {
			if (irq_video_cnt % 2 == 0) {
				isr_work_video.irq_status =
					flags &
					(V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT
					| V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT);
				flags &=
					~(V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT
					| V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT);
				error_code =
					queue_work(
						isr_work_video.irq_work_queue,
						&isr_work_video.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add video irq_work_queue fail: 0x%x,"
				, isr_work_video.irq_status);
			} else {
				isr_work_video_2nd.irq_status =
					flags &
					(V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT
					| V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT);
				flags &=
					~(V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT
					| V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT);
				error_code =
					queue_work(
					isr_work_video_2nd.irq_work_queue,
					&isr_work_video_2nd.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add 2nd video irq_work_queue fail: 0x%x,"
				, isr_work_video_2nd.irq_status);
			}
			irq_video_cnt++;
		}

		if (flags & (V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT |
				V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT)) {
			if (irq_still_cnt % 2 == 0) {
				isr_work_still.irq_status =
					flags &
					(V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT
					| V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT);
				flags &=
					~(V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT
					| V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT);
				error_code =
					queue_work(
						isr_work_still.irq_work_queue,
						&isr_work_still.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add still irq_work_queue fail: 0x%x,"
				, isr_work_still.irq_status);
			} else {
				isr_work_still_2nd.irq_status =
					flags &
					(V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT
					| V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT);
				flags &=
					~(V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT
					| V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT);
				error_code =
					queue_work(
					isr_work_still_2nd.irq_work_queue,
					&isr_work_still_2nd.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add 2nd still irq_work_queue fail: 0x%x,"
				, isr_work_still_2nd.irq_status);
			}
			irq_still_cnt++;
		}

		if (flags & V4L2_ISP_SENSOR1_RAW_DONE_INT) {
			if (irq_raw_cnt % 2 == 0) {
				isr_work_raw.irq_status =
					flags &
					V4L2_ISP_SENSOR1_RAW_DONE_INT;
				flags &=
					~V4L2_ISP_SENSOR1_RAW_DONE_INT;
				error_code =
					queue_work(
						isr_work_raw.irq_work_queue,
						&isr_work_raw.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add raw irq_work_queue fail: 0x%x,"
				, isr_work_raw.irq_status);
			} else {
				isr_work_raw_2nd.irq_status =
					flags &
					V4L2_ISP_SENSOR1_RAW_DONE_INT;
				flags &=
					~V4L2_ISP_SENSOR1_RAW_DONE_INT;
				error_code =
					queue_work(
					isr_work_raw_2nd.irq_work_queue,
					&isr_work_raw_2nd.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
				"Add 2nd raw irq_work_queue fail: 0x%x,"
				, isr_work_raw_2nd.irq_status);
			}
			irq_raw_cnt++;
		}

		if (flags != 0) {
			if (irq_cnt % 2 == 0) {
				isr_work.irq_status = flags;
				error_code =
					queue_work(
						isr_work.irq_work_queue,
						&isr_work.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
					"Add irq_work_queue fail: 0x%x,"
					, isr_work.irq_status);
			} else {
				isr_work_2nd.irq_status = flags;
				error_code =
					queue_work(
						isr_work_2nd.irq_work_queue,
						&isr_work_2nd.isq_work);
				if (error_code == 0)
					isp_err_item_tag(
					"Add 2nd irq_work_queue fail: 0x%x"
					, isr_work_2nd.irq_status);
			}
			irq_cnt++;
		}
	} else {

	}

	INTENABLE(isp_irq);
	return IRQ_HANDLED;
}

static void irq_handle_work(struct work_struct *work)
{
	u32 flags = 0;
	struct isp_irq_work *isr_work = NULL;

	if (g_open_count == 0)
		return;

	isr_work = container_of(work, struct isp_irq_work, isq_work);
	flags = isr_work->irq_status;

#if 0
	/* Dual sensor*/
	if ((V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT ==
		g_uc_sensor_1_scenario_id) &&
	    ((flags & V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT) ||
	     (flags & V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT) ||
	     (flags & V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT) ||
	     (flags & V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT) ||
	     (flags & V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT) ||
	     (flags & V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT) ||
	     (flags & V4L2_ISP_SENSOR1_3A_DONE_INT) ||
	     (flags & V4L2_ISP_SENSOR2_3A_DONE_INT)) && (irq_rear_handle != 0))
		irq_rear_handle(flags);
	/* Single sensor*/
	else {
		if (((flags & V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT) ||
		     (flags & V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT) ||
		     (flags & V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT) ||
		     (flags & V4L2_ISP_SENSOR1_3A_DONE_INT)) &&
		    (irq_rear_handle != 0))
			irq_rear_handle(flags);
		else if (((flags & V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT) ||
			  (flags & V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT) ||
			  (flags & V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT) ||
			  (flags & V4L2_ISP_SENSOR2_3A_DONE_INT)) &&
			 (irq_rear_handle != 0))
			irq_rear_handle(flags);
	}

	/* Front sensor*/
	if ((V4L2_ISP_SCENARIO_PREVIEW_SENSOR_LITE ==
		g_uc_sensor_3_scenario_id) &&
	    ((flags & V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT) ||
	     (flags & V4L2_ISP_SENSOR3_3A_DONE_INT)) && (irq_rear_handle != 0))
		irq_front_handle(flags);

	/* Verify statistic data*/
	if ((flags & V4L2_ISP_SENSOR1_3A_DONE_INT) ||
	     (flags & V4L2_ISP_SENSOR2_3A_DONE_INT) ||
	     (flags & V4L2_ISP_SENSOR3_3A_DONE_INT))
		;
#endif

	if (flags & (V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT
				| V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT
				| V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT)) {
		if (is_preview_stream_on(SENSOR1, V4L2_ISP_IMG1) == false)
			flags &= ~V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT;
		if (is_preview_stream_on(SENSOR2, V4L2_ISP_IMG1) == false)
			flags &= ~V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT;
		if (is_preview_stream_on(SENSOR3, V4L2_ISP_IMG1) == false)
			flags &= ~V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT;
	}

	if (flags & (V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT
			| V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT)) {
		if (is_preview_stream_on(SENSOR1, V4L2_ISP_IMG2) == false)
			flags &= ~V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT;
		if (is_preview_stream_on(SENSOR2, V4L2_ISP_IMG2) == false)
			flags &= ~V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT;
	}

	if (flags & (V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT
			| V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT)) {
		if (is_preview_stream_on(SENSOR1, V4L2_ISP_IMG3) == false &&
			g_sensor_1_scenario_id_check !=
			V4L2_ISP_SCENARIO_PREVIEW_STILL_SS)
			flags &= ~V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT;
		if (is_preview_stream_on(SENSOR2, V4L2_ISP_IMG3) == false)
			flags &= ~V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT;
	}

	if (flags & (V4L2_ISP_SENSOR1_3A_DONE_INT
				| V4L2_ISP_SENSOR2_3A_DONE_INT
				| V4L2_ISP_SENSOR3_3A_DONE_INT
				| V4L2_ISP_SENSOR1_AF_DONE_INT
				| V4L2_ISP_SENSOR2_AF_DONE_INT
				| V4L2_ISP_SENSOR3_AF_DONE_INT)) {
		if (is_preview_stream_on(SENSOR1, V4L2_ISP_STATITISTICS)
			== false)
			flags &= ~(V4L2_ISP_SENSOR1_3A_DONE_INT|
				V4L2_ISP_SENSOR1_AF_DONE_INT);
		if (is_preview_stream_on(SENSOR2, V4L2_ISP_STATITISTICS)
			== false)
			flags &= ~(V4L2_ISP_SENSOR2_3A_DONE_INT|
				V4L2_ISP_SENSOR2_AF_DONE_INT);
		if (is_preview_stream_on(SENSOR3, V4L2_ISP_STATITISTICS)
			== false)
			flags &= ~(V4L2_ISP_SENSOR3_3A_DONE_INT|
				V4L2_ISP_SENSOR3_AF_DONE_INT);
	}

	if (flags & V4L2_ISP_SENSOR1_RAW_DONE_INT)
		if (is_preview_stream_on(SENSOR1, V4L2_ISP_RAW) == false)
			flags &= ~V4L2_ISP_SENSOR1_RAW_DONE_INT;

	if (flags != 0)
		irq_rear_handle(flags);

}

static void *mmu_lock_isp_buf(unsigned int alloc, unsigned long size)
{
	void *tmp_kva_mmu_lock_ptr = NULL;

	tmp_kva_mmu_lock_ptr = MAP_IVA_TO_KVA(alloc, size);
	if (!tmp_kva_mmu_lock_ptr)
		isp_err_lo_combo_desc_tag("mmu_lock_isp_buf map failed.");

	return tmp_kva_mmu_lock_ptr;
}

static void mmu_unlock_isp_buf(void **kva_unlock_ptr)
{
	isp_info_lo_start_tag();

	if (*kva_unlock_ptr)
		UNMAP_IVA_TO_KVA(*kva_unlock_ptr);

	isp_info_lo_end_tag();
}

/* TODO: remove as init function is applied */
static void reset_dump_info(void)
{
	/* ispdrv_dump_firmware_memory */
	g_iva_fw_addr = 0;
	/* MAP_IVA_TO_KVA, dump_memory */
	g_iva_isp_fw_buf_addr = 0;
	g_kva_isp_fw_buff_addr = 0;
	/* ispdrv_dump_irpbin */
	g_iva_sensor1_qmerge_data_addr = 0;
	g_iva_sensor2_qmerge_data_addr = 0;
	g_iva_sensor3_qmerge_data_addr = 0;
	/* ispdrv_dump_shadingbin */
	g_iva_sensor1_shading_data_addr = 0;
	g_iva_sensor2_shading_data_addr = 0;
	g_iva_sensor3_shading_data_addr = 0;
	/* ispdrv_dump_ahb_reg */
	g_ahb_size = 0;
	g_ahb_addr = 0;
	/* dump_output_img */
	g_sensor_id_to_download_img = 0;
	memset((u8 *) g_buffer_number, 0, sizeof(g_buffer_number));
}

static void clear_isp_resource(void)
{
	isp_info_lo_start_tag();

	errcode = 0;
	/* disable irq callback*/
	g_enable_irq_flag = false;
	/* clear irq indicator */
	isp_irq_enable = false;
	/* disable interrupt */
	g_latest_system_int = 0;
	INTDISABLE(isp_irq);
	free_irq(isp_irq, &isp_irq);
	irq_rear_handle = NULL;
	irq_front_handle = NULL;
	g_open_count = 0;
	log_task = NULL;
	/* unmap base_addr addr (isp_get_reg_value) */
	isp_close();

	if (g_kva_share_buf_ptr) {
		mmu_unlock_isp_buf((void **)&g_kva_share_buf_ptr);
		isp_info_desc_tag("isp: clear isp buf success.");
	}

	isp_info_lo_end_tag();
}

/*
 * /////////////////////////////////////////////////
 *		Public Function		  *
 * ////////////////////////////////////////////////
*/

/*
 *\isp register the callback function of the buttom half of the irq
 *\param p_func[in], the call back function
 *\param type[in], the indicator;
 *	1 : register  the handle of the rear sensor
 *	2 : register  the handle of the front sensor
 *	Ohters:  do nothing
 *\return null
 */
void isp_register_irq_CB_func(h8_v4l2_irq_func p_func, int type)
{
	isp_info_pu_start_tag();

	if (type == 1)
		irq_rear_handle = p_func;
	else if (type == 2)
		irq_front_handle = p_func;
	else
		/* TODO */
		isp_info_desc_tag("invalid type:%d", type);
	isp_info_pu_end_tag();
}
EXPORT_SYMBOL(isp_register_irq_CB_func);

/*
 *\isp request and register an irq number
 *	for the interrupt used among AP and ISP
 *\param irq[in], the irq number
 *\param flags[in], refer to http://www.makelinux.net/ldd3/chp-10-sect-2
 *\param device[in], refer to http://www.makelinux.net/ldd3/chp-10-sect-2
 *\return true if the request is successful
 *	or the irq has been requested successfully before.
 *	Otherwise, flase.
 */
bool init_isp_interrupt(unsigned int irq, unsigned long flags,
			const char *device)
{
	bool ret;

	isp_info_pu_start_tag();

	isp_irq = irq;
	if (!isp_irq_enable) {
		/* isp_irq need to be NULL on altek test platform*/
		if (request_irq(irq, isp_irq_handle, flags, device,
			&isp_irq)) {
			isp_err_pu_combo_desc_tag("request_irq failed.");
			ret = false;
		} else {
			isp_irq_enable = true;
			INTDISABLE(irq);
			ret =  true;
		}
	} else {
		ret = true;
	}

	isp_info_pu_end_tag();

	return ret;
}
EXPORT_SYMBOL(init_isp_interrupt);

/*
 * /////////////////////////////////////////////////
 *	  Boot Sequence apIs		 *
 * /////////////////////////////////////////////////
*/
/*
 *\isp memory configuration, config the firmware memory information,
 *	this function must be called before ispdrv_open()
 *\param iva_fw_addr[in], firmware base address
 *	(address limitation is 8-alignment)
 *\param kva_fw_kaddr[in], kerneal address of firmware
 *\return null
 */
void ispdrv_set_firmware_mem_info(u32 iva_fw_addr, u64 fw_kaddr)
{
	/* -----------------------------------------
	 *   firmware:  5M
	 * -----------------------------------------
	 *   isp working buffer:  17M
	 * -----------------------------------------
	 *   isp driver shared buffer:   1M
	*/

	isp_info_pu_start_tag();

	g_kva_isp_fw_buff_addr = fw_kaddr;
	g_iva_isp_fw_buf_addr = iva_fw_addr;

#ifdef DEBUG_LOAD_QMERGE_AND_SHADING_BINARY
	g_iva_isp_shading_buf_addr = g_iva_isp_fw_buf_addr + BUFF_SIZE_ISP_FW;
	g_iva_isp_qmerge_buf_addr =
		g_iva_isp_shading_buf_addr + BUFF_SIZE_ISP_SHADING;
	g_iva_isp_working_buf_addr =
		g_iva_isp_qmerge_buf_addr + BUFF_SIZE_ISP_QMERGE;
#else
	g_iva_isp_working_buf_addr =
		g_iva_isp_fw_buf_addr + BUFF_SIZE_ISP_FW;
#endif

	g_iva_isp_share_buf_addr =
		g_iva_isp_working_buf_addr + BUFF_SIZE_ISP_WORKING;

	isp_info_item_tag("iva_fw_addr: 0x%x, working_mem: 0x%x, shm: 0x%x,",
		g_iva_isp_fw_buf_addr, g_iva_isp_working_buf_addr,
		g_iva_isp_share_buf_addr);

	isp_info_pu_end_tag();
}
EXPORT_SYMBOL(ispdrv_set_firmware_mem_info);

/*
 *\isp memory configuration, config the raw buffer information,
 *	this function must be called before ispdrv_open()
 *\param iva_raw_buffer_addr[in], the base address of raw buffer
 *\param isp_raw_buf_size[in], Size of raw buffer
 *	(Limitation of the address is 8-alignment)
 *\return null
 */
void ispdrv_set_raw_buffer_mem_info(u32 iva_raw_buffer_addr,
				    u32 isp_raw_buf_size)
{
	isp_info_pu_start_tag();

	g_iva_u32_addr_isp_extend_buff = iva_raw_buffer_addr;
	g_isp_extend_buf_size = isp_raw_buf_size;
	isp_info_item_tag("dram_mem: 0x%x,", g_iva_u32_addr_isp_extend_buff);

	isp_info_pu_end_tag();
}
EXPORT_SYMBOL(ispdrv_set_raw_buffer_mem_info);

/*
 *\isp memory configuration, config the HQ buffer information,
 *	this function must be called before ispdrv_open()
 *\param iva_high_quality_buffer_addr[in],
 *	the base address of high quality buffer
 *	(address limitation is 8-alignment)
 *\param isp_high_quality_buf_size[in], Size of high quality mode buffer.
 *	(Limitation of the address is 8-alignment)
 *\return null
 */
void ispdrv_set_hq_buffer_mem_info(u32 iva_high_quality_buffer_addr,
				   u64 isp_high_quality_buf_size)
{
	isp_info_pu_start_tag();

	g_iva_u32_addr_isp_high_quality_mode_buff =
	    iva_high_quality_buffer_addr;
	g_isp_high_quality_mode_buf_size = isp_high_quality_buf_size;
	isp_info_item_tag("high_iso_mem: 0x%x, size: 0x%x",
		g_iva_u32_addr_isp_high_quality_mode_buff,
		g_isp_high_quality_mode_buf_size);

	isp_info_pu_end_tag();
}
EXPORT_SYMBOL(ispdrv_set_hq_buffer_mem_info);

/*
 *\isp memory configuration, config the shading data information,
 *	this function must be called before ispdrv_open()
 *\param iva_shading_sensor_1_addr[in], Shading address for sensor 1
 *	Address limitation is 8-alignment
 *\param iva_shading_sensor_2_addr[in], Shading address for sensor 2
 *	Address limitation is 8-alignment
 *\param iva_shading_sensor_3_addr[in], Shading address for sensor 3
 *	Address limitation is 8-alignment
 *\return null
 */
void ispdrv_set_shading_addr(u32 iva_shading_sensor_1_addr,
			     u32 iva_shading_sensor_2_addr,
			     u32 iva_shading_sensor_3_addr)
{
	isp_info_pu_start_tag();

	if (iva_shading_sensor_1_addr != 0)
		g_iva_sensor1_shading_data_addr = iva_shading_sensor_1_addr;
	if (iva_shading_sensor_2_addr != 0)
		g_iva_sensor2_shading_data_addr = iva_shading_sensor_2_addr;
	if (iva_shading_sensor_3_addr != 0)
		g_iva_sensor3_shading_data_addr = iva_shading_sensor_3_addr;

	isp_info_item_tag("iva_shading_sensor_1_addr: 0x%x",
		iva_shading_sensor_1_addr);
	isp_info_item_tag("iva_shading_sensor_2_addr: 0x%x",
		iva_shading_sensor_2_addr);
	isp_info_item_tag("iva_shading_sensor_3_addr: 0x%x",
		iva_shading_sensor_3_addr);

	isp_info_pu_end_tag();
}
EXPORT_SYMBOL(ispdrv_set_shading_addr);

/*
 *\isp memory configuration, config the Qmerge data information,
 *	this function must be called before ispdrv_open()
 *\param iva_qmerge_sensor_1_addr[in], Qmerge address for sensor 1
 *	Address limitation is 8-alignment
 *\param iva_qmerge_sensor_2_addr[in], Qmerge address for sensor 2
 *	Address limitation is 8-alignment
 *\param iva_qmerge_sensor_3_addr[in], Qmerge address for sensor 3
 *	Address limitation is 8-alignment
 *\return null
 */
void ispdrv_set_qmerge_addr(u32 iva_qmerge_sensor_1_addr,
			    u32 iva_qmerge_sensor_2_addr,
			    u32 iva_qmerge_sensor_3_addr)
{
	isp_info_pu_start_tag();

	if (iva_qmerge_sensor_1_addr != 0)
		g_iva_sensor1_qmerge_data_addr = iva_qmerge_sensor_1_addr;
	if (iva_qmerge_sensor_2_addr != 0)
		g_iva_sensor2_qmerge_data_addr = iva_qmerge_sensor_2_addr;
	if (iva_qmerge_sensor_3_addr != 0)
		g_iva_sensor3_qmerge_data_addr = iva_qmerge_sensor_3_addr;

	isp_info_item_tag
		("iva_qmerge_sensor_1_addr: 0x%x",
		g_iva_sensor1_qmerge_data_addr);
	isp_info_item_tag
		("iva_qmerge_sensor_2_addr: 0x%x",
		g_iva_sensor2_qmerge_data_addr);
	isp_info_item_tag
		("iva_qmerge_sensor_3_addr: 0x%x",
		g_iva_sensor3_qmerge_data_addr);

	isp_info_pu_end_tag();
}
EXPORT_SYMBOL(ispdrv_set_qmerge_addr);


/*
 *\brief initial all used global variables
 *\param none
 *\return nond
 */
void ispdrv_initialize_global_variable(void)
{
	isp_info_pu_start_tag();
	common_ispdrv_init();
	isp_info_pu_end_tag();
}
EXPORT_SYMBOL(ispdrv_initialize_global_variable);

/*
 *\brief Open ISP driver to init AHB config then start ISP firmware
 *\param a_ud_ahb_addr [in], ahb base address
 *\param a_ud_ahb_size [in], the range of ahb addr for isp
 *\param v4l2_device [in], v4l2 device
 *\return EINVAL(DRAM size is too small),
 *	      PTR_ERR(Creat a thread of log-task fail)
 */
u32 ispdrv_open(u64 a_ud_ahb_addr, u32 a_ud_ahb_size, void *v4l2_device)
{
	u32 err = 0, hw_version, sw_version;
	u16 asic_chip_id;

	isp_info_pu_start_tag();
	isp_info_desc_tag("ISP driver version: " VERSION);
	isp_info_item_tag("a_ud_ahb_addr: 0x%llx, size: 0x%x",
		 a_ud_ahb_addr, a_ud_ahb_size);

	/* set the open flag to prevent multiple open*/
	g_open_count++;
	if (g_open_count > 1)
		goto ispdrv_open_end;


	/*Check null for firmware buffers*/
	if (!g_iva_isp_fw_buf_addr ||
	    !g_iva_isp_working_buf_addr || !g_iva_isp_share_buf_addr) {
		isp_err_pu_combo_desc_tag(
			"fw_buf/inter_buf/share_buf is null.");
		errcode = err = EINVAL;
		goto ispdrv_open_end;
	}

	/*  enable irq */
	INTENABLE(isp_irq);

	/* Initialize queue work */
	err = init_queue_work();

	/* Init spin lock */
	spin_lock_init(&system_int_lock);

	/* TODO: remove reset global variable after init api is called */
	/* reset global variable*/
	errcode = 0;
	g_latest_system_int = 0;
	memset((u8 *) g_ready_buffer, 0, sizeof(g_ready_buffer));
	memset((u8 *) g_dequeue_buff_index, 0, sizeof(g_dequeue_buff_index));
	memset((u8 *) g_enqueue_buff_index, 0, sizeof(g_enqueue_buff_index));
	memset((u8 *) g_buffer_number, 0, sizeof(g_buffer_number));
	memset((u8 *) g_stop_path_field, 0, sizeof(g_stop_path_field));


#ifdef DEBUG_LOAD_FW_BINARY
	err = overwrite_firmware(v4l2_device);
	if (err) {
		isp_err_pu_combo_desc_tag(
			"Overwrite firmware error.");
		errcode = err = EINVAL;
		goto ispdrv_open_end;
	}
#endif

#ifdef DEBUG_LOAD_QMERGE_AND_SHADING_BINARY
	/*For BIST or deubg , overwrite qmerge and shading set by AP */
	overwrite_shading_and_qmerge_addr();
	err = overwrite_shading_and_qmerge_bin(v4l2_device);
	if (err) {
		isp_err_pu_combo_desc_tag(
			"Overwrite shading and qmerge error.");
		errcode = err = EINVAL;
		goto ispdrv_open_end;
	}
#endif

	isp_info_item_tag("fw_addr: 0x%08x, size: %d,",
		g_iva_isp_fw_buf_addr, BUFF_SIZE_ISP_FW);
	isp_info_item_tag("isp_buf_addr: 0x%08x, size: %d,",
		g_iva_isp_working_buf_addr, BUFF_SIZE_ISP_WORKING);
	isp_info_item_tag("shared_buf_addr: 0x%08x, size: %d,",
		g_iva_isp_share_buf_addr, BUFF_SIZE_ISP_SHARE_BUFF);

	/* enable irq callback func.*/
	g_enable_irq_flag = true;

	g_iva_fw_addr = g_iva_isp_fw_buf_addr;
	g_ahb_addr = a_ud_ahb_addr;
	g_ahb_size = a_ud_ahb_size;
	/* Save the parameters for BIST*/
	bist_set_ahb_addr(a_ud_ahb_addr);
	bist_set_ahb_size(a_ud_ahb_size);
	bist_set_v4l2_device(v4l2_device);
	bist_set_firmware_addr(g_kva_isp_fw_buff_addr);

	/* remap ahb bus addr & load the boot code*/
	isp_open(a_ud_ahb_addr, a_ud_ahb_size, NULL);

	/* set fw address and range*/
	/* dram min*/
	isp_set_reg_value(AHB_REG_DRAM_BASE_ADDR, 0x00000000);
	/* dram max*/
	isp_set_reg_value(AHB_REG_DRAM_SIZE_0, 0xffffffff);
	isp_set_reg_value(AHB_REG_DRAM_SIZE_1, g_iva_isp_fw_buf_addr);
	/* remap enabled*/
	isp_set_reg_value(AHB_REG_DRAM_SIZE_2, 0x00000001);
	isp_set_reg_value(AHB_REG_DRAM_SIZE, BUFF_SIZE_ISP_FW);

	/* set buffer addr and size for isp*/
	isp_set_entry_value(AHB_ENTRY_SYS_WORK_BUF_ADDR,
		g_iva_isp_working_buf_addr);
	isp_set_entry_value(AHB_ENTRY_SYS_WORK_BUF_SIZE,
		BUFF_SIZE_ISP_WORKING);

	/* clear interrupt status and enable necessary interrupt*/
	isp_set_entry_value(AHB_ENTRY_INT_SYS_ENABLE, 0x7);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_ENABLE, 0x1FF);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_ENABLE, 0x1FF);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_ENABLE, 0x7F);
	/* clear AHB interruput status*/
	clear_ahb_int_status();

	/* Clear initial value of overlap pixel number */
	isp_set_field_value(AHB_ITEM_OVERLAP_PIXEL_NUM, 0);

	/* Set initial y offset */
	isp_set_field_value(AHB_ITEM_SENSOR1_Y_OFFSET, 0);
	isp_set_field_value(AHB_ITEM_SENSOR2_Y_OFFSET, 0);
	isp_set_field_value(AHB_ITEM_SENSOR3_Y_OFFSET, 0);

	/* Set initial value of out type */
	isp_set_field_value(AHB_ITEM_SENSOR1_OUT_TYPE_IMG1, 2);
	isp_set_field_value(AHB_ITEM_SENSOR1_OUT_TYPE_IMG2, 2);
	isp_set_field_value(AHB_ITEM_SENSOR1_OUT_TYPE_IMG3, 2);
	isp_set_field_value(AHB_ITEM_SENSOR2_OUT_TYPE_IMG1, 2);
	isp_set_field_value(AHB_ITEM_SENSOR2_OUT_TYPE_IMG2, 2);
	isp_set_field_value(AHB_ITEM_SENSOR2_OUT_TYPE_IMG3, 2);
	isp_set_field_value(AHB_ITEM_SENSOR3_OUT_TYPE_IMG1, 2);

	/*  Set address of Shading binary file*/
	isp_set_field_value(AHB_ITEM_SENSOR1_SHADING_ADDR,
			    g_iva_sensor1_shading_data_addr);
	isp_set_field_value(AHB_ITEM_SENSOR2_SHADING_ADDR,
			    g_iva_sensor2_shading_data_addr);
	isp_set_field_value(AHB_ITEM_SENSOR3_SHADING_ADDR,
			    g_iva_sensor3_shading_data_addr);

	/*  Set address of Qmerge binary file*/
	isp_set_field_value(AHB_ITEM_SENSOR1_QMERGE_ADDR,
			    g_iva_sensor1_qmerge_data_addr);
	isp_set_field_value(AHB_ITEM_SENSOR2_QMERGE_ADDR,
			    g_iva_sensor2_qmerge_data_addr);
	isp_set_field_value(AHB_ITEM_SENSOR3_QMERGE_ADDR,
			    g_iva_sensor3_qmerge_data_addr);

	/* Allocate shared buffer for specified useage*/
	allocate_shared_buffer_useage(PAGE_SIZE_4K);

	/* Set the log buffer addr and size and start the log task thread*/
	if (1) /* if (get_isplog_flag() == 1) */ {
		/* set log buffer*/
		/*  isp log buffer addr*/
		isp_set_field_value(AHB_ITEM_LOG_BUF_ADDR,
			g_iva_firmware_log_addr);
		/*  isp log buffer size: 4k*/
		isp_set_field_value(AHB_ITEM_LOG_BUF_SIZE, LOG_BUFFER_SIZE);

		/* log task*/
		log_task = kthread_run(log_task_func, NULL, "isp_log_thread");
		if (IS_ERR(log_task)) {
			err = PTR_ERR(log_task);
			log_task = NULL;
			isp_err_pu_combo_desc_tag(
				"log task failed. It: err: %d", err);
			if (errcode == 0 && err != 0)
				errcode = err;
			goto ispdrv_open_end;
		}

	} else {
		/* isp log buffer addr*/
		isp_set_field_value(AHB_ITEM_LOG_BUF_ADDR, 0);
		/* isp log buffer size*/
		isp_set_field_value(AHB_ITEM_LOG_BUF_SIZE, 0);
	}

	/* Always pull Int2AHB low by AP for boot command */
	pull_int2ahb_low_by_ap = true;

	/* reset then wait sys_boot_finish_int == 1*/
	/* sys_rst_n*/
	isp_set_hw_field_value(AHB_REG_CTRL2ISP, MASK_AHB_REG_FD_SYS_RST_N, 1);
	err = wait_for_irq(__func__, V4L2_ISP_SYS_BOOT_FINISH_INT);

	/*Print the firmware version*/
	hw_version = isp_get_field_value(AHB_ITEM_HW_VERSION);
	sw_version = isp_get_field_value(AHB_ITEM_SW_VERSION);
	isp_info_item_tag("ISP firmware version: v%d.%d,",
		hw_version, sw_version);

	/* Get chip info */
	asic_chip_id = isp_get_field_value(AHB_ITEM_ISP_CHIP_ID);

	/* Filt the version of asic chip from chip info */
	switch (asic_chip_id & 0xF) {
	case ASIC_REVID_GENESIS:
		set_isp_chip_id(ASIC_CHIP_ID_GENESIS);
		break;
	case ASIC_REVID_G2V1:
		set_isp_chip_id(ASIC_CHIP_ID_G2V1);
		break;
	case ASIC_REVID_G2V3:
		set_isp_chip_id(ASIC_CHIP_ID_G2V3);
		break;
	default:
		set_isp_chip_id(ASIC_CHIP_ID_UNDEFINED);
		break;
	}

	/* Determine pull INT2AHB low by ISP or AP*/
	/* iwhale2 platform only support high level trigger so pull low by AP */
	if (get_isp_chip_id() == ASIC_CHIP_ID_G2V3)
		pull_int2ahb_low_by_ap = true;
	else
		pull_int2ahb_low_by_ap = false;

	isp_info_item_tag("Pull int low by AP: %d,",
			pull_int2ahb_low_by_ap);

	SEMAINIT(get_command_semaphore(), 1);

#ifdef DEBUG_JTAG
	isp_arc_jtag_cfg();
#endif

ispdrv_open_end:

	if (err)
		isp_info_err("%d", err);

	if (errcode == 0 && err != 0)
		errcode = err;

	isp_info_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_open);

/*
*\brief close ISP driver
*\return Error code
*/
u32 ispdrv_close(void)
{
	u32 err = 0, err_isp = 0;
	int i;

	isp_info_pu_start_tag();

	/* prevent multiple close*/
	if (g_open_count != 1) {
		if (g_open_count > 1)
			g_open_count--;
		return err;
	}

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto ispdrv_close_end;
	}

	/* disable isp INT*/
	/* isp_set_entry_value(AHB_ENTRY_INT_SYS_ENABLE, 0);*/
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_ENABLE, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_ENABLE, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_ENABLE, 0);
#if 0
	isp_set_entry_value(AHB_ENTRY_INT_SYS_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_IMG1_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_IMG2_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_IMG3_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_3A_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_ISP_SOF_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_3A_LINEMEET_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_IMG1_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_IMG2_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_IMG3_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_3A_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_ISP_SOF_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_3A_LINEMEET_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_IMG_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_3A_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_ISP_SOF_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_3A_LINEMEET_STATUS, 0);
#endif
	clear_ahb_int_status();

	/* invoke all modules to do deinit */
	common_ispdrv_deinit();

	/*  Release ISP working extension buf*/
	/* release_ex_buffer();*/
	/* err = command_start(__func__); */
	err = command_start_poll(__func__);
	if (err)
		isp_info_err("%d", err);

	/*memory barrier*/
	smp_mb();
	mdelay(1);

	isp_set_entry_value(AHB_ENTRY_INT_SYS_ENABLE, 0);

	err_isp = isp_get_field_value(AHB_ITEM_SYSTEM_MESSAGE);
	if (err_isp)
		isp_err_pu_combo_item_tag("isp fw err: 0x%08x", err_isp);

	auto_dump_on_error(err_isp);

	/* check err to avoid dumping twice */
	if (err == 0)
		auto_dump_on_error(err_isp);
	/*  reset ISP*/
	isp_set_hw_field_value(AHB_REG_CTRL2ISP, MASK_AHB_REG_FD_SYS_RST_N, 0);

	if (1) /* if (get_isplog_flag() == 1) */ {
		/*  close the log task*/
		if (log_task)
			kthread_stop(log_task);
		log_task = NULL;

		while (g_log_task_working) {
			isp_debug_desc_tag("log task is running %d\n",
				g_log_task_working);
			SLEEPRANGE(1000, 1500);
		}
	}
#ifdef TESTISP
	test_isp();
#endif

	/*  clear the sw reg*/
	for (i = 0; i < AHB_ENTRY_TOTAL; i++)
		isp_set_entry_value(i, 0);

	SEMAPOST(get_command_semaphore());

ispdrv_close_end:
	if (isr_work.irq_work_queue) {
		destroy_workqueue(isr_work.irq_work_queue);
		isr_work.irq_work_queue = NULL;
	}
	if (isr_work_3a.irq_work_queue) {
		destroy_workqueue(isr_work_3a.irq_work_queue);
		isr_work_3a.irq_work_queue = NULL;
	}
	if (isr_work_af.irq_work_queue) {
		destroy_workqueue(isr_work_af.irq_work_queue);
		isr_work_af.irq_work_queue = NULL;
	}
	if (isr_work_sof.irq_work_queue) {
		destroy_workqueue(isr_work_sof.irq_work_queue);
		isr_work_sof.irq_work_queue = NULL;
	}
	if (isr_work_lv.irq_work_queue) {
		destroy_workqueue(isr_work_lv.irq_work_queue);
		isr_work_lv.irq_work_queue = NULL;
	}
	if (isr_work_video.irq_work_queue) {
		destroy_workqueue(isr_work_video.irq_work_queue);
		isr_work_video.irq_work_queue = NULL;
	}
	if (isr_work_still.irq_work_queue) {
		destroy_workqueue(isr_work_still.irq_work_queue);
		isr_work_still.irq_work_queue = NULL;
	}
	if (isr_work_raw.irq_work_queue) {
		destroy_workqueue(isr_work_raw.irq_work_queue);
		isr_work_raw.irq_work_queue = NULL;
	}
	if (isr_work_2nd.irq_work_queue) {
		destroy_workqueue(isr_work_2nd.irq_work_queue);
		isr_work_2nd.irq_work_queue = NULL;
	}
	if (isr_work_3a_2nd.irq_work_queue) {
		destroy_workqueue(isr_work_3a_2nd.irq_work_queue);
		isr_work_3a_2nd.irq_work_queue = NULL;
	}
	if (isr_work_af_2nd.irq_work_queue) {
		destroy_workqueue(isr_work_af_2nd.irq_work_queue);
		isr_work_af_2nd.irq_work_queue = NULL;
	}
	if (isr_work_sof_2nd.irq_work_queue) {
		destroy_workqueue(isr_work_sof_2nd.irq_work_queue);
		isr_work_sof_2nd.irq_work_queue = NULL;
	}
	if (isr_work_lv_2nd.irq_work_queue) {
		destroy_workqueue(isr_work_lv_2nd.irq_work_queue);
		isr_work_lv_2nd.irq_work_queue = NULL;
	}
	if (isr_work_video_2nd.irq_work_queue) {
		destroy_workqueue(isr_work_video_2nd.irq_work_queue);
		isr_work_video_2nd.irq_work_queue = NULL;
	}
	if (isr_work_still_2nd.irq_work_queue) {
		destroy_workqueue(isr_work_still_2nd.irq_work_queue);
		isr_work_still_2nd.irq_work_queue = NULL;
	}
	if (isr_work_raw_2nd.irq_work_queue) {
		destroy_workqueue(isr_work_raw_2nd.irq_work_queue);
		isr_work_raw_2nd.irq_work_queue = NULL;
	}

	if (err)
		isp_info_err("%d", err);

	if (errcode == 0 && err != 0)
		errcode = err;

	clear_isp_resource();

	isp_info_pu_end_tag();
	return err;
}
EXPORT_SYMBOL(ispdrv_close);

void ispdrv_set_buf_status(int status)
{
	g_kva_share_buf_valid = status;
}
EXPORT_SYMBOL(ispdrv_set_buf_status);

/*
*\brief Set overlap pixel number
*\param a_uw_overlap_pixel_num [In], the number of overlap pixels
*\return error code
*/
u32 ispdrv_set_overlap_pixel(u16 a_uw_overlap_pixel_num)
{
	u32 err = 0;

	isp_info_pu_start_tag();

	isp_info_item_tag("overlap_pixel_num: %x", a_uw_overlap_pixel_num);
	isp_set_field_value(AHB_ITEM_OVERLAP_PIXEL_NUM,
		a_uw_overlap_pixel_num);

	if (err)
		isp_info_err("%d", err);

	if (errcode == 0 && err != 0)
		errcode = err;

	isp_info_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_overlap_pixel);

/*
*\brief Get the chip id of ASIC
*\param chip_id [Out], the chip id
*\return null
*/
void ispdrv_get_isp_chip_id(u16 *chip_id)
{
	isp_debug_pu_start_tag();

	/* Get chip info */
	*chip_id = get_isp_chip_id();

	isp_debug_pu_end_tag();
}
EXPORT_SYMBOL(ispdrv_get_isp_chip_id);

/***********************************************************
*	Mode change / Preview on /Taking picture  apIs	  *
*************************************************************/
/*
*\brief Set ScenarioInfo
*\param a_uc_scenario_id [in], Scenario ID (ISPDRV_SCENARIO_MODE)
*\return EINVAL(Invalid scenario ID)
*/
u32 ispdrv_set_scenario_info(u8 a_uc_scenario_id,
			     struct s_scenario_info_ap *a_pt_sensor1_info,
			     struct s_scenario_info_ap *a_pt_sensor2_info,
			     struct s_scenario_info_ap *a_pt_sensor3_info)
{
	u32 err = 0,
	command_sensor_reg_1 = 0,
	command_system_reg_1 = 0,
	command_sensor_reg_2 = 0, command_system_reg_2 = 0;

	struct s_scenario_info_ap *pt_scenario_info_to_be_set_1 = NULL;
	struct s_scenario_info_ap *pt_scenario_info_to_be_set_2 = NULL;
	u8 index1;
	u8 index2;

	isp_info_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_scenario_info_end;
	}

	isp_info_item_tag("ScenarioID: 0x%x, ", a_uc_scenario_id);


	/* Decide the Sensor info to be set by the scenario id.*/
	switch (a_uc_scenario_id) {
	case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_1:
	case V4L2_ISP_SCENARIO_PREVIEW_STRIPE:
	case V4L2_ISP_SCENARIO_PREVIEW_STILL_SS:
		command_system_reg_1 = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg_1 = AHB_ITEM_CHANGE_SENSOR1_SCENARIOINFO;
		index1 = 0;
		pt_scenario_info_to_be_set_1 = a_pt_sensor1_info;
		print_scenario_info(pt_scenario_info_to_be_set_1);
		break;
	case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_2:
		command_system_reg_1 = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg_1 = AHB_ITEM_CHANGE_SENSOR2_SCENARIOINFO;
		index1 = 1;
		pt_scenario_info_to_be_set_1 = a_pt_sensor2_info;
		print_scenario_info(pt_scenario_info_to_be_set_1);
		break;
	case V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT:
		command_system_reg_1 = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg_1 = AHB_ITEM_CHANGE_SENSOR1_SCENARIOINFO;
		command_system_reg_2 = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg_2 = AHB_ITEM_CHANGE_SENSOR2_SCENARIOINFO;
		index1 = 0;
		pt_scenario_info_to_be_set_1 = a_pt_sensor1_info;
		print_scenario_info(pt_scenario_info_to_be_set_1);
		index2 = 1;
		pt_scenario_info_to_be_set_2 = a_pt_sensor2_info;
		print_scenario_info(pt_scenario_info_to_be_set_2);
		break;
	case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_LITE:
		command_system_reg_1 = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg_1 = AHB_ITEM_CHANGE_SENSOR3_SCENARIOINFO;
		index1 = 2;
		pt_scenario_info_to_be_set_1 = a_pt_sensor3_info;
		print_scenario_info(pt_scenario_info_to_be_set_1);
		break;

	default:
		err = EINVAL;
		goto set_scenario_info_semapost;
	}

	/* Set the scenraio info to the isp scenario info strucutre.*/
	MEMCPY((u8 *) &g_kva_scenario_info_ptr->mode_info[index1],
		(u8 *) &pt_scenario_info_to_be_set_1->sensor_info,
		sizeof(struct s_scinfo_mode_info_isp));
	MEMCPY((u8 *) &g_kva_scenario_info_ptr->bayerscl_out_info[index1],
		(u8 *) &pt_scenario_info_to_be_set_1->bayerscl_out_info,
		sizeof(struct s_scinfo_bayerscl_out_info));
	MEMCPY((u8 *) &g_kva_scenario_info_ptr->out_bypass_flag[index1],
		(u8 *) &pt_scenario_info_to_be_set_1->out_bypass_flag,
		sizeof(struct s_scinfo_out_bypassflg));
	MEMCPY((u8 *) &g_kva_scenario_info_ptr->iq_param_idx_info[index1],
		(u8 *) &pt_scenario_info_to_be_set_1->iq_param_idx_info,
		sizeof(struct s_scinfo_iq_param_idx_info));

	/* If dual rear scenario,
	 *  set the sensor 2 scenraio info to the isp scenario info strucutre
	 */
	if (a_uc_scenario_id == V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT) {
		MEMCPY(
			(u8 *) &g_kva_scenario_info_ptr->mode_info[index2],
			(u8 *) &pt_scenario_info_to_be_set_2->sensor_info,
			sizeof(struct s_scinfo_mode_info_isp));
		MEMCPY(
			(u8 *) &g_kva_scenario_info_ptr->
			bayerscl_out_info[index2],
			(u8 *) &pt_scenario_info_to_be_set_2->bayerscl_out_info,
			sizeof(struct s_scinfo_bayerscl_out_info));
		MEMCPY(
		(u8 *) &g_kva_scenario_info_ptr->out_bypass_flag[index2],
		(u8 *) &pt_scenario_info_to_be_set_2->out_bypass_flag,
		sizeof(struct s_scinfo_out_bypassflg));
	}

	/* Set the command register.*/
	isp_set_field_value(command_system_reg_1, 1);
	isp_set_field_value(command_sensor_reg_1, 1);

	if (a_uc_scenario_id == V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT) {
		isp_set_field_value(command_system_reg_2, 1);
		isp_set_field_value(command_sensor_reg_2, 1);
	}
	/* Opstart*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	/* clear the command register*/
	isp_set_field_value(command_system_reg_1, 0);
	isp_set_field_value(command_sensor_reg_1, 0);

	if (a_uc_scenario_id == V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT) {
		isp_set_field_value(command_system_reg_2, 0);
		isp_set_field_value(command_sensor_reg_2, 0);
	}
	/* Save the scenario id
	 *  check for the following ispdrv_set_preview_mode
	 */
	if (set_scenario_id_check(a_uc_scenario_id) != 0)
		err = EINVAL;

set_scenario_info_semapost:
	SEMAPOST(get_command_semaphore());
set_scenario_info_end:

	if (err)
		isp_info_err("%d", err);

	if (errcode == 0 && err != 0)
		errcode = err;

	isp_info_pu_end_tag();
	return err;
}


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
		struct s_scinfo_out_bypassflg *a_pt_sns3_bypassflg)
{
	u32 err = 0;
	int i;
	struct s_scinfo_out_bypassflg *target;
	struct s_scinfo_out_bypassflg *bypassflg[] = {
		a_pt_sns1_bypassflg,
		a_pt_sns2_bypassflg,
		a_pt_sns3_bypassflg
	};

	isp_info_pu_start_tag();
	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto function_end;
	}
	for (i = SENSOR1; i < SENSOR_TOTAL; i++) {
		target = &g_kva_scenario_info_ptr->out_bypass_flag[i];
		if (bypassflg[i] && target) {
			memcpy(target, bypassflg[i], sizeof(*bypassflg[i]));
			print_out_bypass_flag(i, bypassflg[i]);
		}
	}
function_end:
	SEMAPOST(get_command_semaphore());
	if (err)
		isp_info_err("%d", err);
	isp_info_pu_end_tag();
	return err;
}

/*
 *\brief Set the output buffer format
 *\param a_uc_sensor [in], sensor ID
 *\param a_uc_output_image [in],
 *	       Img1 for Preview,Img2 for video ,
 *	       Img3 for still, refert to ISPDRV_OUTPUT_IMAGE
 *\param a_ucEnable [in], disable: 0,  enable: 1, enable the output to dram
 *                        ignore if a_uc_output_image is raw type
 *\param a_ucFormat[in], output format, refer to ISPDRV_OUTPUT_TYPE
 *                       if a_uc_output_image is raw, two formats could
 *                       be chosen:unpack RAW10/ALTEK RAW10
 *\param a_udWidth  [in], output width
 *                        ignore in raw type
 *\param a_udHeight [in], output height
 *                        ignore in raw type
 *\param a_ucBufferNumber [in], buffer number
 *\param a_pt_output_addr_setting [in], structure of image addresses
 *\param a_uwLineOffset [in], image line offset of Y, should be alinged to 8
 *                            ignore it in raw type
 *\return No error code, always return 0
 */
u32 ispdrv_set_output_buff_format(u8 a_uc_sensor, u8 a_uc_output_image,
				  u8 a_uc_enable, u8 a_uc_format,
				  u32 a_ud_width, u32 a_ud_height,
				  u8 a_uc_buffer_number,
				  struct output_addr_setting *
				  a_pt_output_addr_setting,
				  u16 a_uw_line_offset)
{
	u32 err = 0,
	    command_system_reg = 0,
	    command_sensor_size_reg = 0,
	    command_chg_size_reg = 0,
	    command_sensor_addr_reg = 0, command_chg_addrs_reg = 0;
	/* command_sensor_OnOff_reg = 0,*/
	/* command_ChgOnOff_reg = 0;*/
	bool b_chg_size_enable = true;

	isp_info_pu_start_tag();

	isp_info_item_tag("Sensor: %d, outputImage: %d",
		 a_uc_sensor, a_uc_output_image);

	isp_info_item_tag("W: %d, H: %d Offset: %d",
		 a_ud_width, a_ud_height, a_uw_line_offset);

	isp_info_item_tag("Fmt: %d, En: %d, Num: %d",
		 a_uc_format, a_uc_enable, a_uc_buffer_number);

	isp_info_item_tag("Addr: %X %X %X %X",
		 a_pt_output_addr_setting->ud_addr1
		 , a_pt_output_addr_setting->ud_addr2
		 , a_pt_output_addr_setting->ud_addr3
		 , a_pt_output_addr_setting->ud_addr4);

	if (a_uc_sensor >= SENSOR_TOTAL
	    || a_uc_output_image >= V4L2_ISP_OUTPUT_IMAGE_TOTAL) {
		err = EINVAL;
		goto set_output_buff_format_end;
	}

	if (chk_output_format(a_uc_output_image, a_uc_format)) {
		err = EINVAL;
		goto set_output_buff_format_end;
	}
	if (a_uc_output_image == V4L2_ISP_RAW) {
		err = set_output_buff_raw_format(a_uc_sensor,
				a_uc_output_image,
				a_uc_format,
				a_uc_buffer_number,
				a_pt_output_addr_setting);
		goto set_output_buff_format_end;
	}

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_output_buff_format_end;
	}

	/*  Set output buffer format on for LV, VIDEO, and STILL*/
	if (a_uc_output_image != V4L2_ISP_STATITISTICS &&
		a_uc_output_image != V4L2_ISP_STATITISTICS_OF_AF)
		g_ucFormat[a_uc_sensor][a_uc_output_image] = a_uc_format;

	g_buffer_number[a_uc_sensor][a_uc_output_image] = a_uc_buffer_number;

	/* isp1*/
	if (a_uc_sensor == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_size_reg =
		    AHB_ITEM_CHANGE_SENSOR1_OUT_RESOLUTION;
		command_chg_size_reg =
		    AHB_ITEM_CHANGE_SENSOR1_OUT_RESOLUTION_IMG;
		command_sensor_addr_reg = AHB_ITEM_CHANGE_SENSOR1_OUT_ADDR;
		command_chg_addrs_reg = AHB_ITEM_CHANGE_SENSOR1_OUT_ADDR_IMG;
		/* command_sensor_OnOff_reg =
		 *  AHB_ITEM_CHANGE_SENSOR1_OUT_ON_OFF_SETTING;
		 */
		/* command_ChgOnOff_reg =
		 *  AHB_ITEM_CHAGNE_SENSOR1_OUT_ON_OFF_SETTING_IMG;
		 */

		if (a_uc_output_image == V4L2_ISP_IMG1) {
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_TYPE_IMG1,
					    a_uc_format);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG1_EN,
					    a_uc_enable);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG1_WIDTH,
					    a_ud_width);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG1_HEIGHT,
					    a_ud_height);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG1_BUF_NUM,
					    a_uc_buffer_number);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG1_ADDR1,
					    a_pt_output_addr_setting->ud_addr1);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG1_ADDR2,
					    a_pt_output_addr_setting->ud_addr2);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG1_ADDR3,
					    a_pt_output_addr_setting->ud_addr3);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG1_ADDR4,
					    a_pt_output_addr_setting->ud_addr4);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_ADDR1_ISP1,
			     a_pt_output_addr_setting->ud_addr1_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_ADDR2_ISP1,
			     a_pt_output_addr_setting->ud_addr2_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_ADDR3_ISP1,
			     a_pt_output_addr_setting->ud_addr3_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_ADDR4_ISP1,
			     a_pt_output_addr_setting->ud_addr4_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_ADDR1_NV12_UV,
			     a_pt_output_addr_setting->ud_addr1_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_ADDR2_NV12_UV,
			     a_pt_output_addr_setting->ud_addr2_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_ADDR3_NV12_UV,
			     a_pt_output_addr_setting->ud_addr3_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_ADDR4_NV12_UV,
			     a_pt_output_addr_setting->ud_addr4_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_ADDR1_NV12_UV_ISP1,
			     a_pt_output_addr_setting->ud_addr1_nv12_uv_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_ADDR2_NV12_UV_ISP1,
			     a_pt_output_addr_setting->ud_addr2_nv12_uv_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_ADDR3_NV12_UV_ISP1,
			     a_pt_output_addr_setting->ud_addr3_nv12_uv_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_ADDR4_NV12_UV_ISP1,
			     a_pt_output_addr_setting->ud_addr4_nv12_uv_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_LINE_OFFSET,
			     a_uw_line_offset / 8);
		} else if (a_uc_output_image == V4L2_ISP_IMG2) {
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_TYPE_IMG2,
					    a_uc_format);

			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG2_EN,
					    a_uc_enable);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG2_WIDTH,
					    a_ud_width);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG2_HEIGHT,
					    a_ud_height);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG2_BUF_NUM,
					    a_uc_buffer_number);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG2_ADDR1,
					    a_pt_output_addr_setting->ud_addr1);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG2_ADDR2,
					    a_pt_output_addr_setting->ud_addr2);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG2_ADDR3,
					    a_pt_output_addr_setting->ud_addr3);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG2_ADDR4,
					    a_pt_output_addr_setting->ud_addr4);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_ADDR1_ISP1,
			     a_pt_output_addr_setting->ud_addr1_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_ADDR2_ISP1,
			     a_pt_output_addr_setting->ud_addr2_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_ADDR3_ISP1,
			     a_pt_output_addr_setting->ud_addr3_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_ADDR4_ISP1,
			     a_pt_output_addr_setting->ud_addr4_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_ADDR1_NV12_UV,
			     a_pt_output_addr_setting->ud_addr1_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_ADDR2_NV12_UV,
			     a_pt_output_addr_setting->ud_addr2_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_ADDR3_NV12_UV,
			     a_pt_output_addr_setting->ud_addr3_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_ADDR4_NV12_UV,
			     a_pt_output_addr_setting->ud_addr4_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_ADDR1_NV12_UV_ISP1,
			     a_pt_output_addr_setting->ud_addr1_nv12_uv_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_ADDR2_NV12_UV_ISP1,
			     a_pt_output_addr_setting->ud_addr2_nv12_uv_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_ADDR3_NV12_UV_ISP1,
			     a_pt_output_addr_setting->ud_addr3_nv12_uv_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_ADDR4_NV12_UV_ISP1,
			     a_pt_output_addr_setting->ud_addr4_nv12_uv_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_LINE_OFFSET,
			     a_uw_line_offset / 8);

		} else if (a_uc_output_image == V4L2_ISP_IMG3) {
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_TYPE_IMG3,
					    a_uc_format);
			b_chg_size_enable = false;
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG3_EN,
					    a_uc_enable);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG3_WIDTH,
					    a_ud_width);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG3_HEIGHT,
					    a_ud_height);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG3_BUF_NUM,
					    a_uc_buffer_number);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG3_ADDR1,
					    a_pt_output_addr_setting->ud_addr1);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG3_ADDR2,
					    a_pt_output_addr_setting->ud_addr2);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG3_ADDR3,
					    a_pt_output_addr_setting->ud_addr3);
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG3_ADDR4,
					    a_pt_output_addr_setting->ud_addr4);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_ADDR1_ISP1,
			     a_pt_output_addr_setting->ud_addr1_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_ADDR2_ISP1,
			     a_pt_output_addr_setting->ud_addr2_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_ADDR3_ISP1,
			     a_pt_output_addr_setting->ud_addr3_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_ADDR4_ISP1,
			     a_pt_output_addr_setting->ud_addr4_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_ADDR1_NV12_UV,
			     a_pt_output_addr_setting->ud_addr1_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_ADDR2_NV12_UV,
			     a_pt_output_addr_setting->ud_addr2_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_ADDR3_NV12_UV,
			     a_pt_output_addr_setting->ud_addr3_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_ADDR4_NV12_UV,
			     a_pt_output_addr_setting->ud_addr4_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_ADDR1_NV12_UV_ISP1,
			     a_pt_output_addr_setting->ud_addr1_nv12_uv_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_ADDR2_NV12_UV_ISP1,
			     a_pt_output_addr_setting->ud_addr2_nv12_uv_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_ADDR3_NV12_UV_ISP1,
			     a_pt_output_addr_setting->ud_addr3_nv12_uv_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_ADDR4_NV12_UV_ISP1,
			     a_pt_output_addr_setting->ud_addr4_nv12_uv_isp1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_LINE_OFFSET,
			     a_uw_line_offset / 8);

		} else if (a_uc_output_image == V4L2_ISP_STATITISTICS) {
			b_chg_size_enable = false;
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_STATISTICS_EN,
					    a_uc_enable);
			isp_set_field_value(
				AHB_ITEM_SENSOR1_OUT_STATISTICS_BUF_NUM,
				a_uc_buffer_number);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_STATISTICS_ADDR1,
			     a_pt_output_addr_setting->ud_addr1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_STATISTICS_ADDR2,
			     a_pt_output_addr_setting->ud_addr2);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_STATISTICS_ADDR3,
			     a_pt_output_addr_setting->ud_addr3);
			isp_set_field_value
			    (AHB_ITEM_SENSOR1_OUT_STATISTICS_ADDR4,
			     a_pt_output_addr_setting->ud_addr4);

		} else if (a_uc_output_image == V4L2_ISP_STATITISTICS_OF_AF) {
			b_chg_size_enable = false;
			/*
			* enable and number are the same
			* as V4L2_ISP_STATITISTICS
			*/
			isp_set_field_value(
				AHB_ITEM_SENSOR1_OUT_AF_STATISTICS_ADDR1,
				a_pt_output_addr_setting->ud_addr1);
			isp_set_field_value(
				AHB_ITEM_SENSOR1_OUT_AF_STATISTICS_ADDR2,
				a_pt_output_addr_setting->ud_addr2);
			isp_set_field_value(
				AHB_ITEM_SENSOR1_OUT_AF_STATISTICS_ADDR3,
				a_pt_output_addr_setting->ud_addr3);
			isp_set_field_value(
				AHB_ITEM_SENSOR1_OUT_AF_STATISTICS_ADDR4,
				a_pt_output_addr_setting->ud_addr4);
		}
	} else if (a_uc_sensor == SENSOR2) {	/* isp2*/
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_size_reg =
		    AHB_ITEM_CHANGE_SENSOR2_OUT_RESOLUTION;
		command_chg_size_reg =
		    AHB_ITEM_CHANGE_SENSOR2_OUT_RESOLUTION_IMG;
		command_sensor_addr_reg = AHB_ITEM_CHANGE_SENSOR2_OUT_ADDR;
		command_chg_addrs_reg = AHB_ITEM_CHANGE_SENSOR2_OUT_ADDR_IMG;
		/* command_sensor_OnOff_reg =
		 *  AHB_ITEM_CHANGE_SENSOR2_OUT_ON_OFF_SETTING;
		 */
		/* command_ChgOnOff_reg =
		 *  AHB_ITEM_CHAGNE_SENSOR2_OUT_ON_OFF_SETTING_IMG;
		 */

		if (a_uc_output_image == V4L2_ISP_IMG1) {
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_TYPE_IMG1,
					    a_uc_format);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG1_EN,
					    a_uc_enable);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG1_WIDTH,
					    a_ud_width);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG1_HEIGHT,
					    a_ud_height);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG1_BUF_NUM,
					    a_uc_buffer_number);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG1_ADDR1,
					    a_pt_output_addr_setting->ud_addr1);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG1_ADDR2,
					    a_pt_output_addr_setting->ud_addr2);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG1_ADDR3,
					    a_pt_output_addr_setting->ud_addr3);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG1_ADDR4,
					    a_pt_output_addr_setting->ud_addr4);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG1_ADDR1_NV12_UV,
			     a_pt_output_addr_setting->ud_addr1_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG1_ADDR2_NV12_UV,
			     a_pt_output_addr_setting->ud_addr2_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG1_ADDR3_NV12_UV,
			     a_pt_output_addr_setting->ud_addr3_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG1_ADDR4_NV12_UV,
			     a_pt_output_addr_setting->ud_addr4_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG1_LINE_OFFSET,
			     a_uw_line_offset / 8);

		} else if (a_uc_output_image == V4L2_ISP_IMG2) {
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_TYPE_IMG2,
					    a_uc_format);

			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG2_EN,
					    a_uc_enable);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG2_WIDTH,
					    a_ud_width);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG2_HEIGHT,
					    a_ud_height);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG2_BUF_NUM,
					    a_uc_buffer_number);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG2_ADDR1,
					    a_pt_output_addr_setting->ud_addr1);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG2_ADDR2,
					    a_pt_output_addr_setting->ud_addr2);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG2_ADDR3,
					    a_pt_output_addr_setting->ud_addr3);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG2_ADDR4,
					    a_pt_output_addr_setting->ud_addr4);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG2_ADDR1_NV12_UV,
			     a_pt_output_addr_setting->ud_addr1_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG2_ADDR2_NV12_UV,
			     a_pt_output_addr_setting->ud_addr2_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG2_ADDR3_NV12_UV,
			     a_pt_output_addr_setting->ud_addr3_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG2_ADDR4_NV12_UV,
			     a_pt_output_addr_setting->ud_addr4_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG2_LINE_OFFSET,
			     a_uw_line_offset / 8);

		} else if (a_uc_output_image == V4L2_ISP_IMG3) {
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_TYPE_IMG3,
					    a_uc_format);

			b_chg_size_enable = false;
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG3_EN,
					    a_uc_enable);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG3_WIDTH,
					    a_ud_width);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG3_HEIGHT,
					    a_ud_height);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG3_BUF_NUM,
					    a_uc_buffer_number);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG3_ADDR1,
					    a_pt_output_addr_setting->ud_addr1);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG3_ADDR2,
					    a_pt_output_addr_setting->ud_addr2);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG3_ADDR3,
					    a_pt_output_addr_setting->ud_addr3);
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG3_ADDR4,
					    a_pt_output_addr_setting->ud_addr4);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG3_ADDR1_NV12_UV,
			     a_pt_output_addr_setting->ud_addr1_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG3_ADDR2_NV12_UV,
			     a_pt_output_addr_setting->ud_addr2_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG3_ADDR3_NV12_UV,
			     a_pt_output_addr_setting->ud_addr3_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG3_ADDR4_NV12_UV,
			     a_pt_output_addr_setting->ud_addr4_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG3_LINE_OFFSET,
			     a_uw_line_offset / 8);

		} else if (a_uc_output_image == V4L2_ISP_STATITISTICS) {
			b_chg_size_enable = false;
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_STATISTICS_EN,
					    a_uc_enable);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_STATISTICS_BUF_NUM,
			     a_uc_buffer_number);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_STATISTICS_ADDR1,
			     a_pt_output_addr_setting->ud_addr1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_STATISTICS_ADDR2,
			     a_pt_output_addr_setting->ud_addr2);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_STATISTICS_ADDR3,
			     a_pt_output_addr_setting->ud_addr3);
			isp_set_field_value
			    (AHB_ITEM_SENSOR2_OUT_STATISTICS_ADDR4,
			     a_pt_output_addr_setting->ud_addr4);
		} else if (a_uc_output_image == V4L2_ISP_STATITISTICS_OF_AF) {
			b_chg_size_enable = false;

			/*
			* enable and number are the same
			* as V4L2_ISP_STATITISTICS
			*/
			isp_set_field_value(
				AHB_ITEM_SENSOR2_OUT_AF_STATISTICS_ADDR1,
				a_pt_output_addr_setting->ud_addr1);
			isp_set_field_value(
				AHB_ITEM_SENSOR2_OUT_AF_STATISTICS_ADDR2,
				a_pt_output_addr_setting->ud_addr2);
			isp_set_field_value(
				AHB_ITEM_SENSOR2_OUT_AF_STATISTICS_ADDR3,
				a_pt_output_addr_setting->ud_addr3);
			isp_set_field_value(
				AHB_ITEM_SENSOR2_OUT_AF_STATISTICS_ADDR4,
				a_pt_output_addr_setting->ud_addr4);
		}
	} else {	/* isp lite*/
		b_chg_size_enable = false;

		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		/* command_sensor_Size_reg =
		 *  AHB_ITEM_CHANGE_SENSOR3_OUT_RESOLUTION;
		 */
		/* command_ChgSize_reg =
		 *  AHB_ITEM_CHANGE_SENSOR3_OUT_RESOLUTION_IMG;
		 */
		command_sensor_addr_reg = AHB_ITEM_CHANGE_SENSOR3_OUT_ADDR;
		command_chg_addrs_reg = AHB_ITEM_CHANGE_SENSOR3_OUT_ADDR_IMG;
		/* command_sensor_OnOff_reg =
		 *  AHB_ITEM_CHANGE_SENSOR3_OUT_ON_OFF_SETTING;
		 */
		/* command_ChgOnOff_reg =
		 *  AHB_ITEM_CHAGNE_SENSOR3_OUT_ON_OFF_SETTING_IMG;
		 */

		if (a_uc_output_image == V4L2_ISP_IMG1) {
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_IMG1_EN,
					    a_uc_enable);
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_TYPE_IMG1,
					    a_uc_format);
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_IMG1_WIDTH,
					    a_ud_width);
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_IMG1_HEIGHT,
					    a_ud_height);
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_IMG1_BUF_NUM,
					    a_uc_buffer_number);
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_IMG1_ADDR1,
					    a_pt_output_addr_setting->ud_addr1);
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_IMG1_ADDR2,
					    a_pt_output_addr_setting->ud_addr2);
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_IMG1_ADDR3,
					    a_pt_output_addr_setting->ud_addr3);
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_IMG1_ADDR4,
					    a_pt_output_addr_setting->ud_addr4);
			isp_set_field_value
			    (AHB_ITEM_SENSOR3_OUT_IMG1_ADDR1_NV12_UV,
			     a_pt_output_addr_setting->ud_addr1_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR3_OUT_IMG1_ADDR2_NV12_UV,
			     a_pt_output_addr_setting->ud_addr2_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR3_OUT_IMG1_ADDR3_NV12_UV,
			     a_pt_output_addr_setting->ud_addr3_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR3_OUT_IMG1_ADDR4_NV12_UV,
			     a_pt_output_addr_setting->ud_addr4_nv12_uv);
			isp_set_field_value
			    (AHB_ITEM_SENSOR3_OUT_IMG1_LINE_OFFSET,
			     a_uw_line_offset / 8);
		} else if (a_uc_output_image == V4L2_ISP_STATITISTICS) {
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_STATISTICS_EN,
					    a_uc_enable);
			isp_set_field_value
			    (AHB_ITEM_SENSOR3_OUT_STATISTICS_BUF_NUM,
			     a_uc_buffer_number);
			isp_set_field_value
			    (AHB_ITEM_SENSOR3_OUT_STATISTICS_ADDR1,
			     a_pt_output_addr_setting->ud_addr1);
			isp_set_field_value
			    (AHB_ITEM_SENSOR3_OUT_STATISTICS_ADDR2,
			     a_pt_output_addr_setting->ud_addr2);
			isp_set_field_value
			    (AHB_ITEM_SENSOR3_OUT_STATISTICS_ADDR3,
			     a_pt_output_addr_setting->ud_addr3);
			isp_set_field_value
			    (AHB_ITEM_SENSOR3_OUT_STATISTICS_ADDR4,
			     a_pt_output_addr_setting->ud_addr4);
		} else if (a_uc_output_image == V4L2_ISP_STATITISTICS_OF_AF) {
			b_chg_size_enable = false;
			/*
			* enable and number are the same
			* as V4L2_ISP_STATITISTICS
			*/
			isp_set_field_value(
				AHB_ITEM_SENSOR3_OUT_AF_STATISTICS_ADDR1,
				a_pt_output_addr_setting->ud_addr1);
			isp_set_field_value(
				AHB_ITEM_SENSOR3_OUT_AF_STATISTICS_ADDR2,
				a_pt_output_addr_setting->ud_addr2);
			isp_set_field_value(
				AHB_ITEM_SENSOR3_OUT_AF_STATISTICS_ADDR3,
				a_pt_output_addr_setting->ud_addr3);
			isp_set_field_value(
				AHB_ITEM_SENSOR3_OUT_AF_STATISTICS_ADDR4,
				a_pt_output_addr_setting->ud_addr4);
		}
	}

	isp_set_field_value(command_system_reg, 1);

	if (true == b_chg_size_enable) {
		isp_set_field_value(command_sensor_size_reg, 1);
		isp_set_field_value(command_chg_size_reg, a_uc_output_image);
	}

	isp_set_field_value(command_sensor_addr_reg, 1);
	isp_set_field_value(command_chg_addrs_reg, a_uc_output_image);

	/* if(a_uc_output_image != V4L2_ISP_STATITISTICS)*/
	/* {*/
	/*  isp_set_field_value(command_sensor_OnOff_reg, 1);*/
	/*  isp_set_field_value(command_ChgOnOff_reg, a_uc_output_image);*/
	/* }*/

	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/

	isp_set_field_value(command_system_reg, 0);

	if (true == b_chg_size_enable) {
		isp_set_field_value(command_sensor_size_reg, 0);
		isp_set_field_value(command_chg_size_reg, 0);
	}

	isp_set_field_value(command_sensor_addr_reg, 0);
	isp_set_field_value(command_chg_addrs_reg, 0);
	/* isp_set_field_value(command_sensor_OnOff_reg, 0);*/
	/* isp_set_field_value(command_ChgOnOff_reg, 0);*/

	SEMAPOST(get_command_semaphore());

set_output_buff_format_end:

	if (err)
		isp_info_err("%d", err);

	if (errcode == 0 && err != 0)
		errcode = err;

	isp_info_pu_end_tag();
	return err;
}
EXPORT_SYMBOL(ispdrv_set_output_buff_format);


/*
*\brief Set Preview Mode (for ZSL mode)
*\param a_uc_scenario_id [in],
*	       scenario ID (ISPDRV_SCENARIO_MODE).
*	       It must be the same as
*	       that was set by ispdrv_set_scenario_info previously.
*\return EINVAL(Invalid scenario ID)
*\bug Under construct
*/
u32 ispdrv_set_preview_mode(u8 a_uc_scenario_id)
{
	u32 err = 0;

	isp_info_pu_start_tag();
	isp_info_item_tag("Scenario ID: %d", a_uc_scenario_id);

	if (a_uc_scenario_id >= V4L2_ISP_SCENARIO_TOTAL
		|| chk_scenario_id(a_uc_scenario_id) != true) {
		err = EINVAL;
		goto set_preview_mode_end;
	}

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_preview_mode_end;
	}

	/* reset path mode*/
	isp_set_field_value(AHB_ITEM_SENSOR1_PATH_MODE, 0);
	isp_set_field_value(AHB_ITEM_SENSOR2_PATH_MODE, 0);

	/*  Release ISP working extension buf*/
	release_ex_buffer();

	/*  Set address of Shading binary file*/
	if (g_stop_path_field[SENSOR1] == 0)
		isp_set_field_value(AHB_ITEM_SENSOR1_SHADING_ADDR,
					g_iva_sensor1_shading_data_addr);
	if (g_stop_path_field[SENSOR2] == 0)
		isp_set_field_value(AHB_ITEM_SENSOR2_SHADING_ADDR,
					g_iva_sensor2_shading_data_addr);
	if (g_stop_path_field[SENSOR3] == 0)
		isp_set_field_value(AHB_ITEM_SENSOR3_SHADING_ADDR,
					g_iva_sensor3_shading_data_addr);

	/*  Set address of Qmerge binary file*/
	if (g_stop_path_field[SENSOR1] == 0)
		isp_set_field_value(AHB_ITEM_SENSOR1_QMERGE_ADDR,
					g_iva_sensor1_qmerge_data_addr);
	if (g_stop_path_field[SENSOR2] == 0)
		isp_set_field_value(AHB_ITEM_SENSOR2_QMERGE_ADDR,
					g_iva_sensor2_qmerge_data_addr);
	if (g_stop_path_field[SENSOR3] == 0)
		isp_set_field_value(AHB_ITEM_SENSOR3_QMERGE_ADDR,
					g_iva_sensor3_qmerge_data_addr);

	switch (a_uc_scenario_id) {
	case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_1:
		g_uc_sensor_1_scenario_id = a_uc_scenario_id;
		isp_set_field_value(AHB_ITEM_START_SENSOR1_ZSL_PATH, 1);
		/*  Direct frame mode*/
		isp_set_field_value(AHB_ITEM_SENSOR1_PATH_MODE, 0);
		if (g_ucFormat[SENSOR1][V4L2_ISP_IMG1] == V4L2_ISP_OUTPUT_RAW10)
			/* dram frame mode*/
			isp_set_field_value(AHB_ITEM_SENSOR1_PATH_MODE, 1);
		err = command_start(__func__);
		/* err = command_start_poll(__func__);*/
		isp_set_field_value(AHB_ITEM_START_SENSOR1_ZSL_PATH, 0);
		if (err)
			goto set_preview_mode_semapost;
		g_sensor_id_to_download_img = SENSOR1;
		g_stop_path_field[SENSOR1] = AHB_ITEM_STOP_SENSOR1_PATH;
		break;
	case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_2:
		g_uc_sensor_2_scenario_id = a_uc_scenario_id;
		isp_set_field_value(AHB_ITEM_START_SENSOR2_ZSL_PATH, 1);
		/*  Direct frame mode*/
		isp_set_field_value(AHB_ITEM_SENSOR2_PATH_MODE, 0);
		if (g_ucFormat[SENSOR2][V4L2_ISP_IMG1] == V4L2_ISP_OUTPUT_RAW10)
			/* dram frame mode*/
			isp_set_field_value(AHB_ITEM_SENSOR2_PATH_MODE, 1);
		err = command_start(__func__);
		/* err = command_start_poll(__func__);*/
		isp_set_field_value(AHB_ITEM_START_SENSOR2_ZSL_PATH, 0);
		if (err)
			goto set_preview_mode_semapost;
		g_sensor_id_to_download_img = SENSOR1;
		g_stop_path_field[SENSOR2] = AHB_ITEM_STOP_SENSOR2_PATH;
		break;
	case V4L2_ISP_SCENARIO_PREVIEW_STRIPE:
		g_uc_sensor_1_scenario_id = a_uc_scenario_id;
		isp_set_field_value(AHB_ITEM_START_SENSOR1_ZSL_PATH, 1);
		/* Direct stripe mode*/
		isp_set_field_value(AHB_ITEM_SENSOR1_PATH_MODE, 2);
		err = command_start(__func__);
		/* err = command_start_poll(__func__);*/
		isp_set_field_value(AHB_ITEM_START_SENSOR1_ZSL_PATH, 0);
		if (err)
			goto set_preview_mode_semapost;

		g_sensor_id_to_download_img = SENSOR1;
		g_stop_path_field[SENSOR1] = AHB_ITEM_STOP_SENSOR1_PATH;
		break;
	case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_LITE:
		g_uc_sensor_3_scenario_id = a_uc_scenario_id;
		isp_set_field_value(AHB_ITEM_START_SENSOR3_ZSL_PATH, 1);
		err = command_start(__func__);
		/* err = command_start_poll(__func__);*/
		isp_set_field_value(AHB_ITEM_START_SENSOR3_ZSL_PATH, 0);
		if (err)
			goto set_preview_mode_semapost;

		g_sensor_id_to_download_img = SENSOR3;
		g_stop_path_field[SENSOR3] = AHB_ITEM_STOP_SENSOR3_PATH;
		break;
	case V4L2_ISP_SCENARIO_PREVIEW_STILL_SS:
		g_uc_sensor_1_scenario_id = a_uc_scenario_id;
		/*  Request DRAM bufs*/
		err = request_ex_buffer(a_uc_scenario_id);
		if (err)
			goto set_preview_mode_semapost;
		isp_set_field_value(AHB_ITEM_START_SENSOR1_ZSL_PATH, 1);
		isp_set_field_value(AHB_ITEM_SENSOR1_PATH_MODE,
				ISP_S1_PATH_MODE_SS);
		err = command_start(__func__);
		/* err = command_start_poll(__func__);*/
		isp_set_field_value(AHB_ITEM_START_SENSOR1_ZSL_PATH, 0);
		if (err)
			goto set_preview_mode_semapost;

		g_sensor_id_to_download_img = SENSOR1;
		g_stop_path_field[SENSOR1] = AHB_ITEM_STOP_SENSOR1_PATH;
		break;
	default:
		break;
	}

	if (E_SELFTEST_BIST ==
	    bist_get_self_test_mode(g_kva_scenario_info_ptr))
		goto set_preview_mode_semapost;

#ifdef SP_FW_V_1_TEST
#ifdef ENABLE_TEST_PATTERN_SENSOR_1
	/*  Switching pattern;*/

	set_ahb_indirect(0xfffa4004, 16);

	/*  Image width;*/

	set_ahb_indirect(0xfffa4008, 1920);

	/* Image height*/

	set_ahb_indirect(0xfffa400c, 1080);

	/* H blanking*/

	set_ahb_indirect(0xfffa4010, 13440);

	/* V blanking*/

	set_ahb_indirect(0xfffa4014, 100);

	set_ahb_indirect(0xfffa4018, 0x10);

	/* Release soft reset*/

	set_ahb_indirect(0xfffa4000, 0x2);
#endif

#ifdef ENABLE_TEST_PATTERN_SENSOR_2
	/*  Switching pattern;*/

	set_ahb_indirect(0xfffa5004, 16);

	/*  Image width;*/

	set_ahb_indirect(0xfffa5008, 6792);

	/*      Image height*/

	set_ahb_indirect(0xfffa500c, 3816);

	/*  H blanking*/

	set_ahb_indirect(0xfffa5010, 13440);

	/*  V blanking*/

	set_ahb_indirect(0xfffa5014, 100);

	set_ahb_indirect(0xfffa5018, 0x10);

	/*  Release soft reset*/

	set_ahb_indirect(0xfffa5000, 0x2);
#endif

#ifdef ENABLE_TEST_PATTERN_SENSOR_LITE
	/*  Switching pattern;*/

	set_ahb_indirect(0xfffa3004, 16);

	/*  Image width;*/

	set_ahb_indirect(0xfffa3008, 1368);

	/*      Image height*/

	set_ahb_indirect(0xfffa300c, 960);

	/*  H blanking*/

	set_ahb_indirect(0xfffa3010, 13440);

	/*  V blanking*/

	set_ahb_indirect(0xfffa3014, 100);

	set_ahb_indirect(0xfffa3018, 0x10);

	/*  Release soft reset*/

	set_ahb_indirect(0xfffa3000, 0x2);
#endif
#endif

set_preview_mode_semapost:
	SEMAPOST(get_command_semaphore());
set_preview_mode_end:

	if (err)
		isp_info_err("%d", err);

	if (errcode == 0 && err != 0)
		errcode = err;

	isp_info_pu_end_tag();
	return err;
}
EXPORT_SYMBOL(ispdrv_set_preview_mode);

/*
*\brief stop stream path
*\param a_uc_sensor [in], sensor ID
*\return error code
*/
u32 ispdrv_stop_preview_mode(u8 a_uc_sensor)
{
	u32 err = 0;

	isp_info_pu_start_tag();

	if (a_uc_sensor >= SENSOR_TOTAL) {
		err = EINVAL;
		goto stop_preview_mode_end;
	}

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto stop_preview_mode_end;
	}

	isp_info_item_tag("Sensor: %d", a_uc_sensor);

	/* stop paths*/
	if (g_stop_path_field[a_uc_sensor])
		isp_set_field_value(g_stop_path_field[a_uc_sensor], 1);

	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	/* Clear state*/
	if (g_stop_path_field[a_uc_sensor]) {
		isp_set_field_value(g_stop_path_field[a_uc_sensor], 0);
		g_stop_path_field[a_uc_sensor] = 0;
	}

	/* clear the buffer ready state of specific sensor*/
	memset((u8 *) &
	       g_ready_buffer[a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL], 0,
	       sizeof(u8) * V4L2_ISP_OUTPUT_IMAGE_TOTAL);

	SEMAPOST(get_command_semaphore());

stop_preview_mode_end:

	if (err)
		isp_info_err("%d", err);

	if (errcode == 0 && err != 0)
		errcode = err;

	/* Reset the check;*/
	clear_scenario_id_check(a_uc_sensor);

	/*  Clear secnraio ID*/
	if (err == 0) {
		if (a_uc_sensor == SENSOR1)
			g_uc_sensor_1_scenario_id = 0;
		else if (a_uc_sensor == SENSOR2)
			g_uc_sensor_2_scenario_id = 0;
		else
			g_uc_sensor_3_scenario_id = 0;
	}

	isp_info_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_stop_preview_mode);

/*
 *\brief Preview stream on for sensor1/sensor2/sensor3 (1: on , 0: ignore)
 *	Enable sensor status INT, Set op start
 *\param a_uc_sensor [in], sensor ID
 *\param a_uc_output_image [in],
 *	       Img1 for Preview,Img2 for video , Img3 for still,
 *	       refert to ISPDRV_OUTPUT_IMAGE
 *\return ETIMEDOUT.(Without AHB command finish INT)
 */
u32 ispdrv_preview_stream_on(u8 a_uc_sensor, u8 a_uc_output_image)
{
	u32 err = 0,
	    command_sensor_reg = 0,
	    command_system_reg = 0, command_img_on_off_reg = 0;

	isp_info_pu_start_tag();

	if (a_uc_sensor >= SENSOR_TOTAL ||
		a_uc_output_image >= V4L2_ISP_OUTPUT_IMAGE_TOTAL) {
		err = EINVAL;
		goto preview_stream_on_end;
	}

	/* Statistic of AF stream is configured with V4L2_ISP_STATITISTICS */
	if (a_uc_output_image == V4L2_ISP_STATITISTICS_OF_AF)
		goto preview_stream_on_end;

	/* set raw frame rate instead with V4L2_ISP_RAW */
	if (a_uc_output_image == V4L2_ISP_RAW)
		goto preview_stream_on_end;

	/* reset the 1st output index*/
	g_dequeue_buff_index[a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			     a_uc_output_image] = V4L2_ISP_INDEX_MAX;
	g_enqueue_buff_index[a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			     a_uc_output_image] = V4L2_ISP_INDEX_MAX;

	/* reset the 1st output index for AF */
	if (a_uc_output_image == V4L2_ISP_STATITISTICS) {
		g_dequeue_buff_index[
			a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			V4L2_ISP_STATITISTICS_OF_AF] = V4L2_ISP_INDEX_MAX;
		g_enqueue_buff_index[
			a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			V4L2_ISP_STATITISTICS_OF_AF] = V4L2_ISP_INDEX_MAX;
	}

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto preview_stream_on_end;
	}

	isp_info_item_tag("Sensor: %d, Img: %d", a_uc_sensor,
		 a_uc_output_image);

	if (a_uc_sensor == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_OUT_ON_OFF_SETTING;
		command_img_on_off_reg =
		    AHB_ITEM_CHAGNE_SENSOR1_OUT_ON_OFF_SETTING_IMG;

		if (a_uc_output_image == V4L2_ISP_IMG1) {
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG1_EN, 1);
		} else if (a_uc_output_image == V4L2_ISP_IMG2) {
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG2_EN, 1);
		} else if (a_uc_output_image == V4L2_ISP_IMG3) {
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG3_EN, 1);
		} else if (a_uc_output_image == V4L2_ISP_STATITISTICS)
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_STATISTICS_EN,
					    1);
	} else if (a_uc_sensor == SENSOR2) {	/* isp2*/
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_OUT_ON_OFF_SETTING;
		command_img_on_off_reg =
		    AHB_ITEM_CHAGNE_SENSOR2_OUT_ON_OFF_SETTING_IMG;

		if (a_uc_output_image == V4L2_ISP_IMG1) {
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG1_EN, 1);
		} else if (a_uc_output_image == V4L2_ISP_IMG2) {
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG2_EN, 1);
		} else if (a_uc_output_image == V4L2_ISP_IMG3) {
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG3_EN, 1);
		} else if (a_uc_output_image == V4L2_ISP_STATITISTICS)
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_STATISTICS_EN,
					    1);
	} else if (a_uc_sensor == SENSOR3) {	/* isp lite*/
		/*      goto STREAMON_TEST;*/
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_OUT_ON_OFF_SETTING;
		command_img_on_off_reg =
		    AHB_ITEM_CHAGNE_SENSOR3_OUT_ON_OFF_SETTING_IMG;

		if (a_uc_output_image == V4L2_ISP_IMG1) {
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_IMG1_EN, 1);
		} else if (a_uc_output_image == V4L2_ISP_STATITISTICS)
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_STATISTICS_EN,
					    1);
	}

	isp_set_entry_value(AHB_ENTRY_INT_SYS_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_IMG1_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_IMG2_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_IMG3_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_3A_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_ISP_SOF_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_3A_LINEMEET_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_IMG1_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_IMG2_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_IMG3_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_3A_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_ISP_SOF_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_3A_LINEMEET_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_IMG_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_3A_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_ISP_SOF_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_3A_LINEMEET_STATUS, 0);

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);
	isp_set_field_value(command_img_on_off_reg, a_uc_output_image);

	/*  It may be blocked by INTDISABLE of isp_irq_handle*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);
	isp_set_field_value(command_img_on_off_reg, 0);

	SEMAPOST(get_command_semaphore());

preview_stream_on_end:

	if (err)
		isp_info_err("%d", err);

	if (errcode == 0 && err != 0)
		errcode = err;

	isp_info_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_preview_stream_on);

/*
*\brief Preview stream off for sensor1/sensor2/sensor3 (1: off , 0: ignore)
 *	Disable sensor status INT, Clear op start
*\param a_uc_sensor [in], sensor ID
*\param a_uc_output_image [in],
*	       Img1 for Preview,Img2 for video , Img3 for still,
*	       refert to ISPDRV_OUTPUT_IMAGE
*\return ETIMEDOUT.(Without AHB command finish INT)
*/
u32 ispdrv_preview_stream_off(u8 a_uc_sensor, u8 a_uc_output_image)
{
	u32 err = 0,
	    command_sensor_reg = 0,
	    command_system_reg = 0, command_img_on_off_reg = 0;

	isp_info_pu_start_tag();

	if (a_uc_sensor >= SENSOR_TOTAL
	    || a_uc_output_image >= V4L2_ISP_OUTPUT_IMAGE_TOTAL) {
		err = EINVAL;
		goto preview_stream_off_end;
	}

	/* Statistic of AF stream is configured with V4L2_ISP_STATITISTICS */
	if (a_uc_output_image == V4L2_ISP_STATITISTICS_OF_AF)
		goto preview_stream_off_end;

	/* set raw frame rate instead with V4L2_ISP_RAW */
	if (a_uc_output_image == V4L2_ISP_RAW)
		goto preview_stream_off_end;

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto preview_stream_off_end;
	}

	isp_info_item_tag("Sensor: %d, Img:%d", a_uc_sensor,
		 a_uc_output_image);

	if (a_uc_sensor == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_OUT_ON_OFF_SETTING;
		command_img_on_off_reg =
		    AHB_ITEM_CHAGNE_SENSOR1_OUT_ON_OFF_SETTING_IMG;

		if (a_uc_output_image == V4L2_ISP_IMG1)
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG1_EN, 0);
		else if (a_uc_output_image == V4L2_ISP_IMG2)
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG2_EN, 0);
		else if (a_uc_output_image == V4L2_ISP_IMG3)
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_IMG3_EN, 0);
		else if (a_uc_output_image == V4L2_ISP_STATITISTICS)
			isp_set_field_value(AHB_ITEM_SENSOR1_OUT_STATISTICS_EN,
					    0);
	} else if (a_uc_sensor == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_OUT_ON_OFF_SETTING;
		command_img_on_off_reg =
		    AHB_ITEM_CHAGNE_SENSOR2_OUT_ON_OFF_SETTING_IMG;
		if (a_uc_output_image == V4L2_ISP_IMG1)
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG1_EN, 0);
		else if (a_uc_output_image == V4L2_ISP_IMG2)
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG2_EN, 0);
		else if (a_uc_output_image == V4L2_ISP_IMG3)
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_IMG3_EN, 0);
		else if (a_uc_output_image == V4L2_ISP_STATITISTICS)
			isp_set_field_value(AHB_ITEM_SENSOR2_OUT_STATISTICS_EN,
					    0);
	} else if (a_uc_sensor == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_OUT_ON_OFF_SETTING;
		command_img_on_off_reg =
		    AHB_ITEM_CHAGNE_SENSOR3_OUT_ON_OFF_SETTING_IMG;
		if (a_uc_output_image == V4L2_ISP_IMG1)
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_IMG1_EN, 0);
		else if (a_uc_output_image == V4L2_ISP_STATITISTICS)
			isp_set_field_value(AHB_ITEM_SENSOR3_OUT_STATISTICS_EN,
					    0);
	} else {
		err = EINVAL;

		isp_err_pu_combo_item_tag("err: %d", err);

		goto preview_stream_off_semapost;
	}

	/* clear INT state, op start*/
	isp_set_entry_value(AHB_ENTRY_INT_SYS_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_IMG1_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_IMG2_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_IMG3_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_3A_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_ISP_SOF_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_3A_LINEMEET_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_IMG1_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_IMG2_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_IMG3_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_3A_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_ISP_SOF_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_3A_LINEMEET_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_IMG_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_3A_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_ISP_SOF_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_3A_LINEMEET_STATUS, 0);

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);
	isp_set_field_value(command_img_on_off_reg, a_uc_output_image);

	/* isp_set_field_value(AHB_ITEM_OPSTART,1);*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);
	isp_set_field_value(command_img_on_off_reg, 0);

preview_stream_off_semapost:
	SEMAPOST(get_command_semaphore());
preview_stream_off_end:

	if (err)
		isp_info_err("%d", err);

	isp_info_pu_end_tag();

	if (errcode == 0 && err != 0)
		errcode = err;
	return err;

}
EXPORT_SYMBOL(ispdrv_preview_stream_off);



/*
*\brief Get the output buffer info
*	   by output image index and output image buffer index
*\param a_uc_sensor [in], sensor ID. (Sensor1/Sensor2/Sensor3)
*\param a_uc_output_image [in], output source, refer to ISPDRV_OUTPUT_IMAGE
*\param a_pud_output_buffer [out], dequeue the buffer.(0/1/2/3)
*\param a_pud_output_buffer_isp1[out],
*	      dequeue the buffer for ISP1 in stripe mode.(0/1/2/3)
*\param a_pud_output_buffer_nv12uv[out],
*	      dequeue the buffer for UV of NV12 format.(0/1/2/3)
*\param a_pud_output_buffer_nv12uv_isp1 [out],
*	      dequeue the buffer
*	       for UV of NV12 format for ISP1 in stripe mode.(0/1/2/3)
*\param a_pt_crop_size [out], the crop size of the output image
*\param a_pt_iq_info_1 [out], the IQ information of the frame in JPEG exif
*\param a_pt_iq_info_2 [out], the IQ information  of the frame in JPEG debug
*\return error code
*/
u32 ispdrv_get_output_buffer(u8 a_uc_sensor, u8 a_uc_output_image,
			     u32 *a_pud_output_buffer,
			     u32 *a_pud_output_buffer_isp1,
			     u32 *a_pud_output_buffer_nv12uv,
			     u32 *a_pud_output_buffer_nv12uv_isp1,
			     struct s_scenario_size *a_pt_crop_size,
			     struct s_iq_info_1 *a_pt_iq_info_1,
			     struct s_iq_info_2 *a_pt_iq_info_2)
{
	u32 err = 0;
	u32 field, field_index, field_isp1, field_nv12_uv, field_nv12_uv_isp1;
	u8 img_index = 0;
	/* static int dump_count = 0;*/
	u8 *meta;
	u8 *uc_data;
	struct ispdrv_proc_still_pref *pref = NULL;

	isp_debug_pu_start_tag();

	isp_debug_item_tag("Sensor:%d, Img: %d",
		a_uc_sensor, a_uc_output_image);


	if (a_uc_sensor >= SENSOR_TOTAL
	    || a_uc_output_image >= V4L2_ISP_OUTPUT_IMAGE_TOTAL) {
		err = EINVAL;
		goto get_output_buffer_end;
	}

	if (check_raw_out_valid(a_uc_sensor, a_uc_output_image)) {
		err = EINVAL;
		goto get_output_buffer_end;
	}

	/* For still image in SS path */
	if (g_uc_sensor_1_scenario_id == V4L2_ISP_SCENARIO_PREVIEW_STILL_SS &&
		a_uc_output_image == V4L2_ISP_IMG3) {
		pref = (struct ispdrv_proc_still_pref *)
				iq_info_get_proc_still_arg_kva(a_uc_sensor);
		*a_pud_output_buffer = pref->out_addr.buffer_addr;
		MEMCPY(a_pt_iq_info_1, &pref->iq_info_1,
				sizeof(struct s_iq_info_1));
		MEMCPY(a_pt_iq_info_2, &pref->iq_info_2,
				sizeof(struct s_iq_info_2));
		a_pt_crop_size->width =
			g_kva_dzoom_info_ptr[a_uc_sensor]->
			eff_crop_size[a_uc_output_image][img_index].width;
		a_pt_crop_size->height =
			g_kva_dzoom_info_ptr[a_uc_sensor]->
			eff_crop_size[a_uc_output_image][img_index].height;
		goto get_output_buffer_end;
	}

	/* get the output img from local*/
	img_index = g_dequeue_buff_index[a_uc_sensor *
		V4L2_ISP_OUTPUT_IMAGE_TOTAL + a_uc_output_image];

	/* compare the local index to current index
	 *  to check whether there is more img read
	 */
	if (img_index != V4L2_ISP_INDEX_MAX)
		img_index = (img_index + 1) %
			g_buffer_number[a_uc_sensor][a_uc_output_image];
	else {
		field = g_output_img_table[a_uc_sensor *
			V4L2_ISP_OUTPUT_IMAGE_TOTAL + a_uc_output_image];
		img_index = isp_get_field_value(field);
	}

	/*  Get the effective crop size*/
	if (a_uc_output_image < V4L2_ISP_STATITISTICS) {
		/* only LV, Video, Still take crop size */
		a_pt_crop_size->width =
		g_kva_dzoom_info_ptr[a_uc_sensor]->
			eff_crop_size[a_uc_output_image][img_index].width;
		a_pt_crop_size->height =
		g_kva_dzoom_info_ptr[a_uc_sensor]->
			eff_crop_size[a_uc_output_image][img_index].height;
	}

	/* get the output buffer address*/
	field_index = a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL *
		V4L2_ISP_INDEX_TOTAL + a_uc_output_image *
		V4L2_ISP_INDEX_TOTAL + img_index;

	if (field_index >= (sizeof(g_output_field_table) / sizeof(u32))) {
		err = EINVAL;
		goto get_output_buffer_end;
	}

	field = g_output_field_table[field_index];
	*a_pud_output_buffer = isp_get_field_value(field);

	if (g_uc_sensor_1_scenario_id == V4L2_ISP_SCENARIO_PREVIEW_STRIPE &&
		a_uc_output_image < V4L2_ISP_STATITISTICS) {
		field_isp1 = g_output_field_table_isp1[field_index];
		*a_pud_output_buffer_isp1 = isp_get_field_value(field_isp1);
	}
	if (g_ucFormat[a_uc_sensor][a_uc_output_image] ==
		V4L2_ISP_OUTPUT_NV12 &&
		a_uc_output_image < V4L2_ISP_STATITISTICS) {
		field_nv12_uv = g_output_field_table_nv12_uv[field_index];
		*a_pud_output_buffer_nv12uv =
			isp_get_field_value(field_nv12_uv);
	}

	if (g_uc_sensor_1_scenario_id == V4L2_ISP_SCENARIO_PREVIEW_STRIPE &&
		g_ucFormat[a_uc_sensor][a_uc_output_image] ==
		V4L2_ISP_OUTPUT_NV12 &&
		a_uc_output_image < V4L2_ISP_STATITISTICS) {
		field_nv12_uv_isp1 =
			g_output_field_table_nv12_uv_isp1[field_index];
		*a_pud_output_buffer_nv12uv_isp1 =
			isp_get_field_value(field_nv12_uv_isp1);
	}

	/* Get IQ info*/
	if (a_uc_output_image != V4L2_ISP_STATITISTICS &&
		a_uc_output_image != V4L2_ISP_STATITISTICS_OF_AF) {
		isp_debug_desc_tag("copy debug IQ_INFO\n");
		if (g_kva_debug_iq_info_ptr[a_uc_sensor]->
			raw_info_addr[a_uc_output_image][img_index] != 0 &&
			a_pt_iq_info_1 != NULL) {
			uc_data = MAP_IVA_TO_KVA(
					g_kva_debug_iq_info_ptr[a_uc_sensor]->
					raw_info_addr[a_uc_output_image][
					img_index],
			sizeof(struct s_raw_info));
			if (uc_data) {
				MEMCPY((u8 *) &a_pt_iq_info_1->raw_info,
					(u8 *) uc_data,
					sizeof(struct s_raw_info));
				UNMAP_IVA_TO_KVA(uc_data);
			}
		}

		if (g_kva_debug_iq_info_ptr[a_uc_sensor]->
			shading_info_addr[a_uc_output_image][img_index] != 0 &&
			a_pt_iq_info_1 != NULL) {
			uc_data = MAP_IVA_TO_KVA(
				g_kva_debug_iq_info_ptr[a_uc_sensor]->
				shading_info_addr[a_uc_output_image][
				img_index], sizeof(struct s_shading_info));
			if (uc_data) {
				MEMCPY((u8 *) &a_pt_iq_info_1->shading_info,
					(u8 *) uc_data,
					sizeof(struct s_shading_info));
				UNMAP_IVA_TO_KVA(uc_data);
			}
		}

		if (g_kva_debug_iq_info_ptr[a_uc_sensor]->
			irp_info_addr[a_uc_output_image][img_index] != 0 &&
			a_pt_iq_info_1 != NULL) {
			uc_data = MAP_IVA_TO_KVA(
				g_kva_debug_iq_info_ptr[a_uc_sensor]->
				irp_info_addr[a_uc_output_image][img_index],
				sizeof(struct s_irp_info));
			if (uc_data) {
				MEMCPY((u8 *) &a_pt_iq_info_1->irp_info,
				(u8 *) uc_data,
				sizeof(struct s_irp_info));
				UNMAP_IVA_TO_KVA(uc_data);
			}
		}

		if (g_kva_debug_iq_info_ptr[a_uc_sensor]->
			gamma_tone_addr[a_uc_output_image][img_index] != 0 &&
			a_pt_iq_info_2 != NULL) {
			uc_data = MAP_IVA_TO_KVA(
				g_kva_debug_iq_info_ptr[a_uc_sensor]->
				gamma_tone_addr[a_uc_output_image][img_index],
				sizeof(struct s_irp_gamma_tone));
			if (uc_data) {
				MEMCPY((u8 *) &a_pt_iq_info_2->gamma_tone,
				(u8 *) uc_data,
				sizeof(struct s_irp_gamma_tone));
				UNMAP_IVA_TO_KVA(uc_data);
			}
		}

		if (g_kva_debug_iq_info_ptr[a_uc_sensor]->
			sw_info_addr[a_uc_output_image][img_index] != 0 &&
			a_pt_iq_info_1 != NULL) {
			uc_data = MAP_IVA_TO_KVA(
				g_kva_debug_iq_info_ptr[a_uc_sensor]->
				sw_info_addr[a_uc_output_image][img_index],
				sizeof(struct s_sw_info));
			if (uc_data) {
				MEMCPY((u8 *) &a_pt_iq_info_1->sw_info,
				(u8 *) uc_data,
				sizeof(struct s_sw_info));
				UNMAP_IVA_TO_KVA(uc_data);
			}
		}

	}

	if (a_uc_output_image == V4L2_ISP_STATITISTICS) {
		meta = MAP_IVA_TO_KVA(*a_pud_output_buffer, 0x1000);
		isp_debug_item_tag("sensor %d, meta %x %x %x %x %x %x %x %x",
			a_uc_sensor,
			*(meta), *(meta + 1),
			*(meta + 2), *(meta + 3), *(meta + 4), *(meta + 5),
			*(meta + 6), *(meta + 7));
		isp_debug_item_tag("AntiFlicker %x %x Size:%x %x %x %x",
			*(meta + 62),
			*(meta + 63), *(meta + 64), *(meta + 65), *(meta + 66),
			*(meta + 67));
	}

	/* set the enqueue buffer index
	 *  when the first frame done is triggered.
	 */
	if (g_enqueue_buff_index[a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
		a_uc_output_image] == V4L2_ISP_INDEX_MAX)
		g_enqueue_buff_index[a_uc_sensor *
			V4L2_ISP_OUTPUT_IMAGE_TOTAL + a_uc_output_image] =
			img_index;

	g_dequeue_buff_index[a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
		a_uc_output_image] = img_index;

get_output_buffer_end:

	if (a_pud_output_buffer != NULL)
		isp_debug_item_tag("OutputBuf: 0x%x", *a_pud_output_buffer);
	if (a_pud_output_buffer_isp1 != NULL)
		isp_debug_item_tag("OutputBuf_isp1: 0x%x",
			*a_pud_output_buffer_isp1);
	if (a_pud_output_buffer_nv12uv != NULL)
		isp_debug_item_tag("OutputBuf_nv12uv: 0x%x"
			, *a_pud_output_buffer_nv12uv);
	if (a_pud_output_buffer_nv12uv_isp1 != NULL)
		isp_debug_item_tag("OutputBuf_nv12uv_isp1: 0x%x"
			, *a_pud_output_buffer_nv12uv_isp1);
	if (a_pt_crop_size != NULL)
		isp_debug_item_tag("CropW: %d, CropH: %d"
			, a_pt_crop_size->width, a_pt_crop_size->height);

	if (err)
		isp_debug_err("%d", err);

	isp_debug_pu_end_tag();

	if (errcode == 0 && err != 0)
		errcode = err;
	return err;

}
EXPORT_SYMBOL(ispdrv_get_output_buffer);

/*
*\brief Check if there si more image ready
*\param a_uc_sensor [in], sensor ID
*\param a_uc_output_image [in], output source
*\param a_pb_more_img [out], true if there is more imgs that are ready
*\return Error code
*/
u32 ispdrv_is_more_img_ready(u8 a_uc_sensor, u8 a_uc_output_image,
			     bool *a_pb_more_img)
{
	u32 err = 0, img_field;
	u8 img_index = 0, current_img_index = 0;

	isp_debug_pu_start_tag();

	isp_debug_item_tag("Sensor: %d, Img: %d", a_uc_sensor,
		a_uc_output_image);

	if (a_uc_sensor >= SENSOR_TOTAL
	    || a_uc_output_image >= V4L2_ISP_OUTPUT_IMAGE_TOTAL) {
		err = EINVAL;
		goto is_more_img_ready_end;
	}

	if (g_ready_buffer
	    [a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL + a_uc_output_image] ==
	    0) {
		*a_pb_more_img = false;
		goto is_more_img_ready_end;
	}
	if (a_uc_output_image == V4L2_ISP_IMG3)
		isp_debug_desc_tag("BIST: g_ready_buffer is 1\n");

	img_index =
	    g_dequeue_buff_index[a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
				 a_uc_output_image];

	img_field =
	    g_output_img_table[a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       a_uc_output_image];
	current_img_index = isp_get_field_value(img_field);

	if (img_index != current_img_index)
		*a_pb_more_img = true;
	else
		*a_pb_more_img = false;
	if (a_uc_output_image == V4L2_ISP_IMG3)
		isp_debug_item_tag("BIST: idx %d, current idx %d\n",
				img_index, current_img_index);

is_more_img_ready_end:

	isp_debug_item_tag("Rdy %d", *a_pb_more_img);

	if (err)
		isp_debug_err("%d", err);

	isp_debug_pu_end_tag();

	if (errcode == 0 && err != 0)
		errcode = err;

	return err;
}
EXPORT_SYMBOL(ispdrv_is_more_img_ready);

/*
*\brief Set the output buffer
*\param a_uc_sensor [in], sensor ID
*\param a_uc_output_image [in], output source
*\param a_ud_buffer_addr [in], enqueue the buffer
*\param a_ud_buffer_addr_isp1 [in], enqueue the buffer for ISP1 in stripe mode
*\param a_ud_buffer_addr_nv12uv [in],
*	       enqueue the buffer for UV of NV12 format
*\param a_ud_buffer_addr_nv12uv_isp1 [in],
*	      enqueue the buffer for UV of NV12 format for ISP1 in stripe mode
*\return Error code
*/
u32 ispdrv_set_output_buffer(u8 a_uc_sensor, u8 a_uc_output_image,
			     u32 a_ud_buffer_addr, u32 a_ud_buffer_addr_isp1,
			     u32 a_ud_buffer_addr_nv12uv,
			     u32 a_ud_buffer_addr_nv12uv_isp1)
{
	u32 err =
	    0, field, field_index, field_isp1, field_nv12_uv,
	    field_nv12_uv_isp1, command_system_reg, command_sensor_addr_reg,
	    command_out_img_reg, command_index_reg;
	u8 enqueue_index;

	isp_debug_pu_start_tag();

	if (a_uc_sensor >= SENSOR_TOTAL
	    || a_uc_output_image >= V4L2_ISP_OUTPUT_IMAGE_TOTAL) {
		err = EINVAL;
		goto set_output_buffer_end;
	}

	if (check_raw_out_valid(a_uc_sensor, a_uc_output_image)) {
		err = EINVAL;
		goto set_output_buffer_end;
	}

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_output_buffer_end;
	}

	isp_debug_item_tag("Sensor:%d, Img:%d", a_uc_sensor,
		a_uc_output_image);


	if (a_ud_buffer_addr != 0)
		isp_debug_item_tag("OutputBuf: 0x%x", a_ud_buffer_addr);
	if (a_ud_buffer_addr_isp1 != 0)
		isp_debug_item_tag("OutputBuf_isp1: 0x%x",
			a_ud_buffer_addr_isp1);
	if (a_ud_buffer_addr_nv12uv != 0)
		isp_debug_item_tag("OutputBuf_nv12uv: 0x%x",
			a_ud_buffer_addr_nv12uv);
	if (a_ud_buffer_addr_nv12uv_isp1 != 0)
		isp_debug_item_tag("OutputBuf_nv12uv_isp1: 0x%x",
			a_ud_buffer_addr_nv12uv_isp1);

	/*  get the enqueue buffer index*/
	enqueue_index =
	    g_enqueue_buff_index[a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
				 a_uc_output_image];

	/* isp_info("%s - Index:%d", __func__, enqueue_index);*/

	/* get the field then set the new address*/
	field_index =
	    a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL * V4L2_ISP_INDEX_TOTAL +
	    a_uc_output_image * V4L2_ISP_INDEX_TOTAL + enqueue_index;
	if (field_index >= (sizeof(g_output_field_table) / sizeof(u32))) {
		err = EINVAL;
		goto set_output_buffer_semapost;
	}

	field = g_output_field_table[field_index];
	isp_set_field_value(field, a_ud_buffer_addr);

	if (g_uc_sensor_1_scenario_id == V4L2_ISP_SCENARIO_PREVIEW_STRIPE &&
		a_uc_output_image < V4L2_ISP_STATITISTICS) {
		field_isp1 = g_output_field_table_isp1[field_index];
		isp_set_field_value(field_isp1, a_ud_buffer_addr_isp1);
	}

	if (g_ucFormat[a_uc_sensor][a_uc_output_image] ==
		V4L2_ISP_OUTPUT_NV12 &&
		a_uc_output_image < V4L2_ISP_STATITISTICS) {
		field_nv12_uv = g_output_field_table_nv12_uv[field_index];
		isp_set_field_value(field_nv12_uv, a_ud_buffer_addr_nv12uv);
	}

	if (g_uc_sensor_1_scenario_id == V4L2_ISP_SCENARIO_PREVIEW_STRIPE &&
		g_ucFormat[a_uc_sensor][a_uc_output_image] ==
		V4L2_ISP_OUTPUT_NV12 &&
		a_uc_output_image < V4L2_ISP_STATITISTICS) {
		field_nv12_uv_isp1 =
			g_output_field_table_nv12_uv_isp1[field_index];
		isp_set_field_value(field_nv12_uv_isp1,
			a_ud_buffer_addr_nv12uv_isp1);
	}

	if (a_uc_sensor == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_addr_reg = AHB_ITEM_CHANGE_SENSOR1_OUT_ADDR;
		command_out_img_reg = AHB_ITEM_CHANGE_SENSOR1_OUT_ADDR_IMG;
		command_index_reg = AHB_ITEM_CHANGE_SENSOR1_OUT_ADDR_IDX;
	} else if (a_uc_sensor == SENSOR2) {	/* isp2*/
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_addr_reg = AHB_ITEM_CHANGE_SENSOR2_OUT_ADDR;
		command_out_img_reg = AHB_ITEM_CHANGE_SENSOR2_OUT_ADDR_IMG;
		command_index_reg = AHB_ITEM_CHANGE_SENSOR2_OUT_ADDR_IDX;
	} else {		/* isp lite*/
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_addr_reg = AHB_ITEM_CHANGE_SENSOR3_OUT_ADDR;
		command_out_img_reg = AHB_ITEM_CHANGE_SENSOR3_OUT_ADDR_IMG;
		command_index_reg = AHB_ITEM_CHANGE_SENSOR3_OUT_ADDR_IDX;
	}

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_addr_reg, 1);
	isp_set_field_value(command_out_img_reg, a_uc_output_image);
	isp_set_field_value(command_index_reg, enqueue_index);


	/* err = command_start(__func__);*/
	err = command_start_poll(__func__);

	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_addr_reg, 0);
	isp_set_field_value(command_out_img_reg, 0);
	isp_set_field_value(command_index_reg, 0);

	/* isp_info("%s - Sensor:%d, outputImage:%d\n",
	 *		   __func__, a_uc_sensor, a_uc_output_image);
	 */

	/*  isp_info(" index:%d, addr:%x",
	 *		   enqueue_index, a_udBufferAddr);
	 */

	/* update the index*/
	enqueue_index = (enqueue_index + 1) %
		g_buffer_number[a_uc_sensor][a_uc_output_image];
	g_enqueue_buff_index[a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
		a_uc_output_image] = enqueue_index;


set_output_buffer_semapost:
	SEMAPOST(get_command_semaphore());
set_output_buffer_end:

	if (err)
		isp_debug_err("%d", err);

	if (errcode == 0 && err != 0)
		errcode = err;

	isp_debug_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_set_output_buffer);

static u8 is_raw_stream_on(u8 fr_mode)
{
	switch (fr_mode) {
	case ISP_RAW_FR_1:
	case ISP_RAW_FR_2:
	case ISP_RAW_FR_3:
	case ISP_RAW_FR_4:
		return true;
	default:
		return false;
	}
}

/*
 * \brief frame rate control on Raw_top's DRAM output raw path
 * \param sensor_id [in], sensor id
 * \param raw_fr [in], see ispdrv_raw_frame_rate
 * \return Error code
 */
u32 ispdrv_set_raw_frame_rate(u8 sensor_id, u8 raw_fr)
{
	u32 err = 0;
	u32 command_sensor_reg = 0;
	u32 command_system_reg = 0;
	u32 value_int_reg = 0;
	u32 value_fr_reg = 0;
	u8 q_buf_idx = 0;
	u8 stream_on = 0;

	isp_info_pu_start_tag();

	if (sensor_id >= SENSOR_TOTAL) {
		err = EINVAL;
		goto set_raw_frame_rate_end;
	}

	isp_info_item_tag("sensorId:%d, raw_fr:%d", sensor_id, raw_fr);

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_raw_frame_rate_end;
	}

	stream_on = is_raw_stream_on(raw_fr);

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		/* no need to set sensor out_on_off setting & it img type */
		command_sensor_reg =
				AHB_ITEM_CHANGE_SENSOR1_RAW_FRAME_RATE;
		value_fr_reg = AHB_ITEM_SENSOR1_RAW_FRAME_RATE;
		value_int_reg = AHB_ENTRY_INT_SENSOR_1_RAW_STATUS;
	} else /* only support sensor 1 so far */
		goto set_raw_frame_rate_semapost;

	/* state changed, consider switch raw stream */
	if (g_uc_raw_stream_on != stream_on) {
		g_uc_raw_stream_on = stream_on;
		if (stream_on > 0) {
			q_buf_idx = sensor_id * V4L2_ISP_OUTPUT_IMAGE_TOTAL
					+ V4L2_ISP_RAW;
			/* reset the 1st output index */
			g_dequeue_buff_index[q_buf_idx] = V4L2_ISP_INDEX_MAX;
			g_enqueue_buff_index[q_buf_idx] = V4L2_ISP_INDEX_MAX;
		}
	} else {
		/* don't need to reset INT, if no stream on/off triggered */
		value_int_reg = 0;
	}

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);
	isp_set_field_value(value_fr_reg, raw_fr);
	if (value_int_reg > 0)
		isp_set_entry_value(value_int_reg, 0);

	err = command_start(__func__);

	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_raw_frame_rate_semapost:
	SEMAPOST(get_command_semaphore());
set_raw_frame_rate_end:
	if (err != 0) {
		errcode = err;
		isp_warn_pu_combo_item_tag("SensorId:%d, fr:%d err:%d",
			sensor_id, raw_fr, err);
	}

	isp_info_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_raw_frame_rate);

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
u32 ispdrv_proc_still(u8 sensor_id,
		u32 src_addr,
		struct ispdrv_output_addr_still out_addr,
		struct ispdrv_proc_still_opt *opt,
		struct s_iq_info_1 *a_pt_iq_info_1,
		struct s_iq_info_2 *a_pt_iq_info_2)
{
	u32 err = 0;
	struct ispdrv_proc_still_pref *pref = NULL;
#if 0
	u32 command_system_reg = 0,
		command_sensor_addr_reg = 0,
		command_chg_addrs_reg = 0;
	u32 value_still_addr1_reg = 0;
	u32 buffer_addr = 0;
#endif
	isp_info_pu_start_tag();

	isp_info_item_tag("Sensor: %d, src_addr:0x%x out_addr: 0x%x\n",
		sensor_id, src_addr, out_addr.buffer_addr);

	if (check_raw_out_valid(sensor_id, V4L2_ISP_RAW)) {
		err = EINVAL;
		goto proc_still_end;
	}

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto proc_still_end;
	}
#if 0
	switch (sensor_id) {
	case SENSOR1:

		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_addr_reg = AHB_ITEM_CHANGE_SENSOR1_OUT_ADDR;
		command_chg_addrs_reg = AHB_ITEM_CHANGE_SENSOR1_OUT_ADDR_IMG;
		value_still_addr1_reg = AHB_ITEM_SENSOR1_OUT_IMG3_ADDR1;
		buffer_addr = out_addr.buffer_addr;
		break;
	default:
		goto proc_still_semapost;
	}
#endif
	pref = (struct ispdrv_proc_still_pref *)
			iq_info_get_proc_still_arg_kva(sensor_id);

	pref->src_addr = src_addr;
	MEMCPY(&pref->out_addr, &out_addr,
			sizeof(struct ispdrv_output_addr_still));
	MEMCPY(&pref->opt, opt,
			sizeof(struct ispdrv_proc_still_opt));
	MEMCPY(&pref->iq_info_1, a_pt_iq_info_1,
			sizeof(struct s_iq_info_1));
	MEMCPY(&pref->iq_info_2, a_pt_iq_info_2,
			sizeof(struct s_iq_info_2));

#if 0
	/* also set still buf's 1st addr with out_addr */
	isp_set_field_value(value_still_addr1_reg, buffer_addr);

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_addr_reg, 1);
	isp_set_field_value(command_chg_addrs_reg, V4L2_ISP_RAW);
#endif
	isp_set_field_value(AHB_ITEM_DO_SENSOR1_PROC_STILL, 1);

	err = command_start(__func__);
#if 0
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_addr_reg, 0);
	isp_set_field_value(command_chg_addrs_reg, 0);
#endif
	isp_set_field_value(AHB_ITEM_DO_SENSOR1_PROC_STILL, 0);

	SEMAPOST(get_command_semaphore());
proc_still_end:

	if (errcode == 0 && err != 0)
		errcode = err;

	isp_info_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_proc_still);


/*
*\brief Enable independent AF interrupt mode
*\param None
*\return EINVAL
*/
u32 ispdrv_set_ind_af_int(void)
{
	u32 err = 0;

	isp_info_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto enable_independent_af_int;
	}

	isp_set_field_value(AHB_ITEM_ENABLE_INDEPENDENT_AF_STATISTIC_DONE_INT,
		1);

	err = command_start(__func__);

	isp_set_field_value(AHB_ITEM_ENABLE_INDEPENDENT_AF_STATISTIC_DONE_INT,
		0);

	SEMAPOST(get_command_semaphore());
enable_independent_af_int:

	if (errcode == 0 && err != 0)
		errcode = err;

	isp_info_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_ind_af_int);


/***********************************************************
*		misc api		                    *
*************************************************************/
/*
*\brief Get base size for digital zoom
*\param a_uc_sensor [In], Sensor1/ Sensor2
*\param a_uc_output_image [In], LV/ Video/ Still
*\param a_puw_base_width [out], the base width
*\param a_puw_base_height [out], the base height
*/
u32 ispdrv_get_base_size_for_digital_zoom(u8 a_uc_sensor,
	u8 a_uc_output_image,
	u16 *a_puw_base_width, u16 *a_puw_base_height)
{
	u32 err = 0;

	isp_info_pu_start_tag();
	isp_info_item_tag("Sensor: %d, Img: %d", a_uc_sensor,
		a_uc_output_image);

	if (a_uc_sensor > SENSOR3) {
		err = EINVAL;
		goto get_base_size_for_digital_zoom_end;
	}

	*a_puw_base_width =
		g_kva_dzoom_info_ptr[a_uc_sensor]->
		base_size[a_uc_output_image].width;
	*a_puw_base_height =
		g_kva_dzoom_info_ptr[a_uc_sensor]->
		base_size[a_uc_output_image].height;

	isp_info_item_tag("BaseW: %d BaseH: %d",
		*a_puw_base_width, *a_puw_base_height);

get_base_size_for_digital_zoom_end:
	if (err)
		isp_info_err("%d", err);

	isp_info_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_get_base_size_for_digital_zoom);

/*
*\brief Set crop size for digital zoom
*\param a_uc_sensor [In], the sensor ID
*\param a_pt_lv [In], the crop size of LV img
*\param a_pt_video [In], the crop size of video img
*/
u32 ispdrv_set_crop_size_for_digital_zoom(u8 a_uc_sensor,
	struct s_scenario_size *a_pt_lv, struct s_scenario_size *a_pt_video,
	struct s_scenario_size *a_pt_still)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_info_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_crop_size_for_digital_zoom_end;
	}

	isp_info_item_tag("Sensor:%d", a_uc_sensor);

	if (a_uc_sensor == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_CROPSIZE;
	} else if (a_uc_sensor == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_CROPSIZE;
	} else if (a_uc_sensor == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_CROPSIZE;
	} else {
		err = EINVAL;
		goto set_crop_size_for_digital_zoom_semapost;
	}

	/* Set the crop size and flag*/
	if (a_pt_lv != NULL) {
		isp_info_item_tag("LV. W: %d H: %d",
			a_pt_lv->width, a_pt_lv->height);

		g_kva_dzoom_info_ptr[a_uc_sensor]->
			crop_size[V4L2_ISP_IMG1].width = a_pt_lv->width;
		g_kva_dzoom_info_ptr[a_uc_sensor]->
			crop_size[V4L2_ISP_IMG1].height = a_pt_lv->height;
		g_kva_dzoom_info_ptr[a_uc_sensor]->
			crop_size_changed[V4L2_ISP_IMG1] = true;
	}

	/* Set the crop size and flag*/
	if (a_pt_video != NULL) {
		isp_info_item_tag("Video. W: %d H: %d",
			a_pt_video->width, a_pt_video->height);

		g_kva_dzoom_info_ptr[a_uc_sensor]->
			crop_size[V4L2_ISP_IMG2].width = a_pt_video->width;
		g_kva_dzoom_info_ptr[a_uc_sensor]->
			crop_size[V4L2_ISP_IMG2].height = a_pt_video->height;
		g_kva_dzoom_info_ptr[a_uc_sensor]->
			crop_size_changed[V4L2_ISP_IMG2] = true;
	}

	/* Set the crop size and flag*/
	if (a_pt_still != NULL) {
		isp_info_item_tag("Still. W: %d H: %d",
			a_pt_still->width, a_pt_still->height);

		g_kva_dzoom_info_ptr[a_uc_sensor]->
			crop_size[V4L2_ISP_IMG3].width = a_pt_still->width;
		g_kva_dzoom_info_ptr[a_uc_sensor]->
			crop_size[V4L2_ISP_IMG3].height = a_pt_still->height;
		g_kva_dzoom_info_ptr[a_uc_sensor]->
			crop_size_changed[V4L2_ISP_IMG3] = true;
	}

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* opstart*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_crop_size_for_digital_zoom_semapost:
	SEMAPOST(get_command_semaphore());
set_crop_size_for_digital_zoom_end:

	if (err)
		isp_info_err("%d", err);

	if (errcode == 0 && err != 0)
		errcode = err;

	isp_info_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_crop_size_for_digital_zoom);

/*
*\brief Get the address of the scenario info
*           shared between ISP driver and firmware for bist
*\param scenario_info_ptr [Out], the scenario info
*\return none
*/
void *ispdrv_get_shared_scenario_info_ptr(void)
{
	return (void *) g_kva_scenario_info_ptr;
}
EXPORT_SYMBOL(ispdrv_get_shared_scenario_info_ptr);

/*
 *\brief Skip image
 *\param sensor [in], sensor ID
 *\param output_img [in], output source
 *\return Error code
*/
u32 ispdrv_skip_img(u8 sensor, u8 output_img)
{
	u32 err = 0, field;
	u8 img_index = 0, enqueue_index = 0;

	isp_debug_pu_start_tag();

	isp_debug_item_tag("Sensor:%d Img: %d", sensor, output_img);

	if (sensor >= SENSOR_TOTAL
			|| output_img >= V4L2_ISP_OUTPUT_IMAGE_TOTAL) {
		err = EINVAL;
		goto SKIP_IMG_END;
	}

	/* get the output img from local */
	img_index = g_dequeue_buff_index
			[sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL + output_img];

	/* compare the local index to  current index
	 * to check whether there is more img read
	 */
	if (img_index != V4L2_ISP_INDEX_MAX)
		img_index = (img_index + 1) %
			g_buffer_number[sensor][output_img];
	else {
		field = g_output_img_table
			[sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL + output_img];
		img_index = isp_get_field_value(field);
	}

	/* set the enqueue buffer index
	 * when the first frame done is triggered.
	 */
	if (g_enqueue_buff_index[sensor
		* V4L2_ISP_OUTPUT_IMAGE_TOTAL
		+ output_img] == V4L2_ISP_INDEX_MAX)
		g_enqueue_buff_index[sensor
		* V4L2_ISP_OUTPUT_IMAGE_TOTAL
		+ output_img] = img_index;

	g_dequeue_buff_index[sensor
		* V4L2_ISP_OUTPUT_IMAGE_TOTAL + output_img] = img_index;

	/* get the enqueue buffer index */
	enqueue_index = g_enqueue_buff_index[sensor
	* V4L2_ISP_OUTPUT_IMAGE_TOTAL
	+ output_img];

	/* update the index */
	enqueue_index = (enqueue_index+1) % g_buffer_number[sensor][output_img];
	g_enqueue_buff_index[sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL
		+ output_img] = enqueue_index;

SKIP_IMG_END:

	if (err)
		isp_info_err("%d", err);

	isp_debug_pu_end_tag();

	if (errcode == 0 && err != 0)
		errcode = err;

	return err;
}
EXPORT_SYMBOL(ispdrv_skip_img);


/**
 *\brief Set RAW format in dump RAW scenarios.
 *\param sensor_id [in], sensor id
 *\param format, See ispdrv_raw_format.
 *               Default is ISP_RAW_FORMAT_ALTEK_RAW10
 *\return Error code
 */
u32 ispdrv_set_raw_format(u8 sensor_id, u8 format)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;
	u32 value_fmt_reg = 0;

	isp_info_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_raw_format_end;
	}

	isp_info_item_tag("Sensor:%d", sensor_id);

	switch (sensor_id) {
	case SENSOR1:
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_RAW_FORMAT;
		value_fmt_reg = AHB_ITEM_SENSOR1_RAW_DUMP_FORMAT;
		break;
	case SENSOR2:
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_RAW_FORMAT;
		value_fmt_reg = AHB_ITEM_SENSOR2_RAW_DUMP_FORMAT;
		break;
	case SENSOR3:
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_RAW_FORMAT;
		value_fmt_reg = AHB_ITEM_SENSOR3_RAW_DUMP_FORMAT;
		break;
	default:
		err = EINVAL;
		goto set_raw_format_end;
	}

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* set the value register */
	isp_set_field_value(value_fmt_reg, format);

	err = command_start(__func__);

	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_raw_format_end:
	SEMAPOST(get_command_semaphore());

	if (err != 0) {
		isp_info_err("%d", err);
		errcode = err;
	}
	isp_info_pu_end_tag();
	return err;
}
EXPORT_SYMBOL(ispdrv_set_raw_format);

/**
 *\brief Set CBC map addresses.
 *\param sns1_iva[in], the address of the CBC Map addr for sensor1.
 *\param sns2_iva[in], the address of the CBC Map addr for sensor2.
 *\param sns3_iva[in], the address of the CBC Map addr for sensor3.
 *\return Error code
 */
u32 ispdrv_set_cbc_map_addr(u32 sns1_iva, u32 sns2_iva, u32 sns3_iva)
{
	u32 err = 0;

	isp_info_pu_start_tag();
	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_cbc_map_addr_end;
	}

	g_iva_cbc_map_addr[SENSOR1] = sns1_iva;
	g_iva_cbc_map_addr[SENSOR2] = sns2_iva;
	g_iva_cbc_map_addr[SENSOR3] = sns3_iva;

	isp_set_field_value(AHB_ITEM_SENSOR1_CBC_MAP_ADDR, sns1_iva);
	isp_set_field_value(AHB_ITEM_SENSOR2_CBC_MAP_ADDR, sns2_iva);
	isp_set_field_value(AHB_ITEM_SENSOR3_CBC_MAP_ADDR, sns3_iva);

	isp_info_item_tag("cbc_addr1: 0x%x, cbc_addr2: 0x%x, cbc_addr3: 0x%x",
			sns1_iva, sns2_iva, sns3_iva);

set_cbc_map_addr_end:
	SEMAPOST(get_command_semaphore());
	if (err) {
		isp_info_err("%d", err);
		errcode = err;
	}
	isp_info_pu_end_tag();
	return err;
}
EXPORT_SYMBOL(ispdrv_set_cbc_map_addr);
/*
 * /////////////////////////////////////////////////////////
 *		Local Function		*
 * /////////////////////////////////////////////////////////
*/
u32 command_start(const char *func_name)
{
	u32 err = 0;

#if 0/* with semaphore, it has no more use.*/
	/* 1. check "ip_busy" = 0(if not, wait "sys_cmd_finish_int")*/
	if (isp_get_field_value(AHB_ITEM_IP_BUSY)) {
		/* wait for int*/
		err = wait_for_irq(func_name, V4L2_ISP_SYS_CMD_FINISH_INT);
		if (err) {
			isp_err_lo_combo_item_tag("%s - error code: %d",
				func_name, err);
			return err;
		}
		/* clear "sys_cmd_finish_int"*/
		isp_set_field_value(AHB_ITEM_SYS_CMD_FINISH_INT, 0);
	}
#endif

	/* 2. set "ip_busy" = 1*/
	/* isp_set_field_value(AHB_ITEM_IP_BUSY, 1);*/

	/* 3. set "opstart" = 1*/
	isp_set_field_value(AHB_ITEM_OPSTART, 1);

	/* 4. wait "sys_cmd_finish_int" = 1*/

	err = wait_for_irq(func_name, V4L2_ISP_SYS_CMD_FINISH_INT);

	/* clear "sys_cmd_finish_int"*/
	isp_set_field_value(AHB_ITEM_SYS_CMD_FINISH_INT, 0);

#if 0	/* Not here! Dump at the end of wait_for_irq() instead */
	auto_dump_on_error(err);
#endif

	return err;

}

u32 command_start_poll(const char *func_name)
{
	u32 err = 0, reg = 0;
	int i = 0;
	int loop_count = 100;
#ifdef SHOWTIME
	struct timeval t1, t2;
	long long usec1, usec2;
#endif

	isp_info_lo_start_tag();

	g_enable_irq_flag = false;

	/* 2. set "ip_busy" = 1*/
	/* isp_set_field_value(AHB_ITEM_IP_BUSY, 1);*/

	/* 3. set "opstart" = 1*/
	isp_set_field_value(AHB_ITEM_OPSTART, 1);

#ifdef SHOWTIME
	do_gettimeofday(&t1);
#endif

	/* increase the loop count in ISPDrv_Close()*/
	if (strcmp(func_name, "ispdrv_close") == 0) {
		loop_count = 600;
		isp_info_desc_tag("close opstart\n");
	}

	/* 4. wait "sys_cmd_finish_int" = 1*/
	for (i = 0; i < loop_count; i++) {
		reg = isp_get_field_value(AHB_ITEM_SYS_CMD_FINISH_INT);
		if (reg == 1) {
			/* clear "sys_cmd_finish_int"*/
			isp_set_field_value(AHB_ITEM_SYS_CMD_FINISH_INT, 0);

			/* Clear INT state*/
			g_latest_system_int &= ~V4L2_ISP_SYS_CMD_FINISH_INT;

			break;
		}
		udelay(84);
	}

#ifdef SHOWTIME
	do_gettimeofday(&t2);
#endif

	if (i >= loop_count) {
		err = isp_get_field_value(AHB_ITEM_SYSTEM_MESSAGE);
		if (err)
			isp_alert_lo_combo_item_tag(
					"func: %s, isp fw err: 0x%08x\n",
					func_name, err);
		err = ETIMEDOUT;
#ifdef DEBUG_JTAG
		while (1)	/* (g_open_count)*/
			SLEEPRANGE(10000, 10500);
#endif
	}

	/* log the wait/get event time*/
#ifdef SHOWTIME
	usec1 = t1.tv_sec * 1000LL + t1.tv_usec / 1000;
	usec2 = t2.tv_sec * 1000LL + t2.tv_usec / 1000;
	isp_info_item_tag("%s - wait before: %lld ,wait after: %lld",
		func_name, usec1, usec2);
#endif
	g_enable_irq_flag = true;

	auto_dump_on_error(err); /* Dump at the end of command_start_poll() */

	isp_info_lo_end_tag();

	return err;
}

u32 wait_for_irq(const char *func_name, u32 event)
{
	u32 err = 0, wait_time = IRQ_WAIT_TIME;
	int state;
	/* #ifdef SHOWTIME*/
	struct timeval t1, t2;
	long long msec1, msec2;
	/* #endif*/

	/*isp_info_lo_start_tag();
	 * isp_info_item_tag("event: 0x%x", event);
	 */

	/* #ifdef SHOWTIME*/
	do_gettimeofday(&t1);
	/* #endif*/

	/* ISPDrv_StopPreviewMode() needs more time to finish*/
	if (strcmp("ispdrv_stop_preview_mode", func_name) == 0)
		wait_time = 1000;	/* 500*/

	state = wait_event_timeout(WAITQ, (g_latest_system_int & event) != 0,
		msecs_to_jiffies(wait_time));

	/* #ifdef SHOWTIME*/
	do_gettimeofday(&t2);
	/* #endif*/

	/*isp_info_item_tag("int state: 0x%x event bit: 0x%x",
	 * g_latest_system_int, event);
	 */

	if (strcmp("ispdrv_set_output_buffer", func_name) == 0)
		isp_debug_desc_tag("ispdrv_set_output_buffer wait IRQ\n");

	if (state == 0) {
		if (strcmp("ispdrv_set_output_buffer", func_name) == 0)
			isp_debug_item_tag("%s - time out\n", func_name);

		err = isp_get_field_value(AHB_ITEM_SYSTEM_MESSAGE);
		if (err)
			isp_alert_lo_combo_item_tag(
				"func: %s, isp fw err: 0x%08x\n",
				func_name, err);
		err = ETIMEDOUT;

		/* for debug*/
#ifdef DEBUG_JTAG
		while (1)	/* (g_open_count)*/
			SLEEPRANGE(10000, 10500);
#endif

		goto wait_for_irq_end;
	}

	/* clear event bits*/
	spin_lock_irq(&system_int_lock);
	g_latest_system_int &= ~event;
	spin_unlock_irq(&system_int_lock);

	isp_set_entry_value(AHB_ENTRY_INT_SYS_STATUS, g_latest_system_int);

	/* #ifdef SHOWTIME*/
	/* log the wait/get event time*/
	msec1 = t1.tv_sec * 1000LL + t1.tv_usec / 1000;
	msec2 = t2.tv_sec * 1000LL + t2.tv_usec / 1000;
	if (strcmp("ispdrv_stop_preview_mode", func_name) == 0)
		isp_info_lo_combo_item_tag(
			"%s - wait before: %lld ,wait after: %lld\n",
			 func_name, msec1, msec2);
	/* #endif*/

	if (strcmp("ispdrv_set_output_buffer", func_name) == 0)
		isp_info_lo_combo_item_tag(
			"%s - wait before: %lld ,wait after: %lld\n",
			 func_name, msec1, msec2);

wait_for_irq_end:
	auto_dump_on_error(err); /* Dump at the end of wait_for_irq() */
	/* isp_info_err("%d", err);
	 * isp_info_lo_end_tag();
	 */
	return err;
}

u32 common_get_iva_fw_addr(void)
{
	return g_iva_fw_addr;
}

u64 common_get_ahb_addr(void)
{
	return g_ahb_addr;
}

u32 common_get_ahb_size(void)
{
	return g_ahb_size;
}

void common_get_format(u8 sensor, u8 img_type, u8 *format)
{
	*format = g_ucFormat[sensor][img_type];
}

void common_get_buff_number(u8 *buff_ptr)
{
	MEMCPY(buff_ptr, (u8 *) &g_buffer_number[0][0]
		, sizeof(g_buffer_number));
}
void common_get_sensor_id(u8 *sensor_id)
{
	*sensor_id = g_sensor_id_to_download_img;
}

void common_get_out_field_table(u32 *output_field_ptr)
{
	MEMCPY((u8 *)output_field_ptr, (u8 *)g_output_field_table,
		sizeof(g_output_field_table));
}

bool common_is_qmerge_data_addr_set(void)
{
	bool result;

	if (g_iva_sensor1_qmerge_data_addr == 0 ||
		g_iva_sensor2_qmerge_data_addr == 0 ||
		g_iva_sensor3_qmerge_data_addr == 0)
		result = false;
	else
		result = true;

	return result;
}

void common_reset_bist_frame_done_cnt(void)
{
	g_fd_cnt_1stsensor = 0;
	g_fd_cnt_2ndsensor = 0;
}

u32 common_get_1stsensor_frame_done_cnt(void)
{
	return g_fd_cnt_1stsensor;
}

u32 common_get_2ndsensor_frame_done_cnt(void)
{
	return g_fd_cnt_2ndsensor;
}

u32 common_get_qmerge_data_addr_by_sensor_id(u8 sensor_id)
{
	u32 qmerge_data_addr = 0;

	if (sensor_id == SENSOR1)
		qmerge_data_addr = g_iva_sensor1_qmerge_data_addr;
	else if (sensor_id == SENSOR2)
		qmerge_data_addr = g_iva_sensor2_qmerge_data_addr;
	else if (sensor_id == SENSOR3)
		qmerge_data_addr = g_iva_sensor3_qmerge_data_addr;

	return qmerge_data_addr;
}

u32 common_get_shading_data_addr_by_sensor_id(u8 sensor_id)
{
	u32 shading_data_addr = 0;

	if (sensor_id == SENSOR1)
		shading_data_addr = g_iva_sensor1_shading_data_addr;
	else if (sensor_id == SENSOR2)
		shading_data_addr = g_iva_sensor2_shading_data_addr;
	else if (sensor_id == SENSOR3)
		shading_data_addr = g_iva_sensor3_shading_data_addr;

	return shading_data_addr;
}

void isp_drv_init(void *pri_data)
{
	reset_dump_info();
	g_kva_share_buf_ptr = NULL;
	g_isp_extend_buf_size = 0;
	g_isp_high_quality_mode_buf_size = 0;
	log_task = NULL;
	g_open_count = 0;
	errcode = 0;
	g_enable_irq_flag = false;
	g_uc_sensor_1_scenario_id = 0;
	g_uc_sensor_2_scenario_id = 0;
	g_uc_sensor_3_scenario_id = 0;
	g_ucFormat[SENSOR1][V4L2_ISP_IMG1] = V4L2_ISP_OUTPUT_YUY2;
	g_ucFormat[SENSOR1][V4L2_ISP_IMG2] = V4L2_ISP_OUTPUT_YUY2;
	g_ucFormat[SENSOR1][V4L2_ISP_IMG3] = V4L2_ISP_OUTPUT_YUY2;
	g_ucFormat[SENSOR2][V4L2_ISP_IMG1] = V4L2_ISP_OUTPUT_YUY2;
	g_ucFormat[SENSOR2][V4L2_ISP_IMG2] = V4L2_ISP_OUTPUT_YUY2;
	g_ucFormat[SENSOR2][V4L2_ISP_IMG3] = V4L2_ISP_OUTPUT_YUY2;
	g_ucFormat[SENSOR3][V4L2_ISP_IMG1] = V4L2_ISP_OUTPUT_YUY2;
	g_ucFormat[SENSOR3][V4L2_ISP_IMG2] = V4L2_ISP_OUTPUT_YUY2;
	g_ucFormat[SENSOR3][V4L2_ISP_IMG3] = V4L2_ISP_OUTPUT_YUY2;
	isp_irq_enable = false;
	irq_rear_handle = NULL;
	irq_front_handle = NULL;
	g_sensor_1_scenario_id_check = 0;
	g_sensor_2_scenario_id_check = 0;
	g_sensor_3_scenario_id_check = 0;
	g_uc_raw_stream_on = 0;
	memset((u8 *) g_stop_path_field, 0, sizeof(g_stop_path_field));
	common_reset_bist_frame_done_cnt();
}

void isp_drv_deinit(void *pri_data)
{
	int i;

#if 0
	/* for auto dump, do NOT reset address on closing */
	reset_dump_info();
#endif

	g_iva_u32_addr_isp_extend_buff = 0;
	g_iva_u32_addr_isp_high_quality_mode_buff = 0;
	g_iva_firmware_log_addr = 0;
	/* manual start */
	memset(g_iva_manual_data_addr, 0x0,
		sizeof(g_iva_manual_data_addr));
	/* motion start */
	memset(g_iva_motion_addr, 0x0,
			sizeof(g_iva_motion_addr));
	/* scenario info */
	g_iva_ap_t_scenarioInfo_addr = 0;
	/* 3a setting */
	memset(g_iva_ap_t_dld_seq_info_addr, 0x0,
			sizeof(g_iva_ap_t_dld_seq_info_addr));
	memset(g_iva_ap_t_cfg_3a_info_addr, 0x0,
			sizeof(g_iva_ap_t_cfg_3a_info_addr));
	/* digital room */
	memset(g_iva_ap_t_dzoom_info_addr, 0x0,
			sizeof(g_iva_ap_t_dzoom_info_addr));
	g_iva_exbuf_info_addr = 0;
	/* debug info */
	memset(g_iva_debug_iq_info_addr, 0x0,
		sizeof(g_iva_debug_iq_info_addr));

	g_isp_extend_buf_size = 0;
	g_isp_high_quality_mode_buf_size = 0;
	g_uc_sensor_1_scenario_id = 0;
	g_uc_sensor_2_scenario_id = 0;
	g_uc_sensor_3_scenario_id = 0;
	g_uc_raw_stream_on = 0;
	g_iva_isp_fw_buf_addr = 0;
	g_iva_isp_working_buf_addr = 0;
	g_iva_isp_share_buf_addr = 0;
	g_sensor_1_scenario_id_check = 0;
	g_sensor_2_scenario_id_check = 0;
	g_sensor_3_scenario_id_check = 0;
	common_reset_bist_frame_done_cnt();

	memset((u8 *) g_ready_buffer, 0, sizeof(g_ready_buffer));
	memset((u8 *) g_dequeue_buff_index, 0, sizeof(g_dequeue_buff_index));
	memset((u8 *) g_enqueue_buff_index, 0, sizeof(g_enqueue_buff_index));
	/* CBC deinit */
	memset(g_iva_cbc_map_addr, 0x0, sizeof(g_iva_cbc_map_addr));
	/* stop paths*/
	for (i = SENSOR1; i < SENSOR_TOTAL; i++) {
		if (g_stop_path_field[i])
			isp_set_field_value(g_stop_path_field[i], 1);
	}

	if (g_kva_drv_buff_ptr)
		UNMAP_IVA_TO_KVA(g_kva_drv_buff_ptr);
	if (g_kva_manual_ptr[0]) {
		for (i = 0; i < SENSOR_TOTAL; i++)
			UNMAP_IVA_TO_KVA(g_kva_manual_ptr[i]);
	}
	if (g_kva_motion_ptr[0]) {
		for (i = 0; i < SENSOR_TOTAL; i++)
			UNMAP_IVA_TO_KVA(g_kva_motion_ptr[i]);
	}
	if (g_kva_dzoom_info_ptr[0]) {
		for (i = 0; i < SENSOR_TOTAL; i++)
			UNMAP_IVA_TO_KVA(g_kva_dzoom_info_ptr[i]);
	}
	if (g_kva_debug_iq_info_ptr[0]) {
		for (i = 0; i < SENSOR_TOTAL; i++)
			UNMAP_IVA_TO_KVA(g_kva_debug_iq_info_ptr[i]);
	}
	if (g_kva_scenario_info_ptr)
		UNMAP_IVA_TO_KVA(g_kva_scenario_info_ptr);
	if (g_kva_exbuf_info_ptr)
		UNMAP_IVA_TO_KVA(g_kva_exbuf_info_ptr);
#ifdef DEBUG_LOAD_QMERGE_AND_SHADING_BINARY
	g_iva_isp_qmerge_buf_addr = 0;
	g_iva_isp_shading_buf_addr = 0;
#endif
}
/*
 * /////////////////////////////////////////////////////////
 *		Private Function		*
 * /////////////////////////////////////////////////////////
*/
/*
*\brief ISP interrupt callback function
*\param none
*\return INT state, refer to ISPDrv_INT.(System/Sensor1/Sensor2/Sensor3)
*/
static u32 check_irq_flag(void)
{
	u32 err = 0;
	u32 int_state = 0;
	u32 system_int = 0, status_int = 0, sensor1_int = 0, sensor2_int =
	    0, sensor3_int = 0;
	u32 idx = 0;
	u16 entryId = 0;

	/*  struct timeval after;*/
	/* long long usec;*/

	isp_debug_lo_start_tag();

	if (g_enable_irq_flag) {
		/* check interrupt related fields*/

		system_int = isp_get_entry_value(AHB_ENTRY_INT_SYS_STATUS);
		int_state = V4L2_ISP_INT_NONE | system_int;

		if ((V4L2_ISP_SYS_IP_ERROR_INT | V4L2_ISP_SYS_CMD_FINISH_INT) &
		    system_int) {
			isp_set_entry_value(AHB_ENTRY_INT_SYS_STATUS, 0);
		}

		if (V4L2_ISP_SYS_IP_ERROR_INT & system_int) {
			err = isp_get_field_value(AHB_ITEM_SYSTEM_MESSAGE);
			system_int &= ~V4L2_ISP_SYS_IP_ERROR_INT;
		}

		/* spin_lock_irq(&system_int_lock); TODO: is it necessary? */
		g_latest_system_int |= system_int;
		/* spin_unlock_irq(&system_int_lock); TODO: is it necessary? */


		for (idx = 0, entryId = AHB_ENTRY_INT_SENSOR_1_IMG1_STATUS;
#if 0
		     entryId <= AHB_ENTRY_INT_SENSOR_1_3A_LINEMEET_STATUS;
#endif
				entryId <= AHB_ENTRY_INT_SENSOR_1_RAW_STATUS;
		     idx++, entryId++) {
			status_int = isp_get_entry_value(entryId);
			/* clear interrupt related fields*/
			if (status_int) {
				sensor1_int |= (status_int << idx);
				isp_set_entry_value(entryId, 0);
			}
		}

		for (idx = 0, entryId = AHB_ENTRY_INT_SENSOR_2_IMG1_STATUS;
		     entryId <= AHB_ENTRY_INT_SENSOR_2_3A_LINEMEET_STATUS;
		     idx++, entryId++) {
			status_int = isp_get_entry_value(entryId);
			/* clear interrupt related fields*/
			if (status_int) {
				sensor2_int |= (status_int << idx);
				isp_set_entry_value(entryId, 0);
			}
		}

		for (idx = 0, entryId = AHB_ENTRY_INT_SENSOR_3_IMG_STATUS;
		     entryId <= AHB_ENTRY_INT_SENSOR_3_3A_LINEMEET_STATUS;
		     idx++, entryId++) {
			status_int = isp_get_entry_value(entryId);
			/* clear interrupt related fields*/
			if (status_int) {
				sensor3_int |= (status_int << idx);
				isp_set_entry_value(entryId, 0);
			}
		}

		/* set interrupt bits, refer to ISPDrv_INT*/
		int_state |= (sensor1_int << SHIFT_SYS_MAX);
		int_state |= (sensor2_int << SHIFT_SENSOR1_MAX);
		int_state |= (sensor3_int << SHIFT_SENSOR2_MAX);

		wake_up(&WAITQ);

		if (err)
			isp_err_item_tag("INT: %#x, isp fw err: %#x,",
					 int_state, err);
		else
			isp_debug_item_tag("INT: %#x", int_state);

		/* isp_alert("%s - end", __func__);*/
		/* usec = after.tv_sec*1000LL + after.tv_usec/1000;*/
		/* isp_info("%s - end time: %lld", __func__, usec);*/
	}

	isp_debug_lo_end_tag();

	return int_state;
}

/*
 *\brief
 *\param ahb_buf_size [in]
 *\param cur_offset [in] print log until this position
 *\param p_pre_offset [in] start position
 */
static void print_fw_log(const u32 ahb_buf_size,
		u32 cur_offset, u32 *p_pre_offset)
{
	char output_str[BUFFER_SIZE];
	const char *delim = "\n";
	char *token;
	char *p;
	u32 offset = 0, offset2 = 0;
	u32 copy_size = 0;
	u8 *buff_ptr;

	if (cur_offset == 0 || cur_offset >= ahb_buf_size) /* out of range */
		return;
	if (cur_offset == *p_pre_offset) /* no update */
		return;

	/* remove the start, end tags to reduce the number of logs */
	/* isp_alert_lo_start_tag(); */
	if (*p_pre_offset < cur_offset) /* new log added */
		offset = *p_pre_offset;
	while (offset < cur_offset) {
		buff_ptr = g_kva_share_buf_ptr;
		if (!buff_ptr) {
			isp_info_desc_tag("fw log isp closed: %p", buff_ptr);
			break;
		}
		copy_size = cur_offset - offset;
		if (copy_size > BUFFER_SIZE)
			copy_size = BUFFER_SIZE;
		memset(output_str, 0, sizeof(output_str));

		/* Force to turn on firmware log for debugging */
		if (fwlog_force_switch) {
			memcpy(output_str, (buff_ptr + offset), copy_size);
		} else {
			if (g_kva_share_buf_valid)
				memcpy(output_str, (buff_ptr + offset),
					copy_size);
			else
				return;
		}
		p = output_str;
		offset2 = 0;
		do {
			token = strsep(&p, delim);
			if (p != NULL) {
				offset2 = (p - output_str);
				isp_info_desc_tag("fw log %s", token);
			} else if (offset2 == 0) {
				/* can not find a complete line in the buffer.
				 * just print it out. (include the last time)
				 */
				isp_info_desc_tag("fw log %s", output_str);
				offset2 = copy_size;
			} else if ((offset + copy_size) >= cur_offset) {
				/* the last time, print the last token */
				if (token != NULL && strlen(token) > 0)
					isp_info_desc_tag("fw log %s", token);
				offset2 = copy_size;
			}
			/* Others are strings which is possibly truncated
			 * rather than a complete whole line when p is null.
			 */
#if 0
			isp_pr_debug("pos: %p, off: %d %d, token: %s, len:%zd",
					p, offset, offset2,
					token, strlen(token));
#endif
		} while (p != NULL);
		offset += offset2;
	}
	*p_pre_offset = cur_offset;

	/* remove the start, end tags to reduce the number of logs */
	/* isp_info_lo_end_tag(); */
}

/* t_his task is used to log messages from isp*/
static int log_task_func(void *data)
{
	u32 err = 0;
	u32 previous_offset = 0, current_offset = 0;
#if 0
	u16 previous_offset = PAGE_SIZE_4K, current_offset = 0;
	u8 *buff_ptr;
	u16 copy_size = 0;
	char output_str[BUFFER_SIZE];

	buff_ptr = g_kva_share_buf_ptr;
#endif

	g_log_task_working = 1;

	do {
#if 1 /* FIXME: Use fwlog_threshold to control instead of ... */
		if (get_isplog_flag() == 0) {
			SLEEPRANGE(1000000, 1000000);
			continue;
		}
#endif

		/* polling the reg*/
		SLEEPRANGE(25000, 25500);
		/* SLEEPRANGE(10000, 10500);*/

		current_offset =
		    isp_get_field_value(AHB_ITEM_LOG_BUF_WRITE_POINTER);
		print_fw_log(PAGE_SIZE_4K, current_offset, &previous_offset);
#if 0
		if ((current_offset != previous_offset) && (current_offset != 0)
		    && (current_offset < PAGE_SIZE_4K)) {
			memset((u8 *) output_str, 0, sizeof(output_str));
			/*  log*/

			/*  previous > current*/
			if (previous_offset > current_offset) {
				copy_size = PAGE_SIZE_4K - previous_offset;

				if (copy_size < BUFFER_SIZE) {
					MEMCPY((u8 *) output_str,
					       (u8 *) (buff_ptr +
						       previous_offset + 1),
					       copy_size);
					memset((u8 *) output_str, 0,
					       sizeof(output_str));

					/*only copy the buffer max size*/
					if (current_offset < BUFFER_SIZE) {
						copy_size = current_offset + 1;

						/*  update the offset*/
						previous_offset =
						    current_offset;
					} else {
						copy_size = BUFFER_SIZE - 1;

						/*  update the offset*/
						previous_offset = copy_size;
					}
					MEMCPY((u8 *) output_str,
					       (u8 *) buff_ptr, copy_size);
				} else {
					copy_size = BUFFER_SIZE - 1;
					MEMCPY((u8 *) output_str,
					       (u8 *) (buff_ptr +
						       previous_offset + 1),
					       copy_size);

					/*  update the offset*/
					previous_offset += copy_size;
				}
				/*  print the log*/
				isp_info_lo_combo_desc_tag(
					"fw log %s", output_str);
			} else {	/*  current > previous*/
				if ((current_offset - previous_offset) <=
				    BUFFER_SIZE) {
					copy_size =
					    current_offset - previous_offset;
					MEMCPY((u8 *) output_str,
					       (u8 *) (buff_ptr +
						       previous_offset + 1),
					       copy_size);

					/*  update the offset*/
					previous_offset = current_offset;
				} else {
					copy_size = BUFFER_SIZE - 1;
					MEMCPY((u8 *) output_str,
					       (u8 *) (buff_ptr +
						       previous_offset + 1),
					       copy_size);

					/*  update the offset*/
					previous_offset += copy_size;
				}
				/*  print the log*/
				isp_info_lo_combo_desc_tag(
					"fw log %s", output_str);
			}

		}
#endif
	} while (!kthread_should_stop());

	g_log_task_working = 0;

	if (errcode == 0 && err != 0)
		errcode = err;

	return err;
}


#ifdef TESTISP
static void test_isp(void)
{
	/* ISPDrv_DumpAHBReg();*/
	/* ISPDrv_DumpFirmwareMemory();*/
	/* dump_raw();*/
	dump_output_img();
}
#endif

/*
*\brief Get external clock information for debug purpose
*\	  Note: the register address is defined by chip providor
*\return err
*/
/* TO DO ---- Set the clock addr*/
#define CLOCK_REG_BASE_ADDR	 (0xD8390000)
#define CSI_ICLK_VAL_ADDR	(0x9058)	/*  CSI_I0_CLOCK_DIVIDER_VAL*/
#define CSI_LCLK_VAL_ADDR	(0x905C)	/*  CSI_L_CLOCK_DIVIDER_VAL*/
#define ISP_ICLK_VAL_ADDR	(0x9060)	/*  ISP_I0_CLOCK_DIVIDER_VAL*/
#define ISP_LCLK_VAL_ADDR	(0x9064)	/*  ISP_L_CLOCK_DIVIDER_VAL*/
#define ISP_MCLK_VAL_ADDR	(0x9068)	/*  ISP_M_CLOCK_DIVIDER_VAL*/
#define ISP_PCLK_VAL_ADDR	(0x906C)	/*  ISP_P_CLOCK_DIVIDER_VAL*/
#define CLOCK_SOURCE_FREQ	(1200)	/*  1200MHz*/
/* End of TO DO*/

static void turn_on_ready_buffer(int flags)
{
	if (flags & V4L2_ISP_SENSOR1_OUT_IMG1_DONE_INT)
		g_ready_buffer[SENSOR1 * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       V4L2_ISP_IMG1] = 1;
	if (flags & V4L2_ISP_SENSOR1_OUT_IMG2_DONE_INT)
		g_ready_buffer[SENSOR1 * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       V4L2_ISP_IMG2] = 1;
	if (flags & V4L2_ISP_SENSOR1_OUT_IMG3_DONE_INT)
		g_ready_buffer[SENSOR1 * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       V4L2_ISP_IMG3] = 1;
	if (flags & V4L2_ISP_SENSOR1_3A_DONE_INT)
		g_ready_buffer[SENSOR1 * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       V4L2_ISP_STATITISTICS] = 1;
	if (flags & V4L2_ISP_SENSOR1_AF_DONE_INT)
		g_ready_buffer[SENSOR1 * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       V4L2_ISP_STATITISTICS_OF_AF] = 1;
	if (flags & V4L2_ISP_SENSOR1_RAW_DONE_INT)
		g_ready_buffer[IDX_READY_BUF_SENSOR1_RAW] = 1;

	if (flags & V4L2_ISP_SENSOR2_OUT_IMG1_DONE_INT)
		g_ready_buffer[SENSOR2 * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       V4L2_ISP_IMG1] = 1;
	if (flags & V4L2_ISP_SENSOR2_OUT_IMG2_DONE_INT)
		g_ready_buffer[SENSOR2 * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       V4L2_ISP_IMG2] = 1;
	if (flags & V4L2_ISP_SENSOR2_OUT_IMG3_DONE_INT)
		g_ready_buffer[SENSOR2 * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       V4L2_ISP_IMG3] = 1;
	if (flags & V4L2_ISP_SENSOR2_3A_DONE_INT)
		g_ready_buffer[SENSOR2 * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       V4L2_ISP_STATITISTICS] = 1;
	if (flags & V4L2_ISP_SENSOR2_AF_DONE_INT)
		g_ready_buffer[SENSOR2 * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       V4L2_ISP_STATITISTICS_OF_AF] = 1;

	if (flags & V4L2_ISP_SENSOR3_OUT_IMG_DONE_INT)
		g_ready_buffer[SENSOR3 * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       V4L2_ISP_IMG1] = 1;
	if (flags & V4L2_ISP_SENSOR3_3A_DONE_INT)
		g_ready_buffer[SENSOR3 * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       V4L2_ISP_STATITISTICS] = 1;
	if (flags & V4L2_ISP_SENSOR3_AF_DONE_INT)
		g_ready_buffer[SENSOR3 * V4L2_ISP_OUTPUT_IMAGE_TOTAL +
			       V4L2_ISP_STATITISTICS_OF_AF] = 1;

}

static u32 request_ex_buffer(u8 a_uc_scenario_id)
{
	u32 err = 0;

	isp_info_lo_start_tag();

	if (V4L2_ISP_SCENARIO_PREVIEW_STILL_SS ==
			a_uc_scenario_id) {
		g_kva_exbuf_info_ptr->udExBufAddr =
		    g_iva_u32_addr_isp_extend_buff;
		g_kva_exbuf_info_ptr->udExBufSize = g_isp_extend_buf_size;
		g_kva_exbuf_info_ptr->udHighQaulityBufAddr =
		    g_iva_u32_addr_isp_high_quality_mode_buff;
		g_kva_exbuf_info_ptr->udHighQaulityBufSize =
		    g_isp_high_quality_mode_buf_size;

		isp_info_item_tag("EXbuf addr: 0x%x, size: 0x%x",
			 g_kva_exbuf_info_ptr->udExBufAddr,
			 g_kva_exbuf_info_ptr->udExBufSize);
		isp_info_item_tag("HQ buf addr: 0x%x, size: 0x%x",
			 g_kva_exbuf_info_ptr->udHighQaulityBufAddr,
			 g_kva_exbuf_info_ptr->udHighQaulityBufSize);
	} else {
		g_kva_exbuf_info_ptr->udExBufAddr =
		    g_iva_u32_addr_isp_extend_buff;
		g_kva_exbuf_info_ptr->udExBufSize = g_isp_extend_buf_size;
		isp_info_item_tag("EXbuf addr: 0x%x, size: 0x%x",
			 g_kva_exbuf_info_ptr->udExBufAddr,
			 g_kva_exbuf_info_ptr->udExBufSize);
	}

	isp_info_lo_end_tag();

	return err;
}

static void release_ex_buffer(void)
{
	/*  Reset working extension buf info*/
	memset((u8 *) g_kva_exbuf_info_ptr, 0,
	       sizeof(struct ispdrv_exbuffer_info));
}

static u8 chk_img_output_format(u8 format)
{
	switch (format) {
	case V4L2_ISP_OUTPUT_RAW10:
	case V4L2_ISP_OUTPUT_NV12:
	case V4L2_ISP_OUTPUT_YUY2:
		return 0;
	default:
		return 1;
	}
}

static u8 set_img_output_format(u8 sensor, u8 img_type, u8 format)
{
	u8 err = 0;

	err = chk_img_output_format(format);
	if (!err)
		g_ucFormat[sensor][img_type] = format;
	return err;
}

static u8 chk_raw_output_format(u8 format)
{
	switch (format) {
	case V4L2_ISP_OUTPUT_RAW10:
	case V4L2_ISP_OUTPUT_ALTEK_RAW10:
		return 0;
	default:
		return 1;
	}
}

static u8 set_raw_output_format(u8 sensor, u8 img_type, u8 format)
{
	u8 err = 0;

	err = chk_raw_output_format(format);
	return err;
}

/*
 * \brief check if format is valid in its image type
 * \return non-zero if invalid
 */
static u8 chk_output_format(u8 img_type, u8 format)
{
	switch (img_type) {
	case V4L2_ISP_IMG1:
	case V4L2_ISP_IMG2:
	case V4L2_ISP_IMG3:
		return chk_img_output_format(format);
	case V4L2_ISP_RAW:
		return chk_raw_output_format(format);
	case V4L2_ISP_STATITISTICS:
	case V4L2_ISP_STATITISTICS_OF_AF:
		return 0;
	default:
		return 1;
	}
}

/*
 * \brief Save output format into a global variable
 *        based on which image type it is.
 *        Refer to ispdrv_output_type,
 *        img_type = IMG[1-3], RAW10/NV12/YUY2
 *        img_type = RAW, Unpack RAW10/Altek RAW10
 * \param uc_sensor [in]. See ispdrv_output_image.
 * \param img_type [in]. See ispdrv_output_image.
 * \param format [in]. See ispdrv_output_type
 */
static u8 set_output_format(u8 sensor, u8 img_type, u8 format)
{
	u8 err = 0;
	u8 (*fp)(u8, u8, u8) = NULL;

	switch (img_type) {
	case V4L2_ISP_IMG1:
	case V4L2_ISP_IMG2:
	case V4L2_ISP_IMG3:
		fp = set_img_output_format;
		break;
	case V4L2_ISP_RAW:
		fp = set_raw_output_format;
		break;
	default:
		err = 1;
		break;
	}
	if (fp)
		err = (*fp)(sensor, img_type, format);
	return err;
}

/*
 * \brief Inner function of
 *        ispdrv_set_output_buff_format
 * \param a_uc_sensor [in], sensor ID
 * \param a_uc_output_image [in], must be RAW type.
 *                                See ispdrv_output_image.
 * \param a_ucFormat[in], two formats could be chosen.
 *                        unpack RAW10/ALTEK RAW10
 *                        See ispdrv_output_type.
 * \param a_ucBufferNumber [in], buffer number
 * \param a_pt_output_addr_setting [in], image addresses
 */
static u32 set_output_buff_raw_format(u8 a_uc_sensor,
		u8 a_uc_output_image,
		u8 a_uc_format,
		u8 a_uc_buffer_number,
		struct output_addr_setting
				*a_pt_output_addr_setting)
{
	u32 err = 0;
	bool b_send_ahb_cmd = 0;
	u32 command_system_reg = 0;
	u32 command_sensor_addr_reg = 0;
	u32 command_chg_addrs_img_reg = 0;
	u32 value_out_format_reg = 0;
	u32 value_out_buf_num_reg = 0;
	u32 value_out_add1_reg = 0,
		value_out_add2_reg = 0,
		value_out_add3_reg = 0,
		value_out_add4_reg = 0;

	isp_info_lo_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_lo_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_output_buff_raw_format_end;
	}

	if (set_output_format(a_uc_sensor, a_uc_output_image, a_uc_format)) {
		err = EINVAL;
		goto set_output_buff_raw_format_semapost;
	}
	g_buffer_number[a_uc_sensor][a_uc_output_image] = a_uc_buffer_number;

	if (a_uc_sensor == SENSOR1) {
		b_send_ahb_cmd = true;
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_addr_reg =
				AHB_ITEM_CHANGE_SENSOR1_OUT_ADDR;
		command_chg_addrs_img_reg =
				AHB_ITEM_CHANGE_SENSOR1_OUT_ADDR_IMG;
		value_out_format_reg = AHB_ITEM_SENSOR1_RAW_OUT_TYPE;
		value_out_buf_num_reg =
				AHB_ITEM_SENSOR1_OUT_RAW_BUF_NUM;
		value_out_add1_reg =
				AHB_ITEM_SENSOR1_OUT_RAW_ADD1;
		value_out_add2_reg =
				AHB_ITEM_SENSOR1_OUT_RAW_ADD2;
		value_out_add3_reg =
				AHB_ITEM_SENSOR1_OUT_RAW_ADD3;
		value_out_add4_reg =
				AHB_ITEM_SENSOR1_OUT_RAW_ADD4;
	}
	if (!b_send_ahb_cmd) {
		isp_warn_lo_combo_desc_tag("unsent ahb cmd.");
		goto set_output_buff_raw_format_semapost;
	}

	/* raise sensor settings changed */
	isp_set_field_value(command_system_reg, 1);

	/* relevant changed settings */
	isp_set_field_value(value_out_format_reg, a_uc_format);
	isp_set_field_value(value_out_buf_num_reg,
			a_uc_buffer_number);

	/* notify address changed and indicate which img type */
	isp_set_field_value(command_sensor_addr_reg, 1);
	isp_set_field_value(command_chg_addrs_img_reg,
			a_uc_output_image);
	isp_set_field_value(value_out_add1_reg,
			a_pt_output_addr_setting->ud_addr1);
	isp_set_field_value(value_out_add2_reg,
			a_pt_output_addr_setting->ud_addr2);
	isp_set_field_value(value_out_add3_reg,
			a_pt_output_addr_setting->ud_addr3);
	isp_set_field_value(value_out_add4_reg,
			a_pt_output_addr_setting->ud_addr4);

	err = command_start(__func__);

	/* reset command reg down */
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_addr_reg, 0);
	isp_set_field_value(command_chg_addrs_img_reg, 0);

set_output_buff_raw_format_semapost:
	SEMAPOST(get_command_semaphore());

set_output_buff_raw_format_end:

	if (errcode == 0 && err != 0)
		errcode = err;

	isp_info_lo_end_tag();

	return err;
}

static void clear_ahb_int_status(void)
{
	isp_set_entry_value(AHB_ENTRY_INT_SYS_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_IMG1_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_IMG2_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_IMG3_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_3A_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_ISP_SOF_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_3A_LINEMEET_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_STILL_PROC_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_1_RAW_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_IMG1_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_IMG2_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_IMG3_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_3A_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_ISP_SOF_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_2_3A_LINEMEET_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_IMG_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_3A_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_ISP_SOF_STATUS, 0);
	isp_set_entry_value(AHB_ENTRY_INT_SENSOR_3_3A_LINEMEET_STATUS, 0);
}

/*
 * \brief only support out raw in sensor 1 so far.
 *        return non-zero if unsupported
 * \param sensor_id [in]
 * \param out_img_type [in]
 * \return value larger than 0 means valid combination of
 *         img type and sensor id
 *
 */
static u8 check_raw_out_valid(u8 sensor_id, u8 out_img_type)
{
	u8 err = 0;

	if (out_img_type != V4L2_ISP_RAW)
		return err;
	switch (sensor_id) {
	case SENSOR1:
		break;
	default:
		err = 1;
		break;
	}
	return err;
}

/*
* This workround whale2 ISP(ARC) access register bug.
*/
int altek_isp_opened(void)
{
	if (g_open_count >= 1)
		return 1;

	return 0;
}
EXPORT_SYMBOL(altek_isp_opened);

#ifdef DEBUG_JTAG
void isp_arc_jtag_cfg(void)
{
	ulong base_address = 0;

	isp_info_lo_start_tag();

	if (get_isp_chip_id() == ASIC_CHIP_ID_G2V1) {
		base_address = (ulong) ioremap_nocache(0x402e0034, 0x10);
		isp_info_item_tag("jtg reg1: 0x%x",
			(__raw_readl((IO_PTR) base_address)));
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x03),
				 (IO_PTR) base_address);

		base_address = (ulong) ioremap_nocache(0x402a0000, 0x10);
		isp_info_item_tag("jtg reg2: 0x%x",
			(__raw_readl((IO_PTR) base_address)));
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x28),
				 (IO_PTR) base_address);

		base_address = (ulong) ioremap_nocache(0x402a0110, 0x10);
		isp_info_item_tag("jtg reg3: 0x%x",
			(__raw_readl((IO_PTR) base_address)));
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x00002020),
				 (IO_PTR) base_address);
		base_address = (ulong) ioremap_nocache(0x402a0114, 0x10);
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x00002020),
				 (IO_PTR) base_address);
		base_address = (ulong) ioremap_nocache(0x402a0118, 0x10);
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x00002020),
				 (IO_PTR) base_address);
		base_address = (ulong) ioremap_nocache(0x402a011c, 0x10);
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x00002020),
				 (IO_PTR) base_address);
		base_address = (ulong) ioremap_nocache(0x402a0120, 0x10);
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x00002020),
				 (IO_PTR) base_address);

	} else if (get_isp_chip_id() == ASIC_CHIP_ID_G2V3) {

		/* jtag select */
		base_address = (ulong) ioremap_nocache(0xe42e0034, 0x10);
		isp_info_item_tag("jtg reg1: 0x%x",
				(__raw_readl((IO_PTR) base_address)));
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x03),
					 (IO_PTR) base_address);

		/* pin gloable control 0xe42a0000*/
		base_address = (ulong) ioremap_nocache(0xe42a0010, 0x10);
		isp_info_item_tag("jtg reg2: 0x%x",
				(__raw_readl((IO_PTR) base_address)));
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x20),
					 (IO_PTR) base_address);

		/* pin gloable control 0xe42a0000*/
		base_address = (ulong) ioremap_nocache(0xe42a0184, 0x10);
		isp_info_item_tag("jtg reg3: 0x%x",
				(__raw_readl((IO_PTR) base_address)));
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x00000010),
					 (IO_PTR) base_address);
		base_address = (ulong) ioremap_nocache(0xe42a0188, 0x10);
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x00000010),
					 (IO_PTR) base_address);
		base_address = (ulong) ioremap_nocache(0xe42a018c, 0x10);
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x00000010),
					 (IO_PTR) base_address);
		base_address = (ulong) ioremap_nocache(0xe42a0194, 0x10);
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x00000010),
					 (IO_PTR) base_address);
		base_address = (ulong) ioremap_nocache(0xe42a0198, 0x10);
		__raw_writel((__raw_readl((IO_PTR) base_address) | 0x00000010),
					 (IO_PTR) base_address);

	} else
		isp_info_item_tag("Undefined chip");

	isp_info_lo_end_tag();
}
#endif

static void print_scenario_info(struct s_scenario_info_ap *scn_info)
{
	isp_info_lo_start_tag();

	isp_info_item_tag("sensor_info. bit_number: %d, clamp_level: %d,",
		scn_info->sensor_info.bit_number,
		scn_info->sensor_info.clamp_level);
	isp_info_item_tag("original_width: %d, original_height: %d,",
		scn_info->sensor_info.original_width,
		scn_info->sensor_info.original_height);
	isp_info_item_tag("crop_start_x: %d, crop_start_y: %d,",
		scn_info->sensor_info.crop_start_x,
		scn_info->sensor_info.crop_start_y);
	isp_info_item_tag("crop_end_x: %d, crop_end_y: %d,",
		scn_info->sensor_info.crop_end_x,
		scn_info->sensor_info.crop_end_y);
	isp_info_item_tag("format: %d, frame_rate: %d, line_time: %d,",
		scn_info->sensor_info.format,
		scn_info->sensor_info.frame_rate,
		scn_info->sensor_info.line_time);
	isp_info_item_tag("mirror_flip: %d, n_color_order: %d,",
		scn_info->sensor_info.mirror_flip,
		scn_info->sensor_info.n_color_order);
	isp_info_item_tag("sensor_mode: %d, sensor_module_type: %d,",
		scn_info->sensor_info.sensor_mode,
		scn_info->sensor_info.sensor_module_type);
	isp_info_item_tag("width: %d, height: %d,",
		scn_info->sensor_info.width,
		scn_info->sensor_info.height);
	isp_info_item_tag(
		"out_bypass_flag. bypass_lv: %d, bypass_video: %d,",
		scn_info->out_bypass_flag.bypass_lv,
		scn_info->out_bypass_flag.bypass_video);
	isp_info_item_tag("bypass_still: %d, bypass_metadata: %d,",
		scn_info->out_bypass_flag.bypass_still,
		scn_info->out_bypass_flag.bypass_metadata);
	isp_info_item_tag("bayerscl_out_width: %d, bayerscl_out_height: %d,",
		scn_info->bayerscl_out_info.bayerscl_out_width,
		scn_info->bayerscl_out_info.bayerscl_out_height);
	isp_info_item_tag("cbc_enabled: %d",
		scn_info->sensor_info.cbc_enabled);
	isp_info_item_tag("iq_param_idx_lv: %d",
		scn_info->iq_param_idx_info.iq_param_idx_lv);
	isp_info_item_tag("iq_param_idx_video: %d",
		scn_info->iq_param_idx_info.iq_param_idx_video);
	isp_info_item_tag("iq_param_idx_still: %d",
		scn_info->iq_param_idx_info.iq_param_idx_still);

	isp_info_lo_end_tag();
}

static void print_out_bypass_flag(u8 sensor_id,
		struct s_scinfo_out_bypassflg *bypassflg)
{
	isp_info_item_tag(
		"sns %d, bypass_lv: %d, bypass_video: %d,",
		sensor_id,
		bypassflg->bypass_lv,
		bypassflg->bypass_video);
	isp_info_item_tag("bypass_still: %d, bypass_metadata: %d,",
		bypassflg->bypass_still,
		bypassflg->bypass_metadata);
}

static u8 set_scenario_id_check(u8 scenario_id)
{
	u8 err = 0;

	switch (scenario_id) {
	case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_1:
	case V4L2_ISP_SCENARIO_PREVIEW_STRIPE:
	case V4L2_ISP_SCENARIO_PREVIEW_STILL_SS:
		if (g_sensor_1_scenario_id_check == 0)
			g_sensor_1_scenario_id_check = scenario_id;
		else
			err = 1;
		break;
	case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_2:
		if (g_sensor_2_scenario_id_check == 0)
			g_sensor_2_scenario_id_check = scenario_id;
		else
			err = 1;
		break;
	case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_LITE:
		if (g_sensor_3_scenario_id_check == 0)
			g_sensor_3_scenario_id_check = scenario_id;
		else
			err = 1;
		break;
	case V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT:
		if (g_sensor_1_scenario_id_check == 0 &&
			g_sensor_2_scenario_id_check == 0) {
			g_sensor_1_scenario_id_check = scenario_id;
			g_sensor_2_scenario_id_check = scenario_id;
		} else
			err = 1;
		break;
	default:
		break;
	}

	return err;
}


static bool chk_scenario_id(u8 scenario_id)
{
	bool is_pass = false;

	switch (scenario_id) {
	case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_1:
	case V4L2_ISP_SCENARIO_PREVIEW_STRIPE:
	case V4L2_ISP_SCENARIO_PREVIEW_STILL_SS:
		if (g_sensor_1_scenario_id_check == scenario_id)
			is_pass = true;
		break;
	case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_2:
		if (g_sensor_2_scenario_id_check == scenario_id)
			is_pass = true;
		break;
	case V4L2_ISP_SCENARIO_PREVIEW_SENSOR_LITE:
		if (g_sensor_3_scenario_id_check == scenario_id)
			is_pass = true;
		break;
	case V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT:
		if (g_sensor_1_scenario_id_check == scenario_id
		&& g_sensor_2_scenario_id_check == scenario_id)
			is_pass = true;
		break;
	default:
		break;
	}

	return is_pass;
}

static void clear_scenario_id_check(u8 sensor_id)
{
	if (sensor_id == SENSOR1) {
		if (g_sensor_1_scenario_id_check
		== V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT)
			g_sensor_2_scenario_id_check = 0;
		g_sensor_1_scenario_id_check = 0;
	} else if (sensor_id == SENSOR2) {
		if (g_sensor_2_scenario_id_check
		== V4L2_ISP_SCENARIO_PREVIEW_DUAL_SHOT)
			g_sensor_1_scenario_id_check = 0;
		g_sensor_2_scenario_id_check = 0;
	} else if (sensor_id == SENSOR3)
		g_sensor_3_scenario_id_check = 0;
}

static bool is_preview_stream_on(u8 sensor_id, u8 img_type)
{
	u32 value = 0;

	/* Get value from AHB */
	if (sensor_id == SENSOR1) {
		if (img_type == V4L2_ISP_IMG1)
			value
		= isp_get_field_value(AHB_ITEM_SENSOR1_OUT_IMG1_EN);
		else if (img_type == V4L2_ISP_IMG2)
			value
		= isp_get_field_value(AHB_ITEM_SENSOR1_OUT_IMG2_EN);
		else if (img_type == V4L2_ISP_IMG3)
			value
		= isp_get_field_value(AHB_ITEM_SENSOR1_OUT_IMG3_EN);
		else if (img_type == V4L2_ISP_STATITISTICS)
			value
		= isp_get_field_value(AHB_ITEM_SENSOR1_OUT_STATISTICS_EN);
		else if (img_type == V4L2_ISP_RAW)
			value
		= isp_get_field_value(AHB_ITEM_SENSOR1_RAW_FRAME_RATE);
	} else if (sensor_id == SENSOR2) {
		if (img_type == V4L2_ISP_IMG1)
			value
		= isp_get_field_value(AHB_ITEM_SENSOR2_OUT_IMG1_EN);
		else if (img_type == V4L2_ISP_IMG2)
			value
		= isp_get_field_value(AHB_ITEM_SENSOR2_OUT_IMG2_EN);
		else if (img_type == V4L2_ISP_IMG3)
			value
		= isp_get_field_value(AHB_ITEM_SENSOR2_OUT_IMG3_EN);
		else if (img_type == V4L2_ISP_STATITISTICS)
			value
		= isp_get_field_value(AHB_ITEM_SENSOR2_OUT_STATISTICS_EN);
	} else {
		if (img_type == V4L2_ISP_IMG1)
			value
		= isp_get_field_value(AHB_ITEM_SENSOR3_OUT_IMG1_EN);
		else if (img_type == V4L2_ISP_STATITISTICS)
			value
		= isp_get_field_value(AHB_ITEM_SENSOR3_OUT_STATISTICS_EN);
	}

	if (value != 0)
		return true;
	else
		return false;
}

static u32 init_queue_work(void)
{
	int err = 0;
	/* initial irq cnt */
	irq_cnt = 0;
	irq_3a_cnt = 0;
	irq_sof_cnt = 0;
	irq_lv_cnt = 0;
	irq_video_cnt = 0;
	irq_still_cnt = 0;
	irq_raw_cnt = 0;

	/* initial isp irq handler work queue*/
	isr_work.irq_work_queue = create_singlethread_workqueue(irq_work_name);
	if (!isr_work.irq_work_queue) {
		isp_err_pu_combo_desc_tag("irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("irq_work_queue init ok\n");
	}

	INIT_WORK(&isr_work.isq_work, irq_handle_work);

	isr_work_2nd.irq_work_queue =
		create_singlethread_workqueue(irq_work_name_2nd);
	if (!isr_work_2nd.irq_work_queue) {
		isp_err_pu_combo_desc_tag("2nd irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("2nd irq_work_queue init ok\n");
	}

	INIT_WORK(&isr_work_2nd.isq_work, irq_handle_work);

	isr_work_3a.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_3a);
	if (!isr_work_3a.irq_work_queue) {
		isp_err_pu_combo_desc_tag("3a irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("3a irq_work_queue init ok\n");
	}

	INIT_WORK(&isr_work_3a.isq_work, irq_handle_work);

	isr_work_3a_2nd.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_3a_2nd);
	if (!isr_work_3a_2nd.irq_work_queue) {
		isp_err_pu_combo_desc_tag("3a 2nd irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("2nd 3a irq_work_queue init ok\n");
	}
	INIT_WORK(&isr_work_3a_2nd.isq_work, irq_handle_work);

	isr_work_af.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_af);
	if (!isr_work_af.irq_work_queue) {
		isp_err_pu_combo_desc_tag("af irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("af irq_work_queue init ok\n");
	}

	INIT_WORK(&isr_work_af.isq_work, irq_handle_work);

	isr_work_af_2nd.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_af_2nd);
	if (!isr_work_af_2nd.irq_work_queue) {
		isp_err_pu_combo_desc_tag("af 2nd irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("2nd af irq_work_queue init ok\n");
	}
	INIT_WORK(&isr_work_af_2nd.isq_work, irq_handle_work);

	isr_work_sof.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_sof);
	if (!isr_work_sof.irq_work_queue) {
		isp_err_pu_combo_desc_tag("sof irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("sof irq_work_queue init ok\n");
	}
	INIT_WORK(&isr_work_sof.isq_work, irq_handle_work);

	isr_work_sof_2nd.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_sof_2nd);
	if (!isr_work_sof_2nd.irq_work_queue) {
		isp_err_pu_combo_desc_tag("2nd sof irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("2nd sof irq_work_queue init ok\n");
	}
	INIT_WORK(&isr_work_sof_2nd.isq_work, irq_handle_work);

	isr_work_lv.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_lv);
	if (!isr_work_lv.irq_work_queue) {
		isp_err_pu_combo_desc_tag("lv irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("lv irq_work_queue init ok\n");
	}
	INIT_WORK(&isr_work_lv.isq_work, irq_handle_work);

	/* initial isp irq handler work queue */
	isr_work_lv_2nd.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_lv_2nd);
	if (!isr_work_lv_2nd.irq_work_queue) {
		isp_err_pu_combo_desc_tag("2nd lv irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("2nd lv irq_work_queue init ok\n");
	}
	INIT_WORK(&isr_work_lv_2nd.isq_work, irq_handle_work);

	isr_work_video.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_video);
	if (!isr_work_video.irq_work_queue) {
		isp_err_pu_combo_desc_tag("video irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("video irq_work_queue init ok\n");
	}
	INIT_WORK(&isr_work_video.isq_work, irq_handle_work);

	/* initial isp irq handler work queue */
	isr_work_video_2nd.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_video_2nd);
	if (!isr_work_video_2nd.irq_work_queue) {
		isp_err_pu_combo_desc_tag("2nd still irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("2nd video irq_work_queue init ok\n");
	}
	INIT_WORK(&isr_work_video_2nd.isq_work, irq_handle_work);


	isr_work_still.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_still);
	if (!isr_work_still.irq_work_queue) {
		isp_err_pu_combo_desc_tag("still irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("still irq_work_queue init ok\n");
	}
	INIT_WORK(&isr_work_still.isq_work, irq_handle_work);

	/* initial isp irq handler work queue */
	isr_work_still_2nd.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_still_2nd);
	if (!isr_work_still_2nd.irq_work_queue) {
		isp_err_pu_combo_desc_tag("2nd still irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("2nd still irq_work_queue init ok\n");
	}
	INIT_WORK(&isr_work_still_2nd.isq_work, irq_handle_work);

	isr_work_raw.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_raw);
	if (!isr_work_raw.irq_work_queue) {
		isp_err_pu_combo_desc_tag("raw irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("raw irq_work_queue init ok\n");
	}
	INIT_WORK(&isr_work_raw.isq_work, irq_handle_work);

	/* initial isp irq handler work queue */
	isr_work_raw_2nd.irq_work_queue =
			create_singlethread_workqueue(irq_work_name_raw_2nd);
	if (!isr_work_raw_2nd.irq_work_queue) {
		isp_err_pu_combo_desc_tag("2nd raw irq_work_queue is null.");
		err = EINVAL;
		goto init_queue_work_end;
	} else {
		isp_info_desc_tag("2nd raw irq_work_queue init ok\n");
	}
	INIT_WORK(&isr_work_raw_2nd.isq_work, irq_handle_work);


init_queue_work_end:

	return err;

}

#ifdef DEBUG_LOAD_QMERGE_AND_SHADING_BINARY

static void overwrite_shading_and_qmerge_addr(void)
{
	g_iva_sensor1_qmerge_data_addr = g_iva_isp_qmerge_buf_addr;
	g_iva_sensor2_qmerge_data_addr =
	    g_iva_sensor1_qmerge_data_addr + MAX_QMERGE_BIN_SIZE;
	g_iva_sensor3_qmerge_data_addr =
	    g_iva_sensor2_qmerge_data_addr + MAX_QMERGE_BIN_SIZE;

	ispdrv_set_qmerge_addr(g_iva_sensor1_qmerge_data_addr,
			       g_iva_sensor2_qmerge_data_addr,
			       g_iva_sensor3_qmerge_data_addr);

	g_iva_sensor1_shading_data_addr = g_iva_isp_shading_buf_addr;
	g_iva_sensor2_shading_data_addr =
		g_iva_sensor1_shading_data_addr + MAX_SHADING_BIN_SIZE;
	g_iva_sensor3_shading_data_addr =
		g_iva_sensor2_shading_data_addr + MAX_SHADING_BIN_SIZE;

	ispdrv_set_shading_addr(
			g_iva_sensor1_shading_data_addr,
			g_iva_sensor2_shading_data_addr,
			g_iva_sensor3_shading_data_addr);
}


static u32 overwrite_shading_and_qmerge_bin(void *v4l2_device)
{
	u32 err = 0;
	const struct firmware *qmerge_ptr;
	const struct firmware *shading_ptr;

	/* Load Sensor 1 Shading bin file*/
	if (request_firmware
		(&shading_ptr, "TBM_Shading_0.bin",
		(struct device *)v4l2_device)) {
		isp_err_pu_combo_desc_tag("get Sensor 1 Shading failed.");
		errcode = err = EINVAL;
		goto overwrite_shading_and_qmerge_bin_end;
	}

	if (shading_ptr->size > MAX_SHADING_BIN_SIZE) {
		isp_err_pu_combo_desc_tag("Sensor 1 Shading size is too big.");
		errcode = err = EINVAL;
		goto overwrite_shading_and_qmerge_bin_end;
	}
	upload_data_file_mmu(g_iva_sensor1_shading_data_addr,
				(void *)shading_ptr->data, shading_ptr->size);
	release_firmware(shading_ptr);

	/*  Load Sensor 2 Shading bin file*/
	if (request_firmware
		(&shading_ptr, "TBM_Shading_1.bin",
		(struct device *)v4l2_device)) {
		isp_err_pu_combo_desc_tag("get Sensor 2 Shading failed.");
		errcode = err = EINVAL;
		goto overwrite_shading_and_qmerge_bin_end;
	}

	if (shading_ptr->size > MAX_SHADING_BIN_SIZE) {
		isp_err_pu_combo_desc_tag("Sensor 2 Shading size is too big.");
		errcode = err = EINVAL;
		goto overwrite_shading_and_qmerge_bin_end;
	}
	upload_data_file_mmu(g_iva_sensor2_shading_data_addr,
				(void *)shading_ptr->data, shading_ptr->size);
	release_firmware(shading_ptr);

	/*  Load Sensor 3 Shading bin file*/
	if (request_firmware
		(&shading_ptr, "TBM_Shading_2.bin",
		(struct device *)v4l2_device)) {
		isp_err_pu_combo_desc_tag("get Sensor 3 Shading failed.");
		errcode = err = EINVAL;
		goto overwrite_shading_and_qmerge_bin_end;
	}

	if (shading_ptr->size > MAX_SHADING_BIN_SIZE) {
		isp_err_pu_combo_desc_tag("Sensor 3 Shading size is too big.");
		errcode = err = EINVAL;
		goto overwrite_shading_and_qmerge_bin_end;
	}

	upload_data_file_mmu(g_iva_sensor3_shading_data_addr,
				(void *)shading_ptr->data, shading_ptr->size);
	release_firmware(shading_ptr);

	/* Load Sensor 1 Qmerge bin file*/
	if (request_firmware
	    (&qmerge_ptr, "TBM_Qmerge_Sensor1.bin",
	     (struct device *)v4l2_device)) {
		isp_err_pu_combo_desc_tag("get Sensor 1 Qmerge failed.");
		errcode = err = EINVAL;
		goto overwrite_shading_and_qmerge_bin_end;
	}

	if (qmerge_ptr->size > MAX_QMERGE_BIN_SIZE) {
		isp_err_pu_combo_desc_tag("Sensor 1 Qmerge size is too big.");
		errcode = err = EINVAL;
		goto overwrite_shading_and_qmerge_bin_end;
	}
	upload_data_file_mmu(g_iva_sensor1_qmerge_data_addr,
			     (void *)qmerge_ptr->data, qmerge_ptr->size);
	release_firmware(qmerge_ptr);

	/*  Load Sensor 2 Qmerge bin file*/
	if (request_firmware
	    (&qmerge_ptr, "TBM_Qmerge_Sensor2.bin",
	     (struct device *)v4l2_device)) {
		isp_err_pu_combo_desc_tag("get Sensor 2 Qmerge failed.");
		errcode = err = EINVAL;
		goto overwrite_shading_and_qmerge_bin_end;
	}

	if (qmerge_ptr->size > MAX_QMERGE_BIN_SIZE) {
		isp_err_pu_combo_desc_tag("Sensor 2 Qmerge size is too big.");
		errcode = err = EINVAL;
		goto overwrite_shading_and_qmerge_bin_end;
	}
	upload_data_file_mmu(g_iva_sensor2_qmerge_data_addr,
			     (void *)qmerge_ptr->data, qmerge_ptr->size);
	release_firmware(qmerge_ptr);

	/*  Load Sensor 3 Qmerge bin file*/

	if (request_firmware
	    (&qmerge_ptr, "TBM_Qmerge_Sensor3.bin",
	     (struct device *)v4l2_device)) {
		isp_err_pu_combo_desc_tag("get Sensor 3 Qmerge failed.");
		errcode = err = EINVAL;
		goto overwrite_shading_and_qmerge_bin_end;
	}

	if (qmerge_ptr->size > MAX_QMERGE_BIN_SIZE) {
		isp_err_pu_combo_desc_tag("Sensor 3 Qmerge size is too big.");
		errcode = err = EINVAL;
		goto overwrite_shading_and_qmerge_bin_end;
	}

	upload_data_file_mmu(g_iva_sensor3_qmerge_data_addr,
			     (void *)qmerge_ptr->data, qmerge_ptr->size);
	release_firmware(qmerge_ptr);

overwrite_shading_and_qmerge_bin_end:

	return err;
}
#endif

#ifdef DEBUG_LOAD_FW_BINARY
static u32 overwrite_firmware(void *v4l2_device)
{
	u32 err = 0;
	const struct firmware *fw;
	u32 src_crc = 0, dst_crc = 0;
	u32 copy_times = 0;

	/* TO DO ---- Upload firmware.*/
	if (request_firmware
	    (&fw, "TBM_G2v1DDR.bin", (struct device *)v4l2_device)) {
		isp_err_pu_combo_desc_tag("get firmware failed.");
		errcode = err = EINVAL;
		goto overwrite_firmware_end;
	}

	isp_info_item_tag(
		"g_kva_isp_fw_buff_addr: 0x%llx, fw->data: 0x%p, fw->size: 0x%lx",
		g_kva_isp_fw_buff_addr, fw->data, fw->size);

	src_crc = crc32(0xffffeeee, fw->data, fw->size);
	isp_info_item_tag("crc32 magic: 0x%x, 0x%x\n", 0xffffeeee, src_crc);

	while (src_crc != dst_crc) {
		MEMCPY((u8 *) g_kva_isp_fw_buff_addr,
			(void *)fw->data, fw->size);
		__dma_flush_range((void *)g_kva_isp_fw_buff_addr,
				  (void *)(g_kva_isp_fw_buff_addr + fw->size));
		dst_crc = crc32(0xffffeeee, g_kva_isp_fw_buff_addr, fw->size);
		copy_times++;
		isp_info_item_tag("src_crc: 0x%x, dst_crc: 0x%x, times: %d\n",
			src_crc, dst_crc, copy_times);
		if (copy_times >= 10)
			break;
	}

	isp_info_desc_tag("firmware load ok.\n");
	release_firmware(fw);


overwrite_firmware_end:

	return err;
}
#endif

static u32 allocate_shared_buffer_useage(u32 block_size)
{
	u32 err = 0;
	u32 share_buff_total_size = 1024 * 1024;

	/* Mapping the physical memory to kernel memory */
	g_kva_share_buf_ptr =
	    (u8 *) mmu_lock_isp_buf(g_iva_isp_share_buf_addr,
	    share_buff_total_size);
	g_kva_share_buf_valid = 1;

	err = memory_allocate_share_buff(g_iva_isp_share_buf_addr,
		&g_kva_share_buf_ptr,
		g_share_buff_iva_array,
		g_share_buff_kva_array,
		g_share_buff_size_array,
		share_buff_total_size,
		g_share_buff_mask);
	if (err) {
		isp_info_desc_tag("share buffer, memory allocation failed");
		goto allcate_shared_buffer_useage_end;
	} else {
		isp_info_desc_tag("share buffer, memory allocation success");
	}

allcate_shared_buffer_useage_end:

	return err;
}

/*
 * \brief Memory allocation for shared buffer
 * \param share_buff_start_iva [in]. the start address
 *		of the memory of shared buffer
 * \param share_buff_start_kva [in]. the start kernel address
 *		of the memory of shared buffer
 * \param iva_array [in]. Array of pointers
 *		The pointers point to the memory address
 *		of the really used global variables
 * \param kva_array [in]. Array of pointers of kernal addresses
 *		The pointers point to the memory address
 *		of the really used global kernal addresses
 * \param size_array [in]. Array of size of each item in the shared buffer
 * \param share_buff_total_size [in]. the total size of memory of shared buffer
 * \param mask [out]. to check the memory of the index of the shared buffer
 *		is allocated or not.
 * \return Error code
 */
static u32 memory_allocate_share_buff(u32 share_buff_start_iva,
	u8 **share_buff_start_kva,
	u32 *iva_array[ISP_SHARE_BUFF_INDEX_MAX],
	void **kva_array[ISP_SHARE_BUFF_INDEX_MAX],
	u32 *size_array,
	const u32 share_buff_total_size,
	u8 *mask)
{
	u32 err = 0;
	u32 cumulate_size = 0;
	u32 loop = 0;

	isp_info_lo_start_tag();

	init_share_buff_info_array();

	for (loop = ISP_SHARE_BUFF_INDEX_FIRMWARE_LOG_BUFF;
	loop < ISP_SHARE_BUFF_INDEX_MAX;
	loop++) {
		isp_debug_item_tag("share buffer, index:%d, mask:%d",
			loop, mask[loop]);
		err = allocate_share_buff_iva(share_buff_start_iva,
			iva_array, size_array, loop,
			&cumulate_size, share_buff_total_size, mask);
		if (err)
			goto set_share_buff_end;
		isp_debug_item_tag(
			"share buffer, mask:%d, addr:0x%x, cumulate_size:%d",
			mask[loop], *(iva_array[loop]), cumulate_size);
		if (loop == ISP_SHARE_BUFF_INDEX_IQ_FUNC) {
			ispdrv_set_iq_buffer_mem_info(*(iva_array[loop]),
				size_array[loop]);
			iq_info_init_func_info(share_buff_start_iva,
				*share_buff_start_kva);
		} else if (loop != ISP_SHARE_BUFF_INDEX_FIRMWARE_LOG_BUFF) {
			allocate_share_buff_kva(share_buff_start_iva,
				share_buff_start_kva,
				iva_array,
				kva_array,
				size_array,
				loop);
			initial_share_buff_info(loop, kva_array, size_array);
			isp_debug_item_tag("share buffer, kva:%p",
				*(kva_array[loop]));
		}
		set_share_buff_ahb(loop, iva_array, size_array, true);

	}

set_share_buff_end:
	if (err)
		isp_info_err(
			"share buffer allocation failed at index:%d, err:%d",
			loop, err);

	isp_info_lo_end_tag();

	return err;
}

/*
 * \brief Allocate memory for shared buffer
 *	by using pre-defined structures
 * \param start_addr [in]. the start address of the memory of shared buffer
 * \param addr_array [in]. Array of pointers
 *	The pointers point to the memory address
 *	of the really used global variables
 * \param size_array [in]. Array of size of each item in the shared buffer
 * \param index [in]. the index of shared buffer.
 * \param acc_size [out]. count for the total allocated size of memory
 *	to check if over used
 * \param share_buff_total_size [in]. the total size of memory of shared buffer
 * \param mask [out].
 *	to check the index of the shared buffer is memory-allocated or not.
 * \return Error code
 */
static u32 allocate_share_buff_iva(u32 start_addr,
	u32 *addr_array[ISP_SHARE_BUFF_INDEX_MAX],
	u32 *size_array,
	u32 index,
	u32 *acc_size,
	const u32 share_buff_total_size,
	u8 *mask)
{
	u32 err = 0;
	u32 *addr = addr_array[index];

	if (mask[index]) {
		err = EINVAL;
		goto allocate_share_buff_end;
	}
	if ((*acc_size + size_array[index]) > share_buff_total_size) {
		err = EINVAL;
		goto allocate_share_buff_end;
	}
	*addr = start_addr + *acc_size;
	/* Adjust the offset to 4 alignment
	 * for the limitation of memory domain
	 */
	*acc_size += ALIGN_CEIL(size_array[index], 4);

	mask[index] = true;
allocate_share_buff_end:

	return err;
}

/*
 * \brief Allocate kernel memory for shared buffer
 *		by using pre-defined structures
 * \param start_addr [in]. the start physical address
 *		of the memory of shared buffer
 * \param start_kva [in]. the start kernel address
 *		of the memory of shared buffer
 * \param addr_array [in]. Array of pointers
 *		The pointers point to the memory address
 *		of the really used global variables
 * \param kva_array [in]. Array of pointers of kernal addresses
 *		The pointers point to the memory address
 *		of the really used global kernal addresses
 * \param size_array [in]. Array of size of each item in the shared buffer
 * \param index [in]. the index of shared buffer.
 * \return Error code
 */
static void allocate_share_buff_kva(u32 start_iva, u8 **start_kva,
	u32 *addr_array[ISP_SHARE_BUFF_INDEX_MAX],
	void **kva_array[ISP_SHARE_BUFF_INDEX_MAX],
	u32 *size_array, u32 index)
{
	u32 *addr = addr_array[index];
	void **ptr = kva_array[index];

	*ptr = *start_kva + (*addr - start_iva);
}

/*
 * \brief Set the initial values to the memory of shared buffer
 * \param index [in]. the index of shared buffer.
 * \param kva_array [in]. Array of pointers of kernal addresses
 *		The pointers point to the memory address
 *		of the really used global kernal addresses
 * \param size_array [in]. Array of size of each item in the shared buffer
 * \return None
 */
static void initial_share_buff_info(u8 index,
	void **kva_array[ISP_SHARE_BUFF_INDEX_MAX],
	u32 *size_array)
{
	u8 loop = 0;
	void **ptr = kva_array[index];

	if (index == ISP_SHARE_BUFF_INDEX_SENSOR_1_BRIGHTNESS ||
		index == ISP_SHARE_BUFF_INDEX_SENSOR_2_BRIGHTNESS ||
		index == ISP_SHARE_BUFF_INDEX_SENSOR_3_BRIGHTNESS) {
		for (loop = 0; loop < BRIGHTNESS_ARRAY_SIZE; loop++)
			((struct ispdrv_brightness_gain_setting *)(*ptr))->
				gain[loop] = 100;
	} else if (index == ISP_SHARE_BUFF_INDEX_SENSOR_1_DGAIN ||
		index == ISP_SHARE_BUFF_INDEX_SENSOR_2_DGAIN ||
		index == ISP_SHARE_BUFF_INDEX_SENSOR_3_DGAIN) {
		((struct isp_d_gain_info *)(*ptr))->isp_d_gain_r = 10000;
		((struct isp_d_gain_info *)(*ptr))->isp_d_gain_gr = 10000;
		((struct isp_d_gain_info *)(*ptr))->isp_d_gain_gb = 10000;
		((struct isp_d_gain_info *)(*ptr))->isp_d_gain_b = 10000;
	} else {
		memset((u8 *)(*ptr), 0, size_array[index]);
		if (index == ISP_SHARE_BUFF_INDEX_SCENARIO_INFO)
			((struct s_scenario_info_isp *)(*ptr))->ul_magic_num =
				SCINFO_MAGICNUM;
		else if (index == ISP_SHARE_BUFF_INDEX_SENSOR_1_CFG_3A_INFO ||
			index == ISP_SHARE_BUFF_INDEX_SENSOR_2_CFG_3A_INFO ||
			index == ISP_SHARE_BUFF_INDEX_SENSOR_3_CFG_3A_INFO) {
			((struct cfg_3a_info *)(*ptr))->magic_num =
				((AP3A_VER_YEAR << AP3A_VER_YEAR_SHIFT) &
				AP3A_VER_YEAR_MASK);
			((struct cfg_3a_info *)(*ptr))->magic_num +=
				((AP3A_VER_MONTH << AP3A_VER_MONTH_SHIFT) &
				AP3A_VER_MONTH_MASK);
			((struct cfg_3a_info *)(*ptr))->magic_num +=
				((AP3A_VER_DAY << AP3A_VER_DAY_SHIFT) &
				AP3A_VER_DAY_MASK);
			if (index == ISP_SHARE_BUFF_INDEX_SENSOR_1_CFG_3A_INFO)
				isp_info_item_tag("Sensor: 1");
			else if (index ==
				ISP_SHARE_BUFF_INDEX_SENSOR_2_CFG_3A_INFO)
				isp_info_item_tag("Sensor: 2");
			else if (index ==
				ISP_SHARE_BUFF_INDEX_SENSOR_3_CFG_3A_INFO)
				isp_info_item_tag("Sensor: 3");
			isp_info_item_tag(
				"statistics version: 0x%08X",
				((struct cfg_3a_info *)(*ptr))->magic_num);
		}
	}
}

/*
 * \brief Set the address into AHB register
 * \param index [in]. the index of shared buffer.
 * \param addr_array [in]. Array of pointers
 *		The pointers point to the memory address
 *		of the really used global variables
 * \param size_array [in]. Array of size of each item in the shared buffer
 * \param enable [in]. The flag which is used to
 *		indicate to save the firmware log or not
 * \return None
 */
static void set_share_buff_ahb(u32 index,
	u32 *addr_array[],
	u32 *size_array,
	bool enable)
{
	u32 ahb_reg_1 = 0, ahb_reg_2 = 0;
	u32 *addr = addr_array[index];
	u32 size = size_array[index];
	bool match = true;

	switch (index) {
	case ISP_SHARE_BUFF_INDEX_FIRMWARE_LOG_BUFF:
		ahb_reg_1 = AHB_ITEM_LOG_BUF_ADDR;
		ahb_reg_2 = AHB_ITEM_LOG_BUF_SIZE;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_1_DGAIN:
		ahb_reg_1 = AHB_ITEM_SENSOR1_ISP_D_GAIN_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_2_DGAIN:
		ahb_reg_1 = AHB_ITEM_SENSOR2_ISP_D_GAIN_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_3_DGAIN:
		ahb_reg_1 = AHB_ITEM_SENSOR3_ISP_D_GAIN_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SCENARIO_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR1_CONFIG_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_1_DLD_SEQ_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR1_3ADLDSEQ_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_2_DLD_SEQ_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR2_3ADLDSEQ_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_3_DLD_SEQ_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR3_3ADLDSEQ_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_1_CFG_3A_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR1_CONFIG3A_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_2_CFG_3A_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR2_CONFIG3A_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_3_CFG_3A_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR3_CONFIG3A_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_1_DZOOM_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR1_DZOOMINFO_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_2_DZOOM_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR2_DZOOMINFO_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_3_DZOOM_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR3_DZOOMINFO_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_1_DEBUG_IQ_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR1_DEBUG_INFO_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_2_DEBUG_IQ_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR2_DEBUG_INFO_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_3_DEBUG_IQ_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR3_DEBUG_INFO_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_1_CCM_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR1_CCM_INFO_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_2_CCM_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR2_CCM_INFO_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_3_CCM_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR3_CCM_INFO_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_1_OTP_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR1_IQ_OTP_INFO_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_2_OTP_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR2_IQ_OTP_INFO_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_3_OTP_INFO:
		ahb_reg_1 = AHB_ITEM_SENSOR3_IQ_OTP_INFO_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_SENSOR_1_PROC_STILL_ARG:
		ahb_reg_1 = AHB_ITEM_SENSOR1_PROC_STILL_ADDR;
		break;
	case ISP_SHARE_BUFF_INDEX_EXTEND_BUFF_INFO:
		ahb_reg_1 = AHB_ITEM_WORK_EXT_INFO;
		break;
	default:
		match = false;
		break;
	}

	if (match) {
		if (index == ISP_SHARE_BUFF_INDEX_FIRMWARE_LOG_BUFF) {
			if (!enable) {
				*addr = 0;
				size = 0;
			}
			isp_set_field_value(ahb_reg_2, size);
		}
		isp_set_field_value(ahb_reg_1, *addr);
		isp_debug_item_tag(
			"share buffer, set ahb reg, index:%d, addr:0x%x",
			index, *addr);
	}
}

/*
 * \brief Allocate the size, memory address of physical global variables,
 *		memory address of kernel global variables
 * \param None
 * \return None
 */
static void init_share_buff_info_array(void)
{
	u8 loop = 0;
	u32 offset = 0;
	u32 index = 0;

	memset((u8 *)g_share_buff_mask, 0,
		sizeof(u8)*ISP_SHARE_BUFF_INDEX_MAX);

	/* Working Buffer for log */
	g_share_buff_size_array[ISP_SHARE_BUFF_INDEX_FIRMWARE_LOG_BUFF] =
		LOG_BUFFER_SIZE;
	g_share_buff_iva_array[ISP_SHARE_BUFF_INDEX_FIRMWARE_LOG_BUFF] =
		&g_iva_firmware_log_addr;
	g_share_buff_kva_array[ISP_SHARE_BUFF_INDEX_FIRMWARE_LOG_BUFF] = NULL;
	/* Manual data */
	index = ISP_SHARE_BUFF_INDEX_SENSOR_1_MANUAL_DATA;
	offset = sizeof(struct ispdrv_manual_data);
	for (loop = index; loop < (index + SENSOR_TOTAL); loop++) {
		g_share_buff_size_array[loop] = offset;
		g_share_buff_iva_array[loop] =
			&g_iva_manual_data_addr[loop - index];
		g_share_buff_kva_array[loop] =
			(void **)&g_kva_manual_ptr[loop - index];
	}
	/* Rgb2yuv matrix */
	index = ISP_SHARE_BUFF_INDEX_SENSOR_1_RGB2YUV;
	offset = sizeof(struct ispdrv_rgb2yuv_martix);
	for (loop = index; loop < (index + SENSOR_TOTAL); loop++) {
		g_share_buff_size_array[loop] = offset;
		g_share_buff_iva_array[loop] =
			share_buff_iq_info_get_rgb2yuv_iva(loop - index);
		g_share_buff_kva_array[loop] =
			share_buff_iq_info_get_rgb2yuv(loop - index);
	}
	/* Tone map */
	index = ISP_SHARE_BUFF_INDEX_SENSOR_1_TONEMAP;
	offset = sizeof(struct ispdrv_tonemap_curve);
	for (loop = index; loop < (index + SENSOR_TOTAL); loop++) {
		g_share_buff_size_array[loop] = offset;
		g_share_buff_iva_array[loop] =
			share_buff_iq_info_get_tonemap_iva(loop - index);
		g_share_buff_kva_array[loop] =
			share_buff_iq_info_get_tonemap(loop - index);
	}
	/* Motion */
	index = ISP_SHARE_BUFF_INDEX_SENSOR_1_MOTION;
	offset = sizeof(struct ispdrv_motion_input);
	for (loop = index; loop < (index + SENSOR_TOTAL); loop++) {
		g_share_buff_size_array[loop] = offset;
		g_share_buff_iva_array[loop] =
			&g_iva_motion_addr[loop - index];
		g_share_buff_kva_array[loop] =
			(void **)&g_kva_motion_ptr[loop - index];
	}
	/* Brightness */
	index = ISP_SHARE_BUFF_INDEX_SENSOR_1_BRIGHTNESS;
	offset = sizeof(struct ispdrv_brightness_gain_setting);
	for (loop = index; loop < (index + SENSOR_TOTAL); loop++) {
		g_share_buff_size_array[loop] = offset;
		g_share_buff_iva_array[loop] =
			share_buff_iq_info_get_brightness_setting_iva(
			loop - index);
		g_share_buff_kva_array[loop] =
			share_buff_iq_info_brightness_setting(loop - index);
	}
	/* D-Gain */
	index = ISP_SHARE_BUFF_INDEX_SENSOR_1_DGAIN;
	offset = sizeof(struct isp_d_gain_info);
	for (loop = index; loop < (index + SENSOR_TOTAL); loop++) {
		g_share_buff_size_array[loop] = offset;
		g_share_buff_iva_array[loop] =
			share_buff_iq_info_get_isp_d_gain_addr_iva(
			loop - index);
		g_share_buff_kva_array[loop] =
			share_buff_iq_info_get_isp_d_gain_addr_kva(
			loop - index);
	}
	/* Dld seq info */
	index = ISP_SHARE_BUFF_INDEX_SENSOR_1_DLD_SEQ_INFO;
	offset = sizeof(struct dld_sequence);
	for (loop = index; loop < (index + SENSOR_TOTAL); loop++) {
		g_share_buff_size_array[loop] = offset;
		g_share_buff_iva_array[loop] =
			&g_iva_ap_t_dld_seq_info_addr[loop - index];
		g_share_buff_kva_array[loop] =
			share_buff_get_shm_dld_seq_info_ptr_addr(loop - index);
	}
	/* Configuration of 3a info */
	index = ISP_SHARE_BUFF_INDEX_SENSOR_1_CFG_3A_INFO;
	offset = sizeof(struct cfg_3a_info);
	for (loop = index; loop < (index + SENSOR_TOTAL); loop++) {
		g_share_buff_size_array[loop] = offset;
		g_share_buff_iva_array[loop] =
			&g_iva_ap_t_cfg_3a_info_addr[loop - index];
		g_share_buff_kva_array[loop] =
			share_buff_get_shm_cfg_3a_info_ptr_addr(loop - index);
	}
	/* Digital zoom info*/
	index = ISP_SHARE_BUFF_INDEX_SENSOR_1_DZOOM_INFO;
	offset = sizeof(struct s_scenario_dzoom_info);
	for (loop = index; loop < (index + SENSOR_TOTAL); loop++) {
		g_share_buff_size_array[loop] = offset;
		g_share_buff_iva_array[loop] =
			&g_iva_ap_t_dzoom_info_addr[loop - index];
		g_share_buff_kva_array[loop] =
			(void **)&g_kva_dzoom_info_ptr[loop - index];
	}
	/* Scenario Info */
	g_share_buff_size_array[ISP_SHARE_BUFF_INDEX_SCENARIO_INFO] =
		sizeof(struct s_scenario_info_isp);
	g_share_buff_iva_array[ISP_SHARE_BUFF_INDEX_SCENARIO_INFO] =
		&g_iva_ap_t_scenarioInfo_addr;
	g_share_buff_kva_array[ISP_SHARE_BUFF_INDEX_SCENARIO_INFO] =
		(void **)&g_kva_scenario_info_ptr;
	/* IQ debug info */
	index = ISP_SHARE_BUFF_INDEX_SENSOR_1_DEBUG_IQ_INFO;
	offset = sizeof(struct s_iq_info_addr);
	for (loop = index; loop < (index + SENSOR_TOTAL); loop++) {
		g_share_buff_size_array[loop] = offset;
		g_share_buff_iva_array[loop] =
			&g_iva_debug_iq_info_addr[loop - index];
		g_share_buff_kva_array[loop] =
			(void **)&g_kva_debug_iq_info_ptr[loop - index];
	}
	/* CCM info */
	index = ISP_SHARE_BUFF_INDEX_SENSOR_1_CCM_INFO;
	offset = sizeof(struct iq_ccm_info);
	for (loop = index; loop < (index + SENSOR_TOTAL); loop++) {
		g_share_buff_size_array[loop] = offset;
		g_share_buff_iva_array[loop] =
			share_buff_iq_info_get_ccm_addr_iva(loop - index);
		g_share_buff_kva_array[loop] =
			share_buff_iq_info_get_ccm_addr_kva(loop - index);
	}
	/* Otp info */
	index = ISP_SHARE_BUFF_INDEX_SENSOR_1_OTP_INFO;
	offset = sizeof(struct iq_otp_info);
	for (loop = index; loop < (index + SENSOR_TOTAL); loop++) {
		g_share_buff_size_array[loop] = offset;
		g_share_buff_iva_array[loop] =
			share_buff_iq_info_get_otp_addr_iva(loop - index);
		g_share_buff_kva_array[loop] =
			share_buff_iq_info_get_otp_addr_kva(loop - index);
	}
	/* Proc still argument info */
	index = ISP_SHARE_BUFF_INDEX_SENSOR_1_PROC_STILL_ARG;
	offset = sizeof(struct ispdrv_proc_still_pref);
	for (loop = index; loop < (index + SENSOR_TOTAL); loop++) {
		g_share_buff_size_array[loop] = offset;
		g_share_buff_iva_array[loop] =
			share_buff_iq_info_get_proc_still_arg_iva(loop - index);
		g_share_buff_kva_array[loop] =
			share_buff_iq_info_get_proc_still_arg_kva(loop - index);
	}
	/* IQ function */
	g_share_buff_size_array[ISP_SHARE_BUFF_INDEX_IQ_FUNC] =
		IQ_INFO_FUNC_BUFF_SIZE_MAX;
	g_share_buff_iva_array[ISP_SHARE_BUFF_INDEX_IQ_FUNC] =
		share_buff_iq_function_addr();
	g_share_buff_kva_array[ISP_SHARE_BUFF_INDEX_IQ_FUNC] =
		NULL;
	/* Extend buffer */
	g_share_buff_size_array[ISP_SHARE_BUFF_INDEX_EXTEND_BUFF_INFO] =
		sizeof(struct ispdrv_exbuffer_info);
	g_share_buff_iva_array[ISP_SHARE_BUFF_INDEX_EXTEND_BUFF_INFO] =
		&g_iva_exbuf_info_addr;
	g_share_buff_kva_array[ISP_SHARE_BUFF_INDEX_EXTEND_BUFF_INFO] =
		(void **)&g_kva_exbuf_info_ptr;
}


static void set_isp_chip_id(u16 chip_id)
{
	isp_chip_id = chip_id;
}

static u16 get_isp_chip_id(void)
{
	return isp_chip_id;
}
