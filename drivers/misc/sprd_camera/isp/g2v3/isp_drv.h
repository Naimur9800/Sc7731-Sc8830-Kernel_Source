/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#ifndef _ISP_DRV_HEADER_
#define _ISP_DRV_HEADER_

#include <linux/semaphore.h>
#include <linux/spinlock_types.h>
#include <linux/sprd_ion.h>
#include <linux/timer.h>
#include <video/sprd_isp_altek.h>
#include <video/sprd_img.h>
#include "altek_isp/altek_isp_drv.h"

/*66M bytes for 6792x3816*/
#define ISP_DRAM_MODE_BUF_SIZE                  0x4200000
/*131M bytes for 6792x3816*/
#define ISP_HIGH_QUALITY_MODE_BUF_SIZE          0x8300000

#define ISP_DEVICE_NAME                         "sprd_isp"
#define ISP_QUEUE_NAME_LENGTH                   32
#define ISP_IMG_QUEUE_LEN                       16
#define ISP_QUEUE_LENGTH                        16
#define ISP_STATISTICS_BUF_MAX                  5
#define ISP_STATISTICS_QUEUE_LEN                8
/*preview,video,still capture ,statistics,for irp_a and irp_b*/
#define PIPE_COUNT_MAX                          3

#define ISP_CONTINUE_WIDTH_MAX                  5344
#define ISP_CONTINUE_HEIGHT_MAX                 4016
#define ISP_SINGLE_WIDTH_MAX                    5344
#define ISP_SINGLE_HEIGHT_MAX                   4016

#define RESERVED_BUFFER_COUNT_MAX               5

#define ISP_DRV_DEBUG

#ifdef ISP_DRV_DEBUG
#define ISP_DRV_TRACE				pr_info
#else
#define ISP_DRV_TRACE				pr_debug
#endif

#define ISP_FUNC_ID_AE_INFO 0
#define ISP_FUNC_ID_NUMBER 1

enum isp_data_endian {
	ISP_ENDIAN_BIG = 0,
	ISP_ENDIAN_LITTLE,
	ISP_ENDIAN_HALFBIG,
	ISP_ENDIAN_HALFLITTLE,
	ISP_ENDIAN_MAX
};

enum isp_dev_id {
	ISP_DEV0,
	ISP_DEV1,
	ISP_DEV2,
	ISP_DEV_NUM,
};

enum isp_img_id {
	ISP_PATH_PREIVEW,
	ISP_PATH_VIDEO,
	ISP_PATH_STILL,
	ISP_PATH_STATISTICS,
	ISP_PATH_MAX_NUM,
};

enum isp_cfg_id {
	ISP_CFG_PATH_OUTPUT_ADDR,
	ISP_CFG_PATH_OUTPUT_RESERVED_ADDR,
	ISP_CFG_PATH_GET_PARAM,
	ISP_CFG_PATH_SET_PARAM,
	ISP_CFG_PATH_GET_WORK_STATUS,
	ISP_CFG_PATH_SET_WORK_STATUS,
};

enum isp_clk_sel {
	ISP_CLK_480M = 0,
	ISP_CLK_384M,
	ISP_CLK_312M,
	ISP_CLK_256M,
	ISP_CLK_128M,
	ISP_CLK_76M8,
	ISP_CLK_48M,
	ISP_CLK_NONE
};

enum isp_scenario_mode {
	ISP_SCENARIO_PREVIEW_SENSOR_1 = 2,
	ISP_SCENARIO_DUAL_SHOT_REARS,
	ISP_SCENARIO_FRONT_PREVIEW_LITE,
	ISP_SCENARIO_PREVIEW_STRIPE,
	ISP_SCENARIO_PREVIEW_STILL_SS,
	ISP_SCENARIO_PREVIEW_SENSOR_2,
	ISP_SCENARIO_PREVIEW_HIGH_QUALITY,
	ISP_SCENARIO_ALTEKRAW10,
	ISP_SCENARIO_ALTEKRAW10_HIGH_QUALITY,
	ISP_SCENARIO_TOTAL
};

enum isp_capture_mode {
	ISP_CAP_MODE_AUTO = 0,
	ISP_CAP_MODE_RAW10 = 6,
	ISP_CAP_MODE_HIGH_ISO,
	ISP_CAP_MODE_HIGHISO_RAW10,
	ISP_CAP_MODE_DRAM,
	ISP_CAP_MODE_BURST,
};

enum {
	ISP_SHADING_OFF,
	ISP_SHADING_ON,
	ISP_SHADING_TOTAL
};

struct isp_output_addr {
	unsigned int yaddr;
	unsigned int uaddr;
	unsigned int vaddr;
	unsigned int yaddr_vir;
	unsigned int uaddr_vir;
	unsigned int vaddr_vir;
	unsigned int mfd_y;
	unsigned int mfd_u;
	unsigned int mfd_v;
};

struct isp_buf_addr {
	uint32_t buff_size;
	size_t iova_size;
	unsigned long buff_vir_addr;
	unsigned long buff_phys_addr;
	unsigned long iommu_addr;
};

struct img_size {
	uint32_t w;
	uint32_t h;
};

struct isp_iommu_info {
	struct device *dev;
	unsigned int mfd;
	struct sg_table *table;
	size_t size;
	unsigned long iova;
	struct dma_buf *dmabuf_p;
	unsigned int offset;
};

struct isp_img_buf {
	uint32_t format;
	uint32_t channel_id;
	uint32_t base_id;
	uint32_t img_id;
	uint32_t is_reserved_buf;
	struct img_size buf_size;
	unsigned long yaddr;
	unsigned long uaddr;
	unsigned long vaddr;
	unsigned long yaddr_vir;
	unsigned long uaddr_vir;
	unsigned long vaddr_vir;
	uint32_t reserved;
	int32_t img_y_fd;
	int32_t img_u_fd;
	int32_t img_v_fd;
	unsigned long iova_yaddr;
	unsigned long iova_uaddr;
	unsigned long iova_vaddr;
	size_t iova_y_size;
	size_t iova_u_size;
	size_t iova_v_size;
	struct isp_iommu_info pfinfo[3];
	struct sprd_isp_time time_stamp;
};

struct isp_img_frame {
	uint32_t format;
	uint32_t img_id;
	uint32_t irq_id;
	struct img_size buf_size;
	unsigned long yaddr;
	unsigned long uaddr;
	unsigned long vaddr;
	unsigned long yaddr_vir;
	unsigned long uaddr_vir;
	unsigned long vaddr_vir;
};

struct img_statis_frame {
	uint32_t format;
	uint32_t irq_flag;
	unsigned long buf_size;
	unsigned long phy_addr;
	unsigned long vir_addr;
};

struct isp_match_data {
	struct isp_awb_match_data awb_data;
	struct isp_ae_match_data ae_data;
	spinlock_t ae_lock;
	spinlock_t awb_lock;
};

struct img_statis_buf_queue {
	struct img_statis_frame buff[ISP_STATISTICS_QUEUE_LEN];
	struct img_statis_frame *write;
	struct img_statis_frame *read;
	spinlock_t lock;
};

struct isp_img_buf_queue {
	struct isp_img_buf buff[ISP_IMG_QUEUE_LEN];
	struct isp_img_buf *write;
	struct isp_img_buf *read;
	uint8_t name[ISP_QUEUE_NAME_LENGTH];
	uint32_t num;
	spinlock_t lock;
};

struct isp_img_frame_queue {
	struct isp_img_frame buff[ISP_IMG_QUEUE_LEN];
	struct isp_img_frame *write;
	struct isp_img_frame *read;
	spinlock_t lock;
};

struct isp_img_info {
	uint32_t img_fmt;
	uint32_t is_work;
	uint32_t dram_eb;
	uint32_t buf_num;
	uint32_t line_offset;
	atomic_t is_running;
	uint32_t frm_id_base;
	struct isp_addr addr[IMG_BUF_NUM_MAX];
	struct isp_addr addr_vir[IMG_BUF_NUM_MAX];
	struct img_size out_size;
	struct isp_img_buf_queue in_buff_queue;
	struct isp_img_buf_queue buf_queue;
	struct isp_img_buf_queue frame_queue;
	uint32_t reserved_buf_count;
	struct isp_img_buf reserved_buf[RESERVED_BUFFER_COUNT_MAX];
};

struct isp_statis_buf_info {
	uint32_t buf_num;
	uint32_t dram_eb;
	uint32_t line_offset;
	uint32_t img_fmt;
	struct img_size out_size;
	struct img_statis_frame statis_buf[ISP_STATISTICS_BUF_MAX];
};

struct isp_path_info {
	struct img_size in_size;
	struct isp_img_info img_info[ISP_OUTPUT_IMG_TOTAL];
	struct isp_statis_buf_info statis_buf_info;
	struct isp_statis_buf_info af_statis_buf_info;
};

struct isp_node {
	uint32_t irq_val0;
	uint32_t reserved;
	struct timeval time;
};

struct isp_queue {
	struct isp_irq_info node[ISP_QUEUE_LENGTH];
	struct isp_irq_info *write;
	struct isp_irq_info *read;
	spinlock_t lock;
};

struct isp_raw_buf_info {
	int32_t fd;
	unsigned long phy_addr;
	unsigned long virt_addr;
	uint32_t size;
	unsigned long iova_addr;
	size_t iova_size;
};
/*
* sns_id-->set by user
* isp_id-->config according to the sns_id and some other params
* width-->sensor outputsize
* height-->sensor output size
* color_order-->sensor color order
* isp_path--> isp1,isp2 or isp_lite
* output_frame_queue-->when image frame done interrupt occurs,
* all the output frame will write to this queue
*/
struct isp_pipe_dev {
	atomic_t users;
	struct device_node *dn;
	struct clk *clock;
	/*struct platform_device *pdev;*/
	struct semaphore ioctl_lock;
	struct completion isr_img_lock;/*for read/write image queue lock*/
	struct isp_buf_addr fw_mem;
	struct isp_buf_addr dram_mem;
	struct isp_raw_buf_info high_iso_mem;
	struct isp_raw_buf_info raw10_mem[ISP_RAWBUF_NUM];
	uint32_t raw_index;
	atomic_t raw2still_state;   /*0:finish  1:proc in progress*/
	atomic_t raw2still_flowctrl;	/*0:stop  1:multiple proc 2:singe proc*/
	uint32_t still_total;
	uint32_t raw2still_err;
	uint32_t raw2still_errcnt;
	struct timer_list raw2still_timer;
	struct isp_img_buf isp_cap_buf;
	struct isp_img_buf isp_cap_buf_reserved;
	struct isp_img_buf isp_src_sns_raw;
	struct s_scinfo_bayerscl_out_info bayerscl_out_info;
	uint32_t scenario_id;
	uint32_t capture_mode;
	uint32_t sns_id;
	uint32_t isp_id;
	uint32_t width;
	uint32_t height;
	uint32_t color_order;
	struct isp_path_info isp_path;
	struct isp_img_frame_queue output_frame_queue;
	uint32_t dcam_id;
	struct altek_iq_info iq_param;
	struct altek_iq_info iq_af_param;
	uint32_t shading_bin_offset;
	uint32_t irp_bin_offset;
	uint32_t cbc_bin_offset;
	uint32_t pdaf_supported;
	uint32_t frm_index;
	uint32_t af_stats_cnt;
	uint32_t sof_index;
	uint32_t skip_num;
	uint32_t deci_num;
	uint32_t raw_mode;
	uint8_t iq_param_idx;
	uint32_t skip_index;
	void *private_data;
	struct scinfo_out_bypassflg path_bypass;
};

struct isp_if_context {
	struct isp_pipe_dev *isp_pipe;
	struct completion isr_done_lock;
	struct isp_queue queue;
	struct isp_irq_info irq_info;
	struct img_statis_frame reserved_statis_buf;
	struct img_statis_buf_queue statis_frame_queue;
	struct img_statis_buf_queue af_statis_frame_queue;
	spinlock_t statis_buf_lock;
	unsigned int cur_isp_write_buf[ISP_OUTPUT_IMG_TOTAL];
	unsigned int last_isp_write_buf[ISP_OUTPUT_IMG_TOTAL];
	atomic_t isp_if_existed;
};

struct isp_hw {
	struct clk *isp_mclk;
	struct clk *isp_mclk_default;
	struct clk *isp_mclk_p;
	struct clk *isp_pclk;
	struct clk *isp_pclk_default;
	struct clk *isp_pclk_p;
	struct clk *isp_iclk;
	struct clk *isp_iclk_default;
	struct clk *isp_iclk_p;
	struct clk *isp_lclk;
	struct clk *isp_lclk_default;
	struct clk *isp_lclk_p;
	struct clk *isp0_clk;
	struct clk *isp0_clk_default;
	struct clk *isp0_clk_p;
	struct clk *isp1_clk;
	struct clk *isp1_clk_default;
	struct clk *isp1_clk_p;
	struct clk *isp2_clk;
	struct clk *isp2_clk_default;
	struct clk *isp2_clk_p;
	struct clk *isp0_eb;
	struct clk *isp1_eb;
	struct clk *isp2_eb;
	struct clk *isp_mclk_eb;
	struct clk *isp_pclk_eb;
	struct clk *isp_iclk_eb;
	struct clk *isp_lclk_eb;
	struct clk *isp0_axi_eb;
	struct clk *isp1_axi_eb;
	struct clk *isp2_axi_eb;
	struct clk *i0_in_isp_eb;
	struct clk *i1_in_isp_eb;
	struct clk *i2_in_isp_eb;
	unsigned int isp_irq;
	unsigned long isp_regbase;
	unsigned long isp_reg_max_size;
	struct mutex hw_lock;
};

static inline int32_t isp_scenario_id_map(enum isp_scenario_mode scenario_id)
{
	if ((scenario_id == ISP_SCENARIO_PREVIEW_HIGH_QUALITY)
		|| (scenario_id == ISP_SCENARIO_ALTEKRAW10)
		|| (scenario_id == ISP_SCENARIO_ALTEKRAW10_HIGH_QUALITY))
		return ISP_SCENARIO_PREVIEW_STILL_SS;
	else
		return  scenario_id;
}

void _isp_irq_handle(uint32_t irq);
int32_t _isp_img_buf_queue_init(struct isp_img_buf_queue *queue,
		uint8_t *qname);
int32_t _isp_img_buf_queue_write(struct isp_img_buf_queue *queue,
			struct isp_img_buf *buf);
int32_t _isp_img_buf_queue_read(struct isp_img_buf_queue *queue,
			struct isp_img_buf *buf);
uint32_t _isp_img_buf_queue_nodenum(struct isp_img_buf_queue *queue);
int32_t _isp_img_frame_queue_init(struct isp_img_frame_queue *queue);
int32_t _isp_img_frame_queue_write(struct isp_img_frame_queue *queue,
			struct isp_img_frame *buf);
int32_t _isp_img_frame_queue_read(struct isp_img_frame_queue *queue,
			struct isp_img_frame *buf);
int32_t _isp_statis_queue_init(struct img_statis_buf_queue *queue);
int32_t _isp_statis_queue_write(struct img_statis_buf_queue *queue,
			struct img_statis_frame *buf);
int32_t _isp_statis_queue_read(struct img_statis_buf_queue *queue,
			struct img_statis_frame *buf);
int32_t _isp_irq_queue_init(struct isp_queue *queue);
int32_t _isp_irq_queue_write(struct isp_queue *queue,
	struct isp_irq_info *node);
int32_t _isp_irq_queue_read(struct isp_queue *queue,
	struct isp_irq_info *node);
int32_t _isp_img_buf_queue_unread(struct isp_img_buf_queue *queue);
int32_t isp_cfg_fw_param(void __user *param, struct isp_pipe_dev *isp_pipeline);
int32_t isp_capability(void __user *param, struct isp_pipe_dev *isp_pipeline);
int32_t isp_dump_reg_info(void);
void isp_print_reg(void);
int32_t isp_match_stripe_mode(void);
int32_t isp_update_bypass_flag(struct isp_pipe_dev *isp_pipeline);


int32_t  get_unused_isp_id(struct sprd_img_res *res);
void put_used_isp_id(enum isp_dev_id isp_id);
struct isp_if_context *sprd_isp_get_isp_handle(enum isp_dev_id idx);
int32_t sprd_isp_path_set(enum isp_dev_id isp_id, enum isp_img_id img_id,
	enum isp_cfg_id cfg_id, void *param);
int32_t sprd_isp_path_get(enum isp_dev_id isp_id, enum isp_img_id img_id,
	enum isp_cfg_id cfg_id, void *param);
int32_t sprd_isp_stream_on(enum isp_dev_id isp_id);


typedef int (*isp_isr_func) (unsigned int isp_id, unsigned int img_id,
	void *u_data);
int32_t sprd_isp_reg_callback(enum isp_dev_id isp_id, enum ispdrv_int irq_id,
		      isp_isr_func user_func, void *user_data);
#endif
