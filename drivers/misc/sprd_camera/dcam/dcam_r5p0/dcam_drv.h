/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
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
#ifndef _DCAM_DRV_H_
#define _DCAM_DRV_H_

#include <video/sprd_img.h>
#include "dcam_reg.h"
#include "sprd_sensor_drv.h"
#include "cam_iommu.h"
/*#define DCAM_DEBUG*/
#ifdef DCAM_DEBUG
#define DCAM_TRACE				pr_info
#else
#define DCAM_TRACE				pr_debug
#endif

#define DCAM_WAIT_FOREVER			0xFFFFFFFF

#define DCAM_PATH_3_FRM_CNT_MAX			32
#define DCAM_PATH_2_FRM_CNT_MAX			32
#define DCAM_PATH_1_FRM_CNT_MAX			32
#define DCAM_PATH_0_FRM_CNT_MAX			48
/* max between path_1_frm_cnt and path_2_frm_cnt */
#define DCAM_FRM_CNT_MAX			48
#define DCAM_HEIGHT_MIN				4
/* 640X480  div 10 */
#define DCAM_JPEG_LENGTH_MIN			30720

#define SHRINK_Y_UP_TH				235
#define SHRINK_Y_DN_TH				16
#define SHRINK_UV_UP_TH				240
#define SHRINK_UV_DN_TH				16
#define SHRINK_Y_OFFSET				16
#define SHRINK_Y_RANGE				2
#define SHRINK_C_OFFSET				16
#define SHRINK_C_RANGE				6

#define ISP_OVERLAP_ALIGN_16			128

enum dcam_swtich_status {
	DCAM_SWITCH_IDLE = 0,
	DCAM_SWITCH_PAUSE,
	DCAM_SWITCH_DONE,
	DCAM_SWITCH_MAX
};

enum dcam_drv_rtn {
	DCAM_RTN_SUCCESS = 0,
	DCAM_RTN_PARA_ERR = 0x10,
	DCAM_RTN_IO_ID_ERR,
	DCAM_RTN_ISR_ID_ERR,
	DCAM_RTN_MASTER_SEL_ERR,
	DCAM_RTN_MODE_ERR,
	DCAM_RTN_TIMEOUT,

	DCAM_RTN_CAP_FRAME_SEL_ERR = 0x20,
	DCAM_RTN_CAP_IN_FORMAT_ERR,
	DCAM_RTN_CAP_IN_BITS_ERR,
	DCAM_RTN_CAP_IN_YUV_ERR,
	DCAM_RTN_CAP_SYNC_POL_ERR,
	DCAM_RTN_CAP_SKIP_FRAME_ERR,
	DCAM_RTN_CAP_FRAME_DECI_ERR,
	DCAM_RTN_CAP_XY_DECI_ERR,
	DCAM_RTN_CAP_FRAME_SIZE_ERR,
	DCAM_RTN_CAP_SENSOR_MODE_ERR,
	DCAM_RTN_CAP_JPEG_BUF_LEN_ERR,
	DCAM_RTN_CAP_IF_MODE_ERR,

	DCAM_RTN_PATH_SRC_SIZE_ERR = 0x30,
	DCAM_RTN_PATH_TRIM_SIZE_ERR,
	DCAM_RTN_PATH_DES_SIZE_ERR,
	DCAM_RTN_PATH_IN_FMT_ERR,
	DCAM_RTN_PATH_OUT_FMT_ERR,
	DCAM_RTN_PATH_SC_ERR,
	DCAM_RTN_PATH_SUB_SAMPLE_ERR,
	DCAM_RTN_PATH_ADDR_ERR,
	DCAM_RTN_PATH_FRAME_TOO_MANY,
	DCAM_RTN_PATH_FRAME_LOCKED,
	DCAM_RTN_PATH_NO_MEM,
	DCAM_RTN_PATH_GEN_COEFF_ERR,
	DCAM_RTN_PATH_SRC_ERR,
	DCAM_RTN_PATH_ENDIAN_ERR,
	DCAM_RTN_PATH_FRM_DECI_ERR,
	DCAM_RTN_MAX
};

enum dcam_irq_id {
	DCAM_SN_SOF = 0,
	DCAM_SN_EOF,
	DCAM_CAP_SOF,
	DCAM_CAP_EOF,
	DCAM_PATH0_DONE,
	DCAM_PATH0_OV = 5,
	DCAM_PATH1_DONE,
	DCAM_PATH1_OV,
	DCAM_PATH2_DONE,
	DCAM_PATH2_OV,
	DCAM_SN_LINE_ERR = 10,
	DCAM_SN_FRAME_ERR,
	DCAM_JPEG_BUF_OV,
	DCAM_ISP_OV,
	DCAM_MIPI_OV,
	DCAM_ROT_DONE = 15,
	DCAM_PATH1_SLICE_DONE,
	DCAM_PATH2_SLICE_DONE,
	DCAM_RAW_SLICE_DONE,
	DCAM_PATH1_SOF,
	DCAM_PATH2_SOF = 20,
	DCAM_PATH0_END,
	DCAM_PATH1_END,
	DCAM_PATH2_END,
	DCAM_PATH0_SOF,
	DCAM_PATH3_SOF = 25,
	DCAM_PATH3_END,
	DCAM_PATH3_DONE,
	DCAM_PATH3_OV,
	DCAM_I2P_OV,
	DCAM_JPEGLS_DONE = 30,
	DCAM_JPEGLS_OV,
	DCAM_IRQ_NUMBER
};

enum dcam_cfg_id {
	DCAM_CAP_INTERFACE = 0,
	DCAM_CAP_SENSOR_MODE = 1,
	DCAM_CAP_SYNC_POL = 2,
	DCAM_CAP_DATA_BITS,
	DCAM_CAP_DATA_PACKET,
	DCAM_CAP_YUV_TYPE,
	DCAM_CAP_PRE_SKIP_CNT,
	DCAM_CAP_FRM_DECI,
	DCAM_CAP_FRM_COUNT_CLR,
	DCAM_CAP_FRM_COUNT_GET,
	DCAM_CAP_INPUT_RECT,
	DCAM_CAP_IMAGE_XY_DECI,
	DCAM_CAP_JPEG_GET_LENGTH,
	DCAM_CAP_JPEG_SET_BUF_LEN,
	DCAM_CAP_JPEGLS_GET_LENGTH,
	DCAM_CAP_TO_ISP,
	DCAM_CAP_SAMPLE_MODE,
	DCAM_CAP_ZOOM_MODE,

	DCAM_PATH_INPUT_SIZE,
	DCAM_PATH_INPUT_RECT,
	DCAM_PATH_INPUT_ADDR,
	DCAM_PATH_OUTPUT_SIZE,
	DCAM_PATH_OUTPUT_FORMAT,
	DCAM_PATH_OUTPUT_ADDR,
	DCAM_PATH_OUTPUT_RESERVED_ADDR,
	DCAM_PATH_FRAME_BASE_ID,
	DCAM_PATH_SWAP_BUFF,
	DCAM_PATH_SUB_SAMPLE_EN_X,
	DCAM_PATH_SUB_SAMPLE_EN_Y,
	DCAM_PATH_SUB_SAMPLE_MOD,
	DCAM_PATH_SLICE_SCALE_EN,
	DCAM_PATH_SLICE_SCALE_HEIGHT,
	DCAM_PATH_DITHER_EN,
	DCAM_PATH_IS_IN_SCALE_RANGE,
	DCAM_PATH_IS_SCALE_EN,
	DCAM_PATH_SLICE_OUT_HEIGHT,
	DCAM_PATH_DATA_ENDIAN,
	DCAM_PATH_SRC_SEL,
	DCAM_PATH_ENABLE,
	DCAM_PATH_FRAME_TYPE,
	DCAM_PATH_ROT_MODE,
	DCAM_PATH_FRM_DECI,
	DCAM_PATH_SHRINK,
	DCAM_PATH_JPEGLS,
	DCAM_PDAF_CONTROL,
	DCAM_CFG_ID_E_MAX
};

#if 0
enum dcam_sub_sample_mode {
	DCAM_SUB_2  = 0,
	DCAM_SUB_4,
	DCAM_SUB_8,
	DCAM_SUB_16,
	DCAM_SUB_MAX
};

enum dcam_ahb_frm {
	DCAM_AHB_FRAME_SRC,
	DCAM_AHB_FRAME_PATH1_DST,
	DCAM_AHB_FRAME_PATH2_DST,
	DCAM_AHB_FRAME_SWAP,
	DCAM_AHB_FRAME_LINE,
	DCAM_AHB_FRAME_MAX
};
#endif

enum iram_owner {
	IRAM_FOR_DCAM = 0,
	IRAM_FOR_ARM
};

enum dcam_clk_sel {
	DCAM_CLK_384M = 0,
	DCAM_CLK_312M,
	DCAM_CLK_307M2,
	DCAM_CLK_256M,
	DCAM_CLK_128M,
	DCAM_CLK_76M8,
	DCAM_CLK_NONE
};

enum dcam_path_src_sel {
	DCAM_PATH_FROM_CAP = 0,
	DCAM_PATH_FROM_ISP,
	DCAM_PATH_FROM_NONE
};

enum dcam_data_endian {
	DCAM_ENDIAN_BIG = 0,
	DCAM_ENDIAN_LITTLE,
	DCAM_ENDIAN_HALFBIG,
	DCAM_ENDIAN_HALFLITTLE,
	DCAM_ENDIAN_MAX
};

enum dcam_glb_reg_id {
	DCAM_CFG_REG = 0,
	DCAM_CONTROL_REG,
	DCAM_INIT_MASK_REG,
	DCAM_INIT_CLR_REG,
	DCAM_AHBM_STS_REG,
	DCAM_ENDIAN_REG,
	DCAM_REG_MAX
};

enum dcam_v4l2_wtite_cmd_id {
	DCAM_V4L2_WRITE_STOP = 0x5AA5,
	DCAM_V4L2_WRITE_FREE_FRAME = 0xA55A,
	DCAM_V4L2_WRITE_MAX
};

struct dcam_cap_sync_pol {
	unsigned char vsync_pol;
	unsigned char hsync_pol;
	unsigned char pclk_pol;
	unsigned char need_href;
	unsigned char pclk_src;
	unsigned char reserved[3];
};

struct dcam_endian_sel {
	unsigned char y_endian;
	unsigned char uv_endian;
	unsigned char reserved0;
	unsigned char reserved1;
};

struct dcam_cap_dec {
	unsigned char x_factor;
	unsigned char y_factor;
	unsigned char x_mode;
	unsigned char reserved;
};

struct dcam_path_dec {
	unsigned char x_factor;
	unsigned char y_factor;
	unsigned char reserved[2];
};

struct dcam_size {
	unsigned int w;
	unsigned int h;
};

struct dcam_rect {
	unsigned int x;
	unsigned int y;
	unsigned int w;
	unsigned int h;
};

struct dcam_addr {
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

struct dcam_sc_tap {
	unsigned int y_tap;
	unsigned int uv_tap;
};

struct dcam_deci {
	unsigned int deci_x_en;
	unsigned int deci_x;
	unsigned int deci_y_en;
	unsigned int deci_y;
};

struct dcam_frame {
	unsigned int type;
	unsigned int lock;
	unsigned int flags;
	unsigned int fid;
	unsigned int width;
	unsigned int height;
	unsigned int yaddr;
	unsigned int uaddr;
	unsigned int vaddr;
	unsigned int yaddr_vir;
	unsigned int uaddr_vir;
	unsigned int vaddr_vir;
	struct pfiommu_info pfinfo;
	struct dcam_frame *prev;
	struct dcam_frame *next;
};

struct dcam_get_path_id {
	unsigned int fourcc;
	unsigned int is_path_work[DCAM_PATH_MAX];
	unsigned int need_isp_tool;
	unsigned int need_isp;
	unsigned int rt_refocus;
	struct dcam_size input_size;
	struct dcam_rect input_trim;
	struct dcam_size output_size;
};

struct dcam_path_info {
	unsigned int line_buf;
	unsigned int support_yuv;
	unsigned int support_raw;
	unsigned int support_jpeg;
	unsigned int support_scaling;
	unsigned int support_trim;
	unsigned int is_scaleing_path;
};

struct dcam_path_capability {
	unsigned int count;
	struct dcam_path_info path_info[DCAM_PATH_MAX];
};

typedef int (*dcam_isr_func) (struct dcam_frame *frame, void *u_data);

#define dcam_path0_cfg(idx, id, param) \
	sprd_dcam_path_cfg(idx, DCAM_PATH_IDX_0, id, param)
#define dcam_path1_cfg(idx, id, param) \
	sprd_dcam_path_cfg(idx, DCAM_PATH_IDX_1, id, param)
#define dcam_path2_cfg(idx, id, param) \
	sprd_dcam_path_cfg(idx, DCAM_PATH_IDX_2, id, param)
#define dcam_path3_cfg(idx, id, param) \
	sprd_dcam_path_cfg(idx, DCAM_PATH_IDX_3, id, param)

int sprd_dcam_module_init(enum dcam_id idx);
int sprd_dcam_module_deinit(enum dcam_id idx);
int sprd_dcam_module_en(enum dcam_id idx);
int sprd_dcam_module_dis(enum dcam_id idx);
int sprd_dcam_reset(enum dcam_id idx, enum dcam_path_index path_index,
		    unsigned int is_isr);
int sprd_dcam_update_path(enum dcam_id idx, enum dcam_path_index path_index,
			  struct dcam_size *in_size, struct dcam_rect *in_rect,
			  struct dcam_size *out_size);
int sprd_dcam_start_path(enum dcam_id idx, enum dcam_path_index path_index);
int sprd_dcam_start(enum dcam_id idx);
int sprd_dcam_stop_path(enum dcam_id idx, enum dcam_path_index path_index);
int sprd_dcam_stop(enum dcam_id idx);
int sprd_dcam_reg_isr(enum dcam_id idx, enum dcam_irq_id id,
		      dcam_isr_func user_func, void *u_data);
int sprd_dcam_cowork_enable(enum dcam_id idx);
void sprd_dcam_cowork_disable(enum dcam_id idx);
int sprd_dcam_cap_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param);
int sprd_dcam_cap_get_info(enum dcam_id idx, enum dcam_cfg_id id, void *param);
int sprd_dcam_path_cfg(enum dcam_id idx, enum dcam_path_index path_index,
		       enum dcam_cfg_id id, void *param);
int sprd_dcam_read_registers(enum dcam_id idx, unsigned int *reg_buf,
			     unsigned int *buf_len);
void sprd_dcam_glb_reg_awr(enum dcam_id idx, unsigned long addr,
			   unsigned int val, unsigned int reg_id);
void sprd_dcam_glb_reg_owr(enum dcam_id idx, unsigned long addr,
			   unsigned int val, unsigned int reg_id);
void sprd_dcam_glb_reg_mwr(enum dcam_id idx, unsigned long addr,
			   unsigned int mask, unsigned int val,
			   unsigned int reg_id);
int sprd_dcam_drv_init(struct platform_device *pdev);
void sprd_dcam_drv_deinit(void);
int sprd_dcam_get_path_id(struct dcam_get_path_id *path_id,
			  unsigned int *channel_id);
int sprd_dcam_stop_sc_coeff(enum dcam_id idx);
int sprd_dcam_get_path_capability(struct dcam_path_capability *capacity);
int sprd_dcam_parse_clk(struct platform_device *pdev);
int sprd_dcam_parse_irq(struct platform_device *pdev);
int sprd_dcam_parse_regbase(struct platform_device *pdev);

#endif /* _DCAM_DRV_H_ */
