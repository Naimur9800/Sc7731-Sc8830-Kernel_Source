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
#include "cam_iommu.h"
#define DCAM_DEBUG
#ifdef DCAM_DEBUG
#define DCAM_TRACE				pr_debug
#else
#define DCAM_TRACE				pr_err
#endif

#define DCAM_LOWEST_ADDR			0x800
#define DCAM_ADDR_INVALID(addr) \
	((unsigned long)(addr) < DCAM_LOWEST_ADDR)
#define DCAM_YUV_ADDR_INVALID(y, u, v) \
	(DCAM_ADDR_INVALID(y) && \
	 DCAM_ADDR_INVALID(u) && \
	 DCAM_ADDR_INVALID(v))

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
	DCAM_PATH0_TX_DONE,
	DCAM_PATH0_OVF,
	DCAM_PDAF_SN_SOF,
	DCAM_PDAF_SN_EOF,
	DCAM_PDAF_CAP_SOF,
	DCAM_PDAF_CAP_EOF,
	DCAM_CAP_LINE_ERR,
	DCAM_CAP_FRM_ERR,
	DCAM_JPG_BUF_OVF,
	DCAM_ISP_FIFO_OVF,
	DCAM_CAP_MIPI_OVF,
	DCAM_PDAF_TX_DONE,
	DCAM_PDAF_BUF_OVF,
	DCAM_PDAF_END,
	DCAM_RESERVED,
	DCAM_PDAF_SOF,
	DCAM_PATH0_SOF,
	DCAM_PATH0_END,
	DCAM_PATH0_AXIM_DONE,
	DCAM_PDAF_AXIM_DONE,
	DCAM_MMU_VAOR_WR,
	DCAM_MMU_INV_WR,
	DCAM_MMU_UNS_WR,
	DCAM_MMU_PAOR_WR,
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	DCAM_CPP_ERR,
#endif
	DCAM_IRQ_NUMBER
};

enum dcam_cfg_id {
	DCAM_CAP_INTERFACE = 0,
	DCAM_CAP_SENSOR_MODE,
	DCAM_CAP_SYNC_POL,
	DCAM_CAP_DATA_BITS,
	DCAM_CAP_DATA_PACKET,
	DCAM_CAP_PRE_SKIP_CNT,
	DCAM_CAP_FRM_DECI,
	DCAM_CAP_FRM_COUNT_CLR,
	DCAM_CAP_FRM_COUNT_GET,
	DCAM_CAP_INPUT_RECT,
	DCAM_CAP_IMAGE_XY_DECI,
	DCAM_CAP_TO_ISP,
	DCAM_CAP_SAMPLE_MODE,
	DCAM_CAP_ZOOM_MODE,

	DCAM_PATH_INPUT_RECT,
	DCAM_PATH_INPUT_ADDR,
	DCAM_PATH_OUTPUT_SIZE,
	DCAM_PATH_OUTPUT_FORMAT,
	DCAM_PATH0_OUTPUT_ADDR,
	DCAM_PATH_OUTPUT_RESERVED_ADDR,
	DCAM_PATH_PDAF_OUTPUT_ADDR,
	DCAM_PATH_FRAME_BASE_ID,
	DCAM_PATH_IS_SCALE_EN,
	DCAM_PATH_DATA_ENDIAN,
	DCAM_PATH_SRC_SEL,
	DCAM_PATH_ENABLE,
	DCAM_PATH_FRAME_TYPE,
	DCAM_PATH_FRM_DECI,
	DCAM_PDAF_CONTROL,
	DCAM_CFG_ID_E_MAX
};

enum dcam_path_src_sel {
	DCAM_PATH_FROM_CAP = 0,
	DCAM_PATH_FROM_ISP,
	DCAM_PATH_FROM_NONE
};

enum dcam_data_endian {
	DCAM_ENDIAN_LITTLE = 0,
	DCAM_ENDIAN_BIG,
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

enum {
	DCAM_CLK_76M8_INDEX = 0,
	DCAM_CLK_256M_INDEX,
	DCAM_CLK_307M2_INDEX,
	DCAM_CLK_384M_INDEX
};

struct dcam_if_clk_tag {
	unsigned int clock;
	char *clk_name;
};

struct dcam_cap_sync_pol {
	unsigned char vsync_pol;
	unsigned char hsync_pol;
	unsigned char pclk_pol;
	unsigned char need_href;
	unsigned char pclk_src;
	unsigned char reserved[3];
};

struct camera_endian_sel {
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

struct camera_path_dec {
	unsigned char x_factor;
	unsigned char y_factor;
	unsigned char reserved[2];
};

struct camera_size {
	unsigned int w;
	unsigned int h;
};

struct camera_rect {
	unsigned int x;
	unsigned int y;
	unsigned int w;
	unsigned int h;
};

struct camera_addr {
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

struct camera_sc_tap {
	unsigned int y_tap;
	unsigned int uv_tap;
};

struct camera_deci {
	unsigned int deci_x_en;
	unsigned int deci_x;
	unsigned int deci_y_en;
	unsigned int deci_y;
};

struct camera_frame {
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
	unsigned int phy_addr;
	unsigned int vir_addr;
	unsigned int kaddr[2];
	unsigned int buf_size;
	unsigned int statis_phy_addr;
	unsigned int statis_vir_addr;
	unsigned int statis_buf_size;
	unsigned int irq_type;
	unsigned int irq_property;
	struct timeval time;
	ktime_t boot_time;
	struct pfiommu_info pfinfo;
	struct camera_frame *prev;
	struct camera_frame *next;
};

struct camera_get_path_id {
	unsigned int fourcc;
	unsigned int is_path_work[CAMERA_MAX_PATH];
	unsigned int need_isp_tool;
	unsigned int need_isp;
	unsigned int rt_refocus;
	struct camera_size input_size;
	struct camera_rect input_trim;
	struct camera_size output_size;
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

struct isp_path_info {
	unsigned int is_slow_motion;
};

struct dcam_path_capability {
	unsigned int count;
	uint32_t support_3dnr_mode;
	struct dcam_path_info path_info[CAMERA_MAX_PATH];
};

typedef int (*dcam_isr_func) (struct camera_frame *frame, void *u_data);

int sprd_dcam_module_init(enum dcam_id idx);
int sprd_dcam_module_deinit(enum dcam_id idx);
int sprd_dcam_module_en(enum dcam_id idx);
int sprd_dcam_module_dis(enum dcam_id idx);
int sprd_dcam_start(enum dcam_id idx);
int sprd_dcam_stop(enum dcam_id idx, int is_irq);
int sprd_dcam_reg_isr(enum dcam_id idx, enum dcam_irq_id id,
		      dcam_isr_func user_func, void *u_data);
int set_dcam_cap_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param);
int sprd_dcam_cap_get_info(enum dcam_id idx, enum dcam_cfg_id id, void *param);
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
int sprd_camera_get_path_id(enum dcam_id idx,
			struct camera_get_path_id *path_id,
			unsigned int *channel_id,
			unsigned int scene_mode);
int sprd_dcam_get_path_capability(struct dcam_path_capability *capacity);
int sprd_dcam_parse_dt(struct device_node *dn, unsigned int *dcam_count);
int set_dcam_path0_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param);
int set_dcam_path_pdaf_cfg(enum dcam_id idx, enum dcam_cfg_id id, void *param);
int sprd_dcam_update_clk(int clk_index, struct device_node *dn);
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
void sprd_dcam_cpp_error_notify(enum dcam_id idx);
#endif
#endif /* _DCAM_DRV_H_ */
