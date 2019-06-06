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

#include <linux/of.h>
#include <linux/platform_device.h>
#include <video/sprd_img.h>
#include <video/sprd_isp_r6p10v2.h>
#include "isp_reg.h"
#include "../dcam_if_r3p0v2/dcam_drv.h"
#include "cam_iommu.h"

#define ISP_QUEUE_LENGTH		16
#define ISP_SCL0_MAX_WIDTH		2304
#define ISP1_SCL0_MAX_WIDTH		640
#define ISP_ONLINE_ZFACTOR_MAX		2
#define ISP_BUF_QUEUE_LENGTH		16
#define ISP_BING4AWB_NUM		2
#define ISP_STATISTICS_BUF_MAX		4
#define ISP_STATISTICS_QUEUE_LEN	8
#define ISP_IMG_OUTPUT_PATH_MAX		3
#define ISP_FRM_QUEUE_LENGTH		8
#define ISP_STORE_CCE_BUF_NUM		5
#define ISP_PIXEL_ALIGN_WIDTH		4
#define ISP_PIXEL_ALIGN_HEIGHT		2
#define ISP_RAW_AFM_ITEM		40
#define ISP_FRGB_GAMMA_BUF_SIZE     (257 * 4 + 4)
#define PATH_CPP_ION_BUF_NUM 5
#define ISP_LOWEST_ADDR            0x800
#define ISP_ADDR_INVALID(addr)     \
			((unsigned long)(addr) < (unsigned long)ISP_LOWEST_ADDR)

#define ISP_PATH_TIMEOUT			msecs_to_jiffies(500)

#define ISP_PATH_PRE_FRM_CNT_MAX			32
#define ISP_PATH_CAP_FRM_CNT_MAX			32
#define ISP_PATH_VID_FRM_CNT_MAX			32
#define ISP_PATH_0_FRM_CNT_MAX				48

#define ISP_SC_COEFF_BUF_COUNT				20
#define ISP_SC_COEFF_BUF_SIZE			(24 << 10)
#define ISP_SC_COEFF_COEF_SIZE			(1 << 10)
#define ISP_SC_COEFF_TMP_SIZE			(21 << 10)
#define ISP_PATH_SCL_COUNT	5

#define ISP_SC_H_COEF_SIZE				0xC0
#define ISP_SC_V_COEF_SIZE				0x210
#define ISP_SC_V_CHROM_COEF_SIZE			0x210

#define ISP_SC_COEFF_H_NUM			(ISP_SC_H_COEF_SIZE/4)
#define ISP_SC_COEFF_V_NUM			(ISP_SC_V_COEF_SIZE/4)
#define ISP_SC_COEFF_V_CHROMA_NUM		(ISP_SC_V_CHROM_COEF_SIZE/4)

/*should be replaced by the irq id*/
#define ISP_IRQ_ERR_MASK0 \
	(/*(1 << ISP_INT_AEM_ERROR) | */(1 << ISP_INT_AFL_ERROR) | \
	(1 << ISP_INT_BINNING_ERROR))

#define ISP_IRQ_ERR_MASK1 \
	((1 << ISP_INT_BPC_ERR0) | (1 << ISP_INT_BPC_ERR1) | \
	(1 << ISP_INT_BPC_ERR0) | (1 << ISP_INT_BPC_STORE_NOT_EMPTY_ERROR))

#define ISP_IRQ_ERR_MASK2 \
	((1 << ISP_INT_FMCU_CMD_ERROR))

#define ISP_INT_LINE_MASK0 \
	((1 << ISP_INT_AEM_START) | (1 << ISP_INT_AEM_DONE) | \
	(1 << ISP_INT_AFL_START) | (1 << ISP_INT_AFL_DONE)  | \
	(1 << ISP_INT_BINNING_START) | (1 << ISP_INT_BINNING_DONE) | \
	(1 << ISP_INT_DCAM_SOF) | (1 << ISP_INT_DCAM_EOF) | \
	(1 << ISP_INT_STORE_DONE) | (1 << ISP_INT_AFL_LAST_SOF) | \
	(1 << ISP_INT_ISP_START) | (1 << ISP_INT_SHADOW_DONE) | \
	(1 << ISP_INT_ISP_ALL_DONE) | (1 << ISP_INT_FETCH_RAW_DONE) | \
	(1 << ISP_INT_ISP_ALL_DONE) | (1 << ISP_INT_DCAM_SOF) | \
	(1 << ISP_INT_PDAF_MEM_LEFT_DONE) | \
	(1 << ISP_INT_PDAF_MEM_RIGHT_DONE) | \
	(1 << ISP_INT_PDAF_SHADOW_DONE) | \
	 ISP_IRQ_ERR_MASK0)

#define ISP_INT_LINE_MASK1 \
	((1 << ISP_INT_STORE_DONE_OUT) | (1 << ISP_INT_SHADOW_DONE_OUT) | \
	(1 << ISP_INT_STORE_DONE_CAP) | (1 << ISP_INT_SHADOW_DONE_CAP)  | \
	(1 << ISP_INT_STORE_DONE_VID) | (1 << ISP_INT_SHADOW_DONE_VID) | \
	(1 << ISP_INT_STORE_DONE_PRE) | (1 << ISP_INT_SHADOW_DONE_PRE) | \
	(1 << ISP_INT_STORE_DONE_CCE) | (1 << ISP_INT_SHADOW_DONE_CCE) | \
	ISP_IRQ_ERR_MASK1)

#define ISP_INT_LINE_MASK2 \
	((1 << ISP_INT_AFM_RGB_START) | (1 << ISP_INT_AFM_RGB_DONE) | \
	(1 << ISP_INT_FMCU_LOAD_DONE) | (1 << ISP_INT_FMCU_CONFIG_DONE) | \
	(1 << ISP_INT_FMCU_SHADOW_DONE) | ISP_IRQ_ERR_MASK2 | \
	(1 << ISP_INT_NR3_DONE))

#define ISP_INT_LINE_MASK3 \
	((1 << ISP_INT_YUV_DONE) | (1 << ISP_INT_YUV_START) | \
	(1 << ISP_INT_FRGB_DONE) | (1 << ISP_INT_FRGB_START) | \
	(1 << ISP_INT_RRGB_DONE) | (1 << ISP_INT_RRGB_START))

enum isp_pdaf_flag_id {
	ISP_PDAF_START_ID = 0,
	ISP_PDAF_LEFT_DONE_ID = 1,
	ISP_PDAF_RIGHT_DONE_ID = 2,
	ISP_PDAF_MAX_ID

};
enum isp_clk_index {
	ISP_CLK_76M8_INDEX = 0,
	ISP_CLK_256M_INDEX,
	ISP_CLK_307M2_INDEX,
	ISP_CLK_384M_INDEX,
	ISP_CLK_576M_INDEX
};

enum isp_cap_status {
	ISP_CAP_INIT = 0,
	ISP_CAP_START,
	ISP_CAP_STOP
};

#define ISP_PDAF_INIT_FLAG 0

#define ISP_PDAF_START_FLAG \
	(1 << ISP_PDAF_START_ID)

#define ISP_PDAF_LEFT_DONE_FLAG \
	(1 << ISP_PDAF_LEFT_DONE_ID)

#define ISP_PDAF_RIGHT_DONE_FLAG \
	(1 << ISP_PDAF_RIGHT_DONE_ID)

#define ISP_PW_SAVE
enum {
	ISP_ST_STOP = 0,
	ISP_ST_START,
};

enum isp_glb_reg_id {
	ISP_AXI_REG = 0,
	ISP_INIT_MASK_REG,
	ISP_INIT_CLR_REG,
	ISP_REG_MAX
};

enum isp_drv_rtn {
	ISP_RTN_SUCCESS = 0,
	ISP_RTN_PARA_ERR = 0x10,
	ISP_RTN_FRM_DECI_ERR,
	ISP_RTN_OUT_FMT_ERR,
	ISP_RTN_PATH_ADDR_ERR,
	ISP_RTN_PATH_FRAME_LOCKED,
	ISP_RTN_PATH_SC_ERR,
	ISP_RTN_PATH_IN_SIZE_ERR,
	ISP_RTN_PATH_TRIM_SIZE_ERR,
	ISP_RTN_PATH_OUT_SIZE_ERR,
	ISP_RTN_PATH_ENDIAN_ERR,
	ISP_RTN_IRQ_NUM_ERR,
	ISP_RTN_TIME_OUT,
	ISP_RTN_MAX
};

enum isp_path_mode {
	ISP_PRE_ONLINE = 0,
	ISP_PRE_OFFLINE,
	ISP_VID_ONLINE = 0x10,
	ISP_VID_ONLINE_CPP,
	ISP_VID_OFFLINE,
	ISP_CAP_OFFLINE,
	ISP_PRE_ONLINE_CPP,
	ISP_PRE_VID_ONLINE_CPP,
	ISP_MODE_MAX,
};

enum isp_id {
	ISP_ID_0 = 0,
	ISP_ID_1,
	ISP_ID_MAX,
};

enum isp_scl_id {
	ISP_SCL_0 = 0,
	ISP_SCL_PRE,
	ISP_SCL_VID,
	ISP_SCL_CAP,
	ISP_SCL_3DNR,
	ISP_SCL_MAX,
};

enum isp_path_id {
	ISP_PATH_PRE = 0,
	ISP_PATH_VID,
	ISP_PATH_CAP,
	ISP_PATH_MAX
};

enum isp_path_index {
	ISP_PATH_IDX_0 = 0,
	ISP_PATH_IDX_PRE = 1,
	ISP_PATH_IDX_VID = 2,
	ISP_PATH_IDX_CAP = 4,
	ISP_PATH_IDX_ALL = 0x07,
};

enum isp_cfg_id {
	ISP_PATH_INPUT_SIZE,
	ISP_PATH_INPUT_RECT,
	ISP_PATH_OUTPUT_SIZE,
	ISP_PATH_OUTPUT_ADDR,
	ISP_PATH_OUTPUT_RESERVED_ADDR,
	ISP_PATH_STORE_CCE_ADDR,
	ISP_PATH_OUTPUT_FORMAT,
	ISP_PATH_FRM_DECI,
	ISP_PATH_WORK_MODE,
	ISP_PATH_ENABLE,
	ISP_PATH_SHRINK,
	ISP_PATH_DATA_ENDIAN,
	ISP_PATH_ZOOM_MODE,
	ISP_FMCU_VIR_ADDR,
	ISP_PATH_MODE,
	ISP_PATH_SN_MAX_SIZE,
	ISP_PATH_SKIP_NUM,
	ISP_CFG_MAX
};

enum isp_irq_id {
	ISP_PATH_PRE_DONE,
	ISP_PATH_VID_DONE,
	ISP_PATH_CAP_DONE,
	ISP_AEM_DONE,
	ISP_AFL_DONE,
	ISP_AFM_DONE,
	ISP_BINNING_DONE,
	ISP_PDAF_LEFT_DONE,
	ISP_PDAF_RIGHT_DONE,
	ISP_SHADOW_DONE,
	ISP_DCAM_SOF,
	ISP_3DNR_CAP_DONE,
	ISP_IMG_MAX,
};

struct isp_ch_irq {
	int ch0;
	int ch1;
};

struct isp_clk_tag {
	unsigned int clock;
	char *clk_name;
};

struct isp_raw_afm_statistic {
	unsigned int val[ISP_RAW_AFM_ITEM];
};

struct isp_node {
	unsigned int irq_val0;
	unsigned int irq_val1;
	unsigned int irq_val2;
	unsigned int irq_val3;
	unsigned int reserved;
	struct isp_k_time time;
};

struct isp_queue {
	struct isp_node node[ISP_QUEUE_LENGTH];
	struct isp_node *write;
	struct isp_node *read;
};

struct isp_deci_info {
	unsigned int deci_y_eb;
	unsigned int deci_y;
	unsigned int deci_x_eb;
	unsigned int deci_x;
};

struct isp_trim_info {
	unsigned int start_x;
	unsigned int start_y;
	unsigned int size_x;
	unsigned int size_y;
};

struct isp_endian_sel {
	unsigned char y_endian;
	unsigned char uv_endian;
};

struct isp_sc_tap {
	unsigned int y_tap;
	unsigned int uv_tap;
};

struct isp_regular_info {
	unsigned int regular_mode;
	unsigned int shrink_uv_dn_th;
	unsigned int shrink_uv_up_th;
	unsigned int shrink_y_dn_th;
	unsigned int shrink_y_up_th;
	unsigned int effect_v_th;
	unsigned int effect_u_th;
	unsigned int effect_y_th;
	unsigned int shrink_c_range;
	unsigned int shrink_c_offset;
	unsigned int shrink_y_range;
	unsigned int shrink_y_offset;
};

struct isp_scaler_info {
	unsigned int scaler_bypass;
	unsigned int scaler_y_ver_tap;
	unsigned int scaler_uv_ver_tap;
	unsigned int scaler_ip_int;
	unsigned int scaler_ip_rmd;
	unsigned int scaler_cip_int;
	unsigned int scaler_cip_rmd;
	unsigned int scaler_factor_in;
	unsigned int scaler_factor_out;
	unsigned int scaler_ver_ip_int;
	unsigned int scaler_ver_ip_rmd;
	unsigned int scaler_ver_cip_int;
	unsigned int scaler_ver_cip_rmd;
	unsigned int scaler_ver_factor_in;
	unsigned int scaler_ver_factor_out;
	unsigned int scaler_in_width;
	unsigned int scaler_in_height;
	unsigned int scaler_out_width;
	unsigned int scaler_out_height;
	unsigned int *coeff_buf;
};

struct isp_b4awb_buf {
	uint32_t buf_id;
	uint32_t buf_flag;
	unsigned long buf_phys_addr;
};

struct isp_statis_buf_node {
	unsigned long buf_size;
	unsigned int  k_addr;
	unsigned int  u_addr;
	unsigned int  mfd;
};

struct isp_k_block {
	unsigned int lsc_load_buf_id;
	unsigned int lsc_update_buf_id;
	unsigned int full_gamma_buf_id;
	unsigned int yuv_ygamma_buf_id;
	unsigned long full_gamma_buf_addr;
	struct isp_b4awb_buf b4awb_buf[ISP_BING4AWB_NUM];
	unsigned int lsc_buf_phys_addr;
	unsigned int anti_flicker_buf_phys_addr;
	unsigned int raw_nlm_buf_id;
	unsigned int lsc_1d_buf_id;
	unsigned int hsv_buf_id;
	unsigned int lsc_2d_weight_en;
	unsigned int fetch_raw_phys_addr;
	unsigned long fetch_mfd;
	unsigned long lsc_mfd;
	struct pfiommu_info fetch_pfinfo;
	struct pfiommu_info lsc_pfinfo;
	unsigned int is_lsc_param_init_flag;
	spinlock_t lsc_lock;  /* for lsc_updated and SOF int status */
	atomic_t lsc_updated;
};

struct isp_statis_buf {
	unsigned int buf_size;
	int buf_property;
	unsigned long phy_addr;
	unsigned long vir_addr;
	unsigned long kaddr[2];
	unsigned long mfd;
	struct pfiommu_info pfinfo;
};

struct isp_statis_buf_info {
	uint32_t buf_num;
	struct isp_img_size out_size;
	struct isp_statis_buf statis_buf[ISP_STATISTICS_BUF_MAX];
};

struct isp_statis_frm_queue {
	struct isp_statis_buf buf_array[ISP_STATISTICS_QUEUE_LEN];
	unsigned int valid_cnt;
};

struct isp_statis_buf_queue {
	struct isp_statis_buf buff[ISP_STATISTICS_QUEUE_LEN];
	struct isp_statis_buf *write;
	struct isp_statis_buf *read;
	spinlock_t lock;
};

struct isp_frm_queue {
	struct camera_frame frm_array[ISP_FRM_QUEUE_LENGTH];
	unsigned int valid_cnt;
};

struct isp_buf_queue {
	struct camera_frame frame[DCAM_FRM_CNT_MAX];
	struct camera_frame *write;
	struct camera_frame *read;
	int w_index;
	int r_index;
	spinlock_t lock;
};

struct isp_store_info {
	unsigned int bypass;
	unsigned int endian;
	unsigned int speed_2x;
	unsigned int mirror_en;
	unsigned int color_format;
	unsigned int max_len_sel;
	unsigned int shadow_clr;
	unsigned int store_res;
	unsigned int rd_ctrl;
	unsigned int shadow_clr_sel;
	struct camera_size size;
	struct store_border border;
	struct isp_pitch_fs pitch;
};
struct path_cpp_ion_buf {
	struct ion_client *y_client;
	struct ion_client *uv_client;
	struct ion_handle *y_handle;
	struct ion_handle *uv_handle;
	struct sg_table *sg_table;
	struct camera_addr isp_addr;
	struct camera_addr cpp_addr;

};
struct isp_path_desc {
	unsigned int valid;
	unsigned int uv_sync_v;
	unsigned int scaler_bypass;
	unsigned int status;
	unsigned int path_mode;
	unsigned int frm_deci;
	unsigned int output_format;
	unsigned int odata_mode;
	unsigned int frame_base_id;
	unsigned int output_frame_count;
	unsigned int path_sel;
	unsigned int is_clk_update;

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	struct isp_frm_queue cpp_frame_queue;
	#endif
	struct isp_buf_queue buf_queue;
	struct isp_frm_queue frame_queue;

	struct camera_size in_size;
	struct camera_rect in_rect;
	struct camera_size out_size;
	struct camera_size src;
	struct camera_size dst;
	struct isp_endian_sel data_endian;
	struct isp_deci_info deci_info;
	struct isp_trim_info trim0_info;
	struct isp_trim_info trim1_info;
	struct isp_regular_info regular_info;
	struct isp_scaler_info scaler_info;
	struct isp_store_info store_info;

	struct completion sof_com;
	struct completion tx_done_com;

	unsigned int wait_for_done;
	unsigned int is_update;
	unsigned int wait_for_sof;
	unsigned int need_stop;
	unsigned int need_wait;
	unsigned int shadow_done_cnt;
	unsigned int frm_cnt;
	unsigned int skip_num;

};

struct storecce_ion_buf {
	struct ion_client *y_client;
	struct ion_client *uv_client;
	struct ion_handle *y_handle;
	struct ion_handle *uv_handle;
	struct camera_addr addr;
};

struct isp_channel_info {
	unsigned int fetchYUV_bypass;
	unsigned int fetchYUV_format;
	struct isp_pitch_fs fetchYUV_pitch;
	unsigned int store_cce_sdw_done_en;
	unsigned int store_cce_all_done_ctrl;
	unsigned int store_cce_path_sel;
	unsigned int srore_cce_en;
};

struct isp_store_cce_desc {
	unsigned int valid;
	unsigned int status;
	unsigned int output_format;
	unsigned int output_frame_count;
	unsigned int frm_cnt;
	size_t y_buf_len;
	size_t uv_buf_len;
	struct storecce_ion_buf ion_buf[ISP_STORE_CCE_BUF_NUM];
	struct isp_endian_sel data_endian;
	struct camera_size src;
	struct camera_size dst;
	struct store_border border;
	struct isp_buf_queue tmp_buf_queue;
	struct isp_frm_queue frame_queue;
	struct isp_frm_queue zsl_queue;
	struct isp_store_info store_info;
	struct isp_channel_info ch_info;
	unsigned int is_update;
	unsigned int shadow_done_cnt;
	unsigned int read_buf_err;
	unsigned int pw_save;
	unsigned int zsl_pw_save;
	unsigned int is_raw_cap;
};

struct isp_fmcu_slice_desc {
	void *slice_handle;
	unsigned int fmcu_num;
	unsigned int storecce_state;
	unsigned int capture_state;
	unsigned int *iommu_addr_vir;
	unsigned long iommu_addr_phy;
	struct ion_client *fmcu_client;
	struct ion_handle *fmcu_handle;
};

struct isp_fmcu_slw_desc {
	void *slw_handle;
	unsigned int *fmcu_addr_vir;
	unsigned int slw_flags;
	unsigned int status;
	unsigned int vid_num;
};

struct isp_sc_coeff {
	unsigned int buf[ISP_SC_COEFF_BUF_SIZE];
	unsigned int flag;
	struct isp_path_desc path;
	struct isp_store_cce_desc store_cce;
};

struct isp_sc_coeff_queue {
	struct isp_sc_coeff coeff[ISP_SC_COEFF_BUF_COUNT];
	struct isp_sc_coeff *write;
	struct isp_sc_coeff *read;
	int w_index;
	int r_index;
	spinlock_t lock;
};

struct isp_sc_array {
	struct isp_sc_coeff_queue scl0_queue;
	struct isp_sc_coeff_queue pre_queue;
	struct isp_sc_coeff_queue vid_queue;
	struct isp_sc_coeff_queue cap_queue;
	struct isp_sc_coeff_queue store_cce_queue;
	struct isp_sc_coeff coeff[ISP_SCL_MAX];
	unsigned int is_smooth_zoom;
};
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
struct isp_cpp_desc {
	struct path_cpp_ion_buf ion_buf[PATH_CPP_ION_BUF_NUM];
	size_t y_buf_len;
	size_t uv_buf_len;
	struct isp_frm_queue isp_path_queue;
	struct isp_frm_queue isp_cpp_queue;
	struct isp_buf_queue cpp_buf_queue;
	int is_valid;
	struct camera_frame cpp_reserved_frame;
};
#endif
struct isp_module {
	enum isp_id idx;
	struct isp_path_desc isp_path[ISP_SCL_MAX];
	struct camera_frame path_reserved_frame[ISP_SCL_MAX];
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	struct isp_cpp_desc isp_cpp_dev;
#endif
	struct isp_store_cce_desc store_cce;
	struct completion scale_coeff_mem_com;
	struct isp_sc_array *isp_scl_array[ISP_MAX_COUNT];
};

struct isp_statis_module {
	struct isp_statis_frm_queue aem_statis_frm_queue;
	struct isp_statis_frm_queue afl_statis_frm_queue;
	struct isp_statis_frm_queue afm_statis_frm_queue;
	struct isp_statis_frm_queue binning_statis_frm_queue;
	struct isp_statis_frm_queue pdaf_statis_frm_queue;
	struct isp_statis_buf aem_buf_reserved;
	struct isp_statis_buf afl_buf_reserved;
	struct isp_statis_buf afm_buf_reserved;
	struct isp_statis_buf binning_buf_reserved;
	struct isp_statis_buf pdaf_buf_reserved;
	struct isp_statis_buf_queue aem_statis_queue; /*for irq read*/
	struct isp_statis_buf_queue afl_statis_queue; /*for irq read*/
	struct isp_statis_buf_queue afm_statis_queue; /*for irq read*/
	struct isp_statis_buf_queue binning_statis_queue; /*for irq read*/
	struct isp_statis_buf_queue pdaf_statis_queue; /*for irq read*/
	struct isp_statis_buf buf_node;
	struct isp_irq_info irq_info;
	struct isp_statis_buf img_statis_buf;
};
struct isp_cpp_info  {
	struct completion cpp_thread_com;
	struct task_struct *isp_cpp_thread;
	int is_wait_cpp;
	int is_cpp_thread_stop;
};

struct isp_pipe_dev {
	enum isp_id idx;
	unsigned int isp_chn_id[2];
	unsigned int is_raw_capture;
	unsigned int pdaf_status[2];
	unsigned int cap_flag;
	unsigned int cap_scene;
	unsigned int clr_queue;
	unsigned int cap_cur_cnt;
	struct mutex isp_mutex;
	struct completion fmcu_com;
	struct completion irq_com;
	struct completion isr_done_lock;
	struct isp_queue queue;
	struct isp_k_block isp_k_param;
	struct isp_fmcu_slice_desc fmcu_slice;
	struct isp_fmcu_slw_desc fmcu_slw;
	struct isp_module module_info;
	struct isp_statis_module statis_module_info;
	struct camera_frame capture_frame;
	struct completion offline_thread_com;
	struct task_struct *offline_thread;
	atomic_t stop_offline_thread;
	unsigned int is_wait_fmcu;
	struct pfiommu_info pfinfo_3dnr[3];
	unsigned int clk_update_count;
	enum isp_cap_status  cap_status_flag;
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	struct isp_cpp_info cpp_info;
#endif
};

typedef void(*isp_isr)(enum isp_id idx, void *param);
typedef int(*isp_isr_func)(struct camera_frame *frame, void *param);

int sprd_isp_stop(void *isp_handle, int is_irq);
int sprd_isp_stop_path(void *isp_handle, enum isp_path_index path_index);
int sprd_isp_fmcu_slice_proc(void *handle,
		unsigned int cap_flag, unsigned int is_irq);
#ifdef ISP_PW_SAVE
int sprd_isp_store_cce_bypass(void *handle,
		unsigned int cap_flag, unsigned int bypass);
#endif
int sprd_isp_start_fmcu(void *handle,
		struct sprd_img_capture_param capture_param,
		unsigned int is_irq);
void sprd_isp_reset_fmcu(void);
int sprd_isp_fmcu_slice_stop(void *handle);
int sprd_isp_start(void *isp_handle);
int sprd_isp_start_path(void *isp_handle, enum isp_path_index path_index);
int sprd_isp_update_path(void *isp_handle,
			enum isp_path_index path_index,
			struct camera_size *in_size,
			struct camera_rect *in_rect,
			struct camera_size *out_size);
int set_isp_path_cfg(void *isp_handle, enum isp_path_index path_index,
	enum isp_cfg_id id, void *param);
int sprd_isp_module_en(void *isp_handle, enum isp_id idx);
int sprd_isp_module_dis(void *isp_handle, enum isp_id idx);
int sprd_isp_dev_init(void **isp_pipe_dev_handle, enum isp_id idx);
int sprd_isp_dev_deinit(void *isp_dev_handle, enum isp_id idx);
int sprd_isp_drv_init(struct platform_device *pdev);
void sprd_isp_drv_deinit(void);
int sprd_isp_parse_dt(struct device_node *dn, unsigned int *isp_count);
int sprd_isp_k_ioctl(void *isp_dev_handle, unsigned int cmd,
	unsigned long param);
int sprd_isp_reg_isr(enum isp_id idx, enum isp_irq_id irq_id,
	isp_isr_func user_func, void *user_data);
int isp_cfg_param(void *param,
	struct isp_k_block *isp_k_param, enum isp_id idx);
void isp_reg_trace(enum isp_id idx);
int32_t isp_capability(void *param);
int isp_path_scaler(struct isp_module *module,
		enum isp_path_index path_index,
		struct isp_path_desc *path,
		struct isp_sc_coeff *coeff);
void isp_wait_update_done(struct isp_module *module,
			enum isp_path_index path_index,
			unsigned int *p_flag);

int sprd_isp_update_clk(int clk_index, struct device_node *dn);
int sprd_isp_clock_update(struct isp_pipe_dev *dev,
		    int clk_index);

#endif
