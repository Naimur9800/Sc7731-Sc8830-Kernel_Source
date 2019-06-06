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

#include <linux/types.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/kfifo.h>
#include <video/sprd_img.h>

#include "sprd_isp_hw.h"
#include "isp_reg.h"
#include "dcam_drv.h"
#include "cam_iommu.h"

#define ISP_PATH_FRAME_WIDTH_MAX		4224
#define ISP_PATH_FRAME_HEIGHT_MAX		3168
#define ISP_PATH1_LINE_BUF_LENGTH		2592
#define ISP_PATH2_LINE_BUF_LENGTH		2592
#define ISP_PATH3_LINE_BUF_LENGTH		2592

#define ISP_FMCU_CMD_Q_SIZE		0x4000
#define ISP_QUEUE_LENGTH			16
#define ISP_SCL0_MAX_WIDTH			2304
#define ISP_ONLINE_ZFACTOR_MAX		2
#define ISP_BUF_QUEUE_LENGTH		16
#define ISP_BING4AWB_NUM			2
#define ISP_STATISTICS_BUF_MAX		4
#define ISP_STATISTICS_QUEUE_LEN	8
#define ISP_IMG_OUTPUT_PATH_MAX		3
#define ISP_FRM_QUEUE_LENGTH		7
#define ISP_PIXEL_ALIGN_WIDTH		4
#define ISP_PIXEL_ALIGN_HEIGHT		2
#define ISP_RAW_AFM_ITEM		40
#define ISP_FRGB_GAMMA_BUF_SIZE     (257 * 4 + 4)
#define ISP_NLM_BUF_SIZE            (1024 * 4 + 4)
#define LENS_W_BUF_SIZE             (512)

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

#define ISP_LSC_BUF_SIZE			(32 * 1024)

#define OFFLINE_BUFFER_NUM	2

#define ISP_OFF_BUF_BIN		0x00UL
#define ISP_OFF_BUF_FULL	0x01UL
#define ISP_OFF_BUF_BOTH	0x02UL
#define ISP_OFF_BUF_NONE	0x03UL

/* Dynamic switch isp clk */
#define ISP_CLK_P_P		BIT_0	/* for normale,preview, video */
#define ISP_CLK_P_C		BIT_1	/* for capture */
#define ISP_CLK_P_S		BIT_2	/* for slow motion */

/* isp iommu channel r/w type */
#define ISP_IOMMU_CH_AR	SPRD_IOMMU_EX_CH_READ
#define ISP_IOMMU_CH_AW	SPRD_IOMMU_EX_CH_WRITE

/* isp outstanding mask */
#define ISP_AXI_ITI2AXIM_ROSTD_MASK 0xFF00

/* AXI ID for isp */
/* rd */
#define AR_ID_CFG	(BIT_0)
#define AR_ID_BPC	(BIT_1)
#define AR_ID_LSC	(BIT_2)
#define AR_ID_FMCU	(BIT_3)
#define AR_ID_FETCH_Y	(BIT_4)
#define AR_ID_FETCH_U	(BIT_5)
#define AR_ID_FETCH_V	(BIT_6)

/* wr */
#define AW_ID_STORE_O_Y	(BIT_0)
#define AW_ID_STORE_O_U	(BIT_1)
#define AW_ID_STORE_O_V	(BIT_2)
#define	AW_ID_AEM	(BIT_3)
#define	AW_ID_AFL_GLB	(BIT_4)
#define	AW_ID_AFL_REGION	(BIT_5)
#define	AW_ID_BINNING	(BIT_6)
#define	AW_ID_BPC	(BIT_7)
#define	AW_ID_STORE_VID_Y	(BIT_8)
#define	AW_ID_STORE_VID_U	(BIT_9)
#define	AW_ID_STORE_VID_V	(BIT_10)
#define	AW_ID_STORE_VID_YUV	\
	(AW_ID_STORE_VID_Y | AW_ID_STORE_VID_U | AW_ID_STORE_VID_V)
#define	AW_ID_STORE_PRE_CAP_Y	(BIT_11)
#define	AW_ID_STORE_PRE_CAP_U	(BIT_12)
#define	AW_ID_STORE_PRE_CAP_V	(BIT_13)
#define	AW_ID_STORE_PRE_CAP_YUV	\
	(AW_ID_STORE_PRE_CAP_Y | AW_ID_STORE_PRE_CAP_U | AW_ID_STORE_PRE_CAP_V)

#define	ISP_SBLK_MAP_CNT 3
#define ISP_SBLK_MAP_SIZE (sizeof(int) * BITS_PER_BYTE) /* unit: bit */

/* get map id by number of sub-blk */
#define isp_dbg_g_sblk_map(nr)  ((nr) / ISP_SBLK_MAP_SIZE)
/* get bit site in a word */
#define isp_dbg_g_sblk_site(nr) ((nr) % ISP_SBLK_MAP_SIZE)
/* update map using byp_flag on bit_site */
#define isp_dbg_u_sblk_map(map, bit_site, byp_flag) \
	(((map) & ~(1 << (bit_site))) | ((byp_flag) << (bit_site)))

/**
 * for debug purpose,
 * get bypass_flag, setting from sysfs, in bitmap for one sub-block.
 * @nr:		the number of bit in bitmap of isp sblk, from 0 to ISP_SBLK_CNT,
 * @byp_flag:	save the bypass flag achieved from bitmap.
 */
#define isp_dbg_g_sblk_byp_flag(nr, byp_flag)	\
do {						\
	unsigned int map, map_id, site;		\
						\
	if (byp_flag < SBLK_BYP_FLAG_CNT && nr < ISP_SBLK_CNT) {\
		map_id = isp_dbg_g_sblk_map((nr));		\
		site = isp_dbg_g_sblk_site((nr));		\
		map = isp_dbg->sblk_maps[map_id];	\
		byp_flag = (map & (1 << site)) >> site;	\
	}						\
} while (0)

/**
 * for debug purpose,
 * set bypass flag in bitmap for one sub-block, via sysfs dynamic control.
 * @nr:		the number of bit in bitmap of isp sblk, from 0 to ISP_SBLK_CNT,
 * @byp_flag:	the bypass flag seting value: 0/1, from sysfs.
 */
#define isp_dbg_s_sblk_byp_flag(nr, byp_flag)			\
do {								\
	unsigned int map_id, site, old_map, new_map;		\
								\
	if (byp_flag < SBLK_BYP_FLAG_CNT && nr < ISP_SBLK_CNT) {	\
		map_id = isp_dbg_g_sblk_map((nr));			\
		site = isp_dbg_g_sblk_site((nr));			\
		old_map = isp_dbg->sblk_maps[(map_id)];		\
		new_map = isp_dbg_u_sblk_map(old_map, site, byp_flag);\
		isp_dbg->sblk_maps[map_id] = new_map;		\
	}							\
} while (0)

/**
 * for debug purpose,
 * set bypass flag from sysfs, for all sub-blocks,
 * @byp_flag: the bypass flag seting value: 0/1, from sysfs.
 */
#define isp_dbg_s_all_sblk_byp_flag(byp_flag)			\
do {								\
	unsigned int new_map, map_id;				\
	unsigned int map_cnt = DIV_ROUND_UP(ISP_SBLK_CNT, ISP_SBLK_MAP_SIZE);\
								\
	if (byp_flag < SBLK_BYP_FLAG_CNT) {				\
		if ((byp_flag) == SBLK_BYPASS)			\
			new_map = 0xffffffff;			\
		else						\
			new_map = 0x0;				\
								\
		for (map_id = 0; map_id < map_cnt; map_id++)	\
			isp_dbg->sblk_maps[map_id] = new_map;	\
	}							\
} while (0)

/**
 * for debug purpose,
 * set the original bypass state of each isp sub-blocks.
 * @bid: the index of isp sub-block
 * @iid: the index of isp_dev
 */
#define isp_dbg_s_ori_byp(byp_flag, bid, iid)			\
do {								\
	unsigned int map_id, site, old_map, new_map;		\
	struct isp_pipe_dev *isp_dev = NULL;			\
								\
	if (CHECK_ID_VALID(iid) && bid < ISP_SBLK_CNT &&	\
	    byp_flag < SBLK_BYP_FLAG_CNT) {			\
		isp_dev = g_isp_dev_parray[(iid)];		\
		if (!IS_ERR_OR_NULL(isp_dev)) {			\
			map_id = isp_dbg_g_sblk_map((bid));	\
			site = isp_dbg_g_sblk_site((bid));	\
			old_map = isp_dev->sblk_ori_byp_map[map_id];\
			new_map = isp_dbg_u_sblk_map(old_map, site, byp_flag);\
			isp_dev->sblk_ori_byp_map[map_id] = new_map;\
		}						\
	}							\
} while (0)

enum isp_sblk_byp_flag {
	SBLK_WORK,
	SBLK_BYPASS,
	SBLK_BYP_FLAG_CNT,
};

enum isp_sblk {
	/* RAW RGB */
	raw_pgg,
	raw_blc,
	raw_rgbg,
	raw_rgbd,
	raw_postblc,
	raw_nlc,
	raw_2dlsc,
	raw_bin,
	raw_awb,
	raw_aem,
	raw_bpc,
	raw_grgbc,
	raw_vst,
	raw_nlm,
	raw_ivst,
	raw_rlsc,
	raw_afm,

	/* FULL RGB */
	full_cmc,
	full_gama,
	full_hsv,
	full_pstrz,
	full_uvd,

	/* YUV */
	yuv_afl,
	yuv_precdn,
	yuv_ynr,
	yuv_brta,
	yuv_cnta,
	yuv_hist,
	yuv_hist2,
	yuv_cdn,
	yuv_edge,
	yuv_csa,
	yuv_hua,
	yuv_postcdn,
	yuv_gama,
	yuv_iircnr,
	yuv_random,
	yuv_nf,

	ISP_SBLK_CNT,
};

enum isp_wait_full_tx_done_state {
	WAIT_CLEAR,
	WAIT_BEGIN,
	WAIT_DONE,
};

/* isp CFG mode is default */
enum isp_work_mode {
	ISP_CFG_MODE,
	ISP_AP_MODE,
	ISP_WM_MAX
};

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
	ISP_MODE_MAX,
};

/* definition of isp instance id */
enum isp_id {
	ISP_ID_0 = 0,
	ISP_ID_1,
	ISP_ID_MAX,
};

/* definition of scene id for each isp instance */
enum isp_scene_id {
	ISP_SCENE_PRE,
	ISP_SCENE_CAP,
	ISP_SCENE_NUM,
};

enum isp_scl_id {
	ISP_SCL_0 = 0,
	ISP_SCL_PRE,
	ISP_SCL_VID,
	ISP_SCL_CAP,
	ISP_SCL_MAX,
};

enum isp_path_id {
	ISP_PATH_PRE = 0,
	ISP_PATH_VID,
	ISP_PATH_CAP,
	ISP_PATH_MAX
};

enum isp_path_index {
	ISP_PATH_IDX_0 = 0x00,
	ISP_PATH_IDX_PRE = 0x01,
	ISP_PATH_IDX_VID = 0x02,
	ISP_PATH_IDX_CAP = 0x04,
	ISP_PATH_IDX_ALL = 0x07,
};

enum isp_cfg_id {
	ISP_PATH_INPUT_SIZE,
	ISP_PATH_INPUT_RECT,
	ISP_PATH_INPUT_FORMAT,
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
	ISP_PATH_MODE,
	ISP_PATH_SN_MAX_SIZE,
	ISP_CFG_MAX
};

enum isp_irq_id {
	ISP_PATH_PRE_DONE,
	ISP_PATH_VID_DONE,
	ISP_PATH_CAP_DONE,
	ISP_AEM_DONE,
	ISP_AEM_ERROR,
	ISP_AFL_DONE,
	ISP_AFM_DONE,
	ISP_BINNING_DONE,
	ISP_SHADOW_DONE,
	ISP_DCAM_SOF,
	ISP_HIST_DONE,
	ISP_HIST2_DONE,
	ISP_IMG_MAX,
};

enum isp_fetch_format {
	ISP_FETCH_YUV422_3FRAME = 0,
	ISP_FETCH_YUYV,
	ISP_FETCH_UYVY,
	ISP_FETCH_YVYU,
	ISP_FETCH_VYUY,
	ISP_FETCH_YUV422_2FRAME,
	ISP_FETCH_YVU422_2FRAME,
	ISP_FETCH_RAW_10,
	ISP_FETCH_CSI2_RAW_10,  /* MIPI RAW */
	ISP_FETCH_FULL_RGB,
	ISP_FETCH_YUV420_2FRAME = 10,
	ISP_FETCH_YVU420_2FRAME,
	ISP_FETCH_FORMAT_MAX
};

enum isp_store_format {
	ISP_STORE_UYVY = 0x00,
	ISP_STORE_YUV422_2FRAME,
	ISP_STORE_YVU422_2FRAME,
	ISP_STORE_YUV422_3FRAME,
	ISP_STORE_YUV420_2FRAME,
	ISP_STORE_YVU420_2FRAME,
	ISP_STORE_YUV420_3FRAME,
	ISP_STORE_RAW10,
	ISP_STORE_FULL_RGB8,
	ISP_STORE_FORMAT_MAX
};

enum isp_fmcu_cmd {
	P0_SDW_DONE = 0x10,	/* shadow done */
	P0_ALL_DONE,		/* all done */
	P0_LLD_DONE,		/* lens load done */
	C0_SDW_DONE,
	C0_ALL_DONE,
	C0_LLD_DONE,
	P1_SDW_DONE,
	P1_ALL_DONE,
	P1_LLD_DONE,
	C1_SDW_DONE,
	C1_ALL_DONE,
	C1_LLD_DONE,
	CFG_TRIGGER_PULSE,	/* CFG info FMCU page reg seting done*/
	SW_TRIGGER,		/* FPGA debug use*/
	FMCU_CMD_MAX
};

struct isp_clk_gate {
	unsigned int g0;
	unsigned int g1;
	unsigned int g2;
	unsigned int g3;
};

struct isp_ch_irq {
	int ch0;
	int ch1;
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

#define ISP_BUF_SHORT_NAME_LEN	16

struct isp_buf_info {
	char name[ISP_BUF_SHORT_NAME_LEN+1];
	size_t size;
	void *client;
	void *handle;
	void *sw_addr;
	void *hw_addr;
};

struct isp_k_block {
	unsigned int lsc_bypass;
	unsigned int lsc_cap_grid_width;
	unsigned int lsc_cap_grid_pitch;
	unsigned int lsc_load_buf_id;
	unsigned int lsc_update_buf_id;
	unsigned int full_gamma_buf_id;
	unsigned int yuv_ygamma_buf_id;
	unsigned int lsc_buf_phys_addr;
	unsigned int anti_flicker_buf_phys_addr;
	unsigned int raw_nlm_buf_id;
/*  TODO: lsc_1_buf_id replaced with rlsc */
	unsigned int lsc_1d_buf_id;
	unsigned int rlsc_buf_id;
	unsigned int hsv_buf_id;
	unsigned int lsc_2d_weight_en;
	unsigned int fetch_raw_phys_addr;
	unsigned long full_gamma_buf_addr;
	void *nlm_vst_addr;
	void *nlm_ivst_addr;
	void *isp_lens_w_addr;
	void *dcam_lens_w_addr;
	unsigned long fetch_mfd;
	unsigned long lsc_mfd;
	struct isp_b4awb_buf b4awb_buf[ISP_BING4AWB_NUM];
	struct pfiommu_info fetch_pfinfo;
	struct pfiommu_info lsc_pfinfo;
	struct isp_buf_info lsc_buf_info;
};

struct isp_statis_buf {
	unsigned int buf_size;
	int buf_property;
	unsigned long phy_addr;
	unsigned long vir_addr;
	unsigned long addr_offset;
	unsigned long kaddr[2];
	struct pfiommu_info pfinfo;
};

struct isp_statis_buf_info {
	uint32_t buf_num;
	struct isp_img_size out_size;
	struct isp_statis_buf statis_buf[ISP_STATISTICS_BUF_MAX];
};

struct isp_statis_frm_queue {
	/* double the size because kfifo needs size as power of 2 */
	struct isp_statis_buf buf_array[ISP_STATISTICS_QUEUE_LEN * 2];
	struct kfifo fifo;
	spinlock_t lock;
};

struct isp_statis_buf_queue {
	struct isp_statis_buf buff[ISP_STATISTICS_QUEUE_LEN];
	struct isp_statis_buf *write;
	struct isp_statis_buf *read;
	spinlock_t lock;
};

struct isp_frm_queue {
	const char *owner;
	struct camera_frame frame[ISP_FRM_QUEUE_LENGTH + 1];
	int w_index;
	int r_index;
	unsigned int valid_cnt;
	spinlock_t lock;
};

struct isp_buf_queue {
	struct camera_frame frame[DCAM_FRM_CNT_MAX + 1];
	int w_index;
	int r_index;
	unsigned int valid_cnt;
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

struct isp_path_desc {
	unsigned int valid;
	unsigned int uv_sync_v;
	unsigned int scaler_bypass;
	unsigned int status;
	unsigned int path_mode;
	unsigned int frm_deci;
	unsigned int input_format;
	unsigned int output_format;
	unsigned int odata_mode;
	unsigned int frame_base_id;
	unsigned int output_frame_count;
	unsigned int path_sel;

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
};

struct offline_ion_buf {
	struct ion_client *client;
	struct ion_handle *handle;
	struct camera_addr addr;
};

struct offline_buf_desc {
	size_t buf_len;
	struct offline_ion_buf ion_buf[ISP_FRM_QUEUE_LENGTH + 1];
	struct isp_buf_queue tmp_buf_queue;
	struct isp_frm_queue frame_queue;
	struct isp_frm_queue zsl_queue;
	unsigned int output_format;
	unsigned int output_frame_count;
};

struct isp_offline_desc {
	unsigned int valid;
	unsigned int status;
	struct offline_buf_desc buf_desc_bin;
	struct offline_buf_desc buf_desc_full;
	struct isp_endian_sel data_endian;
	struct camera_size src;
	struct camera_size dst;
	struct store_border border;
	struct isp_store_info store_info;
	unsigned int is_update;
	unsigned int shadow_done_cnt;
	unsigned int read_buf_err;
};

struct isp_fmcu_slice_desc {
	void *slice_handle;
	unsigned int fmcu_num;
	struct isp_buf_info cmdq_buf_info;
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
	struct isp_sc_coeff_queue pre_queue;
	struct isp_sc_coeff_queue vid_queue;
	struct isp_sc_coeff_queue cap_queue;
	struct isp_sc_coeff coeff[ISP_SCL_MAX];
	unsigned int is_smooth_zoom;
	struct isp_offline_desc scl_off_desc;
};

struct isp_module {
	struct isp_path_desc isp_path[ISP_SCL_MAX];
	struct camera_frame path_reserved_frame[ISP_SCL_MAX];
	struct isp_offline_desc off_desc;
	struct isp_cctx_desc *cctx_desc;
	struct isp_sc_array *scl_array;
};

struct isp_statis_module {
	uint32_t statis_valid;
	struct isp_statis_frm_queue afl_statis_frm_queue;
	uint32_t afl_statis_cnt;
	struct isp_statis_frm_queue afm_statis_frm_queue;
	uint32_t afm_statis_cnt;
	struct isp_statis_frm_queue binning_statis_frm_queue;
	uint32_t binning_statis_cnt;
	struct isp_statis_frm_queue hist_statis_frm_queue;
	uint32_t hist_statis_cnt;
	struct isp_statis_frm_queue hist2_statis_frm_queue;
	uint32_t hist2_statis_cnt;
	struct isp_statis_buf aem_buf_reserved;
	struct isp_statis_buf afl_buf_reserved;
	struct isp_statis_buf afm_buf_reserved;
	struct isp_statis_buf binning_buf_reserved;
	struct isp_statis_buf hist_buf_reserved;
	struct isp_statis_buf hist2_buf_reserved;
	struct isp_statis_buf_queue aem_statis_queue; /*for irq read*/
	struct isp_statis_buf_queue afl_statis_queue; /*for irq read*/
	struct isp_statis_buf_queue afm_statis_queue; /*for irq read*/
	struct isp_statis_buf_queue binning_statis_queue; /*for irq read*/
	struct isp_statis_buf_queue hist_statis_queue; /*for irq read*/
	struct isp_statis_buf_queue hist2_statis_queue; /*for irq read*/
	struct camera_frame afm_frame_info;
};


struct isp_pipe_dev {
	/*
	 * composite idx(com_idx) is divided into 3 parts:
	 * @scene_id, pre(0) or cap(1)
	 * @isp_work_mode(0: cfg mode, 1:ap mode),
	 * @isp_id, isp instance id
	 * each part has 4bits.
	 *
	 * MSB                              LSB
	 * |----4-----|----4------|-----4------|
	 * | scene id | work_mode |   isp id   |
	 *
	 * i.e. idx = isp_id | work_mode << 4 | scene_id << 8
	 */
	unsigned int com_idx;
	unsigned int fmcu_owner; /* record the owner of fmcu */
	unsigned int cap_on;
	unsigned int cap_flag;
	unsigned int frm_cnt;
	unsigned int pre_state;
	unsigned int bin_path_miss_cnt; /* record the miss cnt of bin_tx done */
	unsigned int is_raw_capture;
	atomic_t cfg_map_lock;
	struct mutex isp_mutex;
	struct completion fmcu_com;
	struct completion irq_com;
	struct completion isr_done_lock;
	struct completion bin_stop;
	struct completion full_stop;
	struct isp_queue queue;
	struct isp_k_block isp_k_param;
	struct isp_fmcu_slice_desc fmcu_slice;
	struct isp_fmcu_slw_desc fmcu_slw;
	struct isp_module module_info;
	struct isp_statis_module statis_module_info;
	struct camera_frame offline_frame[2];
	struct completion offline_bin_thread_com;
	struct completion offline_full_thread_com;
	struct task_struct *offline_bin_thread;
	struct task_struct *offline_full_thread;
	unsigned int is_offline_bin_thread_stop;
	unsigned int is_offline_full_thread_stop;
	unsigned int *fmcu_addr_vir;
	unsigned long fmcu_addr_phy;
	unsigned int is_wait_fmcu;
	unsigned int is_3dnr;
	unsigned int is_hdr;
	/* set this flag when isp waiting for dcam full path tx done */
	enum isp_wait_full_tx_done_state wait_full_tx_done;
	unsigned int frm_cnt_3dnr;
	struct camera_group *cam_grp;
	spinlock_t pre_lock;
	spinlock_t cap_lock;
	/*
	 * used to save the original bypass configs from hal/mw,
	 * each bit for one sub-block.
	 * 1, bypass; 0, work
	 */
	unsigned int sblk_ori_byp_map[ISP_SBLK_MAP_CNT];

	/* ISP flooding */
	int isr_count;
	unsigned long isr_last_time;

	/* distance between matched frame */
	int delta_full;
};

typedef void(*isp_isr)(void *param);
typedef int(*isp_isr_func)(struct camera_frame *frame, void *param);

int sprd_isp_stop(void *isp_handle, int is_irq);
int sprd_isp_start_pipeline_bin(void *handle, unsigned int cap_flag);
int sprd_isp_start_pipeline_full(void *handle, unsigned int cap_flag);
int sprd_isp_stop_pipeline(void *handle);
int sprd_isp_force_stop_pipeline(void *handle);
int sprd_isp_start(void *isp_handle, struct camera_frame *frame);
int sprd_isp_get_afm_frame_info(void *isp_handle,
		struct camera_frame **out_frame);
int sprd_isp_get_offline_buffer(void *isp_handle,
	uint8_t off_type, struct camera_frame *out_frame);
int sprd_isp_set_offline_buffer(void *isp_handle, uint8_t off_type);
int sprd_isp_update_zoom_param(void *isp_handle,
			       enum isp_path_index path_index,
			       struct camera_size *in_size,
			       struct camera_rect *in_rect,
			       struct camera_size *out_size);
int set_isp_path_cfg(void *isp_handle, enum isp_path_index path_index,
	enum isp_cfg_id id, void *param);
int sprd_isp_module_en(void *isp_handle, enum isp_id iid);
int sprd_isp_module_dis(void *isp_handle, enum isp_id iid);
int sprd_isp_dev_init(void **isp_pipe_dev_handle, enum isp_id iid);
int sprd_isp_dev_deinit(void *isp_dev_handle, enum isp_id iid);
int sprd_isp_drv_init(struct platform_device *pdev);
void sprd_isp_drv_deinit(void);
int sprd_isp_parse_dt(struct device_node *dn, unsigned int *isp_count);
int sprd_isp_k_ioctl(void *isp_dev_handle, unsigned int cmd,
	unsigned long param);
int sprd_isp_reg_isr(enum isp_id iid,  enum isp_irq_id irq_id,
	isp_isr_func user_func, void *user_data);
int sprd_isp_slw_flags_init(void *isp_handle, struct isp_path_info *info);
void isp_irq_ctrl(struct isp_pipe_dev *dev, bool enable);
int isp_cfg_param(void *param,
	struct isp_k_block *isp_k_param, struct isp_pipe_dev *dev);
void isp_dbg_reg_trace(struct isp_pipe_dev *dev, unsigned int idx);
void isp_dbg_bypass_sblk(struct isp_pipe_dev *dev, unsigned int idx);
void isp_dbg_dump_fmcu_cmd_q(struct isp_pipe_dev *dev);
int32_t isp_k_capability(void __user *param);
int isp_path_scaler(struct isp_module *module,
		    enum isp_path_index path_index,
		    struct isp_path_desc *path,
		    struct isp_sc_coeff *coeff);
void isp_wait_update_done(struct isp_module *module,
			  enum isp_path_index path_index, unsigned int *p_flag);
unsigned int isp_k_fetch_get_raw_phys_addr(void);
extern uint32_t int_reg_base[][ISP_SCENE_NUM];
unsigned int isp_k_fetch_get_raw_info(unsigned int *width,
	unsigned int *height);

int sprd_isp_external_unmap(void *isp_handle);
int isp_path_cap_with_vid_set_next_frm(struct isp_pipe_dev *dev);

extern unsigned int is_dual_cam;
extern bool has_dual_cap_started;
extern unsigned int fmcu_slice_capture_state;
extern unsigned int isp_frm_queue_len;
extern struct isp_pipe_dev *g_isp_dev_parray[ISP_MAX_COUNT];
extern struct isp_clk_gate isp_clk_gt;
void isp_clk_pause(enum isp_id iid, int i);
void isp_clk_resume(enum isp_id iid, int i);
void isp_handle_dcam_err(void *data);
#endif
