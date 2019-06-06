/*
* File: Altek_AP3AInfo.h                                               *
* Description: 3A data structure shared                                *
*		by user-space and kernel space                         *
*                                                                      *
* (C)Copyright altek Corporation 2016                                  *
*                                                                      *
* History:                                                             *
*   2016/01/015; Caedmon Lai; Initial version                          *
*/
#ifndef _ALTEK_AP3AINFO_SHARED_STRUCT_H
#define _ALTEK_AP3AINFO_SHARED_STRUCT_H

#if 0
/*
 *brief mid mode
 */
enum mid_mode {
	/* 5x5 mask */
	MF_51_MODE,
	/* 3x3 mask */
	MF_31_MODE,
	MF_DISABLE
};
#endif
/* SPRD will copy the 3A config data structure to
*  a shared folder for user space and kernel space
*/
/* #define MAXSEQLEN 20*/
#if 0
/*
 *@typedef statistics_dld_region
 *@brief Statistics download region
 */
#pragma pack(push)
#pragma pack(4)
struct statistics_dld_region {
	/* Used to control ROI of width,
	* if set 50 means use half image to get HW3A stats data, suggest 100
	*/
	u16 border_ratio_x;
	/* Used to control ROI of height,
	*  if set 50 means use half image to get HW3A stats data, suggest 100
	*/
	u16 border_ratio_y;
	/* Block number of horizontal direction of RAW iamge */
	u16 blk_num_x;
	/* Block number of vertical direction of RAW image */
	u16 blk_num_y;
	/* Used to control ROI shift position ratio of width,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_x;
	/* Used to control ROI shift position ratio of height,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_y;
};
#pragma pack(pop)

/*
 *@typedef ae_cfg_info
 *@brief AE configuration information
 */
#pragma pack(push)
#pragma pack(4)
struct ae_cfg_info {
	u16 token_id;
	struct statistics_dld_region ae_region;
};
#pragma pack(pop)


/*
 *@structawbhis
 *@brief AWB histogram Parameters setting
 */
#pragma pack(push)
#pragma pack(4)
struct awbhis {
	/* Enable HW3A range histogram, Suggest set = TRUE */
	bool  enable;
	/* Cr Range Start, Suggest set =  -46 */
	s8 cr_start;
	/* Cr Range End, Suggest set = 110 */
	s8 cr_end;
	/* Offset Range Up, Suggest set = 10 */
	s8 offset_up;
	/* Offset Range Down, Suggest set = -90 */
	s8 offset_down;
	/* Purple compensation, Suggest set = 0 */
	s8 cr_purple;
	/* Purple compensation, Suggest set = 2 */
	u8 offset_purple;
	/* Grass compensation--offset, Suggest set = -22 */
	s8 grass_offset;
	/* Grass compensation--cr left, Suggest set = -30 */
	s8 grass_start;
	/* Grass compensation--cr right, Suggest set = 25 */
	s8 grass_end;
	/* Grass compensation, Suggest set = 4 */
	u8 damp_grass;
	/* Weighting around bbr, Suggest set = -2 */
	s8 offset_bbr_w_start;
	/* Weighting around bbr, Suggest set = 2 */
	s8 offset_bbr_w_end;
	/* Weighting around bbr, Suggest set = 2 */
	u8 yfac_w;
	/* HisInterp=(ucOffsetPurPle - cOffsetUp)*1024/(cCrPurple-cCrStart),
	*  Note: (cCrPurple-cCrStart) !=0, Suggest set = -178
	*/
	u32 his_interp;
};
#pragma pack(pop)


/*
 *@typedef awb_cfg_info
 *@brief AWB configuration information
 */
#pragma pack(push)
#pragma pack(4)
struct awb_cfg_info {
	u16 token_id;
	struct statistics_dld_region awb_region;
	/* Y factor weighting table,
	*{0,0,0,4,7,10,12,14,15,15,15,15,14,13,10,5};
	*/
	u8 y_factor[16];
	/* BBR curve table,
	*{22,20,18,16,15,13,11,10,8,8,6,5,3,1,-1,-3,-4,-5,-6,
	*-7,-8,-9,-10,-11,-12,-13,-14,-15,-16,-18,-18,-18,-18};
	*/
	s8 bbr_factor[33];
	/* Calib R Gain w/ Altek format */
	u16  r_gain;
	/* Calib G Gain w/ Altek format, Suggest set = 0 */
	u16  g_gain;
	/* Calib B Gain w/ Altek format */
	u16  b_gain;
	/* Data-shift for fitting 8bit, Suggest set = 100 */
	u8   cr_shift;
	/* Data-shift for fitting 8bit, Suggest set = 100 */
	u8   offset_shift;
	/* HW parameter, suggest 0 */
	u8   quantize;
	/* HW parameter, suggest 7 */
	u8   damp;
	/* HW parameter, suggest 5 */
	u8   sum_shift;
	struct awbhis t_his;
	/* Calib R Gain w/ format 7bit. */
	u16  r_linear_gain;
	/* Calib B Gain w/ format 7bit. */
	u16  b_linear_gain;
};
#pragma pack(pop)

/*
 *@typedef af_statistics_dld_region
 *@brief AF Statistics download region
 */
#pragma pack(push)
#pragma pack(4)
struct af_statistics_dld_region {
       /* Used to control ROI of width,
	*if set 50 means use half image to get HW3A stats data
	*/
	u16 size_ratio_x;
       /* Used to control ROI of height,
	* if set 50 means use half image to get HW3A stats data
	*/
	u16 size_ratio_y;
	/* Block number of horizontal direction of ROI */
	u16 blk_num_x;
	/* Block number of vertical direction of ROI */
	u16 blk_num_y;
	/* Used to control ROI shift position ratio of width,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_x;
	/* Used to control ROI shift position ratio of height,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_y;
};
#pragma pack(pop)

/**
 *@struct
 *@brief af_hw3aa_lpf
 */
#pragma pack(push)
#pragma pack(4)
struct af_hw3aa_lpf {
	u32 af_lpf_b0;
	u32 af_lpf_b1;
	u32 af_lpf_b2;
	u32 af_lpf_b3;
	u32 af_lpf_b4;
	u32 af_lpf_b5;
	u32 af_lpf_b6;
	u32 af_lpf_shift;
};
#pragma pack(pop)

#pragma pack(push)
#pragma pack(4)
struct af_hw3aa_iir {
	bool af_hpf_initbyuser;
	u32 a3_a_af_hpf_p;
	u32 a3_a_af_hpf_q;
	u32 a3_a_af_hpf_r;
	u32 a3_a_af_hpf_s;
	u32 a3_a_af_hpf_t;
	u32 a3_a_af_hpf_abs_shift;
	u32 a3_a_af_hpf_th;
	u32 a3_a_af_hpf_init_a;
	u32 a3_a_af_hpf_init_b;
};
#pragma pack(pop)

/**
 *@struct
 *@brief af_hw3aa_pseudoy
 */
#pragma pack(push)
#pragma pack(4)
struct af_hw3aa_pseudoy {
	u32 af_py_wr;
	u32 af_py_wgr;
	u32 af_py_wgb;
	u32 af_py_wb;
	u32 af_py_shift;
	u32 af_py_offset;
	u32 af_py_gain;
};
#pragma pack(pop)

/*
 *@typedef af_cfg_info
 *@brief AF configuration information
 */
#pragma pack(push)
#pragma pack(4)
struct af_cfg_info {
	u16 token_id;
	/* AF ROI configuration */
	struct af_statistics_dld_region af_region;
	/* Enable flag for tone mapping, set false would disable auwAFLUT */
	bool enable_af_lut;
	/* Tone mapping for common 3A, no longer used (default bypassing) */
	u16 lut[259];
	/* Tone mapping for AF only */
	u16 af_lut[259];
	/* Weight values when calculate the frequency. */
	u8 weight[6];
	/* Bit for shift */
	u16 sh;
	/* Configure 2 modes. Mode 1: First 82 blocks refer to Th,
	*  Tv from thelookup array. Mode 2: Every 4 blocks reference one element
	*/
	u8 th_mode;
	/* Locate index in an 82 elements array and
	*  reference one of 4 stages Th,
	*  Tv
	*/
	u8 index[82];
	/* Thresholds for calculate the horizontal contrast. */
	u16 th[4];
	/* Thresholds for calculate the vertical contrast. */
	u16 tv[4];
	/* Input data offset */
	u32 af_offset;
	/* Turn pseudo Y on or off */
	bool af_py_enable;
	/* Turn low pass filter on or off */
	bool af_lpf_enable;
	/* Median filter mode, suggest use MF_DISABLE */
	enum mid_mode filter_mode;
	/* Median filter device ID, for AF, use 1 */
	u8 filter_id;
	/* AF timing control anchor point */
	u16 line_cnt;
	struct af_hw3aa_lpf af_hw3aa_lpf;
	/* IIR0, IIR1 */
	struct af_hw3aa_iir af_hw3aa_iir[2];
	struct af_hw3aa_pseudoy af_hw3aa_pseudoy;
};
#pragma pack(pop)

/*
 *@typedef yhis_cfg_info
 *@brief YHIS configuration information
 */
#pragma pack(push)
#pragma pack(4)
struct yhis_cfg_info {
	u16 token_id;
	struct statistics_dld_region yhis_region;
};
#pragma pack(pop)

/*
 *@typedef antiflicker_cfg_info
 *@brief AntiFlicker configuration information
 */
#pragma pack(push)
#pragma pack(4)
struct antiflicker_cfg_info {
	u16 token_id;
	/* Used to control ROI shift position ratio of width,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_x;
	/* Used to control ROI shift position ratio of height,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_y;
};
#pragma pack(pop)

/*
 *@typedef subsample_cfg_info
 *@brief SubSample configuration information
 */
#pragma pack(push)
#pragma pack(4)
struct subsample_cfg_info {
	u16 token_id;
	/* Subsample engine buffer size in bytes */
	u32 buffer_image_size;
	/* Used to control ROI shift position ratio of width,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_x;
	/* Used to control ROI shift position ratio of height,
	*  if set 1 means offset 1% width from start horizontal position,
	*  suggest 0
	*/
	u16 offset_ratio_y;
};
#pragma pack(pop)

/*
 *@typedef dld_sequence
 *@brief Download sequence structure
 */
#pragma pack(push)
#pragma pack(4)
struct dld_sequence {
	u8 preview_baisc_dld_seq_length;
	u8 preview_adv_dld_seq_length;
	u8 fast_converge_baisc_dld_seq_length;
	u8 preview_baisc_dld_seq[MAXSEQLEN];
	u8 preview_adv_dld_seq[MAXSEQLEN];
	u8 fast_converge_baisc_dld_seq[MAXSEQLEN];
};
#pragma pack(pop)

/*
 *@typedef cfg_3a_info
 *@brief 3A configuration
 */
#pragma pack(push)
#pragma pack(4)
struct cfg_3a_info {
	/* For sync between ISP firmware and 3A's lib */
	u32 magic_num;
	struct ae_cfg_info ae_info;
	struct awb_cfg_info awb_info;
	struct af_cfg_info af_info;
	struct yhis_cfg_info yhis_info;
	struct antiflicker_cfg_info antiflicker_info;
	struct subsample_cfg_info subsample_info;
};
#pragma pack(pop)

#endif

#endif
