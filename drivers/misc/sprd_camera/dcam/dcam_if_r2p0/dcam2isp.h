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
#ifndef _DCAM2ISP_H_
#define _DCAM2ISP_H_

#define DCAM2ISP_INT_STATUS            0x00
#define DCAM2ISP_INT_EN0               0x14
#define DCAM2ISP_INT_CLR0              0x18
#define DCAM2ISP_INT_RAW0              0x1c
#define DCAM2ISP_INT0                  0x20

#define DCAM2ISP_MERGE_STATUS0         0x100
#define DCAM2ISP_MERGE_STATUS1         0x104
#define DCAM2ISP_MERGE_STATUS2         0x108
#define DCAM2ISP_MERGE_STATUS3         0x10c
#define DCAM2ISP_MERGE_STATUS4         0x110
#define DCAM2ISP_PARAM                 0x114
#define DCAM2ISP_MERGE_SLICE_SIZE      0x118
#define DCAM2ISP_MERGE_WIDTH           0x11c
#define DCAM2ISP_PMU_RAM_MASK          0x120
#define DCAM2ISP_RESERVED              0x130

#define ISP_AXI0_WR_MASTER_STATUS      0x200
#define ISP_AXI0_RD_MASTER_STATUS      0x204
#define ISP_AXI0_ITI2AXIM_CTRL         0x214
#define ISP_AXI0_CONVERT_WR_CTRL       0x218

#define ISP_AXI1_WR_MASTER_STATUS      0x300
#define ISP_AXI1_RD_MASTER_STATUS      0x304
#define ISP_AXI1_ITI2AXIM_CTRL         0x314
#define ISP_AXI1_CONVERT_WR_CTRL       0x318

#define ISP_FETCH0_MEM_STATUS0         0x400
#define ISP_FETCH0_MEM_STATUS1         0x404
#define ISP_FETCH0_MIPI_STATUS         0x408
#define ISP_FETCH0_PARAM               0x414
#define ISP_FETCH0_START               0x418
#define ISP_FETCH0_MEM_SLICE_SIZE      0x420
#define ISP_FETCH0_SLICE_Y_ADDR        0x424
#define ISP_FETCH0_Y_PITCH             0x428
#define ISP_FETCH0_SLICE_U_ADDR        0x42c
#define ISP_FETCH0_U_PITCH             0x430
#define ISP_FETCH0_SLICE_V_ADDR        0x434
#define ISP_FETCH0_V_PITCH             0x438
#define ISP_FETCH0_MIPI_WORD_INFO      0x43c
#define ISP_FETCH0_MIPI_BYTE_INFO      0x440
#define ISP_FETCH0_LINE_DLY_CTRL       0x444
#define ISP_FETCH0_DATA_ENDIAN         0x448

#define ISP_FETCH1_MEM_STATUS0         0x500
#define ISP_FETCH1_MEM_STATUS1         0x504
#define ISP_FETCH1_MIPI_STATUS         0x508
#define ISP_FETCH1_PARAM               0x514
#define ISP_FETCH1_START               0x518
#define ISP_FETCH1_MEM_SLICE_SIZE      0x520
#define ISP_FETCH1_SLICE_Y_ADDR        0x524
#define ISP_FETCH1_Y_PITCH             0x528
#define ISP_FETCH1_SLICE_U_ADDR        0x52c
#define ISP_FETCH1_U_PITCH             0x530
#define ISP_FETCH1_SLICE_V_ADDR        0x534
#define ISP_FETCH1_V_PITCH             0x538
#define ISP_FETCH1_MIPI_WORD_INFO      0x53c
#define ISP_FETCH1_MIPI_BYTE_INFO      0x540
#define ISP_FETCH1_LINE_DLY_CTRL       0x544
#define ISP_FETCH1_DATA_ENDIAN         0x548

#define DCAM2ISP_ISP0_STATUS           0x600
#define DCAM2ISP_ISP0_SLICE_SIZE       0x614
#define DCAM2ISP_ISP0_HBLANK_NUM       0x618

#define DCAM2ISP_ISP1_STATUS           0x700
#define DCAM2ISP_ISP1_SLICE_SIZE       0x714
#define DCAM2ISP_ISP1_HBLANK_NUM       0x718

#define DCAM2ISP_ISP2_STATUS           0x800
#define DCAM2ISP_ISP2_SLICE_SIZE       0x814
#define DCAM2ISP_ISP2_HBLANK_NUM       0x818

/* every dcam2isp fetch module can get data from sensor or memory */
enum fetch_src_e {
	FETCH_SENSOR = 0,
	FETCH_MEM,
};

/* every isp can select fetch0 or fetch1 */
enum fetch_type_e {
	FETCH0 = 0,
	FETCH1,
	MERGE_MODE,
	CLOSE_MODE,
};

enum fetch_select_e {
	FETCH0_SEL = 0,
	FETCH1_SEL,
	FETCH_SEL_MAX
};

enum isp_select_e {
	ISP0_SEL = 0,
	ISP1_SEL,
	ISP2_SEL,
	ISP_SEL_MAX
};

enum fetch_fmt {
	FETCH_YUV422_THREEPLANE = 0,
	FETCH_YUYV422_ONEPLANE,
	FETCH_UYVY422_ONEPLANE,
	FETCH_YVYU422_ONEPLANE,
	FETCH_VYUY422_ONEPLANE,
	FETCH_YUV422_TWOPLANE,
	FETCH_YVU422_TWOPLANE,
	FETCH_RAW10,
	FETCH_CSI2_RAW10,
	FETCH_FULL_RGB10,/*only for channel 0*/
	FETCH_FTM_MAX
};

struct sprd_dcam2isp_size {
	unsigned int width;
	unsigned int height;
};

struct dcam2isp_fetch_parm {
	unsigned char mode;
	unsigned char sub_stract;
	unsigned char color_fmt;
	unsigned int y_address;
	unsigned int y_pitch;
	unsigned int u_address;
	unsigned int u_pitch;
	unsigned int v_address;
	unsigned int v_pitch;
	unsigned int data_endian;
	struct sprd_dcam2isp_size fetch_size;
};

struct sprd_dcam2isp_ctrl_parm {
	unsigned char isp0_eb;
	unsigned char isp0_src;
	unsigned char isp1_eb;
	unsigned char isp1_src;
	unsigned char isp2_eb;
	unsigned char isp2_src;
	unsigned char fetch0_eb;
	unsigned char fetch1_eb;
	unsigned char merge_bypass;
	unsigned char merge_mode;
	unsigned char ddrmode;
};

struct dcam2isp_cfg {
	struct sprd_dcam2isp_ctrl_parm ctrl_parm;
	struct sprd_dcam2isp_size isp_slice_size[ISP_SEL_MAX];
	struct dcam2isp_fetch_parm fetch_parm[FETCH_SEL_MAX];
};

void sprd_dcam2isp_enable(void);
void sprd_dcam2isp_disable(void);
int sprd_dcam2isp_param_cfg(struct dcam2isp_cfg *cfg);
int sprd_dcam2isp_start(void);
void sprd_dcam2isp_dump_reg(void);

#endif

