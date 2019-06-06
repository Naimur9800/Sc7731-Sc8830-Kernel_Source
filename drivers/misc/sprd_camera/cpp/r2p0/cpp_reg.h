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
#ifndef _CPP_REG_H_
#define _CPP_REG_H_

/*#define CPP_TEST_DRIVER*/
/*#define USE_WHALE2_REG_DEFINE*/

#define CPP_AHB_RESET                  (SPRD_MMAHB_BASE + 0x04UL)
#define CPP_AHB_RESET_BIT              (1 << 22)

#define CPP_BASE                       0x00
#define CPP_PATH_EB                    0x00
#define CPP_PATH3_EB_BIT               BIT(3)
#define CPP_PATH2_EB_BIT               BIT(2)
#define CPP_ROT_EB_BIT                 BIT(1)
#define CPP_SCALE_PATH_EB_BIT          BIT(0)

#define CPP_AUTOGATE_SET               0x04
#define CPP_PATH3_AUTOGATE_EN_BIT      BIT(3) /*path3*/
#define CPP_PATH2_AUTOGATE_EN_BIT      BIT(2) /*path2*/
#define CPP_PATH1_AUTOGATE_EN_BIT      BIT(1) /*rotate*/
#define CPP_PATH0_AUTOGATE_EN_BIT      BIT(0) /*scaling*/

#define CPP_PATH_START                 0x08
#define CPP_PATH3_START_BIT         BIT(3)
#define CPP_PATH2_START_BIT            BIT(2)
#define CPP_ROT_START_BIT              BIT(1)
#define CPP_SCALE_START_BIT            BIT(0)

#define CPP_PATH_STS                   0x0c
#define CPP_PATH3_STATUS_BIT_BUSY      BIT(3) /*path3, 1 busy, 0 idle*/
#define CPP_PATH2_STATUS_BIT_BUSY      BIT(2) /*path2, 1 busy, 0 idle*/
#define CPP_PATH1_STATUS_BIT_BUSY      BIT(1) /*rotate, 1 busy, 0 idle*/
#define CPP_PATH0_STATUS_BIT_BUSY      BIT(0) /*scaling, 1 busy, 0 idle*/

#define CPP_INT_STS                    0x10
#define CPP_PATH3_IRQ_STATUS_BIT       BIT(3) /*path3*/
#define CPP_PATH2_IRQ_STATUS_BIT       BIT(2) /*path2*/
#define CPP_PATH1_IRQ_STATUS_BIT       BIT(1) /*rotate*/
#define CPP_PATH0_IRQ_STATUS_BIT       BIT(0) /*scaling*/

#define CPP_INT_MASK                   0x14
#define CPP_PATH3_IRQ_MASK_BIT         BIT(3) /*path3*/
#define CPP_PATH2_IRQ_MASK_BIT         BIT(2) /*path2*/
#define CPP_PATH1_IRQ_MASK_BIT         BIT(1) /*rotate*/
#define CPP_PATH0_IRQ_MASK_BIT         BIT(0) /*scaling*/

#define CPP_INT_CLR                    0x18
#define CPP_PATH3_IRQ_CLEAR_BIT        BIT(3) /*path3*/
#define CPP_PATH2_IRQ_CLEAR_BIT        BIT(2) /*path2*/
#define CPP_PATH1_IRQ_CLEAR_BIT        BIT(1) /*rotate*/
#define CPP_PATH0_IRQ_CLEAR_BIT        BIT(0) /*scaling*/

#define CPP_INT_RAW                    0x1c
#define CPP_PATH3_IRQ_DONE_BIT         BIT(3) /*path3*/
#define CPP_PATH2_IRQ_DONE_BIT         BIT(2) /*path2*/
#define CPP_PATH1_IRQ_DONE_BIT         BIT(1) /*rotate*/
#define CPP_PATH0_IRQ_DONE_BIT         BIT(0) /*scaling*/

#define CPP_AXIM_CHN_SET               0x20
#define CPP_PATH3_OUTPUT_UV_ENDIAN     (1 << 31)
#define CPP_PATH3_OUTPUT_Y_ENDIAN      (7 << 28)
#define CPP_PATH3_INPUT_UV_ENDIAN      (1 << 27)
#define CPP_PATH3_INPUT_Y_ENDIAN       (7 << 24)
#define CPP_PATH2_OUTPUT_UV_ENDIAN     (1 << 23)
#define CPP_PATH2_OUTPUT_Y_ENDIAN      (7 << 20)
#define CPP_PATH2_INPUT_UV_ENDIAN      (1 << 19)
#define CPP_PATH2_INPUT_Y_ENDIAN       (7 << 16)
#define CPP_ROT_AXI_WR_ENDIAN_MASK     (7 << 8)
#define CPP_SCALE_OUTPUT_UV_ENDIAN     (1 << 7)
#define CPP_SCALE_OUTPUT_Y_ENDIAN      (7 << 4)
#define CPP_SCALE_INPUT_UV_ENDIAN      (1 << 3)
#define CPP_SCALE_INPUT_Y_ENDIAN       (7 << 0)

#define CPP_AXIM_BURST_GAP_PATH0       0x24
#define CPP_PATH0_AXIM_BURST_GAP_READ  (0xffff << 16)
#define CPP_PATH0_AXIM_BURST_GAP_WRITE (0xffff)

#define CPP_AXIM_BURST_GAP_PATH1       0x28
#define CPP_PATH1_AXIM_BURST_GAP_READ  (0xffff << 16)
#define CPP_PATH1_AXIM_BURST_GAP_WRITE (0xffff)

#define CPP_AXIM_BURST_GAP_PATH2       0x2c
#define CPP_PATH2_AXIM_BURST_GAP_READ  (0xffff << 16)
#define CPP_PATH2_AXIM_BURST_GAP_WRITE (0xffff)

#define CPP_AXIM_BURST_GAP_PATH3       0x30
#define CPP_PATH3_AXIM_BURST_GAP_READ  (0xffff << 16)
#define CPP_PATH3_AXIM_BURST_GAP_WRITE (0xffff)

#define CPP_AXIM_PAUSE                 0x34
#define CPP_AXIM_BURST_SPLIT_EN_READ   (1 << 17)
#define CPP_AXIM_BURST_SPLIT_EN_WRITE  (1 << 16)
#define CPP_AXIM_PAUSE_CH3             (1 << 3)
#define CPP_AXIM_PAUSE_CH2             (1 << 2)
#define CPP_AXIM_PAUSE_CH1             (1 << 1)
#define CPP_AXIM_PAUSE_CH0             (1 << 0)

#define CPP_AXIM_STS                   0x38
#define CPP_AXIM_CH_STATUS_READ        (1 << 2)
#define CPP_AXIM_CH_STATUS_WRITE       (1 << 1)

/*path0 configure*/
#ifdef USE_WHALE2_REG_DEFINE
#define CPP_PATH0_SRC_ADDR_Y           0x30
#define CPP_PATH0_SRC_ADDR_UV          0x34
#define CPP_PATH0_SRC_ADDR_V           0x38
#else
#define CPP_PATH0_SRC_ADDR_Y           0x3c /*8 bytes align*/
#define CPP_PATH0_SRC_ADDR_UV          0x40 /*8 bytes align*/
#define CPP_PATH0_SRC_ADDR_V           0x44 /*8 bytes align*/
#endif
#define CPP_PATH0_R_DES_ADDR_Y         0x48 /*8 bytes align*/
#define CPP_PATH0_R_DES_ADDR_UV        0x4c /*8 bytes align*/
#define CPP_PATH0_R_DES_ADDR_V         0x50 /*8 bytes align*/

#define CPP_PATH0_CFG0                 0x54
#define CPP_SCALE_CLK_SWITCH           (1 << 11)
#define CPP_SCALE_SRC_SPLIT_MODE_EB \
	(1 << 10) /*for JPEGLS mode only, not used in iWhale2*/
#define CPP_SCALE_DEC_V_MASK \
	(3 << 8) /*scaler_src_height =*/
			/*(path0_srcc_height >> CPP_SCALE_DEC_V_MASK)*/
#define CPP_SCALE_DEC_H_MASK \
	(3 << 6) /*scaler_src_width =*/
				/*(path0_srcc_width >> CPP_SCALE_DEC_H_MASK)*/
#define CPP_SCALE_OUTPUT_FORMAT \
	(3 << 4) /*2'b00:YUV420 2-plane; 2'b01:YUV420 3-plane;*/
			/*2'b10:YUV422 2-plane; 2'b11: reserved*/
#define CPP_SCALE_INPUT_FORMAT \
	(3 << 2) /*2'b00:YUV420 2-plane;*/
				/*2'b01:YUV420 3-plane;*/
				/*2'b10:YUV422 2-plane;*/
				/*2'b11: JPEG_LS(reserved in iWhale2)*/
#define CPP_SCALE_JPEG_CO_WORK_EB      (1 << 1) /*not used in iWhale2*/
#define CPP_SCALE_SR_MODE_EB           (1 << 0) /* 1:enable scaler; 2:bypass*/

#define CPP_PATH0_CFG1                 0x58
#define CPP_SCALE_SRC_HEIGHT_MASK      (0x1fff << 16)
#define CPP_SCALE_SRC_WIDTH_MASK       0x1fff

#define CPP_PATH0_CFG2                 0x5c
#define CPP_SCALE_DES_HEIGHT_MASK      (0x1fff << 16)
#define CPP_SCALE_DES_WIDTH_MASK       (0x1fff)

#define CPP_PATH0_CFG3                 0x60
#define CPP_SCALE_DES_PITCH_MASK       (0x1fff << 16)
#define CPP_SCALE_SRC_PITCH_MASK       0x1fff

#define CPP_PATH0_CFG4                 0x64
#define CPP_SCALE_SRC_OFFSET_X_MASK    (0x1fff << 16)
#define CPP_SCALE_SRC_OFFSET_Y_MASK    0x1fff

#define CPP_PATH0_CFG5                 0x68
#define CPP_SCALE_DES_OFFSET_X_MASK    (0x1fff << 16)
#define CPP_SCALE_DES_OFFSET_Y_MASK    0x1fff

#define CPP_PATH0_CFG6                 0x6c
#define CPP_PATH0_TRIM_OFFSET_X_MASK   (0x1fff << 16)
#define CPP_PATH0_TRIM_OFFSET_Y_MASK   0x1fff

#define CPP_PATH0_CFG7                     0x70
#define CPP_PATH0_TRIM_HEIGHT_X_MASK   (0x1fff << 16)
#define CPP_PATH0_TRIM_WIDTH_Y_MASK    0x1fff

/*epsilon configure*/
#define CPP_EPF_CFG0                   0x74
#define CPP_EPF_EPSILON_1_MASK \
	(0xffff << 16) /*epsilon for SR Y2 channel*/
#define CPP_EPF_EPSILON_0_MASK \
	(0xffff)       /*epsilon for SR Y1 channel*/
#define CPP_EPF_CFG1                   0x78
#define CPP_EPF_EPSILON_2_MASK \
	(0xffff << 16) /*epsilon for SR Y3 channel*/
#define CPP_EPF_EPSILON_UV_MASK \
	(0xffff)       /*epsilon for SR UV channel*/
#define CPP_EPF_CFG2                   0x7c
#define CPP_EPF_F2_Y_MASK \
	(0xffff << 16) /*f2 for SR Y1/Y2/Y3 channel*/
#define CPP_EPF_F2_UV_MASK             (0xffff)       /*f2 for SR UV channel*/
#define CPP_EPF_CFG3                   0x80
#define CPP_EPF_GAIN3_MASK \
	(0xFF << 24) /*gain for SR, gain_3=gain_2 + 128*/
#define CPP_EPF_GAIN2_MASK             (0xFF << 16) /*gain for SR, [0,127]*/
#define CPP_EPF_GAIN1_MASK             (0xFF << 8)  /*gain for SR, [0,32]*/
#define CPP_EPF_GAIN0_MASK \
	(0xFF << 0)  /*gain for SR, [-16,48], bit 7 is the sign flag*/

#define CPP_EPF_CFG4                   0x84
#define CPP_EPF_GAIN7_MASK \
	(0xFF << 24) /*gain for SR, gain_7=gain_6 + 128*/
#define CPP_EPF_GAIN6_MASK             (0xFF << 16) /*gain for SR, [0,127]*/
#define CPP_EPF_GAIN5_MASK             (0xFF << 8)  /*gain for SR, [0,32]*/
#define CPP_EPF_GAIN4_MASK \
	(0xFF << 0)  /*gain for SR, [-16,48], bit 7 is the sign flag*/

#define CPP_EPF_CFG5                   0x88
#define CPP_EPF_GAIN11_MASK \
	(0xFF << 24) /*gain for SR, gain_11=gain_10 + 128*/
#define CPP_EPF_GAIN10_MASK \
	(0xFF << 16) /*gain for SR, [0,127]*/
#define CPP_EPF_GAIN9_MASK \
	(0xFF << 8)  /*gain for SR, [0,32]*/
#define CPP_EPF_GAIN8_MASK \
	(0xFF << 0)  /*gain for SR, [-16,48], bit 7 is the sign flag*/

#define CPP_EPF_CFG6                   0x8c
#define CPP_EPF_MAX_DIFF_MASK          (0xFF) /*max_diff for SR, [0,255]*/

/*#define CPP_PATH0_JPEGLS_INFOR_Y       0x88*/
/*#define CPP_PATH0_JPEGLS_INFOR_U       0x8c*/
/*#define CPP_PATH0_JPEGLS_INFOR_V       0x90*/
/*#define CPP_PATH0_R_JPEGLS_INFOR_Y     0x94*/
/*#define CPP_PATH0_R_JPEGLS_INFOR_U     0x98*/
/*#define CPP_PATH0_R_JPEGLS_INFOR_V     0x9c*/

#define CPP_SC_TAP                     0x90
#define CPP_SCALE_Y_VER_TAP            (0xf << 5)
#define CPP_SCALE_UV_VER_TAP           (0x1f << 0)

#define CPP_SPARE0 \
	0xA0 /*cpp spare register0 for further usage*/
#define CPP_SPARE1 \
	0xA4 /*cpp spare register1 for further usage*/

#define CPP_PATH1_SRC_ADDR             0x100
#define CPP_PATH1_DES_ADDR             0x104
#define CPP_PATH1_ROT_SIZE             0x108
#define CPP_PATH1_ROT_HEIGHT_MASK      (0xffff << 16)
#define CPP_PATH1_ROT_WIDTH_MASK       (0xffff << 0)
#define CPP_PATH1_SRC_PITCH            0x10c
#define CPP_PATH1_SRC_PITCH_MASK       0xffff
#define CPP_PATH1_SRC_OFFSET           0x110
#define CPP_PATH1_SRC_OFFSET_Y_MASK    (0xffff << 16)
#define CPP_PATH1_SRC_OFFSET_X_MASK    0xffff
#define CPP_ROTATION_PATH_CFG          0x114
#define CPP_ROT_UV_MODE_BIT            (1 << 3) /* 1: uv422 mode*/
#define CPP_ROT_MODE_MASK \
	(3 << 1) /* 2'b00:90; 2'b01:270; 2'b10:180; 2'b11:horizontal mirror*/
#define CPP_ROT_PIXEL_FORMAT_BIT \
	(1 << 0) /* 0:byte per pixel mode; 1: half word per pixel mode*/

#define CPP_PATH0_SRC_YUV_REGULATE_0   0x118
#define CPP_PATH0_SRC_Y_UP_THRES \
	(0xff << 24) /*up threshold for source Y in cut mode*/
#define CPP_PATH0_SRC_Y_DOWN_THRES \
	(0xff << 16) /*down threshold for source Y in cut mode*/
#define CPP_PATH0_SRC_Y_EFFECT_THRES \
	(0xff << 8)  /*special effect threshold for source Y data*/
#define CPP_PATH0_SRC_REGULATE_MODE \
	(0x3 << 0) /* 2'b00: normal mode, out=in; 2'b01: shrink mode;*/
				/*2'b10: clip mode; 2'b11: special effect mode*/

#define CPP_PATH0_SRC_YUV_REGULATE_1   0x11c
#define CPP_PATH0_SRC_UV_UP_THRES \
	(0xff << 16) /*up threshold for source UV in cut mode*/
#define CPP_PATH0_SRC_UV_DOWN_THRES \
	(0xff << 8) /*down threshold for source UV in cut mode*/
#define CPP_PATH0_SRC_UV_EFFECT_THRES \
	(0xff << 0)  /*special effect threshold for source UV data*/

#define CPP_PATH0_SRC_YUV_REGULATE_2   0x120
#define CPP_PATH0_SRC_UV_SHRINK_OFFSET (0x1f << 20) /*UV shrink offset value*/
#define CPP_PATH0_SRC_UV_SHRINK_RANGE  (0xf << 16) /*UV shrink range value*/
#define CPP_PATH0_SRC_Y_SHRINK_OFFSET  (0x1f << 4) /*Y shrink offset value*/
#define CPP_PATH0_SRC_Y_SHRINK_RANGE   (0xf << 0) /*Y shrink range value*/

#define CPP_PATH0_DES_YUV_REGULATE_0   0x124
#define CPP_PATH0_DES_Y_UP_THRES \
	(0xff << 24) /*up threshold for output Y in cut mode*/
#define CPP_PATH0_DES_Y_DOWN_THRES \
	(0xff << 16) /*down threshold for output Y in cut mode*/
#define CPP_PATH0_DES_Y_EFFECT_THRES \
	(0xff << 8)  /*special effect threshold for output Y data*/
#define CPP_PATH0_DES_REGULATE_MODE \
	(0x3 << 0) /* 2'b00: normal mode, out=in; 2'b01: shrink mode;*/
				/*2'b10: clip mode; 2'b11: special effect mode*/

#define CPP_PATH0_DES_YUV_REGULATE_1   0x128
#define CPP_PATH0_DES_UV_UP_THRES \
	(0xff << 16) /*up threshold for output UV in cut mode*/
#define CPP_PATH0_DES_UV_DOWN_THRES \
	(0xff << 8) /*down threshold for output UV in cut mode*/
#define CPP_PATH0_DES_UV_EFFECT_THRES \
	(0xff << 0)  /*special effect threshold for output UV data*/

#define CPP_PATH0_DES_YUV_REGULATE_2   0x12c
#define CPP_PATH0_DES_UV_SHRINK_OFFSET (0x1f << 20) /*UV shrink offset value*/
#define CPP_PATH0_DES_UV_SHRINK_RANGE  (0xf << 16) /*UV shrink range value*/
#define CPP_PATH0_DES_Y_SHRINK_OFFSET  (0x1f << 4) /*Y shrink offset value*/
#define CPP_PATH0_DES_Y_SHRINK_RANGE   (0xf << 0) /*Y shrink range value*/

/*path2 configure*/
#define CPP_PATH2_SRC_Y_BASE_ADDR      0x130
#define CPP_PATH2_SRC_UV_BASE_ADDR     0x134
#define CPP_PATH2_SRC_V_BASE_ADDR      0x138
#define CPP_PATH2_DES_Y_BASE_ADDR      0x13c
#define CPP_PATH2_DES_UV_BASE_ADDR     0x140
#define CPP_PATH2_DES_V_BASE_ADDR      0x144

#define CPP_PATH2_CFG0                 0x148
#define CPP_PATH2_Y_VER_TAP \
	(0xf << 21)  /*path2 Y vertical scaling filter tap number*/
#define CPP_PATH2_UV_VER_TAP \
	(0x1f << 16) /*path2 UV vertical scaling filter tap number*/
#define CPP_PATH2_WORK_WITH_EPF \
	(1 << 9)     /* 0: path2 works without EPF function;*/
					/*1: path2 works with EPF function*/
#define CPP_PATH2_CLK_SWITCH \
	(1 << 8)     /* 0: select cpp_clk; 1: select apb_clk*/
#define CPP_PATH2_DEC_V_MASK \
	(3 << 6)     /*path2 vertical decimation set*/
#define CPP_PATH2_DEC_H_MASK \
	(3 << 4)     /*path2 horizontal decimation set*/
#define CPP_PATH2_OUTPUT_FORMAT \
	(3 << 2)     /* 0: YUV420 2-plane; 1: reserved;*/
					/*2: YUV422 2-plane; 3: reserved*/
#define CPP_PATH2_INPUT_FORMAT \
	(3 << 0)     /* 0: YUV420 2-plane; 1: reserved;*/
					/*2: YUV422 2-plane; 3: reserved*/

#define CPP_PATH2_CFG1                 0x14c
#define CPP_PATH2_SRC_HEIGHT_MASK      (0x1fff << 16)
#define CPP_PATH2_SRC_WIDTH_MASK       0x1fff

#define CPP_PATH2_CFG2                 0x150
#define CPP_PATH2_DES_HEIGHT_MASK      (0x1fff << 16)
#define CPP_PATH2_DES_WIDTH_MASK       (0x1fff)

#define CPP_PATH2_CFG3                 0x154
#define CPP_PATH2_DES_PITCH_MASK       (0x1fff << 16)
#define CPP_PATH2_SRC_PITCH_MASK       0x1fff

#define CPP_PATH2_CFG4                 0x158
#define CPP_PATH2_SRC_OFFSET_X_MASK    (0x1fff << 16)
#define CPP_PATH2_SRC_OFFSET_Y_MASK    0x1fff

#define CPP_PATH2_CFG5                 0x15c
#define CPP_PATH2_DES_OFFSET_X_MASK    (0x1fff << 16)
#define CPP_PATH2_DES_OFFSET_Y_MASK    0x1fff

#define CPP_PATH2_CFG6                 0x160
#define CPP_PATH2_TRIM_OFFSET_X_MASK   (0x1fff << 16)
#define CPP_PATH2_TRIM_OFFSET_Y_MASK   0x1fff

#define CPP_PATH2_CFG7                 0x164
#define CPP_PATH2_TRIM_HEIGHT_X_MASK   (0x1fff << 16)
#define CPP_PATH2_TRIM_WIDTH_Y_MASK    0x1fff

#define CPP_PATH2_SRC_YUV_REGULATE_0   0x168
#define CPP_PATH2_SRC_Y_UP_THRES \
	(0xff << 24) /*up threshold for source Y in cut mode*/
#define CPP_PATH2_SRC_Y_DOWN_THRES \
	(0xff << 16) /*down threshold for source Y in cut mode*/
#define CPP_PATH2_SRC_Y_EFFECT_THRES \
	(0xff << 8)  /*special effect threshold for source Y data*/
#define CPP_PATH2_SRC_REGULATE_MODE \
	(0x3 << 0)   /* 2'b00: normal mode, out=in; 2'b01: shrink mode;*/
			/*2'b10: clip mode; 2'b11: special effect mode*/

#define CPP_PATH2_SRC_YUV_REGULATE_1   0x16c
#define CPP_PATH2_SRC_UV_UP_THRES \
	(0xff << 16) /*up threshold for source UV in cut mode*/
#define CPP_PATH2_SRC_UV_DOWN_THRES \
	(0xff << 8) /*down threshold for source UV in cut mode*/
#define CPP_PATH2_SRC_UV_EFFECT_THRES \
	(0xff << 0)  /*special effect threshold for source UV data*/

#define CPP_PATH2_SRC_YUV_REGULATE_2   0x170
#define CPP_PATH2_SRC_UV_SHRINK_OFFSET \
	(0x1f << 20) /*UV shrink offset value*/
#define CPP_PATH2_SRC_UV_SHRINK_RANGE \
	(0xf << 16)  /*UV shrink range value*/
#define CPP_PATH2_SRC_Y_SHRINK_OFFSET \
	(0x1f << 4)  /*Y shrink offset value*/
#define CPP_PATH2_SRC_Y_SHRINK_RANGE \
	(0xf << 0)   /*Y shrink range value*/

#define CPP_PATH2_DES_YUV_REGULATE_0   0x174
#define CPP_PATH2_DES_Y_UP_THRES \
	(0xff << 24) /*up threshold for output Y in cut mode*/
#define CPP_PATH2_DES_Y_DOWN_THRES \
	(0xff << 16) /*down threshold for output Y in cut mode*/
#define CPP_PATH2_DES_Y_EFFECT_THRES \
	(0xff << 8)  /*special effect threshold for output Y data*/
#define CPP_PATH2_DES_REGULATE_MODE \
	(0x3 << 0)   /* 2'b00: normal mode, out=in; 2'b01: shrink mode;*/
		/*2'b10: clip mode; 2'b11: special effect mode*/

#define CPP_PATH2_DES_YUV_REGULATE_1   0x178
#define CPP_PATH2_DES_UV_UP_THRES \
	(0xff << 16) /*up threshold for output UV in cut mode*/
#define CPP_PATH2_DES_UV_DOWN_THRES \
	(0xff << 8)  /*down threshold for output UV in cut mode*/
#define CPP_PATH2_DES_UV_EFFECT_THRES \
	(0xff << 0)  /*special effect threshold for output UV data*/

#define CPP_PATH2_DES_YUV_REGULATE_2   0x17c
#define CPP_PATH2_DES_UV_SHRINK_OFFSET \
	(0x1f << 20) /*UV shrink offset value*/
#define CPP_PATH2_DES_UV_SHRINK_RANGE \
	(0xf << 16)  /*UV shrink range value*/
#define CPP_PATH2_DES_Y_SHRINK_OFFSET \
	(0x1f << 4)  /*Y shrink offset value*/
#define CPP_PATH2_DES_Y_SHRINK_RANGE \
	(0xf << 0)   /*Y shrink range value*/

/*path3 configure*/
#define CPP_PATH3_SRC_Y_BASE_ADDR      0x190
#define CPP_PATH3_SRC_UV_BASE_ADDR     0x194
#define CPP_PATH3_SRC_V_BASE_ADDR      0x198
#define CPP_PATH3_DES_Y_BASE_ADDR      0x19c
#define CPP_PATH3_DES_UV_BASE_ADDR     0x1A0
#define CPP_PATH3_DES_V_BASE_ADDR      0x1A4

#define CPP_PATH3_CFG0                 0x1A8
#define CPP_PATH3_Y_VER_TAP \
	(0xf << 21)  /*path3 Y vertical scaling filter tap number*/
#define CPP_PATH3_UV_VER_TAP \
	(0x1f << 16) /*path3 UV vertical scaling filter tap number*/
#define CPP_PATH3_WORK_WITH_PATH2 \
	(1 << 10) /* 2'b00: path3 works independently;*/
				/*2'b01: path3 shares path0's input;*/
				/*2'b10: path3 shares path2's input;*/
				/*2'b11: invalid*/
#define CPP_PATH3_WORK_WITH_PATH0 \
	(1 << 9)
#define CPP_PATH3_CLK_SWITCH \
	(1 << 8)     /* 0: select cpp_clk; 1: select apb_clk*/
#define CPP_PATH3_DEC_V_MASK \
	(3 << 6)     /*path3 vertical decimation set*/
#define CPP_PATH3_DEC_H_MASK \
	(3 << 4)     /*path3 horizontal decimation set*/
#define CPP_PATH3_OUTPUT_FORMAT \
	(3 << 2) /* 0: YUV420 2-plane; 1: reserved;*/
				/*2: YUV422 2-plane; 3: reserved*/
#define CPP_PATH3_INPUT_FORMAT \
	(3 << 0) /* 0: YUV420 2-plane; 1: reserved;*/
				/*2: YUV422 2-plane; 3: reserved*/

#define CPP_PATH3_CFG1                 0x1AC
#define CPP_PATH3_SRC_HEIGHT_MASK      (0x1fff << 16)
#define CPP_PATH3_SRC_WIDTH_MASK       0x1fff

#define CPP_PATH3_CFG2                 0x1B0
#define CPP_PATH3_DES_HEIGHT_MASK      (0x1fff << 16)
#define CPP_PATH3_DES_WIDTH_MASK       (0x1fff)

#define CPP_PATH3_CFG3                 0x1B4
#define CPP_PATH3_DES_PITCH_MASK       (0x1fff << 16)
#define CPP_PATH3_SRC_PITCH_MASK       0x1fff

#define CPP_PATH3_CFG4                 0x1B8
#define CPP_PATH3_SRC_OFFSET_X_MASK    (0x1fff << 16)
#define CPP_PATH3_SRC_OFFSET_Y_MASK    0x1fff

#define CPP_PATH3_CFG5                 0x1BC
#define CPP_PATH3_DES_OFFSET_X_MASK    (0x1fff << 16)
#define CPP_PATH3_DES_OFFSET_Y_MASK    0x1fff

#define CPP_PATH3_CFG6                 0x1C0
#define CPP_PATH3_TRIM_OFFSET_X_MASK   (0x1fff << 16)
#define CPP_PATH3_TRIM_OFFSET_Y_MASK   0x1fff

#define CPP_PATH3_CFG7                 0x1C4
#define CPP_PATH2_TRIM_HEIGHT_X_MASK   (0x1fff << 16)
#define CPP_PATH2_TRIM_WIDTH_Y_MASK    0x1fff

#define CPP_PATH3_SRC_YUV_REGULATE_0   0x1C8
#define CPP_PATH3_SRC_Y_UP_THRES \
	(0xff << 24) /*up threshold for source Y in cut mode*/
#define CPP_PATH3_SRC_Y_DOWN_THRES \
	(0xff << 16) /*down threshold for source Y in cut mode*/
#define CPP_PATH3_SRC_Y_EFFECT_THRES \
	(0xff << 8)  /*special effect threshold for source Y data*/
#define CPP_PATH3_SRC_REGULATE_MODE \
	(0x3 << 0)   /* 2'b00: normal mode, out=in; 2'b01: shrink mode;*/
				/*2'b10: clip mode; 2'b11: special effect mode*/

#define CPP_PATH3_SRC_YUV_REGULATE_1   0x1CC
#define CPP_PATH3_SRC_UV_UP_THRES \
	(0xff << 16) /*up threshold for source UV in cut mode*/
#define CPP_PATH3_SRC_UV_DOWN_THRES \
	(0xff << 8) /*down threshold for source UV in cut mode*/
#define CPP_PATH3_SRC_UV_EFFECT_THRES \
	(0xff << 0)  /*special effect threshold for source UV data*/

#define CPP_PATH3_SRC_YUV_REGULATE_2   0x1D0
#define CPP_PATH3_DES_UV_SHRINK_OFFSET \
	(0x1f << 20) /*UV shrink offset value*/
#define CPP_PATH3_DES_UV_SHRINK_RANGE \
	(0xf << 16)  /*UV shrink range value*/
#define CPP_PATH3_DES_Y_SHRINK_OFFSET \
	(0x1f << 4)  /*Y shrink offset value*/
#define CPP_PATH3_DES_Y_SHRINK_RANGE \
	(0xf << 0)   /*Y shrink range value*/

#define CPP_PATH3_DES_YUV_REGULATE_0   0x1D4
#define CPP_PATH3_DES_Y_UP_THRES \
	(0xff << 24) /*up threshold for output Y in cut mode*/
#define CPP_PATH3_DES_Y_DOWN_THRES \
	(0xff << 16) /*down threshold for output Y in cut mode*/
#define CPP_PATH3_DES_Y_EFFECT_THRES \
	(0xff << 8)  /*special effect threshold for output Y data*/
/* 2'b00: normal mode, out=in; 2'b01: shrink mode;*/
/*2'b10: clip mode; 2'b11: special effect mode*/
#define CPP_PATH3_DES_REGULATE_MODE \
	(0x3 << 0)

#define CPP_PATH3_DES_YUV_REGULATE_1   0x1D8
#define CPP_PATH3_DES_UV_UP_THRES \
	(0xff << 16) /*up threshold for output UV in cut mode*/
#define CPP_PATH3_DES_UV_DOWN_THRES \
	(0xff << 8)  /*down threshold for output UV in cut mode*/
#define CPP_PATH3_DES_UV_EFFECT_THRES \
	(0xff << 0)  /*special effect threshold for output UV data*/

#define CPP_PATH3_DES_YUV_REGULATE_2   0x1DC
#define CPP_PATH3_DES_UV_SHRINK_OFFSET (0x1f << 20) /*UV shrink offset value*/
#define CPP_PATH3_DES_UV_SHRINK_RANGE  (0xf << 16)  /*UV shrink range value*/
#define CPP_PATH3_DES_Y_SHRINK_OFFSET  (0x1f << 4)  /*Y shrink offset value*/
#define CPP_PATH3_DES_Y_SHRINK_RANGE   (0xf << 0)   /*Y shrink range value*/

#define CPP_END                        0x1DC

/*scaling coefficient table*/
/*path0*/
#define CPP_PATH0_SCALE_TABLE_HCOEF_LUMA    0x0400 /*range 0x0400~0x047C*/
#define CPP_PATH0_SCALE_TABLE_HCOEF_CHROMA  0x0480 /*range 0x0480~0x04BC*/
#define CPP_PATH0_SCALE_TABLE_VCOEF         0x04F0 /*range 0x04F0~0x06FC*/

/*path2*/
#define CPP_PATH2_SCALE_TABLE_HCOEF_LUMA    0x0800 /*range 0x0800~0x087C*/
#define CPP_PATH2_SCALE_TABLE_HCOEF_CHROMA  0x0880 /*range 0x0880~0x08BC*/
#define CPP_PATH2_SCALE_TABLE_VCOEF         0x08F0 /*range 0x08F0~0x0AFC*/

/*path3*/
#define CPP_PATH3_SCALE_TABLE_HCOEF_LUMA    0x0C00 /*range 0x0C00~0x0C7C*/
#define CPP_PATH3_SCALE_TABLE_HCOEF_CHROMA  0x0C80 /*range 0x0C80~0x0CBC*/
#define CPP_PATH3_SCALE_TABLE_VCOEF         0x0CF0 /*range 0x0CF0~0x0EFC*/

#endif
