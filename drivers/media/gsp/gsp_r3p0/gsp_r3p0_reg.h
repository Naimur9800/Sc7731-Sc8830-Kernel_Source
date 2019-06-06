/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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
#ifndef _GSP_R3P0_REG_H
#define _GSP_R3P0_REG_H

#include "gsp_debug.h"

struct R3P0_GSP_COEF_REG_T {
	uint32_t hor_coef_1_12;
	uint32_t hor_coef_1_34;
	uint32_t hor_coef_1_56;
	uint32_t hor_coef_1_78;
	uint32_t hor_coef_2_12;
	uint32_t hor_coef_2_34;
	uint32_t hor_coef_2_56;
	uint32_t hor_coef_2_78;
	uint32_t hor_coef_3_12;
	uint32_t hor_coef_3_34;
	uint32_t hor_coef_3_56;
	uint32_t hor_coef_3_78;
	uint32_t hor_coef_4_12;
	uint32_t hor_coef_4_34;
	uint32_t hor_coef_4_56;
	uint32_t hor_coef_4_78;
	uint32_t hor_coef_5_12;
	uint32_t hor_coef_5_34;
	uint32_t hor_coef_5_56;
	uint32_t hor_coef_5_78;
	uint32_t hor_coef_6_12;
	uint32_t hor_coef_6_34;
	uint32_t hor_coef_6_56;
	uint32_t hor_coef_6_78;
	uint32_t hor_coef_7_12;
	uint32_t hor_coef_7_34;
	uint32_t hor_coef_7_56;
	uint32_t hor_coef_7_78;
	uint32_t hor_coef_8_12;
	uint32_t hor_coef_8_34;
	uint32_t hor_coef_8_56;
	uint32_t hor_coef_8_78;
	uint32_t vor_coef_1_12;
	uint32_t vor_coef_1_34;
	uint32_t vor_coef_2_12;
	uint32_t vor_coef_2_34;
	uint32_t vor_coef_3_12;
	uint32_t vor_coef_3_34;
	uint32_t vor_coef_4_12;
	uint32_t vor_coef_4_34;
	uint32_t vor_coef_5_12;
	uint32_t vor_coef_5_34;
	uint32_t vor_coef_6_12;
	uint32_t vor_coef_6_34;
	uint32_t vor_coef_7_12;
	uint32_t vor_coef_7_34;
	uint32_t vor_coef_8_12;
	uint32_t vor_coef_8_34;
};

struct R3P0_GSP_GLB_CFG_REG {
	union {
		struct {
			uint32_t RUN_MOD		   :   1;
			uint32_t CMD_EN			:   1;
			uint32_t CMD_ENDIAN_MOD	:   4;
			uint32_t HTAP_MOD		  :   2;
			uint32_t VTAP_MOD		  :   1;
			uint32_t WCH_PAUSE		 :   1;
			uint32_t RCH_PAUSE		   :  1;
			uint32_t SCL_CLR				 : 1;
			uint32_t Reserved1		 :   4;
			uint32_t GAP_WB			:   8;
			uint32_t GAP_RB			:   8;
		};
		uint32_t	value;
	};
};

struct R3P0_GSP_MOD1_CFG_REG {
	union {
		struct {
			uint32_t BLD_RUN	   :   1;
			uint32_t BLD_BUSY	  :   1;
			uint32_t RCH_BUSY	  :   1;
			uint32_t WCH_BUSY	  :   1;
			uint32_t Reserved1	 :   4;
			uint32_t ERR1_FLG	  :   1;
			uint32_t ERR1_CODE	 :   5;
			uint32_t Reserved2	 :   2;
			uint32_t L0_EN		 :   1;
			uint32_t L1_EN		 :   1;
			uint32_t L2_EN		 :   1;
			uint32_t L3_EN		 :   1;
			uint32_t PMARGB_MOD	:   4;
			uint32_t SCALE_SEQ	 :   1;
			uint32_t DITHER1_EN	:   1;
			uint32_t SCALE_EN	  :   1;
			uint32_t PMARGB_EN	 :   1;
			uint32_t BK_EN		 :   1;
			uint32_t Reserved3	 :   3;
		};
		uint32_t	value;
	};
};

struct R3P0_GSP_MOD2_CFG_REG {
	union {
		struct {
			uint32_t SCL_RUN	   :   1;
			uint32_t SCL_BUSY	  :   1;
			uint32_t Reserved1	 :   6;
			uint32_t DITHER2_EN	:   1;
			uint32_t Reserved2	 :   15;
			uint32_t ERR2_FLG	  :   1;
			uint32_t ERR2_CODE	 :   4;
			uint32_t Reserved3	 :   3;
		};
		uint32_t	value;
	};
};



struct R3P0_GSP_CMD_STS0_REG {
	union {
		struct {
			uint32_t STAT_CMD_NUM  :   16;
			uint32_t STOPC		 :   1;
			uint32_t Reserved	  :   8;
			uint32_t NEW_C		 :   1;
			uint32_t NEW_L0		:   1;
			uint32_t NEW_L1		:   1;
			uint32_t NEW_L2		:   1;
			uint32_t NEW_L3		:   1;
			uint32_t NEW_D		 :   1;
			uint32_t NEW_M1		:   1;
		};
		uint32_t	value;
	};
};

struct R3P0_GSP_CMD_STS1_REG {
	union {
		struct {
			uint32_t COEF_CACHE_ADDR   :32;
		};
		uint32_t	value;
	};
};

struct R3P0_GSP_CMD_ADDR_REG {
	union {
		struct {
			uint32_t CMD_BASE_ADDR	 :32;
		};
		uint32_t	value;
	};
};

struct R3P0_DES_DATA_CFG1_REG {
	union {
		struct {
			uint32_t Y_ENDIAN_MOD1		  :3;
			uint32_t UV_ENDIAN_MOD1		 :3;
			uint32_t R5_RGB_SWAP_MOD1	   :3;
			uint32_t A_SWAP_MOD1			:1;
			uint32_t ROT_MOD1			   :3;
			uint32_t R2Y_MOD1			   :3;
			uint32_t DES_IMG_FORMAT1		:3;
			uint32_t COMPRESS_R8			:1;
			uint32_t CR8_SWAP_MOD1		  : 3;
			uint32_t Reserved2			  :9;
		};
		uint32_t	value;
	};
};

struct R3P0_DES_Y_ADDR1_REG {
	union {
		struct {
			uint32_t DES_Y_BASE_ADDR1	 :32;
		};
		uint32_t	value;
	};
};

struct R3P0_DES_U_ADDR1_REG {
	union {
		struct {
			uint32_t DES_U_BASE_ADDR1	 :32;
		};
		uint32_t	value;
	};
};

struct R3P0_DES_V_ADDR1_REG {
	union {
		struct {
			uint32_t DES_V_BASE_ADDR1	 :32;
		};
		uint32_t	value;
	};
};

struct R3P0_DES_PITCH1_REG {
	union {
		struct {
			uint32_t DES_PITCH1	 :13;
			uint32_t Reserved1	  :3;
			uint32_t DES_HEIGHT1	:13;
			uint32_t Reserved2	  :3;
		};
		uint32_t	value;
	};
};

struct R3P0_BACK_RGB_REG {
	union {
		struct {
			uint32_t   BACKGROUND_B				:8;
			uint32_t   BACKGROUND_G				:8;
			uint32_t   BACKGROUND_R				:8;
			uint32_t   BACKGROUND_A				:8;
		};
		uint32_t	   value;
	};
};

struct R3P0_DES_SCL_SIZE_REG {
	union {
		struct {
			uint32_t DES_SCL_W	  :13;
			uint32_t Reserved1	  :3;
			uint32_t DES_SCL_H	  :13;
			uint32_t Reserved2	  :3;
		};
		uint32_t	value;
	};
};

struct R3P0_WORK_DES_START_REG {
	union {
		struct {
			uint32_t WORK_DES_X	  :13;
			uint32_t Reserved1	   :3;
			uint32_t WORK_DES_Y	  :13;
			uint32_t Reserved2	   :3;
		};
		uint32_t	value;
	};
};


struct R3P0_WORK_SCR_SIZE_REG {
	union {
		struct {
			uint32_t WORK_SCR_W	  :13;
			uint32_t Reserved1	   :3;
			uint32_t WORK_SCR_H	  :13;
			uint32_t reserved2	   :3;
		};
		uint32_t	value;
	};
};

struct R3P0_WORK_SCR_START_REG {
	union {
		struct {
			uint32_t WORK_SCR_X	  :13;
			uint32_t Reserved1	   :3;
			uint32_t WORK_SCR_Y	  :13;
			uint32_t Reserved2	   :3;
		};
		uint32_t	value;
	};
};

struct R3P0_DES_DATA_CFG2_REG {
	union {
		struct {
			uint32_t Y_ENDIAN_MOD2		  :3;
			uint32_t UV_ENDIAN_MOD2		 :3;
			uint32_t R5_RGB_SWAP_MOD2	   :3;
			uint32_t A_SWAP_MOD2			:1;
			uint32_t ROT_MOD2			   :3;
			uint32_t R2Y_MOD2			   :3;
			uint32_t DES_IMG_FORMAT2		:3;
			uint32_t COMPRESS2_R8		   :1;
			uint32_t CR8_SWAP_MOD2		: 3;
			uint32_t Reserved2			  :9;
		};
		uint32_t	value;
	};
};

struct R3P0_DES_Y_ADDR2_REG {
	union {
		struct {
			uint32_t DES_Y_BASE_ADDR2	 :32;
		};
		uint32_t	value;
	};
};

struct R3P0_DES_U_ADDR2_REG {
	union {
		struct {
			uint32_t DES_U_BASE_ADDR2	 :32;
		};
		uint32_t	value;
	};
};

struct R3P0_DES_V_ADDR2_REG {
	union {
		struct {
			uint32_t DES_V_BASE_ADDR2	 :32;
		};
		uint32_t	value;
	};
};

struct R3P0_DES_PITCH2_REG {
	union {
		struct {
			uint32_t DES_PITCH2	 :13;
			uint32_t Reserved1	  :3;
			uint32_t DES_HEIGHT2	:13;
			uint32_t Reserved2	  :3;
		};
		uint32_t	value;
	};
};

struct R3P0_AXI_DEBUG_REG {
	union {
		struct {
			uint32_t WCH_BUSY	   : 1;
			uint32_t RCH_BUSY		   : 1;
			uint32_t WBUS_BUSY		: 1;
			uint32_t RBUS_BUSY		   : 1;
			uint32_t Reserved1		   : 28;
		};
		uint32_t	   value;
	};
};

struct R3P0_GSP_INT_REG {
	union {
		struct {
			uint32_t INT_BLD_RAW	   :1;
			uint32_t INT_SCL_RAW	   :1;
			uint32_t INT_ROT_RAW	   :1;
			uint32_t INT_BERR_RAW	  :1;
			uint32_t INT_SERR_RAW	  :1;
			uint32_t Reserved1		 :3;
			uint32_t INT_BLD_EN		:1;
			uint32_t INT_SCL_EN		:1;
			uint32_t INT_ROT_EN		:1;
			uint32_t INT_BERR_EN	   :1;
			uint32_t INT_SERR_EN	   :1;
			uint32_t Reserved2		 :3;
			uint32_t INT_BLD_CLR	   :1;
			uint32_t INT_SCL_CLR	   :1;
			uint32_t INT_ROT_CLR	   :1;
			uint32_t INT_BERR_CLR	  :1;
			uint32_t INT_SERR_CLR	  :1;
			uint32_t Reserved3		 :3;
			uint32_t INT_BLD_STS	   :1;
			uint32_t INT_SCL_STS	   :1;
			uint32_t INT_ROT_STS	   :1;
			uint32_t INT_BERR_STS	  :1;
			uint32_t INT_SERR_STS	  :1;
			uint32_t Reserved4		 :3;
		};
		uint32_t	value;
	};
};

/* LAYER0 */
struct R3P0_LAYER0_CFG_REG {
	union {
		struct {
			uint32_t   Y_ENDIAN_MOD0		  :3;
			uint32_t   UV_ENDIAN_MOD0		 :3;
			uint32_t   RGB_SWAP_MOD0		  :3;
			uint32_t   A_SWAP_MOD0			:1;
			uint32_t   Reserved1			  :6;
			uint32_t   IMG_FORMAT0			:3;
			uint32_t   CK_EN0				:1;
			uint32_t   PALLET_EN0			 :1;
			uint32_t   LAYER_NUM			  :2;
			uint32_t   Y2R_MOD0			   :4;
			uint32_t   Reserved2		  :5;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER0_Y_ADDR_REG {
	union {
		struct {
			uint32_t   Y_BASE_ADDR0			  :32;
		};
		uint32_t	   value;
	};
};


struct R3P0_LAYER0_U_ADDR_REG {
	union {
		struct {
			uint32_t   U_BASE_ADDR0			  :32;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER0_V_ADDR_REG {
	union {
		struct {
			uint32_t   V_BASE_ADDR0			  :32;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER0_PITCH_REG {
	union {
		struct {
			uint32_t   PITCH0				   :13;
			uint32_t   Reserved1				:19;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER0_CLIP_START_REG {
	union {
		struct {
			uint32_t   CLIP_START_X0			:13;
			uint32_t   Reserved1				:3;
			uint32_t   CLIP_START_Y0			:13;
			uint32_t   Reserved2				:3;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER0_CLIP_SIZE_REG {
	union {
		struct {
			uint32_t   CLIP_SIZE_X0				:13;
			uint32_t   Reserved1				:3;
			uint32_t   CLIP_SIZE_Y0				:13;
			uint32_t   Reserved2				:3;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER0_DES_START_REG {
	union {
		struct {
			uint32_t   DES_START_X0				:13;
			uint32_t   Reserved1				   :3;
			uint32_t   DES_START_Y0				:13;
			uint32_t   Reserved2				   :3;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER0_PALLET_RGB_REG {
	union {
		struct {
			uint32_t   PALLET_B0			:8;
			uint32_t   PALLET_G0			:8;
			uint32_t   PALLET_R0			:8;
			uint32_t   PALLET_A0			:8;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER0_CK_REG {
	union {
		struct {
			uint32_t   CK_B0				:8;
			uint32_t   CK_G0				:8;
			uint32_t   CK_R0				:8;
			uint32_t   BLOCK_ALPHA0		 :8;
		};
		uint32_t	   value;
	};
};

struct R3P0_Y2R_Y_PARAM0_REG {
	union {
		struct {
			uint32_t   Y_CONTRAST0			  :10;
			uint32_t   Reserved1				:6;
			uint32_t   Y_BRIGHTNESS0			:9;
			uint32_t   Reserved2				:7;
		};
		uint32_t	   value;
	};
};

struct R3P0_Y2R_U_PARAM0_REG {
	union {
		struct {
			uint32_t   U_SATURATION0			:10;
			uint32_t   Reserved1				:6;
			uint32_t   U_OFFSET0				:8;
			uint32_t   Reserved2				:8;
		};
		uint32_t	   value;
	};
};

struct R3P0_Y2R_V_PARAM0_REG {
	union {
		struct {
			uint32_t   V_SATURATION0			:10;
			uint32_t   Reserved1				:6;
			uint32_t   V_OFFSET0				:8;
			uint32_t   Reserved2				:8;
		};
		uint32_t	   value;
	};
};

/* LAYER1 */
struct R3P0_LAYER1_CFG_REG {
	union {
		struct {
			uint32_t   ENDIAN1				:4;
			uint32_t   RGB_SWAP1			  :3;
			uint32_t   A_RGB_SWAP1			:1;
			uint32_t   Reserved1			  :8;
			uint32_t   IMG_FORMAT1			:2;
			uint32_t   CK_EN1				 :1;
			uint32_t   PALLET_EN1			 :1;
			uint32_t   Reserved2			  :12;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER1_R_ADDR_REG {
	union {
		struct {
			uint32_t   R_BASE_ADDR1			  :32;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER1_PITCH_REG {
	union {
		struct {
			uint32_t   PITCH1				   :13;
			uint32_t   Reserved1				:19;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER1_CLIP_START_REG {
	union {
		struct {
			uint32_t   CLIP_START_X1		   :13;
			uint32_t   Reserved1			   :3;
			uint32_t   CLIP_START_Y1		   :13;
			uint32_t   Reserved2			   :3;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER1_CLIP_SIZE_REG {
	union {
		struct {
			uint32_t   CLIP_SIZE_X1			:13;
			uint32_t   Reserved1			   :3;
			uint32_t   CLIP_SIZE_Y1			:13;
			uint32_t   Reserved2			   :3;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER1_DES_START_REG {
	union {
		struct {
			uint32_t   DES_START_X1			:13;
			uint32_t   Reserved1			   :3;
			uint32_t   DES_START_Y1			:13;
			uint32_t   Reserved2			   :3;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER1_PALLET_RGB_REG {
	union {
		struct {
			uint32_t   PALLET_B1			:8;
			uint32_t   PALLET_G1			:8;
			uint32_t   PALLET_R1			:8;
			uint32_t   PALLET_A1			:8;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER1_CK_REG {
	union {
		struct {
			uint32_t   CK_B1				:8;
			uint32_t   CK_G1				:8;
			uint32_t   CK_R1				:8;
			uint32_t   BLOCK_ALPHA1		 :8;
		};
		uint32_t	   value;
	};
};

/* Layer2 */
struct R3P0_LAYER2_CFG_REG {
	union {
		struct {
			uint32_t   ENDIAN2				:4;
			uint32_t   RGB_SWAP2			  :3;
			uint32_t   A_RGB_SWAP2			:1;
			uint32_t   Reserved1			  :8;
			uint32_t   IMG_FORMAT2			:2;
			uint32_t   CK_EN2				 :1;
			uint32_t   PALLET_EN2			 :1;
			uint32_t   Reserved2			  :12;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER2_R_ADDR_REG {
	union {
		struct {
			uint32_t   R_BASE_ADDR2			  :32;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER2_PITCH_REG {
	union {
		struct {
			uint32_t   PITCH2				   :13;
			uint32_t   Reserved1				:19;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER2_CLIP_START_REG {
	union {
		struct {
			uint32_t   CLIP_START_X2		   :13;
			uint32_t   Reserved1			   :3;
			uint32_t   CLIP_START_Y2		   :13;
			uint32_t   Reserved2			   :3;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER2_CLIP_SIZE_REG {
	union {
		struct {
			uint32_t   CLIP_SIZE_X2				:13;
			uint32_t   Reserved1				   :3;
			uint32_t   CLIP_SIZE_Y2				:13;
			uint32_t   Reserved2				   :3;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER2_DES_START_REG {
	union {
		struct {
			uint32_t   DES_START_X2				:13;
			uint32_t   Reserved1				   :3;
			uint32_t   DES_START_Y2				:13;
			uint32_t   Reserved2				   :3;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER2_PALLET_RGB_REG {
	union {
		struct {
			uint32_t   PALLET_B2				:8;
			uint32_t   PALLET_G2				:8;
			uint32_t   PALLET_R2				:8;
			uint32_t   PALLET_A2				:8;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER2_CK_REG {
	union {
		struct {
			uint32_t   CK_B2				:8;
			uint32_t   CK_G2				:8;
			uint32_t   CK_R2				:8;
			uint32_t   BLOCK_ALPHA2		 :8;
		};
		uint32_t	   value;
	};
};


/* Layer3 */
struct R3P0_LAYER3_CFG_REG {
	union {
		struct {
			uint32_t   ENDIAN3				:4;
			uint32_t   RGB_SWAP3			  :3;
			uint32_t   A_RGB_SWAP3			:1;
			uint32_t   Reserved1			  :8;
			uint32_t   IMG_FORMAT3			:2;
			uint32_t   CK_EN3				 :1;
			uint32_t   PALLET_EN3			 :1;
			uint32_t   Reserved2			  :12;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER3_R_ADDR_REG {
	union {
		struct {
			uint32_t   R_BASE_ADDR3			  :32;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER3_PITCH_REG {
	union {
		struct {
			uint32_t   PITCH3				   :13;
			uint32_t   Reserved1				:19;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER3_CLIP_START_REG {
	union {
		struct {
			uint32_t   CLIP_START_X3			   :13;
			uint32_t   Reserved1				   :3;
			uint32_t   CLIP_START_Y3			   :13;
			uint32_t   Reserved2				   :3;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER3_CLIP_SIZE_REG {
	union {
		struct {
			uint32_t   CLIP_SIZE_X3				:13;
			uint32_t   Reserved1				   :3;
			uint32_t   CLIP_SIZE_Y3				:13;
			uint32_t   Reserved2				   :3;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER3_DES_START_REG {
	union {
		struct {
			uint32_t   DES_START_X3				:13;
			uint32_t   Reserved1				   :3;
			uint32_t   DES_START_Y3				:13;
			uint32_t   Reserved2				   :3;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER3_PALLET_RGB_REG {
	union {
		struct {
			uint32_t   PALLET_B3				:8;
			uint32_t   PALLET_G3				:8;
			uint32_t   PALLET_R3				:8;
			uint32_t   PALLET_A3				:8;
		};
		uint32_t	   value;
	};
};

struct R3P0_LAYER3_CK_REG {
	union {
		struct {
			uint32_t   CK_B3				:8;
			uint32_t   CK_G3				:8;
			uint32_t   CK_R3				:8;
			uint32_t   BLOCK_ALPHA3		 :8;
		};
		uint32_t	   value;
	};
};

struct R3P0_GSP_MAIN_FSM_REG {
	union {
		struct {
			uint32_t   STATE_BLD				:4;
			uint32_t   STATE_SCL				:2;
			uint32_t   STATE_IGS				:2;
			uint32_t   STATE_OGB			 :2;
			uint32_t   STATE_OGS			 :2;
			uint32_t   Reserved1			 :20;
		};
		uint32_t	   value;
	};
};

struct R3P0_GSP_IOUT_FSM_REG {
	union {
		struct {
			uint32_t   STATE_IMG			:2;
			uint32_t   STATE_OSD1			:2;
			uint32_t   STATE_OSD2			:2;
			uint32_t   STATE_OSD3			:2;
			uint32_t   STATE_CMD			:3;
			uint32_t   Reserved1			:5;
			uint32_t   STATE_WCH1			:2;
			uint32_t   STATE_WCH2		 :2;
			uint32_t   Reserved2			:12;
		};
		uint32_t	   value;
	};
};


struct R3P0_GSP_CTL_REG_T {
	struct R3P0_GSP_GLB_CFG_REG glb_cfg;
	struct R3P0_GSP_MOD1_CFG_REG mod1_cfg;
	struct R3P0_GSP_MOD2_CFG_REG mod2_cfg;
	struct R3P0_GSP_CMD_STS0_REG cmd_sts0;
	struct R3P0_GSP_CMD_STS1_REG cmd_sts1;
	struct R3P0_GSP_CMD_ADDR_REG cmd_addr;

	struct R3P0_DES_DATA_CFG1_REG des1_data_cfg;
	struct R3P0_DES_Y_ADDR1_REG des1_y_addr;
	struct R3P0_DES_U_ADDR1_REG  des1_u_addr;
	struct R3P0_DES_V_ADDR1_REG  des1_v_addr;
	struct R3P0_DES_PITCH1_REG  des1_pitch;
	struct R3P0_BACK_RGB_REG  bg_rgb;
	struct R3P0_DES_SCL_SIZE_REG  des_scl_size;
	struct R3P0_WORK_DES_START_REG  work_des_start;
	struct R3P0_WORK_SCR_SIZE_REG  work_src_size;
	struct R3P0_WORK_SCR_START_REG  work_src_start;

	struct R3P0_DES_DATA_CFG2_REG  des2_data_cfg;
	struct R3P0_DES_Y_ADDR2_REG  des2_y_addr;
	struct R3P0_DES_U_ADDR2_REG  des2_u_addr;
	struct R3P0_DES_V_ADDR2_REG  des2_v_addr;
	struct R3P0_DES_PITCH2_REG  des2_pitch;
	struct R3P0_AXI_DEBUG_REG  axi_debug;
	struct R3P0_GSP_INT_REG  int_ctl;

	struct R3P0_LAYER0_CFG_REG  l0_cfg;
	struct R3P0_LAYER0_Y_ADDR_REG  l0_y_addr;
	struct R3P0_LAYER0_U_ADDR_REG  l0_u_addr;
	struct R3P0_LAYER0_V_ADDR_REG  l0_v_addr;
	struct R3P0_LAYER0_PITCH_REG  l0_pitch;
	struct R3P0_LAYER0_CLIP_START_REG  l0_clip_start;
	struct R3P0_LAYER0_CLIP_SIZE_REG  l0_clip_size;
	struct R3P0_LAYER0_DES_START_REG  l0_des_start;
	struct R3P0_LAYER0_PALLET_RGB_REG  l0_pallet_rgb;
	struct R3P0_LAYER0_CK_REG  l0_ck;
	struct R3P0_Y2R_Y_PARAM0_REG  y2r_y_param;
	struct R3P0_Y2R_U_PARAM0_REG  y2r_u_param;
	struct R3P0_Y2R_V_PARAM0_REG  y2r_v_param;

	struct R3P0_LAYER1_CFG_REG  l1_cfg;
	struct R3P0_LAYER1_R_ADDR_REG  l1_r_addr;
	struct R3P0_LAYER1_PITCH_REG  l1_pitch;
	struct R3P0_LAYER1_CLIP_START_REG  l1_clip_start;
	struct R3P0_LAYER1_CLIP_SIZE_REG  l1_clip_size;
	struct R3P0_LAYER1_DES_START_REG  l1_des_start;
	struct R3P0_LAYER1_PALLET_RGB_REG  l1_pallet_rgb;
	struct R3P0_LAYER1_CK_REG  l1_ck;

	struct R3P0_LAYER2_CFG_REG  l2_cfg;
	struct R3P0_LAYER2_R_ADDR_REG  l2_r_addr;
	struct R3P0_LAYER2_PITCH_REG  l2_pitch;
	struct R3P0_LAYER2_CLIP_START_REG  l2_clip_start;
	struct R3P0_LAYER2_CLIP_SIZE_REG  l2_clip_size;
	struct R3P0_LAYER2_DES_START_REG  l2_des_start;
	struct R3P0_LAYER2_PALLET_RGB_REG  l2_pallet_rgb;
	struct R3P0_LAYER2_CK_REG  l2_ck;

	struct R3P0_LAYER3_CFG_REG  l3_cfg;
	struct R3P0_LAYER3_R_ADDR_REG  l3_r_addr;
	struct R3P0_LAYER3_PITCH_REG  l3_pitch;
	struct R3P0_LAYER3_CLIP_START_REG  l3_clip_start;
	struct R3P0_LAYER3_CLIP_SIZE_REG  l3_clip_size;
	struct R3P0_LAYER3_DES_START_REG  l3_des_start;
	struct R3P0_LAYER3_PALLET_RGB_REG  l3_pallet_rgb;
	struct R3P0_LAYER3_CK_REG  l3_ck;

	uint32_t reserved[4];
	struct R3P0_GSP_COEF_REG_T coef_tab;
};

#endif
