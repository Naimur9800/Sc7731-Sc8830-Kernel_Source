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
#ifndef _GSP_R5P0_REG_H
#define _GSP_R5P0_REG_H

#include "gsp_debug.h"

#define R5P0_IMGL_NUM 2
#define R5P0_OSDL_NUM 4
#define R5P0_IMGSEC_NUM 0
#define R5P0_OSDSEC_NUM 1

/* Global config reg */
#define R5P0_GSP_GLB_CFG(base)			 (base)
#define R5P0_GSP_INT(base)			(base + 0x004)
#define R5P0_GSP_CMD_STS(base)			(base + 0x008)
#define R5P0_GSP_CMD_COEF1_ADDR(base)			(base + 0x00C)
#define R5P0_GSP_CMD_COEF2_ADDR(base)			(base + 0x010)
#define R5P0_GSP_CMD_ADDR(base)			(base + 0x014)
#define R5P0_GSP_MOD_CFG(base)			(base + 0x018)
#define R5P0_GSP_SECURE_CFG(base)			(base + 0x01C)

/*Destination reg 1*/
#define R5P0_DES_DATA_CFG(base)		   (base + 0x020)
#define R5P0_DES_Y_ADDR(base)			 (base + 0x024)
#define R5P0_DES_U_ADDR(base)			 (base + 0x028)
#define R5P0_DES_V_ADDR(base)			 (base + 0x02C)
#define R5P0_DES_PITCH(base)			  (base + 0x030)
#define R5P0_BACK_RGB(base)				(base + 0x034)
#define R5P0_DES_SCL_SIZE1(base)			(base + 0x038)
#define R5P0_DES_SCL_SIZE2(base)			(base + 0x03C)
#define R5P0_WORK_AREA_SIZE1(base)		  (base + 0x040)
#define R5P0_WORK_AREA_XY1(base)		   (base + 0x044)
#define R5P0_WORK_AREA_SIZE2(base)		  (base + 0x048)
#define R5P0_WORK_AREA_SXY2(base)		   (base + 0x04C)
#define R5P0_WORK_AREA_DXY2(base)		  (base + 0x050)

#define R5P0_DES_SCL_SIZE_OFFSET 0x004

/*LAYERIMG*/
#define R5P0_LIMG_CFG(base)			  (base + 0x060)
#define R5P0_LIMG_Y_ADDR(base)		   (base + 0x064)
#define R5P0_LIMG_U_ADDR(base)		   (base + 0x068)
#define R5P0_LIMG_V_ADDR(base)		   (base + 0x06C)
#define R5P0_LIMG_PITCH(base)			(base + 0x070)
#define R5P0_LIMG_CLIP_START(base)	   (base + 0x074)
#define R5P0_LIMG_CLIP_SIZE(base)		(base + 0x078)
#define R5P0_LIMG_DES_START(base)		(base + 0x07C)
#define R5P0_LIMG_PALLET_RGB(base)	   (base + 0x080)
#define R5P0_LIMG_CK(base)			   (base + 0x084)
#define R5P0_Y2Y_Y_PARAM(base)			 (base + 0x088)
#define R5P0_Y2Y_U_PARAM(base)			 (base + 0x08C)
#define R5P0_Y2Y_V_PARAM(base)			 (base + 0x090)

#define R5P0_LIMG_BASE_ADDR(base) (base + 0x060)
#define R5P0_LIMG_OFFSET 0x040

/*LAYEROSD*/
#define R5P0_LOSD_CFG(base)			  (base + 0x0E0)
#define R5P0_LOSD_R_ADDR(base)		   (base + 0x0E4)
#define R5P0_LOSD_PITCH(base)			(base + 0x0E8)
#define R5P0_LOSD_CLIP_START(base)	   (base + 0x0EC)
#define R5P0_LOSD_CLIP_SIZE(base)		(base + 0x0F0)
#define R5P0_LOSD_DES_START(base)		(base + 0x0F4)
#define R5P0_LOSD_PALLET_RGB(base)	   (base + 0x0F8)
#define R5P0_LOSD_CK(base)			   (base + 0x0FC)

#define R5P0_LOSD_BASE_ADDR(base) (base + 0x0E0)
#define R5P0_LOSD_OFFSET 0x020
#define R5P0_LOSD_SEC_ADDR(base) (base + 0x160)

#define R5P0_GSP_IP_REV(base)		 (base + 0x1A0)
#define R5P0_GSP_AXI_STS(base)		 (base + 0x1A4)
#define R5P0_GSP_DEBUG_CFG(base)	 (base + 0x1A8)
#define R5P0_GSP_DEBUG1(base)		 (base + 0x1AC)
#define R5P0_GSP_DEBUG2(base)		 (base + 0x1B0)

#define R5P0_SFBCD_IN_INFO(base)     (base + 0x600)
#define R5P0_SFBCD_OUT_INFO(base)     (base + 0x604)

#define R5P0_SCALE_COEF_ADDR(base)			(base + 0x200)
#define R5P0_SCALE_COEF_OFFSET		0x200

struct R5P0_GSP_GLB_CFG_REG {
	union {
		struct {
			uint32_t GSP_RUN		   :   1;
			uint32_t GSP_BUSY		:   1;
			uint32_t RAND_ID_MOD	:   1;
			uint32_t CMD_EN		  :   1;
			uint32_t CMD_ENDIAN_MOD		  :   4;
			uint32_t ERR_FLG		 :   1;
			uint32_t ERR_CODE		   :  6;
			uint32_t ACLK_FORCE				 : 1;
			uint32_t GAP_WB			:   8;
			uint32_t GAP_RB			:   8;
		};
		uint32_t	value;
	};
};

struct R5P0_GSP_INT_REG {
	union {
		struct {
			uint32_t INT_GSP_RAW	   :1;
			uint32_t INT_GERR_RAW	   :1;
			uint32_t Reserved1		 :6;
			uint32_t INT_GSP_EN		:1;
			uint32_t INT_GERR_EN		:1;
			uint32_t Reserved2		 :6;
			uint32_t INT_GSP_CLR	   :1;
			uint32_t INT_GERR_CLR	   :1;
			uint32_t Reserved3		 :6;
			uint32_t INT_GSP_STS	   :1;
			uint32_t INT_GERR_STS	   :1;
			uint32_t Reserved4		 :6;
		};
		uint32_t	value;
	};
};

struct R5P0_GSP_CMD_STS_REG {
	union {
		struct {
			uint32_t STAT_CMD_NUM  :   16;
			uint32_t NEW_C1		 :   1;
			uint32_t NEW_C2		 :   1;
			uint32_t NEW_D		 :   1;
			uint32_t NEW_L0		:   1;
			uint32_t NEW_L1		:   1;
			uint32_t NEW_L2		:   1;
			uint32_t NEW_L3		:   1;
			uint32_t NEW_L4		:   1;
			uint32_t NEW_L5		:   1;
			uint32_t Reserved	  :   6;
			uint32_t STOPC		:   1;
		};
		uint32_t	value;
	};
};

struct R5P0_GSP_CMD_COEF1_REG {
	union {
		struct {
			uint32_t COEF_CACHE_ADDR1   :32;
		};
		uint32_t	value;
	};
};

struct R5P0_GSP_CMD_COEF2_REG {
	union {
		struct {
			uint32_t COEF_CACHE_ADDR2   :32;
		};
		uint32_t	value;
	};
};

struct R5P0_GSP_CMD_ADDR_REG {
	union {
		struct {
			uint32_t CMD_BASE_ADDR	 :32;
		};
		uint32_t	value;
	};
};

struct R5P0_GSP_MOD_CFG_REG {
	union {
		struct {
			uint32_t RUN_MOD	   :   1;
			uint32_t SCALE1_EN	  :   1;
			uint32_t SCALE2_EN	  :   1;
			uint32_t PMARGB_EN	  :   1;
			uint32_t AWQOS	  :   4;
			uint32_t ARQOSL	 :   4;
			uint32_t ARQOSH	 :   4;
			uint32_t BLK_TIMER		 :   13;
			uint32_t Reserved3	 :   1;
			uint32_t AFBCE_AWLEN_MOD	 :   2;
		};
		uint32_t	value;
	};
};

struct R5P0_GSP_SECURE_CFG_REG {
	union {
		struct {
			uint32_t SECURE_MOD	   :   1;
			uint32_t NONSEC_AWPROT	  :   3;
			uint32_t NONSEC_ARPROT	 :   3;
			uint32_t SECURE_AWPROT	:   3;
			uint32_t SECURE_ARPROT	 :  3;
			uint32_t Reserved1	 :   19;
		};
		uint32_t	value;
	};
};

struct R5P0_DES_DATA_CFG1_REG {
	union {
		struct {
			uint32_t Y_ENDIAN_MOD		 :4;
			uint32_t UV_ENDIAN_MOD		 :4;
			uint32_t Reserved1			 :1;
			uint32_t A_SWAP_MOD			:1;
			uint32_t ROT_MOD			   :3;
			uint32_t R2Y_MOD			   :3;
			uint32_t DES_IMG_FORMAT		:3;
			uint32_t CR8_EN			:1;
			uint32_t R8_SWAP_MOD		  : 3;
			uint32_t R5_SWAP_MOD		:3;
			uint32_t FBCE_MOD			:2;
			uint32_t DITHER_EN			:1;
			uint32_t BK_EN				:1;
			uint32_t BK_BLD				:1;
			uint32_t Reserved2			:1;
		};
		uint32_t	value;
	};
};

struct R5P0_DES_Y_ADDR1_REG {
	union {
		struct {
			uint32_t Reserved	 :4;
			uint32_t DES_Y_BASE_ADDR1	 :28;
		};
		uint32_t	value;
	};
};

struct R5P0_DES_U_ADDR1_REG {
	union {
		struct {
			uint32_t Reserved	 :4;
			uint32_t DES_U_BASE_ADDR1	 :28;
		};
		uint32_t	value;
	};
};

struct R5P0_DES_V_ADDR1_REG {
	union {
		struct {
			uint32_t Reserved	 :4;
			uint32_t DES_V_BASE_ADDR1	 :28;
		};
		uint32_t	value;
	};
};

struct R5P0_DES_PITCH1_REG {
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

struct R5P0_BACK_RGB_REG {
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

struct R5P0_DES_SCL_SIZE1_REG {
	union {
		struct {
			uint32_t DES_SCL_W	  :13;
			uint32_t HTAP_MOD	  :2;
			uint32_t Reserved1	  :1;
			uint32_t DES_SCL_H	  :13;
			uint32_t VTAP_MOD	  :2;
			uint32_t Reserved2	  :1;
		};
		uint32_t	value;
	};
};

struct R5P0_DES_SCL_SIZE2_REG {
	union {
		struct {
			uint32_t DES_SCL_W	  :13;
			uint32_t HTAP_MOD	  :2;
			uint32_t Reserved1	  :1;
			uint32_t DES_SCL_H	  :13;
			uint32_t VTAP_MOD	  :2;
			uint32_t Reserved2	  :1;
		};
		uint32_t	value;
	};
};

struct R5P0_WORK_AREA_SIZE1_REG {
	union {
		struct {
			uint32_t WORK_AREA_W1	  :13;
			uint32_t Reserved1	   :3;
			uint32_t WORK_AREA_H1	  :13;
			uint32_t Reserved2	   :3;
		};
		uint32_t	value;
	};
};

struct R5P0_WORK_AREA_XY1_REG {
	union {
		struct {
			uint32_t WORK_AREA_X1	  :13;
			uint32_t Reserved1	   :3;
			uint32_t WORK_AREA_Y1	  :13;
			uint32_t Reserved2	   :3;
		};
		uint32_t	value;
	};
};

struct R5P0_WORK_AREA_SIZE2_REG {
	union {
		struct {
			uint32_t WORK_AREA_W2	  :13;
			uint32_t Reserved1	   :3;
			uint32_t WORK_AREA_H2	  :13;
			uint32_t Reserved2	   :3;
		};
		uint32_t	value;
	};
};

struct R5P0_WORK_AREA_SXY2_REG {
	union {
		struct {
			uint32_t WORK_AREA_SX2	  :13;
			uint32_t Reserved1	   :3;
			uint32_t WORK_AREA_SY2	  :13;
			uint32_t Reserved2	   :3;
		};
		uint32_t	value;
	};
};

struct R5P0_WORK_AREA_DXY2_REG {
	union {
		struct {
			uint32_t WORK_AREA_DX2	  :13;
			uint32_t Reserved1	   :3;
			uint32_t WORK_AREA_DY2	  :13;
			uint32_t Reserved2	   :3;
		};
		uint32_t	value;
	};
};

struct R5P0_GSP_IP_REV_REG {
	union {
		struct {
			uint32_t PATCH_NUM	   : 4;
			uint32_t GSP_IP_REV		   : 12;
			uint32_t Reserved1		   : 16;
		};
		uint32_t	   value;
	};
};

struct R5P0_GSP_AXI_STS_REG {
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

struct R5P0_GSP_DEBUG_CFG_REG {
	union {
		struct {
			uint32_t SCL_CLR1	   : 1;
			uint32_t SCL_CLR2		   : 1;
			uint32_t WCH_PAUSE		: 1;
			uint32_t RCH_PAUSE		   : 1;
			uint32_t CACHE_DIS		   : 1;
			uint32_t Reserved1		   : 27;
		};
		uint32_t	   value;
	};
};

struct R5P0_GSP_DEBUG1_REG {
	union {
		struct {
			uint32_t   LAYER0_DBG_STS		 :8;
			uint32_t   LAYER1_DBG_STS		 :8;
			uint32_t   Reserved1			 :8;
			uint32_t   SCL_OUT_EMP0			 :1;
			uint32_t   SCL_OUT_FULL0		 :1;
			uint32_t   SCL_OUT_EMP1			 :1;
			uint32_t   SCL_OUT_FULL1		 :1;
			uint32_t   BLD_OUT_EMP0			 :1;
			uint32_t   BLD_OUT_FULL0		 :1;
			uint32_t   BLD_OUT_EMP1			 :1;
			uint32_t   BLD_OUT_FULL1		 :1;
		};
		uint32_t	   value;
	};
};

struct R5P0_GSP_DEBUG2_REG {
	union {
		struct {
			uint32_t   LAYER2_DBG_STS		 :8;
			uint32_t   LAYER3_DBG_STS		 :8;
			uint32_t   LAYER4_DBG_STS		 :8;
			uint32_t   LAYER5_DBG_STS		 :8;
		};
		uint32_t	   value;
	};
};

struct R5P0_SFBCD_IN_INFO_REG {
	union {
		struct {
			uint32_t   Reserved			:27;
			uint32_t   SFBCD_HD_ARQOS		:4;
			uint32_t   SFBCD_BYPASS			:1;
		};
		uint32_t	   value;
	};
};

struct R5P0_SFBCD_OUT_INFO_REG {
	union {
		struct {
			uint32_t   Reserved			:11;
			uint32_t   SFBCD_FSM_STA		:5;
			uint32_t   SFBCD_FIFO_STA		:6;
			uint32_t   SFBC_AR_FIFO_STA		:2;
			uint32_t   AFBC_IFIFO_FULL		:1;
			uint32_t   AFBC_IFIFO_EMP		:1;
			uint32_t   AFBC_HFIFO_FULL		:1;
			uint32_t   AFBC_HFIFO_EMP		:1;
			uint32_t   INTER_CFIFO_FULL		:1;
			uint32_t   INTER_CFIFO_EMP		:1;
			uint32_t   RQUEUE_FULL			:1;
			uint32_t   RQUEUE_EMP			:1;
		};
		uint32_t	   value;
	};
};

/* LAYERIMG */
struct R5P0_LAYERIMG_CFG_REG {
	union {
		struct {
			uint32_t   Y_ENDIAN_MOD		  :4;
			uint32_t   UV_ENDIAN_MOD		 :4;
			uint32_t   RGB_SWAP_MOD		  :3;
			uint32_t   A_SWAP_MOD			:1;
			uint32_t   PMARGB_MOD			:1;
			uint32_t   ROT_SRC			  :3;
			uint32_t   IMG_FORMAT			:3;
			uint32_t   CK_EN				:1;
			uint32_t   PALLET_EN			 :1;
			uint32_t   VFBC_MOD			:2;
			uint32_t   Y2R_MOD				:3;
			uint32_t   Y2Y_MOD				:1;
			uint32_t   ZNUM_L			  :3;
			uint32_t   Reserved2		  :1;
			uint32_t   Limg_en			  :1;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYERIMG_Y_ADDR_REG {
	union {
		struct {
			uint32_t   Reserved1			:4;
			uint32_t   Y_BASE_ADDR			:28;
		};
		uint32_t	   value;
	};
};


struct R5P0_LAYERIMG_U_ADDR_REG {
	union {
		struct {
			uint32_t   Reserved1			:4;
			uint32_t   U_BASE_ADDR			:28;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYERIMG_V_ADDR_REG {
	union {
		struct {
			uint32_t   Reserved1			:4;
			uint32_t   V_BASE_ADDR			:28;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYERIMG_PITCH_REG {
	union {
		struct {
			uint32_t   PITCH				   :13;
			uint32_t   Reserved1				:3;
			uint32_t   HEIGHT				   :13;
			uint32_t   Reserved2				:3;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYERIMG_CLIP_START_REG {
	union {
		struct {
			uint32_t   CLIP_START_X			:13;
			uint32_t   Reserved1				:3;
			uint32_t   CLIP_START_Y			:13;
			uint32_t   Reserved2				:3;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYERIMG_CLIP_SIZE_REG {
	union {
		struct {
			uint32_t   CLIP_SIZE_X				:13;
			uint32_t   Reserved1				:3;
			uint32_t   CLIP_SIZE_Y				:13;
			uint32_t   Reserved2				:3;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYERIMG_DES_START_REG {
	union {
		struct {
			uint32_t   DES_START_X				:13;
			uint32_t   Reserved1				   :3;
			uint32_t   DES_START_Y				:13;
			uint32_t   Reserved2				   :3;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYERIMG_PALLET_RGB_REG {
	union {
		struct {
			uint32_t   PALLET_B			:8;
			uint32_t   PALLET_G			:8;
			uint32_t   PALLET_R			:8;
			uint32_t   PALLET_A			:8;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYERIMG_CK_REG {
	union {
		struct {
			uint32_t   CK_B				:8;
			uint32_t   CK_G				:8;
			uint32_t   CK_R				:8;
			uint32_t   BLOCK_ALPHA		 :8;
		};
		uint32_t	   value;
	};
};

struct R5P0_Y2Y_Y_PARAM_REG {
	union {
		struct {
			uint32_t   Y_CONTRAST			  :10;
			uint32_t   Reserved1				:6;
			uint32_t   Y_BRIGHTNESS			:9;
			uint32_t   Reserved2				:7;
		};
		uint32_t	   value;
	};
};

struct R5P0_Y2Y_U_PARAM_REG {
	union {
		struct {
			uint32_t   U_SATURATION			:10;
			uint32_t   Reserved1				:6;
			uint32_t   U_OFFSET				:8;
			uint32_t   Reserved2				:8;
		};
		uint32_t	   value;
	};
};

struct R5P0_Y2Y_V_PARAM_REG {
	union {
		struct {
			uint32_t   V_SATURATION			:10;
			uint32_t   Reserved1				:6;
			uint32_t   V_OFFSET				:8;
			uint32_t   Reserved2				:8;
		};
		uint32_t	   value;
	};
};

/* LAYEROSD */
struct R5P0_LAYEROSD_CFG_REG {
	union {
		struct {
			uint32_t   ENDIAN				:4;
			uint32_t   RGB_SWAP			  :3;
			uint32_t   A_SWAP				:1;
			uint32_t   PMARGB_MOD			:1;
			uint32_t   Reserved1			  :7;
			uint32_t   IMG_FORMAT			:2;
			uint32_t   CK_EN				 :1;
			uint32_t   PALLET_EN			 :1;
			uint32_t   VFBC_MOD			 :1;
			uint32_t   ZNUM_L				:3;
			uint32_t   Reserved2			  :7;
			uint32_t   Losd_en				  :1;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYEROSD_R_ADDR_REG {
	union {
		struct {
			uint32_t   Reserved1			  :4;
			uint32_t   R_BASE_ADDR			  :28;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYEROSD_PITCH_REG {
	union {
		struct {
			uint32_t   PITCH				   :13;
			uint32_t   Reserved1				:3;
			uint32_t   HEIGHT				   :13;
			uint32_t   Reserved2				:3;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYEROSD_CLIP_START_REG {
	union {
		struct {
			uint32_t   CLIP_START_X		   :13;
			uint32_t   Reserved1		   :3;
			uint32_t   CLIP_START_Y		   :13;
			uint32_t   Reserved2		   :3;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYEROSD_CLIP_SIZE_REG {
	union {
		struct {
			uint32_t   CLIP_SIZE_X				:13;
			uint32_t   Reserved1				:3;
			uint32_t   CLIP_SIZE_Y				:13;
			uint32_t   Reserved2				:3;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYEROSD_DES_START_REG {
	union {
		struct {
			uint32_t   DES_START_X				:13;
			uint32_t   Reserved1				:3;
			uint32_t   DES_START_Y				:13;
			uint32_t   Reserved2				:3;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYEROSD_PALLET_RGB_REG {
	union {
		struct {
			uint32_t   PALLET_B				:8;
			uint32_t   PALLET_G				:8;
			uint32_t   PALLET_R				:8;
			uint32_t   PALLET_A				:8;
		};
		uint32_t	   value;
	};
};

struct R5P0_LAYEROSD_CK_REG {
	union {
		struct {
			uint32_t   CK_B				:8;
			uint32_t   CK_G				:8;
			uint32_t   CK_R				:8;
			uint32_t   BLOCK_ALPHA		:8;
		};
		uint32_t	   value;
	};
};

struct R5P0_GSP_CTL_REG_T {
	struct R5P0_GSP_GLB_CFG_REG glb_cfg;
	struct R5P0_GSP_INT_REG int_cfg;
	struct R5P0_GSP_CMD_STS_REG cmd_sts;
	struct R5P0_GSP_CMD_COEF1_REG cmd_coef1_addr;
	struct R5P0_GSP_CMD_COEF2_REG cmd_coef2_addr;
	struct R5P0_GSP_CMD_ADDR_REG cmd_addr;
	struct R5P0_GSP_MOD_CFG_REG mod_cfg;
	struct R5P0_GSP_SECURE_CFG_REG secure_cfg;

	struct R5P0_DES_DATA_CFG1_REG des1_data_cfg;
	struct R5P0_DES_Y_ADDR1_REG des1_y_addr;
	struct R5P0_DES_U_ADDR1_REG  des1_u_addr;
	struct R5P0_DES_V_ADDR1_REG  des1_v_addr;
	struct R5P0_DES_PITCH1_REG  des1_pitch;
	struct R5P0_BACK_RGB_REG  back_rgb;
	struct R5P0_DES_SCL_SIZE1_REG  des_scl_size1;
	struct R5P0_DES_SCL_SIZE2_REG  des_scl_size2;
	struct R5P0_WORK_AREA_SIZE1_REG  work_area_size1;
	struct R5P0_WORK_AREA_XY1_REG  work_area_xy1;
	struct R5P0_WORK_AREA_SIZE2_REG  work_area_size2;
	struct R5P0_WORK_AREA_SXY2_REG  work_area_sxy2;
	struct R5P0_WORK_AREA_DXY2_REG  work_area_dxy2;

	struct R5P0_LAYERIMG_CFG_REG  limg_cfg[R5P0_IMGL_NUM];
	struct R5P0_LAYERIMG_Y_ADDR_REG  limg_y_addr[R5P0_IMGL_NUM];
	struct R5P0_LAYERIMG_U_ADDR_REG  limg_u_addr[R5P0_IMGL_NUM];
	struct R5P0_LAYERIMG_V_ADDR_REG  limg_v_addr[R5P0_IMGL_NUM];
	struct R5P0_LAYERIMG_PITCH_REG  limg_pitch[R5P0_IMGL_NUM];
	struct R5P0_LAYERIMG_CLIP_START_REG  limg_clip_start[R5P0_IMGL_NUM];
	struct R5P0_LAYERIMG_CLIP_SIZE_REG  limg_clip_size[R5P0_IMGL_NUM];
	struct R5P0_LAYERIMG_DES_START_REG  limg_des_start[R5P0_IMGL_NUM];
	struct R5P0_LAYERIMG_PALLET_RGB_REG  limg_pallet_rgb[R5P0_IMGL_NUM];
	struct R5P0_LAYERIMG_CK_REG  limg_ck[R5P0_IMGL_NUM];
	struct R5P0_Y2Y_Y_PARAM_REG  y2y_y_param[R5P0_IMGL_NUM];
	struct R5P0_Y2Y_U_PARAM_REG  y2y_u_param[R5P0_IMGL_NUM];
	struct R5P0_Y2Y_V_PARAM_REG  y2y_v_param[R5P0_IMGL_NUM];

	struct R5P0_LAYEROSD_CFG_REG  losd_cfg[R5P0_OSDL_NUM];
	struct R5P0_LAYEROSD_R_ADDR_REG  losd_r_addr[R5P0_OSDL_NUM];
	struct R5P0_LAYEROSD_PITCH_REG  losd_pitch[R5P0_OSDL_NUM];
	struct R5P0_LAYEROSD_CLIP_START_REG  losd_clip_start[R5P0_OSDL_NUM];
	struct R5P0_LAYEROSD_CLIP_SIZE_REG  losd_clip_size[R5P0_OSDL_NUM];
	struct R5P0_LAYEROSD_DES_START_REG  losd_des_start[R5P0_OSDL_NUM];
	struct R5P0_LAYEROSD_PALLET_RGB_REG  losd_pallet_rgb[R5P0_OSDL_NUM];
	struct R5P0_LAYEROSD_CK_REG  losd_ck[R5P0_OSDL_NUM];

	struct R5P0_LAYEROSD_CFG_REG  osdsec_cfg[R5P0_OSDSEC_NUM];
	struct R5P0_LAYEROSD_R_ADDR_REG  osdsec_r_addr[R5P0_OSDSEC_NUM];
	struct R5P0_LAYEROSD_PITCH_REG  osdsec_pitch[R5P0_OSDSEC_NUM];
	struct R5P0_LAYEROSD_CLIP_START_REG  osdsec_clip_start[R5P0_OSDSEC_NUM];
	struct R5P0_LAYEROSD_CLIP_SIZE_REG  osdsec_clip_size[R5P0_OSDSEC_NUM];
	struct R5P0_LAYEROSD_DES_START_REG  osdsec_des_start[R5P0_OSDSEC_NUM];
	struct R5P0_LAYEROSD_PALLET_RGB_REG  osdsec_pallet_rgb[R5P0_OSDSEC_NUM];
	struct R5P0_LAYEROSD_CK_REG  osdsec_ck[R5P0_OSDSEC_NUM];

	struct R5P0_GSP_IP_REV_REG ip_rev;
	struct R5P0_GSP_AXI_STS_REG axi_sts;
	struct R5P0_GSP_DEBUG_CFG_REG debug_cfg;
	struct R5P0_GSP_DEBUG1_REG debug1_cfg;
	struct R5P0_GSP_DEBUG2_REG debug2_cfg;

	struct R5P0_SFBCD_IN_INFO_REG sfbcd_in_info;
	struct R5P0_SFBCD_OUT_INFO_REG sfbcd_out_info;
};

#endif
