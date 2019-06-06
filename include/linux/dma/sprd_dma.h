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

#ifndef __SPRD_DMA_H
#define __SPRD_DMA_H
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/dmaengine.h>

#define DMA_UID_SOFTWARE		0

#define CHN_PRIORITY_OFFSET		12
#define CHN_WAIT_BDONE			24
#define CHN_PRIORITY_MASK		0x3
#define LLIST_EN_OFFSET			4

#define SRC_DATAWIDTH_OFFSET	30
#define DES_DATAWIDTH_OFFSET	28
#define SWT_MODE_OFFSET			26
#define REQ_MODE_OFFSET			24
#define ADDR_WRAP_SEL_OFFSET	23
#define ADDR_WRAP_EN_OFFSET		22
#define ADDR_FIX_SEL_OFFSET		21
#define ADDR_FIX_SEL_EN			20
#define LLIST_END_OFFSET		19
#define BLK_LEN_REC_H_OFFSET	17
#define FRG_LEN_OFFSET			0

#define DEST_TRSF_STEP_OFFSET	16
#define SRC_TRSF_STEP_OFFSET	0
#define DEST_FRAG_STEP_OFFSET	16
#define SRC_FRAG_STEP_OFFSET	0

#define SRC_CHN_OFFSET			0
#define DST_CHN_OFFSET			8
#define START_MODE_OFFSET		16
#define CHN_START_CHN			BIT(24)

#define TRSF_STEP_MASK			0xffff
#define FRAG_STEP_MASK			0xffff
#define WRAP_DATA_MASK			0xFFFFFFF
#define SRC_DEST_HADDR_MASK		0xF0000000

#define DATAWIDTH_MASK			0x3
#define SWT_MODE_MASK			0x3
#define REQ_MODE_MASK			0x3
#define ADDR_WRAP_SEL_MASK		0x1
#define ADDR_WRAP_EN_MASK		0x1
#define ADDR_FIX_SEL_MASK		0x1
#define ADDR_FIX_SEL_MASK		0x1
#define LLIST_END_MASK			0x1
#define BLK_LEN_REC_H_MASKT		0x3
#define FRG_LEN_MASK			0x1ffff
#define BLK_LEN_MASK			0x1ffff
#define TRSC_LEN_MASK			0x1fffffff

/* DMA glb offset and eb */
#ifndef BIT_AON_APB_AON_DMA_EB
#define BIT_AON_APB_AON_DMA_EB		BIT(22)
#define BIT_AON_APB_AON_DMA_SOFT_RST	BIT(6)
#endif
#ifndef REG_AON_APB_AON_DMA_INT_EN
#define REG_AON_APB_AON_DMA_INT_EN	0x11C
#endif
#ifndef BIT_AON_APB_AON_DMA_INT_AP_EN
#define BIT_AON_APB_AON_DMA_INT_AP_EN	BIT(0)
#endif
#ifndef REG_AON_APB_APB_EB2
#define REG_AON_APB_APB_EB2		0x14
#endif
#ifndef REG_AP_AHB_DMA_SOFT_RST
#define REG_AP_AHB_DMA_SOFT_RST	0x3040
#endif

#define REG_AGCP_AHB_MODULE_DMA_EN	0x0
#define REG_AGCP_AHB_MODULE_DMA_RST	0x8
#define BIT_AGCP_AP_DMA_EB			BIT(6)
#define BIT_AGCP_AP_DMA_ASHB_EB		BIT(17)
#define BIT_AGCP_AP_DMA_SOFT_RST	BIT(1)

#define DMA_OLD_GLB_PAUSE			BIT(16)
#define DMA_OLD_CHN_PAUSE			BIT(16)

#define DMA_NEW_GLB_PAUSE			BIT(2)
#define DMA_NEW_CHN_PAUSE			BIT(2)


typedef enum {
	BYTE_WIDTH = 0x0,
	SHORT_WIDTH,
	WORD_WIDTH,
} dma_datawidth;

typedef enum {
	NONE_STEP = 0x0,
	BYTE_STEP = BIT(0),
	SHORT_STEP = BIT(1),
	WORD_STEP = BIT(2),
} dma_step;

typedef enum {
	LINK_CTN = 0x0,
	LINK_END,
	LINK_CYCLE,
} dma_link_type;

typedef enum {
	FRAG_REQ_MODE = 0x0,
	BLOCK_REQ_MODE,
	TRANS_REQ_MODE,
	LIST_REQ_MODE,
} dma_request_mode;

typedef enum {
	NO_INT = 0x00,
	FRAG_DONE,
	BLK_DONE,
	TRANS_DONE,
	LIST_DONE,
	CONFIG_ERR,
	BLOCK_FRAG_DONE,
	TRANS_FRAG_DONE,
	TRANS_BLOCK_DONE,
} dma_int_type;

typedef enum {
	DMA_PRI_0 = 0,
	DMA_PRI_1,
	DMA_PRI_2,
	DMA_PRI_3,
} dma_pri_level;

typedef enum {
	DMA_WAIT_BDONE = 0,
	DMA_DONOT_WAIT_BDONE,
} dma_wait_bdone;

typedef enum {
	SET_IRQ_TYPE = 0x0,
	SET_WRAP_MODE,
	SET_REQ_MODE,
	SET_CHN_PRIO,
	SET_INT_TYPE,
} dma_cmd;

enum group_idx{
	DMA_CHN_GROUP_1 = 0,
	DMA_CHN_GROUP_2,
	MAX_CHN_GROUP
};

enum dma_switch_mode {
	DATA_ABCD = 0,
	DATA_DCBA,
	DATA_BADC,
	DATA_CDAB,
};

struct sprd_dma_glb_reg {
	u32 pause;
	u32 frag_wait;
	u32 pend0_en;
	u32 pend1_en;
	u32 int_raw_sts;
	u32 int_msk_sts;
	u32 req_sts;
	u32 en_sts;
	u32 debug_sts;
	u32 arb_sel_sts;
	/* channel start channel config */
	u32 cfg_group1;
	u32 cfg_group2;
};

struct sprd_dma_cfg {
	struct dma_slave_config config;
	unsigned long link_cfg_v;
	unsigned long link_cfg_p;
	u32 dev_id;
	dma_pri_level chn_pri;
	dma_datawidth datawidth;
	unsigned long src_addr;
	unsigned long des_addr;
	u32 fragmens_len;
	u32 block_len;
	u32 src_step;
	u32 des_step;
	dma_request_mode req_mode;
	dma_int_type irq_mode;
	enum dma_switch_mode swt_mode;
	/* only full chn need following config */
	u32 transcation_len;
	u32 src_frag_step;
	u32 dst_frag_step;
	u32 wrap_ptr;
	u32 wrap_to;
	u32 src_blk_step;
	u32 dst_blk_step;
	unsigned long linklist_ptr;
	u32 is_end;
};

enum dma_chn_type {
	STANDARD_DMA,
	FULL_DMA,
};

enum dma_chn_status {
	NO_USED,
	USED,
};

enum dma_link_list {
	LINKLIST,
	NO_LINKLIST,
};

enum request_mode {
	SOFTWARE_REQ,
	HARDWARE_REQ,
};

enum dma_flags {
	DMA_CFG_FLAG = BIT(0),
	DMA_HARDWARE_FLAG = BIT(1),
	DMA_SOFTWARE_FLAG = BIT(2),
	DMA_GROUP1_SRC = BIT(3),
	DMA_GROUP1_DST = BIT(4),
	DMA_GROUP2_SRC = BIT(5),
	DMA_GROUP2_DST = BIT(6),
	DMA_MUTL_FRAG_DONE = BIT(7),
	DMA_MUTL_BLK_DONE = BIT(8),
	DMA_MUTL_TRANS_DONE = BIT(9),
	DMA_MUTL_LIST_DONE = BIT(10),
};

enum config_type {
	CONFIG_DESC,
	CONFIG_LINKLIST,
};

enum addr_type {
    SPRD_SRC_ADDR = 0x55,
    SPRD_DST_ADDR = 0xAA,
};

#define DMA_VER_R4P0
/* DMA Request ID def */
#define DMA_SIM_RX		1
#define DMA_SIM_TX		2
#define DMA_IIS0_RX		3
#define DMA_IIS0_TX		4
#define DMA_IIS1_RX		5
#define DMA_IIS1_TX		6
#define DMA_IIS2_RX		7
#define DMA_IIS2_TX		8
#define DMA_IIS3_RX		9
#define DMA_IIS3_TX		10
#define DMA_SPI0_RX		11
#define DMA_SPI0_TX		12
#define DMA_SPI1_RX		13
#define DMA_SPI1_TX		14
#define DMA_SPI2_RX		15
#define DMA_SPI2_TX		16
#define DMA_UART0_RX		17
#define DMA_UART0_TX		18
#define DMA_UART1_RX		19
#define DMA_UART1_TX		20
#define DMA_UART2_RX		21
#define DMA_UART2_TX		22
#define DMA_UART3_RX		23
#define DMA_UART3_TX		24
#define DMA_UART4_RX		25
#define DMA_UART4_TX		26
#define DMA_DRM_CPT		27
#define DMA_DRM_RAW		28
#define DMA_VB_DA0		29
#define DMA_VB_DA1		30
#define DMA_VB_AD0		31
#define DMA_VB_AD1		32
#define DMA_VB_AD2		33
#define DMA_VB_AD3		34
#define DMA_GPS			35
#define DMA_SDIO0_RD		36
#define DMA_SDIO0_WR		37
#define DMA_SDIO1_RD		38
#define DMA_SDIO1_WR		39
#define DMA_SDIO2_RD		40
#define DMA_SDIO2_WR		41
#define DMA_EMMC_RD		42
#define DMA_EMMC_WR		43

#define DMA_IIS_RX	DMA_IIS0_RX
#define DMA_IIS_TX	DMA_IIS0_TX

int sprd_dma_check_register(struct dma_chan *c);
#endif
