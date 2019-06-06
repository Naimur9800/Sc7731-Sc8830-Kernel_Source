/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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

#ifndef __SPRD_AON_BM_H__
#define __SPRD_AON_BM_H__

enum axi_busmon_djtag_ir {
	AXI_CHN_INT = 8,
	AXI_CHN_CFG,
	AXI_ID_CFG,
	AXI_ADDR_MIN,
	AXI_ADDR_MIN_H32,
	AXI_ADDR_MAX,
	AXI_ADDR_MAX_H32,
	AXI_ADDR_MASK,
	AXI_ADDR_MASK_H32,
	AXI_DATA_MIN_L32,
	AXI_DATA_MIN_H32,
	AXI_DATA_MIN_EXT_L32,
	AXI_DATA_MIN_EXT_H32,
	AXI_DATA_MAX_L32,
	AXI_DATA_MAX_H32,
	AXI_DATA_MAX_EXT_L32,
	AXI_DATA_MAX_EXT_H32,
	AXI_DATA_MASK_L32,
	AXI_DATA_MASK_H32,
	AXI_DATA_MASK_EXT_L32,
	AXI_DATA_MASK_EXT_H32,
	AXI_MON_TRANS_LEN,
	AXI_MATCH_ID,
	AXI_MATCH_ADDR,
	AXI_MATCH_ADDR_H32,
	AXI_MATCH_CMD,
	AXI_MATCH_DATA_L32,
	AXI_MATCH_DATA_H32,
	AXI_MATCH_DATA_EXT_L32,
	AXI_MATCH_DATA_EXT_H32,
	AXI_BUS_STATUS,
	AXI_USER_CFG,
	AXI_MATCH_USERID,
};

enum bm_dap_id {
	BM_DAP_0 = 0,
	BM_DAP_1,
	BM_DAP_2,
	BM_DAP_3,
	BM_DAP_4,
	BM_DAP_5,
	BM_DAP_6,
	BM_DAP_7,
	BM_DAP_8,
	BM_DAP_9,
	BM_DAP_10,
};

enum bm_rw {
	BM_READ = 0,
	BM_WRITE,
};

#define BIT_DJTAG_CHAIN_UPDATE_T		BIT(8)

#define BM_INT_MASK_STATUS			BIT(31)
#define BM_INT_RAW_STATUS			BIT(30)
#define BM_INT_CLR				BIT(29)
#define BM_INT_EN				BIT(28)
#define BM_BUF_CLR				BIT(27)
#define BM_RD_WR_SEL				BIT(26)
#define BM_BUF_RD_EN				BIT(25)
#define BM_BUF_EN				BIT(24)
#define BM_CNT_CLR				BIT(4)
#define BM_CNT_INTERNAL_START			BIT(3)
#define BM_CNT_EN				BIT(1)
#define BM_CHN_EN				BIT(0)
#define BM_AXI_SIZE_DWORD			(BIT(21) | BIT(22))
#define BM_AXI_SIZE_WORD			BIT(22)
#define BM_AXI_SIZE_HWORD			BIT(21)
#define BM_AXI_SIZE_EN				BIT(20)
#define BM_WRITE_CFG				BIT(1)
#define BM_WRITE_EN				BIT(0)

#define TEMP_BUF						(100)
#define USER_CFG_USERID_MASK					GENMASK(27, 0)
#define USER_CFG_ENUSERID					0x80000000
#define DJTAG_SOFTRESET_MASK					GENMASK(31, 0)
#define DJTAG_IR_LEN						0x9
#define DJTAG_DR_LEN						0x20
#define DJTAG_DAP_OFFSET					0x8
#define DJTAG_DAP_MUX_RESET					0x108
#define AUTOSCAN_CNT						16
#define MAX_DATA_VALUE					GENMASK(31, 0)

enum sprd_bm_mode {
	R_MODE,
	W_MODE,
};

struct __iomem sprd_djtag_reg {
	u32 ir_len_cfg;
	u32 dr_len_cfg;
	u32 ir_cfg;
	u32 dr_cfg;
	u32 dr_pause_recov_cfg;
	u32 rnd_en_cfg;
	u32 upd_dr_cfg;
	u32 autoscan_chain_addr[AUTOSCAN_CNT];
	u32 autoscan_chain_pattern[AUTOSCAN_CNT];
	u32 autoscan_chain_data[AUTOSCAN_CNT];
	u32 autoscan_chain_mask[AUTOSCAN_CNT];
	u32 autoscan_en;
	u32 autoscan_int_raw;
	u32 autoscan_int_mask;
	u32 autoscan_int_en;
	u32 autoscan_int_clr;
	u32 dap_mux_ctrl_rst;
};

struct bm_dbg_info {
	u32	bm_chn;
	u32	bm_match_id;
	u32	mon_min_l32;
	u32	mon_min_h32;
	u32	mon_max_l32;
	u32	mon_max_h32;
	u32	match_addr_l32;
	u32	match_addr_h32;
	u32	match_userid;
};

struct sprd_aon_bm_dev {
	spinlock_t				bm_lock;
	struct miscdevice				misc;
	struct regmap				*aon_apb;
	struct bm_dbg_info			dbg_info;
	void __iomem				*djtag_reg_base;
	int					bm_irq;
	int					bm_cnt;
	u32					subsys_aon;
	bool					panic;
	const char				**name_list;
	u32					*bm_dap;
};
#endif
