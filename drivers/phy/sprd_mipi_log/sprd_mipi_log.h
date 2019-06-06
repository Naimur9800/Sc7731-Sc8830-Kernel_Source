/*
 *Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 *This software is licensed under the terms of the GNU General Public
 *License version 2, as published by the Free Software Foundation, and
 *may be copied, distributed, and modified under those terms.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 */

#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include "mipi_log_dphy.h"

#define REG_SERDES_APB_FUNC_EN			(0x00)
#define REG_SERDES_APB_CH_EN			(0x04)
#define REG_SERDES_APB_FUNNEL_EN		(0x08)
#define REG_SERDES_APB_FSM_CUT_OFF_LEN		(0x10)

#define BIT_SERDES_APB_MDAR			BIT(0)
#define BIT_SERDES_APB_WTL			BIT(1)
#define BIT_SERDES_APB_DBG_SYS			BIT(2)
#define BIT_SERDES_APB_DBG_BUS			BIT(3)
#define BIT_SERDES_APB_TRAINING			BIT(15)


#define REG_APB_EB1				0x0004
#define REG_PWR_CTRL				0x0024
#define REG_2P2P_S_PHY_CTRL			0x0078
#define REG_2P2P_M_PHY_CTRL			0x007C
#define REG_2P2L_DBG_PHY_CTRL			0x0080
#define REG_APB_EB2				0x00B0
#define REG_APB_RST2				0x0130

#define BIT_SERDES_DPHY_APB_SORT_REST		BIT(13)
#define BIT_SERDES_DPHY_EB			BIT(31)
#define BIT_MIPI_2P2L_PS_PD_M			BIT(13)
#define BIT_MIPI_2P2L_PS_PD_L			BIT(12)
#define BIT_SERDES_DPHY_REF_EB			BIT(9)
#define BIT_SERDES_DPHY_CEG_EB			BIT(8)
#define BIT_2P2L_DBG_EN				BIT(25)
#define BIT_BUGMON_DMA_EB			BIT(10)
#define BIT_2P2L_TESTCLR_S			BIT(23)
#define BIT_2P2L_TESTCLR_S_SET			BIT(22)
#define BIT_2P2L_TESTCLR_M			BIT(23)
#define BIT_2P2L_TESTCLR_M_SET			BIT(22)

#define REG_DBG_PHY_CTRL_0			0x000C
#define BIT_SHUTDOWN_DB				BIT(4)

enum channel_en {
	CH_EN_TRAINING = 1,
	CH_EN_WTL,
	CH_EN_MDAR,
	CH_EN_DBG_SYS,
	CH_EN_DBG_BUS,
	CH_EN_MAX
};


struct dsi_controller_info {
	unsigned long address;
	int dev_id;
};

struct mipi_log_device {
	struct device *dev;
	dphy_t phy_instance;
	uint32_t max_lanes;
	uint32_t phy_freq;
	uint32_t channel;
	bool running;
};

int mipi_log_start(struct mipi_log_device *mipi_log, enum channel_en ch_en);
int mipi_log_stop(struct mipi_log_device *mipi_log);
int mipi_log_create_sysfs(struct mipi_log_device *mipi_log);

#endif
