/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 *
 * Filename : sdio_dev.h
 * Abstract : This file is a implementation for itm sipc command/event function
 *
 * Authors	: gaole.zhang
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __SLP_MGR_H__
#define __SLP_MGR_H__

#define SLEEP_DRV_SDIO	BIT(0)
#define SLEEP_DRV_WIFI	BIT(1)
#define SLEEP_DRV_BT	BIT(2)
#define SLEEP_DRV_FM	BIT(3)
/* tmp config format */
#define REG_CP_SLP_CTL	0x1a2
#define REG_AP_INT_CP0	0x1b0
#define REG_AP_INT_CLR0	0x1d0
#define REG_AP_INT_EN0	0x1c0
#define REG_AP_PUB_INT_STS0	0x1f0
#define BIT_AP_INT_CP0	BIT(1)
#define BIT_AP_INT_CLR0	BIT(2)
#define BIT_AP_INT_EN0	BIT(2)
#define BIT_CP_CALI_NOTIFY_AP	BIT(0)
#define BIT_CP_ACK_AP	BIT(1)
#define BIT_CP_INT_AP	BIT(2)

#define STAY_SLPING	0
#define	STAY_AWAKING	1

#define OK		(0)
#define ERROR		(-1)

extern unsigned char flag_download;
#define SLP_MGR_HEADER "[slp_mgr]"

#define SLP_MGR_ERR(fmt, args...)	\
	pr_err(SLP_MGR_HEADER fmt "\n", ## args)
#define SLP_MGR_INFO(fmt, args...)	\
	pr_info(SLP_MGR_HEADER fmt "\n", ## args)
#define SLP_MGR_DBG(fmt, args...)	\
	pr_debug(SLP_MGR_HEADER fmt "\n", ## args)


extern void marlin_read_cali_data(void);
extern void mdbg_assert_interface(char *str);
int slp_mgr_init(int irq);
int slp_mgr_exit(void);
void slp_mgr_drv_sleep(enum marlin_sub_sys subsys, bool enable);
int slp_mgr_wakeup(enum marlin_sub_sys subsys);

#endif
