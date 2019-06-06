/*
* Copyright (C) 2017 Spreadtrum Communications Inc.
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/
#ifndef __GNSS_DUMP_H__
#define __GNSS_DUMP_H__

#define DUMP_PACKET_SIZE		(1024)
#define GNSS_SHARE_MEMORY_SIZE		(0x15a800)
#define GNSS_CP_IRAM_DATA_NUM		8192
#define DUMP_IRAM_START_ADDR		0x18000000

/*cp reg start*/
/* APB */
#define DUMP_REG_GNSS_APB_CTRL_ADDR	0xA0060000
#define DUMP_REG_GNSS_APB_CTRL_LEN	0x400

/* AHB */
#define DUMP_REG_GNSS_AHB_CTRL_ADDR	0xC0300000
#define DUMP_REG_GNSS_AHB_CTRL_LEN	0x400
/* Com_sys */
#define DUMP_COM_SYS_CTRL_ADDR		0xD0020800
#define DUMP_COM_SYS_CTRL_LEN		0x10

/* wcn_cp_clk */
#define DUMP_WCN_CP_CLK_CORE_ADDR	0xD0020000
#define DUMP_WCN_CP_CLK_LEN		0x100
/*cp reg end*/

#define ANLG_WCN_WRITE_ADDR 0XFF4
#define ANLG_WCN_READ_ADDR 0XFFC

int gnss_dump_mem(char flag);

#endif
