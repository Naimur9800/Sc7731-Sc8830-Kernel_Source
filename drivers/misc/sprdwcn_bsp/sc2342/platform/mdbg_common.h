/*
* Copyright (C) 2015 Spreadtrum Communications Inc.
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/


#ifndef _MDBG_COMMON_H
#define _MDBG_COMMON_H
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/sdiom_rx_api.h>
#include <linux/sdiom_tx_api.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/wakelock.h>

#include "mdbg_type.h"

#define MDBG_SDIO_PACKER_TYPE		3

enum {
	MDBG_SUBTYPE_RING	= 0,
	MDBG_SUBTYPE_LOOPCHECK,
	MDBG_SUBTYPE_AT,
	MDBG_SUBTYPE_ASSERT,
};

#define MDBG_CACHE_FLAG_VALUE	(0xcdcddcdc)

#define WCN_DUMP_END_STRING "marlin_memdump_finish"
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
#define WCN_CP2_STATUS_DUMP_REG	0x6a6b6c6d
#endif

#define MDBG_RX_RING_SIZE		(2*1024*1024)
#define DUMP_PACKET_SIZE	(1024)
#define DUMP_WAIT_TIMEOUT	(100)
#define DUMP_WAIT_COUNT	(40)

#define SMP_HEADERFLAG 0X7E7E7E7E
#define SMP_RESERVEDFLAG 0X5A5A
#define SMP_DSP_CHANNEL_NUM 0X88
#define SMP_DSP_TYPE 0X9D
#define SMP_DSP_DUMP_TYPE 0X32

#define SYSNC_CODE_LEN 0X4
#define CHKSUM_LEN 0X2
#define ARMLOG_HEAD 9

#define SMP_HEAD_STR "at+smphead="

struct ring_rx_data {
	unsigned char		*addr;
	unsigned int		len;
	unsigned int		fifo_id;
	struct list_head	entry;
};

struct ring_device {
	struct mdbg_ring_t	*ring;
	struct wake_lock	rw_wake_lock;
	spinlock_t		rw_lock;
	struct mutex mdbg_read_mutex;
	struct list_head        rx_head;
	struct tasklet_struct   rx_task;
};

struct sme_head_tag {
	unsigned int seq_num;
	unsigned short len;
	unsigned char type;
	unsigned char subtype;
};

struct smp_head {
	unsigned int sync_code;
	unsigned short length;
	unsigned char channel_num;
	unsigned char packet_type;
	unsigned short reserved;
	unsigned short check_sum;
};

enum smp_diag_subtype_t {
	NORMAL_INFO = 0X0,
	DUMP_MEM_DATA,
	DUMP_MEM_END,
};

int mdbg_comm_init(void);
void mdbg_comm_remove(void);
long int mdbg_send(char *buf, long int len, unsigned int subtype);
long int mdbg_receive(void *buf, long int len);
int mdbg_dump_mem(void);
int mdbg_pt_common_reg(unsigned int subtype, void *func);
long mdbg_content_len(void);
void mdbg_clear_log(void);

int mdbg_read_release(unsigned int fifo_id);
unsigned int wcn_get_carddump_status(void);
unsigned int wcn_set_carddump_status(unsigned int flag);

unsigned long long mdbg_get_rx_total_cnt(void);
void wcn_hold_cpu(void);
void mdbg_hold_cpu(void);
bool mdbg_get_download_status(void);
int mdbg_get_module_status(void);
int mdbg_get_module_status_changed(void);
int mdbg_snap_shoot_iram(void *buf);
#endif
