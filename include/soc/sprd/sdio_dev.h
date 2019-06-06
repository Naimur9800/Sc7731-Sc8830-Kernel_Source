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

#ifndef __SDIO_DEV_H__
#define __SDIO_DEV_H__

#include <linux/module.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/suspend.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/wakelock.h>
#include <linux/pm.h>

typedef irqreturn_t(*sdiodev_handler_t) (int, void *);
typedef void (*sdio_tran_callback) (void);
typedef void (*sdio_tran_callback_para) (int);

struct sdio_chn_handler {
	unsigned int chn;
	sdio_tran_callback tran_callback;
	sdio_tran_callback_para tran_callback_para;
};

struct sleep_policy_t {
	struct timer_list gpio_timer;
	unsigned int marlin_waketime;
	unsigned int bt_req_time;
	unsigned int ack_high_time;
	unsigned int wake_lock_time;
	unsigned int  gpio_opt_tag;
	unsigned long gpioreq_up_time;
	unsigned long gpioreq_need_pulldown;
	unsigned long gpio_up_time;
	unsigned long gpio_down_time;
};

struct marlin_sdio_ready_t {
	bool marlin_sdio_init_start_tag;
	bool marlin_sdio_init_end_tag;
};

struct sdio_data {
	u32 data_irq;
	u32 io_ready;
	u32 rfctl_off;
	u32 wake_ack;
	u32 wake_out;
	const char *sdhci;
};

#define MARLIN_SDIO_VERSION		"1.0"
#define INVALID_SDIO_CHN		16
#define INVALID_SDIO_WCHN		8
#define BIT_0				0x01
#define SDIODEV_MAX_FUNCS		2
#define SDIODEV_FUNC_0			0
#define SDIODEV_FUNC_1			1
#define MARLIN_VENDOR_ID		0x00
#define MARLIN_DEVICE_ID		0x2331
#define SDIO_CHN_8			0x1
#define SDIO_CHN_9			0x2
#define SDIO_CHN_10			0x4
#define SDIO_CHN_11			0x8
#define SDIO_CHN_12			0x10
#define SDIO_CHN_13			0x20
#define SDIO_CHN_14			0x40
#define SDIO_CHN_15			0x80
#define SDIOLOG_CHN			14
#define FM_CHANNEL_READ			10
#define DOWNLOAD_CHANNEL_READ		12
#define PSEUDO_ATC_CHANNEL_READ		11
#define PSEUDO_ATC_CHANNEL_LOOPCHECK	(15)
#define MARLIN_ASSERTINFO_CHN		(13)
#define WIFI_CHN_8			8
#define WIFI_CHN_9			9

/* SMP */
#define SMP_BARRIER(x) smp_read_barrier_depends(x)

#define SDIOTRAN_ERR(fmt, args...)	\
	pr_err("%s:" fmt "\n", __func__, ## args)
#define SDIOTRAN_DEBUG(fmt, args...)	\
	pr_debug("%s:" fmt "\n", __func__, ## args)
#define MARLIN_PM_RESUME_WAIT_INIT(a)
#define _MARLIN_PM_RESUME_WAIT(a, b)
#define MARLIN_PM_RESUME_WAIT(a)
#define MARLIN_PM_RESUME_WAIT_FOREVER(a)
#define MARLIN_PM_RESUME_RETURN_ERROR(a)

int  sprd_set_wlan_status(int status);
int  sprd_set_marlin_wakeup(unsigned int chn, unsigned int user_id);
int  sprd_set_marlin_sleep(unsigned int chn, unsigned int user_id);
int  sprd_sdio_dev_chn_idle(unsigned int chn);
int  sprd_sdio_dev_get_chn_datalen(unsigned int chn);
int  sprd_sdio_dev_get_read_chn(void);
bool sprd_get_sdiohal_status(void);
bool sprd_get_apsdiohal_status(void);
int sprd_sdio_dev_write(unsigned int chn, void *data_buf, unsigned int count);
int sprd_sdio_dev_read(unsigned int chn, void *read_buf, unsigned int *count);
int sprd_sdiodev_readchn_init(unsigned int chn, void *callback, bool with_para);
int sprd_sdiodev_readchn_uninit(unsigned int chn);
void sprd_set_download_fin(int dl_tag);
int  sprd_marlin_sdio_init(void);
bool sprd_get_marlin_status(void);
int  sprd_get_marlin_rx_status(void);
void set_blklen(int blklen);
void sprd_flush_blkchn(void);

#endif
