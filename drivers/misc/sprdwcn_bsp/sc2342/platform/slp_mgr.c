/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 *
 * Filename : slp_mgr.c
 * Abstract : This file is a implementation for  sleep manager
 *
 * Authors	: sam.sun
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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/marlin_platform.h>
#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/sdiom_rx_api.h>
#include <linux/sdiom_tx_api.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/time.h>
#include <linux/wakelock.h>
#include "slp_mgr.h"

/*#define SLP_MGR_TEST */

static unsigned int sleep_active_modules;

static int sdio_public_int_num;
static atomic_t  cp2_state, exit_flag;
static struct mutex    wakeup_lock, drv_slp_lock;
static struct completion wakeup_ack, wakeup_thread_complet;
static struct wake_lock pub_int_handle_wakelock;

void slp_mgr_drv_sleep(enum marlin_sub_sys subsys, bool enable)
{
	mutex_lock(&drv_slp_lock);
	/* SLP_MGR_DBG("subsys-%d,set_slp-%d\n", subsys, enable); */
	if (enable)
		sleep_active_modules &= ~(BIT(subsys));
	else
		sleep_active_modules |= (BIT(subsys));
	mutex_unlock(&drv_slp_lock);
}

static int wake_cnt;
int slp_mgr_wakeup(enum marlin_sub_sys subsys)
{
	int ret = 0, retry_cnt = 0;

	mutex_lock(&wakeup_lock);

	if (atomic_read(&cp2_state) == STAY_SLPING) {

		wake_cnt++;
		SLP_MGR_INFO("subsys-%d wakeup, wake_cnt-%d\n",
				subsys, wake_cnt);

		if (wake_cnt == 1)
			mdelay(20);

WAKEUP:
		if (retry_cnt != 0) {
			SLP_MGR_INFO("retry wakeup\n");
			sdiom_aon_writeb(REG_CP_SLP_CTL, 1);
			udelay(100);
		}

		/* avoid fake interrupt or ack reply timeout */
		reinit_completion(&wakeup_ack);
		sdiom_aon_writeb(REG_CP_SLP_CTL, 0);
		/* for wakup success */
		ret = wait_for_completion_timeout(&wakeup_ack,
				msecs_to_jiffies(1000));
		if (ret == 0) {
			retry_cnt++;
			if (retry_cnt < 2)
				goto WAKEUP;

			SLP_MGR_ERR("wakeup fail, ret-%d !!!!", ret);
			sdiom_dump_aon_reg();
			atomic_set(&(cp2_state), STAY_AWAKING);
			mutex_unlock(&wakeup_lock);

			return -ETIMEDOUT;
		}

		atomic_set(&(cp2_state), STAY_AWAKING);
	}

	mutex_unlock(&wakeup_lock);

	return 0;
}

static int slp_mgr_allow_sleep(void)
{
	sdiom_aon_writeb(REG_CP_SLP_CTL, 1);

	return 0;
}

static int sdio_ap_int_en0(void)
{
	unsigned char reg_int_en0 = 0;

	sdiom_aon_readb(REG_AP_INT_EN0, &reg_int_en0);
	reg_int_en0 |= BIT_CP_CALI_NOTIFY_AP;
	reg_int_en0 |= BIT_CP_ACK_AP;
	reg_int_en0 |= BIT_CP_INT_AP;
	sdiom_aon_writeb(REG_AP_INT_EN0, reg_int_en0);
	sdiom_aon_readb(REG_AP_INT_EN0, &reg_int_en0);
	SLP_MGR_INFO("REG_AP_INT_EN0-0x%x\n", reg_int_en0);

	return 0;
}

/* clear sleep interrupt from cp req */
static int sdio_ap_ints_clr0(unsigned char int_sts)
{
	unsigned char reg_int_clr0 = 0;

	sdiom_aon_readb(REG_AP_INT_CLR0, &reg_int_clr0);
	reg_int_clr0 |= int_sts;
	sdiom_aon_writeb(REG_AP_INT_CLR0, reg_int_clr0);

	return 0;
}

/* send sdio irq to cp, and allow cp to enter sleep */
static int sdio_ap_int_cp0(void)
{
	unsigned char reg_int_cp0 = 0;

	sdiom_aon_readb(REG_AP_INT_CP0, &reg_int_cp0);
	reg_int_cp0 |= BIT_AP_INT_CP0;
	sdiom_aon_writeb(REG_AP_INT_CP0, reg_int_cp0);

	return 0;
}

static unsigned int ack_cnt, irq_cnt;

int slp_isr_handle_thread(void *data)
{
	unsigned char reg_ap_pub_int_sts0 = 0;
	struct sched_param param;

	param.sched_priority = 90;
	sched_setscheduler(current, SCHED_FIFO, &param);

	sdiom_aon_readb(REG_AP_PUB_INT_STS0, &reg_ap_pub_int_sts0);
	SLP_MGR_INFO("into slp_task, pub-sts: 0X%x!!", reg_ap_pub_int_sts0);
	while (!kthread_should_stop()) {
		if (!atomic_read(&exit_flag))
			wait_for_completion(&wakeup_thread_complet);
		else
			usleep_range(2000, 4000);

		/* read public interrupt status register */
		sdiom_aon_readb(REG_AP_PUB_INT_STS0, &reg_ap_pub_int_sts0);
		SLP_MGR_INFO("pub-sts: 0X%x!!", reg_ap_pub_int_sts0);

		sdio_ap_ints_clr0(reg_ap_pub_int_sts0);
		if (reg_ap_pub_int_sts0 & BIT_CP_CALI_NOTIFY_AP)
			marlin_read_cali_data();

		/* only marlin2 power on, then handle the below sleep case */
		if (flag_download) {

			/* BIT1: cp response ack to ap for wakeup */
			if (reg_ap_pub_int_sts0 & BIT_CP_ACK_AP) {
				complete(&wakeup_ack);
				ack_cnt++;
				SLP_MGR_INFO("ack_cnt-%d!!", ack_cnt);
			}

			/* BIT2: cp request into deep sleep */
			if (reg_ap_pub_int_sts0 & BIT_CP_INT_AP) {
				mutex_lock(&drv_slp_lock);
				/* allow sleep */
				if (sleep_active_modules == 0) {
					SLP_MGR_INFO("allow sleep\n");
					slp_mgr_allow_sleep();
					sdio_ap_int_cp0();
					/*
					* avoid REG_CP_SLP_CTL high level
					* not keep 2 cycle of 32khz
					*/
					udelay(80);
					atomic_set(&(cp2_state),
						STAY_SLPING);
				} else {
					SLP_MGR_INFO("forbid slp module-0x%x\n",
						sleep_active_modules);
				}
				mutex_unlock(&drv_slp_lock);
			}
		}

		wake_unlock(&pub_int_handle_wakelock);
		enable_irq(sdio_public_int_num);
	}
	return 0;
}

static irqreturn_t slp_mgr_req_sleep_isr(int irq, void *para)
{
	disable_irq_nosync(irq);
	wake_lock(&pub_int_handle_wakelock);
	irq_cnt++;
	if (irq_cnt < 3)
		SLP_MGR_INFO("irq_cnt-%d!!", irq_cnt);

	complete(&wakeup_thread_complet);

	return IRQ_HANDLED;
}

static struct task_struct *slp_isr_task;
static int slp_isr_handle_init(void)
{
	SLP_MGR_INFO("create slp_mgr_thread\n");
	if (!slp_isr_task)
		slp_isr_task = kthread_create(slp_isr_handle_thread,
			NULL, "slp_isr_thread");
	if (slp_isr_task != 0) {
		wake_up_process(slp_isr_task);
		return 0;
	}
	SLP_MGR_ERR("sam.sun create slp_isr_thread fail\n");

	return -1;
}

int slp_mgr_init(int irq)
{
	int ret;

	SLP_MGR_INFO("%s enter\n", __func__);

	mutex_init(&drv_slp_lock);
	mutex_init(&wakeup_lock);
	atomic_set(&cp2_state, STAY_AWAKING);
	atomic_set(&(exit_flag), 0);
	init_completion(&wakeup_ack);
	init_completion(&wakeup_thread_complet);
	wake_lock_init(&pub_int_handle_wakelock, WAKE_LOCK_SUSPEND,
		"pub_int_handle_wakelock");
	SLP_MGR_INFO("public_int, gpio-%d\n", irq);

	ret = gpio_direction_input(irq);
	if (ret < 0) {
		SLP_MGR_ERR("public_int, gpio-%d input set fail!!!", irq);
		return ret;
	}

	sdio_public_int_num = gpio_to_irq(irq);

	ret = request_irq(sdio_public_int_num,
			slp_mgr_req_sleep_isr,
			IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND,
			"slp_mgr_irq",
			NULL);
	if (ret != 0) {
		SLP_MGR_ERR("req irq-%d err!!!", sdio_public_int_num);
		return ret;
	}

	sdio_ap_int_en0();

	slp_isr_handle_init();

	SLP_MGR_INFO("%s ok!\n", __func__);
	return 0;
}

int slp_mgr_exit(void)
{
	SLP_MGR_INFO("%s enter\n", __func__);
	atomic_set(&(exit_flag), 1);
	sleep_active_modules = 0;
	if (slp_isr_task) {
		disable_irq(sdio_public_int_num);
		complete(&wakeup_thread_complet);
		kthread_stop(slp_isr_task);
		slp_isr_task = NULL;
	}

	disable_irq(sdio_public_int_num);
	free_irq(sdio_public_int_num, NULL);
	atomic_set(&cp2_state, STAY_AWAKING);
	wake_cnt = 0;
	ack_cnt = 0;
	irq_cnt = 0;
	mutex_destroy(&drv_slp_lock);
	mutex_destroy(&wakeup_lock);
	wake_lock_destroy(&pub_int_handle_wakelock);
	SLP_MGR_INFO("%s ok!\n", __func__);

	return 0;
}
