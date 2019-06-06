/*
 * FM Radio driver with SPREADTRUM SC2331FM Radio chip
 *
 * Copyright (c) 2015 Spreadtrum
 * Author: Songhe Wei <songhe.wei@spreadtrum.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <soc/sprd/sdio_dev.h>
#include "fmdrv.h"
#include "fmdrv_ops.h"
#include "fmdrv_main.h"
#include "fmdrv_rds_parser.h"

#define FM_CHANNEL_WRITE 5
#define FM_CHANNEL_READ 10
#define FM_WRITE_SIZE	(64)
#define FM_READ_SIZE	(128)

bool read_flag;

struct fmdrv_ops *fmdev;
static struct fm_rds_data *g_rds_data_string;

static void receive_tasklet(unsigned long arg)
{
	unsigned char *pdata;/* the data from SDIO is event data*/

	read_flag = 1;
	pdata = fmdev->read_buf;
	if (*pdata == '1') {
		pdata = pdata + 2;
		rds_parser(pdata, 12);
	}

	else {
		pr_info("receive_tasklet before completion=0x%x",
			fmdev->completed.done);
		complete(&fmdev->completed);
		pr_info("receive_tasklet after completion=0x%x",
			fmdev->completed.done);
	}

}

/*
* read cmd :at+fm=0
* RDS data  event:status
*
*/

void fm_marlin2_rds_read(unsigned char *buffer, unsigned char len)

{
	rds_parser(buffer, len);
}

/* ssize_t */
int fm_read_rds_data(struct file *filp, char __user *buf,
	size_t count, loff_t *pos)
{
/*	RDSData_Struct rds_data; */
	if (copy_to_user(buf, &(fmdev->rds_data), sizeof(fmdev->rds_data))) {
		pr_info("fm_read_rds_data ret value is -eFAULT\n");
		return -EFAULT;
	}
	pr_info("fmevent_status=%x", fmdev->rds_data.event_status);
	return sizeof(fmdev->rds_data);

}

void parse_at_fm_cmd(unsigned short *freq_found)
{
	int comma_cou = 0;
	int i = 0;
	int cmdstart = 0;
	int len = 0;
	char *cur_ptr;
	char num_str[6] = {0};
	int result = 0;

	cur_ptr = fmdev->read_buf;
	read_flag = 0;
	for (i = 0; i < 32 && cur_ptr[i] != '\0'; i++) {
		if (cur_ptr[i] == ',')
			comma_cou++;
		if (comma_cou == 3) {
			comma_cou = 0;
			cmdstart = i;
		}
	}
	cmdstart += 1;
	for (i = 0; cmdstart < 32 && cur_ptr[cmdstart] != '\0'
		&& cur_ptr[cmdstart] != ','; i++, cmdstart++) {
		if (cur_ptr[cmdstart] >= '0' && cur_ptr[cmdstart] <= '9')
			num_str[i] = cur_ptr[cmdstart];
		else if (cur_ptr[cmdstart] == ' ')
			break;
	}
	len = strlen(num_str);
	cur_ptr = num_str;
	result = cur_ptr[0] - '0';
	for (i = 1; i < len; i++)
		result = result * 10 + cur_ptr[i] - '0';
	*freq_found = result;
	pr_info("fm seek event have come freq=%d", result);

}

int fm_open(struct inode *inode, struct file *filep)
{
	return 0;
}

void fm_sdio_read(void)
{
	memset(fmdev->read_buf, 0, FM_READ_SIZE);
	fmdev->rcv_len = sprd_sdio_dev_get_chn_datalen(FM_CHANNEL_READ);
	if (fmdev->rcv_len <= 0) {
		pr_err("FM_CHANNEL_READ len err\n");
		return;
	}
	if (fmdev->rcv_len > FM_READ_SIZE)
		pr_err("The read data len:%d, beyond max read:%d",
		fmdev->rcv_len, FM_READ_SIZE);
	sprd_sdio_dev_read(FM_CHANNEL_READ, fmdev->read_buf, &fmdev->rcv_len);
	pr_info("* fmdev->read_buf: %s *\n", (char *)fmdev->read_buf);

}

int fm_sdio_write(char *buffer, unsigned int size)
{
	sprd_sdio_dev_write(FM_CHANNEL_WRITE, buffer, size);
	pr_info("write size is %d", size);
	return size;
}

int fm_sdio_init(void)
{
	int retval = 0;

	retval = sprd_sdiodev_readchn_init(FM_CHANNEL_READ, fm_sdio_read, 0);
	if (retval != 0) {
		pr_err("Sdio dev read channel init failed!");
		retval = -1;
	}
	return retval;
}

void fm_read(void)
{
	fm_sdio_read();
	tasklet_schedule(&fmdev->rx_task);
}

static int fm_write(unsigned char *array)
{
	unsigned long timeleft;
	int cnt = 0;
	int len = 0;

	pr_info("fm start wake marlin\n");
	while ((sprd_set_marlin_wakeup(FM_CHANNEL_WRITE, 0x3) < 0)
			&& (cnt <= 3)) {
		msleep(300);
		cnt++;
	}
	cnt = 0;
	len = strlen(array);
	fm_sdio_write(array, len);
	pr_info("fm: write_buf: %s\n", array);
	pr_info("fm_write_completion done=0X%x", fmdev->completed.done);
	timeleft = wait_for_completion_timeout(&fmdev->completed,
		FM_DRV_TX_TIMEOUT);
	pr_info("fm_write_after_completion done=0X%x", fmdev->completed.done);
	if (!timeleft) {
		pr_err("Timeout, %d\n", ETIMEDOUT);
		return -ETIMEDOUT;
	}

	pr_info("success!");
	return 0;
}

/*
* open cmd :at+fm=0
* open event:status
*
*/

int fm_powerup(void)
{
	int ret = 0;

	sprintf(fmdev->write_buf, "at+fm=0\r\n");
	ret = fm_write(fmdev->write_buf);
	return ret;
}

/*
* close cmd :at+fm=12
* close event:status
*
*/

int fm_powerdown(void)
{
	int ret = 0;

	sprintf(fmdev->write_buf, "at+fm=12\r\n");
	ret = fm_write(fmdev->write_buf);
	return ret;
}

/*
* tune cmd :at+fm=1,freq
* tune event:status,RSSI,SNR,Freq
*
*/

int fm_tune(void *arg)
{
	struct fm_tune_parm parm;
	int ret = 0;

	if (copy_from_user(&parm, arg, sizeof(parm))) {
		pr_info("fm tune 's ret value is -eFAULT\n");
		return -EFAULT;
	}
	parm.freq *= 10;
	sprintf(fmdev->write_buf, "at+fm=1,%d\r\n", parm.freq);
	ret = fm_write(fmdev->write_buf);
	return ret;
}
/*
* seek cmd :at+fm=4,freq,drection
*seek event:status,RSSI,SNR,Freq
*
*/
int fm_seek(void *arg)
{
	struct fm_seek_parm parm;
	int ret = 0;

	if (copy_from_user(&parm, arg, sizeof(parm))) {
		pr_info("fm seek 's ret value is -eFAULT\n");
		return -EFAULT;
	}
	pr_info("FM seek freq =%d", parm.freq);
	parm.freq *= 10;
	sprintf(fmdev->write_buf, "at+fm=4,%d,%d\r\n",
		parm.freq, parm.seekdir);
	ret = fm_write(fmdev->write_buf);
	if (read_flag == 1)
		parse_at_fm_cmd(&parm.freq);
	parm.freq /= 10;
	if (copy_to_user(arg, &parm, sizeof(parm)))
		ret = -EFAULT;
	fmdev->rcv_len = 0;
	return ret;
}
int fm_mute(void *arg)
{
	int mute = 0;

	if (copy_from_user(&mute, arg, sizeof(mute))) {
		pr_info("fm mute 's ret value is -eFAULT\n");
		return -EFAULT;
	}
	pr_info("fm_mute: %d", mute);
	return 0;
}

struct fm_rds_data *get_rds_data(void)
{
	pr_info("fm get rds data\n");
	return g_rds_data_string;

}

int fm_rds_onoff(void *arg)
{
	unsigned short is_on;
	int ret = 0;

	if (copy_from_user(&is_on, arg, sizeof(is_on))) {
		pr_info("fm rds_onoff 's ret value is -eFAULT\n");
		return -EFAULT;
	}
	sprintf(fmdev->write_buf, "at+fm=6,%d\r\n", is_on);
	ret = fm_write(fmdev->write_buf);
	return ret;
}

void set_rds_drv_data(struct fm_rds_data *fm_rds_info)
{
	/*memcpy(g_rds_data_string, fm_rds_info, sizeof(struct fm_rds_data)); */
	g_rds_data_string = fm_rds_info;


}

int __init init_fm_driver(void)
{
	int ret;
	int retval = 0;
	struct fm_rds_data *fm_rds_info;

	fmdev = kzalloc(sizeof(struct fmdrv_ops), GFP_KERNEL);
	if (!fmdev)
		return -ENOMEM;
	init_completion(&fmdev->completed);
	fmdev->read_buf =  kzalloc(FM_READ_SIZE, GFP_KERNEL);
/*malloc mem for rds struct */
	fm_rds_info = kzalloc(sizeof(struct fm_rds_data), GFP_KERNEL);
	if (NULL == fm_rds_info) {
		ret = -ENOMEM;
		/*pr_err("Could not allocate RDS buffer\n");*/
		return ret;
	}
	set_rds_drv_data(fm_rds_info);
	retval = sprd_sdiodev_readchn_init(FM_CHANNEL_READ, fm_read, 0);

	ret = fm_device_init_driver();

	tasklet_init(&fmdev->rx_task, receive_tasklet, (unsigned long)fmdev);

	return ret;
}

void __exit exit_fm_driver(void)
{
	fm_device_exit_driver();
	tasklet_kill(&fmdev->rx_task);
	kfree(fmdev->read_buf);
	fmdev->read_buf = NULL;
	kfree(fmdev);
	fmdev = NULL;
}

module_init(init_fm_driver);
module_exit(exit_fm_driver);
/* ------------- Module Info ------------- */
MODULE_DESCRIPTION("Radio driver for marlin share SDIO with BT/WIFI");
MODULE_AUTHOR("Songhe Wei<songhe.wei@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(FM_VERSION);
