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

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <misc/mdbg_common.h>
#include <linux/miscdevice.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <soc/sprd/sdio_dev.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>

#define	DLOADER_NAME				"download"
#define	DOWNLOAD_CHANNEL_READ			12
#define	DOWNLOAD_CHANNEL_WRITE			3
#define	DOWNLOAD_CALI_WRITE			0
#define	READ_BUFFER_SIZE			(4096)
#define	WRITE_BUFFER_SIZE			(129*1024)

bool flag_read;
unsigned int read_len;

struct dloader_dev {
	int open_count;
	atomic_t read_excl;
	atomic_t write_excl;
	char *read_buffer;
	char *write_buffer;
};

static struct dloader_dev *download_dev;
static struct wake_lock download_wake_lock;
static bool flag_cali;

static inline int sprd_download_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) != 1) {
		atomic_dec(excl);
		return -1;
	}

	return 0;
}

static inline void sprd_download_unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

static void sprd_download_sdio_read(void)
{
	read_len = sprd_sdio_dev_get_chn_datalen(DOWNLOAD_CHANNEL_READ);
	if (read_len <= 0)
		return;

	sprd_sdio_dev_read(DOWNLOAD_CHANNEL_READ,
			   download_dev->read_buffer, &read_len);
	pr_info("%s read: %s\n", __func__, download_dev->read_buffer);
	flag_read = 1;
}

static int sprd_download_sdio_write(char *buffer, unsigned int size)
{
	if (flag_cali)
		sprd_sdio_dev_write(DOWNLOAD_CALI_WRITE, buffer, size);
	else
		sprd_sdio_dev_write(DOWNLOAD_CHANNEL_WRITE, buffer, size);
	printk_ratelimited("%s size: %d\n", __func__, size);

	return size;
}

static int sprd_download_sdio_init(void)
{
	int retval = 0;

	retval = sprd_sdiodev_readchn_init(DOWNLOAD_CHANNEL_READ,
					   sprd_download_sdio_read, 0);
	if (retval != 0) {
		pr_err("Sdio dev read channel init failed!");
		retval = -1;
	}

	return retval;
}

static int sprd_download_open(struct inode *inode, struct file *filp)
{
	if (download_dev == NULL)
		return -EIO;

	if (download_dev->open_count != 0) {
		pr_err("dloader_open %d\n", download_dev->open_count);
		return -EBUSY;
	}

	sprd_marlin_sdio_init();
	sprd_mdbg_channel_init();
	flag_read = 0;

	download_dev->open_count++;
	wake_lock(&download_wake_lock);
	if (sprd_download_sdio_init() < 0) {
		pr_err("sprd_download_sdio_init failed\n");
		return -EBUSY;
	}

	pr_info("sprd_download_open %d\n", download_dev->open_count);

	return 0;
}

static int sprd_download_read(struct file *filp,
			      char __user *buf, size_t count, loff_t *pos)
{
	if ((download_dev == NULL) || (download_dev->open_count != 1)) {
		pr_err("%s return point 1.\n", __func__);
		return -EIO;
	}

	if (sprd_download_lock(&download_dev->read_excl)) {
		pr_err("%s return point 2.\n", __func__);
		return -EBUSY;
	}

	count = (count > READ_BUFFER_SIZE) ? READ_BUFFER_SIZE : count;

	if (!flag_read) {
		sprd_download_unlock(&download_dev->read_excl);
		return 0;
	}

	if (copy_to_user(buf, download_dev->read_buffer, read_len)) {
		sprd_download_unlock(&download_dev->read_excl);
		return -EFAULT;
	}
	flag_read = 0;
	sprd_download_unlock(&download_dev->read_excl);

	return read_len;
}

static int sprd_download_write(struct file *filp,
			       const char __user *buf, size_t count,
			       loff_t *pos)
{
	if ((download_dev == NULL) || (download_dev->open_count != 1))
		return -EIO;

	if (sprd_download_lock(&download_dev->write_excl))
		return -EBUSY;

	if (count > WRITE_BUFFER_SIZE) {
		sprd_download_unlock(&download_dev->write_excl);
		return -EINVAL;
	}

	if (copy_from_user(download_dev->write_buffer, buf, count)) {
		sprd_download_unlock(&download_dev->write_excl);
		return -EFAULT;
	}

	while (sprd_get_apsdiohal_status() != 1) {
		pr_info("SDIO dev not ready, wait 50ms and try again!\n");
		msleep(50);
	}

	if (strncmp(download_dev->write_buffer, "start_calibration", 17) == 0) {
		pr_info("wait marlin ready start\n");
		/* wait marlin ready */
		while (1) {
			if (sprd_get_sdiohal_status() == 1) {
				flag_cali = true;
				pr_info("wait marlin ready ok\n");
				break;
			}
			msleep(50);
		}
	} else if (strncmp(download_dev->write_buffer,
			   "end_calibration", 15) == 0)
		flag_cali = false;
	else
		sprd_download_sdio_write(download_dev->write_buffer, count);

	sprd_download_unlock(&download_dev->write_excl);

	return count;
}

static int sprd_download_release(struct inode *inode, struct file *filp)
{
	pr_info("%s\n", __func__);
	sprd_sdiodev_readchn_uninit(DOWNLOAD_CHANNEL_READ);
	wake_unlock(&download_wake_lock);
	download_dev->open_count--;
	sprd_set_download_fin(1);

	return 0;
}

static const struct file_operations sprd_download_fops = {
	.owner = THIS_MODULE,
	.read = sprd_download_read,
	.write = sprd_download_write,
	.open = sprd_download_open,
	.release = sprd_download_release,
};

static struct miscdevice sprd_download_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DLOADER_NAME,
	.fops = &sprd_download_fops,
};

static int __init sprd_download_init(void)
{
	struct dloader_dev *dev;
	int err = 0;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->open_count = 0;
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);
	download_dev = dev;
	wake_lock_init(&download_wake_lock,
		       WAKE_LOCK_SUSPEND, "download_wake_lock");
	err = misc_register(&sprd_download_device);

	if (err) {
		pr_err("download dev add failed!!!\n");
		kfree(dev);
	}

	download_dev->read_buffer = kzalloc(READ_BUFFER_SIZE, GFP_KERNEL);
	if (download_dev->read_buffer == NULL)
		return -ENOMEM;

	download_dev->write_buffer = kzalloc(WRITE_BUFFER_SIZE + 4, GFP_KERNEL);
	if (download_dev->write_buffer == NULL) {
		kfree(download_dev->read_buffer);
		download_dev->read_buffer = NULL;
		pr_err("Download open fail(NO MEM)\n");
		return -ENOMEM;
	}

	return err;
}

static void __exit sprd_download_cleanup(void)
{
	kfree(download_dev->read_buffer);
	kfree(download_dev->write_buffer);
	download_dev->read_buffer = NULL;
	download_dev->write_buffer = NULL;
	misc_deregister(&sprd_download_device);
	kfree(download_dev);
	wake_lock_destroy(&download_wake_lock);
	download_dev = NULL;
}

module_init(sprd_download_init);
module_exit(sprd_download_cleanup);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sprd download img driver");
