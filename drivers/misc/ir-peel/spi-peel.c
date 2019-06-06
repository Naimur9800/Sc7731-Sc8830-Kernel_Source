/*
 * Peel's SPI driver
 *
 * Copyright (C) 2014  Peel Technologies Inc
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

#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include "spi-peel.h"

struct photon_data {
	u8 *buffer;
	int is_open;
	unsigned int bufsiz;
	struct spi_device *spi;
	struct strIds id;
	struct miscdevice photon_device;
	int ldo_en_gpio;
	spinlock_t spi_lock;
};

static struct spi_device *spi_g;

static ssize_t
photon_spi_sync(struct photon_data *photon, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;
	struct spi_device *spi;

	spin_lock_irq(&photon->spi_lock);
	spi = photon->spi;
	spin_unlock_irq(&photon->spi_lock);

	if (spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_sync(spi, message);

	if (status == 0)
		status = message->actual_length;

	return status;
}

static inline ssize_t
photon_sync_read(struct photon_data *photon, struct spi_ioc_transfer *u_xfers)
{
	struct spi_transfer t = {
		.tx_buf = photon->buffer,
		.rx_buf = photon->buffer,
		.len = u_xfers->len,
		.bits_per_word = u_xfers->bits_per_word,
		.speed_hz = u_xfers->speed_hz,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return photon_spi_sync(photon, &m);
}

static ssize_t
photon_read(struct photon_data *photon,
		struct spi_ioc_transfer *u_xfers)
{
	ssize_t status = 0;

	memset(photon->buffer, 0, photon->bufsiz);

	if (u_xfers->len > photon->bufsiz) {
		pr_err("peel: Requested too large data\n");
		return -EMSGSIZE;
	}

	status = photon_sync_read(photon, u_xfers);
	if (status < 0) {
		pr_err("peel: photon_sync_read() failed\n");
		return status;
		}

	/* copy rx data to user space */
	if (u_xfers->rx_buf && __copy_to_user(
		(u8 __user *)(uintptr_t) u_xfers->rx_buf,
		photon->buffer, u_xfers->len)) {
		pr_err("peel: copy_to_user() failed\n");
		status =  -EFAULT;
	}

	return status;
}

static inline ssize_t
photon_sync_write(struct photon_data *photon, struct spi_ioc_transfer *u_xfers)
{
	struct spi_transfer t = {
		.tx_buf = photon->buffer,
		.len = u_xfers->len,
		.bits_per_word = u_xfers->bits_per_word,
	.speed_hz = u_xfers->speed_hz,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return photon_spi_sync(photon, &m);
}

static ssize_t
photon_write(struct photon_data *photon,
		struct spi_ioc_transfer *u_xfers)
{
	ssize_t status;

	if (u_xfers->len > photon->bufsiz) {
		pr_err("peel: Requested too large data\n");
		return -EMSGSIZE;
	}

	if (copy_from_user(photon->buffer, (const u8 __user *)
		(uintptr_t) u_xfers->tx_buf, u_xfers->len)) {
		pr_err("peel: copy_from_user() failed\n");
		return -EFAULT;
	}

	status = photon_sync_write(photon, u_xfers);
	if (status < 0) {
		pr_err("peel: photon_sync_write() failed\n");
	}

	return status;
}

static long
photon_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int status = 0;
	struct photon_data *photon;
	struct spi_ioc_transfer ioc;

	photon = filp->private_data;

	switch (cmd) {
	case SPI_IOC_WR_MSG:
		if (__copy_from_user(&ioc, (void __user *)arg,
			sizeof(struct spi_ioc_transfer))) {
			pr_err("peel: copy_from_user failed\n");
			status = -EFAULT;
			break;
		}
		status = photon_write(photon, &ioc);
		break;

	case SPI_IOC_RD_MSG:
		if (__copy_from_user(&ioc, (void __user *)arg,
			sizeof(struct spi_ioc_transfer))) {
			pr_err("peel: copy_from_user failed\n");
			status = -EFAULT;
			break;
		}
		status = photon_read(photon, &ioc);
		break;

	case SPI_IOC_RD_IDS:
		if (__copy_to_user((void __user *)arg, &photon->id,
			sizeof(struct strIds))) {
			pr_err("peel: copy_to_user failed\n");
			status = -EFAULT;
		}
		break;

	default:
		status = -EINVAL;
		break;
	}

	return status;
}

static int photon_open(struct inode *inode, struct file *filp)
{
	struct photon_data *photon = spi_get_drvdata(spi_g);

	if (photon == NULL) {
		pr_err("peel: photon is null\n");
		return -EFAULT;
	}

	if (photon->is_open) {
		pr_err("peel: Device in use\n");
		return -EBUSY;
	}

	gpio_set_value(photon->ldo_en_gpio, 1);
	pr_err("open photon->ldo_en_gpio:%d\n", photon->ldo_en_gpio);
	photon->is_open = 1;
	filp->private_data = photon;
	nonseekable_open(inode, filp);

	return 0;
}

static int photon_close(struct inode *inode, struct file *filp)
{
	struct photon_data *photon = filp->private_data;

	pr_err("close photon->ldo_en_gpio:%d\n", photon->ldo_en_gpio);
	gpio_set_value(photon->ldo_en_gpio, 0);
	photon->is_open = 0;

	return 0;
}

static const struct file_operations photon_dev_fops = {
	.owner		= THIS_MODULE,
	.open		= photon_open,
	.release	= photon_close,
	.unlocked_ioctl	= photon_ioctl,
	.compat_ioctl	= photon_ioctl,
};

static int photon_probe(struct spi_device *spi)
{
	int status = 0;
	unsigned int npages;
	struct photon_data *photon;
	struct device_node *np = spi->dev.of_node;

	photon = kzalloc(sizeof(struct photon_data), GFP_KERNEL|GFP_ATOMIC);
	if (photon == NULL) {
		return -ENOMEM;
	}

	spin_lock_init(&photon->spi_lock);
	spi_g = spi;
	photon->spi = spi;
	photon->is_open = 0;

	status |= of_property_read_u8(np, "photon,spi-bpw",
				&spi->bits_per_word);
	status |= of_property_read_u16(np, "photon,spi-mode", &spi->mode);
	status |= of_property_read_u32(np, "photon,spi-clk-speed",
				&spi->max_speed_hz);
	status |= of_property_read_u32(np, "photon,id1", &photon->id.u32ID1);
	status |= of_property_read_u32(np, "photon,id2", &photon->id.u32ID2);
	status |= of_property_read_u32(np, "photon,id3", &photon->id.u32ID3);
	status |= of_property_read_u32(np, "photon,npages", &npages);

	if (status) {
		pr_err("peel: some properties are missing in dts tree\n");
		status = -EFAULT;
		goto free_photon;
	}

	status = spi_setup(spi);
	if (status < 0) {
		pr_err("peel: fail to setup spi bpw, mode");
		status = -EFAULT;
		goto free_photon;
	}

	photon->ldo_en_gpio = of_get_named_gpio(np, "chip-en-gpios", 0);

	if (photon->ldo_en_gpio < 0) {
		pr_err("peel: get gpio failed\n");
		status = -EFAULT;
		goto free_photon;
	}

	gpio_request(photon->ldo_en_gpio, "ir_ldo_en");
	gpio_direction_output(photon->ldo_en_gpio, 1);
	gpio_set_value(photon->ldo_en_gpio, 0);

	photon->bufsiz = npages * PAGE_SIZE;
	photon->buffer = kzalloc(photon->bufsiz, GFP_KERNEL|GFP_ATOMIC);
	if (photon->buffer == NULL) {
		pr_err("peel: %s: can't allocate memory\n", __func__);
		status = -ENOMEM;
		goto free_photon;
	}

	photon->photon_device.name = "peel_ir";
	photon->photon_device.fops = &photon_dev_fops;
	photon->photon_device.minor = MISC_DYNAMIC_MINOR;

	spi_set_drvdata(spi, photon);
	status = misc_register(&photon->photon_device);
	if (status < 0) {
		pr_err("peel: misc_register failed\n");
		goto free_photon_buffer;
	}

	pr_info("peel: Peel Driver Initialized Successfully\n");
	return 0;

free_photon_buffer:
	kfree(photon->buffer);
free_photon:
	kfree(photon);

	return status;
}

static int photon_remove(struct spi_device *spi)
{
	struct photon_data *photon = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	if (photon) {
		spin_lock_irq(&photon->spi_lock);
		photon->spi = NULL;
		spi_set_drvdata(spi, NULL);
		gpio_free(photon->ldo_en_gpio);
		if (photon->buffer) {
			kfree(photon->buffer);
		}
		spin_unlock_irq(&photon->spi_lock);
		misc_deregister(&photon->photon_device);
		kfree(photon);
	}

	return 0;
}

static const struct of_device_id photon_device_ids[] = {
	{ .compatible = "peel,photon" },
};
MODULE_DEVICE_TABLE(of, photon_device_ids);

static struct spi_driver photon_driver = {
	.driver = {
		.name           = "peel_ir",
		.owner          = THIS_MODULE,
		.of_match_table = photon_device_ids,
	},
	.probe  = photon_probe,
	.remove = photon_remove,
};

static int __init photon_init(void)
{
	int status;

	status = spi_register_driver(&photon_driver);

	if (status < 0)
		pr_err("peel: %s: Error registering peel driver\n", __func__);

	return status;
}
module_init(photon_init);

static void __exit photon_exit(void)
{
	spi_unregister_driver(&photon_driver);
}
module_exit(photon_exit);

MODULE_DESCRIPTION("Peel's SPI driver");
MODULE_LICENSE("GPL");
