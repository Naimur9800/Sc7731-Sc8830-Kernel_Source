/*
 * Copyright (C) 2012-2022 ChipOne Technology (Beijing) Co., Ltd.
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <net/sock.h>
#include <linux/compat.h>
#include <linux/notifier.h>
#include "fpsensor_spi.h"
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#endif



/**************************debug******************************/
#define MODNAME "fpCoreDriver [fpsensor] "
#undef pr_fmt
#define pr_fmt(fmt)  MODNAME"%s %d:" fmt, __func__, __LINE__

/*platform device name*/
#define FPSENSOR_DEV_NAME       "fpsensor"
/*device name after register in charater*/
#define FPSENSOR_CLASS_NAME     "fpsensor"
#define FPSENSOR_MAJOR          255
#define N_SPI_MINORS            32    /* ... up to 256 */
static DECLARE_BITMAP(minors, N_SPI_MINORS);
#define FPSENSOR_SPI_VERSION "fpsensor_spi_tee_v0.11"
#define FPSENSOR_INPUT_NAME  "fpsensor_keys"

/**************************feature control******************************/
#define FPSENSOR_IOCTL      1

/***********************input *************************/
#ifndef FPSENSOR_INPUT_HOME_KEY
/* on MTK EVB board, home key has been redefine to KEY_HOMEPAGE! */
/* double check the define on customer board!!! */
#define FPSENSOR_INPUT_HOME_KEY     KEY_HOMEPAGE /* KEY_HOME */
#define FPSENSOR_INPUT_MENU_KEY     KEY_MENU
#define FPSENSOR_INPUT_BACK_KEY     KEY_BACK
#define FPSENSOR_INPUT_FF_KEY       KEY_POWER
#define FPSENSOR_INPUT_CAMERA_KEY   KEY_CAMERA
#define FPSENSOR_INPUT_OTHER_KEY    KEY_VOLUMEDOWN  /* temporary key value for capture use */
#endif

#define FPSENSOR_NAV_UP_KEY     19  /*KEY_UP*/
#define FPSENSOR_NAV_DOWN_KEY   20  /*KEY_DOWN*/
#define FPSENSOR_NAV_LEFT_KEY   21  /*KEY_LEFT*/
#define FPSENSOR_NAV_RIGHT_KEY  22  /*KEY_RIGHT*/
#define FPSENSOR_NAV_TAP_KEY    23

/*************************************************************/
/* global variables                         */
fpsensor_data_t *g_fpsensor;
EXPORT_SYMBOL(g_fpsensor);

static unsigned bufsiz = (150 * 150);
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");



/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration                                  */
/* -------------------------------------------------------------------- */
static void fpsensor_gpio_free(fpsensor_data_t *fpsensor_dev)
{
	struct device *dev = &fpsensor_dev->platform_device->dev;
	if (g_fpsensor->irq_gpio != 0) {
		devm_gpio_free(dev, g_fpsensor->irq_gpio);
	}
	if (g_fpsensor->reset_gpio != 0) {
		devm_gpio_free(dev, g_fpsensor->reset_gpio);
	}
}

static void fpsensor_irq_gpio_cfg(void)
{
	int error;

	error = gpio_direction_input(g_fpsensor->irq_gpio);

	if (error) {
		pr_err("setup fp error[%d]\n", error);
		return ;
	}

	g_fpsensor->irq = gpio_to_irq(g_fpsensor->irq_gpio);
	pr_debug("fp interrupt irq number[%d]\n", g_fpsensor->irq);

	if (g_fpsensor->irq < 0) {
		pr_err("fp interrupt gpio to irq failed!\n");
		return ;
	}

	return;
}

static int fpsensor_request_named_gpio(fpsensor_data_t *fpsensor_dev, const char *label, int *gpio)
{
	struct device *dev = &fpsensor_dev->platform_device->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);
	if (rc < 0)  {
		pr_err("failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		pr_err("failed to request gpio %d\n", *gpio);
		return rc;
	}
	pr_err("%s %d\n", label, *gpio);
	return 0;
}

/* delay ms after reset */
static void fpsensor_hw_reset(int delay)
{
	pr_debug("entry\n");
	gpio_set_value(g_fpsensor->reset_gpio, 1);

	udelay(100);
	gpio_set_value(g_fpsensor->reset_gpio, 0);

	udelay(1000);
	gpio_set_value(g_fpsensor->reset_gpio, 1);

	if (delay) {
		udelay(delay);/* delay is configurable */
	}
	pr_debug("exit\n");
	return;
}

static int fpsensor_get_gpio_dts_info(void)
{
	int rc = 0;

	pr_debug("entry\n");
	/*get reset resource*/
	rc = fpsensor_request_named_gpio(g_fpsensor, "fpint-gpios", &g_fpsensor->irq_gpio);
	if (rc) {
		pr_err("Failed to request irq GPIO. rc = %d\n", rc);
		return -1;
	}

	rc = fpsensor_request_named_gpio(g_fpsensor, "fpreset-gpios", &g_fpsensor->reset_gpio);
	if (rc) {
		pr_err("Failed to request reset GPIO. rc = %d\n", rc);
		return -1;
	}
	gpio_direction_output(g_fpsensor->reset_gpio, 1);

	return rc;
}

static void fpsensor_spi_clk_enable(u8 bonoff)
{

}

static void fpsensor_hw_power_enable(u8 onoff)
{

}

static void fpsensor_enable_irq(fpsensor_data_t *fpsensor_dev)
{
	pr_debug("entry\n");
	setRcvIRQ(0);
	/* Request that the interrupt should be wakeable */
	if (fpsensor_dev->irq_count == 0) {
		enable_irq(fpsensor_dev->irq);
		fpsensor_dev->irq_count = 1;
	}
	pr_debug("exit\n");
	return;
}

static void fpsensor_disable_irq(fpsensor_data_t *fpsensor_dev)
{
	pr_debug("entry\n");

	if (0 == fpsensor_dev->device_available) {
		pr_err("devices not available\n");
	} else {
		if (0 == fpsensor_dev->irq_count) {
			pr_err("irq already disabled\n");
		} else {
			disable_irq_nosync(fpsensor_dev->irq);
			fpsensor_dev->irq_count = 0;
			pr_debug("disable interrupt!\n");
		}
	}
	setRcvIRQ(0);
	pr_debug("exit\n");
	return;
}

/* -------------------------------------------------------------------- */
/* file operation function                                              */
/* -------------------------------------------------------------------- */
static ssize_t fpsensor_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	pr_err("Not support read opertion in TEE version\n");
	return -EFAULT;
}

static ssize_t fpsensor_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	pr_err("Not support write opertion in TEE version\n");
	return -EFAULT;
}

static irqreturn_t fpsensor_irq(int irq, void *handle)
{
	fpsensor_data_t *fpsensor_dev = (fpsensor_data_t *)handle;


	/* Make sure 'wakeup_enabled' is updated before using it
	** since this is interrupt context (other thread...) */
	smp_rmb();
	wake_lock_timeout(&fpsensor_dev->ttw_wl, msecs_to_jiffies(1000));


#if FPSENSOR_IOCTL
	setRcvIRQ(1);
#endif
	wake_up_interruptible(&fpsensor_dev->wq_irq_return);
	fpsensor_dev->sig_count++;

	return IRQ_HANDLED;
}

void setRcvIRQ(int  val)
{
	fpsensor_data_t *fpsensor_dev = g_fpsensor;
	fpsensor_dev->RcvIRQ = val;
}

static int fpsensor_input_init(fpsensor_data_t *fpsensor_dev)
{
	int status = 0;

	pr_debug("entry\n");
	if (!fpsensor_dev) {
		pr_err("fpsensor device is nullptr.\n");
		status = -EINVAL;
		goto bye;
	}

	fpsensor_dev->input = input_allocate_device();
	if (fpsensor_dev->input == NULL) {
		pr_err("Failed to allocate input device.\n");
		status = -ENOMEM;
		goto bye;
	}
	__set_bit(EV_KEY, fpsensor_dev->input->evbit);
	__set_bit(FPSENSOR_INPUT_HOME_KEY, fpsensor_dev->input->keybit);

	__set_bit(FPSENSOR_INPUT_MENU_KEY, fpsensor_dev->input->keybit);
	__set_bit(FPSENSOR_INPUT_BACK_KEY, fpsensor_dev->input->keybit);
	__set_bit(FPSENSOR_INPUT_FF_KEY, fpsensor_dev->input->keybit);

	__set_bit(FPSENSOR_NAV_TAP_KEY, fpsensor_dev->input->keybit);
	__set_bit(FPSENSOR_NAV_UP_KEY, fpsensor_dev->input->keybit);
	__set_bit(FPSENSOR_NAV_DOWN_KEY, fpsensor_dev->input->keybit);
	__set_bit(FPSENSOR_NAV_RIGHT_KEY, fpsensor_dev->input->keybit);
	__set_bit(FPSENSOR_NAV_LEFT_KEY, fpsensor_dev->input->keybit);
	__set_bit(FPSENSOR_INPUT_CAMERA_KEY, fpsensor_dev->input->keybit);
	fpsensor_dev->input->name = FPSENSOR_INPUT_NAME;
	if (input_register_device(fpsensor_dev->input)) {
		pr_err("Failed to register input device.\n");
		status = -ENODEV;
		goto err1;
	}
	goto bye;

err1:
	input_free_device(fpsensor_dev->input);
	fpsensor_dev->input = NULL;

bye:
	pr_debug("exit\n");
	return status;
}

static long fpsensor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	fpsensor_data_t *fpsensor_dev = NULL;
	struct fpsensor_key fpsensor_key;
	uint32_t key_event;
	int retval = 0;
	unsigned int val = 0;
	int irqf;

	pr_debug("[rickon]: fpsensor ioctl cmd : 0x%x\n", cmd);
	fpsensor_dev = (fpsensor_data_t *)filp->private_data;
	/*clear cancel flag*/
	fpsensor_dev->cancel = 0 ;
	switch (cmd) {
	case FPSENSOR_IOC_INIT:
		pr_debug("%s: fpsensor init started======\n", __func__);
		retval = fpsensor_get_gpio_dts_info();
		if (retval) {
			break;
		}
		fpsensor_irq_gpio_cfg();
		/*regist irq*/
		irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;

		retval = devm_request_threaded_irq(&g_fpsensor->platform_device->dev, g_fpsensor->irq, NULL,
			fpsensor_irq, irqf, dev_name(&g_fpsensor->platform_device->dev), g_fpsensor);
		if (retval == 0)
			pr_debug("irq thread reqquest success\n");
		else
			pr_debug("irq thread retval =%d\n", retval);
		enable_irq_wake(g_fpsensor->irq);
		fpsensor_dev->device_available = 1;
		fpsensor_dev->irq_count = 0;
		/*sunbo: add to avoid "unbalanced enable for IRQ 419" warning - begin*/
		fpsensor_dev->irq_count = 1;
		fpsensor_disable_irq(fpsensor_dev);
		/*sunbo: add to avoid "unbalanced enable for IRQ 419" warning - end*/

		fpsensor_dev->sig_count = 0;

		pr_debug("%s: fpsensor init finished======\n", __func__);
		break;

	case FPSENSOR_IOC_EXIT:
		fpsensor_disable_irq(fpsensor_dev);
		if (fpsensor_dev->irq) {
			free_irq(fpsensor_dev->irq, fpsensor_dev);
			fpsensor_dev->irq_count = 0;
		}
		fpsensor_dev->device_available = 0;
		fpsensor_gpio_free(g_fpsensor);
		pr_debug("fpsensor exit finished======\n");
		break;

	case FPSENSOR_IOC_RESET:
		pr_debug("chip reset command\n");
		fpsensor_hw_reset(1250);
		break;

	case FPSENSOR_IOC_ENABLE_IRQ:
		pr_debug("chip ENable IRQ command\n");
		fpsensor_enable_irq(fpsensor_dev);
		break;

	case FPSENSOR_IOC_DISABLE_IRQ:
		pr_debug("chip disable IRQ command\n");
		fpsensor_disable_irq(fpsensor_dev);
		break;
	case FPSENSOR_IOC_GET_INT_VAL:
		val = gpio_get_value(fpsensor_dev->irq_gpio);
		if (copy_to_user((void __user *)arg, (void *)&val, sizeof(unsigned int))) {
			pr_err("Failed to copy data to user\n");
			retval = -EFAULT;
			break;
		}
		retval = 0;
		break;
	case FPSENSOR_IOC_ENABLE_SPI_CLK:
		pr_debug("ENABLE_SPI_CLK ======\n");
		fpsensor_spi_clk_enable(1);
		break;
	case FPSENSOR_IOC_DISABLE_SPI_CLK:
		pr_debug("DISABLE_SPI_CLK ======\n");
		fpsensor_spi_clk_enable(0);
		break;
	case FPSENSOR_IOC_ENABLE_POWER:
		pr_debug("FPSENSOR_IOC_ENABLE_POWER ======\n");
		fpsensor_hw_power_enable(1);
		break;
	case FPSENSOR_IOC_DISABLE_POWER:
		pr_debug("FPSENSOR_IOC_DISABLE_POWER ======\n");
		fpsensor_hw_power_enable(0);
		break;
	case FPSENSOR_IOC_INIT_INPUT_DEV:
		pr_info("FPSENSOR_IOC_INIT_INPUT_DEV ======\n");
		retval = fpsensor_input_init(fpsensor_dev);
		if (retval) {
			pr_err("fpsensor input init fail\n");
		}
		break;
	case FPSENSOR_IOC_INPUT_KEY_EVENT:
		if (copy_from_user(&fpsensor_key, (struct fpsensor_key *)arg, sizeof(struct fpsensor_key))) {
			pr_err("Failed to copy input key event from user to kernel\n");
			retval = -EFAULT;
			break;
		}
		if (FPSENSOR_KEY_HOME == fpsensor_key.key) {
			key_event = FPSENSOR_INPUT_HOME_KEY;
		} else if (FPSENSOR_KEY_POWER == fpsensor_key.key) {
			key_event = FPSENSOR_INPUT_FF_KEY;
		} else if (FPSENSOR_KEY_CAPTURE == fpsensor_key.key) {
			key_event = FPSENSOR_INPUT_CAMERA_KEY;
		} else {
			/* add special key define */
			key_event = FPSENSOR_INPUT_OTHER_KEY;
		}
		pr_debug("received key event[%d], key=%d, value=%d\n",
			key_event, fpsensor_key.key, fpsensor_key.value);
		if ((FPSENSOR_KEY_POWER == fpsensor_key.key || FPSENSOR_KEY_CAPTURE == fpsensor_key.key)
		&& (fpsensor_key.value == 1)) {
			input_report_key(fpsensor_dev->input, key_event, 1);
			input_sync(fpsensor_dev->input);
			input_report_key(fpsensor_dev->input, key_event, 0);
			input_sync(fpsensor_dev->input);
		} else if (FPSENSOR_KEY_UP == fpsensor_key.key) {
			input_report_key(fpsensor_dev->input, FPSENSOR_NAV_UP_KEY, 1);
			input_sync(fpsensor_dev->input);
			input_report_key(fpsensor_dev->input, FPSENSOR_NAV_UP_KEY, 0);
			input_sync(fpsensor_dev->input);
		} else if (FPSENSOR_KEY_DOWN == fpsensor_key.key) {
			input_report_key(fpsensor_dev->input, FPSENSOR_NAV_DOWN_KEY, 1);
			input_sync(fpsensor_dev->input);
			input_report_key(fpsensor_dev->input, FPSENSOR_NAV_DOWN_KEY, 0);
			input_sync(fpsensor_dev->input);
		} else if (FPSENSOR_KEY_RIGHT == fpsensor_key.key) {
			input_report_key(fpsensor_dev->input, FPSENSOR_NAV_RIGHT_KEY, 1);
			input_sync(fpsensor_dev->input);
			input_report_key(fpsensor_dev->input, FPSENSOR_NAV_RIGHT_KEY, 0);
			input_sync(fpsensor_dev->input);
		} else if (FPSENSOR_KEY_LEFT == fpsensor_key.key) {
			input_report_key(fpsensor_dev->input, FPSENSOR_NAV_LEFT_KEY, 1);
			input_sync(fpsensor_dev->input);
			input_report_key(fpsensor_dev->input, FPSENSOR_NAV_LEFT_KEY, 0);
			input_sync(fpsensor_dev->input);
		} else  if (FPSENSOR_KEY_TAP == fpsensor_key.key) {
			input_report_key(fpsensor_dev->input, FPSENSOR_NAV_TAP_KEY, 1);
			input_sync(fpsensor_dev->input);
			input_report_key(fpsensor_dev->input, FPSENSOR_NAV_TAP_KEY, 0);
			input_sync(fpsensor_dev->input);
		} else if ((FPSENSOR_KEY_POWER != fpsensor_key.key) && (FPSENSOR_KEY_CAPTURE != fpsensor_key.key)) {
			input_report_key(fpsensor_dev->input, key_event, fpsensor_key.value);
			input_sync(fpsensor_dev->input);
		}
		break;

	case FPSENSOR_IOC_ENTER_SLEEP_MODE:
		fpsensor_dev->is_sleep_mode = 1;
		break;
	case FPSENSOR_IOC_REMOVE:

		if (fpsensor_dev->input != NULL) {
			pr_debug("%s: input_unregister_device\n", __func__);
			input_unregister_device(fpsensor_dev->input);
		}
		device_destroy(fpsensor_dev->class, fpsensor_dev->devno);
		unregister_chrdev_region(fpsensor_dev->devno, 1);
		class_destroy(fpsensor_dev->class);


		pr_info("remove finished\n");
		break;

	case FPSENSOR_IOC_CANCEL_WAIT:
		pr_debug("FPSENSOR CANCEL WAIT\n");
		wake_up_interruptible(&fpsensor_dev->wq_irq_return);
		fpsensor_dev->cancel = 1;
		break;
	default:
		pr_err("fpsensor doesn't support this command(%d)\n", cmd);
		break;
	}

	return retval;
}

static long fpsensor_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return fpsensor_ioctl(filp, cmd, (unsigned long)(arg));
}

static unsigned int fpsensor_poll(struct file *filp, struct poll_table_struct *wait)
{
	unsigned int ret = 0;
	pr_info(" support poll opertion  in   version\n");
	ret |= POLLIN;
	poll_wait(filp, &g_fpsensor->wq_irq_return, wait);
	if (g_fpsensor->cancel == 1) {
		pr_err(" cancle\n");
		ret =  POLLERR;
		g_fpsensor->cancel = 0;
		return ret;
	}
	if (g_fpsensor->RcvIRQ) {
		pr_err(" ******get irq\n");
		ret |= POLLRDNORM;
	} else {
		ret = 0;
	}
	return ret;
}

/* -------------------------------------------------------------------- */
/* device function                                  */
/* -------------------------------------------------------------------- */
static int fpsensor_open(struct inode *inode, struct file *filp)
{
	fpsensor_data_t *fpsensor_dev;

	pr_debug("entry\n");
	fpsensor_dev = container_of(inode->i_cdev, fpsensor_data_t, cdev);
	fpsensor_dev->users++;
	fpsensor_dev->device_available = 1;
	filp->private_data = fpsensor_dev;
	pr_debug("exit\n");
	return 0;
}

static int fpsensor_release(struct inode *inode, struct file *filp)
{
	fpsensor_data_t *fpsensor_dev;
	int    status = 0;

	pr_debug("entry\n");
	fpsensor_dev = filp->private_data;
	filp->private_data = NULL;

	/*last close??*/
	fpsensor_dev->users--;
	if (!fpsensor_dev->users) {
		pr_debug("disble_irq. irq = %d\n", fpsensor_dev->irq);
		fpsensor_disable_irq(fpsensor_dev);
	}

	fpsensor_dev->device_available = 0;
	pr_debug("exit\n");
	return status;
}

static const struct file_operations fpsensor_fops = {
	.owner          = THIS_MODULE,
	.write          = fpsensor_write,
	.read           = fpsensor_read,
	.unlocked_ioctl = fpsensor_ioctl,
	.compat_ioctl   = fpsensor_compat_ioctl,
	.open           = fpsensor_open,
	.release        = fpsensor_release,
	.poll           = fpsensor_poll,

};

/* -------------------------------------------------------------------- */
static int fpsensor_create_class(fpsensor_data_t *fpsensor)
{
	int error = 0;
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	error = register_chrdev(FPSENSOR_MAJOR, FPSENSOR_DEV_NAME, &fpsensor_fops);
	fpsensor->class = class_create(THIS_MODULE, FPSENSOR_CLASS_NAME);
	if (IS_ERR(fpsensor->class)) {
		pr_err("Failed to create class.\n");
		error = PTR_ERR(fpsensor->class);
	}

	return error;
}

/* -------------------------------------------------------------------- */
static int fpsensor_create_device(fpsensor_data_t *fpsensor)
{
	int error = 0;
	unsigned long minor;
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		fpsensor->devno = MKDEV(FPSENSOR_MAJOR, minor);
		dev = device_create(fpsensor->class, &fpsensor->platform_device->dev,  fpsensor->devno,
			fpsensor, FPSENSOR_DEV_NAME);
		error = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		pr_err("no minor number available!\n");
		error = -ENODEV;
	}
	set_bit(minor, minors);
	return error;
}

/*-------------------------------------------------------------------------*/
static int fpsensor_probe(struct platform_device *pdev)
{
	fpsensor_data_t *fpsensor_dev = NULL;
	int error = 0;
	int status = -EINVAL;

	pr_debug("entry\n");
	/* Allocate driver data */
	fpsensor_dev = kzalloc(sizeof(*fpsensor_dev), GFP_KERNEL);
	if (!fpsensor_dev) {
		pr_err("Failed to alloc memory for fpsensor device.\n");
		pr_debug("exit\n");
		return -ENOMEM;
	}
	fpsensor_dev->platform_device = pdev ;

	g_fpsensor = fpsensor_dev;
	/* Initialize the driver data */
	mutex_init(&fpsensor_dev->buf_lock);

	fpsensor_dev->device_available = 0;
	fpsensor_dev->irq = 0;
	fpsensor_dev->probe_finish = 0;
	fpsensor_dev->device_count = 0;
	fpsensor_dev->users = 0;
	fpsensor_dev->input = NULL;
	/*setup fpsensor configurations.*/
	pr_debug("Setting fpsensor device configuration.\n");
	error = fpsensor_create_class(fpsensor_dev);
	if (error)
		goto err2;

	error = fpsensor_create_device(fpsensor_dev);
	if (error)
		goto err2;

	cdev_init(&fpsensor_dev->cdev, &fpsensor_fops);
	fpsensor_dev->cdev.owner = THIS_MODULE;
	error = cdev_add(&fpsensor_dev->cdev, fpsensor_dev->devno, 1);
	if (error)
		goto err2;

	fpsensor_dev->device_available = 1;
	fpsensor_dev->irq_count = 0;
	fpsensor_dev->sig_count = 0;
	pr_debug("fpsensor init finished======\n");

	fpsensor_dev->probe_finish = 1;
	fpsensor_dev->is_sleep_mode = 0;
	fpsensor_spi_clk_enable(1);

	init_waitqueue_head(&fpsensor_dev->wq_irq_return);
	wake_lock_init(&g_fpsensor->ttw_wl, WAKE_LOCK_SUSPEND, "fpsensor_ttw_wl");
	pr_debug("probe finished, normal driver version: %s\n",
		FPSENSOR_SPI_VERSION);

	fpsensor_hw_power_enable(1);
	udelay(1000);
	pr_debug("exit\n");
	return 0;

	err2:
	device_destroy(fpsensor_dev->class, fpsensor_dev->devno);
	fpsensor_hw_power_enable(0);
	fpsensor_spi_clk_enable(0);
	clear_bit(MINOR(fpsensor_dev->devno), minors);

	kfree(fpsensor_dev);
	pr_debug("exit\n");
	return status;
}

static int fpsensor_remove(struct platform_device *pdev)
{
	fpsensor_data_t *fpsensor_dev = g_fpsensor;
	pr_debug("entry\n");

	/* make sure ops on existing fds can abort cleanly */
	if (fpsensor_dev->irq) {
		free_irq(fpsensor_dev->irq, fpsensor_dev);
	}
	device_destroy(fpsensor_dev->class, fpsensor_dev->devno);
	unregister_chrdev_region(fpsensor_dev->devno, 1);
	class_destroy(fpsensor_dev->class);
	if (fpsensor_dev->users == 0) {
		if (fpsensor_dev->input != NULL)
			input_unregister_device(fpsensor_dev->input);

		if (fpsensor_dev->buffer != NULL)
			kfree(fpsensor_dev->buffer);
	}
	clear_bit(MINOR(fpsensor_dev->devno), minors);
	fpsensor_hw_power_enable(0);
	wake_lock_destroy(&fpsensor_dev->ttw_wl);

	pr_debug("remove finished\n");
	kfree(fpsensor_dev);
	pr_debug("exit\n");
	return 0;
}

#ifdef CONFIG_PM
static int fpsensor_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int fpsensor_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

/*-------------------------------------------------------------------------*/
static struct of_device_id fpsensor_of_match[] = {
	{ .compatible = "chipone,fingerprint", },
	{}
};
MODULE_DEVICE_TABLE(of, fpsensor_of_match);
static struct platform_driver fpsensor_driver = {
	.driver = {
		.name = FPSENSOR_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = fpsensor_of_match,
	},
	.probe = fpsensor_probe,
	.remove = fpsensor_remove,
#ifdef CONFIG_PM
	.suspend = fpsensor_suspend,
	.resume = fpsensor_resume,
#endif
};
module_platform_driver(fpsensor_driver);

MODULE_AUTHOR("xhli");
MODULE_DESCRIPTION(" Fingerprint chipone TEE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:fpsensor-drivers");

