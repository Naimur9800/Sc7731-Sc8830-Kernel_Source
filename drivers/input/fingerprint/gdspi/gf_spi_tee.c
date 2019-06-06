/*
 * Copyright (C) 2013-2016, Shenzhen Huiding Technology Co., Ltd.
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/compat.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/platform_data/spi-s3c64xx.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <video/adf_notifier.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#include <net/sock.h>

#include "gf_spi_tee.h"


/**************************defination******************************/
#define GF_DEV_NAME "goodix_fp"
#define GF_DEV_MAJOR 0	/* assigned */

#define GF_CLASS_NAME "goodix_fp"
#define GF_INPUT_NAME "gf-keys"

#define GF_LINUX_VERSION "V1.01.03"

#define GF_NETLINK_ROUTE 25   /* GF test,need definein /uapi/linux/netlink.h */
#define MAX_NL_MSG_LEN 16

#ifndef GF_INPUT_HOME_KEY
/* on MTK EVB board, home key has been redefine to KEY_HOMEPAGE! */
/* double check the define on customer board!!! */
#define GF_INPUT_HOME_KEY KEY_HOMEPAGE /* KEY_HOME */

#define GF_INPUT_MENU_KEY  KEY_MENU
#define GF_INPUT_BACK_KEY  KEY_BACK

#define GF_INPUT_FF_KEY  KEY_POWER
#define GF_INPUT_CAMERA_KEY  KEY_CAMERA


#define GF_INPUT_OTHER_KEY KEY_VOLUMEDOWN
#endif
#define GF_NAV_UP_KEY  KEY_UP
#define GF_NAV_DOWN_KEY  KEY_DOWN
#define GF_NAV_LEFT_KEY  KEY_LEFT
#define GF_NAV_RIGHT_KEY  KEY_RIGHT

#define GOODIX_SENSOR_TYPE "GF5206"

/*************************************************************/

/* debug log setting */
static u8 g_debug_level = DEBUG_LOG;

/* align=2, 2 bytes align */
/* align=4, 4 bytes align */
/* align=8, 8 bytes align */
#define ROUND_UP(x, align)		((x+(align-1))&~(align-1))


/*************************************************************/
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#ifdef CONFIG_OF
static const struct of_device_id gf_of_match[] = {
	{ .compatible = "goodix,goodix-fp", },
	{},
};
MODULE_DEVICE_TABLE(of, gf_of_match);
#endif

/* for netlink use */
static int pid;

static u8 g_vendor_id;

static ssize_t gf_debug_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t gf_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, gf_debug_show, gf_debug_store);

static struct attribute *gf_debug_attrs[] = {
	&dev_attr_debug.attr,
	NULL
};

static const struct attribute_group gf_debug_attr_group = {
	.attrs = gf_debug_attrs,
	.name = "debug"
};

/* -------------------------------------------------------------------- */
/* timer function							*/
/* -------------------------------------------------------------------- */
#define TIME_START	   0
#define TIME_STOP	   1

/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration				*/
/* -------------------------------------------------------------------- */

static int gf_get_gpio_dts_info(struct gf_device *gf_dev)
{
	int ret;
	struct device_node *node = gf_dev->pdev->dev.of_node;

	gf_dev->reset_gpio = of_get_gpio(node, 0);
	if (gf_dev->reset_gpio < 0)
		dev_err(gf_dev->device,  "[ERROR] reset gpio request failed.\n");

	gf_dev->irq_gpio = of_get_gpio(node, 1);
	if (gf_dev->irq_gpio < 0)
		dev_err(gf_dev->device,  "[ERROR] cs gpio request failed.\n");

	gf_dev->en3v_gpio = of_get_gpio(node, 2);
	if (gf_dev->en3v_gpio < 0)
		dev_err(gf_dev->device,  "[ERROR] en3v gpio request failed.\n");

	gf_dev->en1v8_gpio = of_get_gpio(node, 3);
	if (gf_dev->en1v8_gpio < 0)
		dev_err(gf_dev->device,  "[ERROR] en1v8 gpio request failed.\n");

	ret = gpio_request(gf_dev->reset_gpio, "gx_reset");

	if (ret)
		pr_err("[error] get gx reset gpio fail\n");

	gpio_direction_output(gf_dev->reset_gpio, 1);

	ret = gpio_request(gf_dev->en3v_gpio, "gx_en3v");

	if (ret)
		pr_err("[error] get gx en3v gpio fail\n");

	gpio_direction_output(gf_dev->en3v_gpio, 1);

	ret = gpio_request(gf_dev->en1v8_gpio, "gx_en1v8");

	if (ret)
		pr_err("[error] get gx en1v8 gpio fail\n");

	gpio_direction_output(gf_dev->en1v8_gpio, 1);

	gf_debug(DEBUG_LOG, "%s, get pinctrl success!\n", __func__);

	return 0;
}

static void gf_irq_gpio_cfg(struct gf_device *gf_dev)
{
	int error;

	error = gpio_request(gf_dev->irq_gpio, "gx_irq");
	if (error) {
		dev_err(gf_dev->device,  "[ERROR] irq request failed.\n");
		return;
	}
	gf_debug(ERR_LOG, "%s, irq gpio pin =%d\n", __func__, gf_dev->irq_gpio);
	error = gpio_direction_input(gf_dev->irq_gpio);
	if (error)
		dev_err(gf_dev->device, "gpio_direction_input (irq) failed.\n");

	gf_dev->irq = gpio_to_irq(gf_dev->irq_gpio);
}

/* delay ms after reset */
static void gf_hw_reset(struct gf_device *gf_dev)
{
	gpio_set_value(gf_dev->reset_gpio, 0);
	msleep(20);
	gpio_set_value(gf_dev->reset_gpio, 1);
}

static void gf_enable_irq(struct gf_device *gf_dev)
{
	if (gf_dev->irq_count == 1) {
		gf_debug(ERR_LOG, "%s, irq already enabled\n", __func__);
	} else {
		enable_irq(gf_dev->irq);
		gf_dev->irq_count = 1;
		gf_debug(DEBUG_LOG, "%s enable interrupt!\n", __func__);
	}
}

static void gf_disable_irq(struct gf_device *gf_dev)
{
	if (gf_dev->irq_count == 0) {
		gf_debug(ERR_LOG, "%s, irq already disabled\n", __func__);
	} else {
		disable_irq(gf_dev->irq);
		gf_dev->irq_count = 0;
		gf_debug(DEBUG_LOG, "%s disable interrupt!\n", __func__);
	}
}


/* -------------------------------------------------------------------- */
/* netlink functions                 */
/* -------------------------------------------------------------------- */
static void gf_netlink_send(struct gf_device *gf_dev, const int command)
{
	struct nlmsghdr *nlh = NULL;
	struct sk_buff *skb = NULL;
	int ret;

	if (gf_dev->nl_sk == NULL) {
		gf_debug(ERR_LOG, "[%s] : invalid socket\n", __func__);
		return;
	}

	if (pid == 0) {
		gf_debug(ERR_LOG, "[%s] : inval nati process pid\n", __func__);
		return;
	}

	/*alloc data buffer for sending to native*/
	/*malloc data space at least 1500 bytes, which is ethernet data length*/
	skb = alloc_skb(MAX_NL_MSG_LEN, GFP_ATOMIC);
	if (skb == NULL)
		return;

	nlh = nlmsg_put(skb, 0, 0, 0, MAX_NL_MSG_LEN, 0);
	if (!nlh) {
		gf_debug(ERR_LOG, "[%s] : nlmsg_put failed\n", __func__);
		kfree_skb(skb);
		return;
	}

	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;

	*(char *)NLMSG_DATA(nlh) = command;
	ret = netlink_unicast(gf_dev->nl_sk, skb, pid, MSG_DONTWAIT);
	if (ret == 0) {
		gf_debug(ERR_LOG, "[%s] : send failed\n", __func__);
		return;
	}

}

static void gf_netlink_recv(struct sk_buff *__skb)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	char str[128];

	gf_debug(INFO_LOG, "[%s] : enter\n", __func__);

	skb = skb_get(__skb);
	if (skb == NULL) {
		gf_debug(ERR_LOG, "[%s] : skb_get return NULL\n", __func__);
		return;
	}

	/* presume there is 5byte payload at leaset */
	if (skb->len >= NLMSG_SPACE(0)) {
		nlh = nlmsg_hdr(skb);
		memcpy(str, NLMSG_DATA(nlh), sizeof(str));
		pid = nlh->nlmsg_pid;
		gf_debug(INFO_LOG, "[%s] : p: %d, m: %s\n", __func__, pid, str);

	} else {
		gf_debug(ERR_LOG, "[%s] : not enough data length\n", __func__);
	}

	kfree_skb(skb);
}

static int gf_netlink_init(struct gf_device *gf_dev)
{
	struct netlink_kernel_cfg cfg;

	memset(&cfg, 0, sizeof(struct netlink_kernel_cfg));
	cfg.input = gf_netlink_recv;

	gf_dev->nl_sk = netlink_kernel_create(&init_net,
				GF_NETLINK_ROUTE, &cfg);
	if (gf_dev->nl_sk == NULL) {
		gf_debug(ERR_LOG, "[%s] : netlink create failed\n", __func__);
		return -1;
	}

	gf_debug(INFO_LOG, "[%s] : netlink create success\n", __func__);
	return 0;
}

static int gf_netlink_destroy(struct gf_device *gf_dev)
{
	if (gf_dev->nl_sk != NULL) {
		netlink_kernel_release(gf_dev->nl_sk);
		gf_dev->nl_sk = NULL;
		return 0;
	}

	gf_debug(ERR_LOG, "[%s] : no netlink socket yet\n", __func__);
	return -1;
}

static int gf_adf_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct gf_device *gf_dev = NULL;
	struct adf_notifier_event *evdata = data;
	unsigned int blank;
	int retval = 0;

	FUNC_ENTRY();

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != ADF_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */)
		return 0;

	gf_dev = container_of(self, struct gf_device, notifier);
	blank = *(int *)evdata->data;

	gf_debug(INFO_LOG, "[%s] : enter, blank=0x%x\n", __func__, blank);

	switch (blank) {
	case DRM_MODE_DPMS_ON:
		gf_debug(INFO_LOG, "[%s] : lcd on notify\n", __func__);
		gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_ON);
		break;

	case DRM_MODE_DPMS_OFF:
		gf_debug(INFO_LOG, "[%s] : lcd off notify\n", __func__);
		gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_OFF);
		break;

	default:
		gf_debug(INFO_LOG, "[%s] : other notifier, ignore\n", __func__);
		break;
	}
	FUNC_EXIT();
	return retval;
}

/* -------------------------------------------------------------------- */
/* file operation function                                              */
/* -------------------------------------------------------------------- */
static ssize_t gf_read(struct file *filp, char __user *buf,
			size_t count, loff_t *f_pos)
{
	FUNC_EXIT();
	return 0;
}

static ssize_t gf_write(struct file *filp, const char __user *buf,
			size_t count, loff_t *f_pos)
{
	gf_debug(ERR_LOG, "%s: Not support write op in TEE mode\n", __func__);
	return -EFAULT;
}

static irqreturn_t gf_irq(int irq, void *handle)
{
	struct gf_device *gf_dev = (struct gf_device *)handle;

	FUNC_ENTRY();

	wake_lock_timeout(&gf_dev->ttw_wl, msecs_to_jiffies(5000));
	gf_netlink_send(gf_dev, GF_NETLINK_IRQ);
	gf_dev->sig_count++;

	FUNC_EXIT();
	return IRQ_HANDLED;
}


static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	uint32_t key_event;
	int retval = 0;
	u8  buf    = 0;
	struct gf_device *gf_dev = NULL;
	struct gf_key gf_key;
	u8 netlink_route = GF_NETLINK_ROUTE;
	struct gf_ioc_chip_info info;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -EINVAL;

	/* Check access direction once here; don't repeat below.
	* IOC_DIR is from the user perspective, while access_ok is
	* from the kernel perspective; so they look reversed.
	*/
	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, (void __user *)arg,
				_IOC_SIZE(cmd));

	if (retval == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok(VERIFY_READ, (void __user *)arg,
				_IOC_SIZE(cmd));

	if (retval)
		return -EINVAL;

	gf_dev = (struct gf_device *)filp->private_data;
	if (!gf_dev) {
		gf_debug(ERR_LOG, "%s: gf_dev IS NULL ======\n", __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case GF_IOC_INIT:
		gf_debug(INFO_LOG, "%s: GF_IOC_INIT gf init======\n", __func__);
		gf_debug(INFO_LOG, "%s: Linux Version %s\n", __func__,
				GF_LINUX_VERSION);

		if (copy_to_user((void __user *)arg, (void *)&netlink_route,
			sizeof(u8))) {
			retval = -EFAULT;
			break;
		}

		if (gf_dev->system_status) {
			gf_debug(INFO_LOG, "%s: system re-started\n", __func__);
			break;
		}
		gf_irq_gpio_cfg(gf_dev);
		retval = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
			IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND | IRQF_ONESHOT,
			"goodix_fp_irq", gf_dev);
		if (!retval)
			gf_debug(INFO_LOG, "%s req success! irq number %d.\n",
				__func__, gf_dev->irq);
		else
			gf_debug(ERR_LOG, "%s irq request failed, retval=%d\n",
				__func__, retval);

		/* Request that the interrupt should be wakeable */
		enable_irq_wake(gf_dev->irq);

		gf_dev->irq_count = 1;
		gf_disable_irq(gf_dev);

		/* register screen on/off callback */
		gf_dev->notifier.notifier_call = gf_adf_notifier_callback;
		adf_register_client(&gf_dev->notifier);

		gf_dev->sig_count = 0;
		gf_dev->system_status = 1;

		gf_debug(INFO_LOG, "%s: gf init finished======\n", __func__);
		break;

	case GF_IOC_CHIP_INFO:
		if (copy_from_user(&info, (void __user *)arg,
			sizeof(struct gf_ioc_chip_info))) {
			retval = -EFAULT;
			break;
		}
		g_vendor_id = info.vendor_id;

		gf_debug(INFO_LOG, "%s: vendor_id 0x%x\n", __func__,
				g_vendor_id);
		gf_debug(INFO_LOG, "%s: mode 0x%x\n", __func__, info.mode);
		gf_debug(INFO_LOG, "%s: operation 0x%x\n", __func__,
				info.operation);
		break;

	case GF_IOC_EXIT:
		gf_debug(INFO_LOG, "%s: GF_IOC_EXIT ======\n", __func__);
		gf_disable_irq(gf_dev);
		if (gf_dev->irq) {
			disable_irq_wake(gf_dev->irq);
			free_irq(gf_dev->irq, gf_dev);
			gpio_free(gf_dev->irq_gpio);
			gf_dev->irq_count = 0;
			gf_dev->irq = 0;
		}
		adf_unregister_client(&gf_dev->notifier);
		gf_dev->system_status = 0;
		gf_debug(INFO_LOG, "%s: gf exit finished ======\n", __func__);
		break;

	case GF_IOC_RESET:
		gf_debug(INFO_LOG, "%s: chip reset command\n", __func__);
		gf_hw_reset(gf_dev);
		break;

	case GF_IOC_ENABLE_IRQ:
		gf_debug(INFO_LOG, "%s: GF_IOC_ENABLE_IRQ ======\n", __func__);
		gf_enable_irq(gf_dev);
		break;

	case GF_IOC_DISABLE_IRQ:
		gf_debug(INFO_LOG, "%s: GF_IOC_DISABLE_IRQ ======\n", __func__);
		gf_disable_irq(gf_dev);
		break;

	case GF_IOC_ENABLE_SPI_CLK:
		break;

	case GF_IOC_DISABLE_SPI_CLK:
		break;

	case GF_IOC_ENABLE_POWER:
		gf_debug(INFO_LOG, "%s: GF_IOC_ENABLE_POWER ==\n", __func__);
		break;

	case GF_IOC_DISABLE_POWER:
		gf_debug(INFO_LOG, "%s: GF_IOC_DISABLE_POWER ==\n", __func__);
		break;

	case GF_IOC_INPUT_KEY_EVENT:
		if (copy_from_user(&gf_key, (void __user *)arg,
			sizeof(struct gf_key))) {
			gf_debug(ERR_LOG, "Failed to copy input key event\n");
			retval = -EFAULT;
			break;
		}

		if (gf_key.key == GF_KEY_HOME) {
			key_event = GF_INPUT_BACK_KEY;
		} else if (gf_key.key == GF_KEY_POWER) {
			key_event = GF_INPUT_FF_KEY;
		} else if (gf_key.key == GF_KEY_CAPTURE) {
			key_event = GF_INPUT_CAMERA_KEY;
		} else {
			/* add special key define */
			key_event = GF_INPUT_OTHER_KEY;
		}
		gf_debug(INFO_LOG, "%s: received key event[%d], key=%d, value=%d\n",
				__func__, key_event, gf_key.key, gf_key.value);

		if ((GF_KEY_POWER == gf_key.key || GF_KEY_CAPTURE == gf_key.key)
			&& (gf_key.value == 1)) {
			input_report_key(gf_dev->input, key_event, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, key_event, 0);
			input_sync(gf_dev->input);
		} else if (gf_key.key == GF_KEY_UP) {
			input_report_key(gf_dev->input, GF_NAV_UP_KEY, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, GF_NAV_UP_KEY, 0);
			input_sync(gf_dev->input);
		} else if (gf_key.key == GF_KEY_DOWN) {
			input_report_key(gf_dev->input, GF_NAV_DOWN_KEY, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, GF_NAV_DOWN_KEY, 0);
			input_sync(gf_dev->input);
		} else if (gf_key.key == GF_KEY_RIGHT) {
			input_report_key(gf_dev->input, GF_NAV_RIGHT_KEY, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, GF_NAV_RIGHT_KEY, 0);
			input_sync(gf_dev->input);
		} else if (gf_key.key == GF_KEY_LEFT) {
			input_report_key(gf_dev->input, GF_NAV_LEFT_KEY, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, GF_NAV_LEFT_KEY, 0);
			input_sync(gf_dev->input);
		} else if ((gf_key.key != GF_KEY_POWER)
			&& (gf_key.key != GF_KEY_CAPTURE)) {
			input_report_key(gf_dev->input, key_event,
					gf_key.value);
			input_sync(gf_dev->input);
		}
		break;

	case GF_IOC_ENTER_SLEEP_MODE:
		gf_debug(INFO_LOG, "%s: GF_IOC_ENTER_SLEEP_MODE\n", __func__);
		break;

	case GF_IOC_GET_FW_INFO:
		gf_debug(INFO_LOG, "%s: GF_IOC_GET_FW_INFO ======\n", __func__);
		buf = gf_dev->need_update;

		gf_debug(DEBUG_LOG, "%s: firmware info  0x%x\n", __func__, buf);
		if (copy_to_user((void __user *)arg, (void *)&buf,
			sizeof(u8))) {
			gf_debug(ERR_LOG, "Failed to copy data to user\n");
			retval = -EFAULT;
		}

		break;
	case GF_IOC_REMOVE:
		gf_debug(INFO_LOG, "%s: GF_IOC_REMOVE ======\n", __func__);

		gf_netlink_destroy(gf_dev);

		mutex_lock(&gf_dev->release_lock);
		if (gf_dev->input == NULL) {
			mutex_unlock(&gf_dev->release_lock);
			break;
		}
		input_unregister_device(gf_dev->input);
		gf_dev->input = NULL;
		mutex_unlock(&gf_dev->release_lock);

		cdev_del(&gf_dev->cdev);
		sysfs_remove_group(&gf_dev->pdev->dev.kobj,
			&gf_debug_attr_group);
		device_destroy(gf_dev->class, gf_dev->devno);
		list_del(&gf_dev->device_entry);
		unregister_chrdev_region(gf_dev->devno, 1);
		class_destroy(gf_dev->class);

		platform_set_drvdata(gf_dev->pdev, NULL);
		gf_dev->pdev = NULL;
		mutex_destroy(&gf_dev->buf_lock);
		mutex_destroy(&gf_dev->release_lock);
		wake_lock_destroy(&gf_dev->ttw_wl);

		break;
	default:
		gf_debug(ERR_LOG, "gf doesn't support this command(%x)\n", cmd);
		break;
	}

	return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	int retval = 0;

	FUNC_ENTRY();

	retval = filp->f_op->unlocked_ioctl(filp, cmd, arg);

	FUNC_EXIT();
	return retval;
}
#endif

static unsigned int gf_poll(struct file *filp, struct poll_table_struct *wait)
{
	gf_debug(ERR_LOG, "Not support poll opertion in TEE version\n");
	return -EFAULT;
}


/* -------------------------------------------------------------------- */
/* devfs                                                              */
/* -------------------------------------------------------------------- */
static ssize_t gf_debug_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	gf_debug(INFO_LOG, "%s: Show debug_level = 0x%x\n", __func__,
			g_debug_level);
	return sprintf(buf, "vendor id 0x%x\n", g_vendor_id);
}

static ssize_t gf_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gf_device *gf_dev =  dev_get_drvdata(dev);
	int retval = 0;

	if (gf_dev == NULL) {
		gf_debug(INFO_LOG, "%s: point null error\n", __func__);
		return count;
	}

	if (!strncmp(buf, "-8", 2)) {
		gf_debug(INFO_LOG, "%s: parameter -8, enable spi clock\n",
				__func__);

		gf_irq_gpio_cfg(gf_dev);
		retval = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
			IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND | IRQF_ONESHOT,
			"goodix_fp_irq", gf_dev);
		if (!retval)
			gf_debug(INFO_LOG, "%s success! irq number %d.\n",
				__func__, gf_dev->irq);
		else
			gf_debug(ERR_LOG, "%s irq req failed, retval=%d\n",
				__func__, retval);

		/* Request that the interrupt should be wakeable */
		enable_irq_wake(gf_dev->irq);

		gf_dev->irq_count = 1;
	} else if (!strncmp(buf, "-9", 2)) {
		gf_debug(INFO_LOG, "%s: parameter -9, disable spi clock test\n",
				__func__);

	} else if (!strncmp(buf, "-10", 3)) {
		gf_debug(INFO_LOG, "%s: parameter -10, init start\n", __func__);

		gf_irq_gpio_cfg(gf_dev);
		retval = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				dev_name(&(gf_dev->pdev->dev)), gf_dev);
		if (!retval)
			gf_debug(INFO_LOG, "%s irq thread request success!\n",
					__func__);
		else
			gf_debug(ERR_LOG, "%s irq req failed, retval=%d\n",
					__func__, retval);

		gf_dev->irq_count = 1;
		gf_disable_irq(gf_dev);

		/* register screen on/off callback */
		gf_dev->notifier.notifier_call = gf_adf_notifier_callback;
		adf_register_client(&gf_dev->notifier);

		gf_dev->sig_count = 0;

		gf_debug(INFO_LOG, "%s: gf init finished======\n", __func__);

	} else if (!strncmp(buf, "-11", 3)) {
		gf_debug(INFO_LOG, "%s: parameter -11, enable irq\n", __func__);
		gf_enable_irq(gf_dev);
	} else if (!strncmp(buf, "-12", 3)) {
		gf_debug(INFO_LOG, "%s: parameter -13, Vendor ID --> 0x%x\n",
			__func__, g_vendor_id);
	} else {
		gf_debug(ERR_LOG, "%s: wrong parameter!====\n", __func__);
	}

	return count;
}

/* -------------------------------------------------------------------- */
/* device function							*/
/* -------------------------------------------------------------------- */
static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_device *gf_dev = NULL;
	int status = -ENXIO;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);
	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devno == inode->i_rdev) {
			gf_debug(INFO_LOG, "%s, Found\n", __func__);
			status = 0;
			break;
		}
	}
	mutex_unlock(&device_list_lock);

	if (status == 0) {
		filp->private_data = gf_dev;
		nonseekable_open(inode, filp);
		gf_debug(INFO_LOG, "%s, Success to open device. irq = %d\n",
			__func__, gf_dev->irq);
	} else {
		gf_debug(ERR_LOG, "%s, No device for minor %d\n",
			__func__, iminor(inode));
	}
	FUNC_EXIT();
	return status;
}

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_device *gf_dev = NULL;

	FUNC_ENTRY();
	gf_dev = filp->private_data;
	if (gf_dev->irq)
		gf_disable_irq(gf_dev);
	gf_dev->need_update = 0;
	FUNC_EXIT();
	return 0;
}

static const struct file_operations gf_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	* gets more complete API coverage.	It'll simplify things
	* too, except for the locking.
	*/
	.write =	gf_write,
	.read =		gf_read,
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif
	.open =		gf_open,
	.release =	gf_release,
	.poll	= gf_poll,
};

/*-------------------------------------------------------------------------*/

static int gf_probe(struct platform_device *pdev)
{
	struct gf_device *gf_dev = NULL;
	int status = -EINVAL;

	FUNC_ENTRY();

	/* Allocate driver data */
	gf_dev = kzalloc(sizeof(struct gf_device), GFP_KERNEL);
	if (!gf_dev) {
		status = -ENOMEM;
		goto err;
	}

	mutex_init(&gf_dev->buf_lock);
	mutex_init(&gf_dev->release_lock);

	INIT_LIST_HEAD(&gf_dev->device_entry);

	gf_dev->device_count     = 0;
	gf_dev->probe_finish     = 0;
	gf_dev->system_status    = 0;
	gf_dev->need_update      = 0;
	gf_dev->irq = 0;

	/*setup gf configurations.*/
	gf_debug(INFO_LOG, "%s, Setting gf device configuration\n", __func__);

	gf_dev->pdev = pdev;
	platform_set_drvdata(pdev, gf_dev);

	/* check firmware Integrity */
	gf_debug(INFO_LOG, "%s, Sensor type : %s.\n", __func__,
			GOODIX_SENSOR_TYPE);
	/* create class */
	gf_dev->class = class_create(THIS_MODULE, GF_CLASS_NAME);
	if (IS_ERR(gf_dev->class)) {
		gf_debug(ERR_LOG, "%s, Failed to create class.\n", __func__);
		status = -ENODEV;
		goto err_class;
	}

	/* get device no */
	if (GF_DEV_MAJOR > 0) {
		gf_dev->devno = MKDEV(GF_DEV_MAJOR, gf_dev->device_count++);
		status = register_chrdev_region(gf_dev->devno, 1, GF_DEV_NAME);
	} else {
		status = alloc_chrdev_region(&gf_dev->devno,
			gf_dev->device_count++, 1, GF_DEV_NAME);
	}
	if (status < 0) {
		gf_debug(ERR_LOG, "%s, Failed to alloc devno.\n", __func__);
		goto err_devno;
	} else {
		gf_debug(INFO_LOG, "%s, major=%d, minor=%d\n", __func__,
				MAJOR(gf_dev->devno), MINOR(gf_dev->devno));
	}

	/* create device */
	gf_dev->device = device_create(gf_dev->class, &pdev->dev,
				gf_dev->devno, gf_dev, GF_DEV_NAME);
	if (IS_ERR(gf_dev->device)) {
		gf_debug(ERR_LOG, "%s, Failed to create device.\n", __func__);
		status = -ENODEV;
		goto err_device;
	} else {
		mutex_lock(&device_list_lock);
		list_add(&gf_dev->device_entry, &device_list);
		mutex_unlock(&device_list_lock);
		gf_debug(INFO_LOG, "%s, device create success.\n", __func__);
	}

	/* get gpio info from dts or defination */
	gf_get_gpio_dts_info(gf_dev);

	/* create sysfs */
	status = sysfs_create_group(&pdev->dev.kobj, &gf_debug_attr_group);
	if (status) {
		gf_debug(ERR_LOG, "%s, Failed create sysfs file\n", __func__);
		status = -ENODEV;
		goto err_sysfs;
	} else {
		gf_debug(INFO_LOG, "%s, Success create sysfs file\n", __func__);
	}

	/* cdev init and add */
	cdev_init(&gf_dev->cdev, &gf_fops);
	gf_dev->cdev.owner = THIS_MODULE;
	status = cdev_add(&gf_dev->cdev, gf_dev->devno, 1);
	if (status) {
		gf_debug(ERR_LOG, "%s, Failed to add cdev.\n", __func__);
		goto err_cdev;
	}

	/*register device within input system.*/
	gf_dev->input = input_allocate_device();
	if (gf_dev->input == NULL) {
		gf_debug(ERR_LOG, "%s, Failed alloc input device\n", __func__);
		status = -ENOMEM;
		goto err_input;
	}

	__set_bit(EV_KEY, gf_dev->input->evbit);
	__set_bit(GF_INPUT_HOME_KEY, gf_dev->input->keybit);

	__set_bit(GF_INPUT_MENU_KEY, gf_dev->input->keybit);
	__set_bit(GF_INPUT_BACK_KEY, gf_dev->input->keybit);
	__set_bit(GF_INPUT_FF_KEY, gf_dev->input->keybit);

	__set_bit(GF_NAV_UP_KEY, gf_dev->input->keybit);
	__set_bit(GF_NAV_DOWN_KEY, gf_dev->input->keybit);
	__set_bit(GF_NAV_RIGHT_KEY, gf_dev->input->keybit);
	__set_bit(GF_NAV_LEFT_KEY, gf_dev->input->keybit);
	__set_bit(GF_INPUT_CAMERA_KEY, gf_dev->input->keybit);

	gf_dev->input->name = GF_INPUT_NAME;
	if (input_register_device(gf_dev->input)) {
		gf_debug(ERR_LOG, "%s, Failed regi input device.\n", __func__);
		status = -ENODEV;
		goto err_input_2;
	}

	/* netlink interface init */
	status = gf_netlink_init(gf_dev);
	if (status == -1) {
		mutex_lock(&gf_dev->release_lock);
		input_unregister_device(gf_dev->input);
		gf_dev->input = NULL;
		mutex_unlock(&gf_dev->release_lock);
		goto err_input;
	}

	wake_lock_init(&gf_dev->ttw_wl, WAKE_LOCK_SUSPEND, "goodix_ttw_wl");
	gf_dev->probe_finish = 1;
	gf_dev->is_sleep_mode = 0;
	gf_debug(INFO_LOG, "%s probe finished\n", __func__);

	FUNC_EXIT();
	return 0;

err_input_2:
	mutex_lock(&gf_dev->release_lock);
	input_free_device(gf_dev->input);
	gf_dev->input = NULL;
	mutex_unlock(&gf_dev->release_lock);

err_input:
	cdev_del(&gf_dev->cdev);

err_cdev:
	sysfs_remove_group(&pdev->dev.kobj, &gf_debug_attr_group);

err_sysfs:
	device_destroy(gf_dev->class, gf_dev->devno);
	list_del(&gf_dev->device_entry);

err_device:
	unregister_chrdev_region(gf_dev->devno, 1);

err_devno:
	class_destroy(gf_dev->class);

err_class:
	mutex_destroy(&gf_dev->buf_lock);
	mutex_destroy(&gf_dev->release_lock);
	platform_set_drvdata(pdev, NULL);
	gf_dev->pdev = NULL;
	kfree(gf_dev);
	gf_dev = NULL;
err:

	FUNC_EXIT();
	return status;
}

static int gf_remove(struct platform_device *pdev)
{
	struct gf_device *gf_dev = platform_get_drvdata(pdev);

	FUNC_ENTRY();

	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->irq) {
		disable_irq_wake(gf_dev->irq);
		free_irq(gf_dev->irq, gf_dev);
		gf_dev->irq_count = 0;
		gf_dev->irq = 0;
	}
	adf_unregister_client(&gf_dev->notifier);
	mutex_lock(&gf_dev->release_lock);
	if (gf_dev->input == NULL) {
		mutex_unlock(&gf_dev->release_lock);
		kfree(gf_dev);
		FUNC_EXIT();
		return 0;
	}
	input_unregister_device(gf_dev->input);
	gf_dev->input = NULL;
	mutex_unlock(&gf_dev->release_lock);

	gf_netlink_destroy(gf_dev);
	cdev_del(&gf_dev->cdev);
	sysfs_remove_group(&pdev->dev.kobj, &gf_debug_attr_group);
	device_destroy(gf_dev->class, gf_dev->devno);
	list_del(&gf_dev->device_entry);

	unregister_chrdev_region(gf_dev->devno, 1);
	class_destroy(gf_dev->class);
	platform_set_drvdata(pdev, NULL);
	gf_dev->pdev = NULL;

	mutex_destroy(&gf_dev->buf_lock);
	mutex_destroy(&gf_dev->release_lock);
	wake_lock_destroy(&gf_dev->ttw_wl);

	kfree(gf_dev);
	FUNC_EXIT();
	return 0;
}

/*-------------------------------------------------------------------------*/
static struct platform_driver gf_platform_driver = {
	.driver = {
		.name = GF_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = gf_of_match,
#endif
	},
	.probe = gf_probe,
	.remove = gf_remove,
};

static int __init gf_init(void)
{
	int status = 0;

	FUNC_ENTRY();

	status = platform_driver_register(&gf_platform_driver);
	if (status < 0) {
		gf_debug(ERR_LOG, "%s, Failed regi PLATFORM drv\n", __func__);
		return -EINVAL;
	}

	FUNC_EXIT();
	return status;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
	FUNC_ENTRY();
	platform_driver_unregister(&gf_platform_driver);
	FUNC_EXIT();
}
module_exit(gf_exit);


MODULE_AUTHOR("goodix");
MODULE_DESCRIPTION("Goodix Fingerprint chip TEE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf_spi");
