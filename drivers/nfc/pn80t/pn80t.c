/*
 * Copyright (C) 2010 Trusted Logic S.A.
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include "pn80t.h"

#define MAX_BUFFER_SIZE	512
#define NFC_CLK_FREQ 26000000

struct pn544_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice pn544_device;
	unsigned int ven_gpio;
	unsigned int firm_gpio;
	bool irq_enabled;
	spinlock_t irq_enabled_lock;
	unsigned int irq_gpio;
	/* CLK_REQ IRQ to signal the state has changed */
	unsigned int irq_gpio_clk_req;
	unsigned int clkreq_gpio;
	/* CLK control */
	unsigned int clk_src_gpio;
	const char *clk_src_name;
	struct clk *s_clk;
	bool clk_run;
};

static const struct of_device_id msm_match_table[] = {
	{.compatible = "nxp,nfc-pn544"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_match_table);

static int nfc_parse_dt(struct device *dev, struct pn544_dev *pdata)
{

	struct device_node *np = dev->of_node;

	pdata->ven_gpio = of_get_named_gpio(np, "nxp,ven-gpio", 0);
	if ((!gpio_is_valid(pdata->ven_gpio)))
		return -EINVAL;

	pdata->irq_gpio = of_get_named_gpio(np, "nxp,irq-gpio", 0);
	if ((!gpio_is_valid(pdata->irq_gpio)))
		return -EINVAL;

	pdata->firm_gpio = of_get_named_gpio(np, "nxp,firm-gpio", 0);
	if ((!gpio_is_valid(pdata->firm_gpio)))
		return -EINVAL;

	pdata->clkreq_gpio = of_get_named_gpio(np, "nxp,clkreq-gpio", 0);
	if ((!gpio_is_valid(pdata->clkreq_gpio)))
		return -EINVAL;

	pr_info("GPIO ven,irq,firm clk_req= %d, %d, %d, %d\n",
		pdata->ven_gpio, pdata->irq_gpio, pdata->firm_gpio,
		pdata->clkreq_gpio);

	return 0;
}

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) {
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;

	if (!gpio_get_value(pn544_dev->irq_gpio))
		return IRQ_HANDLED;

	pn544_disable_irq(pn544_dev);

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
			      size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	memset(tmp, 0, MAX_BUFFER_SIZE);

	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->client->irq);
		ret = wait_event_interruptible(pn544_dev->read_wq,
				gpio_get_value(pn544_dev->irq_gpio));

		pn544_disable_irq(pn544_dev);

		if (ret)
			goto fail;
	}

	/* Read data */
	ret = i2c_master_recv(pn544_dev->client, tmp, count);
	mutex_unlock(&pn544_dev->read_mutex);

	if (ret < 0) {
		pr_info("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_info("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		pr_info("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
			       size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_info("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	/* Write data */
	ret = i2c_master_send(pn544_dev->client, tmp, count);
	if (ret != count) {
		pr_info("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
		goto exit;
	}

exit:
	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	struct pn544_dev *pn544_dev = container_of(filp->private_data,
						   struct pn544_dev,
						   pn544_device);

	filp->private_data = pn544_dev;

	return 0;
}

static long pn544_dev_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;

	switch (cmd) {
	case PN544_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			pr_info("%s power on with firmware\n", __func__);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			gpio_set_value(pn544_dev->firm_gpio, 1);
			msleep(20);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(50);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(20);
		} else if (arg == 1) {
			/* power on */
			pr_info("%s power on\n", __func__);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			irq_set_irq_wake(pn544_dev->client->irq, 1);
			msleep(20);
		} else if (arg == 0) {
			/* power off */
			pr_info("%s power off\n", __func__);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			irq_set_irq_wake(pn544_dev->client->irq, 0);
			msleep(20);
		} else {
			pr_info("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		pr_info("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn544_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = pn544_dev_read,
	.write = pn544_dev_write,
	.open = pn544_dev_open,
	.unlocked_ioctl = pn544_dev_ioctl,
};

static int pn544_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	int ret;
	struct pn544_dev *pn544_dev;
	struct clk *source = NULL;
	struct clk *enable = NULL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_info("%s : need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	pn544_dev = devm_kzalloc(&client->dev, sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL)
		return -ENOMEM;

	pn544_dev->client = client;
	ret = nfc_parse_dt(&client->dev, pn544_dev);
	if (ret) {
		pr_err("parse node error. pls check the dts file\n");
		return -EINVAL;
	}

	if (devm_gpio_request(&client->dev, pn544_dev->clkreq_gpio,
			"nfc_clkreq_gpio") != 0) {
		pr_err("PN544: gpio_clkreq_request error\n");
		return -EINVAL;
	}

	if (devm_gpio_request(&client->dev, pn544_dev->irq_gpio,
			"nfc_int") != 0) {
		pr_err("PN544: gpio_IRQ_request error\n");
		return -EINVAL;
	}

	if (devm_gpio_request(&client->dev, pn544_dev->ven_gpio,
			"nfc_ven") != 0) {
		pr_err("PN544: gpio_VEN_request error\n");
		return -EINVAL;
	}

	if (devm_gpio_request(&client->dev, pn544_dev->firm_gpio,
			"nfc_firm") != 0) {
		pr_err("PN544: gpio_firm_request error\n");
		return -EINVAL;
	}

	gpio_direction_input(pn544_dev->clkreq_gpio);
	gpio_direction_output(pn544_dev->firm_gpio, 0);
	gpio_direction_output(pn544_dev->ven_gpio, 1);
	gpio_direction_input(pn544_dev->irq_gpio);

	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = PN544_NAME;
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	client->irq = gpio_to_irq(pn544_dev->irq_gpio);
	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	pn544_dev->irq_enabled = true;

	ret = devm_request_irq(&client->dev, client->irq, pn544_dev_irq_handler,
			  IRQF_TRIGGER_HIGH, client->name, pn544_dev);
	if (ret) {
		pr_err("request_irq failed\n");
		goto err_request_irq_failed;
	}
	pn544_disable_irq(pn544_dev);

	pn544_dev->s_clk = devm_clk_get(&client->dev, "nfc_clk");
	if (IS_ERR(pn544_dev->s_clk)) {
		pr_err("get nfc_clk failed!\n");
		ret = -ENODEV;
		goto err_request_clk;
	}

	source = devm_clk_get(&client->dev, "source");
	if (IS_ERR(source)) {
		pr_err("get nfc_clk's source failed!\n");
		ret = -ENODEV;
		goto err_request_clk;
	}

	enable = devm_clk_get(&client->dev, "enable");
	if (IS_ERR(enable)) {
		pr_err("get nfc_clk's enable failed!\n");
		ret = -ENODEV;
		goto err_request_clk;
	}

	clk_set_parent(pn544_dev->s_clk, source);
	clk_set_rate(pn544_dev->s_clk, NFC_CLK_FREQ);
	clk_prepare_enable(pn544_dev->s_clk);
	clk_prepare_enable(enable);

	i2c_set_clientdata(client, pn544_dev);

	pr_info("nfc probe successful\n");

	return 0;

err_request_clk:
err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	return ret;
}

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

	pn544_dev = i2c_get_clientdata(client);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);

	return 0;
}

static const struct i2c_device_id pn544_id[] = {
	{PN544_NAME, 0},
	{}
};

static struct i2c_driver pn544_driver = {
	.id_table = pn544_id,
	.probe = pn544_probe,
	.remove = pn544_remove,
	.driver = {
		   .name = PN544_NAME,
		   .of_match_table = msm_match_table,
		   },
};

module_i2c_driver(pn544_driver);

MODULE_AUTHOR("Ernest Li");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
