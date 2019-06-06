/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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

#define pr_fmt(fmt) "[Audio:PIPE] "fmt

#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>

#include "audio-sipc.h"
#include "sprd-asoc-common.h"
#include "sprd-string.h"


#define  AGDSP_COMMUNICATION_TIMEOUT	0x27


/*
* function:
* a: dsp assert active cmd =
* b  dsp assert passive cmd =
* c: dsp voice call sample rate changed cmd =
* d: check dsp status for alive. cmd =
 */

static int aud_pipe_open(struct inode *inode,
	struct file *filp);
static int aud_pipe_release(struct inode *inode,
	struct file *filp);
static ssize_t aud_pipe_read(struct file *filp,
	char __user *buf, size_t count, loff_t *ppos);
static ssize_t aud_pipe_write(struct file *filp,
	const char __user *buf, size_t count, loff_t *ppos);
static long aud_pipe_ioctl(struct file *filp,
	unsigned int cmd, unsigned long args);
static int aud_pipe_recv_cmd(uint16_t channel, struct aud_smsg *o_msg,
	int32_t timeout);

struct aud_pipe_device {
	int32_t major;
	int32_t minor;
	struct class *audio_pipe_class;
	struct cdev chrdev;
	uint32_t devicesnr;
	char *device_name;
};

struct aud_pipe_data_t {
	uint32_t target_id;
	struct aud_pipe_device pipe_chr_device;
	uint32_t dst;
};

static struct aud_pipe_data_t *aud_pipe_data;
static const struct file_operations audio_pipe_chr_fops = {
	.open = aud_pipe_open,
	.release = aud_pipe_release,
	.read = aud_pipe_read,
	.write = aud_pipe_write,
	.unlocked_ioctl = aud_pipe_ioctl,
	.owner = THIS_MODULE,
};

static int aud_pipe_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct aud_pipe_device *aud_pipe_dev = NULL;
	struct aud_pipe_data_t *aud_pipe_data = NULL;

	aud_pipe_dev = container_of(inode->i_cdev, struct aud_pipe_device,
				    chrdev);
	aud_pipe_data = container_of(aud_pipe_dev, struct aud_pipe_data_t,
				     pipe_chr_device);
	filp->private_data = aud_pipe_data;

	if (filp->f_flags & O_NONBLOCK)
		pr_info("%s noblock %d\n", __func__,
			filp->f_flags & O_NONBLOCK);

	ret = aud_ipc_ch_open(AMSG_CH_DSP_ASSERT_CTL);
	if (ret == 0)
		pr_info("%s channel opened %u\n", __func__,
			AMSG_CH_DSP_ASSERT_CTL);
	else
		pr_err("%s channel(%u)_open failed\n", __func__,
			AMSG_CH_DSP_ASSERT_CTL);

	return 0;
}

static int aud_pipe_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct aud_pipe_data_t *aud_pipe_data = filp->private_data;

	if (filp->f_flags & O_NONBLOCK)
		pr_info("%s noblock =%d\n", __func__,
			filp->f_flags & O_NONBLOCK);

	ret = aud_ipc_ch_close(AMSG_CH_DSP_ASSERT_CTL);
	if (ret != 0) {
		pr_err("%s, Failed to close channel(%d)\n",
			__func__, AMSG_CH_DSP_ASSERT_CTL);
	}

	if (aud_pipe_data) {
		aud_pipe_data = NULL;
		filp->private_data = NULL;
	}

	return ret;
}

static ssize_t aud_pipe_read(struct file *filp,
	char __user *buf, size_t count, loff_t *ppos)
{
	int timeout = -1;
	struct aud_smsg  user_msg_out = {0};
	int ret = 0;

	if (sizeof(struct aud_smsg) != count) {
		pr_err("input not a struct aud_smsg type\n");
		ret = -EINVAL;
		goto error;
	}

	if (filp->f_flags & O_NONBLOCK) {
		timeout = 0;
		pr_info("%s noblok %u,timeout = %d\n", __func__,
			filp->f_flags & O_NONBLOCK, timeout);
	}

	ret = aud_pipe_recv_cmd(AMSG_CH_DSP_ASSERT_CTL,
		&user_msg_out, timeout);
	if (ret == 0)
		ret = sizeof(struct aud_smsg);
	else
		pr_err("%s aud_pipe_recv_cmd failed\n", __func__);

	if (unalign_copy_to_user((void __user *)buf,
		&user_msg_out, sizeof(struct aud_smsg))) {
		pr_err("%s: failed to copy to user!\n", __func__);
		ret = -EFAULT;
		goto error;
	}

	pr_info("user_msg_out.channel=%#hx\n", user_msg_out.channel);
	pr_info("user_msg_out.command=%#hx\n", user_msg_out.command);
	pr_info("user_msg_out.parameter0=%#x\n", user_msg_out.parameter0);
	pr_info("user_msg_out.parameter1=%#x\n", user_msg_out.parameter1);
	pr_info("user_msg_out.parameter2=%#x\n", user_msg_out.parameter2);
	pr_info("user_msg_out.parameter3=%#x\n", user_msg_out.parameter3);

	return ret;

error:
	pr_err("%s failed", __func__);

	return ret;
}

static ssize_t aud_pipe_write(struct file *filp,
	const char __user *buf, size_t count, loff_t *ppos)
{
	struct aud_smsg  user_msg_from_user = {0};
	int ret = 0;
	int timeout = -1;

	if (filp->f_flags & O_NONBLOCK) {
		timeout = 0;
		pr_info("%s noblock %u,timeout = %d\n", __func__,
			filp->f_flags & O_NONBLOCK, timeout);
	}

	if (unalign_copy_from_user(&user_msg_from_user,
		(void __user *)buf, sizeof(struct aud_smsg))) {
		pr_err("%s failed unalign_copy_from_user\n", __func__);
		ret = -EFAULT;
		goto error;
	}

	pr_info("user_msg_in.channel=%#hx\n", user_msg_from_user.channel);
	pr_info("user_msg_in.command=%#hx\n", user_msg_from_user.command);
	pr_info("user_msg_in.parameter0=%#x\n", user_msg_from_user.parameter0);
	pr_info("user_msg_in.parameter1=%#x\n", user_msg_from_user.parameter1);
	pr_info("user_msg_in.parameter2=%#x\n", user_msg_from_user.parameter2);
	pr_info("user_msg_in.parameter3=%#x\n", user_msg_from_user.parameter3);
	ret = aud_send_cmd_no_wait(user_msg_from_user.channel,
		user_msg_from_user.command,
		user_msg_from_user.parameter0,
		user_msg_from_user.parameter1,
		user_msg_from_user.parameter2,
		user_msg_from_user.parameter3);
	if (ret != 0) {
		pr_err("%s aud_send_msg failed\n", __func__);
		ret = -1;
		goto error;
	}

	ret = sizeof(struct aud_smsg);

	return ret;
error:
	pr_err("%s failed", __func__);

	return ret;
}

static long aud_pipe_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	pr_info("audio_pipe_ioctl donothing");

	return 0;
}

static int aud_pipe_recv_cmd(uint16_t channel, struct aud_smsg *o_msg,
	int32_t timeout)
{
	int ret = 0;
	struct aud_smsg mrecv = { 0 };

	aud_smsg_set(&mrecv, channel, 0, 0, 0, 0, 0);

	ret = aud_smsg_recv(AUD_IPC_AGDSP, &mrecv, timeout);
	if (ret < 0) {
		if ((channel == AMSG_CH_DSP_ASSERT_CTL)
			&& (ret == -EPIPE)) {
			pr_err("%s, Failed to recv,ret(%d) and notify\n",
				__func__, ret);
			mrecv.command = AGDSP_COMMUNICATION_TIMEOUT;
			mrecv.parameter0 = AGDSP_COMMUNICATION_TIMEOUT;
		} else {
			if (-ENODATA == ret) {
				pr_warn("%s  ENODATA\n", __func__);
				return ret;
			}
			pr_err("%s, Failed to recv,ret(%d)\n",
				__func__, ret);
			return ret;
		}
	}

	sp_asoc_pr_dbg("%s, chan: 0x%x, cmd: 0x%x\n",
		__func__, mrecv.channel, mrecv.command);
	sp_asoc_pr_dbg("value0: 0x%x, value1: 0x%x\n",
		mrecv.parameter0, mrecv.parameter1);
	sp_asoc_pr_dbg(" value2: 0x%x, value3: 0x%x, timeout: %d\n",
		mrecv.parameter2, mrecv.parameter3, timeout);

	if (mrecv.channel == channel) {
		unalign_memcpy(o_msg, &mrecv, sizeof(struct aud_smsg));
		return 0;
	}
	pr_err("%s,wrong chan(0x%x) from agdsp\n",
		__func__, mrecv.channel);

	return ret;
}

static const struct file_operations audio_pipe_fops = {
	.open = aud_pipe_open,
	.release = aud_pipe_release,
	.read = aud_pipe_read,
	.write = aud_pipe_write,
	.unlocked_ioctl = aud_pipe_ioctl,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static int aud_pipe_parse_dt(struct device *dev,
	struct aud_pipe_data_t *aud_pipe_data)
{
	struct device_node *np = dev->of_node;
	int ret = 0;
	unsigned char dst;
	unsigned char devicesnr;

	if (NULL == np || NULL == aud_pipe_data) {
		pr_err("%s failed\n", __func__);
		ret = -1;
		goto error;
	}

	ret = of_property_read_string(np, "sprd,name",
		(const char **)&(aud_pipe_data->pipe_chr_device.device_name));
	if (ret)
		goto error;
	ret = of_property_read_u8(np, "sprd,dst",
		&dst);
	if (ret)
		goto error;
	aud_pipe_data->dst = dst;
	ret = of_property_read_u8(np, "sprd,devicesnr",
		&devicesnr);
	if (ret)
		goto error;
	aud_pipe_data->pipe_chr_device.devicesnr = devicesnr;
	ret = of_property_read_u32(np,
		"mailbox,core", &(aud_pipe_data->target_id));
	if (ret)
		goto error;

	return ret;
error:
	pr_err("%s failed", __func__);

	return ret;
}

static int audio_pipe_probe(struct platform_device *pdev)
{
	int rval = 0;
	dev_t devid;
	int i;

	aud_pipe_data = devm_kzalloc(&pdev->dev, sizeof(struct aud_pipe_data_t),
		GFP_KERNEL);
	if (!aud_pipe_data)
		return -ENOMEM;

	rval = aud_pipe_parse_dt(&pdev->dev, aud_pipe_data);
	if (rval != 0) {
		pr_err("%s parse dt failed\n", __func__);
		return (-EINVAL);
	}
	pr_info("%s:name=%s\n", __func__,
		aud_pipe_data->pipe_chr_device.device_name);
	pr_info("dst=%u, devicesnr=%u, target_id=%u\n",
		aud_pipe_data->dst,
		aud_pipe_data->pipe_chr_device.devicesnr,
		aud_pipe_data->target_id);

	rval = alloc_chrdev_region(&devid, 0,
		aud_pipe_data->pipe_chr_device.devicesnr,
		aud_pipe_data->pipe_chr_device.device_name);
	if (rval != 0) {
		pr_err("Failed to alloc audio_pipe chrdev\n");
		return rval;
	}
	cdev_init(&(aud_pipe_data->pipe_chr_device.chrdev),
		&audio_pipe_fops);
	rval = cdev_add(&(aud_pipe_data->pipe_chr_device.chrdev),
		devid, aud_pipe_data->pipe_chr_device.devicesnr);
	if (rval != 0) {
		unregister_chrdev_region(devid,
			aud_pipe_data->pipe_chr_device.devicesnr);
		pr_err("Failed to add audio_pipe cdev\n");
		return rval;
	}
	aud_pipe_data->pipe_chr_device.major = MAJOR(devid);
	aud_pipe_data->pipe_chr_device.minor = MINOR(devid);
	aud_pipe_data->pipe_chr_device.audio_pipe_class =
		class_create(THIS_MODULE,
			aud_pipe_data->pipe_chr_device.device_name);
	if (IS_ERR(aud_pipe_data->pipe_chr_device.audio_pipe_class))
		return PTR_ERR(aud_pipe_data->pipe_chr_device.audio_pipe_class);

	if (aud_pipe_data->pipe_chr_device.devicesnr > 1) {
		for (i = 0; i < aud_pipe_data->pipe_chr_device.devicesnr; i++) {
			device_create(
				aud_pipe_data->pipe_chr_device.audio_pipe_class,
				NULL,
				MKDEV(aud_pipe_data->pipe_chr_device.major,
				aud_pipe_data->pipe_chr_device.minor + i),
				NULL, "%s%d",
				aud_pipe_data->pipe_chr_device.device_name,
				i);
		}
	} else {
		device_create(aud_pipe_data->pipe_chr_device.audio_pipe_class,
			NULL,
			MKDEV(aud_pipe_data->pipe_chr_device.major,
			aud_pipe_data->pipe_chr_device.minor),
			NULL, "%s", aud_pipe_data->pipe_chr_device.device_name);
	}
	platform_set_drvdata(pdev, aud_pipe_data);

	return 0;
}

static int audio_pipe_remove(struct platform_device *pdev)
{

	struct aud_pipe_data_t *aud_pipe_data = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < aud_pipe_data->pipe_chr_device.devicesnr; i++) {
		device_destroy(aud_pipe_data->pipe_chr_device.audio_pipe_class,
			       MKDEV(aud_pipe_data->pipe_chr_device.major,
				     aud_pipe_data->pipe_chr_device.minor + i));
	}
	cdev_del(&(aud_pipe_data->pipe_chr_device.chrdev));
	unregister_chrdev_region(MKDEV(aud_pipe_data->pipe_chr_device.major,
				       aud_pipe_data->pipe_chr_device.minor),
				 aud_pipe_data->pipe_chr_device.devicesnr);
	class_destroy(aud_pipe_data->pipe_chr_device.audio_pipe_class);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id audio_pipe_match_table[] = {
	{.compatible = "sprd,audio_pipe", },
	{ },
};

static struct platform_driver audio_pipe_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd_audio_pipe",
		.of_match_table = audio_pipe_match_table,
	},
	.probe = audio_pipe_probe,
	.remove = audio_pipe_remove,
};

static int __init audio_pipe_init(void)
{
	return platform_driver_register(&audio_pipe_driver);
}

static void __exit audio_pipe_exit(void)
{
	platform_driver_unregister(&audio_pipe_driver);
}

module_init(audio_pipe_init);
module_exit(audio_pipe_exit);

MODULE_AUTHOR("SPRD");
MODULE_DESCRIPTION("SIPC/AUDIO_PIPE driver");
MODULE_LICENSE("GPL");
