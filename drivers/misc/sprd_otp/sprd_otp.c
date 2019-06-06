/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/bug.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sprd_otp.h>

#define CACHED_BLOCK_MAX                ( 256 )
#define CACHED_BLOCK_WIDTH              ( 32 )
static u8 cached_otp[CACHED_BLOCK_MAX * CACHED_BLOCK_WIDTH / 8];

static LIST_HEAD(sprd_otp_list);
static DEFINE_MUTEX(sprd_otp_mtx);

static struct sprd_otp_device *sprd_otp_find(int minor)
{
	struct sprd_otp_device *otp;

	if (unlikely(list_empty(&sprd_otp_list)))
		return NULL;

	list_for_each_entry(otp, &sprd_otp_list, list) {
		if (otp->misc.minor == minor)
			return otp;
	}
	return NULL;
}

static ssize_t sprd_otp_read(struct file *file, char __user * buf,
			     size_t count, loff_t * f_pos)
{
	int ret;
	int minor = iminor(file_inode(file));
	struct sprd_otp_device *otp = sprd_otp_find(minor);

	mutex_lock(&sprd_otp_mtx);
	if (otp && otp->ops && otp->ops->read) {
		int i, blk_index = (int)*f_pos / otp->blk_width;
		int blk_max = blk_index + DIV_ROUND_UP(count, otp->blk_width);
		pr_info("otp read blk %d - %d\n", blk_index, blk_max);
		for (i = blk_index; i < blk_max; i++) {
			int index = i * otp->blk_width;
			u32 val = otp->ops->read(i);
			memcpy(&cached_otp[index], &val, otp->blk_width);
		}
	}
	pr_info("otp read from %d %d\n", (int)*f_pos, (int)file->f_pos);
	ret =
	    simple_read_from_buffer(buf, count, f_pos, cached_otp,
				    sizeof(cached_otp));

	mutex_unlock(&sprd_otp_mtx);
	return ret;
}

static ssize_t sprd_otp_write(struct file *file, const char __user * buf,
			      size_t count, loff_t * f_pos)
{
	ssize_t ret;
	int pos = (int)*f_pos;
	int minor = iminor(file_inode(file));
	struct sprd_otp_device *otp = sprd_otp_find(minor);

	pr_info("otp write to %d %d\n", (int)*f_pos, (int)file->f_pos);
	mutex_lock(&sprd_otp_mtx);
	ret = simple_write_to_buffer(cached_otp, sizeof(cached_otp), f_pos,
				     buf, count);
	if (IS_ERR_VALUE(ret))
		goto out;

	if (otp && otp->ops && otp->ops->prog) {
		int blk_index = pos / otp->blk_width;
		int index = blk_index * otp->blk_width;
		u32 width, val = 0;

		pr_info("otp prog blk %d\n", blk_index);
		WARN_ON(count != otp->blk_width);
		width = otp->blk_width > sizeof(u32) ? sizeof(u32) :
			otp->blk_width;
		memcpy(&val, &cached_otp[index], width);
		otp->ops->prog(blk_index, val);
	}
out:
	mutex_unlock(&sprd_otp_mtx);
	return ret;
}

static loff_t sprd_otp_llseek(struct file *file, loff_t offset, int whence)
{
	pr_info("otp seek at %d %d\n", (int)offset, whence);

	switch (whence) {
	case SEEK_SET:
		file->f_pos = offset;
		break;
	case SEEK_CUR:
		file->f_pos += offset;
		break;
	default:
		return -EINVAL;
	}
	return file->f_pos;
}

static const struct file_operations sprd_otp_fops = {
	.owner = THIS_MODULE,
	.read = sprd_otp_read,
	.write = sprd_otp_write,
	.llseek = sprd_otp_llseek,
};

static struct miscdevice sprd_otp = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sprd_otp",
	.fops = &sprd_otp_fops,
};

struct device *sprd_otp_register(const char *name, void *ops, int blk_max,
				 int blk_width)
{
	int ret;
	struct sprd_otp_device *otp;

	otp = kzalloc(sizeof(struct sprd_otp_device), GFP_KERNEL);
	if (unlikely(!otp))
		return ERR_PTR(-ENOMEM);
	INIT_LIST_HEAD(&otp->list);
	otp->misc.minor = MISC_DYNAMIC_MINOR;
	otp->misc.name = kstrdup(name, GFP_KERNEL);
	otp->misc.fops = &sprd_otp_fops;
	otp->blk_max = blk_max;
	otp->blk_width = blk_width;
	WARN_ON(otp->blk_width > sizeof(u32));
	WARN_ON(otp->blk_max * otp->blk_width > sizeof(cached_otp));
	otp->ops = ops;
	ret = misc_register(&otp->misc);
	if (ret)
		return ERR_PTR(ret);

	mutex_lock(&sprd_otp_mtx);
	list_add(&otp->list, &sprd_otp_list);
	mutex_unlock(&sprd_otp_mtx);
	pr_info("otp register %s as miscdevice %d, block %uX%ubits\n",
		otp->misc.name, otp->misc.minor, otp->blk_max,
		otp->blk_width * 8);
	return otp->misc.this_device;
}

static int __init sprd_otp_init(void)
{
	int ret;

	ret = misc_register(&sprd_otp);
	return ret;
}

static void __exit sprd_otp_exit(void)
{
	struct sprd_otp_device *otp, *tmp;

	misc_deregister(&sprd_otp);
	list_for_each_entry_safe(otp, tmp, &sprd_otp_list, list) {
		list_del(&otp->list);
		kfree(otp);
	}
}

module_init(sprd_otp_init);
module_exit(sprd_otp_exit);

MODULE_DESCRIPTION("Spreadtrum OTP  Driver");
MODULE_AUTHOR("Freeman Liu <freeman.liu@spreadtrum.com>");;
MODULE_LICENSE("GPL");
