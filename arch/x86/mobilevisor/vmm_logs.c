/*
 * Copyright (c) 2011-2015 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <asm/mv/logging/mv_svc_log.h>


#define MAX_OS_BUF	(16*1024)

struct os_logger_info {
	uint64_t os_buf_addr;
	int buf_len;
	int os_buf_pos;
	int log_filter;
	int is_connected;
};

static struct os_logger_info *os_logger;

static struct dentry *logs_dir;

static int connect_vmm_logs(int log_filter)
{
	int ret;

	if (os_logger) {
		pr_err("%s: pls disable logger first!\n", __func__);
		return -EINVAL;
	}

	/* alloc memory under 4G address */
	os_logger = kmalloc(MAX_OS_BUF, GFP_ATOMIC | GFP_DMA);
	if (!os_logger)
		return -ENOMEM;

	memset((void *)os_logger, 0, MAX_OS_BUF);
	os_logger->buf_len = MAX_OS_BUF - sizeof(struct os_logger_info);
	os_logger->os_buf_addr = __pa(os_logger + 1);
	os_logger->log_filter = log_filter;
	pr_info("%s: connect vmm logs: os_logger=%p\n", __func__, os_logger);

	ret = mv_svc_vmm_logs_connect(__pa(os_logger));
	if (ret) {
		pr_err("%s: failed to connect vmm logs\n", __func__);
		return ret;
	}

	return 0;
}

static int disconnect_vmm_logs(void)
{
	int ret;

	if (!os_logger) {
		pr_err("%s: already distroied os buffer!\n", __func__);
		return -EINVAL;
	}

	ret = mv_svc_vmm_logs_disconnect(0);
	if (ret) {
		pr_err("%s: failed to disconnect vmm logs!\n", __func__);
		return ret;
	}

	kfree(os_logger);
	os_logger = NULL;
	return 0;
}

static ssize_t filter_show(struct file *f, char __user *user_buf,
		size_t cnt, loff_t *pos)
{
	char buff[64];
	size_t len;
	char *help = "0 -- disable\n0x1 -- vmm\n0x4 -- linux\n0x8 -- secvm\n";

	len = sprintf(buff, "current filter 0x%x\n\n%s\n",
		os_logger ? os_logger->log_filter : 0, help);
	return simple_read_from_buffer(user_buf, cnt, pos, buff, len);
}

static ssize_t filter_write(struct file *filep,
		const char __user *user_buf, size_t size, loff_t *pos)
{
	int val, ret;

	ret = kstrtoint_from_user(user_buf, size, 0, &val);
	if (ret)
		return ret;

	if (val)
		connect_vmm_logs(val);
	else
		disconnect_vmm_logs();

	return size;
}

static const struct file_operations filter_ops = {
	.open = simple_open,
	.write = filter_write,
	.read = filter_show,
	.owner = THIS_MODULE,
};

static ssize_t demo_write(struct file *filep,
		const char __user *user_buf, size_t size, loff_t *pos)
{
	char *buff;
	int ret;

	size = size > 32 ? 32 : size;
	/* alloc memory under 4G address */
	buff = kmalloc(size, GFP_ATOMIC | GFP_DMA);
	if (!buff)
		return -ENOMEM;

	if (copy_from_user(buff, user_buf, size)) {
		kfree(buff);
		return -EFAULT;
	}
	ret = mv_svc_vmm_logs_print(buff, size);
	kfree(buff);
	return size;
}

static const struct file_operations demo_ops = {
	.open = simple_open,
	.write = demo_write,
	.owner = THIS_MODULE,
};

static ssize_t buffer_show(struct file *f, char __user *user_buf,
		size_t cnt, loff_t *pos)
{
	char *pbuff = (char *)(os_logger + 1);

	if (!os_logger) {
		pr_err("Please enable buffer first!\n");
		return 0;
	}

	return simple_read_from_buffer(
		user_buf, cnt, pos, pbuff, os_logger->os_buf_pos);
}

static const struct file_operations buffer_ops = {
	.open = simple_open,
	.read = buffer_show,
	.owner = THIS_MODULE,
};

static ssize_t level_write(struct file *filep,
		const char __user *user_buf, size_t size, loff_t *pos)
{
	int level;
	int ret;

	ret = kstrtoint_from_user(user_buf, size, 0, &level);
	if (ret)
		return ret;

	mv_svc_vmm_logs_level(level & 0xff, (level >> 8) & 0xff);

	return size;
}

static const struct file_operations level_ops = {
	.open = simple_open,
	.write = level_write,
	.owner = THIS_MODULE,
};

static int __init vmm_logs_init(void)
{
	struct dentry *filter, *buffer;
	struct dentry *demo;
	struct dentry *level;

	if (mv_svc_vmm_logs_is_initialized()) {
		logs_dir = debugfs_create_dir("vmm_logs", NULL);
		if (IS_ERR_OR_NULL(logs_dir))
			return -EFAULT;

		filter = debugfs_create_file("filter",
			(S_IRUGO|S_IWUSR|S_IWGRP),
			logs_dir, NULL, &filter_ops);
		if (IS_ERR_OR_NULL(filter))
			goto __out;

		buffer = debugfs_create_file("buffer", (S_IRUGO),
			logs_dir, NULL, &buffer_ops);
		if (IS_ERR_OR_NULL(buffer))
			goto __out;

		demo = debugfs_create_file("demo", (S_IWUSR|S_IWGRP),
			logs_dir, NULL, &demo_ops);

		if (IS_ERR_OR_NULL(demo))
			goto __out;

		level = debugfs_create_file("level", (S_IWUSR|S_IWGRP),
			logs_dir, NULL, &level_ops);

		if (IS_ERR_OR_NULL(level))
			goto __out;

		connect_vmm_logs(1);

#ifndef CONFIG_MOBILEVISOR_DEBUG    /* user version */
		mv_svc_vmm_logs_level(VMM_EMERG_VALUE, VMM_ERR_VALUE);
#endif
	}
	return 0;
__out:
	debugfs_remove_recursive(logs_dir);
	return 0;
}

static void __exit vmm_logs_exit(void)
{
	if (!IS_ERR_OR_NULL(logs_dir))
		debugfs_remove_recursive(logs_dir);
}

module_init(vmm_logs_init);
module_exit(vmm_logs_exit);
MODULE_LICENSE("GPL");
