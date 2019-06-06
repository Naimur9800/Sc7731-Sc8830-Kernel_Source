/*
* Copyright (C) 2015 Intel Mobile Communications GmbH
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
*/

#include <linux/device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/kmsg_dump.h>
#include <linux/kthread.h>
#include <linux/delay.h>  /* for msleep*/
#include <asm/page.h>

#include <asm/mv/mv_gal.h>
#include <asm/mv/mv_svc_hypercalls.h>
#include "asm/mv/mv_hypercalls.h"
#include "asm/mv/mv_service_list.h"
#include <asm/mv/sys_exception/pal_sys_exception_types.h>
#include <asm/mv/sys_exception/mv_svc_sys_exception.h>
#include <asm/mv/coredump/pal_coredump_types.h>
#include <asm/mv/coredump/mv_svc_coredump.h>
#include <asm/mv/mobilevisor.h>

#include "vdump.h"
#include "vdump_util.h"

#define VDUMP_MODULE_NAME "vdump"

#if 1
#define VDUMP_DEBUG
#endif

#define VD_LOG(format, args...)	printk(VDUMP_MODULE_NAME": " format, ## args)

#ifdef VDUMP_DEBUG
#define VD_DEBUG(format, args...) printk(VDUMP_MODULE_NAME": " format, ## args)
#else
#define VD_DEBUG(format, args...)
#endif

#define VDUMP_BUF_LEN 8196


/*
 * This is a global struct for storing data use by vdump
 */
struct vdump_data {
	/* Used for registering chardevs */
	int major;
	struct class *vd_class;
	char buffer[VDUMP_BUF_LEN];
	struct cd_sharemem cd_info;
};

static struct vdump_data vdump_data;

static int vdump_control_open(struct inode *p_inode, struct file *p_file);
static int vdump_control_release(struct inode *p_inode, struct file *p_file);
static ssize_t vdump_control_write(struct file *p_file,
		const char __user *user_buffer, size_t count, loff_t *position);
static ssize_t vdump_control_read(struct file *file_ptr,
		char __user *user_buffer, size_t count, loff_t *position);

static const struct file_operations vdump_control_fops = {
	.read = vdump_control_read,
	.write = vdump_control_write,
	.open = vdump_control_open,
	.release = vdump_control_release
};

static void vdump_panic(struct kmsg_dumper *dumper,
		enum kmsg_dump_reason reason);

static struct kmsg_dumper vdumper = {
	.dump = vdump_panic,
};

static int vdump_get_shmem_config(void);
static int vdump_init_sharemem(void);
static void vdump_init_region(void);

struct vdump_vmm_settings {
	int vdump_enable;
	int shmem_enable;
	struct cd_config cd_config_linux;
};
static struct vdump_vmm_settings vdump_vmm_setting;

static void vdump_setting_init(void)
{
	memset(&vdump_vmm_setting, 0, sizeof(vdump_vmm_setting));
}

static int vdump_get_shmem_config(void)
{
	return vdump_vmm_setting.shmem_enable;
}

static int vdump_control_open(struct inode *p_inode, struct file *p_file)
{
	VD_LOG("vdump_control_open\n");
	return 0;
}

static int vdump_control_release(struct inode *p_inode, struct file *p_file)
{
	VD_LOG("vdump_control_release\n");
	return 0;
}

static ssize_t vdump_control_write(struct file *p_file,
		const char __user *user_buffer, size_t count, loff_t *position)
{
	VD_LOG("vdump_control_write\n");
	return 0;
}

static ssize_t vdump_control_read(struct file *file_ptr,
		char __user *user_buffer, size_t count, loff_t *position)
{
	VD_LOG("vdump_control_read\n");
	return 0;
}

static int vdump_init(void)
{
	struct device *vdump_dev;

	VD_LOG("init\n");

	memset(&vdump_data, 0, sizeof(vdump_data));
	vdump_data.major = register_chrdev(0, VDUMP_MODULE_NAME,
			&vdump_control_fops);
	if (vdump_data.major < 0) {
		VD_LOG("cannot register dev errorcode=%i\n", vdump_data.major);
		return -1;
	}

	vdump_data.vd_class = class_create(THIS_MODULE, VDUMP_MODULE_NAME);
	if (IS_ERR(vdump_data.vd_class)) {
		unregister_chrdev(vdump_data.major, VDUMP_MODULE_NAME);
		VD_LOG("error creating class\n");
		return -1;
	}

	vdump_dev = device_create(vdump_data.vd_class, NULL,
			MKDEV(vdump_data.major, 0), NULL, VDUMP_MODULE_NAME);

	if (IS_ERR(vdump_dev)) {
		class_destroy(vdump_data.vd_class);
		unregister_chrdev(vdump_data.major, VDUMP_MODULE_NAME);
		VD_LOG("error creating device\n");
		return -1;
	}

	vdump_setting_init();

	vdump_init_region();

	vdump_init_sharemem();

	if (vdump_data.cd_info.status) {
		int i;
		/* map the coredump memory regions */
		for (i = 0;
			i < vdump_data.cd_info.number_of_ranges; i++) {
			vdump_data.cd_info.
				memory_range[i].logical_start =
				(uintptr_t)ioremap_cache(
					vdump_data.cd_info.
					memory_range[i].physical_start,
					vdump_data.
					cd_info.memory_range[i].
					length);
		}
		/* unmap the coredump memory regions */
		for (i = 0;
			i < vdump_data.cd_info.number_of_ranges;
			i++) {
			iounmap((void *)(uintptr_t)vdump_data.cd_info
				.memory_range[i].logical_start);
			vdump_data.cd_info.memory_range[i]
				.logical_start = 0;
		}
		vdump_data.cd_info.status = 0;
	}

	/* register linux kernal space for dumping */
	kmsg_dump_register(&vdumper);

	return 0;
}

/*
 * callback from kmsg_dump. (s2,l2) has the most recently
 * written bytes, older bytes are in (s1,l1).
 */
static struct sys_trap trap_data;

static void vdump_panic(struct kmsg_dumper *dumper,
		enum kmsg_dump_reason reason)
{
	struct mv_shared_data *mv_shared_data;
	size_t len;

	if (reason != KMSG_DUMP_PANIC)
		return;

	/* Get share data */
	mv_shared_data = mv_gal_get_shared_data();

	/* Format trap data */
	memset(&trap_data, 0, sizeof(trap_data));
	trap_data.exception_type = (uint32_t)SYS_EXCEPTION_LINUX;

	kmsg_dump_get_buffer(dumper, true,
			vdump_data.buffer, VDUMP_BUF_LEN, &len);
	vdump_save_linux_regs((char *)&vdump_data.buffer,
			VDUMP_BUF_LEN, &trap_data.regs);
	trap_data.os.linux_log.kmsg = (uint64_t)__pa(vdump_data.buffer);
	trap_data.os.linux_log.kmsg_len = len;
	mv_svc_sys_exception(SYS_EXCEPTION_DUMP, (void *)__pa(&trap_data));

	/* Program should never return back here */
	unreachable();

	}
static int vdump_init_sharemem(void)
{
	if (vdump_get_shmem_config()) {
		/* Inform VMM of sharemem settings */
		mv_svc_cd_service(CD_CONFIG_SHAREMEM,
			(void *)__pa(&vdump_data.cd_info));
	}

	return 0;
}

static void vdump_init_region(void)
{
#if defined(CONFIG_MOBILEVISOR_32BIT_WA)
	struct cd_ram *p_cd_area;
	struct mv_shared_data *shmem;
#else
	struct cd_ram cd_area;
	struct cd_ram *p_cd_area;
#endif

	if (!is_x86_mobilevisor())
		return;

#if defined(CONFIG_MOBILEVISOR_32BIT_WA)
	shmem = mv_gal_get_shared_data();
	p_cd_area = (struct cd_ram *)&shmem->pal_shared_mem_data;
#else
	p_cd_area = &cd_area;
#endif

	/* Kernel code and data region */
#ifndef CONFIG_X86_32
	p_cd_area->logical_start = (uint32_t)(uintptr_t)_text;
	p_cd_area->physical_start = (uint32_t)__pa(_text);
	p_cd_area->length = (uint32_t)(uintptr_t)_end
		- (uint32_t)(uintptr_t)_text;
#else
	p_cd_area->logical_start = (uint32_t)(PAGE_OFFSET+PAGE_SIZE);
	p_cd_area->physical_start = (uint32_t)__pa((PAGE_OFFSET+PAGE_SIZE));
	p_cd_area->length = (uint32_t)(uintptr_t)_end
		- (uint32_t)(PAGE_OFFSET+PAGE_SIZE);
#endif
	mv_svc_cd_service(CD_ADD_REGION, (void *)__pa(p_cd_area));

	/* Entire low memory range */
	p_cd_area->logical_start = (uintptr_t)_end;
	p_cd_area->physical_start = (uintptr_t)__pa(_end);
#ifdef CONFIG_X86_32
	p_cd_area->length = (uint32_t)(uintptr_t)high_memory
		- (uint32_t)(uintptr_t)_end;
#else
	p_cd_area->length = (uint32_t)((max_pfn * PAGE_SIZE)
		+ __START_KERNEL_map) - (uint32_t)(uintptr_t)_end;
#endif
	mv_svc_cd_service(CD_ADD_REGION, (void *)__pa(p_cd_area));
}

fs_initcall(vdump_init);
