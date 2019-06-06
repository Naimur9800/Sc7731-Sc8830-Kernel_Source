/*copyright (C) 2014 Spreadtrum Communications Inc.
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/nmi.h>
#include <linux/quicklist.h>
#include <asm/setup.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include <asm/ptrace.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include <linux/kallsyms.h>
#include <linux/ext2_fs.h>
#include <linux/debugfs.h>
#include <linux/mm.h>
#include <linux/stat.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <asm/memory.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif

static ssize_t dram_size = 0;
static struct mutex dmc_drv_lock;
#ifdef CONFIG_PROC_FS
static struct proc_dir_entry *sprd_dmc_proc_dir;
static int sprd_dmc_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%04d\n", dram_size/1024/1024);
	return 0;
}
static int sprd_dmc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sprd_dmc_proc_show, PDE_DATA(inode));
}
static const struct file_operations sprd_dmc_fops = {
	.open = sprd_dmc_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif
int dmc_proc_creat(void)
{
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *tmp_proc;
	sprd_dmc_proc_dir = proc_mkdir("sprd_dmc", NULL);
	if (!sprd_dmc_proc_dir)
		return -ENOMEM;
	tmp_proc = proc_create_data("property", S_IRWXUGO,
			     sprd_dmc_proc_dir, &sprd_dmc_fops, 0);
	if (!tmp_proc)
		return -ENOMEM;
#endif
	return 0;
}
typedef struct mem_cs_info
{
        u32 cs_number;
        u32 cs0_size;//bytes
        u32 cs1_size;//bytes
}mem_cs_info_t;
#define MAX_RESERVED_SIZE 0xFFFFF
#define MEM_INFO_BASE 0x1C00
static int __init sci_dmc_init(void)
{
	unsigned long base;

	mem_cs_info_t *dram_size_ptr;
	base = ioremap_nocache(0, 0x2000);
	if (!base) {
		printk("ioremap_nocache err %s\n", __func__);
		BUG();
	}
	mutex_init(&dmc_drv_lock);
	dram_size_ptr = (mem_cs_info_t *)(base + 0x1c00);
	dram_size = dram_size_ptr->cs0_size + dram_size_ptr->cs1_size;
	dram_size += MAX_RESERVED_SIZE;
	dram_size &= ~MAX_RESERVED_SIZE;
	iounmap(base);
	dmc_proc_creat();
	return 0;
}
static void __exit sci_dmc_exit(void)
{
	return;
}

module_init(sci_dmc_init);
module_exit(sci_dmc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("spreadtrum");
MODULE_DESCRIPTION("spreadtrum dmc drv");
