/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <asm/pgtable.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif
#include <soc/sprd/hardware.h>
#include <linux/sprd_cproc.h>
#include <linux/sipc.h>
#include <soc/sprd/sci.h>
#include <soc/sprd/sci_glb_regs.h>
//#include <soc/sprd/arch_misc.h>
#define CPROC_WDT_TRUE   1
#define CPROC_WDT_FLASE  0
/*used for ioremap to limit vmalloc size, shi yunlong*/
#define CPROC_VMALLOC_SIZE_LIMIT 4096

enum {
	CP_NORMAL_STATUS=0,
	CP_STOP_STATUS,
	CP_WDTIRQ_STATUS,
	CP_MAX_STATUS,
};

const char *cp_status_info[] = {
	"started\n",
	"stopped\n",
	"wdtirq\n",
};

struct cproc_proc_fs;

struct cproc_proc_entry {
	char				*name;
	struct proc_dir_entry	*entry;
	struct cproc_device		*cproc;
};

struct cproc_proc_fs {
	struct proc_dir_entry		*procdir;

	struct cproc_proc_entry		start;
	struct cproc_proc_entry		stop;
	struct cproc_proc_entry		modem;
	struct cproc_proc_entry		dsp[2];
	struct cproc_proc_entry		status;
	struct cproc_proc_entry		wdtirq;
	struct cproc_proc_entry		mem;
};

struct cproc_device {
	struct miscdevice		miscdev;
	struct cproc_init_data	*initdata;
	void				*vbase;
	int 				wdtirq;
	int				wdtcnt;
	wait_queue_head_t		wdtwait;
	char *				name;
	int					status;
	struct cproc_proc_fs		procfs;
};

static int sprd_cproc_open(struct inode *inode, struct file *filp)
{
	struct cproc_device *cproc = container_of(filp->private_data,
			struct cproc_device, miscdev);

	filp->private_data = cproc;

	pr_info("cproc %s opened!\n", cproc->initdata->devname);

	return 0;
}

static int sprd_cproc_release (struct inode *inode, struct file *filp)
{
	struct cproc_device *cproc = filp->private_data;

	pr_info("cproc %s closed!\n", cproc->initdata->devname);

	return 0;
}

static long sprd_cproc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/*
	struct cproc_device *cproc = filp->private_data;
	*/
	/* TODO: for general modem download&control */

	return 0;
}

static int sprd_cproc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/*
	struct cproc_device *cproc = filp->private_data;
	*/
	/* TODO: for general modem download&control */

	return 0;
}

static const struct file_operations sprd_cproc_fops = {
	.owner = THIS_MODULE,
	.open = sprd_cproc_open,
	.release = sprd_cproc_release,
	.unlocked_ioctl = sprd_cproc_ioctl,
	.mmap = sprd_cproc_mmap,
};

static int cproc_proc_open(struct inode *inode, struct file *filp)
{
	struct cproc_proc_entry *entry = (struct cproc_proc_entry *)PDE_DATA(inode);
	/*move remap to writing or reading, shi yunlong*/
/*
	struct cproc_device *cproc = entry->cproc;

	cproc->vbase = ioremap(cproc->initdata->base, cproc->initdata->maxsz);
	if (!cproc->vbase) {
		printk(KERN_ERR "Unable to map cproc base: 0x%08x\n", cproc->initdata->base);
		return -ENOMEM;
	}
*/

	filp->private_data = entry;

	return 0;
}

static int cproc_proc_release(struct inode *inode, struct file *filp)
{
	/*move remap to writing or reading, shi yunlong*/
/*
	struct cproc_proc_entry *entry = (struct cproc_proc_entry *)filp->private_data;
	struct cproc_device *cproc = entry->cproc;

	iounmap(cproc->vbase);
*/

	return 0;
}

static ssize_t cproc_proc_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	struct cproc_proc_entry *entry = (struct cproc_proc_entry *)filp->private_data;
	struct cproc_device *cproc = entry->cproc;
	char *type = entry->name;
	unsigned int len;
	void *vmem;
	int rval;
	size_t r, i;/*count ioremap, shi yunlong*/

/*	pr_info("cproc proc read type: %s ppos %ll\n", type, *ppos);*/

	if (strcmp(type, "mem") == 0) {
		if (*ppos >= cproc->initdata->maxsz) {
			return 0;
		}
		if ((*ppos + count) > cproc->initdata->maxsz) {
			count = cproc->initdata->maxsz - *ppos;
		}

		/*remap and unmap in each read operation, shi yunlong, begin*/
		/*
		vmem = cproc->vbase + *ppos;
		if (unalign_copy_to_user(buf, vmem, count)) {
			printk(KERN_ERR "cproc_proc_read copy data to user error !\n");
			return -EFAULT;
		}
		*/
		r = count, i = 0;
		do{
			uint32_t copy_size = CPROC_VMALLOC_SIZE_LIMIT;
			vmem = ioremap_nocache(cproc->initdata->base + *ppos + CPROC_VMALLOC_SIZE_LIMIT*i, CPROC_VMALLOC_SIZE_LIMIT);
			if (!vmem) {
				size_t addr = cproc->initdata->base + *ppos + CPROC_VMALLOC_SIZE_LIMIT*i;
				printk(KERN_ERR "Unable to map cproc base: 0x%lx\n", addr);
				if(i > 0){
					*ppos += CPROC_VMALLOC_SIZE_LIMIT*i;
					return CPROC_VMALLOC_SIZE_LIMIT*i;
				}else{
					return -ENOMEM;
				}
			}
			if(r < CPROC_VMALLOC_SIZE_LIMIT) copy_size = r;
			if (unalign_copy_to_user(buf, vmem, copy_size)) {
				printk(KERN_ERR "cproc_proc_read copy data to user error !\n");
				iounmap(vmem);
				return -EFAULT;
			}
			iounmap(vmem);
			r -= copy_size;
			i++;
		}while(r > 0);
		/*remap and unmap in each read operation, shi yunlong, end*/
	} else if (strcmp(type, "status") == 0) {
		if (cproc->status >= CP_MAX_STATUS) {
			return -EINVAL;
		}
		len = strlen(cp_status_info[cproc->status]);
		if (*ppos >= len) {
			return 0;
		}
		count = (len > count) ? count : len;
		if (unalign_copy_to_user(buf, cp_status_info[cproc->status], count)) {
			printk(KERN_ERR "cproc_proc_read copy data to user error !\n");
			return -EFAULT;
		}
	} else if (strcmp(type, "wdtirq") == 0) {
		/* wait forever */
		rval = wait_event_interruptible(cproc->wdtwait, cproc->wdtcnt  != CPROC_WDT_FLASE);
		if (rval < 0) {
			printk(KERN_ERR "cproc_proc_read wait interrupted error !\n");
		}
		len = strlen(cp_status_info[CP_WDTIRQ_STATUS]);
		if (*ppos >= len) {
			return 0;
		}
		count = (len > count) ? count : len;
		if (unalign_copy_to_user(buf, cp_status_info[CP_WDTIRQ_STATUS], count)) {
			printk(KERN_ERR "cproc_proc_read copy data to user error !\n");
			return -EFAULT;
		}
	} else {
		return -EINVAL;
	}

	*ppos += count;
	return count;
}

static ssize_t cproc_proc_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *ppos)
{
	struct cproc_proc_entry *entry = (struct cproc_proc_entry *)filp->private_data;
	struct cproc_device *cproc = entry->cproc;
	char *type = entry->name;
	uint32_t base, size, offset;
	void *vmem;
	size_t r, i;/*count ioremap, shi yunlong*/

	pr_debug("cproc proc write type: %s\n!", type);

	if (strcmp(type, "start") == 0) {
		printk(KERN_INFO "cproc_proc_write to map cproc base start\n");
		cproc->initdata->start(cproc);
		cproc->wdtcnt = CPROC_WDT_FLASE;
		cproc->status = CP_NORMAL_STATUS;
		return count;
	}
	if (strcmp(type, "stop") == 0) {
		printk(KERN_INFO "cproc_proc_write to map cproc base stop\n");
		cproc->initdata->stop(cproc);
		cproc->status = CP_STOP_STATUS;
		return count;
	}

	if (strcmp(type, "modem") == 0) {
		base = cproc->initdata->segs[0].base;
		size = cproc->initdata->segs[0].maxsz;
		offset = *ppos;
	}
        else if (strcmp(type, "dsp") == 0) {
		base = cproc->initdata->segs[1].base;
		size = cproc->initdata->segs[1].maxsz;
		offset = *ppos;
	}
        else if (strcmp(type, "tgdsp") == 0) {
		base = cproc->initdata->segs[1].base;
		size = cproc->initdata->segs[1].maxsz;
		offset = *ppos;
	}
        else if (strcmp(type, "ldsp") == 0) {
		base = cproc->initdata->segs[2].base;
		size = cproc->initdata->segs[2].maxsz;
		offset = *ppos;
	}
        else {
            return -EINVAL;
	}

	if (size <= offset) {
		pr_debug("cproc_proc_write modem dsp write over pos:%0x\n",offset);
		*ppos += count;
		return count;
	}

	pr_info("cproc proc write: 0x%08x, 0x%08x\n!", base + offset, count);
	count = min((size-offset), count);
	r = count, i = 0;
	do{
		uint32_t copy_size = CPROC_VMALLOC_SIZE_LIMIT;
		vmem = ioremap_nocache(base + offset + CPROC_VMALLOC_SIZE_LIMIT*i, CPROC_VMALLOC_SIZE_LIMIT);
		if (!vmem) {
			size_t addr = base + offset + CPROC_VMALLOC_SIZE_LIMIT*i;
			printk(KERN_ERR "Unable to map cproc base: 0x%lx\n", addr);
			if(i > 0){
				*ppos += CPROC_VMALLOC_SIZE_LIMIT*i;
				return CPROC_VMALLOC_SIZE_LIMIT*i;
			}else{
				return -ENOMEM;
			}
		}
		if(r < CPROC_VMALLOC_SIZE_LIMIT) copy_size = r;
		if (unalign_copy_from_user(vmem, buf+CPROC_VMALLOC_SIZE_LIMIT*i, copy_size)) {
			printk(KERN_ERR "cproc_proc_write copy data from user error !\n");
			iounmap(vmem);
			return -EFAULT;
		}
		iounmap(vmem);
		r -= copy_size;
		i++;
	}while(r > 0);
	/*remap and unmap in each write operation, shi yunlong, end*/

	*ppos += count;
	return count;
}

static loff_t cproc_proc_lseek(struct file* filp, loff_t off, int whence )
{
	struct cproc_proc_entry *entry = (struct cproc_proc_entry *)filp->private_data;
	struct cproc_device *cproc = entry->cproc;
	char *type = entry->name;
	loff_t new;

	switch (whence) {
	case SEEK_SET:
		new = off;
		filp->f_pos = new;
		break;
	case SEEK_CUR:
		new = filp->f_pos + off;
		filp->f_pos = new;
		break;
	case SEEK_END:
		if (strcmp(type, "mem") == 0) {
			new = cproc->initdata->maxsz + off;
			filp->f_pos = new;
		} else {
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	return (new);
}

static unsigned int cproc_proc_poll(struct file *filp, poll_table *wait)
{
	struct cproc_proc_entry *entry = (struct cproc_proc_entry *)filp->private_data;
	struct cproc_device *cproc = entry->cproc;
	char *type = entry->name;
	unsigned int mask = 0;

	pr_info("cproc proc poll type: %s \n", type);

	if (strcmp(type, "wdtirq") == 0) {
		poll_wait(filp, &cproc->wdtwait, wait);
		if (cproc->wdtcnt  != CPROC_WDT_FLASE) {
			mask |= POLLIN | POLLRDNORM;
		}
	} else {
		printk(KERN_ERR "cproc_proc_poll file don't support poll !\n");
		return -EINVAL;
	}
	return mask;
}

struct file_operations cpproc_fs_fops = {
	.open		= cproc_proc_open,
	.release	= cproc_proc_release,
	.llseek  = cproc_proc_lseek,
	.read		= cproc_proc_read,
	.write	= cproc_proc_write,
	.poll		= cproc_proc_poll,
};

static inline void sprd_cproc_fs_init(struct cproc_device *cproc)
{
        uint8_t i= 0;
	cproc->procfs.procdir = proc_mkdir(cproc->name, NULL);

	cproc->procfs.start.name = "start";
	cproc->procfs.start.entry = proc_create_data(cproc->procfs.start.name, S_IWUSR,
			cproc->procfs.procdir, &cpproc_fs_fops, &(cproc->procfs.start));
	cproc->procfs.start.cproc = cproc;

	cproc->procfs.stop.name = "stop";
	cproc->procfs.stop.entry = proc_create_data(cproc->procfs.stop.name, S_IWUSR,
			cproc->procfs.procdir, &cpproc_fs_fops, &(cproc->procfs.stop));
	cproc->procfs.stop.cproc = cproc;

	cproc->procfs.modem.name =cproc->initdata->segs[0].name;
	cproc->procfs.modem.entry = proc_create_data(cproc->procfs.modem.name, S_IWUSR,
			cproc->procfs.procdir, &cpproc_fs_fops, &(cproc->procfs.modem));
	cproc->procfs.modem.cproc = cproc;

       for(i = 0; i < cproc->initdata->segnr - 1; i++){
            cproc->procfs.dsp[i].name = cproc->initdata->segs[1+i].name;
            cproc->procfs.dsp[i].entry = proc_create_data(cproc->procfs.dsp[i].name, S_IWUSR,
                            cproc->procfs.procdir, &cpproc_fs_fops, &(cproc->procfs.dsp[i]));
            cproc->procfs.dsp[i].cproc = cproc;
        }

	cproc->procfs.status.name = "status";
	cproc->procfs.status.entry = proc_create_data(cproc->procfs.status.name, S_IWUSR,
			cproc->procfs.procdir, &cpproc_fs_fops, &(cproc->procfs.status));
	cproc->procfs.status.cproc = cproc;

	cproc->procfs.wdtirq.name = "wdtirq";
	cproc->procfs.wdtirq.entry = proc_create_data(cproc->procfs.wdtirq.name, S_IWUSR,
			cproc->procfs.procdir, &cpproc_fs_fops, &(cproc->procfs.wdtirq));
	cproc->procfs.wdtirq.cproc = cproc;

	cproc->procfs.mem.name = "mem";
	cproc->procfs.mem.entry = proc_create_data(cproc->procfs.mem.name, S_IWUSR,
			cproc->procfs.procdir, &cpproc_fs_fops, &(cproc->procfs.mem));
	cproc->procfs.mem.cproc = cproc;
}

static inline void sprd_cproc_fs_exit(struct cproc_device *cproc)
{
        uint8_t i= 0;
	remove_proc_entry(cproc->procfs.start.name, cproc->procfs.procdir);
	remove_proc_entry(cproc->procfs.stop.name, cproc->procfs.procdir);
	remove_proc_entry(cproc->procfs.modem.name, cproc->procfs.procdir);
        for(i = 0; i < cproc->initdata->segnr - 1; i++){
            remove_proc_entry(cproc->procfs.dsp[i].name, cproc->procfs.procdir);
        }
	remove_proc_entry(cproc->procfs.dsp[1].name, cproc->procfs.procdir);
	remove_proc_entry(cproc->procfs.status.name, cproc->procfs.procdir);
	remove_proc_entry(cproc->procfs.wdtirq.name, cproc->procfs.procdir);
	remove_proc_entry(cproc->procfs.mem.name, cproc->procfs.procdir);
	remove_proc_entry(cproc->name, NULL);
}

static irqreturn_t sprd_cproc_irq_handler(int irq, void *dev_id)
{
	struct cproc_device *cproc = (struct cproc_device *)dev_id;

	printk("sprd_cproc_irq_handler cp watchdog enable !\n");
	cproc->wdtcnt = CPROC_WDT_TRUE;
	cproc->status = CP_WDTIRQ_STATUS;
	wake_up_interruptible_all(&(cproc->wdtwait));
	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static int sprd_cproc_native_cp_start(void* arg)
{
	struct cproc_device *cproc = (struct cproc_device *)arg;
	struct cproc_init_data *pdata = cproc->initdata;
	struct cproc_ctrl *ctrl;
	uint32_t value, state;
        void *vmem = NULL;

        if (!pdata) {
            return -ENODEV;
	}
	ctrl = pdata->ctrl;

        vmem = ioremap_nocache(ctrl->iram_addr, sizeof(ctrl->iram_data));
        if(!vmem)
            return -ENOMEM;

	unalign_memcpy(vmem, (void *)ctrl->iram_data, sizeof(ctrl->iram_data));

	/* clear cp1 force shutdown */
        if(ctrl->ctrl_reg[CPROC_CTRL_SHUT_DOWN] != INVALID_REG){
#if !defined(CONFIG_ARCH_SCX30G)//sharkl & sharkl64 branch
              sci_glb_clr(ctrl->ctrl_reg[CPROC_CTRL_SHUT_DOWN], 0x100000);
#else  //tshark branch
              sci_glb_clr(ctrl->ctrl_reg[CPROC_CTRL_SHUT_DOWN], ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN]);
#endif
        }
#if !defined(CONFIG_ARCH_SCX30G) && !defined(CONFIG_ARCH_SCX35)
        while(1)
	{
		state = __raw_readl((void *)ctrl->ctrl_reg[CPROC_CTRL_GET_STATUS]);
		if (!(state & ctrl->ctrl_mask[CPROC_CTRL_GET_STATUS]))  //(0xf <<16)
			break;
	}
#endif
        if(ctrl->ctrl_reg[CPROC_CTRL_DEEP_SLEEP] != INVALID_REG){
            /* clear cp1 force deep sleep */
             sci_glb_clr(ctrl->ctrl_reg[CPROC_CTRL_DEEP_SLEEP], ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP]);
        }

        if(ctrl->ctrl_reg[CPROC_CTRL_RESET] != INVALID_REG){
             /* clear reset cp1 */
            msleep(50);
            sci_glb_clr(ctrl->ctrl_reg[CPROC_CTRL_RESET],ctrl->ctrl_mask[CPROC_CTRL_RESET]);
            while(1) {
                state = sci_glb_read(ctrl->ctrl_reg[CPROC_CTRL_RESET],-1UL);
                if (!(state & ctrl->ctrl_mask[CPROC_CTRL_RESET]))
                    break;
            }
        }
        iounmap(vmem);
	pr_info("sprd_cproc:start over\n");
	return 0;
}

static int sprd_cproc_native_cp_stop(void *arg)
{
	struct cproc_device *cproc = (struct cproc_device *)arg;
	struct cproc_init_data *pdata = cproc->initdata;
	struct cproc_ctrl *ctrl;
	uint32_t value;

        if (!pdata) {
            return -ENODEV;
	}
	ctrl = pdata->ctrl;

            /* reset cp1 */
        if((ctrl->ctrl_reg[CPROC_CTRL_RESET] & INVALID_REG)!= INVALID_REG){
            sci_glb_set(ctrl->ctrl_reg[CPROC_CTRL_RESET],ctrl->ctrl_mask[CPROC_CTRL_RESET]);
            pr_info("sprd_cproc: stop:read reset=%x\n",value);
            msleep(50);
        }
        if((ctrl->ctrl_reg[CPROC_CTRL_DEEP_SLEEP] & INVALID_REG)!= INVALID_REG){
            /* cp1 force deep sleep */
            sci_glb_set(ctrl->ctrl_reg[CPROC_CTRL_DEEP_SLEEP],ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP]);
            msleep(50);
        }
        if((ctrl->ctrl_reg[CPROC_CTRL_GET_STATUS] & INVALID_REG)!= INVALID_REG){
           sci_glb_clr(ctrl->ctrl_reg[CPROC_CTRL_GET_STATUS],ctrl->ctrl_mask[CPROC_CTRL_GET_STATUS]);
       }
        if((ctrl->ctrl_reg[CPROC_CTRL_SHUT_DOWN] & INVALID_REG)!= INVALID_REG){
            /* cp1 force shutdown */
            sci_glb_set(ctrl->ctrl_reg[CPROC_CTRL_SHUT_DOWN],ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN]);
        }
        return 0;
}

static int sprd_cproc_native_cp2_start(void* arg)
{
	struct cproc_device *cproc = (struct cproc_device *)arg;
	struct cproc_init_data *pdata = cproc->initdata;
	struct cproc_ctrl *ctrl;
	uint32_t state;
        void * cp2_code_addr = NULL;

        if (!pdata) {
            return -ENODEV;
	}
	pr_info("%s\n",__func__);

	ctrl = pdata->ctrl;
	cp2_code_addr = ioremap_nocache(ctrl->iram_addr,0x1000);
        if(!cp2_code_addr)
            return -ENOMEM;


	unalign_memcpy(cp2_code_addr, (void *)ctrl->iram_data, sizeof(ctrl->iram_data));

#ifdef CONFIG_ARCH_SCX30G
	/* clear cp2 force shutdown */
	sci_glb_clr(ctrl->ctrl_reg[CPROC_CTRL_SHUT_DOWN], ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN]);
	msleep(5);

	/* set reset cp2 */
	sci_glb_set(ctrl->ctrl_reg[CPROC_CTRL_RESET], ctrl->ctrl_mask[CPROC_CTRL_RESET]);
	msleep(5);

	/* clear cp2 force deep sleep */
	sci_glb_clr(ctrl->ctrl_reg[CPROC_CTRL_DEEP_SLEEP], ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP]);
	msleep(5);

	/* clear reset cp2 */
	sci_glb_clr(ctrl->ctrl_reg[CPROC_CTRL_RESET], ctrl->ctrl_mask[CPROC_CTRL_RESET]);

	while(1)
	{

		state = sci_glb_read(ctrl->ctrl_reg[CPROC_CTRL_GET_STATUS],-1UL);
		if (!(state & ctrl->ctrl_mask[CPROC_CTRL_GET_STATUS]))	//(0xf <<16)
			break;
	}
#else
	/* clear cp2 force shutdown */
	sci_glb_clr(ctrl->ctrl_reg[CPROC_CTRL_SHUT_DOWN], ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN]);
	while(1)
	{
		state = sci_glb_read(ctrl->ctrl_reg[CPROC_CTRL_GET_STATUS],-1UL);
		if (!(state & ctrl->ctrl_mask[CPROC_CTRL_GET_STATUS]))	//(0xf <<16)
			break;
	}

	/* set reset cp2 */
	sci_glb_set(ctrl->ctrl_reg[CPROC_CTRL_RESET], ctrl->ctrl_mask[CPROC_CTRL_RESET]);

	/* clear cp2 force deep sleep */
	sci_glb_clr(ctrl->ctrl_reg[CPROC_CTRL_DEEP_SLEEP], ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP]);

	/* clear reset cp2 */
	sci_glb_clr(ctrl->ctrl_reg[CPROC_CTRL_RESET], ctrl->ctrl_mask[CPROC_CTRL_RESET]);

#endif
	iounmap(cp2_code_addr);

	return 0;
}

#define WCN_SLEEP_STATUS	(SPRD_PMU_BASE + 0xD4)

static int sprd_cproc_native_cp2_stop(void *arg)
{
	struct cproc_device *cproc = (struct cproc_device *)arg;
	struct cproc_init_data *pdata = cproc->initdata;
	struct cproc_ctrl *ctrl;
	uint32_t state = 0;

        if (!pdata) {
            return -ENODEV;
	}

	pr_info("%s\n",__func__);

	ctrl = pdata->ctrl;

	 while(1)
	 {
		state = sci_glb_read(WCN_SLEEP_STATUS,-1UL);
		if (!(state & (0xf<<12)))
			break;
		msleep(1);
	 }
	pr_info("%s cp2 enter sleep\n",__func__);


	/* reset cp2 */
	sci_glb_set(ctrl->ctrl_reg[CPROC_CTRL_RESET], ctrl->ctrl_mask[CPROC_CTRL_RESET]);

	/* cp2 force deep sleep */
	sci_glb_set(ctrl->ctrl_reg[CPROC_CTRL_DEEP_SLEEP], ctrl->ctrl_mask[CPROC_CTRL_DEEP_SLEEP]);

	/* cp2 force shutdown */
	sci_glb_set(ctrl->ctrl_reg[CPROC_CTRL_SHUT_DOWN], ctrl->ctrl_mask[CPROC_CTRL_SHUT_DOWN]);
	return 0;
}

#endif

static int sprd_cproc_parse_dt(struct cproc_init_data **init, struct device *dev)
{
#ifdef CONFIG_OF
	struct cproc_init_data *pdata;
	struct cproc_ctrl *ctrl;
	struct resource res;
	struct device_node *np = dev->of_node, *chd;
	int ret, i, segnr;
	uint32_t base, offset;
        uint32_t ctrl_reg[4] = {0};
	size_t reg_base[6] = {0};

	segnr = of_get_child_count(np);
	pr_info("sprd_cproc mem size: %u\n", sizeof(struct cproc_init_data) + segnr * sizeof(struct cproc_segments));

	pdata = kzalloc(sizeof(struct cproc_init_data) + segnr * sizeof(struct cproc_segments), GFP_KERNEL);
	if (!pdata) {
		return -ENOMEM;
	}

	ctrl = kzalloc(sizeof(struct cproc_ctrl), GFP_KERNEL);
	if (!ctrl) {
		kfree(pdata);
		return -ENOMEM;
	}

	ret = of_property_read_string(np, "sprd,name", (const char **)&pdata->devname);
	if (ret)
	{
		goto error;
	}

	/* get pmu base addr */
        for(i=0;i < 6;i++){
	ret = of_address_to_resource(np, i, &res);
	if (ret) {
		ret = -ENODEV;
		goto error;
	}
	reg_base[i] = res.start;
	pr_info("sprd_cproc: base 0x%lx\n", reg_base[i]);
        }
	/* get ctrl_reg addr on pmu base */
	ret = of_property_read_u32_array(np, "sprd,ctrl-reg", ctrl_reg, CPROC_CTRL_NR);
	if (ret) {
		goto error;
	}
	for (i = 0; i < CPROC_CTRL_NR; i++) {
                pr_info("sprd_cproc before p2v: ctrl_reg[%d] = 0x%lx\n", i, ctrl_reg[i]);
                if(ctrl_reg[i] != INVALID_REG){
                    ctrl_reg[i] += reg_base[i+2];
                    ctrl->ctrl_reg[i] = SPRD_DEV_P2V(ctrl_reg[i]);
                    pr_info("sprd_cproc: ctrl_reg[%d] = 0x%lx\n", i, ctrl->ctrl_reg[i]);
                }
                else{
                    ctrl->ctrl_reg[i] = INVALID_REG;
                 }
	}

	/* get ctrl_mask */
	ret = of_property_read_u32_array(np, "sprd,ctrl-mask", (uint32_t *)ctrl->ctrl_mask, CPROC_CTRL_NR);
	if (ret) {
		goto error;
	}
	for (i = 0; i < CPROC_CTRL_NR; i++) {
		pr_info("sprd_cproc: ctrl_mask[%d] = 0x%08x\n", i, ctrl->ctrl_mask[i]);
	}

	/* get iram data */
	ret = of_property_read_u32_array(np, "sprd,iram-data", (uint32_t *)ctrl->iram_data, CPROC_IRAM_DATA_NR);
	if (ret) {
		goto error;
	}
	for (i = 0; i < CPROC_IRAM_DATA_NR; i++) {
		pr_info("sprd_cproc: iram-data[%d] = 0x%08x\n", i, ctrl->iram_data[i]);
	}

	/* get irq */
	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		goto error;
	}
	pdata->base = res.start;
	pdata->maxsz = res.end - res.start + 1;
	pr_info("sprd_cproc: cp base = 0x%lx, size = 0x%x\n", pdata->base, pdata->maxsz);

	/* get iram_base+offset */
        ret = of_address_to_resource(np, 1, &res);
	if (ret) {
		goto error;
	}
	ctrl->iram_addr = res.start;
	pr_info("sprd_cproc: iram_addr=0x%x\n", ctrl->iram_addr);

	/* get irq */
	pdata->wdtirq = irq_of_parse_and_map(np, 0);
	pr_info("sprd_cproc: wdt irq %u\n", pdata->wdtirq);

	i = 0;
	for_each_child_of_node(np, chd) {
		struct cproc_segments *seg;
		seg = &pdata->segs[i];
		ret = of_property_read_string(chd, "cproc,name", (const char **)&seg->name);
		if (ret) {
			goto error;
		}
		pr_info("sprd_cproc: child node [%d] name=%s\n", i, seg->name);
		/* get child base addr */
		ret = of_address_to_resource(chd, 0, &res);
		if (ret) {
			goto error;
		}
		seg->base = res.start;
		seg->maxsz = res.end - res.start + 1;
		pr_info("sprd_cproc: child node [%d] base=0x%x, size=0x%0x\n", i, seg->base, seg->maxsz);

		i++;
	}

	pr_info("sprd_cproc: stop callback 0x%x, start callback 0x%x\n",
		sprd_cproc_native_cp_stop, sprd_cproc_native_cp_start);
	pdata->segnr = segnr;

	if(segnr == 1){
		pdata->start = sprd_cproc_native_cp2_start;
		pdata->stop = sprd_cproc_native_cp2_stop;
	}
	else{
		pdata->start = sprd_cproc_native_cp_start;
		pdata->stop = sprd_cproc_native_cp_stop;
	}
	pdata->ctrl = ctrl;
	*init = pdata;
	return 0;
error:
	kfree(ctrl);
	kfree(pdata);
	return ret;
#else
	return -ENODEV;
#endif
}

static void sprd_cproc_destroy_pdata(struct cproc_init_data **init)
{
#ifdef CONFIG_OF
	struct cproc_init_data *pdata = *init;

	if (pdata) {
		if (pdata->ctrl) {
			kfree(pdata->ctrl);
		}
		kfree(pdata);
	}
	*init = NULL;
#else
	return;
#endif
}

static int sprd_cproc_probe(struct platform_device *pdev)
{
	struct cproc_device *cproc;
	struct cproc_init_data *pdata = pdev->dev.platform_data;
	int rval;

	if (!pdata && pdev->dev.of_node) {
		rval = sprd_cproc_parse_dt(&pdata, &pdev->dev);
		if (rval) {
			printk(KERN_ERR "failed to parse device tree!\n");
			return rval;
		}
	}
	pr_info("sprd_cproc: pdata=0x%x, of_node=0x%x\n",
		(uint32_t)pdata, (uint32_t)pdev->dev.of_node);

	cproc = kzalloc(sizeof(struct cproc_device), GFP_KERNEL);
	if (!cproc) {
		sprd_cproc_destroy_pdata(&pdata);
		printk(KERN_ERR "failed to allocate cproc device!\n");
		return -ENOMEM;
	}


	pr_info("%s %p %x\n", __func__, pdata->base, pdata->maxsz);
#if 0
	if ( pdata->base == WCN_START_ADDR)
		set_section_ro(__va(pdata->base), (WCN_TOTAL_SIZE & ~(SECTION_SIZE - 1)) >> SECTION_SHIFT);
	else {
		if (!(pdata->maxsz % SECTION_SIZE))
			set_section_ro(__va(pdata->base), pdata->maxsz >> SECTION_SHIFT);
		else
			printk("%s WARN can't be marked RO now\n", __func__);
	}
#endif
	cproc->initdata = pdata;

	cproc->miscdev.minor = MISC_DYNAMIC_MINOR;
	cproc->miscdev.name = cproc->initdata->devname;
	cproc->miscdev.fops = &sprd_cproc_fops;
	cproc->miscdev.parent = NULL;
	cproc->name = cproc->initdata->devname;
	rval = misc_register(&cproc->miscdev);
	if (rval) {
		sprd_cproc_destroy_pdata(&cproc->initdata);
		kfree(cproc);
		printk(KERN_ERR "failed to register sprd_cproc miscdev!\n");
		return rval;
	}

	cproc->status = CP_NORMAL_STATUS;
	cproc->wdtcnt = CPROC_WDT_FLASE;
	init_waitqueue_head(&(cproc->wdtwait));

	/* register IPI irq */
	if(cproc->initdata->wdtirq > 32){
		rval = request_irq(cproc->initdata->wdtirq, sprd_cproc_irq_handler,
				0, cproc->initdata->devname, cproc);
		if (rval != 0) {
			misc_deregister(&cproc->miscdev);
			sprd_cproc_destroy_pdata(&cproc->initdata);
			printk(KERN_ERR "Cproc failed to request irq %s: %d\n",
					cproc->initdata->devname, cproc->initdata->wdtirq);
			kfree(cproc);
			return rval;
		}
	}

	sprd_cproc_fs_init(cproc);

	platform_set_drvdata(pdev, cproc);

	printk(KERN_INFO "cproc %s probed!\n", cproc->initdata->devname);

	return 0;
}

static int sprd_cproc_remove(struct platform_device *pdev)
{
	struct cproc_device *cproc = platform_get_drvdata(pdev);

	sprd_cproc_fs_exit(cproc);
	misc_deregister(&cproc->miscdev);
	sprd_cproc_destroy_pdata(&cproc->initdata);

	printk(KERN_INFO "cproc %s removed!\n", cproc->initdata->devname);
	kfree(cproc);

	return 0;
}

static const struct of_device_id sprd_cproc_match_table[] = {
	{ .compatible = "sprd,scproc", },
	{},
};

static struct platform_driver sprd_cproc_driver = {
	.probe    = sprd_cproc_probe,
	.remove   = sprd_cproc_remove,
	.driver   = {
		.owner = THIS_MODULE,
		.name = "sprd_cproc",
		.of_match_table = sprd_cproc_match_table,
	},
};

static int __init sprd_cproc_init(void)
{

	if (platform_driver_register(&sprd_cproc_driver) != 0) {
		printk(KERN_ERR "sprd_cproc platform drv register Failed \n");
		return -1;
	}
	return 0;
}

static void __exit sprd_cproc_exit(void)
{
	platform_driver_unregister(&sprd_cproc_driver);
}

module_init(sprd_cproc_init);
module_exit(sprd_cproc_exit);

MODULE_DESCRIPTION("SPRD Communication Processor Driver");
MODULE_LICENSE("GPL");
