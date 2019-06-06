/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
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
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/sipc.h>
#include <linux/earlysuspend.h>


#define MAX_DDR_FREQ 640
#define MIN_DDR_FREQ 192
#define DDR_FREQ_BUF_LEN 4

static DEFINE_SPINLOCK(dfs_request_lock);
static unsigned int freq = 0;
int dfs_request(unsigned int freq)
{
	int err = 0;
	struct smsg msg;
	smsg_set(&msg, SMSG_CH_PM_CTRL, SMSG_TYPE_DFS, 0, freq);
	err = smsg_send(SIPC_ID_PM_SYS, &msg, 10);
	if (err < 0) {
		printk(KERN_ERR "%s, send freq(%d) failed\n",__func__,freq);
		return err;
	} else {
		// Wait for the response
		while (true) {
			err = smsg_recv(SIPC_ID_PM_SYS, &msg, 100);
			if (err < 0) {
				printk(KERN_ERR "%s, send freq(%d) failed\n",__func__,freq);
				return err;
			}
			if (SMSG_CH_PM_CTRL == msg.channel &&
			    SMSG_TYPE_DFS_RSP == msg.type) {
				break;
			}
		}
		if (msg.flag) {
			// Non zero flag indicates DFS not supported
			printk(KERN_WARNING "%s, DFS not supported\n",__func__);
			return -EINVAL;
		} else {
			// Zero flag indicates DFS supported
			if (msg.value) {
				// DFS failed
				printk(KERN_ERR "%s, dfs(%d) failed\n",__func__,freq);
				return -EBUSY;
			} else {
				// DFS succeeded
				printk("%s, dfs(%d) succeeded!\n",__func__,freq);
				return 0;
			}
		}
	}
}

static void scenario_dfs_early_suspend(struct early_suspend *h)
{
	spin_lock(&dfs_request_lock);
	freq = MIN_DDR_FREQ;
	spin_unlock(&dfs_request_lock);
	dfs_request(freq);
}

static void scenoario_dfs_late_resume(struct early_suspend *h)
{
	spin_lock(&dfs_request_lock);
	freq = MAX_DDR_FREQ;
	spin_unlock(&dfs_request_lock);
	dfs_request(freq);
}

static struct early_suspend scenario_dfs_early_suspend_desc = {
        .level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 100,
        .suspend = scenario_dfs_early_suspend,
};

static struct early_suspend scenoario_dfs_late_resume_desc = {
        .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
        .resume = scenoario_dfs_late_resume,
};

static int scenario_dfs_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int scenario_dfs_release(struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t scenario_dfs_write(struct file *file, const char __user *buf,
		size_t count, loff_t *offset)
{
	unsigned char freq_buf[DDR_FREQ_BUF_LEN] = { 0 };
	if(count > DDR_FREQ_BUF_LEN){
		printk(KERN_ERR "%s, invalid freq parameter\n",__func__);
		return -EINVAL;
		}
	copy_from_user(freq_buf,buf,count);
	spin_lock(&dfs_request_lock);
	freq = (freq_buf[0]-'0')*100 + (freq_buf[1]-'0')*10 + (freq_buf[2]-'0');
	spin_unlock(&dfs_request_lock);
	if((freq > MAX_DDR_FREQ) || (freq < MIN_DDR_FREQ)){
		printk(KERN_ERR "%s, invalid freq value\n",__func__);
		return -EINVAL;
		}
	dfs_request(freq);
	return count;
}


static const struct file_operations scenario_dfs_fops = {
	.open           =  scenario_dfs_open,
	.write		=  scenario_dfs_write,
	.release        =  scenario_dfs_release,
	.owner          =  THIS_MODULE,
};

static struct miscdevice scenario_dfs_dev = {
	.minor =    MISC_DYNAMIC_MINOR,
	.name  =    "sprd_scenario_dfs",
	.fops  =    &scenario_dfs_fops
};

static int __init scenario_dfs_init(void)
{
	int err;
	err = smsg_ch_open(SIPC_ID_PM_SYS, SMSG_CH_PM_CTRL, -1);
	if(err < 0){
		printk(KERN_ERR "%s, open sipc channel failed\n",__func__);
		return err;
		}
	register_early_suspend(&scenario_dfs_early_suspend_desc);
	register_early_suspend(&scenoario_dfs_late_resume_desc);
	return misc_register(&scenario_dfs_dev);
}

late_initcall(scenario_dfs_init);

static void __exit scenario_dfs_exit(void)
{
	unregister_early_suspend(&scenario_dfs_early_suspend_desc);
	unregister_early_suspend(&scenoario_dfs_late_resume_desc);
	misc_deregister(&scenario_dfs_dev);
}
module_exit(scenario_dfs_exit);

MODULE_LICENSE("GPL");

