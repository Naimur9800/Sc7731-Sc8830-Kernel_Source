#include <linux/init.h>
#include <linux/kernel.h>
/*
 * on trusty board, uboot will pass the values of efuse
 * block0 and block1 to kernel by cmdline, parse logic
 * is as follows
 */

unsigned int g_val_block0;
unsigned int g_val_block1;

static __init int set_block0_val(char *arg)
{
	int ret;
	unsigned int val;

	ret = kstrtouint(arg, 10, &val);
	if (ret)
		return ret;

	g_val_block0 = val;
	printk(KERN_INFO "val of block0 from uboot:%d\n", g_val_block0);
	return 0;
}
early_param("block0", set_block0_val);

static __init int set_block1_val(char *arg)
{
	int ret;
	unsigned int val;

	ret = kstrtouint(arg, 10, &val);
	if (ret)
		return ret;

	g_val_block1 = val;
	printk(KERN_INFO "val of block1 from uboot:%d\n", g_val_block1);
	return 0;
}
early_param("block1", set_block1_val);
