#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/kthread.h>
#include <linux/linkage.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/plist.h>
#include <linux/rbtree.h>
#include <linux/sprd_map.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/compat.h>

#define MAP_USER_MINOR MISC_DYNAMIC_MINOR

struct pmem_info MemInfo;

#if IS_ENABLED(CONFIG_COMPAT)
#define COMPAT_MAP_USER_VIR  _IOWR(SPRD_MAP_IOCTRL_MAGIC, 0, \
				   struct compat_pmem_info)

struct compat_pmem_info {
	compat_ulong_t phy_addr;
	compat_uint_t phys_offset;
	compat_size_t size;
};
#endif

static int map_user_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int map_user_mmap(struct file *file, struct vm_area_struct *vma)
{
	if (MemInfo.phy_addr && MemInfo.size) {
		vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
		pr_info("%s: phy_addr=0x%lx, size=0x%zx vm_start=0x%lx\n",
			 __func__, MemInfo.phy_addr,
			 MemInfo.size, vma->vm_start);
		return vm_iomap_memory(vma, MemInfo.phy_addr, MemInfo.size);
	} else {
		pr_err("%s, MemInfo.phy_addr=0x%lx, MemInfo.size=0x%zx error!\n",
			   __func__, MemInfo.phy_addr, MemInfo.size);
		return -EINVAL;
	}
}

static long map_user_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	void __user *arg_user;

	arg_user = (void __user *)arg;
	switch (cmd) {
	case MAP_USER_VIR:
	{
		struct pmem_info data;

		if (copy_from_user(&data, arg_user, sizeof(data))) {
			pr_err("%s, PHYS copy_from_user error!\n", __func__);
			return -EFAULT;
		}

		MemInfo.phy_addr = data.phy_addr + MemInfo.phys_offset;
		MemInfo.size = data.size;
		pr_debug("%s: phy_addr=0x%lx, size=0x%zx\n", __func__,
				  data.phy_addr, data.size);
		break;
	}
	default:
		return -ENOTTY;

	}
	return ret;
}

#if IS_ENABLED(CONFIG_COMPAT)
static int compat_get_map_user_data(
			       struct compat_pmem_info __user *data32,
			       struct pmem_info  __user *data)
{
	compat_size_t s;
	compat_uint_t u;
	compat_ulong_t l;
	int err;

	err = get_user(l, &data32->phy_addr);
	err |= put_user(l, &data->phy_addr);
	err |= get_user(u, &data32->phys_offset);
	err |= put_user(u, &data->phys_offset);
	err |= get_user(s, &data32->size);
	err |= put_user(s, &data->size);

	return err;
}

long compat_map_user_ioctl(struct file *filp, unsigned int cmd,
		     unsigned long arg)
{
	long ret;

	if (!filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_MAP_USER_VIR:
	{
		struct compat_pmem_info __user *data32;
		struct pmem_info __user *data;
		int err;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_map_user_data(data32, data);
		if (err)
			return err;

		ret = filp->f_op->unlocked_ioctl(filp, MAP_USER_VIR,
						 (unsigned long)data);
		break;
	}
	default:
		ret = -ENOTTY;
	}
	return ret;
}
#else
#define compat_map_user_ioctl NULL
#endif

static const struct file_operations map_user_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = map_user_ioctl,
	.compat_ioctl = compat_map_user_ioctl,
	.mmap = map_user_mmap,
	.open = map_user_open,
};

static struct miscdevice map_user_dev = {
	.minor = MAP_USER_MINOR,
	.name = "map_user",
	.fops = &map_user_fops,
};

int map_user_probe(struct platform_device *pdev)
{
	int ret;

	ret = of_property_read_u32(pdev->dev.of_node, "phys-offset",
				   &(MemInfo.phys_offset));
	if (ret)
		MemInfo.phys_offset = 0;
	else
		pr_info("%s: phys_offset=0x%x\n",
			__func__, MemInfo.phys_offset);

	ret = misc_register(&map_user_dev);
	if (ret) {
		pr_err("%s: cannot register miscdev on minor=%d (%d)\n",
			   __func__, MAP_USER_MINOR, ret);
		misc_deregister(&map_user_dev);
	}
	return ret;
}

static int map_user_remove(struct platform_device *pdev)
{
	misc_deregister(&map_user_dev);
	pr_debug("map_user_remove Success!\n");
	return 0;
}

static const struct of_device_id of_match_table_map[] = {
	{ .compatible = "sprd,sprd_map_user", },
	{ },
};

static struct platform_driver map_user_driver = {
	.probe = map_user_probe,
	.remove = map_user_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "map_user",
		.of_match_table = of_match_ptr(of_match_table_map),
	}
};

static int __init map_user_init(void)
{
	if (platform_driver_register(&map_user_driver) != 0) {
		pr_err("%s: platform map user device register Failed\n",
			   __func__);
		return -1;
	}
	return 0;
}

static void map_user_exit(void)
{
	platform_driver_unregister(&map_user_driver);
}

module_init(map_user_init);
module_exit(map_user_exit);
MODULE_DESCRIPTION("map_user Driver");
MODULE_LICENSE("GPL");
