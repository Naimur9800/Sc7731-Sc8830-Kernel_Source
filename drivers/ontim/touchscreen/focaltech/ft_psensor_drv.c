#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>

#include <linux/input/touchscreen/focaltech/ft6x06_ts.h>
#include "ft_psensor_drv.h"

#define FT_RW_IIC_DRV  "ft6x06_ts"
#define init_MUTEX(sem)    		sema_init(sem, 1)

static int ft_psensor_drv_major = FT_PSENSOR_DRV_MAJOR;
struct ft_psensor_dev {
	struct cdev cdev;
	struct semaphore ft_rw_i2c_sem;
	struct i2c_client *client;
};
struct ft_psensor_dev * ft_psensor_dev_tt;
static struct class *fts_class;


static int ft_psensor_get_enable(struct i2c_client *client, unsigned long arg)
{
	int ret = 0;
	unsigned char buf = PSENSOR_ENABLE_REG;
	unsigned char en = 0x00;
	if (!access_ok(VERIFY_READ, (unsigned char *)arg, sizeof(unsigned char)))
		return -EFAULT;

	if (copy_from_user(&en,
		(unsigned char *)arg, 
		sizeof(unsigned char)))
		return -EFAULT;
	
	ret = ft6x06_i2c_Read(client, &buf, 1, &en, 1);
	if(ret<0) {
		dev_err(&client->dev, "%s:operate psensor failed\n",
				__func__);
		en = 0x00;
	}
	if (copy_to_user((unsigned char*)arg, &en, sizeof(unsigned char)))
		return -EFAULT;
	return ret;
}


static int ft_psensor_enable(struct i2c_client *client, unsigned long arg)
{
	int ret = 0;
	unsigned char buf[2];
	unsigned char en = 0x00;
	if (!access_ok(VERIFY_READ, (unsigned char *)arg, sizeof(unsigned char)))
		return -EFAULT;

	if (copy_from_user(&en,
		(unsigned char *)arg, 
		sizeof(unsigned char)))
		return -EFAULT;
	
	buf[0] = PSENSOR_ENABLE_REG;
	buf[1]= en;

	ret = ft6x06_i2c_Write(client, buf, 2);
	if(ret<0)
		dev_err(&client->dev, "%s:operate psensor failed\n",
				__func__);
	return ret;
}

/*
*char device open function interface 
*/
static int ft_psensor_drv_open(struct inode *inode, struct file *filp)
{
	filp->private_data = ft_psensor_dev_tt;
	return 0;
}

/*
*char device close function interface 
*/
static int ft_psensor_drv_release(struct inode *inode, struct file *filp)
{

	return 0;
}

static int ft_psensor_drv_ioctl(struct file *filp, unsigned
  int cmd, unsigned long arg)
{
	int ret = 0;
	struct ft_psensor_dev *ftdev = filp->private_data;
	ftdev = filp->private_data;
	
	down(&ft_psensor_dev_tt->ft_rw_i2c_sem);
	switch (cmd)
	{
	case FT_PSENSOR_IOCTL_GET_ENABLE:
		ret = ft_psensor_get_enable(ftdev->client, arg);	
		break; 
	case FT_PSENSOR_IOCTL_ENABLE:
		ret = ft_psensor_enable(ftdev->client, arg);	
		break; 
	default:
		ret =  -ENOTTY;
		break;
	}
	up(&ft_psensor_dev_tt->ft_rw_i2c_sem);
	return ret;	
}


/*    
*char device file operation which will be put to register the char device
*/
static const struct file_operations ft_psensor_drv_fops = {
	.owner			= THIS_MODULE,
	.open			= ft_psensor_drv_open,
	.release			= ft_psensor_drv_release,
	.unlocked_ioctl	= ft_psensor_drv_ioctl,
};


static void ft_psensor_drv_setup_cdev(struct ft_psensor_dev *dev, int index)
{
	int err, devno = MKDEV(ft_psensor_drv_major, index);

	cdev_init(&dev->cdev, &ft_psensor_drv_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &ft_psensor_drv_fops;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding p-sensor", err);
}

static int ft_psensor_drv_myinitdev(struct i2c_client *client)
{
	int err = 0;
	dev_t devno = MKDEV(ft_psensor_drv_major, 0);

	if (ft_psensor_drv_major)
		err = register_chrdev_region(devno, 1, FT_RW_IIC_DRV);
	else {
		err = alloc_chrdev_region(&devno, 0, 1, FT_RW_IIC_DRV);
		ft_psensor_drv_major = MAJOR(devno);
	}
	if (err < 0) {
		dev_err(&client->dev, "%s:ft_psensor_drv failed  error code=%d---\n",
				__func__, err);
		return err;
	}

	ft_psensor_dev_tt = kmalloc(sizeof(struct ft_psensor_dev), GFP_KERNEL);
	if (!ft_psensor_dev_tt){
		err = -ENOMEM;
		unregister_chrdev_region(devno, 1);
		dev_err(&client->dev, "%s:ft_psensor_drv failed\n",
				__func__);
		return err;
	}
	ft_psensor_dev_tt->client = client;
	init_MUTEX(&ft_psensor_dev_tt->ft_rw_i2c_sem);
	ft_psensor_drv_setup_cdev(ft_psensor_dev_tt, 0); 

	fts_class = class_create(THIS_MODULE, "fts_ps_class");
	if (IS_ERR(fts_class)) {
		dev_err(&client->dev, "%s:failed in creating class.\n",
				__func__);
		return -1; 
	} 
	/*create device node*/
	device_create(fts_class, NULL, MKDEV(ft_psensor_drv_major, 0), 
			NULL, FT_RW_IIC_DRV);

	return 0;
}

int ft_psensor_drv_init(struct i2c_client *client)
{
    	dev_dbg(&client->dev, "[FTS]----ft_psensor_drv init ---\n");
	return ft_psensor_drv_myinitdev(client);
}

void  ft_psensor_drv_exit(void)
{
	device_destroy(fts_class, MKDEV(ft_psensor_drv_major, 0)); 
	/*delete class created by us*/
	class_destroy(fts_class); 
	/*delet the cdev*/
	cdev_del(&ft_psensor_dev_tt->cdev);
	kfree(ft_psensor_dev_tt);
	unregister_chrdev_region(MKDEV(ft_psensor_drv_major, 0), 1); 
}

