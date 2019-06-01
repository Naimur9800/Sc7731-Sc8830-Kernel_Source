/*
 *  mma845x.c - Linux kernel modules for 3-Axis Orientation/Motion
 *  Detection Sensor MMA8451/MMA8452/MMA8453
 *
 *  Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>

#include <linux/ontim_dev_dgb.h>
static char gsensor_version[]="mma845x_1.0";
static char gsensor_vendor_name[20]="mma845x";
    DEV_ATTR_DECLARE(gsensor)
    DEV_ATTR_DEFINE("version",gsensor_version)
    DEV_ATTR_DEFINE("vendor",gsensor_vendor_name)
    DEV_ATTR_DECLARE_END;
    ONTIM_DEBUG_DECLARE_AND_INIT(gsensor,gsensor,8);

#define MMA845X_I2C_ADDR	0x1C
#define MMA8451_ID			0x1A
#define MMA8452_ID			0x2A
#define MMA8453_ID			0x3A

#define POLL_INTERVAL_MIN	1
#define POLL_INTERVAL_MAX	500
#define POLL_INTERVAL		100 /* msecs */

// if sensor is standby ,set POLL_STOP_TIME to slow down the poll
#define POLL_STOP_TIME		0  
#define INPUT_FUZZ			32
#define INPUT_FLAT			32
#define MODE_CHANGE_DELAY_MS	100

#define MMA845X_STATUS_ZYXDR	0x08
#define MMA845X_BUF_SIZE	7
/* register enum for mma845x registers */
enum {
	MMA845X_STATUS = 0x00,
	MMA845X_OUT_X_MSB,
	MMA845X_OUT_X_LSB,
	MMA845X_OUT_Y_MSB,
	MMA845X_OUT_Y_LSB,
	MMA845X_OUT_Z_MSB,
	MMA845X_OUT_Z_LSB,

	MMA845X_F_SETUP = 0x09,
	MMA845X_TRIG_CFG,
	MMA845X_SYSMOD,
	MMA845X_INT_SOURCE,
	MMA845X_WHO_AM_I,
	MMA845X_XYZ_DATA_CFG,
	MMA845X_HP_FILTER_CUTOFF,

	MMA845X_PL_STATUS,
	MMA845X_PL_CFG,
	MMA845X_PL_COUNT,
	MMA845X_PL_BF_ZCOMP,
	MMA845X_P_L_THS_REG,

	MMA845X_FF_MT_CFG,
	MMA845X_FF_MT_SRC,
	MMA845X_FF_MT_THS,
	MMA845X_FF_MT_COUNT,

	MMA845X_TRANSIENT_CFG = 0x1D,
	MMA845X_TRANSIENT_SRC,
	MMA845X_TRANSIENT_THS,
	MMA845X_TRANSIENT_COUNT,

	MMA845X_PULSE_CFG,
	MMA845X_PULSE_SRC,
	MMA845X_PULSE_THSX,
	MMA845X_PULSE_THSY,
	MMA845X_PULSE_THSZ,
	MMA845X_PULSE_TMLT,
	MMA845X_PULSE_LTCY,
	MMA845X_PULSE_WIND,

	MMA845X_ASLP_COUNT,
	MMA845X_CTRL_REG1,
	MMA845X_CTRL_REG2,
	MMA845X_CTRL_REG3,
	MMA845X_CTRL_REG4,
	MMA845X_CTRL_REG5,

	MMA845X_OFF_X,
	MMA845X_OFF_Y,
	MMA845X_OFF_Z,

	MMA845X_REG_END,
};

/* The sensitivity is represented in counts/g. In 2g mode the
sensitivity is 1024 counts/g. In 4g mode the sensitivity is 512
counts/g and in 8g mode the sensitivity is 256 counts/g.
 */
enum {
	MODE_2G = 0,
	MODE_4G,
	MODE_8G,
};

enum {
	MMA_STANDBY = 0,
	MMA_ACTIVED,
};
struct mma845x_data_axis{
	short x;
	short y;
	short z;
};
struct mma845x_data{
	struct i2c_client * client;
	struct input_polled_dev *poll_dev;
	struct mutex data_lock;
	int active;
	int position;
	u8 chip_id;
	int mode;
};
static char * mma845x_names[] ={
   "mma8451",
   "mma8452",
   "mma8453",
};
static int mma845x_position_setting[8][3][3] =
{
   {{ 0, -1,  0}, { 1,  0,	0}, {0, 0,	1}},
   {{-1,  0,  0}, { 0, -1,	0}, {0, 0,	1}},
   {{ 0,  1,  0}, {-1,  0,	0}, {0, 0,	1}},
   {{ 1,  0,  0}, { 0,  1,	0}, {0, 0,	1}},
   
   {{ 0, -1,  0}, {-1,  0,	0}, {0, 0,  -1}},
   {{-1,  0,  0}, { 0,  1,	0}, {0, 0,  -1}},
   {{ 0,  1,  0}, { 1,  0,	0}, {0, 0,  -1}},
   {{ 1,  0,  0}, { 0, -1,	0}, {0, 0,  -1}},
};

static int mma845x_data_convert(struct mma845x_data* pdata,struct mma845x_data_axis *axis_data)
{
   short rawdata[3],data[3];
   int i,j;
   int position = pdata->position ;
   if(position < 0 || position > 7 )
   		position = 0;
   rawdata [0] = axis_data->x ; rawdata [1] = axis_data->y ; rawdata [2] = axis_data->z ;  
   for(i = 0; i < 3 ; i++)
   {
   	data[i] = 0;
   	for(j = 0; j < 3; j++)
		data[i] += rawdata[j] * mma845x_position_setting[position][i][j];
   }
   axis_data->x = data[0];
   axis_data->y = data[1];
   axis_data->z = data[2];
   return 0;
}
static char * mma845x_id2name(u8 id){
	int index = 0;
	if(id == MMA8451_ID)
		index = 0;
	else if(id == MMA8452_ID)
		index = 1;
	else if(id == MMA8453_ID)
		index = 2;
	return mma845x_names[index];
}
static int mma845x_device_init(struct i2c_client *client)
{
	int result;
    struct mma845x_data *pdata = i2c_get_clientdata(client);
	result = i2c_smbus_write_byte_data(client, MMA845X_CTRL_REG1, 0);
	if (result < 0)
		goto out;

	result = i2c_smbus_write_byte_data(client, MMA845X_XYZ_DATA_CFG,
					   pdata->mode);
	if (result < 0)
		goto out;
	pdata->active = MMA_STANDBY;
	msleep(MODE_CHANGE_DELAY_MS);
	return 0;
out:
	dev_err(&client->dev, "error when init mma845x:(%d)", result);
	return result;
}
static int mma845x_device_stop(struct i2c_client *client)
{
	u8 val;
	val = i2c_smbus_read_byte_data(client, MMA845X_CTRL_REG1);
	i2c_smbus_write_byte_data(client, MMA845X_CTRL_REG1,val & 0xfe);
	return 0;
}

static int mma845x_read_data(struct i2c_client *client,struct mma845x_data_axis *data)
{
	u8 tmp_data[MMA845X_BUF_SIZE];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client,
					    MMA845X_OUT_X_MSB, 7, tmp_data);
	if (ret < MMA845X_BUF_SIZE) {
		dev_err(&client->dev, "i2c block read failed\n");
		return -EIO;
	}
	data->x = ((tmp_data[0] << 8) & 0xff00) | tmp_data[1];
	data->y = ((tmp_data[2] << 8) & 0xff00) | tmp_data[3];
	data->z = ((tmp_data[4] << 8) & 0xff00) | tmp_data[5];
	return 0;
}

static void mma845x_report_data(struct mma845x_data* pdata)
{
	struct input_polled_dev * poll_dev = pdata->poll_dev;
	struct mma845x_data_axis data;
	mutex_lock(&pdata->data_lock);
	if(pdata->active == MMA_STANDBY){
		poll_dev->poll_interval = POLL_STOP_TIME;  // if standby ,set as 10s to slow the poll,
		goto out;
	}else{
		if(poll_dev->poll_interval == POLL_STOP_TIME)
			poll_dev->poll_interval = POLL_INTERVAL;
	}
	if (mma845x_read_data(pdata->client,&data) != 0)
		goto out;
    mma845x_data_convert(pdata,&data);
	input_report_abs(poll_dev->input, ABS_X, data.x);
	input_report_abs(poll_dev->input, ABS_Y, data.y);
	input_report_abs(poll_dev->input, ABS_Z, data.z);
	input_sync(poll_dev->input);
out:
	mutex_unlock(&pdata->data_lock);
}

static void mma845x_dev_poll(struct input_polled_dev *dev)
{
    struct mma845x_data* pdata = (struct mma845x_data*)dev->private;
	mma845x_report_data(pdata);
}

static ssize_t mma845x_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct input_polled_dev *poll_dev = dev_get_drvdata(dev);
	struct mma845x_data *pdata = (struct mma845x_data *)(poll_dev->private);
	struct i2c_client *client = pdata->client;
	u8 val;
    int enable;
	
	mutex_lock(&pdata->data_lock);
	val = i2c_smbus_read_byte_data(client, MMA845X_CTRL_REG1);  
	if((val & 0x01) && pdata->active == MMA_ACTIVED)
		enable = 1;
	else
		enable = 0;
	mutex_unlock(&pdata->data_lock);
	return sprintf(buf, "%d\n", enable);
}

static ssize_t mma845x_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct input_polled_dev *poll_dev = dev_get_drvdata(dev);
	struct mma845x_data *pdata = (struct mma845x_data *)(poll_dev->private);
	struct i2c_client *client = pdata->client;
	int ret;
	unsigned long enable;
	u8 val = 0;
	enable = simple_strtoul(buf, NULL, 10);    
	mutex_lock(&pdata->data_lock);
	enable = (enable > 0) ? 1 : 0;
    if(enable && pdata->active == MMA_STANDBY)
	{  
	   val = i2c_smbus_read_byte_data(client,MMA845X_CTRL_REG1);
	   ret = i2c_smbus_write_byte_data(client, MMA845X_CTRL_REG1, val|0x01);  
	   //poll_dev->input->open(poll_dev->input);
	   if(!ret){
	   	 pdata->active = MMA_ACTIVED;
		 printk("mma enable setting active \n");
	    }
	}
	else if(enable == 0  && pdata->active == MMA_ACTIVED)
	{
	   //poll_dev->input->close(poll_dev->input);
		val = i2c_smbus_read_byte_data(client,MMA845X_CTRL_REG1);
	    ret = i2c_smbus_write_byte_data(client, MMA845X_CTRL_REG1,val & 0xFE);
		if(!ret){
		 pdata->active= MMA_STANDBY;
		 printk("mma enable setting inactive \n");
		}
	}
	mutex_unlock(&pdata->data_lock);
	return count;
}
static ssize_t mma845x_position_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
    struct input_polled_dev *poll_dev = dev_get_drvdata(dev);
	struct mma845x_data *pdata = (struct mma845x_data *)(poll_dev->private);
    int position = 0;
	mutex_lock(&pdata->data_lock);
    position = pdata->position ;
	mutex_unlock(&pdata->data_lock);
	return sprintf(buf, "%d\n", position);
}

static ssize_t mma845x_position_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
    struct input_polled_dev *poll_dev = dev_get_drvdata(dev);
	struct mma845x_data *pdata = (struct mma845x_data *)(poll_dev->private);
	int  position;
	position = simple_strtoul(buf, NULL, 10);    
	mutex_lock(&pdata->data_lock);
    pdata->position = position;
	mutex_unlock(&pdata->data_lock);
	return count;
}


static ssize_t mma845x_vendor_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	 return sprintf(buf, "mma845x\n");
}

static DEVICE_ATTR(vendor_name, S_IRUGO,
		mma845x_vendor_name_show, NULL);

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		   mma845x_enable_show, mma845x_enable_store);
static DEVICE_ATTR(position, S_IWUSR | S_IRUGO,
		   mma845x_position_show, mma845x_position_store);


static struct attribute *mma845x_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_position.attr,
	&dev_attr_vendor_name.attr,
	NULL
};

static const struct attribute_group mma845x_attr_group = {
	.attrs = mma845x_attributes,
};

static int __init mma845x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result, chip_id;
	struct input_dev *idev;
	struct mma845x_data *pdata;
	struct i2c_adapter *adapter;
	struct input_polled_dev *poll_dev;
	printk(KERN_INFO "%s",__func__);
       if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
       { 
           return -EIO;
       }
	   
	adapter = to_i2c_adapter(client->dev.parent);
	result = i2c_check_functionality(adapter,
					 I2C_FUNC_SMBUS_BYTE |
					 I2C_FUNC_SMBUS_BYTE_DATA);
	if (!result)
		goto err_out;
	
	chip_id = i2c_smbus_read_byte_data(client, MMA845X_WHO_AM_I);

	if (chip_id != MMA8451_ID && chip_id != MMA8452_ID && chip_id != MMA8453_ID  ) {
		dev_err(&client->dev,
			"read chip ID 0x%x is not equal to 0x%x , 0x%x , 0x%x!\n",
			chip_id, MMA8451_ID, MMA8452_ID, MMA8453_ID);
		result = -EINVAL;
		goto err_out;
	}
    pdata = kzalloc(sizeof(struct mma845x_data), GFP_KERNEL);
	if(!pdata){
		result = -ENOMEM;
		dev_err(&client->dev, "alloc data memory error!\n");
		goto err_out;
    }
	/* Initialize the MMA845X chip */
	pdata->client = client;
	pdata->chip_id = chip_id;
	pdata->mode = MODE_2G;
	pdata->position = 0;
	mutex_init(&pdata->data_lock);
	i2c_set_clientdata(client,pdata);
	mma845x_device_init(client);
	poll_dev = input_allocate_polled_device();
	if (!poll_dev) {
		result = -ENOMEM;
		dev_err(&client->dev, "alloc poll device failed!\n");
		goto err_alloc_poll_device;
	}
	poll_dev->poll = mma845x_dev_poll;
	poll_dev->poll_interval = POLL_STOP_TIME;
	poll_dev->poll_interval_min = POLL_INTERVAL_MIN;
	poll_dev->poll_interval_max = POLL_INTERVAL_MAX;
	poll_dev->private = pdata;
	idev = poll_dev->input;
	idev->name = "accelerometer";
	idev->uniq = mma845x_id2name(pdata->chip_id);
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);
	input_set_abs_params(idev, ABS_X, -0x7fff, 0x7fff, 0, 0);
	input_set_abs_params(idev, ABS_Y, -0x7fff, 0x7fff, 0, 0);
	input_set_abs_params(idev, ABS_Z, -0x7fff, 0x7fff, 0, 0);
    pdata->poll_dev = poll_dev;
	result = input_register_polled_device(pdata->poll_dev);
	if (result) {
		dev_err(&client->dev, "register poll device failed!\n");
		goto err_register_polled_device;
	}
    result = sysfs_create_group(&idev->dev.kobj, &mma845x_attr_group);
	if (result) {
		dev_err(&client->dev, "create device file failed!\n");
		result = -EINVAL;
		goto err_create_sysfs;
	}
	REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
	printk("mma845x device driver probe successfully\n");
	return 0;
err_create_sysfs:
	input_unregister_polled_device(pdata->poll_dev);
err_register_polled_device:
	input_free_polled_device(poll_dev);
err_alloc_poll_device:
	kfree(pdata);
err_out:
	return result;
}
static int __devexit mma845x_remove(struct i2c_client *client)
{
	struct mma845x_data *pdata = i2c_get_clientdata(client);
	struct input_polled_dev *poll_dev = pdata->poll_dev;
	mma845x_device_stop(client);
	if(pdata){
		input_unregister_polled_device(poll_dev);
		input_free_polled_device(poll_dev);
		kfree(pdata);
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mma845x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
    struct mma845x_data *pdata = i2c_get_clientdata(client);
    if(pdata->active == MMA_ACTIVED)
		mma845x_device_stop(client);
	return 0;
}

static int mma845x_resume(struct device *dev)
{
    int val = 0;
	struct i2c_client *client = to_i2c_client(dev);
    struct mma845x_data *pdata = i2c_get_clientdata(client);
    if(pdata->active == MMA_ACTIVED){
	   val = i2c_smbus_read_byte_data(client,MMA845X_CTRL_REG1);
	   i2c_smbus_write_byte_data(client, MMA845X_CTRL_REG1, val|0x01);  
    }
	return 0;
	  
}
#endif

static const struct i2c_device_id mma845x_id[] = {
	{"mma845x", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, mma845x_id);

static SIMPLE_DEV_PM_OPS(mma845x_pm_ops, mma845x_suspend, mma845x_resume);
static struct i2c_driver mma845x_driver = {
	.driver = {
		   .name = "mma845x",
		   .owner = THIS_MODULE,
		   .pm = &mma845x_pm_ops,
		   },
	.probe = mma845x_probe,
	.remove = __devexit_p(mma845x_remove),
	.id_table = mma845x_id,
};

static int __init mma845x_init(void)
{
	/* register driver */
	int res;
	printk(KERN_INFO "%s",__func__);

	res = i2c_add_driver(&mma845x_driver);
	if (res < 0) {
		printk(KERN_INFO "add mma845x i2c driver failed\n");
		return -ENODEV;
	}
	return res;
}

static void __exit mma845x_exit(void)
{
	i2c_del_driver(&mma845x_driver);
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MMA845X 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");

module_init(mma845x_init);
module_exit(mma845x_exit);
