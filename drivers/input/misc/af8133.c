/******************** (C) COPYRIGHT Voltafield 2014 ********************
*
* File Name          : af8133.c
* Authors            : Production, CAE Team
*                    : Gary Huang
* Date               : 2015/Jul/10
* Description        : AF7133E/AF8133i Magneto sensor Driver
*
************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************/
/*******************************************************************************
Version History.

20150710  1st version
*******************************************************************************/

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/workqueue.h> 
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>

//#include <linux/af8133.h>
#include <linux/i2c/af8133.h>
//#include <linux/sensors.h>

#define AF8133_DEBUG 0
#define AF8133_DRIVER_VERSION     "3.0.2"

#define DELAY_TIME_DEFAULT         50   //ms, 20Hz
#define DELAY_TIME_MIN             10   //ms, 100Hz
#define DELAY_TIME_MAX            100   //ms, 10Hz

#define AF8133_SET_RESET_NUM      5
#define AF8133_OFFSET_INDEX       2   //(int)(AF8133_SET_RESET_NUM/2)

static struct i2c_client *this_client;

struct af8133_data {
        struct i2c_client *client;
        struct device *class_dev;
        struct class *compass;
        struct af8133_platform_data *pdata;
        struct mutex lock;
        struct input_dev *input;
        struct delayed_work work;
        atomic_t enabled;
};

static const struct af8133_platform_data default_af8133_pdata = {
        .poll_interval = DELAY_TIME_DEFAULT,
        .axis_map_x = 0,
        .axis_map_y = 1,
        .axis_map_z = 2,
        .negate_x = 0,
        .negate_y = 0,
        .negate_z = 0,
        .reg33 =0,
};


static int VTC_i2c_Rx(char *rxData, int length)
{
        uint8_t retry;
        struct i2c_msg msgs[] = 
        {
                {
                        .addr = this_client->addr,
                        .flags = 0,
                        .len = 1,
                        .buf = rxData,
                },
                {
                        .addr = this_client->addr,
                        .flags = I2C_M_RD,
                        .len = length,
                        .buf = rxData,
                },
        };

        for (retry = 0; retry < 3; retry++) 
        {
                if (i2c_transfer(this_client->adapter, msgs, 2) > 0)
                        break;
                else
                        mdelay(10);
        }

        if (retry >= 3) 
        {
                printk(KERN_ERR "%s: retry over 3\n", __func__);
                return -EIO;
        } 
        else
                return 0;
}

static int VTC_i2c_Tx(char *txData, int length)
{
        int retry;
        struct i2c_msg msg[] = 
        {
                {
                        .addr = this_client->addr,
                        .flags = 0,
                        .len = length,
                        .buf = txData,
                },
        };

        for (retry = 0; retry <= 3; retry++) 
        {
                if (i2c_transfer(this_client->adapter, msg, 1) > 0)
                        break;
                else
                        mdelay(10);
        }

        if (retry > 3) 
        {
                printk(KERN_ERR "%s: retry over 3\n", __func__);
                return -EIO;
        }
        else
                return 0;
}


static int af8133_measurement_setting(u8 reg14, u8 reg33)
{
  int err;
  u8  data[2];

  //full measurement, set/reset setting 
  data[0]= 0x14;
  data[1]= reg14;
  err = VTC_i2c_Tx(data, 2);
  if(err < 0) return -1;

  //field range +/-1200uT
  data[0]= 0x33;
  data[1]= reg33;
  err = VTC_i2c_Tx(data, 2);
  if(err < 0) return -1;

  //field range +/-1200uT
  data[0]= 0x0B;
  data[1]= 0x3C; 
  err = VTC_i2c_Tx(data, 2);
  if(err < 0) return -1;

  //disable moving average
  data[0]= 0x13;
  data[1]= 0x00; 
  err = VTC_i2c_Tx(data, 2);
  if(err < 0) return -1;

  return 0;
}

static int af8133_mag_get_data(struct af8133_data *mag)
{
  int err;
  u8  data[6];
  int i;
  int xyz[3];
  u8  report;

  data[0] = 0x02;
  err = VTC_i2c_Rx(data, 1);
  if(err < 0) goto err_i2c_fail;
  report = data[0];

  data[0] = 0x03;
  err = VTC_i2c_Rx(data, 6);
  if(err < 0) goto err_i2c_fail;

  xyz[0] = data[0] | (data[1] << 8); 
  xyz[1] = data[2] | (data[3] << 8);
  xyz[2] = data[4] | (data[5] << 8);

  for(i=0;i<3;i++)
  {
  	if(xyz[i] != 0x8000)
  	{
  	  xyz[i] = (xyz[i] > 32767) ? (xyz[i] - 65536) : xyz[i];

      xyz[i] -= mag->pdata->offset[i];
    }
  }  
        
//  err = af8133_measurement_setting(0x38, mag->pdata->reg33);
//  if(err < 0) goto err_i2c_fail;      

  //next read
  data[0] = 0x0A;
  data[1] = 0x01; 
  err = VTC_i2c_Tx(data, 2);
  if(err < 0) goto err_i2c_fail; 

  if(report)
  {
    input_report_abs(mag->input, ABS_X, xyz[0]);
    input_report_abs(mag->input, ABS_Y, xyz[1]);
    input_report_abs(mag->input, ABS_Z, xyz[2]);
    input_sync(mag->input);
  }
  return 0;

err_i2c_fail:
  printk(KERN_ERR "%s:failed\n", __func__);
  return err;
}

static int af8133_i2c_init(struct af8133_data *mag)
{
  int err;
  int i, j, k;
  int mag_pos[3][5], mag_neg[3][5];
  u8  data[6];

  //*********************************
  //Enable I2C
  //*********************************
  data[0]= 0x10;
  data[1]= 0xAA; 
  err = VTC_i2c_Tx(data, 2);
  //ignore ACK   

  //*********************************
  //get calibration parameter
  //*********************************
  data[0]= 0x14;
  data[1]= 0x3C; 
  err = VTC_i2c_Tx(data, 2);
  if(err < 0) return -1;    

  data[0] = 0x1F;
  err = VTC_i2c_Rx(data, 1);
  if(err < 0) return -1;    

  if(data[0] & 0x04) mag->pdata->reg33 = 0x14;
  else if(data[0] & 0x02) mag->pdata->reg33 = 0x17;
  else if(data[0] & 0x01) mag->pdata->reg33 = 0x15;
  else mag->pdata->reg33 = 0x16; 

  //*********************************
  //check product code
  //*********************************
  data[0] = 0x00;
  err = VTC_i2c_Rx(data, 1);
  if(err < 0) return -1;

  if(data[0] != 0x50) return -2;

  //*********************************
  //get offset by Set/Reset
  //*********************************
  err = af8133_measurement_setting(0x34, mag->pdata->reg33);
  if(err < 0) return err; 

  //full measurement, reset only 
  for(i=0;i<AF8133_SET_RESET_NUM;i++)
  {                   
    data[0]= 0x0A;
    data[1]= 0x01; 
    err = VTC_i2c_Tx(data, 2);
    if(err < 0) return -1;

    mdelay(2);

    data[0]= 0x03; 
    err = VTC_i2c_Rx(data, 6);
    if(err < 0) return -1;

    mag_neg[0][i] = data[0] | (data[1] << 8); 
    mag_neg[1][i] = data[2] | (data[3] << 8);
    mag_neg[2][i] = data[4] | (data[5] << 8);

    for(j=0;j<3;j++) mag_neg[j][i] = (mag_neg[j][i] > 32767) ? (mag_neg[j][i] - 65536) : mag_neg[j][i];
  }

  //full measurement, set only
  err = af8133_measurement_setting(0x38, mag->pdata->reg33);
  if(err < 0) return err; 

  for(i=0;i<AF8133_SET_RESET_NUM;i++)
  {                   
    data[0]= 0x0A;
    data[1]= 0x01; 
    err = VTC_i2c_Tx(data, 2);
    if(err < 0) return -1;

    mdelay(2);

    data[0]= 0x03; 
    err = VTC_i2c_Rx(data, 6);
    if(err < 0) return -1;

    mag_pos[0][i] = data[0] | (data[1] << 8); 
    mag_pos[1][i] = data[2] | (data[3] << 8);
    mag_pos[2][i] = data[4] | (data[5] << 8);

    for(j=0;j<3;j++) mag_pos[j][i] = (mag_pos[j][i] > 32767) ? (mag_pos[j][i] - 65536) : mag_pos[j][i];
  }

  //sort data
  for(i=0;i<3;i++){    
    for(j=0;j<4;j++){
      for(k=0;k<4;k++){
        if(mag_neg[i][k] < mag_neg[i][k+1]){
          int tmp = mag_neg[i][k];
          mag_neg[i][k] = mag_neg[i][k+1];
          mag_neg[i][k+1] = tmp;
         }
        if(mag_pos[i][k] < mag_pos[i][k+1]){
          int tmp = mag_pos[i][k];
          mag_pos[i][k] = mag_pos[i][k+1];
          mag_pos[i][k+1] = tmp;
         } 
       }
     }
   }

  for(i=0;i<3;i++)
    mag->pdata->offset[i] = (mag_pos[i][AF8133_OFFSET_INDEX]+mag_neg[i][AF8133_OFFSET_INDEX])/2; 

  return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf)
{
        struct af8133_data *mag = dev_get_drvdata(dev);
       unsigned long interval_ms;
       mutex_lock(&mag->lock);
        interval_ms = mag->pdata->poll_interval;
        mutex_unlock(&mag->lock);
	return sprintf(buf, "%ld\n", interval_ms);
}

static ssize_t attr_set_polling_rate(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t size)
{
        struct af8133_data *mag = dev_get_drvdata(dev);
        unsigned long interval_ms;

        if (strict_strtoul(buf, 10, &interval_ms))
                return -EINVAL;
        if (!interval_ms)
                return -EINVAL;

        if(interval_ms > DELAY_TIME_MAX)
        	interval_ms = DELAY_TIME_MAX;
        else if(interval_ms < DELAY_TIME_MIN)
        	interval_ms = DELAY_TIME_MIN;

        mutex_lock(&mag->lock);
        mag->pdata->poll_interval = interval_ms;
        mutex_unlock(&mag->lock);

        if(atomic_read(&mag->enabled))
         schedule_delayed_work(&mag->work, msecs_to_jiffies(mag->pdata->poll_interval));

        return size;
}

static ssize_t attr_get_enable(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
        struct af8133_data *mag = dev_get_drvdata(dev);
        int val = atomic_read(&mag->enabled);

		//printk(KERN_ERR "lgf attr_get_enable val=%d\n",val);
        return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t size)
{
       struct af8133_data *mag = dev_get_drvdata(dev);
       unsigned int pre_en = atomic_read(&mag->enabled);
       unsigned int new_en = 0;

        //if (strict_strtoul(buf, 10, &new_en))
        //  return -EINVAL;
        if (sysfs_streq(buf, "1"))
                new_en = 1;
        else if (sysfs_streq(buf, "0"))
                new_en = 0;
		//printk(KERN_ERR "lgf attr_get_enable new_en=%d,pre_en=%d\n",new_en,pre_en);

        if (new_en != pre_en)
        {
         if(new_en)
          {
           schedule_delayed_work(&mag->work, msecs_to_jiffies(mag->pdata->poll_interval));
           atomic_set(&mag->enabled, 1);
          }
         else
          {
           cancel_delayed_work_sync(&mag->work);
           atomic_set(&mag->enabled, 0);
          }
        }

        return size;
}


static struct device_attribute attributes[] = {
        __ATTR(pollrate_ms, 0777, attr_get_polling_rate, attr_set_polling_rate),
        __ATTR(enable_device, 0777, attr_get_enable, attr_set_enable),
};

static int create_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;
	int err = 0;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i) {
		err = device_create_file(dev, &attrs[i]);
		if (err)
			break;
	}

	if (err) {
		for (--i; i >= 0 ; --i)
			device_remove_file(dev, &attrs[i]);
	}

	return err;
}

static char const *const device_link_name = "i2c";
static dev_t const af8133_device_dev_t = MKDEV(MISC_MAJOR, 240);

static int create_sysfs_interfaces(struct af8133_data *af_mag)
{
        int err = 0;
   
        if (NULL == af_mag)
          return -EINVAL;

        af_mag->compass = class_create(THIS_MODULE, AF8133_SYS_CLS_NAME);
	if(IS_ERR(af_mag->compass))
	{
                err = PTR_ERR(af_mag->compass);
	        printk("%s, create class, error\n", __func__);
		return err;
	}

	af_mag->class_dev = device_create(af_mag->compass,
                                          NULL, 
                                          af8133_device_dev_t,
                                          af_mag,
                                          AF8133_SYS_DEV_NAME);
	if (IS_ERR(af_mag->class_dev)) {
		err = PTR_ERR(af_mag->class_dev);
                printk("%s, create class device, error\n", __func__);
		return err;
	}

	err = create_device_attributes(af_mag->class_dev, attributes);
	if (0 > err) {
                printk("%s, create attributes, error\n", __func__);
        }

	return err;
}

static int remove_sysfs_interfaces(struct device *dev)
{
        int i;
        for (i = 0; i < ARRAY_SIZE(attributes); i++)
                device_remove_file(dev, attributes + i);
        return 0;
}

static void af8133_work_func(struct work_struct *work)
{
        struct af8133_data *mag = container_of((struct delayed_work *)work, struct af8133_data, work);

        int err = af8133_mag_get_data(mag);
        if (err < 0)
          dev_err(&mag->client->dev, "get magnetometer data failed\n");

       schedule_delayed_work(&mag->work, msecs_to_jiffies(mag->pdata->poll_interval));
}

static int af8133_mag_input_init(struct af8133_data *mag)
{
       int err;
       struct input_dev *dev;
  
        dev = input_allocate_device();

        if (!dev) {
          return -ENOMEM;
        }

        dev->id.bustype = BUS_I2C;
        dev->name = AF8133_INPUT_DEV_NAME;

        set_bit(EV_ABS, dev->evbit);
        input_set_abs_params(dev, ABS_X, -32768, 32767, 0, 0);
        input_set_abs_params(dev, ABS_Y, -32768, 32767, 0, 0);
        input_set_abs_params(dev, ABS_Z, -32768, 32767, 0, 0);

       input_set_drvdata(dev, mag);

        err = input_register_device(dev);
       if (err < 0) {
         input_free_device(dev);
         return err;
        }

       mag->input = dev;

        return 0;
}

static int af8133_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        struct af8133_data *mag;
        int err;

        printk("%s: driver version is %s\n", __func__, AF8133_DRIVER_VERSION);

        if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
        {
                dev_err(&client->dev, "i2c_check_functionality error\n");
                err = -ENODEV;
                goto err_check_func;
        }

	/* Allocate memory for driver data */
        mag = kzalloc(sizeof(struct af8133_data), GFP_KERNEL);
        if (!mag) {
                dev_err(&client->dev, "failed to allocate memory for module data\n");
                err = -ENOMEM;
                goto err_alloc_mag_data;
        }

        mutex_init(&mag->lock);
        mutex_lock(&mag->lock);
        mag->client = client;

        mag->pdata = kmalloc(sizeof(*mag->pdata), GFP_KERNEL);
        if (!mag->pdata) 
                goto err_alloc_mag_pdata;

        if (client->dev.platform_data == NULL) {
                memcpy(mag->pdata, &default_af8133_pdata, sizeof(*mag->pdata));
        } else {
                memcpy(mag->pdata, client->dev.platform_data, sizeof(*mag->pdata));
        }

	/***** I2C initialization *****/
        i2c_set_clientdata(client, mag);
        this_client = mag->client;

        err = af8133_i2c_init(mag);
        if (err < 0)
          goto err_alloc_mag_pdata;               

	/***** input *****/
        err = af8133_mag_input_init(mag);
        if (err < 0)
          goto err_input_init;

	/***** sysfs *****/
        err = create_sysfs_interfaces(mag);
        if (err < 0) {
                dev_err(&client->dev, "%s create sysfs interfacce failed\n", AF8133_I2C_DEV_NAME);
                goto err_clean_input;
        }

    /* workqueue init */
       INIT_DELAYED_WORK(&mag->work, af8133_work_func);

        mutex_unlock(&mag->lock);
        dev_info(&client->dev, "af8133 probed successfully\n");
        
        atomic_set(&mag->enabled, 0);

        return 0;

err_clean_input:
        input_unregister_device(mag->input);
err_input_init:
        kfree(mag->pdata);
err_alloc_mag_pdata:
        kfree(mag);
err_alloc_mag_data:
err_check_func:
        pr_err("%s: AF8133 Driver Initialization failed\n", __func__);
        i2c_set_clientdata(client, NULL);
        return err;
}

static int af8133_remove(struct i2c_client *client)
{
        struct af8133_data *mag = i2c_get_clientdata(client);

        if( mag == NULL )
                return 0;

       printk("%s: af8133 remove\n", __func__);
 
        remove_sysfs_interfaces(mag->class_dev);
       input_unregister_device(mag->input);

       kfree(mag->pdata);
        kfree(mag);

        return 0;
}

static int af8133_suspend(struct device *dev)
{
       struct af8133_data *mag = dev_get_drvdata(dev);

        if( mag == NULL )
                return 0;

       if (atomic_read(&mag->enabled))
         cancel_delayed_work_sync(&mag->work);

       printk("%s: af8133 suspend\n", __func__);
        return 0;
}

static int af8133_resume(struct device *dev)
{
        struct af8133_data *mag = dev_get_drvdata(dev);

        if( mag == NULL )
                return 0;

        af8133_measurement_setting(0x38, mag->pdata->reg33);

        if (atomic_read(&mag->enabled))
          schedule_delayed_work(&mag->work, msecs_to_jiffies(mag->pdata->poll_interval));

        printk("%s: af8133 resume\n", __func__);
        return 0;
}

static const struct i2c_device_id af8133_i2c_id[] = {
        {AF8133_I2C_DEV_NAME, 0},
        {},
};

//MODULE_DEVICE_TABLE(i2c, af8133_mag_id);

static struct dev_pm_ops af8133_pm = {
        .suspend = af8133_suspend,
        .resume = af8133_resume,
};

static struct of_device_id af8133_match_table[] = {
	{ .compatible = "vtc,af8133", },
	{},
};

static struct i2c_driver af8133_driver = {
        .driver = {
                        .owner = THIS_MODULE,
                        .name = AF8133_I2C_DEV_NAME,
			.of_match_table = af8133_match_table,
                        .pm = &af8133_pm,
            },
        .probe = af8133_probe,
        .remove = af8133_remove,
        .id_table = af8133_i2c_id,
};

//static struct i2c_board_info __initdata i2c_af8133={ I2C_BOARD_INFO(AF8133_I2C_DEV_NAME, 0x1c)};  
static int __init af8133_init(void)
{
        printk("%s: af8133 3-axis megnetometer driver: init\n", __func__);
                                    
        //struct i2c_adapter *adapter;                                     
        //struct i2c_client *client;                                       

        //adapter = i2c_get_adapter(2);                                                        
        //client = i2c_new_device(adapter, &i2c_af8133);                         

        return i2c_add_driver(&af8133_driver);
}

static void __exit af8133_exit(void)
{
        printk("%s: af8133 3-axis megnetometer driver: exit\n", __func__);
        i2c_del_driver(&af8133_driver);
        return;
}

module_init(af8133_init);
module_exit(af8133_exit);

MODULE_DESCRIPTION("af8133 3-axis magnetometer driver");
MODULE_AUTHOR("Voltafield CAE team");
MODULE_LICENSE("GPL");
MODULE_VERSION(AF8133_DRIVER_VERSION);
