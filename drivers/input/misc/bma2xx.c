/* BMA2XX motion sensor driver
 *
 * This driver supports:
 * BMA222E/BMA223/BMA250E/BMA255/BMA254/BMA253/BMA280
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/module.h>

//#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include "bma2xx.h"

#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/kthread.h>

/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_BMA2XX 253
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
     
#define SW_CALIBRATION

/*----------------------------------------------------------------------------*/
#define BMA2XX_AXIS_X           0
#define BMA2XX_AXIS_Y           1
#define BMA2XX_AXIS_Z           2
#define BMA2XX_DATA_LEN         6
#define BMA2XX_DEV_NAME         "BMA2XX"
#define BMA2XX_ACCEL_INPUT_NAME "ZTE_ACCEL"
#define MISC_DEVICE_NAME        "zte_acc"

#define C_I2C_FIFO_SIZE 8
#define BMA2XX_CALI_LEN            (10)
#define	CAL_PATH		"/productinfo/gsensor_calibration.ini"
//#define BMA2XX_CALI_TOLERANCE		(1000) //mg
#define BMA2XX_MAXZ 1024

#define GRAVITY_EARTH_1000 9807

#define ACC_VALUE_MAX (32767)
#define ACC_VALUE_MIN (-32768)

#define	KIONIX_ACCEL_IOCTL_BASE 77
/** The following define the IOCTL command values via the ioctl macros */
#define KIONIX_ACCEL_IOCTL_SET_DELAY		_IOW(KIONIX_ACCEL_IOCTL_BASE, 0, int)
#define KIONIX_ACCEL_IOCTL_GET_DELAY		_IOR(KIONIX_ACCEL_IOCTL_BASE, 1, int)
#define KIONIX_ACCEL_IOCTL_SET_ENABLE		_IOW(KIONIX_ACCEL_IOCTL_BASE, 2, int)
#define KIONIX_ACCEL_IOCTL_GET_ENABLE		_IOR(KIONIX_ACCEL_IOCTL_BASE, 3, int)

/*----------------------------------------------------------------------------*/
struct acc_hw {
	int direction;		/*!< the direction of the chip */
	int firlen;		/*!< the length of low pass filter */
	unsigned int interval;
    unsigned int delay;
    unsigned int min_delay;
};

/*----------------------------------------------------------------------------*/
//for direction convert
#define C_MAX_HWMSEN_EVENT_NUM 4
struct acc_convert {
	s8 sign[C_MAX_HWMSEN_EVENT_NUM];
	u8 map[C_MAX_HWMSEN_EVENT_NUM];
};
struct acc_convert map[] = {
	{ { 1, 1, 1}, {0, 1, 2} },
	{ {-1, 1, 1}, {1, 0, 2} },
	{ {-1, -1, 1}, {0, 1, 2} },
	{ { 1, -1, 1}, {1, 0, 2} },

	{ {-1, 1, -1}, {0, 1, 2} },
	{ { 1, 1, -1}, {1, 0, 2} },
	{ { 1, -1, -1}, {0, 1, 2} },
	{ {-1, -1, -1}, {1, 0, 2} },

};
int acc_get_convert(int direction, struct acc_convert *cvt)
{
	if (!cvt)
		return -EINVAL;
	else if (direction >= sizeof(map)/sizeof(map[0]))
		return -EINVAL;

	*cvt = map[direction];
	return 0;
}
/*----------------------------------------------------------------------------*/

struct acc_hw accel_cust521;
static struct acc_hw *hw = &accel_cust521;

/*********/
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id bma2xx_i2c_id[] = {{BMA2XX_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_BMA2XX={ I2C_BOARD_INFO(BMA2XX_DEV_NAME, 0x18)};

/*----------------------------------------------------------------------------*/
static int bma2xx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int bma2xx_i2c_remove(struct i2c_client *client);
#if !defined(CONFIG_HAS_EARLYSUSPEND) || !defined(USE_EARLY_SUSPEND)
static int bma2xx_suspend(struct i2c_client *client, pm_message_t msg);
static int bma2xx_resume(struct i2c_client *client);
#endif

//static int  gsensor_remove(void);
//static int gsensor_set_delay(u64 ns);
/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][BMA2XX_AXES_NUM];
    int sum[BMA2XX_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct bma2xx_i2c_data {
    struct i2c_client *client;
    struct input_dev *idev;
    struct acc_hw *hw;
    struct acc_convert   cvt;
    struct delayed_work accel_work;
    struct workqueue_struct *accel_workqueue;

    int16_t sensor_type;
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
    s16                     cali_sw[BMA2XX_AXES_NUM+1];

    /*data*/
    s8                      offset[BMA2XX_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[BMA2XX_AXES_NUM+1];

#if defined(CONFIG_BMA2XX_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
    struct early_suspend    early_drv;
#endif   
    u8                      bandwidth;
};
static int gsensor_enable(struct bma2xx_i2c_data *obj);
static int gsensor_disable(struct bma2xx_i2c_data *obj);
static int gsensor_set_delay(struct bma2xx_i2c_data *obj, u64 ms);

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
	{.compatible = "bosch,bma2xx"},
	{},
};
#endif

static struct i2c_driver bma2xx_i2c_driver = {
    .driver = {
//        .owner          = THIS_MODULE,
        .name           = BMA2XX_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = accel_of_match,
#endif  
    },
	.probe      		= bma2xx_i2c_probe,
	.remove    			= bma2xx_i2c_remove,
//#ifndef USE_EARLY_SUSPEND
    .suspend            = bma2xx_suspend,
    .resume             = bma2xx_resume,
//#endif
	.id_table = bma2xx_i2c_id,
//	.address_data = &bma2xx_addr_data,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *bma2xx_i2c_client = NULL;
static struct bma2xx_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = false;
// power_status remember sensor_power for suspend and resume.
static bool power_status = false;
static int sensor_suspend = 0;
//static char selftestRes[8]= {0}; 
static DEFINE_MUTEX(gsensor_mutex);
static DEFINE_MUTEX(gsensor_scp_en_mutex);
//static struct proc_dir_entry *acc_calibrate_proc_file = NULL;

//static bool enable_status = false;

#define GSE_DEBUG
/*----------------------------------------------------------------------------*/
#ifdef GSE_DEBUG
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(GSE_TAG fmt, ##args)
#else 
#define GSE_TAG                 
#define GSE_FUN(f)               
#define GSE_ERR(fmt, args...)    
#define GSE_LOG(fmt, args...) 
#endif

static const struct bma2x2_type_map_t sensor_type_map[] = {
	{BMA222E_CHIP_ID, BMA222E_TYPE, "BMA222E/223"},
	{BMA250E_CHIP_ID, BMA250E_TYPE, "BMA250E"},
	{BMA255_CHIP_ID, BMA255_TYPE, "BMA255/254/253"},
	{BMA280_CHIP_ID, BMA280_TYPE, "BMA280"},
};

/*----------------------------------------------------------------------------*/
static struct data_resolution bma2xx_data_resolution[] = {
 /* combination by {FULL_RES,RANGE}*/
    {{ 15, 6}, 64},	/*bma222E/223: +/-2g  in 8-bit resolution:  15.6 mg/LSB*/
    {{ 3, 91}, 256},	/*bma250E: +/-2g  in 10-bit resolution:  3.91 mg/LSB*/
    {{ 0, 98}, 1024},	/*bma255/254/253: +/-2g  in 12-bit resolution:  0.975 mg/LSB*/
    {{ 0, 24}, 4096},	/*bma280: +/-2g  in 14-bit resolution:  0.244 mg/LSB*/
};
/*----------------------------------------------------------------------------*/
static struct data_resolution bma2xx_offset_resolution[] = {
    {{ 15, 6}, 64},	/*bma222E/223: +/-2g  in 8-bit resolution:  15.6 mg/LSB*/
    {{ 3, 91}, 256},	/*bma250E: +/-2g  in 10-bit resolution:  3.91 mg/LSB*/
    {{ 0, 98}, 1024},	/*bma255/254/253: +/-2g  in 12-bit resolution:  0.975 mg/LSB*/
    {{ 0, 24}, 4096},	/*bma280: +/-2g  in 14-bit resolution:  0.244 mg/LSB*/
};

/*----------------------------------------------------------------------------*/
static int bma_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    u8 beg = addr;
	int err;
	struct i2c_msg msgs[2]={{0},{0}};
	
	mutex_lock(&gsensor_mutex);
	
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len =1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len =len;
	msgs[1].buf = data;
	
	if (!client)
	{
	    mutex_unlock(&gsensor_mutex);
		return -EINVAL;
	}
	else if (len > C_I2C_FIFO_SIZE) 
	{
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&gsensor_mutex);
		return -EINVAL;
	}
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) 
	{
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, err);
		err = -EIO;
	} 
	else 
	{
		err = 0;
	}
	mutex_unlock(&gsensor_mutex);
	return err;

}
EXPORT_SYMBOL(bma_i2c_read_block);
static int bma_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int err, idx, num;
    char buf[C_I2C_FIFO_SIZE];
    err =0;
	mutex_lock(&gsensor_mutex);
    if (!client)
    {
        mutex_unlock(&gsensor_mutex);
        return -EINVAL;
    }
    else if (len >= C_I2C_FIFO_SIZE) 
	{        
        GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&gsensor_mutex);
        return -EINVAL;
    }    

    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++)
    {
        buf[num++] = data[idx];
    }

    err = i2c_master_send(client, buf, num);
    if (err < 0)
	{
        GSE_ERR("send command error!!\n");
		mutex_unlock(&gsensor_mutex);
        return -EFAULT;
    } 
	mutex_unlock(&gsensor_mutex);
    return err;
}
EXPORT_SYMBOL(bma_i2c_write_block);

/*--------------------BMA2XX power control function----------------------------------/
static void BMA2XX_power(struct acc_hw *hw, unsigned int on) 
{
}
/----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int BMA2XX_SetDataResolution(struct bma2xx_i2c_data *obj)
{

/*set g sensor dataresolution here*/

/*BMA2XX only can set to 10-bit dataresolution, so do nothing in bma2xx driver here*/

/*end of set dataresolution*/


 
 /*we set measure range from -2g to +2g in BMA2XX_SetDataFormat(client, BMA2XX_RANGE_2G), 
                                                    and set 10-bit dataresolution BMA2XX_SetDataResolution()*/
                                                    
 /*so bma2xx_data_resolution[0] set value as {{ 3, 9}, 256} when declaration, and assign the value to obj->reso here*/  

 	obj->reso = &bma2xx_data_resolution[obj->sensor_type];
	return 0;
	
/*if you changed the measure range, for example call: BMA2XX_SetDataFormat(client, BMA2XX_RANGE_4G), 
you must set the right value to bma2xx_data_resolution*/

}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadData(struct i2c_client *client, s16 data[BMA2XX_AXES_NUM])
{
	struct bma2xx_i2c_data *priv = i2c_get_clientdata(client);        
	int err = 0;
    u8 addr = BMA2XX_REG_DATAXLOW;
    u8 buf[BMA2XX_DATA_LEN] = {0};

	if(NULL == client)
	{
		err = -EINVAL;
	}
	else if((err = bma_i2c_read_block(client, addr, buf, 6))!=0)
	{
		GSE_ERR("error: %d\n", err);
	}
	else
	{
		switch(priv->sensor_type) {
			case BMA222E_TYPE:
				data[BMA2XX_AXIS_X] =
					(s16)(((u16)buf[1]) << 8 | buf[0]) >> 8;
				data[BMA2XX_AXIS_Y] =
					(s16)(((u16)buf[3]) << 8 | buf[2]) >> 8;
				data[BMA2XX_AXIS_Z] =
					(s16)(((u16)buf[5]) << 8 | buf[4]) >> 8;
				break;
			case BMA250E_TYPE:
				/* 10 bit */
				data[BMA2XX_AXIS_X] =
					(s16)(((u16)buf[1]) << 8 | buf[0]) >> 6;
				data[BMA2XX_AXIS_Y] =
					(s16)(((u16)buf[3]) << 8 | buf[2]) >> 6;
				data[BMA2XX_AXIS_Z] =
					(s16)(((u16)buf[5]) << 8 | buf[4]) >> 6;
				break;
			case BMA255_TYPE:
				/* 12 bit */
				data[BMA2XX_AXIS_X] =
					(s16)(((u16)buf[1]) << 8 | buf[0]) >> 4;
				data[BMA2XX_AXIS_Y] =
					(s16)(((u16)buf[3]) << 8 | buf[2]) >> 4;
				data[BMA2XX_AXIS_Z] =
					(s16)(((u16)buf[5]) << 8 | buf[4]) >> 4;
				break;
			case BMA280_TYPE:
				/* 14 bit */
				data[BMA2XX_AXIS_X] =                                                          
					(s16)(((u16)buf[1]) << 8 | buf[0]) >> 2;                    
				data[BMA2XX_AXIS_Y] =                                                          
					(s16)(((u16)buf[3]) << 8 | buf[2]) >> 2;                    
				data[BMA2XX_AXIS_Z] =                                                          
					(s16)(((u16)buf[5]) << 8 | buf[4]) >> 2;
				break;
			default:
				GSE_ERR("Unsupport sensor_type %d, error!!\n ", priv->sensor_type);
		}

		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] before\n", data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z],
		                               data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z]);
		}

#ifdef CONFIG_BMA2XX_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][BMA2XX_AXIS_X] = data[BMA2XX_AXIS_X];
					priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Y] = data[BMA2XX_AXIS_Y];
					priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Z] = data[BMA2XX_AXIS_Z];
					priv->fir.sum[BMA2XX_AXIS_X] += data[BMA2XX_AXIS_X];
					priv->fir.sum[BMA2XX_AXIS_Y] += data[BMA2XX_AXIS_Y];
					priv->fir.sum[BMA2XX_AXIS_Z] += data[BMA2XX_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][BMA2XX_AXIS_X], priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Y], priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Z],
							priv->fir.sum[BMA2XX_AXIS_X], priv->fir.sum[BMA2XX_AXIS_Y], priv->fir.sum[BMA2XX_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[BMA2XX_AXIS_X] -= priv->fir.raw[idx][BMA2XX_AXIS_X];
					priv->fir.sum[BMA2XX_AXIS_Y] -= priv->fir.raw[idx][BMA2XX_AXIS_Y];
					priv->fir.sum[BMA2XX_AXIS_Z] -= priv->fir.raw[idx][BMA2XX_AXIS_Z];
					priv->fir.raw[idx][BMA2XX_AXIS_X] = data[BMA2XX_AXIS_X];
					priv->fir.raw[idx][BMA2XX_AXIS_Y] = data[BMA2XX_AXIS_Y];
					priv->fir.raw[idx][BMA2XX_AXIS_Z] = data[BMA2XX_AXIS_Z];
					priv->fir.sum[BMA2XX_AXIS_X] += data[BMA2XX_AXIS_X];
					priv->fir.sum[BMA2XX_AXIS_Y] += data[BMA2XX_AXIS_Y];
					priv->fir.sum[BMA2XX_AXIS_Z] += data[BMA2XX_AXIS_Z];
					priv->fir.idx++;
					data[BMA2XX_AXIS_X] = priv->fir.sum[BMA2XX_AXIS_X]/firlen;
					data[BMA2XX_AXIS_Y] = priv->fir.sum[BMA2XX_AXIS_Y]/firlen;
					data[BMA2XX_AXIS_Z] = priv->fir.sum[BMA2XX_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][BMA2XX_AXIS_X], priv->fir.raw[idx][BMA2XX_AXIS_Y], priv->fir.raw[idx][BMA2XX_AXIS_Z],
						priv->fir.sum[BMA2XX_AXIS_X], priv->fir.sum[BMA2XX_AXIS_Y], priv->fir.sum[BMA2XX_AXIS_Z],
						data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}

	return err;
}
/*----------------------------------------------------------------------------*/

static int BMA2XX_ReadOffset(struct i2c_client *client, s8 ofs[BMA2XX_AXES_NUM])
{    
	int err;
	err = 0;

	
#ifdef SW_CALIBRATION
	ofs[0]=ofs[1]=ofs[2]=0x0;
#else
	if((err = bma_i2c_read_block(client, BMA2XX_REG_OFSX, ofs, BMA2XX_AXES_NUM)))
	{
		GSE_ERR("error: %d\n", err);
	}
#endif
	//printk("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]);
	
	return err;    
}

/*----------------------------------------------------------------------------*/
static int BMA2XX_ResetCalibration(struct i2c_client *client)
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	//u8 ofs[4]={0,0,0,0};
	int err = 0;
#ifdef GSENSOR_UT
		GSE_FUN();
#endif
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;    
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadCalibration(struct i2c_client *client, int dat[BMA2XX_AXES_NUM])
{
    struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
    int  err = 0;
    int mul;

    GSE_FUN();
	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
	    if ((err = BMA2XX_ReadOffset(client, obj->offset))) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    	}    
    	mul = obj->reso->sensitivity/bma2xx_offset_resolution[obj->sensor_type].sensitivity;
	#endif

    dat[obj->cvt.map[BMA2XX_AXIS_X]] = obj->cvt.sign[BMA2XX_AXIS_X]*(obj->offset[BMA2XX_AXIS_X]*mul + obj->cali_sw[BMA2XX_AXIS_X]);
    dat[obj->cvt.map[BMA2XX_AXIS_Y]] = obj->cvt.sign[BMA2XX_AXIS_Y]*(obj->offset[BMA2XX_AXIS_Y]*mul + obj->cali_sw[BMA2XX_AXIS_Y]);
    dat[obj->cvt.map[BMA2XX_AXIS_Z]] = obj->cvt.sign[BMA2XX_AXIS_Z]*(obj->offset[BMA2XX_AXIS_Z]*mul + obj->cali_sw[BMA2XX_AXIS_Z]);                        
                                       
    return err;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadCalibrationEx(struct i2c_client *client, int act[BMA2XX_AXES_NUM], int raw[BMA2XX_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;
	err = 0;


	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
		if((err = BMA2XX_ReadOffset(client, obj->offset)))
		{
			GSE_ERR("read offset fail, %d\n", err);
			return err;
		}   
		mul = obj->reso->sensitivity/bma2xx_offset_resolution[obj->sensor_type].sensitivity;
	#endif
	
	raw[BMA2XX_AXIS_X] = obj->offset[BMA2XX_AXIS_X]*mul + obj->cali_sw[BMA2XX_AXIS_X];
	raw[BMA2XX_AXIS_Y] = obj->offset[BMA2XX_AXIS_Y]*mul + obj->cali_sw[BMA2XX_AXIS_Y];
	raw[BMA2XX_AXIS_Z] = obj->offset[BMA2XX_AXIS_Z]*mul + obj->cali_sw[BMA2XX_AXIS_Z];

	act[obj->cvt.map[BMA2XX_AXIS_X]] = obj->cvt.sign[BMA2XX_AXIS_X]*raw[BMA2XX_AXIS_X];
	act[obj->cvt.map[BMA2XX_AXIS_Y]] = obj->cvt.sign[BMA2XX_AXIS_Y]*raw[BMA2XX_AXIS_Y];
	act[obj->cvt.map[BMA2XX_AXIS_Z]] = obj->cvt.sign[BMA2XX_AXIS_Z]*raw[BMA2XX_AXIS_Z];                        
	                       
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_WriteCalibration(struct i2c_client *client, int dat[BMA2XX_AXES_NUM])
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[BMA2XX_AXES_NUM], raw[BMA2XX_AXES_NUM];

	if(0 != (err = BMA2XX_ReadCalibrationEx(client, cali, raw)))	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		raw[BMA2XX_AXIS_X], raw[BMA2XX_AXIS_Y], raw[BMA2XX_AXIS_Z],
		obj->offset[BMA2XX_AXIS_X], obj->offset[BMA2XX_AXIS_Y], obj->offset[BMA2XX_AXIS_Z],
		obj->cali_sw[BMA2XX_AXIS_X], obj->cali_sw[BMA2XX_AXIS_Y], obj->cali_sw[BMA2XX_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[BMA2XX_AXIS_X] += dat[BMA2XX_AXIS_X];
	cali[BMA2XX_AXIS_Y] += dat[BMA2XX_AXIS_Y];
	cali[BMA2XX_AXIS_Z] += dat[BMA2XX_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
		dat[BMA2XX_AXIS_X], dat[BMA2XX_AXIS_Y], dat[BMA2XX_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[BMA2XX_AXIS_X] = obj->cvt.sign[BMA2XX_AXIS_X]*(cali[obj->cvt.map[BMA2XX_AXIS_X]]);
	obj->cali_sw[BMA2XX_AXIS_Y] = obj->cvt.sign[BMA2XX_AXIS_Y]*(cali[obj->cvt.map[BMA2XX_AXIS_Y]]);
	obj->cali_sw[BMA2XX_AXIS_Z] = obj->cvt.sign[BMA2XX_AXIS_Z]*(cali[obj->cvt.map[BMA2XX_AXIS_Z]]);	
#else
	int divisor = obj->reso->sensitivity/lsb;//modified
	obj->offset[BMA2XX_AXIS_X] = (s8)(obj->cvt.sign[BMA2XX_AXIS_X]*(cali[obj->cvt.map[BMA2XX_AXIS_X]])/(divisor));
	obj->offset[BMA2XX_AXIS_Y] = (s8)(obj->cvt.sign[BMA2XX_AXIS_Y]*(cali[obj->cvt.map[BMA2XX_AXIS_Y]])/(divisor));
	obj->offset[BMA2XX_AXIS_Z] = (s8)(obj->cvt.sign[BMA2XX_AXIS_Z]*(cali[obj->cvt.map[BMA2XX_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[BMA2XX_AXIS_X] = obj->cvt.sign[BMA2XX_AXIS_X]*(cali[obj->cvt.map[BMA2XX_AXIS_X]])%(divisor);
	obj->cali_sw[BMA2XX_AXIS_Y] = obj->cvt.sign[BMA2XX_AXIS_Y]*(cali[obj->cvt.map[BMA2XX_AXIS_Y]])%(divisor);
	obj->cali_sw[BMA2XX_AXIS_Z] = obj->cvt.sign[BMA2XX_AXIS_Z]*(cali[obj->cvt.map[BMA2XX_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		obj->offset[BMA2XX_AXIS_X]*divisor + obj->cali_sw[BMA2XX_AXIS_X], 
		obj->offset[BMA2XX_AXIS_Y]*divisor + obj->cali_sw[BMA2XX_AXIS_Y], 
		obj->offset[BMA2XX_AXIS_Z]*divisor + obj->cali_sw[BMA2XX_AXIS_Z], 
		obj->offset[BMA2XX_AXIS_X], obj->offset[BMA2XX_AXIS_Y], obj->offset[BMA2XX_AXIS_Z],
		obj->cali_sw[BMA2XX_AXIS_X], obj->cali_sw[BMA2XX_AXIS_Y], obj->cali_sw[BMA2XX_AXIS_Z]);

	if((err = bma_i2c_write_block(obj->client, BMA2XX_REG_OFSX, obj->offset, BMA2XX_AXES_NUM)))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif
	mdelay(1);
	return err;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[2]={0};    
	int res = 0;
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	
	
	res = bma_i2c_read_block(client,BMA2XX_REG_DEVID,databuf,0x1);
	if(res < 0)
	{
		goto exit_BMA2XX_CheckDeviceID;
	}

	switch(databuf[0])  {
		case BMA222E_CHIP_ID:
			obj->sensor_type = BMA222E_TYPE;
			break;
		case BMA250E_CHIP_ID:
			obj->sensor_type = BMA250E_TYPE;
			break;
		case BMA255_CHIP_ID:
			obj->sensor_type = BMA255_TYPE;
			break;
		case BMA280_CHIP_ID:
			obj->sensor_type = BMA280_TYPE;
			break;
		default:
			obj->sensor_type = UNSUPPORT_TYPE;
			goto exit_BMA2XX_CheckDeviceID;
	}

	GSE_LOG("BMA2XX_CheckDeviceID %d done, the sensor is %s\n ",
			databuf[0], sensor_type_map[obj->sensor_type].sensor_name);

	exit_BMA2XX_CheckDeviceID:
	if (res < 0)
	{
		GSE_ERR("BMA2XX_CheckDeviceID %d failt!\n ", BMA2XX_ERR_I2C);
		return BMA2XX_ERR_I2C;
	}
	mdelay(1);
	return BMA2XX_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_SetPowerMode(struct i2c_client *client, bool enable)
{
    struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
	
    u8 databuf[2];
	u8 addr = BMA2XX_REG_POWER_CTL;

	if(enable == sensor_power )
	{
		GSE_LOG("Sensor power status is newest!\n");
		return BMA2XX_SUCCESS;
	}

	if(bma_i2c_read_block(client, addr, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");
		return BMA2XX_ERR_I2C;
	}
	GSE_LOG("set power mode value = 0x%x!\n",databuf[0]);
	mdelay(1);
	if(enable == true)
	{
		databuf[0] &= ~BMA2XX_MEASURE_MODE;
	}
	else
	{
		databuf[0] |= BMA2XX_MEASURE_MODE;
	}
	
	res = bma_i2c_write_block(client,BMA2XX_REG_POWER_CTL,databuf,0x1);
	if(res < 0)
	{
		GSE_LOG("set power mode failed!\n");
		return BMA2XX_ERR_I2C;
	}
	else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
	{
		GSE_LOG("set power mode ok %d!\n", databuf[1]);
	}

		sensor_power = enable;

	mdelay(1);
	//GSE_LOG("leave Sensor power status is sensor_power = %d\n",sensor_power);
	return BMA2XX_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10]={0};    
	int res = 0;
   
	if(bma_i2c_read_block(client, BMA2XX_REG_DATA_FORMAT, databuf, 0x01))
	{
		printk("bma2xx read Dataformat failt \n");
		return BMA2XX_ERR_I2C;
	}
	mdelay(1);
	databuf[0] &= ~BMA2XX_RANGE_MASK;
	databuf[0] |= dataformat;
	
	res = bma_i2c_write_block(client,BMA2XX_REG_DATA_FORMAT,databuf,0x1);
	if(res < 0)
	{
		return BMA2XX_ERR_I2C;
	}
	
	//printk("BMA2XX_SetDataFormat OK! \n");
	mdelay(1);
	return BMA2XX_SetDataResolution(obj);    
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10]={0};    
	int res = 0;

	if(bma_i2c_read_block(client, BMA2XX_REG_BW_RATE, databuf, 0x01))
	{
		printk("bma2xx read rate failt \n");
		return BMA2XX_ERR_I2C;
	}
	mdelay(1);
	databuf[0] &= ~BMA2XX_BW_MASK;
	databuf[0] |= bwrate;
	

    res = bma_i2c_write_block(client,BMA2XX_REG_BW_RATE,databuf,0x1);
	if(res < 0)
	{
		return BMA2XX_ERR_I2C;
	}
	mdelay(1);
	//printk("BMA2XX_SetBWRate OK! \n");
	
	return BMA2XX_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_SetIntEnable(struct i2c_client *client, u8 intenable)
{
			//u8 databuf[10];    
			//int res = 0;
		
			
			//printk("BMA2XX disable interrupt ...\n");
		
			/*for disable interrupt function*/
			mdelay(1);
			return BMA2XX_SUCCESS;	  
}

/*----------------------------------------------------------------------------*/
static int bma2xx_init_client(struct i2c_client *client, int reset_cali)
{
#ifdef CONFIG_BMA2XX_LOWPASS
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
#endif
	int res = 0;

    GSE_FUN();


	res = BMA2XX_CheckDeviceID(client); 
	if(res != BMA2XX_SUCCESS)
	{
		return res;
	}	
	//printk("BMA2XX_CheckDeviceID ok \n");
	
	res = BMA2XX_SetBWRate(client, BMA2XX_BW_100HZ);
	if(res != BMA2XX_SUCCESS ) 
	{
		return res;
	}
	//printk("BMA2XX_SetBWRate OK!\n");
	
	res = BMA2XX_SetDataFormat(client, BMA2XX_RANGE_2G);
	if(res != BMA2XX_SUCCESS) 
	{
		return res;
	}
	//printk("BMA2XX_SetDataFormat OK!\n");

	//gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	res = BMA2XX_SetIntEnable(client, 0x00);        
	if(res != BMA2XX_SUCCESS)
	{
		return res;
	}
	//printk("BMA2XX disable interrupt function!\n");
	
	res = BMA2XX_SetPowerMode(client, false);//
		if(res != BMA2XX_SUCCESS)
		{
			return res;
		}
	//printk("BMA2XX_SetPowerMode OK!\n");


	if(0 != reset_cali)
	{ 
		/*reset calibration only in power on*/
		res = BMA2XX_ResetCalibration(client);
		if(res != BMA2XX_SUCCESS)
		{
			return res;
		}
	}
	GSE_LOG("bma2xx_init_client OK!\n");
#ifdef CONFIG_BMA2XX_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif
	return BMA2XX_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "BMA2XX Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct bma2xx_i2c_data *obj = (struct bma2xx_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[BMA2XX_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}
		
	if(sensor_suspend == 1)
	{
		//GSE_LOG("sensor in suspend read not data!\n");
		return 0;
	}

	if((res = BMA2XX_ReadData(client, obj->data))!=0)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		//GSE_LOG("raw data x=%d, y=%d, z=%d \n",obj->data[BMA2XX_AXIS_X],obj->data[BMA2XX_AXIS_Y],obj->data[BMA2XX_AXIS_Z]);
		obj->data[BMA2XX_AXIS_X] += obj->cali_sw[BMA2XX_AXIS_X];
		obj->data[BMA2XX_AXIS_Y] += obj->cali_sw[BMA2XX_AXIS_Y];
		obj->data[BMA2XX_AXIS_Z] += obj->cali_sw[BMA2XX_AXIS_Z];
		
		//printk("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[BMA2XX_AXIS_X],obj->cali_sw[BMA2XX_AXIS_Y],obj->cali_sw[BMA2XX_AXIS_Z]);
		
		/*remap coordinate*/
		acc[obj->cvt.map[BMA2XX_AXIS_X]] = obj->cvt.sign[BMA2XX_AXIS_X]*obj->data[BMA2XX_AXIS_X];
		acc[obj->cvt.map[BMA2XX_AXIS_Y]] = obj->cvt.sign[BMA2XX_AXIS_Y]*obj->data[BMA2XX_AXIS_Y];
		acc[obj->cvt.map[BMA2XX_AXIS_Z]] = obj->cvt.sign[BMA2XX_AXIS_Z]*obj->data[BMA2XX_AXIS_Z];
		//printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[BMA2XX_AXIS_X],obj->cvt.sign[BMA2XX_AXIS_Y],obj->cvt.sign[BMA2XX_AXIS_Z]);


		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[BMA2XX_AXIS_X], acc[BMA2XX_AXIS_Y], acc[BMA2XX_AXIS_Z]);

		//Out put the mg
		//printk("mg acc=%d, GRAVITY=%d, sensityvity=%d \n",acc[BMA2XX_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);
		acc[BMA2XX_AXIS_X] = acc[BMA2XX_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA2XX_AXIS_Y] = acc[BMA2XX_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA2XX_AXIS_Z] = acc[BMA2XX_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		

		sprintf(buf, "%04x %04x %04x", acc[BMA2XX_AXIS_X], acc[BMA2XX_AXIS_Y], acc[BMA2XX_AXIS_Z]);
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		{
			GSE_LOG("gsensor data: %s!\n", buf);
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------
static int BMA2XX_ReadRawData(struct i2c_client *client, char *buf)
{
	struct bma2xx_i2c_data *obj = (struct bma2xx_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
		return EINVAL;
	}
	
	if(0 != (res = BMA2XX_ReadData(client, obj->data)))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "BMA2XX_ReadRawData %04x %04x %04x", obj->data[BMA2XX_AXIS_X], 
			obj->data[BMA2XX_AXIS_Y], obj->data[BMA2XX_AXIS_Z]);
	
	}
	
	return 0;
}
----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = obj_i2c_data->client;
	char strbuf[BMA2XX_BUFSIZE];
    GSE_LOG("attr show chipinfo value.......maybe AT+GETSENSORHUB\n");
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	BMA2XX_ReadChipInfo(client, strbuf, BMA2XX_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

#if 0
static ssize_t gsensor_init(struct device_driver *ddri, char *buf, size_t count)
	{
		struct i2c_client *client = bma2xx_i2c_client;
		char strbuf[BMA2XX_BUFSIZE];
		
		if(NULL == client)
		{
			GSE_ERR("i2c client is null!!\n");
			return 0;
		}
		bma2xx_init_client(client, 1);
		return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
	}
#endif


/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = obj_i2c_data->client;
	char strbuf[BMA2XX_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	BMA2XX_ReadSensorData(client, strbuf, BMA2XX_BUFSIZE);
	//BMA2XX_ReadRawData(client, strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}

#if 0
static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf, size_t count)
	{
		struct i2c_client *client = bma2xx_i2c_client;
		char strbuf[BMA2XX_BUFSIZE];
		
		if(NULL == client)
		{
			GSE_ERR("i2c client is null!!\n");
			return 0;
		}
		//BMA2XX_ReadSensorData(client, strbuf, BMA2XX_BUFSIZE);
		BMA2XX_ReadRawData(client, strbuf);
		return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
	}
#endif

/*----------------------------------------------------------------------------*/
//acc calibrate, add by zte dinggaoshan 20171229  --start--
static int is_acc_calibrate_data_valid(int sensitivity, int cali_value[BMA2XX_AXES_NUM])
{
	// cali_value's unit is mg.
	int offset = (sensitivity * 3) / 10;
	if ((abs(cali_value[BMA2XX_AXIS_X]) > offset) 
		|| (abs(cali_value[BMA2XX_AXIS_Y]) > offset)
		|| (abs(cali_value[BMA2XX_AXIS_Z]) > offset))
	{
		return 0;
	}
	return 1;
}

static int acc_calibrate_read(struct bma2xx_i2c_data *obj, char *buf)
{	
	s16 rawacc[BMA2XX_AXES_NUM] = {0};
	s16 raw_sum[BMA2XX_AXES_NUM] = {0};
	int cali_data[BMA2XX_AXES_NUM] = {0};
	int i = 0;
	int res = 0;
    int cnt = 0;

	GSE_FUN();	

	if(sensor_power == false)
	{
		res = BMA2XX_SetPowerMode(obj->client, true);
		if(res)
		{
			GSE_ERR("Power on bma2xx error %d!\n", res);
		}
		msleep(20);
	}
    for (i = 0; i < BMA2XX_CALI_LEN; i++)
    {
		res = BMA2XX_ReadData(obj->client, rawacc);
		if(res < 0)
		{        
			GSE_ERR("I2C error at %d: ret value=%d", i, res);
			return -3;
		}
		raw_sum[BMA2XX_AXIS_X] += rawacc[BMA2XX_AXIS_X];
		raw_sum[BMA2XX_AXIS_Y] += rawacc[BMA2XX_AXIS_Y];
		raw_sum[BMA2XX_AXIS_Z] += rawacc[BMA2XX_AXIS_Z];
		msleep(10);
    }
	// calc avarage
	GSE_LOG("sum,x:%d,y:%d,z:%d\n",raw_sum[BMA2XX_AXIS_X],raw_sum[BMA2XX_AXIS_Y],raw_sum[BMA2XX_AXIS_Z]);
	rawacc[BMA2XX_AXIS_X] = raw_sum[BMA2XX_AXIS_X] / BMA2XX_CALI_LEN;
	rawacc[BMA2XX_AXIS_Y] = raw_sum[BMA2XX_AXIS_Y] / BMA2XX_CALI_LEN;
	rawacc[BMA2XX_AXIS_Z] = raw_sum[BMA2XX_AXIS_Z] / BMA2XX_CALI_LEN;
	GSE_LOG("avg,x:%d,y:%d,z:%d\n",rawacc[BMA2XX_AXIS_X],rawacc[BMA2XX_AXIS_Y],rawacc[BMA2XX_AXIS_Z]);

	cali_data[obj->cvt.map[BMA2XX_AXIS_X]] = 0 - obj->cvt.sign[BMA2XX_AXIS_X] * rawacc[BMA2XX_AXIS_X];
	cali_data[obj->cvt.map[BMA2XX_AXIS_Y]] = 0 - obj->cvt.sign[BMA2XX_AXIS_Y] * rawacc[BMA2XX_AXIS_Y];
	cali_data[obj->cvt.map[BMA2XX_AXIS_Z]] = obj->reso->sensitivity - obj->cvt.sign[BMA2XX_AXIS_Z] * rawacc[BMA2XX_AXIS_Z];
    GSE_LOG("calidata,x:%d,y:%d,z:%d\n",cali_data[BMA2XX_AXIS_X],cali_data[BMA2XX_AXIS_Y],cali_data[BMA2XX_AXIS_Z]);
	
	if (0 == is_acc_calibrate_data_valid(obj->reso->sensitivity, cali_data))
	{
		GSE_ERR("calibration value invalid[X:%d,Y:%d,Z:%d].", cali_data[BMA2XX_AXIS_X], 
			cali_data[BMA2XX_AXIS_Y], cali_data[BMA2XX_AXIS_Z]);
		return -2;
	}
    
	cnt = sprintf(buf, "%d %d %d %d", cali_data[BMA2XX_AXIS_X], cali_data[BMA2XX_AXIS_Y], cali_data[BMA2XX_AXIS_Z],
        cali_data[BMA2XX_AXIS_X] + cali_data[BMA2XX_AXIS_Y] + cali_data[BMA2XX_AXIS_Z]);
    
	BMA2XX_ResetCalibration(obj->client);
	BMA2XX_WriteCalibration(obj->client, cali_data);
	
	GSE_LOG("cnt:%d, buffer:%s\n",cnt, buf);
	return cnt;
}
//acc calibrate, add by zte dinggaoshan 20171229  --end--
/*----------------------------------------------------------------------------*/
#if 1
static ssize_t show_cali_value(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	int err, len = 0, mul;
	int tmp[BMA2XX_AXES_NUM];

	if(NULL == obj->client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	if(0 != (err = BMA2XX_ReadOffset(obj->client, obj->offset)))
	{
		return -EINVAL;
	}
	else if(0 != (err = BMA2XX_ReadCalibration(obj->client, tmp)))
	{
		return -EINVAL;
	}
	else
	{    
		mul = obj->reso->sensitivity/bma2xx_offset_resolution[obj->sensor_type].sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[BMA2XX_AXIS_X], obj->offset[BMA2XX_AXIS_Y], obj->offset[BMA2XX_AXIS_Z],
			obj->offset[BMA2XX_AXIS_X], obj->offset[BMA2XX_AXIS_Y], obj->offset[BMA2XX_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[BMA2XX_AXIS_X], obj->cali_sw[BMA2XX_AXIS_Y], obj->cali_sw[BMA2XX_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[BMA2XX_AXIS_X]*mul + obj->cali_sw[BMA2XX_AXIS_X],
			obj->offset[BMA2XX_AXIS_Y]*mul + obj->cali_sw[BMA2XX_AXIS_Y],
			obj->offset[BMA2XX_AXIS_Z]*mul + obj->cali_sw[BMA2XX_AXIS_Z],
			tmp[BMA2XX_AXIS_X], tmp[BMA2XX_AXIS_Y], tmp[BMA2XX_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = obj_i2c_data->client; 
	int err, x, y, z;
	int dat[BMA2XX_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if(0 != (err = BMA2XX_ResetCalibration(client)))
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[BMA2XX_AXIS_X] = x;
		dat[BMA2XX_AXIS_Y] = y;
		dat[BMA2XX_AXIS_Z] = z;
		if(0 != (err = BMA2XX_WriteCalibration(client, dat)))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return count;
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_BMA2XX_LOWPASS
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][BMA2XX_AXIS_X], obj->fir.raw[idx][BMA2XX_AXIS_Y], obj->fir.raw[idx][BMA2XX_AXIS_Z]);
		}
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[BMA2XX_AXIS_X], obj->fir.sum[BMA2XX_AXIS_Y], obj->fir.sum[BMA2XX_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[BMA2XX_AXIS_X]/len, obj->fir.sum[BMA2XX_AXIS_Y]/len, obj->fir.sum[BMA2XX_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef CONFIG_BMA2XX_LOWPASS
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t res;
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	
	u8 databuf[2];    
	//int res = 0;
	u8 addr = BMA2XX_REG_POWER_CTL;
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	if(bma_i2c_read_block(obj->client, addr, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");
		return 1;
	}
	
	if(sensor_power)
		GSE_LOG("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		GSE_LOG("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return snprintf(buf, PAGE_SIZE, "%x\n", databuf[0]);
}
static ssize_t show_chip_orientation(struct device *dev,
				struct device_attribute *attr, char *pbBuf)
{
    ssize_t          _tLength = 0;
    struct bma2xx_i2c_data *obj = obj_i2c_data;

    GSE_LOG("[%s] default direction: %d\n", __FUNCTION__, obj->hw->direction);

    _tLength = snprintf(pbBuf, PAGE_SIZE, "default direction = %d\n", obj->hw->direction);

    return (_tLength);
}

static ssize_t store_chip_orientation(struct device *dev,
				struct device_attribute *attr, const char *pbBuf, size_t tCount)
{
    int                       _nDirection = 0;
    struct bma2xx_i2c_data   *_pt_i2c_obj = obj_i2c_data;

    if (NULL == _pt_i2c_obj)
        return (0);

    if (1 == sscanf(pbBuf, "%d", &_nDirection))
    {
        _pt_i2c_obj->hw->direction = _nDirection;
    }

    GSE_LOG("[%s] set direction: %d\n", __FUNCTION__, _nDirection);

    return (tCount);
}

static ssize_t show_acc_delay(struct device *dev,
				struct device_attribute *attr, char *pbBuf)
{
    struct bma2xx_i2c_data *obj = obj_i2c_data;
    return snprintf(pbBuf, PAGE_SIZE, "delay = %dms\n", obj->hw->delay);
}
/*---------------------------------------------------------------------------*/
//acc calibrate, add by zte dinggaoshan 20171229  --start--
static ssize_t bma_accel_get_cali(struct device *dev,
				struct device_attribute *attr, char *buf)
{
    char cali[32] = {'\0'};
    struct bma2xx_i2c_data *obj = obj_i2c_data;
    struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos;
    
    acc_calibrate_read(obj, cali);
    fp = filp_open(CAL_PATH, O_CREAT|O_RDWR|O_SYNC, S_IRWXU|S_IRGRP|S_IROTH);       //Open or create file
    if(IS_ERR(fp))
    {
        GSE_ERR("%s: Create calibration file fail.\n", __func__);
        return -1;
    }
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;        //Write position start from 0
    vfs_write(fp, cali, strlen(cali)+1, &pos);
    filp_close(fp, NULL);
    set_fs(old_fs);
    
    return sprintf(buf, "%d\n", 0);
}

static ssize_t bma_accel_set_cali(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma2xx_i2c_data *obj = obj_i2c_data;
    struct i2c_client *client = obj->client; 
    int cali_data[BMA2XX_AXES_NUM] = {0};
		
	sscanf(buf, "%d %d %d", &cali_data[BMA2XX_AXIS_X], &cali_data[BMA2XX_AXIS_Y], &cali_data[BMA2XX_AXIS_Z]);
	GSE_LOG("[x:%d,y:%d,z:%d], count=%d\n", cali_data[BMA2XX_AXIS_X],
		cali_data[BMA2XX_AXIS_Y],cali_data[BMA2XX_AXIS_Z], (int)count);
	
	//check........
	if (0 == is_acc_calibrate_data_valid(obj->reso->sensitivity, cali_data))
	{
		GSE_ERR("calibration value invalid[X:%d,Y:%d,Z:%d].", cali_data[BMA2XX_AXIS_X], 
			cali_data[BMA2XX_AXIS_Y], cali_data[BMA2XX_AXIS_Z]);
		return -EFAULT;
	}

	BMA2XX_ResetCalibration(client);
	BMA2XX_WriteCalibration(client, cali_data);
    return count;
}

static ssize_t bma_accel_dev_name_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return sprintf(buf, "%s\n", BMA2XX_DEV_NAME);
}
//acc calibrate, add by zte dinggaoshan 20171229  --end--
/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(chipinfo,   S_IWUSR | S_IRUGO, show_chipinfo_value,      NULL);
static DEVICE_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value,    NULL);
static DEVICE_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DEVICE_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DEVICE_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DEVICE_ATTR(powerstatus,          S_IRUGO, show_power_status_value,  NULL);
static DEVICE_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation, store_chip_orientation);
static DEVICE_ATTR(delay,      S_IRUGO,            show_acc_delay,           NULL);
static DEVICE_ATTR(enable_cali, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, bma_accel_get_cali, bma_accel_set_cali);
static DEVICE_ATTR(vendor_name, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, bma_accel_dev_name_show, NULL);

/*----------------------------------------------------------------------------*/
static struct attribute *bma2xx_attributes[] = {
	&dev_attr_chipinfo.attr,     /*chip information*/
	&dev_attr_sensordata.attr,   /*dump sensor data*/
	&dev_attr_cali.attr,         /*show calibration data*/
	&dev_attr_firlen.attr,       /*filter length: 0: disable, others: enable*/
	&dev_attr_trace.attr,        /*trace log*/
	&dev_attr_powerstatus.attr,
	&dev_attr_orientation.attr,
	&dev_attr_delay.attr,
	NULL
};

static struct attribute_group acc_attribute_group = {
    .attrs = bma2xx_attributes
};

static struct attribute *bma2xx_input_attributes[] = {
    &dev_attr_enable_cali.attr,
	&dev_attr_vendor_name.attr,
	NULL
};

static struct attribute_group bma2xx_input_sys_group = {
    .attrs = bma2xx_input_attributes
};
/*----------------------------------------------------------------------------*/
#if 0
static struct class *bma2xx_class;
struct device *bma2xx_dev;
static ssize_t bma2xx_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "Get sensor:%s\n", BMA2XX_DEV_NAME); 
}
static DEVICE_ATTR(sensor_info, S_IRUGO, bma2xx_info_show, NULL);

void create_atcmd_test_sys(void)
{
    bma2xx_class = class_create(THIS_MODULE, "sensor");
    if (bma2xx_class < 0)
    {
        GSE_ERR("Failed to create class(SENSOR)!\n");
    }
    bma2xx_dev = device_create(bma2xx_class, NULL, 0, NULL, "gsensor");
    if (bma2xx_dev < 0)
    {
        GSE_ERR("Failed to create device(sensor/device)!\n");
    }
    if (device_create_file(bma2xx_dev, &dev_attr_sensor_info) < 0)
    {
        GSE_ERR("Failed to create device file(%s)!\n", dev_attr_sensor_info.attr.name);
    }
}
#endif
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int bma2xx_open(struct inode *inode, struct file *file)
{
	file->private_data = obj_i2c_data->client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int bma2xx_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}


/*
static const struct file_operations acc_calibrate_proc_fops = {
	.owner		= THIS_MODULE,
	.read       = acc_calibrate_read_proc,
	.write      = acc_calibrate_write_proc,
};
*//*
static void create_acc_calibrate_proc_file(void)
{
	acc_calibrate_proc_file = proc_create("driver/acc_calibration", 0666, NULL, &acc_calibrate_proc_fops);
    GSE_FUN(f);

    if(NULL == acc_calibrate_proc_file)
	{
	    GSE_ERR("create acc_calibrate_proc_file fail!\n");
	}
}
//acc calibrate proc, add by zte yanxinghao 20161008  --e-n-d--
----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
//static int bma2xx_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long bma2xx_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)

{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct bma2xx_i2c_data *obj = (struct bma2xx_i2c_data*)i2c_get_clientdata(client);	
	void __user *data;
	long err = 0;
    int enable = 0;
	u64 delay;

    if(NULL == obj){
        GSE_ERR("%s error [NULL==obj] !!", __func__);
        return -EINVAL;
    }

	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}
    
    data = (void __user *) arg;
    if(data == NULL)
    {
        GSE_ERR("invalid argument.");
        return -EINVAL;
    }

	switch(cmd)
	{
		case KIONIX_ACCEL_IOCTL_SET_ENABLE:
			if(copy_from_user(&enable, data, sizeof(enable)))
			{
				GSE_ERR("copy_from_user failed.");
				return -EFAULT;
			}
		    GSE_LOG("%s enable= %d\n",__func__,enable);
            if (enable == 1) {
                gsensor_enable(obj);
            } else if(enable == 0) {
                gsensor_disable(obj);
            } else {
                GSE_ERR("%s: invild enable arg!\n", __func__);
                err = -ENOIOCTLCMD;
            }
            break;
        case KIONIX_ACCEL_IOCTL_GET_ENABLE:
            enable = sensor_power ? 1 : 0;
            if (copy_to_user(data, &enable, sizeof(enable))){
			    return -EINVAL;
            }
		    break;
            break;
        case KIONIX_ACCEL_IOCTL_SET_DELAY:
            if(copy_from_user(&delay, data, sizeof(delay)))
                return -EINVAL;
    		printk("KIONIX_ACCEL_IOCTL_SET_DELAY %u\n",delay);
    		obj->hw->delay = max((unsigned int)delay, obj->hw->min_delay);
            obj->hw->interval = msecs_to_jiffies(obj->hw->delay);
            if (gsensor_set_delay(obj, obj->hw->delay) < 0)
                return -EINVAL;
            break;
        case KIONIX_ACCEL_IOCTL_GET_DELAY:
            if (copy_to_user(data, &obj->hw->delay, sizeof(obj->hw->delay)))
                return -EINVAL;
            break;
		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}


#ifdef CONFIG_COMPAT
static long bma2xx_compat_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
    long err = 0;

	void __user *arg32 = compat_ptr(arg);
	
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	
    switch (cmd)
    {
        case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
		        return err;
		    }
        break;
        case COMPAT_GSENSOR_IOCTL_SET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;
        case COMPAT_GSENSOR_IOCTL_GET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;
        case COMPAT_GSENSOR_IOCTL_CLR_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;

        default:
            GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
        break;

    }

    return err;
}
#endif
/*----------------------------------------------------------------------------*/
static struct file_operations bma2xx_fops = {
	.owner = THIS_MODULE,
	.open = bma2xx_open,
	.release = bma2xx_release,
	.unlocked_ioctl = bma2xx_unlocked_ioctl,
	#ifdef CONFIG_COMPAT
	.compat_ioctl = bma2xx_compat_ioctl,
	#endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice bma2xx_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MISC_DEVICE_NAME,
	.fops = &bma2xx_fops,
};
/*----------------------------------------------------------------------------*/
//#ifndef USE_EARLY_SUSPEND
/*----------------------------------------------------------------------------*/
static int bma2xx_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;  
    mutex_lock(&gsensor_scp_en_mutex);
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			mutex_unlock(&gsensor_scp_en_mutex);
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);
        power_status = sensor_power;
		if(0 != (err = BMA2XX_SetPowerMode(obj->client, false)))
		{
			GSE_ERR("write power control fail!!\n");
			mutex_unlock(&gsensor_scp_en_mutex);
			return -EINVAL;
		} 
	}
	mutex_unlock(&gsensor_scp_en_mutex);
	return err;
}
/*----------------------------------------------------------------------------*/
static int bma2xx_resume(struct i2c_client *client)
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);        
	int err;

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
    if(0 != (err = BMA2XX_SetPowerMode(obj->client, power_status)))
	{
		GSE_ERR("initialize client fail!!\n");
		
		return err;        
	}
	atomic_set(&obj->suspend, 0);
	
	return 0;
}
/*----------------------------------------------------------------------------*/
#if 0 /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void bma2xx_early_suspend(struct early_suspend *h) 
{
	struct bma2xx_i2c_data *obj = container_of(h, struct bma2xx_i2c_data, early_drv);   
	int err;
		
	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1); 
	
	GSE_FUN();  
	u8 databuf[2]; //for debug read power control register to see the value is OK
	if(bma_i2c_read_block(obj->client, BMA2XX_REG_POWER_CTL, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");
		return BMA2XX_ERR_I2C;
	}
	if(databuf[0]==0xff)//if the value is ff the gsensor will not work anymore, any i2c operations won't be vaild
		GSE_LOG("before BMA2XX_SetPowerMode in suspend databuf = 0x%x\n",databuf[0]);
#ifndef CUSTOM_KERNEL_SENSORHUB
	if((err = BMA2XX_SetPowerMode(obj->client, false)))
#else
    if((err = BMA2XX_SCP_SetPowerMode(false, ID_ACCELEROMETER)))
#endif
	{
		GSE_ERR("write power control fail!!\n");

		return;
	}
	if(bma_i2c_read_block(obj->client, BMA2XX_REG_POWER_CTL, databuf, 0x01)) //for debug read power control register to see the value is OK
	{
		GSE_ERR("read power ctl register err!\n");

		return BMA2XX_ERR_I2C;
	}
	if(databuf[0]==0xff)//if the value is ff the gsensor will not work anymore, any i2c operations won't be vaild
		GSE_LOG("after BMA2XX_SetPowerMode suspend err databuf = 0x%x\n",databuf[0]);
	sensor_suspend = 1;
	
#ifndef CUSTOM_KERNEL_SENSORHUB
	BMA2XX_power(obj->hw, 0);
#endif

}
/*----------------------------------------------------------------------------*/
static void bma2xx_late_resume(struct early_suspend *h)
{
	struct bma2xx_i2c_data *obj = container_of(h, struct bma2xx_i2c_data, early_drv);         
	int err;
	
	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

#ifndef CUSTOM_KERNEL_SENSORHUB
	BMA2XX_power(obj->hw, 1);

#endif



	u8 databuf[2];//for debug read power control register to see the value is OK
	if(bma_i2c_read_block(obj->client, BMA2XX_REG_POWER_CTL, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");		

		return BMA2XX_ERR_I2C;
		
	}
	if(databuf[0]==0xff)//if the value is ff the gsensor will not work anymore, any i2c operations won't be vaild
	
		GSE_LOG("before bma2xx_init_client databuf = 0x%x\n",databuf[0]);
#ifndef CUSTOM_KERNEL_SENSORHUB	
	if((err = bma2xx_init_client(obj->client, 0)))
#else
    if((err = BMA2XX_SCP_SetPowerMode(enable_status, ID_ACCELEROMETER)))
#endif
	{
		GSE_ERR("initialize client fail!!\n");

		return;        
	}
	
	if(bma_i2c_read_block(obj->client, BMA2XX_REG_POWER_CTL, databuf, 0x01)) //for debug read power control register to see the value is OK
	{
		GSE_ERR("read power ctl register err!\n");

		return BMA2XX_ERR_I2C;
	}
	
	if(databuf[0]==0xff)//if the value is ff the gsensor will not work anymore, any i2c operations won't be vaild
		GSE_LOG("after bma2xx_init_client databuf = 0x%x\n",databuf[0]);
	sensor_suspend = 0;

	atomic_set(&obj->suspend, 0);    
}
/*----------------------------------------------------------------------------*/
#endif /*USE_EARLY_SUSPEND*/
/*----------------------------------------------------------------------------
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int gsensor_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
----------------------------------------------------------------------------*/
static int gsensor_enable(struct bma2xx_i2c_data *obj)
{
    int err = 0;

	if(sensor_power)
	{
		GSE_LOG("Gsensor device have enabled!\n");
	}
	else
	{
		if (atomic_read(&obj_i2c_data->suspend) == 0)
		{
			err = BMA2XX_SetPowerMode(obj->client, true);
			GSE_LOG("Gsensor not in suspend BMA2XX_SetPowerMode!\n");
		}
		else
		{
			GSE_LOG("Gsensor in suspend and can not be enable \n");
		}
        
        if(err != BMA2XX_SUCCESS)
	    {
		    GSE_ERR("gsensor_enable fail!\n");
		    return -1;
	    }
        queue_delayed_work(obj->accel_workqueue, &obj->accel_work, 0);
	}
    
    GSE_LOG("gsensor_enable OK!\n");
	return 0;
}

static int gsensor_disable(struct bma2xx_i2c_data *obj)
{
    int err = 0;

	if(!sensor_power)
	{
		GSE_LOG("Gsensor device have disabled!\n");
	}
	else
	{
		if (atomic_read(&obj_i2c_data->suspend) == 0)
		{
		    cancel_delayed_work_sync(&obj->accel_work);
			err = BMA2XX_SetPowerMode(obj->client, false);
			GSE_LOG("Gsensor not in suspend BMA2XX_SetPowerMode!\n");
		}
		else
		{
			GSE_LOG("Gsensor in suspend and can not enable or disable!\n");
		}
        
        if(err != BMA2XX_SUCCESS)
	    {
		    GSE_ERR("gsensor_enable fail!\n");
		    return -1;
	    }
	}
    
    GSE_LOG("gsensor_disable OK!\n");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int gsensor_set_delay(struct bma2xx_i2c_data *obj, u64 ms)
{
    int err = 0;
    int value;
	int sample_delay;

    value = (int)ms;

	if(value <= 5)
	{
		sample_delay = BMA2XX_BW_200HZ;
	}
	else if(value <= 10)
	{
		sample_delay = BMA2XX_BW_100HZ;
	}
	else
	{
		sample_delay = BMA2XX_BW_100HZ;
	}

	mutex_lock(&gsensor_scp_en_mutex);
	err = BMA2XX_SetBWRate(obj->client, sample_delay);
	mutex_unlock(&gsensor_scp_en_mutex);
	if(err != BMA2XX_SUCCESS ) //0x2C->BW=100Hz
	{
		GSE_ERR("Set delay parameter error!\n");
        return -1;
	}

	if(value >= 50)
	{
		atomic_set(&obj->filter, 0);
	}
	else
	{	
	#if defined(CONFIG_BMA2XX_LOWPASS)
		priv->fir.num = 0;
		priv->fir.idx = 0;
		priv->fir.sum[BMA2XX_AXIS_X] = 0;
		priv->fir.sum[BMA2XX_AXIS_Y] = 0;
		priv->fir.sum[BMA2XX_AXIS_Z] = 0;
		atomic_set(&priv->filter, 1);
	#endif
	}
    GSE_LOG("gsensor_set_delay (delay:%d)\n",value);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int gsensor_get_data(int* x ,int* y,int* z)
{
	char buff[BMA2XX_BUFSIZE];
    int ret = 0;
    mutex_lock(&gsensor_scp_en_mutex);
	ret = BMA2XX_ReadSensorData(obj_i2c_data->client, buff, BMA2XX_BUFSIZE);
	mutex_unlock(&gsensor_scp_en_mutex);
	sscanf(buff, "%x %x %x", x, y, z);				

	return ret;
}
/*----------------------------------------------------------------------------*/
static void bma2xx_accel_work(struct work_struct *work)
{
    struct bma2xx_i2c_data *obj = container_of((struct delayed_work *)work,  struct bma2xx_i2c_data, accel_work);
    int ret = 0;
    int x = 0, y = 0, z = 0;

    ret = gsensor_get_data(&x, &y, &z);
    if (ret < 0)
		goto acc_loop;
    input_report_abs(obj->idev, ABS_X, x);
	input_report_abs(obj->idev, ABS_Y, y);
	input_report_abs(obj->idev, ABS_Z, z);
    input_sync(obj->idev);
acc_loop:
    
    queue_delayed_work(obj->accel_workqueue, &obj->accel_work, obj->hw->interval);
}

/*----------------------------------------------------------------------------*/
struct acc_hw *get_accel_dts_func(struct device_node *node, struct acc_hw *hw)
{
	int ret;
	u32 direction[] = {0};
	u32 firlen[] = {0};
	u32 delay[] = {0};
    u32 min_delay[] = {0};
	//struct device_node *node = NULL;

	GSE_LOG("Device Tree get accel info!\n");

	//node = of_find_compatible_node(NULL, NULL, name);
	if (node) {
		ret = of_property_read_u32_array(node , "direction", direction, ARRAY_SIZE(direction));
		if (ret == 0)
			hw->direction = direction[0];

		ret = of_property_read_u32_array(node , "firlen", firlen, ARRAY_SIZE(firlen));
		if (ret == 0)
			hw->firlen	=	firlen[0];

		ret = of_property_read_u32_array(node , "delay", delay, ARRAY_SIZE(delay));
		if (ret == 0)
        {
            hw->delay = delay[0];
            hw->interval = msecs_to_jiffies(delay[0]);
        }
        
        ret = of_property_read_u32_array(node , "min_delay", delay, ARRAY_SIZE(delay));
		if (ret == 0)
            hw->min_delay = min_delay[0];
	} else {
		GSE_ERR("Device Tree: can not find accel node!. Go to use old cust info\n");
		return NULL;
	}

	return hw;
}
/*----------------------------------------------------------------------------*/
static int bma2xx_accel_setup_input_device(struct bma2xx_i2c_data *obj)
{
    struct input_dev *dev;
    int err = 0;

    dev = input_allocate_device();
    if (NULL == dev)
        return -ENOMEM;

    __set_bit(EV_ABS, dev->evbit);
    input_set_abs_params(dev, ABS_X, ACC_VALUE_MIN, ACC_VALUE_MAX, 0, 0);
    input_set_abs_params(dev, ABS_Y, ACC_VALUE_MIN, ACC_VALUE_MAX, 0, 0);
    input_set_abs_params(dev, ABS_Z, ACC_VALUE_MIN, ACC_VALUE_MAX, 0, 0);

    input_set_drvdata(dev, obj);

    dev->name = BMA2XX_ACCEL_INPUT_NAME;
    dev->id.bustype = BUS_I2C;
    dev->dev.parent = &obj->client->dev;

    err = input_register_device(dev);
    if (err < 0) {
        input_free_device(dev);
        return err;
    }
    obj->idev = dev;

    return 0;

}
/*----------------------------------------------------------------------------*/

static int bma2xx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//struct i2c_client *new_client;
	struct bma2xx_i2c_data *obj;
    struct device_node *np = client->dev.of_node;

	int err = 0;
	int retry = 0;
	GSE_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct bma2xx_i2c_data));
    
    if (np)
    {
        hw = get_accel_dts_func(np, hw);
    
        if (!hw)
            GSE_ERR("get dts info fail\n");
    
	    obj->hw = hw;
    } else {
        goto exit_kfree;
    }
    err = acc_get_convert(obj->hw->direction, &obj->cvt);
    if(err) {
        GSE_ERR("invalid direction: %d\n", obj->hw->direction);
        goto exit_kfree;
    }

	obj_i2c_data = obj;
	obj->client = client;
/*
#ifdef FPGA_EARLY_PORTING
    obj->client->timing = 100; 
#else
    obj->client->timing = 400;
#endif
*/
	//new_client = obj->client;
	i2c_set_clientdata(client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	
#ifdef CONFIG_BMA2XX_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}
	
	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}
	
#endif

	bma2xx_i2c_client = client;	

	for(retry = 0; retry < 3; retry++){
		if(0 != (err = bma2xx_init_client(client, 1)))
		{
			GSE_ERR("bma2xx_device init cilent fail time: %d\n", retry);
			continue;
		}
        break;
	}
	if(err != 0)
		goto exit_init_failed;

	if(0 != (err = misc_register(&bma2xx_device)))
	{
		GSE_ERR("bma2xx_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if(0 != (err = sysfs_create_group(&bma2xx_device.this_device->kobj, &acc_attribute_group)))
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

    err = bma2xx_accel_setup_input_device(obj);
    if (err)
        goto exit_setup_input_failed;

    if(0 != (err = sysfs_create_group(&obj->idev->dev.kobj, &bma2xx_input_sys_group)))
    {
        GSE_ERR("create attribute err = %d\n", err);
        goto exit_create_input_attr_failed;
    }

    obj->accel_workqueue = create_freezable_workqueue("Bma2xx Accel Workqueue");
	INIT_DELAYED_WORK(&obj->accel_work, bma2xx_accel_work);

    
    //create_acc_calibrate_proc_file();
    //create_atcmd_test_sys();
	GSE_LOG("%s: OK\n", __func__);    
	return 0;
    
	exit_create_input_attr_failed:
        sysfs_remove_group(&obj->idev->dev.kobj, &bma2xx_input_sys_group);
	exit_setup_input_failed:
        sysfs_remove_group(&bma2xx_device.this_device->kobj, &acc_attribute_group);
    exit_create_attr_failed:
        misc_deregister(&bma2xx_device);
	exit_misc_device_register_failed:
	//i2c_detach_client(new_client);
	exit_init_failed:
	exit_kfree:
	kfree(obj);
	exit:
	GSE_ERR("%s: err = %d\n", __func__, err);
	return err;
}

/*----------------------------------------------------------------------------*/
static int bma2xx_i2c_remove(struct i2c_client *client)
{
	int err = 0;	
	
	sysfs_remove_group(&bma2xx_device.this_device->kobj, &acc_attribute_group);
	
	if(0 != (err = misc_deregister(&bma2xx_device)))
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}
    
	bma2xx_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init bma2xx_init(void)
{
	GSE_FUN();
	
	if(i2c_add_driver(&bma2xx_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}

	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit bma2xx_exit(void)
{
	GSE_FUN();   
    i2c_del_driver(&bma2xx_i2c_driver);
}
/*----------------------------------------------------------------------------*/
module_init(bma2xx_init);
module_exit(bma2xx_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMA2XX I2C driver");
MODULE_AUTHOR("Xiaoli.li@mediatek.com, ding.gaoshan@zte.com.cn");
