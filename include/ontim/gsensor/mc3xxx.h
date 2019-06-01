/*****************************************************************************
 *
 * Copyright (c) 2013 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("mCube Software")
 * have been modified by mCube Inc. All revisions are subject to any receiver's
 * applicable license agreements with mCube Inc.
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
 *
 *****************************************************************************/

#ifndef __MC3XXX_H__
#define __MC3XXX_H__
		  
#include <linux/ioctl.h>	  
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>

#define MC3XXX_ACC_I2C_ADDR     0x4C
#define MC3XXX_ACC_I2C_NAME     "mc3xxx"

typedef struct {
	unsigned short	x;		/**< X axis */
	unsigned short	y;		/**< Y axis */
	unsigned short	z;		/**< Z axis */
} GSENSOR_VECTOR3D;

typedef struct{
	int x;
	int y;
	int z;
} SENSOR_DATA;


#define GSENSOR                                0x95
#define GSENSOR_IOCTL_INIT                     _IO(GSENSOR,  0x01)
#define GSENSOR_IOCTL_READ_CHIPINFO            _IOR(GSENSOR, 0x02, int)
#define GSENSOR_IOCTL_READ_SENSORDATA          _IOR(GSENSOR, 0x03, int)
#define GSENSOR_IOCTL_READ_OFFSET              _IOR(GSENSOR, 0x04, GSENSOR_VECTOR3D)
#define GSENSOR_IOCTL_READ_GAIN                _IOR(GSENSOR, 0x05, GSENSOR_VECTOR3D)
#define GSENSOR_IOCTL_READ_RAW_DATA            _IOR(GSENSOR, 0x06, int)
#define GSENSOR_IOCTL_GET_CALI                 _IOW(GSENSOR, 0x07, SENSOR_DATA)
#define GSENSOR_IOCTL_CLR_CALI                 _IO(GSENSOR, 0x08)
#define GSENSOR_MCUBE_IOCTL_READ_RBM_DATA      _IOR(GSENSOR, 0x09, SENSOR_DATA)
#define GSENSOR_MCUBE_IOCTL_SET_RBM_MODE       _IO(GSENSOR, 0x0a)
#define GSENSOR_MCUBE_IOCTL_CLEAR_RBM_MODE     _IO(GSENSOR, 0x0b)
#define GSENSOR_MCUBE_IOCTL_SET_CALI           _IOW(GSENSOR, 0x0c, SENSOR_DATA)
#define GSENSOR_MCUBE_IOCTL_REGISTER_MAP       _IO(GSENSOR, 0x0d)
#define GSENSOR_IOCTL_SET_CALI_MODE            _IOW(GSENSOR, 0x0e,int)
#define GSENSOR_MCUBE_IOCTL_READ_PRODUCT_ID    _IOR(GSENSOR, 0x11, int)
#define GSENSOR_MCUBE_IOCTL_READ_CHIP_ID       _IOR(GSENSOR, 0x12, int)
#define GSENSOR_MCUBE_IOCTL_READ_FILEPATH      _IOR(GSENSOR, 0x13, char[256])



// register address define
#define MC3XXX_XOUT_REG			0x00
#define MC3XXX_YOUT_REG			0x01
#define MC3XXX_ZOUT_REG			0x02
#define MC3XXX_TILT_REG			0x03
#define MC3XXX_OPSTAT_REG		0x04
#define MC3XXX_SC_REG			0x05
#define MC3XXX_INTEN_REG		0x06
#define MC3XXX_MODE_REG			0x07
#define MC3XXX_SAMPR_REG		0x08
#define MC3XXX_TAPEN_REG		0x09
#define MC3XXX_TAPP_REG			0x0A
#define MC3XXX_DROP_REG			0x0B
#define MC3XXX_SHDB_REG			0x0C
#define MC3XXX_XOUT_EX_L_REG	0x0D
#define MC3XXX_XOUT_EX_H_REG	0x0E
#define MC3XXX_YOUT_EX_L_REG	0x0F
#define MC3XXX_YOUT_EX_H_REG	0x10
#define MC3XXX_ZOUT_EX_L_REG	0x11
#define MC3XXX_ZOUT_EX_H_REG	0x12

#define MC3XXX_CHIPID_REG		0x18

#define MC3XXX_OUTCFG_REG		0x20
#define MC3XXX_XOFFL_REG		0x21
#define MC3XXX_XOFFH_REG		0x22
#define MC3XXX_YOFFL_REG		0x23
#define MC3XXX_YOFFH_REG		0x24
#define MC3XXX_ZOFFL_REG		0x25
#define MC3XXX_ZOFFH_REG		0x26
#define MC3XXX_XGAIN_REG		0x27
#define MC3XXX_YGAIN_REG		0x28
#define MC3XXX_ZGAIN_REG		0x29

#define MC3XXX_SHAKE_TH_REG		0x2B
#define MC3XXX_UD_Z_TH_REG		0x2C
#define MC3XXX_UD_X_TH_REG		0x2D
#define MC3XXX_RL_Z_TH_REG		0x2E
#define MC3XXX_RL_Y_TH_REG		0x2F
#define MC3XXX_FB_Z_TH_REG		0x30
#define MC3XXX_DROP_TH_REG		0x31
#define MC3XXX_TAP_TH_REG		0x32

#define MC3XXX_PCODE_REG		0x3B

// mode
#define MC3XXX_MODE_STANDBY		    0
#define MC3XXX_MODE_WAKE			1

// range
#define MC3XXX_RANGE_2G			    0
#define MC3XXX_RANGE_4G			    1
#define MC3XXX_RANGE_8G_10BIT		2
#define MC3XXX_RANGE_8G_14BIT		3

// bandwidth
#define MC3XXX_BW_512HZ			    0
#define MC3XXX_BW_256HZ			    1
#define MC3XXX_BW_128HZ			    2
#define MC3XXX_BW_64HZ				3
#define MC3XXX_BW_32HZ				4
#define MC3XXX_BW_16HZ				5
#define MC3XXX_BW_8HZ				6

// initial value
#define MC3XXX_RANGE_SET			MC3XXX_RANGE_8G_14BIT  /* +/-8g, 14bit */
#define MC3XXX_BW_SET				MC3XXX_BW_128HZ /* 128HZ  */
#define MC3XXX_MAX_DELAY			200
#define ABSMIN_8G					(-8 * 1024)
#define ABSMAX_8G					(8 * 1024)
#define ABSMIN_1_5G					(-128)
#define ABSMAX_1_5G				    (128)

// 1g constant value
#define GRAVITY_1G_VALUE			1024

// product code
#define MC3XXX_PCODE_3210		0x90
#define MC3XXX_PCODE_3230		0x19
#define MC3XXX_PCODE_3250		0x88
#define MC3XXX_PCODE_3410		0xA8
#define MC3XXX_PCODE_3410N		0xB8
#define MC3XXX_PCODE_3430		0x29
#define MC3XXX_PCODE_3430N		0x39
#define MC3XXX_PCODE_3510B		0x40
#define MC3XXX_PCODE_3530B		0x30
#define MC3XXX_PCODE_3510C		0x10
#define MC3XXX_PCODE_3530C		0x60

#define MC3XXX_IOC_MAGIC 				77 //0x1D
#define MC3XXX_ACC_IOCTL_SET_DELAY		_IOW(MC3XXX_IOC_MAGIC,0,int)
#define MC3XXX_ACC_IOCTL_GET_DELAY		_IOR(MC3XXX_IOC_MAGIC,1,int)
#define MC3XXX_ACC_IOCTL_SET_ENABLE		_IOW(MC3XXX_IOC_MAGIC,2,int)
#define MC3XXX_ACC_IOCTL_GET_ENABLE		_IOR(MC3XXX_IOC_MAGIC,3,int)
#define MC3XXX_ACC_IOCTL_CALIBRATION	_IOW(MC3XXX_IOC_MAGIC,4,int)


// macro define
#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)
#define actual_delay(d)     (jiffies_to_msecs(delay_to_jiffies(d)))

// data declare
enum mc3xxx_orientation
{
    MC3XXX_TOP_LEFT_DOWN = 0,
    MC3XXX_TOP_RIGHT_DOWN,
    MC3XXX_TOP_RIGHT_UP,
    MC3XXX_TOP_LEFT_UP,
    MC3XXX_BOTTOM_LEFT_DOWN,
    MC3XXX_BOTTOM_RIGHT_DOWN,
    MC3XXX_BOTTOM_RIGHT_UP,
    MC3XXX_BOTTOM_LEFT_UP
};

enum mc3xxx_axis
{
    MC3XXX_AXIS_X = 0,
    MC3XXX_AXIS_Y,
    MC3XXX_AXIS_Z,
    MC3XXX_AXIS_NUM
};

struct mc3xxxacc
{
    signed short x;
    signed short y;
    signed short z;
};

struct mc3xxx_hwmsen_convert
{
    signed char sign[3];
    unsigned char map[3];
};

struct mc3xxx_data
{
    atomic_t enable;                /* attribute value */
    atomic_t suspend;                /* attribute value */
    atomic_t delay;                 /* attribute value */
    atomic_t position;              /* attribute value */
    atomic_t threshold;             /* attribute value */
    struct mc3xxxacc value;
    struct mutex value_mutex;
    struct mutex enable_mutex;
    struct i2c_client *client;
    struct input_dev *input;
    //struct delayed_work work;
    struct workqueue_struct *mc3xxx_wq;
    struct work_struct work;
    struct hrtimer mc3xxx_hrtimer;	
    unsigned char product_code;
    unsigned char resolution;
    struct mc3xxx_platform_data *pdata ;

#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
};

/******************************
 *** CONFIGURATIONS
 ******************************/
//#define MCUBE_DOT_CALIBRATION
//#define MCUBE_FUNC_DEBUG
//#define I2C_BUS_NUM_STATIC_ALLOC
#define MCUBE_I2C_BURST_LIMIT_2_BYTES
//#define SUPPORT_VPROXIMITY_SENSOR

#define MC3XXX_DEV_NAME       MC3XXX_ACC_I2C_NAME
#define MC3XXX_DEV_VERSION    "2.0.3"
#define MC3XXX_INPUT_NAME     "accelerometer"
#define MC3XXX_I2C_ADDR       MC3XXX_ACC_I2C_ADDR


#ifdef MCUBE_FUNC_DEBUG
#define MC_PRINTK(x...)        printk(x)
#define MC_ERR_PRINT(x...)    printk(x)
#else
#define MC_PRINTK(x...)
#define MC_ERR_PRINT(x...)
#endif

struct mc3xxx_platform_data{
    int (*power_on)(int on);
    unsigned char current_placement ;
};


#endif

