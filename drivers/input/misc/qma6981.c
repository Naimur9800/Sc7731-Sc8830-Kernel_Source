/*****************************************************************************
 *
 * Copyright (c) 2013 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the QST Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of QST Inc.
 *
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("QST Software")
 * have been modified by QST Inc. All revisions are subject to any receiver's
 * applicable license agreements with QST Inc.
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
#include	<linux/module.h>
#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>

#include	<linux/input.h>
#include	<linux/input-polldev.h>
#include	<linux/miscdevice.h>
#include	<linux/uaccess.h>
#include        <linux/slab.h>

#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

#include <linux/unistd.h>
#include <linux/string.h>
#include "qma6981.h"

/*range*/
#define QMA6981_RANGE_2G   (1<<0)
#define QMA6981_RANGE_4G   (1<<1)
#define QMA6981_RANGE_8G   (1<<2)
#define QMA6981_RANGE_16G  (1<<3)


#define QMA6981_XOUTL			0x01	// 4-bit output value X
#define QMA6981_XOUTH			0x02	// 6-bit output value X
#define QMA6981_YOUTL			0x03
#define QMA6981_YOUTH			0x04
#define QMA6981_ZOUTL			0x05
#define QMA6981_ZOUTH			0x06
#define QMA6981_STEPCOUNT			0x07	// stepcount
#define QMA6981_SRST			0x04	// Sampling Rate Status
#define QMA6981_SPCNT			0x05	// Sleep Count
#define QMA6981_INTSU			0x17	// Interrupt Setup
#define QMA6981_MODE			0x11	// Mode
#define QMA6981_SR			0x10	// Auto-Wake/Sleep and Debounce Filter
#define QMA6981_PDET			0x09	// Tap Detection
#define QMA6981_PD			0x0a	// Tap Debounce Count
#define QMA6981_INT_MAP0		0x19	// INT MAP
#define QMA6981_RANGE                   0x0f    //range set register
#define QMA6981_INT_STAT		0x09    //interrupt statues

#define QMA6981_FIFO_WTMK		0x31	// FIFO water mark level
#define QMA6981_FIFO_CONFIG		0x3e	// fifo configure
#define QMA6981_FIFO_DATA		0x3f	//fifo data out

#define QMA6981_CHIP_ID		0x00
#define QMA6981_DIE_ID			0x07
#define QMA6981_DIE_ID_V2		0x47




// mode
#define QMAX981_MODE_AUTO			0
#define QMAX981_MODE_WAKE			1
#define QMAX981_MODE_SNIFF			2
#define QMAX981_MODE_STANDBY		    3



// initial value
#define QMAX981_BW_SET				QMAX981_BW_128HZ /* 128HZ  */
#define QMAX981_MAX_DELAY			200  /* ms ,5Hz*/
#define ABSMIN_8G					(-8 * 1024)
#define ABSMAX_8G					(8 * 1024)
#define ABSMIN_1_5G					(-128)
#define ABSMAX_1_5G				    (128)

//#define QMAX981_ACC_STEP_COUNT
#define QMAX981_ACC_IOCTL_BASE				77 //0x1D
#define QMAX981_ACC_IOCTL_SET_DELAY		_IOW(QMAX981_ACC_IOCTL_BASE,0,int)
#define QMAX981_ACC_IOCTL_GET_DELAY		_IOR(QMAX981_ACC_IOCTL_BASE,1,int)
#define QMAX981_ACC_IOCTL_SET_ENABLE		_IOW(QMAX981_ACC_IOCTL_BASE,2,int)
#define QMAX981_ACC_IOCTL_GET_ENABLE		_IOR(QMAX981_ACC_IOCTL_BASE,3,int)
#define QMAX981_ACC_IOCTL_CALIBRATION	_IOW(QMAX981_ACC_IOCTL_BASE,4,int)
#ifdef QMAX981_ACC_STEP_COUNT
#define QMAX981_ACC_IOCTL_SET_STEPCOUNT_ENABLE		_IOW(QMAX981_ACC_IOCTL_BASE,5,int)
#define QMAX981_ACC_IOCTL_GET_STEPCOUNT_ENABLE		_IOR(QMAX981_ACC_IOCTL_BASE,6,int)
unsigned char step_count_delay = 0;
#endif

// macro define
#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)
#define actual_delay(d)     (jiffies_to_msecs(delay_to_jiffies(d)))

// data declare
enum qmax981_orientation {
	QMAX981_TOP_LEFT_DOWN = 0,
	QMAX981_TOP_RIGHT_DOWN,
	QMAX981_TOP_RIGHT_UP,
	QMAX981_TOP_LEFT_UP,
	QMAX981_BOTTOM_LEFT_DOWN,
	QMAX981_BOTTOM_RIGHT_DOWN,
	QMAX981_BOTTOM_RIGHT_UP,
	QMAX981_BOTTOM_LEFT_UP
};


enum qmax981_axis {
	QMAX981_AXIS_X = 0,
	QMAX981_AXIS_Y,
	QMAX981_AXIS_Z,
	QMAX981_AXIS_NUM
};

struct qmax981acc {
	signed short x;
	signed short y;
	signed short z;
#ifdef QMAX981_ACC_STEP_COUNT
	unsigned int	 stepcount;
#endif
};

struct qmax981_hwmsen_convert {
	signed char sign[3];
	unsigned char map[3];
};

struct qmax981_data {
	atomic_t enable;                /* attribute value */
	atomic_t step_count_enable;                /* attribute value */
	atomic_t delay;                 /* attribute value */
	atomic_t position;              /* attribute value */
	atomic_t threshold;             /* attribute value */
	struct qmax981acc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	unsigned char product_code;
	unsigned char resolution;
};

/******************************
 *** CONFIGURATIONS
 ******************************/
#define QST_FUNC_DEBUG
#define MCUBE_I2C_BURST_LIMIT_2_BYTES

#define QMAX981_DEV_NAME       QMAX981_ACC_I2C_NAME
#define QMAX981_DEV_VERSION    "1.0.1"
#define QMAX981_INPUT_NAME     "accelerometer"
#define QMAX981_I2C_ADDR       QMAX981_ACC_I2C_ADDR

static unsigned char qmax981_current_placement = QMAX981_BOTTOM_RIGHT_UP; // current soldered placement

#define MODNAME                         "[QMA6981]"
#undef pr_fmt
#define pr_fmt(fmt)  MODNAME"%s %d:" fmt, __func__, __LINE__
#define LOG_FUN()        pr_info("\n")
#define LOG_INFO(fmt, ...)  pr_info(fmt, ##__VA_ARGS__)
#define LOG_DBG(fmt, ...)  pr_debug(fmt, ##__VA_ARGS__)
#define LOG_ERR(fmt, ...)  pr_err(fmt, ##__VA_ARGS__)

// Transformation matrix for chip mounting position
static const struct qmax981_hwmsen_convert qmax981_cvt[] = {
	{{  1,   1,   1}, { QMAX981_AXIS_X,  QMAX981_AXIS_Y,  QMAX981_AXIS_Z}}, // 0: top   , left-down
	{{ -1,   1,   1}, { QMAX981_AXIS_Y,  QMAX981_AXIS_X,  QMAX981_AXIS_Z}}, // 1: top   , right-down
	{{ -1,  -1,   1}, { QMAX981_AXIS_X,  QMAX981_AXIS_Y,  QMAX981_AXIS_Z}}, // 2: top   , right-up
	{{  1,  -1,   1}, { QMAX981_AXIS_Y,  QMAX981_AXIS_X,  QMAX981_AXIS_Z}}, // 3: top   , left-up
	{{ -1,   1,  -1}, { QMAX981_AXIS_X,  QMAX981_AXIS_Y,  QMAX981_AXIS_Z}}, // 4: bottom, left-down
	{{  1,   1,  -1}, { QMAX981_AXIS_Y,  QMAX981_AXIS_X,  QMAX981_AXIS_Z}}, // 5: bottom, right-down
	{{  1,  -1,  -1}, { QMAX981_AXIS_X,  QMAX981_AXIS_Y,  QMAX981_AXIS_Z}}, // 6: bottom, right-up
	{{ -1,  -1,  -1}, { QMAX981_AXIS_Y,  QMAX981_AXIS_X,  QMAX981_AXIS_Z}}, // 7: bottom, left-up
};

static struct i2c_client *qmax981_client = NULL;

//static GSENSOR_VECTOR3D qmax981_gain = {0};
static unsigned char ChipID = 0;

static struct qmax981_data *g_qmax981_data;

#ifdef I2C_BUS_NUM_STATIC_ALLOC
#define I2C_STATIC_BUS_NUM       0  //// SPRD should use 2 !

static struct i2c_board_info  qmax981_i2c_boardinfo = {
	I2C_BOARD_INFO(QMAX981_DEV_NAME, QMAX981_I2C_ADDR)
};
#endif

static inline int qmax981_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	signed int dummy = 0;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return dummy;
	*data = dummy & 0x000000ff;

	return 0;
}

static inline int qmax981_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	signed int dummy = 0;

	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return dummy;
	return 0;
}

static inline int qmax981_smbus_read_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	signed int dummy = 0;

	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return dummy;

	return 0;
}

static inline int qmax981_smbus_write_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	signed int dummy = 0;


	dummy = i2c_smbus_write_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return dummy;
	return 0;
}

static int qmax981_read_chip_id(struct i2c_client *client, char *buf)
{
	unsigned char chipiddata = 0;
	unsigned char outputdata = 0;
	unsigned char tempdata = 0;

	int res = 0;
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif
	res = qmax981_smbus_read_byte(qmax981->client, QMA6981_CHIP_ID, &chipiddata);
	if (res) {
		LOG_ERR("%s: i2c error!\n", __func__);
		return -1;
	}

	ChipID = chipiddata;

	if (ChipID == 0xa9)
		outputdata = QMA6981_DIE_ID;
	else
		outputdata = QMA6981_DIE_ID_V2;

	res = qmax981_smbus_read_byte(qmax981->client, outputdata, &tempdata);
	if (res) {
		LOG_ERR("%s: i2c error!\n", __func__);
		return -1;
	}

	return sprintf(buf, "%02X-%02X\n", chipiddata, tempdata);
}

static int qmax981_set_mode(struct i2c_client *client, unsigned char mode)
{
	int rc = 0;
	unsigned char data = 0;

	if (mode == QMAX981_MODE_STANDBY) {
		data = 0x00;
		rc = qmax981_smbus_write_byte(client,0x11,&data);
	} else if (mode == QMAX981_MODE_WAKE) {
		data = 0x80;
		rc = qmax981_smbus_write_byte(client,0x11,&data);
		msleep(1);
	}

	return 0;
}

#ifdef QMAX981_ACC_STEP_COUNT
static int qmax981_set_stepcount_mode(struct i2c_client *client, unsigned char mode)
{
	int rc = 0;
	unsigned char data = 0;

	if (mode == QMAX981_MODE_STANDBY) {
		data = 0x00;
		rc = qmax981_smbus_write_byte(client, 0x16, &data);

	} else if (mode == QMAX981_MODE_WAKE) {
		data = 0x08;
		rc = qmax981_smbus_write_byte(client, 0x16, &data);
		msleep(1);
	}
	return 0;
}
#endif

static int qmax981_get_mode(struct i2c_client *client, unsigned char *mode)
{

	return 0;
}

#ifdef QMAX981_ACC_STEP_COUNT
static int qmax981_get_stepcount_mode(struct i2c_client *client, unsigned char *mode)
{

	return 0;
}
#endif

static int qmax981_set_range(struct i2c_client *client, unsigned char range)
{

	return 0;
}

static int qmax981_get_range(struct i2c_client *client, unsigned char *range)
{
	return 0;
}

static int qmax981_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
	return 0;
}

static int qmax981_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{

	return 0;
}
#ifdef QMAX981_ACC_STEP_COUNT
static int qmax981_init(struct qmax981_data *qmax981)
{

	int ret = 0;
	unsigned char data = 0;

	//range  2g , 3.9mg/LSB
	data = 0x01;
	ret = qmax981_smbus_write_byte(qmax981->client,0x0f,&data);
	if(ret < 0)
		goto exit_i2c_err;

	//lower output data rate ,Bandwidth = 62.5Hz ODR = 2*BW = 125Hz
	data= 0x0c;
	ret = qmax981_smbus_write_byte(qmax981->client,0x10,&data);
	if(ret < 0)
	  goto exit_i2c_err;

  //active mode, sleep time 10ms

	data= 0x8a;
	ret = qmax981_smbus_write_byte(qmax981->client,0x11,&data);
	if(ret < 0)
		goto exit_i2c_err;

	//0x12,start step_count, sample counts 48
	data= 0x8c;
	ret = qmax981_smbus_write_byte(qmax981->client,0x12,&data);
	if(ret < 0)
		goto exit_i2c_err;

	//0x13, dynamic precision 150mg
	data = 0x26;
	ret = qmax981_smbus_write_byte(qmax981->client,0x13,&data);
	if(ret < 0)
		goto exit_i2c_err;

	//0x14,time window low 0.324s

	data= 0x12;
	ret = qmax981_smbus_write_byte(qmax981->client,0x14,&data);
	if(ret < 0)
		goto exit_i2c_err;

	//0x15,time window up 2.16s
	data= 0x0f;
	ret = qmax981_smbus_write_byte(qmax981->client,0x15,&data);
	if(ret < 0)
		goto exit_i2c_err;

	//0x32,0x00 xy,
	if(qmax981_current_placement%2) ://0 android we selcet static android Y and Z axis
	data= 0x02;
	else
	data= 0x01;
	ret = qmax981_smbus_write_byte(qmax981->client,0x32,&data);
	if(ret < 0)
		goto exit_i2c_err;

	data= 0x64;//0x32;
	ret = qmax981_smbus_write_byte(qmax981->client,0x27,&data);
	if(ret < 0)
		goto exit_i2c_err;


	data= 0x64;//0x32;
	ret = qmax981_smbus_write_byte(qmax981->client,0x28,&data);
	if(ret < 0)
		goto exit_i2c_err;


	data= 0x64;//0x32;
	ret = qmax981_smbus_write_byte(qmax981->client,0x29,&data);
	if(ret < 0)
		goto exit_i2c_err;


	data= 0x08;
	ret = qmax981_smbus_write_byte(qmax981->client,0x16,&data);
	if(ret < 0)
		goto exit_i2c_err;

	return 0;
exit_i2c_err:
	LOG_ERR("qma6981_initialize fail: %d\n",ret);
	return ret;
}

#else
static int qmax981_init(struct qmax981_data *qmax981)
{
	int rc = 0;
	unsigned char data = 0;

	data = 0x01;
	rc = qmax981_smbus_write_byte(qmax981->client,0x0f,&data);

	data = 0x2c;
	rc = qmax981_smbus_write_byte(qmax981->client,0x10,&data);

	data = 0x80;
	rc = qmax981_smbus_write_byte(qmax981->client,0x11,&data);

	data = 0x08;
	rc = qmax981_smbus_write_byte(qmax981->client,0x4a,&data);

	data = 0x70;
	rc = qmax981_smbus_write_byte(qmax981->client,0x5f,&data);

	return rc;
}
#endif

static int qmax981_read_accel_xyz(struct qmax981_data *qmax981, struct qmax981acc *acc, int orient)
{
	int comres = -1;
	unsigned char data[6] = { 0 };
	s16 raw[3] = { 0 };
	int i;

	comres = qmax981_smbus_read_block(qmax981->client, QMA6981_XOUTL, data, 6);
	raw[0] = (s16)((data[1]<<2) | (data[0]>>6));
	raw[1] = (s16)((data[3]<<2) | (data[2]>>6));
	raw[2] = (s16)((data[5]<<2) | (data[4]>>6));
#ifdef QST_FUNC_DEBUG
	/* LOG_INFO("%d, %d, %d\n", raw[0], raw[1], raw[2]); */
#endif
	for (i=0; i<3; i++) {
		if (raw[i] == 0x0200)
			raw[i]= -512;
		else if (raw[i] & 0x0200) {
			raw[i] -= 0x1;
			raw[i] = ~raw[i];
			raw[i] &= 0x01ff;
			raw[i] = -raw[i];
		}
	}

#ifdef QMAX981_ACC_STEP_COUNT
	raw[0] -= 100;
	raw[1] -= 100;
	raw[2] -= 100;
#endif
	/* LOG_INFO("RAW_DATA_AFTER %d,%d,%d\n", data[0], data[1], data[2]); */
	if (ChipID == 0xa9)
		raw[1] = raw[1] + (raw[2]*8)/100;
	if (ChipID == 0xb0)
		raw[1] = raw[1] + raw[0]/10;
	if (comres) {
		LOG_ERR("%s: i2c error!\n", __func__);
		return comres;
	}

#ifdef QST_FUNC_DEBUG
	/* LOG_INFO("fan2: %d, %d, %d,  %x\n", raw[0], raw[1], raw[2], ChipID); */
#endif
	acc->x =  raw[0];
	 acc->y = raw[1];
	acc->z = raw[2];

	return comres;
}

#ifdef QMAX981_ACC_STEP_COUNT
static int qmax981_read_accel_stepcount(struct qmax981_data *qmax981, struct qmax981acc *acc)
{

	int comres = -1;
	unsigned char data[6] = { 0 };
	s16 raw = { 0 };

	comres = qmax981_smbus_read_block(qmax981->client, QMA6981_STEPCOUNT, data, 2);
	raw = (s16)((data[1]<<8) |( data[0]));

	acc->stepcount = raw;

	return comres;
}
#endif

static void qmax981_work_func(struct work_struct *work)
{
	static struct qmax981acc acc = { 0 };
	struct qmax981_data *qmax981 = container_of((struct delayed_work *)work, struct qmax981_data, work);
	unsigned long delay = msecs_to_jiffies(atomic_read(&qmax981->delay));

	int pre_enable_acc = atomic_read(&qmax981->enable);

	if(pre_enable_acc!=0)
		qmax981_read_accel_xyz(qmax981, &acc, qmax981_current_placement);
	/*loopur modilfy start*/
#ifdef QMAX981_ACC_STEP_COUNT
	if (step_count_delay > 5) {
		step_count_delay = 0;
		qmax981_read_accel_stepcount(qmax981, &acc);
		if(pre_enable_sc!=0)
			input_report_abs(qmax981->input, ABS_RX, acc.stepcount);
			/*loopur modilfy*/
	}
	else
		step_count_delay++;
	/*loopur modilfy end*/
#endif

#ifdef QST_FUNC_DEBUG
	//LOG_INFO("%s: %d, %d, %d\n",__func__, acc.x, acc.y, acc.z);
#endif

	input_report_abs(qmax981->input, ABS_X, acc.x);
	input_report_abs(qmax981->input, ABS_Y, acc.y);
	input_report_abs(qmax981->input, ABS_Z, acc.z);
	input_sync(qmax981->input);

	mutex_lock(&qmax981->value_mutex);
	qmax981->value = acc;
	mutex_unlock(&qmax981->value_mutex);
#ifdef QMAX981_ACC_STEP_COUNT
	if (pre_enable_acc == 0)
		delay = msecs_to_jiffies(atomic_read(&qmax981->delay)/10);  // step counter use slow mode
#endif
	schedule_delayed_work(&qmax981->work, delay);
}

static int qmax981_input_init(struct qmax981_data *qmax981)
{
	struct input_dev *dev = NULL;
	int err = 0;


#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	dev = input_allocate_device();
	if (!dev) {
		pr_err("%s: can't allocate device!\n", __func__);
		return -ENOMEM;
	}

	dev->name = QMAX981_INPUT_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);

	{
		input_set_abs_params(dev, ABS_X, ABSMIN_8G, ABSMAX_8G, 0, 0);
		input_set_abs_params(dev, ABS_Y, ABSMIN_8G, ABSMAX_8G, 0, 0);
		input_set_abs_params(dev, ABS_Z, ABSMIN_8G, ABSMAX_8G, 0, 0);
		input_set_abs_params(dev, ABS_RX, 0, 32767, 0, 0);   /*loopur modilfy*/
	}


	input_set_drvdata(dev, qmax981);

	err = input_register_device(dev);
	if (err < 0) {
		pr_err("%s: can't register device!\n", __func__);
		input_free_device(dev);
		return err;
	}

	qmax981->input = dev;

	return 0;
}

static void qmax981_input_deinit(struct qmax981_data *qmax981)
{
	struct input_dev *dev = qmax981->input;

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	input_unregister_device(dev);
	input_free_device(dev);
}

static ssize_t qmax981_register_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int address = 0, value = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	sscanf(buf, "%d %d", &address, &value);
	if (0 != qmax981_smbus_write_byte(qmax981->client, (unsigned char)address,
				(unsigned char *)&value))
		return -EINVAL;
	return count;
}

static ssize_t qmax981_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

	size_t count = 0;
	u8 reg[0x40] = { 0 };
	int i = 0;

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	for (i = 0; i < 0x40; i++) {
		qmax981_smbus_read_byte(qmax981->client, i, reg+i);
		count += sprintf(&buf[count], "0x%x: %x\n", i, reg[i]);
	}

	return count;
}

static ssize_t qmax981_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	if (qmax981_get_range(qmax981->client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t qmax981_range_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data = 0;
	int error = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (qmax981_set_range(qmax981->client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t qmax981_bandwidth_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	if (qmax981_get_bandwidth(qmax981->client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t qmax981_bandwidth_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data = 0;
	int error = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (qmax981_set_bandwidth(qmax981->client,
				(unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t qmax981_position_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	return sprintf(buf, "%d\n", qmax981_current_placement);
}

static ssize_t qmax981_position_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long position = 0;

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	position = simple_strtoul(buf, NULL, 10);
	if ((position >= 0) && (position <= 7))
		qmax981_current_placement = position;

	return count;
}

static ssize_t qmax981_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	if (qmax981_get_mode(qmax981->client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t qmax981_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data = 0;
	int error = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (qmax981_set_mode(qmax981->client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t qmax981_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct qmax981_data *qmax981 = input_get_drvdata(input);
	struct qmax981acc acc_value = { 0 };

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	mutex_lock(&qmax981->value_mutex);
	acc_value = qmax981->value;
	mutex_unlock(&qmax981->value_mutex);

	return sprintf(buf, "%d %d %d\n", acc_value.x, acc_value.y,
			acc_value.z);
}

static ssize_t qmax981_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmax981_data *qmax981 = i2c_get_clientdata(client);
	struct qmax981_data *qmax981 = i2c_get_clientdata(qmax981_client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	return sprintf(buf, "%d\n", atomic_read(&qmax981->delay));
}

static ssize_t qmax981_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

    return sprintf(buf, "%s\n", QMAX981_DEV_VERSION);
}

static ssize_t qmax981_chip_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

    return qmax981_read_chip_id(client, buf);
}

static ssize_t qmax981_product_code_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	return sprintf(buf, "0x%02X\n", qmax981->product_code);
}

static int qmax981_get_enable(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	return atomic_read(&qmax981->enable);
}

static void qmax981_set_enable(struct device *dev, int enable)
{
	struct qmax981_data *qmax981 = i2c_get_clientdata(qmax981_client);
	int pre_enable_acc = atomic_read(&qmax981->enable);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("enable=%d\n", enable);
#endif

	mutex_lock(&qmax981->enable_mutex);
	if (enable) {
		if (pre_enable_acc == 0) {
			qmax981_set_mode(qmax981->client,
					QMAX981_MODE_WAKE);
			schedule_delayed_work(&qmax981->work,
				msecs_to_jiffies(atomic_read(&qmax981->delay)));
			atomic_set(&qmax981->enable, 1);
		}

	} else {
		if (pre_enable_acc == 1) {
			qmax981_set_mode(qmax981->client,
					QMAX981_MODE_STANDBY);
			cancel_delayed_work_sync(&qmax981->work);
			atomic_set(&qmax981->enable, 0);
		}
	}
	mutex_unlock(&qmax981->enable_mutex);
}

#ifdef QMAX981_ACC_STEP_COUNT
static int qmax981_get_stepcount_enable(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	return atomic_read(&qmax981->step_count_enable);
}


static void qmax981_set_stepcount_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);
	int pre_enable_sc = atomic_read(&qmax981->step_count_enable);
	int pre_enable_acc = atomic_read(&qmax981->enable);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called %d\n", __func__, enable);
#endif

	mutex_lock(&qmax981->enable_mutex);
	if (enable) {
		if (pre_enable_sc == 0) {
			qmax981_set_stepcount_mode(qmax981->client,
					QMAX981_MODE_WAKE);
			if (pre_enable_acc == 0)
			schedule_delayed_work(&qmax981->work,msecs_to_jiffies(atomic_read(&qmax981->delay)));
			atomic_set(&qmax981->step_count_enable, 1);
		}

	} else {
		if (pre_enable == 1) {
			#if 0
			qmax981_set_mode(qmax981->client,
					QMAX981_MODE_STANDBY);
			#endif
			if(pre_enable_acc == 0)
			cancel_delayed_work_sync(&qmax981->work);
			atomic_set(&qmax981->step_count_enable, 0);
		}
	}
	mutex_unlock(&qmax981->enable_mutex);
}
#endif

static ssize_t qmax981_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct qmax981_data *qmax981 = i2c_get_clientdata(client);
	struct qmax981_data *qmax981 = i2c_get_clientdata(qmax981_client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n",__func__);
#endif

	return sprintf(buf, "%d\n", atomic_read(&qmax981->enable));
}

static ssize_t qmax981_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int enable = simple_strtoul(buf, NULL, 10);

	LOG_INFO("enable=%d\n", enable);
	if (enable)
		qmax981_set_enable(dev, 1);
	else
		qmax981_set_enable(dev, 0);

	return count;
}

static int qmax981_get_delay(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n",__func__);
#endif

	return atomic_read(&qmax981->delay);
}

static void qmax981_set_delay(struct device *dev, int delay)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

	LOG_INFO("delay=%d\n", delay);

	atomic_set(&qmax981->delay, delay);
	mutex_lock(&qmax981->enable_mutex);

	if (qmax981_get_enable(dev)) {
		cancel_delayed_work_sync(&qmax981->work);
		schedule_delayed_work(&qmax981->work, delay_to_jiffies(delay) + 1);
	}

	mutex_unlock(&qmax981->enable_mutex);
}

static ssize_t qmax981_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct qmax981_data *qmax981 = i2c_get_clientdata(qmax981_client);
	unsigned long delay = simple_strtoul(buf, NULL, 10);

	LOG_INFO("delay=%ld\n", delay);
	if (delay > QMAX981_MAX_DELAY)
		delay = QMAX981_MAX_DELAY;
	atomic_set(&qmax981->delay, delay);

	return count;
}

static DEVICE_ATTR(enable, 0644, qmax981_enable_show, qmax981_enable_store);
static DEVICE_ATTR(delay, 0644, qmax981_delay_show, qmax981_delay_store);
static DEVICE_ATTR(value, S_IRUGO, qmax981_value_show, NULL);
static DEVICE_ATTR(range, 0644, qmax981_range_show, qmax981_range_store);
static DEVICE_ATTR(bandwidth, 0644, qmax981_bandwidth_show, qmax981_bandwidth_store);
static DEVICE_ATTR(position, 0644, qmax981_position_show, qmax981_position_store);
static DEVICE_ATTR(mode, 0644, qmax981_mode_show, qmax981_mode_store);
static DEVICE_ATTR(reg, 0644, qmax981_register_show, qmax981_register_store);
static DEVICE_ATTR(product_code, S_IRUGO, qmax981_product_code_show, NULL);
static DEVICE_ATTR(chip_id, S_IRUGO, qmax981_chip_id_show, NULL);
static DEVICE_ATTR(version, S_IRUGO, qmax981_version_show, NULL);

static DEVICE_ATTR(gsensor, 0644, qmax981_enable_show, qmax981_enable_store);
static DEVICE_ATTR(delay_acc, 0644, qmax981_delay_show, qmax981_delay_store);

static struct attribute *qmax981_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_position.attr,
	&dev_attr_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_reg.attr,
	&dev_attr_product_code.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_version.attr,
	NULL
};

static struct attribute_group qmax981_attribute_group = {
	.attrs = qmax981_attributes
};


static long qmax981_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	void __user *argp = (void __user *)arg;
	struct qmax981_data *qmax981 = file->private_data;
	int temp = 0;

#ifdef MCUBE_DOT_CALIBRATION
	char strbuf[256] = { 0 };
	SENSOR_DATA sensor_data = { 0 };
#endif

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s: %x\n",__func__, cmd);
#endif

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));

	if (err) {
		LOG_ERR("%s: access isn't ok!\n", __func__);
		return -EFAULT;
	}

	if (NULL == qmax981_client) {
		LOG_ERR("%s: i2c client isn't exist!\n", __func__);
		return -EFAULT;
	}

	switch (cmd)
	{
	case QMAX981_ACC_IOCTL_SET_DELAY:
		if (copy_from_user(&temp, argp, sizeof(temp))) {
			LOG_ERR("%s: set delay copy error!\n", __func__);
			return -EFAULT;
		}
		if (temp < 0 || temp > 1000) {
			LOG_ERR("%s: set delay over limit!\n", __func__);
			return -EINVAL;
		}
		qmax981_set_delay(&(qmax981->client->dev), temp);
		break;

	case QMAX981_ACC_IOCTL_GET_DELAY:
		temp = qmax981_get_delay(&(qmax981->client->dev));
		if (copy_to_user(argp, &temp, sizeof(temp))) {
			LOG_ERR("%s: get delay copy error!\n", __func__);
			return -EFAULT;
		}
		break;

	case QMAX981_ACC_IOCTL_SET_ENABLE:
		if (copy_from_user(&temp, argp, sizeof(temp))) {
			LOG_ERR("%s: set enable copy error!\n", __func__);
			return -EFAULT;
		}

		if (1 == temp)
			qmax981_set_enable(&(qmax981->client->dev), 1);
		else if (0 == temp)
			qmax981_set_enable(&(qmax981->client->dev), 0);
		else {
			LOG_ERR("%s: set enable over limit!\n", __func__);
			return -EINVAL;
		}

		break;

	case QMAX981_ACC_IOCTL_GET_ENABLE:
		temp = qmax981_get_enable(&(qmax981->client->dev));
		if (copy_to_user(argp, &temp, sizeof(temp))) {
			LOG_ERR("%s: get enable copy error!\n", __func__);
			return -EINVAL;
		}
		break;

#ifdef QMAX981_ACC_STEP_COUNT
	case QMAX981_ACC_IOCTL_SET_STEPCOUNT_ENABLE:
		if (copy_from_user(&temp, argp, sizeof(temp))) {
			LOG_ERR("%s: set enable copy error!\n", __func__);
			return -EFAULT;
		}

		if (1 == temp)
			qmax981_set_stepcount_enable(&(qmax981->client->dev), 1);
		else if (0 == temp)
			qmax981_set_stepcount_enable(&(qmax981->client->dev), 0);
		else {
			LOG_ERR("%s: set enable over limit!\n", __func__);
			return -EINVAL;
		}

		break;

	case QMAX981_ACC_IOCTL_GET_STEPCOUNT_ENABLE:
		temp = qmax981_get_stepcount_enable(&(qmax981->client->dev));
		if (copy_to_user(argp, &temp, sizeof(temp))) {
			LOG_ERR("%s: get enable copy error!\n", __func__);
			return -EINVAL;
		}
		break;
#endif

	case QMAX981_ACC_IOCTL_CALIBRATION:
		LOG_ERR("%s: don't handle the command!\n", __func__);
		return -EINVAL;


	default:
		LOG_ERR("%s: can't recognize the cmd!\n", __func__);
		return 0;
	}

    return 0;
}

static ssize_t qmax981_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	int ret = 0;
	struct qmax981acc acc = { 0 };
	struct qmax981_data *qmax981 = NULL;

#ifdef QST_FUNC_DEBUG
	LOG_INFO("called\n");
#endif

	if (NULL == qmax981_client) {
		LOG_ERR("%s: I2C driver not install!", __func__);
		return -1;
	}

	qmax981 = i2c_get_clientdata(qmax981_client);

#ifdef MCUBE_DOT_CALIBRATION
	qmax981_read_cali_file(qmax981);
	qmax981_read_true_data(qmax981, &acc);
#else
	qmax981_read_accel_xyz(qmax981, &acc, qmax981_current_placement);
#endif

	mutex_lock(&qmax981->value_mutex);
	qmax981->value = acc;
	mutex_unlock(&qmax981->value_mutex);

	ret = copy_to_user(buf, &acc, sizeof(acc));
	if (ret) {
		LOG_ERR("%s: fail to copy_to_user: %d\n", __func__, ret);
		return 0;
	}

	return sizeof(acc);
}

static ssize_t qmax981_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n",__func__);
#endif

	return 0;
}

static int qmax981_open(struct inode *inode, struct file *file)
{
	int err = 0;

#ifdef QST_FUNC_DEBUG
	LOG_INFO("called\n");
#endif

	err = nonseekable_open(inode, file);
	if (err < 0) {
		LOG_ERR("%s: open fail!\n", __func__);
		return err;
	}

	file->private_data = i2c_get_clientdata(qmax981_client);

	return 0;
}

static int qmax981_close(struct inode *inode, struct file *file)
{
#ifdef QST_FUNC_DEBUG
	LOG_INFO("called\n");
#endif
	return 0;
}

static const struct file_operations qmax981_fops = {
	.owner = THIS_MODULE,
	.read = qmax981_read,
	.write = qmax981_write,
	.open = qmax981_open,
	.release = qmax981_close,
	.unlocked_ioctl = qmax981_ioctl,
};

static int qmax981_remove(struct i2c_client *client)
{
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif

	qmax981_set_enable(&client->dev, 0);

	sysfs_remove_group(&qmax981->input->dev.kobj, &qmax981_attribute_group);
	qmax981_input_deinit(qmax981);
	kfree(qmax981);

	return 0;
}

#ifdef CONFIG_OF
static struct QMAX981_acc_platform_data *QMAX981_acc_parse_dt(struct device *dev)
{
	struct QMAX981_acc_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct lis3dh_acc_platform_data");
		return NULL;
	}

/*	pdata->gpio_int1 = of_get_gpio(np, 0);
	LOG_INFO(" %s:get gpio_int1  %d\n",__func__,pdata->gpio_int1);
	if(pdata->gpio_int1 < 0){
		dev_err(dev, "fail to get irq_gpio_number\n");
		LOG_INFO(" get irq1_gpio_number fail\n");
		goto fail;
	}*/
	ret = of_property_read_u32(np, "layout", &pdata->layout);
	if (ret) {
		dev_err(dev, "fail to get g_range\n");
		goto fail;
	}
	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

static struct miscdevice qmax981_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = QMAX981_DEV_NAME,
	.fops = &qmax981_fops,
};

static int qmax981_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct qmax981_data *qmax981 = NULL;
	struct QMAX981_acc_platform_data *pdata = client->dev.platform_data;
	int err = 0;
	char buf = 0 ;
	struct class *gsensor_class;
	struct device *gsensor_cmd_dev;
	struct device_node *np;

	LOG_INFO("%s: probe start.\n", QMAX981_DEV_NAME);
#ifdef CONFIG_OF
		np = client->dev.of_node;
		if (np && !pdata) {
			pdata = QMAX981_acc_parse_dt(&client->dev);
			if (pdata)
				client->dev.platform_data = pdata;
			if (!pdata) {
				err = -ENOMEM;
				goto exit_alloc_platform_data_failed;
			}
		}
#endif

	/* setup private data */
	qmax981 = kzalloc(sizeof(struct qmax981_data), GFP_KERNEL);
	if (!qmax981) {
		LOG_ERR("can't allocate memory for qmax981_data!\n");
		err = -ENOMEM;
		return err;
	}

	qmax981->product_code = 0;

	mutex_init(&qmax981->enable_mutex);
	mutex_init(&qmax981->value_mutex);

	/* setup i2c client */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		LOG_ERR("%s: i2c function not support!\n", __func__);
		kfree(qmax981);
		err = -ENODEV;
		return err;
	}

	pdata = client->dev.platform_data;

	if (pdata) {
		/* Platform data is available. copy its value to local. */
		atomic_set(&qmax981->position, pdata->layout);
		qmax981_current_placement = pdata->layout;
		atomic_set(&qmax981->delay, QMAX981_MAX_DELAY);
	} else {
		/* Platform data is not available.
		Layout and information should be set by each application. */
		dev_dbg(&client->dev, "%s: No platform data.", __func__);
		atomic_set(&qmax981->position, qmax981_current_placement);
		atomic_set(&qmax981->delay, QMAX981_MAX_DELAY);
	}

	qmax981->client = client;
	qmax981_client = client;
	i2c_set_clientdata(client, qmax981);
	g_qmax981_data = qmax981;


	dev_info(&client->dev, "%s found\n", id->name);

	err = qmax981_read_chip_id(client, &buf);
	if (err < 0) {
		LOG_INFO("%s: qmax981 read id fail!\n", __func__);
		return err;
	}

	qmax981_init(qmax981);
#ifdef QMAX981_ACC_STEP_COUNT
	qmax981_stepcount_init(qmax981);
#endif
	/* setup driver interfaces */
	INIT_DELAYED_WORK(&qmax981->work, qmax981_work_func);
	err = qmax981_input_init(qmax981);
	if (err < 0) {
		LOG_ERR("%s: input init fail!\n", __func__);
		kfree(qmax981);
		return err;
	}

	err = sysfs_create_group(&qmax981->input->dev.kobj, &qmax981_attribute_group);
	if (err < 0) {
		LOG_ERR("%s: create group fail!\n", __func__);
		qmax981_input_deinit(qmax981);
		kfree(qmax981);
		return err;
	}

	err = misc_register(&qmax981_device);
	if (err) {
		LOG_ERR("%s: create register fail!\n", __func__);
	sysfs_remove_group(&qmax981->input->dev.kobj, &qmax981_attribute_group);
		qmax981_input_deinit(qmax981);
		kfree(qmax981);
		return err;
	}

	gsensor_class = class_create(THIS_MODULE, "xr-gsensor");//client->name
	if (IS_ERR(gsensor_class))
		LOG_INFO("Failed to create class(xr-gsensor)!\n");
	gsensor_cmd_dev = device_create(gsensor_class, NULL, 0, NULL, "device");
	if (IS_ERR(gsensor_cmd_dev))
		LOG_INFO("Failed to create device(gsensor_cmd_dev)!\n");
	 /* /sys/class/xr-gsensor/device/gsensor */
	if (device_create_file(gsensor_cmd_dev, &dev_attr_gsensor) < 0)
	    LOG_INFO("Failed to create device file(%s)!\n", dev_attr_gsensor.attr.name);
	/* /sys/class/xr-gsensor/device/delay_acc */
	if (device_create_file(gsensor_cmd_dev, &dev_attr_delay_acc) < 0)
	    LOG_INFO("Failed to create device file(%s)!\n", dev_attr_delay_acc.attr.name);
exit_alloc_platform_data_failed:
	return err;

}

static int qmax981_i2c_remove(struct i2c_client *client)
{
	struct qmax981_data *qmax981 = i2c_get_clientdata(client);

#ifdef QST_FUNC_DEBUG
	LOG_INFO("%s called\n", __func__);
#endif
	return qmax981_remove(qmax981->client);
}

static const struct i2c_device_id qmax981_id[] = {
	{QMAX981_DEV_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, qmax981_id);

static const struct of_device_id qmax981_acc_of_match[] = {
       { .compatible = "QST,qmax981", },
       { }
};

MODULE_DEVICE_TABLE(of, qmax981_acc_of_match);
static struct i2c_driver qmax981_driver = {
	.driver = {
		.name = QMAX981_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = qmax981_acc_of_match,
	},
	.probe    = qmax981_i2c_probe,
	.remove   = qmax981_i2c_remove,
	.id_table = qmax981_id,
};

static int __init qmax981_i2c_init(void)
{
	pr_debug("%s accelerometer driver: init\n", QMAX981_DEV_NAME);

	return i2c_add_driver(&qmax981_driver);
}

static void __exit qmax981_i2c_exit(void)
{
	pr_debug("%s accelerometer driver exit\n", QMAX981_DEV_NAME);

	i2c_del_driver(&qmax981_driver);
}


module_init(qmax981_i2c_init);
module_exit(qmax981_i2c_exit);

MODULE_DESCRIPTION("qmax981 accelerometer driver");
MODULE_AUTHOR("QST-inc");
MODULE_LICENSE("GPL");
MODULE_VERSION(QMAX981_DEV_VERSION);
