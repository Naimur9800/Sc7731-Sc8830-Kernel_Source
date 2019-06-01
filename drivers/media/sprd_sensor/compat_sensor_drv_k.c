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

#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <video/sensor_drv_k.h>
#include "compat_sensor_drv_k.h"

//#define DEBUG_COMPAT_SENSOR_DRV
#ifdef  DEBUG_COMPAT_SENSOR_DRV
#define COMPAT_SENSOR_PRINT                      printk
#else
#define COMPAT_SENSOR_PRINT(...)
#endif

struct compat_sensor_reg_tab_tag {
	compat_caddr_t sensor_reg_tab_ptr;
	uint32_t reg_count;
	uint32_t reg_bits;
	uint32_t burst_mode;
};

struct compat_sensor_i2c_tag {
	compat_caddr_t i2c_data;
	uint16_t i2c_count;
	uint16_t slave_addr;
};

#define COMPAT_SENSOR_IO_I2C_WRITE_REGS    _IOW(SENSOR_IOC_MAGIC,  14, struct compat_sensor_reg_tab_tag)
#define COMPAT_SENSOR_IO_I2C_WRITE_EXT     _IOW(SENSOR_IOC_MAGIC,  17, struct compat_sensor_i2c_tag)
#define COMPAT_SENSOR_IO_I2C_READ_EXT      _IOWR(SENSOR_IOC_MAGIC, 20, struct compat_sensor_i2c_tag)

static long compat_get_sensor_reg_tab_tag(
			struct compat_sensor_reg_tab_tag __user *data32,
			struct sensor_reg_tab_tag __user *data)
{
	compat_caddr_t c;
	uint32_t i;
	int err;

	err = get_user(c, &data32->sensor_reg_tab_ptr);
	err |= put_user(c, &data->sensor_reg_tab_ptr);
	err |= get_user(i, &data32->reg_count);
	err |= put_user(i, &data->reg_count);
	err |= get_user(i, &data32->reg_bits);
	err |= put_user(i, &data->reg_bits);
	err |= get_user(i, &data32->burst_mode);
	err |= put_user(i, &data->burst_mode);

	COMPAT_SENSOR_PRINT("sensor_reg_tab_ptr: %p reg_count: %d reg_bits: %d burst_mode: %d\n",
		data->sensor_reg_tab_ptr, data->reg_count, data->reg_bits, data->burst_mode);
	return err;
}


static long compat_get_sensor_i2c_tag(
			struct compat_sensor_i2c_tag __user *data32,
			struct sensor_i2c_tag __user *data)
{
	compat_caddr_t c;
	uint16_t i;
	int err;

	err = get_user(c, &data32->i2c_data);
	err |= put_user(c, &data->i2c_data);
	err |= get_user(i, &data32->i2c_count);
	err |= put_user(i, &data->i2c_count);
	err |= get_user(i, &data32->slave_addr);
	err |= put_user(i, &data->slave_addr);

	COMPAT_SENSOR_PRINT("i2c_data: %p i2c_count: %d slave_addr: %d\n",
		data->i2c_data, data->i2c_count, data->slave_addr);
	return err;
}

long compat_sensor_k_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int compatible_arg = 1;
	long err = 0;

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_SENSOR_IO_I2C_WRITE_REGS:
	{
		struct compat_sensor_reg_tab_tag __user *data32;
		struct sensor_reg_tab_tag __user *data;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (NULL == data)
			return -EFAULT;

		err = compat_get_sensor_reg_tab_tag(data32, data);
		if (err)
			return err;
		return filp->f_op->unlocked_ioctl(filp, SENSOR_IO_I2C_WRITE_REGS,
								(unsigned long)data);
	}

	case COMPAT_SENSOR_IO_I2C_WRITE_EXT:
	{
		struct compat_sensor_i2c_tag __user *data32;
		struct sensor_i2c_tag __user *data;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (NULL == data)
			return -EFAULT;

		err = compat_get_sensor_i2c_tag(data32, data);
		if (err)
			return err;
		return filp->f_op->unlocked_ioctl(filp, SENSOR_IO_I2C_WRITE_EXT,
								(unsigned long)data);
	}

	case COMPAT_SENSOR_IO_I2C_READ_EXT:
	{
		struct compat_sensor_i2c_tag __user *data32;
		struct sensor_i2c_tag __user *data;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (NULL == data)
			return -EFAULT;

		err = compat_get_sensor_i2c_tag(data32, data);
		if (err)
			return err;
		return filp->f_op->unlocked_ioctl(filp, SENSOR_IO_I2C_READ_EXT,
								(unsigned long)data);
	}

	default:
		return filp->f_op->unlocked_ioctl(filp, cmd,
						(unsigned long)compat_ptr(arg));
	}
}
