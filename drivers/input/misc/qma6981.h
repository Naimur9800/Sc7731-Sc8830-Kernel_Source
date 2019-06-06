#ifndef __QMAX981_H__
#define __QMAX981_H__

#include <linux/ioctl.h>

#define QMAX981_ACC_I2C_ADDR     0x12
#define QMAX981_ACC_I2C_NAME     "qmax981"


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

struct QMAX981_acc_platform_data {
	int layout;
	int gpio_int1;
	};
	/* __KERNEL__ */

#define GSENSOR                                0x95
#define GSENSOR_IOCTL_INIT                     _IO(GSENSOR,  0x01)
#define GSENSOR_IOCTL_READ_CHIPINFO            _IOR(GSENSOR, 0x02, int)
#define GSENSOR_IOCTL_READ_SENSORDATA          _IOR(GSENSOR, 0x03, int)
#define GSENSOR_IOCTL_READ_OFFSET              _IOR(GSENSOR, 0x04, GSENSOR_VECTOR3D)
#define GSENSOR_IOCTL_READ_GAIN                _IOR(GSENSOR, 0x05, GSENSOR_VECTOR3D)
#define GSENSOR_IOCTL_READ_RAW_DATA            _IOR(GSENSOR, 0x06, int)
#define GSENSOR_IOCTL_GET_CALI                 _IOW(GSENSOR, 0x07, SENSOR_DATA)
#define GSENSOR_IOCTL_CLR_CALI                 _IO(GSENSOR, 0x08)
#define GSENSOR_IOCTL_SET_CALI_MODE            _IOW(GSENSOR, 0x0e, int)



#define GSENSOR_MCUBE_IOCTL_READ_RBM_DATA      _IOR(GSENSOR, 0x09, SENSOR_DATA)
#define GSENSOR_MCUBE_IOCTL_SET_RBM_MODE       _IO(GSENSOR, 0x0a)
#define GSENSOR_MCUBE_IOCTL_CLEAR_RBM_MODE     _IO(GSENSOR, 0x0b)
#define GSENSOR_MCUBE_IOCTL_SET_CALI           _IOW(GSENSOR, 0x0c, SENSOR_DATA)
#define GSENSOR_MCUBE_IOCTL_REGISTER_MAP       _IO(GSENSOR, 0x0d)

#define GSENSOR_MCUBE_IOCTL_READ_PRODUCT_ID    _IOR(GSENSOR, 0x11, int)
#define GSENSOR_MCUBE_IOCTL_READ_CHIP_ID       _IOR(GSENSOR, 0x12, int)
#define GSENSOR_MCUBE_IOCTL_READ_FILEPATH      _IOR(GSENSOR, 0x13, char[256])


#endif
