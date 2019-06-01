#ifndef __FT_PSENSOR_DRV_H__
#define __FT_PSENSOR_DRV_H__

#define  FT_PSENSOR_DRV  "ft_psensor_drv"
#define FT_PSENSOR_DRV_MAJOR 211    /*预设的FT_PSENSOR_DRV的主设备号*/

#define FT_PSENSOR_IOCTL_ENABLE          	11
#define FT_PSENSOR_IOCTL_GET_ENABLE          	12

#define PSENSOR_ENABLE_REG		0xB0	
#define ENABLE_PSENSOR		1
#define DISABLE_PSENSOR 	0


int ft_psensor_drv_init(struct i2c_client *client);
void  ft_psensor_drv_exit(void);
#endif

