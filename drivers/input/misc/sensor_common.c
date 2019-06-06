
#include <linux/types.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/module.h>

#include <sensor_common.h>

struct sensor_markinfo_t sensor_markinfo;

static int __init sensor_common_init(void)
{
	pr_info("init sensor common");
	sensor_markinfo.ACC_have_registered  = false;
	sensor_markinfo.PLS_have_registered  = false;
	sensor_markinfo.GYRO_have_registered = false;

	return 0;
}


static void __exit sensor_common_exit(void)
{
	pr_info("exit sensor common");
}

module_init(sensor_common_init);
module_exit(sensor_common_exit);

