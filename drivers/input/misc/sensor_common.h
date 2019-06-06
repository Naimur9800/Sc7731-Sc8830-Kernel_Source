/*
 * sensor common info
 *
 */

#ifndef _SENSOR_COMMON_H_INCLUDE
#define _SENSOR_COMMON_H_INCLUDE

struct sensor_markinfo_t {
	bool ACC_have_registered;
	bool PLS_have_registered;
	bool GYRO_have_registered;
};

extern struct sensor_markinfo_t sensor_markinfo;
#endif
