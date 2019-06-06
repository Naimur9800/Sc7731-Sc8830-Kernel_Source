/*
* File: altek_bist.h                                                        *
* Description: definition of sensor-related info                            *
*                                                                           *
* (C)Copyright altek Corporation 2016                                       *
*                                                                           *
* History:                                                                  *
*   2016/03/012; Caedmon Lai; Initial version                               *
*/

/*
 *@typedef ispdrv_sensor
 *@brief Define sensors
 */
enum ispdrv_sensor {
	IMX214,
};

/* set each sensor type */
#define SENSOR1_TYPE IMX214
#define SENSOR2_TYPE IMX214
#define SENSOR3_TYPE IMX214

/*
 *@typedef ispcmd_sensortype
 *@brief Define sensor type for different sensors.(0:isp_1, 1:isp_2, 2:isp_lite)
 */
enum ispcmd_sensortype {
	SENSOR1,	/* isp_1 */
	SENSOR2,	/* isp_2 */
	SENSOR3,	/* isp_lite */
	SENSOR_TOTAL
};
