/*
 * File:shub_core.h
 * Author:bao.yue@spreadtrum.com
 * Created:2015-10-21
 * Description:Spreadtrum Sensor Hub core header file
 *
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#ifndef SPRD_SHUB_CORE_H
#define SPRD_SHUB_CORE_H

#include "shub_common.h"

#define SENSOR_TYPE_CALIBRATION_CFG				(26)
#define SHUB_NAME                   "SPRDSensor"
#define SHUB_RD                   "shub_rd"
#define SHUB_RD_NWU                   "shub_rd_nwu"
#define SHUB_SENSOR_NUM		(7)
#define SHUB_NODATA                 0xFF
#define HOST_REQUEST_WRITE           0x74
#define HOST_REQUEST_READ            0x75
#define HOST_REQUEST_LEN             2
#define RECEIVE_TIMEOUT_MS           100
#define MAX_SENSOR_LOG_CTL_FLAG_LEN	8
#define SIPC_PM_BUFID0             0
#define SIPC_PM_BUFID1             1

/* ms,-1 is wait  forever */
#define SIPC_WRITE_TIMEOUT             -1

enum calib_cmd {
	CALIB_EN = 0,
	CALIB_CHECK_STATUS = 1,
	CALIB_DATA_WRITE = 2,
	CALIB_DATA_READ = 3,
	CALIB_FLASH_WRITE = 4,
	CALIB_FLASH_READ = 5,
};

enum calib_type {
	CALIB_TYPE_NON = 0,
	CALIB_TYPE_DEFAULT = 1,
	CALIB_TYPE_SELFTEST = 2,
	CALIB_TYPE_SENSORS_ENABLE = 3,
	CALIB_TYPE_SENSORS_DISABLE = 4,
};

enum calib_status {
	CALIB_STATUS_OUT_OF_RANGE = -2,
	CALIB_STATUS_FAIL = -1,
	CALIB_STATUS_NON = 0,
	CALIB_STATUS_INPROCESS = 1,
	CALIB_STATUS_PASS = 2,
};

struct  sensor_info {
	uint8_t en;
	uint8_t mode;
	uint8_t rate;
	uint16_t timeout;
};

struct sensor_report_format {
	uint8_t cmd;
	uint8_t length;
	uint16_t id;
	uint8_t data[];
};

/*
 *   Description:
 *   Control Handle
 */
enum  handle_id {
	NON_WAKEUP_HANDLE = 0,
	WAKEUP_HANDLE = 1,
	INTERNAL_HANDLE = 2,
	HANDLE_ID_END
};

enum scan_status {
	SHUB_SCAN_ID                       = 0,
	SHUB_SCAN_RAW_0                    = 1,
	SHUB_SCAN_RAW_1                    = 2,
	SHUB_SCAN_RAW_2                    = 3,
	SHUB_SCAN_TIMESTAMP                = 4,
};

enum shub_mode {
	SHUB_BOOT = 0,
	SHUB_OPDOWNLOAD = 1,
	SHUB_NORMAL = 2,
	SHUB_SLEEP = 3,
	SHUB_NO_SLEEP = 4,
};

enum shub_cmd {
	/* MCU Internal Cmd: Range 0~63 */
	/* Driver Control enable/disable/set rate */
	MCU_CMD = 0,
	MCU_SENSOR_EN,
	MCU_SENSOR_DSB,
	MCU_SENSOR_SET_RATE,
	/* Sensors Calibrator */
	MCU_SENSOR_CALIB,
	MCU_SENSOR_SELFTEST,
	/* User profile */
	MCU_SENSOR_SETTING,
	KNL_CMD = 64,
	KNL_SEN_DATA,
	 /* for multi package use */
	KNL_SEN_DATAPAC,
	KNL_EN_LIST,
	KNL_LOG,
	KNL_RESET_SENPWR,
	KNL_CALIBINFO,
	KNL_CALIBDATA,
	HAL_CMD = 128,
	HAL_SEN_DATA,
	HAL_FLUSH,
	HAL_LOG,
	HAL_SENSOR_INFO,
	HAL_LOG_CTL,
	SPECIAL_CMD = 192,
	MCU_RDY,
	COMM_DRV_RDY,
};

struct sensor_batch_cmd {
	int handle;
	int report_rate;
	int64_t batch_timeout;
};

struct sensor_log_control {
	uint8_t cmd;
	uint8_t length;
	uint8_t debug_data[5];
	uint32_t udata[MAX_SENSOR_LOG_CTL_FLAG_LEN];
};

struct shub_data {
	struct platform_device *sensor_pdev;
	enum shub_mode mcu_mode;
	/* enable & batch list */
	u32 enabled_list[HANDLE_ID_END];
	u32 interrupt_status;
	/* Calibrator status */
	int cal_cmd;
	int cal_type;
	int cal_id;
	int golden_sample;
	char *calibrated_data;
	int cal_savests;
	s32 iio_data[6];
	struct iio_dev *indio_dev;
	struct irq_work iio_irq_work;
	struct iio_trigger  *trig;
	atomic_t pseudo_irq_enable;
	struct class *sensor_class;
	struct device *sensor_dev;
	wait_queue_head_t rxwq;
	struct mutex mutex_lock;
	struct mutex mutex_read;
	struct mutex mutex_send;
	bool rx_status;
	u8 *rx_buf;
	u32 rx_len;
	/* sprd begin */
	wait_queue_head_t  rw_wait_queue;
	struct sent_cmd  sent_cmddata;
	struct mutex send_command_mutex;
	u8 *regs_addr_buf;
	u8 *regs_value_buf;
	u8 regs_num;
	struct file *filep;/* R/W interface */
	unsigned char readbuff[SERIAL_READ_BUFFER_MAX];
	unsigned char readbuff_nwu[SERIAL_READ_BUFFER_MAX];
	unsigned char writebuff[SERIAL_WRITE_BUFFER_MAX];
	void (*save_mag_offset)(struct shub_data *sensor, u8 *buff, u32 len);
	void (*data_callback)(struct shub_data *sensor, u8 *buff, u32 len);
	void (*readcmd_callback)(struct shub_data *sensor, u8 *buff, u32 len);
	void (*resp_cmdstatus_callback)(struct shub_data *sensor,
		u8 *buff, u32 len);
	void (*cm4_read_callback)(struct shub_data *sensor,
		enum shub_subtype_id subtype,  u8 *buff, u32 len);
	struct sensor_log_control log_control;
	struct workqueue_struct *driver_wq;
	struct delayed_work delay_work;
	atomic_t delay;
	struct work_struct download_work;
	struct work_struct savecalifile_work;
	struct notifier_block early_suspend;
	int is_sensorhub;
};
extern struct shub_data *g_sensor;
/*hw sensor id*/
enum _id_status {
	_IDSTA_NOT = 0,
	_IDSTA_OK  = 1,
	_IDSTA_FAIL = 2,
};
#define _HW_SENSOR_TOTAL 6
struct hw_sensor_id_tag {
	u8 id_status;
	u8 vendor_id;
	u8 chip_id;
	char *pname;
};
struct id_to_name_tag {
	u32 id;
	char *pname;
};
#endif
