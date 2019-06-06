/*
 * File:shub_opcode.h
 * Author:bao.yue@spreadtrum.com
 * Created:2015-10-21
 * Description:Spreadtrum Sensor Hub opcode header file
 *
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#ifndef SPRD_SHUB_OPCODE_H
#define SPRD_SHUB_OPCODE_H

#include <linux/firmware.h>

#define SHUB_SENSOR_NAME   "sprd-shub"
#define SHUB_SENSOR_NAME_LENGTH 10

struct iic_unit {
	/* read/write/msleep */
	uint8_t addr;		/* 0xFF means delay */
	uint8_t val;
	uint8_t shift;
	uint8_t mask;
};

struct fwshub_unit {
	uint32_t cmd;
	uint32_t shift;
	uint32_t units;
};

struct fwshub_head {
	char name[SHUB_SENSOR_NAME_LENGTH];
	int type;
	struct fwshub_unit index_opcode[8];
};
#endif
