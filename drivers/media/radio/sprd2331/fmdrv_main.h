/*
 *  FM Drivr for Connectivity chip of Spreadtrum.
 *
 *  FM driver main module header.
 *
 *  Copyright (C) 2015 Spreadtrum Company
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#ifndef _FMDRV_MAIN_H
#define _FMDRV_MAIN_H

#include <linux/fs.h>

int fm_open(struct inode *inode, struct file *filep);
int fm_powerup(void);
int fm_powerdown(void);
int fm_tune(void *);
int fm_seek(void *);
int fm_mute(void *);
int fm_rds_onoff(void *);
int fm_read_rds_data(struct file *filp, char __user *buf,
	size_t count, loff_t *pos);
#endif
