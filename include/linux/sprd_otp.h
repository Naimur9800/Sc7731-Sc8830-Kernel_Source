/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SPRD_OTP_H__
#define __SPRD_OTP_H__

#include <linux/miscdevice.h>

enum sprd_pmic_efuse_type {
	UNKNOWN_PMIC_EFUSE,
	SC2723_EFUSE,
	SC2731_EFUSE,
	SC2721_EFUSE,
	SC2720_EFUSE,
};

struct sprd_otp_operations {
	void (*reset)(void);
	u32 (*read)(int blk_index);
	int (*prog)(int blk_index, u32 data);
};

struct sprd_otp_device {
	struct miscdevice misc;
	int blk_max;
	int blk_width;
	struct sprd_otp_operations *ops;
	struct list_head list;
};

u32 sprd_pmic_efuse_bits_read(int bit_index, int length);
u32 sprd_pmic_efuse_block_read(int blk_index);
u32 sprd_ap_efuse_read(int blk_index);
int sprd_ap_efuse_prog(int blk_index, u32 val);
u32 sprd_efuse_double_read(int blk, bool backup);
int sprd_efuse_double_prog(int blk, bool backup, bool lock, u32 val);
void sprd_get_chip_uid(char *buf);
struct device *sprd_otp_register(const char *name, void *ops, int blk_max,
				 int blk_width);

#endif
