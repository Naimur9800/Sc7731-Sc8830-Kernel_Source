/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 ************************************************
 * Automatically generated C config: don't edit *
 ************************************************
 */

#ifndef __CTL_THM_H__
#define __CTL_THM_H__

#include <linux/thermal.h>
#include <linux/types.h>

#define THM_CTRL            (0x0000)
#define THM_INT_CTRL        (0x0004)
#define SENSOR_CTRL         (0x0020)
#define SENSOR_DET_PERI     (0x0024)
#define SENSOR_INT_CTRL     (0x0028)
#define SENSOR_INT_STS      (0x002C)
#define SENSOR_INT_RAW_STS      (0x0030)
#define SENSOR_INT_CLR      (0x0034)
#define SENSOR_OVERHEAT_THRES   (0x0040)
#define SENSOR_HOT_THRES   (0X0044)
#define SENSOR_HOT2NOR_THRES   (0x0048)
#define SENSOR_HIGHOFF_THRES   (0x004C)
#define SENSOR_LOWOFF_THRES (0x0050)
#define SENSOR_MON_PERI		(0x0058)
#define SENSOR_MON_CTL		(0x005c)
#define SENSOR_TEMPER0_READ	(0x0060)
#define SENSOR_READ_STATUS	(0x0070)
#define RAW_READ_RANGE_MSK  0x7FFF

#define THM_THM_CTRL        (0x0000)
#define THM_THM_INT_CTRL       (0x0004)
#define THM_THM_INT_STS       (0x0008)
#define THM_SEN_CTRL         (0x0020)
#define THM_SEN_DET_PERI     (0x0024)
#define THM_SEN_INT_CTRL     (0x0028)
#define THM_SEN_INT_STS      (0x002C)
#define THM_SEN_INT_RAW_STS      (0x0030)
#define THM_SEN_INT_CLR      (0x0034)
#define THM_SEN_INT_CNT0      (0x0038)
#define THM_SEN_INT_CNT1      (0x003C)
#define THM_SEN_OVERHEAT_HOT_THRES   (0x0040)
#define THM_SEN_HOT2NOR_HIG_THRES   (0x0044)
#define THM_SEN_LOW_COLD_THRES    (0x0048)
#define THM_SEN_MON_PERI		(0x0050)
#define THM_SEN_MON_CTL		(0x0054)
#define THM_SEN_TEMPER0_READ	(0x0058)
#define THM_SEN_TEMPER1_READ	(0x005C)
#define THM_SEN_TEMPER2_READ	(0x0060)
#define THM_SEN_TEMPER3_READ	(0x0064)
#define THM_SEN_READ_STATUS	(0x0068)
#define THM_SEN_ANALOG_CTRL	(0x0070)
#define THM_SEN_RAW_READ_MSK  0xFF

#define THM_CTL            (0x0000)
#define THM_INT_CTL        (0x0004)
#define THM_INT_STS         (0x0008)
#define THM_INT_RAW_STS     (0x00c)
#define THM_DET_PERIOD_L16B     (0x0010)
#define THM_DET_PERIOD_H4B     (0x0014)
#define THM_INT_CLR      (0x0018)
#define THM_OVERHEAT_HOT_THRES   (0x0024)
#define THM_HOT2NOR_HIGH_THRES   (0x0028)
#define THM_LOW_COLD_THRES   (0x002c)
#define THM_MON_PERIOD		(0x0030)
#define THM_MON_CTL		(0x0034)
#define THM_LAST_TEMPER0_READ	(0x0038)
#define THM_RAW_READ_MSK  0xFF
#define THM_ANALOG_CTRL1 (0x50)

#define THM_NAME_LENGTH	20
#define THM_SENSOR_NUMBER	8

enum sprd_ap_thm_type {
	UNKNOWN_THM,
	SPRD_R1P0_THM,
	SPRD_R2P0_THM,
};

struct thm_control {
	struct sprd_thermal_zone *pzone[THM_SENSOR_NUMBER];
	const struct ap_thm_info *chip_info;
	struct regmap *thm_glb;
	struct clk *clk;
	const char *name[THM_SENSOR_NUMBER];
	struct list_head thmzone_list;
	int nsensor;

};

struct sensor_info {
	int id;
	char *sensor_name;
	unsigned long sensor_addr_off;
	unsigned int sen_enable;
	int raw_data_low;
	int raw_data_high;
	u32 algor_ver;
	u32 cal_k;
	u32 cal_b;
	u32 cal_efuse_blk;
	u32 cal_efuse_bit;
	u32 ratio_off_bit;
	u32 ratio_sign_bit;
	int temp_low;
	int temp_high;
	int thm_cal;
	int (*sensor_cal) (struct sprd_thermal_zone * pzone);
	int (*sensor_raw_temp) (struct sprd_thermal_zone * pzone);
	struct sensor_ops *ops;
};

struct thm_registers {
	u32 thm_ctl;
	u32 thm_int_ctl;
	u32 thm_int_sts;
	u32 thm_int_raw_sts;
	u32 thm_det_perold_l_16b;
	u32 thm_det_perold_h_4b;
	u32 thm_int_clr;
	u32 thm_overheat_hot_thres;
	u32 thm_hot2nor_high_thres;
	u32 thm_low_cold_thres;
	u32 thm_mon_peri;
	u32 thm_mon_ctl;
	u32 thm_temp0_read;
	u32 thm_read_mask;
};

struct sensor_registers {
	u32 thm_ctrl;
	u32 thm_int_ctrl;
	u32 thm_int_sts;
	u32 sen_ctrl;
	u32 sen_det_peri;
	u32 sen_int_ctrl;
	u32 sen_int_sts;
	u32 sen_int_raw_sts;
	u32 sen_int_clr;
	u32 sen_int_cnt0;
	u32 sen_int_cnt1;
	u32 sen_overheat_hot_thr;
	u32 sen_hot2nor_hig_thr;
	u32 sen_low_cold_thr;
	u32 sen_overheat_thr;
	u32 sen_hot_thres;
	u32 sen_hot2nor_thr;
	u32 sen_highoff_thr;
	u32 sen_lowoff_thr;
	u32 sen_cold_thr;
	u32 sen_mon_peri;
	u32 sen_mon_ctl;
	u32 sen_temp0_read;
	u32 sen_temp1_read;
	u32 sen_temp2_read;
	u32 sen_temp3_read;
	u32 sen_read_status;
	u32 sen_analog_ctrl;
	u32 sen_read_mask;
};

struct sensor_ops {
	void (*sensor_init) (struct sprd_thermal_zone * pzone);
	int (*sensor_read) (struct sprd_thermal_zone * pzone, int *);
	int (*sensor_suspend) (struct sprd_thermal_zone * pzone);
	int (*sensor_resume) (struct sprd_thermal_zone * pzone);
};

struct ipa_info {
	/*
	 * Proportional parameter of the PID controller when
	 * overshooting (i.e., when temperature is below the target)
	 */
	s32 k_po;

	/*
	 * Proportional parameter of the PID controller when
	 * undershooting
	 */
	s32 k_pu;

	/* Integral parameter of the PID controller */
	s32 k_i;

	/* Derivative parameter of the PID controller */
	s32 k_d;

	/* threshold below which the error is no longer accumulated */
	s32 integral_cutoff;
};

struct vol_temp_table {
	int x;
	int y;
};

struct temp_sen_config {
	int temp_tab_size;
	struct iio_channel *channel_temp;
	struct vol_temp_table *temp_tab;
};

struct sprd_thermal_zone {
	enum sprd_ap_thm_type thm_type;
	struct thermal_zone_device *therm_dev;
	struct mutex th_lock;
	struct device *dev;
	struct delayed_work resume_delay_work;
	struct delayed_work thm_resume_flag_work;
	struct thm_handle_ops *ops;
	struct sensor_ops *sen_ops;
	struct sensor_registers *reg;
	struct thm_registers *thmreg;
	struct sensor_info *msen_para;
	struct sensor_info *dsen_para;
	struct ipa_info ipa_para;
	struct list_head tz_node;
	struct regmap *thm_glb;
	struct clk *clk;
	struct temp_sen_config *pthm_config;
	unsigned long thm_base;
	unsigned long sensor_base;
	unsigned int rawdata;
	int otp_temp;
	int hot_temp;
	int hot2nor_temp;
	int highoff_temp;
	int lowoff_temp;
	int cold_temp;
	unsigned int otp_rawdata;
	unsigned int hot_rawdata;
	unsigned int hot2nor_rawdata;
	unsigned int highoff_rawdata;
	unsigned int lowoff_rawdata;
	unsigned int cold_rawdata;
	unsigned int sen_enable;
	int power_down;
	char name[THM_NAME_LENGTH];
	int sensor_cal;
	int sensor_cal_offset;
	int ready_flag;
	int lasttemp;
	int id;
};

struct thm_handle_ops {
	void (*hw_init) (struct sprd_thermal_zone *);
	int (*read_temp) (struct sprd_thermal_zone *, int *);
	int (*suspend) (struct sprd_thermal_zone *);
	int (*resume) (struct sprd_thermal_zone *);
};

int sprd_thermal_init(struct sprd_thermal_zone *pzone);
void sprd_thermal_remove(struct sprd_thermal_zone *pzone);
#endif
