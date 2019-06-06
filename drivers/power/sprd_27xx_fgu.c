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
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/wakelock.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include "sprd_27xx_fgu.h"
#include "sprd_charge_helper.h"

#define BITSINDEX(b, o, w)	((b) * (w) + (o))
#define SPRD_FGU_DEBUG(format, arg...) pr_info("sprdfgu: " format, ## arg)

#define BAT_FILE_PATH "/mnt/vendor/battery/calibration_data/.battery_file"
#define CAP_KEY0	0x20160726
#define CAP_KEY1	0x15211517

#define QMAX_START_OCV_LIMIT	3600
#define QMAX_START_CUR_LIMIT	30
#define QMAX_TIMEOUT		108000
#define SPRDFGU_VOL_ONLY_CAP_BUFF_CNT		8
static int sprdfgu_vol_only_cap_buff[SPRDFGU_VOL_ONLY_CAP_BUFF_CNT];

enum QMAX_STATE {
	QMAX_INIT,
	QMAX_IDLE,
	QMAX_UPDATING,
	QMAX_DONE,
	QMAX_ERR,
};

struct clbcnt_test {
	int s_cc;
	int cc_keep;
	int time_keep;
	int avg_cur;
	int ongoing;
	__s64 s_time;
};

struct clbcnt_test cc_test;

#define FULL_CAP	(1000)

static void sprdfgu_record_rawsoc(u32 cap);
static inline uint32_t sprdfgu_load_uicap(void);
static inline uint32_t sprdfgu_load_rawsoc(void);
static struct sprdfgu_tracking sprdfgu_get_track_data(void);

static struct regmap *reg_map;

static struct sprdfgu_drivier_data sprdfgu_data;
static struct sprdfgu_cal fgu_cal = { 2872, 678, 0, 0, SPRDBAT_FGUADC_CAL_NO };

/* for debug */
static struct delayed_work sprdfgu_debug_work;
static uint32_t sprdfgu_debug_log_time = 120;
static int poweron_clbcnt;
static __s64 start_time;
static uint32_t request_vchg_gpio;
/* for debug end */

static int fgu_nv_4200mv = 2752;
static int fgu_nv_3600mv = 2374;
static int fgu_0_cur_adc = 8338;

static int cmd_vol_raw, cmd_cur_raw;

static const struct of_device_id sprdfgu_2723_of_match[] = {
	{.compatible = "sprd,sc2723-fgu", .data = (void *)FGU_2723},
	{.compatible = "sprd,sc2731-fgu", .data = (void *)FGU_2731},
	{.compatible = "sprd,sc2721-fgu", .data = (void *)FGU_2721},
	{.compatible = "sprd,sc2720-fgu", .data = (void *)FGU_2720},
	{}
};

static unsigned int fgu_adi_read(unsigned long reg)
{
	unsigned int val;

	regmap_read(reg_map,
		sprdfgu_data.fgu_pdata->base_addr + reg, &val);
	return val;
}

static int fgu_adi_write(unsigned long reg, unsigned int or_val,
				 unsigned int clear_msk)
{
	return regmap_update_bits(reg_map,
		sprdfgu_data.fgu_pdata->base_addr + reg,
		clear_msk, or_val);
}

#ifdef CONFIG_OTP_SPRD_PMIC_EFUSE
static int sprdfgu_cal_get(unsigned int *p_cal_data)
{
	unsigned int data, blk0;

	if (sprdfgu_data.fgu_pdata->fgu_type == FGU_2723) {
		blk0 = sprd_pmic_efuse_block_read(0);

		if (blk0 & BIT(7))
			return 0;

		data = sprd_pmic_efuse_bits_read(BITSINDEX(12, 0, 8), 9);
		pr_info("fgu_cal_get 4.2 data data:0x%x\n", data);
		p_cal_data[0] = (data + 6963) - 4096 - 256;

		data = sprd_pmic_efuse_bits_read(BITSINDEX(14, 2, 8), 6);
		pr_info("fgu_cal_get 3.6 data data:0x%x\n", data);
		p_cal_data[1] = p_cal_data[0] - 410 - data + 16;

		data = sprd_pmic_efuse_bits_read(BITSINDEX(13, 1, 8), 9);
		pr_info("fgu_cal_get 0 data data:0x%x\n", data);
		p_cal_data[2] = (((data) & 0x1FF) + 8192) - 256;
	} else if ((sprdfgu_data.fgu_pdata->fgu_type == FGU_2731)
		|| (sprdfgu_data.fgu_pdata->fgu_type == FGU_2721)
		|| (sprdfgu_data.fgu_pdata->fgu_type == FGU_2720)) {
		data = sprd_pmic_efuse_bits_read(BITSINDEX(3, 0, 16), 9);
		pr_info("fgu_cal_get 4.2 data data:0x%x\n", data);
		p_cal_data[0] = (data + 6963) - 4096 - 256;
		data = sprd_pmic_efuse_bits_read(BITSINDEX(4, 9, 16), 6);
		pr_info("fgu_cal_get 3.6 data data:0x%x\n", data);
		p_cal_data[1] = p_cal_data[0] - 410 - data + 32;
		data = sprd_pmic_efuse_bits_read(BITSINDEX(4, 0, 16), 9);
		pr_info("fgu_cal_get 0 data data:0x%x\n", data);
		p_cal_data[2] = (((data) & 0x1FF) + 8192) - 256;
	}
	return 1;
}
#endif
static int sprdfgu_vol2capacity(uint32_t voltage)
{
	int percentum = sprdbat_interpolate(voltage,
				     sprdfgu_data.pdata->ocv_tab_size,
				     sprdfgu_data.pdata->ocv_tab);

	return percentum * 10;
}

static int sprdfgu_charge_vol2capacity(uint32_t voltage)
{
	int percentum = sprdbat_interpolate(voltage,
				     sprdfgu_data.pdata->charge_vol_tab_size,
				     sprdfgu_data.pdata->charge_vol_tab);

	return percentum * 10;
}

static int sprdfgu_discharge_vol2capacity(uint32_t voltage)
{
	int percentum = sprdbat_interpolate(voltage,
				     sprdfgu_data.pdata->discharge_vol_tab_size,
				     sprdfgu_data.pdata->discharge_vol_tab);

	return percentum * 10;
}

static int __init fgu_cal_start(char *str)
{
	unsigned int fgu_data[3] = { 0 };
	char *cali_data = &str[1];
	int res;

	if (str) {
		SPRD_FGU_DEBUG("fgu_cal%s!\n", str);
		res = sscanf(cali_data, "%d,%d,%d", &fgu_data[0], &fgu_data[1],
		       &fgu_data[2]);
		if (res != 3)
			return -1;
		SPRD_FGU_DEBUG("fgu_data: 0x%x 0x%x,0x%x!\n",
			fgu_data[0], fgu_data[1], fgu_data[2]);
		fgu_nv_4200mv = (fgu_data[0] >> 16) & 0xffff;
		fgu_nv_3600mv = (fgu_data[1] >> 16) & 0xffff;
		fgu_0_cur_adc = fgu_data[2];
		fgu_cal.cal_type = SPRDBAT_FGUADC_CAL_NV;
	}
	return 1;
}

__setup("fgu_cal", fgu_cal_start);

static int __init fgu_cmd(char *str)
{
	int res, fgu_data[2] = { 0 };
	char *cali_data = &str[1];

	if (str) {
		pr_info("fgu cmd%s!\n", str);
		res = sscanf(cali_data, "%d,%d", &fgu_data[0], &fgu_data[1]);
		if (res != 2)
			return -1;
		pr_info("fgu_cmd adc_data: 0x%x 0x%x!\n", fgu_data[0],
			fgu_data[1]);
		cmd_vol_raw = fgu_data[0];
		cmd_cur_raw = fgu_data[1];
	}
	return 1;
}

__setup("fgu_init", fgu_cmd);

static int sprdfgu_cal_init(void)
{
	if (fgu_nv_3600mv == 0) {
		fgu_cal.vol_1000mv_adc =
		    DIV_ROUND_CLOSEST((fgu_nv_4200mv) * 10, 42);
		fgu_cal.vol_offset = 0;
		fgu_cal.cur_offset = 0;
		fgu_cal.cur_1000ma_adc =
		    DIV_ROUND_CLOSEST(fgu_cal.vol_1000mv_adc * 4 *
				      sprdfgu_data.pdata->rsense_real,
				      sprdfgu_data.pdata->rsense_spec);
	} else {
		fgu_cal.vol_1000mv_adc =
		    DIV_ROUND_CLOSEST((fgu_nv_4200mv - fgu_nv_3600mv) * 10, 6);
		fgu_cal.vol_offset =
		    0 - (fgu_nv_4200mv * 10 - fgu_cal.vol_1000mv_adc * 42) / 10;
		fgu_cal.cur_offset = CUR_0ma_IDEA_ADC - fgu_0_cur_adc;
		fgu_cal.cur_1000ma_adc =
		    DIV_ROUND_CLOSEST(fgu_cal.vol_1000mv_adc * 4 *
				      sprdfgu_data.pdata->rsense_real,
				      sprdfgu_data.pdata->rsense_spec);
	}
	if (fgu_cal.cal_type == SPRDBAT_FGUADC_CAL_CHIP) {
		fgu_cal.vol_offset += sprdfgu_data.pdata->fgu_cal_ajust;
		SPRD_FGU_DEBUG("fgu_cal_ajust = %d\n",
		       sprdfgu_data.pdata->fgu_cal_ajust);
	}

	SPRD_FGU_DEBUG("%s,4200mv=%d,3600mv=%d\n",
		__func__, fgu_nv_4200mv, fgu_nv_3600mv);
	SPRD_FGU_DEBUG("%s,1000ma = %d,1000mv=%d\n",
		__func__, fgu_cal.cur_1000ma_adc,
		fgu_cal.vol_1000mv_adc);
	SPRD_FGU_DEBUG("%s,fgu_0_cur_adc=%d\n",
		__func__, fgu_0_cur_adc);
	SPRD_FGU_DEBUG("%s,vol_offset=%d,cur_offset = %d\n",
		__func__, fgu_cal.vol_offset, fgu_cal.cur_offset);
	return 0;
}

static int sprdfgu_cal_from_chip(void)
{
	unsigned int fgu_data[4] = { 0 };

#ifdef CONFIG_OTP_SPRD_PMIC_EFUSE
	if (!sprdfgu_cal_get(fgu_data)) {
		SPRD_FGU_DEBUG("%s efuse no cal data\n", __func__);
		return 1;
	}
#else
	SPRD_FGU_DEBUG("%s no sprd efuse\n", __func__);
	return 1;
#endif
	SPRD_FGU_DEBUG(" fgu_data: 0x%x 0x%x,0x%x,0x%x!\n",
	fgu_data[0], fgu_data[1], fgu_data[2], fgu_data[3]);
	SPRD_FGU_DEBUG("sprdfgu_cal_from_chip\n");

	fgu_nv_4200mv = fgu_data[0];

	fgu_nv_3600mv = 0;
	fgu_0_cur_adc = 0;
	SPRD_FGU_DEBUG("sprdfgu: one point\n");

	fgu_cal.cal_type = SPRDBAT_FGUADC_CAL_CHIP;

	return 0;
}

static u32 sprdfgu_adc2vol_mv(u32 adc)
{
	return ((adc + fgu_cal.vol_offset) * 1000) / fgu_cal.vol_1000mv_adc;
}

static u32 sprdfgu_vol2adc_mv(u32 vol)
{
	return (vol * fgu_cal.vol_1000mv_adc) / 1000 - fgu_cal.vol_offset;
}

static int sprdfgu_adc2cur_ma(int adc)
{
	return (adc * 1000) / fgu_cal.cur_1000ma_adc;
}

static u32 sprdfgu_cur2adc_ma(u32 cur)
{
	return (cur * fgu_cal.cur_1000ma_adc) / 1000;
}

static void sprdfgu_rtc_reg_write(uint32_t val)
{
	fgu_adi_write(REG_FGU_USER_AREA_CLEAR, BITS_RTC_AREA(~val),
		      BITS_RTC_AREA(~0U));
	fgu_adi_write(REG_FGU_USER_AREA_SET, BITS_RTC_AREA(val),
		      BITS_RTC_AREA(~0U));
}

static uint32_t sprdfgu_rtc_reg_read(void)
{
	int shft = __ffs(BITS_RTC_AREA(~0U));

	return (fgu_adi_read(REG_FGU_USER_AREA_STATUS) & BITS_RTC_AREA(~0)) >>
	    shft;
}

static void sprdfgu_poweron_type_write(uint32_t val)
{
	fgu_adi_write(REG_FGU_USER_AREA_CLEAR, BITS_POWERON_TYPE(~val),
		      BITS_POWERON_TYPE(~0U));
	fgu_adi_write(REG_FGU_USER_AREA_SET, BITS_POWERON_TYPE(val),
		      BITS_POWERON_TYPE(~0U));
}

static uint32_t sprdfgu_poweron_type_read(void)
{
	int shft = __ffs(BITS_POWERON_TYPE(~0U));

	return (fgu_adi_read(REG_FGU_USER_AREA_STATUS) & BITS_POWERON_TYPE(~0))
	    >> shft;
}

static void sprdfgu_rtc_reg1_write(uint32_t val)
{
	fgu_adi_write(REG_FGU_USER_AREA_CLEAR1, BITS_RTC_AREA(~val),
		      BITS_RTC_AREA(~0U));
	fgu_adi_write(REG_FGU_USER_AREA_SET1, BITS_RTC_AREA(val),
		      BITS_RTC_AREA(~0U));
}

static uint32_t sprdfgu_rtc_reg1_read(void)
{
	int shft = __ffs(BITS_RTC_AREA(~0U));

	return (fgu_adi_read(REG_FGU_USER_AREA_STATUS1) & BITS_RTC_AREA(~0U)) >>
	    shft;
}

static int sprdfgu_clbcnt_get(void)
{
	int cc1 = 0, cc2 = 1, times = 100;

	do {
		udelay(10);

		cc1 = (fgu_adi_read(REG_FGU_CLBCNT_VALL)) & 0xFFFF;
		cc1 |= (((fgu_adi_read(REG_FGU_CLBCNT_VALH)) & 0xFFFF) << 16);

		cc2 = (fgu_adi_read(REG_FGU_CLBCNT_VALL)) & 0xFFFF;
		cc2 |= (((fgu_adi_read(REG_FGU_CLBCNT_VALH)) & 0xFFFF) << 16);
	} while ((cc1 != cc2) && (times--));
	return cc1;
}

static int sprdfgu_clbcnt_set(int clbcc)
{
	fgu_adi_write(REG_FGU_CLBCNT_SETL, clbcc & 0xFFFF, ~0U);
	fgu_adi_write(REG_FGU_CLBCNT_SETH, (clbcc >> 16) & 0xFFFF, ~0U);
	fgu_adi_write(REG_FGU_START, BIT_WRITE_SELCLB_EN,
		BIT_WRITE_SELCLB_EN);
	udelay(130);
	return 0;
}

static  int sprdfgu_reg_get(unsigned long reg)
{
	int old_value = fgu_adi_read(reg);
	int new_value, times = 100;

	while ((old_value != (new_value = fgu_adi_read(reg))) &&
		(times > 0)) {
		old_value = new_value;
		cpu_relax();
		times--;
	}
	return new_value;
}

static int sprdfgu_clbcnt_init(u32 capacity)
{
	int init_cap =
	    DIV_ROUND_CLOSEST(sprdfgu_data.cur_cnom * capacity, FULL_CAP);
	int clbcnt =
	    DIV_ROUND_CLOSEST(init_cap * fgu_cal.cur_1000ma_adc * 36 *
			      FGU_CUR_SAMPLE_HZ, 10);

	return clbcnt;
}

static void sprdfgu_soc_adjust(int capacity)
{
	sprdfgu_data.init_cap = capacity;
	sprdfgu_data.init_clbcnt = sprdfgu_clbcnt_get();

	SPRD_FGU_DEBUG("%s  init_cap= %d,%d\n",
		  __func__, sprdfgu_data.init_cap, sprdfgu_data.init_clbcnt);
}

uint32_t sprdfgu_read_vbat_vol(void)
{
	u32 cur_vol_raw;
	uint32_t temp;

	cur_vol_raw = sprdfgu_reg_get(REG_FGU_VOLT_VAL);
	temp = sprdfgu_adc2vol_mv(cur_vol_raw);
	return temp;
}

static inline u32 sprdfgu_ocv_vol_get(void)
{
	u32 ocv_vol_raw;

	ocv_vol_raw = sprdfgu_reg_get(REG_FGU_OCV_VAL);
	return sprdfgu_adc2vol_mv(ocv_vol_raw);
}

static int sprdfgu_cur_current_get(void)
{
	int current_raw;

	current_raw = sprdfgu_reg_get(REG_FGU_CURT_VAL);
	return sprdfgu_adc2cur_ma(current_raw - CUR_0ma_IDEA_ADC);
}

int sprdfgu_read_batcurrent(void)
{
	if (sprdfgu_data.pdata->only_vol_mode)
		return 0;

	return sprdfgu_cur_current_get();
}

#if 0
static int sprdfgu_read_vbat_ocv_pure(uint32_t *vol)
{
	if (sprdfgu_reg_get(REG_FGU_OCV_LAST_CNT) > SPRDFGU_OCV_VALID_TIME
	    || sprdfgu_reg_get(REG_FGU_OCV_VAL) == 0) {
		*vol = 0;
		return 0;
	}
	*vol = sprdfgu_ocv_vol_get();
	return 1;
}
#endif

uint32_t sprdfgu_read_vbat_ocv(void)
{
	int rint = sprdfgu_data.cur_rint;

#ifdef SPRDFGU_TEMP_COMP_SOC
	{
		int temp, percent;

		temp = sprdbat_read_temp();
		percent = sprdbat_interpolate(temp/10,
			sprdfgu_data.pdata->rint_temp_tab_size,
			sprdfgu_data.pdata->rint_temp_tab);
		rint = (rint * percent) / 100;
		SPRD_FGU_DEBUG("rint:%d,temp:%d\n", rint, temp);
	}
#endif

#if 0
	if (sprdfgu_read_vbat_ocv_pure(&vol)) {
		SPRD_FGU_DEBUG("hwocv...\n");
		return vol;
	} else {
		return sprdfgu_read_vbat_vol() -
		    (sprdfgu_read_batcurrent() * rint) / 1000;
	}
#endif

	return sprdfgu_read_vbat_vol() -
		    (sprdfgu_read_batcurrent() * rint) / 1000;
}

#ifdef SPRDFGU_TEMP_COMP_SOC
static int sprdfgu_temp_comp_soc(int soc, int temp)
{
	int cnom_temp = 0;
	int comp_soc, delta_soc;

	cnom_temp = sprdbat_interpolate(temp,
		sprdfgu_data.pdata->cnom_temp_tab_size,
		sprdfgu_data.pdata->cnom_temp_tab);

	delta_soc = (FULL_CAP - cnom_temp);

	comp_soc = (long)(soc - delta_soc) * FULL_CAP / (FULL_CAP - delta_soc);

	if (comp_soc < 0)
		comp_soc = 0;

	if (comp_soc > FULL_CAP)
		comp_soc = FULL_CAP;

	SPRD_FGU_DEBUG("cnom_temp %d, delta_soc %d\n",
		cnom_temp, delta_soc);
	SPRD_FGU_DEBUG("comp_soc%d,soc %d,temp %d\n",
		comp_soc, soc, temp);

	return comp_soc;
}
#endif

static void sprdfgu_clbcnt_test(unsigned long start)
{
	struct timespec64 cur_time = ktime_to_timespec64(ktime_get_boottime());

	if (start) {
		cc_test.s_time = cur_time.tv_sec;
		cc_test.s_cc = sprdfgu_clbcnt_get();
		cc_test.ongoing = 1;
	} else {
		int cur_clbcnt = sprdfgu_clbcnt_get();
		int cc_delta, temp;

		cc_test.time_keep = cur_time.tv_sec - cc_test.s_time;

		cc_delta = cur_clbcnt - cc_test.s_cc;
		temp = DIV_ROUND_CLOSEST(cc_delta,
			(3600 * FGU_CUR_SAMPLE_HZ));
		cc_test.cc_keep = sprdfgu_adc2cur_ma(temp);

		temp = cc_test.time_keep << 1;
		if (temp) {
			temp = cc_delta / temp;
			cc_test.avg_cur = sprdfgu_adc2cur_ma(temp);
		} else
			cc_test.avg_cur = sprdfgu_read_batcurrent();
		cc_test.ongoing = 0;
	}
}

static void sprdfgu_qmax_update_monitor(void)
{
	int *pstate = &sprdfgu_data.qmax.state;
	struct timespec64 cur_time;
	struct sprdfgu_tracking track = sprdfgu_get_track_data();

	if (!sprdfgu_data.is_track)
		return;

	SPRD_FGU_DEBUG("qmax.state:%d\n", *pstate);

	if (!sprdbat_is_bat_present())
		return;

	if (track.c_temp < 150)
		return;

	if (track.relax_vol == 0)
		return;

	if (*pstate == QMAX_ERR)
		return;
	else if (*pstate == QMAX_IDLE) {
		uint32_t ocv;

		if (sprdfgu_data.qmax.force_s_flag) {
			sprdfgu_data.qmax.force_s_flag = 0;
			sprdfgu_data.qmax.s_soc = sprdfgu_data.qmax.force_s_soc;
		} else {
			if (abs(track.relax_cur) > sprdfgu_data.qmax.s_cur)
				return;

			if (track.relax_vol > sprdfgu_data.qmax.s_vol)
				return;

			ocv = track.relax_vol -
				(track.relax_cur * sprdfgu_data.cur_rint)
				/ 1000;
			sprdfgu_data.qmax.s_soc = sprdfgu_vol2capacity(ocv);
			if (sprdfgu_data.qmax.s_soc > 500) {
				sprdfgu_data.qmax.s_soc = 0;
				return;
			}
		}
		cur_time = ktime_to_timespec64(ktime_get_boottime());
		sprdfgu_data.qmax.s_time = cur_time.tv_sec;
		sprdfgu_data.qmax.s_clbcnt = sprdfgu_clbcnt_get();
		SPRD_FGU_DEBUG("qmax switch to updating...start soc:%d\n",
				sprdfgu_data.qmax.s_soc);
		*pstate = QMAX_UPDATING;
	} else if (*pstate == QMAX_UPDATING) {
		cur_time = ktime_to_timespec64(ktime_get_boottime());
		if (cur_time.tv_sec - sprdfgu_data.qmax.s_time >
			QMAX_TIMEOUT) {
			SPRD_FGU_DEBUG("qmax time out...\n");
			*pstate = QMAX_IDLE;
			return;
		}
		if ((track.relax_vol > sprdfgu_data.qmax.e_vol) &&
		    (track.relax_cur < sprdfgu_data.qmax.e_cur)) {
			int cur_clbcnt = sprdfgu_clbcnt_get();
			int cc_delta, temp, cnom;

			SPRD_FGU_DEBUG("trigger qmax update done...\n");

			cc_delta = cur_clbcnt - sprdfgu_data.qmax.s_clbcnt;
			temp = DIV_ROUND_CLOSEST(cc_delta,
				(3600 * FGU_CUR_SAMPLE_HZ));
			temp = sprdfgu_adc2cur_ma(temp);
			cnom = (temp * FULL_CAP)
				/ (FULL_CAP - sprdfgu_data.qmax.s_soc);

			SPRD_FGU_DEBUG("%s update:%d,cnom:%d,st_soc:%d\n",
				__func__, temp, cnom, sprdfgu_data.qmax.s_soc);

			if (abs(cnom - sprdfgu_data.cur_cnom)
			    < (50 * sprdfgu_data.cur_cnom) / 100) {
				SPRD_FGU_DEBUG("update cnom...\n");
				sprdfgu_data.cur_cnom = cnom;
				*pstate = QMAX_DONE;
				queue_delayed_work(sprdfgu_data.fgu_wqueue,
						&sprdfgu_data.fgu_qmax_work, 0);
			} else {
				*pstate = QMAX_IDLE;
			}
		}
	} else {
		SPRD_FGU_DEBUG("nothing todo...\n");
	}
}

static void sprdfgu_qmax_works(struct work_struct *work)
{
	struct sprdfgu_drivier_data *pdrv = container_of(work,
						struct sprdfgu_drivier_data,
						fgu_qmax_work.work);
	int *p_state = &pdrv->qmax.state;
	struct file *filep = NULL;
	mm_segment_t old_fs;
	loff_t pos = 0;
	static int retry_cnt = 5;
	int update_cnom, check_cnom;
	unsigned int file_buf[2] = {0};

	if (!sprdfgu_data.is_track) {
		SPRD_FGU_DEBUG("Not support track!\n");
		return;
	}

	SPRD_FGU_DEBUG("sprdfgu_works start\n");

	if (*p_state == QMAX_ERR)
		return;

	old_fs = get_fs();
	set_fs(get_ds());
	mutex_lock(&pdrv->lock);

	filep = filp_open(BAT_FILE_PATH,
			 O_RDWR | O_CREAT, S_IRUGO | S_IWUSR);
	if (IS_ERR(filep)) {
		SPRD_FGU_DEBUG("open file fail\n");
		if ((*p_state == QMAX_INIT) && (retry_cnt > 0)) {
			SPRD_FGU_DEBUG("file not ready retry...\n");
			retry_cnt--;
			queue_delayed_work(sprdfgu_data.fgu_wqueue,
				&sprdfgu_data.fgu_qmax_work, 5 * HZ);
		} else {
			*p_state = QMAX_ERR;
		}
		goto out;
	}

	if (*p_state == QMAX_INIT) {
		*p_state = QMAX_IDLE;
		if (vfs_read(filep, (char *)&file_buf,
		    sizeof(file_buf), &pos) < 0) {
			SPRD_FGU_DEBUG("battery file is empty or read error\n");
			goto out;
		}

		SPRD_FGU_DEBUG("read buff: 0x%x,0x%x\n",
			file_buf[0], file_buf[1]);

		update_cnom = (int)(file_buf[0] ^ CAP_KEY0);
		check_cnom = (int)(file_buf[1] ^ CAP_KEY1);
		if (update_cnom != check_cnom) {
			SPRD_FGU_DEBUG("buf data err %d,%d!\n",
				update_cnom, check_cnom);
			goto out;
		}

		if (abs(pdrv->cur_cnom - update_cnom)
		    < (50 * pdrv->cur_cnom) / 100) {
			SPRD_FGU_DEBUG("update cnom old:%d, new:%d\n",
				pdrv->cur_cnom, update_cnom);
			pdrv->cur_cnom = update_cnom;
		}

		SPRD_FGU_DEBUG("read cap: %d\n", update_cnom);

	} else if (*p_state == QMAX_DONE) {
		*p_state = QMAX_IDLE;
		SPRD_FGU_DEBUG("save battery capacity: %d\n",
			pdrv->cur_cnom);

		file_buf[0] = (unsigned int)(pdrv->cur_cnom) ^ CAP_KEY0;
		file_buf[1] = (unsigned int)(pdrv->cur_cnom) ^ CAP_KEY1;

		vfs_write(filep, (char *)&file_buf, sizeof(file_buf), &pos);
	} else {
		*p_state = QMAX_IDLE;
		SPRD_FGU_DEBUG("qmax nothing to do...\n");
	}

	SPRD_FGU_DEBUG("sprdfgu_works----------end\n");

out:
	if (!IS_ERR(filep))
		filp_close(filep, NULL);
	set_fs(old_fs);
	mutex_unlock(&pdrv->lock);
}

static void sprdfgu_track(void)
{
	int i = 0, rint;
	int retry_flag = 0, cnt = 3;
	int cur_max, vol_max, cur_min, vol_min;
	int delta_vol, delta_cur;
	int cur_reg[BUFF_LEN_2731], vol_reg[BUFF_LEN_2731];
	struct timespec64 cur_time;
	struct sprdfgu_tracking *ptrack = &sprdfgu_data.track;

	if (!sprdfgu_data.is_track)
		return;

	cur_time = ktime_to_timespec64(ktime_get_boottime());

	mutex_lock(&sprdfgu_data.track_lock);

	if ((cur_time.tv_sec - ptrack->query_time) < 5)
		goto out;

	ptrack->c_temp = sprdbat_read_temp();

	if ((ptrack->vol_buff == NULL)
	    || (ptrack->cur_buff == NULL))
		goto out;

	do {
		for (i = 0; i < BUFF_LEN_2731; i++) {
			vol_reg[i] =
				sprdfgu_reg_get(REG_FGU_VOLT_VALUE_BUF0 +
				i * 4);
			cur_reg[i] =
				sprdfgu_reg_get(REG_FGU_CURT_VALUE_BUF0 +
				i * 4);
		}

		retry_flag = 0;

		for (i = 0; i < BUFF_LEN_2731; i++) {
			if ((cur_reg[i] != sprdfgu_reg_get
			    (REG_FGU_CURT_VALUE_BUF0 + i * 4))
			    || (vol_reg[i] != sprdfgu_reg_get
			    (REG_FGU_VOLT_VALUE_BUF0 + i * 4))) {
				SPRD_FGU_DEBUG("Buff updating, try again\n");
				retry_flag = 1;
				break;
			}
		}
	} while (retry_flag && cnt--);

	if (retry_flag) {
		SPRD_FGU_DEBUG("bat buf is error\n");
		goto out;
	}

	ptrack->ave_vol = 0;
	ptrack->ave_cur = 0;

	for (i = 0; i < BUFF_LEN_2731; i++) {
		ptrack->vol_buff[i] = sprdfgu_adc2vol_mv(vol_reg[i]);
		ptrack->cur_buff[i] = sprdfgu_adc2cur_ma(cur_reg[i] -
							CUR_0ma_IDEA_ADC);

		ptrack->ave_vol += ptrack->vol_buff[i];
		ptrack->ave_cur += ptrack->cur_buff[i];

		if (i > 0) {
			if (abs(ptrack->relax_cur) > abs(ptrack->cur_buff[i])) {
				ptrack->relax_cur = ptrack->cur_buff[i];
				ptrack->relax_vol = ptrack->vol_buff[i];
			}

			if (cur_min > ptrack->cur_buff[i]) {
				cur_min = ptrack->cur_buff[i];
				vol_min = ptrack->vol_buff[i];
			}

			if (cur_max < ptrack->cur_buff[i]) {
				cur_max = ptrack->cur_buff[i];
				vol_max = ptrack->vol_buff[i];
			}
		} else {
			ptrack->relax_vol = ptrack->vol_buff[i];
			ptrack->relax_cur = ptrack->cur_buff[i];
			vol_max = vol_min = ptrack->vol_buff[i];
			cur_max = cur_min = ptrack->cur_buff[i];
		}

		SPRD_FGU_DEBUG("buf_id=%d:vol=%d:cur=%d\n",
			i, ptrack->vol_buff[i], ptrack->cur_buff[i]);
	}

	ptrack->ave_vol /= BUFF_LEN_2731;
	ptrack->ave_cur /= BUFF_LEN_2731;

	ptrack->query_time = cur_time.tv_sec;

	SPRD_FGU_DEBUG("vol vol_min=%d:vol_max=%d\n",
					vol_min, vol_max);
	SPRD_FGU_DEBUG("cur cur_min=%d:cur_max=%d\n",
					cur_min, cur_max);
	SPRD_FGU_DEBUG("rel_cur=%d:rel_vol=%d\n",
					ptrack->relax_cur, ptrack->relax_vol);

	delta_vol = vol_max - vol_min;
	delta_cur = cur_max - cur_min;

	SPRD_FGU_DEBUG("delta_vol=%d:delta_cur=%d\n",
					delta_vol, delta_cur);

	if ((delta_cur < 200) || (delta_vol < 20)) {
		SPRD_FGU_DEBUG("vol is no jump,not cal rint\n");
		goto out;
	}

	rint = (delta_vol * 1000) / delta_cur;

	if (rint > 2000) {
		SPRD_FGU_DEBUG("rint is very large=%d\n", rint);
		goto out;
	}

	ptrack->rint = rint;
	ptrack->r_time = cur_time.tv_sec;
	ptrack->r_vol = ptrack->ave_vol;
	ptrack->r_temp = sprdbat_read_temp();
	if (ptrack->r_temp > 150) {
		SPRD_FGU_DEBUG("update normal rint=%d\n", rint);
		sprdfgu_data.cur_rint = rint;
	}

out:
	mutex_unlock(&sprdfgu_data.track_lock);
}

static struct sprdfgu_tracking sprdfgu_get_track_data(void)
{
	struct sprdfgu_tracking data;

	mutex_lock(&sprdfgu_data.track_lock);
	data = sprdfgu_data.track;
	mutex_unlock(&sprdfgu_data.track_lock);
	return data;
}

int sprdfgu_read_soc(void)
{
	int cur_cc, cc_delta, capcity_delta, temp;
	uint32_t cur_ocv;
	int cur;
	union power_supply_propval val;

	sprdfgu_track();
	mutex_lock(&sprdfgu_data.lock);

	sprdfgu_qmax_update_monitor();

	cur_cc = sprdfgu_clbcnt_get();
	cc_delta = cur_cc - sprdfgu_data.init_clbcnt;

	/* 0.1mah */
	temp = DIV_ROUND_CLOSEST(cc_delta,
		(360 * FGU_CUR_SAMPLE_HZ));
	temp = sprdfgu_adc2cur_ma(temp);

	capcity_delta = DIV_ROUND_CLOSEST(temp * 100,
		sprdfgu_data.cur_cnom);

	SPRD_FGU_DEBUG("d cap %d,cnom %d,g %dmAh(0.1mA),init_cc:%d\n",
		capcity_delta, sprdfgu_data.cur_cnom,
		temp, sprdfgu_data.init_clbcnt);

	capcity_delta += sprdfgu_data.init_cap;

	SPRD_FGU_DEBUG("soc %d,init_cap %d,cap:%d\n",
		  (capcity_delta + 9) / 10, sprdfgu_data.init_cap,
		  capcity_delta);

	cur = sprdfgu_read_batcurrent();
	cur_ocv = sprdfgu_read_vbat_ocv();
	if ((cur_ocv >= sprdfgu_data.bat_full_vol)
	    && (abs(cur) <= sprdfgu_data.pdata->chg_end_cur)) {
		SPRD_FGU_DEBUG("cur_ocv %d\n", cur_ocv);
		if (capcity_delta < FULL_CAP) {
			capcity_delta = FULL_CAP;
			sprdfgu_soc_adjust(FULL_CAP);
		}
	}

	if (capcity_delta > FULL_CAP) {
		capcity_delta = FULL_CAP;
		sprdfgu_soc_adjust(FULL_CAP);
	}

	sprdpsy_get_property("battery",
				POWER_SUPPLY_PROP_STATUS,
				&val);
	if (val.intval != POWER_SUPPLY_STATUS_CHARGING) {
		if (capcity_delta <= sprdfgu_data.warning_cap
		    && cur_ocv > sprdfgu_data.pdata->alm_vol) {
			SPRD_FGU_DEBUG("soc low...\n");
			capcity_delta = sprdfgu_data.warning_cap + 10;
			sprdfgu_soc_adjust(capcity_delta);
		} else if (capcity_delta <= 0
		    && cur_ocv > sprdfgu_data.shutdown_vol) {
			SPRD_FGU_DEBUG("soc 0...\n");
			capcity_delta = 10;
			sprdfgu_soc_adjust(capcity_delta);
		} else if (cur_ocv < sprdfgu_data.shutdown_vol) {
			SPRD_FGU_DEBUG("vol 0...\n");
			capcity_delta = 0;
			sprdfgu_soc_adjust(capcity_delta);
		} else if (capcity_delta > sprdfgu_data.warning_cap
			   && cur_ocv < sprdfgu_data.pdata->alm_vol) {
			SPRD_FGU_DEBUG("soc high...\n");
			sprdfgu_soc_adjust(sprdfgu_vol2capacity(cur_ocv));
			capcity_delta = sprdfgu_vol2capacity(cur_ocv);
		}
	}

	if (capcity_delta < 0)
		capcity_delta = 0;

	sprdfgu_record_rawsoc(capcity_delta);

#ifdef SPRDFGU_TEMP_COMP_SOC
	{
		temp = sprdbat_read_temp();
		capcity_delta =
			sprdfgu_temp_comp_soc(capcity_delta, temp / 10);
	}
#endif

	if (sprdfgu_read_vbat_vol() < sprdfgu_data.pdata->soft_vbat_uvlo) {
		SPRD_FGU_DEBUG("trigger soft uvlo vol 0...\n");
		capcity_delta = 0;
		sprdfgu_soc_adjust(capcity_delta);
	}

	mutex_unlock(&sprdfgu_data.lock);
	return capcity_delta;
}

#if 0
static int sprdfgu_avg_current_query(void)
{
	int cur_cc, cc_delta, raw_avg, temp, curr_avg, capcity;
	struct timespec64 cur_timespec;
	__s64 cur_time;
	__s64 time_delta;

	cur_timespec = ktime_to_timespec64(ktime_get_boottime());
	cur_time = cur_timespec.tv_sec;
	cur_cc = sprdfgu_clbcnt_get();
	time_delta = cur_time - start_time;
	cc_delta = cur_cc - poweron_clbcnt;
	temp = (int)time_delta << 1;
	raw_avg = cc_delta / temp;

	SPRD_FGU_DEBUG("time_delta : %lld\n", time_delta);
	SPRD_FGU_DEBUG("cc_delta: %d,raw_avg = %d\n",
		cc_delta, raw_avg);
	curr_avg = sprdfgu_adc2cur_ma(raw_avg);

	temp = (int)time_delta / 3600;
	capcity = temp * curr_avg;
	SPRD_FGU_DEBUG("capcity/1000 capcity = :  %dmah\n", capcity);
	return curr_avg;
}
#endif

static void sprdfgu_debug_works(struct work_struct *work)
{
	SPRD_FGU_DEBUG("dump fgu msg s\n");
#if 0
	if (!sprdfgu_data.pdata->fgu_mode)
		SPRD_FGU_DEBUG("avg current = %d\n",
			sprdfgu_avg_current_query());
	SPRD_FGU_DEBUG("vol:%d,softocv:%d,hardocv:%d\n",
		  sprdfgu_read_vbat_vol(), sprdfgu_read_vbat_ocv(),
		  sprdfgu_ocv_vol_get());
	SPRD_FGU_DEBUG("current%d,cal_type:%d\n",
		sprdfgu_cur_current_get(), fgu_cal.cal_type);

	if (!sprdfgu_data.pdata->fgu_mode)
		SPRD_FGU_DEBUG("soc():%d\n", sprdfgu_read_soc());
#endif
	SPRD_FGU_DEBUG("dump fgu msg e\n");
	schedule_delayed_work(&sprdfgu_debug_work,
		sprdfgu_debug_log_time * HZ);
}

#if 0
static void sprdfgu_cal_battery_impedance(void)
{
	int delta_vol_raw;
	int delta_current_raw;
	int temp;
	int impedance;

	delta_vol_raw = sprdfgu_reg_get(REG_FGU_VOLT_VAL);
	delta_current_raw = sprdfgu_reg_get(REG_FGU_CURT_VAL);

	SPRD_FGU_DEBUG("delta_vol_raw: 0x%x delta_current_raw 0x%x!\n",
	       delta_vol_raw, delta_current_raw);
	SPRD_FGU_DEBUG("cmd_vol_raw: 0x%x cmd_cur_raw 0x%x!\n",
	       cmd_vol_raw, cmd_cur_raw);

	if (0 == cmd_vol_raw || 0 == cmd_cur_raw) {
		SPRD_FGU_DEBUG(KERN_ERR
			"sprdfgu: sprdfgu_cal_battery_impedance warning.....!\n");
		return;
	}

	delta_vol_raw -= cmd_vol_raw;
	delta_current_raw -= cmd_cur_raw;
	delta_vol_raw = ((delta_vol_raw) * 1000) / fgu_cal.vol_1000mv_adc;
	delta_current_raw = sprdfgu_adc2cur_ma(delta_current_raw);
	SPRD_FGU_DEBUG("delta_vol_raw: %d delta_current_raw %d!\n",
	       (delta_vol_raw), (delta_current_raw));
	temp = (delta_vol_raw * 1000) / delta_current_raw;

	impedance = abs(temp);
	if (impedance > 100)
		sprdfgu_data.poweron_rint = impedance;
	else
		SPRD_FGU_DEBUG("impedance warning: %d!\n", impedance);

	SPRD_FGU_DEBUG("poweron_rint: %d!\n",
		sprdfgu_data.poweron_rint);
}
#endif

uint32_t sprdfgu_read_capacity(void)
{
	int32_t voltage;
	int cap;

	if (sprdfgu_data.pdata->fgu_mode) {
		voltage = sprdfgu_read_vbat_ocv();
		cap = sprdfgu_vol2capacity(voltage);
	} else {
		cap = sprdfgu_read_soc();
	}
	if (cap > FULL_CAP)
		cap = FULL_CAP;
	else if (cap < 0)
		cap = 0;

	SPRD_FGU_DEBUG("cap read ui= %d,raw_soc = %d\n",
		  sprdfgu_load_uicap(), sprdfgu_load_rawsoc());

	return (cap + 9) / 10;
}

static int sprdfgu_only_vol_get_average_cap(int cap)
{
	static int pointer;
	int i, sum = 0;

	if (pointer >= SPRDFGU_VOL_ONLY_CAP_BUFF_CNT)
		pointer = 0;
	sprdfgu_vol_only_cap_buff[pointer++] = cap;

	for (i = 0; i < SPRDFGU_VOL_ONLY_CAP_BUFF_CNT; i++)
		sum += sprdfgu_vol_only_cap_buff[i];

	return sum / SPRDFGU_VOL_ONLY_CAP_BUFF_CNT;
}

unsigned int sprdfgu_only_vol_read_capacity(int val)
{
	unsigned int voltage;
	int cap;

	voltage = sprdfgu_read_vbat_ocv();
	if (val)
		cap = sprdfgu_charge_vol2capacity(voltage);
	else
		cap = sprdfgu_discharge_vol2capacity(voltage);

	cap = sprdfgu_only_vol_get_average_cap(cap);

	if (cap > FULL_CAP)
		cap = FULL_CAP;
	else if (cap < 0)
		cap = 0;

	SPRD_FGU_DEBUG("cap read ui= %d,raw_soc = %d\n",
		       sprdfgu_load_uicap(), sprdfgu_load_rawsoc());
	/*
	 * when the ones place more than 1, the tens place will add 1,
	 * with this, to take integer.
	 */
	return roundup(cap, 10) / 10;
}

uint32_t sprdfgu_poweron_capacity(void)
{
	return sprdfgu_data.poweron_cap;
}

void sprdfgu_adp_status_set(int plugin)
{
	sprdfgu_data.adp_status = plugin;
	if (plugin) {
		uint32_t adc;

		adc = sprdfgu_vol2adc_mv(sprdfgu_data.pdata->alm_vol);
		fgu_adi_write(REG_FGU_INT_CLR, BIT_VOLT_LOW_INT,
			BIT_VOLT_LOW_INT);
		fgu_adi_write(REG_FGU_LOW_OVER, adc & 0xFFFF, ~0U);
		fgu_adi_write(REG_FGU_INT_EN, 0, BIT_CLBCNT_DELTA_INT);
	}
}

static void sprdfgu_irq_works(struct work_struct *work)
{
	uint32_t cur_ocv;
	uint32_t adc;
	int cur_soc;

	wake_lock_timeout(&(sprdfgu_data.low_power_lock), 2 * HZ);

	SPRD_FGU_DEBUG("%s......0x%x.cur vol = %d\n",
		__func__, sprdfgu_data.int_status,
		sprdfgu_read_vbat_vol());

	if (sprdfgu_data.int_status & BIT_VOLT_HIGH_INT)
		power_supply_changed(sprdfgu_data.sprdfgu);

	if (sprdfgu_data.int_status & BIT_VOLT_LOW_INT) {
		cur_soc = sprdfgu_read_soc();
		mutex_lock(&sprdfgu_data.lock);

		cur_ocv = sprdfgu_read_vbat_ocv();
		if (cur_ocv <= sprdfgu_data.shutdown_vol) {
			SPRD_FGU_DEBUG("shutdown_vol .\n");
			sprdfgu_soc_adjust(0);
		} else if (cur_ocv <= sprdfgu_data.pdata->alm_vol) {
			SPRD_FGU_DEBUG("alm_vol %d.\n", cur_soc);
			if (cur_soc > sprdfgu_data.warning_cap) {
				sprdfgu_soc_adjust(sprdfgu_data.warning_cap);
			} else if (cur_soc <= 0) {
				sprdfgu_soc_adjust(sprdfgu_vol2capacity
						   (cur_ocv));
			}
			if (!sprdfgu_data.adp_status) {
				adc = sprdfgu_vol2adc_mv
					(sprdfgu_data.shutdown_vol);
				fgu_adi_write(REG_FGU_LOW_OVER, adc & 0xFFFF,
					      ~0);
			}
		} else {
			SPRD_FGU_DEBUG("need add\n");
		}
		mutex_unlock(&sprdfgu_data.lock);
	}
}

static irqreturn_t _sprdfgu_interrupt(int irq, void *dev_id)
{
	sprdfgu_data.int_status = fgu_adi_read(REG_FGU_INT_STS);
	fgu_adi_write(REG_FGU_INT_CLR, sprdfgu_data.int_status,
		sprdfgu_data.int_status);
	SPRD_FGU_DEBUG("%s raw..0x%x,int_status0x%x\n",
		__func__, fgu_adi_read(REG_FGU_INT_RAW),
		sprdfgu_data.int_status);

	schedule_delayed_work(&sprdfgu_data.fgu_irq_work, 0);
	return IRQ_HANDLED;
}

static int sprdfgu_int_init(void)
{
	uint32_t adc;
	int delta_cc = sprdfgu_clbcnt_init(1);
	int ret = -ENODEV;

	INIT_DELAYED_WORK(&sprdfgu_data.fgu_irq_work, sprdfgu_irq_works);

	fgu_adi_write(REG_FGU_INT_CLR, 0xFFFF, 0xFFFF);

	adc = sprdfgu_vol2adc_mv(sprdfgu_data.pdata->chg_bat_safety_vol);
	fgu_adi_write(REG_FGU_HIGH_OVER, adc & 0xFFFF, ~0U);
	adc = sprdfgu_vol2adc_mv(sprdfgu_data.pdata->alm_vol);
	fgu_adi_write(REG_FGU_LOW_OVER, adc & 0xFFFF, ~0U);

	fgu_adi_write(REG_FGU_CLBCNT_DELTL, delta_cc & 0xFFFF, ~0U);
	fgu_adi_write(REG_FGU_CLBCNT_DELTH, (delta_cc >> 16) & 0xFFFF, ~0U);

	ret = devm_request_threaded_irq(sprdfgu_data.dev,
			sprdfgu_data.fgu_pdata->fgu_irq,
			NULL, _sprdfgu_interrupt,
			IRQF_NO_SUSPEND | IRQF_ONESHOT,
			"sprdfgu", NULL);

	if (ret) {
		SPRD_FGU_DEBUG("request sprdfgu irq %d failed\n",
		       sprdfgu_data.fgu_pdata->fgu_irq);
	}
	/*
	 * TODO: switch polling to interrupt again need open this code.
	 * fgu_adi_write(REG_FGU_INT_EN, BIT_VOLT_HIGH_INT,
		* BIT_VOLT_HIGH_INT);
	 */
	return ret;
}

void sprdfgu_pm_op(int is_suspend)
{
	if (is_suspend) {
		if (!sprdfgu_data.adp_status) {
			fgu_adi_write(REG_FGU_INT_EN,
				BIT_VOLT_LOW_INT,
				BIT_VOLT_LOW_INT);
			if (sprdfgu_read_vbat_ocv() <
			    sprdfgu_data.pdata->alm_vol) {
				fgu_adi_write(REG_FGU_INT_CLR,
					    BIT_CLBCNT_DELTA_INT,
					    BIT_CLBCNT_DELTA_INT);
				fgu_adi_write(REG_FGU_INT_EN,
					    BIT_CLBCNT_DELTA_INT,
					    BIT_CLBCNT_DELTA_INT);
			}
		}
	} else {
		fgu_adi_write(REG_FGU_INT_EN, 0,
			    BIT_VOLT_LOW_INT | BIT_CLBCNT_DELTA_INT);
	}
}

/* this functon only for fgu init when system poweron */
static int sprdfgu_find_firstpoweron_cap(void)
{
	int workaround_2721 = 0;
	unsigned int val = 0;
	int cap, i, poci_raw, poci_curr;
	unsigned int pocv_raw, p_ocv;

	if (sprdfgu_data.fgu_pdata->fgu_type == FGU_2721) {
		regmap_read(reg_map, ANA_REG_GLB_CHIP_ID_LOW, &val);
		/*if the value ==0xa000,it is AA chip */
		if (val == 0xA000)
			workaround_2721 = 1;
	}
	SPRD_FGU_DEBUG("val:%d,fgu_type:%d\n", val,
		       sprdfgu_data.fgu_pdata->fgu_type);
	if (((gpio_get_value(
		sprdfgu_data.pdata->gpio_vchg_detect)) &&
		(!sprdfgu_data.fgu_pdata->ocv_type)) || workaround_2721) {
		int current_raw = sprdfgu_reg_get(REG_FGU_CURT_VAL);
		uint32_t soft_ocv =
				sprdfgu_read_vbat_vol() -
				(sprdfgu_adc2cur_ma
				(current_raw - CUR_0ma_IDEA_ADC +
				fgu_cal.cur_offset) *
				sprdfgu_data.poweron_rint) / 1000;
		if (sprdfgu_data.pdata->only_vol_mode) {
			cap = sprdfgu_discharge_vol2capacity(soft_ocv);
			for (i = 0; i < SPRDFGU_VOL_ONLY_CAP_BUFF_CNT; i++)
				sprdfgu_vol_only_cap_buff[i] = cap;
		} else {
			cap = sprdfgu_vol2capacity(soft_ocv);
		}
		SPRD_FGU_DEBUG("poweron soft_ocv:%d, init_cap:%d\n",
				soft_ocv, cap);
	} else {
		pocv_raw = fgu_adi_read(REG_FGU_POCV_VAL);
		if (sprdfgu_data.pdata->only_vol_mode) {
			/* because only vol mode remove 20mOhm resistance,
			 * so poci_curr = 0
			 */
			p_ocv = sprdfgu_adc2vol_mv(pocv_raw);
			cap = sprdfgu_discharge_vol2capacity(p_ocv);
			for (i = 0; i < SPRDFGU_VOL_ONLY_CAP_BUFF_CNT; i++)
				sprdfgu_vol_only_cap_buff[i] = cap;
		} else {
			poci_raw = sprdfgu_reg_get(REG_FGU_CLBCNT_QMAXL) << 1;
			poci_curr = sprdfgu_adc2cur_ma(poci_raw -
				CUR_0ma_IDEA_ADC + fgu_cal.cur_offset);
			SPRD_FGU_DEBUG("poci_raw:0x%x,poci_cur:%d\n",
				       poci_raw, poci_curr);
			p_ocv = sprdfgu_adc2vol_mv(pocv_raw) -
				(poci_curr * sprdfgu_data.poweron_rint) / 1000;
			cap = sprdfgu_vol2capacity(p_ocv);
		}
		SPRD_FGU_DEBUG("poweron pocv:%d, init_cap:%d\n", p_ocv, cap);
	}

	return cap;
}

extern int in_calibration(void);
int __weak in_calibration(void){return 0; }
static void sprdfgu_hw_init(void)
{
	u32 cur_vol_raw, ocv_raw;
	int current_raw;
	u32 pocv_raw;
	struct timespec64 cur_time;

	SPRD_FGU_DEBUG("FGU_Init\n");
	glb_adi_write(ANA_REG_GLB_ARM_MODULE_EN,
		BIT_ANA_FGU_EN, BIT_ANA_FGU_EN);
	glb_adi_write(ANA_REG_GLB_RTC_CLK_EN,
		BIT_RTC_FGU_EN, BIT_RTC_FGU_EN);
	fgu_adi_write(REG_FGU_RELAX_CURT_THRE,
		BITS_RELAX_CUR_THRE(sprdfgu_cur2adc_ma(sprdfgu_data.
		pdata->relax_current)), BITS_RELAX_CUR_THRE(~0U));
	udelay(130);

	pocv_raw = fgu_adi_read(REG_FGU_POCV_VAL);
	cur_vol_raw = sprdfgu_reg_get(REG_FGU_VOLT_VAL);
	ocv_raw = sprdfgu_reg_get(REG_FGU_OCV_VAL);
	current_raw = sprdfgu_reg_get(REG_FGU_CURT_VAL);

	cur_time = ktime_to_timespec64(ktime_get_boottime());
	start_time = cur_time.tv_sec;
	SPRD_FGU_DEBUG("REG_FGU_USER_AREA_STATUS = 0x%x\n",
		  fgu_adi_read(REG_FGU_USER_AREA_STATUS));

	SPRD_FGU_DEBUG("REG_FGU_USER_AREA_STATUS1-- = 0x%x\n",
		  fgu_adi_read(REG_FGU_USER_AREA_STATUS1));

	if ((sprdfgu_poweron_type_read() == FIRST_POWERTON) ||
		(sprdfgu_load_uicap() == 0xFFF)) {
		SPRD_FGU_DEBUG("FIRST_POWERTON- = 0x%x\n",
			  sprdfgu_poweron_type_read());

		sprdfgu_data.init_cap = sprdfgu_find_firstpoweron_cap();
		sprdfgu_data.poweron_cap = sprdfgu_data.init_cap/10;
		sprdfgu_record_cap(sprdfgu_data.poweron_cap);
		sprdfgu_record_rawsoc(sprdfgu_data.init_cap);
	} else {
		sprdfgu_data.init_cap = sprdfgu_load_rawsoc();
		sprdfgu_data.poweron_cap = sprdfgu_load_uicap();
		if ((sprdfgu_data.poweron_cap > 100)
			|| (sprdfgu_data.init_cap > FULL_CAP)) {
			SPRD_FGU_DEBUG("REG ERR-init_cap= %d,p_cap = %d\n",
			  sprdfgu_data.init_cap, sprdfgu_data.poweron_cap);

			sprdfgu_data.init_cap = sprdfgu_find_firstpoweron_cap();
			sprdfgu_data.poweron_cap = sprdfgu_data.init_cap/10;
			sprdfgu_record_cap(sprdfgu_data.poweron_cap);
			sprdfgu_record_rawsoc(sprdfgu_data.init_cap);
		}
		SPRD_FGU_DEBUG("NORMAIL_POWERTON-init_cap= %d,p_cap = %d\n",
			  sprdfgu_data.init_cap, sprdfgu_data.poweron_cap);
	}
	sprdfgu_poweron_type_write(NORMAIL_POWERTON);

	sprdfgu_data.init_clbcnt = poweron_clbcnt =
	    sprdfgu_clbcnt_init(sprdfgu_data.init_cap);
	sprdfgu_clbcnt_set(poweron_clbcnt);

	/*set current cal data*/
	if (!in_calibration())
		fgu_adi_write(REG_FGU_CURT_OFFSET, fgu_cal.cur_offset, ~0U);

	SPRD_FGU_DEBUG("pocv_raw = 0x%x,pocv_voltage = %d\n",
		pocv_raw, sprdfgu_adc2vol_mv(pocv_raw));
	SPRD_FGU_DEBUG("cur vol raw_data =  0x%x,cur vol = %d\n",
		  cur_vol_raw, sprdfgu_adc2vol_mv(cur_vol_raw));
	SPRD_FGU_DEBUG("ocv_raw: 0x%x,ocv voltage = %d\n",
		ocv_raw, sprdfgu_adc2vol_mv(ocv_raw));
	SPRD_FGU_DEBUG("current_raw: 0x%x,current = %d\n",
		current_raw, sprdfgu_adc2cur_ma(current_raw -
		CUR_0ma_IDEA_ADC));
	SPRD_FGU_DEBUG("poweron_clbcnt: 0x%x,cur_cc0x%x\n",
		poweron_clbcnt, sprdfgu_clbcnt_get());
	SPRD_FGU_DEBUG("poweron_rint = %d\n",
		sprdfgu_data.poweron_rint);

	SPRD_FGU_DEBUG("REG_FGU_STATUS = 0x%x\n",
		sprdfgu_reg_get(REG_FGU_STATUS));
	if (((!sprdfgu_data.fgu_pdata->ocv_type) &&
		sprdfgu_data.pdata->gpio_vchg_detect) &&
		(request_vchg_gpio == 0))
		gpio_free(sprdfgu_data.pdata->gpio_vchg_detect);
}

static inline void sprdfgu_report_show(int *cc, int *tm, int *avg_cur)
{
	struct timespec64 cur_time = ktime_to_timespec64(ktime_get_boottime());
	int cur_clbcnt = sprdfgu_clbcnt_get();
	int cc_delta, temp;

	*tm = cur_time.tv_sec - start_time;

	cc_delta = cur_clbcnt - poweron_clbcnt;
	temp = DIV_ROUND_CLOSEST(cc_delta,
		(3600 * FGU_CUR_SAMPLE_HZ));
	*cc = sprdfgu_adc2cur_ma(temp);

	temp = *tm << 1;
	if (temp) {
		temp = cc_delta / temp;
		*avg_cur = sprdfgu_adc2cur_ma(temp);
	} else
		*avg_cur = sprdfgu_read_batcurrent();
}

static ssize_t sprdfgu_store_attribute(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count);
static ssize_t sprdfgu_show_attribute(struct device *dev,
				      struct device_attribute *attr, char *buf);

#define SPRDFGU_ATTR(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IRUGO | S_IWUSR | S_IWGRP, },  \
	.show = sprdfgu_show_attribute,                  \
	.store = sprdfgu_store_attribute,                              \
}
#define SPRDFGU_ATTR_RO(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IRUGO, },  \
	.show = sprdfgu_show_attribute,                  \
}
#define SPRDFGU_ATTR_WO(_name)                         \
{                                       \
	.attr = { .name = #_name, .mode = S_IWUSR | S_IWGRP, },  \
	.store = sprdfgu_store_attribute,                              \
}

static struct device_attribute sprdfgu_attribute[] = {
	SPRDFGU_ATTR_RO(fgu_vol_adc),
	SPRDFGU_ATTR_RO(fgu_current_adc),
	SPRDFGU_ATTR_RO(fgu_vol),
	SPRDFGU_ATTR_RO(fgu_current),
	SPRDFGU_ATTR_WO(fgu_log_time),
	SPRDFGU_ATTR_RO(fgu_cal_from_type),
	SPRDFGU_ATTR(cc_test_cmd),
	SPRDFGU_ATTR_RO(cc_test_result),
	SPRDFGU_ATTR_WO(qmax_force_start),
	SPRDFGU_ATTR(qmax_force_s_soc),
	SPRDFGU_ATTR_RO(qmax_state),
	SPRDFGU_ATTR(qmax_s_vol),
	SPRDFGU_ATTR(qmax_s_cur),
	SPRDFGU_ATTR(qmax_e_vol),
	SPRDFGU_ATTR(qmax_e_cur),
	SPRDFGU_ATTR_RO(cnom),
	SPRDFGU_ATTR(saved_cnom),
	SPRDFGU_ATTR(full_vol),
	SPRDFGU_ATTR(alarm_vol),
	SPRDFGU_ATTR(shutdown_vol),
	SPRDFGU_ATTR(soft_uvlo),
	SPRDFGU_ATTR(cur_rint),
	SPRDFGU_ATTR(rsense),
	SPRDFGU_ATTR_RO(fgu_report),
};

static struct attribute *sprd_fgu_attrs[] = {
	&sprdfgu_attribute[0].attr,
	&sprdfgu_attribute[1].attr,
	&sprdfgu_attribute[2].attr,
	&sprdfgu_attribute[3].attr,
	&sprdfgu_attribute[4].attr,
	&sprdfgu_attribute[5].attr,
	&sprdfgu_attribute[6].attr,
	&sprdfgu_attribute[7].attr,
	&sprdfgu_attribute[8].attr,
	&sprdfgu_attribute[9].attr,
	&sprdfgu_attribute[10].attr,
	&sprdfgu_attribute[11].attr,
	&sprdfgu_attribute[12].attr,
	&sprdfgu_attribute[13].attr,
	&sprdfgu_attribute[14].attr,
	&sprdfgu_attribute[15].attr,
	&sprdfgu_attribute[16].attr,
	&sprdfgu_attribute[17].attr,
	&sprdfgu_attribute[18].attr,
	&sprdfgu_attribute[19].attr,
	&sprdfgu_attribute[20].attr,
	&sprdfgu_attribute[21].attr,
	&sprdfgu_attribute[22].attr,
	&sprdfgu_attribute[23].attr,
	NULL
};

static struct attribute_group sprd_fgu_group = {
	.name = NULL,
	.attrs = sprd_fgu_attrs,
};
enum SPRDFGU_ATTRIBUTE {
	FGU_VOL_ADC = 0,
	FGU_CURRENT_ADC,
	FGU_VOL,
	FGU_CURRENT,
	FGU_LOG_TIME,
	FGU_CAL_FROM_TYPE,
	CC_TEST_CMD,
	CC_TEST_RESULT,
	QMAX_FORCE_START,
	QMAX_FORCE_S_SOC,
	QMAX_STATE,
	QMAX_S_VOL,
	QMAX_S_CUR,
	QMAX_E_VOL,
	QMAX_E_CUR,
	CNOM,
	SAVED_CNOM,
	FULL_VOL,
	ALARM_VOL,
	SHUTDOWN_VOL,
	SOFT_UVLO,
	CUR_RINT,
	RSENSE,
	FGU_REPORT,
};

static ssize_t sprdfgu_store_attribute(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	unsigned long set_value;
	const ptrdiff_t off = attr - sprdfgu_attribute;

	if (kstrtoul(buf, 0, &set_value))
		return -EINVAL;
	pr_info("sprdfgu_store_attribute %lu\n", set_value);

	switch (off) {
	case FGU_LOG_TIME:
		sprdfgu_debug_log_time = set_value;
		break;
	case CC_TEST_CMD:
		sprdfgu_clbcnt_test(set_value);
		break;
	case QMAX_FORCE_START:
		mutex_lock(&sprdfgu_data.lock);
		if ((sprdfgu_data.qmax.state != QMAX_ERR) ||
		    (sprdfgu_data.qmax.state != QMAX_INIT)) {
			sprdfgu_data.qmax.state = QMAX_IDLE;
			sprdfgu_data.qmax.force_s_flag = set_value;
		}
		mutex_unlock(&sprdfgu_data.lock);
		break;
	case QMAX_FORCE_S_SOC:
		if (set_value < FULL_CAP)
			sprdfgu_data.qmax.force_s_soc = set_value;
		else
			sprdfgu_data.qmax.force_s_soc = 0;
		break;
	case QMAX_S_VOL:
		sprdfgu_data.qmax.s_vol = set_value;
		break;
	case QMAX_S_CUR:
		sprdfgu_data.qmax.s_cur = set_value;
		break;
	case QMAX_E_VOL:
		sprdfgu_data.qmax.e_vol = set_value;
		break;
	case QMAX_E_CUR:
		sprdfgu_data.qmax.e_cur = set_value;
		break;

	case SAVED_CNOM:
		SPRD_FGU_DEBUG("saved cnom...\n");
		sprdfgu_data.cur_cnom = set_value;
		sprdfgu_data.qmax.state = QMAX_DONE;
		queue_delayed_work(sprdfgu_data.fgu_wqueue,
				&sprdfgu_data.fgu_qmax_work, 0);
		break;
	case FULL_VOL:
		SPRD_FGU_DEBUG("set bat_full_vol:%lu\n", set_value);
		sprdfgu_data.bat_full_vol = set_value;
		break;
	case ALARM_VOL:
		SPRD_FGU_DEBUG("set alm_vol:%lu\n", set_value);
		sprdfgu_data.pdata->alm_vol = set_value;
		break;
	case SHUTDOWN_VOL:
		SPRD_FGU_DEBUG("set shutdown_vol:%lu\n", set_value);
		sprdfgu_data.shutdown_vol = set_value;
		break;
	case SOFT_UVLO:
		SPRD_FGU_DEBUG("set soft uvlo:%lu\n", set_value);
		sprdfgu_data.pdata->soft_vbat_uvlo = set_value;
		break;
	case CUR_RINT:
		SPRD_FGU_DEBUG("set rint:%lu\n", set_value);
		sprdfgu_data.cur_rint = set_value;
		break;
	case RSENSE:
		SPRD_FGU_DEBUG("set rsense:%lu\n", set_value);
		sprdfgu_data.pdata->rsense_real = set_value;
		sprdfgu_cal_init();
		break;
	default:
		count = -EINVAL;
		break;
	}
	return count;
}

static ssize_t sprdfgu_show_attribute(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - sprdfgu_attribute;

	switch (off) {
	case FGU_VOL_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       fgu_adi_read(REG_FGU_VOLT_VAL));
		break;
	case FGU_CURRENT_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       fgu_adi_read(REG_FGU_CURT_VAL));
		break;
	case FGU_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       sprdfgu_read_vbat_vol());
		break;
	case FGU_CURRENT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       sprdfgu_read_batcurrent());
		break;
	case FGU_CAL_FROM_TYPE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       fgu_cal.cal_type);
		break;
	case CC_TEST_CMD:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				cc_test.ongoing);
		break;
	case CC_TEST_RESULT:
		i += scnprintf(buf + i, PAGE_SIZE - i,
				"GAUGE:%dmAh,TIME:%ds,AVG_CUR:%dmA\n",
				cc_test.cc_keep, cc_test.time_keep,
				cc_test.avg_cur);
		break;
	case QMAX_FORCE_S_SOC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				sprdfgu_data.qmax.force_s_soc);
		break;
	case QMAX_STATE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				sprdfgu_data.qmax.state);
		break;
	case QMAX_S_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				sprdfgu_data.qmax.s_vol);
		break;
	case QMAX_S_CUR:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				sprdfgu_data.qmax.s_cur);
		break;
	case QMAX_E_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				sprdfgu_data.qmax.e_vol);
		break;
	case QMAX_E_CUR:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				sprdfgu_data.qmax.e_cur);
		break;
	case CNOM:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				sprdfgu_data.cur_cnom);
		break;
	case SAVED_CNOM:
		i += scnprintf(buf + i, PAGE_SIZE - i, "load done\n");
		SPRD_FGU_DEBUG("saved cnom.read..\n");
		sprdfgu_data.qmax.state = QMAX_INIT;
		queue_delayed_work(sprdfgu_data.fgu_wqueue,
				&sprdfgu_data.fgu_qmax_work, 0);
		break;
	case FULL_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				sprdfgu_data.bat_full_vol);
		break;
	case ALARM_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				sprdfgu_data.pdata->alm_vol);
		break;
	case SHUTDOWN_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				sprdfgu_data.shutdown_vol);
		break;
	case SOFT_UVLO:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				sprdfgu_data.pdata->soft_vbat_uvlo);
		break;
	case CUR_RINT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				sprdfgu_data.cur_rint);
		break;
	case RSENSE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				sprdfgu_data.pdata->rsense_real);
		break;
	case FGU_REPORT:
		{
			int cc, tm, cur;

			sprdfgu_report_show(&cc, &tm, &cur);
			i += scnprintf(buf + i, PAGE_SIZE - i,
					"GAUGE:%dmAh,TIME:%ds,AVG_CUR:%dmA\n",
					cc, tm,
					cur);
		}
		break;
	default:
		i = -EINVAL;
		break;
	}

	return i;
}

static int sprdfgu_power_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	return -EINVAL;
}

int sprdfgu_init(struct sprd_battery_platform_data *pdata)
{
	int ret = 0;
	int temp;

	sprdfgu_data.pdata = pdata;
	if (sprdfgu_data.pdata->only_vol_mode) {
		unsigned int alm_vol =
			sprdfgu_data.pdata->alm_vol;

		sprdfgu_data.warning_cap =
			sprdfgu_discharge_vol2capacity(alm_vol);
	} else {
		sprdfgu_data.warning_cap =
			sprdfgu_vol2capacity(sprdfgu_data.pdata->alm_vol);
	}
	temp = sprdfgu_data.pdata->ocv_tab_size;
	sprdfgu_data.shutdown_vol = sprdfgu_data.pdata->ocv_tab[temp - 1].x;

	sprdfgu_data.bat_full_vol = sprdfgu_data.pdata->ocv_tab[0].x;
	sprdfgu_data.cur_rint = sprdfgu_data.pdata->rint;
	sprdfgu_data.cur_cnom = sprdfgu_data.pdata->cnom;
	sprdfgu_data.poweron_rint = sprdfgu_data.pdata->rint;

	if ((!sprdfgu_data.fgu_pdata->ocv_type) &&
		sprdfgu_data.pdata->gpio_vchg_detect) {
		ret = gpio_request(sprdfgu_data.pdata->gpio_vchg_detect,
			"chg_vchg_detect");
		if (ret) {
			SPRD_FGU_DEBUG("Already request vchg gpio:%d\n",
				sprdfgu_data.pdata->gpio_vchg_detect);
			request_vchg_gpio = 1;
		}
		gpio_direction_input(sprdfgu_data.pdata->gpio_vchg_detect);
	}

	if (fgu_cal.cal_type == SPRDBAT_FGUADC_CAL_NO)
		sprdfgu_cal_from_chip();

	sprdfgu_cal_init();
	sprdfgu_hw_init();
	sprdfgu_int_init();

/* For debug, usually do not use it */
#if 0
	schedule_delayed_work(&sprdfgu_debug_work,
		sprdfgu_debug_log_time * HZ);
#endif

	sprdfgu_data.qmax.s_vol = QMAX_START_OCV_LIMIT;
	sprdfgu_data.qmax.s_cur = QMAX_START_CUR_LIMIT;
	sprdfgu_data.qmax.e_vol = sprdfgu_data.pdata->chg_end_vol_l - 5;
	sprdfgu_data.qmax.e_cur = sprdfgu_data.pdata->chg_end_cur + 5;

	if (sprdfgu_data.is_track)
		sprdfgu_data.is_track = sprdfgu_data.pdata->cnom_track_support;

	if (sprdbat_is_support_batdet()) {
		sprdfgu_data.qmax.state = QMAX_INIT;
		queue_delayed_work(sprdfgu_data.fgu_wqueue,
			   &sprdfgu_data.fgu_qmax_work, 5 * HZ);
	} else {
		sprdfgu_data.qmax.state = QMAX_ERR;
	}

	SPRD_FGU_DEBUG("sprdfgu_init end\n");
	return ret;
}

int sprdfgu_reset(void)
{
	struct timespec64 cur_time;

	cur_time = ktime_to_timespec64(ktime_get_boottime());
	start_time = cur_time.tv_sec;
	if (sprdfgu_data.pdata->only_vol_mode)
		sprdfgu_data.init_cap =
			sprdfgu_discharge_vol2capacity(sprdfgu_read_vbat_ocv());
	else
		sprdfgu_data.init_cap =
			sprdfgu_vol2capacity(sprdfgu_read_vbat_ocv());
	sprdfgu_data.poweron_cap = sprdfgu_data.init_cap/10;
	sprdfgu_record_cap(sprdfgu_data.poweron_cap);
	sprdfgu_record_rawsoc(sprdfgu_data.init_cap);
	sprdfgu_data.init_clbcnt = poweron_clbcnt =
	    sprdfgu_clbcnt_init(sprdfgu_data.init_cap);
	sprdfgu_clbcnt_set(poweron_clbcnt);

	if ((sprdfgu_data.qmax.state != QMAX_ERR) ||
	    (sprdfgu_data.qmax.state != QMAX_INIT))
		sprdfgu_data.qmax.state = QMAX_IDLE;

	return 0;
}

int sprdfgu_force_set_soc(unsigned int cap)
{
	SPRD_FGU_DEBUG("set soc to:%d\n", cap);
	mutex_lock(&sprdfgu_data.lock);
	sprdfgu_soc_adjust(cap);
	sprdfgu_record_rawsoc(cap);
	mutex_unlock(&sprdfgu_data.lock);
	return 0;
}

void sprdfgu_record_cap(u32 cap)
{
	sprdfgu_rtc_reg_write(cap);
}

static void sprdfgu_record_rawsoc(u32 cap)
{
	sprdfgu_rtc_reg1_write(cap);
}

static inline uint32_t sprdfgu_load_uicap(void)
{
	return sprdfgu_rtc_reg_read();
}

static inline uint32_t sprdfgu_load_rawsoc(void)
{
	return sprdfgu_rtc_reg1_read();
}

static int sprdfgu_2723_probe(struct platform_device *pdev)
{
	int ret = 0, irq = 0;
	u32 value = 0;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id;
	struct power_supply *ret_ptr = NULL;
	struct power_supply_desc *sprdfgu_desc = NULL;

	reg_map = dev_get_regmap(pdev->dev.parent, NULL);
	if (!reg_map)
		dev_err(&pdev->dev, "%s :NULL regmap for fgu 2723\n",
			__func__);
	sprdfgu_data.fgu_pdata = devm_kzalloc(&pdev->dev,
		sizeof(struct sprdfgu_platform_data),
		GFP_KERNEL);
	of_id = of_match_node(sprdfgu_2723_of_match,
		pdev->dev.of_node);
	if (!of_id) {
		pr_err("get 27xx fgu of device id failed!\n");
		return -ENODEV;
	}
	sprdfgu_data.fgu_pdata->fgu_type = (enum fgu_type)of_id->data;
	SPRD_FGU_DEBUG("fgu_type =%d\n",
		sprdfgu_data.fgu_pdata->fgu_type);

	ret = of_property_read_u32(np, "ocv-type",
		&sprdfgu_data.fgu_pdata->ocv_type);
	if (ret)
		dev_err(&pdev->dev, "%s :no ocv-type\n",
			__func__);
	irq = platform_get_irq(pdev, 0);
	if (unlikely(irq <= 0))
		dev_err(&pdev->dev, "no irq resource specified\n");
	sprdfgu_data.fgu_pdata->fgu_irq = irq;
	ret = of_property_read_u32(np, "reg", &value);
	if (ret)
		dev_err(&pdev->dev, "%s :no reg of property for pmic fgu\n",
			__func__);
	sprdfgu_data.fgu_pdata->base_addr = (unsigned long)value;

	sprdfgu_desc = devm_kzalloc(&pdev->dev,
			sizeof(struct power_supply_desc), GFP_KERNEL);
	if (sprdfgu_desc == NULL)
		return -ENOMEM;
	sprdfgu_desc->name = "sprdfgu";
	sprdfgu_desc->get_property = sprdfgu_power_get_property;
	ret_ptr = power_supply_register(&pdev->dev, sprdfgu_desc, NULL);
	if (IS_ERR(ret_ptr))
		pr_err("register power supply error!\n");
	else
		sprdfgu_data.sprdfgu = ret_ptr;
	sprdfgu_data.dev = &pdev->dev;
	platform_set_drvdata(pdev, &sprdfgu_data);
	ret = sysfs_create_group(&sprdfgu_data.sprdfgu->dev.kobj,
			&sprd_fgu_group);
	if (ret)
		pr_err("failed to create sprd_fgu sysfs device attributes\n");
	INIT_DELAYED_WORK(&sprdfgu_debug_work, sprdfgu_debug_works);
	mutex_init(&sprdfgu_data.lock);
	mutex_init(&sprdfgu_data.track_lock);
	wake_lock_init(&(sprdfgu_data.low_power_lock), WAKE_LOCK_SUSPEND,
		       "sprdfgu_powerlow_lock");

	sprdfgu_data.is_track = 1;

	INIT_DELAYED_WORK(&sprdfgu_data.fgu_qmax_work,
			  sprdfgu_qmax_works);
	sprdfgu_data.fgu_wqueue =
		create_workqueue("sprdfgu_monitor");
	if (!sprdfgu_data.fgu_wqueue)
		sprdfgu_data.is_track = 0;

	sprdfgu_data.track.vol_buff = devm_kzalloc(&pdev->dev,
		sizeof(int) * CUR_VOL_BUFF_LEN,
		GFP_KERNEL);
	if (!sprdfgu_data.track.vol_buff)
		sprdfgu_data.is_track = 0;

	sprdfgu_data.track.cur_buff = devm_kzalloc(&pdev->dev,
		sizeof(int) * CUR_VOL_BUFF_LEN,
		GFP_KERNEL);
	if (!sprdfgu_data.track.cur_buff)
		sprdfgu_data.is_track = 0;

	SPRD_FGU_DEBUG("sprdfgu_2723_probe ok\n");
	return ret;
}

static int sprdfgu_2723_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&sprdfgu_data.sprdfgu->dev.kobj,
		&sprd_fgu_group);
	if (sprdfgu_data.fgu_wqueue)
		destroy_workqueue(sprdfgu_data.fgu_wqueue);

	return 0;
}

static struct platform_driver sprdfgu_2723_driver = {
	.driver = {
		   .name = "sc2723-fgu",
		   .of_match_table = of_match_ptr(sprdfgu_2723_of_match),
		   },
	.probe = sprdfgu_2723_probe,
	.remove = sprdfgu_2723_remove
};

static int __init sprdfgu_2723_driver_init(void)
{
	return platform_driver_register(&sprdfgu_2723_driver);
}

static void __exit sprdfgu_2723_driver_exit(void)
{
	platform_driver_unregister(&sprdfgu_2723_driver);
}

subsys_initcall_sync(sprdfgu_2723_driver_init);
module_exit(sprdfgu_2723_driver_exit);

