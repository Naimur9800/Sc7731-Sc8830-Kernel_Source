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

#include <linux/reboot.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/iio/consumer.h>
#include <linux/spi/spi-sprd-adi.h>
#include <linux/gpio.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/sort.h>
#include "sprd_charge_helper.h"

#define SAMPLE_NUM  (1)
#define TEMP_BUFF_CNT (5)
#define CUR_RESULT_NUM (4)

static struct regmap *reg_map;
static int temp_buff[TEMP_BUFF_CNT] = { 200, 200, 200, 200, 200 };
static struct sprd_battery_platform_data *pbat_data;

static struct regmap *sprd_get_regmap(void)
{
	struct platform_device *pdev_regmap;
	struct device_node *regmap_np =
		of_find_compatible_node(NULL, NULL, "sprd,pmic-glb");

	if (!regmap_np)
		return NULL;

	pdev_regmap = of_find_device_by_node(regmap_np);
	if (!pdev_regmap) {
		of_node_put(regmap_np);
		return NULL;
	}

	reg_map = dev_get_regmap(pdev_regmap->dev.parent, NULL);
	if (!reg_map)
		return NULL;
	return reg_map;
}

static int sprd_chg_check_regmap(void)
{
	if (reg_map)
		return 0;

	reg_map = sprd_get_regmap();
	if (!reg_map) {
		pr_err("%s, get regmap fail\n", __func__);
		return -ENODEV;
	}
	return 0;
}

unsigned int glb_adi_read(u32 reg)
{
	unsigned int val;
	int ret;

	ret = sprd_chg_check_regmap();
	if (ret)
		return 0;
	regmap_read(reg_map, reg, &val);
	return val;
}

int glb_adi_write(u32 reg, u32 or_val, u32 clear_msk)
{
	int ret;

	ret = sprd_chg_check_regmap();
	if (ret)
		return ret;
	return regmap_update_bits(reg_map, reg,
		clear_msk, or_val);
}

int sprd_charge_pd_control(bool enable)
{
	if (enable)
		return glb_adi_write(ANA_REG_GLB_CHGR_CTRL0, 0, BIT_CHGR_PD);

	return glb_adi_write(ANA_REG_GLB_CHGR_CTRL0, BIT_CHGR_PD, BIT_CHGR_PD);
}

static uint32_t sprd_chg_read_adc(struct iio_channel *channel)
{
	int err;
	uint32_t val;

	err = iio_read_channel_processed(channel, &val);
	if (err < 0)
		return err;
	return val;
}

int sprdchg_init(struct sprd_battery_platform_data *pdata)
{
	pbat_data = pdata;
	reg_map = sprd_get_regmap();
	if (!reg_map) {
		pr_err("%s, get regmap fail\n", __func__);
		return -ENODEV;
	}
	return 0;
}

int cmpint(const void *a, const void *b)
{
	return *(int *)a - *(int *)b;
}

int sprdchg_read_ntc_vol(struct iio_channel *channel_temp)
{
	int cnt = SAMPLE_NUM;
	int i;
	int adc_val[SAMPLE_NUM];

	for (i = 0; i < SAMPLE_NUM; i++)
		adc_val[i] = sprd_chg_read_adc(channel_temp);
	sort(adc_val, SAMPLE_NUM, sizeof(int), cmpint, NULL);
	pr_info("sprdchg: sprdchg_read_temp_adc:%d\n",
		adc_val[cnt / 2]);
	return adc_val[cnt / 2];
}

int sprdchg_read_temp_adc(void)
{
	int cnt = SAMPLE_NUM;

	if (pbat_data->temp_support) {
		int i, j, temp;
		int adc_val[SAMPLE_NUM];

		for (i = 0; i < SAMPLE_NUM; i++)
			adc_val[i] = sprd_chg_read_adc(pbat_data->channel_temp);

		for (j = 1; j <= SAMPLE_NUM - 1; j++) {
			for (i = 0; i < SAMPLE_NUM - j; i++) {
				if (adc_val[i] > adc_val[i + 1]) {
					temp = adc_val[i];
					adc_val[i] = adc_val[i + 1];
					adc_val[i + 1] = temp;
				}
			}
		}
		pr_info("sprdchg: sprdchg_read_temp_adc:%d\n",
			adc_val[cnt / 2]);
		return adc_val[cnt / 2];
	} else {
		return 3000;
	}
}

static int sprdchg_temp_vol_comp(int vol)
{
	int bat_cur = sprdfgu_read_batcurrent();
	int res_comp = pbat_data->temp_comp_res;
	int vol_comp = 0;

	pr_info("sprdchg: sprdchg_temp_vol_comp bat_cur:%d\n", bat_cur);
	vol_comp = (bat_cur * res_comp) / 1000;
	vol = vol - vol_comp;
	if (vol < 0)
		vol = 0;

	return vol;
}

static void sprdchg_update_temp_buff(int temp)
{
	static int pointer;

	if (pointer >= TEMP_BUFF_CNT)
		pointer = 0;

	temp_buff[pointer++] = temp;
}

static int sprdchg_get_temp_from_buff(void)
{
	int i = 0, j = 0, temp;
	int t_temp_buff[TEMP_BUFF_CNT] = { 0 };

	for (i = 0; i < TEMP_BUFF_CNT; i++) {
		t_temp_buff[i] = temp_buff[i];
		pr_info("sprdchg: temp_buff[%d]:%d\n", i, temp_buff[i]);
	}

	for (j = 1; j <= TEMP_BUFF_CNT - 1; j++) {
		for (i = 0; i < TEMP_BUFF_CNT - j; i++) {
			if (t_temp_buff[i] > t_temp_buff[i + 1]) {
				temp = t_temp_buff[i];
				t_temp_buff[i] = t_temp_buff[i + 1];
				t_temp_buff[i + 1] = temp;
			}
		}
	}
	return t_temp_buff[TEMP_BUFF_CNT / 2];
}

int sprdchg_search_temp_tab(int val)
{
	return sprdbat_interpolate(val, pbat_data->temp_tab_size,
				   pbat_data->temp_tab);
}

#define TEMP_BUFF_EN
int sprdchg_read_temp(void)
{
	if (pbat_data->temp_support) {
		int temp;
		int val = sprdchg_read_temp_adc();

		pr_info("sprdchg:temp: %d, raw:%d\n",
			val, sprdchg_search_temp_tab(val));
		val = sprdchg_temp_vol_comp(val);
		pr_info("sprdchg: sprdchg_read_temp comp voltage:%d\n",
			val);
		temp = sprdchg_search_temp_tab(val);
#ifdef TEMP_BUFF_EN
		sprdchg_update_temp_buff(temp);
		temp = sprdchg_get_temp_from_buff();
#endif
		pr_info("sprdchg: sprdchg_read_temp temp result:%d\n", temp);
		return temp;
	} else {
		return 200;
	}
}

uint32_t sprdchg_read_vchg_vol(void)
{
	return sprd_chg_read_adc(pbat_data->channel_vchg);
}

uint32_t sprdchg_read_vbat_vol(void)
{
	return sprd_chg_read_adc(pbat_data->channel_vbat);
}

enum usb_charger_type sprdchg_charger_is_adapter(void)
{
	int ret = SDP_TYPE;
	int charger_status, cnt = 40;

	while ((!(glb_adi_read(ANA_REG_GLB_CHGR_STATUS) &
		BIT_CHG_DET_DONE)) && cnt--) {
		/* Adding delay is to read the register chg_status
		 * value successfully.
		 * changing the delay from 200ms to 50ms is to speed
		 * up charging icon display.
		 */
		msleep(50);
	}

	charger_status = glb_adi_read(ANA_REG_GLB_CHGR_STATUS);
	pr_info("cnt:%d,charger_status:0x%x,chgr_det_ctrl:0x%x\n", cnt,
		charger_status, glb_adi_read(ANA_REG_GLB_CHGR_DET_FGU_CTRL));

	charger_status &=
		(BIT_CDP_INT | BIT_DCP_INT |
		BIT_SDP_INT);

	switch (charger_status) {
	case BIT_CDP_INT:
		ret = CDP_TYPE;
		break;
	case BIT_DCP_INT:
		ret = DCP_TYPE;
		break;
	case BIT_SDP_INT:
		ret = SDP_TYPE;
		break;
	default:
		ret = UNKNOWN_TYPE;
	}
	return ret;
}

enum usb_charger_type sprdchg_charger_is_adapter_for_usb(struct usb_charger *p)
{
	return sprdchg_charger_is_adapter();
}
EXPORT_SYMBOL_GPL(sprdchg_charger_is_adapter_for_usb);

#ifdef CONFIG_LEDS_TRIGGERS
void sprdchg_led_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness brightness)
{
	/* TODO: maybe need add */
}
#endif
