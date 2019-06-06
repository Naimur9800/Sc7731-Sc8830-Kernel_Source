/*
 * rt5512.c  --  ALC5512 Smart DMIC bridge driver
 *
 * Copyright 2014 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "rt5512.h"





#define VERSION "0.0.5"

static struct i2c_client *rt5512_i2c;
static const struct firmware *rt5512_fw = NULL;
static struct workqueue_struct *rt5512_wq;

static bool rt5512_booted = false;
static bool rt5512_fw_ready = false;
static bool rt5512_start_rest = false;
static bool rt5512_use_amic = true;
SMicFWSubHeader sMicFWSubHeader_save;


static DECLARE_WAIT_QUEUE_HEAD(voice_input_waitq);




static int rt5512_write(struct i2c_client *i2c, unsigned int reg,
	unsigned int value)
{
	u8 data[3];
	int ret;

	data[0] = reg & 0xff;
	data[1] = (0xff00 & value) >> 8;
	data[2] = value & 0xff;

	dev_dbg(&i2c->dev, "write %02x = %04x\n", reg, value);

	ret = i2c_master_send(i2c, data, 3);
	if (ret == 3)
		return 0;
	if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int rt5512_read(struct i2c_client *i2c, unsigned int r)
{
	struct i2c_msg xfer[2];
	u8 reg[1];
	u8 data[2];
	int value = 0x0;
	int ret;

	/* Write register */
	reg[0] = r & 0xff;
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &reg[0];

	/* Read data */
	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 2;
	xfer[1].buf = data;

	ret = i2c_transfer(i2c->adapter, xfer, 2);
	if (ret != 2) {
		dev_err(&i2c->dev, "i2c_transfer() returned %d\n", ret);
		return -EIO;
	}

	value = (data[0] << 8) | data[1];

	dev_dbg(&i2c->dev, "read %02x => %04x\n", r, value);

	return value;
}
static int rt5512_dsp_burst_write(struct i2c_client *i2c,
        unsigned int addr,  const u8 *buf, size_t size)
{
        unsigned int pos = 0;
        int ret;
        u8 *data;

        rt5512_write(i2c, RT5512_DSP_MEM_CTRL1, addr & 0xffff);
        rt5512_write(i2c, RT5512_DSP_MEM_CTRL2, (addr & 0xffff0000) >> 16);

        data = vmalloc(size + 1);
        if (!data) {
                printk("alloc mem fail\n");
                return 0;
        }

        data[0] = RT5512_DSP_MEM_CTRL7;
        while (size > pos) {
                data[1 + pos] = buf[pos + 1];
                data[2 + pos] = buf[pos + 0];
                data[3 + pos] = buf[pos + 3];
                data[4 + pos] = buf[pos + 2];

                pos += 4;
        }

        ret = i2c_master_send(i2c, data, size + 1);
        if (ret < 0)
                return ret;

        if (data)
                vfree(data);

        return 0;
}
#ifdef RT5512_DUMP_MEM
static unsigned int rt5512_dsp_read(struct i2c_client *i2c,
	unsigned int addr)
{
	unsigned int ret;
	int value;

	rt5512_write(i2c, RT5512_DSP_MEM_CTRL1, addr & 0xffff);
	rt5512_write(i2c, RT5512_DSP_MEM_CTRL2, (addr & 0xffff0000) >> 16);
	rt5512_write(i2c, RT5512_DSP_MEM_CTRL5, 0x02);

	value = rt5512_read(i2c, RT5512_DSP_MEM_CTRL3);
	ret = value;

	value= rt5512_read(i2c, RT5512_DSP_MEM_CTRL4);
	ret |= value << 16;

	dev_dbg(&i2c->dev, "%08x: %08x\n", addr, ret);

	return ret;
}
#endif
void rt5512_dsp_one_shot(void) {
	/* M1/N1/P1/Q1 Option */
	rt5512_write(rt5512_i2c, RT5512_PLL_CLOCK_CTRL3, 0x0901);
	rt5512_write(rt5512_i2c, RT5512_UPFILTER_CTRL1, 0x6aaf);
	rt5512_write(rt5512_i2c, RT5512_UPFILTER_CTRL1, 0x6eaf);
	rt5512_write(rt5512_i2c, RT5512_PWR_DIG, 0x0003);
	rt5512_write(rt5512_i2c, RT5512_DMIC_DATA_CTRL, 0x0091);
	rt5512_write(rt5512_i2c, RT5512_DSP_CTRL2, 0x0000);
}
EXPORT_SYMBOL_GPL(rt5512_dsp_one_shot);

void rt5512_enable_dsp_clock(void) {
	if (rt5512_use_amic) {
		rt5512_write(rt5512_i2c, RT5512_AUTO_MODE_CTRL, 0x00c0);
		rt5512_write(rt5512_i2c, RT5512_PWR_ANLG1, 0x0bbf);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0005);
		rt5512_write(rt5512_i2c, RT5512_DIG_PAD_CTRL2, 0x0400);
		rt5512_write(rt5512_i2c, RT5512_ANA_CIRCUIT_CTRL_MICBST, 0x2000);
		rt5512_write(rt5512_i2c, RT5512_ANA_CIRCUIT_CTRL_ADCFED, 0x0800);
		rt5512_write(rt5512_i2c, RT5512_ADC_EXT_CTRL1, 0x0045);
		rt5512_write(rt5512_i2c, RT5512_UPFILTER_CTRL2, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_ANA_CIRCUIT_CTRL_INPUTBUF, 0x0083);
		rt5512_write(rt5512_i2c, RT5512_AD_DIG_FILTER_CTRL1, 0x8a37);
		rt5512_write(rt5512_i2c, RT5512_DUMMY_RTK4, 0x0000);
	} else {
		rt5512_write(rt5512_i2c, RT5512_AUTO_MODE_CTRL, 0x00c0);
		rt5512_write(rt5512_i2c, RT5512_PWR_ANLG1, 0x8ba1);
		rt5512_write(rt5512_i2c, RT5512_ANA_CIRCUIT_CTRL_LDO2, 0xa342);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0005);
		rt5512_write(rt5512_i2c, RT5512_DIG_PAD_CTRL2, 0x0400);
		rt5512_write(rt5512_i2c, RT5512_DUMMY_RTK3, 0x6395);
		rt5512_write(rt5512_i2c, RT5512_DUMMY_RTK4, 0x0000);
	}
}

void rt5512_set_clock_cal_pll(void) {
	/* M1/M10 Option */
	rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL8, 0x0022);
	if (rt5512_use_amic) {
		rt5512_write(rt5512_i2c, RT5512_PLL_CLK_EXT_CTRL1, 0x0008);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLK_EXT_CTRL2, 0x4202);
	} else {
		rt5512_write(rt5512_i2c, RT5512_PLL_CLK_EXT_CTRL1, 0x0040);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLK_EXT_CTRL2, 0x2202);
	}
	rt5512_write(rt5512_i2c, RT5512_PLL_CLOCK_CTRL1, 0x0a06);
	rt5512_write(rt5512_i2c, RT5512_PLL_CLOCK_CTRL2, 0x00d3);
	rt5512_write(rt5512_i2c, RT5512_PLL_CLOCK_CTRL3, 0x0904);
	rt5512_write(rt5512_i2c, RT5512_BUF_MODE_CTRL_PLL_CAL1, 0x8040);
	rt5512_write(rt5512_i2c, RT5512_BUF_MODE_CTRL_PLL_CAL2, 0x0200);
	rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL1, 0x0000);
	rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL2, 0x01f4);
	rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL8, 0x8022);
	rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL8, 0xc022);
}

void rt5512_dsp_start_first(void) {
	if (rt5512_use_amic) {
		rt5512_write(rt5512_i2c, RT5512_PWR_ANLG2, 0x3f00);
		rt5512_write(rt5512_i2c, RT5512_ANA_CIRCUIT_CTRL_VREF, 0x8e50);
		rt5512_write(rt5512_i2c, RT5512_AUTO_MODE_CTRL, 0x00c0);
		rt5512_write(rt5512_i2c, RT5512_DSP_CTRL1, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_GPIO_CTRL1, 0xa800);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0005);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0007);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0005);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0004);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL7, 0x8000);
		rt5512_write(rt5512_i2c, RT5512_AD_DIG_FILTER_CTRL2, 0x0021);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0001);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0019);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0018);
		rt5512_write(rt5512_i2c, RT5512_PWR_DIG, 0x0001);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL6, 0x003a);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL7, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_VAD_CTRL1, 0x21c0);
		rt5512_write(rt5512_i2c, RT5512_VAD_STATUS1, 0x8000);
		rt5512_write(rt5512_i2c, RT5512_VAD_CTRL2, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_VAD_CTRL3, 0xe001);
		rt5512_write(rt5512_i2c, RT5512_VAD_CTRL4, 0x7c0a);
		rt5512_write(rt5512_i2c, RT5512_VAD_STATUS1, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_DMIC_DATA_CTRL, 0x0090);
		rt5512_write(rt5512_i2c, RT5512_DIG_PAD_CTRL2, 0x0500);
		rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL8, 0x8022);
		rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL8, 0xc022);
	} else {
		rt5512_write(rt5512_i2c, RT5512_PWR_ANLG1, 0x8ba1);
		rt5512_write(rt5512_i2c, RT5512_AUTO_MODE_CTRL, 0x00c0);
		rt5512_write(rt5512_i2c, RT5512_DSP_CTRL1, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_GPIO_CTRL1, 0xa800);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0005);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0007);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0005);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0004);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL7, 0x8000);
		rt5512_write(rt5512_i2c, RT5512_AD_DIG_FILTER_CTRL2, 0x0022);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0001);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0019);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0018);
		rt5512_write(rt5512_i2c, RT5512_PWR_DIG, 0x0001);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL6, 0x003a);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL7, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_VAD_CTRL1, 0x21c0);
		rt5512_write(rt5512_i2c, RT5512_VAD_STATUS1, 0x8000);
		rt5512_write(rt5512_i2c, RT5512_AD_DIG_FILTER_CTRL1, 0x8a2f);
		rt5512_write(rt5512_i2c, RT5512_VAD_CTRL2, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_VAD_CTRL3, 0xe001);
		rt5512_write(rt5512_i2c, RT5512_VAD_CTRL4, 0x7c0a);
		rt5512_write(rt5512_i2c, RT5512_VAD_STATUS1, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_DMIC_DATA_CTRL, 0x0090);
		rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL8, 0x8022);
		rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL8, 0xc022);
	}
#ifdef RT5512_LPSD
	msleep(10);
	rt5512_write(rt5512_i2c, RT5512_VAD_CTRL1, 0x5040);
	rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0004);
	rt5512_write(rt5512_i2c, RT5512_DSP_CTRL5, 0x0011);
#endif
}

void rt5512_dsp_start_rest(void) {
	if (rt5512_use_amic) {
		/* M10 Option */
		rt5512_write(rt5512_i2c, RT5512_AD_DIG_FILTER_CTRL1, 0x8a37);
		rt5512_write(rt5512_i2c, RT5512_DSP_CTRL2, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_AUTO_MODE_CTRL, 0x00c0);
		rt5512_write(rt5512_i2c, RT5512_PWR_ANLG1, 0x0bbf);
		rt5512_write(rt5512_i2c, RT5512_PWR_ANLG2, 0x3f00);
		rt5512_write(rt5512_i2c, RT5512_ANA_CIRCUIT_CTRL_VREF, 0x8e50);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0005);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0004);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL1, 0x0fff);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL7, 0x8000);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLOCK_CTRL1, 0x0a06);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLOCK_CTRL2, 0x00d3);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLOCK_CTRL3, 0x0904);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLK_EXT_CTRL1, 0x0008);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLK_EXT_CTRL2, 0x4202);
		rt5512_write(rt5512_i2c, RT5512_AD_DIG_FILTER_CTRL2, 0x0021);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0001);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0019);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0018);
		rt5512_write(rt5512_i2c, RT5512_PWR_DIG, 0x0001);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL6, 0x003a);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL7, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_VAD_CTRL1, 0x21c0);
		rt5512_write(rt5512_i2c, RT5512_VAD_STATUS1, 0x8000);
		rt5512_write(rt5512_i2c, RT5512_VAD_STATUS1, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_DMIC_DATA_CTRL, 0x0090);
		rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL8, 0x8022);
		rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL8, 0xc022);
	} else {
		/* M1 Option */
		rt5512_write(rt5512_i2c, RT5512_DSP_CTRL2, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_PWR_ANLG1, 0x8ba1);
		rt5512_write(rt5512_i2c, RT5512_AUTO_MODE_CTRL, 0x00c0);
		rt5512_write(rt5512_i2c, RT5512_DSP_CTRL1, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0005);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0004);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL1, 0x0fff);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL7, 0x8000);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLOCK_CTRL3, 0x0904);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLK_EXT_CTRL2, 0x2202);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0001);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0019);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0018);
		rt5512_write(rt5512_i2c, RT5512_PWR_DIG, 0x0001);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL6, 0x003a);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL7, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_VAD_CTRL1, 0x21c0);
		rt5512_write(rt5512_i2c, RT5512_VAD_STATUS1, 0x8000);
		rt5512_write(rt5512_i2c, RT5512_VAD_STATUS1, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_DMIC_DATA_CTRL, 0x0090);
		rt5512_write(rt5512_i2c, RT5512_AD_DIG_FILTER_CTRL1, 0x8a2f);
		rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL8, 0x8022);
		rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL8, 0xc022);
	}
#ifdef RT5512_LPSD
	msleep(10);
	rt5512_write(rt5512_i2c, RT5512_VAD_CTRL1, 0x5040);
	rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0004);
	rt5512_write(rt5512_i2c, RT5512_DSP_CTRL5, 0x0011);
#endif
}

void rt5512_dsp_start(void) {
	if (rt5512_start_rest) {
		rt5512_dsp_start_rest();
	} else {
		rt5512_dsp_start_first();
		rt5512_start_rest = true;

	}
}
EXPORT_SYMBOL_GPL(rt5512_dsp_start);

void rt5512_dsp_stop(void) {
	if (rt5512_use_amic) {
		/* T10 Option */
		rt5512_write(rt5512_i2c, RT5512_AUTO_MODE_CTRL, 0x03c1);
		rt5512_write(rt5512_i2c, RT5512_PWR_ANLG1, 0x0bbf);
		rt5512_write(rt5512_i2c, RT5512_PWR_ANLG2, 0xfcff);
		rt5512_write(rt5512_i2c, RT5512_ANA_CIRCUIT_CTRL_VREF, 0x8f50);
		rt5512_write(rt5512_i2c, RT5512_VAD_STATUS1, 0x8000);
		rt5512_write(rt5512_i2c, RT5512_DSP_CTRL2, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0019);
		rt5512_write(rt5512_i2c, RT5512_VAD_CTRL1, 0x1040);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL6, 0x002a);
		rt5512_write(rt5512_i2c, RT5512_PWR_DIG, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLOCK_CTRL1, 0x0a06);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLOCK_CTRL2, 0x8011);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLK_EXT_CTRL1, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLK_EXT_CTRL2, 0x9208);
		rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL8, 0x8022);
		rt5512_write(rt5512_i2c, RT5512_PLL_CAL_CTRL8, 0xc022);
		rt5512_write(rt5512_i2c, RT5512_DMIC_DATA_CTRL, 0x009f);
	} else {
		rt5512_write(rt5512_i2c, RT5512_VAD_STATUS1, 0x8000);
		rt5512_write(rt5512_i2c, RT5512_DSP_CTRL2, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_PWR_DSP, 0x0019);
		rt5512_write(rt5512_i2c, RT5512_AUTO_MODE_CTRL, 0x03c1);
		rt5512_write(rt5512_i2c, RT5512_PLL_CLK_EXT_CTRL2, 0x2008);
		rt5512_write(rt5512_i2c, RT5512_DMIC_DATA_CTRL, 0x0092);
		rt5512_write(rt5512_i2c, RT5512_PWR_DIG, 0x0000);
		rt5512_write(rt5512_i2c, RT5512_VAD_CTRL1, 0x1040);
		rt5512_write(rt5512_i2c, RT5512_BUF_SRAM_CTRL6, 0x002a);
		rt5512_write(rt5512_i2c, RT5512_PWR_ANLG1, 0x88a0);
	}
#ifdef RT5512_LPSD
	rt5512_write(rt5512_i2c, RT5512_DSP_CTRL5, 0x0000);
#endif
}
EXPORT_SYMBOL_GPL(rt5512_dsp_stop);

void rt5512_dsp_reload(struct rt5512_soundtrigger_data* rt5512_st) {
	schedule_delayed_work(&rt5512_st->reload_work, msecs_to_jiffies(50));
}
EXPORT_SYMBOL_GPL(rt5512_dsp_reload);

int rt5512_irq_is_triggered(void){
	return !!(rt5512_read(rt5512_i2c, RT5512_GPIO_CTRL1) & 0x8000);
}
EXPORT_SYMBOL_GPL(rt5512_irq_is_triggered);

static size_t rt5512_read_file(char *file_path, const u8 **buf)
{
	loff_t pos = 0;
	unsigned int file_size = 0;
	struct file *fp;
	fp = filp_open(file_path, O_RDONLY, 0);
	pr_info("rt5512_read_file , file_path = %s , IS_ERR(fp)= %d" ,file_path ,IS_ERR(fp));

	if (!IS_ERR(fp)) {
		file_size = vfs_llseek(fp, pos, SEEK_END);

		*buf = kzalloc(file_size, GFP_KERNEL);
		if (*buf == NULL) {
			filp_close(fp, 0);
			return 0;
		}

		kernel_read(fp, pos, (char *)*buf, file_size);
		filp_close(fp, 0);

		return file_size;
	}

	return 0;
}
#if defined(RT5512_DUMP_MEM) || defined(RT5512_DUMP_BUF)
static void rt5512_write_file(char *file_path, const u8 *buf, size_t size,
	loff_t pos)
{
	struct file *fp;
	ssize_t res;

	fp = filp_open(file_path, O_CREAT | O_WRONLY, 0);
	if (!IS_ERR(fp)) {
		res = rt5512_kernel_write(fp, (char *)buf, size, pos);
		filp_close(fp, 0);
	}
}
#endif
static unsigned int rt5512_4byte_le_to_uint(const u8 *data)
{
	return data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24;
}
#ifdef RT5512_DUMP_MEM
static void rt5512_uint_to_4byte_le(unsigned int value, u8 *data)
{
		data[0] = (value & 0xff);
		data[1] = ((value & 0xff00) >> 8);
		data[2] = ((value & 0xff0000) >> 16);
		data[3] = ((value & 0xff000000) >> 24);
}
#endif

int rt5512_parse_td_header(struct rt5512_soundtrigger_data *rt_5512)
{
	int i;
	const u8 *data = rt_5512->firmware_data;
	unsigned int size = rt_5512->firmware_size;
	if (!sMicFWSubHeader_save.TDArray)
		return EINVAL;

	for (i = 0 ; i < sMicFWSubHeader_save.NumTD; i++) {
		if (size) {
			rt5512_dsp_burst_write(rt5512_i2c,
				sMicFWSubHeader_save.TDArray[i].Addr, data, size);
			continue;
		}
		#if 0
		 else {
			sprintf(file_path, RT5512_FIRMWARE "SMicTD%u.dat",
				sMicFWSubHeader_save.TDArray[i].ID);

			size = rt5512_read_file(file_path, &data);
			if (size) {
				rt5512_dsp_burst_write(rt5512_i2c,
					sMicFWSubHeader_save.TDArray[i].Addr, data1, size);
		    }
	    }
	   #endif 
	}
	return size ;
}

void rt5512_parse_header(const u8 *buf)
{
	SMicFWHeader sMicFWHeader;
	SMicFWSubHeader sMicFWSubHeader;

	int i, offset = 0;
	const u8 *data;
	char file_path[32];
	unsigned int size;

	sMicFWHeader.Sync = rt5512_4byte_le_to_uint(buf);
	dev_dbg(&rt5512_i2c->dev, "sMicFWHeader.Sync = %08x\n", sMicFWHeader.Sync);

	offset += 4;
	sMicFWHeader.Version =  rt5512_4byte_le_to_uint(buf + offset);
	dev_dbg(&rt5512_i2c->dev, "sMicFWHeader.Version = %08x\n", sMicFWHeader.Version);

	offset += 4;
	sMicFWHeader.NumBin = rt5512_4byte_le_to_uint(buf + offset);
	dev_dbg(&rt5512_i2c->dev, "sMicFWHeader.NumBin = %08x\n", sMicFWHeader.NumBin);

	sMicFWHeader.BinArray =
		kzalloc(sizeof(SMicBinInfo) * sMicFWHeader.NumBin, GFP_KERNEL);

	for (i = 0 ; i < sMicFWHeader.NumBin; i++) {
		offset += 4;
		sMicFWHeader.BinArray[i].Offset = rt5512_4byte_le_to_uint(buf + offset);
		dev_dbg(&rt5512_i2c->dev, "sMicFWHeader.BinArray[%d].Offset = %08x\n", i, sMicFWHeader.BinArray[i].Offset);

		offset += 4;
		sMicFWHeader.BinArray[i].Size = rt5512_4byte_le_to_uint(buf + offset);
		dev_dbg(&rt5512_i2c->dev, "sMicFWHeader.BinArray[%d].Size = %08x\n", i, sMicFWHeader.BinArray[i].Size);

		offset += 4;
		sMicFWHeader.BinArray[i].Addr = rt5512_4byte_le_to_uint(buf + offset);
		dev_dbg(&rt5512_i2c->dev, "sMicFWHeader.BinArray[%d].Addr = %08x\n", i, sMicFWHeader.BinArray[i].Addr);

		rt5512_dsp_burst_write(rt5512_i2c, sMicFWHeader.BinArray[i].Addr,
			buf + sMicFWHeader.BinArray[i].Offset, sMicFWHeader.BinArray[i].Size);

#ifdef RT5512_DUMP_BUF
		if ((sMicFWHeader.BinArray[i].Addr & 0xffff0000) == 0x0fff0000)
			rt5512_write_file("/sdcard/rt5512/0x0fff0000_fw", buf + sMicFWHeader.BinArray[i].Offset, sMicFWHeader.BinArray[i].Size, sMicFWHeader.BinArray[i].Addr - 0x0fff0000);
		else
			rt5512_write_file("/sdcard/rt5512/0x0ffe0000_fw", buf + sMicFWHeader.BinArray[i].Offset, sMicFWHeader.BinArray[i].Size, sMicFWHeader.BinArray[i].Addr - 0x0ffe0000);
#endif
	}

	offset += 4;
	sMicFWSubHeader.NumTD = rt5512_4byte_le_to_uint(buf + offset);
	dev_dbg(&rt5512_i2c->dev, "sMicFWSubHeader.NumTD = %08x\n", sMicFWSubHeader.NumTD);

	sMicFWSubHeader.TDArray =
		kzalloc(sizeof(SMicTDInfo) * sMicFWSubHeader.NumTD, GFP_KERNEL);

	if (sMicFWSubHeader_save.TDArray)
		kfree(sMicFWSubHeader_save.TDArray);
	sMicFWSubHeader_save.NumTD = sMicFWSubHeader.NumTD;
	sMicFWSubHeader_save.TDArray =
	kzalloc(sizeof(SMicTDInfo) * sMicFWSubHeader.NumTD, GFP_KERNEL);

	for (i = 0 ; i < sMicFWSubHeader.NumTD; i++) {
		offset += 4;
		sMicFWSubHeader.TDArray[i].ID = rt5512_4byte_le_to_uint(buf + offset);
		dev_err(&rt5512_i2c->dev, "sMicFWSubHeader.TDArray[%d].ID = %08x\n", i, sMicFWSubHeader.TDArray[i].ID);

		offset += 4;
		sMicFWSubHeader.TDArray[i].Addr = rt5512_4byte_le_to_uint(buf + offset);
		dev_err(&rt5512_i2c->dev, "sMicFWSubHeader.TDArray[%d].Addr = %08x\n", i, sMicFWSubHeader.TDArray[i].Addr);

		sMicFWSubHeader_save.TDArray[i].ID = sMicFWSubHeader.TDArray[i].ID;
		sMicFWSubHeader_save.TDArray[i].Addr = sMicFWSubHeader.TDArray[i].Addr;

		sprintf(file_path, RT5512_FIRMWARE "SMicTD%u.dat",
			sMicFWSubHeader.TDArray[i].ID);

		size = rt5512_read_file(file_path, &data);

		if (size) {
			rt5512_dsp_burst_write(rt5512_i2c,
				sMicFWSubHeader.TDArray[i].Addr, data, size);
#ifdef RT5512_DUMP_BUF
			if ((sMicFWSubHeader.TDArray[i].Addr & 0xffff0000) == 0x0fff0000)
				rt5512_write_file("/sdcard/rt5512/0x0fff0000_fw", data, size, sMicFWSubHeader.TDArray[i].Addr - 0x0fff0000);
			else
				rt5512_write_file("/sdcard/rt5512/0x0ffe0000_fw", data, size, sMicFWSubHeader.TDArray[i].Addr - 0x0ffe0000);
#endif
			kfree(data);
		}
	}

	if (sMicFWHeader.BinArray)
		kfree(sMicFWHeader.BinArray);

	if (sMicFWSubHeader.TDArray)
		kfree(sMicFWSubHeader.TDArray);
}

static void rt5512_reset(struct i2c_client *i2c)
{
	rt5512_write(i2c, RT5512_RESET, 0x10ec);
}
#ifdef RT5512_DUMP_MEM
static void rt5512_dump_mem(struct i2c_client *i2c)
{
	unsigned int i, value;
	char data[4];

	for (i = 0 ; i < 0xc000; i += 4) {
		value = rt5512_dsp_read(i2c, 0x0ffe0000 + i);
		rt5512_uint_to_4byte_le(value, (u8 *)&data);
		rt5512_write_file("/sdcard/rt5512/0x0ffe0000", data, 4, i);
	}

	for (i = 0 ; i < 0x6000; i += 4) {
		value = rt5512_dsp_read(i2c, 0x0fff0000 + i);
		rt5512_uint_to_4byte_le(value, (u8 *)&data);
		rt5512_write_file("/sdcard/rt5512/0x0fff0000", data, 4, i);
	}
}
#endif
#if 1
static int rt5512_dsp_fast_load_fw(struct rt5512_soundtrigger_data *rt_5512)
{
	int length ;
	rt5512_start_rest = false;
	rt5512_fw_ready = false;
	dev_err(&rt5512_i2c->dev, "firmware td loading start\n " );
	rt5512_enable_dsp_clock();
	length = rt5512_parse_td_header(rt_5512);
	rt5512_set_clock_cal_pll();
	rt5512_dsp_start();
	rt5512_dsp_stop();

	/*rt5512_booted = true;*/
	rt5512_fw_ready = true;

	dev_err(&rt5512_i2c->dev, "firmware td loading end\n ");
	return length ;

}
#endif

static void rt5512_dsp_load_fw(struct rt5512_soundtrigger_data *rt5512_st) {
#ifdef RT5512_USE_NATIVE
	const struct firmware *fw = NULL;
#else
	const u8 *data;
	unsigned int size = 0;
	char file_path[32];
	int count = 0;
	if (rt5512_fw == NULL)
		request_firmware(&rt5512_fw, "SMicBin.dat", &rt5512_i2c->dev);
#endif
	rt5512_fw_ready = false;
	rt5512_reset(rt5512_i2c);

	rt5512_enable_dsp_clock();
#ifdef RT5512_USE_NATIVE
	request_firmware(&fw, RT5512_FIRMWARE1, &rt5512_i2c->dev);
	if (fw) {
		rt5512_dsp_burst_write(rt5512_i2c, 0x0ffe0000, fw->data, fw->size);
		release_firmware(fw);
		fw = NULL;
	}

	request_firmware(&fw, RT5512_FIRMWARE2, &rt5512_i2c->dev);
	if (fw) {
		rt5512_dsp_burst_write(rt5512_i2c, 0x0fff0000, fw->data, fw->size);
		release_firmware(fw);
		fw = NULL;
	}
#else
	sprintf(file_path, RT5512_CUSTOM_FIRMWARE "SMicBin.dat");

	while (true) {
		size = rt5512_read_file(file_path, &data);
		dev_err(&rt5512_i2c->dev, "firmware path = %s (%u)\n", file_path, size);

		if (size || count >= RT5512_WAIT_FS_READY_SECONDS || rt5512_booted)
			break;

		msleep(1000);
		count++;
	}
	if (size) {
		rt5512_parse_header(data);
		kfree(data);
	} else {
		if (rt5512_fw)
			rt5512_parse_header(rt5512_fw->data);
	}
#endif
#ifdef RT5512_DUMP_MEM
	rt5512_dump_mem(rt5512_i2c);
#endif
	rt5512_set_clock_cal_pll();
	rt5512_dsp_start();
	rt5512_dsp_stop();

	release_firmware(rt5512_fw);
	rt5512_fw = NULL;

	rt5512_booted = true;
	rt5512_fw_ready = true;
	dev_err(&rt5512_i2c->dev, "firmware loading end \n" );
}

static void rt5512_fw_loaded(const struct firmware *fw, void *context)
{
	if (fw) {
		rt5512_fw = fw;
		rt5512_dsp_load_fw((struct rt5512_soundtrigger_data *)context);
	}
}

static int rt5512_readable_register(unsigned int reg)
{
	switch (reg) {
	case RT5512_RESET:
	case RT5512_ANA_CIRCUIT_CTRL_LDO1:
	case RT5512_ANA_CIRCUIT_CTRL_LDO2:
	case RT5512_ANA_CIRCUIT_CTRL_LDO3:
	case RT5512_ANA_CIRCUIT_CTRL_ADC1_1:
	case RT5512_ANA_CIRCUIT_CTRL_ADC1_2:
	case RT5512_ANA_CIRCUIT_CTRL_ADC2_1:
	case RT5512_ANA_CIRCUIT_CTRL_ADC2_2:
	case RT5512_ANA_CIRCUIT_CTRL_ADC2_3:
	case RT5512_ANA_CIRCUIT_CTRL_MICBST:
	case RT5512_ANA_CIRCUIT_CTRL_ADCFED:
	case RT5512_ANA_CIRCUIT_CTRL_INPUTBUF:
	case RT5512_ANA_CIRCUIT_CTRL_VREF:
	case RT5512_ANA_CIRCUIT_CTRL_MBIAS:
	case RT5512_AD_DIG_FILTER_CTRL1:
	case RT5512_AD_DIG_FILTER_CTRL2:
	case RT5512_DFT_BIST_SCAN:
	case RT5512_UPFILTER_CTRL1:
	case RT5512_UPFILTER_CTRL2:
	case RT5512_GPIO_CTRL1:
	case RT5512_GPIO_CTRL2:
	case RT5512_GPIO_CTRL3:
	case RT5512_GPIO_STATUS:
	case RT5512_DIG_PAD_CTRL1:
	case RT5512_DIG_PAD_CTRL2:
	case RT5512_DMIC_DATA_CTRL:
	case RT5512_TEST_MODE_CTRL1:
	case RT5512_TEST_MODE_CTRL2:
	case RT5512_TEST_MODE_CTRL3:
	case RT5512_VAD_CTRL1:
	case RT5512_VAD_CTRL2:
	case RT5512_VAD_CTRL3:
	case RT5512_VAD_CTRL4:
	case RT5512_VAD_STATUS1:
	case RT5512_VAD_STATUS2:
	case RT5512_BUF_SRAM_CTRL1:
	case RT5512_BUF_SRAM_CTRL2:
	case RT5512_BUF_SRAM_CTRL3:
	case RT5512_BUF_SRAM_CTRL4:
	case RT5512_BUF_SRAM_CTRL5:
	case RT5512_BUF_SRAM_CTRL6:
	case RT5512_BUF_SRAM_CTRL7:
	case RT5512_AUTO_MODE_CTRL:
	case RT5512_PWR_ANLG1:
	case RT5512_PWR_ANLG2:
	case RT5512_PWR_DIG:
	case RT5512_PWR_DSP:
	case RT5512_PRIV_INDEX:
	case RT5512_PRIV_DATA:
	case RT5512_BUF_MODE_CTRL_PLL_CAL1:
	case RT5512_BUF_MODE_CTRL_PLL_CAL2:
	case RT5512_BUF_MODE_CTRL_PLL_CAL3:
	case RT5512_BUF_MODE_CTRL_PLL_CAL4:
	case RT5512_BUF_MODE_CTRL_PLL_CAL5:
	case RT5512_BUF_MODE_CTRL_PLL_CAL6:
	case RT5512_KEY_FHRASE_CTRL_AVD:
	case RT5512_AUTO_CLK_SEL_STATUS1:
	case RT5512_AUTO_CLK_SEL_STATUS2:
	case RT5512_AUTO_CLK_SEL_STATUS3:
	case RT5512_AUTO_CLK_SEL_STATUS4:
	case RT5512_PLL_CLOCK_CTRL1:
	case RT5512_PLL_CLOCK_CTRL2:
	case RT5512_PLL_CLOCK_CTRL3:
	case RT5512_PLL_CAL_CTRL1:
	case RT5512_PLL_CAL_CTRL2:
	case RT5512_PLL_CAL_CTRL3:
	case RT5512_PLL_CAL_CTRL4:
	case RT5512_PLL_CAL_CTRL5:
	case RT5512_PLL_CAL_CTRL6:
	case RT5512_PLL_CAL_CTRL7:
	case RT5512_PLL_CAL_CTRL8:
	case RT5512_PLL_CAL_CTRL9:
	case RT5512_PLL_CAL_STATUS1:
	case RT5512_PLL_CAL_STATUS2:
	case RT5512_PLL_CAL_STATUS3:
	case RT5512_DSP_CTRL1:
	case RT5512_DSP_CTRL2:
	case RT5512_DSP_CTRL3:
	case RT5512_DSP_CTRL4:
	case RT5512_DSP_CTRL5:
	case RT5512_DSP_CTRL6:
	case RT5512_DSP_CTRL7:
	case RT5512_DSP_CTRL8:
	case RT5512_DSP_CTRL9:
	case RT5512_DSP_CTRL10:
	case RT5512_DSP_CTRL11:
	case RT5512_DSP_CTRL12:
	case RT5512_DSP_CTRL13:
	case RT5512_DSP_CTRL14:
	case RT5512_DSP_CTRL15:
	case RT5512_PLL_CLK_EXT_CTRL1:
	case RT5512_PLL_CLK_EXT_CTRL2:
	case RT5512_ADC_EXT_CTRL1:
	case RT5512_DUMMY_RTK1:
	case RT5512_DUMMY_RTK2:
	case RT5512_DUMMY_RTK3:
	case RT5512_DUMMY_RTK4:
	case RT5512_DUMMY_RTK5:
	case RT5512_DUMMY_RTK6:
	case RT5512_DUMMY_RTK7:
	case RT5512_DUMMY_RTK8:
	case RT5512_DUMMY_RTK9:
	case RT5512_DUMMY_RTK10:
	case RT5512_DUMMY_RTK11:
	case RT5512_DUMMY_RTK12:
	case RT5512_DUMMY_RTK13:
	case RT5512_DUMMY_RTK14:
	case RT5512_DUMMY_RTK15:
	case RT5512_DUMMY_RTK16:
	case RT5512_DUMMY_CUSTOMER1:
	case RT5512_DUMMY_CUSTOMER2:
	case RT5512_DUMMY_CUSTOMER3:
	case RT5512_DUMMY_CUSTOMER4:
	case RT5512_DUMMY_CUSTOMER5:
	case RT5512_DUMMY_CUSTOMER6:
	case RT5512_DUMMY_CUSTOMER7:
	case RT5512_DUMMY_CUSTOMER8:
	case RT5512_DUMMY_CUSTOMER9:
	case RT5512_DUMMY_CUSTOMER10:
	case RT5512_DUMMY_CUSTOMER11:
	case RT5512_DUMMY_CUSTOMER12:
	case RT5512_DUMMY_CUSTOMER13:
	case RT5512_DUMMY_CUSTOMER14:
	case RT5512_DUMMY_CUSTOMER15:
	case RT5512_DUMMY_CUSTOMER16:
	case RT5512_DSP_MEM_CTRL1:
	case RT5512_DSP_MEM_CTRL2:
	case RT5512_DSP_MEM_CTRL3:
	case RT5512_DSP_MEM_CTRL4:
	case RT5512_DSP_MEM_CTRL5:
	case RT5512_DSP_MEM_CTRL6:
	case RT5512_DSP_MEM_CTRL7:
	case RT5512_DUMMY1:
	case RT5512_DUMMY2:
	case RT5512_DUMMY3:
	case RT5512_VENDOR_ID:
	case RT5512_VENDOR_ID1:
	case RT5512_VENDOR_ID2:
		return true;
	default:
		return false;
	}
}

static ssize_t rt5512_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int count = 0;
	int i;
	int value;

	for (i = RT5512_RESET; i <= RT5512_VENDOR_ID2; i++) {
		if (rt5512_readable_register(i)) {
			value = rt5512_read(rt5512_i2c, i);
			if (value < 0)
				count += sprintf(buf + count, "%02x: XXXX\n",
					i);
			else
				count += sprintf(buf + count, "%02x: %04x\n", i,
					value);

			if (count >= PAGE_SIZE - 1)
				break;
		}
	}

	return count;
}

static ssize_t rt5512_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val = 0, addr = 0;
	int i;

	for (i = 0; i < count; i++) {
		if (*(buf + i) <= '9' && *(buf + i) >= '0')
			addr = (addr << 4) | (*(buf + i)-'0');
		else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
			addr = (addr << 4) | ((*(buf + i) - 'a') + 0xa);
		else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
			addr = (addr << 4) | ((*(buf + i)-'A') + 0xa);
		else
			break;
	}

	for (i = i + 1 ; i < count; i++) {
		if (*(buf + i) <= '9' && *(buf + i) >= '0')
			val = (val << 4) | (*(buf + i) - '0');
		else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
			val = (val << 4) | ((*(buf + i) - 'a') + 0xa);
		else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
			val = (val << 4) | ((*(buf + i) - 'A') + 0xa);
		else
			break;
	}

	if (addr > RT5512_VENDOR_ID2 || val > 0xffff || val < 0)
		return count;

	if (i == count)
		pr_info("0x%02x = 0x%04x\n", addr,
			rt5512_read(rt5512_i2c, addr));
	else
		rt5512_write(rt5512_i2c, addr, val);

	return count;
}
static DEVICE_ATTR(rt5512_reg, 0664, rt5512_reg_show, rt5512_reg_store);

static ssize_t rt5512_device_read(struct file *file, char __user * buffer,
	size_t length, loff_t * offset)
{
	int ret;
	void __user  *buf = (void __user *)buffer;
	struct rt5512_soundtrigger_data *rt_5512 = (struct rt5512_soundtrigger_data*)file->private_data;
	if(rt_5512 == NULL) {
		printk("rt_5512 is NULL! pid:0x%08x\n",current->pid);
		return -ENODEV;
	}

	/* 1.voicetrigger interrupt  not happend or
		2.app do not want to abort caputring event or
		then wait in the queue */
	ret = wait_event_interruptible(voice_input_waitq, rt_5512->sound_trigger_event_state);
	spin_lock(&rt_5512->val_lock);
	ret = copy_to_user(buf, (void *)&rt_5512->sound_trigger_event_state, 1);
	spin_unlock(&rt_5512->val_lock);
	rt_5512->sound_trigger_event_state = SOUND_TRIGGER_NORMAL ;
	return ret;
}
#if 1
static int rt5512_invalidate_convert_write_regs(int reg, int val)
{
	int ret = 0;
	if (val < 0 || val > 0xffff || reg > RT5512_VENDOR_ID2) {
		pr_err("rt5512:   err  value  reg %d, val %d" ,reg ,val);
		ret = EINVAL;
	}
    if(reg == RT5512_DUMMY_RTK5) {
		pr_err("5512:neo0  val : %d \n" , val );
		//val = (255 - val) * 1800 / 255 ;
		pr_err("5512:neo1  val : %d \n" , val );
		val = (~val) + 0x001 ;
		pr_err("5512:neo2  val : %x \n" , val );

		rt5512_write(rt5512_i2c, reg, val);
	}

    if(reg == RT5512_DUMMY_RTK2) {
		printk("5512:neo3  val : %d \n" , val );
		val = val * 8192 / 100 ;
		printk("5512:neo4  val : %x \n" , val );

		rt5512_write(rt5512_i2c, reg, val);
	}
	return ret ;
}
#endif

static ssize_t device_write(struct file *file, const char __user * buffer,
	size_t length, loff_t * offset)
{
	char mode;
	u8* data ;
    u8  config_data[10];
	int ret = 0;
	struct rt5512_soundtrigger_data *rt_5512 = (struct rt5512_soundtrigger_data*)file->private_data;
	if(rt_5512 == NULL) {
		pr_err("rt_5512 is NULL! pid:0x%08x\n",current->pid);
		return -ENODEV;
	}
	if(length != 1 && length != 2 && length > 0) {
		data = vmalloc(length );
		ret = copy_from_user((void*)data, (void*)buffer, length);
		if(ret != 0) {
			pr_err("copy_from_user firmware data  err :%d %d " ,(int)length ,ret );
		}
		rt_5512->firmware_data = data;
		rt_5512->firmware_size = length ;
		length = rt5512_dsp_fast_load_fw(rt_5512);
		vfree(data);
	}
    else if(length == 2) {
        ret = copy_from_user(config_data, (void*)buffer, length);
		if(ret != 0) {
		    pr_err("copy_from_user firmware data  err :%d %d " ,(int)length ,ret );
			return -ENODEV;
		}
		pr_err("neo 5512: config_data %d  %d"  ,  config_data[0] , config_data[1]);
        ret = rt5512_invalidate_convert_write_regs(0xd4 ,  1200);
		if(ret != 0) {
		   // pr_err("rt5512_invalidate_convert_write_regs   err :%d %d " ,(int)length ,ret );
			return -EINVAL;
		}

        ret = rt5512_invalidate_convert_write_regs(0xd1 ,  35);
		if(ret != 0) {
		    //pr_err("rt5512_invalidate_convert_write_regs   err :%d %d " ,(int)length ,ret );
			return -EINVAL;
		}

    }
	else  {
		if (copy_from_user(&mode, buffer, 1))
		goto out;

		switch(mode - '0') {
		case RT5512_DSP:
			rt5512_dsp_start();
			break;
		case RT5512_DSP_ONESHOT:
			rt5512_dsp_one_shot();
			break;
		case RT5512_DSP_RELOAD:
		case RT5512_USE_AMIC:
		case RT5512_USE_DMIC:
			if ((mode - '0') != RT5512_DSP_RELOAD)
				rt5512_use_amic = ((mode - '0') == RT5512_USE_AMIC);
			rt5512_dsp_reload(rt_5512);
			break;

		case RT5512_NORMAL:
		default:
			rt5512_dsp_stop();
		}
	}
out:
	return length;
}

static int device_open(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev = NULL;
    struct rt5512_soundtrigger_data  *rt5512_st = NULL;
    miscdev = file->private_data;
    if (NULL == miscdev) {
        pr_err("open failed!miscdev == NULL!\n");
        return -EINVAL;
    }
    rt5512_st = container_of(miscdev, struct rt5512_soundtrigger_data, dev);
    if (NULL == rt5512_st) {
        pr_err("open failed!rt5512_st == NULL!\n");
        return -EINVAL;
    }
    file->private_data = (void*)rt5512_st;

    enable_irq(gpio_to_irq(rt5512_st->irq_gpio));

    return 0;
}


// release
static int device_release(struct inode *inode, struct file *file)
{
	struct rt5512_soundtrigger_data *rt_5512 = (struct rt5512_soundtrigger_data*)file->private_data;
	if(rt_5512 == NULL) {
		pr_err("rt_5512 is NULL! pid:0x%08x\n",current->pid);
		return -ENODEV;
	}

	disable_irq(gpio_to_irq(rt_5512->irq_gpio));
	file->private_data = NULL;

	return 0;
}


long rt5512_device_ioctl(struct file *file , unsigned int cmd , unsigned long arg)
{
	struct rt5512_soundtrigger_data *rt_5512 = (struct rt5512_soundtrigger_data*)file->private_data;
	if(rt_5512 == NULL) {
		pr_err("rt_5512 is NULL! pid:0x%08x\n",current->pid);
		return -ENODEV;
	}
	switch (cmd)
	{
		case SOUND_TRIGGER_STOPPING_CAPTURING :
		{
			/* app want to stop capturing , so wake up and exit the capturing thread */
			spin_lock(&rt_5512->val_lock);
			rt_5512->sound_trigger_event_state = SOUND_TRIGGER_STOPPING_CAPTURING;
			spin_unlock(&rt_5512->val_lock);

			wake_up_interruptible(&voice_input_waitq);
			break;
		}
	}

	return 0;
}

struct file_operations fops = {
	.owner    = THIS_MODULE,
	.read     = rt5512_device_read,
	.write    = device_write,
	.open     = device_open,
	.release  = device_release,
	#ifdef CONFIG_COMPAT
	.compat_ioctl    = rt5512_device_ioctl,
	#endif
};

static void rt5512_reload_work(struct work_struct *work)
{
	struct rt5512_soundtrigger_data *rt5512_st = container_of((struct delayed_work *)work, struct rt5512_soundtrigger_data, reload_work);

	rt5512_start_rest = false;
	rt5512_dsp_load_fw(rt5512_st);
}

static void rt5512_work_func(struct work_struct *work)
{
	struct rt5512_soundtrigger_data *rt5512_st = container_of(work, struct rt5512_soundtrigger_data, irq_work);
	pr_err("neo:rt5512_work_func start\n");

	/* voicetrigger interrupt happend so wake up the thread  */
	spin_lock(&rt5512_st->val_lock);
	rt5512_st->sound_trigger_event_state = SOUND_TRIGGER_EVENT_HAPPENED;
	spin_unlock(&rt5512_st->val_lock);
	wake_up_interruptible(&voice_input_waitq);


	/* clear the irq mask and restart to capture */
	rt5512_dsp_start();
	mutex_lock(&rt5512_st->irq_lock);
	enable_irq(gpio_to_irq(rt5512_st->irq_gpio));
	mutex_unlock(&rt5512_st->irq_lock);
	pr_err("neo:rt5512_work_func end\n");
}

static irqreturn_t rt5512_voice_trigger_irq_hander(int irq, void *dev_id)
{
	struct rt5512_soundtrigger_data *rt5512_st = dev_id;

	pr_err("neo:rt5512_voice_trigger_irq happend irq=%d\n" , irq);

	mutex_lock(&rt5512_st->irq_lock);
	disable_irq_nosync(irq);
	mutex_unlock(&rt5512_st->irq_lock);

    queue_work(rt5512_wq, &rt5512_st->irq_work);

	return IRQ_HANDLED;
}



static int rt5512_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *i2c_id)
{
	int ret ,index;
	struct device_node *np ;
	struct rt5512_soundtrigger_data *rt5512_st_data;


	ret = device_create_file(&i2c->dev, &dev_attr_rt5512_reg);
	if (ret < 0)
		pr_err("failed to add rt5512_reg sysfs files\n");

	rt5512_reset(i2c);
	rt5512_i2c = i2c;
	rt5512_st_data = kzalloc(sizeof(*rt5512_st_data), GFP_KERNEL);
	if (rt5512_st_data == NULL) {
		pr_err("Alloc GFP_KERNEL memory failed.");
		return -ENOMEM;
	}
	memset(rt5512_st_data, 0, sizeof(*rt5512_st_data));
	rt5512_st_data->client = rt5512_i2c;

#ifdef RT5512_USE_NATIVE
	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
		RT5512_FIRMWARE1, &i2c->dev, GFP_KERNEL, rt5512_st_data,
		rt5512_fw_loaded);
#else
	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
		"SMicBin.dat", &i2c->dev, GFP_KERNEL, rt5512_st_data,
		rt5512_fw_loaded);
#endif

	/*1.misc dev register*/
	rt5512_st_data->dev.minor = MISC_DYNAMIC_MINOR;
	rt5512_st_data->dev.name = "rt5512";
	rt5512_st_data->dev.fops = &fops;
	ret = misc_register(&rt5512_st_data->dev);
	if (ret) {
		pr_err("soundtrigger cannot register miscdev (%d)\n",ret);
		goto exit;
	}

	/*2. register irq*/
	np = i2c->dev.of_node;
	if (!np) {
		pr_err("%s No device node for voicetrigger!\n", __func__);
		goto exit1;
	}

	index = of_property_match_string(np, "gpio-names", "voicetrigger_irq");
	if (index < 0) {
		pr_err("%s :no match found for voicetrigger_irq gpio\n", __func__);
		goto exit1;
	}

	rt5512_st_data->irq_gpio = of_get_gpio_flags(np, index, NULL);
	if (ret < 0) {
		pr_err("%s :no get irq_gpio for voicetrigger_irq failed!\n", __func__);
		goto exit1;
	}

	gpio_direction_input(rt5512_st_data->irq_gpio);
	ret = devm_gpio_request(&i2c->dev,
		rt5512_st_data->irq_gpio, "voicetrigger_irq");
	if (ret < 0) {
		pr_err("%s :devm_gpio_request voicetrigger_irq failed!\n ret:%d", __func__ , ret);
		goto exit1;
	}

	ret = devm_request_threaded_irq(
		&i2c->dev,gpio_to_irq(rt5512_st_data->irq_gpio), NULL, rt5512_voice_trigger_irq_hander,
		IRQF_TRIGGER_HIGH , "voicetrigger_irq", rt5512_st_data);
	if (ret < 0) {
		pr_err("%s :devm_request_threaded_irq for voicetrigger_irq failed!\n ret:%d", __func__ , ret);
		goto exit1;
	}


	/* 3. download firmware */
	INIT_DELAYED_WORK(&rt5512_st_data->reload_work, rt5512_reload_work);

	/* 4. init some val*/
	mutex_init(&rt5512_st_data->irq_lock);
	spin_lock_init(&rt5512_st_data->val_lock);

	INIT_WORK(&rt5512_st_data->irq_work,rt5512_work_func);
	rt5512_wq = create_singlethread_workqueue("rt5512_wq");

	rt5512_st_data->sound_trigger_event_state = SOUND_TRIGGER_NORMAL;
	disable_irq_nosync(gpio_to_irq(rt5512_st_data->irq_gpio));

	return 0;
exit1:
	misc_deregister(&rt5512_st_data->dev);
exit:
	kfree(rt5512_st_data);
	return ret;
}

static int rt5512_i2c_remove(struct i2c_client *client)
{
    struct rt5512_soundtrigger_data *rt5512_st = i2c_get_clientdata(client);

	rt5512_dsp_stop();

	rt5512_i2c = NULL;

	if (rt5512_wq) {
		destroy_workqueue(rt5512_wq);
	}

	devm_free_irq(&client->dev, gpio_to_irq(rt5512_st->irq_gpio), rt5512_st);

	i2c_set_clientdata(client, rt5512_st);

	if (rt5512_st->dev.minor)
		misc_deregister(&rt5512_st->dev);

	kfree(rt5512_st);
	return 0;
}

static const struct i2c_device_id rt5512_i2c_id[] = {
	{ "rt5512", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt5512_i2c_id);

static const struct of_device_id rt5512_of_match[] = {
	{ .compatible = "realtek,rt5512", },
	{},
};


static struct i2c_driver rt5512_i2c_driver = {
	.driver = {
		.name = "rt5512",
		.owner = THIS_MODULE,
	},
	.probe = rt5512_i2c_probe,
	.remove = rt5512_i2c_remove,
	.id_table = rt5512_i2c_id,
};

static int __init rt5512_modinit(void)
{
	return i2c_add_driver(&rt5512_i2c_driver);
}
module_init(rt5512_modinit);

static void __exit rt5512_modexit(void)
{
	i2c_del_driver(&rt5512_i2c_driver);
}
module_exit(rt5512_modexit);



MODULE_DESCRIPTION("ALC5512 Smart DMIC Bridge Driver");
MODULE_AUTHOR("Oder Chiou <oder_chiou@realtek.com>");
MODULE_LICENSE("GPL v2");
