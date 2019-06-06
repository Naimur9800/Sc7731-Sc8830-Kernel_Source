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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sprd_otp.h>
#include "sprd_thm.h"

/* bits definitions for register THM_CTL */
#define BIT_THM_SET_RDY_STATUS       (BIT(5))
#define BIT_THM_SET_RDY              (BIT(4))
#define BIT_THM_SOFT_RESET           (BIT(3))
#define BIT_THM_MON_EN               (BIT(1))
#define BIT_THM_EN                   (BIT(0))

/* bits definitions for register THM_INT_CTL */
#define BIT_THM_OVERHEAT_EN              (BIT(9))
#define BIT_THM_OVERHEAT_ALARM_EN        (BIT(7))
#define THM_TRIP_THRESHOLD(val)          (val << 8)

/* thm efuse cal para */
#define TEMP_LOW -40000
#define TEMP_HIGH 120000
#define OTP_TEMP 120000
#define HOT_TEMP 75000
#define HOT2NOR_TEMP 65000
#define HIGHOFF_TEMP 55000
#define LOWOFF_TEMP 45000
#define COLD_TEMP 35000
#define RAW_DATA_LOW 0
#define RAW_DATA_HIGH 255
#define THM_FAST_DDR_READ 0x200
#define THM_FAST_GPU_READ 0x300


static struct thm_registers thermal_reg = {
	.thm_ctl = THM_CTL,
	.thm_int_ctl = THM_INT_CTL,
	.thm_int_sts = THM_INT_STS,
	.thm_int_raw_sts = THM_INT_RAW_STS,
	.thm_det_perold_l_16b = THM_DET_PERIOD_L16B,
	.thm_det_perold_h_4b = THM_DET_PERIOD_H4B,
	.thm_int_clr = THM_INT_CLR,
	.thm_overheat_hot_thres = THM_OVERHEAT_HOT_THRES,
	.thm_hot2nor_high_thres = THM_HOT2NOR_HIGH_THRES,
	.thm_low_cold_thres = THM_LOW_COLD_THRES,
	.thm_mon_peri = THM_MON_PERIOD,
	.thm_mon_ctl = THM_MON_CTL,
	.thm_temp0_read = THM_LAST_TEMPER0_READ,
	.thm_read_mask = THM_RAW_READ_MSK,
};

static inline void thm_reg_write(unsigned long reg, u32 bits, u32 clear_msk)
{
	writel_relaxed(((readl_relaxed((void __iomem *)reg) & ~clear_msk) |
			bits), ((void __iomem *)reg));
}

static inline u32 thm_reg_read(unsigned long reg)
{
	return readl_relaxed((void __iomem *)reg);
}

static int sprd_efuse_iwhale2_ddr_gpu_cal(struct sprd_thermal_zone *pzone)
{
	int cal_offset = 0;
	int mismatch = 0;
#if defined(CONFIG_OTP_SPRD_AP_EFUSE) || defined(CONFIG_OTP_SPRD_AP_IEFUSE)
	u32 data = sprd_ap_efuse_read(pzone->dsen_para->cal_efuse_blk);
#else
	u32 data = 0;
#endif
	int ratio = 1;
	int thm_cal = (data >> pzone->dsen_para->ratio_off_bit) & 0x003F;
	int rat_sign_bit = (data >> pzone->dsen_para->ratio_sign_bit) & 0x1;
	int k = pzone->dsen_para->cal_k;
	int b = pzone->dsen_para->cal_b;

	mismatch = ((data & 0x3F) << 2);
	if (data == 0)
		cal_offset = 32;
	else
		cal_offset = (data >> pzone->dsen_para->cal_efuse_bit) & 0x003F;

	if (rat_sign_bit == 1)
		ratio = 1000 - thm_cal;
	else
		ratio = 1000 + thm_cal;

	pzone->sensor_cal = (ratio * 1000) / k;
	pzone->sensor_cal_offset = (b / k) * 1000 + (cal_offset - 32) * 500;

	thm_reg_write((pzone->thm_base + THM_ANALOG_CTRL1), mismatch, 0);
	return 0;
}

static int sprd_efuse_sharkl2_cpu_gpu_cal(struct sprd_thermal_zone *pzone)
{
	int cal_offset = 0;
#if defined(CONFIG_OTP_SPRD_AP_EFUSE) || defined(CONFIG_OTP_SPRD_AP_IEFUSE)
	u32 data = sprd_ap_efuse_read(pzone->dsen_para->cal_efuse_blk);
#else
	u32 data = 0;
#endif
	int ratio = 1;
	int thm_cal = (data >> pzone->dsen_para->ratio_off_bit) & 0x007F;
	int rat_sign_bit = (data >> pzone->dsen_para->ratio_sign_bit) & 0x1;
	int k = pzone->dsen_para->cal_k;
	int b = pzone->dsen_para->cal_b;

	if (data == 0)
		cal_offset = 64;
	else
		cal_offset = (data >> pzone->dsen_para->cal_efuse_bit) & 0x007F;

	if (rat_sign_bit == 1)
		ratio = 1000 - thm_cal;
	else
		ratio = 1000 + thm_cal;

	pzone->sensor_cal = (k * ratio) / 1000;
	pzone->sensor_cal_offset = (b + (cal_offset - 64) * 500);

	return 0;
}

static int sprd_thm_efuse_cal_para(struct sprd_thermal_zone *pzone)
{

	if (pzone->dsen_para->algor_ver == 0)
		pzone->dsen_para->sensor_cal = sprd_efuse_iwhale2_ddr_gpu_cal;
	else if (pzone->dsen_para->algor_ver == 1)
		pzone->dsen_para->sensor_cal = sprd_efuse_sharkl2_cpu_gpu_cal;
	else {
		pr_err("Can not identify the thm type!\n");
		return -EINVAL;
	}
	return 0;
}

static int sprd_rawdata_to_temp_v1(struct sprd_thermal_zone *pzone)
{
	long temp_result;

	if (pzone->rawdata < RAW_DATA_LOW)
		pzone->rawdata = RAW_DATA_LOW;
	else if (pzone->rawdata > RAW_DATA_HIGH)
		pzone->rawdata = RAW_DATA_HIGH;
	else
		pr_debug("normal\n");
	temp_result =
	    ((pzone->sensor_cal * pzone->rawdata) - pzone->sensor_cal_offset);

	return temp_result;
}

static int sprd_temp_to_rawdata_v1(int temp, struct sprd_thermal_zone *pzone)
{
	unsigned int rawdata;

	if (temp < TEMP_LOW)
		temp = TEMP_LOW;
	else if (temp > TEMP_HIGH)
		temp = TEMP_HIGH;
	else
		pr_debug("normal\n");

	rawdata = ((temp + pzone->sensor_cal_offset) / pzone->sensor_cal);
	pr_info("rawdata =%d\n", rawdata);

	return rawdata >= RAW_DATA_HIGH ? (RAW_DATA_HIGH - 1) : rawdata;
}

static int sprd_thm_sensor_temp_read(struct sprd_thermal_zone *pzone, int *temp)
{
	int sensor_temp;

	if (pzone->thm_type == SPRD_R1P0_THM) {
		if (pzone->id == 1) {
			pzone->rawdata =
			    thm_reg_read(pzone->thm_base + THM_FAST_DDR_READ);
		} else if (pzone->id == 2) {
			pzone->rawdata =
			    thm_reg_read(pzone->thm_base + THM_FAST_GPU_READ);
		} else {
			pr_debug("no support\n");
		}
	}
	pzone->rawdata = pzone->rawdata & pzone->thmreg->thm_read_mask;

	if (pzone->ready_flag) {
		sensor_temp = sprd_rawdata_to_temp_v1(pzone);
		pzone->lasttemp = sensor_temp;
		*temp = sensor_temp;
		pr_debug("sensor id:%d, rawdata:0x%x, temp:%d\n",
		       pzone->id, pzone->rawdata, *temp);
	} else {
		*temp = pzone->lasttemp;
		pr_debug("sensor id:%d, last temp:%d\n", pzone->id,
		       pzone->lasttemp);
	}
	return 0;
}

static void sensor_rdy_wait_clear(struct sprd_thermal_zone *pzone)
{
	int cnt = 70;
	int status;
	int off_bit;
	int bit_status;
	unsigned long reg;

	reg = pzone->thm_base + pzone->thmreg->thm_ctl;
	status = thm_reg_read(reg);
	if (pzone->thm_type == SPRD_R1P0_THM)
		off_bit = 4;
	else
		off_bit = 5;

	bit_status = (status >> off_bit) & 0x1;
	while (bit_status && cnt--) {
		status = thm_reg_read(reg);
		bit_status = (status >> off_bit) & 0x1;
		udelay(10);
	}
	if (cnt == -1)
		pr_err("set ready status timeout 0x%x\n", thm_reg_read(reg));
}

static void sprd_thm_sensor_cal(struct sprd_thermal_zone *pzone)
{
	int ret = 0;

	pr_info("thm init id:%d,base 0x%lx\n", pzone->id, pzone->thm_base);
	ret = pzone->dsen_para->sensor_cal(pzone);
	pr_info("thm cal =%d,offset =%d ret =%d\n", pzone->sensor_cal,
		pzone->sensor_cal_offset, ret);
}

static void sprd_thm_sensor_trip_config(struct sprd_thermal_zone *pzone)
{
	pzone->otp_rawdata = sprd_temp_to_rawdata_v1(pzone->otp_temp, pzone);
	pzone->hot_rawdata = sprd_temp_to_rawdata_v1(pzone->hot_temp, pzone);
	pzone->hot2nor_rawdata =
	    sprd_temp_to_rawdata_v1(pzone->hot2nor_temp, pzone);
	pzone->highoff_rawdata =
	    sprd_temp_to_rawdata_v1(pzone->highoff_temp, pzone);
	pzone->lowoff_rawdata =
	    sprd_temp_to_rawdata_v1(pzone->lowoff_temp, pzone);
	pzone->cold_rawdata = sprd_temp_to_rawdata_v1(pzone->cold_temp, pzone);

	pr_info("otp rawdata 0x%x, hot rawdata 0x%x,hot2nor rawdata 0x%x\n",
		pzone->otp_rawdata, pzone->hot_rawdata, pzone->hot2nor_rawdata);
	pr_info("highoff rawdata 0x%x, lowoff rawdata 0x%x,cold rawdata 0x%x\n",
		pzone->highoff_rawdata, pzone->lowoff_rawdata,
		pzone->cold_rawdata);
}

static void sprd_thm_hw_init_sensor(struct sprd_thermal_zone *pzone)
{
	unsigned int mask;

	mask = BIT_THM_OVERHEAT_EN | BIT_THM_OVERHEAT_ALARM_EN;
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_int_ctl), 0, ~0u);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_int_clr), ~0u, 0);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_det_perold_l_16b),
		      0x10, 0xffff);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_mon_ctl), 0x9, 0);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_mon_peri),
		      0x40, 0xffff);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_overheat_hot_thres),
		      (THM_TRIP_THRESHOLD(pzone->otp_rawdata)
		      | pzone->hot_rawdata), 0xffff);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_hot2nor_high_thres),
		      (THM_TRIP_THRESHOLD(pzone->hot2nor_rawdata)
		      | pzone->highoff_rawdata), 0xffff);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_low_cold_thres),
		      (THM_TRIP_THRESHOLD(pzone->lowoff_rawdata)
		      | pzone->cold_rawdata), 0xffff);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_int_ctl), mask, 0);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_ctl),
		      BIT_THM_MON_EN, 0);
	sensor_rdy_wait_clear(pzone);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_ctl),
		      BIT_THM_SET_RDY, 0);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_int_clr), ~0u, 0);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_ctl), BIT_THM_EN,
		      0);
}

static void sprd_thm_sensor_hw_init(struct sprd_thermal_zone *pzone)
{
	sprd_thm_hw_init_sensor(pzone);
}

static int sprd_hw_sensor_suspend(struct sprd_thermal_zone *pzone)
{
	unsigned int mask;

	mask = BIT_THM_OVERHEAT_EN | BIT_THM_OVERHEAT_ALARM_EN;
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_int_ctl), 0, mask);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_ctl),
		      0, BIT_THM_MON_EN);
	sensor_rdy_wait_clear(pzone);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_ctl),
		      BIT_THM_SET_RDY, 0);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_int_clr), ~0u, 0);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_ctl), 0,
		      BIT_THM_EN);
	return 0;
}

static int sprd_hw_sensor_resume(struct sprd_thermal_zone *pzone)
{
	unsigned int mask;

	mask = BIT_THM_OVERHEAT_EN | BIT_THM_OVERHEAT_ALARM_EN;
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_int_ctl), mask, 0);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_ctl),
		      BIT_THM_MON_EN, 0);
	sensor_rdy_wait_clear(pzone);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_ctl),
		      BIT_THM_SET_RDY, 0);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_int_clr), ~0u, 0);
	thm_reg_write((pzone->thm_base + pzone->thmreg->thm_ctl), BIT_THM_EN,
		      0);
	return 0;
}

static int sprd_thm_sensor_suspend(struct sprd_thermal_zone *pzone)
{
	pr_info("thm suspend id:%d\n", pzone->id);
	return 0;
}

static int sprd_thm_sensor_resume(struct sprd_thermal_zone *pzone)
{
	return 0;
}

static void sprd_thm_resume_flag_work(struct work_struct *work)
{
	struct sprd_thermal_zone *pzone;

	pzone =
	    container_of(work, struct sprd_thermal_zone,
			 thm_resume_flag_work.work);
	pzone->ready_flag = 1;
}

static const struct of_device_id ap_thermal_of_match[] = {
	{.compatible = "sprd,r1p0-ithm", .data = (void *)SPRD_R1P0_THM},
	{.compatible = "sprd,r2p0-ithm", .data = (void *)SPRD_R2P0_THM},
	{},
};

static struct thm_handle_ops sprd_ap_ops = {
	.hw_init = sprd_thm_sensor_hw_init,
	.read_temp = sprd_thm_sensor_temp_read,
	.suspend = sprd_thm_sensor_suspend,
	.resume = sprd_thm_sensor_resume,
};

static struct sensor_info *thm_parse_dt(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct sensor_info *para = NULL;
	int ret;

	para = devm_kzalloc(&pdev->dev, sizeof(*para), GFP_KERNEL);
	if (!para)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32(np, "algor_ver", &para->algor_ver);
	if (ret) {
		dev_err(&pdev->dev, "parse algor_ver err\n");
		return ERR_PTR(ret);
	}

	ret = of_property_read_u32(np, "cal_k", &para->cal_k);
	if (ret) {
		dev_err(&pdev->dev, "parse cal_k err\n");
		return ERR_PTR(ret);
	}

	ret = of_property_read_u32(np, "cal_b", &para->cal_b);
	if (ret) {
		dev_err(&pdev->dev, "parse cal_b err\n");
		return ERR_PTR(ret);
	}

	ret = of_property_read_u32(np, "cal_efuse_blk", &para->cal_efuse_blk);
	if (ret) {
		dev_err(&pdev->dev, "parse cal_efuse_blk err\n");
		return ERR_PTR(ret);
	}

	ret = of_property_read_u32(np, "cal_efuse_bit", &para->cal_efuse_bit);
	if (ret) {
		dev_err(&pdev->dev, "parse cal_efuse_bit err\n");
		return ERR_PTR(ret);
	}

	ret = of_property_read_u32(np, "ratio_off_bit", &para->ratio_off_bit);
	if (ret) {
		dev_err(&pdev->dev, "parse ratio_off_bit err\n");
		return ERR_PTR(ret);
	}

	ret = of_property_read_u32(np, "ratio_sign_bit", &para->ratio_sign_bit);
	if (ret) {
		dev_err(&pdev->dev, "parse ratio_sign_bit err\n");
		return ERR_PTR(ret);
	}

	return para;
};

static int sprd_ap_thm_probe(struct platform_device *pdev)
{
	struct sprd_thermal_zone *pzone = NULL;
	struct resource *res;
	struct clk *clk;
	struct regmap *thm_reg;
	const struct of_device_id *of_id;
	enum sprd_ap_thm_type thm_type;
	int ret;
	int sensor_id = 0;
	int power_down = 0;
	int otptemp = OTP_TEMP;
	unsigned long ap_thm_base;
	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "device node not found\n");
		return -ENODEV;
	}

	of_id = of_match_node(ap_thermal_of_match, pdev->dev.of_node);
	if (!of_id) {
		dev_err(&pdev->dev, "get device of id failed!\n");
		return -ENODEV;
	}
	thm_type = (enum sprd_ap_thm_type)of_id->data;
	dev_info(&pdev->dev, "ap thm probe start %d\n", thm_type);

	sensor_id = of_alias_get_id(np, "thm-sensor");
	if (sensor_id == -ENODEV) {
		dev_err(&pdev->dev, "fail to get id\n");
		return -ENODEV;
	}

	clk = of_clk_get_by_name(pdev->dev.of_node, "enable");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev,
			"pmic efuse can't get the clock dts config: enable\n");
		return -EINVAL;
	}

	thm_reg = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						  "sprd,syscon-enable");
	if (IS_ERR(thm_reg)) {
		dev_err(&pdev->dev, "ap thm get aon syscon failed!\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ap_thm_base = (unsigned long)devm_ioremap_resource(&pdev->dev, res);
	if (!ap_thm_base) {
		dev_err(&pdev->dev, "thermal ioremap failed!\n");
		return -ENOMEM;
	}

	ret = of_property_read_s32(np, "otp-temp", &otptemp);
	if (ret) {
		dev_err(&pdev->dev, "parse otp_temp err\n");
		return ret;
	}

	ret = of_property_read_s32(np, "power-down", &power_down);
	if (ret) {
		dev_err(&pdev->dev, "parse power_status err\n");
		return ret;
	}

	pzone = devm_kzalloc(&pdev->dev, sizeof(*pzone), GFP_KERNEL);
	if (!pzone)
		return -ENOMEM;

	pzone->dsen_para = thm_parse_dt(pdev);
	if (IS_ERR_OR_NULL(pzone->dsen_para))
		return -ENOMEM;

	mutex_init(&pzone->th_lock);
	pzone->id = sensor_id;
	pzone->thm_type = thm_type;
	pzone->thm_base = ap_thm_base;
	pzone->thm_glb = thm_reg;
	pzone->clk = clk;
	pzone->otp_temp = otptemp;
	pzone->hot_temp = HOT_TEMP;
	pzone->hot2nor_temp = HOT2NOR_TEMP;
	pzone->highoff_temp = HIGHOFF_TEMP;
	pzone->lowoff_temp = LOWOFF_TEMP;
	pzone->cold_temp = COLD_TEMP;
	pzone->power_down = power_down;
	pzone->ops = &sprd_ap_ops;
	pzone->dev = &pdev->dev;
	pzone->thmreg = &thermal_reg;

	ret = sprd_thm_efuse_cal_para(pzone);
	if (ret) {
		dev_err(&pdev->dev, "parse thm efuse cal err\n");
		return ret;
	}

	clk_prepare_enable(pzone->clk);

	INIT_DELAYED_WORK(&pzone->thm_resume_flag_work,
			  sprd_thm_resume_flag_work);
	sprd_thm_sensor_cal(pzone);
	sprd_thm_sensor_trip_config(pzone);
	sprd_thm_sensor_hw_init(pzone);

	ret = sprd_thermal_init(pzone);
	if (ret) {
		dev_err(&pdev->dev, " pzone sw init error id =%d\n", pzone->id);
		goto clk_fail;
	}
	pzone->ready_flag = 1;

	platform_set_drvdata(pdev, pzone);
	dev_info(&pdev->dev, "sprd ap thm probe end\n");
	return 0;

clk_fail:
	clk_disable_unprepare(pzone->clk);
	return ret;
}

static int sprd_ap_thm_remove(struct platform_device *pdev)
{
	struct sprd_thermal_zone *pzone = platform_get_drvdata(pdev);

	sprd_thermal_remove(pzone);
	return 0;
}

static int sprd_ap_thm_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct sprd_thermal_zone *pzone = platform_get_drvdata(pdev);

	flush_delayed_work(&pzone->thm_resume_flag_work);
	pzone->ready_flag = 0;
	if (pzone->thm_type == SPRD_R1P0_THM)
		pr_info("sprd ap thm suspend\n");
	else
		sprd_hw_sensor_suspend(pzone);

	clk_disable_unprepare(pzone->clk);
	return 0;
}

static int sprd_ap_thm_resume(struct platform_device *pdev)
{
	struct sprd_thermal_zone *pzone = platform_get_drvdata(pdev);

	clk_prepare_enable(pzone->clk);
	if (pzone->thm_type == SPRD_R1P0_THM)
		pr_info("sprd ap thm resume\n");
	else{
		if (pzone->power_down == 0)
			sprd_hw_sensor_resume(pzone);
		else
			sprd_thm_hw_init_sensor(pzone);
	}

	queue_delayed_work(system_power_efficient_wq,
			   &pzone->thm_resume_flag_work, (HZ * 1));
	return 0;
}

static struct platform_driver sprd_ap_thermal_driver = {
	.probe = sprd_ap_thm_probe,
	.suspend = sprd_ap_thm_suspend,
	.resume = sprd_ap_thm_resume,
	.remove = sprd_ap_thm_remove,
	.driver = {
		   .name = "ap-thermal",
		   .of_match_table = of_match_ptr(ap_thermal_of_match),
		   },
};

static int __init sprd_ap_thermal_init(void)
{
	return platform_driver_register(&sprd_ap_thermal_driver);
}

static void __exit sprd_ap_thermal_exit(void)
{
	platform_driver_unregister(&sprd_ap_thermal_driver);
}

device_initcall_sync(sprd_ap_thermal_init);
module_exit(sprd_ap_thermal_exit);

MODULE_AUTHOR("Freeman Liu <freeman.liu@spreadtrum.com>");
MODULE_DESCRIPTION("sprd thermal driver");
MODULE_LICENSE("GPL");
