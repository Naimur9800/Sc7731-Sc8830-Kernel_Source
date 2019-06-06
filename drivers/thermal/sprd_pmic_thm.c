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

#include <linux/init.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/stat.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/spi/spi-sprd-adi.h>
#include "sprd_thm.h"

#define A_HIGH_TAB_SZ 8
#define A_LOW_TAB_SZ  16
#define HIGH_BITS_OFFSET   4
#define LOCAL_SENSOR_ADDR_OFF 0x100
#define A_RAW_TEMP_RANGE_MSK  0x7F
#define THM_SENOR_NUM 8
#define BIT_SEN_MON_EN                    BIT(0)
#define BIT_SEN_SET_RDY                   BIT(3)

static const short a_temp_search_high_152nm[A_HIGH_TAB_SZ] =
    { -56, -29, -2, 26, 53, 79, 106, 133 };
static const short a_temp_search_low_152nm[A_LOW_TAB_SZ] =
    { 0, 2, 3, 5, 7, 8, 10, 11, 13, 15, 17, 18, 20, 21, 23, 25 };

struct regmap *pmic_glb_reg;

struct pmic_thm_info {
	struct sensor_ops *ops;
	struct sensor_registers *reg;
	struct sensor_info sensor[];
};

static struct sensor_registers thm_thm_registers = {
	.thm_ctrl = THM_THM_CTRL,
	.thm_int_ctrl = THM_THM_INT_CTRL,
	.sen_ctrl = THM_SEN_CTRL,
	.sen_det_peri = THM_SEN_DET_PERI,
	.sen_int_ctrl = THM_SEN_INT_CTRL,
	.sen_int_sts = THM_SEN_INT_STS,
	.sen_int_raw_sts = THM_SEN_INT_RAW_STS,
	.sen_int_clr = THM_SEN_INT_CLR,
	.sen_mon_peri = THM_SEN_MON_PERI,
	.sen_mon_ctl = THM_SEN_MON_CTL,
	.sen_temp0_read = THM_SEN_TEMPER0_READ,
	.sen_read_status = THM_SEN_READ_STATUS,
	.sen_read_mask = THM_SEN_RAW_READ_MSK,
};

static inline int pmic_thm_reg_read(unsigned long reg)
{
	unsigned int val;

	regmap_read(pmic_glb_reg, reg, &val);
	return val;
}

static inline int pmic_thm_reg_write(unsigned long reg, unsigned int val)
{
	return regmap_write(pmic_glb_reg, reg, val);
}

static inline int pmic_thm_reg_set(unsigned long reg, unsigned int mask,
				   unsigned int bits)
{
	return regmap_update_bits(pmic_glb_reg, reg, mask, bits);
}

static inline int pmic_thm_reg_clr(unsigned long reg, unsigned int mask,
				   unsigned int bits)
{
	return regmap_update_bits(pmic_glb_reg, reg, mask, ~bits);
}

static int sprd_rawdata_to_temp_v1(struct sprd_thermal_zone *pzone)
{
	u32 temp;
	const short *high_tab = a_temp_search_high_152nm;
	const short *low_tab = a_temp_search_low_152nm;

	temp = high_tab[(pzone->rawdata >> HIGH_BITS_OFFSET) & 0x07] +
	    low_tab[pzone->rawdata & 0x0F];
	temp = (temp + pzone->sensor_cal_offset) * 1000;
	return temp;
}

static int sprd_pmic_sensor_temp_read(struct sprd_thermal_zone *pzone,
				      int *temp)
{
	int sensor_temp;

	pzone->rawdata =
	    pmic_thm_reg_read(pzone->thm_base + pzone->reg->sen_temp0_read);
	pzone->rawdata = pzone->rawdata & A_RAW_TEMP_RANGE_MSK;
	if (pzone->ready_flag) {
		sensor_temp = sprd_rawdata_to_temp_v1(pzone);
		pzone->lasttemp = sensor_temp;
		*temp = sensor_temp;
		pr_info("sensor id:%d, rawdata:0x%x, temp:%d\n",
			pzone->id, pzone->rawdata, sensor_temp);
	} else {
		pr_info("sensor id:%d, last temp:%d\n", pzone->id,
			pzone->lasttemp);
		*temp = pzone->lasttemp;
	}

	return 0;
}

static void sensor_rdy_wait_clear(unsigned long reg)
{
	int cnt = 40;
	int status = pmic_thm_reg_read(reg);
	int bit_status = (status >> 3) & 0x1;

	while (bit_status && cnt--) {
		status = pmic_thm_reg_read(reg);
		bit_status = (status >> 3) & 0x1;
		udelay(10);
	}
	if (cnt == -1) {
		pr_info("thm sensor timeout 0x%x\n", pmic_thm_reg_read(reg));
		pmic_thm_reg_clr((reg), BIT_SEN_SET_RDY, BIT_SEN_SET_RDY);
		udelay(200);
		pmic_thm_reg_set((reg), BIT_SEN_SET_RDY, BIT_SEN_SET_RDY);
	}
}

static void sensor_rdy_status(unsigned long reg)
{
	int status = 0;
	int bit_status = 0;

	udelay(200);
	status = pmic_thm_reg_read(reg);
	bit_status = (status >> 3) & 0x1;
	if (bit_status) {
		pr_info("thm sensor rdy status need clear\n");
		pmic_thm_reg_clr((reg), BIT_SEN_SET_RDY, BIT_SEN_SET_RDY);
		udelay(200);
	}
}

static void sprd_thm_global_hw_init(struct sprd_thermal_zone *pzone)
{
	pmic_thm_reg_set(ANA_REG_GLB_RTC_CLK_EN,
			 BIT_RTC_THMA_AUTO_EN | BIT_RTC_THMA_EN |
			 BIT_RTC_THM_EN,
			 BIT_RTC_THMA_AUTO_EN | BIT_RTC_THMA_EN |
			 BIT_RTC_THM_EN);
}

static void sprd_pmic_sensor_hw_init(struct sprd_thermal_zone *pzone)
{
	pr_info("sprd pmic thm hw init id:%d,base 0x%lx \n",
		pzone->id, pzone->thm_base);
	pmic_thm_reg_set(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_THM_EN,
			 BIT_ANA_THM_EN);
	sprd_thm_global_hw_init(pzone);
	pzone->sensor_cal_offset = 0;
	pmic_thm_reg_write((pzone->thm_base + pzone->reg->thm_ctrl), 0x1);
	pmic_thm_reg_write((pzone->thm_base + pzone->reg->sen_int_ctrl), 0);
	pmic_thm_reg_write((pzone->thm_base + pzone->reg->sen_int_clr), 0);
	pmic_thm_reg_write((pzone->thm_base + pzone->reg->sen_det_peri),
			   0x2000);
	pmic_thm_reg_set((pzone->thm_base + pzone->reg->sen_ctrl),
			 BIT_SEN_MON_EN, BIT_SEN_MON_EN);
	sensor_rdy_status(pzone->thm_base + pzone->reg->sen_ctrl);
	pmic_thm_reg_set((pzone->thm_base + pzone->reg->sen_ctrl),
			 BIT_SEN_SET_RDY, BIT_SEN_SET_RDY);
	sensor_rdy_wait_clear(pzone->thm_base + pzone->reg->sen_ctrl);
}

static int sprd_pmic_sensor_hw_suspend(struct sprd_thermal_zone *pzone)
{
	pmic_thm_reg_clr((pzone->thm_base + pzone->reg->sen_ctrl),
			 BIT_SEN_MON_EN, BIT_SEN_MON_EN);
	sensor_rdy_status(pzone->thm_base + pzone->reg->sen_ctrl);
	pmic_thm_reg_set((pzone->thm_base + pzone->reg->sen_ctrl),
			 BIT_SEN_SET_RDY, BIT_SEN_SET_RDY);
	sensor_rdy_wait_clear(pzone->thm_base + pzone->reg->sen_ctrl);
	return 0;
}

static int sprd_pmic_sensor_hw_resume(struct sprd_thermal_zone *pzone)
{
	sprd_thm_global_hw_init(pzone);
	pmic_thm_reg_set((pzone->thm_base + pzone->reg->sen_ctrl),
			 BIT_SEN_MON_EN, BIT_SEN_MON_EN);
	sensor_rdy_status(pzone->thm_base + pzone->reg->sen_ctrl);
	pmic_thm_reg_set((pzone->thm_base + pzone->reg->sen_ctrl),
			 BIT_SEN_SET_RDY, BIT_SEN_SET_RDY);
	sensor_rdy_wait_clear(pzone->thm_base + pzone->reg->sen_ctrl);
	return 0;
}

static struct sensor_ops gen_sensor_ops = {
	.sensor_init = sprd_pmic_sensor_hw_init,
	.sensor_read = sprd_pmic_sensor_temp_read,
	.sensor_suspend = sprd_pmic_sensor_hw_suspend,
	.sensor_resume = sprd_pmic_sensor_hw_resume,
};

static void sprd_thm_sensor_hw_init(struct sprd_thermal_zone *pzone)
{
	pzone->sen_ops->sensor_init(pzone);
}

static int sprd_thm_sensor_suspend(struct sprd_thermal_zone *pzone)
{
	pzone->sen_ops->sensor_suspend(pzone);
	return 0;
}

static int sprd_thm_sensor_resume(struct sprd_thermal_zone *pzone)
{
	pzone->sen_ops->sensor_resume(pzone);
	return 0;
}

static void sprd_thm_resume_flag_work(struct work_struct *work)
{
	struct sprd_thermal_zone *pzone =
	    container_of(work, struct sprd_thermal_zone,
			 thm_resume_flag_work.work);

	pzone->ready_flag = 1;
}

struct thm_handle_ops sprd_pmic_ops = {
	.hw_init = sprd_thm_sensor_hw_init,
	.read_temp = sprd_pmic_sensor_temp_read,
	.suspend = sprd_thm_sensor_suspend,
	.resume = sprd_thm_sensor_resume,
};

static const struct pmic_thm_info pmic_thm_gen_info = {
	.reg = &thm_thm_registers,
	.ops = &gen_sensor_ops,
};

static const struct of_device_id pmic_thermal_of_match[] = {
	{
	    .compatible = "sprd,sc2723-thermal",
	    .data = (void *)&pmic_thm_gen_info,
	},
	{
	    .compatible = "sprd,sc2731-thermal",
	    .data = (void *)&pmic_thm_gen_info,
	},
	{}
};

static int sprd_pmic_thm_probe(struct platform_device *pdev)
{
	struct sprd_thermal_zone *pzone = NULL;
	const struct of_device_id *of_id;
	const struct pmic_thm_info *info;
	int sensor_id = 0;
	int ret;
	u32 value;
	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "device node not found\n");
		return -ENODEV;
	}

	sensor_id = of_alias_get_id(np, "thm-sensor");
	if (sensor_id == -ENODEV) {
		dev_err(&pdev->dev, "fail to get id\n");
		return -ENODEV;
	}
	dev_info(&pdev->dev, "sprd pmic sensor id %d\n", sensor_id);

	of_id = of_match_node(pmic_thermal_of_match, pdev->dev.of_node);
	if (!of_id) {
		dev_err(&pdev->dev, "get device id failed!\n");
		return -ENODEV;
	}
	info = of_id->data;

	pmic_glb_reg = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pmic_glb_reg) {
		dev_err(&pdev->dev,
			"%s :fail regmap property for pmic thm .", __func__);
		ret = -ENODEV;
	}

	ret = of_property_read_u32(np, "reg", &value);
	if (ret) {
		dev_err(&pdev->dev, "%s :no reg of property for pmic efuse!",
			__func__);
		return ret;
	}

	pzone = devm_kzalloc(&pdev->dev, sizeof(*pzone), GFP_KERNEL);
	if (!pzone)
		return -ENOMEM;
	mutex_init(&pzone->th_lock);

	pzone->thm_base = (unsigned long)value;
	pzone->thm_glb = pmic_glb_reg;
	pzone->id = sensor_id;
	pzone->ops = &sprd_pmic_ops;
	pzone->dev = &pdev->dev;
	pzone->reg = info->reg;
	pzone->sen_ops = info->ops;
	strlcpy(pzone->name, np->name, sizeof(pzone->name));

	INIT_DELAYED_WORK(&pzone->thm_resume_flag_work,
			  sprd_thm_resume_flag_work);
	sprd_thm_sensor_hw_init(pzone);

	ret = sprd_thermal_init(pzone);
	if (ret) {
		dev_err(&pdev->dev, " pzone sw init error id =%d\n", pzone->id);
		return ret;
	}
	pzone->ready_flag = 1;

	platform_set_drvdata(pdev, pzone);
	dev_info(&pdev->dev, "sprd pmic thm probe end\n");
	return 0;
}

static int sprd_pmic_thm_remove(struct platform_device *pdev)
{
	struct sprd_thermal_zone *pzone = platform_get_drvdata(pdev);

	sprd_thermal_remove(pzone);
	return 0;
}


static int sprd_pmic_thm_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct sprd_thermal_zone *pzone = platform_get_drvdata(pdev);

	flush_delayed_work(&pzone->thm_resume_flag_work);
	flush_delayed_work(&pzone->resume_delay_work);
	pzone->ops->suspend(pzone);
	pzone->ready_flag = 0;
	return 0;
}

static int sprd_pmic_thm_resume(struct platform_device *pdev)
{
	struct sprd_thermal_zone *pzone = platform_get_drvdata(pdev);

	queue_delayed_work(system_power_efficient_wq,
		&pzone->resume_delay_work, (HZ * 1));
	queue_delayed_work(system_power_efficient_wq,
		&pzone->thm_resume_flag_work, (HZ * 3));
	return 0;
}

static struct platform_driver sprd_pmic_thermal_driver = {
	.probe = sprd_pmic_thm_probe,
	.suspend = sprd_pmic_thm_suspend,
	.resume = sprd_pmic_thm_resume,
	.remove = sprd_pmic_thm_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "pmic-thermal",
		   .of_match_table = of_match_ptr(pmic_thermal_of_match),
		   },
};

static int __init sprd_pmic_thermal_init(void)
{
	return platform_driver_register(&sprd_pmic_thermal_driver);
}

static void __exit sprd_pmic_thermal_exit(void)
{
	platform_driver_unregister(&sprd_pmic_thermal_driver);
}

device_initcall_sync(sprd_pmic_thermal_init);
module_exit(sprd_pmic_thermal_exit);

MODULE_AUTHOR("Freeman Liu <freeman.liu@spreadtrum.com>");
MODULE_DESCRIPTION("sprd pmic thermal driver");
MODULE_LICENSE("GPL");
