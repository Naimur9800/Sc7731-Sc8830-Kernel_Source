/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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

#include <linux/err.h>
#include <linux/io.h>
#include <linux/iio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include "sprd_thm.h"

static int sprd_vol_to_temp_tab(int x, int n, struct vol_temp_table *tab)
{
	int index;
	int y;

	if (x >= tab[0].x)
		y = tab[0].y;
	else if (x <= tab[n - 1].x)
		y = tab[n - 1].y;
	else {
		/*  find interval */
		for (index = 1; index < n; index++)
			if (x > tab[index].x)
				break;
		/*  interpolate */
		y = (tab[index - 1].y - tab[index].y) * (x - tab[index].x)
		    * 2 / (tab[index - 1].x - tab[index].x);
		y = (y + 1) / 2;
		y += tab[index].y;
	}
	return y;
}

static int temp_sensor_vol_to_temp(struct sprd_thermal_zone *pzone, int val)
{
	return sprd_vol_to_temp_tab(val, pzone->pthm_config->temp_tab_size,
				    pzone->pthm_config->temp_tab);
}

static uint32_t temp_sensor_adc_to_vol(struct iio_channel *channel)
{
	int err;
	uint32_t val;

	err = iio_read_channel_processed(channel, &val);
	if (err < 0)
		return err;
	return val;
}

static int sprd_temp_sensor_read(struct sprd_thermal_zone *pzone, int *temp)
{
	int vol = 0;
	int sensor_temp = 0;

	vol = temp_sensor_adc_to_vol(pzone->pthm_config->channel_temp);
	sensor_temp = temp_sensor_vol_to_temp(pzone, vol);
	*temp = sensor_temp * 100;
	pr_info("sensor id:%d,vol:0x%x, temp:%d\n", pzone->id, vol, *temp);

	return 0;
}

static void sprd_temp_sensor_init(struct sprd_thermal_zone *pzone)
{
	pr_info("sprd_temp_sensor_init\n");
}

static int sprd_temp_sensor_suspend(struct sprd_thermal_zone *pzone)
{
	pr_info("sprd_temp_sensor_suspend\n");
	return 0;
}

static int sprd_temp_sensor_resume(struct sprd_thermal_zone *pzone)
{
	pr_info("sprd_temp_sensor_resume\n");
	return 0;
}

struct thm_handle_ops sprd_boardthm_ops = {
	.hw_init = sprd_temp_sensor_init,
	.read_temp = sprd_temp_sensor_read,
	.suspend = sprd_temp_sensor_suspend,
	.resume = sprd_temp_sensor_resume,
};

static struct temp_sen_config *sprd_temp_sen_parse_dt(struct device *dev)
{
	int ret = 0, i = 0;
	int temp = 0;
	int temp_tab_size = 0;
	struct temp_sen_config *pconfig;
	struct device_node *np = dev->of_node;

	pconfig = devm_kzalloc(dev, sizeof(*pconfig), GFP_KERNEL);
	if (!pconfig)
		return NULL;

	ret = of_property_read_u32(np, "temp-tab-size", &temp_tab_size);
	if (ret) {
		dev_err(dev, "fail to get  temp_tab_size\n");
		return NULL;
	}

	pconfig->temp_tab_size = temp_tab_size;
	pconfig->temp_tab =
	    devm_kzalloc(dev,
			 sizeof(struct vol_temp_table) *
			 pconfig->temp_tab_size - 1, GFP_KERNEL);
	if (!pconfig->temp_tab)
		return NULL;

	for (i = 0; i < pconfig->temp_tab_size - 1; i++) {
		ret = of_property_read_u32_index(np, "temp-tab-val", i,
						 &pconfig->temp_tab[i].x);
		if (ret) {
			dev_err(dev, "fail to get temp-tab-va\n");
			return NULL;
		}
		ret = of_property_read_u32_index(np, "temp-tab-temp", i, &temp);
		if (ret) {
			dev_err(dev, "fail to get temp-tab-temp\n");
			return NULL;
		}
		pconfig->temp_tab[i].y = temp - 1000;
	}

	return pconfig;
}

static int sprd_temp_sensor_probe(struct platform_device *pdev)
{
	int ret = 0, sensor_id = 0;
	struct sprd_thermal_zone *pzone = NULL;
	struct temp_sen_config *pconfig = NULL;
	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "device node not found\n");
		return -EINVAL;
	}

	sensor_id = of_alias_get_id(np, "thm-sensor");
	if (sensor_id == -ENODEV) {
		dev_err(&pdev->dev, "fail to get id\n");
		return -ENODEV;
	}
	pr_info("sprd board sensor probe id %d\n", sensor_id);

	pconfig = sprd_temp_sen_parse_dt(&pdev->dev);
	if (!pconfig) {
		dev_err(&pdev->dev, "not found ptrips\n");
		return -EINVAL;
	}
	pconfig->channel_temp = iio_channel_get(&pdev->dev, "adc_temp");
	if (IS_ERR(pconfig->channel_temp)) {
		dev_err(&pdev->dev, "get iio channel adc temp fail\n");
		return PTR_ERR(pconfig->channel_temp);
	}

	pzone = devm_kzalloc(&pdev->dev, sizeof(*pzone), GFP_KERNEL);
	if (!pzone)
		return -ENOMEM;

	mutex_init(&pzone->th_lock);
	pzone->dev = &pdev->dev;
	pzone->id = sensor_id;
	pzone->ops = &sprd_boardthm_ops;
	pzone->pthm_config = pconfig;
	strlcpy(pzone->name, np->name, sizeof(pzone->name));

	sprd_temp_sensor_init(pzone);
	ret = sprd_thermal_init(pzone);
	if (ret) {
		dev_err(&pdev->dev,
			"thm sensor sw init error id =%d\n", pzone->id);
		return ret;
	}
	platform_set_drvdata(pdev, pzone);
	pr_info("sprd temp sensor probe start end\n");

	return 0;
}

static int sprd_board_thm_remove(struct platform_device *pdev)
{
	struct sprd_thermal_zone *pzone = platform_get_drvdata(pdev);

	sprd_thermal_remove(pzone);
	return 0;
}

static int sprd_board_thm_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	struct sprd_thermal_zone *pzone = platform_get_drvdata(pdev);

	pzone->ops->suspend(pzone);
	return 0;
}

static int sprd_board_thm_resume(struct platform_device *pdev)
{
	pr_info("sprd board thm resume\n");
	return 0;
}

static const struct of_device_id thermal_of_match[] = {
	{.compatible = "sprd,board-thermal",},
	{}
};

static struct platform_driver sprd_thm_sensor_driver = {
	.probe = sprd_temp_sensor_probe,
	.suspend = sprd_board_thm_suspend,
	.resume = sprd_board_thm_resume,
	.remove = sprd_board_thm_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "board-thermal",
		   .of_match_table = of_match_ptr(thermal_of_match),
		   },
};

static int __init sprd_board_thermal_init(void)
{
	return platform_driver_register(&sprd_thm_sensor_driver);
}

static void __exit sprd_board_thermal_exit(void)
{
	platform_driver_unregister(&sprd_thm_sensor_driver);
}

device_initcall_sync(sprd_board_thermal_init);
module_exit(sprd_board_thermal_exit);

MODULE_AUTHOR("Freeman Liu <freeman.liu@spreadtrum.com>");
MODULE_DESCRIPTION("sprd board thermal driver");
MODULE_LICENSE("GPL");
