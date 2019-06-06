/*
* Copyright (C) 2012 Spreadtrum Communications Inc.
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
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define PRINT_INFO(x...)  pr_info("[SPRD_KPLED_INFO]" x)

#define KPLED_V_SHIFT           12
#define KPLED_V_MSK             (0x0F << KPLED_V_SHIFT)
#define KPLED_PD			(1 << 11)
#define KPLED_PULLDOWN_EN		(1 << 10)

enum sprd_pmic_kpled_type {
	UNKNOWN_PMIC_KPLED,
	SC2723_KPLED,
	SC2731_KPLED,
	SC2721_KPLED,
};
enum sprd_pmic_kpled_mode {
	UNKNOWN_KPLED_MODE,
	KPLED_CURRENT_MODE,
	KPLED_LDO_MODE,
};
enum sprd_pmic_kpled_switch {
	KPLED_SWITCH_OFF,
	KPLED_SWITCH_ON,
};
/* sprd keypad backlight */
struct sprd_kpled {
	struct platform_device *dev;
	struct mutex mutex;
	struct work_struct work;
	spinlock_t value_lock;
	enum led_brightness value;
	struct led_classdev cdev;
	struct regmap *sprd_kpled_regmap;
	int enabled;
	int brightness_max;
	int brightness_min;
	int run_mode; /* current or ldo mode */
	enum sprd_pmic_kpled_type chip_version;
	unsigned int reg_kpled_ctrl0;/* current and ldo mode in sc2731 */
	unsigned int reg_kpled_ctrl1; /* ldo mode in sc2721 */
};

struct sprd_kpled_platform_data {
	int brightness_max;
	int brightness_min;
	int run_mode;
	enum sprd_pmic_kpled_type chip_version;
	unsigned int reg_kpled_ctrl0;/* current and ldo mode in sc2731 */
	unsigned int reg_kpled_ctrl1; /* ldo mode in sc2721 */
};

#define to_sprd_led(led_cdev) \
	container_of(led_cdev, struct sprd_kpled, cdev)

static const struct of_device_id kpled_of_match[];

static inline unsigned int kpled_read(struct sprd_kpled *led, unsigned long reg)
{
	unsigned int val;

	regmap_read(led->sprd_kpled_regmap, reg, &val);
	return val;
}

static void sprd_kpled_current_switch(struct sprd_kpled *led, int power)
{
	PRINT_INFO("%s power=%d\n", __func__, power);
	if (power == KPLED_SWITCH_ON)
		regmap_update_bits(led->sprd_kpled_regmap,
				   CTL_BASE_ANA_GLB + led->reg_kpled_ctrl0,
				   KPLED_PD, ~KPLED_PD);
	if (power == KPLED_SWITCH_OFF)
		regmap_update_bits(led->sprd_kpled_regmap,
				   CTL_BASE_ANA_GLB + led->reg_kpled_ctrl0,
				   KPLED_PD, KPLED_PD);
}

static void sprd_kpled_ldo_switch(struct sprd_kpled *led, int power)
{
	unsigned int ldo_reg;
	unsigned int ldo_pd_mask;
	unsigned int ldo_pd_value = 0;

	PRINT_INFO("%s power=%d\n", __func__, power);
	if (led->chip_version == SC2721_KPLED) {
		ldo_reg = led->reg_kpled_ctrl1;
		ldo_pd_mask = (1 << 15);
	} else {
		ldo_reg = led->reg_kpled_ctrl0,
		ldo_pd_mask = (1 << 8);
	}
	if (power == KPLED_SWITCH_ON)
		ldo_pd_value = ~ldo_pd_mask;

	if (power == KPLED_SWITCH_OFF) {
		ldo_pd_value = ldo_pd_mask;
		regmap_update_bits(led->sprd_kpled_regmap,
				CTL_BASE_ANA_GLB + led->reg_kpled_ctrl0,
				KPLED_PULLDOWN_EN, KPLED_PULLDOWN_EN);
	}
	PRINT_INFO("reg:0x%08X ldo_pd_mask:0x%08X  ldo_pd_value:0x%08X\n",
			CTL_BASE_ANA_GLB + ldo_reg,
			ldo_pd_mask, ldo_pd_value);
	regmap_update_bits(led->sprd_kpled_regmap,
			CTL_BASE_ANA_GLB + ldo_reg,
			ldo_pd_mask, ldo_pd_value);
}

static void sprd_kpled_set_brightness(struct sprd_kpled *led)
{
	unsigned long brightness = led->value;
	unsigned long brightness_level;
	unsigned int ldo_reg;
	unsigned int ldo_v_shift;
	unsigned int ldo_v_mask;


	brightness_level = brightness;

	PRINT_INFO("sprd_kpled_set_brightness:led->run_mode = %d\n",
		   led->run_mode);
	if (brightness_level > 255)
		brightness_level = 255;

	if (brightness_level > led->brightness_max)
		brightness_level = led->brightness_max;

	if (brightness_level < led->brightness_min)
		brightness_level = led->brightness_min;

	brightness_level = brightness_level/16;
	/*brightness steps = 16 */

	if (led->run_mode == 1) {
		regmap_update_bits(led->sprd_kpled_regmap,
				   CTL_BASE_ANA_GLB + led->reg_kpled_ctrl0,
				   KPLED_V_MSK,
				   ((brightness_level << KPLED_V_SHIFT) &
				    KPLED_V_MSK));
		PRINT_INFO("reg:0x%08X set_val:0x%08X  brightness:%ld\n",
			CTL_BASE_ANA_GLB + led->reg_kpled_ctrl0,
			kpled_read(led,
			 CTL_BASE_ANA_GLB + led->reg_kpled_ctrl0),
			brightness);
	} else {
		if (led->chip_version == SC2721_KPLED) {
			ldo_reg = led->reg_kpled_ctrl1,
			ldo_v_shift = 7;
			ldo_v_mask = 0xff << ldo_v_shift;
		} else {
			ldo_reg = led->reg_kpled_ctrl0,
			ldo_v_shift = 0;
			ldo_v_mask = 0xff << ldo_v_shift;
		}
			regmap_update_bits(led->sprd_kpled_regmap,
					CTL_BASE_ANA_GLB + ldo_reg,
					ldo_v_mask,
					((brightness_level << ldo_v_shift)
					 & ldo_v_mask));
			PRINT_INFO("reg:0x%08X set_val:0x%08X brightness:%ld\n",
					CTL_BASE_ANA_GLB + ldo_reg,
					kpled_read(led,
						CTL_BASE_ANA_GLB + ldo_reg),
					brightness);
	}
}

static void sprd_kpled_enable(struct sprd_kpled *led)
{
	if (led->run_mode == 1)  /* current mode */
		sprd_kpled_current_switch(led, KPLED_SWITCH_ON);
	else  /* ldo mode */
		sprd_kpled_ldo_switch(led, KPLED_SWITCH_ON);

	PRINT_INFO("sprd_kpled_enable\n");
	sprd_kpled_set_brightness(led);
	led->enabled = 1;
}

static void sprd_kpled_disable(struct sprd_kpled *led)
{
	if (led->run_mode == 1)
		sprd_kpled_current_switch(led, KPLED_SWITCH_OFF);
	else
		sprd_kpled_ldo_switch(led, KPLED_SWITCH_OFF);

	PRINT_INFO("sprd_kpled_disable\n");
	led->enabled = 0;
}

static void sprd_kpled_work(struct work_struct *work)
{
	struct sprd_kpled *led = container_of(work, struct sprd_kpled, work);
	unsigned long flags;

	mutex_lock(&led->mutex);
	spin_lock_irqsave(&led->value_lock, flags);
	if (led->value == LED_OFF) {
		spin_unlock_irqrestore(&led->value_lock, flags);
		sprd_kpled_disable(led);
		goto out;
	}
	spin_unlock_irqrestore(&led->value_lock, flags);
	sprd_kpled_enable(led);
out:
	mutex_unlock(&led->mutex);
}

static void sprd_kpled_set(struct led_classdev *led_cdev,
			   enum led_brightness value)
{
	struct sprd_kpled *led = to_sprd_led(led_cdev);
	unsigned long flags;

	PRINT_INFO("sprd_kpled_set!\n");
	PRINT_INFO("led->chip_version=%d\n", led->chip_version);
	PRINT_INFO("led->brightness_max=%d\n", led->brightness_max);
	PRINT_INFO("led->run_mode=%d\n", led->run_mode);
	PRINT_INFO("led->reg_kpled_ctrl0=0x%x\n", led->reg_kpled_ctrl0);
	if (led->chip_version == SC2721_KPLED)
		PRINT_INFO("led->reg_kpled_ctrl1=0x%x\n", led->reg_kpled_ctrl1);
	spin_lock_irqsave(&led->value_lock, flags);
	led->value = value;
	spin_unlock_irqrestore(&led->value_lock, flags);

	schedule_work(&led->work);
}

static void sprd_kpled_shutdown(struct platform_device *dev)
{
	struct sprd_kpled *led = platform_get_drvdata(dev);

	mutex_lock(&led->mutex);
	sprd_kpled_disable(led);
	mutex_unlock(&led->mutex);
}

#ifdef CONFIG_OF
static struct sprd_kpled_platform_data *sprd_kpled_parse_dt(struct
							    platform_device
							    *pdev)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;
	struct sprd_kpled_platform_data *pdata = NULL;
	const struct of_device_id *of_id;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		PRINT_INFO("sprd_kpled Could not allocate pdata");
		return NULL;
	}
	of_id = of_match_node(kpled_of_match,
		pdev->dev.of_node);
	if (!of_id) {
		PRINT_INFO("fail to get device id fail!\n");
		goto fail;
	}

	pdata->chip_version = (enum sprd_pmic_kpled_type)(of_id->data);
	PRINT_INFO("chip is %d\n", pdata->chip_version);
	ret =
	    of_property_read_u32(np, "brightness_max", &pdata->brightness_max);
	if (ret) {
		PRINT_INFO("fail to get pdata->brightness_max\n");
		goto fail;
	}
	ret =
	    of_property_read_u32(np, "brightness_min", &pdata->brightness_min);
	if (ret) {
		PRINT_INFO("fail to get pdata->brightness_min\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "run_mode", &pdata->run_mode);
	if (ret) {
		PRINT_INFO("fail to get pdata->run_mode\n");
		goto fail;
	}
	ret = of_property_read_u32_index(np, "reg", 0, &pdata->reg_kpled_ctrl0);
	if (ret) {
		PRINT_INFO("fail to get pdata->reg_kpled_ctrl0\n");
		goto fail;
	}
	PRINT_INFO("chip reg_kpled_ctrl0= %x\n", pdata->reg_kpled_ctrl0);

	if (pdata->chip_version == SC2721_KPLED) {
		ret = of_property_read_u32_index(np, "reg",
				1, &pdata->reg_kpled_ctrl1);
		if (ret) {
			PRINT_INFO("fail to get pdata->reg_kpled_ctrl0\n");
			goto fail;
		}
		PRINT_INFO("chip reg_kpled_ctrl1= %x\n",
				pdata->reg_kpled_ctrl1);
	}

	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

static const struct of_device_id kpled_of_match[] = {
	{.compatible = "sprd,sc2723t-kpled", .data = (void *)SC2723_KPLED,},
	{.compatible = "sprd,sc2731-kpled", .data = (void *)SC2731_KPLED,},
	{.compatible = "sprd,sc2721-kpled", .data = (void *)SC2721_KPLED,},
	{}
};

static int sprd_kpled_probe(struct platform_device *dev)
{
	struct sprd_kpled *led;
	int ret;
	struct sprd_kpled_platform_data *pdata = NULL;

#ifdef CONFIG_OF
	struct device_node *np = dev->dev.of_node;

	if (np) {
		pdata = sprd_kpled_parse_dt(dev);
		if (pdata == NULL) {
			PRINT_INFO("get dts data failed!\n");
			return -ENODEV;
		}
	} else {
		PRINT_INFO("dev.of_node is NULL!\n");
		return -ENODEV;
	}
#else
	pdata = dev->dev.platform_data;
	if (!pdata) {
		PRINT_INFO("No kpled_platform data!\n");
		return -ENODEV;
	}
#endif

	led = kzalloc(sizeof(struct sprd_kpled), GFP_KERNEL);
	if (led == NULL) {
		PRINT_INFO("No memory for device\n");
		ret = -ENOMEM;
		goto out1;
	}
	led->sprd_kpled_regmap = dev_get_regmap(dev->dev.parent, NULL);
	if (!led->sprd_kpled_regmap) {
		PRINT_INFO("%s :NULL spi parent property for kpled!", __func__);
		ret = -ENOMEM;
		goto out2;
	}

	led->cdev.brightness_set = sprd_kpled_set;
	led->cdev.default_trigger = "none";
	led->cdev.name = "keyboard-backlight";
	led->cdev.brightness_get = NULL;
	led->cdev.flags |= LED_CORE_SUSPENDRESUME;
	led->enabled = 0;
	led->brightness_max = pdata->brightness_max;
	led->brightness_min = pdata->brightness_min;
	led->run_mode = pdata->run_mode;
	led->chip_version = pdata->chip_version;
	led->reg_kpled_ctrl0 = pdata->reg_kpled_ctrl0;
	if (led->chip_version == SC2721_KPLED)
		led->reg_kpled_ctrl1 = pdata->reg_kpled_ctrl1;
	PRINT_INFO("led->chip_version=%d\n", led->chip_version);
	PRINT_INFO("led->brightness_max=%d\n", led->brightness_max);
	PRINT_INFO("led->run_mode=%d\n", led->run_mode);
	PRINT_INFO("led->reg_kpled_ctrl0=%x\n", led->reg_kpled_ctrl0);
	if (led->chip_version == SC2721_KPLED)
		PRINT_INFO("led->reg_kpled_ctrl1= %x\n", led->reg_kpled_ctrl1);
	spin_lock_init(&led->value_lock);
	mutex_init(&led->mutex);
	INIT_WORK(&led->work, sprd_kpled_work);
	led->value = LED_OFF;
	platform_set_drvdata(dev, led);

	/* register our new led device */
	ret = led_classdev_register(&dev->dev, &led->cdev);
	if (ret < 0) {
		PRINT_INFO("led_classdev_register failed\n");
		ret = -ENOMEM;
		goto out2;
	}
	sprd_kpled_disable(led);
	kfree(pdata);
	return 0;
out2:
	kfree(led);
out1:
	kfree(pdata);
	return ret;
}

static int sprd_kpled_remove(struct platform_device *dev)
{
	struct sprd_kpled *led = platform_get_drvdata(dev);

	led_classdev_unregister(&led->cdev);
	flush_scheduled_work();
	led->value = LED_OFF;
	led->enabled = 1;
	sprd_kpled_disable(led);
	kfree(led);

	return 0;
}

static struct platform_driver sprd_kpled_driver = {
	.driver = {
		   .name = "sprd-kpled",
		   .owner = THIS_MODULE,
		   .of_match_table = kpled_of_match,
		   },
	.probe = sprd_kpled_probe,
	.remove = sprd_kpled_remove,
	.shutdown = sprd_kpled_shutdown,
};

module_platform_driver(sprd_kpled_driver);

MODULE_AUTHOR("ya huang <ya.huang@spreadtrum.com>");
MODULE_DESCRIPTION("Sprd Keyboard backlight driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sprd_kpled");
