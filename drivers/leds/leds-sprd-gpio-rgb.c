/*
* Copyright (C) 2017 Spreadtrum Communications Inc.
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
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/io.h>

#define RED_ATC_GPIO_ADDR 0XE42AC028
#define RED_GPIO_ADDR 0XE42AB008
#define GREEN_GPIO_ADDR 0XE42AB004

struct sprd_leds_gpio_rgb {
	struct platform_device *dev;
	struct mutex mutex;
	spinlock_t value_lock;
	enum led_brightness value;
	struct led_classdev cdev;
	unsigned long red_atc_addr, red_addr, green_addr;
	unsigned long red_atc_val, red_val, green_val;
	int led_red_atc, led_red, led_green;
};

enum sprd_leds_type {
	SPRD_LED_TYPE_R = 0,
	SPRD_LED_TYPE_G,
	SPRD_LED_TYPE_TOTAL
};

static char *sprd_leds_rgb_name[SPRD_LED_TYPE_TOTAL] = {
	"red",
	"green",
};

static struct sprd_leds_gpio_rgb *brgb[SPRD_LED_TYPE_TOTAL];

#define to_sprd_gpio_rgb(led_cdev) \
	container_of(led_cdev, struct sprd_leds_gpio_rgb, cdev)

static void sprd_leds_gpio_rgb_enable(struct sprd_leds_gpio_rgb *brgb)
{
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_R]) == 0) {
		gpio_direction_output(brgb->led_red, 1);
		writel_relaxed(brgb->red_val|BIT(8),
			(void __iomem *)brgb->red_addr);
	}
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_G]) == 0) {
		gpio_direction_output(brgb->led_green, 1);
		writel_relaxed(brgb->green_val|BIT(8),
			(void __iomem *)brgb->green_addr);
	}
	pr_info("[SPRD_GPIO_RGB_INFO] sprd_leds_gpio_rgb_enable\n");
}

static void sprd_leds_gpio_rgb_disable(struct sprd_leds_gpio_rgb *brgb)
{
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_R]) == 0) {
		gpio_direction_output(brgb->led_red, 0);
		writel_relaxed(brgb->red_val & ~(BIT(8)),
			(void __iomem *)brgb->red_addr);
	}
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_G]) == 0) {
		gpio_direction_output(brgb->led_green, 0);
		writel_relaxed(brgb->green_val & ~(BIT(8)),
			(void __iomem *)brgb->green_addr);
	}
	pr_info("[SPRD_GPIO_RGB_INFO] sprd_leds_gpio_rgb_disable\n");
}

static void sprd_leds_rgb_work(struct sprd_leds_gpio_rgb *brgb)
{
	unsigned long flags;

	mutex_lock(&brgb->mutex);
	spin_lock_irqsave(&brgb->value_lock, flags);
	if (brgb->value == LED_OFF) {
		spin_unlock_irqrestore(&brgb->value_lock, flags);
		sprd_leds_gpio_rgb_disable(brgb);
		goto out;
	}
	spin_unlock_irqrestore(&brgb->value_lock, flags);
	sprd_leds_gpio_rgb_enable(brgb);
	pr_info("[SPRD_GPIO_RGB_INFO] sprd_leds_gpio_rgb_work_for rgb!\n");
out:
	mutex_unlock(&brgb->mutex);
}

static void sprd_leds_gpio_rgb_set(struct led_classdev *gpio_rgb_cdev,
				   enum led_brightness value)
{
	struct sprd_leds_gpio_rgb *brgb;
	unsigned long flags;

	brgb = to_sprd_gpio_rgb(gpio_rgb_cdev);
	spin_lock_irqsave(&brgb->value_lock, flags);
	brgb->value = value;
	spin_unlock_irqrestore(&brgb->value_lock, flags);
	if (strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_R]) == 0 ||
	strcmp(brgb->cdev.name, sprd_leds_rgb_name[SPRD_LED_TYPE_G]) == 0)
		sprd_leds_rgb_work(brgb);
}

static void sprd_leds_gpio_rgb_shutdown(struct platform_device *dev)
{
	int i;

	for (i = 0; i < SPRD_LED_TYPE_TOTAL; i++) {
		mutex_lock(&brgb[i]->mutex);
		sprd_leds_gpio_rgb_disable(brgb[i]);
		mutex_unlock(&brgb[i]->mutex);
	}
}

static const struct of_device_id sprd_rgb_gpio_of_match[] = {
	{.compatible = "sprd,gpio-rgb", },
	{}
};

static int sprd_leds_gpio_rgb_probe(struct platform_device *dev)
{
	struct device_node *node = dev->dev.of_node;
	int ret, i;
	unsigned long reg1, reg2, reg3, val1, val2, val3;
	int led_red_atc, led_red, led_green;

	led_red = of_get_gpio(node, 0);
	if (led_red < 0)
		dev_err(&dev->dev,  "[ERROR] failed to parse led_red_gpio.\n");
	led_green = of_get_gpio(node, 1);
	if (led_green < 0)
		dev_err(&dev->dev,  "[ERROR] failed to parse led_green_gpio.\n");
	led_red_atc = of_get_gpio(node, 2);
	if (led_red_atc < 0)
		dev_err(&dev->dev,  "[ERROR] failed to parse led_red_atc_gpio.\n");
	ret = devm_gpio_request(&dev->dev, led_red, "led_red");
	if (ret) {
		dev_err(&dev->dev, "Fail to request led_red gpio %d.\n",
			led_red);
		goto err;
	}
	ret = devm_gpio_request(&dev->dev, led_green, "led_green");
	if (ret) {
		dev_err(&dev->dev, "Fail to request led_green gpio %d.\n",
			led_green);
		goto err;
	}
	ret = devm_gpio_request(&dev->dev, led_red_atc, "led_red_atc");
	if (ret) {
		dev_err(&dev->dev, "Fail to request led_red_atc gpio %d.\n",
			led_red_atc);
		goto err;
	}
	reg1 = (unsigned long)ioremap_nocache(RED_ATC_GPIO_ADDR, 0x4);
	reg2 = (unsigned long)ioremap_nocache(RED_GPIO_ADDR, 0x4);
	reg3 = (unsigned long)ioremap_nocache(GREEN_GPIO_ADDR, 0x4);
	val1 = readl_relaxed((void __iomem *)reg1);
	val2 = readl_relaxed((void __iomem *)reg2);
	val3 = readl_relaxed((void __iomem *)reg3);
	gpio_direction_output(led_red_atc, 1);
	writel_relaxed(val1|BIT(8), (void __iomem *)reg1);
	for (i = 0; i < SPRD_LED_TYPE_TOTAL; i++) {
		struct sprd_leds_gpio_rgb *p = devm_kzalloc(&dev->dev,
			sizeof(*p), GFP_KERNEL);
		if (!p) {
			ret = -ENOMEM;
			goto err;
		}
		p->cdev.brightness_set = sprd_leds_gpio_rgb_set;
		p->cdev.name = sprd_leds_rgb_name[i];
		p->cdev.brightness_get = NULL;
		spin_lock_init(&p->value_lock);
		mutex_init(&p->mutex);
		p->value = LED_OFF;
		p->red_atc_addr = reg1;
		p->red_addr = reg2;
		p->green_addr = reg3;
		p->red_atc_val = val1;
		p->red_val = val2;
		p->green_val = val3;
		p->led_red_atc = led_red_atc;
		p->led_red = led_red;
		p->led_green = led_green;
		ret = led_classdev_register(&dev->dev, &p->cdev);
		if (ret < 0)
			goto err;
		brgb[i] = p;
	}
	return 0;
err:
	for (i = 0; i < SPRD_LED_TYPE_TOTAL; i++) {
		if (brgb[i])
			led_classdev_unregister(&brgb[i]->cdev);
	}
	return ret;
}

static int sprd_leds_gpio_rgb_remove(struct platform_device *dev)
{
	int i;

	for (i = 0; i < SPRD_LED_TYPE_TOTAL; i++) {
		led_classdev_unregister(&brgb[i]->cdev);
		brgb[i]->value = LED_OFF;
		sprd_leds_gpio_rgb_disable(brgb[i]);
	}
	return 0;
}

static struct platform_driver sprd_leds_gpio_rgb_driver = {
	.driver = {
		   .name = "sprd-leds-gpio-rgb",
		   .owner = THIS_MODULE,
		   .of_match_table = sprd_rgb_gpio_of_match,
		   },
	.probe = sprd_leds_gpio_rgb_probe,
	.remove = sprd_leds_gpio_rgb_remove,
	.shutdown = sprd_leds_gpio_rgb_shutdown,
};

module_platform_driver(sprd_leds_gpio_rgb_driver);

MODULE_AUTHOR("russell zhao <russell.zhao@spreadtrum.com>");
MODULE_DESCRIPTION("Sprd leds gpio rgb driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sprd_leds_gpio_rgb");
