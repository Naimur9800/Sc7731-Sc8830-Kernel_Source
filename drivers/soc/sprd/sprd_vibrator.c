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
#include <linux/device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/kernel.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <../../../drivers/staging/android/timed_output.h>

unsigned int ANA_VIBRATOR_CTRL0;

#define CUR_DRV_CAL_SEL	(0x0 << 12)
#define SLP_LDOVIBR_PD_EN	(0x1 << 9)
#define LDO_VIBR_PD	(0x1 << 8)
#ifdef CONFIG_PMIC_SC2720
/*TODO: This macro definition is not the best method,
should removed in next version*/
#define LDO_VIBR_V	(0x2)
#else
#define LDO_VIBR_V	(0xB4)
#endif

static struct work_struct vibrator_work;
static struct hrtimer vibe_timer;
static int vibe_state;
static char *soc_pname;
static struct regmap *spi_dev;

static inline unsigned int vibrator_read(unsigned long reg)
{
	unsigned int val;

	regmap_read(spi_dev, reg, &val);
	return val;
}

static void set_vibrator(int on)
{
	if (on) {
		regmap_update_bits(spi_dev, ANA_VIBRATOR_CTRL0, LDO_VIBR_PD,
				   ~LDO_VIBR_PD);
		regmap_update_bits(spi_dev, ANA_VIBRATOR_CTRL0,
				   SLP_LDOVIBR_PD_EN, ~SLP_LDOVIBR_PD_EN);
	} else {
		regmap_update_bits(spi_dev, ANA_VIBRATOR_CTRL0, LDO_VIBR_PD,
				   LDO_VIBR_PD);
		regmap_update_bits(spi_dev, ANA_VIBRATOR_CTRL0,
				   SLP_LDOVIBR_PD_EN, SLP_LDOVIBR_PD_EN);
	}
}

static void vibrator_hw_init(void)
{
#ifdef BIT_RTC_VIBR_EN
	regmap_update_bits(spi_dev, ANA_REG_GLB_RTC_CLK_EN,
			BIT_RTC_VIBR_EN, BIT_RTC_VIBR_EN);
#endif
	regmap_update_bits(spi_dev, ANA_VIBRATOR_CTRL0, CUR_DRV_CAL_SEL,
			CUR_DRV_CAL_SEL);
	regmap_update_bits(spi_dev, ANA_VIBRATOR_CTRL0, 0xff, 0x00);
	regmap_update_bits(spi_dev, ANA_VIBRATOR_CTRL0, LDO_VIBR_V, LDO_VIBR_V);
}

static void update_vibrator(struct work_struct *work)
{
	set_vibrator(vibe_state);
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	hrtimer_cancel(&vibe_timer);

	if (value == 0)
		vibe_state = 0;
	else {
		value = (value > 15000) ? 15000 : value;
		vibe_state = 1;
		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}

	schedule_work(&vibrator_work);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	ktime_t re;

	if (hrtimer_active(&vibe_timer)) {
		re = hrtimer_get_remaining(&vibe_timer);
		return ktime_to_ns(re);
	} else
		return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	vibe_state = 0;
	schedule_work(&vibrator_work);

	return HRTIMER_NORESTART;
}

static struct timed_output_dev sprd_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static char *soc_name[] = {
	"sc2713",
	"sc2723",
	"sc2731",
	"sc2721",
	"sc2720",
};

static struct of_device_id sprd_vibrator_of_match[] = {
	{.compatible = "sprd,sc2723t-vibrator", .data = &soc_name[1],},
	{.compatible = "sprd,sc2731-vibrator", .data = &soc_name[2],},
	{.compatible = "sprd,sc2721-vibrator", .data = &soc_name[3],},
	{.compatible = "sprd,sc2720-vibrator", .data = &soc_name[4],},
	{}
};

static int sprd_vibrator_timed_output_register(void)
{
	vibrator_hw_init();
	INIT_WORK(&vibrator_work, update_vibrator);
	vibe_state = 0;
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&sprd_vibrator);

	return 0;
}

static int sprd_vibrator_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const struct of_device_id *of_id;
	int ret;

	of_id = of_match_node(sprd_vibrator_of_match, node);
	if (of_id)
		soc_pname = (char *)of_id->data;
	else
		panic("%s: Not find matched id!", __func__);

	spi_dev = dev_get_regmap(pdev->dev.parent, NULL);
	if (!spi_dev)
		panic("%s :NULL spi parent property for vibrator!", __func__);

	ret = of_property_read_u32(node, "reg", &ANA_VIBRATOR_CTRL0);
	if (ret)
		panic("%s :no reg of property for vibrator!", __func__);

	sprd_vibrator_timed_output_register();

	return 0;
}

static struct platform_driver sprd_vibrator_driver = {
	.driver = {
		   .name = "vibrator",
		   .owner = THIS_MODULE,
		   .of_match_table = sprd_vibrator_of_match,
		   },
	.probe = sprd_vibrator_probe,
};

module_platform_driver(sprd_vibrator_driver);

MODULE_DESCRIPTION("vibrator driver for spreadtrum Processors");
MODULE_LICENSE("GPL");
