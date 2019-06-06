/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include "sprd_2731_fchg.h"
#include "sprd_charge_helper.h"

#define SPRD_FCHG_DEBUG(format, arg...) pr_info("sprdfchg: " format, ## arg)

static struct regmap *reg_map;
static struct sprd_fchg_data fchg_data;

static unsigned int fchg_adi_read(unsigned long reg)
{
	unsigned int val;

	regmap_read(reg_map,
		fchg_data.fchg_pdata->base_addr + reg, &val);
	return val;
}

static int fchg_adi_write(unsigned long reg, unsigned int or_val,
				 unsigned int clear_msk)
{
	return regmap_update_bits(reg_map,
		fchg_data.fchg_pdata->base_addr + reg,
		clear_msk, or_val);
}

static void sprd_fchg_dump_regs(void)
{
	SPRD_FCHG_DEBUG("%s:\n", __func__);
	SPRD_FCHG_DEBUG("0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
		fchg_adi_read(FCHG_TIME1),
		fchg_adi_read(FCHG_TIME2),
		fchg_adi_read(FCHG_DELAY),
		fchg_adi_read(FCHG_CFG),
		fchg_adi_read(FCHG_TO_SW),
		fchg_adi_read(FCHG_INT_CTL));
}

static irqreturn_t sprd_fchg_interrupt(int irq, void *dev_id)
{
	unsigned int int_ctl = fchg_adi_read(FCHG_INT_CTL);
	unsigned int det_status = fchg_adi_read(FCHG_TO_SW);

	SPRD_FCHG_DEBUG("%s int_ctl=%x,det_status=%x\n",
		__func__, int_ctl, det_status);

	fchg_adi_write(FCHG_INT_CTL,
		FCHG_INT_CLR_BIT, FCHG_INT_CLR_BIT);
	fchg_data.fchg_check_flag = 1;
	if (det_status & FCHG_OUT_OK_BIT) {
		SPRD_FCHG_DEBUG("output ok\n");
		fchg_data.fchg_detect_ok = 1;
	} else if (det_status & FCHG_ERR0_BIT) {
		SPRD_FCHG_DEBUG("error0\n");
		fchg_data.fchg_detect_ok = 0;
	} else if (det_status & FCHG_ERR1_BIT) {
		SPRD_FCHG_DEBUG("error1\n");
		fchg_data.fchg_detect_ok = 0;
	} else {
		SPRD_FCHG_DEBUG("error2\n");
		fchg_data.fchg_detect_ok = 0;
	}
	return IRQ_HANDLED;
}

int sprd_2731_fchg_init(unsigned int vol)
{
	int times = 10;
	unsigned int vol_reg_val = 0;

	mutex_lock(&fchg_data.lock);
	glb_adi_write(ANA_REG_GLB_MODULE_EN1,
		BIT_ANA_FAST_CHG_EN, BIT_ANA_FAST_CHG_EN);
	glb_adi_write(ANA_REG_GLB_RTC_CLK_EN1,
		BIT_RTC_FAST_CHG_EN, BIT_RTC_FAST_CHG_EN);

	SPRD_FCHG_DEBUG("glb en1 =0x%x,clk=0x%x\n",
		glb_adi_read(ANA_REG_GLB_MODULE_EN1),
		glb_adi_read(ANA_REG_GLB_RTC_CLK_EN1));

	fchg_data.fchg_check_flag = 0;
	fchg_data.fchg_detect_ok = 0;

	if (vol < 9000)
		vol_reg_val = 0;
	else if (vol < 12000)
		vol_reg_val = 1;
	else if (vol < 20000)
		vol_reg_val = 2;
	else
		vol_reg_val = 3;

	fchg_adi_write(FCHG_TIME1, FCHG_TIME1_BITS(0x514),
		FCHG_TIME1_BITS(~0U));
	fchg_adi_write(FCHG_TIME2, FCHG_TIME2_BITS(0x9c4),
		FCHG_TIME2_BITS(~0U));
	fchg_adi_write(FCHG_CFG, FCHG_DET_VOL_BITS(vol_reg_val),
		FCHG_DET_VOL_BITS(~0U));
	fchg_adi_write(FCHG_CFG, FCHG_ENABLE_BIT, FCHG_ENABLE_BIT);
	fchg_adi_write(FCHG_INT_CTL, FCHG_INT_EN_BIT, FCHG_INT_EN_BIT);

	SPRD_FCHG_DEBUG("dump regs\n");
	sprd_fchg_dump_regs();
	while ((!fchg_data.fchg_check_flag) && times) {
		times--;
		msleep(300);
	}
	SPRD_FCHG_DEBUG("times =%d\n", times);

	if (fchg_data.fchg_detect_ok)
		SPRD_FCHG_DEBUG("fchg_detect_ok,set chg vol\n");

	mutex_unlock(&fchg_data.lock);
	return fchg_data.fchg_detect_ok;
}

void sprd_2731_fchg_deinit(void)
{
	SPRD_FCHG_DEBUG("%s\n", __func__);
	fchg_adi_write(FCHG_CFG, 0, FCHG_ENABLE_BIT);
	msleep(100);
	glb_adi_write(ANA_REG_GLB_MODULE_EN1,
		0, BIT_ANA_FAST_CHG_EN);
	glb_adi_write(ANA_REG_GLB_RTC_CLK_EN1,
		0, BIT_RTC_FAST_CHG_EN);
	SPRD_FCHG_DEBUG("2 glb en1 =0x%x,clk=0x%x\n",
		glb_adi_read(ANA_REG_GLB_MODULE_EN1),
		glb_adi_read(ANA_REG_GLB_RTC_CLK_EN1));
	SPRD_FCHG_DEBUG("%s dump regs\n", __func__);
	sprd_fchg_dump_regs();
}

struct sprd_fchg_operations sprd_2731_fchg_ops = {
	.fchg_init = sprd_2731_fchg_init,
	.fchg_deinit = sprd_2731_fchg_deinit,
};

static int sprd_2731_fchg_probe(struct platform_device *pdev)
{
	int ret = 0, irq = 0;
	u32 value = 0;
	struct device_node *np = pdev->dev.of_node;

	SPRD_FCHG_DEBUG("%s\n", __func__);
	reg_map = dev_get_regmap(pdev->dev.parent, NULL);
	if (!reg_map)
		dev_err(&pdev->dev, "%s :NULL regmap for fchg 2731\n",
			__func__);

	fchg_data.fchg_pdata = devm_kzalloc(&pdev->dev,
		sizeof(struct sprd_fchg_platform_data),
		GFP_KERNEL);
	if (!fchg_data.fchg_pdata)
		return -ENOMEM;

	fchg_data.dev = &pdev->dev;
	irq = platform_get_irq(pdev, 0);
	if (unlikely(irq <= 0))
		dev_err(&pdev->dev, "no irq resource specified\n");
	fchg_data.fchg_pdata->fchg_irq = irq;

	ret = of_property_read_u32(np, "reg", &value);
	if (ret)
		dev_err(&pdev->dev, "%s :no reg of property for pmic fgu\n",
			__func__);
	fchg_data.fchg_pdata->base_addr = (unsigned long)value;

	ret = devm_request_threaded_irq(fchg_data.dev,
			fchg_data.fchg_pdata->fchg_irq,
			NULL, sprd_fchg_interrupt,
			IRQF_NO_SUSPEND | IRQF_ONESHOT,
			"sprdfchg", NULL);
	if (ret)
		SPRD_FCHG_DEBUG("request sprdfchg irq %d failed\n",
		       fchg_data.fchg_pdata->fchg_irq);

	platform_set_drvdata(pdev, &fchg_data);
	mutex_init(&fchg_data.lock);
	sprdbat_register_fchg_ops(&sprd_2731_fchg_ops);
	SPRD_FCHG_DEBUG("sprd_2731_fchg_probe ok\n");
	return ret;
}

static int sprd_2731_fchg_remove(struct platform_device *pdev)
{
	sprdbat_unregister_fchg_ops();
	return 0;
}

static struct of_device_id sprd_2731_fchg_of_match[] = {
	{.compatible = "sprd,sc2731-fchg",},
	{}
};

static struct platform_driver sprd_2731_fchg_driver = {
	.driver = {
		   .name = "sc2731-fchg",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(sprd_2731_fchg_of_match),
		   },
	.probe = sprd_2731_fchg_probe,
	.remove = sprd_2731_fchg_remove
};

static int __init sprd_2731_fchg_driver_init(void)
{
	return platform_driver_register(&sprd_2731_fchg_driver);
}

static void __exit sprd_2731_fchg_driver_exit(void)
{
	platform_driver_unregister(&sprd_2731_fchg_driver);
}

subsys_initcall_sync(sprd_2731_fchg_driver_init);
module_exit(sprd_2731_fchg_driver_exit);
