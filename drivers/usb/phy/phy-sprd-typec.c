/**
 * typec-sprd.c - Spreadtrum Type-C v1.1 Glue layer
 *
 * Copyright (c) 2015-2016 Spreadtrum Co., Ltd.
 *		http://www.spreadtrum.com
 *
 * Author: miao zhu <miao.zhu@spreadtrum.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/usb/class-dual-role.h>

#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/usb/sprd_usb.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "phy-sprd-typec.h"

static void typec_set_mode(struct typec *tc, int mode);
static BLOCKING_NOTIFIER_HEAD(typec_chain_head);

static u64 typec_attachwait_snk_time;

static const char *const typec_mode_string[] = {
	[DUAL_ROLE_PROP_MODE_UFP] = "ufp",
	[DUAL_ROLE_PROP_MODE_DFP] = "dfp",
	[DUAL_ROLE_PROP_MODE_NONE] = "drp",
	[3] = "Unknown",
};

static const char *const typec_connection_state_string[] = {
	[TYPEC_DETACHED_SNK] = "Unattached.SNK",
	[TYPEC_ATTACHWAIT_SNK] = "AttachWait.SNK",
	[TYPEC_ATTACHED_SNK] = "Attached.SNK",
	[TYPEC_DETACHED_SRC] = "Unattached.SRC",
	[TYPEC_ATTACHWAIT_SRC] = "AttachWait.SRC",
	[TYPEC_ATTACHED_SRC] = "Attached.SRC",
	[TYPEC_POWERED_CABLE] = "Attached.PoweredCable",
	[TYPEC_AUDIO_CABLE] = "Attached.AudioCable",
	[TYPEC_DEBUG_CABLE] = "Attached.DebugCable",
	[TYPEC_TOGGLE_SLEEP] = "DRP.Sleep",
	[TYPEC_ERR_RECOV] = "ErrorRecovery",
	[TYEEC_DISABLED] = "Disabled",
	[TYEEC_TRY_SNK] = "Unattached.Try.SNK",
	[TYEEC_TRY_WAIT_SRC] = "Unattached.Try.WaitSRC",
	[TYEEC_TRY_SRC] = "Unattached.Try.SRC",
	[TYEEC_TRY_WAIT_SNK] = "Unattached.Try.WaitSNK",
	[TYEEC_UNSUPOORT_ACC] = "Unsupport ACC",
	[TYEEC_ORIENTED_DEBUG] = "Oriented Debug",
};

static enum dual_role_property typec_properties[] = {
	DUAL_ROLE_PROP_SUPPORTED_MODES,
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
	DUAL_ROLE_PROP_VCONN_SUPPLY,
	DUAL_ROLE_PROP_ACC_MODE,
};

/* Only for debug purpose */
static struct typec *__typec;

/*For a new USB3.0 typec solution on board v110 of sp9860g_2h10 project  */
static int is_sp9860g_2h10;
static int is_v110_board;

static __init int get_hardware_version(char *str)
{
	if (strcmp(str, "sp9860g_2h10"))
		is_sp9860g_2h10 = 0;
	else
		is_sp9860g_2h10 = 1;
	pr_info("androidboot.hardware=%s %d\n", str, is_sp9860g_2h10);

	return 0;
}

__setup("androidboot.hardware=", get_hardware_version);

static __init int get_board_id(char *str)
{
	if (strcmp(str, "43"))
		is_v110_board = 0;
	else
		is_v110_board = 1;
	pr_info("rfboard.id=%s %d\n", str, is_v110_board);

	return 0;
}
__setup("rfboard.id=", get_board_id);


static int init_usb30_ldo_resource(struct device_node *np,
	struct typec *tc, struct platform_device *pdev)
{
	struct pinctrl *ldo_ctrl = NULL;
	struct pinctrl_state *ldo_pin_sw = NULL;
	struct pinctrl_state *ldo_pin_md = NULL;
	int ret = 0;

	/* A external LDO need to be control  for new usb30
	* typec solution  on board v110 of sp9860g_2h10 project
	*/
	ldo_ctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(ldo_ctrl)) {
		dev_err(&pdev->dev, "can not find pinctrl\n");
		return PTR_ERR(ldo_ctrl);
	}

	ldo_pin_sw =
		pinctrl_lookup_state(ldo_ctrl, "ldo_pin_sw");
	if (IS_ERR(ldo_pin_sw)) {
		dev_err(&pdev->dev, "can not find pinctrl ldo_pin_sw\n");
		return PTR_ERR(ldo_pin_sw);
	}

	ldo_pin_md =
		pinctrl_lookup_state(ldo_ctrl, "ldo_ctl_md");
	if (IS_ERR(ldo_pin_md)) {
		dev_err(&pdev->dev, "can not find pinctrl ldo_pin_md\n");
		return PTR_ERR(ldo_pin_md);
	}

	ret = pinctrl_select_state(ldo_ctrl, ldo_pin_sw);
	if (ret) {
		dev_err(&pdev->dev, "switch ldo pinctrl ldo_pin_sw failed\n");
		return ret;
	}

	ret = pinctrl_select_state(ldo_ctrl, ldo_pin_md);
	if (ret) {
		dev_err(&pdev->dev, "switch ldo pinctrl ldo_pin_md failed\n");
		return ret;
	}

	tc->ldo_gpio = of_get_named_gpio(np, "ldo-gpios", 0);
	if (tc->ldo_gpio < 0) {
		dev_err(&pdev->dev, "fail to get ldo gpio number\n");
		return tc->ldo_gpio;
	}

	ret = gpio_request(tc->ldo_gpio, "ldo_sw");
	if (ret) {
		dev_err(&pdev->dev, "cannot claim ldo_sw pin\n");
		return ret;
	}

	ret = gpio_direction_output(tc->ldo_gpio, 0);
	if (ret) {
		dev_err(&pdev->dev, "cannot set ldo_sw output direction\n");
		return ret;
	}

	return 0;
}

static void ldo_ss_channel_sw(struct typec *tc, int value)
{
	if (tc->ldo_sw_en)
		gpio_set_value(tc->ldo_gpio, value);
}

/* for super-speed channel switch */
void typec_disable_ldo(void)
{
	ldo_ss_channel_sw(__typec, 0);
}

int register_ext_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&typec_chain_head, nb);
}

int unregister_ext_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&typec_chain_head, nb);
}

static int typec_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&typec_chain_head, val, NULL)
		== NOTIFY_BAD) ? -EINVAL : 0;
}

static ssize_t mode_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct typec *tc = dev_get_drvdata(dev);
	uint32_t mode;

	if (!tc)
		return -ENODEV;

	regmap_read(tc->base, TYPEC_MODE, &mode);
	return sprintf(buf, "%s\n", typec_mode_string[mode]);
}

static ssize_t mode_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t size)
{
	struct typec *tc = dev_get_drvdata(dev);
	int mode;

	if (!tc)
		return -ENODEV;

	if (kstrtoint(buf, 0, &mode) < 0)
		return -EINVAL;

	if (mode > DUAL_ROLE_PROP_MODE_NONE || mode < DUAL_ROLE_PROP_MODE_UFP)
		return -EINVAL;

	typec_set_mode(tc, mode);

	return size;
}
static DEVICE_ATTR_RW(mode);

static ssize_t connection_state_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct typec *tc = dev_get_drvdata(dev);
	uint32_t state;

	if (!tc)
		return -ENODEV;

	regmap_read(tc->base, TYPEC_STATUS, &state);
	state &= TYPEC_STATE_MASK;

	return sprintf(buf, "%s\n", typec_connection_state_string[state]);
}
static DEVICE_ATTR_RO(connection_state);

static ssize_t tccdebounce_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct typec *tc = dev_get_drvdata(dev);
	int cnt;

	if (!tc)
		return -ENODEV;

	regmap_read(tc->base, TYPEC_TCCDE_CNT, &cnt);

	return sprintf(buf, "%ims\n", (1 + cnt) / 32);
}

static ssize_t tccdebounce_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct typec *tc = dev_get_drvdata(dev);
	int time;

	if (!tc)
		return -ENODEV;

	if (kstrtoint(buf, 0, &time) < 0)
		return -EINVAL;

	if (time < 100 || time > 200)
		return -EINVAL;

	regmap_write(tc->base, TYPEC_TCCDE_CNT, (uint16_t)(time * 32 - 1));

	return size;
}
static DEVICE_ATTR_RW(tccdebounce);

static ssize_t tpddebounce_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct typec *tc = dev_get_drvdata(dev);
	int cnt;

	if (!tc)
		return -ENODEV;

	regmap_read(tc->base, TYPEC_TPDDE_CNT, &cnt);

	return sprintf(buf, "%ims\n", (1 + cnt) / 32);
}

static ssize_t tpddebounce_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct typec *tc = dev_get_drvdata(dev);
	int time;

	if (!tc)
		return -ENODEV;

	if (kstrtoint(buf, 0, &time) < 0)
		return -EINVAL;

	if (time < 10 || time > 20)
		return -EINVAL;

	regmap_write(tc->base, TYPEC_TCCDE_CNT, (uint16_t)(time * 32 - 1));

	return size;
}
static DEVICE_ATTR_RW(tpddebounce);

static ssize_t tsleep_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct typec *tc = dev_get_drvdata(dev);
	uint32_t cnt;
	uint32_t config;
	uint32_t sleep_cnt1, sleep_cnt2;

	if (!tc)
		return -ENODEV;

	regmap_read(tc->base, TYPEC_CONFIG, &config);
	regmap_read(tc->base, TYPEC_SLEEP_CNT1, &sleep_cnt1);
	regmap_read(tc->base, TYPEC_SLEEP_CNT2, &sleep_cnt2);

	if (tc->mode == DUAL_ROLE_PROP_MODE_NONE &&
	    config & TYPEC_TOGGLE_SLEEP_EN)
		cnt = sleep_cnt1 | sleep_cnt2 << 16;
	else
		cnt = 0;

	return sprintf(buf, "%ims\n", (1 + cnt) / 32);
}

static ssize_t tsleep_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t size)
{
	struct typec *tc = dev_get_drvdata(dev);
	int tsleep;

	if (!tc)
		return -ENODEV;

	if (kstrtoint(buf, 0, &tsleep) < 0)
		return -EINVAL;

	tsleep = tsleep * 32 - 1;

	regmap_write(tc->base, TYPEC_SLEEP_CNT1, (uint16_t)tsleep);
	regmap_write(tc->base, TYPEC_SLEEP_CNT2, (uint16_t)(tsleep >> 16));

	return size;
}
static DEVICE_ATTR_RW(tsleep);

static ssize_t tdrp_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct typec *tc = dev_get_drvdata(dev);
	uint32_t cnt;

	if (!tc)
		return -ENODEV;

	regmap_read(tc->base, TYPEC_TDRP_CNT, &cnt);

	return sprintf(buf, "%ims\n", (1 + cnt) / 32);
}

static ssize_t tdrp_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t size)
{
	struct typec *tc = dev_get_drvdata(dev);
	int tdrp;

	if (!tc)
		return -ENODEV;

	if (kstrtoint(buf, 0, &tdrp) < 0)
		return -EINVAL;

	if (tdrp < 50 || tdrp > 100)
		return -EINVAL;

	regmap_write(tc->base, TYPEC_TDRP_CNT, (uint16_t)(tdrp * 32 - 1));

	return size;
}
static DEVICE_ATTR_RW(tdrp);

static ssize_t togglecnt_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct typec *tc = dev_get_drvdata(dev);
	int cnt;

	if (!tc)
		return -ENODEV;

	regmap_read(tc->base, TYPEC_TOGGLE_CNT, &cnt);

	return sprintf(buf, "%i\n", cnt);
}

static ssize_t togglecnt_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct typec *tc = dev_get_drvdata(dev);
	int cnt;

	if (!tc)
		return -ENODEV;

	if (kstrtoint(buf, 0, &cnt) < 0)
		return -EINVAL;

	regmap_write(tc->base, TYPEC_TOGGLE_CNT, (uint16_t)cnt);

	return size;
}
static DEVICE_ATTR_RW(togglecnt);

static ssize_t bbat_ccx_state_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct typec *tc = dev_get_drvdata(dev);
	uint32_t state, cc_state, cc_dbg_state, var;

	if (!tc)
		return -ENODEV;

	regmap_read(tc->base, TYPEC_STATUS, &var);
	cc_state = var & TYPEC_CC_CHANNEL_MASK ?
			TYPEC_CC1_CHANNEL : TYPEC_CC2_CHANNEL;

	regmap_read(tc->base, TYPEC_CC_CHANNEL_DET_REG, &var);
	cc_dbg_state = var & TYPEC_CC_CHANNEL_DET_MASK;

	switch (cc_state) {
	case TYPEC_CC1_CHANNEL:
		if (cc_dbg_state == TYPEC_CC1_CHANNEL_DET)
			state = 0;
		else if (cc_dbg_state == TYPEC_CC1_DET_CC2_HIGH)
			state = 1;
		else
			state = 2;
		break;

	case TYPEC_CC2_CHANNEL:
		if (cc_dbg_state == TYPEC_CC2_CHANNEL_DET)
			state = 0;
		else if (cc_dbg_state == TYPEC_CC2_DET_CC1_HIGH)
			state = 1;
		else
			state = 2;
		break;

	default:
		state = 2;
		break;
	}
	dev_info(tc->dev, "cc status=%x\n", var);

	if (state > 1)
		return -EINVAL;
	else
		return sprintf(buf, "%x\n", state);
}
static DEVICE_ATTR_RO(bbat_ccx_state);

static ssize_t bbat_typec_ctrl_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct typec *tc = dev_get_drvdata(dev);
	int cnt;

	if (!tc)
		return -ENODEV;

	regmap_read(tc->base, TYPEC_INT_EN, &cnt);
	if (cnt & (TYPEC_DETACH_INT_EN | TYPEC_ATTACH_INT_EN))
		return sprintf(buf, "%s\n", "enable");
	else
		return sprintf(buf, "%s\n", "disable");
}

static ssize_t bbat_typec_ctrl_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct typec *tc = dev_get_drvdata(dev);
	u32 var;

	if (!tc)
		return -ENODEV;

	if (!strncmp(buf, "disable", 7)) {
		regmap_read(tc->base, TYPEC_MODE, &var);
		var &= ~TYPEC_MODE_MASK;
		var |= TYPEC_MODE_SNK;
		regmap_write(tc->base, TYPEC_MODE, var);
		dev_info(tc->dev, "set typec snk mode!\n");

		regmap_read(tc->base, TYPEC_STATUS, &var);
		tc->cc_status = var & TYPEC_CC_CHANNEL_MASK ?
			TYPEC_CC1_CHANNEL : TYPEC_CC2_CHANNEL;

		regmap_read(tc->base, TYPEC_CC_CHANNEL_DET_REG, &var);
		tc->cc_status |= ((var & TYPEC_CC_CHANNEL_DET_MASK) << 8);

		dev_info(tc->dev, "get typec cc statues [0x%x]\n",
			tc->cc_status);

		regmap_read(tc->base, TYPEC_INT_EN, &var);
		var &= ~(TYPEC_ATTACH_INT_EN | TYPEC_DETACH_INT_EN);
		regmap_write(tc->base, TYPEC_INT_EN, var);
		dev_info(tc->dev, "disable typec attach and detach interrupt!\n");
	}

	if (!strncmp(buf, "enable", 6)) {
		regmap_write(tc->base, TYPEC_MODE, tc->mode);
		dev_info(tc->dev, "restore typec mode [0x%x]\n", tc->mode);

		regmap_read(tc->base, TYPEC_INT_EN, &var);
		var |= TYPEC_ATTACH_INT_EN | TYPEC_DETACH_INT_EN;
		regmap_write(tc->base, TYPEC_INT_EN, var);
		dev_info(tc->dev, "enable typec attach and detach interrupt!\n");
	}

	return size;
}
static DEVICE_ATTR_RW(bbat_typec_ctrl);

static struct attribute *typec_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_connection_state.attr,
	&dev_attr_tccdebounce.attr,
	&dev_attr_tpddebounce.attr,
	&dev_attr_tsleep.attr,
	&dev_attr_tdrp.attr,
	&dev_attr_togglecnt.attr,
	&dev_attr_bbat_ccx_state.attr,
	&dev_attr_bbat_typec_ctrl.attr,
	NULL
};
ATTRIBUTE_GROUPS(typec);

static void typec_interrupt_work(struct work_struct *w)
{
	struct typec *tc = container_of(w, struct typec, isr_work);
	unsigned long flags;
	unsigned int need_report = 1;

	/*
	 * So far, we only concern about SNK/SRC plug in/out events
	 */
	spin_lock_irqsave(&tc->lock, flags);
	if (tc->event & TYPEC_ATTACH_INT) {
		ldo_ss_channel_sw(tc, 1);
		tc->status = CABLE_STATUS_ATTACHED;
		if (tc->state == TYPEC_ATTACHED_SNK) {
			tc->status |= USB_CABLE_DEVICE;
			tc->snk_attach_cnt++;
		} else if (tc->state == TYPEC_ATTACHED_SRC) {
			tc->status |= USB_CABLE_HOST;
			tc->src_attach_cnt++;
		}
	} else if (tc->event & TYPEC_DETACH_INT) {
		ldo_ss_channel_sw(tc, 0);
		tc->status &= ~CABLE_STATUS_ATTACHED;
		if (tc->status & USB_CABLE_DEVICE)
			tc->snk_detach_cnt++;
		else if (tc->status & USB_CABLE_HOST)
			tc->src_detach_cnt++;
	} else {
		need_report = 0;
	}
	spin_unlock_irqrestore(&tc->lock, flags);

	switch (tc->state) {
	case TYPEC_ATTACHWAIT_SNK:
		typec_attachwait_snk_time = ktime_get_ns();
		break;
	case TYPEC_ATTACHED_SRC:
		if (typec_attachwait_snk_time > 0) {
			if (ktime_compare(ns_to_ktime(typec_attachwait_snk_time +
			TYPEC_TIME_WINDOW), ns_to_ktime(ktime_get_ns())) > 0)
				return;
			typec_attachwait_snk_time = 0;
		}
		break;
	default:
		break;
	}

	dev_info(tc->dev,
		"now works as %s and is in %s state,event %x,status %x cc_status %x\n",
		 typec_mode_string[tc->mode],
		 typec_connection_state_string[tc->state],
		 tc->event, tc->status, tc->cc_status);

	if (need_report == 0) {
		pr_err("typec unknown event:%x\n", tc->event);
		return;
	}

#if IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)
	if (tc->dual_role)
		dual_role_instance_changed(tc->dual_role);
#endif
	typec_notifier_call_chain(tc->status);
}

static irqreturn_t typec_interrupt(int irq, void *_tc)
{
	struct typec *tc = (struct typec *)_tc;
	u32 var;

	regmap_read(tc->base, TYPEC_INT_MASK, &var);
	tc->event = var;
	tc->irq_cnt++;

	if (!tc->event)
		return IRQ_NONE;

	regmap_read(tc->base, TYPEC_STATUS, &var);
	tc->state = var & TYPEC_STATE_MASK;
	tc->cc_status = var & TYPEC_CC_CHANNEL_MASK ?
		TYPEC_CC1_CHANNEL : TYPEC_CC2_CHANNEL;
	regmap_read(tc->base, TYPEC_CC_CHANNEL_DET_REG, &var);
	tc->cc_status |= ((var & TYPEC_CC_CHANNEL_DET_MASK) << 8);

	regmap_write(tc->base, TYPEC_INT_CLR, tc->event);

	schedule_work(&tc->isr_work);

	return IRQ_HANDLED;
}

static int typec_set_trims(struct typec *tc)
{
	/*
	 * set TYPEC_ITRIM
	 * TYPEC_RTRIM
	 * TYPEC_VREF from Efuse
	 */
#ifdef CONFIG_OTP_SPRD_PMIC_EFUSE
	unsigned int cc1_rtrim, cc2_rtrim;

	cc1_rtrim = sprd_pmic_efuse_bits_read(EFUSE_CC1_INDEX, EFUSE_CC1_LENGTH);
	dev_info(tc->dev, "Read efuse index %d, length %d, cc1_rtrim 0x%x\n",
		EFUSE_CC1_INDEX, EFUSE_CC1_LENGTH, cc1_rtrim);
	cc2_rtrim = sprd_pmic_efuse_bits_read(EFUSE_CC2_INDEX, EFUSE_CC2_LENGTH);
	dev_info(tc->dev, "Read efuse index %d, length %d, cc2_rtrim 0x%x\n",
		EFUSE_CC2_INDEX, EFUSE_CC2_LENGTH, cc2_rtrim);
	tc->trims.rtrim = ((cc2_rtrim & TYPEC_CC_RTRIM_MASK) << TYPEC_CC2_RTRIM_INDEX) |
		((cc1_rtrim & TYPEC_CC_RTRIM_MASK) << TYPEC_CC1_RTRIM_INDEX);
	dev_info(tc->dev, "Set efuse rtrim 0x%x\n", tc->trims.rtrim);
	regmap_write(tc->base, TYPEC_RTRIM, tc->trims.rtrim);
#endif
	return 0;
}

static void typec_set_mode(struct typec *tc, int mode)
{
	uint32_t val;
	uint32_t tsleep;

	regmap_read(tc->base, TYPEC_MODE, &val);
	if (tc->has_probed && val == mode)
		return;

	/* Reset the module */
	regmap_update_bits(tc->base, TYPEC_MODULE_RST,
			   TYPEC_SOFT_RST_BIT, TYPEC_SOFT_RST_BIT);
	/* Disable RTC */
	regmap_update_bits(tc->base, TYPEC_RTC_EN,
			   TYPEC_RTC_EN_BIT, 0);
	udelay(100);
	regmap_update_bits(tc->base, TYPEC_MODULE_RST,
			   TYPEC_SOFT_RST_BIT, 0);
	/* Set mode */
	regmap_write(tc->base, TYPEC_MODE, mode);
	tc->mode = mode;
	/* Set config */
	val = TYPEC_ENABLE;
	if (mode == DUAL_ROLE_PROP_MODE_UFP)
		val |= tc->usb20_only ? TYPEC_USB20_ONLY : 0;
	if (mode == DUAL_ROLE_PROP_MODE_NONE)
		val &= ~TYPEC_TOGGLE_SLEEP_EN;
	regmap_write(tc->base, TYPEC_CONFIG, val);
	/* Enable interrupt */
	regmap_read(tc->base, TYPEC_INT_EN, &val);
	val |= TYPEC_ATTACH_INT_EN | TYPEC_DETACH_INT_EN;
	regmap_write(tc->base, TYPEC_INT_EN, val);

	if (mode == DUAL_ROLE_PROP_MODE_NONE) {
		tsleep = tc->tsleep * 32 - 1;
		regmap_write(tc->base, TYPEC_SLEEP_CNT1, (uint16_t)tsleep);
		regmap_write(tc->base, TYPEC_SLEEP_CNT2,
			     (uint16_t)(tsleep >> 16));
	}
	/* Enable RTC */
	regmap_update_bits(tc->base, TYPEC_RTC_EN,
		TYPEC_RTC_EN_BIT, TYPEC_RTC_EN_BIT);

	typec_set_trims(tc);

	/* Update state */
	regmap_read(tc->base, TYPEC_STATUS, &val);
	tc->state = val & TYPEC_STATE_MASK;
	if ((tc->status & CABLE_STATUS_ATTACHED) && (tc->has_probed)) {
		dev_info(tc->dev, "Force a disconnect event\n");
		tc->event = TYPEC_DETACH_INT;
		tc->status &= ~CABLE_STATUS_ATTACHED;
		schedule_work(&tc->isr_work);
	}
}

/* Callback for "cat /sys/class/dual_role_usb/<name>/<property>" */
static int dual_role_get_prop(struct dual_role_phy_instance *dual_role,
			      enum dual_role_property prop,
			      unsigned int *val)
{
	struct typec *tc = dev_get_drvdata(dual_role->dev.parent);
	int ret = 0;

	if (!tc)
		return -ENODEV;

	switch (prop) {
	case DUAL_ROLE_PROP_SUPPORTED_MODES:
		*val = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
		break;
	case DUAL_ROLE_PROP_MODE:
		if (tc->status & CABLE_STATUS_ATTACHED) {
			if (tc->status & USB_CABLE_HOST)
				*val = DUAL_ROLE_PROP_MODE_DFP;
			else if (tc->status & USB_CABLE_DEVICE)
				*val = DUAL_ROLE_PROP_MODE_UFP;
		} else
			*val = DUAL_ROLE_PROP_MODE_NONE;

		break;
	case DUAL_ROLE_PROP_PR:
		/* It will be implemented in PMIC 2730 */
		if (tc->status & CABLE_STATUS_ATTACHED) {
			if (tc->status & USB_CABLE_HOST)
				*val = DUAL_ROLE_PROP_PR_SRC;
			else if (tc->status & USB_CABLE_DEVICE)
				*val = DUAL_ROLE_PROP_PR_SNK;
		} else {
			*val = DUAL_ROLE_PROP_PR_NONE;
		}
		break;
	case DUAL_ROLE_PROP_DR:
		if (tc->status & CABLE_STATUS_ATTACHED) {
			if (tc->status & USB_CABLE_HOST)
				*val = DUAL_ROLE_PROP_DR_HOST;
			else if (tc->status & USB_CABLE_DEVICE)
				*val = DUAL_ROLE_PROP_DR_DEVICE;
		} else {
			*val = DUAL_ROLE_PROP_DR_NONE;
		}
		break;
	case DUAL_ROLE_PROP_ACC_MODE:
		if (tc->state == TYPEC_AUDIO_CABLE ||
				tc->state == TYPEC_DEBUG_CABLE)
			*val = DUAL_ROLE_PROP_ACC_MODE_YES;
		else
			*val = DUAL_ROLE_PROP_ACC_MODE_NO;
		break;
	case DUAL_ROLE_PROP_VCONN_SUPPLY:
		/* It will be implemented in PMIC 2730 */
		*val = DUAL_ROLE_PROP_VCONN_SUPPLY_NO;
		break;
	default:
		ret = -ENOTSUPP;
		break;
	}

	return ret;
}

/* Callback for "echo <value> > /sys/class/dual_role_usb/<name>/<property>" */
static int dual_role_set_prop(struct dual_role_phy_instance *dual_role,
			      enum dual_role_property prop,
			      const unsigned int *val)
{
	struct typec *tc = dev_get_drvdata(dual_role->dev.parent);
	int ret = 0;

	if (!tc)
		return -ENODEV;

	switch (prop) {
	case DUAL_ROLE_PROP_MODE:
		typec_set_mode(tc, *val);
		break;
	case DUAL_ROLE_PROP_ACC_MODE:
	case DUAL_ROLE_PROP_SUPPORTED_MODES:
	case DUAL_ROLE_PROP_PR:
	case DUAL_ROLE_PROP_DR:
	case DUAL_ROLE_PROP_VCONN_SUPPLY:
	default:
		ret = -ENOTSUPP;
		break;
	}
	return ret;
}

/* Decides whether userspace can change a specific property */
static int dual_role_is_writeable(struct dual_role_phy_instance *drp,
				  enum dual_role_property prop)
{
	return (prop == DUAL_ROLE_PROP_MODE) ? 1 : 0;
}

static int typec_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct typec *tc;
	struct dual_role_phy_desc *desc;
	uint8_t usb20_only = 0;
	uint32_t mode = 0;
	uint32_t tsleep = 0;
	uint32_t value = 0;
	int ret = 0;

	tc = devm_kzalloc(dev, sizeof(*tc), GFP_KERNEL);
	if (!tc) {
		dev_err(dev, "%s not enough memory\n", __func__);
		return -ENOMEM;
	}

	__typec = tc;

	tc->has_probed = 0;
	platform_set_drvdata(pdev, tc);
	tc->dev = dev;
	tc->irq = platform_get_irq(pdev, 0);
	INIT_WORK(&tc->isr_work, typec_interrupt_work);
	spin_lock_init(&tc->lock);

	tc->base = dev_get_regmap(dev->parent, NULL);
	if (!tc->base) {
		dev_err(dev, "failed to get regmap base!\n");
		goto exit;
	}

	ret = of_property_read_u32(node, "reg", &value);
	if (ret) {
		dev_err(dev, "failed to get reg offset!\n");
		goto exit;
	}
	tc->reg_offset = value;

	ret = of_property_read_u32(node, "mode", &mode);
	if (ret) {
		dev_err(dev, "failed to get port type\n");
		goto exit1;
	}

	ret = of_property_read_u32(node, "ext-ldo-sw", &tc->ldo_sw_en);
	if (ret) {
		dev_err(dev, "failed to get ext-ldo-sw !\n");
		goto exit1;
	}

	tc->ldo_sw_en |= ((is_sp9860g_2h10) && (is_v110_board));

	if (tc->ldo_sw_en) {
		ret = init_usb30_ldo_resource(node, tc, pdev);
		if (ret) {
			dev_err(dev, "failed to init ldo pin:%d\n", ret);
			goto exit1;
		}
	}

	if (mode < DUAL_ROLE_PROP_MODE_UFP || mode > DUAL_ROLE_PROP_MODE_NONE)
		mode = DUAL_ROLE_PROP_MODE_UFP;

	if (mode == DUAL_ROLE_PROP_MODE_UFP) {
		of_property_read_u8(node, "usb-2.0-only", &usb20_only);
		tc->usb20_only = usb20_only;
		dev_info(dev, "usb 2.0 only is %s\n",
			 usb20_only ? "enabled" : "disabled");
	}

	regmap_update_bits(tc->base, TYPEC_MODULE_EN,
			   TYPEC_MODULE_EN_BIT, TYPEC_MODULE_EN_BIT);

	if (mode == DUAL_ROLE_PROP_MODE_NONE) {
		ret = of_property_read_u32(node, "tsleep", &tsleep);
		tc->tsleep = ret ? 1000 : tsleep;
		dev_info(dev, "tsleep is %dms\n", tc->tsleep);

		tsleep = tc->tsleep * 32 - 1;
		regmap_write(tc->base, TYPEC_SLEEP_CNT1, (uint16_t)tsleep);
		regmap_write(tc->base, TYPEC_SLEEP_CNT2,
			     (uint16_t)(tsleep >> 16));
	}

	/* Init efuse trims */
	ret = typec_set_trims(tc);
	if (ret) {
		dev_err(dev, "failed to init efuse trims due to error %d\n",
			ret);
		goto exit1;
	}

	/*
	 * Cause Type-C is enabled as UFP by default.
	 * it's running before this driver,
	 * we therefore should read its status before reset
	 */
	regmap_read(tc->base, TYPEC_STATUS, &tc->state);
	tc->state &= TYPEC_STATE_MASK;
	if (tc->state == TYPEC_ATTACHED_SRC) {
		ldo_ss_channel_sw(tc, 1);
		tc->status = CABLE_STATUS_HOST_CONN;
	} else if (tc->state == TYPEC_ATTACHED_SNK) {
		ldo_ss_channel_sw(tc, 1);
		tc->status = CABLE_STATUS_DEV_CONN;
	} else {
		ldo_ss_channel_sw(tc, 0);
	}
	dev_info(dev, "Type-C is in %s and would work as %s",
		 typec_connection_state_string[tc->state],
		 typec_mode_string[mode]);

	typec_set_mode(tc, mode);

	ret = devm_request_threaded_irq(tc->dev, tc->irq, NULL,
					typec_interrupt,
					IRQF_NO_SUSPEND | IRQF_EARLY_RESUME,
					"typec", tc);
	if (ret) {
		dev_err(dev, "failed to request irq %d due to error %d\n",
			tc->irq, ret);
		goto exit1;
	}

	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		desc = devm_kzalloc(dev,
				    sizeof(struct dual_role_phy_desc),
				    GFP_KERNEL);
		if (!desc) {
			dev_err(dev,
				"unable to allocate dual role descriptor\n");
			goto exit;
		}
		desc->name = "sprd_dual_role_usb";
		desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
		desc->get_property = dual_role_get_prop;
		desc->set_property = dual_role_set_prop;
		desc->properties = typec_properties;
		desc->num_properties = ARRAY_SIZE(typec_properties);
		desc->property_is_writeable = dual_role_is_writeable;
		tc->dual_role = devm_dual_role_instance_register(tc->dev, desc);
		if (tc->dual_role) {
			tc->desc = desc;
			tc->dual_role->drv_data = tc;
		} else
			goto exit1;
	}

	ret = sysfs_create_groups(&dev->kobj, typec_groups);
	if (ret)
		dev_err(dev, "failed to create typec attributes\n");

	tc->has_probed = 1;
	return 0;

exit1:
	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		devm_dual_role_instance_unregister(tc->dev, tc->dual_role);
		devm_kfree(dev, tc->desc);
	}
exit:
	kfree(tc);
	return ret;
}

static int typec_remove(struct platform_device *pdev)
{
	struct typec *tc = platform_get_drvdata(pdev);

	sysfs_remove_groups(&pdev->dev.kobj, typec_groups);
	disable_irq(tc->irq);
	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		devm_dual_role_instance_unregister(tc->dev, tc->dual_role);
		devm_kfree(tc->dev, tc->desc);
	}
	platform_device_unregister(pdev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int typec_suspend(struct device *dev)
{
	struct typec *tc = dev_get_drvdata(dev);
	uint32_t sleep_cnt;

	if (tc->mode != DUAL_ROLE_PROP_MODE_NONE)
		return 0;

	sleep_cnt = tc->tsleep * 5 * 32 - 1;

	regmap_write(tc->base, TYPEC_SLEEP_CNT1, (uint16_t)sleep_cnt);
	regmap_write(tc->base, TYPEC_SLEEP_CNT2, (uint16_t)(sleep_cnt >> 16));

	return 0;
}

static int typec_resume(struct device *dev)
{
	struct typec *tc = dev_get_drvdata(dev);
	uint32_t sleep_cnt;

	if (tc->mode != DUAL_ROLE_PROP_MODE_NONE)
		return 0;

	sleep_cnt = tc->tsleep * 32 - 1;

	regmap_write(tc->base, TYPEC_SLEEP_CNT1, (uint16_t)sleep_cnt);
	regmap_write(tc->base, TYPEC_SLEEP_CNT2, (uint16_t)(sleep_cnt >> 16));

	return 0;
}
#endif

static const struct dev_pm_ops typec_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(typec_suspend, typec_resume)
};

static const struct of_device_id typec_sprd_match[] = {
	{.compatible = "sprd,typec"},
	{},
};
MODULE_DEVICE_TABLE(of, typec_sprd_match);

static struct platform_driver typec_sprd_driver = {
	.probe = typec_probe,
	.remove = typec_remove,
	.driver = {
		.name = "sprd-typec",
		.of_match_table = typec_sprd_match,
		.pm = &typec_dev_pm_ops,
	},
};

static int __init typec_sprd_init(void)
{
	return platform_driver_register(&typec_sprd_driver);
}

static void __exit typec_sprd_exit(void)
{
	platform_driver_unregister(&typec_sprd_driver);
}

late_initcall_sync(typec_sprd_init);
module_exit(typec_sprd_exit);

MODULE_ALIAS("platform: Type-C");
MODULE_AUTHOR("Miao Zhu <miao.zhu@spreadtrum.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Type-C v1.1 SPRD Glue Layer");
