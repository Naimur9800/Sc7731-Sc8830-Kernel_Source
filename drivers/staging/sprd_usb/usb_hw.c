/*
 * Copyright (C) 2012-2015 Spreadtrum Communications Inc.
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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/usb/phy.h>
#include <linux/usb/otg.h>
#include <linux/usb/sprd_usb.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_MFD_SM5504
#include <linux/mfd/sm5504.h>
#endif
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <asm/hardirq.h>
#include <asm/irq.h>
#include "usb_hw.h"
#include "dwc_otg_driver.h"
#include "dwc_otg_core_if.h"

struct usb_priv_data *usb_priv;

#ifndef CONFIG_SPRD_USB_DEVICE_ONLY
extern void usb_otg_cable_detect_work(void *p);
#endif


#ifndef CONFIG_USB_SPRD_FPGA
static void usb_enable_module(int en)
{
	unsigned int mask;
	mask = BIT_AP_AHB_OTG_EB;
	if (en) {
		regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_AHB_EB,
				   mask, mask);
	} else {
		regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_AHB_EB,
				   mask, 0);
	}
	usb_priv->module_en = en;
}

int usb_get_module_enable(void)
{
	unsigned int mask, value;
	int en;

	mask = BIT_AP_AHB_OTG_EB;
	regmap_read(usb_priv->ahb_syscon, REG_AP_AHB_AHB_EB, &value);
	if (mask & value)
		en = 1;
	else
		en = 0;

	return en;
}

int usb_get_module_en(void)
{
	return usb_priv->module_en;
}

static void usb_reset_core(void)
{
	unsigned int mask;

	mask = BIT_AP_AHB_OTG_SOFT_RST;
	regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_AHB_RST,
				   mask, mask);
	mdelay(5);
	regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_AHB_RST,
				   mask, 0);
}

void usb_phy_mode(enum usb_dr_mode mode)
{
	unsigned int mask;

	mask = BIT_AP_AHB_OTG_IDDIG;
	if (mode == USB_DR_MODE_HOST) {
		regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_USB2_CTRL,
				   mask, ~mask);
		usb_phy_vbus_on(usb_priv->hs_phy);
	} else {
		regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_USB2_CTRL,
				   mask, mask);
		usb_phy_vbus_off(usb_priv->hs_phy);
	}
	mask = BIT_AP_AHB_OTG_REMAP;
	regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_USB2_CTRL,
			mask, mask);
	usb_phy_init(usb_priv->hs_phy);
}

enum usb_dr_mode usb_get_phy_mode(void)
{
	unsigned int mask, value;
	enum usb_dr_mode mode;

	mask = BIT_AP_AHB_OTG_IDDIG;

	regmap_read(usb_priv->ahb_syscon, REG_AP_AHB_USB2_CTRL, &value);
	if (mask & value)
		mode = USB_DR_MODE_PERIPHERAL;
	else
		mode = USB_DR_MODE_HOST;

	return mode;
}

static int sprd_dwc_ext_notifier(struct notifier_block *nb,
				unsigned long action, void *data)
{
#ifndef CONFIG_SPRD_USB_DEVICE_ONLY
	dwc_otg_device_t *otg_dev =
		container_of(nb, dwc_otg_device_t, hot_plug_nb);
#endif
	unsigned long flags;
	int ret = 0;
	int vbus_irq = usb_get_vbus_irq();
	int is_device = USB_CABLE_DEVICE & action ? 1 : 0;
	struct gadget_wrapper *g = gadget_wrapper;

	if (is_device && vbus_irq != -1)
		return -EINVAL;

	spin_lock_irqsave(&g->lock, flags);
	switch (action) {
	case CABLE_STATUS_DEV_CONN:
		g->vbus_active = true;
		break;
	case CABLE_STATUS_HOST_CONN:
		g->vbus_active = true;
		break;
	case CABLE_STATUS_DEV_DISCONN:
		g->vbus_active = false;
		break;
	case CABLE_STATUS_HOST_DISCONN:
		g->vbus_active = false;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	spin_unlock_irqrestore(&g->lock, flags);

	if (ret) {
		pr_err("!!!ignore type-c action 0x%lx, dr_mode %d\n",
			action, g->dr_mode);
		return ret;
	}

	if (is_device) {
		queue_work(g->detect_wq, &g->detect_work);
	} else {
#ifndef CONFIG_SPRD_USB_DEVICE_ONLY
		DWC_WORKQ_SCHEDULE_DELAYED(
			otg_dev->core_if->wq_otg,
			usb_otg_cable_detect_work,
			otg_dev, 50,
			"OTG cable connect state change");
#endif
	}

	return ret;
}

ssize_t usb_of_init(struct platform_device *_dev)
{
	int retval;
	struct device_node *np = _dev->dev.of_node;
	dwc_otg_device_t *otg_dev = platform_get_drvdata(_dev);

	usb_priv = devm_kzalloc(&_dev->dev, sizeof(*usb_priv), GFP_KERNEL);
	if (!usb_priv)
		return -ENOMEM;

	usb_priv->ahb_syscon =
	    syscon_regmap_lookup_by_phandle(np, "sprd,syscon-ap-ahb");
	if (IS_ERR(usb_priv->ahb_syscon))
		return PTR_ERR(usb_priv->ahb_syscon);

	usb_priv->ref_clk = devm_clk_get(&_dev->dev, "ref_clk");
	if (IS_ERR(usb_priv->ref_clk))
		return PTR_ERR(usb_priv->ref_clk);

	usb_priv->gpio_vbus = of_get_named_gpio(np, "vbus-gpios", 0);
	if (!gpio_is_valid(usb_priv->gpio_vbus)) {
		pr_info("invalid usb vbus gpio: %d\n", usb_priv->gpio_vbus);
	}

	usb_priv->gpio_id = of_get_named_gpio(np, "id-gpios", 0);
	if (!gpio_is_valid(usb_priv->gpio_id)) {
		pr_info("invalid usb id gpio: %d\n", usb_priv->gpio_id);
	}

	retval = of_property_read_string(_dev->dev.of_node, "sprd,cable-detection-method",
			&usb_priv->cable_detect);
	if (retval) {
		pr_err("fail to get cable detection method\n");
		return retval;
	}
	pr_info("dwc cable detection method is %s\n",
		usb_priv->cable_detect);

	if (!strcmp(usb_priv->cable_detect, "typec")) {
		otg_dev->hot_plug_nb.notifier_call = sprd_dwc_ext_notifier;
		otg_dev->hot_plug_nb.priority = 0;
		register_ext_notifier(&otg_dev->hot_plug_nb);
	}

	usb_priv->hs_phy = devm_usb_get_phy_by_phandle(&_dev->dev, "usb-phy", 0);
	if (IS_ERR(usb_priv->hs_phy)) {
		pr_info("unable to get usb2.0 phy device\n");
		return PTR_ERR(usb_priv->hs_phy);
	}
	usb_priv->module_en = 0;
	usb_priv->mode = USB_DR_MODE_UNKNOWN;

	return 0;
}

void usb_phy_ahb_rst(void)
{
	usb_phy_reset(usb_priv->hs_phy);
}

enum usb_dr_mode usb_get_udc_mode(void)
{
	return usb_priv->mode;
}

void udc_enable(enum usb_dr_mode mode)
{
	if (clk_prepare_enable(usb_priv->ref_clk))
		pr_err("failed to enable usb2 ref clock\n");
	usb_phy_mode(mode);
	mdelay(5);
	usb_enable_module(1);
	usb_reset_core();
	usb_priv->mode = mode;
	pr_info("[%pf]:udc_enable to udc_en(%d)(%d) udc_mode(%d)(%d)\n",
		__builtin_return_address(0),
		usb_get_module_enable(), usb_get_module_en(),
		usb_get_udc_mode(), usb_get_phy_mode());
}

void udc_disable(void)
{
	usb_priv->mode = USB_DR_MODE_UNKNOWN;
	usb_enable_module(0);
	mdelay(5);
	usb_phy_shutdown(usb_priv->hs_phy);
	clk_disable_unprepare(usb_priv->ref_clk);
	pr_info("[%pf]:udc_disable to udc_en(%d)(%d) udc_mode(%d)(%d)\n",
		__builtin_return_address(0),
		usb_get_module_enable(), usb_get_module_en(),
		usb_get_udc_mode(), usb_get_phy_mode());
}

int usb_alloc_vbus_irq(int gpio)
{
	int irq;

	gpio_request(gpio, "sprd_otg");
	gpio_direction_input(gpio);
	irq = gpio_to_irq(gpio);
	return irq;
}

void usb_free_vbus_irq(int irq, int gpio)
{
	gpio_free(gpio);
}

int usb_get_vbus_irq(void)
{
	int value;

	value = gpio_to_irq(usb_priv->gpio_vbus);

	return value;
}

int usb_get_vbus_state(void)
{
	int value;
	value = gpio_get_value(usb_priv->gpio_vbus);
	return !!value;
}

void usb_set_vbus_irq_type(int irq, int irq_type)
{
	if (irq_type == VBUS_PLUG_IN)
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
	else if (irq_type == VBUS_PLUG_OUT)
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
	else {
		pr_warning("error type for usb vbus\n");
	}

	return;
}

#ifndef DWC_DEVICE_ONLY
void charge_pump_set(int gpio, int state)
{
	if (gpio_is_valid(gpio)) {
		gpio_request(gpio, "chg_ pump");
		gpio_direction_output(gpio, 1);
		gpio_set_value(gpio, state);
	}

}

int usb_alloc_id_irq(int gpio)
{
	int irq;

	gpio_request(gpio, "USB OTG CABLE");
	gpio_direction_input(gpio);
	irq = gpio_to_irq(gpio);

	return irq;
}

void usb_free_id_irq(int irq, int gpio)
{
	gpio_free(gpio);
}

int usb_get_id_irq(void)
{
	int value;

	value = gpio_to_irq(usb_priv->gpio_id);

	return value;
}

#ifdef CONFIG_MFD_SM5504
extern bool sm5504_get_otg_status(void);
#endif
/*
 *if otg cable is connected , id state =0
 *as host turn on vbus, in this case shouldn't call this handler
 */
int usb_get_id_state(void)
{
#ifdef CONFIG_MFD_SM5504
	int value = 0;
	value = !sm5504_get_otg_status();
	return value;
#else
	int value;
	if (!gpio_is_valid(usb_priv->gpio_id))
		return -EINVAL;

	value = gpio_get_value(usb_priv->gpio_id);
	return !!value;
#endif
}

void usb_set_id_irq_type(int irq, int irq_type)
{
	if (irq_type == OTG_CABLE_PLUG_IN)
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
	else if (irq_type == OTG_CABLE_PLUG_OUT)
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
	else {
		pr_warning("error type for usb vbus\n");
	}

	return;
}
#endif

#else

#define BIT_AP_AHB_OTG_ID 				BIT(3)
static void usb_phy_mode(int ishost)
{
	unsigned int mask1;
	mask1 = BIT_AP_AHB_OTG_ID;
	if (ishost) {
		regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_USB2_CTRL,
				   mask1, ~mask1);
	} else {
		regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_USB2_CTRL,
				   mask1, mask1);
	}
}

static void usb_enable_module(int en)
{
	unsigned int mask2;
//	mask1 = BIT_AON_APB_USB_PHY_PD;
	mask2 = BIT_AP_AHB_OTG_EB;
	if (en) {
//		regmap_update_bits(usb_priv->aon_apb_syscon,
//				   REG_AON_APB_PWR_CTRL, mask1, ~mask1);
		regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_AHB_EB,
				   mask2, mask2);
	} else {
//		regmap_update_bits(usb_priv->aon_apb_syscon,
//				   REG_AON_APB_PWR_CTRL, mask1, mask1);
		regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_AHB_EB,
				   mask2, ~mask2);
	}
#ifdef DWC_DEVICE_ONLY
	mask2 = BIT_AP_AHB_OTG_IDDIG;
	regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_USB2_CTRL,
			   mask2, mask2);
#endif
}


ssize_t usb_of_init(struct platform_device *_dev)
{
	struct device_node *np = _dev->dev.of_node;

	usb_priv = devm_kzalloc(&_dev->dev, sizeof(*usb_priv), GFP_KERNEL);
	if (!usb_priv)
		return -ENOMEM;

	usb_priv->ahb_syscon =
	    syscon_regmap_lookup_by_phandle(np, "sprd,syscon-ap-ahb");
	if (IS_ERR(usb_priv->ahb_syscon))
		return PTR_ERR(usb_priv->ahb_syscon);
#if 0
	usb_priv->aon_apb_syscon =
	    syscon_regmap_lookup_by_phandle(np, "sprd,syscon-aon-apb");
	if (IS_ERR(usb_priv->aon_apb_syscon))
		return PTR_ERR(usb_priv->aon_apb_syscon);

	if (of_property_read_u32
	    (np, "sprd,tune-value", &usb_priv->phy_tune_value)) {
		pr_info("read tune_value error\n");
		return -ENODEV;
	}
	pr_info("Usb_hw.c: [%s]usb phy tune: 0x%x\n", __FUNCTION__,
		usb_priv->phy_tune_value);

	usb_priv->gpio_vbus = of_get_named_gpio(np, "vbus-gpios", 0);
	if (!gpio_is_valid(usb_priv->gpio_vbus)) {
		pr_info("invalid usb vbus gpio: %d\n", usb_priv->gpio_vbus);
	}

	usb_priv->gpio_id = of_get_named_gpio(np, "id-gpios", 0);
	if (!gpio_is_valid(usb_priv->gpio_id)) {
		pr_info("invalid usb id gpio: %d\n", usb_priv->gpio_id);
	}

	usb_priv->gpio_otg_boost = of_get_named_gpio(np, "boost-gpios", 0);
	if (!gpio_is_valid(usb_priv->gpio_otg_boost)) {
		pr_info("invalid usb boost gpio: %d\n",
			usb_priv->gpio_otg_boost);
	}
	usb_regulator = devm_regulator_get(&_dev->dev,"usb");
	if (IS_ERR(usb_regulator)) {
		pr_info("get usb supply regulator failed\n");
	}
#endif
	return 0;
}

void usb_phy_ahb_rst(void)
{
	//TODO:reg in G4
#if 0
	unsigned int mask;
	mask = BIT_AP_AHB_OTG_PHY_SOFT_RST;
	regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_AHB_RST, mask,
			   mask);
	mdelay(3);
	regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_AHB_RST, mask,
			   ~mask);
	mdelay(3);
#endif
}

static void usb_startup(void)
{
	unsigned int mask;
	//usb_ldo_switch(1);
	mdelay(10);
	usb_enable_module(1);
	mdelay(2);
	mask = BIT_AP_AHB_OTG_SOFT_RST;
	regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_AHB_RST, mask,
			   mask);
	mdelay(5);
	regmap_update_bits(usb_priv->ahb_syscon, REG_AP_AHB_AHB_RST, mask,
			   ~mask);

	mdelay(3);
}

void udc_enable(void)
{
	pr_info("%s \n", __func__);
	usb_startup();
	usb_phy_mode(0);
}

void udc_disable(void)
{
	pr_info("%s \n", __func__);
	usb_enable_module(0);
	//usb_ldo_switch(0);
}



int usb_alloc_vbus_irq(int gpio)
{return 1;}

void usb_free_vbus_irq(int irq,int gpio)
{}

int usb_get_vbus_irq(void)
{return 1;}

int usb_get_vbus_state(void)
{return 1;}

void charge_pump_set(int gpio,int state)
{}

int usb_alloc_id_irq(int gpio)
{return 1;}

void usb_free_id_irq(int irq,int gpio)
{}

int usb_get_id_irq(void)
{return 1;}

int usb_get_id_state(void)
{return 1;}

void usb_set_vbus_irq_type(int irq, int irq_type)
{}

void usb_set_id_irq_type(int irq, int irq_type)
{}

#endif


EXPORT_SYMBOL(udc_disable);
EXPORT_SYMBOL(udc_enable);
