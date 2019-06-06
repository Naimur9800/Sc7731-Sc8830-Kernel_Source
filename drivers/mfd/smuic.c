/*
 * Copyright (C) 2015, SAMSUNG Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <soc/sprd/adi.h>
#include <soc/sprd/sci_glb_regs.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/mfd/muic_noti.h>
#include <linux/of_gpio.h>

struct smuic {
	struct work_struct detect_work;
	struct workqueue_struct *detect_wq;
	int vbus;
	spinlock_t slock;
	struct mutex mlock;
	int gpio_irq;
	bool is_usb;
};

enum adapter_type {
	ADP_TYPE_UNKNOWN = 0,	/* unknown */
	ADP_TYPE_CDP = 1,	/* Charging Downstream Port, USB&standard charger */
	ADP_TYPE_DCP = 2,	/* Dedicated Charging Port, standard charger */
	ADP_TYPE_SDP = 4,	/* Standard Downstrea Port, USB and nonstandard charge */
};

static void usb_detect_works(struct work_struct *);
static int simp_charger_is_adapter(void);
static void simp_charger_plugin(struct smuic *);
static void simp_charger_plugout(struct smuic *);
int epmic_event_handler(int level);

//#define cable_change_callback sec_charger_cb
//extern void sec_charger_cb(u8 attached);

static ATOMIC_NOTIFIER_HEAD(adapter_notifier_list);
int register_adapter_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&adapter_notifier_list, nb);
}
EXPORT_SYMBOL(register_adapter_notifier);
int unregister_adapter_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&adapter_notifier_list, nb);
}
EXPORT_SYMBOL(unregister_adapter_notifier);

static void usb_detect_works(struct work_struct *work)
{
	unsigned long flags;
	int plug_in;
	struct smuic *info = container_of(work, struct smuic, detect_work);


	spin_lock_irqsave(&info->slock, flags);
	plug_in = info->vbus;
	spin_unlock_irqrestore(&info->slock, flags);

	mutex_lock(&info->mlock);
	if (plug_in){
		printk("smuic: usb detect plug in,vbus pull up\n");
		simp_charger_plugin(info);
	} else {
		printk("smuic: usb detect plug out,vbus pull down\n");
		simp_charger_plugout(info);
	}
	mutex_unlock(&info->mlock);

}

static irqreturn_t usb_detect_handler(int irq, void *data)
{
	int value;
	unsigned long flags;
	struct smuic *info = (struct muic*)data;

	value = !!gpio_get_value(info->gpio_irq);
	if (value)
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
	else
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);

	spin_lock_irqsave(&info->slock, flags);
	info->vbus = value;
	spin_unlock_irqrestore(&info->slock, flags);

	queue_work(info->detect_wq, &info->detect_work);

	return IRQ_HANDLED;
}

static int simp_charger_is_adapter(void)
{
	int charger_status;
	int ret;

	charger_status = sci_adi_read(ANA_REG_GLB_CHGR_STATUS)
	    & (BIT_CDP_INT | BIT_DCP_INT | BIT_SDP_INT);

	switch (charger_status) {
	case BIT_CDP_INT:
		ret = ADP_TYPE_CDP;
		break;
	case BIT_DCP_INT:
		ret = ADP_TYPE_DCP;
		break;
	case BIT_SDP_INT:
		ret = ADP_TYPE_SDP;
		break;
	default:
		ret = ADP_TYPE_SDP;
		break;
	}
	return ret;
}


static void simp_charger_plugin(struct smuic *data)
{
	int ret;
	struct muic_notifier_param param;

	ret = simp_charger_is_adapter();
	if (ret == ADP_TYPE_DCP) {
		pr_info("smuic: TA attached!\n");
		data->is_usb = false;
		param.cable_type = MUIC_CABLE_TYPE_REGULAR_TA;
		atomic_notifier_call_chain(&adapter_notifier_list, MUIC_CHG_ATTACH_NOTI, &param);
		
	} else {
		pr_info("smuic: USB attached!\n");
		data->is_usb = true;
		param.cable_type = MUIC_CABLE_TYPE_USB;
		param.vbus_status = 1;
		atomic_notifier_call_chain(&adapter_notifier_list, MUIC_USB_ATTACH_NOTI, &param);
		epmic_event_handler(1);
	}
}

static void simp_charger_plugout(struct smuic *data)
{
	struct muic_notifier_param param;

	param.cable_type = MUIC_CABLE_TYPE_NONE;
	if (data->is_usb) {
		param.vbus_status = 0;
		atomic_notifier_call_chain(&adapter_notifier_list, MUIC_USB_DETACH_NOTI, &param);
		epmic_event_handler(0);
		pr_info("smuic: USB detached!\n");
	} else {
		atomic_notifier_call_chain(&adapter_notifier_list, MUIC_CHG_DETACH_NOTI, &param);
		pr_info("smuic: TA detached!\n");
	}
}

static int __init smuic_init(void)
{
	int ret;
	int plugirq;
	static struct smuic *info;

	info = kzalloc(sizeof(struct smuic), GFP_KERNEL);
	if(!info) {
		pr_err("smuic: failed to alloc smuic info!\n");
		return;
	}

#ifdef CONFIG_OF
	struct device_node *np;

	np = of_find_node_by_name(NULL, "sec-smuic");
	if (!np) {
		pr_err("smuic: can't find sec,smuic\n");
		return -EINVAL;
	} else {
		ret = of_get_gpio(np, 0);
		if (ret < 0) {
			pr_err("smuic: failed to get GPIO irq\n");
			return -EINVAL;
		}
		info->gpio_irq = ret;
	}
#endif

	mutex_init(&info->mlock);
	INIT_WORK(&info->detect_work, usb_detect_works);
	info->detect_wq = create_singlethread_workqueue("usb_detect_wq");
	if (!info->detect_wq) {
		pr_err("smuic: failed to create workqueue\n");
		return -ENOMEM;
	}

	plugirq = gpio_to_irq(info->gpio_irq);
	ret = request_irq(plugirq, usb_detect_handler,
				IRQF_ONESHOT | IRQF_NO_SUSPEND, "usb_detect", info);
	if (ret) {
		pr_err("smuic: failed to request_irq\n");
		return -EINVAL;
	}

	pr_info("smuic: successfully initialized!\n");

	return 0;
}
late_initcall_sync(smuic_init);

MODULE_AUTHOR("SEC");
MODULE_DESCRIPTION("Identify TA or USB");
MODULE_LICENSE("GPL");
