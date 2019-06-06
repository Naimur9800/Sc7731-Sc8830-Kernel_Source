/*
 *Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 *This software is licensed under the terms of the GNU General Public
 *License version 2, as published by the Free Software Foundation, and
 *may be copied, distributed, and modified under those terms.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 */

#define pr_fmt(__fmt) "[sprd-adf][%20s] "__fmt, __func__

#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <video/adf_notifier.h>

static struct notifier_block  notif;
static wait_queue_head_t fb_state_wq;
static DEFINE_SPINLOCK(fb_state_lock);
static enum {
	FB_STATE_STOPPED_DRAWING,
	FB_STATE_REQUEST_STOP_DRAWING,
	FB_STATE_DRAWING_OK,
} fb_state;

/* tell userspace to stop drawing, wait for it to stop */
static void stop_drawing_early_suspend(void)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&fb_state_lock, irq_flags);
	fb_state = FB_STATE_STOPPED_DRAWING;
	spin_unlock_irqrestore(&fb_state_lock, irq_flags);
	wake_up_all(&fb_state_wq);
}

/* tell userspace to start drawing */
static void start_drawing_late_resume(void)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&fb_state_lock, irq_flags);
	fb_state = FB_STATE_DRAWING_OK;
	spin_unlock_irqrestore(&fb_state_lock, irq_flags);
	wake_up(&fb_state_wq);
}

static ssize_t wait_for_fb_sleep_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	int rc;

	rc = wait_event_interruptible(fb_state_wq,
				       fb_state == FB_STATE_STOPPED_DRAWING);
	if (rc && fb_state != FB_STATE_STOPPED_DRAWING)
		return rc;

	s += sprintf(buf, "sleeping");

	return s - buf;
}

static ssize_t wait_for_fb_wake_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	int rc;

	rc = wait_event_interruptible(fb_state_wq,
				       fb_state == FB_STATE_DRAWING_OK);
	if (rc && fb_state != FB_STATE_DRAWING_OK)
		return rc;

	s += sprintf(buf, "awake");

	return s - buf;
}

#define _ro_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0444,			\
	},					\
	.show	= _name##_show,			\
	.store	= NULL,		\
}

_ro_attr(wait_for_fb_sleep);
_ro_attr(wait_for_fb_wake);

static struct attribute *g[] = {
	&wait_for_fb_sleep_attr.attr,
	&wait_for_fb_wake_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static int fb_wait_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct adf_notifier_event *evdata = data;
	int adf_blank;

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != ADF_EVENT_BLANK)
		return 0;

	adf_blank = *(int *)evdata->data;

	switch (adf_blank) {
	case DRM_MODE_DPMS_ON:
		start_drawing_late_resume();
		break;
	case DRM_MODE_DPMS_OFF:
		stop_drawing_early_suspend();
		break;
	default:
		pr_err("error blank event\n");
		break;
	}
	return NOTIFY_OK;
}

static int register_notifier_client(void)
{
	memset(&notif, 0, sizeof(struct notifier_block));
	notif.notifier_call = fb_wait_callback;
	notif.priority = -100;
	return adf_register_client(&notif);
}

static int __init refnotify_sysfs_init(void)
{
	int ret;

	init_waitqueue_head(&fb_state_wq);
	fb_state = FB_STATE_DRAWING_OK;

	ret = sysfs_create_group(power_kobj, &attr_group);
	if (ret) {
		pr_err("create refnotify attr node failed, ret=%d\n", ret);
		return ret;
	}

	register_notifier_client();

	return 0;
}

static void  __exit refnotify_sysfs_exit(void)
{
	adf_unregister_client(&notif);
	sysfs_remove_group(power_kobj, &attr_group);
}

module_init(refnotify_sysfs_init);
module_exit(refnotify_sysfs_exit);

