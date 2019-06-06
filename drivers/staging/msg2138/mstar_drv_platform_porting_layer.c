/*==============================================================================
 *
 * Copyright (c) 2006-2012 MStar Semiconductor, Inc.
 * All rights reserved.
 *
 * Unless otherwise stipulated in writing, any and all information contained
 * herein regardless in any format shall remain the sole proprietary of
 * MStar Semiconductor Inc. and be kept in strict confidence
 * (??MStar Confidential Information??) by the recipient.
 * Any unauthorized act including without limitation unauthorized disclosure,
 * copying, use, reproduction, sale, distribution, modification, disassembling,
 * reverse engineering and compiling of the contents of MStar Confidential
 * Information is unlawful and strictly prohibited. MStar hereby reserves the
 * rights to any and all damages, losses, costs and expenses resulting
 therefrom.
 *
==============================================================================*/

/**
 *
 * @file    mstar_drv_platform_porting_layer.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

/*=============================================================*/
/* INCLUDE FILE */
/*=============================================================*/
#include <linux/kthread.h>
#include <linux/err.h>

#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_interface.h"
#include "mstar_drv_common.h"
/*=============================================================*/
/* LOCAL VARIABLE DEFINITION */
/*=============================================================*/

struct mutex g_mutex;
static struct work_struct _g_finger_touch_work;
static struct workqueue_struct *gfingertouch_workqueue;

#ifdef CONFIG_HAS_EARLYSUSPEND
#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
static struct early_suspend _g_early_suspend;
#endif
#endif

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifndef CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
static DECLARE_WAIT_QUEUE_HEAD(_g_waiter);
static struct task_struct *_g_thread;/* = NULL;*/
static int _g_tpd_flag;/* = 0;*/
#endif/* CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

/*=============================================================*/
/* GLOBAL VARIABLE DEFINITION */
/*=============================================================*/

struct input_dev *g_input_device;/* = NULL;*/
static int _g_irq = -1;

static struct task_struct *thread;/* = NULL;*/
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag;/* = 0;*/

/*=============================================================*/
/* LOCAL FUNCTION DEFINITION */
/*=============================================================*/

/* read data through I2C then report data to input sub-system when interrupt
 occurred */
static void _platform_finger_touch_dowork(struct work_struct *p_work)
{
	LOGTP_FUNC();

	mutex_lock(&g_mutex);

	fwic_handle_finger_touch(NULL, 0);

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
/*    enable_irq(MS_TS_MSG_IC_GPIO_INT); */
	enable_irq(_g_irq);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif

	mutex_unlock(&g_mutex);
}

static int touch_event_handler(void *unused)
{
	struct sched_param param = {.sched_priority = 5 };
	sched_setscheduler(current, SCHED_RR, &param);

	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, (0 != tpd_flag));
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
		mutex_lock(&g_mutex);
		fwic_handle_finger_touch(NULL, 0);
		mutex_unlock(&g_mutex);
	} while (!kthread_should_stop());

	return 0;
}

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
/* The interrupt service routine will be triggered when interrupt occurred */
static irqreturn_t _platform_finger_interrupt_handler(s32 n_irq,
	void *p_device_id)
{
	/* LOGTP_FUNC(); */
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;

	/* disable_irq_nosync(MS_TS_MSG_IC_GPIO_INT); */
	/* disable_irq_nosync(_g_irq); */
	/* queue_work(gfingertouch_workqueue, &_g_finger_touch_work); */

	/* return IRQ_HANDLED; */
	/* return IRQ_WAKE_THREAD; */
}
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
static void _platform_finger_interrupt_handler(void)
{
#ifdef CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	queue_work(gfingertouch_workqueue, &_g_finger_touch_work);
#else
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	_g_tpd_flag = 1;
	wake_up_interruptible(&_g_waiter);
#endif/* CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM */
}

static int _platform_finger_touch_handler(void *p_unused)
{
	struct sched_param param = {.sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);

	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(_g_waiter, _g_tpd_flag != 0);
		_g_tpd_flag = 0;

		set_current_state(TASK_RUNNING);

		mutex_lock(&g_mutex);

		fwic_handle_finger_touch(NULL, 0);

		mutex_unlock(&g_mutex);

		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	} while (!kthread_should_stop());

	return 0;
}
#endif

/*=============================================================*/
/* GLOBAL FUNCTION DEFINITION */
/*=============================================================*/

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
void platform_device_regulator_poweron(void)
{
#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
	s32 n_retval = 0;

	LOGTP_FUNC();

	n_retval = regulator_set_voltage(g_reg_vdd, 2800000, 2800000);

	if (n_retval)
		LOGTP_ERRO("Could not set to 2800mv.\n");

	n_retval = regulator_enable(g_reg_vdd);

	mdelay(20);/* mdelay(100); */
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
hw_poweron(MT6323_POWER_LDO_VGP1, VOL_2800, "TP");
#endif
}
#endif/* CONFIG_ENABLE_REGULATOR_POWER_ON */

void platform_device_poweron(void)
{
	LOGTP_FUNC();

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
	gpio_direction_output(MS_TS_MSG_IC_GPIO_RST, 1);
/*    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1); */
/*    mdelay(100); */
	gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 0);
	mdelay(10);
	gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1);
	mdelay(50);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
	mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
	mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ZERO);

#ifdef TPD_CLOSE_POWER_IN_SLEEP
	hw_powerdown(TPD_POWER_SOURCE, "TP");
	mdelay(100);
	hw_poweron(TPD_POWER_SOURCE, VOL_2800, "TP");
	mdelay(10);/* reset pulse */
#endif/* TPD_CLOSE_POWER_IN_SLEEP */

	mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ONE);
	mdelay(180);/* wait stable */
#endif
}

void platform_device_poweroff(void)
{
	LOGTP_FUNC();

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
/*    gpio_direction_output(MS_TS_MSG_IC_GPIO_RST, 0); */
	gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 0);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
	mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
	mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ZERO);
#ifdef TPD_CLOSE_POWER_IN_SLEEP
	hw_powerdown(TPD_POWER_SOURCE, "TP");
#endif/* TPD_CLOSE_POWER_IN_SLEEP */
#endif
}

void platform_device_reset_hw(void)
{
	LOGTP_FUNC();

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
	gpio_direction_output(MS_TS_MSG_IC_GPIO_RST, 1);
/*    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1); */
	gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 0);
	mdelay(10);
	gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1);
	mdelay(50);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
	mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
	mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ONE);
	mdelay(10);
	mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
	mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ZERO);
	mdelay(50);
	mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
	mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ONE);
	mdelay(50);
#endif
}

void platform_disable_finger_touchreport(void)
{
	LOGTP_FUNC();

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
/*    disable_irq(MS_TS_MSG_IC_GPIO_RST); */
	disable_irq(_g_irq);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
}

void platform_enable_finger_touchreport(void)
{
	LOGTP_FUNC();

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
/*    enable_irq(MS_TS_MSG_IC_GPIO_RST); */
	enable_irq(_g_irq);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
}

void platform_finger_pressed(s32 n_x, s32 n_y, s32 n_pressure, s32 n_id)
{
	LOGTP_FUNC();
	LOGTP_INFO("point touch pressed\n");

	input_report_key(g_input_device, BTN_TOUCH, 1);
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
	input_report_abs(g_input_device, ABS_MT_TRACKING_ID, n_id);
#endif/* CONFIG_ENABLE_CHIP_MSG26XXM */
	input_report_abs(g_input_device, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(g_input_device, ABS_MT_WIDTH_MAJOR, 1);
	input_report_abs(g_input_device, ABS_MT_POSITION_X, n_x);
	input_report_abs(g_input_device, ABS_MT_POSITION_Y, n_y);

	input_mt_sync(g_input_device);

}

void platform_finger_touch_released(s32 n_x, s32 n_y)
{
	LOGTP_FUNC();
	LOGTP_INFO("point touch released\n");

	input_report_key(g_input_device, BTN_TOUCH, 0);
	input_mt_sync(g_input_device);

}

s32 platform_device_initialize(struct i2c_client *p_client)
{
	s32 n_retval = 0;

	LOGTP_FUNC();

	mutex_init(&g_mutex);

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
	/* allocate an input device */
	g_input_device = input_allocate_device();
	if (g_input_device == NULL) {
		LOGTP_ERRO("input device allocation failed\n");
		return -ENOMEM;
	}

	g_input_device->name = MSG_TP_IC_NAME;
	g_input_device->phys = "I2C";
	g_input_device->dev.parent = &p_client->dev;
	g_input_device->id.bustype = BUS_I2C;

	/* set the supported event type for input device */
	__set_bit(EV_KEY, g_input_device->evbit);
	__set_bit(EV_ABS, g_input_device->evbit);
	__set_bit(EV_SYN, g_input_device->evbit);
	/* __set_bit(BTN_TOUCH, g_input_device->keybit); */
	__set_bit(ABS_MT_TOUCH_MAJOR, g_input_device->absbit);
	__set_bit(ABS_MT_POSITION_X, g_input_device->absbit);
	__set_bit(ABS_MT_POSITION_Y, g_input_device->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, g_input_device->absbit);

	__set_bit(KEY_MENU, g_input_device->keybit);
	__set_bit(KEY_BACK, g_input_device->keybit);
	__set_bit(KEY_HOME, g_input_device->keybit);

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	input_set_capability(g_input_device, EV_KEY, KEY_POWER);
	input_set_capability(g_input_device, EV_KEY, KEY_UP);
	input_set_capability(g_input_device, EV_KEY, KEY_DOWN);
	input_set_capability(g_input_device, EV_KEY, KEY_LEFT);
	input_set_capability(g_input_device, EV_KEY, KEY_RIGHT);
	input_set_capability(g_input_device, EV_KEY, KEY_W);
	input_set_capability(g_input_device, EV_KEY, KEY_Z);
	input_set_capability(g_input_device, EV_KEY, KEY_V);
	input_set_capability(g_input_device, EV_KEY, KEY_O);
	input_set_capability(g_input_device, EV_KEY, KEY_M);
	input_set_capability(g_input_device, EV_KEY, KEY_C);
	input_set_capability(g_input_device, EV_KEY, KEY_E);
	input_set_capability(g_input_device, EV_KEY, KEY_S);
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

/*
#ifdef CONFIG_TP_HAVE_KEY
    set_bit(TOUCH_KEY_MENU, g_input_device->keybit); //Menu
    set_bit(TOUCH_KEY_HOME, g_input_device->keybit); //Home
    set_bit(TOUCH_KEY_BACK, g_input_device->keybit); //Back
    set_bit(TOUCH_KEY_SEARCH, g_input_device->keybit); //Search
#endif
*/

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
	input_set_abs_params(g_input_device, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif/* CONFIG_ENABLE_CHIP_MSG26XXM */
	input_set_abs_params(g_input_device, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(g_input_device, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(g_input_device, ABS_MT_POSITION_X, 0,
	TOUCH_SCREEN_X_MAX, 0, 0);
	input_set_abs_params(g_input_device, ABS_MT_POSITION_Y, 0,
	TOUCH_SCREEN_Y_MAX, 0, 0);

	/* register the input device to input sub-system */
	n_retval = input_register_device(g_input_device);
	if (n_retval < 0)
		LOGTP_ERRO("Unable to register touch input device\n");

#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
	g_input_device = tpd->dev;

/*    g_input_device->name = p_client->name; */
	g_input_device->phys = "I2C";
	g_input_device->dev.parent = &p_client->dev;
	g_input_device->id.bustype = BUS_I2C;

	/* set the supported event type for input device */
	set_bit(EV_ABS, g_input_device->evbit);
	set_bit(EV_SYN, g_input_device->evbit);
	set_bit(EV_KEY, g_input_device->evbit);
	set_bit(BTN_TOUCH, g_input_device->keybit);
	set_bit(INPUT_PROP_DIRECT, g_input_device->propbit);

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	input_set_capability(g_input_device, EV_KEY, KEY_POWER);
	input_set_capability(g_input_device, EV_KEY, KEY_UP);
	input_set_capability(g_input_device, EV_KEY, KEY_DOWN);
	input_set_capability(g_input_device, EV_KEY, KEY_LEFT);
	input_set_capability(g_input_device, EV_KEY, KEY_RIGHT);
	input_set_capability(g_input_device, EV_KEY, KEY_W);
	input_set_capability(g_input_device, EV_KEY, KEY_Z);
	input_set_capability(g_input_device, EV_KEY, KEY_V);
	input_set_capability(g_input_device, EV_KEY, KEY_O);
	input_set_capability(g_input_device, EV_KEY, KEY_M);
	input_set_capability(g_input_device, EV_KEY, KEY_C);
	input_set_capability(g_input_device, EV_KEY, KEY_E);
	input_set_capability(g_input_device, EV_KEY, KEY_S);
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

/*
#ifdef CONFIG_TP_HAVE_KEY
    set_bit(TOUCH_KEY_MENU, g_input_device->keybit); //Menu
    set_bit(TOUCH_KEY_HOME, g_input_device->keybit); //Home
    set_bit(TOUCH_KEY_BACK, g_input_device->keybit); //Back
    set_bit(TOUCH_KEY_SEARCH, g_input_device->keybit); //Search
#endif
*/

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
	input_set_abs_params(g_input_device, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif/* CONFIG_ENABLE_CHIP_MSG26XXM */
	input_set_abs_params(g_input_device, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(g_input_device, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(g_input_device, ABS_MT_POSITION_X, 0,
	TOUCH_SCREEN_X_MAX, 0, 0);
	input_set_abs_params(g_input_device, ABS_MT_POSITION_Y, 0,
	TOUCH_SCREEN_Y_MAX, 0, 0);
#endif

	return n_retval;
}

s32 platform_device_request_gpio(void)
{
	s32 n_retval = 0;

	LOGTP_FUNC();

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
	n_retval = gpio_request(MS_TS_MSG_IC_GPIO_RST, "C_TP_RST");
	if (n_retval < 0) {
		LOGTP_ERRO("Failed to request GPIO %d, error %d\n",
				MS_TS_MSG_IC_GPIO_RST, n_retval);
	}

	n_retval = gpio_request(MS_TS_MSG_IC_GPIO_INT, "C_TP_INT");
	if (n_retval < 0) {
		LOGTP_ERRO("Failed to request GPIO %d, error %d\n",
				MS_TS_MSG_IC_GPIO_INT, n_retval);
	}
#endif

	return n_retval;
}

s32 platform_device_register_interrupt_handler(void)
{
	s32 n_retval = 0;

	LOGTP_FUNC();

	if (fwic_register_touch_interrupthandle()) {
#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
		/* initialize the finger touch work queue */
INIT_WORK(&_g_finger_touch_work, _platform_finger_touch_dowork);
		gfingertouch_workqueue =
	create_singlethread_workqueue("msg2138");
		if (!gfingertouch_workqueue) {
			n_retval = -ESRCH;
			LOGTP_ERRO("msg2138: create_singlethread_workqueue\n");
			return n_retval;
		}

		_g_irq = gpio_to_irq(MS_TS_MSG_IC_GPIO_INT);

		/* request an irq and register the isr */
		n_retval =
	request_irq(_g_irq  ,/*MS_TS_MSG_IC_GPIO_INT */
				_platform_finger_interrupt_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT
/* | IRQF_NO_SUSPEND *//* IRQF_TRIGGER_FALLING
 */
				,
				"msg2xxx", NULL);
		if (n_retval != 0) {
			LOGTP_DBUG("Unable to claim irq %d; error %d\n",
					MS_TS_MSG_IC_GPIO_INT, n_retval);
		}
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
		mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_INT,
	GPIO_CTP_EINT_PIN_M_EINT);
		mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_INT, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(MS_TS_MSG_IC_GPIO_INT,
					GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(MS_TS_MSG_IC_GPIO_INT, GPIO_PULL_UP);

		mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM,
					CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM,
	EINTF_TRIGGER_RISING,
	_platform_finger_interrupt_handler,
	1);

		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

#ifdef CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
		/* initialize the finger touch work queue */
INIT_WORK(&_g_finger_touch_work, _platform_finger_touch_dowork);
#else
		_g_thread =
	kthread_run(_platform_finger_touch_handler, 0,
				TPD_DEVICE);
		if (IS_ERR(_g_thread)) {
			n_retval = PTR_ERR(_g_thread);
			LOGTP_DBUG("Failed to create kernel thread: %d\n",
					 n_retval);
		}
#endif/* CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM */
#endif
	}
	thread = kthread_run(touch_event_handler, 0, "msg2138-wait-queue");
	if (IS_ERR(thread)) {
		n_retval = PTR_ERR(thread);
		LOGTP_ERRO("failed to create kernel thread: %d\n",
			n_retval);
	}

	return n_retval;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void platform_device_register_earlysuspend(void)
{
	LOGTP_FUNC();

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
	_g_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	_g_early_suspend.suspend = interface_device_suspend;
	_g_early_suspend.resume = interface_device_resume;
	register_early_suspend(&_g_early_suspend);
#endif
}
#endif

/* remove function is triggered when the input device is removed from input
 sub-system */
s32 platform_device_remove(struct i2c_client *p_client)
{
	LOGTP_FUNC();

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
/*    free_irq(MS_TS_MSG_IC_GPIO_INT, g_input_device); */
	free_irq(_g_irq, g_input_device);
	gpio_free(MS_TS_MSG_IC_GPIO_INT);
	gpio_free(MS_TS_MSG_IC_GPIO_RST);
	cancel_work_sync(&_g_finger_touch_work);
	destroy_workqueue(gfingertouch_workqueue);
	input_unregister_device(g_input_device);
#endif
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
	kset_unregister(g_touch_kset);
	kobject_put(g_touch_kobj);
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

	return 0;
}

void platform_set_i2crate(struct i2c_client *p_client, u32 n_i2c_rate)
{

	LOGTP_DBUG("%s() n_i2c_rate = %d\n", __func__, n_i2c_rate);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
/* TODO : Please FAE colleague to confirm with customer device driver
 engineer for how to set i2c data rate on SPRD platform */
	/* sprd_i2c_ctl_chg_clk(p_client->adapter->nr, n_i2c_rate); */
	/* mdelay(100); */
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
/* TODO : Please FAE colleague to confirm with customer device driver
 engineer for how to set i2c data rate on QCOM platform */
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
	p_client->timing = n_i2c_rate / 1000;
#endif
}
