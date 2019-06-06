#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "../../staging/android/timed_output.h"
#include "sc2705-haptic-helper.h"

struct timed_vibrator_data {
	struct timed_output_dev dev;
	struct hrtimer timer;
	struct work_struct work;
	vibrator_ctrl ctrl;
	void *priv;
	int state;
};

static enum hrtimer_restart timed_vibrator_timer_func(struct hrtimer *timer)
{
	struct timed_vibrator_data *data =
		container_of(timer, struct timed_vibrator_data, timer);

	data->state = 0;
	schedule_work(&data->work);

	return HRTIMER_NORESTART;
}

static int timed_vibrator_get_time(struct timed_output_dev *dev)
{
	struct timed_vibrator_data *data =
		container_of(dev, struct timed_vibrator_data, dev);
	ktime_t re;
	int remain = 0;

	if (hrtimer_active(&data->timer)) {
		re = hrtimer_get_remaining(&data->timer);
		remain = ktime_to_ns(re);
	}

	return remain;
}

static void timed_vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct timed_vibrator_data *data =
		container_of(dev, struct timed_vibrator_data, dev);
	ktime_t delay;

	hrtimer_cancel(&data->timer);
	if (value == 0) {
		data->state = 0;
	} else {
		data->state = 1;
		/* avoid along vibrating */
		value = (value > 15000) ? 15000 : value;
		delay = ktime_set(value / 1000, (value % 1000) * 1000000);
		hrtimer_start(&data->timer, delay, HRTIMER_MODE_REL);
	}
	schedule_work(&data->work);
}

static struct timed_vibrator_data vib = {
	.dev = {
		.name = "vibrator",
		.get_time = timed_vibrator_get_time,
		.enable = timed_vibrator_enable,
	},
	.timer = {
		.function = timed_vibrator_timer_func,
	},
	.ctrl = NULL,
	.priv = NULL,
	.state = 0,
};

static void timed_vibrator_work_func(struct work_struct *work)
{
	struct timed_vibrator_data *data =
		container_of(work, struct timed_vibrator_data, work);

	data->ctrl(data->state, data->priv);
}

int timed_vibrator_register(vibrator_ctrl callback, void *private_data)
{
	if (!callback)
		return -EINVAL;

	if (vib.ctrl)
		return -EEXIST;

	INIT_WORK(&vib.work, timed_vibrator_work_func);
	hrtimer_init(&vib.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib.timer.function = timed_vibrator_timer_func;
	vib.ctrl = callback;
	vib.priv = private_data;
	pr_info("timed vibrator registered: cb=%p, priv=%p\n",
		callback, private_data);

	return timed_output_dev_register(&vib.dev);
}
EXPORT_SYMBOL_GPL(timed_vibrator_register);

void timed_vibrator_unregister(vibrator_ctrl callback)
{
	if (!callback)
		return;

	if (callback == vib.ctrl) {
		vib.ctrl = NULL;
		vib.priv = NULL;
		timed_output_dev_register(&vib.dev);
		pr_info("timed vibrator unregistered: cb=%p\n", callback);
	}
}
EXPORT_SYMBOL_GPL(timed_vibrator_unregister);
