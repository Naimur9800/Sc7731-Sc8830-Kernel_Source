/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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

#include <linux/errno.h>
#include <linux/devfreq.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <video/adf_notifier.h>
#include "../governor.h"
#include <linux/sprd_dfs_drv.h>
#include <linux/ctype.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/list.h>
#ifdef CONFIG_DEVFREQ_SPRD_KEY_BOOST
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#endif
#define DFS_MAX_TABLE 10
#define DFS_LCD_CALLBACK 1

struct dfs_para {
	unsigned int overflow[4];
	unsigned int underflow[4];
	unsigned int timer;
};

struct userspace_data {
	unsigned int scaling_cur_ddr_freq;
	unsigned int scaling_force_ddr_freq;
	unsigned int scaling_axi_wltc;
	unsigned int scaling_axi_rltc;
	unsigned int dfs_on_off;
	unsigned int auto_dfs_on_off;
	unsigned int axi_monitor_on_off;
	unsigned int dfs_debug;
	unsigned int ddrinfo_cur_freq;
	unsigned int ddrinfo_cp_freq;
	unsigned int dfs_count;
	unsigned int dfs_status;
	unsigned int dfs_auto_status;
	unsigned int dfs_axi_status;
	unsigned int ddrinfo_freq_table[DFS_MAX_TABLE];
	struct dfs_para hw_dfs_para;
	unsigned int set_freq;
	unsigned int set_count;
	bool devfreq_enable;
	char scene_num[0];
};

static struct devfreq *g_devfreq;
#ifdef CONFIG_DEVFREQ_SPRD_KEY_BOOST
#define GOVER_BOOT_TIME		msecs_to_jiffies(25000)
#define BOOST_LAST_TIME		msecs_to_jiffies(1000)
static int ddr_boost_freq;
static unsigned long boot_done_time, key_interval_time;
static atomic_t devfreq_event_ready = ATOMIC_INIT(0);
static atomic_t devfreq_boost_state = ATOMIC_INIT(0);
static struct task_struct *ksprd_tb;
static DECLARE_WAIT_QUEUE_HEAD(freq_boost_event);
static void devfreq_boost_callback(struct work_struct *work);
static DECLARE_DELAYED_WORK(freq_boost_work, devfreq_boost_callback);
#endif
/*
* set ddr frequnecy
* @freq: MHz
* if ddr frequency is set through this function, DVS is disabled
*/
int dfs_set_freq(int freq)
{
	struct userspace_data *user_data;
	int err;

	if (freq < 0) {
		err = -1;
		pr_debug("*** %s,freq < 0\n", __func__);
		goto done;
	}

	user_data = (struct userspace_data *)(g_devfreq->data);
	if (user_data) {
		if (freq > 0) {
			user_data->set_count++;
			user_data->devfreq_enable = false;
			if (freq > user_data->set_freq)
				user_data->set_freq = freq;
		} else {
			if (user_data->set_count > 0) {
				user_data->set_count--;
				if (user_data->set_count == 0) {
					user_data->set_freq = 0;
					user_data->devfreq_enable = true;
				}
			}
		}
		pr_info("*** %s, set freq:%d KHz, set_count:%u ***\n",
					__func__, freq, user_data->set_count);
	} else
		pr_info("*** %s, user_data == 0 ***\n", __func__);
	err = update_devfreq(g_devfreq);
done:
	return err;
}

/*****************userspace start********************/
static ssize_t scaling_cur_ddr_freq_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	count = sprintf(buf, "%u\n", data->scaling_cur_ddr_freq);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scaling_cur_ddr_freq_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned int set_freq;
	int err;

	data = devfreq->data;
	err = sscanf(buf, "%u\n", &set_freq);
	mutex_lock(&g_devfreq->lock);
	data->scaling_cur_ddr_freq = set_freq;
	err = dfs_set_freq(set_freq);
	if (err)
		pr_err("%s: set freq fail: %d", __func__, err);
	mutex_unlock(&g_devfreq->lock);
	return count;
}

static ssize_t scaling_force_ddr_freq_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	count = sprintf(buf, "%u\n", data->scaling_force_ddr_freq);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scaling_force_ddr_freq_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned int force_freq;
	int err;

	data = devfreq->data;
	err = sscanf(buf, "%u\n", &force_freq);
	mutex_lock(&g_devfreq->lock);
	data->scaling_force_ddr_freq = force_freq;
	err = dfs_set_para(force_freq, 0, SET_FREQ);
	if (err)
		pr_err("%s: force freq fail: %d", __func__, err);
	mutex_unlock(&g_devfreq->lock);
	return count;
}

static ssize_t scaling_overflow_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;
	unsigned int i = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	while (i < 4) {
		if (data->hw_dfs_para.overflow[i] == 0)
			dfs_inquire(&data->hw_dfs_para.overflow[i],
						i, INQ_OVERFLOW);
		count += scnprintf(&buf[count], 32, "%u ",
				data->hw_dfs_para.overflow[i++]);
	}
	count += sprintf(&buf[count], "\n");
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scaling_overflow_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned int i, overflow;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = sscanf(buf, "%u %u\n", &i, &overflow);
	if ((i < 4) && overflow) {
		data->hw_dfs_para.overflow[i] = overflow;
		err = dfs_set_para(overflow, i, SET_OVERFLOW);
		if (err < 0)
			pr_err("%s, set fail:%d\n", __func__, err);
	}
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scaling_underflow_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;
	unsigned int i = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	while (i < 4) {
		if (data->hw_dfs_para.underflow[i] == 0)
			dfs_inquire(&data->hw_dfs_para.underflow[i],
						i, INQ_UNDERFLOW);
		count += scnprintf(&buf[count], 32, "%u ",
				data->hw_dfs_para.underflow[i++]);
	}
	count += sprintf(&buf[count], "\n");
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scaling_underflow_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned int i, underflow;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = sscanf(buf, "%u %u\n", &i, &underflow);
	if ((i < 4) && underflow) {
		data->hw_dfs_para.underflow[i] = underflow;
		err = dfs_set_para(underflow, i, SET_UNDERFLOW);
		if (err < 0)
			pr_err("%s, set fail:%d\n", __func__, err);
	}
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scaling_timer_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	if (data->hw_dfs_para.timer == 0)
		dfs_inquire(&data->hw_dfs_para.timer,
						0, INQ_TIMER);
	count = sprintf(buf, "%u\n", data->hw_dfs_para.timer);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scaling_timer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned timer;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = sscanf(buf, "%u\n", &timer);
	if (timer) {
		data->hw_dfs_para.timer = timer;
		err = dfs_set_para(timer, 0, SET_TIMER);
		if (err < 0)
			pr_err("%s, set fail:%d\n", __func__, err);
	}
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scene_boost_dfs_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned int active_num;
	unsigned int freq;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = sscanf(buf, "%u %u\n", &active_num, &freq);
	if (err < 0) {
		pr_err("%s, boost dfs fail:%d\n", __func__, err);
		mutex_unlock(&devfreq->lock);
		return count;
	}
	if (active_num == 1)
		dfs_scene_request(true, -1, freq);
	else if (active_num == 0)
		dfs_scene_request(false, -1, freq);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scaling_axi_wltc_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	count = sprintf(buf, "%u\n", data->scaling_axi_wltc);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scaling_axi_wltc_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned int i, value;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = sscanf(buf, "%u %u\n", &i, &value);
	data->scaling_axi_wltc = value;
	err = dfs_set_para(value, i, SET_AXI_WLTC);
	if (err < 0)
		pr_err("%s, set wltc fail:%d\n", __func__, err);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scaling_axi_rltc_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	count = sprintf(buf, "%u\n", data->scaling_axi_rltc);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scaling_axi_rltc_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned int i, value;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = sscanf(buf, "%u %u\n", &i, &value);
	data->scaling_axi_rltc = value;
	err = dfs_set_para(value, i, SET_AXI_RLTC);
	if (err < 0)
		pr_err("%s, set rltc fail:%d\n", __func__, err);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scenario_dfs_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	int err, name_len;
	const char *arg;

	mutex_lock(&devfreq->lock);
	arg = buf;
	data = devfreq->data;
	while (*arg && !isspace(*arg))
		arg++;
	name_len = arg-buf;
	if (!name_len) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	data = kzalloc(sizeof(struct userspace_data) +
				name_len * sizeof(char) + 1, GFP_KERNEL);
	if (data == NULL) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	memcpy(data->scene_num, buf, name_len);
	if (strlen(data->scene_num) == name_len) {
		if (strcmp(data->scene_num, "exit") == 0) {
			scene_exit("camhigh");
			err = 0;
		} else
			err = scene_dfs_request(data->scene_num);
		if (err < 0)
			pr_err("%s, scenario dfs fail:%d\n", __func__, err);
	}
	mutex_unlock(&devfreq->lock);
	kfree(data);
	return count;
}

static ssize_t exit_scenario_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	int name_len;
	const char *arg;

	mutex_lock(&devfreq->lock);
	arg = buf;
	data = devfreq->data;
	while (*arg && !isspace(*arg))
		arg++;
	name_len = arg-buf;
	if (!name_len) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	data = kzalloc(sizeof(struct userspace_data) +
				name_len * sizeof(char) + 1, GFP_KERNEL);
	if (data == NULL) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	memcpy(data->scene_num, buf, name_len);
	if (strlen(data->scene_num) == name_len)
		scene_exit(data->scene_num);
	mutex_unlock(&devfreq->lock);
	kfree(data);
	return count;
}

static ssize_t ddrinfo_axi_rltc_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	ssize_t count = 0;
	unsigned int value = 0;
	unsigned int i = 0;
	unsigned int err;

	mutex_lock(&devfreq->lock);
	while (i < 12) {
		err = dfs_inquire(&value, i++, INQ_AXI_RLTC);
		count += scnprintf(&buf[count], 32, "%u ", value);
		if (err)
			break;
	}
	count += sprintf(&buf[count], "\n");
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t ddrinfo_axi_wltc_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	ssize_t count = 0;
	unsigned int value = 0;
	unsigned int i = 0;
	unsigned int err;

	mutex_lock(&devfreq->lock);
	while (i < 12) {
		err = dfs_inquire(&value, i++, INQ_AXI_WLTC);
		count += scnprintf(&buf[count], 32, "%u ", value);
		if (err)
			break;
	}
	count += sprintf(&buf[count], "\n");
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t dfs_count_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = dfs_inquire(&data->dfs_count, 0, INQ_COUNT);
	if (err < 0)
		pr_err("%s, inquire fail:%d\n", __func__, err);
	else
		count = sprintf(buf, "%u\n", data->dfs_count);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t dfs_status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = dfs_inquire(&data->dfs_status, 0, INQ_STATUS);
	if (err < 0)
		pr_err("%s, inquire fail:%d\n", __func__, err);
	else
		count += scnprintf(&buf[count], 32, "%u ", data->dfs_status);

	err = dfs_inquire(&data->dfs_auto_status, 0, INQ_AUTO_STATUS);
	if (err < 0)
		pr_err("%s, inquire fail:%d\n", __func__, err);
	else
		count += scnprintf(&buf[count], 32, "%u ",
						data->dfs_auto_status);

	err = dfs_inquire(&data->dfs_axi_status, 0, INQ_AXI_STATUS);
	if (err < 0)
		pr_err("%s, inquire fail:%d\n", __func__, err);
	else
		count += scnprintf(&buf[count], 32, "%u ",
				data->dfs_axi_status);

	count += sprintf(&buf[count], "\n");
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t dfs_on_off_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	count = sprintf(buf, "%u\n", data->dfs_on_off);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t dfs_on_off_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = sscanf(buf, "%u\n", &data->dfs_on_off);
	if (data->dfs_on_off > 0) {
		err = dfs_set_para(data->dfs_on_off, 0, SET_ENABLE);
		if (err == 0) {
			data->auto_dfs_on_off = 1;
			err = dfs_set_para(data->auto_dfs_on_off,
						0, SET_AUTO_ENABLE);
		}
	} else {
		data->auto_dfs_on_off = 0;
		err = dfs_set_para(data->auto_dfs_on_off,
						0, SET_AUTO_DISABLE);
		if (err == 0) {
			err = dfs_set_para(data->dfs_on_off,
							0, SET_DISABLE);
		}
	}

	if (err < 0)
		pr_err("%s, set fail:%d\n", __func__, err);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t auto_dfs_on_off_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	count = sprintf(buf, "%u\n", data->auto_dfs_on_off);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t auto_dfs_on_off_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = sscanf(buf, "%u\n", &data->auto_dfs_on_off);
	if (data->auto_dfs_on_off > 0)
		err = dfs_set_para(data->auto_dfs_on_off,
						0, SET_AUTO_ENABLE);
	else
		err = dfs_set_para(data->auto_dfs_on_off,
						0, SET_AUTO_DISABLE);
	if (err < 0)
		pr_err("%s, set fail:%d\n", __func__, err);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t axi_monitor_on_off_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	count = sprintf(buf, "%u\n", data->axi_monitor_on_off);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t axi_monitor_on_off_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned int i = 0;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = sscanf(buf, "%u %u\n", &i, &data->axi_monitor_on_off);
	if (data->axi_monitor_on_off > 0)
		err = dfs_set_para(0, i, SET_AXI_ENABLE);
	else
		err = dfs_set_para(0, i, SET_AXI_DISABLE);
	if (err < 0)
		pr_err("%s, set fail:%d\n", __func__, err);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t dfs_debug_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	count = sprintf(buf, "%u\n", data->dfs_debug);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t dfs_debug_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = sscanf(buf, "%u\n", &data->dfs_debug);
	err = dfs_set_para(data->dfs_debug, 0, SET_DEBUG);
	if (err < 0)
		pr_err("%s, set fail:%d\n", __func__, err);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t ddrinfo_cur_freq_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = dfs_inquire(&data->ddrinfo_cur_freq, 0, INQ_DDR_FREQ);
	if (err < 0)
		pr_err("%s, inquire fail:%d\n", __func__, err);
	else
		count = sprintf(buf, "%u\n", data->ddrinfo_cur_freq);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t ddrinfo_cp_freq_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;
	int err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	err = dfs_inquire(&data->ddrinfo_cp_freq, 0, INQ_CP_FREQ);
	if (err < 0)
		pr_err("%s, inquire fail:%d\n", __func__, err);
	else
		count = sprintf(buf, "%u\n", data->ddrinfo_cp_freq);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t ddrinfo_freq_table_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	ssize_t count = 0;
	int i, err;

	mutex_lock(&devfreq->lock);
	data = devfreq->data;
	for (i = 0; i < DFS_MAX_TABLE; i++) {
		err = dfs_inquire(&data->ddrinfo_freq_table[i],
						i, INQ_DDR_TABLE);
		if (err < 0)
			pr_err("%s, inquire fail:%d\n", __func__, err);
		else
			count += sprintf(&buf[count], "%d ",
					data->ddrinfo_freq_table[i]);
	}
	count += sprintf(&buf[count], "\n");
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scene_dfs_req_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct userspace_data *data;
	unsigned int freq;
	int err, active, id, scene_num;

	mutex_lock(&devfreq->lock);
	scene_num = dfs_scene_count;
	data = devfreq->data;
	err = sscanf(buf, "%d %d %u\n", &id, &active, &freq);
	id = id + scene_num - 1;
	dfs_scene_request(active, id, freq);
	mutex_unlock(&devfreq->lock);
	return count;
}

static ssize_t scene_dfs_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct list_head *pos;
	struct dfs_scene *scene;

	list_for_each(pos, &scene_dfs_list) {
		scene = list_entry(pos, struct dfs_scene, scene_list);
		count += sprintf(&buf[count],
				"id:%d, freq: %d, active: %d, count: %d\n",
				scene->scene_id, scene->scene_freq,
				scene->active, scene->active_count);
	}
	return count;
}

static DEVICE_ATTR(scaling_cur_ddr_freq, 0644,
	scaling_cur_ddr_freq_show, scaling_cur_ddr_freq_store);
static DEVICE_ATTR(scaling_force_ddr_freq, 0644,
	scaling_force_ddr_freq_show, scaling_force_ddr_freq_store);
static DEVICE_ATTR(scaling_overflow, 0644,
	scaling_overflow_show, scaling_overflow_store);
static DEVICE_ATTR(scaling_underflow, 0644,
	scaling_underflow_show, scaling_underflow_store);
static DEVICE_ATTR(scaling_timer, 0644,
	scaling_timer_show, scaling_timer_store);
static DEVICE_ATTR(dfs_on_off, 0644,
	dfs_on_off_show, dfs_on_off_store);
static DEVICE_ATTR(auto_dfs_on_off, 0644,
	auto_dfs_on_off_show, auto_dfs_on_off_store);
static DEVICE_ATTR(axi_monitor_on_off, 0644,
	axi_monitor_on_off_show, axi_monitor_on_off_store);
static DEVICE_ATTR(scaling_axi_wltc, 0644,
	scaling_axi_wltc_show, scaling_axi_wltc_store);
static DEVICE_ATTR(scaling_axi_rltc, 0644,
	scaling_axi_rltc_show, scaling_axi_rltc_store);
static DEVICE_ATTR(ddrinfo_axi_wltc, 0444,
	ddrinfo_axi_wltc_show, NULL);
static DEVICE_ATTR(ddrinfo_axi_rltc, 0444,
	ddrinfo_axi_rltc_show, NULL);
static DEVICE_ATTR(dfs_debug, 0644,
	dfs_debug_show, dfs_debug_store);
static DEVICE_ATTR(dfs_status, 0444,
	dfs_status_show, NULL);
static DEVICE_ATTR(dfs_count, 0444,
	dfs_count_show, NULL);
static DEVICE_ATTR(ddrinfo_cur_freq, 0444,
	ddrinfo_cur_freq_show, NULL);
static DEVICE_ATTR(ddrinfo_cp_freq, 0444,
	ddrinfo_cp_freq_show, NULL);
static DEVICE_ATTR(ddrinfo_freq_table, 0444,
	ddrinfo_freq_table_show, NULL);
static DEVICE_ATTR(scenario_dfs, 0644,
	NULL, scenario_dfs_store);
static DEVICE_ATTR(exit_scene, 0644,
	NULL, exit_scenario_store);
static DEVICE_ATTR(scene_boost_dfs, 0644,
	NULL, scene_boost_dfs_store);
static DEVICE_ATTR(scene_dfs_list, 0644,
	scene_dfs_status_show, scene_dfs_req_store);

static struct attribute *dev_entries[] = {
	&dev_attr_scaling_cur_ddr_freq.attr,
	&dev_attr_scaling_force_ddr_freq.attr,
	&dev_attr_scaling_overflow.attr,
	&dev_attr_scaling_underflow.attr,
	&dev_attr_scaling_timer.attr,
	&dev_attr_dfs_on_off.attr,
	&dev_attr_auto_dfs_on_off.attr,
	&dev_attr_axi_monitor_on_off.attr,
	&dev_attr_scaling_axi_wltc.attr,
	&dev_attr_scaling_axi_rltc.attr,
	&dev_attr_ddrinfo_axi_wltc.attr,
	&dev_attr_ddrinfo_axi_rltc.attr,
	&dev_attr_dfs_debug.attr,
	&dev_attr_dfs_status.attr,
	&dev_attr_dfs_count.attr,
	&dev_attr_ddrinfo_cur_freq.attr,
	&dev_attr_ddrinfo_cp_freq.attr,
	&dev_attr_ddrinfo_freq_table.attr,
	&dev_attr_scenario_dfs.attr,
	&dev_attr_exit_scene.attr,
	&dev_attr_scene_boost_dfs.attr,
	&dev_attr_scene_dfs_list.attr,
	NULL,
};

/*****************userspace end********************/

static int dfs_notifier_callback(struct notifier_block *self,
					unsigned long event, void *data)
{
	struct adf_notifier_event *evdata = data;
	int adf_blank;

	/* If we aren't interested in this event, skip it immediately */
	if (event != ADF_EVENT_BLANK)
		return 0;

	mutex_lock(&g_devfreq->lock);
	adf_blank = *(int *)evdata->data;

#ifdef DFS_LCD_CALLBACK
	switch (adf_blank) {
	case DRM_MODE_DPMS_ON:
		scene_dfs_request("lcdon");
		scene_exit("lcdoff");
		break;
	case DRM_MODE_DPMS_OFF:
		scene_dfs_request("lcdoff");
		scene_exit("lcdon");
		break;
	default:
		pr_info("%s: error blank event\n", __func__);
	}
#endif
	mutex_unlock(&g_devfreq->lock);

	return NOTIFY_OK;
}

static struct notifier_block dfs_fb_notifier = {
	.notifier_call = dfs_notifier_callback,
};

static struct attribute_group dev_attr_group = {
	.name	= "sprd_governor",
	.attrs	= dev_entries,
};

static int devfreq_sprd_gov_start(struct devfreq *devfreq)
{
	int i = 0;
	int err = 0;
	struct userspace_data *data = kzalloc(sizeof(struct userspace_data),
			GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto out;
	}
	data->scaling_cur_ddr_freq = 0;
	data->dfs_on_off = 0;
	data->auto_dfs_on_off = 0;
	data->axi_monitor_on_off = 0;
	data->scaling_axi_wltc = 0;
	data->scaling_axi_rltc = 0;
	data->ddrinfo_cur_freq = 0;
	data->ddrinfo_cp_freq = 0;
	data->dfs_status = 0;
	data->dfs_auto_status = 0;
	data->dfs_axi_status = 0;
	data->dfs_debug = 0;

	for (i = 0; i < DFS_MAX_TABLE; i++)
		data->ddrinfo_freq_table[i] = 0;
	for (i = 0; i < 4; i++) {
		data->hw_dfs_para.overflow[i] = 0;
		data->hw_dfs_para.underflow[i] = 0;
		data->hw_dfs_para.timer = 0;
	}
	devfreq->data = data;
	g_devfreq = devfreq;
	err = sysfs_create_group(&devfreq->dev.kobj, &dev_attr_group);
	if (err)
		pr_err("%s:sysfs create fail: %d", __func__, err);
	err = adf_register_client(&dfs_fb_notifier);
	if (err)
		pr_err("%s:adf register fail: %d", __func__, err);
	pr_debug("%s:governor start, governor name: %s",
				__func__, devfreq->governor->name);
out:
	return err;
}


static int devfreq_sprd_gov_stop(struct devfreq *devfreq)
{
	int err = 0;

	if (devfreq->data)
		kfree(devfreq->data);

	err = adf_unregister_client(&dfs_fb_notifier);
	if (err)
		pr_err("%s:adf unregister fail: %d", __func__, err);
	return err;
}


static int devfreq_sprd_gov_func(struct devfreq *df,
					unsigned long *freq)
{
	struct userspace_data *data;

	data = df->data;
	if (data->set_freq == 0)
		*freq = df->min_freq;
	else
		*freq = data->set_freq;

	return 0;
}

static int devfreq_sprd_gov_handler(struct devfreq *devfreq,
				unsigned int event, void *data)
{
	switch (event) {
	case DEVFREQ_GOV_START:
		devfreq_sprd_gov_start(devfreq);
		break;
	case DEVFREQ_GOV_STOP:
		devfreq_sprd_gov_stop(devfreq);
		break;
	case DEVFREQ_GOV_INTERVAL:
		break;
	case DEVFREQ_GOV_SUSPEND:
		break;
	case DEVFREQ_GOV_RESUME:
		break;
	default:
		break;
	}

	return 0;
}

struct devfreq_governor devfreq_sprd_gov = {
	.name = "sprd_governor",
	.get_target_freq = devfreq_sprd_gov_func,
	.event_handler = devfreq_sprd_gov_handler,
};

#ifdef CONFIG_DEVFREQ_SPRD_KEY_BOOST
static void devfreq_dfs_change(unsigned int active_num, unsigned int freq)
{
	mutex_lock(&g_devfreq->lock);
	switch (active_num) {
	case 0:
		dfs_scene_request(false, -1, freq);
		break;
	case 1:
		dfs_scene_request(true, -1, freq);
		break;
	default:
		break;
	}
	mutex_unlock(&g_devfreq->lock);
}

static void devfreq_boost_callback(struct work_struct *work)
{
	atomic_set(&devfreq_boost_state, 0);
	devfreq_dfs_change(0, ddr_boost_freq);
}

static void dbs_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	if (time_before(jiffies, boot_done_time)
			|| code == 0 || value == 0
			|| atomic_read(&devfreq_event_ready)
			|| atomic_read(&devfreq_boost_state))
		return;

	if (time_after(jiffies, key_interval_time))
		key_interval_time = jiffies + HZ;
	else
		return;

	atomic_set(&devfreq_event_ready, 1);
	wake_up(&freq_boost_event);
}

static int dbs_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "ddrfreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;

err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void dbs_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static int sprd_tb_thread(void *data)
{
	while (1) {
		wait_event_interruptible(freq_boost_event,
				atomic_read(&devfreq_event_ready)
				|| kthread_should_stop());

		if (kthread_should_stop())
			break;

		atomic_set(&devfreq_boost_state, 1);
		atomic_set(&devfreq_event_ready, 0);
		devfreq_dfs_change(1, ddr_boost_freq);
		schedule_delayed_work(&freq_boost_work, BOOST_LAST_TIME);
	}
	return 0;
}

static const struct input_device_id dbs_ids[] = {
	{ .driver_info = 1 },
	{ },
};

struct input_handler dbs_input_handler = {
	.event		= dbs_input_event,
	.connect	= dbs_input_connect,
	.disconnect	= dbs_input_disconnect,
	.name		= "ddrfreq_ond",
	.id_table	= dbs_ids,
};

static int devfreq_boost_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	int error;

	error = of_property_read_u32(node, "sprd,ddr-boost-freq",
			&ddr_boost_freq);
	if (error)
		return error;

	boot_done_time = jiffies + GOVER_BOOT_TIME;
	key_interval_time = jiffies;

	error = input_register_handler(&dbs_input_handler);
	if (error)
		return error;

	ksprd_tb = kthread_run(sprd_tb_thread, NULL, "sprd_tb_thread");
	if (PTR_ERR_OR_ZERO(ksprd_tb)) {
		input_unregister_handler(&dbs_input_handler);
		return PTR_ERR(ksprd_tb);
	}

	return 0;
}

static int devfreq_boost_remove(struct platform_device *pdev)
{
	if (!IS_ERR_OR_NULL(ksprd_tb))
		kthread_stop(ksprd_tb);
	input_unregister_handler(&dbs_input_handler);
	return 0;
}

static const struct of_device_id devfreq_boost_of_match[] = {
	{.compatible = "sprd,ddr-boost-freq",},
	{}
};
MODULE_DEVICE_TABLE(of, devfreq_boost_of_match);

static struct platform_driver devfreq_boost_driver = {
	.driver = {
		.name = "sprd-ddr-boost",
		.of_match_table = devfreq_boost_of_match,
	},
	.probe		= devfreq_boost_probe,
	.remove		= devfreq_boost_remove,
};
#endif

static int __init devfreq_sprd_gov_init(void)
{
#ifdef CONFIG_DEVFREQ_SPRD_KEY_BOOST
	int ret;

	ret = platform_driver_register(&devfreq_boost_driver);
	if (ret)
		pr_err("%s: failed to start ddr boost %d\n", __func__, ret);

	ret = devfreq_add_governor(&devfreq_sprd_gov);
	if (ret) {
		pr_err("%s: failed to add governor %d\n", __func__, ret);
		platform_driver_unregister(&devfreq_boost_driver);
	}
	return ret;
#else
	return devfreq_add_governor(&devfreq_sprd_gov);
#endif
}
subsys_initcall(devfreq_sprd_gov_init);

static void __exit devfreq_sprd_gov_exit(void)
{
	int ret;

#ifdef CONFIG_DEVFREQ_SPRD_KEY_BOOST
	platform_driver_unregister(&devfreq_boost_driver);
#endif

	ret = devfreq_remove_governor(&devfreq_sprd_gov);
	if (ret)
		pr_err("%s: failed remove governor %d\n", __func__, ret);

	return;
}

module_exit(devfreq_sprd_gov_exit);
MODULE_LICENSE("GPL");
