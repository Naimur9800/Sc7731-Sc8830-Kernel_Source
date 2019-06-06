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

#include <linux/fs.h>
#include <linux/devfreq.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/of_device.h>
#include <linux/suspend.h>
#include <linux/sipc.h>
#include <linux/list.h>
#include <linux/sprd_dfs_drv.h>

enum dfs_master_cmd {
	DFS_CMD_NORMAL		= 0x0000,
	DFS_CMD_ENABLE		= 0x0300,
	DFS_CMD_DISABLE		= 0x0305,
	DFS_CMD_AUTO_ENABLE	= 0x0310,
	DFS_CMD_AUTO_DISABLE	= 0x0315,
	DFS_CMD_AXI_ENABLE      = 0x0320,
	DFS_CMD_AXI_DISABLE     = 0x0330,
	DFS_CMD_INQ_DDR_FREQ	= 0x0500,
	DFS_CMD_INQ_CP_FREQ	= 0x0503,
	DFS_CMD_INQ_DDR_TABLE	= 0x0505,
	DFS_CMD_INQ_COUNT	= 0x0507,
	DFS_CMD_INQ_OVERFLOW	= 0x0510,
	DFS_CMD_INQ_UNDERFLOW	= 0x0520,
	DFS_CMD_INQ_TIMER	= 0x0530,
	DFS_CMD_INQ_AXI         = 0x0540,
	DFS_CMD_INQ_AXI_WLTC    = 0x0541,
	DFS_CMD_INQ_AXI_RLTC    = 0x0542,
	DFS_CMD_SET_DDR_FREQ	= 0x0600,
	DFS_CMD_SET_CAL_FREQ	= 0x0603,
	DFS_CMD_PARA_START	= 0x0700,
	DFS_CMD_PARA_OVERFLOW	= 0x0710,
	DFS_CMD_OVERFLOW_MASK	= 0xFFF0,
	DFS_CMD_PARA_UNDERFLOW	= 0x0720,
	DFS_CMD_UNDERFLOW_MASK	= 0xFFF0,
	DFS_CMD_PARA_TIMER	= 0x0730,
	DFS_CMD_PARA_END	= 0x07FF,
	DFS_CMD_SET_AXI_WLTC    = 0x0810,
	DFS_CMD_SET_AXI_RLTC    = 0x0820,
	DFS_CMD_AXI_MASK        = 0xFFF0,
	DFS_CMD_DEBUG		= 0x0FFF
};

enum dfs_slave_cmd {
	DFS_RET_ADJ_OK		= 0x0000,
	DFS_RET_ADJ_VER_FAIL	= 0x0001,
	DFS_RET_ADJ_BUSY	= 0x0002,
	DFS_RET_ADJ_NOCHANGE	= 0x0003,
	DFS_RET_ADJ_FAIL	= 0x0004,
	DFS_RET_DISABLE		= 0x0005,
	DFS_RET_ON_OFF_SUCCEED	= 0x0300,
	DFS_RET_ON_OFF_FAIL	= 0x0303,
	DFS_RET_INQ_SUCCEED	= 0x0500,
	DFS_RET_INQ_FAIL	= 0x0503,
	DFS_RET_SET_SUCCEED	= 0x0600,
	DFS_RET_SET_FAIL	= 0x0603,
	DFS_RET_PARA_OK		= 0x070F,
	DFS_RET_INVALID_CMD	= 0x0F0F
};

enum dfs_status_def {
	DFS_STATUS_DISABLE = 0x00,
	DFS_STATUS_ENABLE = 0x01,
	DFS_STATUS_AUTO_DISABLE = 0x00,
	DFS_STATUS_AUTO_ENABLE = 0x01,
	DFS_STATUS_AXI_DISABLE = 0x00,
	DFS_STATUS_AXI_ENABLE = 0x01,
	DFS_INIT_OPEN_FAIL = 0x01,
	DFS_INIT_ENABLE_FAIL = 0x02,
	DFS_INIT_AUTO_ENABLE_FAIL = 0x03,
	DFS_INIT_AXI_ENABLE_FAIL = 0x04,
	DFS_INIT_FINISH = 0x0f
};
/*The number of dfs supported frequency point*/
#define FREQ_NUM  10
/*The number of supported dfi parameter*/
#define DFI_PARA_NUM  4
/*initial freq is not correct, but it's not uesed, just reserved*/
#define DFS_INITIAL_FREQ 640
/*polling is not correct, but it's not uesed, just reserved*/
#define DFS_POLLING_MS (100)
#define BOOT_TIME	(22*HZ)
/*max,min freq initial value, will be update when dfs enable*/
#define DFS_MIN_FREQ	10
#define DFS_MAX_FREQ	~0
#define MAX_DDR_FREQ	640

struct scene_freq {
	char *scene_name;
	unsigned int scene_freq;
};

struct dmcfreq_data {
	struct device *dev;
	struct devfreq *devfreq;
	unsigned long last_jiffies;
	unsigned long int boot_done;
	spinlock_t lock;
	unsigned int scene_count;
	struct scene_freq *scenefreq;
};

struct init_dfs_para {
	unsigned int overflow_inited;
	unsigned int overflow[DFI_PARA_NUM];
	unsigned int underflow_inited;
	unsigned int underflow[DFI_PARA_NUM];
	unsigned int timer_inited;
	unsigned int timer;
};

static unsigned int freq_table[FREQ_NUM] = {0};
static int freq_count[FREQ_NUM] = {0};
struct list_head scene_dfs_list;
static DEFINE_SPINLOCK(dfs_request_lock);
static DEFINE_MUTEX(scenc_request_lock);
struct dmcfreq_data *scenario_data;
static struct task_struct *dfs_smsg_ch_open;
static unsigned int dfs_status = DFS_STATUS_DISABLE;
static unsigned int dfs_auto_status = DFS_STATUS_AUTO_DISABLE;
static unsigned int dfs_axi_status = DFS_STATUS_AXI_DISABLE;
static unsigned int dfs_init_status = DFS_INIT_OPEN_FAIL;
unsigned int axi_channel_status = 0x01c2;
int dfs_scene_count;
static unsigned int max_freq;
static unsigned int min_freq;
static unsigned int last_freq;
static struct init_dfs_para dfs_para_init;
static unsigned int force_high_freq;

static unsigned int dfs_max_freq(struct dmcfreq_data *data)
{
	return DFS_MAX_FREQ;
}

static unsigned int dfs_min_freq(struct dmcfreq_data *data)
{
	return DFS_MIN_FREQ;
}

int dfs_msg_recv(struct smsg *msg, int timeout)
{
	int err = 0;

	err = smsg_recv(SIPC_ID_PM_SYS, msg, timeout);
	if (err < 0) {
		pr_err("%s, dfs receive failed:%d\n", __func__, err);
		return err;
	}
	if (SMSG_CH_PM_CTRL == msg->channel && SMSG_TYPE_DFS_RSP == msg->type) {
		pr_debug("%s receive: channel=%d, type=%d, flag=0x%04x, value=0x%08x\n",
		__func__, msg->channel, msg->type, msg->flag, msg->value);
		return 0;
	} else
		return -EINVAL;
}

int dfs_msg_send(struct smsg *msg, unsigned int realcmd, int timeout,
		 unsigned int realfreq)
{
	int err = 0;

	smsg_set(msg, SMSG_CH_PM_CTRL, SMSG_TYPE_DFS, realcmd, realfreq);
	err = smsg_send(SIPC_ID_PM_SYS, msg, timeout);
	pr_debug("%s send: channel=%d, type=%d, flag=0x%04x, value=0x%08x\n",
	__func__, msg->channel, msg->type, msg->flag, msg->value);
	if (err < 0) {
		pr_err("%s, dfs send failed, freq:%d, cmd:%d\n",
		       __func__, realfreq, realcmd);
		return err;
	}
	return 0;
}

static int dfs_request(unsigned int freq, unsigned int cmd)
{
	int err = 0;
	struct smsg msg;

	if (dfs_status == DFS_STATUS_DISABLE) {
		pr_err("%s, dfs is disabled\n", __func__);
		return -EINVAL;
	}

	err = dfs_msg_send(&msg, cmd, msecs_to_jiffies(100), freq);
	if (err < 0)
		pr_err("%s, dfs send freq(%d) failed!\n", __func__,
		       freq);

	err = dfs_msg_recv(&msg, msecs_to_jiffies(500));
	if (err < 0)
		pr_err("%s, dfs receive failed!\n", __func__);

	switch (msg.flag) {
	case DFS_RET_ADJ_OK:
		pr_info("%s, dfs succeeded!current freq:%d\n", __func__,
		       msg.value);
		break;
	case DFS_RET_ADJ_VER_FAIL:
		pr_info("%s, dfs verify fail!current freq:%d\n", __func__,
		       msg.value);
		break;
	case DFS_RET_ADJ_BUSY:
		pr_info("%s, dfs busy!current freq:%d\n", __func__, msg.value);
		break;
	case DFS_RET_ADJ_NOCHANGE:
		pr_info("%s, dfs target no change!current freq:%d\n", __func__,
		       msg.value);
		break;
	case DFS_RET_ADJ_FAIL:
		pr_info("%s, dfs fail!current freq:%d\n", __func__, msg.value);
		break;
	case DFS_RET_DISABLE:
		pr_info("%s, dfs is disabled!current freq:%d\n", __func__,
		       msg.value);
		break;
	default:
		pr_info("%s, dfs invalid cmd:%x!current freq:%d\n", __func__,
		       msg.flag, msg.value);
		break;
	}

	return err;
}


static int inq_dfs_msg(unsigned int *data, unsigned int value, unsigned int cmd)
{
	int err = 0;
	struct smsg msg;

	err = dfs_msg_send(&msg, cmd, msecs_to_jiffies(100), value);
	if (err < 0) {
		pr_err("%s, dfs inquire send failed: %x\n", __func__, cmd);
		return err;
	}

	err = dfs_msg_recv(&msg, msecs_to_jiffies(500));
	if (err < 0) {
		pr_err("%s, dfs inquire receive failed: %x\n", __func__, cmd);
		return err;
	}

	if (msg.flag == DFS_RET_INVALID_CMD) {
		pr_err("%s, dfs inquire fail, invalid cmd:%d\n", __func__,
		       msg.flag);
		return -EINVAL;
	}

	*data = msg.value;

	return err;
}


int dfs_para_update(unsigned int value, unsigned int cmd)
{
	int err = 0;
	struct smsg msg;

	err = dfs_msg_send(&msg, cmd, msecs_to_jiffies(100), value);
	if (err < 0)
		goto out;

	err = dfs_msg_recv(&msg, msecs_to_jiffies(500));
	if (err < 0)
		goto out;

	if (msg.flag == DFS_RET_INVALID_CMD) {
		pr_err("%s, dfs update fail, invalid cmd:%d\n", __func__,
		       msg.flag);
		err = -EINVAL;
		goto out;
	}

	if (msg.flag == DFS_RET_PARA_OK) {
		pr_info("%s, dfs configuration, cmd:%x, ret:%d\n", __func__,
			cmd, msg.value);
		return msg.value;
	} else
		pr_err("%s, dfs update fail:cmd-%x, flag-%d, value-%d\n",
		       __func__, cmd, msg.flag, msg.value);

out:
	return err;
}

int dfs_enable(unsigned int cmd)
{
	int err = 0;
	struct smsg msg;

	err = dfs_msg_send(&msg, cmd, msecs_to_jiffies(100), max_freq);
	if (err < 0)
		return err;

	err = dfs_msg_recv(&msg, msecs_to_jiffies(2000));
	if (err < 0)
		return err;

	if (msg.flag != DFS_RET_ON_OFF_SUCCEED) {
		pr_err("%s, dfs enable verify failed:cmd-%x, flag-%d\n",
		       __func__, cmd, msg.flag);
		return -EINVAL;
	}

	if (cmd == DFS_CMD_ENABLE)
		dfs_status = DFS_STATUS_ENABLE;
	else if (cmd == DFS_CMD_AUTO_ENABLE)
		dfs_auto_status = DFS_STATUS_AUTO_ENABLE;
	else
		dfs_axi_status = DFS_STATUS_AXI_ENABLE;

	return 0;
}

int dfs_disable(unsigned int cmd)
{
	int err = 0;
	struct smsg msg;

	err = dfs_msg_send(&msg, cmd, msecs_to_jiffies(100),
					max_freq+0x10000000);
	if (err < 0)
		return err;

	err = dfs_msg_recv(&msg, msecs_to_jiffies(2000));
	if (err < 0)
		return err;

	if (msg.flag != DFS_RET_ON_OFF_SUCCEED) {
		pr_err("%s, dfs disable verify failed:%x\n", __func__,
		       msg.flag);
		return -EINVAL;
	}

	if (cmd == DFS_CMD_DISABLE)
		dfs_status = DFS_STATUS_DISABLE;
	else if (cmd == DFS_CMD_AUTO_DISABLE)
		dfs_auto_status = DFS_STATUS_AUTO_DISABLE;
	else
		dfs_axi_status = DFS_STATUS_AXI_DISABLE;

	return 0;
}

int dfs_inquire(unsigned int *data, unsigned int index, unsigned int event)
{
	int err = 0;
	unsigned int inq_event = event;

	if (dfs_init_status != DFS_INIT_FINISH) {
		pr_err("%s, inquire fail:%d\n", __func__, dfs_init_status);
		err = -1;
		return err;
	}

	switch (inq_event) {
	case INQ_DDR_FREQ:
		err = inq_dfs_msg(data, index, DFS_CMD_INQ_DDR_FREQ);
		break;
	case INQ_CP_FREQ:
		err = inq_dfs_msg(data, index, DFS_CMD_INQ_CP_FREQ);
		break;
	case INQ_DDR_TABLE:
		err = inq_dfs_msg(data, index, DFS_CMD_INQ_DDR_TABLE);
		break;
	case INQ_COUNT:
		err = inq_dfs_msg(data, index, DFS_CMD_INQ_COUNT);
		break;
	case INQ_OVERFLOW:
		*data = dfs_para_init.overflow[index];
		break;
	case INQ_UNDERFLOW:
		*data = dfs_para_init.underflow[index];
		break;
	case INQ_TIMER:
		*data = dfs_para_init.timer;
		break;
	case INQ_STATUS:
		*data = dfs_status;
		break;
	case INQ_AUTO_STATUS:
		*data = dfs_auto_status;
		break;
	case INQ_AXI_STATUS:
		*data = dfs_axi_status;
		break;
	case INQ_AXI_WLTC:
		err = inq_dfs_msg(data, index, DFS_CMD_INQ_AXI_WLTC);
		break;
	case INQ_AXI_RLTC:
		err = inq_dfs_msg(data, index, DFS_CMD_INQ_AXI_RLTC);
		break;
	default:
		pr_info("%s: invalid event: %d", __func__, event);
	}

	return err;
}
int dfs_set_para(unsigned int para, unsigned int index, unsigned int event)
{
	int err = 0;
	unsigned int set_event = event;

	if (dfs_init_status != DFS_INIT_FINISH) {
		pr_err("%s, set fail:%d\n", __func__, dfs_init_status);
		err = -1;
		return err;
	}

	switch (set_event) {
	case SET_FREQ:
		err = dfs_request(para, DFS_CMD_SET_DDR_FREQ);
		break;
	case SET_OVERFLOW:
		err = dfs_para_update(para, DFS_CMD_PARA_OVERFLOW + index);
		break;
	case SET_UNDERFLOW:
		err = dfs_para_update(para, DFS_CMD_PARA_UNDERFLOW + index);
		break;
	case SET_TIMER:
		err = dfs_para_update(para, DFS_CMD_PARA_TIMER + index);
		break;
	case SET_ENABLE:
		err = dfs_enable(DFS_CMD_ENABLE);
		break;
	case SET_DISABLE:
		err = dfs_disable(DFS_CMD_DISABLE);
		break;
	case SET_AUTO_ENABLE:
		err = dfs_enable(DFS_CMD_AUTO_ENABLE);
		break;
	case SET_AUTO_DISABLE:
		err = dfs_disable(DFS_CMD_AUTO_DISABLE);
		break;
	case SET_AXI_ENABLE:
		err = dfs_enable(DFS_CMD_AXI_ENABLE + index);
		break;
	case SET_AXI_DISABLE:
		err = dfs_disable(DFS_CMD_AXI_DISABLE + index);
		break;
	case SET_AXI_WLTC:
		err = dfs_para_update(para, DFS_CMD_SET_AXI_WLTC + index);
		break;
	case SET_AXI_RLTC:
		err = dfs_para_update(para, DFS_CMD_SET_AXI_RLTC + index);
		break;
	case SET_DEBUG:
		err = dfs_para_update(para, DFS_CMD_DEBUG);
		break;

	default:
		pr_info("%s: invalid event: %d", __func__, event);
	}

	return 0;
}

int inq_axi_busmonitor(unsigned int channel)
{
	int value;

	value = (axi_channel_status >> channel) & 0x01;
	return value;
}

int set_axi_busmonitor(unsigned int channel, unsigned int value)
{
	int err;

	if (value)
		err = dfs_set_para(0, channel, SET_AXI_ENABLE);
	else
		err = dfs_set_para(0, channel, SET_AXI_DISABLE);
	return err;
}

static unsigned int find_scene_num(char *scenario)
{
	int err, i;

	if (scenario_data == NULL) {
		return -EINVAL;
	}
	for (i = 0; i < scenario_data->scene_count; i++) {
		err = strcmp(scenario, scenario_data->scenefreq[i].scene_name);
		if (err == 0)
			return i;
	}
	return -EINVAL;
}

static void freq_count_update(unsigned int freq, bool active)
{
	int i;

	for (i = 0; i < FREQ_NUM; i++) {
		if (freq == freq_table[i]) {
			if (active) {
				freq_count[i]++;
			} else {
				freq_count[i]--;
			}
			break;
		}
	}
}

static unsigned int entry_freq(void)
{
	int i;
	unsigned int freq = max_freq;

	for (i = 0; i < FREQ_NUM; i++) {
		if (freq_table[i] > 0) {
			if (freq_count[i] > 0)
				freq = freq_table[i];
		} else
			break;
	}
	return freq;
}

static void scene_dfs_send(unsigned int freq, bool active)
{
	int err = 0;

	spin_lock(&dfs_request_lock);
	freq_count_update(freq, active);
	freq = entry_freq();
	spin_unlock(&dfs_request_lock);

	if (freq != last_freq) {
		spin_lock(&dfs_request_lock);
		last_freq = freq;
		spin_unlock(&dfs_request_lock);
		err = dfs_request(freq, DFS_CMD_NORMAL);
	}
}
static void scene_dfs_entry(int id, unsigned int freq)
{
	struct dfs_scene *scene;
	struct dfs_scene *p;
	struct list_head *pos;

	list_for_each(pos, &scene_dfs_list) {
		p = list_entry(pos, struct dfs_scene, scene_list);
		if (p->scene_id == id) {
			pr_info("The request scene %d node is existed\n",
				p->scene_id);
			scene_dfs_send(p->scene_freq, p->active);
			p->active_count++;
			return;
		}
	}
	scene = kzalloc(sizeof(struct dfs_scene), GFP_KERNEL);
	if (scene == NULL)
		return;
	scene->scene_id = id;
	scene->scene_freq = freq;
	scene_dfs_send(freq, true);
	scene->active_count++;
	scene->active = true;
	list_add_tail(&scene->scene_list, &scene_dfs_list);
}

static void scene_dfs_exit(int id, unsigned int freq)
{
	struct list_head *pos;
	struct list_head *tmp;
	struct dfs_scene *p;

	list_for_each_safe(pos, tmp, &scene_dfs_list) {
		p = list_entry(pos, struct dfs_scene, scene_list);
		if (p->scene_id == id) {
			scene_dfs_send(freq, false);
			p->active_count--;
			if (p->active_count == 0) {
				p->active = false;
				list_del(pos);
				kfree(p);
				p = NULL;
			}
			break;
		}
	}
}

void dfs_scene_request(bool active, int id, unsigned int freq)
{
	mutex_lock(&scenc_request_lock);
	if (active)
		scene_dfs_entry(id, freq);
	else
		scene_dfs_exit(id, freq);
	mutex_unlock(&scenc_request_lock);
}

int scene_dfs_request(char *scenario)
{
	unsigned int freq, num;

	num = find_scene_num(scenario);
	if (num == -EINVAL) {
		pr_err("%s, The scene: %s is invalid\n", __func__, scenario);
		return 0;
	}
	freq = scenario_data->scenefreq[num].scene_freq;
	dfs_scene_request(true, num, freq);
	return 0;
}
EXPORT_SYMBOL(scene_dfs_request);

void scene_exit(char *scenario)
{
	unsigned int freq, num;

	num = find_scene_num(scenario);
	if (num == -EINVAL) {
		pr_err("%s, The scene: %s is invalid\n", __func__, scenario);
		return;
	}
	freq = scenario_data->scenefreq[num].scene_freq;
	dfs_scene_request(false, num, freq);
}
EXPORT_SYMBOL(scene_exit);

static void dfs_freq_table_sort(unsigned int freq[], int n)
{
	int i, j;
	unsigned int temp;
	bool isSorted;

	for (i = 0; i < n-1; i++) {
		isSorted = true;
		for (j = 0; j < n-i-1; j++) {
			if (freq[j] > freq[j+1]) {
				isSorted = false;
				temp = freq[j];
				freq[j] = freq[j+1];
				freq[j+1] = temp;
			}
		}
		if (isSorted)
			break;
	}
}

static int dfs_smsg_thread(void *dmc_data)
{
	struct dmcfreq_data *data = dmc_data;
	int i, j, err = 0;

	err = smsg_ch_open(SIPC_ID_PM_SYS, SMSG_CH_PM_CTRL, -1);
	if (err < 0) {
		pr_err("%s, open sipc channel failed:%d\n", __func__, err);
		dfs_init_status = DFS_INIT_OPEN_FAIL;
		goto err_out;
	}

	while (time_before(jiffies, data->boot_done)) {
		msleep(20);
	}

	err = dfs_enable(DFS_CMD_ENABLE);
	if (err < 0) {
		pr_err("%s, dfs enable failed:%d\n", __func__, err);
		dfs_init_status = DFS_INIT_ENABLE_FAIL;
		goto err_out;
	}
	if (dfs_para_init.overflow_inited) {
		for (i = 0; i < DFI_PARA_NUM; i++) {
			err = dfs_para_update(dfs_para_init.overflow[i],
					DFS_CMD_PARA_OVERFLOW + i);
			if (err < 0) {
				pr_err("%s, dfs overflow init failed:%d\n"
						, __func__, err);
				dfs_init_status = DFS_INIT_ENABLE_FAIL;
				goto err_out;
			}
		}
	} else {
		for (i = 0; i < DFI_PARA_NUM; i++) {
			err = inq_dfs_msg(&dfs_para_init.overflow[i], i,
					DFS_CMD_INQ_OVERFLOW);
			if (err < 0) {
				pr_err("%s, dfs overflow init failed:%d\n",
						__func__, err);
				dfs_init_status = DFS_INIT_ENABLE_FAIL;
				goto err_out;
			}
		}
	}
	if (dfs_para_init.underflow_inited) {
		for (i = 0; i < DFI_PARA_NUM; i++) {
			err = dfs_para_update(dfs_para_init.underflow[i],
					DFS_CMD_PARA_UNDERFLOW + i);
			if (err < 0) {
				pr_err("%s, dfs overflow init failed:%d\n",
						__func__, err);
				dfs_init_status = DFS_INIT_ENABLE_FAIL;
				goto err_out;
			}
		}
	} else {
		for (i = 0; i < DFI_PARA_NUM; i++) {
			err = inq_dfs_msg(&dfs_para_init.underflow[i], i,
					DFS_CMD_INQ_UNDERFLOW);
			if (err < 0) {
				pr_err("%s, dfs overflow init failed:%d\n",
						__func__, err);
				dfs_init_status = DFS_INIT_ENABLE_FAIL;
				goto err_out;
			}
		}
	}
	if (dfs_para_init.timer_inited) {
		err = dfs_para_update(dfs_para_init.timer, DFS_CMD_PARA_TIMER);
	} else {
		err = inq_dfs_msg(&dfs_para_init.timer, 0,
				DFS_CMD_INQ_TIMER);
	}
	if (err < 0) {
		pr_err("%s, dfs timer init  failed:%d\n", __func__, err);
		dfs_init_status = DFS_INIT_ENABLE_FAIL;
		goto err_out;
	}

	if (!force_high_freq) {
		err = dfs_enable(DFS_CMD_AUTO_ENABLE);
		if (err < 0) {
			pr_err("%s, hw dfs enable failed:%d\n", __func__, err);
			dfs_init_status = DFS_INIT_AUTO_ENABLE_FAIL;
			goto err_out;
		}
	}

	for (i = 0, j = 0; i < FREQ_NUM; i++) {
		err = inq_dfs_msg(&freq_table[j], i, DFS_CMD_INQ_DDR_TABLE);
		if (freq_table[j] != 0)
			j++;
	}

	dfs_freq_table_sort(freq_table, j);

	if (j > 0)
		j--;

	if (freq_table[0] >= freq_table[j])
		pr_err("%s, get limit freq fail: min %d, max %d",
			__func__, freq_table[0], freq_table[j]);
	else {
		data->devfreq->min_freq = freq_table[0];
		min_freq = freq_table[0];
		data->devfreq->max_freq = freq_table[j];
		max_freq = freq_table[j];
		pr_debug("%s, get freq: min %d, max %d",
			__func__, freq_table[0], freq_table[j]);
	}

	last_freq = max_freq;

	if (force_high_freq)
		dfs_request(force_high_freq, DFS_CMD_SET_DDR_FREQ);

	scene_dfs_request("lcdon");
	dfs_init_status = DFS_INIT_FINISH;

err_out:
	return 0;
}

static int dfs_freq_target(struct device *dev, unsigned long *freq,
				u32 flags)
{
	int err;

	err = dfs_request(*freq, DFS_CMD_NORMAL);
	if (err < 0)
		pr_err("%s: set freq fail: %d\n", __func__, err);
	return err;
}

static int dfs_get_dev_status(struct device *dev,
				      struct devfreq_dev_status *stat)
{
	return 0;
}

static void dfs_exit(struct device *dev)
{
	return;
}

static struct devfreq_dev_profile dfs_freq_profile = {
	.initial_freq	= DFS_INITIAL_FREQ,
	.polling_ms	= DFS_POLLING_MS,
	.target		= dfs_freq_target,
	.get_dev_status	= dfs_get_dev_status,
	.exit		= dfs_exit,
};

static int dfs_freq_probe(struct platform_device *pdev)
{
	struct dmcfreq_data *data;
	struct device *dev = &pdev->dev;
	int err = 0;
	int ret, count, i;
	struct scene_freq *scene = NULL;
	char *scene_name;
	unsigned int freq;

	scenario_data = NULL;

	INIT_LIST_HEAD(&scene_dfs_list);
	err = of_property_read_u32_array(dev->of_node, "overflow",
			dfs_para_init.overflow, 4);
	if (err != 0) {
		pr_err("failed read overflow\n");
		dfs_para_init.overflow_inited = 0;
	} else {
		dfs_para_init.overflow_inited = 1;
	}
	err = of_property_read_u32_array(dev->of_node, "underflow",
			dfs_para_init.underflow, 4);
	if (err != 0) {
		pr_err("failed read underflow\n");
		dfs_para_init.underflow_inited = 0;
	} else {
		dfs_para_init.underflow_inited = 1;
	}
	err = of_property_read_u32_array(dev->of_node, "timer",
			&dfs_para_init.timer, 1);
	if (err != 0) {
		pr_err("Failed read timer\n");
		dfs_para_init.timer_inited = 0;
	} else {
		dfs_para_init.timer_inited = 1;
	}
	count = of_property_count_strings(dev->of_node, "sprd-scene");
	if (count < 0) {
		dev_err(dev, "Failed read scene dfs\n");
		count = 0;
	}
	pr_emerg("###### dmc dfs name: count: %d\n", count);

	data = kzalloc(sizeof(struct dmcfreq_data) +
			count * sizeof(struct scene_freq), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory.\n");
		return -ENOMEM;
	}

	data->scenefreq = (struct scene_freq *)(data + 1);
	for (i = 0; i < count; i++) {
		pr_emerg("### sc_ptr: 0x%lx\n", (ulong)&data->scenefreq[i]);
		scene = &data->scenefreq[i];
		ret = of_property_read_string_index(dev->of_node,
				"sprd-scene", i, (const char **)&scene_name);
		if (ret < 0 || strlen(scene_name) == 0) {
			dev_err(dev, "Failed to read sprd dfs scene name\n");
			return ret;
		}
		pr_emerg("###### dmc dfs name: scent0x%lx\n", (ulong)scene);
		scene->scene_name = scene_name;
		ret = of_property_read_u32_index(dev->of_node, "sprd-freq",
				i, &freq);
		if (ret) {
			dev_err(dev, "fail to read sprd dfs scene frequency\n");
			return ret;
		}
		scene->scene_freq = freq;
		pr_emerg("###### dmc dfs name: count %d name %s freq: %d\n",
				count, scene->scene_name, scene->scene_freq);
	}

	err = of_property_read_u32(dev->of_node, "force-high-freq", &freq);
	if (err != 0)
		force_high_freq = 0;
	else
		force_high_freq = freq;

	data->scene_count = count;
	spin_lock_init(&data->lock);
	data->last_jiffies = jiffies;
	data->boot_done = jiffies + BOOT_TIME;
	platform_set_drvdata(pdev, data);

	data->devfreq = devfreq_add_device(dev, &dfs_freq_profile,
			"sprd_governor", NULL);
	if (IS_ERR(data->devfreq)) {
		err = PTR_ERR(data->devfreq);
		dev_err(dev, "Failed to add device\n");
		goto err_opp_add;
	}

	data->devfreq->min_freq = dfs_min_freq(data);
	data->devfreq->max_freq = dfs_max_freq(data);
	scenario_data = data;
	dfs_scene_count = count;
	dfs_smsg_ch_open = kthread_run(dfs_smsg_thread, data,
			"dfs-%d-%d", SIPC_ID_PM_SYS, SMSG_CH_PM_CTRL);
	if (IS_ERR(dfs_smsg_ch_open)) {
		pr_err("Failed to create kthread: dfs-%d-%d\n",
					SIPC_ID_PM_SYS, SMSG_CH_PM_CTRL);
		err = -EINVAL;
		goto err_devfreq_add;
	}

	return 0;

err_devfreq_add:
	devfreq_remove_device(data->devfreq);
err_opp_add:
	kfree(data);
	return err;
}

static int dfs_freq_remove(struct platform_device *pdev)
{
	struct dmcfreq_data *data = platform_get_drvdata(pdev);

	devfreq_remove_device(data->devfreq);
	kfree(data);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dfs_freq_match[] = {
	{ .compatible = "sprd,dfs" },
	{},
};

MODULE_DEVICE_TABLE(of, dfs_freq_match);
#endif

static struct platform_driver dfs_freq_drvier = {
	.probe = dfs_freq_probe,
	.remove = dfs_freq_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd_dfs",
		.of_match_table = dfs_freq_match,
		},
};

module_platform_driver(dfs_freq_drvier);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("dfs driver with devfreq framework");
