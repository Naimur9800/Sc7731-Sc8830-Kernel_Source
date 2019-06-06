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

#define pr_fmt(fmt) "imsbr: " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <uapi/linux/ims_bridge/ims_bridge.h>

#include "imsbr_core.h"
#include "imsbr_packet.h"
#include "imsbr_sipc.h"

static int volte_video_apsk_set(const char *val, struct kernel_param *kp);

/**
 * Temporary solution, will notify CP imsbr whether the volte video's socket
 * is located at AP (0 - disabled 1 - enabled).
 *
 * -- I hope that this ugly codes will be abandoned one day :(
 */
static unsigned int volte_video_apsk __read_mostly = 1;

module_param_call(volte_video_apsk, volte_video_apsk_set, param_get_uint,
		  &volte_video_apsk, 0644);

int imsbr_notify_ltevideo_apsk(void)
{
	u32 enable = volte_video_apsk;
	struct sblock blk;

	if (imsbr_build_cmd("ltevideo-apsk", &blk, &enable, sizeof(enable)))
		return -1;

	if (imsbr_sblock_send(&imsbr_ctrl, &blk, sizeof(enable)))
		return -1;

	pr_info("notify ltevideo-apsk=%d success!\n", enable);
	return 0;
}

static int volte_video_apsk_set(const char *val, struct kernel_param *kp)
{
	if (volte_video_apsk) {
		pr_info("we use ltevideo-apsk solution\n");
		imsbr_notify_ltevideo_apsk();
	}

	return 0;
}

static void imsbr_pre_notify(struct imsbr_sipc *sipc)
{
	while (1) {
		if (!imsbr_notify_ltevideo_apsk())
			break;

		/* Waiting for CP to be truely alive! */
		schedule_timeout_interruptible(HZ / 100);
	}
}

struct imsbr_sipc imsbr_data __read_mostly = {
	.dst		= SIPC_ID_LTE,
	.channel	= SMSG_CH_IMSBR_DATA,
	.hdrlen		= sizeof(struct imsbr_packet),
	.blksize	= IMSBR_DATA_BLKSZ,
	.blknum		= IMSBR_DATA_BLKNUM,
	.peer_ready	= ATOMIC_INIT(0),
	.process	= imsbr_process_packet,
};

struct imsbr_sipc imsbr_ctrl __read_mostly = {
	.dst		= SIPC_ID_LTE,
	.channel	= SMSG_CH_IMSBR_CTRL,
	.hdrlen		= sizeof(struct imsbr_msghdr),
	.blksize	= IMSBR_CTRL_BLKSZ,
	.blknum		= IMSBR_CTRL_BLKNUM,
	.peer_ready	= ATOMIC_INIT(0),
	.pre_hook	= imsbr_pre_notify,
	.process	= imsbr_process_msg,
};

int imsbr_build_cmd(const char *cmd, struct sblock *blk,
		    void *payload, int paylen)
{
	struct imsbr_msghdr _msg;

	if (paylen > IMSBR_MSG_MAXLEN) {
		pr_err("paylen %d is too large, max is %d\n",
		       paylen, (int)IMSBR_MSG_MAXLEN);
		WARN_ON_ONCE(1);
		return -1;
	}

	if (imsbr_sblock_get(&imsbr_ctrl, blk, IMSBR_MSG_MAXLEN))
		return -1;

	_msg.imsbr_version = IMSBR_MSG_VERSION;
	_msg.imsbr_paylen = paylen;
	strlcpy(_msg.imsbr_cmd, cmd, IMSBR_CMD_MAXSZ);

	unalign_memcpy(blk->addr, &_msg, sizeof(_msg));

	if (payload) {
		struct imsbr_msghdr *m = blk->addr;

		unalign_memcpy(m->imsbr_payload, payload, paylen);
	}

	return 0;
}

int imsbr_sblock_receive(struct imsbr_sipc *sipc, struct sblock *blk)
{
	int err;

	err = sblock_receive(sipc->dst, sipc->channel, blk, -1);
	if (unlikely(err < 0)) {
		IMSBR_STAT_INC(sipc_receive_fail);
		pr_err("sblock_receive %s fail, error=%d\n",
		       sipc->desc, err);
		return -1;
	}

	return 0;
}

int imsbr_sblock_get(struct imsbr_sipc *sipc, struct sblock *blk, int size)
{
	int err;

	size += sipc->hdrlen;

	err = sblock_get(sipc->dst, sipc->channel, blk, 0);
	if (unlikely(err < 0)) {
		IMSBR_STAT_INC(sipc_get_fail);
		pr_err("sblock_get %s fail, error=%d\n",
		       sipc->desc, err);
		return -1;
	}

	if (unlikely(blk->length < size)) {
		pr_err("sblock len %d is too short, alloc size is %d\n",
		       blk->length, size);
		imsbr_sblock_put(sipc, blk);
		return -1;
	}

	return 0;
}

int imsbr_sblock_send(struct imsbr_sipc *sipc, struct sblock *blk, int size)
{
	int err;

	blk->length = sipc->hdrlen + size;

	err = sblock_send(sipc->dst, sipc->channel, blk);
	if (unlikely(err < 0)) {
		IMSBR_STAT_INC(sipc_send_fail);
		pr_err("sblock_send %s fail, error=%d\n",
		       sipc->desc, err);
		sblock_put(sipc->dst, sipc->channel, blk);
		return -1;
	}

	return 0;
}

void imsbr_sblock_release(struct imsbr_sipc *sipc, struct sblock *blk)
{
	sblock_release(sipc->dst, sipc->channel, blk);
}

void imsbr_sblock_put(struct imsbr_sipc *sipc, struct sblock *blk)
{
	sblock_put(sipc->dst, sipc->channel, blk);
}

static void imsbr_handler(int event, void *data)
{
	struct imsbr_sipc *sipc = (struct imsbr_sipc *)data;

	WARN_ON(!sipc);

	/* Receieve event means peer in CP side is ok! */
	if (!atomic_cmpxchg(&sipc->peer_ready, 0, 1)) {
		pr_info("recv %d event, %s's peer is alive!\n",
			event, sipc->desc);
		complete_all(&sipc->peer_comp);
	}

	switch (event) {
	case SBLOCK_NOTIFY_GET:
	case SBLOCK_NOTIFY_RECV:
	case SBLOCK_NOTIFY_STATUS:
	case SBLOCK_NOTIFY_OPEN:
	case SBLOCK_NOTIFY_CLOSE:
		pr_debug("%s recv event %d\n", sipc->desc, event);
		break;
	default:
		pr_err("%s recv event %d\n", sipc->desc, event);
		break;
	}
}

static int imsbr_kthread(void *arg)
{
	struct imsbr_sipc *sipc = arg;

	set_user_nice(current, -20);
	wait_for_completion_interruptible(&sipc->peer_comp);
	pr_info("%s is ok, kthread run!\n", sipc->desc);

	if (sipc->pre_hook)
		sipc->pre_hook(sipc);

	while (!kthread_should_stop()) {
		struct sblock blk;

		if (imsbr_sblock_receive(sipc, &blk)) {
			schedule_timeout_interruptible(HZ / 50);
			continue;
		}

		if (blk.length < sipc->hdrlen) {
			pr_err("%s recv len=%d, min=%d, too short!\n",
			       sipc->desc, blk.length, sipc->hdrlen);
			imsbr_sblock_release(sipc, &blk);
		} else if (blk.length > sipc->blksize) {
			pr_err("%s recv len=%d, max=%d, too long!\n",
			       sipc->desc, blk.length, sipc->blksize);
			imsbr_sblock_release(sipc, &blk);
		} else {
			sipc->process(sipc, &blk, true);
		}
	}

	return 0;
}

static int imsbr_sipc_create(struct imsbr_sipc *sipc)
{
	struct task_struct *tsk;
	int err;

	snprintf(sipc->desc, sizeof(sipc->desc), "%s[%d-%d]",
		 sipc == &imsbr_ctrl ? "ctrl" : "data",
		 sipc->dst, sipc->channel);

	init_completion(&sipc->peer_comp);

	err = sblock_create(sipc->dst, sipc->channel, sipc->blknum,
			    sipc->blksize, sipc->blknum, sipc->blksize);
	if (err) {
		pr_err("sblock_create %s fail, error=%d\n",
		       sipc->desc, err);
		return -1;
	}

	err = sblock_register_notifier(sipc->dst, sipc->channel,
				       imsbr_handler, sipc);
	if (err) {
		pr_err("sblock_register_notifier %s fail, error=%d\n",
		       sipc->desc, err);
		goto fail;
	}

	tsk = kthread_run(imsbr_kthread, sipc, "imsbr-%s", sipc->desc);
	if (IS_ERR(tsk)) {
		pr_err("%s kthread_create fail: %ld\n",
		       sipc->desc, PTR_ERR(tsk));
		goto fail;
	}

	sipc->task = tsk;
	return 0;

fail:
	sblock_destroy(sipc->dst, sipc->channel);
	return -1;
}

static void imsbr_sipc_destroy(struct imsbr_sipc *sipc)
{
	if (sipc->task)
		kthread_stop(sipc->task);

	sblock_destroy(sipc->dst, sipc->channel);
}

int __init imsbr_sipc_init(void)
{
	if (imsbr_sipc_create(&imsbr_data))
		goto err_data;

	if (imsbr_sipc_create(&imsbr_ctrl))
		goto err_ctrl;

	return 0;

err_ctrl:
	imsbr_sipc_destroy(&imsbr_data);
err_data:
	return -1;
}

void imsbr_sipc_exit(void)
{
	imsbr_sipc_destroy(&imsbr_ctrl);
	imsbr_sipc_destroy(&imsbr_data);
}
