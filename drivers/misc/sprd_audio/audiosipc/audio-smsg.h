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
#ifndef __AUDIO_SMSG_H
#define  __AUDIO_SMSG_H

/*status for ipc internal use*/
#define CHAN_STATE_UNUSED	0
#define CHAN_STATE_WAITING	1
#define CHAN_STATE_OPENED	2
#define CHAN_STATE_FREE		3

#define SMSG_CACHE_NR		256

/* flag for CONNECT/DISCONNECT msg type */
#define	SMSG_CONNECT_MAGIC	0xBEEE
#define	SMSG_DISCONNECT_MAGIC	0xEDDD

/* sipc processor ID definition */
enum {
	AUD_IPC_AP = 0,		/* Application Processor */
	AUD_IPC_AGDSP,		/* AGDSP Processor */
	AUD_IPC_NR,			/* total processor number */
};

/* smsg channel definition */
enum {
	AMSG_CH_VBC_CTL = 0,	/* vbc conrol channel */
	AMSG_CH_MP3_OFFLOAD,	/* MP3 offload control channel */
	AMSG_CH_DSP_ASSERT_CTL, /*audo dsp assert control*/
	AMSG_CH_DSP_PCM,
	AMSG_CH_DSP_LOG,
	AMSG_CH_DSP_MEM,
	AMSG_CH_MP3_OFFLOAD_DRAIN,
	AMSG_CH_DSP_GET_PARAM_FROM_SMSG_NOREPLY,
	AMSG_CH_NR,
};

/* smsg command definition*/
/* first msg to open a channel */
#define SMSG_CMD_CONNECT 1
/* last msg to close a channel */
#define SMSG_CMD_DISCONNECT 2

/* share-mem ring buffer short message */
struct aud_smsg {
	uint16_t command;	/* command */
	uint16_t channel;	/* channel index */
	uint32_t parameter0;	/* msg parameter0 */
	uint32_t parameter1;	/* msg parameter1 */
	uint32_t parameter2;	/* msg parameter2 */
	uint32_t parameter3;	/* msg parameter3 */
};

struct aud_smsg_channel {
	wait_queue_head_t rxwait;	/* wait queue for recv-buffer */
	struct mutex rxlock;

	/* cached msgs for recv */
	uint32_t wrptr[1];
	uint32_t rdptr[1];
	struct aud_smsg caches[SMSG_CACHE_NR];
};

/* aud_smsg ring-buffer between AP/AGDSP ipc */
struct aud_smsg_ipc {
	char *name;
	uint8_t dst;
	uint8_t padding[3];
	size_t txbuf_addr;		/* send-buffer info */
	uint32_t txbuf_size;		/* must be 2^n */
	size_t txbuf_rdptr;
	size_t txbuf_wrptr;

	size_t rxbuf_addr;		/* recv-buffer info */
	uint32_t rxbuf_size;		/* must be 2^n */
	size_t rxbuf_rdptr;
	size_t rxbuf_wrptr;
	int target_id;
	void *irq_handler;		/* sipc irq related */
	struct task_struct *thread;	/* sipc ctrl thread */
	spinlock_t txpinlock;		/* lock for send-buffer */
	spinlock_t rxpinlock;		/* lock for rx-buffer */
	spinlock_t ch_rx_pinlock;	/* lock for rx-buffer */

	/* all fixed channels receivers */
	struct aud_smsg_channel *channels[AMSG_CH_NR];

	/* record the runtime status of smsg channel */
	atomic_t busy[AMSG_CH_NR];

	/* all channel states: 0 unused, 1 opened */
	uint8_t	states[AMSG_CH_NR];
	int	dsp_ready;
};

typedef int (*AGDSP_DUMP_FUNC)(void *private, uint32_t is_timeout);
int aud_smsg_register_dump_func(AGDSP_DUMP_FUNC handler, void *priv_data);

/**
 * aud_smsg_ch_open -- open a channel for aud_smsg
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @return: 0 on success, <0 on failure
 */
int aud_smsg_ch_open(uint8_t dst, uint16_t channel);

/**
 * aud_smsg_ch_close -- close a channel for aud_smsg
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @return: 0 on success, <0 on failure
 */
int aud_smsg_ch_close(uint8_t dst, uint16_t channel);

/**
 * aud_smsg_send -- send smsg for aud_smsg
 *
 * @dst: dest processor ID
 * @msg: smsg body to be sent
 * @return: 0 on success, <0 on failure
 */
int aud_smsg_send(uint8_t dst, struct aud_smsg *msg);

/**
 * aud_smsg_recv -- poll and recv smsg for aud_smsg
 *
 * @dst: dest processor ID
 * @msg: smsg body to be received, channel should be filled as input
 * @timeout: milliseconds, 0 means no wait, -1 means unlimited
 * @return: 0 on success, <0 on failure
 */
int aud_smsg_recv(uint8_t dst, struct aud_smsg *msg, int timeout);

/* create/destroy smsg ipc between AP/CP */
int aud_smsg_ipc_create(uint8_t dst, struct aud_smsg_ipc *ipc);
int aud_smsg_ipc_destroy(uint8_t dst);

/* quickly fill a smsg body */
static inline void aud_smsg_set(struct aud_smsg *msg, uint16_t channel,
				uint16_t cmd,
				uint32_t value0, uint32_t value1,
				uint32_t value2, int value3)
{
	msg->channel = channel;
	msg->command = cmd;
	msg->parameter0 = value0;
	msg->parameter1 = value1;
	msg->parameter2 = value2;
	msg->parameter3 = value3;
}
#endif
