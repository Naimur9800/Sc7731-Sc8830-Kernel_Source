/*
* Copyright (C) 2017 Spreadtrum Communications Inc.
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/
#include "mdbg_sipc.h"

static SPIPE_RX_PROCESS_CALLBACK
	spipe_rx_callback[MDBG_SPIPE_NUM];

/*stype:dst:channel:bufid*/
struct mdbg_spipe g_mdbg_spipe[MDBG_SPIPE_NUM] = {
	{
		.stype = 0,
		.dst = MDBG_SIPC_WCN_DST,
		.channel = MDBG_SIPC_CHANNEL_LOG,
		.bufid = MDBG_SIPC_BUFID_DEFAULT,
		.len = MDBG_SIPC_LOG_SIZE,
		.bufnum = 1,
		.txbufsize = MDBG_SIPC_LOG_TXBUF_SIZE,
		.rxbufsize = MDBG_SIPC_LOG_RXBUF_SIZE,
	},
	{
		.stype = 1,
		.dst = MDBG_SIPC_WCN_DST,
		.channel = MDBG_SIPC_CHANNEL_LOOPCHECK,
		.bufid = MDBG_SIPC_BUFID_DEFAULT,
		.len = MDBG_SIPC_LOOPCHECK_SIZE,
		.bufnum = 1,
		.txbufsize = 0x400,
		.rxbufsize = 0x400,
	},
	{
		.stype = 2,
		.dst = MDBG_SIPC_WCN_DST,
		.channel = MDBG_SIPC_CHANNEL_ATCMD,
		.bufid = MDBG_SIPC_BUFID_ATCMD,
		.len = MDBG_SIPC_AT_CMD_SIZE,
		.bufnum = 16,
		.txbufsize = 0x1000,
		.rxbufsize = 0x1000,
	},
	{
		.stype = 3,
		.dst = MDBG_SIPC_WCN_DST,
		.channel = MDBG_SIPC_CHANNEL_ASSERT,
		.bufid = MDBG_SIPC_BUFID_DEFAULT,
		.len = MDBG_SIPC_ASSERT_SIZE,
		.bufnum = 1,
		.txbufsize = 0x400,
		.rxbufsize = 0x400,
	},
};

static int mdbg_sipc_write(struct mdbg_spipe *p_mdbg_spipe,
		      const unsigned char *buf,
		      int count)
{
	int cnt;

	WCN_DEBUG("sipc write count=%d\n", count);
	cnt = sbuf_write(p_mdbg_spipe->dst,
			 p_mdbg_spipe->channel,
			 p_mdbg_spipe->bufid,
			 (void *)buf, count, -1);
	return cnt;
}

int mdbg_sipc_type_write(void *buf, unsigned int len,
			    unsigned int subtype)
{
	return mdbg_sipc_write(&g_mdbg_spipe[subtype], buf, len);
}

void mdbg_sipc_handler (int event, void *data)
{
	struct  mdbg_spipe *p_mdbg_spipe = data;
	int cnt = 0;
	unsigned char *buf;
	uint8_t stype;

	if (!p_mdbg_spipe)
		return;
	stype = p_mdbg_spipe->stype;
	if (stype >= MDBG_SPIPE_NUM)
		return;
	buf = kzalloc(p_mdbg_spipe->len, GFP_KERNEL);
	if (!buf)
		return;
	switch (event) {
	case SBUF_NOTIFY_WRITE:
		break;
	case SBUF_NOTIFY_READ:
		cnt = sbuf_read(p_mdbg_spipe->dst,
				p_mdbg_spipe->channel,
				p_mdbg_spipe->bufid,
				(void *)buf,
				p_mdbg_spipe->len,
				0);

		 if (cnt > 0) {
			/* log at/loopcheck/assert callback */
			if (spipe_rx_callback[stype])
				spipe_rx_callback[stype](buf, cnt, 0);
			WCN_DEBUG("read cnt %d\n", cnt);
		} else
			WCN_ERR("read cnt invalid!cnt %d\n", cnt);
		break;
	default:
		WCN_DEBUG("Received event is invalid(event=%d)\n", event);
	}

	kfree(buf);
}

int mdbg_sipc_cb_register(unsigned int subtype, void *func)
{
	struct  mdbg_spipe *p_mdbg_spipe;
	int rval;

	MDBG_LOG("register cb :subtype:%d !\n", subtype);

	p_mdbg_spipe = &g_mdbg_spipe[subtype];

	spipe_rx_callback[subtype] = func;

	rval = sbuf_register_notifier(p_mdbg_spipe->dst, p_mdbg_spipe->channel,
			p_mdbg_spipe->bufid, mdbg_sipc_handler, p_mdbg_spipe);

	return 0;
}

void  mdbg_sipc_destroy(void)
{
	u32 i;
	struct mdbg_spipe *p_mdbg_spipe;

	for (i = 0; i < MDBG_SPIPE_NUM; i++) {
		if (i == 2)
			continue;
		p_mdbg_spipe = &g_mdbg_spipe[i];
		sbuf_destroy(p_mdbg_spipe->dst, p_mdbg_spipe->channel);
	}
	WCN_INFO("destroy sbuf success!\n");
}

int mdbg_sipc_init(void)
{
	int i;
	int ret = 0;
	struct mdbg_spipe *p_mdbg_spipe;

	for (i = 0; i < MDBG_SPIPE_NUM; i++) {
		if (i == 2)
			continue;
		p_mdbg_spipe = &g_mdbg_spipe[i];
		ret = sbuf_create(p_mdbg_spipe->dst, p_mdbg_spipe->channel,
			p_mdbg_spipe->bufnum,
			p_mdbg_spipe->txbufsize,
			p_mdbg_spipe->rxbufsize);
		if (ret < 0) {
			mdbg_sipc_destroy();
			WCN_ERR("create sbuf fail!\n");
			return -1;
		}
	}
	WCN_INFO("create sbuf success!\n");

	return 0;
}


