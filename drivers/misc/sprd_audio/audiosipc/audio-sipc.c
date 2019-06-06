/*
 * driver/sipc/audio-sipc.c
 *
 * SPRD SoC AUDIO -- SpreadTrum SOC SIPC for AUDIO Common function.
 *
 * Copyright (C) 2015 SpreadTrum Ltd.
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

#define pr_fmt(fmt) "[Audio:SIPC] "fmt

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <sound/soc.h>

#include "audio_mem.h"
#include "audio-sipc.h"
#include "sprd-asoc-common.h"
#include "sprd-string.h"

/*add by ninglei*/
#define AUDIO_SMSG_RINGHDR_EXT_SIZE	(16)
#define AUDIO_SMSG_RINGHDR_EXT_TXRDPTR	(0)
#define AUDIO_SMSG_RINGHDR_EXT_TXWRPTR	(4)
#define AUDIO_SMSG_RINGHDR_EXT_RXRDPTR	(8)
#define AUDIO_SMSG_RINGHDR_EXT_RXWRPTR	(12)

#define AUDIO_VBC_SHM_MAX		6

#define CMD_SEND_TIMEOUT msecs_to_jiffies(3000)

struct sprd_audio_sharemem {
	int id;
	int type;
	unsigned int phy_iram_addr;
	uint32_t size;
};

struct aud_block_param {
	int type;
	unsigned int addr_p;
	phys_addr_t addr_v;
	uint32_t size;
};

#define MAX_BLOCK_PARAMETERS	8

#define PAD_BYTES		0x10
#define PAD_VALUE		0x5a

struct audio_ipc {
	/* start virtual address in iram of command para txbuf */
	unsigned long		param_addr_v;
	/* start physical address in iram of command para txbuf */
	unsigned long		param_addr_p;
	/* byte size of para txbuf */
	uint32_t	param_size;
	/* lock for send-buffer */
	spinlock_t	param_lock;

	struct aud_block_param block_param[MAX_BLOCK_PARAMETERS];

	struct mutex	lock;		/* lock for send-buffer */
	struct mutex	lock_block_param;

	unsigned long smsg_base_v;
	uint32_t  size_inout;
};

struct audio_ipc g_aud_ipc;

static struct audio_ipc *aud_ipc_get(void)
{
	return (struct audio_ipc *)&g_aud_ipc;
}

static int  aud_ipc_block_param_lock(void)
{
	struct audio_ipc *aud_ipc = aud_ipc_get();

	mutex_lock(&aud_ipc->lock_block_param);

	return 0;
}

static int  aud_ipc_block_param_unlock(void)
{
	struct audio_ipc *aud_ipc = aud_ipc_get();

	mutex_unlock(&aud_ipc->lock_block_param);

	return 0;
}

static int  aud_ipc_lock(void)
{
	struct audio_ipc *aud_ipc = aud_ipc_get();

	mutex_lock(&aud_ipc->lock);

	return 0;
}

static int  aud_ipc_unlock(void)
{
	struct audio_ipc *aud_ipc = aud_ipc_get();

	mutex_unlock(&aud_ipc->lock);

	return 0;
}

static int audio_cmd_copy(void *para, size_t n)
{
	struct audio_ipc *aud_ipc = aud_ipc_get();

	if (aud_ipc->param_addr_v == 0) {
		pr_err("%s param_addr_v is NULLL\n", __func__);
		return -1;
	}

	if (n > aud_ipc->param_size) {
		pr_err("%s, data-size(%zu) out of range(%d)!\n",
				__func__, n, aud_ipc->param_size);
		return -1;
	}
	/* write cmd para */
	unalign_memcpy((void *)(aud_ipc->param_addr_v), para, n);

	sp_asoc_pr_dbg("%s, write cmd para: txbuf_addr_v=0x%lx, tx_len=%zu\n",
			__func__, aud_ipc->param_addr_v, n);

	return 0;
}

static int audio_sipc_create(int target_id)
{
	size_t smsg_txaddr;
	size_t smsg_rxaddr;
	int ret;
	uint32_t smsg_base_p = 0;
	size_t smsg_base_v = 0;
	uint32_t  size_inout = 0;
	uint32_t smsg_txsize = 0;
	uint32_t smsg_rxsize = 0;
	static struct aud_smsg_ipc s_sipc_inst;
	struct audio_ipc *sipc = aud_ipc_get();

	smsg_base_p = audio_mem_alloc(IRAM_SMS, &size_inout);
	if (!smsg_base_p)
		return -1;

	smsg_base_v = (size_t)audio_mem_vmap(smsg_base_p, size_inout, 1);
	if (!smsg_base_v) {
		pr_err("%s:ioremap txbuf_addr return NULL\n", __func__);
		return -1;
	}

	sipc->smsg_base_v = smsg_base_v;
	sipc->size_inout = size_inout;

	memset_io((void *)smsg_base_v, 0, size_inout);
	smsg_txsize = (size_inout - AUDIO_SMSG_RINGHDR_EXT_SIZE)/2;
	smsg_rxsize = smsg_txsize;
	smsg_txaddr = smsg_base_v;
	smsg_rxaddr = smsg_base_v + smsg_txsize;
	pr_info("%s: ioremap txbuf: vbase=%#lx, pbase=0x%x, size=0x%x\n",
		__func__, smsg_txaddr, smsg_base_p, smsg_txsize);

	s_sipc_inst.dst = AUD_IPC_AGDSP;
	s_sipc_inst.name = "ap-agdsp-ipc";
	s_sipc_inst.txbuf_size = ((uint32_t)smsg_txsize) /
		((uint32_t)sizeof(struct aud_smsg));
	s_sipc_inst.txbuf_addr = smsg_txaddr;
	s_sipc_inst.txbuf_rdptr = smsg_txaddr + smsg_txsize +
		smsg_rxsize + (size_t)AUDIO_SMSG_RINGHDR_EXT_TXRDPTR;
	s_sipc_inst.txbuf_wrptr = smsg_txaddr + smsg_txsize +
		smsg_rxsize + (size_t)AUDIO_SMSG_RINGHDR_EXT_TXWRPTR;

	s_sipc_inst.rxbuf_size = ((uint32_t)smsg_rxsize) /
		((uint32_t)sizeof(struct aud_smsg));
	s_sipc_inst.rxbuf_addr = smsg_rxaddr;
	s_sipc_inst.rxbuf_rdptr = smsg_txaddr + smsg_txsize +
		smsg_rxsize + (size_t)AUDIO_SMSG_RINGHDR_EXT_RXRDPTR;
	s_sipc_inst.rxbuf_wrptr = smsg_txaddr + smsg_txsize +
		smsg_rxsize + (size_t)AUDIO_SMSG_RINGHDR_EXT_RXWRPTR;

	s_sipc_inst.target_id = target_id;
	/* create SIPC to target-id */
	ret = aud_smsg_ipc_create(s_sipc_inst.dst, &s_sipc_inst);
	if (ret)
		return -1;

	pr_err("%s successfully\n", __func__);

	return 0;
}

static int audio_sipc_destroy(uint8_t dst)
{
	int ret = 0;

	if (dst >= AUD_IPC_NR) {
		pr_err("%s, invalid dst:%d\n", __func__, dst);
		return -1;
	}
	ret = aud_smsg_ipc_destroy(dst);
	if (ret)
		return -1;

	return 0;
}

static int aud_block_param_init(struct aud_block_param *block_param, int type)
{
	uint32_t buf_p = 0;
	size_t buf_v = 0;
	uint32_t size = 0;

	buf_p = audio_mem_alloc(type, &size);
	if (buf_p)
		buf_v = (size_t)audio_mem_vmap((phys_addr_t)buf_p, size, 1);
	else
		return -1;

	memset_io((void *)buf_v, 0, size);
	block_param->addr_p = buf_p;
	block_param->addr_v = buf_v;
	block_param->type = type;
	block_param->size =  size;

	return 0;
}

static int aud_ipc_command_param_init(void)
{
	struct audio_ipc *aud_ipc = aud_ipc_get();

	aud_ipc->param_addr_p = audio_mem_alloc(IRAM_SMS_COMMD_PARAMS,
		&aud_ipc->param_size);
	if (aud_ipc->param_addr_p) {
		aud_ipc->param_addr_v = (size_t)audio_mem_vmap(
							aud_ipc->param_addr_p,
							aud_ipc->param_size, 1);
		if (aud_ipc->param_addr_v == 0)
			pr_err("%s ioremap_nocache failed\n", __func__);

	} else {
		return -1;
	}

	return 0;
}

static int aud_ipc_block_param_init(void)
{
	struct audio_ipc *aud_ipc = aud_ipc_get();

	aud_block_param_init(&aud_ipc->block_param[0], IRAM_SHM_AUD_STR);
	aud_block_param_init(&aud_ipc->block_param[1], IRAM_SHM_DSP_VBC);
	aud_block_param_init(&aud_ipc->block_param[2], IRAM_SHM_NXP);
	aud_block_param_init(&aud_ipc->block_param[3], IRAM_SHM_REG_DUMP);

	return 0;
}

static struct aud_block_param *aud_ipc_block_param_get(int type)
{
	int i = 0;
	struct audio_ipc *aud_ipc = aud_ipc_get();

	for (i = 0; i < MAX_BLOCK_PARAMETERS; i++) {
		if ((aud_ipc->block_param[i].type == type) &&
			aud_ipc->block_param[i].size) {
			return  &aud_ipc->block_param[i];
		}
	}

	return NULL;
}

uint32_t aud_ipc_dump(void *buf, uint32_t buf_bytes)
{
	uint32_t bytes = 0;
	int i = 0;
	struct audio_ipc *sipc = aud_ipc_get();

	unalign_memcpy(buf, (void *)sipc->smsg_base_v, sipc->size_inout);
	bytes += sipc->size_inout;
	unalign_memset((char *)buf + bytes, PAD_VALUE, PAD_BYTES);
	bytes += PAD_BYTES;
	unalign_memcpy((char *)buf + bytes, (void *)sipc->param_addr_v,
		       sipc->param_size);
	bytes += sipc->param_size;
	unalign_memset((char *)buf + bytes, PAD_VALUE+1, PAD_BYTES);
	bytes += PAD_BYTES;

	for (i = 0; i < MAX_BLOCK_PARAMETERS; i++) {
		if (sipc->block_param[i].size) {
			if ((bytes + sipc->block_param[i].size) < buf_bytes) {
				unalign_memcpy((char *)buf + bytes,
					(void *)sipc->block_param[i].addr_v,
					sipc->block_param[i].size);
				bytes += sipc->block_param[i].size;
			}
		}
	}

	if ((bytes + PAD_BYTES) < buf_bytes) {
		unalign_memset((char *)buf + bytes, PAD_VALUE+2, PAD_BYTES);
		bytes += PAD_BYTES;
	}

	return bytes;
}

static int aud_ipc_init(int target_id)
{
	int ret = 0;
	/* create sipc for audio */
	struct audio_ipc *aud_ipc = aud_ipc_get();

	pr_info("%s", __func__);
	mutex_init(&aud_ipc->lock_block_param);
	mutex_init(&aud_ipc->lock);
	ret = audio_sipc_create(target_id);
	if (ret) {
		pr_err("%s: failed to create sipc, ret=%d\n", __func__, ret);
		return -ENODEV;
	}
	ret = aud_ipc_command_param_init();
	if (ret < 0) {
		pr_err("%s %d aud_ipc_command_param_init failed\n",
			__func__, __LINE__);
		audio_sipc_destroy(AUD_IPC_AGDSP);
		return -1;
	}

	/* config block iram used */
	ret = aud_ipc_block_param_init();
	if (ret < 0) {
		pr_err("%s %d aud_ipc_block_param_init failed\n",
			__func__, __LINE__);
		audio_sipc_destroy(AUD_IPC_AGDSP);
		return -1;
	}

	return 0;
}

void snd_vbc_sipc_deinit(void)
{
	aud_ipc_ch_close(AMSG_CH_VBC_CTL);
	aud_smsg_ipc_destroy(AUD_IPC_AGDSP);
}

int aud_ipc_ch_open(uint16_t channel)
{
	int ret = 0;

	ret = aud_smsg_ch_open(AUD_IPC_AGDSP, channel);
	if (ret != 0) {
		pr_err("%s, Failed to open channel\n", __func__);
		return -1;
	}

	return 0;
}

int aud_ipc_ch_close(uint16_t channel)
{
	int ret = 0;

	ret = aud_smsg_ch_close(AUD_IPC_AGDSP, channel);
	if (ret != 0) {
		pr_err("%s, Failed to close channel\n", __func__);
		return -1;
	}

	return 0;
}

int aud_recv_cmd(uint16_t channel, uint32_t cmd, int *result,
	int32_t timeout)
{
	int ret = 0;
	struct aud_smsg mrecv = { 0 };

	aud_smsg_set(&mrecv, channel, cmd, 0, 0, 0, 0);

	ret = aud_smsg_recv(AUD_IPC_AGDSP, &mrecv, timeout);
	if (ret < 0) {
		if (-ENODATA == ret) {
			pr_warn("%s channel =%u cmd=%u ENODATA\n",
				__func__, channel, cmd);
			return ret;
		}
		pr_err("%s, Failed to recv channel =%d cmd(0x%x),ret(%d)\n",
			__func__, channel, cmd, ret);
		return -EIO;
	}
	sp_asoc_pr_dbg("%s, chan: 0x%x, cmd: 0x%x, value0: 0x%x, value1: 0x%x,",
		__func__, mrecv.channel, mrecv.command,
		mrecv.parameter0, mrecv.parameter1);
	sp_asoc_pr_dbg(" value2: 0x%x, value3: 0x%x, timeout: %d\n",
		mrecv.parameter2, mrecv.parameter3, timeout);

	if (cmd == mrecv.command && mrecv.channel == channel) {
		*result = mrecv.parameter3;
		return 0;
	}
	pr_err("%s, Haven't got right cmd(0x%x), got cmd(0x%x)",
		__func__, cmd, mrecv.command);
	pr_err(" got chan(0x%x)\n",
		mrecv.channel);

	return -EIO;

}

int aud_send_msg(uint16_t channel, uint32_t cmd, uint32_t value0,
		 uint32_t value1, uint32_t value2, int32_t value3)
{
	int ret = 0;
	struct aud_smsg msend = { 0 };

	sp_asoc_pr_dbg("%s,cmd: 0x%x, value0: 0x%x, value1: 0x%x,",
		__func__, cmd, value0, value1);
	sp_asoc_pr_dbg(" value2: 0x%x, value3: 0x%x\n",
		value2, value3);

	aud_smsg_set(&msend, channel, cmd, value0, value1, value2, value3);

	ret = aud_smsg_send(AUD_IPC_AGDSP, &msend);
	if (ret) {
		pr_err("%s, Failed to send cmd(0x%x), ret(%d)\n",
			__func__, cmd, ret);
		return -EIO;
	}

	return 0;
}

int aud_send_cmd_no_wait(uint16_t channel, uint32_t cmd,
	uint32_t value0, uint32_t value1, uint32_t value2, int32_t value3)
{
	int ret;

	pr_info("%s no wait\n", __func__);
	ret = aud_send_msg(channel, cmd,
	value0, value1, value2, value3);
	if (ret < 0) {
		pr_err("%s: channel=%d failed to send command(%d), ret=%d\n",
			__func__, channel, cmd, ret);

		return -EIO;
	}

	return 0;
}

int aud_send_cmd_no_param(uint16_t channel, uint32_t cmd,
	uint32_t value0, uint32_t value1, uint32_t value2, int32_t value3,
	int32_t timeout)
{
	int ret = 0;
	int value = 0;
	int repeat_count = 0;

	/*clean the recev command buffer first */
	do {
		ret = aud_recv_cmd(channel, cmd, &value, 0);
	} while (ret == 0);

	/* send audio cmd */
	ret = aud_send_msg(channel, cmd,
		value0, value1, value2, value3);
	if (ret < 0) {
		pr_err("%s: failed to channel=%d send command(%d), ret=%d\n",
				__func__, channel, cmd, ret);
		goto err;
	}

	do {
		ret = aud_recv_cmd(channel, cmd, &value, timeout);
		repeat_count++;
		pr_warn("%s channel=%d, cmd=%d repeat_count=%d\n",
			__func__, channel, cmd, repeat_count);
	} while ((ret < 0) && (repeat_count < 4));

	if (ret < 0) {
		pr_err("%s: failed to get channel= %d command(%d), ret=%d\n",
			__func__, channel, cmd, ret);
		goto err;
	}
	pr_info(
		"%s out,channel = %d cmd =%d ret-value:%d,repeat_count=%d\n",
		__func__, channel, cmd, value, repeat_count);

	return ret;
err:

	return -1;
}

int aud_send_use_noreplychan(
	uint32_t cmd,
	uint32_t value0, uint32_t value1,
	uint32_t value2, int32_t value3)
{
	return aud_send_cmd_no_wait(
		AMSG_CH_DSP_GET_PARAM_FROM_SMSG_NOREPLY,
		cmd, value0, value1, value2, value3);
}

/*
 * id : parameter0(exchange with dsp)
 * cmd: command
 */
int aud_send_cmd(uint16_t channel, int id, int stream,
	uint32_t cmd, void *para, size_t n, int32_t timeout)
{
	int ret = 0;
	int value = 0;
	int repeat_count = 0;
	struct audio_ipc *aud_ipc = aud_ipc_get();

	aud_ipc_lock();

	/* set audio cmd para */
	ret = audio_cmd_copy(para, n);
	if (ret < 0) {
		pr_err("%s: failed to write command(%d) para, ret=%d\n",
				__func__, cmd, ret);
		goto err;
	}

	/* clean the recev command buffer first */
	do {
		ret = aud_recv_cmd(channel, cmd, &value, 0);
	} while (ret == 0);
	sp_asoc_pr_info("%s in,cmd =%d id:%d ret-value:%d\n",
		__func__, cmd, id, value);

	/* send audio cmd */
	ret = aud_send_msg(channel, cmd,
		id, stream, aud_ipc->param_addr_p, 0);
	if (ret < 0) {
		pr_err("%s: failed to send command(%d), ret=%d\n",
				__func__, cmd, ret);
		goto err;
	}

	do {
		ret = aud_recv_cmd(channel, cmd, &value, CMD_SEND_TIMEOUT);
		repeat_count++;
	} while ((ret < 0) && (repeat_count < 4));

	if (ret < 0) {
		pr_err("%s: failed to get command(%d), ret=%d\n",
				__func__, cmd, ret);
		goto err;
	}

	aud_ipc_unlock();
	sp_asoc_pr_info("%s out,cmd =%d id:%d ret-value:%d,repeat_count=%d\n",
		__func__, cmd, id, value, repeat_count);

	return ret;
err:
	aud_ipc_unlock();

	return -1;
}

int aud_send_block_param(uint16_t channel, int id, int stream,
			int type, void *buf, size_t n, int32_t timeout)
{
	int ret = 0;
	struct sprd_audio_sharemem sharemem_info;
	struct aud_block_param *block_param = NULL;

	unalign_memset(&sharemem_info, 0, sizeof(struct sprd_audio_sharemem));
	aud_ipc_block_param_lock();

	block_param = aud_ipc_block_param_get(type);
	if (!block_param) {
		aud_ipc_block_param_unlock();
		return -1;
	}
	if (block_param->size < n) {
		pr_err("%s block_param->size=%#x < datasize=%#zx\n",
			__func__, block_param->size, n);
		aud_ipc_block_param_unlock();
		return -1;
	}
	/* write cmd para */
	unalign_memcpy((void *)block_param->addr_v, buf, n);
	sharemem_info.id = id;
	sharemem_info.type = type;
	sharemem_info.phy_iram_addr = block_param->addr_p;
	sharemem_info.size = n;

	/* send audio cmd */
	pr_debug("cmd =%d sharemem_info.id = %d, sharemem_info.type=%d,",
		SND_VBC_DSP_IO_SHAREMEM_SET,
		sharemem_info.id, sharemem_info.type);
	pr_debug(" sharemem_info.phy_iram_addr=%#x, sharemem_info.size=%#x\n",
		sharemem_info.phy_iram_addr,
		sharemem_info.size);
	ret = aud_send_cmd(channel, id, stream, SND_VBC_DSP_IO_SHAREMEM_SET,
		&sharemem_info, sizeof(struct sprd_audio_sharemem),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0) {
		aud_ipc_block_param_unlock();
		pr_err("%s: failed to send command(%d), ret=%d\n",
				__func__, SND_VBC_DSP_IO_SHAREMEM_SET, ret);
		return -1;
	}

	aud_ipc_block_param_unlock();

	return 0;
}

int aud_recv_block_param(uint16_t channel, int id, int stream,
			int type, void *buf, uint32_t size, int32_t timeout)
{
	int ret = 0;
	struct sprd_audio_sharemem sharemem_info;
	struct aud_block_param *block_param = NULL;

	unalign_memset(&sharemem_info, 0, sizeof(struct sprd_audio_sharemem));

	block_param =  aud_ipc_block_param_get(type);
	if (!block_param)
		return -1;

	if (block_param->size > size)
		return -1;

	unalign_memset(&sharemem_info, 0, sizeof(struct sprd_audio_sharemem));

	sharemem_info.id = id;
	sharemem_info.type = type;
	/* set iram addr where get info */
	sharemem_info.phy_iram_addr = block_param->addr_p;
	sharemem_info.size = block_param->size;
	/* send audio cmd */
	ret = aud_send_cmd(channel, id, stream, SND_VBC_DSP_IO_SHAREMEM_GET,
		&sharemem_info, sizeof(struct sprd_audio_sharemem), timeout);
	if (ret < 0) {
		pr_err("%s: failed to send command(%d), ret=%d\n",
				__func__, SND_VBC_DSP_IO_SHAREMEM_GET, ret);
		return -1;
	}
	unalign_memcpy(buf, (void *)block_param->addr_v, block_param->size);

	return 0;
}

static int audio_sipc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	int target_id = 0;

	ret = of_property_read_u32(np, "mailbox,core", (u32 *)&target_id);
	if (ret) {
		pr_err("%s parse dt err\n", __func__);
		return -EINVAL;
	}

	aud_ipc_init(target_id);

	return 0;
}

static int audio_sipc_remove(struct platform_device *pdev)
{
	/*do nothing*/
	return 0;
}

static const struct of_device_id audio_sipc_match_table[] = {
	{.compatible = "sprd,audio_sipc", },
	{ },
};

static struct platform_driver audio_sipc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "audio_sipc",
		.of_match_table = audio_sipc_match_table,
	},
	.probe = audio_sipc_probe,
	.remove = audio_sipc_remove,
};

static int __init audio_sipc_init(void)
{
	return platform_driver_register(&audio_sipc_driver);
}

static void __exit audio_sipc_exit(void)
{
	platform_driver_unregister(&audio_sipc_driver);
}

/* after audio_mem probe */
module_init(audio_sipc_init);
module_exit(audio_sipc_exit);

MODULE_DESCRIPTION("audio_sipc");
MODULE_LICENSE("GPL");
