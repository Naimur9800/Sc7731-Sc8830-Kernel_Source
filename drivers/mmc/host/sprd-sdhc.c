/*
 * linux/drivers/mmc/host/sprd-sdhc.c - Secure Digital Host Controller
 * Interface driver
 *
 * Copyright (C) 2015 Spreadtrum corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

#include "sprd-sdhc.h"

#define DRIVER_NAME "sprd_sdhc"

#ifdef CONFIG_PM
static int sprd_sdhc_runtime_pm_get(struct sprd_sdhc_host *host)
{
	return pm_runtime_get_sync(host->mmc->parent);
}

static int sprd_sdhc_runtime_pm_put(struct sprd_sdhc_host *host)
{
	pm_runtime_mark_last_busy(host->mmc->parent);
	return pm_runtime_put_autosuspend(host->mmc->parent);
}
#else
static inline int sprd_sdhc_runtime_pm_get(struct sprd_sdhc_host *host)
{
	return 0;
}

static inline int sprd_sdhc_runtime_pm_put(struct sprd_sdhc_host *host)
{
	return 0;
}
#endif

void dump_sdio_reg(struct sprd_sdhc_host *host)
{
	if (!host->mmc->card)
		return;

	print_hex_dump(KERN_INFO, "sdhc register: ", DUMP_PREFIX_OFFSET,
			16, 4, host->ioaddr, 64, 0);
}

static void sprd_reset_ios(struct sprd_sdhc_host *host)
{
	sprd_sdhc_disable_all_int(host);

	host->ios.clock = 0;
	host->ios.vdd = 0;
	host->ios.power_mode = MMC_POWER_OFF;
	host->ios.bus_width = MMC_BUS_WIDTH_1;
	host->ios.timing = MMC_TIMING_LEGACY;
	host->ios.signal_voltage = MMC_SIGNAL_VOLTAGE_330;

	sprd_sdhc_reset(host, SPRD_SDHC_BIT_RST_ALL);
	sprd_sdhc_set_delay(host, host->write_delay,
			  host->read_pos_delay, host->read_neg_delay);
}

static void sprd_get_rsp(struct sprd_sdhc_host *host)
{
	u32 i, offset;
	unsigned int flags = host->cmd->flags;
	u32 *resp = host->cmd->resp;

	if (!(flags & MMC_RSP_PRESENT))
		return;

	if (flags & MMC_RSP_136) {
		/* CRC is stripped so we need to do some shifting. */
		for (i = 0, offset = 12; i < 3; i++, offset -= 4) {
			resp[i] =
			    sprd_sdhc_readl(host,
					  SPRD_SDHC_REG_32_RESP + offset) << 8;
			resp[i] |=
			    sprd_sdhc_readb(host,
					  SPRD_SDHC_REG_32_RESP + offset - 1);
		}
		resp[3] = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_RESP) << 8;
	} else {
		resp[0] = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_RESP);
	}
}

static void sprd_send_cmd(struct sprd_sdhc_host *host, struct mmc_command *cmd)
{
	struct mmc_data *data = cmd->data;
	int sg_cnt;
	u32 flag = 0;
	u16 rsp_type = 0;
	int if_has_data = 0;
	int if_mult = 0;
	int if_read = 0;
	int if_dma = 0;
	u16 auto_cmd = SPRD_SDHC_BIT_ACMD_DIS;

	pr_debug("%s(%s)  CMD%d, arg 0x%x, flag 0x%x\n", __func__,
	       host->device_name, cmd->opcode, cmd->arg, cmd->flags);
	if (cmd->data)
		pr_debug("%s(%s) block size %d, cnt %d\n", __func__,
		       host->device_name, cmd->data->blksz, cmd->data->blocks);

	sprd_sdhc_disable_all_int(host);

	if (MMC_ERASE == cmd->opcode) {
		/*
		 * if it is erase command , it's busy time will long,
		 * so we set long timeout value here.
		 */
		mod_timer(&host->timer, jiffies +
			msecs_to_jiffies(host->mmc->max_busy_timeout + 1000));
		sprd_sdhc_writeb(host,
				SPRD_SDHC_DATA_TIMEOUT_MAX_VAL,
				SPRD_SDHC_REG_8_TIMEOUT);
	} else {
		mod_timer(&host->timer,
			jiffies + (SPRD_SDHC_MAX_TIMEOUT + 1) * HZ);
		sprd_sdhc_writeb(host, host->data_timeout_val,
				SPRD_SDHC_REG_8_TIMEOUT);
	}

	host->cmd = cmd;
	if (data) {
		/* set data param */
		WARN_ON((data->blksz * data->blocks > 524288) ||
			(data->blksz > host->mmc->max_blk_size) ||
			(data->blocks > 65535));

		data->bytes_xfered = 0;

		if_has_data = 1;
		if_read = (data->flags & MMC_DATA_READ);
		if_mult = (mmc_op_multi(cmd->opcode) || data->blocks > 1);
		if (if_read && !if_mult)
			flag = SPRD_SDHC_DAT_FILTER_RD_SIGLE;
		else if (if_read && if_mult)
			flag = SPRD_SDHC_DAT_FILTER_RD_MULTI;
		else if (!if_read && !if_mult)
			flag = SPRD_SDHC_DAT_FILTER_WR_SIGLE;
		else
			flag = SPRD_SDHC_DAT_FILTER_WR_MULTI;

		if (!host->auto_cmd_mode)
			flag |= SPRD_SDHC_BIT_INT_ERR_ACMD;

		if_dma = 1;
		auto_cmd = host->auto_cmd_mode;
		sprd_sdhc_set_blk_size(host, data->blksz);

		sg_cnt = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
				    (data->flags & MMC_DATA_READ) ?
				    DMA_FROM_DEVICE : DMA_TO_DEVICE);
		if (1 == sg_cnt) {
			sprd_sdhc_set_dma(host, SPRD_SDHC_BIT_SDMA_MOD);
			if (host->version == SPRD_SDHC_BIT_SPEC_300) {
				sprd_sdhc_set_16_blk_cnt(host, data->blocks);
				sprd_sdhc_writel(host, sg_dma_address(data->sg),
					SPRD_SDHC_REG_32_SYS_ADDR);
			} else if (host->version >= SPRD_SDHC_BIT_SPEC_400) {
				u64 addr;

				sprd_sdhc_set_32_blk_cnt(host, data->blocks);
				addr = ((u64)(sg_dma_address(data->sg))) &
					0xffffffff;
				sprd_sdhc_writel(host, addr,
						SPRD_SDHC_REG_32_SDMA_ADDR);
				addr = ((u64)(sg_dma_address(data->sg)) >> 32) &
					0xffffffff;
				sprd_sdhc_writel(host, addr,
						SPRD_SDHC_REG_32_ADMA2_ADDR_H);
			}
		} else {
			flag |= SPRD_SDHC_BIT_INT_ERR_ADMA;
			sprd_sdhc_set_dma(host, SPRD_SDHC_BIT_32ADMA_MOD);
			if (host->version == SPRD_SDHC_BIT_SPEC_300) {
				sprd_sdhc_set_16_blk_cnt(host, data->blocks);
				sprd_sdhc_writel(host, sg_dma_address(data->sg),
					SPRD_SDHC_REG_32_SYS_ADDR);
			} else if (host->version >= SPRD_SDHC_BIT_SPEC_400) {
				u64 addr;

				sprd_sdhc_set_32_blk_cnt(host, data->blocks);

				addr = ((u64)(sg_dma_address(data->sg))) &
					0xffffffff;
				sprd_sdhc_writel(host, addr,
						SPRD_SDHC_REG_32_SDMA_ADDR);
				addr = ((u64)(sg_dma_address(data->sg)) >> 32) &
					0xffffffff;
				sprd_sdhc_writel(host, addr,
						SPRD_SDHC_REG_32_ADMA2_ADDR_H);
			}
		}
	}

	sprd_sdhc_writel(host, cmd->arg, SPRD_SDHC_REG_32_ARG);
	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_R1B:
		rsp_type = SPRD_SDHC_RSP1B_5B;
		flag |= SPRD_SDHC_CMD_FILTER_R1B;
		break;
	case MMC_RSP_NONE:
		rsp_type = SPRD_SDHC_RSP0;
		flag |= SPRD_SDHC_CMD_FILTER_R0;
		break;
	case MMC_RSP_R2:
		rsp_type = SPRD_SDHC_RSP2;
		flag |= SPRD_SDHC_CMD_FILTER_R2;
		break;
	case MMC_RSP_R4:
		rsp_type = SPRD_SDHC_RSP3_4;
		flag |= SPRD_SDHC_CMD_FILTER_R1_R4_R6_R7;
		break;
	case MMC_RSP_R1:
	case MMC_RSP_R1 & ~MMC_RSP_CRC:
		rsp_type = SPRD_SDHC_RSP1_5_6_7;
		flag |= SPRD_SDHC_CMD_FILTER_R1_R4_R6_R7;
		break;
	default:
		WARN_ON(1);
		break;
	}

	host->int_filter = flag;
	sprd_sdhc_enable_int(host, flag);
	pr_debug("sprd_sdhc %s CMD%d rsp:0x%x intflag:0x%x\n"
	       "if_mult:0x%x if_read:0x%x auto_cmd:0x%x if_dma:0x%x\n",
	       host->device_name, cmd->opcode, mmc_resp_type(cmd),
	       flag, if_mult, if_read, auto_cmd, if_dma);

	sprd_sdhc_set_trans_and_cmd(host, if_mult, if_read, auto_cmd, if_mult,
				  if_dma, cmd->opcode, if_has_data, rsp_type);

}

static int irq_err_handle(struct sprd_sdhc_host *host, u32 intmask)
{
	/* some error happened in command */
	if (SPRD_SDHC_INT_FILTER_ERR_CMD & intmask) {
		if (SPRD_SDHC_BIT_INT_ERR_CMD_TIMEOUT & intmask)
			host->cmd->error = -ETIMEDOUT;
		else
			host->cmd->error = -EILSEQ;
	}

	/* some error happened in data token or command  with R1B */
	if (SPRD_SDHC_INT_FILTER_ERR_DAT & intmask) {
		if (host->cmd->data) {
			/* current error is happened in data token */
			if (SPRD_SDHC_BIT_INT_ERR_DATA_TIMEOUT & intmask)
				host->cmd->data->error = -ETIMEDOUT;
			else
				host->cmd->data->error = -EILSEQ;
		} else {
			/* current error is happend in response with busy */
			if (SPRD_SDHC_BIT_INT_ERR_DATA_TIMEOUT & intmask)
				host->cmd->error = -ETIMEDOUT;
			else
				host->cmd->error = -EILSEQ;
		}
	}

	if (SPRD_SDHC_BIT_INT_ERR_ACMD & intmask) {
		/* Auto cmd12 and cmd23 error is belong to data token error */
		host->cmd->data->error = -EILSEQ;
	}
	if (SPRD_SDHC_BIT_INT_ERR_ADMA & intmask)
		host->cmd->data->error = -EIO;

	if ((host->cmd->opcode != 19) && (host->cmd->opcode != 21)) {
		pr_info("sprd_sdhc %s CMD%d int 0x%x\n",
			host->device_name, host->cmd->opcode, intmask);
		dump_sdio_reg(host);
	}
	sprd_sdhc_disable_all_int(host);

	/* if current error happened in data token, we send cmd12 to stop it */
	if ((host->mrq->cmd == host->cmd) && (host->mrq->stop)) {
		sprd_sdhc_reset(host, SPRD_SDHC_BIT_RST_CMD |
				SPRD_SDHC_BIT_RST_DAT);
		sprd_send_cmd(host, host->mrq->stop);
	} else {
		/* request finish with error, so reset and stop it */
		sprd_sdhc_reset(host, SPRD_SDHC_BIT_RST_CMD |
				SPRD_SDHC_BIT_RST_DAT);
		return IRQ_WAKE_THREAD;
	}

	return IRQ_HANDLED;
}

static int irq_normal_handle(struct sprd_sdhc_host *host, u32 intmask,
			u32 *ret_intmask)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd = host->cmd;
	struct mmc_data *data = host->cmd->data;
	u32 sys_addr;

	/* delete irq that wanted in filter */
	host->int_filter &= ~(SPRD_SDHC_FILTER_NORMAL & intmask);

	if (SPRD_SDHC_BIT_INT_DMA_END & intmask) {
		if (host->version == SPRD_SDHC_BIT_SPEC_300) {
			sys_addr = sprd_sdhc_readl(host,
					SPRD_SDHC_REG_32_SYS_ADDR);
			sprd_sdhc_writel(host, sys_addr,
					SPRD_SDHC_REG_32_SYS_ADDR);
		} else if (host->version >= SPRD_SDHC_BIT_SPEC_400) {
			sys_addr = sprd_sdhc_readl(host,
					SPRD_SDHC_REG_32_SDMA_ADDR);
			sprd_sdhc_writel(host, sys_addr,
					SPRD_SDHC_REG_32_SDMA_ADDR);
			sys_addr = sprd_sdhc_readl(host,
					SPRD_SDHC_REG_32_ADMA2_ADDR_H);
			sprd_sdhc_writel(host, sys_addr,
					SPRD_SDHC_REG_32_ADMA2_ADDR_H);
		}
	}

	if (SPRD_SDHC_BIT_INT_CMD_END & intmask) {
		cmd->error = 0;
		sprd_get_rsp(host);
	}

	if (SPRD_SDHC_BIT_INT_TRAN_END & intmask) {
		if (cmd->data) {
			dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
				(data->flags & MMC_DATA_READ) ?
				DMA_FROM_DEVICE : DMA_TO_DEVICE);
			data->error = 0;
			data->bytes_xfered = data->blksz * data->blocks;
		} else {
			/* R1B also can produce transfer complete interrupt */
			cmd->error = 0;
		}
	}

	if (!(SPRD_SDHC_FILTER_NORMAL & host->int_filter)) {
		/* current cmd finished */
		sprd_sdhc_disable_all_int(host);
		if (mrq->sbc == cmd)
			sprd_send_cmd(host, mrq->cmd);
		else if ((mrq->cmd == host->cmd) && (mrq->stop))
			sprd_send_cmd(host, mrq->stop);
		else {
			/* finish with success and stop the request */
			return IRQ_WAKE_THREAD;
		}
	}

	intmask = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_INT_STATE);
	sprd_sdhc_clear_int(host, intmask);
	intmask &= host->int_filter;
	*ret_intmask = intmask;

	return IRQ_HANDLED;
}

static irqreturn_t sprd_sdhc_irq(int irq, void *param)
{
	u32 intmask;
	struct sprd_sdhc_host *host = (struct sprd_sdhc_host *)param;
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd = host->cmd;
	struct mmc_data *data;
	u32 ret_intmask = 0;

	spin_lock(&host->lock);
	/*
	 * maybe sprd_sdhc_timeout() run in one core and _irq() run in
	 * another core, this will panic if access cmd->data
	 */
	if ((!mrq) || (!cmd)) {
		spin_unlock(&host->lock);
		return IRQ_NONE;
	}
	data = cmd->data;

	intmask = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_INT_STATE);
	if (!intmask) {
		spin_unlock(&host->lock);
		return IRQ_NONE;
	}
	pr_debug("%s(%s) CMD%d, intmask 0x%x, filter = 0x%x\n", __func__,
	       host->device_name, cmd->opcode, intmask, host->int_filter);

	/*
	 * sometimes an undesired interrupt will happen, so we must clear
	 * this unused interrupt.
	 */
	sprd_sdhc_clear_int(host, intmask);
	/* just care about the interrupt that we want */
	intmask &= host->int_filter;

	while (intmask) {
		int ret = 0;

		if (SPRD_SDHC_INT_FILTER_ERR & intmask) {
			ret = irq_err_handle(host, intmask);
			if (IRQ_WAKE_THREAD == ret)
				goto out_to_irq_thread;
			else if (IRQ_HANDLED == ret)
				goto out_to_irq;
		} else {
			ret = irq_normal_handle(host, intmask, &ret_intmask);
			if (IRQ_WAKE_THREAD == ret)
				goto out_to_irq_thread;
			else
				intmask = ret_intmask;
		}
	};

out_to_irq:
	spin_unlock(&host->lock);
	return IRQ_HANDLED;

out_to_irq_thread:
	spin_unlock(&host->lock);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t sprd_sdhc_thread_irq(int irq, void *dev_id)
{
	struct sprd_sdhc_host *host = (struct sprd_sdhc_host *)dev_id;
	unsigned long flags;
	struct mmc_request *mrq = host->mrq;

	spin_lock_irqsave(&host->lock, flags);
	if (!host->mrq) {
		pr_err("%s: host->mrq is null\n", __func__);
		spin_unlock_irqrestore(&host->lock, flags);
		return IRQ_HANDLED;
	}

	del_timer(&host->timer);

	pr_debug("%s(%s) cmd %d data %d\n", __func__,
		 host->device_name, mrq->cmd->error,
		 ((!!mrq->cmd->data) ? mrq->cmd->data->error : 0));

	host->mrq = NULL;
	host->cmd = NULL;
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);

	mmc_request_done(host->mmc, mrq);
	sprd_sdhc_runtime_pm_put(host);

	return IRQ_HANDLED;
}

static void sprd_sdhc_timeout(unsigned long data)
{
	struct sprd_sdhc_host *host = (struct sprd_sdhc_host *)data;
	unsigned long flags;
	struct mmc_request *mrq = NULL;

	spin_lock_irqsave(&host->lock, flags);
	if (host->mrq) {
		pr_info("%s(%s) Timeout waiting for hardware interrupt!\n",
			__func__, host->device_name);
		dump_sdio_reg(host);
		if (host->cmd->data)
			host->cmd->data->error = -ETIMEDOUT;
		else if (host->cmd)
			host->cmd->error = -ETIMEDOUT;
		else
			host->mrq->cmd->error = -ETIMEDOUT;

		sprd_sdhc_disable_all_int(host);
		sprd_sdhc_reset(host,
			SPRD_SDHC_BIT_RST_CMD | SPRD_SDHC_BIT_RST_DAT);

		mrq = host->mrq;
		host->mrq = NULL;
		host->cmd = NULL;
	}
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);

	if (mrq)
		mmc_request_done(host->mmc, mrq);

	sprd_sdhc_runtime_pm_put(host);
}

static void sprd_sdhc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	host->mrq = mrq;
	/* 1 find whether card is still in slot */
	if (!(host->mmc->caps & MMC_CAP_NONREMOVABLE)) {
		if (!mmc_gpio_get_cd(host->mmc)) {
			mrq->cmd->error = -ENOMEDIUM;

			host->mrq = NULL;
			host->cmd = NULL;

			mmiowb();
			spin_unlock_irqrestore(&host->lock, flags);

			mmc_request_done(host->mmc, mrq);
			sprd_sdhc_runtime_pm_put(host);

			return;
		}
		/* else asume sdcard is present */
	}

	/*
	 * in our control we can not use auto cmd12 and auto cmd23 together
	 * so in following program we use auto cmd23 prior to auto cmd12
	 */
	pr_debug("%s(%s) CMD%d request %d %d %d\n", __func__,
		host->device_name, mrq->cmd->opcode,
		!!mrq->sbc, !!mrq->cmd, !!mrq->stop);
	host->auto_cmd_mode = SPRD_SDHC_BIT_ACMD_DIS;
	if (!mrq->sbc && mrq->stop && SPRD_SDHC_FLAG_ENABLE_ACMD12) {
		host->auto_cmd_mode = SPRD_SDHC_BIT_ACMD12;
		mrq->data->stop = NULL;
		mrq->stop = NULL;
	}

	/* 3 send cmd list */
	if ((mrq->sbc) && SPRD_SDHC_FLAG_ENABLE_ACMD23) {
		host->auto_cmd_mode = SPRD_SDHC_BIT_ACMD23;
		mrq->stop = NULL;
		sprd_send_cmd(host, mrq->cmd);
	} else if (mrq->sbc) {
		mrq->stop = NULL;
		sprd_send_cmd(host, mrq->sbc);
	} else {
		sprd_send_cmd(host, mrq->cmd);
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void sprd_signal_voltage_on_off(struct sprd_sdhc_host *host, u32 on_off)
{
	if (IS_ERR(host->mmc->supply.vqmmc)) {
		pr_err("%s(%s) there is no signal voltage!\n",
			 __func__, host->device_name);
		return;
	}

	if (on_off) {
		if (!regulator_is_enabled(host->mmc->supply.vqmmc)) {
			if (regulator_enable(host->mmc->supply.vqmmc))
				pr_info("%s signal voltage enable fail!\n",
					host->device_name);
			else if (regulator_is_enabled(host->mmc->supply.vqmmc))
				pr_info("%s signal voltage enable success!\n",
					host->device_name);
			 else
				pr_info("%s signal voltage enable hw fail!\n",
					host->device_name);
		}
	} else {
		if ((strcmp(host->device_name, "sdio_sd") == 0) &&
			(SPRD_R7 == host->sprd_ip_version)) {
			if (pinctrl_select_state(
				host->pinctrl, host->pins_default))
				pr_err("%s switch vddsdio ms pinctrl failed\n",
					__func__);
			else
				pr_info("%s set vddsdio ms bit as 0\n",
					__func__);
		}

		if (regulator_is_enabled(host->mmc->supply.vqmmc)) {
			if (regulator_disable(host->mmc->supply.vqmmc))
				pr_info("%s signal voltage disable fail\n",
					host->device_name);
			 else if (!regulator_is_enabled(
					host->mmc->supply.vqmmc))
				pr_info("%s signal voltage disable success!\n",
					host->device_name);
			 else
				pr_info("%s signal voltage disable hw fail\n",
					host->device_name);
		}
	}

}

/*
 * 1 This votage is always poweron
 * 2 initial votage is 2.7v~3.6v
 * 3 It can be reconfig to 1.7v~1.95v
 */
static int sprd_sdhc_set_vqmmc(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;
	int err;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	if (IS_ERR(mmc->supply.vqmmc)) {
		/* there are no 1.8v signal votage. */
		err = 0;
		pr_err("%s(%s) There is no signalling voltage\n",
			__func__, host->device_name);
		goto out;
	}

	switch (ios->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		/*
		 * SPRD LDO can't support 3.0v signal voltage for emmc,
		 * so we must return
		 */
		if (strcmp(host->device_name, "sdio_emmc") == 0) {
			err = -EINVAL;
			goto out;
		}
		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		err = regulator_set_voltage(mmc->supply.vqmmc,
					    3000000, 3000000);
		spin_lock_irqsave(&host->lock, flags);
		break;
	case MMC_SIGNAL_VOLTAGE_180:
		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		err = regulator_set_voltage(mmc->supply.vqmmc,
					    1800000, 1800000);
		spin_lock_irqsave(&host->lock, flags);
		if ((strcmp(host->device_name, "sdio_sd") == 0) &&
			(SPRD_R7 == host->sprd_ip_version)) {
			err = pinctrl_select_state(
				host->pinctrl, host->pins_uhs);
			if (err)
				pr_err("switch vddsdio ms pinctrl failed\n");
			else
				pr_info("set vddsdio ms bit as 1 ok\n");

			udelay(300);
			sprd_sdhc_reset(host, SPRD_SDHC_BIT_RST_CMD |
					SPRD_SDHC_BIT_RST_DAT);
		}
		break;
	case MMC_SIGNAL_VOLTAGE_120:
		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		err = regulator_set_voltage(mmc->supply.vqmmc,
					    1100000, 1300000);
		spin_lock_irqsave(&host->lock, flags);
		break;
	default:
		/* No signal voltage switch required */
		pr_err("%s(%s) 0x%x is an unsupportted signal voltage\n",
			__func__, host->device_name, ios->signal_voltage);
		err = EIO;
		break;
	}

	if (likely(!err))
		host->ios.signal_voltage = ios->signal_voltage;
	else
		pr_warn("%s(%s): switching to signalling voltage failed\n",
			__func__, host->device_name);

out:
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);

	sprd_sdhc_runtime_pm_put(host);
	return err;
}

static void sprd_sdhc_enable_dpll(struct sprd_sdhc_host *host)
{
	u32 tmp = 0;

	tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_CFG);
	tmp &= ~(SPRD_SDHC_DLL_EN | SPRD_SDHC_DLL_ALL_CPST_EN);
	sprd_sdhc_writel(host, tmp, SPRD_SDHC_REG_32_DLL_CFG);
	mdelay(1);

	tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_CFG);
	tmp |= SPRD_SDHC_DLL_ALL_CPST_EN | SPRD_SDHC_DLL_SEARCH_MODE |
		SPRD_SDHC_DLL_INIT_COUNT | SPRD_SDHC_DLL_PHA_INTERNAL;
	sprd_sdhc_writel(host, tmp, SPRD_SDHC_REG_32_DLL_CFG);
	mdelay(1);

	tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_CFG);
	tmp |= SPRD_SDHC_DLL_EN;
	sprd_sdhc_writel(host, tmp, SPRD_SDHC_REG_32_DLL_CFG);
	mdelay(1);

	tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_STS0);
	while (((tmp & SPRD_SDHC_DLL_LOCKED) != SPRD_SDHC_DLL_LOCKED) ||
		((tmp & SPRD_SDHC_DLL_ERROR) == SPRD_SDHC_DLL_ERROR)) {
		pr_info("++++++++++ sprd sdhc dpll locked faile !++++++++++!\n");
		pr_info("sprd sdhc dpll register SPRD_SDHC_REG_32_DLL_STS0 : 0x%x\n", tmp);
		tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_CFG);
		pr_info("sprd sdhc dpll register SPRD_SDHC_REG_32_DLL_CFG : 0x%x\n", tmp);
		tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_DLY);
		pr_info("sprd sdhc dpll register SPRD_SDHC_REG_32_DLL_DLY : 0x%x\n", tmp);
		tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_STS1);
		pr_info("sprd sdhc dpll register SPRD_SDHC_REG_32_DLL_STS1 : 0x%x\n", tmp);
		tmp = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_DLL_STS0);
		mdelay(1);
	}

	pr_info("%s(%s): dpll locked done\n", __func__, host->device_name);
}

static void sprd_sdhc_aon_clksource(struct sprd_sdhc_host *host,
	struct mmc_ios *ios)
{

	clk_disable_unprepare(host->clk);
	clk_disable_unprepare(host->sdio_ahb);
	clk_disable_unprepare(host->sdio_ckg);
	if (ios->clock <= 400000)
		clk_set_parent(host->clk, host->clk_source2_r7);
	else
		clk_set_parent(host->clk, host->clk_source);

	clk_set_rate(host->clk,  2 * ios->clock);

	clk_prepare_enable(host->clk);
	clk_prepare_enable(host->sdio_ahb);
	clk_prepare_enable(host->sdio_ckg);

	if (ios->clock > 52000000)
		sprd_sdhc_enable_dpll(host);
}

static void sprd_sdhc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;
	u8 ctrl = 0;

	pr_debug("++++++++++sprd-sdhc %s ios : ++++++++++\n"
	       "sprd-sdhc clock = %d---->%d\n"
	       "sprd-sdhc vdd = %d---->%d\n"
	       "sprd-sdhc bus_mode = %d---->%d\n"
	       "sprd-sdhc chip_select = %d---->%d\n"
	       "sprd-sdhc power_mode = %d---->%d\n"
	       "sprd-sdhc bus_width = %d---->%d\n"
	       "sprd-sdhc timing = %d---->%d\n"
	       "sprd-sdhc signal_voltage = %d---->%d\n"
	       "sprd-sdhc drv_type = %d---->%d\n",
	       host->device_name,
	       host->ios.clock, ios->clock,
	       host->ios.vdd, ios->vdd,
	       host->ios.bus_mode, ios->bus_mode,
	       host->ios.chip_select, ios->chip_select,
	       host->ios.power_mode, ios->power_mode,
	       host->ios.bus_width, ios->bus_width,
	       host->ios.timing, ios->timing,
	       host->ios.signal_voltage, ios->signal_voltage, host->ios.drv_type, ios->drv_type);

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	if (0 == ios->clock) {
		sprd_sdhc_all_clk_off(host);
		host->ios.clock = 0;
	} else if (ios->clock != host->ios.clock) {
		u32 div;

		div = sprd_sdhc_calc_div(host->base_clk, ios->clock);
		sprd_sdhc_sd_clk_off(host);
		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		if (SPRD_R7 == host->sprd_ip_version)
			sprd_sdhc_aon_clksource(host, ios);
		spin_lock_irqsave(&host->lock, flags);
		sprd_sdhc_clk_set_and_on(host, div);
		sprd_sdhc_sd_clk_on(host);
		host->ios.clock = ios->clock;
		host->data_timeout_val = sprd_sdhc_calc_timeout(ios->clock,
						SPRD_SDHC_MAX_TIMEOUT);
		mmc->max_busy_timeout = (1 << 30) / (ios->clock / 1000);
	}

	if (ios->power_mode != host->ios.power_mode) {
		switch (ios->power_mode) {
		case MMC_POWER_OFF:
			mmiowb();
			spin_unlock_irqrestore(&host->lock, flags);
			sprd_signal_voltage_on_off(host, 0);
			if (!IS_ERR(mmc->supply.vmmc))
				mmc_regulator_set_ocr(host->mmc,
						mmc->supply.vmmc, 0);
			spin_lock_irqsave(&host->lock, flags);
			sprd_reset_ios(host);
			host->ios.power_mode = ios->power_mode;
			break;
		case MMC_POWER_ON:
		case MMC_POWER_UP:
			mmiowb();
			spin_unlock_irqrestore(&host->lock, flags);
			if (!IS_ERR(mmc->supply.vmmc))
				mmc_regulator_set_ocr(host->mmc,
					mmc->supply.vmmc, ios->vdd);
			sprd_signal_voltage_on_off(host, 1);
			spin_lock_irqsave(&host->lock, flags);
			host->ios.power_mode = ios->power_mode;
			host->ios.vdd = ios->vdd;
			break;
		}
	}

	/* flash power voltage select */
	if (ios->vdd != host->ios.vdd) {
		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		if (!IS_ERR(mmc->supply.vmmc)) {
			pr_info("%s(%s) 3.0 %d!\n", __func__,
				host->device_name, ios->vdd);
			mmc_regulator_set_ocr(host->mmc,
				mmc->supply.vmmc, ios->vdd);
		}
		spin_lock_irqsave(&host->lock, flags);
		host->ios.vdd = ios->vdd;
	}

	if (ios->bus_width != host->ios.bus_width) {
		sprd_sdhc_set_buswidth(host, ios->bus_width);
		host->ios.bus_width = ios->bus_width;
	}

	if (ios->timing != host->ios.timing) {
		/* 1 first close SD clock */
		sprd_sdhc_sd_clk_off(host);
		/* 2 set timing mode, timing specification used*/
		switch (ios->timing) {
		case MMC_TIMING_MMC_HS:
		case MMC_TIMING_SD_HS:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_SDR25;
			break;
		case MMC_TIMING_UHS_SDR12:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_SDR12;
			break;
		case MMC_TIMING_UHS_SDR25:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_SDR25;
			break;
		case MMC_TIMING_UHS_SDR50:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_SDR50;
			break;
		case MMC_TIMING_UHS_SDR104:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_SDR104;
			break;
		case MMC_TIMING_UHS_DDR50:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_DDR50;
			break;
		case MMC_TIMING_MMC_DDR52:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_DDR50;
			break;
		case MMC_TIMING_MMC_HS200:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_HS200;
			break;
		case MMC_TIMING_MMC_HS400:
			ctrl = SPRD_SDHC_BIT_TIMING_MODE_HS400;
			break;
		default:
			break;
		}
		sprd_sdhc_set_uhs_mode(host, ctrl);

		/* 3 open SD clock */
		sprd_sdhc_sd_clk_on(host);
		host->ios.timing = ios->timing;
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	sprd_sdhc_runtime_pm_put(host);
}

int sprd_sdhc_set_nonremovable(struct mmc_host *host)
{
	host->caps |= MMC_CAP_NONREMOVABLE;
	return 0;
}
EXPORT_SYMBOL_GPL(sprd_sdhc_set_nonremovable);

static int sprd_sdhc_get_cd(struct mmc_host *mmc)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;
	int gpio_cd;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	if (host->mmc->caps & MMC_CAP_NONREMOVABLE) {
		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		sprd_sdhc_runtime_pm_put(host);
		return 1;
	}

	gpio_cd = mmc_gpio_get_cd(host->mmc);
	if (IS_ERR_VALUE(gpio_cd))
		gpio_cd = 1;
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	sprd_sdhc_runtime_pm_put(host);
	return !!gpio_cd;
}

static int sprd_sdhc_card_busy(struct mmc_host *mmc)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;
	u32 present_state;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	/* Check whether DAT[3:0] is 0000 */
	present_state = sprd_sdhc_readl(host, SPRD_SDHC_REG_32_PRES_STATE);

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	sprd_sdhc_runtime_pm_put(host);
	return !(present_state & SPRD_SDHC_DATA_LVL_MASK);
}

static void sprd_sdhc_emmc_hw_reset(struct mmc_host *mmc)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;
	int val;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	val = sprd_sdhc_readb(host, SPRD_SDHC_REG_8_RST);
	val &= ~SPRD_SDHC_BIT_RST_EMMC;
	sprd_sdhc_writeb(host, val, SPRD_SDHC_REG_8_RST);
	udelay(10);

	val |= SPRD_SDHC_BIT_RST_EMMC;
	sprd_sdhc_writeb(host, val, SPRD_SDHC_REG_8_RST);
	udelay(300);

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	sprd_sdhc_runtime_pm_put(host);
}

static int sprd_sdhc_prepare_hs400_tuning(struct mmc_host *mmc,
	struct mmc_ios *ios)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	unsigned long flags;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	if (mmc->mmc_ext_csd_rev >= 8)
		host->flags |= SPRD_HS400_V510_TUNING;
	else
		host->flags |= SPRD_HS400_V500_TUNING;

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	sprd_sdhc_runtime_pm_put(host);

	return 0;
}

static int sprd_sdhc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct sprd_sdhc_host *host = mmc_priv(mmc);
	u32 try_phase = 0;
	u32 count = 0;
	u32 ctrl_dly = 0;
	u32 min_fail = 0;
	u32 max_fail = 255;
	u32 min_okay = 0;
	u32 max_okay = 255;
	u32 err_flag = 0;
	u32 okay_flag = 0;
	u32 first_tuning_err = 0;
	unsigned long flags;
	int err;

	sprd_sdhc_runtime_pm_get(host);
	spin_lock_irqsave(&host->lock, flags);

	sprd_sdhc_reset(host, SPRD_SDHC_BIT_RST_CMD | SPRD_SDHC_BIT_RST_DAT);

	do {
		if (host->flags & SPRD_HS400_V510_TUNING) {
			pr_info("%s(%s): eMMC card version is 5.1, no tuning\n",
				__func__, host->device_name);
			break;
		}

		if (host->flags & SPRD_HS400_V500_TUNING)
			sprd_sdhc_writeb(host, try_phase,
					SPRD_SDHC_REG_8_CMDRD_DLY);
		else
			sprd_sdhc_writeb(host, try_phase,
					SPRD_SDHC_REG_8_POSRD_DLY);

		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		err = mmc_send_tuning(mmc, opcode, NULL);
		if (err)
			pr_info("%s(%s):find tuning phase %d fail\n",
				__func__, host->device_name, try_phase);

		spin_lock_irqsave(&host->lock, flags);
		if (try_phase == 0 && err)
			first_tuning_err = 1;

		if (first_tuning_err) {
			if (!err && !okay_flag) {
				min_okay = try_phase;
				okay_flag = 1;
			} else if (!err && okay_flag)
				count++;
			else if (err && okay_flag) {
				max_okay = try_phase - 1;
				break;
			}
		} else {
			if (err && !err_flag) {
				min_fail = try_phase;
				err_flag = 1;
			} else if (!err && err_flag) {
				max_fail = try_phase - 1;
				break;
			} else if (!err && !err_flag)
				count++;
		}
	} while (++try_phase <= 255);

	if (count > 0) {
		if (first_tuning_err) {
			try_phase = (min_okay + max_okay) / 2;
			pr_info("%s(%s): min_okay:%d, max_okay:%d\n", __func__,
					host->device_name, min_okay, max_okay);
		} else {
			try_phase = ((min_fail + max_fail + 256) / 2) % 256;
			pr_info("%s(%s): min_fail:%d, max_fail:%d\n", __func__,
					host->device_name, min_fail, max_fail);
		}
	} else if (host->flags & SPRD_HS400_V510_TUNING)
		try_phase = 0x7f;
	else {
		pr_err("%s(%s): tuning fail\n", __func__, host->device_name);
		err = -EIO;
		goto out;
	}

	if ((host->flags & SPRD_HS400_V500_TUNING) ||
		(host->flags & SPRD_HS400_V510_TUNING)) {
		host->cmd_delay = try_phase;
		ctrl_dly = host->write_delay |
			(host->cmd_delay << 8) |
			(host->read_pos_delay << 16) |
			(host->read_neg_delay << 24);
	} else
		ctrl_dly = ((try_phase & 0xff) << 8) |
			((try_phase & 0xff) << 16);

	pr_info("%s(%s): tuning value: 0x%x\n",
		__func__, host->device_name, ctrl_dly);
	sprd_sdhc_writel(host, ctrl_dly, SPRD_SDHC_REG_32_DLL_DLY);
	err = 0;

	host->flags &= ~SPRD_HS400_V500_TUNING;
	host->flags &= ~SPRD_HS400_V510_TUNING;

out:
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
	sprd_sdhc_runtime_pm_put(host);

	return err;
}

static const struct mmc_host_ops sprd_sdhc_ops = {
	.request = sprd_sdhc_request,
	.set_ios = sprd_sdhc_set_ios,
	.get_cd = sprd_sdhc_get_cd,
	.start_signal_voltage_switch = sprd_sdhc_set_vqmmc,
	.card_busy = sprd_sdhc_card_busy,
	.hw_reset = sprd_sdhc_emmc_hw_reset,
	.prepare_hs400_tuning = sprd_sdhc_prepare_hs400_tuning,
	.execute_tuning = sprd_sdhc_execute_tuning,
};

static int sprd_get_dt_resource(struct platform_device *pdev,
			       struct sprd_sdhc_host *host)
{
	struct device_node *np = pdev->dev.of_node;
	u32 sprd_sdhc_delay[3];
	int ret = 0;

	host->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!host->res)
		return -ENOENT;

	host->ioaddr = devm_ioremap_resource(&pdev->dev, host->res);
	if (IS_ERR(host->ioaddr)) {
		ret = PTR_ERR(host->ioaddr);
		dev_err(&pdev->dev, "can not map iomem: %d\n", ret);
		goto err;
	}

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		ret = host->irq;
		goto err;
	}

	host->clk = of_clk_get_by_name(np, "sdio");
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		dev_err(&pdev->dev,
			"can't get the clock dts config: sdio\n");
		goto err;
	}

	host->clk_source = of_clk_get_by_name(np, "source");
	if (IS_ERR(host->clk_source)) {
		ret = PTR_ERR(host->clk_source);
		dev_err(&pdev->dev,
			"can't get the clock dts config: source\n");
		goto err;
	}

	if (SPRD_R7 == host->sprd_ip_version) {
		host->clk_source2_r7 = of_clk_get_by_name(np, "source2");
		if (IS_ERR(host->clk_source2_r7)) {
			ret = PTR_ERR(host->clk_source2_r7);
			dev_err(&pdev->dev,
				"can't get the clock dts config: source2\n");
			goto err;
		}
	}

	clk_set_parent(host->clk, host->clk_source);
	host->base_clk = clk_get_rate(host->clk_source);
	host->sdio_ahb = of_clk_get_by_name(np, "enable");

	if (IS_ERR(host->sdio_ahb)) {
		ret = PTR_ERR(host->sdio_ahb);
		dev_err(&pdev->dev,
			"sdio can't get the clock dts config: enable\n");
		goto err;
	}

	host->sdio_ckg = of_clk_get_by_name(np, "ckg_eb");
	if (IS_ERR(host->sdio_ckg)) {
		dev_err(&pdev->dev,
			"sdio can't get the clock dts config: ckg_eb\n");
	}

	ret = of_property_read_string(np, "sprd,name", &host->device_name);
	if (ret) {
		dev_err(&pdev->dev,
			"can not read the property of sprd name\n");
		goto err;
	}
	host->detect_gpio = of_get_named_gpio(np, "cd-gpios", 0);
	if (!gpio_is_valid(host->detect_gpio))
		host->detect_gpio = -1;

	ret = of_property_read_u32_array(np, "sprd,delay", sprd_sdhc_delay, 3);
	if (!ret) {
		host->write_delay = sprd_sdhc_delay[0];
		host->read_pos_delay = sprd_sdhc_delay[1];
		host->read_neg_delay = sprd_sdhc_delay[2];
	} else
		dev_err(&pdev->dev,
			"can not read the property of sprd delay\n");

	if ((strcmp(host->device_name, "sdio_sd") == 0) &&
			(SPRD_R7 == host->sprd_ip_version)) {
		host->pinctrl = devm_pinctrl_get(&pdev->dev);
		 if (IS_ERR(host->pinctrl)) {
			dev_err(&pdev->dev, "can not find pinctrl\n");
			goto out;
		 }

		host->pins_uhs =
			pinctrl_lookup_state(host->pinctrl, "sd0_ms_1");
		if (IS_ERR(host->pins_uhs)) {
			dev_err(&pdev->dev, "can not find pinctrl uhs\n");
			goto out;
		}

		host->pins_default =
			pinctrl_lookup_state(host->pinctrl, "sd0_ms_0");
		if (IS_ERR(host->pins_default)) {
			dev_err(&host->pdev->dev, "can not find pins_default\n");
			goto out;
		}
	}

out:
	return 0;

err:
	dev_err(&pdev->dev, "sprd_sdhc get basic resource fail\n");
	return ret;
}

static int sprd_get_ext_resource(struct sprd_sdhc_host *host)
{
	int err;
	struct mmc_host *mmc = host->mmc;

	host->dma_mask = DMA_BIT_MASK(64);
	host->data_timeout_val = 0;


	mmc_regulator_get_supply(mmc);
	host->mmc = mmc;

	err = devm_request_threaded_irq(&host->pdev->dev, host->irq,
			sprd_sdhc_irq, sprd_sdhc_thread_irq, 0,
			mmc_hostname(host->mmc), host);

	if (err) {
		pr_err("%s: can not request irq\n", host->device_name);
		return err;
	}

	/* 4 init timer */
	setup_timer(&host->timer, sprd_sdhc_timeout, (unsigned long)host);

	return 0;


}

static void sprd_set_mmc_struct(struct sprd_sdhc_host *host,
				struct mmc_host *mmc)
{
	struct device_node *np = host->pdev->dev.of_node;

	mmc = host->mmc;
	mmc_dev(host->mmc)->dma_mask = &host->dma_mask;
	mmc->ops = &sprd_sdhc_ops;
	mmc->f_max = host->base_clk;
	mmc->f_min = 400000;

	mmc->caps = MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED |
		 MMC_CAP_ERASE | MMC_CAP_CMD23;

	mmc_of_parse(mmc);
	mmc_of_parse_voltage(np, &host->ocr_mask);
    /*
     * if there is not regulator device in some of the SPRD platform, the
     * regulator driver will not be used and the REGULATOR will not be configed
     * in defconfig, in this case the mmc->supply.vmmc will be NULL.
     */

	mmc->ocr_avail = 0x40000;
	mmc->ocr_avail_sdio = mmc->ocr_avail;
	mmc->ocr_avail_sd = mmc->ocr_avail;
	mmc->ocr_avail_mmc = mmc->ocr_avail;

	mmc->max_current_330 = SPRD_SDHC_MAX_CUR;
	mmc->max_current_300 = SPRD_SDHC_MAX_CUR;
	mmc->max_current_180 = SPRD_SDHC_MAX_CUR;

	mmc->max_segs = 1;
	mmc->max_req_size = 524288;	/* 512k */
	mmc->max_seg_size = mmc->max_req_size;

	mmc->max_blk_size = 512;
	mmc->max_blk_count = 65535;

	pr_info("%s(%s): ocr avail = 0x%x, base clock = %u\n"
		"pm_caps = 0x%x, caps: 0x%x, caps2: 0x%x\n",
		__func__, host->device_name, mmc->ocr_avail,
		host->base_clk, mmc->pm_caps, mmc->caps, mmc->caps2);
}

#ifdef CONFIG_PM_SLEEP
static int sprd_sdhc_suspend(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct sprd_sdhc_host *host = platform_get_drvdata(pdev);

	sprd_sdhc_runtime_pm_get(host);
	disable_irq(host->irq);
	clk_disable_unprepare(host->clk);
	clk_disable_unprepare(host->sdio_ahb);
	clk_disable_unprepare(host->sdio_ckg);

	return 0;
}

static int sprd_sdhc_resume(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct sprd_sdhc_host *host = platform_get_drvdata(pdev);
	struct mmc_ios ios;

	clk_prepare_enable(host->sdio_ckg);
	clk_prepare_enable(host->sdio_ahb);
	clk_prepare_enable(host->clk);
	enable_irq(host->irq);

	ios = host->mmc->ios;
	sprd_reset_ios(host);
	host->mmc->ops->set_ios(host->mmc, &ios);

	sprd_sdhc_runtime_pm_put(host);

	return 0;
}
#endif

#ifdef CONFIG_PM
static int sprd_sdhc_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct sprd_sdhc_host *host = platform_get_drvdata(pdev);

	synchronize_hardirq(host->irq);
	clk_disable_unprepare(host->clk);
	clk_disable_unprepare(host->sdio_ahb);
	clk_disable_unprepare(host->sdio_ckg);

	return 0;
}

static int sprd_sdhc_runtime_resume(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct sprd_sdhc_host *host = platform_get_drvdata(pdev);
	unsigned long flags;

	clk_prepare_enable(host->sdio_ckg);
	clk_prepare_enable(host->sdio_ahb);
	clk_prepare_enable(host->clk);
	spin_lock_irqsave(&host->lock, flags);
	if (host->ios.clock) {
		sprd_sdhc_sd_clk_off(host);
		sprd_sdhc_clk_set_and_on(host,
			sprd_sdhc_calc_div(host->base_clk, host->ios.clock));
		sprd_sdhc_sd_clk_on(host);
	}
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);

	return 0;
}
#endif

static const struct dev_pm_ops sprd_sdhc_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(sprd_sdhc_suspend, sprd_sdhc_resume)
	SET_RUNTIME_PM_OPS(sprd_sdhc_runtime_suspend,
		sprd_sdhc_runtime_resume, NULL)
};

static const struct of_device_id sprd_sdhc_of_match[] = {
	{.compatible = "sprd,sdhc-r5", .data = (void *)SPRD_R5,},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sprd_sdhc_of_match);

static int sprd_sdhc_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct sprd_sdhc_host *host;
	const struct of_device_id *of_id;
	int ret;

	host = devm_kzalloc(&pdev->dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	of_id = of_match_node(sprd_sdhc_of_match,
				   pdev->dev.of_node);
	if (!of_id) {
		pr_err("Get the sdhc of device id failed!\n");
		return -ENODEV;
	}
	/* globe resource */
	mmc = mmc_alloc_host(sizeof(struct sprd_sdhc_host), &pdev->dev);
	if (!mmc) {
		dev_err(&pdev->dev, "no memory for mmc host\n");
		return -ENOMEM;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pdev = pdev;
	spin_lock_init(&host->lock);
	platform_set_drvdata(pdev, host);

	host->sprd_ip_version = (enum sprd_sdhci_ip_version)of_id->data;
	dev_info(&pdev->dev, "host->sprd_ip_version:%d\n",
		host->sprd_ip_version);


	/* get basic resource from device tree */
	ret = sprd_get_dt_resource(pdev, host);
	if (ret) {
		dev_err(&pdev->dev, "fail to get basic resource: %d\n", ret);
		goto err_free_host;
	}

	ret = sprd_get_ext_resource(host);
	if (ret) {
		dev_err(&pdev->dev, "fail to get external resource: %d\n", ret);
		goto err_free_host;
	}

	sprd_set_mmc_struct(host, mmc);

	clk_prepare_enable(host->clk);
	clk_prepare_enable(host->sdio_ahb);
	clk_prepare_enable(host->sdio_ckg);
	sprd_reset_ios(host);

	host->version = readw_relaxed(host->ioaddr + SPRD_SDHC_REG_16_HOST_VER);
	if (host->version == SPRD_SDHC_BIT_SPEC_300)
		sprd_sdhc_set_64bit_addr(host, 0);
	else if (host->version >= SPRD_SDHC_BIT_SPEC_400)
		sprd_sdhc_set_64bit_addr(host, 1);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 100);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_suspend_ignore_children(&pdev->dev, 1);

	mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;
	/* add host */
	ret = mmc_add_host(mmc);
	if (ret) {
		dev_err(&pdev->dev, "failed to add mmc host: %d\n", ret);
		goto err_free_host;
	}

	if (-1 != host->detect_gpio) {
		mmc->caps &= ~MMC_CAP_NONREMOVABLE;
		mmc_gpio_request_cd(mmc, host->detect_gpio, 0);
	}

	dev_info(&pdev->dev,
		"%s[%s] host controller, irq %d\n",
		host->device_name, mmc_hostname(mmc), host->irq);

	return 0;

err_free_host:
	mmc_free_host(mmc);
	return ret;
}

static int sprd_sdhc_remove(struct platform_device *pdev)
{
	struct sprd_sdhc_host *host = platform_get_drvdata(pdev);
	struct mmc_host *mmc = host->mmc;

	mmc_remove_host(mmc);
	clk_disable_unprepare(host->clk);
	clk_disable_unprepare(host->sdio_ahb);
	mmc_free_host(mmc);

	return 0;
}

static struct platform_driver sprd_sdhc_driver = {
	.probe = sprd_sdhc_probe,
	.remove = sprd_sdhc_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = DRIVER_NAME,
		   .of_match_table = of_match_ptr(sprd_sdhc_of_match),
		   .pm = &sprd_sdhc_pmops,
	},
};

module_platform_driver(sprd_sdhc_driver);

MODULE_DESCRIPTION("Spreadtrum sdio host controller driver");
MODULE_LICENSE("GPL");
