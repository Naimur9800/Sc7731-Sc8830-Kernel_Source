/*
 * linux/drivers/mmc/host/sprd-sdhc.h - Secure Digital Host Controller
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

#ifndef __SPRD_SDHC_H_
#define __SPRD_SDHC_H_

#include <linux/clk.h>
#include <linux/compiler.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

/**********************************************************\
 *
 * Controller block structure
 *
\**********************************************************/
enum sprd_sdhci_ip_version {
	SPRD_R5,
	SPRD_R7,
	SPRD_R8,
};

struct sprd_sdhc_host {
	/* --globe resource--- */
	spinlock_t lock;
	struct mmc_host *mmc;

	/*--basic resource-- */
	struct resource *res;
	void __iomem *ioaddr;
	int irq;
	const char *device_name;
	struct platform_device *pdev;

	enum sprd_sdhci_ip_version sprd_ip_version;
	int detect_gpio;
	u32 ocr_avail;
	u32 ocr_mask;
	u32 base_clk;
	u32 write_delay;
	u32 cmd_delay;
	u32 read_pos_delay;
	u32 read_neg_delay;
	u32 version;

	/* --extern resource getted by base resource-- */
	uint64_t dma_mask;
	u8 data_timeout_val;
	struct clk *clk;
	struct clk *sdio_ahb;
	struct clk *sdio_ckg;
	struct clk *clk_source2_r7;
	struct clk  *clk_source;
	struct tasklet_struct finish_tasklet;
	struct timer_list timer;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_uhs;
	struct pinctrl_state *pins_default;

	/* --runtime param-- */
	u32 int_filter;
	struct mmc_ios ios;
	struct mmc_request *mrq;	/* Current request */
	struct mmc_command *cmd;	/* Current command */
	u16 auto_cmd_mode;

	u32 flags;
/* Tuning for HS400 support emmc5.0 */
#define SPRD_HS400_V500_TUNING	(1<<0)
/* Tuning for HS400 support emmc5.1 */
#define SPRD_HS400_V510_TUNING	(1<<1)
};

/* Controller flag */
#define SPRD_SDHC_FLAG_ENABLE_ACMD12	0
#define SPRD_SDHC_FLAG_ENABLE_ACMD23	0
#define SPRD_SDHC_FLAG_USE_ADMA		1
#define SPRD_SDHC_MAX_TIMEOUT		0x3

/* Controller registers */
static inline void sprd_sdhc_writeb(struct sprd_sdhc_host *host, u8 val,
				  int reg)
{
	writeb_relaxed(val, host->ioaddr + reg);
}

static inline void sprd_sdhc_writew(struct sprd_sdhc_host *host, u16 val,
				  int reg)
{
	writew_relaxed(val, host->ioaddr + reg);
}

static inline void sprd_sdhc_writel(struct sprd_sdhc_host *host, u32 val,
				  int reg)
{
	writel_relaxed(val, host->ioaddr + reg);
}

static inline u8 sprd_sdhc_readb(struct sprd_sdhc_host *host, int reg)
{
	return readb_relaxed(host->ioaddr + reg);
}

static inline u16 sprd_sdhc_readw(struct sprd_sdhc_host *host, int reg)
{
	return readw_relaxed(host->ioaddr + reg);
}

static inline u32 sprd_sdhc_readl(struct sprd_sdhc_host *host, int reg)
{
	return readl_relaxed(host->ioaddr + reg);
}

#define SPRD_SDHC_REG_32_SYS_ADDR	0x00
/* used in cmd23 with ADMA in sdio 3.0 */
#define SPRD_SDHC_REG_32_BLK_CNT	0x00
#define SPRD_SDHC_REG_16_BLK_CNT	0x06

static inline void sprd_sdhc_set_16_blk_cnt(struct sprd_sdhc_host *host,
					  u32 blk_cnt)
{
	writew_relaxed((blk_cnt & 0xFFFF),
		host->ioaddr + SPRD_SDHC_REG_16_BLK_CNT);
}

static inline void sprd_sdhc_set_32_blk_cnt(struct sprd_sdhc_host *host,
					  u32 blk_cnt)
{
	writel_relaxed((blk_cnt & 0xFFFFFFFF),
		host->ioaddr + SPRD_SDHC_REG_32_BLK_CNT);
}

#define SPRD_SDHC_REG_16_BLK_SIZE	0x04

static inline void sprd_sdhc_set_blk_size(struct sprd_sdhc_host *host,
					u32 blk_size)
{
	writew_relaxed((blk_size & 0xFFF) | 0x7000,
		host->ioaddr + SPRD_SDHC_REG_16_BLK_SIZE);
}

#define SPRD_SDHC_REG_32_ARG	0x08
#define SPRD_SDHC_16_TR_MODE	0x0C
#define SPRD_SDHC_BIT_ACMD_DIS	0x00
#define SPRD_SDHC_BIT_ACMD12	0x01
#define SPRD_SDHC_BIT_ACMD23	0x02

static inline void sprd_sdhc_set_trans_mode(struct sprd_sdhc_host *host,
					  u16 if_mult, u16 if_read,
					  u16 auto_cmd,
					  u16 if_blk_cnt, u16 if_dma)
{
	writew_relaxed((((if_mult ? 1 : 0) << 5) |
			((if_read ? 1 : 0) << 4) |
			(auto_cmd << 2) |
			((if_blk_cnt ? 1 : 0) << 1) |
			((if_dma ? 1 : 0) << 0)),
			host->ioaddr + SPRD_SDHC_16_TR_MODE);
}

#define SPRD_SDHC_REG_16_CMD		0x0E
#define SPRD_SDHC_BIT_CMD_INDEX_CHK	0x0010
#define SPRD_SDHC_BIT_CMD_CRC_CHK	0x0008
#define SPRD_SDHC_BIT_CMD_RSP_NONE	0x0000
#define SPRD_SDHC_BIT_CMD_RSP_136	0x0001
#define SPRD_SDHC_BIT_CMD_RSP_48	0x0002
#define SPRD_SDHC_BIT_CMD_RSP_48_BUSY	0x0003
#define SPRD_SDHC_RSP0			0
#define SPRD_SDHC_RSP1_5_6_7 \
	(SPRD_SDHC_BIT_CMD_INDEX_CHK | SPRD_SDHC_BIT_CMD_CRC_CHK | \
	SPRD_SDHC_BIT_CMD_RSP_48)
#define SPRD_SDHC_RSP2 \
	(SPRD_SDHC_BIT_CMD_CRC_CHK | SPRD_SDHC_BIT_CMD_RSP_136)
#define SPRD_SDHC_RSP3_4 \
	SPRD_SDHC_BIT_CMD_RSP_48
#define SPRD_SDHC_RSP1B_5B \
	(SPRD_SDHC_BIT_CMD_INDEX_CHK | SPRD_SDHC_BIT_CMD_CRC_CHK | \
	SPRD_SDHC_BIT_CMD_RSP_48_BUSY)

static inline void sprd_sdhc_set_cmd(struct sprd_sdhc_host *host, u16 cmd,
				   int if_has_data, u16 rsp_type)
{
	writew_relaxed(((cmd << 8) |
			((if_has_data ? 1 : 0) << 5) |
			rsp_type), host->ioaddr + SPRD_SDHC_REG_16_CMD);
}

#define SPRD_SDHC_32_TR_MODE_AND_CMD	0x0C

static inline void sprd_sdhc_set_trans_and_cmd(struct sprd_sdhc_host *host,
					     int if_mult, int if_read,
					     u16 auto_cmd, int if_blk_cnt,
					     int if_dma, u32 cmd,
					     int if_has_data, u32 rsp_type)
{
	writel_relaxed((((if_mult ? 1 : 0) << 5) |
			((if_read ? 1 : 0) << 4) |
			(((u32) auto_cmd) << 2) |
			((if_blk_cnt ? 1 : 0) << 1) |
			((if_dma ? 1 : 0) << 0) |
			(((u32) cmd) << 24) |
			((if_has_data ? 1 : 0) << 21) |
			(rsp_type << 16)),
			host->ioaddr + SPRD_SDHC_32_TR_MODE_AND_CMD);
}

#define SPRD_SDHC_REG_32_RESP		0x10
#define SPRD_SDHC_REG_32_PRES_STATE	0x24
#define  SPRD_SDHC_DATA_LVL_MASK	0x00F00000

#define SPRD_SDHC_REG_8_HOST_CTRL	0x28
#define SPRD_SDHC_BIT_8_BIT_MODE	0x20
#define SPRD_SDHC_BIT_4_BIT_MODE	0x02
#define SPRD_SDHC_BIT_1_BIT_MODE	0x00
#define SPRD_SDHC_BIT_SDMA_MOD		0x00
#define SPRD_SDHC_BIT_32ADMA_MOD	0x10
#define SPRD_SDHC_BIT_64ADMA_MOD	0x18
#define SPRD_SDHC_BIT_HISPD_MOD		0x04

static inline void sprd_sdhc_set_buswidth(struct sprd_sdhc_host *host,
					u32 buswidth)
{
	u8 ctrl = 0;

	ctrl = readb_relaxed(host->ioaddr + SPRD_SDHC_REG_8_HOST_CTRL);
	ctrl &= (~(SPRD_SDHC_BIT_8_BIT_MODE | SPRD_SDHC_BIT_4_BIT_MODE |
		SPRD_SDHC_BIT_1_BIT_MODE));
	switch (buswidth) {
	case MMC_BUS_WIDTH_1:
		ctrl |= SPRD_SDHC_BIT_1_BIT_MODE;
		break;
	case MMC_BUS_WIDTH_4:
		ctrl |= SPRD_SDHC_BIT_4_BIT_MODE;
		break;
	case MMC_BUS_WIDTH_8:
		ctrl |= SPRD_SDHC_BIT_8_BIT_MODE;
		break;
	default:
		WARN_ON(1);
		break;
	}
	writeb_relaxed(ctrl, host->ioaddr + SPRD_SDHC_REG_8_HOST_CTRL);
}

static inline void sprd_sdhc_set_dma(struct sprd_sdhc_host *host, u8 dma_mode)
{
	u8 ctrl = 0;

	ctrl = readb_relaxed(host->ioaddr + SPRD_SDHC_REG_8_HOST_CTRL);
	ctrl &= (~(SPRD_SDHC_BIT_SDMA_MOD | SPRD_SDHC_BIT_32ADMA_MOD |
		SPRD_SDHC_BIT_64ADMA_MOD));
	ctrl |= dma_mode;
	writeb_relaxed(ctrl, host->ioaddr + SPRD_SDHC_REG_8_HOST_CTRL);
}

static inline void sprd_sdhc_enable_hispd(struct sprd_sdhc_host *host)
{
	u8 ctrl = 0;

	ctrl = readb_relaxed(host->ioaddr + SPRD_SDHC_REG_8_HOST_CTRL);
	ctrl |= SPRD_SDHC_BIT_HISPD_MOD;
	writeb_relaxed(ctrl, host->ioaddr + SPRD_SDHC_REG_8_HOST_CTRL);
}

#define SPRD_SDHC_8_PWR_CTRL		0x29    /* not used */
#define SPRD_SDHC_8_BLK_GAP		0x2A	/* not used */
#define SPRD_SDHC_8_WACKUP_CTRL		0x2B	/* not used */

#define SPRD_SDHC_REG_16_CLK_CTRL	0x2C
#define SPRD_SDHC_BIT_IN_CLK_EN		0x0001
#define SPRD_SDHC_BIT_IN_CLK_STABLE	0x0002
#define SPRD_SDHC_BIT_SD_CLK_IN_EN	0x0004
#define SPRD_SDHC_CLK_MAX_DIV		2046

static inline void sprd_sdhc_all_clk_off(struct sprd_sdhc_host *host)
{
	writew_relaxed(0, host->ioaddr + SPRD_SDHC_REG_16_CLK_CTRL);
}

static inline void sprd_sdhc_sd_clk_off(struct sprd_sdhc_host *host)
{
	u16 ctrl = 0;

	ctrl = readw_relaxed(host->ioaddr + SPRD_SDHC_REG_16_CLK_CTRL);
	ctrl &= (~SPRD_SDHC_BIT_SD_CLK_IN_EN);
	writew_relaxed(ctrl, host->ioaddr + SPRD_SDHC_REG_16_CLK_CTRL);
}

static inline void sprd_sdhc_sd_clk_on(struct sprd_sdhc_host *host)
{
	u16 ctrl = 0;

	ctrl = readw_relaxed(host->ioaddr + SPRD_SDHC_REG_16_CLK_CTRL);
	ctrl |= SPRD_SDHC_BIT_SD_CLK_IN_EN;
	writew_relaxed(ctrl, host->ioaddr + SPRD_SDHC_REG_16_CLK_CTRL);
}

static inline u32 sprd_sdhc_calc_div(u32 base_clk, u32 clk)
{
	u32 div;

	if (base_clk <= clk)
		return 0;

	div = (u32) (base_clk / clk);
	div = (div >> 1);
	if (div)
		div--;
	if ((base_clk / ((div + 1) << 1)) > clk)
		div++;
	if (SPRD_SDHC_CLK_MAX_DIV < div)
		div = SPRD_SDHC_CLK_MAX_DIV;

	return div;
}

static inline void sprd_sdhc_clk_set_and_on(struct sprd_sdhc_host *host,
					u32 div)
{
	u16 ctrl = 0;
	unsigned long timeout;

	writew_relaxed(0, host->ioaddr + SPRD_SDHC_REG_16_CLK_CTRL);
	ctrl |= (u16) (((div & 0x300) >> 2) | ((div & 0xFF) << 8));
	ctrl |= SPRD_SDHC_BIT_IN_CLK_EN;
	writew_relaxed(ctrl, host->ioaddr + SPRD_SDHC_REG_16_CLK_CTRL);

	/* wait max 20 ms*/
	timeout = 100;
	while (!(SPRD_SDHC_BIT_IN_CLK_STABLE &
		readw_relaxed(host->ioaddr + SPRD_SDHC_REG_16_CLK_CTRL))) {
		if (timeout == 0) {
			pr_err("must check! %s clock set and on fail\n",
				host->device_name);
			return;
		}

		timeout--;
		mdelay(1);
	}
}

#define SPRD_SDHC_REG_8_TIMEOUT		0x2E
#define SPRD_SDHC_DATA_TIMEOUT_MAX_VAL		0xe

static inline u8 sprd_sdhc_calc_timeout(unsigned int clock,
					   u8 timeout_value)
{
	unsigned target_timeout, current_timeout;
	u8 count;

	count = 0;
	current_timeout = 1 << 16;
	target_timeout = timeout_value * clock;

	while (target_timeout > current_timeout) {
		count++;
		current_timeout <<= 1;
	}
	count--;
	if (count >= 0xF)
		count = 0xE;
	return count;
}

#define SPRD_SDHC_REG_8_RST	0x2F
#define SPRD_SDHC_BIT_RST_ALL	0x01
#define SPRD_SDHC_BIT_RST_CMD	0x02
#define SPRD_SDHC_BIT_RST_DAT	0x04
#define SPRD_SDHC_BIT_RST_EMMC	0x08	/* spredtrum define it byself */

static inline void sprd_sdhc_reset(struct sprd_sdhc_host *host, u8 mask)
{
	unsigned long timeout;
	u8 tmp;

	tmp = readb_relaxed(host->ioaddr + SPRD_SDHC_REG_8_RST);
	tmp |= mask;
	writeb_relaxed(tmp, host->ioaddr + SPRD_SDHC_REG_8_RST);

	/* wait max 100 ms*/
	timeout = 100;
	while (readb_relaxed(host->ioaddr + SPRD_SDHC_REG_8_RST) & mask) {
		if (timeout == 0) {
			pr_err("must check! reset %s fail\n",
				host->device_name);
			return;
		}

		timeout--;
		mdelay(1);
	}
}

#define SPRD_SDHC_REG_32_INT_STATE		0x30
#define SPRD_SDHC_REG_32_INT_STATE_EN		0x34
#define SPRD_SDHC_REG_32_INT_SIG_EN		0x38
#define SPRD_SDHC_BIT_INT_CMD_END		0x00000001
#define SPRD_SDHC_BIT_INT_TRAN_END		0x00000002
#define SPRD_SDHC_BIT_INT_DMA_END		0x00000008
#define SPRD_SDHC_BIT_INT_WR_RDY		0x00000010	/* not used */
#define SPRD_SDHC_BIT_INT_RD_RDY		0x00000020	/* not used */
#define SPRD_SDHC_BIT_INT_ERR			0x00008000
#define SPRD_SDHC_BIT_INT_ERR_CMD_TIMEOUT	0x00010000
#define SPRD_SDHC_BIT_INT_ERR_CMD_CRC		0x00020000
#define SPRD_SDHC_BIT_INT_ERR_CMD_END		0x00040000
#define SPRD_SDHC_BIT_INT_ERR_CMD_INDEX		0x00080000
#define SPRD_SDHC_BIT_INT_ERR_DATA_TIMEOUT	0x00100000
#define SPRD_SDHC_BIT_INT_ERR_DATA_CRC		0x00200000
#define SPRD_SDHC_BIT_INT_ERR_DATA_END		0x00400000
#define SPRD_SDHC_BIT_INT_ERR_CUR_LIMIT		0x00800000
#define SPRD_SDHC_BIT_INT_ERR_ACMD		0x01000000
#define SPRD_SDHC_BIT_INT_ERR_ADMA		0x02000000

/* used in irq */
#define SPRD_SDHC_INT_FILTER_ERR_CMD \
	(SPRD_SDHC_BIT_INT_ERR_CMD_TIMEOUT | SPRD_SDHC_BIT_INT_ERR_CMD_CRC | \
	SPRD_SDHC_BIT_INT_ERR_CMD_END | SPRD_SDHC_BIT_INT_ERR_CMD_INDEX)
#define SPRD_SDHC_INT_FILTER_ERR_DAT \
	(SPRD_SDHC_BIT_INT_ERR_DATA_TIMEOUT | SPRD_SDHC_BIT_INT_ERR_DATA_CRC | \
	SPRD_SDHC_BIT_INT_ERR_DATA_END)
#define SPRD_SDHC_INT_FILTER_ERR \
	(SPRD_SDHC_BIT_INT_ERR | SPRD_SDHC_INT_FILTER_ERR_CMD | \
	SPRD_SDHC_INT_FILTER_ERR_DAT | SPRD_SDHC_BIT_INT_ERR_ACMD | \
	SPRD_SDHC_BIT_INT_ERR_ADMA)
#define SPRD_SDHC_FILTER_NORMAL \
	(SPRD_SDHC_BIT_INT_CMD_END | SPRD_SDHC_BIT_INT_TRAN_END)

/* used for setting */
#define SPRD_SDHC_DAT_FILTER_RD_SIGLE \
	(SPRD_SDHC_BIT_INT_TRAN_END | SPRD_SDHC_BIT_INT_DMA_END | \
	SPRD_SDHC_BIT_INT_ERR | SPRD_SDHC_BIT_INT_ERR_DATA_TIMEOUT | \
	SPRD_SDHC_BIT_INT_ERR_DATA_CRC | SPRD_SDHC_BIT_INT_ERR_DATA_END)
#define SPRD_SDHC_DAT_FILTER_RD_MULTI \
	(SPRD_SDHC_BIT_INT_TRAN_END | SPRD_SDHC_BIT_INT_DMA_END | \
	SPRD_SDHC_BIT_INT_ERR | SPRD_SDHC_BIT_INT_ERR_DATA_TIMEOUT | \
	SPRD_SDHC_BIT_INT_ERR_DATA_CRC | SPRD_SDHC_BIT_INT_ERR_DATA_END)
#define SPRD_SDHC_DAT_FILTER_WR_SIGLE \
	(SPRD_SDHC_BIT_INT_TRAN_END | SPRD_SDHC_BIT_INT_DMA_END | \
	SPRD_SDHC_BIT_INT_ERR | SPRD_SDHC_BIT_INT_ERR_DATA_TIMEOUT | \
	SPRD_SDHC_BIT_INT_ERR_DATA_CRC)
#define SPRD_SDHC_DAT_FILTER_WR_MULTI \
	(SPRD_SDHC_BIT_INT_TRAN_END | SPRD_SDHC_BIT_INT_DMA_END | \
	SPRD_SDHC_BIT_INT_ERR | SPRD_SDHC_BIT_INT_ERR_DATA_TIMEOUT | \
	SPRD_SDHC_BIT_INT_ERR_DATA_CRC)
#define SPRD_SDHC_CMD_FILTER_R0 \
	SPRD_SDHC_BIT_INT_CMD_END
#define SPRD_SDHC_CMD_FILTER_R2 \
	(SPRD_SDHC_BIT_INT_CMD_END | SPRD_SDHC_BIT_INT_ERR | \
	SPRD_SDHC_BIT_INT_ERR_CMD_TIMEOUT | SPRD_SDHC_BIT_INT_ERR_CMD_CRC | \
	SPRD_SDHC_BIT_INT_ERR_CMD_END)
#define SPRD_SDHC_CMD_FILTER_R3 \
	(SPRD_SDHC_BIT_INT_CMD_END | SPRD_SDHC_BIT_INT_ERR | \
	SPRD_SDHC_BIT_INT_ERR_CMD_TIMEOUT | SPRD_SDHC_BIT_INT_ERR_CMD_END)
#define SPRD_SDHC_CMD_FILTER_R1_R4_R6_R7 \
	(SPRD_SDHC_BIT_INT_CMD_END | SPRD_SDHC_BIT_INT_ERR | \
	SPRD_SDHC_BIT_INT_ERR_CMD_TIMEOUT | SPRD_SDHC_BIT_INT_ERR_CMD_CRC | \
	SPRD_SDHC_BIT_INT_ERR_CMD_END | SPRD_SDHC_BIT_INT_ERR_CMD_INDEX)
#define SPRD_SDHC_CMD_FILTER_R1B \
	(SPRD_SDHC_BIT_INT_CMD_END | SPRD_SDHC_BIT_INT_ERR | \
	SPRD_SDHC_BIT_INT_ERR_CMD_TIMEOUT | SPRD_SDHC_BIT_INT_ERR_CMD_CRC | \
	SPRD_SDHC_BIT_INT_ERR_CMD_END | SPRD_SDHC_BIT_INT_ERR_CMD_INDEX | \
	SPRD_SDHC_BIT_INT_TRAN_END | SPRD_SDHC_BIT_INT_ERR_DATA_TIMEOUT)

static inline void sprd_sdhc_disable_all_int(struct sprd_sdhc_host *host)
{
	writel_relaxed(0x0, host->ioaddr + SPRD_SDHC_REG_32_INT_SIG_EN);
	writel_relaxed(0x0, host->ioaddr + SPRD_SDHC_REG_32_INT_STATE_EN);
	writel_relaxed(0xFFFFFFFF, host->ioaddr + SPRD_SDHC_REG_32_INT_STATE);
}

static inline void sprd_sdhc_enable_int(struct sprd_sdhc_host *host, u32 mask)
{
	writel_relaxed(mask, host->ioaddr + SPRD_SDHC_REG_32_INT_STATE_EN);
	writel_relaxed(mask, host->ioaddr + SPRD_SDHC_REG_32_INT_SIG_EN);
}

static inline void sprd_sdhc_clear_int(struct sprd_sdhc_host *host, u32 mask)
{
	writel_relaxed(mask, host->ioaddr + SPRD_SDHC_REG_32_INT_STATE);
}

#define SPRD_SDHC_REG_16_ACMD_ERR		0x3C

#define SPRD_SDHC_REG_16_HOST_CTRL2	0x3E
#define SPRD_SDHC_BIT_TIMING_MODE_SDR12		0x0000
#define SPRD_SDHC_BIT_TIMING_MODE_SDR25		0x0001
#define SPRD_SDHC_BIT_TIMING_MODE_SDR50		0x0002
#define SPRD_SDHC_BIT_TIMING_MODE_SDR104	0x0003
#define SPRD_SDHC_BIT_TIMING_MODE_DDR50		0x0004
#define SPRD_SDHC_BIT_TIMING_MODE_HS200		0x0005
#define SPRD_SDHC_BIT_TIMING_MODE_HS400		0x0006
#define SPRD_SDHC_BIT_64BIT_ADDR_EN		0x2000

static inline void sprd_sdhc_set_uhs_mode(struct sprd_sdhc_host *host, u8 mode)
{
	u16 tmp;

	tmp = readw_relaxed(host->ioaddr + SPRD_SDHC_REG_16_HOST_CTRL2);
	tmp &= 0xfff8;
	tmp |= mode;
	writew_relaxed(tmp, host->ioaddr + SPRD_SDHC_REG_16_HOST_CTRL2);
}

static inline void sprd_sdhc_set_64bit_addr(struct sprd_sdhc_host *host,
					u16 addr_en)
{
	u16 tmp;

	tmp = readw_relaxed(host->ioaddr + SPRD_SDHC_REG_16_HOST_CTRL2);
	if (addr_en)
		tmp |= SPRD_SDHC_BIT_64BIT_ADDR_EN;
	else
		tmp &= ~SPRD_SDHC_BIT_64BIT_ADDR_EN;
	writew_relaxed(tmp, host->ioaddr + SPRD_SDHC_REG_16_HOST_CTRL2);
}


#define SPRD_SDHC_MAX_CUR	1020

/*
 * the following register is defined by spreadtrum self.
 * It is not standard register of SDIO
 */
static inline void sprd_sdhc_set_delay(struct sprd_sdhc_host *host,
				     u32 write_delay,
				     u32 read_pos_delay,
				     u32 read_neg_delay)
{
	writel_relaxed(write_delay, host->ioaddr + 0x80);
	writel_relaxed(read_pos_delay, host->ioaddr + 0x84);
	writel_relaxed(read_neg_delay, host->ioaddr + 0x88);
}

/* 58-5C is used for SD SPEC 4.0/4.1 */
#define SPRD_SDHC_REG_32_ADMA2_ADDR_L   0x58
#define SPRD_SDHC_REG_32_SDMA_ADDR      0x58
#define SPRD_SDHC_REG_32_ADMA2_ADDR_H   0x5C

#define SPRD_SDHC_REG_16_HOST_VER   0xFE
#define SPRD_SDHC_BIT_SPEC_100     0
#define SPRD_SDHC_BIT_SPEC_200     1
#define SPRD_SDHC_BIT_SPEC_300     2
#define SPRD_SDHC_BIT_SPEC_400     3
#define SPRD_SDHC_BIT_SPEC_410     4

#define SPRD_SDHC_REG_32_DLL_CFG	0x200
#define SPRD_SDHC_DLL_ALL_CPST_EN	0x0F040000
#define SPRD_SDHC_DLL_EN		0x00200000
#define SPRD_SDHC_DLL_SEARCH_MODE	0x00010000
#define SPRD_SDHC_DLL_INIT_COUNT	0x00000c00
#define SPRD_SDHC_DLL_PHA_INTERNAL	0x00000003

#define SPRD_SDHC_REG_32_DLL_DLY	0x204
#define SPRD_SDHC_REG_8_DATWR_DLY	0x204
#define SPRD_SDHC_REG_8_CMDRD_DLY	0x205
#define SPRD_SDHC_REG_8_POSRD_DLY	0x206
#define SPRD_SDHC_REG_8_NEGRD_DLY	0x207

#define SPRD_SDHC_REG_32_DLL_STS0	0x210
#define SPRD_SDHC_REG_32_DLL_STS1	0x214
#define SPRD_SDHC_DLL_ERROR		0x00020000
#define SPRD_SDHC_DLL_LOCKED		0x00040000

#endif /* __SPRD_SDHC_H_ */
