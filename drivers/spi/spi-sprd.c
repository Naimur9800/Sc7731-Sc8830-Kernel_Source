/*
 * Copyright (C) 2013-2015 Spreadtrum Communications Inc.
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
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dma/sprd_dma.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/sched.h>
#include <linux/slab.h>

#define SPI_TXD				0x0
#define SPI_CLKD			0x4
#define SPI_CTL0			0x8
#define SPI_CTL1			0xc
#define SPI_CTL2			0x10
#define SPI_CTL3			0x14
#define SPI_CTL4			0x18
#define SPI_CTL5			0x1c
#define SPI_INT_EN			0x20
#define SPI_INT_CLR			0x24
#define SPI_INT_RAW_STS			0x28
#define SPI_INT_MASK_STS		0x2c
#define SPI_STS1			0x30
#define SPI_STS2			0x34
#define SPI_DSP_WAIT			0x38
#define SPI_STS3			0x3c
#define SPI_CTL6			0x40
#define SPI_STS4			0x44
#define SPI_FIFO_RST			0x48
#define SPI_CTL7			0x4c
#define SPI_STS5			0x50
#define SPI_CTL8			0x54
#define SPI_CTL9			0x58
#define SPI_CTL10			0x5c
#define SPI_CTL11			0x60
#define SPI_CTL12			0x64
#define SPI_STS6			0x68
#define SPI_STS7			0x6c
#define SPI_STS8			0x70
#define SPI_STS9			0x74

/* Bit define for register SPI_CTL0 */
#define SCK_REV				BIT(13)
#define NG_TX				BIT(1)
#define NG_RX				BIT(0)

/* Bit define for register INT_RAW_STS */
#define SPI_TX_END_RAW_STS		BIT(8)
#define SPI_RX_END_RAW_STS		BIT(9)

/* Bit define for register INT_CLR */
#define SPI_TX_END_CLR			BIT(8)
#define SPI_RX_END_CLR			BIT(9)

/* Bit define for register STS2 */
#define SPI_RX_FIFO_FULL		BIT(0)
#define SPI_RX_FIFO_EMPTY		BIT(1)
#define SPI_TX_FIFO_FULL		BIT(2)
#define SPI_TX_FIFO_EMPTY		BIT(3)
#define SPI_RX_FIFO_REALLY_FULL		BIT(4)
#define SPI_RX_FIFO_REALLY_EMPTY	BIT(5)
#define SPI_TX_FIFO_REALLY_FULL		BIT(6)
#define SPI_TX_FIFO_REALLY_EMPTY	BIT(7)
#define SPI_TX_BUSY			BIT(8)

/* Bit define for register ctr1 */
#define SPI_RX_MODE			BIT(12)
#define SPI_TX_MODE			BIT(13)

/* Bit define for register ctr2 */
#define SPI_DMA_EN			BIT(6)

/* Bit define for register ctr4 */
#define SPI_START_RX			BIT(9)

/* Register ctr5, for master, transmit data interval */
#define ITVL_NUM			0x9

/* Bit define for register ctr12 */
#define SPI_SW_RX_REQ			BIT(0)
#define SPI_SW_TX_REQ			BIT(1)

#define SPRD_SPI_FIFO_SIZE		32
#define SPRD_DMA_STEP_LEN		8
#define FIFO_RX_FULL			8
#define FIFO_TX_EMPTY			2
#define SPRD_SPI_CHIP_CS_NUM		0x4
#define MAX_BITS_PER_WORD		32
#define SPI_TIME_OUT			0xff0000

#define ONLY_RECV_MASK			GENMASK(8, 0)
#define RTX_MD_MASK			GENMASK(13, 12)
#define CHNL_LEN_MASK			GENMASK(4, 0)
#define SPI_CSN_MASK			GENMASK(11, 8)
#define RXF_FULL_THLD_MASK		GENMASK(4, 0)

#define SPRD_SPI_MODE_MASK		GENMASK(5, 3)
#define SPRD_SPI_MODE_OFFSET		3

#define SPI_TX_MAX_LEN_MASK		GENMASK(19, 0)
#define SPI_TX_LEN_H_MASK		GENMASK(3, 0)
#define SPI_TX_LEN_H_OFFSET		16
#define SPI_TX_LEN_L_MASK		GENMASK(15, 0)
#define SPI_RX_MAX_LEN_MASK		GENMASK(19, 0)
#define SPI_RX_LEN_H_MASK		GENMASK(3, 0)
#define SPI_RX_LEN_H_OFFSET		16
#define SPI_RX_LEN_L_MASK		GENMASK(15, 0)

/* timeout for pm runtime autosuspend */
#define SPRD_PM_TIMEOUT_MS		1000

struct clk_src {
	u32 freq;
	const char *name;
};

enum spi_transfer_mode {
	rx_mode = 1,
	tx_mode,
	rt_mode,
};

struct sprd_spi {
	struct spi_bitbang bitbang;
	void __iomem *reg_base;
	u32 spi_base;
	struct device *dev;
	int irq_num;
	struct clk *clk;
	struct clk *hs_clk;
	bool hs_spi;
	spinlock_t lock;
	u32 spi_src_clk;
	u32 max_speed_hz;
	u32 block_num;
	u32 spi_clk_div;
	u32 default_per_word;
	u32 spi_mode_data;
	u32 spi_mode;
	u32 interval;
	enum spi_transfer_mode transfer_mode;

	u32 dma_enable;
	/* dma[0] for rx dma channel, dma[1] for tx dma channel */
	struct dma_chan *dma[2];
	/* callback[0] for rx dma call back, callback [1] for tx */
	dma_async_tx_callback callback[2];
	/* cookie[0] for rx dma cookie, cookie [1] for tx */
	dma_cookie_t cookie[2];
	/* dma_id[0] for rx hardware request id, dma_id[1] for tx */
	u32 dma_id[2];
	struct completion done;
	dma_datawidth datawidth;
	dma_addr_t *rx_buf;
	u32 dma_step;
	int status;

	int  (*read_bufs)(struct sprd_spi *, void *, u32);
	int  (*write_bufs)(struct sprd_spi *, const void *, u32);
};

static void
sprd_spi_dump_regs(struct sprd_spi *spi_chip)
{
	dev_err(spi_chip->dev, "SPI_CLKD:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_CLKD));
	dev_err(spi_chip->dev, "SPI_CTL0:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_CTL0));
	dev_err(spi_chip->dev, "SPI_CTL1:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_CTL1));
	dev_err(spi_chip->dev, "SPI_CTL2:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_CTL2));
	dev_err(spi_chip->dev, "SPI_CTL3:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_CTL3));
	dev_err(spi_chip->dev, "SPI_CTL4:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_CTL4));
	dev_err(spi_chip->dev, "SPI_CTL5:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_CTL5));
	dev_err(spi_chip->dev, "SPI_INT_EN:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_INT_EN));
	dev_err(spi_chip->dev, "SPI_STS2:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_STS2));
	dev_err(spi_chip->dev, "SPI_DSP_WAIT:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_DSP_WAIT));
	dev_err(spi_chip->dev, "SPI_CTL6:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_CTL6));
	dev_err(spi_chip->dev, "SPI_CTL7:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_CTL7));
	dev_err(spi_chip->dev, "SPI_CTL8:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_CTL8));
	dev_err(spi_chip->dev, "SPI_CTL9:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_CTL9));
	dev_err(spi_chip->dev, "SPI_CTL10:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_CTL10));
	dev_err(spi_chip->dev, "SPI_CTL11:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_CTL11));
	dev_err(spi_chip->dev, "SPI_STS6:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_STS6));
	dev_err(spi_chip->dev, "SPI_STS7:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_STS7));
	dev_err(spi_chip->dev, "SPI_STS8:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_STS8));
	dev_err(spi_chip->dev, "SPI_STS9:0x%x\n",
		readl_relaxed(spi_chip->reg_base + SPI_STS9));
}

static int
sprd_spi_wait_for_send_complete(struct sprd_spi *spi_chip)
{
	u32 timeout = 0;

	if (spi_chip->transfer_mode != rx_mode) {
		while (!(readl_relaxed(spi_chip->reg_base + SPI_STS2)
			& SPI_TX_FIFO_REALLY_EMPTY)) {
			if (++timeout > SPI_TIME_OUT) {
				dev_err(spi_chip->dev, "SPI error, spi send timeout!\n");
				sprd_spi_dump_regs(spi_chip);
				break;
			}
		}
	}

	if (spi_chip->spi_mode && spi_chip->transfer_mode == rx_mode) {
		while (!(readl_relaxed(spi_chip->reg_base + SPI_INT_RAW_STS)
				& SPI_RX_END_RAW_STS)) {
			if (++timeout > SPI_TIME_OUT) {
				dev_err(spi_chip->dev, "SPI error, spi rx end timeout\n");
				sprd_spi_dump_regs(spi_chip);
				break;
			}
		}
		writel_relaxed(SPI_RX_END_CLR,
			spi_chip->reg_base + SPI_INT_CLR);
	}

	while (readl_relaxed(spi_chip->reg_base + SPI_STS2)
		& SPI_TX_BUSY) {
		if (++timeout > SPI_TIME_OUT) {
			dev_err(spi_chip->dev, "SPI error, spi busy timeout!\n");
			sprd_spi_dump_regs(spi_chip);
			break;
		}
	}

	if (timeout > SPI_TIME_OUT)
		return -EIO;

	return 0;
}

static void  sprd_spi_chipselect(struct spi_device *spi_dev, int is_on)
{
	u32 reg_val = 0;
	struct sprd_spi *spi_chip;
	struct spi_master	*master = spi_dev->master;
	int ret;

	spi_chip = spi_master_get_devdata(master);

	ret = pm_runtime_get_sync(spi_chip->dev);
	if (ret < 0) {
		dev_err(spi_chip->dev, "pm runtime resume failed!\n");
		return;
	}

	reg_val = readl_relaxed(spi_chip->reg_base + SPI_CTL0);
	if (is_on) {
		reg_val &= ~(0x1 << (spi_dev->chip_select + 8));
		writel_relaxed(reg_val, (spi_chip->reg_base + SPI_CTL0));
	} else {
		reg_val |= SPI_CSN_MASK;
		writel_relaxed(reg_val, (spi_chip->reg_base + SPI_CTL0));
	}
	pm_runtime_mark_last_busy(spi_chip->dev);
	pm_runtime_put_autosuspend(spi_chip->dev);
}

static int
sprd_spi_write_at_only_read(struct sprd_spi *spi_chip,
				const void *tx_buf, u32 block_num)
{
	u32 reg_val, timeout = 0;

	/* if spi_mode is set, do nothing */
	if (spi_chip->spi_mode)
		return block_num;
	/* Clear the start receive bit and reset receive data num */
	reg_val = readl_relaxed(spi_chip->reg_base + SPI_CTL4);
	reg_val &= ~(SPI_START_RX | ONLY_RECV_MASK);
	writel_relaxed(reg_val, spi_chip->reg_base + SPI_CTL4);

	reg_val = readl_relaxed(spi_chip->reg_base + SPI_CTL4);
	reg_val |= block_num & ONLY_RECV_MASK;
	writel_relaxed(reg_val, spi_chip->reg_base + SPI_CTL4);

	/* start send 0 on tx line */
	reg_val = readl_relaxed(spi_chip->reg_base + SPI_CTL4);
	reg_val |= SPI_START_RX;
	writel_relaxed(reg_val, spi_chip->reg_base + SPI_CTL4);

	if (spi_chip->dma_enable)
		return block_num;
	while ((readl_relaxed(spi_chip->reg_base + SPI_STS3))
		!= block_num) {
		if (++timeout > SPI_TIME_OUT)
			dev_err(spi_chip->dev,
				"SPI error, at_only_read mode spi send timeout!\n");
	}

	return block_num;
}

static int
sprd_spi_write_bufs_u8(struct sprd_spi *spi_chip,
				const void *tx_buf, u32 block_num)
{
	int i;
	u8 *tx_p = (u8 *)tx_buf;

	for (i = 0; i < block_num; i++, tx_p++)
		writeb_relaxed(*tx_p, spi_chip->reg_base + SPI_TXD);
	return i;
}
static int
sprd_spi_write_bufs_u16(struct sprd_spi *spi_chip,
				const void *tx_buf, u32 block_num)
{
	int i;
	u16 *tx_p = (u16 *)tx_buf;

	for (i = 0; i < block_num; i++, tx_p++)
		writew_relaxed(*tx_p, spi_chip->reg_base + SPI_TXD);
	return (i << 1);
}
static int
sprd_spi_write_bufs_u32(struct sprd_spi *spi_chip,
				const void *tx_buf, u32 block_num)
{
	int i;
	u32 *tx_p = (u32 *)tx_buf;

	for (i = 0; i < block_num; i++, tx_p++)
		writel_relaxed(*tx_p, spi_chip->reg_base + SPI_TXD);

	return (i << 2);
}

static int
sprd_spi_read_bufs_u8(struct sprd_spi *spi_chip,
				void *rx_buf, u32 block_num)
{
	int i;
	u8 *rx_p = (u8 *)rx_buf;

	for (i = 0; i < block_num; i++, rx_p++)
		*rx_p = readb_relaxed(spi_chip->reg_base + SPI_TXD);

	return i;
}

static int
sprd_spi_read_bufs_u16(struct sprd_spi *spi_chip,
				void *rx_buf, u32 block_num)
{
	int i;
	u16 *rx_p = (u16 *)rx_buf;

	for (i = 0; i < block_num; i++, rx_p++)
		*rx_p = readw_relaxed(spi_chip->reg_base + SPI_TXD);

	return (i << 1);
}
static int
sprd_spi_read_bufs_u32(struct sprd_spi *spi_chip,
				void *rx_buf, u32 block_num)
{
	int i;
	u32 *rx_p = (u32 *)rx_buf;

	for (i = 0; i < block_num; i++, rx_p++)
		*rx_p = readl_relaxed(spi_chip->reg_base + SPI_TXD);

	return (i << 2);
}

static u32
sprd_spi_get_tx_length(struct sprd_spi *spi_chip)
{
	u32 val_l = readl_relaxed(spi_chip->reg_base + SPI_STS6);
	u32 val_h = readl_relaxed(spi_chip->reg_base + SPI_STS7);
	u32 length;

	length = (val_h & SPI_TX_LEN_H_MASK) << SPI_TX_LEN_H_OFFSET;
	length = length | (val_l & SPI_TX_LEN_L_MASK);

	return length;
}

static void
sprd_spi_set_tx_length(struct sprd_spi *spi_chip, u32 length)
{
	u32 reg_val = readl_relaxed(spi_chip->reg_base + SPI_CTL8);

	length &= SPI_TX_MAX_LEN_MASK;
	reg_val &= ~SPI_TX_LEN_H_MASK;
	reg_val |= length >> SPI_TX_LEN_H_OFFSET;
	writel_relaxed(reg_val, spi_chip->reg_base + SPI_CTL8);
	reg_val = length & SPI_TX_LEN_L_MASK;
	writel_relaxed(reg_val, spi_chip->reg_base + SPI_CTL9);
}

static void
sprd_spi_set_rx_length(struct sprd_spi *spi_chip, u32 length)
{
	u32 reg_val = readl_relaxed(spi_chip->reg_base + SPI_CTL10);

	length &= SPI_RX_MAX_LEN_MASK;
	reg_val &= ~SPI_RX_LEN_H_MASK;
	reg_val |= length >> SPI_RX_LEN_H_OFFSET;
	writel_relaxed(reg_val, spi_chip->reg_base + SPI_CTL10);
	reg_val = length & SPI_RX_LEN_L_MASK;
	writel_relaxed(reg_val, spi_chip->reg_base + SPI_CTL11);
}

static void
sprd_spi_set_tx(struct sprd_spi *spi_chip, u32 num)
{
	sprd_spi_set_rx_length(spi_chip, 0);
	sprd_spi_set_tx_length(spi_chip, num);
	writel_relaxed(SPI_SW_TX_REQ, spi_chip->reg_base + SPI_CTL12);
}

static void
sprd_spi_set_rx(struct sprd_spi *spi_chip, u32 num)
{
	sprd_spi_set_tx_length(spi_chip, 0);
	sprd_spi_set_rx_length(spi_chip, num);
	writel_relaxed(SPI_SW_RX_REQ, spi_chip->reg_base + SPI_CTL12);
}

static int sprd_spi_txrx_bufs(struct spi_device *spi_dev,
	struct spi_transfer *trans_node)
{
	int ret, write_num = 0, read_num = 0;
	u32  reg_val = 0, trans_num = 0;
	struct sprd_spi *spi_chip =
		spi_master_get_devdata(spi_dev->master);
	u32  block_num = spi_chip->block_num;
	const void *tx_buf = trans_node->tx_buf;
	void	 *rx_buf = trans_node->rx_buf;

	ret = pm_runtime_get_sync(spi_chip->dev);
	if (ret < 0) {
		dev_err(spi_chip->dev, "pm runtime resume failed!\n");
		return ret;
	}

	while (block_num) {
		trans_num = block_num > SPRD_SPI_FIFO_SIZE
			? SPRD_SPI_FIFO_SIZE : block_num;

		if (spi_chip->spi_mode) {
			if (spi_chip->transfer_mode == tx_mode)
				sprd_spi_set_tx(spi_chip, trans_num);
			else
				sprd_spi_set_rx(spi_chip, trans_num);
		}

		write_num += spi_chip->write_bufs(spi_chip,
				tx_buf, trans_num);

		ret = sprd_spi_wait_for_send_complete(spi_chip);
		if (ret)
			goto complete;

		if (spi_chip->transfer_mode != tx_mode) {
			read_num += spi_chip->read_bufs(spi_chip,
					rx_buf, trans_num);
			rx_buf += trans_num;
		}
		tx_buf +=  trans_num;
		block_num -= trans_num;
	}

	ret = write_num;

complete:
	reg_val = readl_relaxed(spi_chip->reg_base + SPI_CTL1);
	reg_val &= ~RTX_MD_MASK;
	writel_relaxed(reg_val, spi_chip->reg_base + SPI_CTL1);

	pm_runtime_mark_last_busy(spi_chip->dev);
	pm_runtime_put_autosuspend(spi_chip->dev);

	return ret;
}

static int sprd_spi_setup_transfer(struct spi_device *spi_dev,
		struct spi_transfer *t)
{
	int ret;
	u32 reg_val;
	u8 bits_per_word = 0;
	struct sprd_spi *spi_chip =
		spi_master_get_devdata(spi_dev->master);

	if (t)
		bits_per_word = t->bits_per_word;
	else
		return -EINVAL;

	if (!bits_per_word)
		bits_per_word = spi_chip->default_per_word;

	switch (bits_per_word) {
	case 8:
		spi_chip->block_num = t->len;
		spi_chip->datawidth = BYTE_WIDTH;
		spi_chip->dma_step = 1;
		spi_chip->read_bufs = sprd_spi_read_bufs_u8;
		spi_chip->write_bufs = sprd_spi_write_bufs_u8;
		break;
	case 16:
		spi_chip->block_num = (t->len + 1)  >> 1;
		spi_chip->datawidth = SHORT_WIDTH;
		spi_chip->dma_step = 2;
		spi_chip->read_bufs = sprd_spi_read_bufs_u16;
		spi_chip->write_bufs = sprd_spi_write_bufs_u16;
		break;
	case 32:
		spi_chip->block_num = (t->len + 3) >> 2;
		spi_chip->datawidth = WORD_WIDTH;
		spi_chip->dma_step = 4;
		spi_chip->read_bufs = sprd_spi_read_bufs_u32;
		spi_chip->write_bufs = sprd_spi_write_bufs_u32;
		break;
	default:
		dev_err(&spi_dev->dev,
			" Invalid argument bits_per_word = %d\n",
			bits_per_word);
		return -EINVAL;
	}

	spi_chip->status = 0;
	ret = pm_runtime_get_sync(spi_chip->dev);
	if (ret < 0) {
		dev_err(spi_chip->dev, "pm runtime resume failed!\n");
		return ret;
	}

	reg_val = readl_relaxed(spi_chip->reg_base + SPI_CTL0);
	reg_val &= ~(CHNL_LEN_MASK << 2);
	reg_val |= (bits_per_word & CHNL_LEN_MASK) << 2;
	writel_relaxed(reg_val, spi_chip->reg_base + SPI_CTL0);

	reg_val = readl_relaxed(spi_chip->reg_base + SPI_CTL1);
	reg_val &= ~RTX_MD_MASK;
	if (t->tx_buf)
		reg_val |= SPI_TX_MODE;

	if (t->rx_buf) {
		reg_val |= SPI_RX_MODE;
		spi_chip->rx_buf = t->rx_buf;
	}

	writel_relaxed(reg_val, spi_chip->reg_base + SPI_CTL1);
	spi_chip->transfer_mode = (reg_val>>12)&0x3;

	if (spi_chip->transfer_mode == rx_mode)
		spi_chip->write_bufs = sprd_spi_write_at_only_read;

	writel_relaxed(0x1, spi_chip->reg_base + SPI_FIFO_RST);
	writel_relaxed(0x0, spi_chip->reg_base + SPI_FIFO_RST);
	pm_runtime_mark_last_busy(spi_chip->dev);
	pm_runtime_put_autosuspend(spi_chip->dev);

	return 0;
}

static void sprd_spi_dma_rx_callback(void *data)
{
	struct sprd_spi *spi_chip = (struct sprd_spi *)data;

	spi_chip->read_bufs(spi_chip, spi_chip->rx_buf, spi_chip->block_num);
	dev_dbg(spi_chip->dev, "rx_callback!\n");
	complete(&spi_chip->done);
}

static void sprd_spi_dma_tx_callback(void *data)
{
	struct sprd_spi *spi_chip = (struct sprd_spi *)data;
	u32 timeout = 0;

	dev_dbg(spi_chip->dev, "tx_callback!\n");
	if (spi_chip->transfer_mode == tx_mode) {
		while (sprd_spi_get_tx_length(spi_chip)) {
			if (++timeout > SPI_TIME_OUT) {
				dev_err(spi_chip->dev, "SPI error, get_tx_length timeout\n");
				spi_chip->status = -EIO;
				break;
			}
			cpu_relax();
		}
		complete(&spi_chip->done);
	}
}

static void
sprd_spi_dma_mode_enable(struct sprd_spi *spi_chip,
							int enable)
{
	unsigned int temp_reg = readl_relaxed(spi_chip->reg_base + SPI_CTL2);

	if (enable)
		temp_reg |= SPI_DMA_EN;
	else
		temp_reg &= ~SPI_DMA_EN;

	writel_relaxed(temp_reg, spi_chip->reg_base + SPI_CTL2);
}

static int
sprd_spi_dma_config(struct sprd_spi *spi_chip,
				struct sprd_dma_cfg *c, int rx)
{
	int ret = 0;
	struct dma_async_tx_descriptor *dma_des;
	struct dma_device *dma_dev = spi_chip->dma[rx]->device;
	struct spi_master	*master = spi_chip->bitbang.master;

	ret = dmaengine_slave_config(spi_chip->dma[rx],
					     &(c->config));
	if (ret < 0) {
		dev_err(&master->dev, "dma rx config error!\n");
		return ret;
	}

	dma_des = dma_dev->device_prep_dma_memcpy(
						spi_chip->dma[rx], 0, 0, 0,
						DMA_CFG_FLAG|DMA_HARDWARE_FLAG);
	if (!dma_des) {
		dev_err(&master->dev, "DMA chan ID %d memcpy is failed!\n",
			spi_chip->dma[rx]->chan_id);
		return  -ENODEV;
	}

	dma_des->callback = spi_chip->callback[rx];
	dma_des->callback_param = (void *)spi_chip;
	spi_chip->cookie[rx] = dmaengine_submit(dma_des);

	return 0;
}

static void
sprd_spi_set_rx_fifo_thld(struct sprd_spi *spi_chip,
				unsigned len)
{
	u32 spi_ctl3 = readl_relaxed(spi_chip->reg_base + SPI_CTL3);

	spi_ctl3 &= ~RXF_FULL_THLD_MASK;
	writel_relaxed(spi_ctl3 | len, spi_chip->reg_base + SPI_CTL3);
}

static int
sprd_spi_dma_rx_config(struct sprd_spi *spi_chip,
				struct spi_transfer *t, unsigned len)
{
	int ret = 0;
	u32 fragmens_len =
		len > SPRD_DMA_STEP_LEN ? SPRD_DMA_STEP_LEN : len;

	struct sprd_dma_cfg dma_config = {
		.datawidth = spi_chip->datawidth,
		.fragmens_len = fragmens_len,
		.block_len = len,
		.transcation_len = len,
		.src_step = 0,
		.des_step = spi_chip->dma_step,
		.des_addr = t->rx_dma,
		.src_addr = spi_chip->spi_base,
		.req_mode = FRAG_REQ_MODE,
		.irq_mode = TRANS_DONE,
		.dev_id = spi_chip->dma_id[0],
		.is_end = 1,
	};

	ret = sprd_spi_dma_config(spi_chip, &dma_config, 0);
	if (ret)
		return ret;

	sprd_spi_set_rx_fifo_thld(spi_chip, fragmens_len);
	spi_chip->rx_buf = t->rx_buf + len;
	spi_chip->block_num = spi_chip->block_num - len;

	return len;
}

static int
sprd_spi_dma_tx_config(struct sprd_spi *spi_chip,
				struct spi_transfer *t)
{
	int ret = 0;
	unsigned len = t->len;
	struct sprd_dma_cfg dma_config = {
		.datawidth = spi_chip->datawidth,
		.fragmens_len = SPRD_DMA_STEP_LEN,
		.block_len = len,
		.transcation_len = len,
		.src_step = spi_chip->dma_step,
		.des_step = 0,
		.des_addr = spi_chip->spi_base,
		.src_addr = t->tx_dma,
		.req_mode = FRAG_REQ_MODE,
		.irq_mode = TRANS_DONE,
		.dev_id = spi_chip->dma_id[1],
		.is_end = 1,
	};

	ret = sprd_spi_dma_config(spi_chip, &dma_config, 1);
	if (ret)
		return ret;

	return len;
}

static int sprd_spi_request_dma(struct sprd_spi *spi_chip)
{
	struct spi_master	*master = spi_chip->bitbang.master;
	struct device_node	*np = master->dev.of_node;

	spi_chip->dma[0]  = of_dma_request_slave_channel(np,
			"rx_chn");
	if (IS_ERR(spi_chip->dma[0])) {
		dev_err(&master->dev, "request RX DMA channel failed!\n");
		return -ENODEV;
	}

	spi_chip->dma[1]  = of_dma_request_slave_channel(np,
			"tx_chn");
	if (IS_ERR(spi_chip->dma[1])) {
		dev_err(&master->dev, "request TX DMA channel failed!\n");
		dma_release_channel(spi_chip->dma[0]);
		return -ENODEV;
	}

	return 0;
}

static void sprd_spi_release_dma(struct sprd_spi *spi_chip)
{
	if (spi_chip->dma_enable) {
		dma_release_channel(spi_chip->dma[0]);
		dma_release_channel(spi_chip->dma[1]);
	}
}

static int
sprd_spi_dma_buf_mapping(struct sprd_spi *spi_chip,
	struct spi_transfer *t)
{
	if (t->tx_buf) {
		t->tx_dma = dma_map_single(spi_chip->dev, (void *)t->tx_buf,
			t->len, DMA_TO_DEVICE);
		if (dma_mapping_error(spi_chip->dev, t->tx_dma)) {
			dev_err(spi_chip->dev, " spi dma map tx_buf error !\n");
			return -ENOMEM;
		}
	}

	if (t->rx_buf) {
		t->rx_dma = dma_map_single(spi_chip->dev, t->rx_buf,
			t->len, DMA_FROM_DEVICE);
		if (dma_mapping_error(spi_chip->dev, t->rx_dma)) {
			dev_err(spi_chip->dev, " spi dma map rx_buf error !\n");
			dma_unmap_single(spi_chip->dev, t->tx_dma,
				t->len, DMA_TO_DEVICE);
			return -ENOMEM;
		}
	}

	return 0;
}

static int
sprd_spi_dma_mapping_free(struct sprd_spi *spi_chip,
	struct spi_transfer *t)
{
	if (spi_chip->transfer_mode != rx_mode)
		dma_unmap_single(spi_chip->dev, t->tx_dma,
		t->len, DMA_TO_DEVICE);

	if (spi_chip->transfer_mode != tx_mode)
		dma_unmap_single(spi_chip->dev, t->rx_dma,
		t->len, DMA_FROM_DEVICE);

	return 0;
}

static int sprd_spi_dma_txrx_bufs(struct spi_device *spi_dev,
	struct spi_transfer *t)
{
	struct sprd_spi *spi_chip =
		spi_master_get_devdata(spi_dev->master);
	int ret, write_num = 0, block_num = spi_chip->block_num;
	u32 reg_val = 0, len = block_num - block_num % SPRD_DMA_STEP_LEN;

	reinit_completion(&spi_chip->done);
	ret = sprd_spi_dma_buf_mapping(spi_chip, t);
	if (ret)
		return ret;

	ret = pm_runtime_get_sync(spi_chip->dev);
	if (ret < 0) {
		dev_err(spi_chip->dev, "pm runtime resume failed!\n");
		return ret;
	}

	if (spi_chip->transfer_mode != rx_mode) {
		write_num = sprd_spi_dma_tx_config(spi_chip, t);
		if (spi_chip->spi_mode)
			sprd_spi_set_tx(spi_chip, block_num);
	} else
		write_num = spi_chip->write_bufs(spi_chip, NULL, block_num);

	if (write_num < 0) {
		dev_err(&spi_dev->dev, "tx config fail ret = %d\n", write_num);
		ret = write_num;
		goto trans_complete;
	}

	if (spi_chip->transfer_mode != tx_mode) {
		len = len ? len : block_num;
		ret = sprd_spi_dma_rx_config(spi_chip, t, len);
		if (ret < 0) {
			dev_err(&spi_dev->dev,
				"rx config fail ret = %d\n", ret);
			goto trans_complete;
		}
		if (spi_chip->spi_mode)
			sprd_spi_set_rx(spi_chip, block_num);
	}

	ret = write_num;
	sprd_spi_dma_mode_enable(spi_chip, 1);
	wait_for_completion(&(spi_chip->done));
	if (spi_chip->status)
		ret = spi_chip->status;

trans_complete:
	sprd_spi_dma_mode_enable(spi_chip, 0);
	reg_val = readl_relaxed(spi_chip->reg_base + SPI_CTL1);
	reg_val &= ~RTX_MD_MASK;
	writel_relaxed(reg_val, spi_chip->reg_base + SPI_CTL1);

	pm_runtime_mark_last_busy(spi_chip->dev);
	pm_runtime_put_autosuspend(spi_chip->dev);
	sprd_spi_dma_mapping_free(spi_chip, t);

	return ret;
}

static void sprd_spi_set_speed(struct sprd_spi *ss)
{
	/*
	 * From SPI datasheet, the prescale calculation formula:
	 * prescale = SPI source clock / (2 * SPI_freq) - 1;
	 */
	u32 clk_div = DIV_ROUND_UP(ss->spi_src_clk, ss->max_speed_hz << 1) - 1;

	writel_relaxed(clk_div, ss->reg_base + SPI_CLKD);
}

static int sprd_spi_reg_init(struct sprd_spi *spi_chip)
{
	u32 reg_val = SPI_CSN_MASK |
			spi_chip->default_per_word |
			spi_chip->spi_mode_data;

	writel_relaxed(reg_val, spi_chip->reg_base + SPI_CTL0);
	writel_relaxed(0x0, spi_chip->reg_base + SPI_CTL1);
	writel_relaxed(0x0, spi_chip->reg_base + SPI_CTL2);
	writel_relaxed((FIFO_TX_EMPTY << 8) | FIFO_RX_FULL,
		spi_chip->reg_base + SPI_CTL3);
	writel_relaxed(0x0, spi_chip->reg_base + SPI_CTL4);
	writel_relaxed(spi_chip->interval, spi_chip->reg_base + SPI_CTL5);
	writel_relaxed(0x0, spi_chip->reg_base + SPI_INT_EN);

	reg_val = readl_relaxed(spi_chip->reg_base + SPI_CTL7);
	reg_val &= ~SPRD_SPI_MODE_MASK;
	reg_val |= (spi_chip->spi_mode << SPRD_SPI_MODE_OFFSET)
			& SPRD_SPI_MODE_MASK;
	writel_relaxed(reg_val, spi_chip->reg_base + SPI_CTL7);

	writel_relaxed(0x1, spi_chip->reg_base + SPI_FIFO_RST);
	writel_relaxed(0x0, spi_chip->reg_base + SPI_FIFO_RST);

	sprd_spi_set_speed(spi_chip);

	return 0;
}

static int sprd_spi_setup(struct spi_device *spi_dev)
{
	int ret;
	struct spi_master *master = spi_dev->master;
	struct sprd_spi *spi_chip =
		spi_master_get_devdata(spi_dev->master);

	spi_chip->max_speed_hz = spi_dev->max_speed_hz;
	/* Set transmit data bit number, "0~31" */
	spi_chip->default_per_word =
		(spi_dev->bits_per_word & CHNL_LEN_MASK) << 2;

	switch (spi_dev->mode & master->mode_bits) {
	case SPI_MODE_0:
		spi_chip->spi_mode_data = NG_TX;
		break;
	case SPI_MODE_1:
		spi_chip->spi_mode_data = NG_RX;
		break;
	case SPI_MODE_2:
		spi_chip->spi_mode_data = SCK_REV | NG_TX;
		break;
	case SPI_MODE_3:
		spi_chip->spi_mode_data = SCK_REV | NG_RX;
		break;
	default:
		break;
	}

	ret = pm_runtime_get_sync(spi_chip->dev);
	if (ret < 0) {
		dev_err(spi_chip->dev, "pm runtime resume failed!\n");
		return ret;
	}

	sprd_spi_reg_init(spi_chip);
	pm_runtime_mark_last_busy(spi_chip->dev);
	pm_runtime_put_autosuspend(spi_chip->dev);
	return 0;
}

static int sprd_spi_probe(struct platform_device *pdev)
{
	int ret, irq_num;
	struct clk  *clk_spi, *clk_parent;
	struct resource *regs;
	struct spi_master *master;
	struct sprd_spi *spi_chip;

	pdev->id = of_alias_get_id(pdev->dev.of_node, "spi");
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev,
			"spi_%d get io resource failed!\n", pdev->id);
		return -ENODEV;
	}
	irq_num = platform_get_irq(pdev, 0);
	if (irq_num < 0) {
		dev_err(&pdev->dev,
			"spi_%d get irq resource failed!\n", pdev->id);
		return irq_num;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(*spi_chip));
	if (!master) {
		dev_err(&pdev->dev,
			"spi_%d spi_alloc_master failed!\n", pdev->id);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, master);
	spi_chip = spi_master_get_devdata(master);

	clk_spi = devm_clk_get(&pdev->dev, "spi");
	if (IS_ERR(clk_spi)) {
		dev_warn(&pdev->dev,
			"spi_%d can't get the clock dts config: spi\n",
			pdev->id);
		clk_spi = NULL;
	}

	clk_parent = devm_clk_get(&pdev->dev, "source");
	if (IS_ERR(clk_parent)) {
		dev_warn(&pdev->dev,
			"spi_%d can't get the clock dts config: source\n",
			pdev->id);
		clk_parent = NULL;
	}

	spi_chip->clk = devm_clk_get(&pdev->dev, "enable");
	if (IS_ERR(spi_chip->clk)) {
		dev_warn(&pdev->dev,
			"spi_%d can't get the clock dts config: enable\n",
			pdev->id);
		spi_chip->clk = NULL;
	}

	spi_chip->hs_spi = true;
	spi_chip->hs_clk = devm_clk_get(&pdev->dev, "hs_enable");
	if (IS_ERR(spi_chip->hs_clk)) {
		dev_warn(&pdev->dev,
			"spi_%d can't get the clock dts config: hs_enable\n",
			pdev->id);
		spi_chip->hs_spi = false;
	}

	if (of_property_read_u32(pdev->dev.of_node,
		"sprd,spi-interval", &spi_chip->interval))
		spi_chip->interval = ITVL_NUM;

	if (of_property_read_u32(pdev->dev.of_node,
		"sprd,spi-mode", &spi_chip->spi_mode))
		spi_chip->spi_mode = 0;

	if (of_property_read_u32(pdev->dev.of_node,
		"sprd,dma-mode", &spi_chip->dma_enable))
		spi_chip->dma_enable = 0;

	if (spi_chip->dma_enable) {
		if (of_property_read_u32_array(pdev->dev.of_node,
			"sprd,rxtx-dma", spi_chip->dma_id, 2)) {
			dev_err(&pdev->dev,
				"spi get dma req id fail, dma mode disable!\n");
			spi_chip->dma_enable = 0;
		}

		dev_dbg(&pdev->dev, " rx dma id= %d, tx dma id = %d\n",
			spi_chip->dma_id[0], spi_chip->dma_id[1]);
	}

	init_completion(&spi_chip->done);
	if (!clk_set_parent(clk_spi, clk_parent))
		spi_chip->spi_src_clk = clk_get_rate(clk_spi);
	else
		spi_chip->spi_src_clk = 26000000;

	spi_chip->reg_base = devm_ioremap_nocache(&pdev->dev, regs->start,
			regs->end - regs->start);
	spi_chip->spi_base = regs->start;
	spi_chip->irq_num  = irq_num;
	spi_chip->dev = &pdev->dev;

	master->dev.of_node = pdev->dev.of_node;
	master->mode_bits = SPI_CPOL | SPI_CPHA;
	master->bus_num = pdev->id;
	master->setup = sprd_spi_setup;

	spi_chip->bitbang.master = spi_master_get(master);
	spi_chip->bitbang.chipselect = sprd_spi_chipselect;
	spi_chip->bitbang.setup_transfer = sprd_spi_setup_transfer;
	spi_chip->bitbang.txrx_bufs = sprd_spi_txrx_bufs;
	spi_chip->callback[0] = sprd_spi_dma_rx_callback;
	spi_chip->callback[1] = sprd_spi_dma_tx_callback;

	if (spi_chip->dma_enable) {
		spi_chip->bitbang.txrx_bufs = sprd_spi_dma_txrx_bufs;
		ret = sprd_spi_request_dma(spi_chip);
		if (ret)
			goto put_master;
	}

	ret = clk_prepare_enable(spi_chip->clk);
	if (ret)
		goto err_dma;
	if (spi_chip->hs_spi) {
		ret = clk_prepare_enable(spi_chip->hs_clk);
		if (ret) {
			clk_disable_unprepare(spi_chip->clk);
			goto err_dma;
		}
	}

	device_init_wakeup(&pdev->dev, true);
	ret = pm_runtime_set_active(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "spi%d pm_runtime_set_active failed!!!\n",
			pdev->id);
		goto disable_clk;
	}
	pm_runtime_set_autosuspend_delay(&pdev->dev, SPRD_PM_TIMEOUT_MS);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "pm runtime resume failed!\n");
		goto err_rpm_put;
	}

	ret = spi_bitbang_start(&spi_chip->bitbang);
	if (ret) {
		dev_err(&pdev->dev, "register spi%d master probe %d failed!\n",
			pdev->id, master->bus_num);
		goto err_rpm_put;
	}
	dev_err(&pdev->dev, " spi_%d probe is ok\n", pdev->id);
	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_put_autosuspend(&pdev->dev);
	return ret;

err_rpm_put:
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
disable_clk:
	clk_disable_unprepare(spi_chip->clk);
	if (spi_chip->hs_spi)
		clk_disable_unprepare(spi_chip->hs_clk);
	device_init_wakeup(&pdev->dev, false);
err_dma:
	sprd_spi_release_dma(spi_chip);
put_master:
	spi_master_put(master);
	kfree(master);

	return ret;
}

static int __exit sprd_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct sprd_spi *spi_chip = spi_master_get_devdata(master);

	spi_bitbang_stop(&spi_chip->bitbang);

	if (spi_chip->hs_spi)
		clk_disable_unprepare(spi_chip->hs_clk);
	clk_disable_unprepare(spi_chip->clk);

	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	sprd_spi_release_dma(spi_chip);
	device_init_wakeup(&pdev->dev, false);
	spi_master_put(master);

	return 0;
}

#ifdef CONFIG_PM
static int sprd_spi_runtime_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct sprd_spi *spi_chip = spi_master_get_devdata(master);

	dev_dbg(dev, "sprd_spi_runtime_suspend\n");
	if (spi_chip->hs_spi)
		clk_disable_unprepare(spi_chip->hs_clk);
	clk_disable_unprepare(spi_chip->clk);

	sprd_spi_release_dma(spi_chip);
	pm_relax(dev);

	return 0;
}

static int sprd_spi_runtime_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct sprd_spi *spi_chip = spi_master_get_devdata(master);
	int ret;

	dev_dbg(dev, "sprd_spi_runtime_resume\n");
	pm_stay_awake(dev);
	ret = clk_prepare_enable(spi_chip->clk);
	if (ret)
		goto err_spi_clk;

	if (spi_chip->hs_spi) {
		ret = clk_prepare_enable(spi_chip->hs_clk);
		if (ret)
			goto err_hs_spi;
	}

	if (spi_chip->dma_enable) {
		ret = sprd_spi_request_dma(spi_chip);
		if (ret)
			goto err_request_dma;
	}

	sprd_spi_reg_init(spi_chip);
	return 0;

err_request_dma:
	if (spi_chip->hs_spi)
		clk_disable_unprepare(spi_chip->hs_clk);
err_hs_spi:
	clk_disable_unprepare(spi_chip->clk);
err_spi_clk:
	pm_relax(dev);

	return ret;
}
#endif

static const struct dev_pm_ops sprd_spi_pm_ops = {
	SET_RUNTIME_PM_OPS(sprd_spi_runtime_suspend,
		   sprd_spi_runtime_resume, NULL)
};

static const struct of_device_id sprd_spi_of_match[] = {
	{ .compatible = "sprd,spi-r4p0", },
	{ /* sentinel */ }
};

static struct platform_driver sprd_spi_driver = {
	.driver = {
		.name = "sprd spi",
		.of_match_table = of_match_ptr(sprd_spi_of_match),
		.pm = &sprd_spi_pm_ops,
	},
	.probe = sprd_spi_probe,
	.remove  = __exit_p(sprd_spi_remove),
};

static int __init sprd_spi_init(void)
{
	return platform_driver_register(&sprd_spi_driver);
}
subsys_initcall(sprd_spi_init);

static void __exit sprd_spi_exit(void)
{
	platform_driver_unregister(&sprd_spi_driver);
}
module_exit(sprd_spi_exit);

MODULE_DESCRIPTION("Spreadtrum SPI(r4p0 version) Controller driver");
MODULE_AUTHOR("Lanqing Liu <lanqing.liu@spreadtrum.com>");
MODULE_LICENSE("GPL");
