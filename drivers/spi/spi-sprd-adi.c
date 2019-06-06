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

/**
 * For arm read ,delay about 1us when clk_adi runs at 76.8M
 * The interface timing is compatible with spi timing
 */
#include <linux/delay.h>
#include <linux/hwspinlock.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqflags.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/sprd_hwspinlock.h>
#include <linux/sizes.h>

/* registers definitions for ADI controller */

/* bits definitions for register REG_ADI_CTRL0 */
#define REG_ADI_CTRL0			(0x04)
#define BIT_ARM_SCLK_EN			(BIT(1))
#define BITS_CMMB_WR_PRI		((1) << 4 & (BIT(4)|BIT(5)))
#define BIT_ADI_WR(_X_)			((_X_) << 2)
#define BITS_ADDR_BYTE_SEL(_X_)		((_X_) << 0 & (BIT(0)|BIT(1)))

/* bits definitions for register REG_ADI_CHNL_PRI */
#define REG_ADI_CHNL_PRI		(0x08)
#define REG_ADI_CHNL_PRIH		(0x08 + reg_offset)
#define BITS_PD_WR_PRI			((1) << 14 & (BIT(14)|BIT(15)))
#define BITS_RFT_WR_PRI			((1) << 12 & (BIT(12)|BIT(13)))
#define BITS_DSP_RD_PRI			((2) << 10 & (BIT(10)|BIT(11)))
#define BITS_DSP_WR_PRI			((2) << 8 & (BIT(8)|BIT(9)))
#define BITS_ARM_RD_PRI			((3) << 6 & (BIT(6)|BIT(7)))
#define BITS_ARM_WR_PRI			((3) << 4 & (BIT(4)|BIT(5)))
#define BITS_STC_WR_PRI			((1) << 2 & (BIT(2)|BIT(3)))
#define BITS_INT_STEAL_PRI		((3) << 0 & (BIT(0)|BIT(1)))
#define VALUE_CH_PRI			(0x0)

#define REG_ADI_INT_EN			(0x0C + reg_offset)
#define REG_ADI_INT_RAW			(0x10 + reg_offset)
#define REG_ADI_INT_MASK		(0x14 + reg_offset)
#define REG_ADI_INT_CLR			(0x18 + reg_offset)
#define REG_ADI_GSSI_CFG0		(0x1c + reg_offset)
#define BITS_CLK_ALL_ON			(BIT(30))
#define REG_ADI_GSSI_CFG1		(0x20 + reg_offset)
#define REG_ADI_RD_CMD			(0x24 + reg_offset)

/* bits definitions for register REG_ADI_RD_DATA */
#define REG_ADI_RD_DATA			(0x28 + reg_offset)
#define BIT_RD_CMD_BUSY			(BIT(31))
#define SHIFT_RD_ADDR			(16)
#define SHIFT_RD_VALU			(0)
#define MASK_RD_VALU			(0xFFFF)

/* bits definitions for register REG_ADI_ARM_FIFO_STS */
#define REG_ADI_ARM_FIFO_STS		(0x2c + reg_offset)
#define BIT_FIFO_FULL			(BIT(11))
#define BIT_FIFO_EMPTY			(BIT(10))

/* bits definitions for register of REG_ADI_STS */
#define REG_ADI_STS			(0x30 + reg_offset)
#define BITS_ADI_STS_CHN_VAL		(0xF)
#define BITS_ADI_REQ_VAL		(BIT(4))

#define REG_ADI_EVT_FIFO_STS		(0x34 + reg_offset)
#define REG_ADI_ARM_CMD_STS		(0x38 + reg_offset)
#define REG_ADI_CHNL_EN			(0x3c + reg_offset)
#define REG_ADI_CHNL_ADDR(id)		(0x40 + reg_offset + (id - 2) * 4)
#define REG_ADI_CHNL_EN1		(0x20c)

/* const value definition */
/* TODO: will be adapted by device tree */
#define ADI_SLAVE_ADDR_SIZE		(SZ_4K)
#define ADI_SLAVE_OFFSET		(0x8000)
#define FIFO_DRAIN_TIMEOUT		(1000)
#define ADI_READ_TIMEOUT		(2000)
/* TODO: will be adapted by device tree */
#define ADI_SPI_BUS_NUM			(5)
/* Timeout (ms) for the trylock of hardware spinlocks */
#define HWSPINLOCK_TIMEOUT		(5000)

#define HW_CHN_CNT			50

/* structure definition */
typedef enum ARM_WRITE_FIFO_STS {
	FIFO_EMPTY = 0,
	FIFO_FULL,
} write_fifo_sts_t;

typedef enum ADI_TRANS_REQ {
	NO_REQUEST_ONLINE = 0,
	REQUEST_ONLINE,
} adi_trans_req_t;

typedef struct ADI_STS {
	write_fifo_sts_t	write_fifo_sts;
	adi_trans_req_t		adi_trans_request;
	unsigned int		adi_chn_sel;
	unsigned int		arm_cmd_sts;
} adi_sts_t;

struct sprd_adi {
	struct spi_bitbang	bitbang;
	struct hwspinlock	*hw_lock;
	spinlock_t		spinlock;
	adi_sts_t		adi_sts;
	unsigned long		base;
	unsigned long		slave_vbase;
	resource_size_t		slave_pbase;
	u32			irq;
	const void		*tx;
	void			*rx;
	unsigned		rcount;
	unsigned		wcount;
	int	(*get_rx)(struct sprd_adi *, unsigned long, unsigned long *);
	int	(*get_tx)(struct sprd_adi *, unsigned long, unsigned long);
	unsigned long	(*trans_addr)(struct sprd_adi *, unsigned long);
};

/* ADI write type */
enum ADI_WRITE_MODE {
	SET_MODE = 0,
	FAST_SET_MODE,
};

static u32 readback_addr_mak;
static u32 readback_offset;
static u32 reg_offset;

/* get adi debug registers infor */
static void adi_get_sts(struct sprd_adi *sadi)
{
	unsigned int adi_sts;

	adi_sts = readl_relaxed((void __iomem *)(sadi->base + REG_ADI_STS));
	if (adi_sts >= 0) {
		sadi->adi_sts.adi_trans_request =
			(adi_sts & BITS_ADI_REQ_VAL) ?
			REQUEST_ONLINE : NO_REQUEST_ONLINE;
		sadi->adi_sts.adi_chn_sel = adi_sts & BITS_ADI_STS_CHN_VAL;
	}

	adi_sts = readl_relaxed((void __iomem *)
				(sadi->base + REG_ADI_ARM_CMD_STS));
	if (adi_sts >= 0)
		sadi->adi_sts.arm_cmd_sts = adi_sts;
}

/* print adi status information */
static void adi_debug(struct sprd_adi *sadi)
{
	adi_get_sts(sadi);
	pr_emerg("ADI occurs error, and print adi status information:\n"
		 "write fifo status = [0x%x], trans request sts = [0x%x],\n"
		 "channel select = [0x%x], arm command status= [0x%x].\n",
		 sadi->adi_sts.write_fifo_sts,
		 sadi->adi_sts.adi_trans_request,
		 sadi->adi_sts.adi_chn_sel,
		 sadi->adi_sts.arm_cmd_sts);
}

/* check the adi address */
int adi_addr_check(struct sprd_adi *sadi, unsigned long paddr)
{
	if (!sadi->slave_pbase || !sadi->slave_vbase
	    || paddr < sadi->slave_pbase
	    || paddr > (sadi->slave_pbase + ADI_SLAVE_ADDR_SIZE)) {
		pr_err("slave paddr is wrong, paddr = 0x%lx,\n"
		       "slave start paddr = 0x%lx,\n"
		       "slave end paddr = 0x%lx,\n"
		       "slave start vaddr = 0x%lx!\n", paddr,
		       (unsigned long)sadi->slave_pbase,
		       (unsigned long)(sadi->slave_pbase + ADI_SLAVE_ADDR_SIZE),
		       sadi->slave_vbase);
		return -1;
	}

	return 0;
}

/* translate the AP virtual address to A-Die physical address */
static inline unsigned long
adi_paddr_to_vaddr(struct sprd_adi *sadi, unsigned long reg_paddr)
{
	return (reg_paddr - sadi->slave_pbase + sadi->slave_vbase);
}

/* check if write fifo is empty */
static int adi_fifo_drain(struct sprd_adi *sadi)
{
	int timeout = FIFO_DRAIN_TIMEOUT;

	while (!(readl_relaxed((void __iomem *)
			       (sadi->base + REG_ADI_ARM_FIFO_STS))
				& BIT_FIFO_EMPTY) && --timeout) {
		udelay(1);
	}

	if (timeout == 0) {
		pr_err("ADI fifo drain wait timeout!!!\n");
		return -EBUSY;
	}

	sadi->adi_sts.write_fifo_sts = FIFO_EMPTY;
	return 0;
}

/* check if write fifo is full */
static inline int adi_fifo_full(struct sprd_adi *sadi)
{
	int val = -EBUSY;

	val = readl_relaxed((void __iomem *)
			    (sadi->base + REG_ADI_ARM_FIFO_STS)) & BIT_FIFO_FULL;
	if (val > 0)
		sadi->adi_sts.write_fifo_sts = FIFO_FULL;

	return val;
}

/* read value from adi register */
static int adi_read(struct sprd_adi *sadi, unsigned long reg_pddr,
		    unsigned long *read_val)
{
	unsigned long val, rd_addr;
	int read_timeout = ADI_READ_TIMEOUT;

	/*
	 * We don't wait write fifo empty in here,
	 * Because if normal write is SYNC, that will
	 * wait fifo empty at the end of the write.
	 */
	writel_relaxed((u32)reg_pddr, (void __iomem *)
		       (sadi->base + REG_ADI_RD_CMD));

	/*
	 * wait read operation complete, RD_data[31]
	 * is set simulaneously when writing read command.
	 * RD_data[31] will be cleared after the read operation complete,
	 */
	do {
		val = readl_relaxed((void __iomem *)
				    (sadi->base + REG_ADI_RD_DATA));
	} while ((val & BIT_RD_CMD_BUSY) && --read_timeout);

	if (read_timeout == 0) {
		pr_err("ADI read timeout!!!\n");
		adi_debug(sadi);
		return -EIO;
	}

	/* check if the read addr is correct */
	rd_addr = (val >> SHIFT_RD_ADDR) & readback_addr_mak;
	/* The return address range is BIT[2]-BIT[16] for r3p0 */
	if (rd_addr != ((reg_pddr & readback_addr_mak) >> readback_offset)) {
		pr_err("Read error:val = 0x%lx, regPaddr = 0x%lx,\n"
		       "readback_addr_mak = 0x%x!\n",
		       val, reg_pddr, readback_addr_mak);
		adi_debug(sadi);
		return -EIO;
	}

	/* val high part should be the address of the last read operation */
	if (read_val) {
		*read_val = val & MASK_RD_VALU;
	} else {
		pr_err("Read error: read buf is NULL!\n");
		return -EIO;
	}

	return 0;
}

/* write value to ADI registers */
static int adi_write(struct sprd_adi *sadi, unsigned long reg,
		     unsigned long val)
{
	int ret;

	/* check if write fifo is empty */
	ret = adi_fifo_drain(sadi);
	if (ret < 0) {
		adi_debug(sadi);
		return ret;
	}

	/* check if write fifo is full */
	if (!adi_fifo_full(sadi)) {
		writel_relaxed((u32)val, (void __iomem *)reg);
	} else {
		pr_err("Warning: ADI write fifo full,waiting...\n");
		adi_debug(sadi);

		while (adi_fifo_full(sadi))
			cpu_relax();

		writel_relaxed((u32)val, (void __iomem *)reg);
	}

	return 0;
}

/* spi transfer function */
int sprd_adi_bufs(struct spi_device *adi_dev, struct spi_transfer *t)
{
	struct sprd_adi *sadi = spi_master_get_devdata(adi_dev->master);
	unsigned long reg, phy_reg, fast_flag, val, flags;
	unsigned long clear_msk, or_val;
	int ret;

	/* adi read */
	if (t->rx_buf) {
		/* get the read adi reg */
		phy_reg = *(unsigned long *)t->rx_buf + sadi->slave_pbase;
		if (adi_addr_check(sadi, phy_reg)) {
			pr_err("ADI read error, no read register!\n");
			return -EIO;
		}

		if (sadi->hw_lock) {
			ret = hwspin_lock_timeout_irqsave(sadi->hw_lock,
							  HWSPINLOCK_TIMEOUT,
							  &flags);
			if (ret) {
				pr_err("Read: lock the hw lock failed.\n");
				return ret;
			}
		} else {
			spin_lock_irqsave(&sadi->spinlock, flags);
		}

		ret = sadi->get_rx(sadi, phy_reg, &val);

		if (sadi->hw_lock)
			hwspin_unlock_irqrestore(sadi->hw_lock, &flags);
		else
			spin_unlock_irqrestore(&sadi->spinlock, flags);

		if (ret) {
			pr_err("ADI read error: reg = 0x%lx\n", phy_reg);
			return -EIO;
		}

		*(unsigned long *)t->rx_buf = val;
		ret = t->len;
	} else if (t->tx_buf) { /* adi write */
		unsigned long *p = (unsigned long *)t->tx_buf;

		/* get the write adi reg and translate it */
		phy_reg = *p++ + sadi->slave_pbase;
		if (adi_addr_check(sadi, phy_reg)) {
			pr_err("ADI write error, reg = 0x%lx\n", phy_reg);
			return -EIO;
		}
		reg = sadi->trans_addr(sadi, phy_reg);

		/* get the write type */
		fast_flag = *p++;
		if (fast_flag == FAST_SET_MODE) {
			val = *p;

			if (sadi->hw_lock) {
				ret = hwspin_lock_timeout_irqsave(sadi->hw_lock,
								  HWSPINLOCK_TIMEOUT,
								  &flags);
				if (ret) {
					pr_err("Fast write:lock hw spinlock failed.\n");
					return ret;
				}
			} else {
				spin_lock_irqsave(&sadi->spinlock, flags);
			}

			ret = sadi->get_tx(sadi, reg, val);

			if (sadi->hw_lock)
				hwspin_unlock_irqrestore(sadi->hw_lock, &flags);
			else
				spin_unlock_irqrestore(&sadi->spinlock, flags);

			if (ret) {
				pr_err("Fast write error: reg = 0x%lx\n", reg);
				return -EIO;
			}
		} else if (fast_flag == SET_MODE) {
			or_val = *p++;
			clear_msk = *p;

			if (sadi->hw_lock) {
				ret = hwspin_lock_timeout_irqsave(sadi->hw_lock,
								  HWSPINLOCK_TIMEOUT,
								  &flags);
				if (ret) {
					pr_err("Write: lock the hw lock failed.\n");
					return ret;
				}
			} else {
				spin_lock_irqsave(&sadi->spinlock, flags);
			}

			ret = sadi->get_rx(sadi, phy_reg, &val);
			if (!ret)
				ret = sadi->get_tx(sadi, reg,
						   (val & ~clear_msk) | or_val);

			if (sadi->hw_lock)
				hwspin_unlock_irqrestore(sadi->hw_lock, &flags);
			else
				spin_unlock_irqrestore(&sadi->spinlock, flags);

			if (ret) {
				pr_err("ADI write error: reg = 0x%lx\n", reg);
				return -EIO;
			}
		} else {
			pr_err("ADI write type error:%ld!\n", fast_flag);
			return -EIO;
		}

		ret = t->len;
	} else {
		pr_err("ADI data transfer error!\n");
		ret = -EIO;
	}

	return ret;
}

static int sprd_adi_setup_transfer(struct spi_device *adi_dev,
				   struct spi_transfer *t)
{
	/* There is nothing to implement for transfer setup,
	 * the ADI transter doesn't need any other config, just return.
	 */
	return 0;
}

static int sprd_adi_setup(struct spi_device *adi_dev)
{
	/* There is nothing to implement for adi device setup,
	 * cause the adi device has been configured when probe
	 * the device, here just return.
	 */
	return 0;
}

static void sprd_adi_chipselect(struct spi_device *adi_dev, int value)
{
	/* There is nothing to implement for chip select,
	 * ADI no need to select the chip, here is NULL.
	 */
}

static void sprd_adi_hwchannel_setting(struct platform_device *pdev,
				       struct sprd_adi *sadi)
{
	int i, size, chn_cnt;
	const __be32 *list;

	/* set hw channel */
	list = of_get_property(pdev->dev.of_node, "sprd,hw-channels", &size);
	if (!size || !list) {
		pr_warn("no hw channels property in node\n");
		return;
	}

	chn_cnt = size / 8;
	for (i = 0; i < chn_cnt; i++) {
		u32 value;
		u32 chn_id = be32_to_cpu(*list++);
		u32 chn_config = be32_to_cpu(*list++);

		if (chn_id < 2)
			continue;

		writel_relaxed(chn_config, (void __iomem *)
				(sadi->base + REG_ADI_CHNL_ADDR(chn_id)));

		if (chn_id < 31) {
			value = readl_relaxed((void __iomem *)
				(sadi->base + REG_ADI_CHNL_EN));
			value |= BIT(chn_id);
			writel_relaxed(value, (void __iomem *)
				(sadi->base + REG_ADI_CHNL_EN));
		} else if (chn_id < HW_CHN_CNT) {
			value = readl_relaxed((void __iomem *)
				(sadi->base + REG_ADI_CHNL_EN1));
			value |= BIT(chn_id - 32);
			writel_relaxed(value, (void __iomem *)
				(sadi->base + REG_ADI_CHNL_EN1));
		}
	}
}

static int sprd_adi_probe(struct platform_device *pdev)
{
	struct spi_master *adi_master;
	struct sprd_adi *sadi;
	unsigned int value, chip_num;
	struct resource *res;
	unsigned long sprd_adi_phys, sprd_adi_base;
	int ret = 0;

	if (!pdev->dev.of_node) {
		pr_err("Can't find the adi bus node!\n");
		return -ENODEV;
	}

	chip_num = of_get_child_count(pdev->dev.of_node);
	if (chip_num == 0) {
		pr_err("Can't get chip number infor!\n");
		return -EIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sprd_adi_base = (unsigned long)devm_ioremap_resource(&pdev->dev, res);
	if (!sprd_adi_base) {
		pr_err("ADI ioremap failed!\n");
		return -ENOMEM;
	}
	sprd_adi_phys = res->start;

	/* alloc spi_master */
	adi_master = spi_alloc_master(&pdev->dev, sizeof(struct sprd_adi));
	if (!adi_master) {
		pr_err("Spi master allocate memory failed!\n");
		return -ENOMEM;
	}

	dev_set_drvdata(&pdev->dev, adi_master);

	/* initialize the spi_master */
	adi_master->dev.of_node = pdev->dev.of_node;
	adi_master->bus_num = ADI_SPI_BUS_NUM;
	/* chip select number according to dt configuration */
	adi_master->num_chipselect = chip_num;
	adi_master->setup = sprd_adi_setup;
	adi_master->flags = SPI_MASTER_HALF_DUPLEX;
	adi_master->bits_per_word_mask = 0;
	adi_master->no_idle = true;
	/* default is spi_bitbang_transfer */
	/* adi_master->transfer = spi_bitbang_transfer; */

	/* initialize the sprd_adi structure */
	sadi = spi_master_get_devdata(adi_master);

	spin_lock_init(&sadi->spinlock);
	if (!of_device_is_compatible(pdev->dev.of_node, "sprd,r4p0-adi")) {
		sadi->hw_lock = hwspin_lock_request_specific(HWLOCK_ADI);
		if (!sadi->hw_lock) {
			pr_err("ADI can not get the hardware spinlock.\n");
			ret = -ENXIO;
			goto master_alloc_fail;
		}
	}

	sadi->base = sprd_adi_base;
	sadi->slave_vbase = sprd_adi_base + ADI_SLAVE_OFFSET;
	sadi->slave_pbase = sprd_adi_phys + ADI_SLAVE_OFFSET;
	sadi->adi_sts.write_fifo_sts = FIFO_EMPTY;
	sadi->adi_sts.adi_trans_request = NO_REQUEST_ONLINE;
	sadi->adi_sts.adi_chn_sel = 0;
	sadi->adi_sts.arm_cmd_sts = 0;
	sadi->get_rx = adi_read;
	sadi->get_tx = adi_write;
	sadi->trans_addr = adi_paddr_to_vaddr;
	sadi->bitbang.master = spi_master_get(adi_master);
	if (!sadi->bitbang.master) {
		ret = -ENODEV;
		goto master_alloc_fail;
	}

	sadi->bitbang.txrx_bufs = sprd_adi_bufs;
	sadi->bitbang.setup_transfer = sprd_adi_setup_transfer;
	sadi->bitbang.chipselect = sprd_adi_chipselect;

	/* initialize the adi registers */
	value = readl_relaxed((void __iomem *)(sadi->base + REG_ADI_CTRL0));
	/* low level trigger, and word addr transfer */
	if (value) {
		pr_warn("ADI CTRL0 default error: 0x%x!\n", value);
		writel_relaxed(0, (void __iomem *)
			       (sadi->base + REG_ADI_CTRL0));
	}

	if (of_device_is_compatible(pdev->dev.of_node, "sprd,sc9838-adi") ||
	    of_device_is_compatible(pdev->dev.of_node, "sprd,r1p0-adi") ||
	    of_device_is_compatible(pdev->dev.of_node, "sprd,r3p0-adi") ||
	    of_device_is_compatible(pdev->dev.of_node, "sprd,r4p0-adi")) {
		/* for version r0p1 registers' map */
		reg_offset = 0x4;

		/* set the channel priority */
		value = VALUE_CH_PRI;
		writel_relaxed(value, (void __iomem *)
				(sadi->base + REG_ADI_CHNL_PRIH));
	} else {
		reg_offset = 0x0;
	}

	sprd_adi_hwchannel_setting(pdev, sadi);

	/* set the channel priority */
	value = VALUE_CH_PRI;
	writel_relaxed(value, (void __iomem *)(sadi->base + REG_ADI_CHNL_PRI));

	/* get the readback addr mask */
	value = readl_relaxed((void __iomem *)(sadi->base + REG_ADI_GSSI_CFG0));
	/* addr_len = frame_len - data_len - flag_len*/
	readback_addr_mak = (value & 0x3f) - ((value >> 11) & 0x1f) - 1;
	readback_addr_mak = (1 << (readback_addr_mak + 2)) - 1;
	readback_offset = 0;

	if (of_device_is_compatible(pdev->dev.of_node, "sprd,r3p0-adi") ||
	    of_device_is_compatible(pdev->dev.of_node, "sprd,r4p0-adi")) {
		readback_addr_mak = 0xffff;
		readback_offset = 2;
	}

	/* set ADI clock auto gate */
	value &= ~BITS_CLK_ALL_ON;
	writel_relaxed(value, (void __iomem *)(sadi->base + REG_ADI_GSSI_CFG0));

	ret = spi_bitbang_start(&sadi->bitbang);
	if (ret)
		goto put_master;

	pr_info("ADI master probe ok!\n");
	return ret;

put_master:
	spi_master_put(adi_master);
master_alloc_fail:
	kfree(adi_master);
	pr_err("ADI bus probe failed ret=%d!\n", ret);
	return ret;
}

static int sprd_adi_remove(struct platform_device *pdev)
{
	struct spi_master *adi_master = dev_get_drvdata(&pdev->dev);
	struct sprd_adi *sadi = spi_master_get_devdata(adi_master);

	spi_bitbang_stop(&sadi->bitbang);
	hwspin_lock_free(sadi->hw_lock);
	spi_master_put(adi_master);
	kfree(adi_master);
	return 0;
}

static const struct of_device_id sprd_adi_of_match[] = {
	{
		.compatible = "sprd,sc9838-adi",
	},
	{
		.compatible = "sprd,sc9830-adi",
	},
	{
		.compatible = "sprd,r1p0-adi",
	},
	{
		.compatible = "sprd,r3p0-adi",
	},
	{
		.compatible = "sprd,r4p0-adi",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, sprd_adi_of_match);

static struct platform_driver sprd_adi_driver = {
	.driver = {
		.name = "adi-bus",
		.owner = THIS_MODULE,
		.of_match_table = sprd_adi_of_match,
	},
	.probe = sprd_adi_probe,
	.remove = sprd_adi_remove,
};

static int __init sprd_adi_init(void)
{
	return platform_driver_register(&sprd_adi_driver);
}

static void __exit sprd_adi_exit(void)
{
	platform_driver_unregister(&sprd_adi_driver);
}

subsys_initcall(sprd_adi_init);
module_exit(sprd_adi_exit);

MODULE_DESCRIPTION("SPREADTRUM ADI Master Controller Driver");
MODULE_AUTHOR("Baolin.Wang <Baolin.Wang@spreadtrum.com>");
MODULE_LICENSE("GPL");
