/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define TMC_RSZ			0x004
#define TMC_STS			0x00c
#define TMC_RRD			0x010
#define TMC_RRP			0x014
#define TMC_RWP			0x018
#define TMC_TRG			0x01c
#define TMC_CTL			0x020
#define TMC_RWD			0x024
#define TMC_MODE		0x028
#define TMC_LBUFLEVEL		0x02c
#define TMC_CBUFLEVEL		0x030
#define TMC_BUFWM		0x034
#define TMC_RRPHI		0x038
#define TMC_RWPHI		0x03c
#define TMC_AXICTL		0x110
#define TMC_DBALO		0x118
#define TMC_DBAHI		0x11c
#define TMC_FFSR		0x300
#define TMC_FFCR		0x304
#define TMC_PSCR		0x308
#define TMC_ITMISCOP0		0xee0
#define TMC_ITTRFLIN		0xee8
#define TMC_ITATBDATA0		0xeec
#define TMC_ITATBCTR2		0xef0
#define TMC_ITATBCTR1		0xef4
#define TMC_ITATBCTR0		0xef8

/*
 * tmc management registers (0xf00-0xfcc)
 * 0xfa0 - 0xfa4: Management	registers in PFTv1.0
 *		  Trace		registers in PFTv1.1
 */
#define TMC_ITCTRL		0xf00
#define TMC_CLAIMSET		0xfa0
#define TMC_CLAIMCLR		0xfa4
#define TMC_LAR			0xfb0
#define TMC_LSR			0xfb4
#define TMC_AUTHSTATUS		0xfb8
#define TMC_DEVID		0xfc8
#define TMC_DEVTYPE		0xfcc

/* register description */
/* TMC_CTL - 0x020 */
#define TMC_CTL_CAPT_EN		BIT(0)
/* TMC_STS - 0x00C */
#define TMC_STS_TRIGGERED	BIT(1)
/* TMC_AXICTL - 0x110 */
#define TMC_AXICTL_PROT_CTL_B0	BIT(0)
#define TMC_AXICTL_PROT_CTL_B1	BIT(1)
#define TMC_AXICTL_SCT_GAT_MODE	BIT(7)
#define TMC_AXICTL_WR_BURST_LEN 0xf00
/* TMC_FFCR - 0x304 */
#define TMC_FFCR_EN_FMT		BIT(0)
#define TMC_FFCR_EN_TI		BIT(1)
#define TMC_FFCR_FON_FLIN	BIT(4)
#define TMC_FFCR_FON_TRIG_EVT	BIT(5)
#define TMC_FFCR_FLUSHMAN	BIT(6)
#define TMC_FFCR_TRIGON_TRIGIN	BIT(8)
#define TMC_FFCR_STOP_ON_FLUSH	BIT(12)

#define TMC_STS_TRIGGERED_BIT	2
#define TMC_FFCR_FLUSHMAN_BIT	6
#define TMC_SYS_UNLOCK		0xc5acce55

#define TIMEOUT_US		100
#define BMVAL(val, lsb, msb)	((val & GENMASK(msb, lsb)) >> lsb)
#define TMC_DATA_MASK_VAL	GENMASK(31, 0)

enum tmc_config_type {
	TMC_CONFIG_TYPE_ETB,
	TMC_CONFIG_TYPE_ETR,
	TMC_CONFIG_TYPE_ETF,
};

enum tmc_mode {
	TMC_MODE_CIRCULAR_BUFFER,
	TMC_MODE_SOFTWARE_FIFO,
	TMC_MODE_HARDWARE_FIFO,
};

enum tmc_mem_intf_width {
	TMC_MEM_INTF_WIDTH_32BITS	= 0x2,
	TMC_MEM_INTF_WIDTH_64BITS	= 0x3,
	TMC_MEM_INTF_WIDTH_128BITS	= 0x4,
	TMC_MEM_INTF_WIDTH_256BITS	= 0x5,
};

/**
 * struct tmc_drvdata - specifics associated to an TMC component
 * @base:	memory mapped base address for this component.
 * @dev:	the device entity associated to this component.
 * @miscdev:	specifics to handle "/dev/xyz.tmc" entry.
 * @spinlock:	only one at a time pls.
 * @read_count:	manages preparation of buffer for reading.
 * @reading:	the device in reading operation status.
 * @buf:	area of memory where trace data get sent.
 * @size:	@buf size.
 * @tmc_mode: TMC mode, must be of type @tmc_mode.
 */
struct tmc_drvdata {
	void __iomem		*base;
	struct device		*dev;
	struct miscdevice	miscdev;
	spinlock_t		spinlock;
	int			read_count;
	bool			reading;
	char			*buf;
	u32			size;
	enum tmc_mode		tmc_mode;
};

static inline void tmc_lock(void __iomem *addr)
{
	writel_relaxed(0x0, addr + TMC_LAR);
}

static inline void tmc_unlock(void __iomem *addr)
{
	writel_relaxed(TMC_SYS_UNLOCK, addr + TMC_LAR);
}

/**
 * tmc_timeout - loop until a bit has changed to a specific state.
 * @addr: base address of the area of interest.
 * @offset: address of a register, starting from @addr.
 * @position: the position of the bit of interest.
 * @value: the value the bit should have.
 *
 * Return: 0 as soon as the bit has taken the desired state or -EAGAIN if
 * TIMEOUT_US has elapsed, which ever happens first.
 */

static int tmc_timeout(void __iomem *addr, u32 offset, int position, int value)
{
	int i;
	u32 val;

	for (i = TIMEOUT_US; i > 0; i--) {
		val = readl_relaxed(addr + offset);
		/* waiting on the bit to go from 0 to 1 */
		if (value) {
			if (val & BIT(position))
				return 0;
		/* waiting on the bit to go from 1 to 0 */
		} else {
			if (!(val & BIT(position)))
				return 0;
		}

		/*
		 * Delay is arbitrary - the specification doesn't say how long
		 * we are expected to wait.  Extra check required to make sure
		 * we don't wait needlessly on the last iteration.
		 */
		if (i - 1)
			udelay(1);
	}

	return -EAGAIN;
}

static void tmc_wait_for_ready(struct tmc_drvdata *drvdata)
{
	/* Ensure formatter, unformatter and hardware fifo are empty */
	if (tmc_timeout(drvdata->base,
			      TMC_STS, TMC_STS_TRIGGERED_BIT, 1)) {
		dev_err(drvdata->dev,
			"timeout observed when probing at offset %#x\n",
			TMC_STS);
	}
}

static void tmc_flush_and_stop(struct tmc_drvdata *drvdata)
{
	u32 ffcr;

	ffcr = readl_relaxed(drvdata->base + TMC_FFCR);
	ffcr |= TMC_FFCR_STOP_ON_FLUSH;
	writel_relaxed(ffcr, drvdata->base + TMC_FFCR);
	ffcr |= TMC_FFCR_FLUSHMAN;
	writel_relaxed(ffcr, drvdata->base + TMC_FFCR);
	/* Ensure flush completes */
	if (tmc_timeout(drvdata->base,
			      TMC_FFCR, TMC_FFCR_FLUSHMAN_BIT, 0)) {
		dev_err(drvdata->dev,
			"timeout observed when probing at offset %#x\n",
			TMC_FFCR);
	}

	tmc_wait_for_ready(drvdata);
}

static void tmc_enable_hw(struct tmc_drvdata *drvdata)
{
	writel_relaxed(TMC_CTL_CAPT_EN, drvdata->base + TMC_CTL);
}

static void tmc_disable_hw(struct tmc_drvdata *drvdata)
{
	writel_relaxed(0x0, drvdata->base + TMC_CTL);
}

static void tmc_etb_dump_hw(struct tmc_drvdata *drvdata)
{
	enum tmc_mem_intf_width memwidth;
	u8 memwords;
	char *bufp;
	u32 read_data;
	int i;

	memwidth = BMVAL(readl_relaxed(drvdata->base + TMC_DEVID), 8, 10);
	switch (memwidth) {
	case TMC_MEM_INTF_WIDTH_32BITS:
		memwords = 1;
		break;
	case TMC_MEM_INTF_WIDTH_64BITS:
		memwords = 2;
		break;
	case TMC_MEM_INTF_WIDTH_128BITS:
		memwords = 4;
		break;
	default:
		memwords = 8;
	}

	bufp = drvdata->buf;
	while (1) {
		for (i = 0; i < memwords; i++) {
			read_data = readl_relaxed(drvdata->base + TMC_RRD);
			if (read_data == TMC_DATA_MASK_VAL)
				return;
			memcpy(bufp, &read_data, 4);
			bufp += 4;
		}
	}
}

static bool tmc_check_ctrl_enable(struct tmc_drvdata *drvdata)
{
	if (readl_relaxed(drvdata->base + TMC_CTL) & TMC_CTL_CAPT_EN)
		return true;
	return false;
}

static void tmc_dump_data_for_cp(struct tmc_drvdata *drvdata)
{
	/* Zero out the memory to help with debug */
	memset(drvdata->buf, 0, drvdata->size);

	tmc_unlock(drvdata->base);
	tmc_flush_and_stop(drvdata);
	tmc_etb_dump_hw(drvdata);
	tmc_disable_hw(drvdata);
	writel_relaxed(TMC_FFCR_EN_FMT | TMC_FFCR_EN_TI |
			   TMC_FFCR_FON_FLIN | TMC_FFCR_FON_TRIG_EVT |
			   TMC_FFCR_TRIGON_TRIGIN,
			   drvdata->base + TMC_FFCR);
	tmc_enable_hw(drvdata);
	tmc_lock(drvdata->base);
}

static int tmc_read_prepare(struct tmc_drvdata *drvdata)
{
	unsigned long flags;

	spin_lock_irqsave(&drvdata->spinlock, flags);
	/*
	 * ETB in AON power domain, all the sys could use it,
	 * but when CP block, user space want to get ETB data by /dev/tmc-etb.
	 * At that time, ETB is enabled by CP sys, but enable flag is false.
	 */
	if (tmc_check_ctrl_enable(drvdata))
		tmc_dump_data_for_cp(drvdata);

	drvdata->reading = true;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);

	dev_info(drvdata->dev, "TMC read start\n");
	return 0;
}

static void tmc_read_unprepare(struct tmc_drvdata *drvdata)
{
	unsigned long flags;

	spin_lock_irqsave(&drvdata->spinlock, flags);
	drvdata->reading = false;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);

	dev_info(drvdata->dev, "TMC read end\n");
}

static int tmc_open(struct inode *inode, struct file *file)
{
	struct tmc_drvdata *drvdata = container_of(file->private_data,
						   struct tmc_drvdata, miscdev);
	int ret;

	if (drvdata->read_count++)
		goto out;

	ret = tmc_read_prepare(drvdata);
	if (ret)
		return ret;
out:
	nonseekable_open(inode, file);

	dev_dbg(drvdata->dev, "%s: successfully opened\n", __func__);
	return 0;
}

static ssize_t tmc_read(struct file *file, char __user *data, size_t len,
			loff_t *ppos)
{
	struct tmc_drvdata *drvdata = container_of(file->private_data,
						   struct tmc_drvdata, miscdev);
	char *bufp = drvdata->buf + *ppos;

	if (!tmc_check_ctrl_enable(drvdata))
		return -EFAULT;
	if (*ppos + len > drvdata->size)
		len = drvdata->size - *ppos;

	if (copy_to_user(data, bufp, len)) {
		dev_dbg(drvdata->dev, "%s: copy_to_user failed\n", __func__);
		return -EFAULT;
	}

	*ppos += len;

	dev_dbg(drvdata->dev, "%s: %zu bytes copied, %d bytes left\n",
		__func__, len, (int)(drvdata->size - *ppos));
	return len;
}

static int tmc_release(struct inode *inode, struct file *file)
{
	struct tmc_drvdata *drvdata = container_of(file->private_data,
						   struct tmc_drvdata, miscdev);

	if (--drvdata->read_count) {
		if (drvdata->read_count < 0) {
			dev_err(drvdata->dev, "mismatched close\n");
			drvdata->read_count = 0;
		}
		goto out;
	}

	tmc_read_unprepare(drvdata);
out:
	dev_dbg(drvdata->dev, "%s: released\n", __func__);
	return 0;
}

static const struct file_operations tmc_fops = {
	.owner		= THIS_MODULE,
	.open		= tmc_open,
	.read		= tmc_read,
	.release	= tmc_release,
	.llseek		= no_llseek,
};

static ssize_t status_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	u32 tmc_rsz, tmc_sts, tmc_rrp, tmc_rwp, tmc_trg;
	u32 tmc_ctl, tmc_ffsr, tmc_ffcr, tmc_mode, tmc_pscr;
	u32 devid;
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);

	spin_lock_irqsave(&drvdata->spinlock, flags);
	tmc_unlock(drvdata->base);
	tmc_rsz = readl_relaxed(drvdata->base + TMC_RSZ);
	tmc_sts = readl_relaxed(drvdata->base + TMC_STS);
	tmc_rrp = readl_relaxed(drvdata->base + TMC_RRP);
	tmc_rwp = readl_relaxed(drvdata->base + TMC_RWP);
	tmc_trg = readl_relaxed(drvdata->base + TMC_TRG);
	tmc_ctl = readl_relaxed(drvdata->base + TMC_CTL);
	tmc_ffsr = readl_relaxed(drvdata->base + TMC_FFSR);
	tmc_ffcr = readl_relaxed(drvdata->base + TMC_FFCR);
	tmc_mode = readl_relaxed(drvdata->base + TMC_MODE);
	tmc_pscr = readl_relaxed(drvdata->base + TMC_PSCR);
	devid = readl_relaxed(drvdata->base + TMC_DEVID);

	tmc_lock(drvdata->base);
	spin_unlock_irqrestore(&drvdata->spinlock, flags);

	return sprintf(buf,
		       "Depth:\t\t0x%x\n"
		       "Status:\t\t0x%x\n"
		       "RAM read ptr:\t0x%x\n"
		       "RAM wrt ptr:\t0x%x\n"
		       "Trigger cnt:\t0x%x\n"
		       "Control:\t0x%x\n"
		       "Flush status:\t0x%x\n"
		       "Flush ctrl:\t0x%x\n"
		       "Mode:\t\t0x%x\n"
		       "PSRC:\t\t0x%x\n"
		       "DEVID:\t\t0x%x\n",
			tmc_rsz, tmc_sts, tmc_rrp, tmc_rwp, tmc_trg,
			tmc_ctl, tmc_ffsr, tmc_ffcr, tmc_mode, tmc_pscr, devid);
}
static DEVICE_ATTR_RO(status);

static struct attribute *tmc_etb_attrs[] = {
	&dev_attr_status.attr,
	NULL,
};
ATTRIBUTE_GROUPS(tmc_etb);

static int tmc_probe(struct platform_device *pdev)
{
	int ret;
	void __iomem *base;
	struct resource *res;
	struct tmc_drvdata *drvdata;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EBUSY;

	base = devm_ioremap_nocache(&pdev->dev, res->start,
					resource_size(res));
	if (!base)
		return -ENOMEM;

	drvdata->base = base;

	spin_lock_init(&drvdata->spinlock);

	ret = of_property_read_u32(pdev->dev.of_node, "arm,buffer-size",
				   &drvdata->size);
	if (ret)
		return ret;

	drvdata->buf = devm_kzalloc(&pdev->dev, drvdata->size, GFP_KERNEL);
	if (!drvdata->buf)
		return -ENOMEM;

	drvdata->miscdev.parent = &pdev->dev;
	drvdata->miscdev.name = pdev->name;
	drvdata->miscdev.minor = MISC_DYNAMIC_MINOR;
	drvdata->miscdev.fops = &tmc_fops;
	drvdata->miscdev.groups = tmc_etb_groups;
	ret = misc_register(&drvdata->miscdev);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, drvdata);
	drvdata = dev_get_drvdata(&pdev->dev);

	dev_info(&pdev->dev, "SPRD TMC initialized\n");
	return 0;
}

static int tmc_remove(struct platform_device *pdev)
{
	struct tmc_drvdata *drvdata = platform_get_drvdata(pdev);

	misc_deregister(&drvdata->miscdev);

	return 0;
}

static const struct of_device_id dmc_mpu_of_match[] = {
	{ .compatible = "sprd,tmc-etb"},
	{ },
};

static struct platform_driver tmc_driver = {
	.probe = tmc_probe,
	.remove = tmc_remove,
	.driver = {
		.name = "sprd-tmc",
		.owner = THIS_MODULE,
		.of_match_table = dmc_mpu_of_match,
	},
};

static int __init tmc_init(void)
{
	return platform_driver_register(&tmc_driver);
}

static void __exit tmc_exit(void)
{
	platform_driver_unregister(&tmc_driver);
}

module_init(tmc_init);
module_exit(tmc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CoreSight Trace Memory Controller driver");
