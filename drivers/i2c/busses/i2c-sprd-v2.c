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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#define I2C_CTL				(0x000)
#define I2C_ADDR_CFG		(0x004)
#define I2C_COUNT			(0x008)
#define I2C_RX				(0x00C)
#define I2C_TX				(0x010)
#define I2C_STATUS			(0x014)
#define I2C_HSMODE_CFG		(0x018)
#define I2C_VERSION			(0x01C)
#define ADDR_DVD0			(0x020)
#define ADDR_DVD1			(0x024)
#define ADDR_STA0_DVD		(0x028)
#define ADDR_RST			(0x02C)

/* I2C_CTL */
#define STP_EN				BIT(20)
#define FIFO_AF_LVL			(16)
#define FIFO_AE_LVL			(12)
#define I2C_DMA_EN			BIT(11)
#define FULL_INTEN			BIT(10)
#define EMPTY_INTEN			BIT(9)
#define I2C_DVD_OPT			BIT(8)
#define I2C_OUT_OPT			BIT(7)
#define I2C_TRIM_OPT		BIT(6)
#define I2C_HS_MODE			BIT(4)
#define I2C_MODE			BIT(3)
#define I2C_EN				BIT(2)
#define I2C_INT_EN			BIT(1)
#define I2C_START			BIT(0)

/* I2C_STATUS */
#define SDA_IN				BIT(21)
#define SCL_IN				BIT(20)
#define FIFO_FULL			BIT(4)
#define FIFO_EMPTY			BIT(3)
#define I2C_INT				BIT(2)
#define I2C_RX_ACK			BIT(1)
#define I2C_BUSY				BIT(0)

/* ADDR_RST */
#define I2C_RST				BIT(0)

#define I2C_FIFO_DEEP				12
#define I2C_FIFO_FULL_THLD			15
#define I2C_FIFO_EMPTY_THLD		4
#define I2C_DATA_STEP				8
#define SPRD_I2C_TIMEOUT			(msecs_to_jiffies(1000))

/* timeout for pm runtime autosuspend */
#define SPRD_I2C_PM_TIMEOUT		1000	/* ms */

/* i2c data structure */
struct sprd_i2c {
	struct i2c_msg *msg;
	struct i2c_adapter adap;
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	unsigned int src_clk;
	int irq;
	unsigned int bus_freq;
	__u8 *buf;
	__u16 count;
	int err;
	bool is_suspended;
	struct completion complete;
};

static void
sprd_i2c_dump_reg(struct sprd_i2c *pi2c)
{
	dev_err(&pi2c->adap.dev,
		": ======dump i2c-%d reg=======\n", pi2c->adap.nr);
	dev_err(&pi2c->adap.dev, ": I2C_CTRL:0x%x\n",
		readl_relaxed(pi2c->base + I2C_CTL));
	dev_err(&pi2c->adap.dev, ": I2C_ADDR_CFG:0x%x\n",
		readl_relaxed(pi2c->base + I2C_ADDR_CFG));
	dev_err(&pi2c->adap.dev, ": I2C_COUNT:0x%x\n",
		readl_relaxed(pi2c->base + I2C_COUNT));
	dev_err(&pi2c->adap.dev, ": I2C_RX:0x%x\n",
		readl_relaxed(pi2c->base + I2C_RX));
	dev_err(&pi2c->adap.dev, ": I2C_STATUS:0x%x\n",
		readl_relaxed(pi2c->base + I2C_STATUS));
	dev_err(&pi2c->adap.dev, ": ADDR_DVD0:0x%x\n",
		readl_relaxed(pi2c->base + ADDR_DVD0));
	dev_err(&pi2c->adap.dev, ": ADDR_DVD1:0x%x\n",
		readl_relaxed(pi2c->base + ADDR_DVD1));
	dev_err(&pi2c->adap.dev, ": ADDR_STA0_DVD:0x%x\n",
		readl_relaxed(pi2c->base + ADDR_STA0_DVD));
	dev_err(&pi2c->adap.dev, ": ADDR_RST:0x%x\n",
		readl_relaxed(pi2c->base + ADDR_RST));
}

static void sprd_i2c_set_count(struct sprd_i2c *pi2c, u32 count)
{
	writel_relaxed(count, pi2c->base + I2C_COUNT);
}

static int sprd_i2c_send_stop(struct sprd_i2c *pi2c, int stop)
{
	unsigned int tmp = readl_relaxed(pi2c->base + I2C_CTL);

	if (stop)
		writel_relaxed(tmp & (~STP_EN), pi2c->base + I2C_CTL);
	else
		writel_relaxed(tmp | (STP_EN), pi2c->base + I2C_CTL);

	return 0;
}

static int sprd_i2c_clear_start(struct sprd_i2c *pi2c)
{
	unsigned int tmp = readl_relaxed(pi2c->base + I2C_CTL);

	writel_relaxed(tmp & (~I2C_START), pi2c->base + I2C_CTL);

	return 0;
}

static int sprd_i2c_clear_ack(struct sprd_i2c *pi2c)
{
	unsigned int tmp = readl_relaxed(pi2c->base + I2C_STATUS);

	writel_relaxed(tmp & (~I2C_RX_ACK), pi2c->base + I2C_STATUS);

	return 0;
}

static int sprd_i2c_clear_irq(struct sprd_i2c *pi2c)
{
	unsigned int tmp = readl_relaxed(pi2c->base + I2C_STATUS);

	writel_relaxed(tmp & (~I2C_INT), pi2c->base + I2C_STATUS);

	return 0;
}

static void
sprd_i2c_reset_fifo(struct sprd_i2c *pi2c)
{
	writel_relaxed(I2C_RST, pi2c->base + ADDR_RST);
}

static void
sprd_i2c_set_devaddr(struct sprd_i2c *pi2c, struct i2c_msg *m)
{
	writel_relaxed(m->addr << 1, pi2c->base + I2C_ADDR_CFG);
}

static int
sprd_i2c_writebytes(struct sprd_i2c *pi2c, u8 *buf, u32 len)
{
	u32 i = 0;

	for (i = 0; i < len; i++) {
		writel_relaxed(buf[i], pi2c->base + I2C_TX);
		dev_dbg(&pi2c->adap.dev,
			"%s() buf[%d]=%x\n", __func__, i, buf[i]);
	}

	return 0;
}

static void
sprd_i2c_set_full_thld(struct sprd_i2c *pi2c, u32 full_thld)
{
	unsigned int tmp = readl_relaxed(pi2c->base + I2C_CTL);

	tmp = tmp & ~(0xf << FIFO_AF_LVL);
	tmp |= (full_thld << FIFO_AF_LVL);
	writel_relaxed(tmp, pi2c->base + I2C_CTL);

};

static void
sprd_i2c_set_empty_thld(struct sprd_i2c *pi2c, u32 empty_thld)
{
	unsigned int tmp = readl_relaxed(pi2c->base + I2C_CTL);

	tmp = tmp & ~(0xf << FIFO_AE_LVL);
	tmp |= (empty_thld << FIFO_AE_LVL);
	writel_relaxed(tmp, pi2c->base + I2C_CTL);

};

static void
sprd_i2c_set_full_interrupt(struct sprd_i2c *pi2c, int enable)
{
	unsigned int tmp = readl_relaxed(pi2c->base + I2C_CTL);

	if (enable)
		tmp |= FULL_INTEN;
	else
		tmp = tmp & ~FULL_INTEN;

	writel_relaxed(tmp, pi2c->base + I2C_CTL);

};

static void
sprd_i2c_set_empty_interrupt(struct sprd_i2c *pi2c, int enable)
{
	unsigned int tmp = readl_relaxed(pi2c->base + I2C_CTL);

	if (enable)
		tmp |= EMPTY_INTEN;
	else
		tmp = tmp & ~EMPTY_INTEN;

	dev_dbg(&pi2c->adap.dev, "%s() enable = %x\n", __func__, enable);
	writel_relaxed(tmp, pi2c->base + I2C_CTL);

};

static int sprd_i2c_opt_start(struct sprd_i2c *pi2c)
{
	int cmd = readl_relaxed(pi2c->base + I2C_CTL);

	writel_relaxed(cmd | I2C_START, pi2c->base + I2C_CTL);

	return 0;
}

static int
sprd_i2c_readbytes(struct sprd_i2c *pi2c, u8 *buf, u16 len)
{
	u32 i = 0;

	for (i = 0; i < len; i++) {
		buf[i] = readl_relaxed(pi2c->base + I2C_RX);
		dev_dbg(&pi2c->adap.dev,
			"%s() buf[%d]=%x\n", __func__, i, buf[i]);
	}

	return 0;
}

static void sprd_i2c_data_transfer(struct sprd_i2c *pi2c)
{
	struct i2c_msg *m = pi2c->msg;
	u32 i2c_count = pi2c->count;
	u32 need_tran = i2c_count  <= I2C_FIFO_DEEP ? i2c_count : I2C_FIFO_DEEP;

	if ((m->flags & I2C_M_RD)) {
		sprd_i2c_readbytes(pi2c, pi2c->buf, I2C_FIFO_FULL_THLD);
		pi2c->count -= I2C_FIFO_FULL_THLD;
		pi2c->buf += I2C_FIFO_FULL_THLD;

		if (pi2c->count >= I2C_FIFO_FULL_THLD)
			sprd_i2c_set_full_interrupt(pi2c, 1);
	} else {
		sprd_i2c_writebytes(pi2c, pi2c->buf, need_tran);
		pi2c->buf += need_tran;
		pi2c->count -= need_tran;

		if (i2c_count > I2C_FIFO_DEEP)
			sprd_i2c_set_empty_interrupt(pi2c, 1);
	}
}

static int
sprd_i2c_opt_mode(struct sprd_i2c *pi2c, int rw)
{
	int cmd = readl_relaxed(pi2c->base + I2C_CTL) & (~I2C_MODE);

	writel_relaxed((cmd | rw << 3), pi2c->base + I2C_CTL);

	return 0;
}

static int
sprd_i2c_handle_msg(struct i2c_adapter *i2c_adap,
			struct i2c_msg *pmsg, int is_last_msg)
{
	struct sprd_i2c *pi2c = i2c_adap->algo_data;

	pi2c->msg = pmsg;
	pi2c->buf = pmsg->buf;
	pi2c->count = pmsg->len;

	sprd_i2c_reset_fifo(pi2c);
	reinit_completion(&pi2c->complete);
	sprd_i2c_set_devaddr(pi2c, pmsg);

	if ((pmsg->flags & I2C_M_RD)) {
		sprd_i2c_opt_mode(pi2c, 1);
		sprd_i2c_send_stop(pi2c, 1);
	} else {
		sprd_i2c_opt_mode(pi2c, 0);
		if (is_last_msg)
			sprd_i2c_send_stop(pi2c, 1);
		else
			sprd_i2c_send_stop(pi2c, 0);
	}

	sprd_i2c_set_count(pi2c, pmsg->len);
	if ((pmsg->flags & I2C_M_RD))
		sprd_i2c_set_full_interrupt(pi2c, 1);
	else
		sprd_i2c_data_transfer(pi2c);
	sprd_i2c_opt_start(pi2c);

	wait_for_completion(&pi2c->complete);

	return pi2c->err;
}

static int
sprd_i2c_master_xfer(struct i2c_adapter *i2c_adap,
			struct i2c_msg *msgs, int num)
{
	int im = 0, ret = 0;
	struct sprd_i2c *pi2c = i2c_adap->algo_data;

	dev_dbg(&i2c_adap->dev, "%s() msg num=%d\n", __func__, num);

	if (pi2c->is_suspended) {
		dev_err(&i2c_adap->dev, "i2c is suspended !!!\n");
		return -EIO;
	}

	ret = pm_runtime_get_sync(pi2c->dev);
	if (ret < 0)
		return ret;

	for (im = 0; ret >= 0 && im != num; im++)
		ret = sprd_i2c_handle_msg(i2c_adap, &msgs[im], im == num - 1);

	pm_runtime_mark_last_busy(pi2c->dev);
	pm_runtime_put_autosuspend(pi2c->dev);

	return (ret >= 0) ? im : ret;
}

static u32 sprd_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm sprd_i2c_algo = {
	.master_xfer = sprd_i2c_master_xfer,
	.functionality = sprd_i2c_func,
};

static void  sprd_i2c_set_clk(struct sprd_i2c *pi2c, unsigned int freq)
{
	u32 APB_clk = pi2c->src_clk;
	u32 i2c_dvd = APB_clk / (4 * freq) - 1;
	u32 high = ((i2c_dvd << 1) * 2) / 5;
	u32 low = ((i2c_dvd << 1) * 3) / 5;
	u32 div0 = (high & 0xffff) << 16 | (low & 0xffff);
	u32 div1 =  (high & 0xffff0000) | ((low & 0xffff0000) >> 16);

	writel_relaxed(div0, pi2c->base + ADDR_DVD0);
	writel_relaxed(div1, pi2c->base + ADDR_DVD1);

	if (freq == 400000)
		writel_relaxed(((6 * APB_clk) / 10000000),
			pi2c->base + ADDR_STA0_DVD);
	else if (freq == 100000)
		writel_relaxed(((4 * APB_clk) / 1000000),
			pi2c->base + ADDR_STA0_DVD);
}

static void __maybe_unused sprd_i2c_enable(struct sprd_i2c *pi2c)
{
	unsigned int tmp = I2C_DVD_OPT;

	writel_relaxed(tmp, pi2c->base + I2C_CTL);
	sprd_i2c_set_full_thld(pi2c, I2C_FIFO_FULL_THLD);
	sprd_i2c_set_empty_thld(pi2c, I2C_FIFO_EMPTY_THLD);
	dev_dbg(&pi2c->adap.dev, "%s() freq=%d\n",
		__func__, pi2c->bus_freq);

	sprd_i2c_set_clk(pi2c, pi2c->bus_freq);
	sprd_i2c_reset_fifo(pi2c);
	sprd_i2c_clear_irq(pi2c);
	tmp = readl_relaxed(pi2c->base + I2C_CTL);
	writel_relaxed(tmp | (I2C_EN | I2C_INT_EN), pi2c->base + I2C_CTL);
}

static irqreturn_t sprd_i2c_isr(int irq, void *dev_id)
{
	struct sprd_i2c *pi2c = dev_id;
	struct i2c_msg *m = pi2c->msg;
	int ack = readl_relaxed(pi2c->base + I2C_STATUS) & I2C_RX_ACK;
	u32 i2c_tran = 0;

	if (m->flags & I2C_M_RD)
		i2c_tran = pi2c->count >= I2C_FIFO_FULL_THLD;
	else
		i2c_tran = pi2c->count;

	if (!i2c_tran || ack) {
		sprd_i2c_clear_start(pi2c);
		sprd_i2c_clear_irq(pi2c);
	}

	sprd_i2c_set_empty_interrupt(pi2c, 0);
	sprd_i2c_set_full_interrupt(pi2c, 0);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t sprd_i2c_isr_thread(int irq, void *dev_id)
{
	struct sprd_i2c *pi2c = dev_id;
	struct i2c_msg *m = pi2c->msg;
	int ack = readl_relaxed(pi2c->base + I2C_STATUS) & I2C_RX_ACK;
	u32 i2c_tran = 0;

	if (m->flags & I2C_M_RD)
		i2c_tran = pi2c->count >= I2C_FIFO_FULL_THLD;
	else
		i2c_tran = pi2c->count;

	if (i2c_tran && !ack) {
		sprd_i2c_data_transfer(pi2c);
		return IRQ_HANDLED;
	}

	pi2c->err = 0;
	if (ack) {
		pi2c->err = -EIO;
		dev_err(&pi2c->adap.dev, "i2c report: no ack error !!!\n");
		sprd_i2c_dump_reg(pi2c);
	} else if (m->flags & I2C_M_RD && pi2c->count) {
		sprd_i2c_readbytes(pi2c, pi2c->buf, pi2c->count);
	}

	sprd_i2c_clear_ack(pi2c);
	sprd_i2c_clear_start(pi2c);
	complete(&pi2c->complete);

	return IRQ_HANDLED;
}

static int sprd_i2c_clk_init(struct sprd_i2c *pi2c)
{
	struct clk *clk_i2c, *clk_parent;
	struct device_node *np = pi2c->adap.dev.of_node;

	clk_i2c = of_clk_get_by_name(np, "i2c");
	if (IS_ERR(clk_i2c)) {
		dev_warn(&pi2c->adap.dev,
			"i2c%d can't get the clock dts config: i2c\n",
			pi2c->adap.nr);
		clk_i2c = NULL;
	}

	clk_parent = of_clk_get_by_name(np, "source");
	if (IS_ERR(clk_parent)) {
		dev_warn(&pi2c->adap.dev,
			"i2c%d can't get the clock dts config: source\n",
			pi2c->adap.nr);
		clk_parent = NULL;
	}

	if (clk_set_parent(clk_i2c, clk_parent))
		pi2c->src_clk = clk_get_rate(clk_i2c);
	else
		pi2c->src_clk = 26000000;

	dev_info(&pi2c->adap.dev, "i2c%d set source clock is %d\n",
		pi2c->adap.nr, pi2c->src_clk);

	pi2c->clk = of_clk_get_by_name(np, "enable");
	if (IS_ERR(pi2c->clk)) {
		dev_warn(&pi2c->adap.dev,
			"i2c%d can't get the clock dts config: enable\n",
			pi2c->adap.nr);
		pi2c->clk = NULL;
	}

	return 0;
}

static int sprd_i2c_probe(struct platform_device *pdev)
{
	int ret = 0;
	u32 prop = 0;
	struct sprd_i2c *pi2c;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "%s(), no device node\n", __func__);
		return -ENODEV;
	}

	pdev->id = of_alias_get_id(np, "i2c");

	pi2c = devm_kzalloc(&pdev->dev, sizeof(struct sprd_i2c), GFP_KERNEL);
	if (!pi2c)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "%s(), no mem resource?\n", __func__);
		return -ENODEV;
	}

	pi2c->irq = platform_get_irq(pdev, 0);
	if (pi2c->irq < 0) {
		dev_err(&pdev->dev, "not provide irq resource\n");
		return -ENXIO;
	}

	i2c_set_adapdata(&pi2c->adap, pi2c);
	snprintf(pi2c->adap.name, sizeof(pi2c->adap.name), "%s", "sprd-i2c");

	pi2c->bus_freq = 100000;
	pi2c->adap.owner = THIS_MODULE;
	pi2c->dev = &pdev->dev;
	pi2c->adap.retries = 3;
	pi2c->adap.algo = &sprd_i2c_algo;
	pi2c->adap.algo_data = pi2c;
	pi2c->adap.dev.parent = &pdev->dev;
	pi2c->adap.nr = pdev->id;
	pi2c->adap.dev.of_node = pdev->dev.of_node;
	pi2c->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pi2c->base))
		return PTR_ERR(pi2c->base);

	if (!of_property_read_u32(pdev->dev.of_node, "clock-frequency",
		&prop))
		pi2c->bus_freq = prop;

	ret = sprd_i2c_clk_init(pi2c);

	init_completion(&pi2c->complete);

	platform_set_drvdata(pdev, pi2c);
	dev_info(&pdev->dev, "%s() id=%d, base=%p\n",
		__func__, pi2c->adap.nr, pi2c->base);

	ret = clk_prepare_enable(pi2c->clk);
	if (ret) {
		dev_err(&pdev->dev, "clk enable fail !!!\n");
		return ret;
	}
	sprd_i2c_enable(pi2c);

	ret = pm_runtime_set_active(pi2c->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "i2c%d pm_runtime_set_active failed!!!\n",
			pdev->id);
		clk_disable_unprepare(pi2c->clk);
		return ret;
	}

	pm_runtime_enable(pi2c->dev);
	pm_runtime_set_autosuspend_delay(pi2c->dev, SPRD_I2C_PM_TIMEOUT);
	pm_runtime_use_autosuspend(pi2c->dev);

	ret = pm_runtime_get_sync(pi2c->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "i2c%d pm runtime resume failed!\n",
			pdev->id);
		goto err_rpm_put;
	}

	ret = devm_request_threaded_irq(&pdev->dev, pi2c->irq,
		sprd_i2c_isr, sprd_i2c_isr_thread,
		IRQF_NO_SUSPEND | IRQF_ONESHOT,
		pdev->name, pi2c);
	if (ret) {
		dev_err(&pdev->dev, "failure requesting irq %i\n", pi2c->irq);
		goto err_rpm_put;
	}

	ret = i2c_add_numbered_adapter(&pi2c->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "add_adapter failed!\n");
		goto err_rpm_put;
	}

	pm_runtime_mark_last_busy(pi2c->dev);
	pm_runtime_put_autosuspend(pi2c->dev);
	return 0;

err_rpm_put:
	pm_runtime_put_noidle(pi2c->dev);
	pm_runtime_disable(pi2c->dev);
	clk_disable_unprepare(pi2c->clk);
	return ret;
}

static int sprd_i2c_remove(struct platform_device *pdev)
{
	struct sprd_i2c *pi2c = platform_get_drvdata(pdev);
	int ret = pm_runtime_get_sync(pi2c->dev);

	if (ret < 0)
		return ret;

	i2c_del_adapter(&pi2c->adap);
	clk_disable_unprepare(pi2c->clk);

	pm_runtime_put_noidle(pi2c->dev);
	pm_runtime_disable(pi2c->dev);
	return 0;
}

#ifdef CONFIG_PM
static int sprd_i2c_suspend_noirq(struct device *pdev)
{
	struct sprd_i2c *pi2c = dev_get_drvdata(pdev);

	i2c_lock_adapter(&pi2c->adap);
	pi2c->is_suspended = true;
	i2c_unlock_adapter(&pi2c->adap);

	return pm_runtime_force_suspend(pdev);
}

static int sprd_i2c_resume_noirq(struct device *pdev)
{
	struct sprd_i2c *pi2c = dev_get_drvdata(pdev);

	i2c_lock_adapter(&pi2c->adap);
	pi2c->is_suspended = false;
	i2c_unlock_adapter(&pi2c->adap);

	return pm_runtime_force_resume(pdev);
}

static int sprd_i2c_runtime_suspend(struct device *pdev)
{
	struct sprd_i2c *pi2c = dev_get_drvdata(pdev);

	clk_disable_unprepare(pi2c->clk);

	return 0;
}

static int sprd_i2c_runtime_resume(struct device *pdev)
{
	struct sprd_i2c *pi2c = dev_get_drvdata(pdev);
	int ret = clk_prepare_enable(pi2c->clk);

	if (ret) {
		dev_err(pdev, "clk enable fail !!!\n");
		return ret;
	}
	sprd_i2c_enable(pi2c);

	return 0;
}
#endif  /* CONFIG_PM */

static const struct dev_pm_ops sprd_i2c_pm_ops = {
	SET_RUNTIME_PM_OPS(sprd_i2c_runtime_suspend,
			   sprd_i2c_runtime_resume, NULL)
#ifdef CONFIG_PM
	.suspend_noirq = sprd_i2c_suspend_noirq,
	.resume_noirq = sprd_i2c_resume_noirq,
#endif
};

static const struct of_device_id sprd_i2c_of_match[] = {
	{ .compatible = "sprd,r8p0-i2c", },
};

static struct platform_driver sprd_i2c_driver = {
	.probe = sprd_i2c_probe,
	.remove = sprd_i2c_remove,
	.driver = {
		   .name = "sprd-i2c-v2",
		   .of_match_table = of_match_ptr(sprd_i2c_of_match),
		   .pm = &sprd_i2c_pm_ops,
	},
};

static int __init sprd_i2c_init(void)
{
	return platform_driver_register(&sprd_i2c_driver);
}

arch_initcall_sync(sprd_i2c_init);

static void __exit sprd_i2c_exit(void)
{
	platform_driver_unregister(&sprd_i2c_driver);
}

module_exit(sprd_i2c_exit);

MODULE_DESCRIPTION("sprd iic algorithm and driver");
MODULE_AUTHOR("Lanqing Liu <lanqing.liu@spreadtrum.com>");
MODULE_LICENSE("GPL");
