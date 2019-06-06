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

#define SPRD_I2C_CTL_ID				(6)
#define SPRD_I2C_FIXED_TIMING		(0X75)
#define SPRD_I2C_400K_SPEED		(400000)

#define I2C_CTL						0x0000
#define I2C_CTL_INT					BIT(0)
#define I2C_CTL_ACK					BIT(1)
#define I2C_CTL_BUSY				BIT(2)
#define I2C_CTL_IE					BIT(3)
#define I2C_CTL_EN					BIT(4)
#define I2C_CTL_SCL_LINE			BIT(5)
#define I2C_CTL_SDA_LINE			BIT(6)
#define I2C_CTL_NOACK_INT_EN		BIT(7)
#define I2C_CTL_NOACK_INT_STS		BIT(8)
#define I2C_CTL_NOACK_INT_CLR		BIT(9)
#define I2C_CTL_DUTY				BIT(11)

#define I2C_CMD						0x0004
#define I2C_CMD_INT_ACK			BIT(0)
#define I2C_CMD_TX_ACK				BIT(1)
#define I2C_CMD_WRITE				BIT(2)
#define I2C_CMD_READ				BIT(3)
#define I2C_CMD_STOP				BIT(4)
#define I2C_CMD_START				BIT(5)
#define I2C_CMD_ACK					BIT(6)
#define I2C_CMD_BUSY				BIT(7)
#define I2C_CMD_DATA				0xFF00

#define I2C_CLKD0					0x0008
#define I2C_CLKD1					0x000C

#define I2C_RST						0x0010
#define I2C_RST_RST					BIT(0)

#define I2C_CMD_BUF					0x0014

#define I2C_CMD_BUF_CTL			0x0018
#define I2C_CTL_CMDBUF_EN			BIT(0)
#define I2C_CTL_CMDBUF_EXEC		BIT(1)

#define I2C_STA_STO_DVD			0X001C

/* timeout for pm runtime autosuspend */
#define SPRD_I2C_PM_TIMEOUT		1000	/* ms */

struct sprd_platform_i2c {
	unsigned int bus_freq;
};

/* i2c data structure*/
struct sprd_i2c {
	struct i2c_msg *msg;
	struct i2c_adapter adap;
	struct device *dev;
	void __iomem *membase;
	struct clk *clk;
	unsigned int src_clk;
	int irq;
	struct sprd_platform_i2c *pdata;
};

static struct sprd_platform_i2c sprd_platform_i2c_default = {
	.bus_freq = 100 * 1000,
};

static struct sprd_i2c *sprd_i2c_ctl_id[SPRD_I2C_CTL_ID];

static inline struct sprd_platform_i2c *
sprd_i2c_get_platformdata(struct device *dev)
{
	if (dev != NULL && dev->platform_data != NULL)
		return (struct sprd_platform_i2c *)dev->platform_data;

	return &sprd_platform_i2c_default;
}

static int
sprd_i2c_poll_ctl_status(struct sprd_i2c *pi2c, unsigned long bit)
{
	int loop_cntr = 5000;

	do {
		udelay(1);
	} while (!(readl_relaxed(pi2c->membase + I2C_CTL) & bit)
	       && (--loop_cntr > 0));

	if (loop_cntr > 0)
		return 1;
	else
		return -1;
}

static int
sprd_i2c_poll_cmd_status(struct sprd_i2c *pi2c, unsigned long bit)
{
	int loop_cntr = 5000;

	do {
		udelay(1);
	} while ((readl_relaxed(pi2c->membase + I2C_CMD) & bit)
	       && (--loop_cntr > 0));

	if (loop_cntr > 0)
		return 1;
	else
		return -1;
}

static int sprd_i2c_wait_int(struct sprd_i2c *pi2c)
{
	return sprd_i2c_poll_ctl_status(pi2c, I2C_CTL_INT);
}

static int sprd_i2c_wait_busy(struct sprd_i2c *pi2c)
{
	return sprd_i2c_poll_cmd_status(pi2c, I2C_CMD_BUSY);
}

static int sprd_i2c_wait_ack(struct sprd_i2c *pi2c)
{
	return sprd_i2c_poll_cmd_status(pi2c, I2C_CMD_ACK);
}

static  void sprd_i2c_dump_reg(struct sprd_i2c *pi2c)
{
	dev_err(&pi2c->adap.dev,
		": ======dump i2c-%d reg=======\n", pi2c->adap.nr);
	dev_err(&pi2c->adap.dev, ": I2C_CTRL:0x%x\n",
		readl_relaxed(pi2c->membase + I2C_CTL));
	dev_err(&pi2c->adap.dev, ": I2C_CMD:0x%x\n",
		readl_relaxed(pi2c->membase + I2C_CMD));
	dev_err(&pi2c->adap.dev, ": I2C_DVD0:0x%x\n",
		readl_relaxed(pi2c->membase + I2C_CLKD0));
	dev_err(&pi2c->adap.dev, ": I2C_DVD1:0x%x\n",
		readl_relaxed(pi2c->membase + I2C_CLKD1));
	dev_err(&pi2c->adap.dev, ": I2C_RST:0x%x\n",
		readl_relaxed(pi2c->membase + I2C_RST));
	dev_err(&pi2c->adap.dev, ": I2C_CMD_BUF:0x%x\n",
		readl_relaxed(pi2c->membase + I2C_CMD_BUF));
	dev_err(&pi2c->adap.dev, ": I2C_CMD_BUF_CTL:0x%x\n",
		readl_relaxed(pi2c->membase + I2C_CMD_BUF_CTL));
}

static int sprd_i2c_wait_trx_done(struct sprd_i2c *pi2c)
{
	int rc;

	rc = sprd_i2c_wait_int(pi2c);
	if (rc < 0) {
		dump_stack();
		dev_err(&pi2c->adap.dev, "%s() err!i2c no int,rc=%d\n",
			__func__, rc);
		sprd_i2c_dump_reg(pi2c);
		return rc;
	}

	rc = sprd_i2c_wait_busy(pi2c);
	if (rc < 0) {
		dump_stack();
		dev_err(&pi2c->adap.dev, "%s() err!i2c always busy,rc=%d\n",
			__func__, rc);
		sprd_i2c_dump_reg(pi2c);
		return rc;
	}

	rc = sprd_i2c_wait_ack(pi2c);
	if (rc < 0) {
		dump_stack();
		dev_err(&pi2c->adap.dev, "%s() err!no ack,rc=%d\n",
			__func__, rc);
		sprd_i2c_dump_reg(pi2c);
		return rc;
	}

	return rc;
}

static int
sprd_i2c_write_byte(struct sprd_i2c *pi2c, char byte, int stop, int is_last_msg)
{
	int rc , cmd;

	if (stop && is_last_msg)
		cmd = (byte << 8) | I2C_CMD_WRITE
			| I2C_CMD_STOP | I2C_CMD_INT_ACK;
	else
		cmd = (byte << 8) | I2C_CMD_WRITE | I2C_CMD_INT_ACK;

	dev_dbg(&pi2c->adap.dev, "%s() cmd=%x\n", __func__, cmd);
	writel_relaxed(cmd, pi2c->membase + I2C_CMD);

	rc = sprd_i2c_wait_trx_done(pi2c);
	return rc;
}

static int sprd_i2c_read_byte(struct sprd_i2c *pi2c, char *byte, int stop)
{
	int rc, cmd;

	if (stop)
		cmd = I2C_CMD_READ | I2C_CMD_STOP
			| I2C_CMD_TX_ACK | I2C_CMD_INT_ACK;
	else
		cmd = I2C_CMD_READ | I2C_CMD_INT_ACK;

	writel_relaxed(cmd, pi2c->membase + I2C_CMD);
	dev_dbg(&pi2c->adap.dev, "%s() cmd=%x\n", __func__, cmd);

	rc = sprd_i2c_wait_trx_done(pi2c);
	if (rc < 0) {
		dev_err(&pi2c->adap.dev, "%s() err! rc=%d\n", __func__, rc);
		return rc;
	}

	*byte = (unsigned char)(readl_relaxed(pi2c->membase + I2C_CMD) >> 8);
	dev_dbg(&pi2c->adap.dev, "%s() byte=%x, cmd reg=%x\n", __func__, *byte,
		readl_relaxed(pi2c->membase + I2C_CMD));

	return rc;
}

static int
sprd_i2c_writebytes(struct sprd_i2c *pi2c, const char *buf, int count,
		    int is_last_msg)
{
	int i, rc = 0;

	for (i = 0; rc >= 0 && i != count; ++i) {
		rc = sprd_i2c_write_byte(pi2c, buf[i], i == count - 1,
					 is_last_msg);
		if (rc < 0)
			return rc;
	}

	return rc;
}

static int
sprd_i2c_readbytes(struct sprd_i2c *pi2c, char *buf, int count)
{
	int i, rc = 0;

	for (i = 0; rc >= 0 && i != count; ++i) {
		rc = sprd_i2c_read_byte(pi2c, &buf[i], i == count - 1);

		if (rc < 0)
			return rc;
	}

	return rc;
}

static int
sprd_i2c_send_target_addr(struct sprd_i2c *pi2c, struct i2c_msg *msg)
{
	int rc = 0, cmd = 0, cmd2 = 0, tmp = 0;

	if (msg->flags & I2C_M_TEN) {
		cmd = 0xf0 | (((msg->addr >> 8) & 0x03) << 1);
		cmd2 = msg->addr & 0xff;
	} else {
		cmd = (msg->addr & 0x7f) << 1;
	}

	if (msg->flags & I2C_M_RD)
		cmd |= 1;

	tmp = readl_relaxed(pi2c->membase + I2C_CTL);
	writel_relaxed(tmp | I2C_CTL_EN | I2C_CTL_IE, pi2c->membase + I2C_CTL);
	cmd = (cmd << 8) | I2C_CMD_START | I2C_CMD_WRITE | I2C_CMD_INT_ACK;
	writel_relaxed(cmd, pi2c->membase + I2C_CMD);

	rc = sprd_i2c_wait_trx_done(pi2c);
	if (rc < 0)
		return rc;

	if ((msg->flags & I2C_M_TEN) && (!(msg->flags & I2C_M_RD))) {
		cmd2 = (cmd2 << 8) | I2C_CMD_WRITE | I2C_CMD_INT_ACK;
		writel_relaxed(cmd2, pi2c->membase + I2C_CMD);

		rc = sprd_i2c_wait_trx_done(pi2c);
		if (rc < 0)
			return rc;
	}

	return rc;
}

static int
sprd_i2c_handle_msg(struct i2c_adapter *i2c_adap,
struct i2c_msg *pmsg, int is_last_msg)
{
	int rc;
	struct sprd_i2c *pi2c = i2c_adap->algo_data;

	dev_dbg(&i2c_adap->dev, "%s() flag=%x, adr=%x, len=%d\n",
		__func__, pmsg->flags, pmsg->addr, pmsg->len);

	rc = sprd_i2c_send_target_addr(pi2c, pmsg);
	if (rc < 0) {
		dev_err(&i2c_adap->dev, "%s() send addr fail rc=%d\n",
			__func__, rc);
		return rc;
	}

	if ((pmsg->flags & I2C_M_RD))
		return sprd_i2c_readbytes(pi2c, pmsg->buf, pmsg->len);
	else
		return sprd_i2c_writebytes(pi2c, pmsg->buf, pmsg->len,
					   is_last_msg);
}

static int
sprd_i2c_master_xfer(struct i2c_adapter *i2c_adap,
						  struct i2c_msg *msgs, int num)
{
	int im = 0, ret = 0;
	struct sprd_i2c *pi2c = i2c_adap->algo_data;

	ret = pm_runtime_get_sync(pi2c->dev);
	if (ret < 0)
		return ret;
	dev_dbg(&i2c_adap->dev, "%s() msg num=%d\n", __func__, num);

	for (im = 0; ret >= 0 && im != num; im++) {
		dev_dbg(&i2c_adap->dev, "%s() msg im=%d\n", __func__, im);
		ret = sprd_i2c_handle_msg(i2c_adap, &msgs[im], im == num - 1);
	}
	pm_runtime_mark_last_busy(pi2c->dev);
	pm_runtime_put_autosuspend(pi2c->dev);

	return (ret >= 0) ? im : -1;
}

static u32 sprd_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm sprd_i2c_algo = {
	.master_xfer = sprd_i2c_master_xfer,
	.functionality = sprd_i2c_func,
};

static void sprd_i2c_set_clk(struct sprd_i2c *pi2c, unsigned int freq)
{
	unsigned int apb_clk = pi2c->src_clk;
	unsigned int i2c_div = apb_clk / (4 * freq) - 1;

	/*fixed 400k start time issue after r7p0*/
	if (freq == SPRD_I2C_400K_SPEED) {
		writel_relaxed(0X10, pi2c->membase + I2C_STA_STO_DVD);
		writel_relaxed(0x0012000c, pi2c->membase + I2C_CLKD0);
	} else {
		writel_relaxed(((i2c_div & 0xffff)<<16|(i2c_div & 0xffff)),
					   pi2c->membase + I2C_CLKD0);
	}

	writel_relaxed((i2c_div >> 16 | (i2c_div & 0xffff0000)),
				   pi2c->membase + I2C_CLKD1);
}

static int sprd_i2c_clk_init(struct sprd_i2c *pi2c)
{
	struct clk *clk_i2c, *clk_parent;

	struct device_node *np = pi2c->adap.dev.of_node;

	clk_i2c = of_clk_get_by_name(np, "i2c");
	if (IS_ERR(clk_i2c)) {
		dev_dbg(&pi2c->adap.dev,
			"i2c%d can't get the clock dts config: i2c\n",
			pi2c->adap.nr);
		return -EINVAL;
	}

	clk_parent = of_clk_get_by_name(np, "source");
	if (IS_ERR(clk_parent)) {
		dev_dbg(&pi2c->adap.dev,
			"i2c%d can't get the clock dts config: source\n",
			pi2c->adap.nr);
		return -EINVAL;
	}

	clk_set_parent(clk_i2c, clk_parent);
	pi2c->src_clk = clk_get_rate(clk_i2c);

	dev_err(&pi2c->adap.dev, "i2c%d set source clock is %d\n",
		pi2c->adap.nr, pi2c->src_clk);

	pi2c->clk = of_clk_get_by_name(np, "enable");

	if (IS_ERR(pi2c->clk)) {
		dev_dbg(&pi2c->adap.dev,
			"i2c%d can't get the clock dts config: enable\n",
			pi2c->adap.nr);
		return -EINVAL;
	}
	return 0;
}

static void __maybe_unused sprd_i2c_enable(struct sprd_i2c *pi2c)
{
	unsigned int tmp;

	tmp = readl_relaxed(pi2c->membase + I2C_RST);
	tmp |= (I2C_RST_RST);
	writel_relaxed(tmp, pi2c->membase + I2C_RST);

	tmp = readl_relaxed(pi2c->membase + I2C_CTL);
	tmp &= ~(I2C_CTL_EN);
	writel_relaxed(tmp, pi2c->membase + I2C_CTL);

	writel_relaxed(SPRD_I2C_FIXED_TIMING,
		pi2c->membase + I2C_STA_STO_DVD);

	dev_dbg(&pi2c->adap.dev, "%s() freq=%d\n",
		__func__, pi2c->pdata->bus_freq);

	sprd_i2c_set_clk(pi2c, pi2c->pdata->bus_freq);
	tmp = readl_relaxed(pi2c->membase + I2C_CTL);

	writel_relaxed(tmp | I2C_CTL_EN | I2C_CTL_IE|I2C_CTL_DUTY,
				   pi2c->membase + I2C_CTL);
	writel_relaxed(I2C_CMD_INT_ACK, pi2c->membase + I2C_CMD);
};


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

	i2c_set_adapdata(&pi2c->adap, pi2c);
	snprintf(pi2c->adap.name, sizeof(pi2c->adap.name), "%s", "sprd-i2c");
	pi2c->adap.owner = THIS_MODULE;
	pi2c->dev = &pdev->dev;
	pi2c->adap.retries = 3;
	pi2c->adap.algo = &sprd_i2c_algo;
	pi2c->adap.algo_data = pi2c;
	pi2c->adap.dev.parent = &pdev->dev;
	pi2c->adap.nr = pdev->id;
	pi2c->adap.dev.of_node = pdev->dev.of_node;
	pi2c->membase = devm_ioremap_nocache(&pdev->dev, res->start,
		res->end - res->start);
	if (!pi2c->membase) {
		dev_err(&pdev->dev, "ioremap failed!\n");
		return -ENOMEM;
	}

	pi2c->pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct sprd_platform_i2c), GFP_KERNEL);
	if (!pi2c->pdata)
		return -ENOMEM;
	memcpy(pi2c->pdata, &sprd_platform_i2c_default,
			sizeof(struct sprd_platform_i2c));
	if (!of_property_read_u32(pdev->dev.of_node, "clock-frequency",
		&prop))
		pi2c->pdata->bus_freq = prop;

	ret = sprd_i2c_clk_init(pi2c);
	if (ret) {
		dev_dbg(&pdev->dev, "%s(), i2c%d init clk fail\n",
			__func__, pi2c->adap.nr);
		return ret;
	}
	platform_set_drvdata(pdev, pi2c);

	dev_info(&pdev->dev, "%s() id=%d, base=%p\n",
		__func__, pi2c->adap.nr, pi2c->membase);

	pm_runtime_enable(pi2c->dev);
	pm_runtime_set_autosuspend_delay(pi2c->dev, SPRD_I2C_PM_TIMEOUT);
	pm_runtime_use_autosuspend(pi2c->dev);

	ret = pm_runtime_get_sync(pi2c->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "i2c%d pm runtime resume failed!\n",
			pdev->id);
		return ret;
	}

	ret = i2c_add_numbered_adapter(&pi2c->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "add_adapter failed!\n");
		pm_runtime_put(pi2c->dev);
		pm_runtime_disable(pi2c->dev);
		return ret;
	}

	sprd_i2c_ctl_id[pdev->id] = pi2c;
	pm_runtime_mark_last_busy(pi2c->dev);
	pm_runtime_put_autosuspend(pi2c->dev);

	return 0;
}

static int sprd_i2c_remove(struct platform_device *pdev)
{
	struct sprd_i2c *pi2c = platform_get_drvdata(pdev);

	pm_runtime_put(pi2c->dev);
	pm_runtime_disable(pi2c->dev);

	i2c_del_adapter(&pi2c->adap);
	return 0;
}

#ifdef CONFIG_PM
static int sprd_i2c_suspend_noirq(struct device *pdev)
{
	return pm_runtime_force_suspend(pdev);
}

static int sprd_i2c_resume_noirq(struct device *pdev)
{
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

	clk_prepare_enable(pi2c->clk);
	sprd_i2c_enable(pi2c);

	return 0;
}
#endif  /* CONFIG_PM */

static struct dev_pm_ops sprd_i2c_pm_ops = {
	SET_RUNTIME_PM_OPS(sprd_i2c_runtime_suspend,
			   sprd_i2c_runtime_resume, NULL)
#ifdef CONFIG_PM
	.suspend_noirq = sprd_i2c_suspend_noirq,
	.resume_noirq = sprd_i2c_resume_noirq,
#endif
};

static struct of_device_id sprd_i2c_of_match[] = {
	{ .compatible = "sprd,r7p0-i2c", },
};

static struct platform_driver sprd_i2c_driver = {
	.probe = sprd_i2c_probe,
	.remove = sprd_i2c_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "sprd-i2c",
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
