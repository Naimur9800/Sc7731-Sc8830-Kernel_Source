/*
 * copyright (C) 2018 Spreadtrum Communications Inc.
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
#include <linux/hwspinlock.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/soc/sprd/djtag.h>

#define DJTAG_IR_LEN		0x0
#define DJTAG_DR_LEN		0x4
#define DJTAG_IR		0x8
#define DJTAG_DR		0xc
#define DR_PAUSE_RECOV		0x10
#define DJTAG_RND_EN		0x14
#define DJTAG_UPD_DR		0x18

#define SPRD_SYS_BIT		0x8
#define SPRD_INSTRUCTION_LEN	0x9
#define SPRD_WRITE(n)		((n & GENMASK(8, 0)) | BIT(8))
#define SPRD_READ(n)		(n & GENMASK(8, 0))
#define SPRD_SEL_MUX		0x108
#define SPRD_MUX_DLEN		0x20
#define SPRD_DJTAG_DATA_LEN	0X20
#define SPRD_HWSPINLOCK_TIMEOUT	5000

#define SPRD_DJTAG_TCK_EB       BIT(10)

#define SPRD_DJTAG_SET_OFFSET	0x1000
#define SPRD_DJTAG_CLR_OFFSET	0x2000
#define SPRD_DJTAG_SET(reg)	((reg) + SPRD_DJTAG_SET_OFFSET)
#define SPRD_DJTAG_CLR(reg)	((reg) + SPRD_DJTAG_CLR_OFFSET)

struct sprd_djtag {
	void __iomem *base;
	void __iomem *aon_apb;
	struct hwspinlock *lock;
	struct device *dev;
	struct clk *clk;
	struct clk *tck;
	struct djtag_master master;
};

static void sprd_djtag_tck_eb(struct sprd_djtag *djtag, bool eb)
{
	u32 val;

	val = readl_relaxed(djtag->aon_apb + REG_AON_APB_RES_REG0);
	if (eb)
		val |= SPRD_DJTAG_TCK_EB;
	else
		val &= ~SPRD_DJTAG_TCK_EB;
	writel_relaxed(val, djtag->aon_apb + REG_AON_APB_RES_REG0);
}

static void sprd_djtag_soft_rst(struct sprd_djtag *djtag, bool rst)
{
	if (rst)
		writel_relaxed(BIT_AON_APB_DJTAG_SOFT_RST,
			       SPRD_DJTAG_SET(djtag->aon_apb +
					      REG_AON_APB_APB_RST2));
	else
		writel_relaxed(BIT_AON_APB_DJTAG_SOFT_RST,
			       SPRD_DJTAG_CLR(djtag->aon_apb +
					      REG_AON_APB_APB_RST2));
}

static void sprd_djtag_mux_sel(struct djtag_master *m, u32 sys, u32 dap)
{
	struct sprd_djtag *djtag = (struct sprd_djtag *) m->data;
	u32 mux = (sys << SPRD_SYS_BIT) | dap;

	sprd_djtag_soft_rst(djtag, true);
	udelay(5);
	sprd_djtag_soft_rst(djtag, false);
	writel_relaxed(SPRD_INSTRUCTION_LEN, djtag->base + DJTAG_IR_LEN);
	writel_relaxed(SPRD_MUX_DLEN, djtag->base + DJTAG_DR_LEN);
	writel_relaxed(SPRD_SEL_MUX, djtag->base + DJTAG_IR);
	writel_relaxed(mux, djtag->base + DJTAG_DR);
	writel_relaxed(1, djtag->base + DJTAG_RND_EN);
	/* The DJTAG controller need the valid delay time for send data. */
	udelay(100);
	writel_relaxed(0, djtag->base + DJTAG_RND_EN);
}

static void sprd_djtag_write(struct djtag_master *m, u32 num, u32 len, u32 val)
{
	struct sprd_djtag *djtag = (struct sprd_djtag *) m->data;

	writel_relaxed(SPRD_INSTRUCTION_LEN, djtag->base + DJTAG_IR_LEN);
	writel_relaxed(len, djtag->base + DJTAG_DR_LEN);
	writel_relaxed(SPRD_WRITE(num), djtag->base + DJTAG_IR);
	writel_relaxed(val, djtag->base + DJTAG_DR);
	writel_relaxed(1, djtag->base + DJTAG_RND_EN);
	/* The DJTAG controller need the valid delay time for send data. */
	udelay(100);
	writel_relaxed(0, djtag->base + DJTAG_RND_EN);
}

static int sprd_djtag_read(struct djtag_master *m, u32 num, u32 len)
{
	struct sprd_djtag *djtag = (struct sprd_djtag *) m->data;
	u32 val;

	writel_relaxed(0, djtag->base + DJTAG_DR);
	writel_relaxed(SPRD_INSTRUCTION_LEN, djtag->base + DJTAG_IR_LEN);
	writel_relaxed(len, djtag->base + DJTAG_DR_LEN);
	writel_relaxed(SPRD_READ(num), djtag->base + DJTAG_IR);
	writel_relaxed(1, djtag->base + DJTAG_RND_EN);
	/* The DJTAG controller need the valid delay time for send data. */
	udelay(100);
	writel_relaxed(0, djtag->base + DJTAG_RND_EN);
	val = readl_relaxed(djtag->base + DJTAG_UPD_DR);

	return val >> (SPRD_DJTAG_DATA_LEN - len);
}

static int sprd_djtag_lock(struct djtag_master *m)
{
	struct sprd_djtag *djtag = (struct sprd_djtag *) m->data;
	int ret;

	ret = hwspin_lock_timeout(djtag->lock, SPRD_HWSPINLOCK_TIMEOUT);
	if (ret) {
		dev_err(djtag->dev, "lock djtag hw spinlock failed\n");
		return ret;
	}

	ret = clk_enable(djtag->clk);
	if (ret) {
		dev_err(djtag->dev, "djtag clk enable fail\n");
		goto clk_enable_err;
	}

	ret = clk_enable(djtag->tck);
	if (ret) {
		dev_err(djtag->dev, "djtag tck clk enable fail\n");
		goto tck_enable_err;
	}
	return 0;

tck_enable_err:
	clk_disable(djtag->clk);
clk_enable_err:
	hwspin_unlock(djtag->lock);

	return ret;
}

static void sprd_djtag_unlock(struct djtag_master *m)
{
	struct sprd_djtag *djtag = (struct sprd_djtag *) m->data;

	clk_disable(djtag->clk);
	clk_disable(djtag->tck);
	hwspin_unlock(djtag->lock);
}

static struct djtag_ops ops = {
	.mux_sel = sprd_djtag_mux_sel,
	.write = sprd_djtag_write,
	.read =  sprd_djtag_read,
	.lock = sprd_djtag_lock,
	.unlock = sprd_djtag_unlock,
};

static int sprd_djtag_remove(struct platform_device *pdev)
{
	struct sprd_djtag *djtag = platform_get_drvdata(pdev);

	clk_unprepare(djtag->clk);
	clk_unprepare(djtag->tck);
	hwspin_lock_free(djtag->lock);
	djtag_unregister_master(&djtag->master);

	return 0;
}

static int sprd_djtag_probe(struct platform_device *pdev)
{
	struct sprd_djtag *djtag;
	struct resource *res;
	int ret;

	djtag = devm_kzalloc(&pdev->dev, sizeof(*djtag), GFP_KERNEL);
	if (!djtag)
		return -ENOMEM;

	djtag->clk = devm_clk_get(&pdev->dev, "enable");
	if (IS_ERR(djtag->clk)) {
		dev_warn(djtag->dev,
			"djtag can't get the clock dts config: enable\n");
		djtag->clk = NULL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "djtag");
	djtag->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(djtag->base)) {
		dev_err(&pdev->dev, "sprd djtag get base address failed!\n");
		return PTR_ERR(djtag->base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "glb");
	djtag->aon_apb = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(djtag->aon_apb)) {
		dev_err(&pdev->dev, "sprd djtag get base address failed!\n");
		return PTR_ERR(djtag->aon_apb);
	}

	djtag->tck = devm_clk_get(&pdev->dev, "tck");
	if (IS_ERR(djtag->tck)) {
		sprd_djtag_tck_eb(djtag, true);
		djtag->tck = NULL;
	}

	djtag->dev = &pdev->dev;
	djtag->lock = of_hwspin_lock_request(pdev->dev.of_node, "djtag");
	if (!djtag->lock) {
		pr_err("djtag can not get the hardware spinlock.\n");
		return -ENXIO;
	}

	djtag->master.ops = &ops;
	djtag->master.dev.of_node = pdev->dev.of_node;
	djtag->master.dev.parent = djtag->dev;
	djtag->master.data = djtag;
	dev_set_drvdata(&pdev->dev, djtag);
	ret = clk_prepare(djtag->clk);
	if (ret) {
		dev_err(&pdev->dev, "clk_prepare fail ret =%d\n", ret);
		goto clk_err;
	}

	ret = clk_prepare(djtag->tck);
	if (ret) {
		dev_err(&pdev->dev, "tck_clk_prepare fail ret =%d\n", ret);
		goto tck_clk_err;
	}

	ret = register_djtag_master(&djtag->master);
	if (ret) {
		dev_err(&pdev->dev, "register master fail ret =%d\n", ret);
		goto master_err;
	}

	dev_dbg(&pdev->dev, "probe done\n");
	return 0;

master_err:
	clk_unprepare(djtag->tck);
tck_clk_err:
	clk_unprepare(djtag->clk);
clk_err:
	hwspin_lock_free(djtag->lock);

	return ret;
}

static const struct of_device_id sprd_djtag_match[] = {
	 {.compatible = "sprd,djtag",},
	 {},
};
MODULE_DEVICE_TABLE(of, sprd_djtag_match);

static struct platform_driver sprd_djtag_driver = {
	.probe = sprd_djtag_probe,
	.remove = sprd_djtag_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd-djtag",
		.of_match_table = sprd_djtag_match,
	},
};

static int __init sprd_djtag_init(void)
{
	return platform_driver_register(&sprd_djtag_driver);
}
module_init(sprd_djtag_init);

static void __exit sprd_djtag_exit(void)
{
	platform_driver_unregister(&sprd_djtag_driver);
}
module_exit(sprd_djtag_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lanqing Liu<lanqing.liu@spreadtrum.com>");
MODULE_DESCRIPTION("spreadtrum platform DJTAG driver");
