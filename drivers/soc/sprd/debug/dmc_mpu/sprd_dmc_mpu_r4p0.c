/*
 *copyright (C) 2017 Spreadtrum Communications Inc.
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

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/sprd/dmc_mpu.h>

#define SPRD_CFG(n) \
		(REG_PUB_AHB_MPU_CFG0 + 0x10 * (n))
#define SPRD_CFG_ID_MASK(n)	 \
		(REG_PUB_AHB_MPU_CFG0_ID_MASK_VAL + 0x10 * (n))
#define SPRD_CFG_LOW(n) \
		(REG_PUB_AHB_MPU_CFG0_LOW_RANGE + 0x10 * (n))
#define SPRD_CFG_HIGH(n) \
		(REG_PUB_AHB_MPU_CFG0_HIGH_RANGE + 0x10 * (n))

#define SPRD_MPU_SET_OFFSET		0x1000
#define SPRD_MPU_CLR_OFFSET		0x2000
#define SPRD_MPU_SET(reg)		((reg) + SPRD_MPU_SET_OFFSET)
#define SPRD_MPU_CLR(reg)		((reg) + SPRD_MPU_CLR_OFFSET)

#define SPRD_MPU_VIO_USERID(cmd) \
		(((cmd) & GENMASK(31, 24)) >>  24)
#define SPRD_MPU_VIO_ADDR(cmd)		(((cmd) & GENMASK(22, 21)) >> 21)
#define SPRD_MPU_VIO_WR(cmd)		((cmd) & BIT(20))
#define SPRD_MPU_VIO_PORT(cmd)		(((cmd) & GENMASK(19, 16)) >> 16)
#define SPRD_MPU_VIO_ID(cmd)		((cmd) & GENMASK(15, 0))

#define SPRD_MPU_CFG_EN(en)		(((en) << 8) & BIT(8))
#define SPRD_MPU_CFG_ID(id)		(((id) << 7) & BIT(7))
#define SPRD_MPU_CFG_INCLUDE(mode)	(((mode) << 6) & BIT(6))
#define SPRD_MPU_CFG_PORT(n)		((n) & GENMASK(3, 0))
#define SPRD_MPU_ID_VAL(m, v) \
		((((m) << 16) & GENMASK(31, 16)) | ((v) & GENMASK(15, 0)))

#define SPRD_MPU_MON_ADDR(v)		((v) >> 6)
#define SPRD_MPU_BASE_OFFSET		0X10000

struct sprd_dmpu_base {
	void __iomem *pub_top;
	void __iomem *base;
};

struct sprd_dmpu_device {
	struct sprd_dmpu_core core;
	struct sprd_dmpu_base *addr;
};

#define to_sprd_dmpu_device(x)	container_of(x, struct sprd_dmpu_device, core)

static void sprd_dmc_mpu_enable(struct sprd_dmpu_core *core,
			       u32 pub, bool enable)
{
	struct sprd_dmpu_device *sprd_mpu = to_sprd_dmpu_device(core);

	if (enable)
		writel_relaxed(BIT_PUB_AHB_MPU_EN,
			       sprd_mpu->addr[pub].base +
			       REG_PUB_AHB_DMC_MPU_BASE_CFG);
	else
		writel_relaxed(0, sprd_mpu->addr[pub].base +
			       REG_PUB_AHB_DMC_MPU_BASE_CFG);
}

static void sprd_dmc_mpu_clr_irq(struct sprd_dmpu_core *core, u32 pub)
{
	struct sprd_dmpu_device *sprd_mpu = to_sprd_dmpu_device(core);

	writel_relaxed(BIT_PUB_AHB_WRAP_DMC_MPU_VIO_INT_CLR,
		       sprd_mpu->addr[pub].pub_top +
		       REG_PUB_AHB_WRAP_PUB_DMC_MPU_INT);
}

static void sprd_dmc_mpu_irq_enable(struct sprd_dmpu_core *core, u32 pub)
{
	struct sprd_dmpu_device *sprd_mpu = to_sprd_dmpu_device(core);

	writel_relaxed(BIT_PUB_AHB_WRAP_DMC_MPU_VIO_INT_EN,
		       SPRD_MPU_SET(sprd_mpu->addr[pub].pub_top) +
				    REG_PUB_AHB_WRAP_PUB_DMC_MPU_INT);
}

static void sprd_dmc_mpu_irq_disable(struct sprd_dmpu_core *core, u32 pub)
{
	struct sprd_dmpu_device *sprd_mpu = to_sprd_dmpu_device(core);

	writel_relaxed(BIT_PUB_AHB_WRAP_DMC_MPU_VIO_INT_EN,
		       SPRD_MPU_CLR(sprd_mpu->addr[pub].pub_top) +
				    REG_PUB_AHB_WRAP_PUB_DMC_MPU_INT);
}

static void sprd_dmc_mpu_vio_cmd(struct sprd_dmpu_core *core, u32 pub)
{
	struct sprd_dmpu_device *sprd_mpu = to_sprd_dmpu_device(core);
	struct sprd_dmpu_info *mpu_info = core->mpu_info;
	struct sprd_dmpu_violate *vio = &mpu_info->vio;
	u32 vio_cmd = readl_relaxed(sprd_mpu->addr[pub].base +
				    REG_PUB_AHB_DMC_MPU_VIO_CMD);

	vio->userid = SPRD_MPU_VIO_USERID(vio_cmd);
	vio->addr_h = SPRD_MPU_VIO_ADDR(vio_cmd);
	vio->wr = SPRD_MPU_VIO_WR(vio_cmd);
	vio->port = SPRD_MPU_VIO_PORT(vio_cmd);
	vio->id = SPRD_MPU_VIO_ID(vio_cmd);
	vio->addr = readl_relaxed(sprd_mpu->addr[pub].base +
				  REG_PUB_AHB_DMC_MPU_VIO_ADDR);
}

static void
sprd_dmc_mpu_config_channel(struct sprd_dmpu_core *core,
			    u32 pub, u32 n)
{
	struct sprd_dmpu_device *sprd_mpu = to_sprd_dmpu_device(core);
	struct sprd_dmpu_chn_cfg *cfg = core->cfg;
	u32 addr_min = cfg[n].addr_start - core->ddr_addr_offset;
	u32 addr_max = cfg[n].addr_end - core->ddr_addr_offset;
	u32 mpu_cfg = 0;

	if (!cfg[n].en) {
		dev_err(core->dev, "channel%d: disable\n", n);
		return;
	}

	if (addr_min > addr_max) {
		dev_err(core->dev, "channel%d: address config error\n", n);
		return;
	}

	mpu_cfg |= SPRD_MPU_CFG_EN(cfg[n].en);
	mpu_cfg |= SPRD_MPU_CFG_ID(cfg[n].id_type);
	mpu_cfg |= SPRD_MPU_CFG_INCLUDE(cfg[n].include);

	if (!(cfg[n].mode & SPRD_MPU_R_MODE))
		mpu_cfg |= BIT(4);
	else if (cfg[n].mode & SPRD_MPU_W_MODE)
		mpu_cfg |= BIT(5);

	mpu_cfg |= SPRD_MPU_CFG_PORT(cfg[n].port);
	writel_relaxed(mpu_cfg, sprd_mpu->addr[pub].base + SPRD_CFG(n));
	writel_relaxed(SPRD_MPU_ID_VAL(cfg[n].id_mask, cfg[n].userid),
		       sprd_mpu->addr[pub].base + SPRD_CFG_ID_MASK(n));
	writel_relaxed(SPRD_MPU_MON_ADDR(addr_min),
		       sprd_mpu->addr[pub].base + SPRD_CFG_LOW(n));
	writel_relaxed(SPRD_MPU_MON_ADDR(addr_max),
		       sprd_mpu->addr[pub].base + SPRD_CFG_HIGH(n));
}

static void sprd_dmc_mpu_channel_dump_cfg(struct sprd_dmpu_core *core,
					u32 pub)
{
	struct sprd_dmpu_device *sprd_mpu = to_sprd_dmpu_device(core);
	struct sprd_dmpu_info *mpu_info = core->mpu_info;

	writel_relaxed(mpu_info[pub].dump_paddr - core->ddr_addr_offset,
		       sprd_mpu->addr[pub].base + REG_PUB_AHB_MPU_DUMP_ADDR);
}

static struct sprd_dmpu_ops ops = {
	.enable = sprd_dmc_mpu_enable,
	.clr_irq = sprd_dmc_mpu_clr_irq,
	.vio_cmd = sprd_dmc_mpu_vio_cmd,
	.irq_enable = sprd_dmc_mpu_irq_enable,
	.irq_disable = sprd_dmc_mpu_irq_disable,
	.config = sprd_dmc_mpu_config_channel,
	.dump_cfg = sprd_dmc_mpu_channel_dump_cfg,
};

static int sprd_dmc_mpu_probe(struct platform_device *pdev)
{
	struct sprd_dmpu_device *sprd_mpu;
	struct sprd_dmpu_base *addr;
	struct resource *res;
	bool interleaved;
	int i, ret;

	sprd_mpu = devm_kzalloc(&pdev->dev, sizeof(*sprd_mpu), GFP_KERNEL);
	if (!sprd_mpu)
		return -ENOMEM;

	interleaved = of_property_read_bool(pdev->dev.of_node,
					    "sprd,ddr-interleaved");
	addr = devm_kzalloc(&pdev->dev, sizeof(*addr) << interleaved,
			    GFP_KERNEL);
	if (!addr)
		return -ENOMEM;
	sprd_mpu->addr = addr;

	for (i = 0; i <= interleaved; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			dev_err(&pdev->dev,
				"dmc mpu get io resource %d failed\n", i);
			return -ENODEV;
		}

		addr[i].base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(addr[i].pub_top))
			return PTR_ERR(addr[i].pub_top);

		addr->pub_top = addr[i].base + SPRD_MPU_BASE_OFFSET;
	}

	platform_set_drvdata(pdev, &sprd_mpu->core);

	ret = sprd_dmc_mpu_register(pdev, &sprd_mpu->core, &ops);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "sprd dmc mpu init ok\n");

	return 0;
}

static int sprd_dmc_mpu_remove(struct platform_device *pdev)
{
	struct sprd_dmpu_device *sprd_mpu = platform_get_drvdata(pdev);

	sprd_dmc_mpu_unregister(&sprd_mpu->core);

	return 0;
}

static const struct of_device_id sprd_dmc_mpu_of_match[] = {
	{ .compatible = "sprd,dmc-mpu-r4p0", },
};
MODULE_DEVICE_TABLE(of, sprd_dmpu_of_match);

static struct platform_driver sprd_dmc_mpu_driver = {
	.probe    = sprd_dmc_mpu_probe,
	.remove   = sprd_dmc_mpu_remove,
	.driver = {
		.name = "sprd-dmc-mpu",
		.of_match_table = sprd_dmc_mpu_of_match,
	},
};

module_platform_driver(sprd_dmc_mpu_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lanqing Liu <lanqing.liu@spreadtrum.com>");
MODULE_DESCRIPTION("Spreadtrum platform dmc mpu driver");
