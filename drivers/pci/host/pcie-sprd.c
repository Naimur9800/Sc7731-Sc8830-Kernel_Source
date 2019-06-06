/*
 * PCIe host controller driver for Spreadtrum SoCs
 *
 * Copyright (c) 2018 Spreadtrum Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#include "pcie-designware.h"

struct sprd_pcie {
	struct pcie_port pp;
	struct gpio_desc *perst;
	struct clk *pcie_eb;
};

static void sprd_pcie_fix_class(struct pci_dev *pdev)
{
	if (pdev->class == PCI_CLASS_NOT_DEFINED)
		pdev->class = (((uint32_t)PCI_CLASS_OTHERS)<<16);
	dev_info(&pdev->dev, "fix device:0x%x:0x%x class to: 0x%x\n",
		 pdev->vendor, pdev->device, pdev->class);
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_SYNOPSYS, 0xedda, sprd_pcie_fix_class);

static irqreturn_t sprd_pcie_msi_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	return dw_handle_msi_irq(pp);
}

static void sprd_pcie_assert_reset(struct pcie_port *pp)
{
	/* TODO */
}

static int sprd_pcie_wait_for_link(struct pcie_port *pp)
{
	unsigned int retries;

	if (dw_pcie_link_up(pp))
		return 0;

	for (retries = 0; retries < 10; retries++) {
		if (dw_pcie_link_up(pp)) {
			dev_info(pp->dev, "Link up\n");
			return 0;
		}
		usleep_range(90000, 100000);
	}
	dev_err(pp->dev, "PCIe Link Fail\n");

	return -EINVAL;
}

static void sprd_pcie_host_init(struct pcie_port *pp)
{
	sprd_pcie_assert_reset(pp);

	dw_pcie_setup_rc(pp);

	if (IS_ENABLED(CONFIG_PCI_MSI))
		dw_pcie_msi_init(pp);

	sprd_pcie_wait_for_link(pp);
}

static long sprd_pcie_get_resource(struct sprd_pcie *sprd_pcie,
				    struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *dbi;

	dbi = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	sprd_pcie->pp.dbi_base = devm_ioremap_resource(dev, dbi);
	if (IS_ERR(sprd_pcie->pp.dbi_base)) {
		dev_err(&pdev->dev,
			"pci can't get the dbi base\n");
		return PTR_ERR(sprd_pcie->pp.dbi_base);
	}

	/* TODO: add other resource */
	sprd_pcie->pcie_eb = devm_clk_get(&pdev->dev, "pcie_eb");

	if (IS_ERR(sprd_pcie->pcie_eb)) {
		dev_warn(&pdev->dev,
			"pci can't get the clock dts config: pcie_eb\n");
		sprd_pcie->pcie_eb = NULL;
	}

	return 0;
}

static int sprd_pcie_power_on(struct sprd_pcie *sprd_pcie)
{
	/* TODO: enable power and clock for host and phy */
	clk_prepare_enable(sprd_pcie->pcie_eb);

	return 0;
}

static struct pcie_host_ops sprd_pcie_host_ops = {
	.host_init = sprd_pcie_host_init,
};

static int sprd_add_pcie_port(struct pcie_port *pp,
				      struct platform_device *pdev)
{
	int ret;

	pp->ops = &sprd_pcie_host_ops;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq(pdev, 0);
		if (pp->msi_irq < 0) {
			dev_err(&pdev->dev, "failed to get msi irq\n");
			return pp->msi_irq;
		}

		ret = devm_request_irq(&pdev->dev, pp->msi_irq,
				       sprd_pcie_msi_irq_handler,
				       IRQF_SHARED | IRQF_NO_THREAD,
				       "sprd-pcie-msi", pp);
		if (ret) {
			dev_err(&pdev->dev, "cannot request msi irq\n");
			return ret;
		}
	}

	return dw_pcie_host_init(pp);
}

static int sprd_pcie_probe(struct platform_device *pdev)
{
	struct sprd_pcie *sprd_pcie;
	struct pcie_port *pp;
	int ret;

	sprd_pcie = devm_kzalloc(&pdev->dev, sizeof(*sprd_pcie), GFP_KERNEL);
	if (!sprd_pcie)
		return -ENOMEM;

	pp = &sprd_pcie->pp;
	pp->dev = &pdev->dev;

	ret = sprd_pcie_get_resource(sprd_pcie, pdev);
	if (ret)
		return ret;

	ret = sprd_pcie_power_on(sprd_pcie);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, sprd_pcie);

	ret = sprd_add_pcie_port(pp, pdev);
	if (ret) {
		dev_err(&pdev->dev, "cannot initialize host\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id sprd_pcie_of_match[] = {
	{ .compatible = "sprd,pcie"},
	{ }
};

static struct platform_driver sprd_pcie_driver = {
	.probe = sprd_pcie_probe,
	.driver = {
		.name = "sprd-pcie",
		.suppress_bind_attrs = true,
		.of_match_table = sprd_pcie_of_match,
	},
};

builtin_platform_driver(sprd_pcie_driver);
