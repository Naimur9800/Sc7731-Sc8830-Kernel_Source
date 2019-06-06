/**
 * host.c - DesignWare USB3 DRD Controller Host Glue
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/usb/xhci_pdriver.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "core.h"
#include "../host/xhci.h"

#define DWC3_HOST_SUSPEND_COUNT		100
#define DWC3_HOST_SUSPEND_TIMEOUT	100

int dwc3_host_init(struct dwc3 *dwc)
{
	struct platform_device	*xhci;
	struct usb_xhci_pdata	pdata;
	int			ret;

	xhci = platform_device_alloc("xhci-hcd", PLATFORM_DEVID_AUTO);
	if (!xhci) {
		dev_err(dwc->dev, "couldn't allocate xHCI device\n");
		return -ENOMEM;
	}

	/*
	 * FIXME: It seems we should set the dma operations firstly when setting
	 * the dma mask, otherwise it will set dma mask failed.
	 */
	if (get_dma_ops(&xhci->dev) == get_dma_ops(NULL))
		xhci->dev.archdata.dma_ops = get_dma_ops(dwc->dev);

	dma_set_coherent_mask(&xhci->dev, dwc->dev->coherent_dma_mask);

	xhci->dev.parent	= dwc->dev;
	xhci->dev.dma_mask	= dwc->dev->dma_mask;
	xhci->dev.dma_parms	= dwc->dev->dma_parms;

	dwc->xhci = xhci;

	ret = platform_device_add_resources(xhci, dwc->xhci_resources,
						DWC3_XHCI_RESOURCES_NUM);
	if (ret) {
		dev_err(dwc->dev, "couldn't add resources to xHCI device\n");
		goto err1;
	}

	memset(&pdata, 0, sizeof(pdata));

	pdata.usb3_lpm_capable = dwc->usb3_lpm_capable;
	pdata.usb3_slow_suspend = 1;

	ret = platform_device_add_data(xhci, &pdata, sizeof(pdata));
	if (ret) {
		dev_err(dwc->dev, "couldn't add platform data to xHCI device\n");
		goto err1;
	}

	phy_create_lookup(dwc->usb2_generic_phy, "usb2-phy",
			  dev_name(&xhci->dev));
	phy_create_lookup(dwc->usb3_generic_phy, "usb3-phy",
			  dev_name(&xhci->dev));

	ret = platform_device_add(xhci);
	if (ret) {
		dev_err(dwc->dev, "failed to register xHCI device\n");
		goto err2;
	}

	return 0;
err2:
	phy_remove_lookup(dwc->usb2_generic_phy, "usb2-phy",
			  dev_name(&xhci->dev));
	phy_remove_lookup(dwc->usb3_generic_phy, "usb3-phy",
			  dev_name(&xhci->dev));
err1:
	platform_device_put(xhci);
	return ret;
}

void dwc3_host_exit(struct dwc3 *dwc)
{
	phy_remove_lookup(dwc->usb2_generic_phy, "usb2-phy",
			  dev_name(&dwc->xhci->dev));
	phy_remove_lookup(dwc->usb3_generic_phy, "usb3-phy",
			  dev_name(&dwc->xhci->dev));
	platform_device_unregister(dwc->xhci);
}

int dwc3_host_suspend(struct dwc3 *dwc)
{
	struct device *xhci = &dwc->xhci->dev;
	int ret, cnt = DWC3_HOST_SUSPEND_COUNT;

	if (!dwc->host_suspend_capable)
		return 0;

	/*
	 * We need make sure the children of the xhci device had been into
	 * suspend state, or we will suspend xhci device failed.
	 */
	while (!pm_children_suspended(xhci) && --cnt > 0)
		msleep(DWC3_HOST_SUSPEND_TIMEOUT);

	if (cnt <= 0) {
		dev_err(xhci, "failed to suspend xHCI children device\n");
		return -EBUSY;
	}

	/*
	 * If the xhci device had been into suspend state, thus just return.
	 */
	if (pm_runtime_suspended(xhci))
		return 0;

	/* Suspend the xhci device synchronously. */
	ret = pm_runtime_put_sync(xhci);
	if (ret) {
		dev_err(xhci, "failed to suspend xHCI device\n");
		return ret;
	}

	return 0;
}

int dwc3_host_resume(struct dwc3 *dwc)
{
	struct device *xhci = &dwc->xhci->dev;
	int ret;

	if (!dwc->host_suspend_capable)
		return 0;

	/* Resume the xhci device synchronously. */
	ret = pm_runtime_get_sync(xhci);
	if (ret) {
		dev_err(xhci, "failed to resume xHCI device\n");
		return ret;
	}

	return 0;
}
