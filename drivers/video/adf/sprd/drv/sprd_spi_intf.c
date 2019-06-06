/*
 *Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 *This software is licensed under the terms of the GNU General Public
 *License version 2, as published by the Free Software Foundation, and
 *may be copied, distributed, and modified under those terms.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqreturn.h>
#include <linux/kgdb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_graph.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include "disp_notify.h"
#include "sprd_spi_intf.h"
#include "sysfs/sysfs_display.h"

static uint8_t *pframe;
static bool first_alloc = true;

static struct sprd_spi *spi;
static unsigned char sprd_refresh_cmd[] = { 0x2a, 0x2b, 0x2c };
static unsigned char sprd_refresh_data[][4] = { { 0x00, 0x00, 0x00, 0xef },
					{ 0x00, 0x00, 0x01, 0x3f } };

static struct spi_transfer sprd_refresh_transfers[] = {
	{
		.len = 1,
		.tx_buf = &sprd_refresh_cmd[0],
		.bits_per_word = 8,
	},
	{
		.len = 4,
		.tx_buf = &sprd_refresh_data[0][0],
		.bits_per_word = 8,
	},
	{
		.len = 1,
		.tx_buf = &sprd_refresh_cmd[1],
		.bits_per_word = 8,
	},
	{
		.len = 4,
		.tx_buf = &sprd_refresh_data[1][0],
		.bits_per_word = 8,
	},
	{
		.len = 1,
		.tx_buf = &sprd_refresh_cmd[2],
		.bits_per_word = 8,
	},
};

static struct spi_message sprd_refresh_msg;
static struct spi_transfer sprd_refresh_xfer = {
	.bits_per_word = 32,
};


static uint16_t bgra888_to_rgb565(uint8_t b, uint8_t g, uint8_t r)
{
	uint16_t rgb565;

	rgb565 = (((r >> 3) & 0x1f) << 11) |
		(((g >> 2) & 0x3f) << 5) |
		(((b >> 3) & 0x1f) << 0);
	return rgb565;
}

static uint32_t spi_rgb_ctrl(u32 format)
{

	u32 rgb_f = 0;

	switch (format) {
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
		/* rb switch */
		rgb_f = 1;
		break;
	case DRM_FORMAT_BGRA8888:
		rgb_f = 2;
		break;
	case DRM_FORMAT_BGR888:
	case DRM_FORMAT_RGB888:
		pr_err("sharkle not support rgb888 format\n");
		break;
	default:
		break;
	}

	return rgb_f;
}

static int abgr32_to_rgb16(struct panel_info *panel, void *base)
{
	int i = 0;
	uint32_t *prgb32;
	uint16_t *prgb16;
	uint32_t rgb32;
	uint16_t rgb16;
	uint8_t r, g, b;
	uint16_t width, height;
	uint16_t line_bytes;

	width = panel->width;
	height = panel->height;
	line_bytes = (width * panel->bpp) / 8;
	if (first_alloc) {
		pframe = kzalloc(line_bytes * height, GFP_KERNEL);
		if (!pframe)
			return -ENOMEM;
		first_alloc = false;
	}
	prgb32 = (uint32_t *)base;
	prgb16 = (uint16_t *)pframe;
	for (i = 0; i < width * height; i++) {
		rgb32 = *prgb32++;
		r = rgb32 & 0xff;
		g = (rgb32 >> 8) & 0xff;
		b = (rgb32 >> 16) & 0xff;
		rgb16 = bgra888_to_rgb565(b, g, r);
		*prgb16 = rgb16;
		prgb16++;
	}

	return 0;
}

static int argb32_to_rgb16(struct panel_info *panel, void *base)
{
	int i = 0;
	uint32_t *prgb32;
	uint16_t *prgb16;
	uint32_t rgb32;
	uint16_t rgb16;
	uint8_t r, g, b;
	uint16_t width, height;
	uint16_t line_bytes;

	width = panel->width;
	height = panel->height;
	line_bytes = (width * panel->bpp) / 8;
	if (first_alloc) {
		pframe = kzalloc(line_bytes * height, GFP_KERNEL);
		if (!pframe)
			return -ENOMEM;
		first_alloc = false;
	}
	prgb32 = (uint32_t *)base;
	prgb16 = (uint16_t *)pframe;
	for (i = 0; i < width * height; i++) {
		rgb32 = *prgb32++;
		b = rgb32 & 0xff;
		g = (rgb32 >> 8) & 0xff;
		r = (rgb32 >> 16) & 0xff;
		rgb16 = bgra888_to_rgb565(b, g, r);
		*prgb16 = rgb16;
		prgb16++;
	}

	return 0;
}

static int copy_to_pframe(void *base, uint16_t stride, uint16_t line_bytes)
{
	int i = 0;
	uint8_t *pbase;
	uint8_t *ptemp;
	uint16_t height;

	pbase = base;
	height = spi->panel->height;
	if (first_alloc) {
		pframe = kzalloc(line_bytes * height, GFP_KERNEL);
		if (!pframe)
			return -ENOMEM;
		first_alloc = false;
	}
	ptemp = pframe;
	for (i = 0; i < height; i++)
		memcpy(ptemp + i * line_bytes,
			pbase + i * stride, line_bytes);

	return 0;
}

static int exchange_pixels(void *base)
{
	uint32_t i;
	uint16_t height;
	uint16_t width;
	uint16_t *pbase;

	pbase = (uint16_t *)base;
	height = spi->panel->height;
	width = spi->panel->width;

	for (i = 0; i < height * width; i += 2)
		swap(pbase[i], pbase[i+1]);

	return 0;
}

static int sprd_spi_config(struct panel_info *panel)
{
	uint16_t width, height, bpp;

	bpp = panel->bpp;
	width = panel->width;
	height = panel->height;
	spi->refresh_len = width * height * bpp / 8;
	sprd_refresh_xfer.len = spi->refresh_len;
	sprd_refresh_data[0][2] = ((width - 1) >> 8) & 0xff;
	sprd_refresh_data[0][3] = (width - 1) & 0xff;
	sprd_refresh_data[1][2] = ((height - 1) >> 8) & 0xff;
	sprd_refresh_data[1][3] = (height - 1) & 0xff;
	init_completion(&spi->spi_complete);
	spi->spi_complete.done = 1;
	return 0;
}

static int sprd_spi_dev_probe(struct spi_device *spi_dev)
{
	struct panel_info *panel = spi->panel;

	spi->spi_dev = spi_dev;
	spi_dev->max_speed_hz = panel->spi_freq;
	spi_dev->mode = panel->spi_mode;
	spi_dev->chip_select = panel->spi_cs;
	sprd_spi_config(spi->panel);
	spi_setup(spi_dev);
	device_init_wakeup(&spi_dev->dev, 1);
	spi_set_drvdata(spi_dev, spi);
	pr_info("spi drv probe success\n");
	return 0;
}

static int sprd_spi_dev_remove(struct spi_device *spi_dev)
{
	spi->spi_dev = NULL;
	spi_set_drvdata(spi_dev, NULL);
	return 0;
}

static const struct of_device_id spi_device_ids[] = {
	{ .compatible = "sprd,spi-panel" },
};
MODULE_DEVICE_TABLE(of, spi_device_ids);

static struct spi_driver spi_drv = {
	.driver = {
		.name = "spi-panel-drv",
		.owner = THIS_MODULE,
		.of_match_table = spi_device_ids,
	},
	.probe  = sprd_spi_dev_probe,
	.remove = sprd_spi_dev_remove,
};

static void sprd_spi_complete(void *context)
{
	complete_all(&spi->spi_complete);
}

static void sprd_spi_cmd(struct panel_info *panel, int n)
{
	struct spi_message msg;

	spi_message_init(&msg);
	if (panel->spi_cd_gpio)
		gpio_set_value(panel->spi_cd_gpio, 0);
	spi_message_add_tail(&sprd_refresh_transfers[n], &msg);
	spi_sync(spi->spi_dev, &msg);
}

static void sprd_spi_data(struct panel_info *panel, int n)
{
	struct spi_message msg;

	spi_message_init(&msg);
	if (panel->spi_cd_gpio)
		gpio_set_value(panel->spi_cd_gpio, 1);
	spi_message_add_tail(&sprd_refresh_transfers[n], &msg);
	spi_sync(spi->spi_dev, &msg);
}

static int sprd_spi_flip_bg(void)
{
	struct panel_info *panel;
	uint16_t width, height, line_bytes;
	unsigned char i = 0;
	int ret;

	panel = spi->panel;
	height = panel->height;
	width = panel->width;
	line_bytes = (width * panel->bpp) / 8;
	if (first_alloc) {
		pframe = kzalloc(line_bytes * height, GFP_KERNEL);
		if (!pframe)
			return -ENOMEM;
		first_alloc = false;
	}
	memset(pframe, 0, line_bytes * height);
	sprd_refresh_xfer.tx_buf = pframe;
	spi_message_init(&sprd_refresh_msg);
	spi->spi_complete.done = 1;
	sprd_refresh_msg.complete = sprd_spi_complete;
	if (panel->spi_cd_gpio)
		gpio_set_value(spi->panel->spi_cd_gpio, 1);
	spi_message_add_tail(&sprd_refresh_xfer, &sprd_refresh_msg);
	do {
		ret = spi_sync(spi->spi_dev, &sprd_refresh_msg);
		i++;
	} while ((ret < 0) && (i < 6));

	return 0;
}

static void sprd_spi_flip_layer(struct sprd_adf_hwlayer *hwlayer)
{
	struct panel_info *panel;
	unsigned int layer_base;
	void *layer_vbase;
	unsigned int layer_bpp;
	unsigned int rgbf;
	uint16_t width, height, line_bytes, stride;
	unsigned char i = 0;
	int ret;

	layer_bpp = adf_format_bpp(hwlayer->format);
	rgbf = spi_rgb_ctrl(hwlayer->format);
	panel = spi->panel;
	height = panel->height;
	width = panel->width;
	layer_base = hwlayer->iova_plane[0];
	layer_vbase = phys_to_virt(layer_base);
	line_bytes = (width * panel->bpp) / 8;
	stride = hwlayer->pitch[0];

	if (!layer_vbase)
		return;
	if (layer_bpp == 32) {
		if (rgbf == 1)
			abgr32_to_rgb16(panel, layer_vbase);
		else if (rgbf == 2)
			argb32_to_rgb16(panel, layer_vbase);
		else
			return;
		pr_info("layer base %x bpp %d\n", layer_base, layer_bpp);
		if (pframe)
			exchange_pixels(pframe);
		else
			return;
		sprd_refresh_xfer.tx_buf = pframe;
	} else {
		copy_to_pframe(layer_vbase, stride, line_bytes);
		if (pframe)
			exchange_pixels(pframe);
		else
			return;
		sprd_refresh_xfer.tx_buf = pframe;
		pr_info("layer base %x bpp %d\n", layer_base, layer_bpp);
	}
	spi_message_init(&sprd_refresh_msg);
	spi->spi_complete.done = 1;
	sprd_refresh_msg.complete = sprd_spi_complete;
	if (panel->spi_cd_gpio)
		gpio_set_value(spi->panel->spi_cd_gpio, 1);
	spi_message_add_tail(&sprd_refresh_xfer, &sprd_refresh_msg);
	do {
		ret = spi_sync(spi->spi_dev, &sprd_refresh_msg);
		i++;
	} while ((ret < 0) && (i < 6));
}

static int sprd_spi_transfer(struct sprd_adf_hwlayer *hwlayer, uint8_t bg_flag)
{
	struct panel_info *panel = spi->panel;

	if (!completion_done(&spi->spi_complete))
		wait_for_completion(&spi->spi_complete);
	sprd_spi_cmd(panel, 0);
	sprd_spi_data(panel, 1);
	sprd_spi_cmd(panel, 2);
	sprd_spi_data(panel, 3);
	sprd_spi_cmd(panel, 4);
	if (bg_flag)
		sprd_spi_flip_bg();
	else
		sprd_spi_flip_layer(hwlayer);

	return 0;
}

static void sprd_spi_read_id(void)
{
#if 0
	struct spi_device *spi_dev;
	struct panel_info *panel = spi->panel;

	spi_dev = spi->spi_dev;
	spi_dev->max_speed_hz = 24000000;
	spi_setup(spi_dev);
	sprd_spi_cmd(panel, 5);
	sprd_spi_cmd(panel, 6);
	sprd_spi_cmd(panel, 7);
	sprd_spi_data(panel, 8);
	spi_dev->max_speed_hz = 48000000;
	spi_setup(spi_dev);
#endif

}

int sprd_spi_refresh(struct sprd_adf_hwlayer *hwlayer, uint8_t bg_flag)
{
	int ret;

	ret = sprd_spi_transfer(hwlayer, bg_flag);
	sprd_spi_read_id();
	if (ret)
		pr_info("sprd:sprd_spi_refresh spi_async error\n");

	return ret;
}

static int spi_notify_callback(struct notifier_block *nb,
			unsigned long action, void *data)
{
	struct sprd_spi *spi = container_of(nb, struct sprd_spi, nb);
	struct device *dev = spi->dev.parent;
	struct panel_info *panel = spi->panel;
	unsigned long enable;

	if (!panel) {
		pr_err("panel pointer is null\n");
		return NOTIFY_DONE;
	}

	switch (action) {
	case DISP_EVENT_ESD_RECOVER:
		enable = (unsigned long)data;
		if (enable)
			pm_runtime_get_sync(dev);
		else
			pm_runtime_put_sync(dev);
		break;
	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static int spi_notifier_register(struct sprd_spi *spi)
{
	spi->nb.notifier_call = spi_notify_callback;

	return disp_notifier_register(&spi->nb);
}

static int spi_device_register(struct sprd_spi *spi,
				struct device *parent)
{
	int ret;

	spi->dev.class = display_class;
	spi->dev.parent = parent;
	spi->dev.of_node = parent->of_node;
	dev_set_name(&spi->dev, "spi");
	dev_set_drvdata(&spi->dev, spi);

	ret = device_register(&spi->dev);
	if (ret)
		pr_err("spi device register failed\n");

	return ret;
}

static int spi_context_init(struct sprd_spi *spi, struct device_node *np)
{

	if (spi->glb && spi->glb->parse_dt)
		spi->glb->parse_dt(&spi->ctx, np);
	return 0;
}


static int sprd_spi_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dispc_dev;
	struct device_node *np = pdev->dev.of_node;

	spi = kzalloc(sizeof(struct sprd_spi), GFP_KERNEL);
	if (!spi)
		return -ENOMEM;
	spi->panel = platform_get_drvdata(pdev);
	if (spi->panel == NULL) {
		pr_err("error: spi->panel is null\n");
		goto err;
	}

	ret = spi_register_driver(&spi_drv);
	if (ret < 0)
		pr_err("spi driver register spi_drv fail\n");

	dispc_dev = dev_get_prev(&pdev->dev);
	if (dispc_dev)
		dev_set_drvdata(dispc_dev, spi->panel);
	if (spi_context_init(spi, np))
		goto err;

	spi_device_register(spi, &pdev->dev);
	spi_notifier_register(spi);
	platform_set_drvdata(pdev, spi);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	pr_info("spi probe success\n");
	return 0;

err:
	kfree(spi);
	return -ENODEV;
}

static int spi_runtime_resume(struct device *dev)
{
	struct sprd_spi *spi = dev_get_drvdata(dev);
	struct device *next = dev_get_next(dev);

	pr_info("spi_runtime_resume now\n");
	spi->spi_complete.done = 1;
	pm_runtime_get_sync(next);

	return 0;
}

static int spi_runtime_suspend(struct device *dev)
{

	struct sprd_spi *spi = dev_get_drvdata(dev);
	struct device *next = dev_get_next(dev);

	pr_info("spi_runtime_suspend  now\n");
	if (!completion_done(&spi->spi_complete))
		try_wait_for_completion(&spi->spi_complete);
	pm_runtime_put_sync(next);

	return 0;
}

static const struct dev_pm_ops spi_pm_ops = {
	.runtime_suspend = spi_runtime_suspend,
	.runtime_resume = spi_runtime_resume,
};

static const struct of_device_id dt_ids[] = {
	{ .compatible = "sprd,spi-intf", },
	{}
};

static struct platform_driver sprd_spi_driver = {
	.probe = sprd_spi_probe,
	.driver = {
		.name = "spi-intf-drv",
		.of_match_table = of_match_ptr(dt_ids),
		.pm = &spi_pm_ops,
	},
};

module_platform_driver(sprd_spi_driver);

MODULE_AUTHOR("junxiao.feng@spreadtrum.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SPRD SPI Interface Driver");
