/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_graph.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <video/of_display_timing.h>

#include "disp_lib.h"
#include "disp_notify.h"
#include "sprd_panel.h"
#include "zte_lcd_common.h"
#ifndef CONFIG_ZTE_LCD_COMMON_FUNCTION
#include "sprd_spi_intf.h"
#else
extern struct sprd_dispc *g_zte_ctrl_pdata;
#endif
#include "sysfs/sysfs_display.h"

static struct device_node *oled_node;
static struct panel_device *panels[SPRD_MAX_LCD_ID];
static uint32_t lcd_id_from_uboot;
static uint32_t pixel_clk_from_uboot;
static struct sprd_oled_device *oled_bl;

static int __init lcd_id_get(char *str)
{
	int len = 0;

	if ((str != NULL) && (str[0] == 'I') && (str[1] == 'D'))
		len = kstrtou32(str+2, 16, &lcd_id_from_uboot);
	pr_info("LCD ID from uboot: 0x%x\n", lcd_id_from_uboot);
	return 0;
}
__setup("lcd_id=", lcd_id_get);

static int __init pixel_clk_get(char *str)
{
	int len = 0;

	len = kstrtou32(str, 10, &pixel_clk_from_uboot);
	pr_info("pixel clock from uboot: %u\n", pixel_clk_from_uboot);
	return 0;
}
__setup("pixel_clock=", pixel_clk_get);

static int panel_gpio_request(struct power_sequence *seq)
{
	unsigned int items;
	struct gpio_timing *timing;
	int i;

	items = seq->items;
	timing = seq->timing;

	for (i = 0; i < items; i++)
		gpio_request(timing[i].gpio, NULL);

	return 0;
}

extern int tpd_gpio_shutdown_config(void);
void panel_power_ctrl(struct power_sequence *seq)
{
	unsigned int items;
	struct gpio_timing *timing;
	int i;

	items = seq->items;
	timing = seq->timing;

	for (i = 0; i < items; i++) {
		gpio_direction_output(timing[i].gpio, timing[i].level);
		mdelay(timing[i].delay);
	}
#if defined(CONFIG_ZTE_LCD_COMMON_FUNCTION) && defined(CONFIG_ZTE_TOUCHSCREEN)
	if (g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_powerdown_for_shutdown) {
		pr_info("[SPRD_LCD]panel_power_ctrl: only shutdonwfor TP\n");
		tpd_gpio_shutdown_config();
	}
#endif
}

static void panel_suspend(struct panel_device *pd)
{
	struct panel_info *panel = pd->panel;

	if (panel == NULL)
		return;

	if (pd->esd_work_start) {
		cancel_delayed_work_sync(&pd->esd_work);
		pd->esd_work_start = false;
		pr_info("cancel ESD work queue!\n");
	}

	disp_notifier_call_chain(DISP_EVENT_DISPC_STOP, NULL);

	if (pd->ops && pd->ops->sleep_in)
		pd->ops->sleep_in(pd);

	panel_power_ctrl(&panel->pwr_off_seq);

	regulator_disable(panel->supply);

	pr_info("[SPRD_LCD] panel suspend OK\n");
}

static void panel_resume(struct panel_device *pd)
{
	struct panel_info *panel = pd->panel;
	int ret;

	if (panel == NULL)
		return;

	ret = regulator_enable(panel->supply);
	if (ret < 0)
		pr_err("enable lcd regulator failed\n");

	panel_power_ctrl(&panel->pwr_on_seq);

	if (pd->ops && pd->ops->init)
		pd->ops->init(pd);

	pr_info("[SPRD_LCD] panel resume OK\n");

	if (panel->type == SPRD_PANEL_TYPE_SPI)
		disp_notifier_call_chain(DISP_EVENT_DISPC_BLACK, NULL);

	disp_notifier_call_chain(DISP_EVENT_DISPC_RUN, NULL);

	if (panel->esd_check_en) {
		pr_info("schedule ESD work queue!\n");
		schedule_delayed_work(&pd->esd_work, msecs_to_jiffies
				      (panel->esd_timeout));
		pd->esd_work_start = true;
	}
}

static void esd_work_func(struct work_struct *work)
{
	struct panel_device *pd = container_of(work, struct panel_device,
						esd_work.work);
	struct panel_info *panel = pd->panel;
	struct device *dev = pd->dev.parent;
	static uint32_t count;
	int ret = 0;
	#ifdef CONFIG_ZTE_LCD_ESD_ERROR_CTRL
	static uint32_t esd_read_error_num = 0;
	#endif

	if (++count % 60 == 0)
		pr_err("ESD count: %d\n", count);

	if (panel->esd_check_en == ESD_MODE_WAIT_TE) {
		int rc;

		panel->te_esd_waiter = true;
		/*wait for esd TE interrupt*/
		rc = wait_event_interruptible_timeout(panel->wq_te_esd,
				panel->te_esd_flag, msecs_to_jiffies(500));
		panel->te_esd_flag = false;
		panel->te_esd_waiter = false;

		#ifdef CONFIG_ZTE_LCD_ESD_ERROR_CTRL
		if (!rc) {
			esd_read_error_num++;
			pr_err("TE esd timeout:esd_read_error_num=%d\n", esd_read_error_num);
			if (esd_read_error_num >= 2) {
				esd_read_error_num = 0;
				g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_esd_num++;
				ret = -1;
			} else {
				ret = 0;
			}
		} else {
			esd_read_error_num = 0;
			ret = 0;
		}
		#else
		if (!rc) {
			pr_err("TE esd timeout\n");
			ret = -1;
		}
		#endif
	} else if (pd->ops && pd->ops->esd_check)
		ret = pd->ops->esd_check(pd);

	/*
	 * If the panel_runtime_suspend() is being excuted, ignore
	 * the esd check result.
	 */
	if (dev->power.runtime_status == RPM_SUSPENDING)
		return;

	if (ret) {
		pd->esd_work_start = false;
		pr_err("====== esd recovery start ========\n");
		disp_notifier_call_chain(DISP_EVENT_ESD_RECOVER, (void *)0);
		disp_notifier_call_chain(DISP_EVENT_ESD_RECOVER, (void *)1);
		if (panel->is_oled)
			oled_bl->bd->ops->update_status(oled_bl->bd);
		pr_err("======= esd recovery end =========\n");
		count = 0;
	} else {
		schedule_delayed_work(&pd->esd_work,
				      msecs_to_jiffies(panel->esd_timeout));
	}
}

static int of_parse_power_seq(struct device_node *node,
				struct panel_info *panel)
{
	struct property *prop;
	int bytes;
	unsigned int *p;
	int rc;

	prop = of_find_property(node, "power-on-sequence", &bytes);
	if (!prop) {
		pr_err("error: power-on-sequence property not found\n");
		return -EINVAL;
	}

	p = kzalloc(bytes, GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	rc = of_property_read_u32_array(node,
				"power-on-sequence", p, bytes / 4);
	if (rc) {
		pr_err("get power-on-sequence failed\n");
		kfree(p);
		return rc;
	}

	panel->pwr_on_seq.items = bytes / 12;
	panel->pwr_on_seq.timing = (struct gpio_timing *)p;

	prop = of_find_property(node, "power-off-sequence", &bytes);
	if (!prop) {
		pr_err("error: power-off-sequence property not found");
		return -EINVAL;
	}

	p = kzalloc(bytes, GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	rc = of_property_read_u32_array(node,
				"power-off-sequence", p, bytes / 4);
	if (rc) {
		pr_err("get power-off-sequence failed\n");
		kfree(p);
		return rc;
	}

	panel->pwr_off_seq.items = bytes / 12;
	panel->pwr_off_seq.timing = (struct gpio_timing *)p;

	return 0;
}

int of_parse_cmds(struct device_node *np,
			struct panel_cmds **in_pcmds, const char *name)
{
	const void *base;
	unsigned char *buf, *bp;
	struct dsi_cmd_header *header;
	struct panel_cmds *pcmds;
	int blen = 0, len;
	int i, total = 0;

	if (!in_pcmds)
		return -EINVAL;

	base = of_get_property(np, name, &blen);
	if (!base)
		return -EINVAL;

	pcmds = kzalloc(sizeof(struct panel_cmds), GFP_KERNEL);
	if (!pcmds)
		return -ENOMEM;

	*in_pcmds = pcmds;

	buf = kzalloc(blen, GFP_KERNEL);
	if (!buf)
		goto err0;

	memcpy(buf, base, blen);

	for (i = 0; i < blen; i += 4)
		pr_debug("sprd panel cmds  %2x  %2x %2x %2x\n",
			 buf[i], buf[i + 1], buf[i + 2], buf[i + 3]);
	/* scan dcs commands */
	bp = buf;
	len = blen;
	pr_debug("sprd is panel cmds len %d\n", len);
	total = 0;
	while (len >= sizeof(struct dsi_cmd_header)) {
		header = (struct dsi_cmd_header *)bp;
		header->len = ntohs(header->len);
		if (header->len > len) {
			pr_err("%s: dsi total =%2x wait=%x error, len=%x\n",
			       __func__, total, header->wait, header->len);
			goto err1;
		}
		bp += sizeof(*header);
		len -= sizeof(*header);
		bp += header->len;
		len -= header->len;
		total++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!", __func__, buf[0], blen);
		goto err1;
	}

	pcmds->cmds = kcalloc(total, sizeof(struct dsi_cmd_desc), GFP_KERNEL);
	if (!pcmds->cmds)
		goto err2;

	pcmds->cmd_total = total;
	pcmds->cmd_base = buf;
	pcmds->byte_len = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < total; i++) {
		header = (struct dsi_cmd_header *)bp;
		len -= sizeof(*header);
		bp += sizeof(*header);
		pcmds->cmds[i].header = *header;
		pcmds->cmds[i].payload = bp;
		bp += header->len;
		len -= header->len;
	}

	return 0;

err2:
	kfree(pcmds->cmds);
err1:
	kfree(buf);
err0:
	kfree(pcmds);
	*in_pcmds = NULL;

	return -ENOMEM;
}

static struct panel_info *of_parse_panel(struct device_node *panel_node)
{
	unsigned int temp;
	int rc;
	int i;
	struct panel_info *panel;
	const char *panel_name;
	struct panel_cmds *pcmds = NULL;
	int count = 0;

	panel = kzalloc(sizeof(struct panel_info), GFP_KERNEL);
	if (panel == NULL)
		return NULL;

	panel->of_node = panel_node;
	panel->pixel_clk = pixel_clk_from_uboot;

	rc = of_property_read_u32(panel_node, "if-type", &temp);
	if (!rc)
		panel->type = temp;
	else
		panel->type = SPRD_PANEL_TYPE_MIPI;

	if (panel->type == SPRD_PANEL_TYPE_SPI) {

		rc = of_property_read_u32(panel_node, "spi_bus_num", &temp);
		if (!rc)
			panel->spi_bus_num = temp;

		rc = of_property_read_u32(panel_node, "spi_cs", &temp);
		if (!rc)
			panel->spi_cs = temp;

		rc = of_property_read_u32(panel_node, "spi_mode", &temp);
		if (!rc)
			panel->spi_mode = temp;

		rc = of_property_read_u32(panel_node, "spi_te_gpio", &temp);
		if (!rc)
			panel->spi_te_gpio = temp;

		rc = of_property_read_u32(panel_node, "spi_pol_mode", &temp);
		if (!rc)
			panel->spi_pol_mode = temp;

		rc = of_property_read_u32(panel_node, "spi_cd_gpio", &temp);
		if (!rc) {
			panel->spi_cd_gpio = temp;
			gpio_request(panel->spi_cd_gpio, "GPIO_CD");
			gpio_direction_output(panel->spi_cd_gpio, 0);
		} else
			panel->spi_cd_gpio = 0;

		rc = of_property_read_u32(panel_node, "spi_endian", &temp);
		if (!rc)
			panel->spi_endian = temp;

		rc = of_property_read_u32(panel_node, "hactive", &temp);
		if (!rc)
			panel->width = temp;

		rc = of_property_read_u32(panel_node, "vactive", &temp);
		if (!rc)
			panel->height = temp;

		rc = of_property_read_u32(panel_node, "spi_freq", &temp);
		if (!rc)
			panel->spi_freq = temp;

		rc = of_parse_cmds(panel_node, &pcmds, "enable-te");
		if (!rc)
			panel->cmd_codes[CMD_SPI_ENABLE_TE] = pcmds;
		else
			pr_err("enable-te cmd not found\n");

		rc = of_parse_cmds(panel_node, &pcmds, "disable-te");
		if (!rc)
			panel->cmd_codes[CMD_SPI_DISABLE_TE] = pcmds;
		else
			pr_err("disable-te cmd not found\n");
	}

	rc = of_property_read_u32(panel_node, "work-mode", &temp);
	if (!rc)
		panel->work_mode = temp;
	else
		panel->work_mode = SPRD_MIPI_MODE_CMD;

	rc = of_property_read_u32(panel_node, "burst-mode", &temp);
	if (!rc)
		panel->burst_mode = temp;
	else
		panel->burst_mode = PANEL_VIDEO_BURST_MODE;

	rc = of_property_read_u32(panel_node, "bpp", &temp);
	if (!rc)
		panel->bpp = temp;
	else
		panel->bpp = 24;

	rc = of_property_read_u32(panel_node, "lane-number", &temp);
	if (!rc)
		panel->lane_num = temp;
	else
		panel->lane_num = 4;

	rc = of_property_read_u32(panel_node, "dln-timer", &temp);
	if (!rc)
		panel->dln_timer = temp;
	else
		panel->dln_timer = 0x39;

	rc = of_property_read_u32(panel_node, "hsync-pol", &temp);
	if (!rc)
		panel->h_sync_pol = temp;

	rc = of_property_read_u32(panel_node, "vsync-pol", &temp);
	if (!rc)
		panel->v_sync_pol = temp;

	rc = of_property_read_u32(panel_node, "de-pol", &temp);
	if (!rc)
		panel->de_pol = temp;

	rc = of_property_read_u32(panel_node, "te-pol", &temp);
	if (!rc)
		panel->te_pol = temp;

	rc = of_property_read_u32(panel_node, "need-check-esd", &temp);
	if (!rc)
		panel->esd_check_en = temp;

	rc = of_property_read_u32(panel_node, "esd-timeout", &temp);
	if (!rc)
		panel->esd_timeout = temp;
	else
		panel->esd_timeout = 2000;

	rc = of_property_read_u32(panel_node, "esd-check-reg", &temp);
	if (!rc)
		panel->esd_reg = temp;
	else
		panel->esd_reg = 0x0A;

	count = of_property_count_u32_elems(panel_node, "esd-return-code");
	if (count > 0) {
		panel->esd_read_count = count;
		rc = of_property_read_u32_array(panel_node, "esd-return-code",
					panel->esd_return_code, count);
		if (rc) {
			pr_err("get esd-return-code failed\n");
			panel->esd_read_count = 0;
		}
	}

	rc = of_property_read_u32(panel_node, "dsc-compression", &temp);
	if (!rc)
		panel->dsc_en = temp;

	rc = of_property_read_u32(panel_node, "bv3-compression", &temp);
	if (!rc)
		panel->bv3_en = temp;

	rc = of_property_read_u32(panel_node, "fps", &temp);
	if (!rc)
		panel->fps = temp;
	else
		panel->fps = 60;

	rc = of_property_read_u32(panel_node, "non-continue-clk-en", &temp);
	if (!rc)
		panel->nc_clk_en = temp;

	rc = of_property_read_u32(panel_node, "width-mm", &temp);
	if (!rc)
		panel->width_mm = temp;
	else
		panel->width_mm = 68;

	rc = of_property_read_u32(panel_node, "height-mm", &temp);
	if (!rc)
		panel->height_mm = temp;
	else
		panel->height_mm = 121;

	rc = of_property_read_u32(panel_node, "low-res-simu", &temp);
	if (!rc)
		panel->low_res_simu = temp;

	rc = of_property_read_u32(panel_node, "simu-width", &temp);
	if (!rc)
		panel->simu_width = temp;
	else
		panel->simu_width = 720;

	rc = of_property_read_u32(panel_node, "simu-height", &temp);
	if (!rc)
		panel->simu_height = temp;
	else
		panel->simu_height = 1280;

	rc = of_property_read_string(panel_node, "panel-name", &panel_name);
	if (!rc)
		panel->lcd_name = panel_name;

	rc = of_parse_power_seq(panel_node, panel);
	if (!rc)
		panel_gpio_request(&panel->pwr_off_seq);
	else
		pr_err("get power sequence form dts error\n");

	rc = of_parse_cmds(panel_node, &pcmds, "init-data");
	if (!rc)
		panel->cmd_codes[CMD_CODE_INIT] = pcmds;
	else
		pr_err("init-data cmd not found\n");

	rc = of_parse_cmds(panel_node, &pcmds, "sleep-out");
	if (!rc)
		panel->cmd_codes[CMD_CODE_SLEEP_OUT] = pcmds;
	else
		pr_err("sleep-out cmd not found\n");

	rc = of_parse_cmds(panel_node, &pcmds, "sleep-in");
	if (!rc)
		panel->cmd_codes[CMD_CODE_SLEEP_IN] = pcmds;
	else
		pr_err("sleep-in cmd not found\n");

	for (i = 0; i < CMD_CODE_MAX - CMD_CODE_RESERVED0; i++) {
		char name[32] = {};

		sprintf(name, "code-reserved%d", i);
		rc = of_parse_cmds(panel_node, &pcmds, name);
		if (!rc)
			panel->cmd_codes[i + CMD_CODE_RESERVED0] = pcmds;
	}

	if (of_get_display_timing(panel_node, "display-timings",
				  &panel->display_timing))
		pr_err("get display timing failed\n");
	else {
		struct display_timing *dt = &panel->display_timing;
		struct rgb_timing *t = &panel->rgb_timing;

		t->hfp = dt->hfront_porch.typ;
		t->hbp = dt->hback_porch.typ;
		t->hsync = dt->hsync_len.typ;

		t->vfp = dt->vfront_porch.typ;
		t->vbp = dt->vback_porch.typ;
		t->vsync = dt->vsync_len.typ;

		panel->phy_freq = dt->pixelclock.typ;
		panel->width = dt->hactive.typ;
		panel->height = dt->vactive.typ;
	}

	oled_node = of_get_child_by_name(panel_node, "oled-backlight");
	if (oled_node)
		panel->is_oled = true;

	pr_info("lane_number = %d\n", panel->lane_num);
	pr_info("phy_freq = %d\n", panel->phy_freq);
	pr_info("resolution: %d x %d\n", panel->width, panel->height);

	return panel;
}

static int of_parse_oled(struct device_node *np,
		struct sprd_oled_device *oled)
{
	unsigned int temp;
	struct panel_cmds *pcmds = NULL;
	struct panel_info *panel = oled->pd->panel;
	int rc;

	if (!of_property_read_u32(np, "default-brightness", &temp))
		oled->bd->props.brightness = temp;

	if (!of_property_read_u32(np, "max-level", &temp))
		oled->max_level = temp;

	rc = of_parse_cmds(np, &pcmds, "brightness-levels");
	if (!rc)
		panel->cmd_codes[CMD_OLED_BRIGHTNESS] = pcmds;
	else
		pr_err("oled brightness-levels error\n");

	rc = of_parse_cmds(np, &pcmds, "reg-lock");
	if (!rc)
		panel->cmd_codes[CMD_OLED_REG_LOCK] = pcmds;
	else
		pr_err("oled reg-lock error\n");

	rc = of_parse_cmds(np, &pcmds, "reg-unlock");
	if (!rc)
		panel->cmd_codes[CMD_OLED_REG_UNLOCK] = pcmds;
	else
		pr_err("oled reg-unlock error\n");

	return 0;
}

static int oled_set_brightness(struct backlight_device *bd)
{
	int ret = 0;
	int level;
	int brightness;

	struct sprd_oled_device *oled = bl_get_data(bd);
	struct panel_device *pd = oled->pd;
	struct panel_ops *ops = pd->ops;

	brightness = bd->props.brightness;
	level = brightness * oled->max_level / 255;

	if (ops->set_brightness) {
		if (ops->set_brightness(oled->pd, level)) {
			pr_err("lcd brightness setting failed.\n");
			return -EIO;
		}
	}

	return ret;
}

static const struct backlight_ops oled_backlight_ops = {
	.update_status = oled_set_brightness,
};

static int oled_backlight_init(struct panel_device *pd)
{
	struct sprd_oled_device *oled;

	oled = devm_kzalloc(&pd->dev,
			sizeof(struct sprd_oled_device), GFP_KERNEL);
	if (!oled)
		return -ENOMEM;

	oled->bd = devm_backlight_device_register(&pd->dev,
			"sprd_backlight", &pd->dev, oled,
			&oled_backlight_ops, NULL);
	if (IS_ERR(oled->bd)) {
		dev_err(&pd->dev, "failed to register backlight ops.\n");
		return PTR_ERR(oled->bd);
	}

	oled->bd->props.max_brightness = 255;
	oled->pd = pd;
	of_parse_oled(oled_node, oled);
	oled_bl = oled;

	return 0;
}

static struct panel_ops *panel_ops_attach(int type)
{
	switch (type) {
	case SPRD_PANEL_TYPE_MIPI:
		return &panel_ops_mipi;

	case SPRD_PANEL_TYPE_RGB:
		return &panel_ops_rgb;

	case SPRD_PANEL_TYPE_SPI:
		return &panel_ops_spi;
	default:
		pr_err("doesn't support current interface type %d\n", type);
		return NULL;
	}
}

static struct panel_info *panel_info_attach(struct device_node *np,
					 int panel_id)
{
	int i;
	int count;
	struct device_node *panel_node = NULL;
	char uboot_name[20] = {};

	if (panel_id == 0) {
		pr_err("LCD_ID from uboot is 0, enter Calibration Mode\n");
		return NULL;
	}

	sprintf(uboot_name, "lcd%x", panel_id);

	count = of_property_count_elems_of_size(np, "panel-drivers", 4);
	if ((count == -EINVAL) || (count == -ENODATA)) {
		pr_err("not found panel-drivers property\n");
		return NULL;
	}

	for (i = 0; i < count; i++) {
		panel_node = of_parse_phandle(np, "panel-drivers", i);
		if (!panel_node) {
			pr_err("parse panel-driver %d error\n", i);
			continue;
		}

		if (!strcmp(panel_node->name, uboot_name)) {
			pr_info("found %s panel success!\n", panel_node->name);
			break;
		}
	}

	if (i == count) {
		pr_err("get panel failed\n");
		return NULL;
	}

	return of_parse_panel(panel_node);
}

static int panel_device_register(struct panel_device *pd,
			struct device *parent, int index)
{
	pd->dev.class = display_class;
	pd->dev.parent = parent;
	pd->dev.of_node = pd->panel->of_node;
	dev_set_name(&pd->dev, "panel%d", index);
	dev_set_drvdata(&pd->dev, pd);

	return device_register(&pd->dev);
}

static int sprd_panel_probe(struct platform_device *pdev)
{
	int index;
	int ret;
	uint32_t temp;
	struct panel_info *panel;
	struct panel_device *pd;
	struct device *dev;
	int panel_id[2] = {lcd_id_from_uboot, 0};

	if (!of_property_read_u32(pdev->dev.of_node, "force-id", &temp)) {
		pr_info("warning: use force id 0x%x\n", temp);
		panel_id[0] = temp;
	}

	if (panel_id[0] == 0) {
		pr_err("LCD_ID from uboot is 0, enter Calibration Mode\n");
		return 0;
	}

	for (index = 0; (index < 2) && panel_id[index]; index++) {

		pd = kzalloc(sizeof(struct panel_device), GFP_KERNEL);
		if (pd == NULL)
			return -ENOMEM;

		panel = panel_info_attach(pdev->dev.of_node, panel_id[index]);
		if (panel == NULL) {
			pr_err("attach panel info failed\n");
			goto err;
		}
		pd->panel = panel;
		panel->pd = pd;

		dev = dev_get_prev(&pdev->dev);
		if (dev) {
			dev_set_drvdata(dev, panel);
			if (panel->type == SPRD_PANEL_TYPE_MIPI)
				dev = dev_get_prev(dev);
			pd->intf = dev;
		}

		pd->ops = panel_ops_attach(panel->type);
		if (pd->ops == NULL) {
			pr_err("attach panel ops failed\n");
			goto err;
		}

		ret = panel_device_register(pd, &pdev->dev, index);
		if (ret) {
			pr_err("register panel device failed\n");
			goto err;
		}

		panel->supply = devm_regulator_get(&pd->dev, "power");
		if (IS_ERR(panel->supply))
			pr_err("get lcd regulator failed\n");

		init_waitqueue_head(&panel->wq_te_esd);
		INIT_DELAYED_WORK(&pd->esd_work, esd_work_func);
		if (panel->esd_check_en) {
			pr_info("schedule ESD work queue!\n");
			panel->te_esd_flag = false;
			schedule_delayed_work(&pd->esd_work,
					      msecs_to_jiffies
					      (panel->esd_timeout));
			pd->esd_work_start = true;
		}

		if (panel->is_oled)
			oled_backlight_init(pd);

		sprd_panel_sysfs_init(&pd->dev);

		panels[index] = pd;

		continue;

err:
		kfree(pd);
		return -ENODEV;
	}

	pr_info("panel probe sucvess\n");
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return 0;
}

static int panel_runtime_suspend(struct device *dev)
{
	int index;

	for (index = 0; index < SPRD_MAX_LCD_ID; index++) {
		struct panel_device *pd = panels[index];

		if (pd)
			panel_suspend(pd);
	}
	return 0;
}

static int panel_runtime_resume(struct device *dev)
{
	int index;

	for (index = 0; index < SPRD_MAX_LCD_ID; index++) {
		struct panel_device *pd = panels[index];

		if (pd)
			panel_resume(pd);
	}
	return 0;
}

static const struct dev_pm_ops panel_pm_ops = {
	.runtime_suspend = panel_runtime_suspend,
	.runtime_resume = panel_runtime_resume,
};

static const struct of_device_id panel_dt_ids[] = {
	{ .compatible = "sprd-panel-if", },
	{}
};

static struct platform_driver sprd_panel_driver = {
	.probe = sprd_panel_probe,
	.driver = {
		   .name = "sprd-panel-if",
		   .of_match_table = of_match_ptr(panel_dt_ids),
		   .pm = &panel_pm_ops,
	},
};

module_platform_driver(sprd_panel_driver);

MODULE_DESCRIPTION("Panel Interface Driver");
MODULE_LICENSE("GPL");
