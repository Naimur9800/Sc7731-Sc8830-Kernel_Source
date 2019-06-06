/*
 * Spreadtrum MIPI Panel Driver
 *
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"sprdfb: " fmt

#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include "sprdfb_panel.h"

/* Macro Declaration */
#define RX_SET 1
#define RX_UNSET 0
#define TX_SET 1
#define TX_UNSET 0

#define ERR 1

/* Data Structure */
struct panel_ctx {
	int panel_cnt;
	struct panel_cfg *panels_cfg;
};

struct sprd_panel_dt_param {
	int eotp_type;
};

/* Globals */
struct sprd_panel_dt_param sprd_panel_hx8394a_data = {.eotp_type = 1};
struct sprd_panel_dt_param sprd_panel_ssd2075_data = {.eotp_type = 0};

static const struct of_device_id sprd_panel_dt_ids[] = {
	{
		.name = "lcd_hx8394a_mipi",
		.data = &sprd_panel_hx8394a_data,
	},
	{
		.name = "lcd_ssd2075_mipi",
		.data = &sprd_panel_ssd2075_data,
	},
};

struct panel_ctx pctx;
struct panel_cfg *cur_panel_cfg;

static int32_t sprd_panel_mipi_init(struct panel_spec *self);
static uint32_t sprd_panel_readid(struct panel_spec *self);
static int32_t sprd_panel_check_esd(struct panel_spec *self);
static int32_t sprd_panel_enter_sleep(struct panel_spec *self,
					uint8_t is_sleep);

static struct panel_operations lcd_panel_mipi_ops = {
	.panel_init			= sprd_panel_mipi_init,
	.panel_readid		= sprd_panel_readid,
	.panel_esd_check	= sprd_panel_check_esd,
	.panel_enter_sleep	= sprd_panel_enter_sleep,
};

/* panel operations definition */
static int32_t sprd_panel_mipi_init(struct panel_spec *self)
{
	struct panel_cmds *on_cmds;
	unsigned int i;
	uint8_t rx_set, tx_set, rx_unset, tx_unset;
	int mipi_write_type = self->mipi_write_type;

	struct ops_mipi mipi_ops = {
		.mipi_set_cmd_mode	=
			self->info.mipi->ops->mipi_set_cmd_mode,
		.mipi_eotp_set		=
			self->info.mipi->ops->mipi_eotp_set,
		.mipi_dcs_write		=
			self->info.mipi->ops->mipi_dcs_write,
	};

	pr_info("lcd-panel: %s processing\n", __func__);

	if (mipi_write_type == 1) {
		rx_set = RX_UNSET;
		tx_set = TX_UNSET;
		rx_unset = RX_UNSET;
		tx_unset = TX_UNSET;
	} else {
		if (mipi_write_type == 0) {
			rx_set = RX_SET;
			tx_set = TX_UNSET;
			rx_unset = RX_SET;
			tx_unset = TX_SET;
		} else {
			goto err;
		}
	}

	mipi_ops.mipi_set_cmd_mode();
	mipi_ops.mipi_eotp_set(rx_set, tx_set);
	on_cmds = self->init_cmds;

	for (i = 0; i < on_cmds->cmd_total; i++) {
		mipi_ops.mipi_dcs_write(on_cmds->cmds[i].payload,
					on_cmds->cmds[i].header.len);

		if (on_cmds->cmds[i].header.wait)
			msleep(on_cmds->cmds[i].header.wait);
	}

	mipi_ops.mipi_eotp_set(rx_unset, tx_unset);
	return 0;
err:
	pr_err("lcd-panel: %s failed!\n", __func__);
	return -ERR;
}

static uint32_t sprd_panel_readid(struct panel_spec *self)
{
	uint32_t i, j;
	uint8_t read_data[10];
	uint8_t max_try = 4;
	int mipi_write_type = self->mipi_write_type;
	uint8_t rx_set, tx_set, rx_unset, tx_unset;

	struct ops_mipi mipi_ops = {
		.mipi_set_cmd_mode	=
			self->info.mipi->ops->mipi_set_cmd_mode,
		.mipi_eotp_set		=
			self->info.mipi->ops->mipi_eotp_set,
		.mipi_force_write	=
			self->info.mipi->ops->mipi_force_write,
		.mipi_force_read	=
			self->info.mipi->ops->mipi_force_read,
	};

	struct panel_cmds *on_cmds;
	struct panel_pair_cmds *read_cmds, *cmp_cmds;
	struct pair_cmds *on_read_cmds, *on_cmp_cmds;

	if (mipi_write_type == 1) {
		rx_set = RX_UNSET;
		tx_set = TX_UNSET;
		rx_unset = RX_UNSET;
		tx_unset = TX_UNSET;
	} else {
		if (mipi_write_type == 0) {
			rx_set = RX_SET;
			tx_set = TX_UNSET;
			rx_unset = RX_SET;
			tx_unset = TX_SET;
		} else {
			goto err;
		}
	}

	mipi_ops.mipi_set_cmd_mode();
	mipi_ops.mipi_eotp_set(rx_set, tx_set);

	if ((!self->readid_cmds->cmd_total)
		|| (!self->readid_rcmds->num_cmd)
		|| (!self->readid_cmp->num_cmd)
		|| (self->readid_cmp->num_cmd > 10)) {
		goto err;
	}

	for (i = 0; i < max_try; i++) {
		on_cmds = self->readid_cmds;
		/* readid Write */
		for (j = 0; j < on_cmds->cmd_total; j++) {
			mipi_ops.mipi_force_write(
				on_cmds->cmds[j].header.data_type,
				on_cmds->cmds[j].payload,
				on_cmds->cmds[j].header.len);
			if (on_cmds->cmds[j].header.wait)
				msleep(on_cmds->cmds[j].header.wait);
		}

		/* readid read */
		read_cmds = self->readid_rcmds;
		on_read_cmds = read_cmds->pair_cmd;

		if (read_cmds->num_cmd == 1) {
			mipi_ops.mipi_force_read(on_read_cmds->addr,
				on_read_cmds->bytes, (uint8_t *)&read_data);
		} else if (read_cmds->num_cmd == 3) {
			mipi_ops.mipi_force_read(
					on_read_cmds[0].addr,
					on_read_cmds[0].bytes,
					(uint8_t *)&read_data[0]);
			mipi_ops.mipi_force_read(
					on_read_cmds[1].addr,
					on_read_cmds[1].bytes,
					(uint8_t *)&read_data[1]);
			mipi_ops.mipi_force_read(
					on_read_cmds[2].addr,
					on_read_cmds[2].bytes,
					(uint8_t *)&read_data[2]);
		} else {
			pr_err(
			"lcd-panel: check readid read sequence and volume\n");
		}

		/* readid compare */
		cmp_cmds = self->readid_cmp;
		on_cmp_cmds = cmp_cmds->pair_cmd;

		for (j = 0; j < cmp_cmds->num_cmd; j++) {
			if (!(read_data[(int)on_cmp_cmds[j].addr]
				== on_cmp_cmds[j].bytes)) {
				pr_err("lcd-panel: readid comp fail");
				pr_err("read_data= %d, cmp_data= %d\n",
					read_data[(int)on_cmp_cmds[j].addr],
					on_cmp_cmds[j].bytes);
				goto err;
			}
		}

		pr_info("lcd-panel: readid success! 0x%x\n",
					cur_panel_cfg->lcd_id);
		mipi_ops.mipi_eotp_set(rx_unset, tx_unset);
		return cur_panel_cfg->lcd_id;
	}

	mipi_ops.mipi_eotp_set(rx_unset, tx_unset);
err:
	pr_err("lcd-panel: readid fail! 0x%x\n", cur_panel_cfg->lcd_id);
	return -ERR;
}

static int32_t sprd_panel_check_esd(struct panel_spec *self)
{
	struct panel_cmds *codes;
	int i = 0;
	uint8_t read_data[3] = {0};
	uint8_t rx_set, tx_set, rx_unset, tx_unset;

	struct ops_mipi mipi_ops = {
		.mipi_set_cmd_mode	=
			self->info.mipi->ops->mipi_set_cmd_mode,
		.mipi_eotp_set		=
			self->info.mipi->ops->mipi_eotp_set,
		.mipi_force_write	=
			self->info.mipi->ops->mipi_force_write,
		.mipi_force_read	=
			self->info.mipi->ops->mipi_force_read,
	};

	int mipi_write_type = self->mipi_write_type;

	if (!self->esd) {
		pr_err(" lcd-panel: esd not supported !");
		goto err;
	}

	codes = self->esd_cmds;

	if (!codes) {
		pr_err("lcd-panel: ESD Check CMD is NULL\n");
		goto err;
	}

	if (mipi_write_type == 1) {
		rx_set = RX_UNSET;
		tx_set = TX_UNSET;
		rx_unset = RX_UNSET;
		tx_unset = TX_UNSET;
	} else {
		rx_set = RX_SET;
		tx_set = TX_UNSET;
		rx_unset = RX_SET;
		tx_unset = TX_SET;
	}

	mipi_ops.mipi_set_cmd_mode();
	mipi_ops.mipi_eotp_set(rx_set, tx_set);

	for (i = 0; i < codes->cmd_total; i++) {
		mipi_ops.mipi_force_write(
			codes->cmds[i].header.data_type,
			codes->cmds[i].payload,
			codes->cmds[i].header.len);

		if (codes->cmds[i].header.wait)
			msleep(codes->cmds[i].header.wait);
	}

	mipi_ops.mipi_force_read(0x0A, 1, (uint8_t *)read_data);
	mipi_ops.mipi_eotp_set(rx_unset, tx_unset);

	if (read_data[0] == (self->esd & 0xff))
		return 0;
	pr_err("lcd-panel: esd check error, code <0x%x>\n", read_data[0]);
	return -ERR;

err:
	pr_err("lcd-panel: esd check error!");
	return -ENODATA;
}

static int32_t sprd_panel_enter_sleep(struct panel_spec *self,
					uint8_t is_sleep)
{
	struct panel_cmds *off_cmds;
	int i = 0;

	struct ops_mipi mipi_ops = {
		.mipi_set_cmd_mode	=
			self->info.mipi->ops->mipi_set_cmd_mode,
		.mipi_eotp_set		=
			self->info.mipi->ops->mipi_eotp_set,
		.mipi_dcs_write		=
			self->info.mipi->ops->mipi_dcs_write,
	};

	mipi_ops.mipi_set_cmd_mode();
	mipi_ops.mipi_eotp_set(0, 0);

	if (is_sleep)
		off_cmds = self->sleep_in_cmds;
	else
		off_cmds = self->sleep_out_cmds;

	for (i = 0; i < off_cmds->cmd_total; i++) {
		mipi_ops.mipi_dcs_write(off_cmds->cmds[i].payload,
			off_cmds->cmds[i].header.len);
		if (off_cmds->cmds[i].header.wait)
			msleep(off_cmds->cmds[i].header.wait);
	}

	mipi_ops.mipi_eotp_set(0, 0);
	pr_info("lcd-panel: sleep %d\n", is_sleep);

	return 0;
}

/* get panel mipi timing parameters from dt */
static int32_t sprd_panel_get_mipi_timing(struct device_node *panel_node,
						struct panel_cfg *panel_cfg)
{
	unsigned int temp;
	int ret;

	pr_info("lcd-panel: %s processing\n", __func__);
	/* mipi timing */
	ret = of_property_read_u32(panel_node, "hfront-porch", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->timing->hfp = temp;

	ret = of_property_read_u32(panel_node, "hback-porch", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->timing->hbp = temp;

	ret = of_property_read_u32(panel_node, "hsync-len", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->timing->hsync = temp;

	ret = of_property_read_u32(panel_node, "vback-porch", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->timing->vbp = temp;

	ret = of_property_read_u32(panel_node, "vfront-porch", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->timing->vfp = temp;

	ret = of_property_read_u32(panel_node, "vsync-len", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->timing->vsync = temp;

	return 0;
err:
	pr_err("lcd-panel: sprdfb_get_panel_mipi_timing err\n");
	return ret;
}

/* get panel mipi info parameters from dt */
static int32_t sprd_panel_get_mipi_info(struct device_node *panel_node,
						struct panel_cfg *panel_cfg)
{
	unsigned int temp;
	int ret;

	pr_info("lcd-panel: sprdfb_get_panel_dt_data: Start\n");

	/* mipi info */
	ret = of_property_read_u32(panel_node, "hsync-active", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->h_sync_pol = temp;

	ret = of_property_read_u32(panel_node, "vsync-active", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->v_sync_pol = temp;

	ret = of_property_read_u32(panel_node, "te-active", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->te_pol = temp;

	ret = of_property_read_u32(panel_node, "de-active", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->de_pol = temp;

	ret = of_property_read_u32(panel_node, "work-mode", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->work_mode = temp;

	ret = of_property_read_u32(panel_node, "bpp", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->video_bus_width = temp;

	ret = of_property_read_u32(panel_node, "lane-number", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->lan_number = temp;

	ret = of_property_read_u32(panel_node, "dphy-freq", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->phy_feq = temp;

	ret = of_property_read_u32(panel_node, "color-mode-pol", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->color_mode_pol = temp;

	ret = of_property_read_u32(panel_node, "shut-down-pol", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->info.mipi->shut_down_pol = temp;
	panel_cfg->panel->info.mipi->ops = NULL;

	return 0;
err:
	pr_err("lcd-panel: sprdfb_get_panel_dt_misc err ");
	return ret;
}

/* Parse panel commands */
static int sprd_parse_panel_cmds(struct device_node *np,
				struct panel_cmds **in_pcmds, char *name)
{
	const char *base;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_cmd_header *header;
	struct panel_cmds *pcmds;
	int i, total = 0;

	*in_pcmds = kzalloc(sizeof(struct panel_cmds), GFP_KERNEL);
	pcmds = *in_pcmds;

	if (IS_ERR_OR_NULL(pcmds)) {
		pr_err("pcmds alloc memory fail\n");
		return -ENOMEM;
	}

	base = of_get_property(np, name, &blen);

	if (!base) {
		pr_err("failed, name=%s\n", name);
		return -ENOMEM;
	}

	buf = kcalloc(blen, sizeof(char), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, base, blen);

	for (i = 0; i < blen; i += 4)
		pr_debug("sprd panel cmds %2x %2x %2x %2x\n",
			buf[i], buf[i + 1], buf[i + 2], buf[i + 3]);
	/* scan dcs commands */
	bp = buf;
	len = blen;
	pr_debug("sprd is panel cmds len %2x\n", len);
	total = 0;

	while (len >= sizeof(struct dsi_cmd_header)) {
		header = (struct dsi_cmd_header *)bp;
		header->len = ntohs(header->len);

		if (header->len > len) {
			pr_err("%s: dsi total =%2x wait=%x error, len=%x\n",
				__func__, total, header->wait, header->len);
			goto error;
		}

		bp	+= sizeof(*header);
		len	-= sizeof(*header);
		bp	+= header->len;
		len	-= header->len;
		total++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!", __func__, buf[0], blen);
		goto error;
	}

	pcmds->cmds = kcalloc(total, sizeof(struct dsi_cmd_desc), GFP_KERNEL);
	if (!pcmds->cmds)
		goto error;

	pcmds->cmd_total = total;
	pcmds->cmd_base = buf;
	pcmds->byte_len = blen;
	bp = buf;
	len = blen;

	for (i = 0; i < total; i++) {
		header = (struct dsi_cmd_header *)bp;
		len	-= sizeof(*header);
		bp	+= sizeof(*header);

		pcmds->cmds[i].header	= *header;
		pcmds->cmds[i].payload	= bp;
		bp	+= header->len;
		len	-= header->len;
	}

	return 0;

error:
	kfree(buf);
	return -ENOMEM;
}
static int32_t sprd_parse_panel_pair_cmds(struct device_node *np,
		struct panel_pair_cmds **p_cmds, char *cmd_name)
{
	int ret, len = 0;
	char *buf;
	const char *base;
	struct panel_pair_cmds *pcmds = NULL;

	pcmds = kzalloc(sizeof(*pcmds), GFP_KERNEL);

	if (IS_ERR(pcmds)) {
		pr_err("lcd-panel: pair cmd memory allocation failed\n");
		return -ENOMEM;
	}

	*p_cmds = pcmds;
	base = of_get_property(np, cmd_name, &len);

	if (!base) {
		pr_err("lcd-panel: failed, name=%s\n", cmd_name);
		ret = -ENODEV;
		goto error;
	}

	buf = kcalloc(len, sizeof(char), GFP_KERNEL);

	if (!buf) {
		ret = -ENOMEM;
		goto error;
	}

	memcpy(buf, base, len);
	pcmds->num_cmd = len / 2;
	pcmds->pair_cmd = (struct pair_cmds *)buf;

	return 0;

error:
	kfree(pcmds);
	return ret;
}

static int32_t sprd_panel_get_cmds_params(struct device_node *panel_node,
		struct panel_cfg *panel_cfg)
{
	struct panel_cmds *pcmds = NULL;
	struct panel_pair_cmds *ppcmds = NULL;
	int ret;

	ret = sprd_parse_panel_cmds(panel_node, &pcmds, "init-data");
	if (ret)
		goto err;
	panel_cfg->panel->init_cmds = pcmds;

	ret = sprd_parse_panel_cmds(panel_node, &pcmds, "esd-check");
	if (ret)
		goto err;
	panel_cfg->panel->esd_cmds = pcmds;

	ret = sprd_parse_panel_cmds(panel_node, &pcmds, "readid-wrcmd");
	if (ret)
		goto err;
	panel_cfg->panel->readid_cmds = pcmds;

	ret = sprd_parse_panel_cmds(panel_node, &pcmds, "sleep-in");
	if (ret)
		goto err;
	panel_cfg->panel->sleep_in_cmds = pcmds;

	ret = sprd_parse_panel_cmds(panel_node, &pcmds, "sleep-out");
	if (ret)
		goto err;
	panel_cfg->panel->sleep_out_cmds = pcmds;

	ret = sprd_parse_panel_pair_cmds(panel_node, &ppcmds, "readid-rdcmd");
	if (ret)
		goto err;
	panel_cfg->panel->readid_rcmds = ppcmds;

	ret = sprd_parse_panel_pair_cmds(panel_node, &ppcmds, "readid-cmp");
	if (ret)
		goto err;
	panel_cfg->panel->readid_cmp = ppcmds;

	return 0;
err:
	pr_err(" lcd-panel: %s panel cmd parsing failed!\n ", __func__);
	return ret;
}

static int32_t sprd_panel_get_mipi_misc(struct device_node *panel_node,
		struct panel_cfg *panel_cfg)
{
	unsigned int temp;
	int i, ret, panel_cnt=0;
	struct sprd_panel_dt_param *panel_dt_param;

	ret = of_property_read_u32(panel_node, "reg", &temp);
	if (ret)
		goto err;
	panel_cfg->lcd_id = temp;

	ret = of_property_read_string(panel_node, "panel-name",
		&panel_cfg->lcd_name);
	if (ret)
		goto err;

	ret = of_property_read_u32(panel_node, "hactive", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->width = temp;

	ret = of_property_read_u32(panel_node, "vactive", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->height = temp;

	ret = of_property_read_u32(panel_node, "fps", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->fps = temp;

	ret = of_property_read_u32(panel_node, "direction", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->direction = temp;
	panel_cfg->panel->type = LCD_MODE_DSI;

	ret = of_property_read_u32(panel_node, "esd", &temp);
	if (ret)
		goto err;
	panel_cfg->panel->esd = temp;
	
	panel_cnt = sizeof(sprd_panel_dt_ids)/sizeof(struct of_device_id);
	for (i = 0; i < panel_cnt; i++) {
		if (!strcmp(sprd_panel_dt_ids[i].name, panel_cfg->lcd_name)) {
			panel_dt_param = (struct sprd_panel_dt_param *)
					(sprd_panel_dt_ids[i].data);
			panel_cfg->panel->mipi_write_type =
					panel_dt_param->eotp_type;
			return 0;
		}
	}
	return 0;
err:
	pr_err("lcd-panel: %s DT MISC info parsing failed", __func__);
	return ret;
}

static int32_t sprd_panel_get_panel_dt(struct device_node *panel_node,
		struct panel_cfg *panel_cfg)
{
	int ret;

	ret = sprd_panel_get_mipi_timing(panel_node, panel_cfg);
	if (ret)
		goto err;

	ret = sprd_panel_get_mipi_info(panel_node, panel_cfg);
	if (ret)
		goto err;

	ret = sprd_panel_get_mipi_misc(panel_node, panel_cfg);
	if (ret)
		goto err;

	ret = sprd_panel_get_cmds_params(panel_node, panel_cfg);
	if (ret)
		goto err;

	return 0;

err:
	pr_err("lcd-panel: %s DT parsing failed\n", __func__);
	return ret;
};

static int32_t sprd_panel_mem_free(int panel_cnt)
{
	int i = 0;
	struct panel_spec *panel;

	for (i = 0; i < panel_cnt; i++) {
		panel = pctx.panels_cfg[i].panel;
		kfree(panel->info.mipi->timing);
		kfree(panel->info.mipi);
		kfree(panel);
	}

	kfree(pctx.panels_cfg);
	pctx.panels_cfg = NULL;
	pctx.panel_cnt = 0;

	return 0;
}

static int32_t sprd_panel_mem_alloc(int panel_cnt)
{
	int cnt = 0, ret = 0;
	struct panel_spec *panel = NULL;
	struct info_mipi *mipi;
	struct timing_rgb *timing;
	struct panel_cfg *panels_cfg;

	pr_info("lcd-panel: %s processing", __func__);

	panels_cfg = kcalloc(panel_cnt, sizeof(*(panels_cfg)), GFP_KERNEL);
	if (!panels_cfg) {
		ret = -ENOMEM;
		goto mem_err;
	}

	pctx.panels_cfg = panels_cfg;

	for (cnt = 0; cnt < panel_cnt; cnt++) {
		panel = kzalloc(sizeof(*panel), GFP_KERNEL);
		if (!panel) {
			ret = -ENOMEM;
			goto free_cfg;
		}

		mipi = kzalloc(sizeof(*(mipi)), GFP_KERNEL);
		if (!mipi) {
			ret = -ENOMEM;
			goto free_panel;
		}

		timing = kzalloc(sizeof(*(timing)), GFP_KERNEL);
		if (!timing) {
			ret = -ENOMEM;
			goto free_mipi;
		}

		panel->info.mipi			= mipi;
		panel->info.mipi->timing	= timing;
		panel->ops	= &lcd_panel_mipi_ops;
		pctx.panels_cfg[cnt].panel	= panel;
	}
	pr_info("lcd-panel: %s success!", __func__);
	return 0;

free_mipi:
	kfree(mipi);

free_panel:
	kfree(panel);

free_cfg:
	sprd_panel_mem_free(cnt);

mem_err:
	pr_err("lcd-panel: %s memory allocation failed", __func__);
	return ret;
}

static int32_t sprd_get_dt_param(struct device_node *panel_node)
{
	struct device_node *node;
	const __be32 *list, *list_end;
	phandle curr_phandle;
	int ret, cnt = 0, size = 0;

	pr_info("lcd-panel: %s", __func__);

	/* read phandle of lcd panel dt node */
	list = of_get_property(panel_node, "sprd,lcd-panels", &size);
	if (!list) {
		pr_err("lcd-panel: %s No lcd-panel-driver reference in dtsi\n"
			, __func__);
		return -ENODEV;
	}

	list_end = list + size / sizeof(*list);
	pctx.panel_cnt = size / sizeof(*list);

	pr_info("lcd-panel: number of lcd panel = %d\n", pctx.panel_cnt);

	/* get dev node from phandle */
	if (pctx.panel_cnt > 0) {
		if (!sprd_panel_mem_alloc(pctx.panel_cnt)) {
			cnt = 0;

			while (list < list_end) {
				curr_phandle = be32_to_cpup(list++);
				node = of_find_node_by_phandle(curr_phandle);
				if (!node) {
						pr_err(" no node for phandle %d\n",
						curr_phandle);
					goto err_dt;
				}

				ret = sprd_panel_get_panel_dt(node,
					&pctx.panels_cfg[cnt]);
				if (ret)
					goto err_dt;
				cnt++;
			}

			pr_info("lcd-panel: %s panel dt parsing successful",
				__func__);
			return 0;
		}
		goto err_mem;
	} else {
		pr_err("lcd-panel: %s No lcd panel in dt\n", __func__);
		return -ERR;
	}

err_mem:
	pr_err("lcd-panel: %s memory failed.", __func__);
	return -ENOMEM;

err_dt:
	pr_err("lcd-panel: %s DT Parsing failed.", __func__);
	sprd_panel_mem_free(pctx.panel_cnt);
	return -ERR;
}

static int sprd_panel_get_param(void)
{
	int err;
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "sprd,lcd-mipi-panel");
	if (!node)
		return -ENODEV;
	err = sprd_get_dt_param(node);
	if (err)
		return err;

	return 0;
}

static int __init sprd_mipi_panel(void)
{
	int i, err;

	pr_info("lcd-panel: %s processing\n", __func__);

	err = sprd_panel_get_param();
	if (err) {
		pr_err("lcd-panel: %s dt parsing failed!\n", __func__);
		return err;
	}
	for (i = 0; i < pctx.panel_cnt; i++)
		sprd_panel_register(&pctx.panels_cfg[i]);

	return 0;
}

subsys_initcall(sprd_mipi_panel);
