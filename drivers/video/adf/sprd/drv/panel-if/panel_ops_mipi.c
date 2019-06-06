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

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>

#include "sprd_panel.h"
#include "sprd_dphy.h"
#include "../dsi/mipi_dsi_api.h"

#include "../zte_lcd_common.h"
#ifdef CONFIG_ZTE_LCD_COMMON_FUNCTION
extern struct sprd_dispc *g_zte_ctrl_pdata;
#endif
int mipi_dsi_send_cmds(struct sprd_dsi *dsi, struct panel_cmds *in_cmds)
{
	int i;

	if ((in_cmds == NULL) || (dsi == NULL))
		return -1;

	for (i = 0; i < in_cmds->cmd_total; i++) {
		mipi_dsi_gen_write(dsi, in_cmds->cmds[i].payload,
				      in_cmds->cmds[i].header.len);
		if (in_cmds->cmds[i].header.wait)
			msleep(in_cmds->cmds[i].header.wait);
	}
	return 0;
}


static int32_t mipi_panel_init(struct panel_device *pd)
{
	struct sprd_dsi *dsi = dev_get_drvdata(pd->intf);
	struct device *dev = dev_get_next(&dsi->dev);
	struct sprd_dphy *dphy = dev_get_drvdata(dev);
	struct panel_info *panel = pd->panel;

	mipi_dsi_lp_cmd_enable(dsi, true);
	mipi_dsi_send_cmds(dsi, panel->cmd_codes[CMD_CODE_INIT]);
	mipi_dsi_set_work_mode(dsi, panel->work_mode);
	mipi_dsi_state_reset(dsi);

	/* the AA Chip needs to reset D-PHY before HS transmition */
	if (dphy->ctx.chip_id == 0)
		mipi_dphy_reset(dphy);

	mipi_dphy_hs_clk_en(dphy, true);

	ZTE_LCD_INFO("%s: on_cmds_cnt=%d\n", __func__, panel->cmd_codes[CMD_CODE_INIT]->cmd_total);
	#if defined(CONFIG_ZTE_LCD_COMMON_FUNCTION) && defined(CONFIG_ZTE_LCD_CABC3_EXTREME_POWER_SAVE)
	mutex_lock(&g_zte_ctrl_pdata->zte_lcd_ctrl->panel_sys_lock);
	g_zte_ctrl_pdata->zte_lcd_ctrl->zte_set_cabc_mode(g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_value);
	mutex_unlock(&g_zte_ctrl_pdata->zte_lcd_ctrl->panel_sys_lock);
	#endif
	return 0;
}

static int32_t mipi_panel_esd_check(struct panel_device *pd)
{
	struct sprd_dsi *dsi = dev_get_drvdata(pd->intf);
	struct panel_info *panel = pd->panel;
	uint8_t read_buf[4] = {0};
	uint8_t i;

	mipi_dsi_lp_cmd_enable(dsi, false);
	mipi_dsi_set_max_return_size(dsi, panel->esd_read_count);
	mipi_dsi_dcs_read(dsi, panel->esd_reg, read_buf,
				panel->esd_read_count);

	for (i = 0; i < panel->esd_read_count; i++) {
		if (read_buf[i] != panel->esd_return_code[i]) {
			pr_err("[SPRD_LCD]error code read_buf[%d] = <0x%x>\n",
					i, read_buf[i]);
			return -ENODATA;
		}
		pr_debug("read_buf[%d] = 0x%x\n", i, read_buf[i]);
	}

	return 0;
}

static uint32_t mipi_panel_read_id(struct panel_device *pd)
{
	struct sprd_dsi *dsi = dev_get_drvdata(pd->intf);
	struct panel_info *panel = pd->panel;
	uint8_t read_buf[4] = {0};
	int i;

	mipi_dsi_lp_cmd_enable(dsi, true);
	mipi_dsi_set_max_return_size(dsi, panel->id_size);
	mipi_dsi_dcs_read(dsi, panel->id_reg, read_buf, panel->id_size);

	for (i = 0; i < panel->id_size; i++) {
		if (read_buf[i] != panel->id_val[i]) {
			pr_err("read id failed!\n");
			return -ENODATA;
		}
		pr_info("read id[%d] = 0x%x\n", i, read_buf[i]);
	}

	return 0;
}

static int32_t mipi_panel_sleep_in(struct panel_device *pd)
{
	struct sprd_dsi *dsi = dev_get_drvdata(pd->intf);
	struct panel_info *panel = pd->panel;

	ZTE_LCD_INFO("%s: off_cmds_cnt=%d\n", __func__, panel->cmd_codes[CMD_CODE_SLEEP_IN]->cmd_total);
	mipi_dsi_set_work_mode(dsi, SPRD_MIPI_MODE_CMD);
	mipi_dsi_lp_cmd_enable(dsi, true);
	mipi_dsi_send_cmds(dsi, panel->cmd_codes[CMD_CODE_SLEEP_IN]);

	return 0;
}

static int32_t mipi_panel_send_cmd(struct panel_device *pd, int cmd_id)
{
	struct sprd_dsi *dsi = dev_get_drvdata(pd->intf);
	struct panel_info *panel = pd->panel;

	if (cmd_id >= CMD_CODE_MAX) {
		pr_err("cmd_id invalid\n");
		return -1;
	}

	mipi_dsi_lp_cmd_enable(dsi, true);
	mipi_dsi_send_cmds(dsi, panel->cmd_codes[cmd_id]);

	return 0;
}

static int32_t mipi_panel_set_brightness(struct panel_device *pd,
					int level)
{
	struct sprd_dsi *dsi = dev_get_drvdata(pd->intf);
	struct panel_info *panel = pd->panel;
	uint8_t *payload;
	uint8_t len;
	static u16 bldcs_level_last = 0xFFFF;
	#ifdef CONFIG_ZTE_LCD_DCSBL_CABC_GRADIENT
	static u8 lcd_cabc_53h = 0x24;
	#endif
	u32 bl_preset;

	if (pm_runtime_suspended(dsi->dev.parent)) {
		pr_err("dsi is not initialized\n");
		return -1;
	}

	bl_preset = level;
	#if defined(CONFIG_ZTE_LCD_COMMON_FUNCTION) && defined(CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE)
		if (g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_bl_register_len == 12) {
			level = g_zte_ctrl_pdata->zte_lcd_ctrl->zte_convert_brightness(level, 4095);
		} else {
			level = g_zte_ctrl_pdata->zte_lcd_ctrl->zte_convert_brightness(level, 255);
		}
	#endif

	#ifdef CONFIG_ZTE_LCD_DCSBL_CABC_GRADIENT
	if (g_zte_ctrl_pdata->zte_lcd_ctrl->close_dynamic_dimming) {
		ZTE_LCD_INFO("%s: close_dynamic_dimming\n", __func__);
	} else if (level != 0) {
		if (lcd_cabc_53h == 0x2c) {
			mipi_dsi_lp_cmd_enable(dsi, true);
			payload = panel->cmd_codes[CMD_OLED_BRIGHTNESS]->cmds[1].payload;
			len = panel->cmd_codes[CMD_OLED_BRIGHTNESS]->cmds[1].header.len;
			payload[1] = lcd_cabc_53h;
			mipi_dsi_gen_write(dsi, payload, len);
			ZTE_LCD_INFO("%s:lcd on 0x53 =0x%x\n", __func__, payload[1]);
			lcd_cabc_53h = 0X24;
		}
		if (bldcs_level_last == 0)
			lcd_cabc_53h = 0x2c;
	} else if (level == 0) {
		mipi_dsi_lp_cmd_enable(dsi, true);
		payload = panel->cmd_codes[CMD_OLED_BRIGHTNESS]->cmds[1].payload;
		len = panel->cmd_codes[CMD_OLED_BRIGHTNESS]->cmds[1].header.len;
		payload[1] = 0x24;
		lcd_cabc_53h = 0x24;
		mipi_dsi_gen_write(dsi, payload, len);
		ZTE_LCD_INFO("%s:lcd off 0x53 =0x%x\n", __func__, payload[1]);
	}
	#endif

	#ifdef CONFIG_ZTE_LCD_COMMON_FUNCTION
		ZTE_LCD_INFO("%s:level=%d --> convert_level = %d\n", __func__, bl_preset, level);
		#ifdef CONFIG_ZTE_LCD_DELAY_OPEN_BL
		if ((bldcs_level_last == 0) && (level != 0))
			msleep(32);
		#endif
		if (g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_bl_register_len == 12) {
			payload = panel->cmd_codes[CMD_OLED_BRIGHTNESS]->cmds[0].payload;
			len = panel->cmd_codes[CMD_OLED_BRIGHTNESS]->cmds[0].header.len;
			payload[1] = (uint8_t) (level >> 8) & 0x0f;
			payload[2] = (uint8_t) level & 0xff;
		} else {
			payload = panel->cmd_codes[CMD_OLED_BRIGHTNESS]->cmds[0].payload;
			len = panel->cmd_codes[CMD_OLED_BRIGHTNESS]->cmds[0].header.len;
			payload[1] = level;
		}

		mipi_dsi_lp_cmd_enable(dsi, false);
		mipi_dsi_gen_write(dsi, payload, len);

	#else
	if (panel->cmd_codes[CMD_OLED_BRIGHTNESS]->cmd_total == 1) {
		pr_info("mipi_panel_set_brightness level:%d\n", level);
		payload = panel->cmd_codes[CMD_OLED_BRIGHTNESS]->cmds[0].payload;
		len = panel->cmd_codes[CMD_OLED_BRIGHTNESS]->cmds[0].header.len;
		payload[1] = level;
	} else {
		payload = panel->cmd_codes[CMD_OLED_BRIGHTNESS]->cmds[level].payload;
		len = panel->cmd_codes[CMD_OLED_BRIGHTNESS]->cmds[level].header.len;
	}

	mipi_dsi_lp_cmd_enable(dsi, false);

	mipi_dsi_send_cmds(dsi, panel->cmd_codes[CMD_OLED_REG_LOCK]);
	mipi_dsi_gen_write(dsi, payload, len);

	mipi_dsi_send_cmds(dsi, panel->cmd_codes[CMD_OLED_REG_UNLOCK]);

	#endif
	bldcs_level_last = level;

	return 0;
}

struct panel_ops panel_ops_mipi = {
	.init = mipi_panel_init,
	.sleep_in = mipi_panel_sleep_in,
	.read_id = mipi_panel_read_id,
	.esd_check = mipi_panel_esd_check,
	.send_cmd = mipi_panel_send_cmd,
	.set_brightness = mipi_panel_set_brightness,
};
