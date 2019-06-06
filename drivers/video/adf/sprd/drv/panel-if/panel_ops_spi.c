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
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <soc/sprd/sci.h>
#include "sprd_panel.h"
#include "../sprd_spi_intf.h"


static void spi_write_data(struct sprd_spi *spi, struct spi_transfer *xfer)
{
	struct spi_device *spi_dev = spi->spi_dev;
	struct spi_message msg;
	struct panel_info *panel = spi->panel;

	if (panel->spi_cd_gpio)
		gpio_direction_output(panel->spi_cd_gpio, 1);
	spi_message_init(&msg);
	spi_message_add_tail(xfer, &msg);
	spi_sync(spi_dev, &msg);

}

static void spi_write_cmd(struct sprd_spi *spi, struct spi_transfer *xfer)
{
	struct spi_device *spi_dev = spi->spi_dev;
	struct spi_message msg;
	struct panel_info *panel = spi->panel;

	if (panel->spi_cd_gpio)
		gpio_direction_output(panel->spi_cd_gpio, 0);
	spi_message_init(&msg);
	spi_message_add_tail(xfer, &msg);
	spi_sync(spi_dev, &msg);
}

static void spi_send_cmds(struct sprd_spi *spi, int8_t *data, uint16_t len)
{
	struct spi_transfer xfer;

	memset(&xfer, 0, sizeof(xfer));
	xfer.len = 1;
	xfer.tx_buf = &(data[0]);
	xfer.bits_per_word = 8;
	spi_write_cmd(spi, &xfer);
	if (len == 1)
		return;
	xfer.len = len - 1;
	xfer.tx_buf = &(data[1]);
	xfer.bits_per_word = 8;
	spi_write_data(spi, &xfer);
}

static int spi_panel_send_cmds(struct sprd_spi *spi, struct panel_cmds *in_cmds)
{
	int i;

	if ((in_cmds == NULL) || (spi == NULL))
		return -1;
	for (i = 0; i < in_cmds->cmd_total; i++) {
		spi_send_cmds(spi, in_cmds->cmds[i].payload,
				      in_cmds->cmds[i].header.len);
		if (in_cmds->cmds[i].header.wait)
			msleep(in_cmds->cmds[i].header.wait);
	}
	return 0;
}

static int32_t spi_panel_init(struct panel_device *pd)
{

	struct sprd_spi *spi = dev_get_drvdata(pd->intf);
	struct panel_info *panel = pd->panel;

	spi_panel_send_cmds(spi, panel->cmd_codes[CMD_CODE_INIT]);
	return 0;
}

static int32_t spi_panel_sleep_in(struct panel_device *pd)
{
	struct sprd_spi *spi = dev_get_drvdata(pd->intf);
	struct panel_info *panel = pd->panel;

	spi_panel_send_cmds(spi, panel->cmd_codes[CMD_CODE_SLEEP_IN]);
	return 0;
}

static int32_t spi_panel_esd_check(struct panel_device *pd)
{

	return 0;
}

static int32_t spi_disable_te(struct panel_device *pd)
{
	struct sprd_spi *spi = dev_get_drvdata(pd->intf);
	struct panel_info *panel = pd->panel;

	spi_panel_send_cmds(spi, panel->cmd_codes[CMD_SPI_DISABLE_TE]);
	return 0;
}

static int32_t spi_enable_te(struct panel_device *pd)
{
	struct sprd_spi *spi = dev_get_drvdata(pd->intf);
	struct panel_info *panel = pd->panel;

	spi_panel_send_cmds(spi, panel->cmd_codes[CMD_SPI_ENABLE_TE]);
	return 0;
}

struct panel_ops panel_ops_spi = {
	.init = spi_panel_init,
	.sleep_in = spi_panel_sleep_in,
	.esd_check = spi_panel_esd_check,
	.disable_te = spi_disable_te,
	.enable_te = spi_enable_te,
};


