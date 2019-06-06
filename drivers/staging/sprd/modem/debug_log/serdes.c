/*
 * Copyright (C) 2018 Spreadtrum Communications Inc.
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

#include "serdes.h"
#include "core.h"
#ifdef pr_fmt
#undef pr_fmt
#define pr_fmt(fmt) "[serdes] "fmt
#endif

#define serdes_reg(s, reg, o) ((void * __iomem)((char *)serdes->base + o + reg))
#define s_reg(s, reg) ((unsigned long)serdes_reg(s, reg, 0))

#define ENABLE_REG (0x0000)
#define CHAN_SEL_REG (0x0004)
#define FUNNEL_REG (0x0008)
#define FUNNEL_BIT (BIT(0))
#define OVERFLOW_REG (0x000C)
#define CUT_OFF_REG (0x0010)

#define LA_SAMPLE_REG (0x008C)
#define USE_PORT_REG (0x0090)

#define DC_BLNC_REG (0x0200)
#define VERSION_REG (0x0400)

static inline void reg_set(struct serdes_drv_data *serdes, u32 reg, u32 bits)
{
	writel(bits, serdes_reg(serdes, reg, 0x1000));
}

static inline void reg_clr(struct serdes_drv_data *serdes, u32 reg, u32 bits)
{
	writel(bits, serdes_reg(serdes, reg, 0x2000));
}

static inline void reg_enable(struct serdes_drv_data *serdes, u32 reg, u32 bits,
			      int enable)
{
	writel(bits, serdes_reg(serdes, reg, (0x2000 >> enable)));
}

static int serdes_enable_i(struct serdes_drv_data *serdes, int enable)
{
	enable = !!enable;
	if (serdes->enabled == enable)
		return 0;
	reg_enable(serdes, ENABLE_REG, BIT(0), enable);
	serdes->enabled = enable;
	return 0;
}

static u32 serdes_funnel_enable(struct serdes_drv_data *serdes, int enable)
{
	u32 back_val;

	back_val = reg_read(s_reg(serdes, FUNNEL_REG));
	reg_enable(serdes, FUNNEL_REG, FUNNEL_BIT, enable);
	return !!(back_val & FUNNEL_BIT);
}

static int serdes_chan_sel(struct serdes_drv_data *serdes, int ch)
{
	u16 use_port = 0xFFFF;
	u16 sel;
	u32 funnel_enable;

	if (serdes->version >= 0x402)
		use_port = reg_read(s_reg(serdes, USE_PORT_REG));

	sel = (1 << ch);
	if (use_port & sel) {
		funnel_enable = serdes_funnel_enable(serdes, 0);
		reg_write(s_reg(serdes, CHAN_SEL_REG), sel);
		reg_enable(serdes, FUNNEL_REG, FUNNEL_BIT, funnel_enable);
	} else {
		pr_err("can't support select this port %d\n", ch);
		return -1;
	}
	return 0;
}

#if 0
static int serdes_ov_read(struct serdes_drv_data *serdes)
{
	u32 ov_val;

	ov_val = reg_read(s_reg(serdes, OVERFLOW_REG));
	reg_set(serdes, OVERFLOW_REG, BIT(4));
	serdes->ov_flow_last = ov_val & BIT(0);
	serdes->ov_flow |= serdes->ov_flow_last;
	return 0;
}
#endif

static int serdes_cut_off(struct serdes_drv_data *serdes, int cut_off)
{
	reg_write(s_reg(serdes, CUT_OFF_REG), cut_off);
	return 0;
}

#if 0
static int serdes_la_sample_rate(struct serdes_drv_data *serdes,
				 int la_sample_rate)
{
	reg_write(s_reg(serdes, LA_SAMPLE_REG), la_sample_rate);
	return 0;
}

static int serdes_get_use_info(struct serdes_drv_data *serdes, char *buf)
{
	int ret = 0;

	if (buf) {
		ret = snprintf(buf, PAGE_SIZE,
			       " port   sync  ready   la\n0x%04x 0x%04x 0x%04x 0x%04x\n",
			       reg_read(s_reg(serdes, USE_PORT_REG + 0x00)),
			       reg_read(s_reg(serdes, USE_PORT_REG + 0x04)),
			       reg_read(s_reg(serdes, USE_PORT_REG + 0x08)),
			       reg_read(s_reg(serdes, USE_PORT_REG + 0x0C)));
	}
	return ret;
}
#endif

static int serdes_dc_blnc_fix(struct serdes_drv_data *serdes, int fix)
{
	reg_write(s_reg(serdes, DC_BLNC_REG), !!fix);
	return 0;
}

static int serdes_get_version(struct serdes_drv_data *serdes)
{
	u32 version;

	version = reg_read(s_reg(serdes, VERSION_REG));
	if (version)
		serdes->version = version;

	return 0;
}

int serdes_enable(struct serdes_drv_data *serdes, int enable)
{
	static int is_enabled;

	if (is_enabled == enable) {
		if (enable)
			serdes_chan_sel(serdes, serdes->channel);
		return 0;
	}
	is_enabled = enable;

	if (enable) {
		serdes_get_version(serdes);
		serdes_cut_off(serdes, serdes->cut_off);
		serdes_dc_blnc_fix(serdes, serdes->dc_blnc_fix);
		serdes_funnel_enable(serdes, 1);
		serdes_chan_sel(serdes, serdes->channel);
		serdes_enable_i(serdes, 1);
	} else {
		serdes_enable_i(serdes, 0);
	}
	return 0;
}
