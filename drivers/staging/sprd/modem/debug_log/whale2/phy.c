/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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

#include <linux/io.h>
#include <linux/init.h>
#include <linux/delay.h>

#include "core.h"
#include "phy.h"

/**
 * Reset D-PHY module
 * @param phy: pointer to structure
 *  which holds information about the d-phy module
 * @param reset
 */
static void dsi_phy1_rstz(unsigned long base, int level)
{
}

/**
 * Power up/down D-PHY module
 * @param phy: pointer to structure
 *  which holds information about the d-phy module
 * @param enable (1: shutdown)
 */
static void dsi_phy1_shutdownz(unsigned long base, int level)
{
}

/**
 * Set number of active lanes
 * @param phy: pointer to structure
 *  which holds information about the d-phy module
 * @param no_of_lanes
 */
static void dsi_phy1_datalane_en(unsigned long base, int no)
{
}

/**
 * Enable clock lane module
 * @param phy pointer to structure
 *  which holds information about the d-phy module
 * @param en
 */
static void dsi_phy1_clklane_en(unsigned long base, int en)
{
}

/**
 * Get D-PHY PPI status
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param mask
 * @return status
 */
static u8 dsi_phy1_is_pll_locked(unsigned long base)
{
	return 1;
}

/**
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param value
 */
static void dsi_phy1_test_clk(unsigned long base, u8 value)
{
}

/**
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param value
 */
static void dsi_phy1_test_clr(unsigned long base, u8 value)
{
}

/**
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param on_falling_edge
 */
static void dsi_phy1_test_en(unsigned long base, u8 value)
{
}

/**
 * @param phy pointer to structure which holds information about the d-phy
 * module
 */
#if 0
static u8 dsi_phy1_test_dout(unsigned long base)
{
	return 0;
}
#endif

/**
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param test_data
 */
static void dsi_phy1_test_din(unsigned long base, u8 data)
{
}

/**
 * Write to D-PHY module (encapsulating the digital interface)
 * @param base pointer to structure which holds information about the d-base
 * module
 * @param address offset inside the D-PHY digital interface
 * @param data array of bytes to be written to D-PHY
 * @param data_length of the data array
 */
static void dsi_phy1_test_write(unsigned long base, u8 address, u8 data)
{
	dsi_phy1_test_en(base, 1);

	dsi_phy1_test_din(base, address);

	dsi_phy1_test_clk(base, 1);
	dsi_phy1_test_clk(base, 0);

	dsi_phy1_test_en(base, 0);

	dsi_phy1_test_din(base, data);

	dsi_phy1_test_clk(base, 1);
	dsi_phy1_test_clk(base, 0);
}

#if 0
static u8 dsi_phy1_test_read(unsigned long base, u8 address)
{
	dsi_phy1_test_en(base, 1);

	dsi_phy1_test_din(base, address);

	dsi_phy1_test_clk(base, 1);
	dsi_phy1_test_clk(base, 0);

	dsi_phy1_test_en(base, 0);

	udelay(1);

	return dsi_phy1_test_dout(base);
}
#endif

static int dsi_phy1_wait_pll_locked(unsigned long base)
{
	unsigned i = 0;

	for (i = 0; i < 50000; i++) {
		if (dsi_phy1_is_pll_locked(base))
			return 0;
		udelay(3);
	}

	pr_err("error: base pll can not be locked\n");
	return -1;
}

static void dsi_phy1_pll_config(unsigned long base, u32 freq)
{
	switch (freq) {
	case 1500000:
		pr_err("not implemented\n");
		break;
	case 1000000:
		pr_err("not implemented\n");
		break;
	default:
		pr_err("the target freq is not supported\n");
		return;
	}

	/* the following code is just for compiling success*/
	dsi_phy1_test_write(base, 0, 0);

	pr_info("set phy freq %u OK\n", freq);
}


/**
 * Configure D-PHY and PLL module to desired operation mode
 * @param base: pointer to structure
 *  which holds information about the d-base module
 * @param no_of_lanes active
 * @param freq desired high speed frequency
 * @return error code
 */
int dsi_phy1_init(struct phy_ctx *phy)
{
	unsigned long base = phy->base;

	dsi_phy1_rstz(base, 0);
	dsi_phy1_shutdownz(base, 0);
	dsi_phy1_clklane_en(base, 0);
	dsi_phy1_test_clr(base, 1);
	dsi_phy1_test_clr(base, 0);

	dsi_phy1_pll_config(base, phy->freq);

	dsi_phy1_datalane_en(base, phy->lanes);
	dsi_phy1_clklane_en(base, 1);
	dsi_phy1_shutdownz(base, 1);
	dsi_phy1_rstz(base, 1);

	if (dsi_phy1_wait_pll_locked(base))
		return -1;

	return 0;
}

/**
 * Close and power down D-PHY module
 * @param base pointer to structure which holds information about the d-base
 * module
 * @return error code
 */
int dsi_phy1_exit(struct phy_ctx *phy)
{
	unsigned long base = phy->base;

	dsi_phy1_rstz(base, 0);
	dsi_phy1_shutdownz(base, 0);
	dsi_phy1_rstz(base, 1);

	return 0;
}
