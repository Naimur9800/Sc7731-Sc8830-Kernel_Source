/*
 * Copyright (C) 2014 Spreadtrum Communications Inc.
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

#define pr_fmt(__fmt) "sprd-adf:<%d> [%20s] "__fmt,  __LINE__, __func__

#include <asm/div64.h>
#include <linux/string.h>
#include <linux/delay.h>
#include "mipi_log_dphy.h"

#define MIN_OUTPUT_FREQ		(100)

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char  u8;

enum TIMING {
	NONE,
	REQUEST_TIME,
	PREPARE_TIME,
	SETTLE_TIME,
	ZERO_TIME,
	TRAIL_TIME,
	EXIT_TIME,
	CLKPOST_TIME,
	TA_GET,
	TA_GO,
	TA_SURE,
	TA_WAIT,
};

enum PHY_FREQ {
	FREQ_1G = 1000000,
	FREQ_1G2 = 1200000,
	FREQ_1G3 = 1300000,
	FREQ_1G4 = 1400000,
	FREQ_1G5 = 1500000,
};

/**
 * Write to whole register to D-PHY module (encapsulating the bus interface)
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param reg_address offset
 * @param data 32-bit word
 */
static void mipi_log_dphy_write_word(dphy_t *phy,
				u32 reg_address, u32 data)
{
	if (phy->core_write_function)
		phy->core_write_function(phy->address,
				reg_address, data);
}

/**
 * Read whole register from D-PHY module (encapsulating the bus interface)
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param reg_address offset
 * @return data 32-bit word
 */
static u32 mipi_log_dphy_read_word(dphy_t *phy, u32 reg_address)
{
	if (!phy->core_read_function)
		return ERR_DSI_INVALID_IO;

	return phy->core_read_function(phy->address, reg_address);
}

/**
 * Write bit field to D-PHY module (encapsulating the bus interface)
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param reg_address offset
 * @param data bits to be written to D-PHY
 * @param shift from the right hand side of the register (big endian)
 * @param width of the bit field
 */
static void mipi_log_dphy_write_part(dphy_t *phy,
				u32 reg_address, u32 data,
				u8 shift, u8 width)
{
	u32 mask = 0;
	u32 temp = 0;

	if (phy->core_read_function) {
		mask = (1 << width) - 1;
		temp = mipi_log_dphy_read_word(phy, reg_address);
		temp &= ~(mask << shift);
		temp |= (data & mask) << shift;
		mipi_log_dphy_write_word(phy, reg_address, temp);
	}
}

/**
 * Enable clock lane module
 * @param phy pointer to structure
 *  which holds information about the d-phy module
 * @param en
 */
static void mipi_log_dphy_clock_en(dphy_t *phy, int en)
{
	mipi_log_dphy_write_part(phy, REG_DBG_APB_PHY_CTRL5, en,
			OFFEST_DBG_APB_ENABLECLK, 1);
}

/**
 * Reset D-PHY module
 * @param phy: pointer to structure
 *  which holds information about the d-phy module
 * @param reset
 */
static void mipi_log_dphy_reset(dphy_t *phy, int reset)
{
	mipi_log_dphy_write_part(phy, REG_DBG_APB_PHY_CTRL0, reset,
			OFFEST_DBG_APB_RSTZ, 1);
}

/**
 * Power up/down D-PHY module
 * @param phy: pointer to structure
 *  which holds information about the d-phy module
 * @param enable (1: shutdown)
 */
static void mipi_log_dphy_shutdown(dphy_t *phy, int enable)
{
	mipi_log_dphy_write_part(phy, REG_DBG_APB_PHY_CTRL0, enable,
			OFFEST_DBG_APB_SHUTDOWN, 1);
}

/**
 * Set number of active lanes
 * @param phy: pointer to structure
 *  which holds information about the d-phy module
 * @param no_of_lanes
 */
static void mipi_log_dphy_no_of_lanes(dphy_t *phy, u8 no_of_lanes)
{
	mipi_log_dphy_write_part(phy, REG_DBG_APB_PHY_CTRL5,
			0xF, 3, no_of_lanes);
}

/**
 * Get D-PHY PPI status
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param mask
 * @return status
 */
u32 mipi_log_dphy_is_locked(dphy_t *phy)
{
	return mipi_log_dphy_read_word(phy,
			REG_DBG_APB_PHY_CTRL_RO_1)
			&BIT(16);
}

/**
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param value
 */
static void mipi_log_dphy_test_clock(dphy_t *phy, int value)
{
	mipi_log_dphy_write_part(phy, REG_DBG_APB_TEST_CTRL,
			value, OFFEST_DBG_APB_TESTCLK, 1);
}

/**
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param value
 */
static void mipi_log_dphy_test_clear(dphy_t *phy, int value)
{
	mipi_log_dphy_write_part(phy, REG_DBG_APB_TEST_CTRL,
				value, OFFEST_DBG_APB_TESTCLR, 1);
}

/**
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param on_falling_edge
 */
static void mipi_log_dphy_test_en(dphy_t *phy, u8 on_falling_edge)
{
	mipi_log_dphy_write_part(phy, REG_DBG_APB_TEST_CTRL,
			on_falling_edge, OFFEST_DBG_APB_TESTEN, 1);
}

/**
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param test_data
 */
static void mipi_log_dphy_test_data_in(dphy_t *phy, u8 test_data)
{
	mipi_log_dphy_write_word(phy, REG_DBG_APB_TEST_DIN, test_data);
}

/**
 * Write to D-PHY module (encapsulating the digital interface)
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param address offset inside the D-PHY digital interface
 * @param data array of bytes to be written to D-PHY
 * @param data_length of the data array
 */
static void mipi_log_dphy_test_write(dphy_t *phy, u8 address,
				u8 *data, u8 data_length)
{
	int i = 0;

	if (data) {
		mipi_log_dphy_test_clock(phy, 0);
		/* set TESTEN input high  */
		mipi_log_dphy_test_en(phy, 1);
		/*
		 * set the TESTCLK input high in preparation
		 * to latch in the desired test mode
		 */
		mipi_log_dphy_test_clock(phy, 1);
		/**
		 * set the desired test code in the input 8-bit
		 *bus TESTDIN[7:0]
		 */
		mipi_log_dphy_test_data_in(phy, address);
		/*
		 * drive the TESTCLK input low;
		 * the falling edge captures the chosen test code
		 * into the transceiver
		 */
		mipi_log_dphy_test_clock(phy, 0);
		/*
		 * set TESTEN input low
		 * to disable further test mode code latching
		 */
		mipi_log_dphy_test_en(phy, 0);

		/*
		 * start writing MSB first
		 * set TESTDIN[7:0] to the desired test data appropriate
		 * to the chosen test mode
		 */
		for (i = data_length; i > 0; i--) {
			mipi_log_dphy_test_data_in(phy, data[i - 1]);
			/*
			 * pulse TESTCLK high to capture this test data
			 * into the macrocell; repeat these two steps
			 *as necessary
			 */
			mipi_log_dphy_test_clock(phy, 1);
			mipi_log_dphy_test_clock(phy, 0);
		}
	}
}

/**
 * Initialise D-PHY module and power up
 * @param phy: pointer to structure
 *  which holds information about the d-phy module
 * @return error code
 */
dsih_error_t mipi_log_dphy_open(dphy_t *phy)
{
	if (!phy || !phy->core_read_function || !phy->core_write_function)
		return ERR_DSI_PHY_INVALID;
	else if (phy->status == INITIALIZED)
		return ERR_DSI_PHY_INVALID;

	phy->status = INITIALIZED;
	return OK;
}

static int mipi_log_dphy_powerup(dphy_t *phy, int enable)
{
	/*
	 * Make sure that DSI IP is connected to TestChip IO
	 *
	 * get the PHY in power down mode (shutdownz = 0) and
	 * reset it (rstz = 0) to avoid transient periods in
	 * PHY operation during re-configuration procedures.
	 */
	if (enable) {
		mipi_log_dphy_clock_en(phy, 1);
		mipi_log_dphy_shutdown(phy, 1);
		mipi_log_dphy_reset(phy, 1);
	} else {
		mipi_log_dphy_reset(phy, 0);
		mipi_log_dphy_clock_en(phy, 0);
		mipi_log_dphy_shutdown(phy, 0);
		/**
		 *provide an initial active-high test clear pulse
		 * in TESTCLR, DPHY registers will be reset after test
		 *clear is set to high
		 */
		mipi_log_dphy_test_clear(phy, 1);
		mipi_log_dphy_test_clear(phy, 0);
	}
	return 0;
}

/**
 * Configure D-PHY and PLL module to desired operation mode
 * @param phy: pointer to structure
 *  which holds information about the d-phy module
 * @param no_of_lanes active
 * @param output_freq desired high speed frequency
 * @return error code
 */
dsih_error_t mipi_log_dphy_configure(dphy_t *phy,
				u8 lane_num,
				u32 output_freq)
{
	u8 data;

	if (!phy)
		return ERR_DSI_INVALID_INSTANCE;
	if (phy->status < INITIALIZED)
		return ERR_DSI_INVALID_INSTANCE;
	if (output_freq < MIN_OUTPUT_FREQ)
		return ERR_DSI_PHY_FREQ_OUT_OF_BOUND;

	mipi_log_dphy_powerup(phy, false);

	switch (output_freq) {
	case FREQ_1G5:
		data = 0x39;
		mipi_log_dphy_test_write(phy, 0x06, &data, 1);
		data = 0x5f;
		mipi_log_dphy_test_write(phy, 0x08, &data, 1);
		data = 0xb1;
		mipi_log_dphy_test_write(phy, 0x09, &data, 1);
		data = 0x3b;
		mipi_log_dphy_test_write(phy, 0x0a, &data, 1);
		data = 0x11;
		mipi_log_dphy_test_write(phy, 0x0b, &data, 1);
		break;
	case FREQ_1G4:
		data = 0x35;
		mipi_log_dphy_test_write(phy, 0x06, &data, 1);
		data = 0xaf;
		mipi_log_dphy_test_write(phy, 0x08, &data, 1);
		data = 0xd8;
		mipi_log_dphy_test_write(phy, 0x09, &data, 1);
		data = 0x9d;
		mipi_log_dphy_test_write(phy, 0x0a, &data, 1);
		data = 0x81;
		mipi_log_dphy_test_write(phy, 0x0b, &data, 1);
		break;
	case FREQ_1G3:
		data = 0x32;
		mipi_log_dphy_test_write(phy, 0x06, &data, 1);
		data = 0xff;
		mipi_log_dphy_test_write(phy, 0x08, &data, 1);
		data = 0x00;
		mipi_log_dphy_test_write(phy, 0x09, &data, 1);
		data = 0x00;
		mipi_log_dphy_test_write(phy, 0x0a, &data, 1);
		data = 0x01;
		mipi_log_dphy_test_write(phy, 0x0b, &data, 1);
		break;
	case FREQ_1G2:
		data = 0x2e;
		mipi_log_dphy_test_write(phy, 0x06, &data, 1);
		data = 0x4f;
		mipi_log_dphy_test_write(phy, 0x08, &data, 1);
		data = 0x27;
		mipi_log_dphy_test_write(phy, 0x09, &data, 1);
		data = 0x62;
		mipi_log_dphy_test_write(phy, 0x0a, &data, 1);
		data = 0x71;
		mipi_log_dphy_test_write(phy, 0x0b, &data, 1);
		break;
	default:
		break;
	}
	data = 0x20;
	mipi_log_dphy_test_write(phy, 0x32, &data, 1);
	mipi_log_dphy_test_write(phy, 0x42, &data, 1);
	mipi_log_dphy_test_write(phy, 0x52, &data, 1);
	mipi_log_dphy_test_write(phy, 0x62, &data, 1);
	mipi_log_dphy_test_write(phy, 0x72, &data, 1);
	mipi_log_dphy_test_write(phy, 0x33, &data, 1);
	mipi_log_dphy_test_write(phy, 0x43, &data, 1);
	mipi_log_dphy_test_write(phy, 0x53, &data, 1);
	mipi_log_dphy_test_write(phy, 0x63, &data, 1);
	mipi_log_dphy_test_write(phy, 0x73, &data, 1);

	mipi_log_dphy_no_of_lanes(phy, lane_num);
	mipi_log_dphy_powerup(phy, true);

	return 0;
}

/**
 * Close and power down D-PHY module
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @return error code
 */
dsih_error_t mipi_log_dphy_close(dphy_t *phy)
{
	if (!phy)
		return ERR_DSI_INVALID_INSTANCE;

	if (!phy->core_read_function || !phy->core_write_function)
		return ERR_DSI_INVALID_IO;

	if (phy->status < INITIALIZED)
		return ERR_DSI_INVALID_INSTANCE;

	mipi_log_dphy_reset(phy, 0);
	mipi_log_dphy_reset(phy, 1);
	mipi_log_dphy_shutdown(phy, 0);

	phy->status = NOT_INITIALIZED;

	return OK;
}

