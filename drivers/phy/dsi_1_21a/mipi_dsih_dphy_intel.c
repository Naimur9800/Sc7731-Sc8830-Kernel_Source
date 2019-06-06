/**
 * @file mipi_dsih_dphy_intel.c
 * @brief D-PHY driver
 *
 *  Synopsys Inc.
 *  SG DWC PT02
 */
#include "mipi_dsih_dphy.h"
#include <linux/io.h>
#define PRECISION_FACTOR		(1000)
/* Reference clock frequency divided by Input Frequency Division Ratio LIMITS */
#define DPHY_DIV_UPPER_LIMIT	(40000)
#ifdef GEN_2
#define DPHY_DIV_LOWER_LIMIT	(5000)
#else
#define DPHY_DIV_LOWER_LIMIT	(1000)
#endif

#if ((defined DWC_MIPI_DPHY_BIDIR_TSMC40LP) || (defined GEN_2))
#define MIN_OUTPUT_FREQ			(80)
#elif defined DPHY2Btql
#define MIN_OUTPUT_FREQ			(200)
#undef GEN_2
#endif

/**
 * Initialise D-PHY module and power up
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @return error code
 */
dsih_error_t mipi_dsih_dphy_open(dphy_t *phy)
{
	if (phy == 0)
		return ERR_DSI_PHY_INVALID;
	else if ((phy->core_read_function == 0)
			|| (phy->core_write_function == 0))
		return ERR_DSI_INVALID_IO;
	else if (phy->status == INITIALIZED)
		return ERR_DSI_PHY_INVALID;
	phy->status = NOT_INITIALIZED;
	phy->status = INITIALIZED;
	return OK;
}
/**
 * Configure D-PHY and PLL module to desired operation mode
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @param no_of_lanes active
 * @param output_freq desired high speed frequency
 * @return error code
 */
void pll_bits_or(dphy_t *phy, uint32_t value, unsigned long addr)
{
	writel_relaxed(readl_relaxed((void __iomem *)(phy->address_pllapb+addr))
			| value, (void __iomem *)(phy->address_pllapb+addr));
}

uint32_t pll_readl(dphy_t *phy, unsigned long addr)
{
	uint32_t value;

	value = readl_relaxed((void __iomem *)(phy->address_pllapb+addr));
	return value;
}

void pll_writel(dphy_t *phy, uint32_t value, unsigned long addr)
{
	writel_relaxed(value, (void __iomem *)(phy->address_pllapb+addr));
}

void pll_writel_nbits(dphy_t *phy, uint32_t value, uint8_t nbits,
				uint8_t offset, unsigned long addr)
{
	uint32_t tmp;

	tmp = readl_relaxed((void __iomem *)(phy->address_pllapb+addr));
	tmp &= ~(((1 << nbits)-1) << offset);
	tmp |= value << offset;
	pll_writel(phy, tmp, addr);
}

void phy_bits_or(dphy_t *phy, uint32_t value, unsigned long addr)
{
	writel_relaxed(readl_relaxed((void __iomem *)(phy->address_phyapb+addr))
			| value, (void __iomem *)(phy->address_phyapb+addr));
}

uint32_t phy_readl(dphy_t *phy, unsigned long addr)
{
	uint32_t value;

	value = readl_relaxed((void __iomem *)(phy->address_phyapb+addr));
	return value;
}

void phy_writel(dphy_t *phy, uint32_t value, unsigned long addr)
{
	writel_relaxed(value, (void __iomem *)(phy->address_phyapb+addr));
}

void writel_nbits(dphy_t *phy, uint32_t value, uint8_t nbits,
				uint8_t offset, unsigned long addr)
{
	uint32_t tmp;

	tmp = readl_relaxed((void __iomem *)(phy->address_phyapb+addr));
	tmp &= ~(((1 << nbits)-1) << offset);
	tmp |= value << offset;
	phy_writel(phy, tmp, addr);
}

void phy_bios_programming(dphy_t *phy)
{
	writel_nbits(phy, 0x60, 8, 0, 0x300);
	writel_nbits(phy, 0x7a, 8, 0, 0x308);
	writel_nbits(phy, 0xa1, 8, 8, 0x308);
	writel_nbits(phy, 0x17, 6, 16, 0x308);
	writel_nbits(phy, 0x3, 4, 0, 0x30c);
	writel_nbits(phy, 0x0, 8, 0, 0x310);
	writel_nbits(phy, 0x0, 2, 8, 0x310);
	writel_nbits(phy, 0x0, 8, 16, 0x310);
	writel_nbits(phy, 0x0, 2, 24, 0x310);
	writel_nbits(phy, 0x0, 3, 0, 0x314);
	writel_nbits(phy, 0x0, 1, 16, 0x314);
	writel_nbits(phy, 0x4, 4, 0, 0x318);
	writel_nbits(phy, 0x8, 5, 8, 0x318);
	writel_nbits(phy, 0x1, 3, 16, 0x318);
	writel_nbits(phy, 0x22, 8, 0, 0x320);
	writel_nbits(phy, 0x0, 2, 8, 0x320);
	writel_nbits(phy, 0x0, 1, 0, 0x324);
	writel_nbits(phy, 0x2, 3, 16, 0x324);
	writel_nbits(phy, 0x1, 1, 16, 0x30c);

	writel_nbits(phy, 0x1, 1, 5, 0x04);
	writel_nbits(phy, 0x0, 1, 4, 0x04);
	writel_nbits(phy, 0x4, 3, 1, 0x04);
	writel_nbits(phy, 0xf, 8, 0, 0xe0);
	writel_nbits(phy, 0xd, 8, 0, 0xe4);
	writel_nbits(phy, 0x22, 8, 0, 0xe8);
	writel_nbits(phy, 0x13, 8, 0, 0xec);
	writel_nbits(phy, 0x13, 8, 0, 0xfc);
	writel_nbits(phy, 0x20, 8, 0, 0xf0);
	writel_nbits(phy, 0x20, 8, 0, 0x100);
	writel_nbits(phy, 0xf, 8, 0, 0x104);
	writel_nbits(phy, 0x53, 8, 0, 0x10c);
	writel_nbits(phy, 0xb, 8, 0, 0x108);
	writel_nbits(phy, 0x1a, 8, 0, 0x114);
	writel_nbits(phy, 0x186, 12, 0, 0x110);
	writel_nbits(phy, 0x9c, 8, 12, 0x110);
	writel_nbits(phy, 0x7, 5, 11, 0x18);
	writel_nbits(phy, 0x4, 3, 28, 0x58);
}
dsih_error_t mipi_dsih_dphy_configure_hop(dphy_t *phy, int freq_a, int delta,
		uint32_t mipi_clk_hop_en)
{
	u32 synthfdiv_hopmaxcode[2] = {0, 0};
	u32 synthfdiv_stepsize[2] = {0, 0};
	u8 synthfdiv_sscupdatecycle = 0;
	u8 synthfdiv_locwren = 0;
	u8 synthfdiv_hop_dir = 0;
	u8 synthfdiv_hop_req = 0;

	if (!mipi_clk_hop_en) {
		delta = ~(delta - 1);
		if (delta * 1000 < (freq_a * 12 / 1012)) {
			pr_emerg("dphy hop down 1.03 percent\n");
			synthfdiv_hopmaxcode[0] = 0;
			synthfdiv_hopmaxcode[1] = 0x3f4a00;
			synthfdiv_stepsize[0] = 0x25;
			synthfdiv_stepsize[1] = 0x7e;
			synthfdiv_sscupdatecycle = 5;
		} else {
			pr_emerg("dphy hop down 2 percent\n");
			synthfdiv_hopmaxcode[0] = 2;
			synthfdiv_hopmaxcode[1] = 0x7ae147;
			synthfdiv_stepsize[0] = 0x32;
			synthfdiv_stepsize[1] = 0x3c9;
			synthfdiv_sscupdatecycle = 5;
		}
		pll_writel_nbits(phy, synthfdiv_hopmaxcode[0], 2, 30, 0x2a0);
		pll_writel_nbits(phy, synthfdiv_hopmaxcode[1], 25, 0, 0x2a4);
		pll_writel_nbits(phy, synthfdiv_stepsize[0], 6, 26, 0x2a4);
		pll_writel_nbits(phy, synthfdiv_stepsize[1], 14, 0, 0x2a8);
		pll_writel_nbits(phy, synthfdiv_sscupdatecycle, 8, 24, 0x248);

		synthfdiv_locwren = 1;
		synthfdiv_hop_dir = 2;
		synthfdiv_hop_req = 1;
		pll_writel_nbits(phy, synthfdiv_locwren, 1, 14, 0x2a8);
		pll_writel_nbits(phy, synthfdiv_hop_dir, 2, 26, 0x2a0);
		pll_writel_nbits(phy, synthfdiv_hop_req, 1, 29, 0x2a0);
		synthfdiv_hop_req = 0;
		pll_writel_nbits(phy, synthfdiv_hop_req, 1, 29, 0x2a0);
	} else {
		pr_emerg("dphy hop up\n");
		synthfdiv_locwren = 1;
		synthfdiv_hop_dir = 1;
		synthfdiv_hop_req = 1;
		pll_writel_nbits(phy, synthfdiv_locwren, 1, 14, 0x2a8);
		pll_writel_nbits(phy, synthfdiv_hop_dir, 2, 26, 0x2a0);
		pll_writel_nbits(phy, synthfdiv_hop_req, 1, 29, 0x2a0);
		synthfdiv_hop_req = 0;
		pll_writel_nbits(phy, synthfdiv_hop_req, 1, 29, 0x2a0);
	}
	return OK;
}
dsih_error_t mipi_dsih_dphy_configure_hop_freq(dphy_t *phy,
		int freq_a, int delta)
{
	static int delta_sum;
	int hop_dir;

	if (!phy)
		return ERR_DSI_INVALID_INSTANCE;
	if (phy->status < INITIALIZED)
		return ERR_DSI_INVALID_INSTANCE;
	if (freq_a < MIN_OUTPUT_FREQ)
		return ERR_DSI_PHY_FREQ_OUT_OF_BOUND;
	if (((delta_sum < 0) && (delta < 0))  ||
			((delta_sum > 0) && (delta > 0)))
		return ERR_DSI_INVALID_EVENT;
	if (delta > 0)
		hop_dir = 1;
	else
		hop_dir = 0;

	mipi_dsih_dphy_configure_hop(phy, freq_a, delta, hop_dir);
	delta_sum += delta;
	return 0;
}

dsih_error_t mipi_dsih_dphy_ssc(dphy_t *phy, int freq, int mipi_ssc)
{
	return -EINVAL;
}

dsih_error_t mipi_dsih_dphy_configure(dphy_t *phy,
			uint8_t no_of_lanes, uint32_t output_freq)
{
	uint32_t loop_divider = 0; /* (M) */
	uint32_t input_divider = 1; /* (N) */
	uint8_t n = 0;/* iterator */
	int flag = 0;
	uint32_t delta = 0;
	uint32_t tmp_loop_divider = 0;
	unsigned step = 0;

	if (phy == 0)
		return ERR_DSI_INVALID_INSTANCE;

	if (phy->status < INITIALIZED)
		return ERR_DSI_INVALID_INSTANCE;

	if (output_freq < MIN_OUTPUT_FREQ)
		return ERR_DSI_PHY_FREQ_OUT_OF_BOUND;

	loop_divider = ((output_freq * (phy->reference_freq /
			DPHY_DIV_LOWER_LIMIT)) / phy->reference_freq);
	/* here delta will account for the rounding */
	delta = ((loop_divider * phy->reference_freq) /
		(phy->reference_freq / DPHY_DIV_LOWER_LIMIT)) - output_freq;
	for (input_divider = 1 + (phy->reference_freq / DPHY_DIV_UPPER_LIMIT);
		((phy->reference_freq / input_divider) >=
			DPHY_DIV_LOWER_LIMIT) && (!flag); input_divider++) {
		tmp_loop_divider = ((output_freq * input_divider) /
			(phy->reference_freq));
		if ((tmp_loop_divider % 2) == 0) {
			/* if even */
			if (output_freq ==
				(tmp_loop_divider * (phy->reference_freq
							/ input_divider))) {
				/* exact values found */
				flag = 1;
				loop_divider = tmp_loop_divider;
				delta = output_freq - (tmp_loop_divider *
					(phy->reference_freq / input_divider));
				break;
			} else if ((output_freq - (tmp_loop_divider *
			  (phy->reference_freq / input_divider))) < delta) {
				/* values found with smaller delta */
				loop_divider = tmp_loop_divider;
				delta = output_freq - (tmp_loop_divider
				  * (phy->reference_freq / input_divider));
				step = 1;
			}
		} else {
			tmp_loop_divider += 1;
			if (output_freq == (tmp_loop_divider *
				(phy->reference_freq / input_divider))) {
				/* exact values found */
				flag = 1;
				loop_divider = tmp_loop_divider;
				delta = (tmp_loop_divider * (phy->reference_freq
					/ input_divider)) - output_freq;
				break;
			} else if (((tmp_loop_divider * (phy->reference_freq
				   / input_divider)) - output_freq) < delta) {
				/* values found with smaller delta */
				loop_divider = tmp_loop_divider;
				delta = (tmp_loop_divider * (phy->reference_freq
					/ input_divider)) - output_freq;
				step = 0;
			}
		}
	}
	if (!flag) {
		input_divider = step + (loop_divider * phy->reference_freq)
								/ output_freq;
		pr_err("D-PHY: Approximated Frequency: %d KHz",
			(loop_divider * (phy->reference_freq / input_divider)));
	}
#ifdef CONFIG_FB_DYNAMIC_FREQ_SCALING
	if (phy->phy_keep_work != true)
#endif
	{
		/* get power-down mode,reset to avoid transient periods*/
		/*in PHY operation during re-configuration procedures. */
		mipi_dsih_dphy_reset(phy, 0);
		mipi_dsih_dphy_clock_en(phy, 0);
		mipi_dsih_dphy_shutdown(phy, 0);
		for (n = 0; n < 100; n++)
			;
	}

	/* set up board depending on environment if any */
	if (phy->bsp_pre_config != 0)
		phy->bsp_pre_config(phy, 0);

	/***********intel dphy enable dsi**************/
	phy_writel(phy, 0x0, (unsigned long)0x14);
	phy_writel(phy, 0x0, (unsigned long)0x0);
	phy_writel(phy, 0xf, (unsigned long)0x18);
	phy_writel(phy, 0x0, (unsigned long)0x58);
	phy_writel(phy, 0x3810, (unsigned long)0x44);
	phy_bits_or(phy, 1<<4, (unsigned long)0x120);
	set_pll_frequency(phy, output_freq);
	phy_bits_or(phy, 1<<0, (unsigned long)0x120);
	mipi_dsih_dphy_force_pll(phy, 1);
	/************set hs prepare time*************/
	phy_writel(phy, 0x6, (unsigned long)0xe4);
	phy_writel(phy, 0x1c, (unsigned long)0xe8);
	phy_writel(phy, 0x9, (unsigned long)0xec);

	mipi_dsih_dphy_no_of_lanes(phy, no_of_lanes);
#ifdef CONFIG_FB_DYNAMIC_FREQ_SCALING
	if (phy->phy_keep_work != true)
#endif
	{
		mipi_dsih_dphy_stop_wait_time(phy, 0x1C);
		mipi_dsih_dphy_clock_en(phy, 1);
		for (n = 0; n < 100; n++)
			;
		mipi_dsih_dphy_shutdown(phy, 1);
		for (n = 0; n < 100; n++)
			;
		mipi_dsih_dphy_reset(phy, 1);
	}
	return OK;
}
/**
 * Close and power down D-PHY module
 * @param phy pointer to structure which holds information about the d-phy
 * module
 * @return error code
 */
dsih_error_t mipi_dsih_dphy_close(dphy_t *phy)
{
	if (phy == 0)
		return ERR_DSI_INVALID_INSTANCE;
	else if ((phy->core_read_function == 0)
			|| (phy->core_write_function == 0))
		return ERR_DSI_INVALID_IO;

	if (phy->status < NOT_INITIALIZED)
		return ERR_DSI_INVALID_INSTANCE;

	mipi_dsih_dphy_reset(phy, 0);
	mipi_dsih_dphy_reset(phy, 1);
	mipi_dsih_dphy_shutdown(phy, 0);
	phy->status = NOT_INITIALIZED;
	return OK;
}
/**
 * Enable clock lane module
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param en
 */
void mipi_dsih_dphy_clock_en(dphy_t *instance, int en)
{
	mipi_dsih_dphy_write_part(instance, R_DPHY_RSTZ,
					en, 2, 1);
}
/**
 * Reset D-PHY module
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param reset
 */
void mipi_dsih_dphy_reset(dphy_t *instance, int reset)
{
	mipi_dsih_dphy_write_part(instance, R_DPHY_RSTZ,
					reset, 1, 1);
}
/**
 * Power up/down D-PHY module
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param powerup (1) shutdown (0)
 */
void mipi_dsih_dphy_shutdown(dphy_t *instance, int powerup)
{
	mipi_dsih_dphy_write_part(instance, R_DPHY_RSTZ,
					powerup, 0, 1);
}
/**
 * Force D-PHY PLL to stay on while in ULPS
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param force (1) disable (0)
 * @note To follow the programming model, use wakeup_pll function
 */
void mipi_dsih_dphy_force_pll(dphy_t *instance, int force)
{
	mipi_dsih_dphy_write_part(instance, R_DPHY_RSTZ,
					force, 3, 1);
}
/**
 * Get force D-PHY PLL module
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @return force value
 */
int mipi_dsih_dphy_get_force_pll(dphy_t *instance)
{
	return mipi_dsih_dphy_read_part(instance, R_DPHY_RSTZ, 3, 1);
}
/**
 * Wake up or make sure D-PHY PLL module is awake
 * This function must be called after going into ULPS and before exiting it
 * to force the DPHY PLLs to wake up. It will wait until the DPHY status is
 * locked. It follows the procedure described in the user guide.
 * This function should be used to make sure the PLL is awake, rather than
 * the force_pll above.
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @return error code
 * @note this function has an active wait
 */
int mipi_dsih_dphy_wakeup_pll(dphy_t *instance)
{
	unsigned i = 0;

	if (mipi_dsih_dphy_status(instance, 0x1) == 0) {
		mipi_dsih_dphy_force_pll(instance, 1);
		for (i = 0; i < DSIH_PHY_ACTIVE_WAIT; i++) {
			if (mipi_dsih_dphy_status(instance, 0x1))
				break;
		}
		if (mipi_dsih_dphy_status(instance, 0x1) == 0)
			return ERR_DSI_PHY_PLL_NOT_LOCKED;
	}
	return OK;
}
/**
 * Configure minimum wait period for HS transmission request after a stop state
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param no_of_byte_cycles [in byte (lane) clock cycles]
 */
void mipi_dsih_dphy_stop_wait_time(dphy_t *instance, uint8_t no_of_byte_cycles)
{
	mipi_dsih_dphy_write_part(instance, R_DPHY_IF_CFG,
					no_of_byte_cycles, 8, 8);
}
/**
 * Set number of active lanes
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param no_of_lanes
 */
void mipi_dsih_dphy_no_of_lanes(dphy_t *instance, uint8_t no_of_lanes)
{
	mipi_dsih_dphy_write_part(instance, R_DPHY_IF_CFG,
					no_of_lanes - 1, 0, 2);
}
/**
 * Get number of currently active lanes
 * @param instance pointer to structure which holds information about the d-phy
 *  module
 * @return number of active lanes
 */
uint8_t mipi_dsih_dphy_get_no_of_lanes(dphy_t *instance)
{
	return mipi_dsih_dphy_read_part(instance, R_DPHY_IF_CFG, 0, 2);
}

/**
 * SPRD ADD
 * Set non-continuous clock mode
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param enable
 */
void mipi_dsih_dphy_enable_nc_clk(dphy_t *instance, int enable)
{
	mipi_dsih_dphy_write_part(instance, R_DPHY_LPCLK_CTRL,
					enable, 1, 1);
}

/**
 * Request the PHY module to start transmission of high speed clock.
 * This causes the clock lane to start transmitting DDR clock on the
 * lane interconnect.
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param enable
 * @note this function should be called explicitly by user always except for
 * transmitting
 */
void mipi_dsih_dphy_enable_hs_clk(dphy_t *instance, int enable)
{
	mipi_dsih_dphy_write_part(instance, R_DPHY_LPCLK_CTRL,
					enable, 0, 1);
}
/**
 * One bit is asserted in the trigger_request (4bits) to cause the lane module
 * to cause the associated trigger to be sent across the lane interconnect.
 * The trigger request is synchronous with the rising edge of the clock.
 * @note: Only one bit of the trigger_request is asserted at any given time, the
 * remaining must be left set to 0, and only when not in LPDT or ULPS modes
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param trigger_request 4 bit request
 */
dsih_error_t mipi_dsih_dphy_escape_mode_trigger(dphy_t *instance,
				uint8_t trigger_request)
{
	uint8_t sum = 0;
	int i = 0;

	for (i = 0; i < 4; i++)
		sum += ((trigger_request >> i) & 1);

	if (sum == 1) {
		/* clear old trigger */
		mipi_dsih_dphy_write_part(instance, R_DPHY_TX_TRIGGERS,
						0x00, 0, 4);
		mipi_dsih_dphy_write_part(instance, R_DPHY_TX_TRIGGERS,
						trigger_request, 0, 4);
		for (i = 0; i < DSIH_PHY_ACTIVE_WAIT; i++) {
			if (mipi_dsih_dphy_status(instance, 0x0010))
				break;
		}
		mipi_dsih_dphy_write_part(instance, R_DPHY_TX_TRIGGERS,
						0x00, 0, 4);
		if (i >= DSIH_PHY_ACTIVE_WAIT)
			return ERR_DSI_TIMEOUT;
		return OK;
	}
	return ERR_DSI_INVALID_COMMAND;
}
/**
 * ULPS mode request/exit on all active data lanes.
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param enable (request 1/ exit 0)
 * @return error code
 * @note this is a blocking function. wait upon exiting the ULPS will exceed 1ms
 */
#ifdef GEN_2
dsih_error_t mipi_dsih_dphy_ulps_data_lanes(dphy_t *instance, int enable)
{
	int timeout;
	/* mask 1 0101 0010 0000 */
	uint16_t data_lanes_mask = 0;

	if (enable) {
		mipi_dsih_dphy_write_part(instance, R_DPHY_ULPS_CTRL,
						1, 2, 1);
	} else {
		if (mipi_dsih_dphy_status(instance, 0x1) == 0)
			return ERR_DSI_PHY_PLL_NOT_LOCKED;
		mipi_dsih_dphy_write_part(instance, R_DPHY_ULPS_CTRL,
						1, 3, 1);
		switch (mipi_dsih_dphy_get_no_of_lanes(instance)) {
		/* Fall through */
		case 3:
			data_lanes_mask |= (1 << 12);
		/* Fall through */
		case 2:
			data_lanes_mask |= (1 << 10);
		/* Fall through */
		case 1:
			data_lanes_mask |= (1 << 8);
		/* Fall through */
		case 0:
			data_lanes_mask |= (1 << 5);
			break;
		default:
			data_lanes_mask = 0;
			break;
		}
		for (timeout = 0; timeout < DSIH_PHY_ACTIVE_WAIT; timeout++) {
			/* verify that the DPHY has left ULPM */
			if (mipi_dsih_dphy_status(instance, data_lanes_mask)
							== data_lanes_mask)
				break;
			/* wait at least 1ms */
			for (timeout = 0; timeout < ONE_MS_ACTIVE_WAIT;
								timeout++)
				;
		}
		if (mipi_dsih_dphy_status(instance, data_lanes_mask)
							!= data_lanes_mask) {
			instance->log_info(" stat %x, mask %x",
				mipi_dsih_dphy_status(instance,
					data_lanes_mask), data_lanes_mask);
			return ERR_DSI_TIMEOUT;
		}
		mipi_dsih_dphy_write_part(instance, R_DPHY_ULPS_CTRL,
						0, 2, 1);
		mipi_dsih_dphy_write_part(instance, R_DPHY_ULPS_CTRL,
						0, 3, 1);
	}
	return OK;
}
#else
void mipi_dsih_dphy_ulps_data_lanes(dphy_t *instance, int enable)
{
	int timeout;

	if (enable) {
		mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL,
						1, 3, 1);
	} else {
		mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL,
						1, 4, 1);
		for (timeout = 0; timeout < DSIH_PHY_ACTIVE_WAIT; timeout++) {
			/* verify that the DPHY has left ULPM */
			/* mask 1010100100000 */
			if (mipi_dsih_dphy_status(instance, 0x1520) == 0) {
				/* wait at least 1ms */
				for (timeout = 0; timeout < ONE_MS_ACTIVE_WAIT;
						timeout++)
					;
				break;
			}
		}
		mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL,
						0, 3, 1);
		mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL,
						0, 4, 1);
	}
}
#endif
/**
 * ULPS mode request/exit on Clock Lane.
 * @param instance pointer to structure which holds information about the
 * d-phy module
 * @param enable 1 or disable 0 of the Ultra Low Power State of the clock lane
 * @return error code
 * @note this is a blocking function. wait upon exiting the ULPS will exceed 1ms
 */
#ifdef GEN_2
dsih_error_t mipi_dsih_dphy_ulps_clk_lane(dphy_t *instance, int enable)
{
	int timeout;
	/* mask 1000 */
	uint16_t clk_lane_mask = 0x0008;

	if (enable) {
		mipi_dsih_dphy_write_part(instance, R_DPHY_ULPS_CTRL,
						1, 0, 1);
	} else {
		if (mipi_dsih_dphy_status(instance, 0x1) == 0)
			return ERR_DSI_PHY_PLL_NOT_LOCKED;
		mipi_dsih_dphy_write_part(instance, R_DPHY_ULPS_CTRL,
						1, 1, 1);
		for (timeout = 0; timeout < DSIH_PHY_ACTIVE_WAIT; timeout++) {
			/* verify that the DPHY has left ULPM */
			/* mask 1010100100000 */
			if (mipi_dsih_dphy_status(instance, clk_lane_mask)
					== clk_lane_mask) {
				/* wait at least 1ms */
				instance->log_info(" stat %x, mask %x",
				  mipi_dsih_dphy_status
				   (instance, clk_lane_mask), clk_lane_mask);
				break;
			}
			/* wait at least 1ms */
			/* dummy operation for the loop not to be optimised */
			for (timeout = 0; timeout < ONE_MS_ACTIVE_WAIT;
								timeout++)
				enable = mipi_dsih_dphy_status(instance,
								clk_lane_mask);
		}
		if (mipi_dsih_dphy_status(instance, clk_lane_mask)
							!= clk_lane_mask)
			return ERR_DSI_TIMEOUT;
		mipi_dsih_dphy_write_part(instance, R_DPHY_ULPS_CTRL,
						0, 0, 1);
		mipi_dsih_dphy_write_part(instance, R_DPHY_ULPS_CTRL,
						0, 1, 1);
	}
	return OK;
}
#else
void mipi_dsih_dphy_ulps_clk_lane(dphy_t *instance, int enable)
{
	int timeout;

	if (enable) {
		mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL,
						0, 0, 1);
		mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL,
						1, 1, 1);
	} else {
		mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL,
						1, 2, 1);
		for (timeout = 0; timeout < DSIH_PHY_ACTIVE_WAIT;
								timeout++) {
			/* verify that the DPHY has left ULPM */
			/* mask 1010100100000 */
			if (mipi_dsih_dphy_status(instance, 0x0004) == 0) {
				/* wait at least 1ms */
				for (timeout = 0; timeout < ONE_MS_ACTIVE_WAIT;
					timeout++)
					;
				break;
			}
		}
		mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL,
						0, 1, 1);
		mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL,
						0, 2, 1);
	}
}
#endif
/**
 * Get D-PHY PPI status
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param mask
 * @return status
 */
uint32_t mipi_dsih_dphy_status(dphy_t *instance, uint16_t mask)
{
	return mipi_dsih_dphy_read_word(instance, R_DPHY_STATUS) & mask;
}
/**
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param value
 */
void mipi_dsih_dphy_test_clock(dphy_t *instance, int value)
{
	mipi_dsih_dphy_write_part(instance, R_DPHY_TST_CRTL0,
					value, 1, 1);
}
/**
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param value
 */
void mipi_dsih_dphy_test_clear(dphy_t *instance, int value)
{
	mipi_dsih_dphy_write_part(instance, R_DPHY_TST_CRTL0,
					value, 0, 1);
}
/**
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param on_falling_edge
 */
void mipi_dsih_dphy_test_en(dphy_t *instance, uint8_t on_falling_edge)
{
	mipi_dsih_dphy_write_part(instance, R_DPHY_TST_CRTL1,
					on_falling_edge, 16, 1);
}
/**
 * @param instance pointer to structure which holds information about the d-phy
 * module
 */
uint8_t mipi_dsih_dphy_test_data_out(dphy_t *instance)
{
	return mipi_dsih_dphy_read_part(instance, R_DPHY_TST_CRTL1, 8, 8);
}
/**
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param test_data
 */
void mipi_dsih_dphy_test_data_in(dphy_t *instance, uint8_t test_data)
{
	mipi_dsih_dphy_write_word(instance, R_DPHY_TST_CRTL1, test_data);
}

/* abstracting BSP */
/**
 * Write to whole register to D-PHY module (encapsulating the bus interface)
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param reg_address offset
 * @param data 32-bit word
 */
void mipi_dsih_dphy_write_word(dphy_t *instance,
		uint32_t reg_address, uint32_t data)
{
	if (instance->core_write_function != 0)
		instance->core_write_function
			(instance->address, reg_address, data);
}
/**
 * Write bit field to D-PHY module (encapsulating the bus interface)
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param reg_address offset
 * @param data bits to be written to D-PHY
 * @param shift from the right hand side of the register (big endian)
 * @param width of the bit field
 */
void mipi_dsih_dphy_write_part(dphy_t *instance,
	uint32_t reg_address, uint32_t data, uint8_t shift, uint8_t width)
{
	uint32_t mask = 0;
	uint32_t temp = 0;

	if (instance->core_read_function != 0) {
		mask = (1 << width) - 1;
		temp = mipi_dsih_dphy_read_word(instance, reg_address);
		temp &= ~(mask << shift);
		temp |= (data & mask) << shift;
		mipi_dsih_dphy_write_word(instance, reg_address, temp);
	}
}
/**
 * Read whole register from D-PHY module (encapsulating the bus interface)
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param reg_address offset
 * @return data 32-bit word
 */
uint32_t mipi_dsih_dphy_read_word(dphy_t *instance, uint32_t reg_address)
{
	if (instance->core_read_function == 0)
		return ERR_DSI_INVALID_IO;
	return instance->core_read_function(instance->address, reg_address);
}
/**
 * Read bit field from D-PHY module (encapsulating the bus interface)
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param reg_address offset
 * @param shift from the right hand side of the register (big endian)
 * @param width of the bit field
 * @return data bits to be written to D-PHY
 */
uint32_t mipi_dsih_dphy_read_part(dphy_t *instance,
		uint32_t reg_address, uint8_t shift, uint8_t width)
{
	return (mipi_dsih_dphy_read_word(instance, reg_address) >> shift)
						& ((1 << width) - 1);
}
