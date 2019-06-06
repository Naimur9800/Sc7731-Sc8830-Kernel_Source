/*
 * @file mipi_dsih_dphy.h
 *
 *  Synopsys Inc.
 *  SG DWC PT02
 */

#ifndef MIPI_DSIH_DPHY_H_
#define MIPI_DSIH_DPHY_H_

#include "dsi_1_21a/mipi_dsih_local.h"

#define REG_DBG_APB_TEST_CTRL				(0x00)
#define REG_DBG_APB_TEST_DIN				(0x04)
#define REG_DBG_APB_TEST_DOUT				(0x08)
#define REG_DBG_APB_PHY_CTRL0				(0x0C)
#define REG_DBG_APB_PHY_CTRL5				(0x20)
#define REG_DBG_APB_PHY_CTRL_RO_1		(0x28)

/* REG_DBG_APB_TEST_CTRL */
#define OFFEST_DBG_APB_TESTEN				(0)
#define OFFEST_DBG_APB_TESTCLR			(1)
#define OFFEST_DBG_APB_TESTCLK			(2)

/* REG_DBG_APB_TEST_DIN */
#define OFFEST_DBG_APB_DBG_TESTIN			(0)

/* REG_DBG_APB_TEST_DOUT */
#define OFFEST_DBG_APB_DBG_TESTOUT		(0)


/* REG_DBG_APB_PHY_CTRL0 */
#define OFFEST_DBG_APB_TXULPSEXITCLK		(0)
#define OFFEST_DBG_APB_TXULPSCLK			(1)
#define OFFEST_DBG_APB_TXCLKESC			(2)
#define OFFEST_DBG_APB_RSTZ				(3)
#define OFFEST_DBG_APB_SHUTDOWN			(4)


/* REG_DBG_APB_PHY_CTRL5 */
#define OFFEST_DBG_APB_BISTON				(0)
#define OFFEST_DBG_APB_FORCEPLL			(1)
#define OFFEST_DBG_APB_ENABLECLK			(2)
#define OFFEST_DBG_APB_ENABLE_0			(3)
#define OFFEST_DBG_APB_ENABLE_1			(4)
#define OFFEST_DBG_APB_ENABLE_2			(5)
#define OFFEST_DBG_APB_ENABLE_3			(6)



/* obligatory functions - code can be changed for different phys*/
dsih_error_t mipi_log_dphy_open(dphy_t *phy);
dsih_error_t mipi_log_dphy_configure(dphy_t *phy,
				uint8_t no_of_lanes, uint32_t output_freq);
dsih_error_t mipi_log_dphy_close(dphy_t *phy);
u32 mipi_log_dphy_is_locked(dphy_t *phy);

#endif
