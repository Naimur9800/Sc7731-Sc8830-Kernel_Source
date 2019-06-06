/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 */

#ifndef _SERDES_APB_REG_H
#define _SERDES_APB_REG_H

#define REG_SERDES_APB_FUNC_EN					0x0000
#define REG_SERDES_APB_CH_EN					0x0004
#define REG_SERDES_APB_FUNNEL_EN				0x0008
#define REG_SERDES_APB_FSM_CUT_OFF_LEN				0x0010
#define REG_SERDES_APB_CH3_LA_SAMPLE_RATE			0x008C

/* REG_SERDES_APB_FUNC_EN */

#define BIT_SERDES_APB_FUNC_EN_FUNC_EN				BIT(0)

/* REG_SERDES_APB_CH_EN */

#define BIT_SERDES_APB_CH_EN_CH_EN(x)				(((x) & 0xFFFF))

/* REG_SERDES_APB_FUNNEL_EN */

#define BIT_SERDES_APB_FUNNEL_EN				BIT(0)

/* REG_SERDES_APB_FSM_CUT_OFF_LEN */

#define BIT_SERDES_APB_FSM_CUT_OFF_LEN_FSM_CUT_OFF(x)		(((x) & 0xFFFF))

/* REG_SERDES_APB_CH3_LA_SAMPLE_RATE */

#define BIT_SERDES_APB_CH3_LA_SAMPLE_RATE_CH3_LA_SAMPLE_RATE(x)	(((x) & 0xF))

#endif
