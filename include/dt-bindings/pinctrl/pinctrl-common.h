/*
 * Driver header file for pin controller DT macro define
 *
 * Copyright (C) 2015 Spreadtrum
 * Baolin Wang <baolin.wang@spreadtrum.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PINCTRL_COMMON_H__
#define __PINCTRL_COMMON_H__

#define NUM_OFFSET	(20)
#define MODE_OFFSET	(16)
#define BIT_OFFSET	(8)
#define WIDTH_OFFSET	(4)

#define SPRD_PIN_INFO(num, mode, offset, width, reg)	\
		(((num) & 0xFFF) << NUM_OFFSET |	\
		 ((mode) & 0xF) << MODE_OFFSET |	\
		 ((offset) & 0xFF) << BIT_OFFSET |	\
		 ((width) & 0xF) << WIDTH_OFFSET |	\
		 ((reg) & 0xF))

#define SPRD_PINCTRL_PIN(pin)		SPRD_PINCTRL_PIN_DATA(pin, #pin)
#define SPRD_PINCTRL_PIN_DATA(a, b)			\
	{						\
		.name = b,				\
		.num = (((a) >> NUM_OFFSET) & 0xfff),		\
		.mode = (((a) >> MODE_OFFSET) & 0xf),		\
		.bit_offset = (((a) >> BIT_OFFSET) & 0xff),	\
		.bit_width = ((a) >> WIDTH_OFFSET & 0xf),	\
		.reg = ((a) & 0xf)				\
	}
#endif
