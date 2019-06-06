/*
 * Driver header file for pin controller driver
 *
 * Copyright (C) 2015 Spreadtrum
 * Baolin Wang <baolin.wang@spreadtrum.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PINCTRL_SPRD_H__
#define __PINCTRL_SPRD_H__

#if defined(CONFIG_SOC_WHALE2)
#include "sc9850-pins.h"
#elif defined(CONFIG_SC9830)
#include "sc9830-pins.h"
#elif defined(CONFIG_SOC_IWHALE2)
#include "sc9861-pins.h"
#elif defined(CONFIG_SOC_ISHARKL2)
#include "sc9853i-pins.h"
#elif defined(CONFIG_SC9833) || defined(CONFIG_SOC_SHARKLJ1) || defined(CONFIG_SOC_PIKE2)
#include "sc9833-pins.h"
#elif defined(CONFIG_SOC_SHARKLE)
#include "sc9832e-pins.h"
#elif defined(CONFIG_SOC_SHARKL3)
#include "sc9863-pins.h"
#elif defined(CONFIG_SOC_ROC1)
#include "sc9863-pins.h"
#endif

#endif /* __PINCTRL_SPRD_H__ */
