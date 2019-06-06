/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 */
#ifndef __SPRD_GLB__
#define __SPRD_GLB__

#include <linux/bitops.h>

#if defined(CONFIG_SC9838)
#include "./sprd/sc9838/sc9838_glb.h"
#elif defined(CONFIG_SC9830)
#include "./sprd/sc9830/sc9830_glb.h"
#elif defined(CONFIG_SC9833)
#include "./sprd/sc9833/sc9833_glb.h"
#elif defined(CONFIG_SOC_SHARKLJ1)
#include "./sprd/sharklj1/glb.h"
#elif defined(CONFIG_SOC_SHARKLE)
#include "./sprd/sharkle/glb.h"
#elif defined(CONFIG_SOC_IWHALE2)
#include "./sprd/iwhale2/iwhale2_glb.h"
#elif defined(CONFIG_SOC_ISHARKL2)
#include "./sprd/isharkl2/isharkl2_glb.h"
#elif defined(CONFIG_SOC_WHALE2)
#include "./sprd/whale2/whale2_glb.h"
#elif defined(CONFIG_SOC_PIKE2)
#include "./sprd/pike2/glb.h"
#elif defined(CONFIG_SOC_SHARKL3)
#include "./sprd/sharkl3/glb.h"
#elif defined(CONFIG_SOC_ROC1)
#include "./sprd/roc1/glb.h"
#elif defined(CONFIG_SOC_SHARKL5)
#include "./sprd/sharkl5/glb.h"
#endif

#endif
