/*
 * Copyright (C) 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MV_VPIC__
#define __MV_VPIC__

extern void __init mv_vpic_fixup_affinity(void);
extern struct irq_domain *mv_vpic_get_domain(void);

#endif /* __MV_VPIC__ */

