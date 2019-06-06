/*
 * Copyright (C) 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MOBILEVISOR__
#define __MOBILEVISOR__

#define MV_APIC_NAME "mv"

extern void __init x86_mv_early_init(void);

#ifdef CONFIG_MOBILEVISOR
int is_x86_mobilevisor(void);
int x86_mv_map_vector(int vector);
#else
static inline int is_x86_mobilevisor(void)
{
	return 0;
}
static inline int x86_mv_map_vector(int vector)
{
	return vector;
}
#endif

#endif /* __MOBILEVISOR__ */

