/* Copyright (C) 2014 Intel Mobile Communications GmbH
 * *
 * * This software is licensed under the terms of the GNU General Public
 * * License version 2, as published by the Free Software Foundation, and
 * * may be copied, distributed, and modified under those terms.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * * GNU General Public License for more details.
 * */

#ifndef _MV_CPU_H
#define _MV_CPU_H

bool mv_is_cpu_shared(unsigned);

bool mv_is_cpu_exclusive(unsigned);

bool mv_is_irq_bypass(void);

bool mv_is_guest_vector(unsigned vector);

unsigned mv_cpu_get_apicid(unsigned);

bool mv_cpu_get_settings(void);

unsigned int mv_get_shared_cpu(void);

#endif
