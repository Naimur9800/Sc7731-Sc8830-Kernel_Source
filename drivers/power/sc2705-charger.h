/*
 * sc2505-charger.h - CHARGER H for SC2705
 * Copyright (c) 2017 Dialog Semiconductor.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SC2705_CHARGER_H
#define __SC2705_CHARGER_H

void sc2705_charger_ic_init(void);
void sc2705_charger_start(void);
void sc2705_charger_stop(unsigned int flag);
unsigned int sc2705_charger_cc_current_get(void);
void sc2705_charger_cc_current_set(unsigned int val);
void sc2705_charger_input_current_set(unsigned int val);
void sc2705_charger_otg_enable(int enable);
void sc2705_termina_vol_set(unsigned int val);
int sc2705_charger_status_get(void);
void sc2705_charger_vindpm_set(int val);
void sc2705_charger_reset_timer(void);
#endif
