/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/types.h>

#include "sin_table_fixed.c"

#define INTERPOLATION_STEP 128

int scale_sin_32(int index_in)
{
	int sin_cnt = 4096 * INTERPOLATION_STEP;
	int index = (index_in % sin_cnt) / INTERPOLATION_STEP;
	int depart = (index_in / sin_cnt) % 4;
	int sin_value_up;
	int sin_value_down;
	int sin_value;
	int index_diff;

	index_in = index_in % sin_cnt;

	if (1 == depart || 3 == depart) {
		index = 4096 - 1 - index;
		index_in = 4096 * 128 - 1 - index_in;
	}

	if (index == 4095) {
		sin_value_down = sin_table[index];
		sin_value_up = sin_value_down;
		index_diff = 0;
	} else {
		if (1 == depart || 3 == depart) {
			sin_value_down = sin_table[index - 1];
			sin_value_up = sin_table[index];
			index_diff = index_in - (index - 1) * 128;
		} else {
			sin_value_down = sin_table[index];
			sin_value_up = sin_table[index + 1];
			index_diff = index_in - index * 128;
		}
	}

	sin_value = sin_value_down +
	    (sin_value_up - sin_value_down) * index_diff / 128;

	if (2 == depart || 3 == depart)
		sin_value *= -1;

	return sin_value;
}

int scale_cos_32(int index_in)
{
	int cos_cnt = 4096 * INTERPOLATION_STEP;
	int index = (index_in % cos_cnt) / INTERPOLATION_STEP;
	int depart = (index_in / cos_cnt) % 4;
	int sin_value_up;
	int sin_value_down;
	int sin_value;
	int index_diff;

	index_in = index_in % cos_cnt;

	if (0 == depart || 2 == depart) {
		index = 4096 - 1 - index;
		index_in = 4096 * 128 - 1 - index_in;
	}

	if (index == 4095) {
		sin_value_down = sin_table[index];
		sin_value_up = sin_value_down;
		index_diff = 0;
	} else {
		if (1 == depart || 3 == depart) {
			sin_value_down = sin_table[index - 1];
			sin_value_up = sin_table[index];
			index_diff = index_in - (index - 1) * 128;
		} else {
			sin_value_down = sin_table[index];
			sin_value_up = sin_table[index + 1];
			index_diff = index_in - index * 128;
		}
	}

	sin_value = sin_value_down +
	    (sin_value_up - sin_value_down) * index_diff / 128;

	if (1 == depart || 2 == depart)
		sin_value *= -1;

	return sin_value;
}
