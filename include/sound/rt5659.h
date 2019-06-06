/*
 * include/sound/rt5659.h -- Platform data for RT5659
 *
 * Copyright 2013 Realtek Microelectronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_RT5659_H
#define __LINUX_SND_RT5659_H

#include <sound/soc.h>

enum rt5659_dmic1_data_pin {
	RT5659_DMIC1_NULL,
	RT5659_DMIC1_DATA_IN2N,
	RT5659_DMIC1_DATA_GPIO5,
	RT5659_DMIC1_DATA_GPIO9,
	RT5659_DMIC1_DATA_GPIO11,
};

enum rt5659_dmic2_data_pin {
	RT5659_DMIC2_NULL,
	RT5659_DMIC2_DATA_IN2P,
	RT5659_DMIC2_DATA_GPIO6,
	RT5659_DMIC2_DATA_GPIO10,
	RT5659_DMIC2_DATA_GPIO12,
};

/* System Clock Source */
enum {
	RT5659_SCLK_S_MCLK,
	RT5659_SCLK_S_PLL1,
	RT5659_SCLK_S_RCCLK,
};

/* PLL1 Source */
enum {
	RT5659_PLL1_S_MCLK,
	RT5659_PLL1_S_BCLK1,
	RT5659_PLL1_S_BCLK2,
	RT5659_PLL1_S_BCLK3,
	RT5659_PLL1_S_BCLK4,
};

struct rt5659_platform_data {
	bool in1_diff;
	bool in3_diff;
	bool in4_diff;

	enum rt5659_dmic1_data_pin dmic1_data_pin;
	enum rt5659_dmic2_data_pin dmic2_data_pin;

	const char *regulator_1v8;
	const char *regulator_3v3;
	const char *regulator_5v;

	int gpio_ldo;
	int gpio_reset;
	int gpio_jd1_sel;
	int gpio_jd2_sel;
	/* If uses one pin(GPIO of rt5659) to detect headset inserting/removing,
	 * set it non-zero. Else combination of GPIO and JD3 pin are used for
	 * headset detecting.
	 */
	int hp_det_use_one_pin;
	unsigned int fixed_rate;
};

int rt5659_headset_detect(struct snd_soc_codec *codec, int jack_insert);
int rt5659_button_detect(struct snd_soc_codec *codec);
int rt5659_check_jd_status(struct snd_soc_codec *codec);
#endif

