/*
 * sound/soc/sprd/vbc-r3p0-realtek_machine.c
 *
 * Copyright (C) 2015 SpreadTrum Ltd.
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

#include "sprd-asoc-debug.h"
#define pr_fmt(fmt) pr_sprd_fmt("BOARD")""fmt

#include <linux/of_gpio.h>
#include <uapi/linux/input.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/rt5659.h>
#include <sound/soc.h>

#include "sprd-asoc-card-utils.h"
#include "sprd-asoc-common.h"

struct rt5659_machine_priv {
	struct sprd_card_data *card_data;
	struct snd_soc_codec *codec;
	struct snd_soc_jack rt5659_hp_jack;
	struct snd_soc_jack_gpio rt5659_hp_jack_gpio;
	bool jd_status;
	int det_delay_time;
	unsigned long det_jd_time;
	struct mutex mutex;
	int headset_state;
};

int dsp_fm_mute_by_set_dg(void)
	__attribute__ ((weak, alias("__dsp_fm_mute_by_set_dg")));

static int __dsp_fm_mute_by_set_dg(void)
{
	pr_err("ERR: dsp_fm_mute_by_set_dg is not defined!\n");

	return -1;
}

static int rt5659_jack_status_check(void *data);

static struct rt5659_machine_priv s5pv210_rt5659_priv = {
	.rt5659_hp_jack_gpio = {
		.name = "headphone detect",
		.invert = 1,
		.report = SND_JACK_HEADSET | SND_JACK_BTN_0 | SND_JACK_BTN_1
			| SND_JACK_BTN_2 | SND_JACK_BTN_3,
		.debounce_time = 0,
		.wake = true,
		.jack_status_check = rt5659_jack_status_check,
	},
};

static int rt5659_codec_info_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("%s, do nothing\n", __func__);

	return 0;
}

static int rt5659_codec_info_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = AUDIO_CODEC_RT5659;

	pr_info("%s, codec info = %ld\n",
		__func__, ucontrol->value.integer.value[0]);

	return 0;
}

static const char * const codec_hw_info[] = {
	CODEC_HW_INFO
};
static const struct soc_enum codec_info_enum =
	SOC_ENUM_SINGLE_EXT(SP_AUDIO_CODEC_NUM, codec_hw_info);

static const struct snd_kcontrol_new vbc_r3p0_codec_rt5659_controls[] = {
	SOC_ENUM_EXT("Aud Codec Info", codec_info_enum,
			rt5659_codec_info_get, rt5659_codec_info_put),
};

static ssize_t headset_state_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buff)
{
	pr_debug("%s headset_state = %d\n",
		__func__, s5pv210_rt5659_priv.headset_state);

	return sprintf(buff, "%d\n", s5pv210_rt5659_priv.headset_state);
}

static ssize_t headset_state_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buff, size_t len)
{
	return len;
}

static int headset_state_sysfs_init(void)
{
	int ret = -1;
	static struct kobject *headset_state_kobj;
	static struct kobj_attribute headset_state_attr =
		__ATTR(state, 0644,
		headset_state_show,
		headset_state_store);

	headset_state_kobj = kobject_create_and_add("headset", kernel_kobj);
	if (headset_state_kobj == NULL) {
		ret = -ENOMEM;
		pr_err("register sysfs failed. ret = %d\n", ret);
		return ret;
	}

	ret = sysfs_create_file(headset_state_kobj, &headset_state_attr.attr);
	if (ret) {
		pr_err("create sysfs failed. ret = %d\n", ret);
		return ret;
	}

	pr_info("%s success\n", __func__);

	return ret;
}

static int rt5659_jack_status_check(void *data)
{
	int report = 0;
	int ret = 0;
	int val = 0;
	struct rt5659_machine_priv *priv = &s5pv210_rt5659_priv;
	struct snd_soc_codec *codec = priv->codec;

	sprd_msleep(priv->det_delay_time);
	mutex_lock(&priv->mutex);

	if (rt5659_check_jd_status(codec) && !priv->jd_status) {
		/* jack insert, then to check JD type */
		priv->jd_status = true;
		ret = rt5659_headset_detect(codec, 1);
		if ((ret & SND_JACK_HEADSET) == SND_JACK_HEADSET) {
			report |= SND_JACK_HEADSET;
			priv->det_delay_time = 50;
			priv->det_jd_time = jiffies;
			priv->headset_state = 1;
		} else {
			report |= SND_JACK_HEADPHONE;
			priv->headset_state = 2;
		}
		pr_info("%s: Jack inserted - %#x\n", __func__, ret);
	} else if (rt5659_check_jd_status(codec) &&
		priv->jd_status) {
		/* do button detect */
		val = rt5659_button_detect(codec);
		if (time_before(jiffies,
		    priv->det_jd_time + HZ)) {
			pr_warn("%s Ignore all buttons reporting in 1s after headset detecting.\n",
				__func__);
			mutex_unlock(&priv->mutex);
			return SND_JACK_HEADSET;
		}
		switch (val) {
		case 0x8000:
		case 0x4000:
		case 0x2000:
			report |= SND_JACK_BTN_0;
			break;
		case 0x1000:
		case 0x0800:
		case 0x0400:
			report |= SND_JACK_BTN_1;
			break;
		case 0x0200:
		case 0x0100:
		case 0x0080:
			report |= SND_JACK_BTN_2;
			break;
		case 0x0040:
		case 0x0020:
		case 0x0010:
			report |= SND_JACK_BTN_3;
			break;
		default:
			break;
		}
		pr_debug("key val=0x%04x\n", val);

		if (report)
			pr_debug("Jack key pressed - 0x%04x\n", report);
		else
			pr_debug("Jack key realsed\n");

		report |= SND_JACK_HEADSET;
	} else {
		dsp_fm_mute_by_set_dg();
		/* jack unplug */
		if (priv->jd_status)
			pr_info("%s: Jack removed\n", __func__);

		priv->jd_status = false;
		rt5659_headset_detect(codec, 0);
		priv->det_delay_time = 200;
		priv->headset_state = 0;
	}

	mutex_unlock(&priv->mutex);

	return report;
}

static int _sprd_rt5659_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct rt5659_machine_priv *priv = &s5pv210_rt5659_priv;
	struct snd_soc_jack *jack = &priv->rt5659_hp_jack;
	struct snd_soc_card *card = rtd->card;
	int ret;

	pr_info("%s\n", __func__);

	/* init rt5659_machine_priv */
	mutex_init(&priv->mutex);
	priv->jd_status = false;
	priv->codec = codec;
	priv->det_delay_time = 200;

	if (gpio_is_valid(priv->rt5659_hp_jack_gpio.gpio)) {
		ret = snd_soc_card_jack_new(card,
			"Headphone Jack", SND_JACK_HEADSET, jack, NULL, 0);
		if (ret < 0) {
			pr_err("snd_soc_jack_new failed\n");
			return -1;
		}

		ret = snd_soc_jack_add_gpios(
			jack, 1, &priv->rt5659_hp_jack_gpio);
		if (ret < 0) {
			pr_err("snd_soc_jack_add_gpios failed\n");
			return -1;
		}

		ret = snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_MEDIA);
		ret |= snd_jack_set_key(jack->jack,
			SND_JACK_BTN_1, KEY_VOLUMEUP);
		ret |= snd_jack_set_key(jack->jack,
			SND_JACK_BTN_2, KEY_VOLUMEDOWN);
		ret |= snd_jack_set_key(jack->jack,
			SND_JACK_BTN_3, KEY_VOLUMEDOWN);
		if (ret < 0) {
			pr_err("%s snd_jack_set_key failed\n", __func__);
			return -1;
		}
	} else {
		pr_err("gpio is not valid\n");
		return -1;
	}

	headset_state_sysfs_init();

	return 0;
}

static int (*sprd_rt5659_init[]) (struct snd_soc_pcm_runtime *rtd) = {
	_sprd_rt5659_init,
};

static int sprd_rt5658_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret = 0;
	unsigned int rate;
	int channels = params_channels(params);
	snd_pcm_format_t fmt = params_format(params);
	unsigned int RCLK;
	struct rt5659_platform_data *pdata =
		snd_soc_codec_get_drvdata(rtd->codec);

	/* sample rate * 256fs */
	rate = pdata->fixed_rate > 0 ? pdata->fixed_rate : params_rate(params);
	RCLK = rate * 256;

	pr_info("rate=%u, channels=%d, fmt=%d\n", rate, channels, fmt);

	ret = snd_soc_dai_set_sysclk(codec_dai,
		RT5659_SCLK_S_PLL1, RCLK * 2, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("snd_soc_dai_set_sysclk failed ret=%d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai,
		0, RT5659_PLL1_S_MCLK, 12288000, RCLK * 2);
	if (ret < 0) {
		pr_err("snd_soc_dai_set_pll failed ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops sprd_rt5658_ops[] = {
	{.hw_params = sprd_rt5658_hw_params,},
};

static int rt5659_board_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_dapm_context *codec_dapm =
		snd_soc_component_get_dapm(&card->rtd->codec->component);

	sprd_audio_debug_init(card->snd_card);

	snd_soc_dapm_ignore_suspend(&card->dapm, "Headset-mic");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Main-mic");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Aux-mic");
	/* snd_soc_dapm_ignore_suspend(&card->dapm, "ANC-mic"); */

	snd_soc_dapm_ignore_suspend(&card->dapm, "Headphone Jack");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Speaker");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Earpiece");

	snd_soc_dapm_ignore_suspend(codec_dapm, "AIF1 Playback ofld");
	snd_soc_dapm_ignore_suspend(codec_dapm, "AIF1 Playback fm");
	snd_soc_dapm_ignore_suspend(codec_dapm, "AIF1 Playback voice");
	snd_soc_dapm_ignore_suspend(codec_dapm, "AIF1 Capture voice");

	return 0;
}

static int vbc_r3p0_codec_rt5659_probe(struct platform_device *pdev)
{
	int ret;
	struct rt5659_machine_priv *priv = &s5pv210_rt5659_priv;
	struct snd_soc_card *card;
	struct asoc_sprd_ptr_num pn_ops = ASOC_SPRD_PRT_NUM(sprd_rt5658_ops);
	struct asoc_sprd_ptr_num pn_init = ASOC_SPRD_PRT_NUM(sprd_rt5659_init);

	asoc_sprd_card_set_ops(&pn_ops);
	asoc_sprd_card_set_init(&pn_init);

	ret = asoc_sprd_card_probe(pdev, &card);
	if (ret) {
		pr_err("err: %s asoc_sprd_card_probe failed!\n", __func__);
		return ret;
	}

	priv->card_data = snd_soc_card_get_drvdata(card);
	priv->rt5659_hp_jack_gpio.gpio = priv->card_data->gpio_hp_det;

	card->late_probe = rt5659_board_late_probe;

	return asoc_sprd_register_card(&pdev->dev, card);
}

static int vbc_r3p0_codec_rt5659_remove(struct platform_device *pdev)
{
	struct rt5659_machine_priv *priv = &s5pv210_rt5659_priv;

	if (gpio_is_valid(priv->card_data->gpio_hp_det))
		snd_soc_jack_free_gpios(&priv->rt5659_hp_jack, 1,
					&priv->rt5659_hp_jack_gpio);

	return asoc_sprd_card_remove(pdev);
}

#ifdef CONFIG_OF
static const struct of_device_id vbc_r3p0_codec_rt5659_of_match[] = {
	{.compatible = "sprd,vbc-r3p0-codec-rt5659",},
	{},
};

MODULE_DEVICE_TABLE(of, vbc_r3p0_codec_rt5659_of_match);
#endif

static struct platform_driver vbc_r3p0_codec_rt5659_driver = {
	.driver = {
		.name = "vbc-r3p0-codec-rt5659",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(vbc_r3p0_codec_rt5659_of_match),
	},
	.probe = vbc_r3p0_codec_rt5659_probe,
	.remove = vbc_r3p0_codec_rt5659_remove,
};

static int __init vbc_r3p0_codec_rt5659_init(void)
{
	return platform_driver_register(&vbc_r3p0_codec_rt5659_driver);
}

late_initcall_sync(vbc_r3p0_codec_rt5659_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASoC SPRD Sound Card VBC R3p0 & Codec Rt5659");
MODULE_AUTHOR("Peng Lee <peng.lee@spreadtrum.com>");
