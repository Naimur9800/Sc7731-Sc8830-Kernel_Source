/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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

#ifndef __SPRD_AUDIO_WHALE2_H
#define __SPRD_AUDIO_WHALE2_H

#ifndef __SPRD_AUDIO_H
#error  "Don't include this file directly, include sprd-audio.h"
#endif

#include "agdsp_access.h"


/* these are cpu dai id, comunicate with dsp, negotiate with dsp. */
enum {
	VBC_DAI_ID_NORMAL_OUTDSP = 0,
	VBC_DAI_ID_NORMAL_WITHDSP,
	VBC_DAI_ID_FAST_P,
	VBC_DAI_ID_OFFLOAD,
	/*default ad2 + da1*/
	VBC_DAI_ID_VOICE,
	/*default ad2 + da1*/
	VBC_DAI_ID_VOIP,
	/*default ad3 + outdsp*/
	VBC_DAI_ID_FM,
	VBC_DAI_ID_FM_C_WITHDSP,
	VBC_DAI_ID_VOICE_CAPTURE,
	VBC_DAI_ID_LOOP_RECORD,
	VBC_DAI_ID_LOOP_PLAY,
	VBC_DAI_ID_VOIP_RECORD,
	VBC_DAI_ID_VOIP_PLAY,
	VBC_DAI_ID_FM_CAPTURE,
	VBC_DAI_ID_BT_CAPTURE,
	VBC_DAI_ID_MAX
};
/* Audio iis master settings */
enum {
	AUD_IIS_MASTER_IIS0_DISABLE = 0,
	AUD_IIS_MASTER_LOOP_DISABLE,
	AUD_IIS_MASTER_IIS0,
	AUD_IIS_MASTER_LOOP,
	AUD_IIS_MASTER_MAX
};

#define CODEC_DP_BASE		(0x1000)
#define CODEC_AP_BASE		(0x2000)
#define CODEC_AP_OFFSET		(0)

#define CLASS_G_LDO_ID			"vddclsg"

/* AGCP AHB registers doesn't defined by global header file. So
 * define them here.
 */
#define REG_AGCP_AHB_MODULE_EB0_STS	(0x00)
#define REG_AGCP_AHB_MODULE_RST0_STS	(0x08)
#define REG_AGCP_AHB_EXT_ACC_AG_SEL	(0x3c)

/*--------------------------------------------------
 * Register Name   : REG_AGCP_AHB_MODULE_EB0_STS
 * Register Offset : 0x0000
 * Description     :
 * ---------------------------------------------------
 */
#define BIT_AUDIF_CKG_AUTO_EN	BIT(20)
#define BIT_AUD_EB				BIT(19)

/*--------------------------------------------------
 * Register Name   : REG_AGCP_AHB_MODULE_RST0_STS
 * Register Offset : 0x0008
 * Description     :
 * ---------------------------------------------------
 */
#define BIT_AUD_SOFT_RST		BIT(25)

/*--------------------------------------------------
 * Register Name   : REG_AGCP_AHB_EXT_ACC_AG_SEL
 * Register Offset : 0x003c
 * Description     :
 * ---------------------------------------------------
 */
#define BIT_AG_IIS2_EXT_SEL                      BIT(2)
#define BIT_AG_IIS1_EXT_SEL                      BIT(1)
#define BIT_AG_IIS0_EXT_SEL                      BIT(0)

/* ----------------------------------------------- */
enum ag_iis {
	AG_IIS0,
	AG_IIS1,
	AG_IIS2,
	AG_IIS_MAX
};

#define sprd_is_normal_playback(cpu_dai_id, stream) \
	((cpu_dai_id) == VBC_DAI_ID_NORMAL_OUTDSP && \
	(stream) == SNDRV_PCM_STREAM_PLAYBACK)

/* AGCP IIS multiplexer setting.
 * @iis: the iis channel to be set.
 * @en:
 *   0: AG_IIS0_EXT_SEL to whale2 top
 *   1: AG_IIS0_EXT_SEL to audio top
 */
static inline int arch_audio_iis_to_audio_top_enable(
	enum ag_iis iis, int en)
{
	u32 val;
	int ret;

	agcp_ahb_gpr_null_check();

	switch (iis) {
	case AG_IIS0:
		val = BIT_AG_IIS0_EXT_SEL;
		break;
	case AG_IIS1:
		val = BIT_AG_IIS1_EXT_SEL;
		break;
	case AG_IIS2:
		val = BIT_AG_IIS2_EXT_SEL;
		break;
	default:
		pr_err("%s, agcp iis mux setting error!\n", __func__);
		return -1;
	};

	ret = agdsp_access_enable();
	if (ret) {
		pr_err("%s, agdsp_access_enable failed!\n", __func__);
		return ret;
	}

	if (en)
		agcp_ahb_reg_hw_set(REG_AGCP_AHB_EXT_ACC_AG_SEL, val);
	else
		agcp_ahb_reg_hw_clr(REG_AGCP_AHB_EXT_ACC_AG_SEL, val);
	agdsp_access_disable();

	return 0;
}

/* Codec digital part in soc setting */
static inline int arch_audio_codec_digital_reg_enable(void)
{
	int ret = 0;

	agcp_ahb_gpr_null_check();
	ret = agdsp_access_enable();
	if (ret) {
		pr_err("%s, agdsp_access_enable failed!\n", __func__);
		return ret;
	}
	ret = agcp_ahb_reg_set(REG_AGCP_AHB_MODULE_EB0_STS, BIT_AUD_EB);
	if (ret >= 0)
		ret = agcp_ahb_reg_set(REG_AGCP_AHB_MODULE_EB0_STS,
			BIT_AUDIF_CKG_AUTO_EN);

	agdsp_access_disable();

	return ret;
}

static inline int arch_audio_codec_digital_reg_disable(void)
{
	int ret = 0;

	agcp_ahb_gpr_null_check();
	ret = agdsp_access_enable();
	if (ret) {
		pr_err("%s, agdsp_access_enable failed!\n", __func__);
		return ret;
	}
	ret = agcp_ahb_reg_clr(REG_AGCP_AHB_MODULE_EB0_STS,
		BIT_AUDIF_CKG_AUTO_EN);
	if (ret >= 0)
		ret = agcp_ahb_reg_clr(REG_AGCP_AHB_MODULE_EB0_STS, BIT_AUD_EB);

	agdsp_access_disable();

	return ret;
}

static inline int arch_audio_codec_digital_enable(void)
{
	return 0;
}

static inline int arch_audio_codec_digital_disable(void)
{
	return 0;
}

static inline int arch_audio_codec_switch2ap(void)
{
	return 0;
}

static inline int arch_audio_codec_digital_reset(void)
{
	int ret = 0;

	ret = agdsp_access_enable();
	if (ret) {
		pr_err("%s, agdsp_access_enable failed!\n", __func__);
		return ret;
	}
	agcp_ahb_gpr_null_check();
	agcp_ahb_reg_set(REG_AGCP_AHB_MODULE_RST0_STS, BIT_AUD_SOFT_RST);
	udelay(10);
	agcp_ahb_reg_clr(REG_AGCP_AHB_MODULE_RST0_STS, BIT_AUD_SOFT_RST);

	agdsp_access_disable();

	return ret;
}

#endif/* __SPRD_AUDIO_WHALE2_H */
