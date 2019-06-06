/*
 * sound/soc/sprd/dai/vbc/r3p0/vbc-codec.c
 *
 * SPRD SoC VBC Codec -- SpreadTrum SOC VBC Codec function.
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
#define pr_fmt(fmt) pr_sprd_fmt(" VBC ") ""fmt

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/pcm_params.h>
#include <linux/pinctrl/consumer.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include "audio-sipc.h"
#include "sprd-asoc-card-utils.h"
#include "sprd-asoc-common.h"
#include "sprd-string.h"
#include "vbc-codec.h"
#include "vbc-phy-r3p0.h"

#define SPRD_VBC_ENUM(xreg, xmax, xtexts)\
		  SOC_ENUM_SINGLE(FUN_REG(xreg), 0, xmax, xtexts)

#define SPRD_VBC_GAIN_MAX(xname, xreg, max, tlv_array) \
	SOC_SINGLE_EXT_TLV(xname, FUN_REG(xreg), 0, max, 0, \
			sprd_codec_vol_get, sprd_codec_vol_put, tlv_array)

static const char * const adc0_source_sel_txt[] = {
	"iis0", "vbc_if",
};
static const char * const adc1_source_sel_txt[] = {
	"iis1", "vbc_if",
};
static const char * const adc2_source_sel_txt[] = {
	"iis2", "vbc_if",
};
static const char * const adc3_source_sel_txt[] = {
	"iis3", "vbc_if",
};
static const char * const adc_in_sel_txt[] = {
	"adc0_source mux", "adc1_source mux", "adc2_source mux",
	"adc3_source mux", "dac0", "dac1", "dac loop",
};
static const char * const fm_sel_txt[] = {
	"fm src out", "vbc_if adc0", "vbc_if adc1", "vbc_if adc2",
};
static const char * const st_sel_txt[] = {
	"adc0", "adc0 dg out", "adc1", "adc1 dg out", "adc2",
	"adc2 dg out", "adc3", "adc3 dg out",
};
static const char * const dac_adc_in_sel_txt[] = {
	"smthdg out", "mix1 out", "eq4 out", "mbdrc out",
};
static const char * const dac0_dac1_in_sel_txt[] = {
	"dac1", "dac0",
};
static const char * const audrcd_in_sel1_txt[] = {
	"adc0", "adc1", "adc2", "adc3",
};
static const char * const dac_out_sel_txt[] = {
	"iis", "vbc_if",
};
static const char * const iis_sel_txt[] = {
	"iis0", "iis1", "iis2", "iis3",
};
static const char * const adder_mode_txt[] = {
	"IGNORE", "ADD", "MINUS",
};

static const char * const dsp_loopback_type_txt[] = {
	"ADDA", "AD_ULDL_DA_PROCESS", "AD_UL_ENCODE_DECODE_DL_DA_PROCESS",

};

static const char * const data_path_sel_txt[] = {
	"disable", "enable",
};

static const char * const sys_iis_sel_txt[] = {
	"vbc_iis0", "vbc_iis1", "vbc_iis2", "vbc_iis3",
};

static const char * const enable_disable_txt[] = {
	"disable", "enable",
};

static const char * const vbc_iis_width_txt[] = {
	"bit16", "bit24",
};

static const char * const vbc_iis_lr_mod_txt[] = {
	"left_high", "right_high",
};

static const char * const iis_master_setting_txt[] = {
	"disable_iis0", "disable_loop", "iis0", "loop",
};

static const char * const dac_mixer_out_sel_text[] = {
	"original_data", "opposite_num_data",
};

static const char * const dac_mixer_left_mux_sel_text[] = {
	"left_channel", "right_channel",
	"half_left_add_right", "half_left_sub_right"
};

static const char * const dac_mixer_right_mux_sel_text[] = {
	"right_channel", "left_channel",
	"half_left_add_right", "half_left_sub_right"
};

static const struct soc_enum iis_master_setting_enum  =
	SPRD_VBC_ENUM(SND_SOC_NOPM, 4, iis_master_setting_txt);

static const struct soc_enum dsp_loopback_enum  =
	SPRD_VBC_ENUM(SND_SOC_NOPM, 3, dsp_loopback_type_txt);

static const struct soc_enum
vbc_mixer_out_sel_enum[VBC_DAC_MIXER_MAX - VBC_DAC_MIXER_START] = {
	SPRD_VBC_ENUM(VBC_DAC0_MIXER0, 2, dac_mixer_out_sel_text),
	SPRD_VBC_ENUM(VBC_DAC0_MIXER1, 2, dac_mixer_out_sel_text),
	SPRD_VBC_ENUM(VBC_DAC0_MIXER2, 2, dac_mixer_out_sel_text),
	SPRD_VBC_ENUM(VBC_DAC1_MIXER0, 2, dac_mixer_out_sel_text),
	SPRD_VBC_ENUM(VBC_DAC1_MIXER1, 2, dac_mixer_out_sel_text),
	SPRD_VBC_ENUM(VBC_DAC1_MIXER2, 2, dac_mixer_out_sel_text),
};

static const struct soc_enum
vbc_mixer_left_mux_sel_enum[VBC_DAC_MIXER_MAX - VBC_DAC_MIXER_START] = {
	SPRD_VBC_ENUM(VBC_DAC0_MIXER0, 4, dac_mixer_left_mux_sel_text),
	SPRD_VBC_ENUM(VBC_DAC0_MIXER1, 4, dac_mixer_left_mux_sel_text),
	SPRD_VBC_ENUM(VBC_DAC0_MIXER2, 4, dac_mixer_left_mux_sel_text),
	SPRD_VBC_ENUM(VBC_DAC1_MIXER0, 4, dac_mixer_left_mux_sel_text),
	SPRD_VBC_ENUM(VBC_DAC1_MIXER1, 4, dac_mixer_left_mux_sel_text),
	SPRD_VBC_ENUM(VBC_DAC1_MIXER2, 4, dac_mixer_left_mux_sel_text),
};

static const struct soc_enum
vbc_mixer_right_mux_sel_enum[VBC_DAC_MIXER_MAX - VBC_DAC_MIXER_START] = {
	SPRD_VBC_ENUM(VBC_DAC0_MIXER0, 4, dac_mixer_right_mux_sel_text),
	SPRD_VBC_ENUM(VBC_DAC0_MIXER1, 4, dac_mixer_right_mux_sel_text),
	SPRD_VBC_ENUM(VBC_DAC0_MIXER2, 4, dac_mixer_right_mux_sel_text),
	SPRD_VBC_ENUM(VBC_DAC1_MIXER0, 4, dac_mixer_right_mux_sel_text),
	SPRD_VBC_ENUM(VBC_DAC1_MIXER1, 4, dac_mixer_right_mux_sel_text),
	SPRD_VBC_ENUM(VBC_DAC1_MIXER2, 4, dac_mixer_right_mux_sel_text),
};

static const struct soc_enum
vbc_mux_sel_enum[VBC_MUX_MAX-VBC_MUX_START] = {
	/*ADC0 MUX */
	SPRD_VBC_ENUM(VBC_MUX_ADC0_SOURCE, 2, adc0_source_sel_txt),
	SPRD_VBC_ENUM(VBC_MUX_ADC0_INSEL, 7, adc_in_sel_txt),
	/*ADC1 MUX */
	SPRD_VBC_ENUM(VBC_MUX_ADC1_SOURCE, 2, adc1_source_sel_txt),
	SPRD_VBC_ENUM(VBC_MUX_ADC1_INSEL, 7, adc_in_sel_txt),
	/*ADC2 MUX */
	SPRD_VBC_ENUM(VBC_MUX_ADC2_SOURCE, 2, adc2_source_sel_txt),
	SPRD_VBC_ENUM(VBC_MUX_ADC2_INSEL, 7, adc_in_sel_txt),
	/*ADC3 MUX */
	SPRD_VBC_ENUM(VBC_MUX_ADC3_SOURCE, 2, adc3_source_sel_txt),
	SPRD_VBC_ENUM(VBC_MUX_ADC3_INSEL, 7, adc_in_sel_txt),
	/*FM MUX */
	SPRD_VBC_ENUM(VBC_MUX_FM_INSEL, 4, fm_sel_txt),
	/*ST MUX */
	SPRD_VBC_ENUM(VBC_MUX_ST_INSEL, 8, st_sel_txt),
	/* LOOP MUX */
	SPRD_VBC_ENUM(VBC_MUX_DAC0_ADC_INSEL, 4, dac_adc_in_sel_txt),
	SPRD_VBC_ENUM(VBC_MUX_DAC1_ADC_INSEL, 4, dac_adc_in_sel_txt),
	SPRD_VBC_ENUM(VBC_MUX_DAC0_DAC1_INSEL, 2, dac0_dac1_in_sel_txt),
	/* AUDRCD MUX */
	SPRD_VBC_ENUM(VBC_MUX_AUDRCD_INSEL1, 4, audrcd_in_sel1_txt),
	/* DAC OUT SEL MUX */
	SPRD_VBC_ENUM(VBC_MUX_DAC0_OUT_SEL, 2, dac_out_sel_txt),
	SPRD_VBC_ENUM(VBC_MUX_DAC1_OUT_SEL, 2, dac_out_sel_txt),
	/* DAC IIS OUT SEL MUX */
	SPRD_VBC_ENUM(VBC_MUX_DAC0_IIS_SEL, 4, iis_sel_txt),
	SPRD_VBC_ENUM(VBC_MUX_DAC1_IIS_SEL, 4, iis_sel_txt),
	/* ADC IIS IN SEL MUX */
	SPRD_VBC_ENUM(VBC_MUX_ADC0_IIS_SEL, 4, iis_sel_txt),
	SPRD_VBC_ENUM(VBC_MUX_ADC1_IIS_SEL, 4, iis_sel_txt),
	SPRD_VBC_ENUM(VBC_MUX_ADC2_IIS_SEL, 4, iis_sel_txt),
	SPRD_VBC_ENUM(VBC_MUX_ADC3_IIS_SEL, 4, iis_sel_txt),
};

static const struct soc_enum
vbc_adder_enum[VBC_ADDER_MAX-VBC_ADDER_START] = {
	/* ADDER */
	SPRD_VBC_ENUM(VBC_ADDER_AUDPLY_DAC0, 3, adder_mode_txt),
	SPRD_VBC_ENUM(VBC_ADDER_FM_DAC0, 3, adder_mode_txt),
	SPRD_VBC_ENUM(VBC_ADDER_SMTHDG_VOICE_DAC0, 3, adder_mode_txt),
	SPRD_VBC_ENUM(VBC_ADDER_MBDRC_VOICE_DAC0, 3, adder_mode_txt),
	SPRD_VBC_ENUM(VBC_ADDER_ST_DAC0, 3, adder_mode_txt),
	SPRD_VBC_ENUM(VBC_ADDER_AUDPLY_DAC1, 3, adder_mode_txt),
	SPRD_VBC_ENUM(VBC_ADDER_FM_DAC1, 3, adder_mode_txt),
	SPRD_VBC_ENUM(VBC_ADDER_SMTHDG_VOICE_DAC1, 3, adder_mode_txt),
	SPRD_VBC_ENUM(VBC_ADDER_MBDRC_VOICE_DAC1, 3, adder_mode_txt),
	SPRD_VBC_ENUM(VBC_ADDER_ST_DAC1, 3, adder_mode_txt),
};

static const struct soc_enum
vbc_sys_iis_enum[SYS_IIS_MAX-SYS_IIS_START] = {
	SPRD_VBC_ENUM(SYS_IIS0, 4, sys_iis_sel_txt),
	SPRD_VBC_ENUM(SYS_IIS1, 4, sys_iis_sel_txt),
	SPRD_VBC_ENUM(SYS_IIS2, 4, sys_iis_sel_txt),
	SPRD_VBC_ENUM(SYS_IIS3, 4, sys_iis_sel_txt),
	SPRD_VBC_ENUM(SYS_IIS4, 4, sys_iis_sel_txt),
	SPRD_VBC_ENUM(SYS_IIS5, 4, sys_iis_sel_txt),
	SPRD_VBC_ENUM(SYS_IIS6, 4, sys_iis_sel_txt),
};

static const struct soc_enum
vbc_datapath_enum[VBC_DP_EN_MAX-VBC_DP_EN_START] = {
	SPRD_VBC_ENUM(VBC_DAC0_DP_EN, 2, data_path_sel_txt),
	SPRD_VBC_ENUM(VBC_DAC1_DP_EN, 2, data_path_sel_txt),
	SPRD_VBC_ENUM(VBC_ADC0_DP_EN, 2, data_path_sel_txt),
	SPRD_VBC_ENUM(VBC_ADC1_DP_EN, 2, data_path_sel_txt),
	SPRD_VBC_ENUM(VBC_ADC2_DP_EN, 2, data_path_sel_txt),
	SPRD_VBC_ENUM(VBC_ADC3_DP_EN, 2, data_path_sel_txt),
	SPRD_VBC_ENUM(VBC_ofld_DP_EN, 2, data_path_sel_txt),
	SPRD_VBC_ENUM(VBC_fm_DP_EN, 2, data_path_sel_txt),
	SPRD_VBC_ENUM(VBC_st_DP_EN, 2, data_path_sel_txt),
};
static const struct soc_enum
vbc_call_mute_enum[VBC_CALL_MUTE_MAX-VBC_CALL_MUTE_START] = {
	SPRD_VBC_ENUM(VBC_UL_MUTE, 2, enable_disable_txt),
	SPRD_VBC_ENUM(VBC_DL_MUTE, 2, enable_disable_txt),
};

static const struct soc_enum vbc_ag_iis_ext_sel_enum[AG_IIS_MAX] = {
	SPRD_VBC_ENUM(AG_IIS0, 2, enable_disable_txt),
	SPRD_VBC_ENUM(AG_IIS1, 2, enable_disable_txt),
	SPRD_VBC_ENUM(AG_IIS2, 2, enable_disable_txt),
};

static const struct soc_enum vbc_dac_iis_wide_sel_enum[VBC_DA_MAX] = {
	SPRD_VBC_ENUM(VBC_DA0, 2, vbc_iis_width_txt),
	SPRD_VBC_ENUM(VBC_DA1, 2, vbc_iis_width_txt),
};

static const struct soc_enum vbc_adc_iis_wide_sel_enum[VBC_AD_MAX] = {
	SPRD_VBC_ENUM(VBC_AD0, 2, vbc_iis_width_txt),
	SPRD_VBC_ENUM(VBC_AD1, 2, vbc_iis_width_txt),
	SPRD_VBC_ENUM(VBC_AD2, 2, vbc_iis_width_txt),
	SPRD_VBC_ENUM(VBC_AD3, 2, vbc_iis_width_txt),
};

static const struct soc_enum vbc_dac_iis_lr_mod_sel_enum[VBC_DA_MAX] = {
	SPRD_VBC_ENUM(VBC_DA0, 2, vbc_iis_lr_mod_txt),
	SPRD_VBC_ENUM(VBC_DA1, 2, vbc_iis_lr_mod_txt),
};

static const struct soc_enum vbc_adc_iis_lr_mod_sel_enum[VBC_AD_MAX] = {
	SPRD_VBC_ENUM(VBC_AD0, 2, vbc_iis_lr_mod_txt),
	SPRD_VBC_ENUM(VBC_AD1, 2, vbc_iis_lr_mod_txt),
	SPRD_VBC_ENUM(VBC_AD2, 2, vbc_iis_lr_mod_txt),
	SPRD_VBC_ENUM(VBC_AD3, 2, vbc_iis_lr_mod_txt),
};

static const struct soc_enum vbc_dump_enum =
	SPRD_VBC_ENUM(SND_SOC_NOPM, 2, enable_disable_txt);

int vbc_codec_getpathinfo(struct device *dev,
	struct snd_pcm_statup_paras *para, struct sprd_vbc_priv *vbc_priv)
{
	int i = 0;
	struct vbc_codec_priv *vbc_codec = dev_get_drvdata(dev);

	/*DAC*/
	para->dac_id = vbc_priv->dac_id;
	para->dac_iis_port = vbc_codec->vbc_mux[vbc_priv->dac_iis_port];
	para->dac_out_sel = vbc_codec->vbc_mux[vbc_priv->out_sel];
	para->dac0_iis_port_for_voice =
		vbc_codec->vbc_mux[VBC_MUX_DAC0_IIS_SEL-VBC_MUX_START];
	/*ADC*/
	para->adc_id = vbc_priv->adc_id;
	para->adc_iis_port = vbc_codec->vbc_mux[vbc_priv->adc_iis_port];
	para->adc_source_sel = vbc_codec->vbc_mux[vbc_priv->adc_source_sel];
	/*iis width 0 16bit, 1 24 bit*/
	para->dac_iis_width = vbc_codec->dac_iis_width[vbc_priv->dac_id];
	para->adc_iis_width = vbc_codec->adc_iis_width[vbc_priv->adc_id];
	/*iis lr mode*/
	para->dac_iis_lr_mod = vbc_codec->dac_iis_lr_mod[vbc_priv->dac_id];
	para->adc_iis_lr_mod = vbc_codec->adc_iis_lr_mod[vbc_priv->adc_id];
	/*loopback parameters*/
	para->loopback.loopback_type = vbc_codec->loopback.loopback_type;
	para->loopback.amr_rate = vbc_codec->loopback.amr_rate;
	para->loopback.voice_fmt = vbc_codec->loopback.voice_fmt;

	for (i = 0; i < VBC_DAC_MIXER_MAX; i++) {
		para->mix_left_out_sel[i] = vbc_codec->mix_left_out_sel[i];
		para->mix_left_mux_sel[i] = vbc_codec->mix_left_mux_sel[i];
		para->mix_right_out_sel[i] = vbc_codec->mix_right_out_sel[i];
		para->mix_right_mux_sel[i] = vbc_codec->mix_right_mux_sel[i];
		sp_asoc_pr_dbg("para->mix_left_out_sel[%d]=%#x, enable=%u,id =%u\n",
			       i, para->mix_left_out_sel[i].dac_mixer_out_sel,
			       para->mix_left_out_sel[i].enable,
			       para->mix_left_out_sel[i].id);
		sp_asoc_pr_dbg("para->mix_left_mux_sel[%d]=%#x enable=%u,id =%u\n",
			       i, para->mix_left_mux_sel[i].dac_mixer_mux_sel,
			       para->mix_left_mux_sel[i].enable,
			       para->mix_left_mux_sel[i].id);
		sp_asoc_pr_dbg("para->mix_right_out_sel[%d]=%#x enable=%u,id =%u\n",
			       i, para->mix_right_out_sel[i].dac_mixer_out_sel,
			       para->mix_right_out_sel[i].enable,
			       para->mix_right_out_sel[i].id);
		sp_asoc_pr_dbg("para->mix_right_mux_sel[%d]=%#x enable=%u,id =%u\n",
			       i, para->mix_right_mux_sel[i].dac_mixer_mux_sel,
			       para->mix_right_mux_sel[i].enable,
			       para->mix_right_mux_sel[i].id);
	}

	sp_asoc_pr_dbg("para->dac_id = %d, para->dac_iis_port=%d\n",
		para->dac_id, para->dac_iis_port);
	sp_asoc_pr_dbg("para->dac0_iis_port_for_voice=%d\n",
		para->dac0_iis_port_for_voice);
	sp_asoc_pr_dbg(" para->adc_id=%d, para->adc_iis_port =%d\n",
		para->adc_id, para->adc_iis_port);
	sp_asoc_pr_dbg("para->dac_iis_width=%d bit\n",
		(para->dac_iis_width == 0) ? 16:24);
	sp_asoc_pr_dbg(" para->adc_iis_width=%d bit\n",
		(para->adc_iis_width == 0) ? 16:24);
	sp_asoc_pr_dbg("para->dac_iis_lr_mod=%s\n",
		(para->dac_iis_lr_mod == 0) ? "left_high":"right_high");
	sp_asoc_pr_dbg(" para->adc_iis_lr_mod =%s\n",
		(para->adc_iis_lr_mod == 0) ? "left_high":"right_high");
	sp_asoc_pr_dbg("para->loopback.loopback_type=%d\n",
		para->loopback.loopback_type);
	sp_asoc_pr_dbg("para->loopback.amr_rate=%d\n",
		para->loopback.amr_rate);
	sp_asoc_pr_dbg("para->loopback.voice_fmt=%d\n",
		para->loopback.voice_fmt);

	return 0;
}

static int vbc_profile_set(struct snd_soc_codec *codec, void *data,
				 int profile_type, int mode)
{
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_profile *p_profile_setting = &vbc_codec->vbc_profile_setting;
	int ret = 0;

	ret = aud_send_block_param(AMSG_CH_VBC_CTL, mode, -1, profile_type,
		data, p_profile_setting->hdr[profile_type].len_mode,
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		return -EIO;

	return 0;
}

static void vbc_profile_try_apply(struct snd_soc_codec *codec,
	int profile_id)
{
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_profile *p_profile_setting = &vbc_codec->vbc_profile_setting;
	void *data;
	unsigned int mode_offset = 0;

	sp_asoc_pr_dbg("%s 0x%p, profile_id:%d\n",
		__func__,
		p_profile_setting->vbc_profile_apply, profile_id);
	if (p_profile_setting->vbc_profile_apply) {
		mode_offset =
		(p_profile_setting->now_mode[profile_id] >> 24) & 0xff;
		mutex_lock(&vbc_codec->load_mutex);
		/* get profile data wanted */
		data = (void *)((u8 *)(p_profile_setting->data[profile_id])
			+ p_profile_setting->hdr[profile_id].len_mode *
			mode_offset);
		sp_asoc_pr_dbg("now_mode[%d]=%d,mode=%u, mode_offset=%u\n",
			profile_id, p_profile_setting->now_mode[profile_id],
			(p_profile_setting->now_mode[profile_id] >> 16) & 0xff,
			mode_offset);
		/* update the para*/
		p_profile_setting->vbc_profile_apply(codec, data, profile_id,
			p_profile_setting->now_mode[profile_id]);
		mutex_unlock(&vbc_codec->load_mutex);
	}
}

/*
 * profile structure:
 *	| fw_header | data |
 */
int vbc_profile_loading(struct snd_soc_codec *codec, int profile_id)
{
	int ret;
	const u8 *fw_data;
	const struct firmware *fw;
	int offset = 0;
	int len = 0;
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_profile *p_profile_setting = &vbc_codec->vbc_profile_setting;

	sp_asoc_pr_dbg("%s %s\n", __func__, vbc_get_profile_name(profile_id));
	mutex_lock(&vbc_codec->load_mutex);
	p_profile_setting->is_loading[profile_id] = 1;

	/* request firmware for AUDIO profile */
	ret = request_firmware(&fw, vbc_get_profile_name(profile_id),
		p_profile_setting->dev);
	if (ret != 0) {
		pr_err("ERR:Failed to load firmware: %d, %s\n", ret,
			vbc_get_profile_name(profile_id));
		goto req_fw_err;
	}
	fw_data = fw->data;
	unalign_memcpy(&p_profile_setting->hdr[profile_id], fw_data,
		sizeof(p_profile_setting->hdr[profile_id]));
	sp_asoc_pr_dbg("&p_profile_setting->hdr[profile_id(%d)]",
		profile_id);
	sp_asoc_pr_dbg(" =%#lx,phys=%#lx\n",
		(unsigned long)&p_profile_setting->hdr[profile_id],
		(unsigned long)
		virt_to_phys(&p_profile_setting->hdr[profile_id]));
	if (strncmp
		(p_profile_setting->hdr[profile_id].magic,
			VBC_PROFILE_FIRMWARE_MAGIC_ID,
		 VBC_PROFILE_FIRMWARE_MAGIC_LEN)) {
		pr_err("ERR:Firmware %s magic error!\n",
			vbc_get_profile_name(profile_id));
		ret = -EINVAL;
		goto profile_out;
	}

	offset = sizeof(struct vbc_fw_header);
	len = p_profile_setting->hdr[profile_id].num_mode *
		p_profile_setting->hdr[profile_id].len_mode;
	if (p_profile_setting->data[profile_id] == NULL) {
		p_profile_setting->data[profile_id] = kzalloc(len, GFP_KERNEL);
		if (p_profile_setting->data[profile_id] == NULL) {
			ret = -ENOMEM;
			goto profile_out;
		}
	}
	unalign_memcpy(p_profile_setting->data[profile_id],
		fw_data + offset, len);
	sp_asoc_pr_dbg("p_profile_setting->data[profile_id (%d)]",
		profile_id);

	sp_asoc_pr_dbg(" =%#lx,phys=%#lx\n",
		(unsigned long)p_profile_setting->data[profile_id],
		(unsigned long)
			virt_to_phys(p_profile_setting->data[profile_id]));
	ret = 0;
	goto profile_out;

profile_out:
	release_firmware(fw);
req_fw_err:
	mutex_unlock(&vbc_codec->load_mutex);
	sp_asoc_pr_info("%s, return %i\n", __func__, ret);

	return ret;
}

static int vbc_try_mdg_set(struct vbc_codec_priv *vbc_codec, int id)
{
	struct vbc_mdg *p_mdg = &vbc_codec->mdg;
	int mdg = p_mdg->mdg_step[id];
	int enable = p_mdg->mute_switch[id] ? 1 : 0;

	p_mdg->mdg_set(id, enable, mdg);

	return 0;
}

static int vbc_try_src_set(struct vbc_codec_priv *vbc_codec, int id)
{
	struct vbc_src *p_src_para = &vbc_codec->vbc_src;

	p_src_para->src_set(id, p_src_para->src_fs[id]);

	return 0;
}

static int vbc_try_dg_set(struct vbc_codec_priv *vbc_codec, int id)
{
	struct vbc_dg *p_dg = &vbc_codec->dg;
	int dg_l = p_dg->dg_l[id];
	int dg_r = p_dg->dg_r[id];

	p_dg->dg_set(id, dg_l, dg_r);

	return 0;
}

static int offload_try_dg_set(struct vbc_codec_priv *vbc_codec, int id)
{
	struct vbc_dg *p_dg = &vbc_codec->offload_dg;
	int dg_l = p_dg->dg_l[id];
	int dg_r = p_dg->dg_r[id];

	p_dg->dg_set(id, dg_l, dg_r);

	return 0;
}

static int vbc_try_smthdg_set(struct vbc_codec_priv *vbc_codec, int id)
{
	struct vbc_smthdg *p_smthdg = &vbc_codec->smthdg;
	int smthdg_l = p_smthdg->smthdg_l[id];
	int smthdg_r = p_smthdg->smthdg_r[id];

	p_smthdg->smthdg_set(id, smthdg_l, smthdg_r);

	return 0;
}

static int vbc_try_smthdg_step_set(struct vbc_codec_priv *vbc_codec, int id)
{
	struct vbc_smthdg *p_smthdg = &vbc_codec->smthdg;
	int smthdg_step = p_smthdg->smthdg_step[id];

	p_smthdg->smthdg_step_set(id, smthdg_step);

	return 0;
}

static int vbc_try_mixerdg_step_set(struct vbc_codec_priv *vbc_codec)
{
	struct vbc_mixerdg *p_mixerdg = &vbc_codec->mixerdg;
	int mixerdg_step = p_mixerdg->mixerdg_step;

	p_mixerdg->mixerdg_step_set(mixerdg_step);

	return 0;
}

static int vbc_try_mixerdg_mainpath_set(struct vbc_codec_priv *vbc_codec,
	int id)
{
	struct vbc_mixerdg *p_mixerdg = &vbc_codec->mixerdg;
	int mixerdg_main_l = p_mixerdg->mixerdg_main_l[id];
	int mixerdg_main_r = p_mixerdg->mixerdg_main_r[id];

	p_mixerdg->mixerdg_mainpath_set(id, mixerdg_main_l, mixerdg_main_r);

	return 0;
}

static int vbc_try_mixerdg_mixpath_set(struct vbc_codec_priv *vbc_codec,
	int id)
{
	struct vbc_mixerdg *p_mixerdg = &vbc_codec->mixerdg;
	int mixerdg_mix_l = p_mixerdg->mixerdg_mix_l[id];
	int mixerdg_mix_r = p_mixerdg->mixerdg_mix_r[id];

	p_mixerdg->mixerdg_mixpath_set(id, mixerdg_mix_l, mixerdg_mix_r);

	return 0;
}

static int vbc_try_vbc_mux_set(int id, int mux_sel)
{
	return dsp_vbc_mux_set(id, mux_sel);
}

static int vbc_try_vbc_adder_set(int id, int adder_mode)
{
	return dsp_vbc_adder_set(id, adder_mode, adder_mode);
}

/*
 * proc interface
 */
#ifdef CONFIG_PROC_FS
static unsigned int dsp_vbc_reg_shm_proc_read(struct snd_info_buffer *buffer)
{
	int ret = 0;
	int reg;
	uint32_t size = 8*2048;
	uint32_t *addr = kzalloc(size, GFP_KERNEL);

	if (!addr)
		return -ENOMEM;

	ret = aud_recv_block_param(AMSG_CH_VBC_CTL,
		-1, -1, SND_VBC_SHM_VBC_REG, addr, size,
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0) {
		kfree(addr);
		return -1;
	}

	snd_iprintf(buffer, "dsp-vbc register dump:\n");
	for (reg = VBC_MODULE_CLR0;
		reg <= VBC_CTL_ADDR_END; reg += 0x10, addr += 4) {
		snd_iprintf(buffer,
			"0x%04x | 0x%04x 0x%04x 0x%04x 0x%04x\n",
			reg - CTL_BASE_VBC, (*addr),
			*(addr + 1), *(addr + 2), *(addr + 3));
	}
	kfree(addr);

	return 0;
}

static unsigned int ap_vbc_reg_proc_read(struct snd_info_buffer *buffer)
{
	int reg;

	agdsp_access_enable();
	snd_iprintf(buffer, "ap-vbc register dump\n");
	for (reg = VBC_AUDPLY_FIFO_CTRL;
		reg <= VBC_OFLD_ADDR_END; reg += 0x10) {
		snd_iprintf(buffer, "0x%04x | 0x%04x 0x%04x 0x%04x 0x%04x\n",
			    (reg - VBC_OFLD_ADDR)
			    , ap_vbc_reg_read(reg + 0x00)
			    , ap_vbc_reg_read(reg + 0x04)
			    , ap_vbc_reg_read(reg + 0x08)
			    , ap_vbc_reg_read(reg + 0x0C)
			);
	}
	agdsp_access_disable();

	return 0;
}

static void vbc_proc_write(struct snd_info_entry *entry,
			  struct snd_info_buffer *buffer)
{
	char line[64];
	unsigned int reg, val;
	struct snd_soc_codec *codec = entry->private_data;

	agdsp_access_enable();
	while (!snd_info_get_line(buffer, line, sizeof(line))) {
		if (sscanf(line, "%x %x", &reg, &val) != 2)
			continue;
		pr_err("%s, reg:0x%x, val:0x%x\n", __func__, reg, val);
		if (val <= 0xfffffff)
			snd_soc_write(codec, reg, val);
	}
	agdsp_access_disable();
}

static void vbc_proc_read(struct snd_info_entry *entry,
			  struct snd_info_buffer *buffer)
{
	int ret = 0;

	ret = dsp_vbc_reg_shm_proc_read(buffer);
	if (ret < 0)
		snd_iprintf(buffer, "dsp-vbc register dump error\n");

	ret = ap_vbc_reg_proc_read(buffer);
	if (ret < 0)
		snd_iprintf(buffer, "ap-vbc register dump error\n");
}

static void vbc_proc_init(struct snd_soc_codec *codec)
{
	struct snd_info_entry *entry;
	struct snd_card *card = codec->component.card->snd_card;

	if (!snd_card_proc_new(card, "vbc", &entry))
		snd_info_set_text_ops(entry, codec, vbc_proc_read);
		entry->c.text.write = vbc_proc_write;
		entry->mode |= S_IWUSR;
}
#else
/* !CONFIG_PROC_FS */
static inline void vbc_proc_init(struct snd_soc_codec *codec)
{
}
#endif

static int vbc_profile_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_profile *p_profile_setting =
		&vbc_codec->vbc_profile_setting;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int profile_idx = FUN_REG(mc->reg) - SND_VBC_PROFILE_START;

	ucontrol->value.integer.value[0] =
		p_profile_setting->now_mode[profile_idx];

	return 0;
}

static int vbc_profile_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	unsigned int ret = 0;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_profile *p_profile_setting = &vbc_codec->vbc_profile_setting;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int profile_idx = FUN_REG(mc->reg) - SND_VBC_PROFILE_START;
	unsigned int mode_max_offset = 0;
	unsigned int current_offset = 0;

	mode_max_offset = p_profile_setting->hdr[profile_idx].num_mode;

	ret = ucontrol->value.integer.value[0];
	current_offset = ((ret >> 24) & 0xff);
	sp_asoc_pr_dbg("%s %s, value=%#x,",
		__func__, vbc_get_profile_name(profile_idx), ret);
	sp_asoc_pr_dbg(" current_offset=%d, mode_max_offset=%d\n",
		current_offset, mode_max_offset);

	if (current_offset < mode_max_offset)
		p_profile_setting->now_mode[profile_idx] = ret;

	if (p_profile_setting->data[profile_idx])
		vbc_profile_try_apply(codec, profile_idx);

	return ret;
}

static int vbc_profile_load_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_profile *p_profile_setting = &vbc_codec->vbc_profile_setting;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int profile_idx = FUN_REG(mc->reg) - SND_VBC_PROFILE_START;

	ucontrol->value.integer.value[0] =
		p_profile_setting->is_loading[profile_idx];

	return 0;
}

static int vbc_profile_load_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int profile_idx = FUN_REG(mc->reg) - SND_VBC_PROFILE_START;

	ret = ucontrol->value.integer.value[0];

	sp_asoc_pr_dbg("%s %s, %s\n",
			__func__, vbc_get_profile_name(profile_idx),
			(ret == 1) ? "load" : "idle");
	if (ret == 1)
		ret = vbc_profile_loading(codec, profile_idx);

	return ret;
}

static int vbc_mdg_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int id = mc->shift - VBC_MDG_START;

	ucontrol->value.integer.value[0] = vbc_codec->mdg.mute_switch[id];
	ucontrol->value.integer.value[1] = vbc_codec->mdg.mdg_step[id];

	return 0;
}

static int vbc_mdg_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val1, val2;
	int id = mc->shift - VBC_MDG_START;

	val1 = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];

	sp_asoc_pr_dbg("%s %s switch:%02d, step:%02d\n",
				__func__, vbc_get_mdg_name(id), val1, val2);
	vbc_codec->mdg.mute_switch[id] = val1;
	vbc_codec->mdg.mdg_step[id] = val2;

	vbc_try_mdg_set(vbc_codec, id);

	return 0;
}

static int vbc_dg_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int id = mc->shift - VBC_DG_START;

	ucontrol->value.integer.value[0] = vbc_codec->dg.dg_l[id];
	ucontrol->value.integer.value[1] = vbc_codec->dg.dg_r[id];

	return 0;
}

static int vbc_dg_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val1, val2;
	int id = mc->shift - VBC_DG_START;

	val1 = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];
	vbc_codec->dg.dg_l[id] = val1;
	vbc_codec->dg.dg_r[id] = val2;

	vbc_try_dg_set(vbc_codec, id);
	sp_asoc_pr_dbg("%s %s l:%02d r:%02d\n",
		__func__, vbc_get_dg_name(id), vbc_codec->dg.dg_l[id],
		vbc_codec->dg.dg_r[id]);

	return 0;
}

static int offload_dg_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int id = mc->shift - VBC_DG_START;

	ucontrol->value.integer.value[0] = vbc_codec->offload_dg.dg_l[id];
	ucontrol->value.integer.value[1] = vbc_codec->offload_dg.dg_r[id];

	return 0;
}

static int offload_dg_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val1, val2;
	int id = mc->shift - VBC_DG_START;

	val1 = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];

	if (val1 > OFFLOAD_DG_MAX || val2 > OFFLOAD_DG_MAX)
		return -EINVAL;

	sp_asoc_pr_dbg("%s %s l:%02d r:%02d\n",
			__func__, vbc_get_dg_name(id), val1, val2);
	vbc_codec->offload_dg.dg_l[id] = val1;
	vbc_codec->offload_dg.dg_r[id] = val2;

	offload_try_dg_set(vbc_codec, id);

	return 0;
}

static int vbc_smthdg_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int id = mc->shift - VBC_SMTHDG_START;

	ucontrol->value.integer.value[0] = vbc_codec->smthdg.smthdg_l[id];
	ucontrol->value.integer.value[1] = vbc_codec->smthdg.smthdg_r[id];

	return 0;
}

static int vbc_smthdg_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val1, val2;
	int id = mc->shift - VBC_SMTHDG_START;

	val1 = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];

	sp_asoc_pr_dbg("%s %s l:%02d, r:%02d\n",
			__func__, vbc_get_smthdg_name(id), val1, val2);

	vbc_codec->smthdg.smthdg_l[id] = val1;
	vbc_codec->smthdg.smthdg_r[id] = val2;

	vbc_try_smthdg_set(vbc_codec, id);

	return 0;
}

static int vbc_smthdg_step_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int id = mc->shift - VBC_SMTHDG_START;

	ucontrol->value.integer.value[0] = vbc_codec->smthdg.smthdg_step[id];

	return 0;
}

static int vbc_smthdg_step_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val;
	int id = mc->shift - VBC_SMTHDG_START;

	val = ucontrol->value.integer.value[0];
	sp_asoc_pr_dbg("%s %s %02d\n",
			__func__, vbc_get_smthdg_name(id), val);

	vbc_codec->smthdg.smthdg_step[id] = val;

	vbc_try_smthdg_step_set(vbc_codec, id);

	return 0;
}

static int vbc_mixerdg_step_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = vbc_codec->mixerdg.mixerdg_step;

	return 0;
}

static int vbc_mixerdg_step_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int val = 0;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	val = ucontrol->value.integer.value[0];
	sp_asoc_pr_dbg("%s set %02d\n", __func__, val);

	vbc_codec->mixerdg.mixerdg_step = val;

	vbc_try_mixerdg_step_set(vbc_codec);

	return 0;
}

static int vbc_mixerdg_mainpath_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int id = mc->shift - VBC_MIXERDG_START;

	ucontrol->value.integer.value[0] =
		vbc_codec->mixerdg.mixerdg_main_l[id];
	ucontrol->value.integer.value[1] =
		vbc_codec->mixerdg.mixerdg_main_r[id];

	return 0;
}

static int vbc_mixerdg_mainpath_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val1, val2;
	int id = mc->shift - VBC_MIXERDG_START;

	val1 = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];

	sp_asoc_pr_dbg("%s %s l:%02d, r:%02d\n",
			__func__, vbc_get_mixerdg_name(id), val1, val2);

	vbc_codec->mixerdg.mixerdg_main_l[id] = val1;
	vbc_codec->mixerdg.mixerdg_main_r[id] = val2;

	vbc_try_mixerdg_mainpath_set(vbc_codec, id);

	return 0;
}

static int vbc_mixerdg_mixpath_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int id = mc->shift - VBC_MIXERDG_START;

	ucontrol->value.integer.value[0] = vbc_codec->mixerdg.mixerdg_mix_l[id];
	ucontrol->value.integer.value[1] = vbc_codec->mixerdg.mixerdg_mix_r[id];

	return 0;
}

static int vbc_mixerdg_mixpath_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val1, val2;
	int id = mc->shift - VBC_MIXERDG_START;

	val1 = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];

	sp_asoc_pr_dbg("%s %s l:%02d, r:%02d\n",
			__func__, vbc_get_mixerdg_name(id), val1, val2);

	vbc_codec->mixerdg.mixerdg_mix_l[id] = val1;
	vbc_codec->mixerdg.mixerdg_mix_r[id] = val2;

	vbc_try_mixerdg_mixpath_set(vbc_codec, id);

	return 0;
}

static int vbc_mux_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_MUX_START;

	ucontrol->value.integer.value[0] = vbc_codec->vbc_mux[id];

	return 0;

}

static int vbc_mux_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_MUX_START;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	sp_asoc_pr_dbg("%s, %s to %s\n",
			__func__, vbc_get_mux_name(id),
			texts->texts[ucontrol->value.integer.value[0]]);

	vbc_codec->vbc_mux[id] = ucontrol->value.integer.value[0];
	sp_asoc_pr_dbg("%s,vbc_codec->vbc_mux[%d]=%d", __func__,
		id, vbc_codec->vbc_mux[id]);

	vbc_try_vbc_mux_set(id, vbc_codec->vbc_mux[id]);

	return 1;

}

static int vbc_adder_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_ADDER_START;

	ucontrol->value.integer.value[0] = vbc_codec->vbc_adder[id];

	return 0;

}

static int vbc_adder_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_ADDER_START;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	sp_asoc_pr_dbg("%s %s to %s\n",
			__func__, vbc_get_adder_name(id),
			texts->texts[ucontrol->value.integer.value[0]]);

	vbc_codec->vbc_adder[id] = ucontrol->value.integer.value[0];

	vbc_try_vbc_adder_set(id, vbc_codec->vbc_adder[id]);

	return 1;

}

/*src*/
static int vbc_src_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int id = mc->shift - VBC_SRC_DAC0;

	ucontrol->value.integer.value[0] = vbc_codec->vbc_src.src_fs[id];

	return 0;
}

static int vbc_src_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val1;
	int id = mc->shift - VBC_SRC_DAC0;

	val1 = ucontrol->value.integer.value[0];
	sp_asoc_pr_dbg("%s id = %d, src_fs=%d\n", __func__, id, val1);
	vbc_codec->vbc_src.src_fs[id] = val1;
	vbc_try_src_set(vbc_codec, id);

	return 0;
}

static int vbc_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = vbc_codec->volume;

	return 0;
}

static int vbc_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int value = 0;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	value = ucontrol->value.integer.value[0];
	sp_asoc_pr_dbg("%s volume = %d\n",
			__func__, value);
	vbc_codec->volume = value;
	dsp_vbc_set_volume(codec);

	return value;
}

static int vbc_reg_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = vbc_codec->dsp_reg;

	return 0;
}

static int vbc_reg_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	unsigned int reg = 0;
	unsigned int value = 0;

	struct soc_mixer_control *mc =
	(struct soc_mixer_control *)kcontrol->private_value;
	reg = mc->reg;
	value = ucontrol->value.integer.value[0];
	pr_debug("%s %#x(reg) = %#x(value)\n",
			__func__, reg, value);
	dsp_vbc_reg_write(reg, value, 0xffffffff);

	return value;
}

static int vbc_get_aud_iis_clock(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = vbc_codec->aud_iis0_master_setting;

	return 0;
}

static int vbc_put_aud_iis_clock(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	unsigned int value = 0;
	int ret = 0;
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	if (!vbc_codec->need_aud_top_clk) {
		pr_debug("%s No need audio top to provide da clock.\n",
			 __func__);
		return 0;
	}

	value = ucontrol->value.enumerated.item[0];
	sp_asoc_pr_dbg("%s, texts->texts[%d] = '%s', texts->items=%d\n",
			__func__, value, texts->texts[value], texts->items);
	if (value >= texts->items) {
		pr_err("err: %s value(%u) >= items(%u)\n",
			__func__, value, texts->items);
		return -1;
	}

	ret = vbc_enable_aud_digital_codec_iis_master(codec->component.card,
						      value);
	if (ret < 0) {
		pr_err("%s failed. value: %u ret = %d\n", __func__, value, ret);
		return -1;
	}
	vbc_codec->aud_iis0_master_setting = value;

	return value;
}

static int vbc_loopback_type_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = vbc_codec->loopback.loopback_type;

	return 0;
}

static int vbc_loopback_type_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int value = 0;
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	value = ucontrol->value.enumerated.item[0];
	sp_asoc_pr_dbg("%s, texts->texts[%d] =%s\n",
			__func__, value, texts->texts[value]);
	vbc_codec->loopback.loopback_type = value;
	dsp_vbc_loopback_set(codec);

	return value;
}

static int vbc_loopback_voice_fmt_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = vbc_codec->loopback.voice_fmt;

	return 0;
}

static int vbc_loopback_voice_fmt_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int value = 0;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	value = ucontrol->value.integer.value[0];
	sp_asoc_pr_dbg("%s, value=%d\n", __func__, value);
	vbc_codec->loopback.voice_fmt = value;
	dsp_vbc_loopback_set(codec);

	return value;
}

static int vbc_loopback_amr_rate_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = vbc_codec->loopback.amr_rate;

	return 0;
}

static int vbc_loopback_amr_rate_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int value = 0;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	value = ucontrol->value.enumerated.item[0];
	sp_asoc_pr_dbg("%s, value=%d\n", __func__, value);
	vbc_codec->loopback.amr_rate = value;
	dsp_vbc_loopback_set(codec);

	return value;
}

static int vbc_dp_en_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_DP_EN_START;
	/*read from dsp ,dsp should implement it if 0 temporary*/
	#if 0
	unsigned int dsp_reg = 0;

	dsp_reg = dsp_vbc_reg_read(VBC_DATAPATH_EN);
	pr_debug("%s, id=%d, dsp_reg %x=%hx\n", __func__, id,
		VBC_DATAPATH_EN, dsp_reg);
	vbc_codec->vbc_dp_en[id] = dsp_reg & (1 << id);
	#endif
	ucontrol->value.integer.value[0] = vbc_codec->vbc_dp_en[id];

	return 0;
}

static int vbc_dp_en_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	unsigned short value = 0;
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_DP_EN_START;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	value = ucontrol->value.enumerated.item[0];
	sp_asoc_pr_dbg("%s, id=%d,value=%d, texts->texts[] =%s\n",
			__func__, id, value, texts->texts[value]);
	vbc_codec->vbc_dp_en[id] = value;
	dsp_vbc_dp_en_set(codec, id);

	return true;
}

static int vbc_call_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_CALL_MUTE_START;

	ucontrol->value.integer.value[0] = vbc_codec->vbc_call_mute[id];

	return 0;
}

static int vbc_call_mute_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	unsigned short value = 0;
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_CALL_MUTE_START;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	value = ucontrol->value.enumerated.item[0];
	sp_asoc_pr_dbg("%s, id=%d,value=%d, texts->texts[] =%s\n",
			__func__, id, value, texts->texts[value]);
	vbc_codec->vbc_call_mute[id] = value;
	dsp_call_mute_set(codec, id);

	return true;
}

static int sys_iis_sel_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - SYS_IIS_START;

	ucontrol->value.integer.value[0] = vbc_codec->sys_iis_sel[id];

	return 0;
}

static int sys_iis_sel_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	unsigned short value = 0;
	int ret = 0;
	struct pinctrl_state *state;
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	char buf[128] = {0};
	unsigned int id = FUN_REG(e->reg) - SYS_IIS_START;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	value = ucontrol->value.enumerated.item[0];
	sp_asoc_pr_dbg("%s, id=%d,value=%d, texts->texts[] =%s\n",
			__func__, id, value, texts->texts[value]);
	vbc_codec->sys_iis_sel[id] = value;
	sprintf(buf, "%s_%u", sys_iis_sel_txt[value], id);
	state = pinctrl_lookup_state(vbc_codec->pctrl, buf);
	if (IS_ERR(state)) {
		pr_err("%s line=%d failed\n", __func__, __LINE__);
		return -EINVAL;
	}
	ret =  pinctrl_select_state(vbc_codec->pctrl, state);
	if (ret != 0)
		pr_err("%s failed ret = %d\n", __func__, ret);

	sp_asoc_pr_dbg("%s,soc iis%d -> %s\n", __func__, id, buf);

	return true;
}

static int vbc_get_ag_iis_ext_sel(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int ag_iis_num = FUN_REG(e->reg);

	ucontrol->value.integer.value[0] =
		vbc_codec->ag_iis_ext_sel[ag_iis_num];

	return 0;
}

static int vbc_put_ag_iis_ext_sel(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	unsigned short enable = 0;
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int ag_iis_num = FUN_REG(e->reg);

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	enable = ucontrol->value.enumerated.item[0];
	sp_asoc_pr_dbg("%s, ag_iis_num=%d,value=%d, texts->texts[] =%s\n",
			__func__, ag_iis_num, enable, texts->texts[enable]);
	arch_audio_iis_to_audio_top_enable(ag_iis_num, enable);

	vbc_codec->ag_iis_ext_sel[ag_iis_num] = enable;
	/* dsp_vbc_dp_en_set(codec, id); */

	return true;
}

static int vbc_get_iis_dac_width_sel(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int vbc_iis_dac_sel = FUN_REG(e->reg) - VBC_DA0;

	ucontrol->value.integer.value[0] =
		vbc_codec->dac_iis_width[vbc_iis_dac_sel];

	return 0;
}

static int vbc_put_iis_dac_width_sel(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	unsigned int value = 0;
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int vbc_iis_dac_sel = FUN_REG(e->reg) - VBC_DA0;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	value = ucontrol->value.enumerated.item[0];
	sp_asoc_pr_dbg("%s, vbc_iis_dac_sel=%d,value=%d, texts->texts[] =%s\n",
			__func__, vbc_iis_dac_sel, value, texts->texts[value]);
	vbc_codec->dac_iis_width[vbc_iis_dac_sel] = value;
	dsp_vbc_iis_dac_width_set(codec, vbc_iis_dac_sel);

	return true;
}

static int vbc_get_iis_adc_width_sel(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int vbc_iis_adc_sel = FUN_REG(e->reg) - VBC_AD0;

	ucontrol->value.integer.value[0] =
		vbc_codec->adc_iis_width[vbc_iis_adc_sel];

	return 0;
}

static int vbc_put_iis_adc_width_sel(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	unsigned int value = 0;
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int vbc_iis_adc_sel = FUN_REG(e->reg) - VBC_AD0;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	value = ucontrol->value.enumerated.item[0];
	sp_asoc_pr_dbg("%s, vbc_iis_adc_sel=%d,value=%d, texts->texts[] =%s\n",
			__func__, vbc_iis_adc_sel, value, texts->texts[value]);
	vbc_codec->adc_iis_width[vbc_iis_adc_sel] = value;
	dsp_vbc_iis_adc_width_set(codec, vbc_iis_adc_sel);

	return true;
}

static int vbc_get_iis_dac_lr_mod_sel(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int vbc_iis_dac_sel = FUN_REG(e->reg) - VBC_DA0;

	ucontrol->value.integer.value[0] =
		vbc_codec->dac_iis_lr_mod[vbc_iis_dac_sel];

	return 0;
}

static int vbc_put_iis_dac_lr_mod_sel(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	unsigned int value = 0;
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int vbc_iis_dac_sel = FUN_REG(e->reg) - VBC_DA0;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	value = ucontrol->value.enumerated.item[0];
	sp_asoc_pr_dbg("%s, vbc_iis_dac_sel=%d,value=%d, texts->texts[] =%s\n",
			__func__, vbc_iis_dac_sel, value, texts->texts[value]);
	vbc_codec->dac_iis_lr_mod[vbc_iis_dac_sel] = value;
	dsp_vbc_iis_dac_lr_mod_set(codec, vbc_iis_dac_sel);

	return true;
}

static int vbc_get_dac_mixer_left_mux_sel(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_DAC_MIXER_START;

	ucontrol->value.integer.value[0] =
		vbc_codec->mix_left_mux_sel[id].dac_mixer_mux_sel;

	return 0;
}

static int vbc_put_dac_mixer_left_mux_sel(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_DAC_MIXER_START;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	sp_asoc_pr_dbg("%s,to %s\n", __func__,
			texts->texts[ucontrol->value.integer.value[0]]);

	vbc_codec->mix_left_mux_sel[id].dac_mixer_mux_sel =
		ucontrol->value.integer.value[0];
	vbc_codec->mix_left_mux_sel[id].enable = 1;
	vbc_codec->mix_left_mux_sel[id].id = id;
	sp_asoc_pr_dbg("%s,vbc_codec->mix_left_mux_sel[%u]=%#x",
		       __func__, id,
		       vbc_codec->mix_left_mux_sel[id].dac_mixer_mux_sel);

	vbc_try_vbc_mix_left_mux_sel_set(codec, id);

	return 0;

}

static int vbc_get_dac_mixer_right_mux_sel(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_DAC_MIXER_START;

	ucontrol->value.integer.value[0] =
		vbc_codec->mix_right_mux_sel[id].dac_mixer_mux_sel;

	return 0;
}

static int vbc_put_dac_mixer_right_mux_sel(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_DAC_MIXER_START;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	sp_asoc_pr_dbg("%s,to %s\n", __func__,
			texts->texts[ucontrol->value.integer.value[0]]);

	vbc_codec->mix_right_mux_sel[id].dac_mixer_mux_sel =
		ucontrol->value.integer.value[0];
	vbc_codec->mix_right_mux_sel[id].enable = 1;
	vbc_codec->mix_right_mux_sel[id].id = id;
	sp_asoc_pr_dbg("%s,vbc_codec->mix_right_mux_sel[%u]=%#x",
		       __func__, id,
		       vbc_codec->mix_right_mux_sel[id].dac_mixer_mux_sel);

	vbc_try_vbc_mix_right_mux_sel_set(codec, id);

	return 0;

}

static int vbc_get_dac_mixer_left_out_sel(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_DAC_MIXER_START;

	ucontrol->value.integer.value[0] =
		vbc_codec->mix_left_out_sel[id].dac_mixer_out_sel;

	return 0;
}

static int vbc_put_dac_mixer_left_out_sel(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_DAC_MIXER_START;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	sp_asoc_pr_dbg("%s,to %s\n", __func__,
			texts->texts[ucontrol->value.integer.value[0]]);

	vbc_codec->mix_left_out_sel[id].dac_mixer_out_sel =
		ucontrol->value.integer.value[0];
	vbc_codec->mix_left_out_sel[id].enable = 1;
	vbc_codec->mix_left_out_sel[id].id = id;
	sp_asoc_pr_dbg("%s,vbc_codec->mix_left_out_sel[%u]=%#x",
		       __func__, id,
		       vbc_codec->mix_left_out_sel[id].dac_mixer_out_sel);

	vbc_try_vbc_mix_left_out_sel_set(codec, id);

	return 0;

}

static int vbc_get_dac_mixer_right_out_sel(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_DAC_MIXER_START;

	ucontrol->value.integer.value[0] =
		vbc_codec->mix_right_out_sel[id].dac_mixer_out_sel;

	return 0;
}

static int vbc_put_dac_mixer_right_out_sel(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int id = FUN_REG(e->reg) - VBC_DAC_MIXER_START;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	sp_asoc_pr_dbg("%s,to %s\n", __func__,
		texts->texts[ucontrol->value.integer.value[0]]);

	vbc_codec->mix_right_out_sel[id].dac_mixer_out_sel =
		ucontrol->value.integer.value[0];
	vbc_codec->mix_right_out_sel[id].enable = 1;
	vbc_codec->mix_right_out_sel[id].id = id;
	sp_asoc_pr_dbg("%s,vbc_codec->mix_right_out_sel[%u]=%#x",
		       __func__, id,
		       vbc_codec->mix_right_out_sel[id].dac_mixer_out_sel);

	vbc_try_vbc_mix_right_out_sel_set(codec, id);

	return 0;

}

static int vbc_get_iis_adc_lr_mod_sel(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int vbc_iis_adc_sel = FUN_REG(e->reg) - VBC_AD0;

	ucontrol->value.integer.value[0] =
		vbc_codec->adc_iis_lr_mod[vbc_iis_adc_sel];

	return 0;
}

static int vbc_put_iis_adc_lr_mod_sel(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	unsigned int value = 0;
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int vbc_iis_adc_sel = FUN_REG(e->reg) - VBC_AD0;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	value = ucontrol->value.enumerated.item[0];
	sp_asoc_pr_dbg("%s, vbc_iis_adc_sel=%d,value=%d, texts->texts[] =%s\n",
			__func__, vbc_iis_adc_sel, value, texts->texts[value]);
	vbc_codec->adc_iis_lr_mod[vbc_iis_adc_sel] = value;
	dsp_vbc_iis_adc_lr_mod_set(codec, vbc_iis_adc_sel);

	return true;
}

static int vbc_get_agdsp_access(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	unsigned short enable = 0;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	enable = ucontrol->value.enumerated.item[0];
	ucontrol->value.integer.value[0] = vbc_codec->agdsp_access_en;

	return true;
}
static int vbc_put_agdsp_access(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	unsigned short enable = 0;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}
	enable = ucontrol->value.enumerated.item[0];

	sp_asoc_pr_dbg("%s, cnt=%d, enable=%d",
		__func__, vbc_codec->agdsp_access_en, enable);
	if (enable) {
		if (vbc_codec->agdsp_access_en == 0)
			agdsp_access_enable();
		vbc_codec->agdsp_access_en++;
	} else {
		if (vbc_codec->agdsp_access_en)
			agdsp_access_disable();
		vbc_codec->agdsp_access_en = 0;
	}

	return true;
}

static int vbc_get_vbc_dump_flag(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = vbc_codec->open_vbc_dump_flag;

	return 0;
}

static int vbc_set_vbc_dump_flag(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	if (ucontrol->value.integer.value[0] >= texts->items ||
		ucontrol->value.integer.value[0] < 0) {
		pr_err("ERR: %s,index outof bounds error\n", __func__);
		return -EINVAL;
	}

	sp_asoc_pr_dbg("%s,to %s\n", __func__,
		texts->texts[ucontrol->value.integer.value[0]]);
	vbc_codec->open_vbc_dump_flag =
		ucontrol->value.integer.value[0];

	sp_asoc_pr_dbg("%s,vbc_codec->open_vbc_dump_flag =%#x",
		__func__, vbc_codec->open_vbc_dump_flag);

	return 0;
}

/* -9450dB to 0dB in 150dB steps ( mute instead of -9450dB) */
static const DECLARE_TLV_DB_SCALE(mdg_tlv, -9450, 150, 1);
static const DECLARE_TLV_DB_SCALE(dg_tlv, -9450, 150, 1);
static const DECLARE_TLV_DB_SCALE(smthdg_tlv, -9450, 150, 1);
static const DECLARE_TLV_DB_SCALE(smthdg_step_tlv, -9450, 150, 1);
static const DECLARE_TLV_DB_SCALE(mixerdg_tlv, -9450, 150, 1);
static const DECLARE_TLV_DB_SCALE(mixerdg_step_tlv, -9450, 150, 1);
static const DECLARE_TLV_DB_SCALE(offload_dg_tlv, 0, 150, 1);

static const struct snd_kcontrol_new vbc_codec_snd_controls[] = {
	/* vbc dump */
	SOC_ENUM_EXT("vbc_dump_enable", vbc_dump_enum,
			vbc_get_vbc_dump_flag, vbc_set_vbc_dump_flag),

	/*MIXER*/
	SOC_ENUM_EXT("vbc_dac0_mixer0_left_mux_sel",
		     vbc_mixer_left_mux_sel_enum[0],
		     vbc_get_dac_mixer_left_mux_sel,
		     vbc_put_dac_mixer_left_mux_sel),
	SOC_ENUM_EXT("vbc_dac0_mixer0_right_mux_sel",
		     vbc_mixer_right_mux_sel_enum[0],
		     vbc_get_dac_mixer_right_mux_sel,
		     vbc_put_dac_mixer_right_mux_sel),
	SOC_ENUM_EXT("vbc_dac0_mixer1_left_mux_sel",
		     vbc_mixer_left_mux_sel_enum[1],
		     vbc_get_dac_mixer_left_mux_sel,
		     vbc_put_dac_mixer_left_mux_sel),
	SOC_ENUM_EXT("vbc_dac0_mixer1_right_mux_sel",
		     vbc_mixer_right_mux_sel_enum[1],
		     vbc_get_dac_mixer_right_mux_sel,
		     vbc_put_dac_mixer_right_mux_sel),
	SOC_ENUM_EXT("vbc_dac0_mixer2_left_mux_sel",
		     vbc_mixer_left_mux_sel_enum[2],
		     vbc_get_dac_mixer_left_mux_sel,
		     vbc_put_dac_mixer_left_mux_sel),
	SOC_ENUM_EXT("vbc_dac0_mixer2_right_mux_sel",
		     vbc_mixer_right_mux_sel_enum[2],
		     vbc_get_dac_mixer_right_mux_sel,
		     vbc_put_dac_mixer_right_mux_sel),
	SOC_ENUM_EXT("vbc_dac1_mixer0_left_mux_sel",
		     vbc_mixer_left_mux_sel_enum[3],
		     vbc_get_dac_mixer_left_mux_sel,
		     vbc_put_dac_mixer_left_mux_sel),
	SOC_ENUM_EXT("vbc_dac1_mixer0_right_mux_sel",
		     vbc_mixer_right_mux_sel_enum[3],
		     vbc_get_dac_mixer_right_mux_sel,
		     vbc_put_dac_mixer_right_mux_sel),
	SOC_ENUM_EXT("vbc_dac1_mixer1_left_mux_sel",
		     vbc_mixer_left_mux_sel_enum[4],
		     vbc_get_dac_mixer_left_mux_sel,
		     vbc_put_dac_mixer_left_mux_sel),
	SOC_ENUM_EXT("vbc_dac1_mixer1_right_mux_sel",
		     vbc_mixer_right_mux_sel_enum[4],
		     vbc_get_dac_mixer_right_mux_sel,
		     vbc_put_dac_mixer_right_mux_sel),
	SOC_ENUM_EXT("vbc_dac1_mixer2_left_mux_sel",
		     vbc_mixer_left_mux_sel_enum[5],
		     vbc_get_dac_mixer_left_mux_sel,
		     vbc_put_dac_mixer_left_mux_sel),
	SOC_ENUM_EXT("vbc_dac1_mixer2_right_mux_sel",
		     vbc_mixer_right_mux_sel_enum[5],
		     vbc_get_dac_mixer_right_mux_sel,
		     vbc_put_dac_mixer_right_mux_sel),

	SOC_ENUM_EXT("vbc_dac0_mixer0_left_out_sel", vbc_mixer_out_sel_enum[0],
		     vbc_get_dac_mixer_left_out_sel,
		     vbc_put_dac_mixer_left_out_sel),
	SOC_ENUM_EXT("vbc_dac0_mixer0_right_out_sel", vbc_mixer_out_sel_enum[0],
		     vbc_get_dac_mixer_right_out_sel,
		     vbc_put_dac_mixer_right_out_sel),
	SOC_ENUM_EXT("vbc_dac0_mixer1_left_out_sel", vbc_mixer_out_sel_enum[1],
		     vbc_get_dac_mixer_left_out_sel,
		     vbc_put_dac_mixer_left_out_sel),
	SOC_ENUM_EXT("vbc_dac0_mixer1_right_out_sel", vbc_mixer_out_sel_enum[1],
		     vbc_get_dac_mixer_right_out_sel,
		     vbc_put_dac_mixer_right_out_sel),
	SOC_ENUM_EXT("vbc_dac0_mixer2_left_out_sel", vbc_mixer_out_sel_enum[2],
		     vbc_get_dac_mixer_left_out_sel,
		     vbc_put_dac_mixer_left_out_sel),
	SOC_ENUM_EXT("vbc_dac0_mixer2_right_out_sel", vbc_mixer_out_sel_enum[2],
		     vbc_get_dac_mixer_right_out_sel,
		     vbc_put_dac_mixer_right_out_sel),
	SOC_ENUM_EXT("vbc_dac1_mixer0_left_out_sel", vbc_mixer_out_sel_enum[3],
		     vbc_get_dac_mixer_left_out_sel,
		     vbc_put_dac_mixer_left_out_sel),
	SOC_ENUM_EXT("vbc_dac1_mixer0_right_out_sel", vbc_mixer_out_sel_enum[3],
		     vbc_get_dac_mixer_right_out_sel,
		     vbc_put_dac_mixer_right_out_sel),
	SOC_ENUM_EXT("vbc_dac1_mixer1_left_out_sel", vbc_mixer_out_sel_enum[4],
		     vbc_get_dac_mixer_left_out_sel,
		     vbc_put_dac_mixer_left_out_sel),
	SOC_ENUM_EXT("vbc_dac1_mixer1_right_out_sel", vbc_mixer_out_sel_enum[4],
		     vbc_get_dac_mixer_right_out_sel,
		     vbc_put_dac_mixer_right_out_sel),
	SOC_ENUM_EXT("vbc_dac1_mixer2_left_out_sel", vbc_mixer_out_sel_enum[5],
		     vbc_get_dac_mixer_left_out_sel,
		     vbc_put_dac_mixer_left_out_sel),
	SOC_ENUM_EXT("vbc_dac1_mixer2_right_out_sel", vbc_mixer_out_sel_enum[5],
		     vbc_get_dac_mixer_right_out_sel,
		     vbc_put_dac_mixer_right_out_sel),

	/* iis dac and adc iis wide sel */
	SOC_ENUM_EXT("vbc_dac0_iis_wide_sel", vbc_dac_iis_wide_sel_enum[0],
		vbc_get_iis_dac_width_sel, vbc_put_iis_dac_width_sel),
	SOC_ENUM_EXT("vbc_dac1_iis_wide_sel", vbc_dac_iis_wide_sel_enum[1],
		vbc_get_iis_dac_width_sel, vbc_put_iis_dac_width_sel),
	SOC_ENUM_EXT("vbc_adc0_iis_wide_sel", vbc_adc_iis_wide_sel_enum[0],
		vbc_get_iis_adc_width_sel, vbc_put_iis_adc_width_sel),
	SOC_ENUM_EXT("vbc_adc1_iis_wide_sel", vbc_adc_iis_wide_sel_enum[1],
		vbc_get_iis_adc_width_sel, vbc_put_iis_adc_width_sel),
	SOC_ENUM_EXT("vbc_adc2_iis_wide_sel", vbc_adc_iis_wide_sel_enum[2],
		vbc_get_iis_adc_width_sel, vbc_put_iis_adc_width_sel),
	SOC_ENUM_EXT("vbc_adc3_iis_wide_sel", vbc_adc_iis_wide_sel_enum[3],
		vbc_get_iis_adc_width_sel, vbc_put_iis_adc_width_sel),

	/*iis dac and adc iis lr mod sel*/
	SOC_ENUM_EXT("vbc_dac0_iis_lr_mod_sel", vbc_dac_iis_lr_mod_sel_enum[0],
		vbc_get_iis_dac_lr_mod_sel, vbc_put_iis_dac_lr_mod_sel),
	SOC_ENUM_EXT("vbc_dac1_iis_lr_mod_sel", vbc_dac_iis_lr_mod_sel_enum[1],
		vbc_get_iis_dac_lr_mod_sel, vbc_put_iis_dac_lr_mod_sel),
	SOC_ENUM_EXT("vbc_adc0_iis_lr_mod_sel", vbc_adc_iis_lr_mod_sel_enum[0],
		vbc_get_iis_adc_lr_mod_sel, vbc_put_iis_adc_lr_mod_sel),
	SOC_ENUM_EXT("vbc_adc1_iis_lr_mod_sel", vbc_adc_iis_lr_mod_sel_enum[1],
		vbc_get_iis_adc_lr_mod_sel, vbc_put_iis_adc_lr_mod_sel),
	SOC_ENUM_EXT("vbc_adc2_iis_lr_mod_sel", vbc_adc_iis_lr_mod_sel_enum[2],
		vbc_get_iis_adc_lr_mod_sel, vbc_put_iis_adc_lr_mod_sel),
	SOC_ENUM_EXT("vbc_adc3_iis_lr_mod_sel", vbc_adc_iis_lr_mod_sel_enum[3],
		vbc_get_iis_adc_lr_mod_sel, vbc_put_iis_adc_lr_mod_sel),

	/*ag iis ext sel*/
	SOC_ENUM_EXT("ag_iis0_ext_sel", vbc_ag_iis_ext_sel_enum[0],
		vbc_get_ag_iis_ext_sel, vbc_put_ag_iis_ext_sel),
	SOC_ENUM_EXT("ag_iis1_ext_sel", vbc_ag_iis_ext_sel_enum[1],
		vbc_get_ag_iis_ext_sel, vbc_put_ag_iis_ext_sel),
	SOC_ENUM_EXT("ag_iis2_ext_sel", vbc_ag_iis_ext_sel_enum[2],
		vbc_get_ag_iis_ext_sel, vbc_put_ag_iis_ext_sel),
	/*iis pin map*/
	SOC_ENUM_EXT("SYS_IIS0", vbc_sys_iis_enum[SYS_IIS0 - SYS_IIS_START],
		sys_iis_sel_get, sys_iis_sel_put),
	SOC_ENUM_EXT("SYS_IIS1", vbc_sys_iis_enum[SYS_IIS1 - SYS_IIS_START],
		sys_iis_sel_get, sys_iis_sel_put),
	SOC_ENUM_EXT("SYS_IIS2", vbc_sys_iis_enum[SYS_IIS2 - SYS_IIS_START],
		sys_iis_sel_get, sys_iis_sel_put),
	SOC_ENUM_EXT("SYS_IIS3", vbc_sys_iis_enum[SYS_IIS3 - SYS_IIS_START],
		sys_iis_sel_get, sys_iis_sel_put),

	SOC_ENUM_EXT("SYS_IIS4", vbc_sys_iis_enum[SYS_IIS4 - SYS_IIS_START],
		sys_iis_sel_get, sys_iis_sel_put),
	SOC_ENUM_EXT("SYS_IIS5", vbc_sys_iis_enum[SYS_IIS5 - SYS_IIS_START],
		sys_iis_sel_get, sys_iis_sel_put),
	SOC_ENUM_EXT("SYS_IIS6", vbc_sys_iis_enum[SYS_IIS6 - SYS_IIS_START],
		sys_iis_sel_get, sys_iis_sel_put),

	/*KCTL_TYPE_DATAPATH*/
	SOC_ENUM_EXT("VBC_DAC0_DP_EN",
		vbc_datapath_enum[VBC_DAC0_DP_EN - VBC_DP_EN_START],
		vbc_dp_en_get, vbc_dp_en_put),
	SOC_ENUM_EXT("VBC_DAC1_DP_EN",
		vbc_datapath_enum[VBC_DAC1_DP_EN - VBC_DP_EN_START],
		vbc_dp_en_get, vbc_dp_en_put),
	SOC_ENUM_EXT("VBC_ADC0_DP_EN",
		vbc_datapath_enum[VBC_ADC0_DP_EN - VBC_DP_EN_START],
		vbc_dp_en_get, vbc_dp_en_put),
	SOC_ENUM_EXT("VBC_ADC1_DP_EN",
		vbc_datapath_enum[VBC_ADC1_DP_EN - VBC_DP_EN_START],
		vbc_dp_en_get, vbc_dp_en_put),
	SOC_ENUM_EXT("VBC_ADC2_DP_EN",
		vbc_datapath_enum[VBC_ADC2_DP_EN - VBC_DP_EN_START],
		vbc_dp_en_get, vbc_dp_en_put),
	SOC_ENUM_EXT("VBC_ADC3_DP_EN",
		vbc_datapath_enum[VBC_ADC3_DP_EN - VBC_DP_EN_START],
		vbc_dp_en_get, vbc_dp_en_put),
	SOC_ENUM_EXT("VBC_ofld_DP_EN",
		vbc_datapath_enum[VBC_ofld_DP_EN - VBC_DP_EN_START],
		vbc_dp_en_get, vbc_dp_en_put),
	SOC_ENUM_EXT("VBC_fm_DP_EN",
		vbc_datapath_enum[VBC_fm_DP_EN - VBC_DP_EN_START],
		vbc_dp_en_get, vbc_dp_en_put),
	SOC_ENUM_EXT("VBC_st_DP_EN",
		vbc_datapath_enum[VBC_st_DP_EN - VBC_DP_EN_START],
		vbc_dp_en_get, vbc_dp_en_put),
	/* VBC BAK REG */
	/*
	 * VBC_AUDPLY_DATA_DAC0_DSP : [0] = 0
	 *	auply data to dac0 work, [0] = 1 auply data only to dsp,
	 *	so when fm use dac0, it should be configured with 1,
	 * VBC_DAC_MIX_FIX_CTL : [1] = 1,
	 *	when dac bt src enable it be configured with 1
	 */
	SOC_SINGLE_EXT("VBC_AUDPLY_DATA_DAC0_DSP", VBC_BAK_REG, 0,
		2, 0,
		vbc_reg_get, vbc_reg_put),
	 SOC_SINGLE_EXT("VBC_DAC_MIX_FIX_CTL", VBC_BAK_REG, 1,
		2, 0,
		vbc_reg_get, vbc_reg_put),
	/*CALL MUTE*/
	SOC_ENUM_EXT("VBC_UL_MUTE",
		vbc_call_mute_enum[VBC_UL_MUTE - VBC_CALL_MUTE_START],
		vbc_call_mute_get, vbc_call_mute_put),
	SOC_ENUM_EXT("VBC_DL_MUTE",
		vbc_call_mute_enum[VBC_DL_MUTE - VBC_CALL_MUTE_START],
		vbc_call_mute_get, vbc_call_mute_put),
	/*will use later if 0 temporary*/
	SOC_ENUM_EXT("VBC IIS Master Setting", iis_master_setting_enum,
		vbc_get_aud_iis_clock, vbc_put_aud_iis_clock),
	SOC_SINGLE_BOOL_EXT("agdsp_access_en", 0,
		vbc_get_agdsp_access, vbc_put_agdsp_access),
	/*vbc volume*/
	SOC_SINGLE_EXT("VBC_VOLUME", SND_SOC_NOPM, 0,
		MAX_32_BIT, 0,
		vbc_volume_get, vbc_volume_put),
	/*loop back*/
	SOC_SINGLE_EXT("VBC_DSP_LOOPBACK_ARM_RATE",
		SND_SOC_NOPM, 0,
		MAX_32_BIT, 0,
		vbc_loopback_amr_rate_get,
		vbc_loopback_amr_rate_put),
	SOC_SINGLE_EXT("VBC_DSP_LOOPBACK_VOICE_FMT", SND_SOC_NOPM, 0,
		MAX_32_BIT, 0,
		vbc_loopback_voice_fmt_get,
		vbc_loopback_voice_fmt_put),
	SOC_ENUM_EXT("VBC_DSP_LOOPBACK_TYPE",
		dsp_loopback_enum,
		vbc_loopback_type_get, vbc_loopback_type_put),
	/*src*/
	SOC_DOUBLE_R_EXT_TLV("VBC_SRC_BT",
		FUN_REG(VBC_LEFT), FUN_REG(VBC_RIGHT),
		VBC_SRC_BT, AUD_SRC_MODE_MAX - 1, 0,
		vbc_src_get,
		vbc_src_put, mixerdg_tlv),
	/* profile */
	SOC_SINGLE_EXT("Audio Structure Profile Update",
		FUN_REG(SND_VBC_PROFILE_AUDIO_STRUCTURE), 0,
		2, 0,
		vbc_profile_load_get, vbc_profile_load_put),
	SOC_SINGLE_EXT("DSP VBC Profile Update",
		FUN_REG(SND_VBC_PROFILE_DSP), 0,
		2, 0,
		vbc_profile_load_get, vbc_profile_load_put),

	SOC_SINGLE_EXT("NXP Profile Update",
		FUN_REG(SND_VBC_PROFILE_NXP), 0,
		2, 0,
		vbc_profile_load_get, vbc_profile_load_put),
	SOC_SINGLE_EXT("Audio Structure Profile Select",
		FUN_REG(SND_VBC_PROFILE_AUDIO_STRUCTURE), 0,
		VBC_PROFILE_CNT_MAX, 0,
		vbc_profile_get, vbc_profile_put),
	SOC_SINGLE_EXT("DSP VBC Profile Select",
		FUN_REG(SND_VBC_PROFILE_DSP), 0,
		VBC_PROFILE_CNT_MAX, 0,
		vbc_profile_get, vbc_profile_put),
	SOC_SINGLE_EXT("NXP Profile Select",
		FUN_REG(SND_VBC_PROFILE_NXP), 0,
		VBC_PROFILE_CNT_MAX, 0,
		vbc_profile_get, vbc_profile_put),
	/* MDG */
	SOC_DOUBLE_R_EXT_TLV("VBC DAC0 DSP MDG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_MDG_DAC0_DSP, MDG_STP_MAX_VAL, 0,
		vbc_mdg_get,
		vbc_mdg_put, mdg_tlv),
	SOC_DOUBLE_R_EXT_TLV("VBC DAC1 DSP MDG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_MDG_DAC1_DSP, MDG_STP_MAX_VAL, 0,
		vbc_mdg_get,
		vbc_mdg_put, mdg_tlv),
	SOC_DOUBLE_R_EXT_TLV("VBC DAC0 AUD MDG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_MDG_DAC0_AUD, MDG_STP_MAX_VAL, 0,
		vbc_mdg_get,
		vbc_mdg_put, mdg_tlv),
	SOC_DOUBLE_R_EXT_TLV("VBC DAC1 AUD MDG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_MDG_DAC1_AUD, MDG_STP_MAX_VAL, 0,
		vbc_mdg_get,
		vbc_mdg_put, mdg_tlv),
	/* DG */
	SOC_DOUBLE_R_EXT_TLV("VBC DAC0 DG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_DG_DAC0, DG_MAX_VAL, 0,
		vbc_dg_get,
		vbc_dg_put, dg_tlv),
	SOC_DOUBLE_R_EXT_TLV("VBC DAC1 DG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_DG_DAC1, DG_MAX_VAL, 0,
		vbc_dg_get,
		vbc_dg_put, dg_tlv),
	SOC_DOUBLE_R_EXT_TLV("VBC ADC0 DG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_DG_ADC0, DG_MAX_VAL, 0,
		vbc_dg_get,
		vbc_dg_put, dg_tlv),
	SOC_DOUBLE_R_EXT_TLV("VBC ADC1 DG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_DG_ADC1, DG_MAX_VAL, 0,
		vbc_dg_get,
		vbc_dg_put, dg_tlv),
	SOC_DOUBLE_R_EXT_TLV("VBC ADC2 DG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_DG_ADC2, DG_MAX_VAL, 0,
		vbc_dg_get,
		vbc_dg_put, dg_tlv),
	SOC_DOUBLE_R_EXT_TLV("VBC ADC3 DG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_DG_ADC3, DG_MAX_VAL, 0,
		vbc_dg_get,
		vbc_dg_put, dg_tlv),
	SOC_DOUBLE_R_EXT_TLV("VBC FM DG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_DG_FM, DG_MAX_VAL, 0,
		vbc_dg_get,
		vbc_dg_put, dg_tlv),
	SOC_DOUBLE_R_EXT_TLV("VBC ST DG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_DG_ST, DG_MAX_VAL, 0,
		vbc_dg_get,
		vbc_dg_put, dg_tlv),
	SOC_DOUBLE_R_EXT_TLV("OFFLOAD DG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), OFFLOAD_DG, OFFLOAD_DG_MAX, 0,
		offload_dg_get,
		offload_dg_put, offload_dg_tlv),
	/* SMTHDG */
	SOC_DOUBLE_R_EXT_TLV("VBC DAC0 SMTHDG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_SMTHDG_DAC0, SMTHDG_MAX_VAL, 0,
		vbc_smthdg_get,
		vbc_smthdg_put, smthdg_tlv),
	SOC_DOUBLE_R_EXT_TLV("VBC DAC1 SMTHDG Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_SMTHDG_DAC1, SMTHDG_MAX_VAL, 0,
		vbc_smthdg_get,
		vbc_smthdg_put, smthdg_tlv),
	SOC_SINGLE_EXT_TLV("VBC DAC0 SMTHDG Step Set",
		SND_SOC_NOPM,
		VBC_SMTHDG_DAC0, SMTHDG_MAX_VAL, 0,
		vbc_smthdg_step_get,
		vbc_smthdg_step_put, smthdg_step_tlv),
	SOC_SINGLE_EXT_TLV("VBC DAC1 SMTHDG Step Set",
		SND_SOC_NOPM,
		VBC_SMTHDG_DAC1, SMTHDG_MAX_VAL, 0,
		vbc_smthdg_step_get,
		vbc_smthdg_step_put, smthdg_step_tlv),
	/* MIXERDG */
	/*SND_KCTL_TYPE_MIXERDG_MAIN , 6 1*/
	SOC_DOUBLE_R_EXT_TLV("VBC MIXERDG AUDIO DAC0 Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_MIXERDG_DAC0, MIXERDG_MAX_VAL, 0,
		vbc_mixerdg_mainpath_get,
		vbc_mixerdg_mainpath_put, mixerdg_tlv),
	/*SND_KCTL_TYPE_MIXERDG_MIX , 7 1*/
	SOC_DOUBLE_R_EXT_TLV("VBC MIXERDG VOICE DAC0 Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_MIXERDG_DAC0, MIXERDG_MAX_VAL, 0,
		vbc_mixerdg_mixpath_get,
		vbc_mixerdg_mixpath_put, mixerdg_tlv),
	/*SND_KCTL_TYPE_MIXERDG_MAIN , 6 2*/
	SOC_DOUBLE_R_EXT_TLV("VBC MIXERDG VOICE DAC1 Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_MIXERDG_DAC1, MIXERDG_MAX_VAL, 0,
		vbc_mixerdg_mainpath_get,
		vbc_mixerdg_mainpath_put, mixerdg_tlv),
	/*SND_KCTL_TYPE_MIXERDG_MIX ,7 2*/
	SOC_DOUBLE_R_EXT_TLV("VBC MIXERDG AUDIO DAC1 Set",
		FUN_REG(VBC_LEFT),
		FUN_REG(VBC_RIGHT), VBC_MIXERDG_DAC1, MIXERDG_MAX_VAL, 0,
		vbc_mixerdg_mixpath_get,
		vbc_mixerdg_mixpath_put, mixerdg_tlv),
	/*SND_KCTL_TYPE_MIXERDG_STEP , 8*/
	SOC_SINGLE_EXT_TLV("VBC MIXERDG DAC STEP Set",
		SND_SOC_NOPM,
		0, MIXERDG_STP_MAX_VAL, 0,
		vbc_mixerdg_step_get,
		vbc_mixerdg_step_put, mixerdg_step_tlv),
	/* MUX */
	SOC_ENUM_EXT("VBC ADC0 SOURCE MUX",
		vbc_mux_sel_enum[VBC_MUX_ADC0_SOURCE-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC ADC0 INSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_ADC0_INSEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC ADC1 SOURCE MUX",
		vbc_mux_sel_enum[VBC_MUX_ADC1_SOURCE-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC ADC1 INSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_ADC1_INSEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC ADC2 SOURCE MUX",
		vbc_mux_sel_enum[VBC_MUX_ADC2_SOURCE-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC ADC2 INSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_ADC2_INSEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC ADC3 SOURCE MUX",
		vbc_mux_sel_enum[VBC_MUX_ADC3_SOURCE-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC ADC3 INSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_ADC3_INSEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC FM INSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_FM_INSEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC ST INSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_ST_INSEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC DAC0 ADC LOOP INSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_DAC0_ADC_INSEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC DAC1 ADC LOOP INSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_DAC1_ADC_INSEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC DAC0/DAC1 INSEL LOOP MUX",
		vbc_mux_sel_enum[VBC_MUX_DAC0_DAC1_INSEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC AUDRCD INSEL1 MUX",
		vbc_mux_sel_enum[VBC_MUX_AUDRCD_INSEL1-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC DAC0 OUTSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_DAC0_OUT_SEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC DAC1 OUTSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_DAC1_OUT_SEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC DAC0 IIS OUTSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_DAC0_IIS_SEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC DAC1 IIS OUTSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_DAC1_IIS_SEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC ADC0 IIS INSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_ADC0_IIS_SEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC ADC1 IIS INSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_ADC1_IIS_SEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC ADC2 IIS INSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_ADC2_IIS_SEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),
	SOC_ENUM_EXT("VBC ADC3 IIS INSEL MUX",
		vbc_mux_sel_enum[VBC_MUX_ADC3_IIS_SEL-VBC_MUX_START],
			vbc_mux_get, vbc_mux_put),

	/* ADDER */
	SOC_ENUM_EXT("VBC ADDER AUDPLY DAC0 MUX",
		vbc_adder_enum[VBC_ADDER_AUDPLY_DAC0-VBC_ADDER_START],
			vbc_adder_get, vbc_adder_put),
	SOC_ENUM_EXT("VBC ADDER FM DAC0 MUX",
		vbc_adder_enum[VBC_ADDER_FM_DAC0-VBC_ADDER_START],
			vbc_adder_get, vbc_adder_put),
	SOC_ENUM_EXT("VBC ADDER SMTHDG_VOICE DAC0 MUX",
		vbc_adder_enum[VBC_ADDER_SMTHDG_VOICE_DAC0-VBC_ADDER_START],
			vbc_adder_get, vbc_adder_put),
	/*only surport  0(IGNORE) 1(ADD), MINUS default action like ADD*/
	SOC_ENUM_EXT("VBC ADDER MBDRC_VOICE DAC0 MUX",
		vbc_adder_enum[VBC_ADDER_MBDRC_VOICE_DAC0-VBC_ADDER_START],
			vbc_adder_get, vbc_adder_put),
	SOC_ENUM_EXT("VBC ADDER ST DAC0 MUX",
		vbc_adder_enum[VBC_ADDER_ST_DAC0-VBC_ADDER_START],
			vbc_adder_get, vbc_adder_put),
	SOC_ENUM_EXT("VBC ADDER AUDPLY DAC1 MUX",
		vbc_adder_enum[VBC_ADDER_AUDPLY_DAC1-VBC_ADDER_START],
			vbc_adder_get, vbc_adder_put),
	SOC_ENUM_EXT("VBC ADDER FM DAC1 MUX",
		vbc_adder_enum[VBC_ADDER_FM_DAC1-VBC_ADDER_START],
			vbc_adder_get, vbc_adder_put),
	SOC_ENUM_EXT("VBC ADDER SMTHDG_VOICE DAC1 MUX",
		vbc_adder_enum[VBC_ADDER_SMTHDG_VOICE_DAC1-VBC_ADDER_START],
			vbc_adder_get, vbc_adder_put),
	/*only surport  0(IGNORE) 1(ADD), MINUS default action like ADD*/
	SOC_ENUM_EXT("VBC ADDER MBDRC_VOICE DAC1 MUX",
		vbc_adder_enum[VBC_ADDER_MBDRC_VOICE_DAC1-VBC_ADDER_START],
			vbc_adder_get, vbc_adder_put),
	SOC_ENUM_EXT("VBC ADDER ST DAC1 MUX",
		vbc_adder_enum[VBC_ADDER_ST_DAC1-VBC_ADDER_START],
			vbc_adder_get, vbc_adder_put),

};

static unsigned int vbc_codec_read(struct snd_soc_codec *codec,
				   unsigned int reg)
{
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	/* Because snd_soc_update_bits reg is 16 bits short type,
	 * so muse do following convert
	 */
	if (IS_AP_VBC_RANG(reg | AP_VBC_BASE_HI)) {
		reg |= AP_VBC_BASE_HI;
		return ap_vbc_reg_read(reg);
	} else if (IS_DSP_VBC_RANG(reg | DSP_VBC_BASE_HI)) {
		reg |= DSP_VBC_BASE_HI;
		return dsp_vbc_reg_read(reg);
	} else if (IS_SPRD_VBC_PROFILE_RANG(FUN_REG(reg))) {
		/*do nothing*/
	} else if (IS_SPRD_VBC_MDG_RANG(FUN_REG(reg))) {
		/*do nothing*/
	} else if (SPRD_VBC_DG_IDX(FUN_REG(reg))) {
		/*do nothing*/
	} else if (SPRD_VBC_SMTHDG_IDX(FUN_REG(reg))) {
		/*do nothing*/
	} else if (SPRD_VBC_MIXERDG_IDX(FUN_REG(reg))) {
		/*do nothing*/
	} else if (IS_SPRD_VBC_MUX_RANG(FUN_REG(reg))) {
		int id = SPRD_VBC_MUX_IDX(FUN_REG(reg));

		return vbc_codec->vbc_mux[id];
	} else if (IS_SPRD_VBC_ADDER_MUX_RANG(FUN_REG(reg))) {
		int id = SPRD_VBC_ADDER_IDX(FUN_REG(reg));

		return vbc_codec->vbc_adder[id];
	}
	sp_asoc_pr_dbg("%s, The Register is NOT VBC",
		__func__);
	sp_asoc_pr_dbg(" Codec's reg = 0x%x\n",
		reg);

	return 0;
}

static int vbc_codec_write(struct snd_soc_codec *codec, unsigned int reg,
			unsigned int val)
{
	if (IS_AP_VBC_RANG(reg | AP_VBC_BASE_HI)) {
		reg |= AP_VBC_BASE_HI;

		return ap_vbc_reg_write(reg, val);
	} else if (IS_DSP_VBC_RANG(reg | DSP_VBC_BASE_HI)) {
		reg |= DSP_VBC_BASE_HI;

		return dsp_vbc_reg_write(reg, val, 0xffffffff);
	}
	sp_asoc_pr_reg("%s, The Register is NOT VBC", __func__);
	sp_asoc_pr_reg(" Codec's reg = 0x%x\n", reg);

	return 0;
}

static int vbc_codec_soc_probe(struct snd_soc_codec *codec)
{
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_profile *vbc_profile_setting =
		&vbc_codec->vbc_profile_setting;
	struct snd_soc_card *card = codec->component.card;
	struct sprd_card_data *mdata = snd_soc_card_get_drvdata(card);
	int ret = 0;
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	sp_asoc_pr_dbg("%s\n", __func__);

	if (mdata->codec_type) /* external */
		vbc_phy_dt_parse_dig_codec_reg(codec->dev);

	dapm->idle_bias_off = 1;

	vbc_codec->codec = codec;
	vbc_profile_setting->codec = codec;

	vbc_proc_init(codec);

	return ret;
}

/* power down chip */
static int vbc_codec_soc_remove(struct snd_soc_codec *codec)
{
	return 0;
}

struct snd_soc_codec_driver sprd_vbc_codec = {
	.probe = vbc_codec_soc_probe,
	.remove = vbc_codec_soc_remove,
	.read = vbc_codec_read,
	.write = vbc_codec_write,
	.reg_word_size = sizeof(u16),
	.reg_cache_step = 2,
	.controls = vbc_codec_snd_controls,
	.num_controls = ARRAY_SIZE(vbc_codec_snd_controls),
};
EXPORT_SYMBOL(sprd_vbc_codec);

int sprd_vbc_codec_probe(struct platform_device *pdev)
{
	struct vbc_codec_priv *vbc_codec;
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;
	struct regmap *agcp_ahb_gpr;

	sp_asoc_pr_dbg("%s\n", __func__);

	vbc_codec = devm_kzalloc(&pdev->dev, sizeof(struct vbc_codec_priv),
				 GFP_KERNEL);
	if (vbc_codec == NULL)
		return -ENOMEM;
	platform_set_drvdata(pdev, vbc_codec);

	/* Prepare for global registers accessing. */
	agcp_ahb_gpr = syscon_regmap_lookup_by_phandle(
		np, "sprd,syscon-agcp-ahb");
	if (IS_ERR(agcp_ahb_gpr)) {
		pr_err("ERR: [%s] Get the agcp ahb syscon failed!\n",
			__func__);
		agcp_ahb_gpr = NULL;
		return -EPROBE_DEFER;
	}
	arch_audio_set_agcp_ahb_gpr(agcp_ahb_gpr);

	/* If need internal codec(audio top) to provide clock for
	 * vbc da path.
	 */
	vbc_codec->need_aud_top_clk =
		of_property_read_bool(np, "sprd,need-aud-top-clk");

	/* AP refer to array vbc_switch_reg_val */
	vbc_codec->vbc_profile_setting.dev = &pdev->dev;
	vbc_codec->mdg.mdg_set = dsp_vbc_mdg_set;
	vbc_codec->dg.dg_set = dsp_vbc_dg_set;
	vbc_codec->offload_dg.dg_set = dsp_offload_dg_set;
	vbc_codec->smthdg.smthdg_set = dsp_vbc_smthdg_set;
	vbc_codec->smthdg.smthdg_step_set = dsp_vbc_smthdg_step_set;
	vbc_codec->mixerdg.mixerdg_mainpath_set = dsp_vbc_mixerdg_mainpath_set;
	vbc_codec->mixerdg.mixerdg_mixpath_set = dsp_vbc_mixerdg_mixpath_set;
	vbc_codec->mixerdg.mixerdg_step_set = dsp_vbc_mixerdg_step_set;
	vbc_codec->vbc_profile_setting.vbc_profile_apply = vbc_profile_set;
	vbc_codec->vbc_src.src_set = dsp_vbc_src_set;
	mutex_init(&vbc_codec->load_mutex);

	return ret;
}

int sprd_vbc_codec_remove(struct platform_device *pdev)
{
	struct vbc_codec_priv *vbc_codec = platform_get_drvdata(pdev);

	vbc_codec->vbc_profile_setting.dev = 0;

	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

MODULE_DESCRIPTION("SPRD ASoC VBC Codec Driver");
MODULE_AUTHOR("Jian chen <jian.chen@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("codec:VBC Codec");
