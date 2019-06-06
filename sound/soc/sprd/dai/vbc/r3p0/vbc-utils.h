/*
 * include/sound/vbc-utils.h
 *
 * SPRD SoC VBC -- SpreadTrum SOC utils function for VBC DAI.
 *
 * Copyright (C) 2015 SpreadTrum Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY ork FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __VBC_UTILS_H
#define __VBC_UTILS_H
#include <audio-smsg.h>
#include <linux/device.h>
#include <linux/types.h>
#include <mcdt_phy_v0.h>

#define SPRD_VBC_VERSION	"vbc.r3p0"
/* store vbc pathset configuration for dsp */
#define VBC_PATHSET_CONFIG_MAX 30
/*mcdt channel 5 6 7 has been used by modem*/

/*voip*/
#define MCDT_CHAN_VOIP MCDT_CHAN1
#define MCDT_AP_DMA_CHAN_VOIP MCDT_AP_DMA_CH1

#define MCDT_FULL_WMK_VOIP (160)
#define MCDT_EMPTY_WMK_VOIP (320)
#define MCDT_VOIP_P_FRAGMENT (160)
#define MCDT_VOIP_C_FRAGMENT (160)

/*voice capture*/
#define MCDT_CHAN_VOICE_CAPTURE MCDT_CHAN2
#define MCDT_AP_DMA_CHAN_VOICE_CAPTURE MCDT_AP_DMA_CH2

#define MCDT_FULL_WMK_VOICE_CAPTURE 160
#define MCDT_VOICE_C_FRAGMENT 160

/*loop back (mcdt)*/
#define MCDT_CHAN_LOOP MCDT_CHAN3
#define MCDT_AP_DMA_CHAN_LOOP MCDT_AP_DMA_CH3

#define MCDT_LOOP_P_FRAGMENT 160
#define MCDT_LOOP_C_FRAGMENT 160
#define MCDT_FULL_WMK_LOOP 160
#define MCDT_EMPTY_WMK_LOOP 160

/*fast play*/
#define MCDT_CHAN_FAST_PLAY MCDT_CHAN4
#define MCDT_AP_DMA_CHAN_FAST_PLAY MCDT_AP_DMA_CH4

#define MCDT_EMPTY_WMK_FAST_PLAY 320
#define MCDT_FAST_PLAY_FRAGMENT 160

enum AUD_AP_SRC_MODE_E {
	AUD_AP_SRC_MODE_48000,
	AUD_AP_SRC_MODE_44100,
	AUD_AP_SRC_MODE_32000,
	AUD_AP_SRC_MODE_24000,
	AUD_AP_SRC_MODE_22050,
	AUD_AP_SRC_MODE_16000,
	AUD_AP_SRC_MODE_12000,
	AUD_AP_SRC_MODE_11025,
	AUD_AP_SRC_MODE_NA,
	AUD_AP_SRC_MODE_8000
};

enum VBC_DA_ID_E {
	VBC_DA0,
	VBC_DA1,
	VBC_DA_MAX,
};

enum VBC_AD_ID_E {
	VBC_AD0,
	VBC_AD1,
	VBC_AD2,
	VBC_AD3,
	VBC_AD_MAX,
};

enum SND_PCM_DIRECTION_E {
	SND_PCM_PLAYBACK = 0,
	SND_PCM_CAPTURE,
	SND_PCM_DIRECTION_MAX
};

/*src*/
enum VBC_SRC_MODE_E {
	AUD_SRC_MODE_48000 = 0,
	AUD_SRC_MODE_44100,
	AUD_SRC_MODE_32000,
	AUD_SRC_MODE_24000,
	AUD_SRC_MODE_22050,
	AUD_SRC_MODE_16000,
	AUD_SRC_MODE_12000,
	AUD_SRC_MODE_11025,
	AUD_SRC_MODE_NA,
	AUD_SRC_MODE_8000,
	AUD_SRC_MODE_MAX,
};

enum VBC_LOOPBACK_TYPE_E {
	VBC_LOOPBACK_ADDA,
	/*echo cancellation, noise cancellation etc*/
	VBC_LOOPBACK_AD_ULDL_DA_PROCESS,
	VBC_LOOPBACK_AD_UL_ENCODE_DECODE_DL_DA_PROCESS,
	VBC_LOOPBACK_TYPE_MAX,
};

enum {
	SND_KCTL_TYPE_REG = 0,
	SND_KCTL_TYPE_MDG,
	SND_KCTL_TYPE_SRC,
	SND_KCTL_TYPE_DG,
	SND_KCTL_TYPE_SMTHDG,
	SND_KCTL_TYPE_SMTHDG_STEP,
	SND_KCTL_TYPE_MIXERDG_MAIN,
	SND_KCTL_TYPE_MIXERDG_MIX,
	SND_KCTL_TYPE_MIXERDG_STEP,
	SND_KCTL_TYPE_MIXER,
	SND_KCTL_TYPE_MUX,
	SND_KCTL_TYPE_VOLUME,
	SND_KCTL_TYPE_ADDER,
	SND_KCTL_TYPE_LOOPBACK_TYPE,
	SND_KCTL_TYPE_DATAPATH,
	/*smsg_prt->parameter1 = 1 ul mute,smsg_prt->parameter1 = 0 ul unmute*/
	SND_KCTL_TYPE_UL_MUTE,
	/*smsg_prt->parameter1 = 1 dl mute,smsg_prt->parameter1 = 0 dl unmute*/
	SND_KCTL_TYPE_DL_MUTE,
	SND_KCTL_TYPE_DAC_IIS_WIDTH_SEL,
	SND_KCTL_TYPE_ADC_IIS_WIDTH_SEL,
	SND_KCTL_TYPE_DAC_IIS_LR_MOD_SEL,
	SND_KCTL_TYPE_ADC_IIS_LR_MOD_SEL,
	SND_KCTL_TYPE_DAC_MIXER_L_MUX_SEL,
	SND_KCTL_TYPE_DAC_MIXER_R_MUX_SEL,
	/*0 original data out, 1 invert original data +1*/
	SND_KCTL_TYPE_DAC_MIXER_L_OUT_SEL,
	SND_KCTL_TYPE_DAC_MIXER_R_OUT_SEL,
	SND_KCTL_TYPE_END,
};

enum {
	VBC_LEFT = 0,
	VBC_RIGHT = 1,
	VBC_ALL_CHAN = 2,
	VBC_CHAN_MAX
};

enum VBC_DAT_FORMAT {
	VBC_DAT_H24 = 0,
	VBC_DAT_L24,
	VBC_DAT_H16,
	VBC_DAT_L16
};

enum VBC_MDG_ID_E {
	VBC_MDG_START = 3,
	VBC_MDG_DAC0_DSP = VBC_MDG_START,
	VBC_MDG_DAC1_DSP,
	VBC_MDG_DAC0_AUD,
	/*not realized, reserved for feature*/
	VBC_MDG_DAC1_AUD,
	VBC_MDG_MAX
};

enum VBC_SRC_ID_E {
	VBC_SRC_DAC0 = 0,
	/*not open*/
	VBC_SRC_DAC1,
	VBC_SRC_ADC0,
	VBC_SRC_ADC1,
	VBC_SRC_ADC2,
	VBC_SRC_ADC3,
	VBC_SRC_BT,
	VBC_SRC_FM,
	VBC_SRC_AUDPLY,
	VBC_SRC_AUDRCD,
	VBC_SRC_MAX
};

enum VBC_DG_ID_E {
	VBC_DG_START = VBC_MDG_MAX,
	VBC_DG_DAC0 = VBC_DG_START,
	VBC_DG_DAC1,
	VBC_DG_ADC0,
	VBC_DG_ADC1,
	VBC_DG_ADC2,
	VBC_DG_ADC3,
	VBC_DG_FM,
	VBC_DG_ST,
	OFFLOAD_DG,
	VBC_DG_MAX
};

enum VBC_SMTHDG_ID_E {
	VBC_SMTHDG_START = VBC_DG_MAX,
	VBC_SMTHDG_DAC0 = VBC_SMTHDG_START,
	VBC_SMTHDG_DAC1,
	VBC_SMTHDG_MAX
};

enum VBC_MIXERDG_ID_E {
	VBC_MIXERDG_START = VBC_SMTHDG_MAX,
	/*dac0(audio && voice)*/
	VBC_MIXERDG_DAC0 = VBC_MIXERDG_START,
	/*dac1(voice && audio)*/
	VBC_MIXERDG_DAC1,
	VBC_MIXERDG_MAX
};

enum VBC_MIXER_INDEX_E {
	VBC_MIXER0_DAC0 = 0,
	VBC_MIXER1_DAC0,
	VBC_MIXER2_DAC0,
	VBC_MIXER0_DAC1,
	VBC_MIXER1_DAC1,
	VBC_MIXER2_DAC1,
	VBC_MIXER_ST,
	VBC_MIXER_FM,
	VBC_MIXER_MAX
};

/*vbc mux setting*/
enum VBC_MUX_ID_E {
	VBC_MUX_START = VBC_MIXERDG_MAX,
	/*adc0 mux: iis0 or vbc_if*/
	VBC_MUX_ADC0_SOURCE = VBC_MUX_START,
	VBC_MUX_ADC0_INSEL,
	/*adc1 mux*/
	VBC_MUX_ADC1_SOURCE,
	VBC_MUX_ADC1_INSEL,
	/*adc2 mux*/
	VBC_MUX_ADC2_SOURCE,
	VBC_MUX_ADC2_INSEL,
	/*adc3 mux*/
	VBC_MUX_ADC3_SOURCE,
	VBC_MUX_ADC3_INSEL,
	/*fm mux*/
	VBC_MUX_FM_INSEL,
	/*st mux*/
	VBC_MUX_ST_INSEL,
	/*loopback mux*/
	VBC_MUX_DAC0_ADC_INSEL,
	VBC_MUX_DAC1_ADC_INSEL,
	VBC_MUX_DAC0_DAC1_INSEL,
	/*record without dsp*/
	VBC_MUX_AUDRCD_INSEL1,
	/*0:iis; 1:vbc_if;*/
	VBC_MUX_DAC0_OUT_SEL,
	/*0:iis; 1:vbc_if;*/
	VBC_MUX_DAC1_OUT_SEL,
	/*0:iis0; 1:iis1; 2:iis2; 3:iis3;*/
	VBC_MUX_DAC0_IIS_SEL,
	/*0:iis0; 1:iis1; 2:iis2; 3:iis3;*/
	VBC_MUX_DAC1_IIS_SEL,
	/*0:iis0; 1:iis1; 2:iis2; 3:iis3;*/
	VBC_MUX_ADC0_IIS_SEL,
	/*0:iis0; 1:iis1; 2:iis2; 3:iis3;*/
	VBC_MUX_ADC1_IIS_SEL,
	/*0:iis0; 1:iis1; 2:iis2; 3:iis3;*/
	VBC_MUX_ADC2_IIS_SEL,
	/*0:iis0; 1:iis1; 2:iis2; 3:iis3;*/
	VBC_MUX_ADC3_IIS_SEL,
	VBC_MUX_MAX
};

enum VBC_MIXER_E {
	VBC_DAC_MIXER_START = 0,
	VBC_DAC0_MIXER0 = VBC_DAC_MIXER_START,
	VBC_DAC0_MIXER1,
	VBC_DAC0_MIXER2,
	VBC_DAC1_MIXER0,
	VBC_DAC1_MIXER1,
	VBC_DAC1_MIXER2,
	VBC_DAC_MIXER_MAX,
};

enum VBC_ADDER_ID_E {
	VBC_ADDER_START = VBC_MUX_MAX,
	VBC_ADDER_AUDPLY_DAC0 = VBC_ADDER_START,
	VBC_ADDER_FM_DAC0,
	VBC_ADDER_SMTHDG_VOICE_DAC0,
	VBC_ADDER_MBDRC_VOICE_DAC0,
	VBC_ADDER_ST_DAC0,
	VBC_ADDER_AUDPLY_DAC1,
	VBC_ADDER_FM_DAC1,
	VBC_ADDER_SMTHDG_VOICE_DAC1,
	VBC_ADDER_MBDRC_VOICE_DAC1,
	VBC_ADDER_ST_DAC1,
	VBC_ADDER_MAX
};

enum VBC_ADDER_MODE_E {
	VBC_ADDER_MODE_IGNORE = 0,
	VBC_ADDER_MODE_ADD,
	VBC_ADDER_MODE_MINUS,
	VBC_ADDER_MODE_MAX,
};

enum VBC_DATAPATH_ID_E {
	VBC_DP_EN_START = 0,
	VBC_DAC0_DP_EN = VBC_DP_EN_START,
	VBC_DAC1_DP_EN,
	VBC_ADC0_DP_EN,
	VBC_ADC1_DP_EN,
	VBC_ADC2_DP_EN,
	VBC_ADC3_DP_EN,
	VBC_ofld_DP_EN,
	VBC_fm_DP_EN,
	VBC_st_DP_EN,
	VBC_DP_EN_MAX,
};

#define VBC_MODULE_CLR_ADC0_IIS_AFIFO 28

enum VBC_CALL_MUTE_E {
	VBC_CALL_MUTE_START = 0,
	VBC_UL_MUTE = VBC_CALL_MUTE_START,
	VBC_DL_MUTE,
	VBC_CALL_MUTE_MAX,
};

enum SND_VBC_PROFILE_USE_E {
	SND_VBC_PROFILE_START = 0,
	SND_VBC_PROFILE_AUDIO_STRUCTURE = SND_VBC_PROFILE_START,
	/*vbc eq*/
	SND_VBC_PROFILE_DSP,
	SND_VBC_PROFILE_NXP,
	SND_VBC_PROFILE_MAX
};

enum SND_VBC_SHM_USE_E {
	SND_VBC_SHM_START = SND_VBC_PROFILE_MAX,
	SND_VBC_SHM_VBC_REG = SND_VBC_SHM_START,
	SND_VBC_SHM_MAX
};

struct sprd_vbc_kcontrol {
	unsigned int reg;
	unsigned int mask;
	unsigned int value;
};

struct vbc_mute_dg_para {
	enum VBC_MDG_ID_E mdg_id;
	bool mdg_mute;
	int16_t mdg_step;
};

struct vbc_loopback_para {
	enum VBC_LOOPBACK_TYPE_E loopback_type;
	int voice_fmt;
	int amr_rate;
};

struct vbc_src_para {
	enum VBC_SRC_ID_E src_id;
	int32_t fs;
};

struct vbc_bt_call_para {
	int enable;
	enum VBC_SRC_MODE_E src_mode;
};

struct vbc_dg_para {
	enum VBC_DG_ID_E dg_id;
	int16_t dg_left;
	int16_t dg_right;
};

struct vbc_smthdg_para {
	enum VBC_SMTHDG_ID_E smthdg_id;
	int16_t smthdg_left;
	int16_t smthdg_right;
};

struct vbc_smthdg_step_para {
	enum VBC_SMTHDG_ID_E smthdg_id;
	int16_t smthdg_step;
};

struct vbc_mixerdg_mainpath_para {
	enum VBC_MIXERDG_ID_E mixerdg_id;
	int16_t mixerdg_main_left;
	int16_t mixerdg_main_right;
};

struct vbc_mixerdg_mixpath_para {
	enum VBC_MIXERDG_ID_E mixerdg_id;
	int16_t mixerdg_mix_left;
	int16_t mixerdg_mix_right;
};

struct vbc_mixer_para {
	enum VBC_MIXER_INDEX_E mixer_id;
	int16_t mixer_mux_left;
	int16_t mixer_mux_right;
	int16_t mixer_out_left;
	int16_t mixer_out_right;
};

struct vbc_mux_para {
	enum VBC_MUX_ID_E mux_id;
	int16_t mux_sel;
};

struct vbc_adder_para {
	enum VBC_ADDER_ID_E adder_id;
	enum VBC_ADDER_MODE_E adder_mode_l;
	enum VBC_ADDER_MODE_E adder_mode_r;
};

struct sprd_vbc_priv {
	int adc_used_chan_count;
	int dac_used_chan_count;
	enum VBC_DA_ID_E dac_id;
	enum VBC_AD_ID_E adc_id;
	enum VBC_MUX_ID_E out_sel;
	enum VBC_MUX_ID_E adc_source_sel;
	enum VBC_MUX_ID_E dac_iis_port;
	enum VBC_MUX_ID_E adc_iis_port;
	struct sprd_pcm_dma_params *dma_params[SND_PCM_DIRECTION_MAX];
};

struct aud_pm_vbc {
	int is_startup;
	int other_case_cnt;
	int is_access_send;
	/* prot is_startup */
	struct mutex pm_mtx_cmd_prot;
	/* prot other_case_cnt */
	struct mutex pm_mtx_cnt;
	/* prot is_pm_shutdown */
	spinlock_t pm_spin_cmd_prot;
};

struct vbc_dp_en_para {
	int id;
	unsigned short enable;
};

struct vbc_iis_dac_width_sel_para {
	int32_t dac_id;
	int32_t dac_iis_width;
};

struct vbc_iis_adc_width_sel_para {
	int32_t adc_id;
	int32_t adc_iis_width;
};

struct vbc_iis_dac_lr_mod_sel_para {
	int32_t dac_id;
	int32_t dac_iis_lr_mod;
};

struct vbc_iis_adc_lr_mod_sel_para {
	int32_t adc_id;
	int32_t adc_iis_lr_mod;
};

struct vbc_dac_mixer_mux_sel {
	uint32_t enable;
	/* VBC_MIXER_E */
	uint32_t id;
	uint32_t dac_mixer_mux_sel;
};

struct vbc_dac_mixer_out_sel {
	uint32_t enable;
	/* VBC_MIXER_E */
	uint32_t id;
	uint32_t dac_mixer_out_sel;
};

struct snd_pcm_statup_paras {
	enum VBC_DA_ID_E dac_id;
	enum VBC_AD_ID_E adc_id;
	/* iis or vbc_if */
	int32_t dac_out_sel;
	/* iis or vbc_if */
	int32_t adc_source_sel;
	/* iis0-iis3 */
	int32_t dac_iis_port;
	/* iis0-iis3 */
	int32_t adc_iis_port;
	/* dac0 datapath iis selelction
	 * while dac1 datapath enable
	 */
	int32_t dac0_iis_port_for_voice;
	/* 0 16bit, 1 24bit */
	int32_t dac_iis_width;
	int32_t adc_iis_width;
	/* 0 left channel from high level */
	int32_t dac_iis_lr_mod;
	int32_t adc_iis_lr_mod;
	struct vbc_loopback_para loopback;
	struct vbc_dac_mixer_out_sel mix_left_out_sel[VBC_DAC_MIXER_MAX];
	struct vbc_dac_mixer_mux_sel mix_left_mux_sel[VBC_DAC_MIXER_MAX];
	struct vbc_dac_mixer_out_sel mix_right_out_sel[VBC_DAC_MIXER_MAX];
	struct vbc_dac_mixer_mux_sel mix_right_mux_sel[VBC_DAC_MIXER_MAX];
};

const char *vbc_get_adder_name(int adder_id);
const char *vbc_get_mux_name(int mux_id);
const char *vbc_get_chan_name(int chan_id);
const char *vbc_get_mixerdg_name(int mixerdg_id);
const char *vbc_get_smthdg_name(int smthdg_id);
const char *vbc_get_dg_name(int dg_id);
const char *vbc_get_mdg_name(int mdg_id);
const char *vbc_get_profile_name(int profile_id);
const char *vbc_get_stream_name(int stream);
extern unsigned int sprd_ap_vbc_phy_base;
extern struct snd_soc_codec_driver sprd_vbc_codec;

#endif /* __VBC_UTILS_H */
