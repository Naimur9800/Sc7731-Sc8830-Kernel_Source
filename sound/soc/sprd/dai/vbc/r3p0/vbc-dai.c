/*
 * sound/soc/sprd/dai/vbc/r3p0/vbc-dai.c
 *
 * SPRD SoC VBC -- SpreadTrum SOC for VBC DAI function.
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

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include "mcdt_hw.h"
#include "sprd-asoc-common.h"
#include "sprd-dmaengine-pcm.h"
#include "vbc-dai.h"

static struct aud_pm_vbc *pm_vbc;
static struct aud_pm_vbc *aud_pm_vbc_get(void);
#define DEFAULT_WATERMARK 160
#define DEFAULT_RATE 48000

static struct sprd_pcm_dma_params vbc_pcm_normal_p_outdsp = {
	.name = "VBC PCM Normal P Out DSP",
	.irq_type = BLK_DONE,
	.desc = {
		.datawidth = 1,
		.fragmens_len = VB_AUDPLY_FRAGMENT,
		/* src_step, dst_step:sprd-dmaengine-pcm.c */
	},
};

static struct sprd_pcm_dma_params vbc_pcm_normal_c_outdsp = {
	.name = "VBC PCM Normal C Out DSP",
	.irq_type = BLK_DONE,
	.desc = {
		.datawidth = 1,
		.fragmens_len = VB_AUDRCD_FRAGMENT,
	},
};

static struct sprd_pcm_dma_params pcm_fast_play_mcdt = {
	.name = "VBC PCM Fast P",
	.irq_type = BLK_DONE,
	.desc = {
		.datawidth = WORD_WIDTH,
		.fragmens_len = MCDT_FAST_PLAY_FRAGMENT,
		.src_step = 4,
		.des_step = 0,
		},
};

static struct sprd_pcm_dma_params
	vbc_pcm_voice_capture_mcdt = {
	.name = "VBC PCM voice C With MCDT",
	.irq_type = BLK_DONE,
	.desc = {
		.datawidth = WORD_WIDTH,
		.fragmens_len = MCDT_VOICE_C_FRAGMENT,
		/*ignore, config in sprd-dmaengine-pcm.c*/
		.src_step = 0,
		/*ignore, config in sprd-dmaengine-pcm.c*/
		.des_step = 4,
	},
};

static struct sprd_pcm_dma_params pcm_loop_record_mcdt = {
	.name = "PCM loop record With MCDT",
	.irq_type = BLK_DONE,
	.desc = {
		.datawidth = WORD_WIDTH,
		.fragmens_len = MCDT_LOOP_C_FRAGMENT,
		.src_step = 0,
		.des_step = 4,
	},
};

static struct sprd_pcm_dma_params pcm_loop_play_mcdt = {
	.name = "PCM loop play With MCDT",
	.irq_type = BLK_DONE,
	.desc = {
		.datawidth = WORD_WIDTH,
		.fragmens_len = MCDT_LOOP_P_FRAGMENT,
		.src_step = 0,
		.des_step = 4,
	},
};

/*voip*/
static struct sprd_pcm_dma_params pcm_voip_record_mcdt = {
	.name = "PCM voip record With MCDT",
	.irq_type = BLK_DONE,
	.desc = {
		.datawidth = WORD_WIDTH,
		.fragmens_len = MCDT_VOIP_C_FRAGMENT,
		.src_step = 0,
		.des_step = 4,
	},
};

static struct sprd_pcm_dma_params pcm_voip_play_mcdt = {
	.name = "PCM voip play With MCDT",
	.irq_type = BLK_DONE,
	.desc = {
		.datawidth = WORD_WIDTH,
		.fragmens_len = MCDT_VOIP_P_FRAGMENT,
		.src_step = 0,
		.des_step = 4,
	},
};

/*fm_caputre*/
static struct sprd_pcm_dma_params vbc_pcm_fm_caputre = {
	.name = "VBC PCM fm capture",
	.irq_type = BLK_DONE,
	.desc = {
		.datawidth = 1,
		.fragmens_len = VB_AUDRCD_FRAGMENT,
		},
};

/* bt_caputre */
static struct sprd_pcm_dma_params vbc_pcm_bt_caputre = {
	.name = "VBC PCM bt capture",
	.irq_type = BLK_DONE,
	.desc = {
		.datawidth = SHORT_WIDTH,
		.fragmens_len = (VB_AUDRCD_FRAGMENT),
	},
};

/*index=dai_id*/
struct sprd_vbc_priv g_vbc[VBC_DAI_ID_MAX] = {
	/* normal outdsp */
	[VBC_DAI_ID_NORMAL_OUTDSP] = {
		.dac_id = VBC_DA0,
		.adc_id = VBC_AD0,
		.out_sel = VBC_MUX_DAC0_OUT_SEL-VBC_MUX_START,
		.adc_source_sel = VBC_MUX_ADC0_SOURCE-VBC_MUX_START,
		.dac_iis_port = VBC_MUX_DAC0_IIS_SEL-VBC_MUX_START,
		.adc_iis_port = VBC_MUX_ADC0_IIS_SEL-VBC_MUX_START,
		.dma_params[SND_PCM_PLAYBACK] = &vbc_pcm_normal_p_outdsp,
		.dma_params[SND_PCM_CAPTURE] = &vbc_pcm_normal_c_outdsp,
	},
	/* fast play */
	[VBC_DAI_ID_FAST_P] = {
		.dac_id = VBC_DA0,
		.out_sel = VBC_MUX_DAC0_OUT_SEL-VBC_MUX_START,
		.dac_iis_port = VBC_MUX_DAC0_IIS_SEL-VBC_MUX_START,
		.dma_params[SND_PCM_PLAYBACK] = &pcm_fast_play_mcdt,
	},
	/* offload */
	[VBC_DAI_ID_OFFLOAD] = {
		.dac_id = VBC_DA0,
		.adc_id = VBC_AD1,
		.adc_source_sel = VBC_MUX_ADC1_SOURCE-VBC_MUX_START,
		.out_sel = VBC_MUX_DAC0_OUT_SEL-VBC_MUX_START,
		.dac_iis_port = VBC_MUX_DAC0_IIS_SEL-VBC_MUX_START,
		.adc_iis_port = VBC_MUX_ADC1_IIS_SEL-VBC_MUX_START,
	},
	/* voice */
	[VBC_DAI_ID_VOICE] = {
		.dac_id = VBC_DA1,
		.adc_id = VBC_AD2,
		.out_sel = VBC_MUX_DAC1_OUT_SEL-VBC_MUX_START,
		.adc_source_sel = VBC_MUX_ADC2_SOURCE-VBC_MUX_START,
		.dac_iis_port = VBC_MUX_DAC1_IIS_SEL-VBC_MUX_START,
		.adc_iis_port = VBC_MUX_ADC2_IIS_SEL-VBC_MUX_START,
	},
	/* voip record */
	[VBC_DAI_ID_VOIP_RECORD] = {
		.dac_id = VBC_DA1,
		.adc_id = VBC_AD2,
		.out_sel = VBC_MUX_DAC1_OUT_SEL-VBC_MUX_START,
		.adc_source_sel = VBC_MUX_ADC2_SOURCE-VBC_MUX_START,
		.dac_iis_port = VBC_MUX_DAC1_IIS_SEL-VBC_MUX_START,
		.adc_iis_port = VBC_MUX_ADC2_IIS_SEL-VBC_MUX_START,
		.dma_params[SND_PCM_CAPTURE] = &pcm_voip_record_mcdt,
	},
	/* voip play */
	[VBC_DAI_ID_VOIP_PLAY] = {
		.dac_id = VBC_DA1,
		.adc_id = VBC_AD2,
		.out_sel = VBC_MUX_DAC1_OUT_SEL-VBC_MUX_START,
		.adc_source_sel = VBC_MUX_ADC2_SOURCE-VBC_MUX_START,
		.dac_iis_port = VBC_MUX_DAC1_IIS_SEL-VBC_MUX_START,
		.adc_iis_port = VBC_MUX_ADC2_IIS_SEL-VBC_MUX_START,
		.dma_params[SND_PCM_PLAYBACK] = &pcm_voip_play_mcdt,
	},
	/* fm */
	[VBC_DAI_ID_FM] = {
		.dac_id = VBC_DA0,
		.adc_id = VBC_AD3,
		.out_sel = VBC_MUX_DAC0_OUT_SEL-VBC_MUX_START,
		.adc_source_sel = VBC_MUX_ADC3_SOURCE-VBC_MUX_START,
		.dac_iis_port = VBC_MUX_DAC0_IIS_SEL-VBC_MUX_START,
		.adc_iis_port = VBC_MUX_ADC3_IIS_SEL-VBC_MUX_START,
		.dma_params[SND_PCM_CAPTURE] = NULL,
		.dma_params[SND_PCM_PLAYBACK] = NULL,
	},
	/* voice-capture */
	[VBC_DAI_ID_VOICE_CAPTURE] = {
		.dac_id = VBC_DA1,
		.adc_id = VBC_AD1,
		.out_sel = VBC_MUX_DAC1_OUT_SEL-VBC_MUX_START,
		.adc_source_sel = VBC_MUX_ADC1_SOURCE-VBC_MUX_START,
		.dac_iis_port = VBC_MUX_DAC1_IIS_SEL-VBC_MUX_START,
		.adc_iis_port = VBC_MUX_ADC1_IIS_SEL-VBC_MUX_START,
		.dma_params[SND_PCM_CAPTURE] = &vbc_pcm_voice_capture_mcdt,
	},
	/* loop record
	*/
	[VBC_DAI_ID_LOOP_RECORD] = {
		.adc_id = VBC_AD2,
		.dac_id = VBC_DA1,
		.adc_source_sel = VBC_MUX_ADC2_SOURCE-VBC_MUX_START,
		.adc_iis_port = VBC_MUX_ADC2_IIS_SEL-VBC_MUX_START,
		.dma_params[SND_PCM_CAPTURE] = &pcm_loop_record_mcdt,
	},
	/* loop play
	*/
	[VBC_DAI_ID_LOOP_PLAY] = {
		.adc_id = VBC_AD2,
		.dac_id = VBC_DA1,
		.out_sel = VBC_MUX_DAC1_OUT_SEL-VBC_MUX_START,
		.dac_iis_port = VBC_MUX_DAC1_IIS_SEL-VBC_MUX_START,
		.dma_params[SND_PCM_PLAYBACK] = &pcm_loop_play_mcdt,
	},
	/* fm_capture */
	[VBC_DAI_ID_FM_CAPTURE] = {
		.adc_id = VBC_AD3,
		.adc_source_sel = VBC_MUX_ADC3_SOURCE-VBC_MUX_START,
		.adc_iis_port = VBC_MUX_ADC3_IIS_SEL-VBC_MUX_START,
		.dma_params[SND_PCM_CAPTURE] = &vbc_pcm_fm_caputre,
	},
	/* bt_caputure */
	[VBC_DAI_ID_BT_CAPTURE] = {
		.adc_id = VBC_AD2,
		.adc_source_sel = VBC_MUX_ADC2_SOURCE-VBC_MUX_START,
		.adc_iis_port = VBC_MUX_ADC2_IIS_SEL-VBC_MUX_START,
		.dma_params[SND_PCM_CAPTURE] = &vbc_pcm_bt_caputre,
	},
};

static int is_ap_vbc_ctl(int id, int stream)
{
	int ret = 0;

	switch (id) {
	case VBC_DAI_ID_NORMAL_OUTDSP:
		ret = 1;
		break;
	case VBC_DAI_ID_FM_CAPTURE:
		ret = 1;
		break;
	case VBC_DAI_ID_BT_CAPTURE:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}

static void normal_vbc_protect_spin_lock(
	struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *srtd = substream->private_data;
	struct aud_pm_vbc *pm_vbc;

	pm_vbc = aud_pm_vbc_get();
	if (sprd_is_normal_playback(srtd->cpu_dai->id,
		substream->stream))
		spin_lock(&pm_vbc->pm_spin_cmd_prot);
}

static void normal_vbc_protect_spin_unlock(
	struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *srtd = substream->private_data;
	struct aud_pm_vbc *pm_vbc;

	pm_vbc = aud_pm_vbc_get();
	if (sprd_is_normal_playback(srtd->cpu_dai->id,
		substream->stream))
		spin_unlock(&pm_vbc->pm_spin_cmd_prot);
}

static void normal_vbc_protect_mutex_lock(
	struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *srtd = substream->private_data;
	struct aud_pm_vbc *pm_vbc;

	pm_vbc = aud_pm_vbc_get();
	if (sprd_is_normal_playback(srtd->cpu_dai->id,
		substream->stream))
		mutex_lock(&pm_vbc->pm_mtx_cmd_prot);
}

static void normal_vbc_protect_mutex_unlock(
	struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *srtd = substream->private_data;
	struct aud_pm_vbc *pm_vbc;

	pm_vbc = aud_pm_vbc_get();
	if (sprd_is_normal_playback(srtd->cpu_dai->id,
		substream->stream))
		mutex_unlock(&pm_vbc->pm_mtx_cmd_prot);
}

static void set_dai_used_channel(int dai_id,
	int stream, struct snd_pcm_hw_params *params)
{
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		g_vbc[dai_id].dac_used_chan_count = params_channels(params);
	else
		g_vbc[dai_id].adc_used_chan_count = params_channels(params);
}

static int get_dai_used_channel(int dai_id, int stream)
{
	int chan = VBC_ALL_CHAN;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		if (g_vbc[dai_id].dac_used_chan_count == 1)
			chan = VBC_LEFT;
		else if (g_vbc[dai_id].dac_used_chan_count == 2)
			chan = VBC_ALL_CHAN;
		else
			chan = VBC_ALL_CHAN;

	else
		if (g_vbc[dai_id].adc_used_chan_count == 1)
			chan = VBC_LEFT;
		else if (g_vbc[dai_id].adc_used_chan_count == 2)
			chan = VBC_ALL_CHAN;
		else
			chan = VBC_ALL_CHAN;

	return chan;
}

static void sprd_dma_config_init(struct platform_device *pdev)
{
	/*normal playback*/
	vbc_pcm_normal_p_outdsp.dev_paddr[0] =
		vbc_phy_ap2dsp(VBC_AUDPLY_FIFO_WR_0 -
		VBC_OFLD_ADDR + sprd_ap_vbc_phy_base);
	vbc_pcm_normal_p_outdsp.dev_paddr[1] =
	vbc_phy_ap2dsp(VBC_AUDPLY_FIFO_WR_1 - VBC_OFLD_ADDR +
		sprd_ap_vbc_phy_base);
	vbc_pcm_normal_p_outdsp.channels[0] = DMA_REQ_DA0_DEV_ID;
	vbc_pcm_normal_p_outdsp.channels[1] = DMA_REQ_DA1_DEV_ID;
	vbc_pcm_normal_p_outdsp.used_dma_channel_name[0] = "normal_p_l";
	vbc_pcm_normal_p_outdsp.used_dma_channel_name[1] = "normal_p_r";

	/*normal capture*/
	vbc_pcm_normal_c_outdsp.dev_paddr[0] =
		vbc_phy_ap2dsp(VBC_AUDRCD_FIFO_RD_0 - VBC_OFLD_ADDR +
		sprd_ap_vbc_phy_base);
	vbc_pcm_normal_c_outdsp.dev_paddr[1] =
		vbc_phy_ap2dsp(VBC_AUDRCD_FIFO_RD_1 - VBC_OFLD_ADDR +
		sprd_ap_vbc_phy_base);
	vbc_pcm_normal_c_outdsp.channels[0] = DMA_REQ_AD0_DEV_ID;
	vbc_pcm_normal_c_outdsp.channels[1] = DMA_REQ_AD1_DEV_ID;
	vbc_pcm_normal_c_outdsp.used_dma_channel_name[0] = "normal_c_l";
	vbc_pcm_normal_c_outdsp.used_dma_channel_name[1] = "normal_c_r";

	/*voic capture*/
	vbc_pcm_voice_capture_mcdt.dev_paddr[0] =
		/* dma src address*/
		mcdt_adc_dma_phy_addr(MCDT_CHAN_VOICE_CAPTURE);
	vbc_pcm_voice_capture_mcdt.used_dma_channel_name[0] = "voice_c";
	vbc_pcm_voice_capture_mcdt.use_mcdt = 1;

	/*loop back record*/
	pcm_loop_record_mcdt.dev_paddr[0] =
		mcdt_adc_dma_phy_addr(MCDT_CHAN_LOOP);
	pcm_loop_record_mcdt.used_dma_channel_name[0] = "loop_c";
	pcm_loop_record_mcdt.use_mcdt = 1;
	/*loop back play*/
	pcm_loop_play_mcdt.dev_paddr[0] =
		mcdt_dac_dma_phy_addr(MCDT_CHAN_LOOP);
	pcm_loop_play_mcdt.used_dma_channel_name[0] = "loop_p";
	pcm_loop_play_mcdt.use_mcdt = 1;
	/*fast playback*/
	pcm_fast_play_mcdt.dev_paddr[0] =
		mcdt_dac_dma_phy_addr(MCDT_CHAN_FAST_PLAY);
	pcm_fast_play_mcdt.used_dma_channel_name[0] = "fast_p";
	pcm_fast_play_mcdt.use_mcdt = 1;

	/*voip capture*/
	pcm_voip_record_mcdt.dev_paddr[0] =
		mcdt_adc_dma_phy_addr(MCDT_CHAN_VOIP);
	pcm_voip_record_mcdt.used_dma_channel_name[0] = "voip_c";
	pcm_voip_record_mcdt.use_mcdt = 1;
	/*voip play*/
	pcm_voip_play_mcdt.dev_paddr[0] =
		mcdt_dac_dma_phy_addr(MCDT_CHAN_VOIP);
	pcm_voip_play_mcdt.used_dma_channel_name[0] = "voip_p";
	pcm_voip_play_mcdt.use_mcdt = 1;
	/* fm capture dma data equal normal capture so they
	 * can not be concurrent
	 */
	vbc_pcm_fm_caputre.dev_paddr[0] =
		vbc_pcm_normal_c_outdsp.dev_paddr[0];
	vbc_pcm_fm_caputre.dev_paddr[1] =
		vbc_pcm_normal_c_outdsp.dev_paddr[1];
	vbc_pcm_fm_caputre.channels[0] =
		vbc_pcm_normal_c_outdsp.channels[0];
	vbc_pcm_fm_caputre.channels[1] =
		vbc_pcm_normal_c_outdsp.channels[1];
	vbc_pcm_fm_caputre.used_dma_channel_name[0] =
		vbc_pcm_normal_c_outdsp.used_dma_channel_name[0];
	vbc_pcm_fm_caputre.used_dma_channel_name[1] =
		vbc_pcm_normal_c_outdsp.used_dma_channel_name[1];
	/* bt capture */
	vbc_pcm_bt_caputre.dev_paddr[0] =
		vbc_phy_ap2dsp(VBC_AUDRCD_FIFO_RD_0 - VBC_OFLD_ADDR +
		sprd_ap_vbc_phy_base);
	vbc_pcm_bt_caputre.dev_paddr[1] =
		vbc_phy_ap2dsp(VBC_AUDRCD_FIFO_RD_1 - VBC_OFLD_ADDR +
		sprd_ap_vbc_phy_base);
	vbc_pcm_bt_caputre.channels[0] = DMA_REQ_AD0_DEV_ID;
	vbc_pcm_bt_caputre.channels[1] = DMA_REQ_AD1_DEV_ID;
	vbc_pcm_bt_caputre.used_dma_channel_name[0] = "normal_c_l";
	vbc_pcm_bt_caputre.used_dma_channel_name[1] = "normal_c_r";
}

/*
 * debug_change_normal_capture_to_vbc_dump
 */
static int32_t is_vbc_dump(struct snd_soc_dai *dai,
	struct snd_pcm_substream *substream)
{
	struct vbc_codec_priv *priv_data = NULL;

	priv_data = snd_soc_dai_get_drvdata(dai);
	if (priv_data == NULL) {
		pr_err("%s drv_data is not set\n", __func__);
		return false;
	}

	if (dai->id == VBC_DAI_ID_NORMAL_OUTDSP &&
	    substream->stream == SNDRV_PCM_STREAM_CAPTURE &&
	    priv_data->open_vbc_dump_flag == true) {
		pr_info("open vbc dump function\n");
		return true;
	}

	return false;
}

#define VBC_DAI_OPS_TYPE_START_UP 1
#define VBC_DAI_OPS_TYPE_HW_PARAMS 2
#define VBC_DAI_OPS_TYPE_TRIGER 3
#define VBC_DAI_OPS_TYPE_FREE 4
#define VBC_DAI_OPS_TYPE_SHUTDOWN 5
#define VBC_DAI_OPS_TYPE_TO_STRING(type) #type

static const char *dai_ops_type_to_string(int type)
{
	switch (type) {
	case VBC_DAI_OPS_TYPE_START_UP:
		return VBC_DAI_OPS_TYPE_TO_STRING(VBC_DAI_OPS_TYPE_START_UP);
	case VBC_DAI_OPS_TYPE_HW_PARAMS:
		return VBC_DAI_OPS_TYPE_TO_STRING(VBC_DAI_OPS_TYPE_HW_PARAMS);
	case VBC_DAI_OPS_TYPE_TRIGER:
		return VBC_DAI_OPS_TYPE_TO_STRING(VBC_DAI_OPS_TYPE_TRIGER);
	case VBC_DAI_OPS_TYPE_FREE:
		return VBC_DAI_OPS_TYPE_TO_STRING(VBC_DAI_OPS_TYPE_FREE);
	case VBC_DAI_OPS_TYPE_SHUTDOWN:
		return VBC_DAI_OPS_TYPE_TO_STRING(VBC_DAI_OPS_TYPE_SHUTDOWN);
	default:
		return "";
	}
}

static int32_t vbc_dai_need_notify_dsp(struct snd_soc_dai *dai,
	struct snd_pcm_substream *substream, int dai_ops_type)
{
	int ret;
	int is_dump = true;

	is_dump = is_vbc_dump(dai, substream);
	switch (dai->id) {
	case VBC_DAI_ID_FM_CAPTURE:
		ret = false;
		break;
	case VBC_DAI_ID_NORMAL_OUTDSP:
		/* vbc dump */
		ret = (is_dump == true) ? false : true;
		break;
	case VBC_DAI_ID_VOICE_CAPTURE:
		/* voice capture */
		ret = (dai_ops_type == VBC_DAI_OPS_TYPE_HW_PARAMS) ?
			false : true;
		break;
	default:
		ret = true;
		break;
	}

	return ret;
}

static int vbc_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	int ret = 0;
	struct aud_pm_vbc *pm_vbc;

	sp_asoc_pr_info("%s VBC(%s-%s)\n", __func__, dai->driver->name,
		(substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
			"Capture" : "Playback");
	pm_vbc = aud_pm_vbc_get();
	mutex_lock(&pm_vbc->pm_mtx_cnt);
	if (!sprd_is_normal_playback(dai->id, substream->stream))
		pm_vbc->other_case_cnt++;
	else {
		/*
		 * if it is normal playback, all varialbles should be
		 * inited any way.
		*/
		mutex_lock(&pm_vbc->pm_mtx_cmd_prot);
		pm_vbc->is_access_send = false;
		spin_lock(&pm_vbc->pm_spin_cmd_prot);
		pm_vbc->is_startup = false;
		spin_unlock(&pm_vbc->pm_spin_cmd_prot);
		mutex_unlock(&pm_vbc->pm_mtx_cmd_prot);
	}
	mutex_unlock(&pm_vbc->pm_mtx_cnt);
	/*
	 * offload mixer normal: 1.1 startup dsp disable dac0 fifo,
	 * enable ofld_dp  1.2 hwparam ap pre data,enable audpply fifo,
	 * dsp enable dac0 fifo
	 */
	agdsp_access_enable();

	/* set xtl0 and xtlbuf0 force on when play fm */
	if (dai->id == VBC_DAI_ID_FM) {
		pr_info("%s, force_on_xtl\n", __func__);
		force_on_xtl(true);
	}

	/* fm caputre no startup */
	if (true == vbc_dai_need_notify_dsp(dai, substream,
		VBC_DAI_OPS_TYPE_START_UP)) {
		normal_vbc_protect_mutex_lock(substream);
		ret = vbc_dsp_func_startup(dai->dev, dai->id,
			substream->stream, dai->driver->name);
		if (ret < 0) {
			normal_vbc_protect_mutex_unlock(substream);
			pr_err("vbc_dsp_func_startup return error");
			agdsp_access_disable();
			return 0;
		}
		normal_vbc_protect_spin_lock(substream);
		if (sprd_is_normal_playback(dai->id, substream->stream))
			pm_vbc->is_startup = true;
		normal_vbc_protect_spin_unlock(substream);
		normal_vbc_protect_mutex_unlock(substream);
	}

	agdsp_access_disable();

	return 0;
}

static void vbc_shutdown(struct snd_pcm_substream *substream,
			 struct snd_soc_dai *dai)
{
	int ret;
	int chan = VBC_ALL_CHAN;
	struct aud_pm_vbc *pm_vbc;

	sp_asoc_pr_info("%s VBC(%s-%s)\n", __func__, dai->driver->name,
		(substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
		"Capture" : "Playback");
	pm_vbc = aud_pm_vbc_get();
	mutex_lock(&pm_vbc->pm_mtx_cnt);
	if (!sprd_is_normal_playback(dai->id, substream->stream)) {
		if (--pm_vbc->other_case_cnt < 0) {
			pr_warn("%s pm_vbc->other_case_cnt =%d\n",
				__func__, pm_vbc->other_case_cnt);
			pm_vbc->other_case_cnt = 0;
		}
	}
	mutex_unlock(&pm_vbc->pm_mtx_cnt);
	ret = agdsp_access_enable();
	if (ret) {
		pr_err("%s, agdsp_access_enable failed!\n", __func__);
		return;
	}
	chan = get_dai_used_channel(dai->id, substream->stream);

	if (is_ap_vbc_ctl(dai->id, substream->stream)) {
		/* fifo disable */
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			ap_vbc_fifo_enable(0, substream->stream, chan);

		/* clear aud fifo */
		ap_vbc_fifo_clear(substream->stream);
	}

	if (dai->id == VBC_DAI_ID_VOICE_CAPTURE)
		/* voic capture dma uid */
		mcdt_adc_dma_disable(MCDT_CHAN_VOICE_CAPTURE);

	if (dai->id == VBC_DAI_ID_LOOP_RECORD)
		mcdt_adc_dma_disable(MCDT_CHAN_LOOP);

	if (dai->id == VBC_DAI_ID_LOOP_PLAY)
		mcdt_dac_dma_disable(MCDT_CHAN_LOOP);

	if (dai->id == VBC_DAI_ID_FAST_P)
		mcdt_dac_dma_disable(MCDT_CHAN_FAST_PLAY);

	if (dai->id == VBC_DAI_ID_VOIP_RECORD)
		mcdt_adc_dma_disable(MCDT_CHAN_VOIP);

	if (dai->id == VBC_DAI_ID_VOIP_PLAY)
		mcdt_dac_dma_disable(MCDT_CHAN_VOIP);

	/* release xtl0 and xtlbuf0 force on when fm close */
	if (dai->id == VBC_DAI_ID_FM) {
		pr_info("%s, force_on_xtl\n", __func__);
		force_on_xtl(false);
	}

	agdsp_access_disable();
	if (true == vbc_dai_need_notify_dsp(dai, substream,
		VBC_DAI_OPS_TYPE_SHUTDOWN)) {
		normal_vbc_protect_mutex_lock(substream);
		if (sprd_is_normal_playback(dai->id, substream->stream)) {
			if (pm_vbc->is_startup == false) {
				normal_vbc_protect_mutex_unlock(substream);
				pr_info("%s normal has already send %s\n",
					__func__, dai_ops_type_to_string(
					VBC_DAI_OPS_TYPE_SHUTDOWN));
				return;
			}
		}
		ret = vbc_dsp_func_shutdown(dai->dev, dai->id,
			substream->stream, dai->driver->name);
		if (ret < 0) {
			normal_vbc_protect_mutex_unlock(substream);
			return;
		}
		normal_vbc_protect_spin_lock(substream);
		if (sprd_is_normal_playback(dai->id, substream->stream))
			pm_vbc->is_startup = false;
		normal_vbc_protect_spin_unlock(substream);
		normal_vbc_protect_mutex_unlock(substream);
	}
}

static int vbc_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	int ret;
	unsigned int rate;
	int data_fmt = VBC_DAT_L16;
	int chan = VBC_ALL_CHAN;
	int default_watermark = 160;
	struct sprd_pcm_dma_params *dma_data = NULL;
	struct aud_pm_vbc *pm_vbc;

	pm_vbc = aud_pm_vbc_get();
	sp_asoc_pr_info("%s VBC(%s-%s)\n", __func__, dai->driver->name,
		(substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
		"Capture" : "Playback");

	if (dai->id == VBC_DAI_ID_VOICE_CAPTURE) {
		/* voic capture dma uid */
		vbc_pcm_voice_capture_mcdt.channels[0] =
			mcdt_adc_dma_enable(MCDT_CHAN_VOICE_CAPTURE,
			MCDT_FULL_WMK_VOICE_CAPTURE);
		/*
		 * get_mcdt_adc_dma_uid(MCDT_CHAN_VOICE_CAPTURE,
		 * MCDT_AP_DMA_CHAN_VOICE_CAPTURE,
		 * MCDT_FULL_WMK_VOICE_CAPTURE);
		 */
		if (vbc_pcm_voice_capture_mcdt.channels[0] < 0) {
			pr_err("%s vbc_pcm_voice_capture_mcdt.channels[0](dma device id) =%d\n",
				__func__,
				vbc_pcm_voice_capture_mcdt.channels[0]);
			return -EIO;
		}
		sp_asoc_pr_dbg("%s dai->id =%d,", __func__, dai->id);
		sp_asoc_pr_dbg("vbc_pcm_voice_capture_mcdt.channels[0]=%d\n",
			vbc_pcm_voice_capture_mcdt.channels[0]);
	}

	if (dai->id == VBC_DAI_ID_LOOP_RECORD) {
		/* loop record dma uid */
		pcm_loop_record_mcdt.channels[0] =
		mcdt_adc_dma_enable(MCDT_CHAN_LOOP, MCDT_FULL_WMK_LOOP);
		/*
		 * get_mcdt_adc_dma_uid(MCDT_CHAN_LOOP,
		 * MCDT_AP_DMA_CHAN_LOOP, MCDT_FULL_WMK_LOOP);
		 */
		if (pcm_loop_record_mcdt.channels[0] < 0) {
			pr_err("%s pcm_loop_record_mcdt.channels[0](dma device id) =%d\n",
				__func__,
				pcm_loop_record_mcdt.channels[0]);
			return -EIO;
		}
		sp_asoc_pr_dbg("%s dai->id =%d,", __func__, dai->id);
		sp_asoc_pr_dbg(" pcm_loop_record_mcdt.channels[0]=%d\n",
			pcm_loop_record_mcdt.channels[0]);
	}

	if (dai->id == VBC_DAI_ID_LOOP_PLAY) {
		/* loop play dma uid */
		pcm_loop_play_mcdt.channels[0] =
		mcdt_dac_dma_enable(MCDT_CHAN_LOOP, MCDT_EMPTY_WMK_LOOP);
		/* get_mcdt_dac_dma_uid(MCDT_CHAN_LOOP,
		 *	MCDT_AP_DMA_CHAN_LOOP, MCDT_EMPTY_WMK_LOOP);
		 */
		if (pcm_loop_play_mcdt.channels[0] < 0) {
			pr_err("%s pcm_loop_play_mcdt.channels[0](dma device id) =%d\n",
				__func__,
				pcm_loop_play_mcdt.channels[0]);
			return -EIO;
		}
		sp_asoc_pr_dbg("%s dai->id =%d", __func__, dai->id);
		sp_asoc_pr_dbg("pcm_loop_play_mcdt.channels[0] =%d\n",
			pcm_loop_play_mcdt.channels[0]);
	}

	if (dai->id == VBC_DAI_ID_FAST_P) {
		/* fast play dma uid */
		pcm_fast_play_mcdt.channels[0] =
			mcdt_dac_dma_enable(MCDT_CHAN_FAST_PLAY,
			MCDT_EMPTY_WMK_FAST_PLAY);
		/*
		 * get_mcdt_dac_dma_uid(MCDT_CHAN_FAST_PLAY,
		 * MCDT_AP_DMA_CHAN_FAST_PLAY, MCDT_EMPTY_WMK_FAST_PLAY);
		 */
		if (pcm_fast_play_mcdt.channels[0] < 0) {
			pr_err("%s error pcm_fast_play_mcdt.channels[0](dma device id) =%d\n",
				__func__,
				pcm_fast_play_mcdt.channels[0]);
			return -EIO;
		}
		sp_asoc_pr_dbg("%s dai->id =%d,",
			__func__, dai->id);
		sp_asoc_pr_dbg(" pcm_fast_play_mcdt.channels[0]=%d\n",
			pcm_fast_play_mcdt.channels[0]);
	}

	if (dai->id == VBC_DAI_ID_VOIP_RECORD) {
		/* voip record dma uid */
		pcm_voip_record_mcdt.channels[0] =
			mcdt_adc_dma_enable(MCDT_CHAN_VOIP, MCDT_FULL_WMK_VOIP);
		if (pcm_voip_record_mcdt.channels[0] < 0) {
			pr_err("%s pcm_voip_record_mcdt.channels[0]=%d\n",
				__func__,
				pcm_voip_record_mcdt.channels[0]);
			return -EIO;
		}
		sp_asoc_pr_dbg("%s dai->id =%d,", __func__, dai->id);
		sp_asoc_pr_dbg(" pcm_voip_record_mcdt.channels[0]=%d\n",
			pcm_voip_record_mcdt.channels[0]);
	}

	if (dai->id == VBC_DAI_ID_VOIP_PLAY) {
		/*voip play dma uid*/
		pcm_voip_play_mcdt.channels[0] =
		mcdt_dac_dma_enable(MCDT_CHAN_VOIP, MCDT_EMPTY_WMK_VOIP);
		if (pcm_voip_play_mcdt.channels[0] < 0) {
			pr_err("%s pcm_voip_play_mcdt.channels[0]=%d\n",
				__func__, pcm_voip_play_mcdt.channels[0]);
			return -EIO;
		}
		sp_asoc_pr_dbg("%s dai->id =%d,", __func__, dai->id);
		sp_asoc_pr_dbg(" pcm_voip_play_mcdt.channels[0] =%d\n",
			pcm_voip_play_mcdt.channels[0]);
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		data_fmt = VBC_DAT_L16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data_fmt = VBC_DAT_L24;
		break;
	default:
		pr_err("%s, ERR:VBC Only Supports Format", __func__);
		pr_err("SNDRV_PCM_FORMAT_S16_LE and SNDRV_PCM_FORMAT_S24_LE\n");
		break;
	}
	pr_info("%s data_fmt=%d\n", __func__, data_fmt);

	if (params_channels(params) > 2)
		pr_err("%s, ERR:VBC Can NOT Supports Grate 2 Channels\n",
			__func__);
	set_dai_used_channel(dai->id, substream->stream, params);
	chan = get_dai_used_channel(dai->id, substream->stream);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE
		&& dai->id == VBC_DAI_ID_NORMAL_OUTDSP) {
		rate = params_rate(params);
		g_vbc[dai->id].dma_params[SND_PCM_CAPTURE]->desc.fragmens_len
			= ((rate * DEFAULT_WATERMARK) / DEFAULT_RATE) & ~BIT(0);
	}

	dma_data = ((substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
		g_vbc[dai->id].dma_params[SND_PCM_PLAYBACK] :
		g_vbc[dai->id].dma_params[SND_PCM_CAPTURE]);

	if (dma_data) {
		if (VBC_DAT_L24 == data_fmt || 1 == dma_data->use_mcdt)
			dma_data->desc.datawidth = WORD_WIDTH;
		else if (data_fmt == VBC_DAT_L16)
			dma_data->desc.datawidth = SHORT_WIDTH;
	}
	snd_soc_dai_set_dma_data(dai, substream, dma_data);

	if (is_ap_vbc_ctl(dai->id, substream->stream)) {
		/* clear aud fifo */
		ap_vbc_fifo_clear(substream->stream);
		/* set aud fifo size */
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE
			&& dai->id == VBC_DAI_ID_NORMAL_OUTDSP) {
			rate = params_rate(params);
			default_watermark = ((rate * DEFAULT_WATERMARK) / DEFAULT_RATE) & ~BIT(0);
		}
		ap_vbc_set_fifo_size(substream->stream,
			default_watermark, VB_AUDPLY_EMPTY_WATERMARK);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			/*offload mixer normal need in hw_params*/
			sp_asoc_pr_dbg("ap_vbc_fifo_enable 1 %s\n", __func__);
			ap_vbc_reg_write(VBC_AUDPLY_FIFO_WR_0, 0);
			ap_vbc_reg_write(VBC_AUDPLY_FIFO_WR_1, 0);
			ap_vbc_fifo_enable(1, substream->stream, chan);
		}
		/* set data format */
		ap_vbc_aud_dat_format_set(substream->stream, data_fmt);
		/* open AP SRC */
		ap_vbc_src_set(1,
			substream->stream, params_rate(params));
	}

	rate = params_rate(params);
	if (true == vbc_dai_need_notify_dsp(dai, substream,
		VBC_DAI_OPS_TYPE_HW_PARAMS)){
		normal_vbc_protect_mutex_lock(substream);
		if (sprd_is_normal_playback(dai->id, substream->stream)) {
			if (pm_vbc->is_startup == false) {
				normal_vbc_protect_mutex_unlock(substream);
				pr_info("%s normal has already send %s\n",
					__func__, dai_ops_type_to_string(
					VBC_DAI_OPS_TYPE_HW_PARAMS));
				return 0;
			}
		}

		ret = vbc_dsp_func_hwparam(dai->dev, dai->id,
			substream->stream, dai->driver->name,
			params_channels(params), VBC_DAT_L16,
			dsp_vbc_get_src_mode(rate));
		if (ret < 0) {
			normal_vbc_protect_mutex_unlock(substream);
			pr_err("vbc_dsp_func_hwparam return error\n");
			return 0;
		}
		normal_vbc_protect_mutex_unlock(substream);
	}

	return 0;
}

static int vbc_hw_free(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	int ret;
	struct aud_pm_vbc *pm_vbc;

	pm_vbc = aud_pm_vbc_get();
	sp_asoc_pr_info("%s VBC(%s-%s)\n", __func__, dai->driver->name,
		(substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
		"Capture" : "Playback");
	if (true == vbc_dai_need_notify_dsp(dai, substream,
				VBC_DAI_OPS_TYPE_FREE)) {
		normal_vbc_protect_mutex_lock(substream);
		if (sprd_is_normal_playback(dai->id, substream->stream)) {
			if (pm_vbc->is_startup == false) {
				normal_vbc_protect_mutex_unlock(substream);
				pr_info("%s normal has already send %s\n",
					__func__, dai_ops_type_to_string(
					VBC_DAI_OPS_TYPE_FREE));
				return 0;
			}
		}
		ret = vbc_dsp_func_hw_free(dai->dev, dai->id,
			substream->stream, dai->driver->name);
		if (ret < 0) {
			normal_vbc_protect_mutex_unlock(substream);
			return 0;
		}
		normal_vbc_protect_mutex_unlock(substream);
	}

	return 0;
}

static int vbc_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	int ret;
	int chan = VBC_ALL_CHAN;
	struct aud_pm_vbc *pm_vbc;

	pm_vbc = aud_pm_vbc_get();
	sp_asoc_pr_info("%s VBC(%s-%s), cmd:%d\n", __func__, dai->driver->name,
		(substream->stream == SNDRV_PCM_STREAM_CAPTURE) ?
		"Capture" : "Playback", cmd);

	chan = get_dai_used_channel(dai->id, substream->stream);
	if (is_ap_vbc_ctl(dai->id, substream->stream))
		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			/* fifo enable */
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				ap_vbc_fifo_enable(1,
					substream->stream, chan);

			/* vbc dma enable */
			ap_vbc_aud_dma_chn_en(1, substream->stream, chan);
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			/* fifo disable */
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				ap_vbc_fifo_enable(0,
					substream->stream, chan);

			/* vbc dma disable */
			ap_vbc_aud_dma_chn_en(0, substream->stream, chan);
			break;
		}

	if (true == vbc_dai_need_notify_dsp(dai, substream,
		VBC_DAI_OPS_TYPE_TRIGER)) {
		normal_vbc_protect_spin_lock(substream);
		if (sprd_is_normal_playback(dai->id, substream->stream)) {
			if (pm_vbc->is_startup == false) {
				normal_vbc_protect_spin_unlock(substream);
				pr_info("%s normal has already send %s\n",
					__func__, dai_ops_type_to_string(
					VBC_DAI_OPS_TYPE_TRIGER));
				return 0;
			}
		}
		ret = vbc_dsp_func_trigger(dai->dev, dai->id,
				substream->stream, dai->driver->name, cmd);
		if (ret < 0) {
			normal_vbc_protect_spin_unlock(substream);
			pr_err("vbc_dsp_func_trigger return error\n");
			return ret;
		}
		normal_vbc_protect_spin_unlock(substream);
	}

	return 0;
}

static struct snd_soc_dai_ops vbc_dai_ops = {
	.startup = vbc_startup,
	.shutdown = vbc_shutdown,
	.hw_params = vbc_hw_params,
	.trigger = vbc_trigger,
	.hw_free = vbc_hw_free,
};

static int vbc_normal_resume(struct snd_soc_dai *dai)
{
	struct aud_pm_vbc *pm_vbc;

	/* Make sure it is normal playback */
	if (!dai->playback_active)
		return 0;
	pm_vbc = aud_pm_vbc_get();
	if (pm_vbc->is_access_send == true) {
		restore_access();
		pm_vbc->is_access_send = false;
		pr_info("%s resumed\n", __func__);
	}

	return 0;
}

static int vbc_normal_suspend(struct snd_soc_dai *dai)
{
	struct aud_pm_vbc *pm_vbc;

	/* make sure it is normal playback */
	if (!dai->playback_active)
		return 0;

	pm_vbc = aud_pm_vbc_get();
	/* make sure it is normal playback only and startup has sended */
	mutex_lock(&pm_vbc->pm_mtx_cnt);
	if (pm_vbc->other_case_cnt != 0) {
		pr_info("%s not normal only cmd other_case_cnt=%d\n",
			__func__, pm_vbc->other_case_cnt);
		mutex_unlock(&pm_vbc->pm_mtx_cnt);
		return 0;
	}
	/*
	 * vbc clock clear and agcp access disable must after codec
	 * suspened because codec needed agcp access enable.
	 * Puting these function here and set bus_control to be true
	 * will meet requirements. Do not put them in PM_SUSPEND_PREPARE
	 * it is too early.
	 */
	pr_info("%s enter suspend\n", __func__);
	mutex_lock(&pm_vbc->pm_mtx_cmd_prot);
	if (pm_vbc->is_startup == false) {
		pr_info("%s startup not called just return\n", __func__);
		mutex_unlock(&pm_vbc->pm_mtx_cmd_prot);
		mutex_unlock(&pm_vbc->pm_mtx_cnt);
		return 0;
	}
	spin_lock(&pm_vbc->pm_spin_cmd_prot);
	pm_vbc->is_startup = false;
	spin_unlock(&pm_vbc->pm_spin_cmd_prot);
	pr_info("%s send shutdown\n", __func__);
	pm_shutdown();
	mutex_unlock(&pm_vbc->pm_mtx_cmd_prot);
	disable_access_force();
	pm_vbc->is_access_send = true;
	pr_info("%s suspeded\n", __func__);
	mutex_unlock(&pm_vbc->pm_mtx_cnt);

	return 0;
}


#define SPRD_VBC_DAI_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_S24_LE)
static struct snd_soc_dai_driver vbc_dai[] = {
	{/* 0 */
	.name = "normal-without-dsp",
	.id = VBC_DAI_ID_NORMAL_OUTDSP,
	.playback = {
		.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.ops = &vbc_dai_ops,
	.resume = vbc_normal_resume,
	.suspend = vbc_normal_suspend,
	.bus_control = true,
	},
	{/* 1 */
	.name = "fast-playback",
	.id = VBC_DAI_ID_FAST_P,
	.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.ops = &vbc_dai_ops,
	},
	{/* 2 */
	.name = "offload",
	.id = VBC_DAI_ID_OFFLOAD,
	.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			},
	.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			},
	.ops = &vbc_dai_ops,
	},
	{/* 3 */
	.name = "voice",
	.id = VBC_DAI_ID_VOICE,
	.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.ops = &vbc_dai_ops,
	},
	{/* 4 */
	.name = "voip_record",
	.id = VBC_DAI_ID_VOIP_RECORD,
	.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.ops = &vbc_dai_ops,
	},
	{/* 5 */
	.name = "voip_play",
	.id = VBC_DAI_ID_VOIP_PLAY,
	.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.ops = &vbc_dai_ops,
	},
	{/* 6 */
	.name = "fm",
	.id = VBC_DAI_ID_FM,
	.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.ops = &vbc_dai_ops,
	},
	{/* 7 */
	.name = "voice-capture",
	.id = VBC_DAI_ID_VOICE_CAPTURE,
	.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.ops = &vbc_dai_ops,
	},
	{/* 8 */
	.name = "loop_record",
	.id = VBC_DAI_ID_LOOP_RECORD,
	.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.ops = &vbc_dai_ops,
	},
	{/* 9 */
	.name = "loop_play",
	.id = VBC_DAI_ID_LOOP_PLAY,
	.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	 .capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	 .ops = &vbc_dai_ops,
	 },
	 {/* 10 */
	 .name = "fm_capture",
	 .id = VBC_DAI_ID_FM_CAPTURE,
	 .capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.ops = &vbc_dai_ops,
	},
	{/* 11 */
	 .name = "bt_capture",
	 .id = VBC_DAI_ID_BT_CAPTURE,
	 .capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_CONTINUOUS,
			.rate_max = 96000,
			.formats = SPRD_VBC_DAI_PCM_FORMATS,
			},
	.ops = &vbc_dai_ops,
	},
};

static struct aud_pm_vbc *aud_pm_vbc_get(void)
{
	return pm_vbc;
}

static void pm_vbc_init(void)
{
	struct aud_pm_vbc *pm_vbc;

	pm_vbc = aud_pm_vbc_get();
	if (pm_vbc == NULL)
		return;

	mutex_init(&pm_vbc->pm_mtx_cmd_prot);
	spin_lock_init(&pm_vbc->pm_spin_cmd_prot);
	mutex_init(&pm_vbc->pm_mtx_cnt);
}

static int vbc_drv_probe(struct platform_device *pdev)
{
	int ret;

	sp_asoc_pr_dbg("%s\n", __func__);

	/* 1. probe CODEC */
	ret = sprd_vbc_codec_probe(pdev);
	if (ret < 0)
		goto probe_err;
	/*
	 * should first call sprd_vbc_codec_probe
	 * because we will call platform_get_drvdata(pdev)
	 */
	ret = vbc_of_setup(pdev);
	if (ret < 0) {
		pr_err("%s: failed to setup vbc dt, ret=%d\n", __func__, ret);
		return -ENODEV;
	}
	aud_ipc_ch_open(AMSG_CH_DSP_GET_PARAM_FROM_SMSG_NOREPLY);
	aud_ipc_ch_open(AMSG_CH_VBC_CTL);
	sprd_dma_config_init(pdev);

	/* 2. probe DAIS */
	ret = snd_soc_register_codec(&pdev->dev, &sprd_vbc_codec, vbc_dai,
			ARRAY_SIZE(vbc_dai));

	if (ret < 0) {
		pr_err("%s, Register VBC to DAIS Failed!\n", __func__);
		goto probe_err;
	}

	pm_vbc = devm_kzalloc(&pdev->dev, sizeof(*pm_vbc), GFP_KERNEL);
	if (!pm_vbc) {
		ret = -ENOMEM;
		goto probe_err;
	}
	pm_vbc_init();

	return ret;
probe_err:
	pr_err("%s, error return %i\n", __func__, ret);

	return ret;
}

static int vbc_drv_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);

	sprd_vbc_codec_remove(pdev);
	aud_ipc_ch_close(AMSG_CH_DSP_GET_PARAM_FROM_SMSG_NOREPLY);
	aud_ipc_ch_close(AMSG_CH_VBC_CTL);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id vbc_of_match[] = {
	{.compatible = "sprd,vbc-r3p0",},
	{},
};

MODULE_DEVICE_TABLE(of, vbc_of_match);
#endif

static struct platform_driver vbc_driver = {
	.driver = {
		   .name = "vbc-r3p0",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(vbc_of_match),
		   },

	.probe = vbc_drv_probe,
	.remove = vbc_drv_remove,
};

static int __init sprd_vbc_driver_init(void)
{
	return platform_driver_register(&vbc_driver);
}

late_initcall(sprd_vbc_driver_init);

MODULE_DESCRIPTION("SPRD ASoC VBC CPU-DAI driver");
MODULE_AUTHOR("Jian chen <jian.chen@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("cpu-dai:vbc-r3p0");
