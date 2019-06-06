/*
 * sound/soc/sprd/dai/vbc/r3p0/vbc-phy-r3p0.c
 *
 * SPRD SoC VBC -- SpreadTrum SOC for VBC driver function.
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
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <sound/soc.h>

#include "audio-sipc.h"
#include "sprd-asoc-common.h"
#include "sprd-codec.h"
#include "vbc-codec.h"
#include "vbc-phy-r3p0.h"

unsigned long sprd_ap_vbc_virt_base;
unsigned int sprd_ap_vbc_phy_base;
static u32 vbc_phy_offset;

static unsigned long sprd_codec_dig_reg_base_v;
static unsigned int sprd_codec_dig_reg_base_p;

enum SND_PCM_TRIGGER_CMD_E {
	SND_PCM_TRIGGER_CMD_STOP = 0,
	SND_PCM_TRIGGER_CMD_START,
	SND_PCM_TRIGGER_CMD_PAUSE_PUSH = 3,
	SND_PCM_TRIGGER_CMD_PAUSE_RELEASE,
	SND_PCM_TRIGGER_CMD_SUSPEND,
	SND_PCM_TRIGGER_CMD_RESUME,
	SND_PCM_TRIGGER_CMD_END
};

struct snd_pcm_hw_paras {
	unsigned int channels;
	unsigned int rate;
	unsigned int format;
};

struct snd_pcm_stream_info {
	char name[32];
	int id;
	enum SND_PCM_DIRECTION_E stream;
};

struct sprd_vbc_stream_startup_shutdown {
	struct snd_pcm_stream_info stream_info;
	struct snd_pcm_statup_paras startup_para;
};

struct sprd_vbc_stream_hw_paras {
	struct snd_pcm_stream_info stream_info;
	struct snd_pcm_hw_paras hw_params_info;
};

struct sprd_vbc_stream_trigger {
	struct snd_pcm_stream_info stream_info;
	enum SND_PCM_TRIGGER_CMD_E pcm_trigger_cmd;
};

static DEFINE_SPINLOCK(vbc_lock);
/* ap vbc phy driver start */
static void ap_vbc_phy_fifo_enable_raw(int enable, int stream, int chan)
{
	sp_asoc_pr_dbg("%s %d, enable=%d, chan=%d\n",
		__func__, __LINE__, enable, chan);
	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (chan == VBC_LEFT) {
			ap_vbc_reg_update(VBC_AUD_EN,
				((enable ? 1 : 0) << (AUDPLY_AP_FIFO0_EN)),
				(1 << AUDPLY_AP_FIFO0_EN));
		} else if (chan == VBC_RIGHT) {
			ap_vbc_reg_update(VBC_AUD_EN,
				((enable ? 1 : 0) << (AUDPLY_AP_FIFO1_EN)),
				(1 << AUDPLY_AP_FIFO1_EN));
		} else if (chan == VBC_ALL_CHAN) {
			ap_vbc_reg_update(VBC_AUD_EN,
				((enable ? 1 : 0) << (AUDPLY_AP_FIFO0_EN)),
				(1 << AUDPLY_AP_FIFO0_EN));
			ap_vbc_reg_update(VBC_AUD_EN,
				((enable ? 1 : 0) << (AUDPLY_AP_FIFO1_EN)),
				(1 << AUDPLY_AP_FIFO1_EN));
		}
	}
}

static void ap_vbc_phy_fifo_clear(int stream)
{
	int shift = 0;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		shift = AUDPLY_AP_FIFO_CLR;
	} else if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		shift = AUDRCD_AP_FIFO_CLR;
	} else {
		pr_err("%s, wrong stream(%d)\n", __func__, stream);
		return;
	}
	ap_vbc_reg_update(VBC_AUD_CLR, (1 << shift),
				(1 << shift));
}

static void ap_vbc_phy_reg_raw_write(unsigned int reg, int val)
{
	writel_relaxed(val, (void *__iomem)(reg - VBC_OFLD_ADDR +
		sprd_ap_vbc_virt_base));

}

static void dsp_vbc_phy_reg_raw_write(unsigned int reg,
	int val, unsigned int mask)
{
	int ret = 0;
	struct sprd_vbc_kcontrol vbc_reg = {0};

	vbc_reg.reg = reg;
	vbc_reg.value = val;
	vbc_reg.mask = mask;
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_REG, -1,
		SND_VBC_DSP_IO_KCTL_SET,
		&vbc_reg, sizeof(struct sprd_vbc_kcontrol),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

}

/* for whale2  0x2c[7:4] move to 0x50(VBC_AUD_DMA_EN)[0:3] */
static void vbc_phy_audply_dma_chn_en(int en, int chan)
{
	if (chan == VBC_LEFT) {
		ap_vbc_reg_update(VBC_AUD_DMA_EN, (en << AUDPLY_DMA_DA0_EN),
				(1 << AUDPLY_DMA_DA0_EN));
	} else if (chan == VBC_RIGHT) {
		ap_vbc_reg_update(VBC_AUD_DMA_EN, (en << AUDPLY_DMA_DA1_EN),
				(1 << AUDPLY_DMA_DA1_EN));
	} else if (chan == VBC_ALL_CHAN) {
		ap_vbc_reg_update(VBC_AUD_DMA_EN, (en << AUDPLY_DMA_DA0_EN),
				(1 << AUDPLY_DMA_DA0_EN));
		ap_vbc_reg_update(VBC_AUD_DMA_EN, (en << AUDPLY_DMA_DA1_EN),
				(1 << AUDPLY_DMA_DA1_EN));
	}
}

static void vbc_phy_audrcd_dma_chn_en(int en, int chan)
{
	if (chan == VBC_LEFT) {
		ap_vbc_reg_update(VBC_AUD_DMA_EN, (en << AUDRCD_DMA_AD0_EN),
				(1 << AUDRCD_DMA_AD0_EN));
	} else if (chan == VBC_RIGHT) {
		ap_vbc_reg_update(VBC_AUD_DMA_EN, (en << AUDRCD_DMA_AD1_EN),
				(1 << AUDRCD_DMA_AD1_EN));
	} else if (chan == VBC_ALL_CHAN) {
		ap_vbc_reg_update(VBC_AUD_DMA_EN, (en << AUDRCD_DMA_AD0_EN),
				(1 << AUDRCD_DMA_AD0_EN));
		ap_vbc_reg_update(VBC_AUD_DMA_EN, (en << AUDRCD_DMA_AD1_EN),
				(1 << AUDRCD_DMA_AD1_EN));
	}
}

static void vbc_phy_audply_src0_en(int en)
{
	ap_vbc_reg_update(VBC_AUD_EN, (en << AUDPLY_SRC_EN_0),
				(1 << AUDPLY_SRC_EN_0));
}

static void vbc_phy_audply_src1_en(int en)
{
	ap_vbc_reg_update(VBC_AUD_EN, (en << AUDPLY_SRC_EN_1),
				(1 << AUDPLY_SRC_EN_1));
}

static void vbc_phy_audrcd_src0_en(int en)
{
	ap_vbc_reg_update(VBC_AUD_EN, (en << AUDRCD_SRC_EN_0),
				(1 << AUDRCD_SRC_EN_0));
}

static void vbc_phy_audrcd_src1_en(int en)
{
	ap_vbc_reg_update(VBC_AUD_EN, (en << AUDRCD_SRC_EN_1),
				(1 << AUDRCD_SRC_EN_1));
}

static void vbc_phy_audply_set_src_mode(enum AUD_AP_SRC_MODE_E mode)
{
	ap_vbc_reg_update(VBC_AUD_SRC_CTRL,
		((mode << AUDPLY_AP_SRC_MODE) &
		 (AUDPLY_AP_SRC_MODE_MASK << AUDPLY_AP_SRC_MODE)),
		(AUDPLY_AP_SRC_MODE_MASK << AUDPLY_AP_SRC_MODE));
}

static void vbc_phy_audrcd_set_src_mode(enum AUD_AP_SRC_MODE_E mode)
{
	ap_vbc_reg_update(VBC_AUD_SRC_CTRL,
		((mode << AUDRCD_AP_SRC_MODE) &
		 (AUDRCD_AP_SRC_MODE_MASK << AUDRCD_AP_SRC_MODE)),
		(AUDRCD_AP_SRC_MODE_MASK << AUDRCD_AP_SRC_MODE));
}

static void vbc_phy_audply_set_watermark(int full, int empty)
{
	ap_vbc_reg_update(VBC_AUDPLY_FIFO_CTRL,
		(full << AUDPLY_AP_FIFO_FULL_LVL)
		| (empty << AUDPLY_AP_FIFO_EMPTY_LVL),
		(AUDPLY_AP_FIFO_FULL_LVL_MASK << AUDPLY_AP_FIFO_FULL_LVL)
		| (AUDPLY_AP_FIFO_EMPTY_LVL_MASK << AUDPLY_AP_FIFO_EMPTY_LVL));
}

static void vbc_phy_audrcd_set_watermark(int full, int empty)
{
	ap_vbc_reg_update(VBC_AUDRCD_FIFO_CTRL,
		(full << AUDRCD_AP_FIFO_FULL_LVL)
		| (empty << AUDRCD_AP_FIFO_EMPTY_LVL),
		(AUDRCD_AP_FIFO_FULL_LVL_MASK << AUDRCD_AP_FIFO_FULL_LVL)
		| (AUDRCD_AP_FIFO_EMPTY_LVL_MASK << AUDRCD_AP_FIFO_EMPTY_LVL));
}

static void to_stream_info(int id, int stream,
		const char *name, struct snd_pcm_stream_info *stream_info)
{
	stream_info->id = id;
	strcpy(stream_info->name, name);
	stream_info->stream = stream;
	sp_asoc_pr_dbg("dai[%d](%s), strem[%d](%s)\n",
		id, name, stream, (stream == SNDRV_PCM_STREAM_CAPTURE) ?
		"Capture" : "Playback");
}

/*
 * Returns 1 for change, 0 for no change, or negative error code.
 */
int ap_reg_update(unsigned long reg_virt, int val, int mask)
{
	unsigned int new, old;

	spin_lock(&vbc_lock);
	old = ap_reg_read(reg_virt);
	new = (old & ~mask) | (val & mask);
	writel_relaxed(new, (void *__iomem)(reg_virt));
	spin_unlock(&vbc_lock);
	sp_asoc_pr_reg("[0x%04lx] U:[0x%08x] R:[0x%08x]\n",
		reg_virt & 0xFFFFFFFF, new,
		ap_reg_read(reg_virt));

	return old != new;
}

int ap_reg_write(unsigned long reg_virt, int val)
{
	spin_lock(&vbc_lock);
	writel_relaxed(val, (void *__iomem)(reg_virt));
	spin_unlock(&vbc_lock);
	sp_asoc_pr_reg("AP-VBC:[0x%lx] W:[0x%08x] R:[0x%08x]\n",
		(reg_virt) & 0xFFFFFFFF, val,
		ap_reg_read(reg_virt));

	return 0;
}

unsigned int ap_reg_read(unsigned long reg_virt)
{
	unsigned int ret = 0;

	ret = readl_relaxed((void *__iomem)(reg_virt));

	return ret;
}

/*
 * @return <0 failed, @@return ==0, get address success
 */
static int get_digital_codec_reg_base(unsigned long *virt_out,
	unsigned int *phy_out)
{

	if (0 == sprd_codec_dig_reg_base_v || 0 == sprd_codec_dig_reg_base_p) {
		pr_err("%s not get audio codec digital", __func__);
		pr_err("address virt and phy\n");
		return -1;
	}

	*virt_out = sprd_codec_dig_reg_base_v;
	*phy_out = sprd_codec_dig_reg_base_p;

	return 0;
}

#define REG_OFFSET(reg) ((reg) - SPRD_CODEC_DP_BASE)
int aud_digital_iis0_loop_enable(int setting, unsigned long virt_out)
{
	int msk, val, ret;

	switch (setting) {
	case AUD_IIS_MASTER_IIS0:
		/*
		 * 1.open aud clock enable
		 * 0x015E0000 bit[19] bit[20] set 1
		 */
		arch_audio_codec_digital_reg_enable();
		ret = agdsp_access_enable();
		if (ret) {
			pr_err("%s, agdsp_access_enable failed!\n", __func__);
			return ret;
		}

		/* 2.aud dac src set 48k 0x0175000C bit[0] set 1 */
		ap_reg_update(REG_OFFSET(AUD_DAC_CTL) + virt_out,
			DAC_FS_MODE_48k << DAC_FS_MODE,
			DAC_FS_MODE_MASK << DAC_FS_MODE);
		/* aud adc src 0x01750018 bit[3:0] 0xC */
		ap_reg_update(REG_OFFSET(AUD_ADC_CTL) + virt_out,
			ADC_FS_MODE_48k << ADC_FS_MODE,
			ADC_SRC_N_MASK << ADC_FS_MODE);
		/* 3.select iis path 0x015E003C bit[0] set 1 */
		arch_audio_iis_to_audio_top_enable(AG_IIS0, 1);
		/*
		 * 4.open aud dac path enable
		 * 0x01750000 bit[0] bit[2] set 1
		 */
		msk = BIT(DAC_EN_L) | BIT(DAC_EN_R);
		ap_reg_update(REG_OFFSET(AUD_TOP_CTL) + virt_out, msk, msk);
		/*
		 * match agdsp_access_disable called in
		 *arch_audio_codec_digital_reg_enable
		 */
		agdsp_access_disable();
		break;
	case AUD_IIS_MASTER_LOOP:
		/*
		 * 1.open aud clock enable
		 * 0x015E0000 bit[19] bit[20] set 1
		 arch_audio_codec_digital_reg_enable call
		 agdsp_access_enable implicitly, so
		 arch_audio_iis_to_audio_top_enable shuold not
		 call agdsp_access_enable repeatedly.
		*/
		arch_audio_codec_digital_reg_enable();
		ret = agdsp_access_enable();
		if (ret) {
			pr_err("%s, agdsp_access_enable failed!\n", __func__);
			return ret;
		}
		/* 2.aud dac src set 48k 0x0175000C bit[0] set 1 */
		ap_reg_update(REG_OFFSET(AUD_DAC_CTL) + virt_out,
			DAC_FS_MODE_48k << DAC_FS_MODE,
			DAC_FS_MODE_MASK << DAC_FS_MODE);
		/* aud adc src 0x01750018 bit[3:0] 0xC */
		ap_reg_update(REG_OFFSET(AUD_ADC_CTL) + virt_out,
			ADC_FS_MODE_48k << ADC_FS_MODE,
			ADC_SRC_N_MASK << ADC_FS_MODE);
		/* 3.select iis path 0x015E003C bit[0] set 1 */
		arch_audio_iis_to_audio_top_enable(AG_IIS0, 1);
		/*
		 *4.0 open aud dac path enable
		 * 0x01750000 bit[0] bit[2] set 1
		 */
		msk = BIT(DAC_EN_L) | BIT(DAC_EN_R);
		ap_reg_update(REG_OFFSET(AUD_TOP_CTL) + virt_out, msk, msk);
		/* LOOP */

		/*
		 * 4.1open aud adc path enable
		 * 0x01750000 bit[1] bit[3] set 1
		 */
		val = msk = BIT(ADC_EN_L) | BIT(ADC_EN_R);
		/*
		 * 5. open aud dac to adc loop function :
		 * 0x01750000 bit[9:8] set 1
		 */
		msk |= (ADC_SINC_SEL_MASK << ADC_SINC_SEL);
		val |= (ADC_SINC_SEL_DAC << ADC_SINC_SEL);
		ap_reg_update(REG_OFFSET(AUD_TOP_CTL) +
			virt_out, val, msk);
		/*
		 * match agdsp_access_disable called in
		 * arch_audio_codec_digital_reg_enable
		 */
		agdsp_access_disable();
		break;
	case AUD_IIS_MASTER_IIS0_DISABLE:
		/*
		 * I only close DAC_L DAC_R
		 * you should call agdsp_access_enable first
		 */
		agdsp_access_enable();
		msk = BIT(DAC_EN_L) | BIT(DAC_EN_R);
		ap_reg_update(REG_OFFSET(AUD_TOP_CTL) + virt_out, 0, msk);
		agdsp_access_disable();
		break;
	case AUD_IIS_MASTER_LOOP_DISABLE:
		/* I only close the bit for loop, not close ADC_L, ADC_R etc */
		agdsp_access_enable();
		msk = (ADC_SINC_SEL_MASK << ADC_SINC_SEL);
		ap_reg_update(REG_OFFSET(AUD_TOP_CTL) +
			virt_out, 0, msk);
		agdsp_access_disable();
		break;
	default:
		pr_err("%s unkonwn settings for audio digital iis0 and loop\n",
		       __func__);
		break;
	}

	return 0;
}

/* @@return < 0 failed, @@return ==0 success */
int vbc_enable_aud_digital_codec_iis_master(struct snd_soc_card *card,
					    int setting)
{
	unsigned long virt_out = 0;
	unsigned int phy_out = 0;
	int ret = 0;
	int codec_type = 0;

	sp_asoc_pr_dbg("%s setting: %d\n", __func__, setting);
	virt_out = sprd_get_codec_dp_base();
	if (virt_out == 0)
		/* it means using external codec */
		codec_type = 1;
	else
		codec_type = 0;

	if (codec_type == 0) {
		struct snd_ctl_elem_id id = {
			.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		};
		struct snd_kcontrol *kctrl;
		struct snd_ctl_elem_value ucontrol = {
			.value.integer.value[0] = !!(setting / 2),
		};

		if (setting == 0 || setting == 2) /* IIS0 */
			strcpy(id.name,
			       "Virt VBC Master Clock Mixer IIS0 Switch");
		else if (setting == 1 || setting == 3) /* Loop */
			strcpy(id.name,
			       "Virt VBC Master Clock Mixer Loop Switch");
		kctrl = snd_ctl_find_id(card->snd_card, &id);
		if (!kctrl) {
			pr_err("%s can't find kctrl '%s'\n", __func__, id.name);
			return -1;
		}

		return sprd_codec_virt_mclk_mixer_put(kctrl, &ucontrol);
	}

	if (codec_type == 1) {
		/* using external codec */
		ret = get_digital_codec_reg_base(&virt_out, &phy_out);
		if (ret < 0)
			return -1;
		aud_digital_iis0_loop_enable(setting, virt_out);
		return 0;
	}

	pr_err("%s unknown codec_type\n", __func__);

	return -1;
}

int vbc_phy_dt_parse_dig_codec_reg(struct device *dev)
{
	struct device_node *np;
	struct resource res;
	int ret;

	/* parse aud digital part regs */
	np = of_find_compatible_node(
		NULL, NULL, "sprd,whale-audio-codec");
	if (!np) {
		pr_err("ERR: [%s] there must be a", __func__);
		pr_err(" 'sprd,whale-audio-codec' node!\n");
		return -ENODEV;
	}

	ret = of_address_to_resource(np, 0, &res);
	if (!ret) {
		sprd_codec_dig_reg_base_v = (unsigned long)
			devm_ioremap_resource(dev, &res);
		if (IS_ERR_VALUE(sprd_codec_dig_reg_base_v)) {
			pr_err("ERR: cannot create iomap address for codec DP");
			return PTR_ERR((void *)sprd_codec_dig_reg_base_v);
		}
		sprd_codec_dig_reg_base_p = res.start;
	} else {
		sprd_codec_dig_reg_base_v = 0;
		sprd_codec_dig_reg_base_p = 0;
		pr_err("ERR:Must give me the 'sprd,whale-audio-codec' reg address!\n");
		return ret;
	}

	pr_info("%s vir: %#lx, phy: %#x\n",
		__func__, sprd_codec_dig_reg_base_v, sprd_codec_dig_reg_base_p);

	return 0;
}

u32 vbc_phy_ap2dsp(u32 addr)
{
	u32 ret_val = 0;

	if (addr >= vbc_phy_offset) {
		pr_debug("%s, addr=%#x, vbc_phy_offset:%#x\n",
			 __func__, addr, vbc_phy_offset);
		ret_val = addr - vbc_phy_offset;
		return ret_val;
	}
	pr_err("%s, addr=%#x, ret_val=%d\n", __func__, addr, ret_val);

	return ret_val;
}

int vbc_of_setup(struct platform_device *pdev)
{
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	struct regmap *agcp_ahb_gpr;
	struct pinctrl *pctrl;
	struct vbc_codec_priv *vbc_codec = NULL;
	int ret = 0;
	u32 val;

	if (!pdev) {
		pr_err("ERR: %s, pdev is NULL!\n", __func__);
		return -ENODEV;
	}
	vbc_codec = platform_get_drvdata(pdev);
	if (vbc_codec == NULL)
		pr_err("%s vbc_codec is null failed\n", __func__);

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

	/* Prepare for vbc registers accessing. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res != NULL) {
		sprd_ap_vbc_phy_base = (unsigned int)res->start;
		sprd_ap_vbc_virt_base = (unsigned long)
			devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR_VALUE(sprd_ap_vbc_virt_base)) {
			pr_err("ERR: cannot create iomap address for AP-VBC!\n");
			return -EINVAL;
		}
	} else {
		pr_err("ERR:Must give me the AP-VBC reg address!\n");
		return -EINVAL;
	}
	/* iis pin map */

	pctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pctrl))
		return -ENODEV;
	sp_asoc_pr_dbg("get pinctrl device!\n");
	vbc_codec->pctrl = pctrl;

	ret = of_property_read_u32(np, "sprd,vbc-phy-offset", &val);
	if (ret) {
		pr_err("ERR: %s :no property of 'reg'\n", __func__);
		return -EINVAL;
	}
	vbc_phy_offset = val;

	return 0;
}

int dsp_vbc_get_src_mode(unsigned int rate)
{
	int mode = -1;
	int i;
	struct sprd_codec_src_tbl {
		unsigned int rate;
		int src_mode;
	} src_tbl[] = {
		/* rate in Hz, AUD_AP_SRC_MODE_xxx */
		{48000, AUD_AP_SRC_MODE_48000},
		{44100, AUD_AP_SRC_MODE_44100},
		{32000, AUD_AP_SRC_MODE_32000},
		{24000, AUD_AP_SRC_MODE_24000},
		{22050, AUD_AP_SRC_MODE_22050},
		{16000, AUD_AP_SRC_MODE_16000},
		{12000, AUD_AP_SRC_MODE_12000},
		{11025, AUD_AP_SRC_MODE_11025},
		{8000, AUD_AP_SRC_MODE_8000},
	};

	for (i = 0; i < ARRAY_SIZE(src_tbl); i++) {
		if (src_tbl[i].rate == rate)
			mode = src_tbl[i].src_mode;
	}

	if (mode == -1) {
		sp_asoc_pr_info("%s, not supported samplerate (%d)\n",
				__func__, rate);
		mode = AUD_AP_SRC_MODE_48000;
	}

	return mode;
}

/*
 * Returns 1 for change, 0 for no change, or negative error code.
 */
int ap_vbc_reg_update(unsigned int reg, int val, int mask)
{
	unsigned int new, old;

	spin_lock(&vbc_lock);
	old = ap_vbc_reg_read(reg);
	new = (old & ~mask) | (val & mask);
	ap_vbc_phy_reg_raw_write(reg, new);
	spin_unlock(&vbc_lock);
	sp_asoc_pr_reg("[0x%04x] U:[0x%08x] R:[0x%08x]\n",
		reg & 0xFFFFFFFF, new,
		ap_vbc_reg_read(reg));

	return old != new;
}

int ap_vbc_reg_write(unsigned int reg, int val)
{
	spin_lock(&vbc_lock);
	ap_vbc_phy_reg_raw_write(reg, val);
	spin_unlock(&vbc_lock);
	sp_asoc_pr_reg("AP-VBC:[0x%04x] W:[0x%08x] R:[0x%08x]\n",
		(reg) & 0xFFFFFFFF, val,
		ap_vbc_reg_read(reg));

	return 0;
}

unsigned int dsp_vbc_reg_read(unsigned int reg)
{
	int ret = 0;
	struct sprd_vbc_kcontrol vbc_reg = {0};

	vbc_reg.reg = reg;
	vbc_reg.mask = 0xFFFFFFFF;
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_REG,
		-1, SND_VBC_DSP_IO_KCTL_GET,
		&vbc_reg, sizeof(struct sprd_vbc_kcontrol),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed, ret: %d\n", __func__, ret);

	return vbc_reg.value;
}

int dsp_vbc_reg_write(unsigned int reg, int val, unsigned int mask)
{
	spin_lock(&vbc_lock);
	dsp_vbc_phy_reg_raw_write(reg, val, mask);
	spin_unlock(&vbc_lock);
	sp_asoc_pr_dbg("%s reg=%x, value = %x, mask=%x\n", __func__,
		       reg, val, mask);

	return 0;
}

unsigned int ap_vbc_reg_read(unsigned int reg)
{
	unsigned int ret = 0;

	ret = readl_relaxed((void *__iomem)(reg - VBC_OFLD_ADDR +
		sprd_ap_vbc_virt_base));

	return ret;
}

void ap_vbc_src_clear(int stream)
{
	int shift = 0;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		shift = AUDPLY_AP_SRC_CLR;
	} else if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		shift = AUDRCD_AP_SRC_CLR;
	} else {
		pr_err("%s, wrong stream(%d)\n", __func__, stream);
		return;
	}
	ap_vbc_reg_update(VBC_AUD_CLR, (1 << shift),
			       (1 << shift));
}

void ap_vbc_aud_int_en(int stream, int en)
{
	int shift = 0;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		shift = AUDPLY_AP_INT_EN;
	} else if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		shift = AUDRCD_AP_INT_EN;
	} else {
		pr_err("%s, wrong stream(%d)\n", __func__, stream);
		return;
	}
	ap_vbc_reg_update(VBC_AUD_INT_EN, (1 << shift),
			       (1 << shift));
}

int ap_vbc_fifo_enable(int enable, int stream, int chan)
{
	sp_asoc_pr_dbg("ap_vbc_fifo_enable:enable %d, stream %d\n",
		enable, stream);
	if (enable) {
		ap_vbc_phy_fifo_enable_raw(1, stream, chan);
		sp_asoc_pr_dbg(" %s, %s-%s On\n", __func__,
			vbc_get_stream_name(stream),
			vbc_get_chan_name(chan));
	} else {
		ap_vbc_phy_fifo_enable_raw(0, stream, chan);
		sp_asoc_pr_dbg("%s, %s-%s Off\n",
			__func__, vbc_get_stream_name(stream),
			vbc_get_chan_name(chan));
	}

	return 0;
}

void ap_vbc_fifo_clear(int stream)
{
	/* clear audrcd data fifo */
	ap_vbc_phy_fifo_clear(stream);
}

void ap_vbc_aud_dat_format_set(int stream, enum VBC_DAT_FORMAT dat_fmt)
{
	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ap_vbc_reg_update(VBC_AUDPLY_FIFO_CTRL,
			(dat_fmt << AUDPLY_AP_DAT_FMT_CTL),
			(AUDPLY_AP_DAT_FMT_CTL_MASK << AUDPLY_AP_DAT_FMT_CTL));
	} else if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		ap_vbc_reg_update(VBC_AUDRCD_FIFO_CTRL,
			(dat_fmt << AUDRCD_AP_DAT_FMT_CTL),
			(AUDRCD_AP_DAT_FMT_CTL_MASK << AUDRCD_AP_DAT_FMT_CTL));
	}
}

void ap_vbc_aud_dma_chn_en(int enable, int stream, int chan)
{
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		vbc_phy_audply_dma_chn_en(enable, chan);
	else if (stream == SNDRV_PCM_STREAM_CAPTURE)
		vbc_phy_audrcd_dma_chn_en(enable, chan);
}

int ap_vbc_set_fifo_size(int stream, int full, int empty)
{
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		vbc_phy_audply_set_watermark(full, empty);
	else if (stream == SNDRV_PCM_STREAM_CAPTURE)
		vbc_phy_audrcd_set_watermark(full, empty);

	return 0;
}

void ap_vbc_src_set(int en, int stream, unsigned int rate)
{
	int mode = dsp_vbc_get_src_mode(rate);

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		vbc_phy_audply_set_src_mode(mode);
		vbc_phy_audply_src0_en(en ? 1:0);
		vbc_phy_audply_src1_en(en ? 1:0);
	} else if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		vbc_phy_audrcd_set_src_mode(mode);
		vbc_phy_audrcd_src0_en(en ? 1:0);
		vbc_phy_audrcd_src1_en(en ? 1:0);
	}
}
/* ap vbc phy driver end */

/* dsp vbc phy driver start */
int dsp_vbc_mdg_set(int id, int enable, int mdg_step)
{
	int ret = 0;
	struct vbc_mute_dg_para mdg_para = {0};

	mdg_para.mdg_id = id;
	mdg_para.mdg_mute = enable;
	mdg_para.mdg_step = mdg_step;
	sp_asoc_pr_dbg("%s mdg_para.mdg_id=%d\n", __func__,
		mdg_para.mdg_id);
	sp_asoc_pr_dbg("mdg_para.mdg_mute=%d\n", mdg_para.mdg_mute);
	sp_asoc_pr_dbg("mdg_para.mdg_step=%d\n", mdg_para.mdg_step);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		SND_KCTL_TYPE_MDG);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_MDG,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&mdg_para, sizeof(struct vbc_mute_dg_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_dg_set(int id, int dg_l, int dg_r)
{
	int ret = 0;
	struct vbc_dg_para dg_para = {0};

	dg_para.dg_id = id;
	dg_para.dg_left = dg_l;
	dg_para.dg_right = dg_r;
	sp_asoc_pr_dbg("%s dg_para.dg_id=%d\n", __func__,
		dg_para.dg_id);
	sp_asoc_pr_dbg("dg_para.dg_left=%d\n", dg_para.dg_left);
	sp_asoc_pr_dbg("dg_para.dg_right=%d\n", dg_para.dg_right);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		SND_KCTL_TYPE_DG);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_DG,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&dg_para, sizeof(struct vbc_dg_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_offload_dg_set(int id, int dg_l, int dg_r)
{
	int ret = 0;
	struct vbc_dg_para dg_para = {0};

	dg_para.dg_id = id;
	dg_para.dg_left = dg_l;
	dg_para.dg_right = dg_r;
	sp_asoc_pr_dbg("%s dg_para.dg_id=%d\n", __func__,
		dg_para.dg_id);
	sp_asoc_pr_dbg("dg_para.dg_left=%d\n", dg_para.dg_left);
	sp_asoc_pr_dbg("dg_para.dg_right=%d\n", dg_para.dg_right);
	sp_asoc_pr_dbg("cmd=%d, channel=%d parameter0=%d\n",
		SND_VBC_DSP_IO_KCTL_SET,
		AMSG_CH_MP3_OFFLOAD, SND_KCTL_TYPE_DG);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_DG,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&dg_para, sizeof(struct vbc_dg_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_smthdg_set(int id, int smthdg_l, int smthdg_r)
{
	int ret = 0;
	struct vbc_smthdg_para smthdg_para = {0};

	smthdg_para.smthdg_id = id;
	smthdg_para.smthdg_left = smthdg_l;
	smthdg_para.smthdg_right = smthdg_r;
	sp_asoc_pr_dbg("%s smthdg_para.smthdg_id = %d\n", __func__,
	smthdg_para.smthdg_id);
	sp_asoc_pr_dbg("smthdg_para.smthdg_left=%d\n", smthdg_para.smthdg_left);
	sp_asoc_pr_dbg("smthdg_para.smthdg_right=%d\n",
		       smthdg_para.smthdg_right);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		       SND_KCTL_TYPE_SMTHDG);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_SMTHDG,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&smthdg_para, sizeof(struct vbc_smthdg_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_smthdg_step_set(int id, int smthdg_step)
{
	int ret = 0;
	struct vbc_smthdg_step_para smthdg_step_para = {0};

	smthdg_step_para.smthdg_id = id;
	smthdg_step_para.smthdg_step = smthdg_step;
	sp_asoc_pr_dbg("%s smthdg_step_para.smthdg_id=%d\n", __func__,
		       smthdg_step_para.smthdg_id);
	sp_asoc_pr_dbg("smthdg_step_para.smthdg_step=%d\n",
		       smthdg_step_para.smthdg_step);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		       SND_KCTL_TYPE_SMTHDG_STEP);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_SMTHDG_STEP,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&smthdg_step_para, sizeof(struct vbc_smthdg_step_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_mixerdg_step_set(int mixerdg_step)
{
	int ret = 0;
	int16_t mixerdg_step_para = 0;

	mixerdg_step_para = mixerdg_step;
	sp_asoc_pr_dbg("%s mixerdg_step_para=%d\n", __func__,
		       mixerdg_step_para);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		       SND_KCTL_TYPE_MIXERDG_STEP);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_MIXERDG_STEP,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&mixerdg_step_para, sizeof(int16_t),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_mixerdg_mainpath_set(int id,
	int mixerdg_main_l, int mixerdg_main_r)
{
	int ret = 0;
	struct vbc_mixerdg_mainpath_para mixerdg_mainpath_para = {0};

	mixerdg_mainpath_para.mixerdg_id = id;
	mixerdg_mainpath_para.mixerdg_main_left = mixerdg_main_l;
	mixerdg_mainpath_para.mixerdg_main_right = mixerdg_main_r;

	sp_asoc_pr_dbg("%s mixerdg_mainpath_para.mixerdg_id= %d\n", __func__,
		mixerdg_mainpath_para.mixerdg_id);
	sp_asoc_pr_dbg("mixerdg_mainpath_para.mixerdg_main_left=%d\n",
		mixerdg_mainpath_para.mixerdg_main_left);
	sp_asoc_pr_dbg("mixerdg_mainpath_para.mixerdg_main_right=%d\n",
		mixerdg_mainpath_para.mixerdg_main_right);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		SND_KCTL_TYPE_MIXERDG_MAIN);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_MIXERDG_MAIN,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&mixerdg_mainpath_para,
		sizeof(struct vbc_mixerdg_mainpath_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_mixerdg_mixpath_set(int id,
	int mixerdg_mix_l, int mixerdg_mix_r)
{
	int ret = 0;
	struct vbc_mixerdg_mixpath_para mixerdg_mixpath_para = {0};

	mixerdg_mixpath_para.mixerdg_id = id;
	mixerdg_mixpath_para.mixerdg_mix_left = mixerdg_mix_l;
	mixerdg_mixpath_para.mixerdg_mix_right = mixerdg_mix_r;
	sp_asoc_pr_dbg("%s mixerdg_mixpath_para.mixerdg_id=%d\n", __func__,
		mixerdg_mixpath_para.mixerdg_id);
	sp_asoc_pr_dbg("mixerdg_mixpath_para.mixerdg_mix_left=%d\n",
		mixerdg_mixpath_para.mixerdg_mix_left);
	sp_asoc_pr_dbg("mixerdg_mixpath_para.mixerdg_mix_right=%d\n",
		mixerdg_mixpath_para.mixerdg_mix_right);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		SND_KCTL_TYPE_MIXERDG_MIX);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_MIXERDG_MIX,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&mixerdg_mixpath_para, sizeof(struct vbc_mixerdg_mixpath_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_mux_set(int id, int mux_sel)
{
	int ret = 0;
	struct vbc_mux_para mux_para = {0};

	mux_para.mux_id = id;
	mux_para.mux_sel = mux_sel;
	sp_asoc_pr_dbg("%s mux_id = %#x mux_sel=%#x\n", __func__,
		mux_para.mux_id, mux_para.mux_sel);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n\n", SND_VBC_DSP_IO_KCTL_SET,
		SND_KCTL_TYPE_MUX);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_MUX,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&mux_para, sizeof(struct vbc_mux_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_adder_set(int id,
	int adder_mode_l, int adder_mode_r)
{
	int ret = 0;
	struct vbc_adder_para adder_para = {0};

	adder_para.adder_id = id;
	adder_para.adder_mode_l = adder_mode_l;
	adder_para.adder_mode_r = adder_mode_r;
	sp_asoc_pr_dbg("%s,adder_para.adder_id=%#x\n", __func__,
		adder_para.adder_id);
	sp_asoc_pr_dbg("adder_para.adder_mode_l=%#x\n",
		adder_para.adder_mode_l);
	sp_asoc_pr_dbg("adder_para.adder_mode_r=%#x\n",
		adder_para.adder_mode_r);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		SND_KCTL_TYPE_ADDER);

	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_ADDER,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&adder_para, sizeof(struct vbc_adder_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}
/* dsp vbc phy driver end */

/* vbc driver function start */
int vbc_dsp_func_startup(struct device *dev, int id,
			int stream, const char *name)
{
	int ret;
	struct sprd_vbc_stream_startup_shutdown startup_info;

	memset(&startup_info, 0,
		sizeof(struct sprd_vbc_stream_startup_shutdown));
	/* fill the cmd para */
	to_stream_info(id, stream, name, &startup_info.stream_info);
	vbc_codec_getpathinfo(dev, &startup_info.startup_para, &g_vbc[id]);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, id, stream,
		SND_VBC_DSP_FUNC_STARTUP,
		&startup_info, sizeof(struct sprd_vbc_stream_startup_shutdown),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		return -EIO;

	return 0;
}

int pm_shutdown(void)
{
	struct sprd_vbc_stream_startup_shutdown shutdown_info;

	memset(&shutdown_info, 0,
		sizeof(struct sprd_vbc_stream_startup_shutdown));
	/* fill the cmd para */
	to_stream_info(VBC_DAI_ID_NORMAL_OUTDSP, SNDRV_PCM_STREAM_PLAYBACK,
		"normal-without-dsp", &shutdown_info.stream_info);
	shutdown_info.startup_para.dac_id = VBC_DA0;
	shutdown_info.startup_para.dac_iis_port =
		VBC_MUX_DAC0_IIS_SEL-VBC_MUX_START;
	shutdown_info.startup_para.dac_out_sel = 0;
	shutdown_info.startup_para.dac0_iis_port_for_voice = 0;
	shutdown_info.startup_para.dac_iis_width = 0;
	shutdown_info.startup_para.dac_iis_lr_mod = 0;
	return aud_send_cmd(AMSG_CH_VBC_CTL, VBC_DAI_ID_NORMAL_OUTDSP,
		SNDRV_PCM_STREAM_PLAYBACK, SND_VBC_DSP_FUNC_SHUTDOWN,
		&shutdown_info,
		sizeof(struct sprd_vbc_stream_startup_shutdown),
		AUDIO_SIPC_WAIT_FOREVER);
}

int vbc_dsp_func_shutdown(struct device *dev, int id,
			int stream, const char *name)
{
	int ret;
	struct sprd_vbc_stream_startup_shutdown shutdown_info;

	memset(&shutdown_info, 0,
		sizeof(struct sprd_vbc_stream_startup_shutdown));
	/* fill the cmd para */
	to_stream_info(id, stream, name, &shutdown_info.stream_info);
	vbc_codec_getpathinfo(dev, &shutdown_info.startup_para, &g_vbc[id]);
	/* send audio cmd */
	sp_asoc_pr_dbg("shutdown_info.startup_para.dac_id=%d\n",
		shutdown_info.startup_para.dac_id);
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, id, stream,
		SND_VBC_DSP_FUNC_SHUTDOWN, &shutdown_info,
		sizeof(struct sprd_vbc_stream_startup_shutdown),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		return -EIO;

	return 0;
}

int vbc_dsp_func_hwparam(struct device *dev, int id, int stream,
	const char *name, unsigned int channels,
	unsigned int format, unsigned int rate)
{
	int ret;
	struct sprd_vbc_stream_hw_paras stream_param;

	memset(&stream_param, 0, sizeof(struct sprd_vbc_stream_hw_paras));
	/* fill the cmd para */
	to_stream_info(id, stream, name, &stream_param.stream_info);
	stream_param.hw_params_info.channels = channels;
	stream_param.hw_params_info.format = format;
	stream_param.hw_params_info.rate = rate;
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, id, stream,
		SND_VBC_DSP_FUNC_HW_PARAMS, &stream_param,
		sizeof(struct sprd_vbc_stream_hw_paras),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		return -EIO;

	return 0;
}

int vbc_dsp_func_hw_free(struct device *dev, int id,
			int stream, const char *name)
{
	int ret;
	struct snd_pcm_stream_info stream_info;

	memset(&stream_info, 0, sizeof(struct snd_pcm_stream_info));
	/* fill the cmd para */
	to_stream_info(id, stream, name, &stream_info);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, id, stream,
		SND_VBC_DSP_FUNC_HW_FREE, &stream_info,
		sizeof(struct snd_pcm_stream_info),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		return -EIO;

	return 0;
}
int vbc_triggered_flag(int cmd)
{
	int32_t trigger_flag = false;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		trigger_flag = true;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		trigger_flag = false;
		break;
	default:
		trigger_flag = false;
	}

	return trigger_flag;
}

int vbc_dsp_func_trigger(struct device *dev, int id, int stream,
	const char *name, int cmd)
{
	int ret;
	/* send audio cmd */
	ret = aud_send_cmd_no_wait(AMSG_CH_VBC_CTL,
		SND_VBC_DSP_FUNC_HW_TRIGGER, id, stream,
		vbc_triggered_flag(cmd), 0);
	if (ret < 0)
		return -EIO;

	return 0;
}

int dsp_vbc_src_set(int id, int32_t fs)
{
	int ret = 0;
	struct vbc_src_para src_para = {0};

	src_para.src_id = id;
	src_para.fs = fs;
	sp_asoc_pr_dbg("%s, id=%d,fs=%d\n", __func__, id, fs);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		SND_KCTL_TYPE_SRC);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_SRC,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&src_para, sizeof(struct vbc_src_para),
		AUDIO_SIPC_WAIT_FOREVER);

	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_set_volume(struct snd_soc_codec *codec)
{
	int value = 0;
	int ret = 0;
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);

	value = vbc_codec->volume;
	sp_asoc_pr_dbg("%s value = %d", __func__, value);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d parameter1=%d\n",
		       SND_VBC_DSP_IO_KCTL_SET, SND_KCTL_TYPE_VOLUME, value);
	/* dsp will get it from parameter1 other than iram */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_VOLUME,
		value, SND_VBC_DSP_IO_KCTL_SET,
		&value, sizeof(value), AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_loopback_set(struct snd_soc_codec *codec)
{
	int ret = 0;
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_loopback_para loopback = {0};

	loopback = vbc_codec->loopback;
	sp_asoc_pr_dbg("%s loopback_type = %d,loopback\n",
		__func__, loopback.loopback_type);
	sp_asoc_pr_dbg("voice_fmt =%d, loopback.amr_rate =%d\n",
		loopback.voice_fmt, loopback.amr_rate);
	/* send audio cmd */
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		SND_KCTL_TYPE_LOOPBACK_TYPE);
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_LOOPBACK_TYPE,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&loopback, sizeof(struct vbc_loopback_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_dp_en_set(struct snd_soc_codec *codec, int id)
{
	int ret = 0;
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_dp_en_para dp_en = {0};

	dp_en.id = id;
	dp_en.enable = vbc_codec->vbc_dp_en[id];
	sp_asoc_pr_dbg("%s dp_en.id=%d dp_en.enable=%hu\n", __func__,
	dp_en.id, dp_en.enable);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		SND_KCTL_TYPE_DATAPATH);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_DATAPATH,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&dp_en, sizeof(struct vbc_dp_en_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_call_mute_set(struct snd_soc_codec *codec, int id)
{
	int ret = 0;
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	int kctrl_type = 0;

	if (id == VBC_UL_MUTE) {
		kctrl_type = SND_KCTL_TYPE_UL_MUTE;
	} else if (id == VBC_DL_MUTE) {
		kctrl_type = SND_KCTL_TYPE_DL_MUTE;
	} else {
		pr_err("%s failed unknown kctrl type kctrl_type =%d\n",
			__func__, kctrl_type);
		return -1;
	}

	/*
	 * smsg_prt->parameter1 = 1 ul mute,smsg_prt->parameter1 = 0 ul unmute
	 * smsg_prt->parameter1 = 1 dl mute,smsg_prt->parameter1 = 0 dl unmute
	 */
	sp_asoc_pr_dbg("%s kctrl_type =%d\n",
		__func__, kctrl_type);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d, parameter1 = %d\n",
		SND_VBC_DSP_IO_KCTL_SET, kctrl_type,
		vbc_codec->vbc_call_mute[id]);

	ret = aud_send_cmd(AMSG_CH_VBC_CTL, kctrl_type,
		vbc_codec->vbc_call_mute[id], SND_VBC_DSP_IO_KCTL_SET,
		&kctrl_type, sizeof(int), AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_iis_dac_width_set(struct snd_soc_codec *codec, int dac_id)
{
	int ret = 0;
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_iis_dac_width_sel_para dac_width = {0};

	dac_width.dac_id = dac_id;
	dac_width.dac_iis_width = vbc_codec->dac_iis_width[dac_id];
	sp_asoc_pr_dbg("%s dac_width.dac_id=%d dac_width.dac_iis_width = %d\n",
		__func__, dac_width.dac_id, dac_width.dac_iis_width);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		SND_KCTL_TYPE_DAC_IIS_WIDTH_SEL);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_DAC_IIS_WIDTH_SEL,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&dac_width, sizeof(struct vbc_iis_dac_width_sel_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_iis_adc_width_set(struct snd_soc_codec *codec, int adc_id)
{
	int ret = 0;
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_iis_adc_width_sel_para adc_width = {0};

	adc_width.adc_id = adc_id;
	adc_width.adc_iis_width = vbc_codec->adc_iis_width[adc_id];
	sp_asoc_pr_dbg("%s adc_width.ac_id=%d adc_width.adc_iis_width = %d\n",
		__func__, adc_width.adc_id, adc_width.adc_iis_width);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		SND_KCTL_TYPE_ADC_IIS_WIDTH_SEL);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_ADC_IIS_WIDTH_SEL,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&adc_width, sizeof(struct vbc_iis_adc_width_sel_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_iis_dac_lr_mod_set(struct snd_soc_codec *codec, int dac_id)
{
	int ret = 0;
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_iis_dac_lr_mod_sel_para dac_lr_mod = {0};

	dac_lr_mod.dac_id = dac_id;
	dac_lr_mod.dac_iis_lr_mod = vbc_codec->dac_iis_lr_mod[dac_id];
	pr_debug("%s dac_lr_mod.dac_id=%d dac_lr_mod.dac_iis_lr_mod = %d\n",
		       __func__, dac_lr_mod.dac_id, dac_lr_mod.dac_iis_lr_mod);
	sp_asoc_pr_dbg("cmd=%d, parameter0=%d\n", SND_VBC_DSP_IO_KCTL_SET,
		       SND_KCTL_TYPE_DAC_IIS_LR_MOD_SEL);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_DAC_IIS_LR_MOD_SEL,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&dac_lr_mod, sizeof(struct vbc_iis_dac_lr_mod_sel_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int dsp_vbc_iis_adc_lr_mod_set(struct snd_soc_codec *codec, int adc_id)
{
	int ret = 0;
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_iis_adc_lr_mod_sel_para adc_lr_mod = {0};

	adc_lr_mod.adc_id = adc_id;
	adc_lr_mod.adc_iis_lr_mod = vbc_codec->adc_iis_lr_mod[adc_id];
	pr_debug("%s adc_lr_mod.ac_id=%d adc_lr_mod.adc_iis_lr_mod = %d\n",
		       __func__, adc_lr_mod.adc_id, adc_lr_mod.adc_iis_lr_mod);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_ADC_IIS_LR_MOD_SEL,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&adc_lr_mod, sizeof(struct vbc_iis_adc_lr_mod_sel_para),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int vbc_try_vbc_mix_left_mux_sel_set(struct snd_soc_codec *codec,
	uint32_t id)
{
	int ret = 0;
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_dac_mixer_mux_sel dac_mixer_mux_sel = {0};

	dac_mixer_mux_sel.enable =
		vbc_codec->mix_left_mux_sel[id].enable;
	dac_mixer_mux_sel.id = id;
	dac_mixer_mux_sel.dac_mixer_mux_sel =
		vbc_codec->mix_left_mux_sel[id].dac_mixer_mux_sel;
	sp_asoc_pr_dbg("%s dac_mixer_mux_sel.enable= %u\n", __func__,
		dac_mixer_mux_sel.enable);
	sp_asoc_pr_dbg("dac_mixer_mux_sel.id= %u\n",
		dac_mixer_mux_sel.id);
	sp_asoc_pr_dbg("dac_mixer_mux_sel.dac_mixer_mux_sel= %#x\n",
		dac_mixer_mux_sel.dac_mixer_mux_sel);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_DAC_MIXER_L_MUX_SEL,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&dac_mixer_mux_sel, sizeof(struct vbc_dac_mixer_mux_sel),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int vbc_try_vbc_mix_right_mux_sel_set(struct snd_soc_codec *codec,
	uint32_t id)
{
	int ret = 0;
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_dac_mixer_mux_sel dac_mixer_mux_sel = {0};

	dac_mixer_mux_sel.enable =
		vbc_codec->mix_right_mux_sel[id].enable;
	dac_mixer_mux_sel.id = id;
	dac_mixer_mux_sel.dac_mixer_mux_sel =
		vbc_codec->mix_right_mux_sel[id].dac_mixer_mux_sel;
	sp_asoc_pr_dbg("%s dac_mixer_mux_sel.enable= %u\n", __func__,
		dac_mixer_mux_sel.enable);
	sp_asoc_pr_dbg("dac_mixer_mux_sel.id= %u\n",
		dac_mixer_mux_sel.id);
	sp_asoc_pr_dbg("dac_mixer_mux_sel.dac_mixer_mux_sel= %#x\n",
		dac_mixer_mux_sel.dac_mixer_mux_sel);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_DAC_MIXER_R_MUX_SEL,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&dac_mixer_mux_sel, sizeof(struct vbc_dac_mixer_mux_sel),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int vbc_try_vbc_mix_left_out_sel_set(struct snd_soc_codec *codec,
	uint32_t id)
{
	int ret = 0;
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_dac_mixer_out_sel dac_mixer_out_sel = {0};

	dac_mixer_out_sel.enable =
		vbc_codec->mix_left_out_sel[id].enable;
	dac_mixer_out_sel.id = id;
	dac_mixer_out_sel.dac_mixer_out_sel =
		vbc_codec->mix_left_out_sel[id].dac_mixer_out_sel;
	sp_asoc_pr_dbg("%s dac_mixer_out_sel.enable= %u\n", __func__,
		dac_mixer_out_sel.enable);
	sp_asoc_pr_dbg("dac_mixer_out_sel.id= %u\n",
		dac_mixer_out_sel.id);
	sp_asoc_pr_dbg("dac_mixer_out_sel.dac_mixer_out_sel= %#x\n",
		dac_mixer_out_sel.dac_mixer_out_sel);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_DAC_MIXER_L_OUT_SEL,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&dac_mixer_out_sel, sizeof(struct vbc_dac_mixer_mux_sel),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

int vbc_try_vbc_mix_right_out_sel_set(struct snd_soc_codec *codec,
	uint32_t id)
{
	int ret = 0;
	struct vbc_codec_priv *vbc_codec = snd_soc_codec_get_drvdata(codec);
	struct vbc_dac_mixer_out_sel dac_mixer_out_sel = {0};

	dac_mixer_out_sel.enable =
		vbc_codec->mix_right_out_sel[id].enable;
	dac_mixer_out_sel.id = id;
	dac_mixer_out_sel.dac_mixer_out_sel =
		vbc_codec->mix_right_out_sel[id].dac_mixer_out_sel;
	sp_asoc_pr_dbg("%s dac_mixer_out_sel.enable= %u\n", __func__,
		dac_mixer_out_sel.enable);
	sp_asoc_pr_dbg("dac_mixer_out_sel.id= %u\n",
		dac_mixer_out_sel.id);
	sp_asoc_pr_dbg("dac_mixer_out_sel.dac_mixer_out_sel= %#x\n",
		dac_mixer_out_sel.dac_mixer_out_sel);
	/* send audio cmd */
	ret = aud_send_cmd(AMSG_CH_VBC_CTL, SND_KCTL_TYPE_DAC_MIXER_R_OUT_SEL,
		-1, SND_VBC_DSP_IO_KCTL_SET,
		&dac_mixer_out_sel, sizeof(struct vbc_dac_mixer_mux_sel),
		AUDIO_SIPC_WAIT_FOREVER);
	if (ret < 0)
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);

	return 0;
}

/*
 * @return: 0, success; -1, error
 */
int dsp_fm_mute_by_set_dg(void)
{
	int dg_id = VBC_DG_FM - VBC_DG_START;
	int dg_l = 127;
	int dg_r = 127;
	int ret = 0;

	/* send audio cmd no receive*/
	ret = aud_send_use_noreplychan(
			SND_VBC_DSP_IO_KCTL_SET,
			SND_KCTL_TYPE_DG, dg_id, dg_l, dg_r);
	if (ret < 0) {
		pr_err("%s, Failed to set, ret: %d\n", __func__, ret);
		return -1;
	}
	pr_debug("%s\n", __func__);

	return 0;
}

/* vbc driver function end */
