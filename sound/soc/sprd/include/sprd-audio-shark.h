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

#ifndef __SPRD_AUDIO_SHARK_H
#define __SPRD_AUDIO_SHARK_H

#ifndef __SPRD_AUDIO_H
#error  "Don't include this file directly, include sprd-audio.h"
#endif

#define CODEC_DP_BASE		(0x1000)
#define VBC_BASE		(0x1000)
#define CODEC_AP_BASE		(0x2000)
#define CODEC_AP_OFFSET		(0)

#define CODEC_DP_BASE_DEFAULT		0x40000000
#define CODEC_DP_SIZE_DEFAULT		SZ_8K

#define VBC_BASE_DEFAULT			0x40020000
#define VBC_BASE_SIZE_DEFAULT		(SZ_4K + SZ_8K)

#define VBC_PHY_BASE		0

/* for CP2 */
#define VBC_CP2_PHY_BASE	(0x02020000)
#define CP2_PHYS_VBDA0		(VBC_CP2_PHY_BASE + 0x0000)
#define CP2_PHYS_VBDA1		(VBC_CP2_PHY_BASE + 0x0004)

/* TODO: move them to dts */
#define SPRD_IRAM_ALL_PHYS	  (0x50002000)
#define SPRD_IRAM_ALL_SIZE	  (SZ_16K + SZ_4K + SZ_2K + SZ_1K)

#define DMA_VB_DA0_AON		7
#define DMA_VB_DA1_AON		8

#define CLASS_G_LDO_ID			"vddclsg"

enum {
	AUDIO_NO_CHANGE,
	AUDIO_TO_CP0_DSP_CTRL,
	AUDIO_TO_CP1_DSP_CTRL,
	AUDIO_TO_AP_ARM_CTRL,
	AUDIO_TO_ARM_CTRL = AUDIO_TO_AP_ARM_CTRL,
	AUDIO_TO_CP0_ARM_CTRL,
	AUDIO_TO_CP1_ARM_CTRL,
	AUDIO_TO_CP2_ARM_CTRL,
};

/* vbc setting */
static inline int arch_audio_vbc_reg_enable(void)
{
	int ret = 0;

	aon_apb_gpr_null_check();
	aon_apb_reg_set(REG_AON_APB_APB_EB0, BIT_AON_APB_VBC_EB);

	return ret;
}

static inline int arch_audio_vbc_reg_disable(void)
{
	int ret = 0;

	aon_apb_gpr_null_check();
	aon_apb_reg_clr(REG_AON_APB_APB_EB0, BIT_AON_APB_VBC_EB);

	return ret;
}

static inline int arch_audio_vbc_enable(void)
{
	int ret = 0;

	return ret;
}

static inline int arch_audio_vbc_disable(void)
{
	int ret = 0;

	return ret;
}

static inline int arch_audio_vbc_reset(void)
{
	int ret = 0;

	aon_apb_gpr_null_check();
	aon_apb_reg_set(REG_AON_APB_APB_RST0, BIT_AON_APB_VBC_SOFT_RST);
	udelay(10);
	aon_apb_reg_clr(REG_AON_APB_APB_RST0, BIT_AON_APB_VBC_SOFT_RST);

	return ret;
}

static inline int arch_audio_vbc_switch(int master)
{
	int ret = 0;
	int val = 0;
	int mask = BIT_AON_APB_VBC_AFIFO_INT_SYS_SEL(3)
		| BIT_AON_APB_VBC_DA01_INT_SYS_SEL(3)
		| BIT_AON_APB_VBC_AD01_INT_SYS_SEL(3)
		| BIT_AON_APB_VBC_AD23_INT_SYS_SEL(3)
		| BIT_AON_APB_VBC_DA01_DMA_SYS_SEL(3)
		| BIT_AON_APB_VBC_AD01_DMA_SYS_SEL(3)
		| BIT_AON_APB_VBC_AD23_DMA_SYS_SEL(3);

	AON_APB_GPR_NULL_CHECK();

	if (master != AUDIO_NO_CHANGE) {
		aon_apb_reg_set(REG_AON_APB_CP0_WPROT_EN0, BIT(6));
		aon_apb_reg_set(REG_AON_APB_CP1_WPROT_EN0, BIT(6));
		/* changed by jian.chen
		 * aon_apb_reg_set(REG_AON_APB_CP2_WPROT_EN, BIT(6));
		 */
	}
	switch (master) {
	case AUDIO_TO_AP_ARM_CTRL:
		val = BIT_AON_APB_VBC_AFIFO_INT_SYS_SEL(0)
			| BIT_AON_APB_VBC_DA01_INT_SYS_SEL(0)
			| BIT_AON_APB_VBC_AD01_INT_SYS_SEL(0)
			| BIT_AON_APB_VBC_AD23_INT_SYS_SEL(0);
#ifndef CONFIG_SND_SOC_SPRD_AUDIO_USE_AON_DMA
		val |= (BIT_AON_APB_VBC_DA01_DMA_SYS_SEL(0)
			| BIT_AON_APB_VBC_AD01_DMA_SYS_SEL(0)
			| BIT_AON_APB_VBC_AD23_DMA_SYS_SEL(0));
#else
		/* Capture will only use AP DMA for the limitation
		 * of IRAM size
		 */
		val |= (BIT_AON_APB_VBC_DA01_DMA_SYS_SEL(2)
			| BIT_AON_APB_VBC_AD01_DMA_SYS_SEL(0)
			| BIT_AON_APB_VBC_AD23_DMA_SYS_SEL(0));
#endif
		aon_apb_reg_update(REG_AON_APB_VBC_CTRL, mask, val);
		aon_apb_reg_clr(REG_AON_APB_AP_WPROT_EN0, BIT(6));
		arch_audio_vbc_reset();
		arch_audio_vbc_reg_disable();
		break;
	case AUDIO_TO_CP0_DSP_CTRL:
		arch_audio_vbc_reset();
		arch_audio_vbc_reg_enable();
		aon_apb_reg_clr(REG_AON_APB_CP0_WPROT_EN0, BIT(6));
		val = BIT_AON_APB_VBC_AFIFO_INT_SYS_SEL(1)
			| BIT_AON_APB_VBC_DA01_INT_SYS_SEL(1)
			| BIT_AON_APB_VBC_AD01_INT_SYS_SEL(1)
			| BIT_AON_APB_VBC_AD23_INT_SYS_SEL(1)
			| BIT_AON_APB_VBC_DA01_DMA_SYS_SEL(1)
			| BIT_AON_APB_VBC_AD01_DMA_SYS_SEL(1)
			| BIT_AON_APB_VBC_AD23_DMA_SYS_SEL(1);
		aon_apb_reg_update(REG_AON_APB_VBC_CTRL, mask, val);
		aon_apb_reg_clr(REG_AON_APB_VBC_CTRL,
			(BIT_AON_APB_VBC_DMA_CP0_ARM_SEL |
			BIT_AON_APB_VBC_DMA_CP0_ARM_SEL));
		break;
	case AUDIO_TO_CP1_DSP_CTRL:
		arch_audio_vbc_reset();
		arch_audio_vbc_reg_enable();
		aon_apb_reg_clr(REG_AON_APB_CP1_WPROT_EN0, BIT(6));
		val = BIT_AON_APB_VBC_AFIFO_INT_SYS_SEL(2)
			| BIT_AON_APB_VBC_DA01_INT_SYS_SEL(2)
			| BIT_AON_APB_VBC_AD01_INT_SYS_SEL(2)
			| BIT_AON_APB_VBC_AD23_INT_SYS_SEL(2)
			| BIT_AON_APB_VBC_DA01_DMA_SYS_SEL(2)
			| BIT_AON_APB_VBC_AD01_DMA_SYS_SEL(2)
			| BIT_AON_APB_VBC_AD23_DMA_SYS_SEL(2);
		aon_apb_reg_update(REG_AON_APB_VBC_CTRL, mask, val);
		aon_apb_reg_clr(REG_AON_APB_VBC_CTRL,
			(BIT_AON_APB_VBC_DMA_CP1_ARM_SEL |
			BIT_AON_APB_VBC_DMA_CP1_ARM_SEL));
		break;
	case AUDIO_TO_CP0_ARM_CTRL:
		arch_audio_vbc_reset();
		arch_audio_vbc_reg_enable();
		aon_apb_reg_clr(REG_AON_APB_CP0_WPROT_EN0, BIT(6));
		val = BIT_AON_APB_VBC_AFIFO_INT_SYS_SEL(1)
			| BIT_AON_APB_VBC_DA01_INT_SYS_SEL(1)
			| BIT_AON_APB_VBC_AD01_INT_SYS_SEL(1)
			| BIT_AON_APB_VBC_AD23_INT_SYS_SEL(1)
			| BIT_AON_APB_VBC_DA01_DMA_SYS_SEL(1)
			| BIT_AON_APB_VBC_AD01_DMA_SYS_SEL(1)
			| BIT_AON_APB_VBC_AD23_DMA_SYS_SEL(1);
		aon_apb_reg_update(REG_AON_APB_VBC_CTRL,
			(mask | BIT_AON_APB_VBC_INT_CP0_ARM_SEL
			| BIT_AON_APB_VBC_DMA_CP0_ARM_SEL),
			(val | BIT_AON_APB_VBC_INT_CP0_ARM_SEL
			| BIT_AON_APB_VBC_DMA_CP0_ARM_SEL));
		break;
	case AUDIO_TO_CP1_ARM_CTRL:
		arch_audio_vbc_reset();
		arch_audio_vbc_reg_enable();
		aon_apb_reg_clr(REG_AON_APB_CP1_WPROT_EN0, BIT(6));
		val = BIT_AON_APB_VBC_AFIFO_INT_SYS_SEL(2)
			| BIT_AON_APB_VBC_DA01_INT_SYS_SEL(2)
			| BIT_AON_APB_VBC_AD01_INT_SYS_SEL(2)
			| BIT_AON_APB_VBC_AD23_INT_SYS_SEL(2)
			| BIT_AON_APB_VBC_DA01_DMA_SYS_SEL(2)
			| BIT_AON_APB_VBC_AD01_DMA_SYS_SEL(2)
			| BIT_AON_APB_VBC_AD23_DMA_SYS_SEL(2);
		aon_apb_reg_update(REG_AON_APB_VBC_CTRL,
			(mask | BIT_AON_APB_VBC_INT_CP1_ARM_SEL
			| BIT_AON_APB_VBC_DMA_CP1_ARM_SEL),
			(val | BIT_AON_APB_VBC_INT_CP1_ARM_SEL
			| BIT_AON_APB_VBC_DMA_CP1_ARM_SEL));
		break;
	case AUDIO_NO_CHANGE:
		ret = aon_apb_reg_read(REG_AON_APB_VBC_CTRL, &val);
		if (ret < 0)
			return ret;
		val &= BIT_AON_APB_VBC_DA01_INT_SYS_SEL(3);
		if (val == BIT_AON_APB_VBC_DA01_INT_SYS_SEL(0)) {
			ret = AUDIO_TO_AP_ARM_CTRL;
		} else if (val == BIT_AON_APB_VBC_DA01_INT_SYS_SEL(1)) {
			ret = aon_apb_reg_read(REG_AON_APB_VBC_CTRL, &val);
			if (ret < 0)
				return ret;
			if (val & BIT_AON_APB_VBC_INT_CP0_ARM_SEL)
				ret = AUDIO_TO_CP0_ARM_CTRL;
			else
				ret = AUDIO_TO_CP0_DSP_CTRL;
		} else if (val == BIT_AON_APB_VBC_DA01_INT_SYS_SEL(2)) {
			ret = aon_apb_reg_read(REG_AON_APB_VBC_CTRL, &val);
			if (val & BIT_AON_APB_VBC_INT_CP1_ARM_SEL)
				ret = AUDIO_TO_CP1_ARM_CTRL;
			else
				ret = AUDIO_TO_CP1_DSP_CTRL;
		} else if (val == BIT_AON_APB_VBC_DA01_INT_SYS_SEL(3)) {
			ret = AUDIO_TO_CP2_ARM_CTRL;
		}
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static inline int arch_audio_vbc_da_dma_info(int chan)
{
	int ret = 0;

	switch (chan) {
	case 0:
#ifndef CONFIG_SND_SOC_SPRD_AUDIO_USE_AON_DMA
		ret = DMA_VB_DA0;
#else
		ret = DMA_VB_DA0_AON;
#endif
		break;
	case 1:
#ifndef CONFIG_SND_SOC_SPRD_AUDIO_USE_AON_DMA
		ret = DMA_VB_DA1;
#else
		ret = DMA_VB_DA1_AON;
#endif
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static inline int arch_audio_vbc_ad_dma_info(int chan)
{
	int ret = 0;

	switch (chan) {
	case 0:
		ret = DMA_VB_AD0;
		break;
	case 1:
		ret = DMA_VB_AD1;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static inline int arch_audio_vbc_ad23_dma_info(int chan)
{
	int ret = 0;

	switch (chan) {
	case 0:
		ret = DMA_VB_AD2;
		break;
	case 1:
		ret = DMA_VB_AD3;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

/* ----------------------------------------------- */
/* Codec digital part in soc setting */
static inline int arch_audio_codec_audif_enable(int auto_clk)
{
	int ret = 0;

	aon_apb_gpr_null_check();
	if (auto_clk) {
		aon_apb_reg_clr(REG_AON_APB_APB_EB0,
			BIT_AON_APB_AUDIF_EB);
		aon_apb_reg_set(REG_AON_APB_VBC_CTRL,
			BIT_AON_APB_AUDIF_CKG_AUTO_EN);
	} else {
		aon_apb_reg_set(REG_AON_APB_APB_EB0,
			BIT_AON_APB_AUDIF_EB);
		aon_apb_reg_clr(REG_AON_APB_VBC_CTRL,
			BIT_AON_APB_AUDIF_CKG_AUTO_EN);
	}

	return ret;
}

static inline int arch_audio_codec_audif_disable(void)
{
	int ret = 0;

	aon_apb_gpr_null_check();
	aon_apb_reg_clr(REG_AON_APB_APB_EB0,
		BIT_AON_APB_AUDIF_EB);
	aon_apb_reg_clr(REG_AON_APB_VBC_CTRL,
		BIT_AON_APB_AUDIF_CKG_AUTO_EN);

	return ret;
}

static inline int arch_audio_codec_digital_reg_enable(void)
{
	int ret = 0;

	aon_apb_gpr_null_check();
	ret = aon_apb_reg_set(REG_AON_APB_APB_EB0, BIT_AON_APB_AUD_EB);
	if (ret >= 0)
		arch_audio_codec_audif_enable(0);

	return ret;
}

static inline int arch_audio_codec_digital_reg_disable(void)
{
	int ret = 0;

	aon_apb_gpr_null_check();
	arch_audio_codec_audif_disable();
	aon_apb_reg_clr(REG_AON_APB_APB_EB0, BIT_AON_APB_AUD_EB);

	return ret;
}

static inline int arch_audio_codec_digital_enable(void)
{
	int ret = 0;

	aon_apb_gpr_null_check();
	/* internal digital 26M enable */
	aon_apb_reg_set(REG_AON_APB_SINDRV_CTRL, BIT_AON_APB_SINDRV_ENA);

	return ret;
}

static inline int arch_audio_codec_digital_disable(void)
{
	int ret = 0;

	aon_apb_gpr_null_check();
	/* internal digital 26M disable */
	aon_apb_reg_clr(REG_AON_APB_SINDRV_CTRL, BIT_AON_APB_SINDRV_ENA);

	return ret;
}

static inline int arch_audio_codec_switch(int master)
{
	int ret = 0;
	unsigned int msk, val;

	aon_apb_gpr_null_check();

	msk = BIT_AON_APB_AUD_INT_SYS_SEL(3);
	switch (master) {
	case AUDIO_TO_AP_ARM_CTRL:
		ret = aon_apb_reg_update(REG_AON_APB_VBC_CTRL, msk,
			BIT_AON_APB_AUD_INT_SYS_SEL(0));
		break;
	case AUDIO_TO_CP0_ARM_CTRL:
		ret = aon_apb_reg_update(REG_AON_APB_VBC_CTRL, msk,
			BIT_AON_APB_AUD_INT_SYS_SEL(1));
		break;
	case AUDIO_TO_CP1_ARM_CTRL:
		ret = aon_apb_reg_update(REG_AON_APB_VBC_CTRL, msk,
			BIT_AON_APB_AUD_INT_SYS_SEL(2));
		break;
	case AUDIO_TO_CP2_ARM_CTRL:
		ret = aon_apb_reg_update(REG_AON_APB_VBC_CTRL, msk,
			BIT_AON_APB_AUD_INT_SYS_SEL(3));
		break;
	case AUDIO_NO_CHANGE:
		ret = aon_apb_reg_read(REG_AON_APB_VBC_CTRL, &val);
		if (ret < 0)
			return ret;
		val &= msk;
		if (val == BIT_AON_APB_AUD_INT_SYS_SEL(0))
			ret = AUDIO_TO_AP_ARM_CTRL;
		else if (val == BIT_AON_APB_AUD_INT_SYS_SEL(1))
			ret = AUDIO_TO_CP0_ARM_CTRL;
		else if (val == BIT_AON_APB_AUD_INT_SYS_SEL(2))
			ret = AUDIO_TO_CP1_ARM_CTRL;
		else if (val == BIT_AON_APB_AUD_INT_SYS_SEL(3))
			ret = AUDIO_TO_CP2_ARM_CTRL;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static inline int arch_audio_codec_switch2ap(void)
{
	return arch_audio_codec_switch(AUDIO_TO_AP_ARM_CTRL);
}

static inline int arch_audio_codec_digital_reset(void)
{
	int ret = 0;

	aon_apb_gpr_null_check();
	aon_apb_reg_set(REG_AON_APB_APB_RST0, BIT_AON_APB_AUD_SOFT_RST);
	aon_apb_reg_set(REG_AON_APB_APB_RST0, BIT_AON_APB_AUDIF_SOFT_RST);
	udelay(10);
	aon_apb_reg_clr(REG_AON_APB_APB_RST0, BIT_AON_APB_AUD_SOFT_RST);
	aon_apb_reg_clr(REG_AON_APB_APB_RST0, BIT_AON_APB_AUDIF_SOFT_RST);

	return ret;
}

static inline int arch_audio_sleep_xtl_enable(void)
{
	pmu_apb_gpr_null_check();
	pmu_apb_reg_set(REG_PMU_APB_SLEEP_XTLON_CTRL,
		BIT_PMU_APB_AP_SLEEP_XTL_ON);

	return 0;
}
static inline int arch_audio_sleep_xtl_disable(void)
{

	pmu_apb_gpr_null_check();
	pmu_apb_reg_clr(REG_PMU_APB_SLEEP_XTLON_CTRL,
		BIT_PMU_APB_AP_SLEEP_XTL_ON);

	return 0;
}

static inline int arch_audio_memory_sleep_enable(void)
{
	return 0;
}

static inline int arch_audio_memory_sleep_disable(void)
{
	return 0;
}
#endif/* __SPRD_AUDIO_SHARK_H */
