/*
 * sound/soc/sprd/codec/sprd/sc2731/sprd-codec.h
 *
 * SPRD-CODEC -- SpreadTrum Tiger intergrated codec.
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
#ifndef __SPRD_CODEC_H
#define __SPRD_CODEC_H

#include <linux/io.h>
#include "sprd-audio.h"
#include "sprd-audio-sc2731.h"

/* unit: ms */
#define SPRD_CODEC_LDO_WAIT_TIME	(5)
#define SPRD_CODEC_LDO_VCM_TIME		(2)
#define SPRD_CODEC_DAC_MUTE_TIMEOUT	(600)

#define SPRD_CODEC_HP_POP_TIMEOUT	(1000)

#define SPRD_CODEC_INFO	(AUDIO_CODEC_2723)

#define SPRD_CODEC_RATE_8000   (10)
#define SPRD_CODEC_RATE_9600   (9)
#define SPRD_CODEC_RATE_11025  (8)
#define SPRD_CODEC_RATE_12000  (7)
#define SPRD_CODEC_RATE_16000  (6)
#define SPRD_CODEC_RATE_22050  (5)
#define SPRD_CODEC_RATE_24000  (4)
#define SPRD_CODEC_RATE_32000  (3)
#define SPRD_CODEC_RATE_44100  (2)
#define SPRD_CODEC_RATE_48000  (1)
#define SPRD_CODEC_RATE_96000  (0)

/* AUD_TOP_CTL */
#define DAC_EN_L		(0)
#define ADC_EN_L		(1)
#define DAC_EN_R		(2)
#define ADC_EN_R		(3)
#define ADC_SINC_SEL		(8)
#define ADC_SINC_SEL_ADC	(0)
#define ADC_SINC_SEL_DAC	(1)
#define ADC_SINC_SEL_D		(2)
#define ADC_SINC_SEL_MASK	(0x3)
#define ADC1_EN_L		(10)
#define ADC1_EN_R		(11)
#define ADC1_SINC_SEL		(14)
#define ADC1_SINC_SEL_ADC	(0)
#define ADC1_SINC_SEL_DAC	(1)
#define ADC1_SINC_SEL_D		(2)
#define ADC1_SINC_SEL_MASK	(0x3)

/* AUD_I2S_CTL */
#define ADC_LR_SEL	(2)
#define DAC_LR_SEL	(1)

/* AUD_DAC_CTL */
#define DAC_MUTE_EN		(15)
#define DAC_MUTE_CTL		(14)
#define DAC_FS_MODE		(0)
#define DAC_FS_MODE_96k		(0)
#define DAC_FS_MODE_48k		(1)
#define DAC_FS_MODE_MASK	(0xf)

/* AUD_ADC_CTL */
#define ADC_SRC_N		(0)
#define ADC_SRC_N_MASK		(0xf)
#define ADC1_SRC_N		(4)
#define ADC1_SRC_N_MASK		(0xf)

#define ADC_FS_MODE_48k		(0xc)
#define ADC_FS_MODE		(0)

/* AUD_LOOP_CTL */
#define AUD_LOOP_TEST		(0)
#define LOOP_ADC_PATH_SEL	(9)
#define LOOP_PATH_SEL		(1)
#define LOOP_PATH_SEL_MASK	(0x3)
/* AUD_AUD_STS0 */
#define DAC_MUTE_U_MASK		(5)
#define DAC_MUTE_D_MASK		(4)
#define DAC_MUTE_U_RAW		(3)
#define DAC_MUTE_D_RAW		(2)
#define DAC_MUTE_ST		(0)
#define DAC_MUTE_ST_MASK	(0x3)

/* AUD_INT_CLR */
/* AUD_INT_EN */
#define DAC_MUTE_U		(1)
#define DAC_MUTE_D		(0)

/*AUD_DMIC_CTL*/
#define ADC_DMIC_CLK_MODE	(0)
#define ADC_DMIC_CLK_MODE_MASK	(0x3)
#define ADC_DMIC_LR_SEL		(2)
#define ADC1_DMIC_CLK_MODE	(3)
#define ADC1_DMIC_CLK_MODE_MASK	(0x3)
#define ADC1_DMIC_LR_SEL	(5)
#define ADC_DMIC_EN		(6)
#define ADC1_DMIC1_EN		(7)

/* AUD_ADC1_I2S_CTL */
#define ADC1_LR_SEL	(0)

/*DAC_SDM_DC_L*/
#define DAC_SDM_DC_L		(0)
#define DAC_SDM_DC_L_MASK	(0xffff)
/*DAC_SDM_DC_H*/
#define DAC_SDM_DC_R		(0)
#define DAC_SDM_DC_R_MASK	(0xff)

/*VB_V*/
#define LDO_V_3000		(4)
#define LDO_V_3025		(5)
#define LDO_V_3050		(6)
#define LDO_V_3075		(7)
#define LDO_V_3100		(8)
#define LDO_V_3125		(9)
#define LDO_V_3150		(10)
#define LDO_V_3175		(11)
#define LDO_V_3200		(12)
#define LDO_V_3225		(13)
#define LDO_V_3250		(14)
#define LDO_V_3275		(15)
#define LDO_V_3300		(16)
#define LDO_V_3325		(17)
#define LDO_V_3350		(18)
#define LDO_V_3375		(19)
#define LDO_V_3400		(20)
#define LDO_V_3425		(21)
#define LDO_V_3450		(22)
#define LDO_V_3475		(23)
#define LDO_V_3500		(24)
#define LDO_V_3525		(25)
#define LDO_V_3550		(26)
#define LDO_V_3575		(27)
#define LDO_V_3600		(28)

#define MIC_LDO_V_21		(0)
#define MIC_LDO_V_19		(1)
#define MIC_LDO_V_23		(2)
#define MIC_LDO_V_25		(3)

/*bits definitions for register ANA_PMU0*/
#define VB_EN			(15)
#define VB_NLEAK_PD		(14)
#define VB_EADBIAS_CTRL		(13)
#define BG_IBIAS_EN		(12)
#define VCM_EN			(11)
#define VCM_BUF_EN		(10)
#define SEL_VCMI		(9)
#define VCMI_FAST_EN		(8)
#define MIC1_BIAS_EN		(7)
#define MIC2_BIAS_EN		(6)
#define HMIC_BIAS_EN		(5)
#define HMIC_SLEEP_EN		(3)
#define HMIC_SLEEP_EN_MASK	(3)
#define CP_EN			(1)
#define CP_EN_DIS		(0)
#define CP_EN_POS		(1)
#define CP_EN_NEG		(2)
#define CP_EN_POS_NEG		(3)
#define CP_EN_MASK		(3)
#define VB_HP_EN		(0)

/*bits definitions for register ANA_PMU1*/
#define CP_REG			(14)
#define CP_DISCHG_PD		(13)
#define CP_CLK_SEL		(12)
#define CP_OSC			(9)
#define CP_OSC_MASK		(0x7)
#define CP_SS_EN		(8)
#define CP_SS_RST		(7)
#define CP_SS_F			(5)
#define CP_SS_F_MASK		(3)
#define CP_BYPASS		(4)
#define CP_LDO_EADBIAS_CTRL	(3)
#define CP_QUICK_PD		(2)
#define CP_BW			(0)
#define CP_BW_MASK		(3)
#define CP_BW_40K		(0)
#define CP_BW_60K		(1)
#define CP_BW_100K		(2)
#define CP_BW_150K		(3)

/*bits definitions for register ANA_PMU2*/

/*bits definitions for register ANA_PMU3*/
#define VB_V			(9)
#define VB_V_MASK		(0x1f)
#define VREFP_V			(6)
#define VREFP_V_MASK		(0x7)
#define VREFP_V_1P25		(0)
#define VREFP_V_1P30		(1)
#define VREFP_V_1P35		(2)
#define VREFP_V_1P40		(3)
#define VREFP_V_1P45		(4)
#define VREFP_V_1P50		(5)
#define VREFP_V_1P55		(6)
#define VREFP_V_1P60		(7)
#define VDACS_V			(3)
#define VDACS_V_MASK		(0x7)
#define VDACS_V_1P25		(0)
#define VDACS_V_1P30		(1)
#define VDACS_V_1P35		(2)
#define VDACS_V_1P40		(3)
#define VDACS_V_1P45		(4)
#define VDACS_V_1P50		(5)
#define VDACS_V_1P55		(6)
#define VDACS_V_1P60		(7)
#define VSPKAB_V		(0)
#define VSPKAB_V_1P45		(0)
#define VSPKAB_V_1P50		(1)
#define VSPKAB_V_1P55		(2)
#define VSPKAB_V_1P60		(3)
#define VSPKAB_V_1P65		(4)
#define VSPKAB_V_1P70		(5)
#define VSPKAB_V_1P75		(6)
#define VSPKAB_V_1P80		(7)
#define VSPKAB_V_MASK		(0x7)

/*bits definitions for register ANA_PMU4*/

/*bits definitions for register ANA_PMU5*/
#define MIC_BIAS_V		(4)
#define MIC_BIAS_V_MASK		(0xf)
#define HMIC_BIAS_V		(0)
#define HMIC_BIAS_V_MASK	(0xf)

/*bits definitions for register ANA_PMU6*/
#define DALR_LP_EN		(7)
#define DAS_LP_EN		(6)
#define CP_LP_EN		(4)
#define CP_LP_EN_MASK		(0x3)
#define CP_LP_EN_DIS		(0)
#define CP_LP_EN_POS		(1)
#define CP_LP_EN_NEG		(2)
#define CP_LP_EN_POS_NEG	(3)

/*bits definitions for register ANA_PMU7*/
#define RCV_I			(4)
#define RCV_I_MASK		(0x3)
#define RCV_I_0P5X		(0)
#define RCV_I_0P75X		(1)
#define RCV_I_1X		(2)
#define RCV_I_1P25X		(3)
#define RCV_IF			(2)
#define RCV_IF_MASK		(0x3)

/*bits definitions for register ANA_PMU8*/

/*bits definitions for register ANA_PMU9*/

/*bits definitions for register ANA_PMU10*/
#define PA_OVP_PD		(5)
#define PA_OVP_VREFIN_EN	(4)
#define PA_OVP_V		(0)
#define PA_OVP_V_MASK		(7)
#define PA_OVP_V_5P8		(0)
#define PA_OVP_V_6P0		(1)
#define PA_OVP_V_6P2		(2)
#define PA_OVP_V_6P4		(3)
#define PA_OVP_V_6P6		(4)
#define PA_OVP_V_6P8		(5)
#define PA_OVP_V_7P0		(6)
#define PA_OVP_V_7P2		(7)

/*bits definitions for register ANA_PMU11*/
#define DRV_OCP_PD_SPKR		(7) /*Reserved for SPKR & AOR*/
#define DRV_OCP_PD_SPKL		(6)
#define DRV_OCP_PD_EAR		(5)
#define DRV_OCP_PD_HP		(4)

/*bits definitions for register ANA_CLK0*/
#define DIG_CLK_6P5M_EN		(14)
#define ANA_CLK_EN		(12)
#define DCDCGEN_CLK_EN		(7)
#define DCDCMEM_CLK_EN		(6)
#define DCDCCORE_CLK_EN		(5)
#define AD_CLK_EN		(11)
#define AD_CLK_RST		(10)
#define DA_CLK_EN		(9)
#define CP_CLK_EN		(8)
#define DCDCCHG_CLK_EN		(4)
#define DRV_CLK_EN		(3)
#define IMPD_CLK_EN		(2)

/*bits definitions for register ANA_CLK1*/

/*bits definitions for register ANA_CLK2*/

/*bits definitions for register ANA_CDC0*/
#define ADPGA_IBUF_EN			(15)
#define ADPGAL_EN			(13)
#define ADPGAR_EN			(12)
#define ADPGAR_BYP			(8)
#define ADPGAR_BYP_MASK			(3)
#define ADPGAR_BYP_NORMAL		(0)
#define ADPGAR_BYP_HDST2ADC		(1)
#define ADPGAR_BYP_ALL_DISCON		(2)
#define ADL_EN				(7)
#define ADL_RST				(6)
#define ADR_EN				(5)
#define ADR_RST				(4)

/*bits definitions for register ANA_CDC1*/
#define DAL_EN			(15)
#define DAR_EN			(14)
#define DAS_EN			(13)
#define DALR_OS_EN		(9)
#define DAS_OS_EN		(8)
#define DALR_OS_D		(4)
#define DALR_OS_D_MASK		(0xf)
#define DALR_OS_D_0		(0)
#define DALR_OS_D_4		(4)
#define DAS_OS_D		(0)
#define DAS_OS_D_MASK		(0xf)
#define DAS_OS_D_0		(0)
#define DAS_OS_D_4		(4)

/*bits definitions for register ANA_CDC2*/
#define HPL_EN		(3)
#define HPR_EN		(2)
#define HPBUF_EN	(1)
#define RCV_EN		(0)

/*bits definitions for register ANA_CDC3*/
#define PA_EN			(15)
#define PA_D_EN			(14)
#define PA_DTRI_F		(9)
#define PA_DTRI_F_MASK		(0x7)

/*bits definitions for register ANA_CDC4*/
#define PA_DEMI_PD		(11)
#define PA_EMI_L		(8)
#define PA_EMI_L_MASK		(0x7)

/*bits definitions for register ANA_CDC5*/
#define SMIC1PGAL		(10)
#define SHMICPGAL		(9)
#define SLINL_PGAL		(8)
#define SLINR_PGAL		(7)
#define SMIC2PGAR		(6)
#define SHMICPGAR		(5)
#define SLINL_PGAR		(4)
#define SLINR_PGAR		(3)

/*bits definitions for register ANA_CDC6*/
#define SDALHPL			(9)
#define SDARHPR			(8)
#define SDALRCV			(7)
#define RCV_IDAC_S		(6)
#define SDAPA			(5)
#define SLINPA			(4)
#define SLINLHPL		(3)
#define SLINRHPR		(2)

/*bits definitions for register ANA_CDC7*/
#define PMIC1			(12)
#define PMIC1_MASK		(3)
#define PMIC2			(10)
#define PMIC2_MASK		(3)
#define PHMIC			(8)
#define PHMIC_MASK		(3)
#define PLIN			(6)
#define PLIN_MASK		(3)
#define ADPGAL_G		(3)
#define ADPGAL_G_MASK		(0x7)
#define ADPGAR_G		(0)
#define ADPGAR_G_MASK		(0x7)

/*bits definitions for register ANA_CDC8*/
#define DALR_IG			(4)
#define DALR_IG_MASK		(3)
#define DALR_IG_MINUS_2P09DB	(3)
#define DALR_IG_MINUS_1P45DB	(2)
#define DALR_IG_MINUS_0P75DB	(1)
#define DALR_IG_0DB		(0)
#define DAD_RG			(0)
#define DAD_RG_MASK		(3)
#define DAD_RG_MINUS_2P22DB	(3)
#define DAD_RG_MINUS_1P53DB	(2)
#define DAD_RG_MINUS_0P73DB	(1)
#define DAD_RG_0DB		(0)

/*bits definitions for register ANA_CDC9*/
#define SPK_G		(12)
#define SPK_G_MASK	(0x7)
#define EAR_G		(8)
#define EAR_G_MUTE	(0xf)
#define EAR_G_MASK	(0xf)
#define HPL_G		(4)
#define HPL_G_MUTE	(0xf)
#define HPL_G_MASK	(0xf)
#define HPR_G		(0)
#define HPR_G_MUTE	(0xf)
#define HPR_G_MASK	(0xf)

/*bits definitions for register ANA_CDC10*/
#define NG_PA_EN		(3)
#define HP_DPOP_EN		(2)
#define RCV_DPOP_EN		(0)

/*bits definitions for register ANA_HDT0*/
#define HEAD_INS_PD		(15)
#define HEAD_L_INT_PU_PD	(12)
#define HEAD_INS_VREF		(9)
#define HEAD_INS_VREF_MASK	(0x3)
#define HEAD_INS_VREF_2P6	(0)
#define HEAD_INS_VREF_2P3	(1)
#define HEAD_INS_VREF_2P0	(2)
#define HEAD_INS_VREF_1P7	(3)
#define BUT_DET_PD		(8)
#define HEAD_SDET		(5)
#define HEAD_SDET_MASK		(0x3)
#define HEAD_SDET_2P7		(0)
#define HEAD_SDET_2P5		(1)
#define HEAD_SDET_2P3		(2)
#define HEAD_SDET_2P1		(3)
#define HEAD_SBUT		(0)
#define HEAD_SBUT_MASK		(0xf)

/*bits definitions for register ANA_HDT1*/

/*bits definitions for register ANA_HDT2*/

/*bits definitions for register ANA_HDT3*/
#define V2AD_EN				(9)
#define V2AD_SEL			(4)
#define V2AD_SEL_MASK			(0xf)
#define V2AD_SEL_DISABLE		(0)
#define V2AD_SEL_HEADMIC_IN_DET		(1)
#define V2AD_SEL_HEADSET_L_INT		(4)

/*bits definitions for register ANA_HDT4*/

/*bits definitions for register ANA_HDT6*/
#define HEDET_INTRES_SEL	(2)
#define HEDET_INTRES_SEL_MASK	(0x3)
#define HEDET_INTRES_SEL_30K	(3)

/*bits definitions for register ANA_DCL0*/
#define DCL_EN		(11)
#define DCL_RST		(10)
#define DRV_SOFT_EN	(4)
#define DRV_CPDLY_PD	(0)

/*bits definitions for register ANA_DCL1*/

/*bits definitions for register ANA_DCL2*/

/*bits definitions for register ANA_DCL3*/
#define CP_POS_HV		(7)
#define CP_POS_HV_MAX		(187)
#define CP_POS_HV_MASK		(0x7f)
#define CP_POS_LV		(0)
#define CP_POS_LV_MAX		(187)
#define CP_POS_LV_MASK		(0x7f)
#define CP_POS_VAL(x)		(CP_POS_HV_MAX - (x))/* x unit: 10mV */

/*bits definitions for register ANA_DCL4*/
#define CP_NEG_HV		(6)
#define CP_NEG_HV_MIN		(-1850)
#define CP_NEG_HV_MASK		(0x3f)
#define CP_NEG_LV		(0)
#define CP_NEG_LV_MIN		(-1850)
#define CP_NEG_LV_MASK		(0x3f)
#define CP_NEG_VAL(x)		(((x) - CP_NEG_HV_MIN) / 25)/* x unit: 1mV */

/*bits definitions for register ANA_DCL5*/

/*bits definitions for register ANA_DCL6*/
#define CP_OSCBW_AUTO_EN	(10)

/*bits definitions for register ANA_DCL7*/
#define DPOP_AUTO_EN		(13)
#define HP_DPOP_S		(10)
#define HP_DPOP_S_MASK		(0x3)
#define HP_DPOP_S_DIS		(0)
#define HP_DPOP_S_VCM_BUF	(1)
#define HP_DPOP_S_HPL		(2)
#define HP_DPOP_S_HPR		(3)
#define HP_DPOP_S_BYPASS	(1)
#define HP_DPOP_S_NORMAL	(3)
#define HP_DPOP_T1		(8)
#define HP_DPOP_T1_MASK		(0x3)
#define HP_DPOP_T1_60US		(0)
#define HP_DPOP_T1_120US	(1)
#define HP_DPOP_T1_240US	(2)
#define HP_DPOP_T2		(6)
#define HP_DPOP_T2_MASK		(0x3)
#define HP_DPOP_T2_120US	(0)
#define HP_DPOP_T2_240US	(1)
#define HP_DPOP_T2_480US	(2)
#define HP_DPOP_TH		(4)
#define HP_DPOP_TH_MASK		(0x3)
#define HP_DPOP_TH_2		(0)
#define HP_DPOP_TH_4		(1)
#define HP_DPOP_TH_8		(2)
#define HP_DPOP_TH_16		(3)
#define HP_DPOP_TO		(2)
#define HP_DPOP_TO_MASK		(0x3)
#define HP_DPOP_TO_2		(0)
#define HP_DPOP_TO_4		(1)
#define HP_DPOP_TO_8		(2)
#define HP_DPOP_TO_16		(3)
#define HP_DPOP_AVG		(0)
#define HP_DPOP_AVG_MASK	(0x3)
#define HP_DPOP_AVG_4		(0)
#define HP_DPOP_AVG_8		(1)
#define HP_DPOP_AVG_16		(2)
#define HP_DPOP_AVG_32		(3)

/*bits definitions for register ANA_DCL8*/
#define HPL_DPOP_SW_HOLD	(13)
#define HPR_DPOP_SW_HOLD	(11)
#define HP_DPOP_GAIN_EN		(10)
#define HP_DPOP_GAIN_N1		(6)
#define HP_DPOP_GAIN_N1_MASK	(0x7)
#define HP_DPOP_GAIN_N1_16	(0x4)
#define HP_DPOP_GAIN_N2		(3)
#define HP_DPOP_GAIN_N2_MASK	(0x7)
#define HP_DPOP_GAIN_N2_1	(0)
#define HP_DPOP_GAIN_T		(0)
#define HP_DPOP_GAIN_T_MASK	(0x7)
#define HP_DPOP_GAIN_T_120US	(0x2)
#define HP_DPOP_GAIN_T_500US	(0x4)

/*bits definitions for register ANA_DCL8*/

/*bits definitions for register ANA_DCL9*/

/*bits definitions for register ANA_DCL10*/

/*bits definitions for register ANA_DCL12*/
#define RCV_DPOP_T		(0)
#define RCV_DPOP_T_MASK		(0x7)
#define RCV_DPOP_T_30US		(0x0)
#define RCV_DPOP_T_60US		(0x1)
#define RCV_DPOP_T_120US	(0x2)
#define RCV_DPOP_T_250US	(0x3)
#define RCV_DPOP_T_500US	(0x4)
#define RCV_DPOP_T_1000US	(0x5)
#define RCV_DPOP_T_2000US	(0x6)
#define RCV_DPOP_T_4000US	(0x7)

/*bits definitions for register ANA_STS0*/
#define HEAD_INSERT		(12)
#define HEAD_INSERT_2		(11)
#define HPBUF_DPOP_STS		(8)
#define HPBUF_DPOP_STS_START	(1)
#define HPBUF_DPOP_STS_FINISH	(2)
#define HPBUF_DPOP_STS_MASK	(0x3)
#define HPL_DPOP_DVLD		(3)
#define HPR_DPOP_DVLD		(2)
#define HP_DPOP_DVLD		(1)
#define HP_DPOP_DVLD_MASK	(1)
#define RCV_DPOP_DVLD		(0)

/*bits definitions for register ANA_STS1*/

/*bits definitions for register ANA_STS2*/
#define PA_OVP_FLAG		(9)

/*bits definitions for register ANA_STS3*/

/*bits definitions for register ANA_STS4*/

#define SPRD_CODEC_DP_BASE	(CODEC_DP_BASE)
#define AUD_TOP_CTL		(SPRD_CODEC_DP_BASE + 0x0000)
#define AUD_AUD_CTR		(SPRD_CODEC_DP_BASE + 0x0004)
#define AUD_I2S_CTL		(SPRD_CODEC_DP_BASE + 0x0008)
#define AUD_DAC_CTL		(SPRD_CODEC_DP_BASE + 0x000C)
#define AUD_SDM_CTL0		(SPRD_CODEC_DP_BASE + 0x0010)
#define AUD_SDM_CTL1		(SPRD_CODEC_DP_BASE + 0x0014)
#define AUD_ADC_CTL		(SPRD_CODEC_DP_BASE + 0x0018)
#define AUD_LOOP_CTL		(SPRD_CODEC_DP_BASE + 0x001C)
#define AUD_AUD_STS0		(SPRD_CODEC_DP_BASE + 0x0020)
#define AUD_INT_CLR		(SPRD_CODEC_DP_BASE + 0x0024)
#define AUD_INT_EN		(SPRD_CODEC_DP_BASE + 0x0028)
#define AUDIF_FIFO_CTL		(SPRD_CODEC_DP_BASE + 0x002C)
#define AUD_DMIC_CTL		(SPRD_CODEC_DP_BASE + 0x0030)
#define AUD_ADC1_I2S_CTL	(SPRD_CODEC_DP_BASE + 0x0034)
#define AUD_DAC_SDM_L		(SPRD_CODEC_DP_BASE + 0x0038)
#define AUD_DAC_SDM_H		(SPRD_CODEC_DP_BASE + 0x003C)
#define SPRD_CODEC_DP_END	(SPRD_CODEC_DP_BASE + 0x0040)
#define IS_SPRD_CODEC_DP_RANG(reg) (((reg) >= SPRD_CODEC_DP_BASE) \
				    && ((reg) < SPRD_CODEC_DP_END))

#define SPRD_CODEC_AP_BASE	(CODEC_AP_BASE)
#define ANA_PMU0		(SPRD_CODEC_AP_BASE + 0x0000)
#define ANA_PMU1		(SPRD_CODEC_AP_BASE + 0x0004)
#define ANA_PMU2		(SPRD_CODEC_AP_BASE + 0x0008)
#define ANA_PMU3		(SPRD_CODEC_AP_BASE + 0x000C)
#define ANA_PMU4		(SPRD_CODEC_AP_BASE + 0x0010)
#define ANA_PMU5		(SPRD_CODEC_AP_BASE + 0x0014)
#define ANA_PMU6		(SPRD_CODEC_AP_BASE + 0x0018)
#define ANA_PMU7		(SPRD_CODEC_AP_BASE + 0x001C)
#define ANA_PMU8		(SPRD_CODEC_AP_BASE + 0x0020)
#define ANA_PMU9		(SPRD_CODEC_AP_BASE + 0x0024)
#define ANA_PMU10		(SPRD_CODEC_AP_BASE + 0x0028)
#define ANA_PMU11		(SPRD_CODEC_AP_BASE + 0x002C)
#define ANA_CLK0		(SPRD_CODEC_AP_BASE + 0x0038)
#define ANA_CLK1		(SPRD_CODEC_AP_BASE + 0x003C)
#define ANA_CLK2		(SPRD_CODEC_AP_BASE + 0x0040)
#define ANA_CDC0		(SPRD_CODEC_AP_BASE + 0x004C)
#define ANA_CDC1		(SPRD_CODEC_AP_BASE + 0x0050)
#define ANA_CDC2		(SPRD_CODEC_AP_BASE + 0x0054)
#define ANA_CDC3		(SPRD_CODEC_AP_BASE + 0x0058)
#define ANA_CDC4		(SPRD_CODEC_AP_BASE + 0x005C)
#define ANA_CDC5		(SPRD_CODEC_AP_BASE + 0x0060)
#define ANA_CDC6		(SPRD_CODEC_AP_BASE + 0x0064)
#define ANA_CDC7		(SPRD_CODEC_AP_BASE + 0x0068)
#define ANA_CDC8		(SPRD_CODEC_AP_BASE + 0x006C)
#define ANA_CDC9		(SPRD_CODEC_AP_BASE + 0x0070)
#define ANA_CDC10		(SPRD_CODEC_AP_BASE + 0x0074)
#define ANA_HDT6		(SPRD_CODEC_AP_BASE + 0x007C)
#define ANA_HDT0		(SPRD_CODEC_AP_BASE + 0x0080)
#define ANA_HDT1		(SPRD_CODEC_AP_BASE + 0x0084)
#define ANA_HDT2		(SPRD_CODEC_AP_BASE + 0x0088)
#define ANA_HDT3		(SPRD_CODEC_AP_BASE + 0x008C)
#define ANA_HDT4		(SPRD_CODEC_AP_BASE + 0x0090)
#define ANA_HDT5		(SPRD_CODEC_AP_BASE + 0x0094)
#define ANA_DCL0		(SPRD_CODEC_AP_BASE + 0x0098)
#define ANA_DCL1		(SPRD_CODEC_AP_BASE + 0x009C)
#define ANA_DCL2		(SPRD_CODEC_AP_BASE + 0x00A0)
#define ANA_DCL3		(SPRD_CODEC_AP_BASE + 0x00A4)
#define ANA_DCL4		(SPRD_CODEC_AP_BASE + 0x00A8)
#define ANA_DCL5		(SPRD_CODEC_AP_BASE + 0x00AC)
#define ANA_DCL6		(SPRD_CODEC_AP_BASE + 0x00B0)
#define ANA_DCL7		(SPRD_CODEC_AP_BASE + 0x00B4)
#define ANA_DCL8		(SPRD_CODEC_AP_BASE + 0x00B8)
#define ANA_DCL9		(SPRD_CODEC_AP_BASE + 0x00BC)
#define ANA_DCL10		(SPRD_CODEC_AP_BASE + 0x00C0)
#define ANA_DCL11		(SPRD_CODEC_AP_BASE + 0x00C4)
#define ANA_DCL12		(SPRD_CODEC_AP_BASE + 0x00C8)
#define ANA_STS0		(SPRD_CODEC_AP_BASE + 0x00E0)
#define ANA_STS1		(SPRD_CODEC_AP_BASE + 0x00E4)
#define ANA_STS2		(SPRD_CODEC_AP_BASE + 0x00E8)
#define ANA_STS3		(SPRD_CODEC_AP_BASE + 0x00EC)
#define ANA_STS4		(SPRD_CODEC_AP_BASE + 0x00F0)
#define SPRD_CODEC_AP_ANA_END	(SPRD_CODEC_AP_BASE + 0x00F4)

#define AUD_CFGA_REG_BASE		(CODEC_AP_BASE + 0x100)
#define AUD_CFGA_CLR			(AUD_CFGA_REG_BASE + 0x0000)
#define AUD_CFGA_HBD_CFG1		(AUD_CFGA_REG_BASE + 0x0004)
#define AUD_CFGA_HBD_CFG2		(AUD_CFGA_REG_BASE + 0x0008)
#define AUD_CFGA_HBD_CFG3		(AUD_CFGA_REG_BASE + 0x000C)
#define AUD_CFGA_HBD_CFG4		(AUD_CFGA_REG_BASE + 0x0010)
#define AUD_CFGA_HBD_CFG5		(AUD_CFGA_REG_BASE + 0x0014)
#define AUD_CFGA_HBD_CFG6		(AUD_CFGA_REG_BASE + 0x0018)
#define AUD_CFGA_HBD_CFG7		(AUD_CFGA_REG_BASE + 0x001C)
#define AUD_CFGA_HBD_CFG8		(AUD_CFGA_REG_BASE + 0x0020)
#define AUD_CFGA_HBD_CFG9		(AUD_CFGA_REG_BASE + 0x0024)
#define AUD_CFGA_HBD_CFG10		(AUD_CFGA_REG_BASE + 0x0028)
#define AUD_CFGA_HBD_CFG11		(AUD_CFGA_REG_BASE + 0x002C)
#define AUD_CFGA_HBD_CFG12		(AUD_CFGA_REG_BASE + 0x0030)
#define AUD_CFGA_HBD_CFG13		(AUD_CFGA_REG_BASE + 0x0034)
#define AUD_CFGA_HBD_CFG14		(AUD_CFGA_REG_BASE + 0x0038)
#define AUD_CFGA_HBD_CFG15		(AUD_CFGA_REG_BASE + 0x003C)
#define AUD_CFGA_HBD_CFG16		(AUD_CFGA_REG_BASE + 0x0040)
#define AUD_CFGA_HBD_STS0		(AUD_CFGA_REG_BASE + 0x0044)
#define AUD_CFGA_HBD_STS1		(AUD_CFGA_REG_BASE + 0x0048)
#define AUD_CFGA_HBD_STS2		(AUD_CFGA_REG_BASE + 0x004C)
#define AUD_CFGA_HBD_STS3		(AUD_CFGA_REG_BASE + 0x0050)
#define AUD_CFGA_HID_CFG0		(AUD_CFGA_REG_BASE + 0x0080)
#define AUD_CFGA_HID_CFG1		(AUD_CFGA_REG_BASE + 0x0084)
#define AUD_CFGA_HID_CFG2		(AUD_CFGA_REG_BASE + 0x0088)
#define AUD_CFGA_HID_CFG3		(AUD_CFGA_REG_BASE + 0x008C)
#define AUD_CFGA_HID_CFG4		(AUD_CFGA_REG_BASE + 0x0090)
#define AUD_CFGA_HID_CFG5		(AUD_CFGA_REG_BASE + 0x0094)
#define AUD_CFGA_HID_STS0		(AUD_CFGA_REG_BASE + 0x0098)
#define AUD_CFGA_PRT_CFG_0		(AUD_CFGA_REG_BASE + 0x009C)
#define AUD_CFGA_PRT_CFG_1		(AUD_CFGA_REG_BASE + 0x00A0)
#define AUD_CFGA_RD_STS			(AUD_CFGA_REG_BASE + 0x00A4)
#define AUD_CFGA_INT_MODULE_CTRL	(AUD_CFGA_REG_BASE + 0x00A8)
#define AUD_CFGA_LP_MODULE_CTRL		(AUD_CFGA_REG_BASE + 0x00AC)
#define AUD_CFGA_ANA_ET1		(AUD_CFGA_REG_BASE + 0x00B0)
#define AUD_CFGA_ANA_ET2		(AUD_CFGA_REG_BASE + 0x00B4)
#define AUD_CFGA_ANA_ET3		(AUD_CFGA_REG_BASE + 0x00B8)
#define AUD_CFGA_ANA_ET4		(AUD_CFGA_REG_BASE + 0x00BC)
#define AUD_CFGA_IMPD_CTRL		(AUD_CFGA_REG_BASE + 0x00C0)
#define AUD_CFGA_IMPD_DATA		(AUD_CFGA_REG_BASE + 0x00C4)
#define AUD_CFGA_IMPD_CHARGE_INT_CLR	(AUD_CFGA_REG_BASE + 0x00C8)
#define AUD_CFGA_IMPD_STS		(AUD_CFGA_REG_BASE + 0x00CC)
#define AUD_CFGA_CLK_EN			(AUD_CFGA_REG_BASE + 0x00D0)
#define AUD_CFGA_SOFT_RST		(AUD_CFGA_REG_BASE + 0x00D4)
#define SPRD_CODEC_AP_END		(AUD_CFGA_REG_BASE + 0x00D8)
#define IS_SPRD_CODEC_AP_RANG(reg)	(((reg) >= SPRD_CODEC_AP_BASE) \
					 && ((reg) < SPRD_CODEC_AP_END))

/*bits definitions for register AUD_CFGA_CLR*/
#define AUD_A_INT_CLR		(4)
#define AUD_A_INT_CLR_MASK	(0x7F)

/*bits definitions for register AUD_CFGA_HBD_CFG1*/

/*bits definitions for register AUD_CFGA_HBD_CFG2*/

/*bits definitions for register AUD_CFGA_HBD_CFG3*/

/*bits definitions for register AUD_CFGA_HBD_CFG4*/

/*bits definitions for register AUD_CFGA_HBD_CFG5*/

/*bits definitions for register AUD_CFGA_HBD_CFG6*/

/*bits definitions for register AUD_CFGA_HBD_CFG7*/

/*bits definitions for register AUD_CFGA_HBD_CFG8*/

/*bits definitions for register AUD_CFGA_HBD_CFG9*/

/*bits definitions for register AUD_CFGA_HBD_CFG10*/

/*bits definitions for register AUD_CFGA_HBD_CFG11*/

/*bits definitions for register AUD_CFGA_HBD_CFG12*/

/*bits definitions for register AUD_CFGA_HBD_CFG13*/

/*bits definitions for register AUD_CFGA_HBD_CFG14*/

/*bits definitions for register AUD_CFGA_HBD_CFG15*/

/*bits definitions for register AUD_CFGA_HBD_CFG16*/

/*bits definitions for register AUD_CFGA_HBD_STS0*/

/*bits definitions for register AUD_CFGA_HBD_STS1*/

/*bits definitions for register AUD_CFGA_HBD_STS2*/

/*bits definitions for register AUD_CFGA_HBD_STS3*/

/*bits definitions for register AUD_CFGA_HID_CFG0*/

/*bits definitions for register AUD_CFGA_HID_CFG1*/

/*bits definitions for register AUD_CFGA_HID_CFG2*/

/*bits definitions for register AUD_CFGA_HID_CFG3*/

/*bits definitions for register AUD_CFGA_HID_CFG4*/

/*bits definitions for register AUD_CFGA_HID_CFG5*/

/*bits definitions for register AUD_CFGA_HID_STS0*/

/*bits definitions for register AUD_CFGA_PRT_CFG_0*/

/*bits definitions for register AUD_CFGA_PRT_CFG_1*/

/*bits definitions for register AUD_CFGA_RD_STS*/
#define AUD_IRQ_MSK		(0)
#define AUD_IRQ_MSK_MASK	(0x7F)

/*bits definitions for register AUD_CFGA_INT_MODULE_CTRL*/
#define AUD_A_INT_EN		(0)
#define AUD_A_INT_EN_MASK	(0x7F)

#define AUDIO_PACAL_IRQ		(6)
#define AUDIO_HP_DPOP_IRQ	(5)
#define OVP_IRQ			(4)
#define OTP_IRQ			(3)
#define PA_OCP_IRQ		(2)
#define EAR_OCP_IRQ		(1)
#define HP_OCP_IRQ		(0)

/*bits definitions for register AUD_CFGA_LP_MODULE_CTRL*/
#define AUDIFA_ADCR_EN		(5)
#define AUDIFA_DACR_EN		(4)
#define AUDIFA_ADCL_EN		(3)
#define AUDIFA_DACL_EN		(2)
#define AUDIO_ADIE_LOOP_EN	(0)

/*bits definitions for register ANA_ET1*/
#define AUD_DA0_DC		(7)
#define AUD_DA0_DC_MASK		(0x7f)
#define AUD_DA1_DC		(0)
#define AUD_DA1_DC_MASK		(0x7f)
#define BITS_AUD_DA0_DC(_X_)	(((_X_)&0x7f)<<7)
#define BITS_AUD_DA1_DC(_X_)	((_X_)&0x7f)

/*bits definitions for register ANA_ET2*/
#define AUD_ET_EN		(8)

/*bits definitions for register ANA_ET3*/
#define BITS_AUD_ET_HOLD_MS(_X_)	((_X_)&0xffff)

/*bits definitions for register ANA_ET4*/
#define BITS_AUD_ET_MAX_SEL(_X_)	(((_X_)&0x3)<<14)
#define BITS_AUD_ET_MAX_SET(_X_)	(((_X_)&0x7f)<<7)
#define BITS_AUD_ET_VTRIG(_X_)		((_X_)&0x7f)

/*bits definitions for register AUD_IMPD_CTRL*/

/*bits definitions for register AUD_IMPD_DATA*/

/*bits definitions for register AUD_IMPD_CHARGE_INT_CLR*/

/*bits definitions for register AUD_IMPD_STS*/

/*bits definitions for register CLK_EN*/
#define CLK_AUD_6P5M_EN		(6)
#define CLK_AUD_LOOP_EN		(5)
#define CLK_AUD_HBD_EN		(4)
#define CLK_AUD_HID_EN		(3)
#define CLK_AUD_1K_EN		(2)
#define CLK_AUD_32K_EN		(0)

/*bits definitions for register SOFT_RST*/
#define DAC_POST_SOFT_RST	(7)
#define DIG_6P5M_SOFT_RST	(6)
#define AUD_1K_SOFT_RST		(4)
#define AUD_32K_SOFT_RST	(0)

#define SPRD_CODEC_IIS1_ID 111

unsigned long sprd_get_codec_dp_base(void);

struct snd_kcontrol;
struct snd_ctl_elem_value;
int sprd_codec_virt_mclk_mixer_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol);
#endif /* __SPRD_CODEC_H */
