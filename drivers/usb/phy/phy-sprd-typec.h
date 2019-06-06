/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 */
#ifndef _PHY_SPRD_TYPEC_H_
#define _PHY_SPRD_TYPEC_H_


#define TYPEC_REG(n)			(tc->reg_offset + (n))

#ifdef CONFIG_PMIC_SC2723
/* TODO: */
#endif

#ifdef CONFIG_PMIC_SC2731
#define TYPEC_MODULE_EN			ANA_REG_GLB_MODULE_EN1
#define TYPEC_MODULE_RST		ANA_REG_GLB_SOFT_RST1
#define TYPEC_RTC_EN			ANA_REG_GLB_RTC_CLK_EN1

#define TYPEC_MODULE_EN_BIT		BIT_ANA_TYPEC_EN
#define TYPEC_SOFT_RST_BIT		BIT_TYPEC_SOFT_RST
#define TYPEC_RTC_EN_BIT		BIT_RTC_TYPEC_EN


/* registers definitions for controller REGS_TYPEC */

#define TYPEC_CONFIG			TYPEC_REG(0x00)
#define TYPEC_INT_CLR			TYPEC_REG(0x04)
#define TYPEC_MODE				TYPEC_REG(0x08)
#define TYPEC_TCCDE_CNT			TYPEC_REG(0x0c)
#define TYPEC_TPDDE_CNT			TYPEC_REG(0x10)
#define TYPEC_INT_RAW			TYPEC_REG(0x14)
#define TYPEC_INT_MASK			TYPEC_REG(0x18)
#define TYPEC_STATUS			TYPEC_REG(0x1c)
#define TYPEC_TOGGLE_CNT		TYPEC_REG(0x20)
#define TYPEC_SLEEP_CNT1		TYPEC_REG(0x24)
#define TYPEC_SLEEP_CNT2		TYPEC_REG(0x28)
#define TYPEC_ITRIM				TYPEC_REG(0x2c)
#define TYPEC_RTRIM				TYPEC_REG(0x30)
#define TYPEC_VREF				TYPEC_REG(0x34)
#define TYPEC_TERR_CNT			TYPEC_REG(0x38)
#define TYPEC_TDRP_CNT			TYPEC_REG(0x3c)
#define TYPEC_INT_EN			TYPEC_CONFIG

/* TYPEC_CONFIG */
#define TYPEC_ENABLE				BIT(0)
#define TYPEC_TOGGLE_SLEEP_EN		BIT(1)
#define TYPEC_ERR_STS_EN			BIT(2)
#define TYPEC_DIS_STS_EN			BIT(3)
#define TYPEC_USB20_ONLY			BIT(4)
#define TYPEC_ATTACH_INT_EN			BIT(5)
#define TYPEC_DETACH_INT_EN			BIT(6)
#define TYPEC_IN_ERR_STS_INT_EN		BIT(7)
#define TYPEC_OUT_ERR_STS_INT_EN	BIT(8)
#define TYPEC_DIS_STS_INT_EN		BIT(9)

/* TYPEC_INT_CLR */
#define TYPEC_ATTACH_INT_CLR		BIT(0)
#define TYPEC_DETACH_INT_CLR		BIT(1)

/* TYPEC_INT_MASK */
#define TYPEC_ATTACH_INT			BIT(0)
#define TYPEC_DETACH_INT			BIT(1)

/* TYPEC_MODE */
#define TYPEC_MODE_SNK	0
#define TYPEC_MODE_SRC	1
#define TYPEC_MODE_DRP	2
#define TYPEC_MODE_MASK	3

/* TYPEC_CC_STATUS */
#define TYPEC_CC_CHANNEL_MASK		BIT(4)

#define TYPEC_CC_CHANNEL_DET_REG	TYPEC_STATUS
#define TYPEC_CC_CHANNEL_DET_MASK	GENMASK(8, 5)
#define TYPEC_CC1_CHANNEL_DET		0x20
#define TYPEC_CC1_CHANNEL_HIGH		0x60
#define TYPEC_CC2_CHANNEL_DET		0x80
#define TYPEC_CC2_CHANNEL_HIGH		0x180

#define TYPEC_STATE_MASK			GENMASK(3, 0)

/* TYPEC_TYPEC_RTRIM */
#define TYPEC_CC_RTRIM_MASK		GENMASK(4, 0)
#define TYPEC_CC1_RTRIM_INDEX	0
#define TYPEC_CC2_RTRIM_INDEX	5

#define EFUSE_BITSINDEX(b, o)	((b) * 16 + (o))
#define EFUSE_CC1_INDEX			EFUSE_BITSINDEX(13, 6)
#define EFUSE_CC1_LENGTH		5
#define EFUSE_CC2_INDEX			EFUSE_BITSINDEX(13, 11)
#define EFUSE_CC2_LENGTH		5

#endif

#ifdef CONFIG_PMIC_SC2721
#define TYPEC_MODULE_EN			ANA_REG_GLB_MODULE_EN0
#define TYPEC_MODULE_RST		ANA_REG_GLB_SOFT_RST1
#define TYPEC_RTC_EN			ANA_REG_GLB_RTC_CLK_EN0

#define TYPEC_MODULE_EN_BIT		BIT_ANA_TYPEC_EN
#define TYPEC_SOFT_RST_BIT		BIT_TYPEC_SOFT_RST
#define TYPEC_RTC_EN_BIT		BIT_RTC_TYPEC_EN

/* registers definitions for controller REGS_TYPEC */

#define TYPEC_CONFIG			TYPEC_REG(0x00)
#define TYPEC_INT_CLR			TYPEC_REG(0x04)
#define TYPEC_MODE				TYPEC_REG(0x08)
#define TYPEC_TCCDE_CNT			TYPEC_REG(0x0c)
#define TYPEC_TPDDE_CNT			TYPEC_REG(0x10)
#define TYPEC_INT_RAW			TYPEC_REG(0x14)
#define TYPEC_INT_MASK			TYPEC_REG(0x18)
#define TYPEC_STATUS			TYPEC_REG(0x1c)
#define TYPEC_TOGGLE_CNT		TYPEC_REG(0x20)
#define TYPEC_SLEEP_CNT1		TYPEC_REG(0x24)
#define TYPEC_SLEEP_CNT2		TYPEC_REG(0x28)
#define TYPEC_DRPTRY_CNT		TYPEC_REG(0x2c)
#define TYPEC_RTRIM				TYPEC_REG(0x30)
#define TYPEC_VREF				TYPEC_REG(0x34)
#define TYPEC_TERR_CNT			TYPEC_REG(0x38)
#define TYPEC_TDRP_CNT			TYPEC_REG(0x3c)
#define TYPEC_ITRIM				TYPEC_REG(0x40)
#define TYPEC_INT_EN			TYPEC_CONFIG

/* TYPEC_CONFIG */
#define TYPEC_ENABLE				BIT(0)
#define TYPEC_TOGGLE_SLEEP_EN		BIT(1)
#define TYPEC_ERR_STS_EN			BIT(2)
#define TYPEC_DIS_STS_EN			BIT(3)
#define TYPEC_USB20_ONLY			BIT(4)
#define TYPEC_ATTACH_INT_EN			BIT(5)
#define TYPEC_DETACH_INT_EN			BIT(6)
#define TYPEC_IN_ERR_STS_INT_EN		BIT(7)
#define TYPEC_OUT_ERR_STS_INT_EN	BIT(8)
#define TYPEC_DIS_STS_INT_EN		BIT(9)

/* TYPEC_INT_CLR */
#define TYPEC_ATTACH_INT_CLR		BIT(0)
#define TYPEC_DETACH_INT_CLR		BIT(1)

/* TYPEC_INT_MASK */
#define TYPEC_ATTACH_INT			BIT(0)
#define TYPEC_DETACH_INT			BIT(1)

/* TYPEC_MODE */
#define TYPEC_MODE_SNK	0
#define TYPEC_MODE_SRC	1
#define TYPEC_MODE_DRP	2
#define TYPEC_MODE_MASK	3

/* TYPEC_CC_STATUS */
#define TYPEC_CC_CHANNEL_MASK		BIT(4)

#define TYPEC_CC_CHANNEL_DET_REG	TYPEC_STATUS
#define TYPEC_CC_CHANNEL_DET_MASK	GENMASK(8, 5)
#define TYPEC_CC1_CHANNEL_DET		0x20
#define TYPEC_CC1_CHANNEL_HIGH		0x60
#define TYPEC_CC2_CHANNEL_DET		0x80
#define TYPEC_CC2_CHANNEL_HIGH		0x180

#define TYPEC_STATE_MASK			GENMASK(3, 0)

/* TYPEC_TYPEC_RTRIM */
#define TYPEC_CC_RTRIM_MASK		GENMASK(4, 0)
#define TYPEC_CC1_RTRIM_INDEX	0
#define TYPEC_CC2_RTRIM_INDEX	5

#define EFUSE_BITSINDEX(b, o)	((b) * 16 + (o))
#define EFUSE_CC1_INDEX			EFUSE_BITSINDEX(13, 6)
#define EFUSE_CC1_LENGTH		5
#define EFUSE_CC2_INDEX			EFUSE_BITSINDEX(13, 11)
#define EFUSE_CC2_LENGTH		5

#endif

#ifdef CONFIG_PMIC_SC2720

#define TYPEC_MODULE_EN			ANA_REG_GLB_MODULE_EN0
#define TYPEC_MODULE_RST		ANA_REG_GLB_SOFT_RST1
#define TYPEC_RTC_EN			ANA_REG_GLB_RTC_CLK_EN0

#define TYPEC_MODULE_EN_BIT		BIT_ANA_TYPEC_EN
#define TYPEC_SOFT_RST_BIT		BIT_TYPEC_SOFT_RST
#define TYPEC_RTC_EN_BIT		BIT_RTC_TYPEC_EN


/* registers definitions for controller REGS_TYPEC */

#define TYPEC_CONFIG			TYPEC_REG(0x00)
#define TYPEC_MODE				TYPEC_REG(0x04)
#define TYPEC_PD_CFG			TYPEC_REG(0x08)
#define TYPEC_INT_EN			TYPEC_REG(0x0c)
#define TYPEC_INT_CLR			TYPEC_REG(0x10)
#define TYPEC_INT_RAW			TYPEC_REG(0x14)
#define TYPEC_INT_MASK			TYPEC_REG(0x18)
#define TYPEC_STATUS			TYPEC_REG(0x1c)
#define TYPEC_TCCDE_CNT			TYPEC_REG(0x20)
#define TYPEC_TPDDE_CNT			TYPEC_REG(0x24)
#define TYPEC_TOGGLE_CNT		TYPEC_REG(0x28)
#define TYPEC_SLEEP_CNT1		TYPEC_REG(0x2c)
#define TYPEC_SLEEP_CNT2		TYPEC_REG(0x30)
#define TYPEC_DRPTRY_CNT		TYPEC_REG(0x34)
#define TYPEC_ITRIM				TYPEC_REG(0x38)
#define TYPEC_RTRIM				TYPEC_REG(0x3c)
#define TYPEC_VREF				TYPEC_REG(0x40)
#define TYPEC_TERR_CNT			TYPEC_REG(0x44)
#define TYPEC_TDRP_CNT			TYPEC_REG(0x48)
#define TYPEC_DRPTRYWAIT_CNT	TYPEC_REG(0x4c)
#define TYPEC_DBG				TYPEC_REG(0x60)

/* TYPEC_CONFIG */
#define TYPEC_ENABLE				BIT(0)
#define TYPEC_TOGGLE_SLEEP_EN		BIT(1)
#define TYPEC_ERR_STS_EN			BIT(2)
#define TYPEC_DIS_STS_EN			BIT(3)
#define TYPEC_USB20_ONLY			BIT(4)
#define TYPEC_DRP_ROLE_POLARITY		BIT(5)
#define TYPEC_DRP_ROLE_HOLD_EN		BIT(6)
#define TYPEC_TRY_SRC_EN			BIT(7)
#define TYPEC_TRY_SNK_EN			BIT(8)
#define TYPEC_POWERCABLE_DET_EN		BIT(9)
#define TYPEC_AUDIOCABLE_DET_EN		BIT(10)
#define TYPEC_DEBUGCABLE_DET_EN		BIT(11)
#define TYPEC_DBGO_DET_EN			BIT(12)

/* TYPEC_INT_EN */
#define TYPEC_ATTACH_INT_EN			BIT(0)
#define TYPEC_DETACH_INT_EN			BIT(1)
#define TYPEC_IN_ERR_STS_INT_EN		BIT(2)
#define TYPEC_OUT_ERR_STS_INT_EN	BIT(3)
#define TYPEC_DIS_STS_INT_EN		BIT(4)
#define TYPEC_DRP_SLEEP_INT_EN		BIT(5)
#define TYPEC_DRP_WAKEUP_INT_EN		BIT(6)

/* TYPEC_INT_CLR */
#define TYPEC_ATTACH_INT_CLR		BIT(0)
#define TYPEC_DETACH_INT_CLR		BIT(1)
#define TYPEC_IN_ERR_STS_INT_CLR	BIT(2)
#define TYPEC_OUT_ERR_STS_INT_CLR	BIT(3)
#define TYPEC_DIS_STS_INT_CLR		BIT(4)
#define TYPEC_DRP_SLEEP_INT_CLR		BIT(5)
#define TYPEC_DRP_WAKEUP_INT_CLR	BIT(6)

/* TYPEC_INT_MASK */
#define TYPEC_ATTACH_INT			BIT(0)
#define TYPEC_DETACH_INT			BIT(1)
#define TYPEC_IN_ERR_STS_INT		BIT(2)
#define TYPEC_OUT_ERR_STS_INT		BIT(3)
#define TYPEC_DIS_STS_INT			BIT(4)
#define TYPEC_DRP_SLEEP_INT			BIT(5)
#define TYPEC_DRP_WAKEUP_INT		BIT(6)

/* TYPEC_MODE */
#define TYPEC_MODE_SNK	0
#define TYPEC_MODE_SRC	1
#define TYPEC_MODE_DRP	2
#define TYPEC_MODE_MASK	3

/* TYPEC_CC_STATUS */
#define TYPEC_CC_CHANNEL_MASK		BIT(5)

#define TYPEC_CC_CHANNEL_DET_REG	TYPEC_DBG
#define TYPEC_CC_CHANNEL_DET_MASK	GENMASK(7, 0)
#define TYPEC_CC1_CHANNEL_DET		0x01
#define TYPEC_CC1_CHANNEL_HIGH		0x03
#define TYPEC_CC2_CHANNEL_DET		0x10
#define TYPEC_CC2_CHANNEL_HIGH		0x30

#define TYPEC_STATE_MASK			GENMASK(4, 0)

/* TYPEC_TYPEC_RTRIM */
#define TYPEC_CC_RTRIM_MASK		GENMASK(4, 0)
#define TYPEC_CC1_RTRIM_INDEX	0
#define TYPEC_CC2_RTRIM_INDEX	5

#define EFUSE_BITSINDEX(b, o)	((b) * 16 + (o))
#define EFUSE_CC1_INDEX			EFUSE_BITSINDEX(13, 6)
#define EFUSE_CC1_LENGTH		5
#define EFUSE_CC2_INDEX			EFUSE_BITSINDEX(13, 11)
#define EFUSE_CC2_LENGTH		5

#endif

/* Common Define */
#define TYPEC_CC1_CHANNEL			1
#define TYPEC_CC2_CHANNEL			2
#define TYPEC_CC1_DET_CC2_HIGH		(TYPEC_CC1_CHANNEL_DET | \
							TYPEC_CC2_CHANNEL_HIGH)
#define TYPEC_CC2_DET_CC1_HIGH		(TYPEC_CC2_CHANNEL_DET | \
							TYPEC_CC1_CHANNEL_HIGH)

#define TYPEC_TIME_WINDOW	1000000000ULL

enum typec_connection_state {
	TYPEC_DETACHED_SNK,
	TYPEC_ATTACHWAIT_SNK,
	TYPEC_ATTACHED_SNK,
	TYPEC_DETACHED_SRC,
	TYPEC_ATTACHWAIT_SRC,
	TYPEC_ATTACHED_SRC,
	TYPEC_POWERED_CABLE,
	TYPEC_AUDIO_CABLE,
	TYPEC_DEBUG_CABLE,
	TYPEC_TOGGLE_SLEEP,
	TYPEC_ERR_RECOV,
	TYEEC_DISABLED,
	TYEEC_TRY_SNK,
	TYEEC_TRY_WAIT_SRC,
	TYEEC_TRY_SRC,
	TYEEC_TRY_WAIT_SNK,
	TYEEC_UNSUPOORT_ACC,
	TYEEC_ORIENTED_DEBUG,
};

struct typec_efuse_trim {
	uint16_t itrim;
	uint16_t rtrim;
	uint16_t vref;
};

struct typec {
	struct device *dev;
	struct regmap *base;
	uint32_t reg_offset;
	int irq;

	/* irq events */
	uint16_t event;
	/* typec state */
	enum typec_connection_state state;
	struct work_struct isr_work;
	spinlock_t lock;

	struct typec_efuse_trim trims;
	bool usb20_only;
	uint32_t tsleep;
	/* role status */
	uint32_t status;
	uint32_t mode;
	uint32_t cc_status;
	struct dual_role_phy_instance *dual_role;
	struct dual_role_phy_desc *desc;

	/* for super-speed channel switch */
	uint32_t ldo_sw_en;
	int ldo_gpio;

	/* Events debug information */
	unsigned long irq_cnt;
	unsigned long snk_attach_cnt, snk_detach_cnt;
	unsigned long src_attach_cnt, src_detach_cnt;

	/* Record driver probe state */
	uint32_t has_probed;
};

#ifdef CONFIG_OTP_SPRD_PMIC_EFUSE
extern u32 sprd_pmic_efuse_bits_read(int bit_index, int length);
#endif

#endif
