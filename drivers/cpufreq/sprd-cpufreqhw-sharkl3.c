/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#undef DEBUG_SPRD_CPUFREQ
#define pr_fmt(fmt) "sprd_cpufreqhw: " fmt

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/cpufreq.h>
#include <linux/sprd-cpufreqhw.h>
#include "sprd-cpufreqhw-sharkl3.h"

#ifdef P_DBG
#undef P_DBG
#endif
#ifdef P_INF
#undef P_INF
#endif
#ifdef P_WARN
#undef P_WARN
#endif
#ifdef P_ERR
#undef P_ERR
#endif

#ifdef DEBUG_SPRD_CPUFREQ
#define P_DBG(fmt, ...) pr_emerg("%s: " fmt, __func__, ##__VA_ARGS__)
#define P_INF pr_emerg
#define P_WARN pr_emerg
#define P_ERR pr_emerg
#define P_DBG_INT(fmt, ...) pr_crit("%s: " fmt, __func__, ##__VA_ARGS__)
#define P_CRIT(fmt, ...) pr_crit("%s: " fmt, __func__, ##__VA_ARGS__)
#else
#define P_DBG(fmt, ...) pr_debug("%s: " fmt, __func__, ##__VA_ARGS__)
#define P_INF pr_info
#define P_WARN pr_warn
#define P_ERR pr_err
#define P_DBG_INT(fmt, ...) pr_debug("%s: " fmt, __func__, ##__VA_ARGS__)
#define P_CRIT(fmt, ...) pr_debug("%s: " fmt, __func__, ##__VA_ARGS__)
#endif

#define SPRD_CPUFREQ_IRQ_ENABLE
#define  SPRD_CPUFREQ_I2C_LOCK

#define MPLL_HW_DVFS_EACH_NUM (6)
#define MPLL_HW_DVFS_MAX_NUM (MPLL_HW_DVFS_EACH_NUM * 3)
#define SWDVFS_HWDVFS_KINT_DIFF_BITS (21)

#define REG_DVFS_CTRL_BASE ((unsigned int)0x0)

#define BIT_DVFS_CTRL_PROJ_NAME1 \
BIT_DVFS_CTRL_PROJ_NAME(0xff00)
#define BIT_DVFS_CTRL_PROJ_NAME0 \
BIT_DVFS_CTRL_PROJ_NAME(0xff)
#define BIT_DVFS_CTRL_VERSION1 \
BIT_DVFS_CTRL_VERSION(0xff00)
#define BIT_DVFS_CTRL_VERSION0 \
BIT_DVFS_CTRL_VERSION(0xff)

#define DVFS_CTRL_MAGIC_NUM_LOCK (0x5348554c)
#define DVFS_CTRL_MAGIC_NUM_UNLOCK (0x00000000)

#define SW_CHNL02_EN_MSK \
BIT_DVFS_CTRL_SW_CHNL_EN(0x1 << 2)
#define SW_CHNL01_EN_MSK \
BIT_DVFS_CTRL_SW_CHNL_EN(0x1 << 1)
#define SW_CHNL00_EN_MSK \
BIT_DVFS_CTRL_SW_CHNL_EN(0x1 << 0)

/*chnl1/2-adi-core0,chnl0-i2c-core1*/
#define DCDC_MAP_ADI_CHNL12_I2C_CHNL0 (0x001)

#define GLB_CFG_DEBUG_SEL_EN (0x1d)

#define TMR_PRESCALE_1US (25)

#define TMR_CTRLx_DCDCx_STABLE_US 10
#define TMR_CTRLx_DCDCx_I2C_PAUSE_US (75 + TMR_CTRLx_DCDCx_STABLE_US + 50)
#define TMR_CTRLx_DCDCx_ADI_PAUSE_US (25 + TMR_CTRLx_DCDCx_STABLE_US)

/*DCDC 2721 CONFIG*/
#define VTUNE_STEP_CORE0_MV (25)
#define VTUNE_STEP_CORE1_MV (25)
#define TMR_CTRL2_CORE1_STABLE_US_2721 (VTUNE_STEP_CORE0_MV * 10 / 50)
#define VTUNE_STEP_VAL_CORE00_2721 (((int)VTUNE_STEP_CORE0_MV * 32) / 100)

/*DCDC FAN5355 CONFIG*/
#define TMR_CTRL2_CORE1_STABLE_US_FAN5355 (20)

#define VTUNE_STEP_VAL_CORE00_DEFALUT (20)
#define VTUNE_STEP_VAL_CORE00_VAL  VTUNE_STEP_VAL_CORE00_2721

#define TMR_CTRL1_CORE0_HOLD_US (50)
#define TMR_CTRL1_CORE0_PAUSE_US TMR_CTRL2_CORE1_STABLE_US_2721

#define TMR_CTRL2_CORE0_TIMEOUT_US (800)
#define TMR_CTRL2_CORE0_STABLE_US TMR_CTRL2_CORE1_STABLE_US_2721

#define TMR_CTRL1_CORE1_HOLD_US (25)
#define TMR_CTRL1_CORE1_PAUSE_US TMR_CTRL2_CORE1_STABLE_US_FAN5355

#define TMR_CTRL2_CORE1_TIMEOUT_US (800)
#define TMR_CTRL2_CORE1_STABLE_US TMR_CTRL2_CORE1_STABLE_US_FAN5355

#define TMR_CTRL_PLL_PD_US (2)
#define TMR_CTRL_PLL_STABLE_US (199)

/*clock mux change stable period. it begins
  *from the end of fsel changes to backup.
  *all pll use the same value.
  *suggestive value is four clock period of the
  *slowest clock. and the possible slowest clock
  *during hw dvfs may be 512MHz,
  *so set to one 26MHz clock period is enough.
  */
#define TMR_CTRL_FMUX_US (0)

#define SCALE_TAB_VTUNE_MSK (0x3FF << 7)
#define SCALE_TAB_FCFG_MSK (0xF << 3)
#define SCALE_TAB_FSEL_MSK (0x7 << 0)
#define SCALE_TAB_EACH_NUM (8)
#define SCALE_TAB_MAX_NUM (SCALE_TAB_EACH_NUM * 3)

#define REG_DCDC0_VOL_CTL (0xc54)
#define DCDC0_VOL_CTL_MSK (0x1ff)
#define REG_DCDC1_VOL_CTL (0xc64)
#define DCDC1_VOL_CTL_MSK (0x1ff)

#define REG_DCDC0_VOL_CTL_FAN5355 (0x01)
#define DCDC0_VOL_CTL_MSK_FAN5355 (0xFF)

#define VAL2REG(val, msk) \
((((unsigned int)(val)) << (__ffs(msk))) & ((unsigned int)(msk)))

#define REG2VAL(val_in_reg, msk) \
((((unsigned int)(val_in_reg)) & ((unsigned int)(msk))) >> \
__ffs(msk))

/*frequency calculation*/
#define FCFG_REFIN_MHZ ((unsigned long)26)
#define FCFG_NINT(fvco_khz) \
((unsigned long)(fvco_khz * 1000) / (FCFG_REFIN_MHZ * 1000000))

#define FCFG_KINT_REMAINDER(fvco_khz) \
((unsigned long)(((fvco_khz * 1000) - \
FCFG_REFIN_MHZ * FCFG_NINT(fvco_khz) * 1000000) / \
10000))

#define FCFG_KINT_COEF \
((unsigned long)(( \
BIT_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_KINT(0xffffffff) >> \
__ffs(BIT_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_KINT(0xffffffff))) + \
1))

#define FCFG_KINT_DIVISOR ((unsigned long)(FCFG_REFIN_MHZ * 100))

#ifdef CONFIG_64BIT
#define FCFG_KINT(fvco_khz)  \
((FCFG_KINT_REMAINDER(fvco_khz) *  \
FCFG_KINT_COEF) / \
FCFG_KINT_DIVISOR)
#else
#define FCFG_KINT(fvco_khz) \
((FCFG_KINT_REMAINDER(fvco_khz) * \
(FCFG_KINT_COEF / FCFG_KINT_DIVISOR)) + \
((FCFG_KINT_REMAINDER(fvco_khz) * \
(FCFG_KINT_COEF % FCFG_KINT_DIVISOR)) / FCFG_KINT_DIVISOR))
#endif
#define VAL2REG_IBIAS_RESERVED(in) (VAL2REG(in, \
BIT_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_IBIAS_DVFS_0(0x3) |  \
BIT_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_RESERVED_DVFS_0(0x3)))

#define VAL2REG_IBIAS(ibias) (VAL2REG(ibias, \
BIT_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_IBIAS_DVFS_0(0xffffffff)))

#define VAL2REG_POSTDIV(postdiv) (VAL2REG(postdiv, \
BIT_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_POSTDIV_DVFS_0))

#define VAL2REG_KN_HW(fvco_khz) \
(VAL2REG(FCFG_NINT(fvco_khz), \
BIT_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_NINT_DVFS_0(0xffffffff)) |  \
VAL2REG((FCFG_KINT(fvco_khz) >> SWDVFS_HWDVFS_KINT_DIFF_BITS), \
BIT_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_KINT_DVFS_0(0xffffffff)))

#define VAL2REG_KN_MPLL(nINT, kINT) \
(VAL2REG(nINT, \
BIT_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_NINT_DVFS_0(0xffffffff)) | \
VAL2REG((kINT >> SWDVFS_HWDVFS_KINT_DIFF_BITS), \
BIT_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_KINT_DVFS_0(0xffffffff)))

#define VAL2REG_KN_SW(fvco_khz)  \
(VAL2REG(FCFG_NINT(fvco_khz), \
BIT_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_NINT(0xffffffff)) | \
VAL2REG((FCFG_KINT(fvco_khz)), \
BIT_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_KINT(0xffffffff)))

#define VAL2REG_FSEL(val) VAL2REG(val, SCALE_TAB_FSEL_MSK)
#define VAL2REG_FCFG(val) VAL2REG(val, SCALE_TAB_FCFG_MSK)

#define VAL2REG_DCDC(to_uv, min_uv, step_uv, msk) \
VAL2REG(DIV_ROUND_UP((int)((to_uv) - (min_uv)), (step_uv)), msk)

#define VAL2REG_DCDC_FAN5355(to_uv, min_uv, step_uv, msk) \
VAL2REG(DIV_ROUND_UP((int)((to_uv) - (min_uv)), (step_uv)) | 0x80, msk)

#define REG2VAL_DCDC(reg_uv, min_uv, step_uv, hide_offset_uv, msk) \
((min_uv + REG2VAL(reg_uv, msk) * step_uv))

/*channel0:little core*/
#define HW_DVFS_TAB_CLUSTER0(hz, uv, idx_freq, idx_dcdc) \
(val2reg_dcdcx_vtune(uv, idx_dcdc) | \
VAL2REG_FCFG((((hz) <= 768000000 ? 1 : 0) << 3) | idx_freq) | \
VAL2REG_FSEL((hz) <= 768000000 ? 0x02 : 0x06))

/*channel1:big core*/
#define HW_DVFS_TAB_CLUSTER1(hz, uv, idx_freq, idx_dcdc) \
(val2reg_dcdcx_vtune(uv, idx_dcdc) | \
VAL2REG_FCFG((((hz) <= 768000000 ? 1 : 0) << 3) | idx_freq) | \
VAL2REG_FSEL((hz) <= 768000000 ? 0x02 : 0x07))

/*channel2:fcm*/
#define HW_DVFS_TAB_CLUSTER2(hz, uv, idx_freq, idx_dcdc) \
(val2reg_dcdcx_vtune(uv, idx_dcdc) | \
VAL2REG_FCFG((((hz) <= 768000000 ? 1 : 0) << 3) | idx_freq) | \
VAL2REG_FSEL((hz) <= 768000000 ? 0x02 : 0x05))

/*FAN5355 + 2721 verify, update dvfs table*/
#define MPLL_HW_DVFS_TAB_CLUSTER0(khz)  \
(VAL2REG_IBIAS((khz) < 1200000 ? 0x00 : 0x01) | \
VAL2REG_POSTDIV((khz) < 1000000 ? 0x01 : 0x00) | \
VAL2REG_KN_HW((khz) * ((khz) < 1000000 ? 0x02 : 0x01)))

#define MPLL_HW_DVFS_TAB_CLUSTER1(khz) \
(VAL2REG_IBIAS((khz) < 1225000 ? 0x01 : ((khz) < 1500000 ? 0x02 : 0x03)) | \
VAL2REG_POSTDIV((khz) < 1000000 ? 0x01 : 0x00) |  \
VAL2REG_KN_HW((khz) * ((khz) < 1000000 ? 0x02 : 0x01)))

#define MPLL_HW_DVFS_TAB_CLUSTER2(khz) \
(VAL2REG_IBIAS((khz) < 1066000 ? 0x00 : ((khz) < 1350000 ? 0x01 : 0x02)) | \
VAL2REG_POSTDIV((khz) < 1000000 ? 0x01 : 0x00) |  \
VAL2REG_KN_HW((khz) * ((khz) < 1000000 ? 0x02 : 0x01)))

struct sprd_cpufreqhw_reg {
	unsigned int default_uv;
	unsigned int step_uv;
	unsigned int min_uv;
	unsigned int max_uv;
	unsigned int hide_offset_uv;
	unsigned long reg_vol_trm;
	unsigned int vol_trm_bits;
	/*const bits in vol reg*/
	unsigned int vol_const_bits;
};

enum sprd_cpufreqhw_state {
	CPUFREQHW_STATE_UNKNOWN,
	CPUFREQHW_STATE_RUNNING,
	CPUFREQHW_STATE_DONE,
};

enum sprd_cpufreqhw_i2c_state {
	CPUFREQHW_I2C_UNLOCK,
	CPUFREQHW_I2C_LOCK,
};

enum sprd_cpufreqhw_dcdc {
	DCDC_2731_VOL0,
	DCDC_2731_VOL1,
	DCDC_5355_VOL0,
	DCDC_2703_VOL0,
	DCDC_MAX,
};

static struct sprd_cpufreqhw_reg dcdc[DCDC_MAX] = {
	/*DCDC_2731_VOL0*/
	{900000, 3125, 400000, 1996000, 1000, REG_DCDC0_VOL_CTL,
	 DCDC0_VOL_CTL_MSK, 0},
	 /*DCDC_2731_VOL1*/
	{900000, 3125, 400000, 1996000, 1000, REG_DCDC1_VOL_CTL,
	 DCDC1_VOL_CTL_MSK, 0},
	 /*DCDC_5355_VOL0*/
	{603000 + 602822, 12826, 603000, 1411000, 1000,
	 REG_DCDC0_VOL_CTL_FAN5355,
	 DCDC0_VOL_CTL_MSK_FAN5355, 0x80},
	  /*DCDC_2703_VOL0*/
	{300000 + 10000*0x46, 10000, 300000, 1570000, 1000,
	 0x03,
	 0x7f, 0x00},
};

static inline unsigned int
val2reg_dcdcx_vtune(unsigned int to_uv, unsigned int idx) {
	unsigned int valreg = 0;

	valreg = DIV_ROUND_UP(to_uv - dcdc[idx].min_uv, dcdc[idx].step_uv);
	valreg |= dcdc[idx].vol_const_bits;
	valreg = VAL2REG(valreg, SCALE_TAB_VTUNE_MSK);

	return valreg;
}

static unsigned int
mpll_hw_dvfs_table(unsigned long freq_khz) {
	unsigned int mpll_hw = 0x0000082E;
	unsigned long vco_khz;
	unsigned long ibias_reserved = 0;
	unsigned long postdiv = 0;

	if (freq_khz < 400000) {
		P_ERR("freq_khz %lu less than 0.4GHZ\n",
			freq_khz);
		goto EXIT;
	}

	postdiv = (freq_khz < 800000) ? 1 : 0;
	vco_khz = freq_khz * (postdiv + 1);

	if (vco_khz > 2000000) {
		P_ERR("freq_khz %lu needs vco_khz %lu over 2.0GHZ\n",
			freq_khz, vco_khz);
		goto EXIT;
	}

	if (vco_khz < 1000000)
		ibias_reserved = 0x0;
	else if (vco_khz < 1200000)
		ibias_reserved = 0x1;
	else if (vco_khz < 1400000)
		ibias_reserved = 0x2;
	else if (vco_khz < 1600000)
		ibias_reserved = 0x3;
	else if (vco_khz < 1800000)
		ibias_reserved = 0x4;
	else
		ibias_reserved = 0x5;

	mpll_hw = VAL2REG_IBIAS_RESERVED(ibias_reserved) |
		VAL2REG_POSTDIV(postdiv) |
		VAL2REG_KN_HW(vco_khz);

EXIT:
	return mpll_hw;
}

#define _dvfs_rd_(reg) \
readl_relaxed((void __iomem *)(reg + cpufreqhw->base_dvfs))
#define _dvfs_wr_(val, reg) \
writel_relaxed(val, (void __iomem *)(reg + cpufreqhw->base_dvfs))

#define SPRD_CPUFREQHW_MAX_FREQ_VOLT 8
#define SPRD_CPUFREQHW_CHNL00_DEFAULT_FREQ 2
#define SPRD_CPUFREQHW_CHNL01_DEFAULT_FREQ 5
#define SPRD_CPUFREQHW_CHNL02_DEFAULT_FREQ 5

#define SPRD_CPUFREQHW_BUSY_DURATOIN (2ul * HZ)

enum sprd_cpufreqhw_chnl {
	CPUFREQHW_CHNL00,
	CPUFREQHW_CHNL01,
	CPUFREQHW_CHNL02,
	CPUFREQHW_CHNL_MAX,
};

enum sprd_cpufreqhw_type {
	UNKNOWN_CPUFREQHW,
	SPRD_CPUFREQHW_SHARKL3,
};

struct sprd_cpufreqhw_group {
	/*HZ*/
	unsigned long freq;
	/*uV */
	unsigned long volt;
};

struct sprd_cpufreqhw {
	struct regmap *aon_apb_base;
	struct regmap *anlg_phy_g4_ctrl;
	void __iomem *base_dvfs;
#ifdef SPRD_CPUFREQ_I2C_LOCK
	struct i2c_client *i2c_client;
	struct completion i2c_done;
	unsigned int on_i2c[CPUFREQHW_CHNL_MAX];
#endif
	unsigned int dcdc_index[CPUFREQHW_CHNL_MAX];
	enum sprd_cpufreqhw_type type;
	int irq;
	spinlock_t mlock;
	bool probed;
	bool ready;
	bool enabled;
	bool triggered[CPUFREQHW_CHNL_MAX];
	/*busy time */
	unsigned long busy[CPUFREQHW_CHNL_MAX];
	/*0-cluster0, 1-cluster1, 2-suc */
	/*min freq is on freqvolt[0] */
	struct sprd_cpufreqhw_group
	    freqvolt[CPUFREQHW_CHNL_MAX][SPRD_CPUFREQHW_MAX_FREQ_VOLT];
	atomic_t dvfs_state[CPUFREQHW_CHNL_MAX];
	int idx_max[CPUFREQHW_CHNL_MAX];
};

static struct sprd_cpufreqhw *cpufreqhw;
static struct kobject *cpufreqhw_kobj;
static atomic_t cpufreqhw_suspend;
static void sprd_cpufreqhw_dump(bool forcedump);
static int hwdvfs_trigger_lit(unsigned int scalecode00, bool sync, bool force);
static int hwdvfs_trigger_big_scu(unsigned int scalecode01, bool sync,
				  bool force);

static const struct of_device_id sprd_cpufreqhw_of_match[] = {
	{
	 .compatible = "sprd,sharkl3-cpufreqhw",
	 .data = (void *)SPRD_CPUFREQHW_SHARKL3,
	 },
};

#define CPUFREQHW_KOBJ_ATT(_name) \
static struct kobj_attribute _name##_attr = { \
	.attr = \
	  { \
	.name = __stringify(_name), .mode = 0644, \
	  },  \
	.show = _name##_show, \
	.store = _name##_store, \
}

static ssize_t cpufreqhw_enable_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	if (cpufreqhw == NULL)
		ret = sprintf(buf, "NULL\n");
	else {
		ret =
			(cpufreqhw->
			probed ? sprintf(buf, "1\n") : sprintf(buf, "0\n"));

		P_DBG("dvfs_state:[%d %d %d]\n",
		    atomic_read(&cpufreqhw->dvfs_state[CPUFREQHW_CHNL00]),
		    atomic_read(&cpufreqhw->dvfs_state[CPUFREQHW_CHNL01]),
		    atomic_read(&cpufreqhw->dvfs_state[CPUFREQHW_CHNL02]));
	}

	return ret;
}

static ssize_t cpufreqhw_enable_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t n)
{
	unsigned int en;

	if (n != 2 || cpufreqhw == NULL)
		return -EINVAL;

	if (kstrtouint(buf, 0, &en))
		return -EINVAL;

	/*TODO: need to enable magic number, and add spinlock here */
	if (en == 0 &&
	!hwdvfs_trigger_lit(SPRD_CPUFREQHW_CHNL00_DEFAULT_FREQ,
			true, true) &&
	!hwdvfs_trigger_big_scu(SPRD_CPUFREQHW_CHNL01_DEFAULT_FREQ,
			true, true)) {
		cpufreqhw->probed = false;
		_dvfs_wr_(DVFS_CTRL_MAGIC_NUM_UNLOCK,
			REG_DVFS_CTRL_MAGIC_NUM);
		_dvfs_wr_(VAL2REG(0x0, BIT_DVFS_CTRL_HW_DVFS_SEL),
			REG_DVFS_CTRL_HW_DVFS_SEL);
		P_INF("DISABLE HWDVFS!\n");
	} else if (en == 1) {
		_dvfs_wr_(DVFS_CTRL_MAGIC_NUM_LOCK,
			REG_DVFS_CTRL_MAGIC_NUM);
		_dvfs_wr_(VAL2REG(0x1, BIT_DVFS_CTRL_HW_DVFS_SEL),
			REG_DVFS_CTRL_HW_DVFS_SEL);
		cpufreqhw->probed = true;
		hwdvfs_trigger_lit(SPRD_CPUFREQHW_CHNL00_DEFAULT_FREQ,
					true, true);
		hwdvfs_trigger_big_scu(SPRD_CPUFREQHW_CHNL01_DEFAULT_FREQ,
					true, true);
		P_INF("ENABLE HWDVFS!\n");
	} else
		return -EINVAL;

	return n;
}

CPUFREQHW_KOBJ_ATT(cpufreqhw_enable);
static struct attribute *g[] = {
	&cpufreqhw_enable_attr.attr, NULL,
};

static struct attribute_group attr_group_cpufreqhw = {
	.attrs = g,
};

static void sprd_cpufreqhw_dump(bool forcedump)
{
	unsigned int i = 0;
	static int prt_cnt;

	/*dump twice */
	if (!forcedump && prt_cnt > 3)
		return;

	prt_cnt++;
	for (i = REG_DVFS_CTRL_BASE; i < REG_DVFS_CTRL_CHNL02_SCALE07; i += 32)
		P_ERR("0x%03x: %08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x\n",
			i,
			(unsigned int)_dvfs_rd_(REG_DVFS_CTRL_BASE + i + 0),
			(unsigned int)_dvfs_rd_(REG_DVFS_CTRL_BASE + i + 4),
			(unsigned int)_dvfs_rd_(REG_DVFS_CTRL_BASE + i + 8),
			(unsigned int)_dvfs_rd_(REG_DVFS_CTRL_BASE + i + 12),
			(unsigned int)_dvfs_rd_(REG_DVFS_CTRL_BASE + i + 16),
			(unsigned int)_dvfs_rd_(REG_DVFS_CTRL_BASE + i + 20),
			(unsigned int)_dvfs_rd_(REG_DVFS_CTRL_BASE + i + 24),
			(unsigned int)_dvfs_rd_(REG_DVFS_CTRL_BASE + i + 28));
}
#ifdef SPRD_CPUFREQ_I2C_LOCK
static int sprd_cpufreqhw_i2c_trylock(struct i2c_adapter *adapter)
{
	struct i2c_adapter *parent = i2c_parent_is_i2c_adapter(adapter);

	if (parent)
		return sprd_cpufreqhw_i2c_trylock(parent);
	else
		return rt_mutex_trylock(&adapter->bus_lock);
}


static int sprd_cpufreqhw_i2c_unlocking(unsigned int cluster)
{
	if (!wait_for_completion_timeout(&(cpufreqhw->i2c_done),
		msecs_to_jiffies(2000)))
		P_ERR("CHNL%u I2C gets dvfs timeout\n", cluster);
	i2c_unlock_adapter(cpufreqhw->i2c_client->adapter);

	return 0;
}

static int sprd_cpufreqhw_i2c_unlock_sync(unsigned int cluster)
{

	if (cpufreqhw->on_i2c[cluster] &&
	cpufreqhw->i2c_client != NULL) {
		sprd_cpufreqhw_i2c_unlocking(cluster);
		P_DBG_INT("CHNL%u DONE i2c_unlock_adapter\n", cluster);
	}

	return 0;
}
static int sprd_cpufreqhw_i2c_unlock_async(unsigned int cluster)
{
	if (cpufreqhw->on_i2c[cluster] &&
	cpufreqhw->i2c_client != NULL &&
	atomic_read(&cpufreqhw->dvfs_state[cluster]) ==
	CPUFREQHW_STATE_RUNNING) {
		complete(&cpufreqhw->i2c_done);
		P_DBG_INT("CHNL%u i2c_unlock_async\n", cluster);
	}
	return 0;
}
#endif
static int sprd_cpufreqhw_try_lock(unsigned int cluster)
{

	int ret = 0;
	if (atomic_read(&cpufreqhw->dvfs_state[cluster]) !=
	    CPUFREQHW_STATE_RUNNING) {
#ifdef SPRD_CPUFREQ_I2C_LOCK
		if (cpufreqhw->on_i2c[cluster] &&
		cpufreqhw->i2c_client != NULL) {
			if (sprd_cpufreqhw_i2c_trylock(
				cpufreqhw->i2c_client->adapter))
				P_DBG_INT("CHNL%d i2c_lock_adapter\n", cluster);
			else {
				P_WARN("CHNL%d i2c_lock_adapter fail\n",
					cluster);
				return -EBUSY;
			}
		}
#endif
		atomic_set(&cpufreqhw->dvfs_state[cluster],
			   CPUFREQHW_STATE_RUNNING);
	} else
		ret = -EBUSY;

	return ret;
}

static int sprd_cpufreqhw_try_lock_big_scu(void)
{
	int ret = 0;

	if (atomic_read(&cpufreqhw->dvfs_state[CPUFREQHW_CHNL01]) !=
	    CPUFREQHW_STATE_RUNNING &&
	    atomic_read(&cpufreqhw->dvfs_state[CPUFREQHW_CHNL02]) !=
	    CPUFREQHW_STATE_RUNNING) {
#ifdef SPRD_CPUFREQ_I2C_LOCK
		if (cpufreqhw->on_i2c[CPUFREQHW_CHNL01] &&
		cpufreqhw->i2c_client != NULL) {
			if (sprd_cpufreqhw_i2c_trylock(
				cpufreqhw->i2c_client->adapter))
				P_DBG_INT("CHNL1 i2c_lock_adapter\n");
			else {
				P_WARN("CHNL1 i2c_lock_adapter fail\n");
				return -EBUSY;
			}
		}
#endif
		atomic_set(&cpufreqhw->dvfs_state[CPUFREQHW_CHNL01],
			   CPUFREQHW_STATE_RUNNING);
		atomic_set(&cpufreqhw->dvfs_state[CPUFREQHW_CHNL02],
			   CPUFREQHW_STATE_RUNNING);
	} else
		ret = -EBUSY;

	return ret;
}

static void sprd_cpufreqhw_unlock(unsigned int cluster, bool unlock_i2c)
{
	if (atomic_read(&cpufreqhw->dvfs_state[cluster]) !=
	CPUFREQHW_STATE_RUNNING) {
		P_ERR("ERROR! CHNL%u get multiple DVFS INT\n", cluster);
		sprd_cpufreqhw_dump(false);
	}

	if (unlock_i2c)
		sprd_cpufreqhw_i2c_unlock_async(cluster);
	cpufreqhw->busy[cluster] = 0;
	atomic_set(&cpufreqhw->dvfs_state[cluster],
		   CPUFREQHW_STATE_DONE);

}

/**
 * sprd_cpufreqhw_init_param - init hw dvfs common registers
 */
static int sprd_cpufreqhw_init_param(struct device_node *np)
{
	struct regmap *aon_apb;
	unsigned int regval = 0;
	unsigned int cfg[5] = {0, 0, 0, 0, 0};

	if (cpufreqhw == NULL)
		return -ENODEV;

	aon_apb = cpufreqhw->aon_apb_base;

	/*enable  HW dvfs */
	regmap_update_bits(aon_apb, REG_AON_APB_APB_EB4, BIT_AON_APB_DVFS_EB,
			   BIT_AON_APB_DVFS_EB);
	/*Reset  HW dvfs */
	regmap_update_bits(aon_apb, REG_AON_APB_APB_RST4,
			   BIT_AON_APB_DVFS_SOFT_RST,
			   BIT_AON_APB_DVFS_SOFT_RST);
	udelay(50);
	regmap_update_bits(aon_apb, REG_AON_APB_APB_RST4,
			   BIT_AON_APB_DVFS_SOFT_RST,
			   (unsigned int)(~BIT_AON_APB_DVFS_SOFT_RST));

	/*not select HW DVFS at first */
	_dvfs_wr_(VAL2REG(0x0, BIT_DVFS_CTRL_HW_DVFS_SEL),
		REG_DVFS_CTRL_HW_DVFS_SEL);

	regval = _dvfs_rd_(REG_DVFS_CTRL_VERSION);
	P_INF("Project Name:0x%x-0x%x;Version:0x%x-0x%x\n",
	      (char)(REG2VAL(regval, BIT_DVFS_CTRL_PROJ_NAME1)),
	      (char)(REG2VAL(regval, BIT_DVFS_CTRL_PROJ_NAME0)),
	      (REG2VAL(regval, BIT_DVFS_CTRL_VERSION1)),
	      (REG2VAL(regval, BIT_DVFS_CTRL_VERSION0)));

	/*Debug_sel: 0x1b or 0x1d*/
	_dvfs_wr_(BIT_DVFS_CTRL_DEBUG_SEL(0x1b) |
		  VAL2REG(0x1, BIT_DVFS_CTRL_TO_CHECK_EN) |
		  VAL2REG(0x0, BIT_DVFS_CTRL_AUTO_GATE_EN) |
		  VAL2REG(0x0, BIT_DVFS_CTRL_SMART_MODE),
		  REG_DVFS_CTRL_GLB_CFG);

	/*configure Vtune step*/
	cfg[0] = 0;
	cfg[1] = VTUNE_STEP_VAL_CORE00_DEFALUT;
	of_property_read_u32(np, "sprd,vtune-step-fast-core00",
		&cfg[0]);
	of_property_read_u32(np, "sprd,vtune-step-val-core00",
		&cfg[1]);
	_dvfs_wr_(VAL2REG(cfg[0], BIT_DVFS_CTRL_VTUNE_STEP_FAST_CORE00) |
		  BIT_DVFS_CTRL_VTUNE_STEP_VAL_CORE00(cfg[1]),
		  REG_DVFS_CTRL_VTUNE_STEP_CORE00);
	cfg[0] = 0;
	cfg[1] = VTUNE_STEP_VAL_CORE00_DEFALUT;
	of_property_read_u32(np, "sprd,vtune-step-fast-core01",
		&cfg[0]);
	of_property_read_u32(np, "sprd,vtune-step-val-core01",
		&cfg[1]);
	_dvfs_wr_(VAL2REG(cfg[0], BIT_DVFS_CTRL_VTUNE_STEP_FAST_CORE01) |
		  BIT_DVFS_CTRL_VTUNE_STEP_VAL_CORE01(cfg[1]),
		  REG_DVFS_CTRL_VTUNE_STEP_CORE01);

	/*configure voltage valid bits*/
	cfg[0] = 0;
	cfg[1] = VTUNE_STEP_VAL_CORE00_DEFALUT;
	of_property_read_u32(np, "sprd,vtune-vld-bit-core01",
		&cfg[0]);
	of_property_read_u32(np, "sprd,vtune-vld-bit-core00",
		&cfg[1]);
	_dvfs_wr_(BIT_DVFS_CTRL_VTUNE_VLD_BIT_CORE01(cfg[0]) |
		  BIT_DVFS_CTRL_VTUNE_VLD_BIT_CORE00(cfg[1]),
		  REG_DVFS_CTRL_VOLT_VALID_BIT);

	/*configure TUNE_EN*/
	_dvfs_wr_(BIT_DVFS_CTRL_VTUNE_EN(0x3) |
		  BIT_DVFS_CTRL_DCDC_EN_DTCT_EN(0x0) |
		  BIT_DVFS_CTRL_FCFG_TUNE_EN(0x7),
			REG_DVFS_CTRL_TUNE_EN);

#ifdef SPRD_CPUFREQ_I2C_LOCK
	/*configure chnlx in i2c*/
	cfg[0] = 0;
	of_property_read_u32(np, "sprd,chnl-in-i2c",
		&cfg[0]);
	cpufreqhw->on_i2c[CPUFREQHW_CHNL00] = (cfg[0] & 0x00f);
	cpufreqhw->on_i2c[CPUFREQHW_CHNL01] = (cfg[0] & 0x0f0) >> 4;
	cpufreqhw->on_i2c[CPUFREQHW_CHNL02] = (cfg[0] & 0xf00) >> 8;
	P_DBG("sprd,chnl-in-i2c %u->%u,%u,%u\n", cfg[0],
		cpufreqhw->on_i2c[CPUFREQHW_CHNL00],
		cpufreqhw->on_i2c[CPUFREQHW_CHNL01],
		cpufreqhw->on_i2c[CPUFREQHW_CHNL02]);
#endif
	/*configure DCDC MAP*/
	cfg[0] = 0;
	of_property_read_u32(np, "sprd,chnl-core-map",
		&cfg[0]);
	_dvfs_wr_(cfg[0], REG_DVFS_CTRL_CHNL_CORE_MAP);

	/*configure IRQ*/
	_dvfs_wr_(VAL2REG(0x1, BIT_DVFS_CTRL_IRQ_VREAD_MIS_EN_CHNL00) |
		  VAL2REG(0x1, BIT_DVFS_CTRL_IRQ_TO_EN_CHNL00) |
		  VAL2REG(0x1, BIT_DVFS_CTRL_IRQ_DONE_EN_CHNL00),
		  REG_DVFS_CTRL_IRQ_EN_CHNL00);
	_dvfs_wr_(VAL2REG(0x1, BIT_DVFS_CTRL_IRQ_VREAD_MIS_EN_CHNL01) |
		  VAL2REG(0x1, BIT_DVFS_CTRL_IRQ_TO_EN_CHNL01) |
		  VAL2REG(0x1, BIT_DVFS_CTRL_IRQ_DONE_EN_CHNL01),
		  REG_DVFS_CTRL_IRQ_EN_CHNL01);
	_dvfs_wr_(VAL2REG(0x1, BIT_DVFS_CTRL_IRQ_VREAD_MIS_EN_CHNL02) |
		  VAL2REG(0x1, BIT_DVFS_CTRL_IRQ_TO_EN_CHNL02) |
		  VAL2REG(0x1, BIT_DVFS_CTRL_IRQ_DONE_EN_CHNL02),
		  REG_DVFS_CTRL_IRQ_EN_CHNL02);

	/*VDDARMx , CHNLx, DCDCx*/
	cfg[0] = DCDC_5355_VOL0;
	of_property_read_u32(np, "sprd,chnl0-dcdc-index",
		&cfg[0]);
	cpufreqhw->dcdc_index[CPUFREQHW_CHNL00] = cfg[0];
	cfg[0] = DCDC_2731_VOL0;
	of_property_read_u32(np, "sprd,chnl1-dcdc-index",
		&cfg[0]);
	cpufreqhw->dcdc_index[CPUFREQHW_CHNL01] = cfg[0];
	cfg[0] = DCDC_2731_VOL0;
	of_property_read_u32(np, "sprd,chnl2-dcdc-index",
		&cfg[0]);
	cpufreqhw->dcdc_index[CPUFREQHW_CHNL02] = cfg[0];

	/*configure timer, stable time...*/
	_dvfs_wr_(BIT_DVFS_CTRL_TMR_PRESCALE(TMR_PRESCALE_1US),
		  REG_DVFS_CTRL_TMR_PRESCALE);

	cfg[0] = TMR_CTRL1_CORE0_HOLD_US;
	cfg[1] = TMR_CTRL1_CORE0_PAUSE_US;
	of_property_read_u32(np, "sprd,hold-val-core00",
		&cfg[0]);
	of_property_read_u32(np, "sprd,pause-val-core00",
		&cfg[1]);
	_dvfs_wr_(BIT_DVFS_CTRL_HOLD_VAL_CORE00(cfg[0]) |
		  BIT_DVFS_CTRL_PAUSE_VAL_CORE00(cfg[1]),
		  REG_DVFS_CTRL_TMR_CTRL1_CORE00);

	cfg[0] = TMR_CTRL2_CORE0_TIMEOUT_US;
	cfg[1] = TMR_CTRL2_CORE0_STABLE_US;
	of_property_read_u32(np, "sprd,to-val-core00",
		&cfg[0]);
	of_property_read_u32(np, "sprd,stable-val-core00",
		&cfg[1]);
	_dvfs_wr_(BIT_DVFS_CTRL_TO_VAL_CORE00(cfg[0]) |
		  BIT_DVFS_CTRL_DCDC_STABLE_VAL_CORE00(cfg[1]),
		  REG_DVFS_CTRL_TMR_CTRL2_CORE00);

	cfg[0] = TMR_CTRL1_CORE1_HOLD_US;
	cfg[1] = TMR_CTRL1_CORE1_PAUSE_US;
	of_property_read_u32(np, "sprd,hold-val-core01",
		&cfg[0]);
	of_property_read_u32(np, "sprd,pause-val-core01",
		&cfg[1]);
	_dvfs_wr_(BIT_DVFS_CTRL_HOLD_VAL_CORE01(cfg[0]) |
		  BIT_DVFS_CTRL_PAUSE_VAL_CORE01(cfg[1]),
		  REG_DVFS_CTRL_TMR_CTRL1_CORE01);

	cfg[0] = TMR_CTRL2_CORE1_TIMEOUT_US;
	cfg[1] = TMR_CTRL2_CORE1_STABLE_US;
	of_property_read_u32(np, "sprd,to-val-core01",
		&cfg[0]);
	of_property_read_u32(np, "sprd,stable-val-core01",
		&cfg[1]);
	_dvfs_wr_(BIT_DVFS_CTRL_TO_VAL_CORE01(cfg[0]) |
		  BIT_DVFS_CTRL_DCDC_STABLE_VAL_CORE01(cfg[1]),
		  REG_DVFS_CTRL_TMR_CTRL2_CORE01);

	_dvfs_wr_(BIT_DVFS_CTRL_PLL_PD_VAL(TMR_CTRL_PLL_PD_US) |
		  BIT_DVFS_CTRL_PLL_STABLE_VAL(TMR_CTRL_PLL_STABLE_US),
		  REG_DVFS_CTRL_TMR_CTRL_PLL);
	_dvfs_wr_(BIT_DVFS_CTRL_FMUX_STABLE_VAL(TMR_CTRL_FMUX_US),
		  REG_DVFS_CTRL_TMR_CTRL_FMUX);

	_dvfs_wr_(VAL2REG(0x00, BIT_DVFS_CTRL_FCFG_PD_SW_CHNL00) |
		  BIT_DVFS_CTRL_FSEL_PARK_CHNL00(0x02) |
		  BIT_DVFS_CTRL_FSEL_BKP_CHNL00(0x02) |
		  BIT_DVFS_CTRL_FSEL_SW_CHNL00(0x06),
		  REG_DVFS_CTRL_CFG_CHNL00);

	_dvfs_wr_(VAL2REG(0x00, BIT_DVFS_CTRL_FCFG_PD_SW_CHNL01) |
		  BIT_DVFS_CTRL_FSEL_PARK_CHNL01(0x02) |
		  BIT_DVFS_CTRL_FSEL_BKP_CHNL01(0x02) |
		  BIT_DVFS_CTRL_FSEL_SW_CHNL01(0x07),
		  REG_DVFS_CTRL_CFG_CHNL01);

	_dvfs_wr_(VAL2REG(0x00, BIT_DVFS_CTRL_FCFG_PD_SW_CHNL02) |
		  BIT_DVFS_CTRL_FSEL_PARK_CHNL02(0x02) |
		  BIT_DVFS_CTRL_FSEL_BKP_CHNL02(0x02) |
		  BIT_DVFS_CTRL_FSEL_SW_CHNL02(0x05),
		  REG_DVFS_CTRL_CFG_CHNL02);

	return 0;
}

static int hwdvfs_trigger_lit(unsigned int scalecode00, bool sync, bool force)
{
	unsigned int regval = 0, i = 0;
	const unsigned int RETRY_MAX = 150;
	unsigned int scalecodeing;

	if (scalecode00 >= SCALE_TAB_EACH_NUM) {
		P_ERR("CHNL0 illegal scalecode\n");
		return -EINVAL;
	}

	if (!cpufreqhw->enabled) {
		P_ERR("CHNL0:lit is disabled\n");
		return -ENODEV;
	}

	regval = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL00);
	scalecodeing = _dvfs_rd_(REG_DVFS_CTRL_SW_TRG_CHNL00) &
		BIT_DVFS_CTRL_SW_SCL_CHNL00(0xf);
#ifdef SPRD_CPUFREQ_IRQ_ENABLE
	if (scalecodeing == scalecode00
	    && cpufreqhw->triggered[CPUFREQHW_CHNL00]
	    && !force) {
		P_DBG("CHNL0 reject same idx-%u int-%u\n", scalecode00,
		      REG2VAL(regval, BIT_DVFS_CTRL_IRQ_DONE_CHNL00));
		return 0;
	}

	if (sprd_cpufreqhw_try_lock(CPUFREQHW_CHNL00)) {
		if (cpufreqhw->busy[CPUFREQHW_CHNL00] == 0)
			P_DBG("CHNL0 busy! cur-%u ing-%u to-%u\n",
			REG2VAL(regval, BIT_DVFS_CTRL_DONE_SCL_CHNL00(0xf)),
			scalecodeing,
			scalecode00);
		else
			P_DBG_INT("CHNL0 busy! cur-%u ing-%u to-%u\n",
			REG2VAL(regval, BIT_DVFS_CTRL_DONE_SCL_CHNL00(0xf)),
			scalecodeing,
			scalecode00);
		if (cpufreqhw->busy[CPUFREQHW_CHNL00] == 0)
			cpufreqhw->busy[CPUFREQHW_CHNL00] =
			    jiffies + SPRD_CPUFREQHW_BUSY_DURATOIN;
		if (cpufreqhw->busy[CPUFREQHW_CHNL00] &&
		    time_after(jiffies,
			       cpufreqhw->busy[CPUFREQHW_CHNL00])) {
			P_ERR("CHNL0 busy expired! cur-%u ing-%u to-%u\n",
			REG2VAL(regval, BIT_DVFS_CTRL_DONE_SCL_CHNL00(0xf)),
			scalecodeing,
			scalecode00);
			sprd_cpufreqhw_unlock(CPUFREQHW_CHNL00, false);
		}
		return 0;
	}
#endif

	if (force)
		P_DBG("CHNL0 idx %d START\n", scalecode00);

	_dvfs_wr_(regval |
		BIT_DVFS_CTRL_CONFLICT1_CHNL00 |
		BIT_DVFS_CTRL_CONFLICT0_CHNL00 |
		BIT_DVFS_CTRL_IRQ_VREAD_MIS_CHNL00 |
		BIT_DVFS_CTRL_IRQ_TO_CHNL00 |
		BIT_DVFS_CTRL_IRQ_DONE_CHNL00,
		REG_DVFS_CTRL_STS_CHNL00);

#ifdef SPRD_CPUFREQ_I2C_LOCK
	if (cpufreqhw->on_i2c[CPUFREQHW_CHNL00] &&
	cpufreqhw->i2c_client != NULL)
		reinit_completion(&cpufreqhw->i2c_done);
#endif
	_dvfs_wr_(BIT_DVFS_CTRL_SW_TRG_CHNL00 |
		BIT_DVFS_CTRL_SW_SCL_CHNL00(scalecode00),
		REG_DVFS_CTRL_SW_TRG_CHNL00);

	cpufreqhw->triggered[CPUFREQHW_CHNL00] = true;
#ifdef SPRD_CPUFREQ_I2C_LOCK
	sprd_cpufreqhw_i2c_unlock_sync(CPUFREQHW_CHNL00);
#endif
#ifdef SPRD_CPUFREQ_IRQ_ENABLE
	if (sync) {
		while (i < RETRY_MAX &&
		       atomic_read(&cpufreqhw->
				   dvfs_state[CPUFREQHW_CHNL00]) !=
		       CPUFREQHW_STATE_DONE) {
			i++;
			udelay(100);
		}
		if (i >= RETRY_MAX) {
			regval = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL00);
			P_ERR("CHN0=%d fail (0x%x), DONE=0x%x,SCALE=0x%x\n",
			     scalecode00, regval,
			     REG2VAL(regval, BIT_DVFS_CTRL_IRQ_DONE_CHNL00),
			     REG2VAL(regval,
				BIT_DVFS_CTRL_DONE_SCL_CHNL00(0xf)));
			return -EBUSY;
		}
		P_DBG("index(%d)retry(%d)ok\n", scalecode00, i);
	}
#endif

	return 0;
}

static int hwdvfs_trigger_big(unsigned int scalecode01, bool sync, bool force)
{
	unsigned int regval = 0, i = 0;
	const unsigned int RETRY_MAX = 100;
	unsigned int scalecodeing;

	if (scalecode01 >= SCALE_TAB_EACH_NUM) {
		P_ERR("CHNL1 illegal scalecode\n");
		return -EINVAL;
	}

	if (!cpufreqhw->enabled) {
		P_ERR("CHNL1:big is disabled\n");
		return -ENODEV;
	}

	regval = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL01);
	scalecodeing = _dvfs_rd_(REG_DVFS_CTRL_SW_TRG_CHNL01) &
		BIT_DVFS_CTRL_SW_SCL_CHNL01(0xf);
#ifdef SPRD_CPUFREQ_IRQ_ENABLE
	if (scalecodeing == scalecode01
	    && cpufreqhw->triggered[CPUFREQHW_CHNL01]
	    && !force) {
		P_DBG("CHNL1 reject same idx-%u int-%u\n", scalecode01,
		      REG2VAL(regval, BIT_DVFS_CTRL_IRQ_DONE_CHNL01));
		return 0;
	}

	if (sprd_cpufreqhw_try_lock(CPUFREQHW_CHNL01)) {
		if (cpufreqhw->busy[CPUFREQHW_CHNL01] == 0)
			P_DBG("CHNL1 busy! cur-%u ing-%u to-%u\n",
			REG2VAL(regval, BIT_DVFS_CTRL_DONE_SCL_CHNL01(0xf)),
			scalecodeing,
			scalecode01);
		else
			P_DBG_INT("CHNL1 busy! cur-%u ing-%u to-%u\n",
			REG2VAL(regval, BIT_DVFS_CTRL_DONE_SCL_CHNL01(0xf)),
			scalecodeing,
			scalecode01);
		if (cpufreqhw->busy[CPUFREQHW_CHNL01] == 0)
			cpufreqhw->busy[CPUFREQHW_CHNL01] =
			    jiffies + SPRD_CPUFREQHW_BUSY_DURATOIN;
		if (cpufreqhw->busy[CPUFREQHW_CHNL01] &&
		    time_after(jiffies,
			       cpufreqhw->busy[CPUFREQHW_CHNL01])) {
			P_ERR("CHNL1 busy expired! cur-%u ing-%u to-%u\n",
			REG2VAL(regval, BIT_DVFS_CTRL_DONE_SCL_CHNL01(0xf)),
			scalecodeing,
			scalecode01);
			sprd_cpufreqhw_unlock(CPUFREQHW_CHNL01, false);
		}
		return 0;
	}
#endif
	if (force)
		P_DBG_INT("do index %d\n", scalecode01);

	_dvfs_wr_(regval |
		BIT_DVFS_CTRL_CONFLICT1_CHNL01 |
		BIT_DVFS_CTRL_CONFLICT0_CHNL01 |
		BIT_DVFS_CTRL_IRQ_VREAD_MIS_CHNL01 |
		BIT_DVFS_CTRL_IRQ_TO_CHNL01 |
		BIT_DVFS_CTRL_IRQ_DONE_CHNL01,
		REG_DVFS_CTRL_STS_CHNL01);

#ifdef SPRD_CPUFREQ_I2C_LOCK
	if (cpufreqhw->on_i2c[CPUFREQHW_CHNL01] &&
	cpufreqhw->i2c_client != NULL)
		reinit_completion(&cpufreqhw->i2c_done);
#endif
	_dvfs_wr_(BIT_DVFS_CTRL_SW_TRG_CHNL01 |
		BIT_DVFS_CTRL_SW_SCL_CHNL01(scalecode01),
		REG_DVFS_CTRL_SW_TRG_CHNL01);

	cpufreqhw->triggered[CPUFREQHW_CHNL01] = true;
#ifdef SPRD_CPUFREQ_I2C_LOCK
	sprd_cpufreqhw_i2c_unlock_sync(CPUFREQHW_CHNL01);
#endif
#ifdef SPRD_CPUFREQ_IRQ_ENABLE
	if (sync) {
		while (i < RETRY_MAX &&
		       atomic_read(&cpufreqhw->
				   dvfs_state[CPUFREQHW_CHNL01]) !=
		       CPUFREQHW_STATE_DONE) {
			i++;
			udelay(100);
		}
		if (i >= RETRY_MAX) {
			regval = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL01);
			P_DBG("CHN1=%d fail (0x%x), DONE=0x%x,SCALE=0x%x\n",
			      scalecode01, regval,
			      REG2VAL(regval, BIT_DVFS_CTRL_IRQ_DONE_CHNL01),
			      REG2VAL(regval,
				BIT_DVFS_CTRL_DONE_SCL_CHNL01(0xf)));
			return -EBUSY;
		}
		P_DBG("index(%d)retry(%d)ok\n", scalecode01, i);
	}
#endif
	return 0;
}

static int hwdvfs_trigger_scu(unsigned int scalecode02, bool sync, bool force)
{
	unsigned int regval = 0, i = 0;
	const unsigned int RETRY_MAX = 50;
	unsigned int scalecodeing;

	if (scalecode02 >= SCALE_TAB_EACH_NUM) {
		P_ERR("CHNL2 illegal scalecode\n");
		return -EINVAL;
	}

	if (!cpufreqhw->enabled) {
		P_ERR("CHNL2:scu is disabled\n");
		return -ENODEV;
	}

	regval = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL02);
	scalecodeing = _dvfs_rd_(REG_DVFS_CTRL_SW_TRG_CHNL02) &
		BIT_DVFS_CTRL_SW_SCL_CHNL02(0xf);
#ifdef SPRD_CPUFREQ_IRQ_ENABLE
	if (scalecodeing == scalecode02
	    && cpufreqhw->triggered[CPUFREQHW_CHNL02]
	    && !force) {
		P_DBG("CHNL2 reject same idx-%u int-%u\n", scalecode02,
		      REG2VAL(regval, BIT_DVFS_CTRL_IRQ_DONE_CHNL02));
		return 0;
	}

	if (sprd_cpufreqhw_try_lock(CPUFREQHW_CHNL02)) {
		if (cpufreqhw->busy[CPUFREQHW_CHNL02] == 0)
			P_DBG("CHNL2 busy! cur-%u ing-%u to-%u\n",
			REG2VAL(regval, BIT_DVFS_CTRL_DONE_SCL_CHNL02(0xf)),
			scalecodeing,
			scalecode02);
		else
			P_DBG_INT("CHNL2 busy! cur-%u ing-%u to-%u\n",
			REG2VAL(regval, BIT_DVFS_CTRL_DONE_SCL_CHNL02(0xf)),
			scalecodeing,
			scalecode02);
		if (cpufreqhw->busy[CPUFREQHW_CHNL02] == 0)
			cpufreqhw->busy[CPUFREQHW_CHNL02] =
			    jiffies + SPRD_CPUFREQHW_BUSY_DURATOIN;
		if (cpufreqhw->busy[CPUFREQHW_CHNL02] &&
		    time_after(jiffies,
			       cpufreqhw->busy[CPUFREQHW_CHNL02])) {
			P_ERR("CHNL2 busy expired! cur-%u ing-%u to-%u\n",
			REG2VAL(regval, BIT_DVFS_CTRL_DONE_SCL_CHNL02(0xf)),
			scalecodeing,
			scalecode02);
			sprd_cpufreqhw_unlock(CPUFREQHW_CHNL02, false);
		}
		return 0;
	}
#endif
	if (force)
		P_DBG("CHNL2 idx %d START\n", scalecode02);

	_dvfs_wr_(regval |
		BIT_DVFS_CTRL_CONFLICT1_CHNL02 |
		BIT_DVFS_CTRL_CONFLICT0_CHNL02 |
		BIT_DVFS_CTRL_IRQ_VREAD_MIS_CHNL02 |
		BIT_DVFS_CTRL_IRQ_TO_CHNL02 |
		BIT_DVFS_CTRL_IRQ_DONE_CHNL02,
		REG_DVFS_CTRL_STS_CHNL02);

#ifdef SPRD_CPUFREQ_I2C_LOCK
	if (cpufreqhw->on_i2c[CPUFREQHW_CHNL02] &&
	cpufreqhw->i2c_client != NULL)
		reinit_completion(&cpufreqhw->i2c_done);
#endif
	_dvfs_wr_(BIT_DVFS_CTRL_SW_TRG_CHNL02 |
		BIT_DVFS_CTRL_SW_SCL_CHNL02(scalecode02),
		REG_DVFS_CTRL_SW_TRG_CHNL02);

	cpufreqhw->triggered[CPUFREQHW_CHNL02] = true;
#ifdef SPRD_CPUFREQ_I2C_LOCK
	sprd_cpufreqhw_i2c_unlock_sync(CPUFREQHW_CHNL02);
#endif
#ifdef SPRD_CPUFREQ_IRQ_ENABLE
	if (sync) {
		while (i < RETRY_MAX &&
		       atomic_read(&cpufreqhw->
				   dvfs_state[CPUFREQHW_CHNL02]) !=
		       CPUFREQHW_STATE_DONE) {
			i++;
			udelay(100);
		}
		if (i >= RETRY_MAX) {
			regval = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL02);
			P_DBG("CHN2=%d fail (0x%x), DONE=0x%x,SCALE=0x%x\n",
			      scalecode02, regval,
			      REG2VAL(regval, BIT_DVFS_CTRL_IRQ_DONE_CHNL02),
			      REG2VAL(regval,
				BIT_DVFS_CTRL_DONE_SCL_CHNL02(0xf)));
			return -EBUSY;
		}
		P_DBG("index(%d)retry(%d)ok\n", scalecode02, i);
	}
#endif
	return 0;
}

static int hwdvfs_trigger_big_scu(unsigned int scalecode01, bool sync,
				  bool force)
{
	unsigned int regval1 = 0, regval2 = 2, i = 0;
	const unsigned int RETRY_MAX = 100;
	unsigned int scalecodeing;

	if (scalecode01 >= SCALE_TAB_EACH_NUM) {
		P_ERR("CHNL1 CHNL2: illegal scalecode\n");
		return -EINVAL;
	}

	if (!cpufreqhw->enabled) {
		P_ERR("CHNL1 CHNL2:big is disabled\n");
		return -ENODEV;
	}

	regval1 = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL01);
	regval2 = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL02);
	scalecodeing = _dvfs_rd_(REG_DVFS_CTRL_SW_TRG_CHNL01) &
		BIT_DVFS_CTRL_SW_SCL_CHNL01(0xf);

#ifdef SPRD_CPUFREQ_IRQ_ENABLE
	if (scalecodeing == scalecode01
	    && cpufreqhw->triggered[CPUFREQHW_CHNL01]
	    && !force) {
		P_DBG("CHNL1 CHNL2 reject same idx-%u int-%u\n", scalecode01,
		      REG2VAL(regval1, BIT_DVFS_CTRL_IRQ_DONE_CHNL01));
		return 0;
	}

	if (sprd_cpufreqhw_try_lock_big_scu()) {
		if (cpufreqhw->busy[CPUFREQHW_CHNL01] == 0)
			P_DBG("CHNL1 CHNL2 busy! cur-%u ing-%u to-%u\n",
			REG2VAL(regval1, BIT_DVFS_CTRL_DONE_SCL_CHNL01(0xf)),
			scalecodeing,
			scalecode01);
		else
			P_DBG_INT("CHNL1 CHNL2 busy! cur-%u ing-%u to-%u\n",
			REG2VAL(regval1, BIT_DVFS_CTRL_DONE_SCL_CHNL01(0xf)),
			scalecodeing,
			scalecode01);
		if (cpufreqhw->busy[CPUFREQHW_CHNL01] == 0)
			cpufreqhw->busy[CPUFREQHW_CHNL01] =
			    jiffies + SPRD_CPUFREQHW_BUSY_DURATOIN;
		if (cpufreqhw->busy[CPUFREQHW_CHNL01] &&
		    time_after(jiffies,
			       cpufreqhw->busy[CPUFREQHW_CHNL01])) {
			P_ERR("CHNL1 CHNL2 busy expired! cur-%u ing-%u to-%u\n",
			REG2VAL(regval1, BIT_DVFS_CTRL_DONE_SCL_CHNL01(0xf)),
			scalecodeing,
			scalecode01);
			sprd_cpufreqhw_unlock(CPUFREQHW_CHNL01, false);
			sprd_cpufreqhw_unlock(CPUFREQHW_CHNL02, false);
		}
		return 0;
	}
#endif
	if (force)
		P_DBG("CHNL1 CHNL2 idx %d START\n", scalecode01);

	_dvfs_wr_(regval1 |
		BIT_DVFS_CTRL_CONFLICT1_CHNL01 |
		BIT_DVFS_CTRL_CONFLICT0_CHNL01 |
		BIT_DVFS_CTRL_IRQ_VREAD_MIS_CHNL01 |
		BIT_DVFS_CTRL_IRQ_TO_CHNL01 |
		BIT_DVFS_CTRL_IRQ_DONE_CHNL01,
		REG_DVFS_CTRL_STS_CHNL01);
	_dvfs_wr_(regval2 |
		BIT_DVFS_CTRL_CONFLICT1_CHNL02 |
		BIT_DVFS_CTRL_CONFLICT0_CHNL02 |
		BIT_DVFS_CTRL_IRQ_VREAD_MIS_CHNL02 |
		BIT_DVFS_CTRL_IRQ_TO_CHNL02 |
		BIT_DVFS_CTRL_IRQ_DONE_CHNL02,
		REG_DVFS_CTRL_STS_CHNL02);

#ifdef SPRD_CPUFREQ_I2C_LOCK
	if (cpufreqhw->on_i2c[CPUFREQHW_CHNL01] &&
	cpufreqhw->i2c_client != NULL)
		reinit_completion(&cpufreqhw->i2c_done);
#endif
	_dvfs_wr_(BIT_DVFS_CTRL_SW_TRG_CHNL02 |
		  BIT_DVFS_CTRL_SW_SCL_CHNL02(scalecode01),
		  REG_DVFS_CTRL_SW_TRG_CHNL02);
	_dvfs_wr_(BIT_DVFS_CTRL_SW_TRG_CHNL01 |
		  BIT_DVFS_CTRL_SW_SCL_CHNL01(scalecode01),
		  REG_DVFS_CTRL_SW_TRG_CHNL01);

	cpufreqhw->triggered[CPUFREQHW_CHNL01] = true;
	cpufreqhw->triggered[CPUFREQHW_CHNL02] = true;
#ifdef SPRD_CPUFREQ_I2C_LOCK
	sprd_cpufreqhw_i2c_unlock_sync(CPUFREQHW_CHNL01);
#endif
#ifdef SPRD_CPUFREQ_IRQ_ENABLE
	if (sync) {
		while (i < RETRY_MAX &&
		       atomic_read(&cpufreqhw->
				   dvfs_state[CPUFREQHW_CHNL01]) !=
		       CPUFREQHW_STATE_DONE) {
			i++;
			udelay(100);
		}
		if (i >= RETRY_MAX) {
			regval1 = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL01);
			P_DBG("CHN1=%d fail (0x%x), DONE=0x%x,SCALE=0x%x\n",
			      scalecode01, regval1,
			      REG2VAL(regval1, BIT_DVFS_CTRL_IRQ_DONE_CHNL01),
			      REG2VAL(regval1,
				BIT_DVFS_CTRL_DONE_SCL_CHNL01(0xf)));
			return -EBUSY;
		}
		P_DBG("index(%d)retry(%d)ok\n", scalecode01, i);
	}
#endif
	return 0;
}
#ifdef SPRD_CPUFREQ_IRQ_ENABLE
static irqreturn_t sprd_cpufreqhw_isr(int irq, void *dev_id)
{
	unsigned int regval00 = 0, regval01 = 0, regval02 = 0;

	regval00 = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL00);
	if (regval00 & BIT_DVFS_CTRL_IRQ_DONE_CHNL00)
		_dvfs_wr_(regval00 |
			BIT_DVFS_CTRL_CONFLICT1_CHNL00 |
			BIT_DVFS_CTRL_CONFLICT0_CHNL00 |
			BIT_DVFS_CTRL_IRQ_VREAD_MIS_CHNL00 |
			BIT_DVFS_CTRL_IRQ_TO_CHNL00 |
			BIT_DVFS_CTRL_IRQ_DONE_CHNL00,
			REG_DVFS_CTRL_STS_CHNL00);
	regval01 = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL01);
	if (regval01 & BIT_DVFS_CTRL_IRQ_DONE_CHNL01)
		_dvfs_wr_(regval01 |
			BIT_DVFS_CTRL_CONFLICT1_CHNL01 |
			BIT_DVFS_CTRL_CONFLICT0_CHNL01 |
			BIT_DVFS_CTRL_IRQ_VREAD_MIS_CHNL01 |
			BIT_DVFS_CTRL_IRQ_TO_CHNL01 |
			BIT_DVFS_CTRL_IRQ_DONE_CHNL01,
			REG_DVFS_CTRL_STS_CHNL01);
	regval02 = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL02);
	if (regval02 & BIT_DVFS_CTRL_IRQ_DONE_CHNL02)
		_dvfs_wr_(regval02 |
			BIT_DVFS_CTRL_CONFLICT1_CHNL02 |
			BIT_DVFS_CTRL_CONFLICT0_CHNL02 |
			BIT_DVFS_CTRL_IRQ_VREAD_MIS_CHNL02 |
			BIT_DVFS_CTRL_IRQ_TO_CHNL02 |
			BIT_DVFS_CTRL_IRQ_DONE_CHNL02,
			REG_DVFS_CTRL_STS_CHNL02);

	if (regval00 & BIT_DVFS_CTRL_IRQ_DONE_CHNL00)
		P_DBG_INT("CHNL0 0x%x DONE\n",
			  REG2VAL(regval00,
				BIT_DVFS_CTRL_DONE_SCL_CHNL00(0xf)));

	if (regval01 & BIT_DVFS_CTRL_IRQ_DONE_CHNL01)
		P_DBG_INT("CHNL1 0x%x DONE\n",
			  REG2VAL(regval01,
				BIT_DVFS_CTRL_DONE_SCL_CHNL01(0xf)));

	if (regval02 & BIT_DVFS_CTRL_IRQ_DONE_CHNL02)
		P_DBG_INT("CHNL2 0x%x DONE\n",
			  REG2VAL(regval02,
				BIT_DVFS_CTRL_DONE_SCL_CHNL02(0xf)));

	/*CHNL00: ERROR INTERRUPT*/
	if (regval00 & BIT_DVFS_CTRL_CONFLICT1_CHNL00)
		P_ERR("%s: CHNL0 CONFLICT1\n", __func__);

	if (regval00 & BIT_DVFS_CTRL_CONFLICT0_CHNL00)
		P_ERR("%s: CHNL0 CONFLICT0\n", __func__);

	if (regval00 & BIT_DVFS_CTRL_IRQ_TO_CHNL00)
		P_ERR("%s: CHNL0 TIMEOUT\n", __func__);

	if (regval00 & BIT_DVFS_CTRL_IRQ_VREAD_MIS_CHNL00)
		P_ERR("%s: CHNL0 VREAD_MIS in 0x%x\n", __func__,
			REG2VAL(regval00,
				BIT_DVFS_CTRL_DONE_SCL_CHNL00(0xf)));
	if ((regval00 & BIT_DVFS_CTRL_IRQ_DONE_CHNL00) ||
	(regval00 & BIT_DVFS_CTRL_CONFLICT1_CHNL00) ||
	(regval00 & BIT_DVFS_CTRL_IRQ_TO_CHNL00))
		sprd_cpufreqhw_unlock(CPUFREQHW_CHNL00, true);

	/*CHNL01: ERROR INTERRUPT*/
	if (regval01 & BIT_DVFS_CTRL_CONFLICT1_CHNL01)
		P_ERR("%s: CHNL1 CONFLICT1\n", __func__);

	if (regval01 & BIT_DVFS_CTRL_CONFLICT0_CHNL01)
		P_ERR("%s: CHNL1 CONFLICT0\n", __func__);

	if (regval01 & BIT_DVFS_CTRL_IRQ_TO_CHNL01)
		P_ERR("%s: CHNL1 TIMEOUT\n", __func__);

	if (regval01 & BIT_DVFS_CTRL_IRQ_VREAD_MIS_CHNL01)
		P_ERR("%s: CHNL1 VREAD_MIS in 0x%x\n", __func__,
			REG2VAL(regval01,
				BIT_DVFS_CTRL_DONE_SCL_CHNL01(0xf)));
	if ((regval01 & BIT_DVFS_CTRL_IRQ_DONE_CHNL01) ||
	(regval01 & BIT_DVFS_CTRL_CONFLICT1_CHNL01) ||
	(regval01 & BIT_DVFS_CTRL_IRQ_TO_CHNL01))
		sprd_cpufreqhw_unlock(CPUFREQHW_CHNL01, true);

	/*CHNL02: ERROR INTERRUPT*/
	if (regval02 & BIT_DVFS_CTRL_CONFLICT1_CHNL02)
		P_ERR("%s: CHNL2 CONFLICT1\n", __func__);

	if (regval02 & BIT_DVFS_CTRL_CONFLICT0_CHNL02)
		P_ERR("%s: CHNL2 CONFLICT0\n", __func__);

	if (regval02 & BIT_DVFS_CTRL_IRQ_TO_CHNL02)
		P_ERR("%s: CHNL2 TIMEOUT\n", __func__);

	if (regval02 & BIT_DVFS_CTRL_IRQ_VREAD_MIS_CHNL02)
		P_ERR("%s: CHNL2 VREAD_MIS in 0x%x\n", __func__,
			REG2VAL(regval02,
				BIT_DVFS_CTRL_DONE_SCL_CHNL02(0xf)));
	if ((regval02 & BIT_DVFS_CTRL_IRQ_DONE_CHNL02) ||
	(regval02 & BIT_DVFS_CTRL_CONFLICT1_CHNL02) ||
	(regval02 & BIT_DVFS_CTRL_IRQ_TO_CHNL02))
		sprd_cpufreqhw_unlock(CPUFREQHW_CHNL02, true);

	return IRQ_HANDLED;
}
#endif
/**
 * sprd_cpufreqhw_table_store - store freq&volt table for cluster
 * @cluster: 0-cluster0, 1-cluster1, 3-scu
 *
 * This is the last known freq, without actually getting it from the driver.
 * Return value will be same as what is shown in scaling_cur_freq in sysfs.
 */
int sprd_cpufreqhw_opp_add(unsigned int cluster, unsigned long hz_freq,
			   unsigned long u_volt, int idx_volt)
{
	int ret = 0;
	unsigned int idx_freq;

	if (cpufreqhw == NULL || !cpufreqhw->probed) {
		P_DBG("opp_add exits due to no HW DVFS\n");
		return -ENODEV;
	}

	if (cluster > CPUFREQHW_CHNL_MAX) {
		P_DBG("opp_add cluster %u idx %d error!\n", cluster, idx_volt);
		return -EINVAL;
	}

	if (cluster == CPUFREQHW_CHNL_MAX)
		cluster = CPUFREQHW_CHNL02;

	if (atomic_read(&cpufreqhw->dvfs_state[cluster]) ==
	CPUFREQHW_STATE_RUNNING) {
		P_ERR("cluster%d cpufreqhw_opp_add error!busy!\n", cluster);
		return -EBUSY;
	}

	idx_freq = (hz_freq <= 768000000 ? 0 : (idx_volt - 1));
	switch (cluster) {
	case CPUFREQHW_CHNL00:
		_dvfs_wr_(HW_DVFS_TAB_CLUSTER0(hz_freq, u_volt, idx_freq,
				cpufreqhw->dcdc_index[CPUFREQHW_CHNL00]),
			  REG_DVFS_CTRL_CHNL00_SCALE00 +
			  (cluster * SCALE_TAB_EACH_NUM + idx_volt) * 0x4);
		if (hz_freq > 768000000)
			regmap_write(cpufreqhw->anlg_phy_g4_ctrl,
			REG_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_DVFS_0 +
			(cluster * MPLL_HW_DVFS_EACH_NUM +
			idx_freq) * 0x4,
			mpll_hw_dvfs_table(hz_freq / 1000));

		P_INF("cluster%d hw_opp[freq-0x%x & volt-0x%x]\n",
			cluster,
			mpll_hw_dvfs_table(hz_freq / 1000),
			HW_DVFS_TAB_CLUSTER0(hz_freq, u_volt, idx_freq,
			cpufreqhw->dcdc_index[CPUFREQHW_CHNL00]));
		break;
	case CPUFREQHW_CHNL01:
		_dvfs_wr_(HW_DVFS_TAB_CLUSTER1(hz_freq, u_volt, idx_freq,
				cpufreqhw->dcdc_index[CPUFREQHW_CHNL01]),
			  REG_DVFS_CTRL_CHNL00_SCALE00 +
			  (cluster * SCALE_TAB_EACH_NUM + idx_volt) * 0x4);
		if (hz_freq > 768000000)
			regmap_write(cpufreqhw->anlg_phy_g4_ctrl,
			REG_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_DVFS_0 +
			(cluster * MPLL_HW_DVFS_EACH_NUM +
			idx_freq) * 0x4,
			mpll_hw_dvfs_table(hz_freq / 1000));

		P_INF("cluster%d hw_opp[freq-0x%x & volt-0x%x]\n",
			cluster,
			mpll_hw_dvfs_table(hz_freq / 1000),
			HW_DVFS_TAB_CLUSTER1(hz_freq, u_volt, idx_freq,
			cpufreqhw->dcdc_index[CPUFREQHW_CHNL01]));
		break;
	case CPUFREQHW_CHNL02:
		_dvfs_wr_(HW_DVFS_TAB_CLUSTER2(hz_freq, u_volt, idx_freq,
				cpufreqhw->dcdc_index[CPUFREQHW_CHNL02]),
			  REG_DVFS_CTRL_CHNL00_SCALE00 +
			  (cluster * SCALE_TAB_EACH_NUM + idx_volt) * 0x4);
		if (hz_freq > 768000000)
			regmap_write(cpufreqhw->anlg_phy_g4_ctrl,
			REG_ANLG_PHY_G4_ANALOG_MPLL_THM_TOP_MPLL0_DVFS_0 +
			(cluster * MPLL_HW_DVFS_EACH_NUM +
			idx_freq) * 0x4,
			mpll_hw_dvfs_table(hz_freq / 1000));

		P_INF("cluster%d hw_opp[freq-0x%x & volt-0x%x]\n",
			cluster,
			mpll_hw_dvfs_table(hz_freq / 1000),
			HW_DVFS_TAB_CLUSTER2(hz_freq, u_volt, idx_freq,
			cpufreqhw->dcdc_index[CPUFREQHW_CHNL02]));
		break;
	default:
		ret = -ENODEV;
		P_ERR("cpufreqhw_opp_add cluster %u error!\n", cluster);
		break;
	}

	if (ret == 0) {
		cpufreqhw->freqvolt[cluster][idx_volt].freq = hz_freq;
		cpufreqhw->freqvolt[cluster][idx_volt].volt = u_volt;
		if (idx_volt > cpufreqhw->idx_max[cluster])
			cpufreqhw->idx_max[cluster] = idx_volt;
	}

	return ret;
}
EXPORT_SYMBOL(sprd_cpufreqhw_opp_add);

/*
 * @idx:        0 points to min freq, ascending order
*/
int sprd_cpufreqhw_set_target(unsigned int cluster, unsigned int idx_volt)
{
	int ret = 0;

	if (cpufreqhw == NULL || !cpufreqhw->probed) {
		P_DBG("opp_add exits due to no HW DVFS\n");
		return -ENODEV;
	}

	if (cluster == CPUFREQHW_CHNL_MAX)
		cluster = CPUFREQHW_CHNL02;

	if (cluster > CPUFREQHW_CHNL_MAX ||
	    idx_volt > cpufreqhw->idx_max[cluster]) {
		P_DBG("opp_add cluster%u idx%d error\n", cluster, idx_volt);
		return -EINVAL;
	}

	P_CRIT("CHNL%u idx %u START\n", cluster, idx_volt);

	/*trigger freq&volt */
	if (cluster == CPUFREQHW_CHNL00)
		ret = hwdvfs_trigger_lit(idx_volt, false, false);
	else if (cluster == CPUFREQHW_CHNL01)
		ret = hwdvfs_trigger_big_scu(idx_volt, false, false);
	else if (cluster == CPUFREQHW_CHNL02)
		ret = 0;
	else
		ret = -ENODEV;

	return ret;
}
EXPORT_SYMBOL(sprd_cpufreqhw_set_target);

unsigned int sprd_cpufreqhw_get(int cluster)
{
	unsigned int regval = 0, reg = 0, freq_khz = 0;

	if (cpufreqhw == NULL || !cpufreqhw->probed) {
		P_DBG("%s:exits due to no HW DVFS\n", __func__);
		return -ENODEV;
	}

	if (cluster > CPUFREQHW_CHNL_MAX) {
		P_DBG("%s cluster%u  error!\n", __func__, cluster);
		return -EINVAL;
	}

	if (cluster == CPUFREQHW_CHNL_MAX)
		cluster = CPUFREQHW_CHNL02;

	if (cpufreqhw->enabled &&
	cpufreqhw->triggered[cluster]) {
		switch (cluster) {
		case CPUFREQHW_CHNL00:
			regval = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL00);
			reg = regval;
			regval = REG2VAL(regval,
				BIT_DVFS_CTRL_DONE_SCL_CHNL00(0xf));
			break;
		case CPUFREQHW_CHNL01:
			regval = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL01);
			reg = regval;
			regval = REG2VAL(regval,
				BIT_DVFS_CTRL_DONE_SCL_CHNL01(0xf));
			break;
		case CPUFREQHW_CHNL02:
			regval = _dvfs_rd_(REG_DVFS_CTRL_STS_CHNL02);
			reg = regval;
			regval = REG2VAL(regval,
				BIT_DVFS_CTRL_DONE_SCL_CHNL02(0xf));
			break;
		default:
			break;
		}
	} else {
		if (cluster == CPUFREQHW_CHNL00)
			regval = SPRD_CPUFREQHW_CHNL00_DEFAULT_FREQ;
		else if (cluster == CPUFREQHW_CHNL01)
			regval = SPRD_CPUFREQHW_CHNL01_DEFAULT_FREQ;
		else if (cluster == CPUFREQHW_CHNL02)
			regval = SPRD_CPUFREQHW_CHNL02_DEFAULT_FREQ;
	}

	if (regval >  cpufreqhw->idx_max[cluster]) {
		if (cluster == CPUFREQHW_CHNL00)
			regval = SPRD_CPUFREQHW_CHNL00_DEFAULT_FREQ;
		else if (cluster == CPUFREQHW_CHNL01)
			regval = SPRD_CPUFREQHW_CHNL01_DEFAULT_FREQ;
		else if (cluster == CPUFREQHW_CHNL02)
			regval = SPRD_CPUFREQHW_CHNL02_DEFAULT_FREQ;
	}

	freq_khz = cpufreqhw->freqvolt[cluster][regval].freq / 1000;

	P_DBG("[%d,%ukhz,en-%d:%d, reg-0x%x, val-0x%x]\n",
		cluster, freq_khz,
		cpufreqhw->enabled,
		cpufreqhw->triggered[cluster],
		reg, regval);

	return freq_khz;
}
EXPORT_SYMBOL(sprd_cpufreqhw_get);

bool sprd_cpufreqhw_enable(int cluster, bool en)
{
	unsigned int regval0, regval1;

	if (cpufreqhw == NULL || !cpufreqhw->probed) {
		P_DBG("opp_add exits due to no HW DVFS\n");
		return false;
	}

	if (cluster > CPUFREQHW_CHNL_MAX) {
		P_DBG("%s cluster%u  error!\n", __func__, cluster);
		return false;
	}

	if (cluster == CPUFREQHW_CHNL_MAX)
		cluster = CPUFREQHW_CHNL02;

	P_DBG("cluster %u en %u\n", cluster, en);

	regval1 = _dvfs_rd_(REG_DVFS_CTRL_SW_CHNL_EN);
	if (en) {
		switch (cluster) {
		case CPUFREQHW_CHNL00:
			if (!(regval1 & SW_CHNL00_EN_MSK))
				_dvfs_wr_(regval1 |
					  VAL2REG(0x1, SW_CHNL00_EN_MSK),
					  REG_DVFS_CTRL_SW_CHNL_EN);
			P_INF("SW_CHNL00_EN\n");
			break;
		case CPUFREQHW_CHNL01:
			if (!(regval1 & SW_CHNL01_EN_MSK))
				_dvfs_wr_(regval1 |
					  VAL2REG(0x1, SW_CHNL01_EN_MSK),
					  REG_DVFS_CTRL_SW_CHNL_EN);
			P_INF("SW_CHNL01_EN\n");
			break;
		case CPUFREQHW_CHNL02:
			if (!(regval1 & SW_CHNL02_EN_MSK))
				_dvfs_wr_(regval1 |
					  VAL2REG(0x1, SW_CHNL02_EN_MSK),
					  REG_DVFS_CTRL_SW_CHNL_EN);
			P_INF("SW_CHNL02_EN\n");
			break;
		default:
			P_ERR("%s cluster %u error!\n", __func__, cluster);
			break;
		}

		regval0 = _dvfs_rd_(REG_DVFS_CTRL_HW_DVFS_SEL);
		regval1 = _dvfs_rd_(REG_DVFS_CTRL_SW_CHNL_EN);
		if (!(regval0 & BIT_DVFS_CTRL_HW_DVFS_SEL) &&
		    (regval1 & SW_CHNL00_EN_MSK)
		    && (regval1 & SW_CHNL01_EN_MSK)
		    && (regval1 & SW_CHNL02_EN_MSK)) {
			_dvfs_wr_(DVFS_CTRL_MAGIC_NUM_LOCK,
				REG_DVFS_CTRL_MAGIC_NUM);
			_dvfs_wr_(VAL2REG(0x1, BIT_DVFS_CTRL_HW_DVFS_SEL),
				REG_DVFS_CTRL_HW_DVFS_SEL);
			cpufreqhw->enabled = true;
			hwdvfs_trigger_lit(SPRD_CPUFREQHW_CHNL00_DEFAULT_FREQ,
					true, true);
			hwdvfs_trigger_big_scu(
					SPRD_CPUFREQHW_CHNL01_DEFAULT_FREQ,
					true, true);
			P_INF("ENABLE HWDVFS!\n");
		}
	} else if (atomic_read(&cpufreqhw_suspend) != 1) {
		switch (cluster) {
		case CPUFREQHW_CHNL00:
			if (!hwdvfs_trigger_lit(0, false, false))
				P_INF("SW_CHNL00_DISABLE\n");
			break;
		case CPUFREQHW_CHNL01:
			if (!hwdvfs_trigger_big(0, false, false))
				P_INF("SW_CHNL01_DISABLE\n");
			break;
		case CPUFREQHW_CHNL02:
			if (!hwdvfs_trigger_scu(0, false, false))
				P_INF("SW_CHNL02_DISABLE\n");
			break;
		default:
			P_ERR("%s cluster %u error!\n", __func__, cluster);
			break;
		}

		regval0 = _dvfs_rd_(REG_DVFS_CTRL_HW_DVFS_SEL);
		regval1 = _dvfs_rd_(REG_DVFS_CTRL_SW_CHNL_EN);
		if ((regval0 & BIT_DVFS_CTRL_HW_DVFS_SEL) &&
		    !(regval1 & SW_CHNL00_EN_MSK)
		    && !(regval1 & SW_CHNL01_EN_MSK)
		    && !(regval1 & SW_CHNL02_EN_MSK)) {
			_dvfs_wr_(DVFS_CTRL_MAGIC_NUM_UNLOCK,
				REG_DVFS_CTRL_MAGIC_NUM);
			_dvfs_wr_(VAL2REG(0x0, BIT_DVFS_CTRL_HW_DVFS_SEL),
				REG_DVFS_CTRL_HW_DVFS_SEL);
			cpufreqhw->enabled = false;
			P_INF("DISABLE HWDVFS!\n");
		}
	}

	return true;
}
EXPORT_SYMBOL(sprd_cpufreqhw_enable);

#ifdef SPRD_CPUFREQ_I2C_LOCK
static int cpufreqhw_sharkl3_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	cpufreqhw->i2c_client = client;

	P_INF("cpufreqhw,sharkl3,i2c probe ok\n");
	return 0;
}

static int cpufreqhw_sharkl3_i2c_remove(struct i2c_client *client)
{
	P_INF("cpufreqhw,sharkl3,i2c remove ok\n");
	return 0;
}

#ifdef CONFIG_PM
static int cpufreqhw_sharkl3_i2c_suspend(struct device *dev)
{
	P_INF("cpufreqhw,sharkl3,i2c suspend\n");
	return 0;
}

static int cpufreqhw_sharkl3_i2c_resume(struct device *dev)
{
	P_INF("cpufreqhw,sharkl3,i2c resume\n");
	return 0;
}
static UNIVERSAL_DEV_PM_OPS(cpufreqhw_sharkl3_i2c_pm,
	cpufreqhw_sharkl3_i2c_suspend,
	cpufreqhw_sharkl3_i2c_resume,
	NULL);
#endif

static const struct i2c_device_id cpufreqhw_sharkl3_i2c_id[] = {
	{"cpufreqhw_sharkl3", 0},
	{}
};

static const struct of_device_id cpufreqhw_sharkl3_i2c_of_match[] = {
	{.compatible = "sprd,cpufreqhw-regulator-sharkl3",},
	{}
};

static struct i2c_driver cpufreqhw_sharkl3_i2c_driver = {
	.driver = {
		   .name = "cpufreqhw_sharkl3",
		   .owner = THIS_MODULE,
		   .of_match_table =
			of_match_ptr(cpufreqhw_sharkl3_i2c_of_match),
#ifdef CONFIG_PM
		   .pm	= &cpufreqhw_sharkl3_i2c_pm,
#endif
		   },
	.probe = cpufreqhw_sharkl3_i2c_probe,
	.remove = cpufreqhw_sharkl3_i2c_remove,
	.id_table = cpufreqhw_sharkl3_i2c_id,
};
#endif

bool sprd_cpufreqhw_probed(int cluster)
{
	if (cpufreqhw == NULL || !cpufreqhw->probed)
		return false;

	if (cluster > CPUFREQHW_CHNL_MAX) {
		P_ERR("%s cluster%u  error!\n", __func__, cluster);
		return false;
	}

	return true;
}
EXPORT_SYMBOL(sprd_cpufreqhw_probed);

static int sprd_cpufreqhw_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	struct regmap *aon_apb_base = NULL;
	struct regmap *anlg_phy_g4_ctrl_base = NULL;
	const struct of_device_id *of_id;
	void __iomem *base = NULL;
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "device node not found\n");
		return -ENODEV;
	}

	dev_info(&pdev->dev, "sprd cpufreqhw probe start\n");
	of_id = of_match_node(sprd_cpufreqhw_of_match, pdev->dev.of_node);
	if (!of_id) {
		dev_err(&pdev->dev, "get id of cpufreqhw failed!\n");
		return -ENODEV;
	}

	aon_apb_base =
	    syscon_regmap_lookup_by_phandle(np, "sprd,syscon-enable");
	if (IS_ERR(aon_apb_base)) {
		dev_err(&pdev->dev, "cpufreqhw syscon failed!\n");
		return -ENODEV;
	}
	anlg_phy_g4_ctrl_base =
	    syscon_regmap_lookup_by_phandle(np, "sprd,anlg-phy-g4-ctrl");
	if (IS_ERR(anlg_phy_g4_ctrl_base)) {
		dev_err(&pdev->dev, "get anlg-phy-g4 failed!\n");
		return -ENODEV;
	}

	cpufreqhw =
	    devm_kzalloc(&pdev->dev, sizeof(struct sprd_cpufreqhw), GFP_KERNEL);
	if (!cpufreqhw) {
		dev_err(&pdev->dev, "devm_kzalloc failed!\n");
		return -ENOMEM;
	}

	atomic_set(&cpufreqhw_suspend, 0);
	cpufreqhw->probed = false;
	cpufreqhw->type = (enum sprd_cpufreqhw_type)of_id->data;
	cpufreqhw->aon_apb_base = aon_apb_base;
	cpufreqhw->anlg_phy_g4_ctrl = anlg_phy_g4_ctrl_base;

	init_completion(&cpufreqhw->i2c_done);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (!base) {
		dev_err(&pdev->dev, "remap cpufreqhw address failed!\n");
		ret = -ENOMEM;
		goto free_mem;
	}
	cpufreqhw->base_dvfs = base;

#ifdef SPRD_CPUFREQ_IRQ_ENABLE
	cpufreqhw->irq = platform_get_irq(pdev, 0);
	if (cpufreqhw->irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource info\n");
		ret = -EIO;
		goto free_mem;
	}

	ret = devm_request_irq(&pdev->dev, cpufreqhw->irq, sprd_cpufreqhw_isr,
			       IRQF_NO_SUSPEND, "sprd_cpufreqhw", NULL);
	if (ret)
		dev_err(&pdev->dev, "sprd cpufreqhw isr register failed\n");
#endif
	if (sprd_cpufreqhw_init_param(np)) {
		dev_err(&pdev->dev, "failed to sprd_cpufreqhw_init\n");
		ret = -ENODEV;
		goto free_mem;
	}

	cpufreqhw_kobj =
	    kobject_create_and_add("cpufreqhw", cpufreq_global_kobject);
	if (!
	    (cpufreqhw_kobj
	     && !sysfs_create_group(cpufreqhw_kobj, &attr_group_cpufreqhw))) {
		ret = -EPERM;
		dev_err(&pdev->dev,
			"failed to add cpufreqhw debug node in cpufreq\n");
		goto free_mem;
	}

	atomic_set(&cpufreqhw->dvfs_state[CPUFREQHW_CHNL00],
		   CPUFREQHW_STATE_UNKNOWN);
	atomic_set(&cpufreqhw->dvfs_state[CPUFREQHW_CHNL01],
		   CPUFREQHW_STATE_UNKNOWN);
	atomic_set(&cpufreqhw->dvfs_state[CPUFREQHW_CHNL02],
		   CPUFREQHW_STATE_UNKNOWN);

#ifdef SPRD_CPUFREQ_I2C_LOCK
	cpufreqhw->i2c_client = NULL;
	ret = i2c_add_driver(&cpufreqhw_sharkl3_i2c_driver);
	if (ret)
		dev_err(&pdev->dev, "failed to get i2c client\n");
#endif
	cpufreqhw->probed = true;
	P_INF("sprd cpufreqhw probed %s\n",
	      cpufreqhw->probed ? "true" : "false");
	return ret;

 free_mem:
	devm_kfree(&pdev->dev, cpufreqhw);
	cpufreqhw = NULL;
	P_INF("sprd cpufreqhw probed error!\n");

	return ret;
}

static int sprd_cpufreqhw_remove(struct platform_device *pdev)
{
	if (cpufreqhw_kobj)
		sysfs_remove_group(cpufreqhw_kobj, &attr_group_cpufreqhw);
#ifdef SPRD_CPUFREQ_I2C_LOCK
	i2c_del_driver(&cpufreqhw_sharkl3_i2c_driver);
#endif
	devm_kfree(&pdev->dev, cpufreqhw);
	return 0;
}

static int sprd_cpufreqhw_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	atomic_set(&cpufreqhw_suspend, 1);
	P_DBG("END\n");
	return 0;
}

static int sprd_cpufreqhw_resume(struct platform_device *pdev)
{
	atomic_set(&cpufreqhw_suspend, 0);
	P_DBG("END\n");
	return 0;
}

static struct platform_driver sprd_cpufreqhw_driver = {
	.probe = sprd_cpufreqhw_probe,
	.remove = sprd_cpufreqhw_remove,
	.suspend = sprd_cpufreqhw_suspend,
	.resume = sprd_cpufreqhw_resume,
	.driver = {
		   .name = "sprd_cpufreqhw",
		   .of_match_table = of_match_ptr(sprd_cpufreqhw_of_match),
		   },
};

static int __init sprd_cpufreqhw_init(void)
{
	cpufreqhw = NULL;
	cpufreqhw_kobj = NULL;
	return platform_driver_register(&sprd_cpufreqhw_driver);
}

static void __exit sprd_cpufreqhw_exit(void)
{
	platform_driver_unregister(&sprd_cpufreqhw_driver);
}

subsys_initcall(sprd_cpufreqhw_init);
module_exit(sprd_cpufreqhw_exit);

MODULE_AUTHOR("Ling Xu <ling_ling.xu@spreadtrum.com>");
MODULE_DESCRIPTION("Spreadtrum cpufreq hardware Driver");
MODULE_LICENSE("GPL");
