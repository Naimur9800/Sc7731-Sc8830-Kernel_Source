/*
 * Support for power management features of the SPRD iSoC
 *
 * Based on arch/x86/kernel/acpi/sleep.c.
 *
 * Copyright (C) 2016 Intel Corporation
 * Author Bin Gao <bin.gao@intel.com>
 *
 * This file is released under the GPLv2.
 */

#include <linux/mfd/syscon/sprd/isharkl2/isharkl2_glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>

/*
* Light sleep mode :
* mode = 1 : Smart Mode
* mode = 0 : Normal Mode
*/
static unsigned int mode = 1;
static unsigned int ap_light_enable = 1;
static struct kobject *light_mode_kobj;
static struct regmap *g_sys_pwu_apb;

void sc_light_init(void);
static void change_light_mode(int targetmode);

#define LIGHT_SLEEP_MODE(_name) \
static struct kobj_attribute _name##_attr = {   \
	.attr   = {                             \
		.name = __stringify(_name),     \
		.mode = 0644,                   \
	},                                      \
	.show   = _name##_show,                 \
	.store  = _name##_store,                \
}

static spinlock_t aplslp_lock;

/**
* @req: 0--disable ap light; 1--enable ap light;
* @name : request name;
*/
void ap_light_enable_disable(unsigned int req, const char *name)
{
	struct device_node *soc_pm_np;
	unsigned long irq_flags;
	unsigned int ddr_stat_us, ddr_stat_val;

	if (IS_ERR_OR_NULL(g_sys_pwu_apb)) {
		soc_pm_np = of_find_node_by_name(NULL, "soc-pm");
		if (IS_ERR(soc_pm_np)) {
			pr_err("%s, failed to find light_doze_np\n", __func__);
			return;
		}

		g_sys_pwu_apb = syscon_regmap_lookup_by_phandle(soc_pm_np,
			"sprd,sys-aon-pwu-apb");
		if (IS_ERR(g_sys_pwu_apb)) {
			pr_err("%s, failed to find g_sys_pwu_apb\n", __func__);
			return;
		}
	}

	spin_lock_irqsave(&aplslp_lock, irq_flags);
	if (req == 0) {
		regmap_update_bits(g_sys_pwu_apb,
			REG_PMU_APB_LIGHT_SLEEP_ENABLE0,
			BIT_PMU_APB_AP_SYS_LSLP0_ENA,
			(unsigned int)(~BIT_PMU_APB_AP_SYS_LSLP0_ENA));
		ap_light_enable = 0;
	} else {
		regmap_update_bits(g_sys_pwu_apb,
			REG_PMU_APB_LIGHT_SLEEP_ENABLE0,
			BIT_PMU_APB_AP_SYS_LSLP0_ENA,
			BIT_PMU_APB_AP_SYS_LSLP0_ENA);
		ap_light_enable = 1;
	}
	spin_unlock_irqrestore(&aplslp_lock, irq_flags);

	/*
	* In case of disabling AP light, must wait
	* until ddr control register is ready
	*/
	if (req == 0) {
		ddr_stat_us = 30;
		ddr_stat_val = BIT_PMU_APB_PUB0_SYS_SLP_STATUS(0xf);
		while (ddr_stat_us && ddr_stat_val) {
			udelay(1);
			regmap_read(g_sys_pwu_apb,
				REG_PMU_APB_SYS_SLP_STATUS0,
				&ddr_stat_val);
			ddr_stat_val &= BIT_PMU_APB_PUB0_SYS_SLP_STATUS(0xf);
			ddr_stat_us--;
		}
		if (ddr_stat_us == 0)
			pr_err("%s,ddr_stat_val %u is not ready!\n",
				__func__, ddr_stat_val);
		else
			pr_debug("%s,ddr_stat_us %u ddr_stat_val %u\n",
				__func__, ddr_stat_us, ddr_stat_val);
	}

	return;
}
EXPORT_SYMBOL_GPL(ap_light_enable_disable);

static void light_doze_enable(void)
{
	struct device_node *soc_pm_np;
	struct regmap *sys_ap_ahb;
	struct regmap *sys_pwu_apb;
	struct regmap *sys_aon_apb;
	struct regmap *sys_aon_com_pmu_apb;

	pr_info("###### %s start ###\n", __func__);

	soc_pm_np = of_find_node_by_name(NULL, "soc-pm");
	if (IS_ERR(soc_pm_np)) {
		pr_err("%s, failed to find light_doze_np\n", __func__);
		return;
	}
	/* 0xE2210000 */
	sys_ap_ahb = syscon_regmap_lookup_by_phandle(soc_pm_np,
		"sprd,sys-ap-ahb");
	if (IS_ERR(sys_ap_ahb)) {
		pr_err("%s, failed to find sys_ap_ahb\n", __func__);
		return;
	}
	/* E42c0000 */
	sys_aon_com_pmu_apb = syscon_regmap_lookup_by_phandle(soc_pm_np,
		"sprd,sys-aon-com-pmu-apb");
	if (IS_ERR(sys_aon_com_pmu_apb)) {
		pr_err("%s, failed to find sys_aon_com_pmu_apb\n", __func__);
		return;
	}
	/*E42b0000*/
	sys_pwu_apb = syscon_regmap_lookup_by_phandle(soc_pm_np,
		"sprd,sys-aon-pwu-apb");
	if (IS_ERR(sys_pwu_apb)) {
		pr_err("%s, failed to find sys_pwu_apb\n", __func__);
		return;
	}
	/*0xE42e0000*/
	sys_aon_apb = syscon_regmap_lookup_by_phandle(soc_pm_np,
		"sprd,sys-aon-apb");
	if (IS_ERR(sys_aon_apb)) {
		pr_err("%s, failed to find sys_aon_apb\n", __func__);
		return;
	}

	/*enable ap sys lightsleep0 en*/
	regmap_update_bits(sys_pwu_apb,
		REG_PMU_APB_LIGHT_SLEEP_ENABLE0,
		BIT_PMU_APB_AP_SYS_LSLP0_ENA,
		BIT_PMU_APB_AP_SYS_LSLP0_ENA);

	/*enable ap sys lightsleep0 wakeup en*/
	regmap_update_bits(sys_pwu_apb,
		REG_PMU_APB_AP_SYS_SLEEP_CTRL,
		BIT_PMU_APB_AP_SYS_LSLP0_WAKEUP_EN,
		BIT_PMU_APB_AP_SYS_LSLP0_WAKEUP_EN);

	/*enable ap sys doze en*/
	regmap_update_bits(sys_pwu_apb,
		REG_PMU_APB_DOZE_SLEEP_ENABLE0,
		BIT_PMU_APB_AP_SYS_DOZE_ENA,
		BIT_PMU_APB_AP_SYS_DOZE_ENA);

	/* PUB0_FRC_LSLP dap_ep and bia_lp_eb*/
	regmap_update_bits(sys_ap_ahb,
		REG_AP_AHB_PUB0_FRC_LSLP,
		BIT_AP_AHB_PUB0_FRC_LSLP(0x3),
		BIT_AP_AHB_PUB0_FRC_LSLP(0x3));

	 /* FRC_DOZE DAP_eb and bia_lp_eb*/
	regmap_update_bits(sys_ap_ahb,
		REG_AP_AHB_FRC_DOZE,
		BIT_AP_AHB_FRC_DOZE(0x3),
		BIT_AP_AHB_FRC_DOZE(0x3));

	/* bia_s3_pub0_frc_lslp*/
	regmap_update_bits(sys_aon_com_pmu_apb,
		REG_COM_PMU_APB_AON_PUB_FRC_SLP_CTRL,
		BIT_COM_PMU_APB_BIA_S3_PUB0_FRC_LSLP,
		BIT_COM_PMU_APB_BIA_S3_PUB0_FRC_LSLP);

	/*Enable aon_apb_idle_en*/
	regmap_update_bits(sys_aon_apb,
		REG_AON_APB_EB_AON_ADD1,
		BIT_AON_APB_AON_APB_IDLE_EN,
		BIT_AON_APB_AON_APB_IDLE_EN);

	change_light_mode(mode);

	pr_info("###### %s end ###\n", __func__);
}

static void change_light_mode(int targetmode)
{
	struct device_node *soc_pm_np;
	struct regmap *sys_ap_ahb;
	struct regmap *sys_pwu_apb;
	struct regmap *sys_aon_com_pmu_apb;

	soc_pm_np = of_find_node_by_name(NULL, "soc-pm");
	if (IS_ERR(soc_pm_np)) {
		pr_err("%s, failed to find light_doze_np\n", __func__);
		return;
	}

	/*0xE2210000*/
	sys_ap_ahb = syscon_regmap_lookup_by_phandle(soc_pm_np,
		"sprd,sys-ap-ahb");
	if (IS_ERR(sys_ap_ahb)) {
		pr_err("%s, failed to find sys_ap_ahb\n", __func__);
		return;
	}

	/*E42c0000*/
	sys_aon_com_pmu_apb = syscon_regmap_lookup_by_phandle(soc_pm_np,
		"sprd,sys-aon-com-pmu-apb");
	if (IS_ERR(sys_aon_com_pmu_apb)) {
		pr_err("%s, failed to find sys_aon_com_pmu_apb\n", __func__);
		return;
	}

	/*E42b0000*/
	sys_pwu_apb = syscon_regmap_lookup_by_phandle(soc_pm_np,
		"sprd,sys-aon-pwu-apb");
	if (IS_ERR(sys_pwu_apb)) {
		pr_err("%s, failed to find sys_pwu_apb\n", __func__);
		return;
	}

	if (targetmode == 0) {
		/*enable ap sys lightsleep0 wakeup en*/
		regmap_update_bits(sys_pwu_apb,
			REG_PMU_APB_AP_SYS_SLEEP_CTRL,
			BIT_PMU_APB_AP_SYS_LSLP0_WAKEUP_EN,
			BIT_PMU_APB_AP_SYS_LSLP0_WAKEUP_EN);

		/*clear bia/gpu/vsp/ap smart light enable*/
		regmap_update_bits(sys_aon_com_pmu_apb,
			REG_COM_PMU_APB_SMART_LSLP_ENA,
			BIT_COM_PMU_APB_GPU_TOP_SMART_LSLP_ENA
			| BIT_COM_PMU_APB_VSP_SYS_SMART_LSLP_ENA
			| BIT_COM_PMU_APB_AP_SYS_SMART_LSLP_ENA
			| BIT_COM_PMU_APB_BIA_MEMAXI_SMART_LSLP_ENA,
			(unsigned int)
			(~(BIT_COM_PMU_APB_GPU_TOP_SMART_LSLP_ENA
			| BIT_COM_PMU_APB_VSP_SYS_SMART_LSLP_ENA
			| BIT_COM_PMU_APB_AP_SYS_SMART_LSLP_ENA
			| BIT_COM_PMU_APB_BIA_MEMAXI_SMART_LSLP_ENA)));

		/*clear bia/gpu/vsp/ap fencing lock wait num*/
		regmap_update_bits(sys_aon_com_pmu_apb,
			REG_COM_PMU_APB_FENCING_LOCK_WAIT_NUM0,
			BIT_COM_PMU_APB_AP_SYS_FENCING_LOCK_WAIT_NUM(0xff)
			| BIT_COM_PMU_APB_VSP_SYS_FENCING_LOCK_WAIT_NUM(0xff)
			| BIT_COM_PMU_APB_GPU_TOP_FENCING_LOCK_WAIT_NUM(0xff),
			(unsigned int)
			(~(BIT_COM_PMU_APB_GPU_TOP_FENCING_LOCK_WAIT_NUM(0xff)
			| BIT_COM_PMU_APB_VSP_SYS_FENCING_LOCK_WAIT_NUM(0xff)
			| BIT_COM_PMU_APB_GPU_TOP_FENCING_LOCK_WAIT_NUM(0xff))));
		regmap_update_bits(sys_aon_com_pmu_apb,
			REG_COM_PMU_APB_FENCING_LOCK_WAIT_NUM2,
			BIT_COM_PMU_APB_BIA_MEMAXI_FENCING_LOCK_WAIT_NUM(0xff),
			(unsigned int)
			(~BIT_COM_PMU_APB_BIA_MEMAXI_FENCING_LOCK_WAIT_NUM(0xff)));

		/*clear all masters in ap top*/
		regmap_update_bits(sys_ap_ahb,
			REG_AP_AHB_PUB0_FRC_LSLP,
			(0x1ff << 2),
			(unsigned int)(~(0x1ff << 2)));

		/*clear vsp/gpu/bia_pub0_frce_lsp to consider vsp/gpu/bia */
		regmap_update_bits(sys_aon_com_pmu_apb,
			REG_COM_PMU_APB_AON_PUB_FRC_SLP_CTRL,
			BIT(1) | BIT(2) | BIT(3),
			(unsigned int)(~(BIT(1) | BIT(2) | BIT(3))));

		mode = 0;
		pr_info("Change Light sleep mode to Normal Mode\n");

	} else if (targetmode == 1) {

		/*clear ap sys lightsleep0 wakeup en*/
		regmap_update_bits(sys_pwu_apb,
			REG_PMU_APB_AP_SYS_SLEEP_CTRL,
			BIT_PMU_APB_AP_SYS_LSLP0_WAKEUP_EN,
			(unsigned int)(~BIT_PMU_APB_AP_SYS_LSLP0_WAKEUP_EN));

		/*set bia/gpu/vsp/ap smart light enable*/
		regmap_update_bits(sys_aon_com_pmu_apb,
			REG_COM_PMU_APB_SMART_LSLP_ENA,
			BIT_COM_PMU_APB_GPU_TOP_SMART_LSLP_ENA
			| BIT_COM_PMU_APB_VSP_SYS_SMART_LSLP_ENA
			| BIT_COM_PMU_APB_AP_SYS_SMART_LSLP_ENA
			| BIT_COM_PMU_APB_BIA_MEMAXI_SMART_LSLP_ENA,
			(BIT_COM_PMU_APB_GPU_TOP_SMART_LSLP_ENA
			| BIT_COM_PMU_APB_VSP_SYS_SMART_LSLP_ENA
			| BIT_COM_PMU_APB_AP_SYS_SMART_LSLP_ENA
			| BIT_COM_PMU_APB_BIA_MEMAXI_SMART_LSLP_ENA));

		/*set bia/gpu/vsp/ap fencing lock wait num to 0x64*/
		regmap_update_bits(sys_aon_com_pmu_apb,
			REG_COM_PMU_APB_FENCING_LOCK_WAIT_NUM0,
			BIT_COM_PMU_APB_AP_SYS_FENCING_LOCK_WAIT_NUM(0xff)
			| BIT_COM_PMU_APB_VSP_SYS_FENCING_LOCK_WAIT_NUM(0xff)
			| BIT_COM_PMU_APB_GPU_TOP_FENCING_LOCK_WAIT_NUM(0xff),
			(BIT_COM_PMU_APB_GPU_TOP_FENCING_LOCK_WAIT_NUM(0x1)
			| BIT_COM_PMU_APB_VSP_SYS_FENCING_LOCK_WAIT_NUM(0x1)
			| BIT_COM_PMU_APB_GPU_TOP_FENCING_LOCK_WAIT_NUM(0x1)));
		regmap_update_bits(sys_aon_com_pmu_apb,
			REG_COM_PMU_APB_FENCING_LOCK_WAIT_NUM2,
			BIT_COM_PMU_APB_BIA_MEMAXI_FENCING_LOCK_WAIT_NUM(0xff),
			BIT_COM_PMU_APB_BIA_MEMAXI_FENCING_LOCK_WAIT_NUM(0x1));

		/*Bypass all masters in ap top*/
		regmap_update_bits(sys_ap_ahb,
			REG_AP_AHB_PUB0_FRC_LSLP,
			(0x1ff << 2),
			(0x1ff << 2));

		/*set vsp/gpu/bia_pub0/1_frce_lsp to bypass vsp/gpu/bia */
		regmap_update_bits(sys_aon_com_pmu_apb,
			REG_COM_PMU_APB_AON_PUB_FRC_SLP_CTRL,
			BIT(1) | BIT(2) | BIT(3),
			BIT(1) | BIT(2) | BIT(3));

		/*set ddr0_sleep_wait_cnt to 50*/
		regmap_update_bits(sys_pwu_apb,
		REG_PMU_APB_DDR0_SLP_CFG,
		BIT_PMU_APB_DDR0_SLEEP_WAIT_CNT(0xffff),
		BIT_PMU_APB_DDR0_SLEEP_WAIT_CNT(0x32));

		mode = 1;
		pr_info("Change Light sleep mode to  Smart Mode\n");
	}
}
static ssize_t light_mode_show(struct kobject *kobj,
			struct kobj_attribute *attr,
			char *buf)
{
	return mode ? sprintf(buf, "smart\n") : sprintf(buf, "normal\n");
}

static ssize_t light_mode_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t n)
{
	if (n != 2)
		return n;
	if ('0' == (*buf))
		if (mode != 0)
			change_light_mode(0);
	if ('1' == (*buf))
		if (mode != 1)
			change_light_mode(1);
	return n;
}

LIGHT_SLEEP_MODE(light_mode);
static ssize_t light_enable_show(struct kobject *kobj,
			struct kobj_attribute *attr,
			char *buf)
{
	return ap_light_enable ? sprintf(buf, "1\n") : sprintf(buf, "0\n");
}

static ssize_t light_enable_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t n)
{
	if (n != 2)
		return n;
	if ('0' == (*buf))
		if (ap_light_enable != 0)
			ap_light_enable_disable(0, "userspace");
	if ('1' == (*buf))
		if (ap_light_enable != 1)
			ap_light_enable_disable(1, "userspace");
	return n;
}
LIGHT_SLEEP_MODE(light_enable);
static struct attribute *g[] = {
	&light_mode_attr.attr,
	&light_enable_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

void sc_light_init(void)
{
	g_sys_pwu_apb = NULL;
	spin_lock_init(&aplslp_lock);
	light_doze_enable();

	light_mode_kobj = kobject_create_and_add("ap_light_mode", NULL);
	if (light_mode_kobj)
		if (sysfs_create_group(light_mode_kobj, &attr_group))
			pr_err("%s, failed to add ap_light_mode\n", __func__);
}

