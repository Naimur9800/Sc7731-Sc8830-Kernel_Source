/*
 * Copyright (C) 2015--2016 Spreadtrum Communications Inc.
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

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <uapi/video/sprd_vsp_pw_domain.h>
#include "vsp_common.h"

#if IS_ENABLED(CONFIG_SPRD_VSP_PW_DOMAIN_R6P0)
#include <linux/platform_device.h>
#include "cam_pw_domain.h"
#endif

struct vsp_pw_domain_info_t *vsp_pw_domain_info;

#if IS_ENABLED(CONFIG_SPRD_VSP_PW_DOMAIN)

static int is_vsp_domain_power_on(void)
{
	int power_count = 0;
	int ins_count = 0;

	for (ins_count = 0; ins_count < VSP_PW_DOMAIN_COUNT_MAX; ins_count++) {
		power_count +=
		    vsp_pw_domain_info->pw_vsp_info[ins_count].pw_count;
	}

	return (power_count > 0) ? 1 : 0;
}

#define __SPRD_VSP_TIMEOUT            (30 * 1000)
int vsp_pw_on(u8 client)
{
	int ret = 0;
	u32 power_state1, power_state2, power_state3;
	unsigned long timeout = jiffies + msecs_to_jiffies(__SPRD_VSP_TIMEOUT);
	u32 read_count = 0;

	pr_info("vsp_pw_domain:vsp_pw_on Enter client %d\n", client);
	if (client >= VSP_PW_DOMAIN_COUNT_MAX) {
		pr_err("vsp_pw_domain:vsp_pw_on with error client\n");
		return -1;
	}

	mutex_lock(&vsp_pw_domain_info->client_lock);

	if (is_vsp_domain_power_on() == 0) {
#if  !IS_ENABLED(CONFIG_X86)
		ret = regmap_update_bits(gpr_pmu_apb,
				REG_PMU_APB_PD_VSP_SYS_CFG,
				BIT_PMU_APB_PD_VSP_SYS_AUTO_SHUTDOWN_EN,
				(unsigned int)
				(~BIT_PMU_APB_PD_VSP_SYS_AUTO_SHUTDOWN_EN));
		if (ret) {
			pr_err("regmap_update_bits failed %s, %d\n",
				__func__, __LINE__);
			goto pw_on_exit;
		}

		ret = regmap_update_bits(gpr_pmu_apb,
			REG_PMU_APB_PD_VSP_SYS_CFG,
			BIT_PMU_APB_PD_VSP_SYS_FORCE_SHUTDOWN,
			(unsigned int)(~BIT_PMU_APB_PD_VSP_SYS_FORCE_SHUTDOWN));
		if (ret) {
			pr_err("regmap_update_bits failed %s, %d\n",
				__func__, __LINE__);
			goto pw_on_exit;
		}
#else
		ret = regmap_update_bits(gpr_pmu_apb,
				REG_PMU_APB_PD_VSP_SYS_PWR_CFG,
				BIT_PMU_APB_PD_VSP_SYS_AUTO_SHUTDOWN_EN,
				(unsigned int)
				(~BIT_PMU_APB_PD_VSP_SYS_AUTO_SHUTDOWN_EN));
		if (ret) {
			pr_err("regmap_update_bits failed %s, %d\n",
				__func__, __LINE__);
			goto pw_on_exit;
		}

		ret = regmap_update_bits(gpr_pmu_apb,
			REG_PMU_APB_PD_VSP_SYS_PWR_CFG,
			BIT_PMU_APB_PD_VSP_SYS_FORCE_SHUTDOWN,
			(unsigned int)(~BIT_PMU_APB_PD_VSP_SYS_FORCE_SHUTDOWN));
		if (ret) {
			pr_err("regmap_update_bits failed %s, %d\n",
				__func__, __LINE__);
			goto pw_on_exit;
		}


#endif
		do {
			cpu_relax();
			udelay(300);
			read_count++;
#if  !IS_ENABLED(CONFIG_X86)
			regmap_read(gpr_pmu_apb,
						REG_PMU_APB_PWR_STATUS4_DBG,
						&power_state1);
			power_state1 &= BIT_PMU_APB_PD_VSP_SYS_STATE(0x1F);
			regmap_read(gpr_pmu_apb,
						REG_PMU_APB_PWR_STATUS4_DBG,
						&power_state2);
			power_state2 &= BIT_PMU_APB_PD_VSP_SYS_STATE(0x1F);
			regmap_read(gpr_pmu_apb,
						REG_PMU_APB_PWR_STATUS4_DBG,
						&power_state3);
			power_state3 &= BIT_PMU_APB_PD_VSP_SYS_STATE(0x1F);
#else
			regmap_read(gpr_pmu_apb,
						REG_PMU_APB_PWR_STATE_DBG2,
						&power_state1);
			power_state1 &= BIT_PMU_APB_PD_VSP_SYS_STATE(0x1F);
			regmap_read(gpr_pmu_apb,
						REG_PMU_APB_PWR_STATE_DBG2,
						&power_state2);
			power_state2 &= BIT_PMU_APB_PD_VSP_SYS_STATE(0x1F);
			regmap_read(gpr_pmu_apb,
						REG_PMU_APB_PWR_STATE_DBG2,
						&power_state3);
			power_state3 &= BIT_PMU_APB_PD_VSP_SYS_STATE(0x1F);
#endif
			WARN_ON(time_after(jiffies, timeout));
		} while ((power_state1 && read_count < 100)
			 || (power_state1 != power_state2)
			 || (power_state2 != power_state3));

		if (power_state1) {
			pr_err("vsp_pw_domain:vsp_pw_on set failed 0x%x\n",
			       power_state1);
			mutex_unlock(&vsp_pw_domain_info->client_lock);
			return -1;
		}

		pr_info("vsp_pw_domain:vsp_pw_on set OK\n");
#if IS_ENABLED(CONFIG_X86)
		ret = regmap_update_bits(gpr_aon_apb,
			REG_AON_APB_APB_EB1,
			BIT_AON_APB_AON_VSP_EB,
			BIT_AON_APB_AON_VSP_EB);
		udelay(1000);
		if (ret) {
			pr_info("regmap_update_bits failed %s, %d\n",
				__func__, __LINE__);
			goto pw_on_exit;
		}

		ret = regmap_hwlock_update_bits(gpr_mm_ahb,
			0x08,
			0xffff,
			0x3333);
		if (ret) {
			pr_info("regmap_update_bits failed %s, %d\n",
				__func__, __LINE__);
			goto pw_on_exit;
		}

		ret = regmap_hwlock_update_bits(gpr_mm_ahb,
			0x10,
			0xff,
			0x17);
		if (ret) {
			pr_info("regmap_update_bits failed %s, %d\n",
				__func__, __LINE__);
			goto pw_on_exit;
		}

		ret = regmap_hwlock_update_bits(gpr_mm_ahb,
			0x44,
			0x80007F7F,
			0x408);
		if (ret) {
			pr_info("regmap_update_bits failed %s, %d\n",
				__func__, __LINE__);
			goto pw_on_exit;
		}

		ret = regmap_hwlock_update_bits(gpr_mm_ahb,
			0x48,
			0x80007F7F,
			0x408);
		if (ret) {
			pr_info("regmap_update_bits failed %s, %d\n",
			__func__, __LINE__);
			goto pw_on_exit;
		}
#endif
	} else {
		pr_info("vsp_pw_domain:vsp_domain is already power on\n");
	}

	vsp_pw_domain_info->pw_vsp_info[client].pw_state = VSP_PW_DOMAIN_ON;
	vsp_pw_domain_info->pw_vsp_info[client].pw_count++;

pw_on_exit:
	mutex_unlock(&vsp_pw_domain_info->client_lock);
	return ret;
}
EXPORT_SYMBOL(vsp_pw_on);

int vsp_pw_off(u8 client)
{
	int ret = 0;
	unsigned int vsp_fencing_status = 0;

	pr_info("vsp_pw_domain: vsp_pw_off Enter client %d\n", client);
	if (client >= VSP_PW_DOMAIN_COUNT_MAX) {
		pr_err("vsp_pw_domain:vsp_pw_off with error client\n");
		return -1;
	}
	mutex_lock(&vsp_pw_domain_info->client_lock);

	if (vsp_pw_domain_info->pw_vsp_info[client].pw_count >= 1) {

		vsp_pw_domain_info->pw_vsp_info[client].pw_count--;
		if (vsp_pw_domain_info->pw_vsp_info[client].pw_count == 0) {
			vsp_pw_domain_info->pw_vsp_info[client].pw_state =
			    VSP_PW_DOMAIN_OFF;
		}

		if (is_vsp_domain_power_on() == 0) {
#if  !IS_ENABLED(CONFIG_X86)
			vsp_fencing_status++;

			ret = regmap_update_bits(gpr_pmu_apb,
				REG_PMU_APB_PD_VSP_SYS_CFG,
				BIT_PMU_APB_PD_VSP_SYS_AUTO_SHUTDOWN_EN,
				(unsigned int)
				(~BIT_PMU_APB_PD_VSP_SYS_AUTO_SHUTDOWN_EN));

			if (ret) {
				pr_err("regmap_update_bits failed %s, %d\n",
					__func__, __LINE__);
				goto pw_off_exit;
			}

			ret = regmap_update_bits(gpr_pmu_apb,
				REG_PMU_APB_PD_VSP_SYS_CFG,
				BIT_PMU_APB_PD_VSP_SYS_FORCE_SHUTDOWN,
				BIT_PMU_APB_PD_VSP_SYS_FORCE_SHUTDOWN);
			if (ret) {
				pr_err("regmap_update_bits failed %s, %d\n",
					__func__, __LINE__);
				goto pw_off_exit;
			}
#else

			ret = regmap_update_bits(gpr_aon_apb,
				REG_AON_APB_APB_EB1,
				BIT_AON_APB_AON_VSP_EB,
			(unsigned int)(~BIT_AON_APB_AON_VSP_EB));
			if (ret) {
				pr_err("regmap_update_bits failed %s, %d\n",
					__func__, __LINE__);
				goto pw_off_exit;
			}

			udelay(500);
			regmap_read(gpr_com_pmu_apb,
				REG_COM_PMU_APB_FENCING0_CTRL_STATE,
				&vsp_fencing_status);
			vsp_fencing_status &=
			BIT_COM_PMU_APB_VSP_SYS_FENCING0_CTRL_STATE(0xf);

			if (vsp_fencing_status != 0x30 &&
				vsp_fencing_status != 0x40) {
				pr_err("vsp_fencing abnormal %s, %d\n",
					__func__, __LINE__);
				WARN_ON(1);
			}

			ret = regmap_update_bits(gpr_pmu_apb,
				REG_PMU_APB_PD_VSP_SYS_PWR_CFG,
				BIT_PMU_APB_PD_VSP_SYS_AUTO_SHUTDOWN_EN,
				(unsigned int)
				(~BIT_PMU_APB_PD_VSP_SYS_AUTO_SHUTDOWN_EN));
			if (ret) {
				pr_err("regmap_update_bits failed %s, %d\n",
					__func__, __LINE__);
				goto pw_off_exit;
			}

			ret = regmap_update_bits(gpr_pmu_apb,
				REG_PMU_APB_PD_VSP_SYS_PWR_CFG,
				BIT_PMU_APB_PD_VSP_SYS_FORCE_SHUTDOWN,
				BIT_PMU_APB_PD_VSP_SYS_FORCE_SHUTDOWN);
			if (ret) {
				pr_err("regmap_update_bits failed %s, %d\n",
					__func__, __LINE__);
				goto pw_off_exit;
			}
#endif
			pr_info("vsp_pw_domain:vsp_pw_off set OK\n");
		}
	} else {
		vsp_pw_domain_info->pw_vsp_info[client].pw_count = 0;
		vsp_pw_domain_info->pw_vsp_info[client].pw_state =
		    VSP_PW_DOMAIN_OFF;
		pr_info("vsp_pw_domain:vsp_domain is already power off\n");
	}

pw_off_exit:
	mutex_unlock(&vsp_pw_domain_info->client_lock);
	return ret;
}
EXPORT_SYMBOL(vsp_pw_off);

#if  !IS_ENABLED(CONFIG_X86)
/* whale2 */
int vsp_core_pw_on(void)
{
	return 0;
}
EXPORT_SYMBOL(vsp_core_pw_on);

int vsp_core_pw_off(void)
{
	return 0;
}
EXPORT_SYMBOL(vsp_core_pw_off);
#else
/* iwhale2 */
int vsp_core_pw_on(void)
{
	u32 power_state1, power_state2, power_state3;
	unsigned long timeout = jiffies + msecs_to_jiffies(__SPRD_VSP_TIMEOUT);
	u32 read_count = 0;
	int ret = 0;

	pr_info("vsp_core_pw_on Enter\n");
	mutex_lock(&vsp_pw_domain_info->client_lock);
	ret =  regmap_update_bits(gpr_pmu_apb,
		REG_PMU_APB_PD_VSP_CORE_PWR_CFG,
		BIT_PMU_APB_PD_VSP_SYS_AUTO_SHUTDOWN_EN,
		(unsigned int)(~BIT_PMU_APB_PD_VSP_SYS_AUTO_SHUTDOWN_EN));

	if (ret) {
		pr_err("regmap_update_bits failed %s, %d\n",
			__func__, __LINE__);
		goto core_pw_on_exit;
	}

	ret = regmap_update_bits(gpr_pmu_apb, REG_PMU_APB_PD_VSP_CORE_PWR_CFG,
		BIT_PMU_APB_PD_VSP_SYS_FORCE_SHUTDOWN,
		(unsigned int)(~BIT_PMU_APB_PD_VSP_SYS_FORCE_SHUTDOWN));

	if (ret) {
		pr_err("regmap_update_bits failed %s, %d\n",
			__func__, __LINE__);
		goto core_pw_on_exit;
	}

	do {
		cpu_relax();
		udelay(300);
		read_count++;

		regmap_read(gpr_pmu_apb,
			REG_PMU_APB_PWR_STATE_DBG4,
			&power_state1);
		power_state1 &= BIT_PMU_APB_PD_VSP_CORE_STATE(0x1F);
		regmap_read(gpr_pmu_apb,
			REG_PMU_APB_PWR_STATE_DBG4,
			&power_state2);
		power_state2 &= BIT_PMU_APB_PD_VSP_CORE_STATE(0x1F);
		regmap_read(gpr_pmu_apb,
			REG_PMU_APB_PWR_STATE_DBG4,
			&power_state3);
		power_state3 &= BIT_PMU_APB_PD_VSP_CORE_STATE(0x1F);

		WARN_ON(time_after(jiffies, timeout));
	} while ((power_state1 && read_count < 100)
		|| (power_state1 != power_state2)
		|| (power_state2 != power_state3));

	if (power_state1) {
		pr_err("vsp_core_pw_on set failed 0x%x\n", power_state1);
		mutex_unlock(&vsp_pw_domain_info->client_lock);
		return -1;
	}
core_pw_on_exit:
	mutex_unlock(&vsp_pw_domain_info->client_lock);
	pr_info("vsp_core_pw_on set OK\n");

	return 0;
}
EXPORT_SYMBOL(vsp_core_pw_on);

int vsp_core_pw_off(void)
{
	int ret = 0;

	pr_info("vsp_core_pw_off:  Enter\n");
	mutex_lock(&vsp_pw_domain_info->client_lock);

	if (vsp_pw_domain_info->pw_vsp_info[VSP_PW_DOMAIN_VSP].pw_count  +
		vsp_pw_domain_info->pw_vsp_info[VSP_PW_DOMAIN_VPP].pw_count > 1)
		goto vsp_core_pw_off_exit;
	ret = regmap_update_bits(gpr_pmu_apb,
		REG_PMU_APB_PD_VSP_CORE_PWR_CFG,
		BIT_PMU_APB_PD_VSP_SYS_AUTO_SHUTDOWN_EN,
		(unsigned int)
		(~BIT_PMU_APB_PD_VSP_SYS_AUTO_SHUTDOWN_EN));

	if (ret) {
		pr_err("regmap_update_bits failed %s, %d\n",
			__func__, __LINE__);
		goto vsp_core_pw_off_exit;
	}

	ret = regmap_update_bits(gpr_pmu_apb,
		REG_PMU_APB_PD_VSP_CORE_PWR_CFG,
		BIT_PMU_APB_PD_VSP_SYS_FORCE_SHUTDOWN,
		BIT_PMU_APB_PD_VSP_SYS_FORCE_SHUTDOWN);

	if (ret) {
		pr_err("regmap_update_bits failed %s, %d\n",
			__func__, __LINE__);
		goto vsp_core_pw_off_exit;
	}

vsp_core_pw_off_exit:
	mutex_unlock(&vsp_pw_domain_info->client_lock);
	pr_info("vsp_core_pw_off set OK\n");

	return 0;
}
EXPORT_SYMBOL(vsp_core_pw_off);
#endif

#elif IS_ENABLED(CONFIG_SPRD_VSP_PW_DOMAIN_R6P0)

int vsp_pw_on(u8 client)
{
	int ret;

	ret = sprd_cam_pw_on();
	return ret;
}
EXPORT_SYMBOL(vsp_pw_on);
int vsp_pw_off(u8 client)
{
	sprd_cam_pw_off();
	return 0;
}
EXPORT_SYMBOL(vsp_pw_off);

int vsp_core_pw_on(void)
{
	return 0;
}
EXPORT_SYMBOL(vsp_core_pw_on);

int vsp_core_pw_off(void)
{
	return 0;
}
EXPORT_SYMBOL(vsp_core_pw_off);

#elif IS_ENABLED(CONFIG_SPRD_VSP_PW_DOMAIN_R7P0)

static int is_vsp_domain_power_on(void)
{
	int power_count = 0;
	int ins_count = 0;

	for (ins_count = 0; ins_count < VSP_PW_DOMAIN_COUNT_MAX; ins_count++) {
		power_count +=
		    vsp_pw_domain_info->pw_vsp_info[ins_count].pw_count;
	}

	return (power_count > 0) ? 1 : 0;
}

#define __SPRD_VSP_TIMEOUT            (30 * 1000)

int vsp_pw_on(u8 client)
{
	int ret = 0;
	u32 power_state1, power_state2, power_state3;
	unsigned long timeout = jiffies + msecs_to_jiffies(__SPRD_VSP_TIMEOUT);
	u32 read_count = 0;

	pr_info("vsp_pw_domain:vsp_pw_on Enter client %d\n", client);
	if (client >= VSP_PW_DOMAIN_COUNT_MAX) {
		pr_err("vsp_pw_domain:vsp_pw_on with error client\n");
		return -1;
	}

	mutex_lock(&vsp_pw_domain_info->client_lock);

	if (is_vsp_domain_power_on() == 0) {

		ret = regmap_update_bits(gpr_pmu_apb,
				REG_PMU_APB_PD_MM_VSP_CFG,
				BIT_PMU_APB_PD_MM_VSP_AUTO_SHUTDOWN_EN,
				(unsigned int)
				(~BIT_PMU_APB_PD_MM_VSP_AUTO_SHUTDOWN_EN));
		if (ret) {
			pr_err("regmap_update_bits failed %s, %d\n",
				__func__, __LINE__);
			goto pw_on_exit;
		}

		ret = regmap_update_bits(gpr_pmu_apb,
			REG_PMU_APB_PD_MM_VSP_CFG,
			BIT_PMU_APB_PD_MM_VSP_FORCE_SHUTDOWN,
			(unsigned int)(~BIT_PMU_APB_PD_MM_VSP_FORCE_SHUTDOWN));
		if (ret) {
			pr_err("regmap_update_bits failed %s, %d\n",
				__func__, __LINE__);
			goto pw_on_exit;
		}

		do {
			cpu_relax();
			udelay(300);
			read_count++;

			regmap_read(gpr_pmu_apb,
						REG_PMU_APB_PWR_STATUS5_DBG,
						&power_state1);
			power_state1 &= BIT_PMU_APB_PD_MM_VSP_STATE(0x1F);
			regmap_read(gpr_pmu_apb,
						REG_PMU_APB_PWR_STATUS5_DBG,
						&power_state2);
			power_state2 &= BIT_PMU_APB_PD_MM_VSP_STATE(0x1F);
			regmap_read(gpr_pmu_apb,
						REG_PMU_APB_PWR_STATUS5_DBG,
						&power_state3);
			power_state3 &= BIT_PMU_APB_PD_MM_VSP_STATE(0x1F);

			WARN_ON(time_after(jiffies, timeout));
		} while ((power_state1 && read_count < 100)
			 || (power_state1 != power_state2)
			 || (power_state2 != power_state3));

		if (power_state1) {
			pr_err("vsp_pw_domain:vsp_pw_on set failed 0x%x\n",
			       power_state1);
			mutex_unlock(&vsp_pw_domain_info->client_lock);
			return -1;
		}
		pr_info("vsp_pw_domain:vsp_pw_on set OK\n");
	} else {
		pr_info("vsp_pw_domain:vsp_domain is already power on\n");
	}

	vsp_pw_domain_info->pw_vsp_info[client].pw_state = VSP_PW_DOMAIN_ON;
	vsp_pw_domain_info->pw_vsp_info[client].pw_count++;

pw_on_exit:
	mutex_unlock(&vsp_pw_domain_info->client_lock);
	return ret;
}
EXPORT_SYMBOL(vsp_pw_on);

int vsp_pw_off(u8 client)
{
	int ret = 0;
	unsigned int vsp_fencing_status = 0;

	pr_info("vsp_pw_domain: vsp_pw_off Enter client %d\n", client);
	if (client >= VSP_PW_DOMAIN_COUNT_MAX) {
		pr_err("vsp_pw_domain:vsp_pw_off with error client\n");
		return -1;
	}
	mutex_lock(&vsp_pw_domain_info->client_lock);

	if (vsp_pw_domain_info->pw_vsp_info[client].pw_count >= 1) {

		vsp_pw_domain_info->pw_vsp_info[client].pw_count--;
		if (vsp_pw_domain_info->pw_vsp_info[client].pw_count == 0) {
			vsp_pw_domain_info->pw_vsp_info[client].pw_state =
			    VSP_PW_DOMAIN_OFF;
		}

		if (is_vsp_domain_power_on() == 0) {

			vsp_fencing_status++;

			ret = regmap_update_bits(gpr_pmu_apb,
				REG_PMU_APB_PD_MM_VSP_CFG,
				BIT_PMU_APB_PD_MM_VSP_AUTO_SHUTDOWN_EN,
				(unsigned int)
				(~BIT_PMU_APB_PD_MM_VSP_AUTO_SHUTDOWN_EN));

			if (ret) {
				pr_err("regmap_update_bits failed %s, %d\n",
					__func__, __LINE__);
				goto pw_off_exit;
			}

			ret = regmap_update_bits(gpr_pmu_apb,
				REG_PMU_APB_PD_MM_VSP_CFG,
				BIT_PMU_APB_PD_MM_VSP_FORCE_SHUTDOWN,
				BIT_PMU_APB_PD_MM_VSP_FORCE_SHUTDOWN);
			if (ret) {
				pr_err("regmap_update_bits failed %s, %d\n",
					__func__, __LINE__);
				goto pw_off_exit;
			}

			pr_info("vsp_pw_domain:vsp_pw_off set OK\n");
		}
	} else {
		vsp_pw_domain_info->pw_vsp_info[client].pw_count = 0;
		vsp_pw_domain_info->pw_vsp_info[client].pw_state =
		    VSP_PW_DOMAIN_OFF;
		pr_info("vsp_pw_domain:vsp_domain is already power off\n");
	}

pw_off_exit:

	mutex_unlock(&vsp_pw_domain_info->client_lock);
	return ret;
}
EXPORT_SYMBOL(vsp_pw_off);

int vsp_core_pw_on(void)
{
	return 0;
}
EXPORT_SYMBOL(vsp_core_pw_on);

int vsp_core_pw_off(void)
{
	return 0;
}
EXPORT_SYMBOL(vsp_core_pw_off);

#else
int vsp_pw_on(u8 client)
{
	return 0;
}
EXPORT_SYMBOL(vsp_pw_on);
int vsp_pw_off(u8 client)
{
	return 0;
}
EXPORT_SYMBOL(vsp_pw_off);

int vsp_core_pw_on(void)
{
	return 0;
}
EXPORT_SYMBOL(vsp_core_pw_on);

int vsp_core_pw_off(void)
{
	return 0;
}
EXPORT_SYMBOL(vsp_core_pw_off);
#endif

static int __init vsp_pw_domain_init(void)
{
	int i = 0;

	pr_info("vsp_pw_domain: vsp_pw_domain_init\n");
	vsp_pw_domain_info =
	    kmalloc(sizeof(struct vsp_pw_domain_info_t), GFP_KERNEL);

	for (i = 0; i < VSP_PW_DOMAIN_COUNT_MAX; i++) {
		vsp_pw_domain_info->pw_vsp_info[i].pw_state = VSP_PW_DOMAIN_OFF;
		vsp_pw_domain_info->pw_vsp_info[i].pw_count = 0;
	}
	mutex_init(&vsp_pw_domain_info->client_lock);

	return 0;
}

fs_initcall(vsp_pw_domain_init);
