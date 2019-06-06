/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include "cam_pw_domain.h"

#include "video/sprd_mm.h"

struct cam_pw_domain_info {
	atomic_t users_pw;
	atomic_t users_isptop_pw;
	atomic_t users_clk;

	struct mutex client_lock;
	struct regmap *cam_ahb_gpr;
	struct regmap *pmu_apb_gpr;
	struct clk *cam_ckg_eb;
	struct clk *sys_mtx_eb;
	struct clk *cam_aon_eb;
	struct clk *cam_ahb_clk;
	struct clk *cam_ahb_clk_default;
	struct clk *cam_ahb_clk_parent;
};

static struct cam_pw_domain_info *cam_pw;

int sprd_cam_pw_domain_init(struct platform_device *pdev)
{
	pr_info("%s,  cb: %pS\n", __func__,
			__builtin_return_address(0));

	cam_pw->cam_ckg_eb = devm_clk_get(&pdev->dev, "clk_gate_eb");
	if (IS_ERR(cam_pw->cam_ckg_eb))
		return PTR_ERR(cam_pw->cam_ckg_eb);

	cam_pw->sys_mtx_eb = devm_clk_get(&pdev->dev, "sys_mtx_eb");
	if (IS_ERR(cam_pw->sys_mtx_eb))
		return PTR_ERR(cam_pw->sys_mtx_eb);

	cam_pw->cam_aon_eb = devm_clk_get(&pdev->dev, "cam_aon_eb");
	if (IS_ERR(cam_pw->cam_aon_eb))
		return PTR_ERR(cam_pw->cam_aon_eb);


	cam_pw->cam_ahb_clk = devm_clk_get(&pdev->dev, "clk_ahb_cam");
	if (IS_ERR(cam_pw->cam_ahb_clk))
		return PTR_ERR(cam_pw->cam_ahb_clk);

	cam_pw->cam_ahb_clk_parent =
		devm_clk_get(&pdev->dev, "clk_ahb_cam_parent");
	if (IS_ERR(cam_pw->cam_ahb_clk_parent))
		return PTR_ERR(cam_pw->cam_ahb_clk_parent);

	cam_pw->cam_ahb_clk_default = clk_get_parent(cam_pw->cam_ahb_clk);
	if (IS_ERR(cam_pw->cam_ahb_clk_default))
		return PTR_ERR(cam_pw->cam_ahb_clk_default);


	return 0;
}
EXPORT_SYMBOL(sprd_cam_pw_domain_init);

int sprd_cam_pw_domain_deinit(void)
{
	pr_info("%s,  cb: %pS\n", __func__,
			__builtin_return_address(0));

	cam_pw = NULL;

	return 0;
}
EXPORT_SYMBOL(sprd_cam_pw_domain_deinit);

int sprd_cam_pw_on(void)
{
	int ret = 0;
	unsigned int power_state1;
	unsigned int power_state2;
	unsigned int power_state3;
	unsigned int read_count = 0;
	unsigned int val = 0;

	pr_info("%s, count:%d, cb: %pS\n", __func__,
			atomic_read(&cam_pw->users_pw),
			__builtin_return_address(0));

	mutex_lock(&cam_pw->client_lock);

	if (atomic_inc_return(&cam_pw->users_pw) == 1) {
		/* cam_sys power on */
		ret = regmap_update_bits(cam_pw->pmu_apb_gpr,
				   REG_PMU_APB_PD_CAM_SYS_PWR_CFG,
				   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN,
				   ~(unsigned int)
				   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN);
		if (ret) {
			ret = -3;
			goto err_pw_on;
		}

		ret = regmap_update_bits(cam_pw->pmu_apb_gpr,
				   REG_PMU_APB_PD_CAM_SYS_PWR_CFG,
				   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN,
				   ~(unsigned int)
				   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN);
		if (ret) {
			ret = -3;
			goto err_pw_on;
		}


		do {
			cpu_relax();
			usleep_range(300, 350);
			read_count++;

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret) {
				ret = -2;
				goto err_pw_on;
			}
			power_state1 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x7);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret) {
				ret = -2;
				goto err_pw_on;
			}
			power_state2 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x7);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret) {
				ret = -2;
				goto err_pw_on;
			}
			power_state3 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x7);

		} while ((power_state1 && read_count < 10) ||
			 (power_state1 != power_state2) ||
			 (power_state2 != power_state3));

		if (power_state1) {
			pr_err("%s: cam_sys pw on failed 0x%x\n",
			       __func__, power_state1);
			ret = -1;
			goto err_pw_on;
		} else
		    pr_info("%s: cam_sys pw on!!\n", __func__);
	} else {
		pr_info("%s: cam_sys already power on\n", __func__);
	}

	mutex_unlock(&cam_pw->client_lock);
	return 0;

err_pw_on:
	atomic_dec_return(&cam_pw->users_pw);
	regmap_read(cam_pw->pmu_apb_gpr,
			    REG_PMU_APB_PD_CAM_SYS_PWR_CFG, &val);
	if (ret == -3)
		pr_err("%s: set cam_sys pmu failed!\n", __func__);
	if (ret == -2)
		pr_err("%s: read pw_state4 reg failed!\n", __func__);
	pr_err("%s: failed, ret: %d, read_count: %d, cam_sys_pmu_reg: 0x%x!\n",
		       __func__, ret, read_count, val);

	mutex_unlock(&cam_pw->client_lock);
	return ret;
}
EXPORT_SYMBOL(sprd_cam_pw_on);

int sprd_cam_pw_off(void)
{
	int ret = 0;
	unsigned int power_state1;
	unsigned int power_state2;
	unsigned int power_state3;
	unsigned int read_count = 0;
	unsigned int val = 0;

	pr_info("%s, count:%d, cb: %pS\n", __func__,
			atomic_read(&cam_pw->users_pw),
			__builtin_return_address(0));

	mutex_lock(&cam_pw->client_lock);

	if (atomic_dec_return(&cam_pw->users_pw) == 0) {

		usleep_range(300, 350);

		ret = regmap_update_bits(cam_pw->pmu_apb_gpr,
				REG_PMU_APB_PD_CAM_SYS_PWR_CFG,
				BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN,
				~(unsigned int)
				BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN);
		if (ret) {
			ret = -3;
			goto err_pw_off;
		}

		ret = regmap_update_bits(cam_pw->pmu_apb_gpr,
				REG_PMU_APB_PD_CAM_SYS_PWR_CFG,
				BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN,
				BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN);
		if (ret) {
			ret = -3;
			goto err_pw_off;
		}

		do {
			cpu_relax();
			usleep_range(300, 350);
			read_count++;

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret) {
				ret = -2;
				goto err_pw_off;
			}
			power_state1 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x7);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret) {
				ret = -2;
				goto err_pw_off;
			}
			power_state2 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x7);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret) {
				ret = -2;
				goto err_pw_off;
			}
			power_state3 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x7);

		} while (((power_state1 != REG_PMU_CAM_SYS_PD_STATE)
					&& read_count < 10) ||
			 (power_state1 != power_state2) ||
			 (power_state2 != power_state3));

		if (power_state1 != REG_PMU_CAM_SYS_PD_STATE) {
			pr_err("%s: cam_sys pw off failed 0x%x\n",
					__func__, power_state1);
			ret = -1;
			goto err_pw_off;
		} else
		    pr_info("%s: cam_sys pw off!!\n", __func__);
	} else {
	    pr_info("%s: cam_sys, other camera module is working\n", __func__);
	}

	mutex_unlock(&cam_pw->client_lock);
	return 0;

err_pw_off:
	regmap_read(cam_pw->pmu_apb_gpr,
			    REG_PMU_APB_PD_CAM_SYS_PWR_CFG, &val);
	if (ret == -3)
		pr_err("%s: set cam_sys pmu failed!\n", __func__);
	if (ret == -2)
		pr_err("%s: read pw_state4 reg failed!\n", __func__);
	pr_err("%s: failed, ret: %d, read_count: %d,cam_sys_pmu_reg: 0x%x!\n",
		       __func__, ret, read_count, val);
	mutex_unlock(&cam_pw->client_lock);
	return ret;
}
EXPORT_SYMBOL(sprd_cam_pw_off);

int sprd_isp_pw_on(void)
{
	int ret = 0;
	unsigned int power_state1;
	unsigned int power_state2;
	unsigned int power_state3;
	unsigned int read_count = 0;
	unsigned int val = 0;

	pr_info("%s, refcnt=%d, cb: %pS\n", __func__,
			atomic_read(&cam_pw->users_isptop_pw),
			__builtin_return_address(0));

	mutex_lock(&cam_pw->client_lock);

	if (atomic_inc_return(&cam_pw->users_isptop_pw) == 1) {
		ret = regmap_update_bits(cam_pw->pmu_apb_gpr,
				REG_PMU_APB_PD_ISP_TOP_PWR_CFG,
				BIT_PMU_APB_PD_ISP_TOP_AUTO_SHUTDOWN_EN,
				~(unsigned int)
				BIT_PMU_APB_PD_ISP_TOP_AUTO_SHUTDOWN_EN);
		if (ret) {
			ret = -3;
			goto err_pw_on;
		}

		ret = regmap_update_bits(cam_pw->pmu_apb_gpr,
				REG_PMU_APB_PD_ISP_TOP_PWR_CFG,
				BIT_PMU_APB_PD_ISP_TOP_FORCE_SHUTDOWN,
				~(unsigned int)
				BIT_PMU_APB_PD_ISP_TOP_FORCE_SHUTDOWN);
		if (ret) {
			ret = -3;
			goto err_pw_on;
		}

		mdelay(35);
		do {
			cpu_relax();
			usleep_range(300, 350);
			read_count++;

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret) {
				ret = -2;
				goto err_pw_on;
			}
			power_state1 = val & BIT_PMU_APB_PD_ISP_TOP_STATE(0x7);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret) {
				ret = -2;
				goto err_pw_on;
			}
			power_state2 = val & BIT_PMU_APB_PD_ISP_TOP_STATE(0x7);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret) {
				ret = -2;
				goto err_pw_on;
			}
			power_state3 = val & BIT_PMU_APB_PD_ISP_TOP_STATE(0x7);

		} while ((power_state1 && read_count < 10) ||
			 (power_state1 != power_state2) ||
			 (power_state2 != power_state3));

		if (power_state1) {
			pr_err("%s: isp_top pw on failed 0x%x\n",
					__func__, power_state1);
			ret = -1;
			goto err_pw_on;
		} else
			pr_info("%s: isp_top power on!!\n", __func__);
	} else {
		pr_info("%s: isp_top already power on!!\n", __func__);
	}

	mutex_unlock(&cam_pw->client_lock);
	return 0;

err_pw_on:
	atomic_dec_return(&cam_pw->users_isptop_pw);
	regmap_read(cam_pw->pmu_apb_gpr,
			    REG_PMU_APB_PD_ISP_TOP_PWR_CFG, &val);
	if (ret == -3)
		pr_err("%s: set ip_top pmu failed!\n", __func__);
	if (ret == -2)
		pr_err("%s: read pw_state4 reg failed!\n", __func__);
	pr_err("%s: failed, ret: %d, read_count: %d, isp_top_pmu_reg: 0x%x!\n",
		       __func__, ret, read_count, val);

	mutex_unlock(&cam_pw->client_lock);
	return ret;
}
EXPORT_SYMBOL(sprd_isp_pw_on);

int sprd_isp_pw_off(void)
{
	int ret = 0;
	unsigned int power_state1;
	unsigned int power_state2;
	unsigned int power_state3;
	unsigned int read_count = 0;
	unsigned int val = 0;

	pr_info("%s, refcnt:%d, cb: %pS\n", __func__,
			atomic_read(&cam_pw->users_isptop_pw),
			__builtin_return_address(0));

	mutex_lock(&cam_pw->client_lock);

	if (atomic_dec_return(&cam_pw->users_isptop_pw) == 0) {
		ret = regmap_update_bits(cam_pw->pmu_apb_gpr,
				REG_PMU_APB_PD_ISP_TOP_PWR_CFG,
				BIT_PMU_APB_PD_ISP_TOP_AUTO_SHUTDOWN_EN,
				~(unsigned int)
				BIT_PMU_APB_PD_ISP_TOP_AUTO_SHUTDOWN_EN);
		if (ret) {
			ret = -3;
			goto err_pw_off;
		}

		ret = regmap_update_bits(cam_pw->pmu_apb_gpr,
				REG_PMU_APB_PD_ISP_TOP_PWR_CFG,
				BIT_PMU_APB_PD_ISP_TOP_FORCE_SHUTDOWN,
				BIT_PMU_APB_PD_ISP_TOP_FORCE_SHUTDOWN);
		if (ret) {
			ret = -3;
			goto err_pw_off;
		}

		do {
			cpu_relax();
			usleep_range(300, 350);
			read_count++;

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret) {
				ret = -2;
				goto err_pw_off;
			}
			power_state1 = val & BIT_PMU_APB_PD_ISP_TOP_STATE(0x7);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret) {
				ret = -2;
				goto err_pw_off;
			}
			power_state2 = val & BIT_PMU_APB_PD_ISP_TOP_STATE(0x7);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret) {
				ret = -2;
				goto err_pw_off;
			}
			power_state3 = val & BIT_PMU_APB_PD_ISP_TOP_STATE(0x7);

		} while (((power_state1 != REG_PMU_ISP_TOP_PD_STATE)
					&& read_count < 10) ||
			 (power_state1 != power_state2) ||
			 (power_state2 != power_state3));

		if (power_state1 != REG_PMU_ISP_TOP_PD_STATE) {
			pr_err("%s: isp_top pw off failed 0x%x\n",
					__func__, power_state1);
			ret = -1;
			goto err_pw_off;
		} else
		    pr_info("%s: isp_top pw off!!\n", __func__);
	} else {
		pr_info("%s: isp_top still has users!!\n", __func__);
	}

	mutex_unlock(&cam_pw->client_lock);
	return 0;

err_pw_off:
	regmap_read(cam_pw->pmu_apb_gpr,
			    REG_PMU_APB_PD_ISP_TOP_PWR_CFG, &val);
	if (ret == -3)
		pr_err("%s: set ip_top pmu failed!\n", __func__);
	if (ret == -2)
		pr_err("%s: read pw_state4 reg failed!\n", __func__);
	pr_err("%s: failed, ret: %d, read_count: %d, isp_top_pmu_reg: 0x%x!\n",
		       __func__, ret, read_count, val);

	mutex_unlock(&cam_pw->client_lock);
	return ret;
}
EXPORT_SYMBOL(sprd_isp_pw_off);

int sprd_cam_domain_eb(void)
{
	pr_info("%s, count:%d, cb: %pS\n", __func__,
			atomic_read(&cam_pw->users_clk),
			__builtin_return_address(0));

	mutex_lock(&cam_pw->client_lock);

	if (atomic_inc_return(&cam_pw->users_clk) == 1) {
		clk_prepare_enable(cam_pw->cam_aon_eb);
		clk_prepare_enable(cam_pw->cam_ckg_eb);
		clk_prepare_enable(cam_pw->sys_mtx_eb);
		clk_set_parent(cam_pw->cam_ahb_clk, cam_pw->cam_ahb_clk_parent);
		clk_prepare_enable(cam_pw->cam_ahb_clk);

		pr_info("%s: cam domain_eb enable!!", __func__);
	} else
	    pr_info("%s: cam domain_eb already enable!!", __func__);

	mutex_unlock(&cam_pw->client_lock);

	return 0;
}
EXPORT_SYMBOL(sprd_cam_domain_eb);

int sprd_cam_domain_disable(void)
{
	pr_info("%s, count:%d, cb: %pS\n", __func__,
		atomic_read(&cam_pw->users_clk),
		__builtin_return_address(0));

	mutex_lock(&cam_pw->client_lock);

	if (atomic_dec_return(&cam_pw->users_clk) == 0) {
		clk_set_parent(cam_pw->cam_ahb_clk,
				cam_pw->cam_ahb_clk_default);
		clk_disable_unprepare(cam_pw->cam_ahb_clk);
		clk_disable_unprepare(cam_pw->sys_mtx_eb);
		clk_disable_unprepare(cam_pw->cam_ckg_eb);
		clk_disable_unprepare(cam_pw->cam_aon_eb);
		pr_info("%s: cam domain_eb disable!!", __func__);
	} else
	    pr_info("%s: can't disable cam domain_eb\n", __func__);

	mutex_unlock(&cam_pw->client_lock);

	return 0;
}
EXPORT_SYMBOL(sprd_cam_domain_disable);

static int __init sprd_mm_pw_domain_init(void)
{
	struct regmap *cam_ahb_gpr;
	struct regmap *pmu_apb_gpr;

	pr_info("%s:mm power domain init\n", __func__);

	cam_pw = kzalloc(sizeof(*cam_pw), GFP_KERNEL);
	if (!cam_pw)
		return -ENOMEM;

	cam_ahb_gpr =
		syscon_regmap_lookup_by_compatible("sprd,iwhale2-lpc-ahb");
	if (IS_ERR(cam_ahb_gpr))
		return PTR_ERR(cam_ahb_gpr);
	cam_pw->cam_ahb_gpr = cam_ahb_gpr;

	pmu_apb_gpr =
		syscon_regmap_lookup_by_compatible("sprd,iwhale2-aon-pwu-apb");
	if (IS_ERR(pmu_apb_gpr))
		return PTR_ERR(pmu_apb_gpr);
	cam_pw->pmu_apb_gpr = pmu_apb_gpr;

	mutex_init(&cam_pw->client_lock);

	return 0;
}
fs_initcall(sprd_mm_pw_domain_init);

