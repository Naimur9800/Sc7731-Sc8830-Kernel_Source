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
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include "cam_pw_domain.h"

#define SHAR				0x53686172
#define KLJ1				0x6B4C4A31

struct cam_pw_domain_info {
	atomic_t users_pw;
	atomic_t users_clk;

	unsigned int chip_id0;
	unsigned int chip_id1;

	struct mutex client_lock;
	struct regmap *cam_ahb_gpr;
	struct regmap *aon_apb_gpr;
	struct regmap *pmu_apb_gpr;

	struct clk *cam_clk_cphy_cfg_gate_eb;
	struct clk *cam_ckg_eb;
	struct clk *cam_mm_eb;
	struct clk *cam_ahb_clk;
	struct clk *cam_ahb_clk_default;
	struct clk *cam_ahb_clk_parent;
};

static struct cam_pw_domain_info *cam_pw;

int sprd_cam_pw_domain_init(struct platform_device *pdev)
{
	int ret = 0;
	struct regmap *cam_ahb_gpr = NULL;
	struct regmap *aon_apb_gpr = NULL;
	struct regmap *pmu_apb_gpr = NULL;
	unsigned int chip_id0 = 0, chip_id1 = 0;

	cam_pw = devm_kzalloc(&pdev->dev, sizeof(*cam_pw), GFP_KERNEL);
	if (!cam_pw)
		return -ENOMEM;

	cam_pw->cam_clk_cphy_cfg_gate_eb =
		devm_clk_get(&pdev->dev, "clk_cphy_cfg_gate_eb");
	if (IS_ERR_OR_NULL(cam_pw->cam_clk_cphy_cfg_gate_eb))
		return PTR_ERR(cam_pw->cam_clk_cphy_cfg_gate_eb);

	cam_pw->cam_ckg_eb = devm_clk_get(&pdev->dev, "clk_gate_eb");
	if (IS_ERR_OR_NULL(cam_pw->cam_ckg_eb))
		return PTR_ERR(cam_pw->cam_ckg_eb);

	cam_pw->cam_mm_eb = devm_clk_get(&pdev->dev, "clk_mm_eb");
	if (IS_ERR_OR_NULL(cam_pw->cam_mm_eb))
		return PTR_ERR(cam_pw->cam_mm_eb);

	cam_pw->cam_ahb_clk = devm_clk_get(&pdev->dev, "clk_mm_ahb");
	if (IS_ERR_OR_NULL(cam_pw->cam_ahb_clk))
		return PTR_ERR(cam_pw->cam_ahb_clk);

	cam_pw->cam_ahb_clk_parent =
		devm_clk_get(&pdev->dev, "clk_mm_ahb_parent");
	if (IS_ERR_OR_NULL(cam_pw->cam_ahb_clk_parent))
		return PTR_ERR(cam_pw->cam_ahb_clk_parent);

	cam_pw->cam_ahb_clk_default = clk_get_parent(cam_pw->cam_ahb_clk);
	if (IS_ERR_OR_NULL(cam_pw->cam_ahb_clk_default))
		return PTR_ERR(cam_pw->cam_ahb_clk_default);

	cam_ahb_gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						      "sprd,cam-ahb-syscon");
	if (IS_ERR_OR_NULL(cam_ahb_gpr))
		return PTR_ERR(cam_ahb_gpr);
	cam_pw->cam_ahb_gpr = cam_ahb_gpr;

	aon_apb_gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						      "sprd,aon-apb-syscon");
	if (IS_ERR_OR_NULL(aon_apb_gpr))
		return PTR_ERR(aon_apb_gpr);
	cam_pw->aon_apb_gpr = aon_apb_gpr;

	pmu_apb_gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						      "sprd,syscon-pmu-apb");
	if (IS_ERR_OR_NULL(pmu_apb_gpr))
		return PTR_ERR(pmu_apb_gpr);
	cam_pw->pmu_apb_gpr = pmu_apb_gpr;

	ret = regmap_read(aon_apb_gpr, REG_AON_APB_AON_CHIP_ID0, &chip_id0);
	if (ret) {
		cam_pw->chip_id0 = 0;
		pr_err("Read chip id0 error\n");
	} else
		cam_pw->chip_id0 = chip_id0;

	ret = regmap_read(aon_apb_gpr, REG_AON_APB_AON_CHIP_ID1, &chip_id1);
	if (ret) {
		cam_pw->chip_id1 = 0;
		pr_err("Read chip id1 error\n");
	} else
		cam_pw->chip_id1 = chip_id1;

	pr_debug("chip_id0 %x, chip_id1 %x\n", chip_id0, chip_id1);

	mutex_init(&cam_pw->client_lock);

	return 0;
}
EXPORT_SYMBOL(sprd_cam_pw_domain_init);

int sprd_cam_pw_off(void)
{
	int ret = 0;
	unsigned int power_state1 = 0;
	unsigned int power_state2 = 0;
	unsigned int power_state3 = 0;
	unsigned int read_count = 0;
	unsigned int val = 0;
	unsigned int pmu_mm_bit = 0, pmu_mm_state = 0;
	unsigned int mm_off = 0;

	pr_info("%s, count:%d, cb: %pS\n", __func__,
			atomic_read(&cam_pw->users_pw),
			__builtin_return_address(0));

	if (cam_pw->chip_id0 == KLJ1 && cam_pw->chip_id1 == SHAR) {
		pmu_mm_bit = 27;
		pmu_mm_state = 0x1f;
		mm_off = 0x38000000;
	} else {
		pmu_mm_bit = 28;
		pmu_mm_state = 0xf;
		mm_off = 0x70000000;
	}

	mutex_lock(&cam_pw->client_lock);

	if (atomic_dec_return(&cam_pw->users_pw) == 0) {

		usleep_range(300, 350);

		regmap_update_bits(cam_pw->pmu_apb_gpr,
				   REG_PMU_APB_PD_MM_SYS_CFG,
				   BIT_PMU_APB_PD_MM_SYS_AUTO_SHUTDOWN_EN,
				   ~(unsigned int)
				   BIT_PMU_APB_PD_MM_SYS_AUTO_SHUTDOWN_EN);
		regmap_update_bits(cam_pw->pmu_apb_gpr,
				   REG_PMU_APB_PD_MM_SYS_CFG,
				   BIT_PMU_APB_PD_MM_SYS_FORCE_SHUTDOWN,
				   BIT_PMU_APB_PD_MM_SYS_FORCE_SHUTDOWN);

		do {
			cpu_relax();
			usleep_range(300, 350);
			read_count++;

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PD_STATE, &val);
			if (ret)
				goto err_pw_off;
			power_state1 = val & (pmu_mm_state << pmu_mm_bit);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PD_STATE, &val);
			if (ret)
				goto err_pw_off;
			power_state2 = val & (pmu_mm_state << pmu_mm_bit);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PD_STATE, &val);
			if (ret)
				goto err_pw_off;
			power_state3 = val & (pmu_mm_state << pmu_mm_bit);
		} while (((power_state1 != mm_off) && read_count < 10) ||
			 (power_state1 != power_state2) ||
			 (power_state2 != power_state3));

		if (power_state1 != mm_off) {
			pr_err("cam domain pw off failed 0x%x\n", power_state1);
			ret = -1;
			goto err_pw_off;
		}
	} else {
		pr_info("cam domain, other camera module is working\n");
	}
	mutex_unlock(&cam_pw->client_lock);
	return 0;

err_pw_off:
	pr_err("cam domain pw off failed, ret: %d, count: %d!\n",
	       ret, read_count);
	mutex_unlock(&cam_pw->client_lock);

	return 0;
}
EXPORT_SYMBOL(sprd_cam_pw_off);

int sprd_cam_pw_on(void)
{
	int ret = 0;
	unsigned int power_state1 = 0;
	unsigned int power_state2 = 0;
	unsigned int power_state3 = 0;
	unsigned int read_count = 0;
	unsigned int val = 0;
	unsigned int pmu_mm_bit = 0, pmu_mm_state = 0;

	pr_info("%s, count:%d, cb: %pS\n", __func__,
			atomic_read(&cam_pw->users_pw),
			__builtin_return_address(0));

	if (cam_pw->chip_id0 == KLJ1 && cam_pw->chip_id1 == SHAR) {
		pmu_mm_bit = 27;
		pmu_mm_state = 0x1f;
	} else {
		pmu_mm_bit = 28;
		pmu_mm_state = 0xf;
	}

	mutex_lock(&cam_pw->client_lock);

	if (atomic_inc_return(&cam_pw->users_pw) == 1) {
		/* cam domain power on */
		regmap_update_bits(cam_pw->pmu_apb_gpr,
				   REG_PMU_APB_PD_MM_SYS_CFG,
				   BIT_PMU_APB_PD_MM_SYS_AUTO_SHUTDOWN_EN,
				   ~(unsigned int)
				   BIT_PMU_APB_PD_MM_SYS_AUTO_SHUTDOWN_EN);
		regmap_update_bits(cam_pw->pmu_apb_gpr,
				   REG_PMU_APB_PD_MM_SYS_CFG,
				   BIT_PMU_APB_PD_MM_SYS_FORCE_SHUTDOWN,
				   ~(unsigned int)
				   BIT_PMU_APB_PD_MM_SYS_FORCE_SHUTDOWN);

		do {
			cpu_relax();
			usleep_range(300, 350);
			read_count++;

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PD_STATE, &val);
			if (ret)
				goto err_pw_on;
			power_state1 = val & (pmu_mm_state << pmu_mm_bit);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PD_STATE, &val);
			if (ret)
				goto err_pw_on;
			power_state2 = val & (pmu_mm_state << pmu_mm_bit);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PD_STATE, &val);
			if (ret)
				goto err_pw_on;
			power_state3 = val & (pmu_mm_state << pmu_mm_bit);

		} while ((power_state1 && read_count < 10) ||
			 (power_state1 != power_state2) ||
			 (power_state2 != power_state3));

		if (power_state1) {
			pr_err("cam domain pw on failed 0x%x\n", power_state1);
			ret = -1;
			goto err_pw_on;
		}
	} else {
		pr_info("cam domain, already power on\n");
	}
	mutex_unlock(&cam_pw->client_lock);

	return 0;

err_pw_on:
	atomic_dec_return(&cam_pw->users_pw);
	pr_info("cam domain, failed to power on\n");
	mutex_unlock(&cam_pw->client_lock);

	return ret;
}
EXPORT_SYMBOL(sprd_cam_pw_on);

int sprd_cam_domain_eb(void)
{
	unsigned int rst_bit = 0;
	unsigned int eb_bit = 0;

	pr_info("%s, count:%d, cb: %pS\n", __func__,
			atomic_read(&cam_pw->users_clk),
			__builtin_return_address(0));

	mutex_lock(&cam_pw->client_lock);

	if (atomic_inc_return(&cam_pw->users_clk) == 1) {
		/* mm bus enable */
		clk_prepare_enable(cam_pw->cam_mm_eb);

		/* cam CKG enable */
		clk_prepare_enable(cam_pw->cam_ckg_eb);

		clk_prepare_enable(cam_pw->cam_clk_cphy_cfg_gate_eb);

		/* config cam ahb clk */
		clk_set_parent(cam_pw->cam_ahb_clk, cam_pw->cam_ahb_clk_parent);
		clk_prepare_enable(cam_pw->cam_ahb_clk);

		eb_bit = BIT(0) | BIT(1) | BIT(3) | BIT(4) | BIT(6) | BIT(7);
		regmap_update_bits(cam_pw->cam_ahb_gpr,
				   REG_MM_AHB_AHB_EB,
				   eb_bit,
				   eb_bit);

		rst_bit = 0xF | BIT(5) | BIT(7) | BIT(9) | BIT(10) |
			BIT(11) | BIT(12) | BIT(15);
		regmap_update_bits(cam_pw->cam_ahb_gpr,
				   REG_MM_AHB_AHB_RST,
				   rst_bit,
				   rst_bit);
		udelay(1);
		regmap_update_bits(cam_pw->cam_ahb_gpr,
				   REG_MM_AHB_AHB_RST,
				   rst_bit,
				   ~rst_bit);
	}
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

		clk_disable_unprepare(cam_pw->cam_ckg_eb);

		clk_disable_unprepare(cam_pw->cam_clk_cphy_cfg_gate_eb);

		clk_disable_unprepare(cam_pw->cam_mm_eb);
	}
	mutex_unlock(&cam_pw->client_lock);

	return 0;
}
EXPORT_SYMBOL(sprd_cam_domain_disable);
