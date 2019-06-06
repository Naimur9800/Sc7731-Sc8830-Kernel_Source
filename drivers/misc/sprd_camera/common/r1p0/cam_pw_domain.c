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
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include "cam_pw_domain.h"

#define CAM_TIMEOUT              (3 * 1000)

struct cam_pw_domain_info {
	atomic_t users;

	struct regmap *cam_ahb_gpr;
	struct regmap *pmu_apb_gpr;

	struct clk *cam_eb;
	struct clk *cam_ckg_eb;
	struct clk *cam_ahb_clk;
	struct clk *cam_ahb_clk_default;
	struct clk *cam_ahb_clk_parent;
};


#if !IS_ENABLED(CONFIG_SPRD_CAM_PW_DOMAIN_R1P0)

int sprd_cam_pw_domain_init(struct platform_device *pdev)
{
	return 0;
}
EXPORT_SYMBOL(sprd_cam_pw_domain_init);

int sprd_cam_pw_domain_deinit(void)
{
	return 0;
}
EXPORT_SYMBOL(sprd_cam_pw_domain_deinit);

int sprd_cam_pw_off(void)
{
	return 0;
}
EXPORT_SYMBOL(sprd_cam_pw_off);

int sprd_cam_pw_on(void)
{
	return 0;
}
EXPORT_SYMBOL(sprd_cam_pw_on);

int sprd_cam_domain_eb(void)
{
	return 0;
}
EXPORT_SYMBOL(sprd_cam_domain_eb);

int sprd_cam_domain_disable(void)
{
	return 0;
}
EXPORT_SYMBOL(sprd_cam_domain_disable);
#else
static struct cam_pw_domain_info *cam_pw;

int sprd_cam_pw_domain_init(struct platform_device *pdev)
{
	struct regmap *cam_ahb_gpr;
	struct regmap *pmu_apb_gpr;

	cam_pw = devm_kzalloc(&pdev->dev, sizeof(*cam_pw), GFP_KERNEL);
	if (!cam_pw)
		return -ENOMEM;

	cam_pw->cam_eb = devm_clk_get(&pdev->dev, "clk_cam_eb");
	if (IS_ERR(cam_pw->cam_eb))
		return PTR_ERR(cam_pw->cam_eb);

	cam_pw->cam_ckg_eb = devm_clk_get(&pdev->dev, "cam_ckg_eb");
	if (IS_ERR(cam_pw->cam_ckg_eb))
		return PTR_ERR(cam_pw->cam_ckg_eb);

	cam_pw->cam_ahb_clk = devm_clk_get(&pdev->dev, "clk_ahb_cam");
	if (IS_ERR(cam_pw->cam_ahb_clk))
		return PTR_ERR(cam_pw->cam_ahb_clk);

	cam_pw->cam_ahb_clk_parent =
		devm_clk_get(&pdev->dev, "clk_ahb_cam_parent");
	if (IS_ERR(cam_pw->cam_ahb_clk_parent))
		return PTR_ERR(cam_pw->cam_ahb_clk_parent);

	cam_ahb_gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						      "sprd,syscon-cam-ahb");
	if (IS_ERR(cam_ahb_gpr))
		return PTR_ERR(cam_ahb_gpr);
	cam_pw->cam_ahb_gpr = cam_ahb_gpr;

	pmu_apb_gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						      "sprd,syscon-pmu-apb");
	if (IS_ERR(pmu_apb_gpr))
		return PTR_ERR(pmu_apb_gpr);
	cam_pw->pmu_apb_gpr = pmu_apb_gpr;

	cam_pw->cam_ahb_clk_default = clk_get_parent(cam_pw->cam_ahb_clk);
	if (IS_ERR(cam_pw->cam_ahb_clk_default))
		return PTR_ERR(cam_pw->cam_ahb_clk_default);

	return 0;
}
EXPORT_SYMBOL(sprd_cam_pw_domain_init);

int sprd_cam_pw_domain_deinit(void)
{
	cam_pw = NULL;

	return 0;
}
EXPORT_SYMBOL(sprd_cam_pw_domain_deinit);

static void sprd_cam_status_read(void)
{
	unsigned long base_address = 0;
	unsigned int cam_pw_status[2] = {0x0};

	base_address = (unsigned long)ioremap_nocache(0x402e310c, 0x4);
	regmap_read(cam_pw->pmu_apb_gpr,
		REG_PMU_APB_NOC_SLEEP_STOP_BYP, &cam_pw_status[0]);
	cam_pw_status[1] = readl_relaxed((void *)base_address);
	pr_info("cam_pw_status: 0x%.8x 0x%.8x\n",
		cam_pw_status[0],
		cam_pw_status[1]);

	iounmap((void *)base_address);
}

int sprd_cam_pw_off(void)
{
	int ret = 0;
	unsigned int power_state1, power_state2, power_state3;
	unsigned long timeout = jiffies + msecs_to_jiffies(CAM_TIMEOUT);
	unsigned int read_count = 0;
	unsigned int val;

	pr_info("%s, count:%d, cb: %pS\n", __func__,
			atomic_read(&cam_pw->users),
			__builtin_return_address(0));
	if (atomic_dec_return(&cam_pw->users) == 0) {
		clk_set_parent(cam_pw->cam_ahb_clk,
			       cam_pw->cam_ahb_clk_default);
		clk_disable_unprepare(cam_pw->cam_ahb_clk);
		clk_disable_unprepare(cam_pw->cam_ckg_eb);
		clk_disable_unprepare(cam_pw->cam_eb);

		usleep_range(300, 350);

		regmap_update_bits(cam_pw->pmu_apb_gpr,
				   REG_PMU_APB_PD_CAM_SYS_CFG,
				   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN,
				   ~(unsigned int)
				   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN);
		regmap_update_bits(cam_pw->pmu_apb_gpr,
				   REG_PMU_APB_PD_CAM_SYS_CFG,
				   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN,
				   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN);

		do {
			cpu_relax();
			usleep_range(300, 350);
			read_count++;

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATUS4_DBG, &val);
			if (ret)
				goto err_pw_off;
			power_state1 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x1f);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATUS4_DBG, &val);
			if (ret)
				goto err_pw_off;
			power_state2 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x1f);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATUS4_DBG, &val);
			if (ret)
				goto err_pw_off;
			power_state3 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x1f);

			BUG_ON(time_after(jiffies, timeout));
		} while (((power_state1 != 0xe0) && read_count < 10) ||
			 (power_state1 != power_state2) ||
			 (power_state2 != power_state3));

		if (power_state1 != 0xe0) {
			pr_err("cam domain pw off failed 0x%x\n", power_state1);
			ret = -1;
			goto err_pw_off;
		}
		sprd_cam_status_read();
	} else {
		pr_debug("cam domain, already power off\n");
	}
	return 0;

err_pw_off:
	pr_err("cam domain pw off failed, ret: %d, count: %d!\n",
	       ret, read_count);
	return 0;
}
EXPORT_SYMBOL(sprd_cam_pw_off);

/*
 * When clk tree initiated, the cam system hasn't been enable, clk sub system
 * can't get real sensor's default clock, so it initiate sensor's clock as 26m.
 * But when cam system enabled, the default clock is 48m, soft-ware record
 * and hard-ware record are not match. So, we change hard-ware clock to 26m
 * after cam system enable.
 */
static void sprd_sensor_init_clk(void)
{
	unsigned long  base_address = 0;

	pr_info("begin to init sensor clock!\n");
	base_address = (unsigned long)ioremap_nocache(0x62000024, 0xc);
	writel_relaxed(0x0, (void *)(base_address));
	writel_relaxed(0x0, (void *)(base_address + 0x4));
	writel_relaxed(0x0, (void *)(base_address + 0x8));

	iounmap((void *)base_address);
}

int sprd_cam_pw_on(void)
{
	int ret = 0;
	unsigned int power_state1, power_state2, power_state3;
	unsigned long timeout = jiffies + msecs_to_jiffies(CAM_TIMEOUT);
	unsigned int read_count = 0;
	unsigned int val;
	unsigned int rst_bit;
	unsigned int flag;

	pr_info("%s, count:%d, cb: %pS\n", __func__,
			atomic_read(&cam_pw->users),
			__builtin_return_address(0));
	if (atomic_inc_return(&cam_pw->users) == 1) {
		/* cam domain power on */
		regmap_update_bits(cam_pw->pmu_apb_gpr,
				   REG_PMU_APB_PD_CAM_SYS_CFG,
				   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN,
				   ~(unsigned int)
				   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN);
		regmap_update_bits(cam_pw->pmu_apb_gpr,
				   REG_PMU_APB_PD_CAM_SYS_CFG,
				   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN,
				   ~(unsigned int)
				   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN);

		do {
			cpu_relax();
			usleep_range(300, 350);
			read_count++;

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATUS4_DBG, &val);
			if (ret)
				goto err_pw_on;
			power_state1 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x1f);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATUS4_DBG, &val);
			if (ret)
				goto err_pw_on;
			power_state2 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x1f);

			ret = regmap_read(cam_pw->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATUS4_DBG, &val);
			if (ret)
				goto err_pw_on;
			power_state3 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x1f);

			BUG_ON(time_after(jiffies, timeout));
		} while ((power_state1 && read_count < 10) ||
			 (power_state1 != power_state2) ||
			 (power_state2 != power_state3));

		if (power_state1) {
			pr_err("cam domain pw on failed 0x%x\n", power_state1);
			ret = -1;
			goto err_pw_on;
		}

		/* cam domain enable */
		clk_prepare_enable(cam_pw->cam_eb);

		/* cam CKG enable */
		clk_prepare_enable(cam_pw->cam_ckg_eb);

		/* config cam ahb clk */
		clk_set_parent(cam_pw->cam_ahb_clk, cam_pw->cam_ahb_clk_parent);
		clk_prepare_enable(cam_pw->cam_ahb_clk);

		rst_bit = BIT_CAM_AHB_DCAM0_SOFT_RST |
			BIT_CAM_AHB_DCAM1_SOFT_RST |
			BIT_CAM_AHB_DCAM0_CAM0_SOFT_RST |
			BIT_CAM_AHB_DCAM0_CAM1_SOFT_RST |
			BIT_CAM_AHB_DCAM0_CAM2_SOFT_RST |
			BIT_CAM_AHB_DCAM0_CAM3_SOFT_RST |
			BIT_CAM_AHB_DCAM1_CAM0_SOFT_RST |
			BIT_CAM_AHB_DCAM1_CAM1_SOFT_RST |
			BIT_CAM_AHB_DCAM1_CAM2_SOFT_RST |
			BIT_CAM_AHB_DCAM1_CAM3_SOFT_RST |
			BIT_CAM_AHB_CCIR_SOFT_RST |
			BIT_CAM_AHB_CSI0_SOFT_RST |
			BIT_CAM_AHB_CSI1_SOFT_RST |
			BIT_CAM_AHB_CKG_SOFT_RST |
			BIT_CAM_AHB_MMU_SOFT_RST |
			BIT_CAM_AHB_MMU_PF_SOFT_RST |
			BIT_CAM_AHB_CPP_SOFT_RST;
		regmap_update_bits(cam_pw->cam_ahb_gpr,
				   REG_CAM_AHB_AHB_RST,
				   rst_bit,
				   rst_bit);
		udelay(1);
		regmap_update_bits(cam_pw->cam_ahb_gpr,
				   REG_CAM_AHB_AHB_RST,
				   rst_bit,
				   ~rst_bit);

		rst_bit = BIT_CAM_AHB_SM_DCAM0_IF_IN_DCAM_SOFT_RST |
			BIT_CAM_AHB_SM_DCAM1_IF_IN_DCAM_SOFT_RST;
		regmap_update_bits(cam_pw->cam_ahb_gpr,
				   REG_CAM_AHB_MODULE_SOFT_RST,
				   rst_bit,
				   rst_bit);
		udelay(1);
		regmap_update_bits(cam_pw->cam_ahb_gpr,
				   REG_CAM_AHB_MODULE_SOFT_RST,
				   rst_bit,
				   ~rst_bit);

		flag = BIT_CAM_AHB_MMU_EB |
			BIT_CAM_AHB_MMU_PF_EB |
			BIT_CAM_AHB_DCAM0_IF_EB;
		regmap_update_bits(cam_pw->cam_ahb_gpr,
				   REG_CAM_AHB_AHB_EB,
				   flag,
				   flag);

		sprd_sensor_init_clk();
	} else {
		pr_debug("cam domain, already power on\n");
	}

	return 0;

err_pw_on:
	pr_info("cam domain, failed to power on\n");
	return ret;
}
EXPORT_SYMBOL(sprd_cam_pw_on);

int sprd_cam_domain_eb(void)
{
	return 0;
}
EXPORT_SYMBOL(sprd_cam_domain_eb);

int sprd_cam_domain_disable(void)
{
	return 0;
}
EXPORT_SYMBOL(sprd_cam_domain_disable);
#endif
