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
#include <linux/sched.h>
#include "cam_pw_domain.h"


#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "[cam sys]: %d: %s line=%d: " fmt, \
	current->pid, __func__, __LINE__


#define CAM_TIMEOUT              (3 * 1000)

enum {
	SPRD_PW_DOMAIN_OFF = 0,
	SPRD_PW_DOMAIN_ON,
};

struct cam_pw_domain_info {
	struct regmap *cam_ahb_gpr;

	struct clk *cam_mtx_eb;
	struct clk *cam_ahb_clk;
	struct clk *cam_ahb_clk_default;
	struct clk *cam_ahb_clk_parent;
};

struct client_info {
	unsigned int pw_state;
};

struct pw_domain_info {
	struct client_info pw_dispc_info;
	struct client_info pw_dsi_info;
	struct client_info pw_dcam_info;
	struct client_info pw_isp_info;
	struct client_info pw_cpp_info;
	struct client_info pw_sensor_info;
	struct client_info pw_jpg_info;
	struct mutex client_lock;
	struct regmap *aon_apb_gpr;
	struct regmap *dispc_ahb_gpr;
	struct regmap *pmu_apb_gpr;
	struct regmap *cam_ahb_gpr;
	struct regmap *com_pmu_apb_gpr;
	unsigned int pw_count;
};

static struct cam_pw_domain_info *cam_pw;
static struct pw_domain_info *pw_domain;

int sprd_cam_pw_domain_init(struct platform_device *pdev)
{
	struct regmap *syscon_gpr;

	cam_pw = devm_kzalloc(&pdev->dev, sizeof(*cam_pw), GFP_KERNEL);
	if (!cam_pw)
		return -ENOMEM;

	cam_pw->cam_mtx_eb = devm_clk_get(&pdev->dev, "cam_mtx_eb");
	if (IS_ERR(cam_pw->cam_mtx_eb))
		return PTR_ERR(cam_pw->cam_mtx_eb);

	cam_pw->cam_ahb_clk = devm_clk_get(&pdev->dev, "clk_ahb_cam");
	if (IS_ERR(cam_pw->cam_ahb_clk))
		return PTR_ERR(cam_pw->cam_ahb_clk);

	cam_pw->cam_ahb_clk_parent =
		devm_clk_get(&pdev->dev, "clk_ahb_cam_parent");
	if (IS_ERR(cam_pw->cam_ahb_clk_parent))
		return PTR_ERR(cam_pw->cam_ahb_clk_parent);

	syscon_gpr = syscon_regmap_lookup_by_compatible("sprd,iwhale2-lpc-ahb");
	if (IS_ERR(syscon_gpr))
		return PTR_ERR(syscon_gpr);
	cam_pw->cam_ahb_gpr = syscon_gpr;

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

void sprd_isp_pw_on(void)
{
	pr_info("cam core power on\n");

	regmap_update_bits(pw_domain->pmu_apb_gpr,
			   REG_PMU_APB_PD_CAM_CORE_PWR_CFG,
			   BIT_PMU_APB_PD_CAM_CORE_AUTO_SHUTDOWN_EN,
			   ~(unsigned int)
			   BIT_PMU_APB_PD_CAM_CORE_AUTO_SHUTDOWN_EN);
	regmap_update_bits(pw_domain->pmu_apb_gpr,
			   REG_PMU_APB_PD_CAM_CORE_PWR_CFG,
			   BIT_PMU_APB_PD_CAM_CORE_FORCE_SHUTDOWN,
			   ~(unsigned int)
			   BIT_PMU_APB_PD_CAM_CORE_FORCE_SHUTDOWN);
}
EXPORT_SYMBOL(sprd_isp_pw_on);

void sprd_isp_pw_off(void)
{
	pr_info("cam core power off\n");

	regmap_update_bits(pw_domain->pmu_apb_gpr,
			   REG_PMU_APB_PD_CAM_CORE_PWR_CFG,
			   BIT_PMU_APB_PD_CAM_CORE_AUTO_SHUTDOWN_EN,
			   ~(unsigned int)
			   BIT_PMU_APB_PD_CAM_CORE_AUTO_SHUTDOWN_EN);
	regmap_update_bits(pw_domain->pmu_apb_gpr,
			   REG_PMU_APB_PD_CAM_CORE_PWR_CFG,
			   BIT_PMU_APB_PD_CAM_CORE_FORCE_SHUTDOWN,
			   BIT_PMU_APB_PD_CAM_CORE_FORCE_SHUTDOWN);
}
EXPORT_SYMBOL(sprd_isp_pw_off);

void sprd_mm_pw_on(unsigned int client)
{
	int ret;
	unsigned int power_state1, power_state2, power_state3;
	unsigned long timeout = jiffies + msecs_to_jiffies(CAM_TIMEOUT);
	unsigned int read_count = 0;
	unsigned int val;
	unsigned int ahb_rst_bit;
	unsigned int flag;
	struct device_node *np;
	struct clk *cam_mtx_eb;
	struct clk *cam_ahb_clk;
	struct clk *cam_ahb_clk_parent;
	struct clk *cam_ahb_clk_default;

	mutex_lock(&pw_domain->client_lock);

	if (pw_domain->pw_count == 0) {
		pr_info("cam sys power on\n");

		regmap_update_bits(pw_domain->pmu_apb_gpr,
				   REG_PMU_APB_PD_CAM_SYS_PWR_CFG,
				   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN,
				   ~(unsigned int)
				   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN);
		regmap_update_bits(pw_domain->pmu_apb_gpr,
				   REG_PMU_APB_PD_CAM_SYS_PWR_CFG,
				   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN,
				   ~(unsigned int)
				   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN);

		do {
			cpu_relax();
			usleep_range(300, 350);
			read_count++;

			ret = regmap_read(pw_domain->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret)
				break;
			power_state1 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x1f);

			ret = regmap_read(pw_domain->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret)
				break;
			power_state2 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x1f);

			ret = regmap_read(pw_domain->pmu_apb_gpr,
					  REG_PMU_APB_PWR_STATE_DBG4, &val);
			if (ret)
				break;
			power_state3 = val & BIT_PMU_APB_PD_CAM_SYS_STATE(0x1f);

			WARN_ON(time_after(jiffies, timeout));
		} while ((power_state1 && read_count < 10) ||
			 (power_state1 != power_state2) ||
			 (power_state2 != power_state3));

		if (ret || power_state1)
			pr_err("cam sys failed to power on\n");

		regmap_update_bits(pw_domain->aon_apb_gpr,
				   REG_AON_APB_APB_EB1,
				   BIT_AON_APB_AON_CAM_EB,
				   BIT_AON_APB_AON_CAM_EB);
		regmap_update_bits(pw_domain->dispc_ahb_gpr,
				   REG_DISP_AHB_AHB_EB,
				   BIT_DISP_AHB_CKG_EB,
				   BIT_DISP_AHB_CKG_EB);
		regmap_update_bits(pw_domain->dispc_ahb_gpr,
				   REG_DISP_AHB_AHB_EB,
				   BIT_DISP_AHB_SYS_MTX_EB,
				   BIT_DISP_AHB_SYS_MTX_EB);

		np = of_find_compatible_node(NULL, NULL, "sprd,dcam-r2p0");
		cam_mtx_eb = of_clk_get_by_name(np, "cam_mtx_eb");
		cam_ahb_clk = of_clk_get_by_name(np, "clk_ahb_cam");
		cam_ahb_clk_parent =
			of_clk_get_by_name(np, "clk_ahb_cam_parent");
		cam_ahb_clk_default = clk_get_parent(cam_ahb_clk);

		clk_prepare_enable(cam_mtx_eb);
		clk_set_parent(cam_ahb_clk, cam_ahb_clk_parent);
		clk_prepare_enable(cam_ahb_clk);

		regmap_update_bits(pw_domain->cam_ahb_gpr,
				   REG_CAM_AHB_CAM_AXI_LPC2,
				   BIT_CAM_AHB_LP_EB_CAM_PUB0,
				   ~(unsigned int)
				   BIT_CAM_AHB_LP_EB_CAM_PUB0);

		regmap_update_bits(pw_domain->cam_ahb_gpr,
				   REG_CAM_AHB_CAM_AXI_LPC4,
				   0xffff,
				   0xfff0);

		clk_set_parent(cam_ahb_clk, cam_ahb_clk_default);
		clk_disable_unprepare(cam_ahb_clk);
		clk_disable_unprepare(cam_mtx_eb);

		pr_info("set lp_eb_cam_pub0 to 0\n");
	} else {
		pr_debug("cam sys already power on\n");
	}

	/* camera */
	switch (client) {
	case SPRD_PW_DOMAIN_DCAM:
	case SPRD_PW_DOMAIN_ISP:
	case SPRD_PW_DOMAIN_CPP:
	case SPRD_PW_DOMAIN_SENSOR:
		if (pw_domain->pw_dcam_info.pw_state == SPRD_PW_DOMAIN_OFF &&
		    pw_domain->pw_isp_info.pw_state == SPRD_PW_DOMAIN_OFF &&
		    pw_domain->pw_cpp_info.pw_state == SPRD_PW_DOMAIN_OFF &&
		    pw_domain->pw_sensor_info.pw_state == SPRD_PW_DOMAIN_OFF) {
			pr_info("enable camera ahb clk ...\n");

			if (!cam_pw) {
				pr_info("cam_pw is not inited\n");
				break;
			}

			/* cam mtx enable */
			clk_prepare_enable(cam_pw->cam_mtx_eb);

			/* config cam ahb clk */
			clk_set_parent(cam_pw->cam_ahb_clk,
				       cam_pw->cam_ahb_clk_parent);
			clk_prepare_enable(cam_pw->cam_ahb_clk);

			ahb_rst_bit = BIT_CAM_AHB_DCAM0_SOFT_RST |
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
				BIT_CAM_AHB_ISP0_LOG_SOFT_RST |
				BIT_CAM_AHB_ISP1_LOG_SOFT_RST |
				BIT_CAM_AHB_ISP2_LOG_SOFT_RST |
				BIT_CAM_AHB_CSI0_SOFT_RST |
				BIT_CAM_AHB_CSI1_SOFT_RST |
				BIT_CAM_AHB_CKG_SOFT_RST |
				BIT_CAM_AHB_MMU_SOFT_RST |
				BIT_CAM_AHB_MMU_PF_SOFT_RST |
				BIT_CAM_AHB_CPP_SOFT_RST |
				BIT_CAM_AHB_DCAM2ISP_IF_SOFT_RST |
				BIT_CAM_AHB_ISP2DCAM_IF_SOFT_RST;
			regmap_update_bits(cam_pw->cam_ahb_gpr,
					   REG_CAM_AHB_AHB_RST,
					   ahb_rst_bit,
					   ahb_rst_bit);
			udelay(1);
			regmap_update_bits(cam_pw->cam_ahb_gpr,
					   REG_CAM_AHB_AHB_RST,
					   ahb_rst_bit,
					   ~ahb_rst_bit);

			regmap_update_bits(cam_pw->cam_ahb_gpr,
					   REG_CAM_AHB_MODULE_SOFT_RST,
					   0x3ff,
					   0x3ff);
			udelay(1);
			regmap_update_bits(cam_pw->cam_ahb_gpr,
					   REG_CAM_AHB_MODULE_SOFT_RST,
					   0x3ff,
					   ~(0x3ff));

			flag = BIT_CAM_AHB_MMU_EB |
				BIT_CAM_AHB_MMU_PF_EB |
				BIT_CAM_AHB_DCAM0_IF_EB;
			regmap_update_bits(cam_pw->cam_ahb_gpr,
					   REG_CAM_AHB_AHB_EB,
					   flag,
					   flag);
		} else {
			pr_debug("already enable camera ahb clk ...\n");
		}
		break;
	default:
		break;
	}

	switch (client) {
	case SPRD_PW_DOMAIN_DISPC:
		if (pw_domain->pw_dispc_info.pw_state == SPRD_PW_DOMAIN_OFF) {
			pw_domain->pw_dispc_info.pw_state = SPRD_PW_DOMAIN_ON;
			pw_domain->pw_count++;
			pr_info("dispc power on\n");
		}
		break;

	case SPRD_PW_DOMAIN_DSI:
		if (pw_domain->pw_dsi_info.pw_state == SPRD_PW_DOMAIN_OFF) {
			pw_domain->pw_dsi_info.pw_state = SPRD_PW_DOMAIN_ON;
			pw_domain->pw_count++;
			pr_info("dispc power on\n");
		}
		break;

	case SPRD_PW_DOMAIN_DCAM:
		if (pw_domain->pw_dcam_info.pw_state == SPRD_PW_DOMAIN_OFF) {
			pw_domain->pw_dcam_info.pw_state = SPRD_PW_DOMAIN_ON;
			pw_domain->pw_count++;
			pr_info("dcam power on\n");
		}
		break;

	case SPRD_PW_DOMAIN_ISP:
		if (pw_domain->pw_isp_info.pw_state == SPRD_PW_DOMAIN_OFF) {
			pw_domain->pw_isp_info.pw_state = SPRD_PW_DOMAIN_ON;
			pw_domain->pw_count++;
			pr_info("isp power on\n");
		}
		break;

	case SPRD_PW_DOMAIN_CPP:
		if (pw_domain->pw_cpp_info.pw_state == SPRD_PW_DOMAIN_OFF) {
			pw_domain->pw_cpp_info.pw_state = SPRD_PW_DOMAIN_ON;
			pw_domain->pw_count++;
			pr_info("cpp power on\n");
		}
		break;

	case SPRD_PW_DOMAIN_SENSOR:
		if (pw_domain->pw_sensor_info.pw_state == SPRD_PW_DOMAIN_OFF) {
			pw_domain->pw_sensor_info.pw_state = SPRD_PW_DOMAIN_ON;
			pw_domain->pw_count++;
			pr_info("sensor power on\n");
		}
		break;

	case SPRD_PW_DOMAIN_JPG:
		if (pw_domain->pw_jpg_info.pw_state == SPRD_PW_DOMAIN_OFF) {
			pw_domain->pw_jpg_info.pw_state = SPRD_PW_DOMAIN_ON;
			pw_domain->pw_count++;
			pr_info("jpg power on\n");
		}
		break;

	default:
		break;
	}

	mutex_unlock(&pw_domain->client_lock);
}
EXPORT_SYMBOL(sprd_mm_pw_on);

void sprd_mm_pw_off(unsigned int client)
{
	unsigned int val;
	unsigned int need_close_cam = 0;
	struct device_node *np;
	struct clk *cam_mtx_eb;
	struct clk *cam_ahb_clk;
	struct clk *cam_ahb_clk_parent;
	struct clk *cam_ahb_clk_default;

	mutex_lock(&pw_domain->client_lock);

	switch (client) {
	case SPRD_PW_DOMAIN_DCAM:
		if (pw_domain->pw_dcam_info.pw_state == SPRD_PW_DOMAIN_ON &&
		    pw_domain->pw_isp_info.pw_state == SPRD_PW_DOMAIN_OFF &&
		    pw_domain->pw_cpp_info.pw_state == SPRD_PW_DOMAIN_OFF &&
		    pw_domain->pw_sensor_info.pw_state == SPRD_PW_DOMAIN_OFF) {
			need_close_cam = 1;
		}
		break;
	case SPRD_PW_DOMAIN_ISP:
		if (pw_domain->pw_dcam_info.pw_state == SPRD_PW_DOMAIN_OFF &&
		    pw_domain->pw_isp_info.pw_state == SPRD_PW_DOMAIN_ON &&
		    pw_domain->pw_cpp_info.pw_state == SPRD_PW_DOMAIN_OFF &&
		    pw_domain->pw_sensor_info.pw_state == SPRD_PW_DOMAIN_OFF) {
			need_close_cam = 1;
		}
		break;
	case SPRD_PW_DOMAIN_CPP:
		if (pw_domain->pw_dcam_info.pw_state == SPRD_PW_DOMAIN_OFF &&
		    pw_domain->pw_isp_info.pw_state == SPRD_PW_DOMAIN_OFF &&
		    pw_domain->pw_cpp_info.pw_state == SPRD_PW_DOMAIN_ON &&
		    pw_domain->pw_sensor_info.pw_state == SPRD_PW_DOMAIN_OFF) {
			need_close_cam = 1;
		}
		break;
	case SPRD_PW_DOMAIN_SENSOR:
		if (pw_domain->pw_dcam_info.pw_state == SPRD_PW_DOMAIN_OFF &&
		    pw_domain->pw_isp_info.pw_state == SPRD_PW_DOMAIN_OFF &&
		    pw_domain->pw_cpp_info.pw_state == SPRD_PW_DOMAIN_OFF &&
		    pw_domain->pw_sensor_info.pw_state == SPRD_PW_DOMAIN_ON) {
			need_close_cam = 1;
		}
		break;
	default:
		break;
	}

	if (need_close_cam) {
		pr_info("disable camera ahb clk ...\n");
		if (cam_pw) {
			clk_set_parent(cam_pw->cam_ahb_clk,
				       cam_pw->cam_ahb_clk_default);
			clk_disable_unprepare(cam_pw->cam_ahb_clk);
			clk_disable_unprepare(cam_pw->cam_mtx_eb);
		}
	}

	if (pw_domain->pw_count == 0) {
		pr_debug("cam sys already power off\n");
		mutex_unlock(&pw_domain->client_lock);
		return;
	}

	switch (client) {
	case SPRD_PW_DOMAIN_DISPC:
		if (pw_domain->pw_dispc_info.pw_state == SPRD_PW_DOMAIN_ON) {
			pw_domain->pw_dispc_info.pw_state = SPRD_PW_DOMAIN_OFF;
			pw_domain->pw_count--;
			pr_info("dispc power off\n");
		}
		break;

	case SPRD_PW_DOMAIN_DSI:
		if (pw_domain->pw_dsi_info.pw_state == SPRD_PW_DOMAIN_ON) {
			pw_domain->pw_dsi_info.pw_state = SPRD_PW_DOMAIN_OFF;
			pw_domain->pw_count--;
			pr_info("dispc power off\n");
		}
		break;

	case SPRD_PW_DOMAIN_DCAM:
		if (pw_domain->pw_dcam_info.pw_state == SPRD_PW_DOMAIN_ON) {
			pw_domain->pw_dcam_info.pw_state = SPRD_PW_DOMAIN_OFF;
			pw_domain->pw_count--;
			pr_info("dcam power off\n");
		}
		break;

	case SPRD_PW_DOMAIN_ISP:
		if (pw_domain->pw_isp_info.pw_state == SPRD_PW_DOMAIN_ON) {
			pw_domain->pw_isp_info.pw_state = SPRD_PW_DOMAIN_OFF;
			pw_domain->pw_count--;
			pr_info("isp power off\n");
		}
		break;

	case SPRD_PW_DOMAIN_CPP:
		if (pw_domain->pw_cpp_info.pw_state == SPRD_PW_DOMAIN_ON) {
			pw_domain->pw_cpp_info.pw_state = SPRD_PW_DOMAIN_OFF;
			pw_domain->pw_count--;
			pr_info("cpp power off\n");
		}
		break;

	case SPRD_PW_DOMAIN_SENSOR:
		if (pw_domain->pw_sensor_info.pw_state == SPRD_PW_DOMAIN_ON) {
			pw_domain->pw_sensor_info.pw_state = SPRD_PW_DOMAIN_OFF;
			pw_domain->pw_count--;
			pr_info("sensor power off\n");
		}
		break;

	case SPRD_PW_DOMAIN_JPG:
		if (pw_domain->pw_jpg_info.pw_state == SPRD_PW_DOMAIN_ON) {
			pw_domain->pw_jpg_info.pw_state = SPRD_PW_DOMAIN_OFF;
			pw_domain->pw_count--;
			pr_info("jpg power off\n");
		}
		break;

	default:
		break;
	}

	if (pw_domain->pw_count == 0) {
		pr_info("cam sys power off\n");

		np = of_find_compatible_node(NULL, NULL, "sprd,dcam-r2p0");
		cam_mtx_eb = of_clk_get_by_name(np, "cam_mtx_eb");
		cam_ahb_clk = of_clk_get_by_name(np, "clk_ahb_cam");
		cam_ahb_clk_parent =
			of_clk_get_by_name(np, "clk_ahb_cam_parent");
		cam_ahb_clk_default = clk_get_parent(cam_ahb_clk);

		clk_prepare_enable(cam_mtx_eb);
		clk_set_parent(cam_ahb_clk, cam_ahb_clk_parent);
		clk_prepare_enable(cam_ahb_clk);

		regmap_update_bits(pw_domain->cam_ahb_gpr,
				   REG_CAM_AHB_CAM_AXI_LPC2,
				   BIT_CAM_AHB_LP_EB_CAM_PUB0,
				   BIT_CAM_AHB_LP_EB_CAM_PUB0);

		clk_set_parent(cam_ahb_clk, cam_ahb_clk_default);
		clk_disable_unprepare(cam_ahb_clk);
		clk_disable_unprepare(cam_mtx_eb);

		pr_info("set lp_eb_cam_pub0 to 1\n");

		while (1) {
			regmap_read(pw_domain->cam_ahb_gpr,
				    REG_CAM_AHB_CAM_AB0_DBG, &val);
			val &= (0x7 << 24);
			if (val == 0)
				break;

			usleep_range(50, 100);
		};

		regmap_update_bits(pw_domain->dispc_ahb_gpr,
				   REG_DISP_AHB_AHB_EB,
				   BIT_DISP_AHB_CKG_EB,
				   ~(unsigned int)BIT_DISP_AHB_CKG_EB);
		regmap_update_bits(pw_domain->dispc_ahb_gpr,
				   REG_DISP_AHB_AHB_EB,
				   BIT_DISP_AHB_SYS_MTX_EB,
				   ~(unsigned int)BIT_DISP_AHB_SYS_MTX_EB);
		regmap_update_bits(pw_domain->aon_apb_gpr,
				   REG_AON_APB_APB_EB1,
				   BIT_AON_APB_AON_CAM_EB,
				   ~(unsigned int)BIT_AON_APB_AON_CAM_EB);

		usleep_range(500, 550);

		while (1) {
			regmap_read(pw_domain->com_pmu_apb_gpr,
				    REG_COM_PMU_APB_FENCING0_CTRL_STATE, &val);
			val &= BIT_COM_PMU_APB_CAM_SYS_FENCING0_CTRL_STATE(0xf);
			if (val == 0x300 || val == 0x400)
				break;

			usleep_range(50, 100);
		}

		regmap_update_bits(pw_domain->pmu_apb_gpr,
				   REG_PMU_APB_PD_CAM_SYS_PWR_CFG,
				   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN,
				   ~(unsigned int)
				   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN);
		regmap_update_bits(pw_domain->pmu_apb_gpr,
				   REG_PMU_APB_PD_CAM_SYS_PWR_CFG,
				   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN,
				   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN);
	}

	mutex_unlock(&pw_domain->client_lock);
}
EXPORT_SYMBOL(sprd_mm_pw_off);

/* workarount for power domain, r2p0, only can be called by sensor driver */

int sprd_cam_pw_on(void)
{
	sprd_mm_pw_on(SPRD_PW_DOMAIN_SENSOR);
	return 0;
}
EXPORT_SYMBOL(sprd_cam_pw_on);

int sprd_cam_pw_off(void)
{
	sprd_mm_pw_off(SPRD_PW_DOMAIN_SENSOR);
	return 0;
}
EXPORT_SYMBOL(sprd_cam_pw_off);

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

static int __init sprd_mm_pw_domain_init(void)
{
	struct regmap *syscon_gpr;

	pr_info("mm power domain init\n");

	pw_domain = kzalloc(sizeof(*pw_domain), GFP_KERNEL);

	syscon_gpr = syscon_regmap_lookup_by_compatible("sprd,iwhale2-aon-apb");
	if (IS_ERR(syscon_gpr))
		return PTR_ERR(syscon_gpr);
	pw_domain->aon_apb_gpr = syscon_gpr;

	syscon_gpr =
		syscon_regmap_lookup_by_compatible("sprd,iwhale2-dispc-ahb");
	if (IS_ERR(syscon_gpr))
		return PTR_ERR(pw_domain->dispc_ahb_gpr);
	pw_domain->dispc_ahb_gpr = syscon_gpr;

	syscon_gpr =
		syscon_regmap_lookup_by_compatible("sprd,iwhale2-aon-pwu-apb");
	if (IS_ERR(syscon_gpr))
		return PTR_ERR(syscon_gpr);
	pw_domain->pmu_apb_gpr = syscon_gpr;

	syscon_gpr = syscon_regmap_lookup_by_compatible("sprd,iwhale2-lpc-ahb");
	if (IS_ERR(syscon_gpr))
		return PTR_ERR(syscon_gpr);
	pw_domain->cam_ahb_gpr = syscon_gpr;

	syscon_gpr = syscon_regmap_lookup_by_compatible("sprd,iwhale2-aon-com-pmu-apb");
	if (IS_ERR(syscon_gpr))
		return PTR_ERR(syscon_gpr);
	pw_domain->com_pmu_apb_gpr = syscon_gpr;

	mutex_init(&pw_domain->client_lock);

	return 0;
}
fs_initcall(sprd_mm_pw_domain_init);

