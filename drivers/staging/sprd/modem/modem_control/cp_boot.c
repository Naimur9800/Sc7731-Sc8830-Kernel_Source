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

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/sipc.h>

#include "../include/sprd_modem_control.h"

static char *com_pmu_apb_name = "sprd,syscon-com-pmu-apb";
static char *com_apb_name = "sprd,syscon-com-apb";
static char *pwu_apb_name = "sprd,syscon-pwu-apb";

static int sprd_cproc_native_cm4_start(void *arg)
{
	struct cproc_device *cproc;
	struct platform_device *pdev;
	struct device_node *np;
	struct regmap *com_pwu_apb, *comm_apb;

	pr_info(CPROC_TAG "enter the %s\n", __func__);

	cproc = (struct cproc_device *)arg;
	pdev = cproc->pdev;
	np = pdev->dev.of_node;

	com_pwu_apb = syscon_regmap_lookup_by_phandle(np, com_pmu_apb_name);
	if (IS_ERR(com_pwu_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, com_pmu_apb_name);
		return -EAGAIN;
	}

	comm_apb = syscon_regmap_lookup_by_phandle(np, com_apb_name);
	if (IS_ERR(comm_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, com_apb_name);
		return -EAGAIN;
	}

	/* clear M4 software interrupt */
	regmap_update_bits(comm_apb, REG_COMMON_APB_SP_CFG_BUS,
			   BIT_COMMON_APB_INT_REQ_CM4_SOFT,
			   (unsigned)~BIT_COMMON_APB_INT_REQ_CM4_SOFT);
	/* release cm4 */
	regmap_update_bits(com_pwu_apb, REG_COM_PMU_APB_SYS_SOFT_RESET,
			   (BIT_COM_PMU_APB_CM4_CORE_SRST |
			    BIT_COM_PMU_APB_CM4_SYS_SRST),
			   (unsigned)~(BIT_COM_PMU_APB_CM4_CORE_SRST |
				       BIT_COM_PMU_APB_CM4_SYS_SRST));

	pr_info(CPROC_TAG "%s over\n", __func__);

	return 0;
}

static int sprd_cproc_native_cm4_stop(void *arg)
{
	struct cproc_device *cproc;
	struct device_node *np;
	struct regmap *pwu_apb, *com_pwu_apb, *comm_apb;

	pr_info(CPROC_TAG "enter the %s\n", __func__);

	cproc = (struct cproc_device *)arg;
	np = cproc->pdev->dev.of_node;

	pwu_apb = syscon_regmap_lookup_by_phandle(np, pwu_apb_name);
	if (IS_ERR(pwu_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, pwu_apb_name);
		return -EAGAIN;
	}

	com_pwu_apb = syscon_regmap_lookup_by_phandle(np, com_pmu_apb_name);
	if (IS_ERR(com_pwu_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, com_pmu_apb_name);
		return -EAGAIN;
	}

	comm_apb = syscon_regmap_lookup_by_phandle(np, com_apb_name);
	if (IS_ERR(comm_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, com_apb_name);
		return -EAGAIN;
	}

	/* enable ap access to cm4 permissions */
	regmap_update_bits(comm_apb, REG_COMMON_APB_SP_CFG_BUS,
			   BIT_COMMON_APB_SP_CFG_BUS_SLEEP,
			   (unsigned)~BIT_COMMON_APB_SP_CFG_BUS_SLEEP);
	/* hold cm4 */
	regmap_update_bits(com_pwu_apb, REG_COM_PMU_APB_SYS_SOFT_RESET,
			   (BIT_COM_PMU_APB_CM4_CORE_SRST |
			    BIT_COM_PMU_APB_CM4_SYS_SRST),
			   (BIT_COM_PMU_APB_CM4_CORE_SRST |
			    BIT_COM_PMU_APB_CM4_SYS_SRST));

	/* clear cm4 force sleep */
	regmap_update_bits(pwu_apb, REG_PMU_APB_CM4_SYS_SLEEP_CTRL,
			   (BIT_PMU_APB_CM4_SYS_FORCE_DEEP_SLEEP |
			    BIT_PMU_APB_CM4_SYS_FORCE_DOZE_SLEEP),
			   (unsigned)~(BIT_PMU_APB_CM4_SYS_FORCE_DEEP_SLEEP |
				       BIT_PMU_APB_CM4_SYS_FORCE_DOZE_SLEEP));

	pr_info(CPROC_TAG "%s over\n", __func__);

	return 0;
}

static int sprd_cproc_native_cr5_start(void *arg)
{
	unsigned val;
	struct cproc_device *cproc;
	struct device_node *np;
	struct regmap *pwu_apb, *com_pwu_apb;

	pr_info(CPROC_TAG "enter the %s\n", __func__);

	cproc = (struct cproc_device *)arg;
	np = cproc->pdev->dev.of_node;

	pwu_apb = syscon_regmap_lookup_by_phandle(np, pwu_apb_name);
	if (IS_ERR(pwu_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, pwu_apb_name);
		return -EAGAIN;
	}

	com_pwu_apb = syscon_regmap_lookup_by_phandle(np, com_pmu_apb_name);
	if (IS_ERR(com_pwu_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, com_pmu_apb_name);
		return -EAGAIN;
	}

	/* clear r5 sleep */
	regmap_update_bits(pwu_apb, REG_PMU_APB_PUBCP_SYS_SLEEP_CTRL,
			   BIT_PMU_APB_PUBCP_SYS_FORCE_DEEP_SLEEP,
			   (unsigned)~BIT_PMU_APB_PUBCP_SYS_FORCE_DEEP_SLEEP);
	regmap_update_bits(pwu_apb, REG_PMU_APB_PUBCP_SYS_SLEEP_CTRL,
			   BIT_PMU_APB_PUBCP_SYS_FORCE_LIGHT_SLEEP0,
			   (unsigned)~BIT_PMU_APB_PUBCP_SYS_FORCE_LIGHT_SLEEP0);
	regmap_update_bits(pwu_apb, REG_PMU_APB_PUBCP_SYS_SLEEP_CTRL,
			   BIT_PMU_APB_PUBCP_SYS_FORCE_DOZE_SLEEP,
			   (unsigned)~BIT_PMU_APB_PUBCP_SYS_FORCE_DOZE_SLEEP);
	val = (unsigned)~BIT_PMU_APB_PUBCP_SYS_FORCE_PUB_DEEP_SLEEP0;
	regmap_update_bits(pwu_apb, REG_PMU_APB_PUBCP_SYS_SLEEP_CTRL,
			   BIT_PMU_APB_PUBCP_SYS_FORCE_PUB_DEEP_SLEEP0,
			   val);
	/* power on */
	val = (unsigned)~(BIT_PMU_APB_PD_PUBCP_SYS_AUTO_SHUTDOWN_EN |
			  BIT_PMU_APB_PD_PUBCP_SYS_FORCE_SHUTDOWN);
	regmap_update_bits(pwu_apb, REG_PMU_APB_PD_PUBCP_SYS_PWR_CFG,
			   BIT_PMU_APB_PD_PUBCP_SYS_AUTO_SHUTDOWN_EN |
			   BIT_PMU_APB_PD_PUBCP_SYS_FORCE_SHUTDOWN,
			   val);
	/* release cr5 */
	msleep(50);
	regmap_update_bits(com_pwu_apb, REG_COM_PMU_APB_SYS_SOFT_RESET,
			   BIT_COM_PMU_APB_PUBCP_SYS_SRST,
			   (unsigned)~BIT_COM_PMU_APB_PUBCP_SYS_SRST);
	msleep(50);
	regmap_update_bits(com_pwu_apb, REG_COM_PMU_APB_SYS_SOFT_RESET,
			   BIT_COM_PMU_APB_PUBCP_CR5_CORE_SRST,
			   (unsigned)~BIT_COM_PMU_APB_PUBCP_CR5_CORE_SRST);

	pr_info(CPROC_TAG "%s over\n", __func__);

	return 0;
}

static int sprd_cproc_native_cr5_stop(void *arg)
{
	unsigned val;
	struct cproc_device *cproc;
	struct device_node *np;
	struct regmap *pwu_apb, *com_pwu_apb;

	pr_info(CPROC_TAG "enter the %s\n", __func__);

	cproc = (struct cproc_device *)arg;
	np = cproc->pdev->dev.of_node;

	pwu_apb = syscon_regmap_lookup_by_phandle(np, pwu_apb_name);
	if (IS_ERR(pwu_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, pwu_apb_name);
		return -EAGAIN;
	}

	com_pwu_apb = syscon_regmap_lookup_by_phandle(np, com_pmu_apb_name);
	if (IS_ERR(com_pwu_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, com_pmu_apb_name);
		return -EAGAIN;
	}

	/* power down */
	val = (unsigned)~BIT_PMU_APB_PD_PUBCP_SYS_AUTO_SHUTDOWN_EN;
	regmap_update_bits(pwu_apb, REG_PMU_APB_PD_PUBCP_SYS_PWR_CFG,
			   BIT_PMU_APB_PD_PUBCP_SYS_AUTO_SHUTDOWN_EN,
			   val);
	regmap_update_bits(pwu_apb, REG_PMU_APB_PD_PUBCP_SYS_PWR_CFG,
			   BIT_PMU_APB_PD_PUBCP_SYS_FORCE_SHUTDOWN,
			   BIT_PMU_APB_PD_PUBCP_SYS_FORCE_SHUTDOWN);

	/* hold cr5 */
	regmap_update_bits(com_pwu_apb, REG_COM_PMU_APB_SYS_SOFT_RESET,
			   BIT_COM_PMU_APB_PUBCP_SYS_SRST |
			   BIT_COM_PMU_APB_PUBCP_CR5_CORE_SRST,
			   BIT_COM_PMU_APB_PUBCP_SYS_SRST |
			   BIT_COM_PMU_APB_PUBCP_CR5_CORE_SRST);

	pr_info(CPROC_TAG "%s over\n", __func__);

	return 0;
}

#ifdef SPRD_MODEM_CTRL_AGDSP
static int sprd_cproc_native_agdsp_start(void *arg)
{
	struct cproc_device *cproc;
	struct device_node *np;
	struct regmap *pwu_apb, *com_pwu_apb, *comm_apb;

	pr_info(CPROC_TAG "enter the %s\n", __func__);

	cproc = (struct cproc_device *)arg;
	np = cproc->pdev->dev.of_node;

	pwu_apb = syscon_regmap_lookup_by_phandle(np, pwu_apb_name);
	if (IS_ERR(pwu_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, pwu_apb_name);
		return -EAGAIN;
	}

	com_pwu_apb = syscon_regmap_lookup_by_phandle(np, com_pmu_apb_name);
	if (IS_ERR(com_pwu_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, com_pmu_apb_name);
		return -EAGAIN;
	}

	comm_apb = syscon_regmap_lookup_by_phandle(np, com_apb_name);
	if (IS_ERR(comm_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, com_apb_name);
		return -EAGAIN;
	}

	/* power on */
	regmap_update_bits(pwu_apb, REG_PMU_APB_PD_AGCP_SYS_PWR_CFG,
			   BIT_PMU_APB_PD_AGCP_SYS_FORCE_SHUTDOWN,
			   (unsigned)~BIT_PMU_APB_PD_AGCP_SYS_FORCE_SHUTDOWN);
	regmap_update_bits(pwu_apb, REG_PMU_APB_PD_AGCP_SYS_PWR_CFG,
			   BIT_PMU_APB_PD_AGCP_DSP_FORCE_SHUTDOWN,
			   (unsigned)~BIT_PMU_APB_PD_AGCP_DSP_FORCE_SHUTDOWN);
	/* clear agdsp sleep */
	regmap_update_bits(pwu_apb, REG_PMU_APB_AGCP_SYS_SLEEP_CTRL,
			   BIT_PMU_APB_AGCP_SYS_FORCE_LIGHT_SLEEP0,
			   (unsigned)~BIT_PMU_APB_AGCP_SYS_FORCE_LIGHT_SLEEP0);
	regmap_update_bits(pwu_apb, REG_PMU_APB_AGCP_SYS_SLEEP_CTRL,
			   BIT_PMU_APB_AGCP_SYS_FORCE_DEEP_SLEEP,
			   (unsigned)~BIT_PMU_APB_AGCP_SYS_FORCE_DEEP_SLEEP);
	regmap_update_bits(pwu_apb, REG_PMU_APB_AGCP_SYS_SLEEP_CTRL,
			   BIT_PMU_APB_AGCP_SYS_FORCE_DOZE_SLEEP,
			   (unsigned)~BIT_PMU_APB_AGCP_SYS_FORCE_DOZE_SLEEP);
	/* config ctrl regs */
	regmap_update_bits(comm_apb, REG_AON_APB_AGCP_BOOT_PROT,
			   0x9620,
			   0x9620);
	regmap_update_bits(comm_apb, REG_AON_APB_AGCP_DSP_CTRL0,
			   0xFFFFFFFF,
			   0x60050040);
	regmap_update_bits(comm_apb, REG_AON_APB_AGCP_DSP_CTRL1,
			   BIT_AON_APB_AGCP_DSP_BOOT,
			   BIT_AON_APB_AGCP_DSP_BOOT);
	regmap_update_bits(comm_apb, REG_AON_APB_AGCP_CTRL,
			   BIT_AON_APB_AGCP_TOP_ACCESS_EN,
			   BIT_AON_APB_AGCP_TOP_ACCESS_EN);
	/* relase agdsp */
	regmap_update_bits(com_pwu_apb, REG_COM_PMU_APB_SYS_SOFT_RESET,
			   BIT_COM_PMU_APB_AGCP_DSP_SRST |
			   BIT_COM_PMU_APB_AGCP_DSP_SYS_SRST |
			   BIT_COM_PMU_APB_AGCP_SYS_SRST,
			   (unsigned)~(BIT_COM_PMU_APB_AGCP_DSP_SRST |
				       BIT_COM_PMU_APB_AGCP_DSP_SYS_SRST |
				       BIT_COM_PMU_APB_AGCP_SYS_SRST));

	pr_info(CPROC_TAG "%s over\n", __func__);

	return 0;
}

static int sprd_cproc_native_agdsp_stop(void *arg)
{
	struct cproc_device *cproc;
	struct device_node *np;
	struct regmap *pwu_apb, *com_pwu_apb;

	pr_info(CPROC_TAG "enter the %s\n", __func__);

	cproc = (struct cproc_device *)arg;
	np = cproc->pdev->dev.of_node;

	pwu_apb = syscon_regmap_lookup_by_phandle(np, pwu_apb_name);
	if (IS_ERR(pwu_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, pwu_apb_name);
		return -EAGAIN;
	}

	com_pwu_apb = syscon_regmap_lookup_by_phandle(np, com_pmu_apb_name);
	if (IS_ERR(com_pwu_apb)) {
		pr_err(CPROC_TAG "%s: failed to find %s\n",
		       __func__, com_pmu_apb_name);
		return -EAGAIN;
	}

	/* power down */
	regmap_update_bits(pwu_apb, REG_PMU_APB_PD_AGCP_SYS_PWR_CFG,
			   BIT_PMU_APB_PD_AGCP_SYS_AUTO_SHUTDOWN_EN,
			   (unsigned)~BIT_PMU_APB_PD_AGCP_SYS_AUTO_SHUTDOWN_EN);
	regmap_update_bits(pwu_apb, REG_PMU_APB_PD_AGCP_SYS_PWR_CFG,
			   BIT_PMU_APB_PD_AGCP_SYS_FORCE_SHUTDOWN,
			   BIT_PMU_APB_PD_AGCP_SYS_FORCE_SHUTDOWN);
	regmap_update_bits(pwu_apb, REG_PMU_APB_PD_AGCP_SYS_PWR_CFG,
			   BIT_PMU_APB_PD_AGCP_DSP_AUTO_SHUTDOWN_EN,
			   (unsigned)~BIT_PMU_APB_PD_AGCP_DSP_AUTO_SHUTDOWN_EN);
	regmap_update_bits(pwu_apb, REG_PMU_APB_PD_AGCP_SYS_PWR_CFG,
			   BIT_PMU_APB_PD_AGCP_DSP_FORCE_SHUTDOWN,
			   BIT_PMU_APB_PD_AGCP_DSP_FORCE_SHUTDOWN);
	/* hold agdsp */
	regmap_update_bits(com_pwu_apb, REG_COM_PMU_APB_SYS_SOFT_RESET,
			   BIT_COM_PMU_APB_AGCP_DSP_SRST |
			   BIT_COM_PMU_APB_AGCP_DSP_SYS_SRST,
			   BIT_COM_PMU_APB_AGCP_DSP_SRST |
			   BIT_COM_PMU_APB_AGCP_DSP_SYS_SRST);

	pr_info(CPROC_TAG "%s over\n", __func__);

	return 0;
}
#endif

int register_cproc_controller(struct cproc_controller *controller)
{
	if (strstr(controller->devname, "pm")) {
		controller->start = sprd_cproc_native_cm4_start;
		controller->stop = sprd_cproc_native_cm4_stop;
	} else if (strstr(controller->devname, "cp")) {
		controller->start = sprd_cproc_native_cr5_start;
		controller->stop = sprd_cproc_native_cr5_stop;
	}
#ifdef SPRD_MODEM_CTRL_AGDSP
	else if (strstr(controller->devname, "ag")) {
		controller->start = sprd_cproc_native_agdsp_start;
		controller->stop = sprd_cproc_native_agdsp_stop;
	}
#endif
	return 0;
}
EXPORT_SYMBOL(register_cproc_controller);

