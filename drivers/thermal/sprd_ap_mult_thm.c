/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sprd_otp.h>
#include "sprd_thm.h"

#define SENSOR_ADDR_OFF  0x100
#define TEMP_LOW         (-40000)
#define TEMP_HIGH        (120000)
#define RAW_ADC_LOW         (550)
#define RAW_ADC_HIGH        (1200)
#define THM_SENOR_NUM 8

#define RAW_DATA_LOW         (623)
#define RAW_DATA_HIGH        (1030)

#define WAHLE_RAT_SIGN_BIT 0
#define WHALE_RATION_OFF_BIT 1
#define WHALE_THM_RATION_BLOCK 14

struct ap_thm_info {
	struct sensor_registers *reg;
	struct sensor_info sensor[];
};

static struct sensor_registers sensor_reg = {
	.thm_ctrl = THM_CTRL,
	.thm_int_ctrl = THM_INT_CTRL,
	.sen_ctrl = SENSOR_CTRL,
	.sen_det_peri = SENSOR_DET_PERI,
	.sen_int_ctrl = SENSOR_INT_CTRL,
	.sen_int_sts = SENSOR_INT_STS,
	.sen_int_raw_sts = SENSOR_INT_RAW_STS,
	.sen_int_clr = SENSOR_INT_CLR,
	.sen_mon_peri = SENSOR_MON_PERI,
	.sen_mon_ctl = SENSOR_MON_CTL,
	.sen_temp0_read = SENSOR_TEMPER0_READ,
	.sen_read_status = SENSOR_READ_STATUS,
	.sen_read_mask = RAW_READ_RANGE_MSK,
};

static struct sensor_registers thm_thm_registers = {
	.thm_ctrl = THM_THM_CTRL,
	.thm_int_ctrl = THM_THM_INT_CTRL,
	.sen_ctrl = THM_SEN_CTRL,
	.sen_det_peri = THM_SEN_DET_PERI,
	.sen_int_ctrl = THM_SEN_INT_CTRL,
	.sen_int_sts = THM_SEN_INT_STS,
	.sen_int_raw_sts = THM_SEN_INT_RAW_STS,
	.sen_int_clr = THM_SEN_INT_CLR,
	.sen_mon_peri = THM_SEN_MON_PERI,
	.sen_mon_ctl = THM_SEN_MON_CTL,
	.sen_temp0_read = THM_SEN_TEMPER0_READ,
	.sen_read_status = THM_SEN_READ_STATUS,
	.sen_read_mask = THM_SEN_RAW_READ_MSK,
};

static DEFINE_MUTEX(thm_list_lock);

static inline void thm_reg_write(unsigned long reg, u32 bits, u32 clear_msk)
{
	writel_relaxed(((readl_relaxed((void __iomem *)reg) & ~clear_msk) |
			bits), ((void __iomem *)reg));
}

static inline u32 thm_reg_read(unsigned long reg)
{
	return readl_relaxed((void __iomem *)reg);
}

static int sprd_rawdata_to_temp_v1(struct sprd_thermal_zone *pzone)
{
	u32 temp_result;

	if (pzone->rawdata < pzone->msen_para->raw_data_low)
		pzone->rawdata = pzone->msen_para->raw_data_low;
	else if (pzone->rawdata > pzone->msen_para->raw_data_high)
		pzone->rawdata = pzone->msen_para->raw_data_high;
	else
		pr_debug("normal\n");
	temp_result = TEMP_LOW +
	    (pzone->rawdata - pzone->msen_para->raw_data_low) * (TEMP_HIGH -
								 TEMP_LOW) /
	    (pzone->msen_para->raw_data_high - pzone->msen_para->raw_data_low);
	return temp_result - pzone->sensor_cal;
}

static int sprd_rawdata_to_temp_v2(struct sprd_thermal_zone *pzone)
{
	int temp_result;

	if (pzone->rawdata < pzone->msen_para->raw_data_low)
		pzone->rawdata = pzone->msen_para->raw_data_low;
	else if (pzone->rawdata > pzone->msen_para->raw_data_high)
		pzone->rawdata = pzone->msen_para->raw_data_high;
	else
		pr_debug("normal\n");
	temp_result =
	    (pzone->sensor_cal * pzone->rawdata + pzone->sensor_cal_offset);

	return temp_result;
}

static int sprd_efuse_whale_arm_cal(struct sprd_thermal_zone *pzone)
{
#if defined(CONFIG_OTP_SPRD_AP_EFUSE)
	u32 data = sprd_ap_efuse_read(pzone->msen_para->cal_efuse_blk);
	u32 ratio_data = sprd_ap_efuse_read(WHALE_THM_RATION_BLOCK);
#else
	u32 data = 0;
#endif
	int rat_sign_bit = 0;
	int ratio = 1;
	int cal_offset;
	int thm_cal = (ratio_data >> WHALE_RATION_OFF_BIT) & 0x001F;
	rat_sign_bit = (ratio_data >> WAHLE_RAT_SIGN_BIT) & 0x1;

	if (rat_sign_bit == 1)
		ratio = 1000 - thm_cal;
	else
		ratio = 1000 + thm_cal;

	cal_offset = (data >> pzone->msen_para->cal_efuse_bit) & 0x001F;
	pzone->sensor_cal = (717 * ratio) / 1000;
	if (cal_offset == 0) {
		cal_offset = 16;
		pzone->sensor_cal_offset = 16000 - 1000 * cal_offset - 42110;
		return -1;
	}
	pzone->sensor_cal_offset = 16000 - 1000 * cal_offset - 42110;

	return 0;
}

static int sprd_efuse_whale_ddr_gpu_cal(struct sprd_thermal_zone *pzone)
{
#if defined(CONFIG_OTP_SPRD_AP_EFUSE)
	u32 data = sprd_ap_efuse_read(pzone->msen_para->cal_efuse_blk);
#else
	u32 data = 0;
#endif
	int cal_offset;

	cal_offset = (data >> pzone->msen_para->cal_efuse_bit) & 0x001F;
	pzone->sensor_cal = 717;
	if (cal_offset == 0) {
		cal_offset = 16;
		pzone->sensor_cal_offset = 16000 - 1000 * cal_offset - 42110;
		return -1;
	}
	pzone->sensor_cal_offset = 16000 - 1000 * cal_offset - 42110;

	return 0;
}

static int sprd_efuse_general_thm_cal_get(struct sprd_thermal_zone *pzone)
{
#if defined(CONFIG_OTP_SPRD_AP_EFUSE)
	u32 data = sprd_ap_efuse_read(pzone->msen_para->cal_efuse_blk);
#else
	u32 data = 0;
#endif
	int thm_cal = (data >> pzone->msen_para->cal_efuse_bit) & 0x001F;

	if (thm_cal == 0) {
		pzone->sensor_cal = 0;
		return -1;
	}
	pzone->sensor_cal = 1000 * thm_cal - 16000;
	return 0;
}

void sprd_pmic_thm_reg_dump(struct sprd_thermal_zone *pzone)
{
	unsigned long base = (unsigned long)(pzone->thm_base);
	unsigned long end = base + 0x64;

	for (; base < end; base += 4) {
		pr_info("thm_base = 0x%lx, value = 0x%x\n", base,
			thm_reg_read(base));
	}
}

static int sprd_gen_sensor_temp_read(struct sprd_thermal_zone *pzone, int *temp)
{
	int sensor_temp;

	pzone->rawdata =
	    thm_reg_read(pzone->sensor_base + pzone->reg->sen_temp0_read);
	pzone->rawdata = pzone->rawdata & pzone->reg->sen_read_mask;

	if (pzone->ready_flag) {
		sensor_temp = pzone->msen_para->sensor_raw_temp(pzone);
		pzone->lasttemp = sensor_temp;
		*temp = sensor_temp;
		pr_debug("sensor_id:%d, rawdata:0x%x, temp:%d\n",
			 pzone->id, pzone->rawdata, sensor_temp);
	} else {
		pr_debug("sensor_id:%d, last temp:%d\n", pzone->id,
			 pzone->lasttemp);
		*temp = pzone->lasttemp;
	}
	return 0;
}

static void sensor_rdy_wait_clear(unsigned long reg)
{
	int cnt = 40;
	int status = thm_reg_read(reg);
	int bit_status = (status >> 3) & 0x1;

	while (bit_status && cnt--) {
		status = thm_reg_read(reg);
		bit_status = (status >> 3) & 0x1;
		udelay(10);
	}
	if (cnt == -1) {
		pr_info("thm sensor timeout 0x%x\n", thm_reg_read(reg));
		thm_reg_write((reg), 0x0, 0x08);
		udelay(200);
		thm_reg_write((reg), 0x8, 0x08);
	}
}

static void sensor_rdy_status(unsigned long reg)
{
	int status = 0;
	int bit_status = 0;

	udelay(200);
	status = thm_reg_read(reg);
	bit_status = (status >> 3) & 0x1;
	if (bit_status) {
		pr_info("thm sensor rdy status need clear\n");
		thm_reg_write((reg), 0x0, 0x08);
		udelay(200);
	}
}

static void sprd_thm_glb_rtc_hw_init(struct regmap *thm_glb)
{
	regmap_update_bits(thm_glb, REG_AON_APB_APB_RTC_EB,
			   (BIT_AON_APB_THM_RTC_EB | BIT_AON_APB_GPU_THMA_RTC_EB
			    | BIT_AON_APB_GPU_THMA_RTC_AUTO_EN |
			    BIT_AON_APB_ARM_THMA_RTC_AUTO_EN),
			   (BIT_AON_APB_THM_RTC_EB | BIT_AON_APB_GPU_THMA_RTC_EB
			    | BIT_AON_APB_GPU_THMA_RTC_AUTO_EN |
			    BIT_AON_APB_ARM_THMA_RTC_AUTO_EN));
}

static void sprd_thm_global_hw_init(struct thm_control *pthm)
{
	unsigned int mask;

	mask = BIT_AON_APB_THM_SOFT_RST;
	sprd_thm_glb_rtc_hw_init(pthm->thm_glb);
	regmap_hwlock_update_bits(pthm->thm_glb,
		REG_AON_APB_APB_RST1, mask, mask);
	udelay(200);
	regmap_hwlock_update_bits(pthm->thm_glb,
		REG_AON_APB_APB_RST1, mask, ~mask);
}

static void sprd_sensor_hw_common_init(struct sprd_thermal_zone *pzone)
{
	int ret = 0;

	ret = pzone->msen_para->sensor_cal(pzone);
	pr_info("sensor cal =%d,cal_off =%d,ret = %d\n", pzone->sensor_cal,
		pzone->sensor_cal_offset, ret);
	pr_info("sprd sensor id:%d,base 0x%lx \n", pzone->id,
		pzone->sensor_base);
	thm_reg_write((pzone->thm_base + pzone->reg->thm_ctrl),
		      pzone->sen_enable, 0);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_int_ctrl), 0, ~0u);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_int_clr), ~0u, 0);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_det_peri),
		      0x800, 0x800);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_mon_ctl), 0x41,
		      0x41);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_mon_peri), 0x40,
		      0x40);
}

static void sprd_cpu_sensor_hw_init(struct sprd_thermal_zone *pzone)
{
	sprd_sensor_hw_common_init(pzone);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_ctrl), 0x30, 0x30);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_ctrl +
		       SENSOR_ADDR_OFF), 0x030, 0x030);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_ctrl), 0x1, 0x1);
	sensor_rdy_status(pzone->sensor_base + pzone->reg->sen_ctrl);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_ctrl), 0x8, 0x08);
	sensor_rdy_wait_clear(pzone->sensor_base + pzone->reg->sen_ctrl);
}

static void sprd_gen_sensor_hw_init(struct sprd_thermal_zone *pzone)
{
	sprd_sensor_hw_common_init(pzone);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_ctrl), 0x1, 0x1);
	sensor_rdy_status(pzone->sensor_base + pzone->reg->sen_ctrl);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_ctrl), 0x8, 0x08);
	sensor_rdy_wait_clear(pzone->sensor_base + pzone->reg->sen_ctrl);
}

static int sprd_gen_sensor_hw_suspend(struct sprd_thermal_zone *pzone)
{
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_ctrl), 0x00, 0x01);
	sensor_rdy_status(pzone->sensor_base + pzone->reg->sen_ctrl);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_ctrl), 0x8, 0x8);
	sensor_rdy_wait_clear(pzone->sensor_base + pzone->reg->sen_ctrl);
	return 0;
}

static int sprd_cpu_sensor_hw_resume(struct sprd_thermal_zone *pzone)
{
	sprd_thm_glb_rtc_hw_init(pzone->thm_glb);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_ctrl +
		       SENSOR_ADDR_OFF), 0x030, 0x0);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_ctrl), 0x31, 0x0);
	sensor_rdy_status(pzone->sensor_base + pzone->reg->sen_ctrl);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_ctrl), 0x8, 0x08);
	sensor_rdy_wait_clear(pzone->sensor_base + pzone->reg->sen_ctrl);

	return 0;
}

static int sprd_gen_sensor_hw_resume(struct sprd_thermal_zone *pzone)
{
	sprd_thm_glb_rtc_hw_init(pzone->thm_glb);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_ctrl), 0x1, 0x1);
	sensor_rdy_status(pzone->sensor_base + pzone->reg->sen_ctrl);
	thm_reg_write((pzone->sensor_base + pzone->reg->sen_ctrl), 0x8, 0x08);
	sensor_rdy_wait_clear(pzone->sensor_base + pzone->reg->sen_ctrl);
	return 0;
}

static void sprd_thm_sensor_hw_init(struct sprd_thermal_zone *pzone)
{
	pzone->sen_ops->sensor_init(pzone);
}

static int sprd_thm_sensor_suspend(struct sprd_thermal_zone *pzone)
{
	pzone->sen_ops->sensor_suspend(pzone);

	return 0;
}

static int sprd_thm_sensor_hw_resume(struct sprd_thermal_zone *pzone)
{
	pzone->sen_ops->sensor_resume(pzone);

	return 0;
}

struct thm_handle_ops sprd_thm_ops = {
	.hw_init = sprd_thm_sensor_hw_init,
	.read_temp = sprd_gen_sensor_temp_read,
	.suspend = sprd_thm_sensor_suspend,
	.resume = sprd_thm_sensor_hw_resume,
};

static void thm_resume_flag_work(struct work_struct *work)
{
	struct sprd_thermal_zone *pzone;
	pzone =
	    container_of(work, struct sprd_thermal_zone,
			 thm_resume_flag_work.work);
	pzone->ready_flag = 1;
}

static struct sensor_ops arm_sensor_info = {
	.sensor_init = sprd_cpu_sensor_hw_init,
	.sensor_read = sprd_gen_sensor_temp_read,
	.sensor_suspend = sprd_gen_sensor_hw_suspend,
	.sensor_resume = sprd_cpu_sensor_hw_resume,
};

static struct sensor_ops gen_sensor_ops = {
	.sensor_init = sprd_gen_sensor_hw_init,
	.sensor_read = sprd_gen_sensor_temp_read,
	.sensor_suspend = sprd_gen_sensor_hw_suspend,
	.sensor_resume = sprd_gen_sensor_hw_resume,
};

static const struct ap_thm_info ap_thm_gen_info = {
	.reg = &sensor_reg,
	.sensor = {
		   {
		    .id = 0,
		    .sensor_name = "arm_sensor",
		    .sensor_addr_off = 0x0,
		    .sen_enable = 0x1,
		    .raw_data_low = 623,
		    .raw_data_high = 1030,
		    .cal_efuse_blk = 7,
		    .cal_efuse_bit = 24,
		    .sensor_cal = sprd_efuse_general_thm_cal_get,
		    .sensor_raw_temp = sprd_rawdata_to_temp_v1,
		    .ops = &arm_sensor_info,
		    },
		   {
		    .id = 1,
		    .sensor_name = "gpu_sensor",
		    .sensor_addr_off = 0x100,
		    .sen_enable = 0x2,
		    .raw_data_low = 623,
		    .raw_data_high = 1030,
		    .cal_efuse_blk = 7,
		    .cal_efuse_bit = 24,
		    .sensor_cal = sprd_efuse_general_thm_cal_get,
		    .sensor_raw_temp = sprd_rawdata_to_temp_v1,
		    .ops = &gen_sensor_ops,
		    },
		   },
};

static const struct ap_thm_info ap_thm_sc7731ce_info = {
	.reg = &sensor_reg,
	.sensor = {
		   {
		    .id = 0,
		    .sensor_name = "arm_sensor",
		    .sensor_addr_off = 0x0,
		    .sen_enable = 0x1,
		    .raw_data_low = 627,
		    .raw_data_high = 1075,
		    .cal_efuse_blk = 13,
		    .cal_efuse_bit = 26,
		    .sensor_cal = sprd_efuse_general_thm_cal_get,
		    .sensor_raw_temp = sprd_rawdata_to_temp_v1,
		    .ops = &arm_sensor_info,
		    },
		   {
		    .id = 1,
		    .sensor_name = "gpu_sensor",
		    .sensor_addr_off = 0x100,
		    .sen_enable = 0x2,
		    .raw_data_low = 627,
		    .raw_data_high = 1075,
		    .cal_efuse_blk = 13,
		    .cal_efuse_bit = 26,
		    .sensor_cal = sprd_efuse_general_thm_cal_get,
		    .sensor_raw_temp = sprd_rawdata_to_temp_v1,
		    .ops = &gen_sensor_ops,
		    },
		   },
};

static const struct ap_thm_info ap_whale_thm_info = {
	.reg = &thm_thm_registers,
	.sensor = {
		   {
		    .id = 0,
		    .sensor_name = "arm_sensor",
		    .sensor_addr_off = 0x0,
		    .sen_enable = 0x1,
		    .raw_data_low = 0,
		    .raw_data_high = 240,
		    .cal_efuse_blk = 14,
		    .cal_efuse_bit = 6,
		    .sensor_cal = sprd_efuse_whale_arm_cal,
		    .sensor_raw_temp = sprd_rawdata_to_temp_v2,
		    .ops = &gen_sensor_ops,
		    },
		   {
		    .id = 1,
		    .sensor_name = "remote_sensor",
		    .sensor_addr_off = 0x100,
		    .sen_enable = 0x2,
		    .raw_data_low = 0,
		    .raw_data_high = 240,
		    .cal_efuse_blk = 14,
		    .cal_efuse_bit = 11,
		    .sensor_cal = sprd_efuse_whale_arm_cal,
		    .sensor_raw_temp = sprd_rawdata_to_temp_v2,
		    .ops = &gen_sensor_ops,
		    },
		   {
		    .id = 2,
		    .sensor_name = "north_west_ddr_sensor",
		    .sensor_addr_off = 0x200,
		    .sen_enable = 0x4,
		    .raw_data_low = 0,
		    .raw_data_high = 240,
		    .cal_efuse_blk = 14,
		    .cal_efuse_bit = 16,
		    .sensor_cal = sprd_efuse_whale_ddr_gpu_cal,
		    .sensor_raw_temp = sprd_rawdata_to_temp_v2,
		    .ops = &gen_sensor_ops,
		    },
		   {
		    .id = 3,
		    .sensor_name = "gpu_sensor",
		    .sensor_addr_off = 0x300,
		    .sen_enable = 0x8,
		    .raw_data_low = 0,
		    .raw_data_high = 240,
		    .cal_efuse_blk = 14,
		    .cal_efuse_bit = 21,
		    .sensor_cal = sprd_efuse_whale_ddr_gpu_cal,
		    .sensor_raw_temp = sprd_rawdata_to_temp_v2,
		    .ops = &gen_sensor_ops,
		    },
		   {
		    .id = 4,
		    .sensor_name = "middle_east_ddr_sensor",
		    .sensor_addr_off = 0x400,
		    .sen_enable = 0x10,
		    .raw_data_low = 0,
		    .raw_data_high = 240,
		    .cal_efuse_blk = 14,
		    .cal_efuse_bit = 26,
		    .sensor_cal = sprd_efuse_whale_ddr_gpu_cal,
		    .sensor_raw_temp = sprd_rawdata_to_temp_v2,
		    .ops = &gen_sensor_ops,
		    },
		   {
		    .id = 5,
		    .sensor_name = "middle_west_ddr_sensor",
		    .sensor_addr_off = 0x500,
		    .sen_enable = 0x20,
		    .raw_data_low = 0,
		    .raw_data_high = 240,
		    .cal_efuse_blk = 13,
		    .cal_efuse_bit = 19,
		    .sensor_cal = sprd_efuse_whale_ddr_gpu_cal,
		    .sensor_raw_temp = sprd_rawdata_to_temp_v2,
		    .ops = &gen_sensor_ops,
		    },
		   },
};

static const struct of_device_id ap_thermal_of_match[] = {
	{.compatible = "sprd,ap-thermal",.data = (void *)&ap_thm_gen_info,},
	{.compatible = "sprd,sc7731ce-thermal",.data =
	 (void *)&ap_thm_sc7731ce_info,},
	{.compatible = "sprd,whale-thermal",.data =
	 (void *)&ap_whale_thm_info,},
	{}
};

static void parse_ipa_params(const struct device_node *np,
			     struct sprd_thermal_zone *pzone)
{
	int ret = 0;
	ret = of_property_read_u32(np, "k-po", &pzone->ipa_para.k_po);
	if (ret) {
		pzone->ipa_para.k_po = 0;
		pr_err("no k_po property for ap thm");
		return;
	}
	ret = of_property_read_u32(np, "k-pu", &pzone->ipa_para.k_pu);
	if (ret) {
		pzone->ipa_para.k_pu = 0;
		pr_err("no k_pu property for ap thm");
		return;
	}
	ret = of_property_read_u32(np, "k-i", &pzone->ipa_para.k_i);
	if (ret) {
		pzone->ipa_para.k_i = 0;
		pr_err("no k_i property for ap thm");
		return;
	}
	ret = of_property_read_u32(np, "k-d", &pzone->ipa_para.k_d);
	if (ret) {
		pzone->ipa_para.k_d = 0;
		pr_err("no k_d property for ap thm");
		return;
	}

	ret =
	    of_property_read_u32(np, "cutoff",
				 &pzone->ipa_para.integral_cutoff);
	if (ret) {
		pzone->ipa_para.integral_cutoff = 0;
		pr_err("no cutoff property for ap thm");
		return;
	}
}

static int sprd_ap_thm_probe(struct platform_device *pdev)
{
	struct sprd_thermal_zone *pzone = NULL;
	struct thm_control *thm_dev = NULL;
	struct device_node *sensor_child = NULL;
	struct device_node *np = NULL;
	struct resource *res;
	struct regmap *thm_reg;
	struct sensor_info *mpara;
	const struct of_device_id *of_id;
	const struct ap_thm_info *info;
	unsigned long ap_thm_base;
	int ret, nsensor;
	int i = 0;

	pr_info("sprd ap thm probe start\n");
	np = pdev->dev.of_node;
	if (!np) {
		dev_err(&pdev->dev, "device node not found\n");
		return -EINVAL;
	}
	nsensor = of_get_child_count(np);
	if (nsensor == 0) {
		dev_err(&pdev->dev, "fail to get child node \n");
		return -EINVAL;
	}
	of_id = of_match_node(ap_thermal_of_match, pdev->dev.of_node);
	if (!of_id) {
		pr_err("get device id failed!\n");
		return -ENODEV;
	}
	info = of_id->data;
	thm_reg = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						  "sprd,syscon-enable");
	if (IS_ERR(thm_reg)) {
		dev_err(&pdev->dev, "get the syscon enable failed!\n");
		return -ENODEV;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ap_thm_base = (unsigned long)devm_ioremap_resource(&pdev->dev, res);
	if (!ap_thm_base) {
		dev_err(&pdev->dev, "thm ioremap failed!\n");
		return -ENOMEM;
	}

	thm_dev = devm_kzalloc(&pdev->dev, sizeof(*thm_dev), GFP_KERNEL);
	if (!thm_dev)
		return -ENOMEM;
	thm_dev->nsensor = nsensor;
	thm_dev->thm_glb = thm_reg;

	thm_dev->clk = of_clk_get_by_name(pdev->dev.of_node, "enable");
	if (IS_ERR(thm_dev->clk)) {
		pr_err("ap efuse can't get the clock dts config: enable\n");
		return -EINVAL;
	}
	clk_prepare_enable(thm_dev->clk);
	sprd_thm_global_hw_init(thm_dev);
	INIT_LIST_HEAD(&thm_dev->thmzone_list);

	for_each_child_of_node(np, sensor_child) {
		pr_info("sprd sensor child start\n");
		pzone = devm_kzalloc(&pdev->dev, sizeof(*pzone), GFP_KERNEL);
		if (!pzone)
			return -ENOMEM;

		ret = of_property_read_u32(sensor_child, "reg", &pzone->id);
		if (ret) {
			dev_err(&pdev->dev,
				"no reg sensor id property for ap thm");
			continue;
		}

		mutex_init(&pzone->th_lock);
		parse_ipa_params(sensor_child, pzone);

		mpara = devm_kzalloc(&pdev->dev, sizeof(*mpara), GFP_KERNEL);
		if (!mpara)
			return -ENOMEM;

		pr_info(" sprd ap thm probe id:%d\n", pzone->id);
		pzone->dev = &pdev->dev;
		pzone->thm_base = ap_thm_base;
		pzone->thm_glb = thm_reg;
		pzone->ops = &sprd_thm_ops;
		pzone->reg = info->reg;
		pzone->sen_ops = info->sensor[pzone->id].ops;
		pzone->sensor_base =
		    pzone->thm_base + info->sensor[pzone->id].sensor_addr_off;
		pzone->sen_enable = info->sensor[pzone->id].sen_enable;

		mpara->raw_data_low = info->sensor[pzone->id].raw_data_low;
		mpara->raw_data_high = info->sensor[pzone->id].raw_data_high;
		mpara->cal_efuse_blk = info->sensor[pzone->id].cal_efuse_blk;
		mpara->cal_efuse_bit = info->sensor[pzone->id].cal_efuse_bit;
		mpara->sensor_cal = info->sensor[pzone->id].sensor_cal;
		mpara->sensor_raw_temp =
		    info->sensor[pzone->id].sensor_raw_temp;
		pzone->msen_para = mpara;

		INIT_DELAYED_WORK(&pzone->thm_resume_flag_work,
				  thm_resume_flag_work);
		strlcpy(pzone->name, sensor_child->name, sizeof(pzone->name));
		sprd_thm_sensor_hw_init(pzone);
		pr_info("thm hw init end \n");
		ret = sprd_thermal_init(pzone);
		if (ret) {
			dev_err(&pdev->dev,
				" thm sensor sw init error id =%d\n",
				pzone->id);
			continue;
		}
		pzone->ready_flag = 1;
		thm_dev->name[i++] = pzone->name;
		list_add_tail(&pzone->tz_node, &thm_dev->thmzone_list);
		pzone++;
	}

	if (list_empty(&thm_dev->thmzone_list)) {
		dev_err(&pdev->dev, " thm sensor probe error\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, thm_dev);
	pr_info("sprd ap thm probe end\n");

	return 0;
}

static struct sprd_thermal_zone *get_thmzone_by_name(struct thm_control
						     *pthmdev, const char *name)
{
	struct sprd_thermal_zone *pos = NULL;
	struct sprd_thermal_zone *target_thmzone = NULL;
	if (!name)
		goto exit;

	mutex_lock(&thm_list_lock);
	if (list_empty(&pthmdev->thmzone_list)) {
		mutex_unlock(&thm_list_lock);
		return target_thmzone;
	}

	list_for_each_entry(pos, &pthmdev->thmzone_list, tz_node) {
		if (!strncmp(name, pos->name, THERMAL_NAME_LENGTH)) {
			target_thmzone = pos;
			break;
		}
	}
	mutex_unlock(&thm_list_lock);
exit:
	return target_thmzone;
}

static int sprd_ap_thm_remove(struct platform_device *pdev)
{
	int i;
	struct thm_control *pthmdev = platform_get_drvdata(pdev);
	struct sprd_thermal_zone *pzone = NULL;

	for (i = 0; i < pthmdev->nsensor; i++) {
		pzone = get_thmzone_by_name(pthmdev, pthmdev->name[i]);
		if (pzone == NULL)
			continue;
		sprd_thermal_remove(pzone);
	}
	return 0;
}

static int sprd_ap_thm_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i;
	struct thm_control *pthmdev = platform_get_drvdata(pdev);
	struct sprd_thermal_zone *pzone = NULL;

	for (i = 0; i < pthmdev->nsensor; i++) {
		pzone = get_thmzone_by_name(pthmdev, pthmdev->name[i]);
		if (pzone == NULL)
			continue;
		flush_delayed_work(&pzone->thm_resume_flag_work);
		flush_delayed_work(&pzone->resume_delay_work);
		pzone->ops->suspend(pzone);
		pzone->ready_flag = 0;
	}
	return 0;
}

static int sprd_ap_thm_resume(struct platform_device *pdev)
{
	int i;
	struct thm_control *pthmdev = platform_get_drvdata(pdev);
	struct sprd_thermal_zone *pzone = NULL;

	for (i = 0; i < pthmdev->nsensor; i++) {
		pzone = get_thmzone_by_name(pthmdev, pthmdev->name[i]);
		if (pzone == NULL)
			continue;
		queue_delayed_work(system_power_efficient_wq,
				   &pzone->resume_delay_work, (HZ * 1));
		queue_delayed_work(system_power_efficient_wq,
				   &pzone->thm_resume_flag_work, (HZ * 3));
	}
	return 0;
}

static struct platform_driver sprd_ap_thermal_driver = {

	.probe = sprd_ap_thm_probe,
	.suspend = sprd_ap_thm_suspend,
	.resume = sprd_ap_thm_resume,
	.remove = sprd_ap_thm_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ap-thermal",
		   .of_match_table = of_match_ptr(ap_thermal_of_match),
		   },
};

static int __init sprd_ap_thermal_init(void)
{
	return platform_driver_register(&sprd_ap_thermal_driver);
}

static void __exit sprd_ap_thermal_exit(void)
{
	platform_driver_unregister(&sprd_ap_thermal_driver);
}

device_initcall_sync(sprd_ap_thermal_init);
module_exit(sprd_ap_thermal_exit);

MODULE_AUTHOR("Freeman Liu <freeman.liu@spreadtrum.com>");
MODULE_DESCRIPTION("sprd thermal driver");
MODULE_LICENSE("GPL");
