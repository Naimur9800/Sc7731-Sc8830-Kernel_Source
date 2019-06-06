/*copyright (C) 2016 Spreadtrum Communications Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/compat.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include "sprd_dmc_mpu.h"

const struct of_device_id *lock_of_id;
static int sprd_dmc_mpu_iq;
static const char **sprd_dmc_mpu_list;

static const struct of_device_id dmc_mpu_of_match[] = {
	{ .compatible = "sprd,dmc-mpu-whale2", .data = &whale2},
	{ .compatible = "sprd,dmc-mpu-isharkl2", .data = &isharkl2},
	{ .compatible = "sprd,dmc-mpu-sharkl2", .data = &sharkl2},
	{ .compatible = "sprd,dmc-mpu-sharklj1", .data = &sharklj1},
	{ },
};

static int __init sprd_dmc_mpu_early_mode(char *str)
{
	pr_info("DMC_MPU:early_mode\n");
	if (!memcmp(str, "iq", 2))
		sprd_dmc_mpu_iq = 1;

	return 0;
}
early_param("androidboot.mode", sprd_dmc_mpu_early_mode);

static void sprd_dmc_mpu_clk_set(void __iomem *mpu_pub_base,
	bool enable)
{
	u32 val;

	close_smart_lightsleep();
	val = readl_relaxed(mpu_pub_base + MPU_EN);
	if (enable)
		val |= BIT_MPU_EB;
	else
		val &= ~BIT_MPU_EB;
	writel_relaxed(val, mpu_pub_base + MPU_EN);
	open_smart_lightsleep();
}

static void sprd_dmc_mpu_clk_enable(struct sprd_dmc_mpu_dev *sdev,
	bool enable)
{
	sprd_dmc_mpu_clk_set(sdev->mpu_pub0_base, enable);

	if (sdev->interleaved_mod)
		sprd_dmc_mpu_clk_set(sdev->mpu_pub1_base, enable);
}

static void sprd_dmc_mpu_int_set(void __iomem *mpu_pub_base,
	bool enable)
{
	u32 val;

	close_smart_lightsleep();
	val = readl_relaxed(mpu_pub_base + MPU_INT);
	if (enable)
		val |= BIT_MPU_INT;
	else
		val &= ~BIT_MPU_INT;
	writel_relaxed(val, mpu_pub_base + MPU_INT);
	open_smart_lightsleep();
}

static void sprd_dmc_mpu_int_enable(struct sprd_dmc_mpu_dev *sdev,
	bool enable)
{
	sprd_dmc_mpu_int_set(sdev->mpu_pub0_base, enable);

	if (sdev->interleaved_mod)
		sprd_dmc_mpu_int_set(sdev->mpu_pub1_base, enable);
}

static void sprd_dmc_mpu_int_clr(struct sprd_dmc_mpu_dev *sdev)
{
	u32 val;

	close_smart_lightsleep();
	val = readl_relaxed(sdev->mpu_pub0_base + MPU_INT);
	val |= BIT_MPU_INT_CLR;
	writel_relaxed(val, sdev->mpu_pub0_base + MPU_INT);
	val &= ~BIT_MPU_INT_CLR;
	writel_relaxed(val, sdev->mpu_pub0_base + MPU_INT);

	if (sdev->interleaved_mod) {
		val = readl_relaxed(sdev->mpu_pub1_base + MPU_INT);
		val |= BIT_MPU_INT_CLR;
		writel_relaxed(val, sdev->mpu_pub1_base + MPU_INT);
		val &= ~BIT_MPU_INT_CLR;
		writel_relaxed(val, sdev->mpu_pub1_base + MPU_INT);
	}
	open_smart_lightsleep();
}

static void sprd_dmc_mpu_rst(struct sprd_dmc_mpu_dev *sdev,
	int pub, int chn, enum sprd_mpu_mode flag)
{
	u32 val;

	close_smart_lightsleep();
	if (!pub) {
		val = readl_relaxed(sdev->mpu_pub0_base + MPU_RESET);
		val &= ~(BIT_MPU_RST(chn * 2 + (int)flag));
		writel_relaxed(val, sdev->mpu_pub0_base + MPU_RESET);
		val |= BIT_MPU_RST(chn * 2 + (int)flag);
		writel_relaxed(val, sdev->mpu_pub0_base + MPU_RESET);
	} else {
		if (sdev->interleaved_mod) {
			val = readl_relaxed(sdev->mpu_pub1_base + MPU_RESET);
			val &= ~(BIT_MPU_RST(chn * 2 + (int)flag));
			writel_relaxed(val, sdev->mpu_pub1_base + MPU_RESET);
			val |= BIT_MPU_RST(chn * 2 + (int)flag);
			writel_relaxed(val, sdev->mpu_pub1_base + MPU_RESET);
		}
	}
	open_smart_lightsleep();
}

static void sprd_dmc_mpu_chn_disable(struct sprd_dmc_mpu_dev *sdev)
{
	close_smart_lightsleep();
	writel_relaxed(0, sdev->mpu_pub0_base + MPU_RESET);

	if (sdev->interleaved_mod)
		writel_relaxed(0, sdev->mpu_pub1_base + MPU_RESET);
	open_smart_lightsleep();
}

static void sprd_dmc_mpu_dump_init(struct sprd_dmc_mpu_dev *sdev,
	int pub)
{
	u32 dump_addr;
	int chn;

	close_smart_lightsleep();
	if (!pub) {
		if (!sdev->pub0_dump_addr_p) {
			pr_err("DMC MPU: There is not pub0 dump addr!\n");
			goto err_mpu_config;
		}
		dump_addr = (u32)sdev->pub0_dump_addr_p & ~MPU_ADDR_MASK;
		for (chn = 0; chn < sdev->pub_chn; chn++)
			writel_relaxed(dump_addr,
				sdev->mpu_pub0_base + ADDR_DUMP(chn));
	} else {
		if (sdev->interleaved_mod) {
			if (!sdev->pub1_dump_addr_p) {
				pr_err("DMC MPU: There is not pub0 dump addr!\n");
				goto err_mpu_config;
			}
			dump_addr = (u32)sdev->pub1_dump_addr_p &
				~MPU_ADDR_MASK;
			for (chn = 0; chn < sdev->pub_chn; chn++)
				writel_relaxed(dump_addr,
					sdev->mpu_pub1_base + ADDR_DUMP(chn));
		} else {
			pr_err("DMC MPU: pub1 INTERLEAVED mode failed!\n");
			goto err_mpu_config;
		}
	}

err_mpu_config:
	open_smart_lightsleep();
}

static int sprd_dmc_mpu_config(struct sprd_dmc_mpu_dev *sdev,
	struct sprd_mpu_chn *mpu_cfg)
{
	void __iomem *mpu_pub;
	u32 addr_range, mod_sel, val, reg_chn;

	close_smart_lightsleep();
	addr_range = mpu_cfg->range_addr & ~MPU_DDR_ADDR_MASK;
	/* confirm which read/write reg_chn to config */
	reg_chn = mpu_cfg->chn * 2 + (int)mpu_cfg->flag;
	mod_sel = BIT(reg_chn);

	if (!mpu_cfg->pub)
		mpu_pub = sdev->mpu_pub0_base;
	else
		mpu_pub = sdev->mpu_pub1_base;
	writel_relaxed(addr_range, mpu_pub + MPU_RANGE(mpu_cfg->chn));
	val = readl_relaxed(mpu_pub + MPU_SEL);
	if (mpu_cfg->incld_mod == true)
		val = val | mod_sel;
	else
		val = val & ~mod_sel;
	writel_relaxed(val, mpu_pub + MPU_SEL);

	sprd_dmc_mpu_dump_init(sdev, mpu_cfg->pub);
	sprd_dmc_mpu_rst(sdev, mpu_cfg->pub, mpu_cfg->chn, mpu_cfg->flag);

	mpu_cfg->sticky_enable = true;

	reg_chn = mpu_cfg->chn + mpu_cfg->pub * sdev->pub_chn;
	memcpy(&sdev->chns[reg_chn], mpu_cfg,
		sizeof(struct sprd_mpu_chn));
	open_smart_lightsleep();

	return 0;
}

static void sprd_dmc_mpu_disable(struct sprd_dmc_mpu_dev *sdev)
{
	sprd_dmc_mpu_int_enable(sdev, false);
	sprd_dmc_mpu_clk_enable(sdev, false);
}

/*
 * sprd_dmc_mpu_ctrl - enable/disable dmc mpu.
 * If CP assert, AP needs to copy img to CP modem addr rang
 * to restart CP, so it needs to disable dmc mpu.
 */
void sprd_dmc_mpu_ctrl(bool enable)
{
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct sprd_dmc_mpu_dev *sdev = NULL;
	struct sprd_mpu_chn *chn_cfg = NULL;
	int i, cnt;

	pr_info("%s, enable %d\n", __func__, enable);
	cnt = sizeof(dmc_mpu_of_match) / sizeof(struct of_device_id);
	for (i = 0; i < cnt - 1; i++) {
		np = of_find_compatible_node(NULL, NULL,
				dmc_mpu_of_match[i].compatible);
		if (np)
			break;
	}
	if (!np) {
		pr_err("%s, can't get device node!\n", __func__);
		return;
	}
	pdev = of_find_device_by_node(np);
	sdev = platform_get_drvdata(pdev);
	if (!sdev) {
		pr_err("%s, can't get sdev!\n", __func__);
		return;
	}

	if (enable) {
		for (cnt = 0; cnt < sdev->pub_chn * (sdev->interleaved_mod + 1);
				cnt++) {
			chn_cfg = &sdev->chns[cnt];
			if (chn_cfg->sticky_enable) {
				sprd_dmc_mpu_config(sdev, chn_cfg);
				sprd_dmc_mpu_clk_enable(sdev, true);
				sprd_dmc_mpu_int_enable(sdev, true);
			}
		}
	} else {
		sprd_dmc_mpu_chn_disable(sdev);
		sprd_dmc_mpu_disable(sdev);
	}
}
EXPORT_SYMBOL_GPL(sprd_dmc_mpu_ctrl);

/*
 * sprd_dmc_mpu_ap - protect CP addr rang.
 * Protect CP addr rang, AP masters can not access this addr rang.
 * If it is exclude mode, the addr rang should be AP addr rang.
 * @cp_start_addr: address of start address.
 * @cp_end_addr: address of end address.
 * @mode: if true, it is include mode, else it is exclude mode.
 */
void sprd_dmc_mpu_ap(u64 cp_start_addr, u64 cp_end_addr, bool mode)
{
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct sprd_dmc_mpu_dev *sdev = NULL;
	int i, cnt;

	if (cp_start_addr & ~MPU_ADDR_ALIGN)
		cp_start_addr = (cp_start_addr & MPU_ADDR_ALIGN)
					+ 0x10000;
	else
		cp_start_addr = cp_start_addr & MPU_ADDR_ALIGN;
	cp_end_addr &= MPU_ADDR_ALIGN;
	if ((cp_end_addr < cp_start_addr) ||
		(cp_start_addr & ~MPU_ADDR_ALIGN) ||
		(cp_end_addr & ~MPU_ADDR_ALIGN)) {
		pr_err("%s, the address is illegal!\n", __func__);
		return;
	}
	cnt = sizeof(dmc_mpu_of_match) / sizeof(struct of_device_id);
	for (i = 0; i < cnt - 1; i++) {
		np = of_find_compatible_node(NULL, NULL,
				dmc_mpu_of_match[i].compatible);
		if (np)
			break;
	}
	if (!np) {
		pr_err("%s, can't get device node!\n", __func__);
		return;
	}
	pdev = of_find_device_by_node(np);
	sdev = platform_get_drvdata(pdev);
	if (!sdev) {
		pr_err("%s, can't get sdev!\n", __func__);
		return;
	}
	sdev->dbg_info.chn.range_addr = (cp_start_addr >> 16) | cp_end_addr;
	sdev->dbg_info.chn.incld_mod = mode;
	sdev->dbg_info.chn.pub = 0;
	sdev->dbg_info.chn.flag = W_MODE;
	sprd_dmc_mpu_clk_enable(sdev, true);
	for (i = 0; i < sdev->pub_chn; i++) {
		if (strstr(sprd_dmc_mpu_list[i], "CP"))
			continue;
		if (strstr(sprd_dmc_mpu_list[i], "DSP"))
			continue;
		if (strstr(sprd_dmc_mpu_list[i], "ACC"))
			continue;
		sdev->dbg_info.chn.chn = i;
		sprd_dmc_mpu_config(sdev, &sdev->dbg_info.chn);
	}
	sprd_dmc_mpu_int_enable(sdev, true);
}
EXPORT_SYMBOL_GPL(sprd_dmc_mpu_ap);

/*
 * sprd_dmc_mpu_cp - protect AP addr rang.
 * Protect AP addr rang, CP masters can not access this addr rang.
 * If it is exclude mode, the addr rang should be CP addr rang.
 * @ap_start_addr: address of start address.
 * @ap_end_addr: address of end address.
 * @mode: if true, it is include mode, else it is exclude mode.
 */
void sprd_dmc_mpu_cp(u64 ap_start_addr, u64 ap_end_addr, bool mode)
{
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct sprd_dmc_mpu_dev *sdev = NULL;
	int i, cnt;

	if (ap_end_addr & ~MPU_ADDR_ALIGN)
		ap_end_addr = (ap_end_addr & MPU_ADDR_ALIGN)
			+ 0x10000;
	else
	ap_start_addr = ap_start_addr & MPU_ADDR_ALIGN;
	if ((ap_end_addr < ap_start_addr) ||
		(ap_start_addr & ~MPU_ADDR_ALIGN) ||
		(ap_end_addr & ~MPU_ADDR_ALIGN)) {
		pr_err("%s, the address is illegal!\n", __func__);
		return;
	}
	cnt = sizeof(dmc_mpu_of_match) / sizeof(struct of_device_id);
	for (i = 0; i < cnt - 1; i++) {
		np = of_find_compatible_node(NULL, NULL,
				dmc_mpu_of_match[i].compatible);
		if (np)
			break;
	}
	if (!np) {
		pr_err("%s, can't get device node!\n", __func__);
		return;
	}
	pdev = of_find_device_by_node(np);
	sdev = platform_get_drvdata(pdev);
	if (!sdev) {
		pr_err("%s, can't get sdev!\n", __func__);
		return;
	}
	sdev->dbg_info.chn.range_addr = (ap_start_addr >> 16) | ap_end_addr;
	sdev->dbg_info.chn.incld_mod = mode;
	sdev->dbg_info.chn.pub = 0;
	sdev->dbg_info.chn.flag = W_MODE;
	sprd_dmc_mpu_clk_enable(sdev, true);
cfg_again:
	for (i = 0; i < sdev->pub_chn; i++) {
		/* isharkl2 chn 5 is wtlcp/aon
		 * will access ap and cp memory
		 */
		if (this_offset->chip == ISHARKL2) {
			if (i == 5)
				continue;
		}
		if (strstr(sprd_dmc_mpu_list[i], "CP")) {
			sdev->dbg_info.chn.chn = i;
			sprd_dmc_mpu_config(sdev, &sdev->dbg_info.chn);
		}
		if (strstr(sprd_dmc_mpu_list[i], "DSP")) {
			sdev->dbg_info.chn.chn = i;
			sprd_dmc_mpu_config(sdev, &sdev->dbg_info.chn);
		}
		if (strstr(sprd_dmc_mpu_list[i], "ACC")) {
			sdev->dbg_info.chn.chn = i;
			sprd_dmc_mpu_config(sdev, &sdev->dbg_info.chn);
		}
	}
	/* pub1 DDR mpu is configed */
	if (sdev->interleaved_mod && sdev->dbg_info.chn.pub)
		return;
	if (sdev->interleaved_mod == true) {
		sdev->dbg_info.chn.pub = 1;
		sdev->dbg_info.chn.incld_mod = true;
		sdev->dbg_info.chn.range_addr = 0xffff0000;
		goto cfg_again;
	}
	sprd_dmc_mpu_int_enable(sdev, true);
}
EXPORT_SYMBOL_GPL(sprd_dmc_mpu_cp);

/*
 * sprd_dmc_mpu_def_prot - set def protect rang.
 * All the AP master(include arm, ap, cam, dispc) will be protected in
 * CP address rang, and all the CP master(include pub cp, dsp, wtlcp, agcp)
 * will be pretected in AP address rang.
 * Return: 0 if the cfg is set success, else the return value is -1.
 */
static int sprd_dmc_mpu_def_prot(struct sprd_dmc_mpu_dev *sdev)
{
	struct device_node *np = NULL;
	struct resource res;
	u64 mem_rang[6] = {};
	u64 cp_start, cp_end;
	int ret;

	/* get CP modem addr rang */
	np = of_find_node_by_name(NULL, "cp-modem");
	if (np) {
		ret = of_address_to_resource(np, 0, &res);
		if (!ret) {
			mem_rang[0] = res.start;
			mem_rang[1] = res.end;
		}
	} else {
		pr_err("sprd dmc mpu get cp moden addr error!\n");
		mem_rang[0] = 0;
		mem_rang[0] = mem_rang[0] - 1;
		mem_rang[1] = 0;
	}

	/* get Audio share memory addr rang, AGCP will access this addr rang */
	np = of_find_node_by_name(NULL, "audio-mem");
	if (np) {
		ret = of_address_to_resource(np, 0, &res);
		if (!ret) {
			mem_rang[2] = res.start;
			mem_rang[3] = res.end;
		}
	} else {
		pr_err("sprd dmc mpu get audio addr error!\n");
		mem_rang[2] = 0;
		mem_rang[2] = mem_rang[2] - 1;
		mem_rang[3] = 0;
	}

	/* get sipc share memory addr rang */
	np = of_find_node_by_name(NULL, "sipc-mem");
	if (np) {
		ret = of_address_to_resource(np, 0, &res);
		if (!ret) {
			mem_rang[4] = res.start;
			mem_rang[5] = res.end;
		}
	} else {
		pr_err("sprd dmc mpu get audio addr error!\n");
		mem_rang[4] = 0;
		mem_rang[4] = mem_rang[4] - 1;
		mem_rang[5] = 0;
	}
	cp_start = mem_rang[0];
	if (cp_start > mem_rang[2])
		cp_start = mem_rang[2];
	if (cp_start > mem_rang[4])
		cp_start = mem_rang[4];
	cp_end = mem_rang[1];
	if (cp_end < mem_rang[3])
		cp_end = mem_rang[3];
	if (cp_end < mem_rang[5])
		cp_end = mem_rang[5];

	sprd_dmc_mpu_cp(cp_start, cp_end, false);
	if (this_offset->chip == SHARKL2)
		sprd_dmc_mpu_ap(mem_rang[0], mem_rang[1], true);
	return 0;
}


static irqreturn_t sprd_dmc_mpu_irq(int irq_num, void *dev)
{
	struct sprd_dmc_mpu_dev *sdev = NULL;
	void __iomem *mpu_pub, *dump_addr_v;
	u32 voi_addr = 0;
	int chn, offset;

	sdev = (struct sprd_dmc_mpu_dev *)platform_get_drvdata(
		(struct platform_device *)dev);
	if (sdev == NULL) {
		pr_emerg("DMC MPU: irq get dev data fail!\n");
		return IRQ_RETVAL(-EINVAL);
	}

	sprd_dmc_mpu_int_clr(sdev);

	if (irq_num == sdev->pub0_irq) {
		sdev->dbg_info.chn.pub = 0;
		mpu_pub = sdev->mpu_pub0_base;
		dump_addr_v = sdev->pub0_dump_addr_v;
	} else {
		if (!sdev->interleaved_mod)
			return IRQ_RETVAL(-EINVAL);
		sdev->dbg_info.chn.pub = 1;
		mpu_pub = sdev->mpu_pub1_base;
		dump_addr_v = sdev->pub1_dump_addr_v;
	}

	close_smart_lightsleep();
	/* get the real dump, because there is not chn int raw/mask */
	for (chn = 0; chn < sdev->pub_chn; chn++) {
		voi_addr = readl_relaxed(mpu_pub + MPU_VIO_WADDR(chn));
		voi_addr |= readl_relaxed(mpu_pub + MPU_VIO_WCMD(chn));
		voi_addr |= readl_relaxed(mpu_pub + MPU_VIO_RADDR(chn));
		voi_addr |= readl_relaxed(mpu_pub + MPU_VIO_RCMD(chn));
		if (voi_addr)
			break;
	}
	open_smart_lightsleep();

	if (chn > sdev->pub_chn) {
		pr_emerg("DMC MPU: get pub dmc chn fail!\n");
		return IRQ_RETVAL(-EINVAL);
	}

	sdev->dbg_info.chn.chn = chn;

	close_smart_lightsleep();
	sdev->dbg_info.dump_addr =
		readl_relaxed(mpu_pub + ADDR_DUMP(chn));
	sdev->dbg_info.vio_waddr =
		readl_relaxed(mpu_pub + MPU_VIO_WADDR(chn));
	sdev->dbg_info.vio_raddr =
		readl_relaxed(mpu_pub + MPU_VIO_RADDR(chn));
	sdev->dbg_info.vio_wcmd =
		readl_relaxed(mpu_pub + MPU_VIO_WCMD(chn));
	sdev->dbg_info.vio_rcmd =
		readl_relaxed(mpu_pub + MPU_VIO_RCMD(chn));
	/* 0x40 is the dump offset, we need to confirm with ASIC.
	  * the dump addr is only 0x40 align, so we want to get the
	  * offset from dump data offset.
	  * But whale2 chip can't get the offset addr.
	  */
	for (offset = 0; offset < 16; offset++)
		sdev->dbg_info.dump_data[offset] =
			readl_relaxed(dump_addr_v + offset * 4);
	open_smart_lightsleep();

	if ((sdev->dbg_info.vio_waddr || sdev->dbg_info.vio_wcmd))
		sdev->dbg_info.vio_waddr |= MPU_ADDR_MASK;
	else if ((sdev->dbg_info.vio_raddr || sdev->dbg_info.vio_rcmd))
		sdev->dbg_info.vio_raddr |= MPU_ADDR_MASK;

	pr_emerg("Warning! DMC MPU detected violated transaction!!!\n");
	pr_emerg("PUB%d: chn%d: %s\ndump: 0x%08X\nraddr: 0x%08X - cmd: 0x%08X\n",
		sdev->dbg_info.chn.pub,
		sdev->dbg_info.chn.chn,
		sprd_dmc_mpu_list[sdev->chns[chn].chn],
		sdev->dbg_info.dump_addr,
		sdev->dbg_info.vio_raddr,
		sdev->dbg_info.vio_rcmd);
	pr_emerg("waddr: 0x%08X - cmd: 0x%08X\ndump data:\n",
		sdev->dbg_info.vio_waddr,
		sdev->dbg_info.vio_wcmd);
	for (offset = 0; offset < 16; offset += 4)
		pr_emerg("0x%08X\n0x%08X\n0x%08X\n0x%08X\n",
		sdev->dbg_info.dump_data[offset],
		sdev->dbg_info.dump_data[offset + 1],
		sdev->dbg_info.dump_data[offset + 2],
		sdev->dbg_info.dump_data[offset + 3]);
	if (sdev->dbg_info.chn.panic == true)
		BUG();
	sprd_dmc_mpu_rst(sdev, sdev->dbg_info.chn.pub,
		chn, sdev->chns[chn].flag);

	return IRQ_HANDLED;
}

static ssize_t pub_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(dev);

	switch (sdev->dbg_info.chn.pub) {
	case 0:
		return sprintf(buf, "PUB0 selected.\n");
	case 1:
		return sprintf(buf, "PUB1 selected.\n");
	default:
		return sprintf(buf, "PUB0/PUB1 selected.\n");
	}
}

static ssize_t pub_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(dev);
	unsigned long pub, ret;

	ret = kstrtoul(buf, 0, &pub);
	if (ret)
		return -EINVAL;
	switch (pub) {
	case 0:
		pr_info("PUB0 selected.\n");
		sdev->dbg_info.chn.pub = 0;
		break;
	case 1:
		pr_info("PUB1 selected.\n");
		sdev->dbg_info.chn.pub = 1;
		break;
	default:
		pr_info("PUB0/PUB1 selected.\n");
		sdev->dbg_info.chn.pub = 3;
	}
	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(pub);

static ssize_t range_mod_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(dev);

	if (sdev->dbg_info.chn.incld_mod == false)
		return sprintf(buf, "Exclude mode selected.\n");
	else
		return sprintf(buf, "Include mode selected.\n");
}

static ssize_t range_mod_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(dev);
	unsigned long sel, ret;

	ret = kstrtoul(buf, 0, &sel);
	if (ret)
		return -EINVAL;
	if (!sel) {
		pr_info("Exclude mode selected.\n");
		sdev->dbg_info.chn.incld_mod = false;
	} else {
		pr_info("Include mode selected.\n");
		sdev->dbg_info.chn.incld_mod = true;
	}
	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(range_mod);

static ssize_t rw_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(dev);

	switch (sdev->dbg_info.chn.flag) {
	case W_MODE:
		return sprintf(buf, "Write protect mode selected.\n");
	case R_MODE:
		return sprintf(buf, "Read protect mode selected.\n");
	case RW_MODE:
		return sprintf(buf, "Read/Write protect mode selected.\n");
	default:
		return sprintf(buf, "Protect mode not support!\n");
	}
}

static ssize_t rw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(dev);
	unsigned long sel, ret;

	ret = kstrtoul(buf, 0, &sel);
	if (ret)
		return -EINVAL;
	switch (sel) {
	case 0:
		pr_info("Write protect mode selected.\n");
		sdev->dbg_info.chn.flag = W_MODE;
		break;
	case 1:
		pr_info("Read protect mode selected.\n");
		sdev->dbg_info.chn.flag = R_MODE;
		break;
	case 2:
		pr_info("Read/Write protect mode selected.\n");
		sdev->dbg_info.chn.flag = RW_MODE;
		break;
	default:
		pr_info("Protect mode not support!\n");
		sdev->dbg_info.chn.flag = NONE_MODE;
	}
	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(rw);

static ssize_t cfg_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(dev);
	struct sprd_mpu_chn chn;
	int i, cnt = 0;

	for (i = 0; i < sdev->pub_chn * (sdev->interleaved_mod + 1); i++) {
		chn = sdev->chns[i];
		if (!chn.sticky_enable) {
			cnt += sprintf(buf + cnt,
				"PUB%d - chn%d: Closed\n",
				chn.pub,
				i%sdev->pub_chn);
			continue;
		}
		cnt += sprintf(buf + cnt,
			"PUB%d - chn%d:0x%08X ~ 0x%08X	",
			chn.pub,
			i%sdev->pub_chn,
			(chn.range_addr << 16) + MPU_ADDR_MASK,
			(chn.range_addr & MPU_ADDR_ALIGN) + MPU_ADDR_MASK);
		if (chn.incld_mod == true)
			cnt += sprintf(buf + cnt, "Include	");
		else
			cnt += sprintf(buf + cnt, "Exchule	");

		if (chn.flag == W_MODE)
			cnt += sprintf(buf + cnt, "W	");
		else if (chn.flag == R_MODE)
			cnt += sprintf(buf + cnt, "R	");
		else
			cnt += sprintf(buf + cnt, "RW	");
		cnt += sprintf(buf + cnt, "\n");
	}

	return cnt;
}

static ssize_t cfg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(dev);
	u32 pub, chn, addr, min_addr, max_addr, ret;

	ret = sscanf(buf, "%d %d %x", &pub, &chn, &addr);
	if (!ret)
		return -EINVAL;
	min_addr = addr << 16;
	max_addr = addr & MPU_ADDR_ALIGN;
	if (min_addr >= max_addr)
		return -EINVAL;

	sdev->dbg_info.chn.range_addr = addr;
	sdev->dbg_info.chn.pub = pub;
	sdev->dbg_info.chn.chn = chn;

	sprd_dmc_mpu_clk_enable(sdev, true);
	sprd_dmc_mpu_config(sdev, &sdev->dbg_info.chn);
	sprd_dmc_mpu_int_enable(sdev, true);

	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(cfg);

static ssize_t chn_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(dev);
	int chn, cnt = 0;

	for (chn = 0; chn < sdev->pub_chn; chn++)
		cnt += sprintf(buf + cnt,  "%d:	%s\n",
			chn, sprd_dmc_mpu_list[chn]);

	return cnt;
}
static DEVICE_ATTR_RO(chn);

static ssize_t occur_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(dev);
	struct sprd_mpu_dbg_info info = sdev->dbg_info;
	int cnt = 0, offset;

	cnt += sprintf(buf + cnt,
		"\n\
		PUB%d: chn%d:\n\
		dump: 0x%08X\n\
		raddr: 0x%08X - cmd: 0x%08X\n\
		waddr: 0x%08X - cmd: 0x%08X\n\
		dump data:\n",
		info.chn.pub,
		info.chn.chn,
		info.dump_addr,
		info.vio_raddr,
		info.vio_rcmd,
		info.vio_waddr,
		info.vio_wcmd);
	for (offset = 0; offset < 16; offset += 4) {
		cnt += sprintf(buf + cnt,
			"\
			0x%08X 0x%08X\n\
			0x%08X 0x%08X\n",
			info.dump_data[offset],
			info.dump_data[offset + 1],
			info.dump_data[offset + 2],
			info.dump_data[offset + 3]);
	}

	return cnt;
}
static DEVICE_ATTR_RO(occur);

static ssize_t panic_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(dev);

	if (sdev->dbg_info.chn.panic == 0)
		return sprintf(buf, "Panic mode closed.\n");
	else
		return sprintf(buf, "Panic mode open.\n");
}

static ssize_t panic_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(dev);
	unsigned long sel, ret;

	ret = kstrtoul(buf, 0, &sel);
	if (ret)
		return -EINVAL;
	switch (sel) {
	case 0:
		pr_info("Panic mode closed.\n");
		sdev->dbg_info.chn.panic = 0;
		break;
	case 1:
		pr_info("Panic mode open.\n");
		sdev->dbg_info.chn.panic = 1;
		break;
	default:
		pr_info("Enter wrong para!\n");
	}
	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(panic);

static struct attribute *dmc_mpu_attrs[] = {
	&dev_attr_pub.attr,
	&dev_attr_range_mod.attr,
	&dev_attr_rw.attr,
	&dev_attr_cfg.attr,
	&dev_attr_chn.attr,
	&dev_attr_occur.attr,
	&dev_attr_panic.attr,
	NULL,
};
static struct attribute_group dmc_mpu_group = {
	.attrs = dmc_mpu_attrs,
};

static int sprd_dmc_mpu_open(struct inode *inode, struct file *filp)
{
	pr_info("%s!\n", __func__);
	return 0;
}

static int sprd_dmc_mpu_release(struct inode *inode, struct file *filp)
{
	pr_info("%s!\n", __func__);
	return 0;
}

static long sprd_dmc_mpu_ioctl(struct file *filp, unsigned int cmd,
	unsigned long args)
{
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct sprd_dmc_mpu_dev *sdev = NULL;
	struct sprd_mpu_chn *chn_cfg = NULL;
	int i, cnt;

	pr_info("%s, cmd %d\n", __func__, cmd);
	cnt = sizeof(dmc_mpu_of_match) / sizeof(struct of_device_id);
	for (i = 0; i < cnt - 1; i++) {
		np = of_find_compatible_node(NULL, NULL,
				dmc_mpu_of_match[i].compatible);
		if (np)
			break;
	}
	if (!np)
		return -EINVAL;
	pdev = of_find_device_by_node(np);
	sdev = platform_get_drvdata(pdev);
	if (!sdev)
		return -EINVAL;

	switch (cmd) {
	case 0:
		sprd_dmc_mpu_chn_disable(sdev);
		sprd_dmc_mpu_disable(sdev);
		break;
	case 1:
		for (cnt = 0; cnt < sdev->pub_chn * (sdev->interleaved_mod + 1);
				cnt++) {
			chn_cfg = &sdev->chns[cnt];
			if (chn_cfg->sticky_enable) {
				sprd_dmc_mpu_clk_enable(sdev, true);
				sprd_dmc_mpu_config(sdev, chn_cfg);
				sprd_dmc_mpu_int_enable(sdev, true);
			}
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static long sprd_dmc_mpu_ioctl_compat(struct file *filp, unsigned int cmd,
	unsigned long args)
{
	return sprd_dmc_mpu_ioctl(filp, cmd, (unsigned long)compat_ptr(args));
}
#endif

static const struct file_operations sprd_dmc_mpu_fops = {
	.owner = THIS_MODULE,
	.open = sprd_dmc_mpu_open,
	.release = sprd_dmc_mpu_release,
	.unlocked_ioctl = sprd_dmc_mpu_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = sprd_dmc_mpu_ioctl_compat,
#endif
	.llseek = no_llseek,
};

static int sprd_dmc_mpu_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	return 0;
}

static int sprd_dmc_mpu_resume(struct platform_device *pdev)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(&pdev->dev);
	struct sprd_mpu_chn *chn_cfg;
	int chn;

	for (chn = 0; chn < sdev->pub_chn * (sdev->interleaved_mod + 1);
			chn++) {
		chn_cfg = &sdev->chns[chn];
		if (chn_cfg->sticky_enable) {
			sprd_dmc_mpu_clk_enable(sdev, true);
			sprd_dmc_mpu_config(sdev, chn_cfg);
			sprd_dmc_mpu_int_enable(sdev, true);
		}
	}

	return 0;
}

static int sprd_dmc_mpu_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	struct sprd_dmc_mpu_dev *sdev;
	struct sprd_mpu_chn *mpu_chn;
	struct device_node *res_mem;
	const struct chip_id_data *chip;
	void __iomem *mpu_pub0_base = NULL, *mpu_pub1_base = NULL;
	void __iomem *pub0_dump_addr_v = NULL, *pub1_dump_addr_v = NULL;
	uint64_t pub0_dump_addr_p = 0, pub1_dump_addr_p = 0;
	uint64_t dump_size = 0;
	uint32_t out_values[4];
	int pub0_irq = 0, pub1_irq = 0;
	int i, ret, interleaved_mode, pub_chn;

	if (sprd_dmc_mpu_iq) {
		pr_err("dmc_mpu:It is iq mode, do not need to probe!\n");
		return -ENODEV;
	}
	lock_of_id = of_match_node(dmc_mpu_of_match,
				   pdev->dev.of_node);
	chip = lock_of_id->data;
	this_offset = reg_offset + chip->id;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("Error: dmc mpu get io resource 0 failed!\n");
		return -ENODEV;
	}
	mpu_pub0_base = devm_ioremap_nocache(&pdev->dev,
					res->start, resource_size(res));
	if (!mpu_pub0_base) {
		pr_err("Error: mpu get pub0 base address failed!\n");
		return -ENOMEM;
	}

	/* irq request */
	pub0_irq = platform_get_irq(pdev, 0);
	if (pub0_irq < 0) {
		pr_err("Error: Can't get the pub0 mpu irq number!\n");
		return -ENXIO;
	}
	ret = devm_request_threaded_irq(&pdev->dev, pub0_irq,
			sprd_dmc_mpu_irq, NULL,
			IRQF_TRIGGER_NONE, "pub0_dmc_mpu ", pdev);
	if (ret) {
		pr_err("Error: Can't request pub1 mpu irq!\n");
		return ret;
	}

	/* get DDR interleaved mode */
	if (of_property_read_u32_index(pdev->dev.of_node,
				       "sprd,ddr-interleaved",
				       0, &interleaved_mode)) {
		pr_err("Error: Can't get pub0 mpu chns number!\n");
		return -EINVAL;
	}

	/* get pub0 dmc mpu channel number */
	if (of_property_read_u32_index(pdev->dev.of_node,
				       "sprd,ddr-chn",
				       0, &pub_chn)) {
		pr_err("Error: Can't get pub0 mpu chns number!\n");
		return -EINVAL;
	}
	/* get pub dmc mpu channel name */
	sprd_dmc_mpu_list = devm_kzalloc(&pdev->dev, pub_chn * sizeof(char *),
				GFP_KERNEL);
	for (i = 0; i < pub_chn; i++) {
		if (of_property_read_string_index(pdev->dev.of_node, "chn-name",
			i, sprd_dmc_mpu_list + i)) {
			pr_err("Error: Read Chn name\n");
			return -EINVAL;
		}
	}
	/* get reserved memory for dump reg */
	res_mem = of_parse_phandle(pdev->dev.of_node,
					"memory-region", 0);
	if (!res_mem) {
		pr_err("Error: Can't get memory region!\n");
		return -EINVAL;
	}
	ret = of_property_read_u32_array(res_mem, "reg",
			out_values, 4);
	if (!ret) {
		pub0_dump_addr_p = out_values[0];
		pub0_dump_addr_p <<= 32;
		pub0_dump_addr_p |= out_values[1];
		dump_size = out_values[3];
		pub0_dump_addr_v = __va(pub0_dump_addr_p);
		memset(pub0_dump_addr_v, 0x0, dump_size);
	} else {
		ret = of_property_read_u32_array(res_mem, "reg",
			out_values, 2);
		if (!ret) {
			pub0_dump_addr_p = out_values[0];
			dump_size = out_values[1];
			pub0_dump_addr_v = __va(pub0_dump_addr_p);
			memset(pub0_dump_addr_v, 0x0, dump_size);
		} else {
			pr_err("Failed to get out_values\n");
			return -EINVAL;
		}
	}

	/* there are 2 DDR, need to get pub1 para */
	if (interleaved_mode == 1) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (!res) {
			pr_err("Error: dmc mpu get io resource 1 failed!\n");
			return -ENODEV;
		}

		mpu_pub1_base = devm_ioremap_nocache(&pdev->dev,
			res->start, resource_size(res));
		if (!mpu_pub1_base) {
			pr_err("Error: mpu get pub1 base address failed!\n");
			return -ENOMEM;
		}

		/* irq request */
		pub1_irq = platform_get_irq(pdev, 1);
		if (pub1_irq < 0) {
			pr_err("Error: Can't get the pub0 mpu irq number!\n");
			return -ENXIO;
		}
		ret = devm_request_threaded_irq(&pdev->dev,
				pub1_irq,
				sprd_dmc_mpu_irq, NULL,
				IRQF_TRIGGER_NONE, "pub1_dmc_mpu ", pdev);
		if (ret) {
			pr_err("Error: Can't request pub1 mpu irq!\n");
			return ret;
		}

		/* get reserved memory for dump reg */
		res_mem = of_parse_phandle(pdev->dev.of_node,
						"memory-region", 1);
		if (!res_mem) {
			/* memory reserve is symmetrically */
			pub1_dump_addr_p = pub0_dump_addr_p;
		} else {
			ret = of_property_read_u32_array(res_mem, "reg",
					out_values, 4);
			if (!ret) {
				pub1_dump_addr_p = out_values[0];
				pub1_dump_addr_p <<= 32;
				pub1_dump_addr_p |= out_values[1];
				pub1_dump_addr_v = __va(pub1_dump_addr_p);
				memset(pub1_dump_addr_v, 0x0, dump_size);
			} else {
				ret = of_property_read_u32_array(res_mem, "reg",
					out_values, 2);
				if (!ret) {
					pub1_dump_addr_p = out_values[0];
					pub1_dump_addr_v =
						__va(pub1_dump_addr_p);
					memset(pub1_dump_addr_v,
						0x0, dump_size);
				} else {
					pr_err("Failed to get out_values\n");
					return -EINVAL;
				}
			}
		}
	}

	/* mpu device momery alloc */
	sdev = devm_kzalloc(&pdev->dev,
				(sizeof(*sdev) + (sizeof(struct sprd_mpu_chn) *
				(pub_chn * (interleaved_mode + 1)))),
			    GFP_KERNEL);
	if (!sdev)
		return -ENOMEM;

	/* initialize mpu chns */
	for (i = 0; i < pub_chn * (interleaved_mode + 1); i++) {
		mpu_chn = &sdev->chns[i];
		if (i < pub_chn) {
			mpu_chn->pub = 0;
			mpu_chn->chn = i;
		} else {
			mpu_chn->pub = 1;
			mpu_chn->chn = i - pub_chn;
		}
		mpu_chn->flag = NONE_MODE;
		mpu_chn->incld_mod = true;
	}

	sdev->pub_chn = pub_chn;
	sdev->pub0_irq = pub0_irq;
	sdev->pub1_irq = pub1_irq;
	sdev->interleaved_mod = interleaved_mode;
	sdev->mpu_pub0_base = mpu_pub0_base;
	sdev->mpu_pub1_base = mpu_pub1_base;
	sdev->pub0_dump_addr_p = pub0_dump_addr_p;
	sdev->pub1_dump_addr_p = pub1_dump_addr_p;
	sdev->pub0_dump_addr_v = pub0_dump_addr_v;
	sdev->pub1_dump_addr_v = pub1_dump_addr_v;
	sdev->dbg_info.chn.incld_mod = true;
	sdev->dbg_info.chn.panic = true;
	sdev->misc.name = "dmc_mpu";
	sdev->misc.parent = &pdev->dev;
	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	sdev->misc.fops = &sprd_dmc_mpu_fops;
	ret = misc_register(&sdev->misc);
	if (ret) {
		pr_err("Error: Unable to register mpu misc device!\n");
		return ret;
	}
	ret = sysfs_create_group(&sdev->misc.this_device->kobj, &dmc_mpu_group);
	if (ret) {
		pr_err("Error: Unable to export dmc mpu sysfs\n");
		misc_deregister(&sdev->misc);
		return ret;
	}
	/* save the sdev as private data */
	platform_set_drvdata(pdev, sdev);
	sprd_dmc_mpu_def_prot(sdev);
	pr_info("sprd dmc mpu probe done!\n");

	return 0;
}

static int sprd_dmc_mpu_remove(struct platform_device *pdev)
{
	struct sprd_dmc_mpu_dev *sdev = dev_get_drvdata(&pdev->dev);

	sysfs_remove_group(&sdev->misc.this_device->kobj, &dmc_mpu_group);
	misc_deregister(&sdev->misc);
	return 0;
}

static struct platform_driver sprd_dmc_mpu_driver = {
	.probe    = sprd_dmc_mpu_probe,
	.remove   = sprd_dmc_mpu_remove,
	.suspend  = sprd_dmc_mpu_suspend,
	.resume   = sprd_dmc_mpu_resume,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd_dmc_mpu",
		.of_match_table = dmc_mpu_of_match,
	},
};

static int __init sprd_dmc_mpu_init(void)
{
	return platform_driver_register(&sprd_dmc_mpu_driver);
}

static void __exit sprd_dmc_mpu_exit(void)
{
	platform_driver_unregister(&sprd_dmc_mpu_driver);
}

module_init(sprd_dmc_mpu_init);
module_exit(sprd_dmc_mpu_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eric Long<eric.long@spreadtrum.com>");
MODULE_AUTHOR("Aiden Cheng<aiden.cheng@spreadtrum.com>");
MODULE_DESCRIPTION("spreadtrum platform dmc mpu driver");
