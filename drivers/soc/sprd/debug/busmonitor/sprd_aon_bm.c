/*copyright (C) 2017 Spreadtrum Communications Inc.
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
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>

#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include "sprd_aon_bm.h"

static const struct of_device_id sprd_aon_bm_match[] = {
	{ .compatible = "sprd,aon-bm-r5p3"},
	{ },
};

static void sprd_djtag_delay(void)
{
	udelay(10);
}

static void sprd_djtag_init(struct sprd_aon_bm_dev *sdev)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;

	disable_irq_nosync(sdev->bm_irq);
	spin_lock(&sdev->bm_lock);
	regmap_update_bits(sdev->aon_apb, REG_AON_APB_APB_EB1,
		BIT_AON_APB_DJTAG_EB, BIT_AON_APB_DJTAG_EB);

	writel_relaxed(DJTAG_IR_LEN, (void __iomem *)&djtag_reg->ir_len_cfg);
	writel_relaxed(DJTAG_DR_LEN, (void __iomem *)&djtag_reg->dr_len_cfg);
	writel_relaxed(0x1, (void __iomem *)
			&djtag_reg->dap_mux_ctrl_rst);
	sprd_djtag_delay();
	writel_relaxed(0x0, (void __iomem *)
		&djtag_reg->dap_mux_ctrl_rst);
}

static void sprd_djtag_deinit(struct sprd_aon_bm_dev *sdev)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;

	writel_relaxed(0x1, (void __iomem *)&djtag_reg->dap_mux_ctrl_rst);
	sprd_djtag_delay();
	writel_relaxed(0x0, (void __iomem *)&djtag_reg->dap_mux_ctrl_rst);
	regmap_update_bits(sdev->aon_apb, REG_AON_APB_APB_EB1,
		BIT_AON_APB_DJTAG_EB, 0);
	spin_unlock(&sdev->bm_lock);
	enable_irq(sdev->bm_irq);
}

static u32 sprd_djtag_busmon_write(struct sprd_aon_bm_dev *sdev,
	u32 ir, u32 dr)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;
	u32 ir_write = ir | BIT_DJTAG_CHAIN_UPDATE_T;

	writel_relaxed(ir_write, (void __iomem *)&djtag_reg->ir_cfg);
	writel_relaxed(dr, (void __iomem *)&djtag_reg->dr_cfg);
	writel_relaxed(0x1, (void __iomem *)&djtag_reg->rnd_en_cfg);
	sprd_djtag_delay();
	writel_relaxed(0x0, (void __iomem *)&djtag_reg->rnd_en_cfg);

	return readl_relaxed((void __iomem *)&djtag_reg->upd_dr_cfg);
}

static u32 sprd_djtag_busmon_read(struct sprd_aon_bm_dev *sdev, u32 ir)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;
	u32 dr_value = 0;

	writel_relaxed(ir, (void __iomem *)&djtag_reg->ir_cfg);
	writel_relaxed(dr_value, (void __iomem *)&djtag_reg->dr_cfg);
	writel_relaxed(0x1, (void __iomem *)&djtag_reg->rnd_en_cfg);
	sprd_djtag_delay();
	writel_relaxed(0x0, (void __iomem *)&djtag_reg->rnd_en_cfg);
	return readl_relaxed((void __iomem *)&djtag_reg->upd_dr_cfg);
}

static void sprd_djtag_mux_sel(struct sprd_aon_bm_dev *sdev,
	u32 mux, u32 dap)
{
	struct sprd_djtag_reg *djtag_reg = sdev->djtag_reg_base;
	u32 mux_value = 0;

	mux_value = mux << DJTAG_DAP_OFFSET;
	mux_value |= dap;
	writel_relaxed(DJTAG_DAP_MUX_RESET, (void __iomem *)&djtag_reg->ir_cfg);
	writel_relaxed(mux_value, (void __iomem *)&djtag_reg->dr_cfg);
	writel_relaxed(0x1, (void __iomem *)&djtag_reg->rnd_en_cfg);
	sprd_djtag_delay();
	writel_relaxed(0x0, (void __iomem *)&djtag_reg->rnd_en_cfg);
}

static ssize_t panic_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct sprd_aon_bm_dev *sdev = dev_get_drvdata(dev->parent);

	if (sdev->panic == false)
		return sprintf(buf, "Panic mode closed.\n");
	else
		return sprintf(buf, "Panic mode open.\n");
}

static ssize_t panic_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sprd_aon_bm_dev *sdev = dev_get_drvdata(dev->parent);
	unsigned long sel, ret;

	ret = kstrtoul(buf, 0, &sel);
	if (ret)
		return -EINVAL;
	switch (sel) {
	case 0:
		dev_info(dev, "Panic mode closed.\n");
		sdev->panic = false;
		break;
	case 1:
		dev_info(dev, "Panic mode open.\n");
		sdev->panic = true;
		break;
	default:
		dev_info(dev, "Enter wrong para!\n");
	}
	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(panic);

static ssize_t chn_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	struct sprd_aon_bm_dev *sdev = dev_get_drvdata(dev->parent);
	int chn, cnt = 0;

	for (chn = 0; chn < sdev->bm_cnt; chn++)
		cnt += sprintf(buf + cnt,  "%d:	%s\n",
			chn, sdev->name_list[chn]);

	return cnt;
}
static DEVICE_ATTR_RO(chn);

static ssize_t dbg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sprd_aon_bm_dev *sdev = dev_get_drvdata(dev->parent);
	char *chn_info;
	char *info_buf;
	u32 bm_index;
	int cnt;

	/* aon bm info momery alloc */
	chn_info = kzalloc(TEMP_BUF, GFP_KERNEL);
	if (!chn_info)
		return -ENOMEM;
	info_buf = kzalloc(TEMP_BUF * sdev->bm_cnt, GFP_KERNEL);
	if (!info_buf) {
		kfree(chn_info);
		return -ENOMEM;
	}

	for (bm_index = 0; bm_index < sdev->bm_cnt; bm_index++) {
		sprd_djtag_init(sdev);
		sprd_djtag_mux_sel(sdev, sdev->subsys_aon,
		sdev->bm_dap[bm_index]);
		sprintf(chn_info,
			"%d:0x%08X%08X~0x%08X%08X userid:0x%08lX mask: 0x%X_%X%s\n",
			bm_index,
			sprd_djtag_busmon_read(sdev, AXI_ADDR_MIN_H32),
			sprd_djtag_busmon_read(sdev, AXI_ADDR_MIN),
			sprd_djtag_busmon_read(sdev, AXI_ADDR_MAX_H32),
			sprd_djtag_busmon_read(sdev, AXI_ADDR_MAX),
			sprd_djtag_busmon_read(sdev, AXI_USER_CFG)
				& USER_CFG_USERID_MASK,
			sprd_djtag_busmon_read(sdev, AXI_ADDR_MASK_H32),
			sprd_djtag_busmon_read(sdev, AXI_ADDR_MASK),
			sdev->name_list[bm_index]);
		strcat(info_buf, chn_info);
		sprd_djtag_deinit(sdev);
	}

	if (info_buf[0] == 0) {
		kfree(info_buf);
		kfree(chn_info);
		return sprintf(buf, ":-) ! No action was monitored by BM!!!\n");
	}

	cnt = sprintf(buf, "%s", info_buf);
	kfree(info_buf);
	kfree(chn_info);

	return cnt;
}

static ssize_t occur_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct sprd_aon_bm_dev *sdev = dev_get_drvdata(dev->parent);
	ssize_t cnt;

	if (sdev->dbg_info.bm_chn == 0xff)
		return sprintf(buf, ":-) ! No action was monitored by BM!!!\n");
	cnt = sprintf(buf, "aon bm get overlap:\nBM CHN:	%d\n",
				sdev->dbg_info.bm_chn);
	cnt += sprintf(buf + cnt, "MONITOR START:	0x%08X%08X\n",
				sdev->dbg_info.mon_min_h32,
				sdev->dbg_info.mon_min_l32);
	cnt += sprintf(buf + cnt, "MONITOR END:	0x%08X%08X\n",
				sdev->dbg_info.mon_max_h32,
				sdev->dbg_info.mon_max_l32);
	cnt += sprintf(buf + cnt, "Overlap ADDR:	0x%08X%08X\n",
				sdev->dbg_info.match_addr_h32,
				sdev->dbg_info.match_addr_l32);
	cnt += sprintf(buf + cnt, "Overlap ID:	%X\n",
				sdev->dbg_info.bm_match_id);
	cnt += sprintf(buf + cnt, "Overlap USERID:	%X\n",
				sdev->dbg_info.match_userid);

	return cnt;
}
static DEVICE_ATTR_RO(occur);

static ssize_t dbg_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct sprd_aon_bm_dev *sdev = dev_get_drvdata(dev->parent);
	unsigned char chn[8], start[20], end[20], mod[3], user[20];
	unsigned long start_addr, end_addr, channel, userid;
	u32 rd_wt, ret;

	ret = sscanf(buf, "%s %s %s %s %s",
		chn, start, end, mod, user);
	if (ret < 5) {
		dev_err(dev, "format: chn start end mod userid.\n"
			"example: ap or 0 0x1 0x2 w 0x2\n"
			"if userid is 0xff monitor all userid\n");
		return -EINVAL;
	}
	ret = kstrtoul(user, 0, &userid);
	if (ret) {
		dev_err(dev, "userid is not in hex or decimal form.\n");
		return -EINVAL;
	}
	if (chn[0] >= '0' && chn[0] <= '9') {
		ret = kstrtoul(chn, 0, &channel);
		if (ret)
			dev_err(dev, "chn %s is not num.\n", chn);
		if (channel >= sdev->bm_cnt)
			dev_err(dev, "the BM channel is too big\n");
	} else {
		dev_err(dev, "please input a channel number! as 0-9,ap\n");
		return -EINVAL;
	}

	/* get the monitor start and end address */
	ret = kstrtoul(start, 0, &start_addr);
	if (ret) {
		dev_err(dev, "start is not in hex or decimal form.\n");
		return -EINVAL;
	}
	ret = kstrtoul(end, 0, &end_addr);
	if (ret) {
		dev_err(dev, "end is not in hex or decimal form.\n");
		return -EINVAL;
	}
	/* get the monitor action */
	if (strcmp(mod, "r") == 0 || strcmp(mod, "R") == 0)
		rd_wt = BM_READ;
	else if (strcmp(mod, "w") == 0 || strcmp(mod, "W") == 0)
		rd_wt = BM_WRITE;
	else {
		dev_err(dev, "please input a legal channel mode! e.g: r,w\n");
		return -EINVAL;
	}
	dev_info(dev, "str addr 0x%lx; end addr 0x%lx; chn %s; rw %d\n",
		start_addr, end_addr, chn, rd_wt);
	if (channel >= sdev->bm_cnt ||
		rd_wt > BM_WRITE || start_addr > end_addr)
		return -EINVAL;

	/* set subsys BM config */
	sprd_djtag_init(sdev);
	sprd_djtag_mux_sel(sdev, sdev->subsys_aon,
		sdev->bm_dap[channel]);
	sprd_djtag_busmon_write(sdev, AXI_CHN_INT, BM_INT_CLR);

	if (rd_wt == BM_WRITE)
		sprd_djtag_busmon_write(sdev, AXI_CHN_CFG,
		BM_WRITE_EN | BM_WRITE_CFG);
	else
		sprd_djtag_busmon_write(sdev, AXI_CHN_CFG, BM_WRITE_EN);
	sprd_djtag_busmon_write(sdev, AXI_ADDR_MIN,
		start_addr & MAX_DATA_VALUE);
	sprd_djtag_busmon_write(sdev, AXI_ADDR_MIN_H32,
		(u64)start_addr >> 32);
	sprd_djtag_busmon_write(sdev, AXI_ADDR_MAX,
		end_addr & MAX_DATA_VALUE);
	sprd_djtag_busmon_write(sdev, AXI_ADDR_MAX_H32,
		(u64)end_addr >> 32);
	sprd_djtag_busmon_write(sdev, AXI_ADDR_MASK, 0);
	sprd_djtag_busmon_write(sdev, AXI_ADDR_MASK_H32, 0);
	sprd_djtag_busmon_write(sdev, AXI_DATA_MIN_L32, MAX_DATA_VALUE);
	sprd_djtag_busmon_write(sdev, AXI_DATA_MIN_H32, 0x0);
	sprd_djtag_busmon_write(sdev, AXI_DATA_MIN_EXT_L32, MAX_DATA_VALUE);
	sprd_djtag_busmon_write(sdev, AXI_DATA_MIN_EXT_H32, 0x0);
	sprd_djtag_busmon_write(sdev, AXI_DATA_MAX_L32, 0x0);
	sprd_djtag_busmon_write(sdev, AXI_DATA_MAX_H32, 0x0);
	sprd_djtag_busmon_write(sdev, AXI_DATA_MAX_EXT_L32, 0x0);
	sprd_djtag_busmon_write(sdev, AXI_DATA_MAX_EXT_H32, 0x0);
	sprd_djtag_busmon_write(sdev, AXI_DATA_MASK_L32, 0x0);
	sprd_djtag_busmon_write(sdev, AXI_DATA_MASK_H32, 0x0);
	sprd_djtag_busmon_write(sdev, AXI_DATA_MASK_EXT_L32, 0x0);
	sprd_djtag_busmon_write(sdev, AXI_DATA_MASK_EXT_H32, 0x0);
	sprd_djtag_busmon_write(sdev, AXI_USER_CFG,
				userid | USER_CFG_ENUSERID);
	sprd_djtag_busmon_write(sdev, AXI_CHN_INT,
		BM_INT_CLR | BM_INT_EN | BM_CHN_EN);
	sprd_djtag_deinit(sdev);

	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(dbg);

static struct attribute *aon_bm_attrs[] = {
	&dev_attr_panic.attr,
	&dev_attr_chn.attr,
	&dev_attr_occur.attr,
	&dev_attr_dbg.attr,
	NULL,
};
static struct attribute_group aon_bm_group = {
	.attrs = aon_bm_attrs,
};

static irqreturn_t sprd_aon_bm_isr(int irq_num, void *dev)
{
	struct sprd_aon_bm_dev *sdev = dev;
	int index;

	for (index = 0; index < sdev->bm_cnt; index++) {
		sprd_djtag_init(sdev);
		sprd_djtag_mux_sel(sdev, sdev->subsys_aon,
			sdev->bm_dap[index]);
		if (sprd_djtag_busmon_read(sdev, AXI_CHN_INT) &
			BM_INT_MASK_STATUS) {
			sdev->dbg_info.bm_chn = index;
			sdev->dbg_info.mon_min_h32 =
				sprd_djtag_busmon_read(sdev, AXI_ADDR_MIN_H32);
			sdev->dbg_info.mon_min_l32 =
				sprd_djtag_busmon_read(sdev, AXI_ADDR_MIN);
			sdev->dbg_info.mon_max_h32 =
				sprd_djtag_busmon_read(sdev, AXI_ADDR_MAX_H32);
			sdev->dbg_info.mon_max_l32 =
				sprd_djtag_busmon_read(sdev, AXI_ADDR_MAX);
			sdev->dbg_info.match_addr_h32 =
				sprd_djtag_busmon_read(sdev,
					AXI_MATCH_ADDR_H32);
			sdev->dbg_info.match_addr_l32 =
				sprd_djtag_busmon_read(sdev, AXI_MATCH_ADDR);
			sdev->dbg_info.bm_match_id =
				sprd_djtag_busmon_read(sdev, AXI_MATCH_ID);
			sdev->dbg_info.match_userid =
				sprd_djtag_busmon_read(sdev, AXI_MATCH_USERID)
				& USER_CFG_USERID_MASK;
			pr_emerg("aon bm get overlap:\nBM CHN:	%d\n",
				sdev->dbg_info.bm_chn);
			pr_emerg("MONITOR START:	0x%08X%08X\n",
				sdev->dbg_info.mon_min_h32,
				sdev->dbg_info.mon_min_l32);
			pr_emerg("MONITOR END:	0x%08X%08X\n",
				sdev->dbg_info.mon_max_h32,
				sdev->dbg_info.mon_max_l32);
			pr_emerg("Overlap ADDR:	0x%08X%08X\n",
				sdev->dbg_info.match_addr_h32,
				sdev->dbg_info.match_addr_l32);
			pr_emerg("Overlap ID:	%X\n",
				sdev->dbg_info.bm_match_id);
			pr_emerg("Overlap USERID:	%X\n",
				sdev->dbg_info.match_userid);
			sprd_djtag_busmon_write(sdev, AXI_CHN_INT, 0);
			sprd_djtag_busmon_write(sdev, AXI_CHN_INT,
				BM_INT_CLR | BM_INT_EN | BM_CHN_EN);
			if (sdev->panic) {
				pr_emerg("Aon Bus Monitor enter panic!\n");
				BUG();
			}
		}
		sprd_djtag_deinit(sdev);
	}

	return IRQ_HANDLED;
}

static int sprd_aon_bm_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct sprd_aon_bm_dev *sdev;
	int bm_cnt, i, ret;

	bm_cnt = of_property_count_strings(pdev->dev.of_node, "bm-names");
	if (bm_cnt <= 0) {
		dev_err(&pdev->dev, "Error: sprd aon bm no find bm-names!\n");
		return -EINVAL;
	}
	sdev = devm_kzalloc(&pdev->dev, sizeof(*sdev),
				GFP_KERNEL);
	if (!sdev)
		return -ENOMEM;
	sdev->name_list = devm_kzalloc(&pdev->dev, bm_cnt * sizeof(char *),
				GFP_KERNEL);
	if (!sdev->name_list)
		return -ENOMEM;
	sdev->bm_dap = devm_kzalloc(&pdev->dev, bm_cnt * sizeof(u32),
				GFP_KERNEL);
	if (!sdev->bm_dap)
		return -ENOMEM;
	sdev->bm_cnt = bm_cnt;
	for (i = 0; i < sdev->bm_cnt; i++) {
		if (of_property_read_string_index(pdev->dev.of_node, "bm-names",
			i, sdev->name_list + i)) {
			dev_err(&pdev->dev, "Error: read bm name!\n");
			return -EINVAL;
		}
	}
	if (of_property_read_u32_array(pdev->dev.of_node, "bm-daps",
		sdev->bm_dap, sdev->bm_cnt)) {
		dev_err(&pdev->dev, "Error: read bm dap!\n");
		return -EINVAL;
	}
	if (of_property_read_u32_index(pdev->dev.of_node,
				       "aon_dap_index",
				       0, &sdev->subsys_aon)) {
		dev_err(&pdev->dev, "Error: can't get aon dap index!\n");
		return -EINVAL;
	}
	sdev->aon_apb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
				"sprd,aon-glb");
	if (IS_ERR(sdev->aon_apb)) {
		dev_err(&pdev->dev, "Error: can't get aon_apb regmap!\n");
		return -ENODEV;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Error:aon bm get io resource failed!\n");
		return -ENOMEM;
	}
	sdev->djtag_reg_base = devm_ioremap_nocache(&pdev->dev,
		res->start, resource_size(res));
	if (!sdev->djtag_reg_base) {
		dev_err(&pdev->dev, "sprd djtag get base address failed!\n");
		return -ENODEV;
	}

	sdev->bm_irq = platform_get_irq(pdev, 0);
	if (sdev->bm_irq <= 0) {
		dev_err(&pdev->dev, "Can't get the djtag irq number!\n");
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, sdev->bm_irq,
			sprd_aon_bm_isr, NULL,
			IRQF_TRIGGER_NONE, "aon_bm", pdev);
	if (ret) {
		dev_err(&pdev->dev, "Error: Can't request aon bm irq!\n");
		return ret;
	}

	sdev->panic = true;
	sdev->dbg_info.bm_chn = 0xff;
	sdev->misc.name = "aon_bm";
	sdev->misc.parent = &pdev->dev;
	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	spin_lock_init(&sdev->bm_lock);
	ret = misc_register(&sdev->misc);
	if (ret) {
		pr_err("Error: Unable to register aon bm device!\n");
		return ret;
	}
	ret = sysfs_create_group(&sdev->misc.this_device->kobj, &aon_bm_group);
	if (ret) {
		dev_err(&pdev->dev, "Error: Unable to export aon bm sysfs\n");
		misc_deregister(&sdev->misc);
		return ret;
	}
	/* save the sdev as private data */
	platform_set_drvdata(pdev, sdev);
	dev_info(&pdev->dev, "sprd aon bm probe done!\n");

	return 0;
}

static int sprd_aon_bm_remove(struct platform_device *pdev)
{
	struct sprd_aon_bm_dev *sdev = dev_get_drvdata(&pdev->dev);

	sysfs_remove_group(&sdev->misc.this_device->kobj, &aon_bm_group);
	misc_deregister(&sdev->misc);
	return 0;
}

static struct platform_driver sprd_aon_bm_driver = {
	.probe    = sprd_aon_bm_probe,
	.remove    = sprd_aon_bm_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd-aon-bm",
		.of_match_table = sprd_aon_bm_match,
	},
};

module_platform_driver(sprd_aon_bm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aiden Cheng<aiden.cheng@spreadtrum.com>");
MODULE_DESCRIPTION("spreadtrum platform aon bm driver");

