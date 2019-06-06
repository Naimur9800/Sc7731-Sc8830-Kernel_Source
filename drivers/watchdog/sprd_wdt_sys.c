 /* Watchdog driver for the SC9001
  *
  * Copyright (C) 2015 Spreadtrum
  *
  * based on sa1100_wdt.c and whale arch reset implementation.
  *
  * This software is licensed under the terms of the GNU General Public
  * License version 2, as published by the Free Software Foundation, and
  * may be copied, distributed, and modified under those terms.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/timex.h>
#include <linux/syscore_ops.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <asm/cacheflush.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/regmap.h>
#include <linux/clk.h>

#define WDT_LOAD_LOW		0x00
#define WDT_LOAD_HIGH		0x04
#define WDT_CTRL		0x08
#define WDT_INT_CLR		0x0C
#define WDT_INT_RAW		0x10
#define WDT_INT_MSK		0x14
#define WDT_CNT_LOW		0x18
#define WDT_CNT_HIGH		0x1C
#define WDT_LOCK		0x20
#define WDT_IRQ_LOAD_LOW	0x2c
#define WDT_IRQ_LOAD_HIGH	0x30

#define WDT_ANA_RST_STATUS	ANA_REG_GLB_POR_RST_MONITOR
#define WDT_ANA_AGEN		ANA_REG_GLB_ARM_MODULE_EN
#define WDT_ANA_RTC_CLK_EN	ANA_REG_GLB_RTC_CLK_EN
#define WDT_AGEN_WDG_EN		BIT_ANA_WDG_EN
#define WDT_AGEN_RTC_WDG_EN	BIT_RTC_WDG_EN

#ifndef BIT_AON_APB_APCPU_WDG_EB
#define BIT_AON_APB_APCPU_WDG_EB	BIT_AON_APB_AP_WDG_EB
#endif
#ifndef BIT_AON_APB_APCPU_WDG_RTC_EB
#define BIT_AON_APB_APCPU_WDG_RTC_EB	BIT_AON_APB_AP_WDG_RTC_EB
#endif

#define WDT_CLK			32768
#define WDT_UNLOCK_KEY		0xE551
#define WDT_INT_EN_BIT		BIT(0)
#define WDT_CNT_EN_BIT		BIT(1)
#define WDT_NEW_VER_EN		BIT(2)
#define WDT_RST_EN_BIT		BIT(3)
#define WDT_LD_BUSY_BIT		BIT(4)
#define WDT_INT_CLEAR_BIT	BIT(0)

#define WDT_LOAD_TIMEOUT_NUM	(10000)
#define HWRST_STATUS_NORMAL2	(0x00f0)


struct sprd_wdt_data {
	void (*wdt_enable)(void);
	void (*wdt_kick)(void);
	void (*wdt_start)(void);
	void (*wdt_stop)(void);
};

struct sprd_wdt {
	void __iomem *reg;
	unsigned int pmic_reg;
	unsigned int irq;
	int wdt_enabled;
	struct task_struct *feed_task;
	struct regmap *wdt_gpr;
	struct regmap *pmic_wdt_gpr;
	struct dentry *debugfs;
	struct sprd_wdt_data *wdt_ops;
	struct clk *enable;
	struct clk *rtc_enable;
} sprd_wdt;

int wdt_flag = 0;

int irq_timeout = 9;
int reset_timeout = 12;
static int pmic_timeout = 15;
static int feed_period = 3;
static int wdt_dt_support = 1;
static unsigned long enabled = 1;
module_param(irq_timeout, int, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(irq_timeout,
		 "SPRD Watchdog IRQ timeout in seconds (default 9s)");
module_param(reset_timeout, int, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(reset_timeout,
		 "SPRD Watchdog reset timeout in seconds (default 10s)");
MODULE_PARM_DESC(feed_period, "Watchdog feed period in seconds (default 3s)");
module_param(pmic_timeout, int, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(pmic_timeout, "Chip Watchdog margin in seconds (default 15s)");

#if 0
extern int in_calibration(void);
#endif

void sysdump_enable_watchdog(int on);

static irqreturn_t sprd_wdt_isr(int irq, void *dev_id)
{
	pr_info("watchdog timeout interrupt happen\n");

	writel_relaxed(WDT_INT_CLEAR_BIT, sprd_wdt.reg + WDT_INT_CLR);
	handle_sysrq('m');
	handle_sysrq('w');
	pr_info("Current PID %d is %s\n", task_pid_nr(current), current->comm);
	panic("watchdog timeout interrupt happen\n");
	return IRQ_HANDLED;
}

static void ap_wdt_load_value(unsigned int rst_value, unsigned int irq_value)
{
	unsigned int cnt = 0;

	writel_relaxed((uint16_t) (((rst_value) >> 16) & 0xffff),
		       sprd_wdt.reg + WDT_LOAD_HIGH);
	writel_relaxed((uint16_t) ((rst_value) & 0xffff),
		       sprd_wdt.reg + WDT_LOAD_LOW);

	writel_relaxed((uint16_t) (((irq_value) >> 16) & 0xffff),
		       sprd_wdt.reg + WDT_IRQ_LOAD_HIGH);
	writel_relaxed((uint16_t) ((irq_value) & 0xffff),
		       sprd_wdt.reg + WDT_IRQ_LOAD_LOW);

	while ((readl_relaxed(sprd_wdt.reg + WDT_INT_RAW) & WDT_LD_BUSY_BIT)
	       && (cnt < WDT_LOAD_TIMEOUT_NUM))
		cnt++;
}

static void ap_wdt_start(void)
{
	u32 val = 0;

	ap_wdt_load_value(reset_timeout * WDT_CLK,
			  (reset_timeout - irq_timeout) * WDT_CLK);

	val = readl_relaxed(sprd_wdt.reg + WDT_CTRL);
	val |= WDT_CNT_EN_BIT | WDT_INT_EN_BIT | WDT_RST_EN_BIT;
	writel_relaxed(val, sprd_wdt.reg + WDT_CTRL);
}

static void ap_wdt_stop(void)
{
	u32 val = 0;

	val = readl_relaxed(sprd_wdt.reg + WDT_CTRL);
	val &= ~(WDT_CNT_EN_BIT | WDT_RST_EN_BIT | WDT_INT_EN_BIT);
	writel_relaxed(val, sprd_wdt.reg + WDT_CTRL);
}

static void ap_wdt_kick(void)
{
	ap_wdt_load_value(reset_timeout * WDT_CLK,
			  (reset_timeout - irq_timeout) * WDT_CLK);
}

static void ap_wdt_enable(void)
{
	int val;
	if (wdt_dt_support) {
		clk_prepare_enable(sprd_wdt.enable);
		clk_prepare_enable(sprd_wdt.rtc_enable);
	} else {
		regmap_update_bits(sprd_wdt.wdt_gpr, REG_AON_APB_APB_EB1,
					BIT_AON_APB_APCPU_WDG_EB,
					BIT_AON_APB_APCPU_WDG_EB);
		regmap_update_bits(sprd_wdt.wdt_gpr, REG_AON_APB_APB_RTC_EB,
					BIT_AON_APB_APCPU_WDG_RTC_EB,
					BIT_AON_APB_APCPU_WDG_RTC_EB);
	}
	writel_relaxed(WDT_UNLOCK_KEY, sprd_wdt.reg + WDT_LOCK);
	val = readl_relaxed(sprd_wdt.reg + WDT_CTRL);
	val |= WDT_NEW_VER_EN;
	writel_relaxed(val, sprd_wdt.reg + WDT_CTRL);
}

static void pmic_load_timer_value(unsigned int value)
{
	regmap_write(sprd_wdt.pmic_wdt_gpr, sprd_wdt.pmic_reg + WDT_LOAD_HIGH,
		     (uint16_t) (((value) >> 16) & 0xffff));
	regmap_write(sprd_wdt.pmic_wdt_gpr, sprd_wdt.pmic_reg + WDT_LOAD_LOW,
		     (uint16_t) ((value) & 0xffff));
}

static void pmic_wdt_enable(void)
{
	regmap_update_bits(sprd_wdt.pmic_wdt_gpr, WDT_ANA_AGEN, WDT_AGEN_WDG_EN,
			   WDT_AGEN_WDG_EN);

	regmap_update_bits(sprd_wdt.pmic_wdt_gpr, WDT_ANA_RTC_CLK_EN,
			   WDT_AGEN_RTC_WDG_EN, WDT_AGEN_RTC_WDG_EN);
	regmap_write(sprd_wdt.pmic_wdt_gpr, sprd_wdt.pmic_reg + WDT_LOCK,
		     WDT_UNLOCK_KEY);
	regmap_update_bits(sprd_wdt.pmic_wdt_gpr, sprd_wdt.pmic_reg + WDT_CTRL,
			   WDT_NEW_VER_EN, WDT_NEW_VER_EN);
}

static void pmic_wdt_stop(void)
{
	int msk = WDT_CNT_EN_BIT | WDT_RST_EN_BIT;

	regmap_update_bits(sprd_wdt.pmic_wdt_gpr, sprd_wdt.pmic_reg + WDT_CTRL,
			   msk, 0);
}

static void pmic_wdt_start(void)
{
	int val = WDT_CNT_EN_BIT | WDT_RST_EN_BIT;

	pmic_load_timer_value(pmic_timeout * WDT_CLK);
	regmap_update_bits(sprd_wdt.pmic_wdt_gpr, sprd_wdt.pmic_reg + WDT_CTRL,
			   val, val);
}

static void pmic_wdt_kick(void)
{
	pmic_load_timer_value(pmic_timeout * WDT_CLK);
}

static void sprd_wdt_start_v1(void)
{
	pmic_wdt_start();
	ap_wdt_start();
}

static void sprd_wdt_stop_v1(void)
{
	ap_wdt_stop();
	pmic_wdt_stop();
}

static void sprd_wdt_kick_v1(void)
{
	pmic_wdt_kick();
	ap_wdt_kick();
}

static void sprd_wdt_enable_v1(void)
{
	pmic_wdt_enable();
	ap_wdt_enable();
}

static void sprd_wdt_start_v2(void)
{
	ap_wdt_start();
}

static void sprd_wdt_stop_v2(void)
{
	ap_wdt_stop();
}

static void sprd_wdt_kick_v2(void)
{
	ap_wdt_kick();
}

static void sprd_wdt_enable_v2(void)
{
	ap_wdt_enable();
}

static const struct sprd_wdt_data sprd_wdt_data_v1 = {
	.wdt_enable = sprd_wdt_enable_v1,
	.wdt_kick = sprd_wdt_kick_v1,
	.wdt_start = sprd_wdt_start_v1,
	.wdt_stop = sprd_wdt_stop_v1,
};

static const struct sprd_wdt_data sprd_wdt_data_v2 = {
	.wdt_enable = sprd_wdt_enable_v2,
	.wdt_kick = sprd_wdt_kick_v2,
	.wdt_start = sprd_wdt_start_v2,
	.wdt_stop = sprd_wdt_stop_v2,
};

void sprd_wdt_enable(void)
{
	sprd_wdt.wdt_ops->wdt_enable();
}

void sprd_wdt_kick(void)
{
	sprd_wdt.wdt_ops->wdt_kick();
}

void sprd_wdt_start(void)
{
	sprd_wdt.wdt_ops->wdt_start();
}

void sprd_wdt_stop(void)
{
	sprd_wdt.wdt_ops->wdt_stop();
}

static int sprd_wdt_feeder(void *data)
{
	do {
		if (kthread_should_stop())
			break;

		if (sprd_wdt.wdt_enabled)
			sprd_wdt_kick();

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(feed_period * HZ);
	} while (1);

	return 0;
}

static void sprd_wdt_feeder_init(void)
{
	int cpu = 0;

	do {
		sprd_wdt.feed_task = kthread_create_on_node(sprd_wdt_feeder,
							    NULL,
							    cpu_to_node(cpu),
							    "watchdog_feeder/%d",
							    cpu);

		kthread_bind(sprd_wdt.feed_task, cpu);
	} while (0);

	if (IS_ERR(sprd_wdt.feed_task)) {
		pr_err("Can't crate watchdog_feeder thread!\n");
	} else {
		sprd_wdt.wdt_enabled = 1;
		sprd_wdt_start();
		wake_up_process(sprd_wdt.feed_task);
	}

	pr_info("sprd watchdog:irq %d,rst %d,chip %d,feed %d\n", irq_timeout,
		reset_timeout, pmic_timeout, feed_period);
}

int param_set_enabled(const char *val, struct kernel_param *kp)
{
	int ret;

	if (!wdt_flag)
		return 0;

	ret = param_set_ulong(val, kp);
	if (ret < 0)
		return ret;

	if (enabled) {
		sprd_wdt.wdt_enabled = 1;
		sprd_wdt_start();
	} else {
		sprd_wdt.wdt_enabled = 0;
		sprd_wdt_stop();
	}
	wake_up_process(sprd_wdt.feed_task);

	return ret;
}

module_param_call(enabled, param_set_enabled, param_get_ulong, &enabled, 0664);
MODULE_PARM_DESC(enabled, "Enable kernel watchdog task to feed dog");

static char cmd_buf[64];
static int sprd_wdt_read(struct seq_file *s, void *v)
{
	seq_printf(s, "irq %d, rst %d, feed %d, chip %d enable %d\n",
		   irq_timeout, reset_timeout, feed_period, pmic_timeout,
		   sprd_wdt.wdt_enabled);

	return 0;
}

static int sprd_wdt_open(struct inode *inode, struct file *file)
{
	return single_open(file, sprd_wdt_read, NULL);
}

static ssize_t sprd_wdt_write(struct file *file, const char __user *buf,
			      size_t count, loff_t * data)
{
	int ret;
	int irq_time, reset_time, feed_time, chip_time, en;

	if (count == 0)
		return -1;

	if (count > 255)
		count = 255;

	ret = copy_from_user(cmd_buf, buf, count);
	if (ret < 0)
		return -1;

	cmd_buf[count] = '\0';
	ret =
	    sscanf(cmd_buf, "%d %d %d %d %d", &irq_time, &reset_time,
		   &feed_time, &chip_time, &en);

	if (reset_time > chip_time)
		return -1;

	if (irq_time > reset_time)
		return -1;

	if ((irq_time < feed_time) || reset_time < feed_time)
		return -1;

	reset_timeout = reset_time;
	irq_timeout = irq_time;
	feed_period = feed_time;
	pmic_timeout = chip_time;

	if (1 == en) {
		sprd_wdt.wdt_enabled = 1;
		sprd_wdt_start();
	}

	if (0 == en) {
		sprd_wdt.wdt_enabled = 0;
		sprd_wdt_stop();
	}

	return count;
}

static const struct file_operations sprd_proc_fops = {
	.owner = THIS_MODULE,
	.open = sprd_wdt_open,
	.read = seq_read,
	.write = sprd_wdt_write,
	.llseek = seq_lseek,
	.release = single_release,
};

void sysdump_enable_watchdog(int on)
{
	if (on) {
		sprd_wdt.wdt_enabled = 1;
		sprd_wdt_start();
	} else {
		sprd_wdt.wdt_enabled = 0;
		sprd_wdt_stop();
	}
}

#define sprd_wdt_shutdown NULL
static int sprd_wdt_suspend(void)
{
	if (!sprd_wdt.wdt_enabled)
		return 0;

	sprd_wdt_stop();

	return 0;
}

static void sprd_wdt_resume(void)
{
	if (!sprd_wdt.wdt_enabled)
		return;

	sprd_wdt_start();
}

static struct syscore_ops sprd_wdt_syscore_ops = {
	.shutdown = sprd_wdt_shutdown,
	.suspend = sprd_wdt_suspend,
	.resume = sprd_wdt_resume,
};

static const struct of_device_id sprd_wdt_match_table[] = {
	{.compatible = "sprd,shark-wdt", .data = (void *)&sprd_wdt_data_v1},
	{.compatible = "sprd,whale-wdt", .data = (void *)&sprd_wdt_data_v2},
	{.compatible = "sprd,iwhale-wdt", .data = (void *)&sprd_wdt_data_v2},
	{.compatible = "sprd,sharkl2-wdt", .data = (void *)&sprd_wdt_data_v2},
	{.compatible = "sprd,sharkle-wdt", .data = (void *)&sprd_wdt_data_v2},
	{ /* sentinel */ }
};

static int sprd_wdt_probe(struct platform_device *pdev)
{
	int ret = 0;
	unsigned int wdt_rst_mode = 0;
	struct resource *wdt_res;
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct platform_device *pdev_regmap;

	const struct of_device_id *of_id =
	    of_match_device(sprd_wdt_match_table, &pdev->dev);
	if (!of_id)
		return -ENODEV;

	sprd_wdt.wdt_ops = (struct sprd_wdt_data *)of_id->data;

	wdt_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!wdt_res) {
		dev_err(&pdev->dev, "sprd_wdt get base resource failed!\n");
		return -ENODEV;
	}

	sprd_wdt.reg = devm_ioremap_nocache(dev, wdt_res->start,
					    resource_size(wdt_res));
	if (IS_ERR(sprd_wdt.reg)) {
		dev_err(&pdev->dev, "sprd_wdt get base address failed!\n");
		return PTR_ERR(sprd_wdt.reg);
	}

	sprd_wdt.irq = platform_get_irq(pdev, 0);
	if (sprd_wdt.irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource info\n");
		return -EIO;
	}

	ret = devm_request_irq(dev, sprd_wdt.irq, sprd_wdt_isr,
			       IRQF_NO_SUSPEND, "sprd_wdt", NULL);
	if (ret)
		dev_err(&pdev->dev, "sprd wdg isr register failed");

	sprd_wdt.wdt_gpr =
	    syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,wdt-enable");
	if (IS_ERR(sprd_wdt.wdt_gpr)) {
		dev_err(&pdev->dev, "syscon regmap lookup failed.\n");
		return -ENODEV;
	}

	np = of_parse_phandle(pdev->dev.of_node, "sprd,wdt-phandle", 0);
	if (!np)
		dev_err(&pdev->dev, "sprd,wdt-phandle get failed");

	pdev_regmap = of_find_device_by_node(np);
	if (!pdev_regmap) {
		dev_err(&pdev->dev, "sprd wdt find device node failed!\n");
		of_node_put(np);
		return -ENODEV;
	}

	sprd_wdt.pmic_wdt_gpr = dev_get_regmap(pdev_regmap->dev.parent, NULL);
	if (!sprd_wdt.pmic_wdt_gpr) {
		dev_err(&pdev->dev, "sprd wdt get regmap failed!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "reg", &sprd_wdt.pmic_reg);
	if (ret) {
		dev_err(&pdev->dev, "sprd wdt read reg failed!\n");
		of_node_put(np);
		return -EINVAL;
	}

	sprd_wdt.enable = of_clk_get_by_name(pdev->dev.of_node, "enable");
	sprd_wdt.rtc_enable = of_clk_get_by_name(pdev->dev.of_node, "rtc_enable");
	if (IS_ERR(sprd_wdt.rtc_enable) || IS_ERR(sprd_wdt.enable)) {
		wdt_dt_support = 0;
		dev_err(&pdev->dev, "can't get the wdg clock dts config!\n");
	}
	wdt_flag = 1;

	sprd_wdt_enable();
	sprd_wdt_feeder_init();

	/* 0xf0 coressponding to wachdog reboot mode of bootloader */
	regmap_read(sprd_wdt.pmic_wdt_gpr, ANA_REG_GLB_POR_RST_MONITOR,
		&wdt_rst_mode);
	pr_emerg("sprd_wdt_probe: ANA_REG_GLB_POR_RST_MONITOR value is = %x\n",
		wdt_rst_mode);
	regmap_write(sprd_wdt.pmic_wdt_gpr, ANA_REG_GLB_POR_RST_MONITOR,
		wdt_rst_mode|HWRST_STATUS_NORMAL2);

	regmap_read(sprd_wdt.pmic_wdt_gpr, ANA_REG_GLB_POR_RST_MONITOR,
		&wdt_rst_mode);
	pr_emerg("sprd_wdt_probe: ANA_REG_GLB_POR_RST_MONITOR value is = %x\n",
		wdt_rst_mode);

	sprd_wdt.debugfs = debugfs_create_dir("sprd_wdt", NULL);
	if (!sprd_wdt.debugfs)
		dev_err(&pdev->dev, "sprd wdt debugfs create failed");

	debugfs_create_file("info", S_IRUGO | S_IWUSR, sprd_wdt.debugfs,
			    NULL, &sprd_proc_fops);

	register_syscore_ops(&sprd_wdt_syscore_ops);

	dev_info(&pdev->dev, "sprd wdt probe OK!\n");
	return 0;
}

static int sprd_wdt_remove(struct platform_device *pdev)
{
	unregister_syscore_ops(&sprd_wdt_syscore_ops);
	kthread_stop(sprd_wdt.feed_task);

	return 0;
}

static struct platform_driver sprd_wdt_driver = {
	.probe = sprd_wdt_probe,
	.remove = sprd_wdt_remove,
	.driver = {
		   .name = "sprd-wdt",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(sprd_wdt_match_table),
		   },
};

static int __init sprd_wdt_init(void)
{
#if 0
	if (in_calibration()) {
		pr_info("calibration mode, quit...\n");
		return 1;
	}
#endif

	platform_driver_register(&sprd_wdt_driver);

	return 0;
}

static void __exit sprd_wdt_exit(void)
{
	platform_driver_unregister(&sprd_wdt_driver);
}

module_init(sprd_wdt_init);
module_exit(sprd_wdt_exit);
MODULE_DESCRIPTION("Spreadtrum Watchdog Timer Controller Driver");
MODULE_LICENSE("GPL v2");
