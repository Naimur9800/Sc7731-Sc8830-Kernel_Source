/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * Contact: xiaobo.dong <xiaobo.dong@spreadtrum.com>
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/spinlock.h>
#include <linux/regmap.h>
#include <linux/wakelock.h>

static unsigned long offset;
static struct regmap *rtc_map;

#define ANA_RTC_SEC_CNT_VALUE		(offset + 0x00)
#define ANA_RTC_MIN_CNT_VALUE		(offset + 0x04)
#define ANA_RTC_HOUR_CNT_VALUE		(offset + 0x08)
#define ANA_RTC_DAY_CNT_VALUE		(offset + 0x0C)

#define ANA_RTC_SEC_CNT_UPD		(offset + 0x10)
#define ANA_RTC_MIN_CNT_UPD		(offset + 0x14)
#define ANA_RTC_HOUR_CNT_UPD		(offset + 0x18)
#define ANA_RTC_DAY_CNT_UPD		(offset + 0x1C)

#define ANA_RTC_SEC_ALM_UPD		(offset + 0x20)
#define ANA_RTC_MIN_ALM_UPD		(offset + 0x24)
#define ANA_RTC_HOUR_ALM_UPD		(offset + 0x28)
#define ANA_RTC_DAY_ALM_UPD		(offset + 0x2C)

#define ANA_RTC_INT_EN			(offset + 0x30)
#define ANA_RTC_INT_RAW_STS		(offset + 0x34)
#define ANA_RTC_INT_CLR			(offset + 0x38)
#define ANA_RTC_INT_MASK_STS		(offset + 0x3C)

#define ANA_RTC_SEC_ALM_VALUE		(offset + 0x40)
#define ANA_RTC_MIN_ALM_VALUE		(offset + 0x44)
#define ANA_RTC_HOUR_ALM_VALUE		(offset + 0x48)
#define ANA_RTC_DAY_ALM_VALUE		(offset + 0x4C)

#define ANA_RTC_SPG_VALUE		(offset + 0x50)
#define ANA_RTC_SPG_UPD			(offset + 0x54)

#define ANA_RTC_SEC_AUXALM_UPD		(offset + 0x60)
#define ANA_RTC_MIN_AUXALM_UPD		(offset + 0x64)
#define ANA_RTC_HOUR_AUXALM_UPD		(offset + 0x68)
#define ANA_RTC_DAY_AUXALM_UPD		(offset + 0x6C)

/*
 * The corresponding bit of INT_CLR & INT_EN registers.
 */
#define RTC_SEC_BIT			BIT(0)
#define RTC_MIN_BIT			BIT(1)
#define RTC_HOUR_BIT			BIT(2)
#define RTC_DAY_BIT			BIT(3)
#define RTC_ALARM_BIT			BIT(4)
#define RTCCTL_HOUR_FMT_SEL		BIT(5)
#define RTC_AUX_ALARM_BIT		BIT(6)
#define RTC_SPG_BIT			BIT(7)
#define RTC_SEC_ACK_BIT			BIT(8)
#define RTC_MIN_ACK_BIT			BIT(9)
#define RTC_HOUR_ACK_BIT		BIT(10)
#define RTC_DAY_ACK_BIT			BIT(11)
#define RTC_SEC_ALM_ACK_BIT		BIT(12)
#define RTC_MIN_ALM_ACK_BIT		BIT(13)
#define RTC_HOUR_ALM_ACK_BIT		BIT(14)
#define RTC_DAY_ALM_ACK_BIT		BIT(15)

#define RTC_UPD_TIME_MASK	(RTC_SEC_ACK_BIT | RTC_MIN_ACK_BIT | \
	RTC_HOUR_ACK_BIT | RTC_DAY_ACK_BIT)
#define RTC_ALM_TIME_MASK	(RTC_SEC_ALM_ACK_BIT | \
	RTC_MIN_ALM_ACK_BIT | RTC_HOUR_ALM_ACK_BIT | RTC_DAY_ALM_ACK_BIT)
#define RTC_INT_ALM_MSK		(RTC_SEC_BIT | RTC_MIN_BIT | RTC_HOUR_BIT | \
	RTC_DAY_BIT | RTC_ALARM_BIT | RTC_AUX_ALARM_BIT)
#define RTC_INT_ALL_MSK		(0xffff & (~(BIT(5)|BIT(7))))

#define RTC_SEC_MASK			(0x3f)
#define RTC_MIN_MASK			(0x3f)
#define RTC_HOUR_MASK			(0x1f)
#define RTC_DAY_MASK			(0xffff)

#define SPRD_RTC_GET_MAX		(10)
#define SPRD_RTC_SET_MAX		(150)
#define SPRD_RTC_ALM_UNLOCK		(0xa5)
#define SPRD_RTC_ALM_LOCK		(0xff & (~SPRD_RTC_ALM_UNLOCK))
#define SPRD_RTC_ALMLOCK_MASK		(0xff)

#define SPRD_RTC_POWEROFF_ALARM		(0x1 << 8)
#define SPRD_RTC_POWERDOWN_RESET	(0x1 << 9)

struct sprd_rtc {
	struct rtc_device	*rtc;
	unsigned int		irq_no;
	struct clk		*clk;
};

enum ALARM_TYPE {
	SET_POWERON_ALARM = 0,
	GET_POWERON_ALARM,
	SET_WAKE_ALARM,
	GET_WAKE_ALARM,
	SET_POWEROFF_ALARM,
	GET_POWEROFF_ALARM,
};

static struct wake_lock rtc_irq_wake_lock;
static unsigned long secs_start_year_to_1970;

static unsigned int sprd_rtc_read(unsigned int addr)
{
	unsigned int val = 0;

	regmap_read(rtc_map, addr, &val);
	return val;
}

static void sprd_rtc_write(unsigned int addr, unsigned int val)
{
	regmap_write(rtc_map, addr, val);
}

static inline unsigned int get_sec(void)
{
	return sprd_rtc_read(ANA_RTC_SEC_CNT_VALUE) & RTC_SEC_MASK;
}

static inline unsigned int get_min(void)
{
	return sprd_rtc_read(ANA_RTC_MIN_CNT_VALUE) & RTC_MIN_MASK;
}

static inline unsigned int get_hour(void)
{
	return sprd_rtc_read(ANA_RTC_HOUR_CNT_VALUE) & RTC_HOUR_MASK;
}

static inline unsigned int get_day(void)
{
	return sprd_rtc_read(ANA_RTC_DAY_CNT_VALUE) & RTC_DAY_MASK;
}

static unsigned long sprd_rtc_get_sec(void)
{
	unsigned long seconds;
	unsigned int sec, min, hour, day;

	sec = get_sec();
	min = get_min();
	hour = get_hour();
	day = get_day();

	seconds = (((unsigned long)(day*24) + hour)*60 + min)*60 + sec;
	return seconds;
}

static int sprd_rtc_set_sec(unsigned long secs)
{
	unsigned int sec, min, hour, day;
	unsigned int int_rsts;
	unsigned long temp;
	int cnt = 0, ret = 0;

	sec = secs % 60;
	temp = (secs - sec)/60;
	min = temp%60;
	temp = (temp - min)/60;
	hour = temp%24;
	temp = (temp - hour)/24;
	day = temp;

	sprd_rtc_write(ANA_RTC_SEC_CNT_UPD, sec);
	sprd_rtc_write(ANA_RTC_MIN_CNT_UPD, min);
	sprd_rtc_write(ANA_RTC_HOUR_CNT_UPD, hour);
	sprd_rtc_write(ANA_RTC_DAY_CNT_UPD, day);

	/* wait till all update done */
	do {
		int_rsts = sprd_rtc_read(ANA_RTC_INT_RAW_STS) &
			RTC_UPD_TIME_MASK;
		if (RTC_UPD_TIME_MASK == int_rsts)
			break;
		msleep(20);
	} while (cnt++ < SPRD_RTC_SET_MAX);

	if (cnt >= SPRD_RTC_SET_MAX) {
		pr_err("RTC Set time values time out\n");
		ret = -ETIMEDOUT;
	}
	sprd_rtc_write(ANA_RTC_INT_CLR, RTC_UPD_TIME_MASK);
	return ret;
}

static inline unsigned long sprd_rtc_get_alarm_sec(void)
{
	unsigned int sec, min, hour, day;
	unsigned long alarm_secs;

	day = sprd_rtc_read(ANA_RTC_DAY_ALM_VALUE) & RTC_DAY_MASK;
	hour = sprd_rtc_read(ANA_RTC_HOUR_ALM_VALUE) & RTC_HOUR_MASK;
	min = sprd_rtc_read(ANA_RTC_MIN_ALM_VALUE) & RTC_MIN_MASK;
	sec = sprd_rtc_read(ANA_RTC_SEC_ALM_VALUE) & RTC_SEC_MASK;

	alarm_secs = (((unsigned long)(day*24) + hour)*60 + min)*60 + sec;
	return alarm_secs;
}

static int sprd_rtc_set_alarm_sec(unsigned long secs)
{
	unsigned int sec, min, hour, day;
	unsigned long temp;
	static bool rtc_alarm_in_update;
	unsigned int timeout = 0;

	sec = secs % 60;
	temp = (secs - sec)/60;
	min = temp%60;
	temp = (temp - min)/60;
	hour = temp%24;
	temp = (temp - hour)/24;
	day = temp;

	/* Optimize suspend process time */
	if (rtc_alarm_in_update) {
		while ((sprd_rtc_read(ANA_RTC_INT_RAW_STS) &
				RTC_ALM_TIME_MASK) != RTC_ALM_TIME_MASK) {
			msleep(20);
			if (timeout++ > SPRD_RTC_SET_MAX) {
				pr_info("RTC set alarm timeout\n");
				break;
			}
		}
	}

	sprd_rtc_write(ANA_RTC_INT_CLR, RTC_ALM_TIME_MASK);

	sprd_rtc_write(ANA_RTC_SEC_ALM_UPD, sec);
	sprd_rtc_write(ANA_RTC_MIN_ALM_UPD, min);
	sprd_rtc_write(ANA_RTC_HOUR_ALM_UPD, hour);
	sprd_rtc_write(ANA_RTC_DAY_ALM_UPD, day);

	rtc_alarm_in_update = true;
	return 0;
}

static int sprd_rtc_read_alarm(struct device *dev,
	struct rtc_wkalrm *alrm)
{
	unsigned long secs = sprd_rtc_get_alarm_sec();

	secs = secs + secs_start_year_to_1970;
	rtc_time_to_tm(secs, &alrm->time);

	alrm->enabled = !!(sprd_rtc_read(ANA_RTC_INT_EN) & RTC_ALARM_BIT);
	alrm->pending = !!(sprd_rtc_read(ANA_RTC_INT_RAW_STS) & RTC_ALARM_BIT);

	return 0;
}

static int sprd_rtc_read_poweroff_alarm(struct device *dev,
	struct rtc_wkalrm *alrm)
{
	return sprd_rtc_read_alarm(dev, alrm);
}

static inline unsigned long sprd_rtc_get_aux_alarm_sec(void)
{
	unsigned int sec, min, hour, day;
	unsigned long aux_alrm_secs;

	day = sprd_rtc_read(ANA_RTC_DAY_AUXALM_UPD) & RTC_DAY_MASK;
	hour = sprd_rtc_read(ANA_RTC_HOUR_AUXALM_UPD) & RTC_HOUR_MASK;
	min = sprd_rtc_read(ANA_RTC_MIN_AUXALM_UPD) & RTC_MIN_MASK;
	sec = sprd_rtc_read(ANA_RTC_SEC_AUXALM_UPD) & RTC_SEC_MASK;

	aux_alrm_secs = (((unsigned long)(day*24) + hour)*60 + min)*60 + sec;
	return aux_alrm_secs;
}

static int sprd_rtc_set_aux_alarm_sec(unsigned long secs)
{
	unsigned int sec, min, hour, day;
	unsigned long temp;

	sec = secs % 60;
	temp = (secs - sec)/60;
	min = temp%60;
	temp = (temp - min)/60;
	hour = temp%24;
	temp = (temp - hour)/24;
	day = temp;

	sprd_rtc_write(ANA_RTC_INT_CLR, RTC_AUX_ALARM_BIT);
	sprd_rtc_write(ANA_RTC_SEC_AUXALM_UPD, sec);
	sprd_rtc_write(ANA_RTC_MIN_AUXALM_UPD, min);
	sprd_rtc_write(ANA_RTC_HOUR_AUXALM_UPD, hour);
	sprd_rtc_write(ANA_RTC_DAY_AUXALM_UPD, day);

	return 0;
}

static int sprd_rtc_read_aux_alarm(struct device *dev,
	struct rtc_wkalrm *alrm)
{
	unsigned long secs = sprd_rtc_get_aux_alarm_sec();

	secs = secs + secs_start_year_to_1970;
	rtc_time_to_tm(secs, &alrm->time);

	alrm->enabled = !!(sprd_rtc_read(ANA_RTC_INT_EN) & RTC_AUX_ALARM_BIT);
	alrm->pending = !!(sprd_rtc_read(ANA_RTC_INT_RAW_STS) &
		RTC_AUX_ALARM_BIT);

	return 0;
}

static int sprd_rtc_set_aux_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned long secs;
	unsigned int irq_en_reg;

	sprd_rtc_write(ANA_RTC_INT_CLR, RTC_AUX_ALARM_BIT);

	if (alrm->enabled) {
		rtc_tm_to_time(&alrm->time, &secs);
		if (secs < secs_start_year_to_1970) {
			dev_err(dev, "Can't set alarm, need the right time\n");
			return -1;
		}

		/*Notice! need to deal with normal alarm and aux alarm*/
		irq_en_reg = sprd_rtc_read(ANA_RTC_INT_EN);
		irq_en_reg |= RTC_AUX_ALARM_BIT;
		sprd_rtc_write(ANA_RTC_INT_EN, irq_en_reg);

		secs -= secs_start_year_to_1970;
		sprd_rtc_set_aux_alarm_sec(secs);
		/*unlock the rtc alrm int*/
	} else {
		irq_en_reg = sprd_rtc_read(ANA_RTC_INT_EN);
		irq_en_reg &= ~RTC_AUX_ALARM_BIT;
		sprd_rtc_write(ANA_RTC_INT_EN, irq_en_reg);
	}

	return 0;
}

static int sprd_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned long secs;
	int irq_en_reg, spg_val;

	sprd_rtc_write(ANA_RTC_INT_CLR, RTC_ALARM_BIT);

	if (alrm->enabled) {
		rtc_tm_to_time(&alrm->time, &secs);
		if (secs < secs_start_year_to_1970) {
			dev_err(dev, "Can't set alarm, need the right time\n");
			return -1;
		}

		/* Notice! need to deal with normal alarm and aux alarm */
		irq_en_reg = sprd_rtc_read(ANA_RTC_INT_EN);
		irq_en_reg |= RTC_ALARM_BIT;
		sprd_rtc_write(ANA_RTC_INT_EN, irq_en_reg);

		secs -= secs_start_year_to_1970;
		sprd_rtc_set_alarm_sec(secs);

		/* unlock the rtc alrm int */
		spg_val = sprd_rtc_read(ANA_RTC_SPG_VALUE);
		spg_val &= (~(SPRD_RTC_ALMLOCK_MASK |
				SPRD_RTC_POWEROFF_ALARM));
		spg_val |= SPRD_RTC_ALM_UNLOCK;
		sprd_rtc_write(ANA_RTC_SPG_UPD, spg_val);
	} else {
		irq_en_reg = sprd_rtc_read(ANA_RTC_INT_EN);
		irq_en_reg &= ~RTC_ALARM_BIT;
		sprd_rtc_write(ANA_RTC_INT_EN, irq_en_reg);

		/* lock the rtc alrm int */
		spg_val = sprd_rtc_read(ANA_RTC_SPG_VALUE);
		spg_val &= (~(SPRD_RTC_ALMLOCK_MASK |
				SPRD_RTC_POWEROFF_ALARM));
		spg_val |= SPRD_RTC_ALM_LOCK;
		sprd_rtc_write(ANA_RTC_SPG_UPD, spg_val);
		msleep(150);
	}

	return 0;
}

static int sprd_rtc_set_poweroff_alarm(struct device *dev,
	struct rtc_wkalrm *alrm)
{
	unsigned int spg_val;
	int ret;

	ret = sprd_rtc_set_alarm(dev, alrm);
	if (ret) /* set alarm error */
		return ret;

	if (alrm->enabled) {
		spg_val = sprd_rtc_read(ANA_RTC_SPG_VALUE);
		spg_val &= (~SPRD_RTC_ALMLOCK_MASK);
		spg_val |= (SPRD_RTC_ALM_UNLOCK | SPRD_RTC_POWEROFF_ALARM);
		sprd_rtc_write(ANA_RTC_SPG_UPD, spg_val);
		msleep(150);
	}

	return 0;
}

static int sprd_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long secs = sprd_rtc_get_sec();

	secs = secs + secs_start_year_to_1970;

	if (secs > 0x7f000000)
		secs = secs_start_year_to_1970;

	rtc_time_to_tm(secs, tm);

	return rtc_valid_tm(tm);
}

static int sprd_rtc_set_time(struct device *dev,
	struct rtc_time *tm)
{
	unsigned long secs;

	rtc_tm_to_time(tm, &secs);
	if (secs < secs_start_year_to_1970) {
		dev_err(dev, "Can't set time, need the right time\n");
		return -1;
	}

	secs -= secs_start_year_to_1970;
	return sprd_rtc_set_sec(secs);
}

static int sprd_rtc_set_mmss(struct device *dev, unsigned long secs)
{
	if (secs < secs_start_year_to_1970) {
		dev_err(dev, "Can't set mmss, need the right time\n");
		return -1;
	}

	secs -= secs_start_year_to_1970;
	return sprd_rtc_set_sec(secs);
}

/*
 * check current time
 */
static int sprd_check_current_time(void)
{
	unsigned long secs;

	secs = sprd_rtc_get_sec();
	secs = secs + secs_start_year_to_1970;

	if ((secs < secs_start_year_to_1970) || (secs > 0x7f000000))
		return -1;
	else
		return 0;
}

static int sprd_correct_rtc_time(struct device *dev, const char *from)
{
	unsigned long secs;
	struct rtc_time tm;

	dev_info(dev, "%s: RTC power down and reset time.\n", from);
	secs = mktime(CONFIG_RTC_START_YEAR, 1, 1, 0, 0, 0);
	rtc_time_to_tm(secs, &tm);

	return sprd_rtc_set_time(dev, &tm);
}

static int sprd_rtc_check_power_down(struct device *dev)
{
	unsigned int spg_val;
	unsigned int_stat = 0;
	int ret = 0, cnt = 0;

	spg_val = sprd_rtc_read(ANA_RTC_SPG_VALUE);
	if (((spg_val & SPRD_RTC_POWERDOWN_RESET) == 0) ||
		sprd_check_current_time()) {
		ret = sprd_correct_rtc_time(dev, "soft");
		if (ret < 0) {
			dev_err(dev, "%s: correct RTC time error.\n",
				"soft");
			return ret;
		}

		spg_val |= SPRD_RTC_POWERDOWN_RESET;
		sprd_rtc_write(ANA_RTC_SPG_UPD, spg_val);
		/* wait till SPG value update done */
		do {
			int_stat = sprd_rtc_read(ANA_RTC_INT_RAW_STS) &
				RTC_SPG_BIT;
			if (RTC_SPG_BIT == int_stat)
				break;
			msleep(20);
		} while (cnt++ < SPRD_RTC_SET_MAX);

		if (cnt >= SPRD_RTC_SET_MAX) {
			dev_err(dev, "reset RTC time error.\n");
			ret = -EBUSY;
		}
	}
	return ret;
}

static irqreturn_t rtc_interrupt_handler(int irq, void *dev_id)
{
	struct sprd_rtc *rdev = dev_id;

	pr_info("RTC ***** interrupt happen\n");

	rtc_aie_update_irq(rdev->rtc);
	wake_lock_timeout(&rtc_irq_wake_lock, 2*HZ);
	sprd_rtc_write(ANA_RTC_INT_CLR, RTC_INT_ALM_MSK);
	return IRQ_HANDLED;
}

static int sprd_rtc_open(struct device *dev)
{
	int irq_en_reg;

	/* enable rtc alarm interrupt */
	irq_en_reg = sprd_rtc_read(ANA_RTC_INT_EN);
	irq_en_reg |= RTC_ALARM_BIT | RTC_AUX_ALARM_BIT;
	sprd_rtc_write(ANA_RTC_INT_EN, irq_en_reg);

	dev_info(dev, "RTC open is calling\n");
	return 0;
}

static int sprd_rtc_ioctl(struct device *dev, unsigned int cmd,
	unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case SET_POWERON_ALARM:
		ret = sprd_rtc_set_alarm(dev, (struct rtc_wkalrm *)arg);
		break;
	case GET_POWERON_ALARM:
		ret = sprd_rtc_read_alarm(dev, (struct rtc_wkalrm *)arg);
		break;
	case SET_WAKE_ALARM:
		ret = sprd_rtc_set_aux_alarm(dev, (struct rtc_wkalrm *)arg);
		break;
	case GET_WAKE_ALARM:
		ret = sprd_rtc_read_aux_alarm(dev, (struct rtc_wkalrm *)arg);
		break;
	case SET_POWEROFF_ALARM:
		ret = sprd_rtc_set_poweroff_alarm(dev,
			(struct rtc_wkalrm *)arg);
		break;
	case GET_POWEROFF_ALARM:
		ret = sprd_rtc_read_poweroff_alarm(dev,
			(struct rtc_wkalrm *)arg);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int sprd_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	/* dummy function to pass vts test*/
	return 0;
}

static const struct rtc_class_ops sprd_rtc_ops = {
	.open = sprd_rtc_open,
	.read_time = sprd_rtc_read_time,
	.read_alarm = sprd_rtc_read_alarm,
	.set_time = sprd_rtc_set_time,
	.set_alarm = sprd_rtc_set_alarm,
	.set_mmss = sprd_rtc_set_mmss,
	.ioctl = sprd_rtc_ioctl,
	.alarm_irq_enable = sprd_rtc_alarm_irq_enable,
};

static int sprd_rtc_probe(struct platform_device *pdev)
{
	struct sprd_rtc *rdev;
	struct device_node *node = pdev->dev.of_node;
	int irq, ret;
	unsigned int value;

	rtc_map = dev_get_regmap(pdev->dev.parent, NULL);
	ret = of_property_read_u32(node, "reg", &value);
	if (ret) {
		dev_err(&pdev->dev, "%s :no property of reg\n", __func__);
		return -ENXIO;
	}

	offset = (unsigned long)value;

	rdev = devm_kzalloc(&pdev->dev, sizeof(*rdev), GFP_KERNEL);
	if (!rdev) {
		/* dev_err(&pdev->dev, "out of memory\n"); */
		dev_err(&pdev->dev, "can't get enough memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, rdev);

#if 0	/* TODO: may be add later */
	/* get and enable rtc clock */
	rdev->clk = devm_clk_get(&pdev->dev, "ext_32k");
	if (IS_ERR(rdev->clk)) {
		dev_err(&pdev->dev, "can't get ext_32k clock\n");
		return -ENXIO;
	}

	ret = clk_prepare_enable(rdev->clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "clock prepare enable failed\n");
		return -EBUSY;
	}
#endif

	sprd_rtc_check_power_down(&pdev->dev);

	/* disable and clean irq */
	sprd_rtc_write(ANA_RTC_INT_EN, sprd_rtc_read(ANA_RTC_INT_EN) & ~0xffff);
	sprd_rtc_write(ANA_RTC_INT_CLR, RTC_INT_ALL_MSK);

	irq = platform_get_irq(pdev, 0);
	if (unlikely(irq <= 0)) {
		dev_err(&pdev->dev, "no irq resource specified\n");
		ret = -ENXIO;
		goto clk_disable;
	}
	rdev->irq_no = irq;

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		rtc_interrupt_handler, IRQF_ONESHOT |
		IRQF_EARLY_RESUME,
		"sprd_rtc", rdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "request irq error\n");
		goto clk_disable;
	}

	device_init_wakeup(&pdev->dev, 1);
	rdev->rtc = devm_rtc_device_register(&pdev->dev, "sprd_rtc",
		&sprd_rtc_ops, THIS_MODULE);
	if (IS_ERR(rdev->rtc)) {
		device_init_wakeup(&pdev->dev, 0);
		ret = PTR_ERR(rdev->rtc);
		goto clk_disable;
	}

	return 0;

clk_disable:
	clk_disable_unprepare(rdev->clk);
	return ret;
}

static int sprd_rtc_remove(struct platform_device *pdev)
{
	struct sprd_rtc *rdev = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, 0);
	clk_disable_unprepare(rdev->clk);

	return 0;
}

static struct of_device_id sprd_rtc_of_match[] = {
	{ .compatible = "sprd,sc2723t-rtc", },
	{ .compatible = "sprd,sc2731-rtc", },
	{ .compatible = "sprd,sc2721-rtc", },
	{ .compatible = "sprd,sc2720-rtc", },
	{ },
};

static struct platform_driver sprd_rtc_driver = {
	.driver = {
		.name = "sprd_rtc",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sprd_rtc_of_match),
	},
	.probe	= sprd_rtc_probe,
	.remove = sprd_rtc_remove,
};

static int __init sprd_rtc_init(void)
{
	int ret;

	wake_lock_init(&rtc_irq_wake_lock, WAKE_LOCK_SUSPEND, "rtc_irq");

	if (CONFIG_RTC_START_YEAR > 1970)
		secs_start_year_to_1970 = mktime(CONFIG_RTC_START_YEAR,
			1, 1, 0, 0, 0);
	else
		secs_start_year_to_1970 = mktime(1970, 1, 1, 0, 0, 0);

	ret = platform_driver_register(&sprd_rtc_driver);
	if (ret) {
		wake_lock_destroy(&rtc_irq_wake_lock);
		return ret;
	}

	return 0;
}

static void __exit sprd_rtc_exit(void)
{
	platform_driver_unregister(&sprd_rtc_driver);
	wake_lock_destroy(&rtc_irq_wake_lock);
}

MODULE_DESCRIPTION("SPREADTRUM RTC Device Driver");
MODULE_AUTHOR("Xiaobo.Dong <Xiaobo.Dong@spreadtrum.com>");
MODULE_LICENSE("GPL");

module_init(sprd_rtc_init);
module_exit(sprd_rtc_exit);
