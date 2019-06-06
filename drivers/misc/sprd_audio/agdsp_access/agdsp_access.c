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

#define pr_fmt(fmt) "[Audio:AGDSP_ACCESS] "fmt

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/hwspinlock.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/sipc.h>
#include <linux/slab.h>
#include <linux/sprd_hwspinlock.h>

#include <sprd_mailbox.h>
#include "agdsp_access.h"

/* flag for CMD/DONE msg type */
#define SMSG_CMD_AGDSP_ACCESS_INIT		0x0001
#define SMSG_DONE_AGDSP_ACCESS_INIT		0x0002

#define HWSPINLOCK_TIMEOUT		(5000)

#define AGCP_READL(addr)      readl((void __iomem *) (addr))
#define AGCP_WRITEL(b, addr)  writel(b, (void __iomem *)(addr))

#define AGDSP_ACCESS_DEBUG 0

#if AGDSP_ACCESS_DEBUG
#define pr_dbg(fmt, ...) pr_err(fmt, ##__VA_ARGS__)
#else
#define pr_dbg(fmt, ...)
#endif

#if !defined(REG_PMU_APB_PWR_STATUS3_DBG)
#if defined(REG_PMU_APB_AGCP_SYS_SLEEP_CTRL)
#define REG_PMU_APB_SLEEP_CTRL REG_PMU_APB_AGCP_SYS_SLEEP_CTRL
#define BIT_PMU_APB_AGCP_DEEP_SLEEP BIT_PMU_APB_AGCP_SYS_DEEP_SLEEP
#endif
#endif

#if !defined(REG_PMU_APB_SYS_SLP_STATUS0)
#if defined(REG_PMU_APB_AGCP_SLP_STATUS_DBG0)
#define REG_PMU_APB_SYS_SLP_STATUS0 REG_PMU_APB_AGCP_SLP_STATUS_DBG0
#define AGCP_SLP_CONDIDTION(x) ((BIT_PMU_APB_AGCP_DEEP_SLP_DBG(0xFFFFFFFF) \
				& (x)) != BIT_PMU_APB_AGCP_DEEP_SLP_DBG(0x6))
#endif
#else
#define AGCP_SLP_CONDIDTION(x) ((BIT_PMU_APB_AGCP_SYS_SLP_STATUS(0xf) \
				& (x)) != BIT_PMU_APB_AGCP_SYS_SLP_STATUS(0x6))
#endif

#if !defined(REG_PMU_APB_PWR_STATE_DBG2)
#if defined(REG_PMU_APB_PWR_STATUS3_DBG)
#define REG_PMU_APB_PWR_STATE_DBG2 REG_PMU_APB_PWR_STATUS3_DBG
#define REG_PMU_APB_PWR_STATE_DBG1 REG_PMU_APB_PWR_STATUS3_DBG
#endif
#endif

struct agdsp_access_state {
	int32_t ap_enable_cnt;
	int32_t cp_enable_cnt;
};

struct agdsp_access {
	struct hwspinlock	*hw_lock;
	struct agdsp_access_state *state;
	uint32_t smem_phy_addr;
	uint32_t smem_size;
	uint32_t ddr_addr_offset;
	struct task_struct *thread;
	int ready;
	uint8_t dst;
	uint8_t channel;
	uint8_t mbx_core;
	struct regmap *agcp_ahb;
	struct regmap *pmu_apb;
};

static struct agdsp_access  *g_agdsp_access;

/*extern int mbox_raw_sent(u8 target_id, u64 msg);*/
static int agdsp_access_init_thread(void *data)
{
	int rval = 0;
	struct smsg mcmd, mrecv;
	struct agdsp_access *dsp_ac = (struct agdsp_access *)data;
	const uint32_t offset = dsp_ac->ddr_addr_offset;

	pr_dbg("agdsp access thread entry.\n");

	/* since the channel open may hang, we call it in the sblock thread */
	rval = smsg_ch_open(dsp_ac->dst, dsp_ac->channel, -1);
	if (rval != 0) {
		pr_info("Failed to open channel %d,dst=%d,rval=%d\n",
			dsp_ac->channel, dsp_ac->dst, rval);
		/* assign NULL to thread poniter as failed to open channel */
		dsp_ac->thread = NULL;
		return rval;
	}

	/* handle the sblock events */
	while (!kthread_should_stop()) {
		/* monitor sblock recv smsg */
		smsg_set(&mrecv, dsp_ac->channel, 0, 0, 0);
		rval = smsg_recv(dsp_ac->dst, &mrecv, -1);
		if (rval == -EIO || rval == -ENODEV) {
			/* channel state is FREE */
			msleep(20);
			continue;
		}

		pr_info("%s: dst=%d, channel=%d, type=%d, flag=0x%04x, smem_phy_addr=0x%x, value=0x%08x\n",
			__func__, dsp_ac->dst, dsp_ac->channel, mrecv.type,
			mrecv.flag, mrecv.value, dsp_ac->smem_phy_addr);

		switch (mrecv.type) {
		case SMSG_TYPE_OPEN:
			smsg_open_ack(dsp_ac->dst, dsp_ac->channel);
			/*dsp_ac->ready = false;*/
			break;
		case SMSG_TYPE_CMD:
			/* respond cmd done for sblock init */
			WARN_ON(mrecv.flag != SMSG_CMD_AGDSP_ACCESS_INIT);
			smsg_set(&mcmd,
				 dsp_ac->channel,
				 SMSG_TYPE_DONE,
				 SMSG_DONE_AGDSP_ACCESS_INIT,
				 dsp_ac->smem_phy_addr + offset);
			smsg_send(dsp_ac->dst, &mcmd, -1);
			/*dsp_ac->ready = true;*/
			break;
		default:
			pr_info("non-handled agdsp access msg: %d-%d, %d, %d, %d\n",
				dsp_ac->dst, dsp_ac->channel,
				mrecv.type, mrecv.flag, mrecv.value);
			break;
		};
	}
	smsg_ch_close(dsp_ac->dst, dsp_ac->channel, -1);

	return rval;

	pr_err("agdsp access thread error exit.\n");

	return 0;
}

static int agdsp_access_initialize(struct regmap *agcp_ahb,
				   struct regmap *pmu_apb,
				   uint32_t mbx_core,
				   uint32_t dst, uint32_t channel,
				   const uint32_t offset)
{
	struct agdsp_access *dsp_ac;

	g_agdsp_access = kzalloc(sizeof(struct agdsp_access), GFP_KERNEL);
	if ((g_agdsp_access == NULL) || (!agcp_ahb)) {
		pr_err("agdsp_access init:Failed to allocate\n");
		return -ENOMEM;
	}
	g_agdsp_access->agcp_ahb = agcp_ahb;
	g_agdsp_access->pmu_apb = pmu_apb;
	g_agdsp_access->mbx_core = mbx_core;
	g_agdsp_access->dst = dst;
	g_agdsp_access->channel = channel;
	g_agdsp_access->ddr_addr_offset = offset;

	dsp_ac = g_agdsp_access;
	dsp_ac->hw_lock = hwspin_lock_request_specific(11);
	if (!dsp_ac->hw_lock) {
		pr_err("agdsp_access_init can not get the hardware spinlock.\n");
		goto error;
	}

	dsp_ac->smem_size = sizeof(struct agdsp_access_state);
	dsp_ac->smem_phy_addr = smem_alloc(dsp_ac->smem_size);

	if (!dsp_ac->smem_phy_addr) {
		pr_err("%s,smem_phy_addr is 0.\n", __func__);
		goto error;
	}

	dsp_ac->state = shmem_ram_vmap_nocache(dsp_ac->smem_phy_addr,
		sizeof(struct agdsp_access_state));
	if (!dsp_ac->state) {
		pr_err("%s,shmem_ram_vmap_nocache return 0.\n", __func__);
		goto error;
	}

	dsp_ac->state->ap_enable_cnt = 0;
	dsp_ac->state->cp_enable_cnt = 0;
	dsp_ac->ready = true;

	pr_dbg("agdsp access init, ready.\n");
	regmap_update_bits(dsp_ac->agcp_ahb, REG_AON_APB_AGCP_CTRL,
			BIT_AON_APB_AGCP_TOP_ACCESS_EN, 0);

	g_agdsp_access->thread =
		kthread_create(agdsp_access_init_thread, g_agdsp_access,
			       "agdsp_access");
	if (IS_ERR(g_agdsp_access->thread))
		goto error;

	wake_up_process(g_agdsp_access->thread);

	return 0;
error:
	pr_err("agdsp_access init failed, exit.\n");

	return -EINVAL;
}

int agdsp_can_access(void)
{
	int ret = 0;
	int val = 0;
	struct agdsp_access *dsp_ac = g_agdsp_access;

	ret = regmap_read(dsp_ac->agcp_ahb,
		REG_AON_APB_AGCP_CTRL, &val);
	if (ret != 0) {
		pr_err("%s, regmap_read   REG_AON_APB_AGCP_CTRLerror!\n",
		__func__);
		return 0;
	}
	if (!(BIT_AON_APB_AGCP_TOP_ACCESS_EN & val)) {
		pr_err("%s, BIT_AON_APB_AGCP_TOP_ACCESS_EN not enable! value: %x\n",
		__func__, val);
		return 0;
	}

	ret = regmap_read(dsp_ac->pmu_apb,
		REG_PMU_APB_SLEEP_CTRL, &val);
	if (ret != 0) {
		pr_err("%s, regmap_read   REG_AON_APB_AGCP_CTRLerror!\n",
		__func__);
		return 0;
	}
	if (val & BIT_PMU_APB_AGCP_DEEP_SLEEP) {
		pr_err("%s, agdsp   BIT_PMU_APB_AGCP_DEEP_SLEEP!value : %x\n",
		__func__, val);
		return 0;
	}

	ret = regmap_read(dsp_ac->pmu_apb, REG_PMU_APB_SYS_SLP_STATUS0, &val);
	if (ret != 0) {
		pr_err("%s, regmap_read REG_PMU_APB_SYS_SLP_STATUS0!\n",
		       __func__);
		return 0;
	}
	if (AGCP_SLP_CONDIDTION(val)) {
		pr_err("%s, BIT_PMU_APB_AGCP_SYS_SLP_STATUS not enable! value: %x\n",
		__func__, val);
		return 0;
	}

	ret = regmap_read(dsp_ac->pmu_apb, REG_PMU_APB_PWR_STATE_DBG2, &val);
	if (ret != 0) {
		pr_err("%s, regmap_read   REG_PMU_APB_PWR_STATE_DBG2!\n",
		       __func__);
		return 0;
	}
	if ((BIT_PMU_APB_PD_AGCP_DSP_STATE(0x1f) & val) != 0) {
		pr_err("%s, BIT_PMU_APB_PD_AGCP_DSP_STATE not enable! value: %x\n",
		__func__, val);
		return 0;
	}

	ret = regmap_read(dsp_ac->pmu_apb, REG_PMU_APB_PWR_STATE_DBG1, &val);
	if (ret != 0) {
		pr_err("%s, regmap_read   REG_PMU_APB_PWR_STATE_DBG1!\n",
		       __func__);
		return 0;
	}
	if ((BIT_PMU_APB_PD_AGCP_SYS_STATE(0x1f) & val) != 0) {
		pr_err("%s, BIT_PMU_APB_PD_AGCP_SYS_STATE not enable! value: %x\n",
		__func__, val);
		return 0;
	}

	return 1;
}

#define TRY_CNT_MAX 1000000
int agdsp_access_enable(void)
{
	int ret = 0;
	unsigned long flags;
	int cnt = 0;
	int val = 0;
	struct agdsp_access *dsp_ac = g_agdsp_access;

	pr_dbg("%s entry\n", __func__);
	if (!dsp_ac) {
		pr_err("agdsp access not init, exit\n");
		return -EINVAL;
	}

	if (!dsp_ac->ready || !dsp_ac->state) {
		pr_err("agdsp access not ready, dsp_ac->ready=%d, dsp_ac->state=%p\n",
		       dsp_ac->ready, dsp_ac->state);
		return -EINVAL;
	}
	ret = hwspin_lock_timeout_irqsave(dsp_ac->hw_lock,
					  HWSPINLOCK_TIMEOUT, &flags);
	if (ret) {
		pr_err("agdsp access, Read: lock the hw lock failed.\n");
		return ret;
	}

	ret = regmap_update_bits(dsp_ac->agcp_ahb,
		REG_AON_APB_AGCP_CTRL,
		BIT_AON_APB_AGCP_TOP_ACCESS_EN,
		BIT_AON_APB_AGCP_TOP_ACCESS_EN);
	if (ret != 0) {
		pr_err("%s, regmap_update_bits REG_AON_APB_AGCP_CTRL error!\n",
		__func__);
		ret = -EINVAL;
		goto exit;
	}

	ret = regmap_read(dsp_ac->agcp_ahb,
		REG_AON_APB_AGCP_CTRL, &val);
	if (ret != 0) {
		pr_err("%s, regmap_read REG_AON_APB_AGCP_CTRL error!\n",
			__func__);
		ret = -EINVAL;
		goto exit;
	}
	if (!(BIT_AON_APB_AGCP_TOP_ACCESS_EN & val)) {
		pr_err("%s, BIT_AON_APB_AGCP_TOP_ACCESS_EN not enable! value: %x\n",
			__func__, val);
		ret = -EINVAL;
		goto exit;
	}

	if (dsp_ac->state->ap_enable_cnt == 0) {
		/* send a mail to AGDSP to wake up it,
		 * 100 is an invalid command
		 */
		mbox_raw_sent(dsp_ac->mbx_core, 100);
		udelay(20);

		do {
			cnt++;
			ret = regmap_read(dsp_ac->pmu_apb,
				REG_PMU_APB_PWR_STATE_DBG1, &val);
			pr_dbg("%s, regmap_read val=0x%x!\n", __func__, val);
			if (ret != 0) {
				pr_err("%s, regmap_read error!\n", __func__);
				break;
			}
			/* 0: power up finished;7:power off */
			if ((BIT_PMU_APB_PD_AGCP_SYS_STATE(0x1F) & val) == 0)
				break;

			/* Do not need delay,
			 * normally we need read only one time
			 */
			udelay(5);
		} while (cnt < TRY_CNT_MAX);
		if (cnt == TRY_CNT_MAX) {
			pr_err("ERR: %s wait agdsp power up timeout!\n",
			       __func__);
			ret = -EBUSY;
		}
	}

	AGCP_WRITEL(AGCP_READL(&dsp_ac->state->ap_enable_cnt) + 1,
		&dsp_ac->state->ap_enable_cnt);

exit:
	hwspin_unlock_irqrestore(dsp_ac->hw_lock, &flags);

	pr_debug("%s, ap_enable_cnt=%d, cp_enable_cnt=%d.cnt=%d\n",
		__func__, dsp_ac->state->ap_enable_cnt,
		dsp_ac->state->cp_enable_cnt, cnt);

	return ret;
}
EXPORT_SYMBOL(agdsp_access_enable);

int agdsp_access_disable(void)
{
	int ret = 0;
	unsigned long flags;
	struct agdsp_access *dsp_ac = g_agdsp_access;

	pr_dbg("%s entry\n", __func__);
	if (!dsp_ac)
		return -EINVAL;

	if (!dsp_ac->ready || !dsp_ac->state)
		return -EINVAL;

	ret = hwspin_lock_timeout_irqsave(dsp_ac->hw_lock,
					  HWSPINLOCK_TIMEOUT, &flags);
	if (ret) {
		pr_err("agdsp access, Read: lock the hw lock failed.\n");
		return ret;
	}

	if (AGCP_READL(&dsp_ac->state->ap_enable_cnt) > 0) {
		AGCP_WRITEL(AGCP_READL(&dsp_ac->state->ap_enable_cnt) - 1,
			&dsp_ac->state->ap_enable_cnt);
	}
	if ((AGCP_READL(&dsp_ac->state->ap_enable_cnt) == 0)
			&& (AGCP_READL(&dsp_ac->state->cp_enable_cnt) == 0)) {
		ret = regmap_update_bits(dsp_ac->agcp_ahb,
			REG_AON_APB_AGCP_CTRL,
			BIT_AON_APB_AGCP_TOP_ACCESS_EN, 0);
		pr_debug("%s,update register REG_AON_APB_AGCP_CTRL, ret=%d\n",
			__func__, ret);
	}
	hwspin_unlock_irqrestore(dsp_ac->hw_lock, &flags);

	pr_dbg("%s,dsp_ac->state->ap_enable_cnt=%d,dsp_ac->state->cp_enable_cnt=%d.\n",
		__func__, dsp_ac->state->ap_enable_cnt,
		dsp_ac->state->cp_enable_cnt);

	return 0;
}
EXPORT_SYMBOL(agdsp_access_disable);

int restore_access(void)
{
	int ret;
	unsigned long flags;
	int cnt = 0;
	int val;
	struct agdsp_access *dsp_ac = g_agdsp_access;

	pr_dbg("%s entry\n", __func__);
	if (!dsp_ac) {
		pr_err("agdsp access not init, exit\n");
		return -EINVAL;
	}

	if (!dsp_ac->ready || !dsp_ac->state) {
		pr_err("agdsp access not ready, dsp_ac->ready=%d, dsp_ac->state=%p\n",
			   dsp_ac->ready, dsp_ac->state);
		return -EINVAL;
	}
	ret = hwspin_lock_timeout_irqsave(dsp_ac->hw_lock,
					  HWSPINLOCK_TIMEOUT, &flags);
	if (ret) {
		pr_err("agdsp access, Read: lock the hw lock failed.\n");
		return ret;
	}

	ret = regmap_update_bits(dsp_ac->agcp_ahb,
		REG_AON_APB_AGCP_CTRL,
		BIT_AON_APB_AGCP_TOP_ACCESS_EN,
		BIT_AON_APB_AGCP_TOP_ACCESS_EN);
	if (ret != 0) {
		pr_err("%s, regmap_update_bits REG_AON_APB_AGCP_CTRL error!\n",
		__func__);
		goto exit;
	}

	ret = regmap_read(dsp_ac->agcp_ahb,
		REG_AON_APB_AGCP_CTRL, &val);
	if (ret != 0) {
		pr_err("%s, regmap_read REG_AON_APB_AGCP_CTRL error!\n",
			__func__);
		goto exit;
	}
	if (!(BIT_AON_APB_AGCP_TOP_ACCESS_EN & val)) {
		pr_err("%s, BIT_AON_APB_AGCP_TOP_ACCESS_EN not enable! value: %x\n",
			__func__, val);
		goto exit;
	}

	if (dsp_ac->state->ap_enable_cnt != 0) {
		/* send a mail to AGDSP to wake up it,
		 * 100 is an invalid command
		 */
		mbox_raw_sent(dsp_ac->mbx_core, 100);
		udelay(20);

		do {
			ret = regmap_read(dsp_ac->pmu_apb,
				REG_PMU_APB_PWR_STATE_DBG1, &val);
			pr_dbg("%s, regmap_read val=0x%x!\n", __func__, val);
			if (ret != 0) {
				pr_err("%s, regmap_read error!\n", __func__);
				break;
			}
			/* 0: power up finished;7:power off */
			if ((BIT_PMU_APB_PD_AGCP_SYS_STATE(0x1F) & val) == 0)
				break;
			udelay(5);
		} while (++cnt < TRY_CNT_MAX);
		if (cnt == TRY_CNT_MAX) {
			pr_err("ERR: %s wait agdsp power up timeout!\n",
				   __func__);
			ret = -EBUSY;
		}
	}
exit:
	regmap_read(dsp_ac->agcp_ahb, REG_AON_APB_AGCP_CTRL, &val);
	pr_info("%s, ap_enable_cnt=%d, cp_enable_cnt=%d.cnt=%d, access value=%#x\n",
			__func__, dsp_ac->state->ap_enable_cnt,
			dsp_ac->state->cp_enable_cnt, cnt, val);
	hwspin_unlock_irqrestore(dsp_ac->hw_lock, &flags);

	return ret;
}
EXPORT_SYMBOL(restore_access);

int disable_access_force(void)
{
	int ret;
	unsigned long flags;
	int retval;
	struct agdsp_access *dsp_ac = g_agdsp_access;

	if (!dsp_ac) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -EINVAL;
	}
	if (!dsp_ac->ready || !dsp_ac->state) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	ret = hwspin_lock_timeout_irqsave(dsp_ac->hw_lock,
				HWSPINLOCK_TIMEOUT, &flags);
	if (ret) {
		pr_err("agdsp access, Read: lock the hw lock failed.\n");
		return ret;
	}
	ret = regmap_update_bits(dsp_ac->agcp_ahb,
		REG_AON_APB_AGCP_CTRL,
		BIT_AON_APB_AGCP_TOP_ACCESS_EN, 0);
	regmap_read(dsp_ac->agcp_ahb, REG_AON_APB_AGCP_CTRL, &retval);
	pr_info("%s,update register REG_AON_APB_AGCP_CTRL, ret=%d, access value =%#x\n",
		__func__, ret, retval);

	hwspin_unlock_irqrestore(dsp_ac->hw_lock, &flags);

	return 0;
}
EXPORT_SYMBOL(disable_access_force);

#if defined(BIT_PMU_APB_XTL0_FRC_ON)
int force_on_xtl(bool on_off)
{
	int ret;
	struct agdsp_access *dsp_ac = g_agdsp_access;
	unsigned int mask;

	pr_dbg("%s entry\n", __func__);
	if (!dsp_ac)
		return -EINVAL;

	mask = BIT_PMU_APB_XTL0_FRC_ON;
	ret = regmap_update_bits(dsp_ac->pmu_apb, REG_PMU_APB_XTL0_REL_CFG,
				 mask, on_off ? mask : 0);
	if (ret) {
		pr_err("%s, regmap_update_bits REG_PMU_APB_XTL0_REL_CFG error!\n",
			__func__);
		return ret;
	}

	mask = BIT_PMU_APB_XTLBUF0_FRC_ON;
	ret = regmap_update_bits(dsp_ac->pmu_apb, REG_PMU_APB_XTLBUF0_REL_CFG,
				 mask, on_off ? mask : 0);
	if (ret)
		pr_err("%s, regmap_update_bits REG_PMU_APB_XTLBUF0_REL_CFG error!\n",
			__func__);

	return ret;
}
#else
int force_on_xtl(bool on_off) { return 0; }
#endif

#if defined(CONFIG_DEBUG_FS)
static int agdsp_access_debug_info_show(struct seq_file *m, void *private)
{
	uint32_t val = 0;

	seq_printf(m, "ap_enable_cnt:%d  cp_enable_cnt=%d\n",
		g_agdsp_access->state->ap_enable_cnt,
		g_agdsp_access->state->cp_enable_cnt);
	seq_printf(m, "hw_lock:0x%p\n", g_agdsp_access->hw_lock);
	seq_printf(m, "smem_phy_addr:0x%x  size=%d\n",
		g_agdsp_access->smem_phy_addr, g_agdsp_access->smem_size);
	seq_printf(m, "thread:0x%p\n", g_agdsp_access->thread);
	seq_printf(m, "ready:%d\n", g_agdsp_access->ready);
	seq_printf(m, "dst:%d  channel=%d\n",
		g_agdsp_access->dst, g_agdsp_access->channel);
	seq_printf(m, "agcp_ahb:0x%p  pmu_apb=0x%p\n",
		g_agdsp_access->agcp_ahb, g_agdsp_access->pmu_apb);

	regmap_read(g_agdsp_access->agcp_ahb, REG_AON_APB_AGCP_CTRL, &val);
	seq_printf(m, "REG_AON_APB_AGCP_CTRL:0x%x (Bit0 0:sleep en)\n", val);

	regmap_read(g_agdsp_access->pmu_apb, REG_PMU_APB_PWR_STATE_DBG1, &val);
	seq_printf(m,
		"REG_PMU_APB_PWR_STATE_DBG1:0x%x (Bit25-29: 0: power on ok; 111:power off)\n",
		val);

	regmap_read(g_agdsp_access->pmu_apb, REG_PMU_APB_SLEEP_CTRL, &val);
	seq_printf(m, "REG_PMU_APB_SLEEP_CTRL:0x%x (Bit4 1:sleep)\n", val);

	return 0;
}

static int agdsp_access_debug_info_open(struct inode *inode, struct file *file)
{
	return single_open(file,
		agdsp_access_debug_info_show, inode->i_private);
}

static const struct file_operations agdsp_access_debug_info_fops = {
	.open = agdsp_access_debug_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init agdsp_access_init_debugfs_info(void)
{
	debugfs_create_file("agdsp_access",
		S_IRUGO, NULL, NULL, &agdsp_access_debug_info_fops);

	return 0;
}

device_initcall(agdsp_access_init_debugfs_info);
#endif /* CONFIG_DEBUG_FS */

static int agdsp_access_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct regmap *agcp_ahb;
	struct regmap *pmu_apb;
	uint32_t mbx_core = 0;
	uint32_t dst = 0, channel = 0;
	int ret = 0;
	u32 offset;

	ret = of_property_read_u32(node, "mailbox,core", &mbx_core);
	if (ret) {
		pr_err("%s get mbx core err\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "sprd,dst", &dst);
	if (ret) {
		pr_err("%s get dst err\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "sprd,channel", &channel);
	if (ret) {
		pr_err("%s get channel err\n", __func__);
		return -EINVAL;
	}

	if (of_property_read_bool(node, "sprd,ddr-addr-offset")) {
		if (of_property_read_u32
		    (node, "sprd,ddr-addr-offset", &offset)) {
			pr_err("%s, parse 'sprd,ddr-addr-offset' failed!\n",
			       __func__);
			return -EINVAL;
		}
	} else
		offset = 0;
	pr_debug("%s ddr addr offset: %#x\n", __func__, offset);

	agcp_ahb = syscon_regmap_lookup_by_phandle(node,
		"sprd,syscon-agcp-ahb");
	pmu_apb = syscon_regmap_lookup_by_phandle(node,
		"sprd,syscon-pmu-apb");
	if (IS_ERR(agcp_ahb) || IS_ERR(pmu_apb)) {
		pr_err("%s, agcp-ahb or pmu_apb not exist!\n", __func__);
	} else {
		pr_debug("%s, agcp_ahb=0x%p, pmu_apb=0x%p\n",
			__func__, agcp_ahb, pmu_apb);
		return agdsp_access_initialize(agcp_ahb, pmu_apb, mbx_core, dst,
					 channel, offset);
	}

	return -EINVAL;
}

static const struct of_device_id agdsp_access_of_match[] = {
	{.compatible = "sprd,agdsp-access", },
	{ }
};

static struct platform_driver agdsp_access_driver = {
	.driver = {
		.name = "agdsp_access",
		.owner = THIS_MODULE,
		.of_match_table = agdsp_access_of_match,
	},
	.probe = agdsp_access_probe,
};

static int __init agdsp_access_init(void)
{
	int ret;

	ret = platform_driver_register(&agdsp_access_driver);

	return ret;
}

static void __exit agdsp_access_exit(void)
{
	platform_driver_unregister(&agdsp_access_driver);
}

module_init(agdsp_access_init);
module_exit(agdsp_access_exit);

MODULE_DESCRIPTION("agdsp access driver");
MODULE_AUTHOR("yintang.ren <yintang.ren@spreadtrum.com>");
MODULE_LICENSE("GPL");
