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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/pagemap.h>
#include <linux/sprd_iommu.h>
#include <linux/vmalloc.h>
#include <video/sprd_mm.h>
#include <video/sprd_img.h>
#include "isp_buf.h"
#include "isp_drv.h"
#include "isp_int.h"
#include "isp_path.h"
#include "isp_slice.h"
#include "gen_scale_coef.h"
#include "isp_slw.h"
#include "cam_pw_domain.h"
#include "video/sprd_mm.h"
#include "isp_3dnr.h"
#include "sprd_k_cpp.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_DRV: %d: %d " fmt, current->pid, __LINE__

struct platform_device *s_isp_pdev;
struct platform_device *s_isp1_pdev;
static atomic_t s_isp_users[ISP_MAX_COUNT];
static unsigned int s_isp_count;
unsigned long s_isp_regbase[ISP_MAX_COUNT];
unsigned long isp_phys_base[ISP_MAX_COUNT];
unsigned int s_isp_chnid[ISP_MAX_COUNT];
static struct clk *isp_mtx_eb;
static struct clk *cam_mtx_eb;
static struct clk *isp_clk[ISP_MAX_COUNT];
static struct clk *isp_clk_parent[ISP_MAX_COUNT];
static struct clk *isp_clk_default[ISP_MAX_COUNT];
static struct clk *isp_axi_eb[ISP_MAX_COUNT];
static struct clk *isp_eb;
static struct clk *isp_clk_eb[ISP_MAX_COUNT];
static struct clk *isp_iab_i_eb[ISP_MAX_COUNT];
struct isp_ch_irq s_isp_irq[ISP_MAX_COUNT];
static struct mutex isp_module_sema[ISP_MAX_COUNT];
static struct regmap *cam_ahb_gpr;

static struct wake_lock isp_wakelock[ISP_MAX_COUNT];
static char isp_wakelock_name[ISP_MAX_COUNT][30];
static spinlock_t isp_glb_reg_axi_lock[ISP_MAX_COUNT];
static spinlock_t isp_glb_reg_mask_lock[ISP_MAX_COUNT];
static spinlock_t isp_glb_reg_clr_lock[ISP_MAX_COUNT];
spinlock_t isp_mod_lock[ISP_MAX_COUNT];
#define ISP_AXI_STOP_TIMEOUT			1000
#define ISP_TEST
#define ISP_CLK_NUM				5
#define CLK_ISPPLL			(468000000 * 2)

static const struct isp_clk_tag isp_clk_tab[ISP_CLK_NUM] = {
	{77, "isp_clk_76m8"},
	{256, "isp_clk_256m"},
	{307, "isp_clk_307m2"},
	{384, "isp_clk_384m"},
	{576, "isp_clk_576m"},
};
static struct clk *isp_clk_par_tab[ISP_CLK_NUM];

void isp_reg_trace(enum isp_id idx)
{

#ifdef ISP_DRV_DEBUG
	unsigned long addr = 0;

	pr_info("ISP%d: Register list", idx);
	for (addr = ISP_INT_EN0; addr <= ISP_ALL_DONE_SRC_CTRL; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(idx, addr),
			ISP_REG_RD(idx, addr + 4),
			ISP_REG_RD(idx, addr + 8),
			ISP_REG_RD(idx, addr + 12));
	}
#endif
}
EXPORT_SYMBOL(isp_reg_trace);

static int offline_thread_loop(void *arg)
{
	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)arg;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	unsigned int is_irq = 0;
	unsigned long flag;
	enum isp_id idx = 0;

	if (dev == NULL) {
		pr_err("offline_thread_loop, dev is NULL\n");
		return -1;
	}
	fmcu_slice = &dev->fmcu_slice;
	idx = dev->idx;

	while (1) {
		if (wait_for_completion_interruptible(
			&dev->offline_thread_com) == 0) {
			if (atomic_cmpxchg(
					&dev->stop_offline_thread, 1, 0) == 1) {
				pr_info("ISP%d %s stop\n", idx, __func__);
				break;
			}
			spin_lock_irqsave(&isp_mod_lock[idx], flag);
			dev->is_wait_fmcu = 1;
			spin_unlock_irqrestore(&isp_mod_lock[idx], flag);
			if (wait_for_completion_interruptible(
					&dev->fmcu_com) == 0) {
				if (sprd_isp_fmcu_slice_proc(arg,
						DCAM_CAPTURE_START, is_irq)) {
					pr_info("%s: ISP%d start slice capture error\n",
							__func__, idx);
				}
			}
		} else {
			DCAM_TRACE("offline thread exit!");
			break;
		}
	}

	return 0;
}

static int isp_create_offline_thread(void *param)
{
	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)param;
	char thread_name[20] = { 0 };

	if (dev == NULL) {
		pr_err("dev is NULL\n");
		return -1;
	}

	dev->is_wait_fmcu = 0;
	init_completion(&dev->fmcu_com);
	complete(&dev->fmcu_com);
	atomic_set(&dev->stop_offline_thread, 0);
	init_completion(&dev->offline_thread_com);
	sprintf(thread_name, "isp%d_offline_thread", dev->idx);
	dev->offline_thread = kthread_run(offline_thread_loop,
					  param, thread_name);
	if (IS_ERR(dev->offline_thread)) {
		pr_err("isp_create_offline_thread error!\n");
		return -1;
	}

	return 0;
}

static int isp_stop_offline_thread(void *param)
{
	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)param;
	int cnt = 0;

	if (dev == NULL) {
		pr_err("dev is NULL\n");
		return -1;
	}

	if (dev->offline_thread) {
		atomic_set(&dev->stop_offline_thread, 1);
		complete(&dev->offline_thread_com);
		while (cnt < 1000) {
			cnt++;
			if (atomic_read(&dev->stop_offline_thread) == 0)
				break;
			udelay(1000);
		}
		dev->offline_thread = NULL;
	}

	return 0;
}


void isp_wait_update_done(struct isp_module *module,
			enum isp_path_index path_index,
			unsigned int *p_flag)
{
	int ret = 0;
	struct isp_path_desc *p_path = NULL;
	struct isp_path_desc *pre = NULL;
	struct isp_path_desc *vid = NULL;
	struct isp_path_desc *cap = NULL;
	enum isp_scl_id scl_path_id;
	unsigned long flag;
	enum isp_id idx = 0;

	if (!module)
		return;

	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];
	cap = &module->isp_path[ISP_SCL_CAP];

	idx = module->idx;
	if (path_index >= ISP_PATH_IDX_0 && path_index <= ISP_PATH_IDX_CAP) {
		scl_path_id = isp_get_scl_index(path_index);
		p_path = &module->isp_path[scl_path_id];
		pr_debug("ISP%d: update done wait %d, %d\n", idx,
			   p_path->wait_for_sof, p_path->sof_com.done);

		spin_lock_irqsave(&isp_mod_lock[idx], flag);
		if (p_flag) {
			*p_flag = 1;
			if (pre->valid)
				pre->is_update = 1;
			if (vid->valid)
				vid->is_update = 1;
			if (cap->valid)
				cap->is_update = 1;
			if (module->store_cce.valid)
				module->store_cce.is_update = 1;
		}
		p_path->wait_for_sof = 1;
		spin_unlock_irqrestore(&isp_mod_lock[idx], flag);
		ret = wait_for_completion_timeout(&p_path->sof_com,
						  ISP_PATH_TIMEOUT);
		if (ret == 0) {
			isp_reg_trace(idx);
			pr_err("ISP%d: failed to wait update path 0x%x done\n",
			       idx, path_index);
		}
	} else {
		pr_info("ISP%d: wrong index 0x%x\n", idx, path_index);
	}
}

static void isp_wait_path_done(struct isp_module *module,
	enum isp_path_index path_index, unsigned int *p_flag)
{
	int ret = 0;
	struct isp_path_desc *p_path = NULL;
	enum isp_scl_id scl_path_id;
	unsigned long flag;
	enum isp_id idx = 0;

	if (!module)
		return;

	idx = module->idx;
	if (path_index >= ISP_PATH_IDX_0 && path_index <= ISP_PATH_IDX_CAP) {
		scl_path_id = isp_get_scl_index(path_index);
		p_path = &module->isp_path[scl_path_id];

		pr_debug("path done wait %d, %d\n",
			   p_path->wait_for_done, p_path->tx_done_com.done);

		spin_lock_irqsave(&isp_mod_lock[idx], flag);
		if (p_flag)
			*p_flag = 1;

		p_path->wait_for_done = 1;
		spin_unlock_irqrestore(&isp_mod_lock[idx], flag);
		pr_info("begin to wait tx_done_com: %d\n",
			p_path->tx_done_com.done);
		ret = wait_for_completion_timeout(&p_path->tx_done_com,
						  ISP_PATH_TIMEOUT);

		if (ret == 0) {
			isp_reg_trace(idx);
			pr_err("ISP%d: failed to wait path 0x%x done\n", idx,
			       path_index);
		}
	} else {
		pr_info("ISP%d: wrong index 0x%x\n", idx, path_index);
		return;
	}
}

static void isp_wait_for_channel_stop(struct isp_module *module,
	enum isp_path_index path_index)
{
	unsigned int ret = 0;
	int time_out = 5000;
	struct isp_path_desc *path = NULL;
	/*unsigned int mask;*/

	if (!module)
		return;

	if (ISP_PATH_IDX_0 & path_index) {
		path = &module->isp_path[ISP_SCL_0];
		/*mask = BIT_17;*/
	} else if (ISP_PATH_IDX_PRE & path_index) {
		path = &module->isp_path[ISP_SCL_PRE];
		/*mask = BIT_18;*/
	} else if (ISP_PATH_IDX_VID & path_index) {
		path = &module->isp_path[ISP_SCL_VID];
		/*mask = BIT_19;*/
	} else if (ISP_PATH_IDX_CAP & path_index) {
		path = &module->isp_path[ISP_SCL_CAP];
		/*mask = BIT_21;*/
	} else {
		pr_err("path index 0x%0x, error!\n", path_index);
		return;
	}

	/* wait for AHB path busy cleared */
	/*while (path->valid && time_out) {
	*	ret = ISP_REG_RD(idx, DCAM_AHBM_STS) & mask;
	*	if (!ret)
	*		break;
	*	time_out--;
	*}
	*/
	pr_debug("ISP%d:wait channel 0x%0x stop %d %d\n",
		module->idx, path_index, ret, time_out);
}


static void isp_quickstop_set_all(struct isp_module *module)
{
	if (!module)
		return;

	isp_wait_for_channel_stop(module, ISP_PATH_IDX_PRE);
	isp_wait_for_channel_stop(module, ISP_PATH_IDX_VID);
	isp_wait_for_channel_stop(module, ISP_PATH_IDX_CAP);
}


static void isp_quickstop_set(struct isp_module *module,
	enum isp_path_index path_index, unsigned int cfg_bit,
	unsigned int ahbm_bit)
{
	if (!module)
		return;

	isp_wait_for_channel_stop(module, path_index);
}

static void isp_wait_for_quickstop(struct isp_module *module,
	enum isp_path_index path_index)
{
	enum isp_id idx = 0;

	if (!module)
		return;

	idx = module->idx;
	if (path_index == ISP_PATH_IDX_ALL) {
		isp_quickstop_set_all(module);
	} else {
		/* need to check isp path */
		if (module->isp_path[ISP_SCL_PRE].valid &&
			(ISP_PATH_IDX_PRE & path_index)) {
			isp_quickstop_set(module,
				ISP_PATH_IDX_PRE, BIT_1, BIT_4);
		}
		if (module->isp_path[ISP_SCL_VID].valid &&
			(ISP_PATH_IDX_VID & path_index)) {
			isp_quickstop_set(module,
				ISP_PATH_IDX_VID, BIT_2, BIT_5);
		}
		if (module->isp_path[ISP_SCL_CAP].valid &&
			(ISP_PATH_IDX_CAP & path_index)) {
			isp_quickstop_set(module,
				ISP_PATH_IDX_CAP, BIT_3, BIT_20);
		}
	}

}

static int isp_stop_path(struct isp_pipe_dev *dev,
	enum isp_path_index path_index)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_path_desc *p_path = NULL;
	struct isp_module *module = NULL;
	unsigned long flag;
	enum isp_id idx = 0;

	if (!dev) {
		pr_err("Input dev ptr is NULL\n");
		return -EFAULT;
	}

	module = &dev->module_info;
	idx = module->idx;
	spin_lock_irqsave(&isp_mod_lock[idx], flag);

	if (path_index >= ISP_PATH_IDX_0 && path_index <= ISP_PATH_IDX_CAP) {
		if (path_index == ISP_PATH_IDX_0)
			p_path = &module->isp_path[ISP_SCL_0];
		else if (path_index == ISP_PATH_IDX_PRE)
			p_path = &module->isp_path[ISP_SCL_PRE];
		else if (path_index == ISP_PATH_IDX_VID)
			p_path = &module->isp_path[ISP_SCL_VID];
		else if (path_index == ISP_PATH_IDX_CAP)
			p_path = &module->isp_path[ISP_SCL_CAP];

		isp_wait_for_quickstop(module, path_index);
		p_path->status = ISP_ST_STOP;
		p_path->valid = 0;
	} else {
		pr_info("ISP%d: stop path, wrong index 0x%x\n",
			idx, path_index);
		spin_unlock_irqrestore(&isp_mod_lock[idx], flag);
		return -rtn;
	}

	isp_frm_clear(dev, path_index);
	spin_unlock_irqrestore(&isp_mod_lock[idx], flag);

	return rtn;
}


int sprd_isp_stop_path(void *isp_handle, enum isp_path_index path_index)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	enum isp_id idx = 0;

	if (isp_handle == NULL) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	idx = module->idx;
	pr_info("ISP%d: stop path 0x%x\n", idx, path_index);
	if (path_index < ISP_PATH_IDX_0 || path_index > ISP_PATH_IDX_ALL) {
		pr_err("ISP%d: error path index %d\n", idx, path_index);
		return -rtn;
	}

	if ((ISP_PATH_IDX_PRE & path_index) &&
		module->isp_path[ISP_SCL_PRE].valid) {
		pr_debug("ISP%d: stop path pre In\n", idx);
		isp_wait_path_done(module, ISP_PATH_IDX_PRE,
			&module->isp_path[ISP_SCL_PRE].need_stop);
		isp_stop_path(dev, ISP_PATH_IDX_PRE);
	}

	if ((ISP_PATH_IDX_VID & path_index) &&
		module->isp_path[ISP_SCL_VID].valid) {
		pr_debug("ISP%d: stop path vid In\n", idx);
		isp_wait_path_done(module, ISP_PATH_IDX_VID,
			&module->isp_path[ISP_SCL_VID].need_stop);
		isp_stop_path(dev, ISP_PATH_IDX_VID);
	}

	if ((ISP_PATH_IDX_CAP & path_index) &&
		module->isp_path[ISP_SCL_CAP].valid) {
		pr_debug("ISP%d: stop path cap In\n", idx);
		isp_wait_path_done(module, ISP_PATH_IDX_CAP,
			&module->isp_path[ISP_SCL_CAP].need_stop);
		isp_stop_path(dev, ISP_PATH_IDX_CAP);
	}

	return -rtn;
}

static void sprd_isp_glb_reg_awr(enum isp_id idx, unsigned long addr,
			   unsigned int val, unsigned int reg_id)
{
	unsigned long flag;

	switch (reg_id) {
	case ISP_AXI_REG:
		spin_lock_irqsave(&isp_glb_reg_axi_lock[idx], flag);
		ISP_REG_WR(idx, addr, ISP_REG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&isp_glb_reg_axi_lock[idx], flag);
		break;
	default:
		ISP_REG_WR(idx, addr, ISP_REG_RD(idx, addr) & (val));
		break;
	}
}

static void sprd_isp_glb_reg_owr(enum isp_id idx, unsigned long addr,
			   unsigned int val, unsigned int reg_id)
{
	unsigned long flag;

	switch (reg_id) {
	case ISP_AXI_REG:
		spin_lock_irqsave(&isp_glb_reg_axi_lock[idx], flag);
		ISP_REG_WR(idx, addr, ISP_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&isp_glb_reg_axi_lock[idx], flag);
		break;
	case ISP_INIT_MASK_REG:
		spin_lock_irqsave(&isp_glb_reg_mask_lock[idx], flag);
		ISP_REG_WR(idx, addr, ISP_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&isp_glb_reg_mask_lock[idx], flag);
		break;
	case ISP_INIT_CLR_REG:
		spin_lock_irqsave(&isp_glb_reg_clr_lock[idx], flag);
		ISP_REG_WR(idx, addr, ISP_REG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&isp_glb_reg_clr_lock[idx], flag);
		break;
	default:
		ISP_REG_WR(idx, addr, ISP_REG_RD(idx, addr) | (val));
		break;
	}
}

static int sprd_isp_reset(enum isp_id idx)
{
	enum dcam_drv_rtn rtn = ISP_RTN_SUCCESS;
	unsigned int time_out = 0;
	unsigned int flag = 0;

	pr_info("[%s] enter ISP%d: reset:\n", __func__, idx);

	/* then wait for AHB busy cleared */
	while (++time_out < ISP_AXI_STOP_TIMEOUT) {
		if (1 == ((ISP_REG_RD(idx, ISP_INT_STATUS) & BIT_3) >> 3))
			break;
	}

	if (time_out >= ISP_AXI_STOP_TIMEOUT) {
		pr_info("ISP%d: reset timeout %d\n", idx, time_out);
		return ISP_RTN_TIME_OUT;
	}


	if (idx == 0)
		regmap_update_bits(cam_ahb_gpr,
			REG_MM_AHB_MODULE_RST, BIT_2, BIT_2);
	else
		regmap_update_bits(cam_ahb_gpr,
			REG_MM_AHB_MODULE_RST, BIT_3, BIT_3);

	if (idx == 0)
		flag = BIT(2) | BIT(3) | BIT(7) | BIT(14) | BIT(21);
	else
		flag = BIT(10) | BIT(11) | BIT(9) | BIT(13) | BIT(20);

	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, flag);

	udelay(1);

	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, ~flag);

	if (idx == 0)
		regmap_update_bits(cam_ahb_gpr,
			REG_MM_AHB_MODULE_RST, BIT_2, ~BIT_2);
	else
		regmap_update_bits(cam_ahb_gpr,
			REG_MM_AHB_MODULE_RST, BIT_3, ~BIT_3);

	sprd_isp_glb_reg_owr(idx, ISP_INT_CLR0,
		0xFFFFFFFF, ISP_INIT_CLR_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_CLR1,
		0xFFFFFFFF, ISP_INIT_CLR_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_CLR2,
		0xFFFFFFFF, ISP_INIT_CLR_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_CLR3,
		0xFFFFFFFF, ISP_INIT_CLR_REG);
	sprd_isp_glb_reg_awr(idx, ISP_INT_EN0,
		0x0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_awr(idx, ISP_INT_EN1,
		0x0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_awr(idx, ISP_INT_EN2,
		0x0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_awr(idx, ISP_INT_EN3,
		0x0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_EN0,
		ISP_INT_LINE_MASK0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_EN1,
		ISP_INT_LINE_MASK1, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_EN2,
		ISP_INT_LINE_MASK2, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_EN3,
		ISP_INT_LINE_MASK3, ISP_INIT_MASK_REG);

	sprd_isp_glb_reg_owr(idx, ISP_INT_CLR0+ISP_CH1_ADDR_OFFSET,
		0xFFFFFFFF, ISP_INIT_CLR_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_CLR1+ISP_CH1_ADDR_OFFSET,
		0xFFFFFFFF, ISP_INIT_CLR_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_CLR2+ISP_CH1_ADDR_OFFSET,
		0xFFFFFFFF, ISP_INIT_CLR_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_CLR3+ISP_CH1_ADDR_OFFSET,
		0xFFFFFFFF, ISP_INIT_CLR_REG);
	sprd_isp_glb_reg_awr(idx, ISP_INT_EN0+ISP_CH1_ADDR_OFFSET,
		0x0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_awr(idx, ISP_INT_EN1+ISP_CH1_ADDR_OFFSET,
		0x0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_awr(idx, ISP_INT_EN2+ISP_CH1_ADDR_OFFSET,
		0x0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_awr(idx, ISP_INT_EN3+ISP_CH1_ADDR_OFFSET,
		0x0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_EN0+ISP_CH1_ADDR_OFFSET,
		ISP_INT_LINE_MASK0, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_EN1+ISP_CH1_ADDR_OFFSET,
		ISP_INT_LINE_MASK1, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_EN2+ISP_CH1_ADDR_OFFSET,
		ISP_INT_LINE_MASK2, ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_owr(idx, ISP_INT_EN3+ISP_CH1_ADDR_OFFSET,
		ISP_INT_LINE_MASK3, ISP_INIT_MASK_REG);

	sprd_isp_glb_reg_awr(idx, ISP_AXI_AXIM_CTRL, ~BIT_26, ISP_AXI_REG);

	pr_info("ISP%d:end\n", idx);
	return -rtn;
}

static void isp_quickstop(struct isp_module *module)
{
	enum isp_id idx = 0;

	if (module == NULL) {
		pr_err("zero pointer\n");
		return;
	}

	idx = module->idx;

	sprd_isp_glb_reg_owr(idx, ISP_AXI_AXIM_CTRL, BIT_26, ISP_AXI_REG);

	pr_info("ISP%d:ISP_AXI_AXIM_CTRL 0x%x STATUS 0x%x\n",
		idx, ISP_REG_RD(idx, ISP_AXI_AXIM_CTRL),
		ISP_REG_RD(idx, ISP_INT_STATUS));
	udelay(10);

}

int sprd_isp_stop(void *isp_handle, int is_irq)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	unsigned long flag;
	unsigned int i = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_statis_module *statis_module = NULL;
	struct isp_k_block *isp_k_param = NULL;
	enum isp_id idx = 0;
	struct device *p_dev = NULL;

	if (isp_handle == NULL) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}
	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	statis_module = &dev->statis_module_info;
	isp_k_param = &dev->isp_k_param;
	idx = module->idx;
	dev->pdaf_status[idx] = ISP_PDAF_INIT_FLAG;

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	isp_cpp_deinit(isp_handle);
#endif
	if (!is_irq)
		spin_lock_irqsave(&isp_mod_lock[idx], flag);

	isp_quickstop(module);

	if (dev->fmcu_slw.status == ISP_ST_START)
		isp_slw_clear(isp_handle);

	if (dev->fmcu_slw.slw_flags) {
		dev->fmcu_slw.slw_flags = 0;
		dev->fmcu_slw.vid_num = 0;
	}

	if (dev->is_wait_fmcu == 1) {
		complete(&dev->fmcu_com);
		dev->is_wait_fmcu = 0;
	}
	dev->clk_update_count = 0;
	for (i = ISP_SCL_0; i < ISP_SCL_3DNR; i++) {
		module->isp_path[i].status = ISP_ST_STOP;
		module->isp_path[i].valid = 0;
		module->isp_path[i].shadow_done_cnt = 0;
		module->isp_path[i].skip_num = 0;
		module->isp_path[i].frm_cnt = 0;
		module->isp_path[i].is_clk_update = 0;
		isp_frm_clear(dev, 1 << i);
	}
	module->store_cce.valid = 0;
	module->store_cce.status = ISP_ST_STOP;
	module->store_cce.shadow_done_cnt = 0;
	module->store_cce.frm_cnt = 0;
	module->store_cce.zsl_pw_save = 0;
	module->store_cce.pw_save = 0;
	module->store_cce.is_raw_cap = 0;
	isp_buf_queue_init(&module->store_cce.tmp_buf_queue);
	isp_frm_queue_clear(&module->store_cce.frame_queue);
	isp_frm_queue_clear(&module->store_cce.zsl_queue);

	isp_coeff_queue_init(&module->isp_scl_array[idx]->scl0_queue);
	isp_coeff_queue_init(&module->isp_scl_array[idx]->pre_queue);
	isp_coeff_queue_init(&module->isp_scl_array[idx]->vid_queue);
	isp_coeff_queue_init(&module->isp_scl_array[idx]->cap_queue);
	isp_coeff_queue_init(&module->isp_scl_array[idx]->store_cce_queue);

	pfiommu_free_addr(&statis_module->img_statis_buf.pfinfo);
	memset(&statis_module->img_statis_buf.pfinfo, 0,
	       sizeof(struct pfiommu_info));

	pfiommu_free_addr(&isp_k_param->fetch_pfinfo);
	memset(&isp_k_param->fetch_pfinfo, 0, sizeof(struct pfiommu_info));

	pfiommu_free_addr(&isp_k_param->lsc_pfinfo);
	memset(&isp_k_param->lsc_pfinfo, 0, sizeof(struct pfiommu_info));

	isp_storecce_buf_iommu_unmap(&module->store_cce, idx);
	isp_fmcu_iommu_unmap(&dev->fmcu_slice, idx);

	if (!is_irq)
		spin_unlock_irqrestore(&isp_mod_lock[idx], flag);
	sprd_isp_reset(idx);

	if (idx == ISP_ID_0)
		p_dev = &s_isp_pdev->dev;
	else
		p_dev = &s_isp1_pdev->dev;
	if (sprd_iommu_attach_device(p_dev) == 0)
		sprd_iommu_release(p_dev);


	pr_info("isp stop\n");

	return -rtn;
}

static int get_slice_init_scaler(struct slice_scaler_path *scaler,
	struct isp_path_desc *path)
{
	if (!scaler || !path) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	scaler->trim0_size_x = path->trim0_info.size_x;
	scaler->trim0_size_y = path->trim0_info.size_y;
	scaler->trim0_start_x = path->trim0_info.start_x;
	scaler->trim0_start_y = path->trim0_info.start_y;

	if (path->deci_info.deci_x_eb)
		scaler->deci_x = 1 << (path->deci_info.deci_x + 1);
	else
		scaler->deci_x = 1;

	if (path->deci_info.deci_y_eb)
		scaler->deci_y = 1 << (path->deci_info.deci_y + 1);
	else
		scaler->deci_y = 1;

	scaler->odata_mode = path->odata_mode;
	scaler->scaler_bypass = path->scaler_bypass;
	scaler->scaler_out_width = path->dst.w;
	scaler->scaler_out_height = path->dst.h;
	scaler->scaler_factor_in = path->scaler_info.scaler_factor_in;
	scaler->scaler_ver_factor_in = path->scaler_info.scaler_ver_factor_in;
	scaler->scaler_factor_out = path->scaler_info.scaler_factor_out;
	scaler->scaler_ver_factor_out = path->scaler_info.scaler_ver_factor_out;
	scaler->scaler_y_ver_tap = path->scaler_info.scaler_y_ver_tap;
	scaler->scaler_uv_ver_tap = path->scaler_info.scaler_uv_ver_tap;

	return 0;
}

static int isp_update_offline_path_param(struct isp_pipe_dev *dev,
					 struct camera_frame *frame)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_path_desc *scl0 = NULL;
	struct isp_path_desc *pre = NULL;
	struct isp_path_desc *vid = NULL;
	struct isp_path_desc *cap = NULL;
	struct isp_store_cce_desc *store_cce = NULL;
	struct isp_module *module = NULL;
	enum isp_id idx = ISP_ID_0;
	struct isp_sc_array *scl_array = NULL;

	if (!dev || !frame) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	idx = dev->idx;
	module = &dev->module_info;
	scl0 = &module->isp_path[ISP_SCL_0];
	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];
	cap = &module->isp_path[ISP_SCL_CAP];
	store_cce = &module->store_cce;
	scl_array = module->isp_scl_array[idx];

	if (module->isp_path[ISP_SCL_CAP].valid) {
		cap->src.w = frame->width;
		cap->src.h = frame->height;
		cap->trim0_info.start_x = 0;
		cap->trim0_info.start_y = 0;
		cap->trim0_info.size_x = cap->src.w;
		cap->trim0_info.size_y = cap->src.h;
		cap->dst.w = cap->out_size.w;
		cap->dst.h = cap->out_size.h;
		cap->trim1_info.start_x = 0;
		cap->trim1_info.start_y = 0;
		cap->trim1_info.size_x = cap->dst.w;
		cap->trim1_info.size_y = cap->dst.h;
		rtn = isp_path_scaler(module, ISP_PATH_IDX_CAP, cap,
			&scl_array->coeff[ISP_SCL_CAP]);
		if (rtn) {
			pr_err("%s err, code %d", __func__, rtn);
			return -(rtn);
		}
		isp_path_set(module, cap, ISP_PATH_IDX_CAP);
	}

	return rtn;
}
static int get_slice_init_param(struct slice_param_in *in_ptr,
	struct isp_pipe_dev *dev, unsigned int cap_flag)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_path_desc *path_pre = NULL;
	struct isp_path_desc *path_vid = NULL;
	struct isp_path_desc *path_cap = NULL;
	struct isp_store_cce_desc *store_cce = NULL;
	struct camera_frame *frame = NULL;
	struct slice_scaler_path *scaler = NULL;
	struct isp_module *module = NULL;
	enum isp_id idx = ISP_ID_0;

	if (!dev || !in_ptr) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	idx = dev->idx;
	module = &dev->module_info;
	path_pre = &module->isp_path[ISP_SCL_PRE];
	path_vid = &module->isp_path[ISP_SCL_VID];
	path_cap = &module->isp_path[ISP_SCL_CAP];
	store_cce = &module->store_cce;

	in_ptr->idx = dev->idx;
	frame = &dev->capture_frame;

	pr_debug("fmcu yaddr 0x%x yaddr_vir 0x%x fd 0x%x\n",
		frame->yaddr, frame->yaddr_vir, frame->pfinfo.mfd[0]);
	ret = pfiommu_get_addr(&frame->pfinfo);
	if (ret) {
		pr_err("storecce get buffer failed: iommu mapping error.\n");
		return -1;
	}

	in_ptr->fetchYUV_addr.chn0 = frame->yaddr;
	in_ptr->fetchYUV_addr.chn1 = frame->uaddr;
	in_ptr->fetchYUV_addr.chn2 = frame->vaddr;
	in_ptr->img_size.width = frame->width;
	in_ptr->img_size.height = frame->height;
	in_ptr->fetchYUV_format = store_cce->store_info.color_format;
	in_ptr->fmcu_addr_vir = dev->fmcu_slice.iommu_addr_vir;

	if (path_pre->valid && path_pre->path_mode == ISP_PRE_OFFLINE) {
		in_ptr->pre_slice_need = 1;
		in_ptr->store_frame[SLICE_PATH_PRE].format =
			path_pre->store_info.color_format;
		in_ptr->store_frame[SLICE_PATH_PRE].size.width =
			path_pre->dst.w;
		in_ptr->store_frame[SLICE_PATH_PRE].size.height =
			path_pre->dst.h;
		scaler = &in_ptr->scaler_frame[SLICE_PATH_PRE];
		ret = get_slice_init_scaler(scaler, path_pre);
		ret = isp_path_set_next_frm(module, ISP_PATH_IDX_PRE,
			&in_ptr->store_frame[SLICE_PATH_PRE].addr);
	} else
		in_ptr->pre_slice_need = 0;

	if (path_vid->valid && path_vid->path_mode == ISP_VID_OFFLINE) {
		in_ptr->vid_slice_need = 1;
		in_ptr->store_frame[SLICE_PATH_VID].format =
			path_vid->store_info.color_format;
		in_ptr->store_frame[SLICE_PATH_VID].size.width =
			path_vid->dst.w;
		in_ptr->store_frame[SLICE_PATH_VID].size.height =
			path_vid->dst.h;
		scaler = &in_ptr->scaler_frame[SLICE_PATH_VID];
		ret = get_slice_init_scaler(scaler, path_vid);
		ret = isp_path_set_next_frm(module, ISP_PATH_IDX_VID,
			&in_ptr->store_frame[SLICE_PATH_VID].addr);
	} else
		in_ptr->vid_slice_need = 0;

	if (path_cap->valid && cap_flag) {
		in_ptr->cap_slice_need = 1;
		in_ptr->store_frame[SLICE_PATH_CAP].format =
			path_cap->store_info.color_format;
		in_ptr->store_frame[SLICE_PATH_CAP].size.width =
			path_cap->dst.w;
		in_ptr->store_frame[SLICE_PATH_CAP].size.height =
			path_cap->dst.h;
		scaler = &in_ptr->scaler_frame[SLICE_PATH_CAP];
		ret = get_slice_init_scaler(scaler, path_cap);
		ret = isp_path_set_next_frm(module, ISP_PATH_IDX_CAP,
			&in_ptr->store_frame[SLICE_PATH_CAP].addr);
	} else {
		in_ptr->cap_slice_need = 0;
	}

	in_ptr->is_raw_capture = dev->is_raw_capture;

	pr_debug("End get slice init param\n");
	return ret;
}

void sprd_isp_cap_hblank_cfg(void *handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_path_desc *cap_path = NULL;
	unsigned int zoom = 1;
	unsigned int hblank0 = 2500;
	unsigned int hblank1 = 2500;
	unsigned int factor = 200;
	unsigned int width = 0;
	unsigned int height = 0;

	if (!handle) {
		pr_err("handle zero pointer\n");
		return;
	}

	dev = (struct isp_pipe_dev *)handle;

	cap_path = &dev->module_info.isp_path[ISP_SCL_CAP];

	if (!cap_path) {
		pr_err("cap_path zero pointer\n");
		return;
	}

	width = cap_path->out_size.w;
	height = cap_path->out_size.h;

	if (cap_path->trim0_info.size_x == 0 || dev->is_raw_capture == 1)
		return;
	zoom = width / cap_path->trim0_info.size_x;

	if (width % cap_path->trim0_info.size_x != 0)
		zoom += 1;

	/* To resolve isp overflow */
	if (zoom > 3)
		zoom = 3;

	hblank1 = (width + hblank0) * (height + factor) /
		(height / zoom + factor) - (width / zoom);


	ISP_REG_MWR(dev->idx, ISP_COMMON_YUV_PATH_SEL_CH1, 0x3, 0);
	if (dev->cap_flag == DCAM_CAPTURE_START_3DNR) {
		ISP_REG_OWR(dev->idx, ISP_INT_DONE_CTRL, BIT_8);

		ISP_REG_WR(dev->idx, ISP_FETCH2_LINE_DLY_CTRL, 0x200);
		ISP_REG_WR(dev->idx, ISP_COMMON_SCL_PATH_SEL, 0x1F);
	}

	ISP_REG_WR(dev->idx, ISP_FETCH2_LINE_DLY_CTRL, hblank1);

	pr_debug("out:w:%d h:%d, in:w:%d h:%d, hblank1:0x%x, zoom:%d\n",
		cap_path->out_size.w, cap_path->out_size.h,
		width, height, hblank1, zoom);
}

#ifdef ISP_PW_SAVE
int sprd_isp_store_cce_bypass(void *handle,
		unsigned int cap_flag, unsigned int bypass)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_store_cce_desc *store_cce = NULL;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct isp_module *module = NULL;
	enum isp_id idx = ISP_ID_0;

	dev = (struct isp_pipe_dev *)handle;
	idx = dev->idx;

	fmcu_slice = &dev->fmcu_slice;
	module = &dev->module_info;
	store_cce = &module->store_cce;
	pr_info("sprd_isp_store_cce_bypass cap_sceme %d, bypass %d\n",
		cap_flag, bypass);
	dev->cap_scene = cap_flag;
	dev->cap_flag = cap_flag;

	if (!bypass)
		dev->cap_status_flag = ISP_CAP_INIT;
	else
		dev->cap_status_flag = ISP_CAP_STOP;

	if ((dev->cap_scene == DCAM_CAPTURE_START_3DNR ||
		dev->cap_scene == DCAM_CAPTURE_START_HDR) &&
		bypass) {
		pr_info("sprd_isp_store_cce_bypass needn't set on 3dnr\n");
		return 0;
	}

	pr_info("set storecce bypass to %d, storecce_state:%d\n",
		bypass, fmcu_slice->storecce_state);
	ISP_REG_MWR(idx, ISP_STORE_CCE_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_STORE_CCE_SHADOW_CLR, BIT_0, 1);
	if (!bypass) {
		if (store_cce->zsl_pw_save == 1) {
			if (isp_storecce_buf_reset(store_cce))
				pr_err("isp storecce reset error\n");

			if (isp_storecce_set_next_frm(module))
				pr_err("Set storecce next frame error\n");
		}
		fmcu_slice->storecce_state = ISP_ST_START;
		fmcu_slice->capture_state = ISP_ST_START;
		store_cce->pw_save = 0;
	} else {
		fmcu_slice->storecce_state = ISP_ST_STOP;
		fmcu_slice->capture_state = ISP_ST_STOP;
		store_cce->pw_save = 1;
	}

	return 0;
}
#endif

int sprd_isp_start_fmcu(void *handle,
			struct sprd_img_capture_param capture_param,
			unsigned int is_irq)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct isp_pipe_dev *dev = NULL;
	enum isp_id idx = ISP_ID_0;
	unsigned long flag;
	unsigned int storecce_state;
	struct isp_store_cce_desc *store_cce = NULL;
	struct camera_frame frame;
	struct isp_module *module = NULL;
	unsigned int cap_flag = capture_param.type;

	if (!handle) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)handle;
	idx = dev->idx;

	if (!is_irq)
		spin_lock_irqsave(&isp_mod_lock[idx], flag);

	fmcu_slice = &dev->fmcu_slice;
	module = &dev->module_info;
	store_cce = &module->store_cce;

	if (!is_irq)
		dev->cap_flag = cap_flag;
	pr_debug("cap_flag %d\n", dev->cap_flag);
	fmcu_slice->capture_state = ISP_ST_START;
	storecce_state = fmcu_slice->storecce_state;

	pr_debug("%s,idx:%d, iommu_addr_vir:%p\n", __func__,
			idx, fmcu_slice->iommu_addr_vir);

	if (store_cce->zsl_queue.valid_cnt == 0
	    || cap_flag == DCAM_CAPTURE_START_WITH_FLASH
	    || cap_flag == DCAM_CAPTURE_START_HDR
	    || cap_flag == DCAM_CAPTURE_START_3DNR) {
		pr_info("storecce wait next zsl frame\n");
		dev->clr_queue = 1;
		dev->cap_cur_cnt = 0;
		ret = ISP_RTN_SUCCESS;
		goto exit;
	}

	if (cap_flag == DCAM_CAPTURE_START_WITH_TIMESTAMP) {
		if (isp_frame_dequeue_withtime(&store_cce->zsl_queue,
					&frame, capture_param.timestamp)) {
			pr_err("storecce frame deque error\n");
			goto exit; }
	} else if (isp_frame_dequeue(&store_cce->zsl_queue, &frame)) {
		pr_err("storecce frame deque error\n");
		goto exit;
	}

	memcpy(&dev->capture_frame, &frame,
	       sizeof(struct camera_frame));
	if (isp_buf_queue_write(&store_cce->tmp_buf_queue, &frame)) {
		pr_err("storecce write buffer error\n");
		goto exit;
	}
	fmcu_slice->storecce_state = ISP_ST_STOP;
	complete(&dev->offline_thread_com);

exit:
	if (!is_irq)
		spin_unlock_irqrestore(&isp_mod_lock[idx], flag);

	return ret;
}

void sprd_isp_reset_fmcu(void)
{
	unsigned int flag = 0;

	flag = BIT(15);
	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, flag);
	udelay(1);
	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, ~flag);
}


int sprd_isp_fmcu_slice_proc(void *handle,
		unsigned int cap_flag, unsigned int is_irq)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct slice_param_in slice_in = {0};
	unsigned int fmcu_num = 0;
	unsigned int reg = 0;
	enum isp_id idx = ISP_ID_0;
	unsigned long flag;

	if (!handle) {
		pr_err("%s zero pointer\n", __func__);
		return -EFAULT;
	}
	dev = (struct isp_pipe_dev *)handle;
	idx = dev->idx;
	fmcu_slice = &dev->fmcu_slice;

	if (dev->is_raw_capture != 1)
		isp_update_offline_path_param(dev, &dev->capture_frame);
	if (!is_irq)
		spin_lock_irqsave(&isp_mod_lock[idx], flag);

	pr_debug("%s, iommu_addr_vir:%p, phy_addr %lx\n", __func__,
			fmcu_slice->iommu_addr_vir, fmcu_slice->iommu_addr_phy);

	ret = get_slice_init_param(&slice_in, dev, cap_flag);
	if (ret) {
		ret = ISP_RTN_SUCCESS;
		pr_info("get slice init param need new frame\n");
		goto exit;
	}
	sprd_isp_cap_hblank_cfg(handle);

	if (idx == 0)
		ISP_REG_WR(0, ISP_FMCU_ISP_REG_REGION, 0x0D2F0D2E);
	else
		ISP_REG_WR(1, ISP_FMCU_ISP_REG_REGION, 0x0D300D2F);

	ret = isp_fmcu_slice_cfg(idx, fmcu_slice->slice_handle,
		&slice_in, &fmcu_num);
	if (ret) {
		pr_err("get fmcu slice cfg error\n");
		goto exit;
	}
	fmcu_slice->fmcu_num = fmcu_num;

#ifdef ISP_DRV_DEBUG
	{
		unsigned int i = 0;
		unsigned long addr = 0;

		addr = (unsigned long)fmcu_slice->iommu_addr_vir;

		pr_info("fmcu slice cmd num %d\n", fmcu_slice->fmcu_num);
		for (i = 0; i <= fmcu_slice->fmcu_num; i += 2) {
			pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
					addr,
					REG_RD(addr),
					REG_RD(addr + 4),
					REG_RD(addr + 8),
					REG_RD(addr + 12));
			addr += 16;
		}
	}

#endif

	/*fmcu reset:frame mode */
	/*sprd_isp_reset_fmcu();   why sharkL2 add this. */

	clflush_cache_range((void *)fmcu_slice->iommu_addr_vir, PAGE_SIZE);

	reg = ISP_FMCU_DDR_ADR + ISP_CH1_ADDR_OFFSET;
	ISP_REG_WR(idx, reg, fmcu_slice->iommu_addr_phy);
	reg = ISP_FMCU_CTRL + ISP_CH1_ADDR_OFFSET;
	ISP_REG_MWR(idx, reg, 0xFFFF0000,
		fmcu_slice->fmcu_num << 16);
	reg = ISP_FMCU_START + ISP_CH1_ADDR_OFFSET;
	ISP_REG_MWR(idx, reg, BIT_0, 1);

exit:
	if (!is_irq)
		spin_unlock_irqrestore(&isp_mod_lock[idx], flag);
	pr_debug("%s done.\n", __func__);

	return ret;
}

int sprd_isp_fmcu_slice_stop(void *handle)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct isp_pipe_dev *dev = NULL;
	enum isp_id idx = ISP_ID_0;
	unsigned long flag;

	if (!handle) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)handle;
	idx = dev->idx;
	fmcu_slice = &dev->fmcu_slice;

	spin_lock_irqsave(&isp_mod_lock[idx], flag);
	fmcu_slice->capture_state = ISP_ST_STOP;
	dev->clr_queue = 0;
	dev->cap_cur_cnt = 0;
	dev->cap_flag = DCAM_CAPTURE_STOP;
	spin_unlock_irqrestore(&isp_mod_lock[idx], flag);

	return ret;
}

static unsigned int isp_get_path_deci_factor(unsigned int src_size,
					      unsigned int dst_size)
{
	unsigned int factor = 0;

	if (0 == src_size || 0 == dst_size)
		return factor;

	/* factor: 0 - 1/2, 1 - 1/4, 2 - 1/8, 3 - 1/16 */
	for (factor = 0; factor < CAMERA_PATH_DECI_FAC_MAX; factor++) {
		if (src_size < (unsigned int) (dst_size * (1 << (factor + 1))))
			break;
	}

	return factor;
}

static int isp_calc_sc_size(enum isp_id idx, struct isp_path_desc *path)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	unsigned int tmp_dstsize = 0;
	unsigned int align_size = 0;
	struct isp_trim_info *in_trim;
	struct camera_size *out_size;
	unsigned int d_max = CAMERA_SC_COEFF_DOWN_MAX;
	unsigned int u_max = CAMERA_SC_COEFF_UP_MAX;
	unsigned int f_max = CAMERA_PATH_DECI_FAC_MAX;

	if (path == NULL) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	in_trim = &path->trim0_info;
	out_size = &path->dst;
	if (in_trim->size_x > (out_size->w * d_max * (1 << f_max)) ||
	    in_trim->size_y > (out_size->h * d_max * (1 << f_max)) ||
	    in_trim->size_x * u_max < out_size->w ||
	    in_trim->size_y * u_max < out_size->h) {
		rtn = ISP_RTN_PATH_SC_ERR;
	} else {
		path->scaler_info.scaler_factor_in = in_trim->size_x;
		path->scaler_info.scaler_ver_factor_in = in_trim->size_y;
		if (in_trim->size_x > out_size->w * d_max) {
			tmp_dstsize =
				out_size->w * d_max;
			path->deci_info.deci_x =
				isp_get_path_deci_factor(in_trim->size_x,
							  tmp_dstsize);
			path->deci_info.deci_x_eb = 1;
			align_size = (1 << (path->deci_info.deci_x + 1)) *
				ISP_PIXEL_ALIGN_WIDTH;
			in_trim->size_x = (in_trim->size_x)
				& ~(align_size - 1);
			in_trim->start_x = (in_trim->start_x)
				& ~(align_size - 1);
			path->scaler_info.scaler_factor_in =
				in_trim->size_x >> (path->deci_info.deci_x + 1);
		} else {
			path->deci_info.deci_x = 0;
			path->deci_info.deci_x_eb = 0;
		}

		if (in_trim->size_y > out_size->h * d_max) {
			tmp_dstsize = out_size->h * d_max;
			path->deci_info.deci_y =
				isp_get_path_deci_factor(in_trim->size_y,
							  tmp_dstsize);
			path->deci_info.deci_y_eb = 1;
			align_size = (1 << (path->deci_info.deci_y + 1)) *
				ISP_PIXEL_ALIGN_HEIGHT;
			in_trim->size_y = (in_trim->size_y)
				& ~(align_size - 1);
			in_trim->start_y = (in_trim->start_y)
				& ~(align_size - 1);
			path->scaler_info.scaler_ver_factor_in =
				in_trim->size_y >> (path->deci_info.deci_y + 1);
		} else {
			path->deci_info.deci_y = 0;
			path->deci_info.deci_y_eb = 0;
		}

		path->scaler_info.scaler_ver_factor_out = path->dst.h;
		path->scaler_info.scaler_factor_out = path->dst.w;
	}

	return -rtn;
}

static int isp_set_sc_coeff(struct isp_module *module,
		enum isp_path_index path_index,
		struct isp_path_desc *path,
		struct isp_sc_coeff *coeff)
{
	unsigned int scale2yuv420 = 0;
	unsigned int *tmp_buf = NULL;
	unsigned int *h_coeff = NULL;
	unsigned int *v_coeff = NULL;
	unsigned int *v_chroma_coeff = NULL;
	unsigned char y_tap = 0;
	unsigned char uv_tap = 0;
	enum isp_id idx = 0;
	enum isp_scl_id scl_path_id;

	if (module == NULL || !path || !coeff) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}
	idx = module->idx;

	if (module->isp_scl_array[idx] == NULL) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	scl_path_id = isp_get_scl_index(path_index);

	if (path->output_format == DCAM_YUV420)
		scale2yuv420 = 1;

	tmp_buf = coeff->buf;
	if (tmp_buf == NULL)
		return -EFAULT; /*DCAM_RTN_PATH_NO_MEM;*/

	path->scaler_info.coeff_buf = tmp_buf;
	h_coeff = tmp_buf;
	v_coeff = tmp_buf + (ISP_SC_COEFF_COEF_SIZE / 4);
	v_chroma_coeff = v_coeff + (ISP_SC_COEFF_COEF_SIZE / 4);

	wait_for_completion_interruptible(
			&module->scale_coeff_mem_com);

	if (!(dcam_gen_scale_coeff((short)path->scaler_info.scaler_factor_in,
				(short)path->scaler_info.scaler_ver_factor_in,
				(short)path->dst.w,
				(short)path->dst.h,
				h_coeff,
				v_coeff,
				v_chroma_coeff,
				scale2yuv420,
				&y_tap,
				&uv_tap,
				tmp_buf + (ISP_SC_COEFF_COEF_SIZE * 3 / 4),
				ISP_SC_COEFF_TMP_SIZE))) {
		pr_err("failed to call dcam_gen_scale_coeff\n");
		complete(&module->scale_coeff_mem_com);
		return -DCAM_RTN_PATH_GEN_COEFF_ERR;
	}

	path->scaler_info.scaler_y_ver_tap = y_tap;
	path->scaler_info.scaler_uv_ver_tap = uv_tap;

	pr_debug("Scaler y_tap %d, uv_tap %d\n", y_tap, uv_tap);

	complete(&module->scale_coeff_mem_com);
	return ISP_RTN_SUCCESS;
}

int isp_path_scaler(struct isp_module *module,
	enum isp_path_index path_index,
	struct isp_path_desc *path,
	struct isp_sc_coeff *coeff)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	unsigned long cfg_reg = 0;
	enum isp_id idx = 0;

	if (module == NULL || !path || !coeff) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}
	idx = module->idx;

	if (module->isp_scl_array[idx] == NULL) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	if (path_index == ISP_PATH_IDX_0)
		cfg_reg = ISP_SCALER0_BASE + ISP_SCALER_CFG;
	else if (path_index == ISP_PATH_IDX_PRE)
		cfg_reg = ISP_SCALER_PRE_BASE + ISP_SCALER_CFG;
	else if (path_index == ISP_PATH_IDX_VID)
		cfg_reg = ISP_SCALER_VID_BASE + ISP_SCALER_CFG;
	else if (path_index == ISP_PATH_IDX_CAP)
		cfg_reg = ISP_SCALER_CAP_BASE + ISP_SCALER_CFG;
	else
		return -EFAULT;


	if (path->output_format == DCAM_RAWRGB ||
			path->output_format == DCAM_JPEG) {
		DCAM_TRACE("out format is %d, no need scaler\n",
				path->output_format);
		return ISP_RTN_SUCCESS;
	}

	rtn = isp_calc_sc_size(idx, path);
	if (rtn)
		return rtn;
	if (path->scaler_info.scaler_factor_in ==
			path->scaler_info.scaler_factor_out &&
			path->scaler_info.scaler_ver_factor_in ==
			path->scaler_info.scaler_ver_factor_out &&
			path->output_format == DCAM_YUV422 &&
			path_index != ISP_PATH_IDX_0) {
		path->scaler_bypass = 1;
		ISP_REG_MWR(idx, cfg_reg, BIT_20, 1 << 20);
	} else {
		path->scaler_bypass = 0;
		ISP_REG_MWR(idx, cfg_reg, BIT_20, 0 << 20);
		rtn = isp_set_sc_coeff(module, path_index,
				path, coeff);
	}

	return rtn;
}

static int isp_raw_cap_proc(struct isp_pipe_dev *dev,
	struct isp_raw_proc_info *raw_cap)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_module *module = NULL;
	struct isp_path_desc *cap = NULL;
	struct isp_store_cce_desc *store_cce = NULL;
	struct camera_addr frm_addr = {0};
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;

	if (!dev || !raw_cap) {
		ret = -EFAULT;
		pr_err("isp dev handle is NULL\n");
		goto exit;
	}

	module = &dev->module_info;
	cap = &module->isp_path[ISP_SCL_CAP];
	store_cce = &module->store_cce;
	fmcu_slice = &dev->fmcu_slice;

	dev->is_raw_capture = 1;
	fmcu_slice->capture_state = ISP_ST_START;

	cap->in_size.w = raw_cap->in_size.width;
	cap->in_size.h = raw_cap->in_size.height;
	cap->out_size.w = raw_cap->out_size.width;
	cap->out_size.h = raw_cap->out_size.height;
	cap->in_rect.x = 0;
	cap->in_rect.y = 0;
	cap->in_rect.w = raw_cap->in_size.width;
	cap->in_rect.h = raw_cap->in_size.height;
	cap->output_format = DCAM_YUV420;
	cap->valid = 1;
	store_cce->output_format = DCAM_YUV420;

	frm_addr.yaddr = raw_cap->img_offset.chn0;
	frm_addr.uaddr = raw_cap->img_offset.chn1;
	frm_addr.vaddr = raw_cap->img_offset.chn2;
	frm_addr.yaddr_vir = raw_cap->img_vir.chn0;
	frm_addr.uaddr_vir = raw_cap->img_vir.chn1;
	frm_addr.vaddr_vir = raw_cap->img_vir.chn2;
	frm_addr.mfd_y = raw_cap->img_fd;
	pr_info("start path config\n");
	ret = set_isp_path_cfg((void *)dev, ISP_PATH_IDX_CAP,
		ISP_PATH_OUTPUT_ADDR, &frm_addr);

	pr_info("start isp path\n");
	ret = sprd_isp_start((void *)dev);
	pr_info("end isp path\n");

exit:
	return ret;
}

static int isp_3dnr_process(struct isp_pipe_dev *dev,
			    struct isp_3dnr_param *param)
{
	int ret = 0;
	struct pfiommu_info *pfinfo = NULL;
	struct isp_3dnr_info isp_3dnr_info;
	unsigned long flag;
	enum isp_id idx = 0;

	idx = dev->module_info.idx;
	spin_lock_irqsave(&isp_mod_lock[idx], flag);
	dev->is_wait_fmcu = 1;
	spin_unlock_irqrestore(&isp_mod_lock[idx], flag);
	if (wait_for_completion_interruptible(&dev->fmcu_com) != 0) {
		pr_err("Fail to wait the fmcu_com for post 3dnr.\n");
		return -EINVAL;
	}

	ret = isp_fmcu_iommu_map(&dev->fmcu_slice, idx);
	if (ret) {
		pr_err("%s err, failed to map fmcu\n", __func__);
		return -EINVAL;
	}

	isp_3dnr_info.fmcu_cmd_vbuf =
		dev->fmcu_slice.iommu_addr_vir;
	isp_3dnr_info.fmcu_cmd_pbuf =
		dev->fmcu_slice.iommu_addr_phy;
	isp_3dnr_info.isp_reg_base = 0;

	pfinfo = &dev->pfinfo_3dnr[0];
	pfinfo->dev = &s_isp_pdev->dev;
	pfinfo->mfd[0] = param->fetch_cur_addr_fd;
	pfinfo->mfd[1] = param->fetch_cur_addr_fd;
	pfinfo->mfd[2] = param->fetch_cur_addr_fd;
	ret = pfiommu_get_sg_table(pfinfo);
	if (ret != 0) {
		pr_err("err to get sg table. ret=%d, fd=%d.\n",
		       ret, param->fetch_cur_addr_fd);
		return -EINVAL;
	}

	ret = pfiommu_get_addr(pfinfo);
	if (ret != 0) {
		pr_err("err to get pfiommu addr. ret=%d, fd=%d.\n",
		       ret, param->fetch_cur_addr_fd);
		return -EINVAL;
	}
	isp_3dnr_info.fetch_cur_addr[0] = pfinfo->iova[0];
	isp_3dnr_info.fetch_cur_addr[1] = pfinfo->iova[0] + param->image_width *
		param->image_height;

	pfinfo = &dev->pfinfo_3dnr[1];
	pfinfo->dev = &s_isp_pdev->dev;
	pfinfo->mfd[0] = param->fetch_ref_addr_fd;
	pfinfo->mfd[1] = param->fetch_ref_addr_fd;
	pfinfo->mfd[2] = param->fetch_ref_addr_fd;
	ret = pfiommu_get_sg_table(pfinfo);
	if (ret != 0) {
		pr_err("err to get sg table. ret=%d, fd=%d.\n",
		       ret, param->fetch_ref_addr_fd);
		return -EINVAL;
	}

	ret = pfiommu_get_addr(pfinfo);
	if (ret != 0) {
		pr_err("err to get pfiommu addr. ret=%d, fd=%d.\n",
		       ret, param->fetch_cur_addr_fd);
		return -EINVAL;
	}
	isp_3dnr_info.fetch_ref_addr[0] = pfinfo->iova[0];
	isp_3dnr_info.fetch_ref_addr[1] =
		pfinfo->iova[0] + param->image_width * param->image_height;

	pfinfo = &dev->pfinfo_3dnr[2];
	pfinfo->dev = &s_isp_pdev->dev;
	pfinfo->mfd[0] = param->store_ref_addr_fd;
	pfinfo->mfd[1] = param->store_ref_addr_fd;
	pfinfo->mfd[2] = param->store_ref_addr_fd;
	ret = pfiommu_get_sg_table(pfinfo);
	if (ret != 0) {
		pr_err("err to get sg table. ret=%d, fd=%d.\n",
		       ret, param->store_ref_addr_fd);
		return -EINVAL;
	}
	ret = pfiommu_get_addr(pfinfo);
	if (ret != 0) {
		pr_err("err to get pfiommu addr. ret=%d, fd=%d.\n",
		       ret, param->fetch_cur_addr_fd);
		return -EINVAL;
	}
	isp_3dnr_info.store_ref_addr[0] = pfinfo->iova[0];
	isp_3dnr_info.store_ref_addr[1] =
		pfinfo->iova[0] + param->image_width * param->image_height;
	isp_3dnr_info.mv_x = param->mv_x;
	isp_3dnr_info.mv_y = param->mv_y;
	isp_3dnr_info.fetch_cur_endian = param->fetch_cur_endian;
	isp_3dnr_info.fetch_ref_endian = param->fetch_ref_endian;
	isp_3dnr_info.store_ref_endian = param->store_ref_endian;
	isp_3dnr_info.image_width = param->image_width;
	isp_3dnr_info.image_height = param->image_height;
	isp_3dnr_info.blending_no = param->blending_no;
	if (idx == 0)
		ISP_REG_WR(0, ISP_FMCU_ISP_REG_REGION, 0x0D2F0D2E);
	else
		ISP_REG_WR(1, ISP_FMCU_ISP_REG_REGION, 0x0D300D2F);
	ret = isp_3dnr_process_one_frame(&isp_3dnr_info);
	if (ret != 0) {
		pr_err("fail to call 3dnr function,ret=%d, blending_no=%d.\n",
		       ret, isp_3dnr_info.blending_no);
		return -EINVAL;
	}

	return ret;
}

int sprd_isp_k_ioctl(void *isp_pipe_dev_handle,
	unsigned int cmd, unsigned long param)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct sprd_isp_irq irq_param;
	struct isp_node node = {0};
	struct isp_statis_buf_input parm_inptr;
	struct isp_k_block *isp_k_param = NULL;
	struct isp_raw_proc_info raw_cap;
	struct isp_3dnr_param isp_3dnr;

	if (!isp_pipe_dev_handle) {
		ret = -EFAULT;
		pr_err("isp dev handle is NULL\n");
		goto exit;
	}
	dev = (struct isp_pipe_dev *)isp_pipe_dev_handle;

	DCAM_TRACE("cmd: 0x%x, %d\n", cmd, _IOC_NR(cmd));
	isp_k_param = &dev->isp_k_param;
	if (!isp_k_param) {
		ret = -EFAULT;
		pr_err("isp_ioctl: isp_private is null error.\n");
		return ret;
	}
	switch (cmd) {
	case SPRD_ISP_IO_IRQ:
	{
		ret = wait_for_completion_interruptible(&dev->isr_done_lock);
		if (ret) {
			memset(&irq_param, 0, sizeof(irq_param));
			irq_param.ret_val = ret;
			ret = copy_to_user((void __user *)param,
				(void *)&irq_param, sizeof(irq_param));
			if (ret != 0)
				pr_err("isp_ioctl: irq: copy_to_user error ret = %d\n",
				(uint32_t)ret);
			ret = -ERESTARTSYS;
			return ret;
		}

		ret = isp_queue_read(&dev->queue, &node);
		if (ret != 0) {
			ret = -EFAULT;
			pr_err("isp_ioctl: isp_queue_read error, ret = 0x%x\n",
				(uint32_t)ret);
			return ret;
		}

		memset(&irq_param, 0, sizeof(irq_param));
		irq_param.irq_val0 = node.irq_val0;
		irq_param.irq_val1 = node.irq_val1;
		irq_param.irq_val2 = node.irq_val2;
		irq_param.irq_val3 = node.irq_val3;
		irq_param.reserved = node.reserved;
		irq_param.time = node.time;
		ret = copy_to_user((void __user *)param,
			(void *)&irq_param, sizeof(irq_param));
		if (ret != 0) {
			ret = -EFAULT;
			pr_err("isp_k: ioctl irq: copy_to_user error, ret = 0x%x",
				(uint32_t)ret);
		}
		break;
	}
	case SPRD_ISP_IO_READ:
	{
		break;
	}
	case SPRD_ISP_IO_RAW_CAP:
		mutex_lock(&dev->isp_mutex);
		pr_info("start isp_raw_cap_proc %p\n", dev);
		memset((void *)&raw_cap, 0x0, sizeof(struct isp_raw_proc_info));
		ret = copy_from_user(&raw_cap,
				     (void __user *)param,
				     sizeof(struct isp_raw_proc_info));
		if (ret != 0) {
			pr_err("isp_ioctl: copy_from_user error\n");
			mutex_unlock(&dev->isp_mutex);
			goto exit;
		}
		ret = isp_raw_cap_proc(dev, &raw_cap);
		mutex_unlock(&dev->isp_mutex);
		break;
	case SPRD_ISP_IO_WRITE:
	{
		break;
	}
	case SPRD_ISP_IO_RST:
	{
		isp_k_param->lsc_2d_weight_en = 0;
		isp_k_param->is_lsc_param_init_flag = 0;
		atomic_set(&isp_k_param->lsc_updated, 1);
		break;
	}
	case SPRD_ISP_IO_STOP:
	{
		pr_info("%s, SPRD_ISP_IO_STOP\n", __func__);
		isp_statis_reset_all_queue(dev);
		break;
	}
	case SPRD_ISP_IO_INT:
	{
		break;
	}
	case SPRD_ISP_IO_CFG_PARAM:
		ret = isp_cfg_param((void *)param, isp_k_param, dev->idx);
		break;
	case SPRD_ISP_IO_SET_STATIS_BUF:
		mutex_lock(&dev->isp_mutex);
		ret = copy_from_user(&parm_inptr,
				     (void __user *)param,
				     sizeof(struct isp_statis_buf_input));
		if (ret != 0) {
			pr_err("isp_ioctl: copy_from_user error\n");
			mutex_unlock(&dev->isp_mutex);
			goto exit;
		}
		if (parm_inptr.buf_flag == STATIS_BUF_FLAG_INIT)
			ret = sprd_isp_cfg_statis_buf(dev, &parm_inptr);
		else
			ret = sprd_isp_set_statis_addr(dev, &parm_inptr);

		mutex_unlock(&dev->isp_mutex);
		break;
	case SPRD_ISP_IO_CAPABILITY:
		ret = isp_capability((void *)param);
		break;
	case SPRD_ISP_REG_READ:
	{
		break;
	}
	case SPRD_ISP_IO_POST_3DNR:
	{
		mutex_lock(&dev->isp_mutex);
		pr_debug("start isp_3dnr_process.\n");
		memset((void *)&isp_3dnr, 0x0, sizeof(struct isp_3dnr_param));
		ret = copy_from_user(&isp_3dnr,
				     (void __user *)param,
				     sizeof(struct isp_3dnr_param));
		if (ret != 0) {
			pr_err("isp_ioctl: copy_from_user error\n");
			mutex_unlock(&dev->isp_mutex);
			goto exit;
		}
		ret = isp_3dnr_process(dev, &isp_3dnr);
		mutex_unlock(&dev->isp_mutex);
		break;
	}
	default:
		pr_err("isp_ioctl: cmd is unsupported, cmd = %x [%d]\n",
			(int32_t)cmd, _IOC_NR(cmd));
		return -EFAULT;
	}

exit:
	return ret;
}

int sprd_isp_update_path(void *isp_handle,
			enum isp_path_index path_index,
			struct camera_size *in_size,
			struct camera_rect *in_rect,
			struct camera_size *out_size)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_path_desc *scl0 = NULL;
	struct isp_path_desc *pre = NULL;
	struct isp_path_desc *vid = NULL;
	struct isp_path_desc *cap = NULL;
	struct isp_sc_array *scl_array = NULL;
	struct isp_store_cce_desc *store_cce = NULL;
	struct isp_sc_coeff *scl0_coeff = NULL;
	struct isp_sc_coeff *pre_coeff = NULL;
	struct isp_sc_coeff *vid_coeff = NULL;
	struct isp_sc_coeff *cap_coeff = NULL;
	struct isp_sc_coeff *store_cce_coeff = NULL;

	enum isp_id idx = 0;

	if (isp_handle == NULL) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	idx = module->idx;
	scl_array = module->isp_scl_array[idx];

	if (scl_array == NULL) {
		pr_err("zero pointer\n");
		return -EFAULT;
	}
	pr_debug("ISP%d: update path\n", idx);

	scl0 = &module->isp_path[ISP_SCL_0];
	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];
	cap = &module->isp_path[ISP_SCL_CAP];
	store_cce = &module->store_cce;
	rtn = isp_coeff_get_new_node(&scl_array->scl0_queue, &scl0_coeff, 0);
	if (rtn) {
		pr_err("isp coeff get new node fail\n");
		return -rtn;
	}
	rtn = isp_coeff_get_new_node(&scl_array->vid_queue, &vid_coeff, 0);
	if (rtn) {
		pr_err("isp coeff get new node fail\n");
		return -rtn;
	}
	rtn = isp_coeff_get_new_node(&scl_array->pre_queue, &pre_coeff, 0);

	rtn = isp_coeff_get_new_node(&scl_array->cap_queue, &cap_coeff, 0);
	if (rtn) {
		pr_err("isp coeff get new node fail\n");
		return -rtn;
	}
	rtn = isp_coeff_get_new_node(&scl_array->store_cce_queue,
				     &store_cce_coeff, 0);
	if (rtn) {
		pr_err("isp coeff get new node fail\n");
		return -rtn;
	}
	if (ISP_PATH_IDX_PRE & path_index && pre->valid) {
		memcpy(&scl0_coeff->path, scl0,
		       sizeof(struct isp_path_desc));
		memcpy(&pre_coeff->path, pre,
		       sizeof(struct isp_path_desc));
		pre_coeff->path.in_size.w = in_size->w;
		pre_coeff->path.in_size.h = in_size->h;
		pre_coeff->path.in_rect.x = in_rect->x;
		pre_coeff->path.in_rect.y = in_rect->y;
		pre_coeff->path.in_rect.w = in_rect->w;
		pre_coeff->path.in_rect.h = in_rect->h;
		pre_coeff->path.out_size.w = out_size->w;
		pre_coeff->path.out_size.h = out_size->h;
#ifndef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (out_size->w/in_rect->w >= ISP_ONLINE_ZFACTOR_MAX)
			pre->is_clk_update = 1;
#endif

		if (!vid->valid && !cap->valid) {
			rtn = isp_start_pre_proc(module, &scl0_coeff->path,
				&pre_coeff->path,
				&vid_coeff->path,
				&cap_coeff->path,
				&store_cce_coeff->store_cce);
			if (rtn) {
				pr_err("isp start pre proc failed\n");
				return -rtn;
			}

			rtn = isp_path_scaler(module, ISP_PATH_IDX_0,
					      &scl0_coeff->path,
					      scl0_coeff);
			if (rtn) {
				pr_err("%s err, code %d", __func__, rtn);
				return -(rtn);
			}
			pr_debug("ISP%d: to update path1\n", idx);
			rtn = isp_path_scaler(module, ISP_PATH_IDX_PRE,
					      &pre_coeff->path,
					      pre_coeff);
			if (rtn) {
				pr_err("%s err, code %d", __func__, rtn);
				return -(rtn);
			}
			rtn = isp_coeff_get_new_node(&scl_array->scl0_queue,
						     &scl0_coeff, 1);
			if (rtn) {
				pr_err("isp coeff get new node fail\n");
				return -rtn;
			}
			rtn = isp_coeff_get_new_node(&scl_array->pre_queue,
						     &pre_coeff, 1);
			if (rtn) {
				pr_err("isp coeff get new node fail\n");
				return -rtn;
			}
		}
	}

	if (ISP_PATH_IDX_VID & path_index && vid->valid) {
		memcpy(&vid_coeff->path, vid,
				sizeof(struct isp_path_desc));
		vid_coeff->path.in_size.w = in_size->w;
		vid_coeff->path.in_size.h = in_size->h;
		vid_coeff->path.in_rect.x = in_rect->x;
		vid_coeff->path.in_rect.y = in_rect->y;
		vid_coeff->path.in_rect.w = in_rect->w;
		vid_coeff->path.in_rect.h = in_rect->h;
		vid_coeff->path.out_size.w = out_size->w;
		vid_coeff->path.out_size.h = out_size->h;
#ifndef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (out_size->w/in_rect->w >= ISP_ONLINE_ZFACTOR_MAX)
			vid->is_clk_update = 1;
#endif
		if (!cap->valid) {
			pr_err("update_path - pre in %d %d rect %d %d %d %d out %d %d\n",
				pre_coeff->path.in_size.w,
				pre_coeff->path.in_size.h,
				pre_coeff->path.in_rect.x,
				pre_coeff->path.in_rect.y,
				pre_coeff->path.in_rect.w,
				pre_coeff->path.in_rect.h,
				pre_coeff->path.out_size.w,
				pre_coeff->path.out_size.h);
			rtn = isp_start_pre_proc(module,
					&scl0_coeff->path,
					&pre_coeff->path,
					&vid_coeff->path,
					&cap_coeff->path,
					&store_cce_coeff->store_cce);
			if (rtn) {
				pr_err("isp start pre proc failed\n");
				return -rtn;
			}
#ifndef CONFIG_ISP_CPP_COWORK_SUPPORT
			pr_debug("ISP%d: to update path vid\n", idx);
			rtn = isp_path_scaler(module, ISP_PATH_IDX_VID,
					&vid_coeff->path,
					vid_coeff);
			if (rtn) {
				pr_err("%s err, code %d", __func__,
						rtn);
				return -(rtn);
			}
			rtn = isp_coeff_get_new_node(
					&scl_array->vid_queue,
					&vid_coeff, 1);
			if (rtn) {
				pr_err("isp coeff get new node fail\n");
				return -rtn;
			}
#endif
			if (pre->valid) {
				rtn = isp_path_scaler(module,
						ISP_PATH_IDX_0,
						&scl0_coeff->path,
						scl0_coeff);
				if (rtn) {
					pr_err("err, code %d", rtn);
					return -(rtn);
				}
				pr_debug("ISP%d: to update path1\n",
						idx);
				rtn = isp_path_scaler(module,
						ISP_PATH_IDX_PRE,
						&pre_coeff->path,
						pre_coeff);
				if (rtn) {
					pr_err("pre path scaler err\n");
					return -(rtn);
				}
				rtn = isp_coeff_get_new_node(&scl_array
						->scl0_queue,
						&scl0_coeff, 1);

				if (rtn) {
					pr_err("isp coeff get new node fail\n");
					return -rtn;
				}
				rtn = isp_coeff_get_new_node(
						&scl_array->pre_queue,
						&pre_coeff, 1);
				if (rtn) {
					pr_err("isp coeff get new node fail\n");
					return -rtn;
				}
			}
		}

	}

	if (ISP_PATH_IDX_CAP & path_index && cap->valid) {
		memcpy(&cap_coeff->path, cap,
				sizeof(struct isp_path_desc));
		memcpy(&store_cce_coeff->store_cce, store_cce,
				sizeof(struct isp_store_cce_desc));
		cap_coeff->path.in_size.w = in_size->w;
		cap_coeff->path.in_size.h = in_size->h;
		cap_coeff->path.in_rect.x = in_rect->x;
		cap_coeff->path.in_rect.y = in_rect->y;
		cap_coeff->path.in_rect.w = in_rect->w;
		cap_coeff->path.in_rect.h = in_rect->h;
		cap_coeff->path.out_size.w = out_size->w;
		cap_coeff->path.out_size.h = out_size->h;
		pr_debug("pre in %d %d rect %d %d %d %d out %d %d\n",
				pre_coeff->path.in_size.w,
				pre_coeff->path.in_size.h,
				pre_coeff->path.in_rect.x,
				pre_coeff->path.in_rect.y,
				pre_coeff->path.in_rect.w,
				pre_coeff->path.in_rect.h,
				pre_coeff->path.out_size.w,
				pre_coeff->path.out_size.h);

		rtn = isp_start_pre_proc(module, &scl0_coeff->path,
				&pre_coeff->path,
				&vid_coeff->path,
				&cap_coeff->path,
				&store_cce_coeff->store_cce);
		if (rtn) {
			pr_err("isp start pre proc failed\n");
			return -rtn;
		}

		pr_debug("ISP%d: to update path1\n", idx);
		rtn = isp_path_scaler(module, ISP_PATH_IDX_CAP,
				&cap_coeff->path, cap_coeff);
		if (rtn) {
			pr_err("%s err, code %d", __func__, rtn);
			return -(rtn);
		}

		rtn = isp_coeff_get_new_node(&scl_array->store_cce_queue,
				&store_cce_coeff, 1);
		if (rtn) {
			pr_err("isp coeff get new node fail\n");
			return -rtn;
		}

		if (vid->valid) {
			pr_debug("ISP%d: to update path vid\n", idx);
			rtn = isp_path_scaler(module, ISP_PATH_IDX_VID,
					&vid_coeff->path, vid_coeff);
			if (rtn) {
				pr_err("%s err, code %d", __func__, rtn);
				return -(rtn);
			}
			rtn = isp_coeff_get_new_node(&scl_array->vid_queue,
					&vid_coeff, 1);
			if (rtn) {
				pr_err("isp coeff get new node fail\n");
				return -rtn;
			}
		}

		if (pre->valid) {
			pr_debug("ISP%d: to update path1\n", idx);
			rtn = isp_path_scaler(module, ISP_PATH_IDX_0,
					&scl0_coeff->path, scl0_coeff);
			if (rtn) {
				pr_err("%s err, code %d", __func__, rtn);
				return -(rtn);
			}
			rtn = isp_path_scaler(module, ISP_PATH_IDX_PRE,
					&pre_coeff->path, pre_coeff);
			if (rtn) {
				pr_err("%s err, code %d", __func__, rtn);
				return -(rtn);
			}
			rtn = isp_coeff_get_new_node(&scl_array->scl0_queue,
					&scl0_coeff, 1);
			if (rtn) {
				pr_err("isp coeff get new node fail\n");
				return -rtn;
			}
			rtn = isp_coeff_get_new_node(&scl_array->pre_queue,
					&pre_coeff, 1);
			if (rtn) {
				pr_err("isp coeff get new node fail\n");
				return -rtn;
			}
		}
	}

	pr_debug("update path done\n");

	return -rtn;
}

static void isp_test_set_reg(enum isp_id idx)
{
	/* ISP_YDELAY_STEP */
	if (idx == ISP_ID_1) {
		ISP_REG_WR(idx, ISP_YDELAY_STEP, 0xC1);
		ISP_REG_WR(idx, ISP_COMMON_LBUF_OFFSET0, 0x140);
		ISP_REG_WR(idx, ISP_COMMON_LBUF_OFFSET1, (0x51 << 16));
	} else {
		ISP_REG_WR(idx, ISP_YDELAY_STEP, 0x241);
		ISP_REG_WR(idx, ISP_YDELAY_STEP + 0x40000, 0x241);
	}

	pr_info("Start isp block bypass reg config\n");
#if 1 /* Must be bypass block */
	ISP_REG_MWR(idx, ISP_POST_BLC_PARA, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_NLC_PARA, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_RLSC_CTRL, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_PSTRZ_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HUE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YGAMMA_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_SCL_CFG, BIT_9, 1 << 9);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST_CFG_READY, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST2_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST2_CFG_RDY, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_PDAF_AF_CTRL1, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_STORE_BASE+ISP_STORE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_PRE_BLEND_CONTRL0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_CAP_BLEND_CONTRL0, BIT_0, 1);
#endif
	pr_debug("Start set isp ch1 reg config\n");
	ISP_REG_MWR(idx, ISP_CH1_ADDR_OFFSET + ISP_HUE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_CH1_ADDR_OFFSET + ISP_YGAMMA_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_CH1_ADDR_OFFSET + ISP_YRANDOM_PARAM0, BIT_0, 1);
}

static int isp_set_statis_buf(struct isp_pipe_dev *dev)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_statis_module *statis_module = NULL;

	statis_module = &dev->statis_module_info;

	rtn = isp_set_next_statis_buf(dev->idx, statis_module,
				      ISP_AEM_BLOCK);
	if (rtn) {
		pr_err("%s err, code %d", __func__, rtn);
		return -(rtn);
	}
	rtn = isp_set_next_statis_buf(dev->idx, statis_module,
				      ISP_AFL_BLOCK);
	if (rtn) {
		pr_err("%s err, code %d", __func__, rtn);
		return -(rtn);
	}
	rtn = isp_set_next_statis_buf(dev->idx, statis_module,
				      ISP_AFM_BLOCK);
	if (rtn) {
		pr_err("%s err, code %d", __func__, rtn);
		return -(rtn);
	}
	rtn = isp_set_next_statis_buf(dev->idx, statis_module,
				      ISP_BINNING_BLOCK);
	if (rtn) {
		pr_err("%s err, code %d", __func__, rtn);
		return -(rtn);
	}

	rtn = isp_set_next_statis_buf(dev->idx, statis_module,
				      ISP_PDAF_BLOCK);
	if (rtn) {
		pr_err("%s err, code %d", __func__, rtn);
		return -(rtn);
	}

	return rtn;
}

int sprd_isp_start_path(void *isp_handle, enum isp_path_index path_index)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_path_desc *scl0 = NULL;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct isp_path_desc *pre = NULL;
	struct isp_path_desc *vid = NULL;
	struct isp_path_desc *cap = NULL;
	struct isp_store_cce_desc *store_cce = NULL;
	struct isp_sc_array *scl_array = NULL;
	unsigned int val = 0;
	enum isp_id idx = 0;
	unsigned long flag;


	if (!isp_handle) {
		pr_err("Input ptr is NULL\n");
		return -ISP_RTN_PARA_ERR;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	module->idx = dev->idx;
	scl0 = &module->isp_path[ISP_SCL_0];
	fmcu_slice = &dev->fmcu_slice;
	idx = module->idx;
	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];
	cap = &module->isp_path[ISP_SCL_CAP];
	store_cce = &module->store_cce;
	scl_array = module->isp_scl_array[idx];
	pr_info("ISP%d: start path 0x%x,  path {%d %d %d %d}\n",
		module->idx, path_index, module->isp_path[ISP_SCL_0].valid,
		module->isp_path[ISP_SCL_PRE].valid,
		module->isp_path[ISP_SCL_VID].valid,
		module->isp_path[ISP_SCL_CAP].valid);

	if (dev->is_raw_capture == 1)
		store_cce->is_raw_cap = 1;

	rtn = isp_start_pre_proc(module, scl0, pre, vid, cap, store_cce);
	if (rtn) {
		pr_err("isp start pre proc failed\n");
		return -rtn;
	}
	/*set statis buf before stream on*/
	rtn = isp_set_statis_buf(dev);
	if (rtn) {
		pr_err("isp start set statis buf failed\n");
		return -rtn;
	}

	rtn = isp_fmcu_iommu_map(fmcu_slice, idx);
	if (rtn) {
		pr_err("%s err, failed to map fmcu\n", __func__);
		return -(rtn);
	}

	if (scl0->valid) {
		pr_info("start scl0 scaler, scl0 dst w:%d, h:%d\n",
				scl0->dst.w, scl0->dst.h);
		rtn = isp_path_scaler(module, ISP_PATH_IDX_0, scl0,
				&scl_array->coeff[ISP_SCL_0]);
		if (rtn) {
			pr_err("%s err, code %d", __func__, rtn);
			return -(rtn);
		}
		isp_path_set_scl(module->idx, scl0, ISP_SCALER0_BASE);
		val = ((scl0->dst.h & 0xFFFF) << 16) | (scl0->dst.w & 0xFFFF);
		ISP_REG_WR(module->idx, ISP_DISPATCH_YUV_CH0_SIZE, val);
		ISP_REG_MWR(module->idx, ISP_COMMON_YUV_PATH_SEL_CH0,
				(BIT_0 | BIT_1), 1);
	}
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	if (module->isp_cpp_dev.is_valid) {
		pr_info("module->isp_cpp_dev.is_valid %d\n",
				module->isp_cpp_dev.is_valid);
		isp_cpp_init(isp_handle);
	}
#endif

	if (module->store_cce.valid) {
		rtn = isp_storecce_get_buf(&module->store_cce, module->idx);
		if (rtn) {
			pr_err("%s err, failed to get storecce buf", __func__);
			return -(rtn);
		}
		rtn = isp_storecce_buf_iommu_map(&module->store_cce,
				module->idx);
		if (rtn) {
			pr_err("%s err, failed to get storecce buf", __func__);
			return -(rtn);
		}

		isp_set_storecce_info(module->idx, &module->store_cce,
				ISP_STOREA_BASE);
		rtn = isp_storecce_set_next_frm(module);
		if (rtn) {
			pr_err("%s err, code %d", __func__, rtn);
			return -(rtn);
		}
		module->store_cce.status = ISP_ST_START;
		fmcu_slice->storecce_state = ISP_ST_START;
		module->store_cce.shadow_done_cnt = 0;
		module->store_cce.frm_cnt = 0;
	}

	if ((ISP_PATH_IDX_PRE & path_index) &&
			module->isp_path[ISP_SCL_PRE].valid) {
		rtn = isp_path_scaler(module, ISP_PATH_IDX_PRE, pre,
				&scl_array->coeff[ISP_SCL_PRE]);
		if (rtn) {
			pr_err("%s err, code %d", __func__, rtn);
			return -(rtn);
		}
		isp_path_set(module, pre, ISP_PATH_IDX_PRE);
		module->isp_path[ISP_SCL_PRE].frm_cnt = 0;
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (pre->path_mode == ISP_PRE_ONLINE_CPP ||
				pre->path_mode == ISP_PRE_VID_ONLINE_CPP) {
			rtn = isp_cpp_set_next_frame_for_path(module,
					ISP_SCL_PRE);
			if (rtn) {
				pr_err("%s cpp err, code %d",
						__func__, rtn);
				return -(rtn);
			}
		} else
#endif
		{
			rtn = isp_path_set_next_frm(module,
				    ISP_PATH_IDX_PRE, NULL);
			if (rtn) {
				pr_err("%s err, code %d",
					__func__, rtn);
				return -(rtn);
			}
		}
		ISP_REG_MWR(module->idx,
				ISP_COMMON_SCL_PATH_SEL, (BIT_0 | BIT_1),
				module->isp_path[ISP_SCL_PRE].path_sel);


		if (pre->path_mode == ISP_PRE_OFFLINE) {
			spin_lock_irqsave(&isp_mod_lock[idx], flag);
			fmcu_slice->capture_state = ISP_ST_START;
			spin_unlock_irqrestore(&isp_mod_lock[idx], flag);
		}
		module->isp_path[ISP_SCL_PRE].status = ISP_ST_START;
		module->isp_path[ISP_SCL_PRE].need_wait = 0;
		module->isp_path[ISP_SCL_PRE].shadow_done_cnt = 0;
#ifndef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (pre->out_size.w/pre->in_rect.w >= ISP_ONLINE_ZFACTOR_MAX &&
				!cap->valid)
			sprd_isp_clock_update(dev, ISP_CLK_384M_INDEX);
#endif
	}

	if ((ISP_PATH_IDX_VID & path_index) &&
			module->isp_path[ISP_SCL_VID].valid
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
			&&
			 (vid->path_mode != ISP_PRE_VID_ONLINE_CPP)
#endif
			) {
		pr_err("gaurav_cpp configure video path\n");
		rtn = isp_path_scaler(module, ISP_PATH_IDX_VID, vid,
				&scl_array->coeff[ISP_SCL_VID]);
		if (rtn) {
			pr_err("%s err, code %d", __func__, rtn);
			return -(rtn);
		}
		isp_path_set(module, vid, ISP_PATH_IDX_VID);

		module->isp_path[ISP_SCL_VID].frm_cnt = 0;
		if (dev->fmcu_slw.slw_flags == ISP_SLW_VIDEO) {
			rtn = set_fmcu_slw_cfg(isp_handle);
			if (rtn) {
				pr_err("%s err, code %d", __func__, rtn);
				return -(rtn);
			}
			rtn = isp_fmcu_slw_start(ISP_SCL_VID, isp_handle);
			if (rtn) {
				pr_err("%s err, code %d", __func__, rtn);
				return -(rtn);
			}
		} else {

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
			if (vid->path_mode == ISP_VID_ONLINE_CPP) {
				rtn = isp_cpp_set_next_frame_for_path(module,
					    ISP_SCL_VID);
				if (rtn) {
					pr_err("%s cpp err, code %d",
						__func__, rtn);
					return -(rtn);
				}
			} else
#endif
			{
				rtn = isp_path_set_next_frm(module,
					ISP_PATH_IDX_VID, NULL);
				if (rtn) {
					pr_err("%s err, code %d",
						__func__, rtn);
					return -(rtn);
				}
			}
		}
		ISP_REG_MWR(module->idx,
				ISP_COMMON_SCL_PATH_SEL, (BIT_2 | BIT_3),
				module->isp_path[ISP_SCL_VID].path_sel << 2);

		if (vid->path_mode == ISP_VID_OFFLINE) {
			spin_lock_irqsave(&isp_mod_lock[idx], flag);
			fmcu_slice->capture_state = ISP_ST_START;
			spin_unlock_irqrestore(&isp_mod_lock[idx], flag);
		}
		module->isp_path[ISP_SCL_VID].status = ISP_ST_START;
		module->isp_path[ISP_SCL_VID].need_wait = 0;
		module->isp_path[ISP_SCL_VID].shadow_done_cnt = 0;
#ifndef CONFIG_ISP_CPP_COWORK_SUPPORT
		if (vid->out_size.w/vid->in_rect.w >= ISP_ONLINE_ZFACTOR_MAX)
			sprd_isp_clock_update(dev, ISP_CLK_384M_INDEX);
#endif
	}

	if ((ISP_PATH_IDX_CAP & path_index) &&
	     module->isp_path[ISP_SCL_CAP].valid) {
		rtn = isp_path_scaler(module, ISP_PATH_IDX_CAP, cap,
				&scl_array->coeff[ISP_SCL_CAP]);
		if (rtn) {
			pr_err("%s err, code %d", __func__, rtn);
			return -(rtn);
		}
		isp_path_set(module, cap, ISP_PATH_IDX_CAP);
		module->isp_path[ISP_SCL_CAP].frm_cnt = 0;
		ISP_REG_MWR(module->idx,
				ISP_COMMON_SCL_PATH_SEL, (BIT_4 | BIT_5),
				module->isp_path[ISP_SCL_CAP].path_sel << 4);

		module->isp_path[ISP_SCL_CAP].status = ISP_ST_START;
		module->isp_path[ISP_SCL_CAP].need_wait = 0;
		module->isp_path[ISP_SCL_CAP].shadow_done_cnt = 0;
	}
#ifdef ISP_TEST
	pr_debug("Start set test isp reg info\n");
	isp_test_set_reg(module->idx);
	pr_debug("End set test isp reg info\n");
#endif
	if (path_index != ISP_PATH_IDX_ALL) {
		if ((ISP_PATH_IDX_PRE & path_index))
			isp_wait_path_done(module, ISP_SCL_PRE, NULL);
		else if ((ISP_PATH_IDX_VID & path_index))
			isp_wait_path_done(module, ISP_SCL_VID, NULL);
		else if ((ISP_PATH_IDX_CAP & path_index))
			isp_wait_path_done(module, ISP_SCL_CAP, NULL);
	}
	return rtn;
}

int sprd_isp_start(void *isp_handle)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;

	if (!isp_handle) {
		pr_err("Input ptr is NULL\n");
		return -ISP_RTN_PARA_ERR;
	}

	rtn = sprd_isp_start_path(isp_handle, ISP_PATH_IDX_ALL);

	/* to debug memory covering.  */
	if (isp_handle) {
#define IOMMU_REG_OFFSET  0x820
		struct isp_pipe_dev *dev = NULL;
		struct device *p_dev = NULL;
		unsigned int idx,  val0;

		dev = (struct isp_pipe_dev *)isp_handle;
		idx = dev->idx;

		if (idx == ISP_ID_0)
			p_dev = &s_isp_pdev->dev;
		else
			p_dev = &s_isp1_pdev->dev;
		if (sprd_iommu_attach_device(p_dev) == 0)
			sprd_iommu_open(p_dev);


		val0 = ISP_REG_RD(idx, IOMMU_REG_OFFSET);
		pr_info("ISP%d:  iommu ctrl: 0x%x\n", idx, val0);
	}
	return rtn;
}

int set_isp_path_cfg(void *isp_handle, enum isp_path_index path_index,
	enum isp_cfg_id id, void *param)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_path_desc *path = NULL;
	struct isp_path_desc *scl0 = NULL;
	struct isp_store_cce_desc *store_cce = NULL;
	struct isp_regular_info *regular_info = NULL;
	struct isp_endian_sel *endian = NULL;
	enum isp_scl_id path_id;
	struct isp_module *module = NULL;
	struct camera_size *size = NULL;
	struct camera_rect *rect = NULL;
	struct camera_addr *p_addr;
	struct isp_pipe_dev *dev = NULL;
	unsigned int frm_deci = 0;
	unsigned int format = 0;
	unsigned int path_mode = 0;
	unsigned int zoom_mode = 0;
	enum isp_id idx  = 0;
	unsigned int  isp_path_frame_w_max = ISP_PATH_FRAME_WIDTH_MAX;
	unsigned int  isp_path_frame_h_max = ISP_PATH_FRAME_HEIGHT_MAX;
	unsigned int skip_num = 0;
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	struct isp_cpp_desc *isp_cpp_dev;
#endif
	if (!isp_handle || !param) {
		pr_err("parm is null\n");
		return -ISP_RTN_PARA_ERR;
	}

	if (idx == ISP_ID_1) {
		isp_path_frame_w_max = ISP1_PATH_FRAME_WIDTH_MAX;
		isp_path_frame_h_max = ISP1_PATH_FRAME_HEIGHT_MAX;

	} else{
		isp_path_frame_w_max = ISP_PATH_FRAME_WIDTH_MAX;
		isp_path_frame_h_max = ISP_PATH_FRAME_HEIGHT_MAX;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	idx = dev->idx;
	if (ISP_PATH_IDX_PRE & path_index) {
		path_id = ISP_SCL_PRE;
	} else if (ISP_PATH_IDX_VID & path_index) {
		path_id = ISP_SCL_VID;
	} else if (ISP_PATH_IDX_CAP & path_index) {
		path_id = ISP_SCL_CAP;
	} else {
		pr_err("%s error", __func__);
		return -ISP_RTN_PARA_ERR;
	}

	module = &dev->module_info;
	path = &module->isp_path[path_id];
	scl0 = &module->isp_path[ISP_SCL_0];
	store_cce = &module->store_cce;

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	isp_cpp_dev = &module->isp_cpp_dev;
#endif
	switch (id) {
	case ISP_PATH_INPUT_SIZE:
		size = (struct camera_size *)param;

		pr_debug("ISP path%d input size:{%d %d}\n", path_id,
			size->w, size->h);

		if (size->w > isp_path_frame_w_max ||
			size->h > isp_path_frame_h_max) {
			rtn = ISP_RTN_PATH_IN_SIZE_ERR;
		} else {
			path->in_size.w = size->w;
			path->in_size.h = size->h;
		}
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
		isp_cpp_dev->y_buf_len = size->w * size->h;
		isp_cpp_dev->uv_buf_len = isp_cpp_dev->y_buf_len / 2;
#endif
		break;

	case ISP_PATH_INPUT_RECT:
		rect = (struct camera_rect *)param;

		pr_debug("ISP path%d trim size:{%d %d} {%d %d}\n", path_id,
			rect->x, rect->y, rect->w, rect->h);

		if (rect->x > isp_path_frame_w_max ||
		    rect->y > isp_path_frame_h_max ||
		    rect->w > isp_path_frame_w_max ||
		    rect->h > isp_path_frame_h_max) {
			rtn = ISP_RTN_PATH_TRIM_SIZE_ERR;
		} else {
			path->in_rect.x = rect->x;
			path->in_rect.y = rect->y;
			path->in_rect.w = rect->w;
			path->in_rect.h = rect->h;
		}


		break;

	case ISP_PATH_OUTPUT_SIZE:
		size = (struct camera_size *)param;

		pr_debug("ISP path%d output size:{%d %d}\n", path_id,
			size->w, size->h);

		if (size->w > isp_path_frame_w_max ||
		    size->h > isp_path_frame_h_max) {
			rtn = ISP_RTN_PATH_OUT_SIZE_ERR;
		} else {
			path->out_size.w = size->w;
			path->out_size.h = size->h;
		}
		break;

	case ISP_PATH_OUTPUT_FORMAT:
		format = *(unsigned int *)param;
		if ((format == DCAM_YUV422) || (format == DCAM_YUV420) ||
		    (format == DCAM_YVU420) || (format == DCAM_YUV420_3FRAME)) {
			path->output_format = format;
			store_cce->output_format = format;
		} else {
			rtn = ISP_RTN_OUT_FMT_ERR;
			path->output_format = DCAM_FTM_MAX;
		}
		scl0->output_format = DCAM_YUV422;
		break;

	case ISP_PATH_OUTPUT_ADDR:
		p_addr = (struct camera_addr *)param;

		if (DCAM_YUV_ADDR_INVALID(p_addr->yaddr, p_addr->uaddr,
					  p_addr->vaddr) &&
		    p_addr->mfd_y == 0) {
			rtn = ISP_RTN_PATH_ADDR_ERR;
		} else {
			struct camera_frame frame;

			memset((void *)&frame, 0,
			       sizeof(struct camera_frame));
			frame.yaddr = p_addr->yaddr;
			frame.uaddr = p_addr->uaddr;
			frame.vaddr = p_addr->vaddr;
			frame.yaddr_vir = p_addr->yaddr_vir;
			frame.uaddr_vir = p_addr->uaddr_vir;
			frame.vaddr_vir = p_addr->vaddr_vir;

			frame.type = path_id;
			frame.fid = path->frame_base_id;

			if (idx == ISP_ID_0)
				frame.pfinfo.dev = &s_isp_pdev->dev;
			else
				frame.pfinfo.dev = &s_isp1_pdev->dev;
			frame.pfinfo.mfd[0] = p_addr->mfd_y;
			frame.pfinfo.mfd[1] = p_addr->mfd_u;
			frame.pfinfo.mfd[2] = p_addr->mfd_u;
			/*may need update iommu here*/
			rtn = pfiommu_get_sg_table(&frame.pfinfo);
			if (rtn) {
				pr_err("cfg output addr failed!\n");
				rtn = DCAM_RTN_PATH_ADDR_ERR;
				break;
			}

			frame.pfinfo.offset[0] = 0;
			frame.pfinfo.offset[1] = 0;
			frame.pfinfo.offset[2] = 0;
			if (!isp_buf_queue_write(&path->buf_queue,
						 &frame))
				path->output_frame_count++;
		}
		break;

	case ISP_PATH_STORE_CCE_ADDR:
		p_addr = (struct camera_addr *)param;

		if (DCAM_YUV_ADDR_INVALID(p_addr->yaddr, p_addr->uaddr,
					  p_addr->vaddr) &&
		    p_addr->mfd_y == 0) {
			rtn = ISP_RTN_PATH_ADDR_ERR;
		} else {
			struct camera_frame frame;

			memset((void *)&frame, 0,
			       sizeof(struct camera_frame));
			frame.yaddr = p_addr->yaddr;
			frame.uaddr = p_addr->uaddr;
			frame.vaddr = p_addr->vaddr;
			frame.yaddr_vir = p_addr->yaddr_vir;
			frame.uaddr_vir = p_addr->uaddr_vir;
			frame.vaddr_vir = p_addr->vaddr_vir;

			frame.type = path_id;
			frame.fid = path->frame_base_id;

			if (idx == ISP_ID_0)
				frame.pfinfo.dev = &s_isp_pdev->dev;
			else
				frame.pfinfo.dev = &s_isp1_pdev->dev;
			frame.pfinfo.mfd[0] = p_addr->mfd_y;
			frame.pfinfo.mfd[1] = p_addr->mfd_u;
			frame.pfinfo.mfd[2] = p_addr->mfd_u;
			/*may need update iommu here*/
			rtn = pfiommu_get_sg_table(&frame.pfinfo);
			if (rtn) {
				pr_err("cfg store cce addr failed!\n");
				rtn = DCAM_RTN_PATH_ADDR_ERR;
				break;
			}

			frame.pfinfo.offset[0] = 0;
			frame.pfinfo.offset[1] = 0;
			frame.pfinfo.offset[2] = 0;

			if (!isp_buf_queue_write(&store_cce->tmp_buf_queue,
						 &frame))
				store_cce->output_frame_count++;

			pr_info("y=0x%x u=0x%x v=0x%x mfd=0x%x 0x%x\n",
				 p_addr->yaddr, p_addr->uaddr,
				 p_addr->vaddr, frame.pfinfo.mfd[0],
				 frame.pfinfo.mfd[1]);
		}
		break;

	case ISP_PATH_OUTPUT_RESERVED_ADDR:
		p_addr = (struct camera_addr *)param;

		if (DCAM_YUV_ADDR_INVALID(p_addr->yaddr,
					  p_addr->uaddr,
					  p_addr->vaddr) &&
		    p_addr->mfd_y == 0) {
			rtn = ISP_RTN_PATH_ADDR_ERR;
		} else {
			unsigned int output_frame_count = 0;
			struct camera_frame *frame = NULL;

			frame = &module->path_reserved_frame[path_id];
			output_frame_count = path->output_frame_count;
			frame->yaddr = p_addr->yaddr;
			frame->uaddr = p_addr->uaddr;
			frame->vaddr = p_addr->vaddr;
			frame->yaddr_vir = p_addr->yaddr_vir;
			frame->uaddr_vir = p_addr->uaddr_vir;
			frame->vaddr_vir = p_addr->vaddr_vir;

			if (idx == ISP_ID_0)
				frame->pfinfo.dev = &s_isp_pdev->dev;
			else
				frame->pfinfo.dev = &s_isp1_pdev->dev;
			frame->pfinfo.mfd[0] = p_addr->mfd_y;
			frame->pfinfo.mfd[1] = p_addr->mfd_u;
			frame->pfinfo.mfd[2] = p_addr->mfd_u;
			/*may need update iommu here*/
			rtn = pfiommu_get_sg_table(&frame->pfinfo);
			if (rtn) {
				pr_err("ISP%d:reserved out addr fail!\n", idx);
				rtn = ISP_RTN_PATH_ADDR_ERR;
				break;
			}
			if (pfiommu_get_addr(&frame->pfinfo)) {
				pr_err("ISP%d:reserve buffer addr fail\n", idx);
				rtn = ISP_RTN_PATH_ADDR_ERR;
				break;
			}

			frame->pfinfo.offset[0] = 0;
			frame->pfinfo.offset[1] = 0;
			frame->pfinfo.offset[2] = 0;
		}
		break;

	case ISP_PATH_FRM_DECI:
		frm_deci = *(unsigned int *)param;
		if (frm_deci >= DCAM_FRM_DECI_FAC_MAX)
			rtn = ISP_RTN_FRM_DECI_ERR;
		else
			path->frm_deci = frm_deci;
		pr_err("ISP path%d frm deci is %d\n", path_id, path->frm_deci);
		break;

	case ISP_PATH_MODE:
		path_mode = *(unsigned int *)param;
		path->path_mode = path_mode;
		pr_err("ISP path%d mode is %d\n", path_id, path->path_mode);
		break;

	case ISP_PATH_ZOOM_MODE:
		zoom_mode = *(unsigned int *)param;
		module->isp_scl_array[idx]->is_smooth_zoom = zoom_mode;
		break;
	case ISP_PATH_SN_MAX_SIZE:
		size = (struct camera_size *)param;
		store_cce->y_buf_len = size->w * size->h;
		store_cce->uv_buf_len = store_cce->y_buf_len / 2;
		break;

	case ISP_PATH_ENABLE:
		path->valid = *(unsigned int *)param;
		pr_info("ISP path%d enable is %d\n", path_id, path->valid);
		break;

	case ISP_PATH_SHRINK:
		regular_info = (struct isp_regular_info *)param;
		memcpy(&path->regular_info, regular_info,
		       sizeof(struct isp_regular_info));
		break;
	case ISP_PATH_DATA_ENDIAN:
		endian = (struct isp_endian_sel *)param;
		if (endian->y_endian >= DCAM_ENDIAN_MAX ||
		    endian->uv_endian >= DCAM_ENDIAN_MAX) {
			rtn = ISP_RTN_PATH_ENDIAN_ERR;
		} else {
			path->data_endian.y_endian = endian->y_endian;
			store_cce->data_endian.y_endian = endian->y_endian;
			if ((ISP_PATH_IDX_PRE |
			     ISP_PATH_IDX_CAP |
			     ISP_PATH_IDX_VID) & path_index) {
				path->data_endian.uv_endian = endian->uv_endian;
				store_cce->data_endian.uv_endian =
					endian->uv_endian;
				pr_debug("isp path data endian %d\n",
					path->data_endian.uv_endian);
			}
		}
		break;
	case ISP_PATH_SKIP_NUM:
		skip_num = *(unsigned int *)param;
		if (skip_num >= DCAM_CAP_SKIP_FRM_MAX)
			rtn = ISP_RTN_FRM_DECI_ERR;
		else
			path->skip_num = skip_num;
		break;

	default:
		pr_err("%s error", __func__);
		break;
	}

	return -rtn;
}

static int isp_enable_clk(enum isp_id idx)
{

	int ret = 0;

	pr_err("ppp:[%s] enter, idx=%d\n", __func__, idx);

	/*set isp clock to max value*/
	ret = clk_set_parent(isp_clk[idx], isp_clk_parent[idx]);
	if (ret) {
		pr_info("isp_clk_parent[%d] error.\n", idx);
		clk_set_parent(isp_clk[idx], isp_clk_default[idx]);
		clk_disable_unprepare(isp_clk[idx]);
		goto exit;
	}

	ret = clk_prepare_enable(isp_clk[idx]);
	if (ret) {
		pr_info("isp0_clk error.\n");
		clk_set_parent(isp_clk[idx], isp_clk_default[idx]);
		clk_disable_unprepare(isp_clk[idx]);
		goto exit;
	}

	ret = clk_prepare_enable(isp_eb);
	if (ret) {
		pr_info("isp0_eb error.\n");
		goto exit;
	}

	ret = clk_prepare_enable(isp_mtx_eb);
	if (ret) {
		pr_info("isp_mtx_eb error.\n");
		goto exit;
	}

	ret = clk_prepare_enable(cam_mtx_eb);
	if (ret) {
		pr_info("cam_mtx_eb error.\n");
		goto exit;
	}

	ret = clk_prepare_enable(isp_clk_eb[idx]);
	if (ret) {
		pr_info("isp_clk_eb error.\n");
		goto exit;
	}

	ret = clk_prepare_enable(isp_iab_i_eb[idx]);
	if (ret) {
		pr_info("isp_iab_i_eb error.\n");
		goto exit;
	}

	ret = clk_prepare_enable(isp_axi_eb[idx]);
	if (ret) {
		pr_info("isp0_axi_eb error.\n");
		goto exit;
	}

	pr_info("isp_eb_clk end. idx:%d\n", idx);
exit:
	return ret;
}

static int isp_disable_clk(enum isp_id idx)
{
	pr_info("[%s] enter, idx:%d\n", __func__, idx);
	/*cut off the isp colck source*/
	clk_disable_unprepare(isp_mtx_eb);
	clk_disable_unprepare(cam_mtx_eb);
	clk_disable_unprepare(isp_eb);
	clk_disable_unprepare(isp_clk_eb[idx]);
	clk_disable_unprepare(isp_axi_eb[idx]);
	clk_disable_unprepare(isp_iab_i_eb[idx]);

	/*set isp clock to default value before power off*/
	clk_set_parent(isp_clk[idx], isp_clk_default[idx]);
	clk_disable_unprepare(isp_clk[idx]);

	return 0;
}

/*register the callback func from the dcam_core*/
int sprd_isp_reg_isr(enum isp_id idx, enum isp_irq_id irq_id,
	isp_isr_func user_func, void *user_data)
{
	int ret = 0;

	pr_debug("%s:irq_id %d\n", __func__, irq_id);
	if (irq_id >= ISP_IMG_MAX) {
		pr_err("isp IRQ is error.\n");
		ret = ISP_RTN_IRQ_NUM_ERR;
	} else {
		pr_debug("%s:irq user_data %p\n", __func__, user_data);
		ret = isp_irq_callback(idx, irq_id, user_func, user_data);
		if (ret)
			pr_err("Register isp callback error\n");
	}

	return ret;
}

static int isp_module_init(struct isp_module *module_info, enum isp_id idx)
{
	int ret = 0;
	int i = 0;

	if (!module_info) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

	for (i = ISP_SCL_PRE; i <= ISP_SCL_CAP; i++) {
		isp_buf_queue_init(&module_info->isp_path[i].buf_queue);
		init_completion(&module_info->isp_path[i].sof_com);
	}

	isp_buf_queue_init(&module_info->store_cce.tmp_buf_queue);
	isp_frm_queue_clear(&module_info->store_cce.frame_queue);
	isp_frm_queue_clear(&module_info->store_cce.zsl_queue);
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	module_info->isp_cpp_dev.is_valid = 0;
#endif
	/*TODO: isp_scl_array[2] changed to isp_scl_array*/
	module_info->isp_scl_array[0] = NULL;
	module_info->isp_scl_array[1] = NULL;
	module_info->isp_scl_array[idx] = (struct isp_sc_array *)
					vzalloc(sizeof(struct isp_sc_array));
	if (module_info->isp_scl_array[idx] == NULL) {
		pr_err("failed to alloc isp_scl_array, %d\n", idx);
		ret = -1;
	}

	isp_coeff_queue_init(&module_info->isp_scl_array[idx]->scl0_queue);
	isp_coeff_queue_init(&module_info->isp_scl_array[idx]->pre_queue);
	isp_coeff_queue_init(&module_info->isp_scl_array[idx]->vid_queue);
	isp_coeff_queue_init(&module_info->isp_scl_array[idx]->cap_queue);
	isp_coeff_queue_init(&module_info->isp_scl_array[idx]->store_cce_queue);
	init_completion(&module_info->scale_coeff_mem_com);
	complete(&module_info->scale_coeff_mem_com);
	return ret;
}

static int isp_module_deinit(struct isp_module *module_info, enum isp_id idx)
{
	int ret = 0;
	int i = 0;

	if (!module_info) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

	for (i = ISP_SCL_PRE; i <= ISP_SCL_CAP; i++)
		isp_buf_queue_init(&module_info->isp_path[i].buf_queue);

	isp_storecce_clear_buf(&module_info->store_cce);
	isp_buf_queue_init(&module_info->store_cce.tmp_buf_queue);
	isp_frm_queue_clear(&module_info->store_cce.frame_queue);
	isp_frm_queue_clear(&module_info->store_cce.zsl_queue);
	for (i = 0; i < s_isp_count; i++) {
		if (module_info->isp_scl_array[i]) {
			vfree(module_info->isp_scl_array[i]);
			module_info->isp_scl_array[i] = NULL;
		}
	}

	return ret;
}

int sprd_isp_module_en(void *isp_handle, enum isp_id idx)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;

	if (!isp_handle) {
		pr_err("Input dev is NULL\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	dev->idx = idx;

	pr_info("ISP%d: enable isp module in: %d\n", idx,
		atomic_read(&s_isp_users[idx]));

	mutex_lock(&isp_module_sema[idx]);
	if (atomic_inc_return(&s_isp_users[idx]) == 1) {

		wake_lock(&isp_wakelock[idx]);

		/*sprd_isp_pw_on */
		ret = sprd_cam_pw_on();
		if (ret != 0) {
			pr_err("%s : sprd cam_sys power on failed, ret: %d\n",
			       __func__, ret);
			mutex_unlock(&isp_module_sema[idx]);
			return ret;
		}

		ret = sprd_isp_pw_on();
		if (ret != 0) {
			pr_err("%s : sprd isp_top power on failed, ret: %d\n",
			       __func__, ret);
			mutex_unlock(&isp_module_sema[idx]);
			return ret;
		}

		sprd_cam_domain_eb();

		ret = isp_enable_clk(idx);
		if (ret)
			pr_err("failed to enable clk %d\n", ret);

		ret = sprd_isp_reset(idx);
		if (ret)
			pr_err("failed to reset isp %d\n", ret);

		ret = isp_irq_request(&s_isp_pdev->dev, &s_isp_irq[idx], dev);
		if (ret)
			pr_err("failed to install isp IRQ %d\n", ret);
	}

	mutex_unlock(&isp_module_sema[idx]);

	pr_info("ISP%d: enable isp module end: %d\n", idx,
		atomic_read(&s_isp_users[idx]));

	return ret;
}

int sprd_isp_module_dis(void *isp_handle, enum isp_id idx)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;

	if (!isp_handle) {
		pr_err("Input dev is NULL\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;

	pr_info("ISP%d: disable isp module, in %d\n", idx,
		   atomic_read(&s_isp_users[idx]));

	if (atomic_read(&s_isp_users[idx]) == 0)
		return rtn;

	mutex_lock(&isp_module_sema[idx]);
	if (atomic_dec_return(&s_isp_users[idx]) == 0) {
		isp_disable_clk(idx);
		sprd_cam_domain_disable();

		rtn = sprd_isp_pw_off();
		if (rtn != 0) {
			pr_err("%s : sprd isp_top power off failed, ret: %d\n",
					__func__, rtn);
			isp_irq_free(&s_isp_irq[idx], dev);
			wake_unlock(&isp_wakelock[idx]);
			mutex_unlock(&isp_module_sema[idx]);
			return rtn;
		}
		rtn = sprd_cam_pw_off();
		if (rtn != 0) {
			pr_err("%s : sprd cam_sys power off failed, ret: %d\n",
					__func__, rtn);
			isp_irq_free(&s_isp_irq[idx], dev);
			wake_unlock(&isp_wakelock[idx]);
			mutex_unlock(&isp_module_sema[idx]);
			return rtn;
		}
		rtn = isp_irq_free(&s_isp_irq[idx], dev);
		if (rtn)
			pr_err("failed to free isp IRQ %d\n", rtn);

		wake_unlock(&isp_wakelock[idx]);

	}

	mutex_unlock(&isp_module_sema[idx]);

	pr_info("ISP%d: disable isp module end. %d\n", idx,
		   atomic_read(&s_isp_users[idx]));

	return rtn;
}

static int32_t isp_block_buf_alloc(struct isp_k_block *isp_k_param)
{
	int32_t ret = 0;
	uint32_t buf_len = 0;

	buf_len = ISP_FRGB_GAMMA_BUF_SIZE;

	isp_k_param->full_gamma_buf_addr = (unsigned long)vzalloc(buf_len);
	if (isp_k_param->full_gamma_buf_addr == 0) {
		ret = -1;
		pr_err("isp_block_buf_alloc: no memory error.\n");
	}

	return ret;
}

static int32_t isp_block_buf_free(struct isp_k_block *isp_k_param)
{

	if (isp_k_param->full_gamma_buf_addr != 0x00) {
		vfree((void *)isp_k_param->full_gamma_buf_addr);
		isp_k_param->full_gamma_buf_addr = 0x00;
	}

	return 0;
}

int sprd_isp_dev_init(void **isp_pipe_dev_handle, enum isp_id idx)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_k_block *isp_k_param = NULL;

	if (!isp_pipe_dev_handle) {
		pr_err("%s input handle is NULL %d\n", __func__, idx);
		return -EINVAL;
	}

	dev = vzalloc(sizeof(*dev));

	dev->idx = idx;
	dev->is_raw_capture = 0;
	mutex_init(&dev->isp_mutex);
	init_completion(&dev->irq_com);
	init_completion(&dev->isr_done_lock);
	isp_k_param = &dev->isp_k_param;
	spin_lock_init(&isp_k_param->lsc_lock);
	ret = isp_block_buf_alloc(isp_k_param);
	if (ret != 0)
		isp_block_buf_free(isp_k_param);

	isp_irq_done_init(idx);
	ret = isp_queue_init(&dev->queue);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to init queue\n", __func__);
		ret = -EIO;
		goto queue_exit;
	}
	ret = isp_statis_queue_init(&dev->statis_module_info.aem_statis_queue);
	ret = isp_statis_queue_init(&dev->statis_module_info.afl_statis_queue);
	ret = isp_statis_queue_init(&dev->statis_module_info.afm_statis_queue);
	ret = isp_statis_queue_init(
		&dev->statis_module_info.binning_statis_queue);
	ret = isp_statis_queue_init(&dev->statis_module_info.pdaf_statis_queue);
	ret = isp_module_init(&dev->module_info, idx);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to init isp module info\n", __func__);
		ret = -EIO;
		goto queue_exit;
	}

	ret = isp_fmcu_slice_init(&dev->fmcu_slice.slice_handle);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to init fmcu slice\n", __func__);
		ret = -EIO;
		goto queue_exit;
	}

	ret = isp_fmcu_slw_init(&dev->fmcu_slw.slw_handle);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to init fmcu slw\n", __func__);
		ret = -EIO;
		goto slice_cleanup;
	}

	ret = isp_fmcu_get_buf(&dev->fmcu_slice, idx);
	if (ret) {
		pr_err("%s Failed to get fmcu buf\n", __func__);
		ret = -EIO;
		goto slw_cleanup;
	}

	ret = isp_create_offline_thread(dev);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to create offline thread\n", __func__);
		isp_stop_offline_thread(dev);
		ret = -EINVAL;
		goto slw_cleanup;
	}

	pr_info("sprd_isp: dev init end!\n");
	*isp_pipe_dev_handle = (void *)dev;
	goto exit;

slw_cleanup:
	isp_fmcu_slw_deinit(dev->fmcu_slw.slw_handle);
	dev->fmcu_slw.slw_handle = NULL;

slice_cleanup:
	isp_fmcu_slice_deinit(dev->fmcu_slice.slice_handle);
	dev->fmcu_slice.slice_handle = NULL;

queue_exit:
	vfree(dev);
	pr_err("%s: dev[%d] init failed!\n", __func__, idx);
exit:
	return ret;
}

int sprd_isp_dev_deinit(void *isp_pipe_dev_handle, enum isp_id idx)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_k_block *isp_k_param = NULL;

	if (!isp_pipe_dev_handle) {
		pr_err("%s input handle is NULL %d\n", __func__, idx);
		return -EINVAL;
	}

	dev = (struct isp_pipe_dev *)isp_pipe_dev_handle;
	isp_k_param = &dev->isp_k_param;
	mutex_lock(&dev->isp_mutex);
	ret = isp_queue_init(&dev->queue);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to init queue\n", __func__);
		mutex_unlock(&dev->isp_mutex);
		vfree(dev);
		return -EIO;
	}

	ret = isp_fmcu_clear_buf(&dev->fmcu_slice, idx);
	if (unlikely(ret != 0)) {
		pr_err("Failed to clear fmcu buf\n");
		mutex_unlock(&dev->isp_mutex);
		vfree(dev);
		return -EIO;
	}

	ret = isp_module_deinit(&dev->module_info, idx);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to init queue\n", __func__);
		mutex_unlock(&dev->isp_mutex);
		vfree(dev);
		return -EIO;
	}

	ret = isp_fmcu_slice_deinit(dev->fmcu_slice.slice_handle);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to deinit fmcu slice\n", __func__);
		mutex_unlock(&dev->isp_mutex);
		vfree(dev);
		return -EIO;
	}

	ret = isp_fmcu_slw_deinit(dev->fmcu_slw.slw_handle);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to deinit fmcu slw\n", __func__);
		mutex_unlock(&dev->isp_mutex);
		vfree(dev);
		return -EIO;
	}
	ret = isp_stop_offline_thread(dev);
	if (unlikely(ret != 0)) {
		pr_err("%s: Failed to stop offline thread\n", __func__);
		mutex_unlock(&dev->isp_mutex);
		vfree(dev);
		return -EIO;
	}
	init_completion(&dev->irq_com);
	init_completion(&dev->isr_done_lock);

	mutex_unlock(&dev->isp_mutex);

	isp_block_buf_free(isp_k_param);
	vfree(dev);

	pr_info("%s: dev[%d] deinit OK!\n", __func__, idx);

	return ret;
}

int sprd_isp_drv_init(struct platform_device *pdev)
{
	int ret = 0;
	int i;

	for (i = 0; i < s_isp_count; i++) {
		atomic_set(&s_isp_users[i], 0);
		mutex_init(&isp_module_sema[i]);
		isp_mod_lock[i] = __SPIN_LOCK_UNLOCKED(&isp_mod_lock[i]);
		isp_glb_reg_axi_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_axi_lock[i]);
		isp_glb_reg_mask_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_mask_lock[i]);
		isp_glb_reg_clr_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_clr_lock[i]);

		sprintf(isp_wakelock_name[i], "sprd_cam_wakelock_isp%d", i);

		wake_lock_init(&isp_wakelock[i], WAKE_LOCK_SUSPEND,
		   isp_wakelock_name[i]);

	}

	return ret;
}

void sprd_isp_drv_deinit(void)
{
	int i;

	for (i = 0; i < s_isp_count; i++) {
		atomic_set(&s_isp_users[i], 0);
		s_isp_irq[i].ch0 = 0;
		s_isp_irq[i].ch1 = 0;
		mutex_init(&isp_module_sema[i]);
		isp_mod_lock[i] = __SPIN_LOCK_UNLOCKED(&isp_mod_lock[i]);
		isp_glb_reg_axi_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_axi_lock[i]);
		isp_glb_reg_mask_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_mask_lock[i]);
		isp_glb_reg_clr_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_clr_lock[i]);

		wake_lock_destroy(&isp_wakelock[i]);
	}
}

int sprd_isp_clock_update(struct isp_pipe_dev *dev, int clk_index)
{

	int ret = 0;

	if (isp_clk_par_tab[clk_index] == NULL) {
		pr_err("isp update par clk is NULL %d\n",
				clk_index);
		return -1;
	}
	ret = clk_set_parent(isp_clk[0], isp_clk_par_tab[clk_index]);
	dev->clk_update_count++;
	pr_debug("isp clock update index %d, update cnt %d\n",
			clk_index, dev->clk_update_count);
	return ret;
}
static void sprd_isp_update_clk_par_tab(int index,
		struct device_node *dn)
{
	char *clk_name = NULL;
	unsigned int clk_isp_if = 0;


	clk_isp_if = isp_clk_tab[index].clock;
	clk_name = isp_clk_tab[index].clk_name;
	isp_clk_par_tab[index] = of_clk_get_by_name(dn, clk_name);
}

int sprd_isp_update_clk(int clk_index, struct device_node *dn)
{
	int ret = 0;
	char *clk_name = NULL;
	unsigned int clk_isp_if = 0;
	struct clk *clk_isppll = NULL;
	struct device_node *isp_node = NULL;

	if (!dn) {
		pr_err("Input ptr is NULL!\n");
		return -EINVAL;
	}

	clk_isp_if = isp_clk_tab[clk_index].clock;
	clk_name = isp_clk_tab[clk_index].clk_name;

	pr_info("isp_clk_name is %s, clk_isp %d\n", clk_name, clk_isp_if);
	if (clk_name != NULL) {
		isp_node = of_parse_phandle(dn, "sprd,isp", 0);
		if (isp_node == NULL) {
			pr_err("failed to parse the property of sprd,isp\n");
			return -EFAULT;
		}
		isp_clk_parent[0] = of_clk_get_by_name(isp_node, clk_name);

		if (IS_ERR(isp_clk_parent[0])) {
			pr_err("failed to get isp0_clk_parent\n");
			return PTR_ERR(isp_clk_parent[0]);
		}
	} else {
		pr_err("clk_name is NULL & clk_isp_if %d\n", clk_isp_if);
	}

	ret = clk_set_parent(isp_clk[0], isp_clk_parent[0]);

	if (clk_index == ISP_CLK_576M_INDEX) {
		clk_isppll = clk_get_parent(isp_clk_parent[0]);
		clk_set_rate(clk_isppll,  CLK_ISPPLL);
		pr_info("set isp0 clk to 468MHz");
	}

	if (ret) {
		pr_info("isp0_clk_parent error.\n");
		clk_set_parent(isp_clk[0], isp_clk_default[0]);
		clk_disable_unprepare(isp_clk[0]);
	}

	return ret;
}
int sprd_isp_parse_dt(struct device_node *dn, unsigned int *isp_count)
{
	int i = 0;
	unsigned int count = 0;
	void __iomem *reg_base;
	struct device_node *isp_node = NULL;
	struct resource res = {0};

	pr_info("isp dev device node %s, full name %s\n",
		dn->name, dn->full_name);
	isp_node = of_parse_phandle(dn, "sprd,isp", 0);
	if (isp_node == NULL) {
		pr_err("failed to parse the property of sprd,isp\n");
		return -EFAULT;
	}
	pr_info("after isp dev device node %s, full name %s\n",
		isp_node->name, isp_node->full_name);
	s_isp_pdev = of_find_device_by_node(isp_node);
	pr_info("sprd s_isp_pdev name %s\n", s_isp_pdev->name);
	if (of_device_is_compatible(isp_node, "sprd,isp")) {
		if (of_property_read_u32_index(isp_node,
			"sprd,isp-count", 0, &count)) {
			pr_err("failed to parse the property of sprd,isp-count\n");
			return -EINVAL;
		}
		s_isp_count = count;
		*isp_count = count;
		isp_eb = of_clk_get_by_name(isp_node, "isp0_eb");
		if (IS_ERR(isp_eb)) {
			pr_err("failed to get isp0_eb\n");
			return PTR_ERR(isp_eb);
		}

		isp_mtx_eb = of_clk_get_by_name(isp_node, "isp_mtx_eb");
		if (IS_ERR(isp_mtx_eb)) {
			pr_err("failed to get isp_mtx_eb\n");
			return PTR_ERR(isp_mtx_eb);
		}

		cam_mtx_eb = of_clk_get_by_name(isp_node, "cam_mtx_eb");
		if (IS_ERR(cam_mtx_eb)) {
			pr_err("failed to get cam_mtx_eb\n");
			return PTR_ERR(cam_mtx_eb);
		}

		isp_clk_eb[0] = of_clk_get_by_name(isp_node, "isp0_iclk_eb");
		if (IS_ERR(isp_clk_eb[0])) {
			pr_err("failed to get isp0_iclk_eb\n");
			return PTR_ERR(isp_clk_eb[0]);
		}

		isp_iab_i_eb[0] = of_clk_get_by_name(isp_node, "isp0_ia_i_eb");
		if (IS_ERR(isp_iab_i_eb[0])) {
			pr_err("failed to get isp0_ia_i_eb\n");
			return PTR_ERR(isp_iab_i_eb[0]);
		}

		isp_axi_eb[0] = of_clk_get_by_name(isp_node, "isp0_axi_eb");
		if (IS_ERR(isp_axi_eb[0])) {
			pr_err("failed to get isp0_axi_eb\n");
			return PTR_ERR(isp_axi_eb[0]);
		}

		isp_clk[0] = of_clk_get_by_name(isp_node, "isp0_clk");
		if (IS_ERR(isp_clk[0])) {
			pr_err("failed to get isp0_clk\n");
			return PTR_ERR(isp_clk[0]);
		}

		isp_clk_parent[0] = of_clk_get_by_name(
				isp_node, "isp0_clk_parent");
		if (IS_ERR(isp_clk_parent[0])) {
			pr_err("failed to get isp0_clk_parent\n");
			return PTR_ERR(isp_clk_parent[0]);
		}

		isp_clk_default[0] = clk_get_parent(isp_clk[0]);
		if (IS_ERR(isp_clk_default[0])) {
			pr_err("failed to get isp_clk_default[0]\n");
			return PTR_ERR(isp_clk_default[0]);
		}

		cam_ahb_gpr = syscon_regmap_lookup_by_phandle(isp_node,
			"sprd,cam-ahb-syscon");
		if (IS_ERR(cam_ahb_gpr))
			return PTR_ERR(cam_ahb_gpr);

		if (count == ISP_MAX_COUNT) {
			isp_clk_eb[1] =
				of_clk_get_by_name(isp_node, "isp1_mclk_eb");
			if (IS_ERR(isp_clk_eb[1])) {
				pr_err("failed to get isp1_mclk_eb\n");
				return PTR_ERR(isp_clk_eb[1]);
			}

			isp_iab_i_eb[1] =
				of_clk_get_by_name(isp_node, "isp1_ib_i_eb");
			if (IS_ERR(isp_iab_i_eb[1])) {
				pr_err("failed to get isp1_ib_i_eb\n");
				return PTR_ERR(isp_iab_i_eb[1]);
			}

			isp_axi_eb[1] =
				of_clk_get_by_name(isp_node, "isp1_axi_eb");
			if (IS_ERR(isp_axi_eb[1])) {
				pr_err("failed to get isp1_axi_eb\n");
				return PTR_ERR(isp_axi_eb[1]);
			}

			isp_clk[1] = of_clk_get_by_name(isp_node, "isp1_clk");
			if (IS_ERR(isp_clk[1])) {
				pr_err("failed to get isp1_clk\n");
				return PTR_ERR(isp_clk[1]);
			}

			isp_clk_parent[1] = of_clk_get_by_name(
					isp_node, "isp1_clk_parent");
			if (IS_ERR(isp_clk_parent[1])) {
				pr_err("failed to get isp1_clk_parent\n");
				return PTR_ERR(isp_clk_parent[1]);
			}

			isp_clk_default[1] = clk_get_parent(isp_clk[1]);
			if (IS_ERR(isp_clk_default[1])) {
				pr_err("failed to get isp_clk_default[1]\n");
				return PTR_ERR(isp_clk_default[1]);
			}
		}

		for (i = 0; i < count; i++) {
			if (of_address_to_resource(isp_node, i, &res))
				pr_err("failed to get isp phys addr\n");

			isp_phys_base[i] = (unsigned long)res.start;
			pr_info("isp phys reg base is %lx\n", isp_phys_base[i]);
			reg_base = of_iomap(isp_node, i);
			if (!reg_base) {
				pr_err("failed to get isp reg_base %d\n", i);
				return -ENXIO;
			}

			s_isp_regbase[i] = (unsigned long)reg_base;
			s_isp_chnid[i] = 0;
			s_isp_irq[i].ch0 = irq_of_parse_and_map(isp_node,
				i * 2);
			s_isp_irq[i].ch1 = irq_of_parse_and_map(isp_node,
				i * 2 + 1);
			if (s_isp_irq[i].ch0 <= 0 || s_isp_irq[i].ch1 <= 0) {
				pr_err("failed to get isp irq %d\n", i);
				return -EFAULT;
			}

		pr_info("ISP dts OK! base %lx, irq0 %d, irq1 %d\n",
				s_isp_regbase[i], s_isp_irq[i].ch0,
				s_isp_irq[i].ch1);
		}

		if (count == ISP_MAX_COUNT) {
			struct device_node *isp1_node = NULL;

			isp1_node = of_parse_phandle(isp_node, "sprd,isp1", 0);
			if (isp1_node == NULL) {
				pr_err("failed to parse the property of sprd,isp1\n");
				s_isp1_pdev = NULL;
			} else {
				pr_info("parse isp1 dev device node %s, full name %s\n",
						isp1_node->name,
						isp1_node->full_name);
				s_isp1_pdev = of_find_device_by_node(isp1_node);
				pr_info("sprd s_isp1_pdev name %s\n",
						s_isp1_pdev->name);
			}
		}
		sprd_isp_update_clk_par_tab(ISP_CLK_384M_INDEX, isp_node);
	} else {
		pr_err("failed to match isp device node\n");
		return -EINVAL;
	}

	return 0;
}
