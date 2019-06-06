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
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/pagemap.h>
#include <linux/sprd_iommu.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <video/sprd_mm.h>
#include <video/sprd_img.h>
#include "isp_buf.h"
#include "isp_statis_buf.h"
#include "isp_drv.h"
#include "isp_int.h"
#include "isp_path.h"
#include "isp_slice.h"
#include "gen_scale_coef.h"
#include "isp_slw.h"
#include "cam_pw_domain.h"
#include "isp_block.h"
#include "dcam_core.h"
#include "ion.h"
#include "ion_priv.h"

#define ISP_OFF_PRODUCER_Q_SIZE_MAX	2
#define ISP_OFF_PRODUCER_Q_SIZE_MIN	1
#define ISP_OFF_CONSUMER_Q_SIZE		2
#define ISP_OFF_CONSUMER_Q_SIZE_MULTI	4
#define ISP_HDR_FRM_NUM		3
#define ISP_3DNR_NUM            5
#define ISP_AXI_STOP_TIMEOUT	1000
#define ISP_SCALER_CFG_MASK	0x21800000
#define LSC_BUF_NAME		"2D_LSC"
#define WAIT_CNT_MS		50
#define FMCU_STOP_WAIT_CNT	(WAIT_CNT_MS * 2)

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_DRV (%d): %s %d: " \
	fmt, current->pid, __func__, __LINE__

#define ISP_OTHER_ID(iid)	(iid == ISP_ID_0 ? ISP_ID_1 : ISP_ID_0)

struct platform_device *s_isp_pdev;
struct isp_pipe_dev *g_isp_dev_parray[ISP_MAX_COUNT];
static atomic_t s_isp_users[ISP_MAX_COUNT];
static unsigned int s_isp_count;
unsigned long s_isp_regbase[ISP_MAX_COUNT];
unsigned long isp_phys_base[ISP_MAX_COUNT];
static struct clk *isp0_clk;
static struct clk *isp0_clk_parent;
static struct clk *isp0_clk_default;
static struct clk *isp0_axi_eb;
static struct clk *isp0_eb;
struct isp_ch_irq s_isp_irq[ISP_MAX_COUNT];
static struct mutex isp_module_sema[ISP_MAX_COUNT];
static struct regmap *cam_ahb_gpr;

static spinlock_t isp_glb_reg_axi_lock[ISP_MAX_COUNT];
static spinlock_t isp_glb_reg_mask_lock[ISP_MAX_COUNT];
static spinlock_t isp_glb_reg_clr_lock[ISP_MAX_COUNT];
spinlock_t isp_mod_lock;
static atomic_t isp_enable_lock;
/* Dynamic switch isp clk: flag D0-D15 for iid 0 */
static DEFINE_SPINLOCK(isp_clk_pr);
static int isp_clk_flag;
struct isp_clk_gate isp_clk_gt;
unsigned int is_dual_cam;
unsigned int fmcu_slice_capture_state;
unsigned int isp_frm_queue_len;
static int sprd_isp_pipeline_proc_bin(void *handle);
static int sprd_isp_pipeline_proc_full(void *handle);

/*======================ISP debug control=====================*/
/*
 * This function will actually do the bypass or work on
 * isp sub block with the help of
 * sys/sprd_image/isp_sblk/<sblk_file>.
 *
 * TODO: maybe need to mask this dbg function for user version,
 * using macro like: CONFIG_CAM_DEBUG.
 */
void isp_dbg_bypass_sblk(struct isp_pipe_dev *dev, unsigned int idx)
{
	struct isp_dbg_info *dbg = NULL;
	unsigned int i = 0;
	unsigned int cur_map, ori_map = 0;
	unsigned int map_id, sblk_id = 0;
	unsigned int sblk_cnt, addr = 0;
	struct isp_sub_blk_base *sblk_base = NULL;
	unsigned int bypass;

	pr_debug("enter, cb:%pS\n", __builtin_return_address(0));
	if (!dev || !dev->cam_grp) {
		pr_err("fail to exit1 exception\n");
		return;
	}

	dbg = &dev->cam_grp->dbg_info.isp_dbg;
	if (!dbg || !dbg->dbg_on) {
		pr_debug("exit2\n");
		return;
	}

	sblk_cnt = dbg->sblk_cnt;
	sblk_base = dbg->sblk_base;

	for (map_id = 0; map_id < ISP_SBLK_MAP_CNT; map_id++) {
		cur_map = dbg->sblk_maps[map_id];
		ori_map = dev->sblk_ori_byp_map[map_id];

		pr_debug("map %d: cur_map: 0x%x, ori_map: 0x%x\n",
			 map_id, cur_map, ori_map);
		for (i = 0; i < ISP_SBLK_MAP_SIZE; i++) {
			sblk_id = map_id * ISP_SBLK_MAP_SIZE + i;

			/* to the end of sblk_base table, return */
			if (unlikely(sblk_id == sblk_cnt)) {
				pr_debug("through all the %d sub-blocks, return\n",
					 sblk_cnt);
				return;
			}

			addr = sblk_base[sblk_id].base_addr;

			/*
			 * Get the original bypass flag, if user has already
			 * set the bypass flag 1 from sysfs, updating hardware
			 * bypass regs using 1, or else using the original
			 * bypass flag.
			 */
			bypass = (ori_map & (1 << i)) >> i;
			if (cur_map & 1) {
				bypass = SBLK_BYPASS;
				pr_debug("isp: bypass %s, com_idx:0x%x\n",
					 sblk_base[sblk_id].name, idx);
			}

			ISP_REG_MWR(idx, addr, BIT_0, bypass);

			cur_map >>= 1;
		}
	}
	pr_debug("exit\n");
}
EXPORT_SYMBOL(isp_dbg_bypass_sblk);

/*
 * sys/sprd_image/isp_dbg can control This
 * function's process, reg_start, reg_end...
 * This function can be called anywhere, according
 * to user's request.
 */
void isp_dbg_reg_trace(struct isp_pipe_dev *dev, unsigned int idx)
{
	unsigned int i = 0;
	unsigned long reg_start, reg_end, addr = 0;
	unsigned int limit_size = 0;
	unsigned int to_log_word_size = 0;
	unsigned int remain_word_size = 0;
	struct isp_dbg_info *dbg = NULL;

	pr_debug("enter, cb:%pS\n", __builtin_return_address(0));
	if (!dev || !dev->cam_grp) {
		pr_err("fail to exit1 exception\n");
		return;
	}

	dbg = &dev->cam_grp->dbg_info.isp_dbg;
	if (!dbg || !dbg->dbg_on) {
		pr_debug("exit2\n");
		return;
	}

	limit_size = dbg->dump2log_max_word_size;

	for (i = 0; i < ARRAY_SIZE(dbg->dump_range); i += 2) {
		/*
		 * TODO: maybe necessary using a thread to process
		 * the remain_word_size and save the dump data into
		 * a specified file.
		 * Now I just ignore the remain and output warning.
		 */
		reg_start = dbg->dump_range[i];
		reg_end = dbg->dump_range[i+1];
		to_log_word_size = reg_end - reg_start + 1;
		if (reg_end == 0 || to_log_word_size == 0)
			return;

		if (to_log_word_size > limit_size) {
			remain_word_size = to_log_word_size - limit_size;
			reg_end = reg_start +  limit_size;
			pr_warn("larger than %d words, ignore the remainings\n",
				limit_size);
		}

		pr_info("====Dump ISP REG Section-%d, com_idx 0x%x, cb:%pS\n",
			i/2, idx, __builtin_return_address(0));

		pr_info("--HW REGS--\n");
		for (addr = reg_start; addr <= reg_end; addr += 16) {
			pr_info("0x%5lx:	0x%8x  %8x  %8x  %8x\n",
				addr,
				ISP_HREG_RD(idx, addr),
				ISP_HREG_RD(idx, addr + 4),
				ISP_HREG_RD(idx, addr + 8),
				ISP_HREG_RD(idx, addr + 12));
		}

		if (ISP_GET_MID(idx) == ISP_CFG_MODE) {
			pr_info("--CFG PAGE REGS--\n");
			for (addr = reg_start; addr <= reg_end; addr += 16) {
				pr_info("0x%5lx:	0x%8x  %8x  %8x  %8x\n",
					addr,
					ISP_PAGE_REG_RD(idx, addr),
					ISP_PAGE_REG_RD(idx, addr + 4),
					ISP_PAGE_REG_RD(idx, addr + 8),
					ISP_PAGE_REG_RD(idx, addr + 12));
			}
		}
	}
	pr_debug("exit\n");
}
EXPORT_SYMBOL(isp_dbg_reg_trace);

/*
 * /sys/sprd_image/fmcu_dbg can control This
 * function's process.
 * This function can be called anywhere, according
 * to user's request.
 */
void isp_dbg_dump_fmcu_cmd_q(struct isp_pipe_dev *dev)
{
	unsigned int i = 0;
	unsigned long addr = 0;
	struct isp_dbg_info *dbg = NULL;

	pr_debug("enter, cb:%pS\n", __builtin_return_address(0));
	if (!dev || !dev->fmcu_addr_vir || !dev->cam_grp) {
		pr_err("fail to exception exit1\n");
		return;
	}

	dbg = &dev->cam_grp->dbg_info.isp_dbg;
	if (!dbg || !dbg->dbg_on || !dbg->fmcu_dbg_on) {
		pr_debug("exit2\n");
		return;
	}

	addr = (unsigned long)dev->fmcu_addr_vir;
	pr_info("====Dump ISP FMCU cmd queue here====\n"
		"fmcu slice cmd num total:%d, cb:%pS\n",
		dev->fmcu_slice.fmcu_num,
		__builtin_return_address(0));

	for (i = 0; i <= dev->fmcu_slice.fmcu_num; i += 2) {
		pr_info("0x%5lx: 0x%8x 0x%8x 0x%8x 0x%8x\n",
			addr,
			REG_RD(addr),
			REG_RD(addr + 4),
			REG_RD(addr + 8),
			REG_RD(addr + 12));
		addr += 16;
	}
	pr_debug("exit\n");
}
EXPORT_SYMBOL(isp_dbg_dump_fmcu_cmd_q);

/*================end of ISP debug control=====================*/

static void sprd_isp_reset_fmcu(void)
{
	unsigned int flag = 0;

	flag = BIT(7);
	regmap_update_bits(cam_ahb_gpr,
	REG_MM_AHB_AHB_RST, flag, flag);
	udelay(1);
	regmap_update_bits(cam_ahb_gpr,
	REG_MM_AHB_AHB_RST, flag, ~flag);
}

static int offline_bin_thread_loop(void *arg)
{
	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)arg;

	if (dev == NULL) {
		pr_err("fail to get ptr dev is NULL\n");
		return -1;
	}
	pr_debug("+\n");
	while (!dev->is_offline_bin_thread_stop) {
		wait_for_completion(&dev->offline_bin_thread_com);
		if (!dev->is_offline_bin_thread_stop)
			sprd_isp_pipeline_proc_bin(arg);
	}
	complete(&dev->bin_stop);
	pr_info("-\n");

	return 0;
}

static int offline_full_thread_loop(void *arg)
{
	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)arg;

	if (dev == NULL) {
		pr_err("fail to get ptr dev is NULL\n");
		return -1;
	}
	pr_debug("+\n");
	while (!dev->is_offline_full_thread_stop) {
		wait_for_completion(&dev->offline_full_thread_com);
		if (!dev->is_offline_full_thread_stop) {
			dev->is_wait_fmcu = 1;
			wait_for_completion(&dev->fmcu_com);
			if (!dev->is_offline_full_thread_stop) {
				if (sprd_isp_pipeline_proc_full(arg)) {
					pr_err("fail to start capture!\n");
					spin_lock(&isp_mod_lock);
					fmcu_slice_capture_state = ISP_ST_STOP;
					spin_unlock(&isp_mod_lock);
					sprd_isp_reset_fmcu();
				}
			}
		}
	}
	complete(&dev->full_stop);
	pr_info("-\n");

	return 0;
}

static int isp_create_offline_thread(void *param)
{
	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)param;
	char thread_name[20] = { 0 };
	enum isp_id iid = ISP_ID_0;

	if (dev == NULL) {
		pr_err("fail to get ptr dev is NULL\n");
		return -1;
	}

	iid = ISP_GET_IID(dev->com_idx);

	dev->is_wait_fmcu = 0;
	init_completion(&dev->fmcu_com);
	complete(&dev->fmcu_com);

	dev->is_offline_bin_thread_stop = 0;
	init_completion(&dev->offline_bin_thread_com);
	sprintf(thread_name, "isp%d_bin", iid);
	dev->offline_bin_thread = kthread_run(offline_bin_thread_loop,
					      param, thread_name);
	if (IS_ERR(dev->offline_bin_thread)) {
		pr_err("fail to create bin thread %ld\n",
				PTR_ERR(dev->offline_bin_thread));
		dev->offline_bin_thread = NULL;
		return -1;
	}

	dev->is_offline_full_thread_stop = 0;
	init_completion(&dev->bin_stop);
	init_completion(&dev->offline_full_thread_com);
	sprintf(thread_name, "isp%d_full", iid);
	dev->offline_full_thread = kthread_run(offline_full_thread_loop,
					      param, thread_name);
	if (IS_ERR(dev->offline_full_thread)) {
		pr_err("fail to create full %ld\n",
				PTR_ERR(dev->offline_full_thread));
		dev->offline_full_thread = NULL;
		return -1;
	}
	init_completion(&dev->full_stop);

	return 0;
}

static int isp_stop_offline_thread(void *param)
{
	struct isp_pipe_dev *dev = (struct isp_pipe_dev *)param;

	if (dev == NULL) {
		pr_err("fail to get ptr dev is NULL\n");
		return -1;
	}
	if (dev->offline_bin_thread) {
		dev->is_offline_bin_thread_stop = 1;
		complete(&dev->offline_bin_thread_com);
		wait_for_completion(&dev->bin_stop);
		dev->offline_bin_thread = NULL;
	}

	if (dev->offline_full_thread) {
		dev->is_offline_full_thread_stop = 1;
		if (dev->is_wait_fmcu)
			complete(&dev->fmcu_com);
		else
			complete(&dev->offline_full_thread_com);
		wait_for_completion(&dev->full_stop);
		dev->offline_full_thread = NULL;
	}

	return 0;
}


void isp_wait_update_done(struct isp_module *module,
			  enum isp_path_index path_index, unsigned int *p_flag)
{
	int ret = 0;
	struct isp_path_desc *p_path = NULL;
	struct isp_path_desc *pre = NULL;
	struct isp_path_desc *vid = NULL;
	struct isp_path_desc *cap = NULL;
	enum isp_scl_id scl_path_id;
	unsigned long flag;

	if (!module)
		return;

	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];
	cap = &module->isp_path[ISP_SCL_CAP];

	if (path_index >= ISP_PATH_IDX_PRE && path_index <= ISP_PATH_IDX_CAP) {
		scl_path_id = isp_get_scl_index(path_index);
		p_path = &module->isp_path[scl_path_id];
		pr_debug("update done wait %d, %d\n",
			   p_path->wait_for_sof, p_path->sof_com.done);

		spin_lock_irqsave(&isp_mod_lock, flag);
		if (p_flag) {
			*p_flag = 1;
			if (pre->valid)
				pre->is_update = 1;
			if (vid->valid)
				vid->is_update = 1;
			if (cap->valid)
				cap->is_update = 1;
			if (module->off_desc.valid)
				module->off_desc.is_update = 1;
		}
		p_path->wait_for_sof = 1;
		spin_unlock_irqrestore(&isp_mod_lock, flag);
		ret = wait_for_completion_timeout(&p_path->sof_com,
						  ISP_PATH_TIMEOUT);
		if (ret == 0) {
			/* isp_reg_trace(); */
			pr_err("fail to wait update path 0x%x done\n",
			path_index);
		}
	} else {
		pr_info("wrong index 0x%x\n", path_index);
	}
}

static void isp_wait_path_done(struct isp_module *module,
	enum isp_path_index path_index, unsigned int *p_flag)
{
	int ret = 0;
	struct isp_path_desc *p_path = NULL;
	enum isp_scl_id scl_path_id;
	unsigned long flag;

	if (!module)
		return;

	if (path_index >= ISP_PATH_IDX_PRE && path_index <= ISP_PATH_IDX_CAP) {
		scl_path_id = isp_get_scl_index(path_index);
		p_path = &module->isp_path[scl_path_id];

		pr_debug("path done wait %d, %d\n",
			   p_path->wait_for_done, p_path->tx_done_com.done);

		spin_lock_irqsave(&isp_mod_lock, flag);
		if (p_flag)
			*p_flag = 1;

		p_path->wait_for_done = 1;
		spin_unlock_irqrestore(&isp_mod_lock, flag);
		pr_info("begin to wait tx_done_com: %d\n",
			p_path->tx_done_com.done);
		ret = wait_for_completion_timeout(&p_path->tx_done_com,
						  ISP_PATH_TIMEOUT);

		if (ret == 0) {
			/* isp_reg_trace(); */
			pr_err("fail to wait path 0x%x done\n",
				path_index);
		}
	} else {
		pr_info("wrong index 0x%x\n", path_index);
		return;
	}
}

static void sprd_isp_glb_reg_awr(unsigned int idx, unsigned long addr,
			   unsigned int val, unsigned int reg_id)
{
	unsigned long flag;
	enum isp_id iid = ISP_GET_IID(idx);

	switch (reg_id) {
	case ISP_AXI_REG:
		spin_lock_irqsave(&isp_glb_reg_axi_lock[iid], flag);
		ISP_HREG_WR(idx, addr, ISP_HREG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&isp_glb_reg_axi_lock[iid], flag);
		break;
	case ISP_INIT_MASK_REG:
		spin_lock_irqsave(&isp_glb_reg_mask_lock[iid], flag);
		ISP_HREG_WR(idx, addr, ISP_HREG_RD(idx, addr) & (val));
		spin_unlock_irqrestore(&isp_glb_reg_mask_lock[iid], flag);
		break;
	default:
		ISP_HREG_WR(idx, addr, ISP_HREG_RD(idx, addr) & (val));
		break;
	}
}

static void sprd_isp_glb_reg_owr(unsigned int idx, unsigned long addr,
			   unsigned int val, unsigned int reg_id)
{
	unsigned long flag;
	enum isp_id iid = ISP_GET_IID(idx);

	switch (reg_id) {
	case ISP_AXI_REG:
		spin_lock_irqsave(&isp_glb_reg_axi_lock[iid], flag);
		ISP_HREG_WR(idx, addr, ISP_HREG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&isp_glb_reg_axi_lock[iid], flag);
		break;
	case ISP_INIT_MASK_REG:
		spin_lock_irqsave(&isp_glb_reg_mask_lock[iid], flag);
		ISP_HREG_WR(idx, addr, ISP_HREG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&isp_glb_reg_mask_lock[iid], flag);
		break;
	case ISP_INIT_CLR_REG:
		spin_lock_irqsave(&isp_glb_reg_clr_lock[iid], flag);
		ISP_HREG_WR(idx, addr, ISP_HREG_RD(idx, addr) | (val));
		spin_unlock_irqrestore(&isp_glb_reg_clr_lock[iid], flag);
		break;
	default:
		ISP_HREG_WR(idx, addr, ISP_HREG_RD(idx, addr) | (val));
		break;
	}
}

static void do_reset_isp_hw(struct isp_pipe_dev *dev)
{
	unsigned int idx = dev->com_idx;
	unsigned int flag = 0;
	struct isp_cctx_desc *cctx_desc = NULL;

	cctx_desc = dev->module_info.cctx_desc;

	flag = BIT(5) | BIT(6) | BIT(7);
	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, flag);
	udelay(1);
	regmap_update_bits(cam_ahb_gpr,
		REG_MM_AHB_AHB_RST, flag, ~flag);

	sprd_isp_glb_reg_awr(idx, ISP_AXI_ITI2AXIM_CTRL,
				~BIT_26, ISP_AXI_REG);
	/*
	 * After do_reset_isp_hw, isp cfg map buf
	 * will be reset together, so it's necessary
	 * to reset cfg_map_lock to 0 here, and then
	 * cfg map will be re-inited after isp_reset.
	 */
	atomic_set(&dev->cfg_map_lock, 0);
	sprd_iommu_restore(&s_isp_pdev->dev);

	/*
	 * reset isp cfg page buf to default value after isp
	 * reset, like actual operation in H/W.
	 */
	if (cctx_desc)
		cctx_desc->intf->rst_page_buf(cctx_desc,
			ISP_GET_IID(idx));
}

void isp_irq_ctrl(struct isp_pipe_dev *dev, bool enable)
{
	unsigned int idx = dev->com_idx;
	enum isp_id iid = ISP_GET_IID(dev->com_idx);
	enum isp_scene_id sid = ISP_SCENE_PRE;
	unsigned int base_addr = 0;
	unsigned int set_idx = 0;
	unsigned int clr_reg = 0;
	unsigned int en_reg = 0;
	unsigned int en_mask = 0;

	for (sid = ISP_SCENE_PRE; sid < ISP_SCENE_NUM; sid++) {
		base_addr = int_reg_base[iid][sid];
		for (set_idx = 0;
		     set_idx < ISP_INT_REG_SETS_NUM;
		     set_idx++) {
			clr_reg = irq_sets[set_idx].offset[INT_REG_OFF_CLR];
			en_reg = irq_sets[set_idx].offset[INT_REG_OFF_EN];
			en_mask = irq_sets[set_idx].msk_bmap[sid];

			if (enable) {
				sprd_isp_glb_reg_owr(idx,
						     base_addr + clr_reg,
						     0xFFFFFFFF,
						     ISP_INIT_CLR_REG);
				sprd_isp_glb_reg_owr(idx,
						     base_addr + en_reg,
						     en_mask,
						     ISP_INIT_MASK_REG);
			} else {
				sprd_isp_glb_reg_awr(idx,
						     base_addr + en_reg,
						     0,
						     ISP_INIT_MASK_REG);
				sprd_isp_glb_reg_owr(idx,
						     base_addr + clr_reg,
						     0xFFFFFFFF,
						     ISP_INIT_CLR_REG);
			}
		}
	}
}

/*
 * TODO: should be compatible with dual-cam
 */
static int sprd_isp_reset(struct isp_pipe_dev *dev)
{
	enum dcam_drv_rtn rtn = ISP_RTN_SUCCESS;
	enum isp_id iid = ISP_ID_0;
	unsigned int idx = 0;
	struct isp_cctx_desc *cctx_desc = NULL;

	if (!dev) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	cctx_desc = dev->module_info.cctx_desc;
	if (!cctx_desc) {
		pr_err("fail to get ptr of cctx_desc\n");
		return -EFAULT;
	}

	idx = dev->com_idx;
	iid = ISP_GET_IID(idx);
	pr_info("iid %d, is_dual_cam %d\n", iid, is_dual_cam);

	do_reset_isp_hw(dev);

	/*
	 * This cfg_map_init func should be called
	 * after isp reset.
	 */
	rtn = cctx_desc->intf->init_cfg_map(dev);
	if (rtn)
		pr_err("fail to init cfg map,iid%d %d\n",
		       iid, rtn);

	return -rtn;
}

static int wait_for_pipeline_proc_stop(void *handle)
{
	int loop = 0;
	struct isp_pipe_dev *dev = NULL;

	if (!handle) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)handle;
	pr_debug("enter\n");

	while (fmcu_slice_capture_state != ISP_ST_STOP ||
	       dev->wait_full_tx_done != WAIT_CLEAR) {

		usleep_range(1000, 1100);

		if (loop++ > FMCU_STOP_WAIT_CNT) {
			if (fmcu_slice_capture_state != ISP_ST_STOP)
				pr_err("fail to stop fmcu timeout!\n");
			if (dev->wait_full_tx_done != WAIT_CLEAR)
				pr_info("wait_full_tx_done was not cleared, %u\n",
					dev->wait_full_tx_done);
			sprd_isp_reset_fmcu();
			break;
		}
	}

	pr_debug("exit\n");

	return 0;
}

static void isp_quickstop(struct isp_module *module)
{
	unsigned int idx = 0;
	struct isp_pipe_dev *dev = NULL;
	unsigned int time_out = 0;
	enum isp_id iid;

	if (module == NULL) {
		pr_err("fail to get ptr\n");
		return;
	}

	dev = container_of(module, struct isp_pipe_dev, module_info);
	idx = dev->com_idx;
	iid = ISP_GET_IID(idx);

	sprd_isp_glb_reg_owr(idx, ISP_AXI_ITI2AXIM_CTRL, BIT_26, ISP_AXI_REG);

	/* then wait for AXI busy cleared */
	while (++time_out < ISP_AXI_STOP_TIMEOUT) {
		if (1 == ((ISP_HREG_RD(idx, ISP_INT_STATUS) & BIT_3) >> 3))
			break;
	}

	if (time_out >= ISP_AXI_STOP_TIMEOUT) {
		pr_warn("ISP%d: reset timeout %d\n", iid, time_out);
		return;
	}
	udelay(10);

	pr_info("ISP%d:ISP_AXI_ITI2AXIM_CTRL 0x%x STATUS 0x%x\n",
		ISP_GET_IID(idx), ISP_HREG_RD(idx, ISP_AXI_ITI2AXIM_CTRL),
		ISP_HREG_RD(idx, ISP_INT_INT0));
}

static int sprd_isp_stop_nolock(void *isp_handle, int is_post_stop)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	unsigned int i = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_statis_module *statis_module = NULL;
	struct isp_k_block *isp_k_param = NULL;

	if (isp_handle == NULL) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	if (module == NULL) {
		pr_err("fail to get isp module is null\n");
		return -EFAULT;
	}
	/* There are 2 phases of ISP stop involving spinlock */
	if (is_post_stop) {
		sprd_isp_reset(dev);
		isp_irq_ctrl(dev, false);
		return rtn;
	}

	statis_module = &dev->statis_module_info;
	isp_k_param = &dev->isp_k_param;

	isp_quickstop(module);
	isp_irq_ctrl(dev, false);

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

	if (dev->cap_on)
		dev->cap_on = 0;

	dev->isr_count = 0;
	dev->isr_last_time = 0;

	if (fmcu_slice_capture_state != ISP_ST_STOP) {
		pr_warn("resetting fmcu_slice_capture_state\n");
		fmcu_slice_capture_state = ISP_ST_STOP;
		sprd_isp_reset_fmcu();
	}

	for (i = ISP_SCL_0; i < ISP_SCL_CAP; i++) {
		module->isp_path[i].status = ISP_ST_STOP;
		module->isp_path[i].valid = 0;
		module->isp_path[i].shadow_done_cnt = 0;
		isp_frm_clear(dev, 1 << i);
	}


	isp_offline_init_buf(&module->off_desc, ISP_OFF_BUF_BIN, true);
	isp_offline_init_buf(&module->off_desc, ISP_OFF_BUF_FULL, true);

	isp_statis_queue_init(&statis_module->aem_statis_queue);
	isp_statis_queue_init(&statis_module->afl_statis_queue);
	isp_statis_queue_init(&statis_module->afm_statis_queue);
	isp_statis_queue_init(&statis_module->binning_statis_queue);
	isp_statis_queue_init(&statis_module->hist_statis_queue);
	isp_statis_queue_init(&statis_module->hist2_statis_queue);

	isp_statis_frm_queue_init(&statis_module->afl_statis_frm_queue);
	isp_statis_frm_queue_init(&statis_module->afm_statis_frm_queue);
	isp_statis_frm_queue_init(&statis_module->binning_statis_frm_queue);
	isp_statis_frm_queue_init(&statis_module->hist_statis_frm_queue);
	isp_statis_frm_queue_init(&statis_module->hist2_statis_frm_queue);

	isp_coeff_queue_init(module->scl_array);

	pfiommu_free_addr(&isp_k_param->fetch_pfinfo);
	memset(&isp_k_param->fetch_pfinfo, 0, sizeof(struct pfiommu_info));

	isp_offline_buf_iommu_unmap(&module->off_desc, ISP_OFF_BUF_BIN);
	isp_offline_buf_iommu_unmap(&module->off_desc, ISP_OFF_BUF_FULL);

	module->off_desc.valid = 0;
	module->off_desc.status = ISP_ST_STOP;
	module->off_desc.shadow_done_cnt = 0;

	return -rtn;
}

#define isp_lock_all(dev, flag, flag_p, flag_c)	\
	do {	\
		spin_lock_irqsave(&dev->pre_lock, flag_p);	\
		spin_lock_irqsave(&dev->cap_lock, flag_c);	\
		spin_lock_irqsave(&isp_mod_lock, flag);	\
	} while (0)

#define isp_unlock_all(dev, flag, flag_p, flag_c)	\
	do {	\
		spin_unlock_irqrestore(&isp_mod_lock, flag);	\
		spin_unlock_irqrestore(&dev->cap_lock, flag_c);	\
		spin_unlock_irqrestore(&dev->pre_lock, flag_p);	\
	} while (0)

static int sprd_isp_stop_lock(void *isp_handle, int is_post_stop)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	unsigned long flags;
	unsigned long flags_p;
	unsigned long flags_c;
	struct isp_pipe_dev *dev;

	if (isp_handle == NULL) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;

	isp_lock_all(dev, flags, flags_p, flags_c);
	rtn = sprd_isp_stop_nolock(isp_handle, is_post_stop);
	isp_unlock_all(dev, flags, flags_p, flags_c);

	return rtn;
}
#undef isp_lock_all
#undef isp_unlock_all

int sprd_isp_stop(void *isp_handle, int is_irq)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev;
	enum isp_id iid = ISP_ID_0, iid_other = ISP_ID_1;

	dev = (struct isp_pipe_dev *)isp_handle;
	iid = ISP_GET_IID(dev->com_idx);
	iid_other = ISP_OTHER_ID(iid);

	if (is_irq)
		rtn = sprd_isp_stop_nolock(isp_handle, 0);
	else
		rtn = sprd_isp_stop_lock(isp_handle, 0);

	if (rtn)
		pr_warn("fail to stop ISP, %s\n",
			is_irq?"not locked":"locked");

	if (g_isp_dev_parray[iid_other] == NULL ||
		!g_isp_dev_parray[iid_other]->module_info.off_desc.valid)
		pr_info("ISP %d stopped but not reset yet, ISP %d invalid.\n",
			iid, iid_other);

	if (is_irq)
		rtn = sprd_isp_stop_nolock(isp_handle, 1);
	else
		rtn = sprd_isp_stop_lock(isp_handle, 1);

	pr_info("isp stop\n");

	return rtn;
}

static int get_slice_init_scaler(struct slice_scaler_path *scaler,
	struct isp_path_desc *path)
{
	if (!scaler || !path) {
		pr_err("fail to get ptr\n");
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
					 struct camera_frame *frame,
					 unsigned int idx)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_path_desc *pre = NULL;
	struct isp_path_desc *vid = NULL;
	struct isp_path_desc *cap = NULL;
	struct isp_offline_desc *off_desc = NULL;
	struct isp_module *module = NULL;
	enum isp_id iid = ISP_ID_0;
	struct isp_sc_array *scl_array = NULL;

	if (!dev || !frame) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	module = &dev->module_info;
	iid = ISP_GET_IID(idx);
	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];
	cap = &module->isp_path[ISP_SCL_CAP];
	off_desc = &module->off_desc;
	scl_array = module->scl_array;

	if (module->isp_path[ISP_SCL_CAP].valid) {
		cap->src.w = frame->width;
		cap->src.h = frame->height;
		if (unlikely(dev->is_raw_capture)) {
			cap->src.w =  cap->in_size.w;
			cap->src.h = cap->in_size.h;
			pr_debug("RAW:set cap src w*h:%d,%d\n", cap->src.w,
					cap->src.h);
		}
		cap->trim0_info.start_x = cap->in_rect.x;
		cap->trim0_info.start_y = cap->in_rect.y;
		cap->trim0_info.size_x = cap->in_rect.w;
		cap->trim0_info.size_y = cap->in_rect.h;
		cap->dst.w = cap->out_size.w;
		cap->dst.h = cap->out_size.h;
		cap->trim1_info.start_x = 0;
		cap->trim1_info.start_y = 0;
		cap->trim1_info.size_x = cap->dst.w;
		cap->trim1_info.size_y = cap->dst.h;

		pr_debug("src_x:%d, src_y:%d\n",
			cap->src.w, cap->src.h);

		rtn = isp_path_scaler(module, ISP_PATH_IDX_CAP, cap,
			&scl_array->coeff[ISP_SCL_CAP]);
		if (rtn) {
			pr_err("fail to isp scaler path code%d", rtn);
			return rtn;
		}
		isp_path_set(module, cap, ISP_PATH_IDX_CAP);

		if (dev->is_3dnr) {
			/* use input data from cap path */
			vid->src = cap->src;
			vid->in_size = cap->in_size;
			vid->trim0_info = cap->trim0_info;

			vid->dst.w = vid->out_size.w;
			vid->dst.h = vid->out_size.h;
			vid->trim1_info.start_x = 0;
			vid->trim1_info.start_y = 0;
			vid->trim1_info.size_x = vid->dst.w;
			vid->trim1_info.size_y = vid->dst.h;
			vid->valid = 1;
			rtn = isp_path_scaler(module, ISP_PATH_IDX_VID, vid,
					&scl_array->coeff[ISP_SCL_VID]);
			if (rtn) {
				pr_err("fail to isp scaler path code %d", rtn);
				return rtn;
			}

			isp_path_set(module, vid, ISP_PATH_IDX_VID);
		}
	}

	return rtn;
}

static int get_slice_init_param(struct slice_param_in *in_ptr,
				struct isp_pipe_dev *dev,
				struct camera_frame *frame,
				unsigned int idx)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_path_desc *path_pre = NULL;
	struct isp_path_desc *path_vid = NULL;
	struct isp_path_desc *path_cap = NULL;
	struct isp_offline_desc *off_desc = NULL;
	struct slice_scaler_path *scaler = NULL;
	struct isp_module *module = NULL;

	if (!dev || !in_ptr) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	module = &dev->module_info;
	path_pre = &module->isp_path[ISP_SCL_PRE];
	path_vid = &module->isp_path[ISP_SCL_VID];
	path_cap = &module->isp_path[ISP_SCL_CAP];
	off_desc = &module->off_desc;

	in_ptr->iid = ISP_GET_IID(idx);
	in_ptr->sid = ISP_GET_SID(idx);
	in_ptr->mid = ISP_GET_MID(idx);

	if (dev->is_raw_capture) {
		/* get the fetch address, width and height */
		in_ptr->fetch_addr.chn0 =
			isp_k_fetch_get_raw_info(&in_ptr->img_size.width,
						 &in_ptr->img_size.height);
		in_ptr->fetch_addr.chn1 = 0;
		in_ptr->fetch_addr.chn2 = 0;
	} else {
		pr_debug("yaddr 0x%x yaddr_vir 0x%x fd 0x%x s_m_i-id:%d%d%d\n",
			 frame->yaddr, frame->yaddr_vir, frame->pfinfo.mfd[0],
			 in_ptr->sid, in_ptr->mid, in_ptr->iid);

		in_ptr->fetch_addr.chn0 = frame->yaddr_vir;
		in_ptr->fetch_addr.chn1 = frame->uaddr;
		in_ptr->fetch_addr.chn2 = frame->vaddr;
		in_ptr->img_size.width = frame->width;
		in_ptr->img_size.height = frame->height;
	}

	if (!dev->fmcu_addr_vir) {
		pr_err("fail to get fmcu_addr_vir\n");
		return -EFAULT;
	}
	in_ptr->fmcu_addr_vir = dev->fmcu_addr_vir;
	in_ptr->isp_dev = dev;

	if (path_pre->valid && path_pre->path_mode == ISP_PRE_OFFLINE) {
		pr_info("init for pre slice");
		in_ptr->fetch_format = path_pre->input_format;
		in_ptr->pre_slice_need = 1;
		in_ptr->store_frame[SLICE_PATH_PRE].format =
			path_pre->store_info.color_format;
		in_ptr->store_frame[SLICE_PATH_PRE].size.width =
			path_pre->dst.w;
		in_ptr->store_frame[SLICE_PATH_PRE].size.height =
			path_pre->dst.h;
		scaler = &in_ptr->scaler_frame[SLICE_PATH_PRE];
		ret = get_slice_init_scaler(scaler, path_pre);
		pr_debug("get fetch format:%d, path_idx:%d\n",
			 in_ptr->fetch_format, ISP_PATH_IDX_PRE);
		ret = isp_path_set_next_frm(module, ISP_PATH_IDX_PRE,
			&in_ptr->store_frame[SLICE_PATH_PRE].addr);
	} else {
		in_ptr->pre_slice_need = 0;
	}

	if (path_vid->valid && path_vid->path_mode == ISP_VID_OFFLINE) {
		pr_info("init for vid slice");
		in_ptr->vid_slice_need = 1;
		in_ptr->fetch_format = path_vid->input_format;
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
	} else {
		in_ptr->vid_slice_need = 0;
	}

	if (path_cap->valid) {
		pr_info("init for cap slice");
		in_ptr->cap_slice_need = 1;
		in_ptr->fetch_format = path_cap->input_format;
		in_ptr->store_frame[SLICE_PATH_CAP].format =
			path_cap->store_info.color_format;
		in_ptr->store_frame[SLICE_PATH_CAP].size.width =
			path_cap->dst.w;
		in_ptr->store_frame[SLICE_PATH_CAP].size.height =
			path_cap->dst.h;
		scaler = &in_ptr->scaler_frame[SLICE_PATH_CAP];
		pr_debug("get fetch format:%d, path_idx:%d\n",
			 in_ptr->fetch_format, ISP_PATH_IDX_CAP);
		ret = get_slice_init_scaler(scaler, path_cap);
		ret = isp_path_set_next_frm(module, ISP_PATH_IDX_CAP,
			&in_ptr->store_frame[SLICE_PATH_CAP].addr);

		if (dev->is_3dnr) {
			struct camera_size *out_size = &path_vid->out_size;

			pr_info("init for 3dnr slice");
			in_ptr->vid_slice_need = 1;
			in_ptr->store_frame[SLICE_PATH_VID].format =
				path_cap->store_info.color_format;
			in_ptr->store_frame[SLICE_PATH_VID].size.width =
				out_size->w;
			in_ptr->store_frame[SLICE_PATH_VID].size.height =
				out_size->h;
			scaler = &in_ptr->scaler_frame[SLICE_PATH_VID];
			ret = get_slice_init_scaler(scaler, path_vid);

			in_ptr->store_frame[SLICE_PATH_VID].addr.chn0 =
				in_ptr->store_frame[SLICE_PATH_CAP].addr.chn0 +
				(path_cap->dst.w) * (path_cap->dst.h)*3/2;
			in_ptr->store_frame[SLICE_PATH_VID].addr.chn1 =
				in_ptr->store_frame[SLICE_PATH_VID].addr.chn0 +
				(out_size->w) * (out_size->h);

			in_ptr->store_frame[SLICE_PATH_VID].addr.chn2 = 0;

		}
	} else {
		in_ptr->cap_slice_need = 0;
	}

	in_ptr->is_raw_capture = dev->is_raw_capture;

	pr_debug("End get slice init param\n");
	return ret;
}

#if 0
static void sprd_isp_cap_hblank_cfg(void *handle)
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
		pr_err("handle fail to get ptr\n");
		return;
	}

	dev = (struct isp_pipe_dev *)handle;

	cap_path = &dev->module_info.isp_path[ISP_SCL_CAP];

	if (!cap_path) {
		pr_err("cap_path fail to get ptr\n");
		return;
	}

	width = cap_path->in_rect.w;
	height = cap_path->in_rect.h;

	if (cap_path->in_rect.w == 0 || dev->is_raw_capture == 1)
		return;
	zoom = cap_path->out_size.w / cap_path->in_rect.w;

	if (cap_path->out_size.w % cap_path->in_rect.w != 0)
		zoom += 1;

	hblank1 = (width + hblank0) * (height + factor) /
		(height / zoom + factor) - (width / zoom);

	ISP_REG_WR(dev->idx, ISP_FETCH2_LINE_DLY_CTRL, hblank1);

	pr_debug("out:w:%d h:%d, in:w:%d h:%d, hblank1:0x%x, zoom:%d\n",
		cap_path->out_size.w, cap_path->out_size.h,
		width, height, hblank1, zoom);
}
#endif

int sprd_isp_start_pipeline_bin(void *handle, unsigned int cap_flag)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	enum isp_id iid = ISP_ID_0;
	struct isp_offline_desc *off_desc = NULL;
	struct offline_buf_desc *buf_desc = NULL;
	struct camera_frame frame;
	struct isp_module *module = NULL;
	struct camera_frame *p_offline_frame = NULL;
	int consumer_q_size = is_dual_cam ? ISP_OFF_CONSUMER_Q_SIZE_MULTI :
		ISP_OFF_CONSUMER_Q_SIZE;

	pr_debug("enter\n");

	if (!handle) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)handle;
	iid = ISP_GET_IID(dev->com_idx);
	module = &dev->module_info;
	off_desc = &module->off_desc;
	buf_desc = &off_desc->buf_desc_bin;

	if (buf_desc->zsl_queue.valid_cnt > consumer_q_size) {
		ret = isp_buf_recycle(buf_desc, &buf_desc->tmp_buf_queue,
				      &buf_desc->zsl_queue, 1);
		if (ret)
			goto exit;
	}

	if (dev->is_raw_capture) {
		if (cap_flag != DCAM_CAPTURE_NONE) {
			complete(&dev->offline_bin_thread_com);
			pr_info("bin path raw cap\n");
		} else
			pr_info("bin raw cap:skip this cap request");
		goto exit;
	}

	if (buf_desc->zsl_queue.valid_cnt == 0) {
		ret = ISP_RTN_SUCCESS;
		pr_info("wait new zsl frame\n");
		goto exit;
	}

	/*
	 * In case of bin path is done but isp not ready,
	 * need to check if isp is idle here. If isp is not
	 * idle, just return and miss current bin frame, and
	 * this flag dev->pre_state will be set to ISP_ST_STOP
	 * in isp_all_done function in isr.
	 */
	if (dev->pre_state == ISP_ST_START) {
		dev->bin_path_miss_cnt += 1;
		pr_debug("dev->pre_state not done yet, miss this frame. %d",
			 dev->bin_path_miss_cnt);
		isp_dbg_reg_trace(dev, dev->com_idx);
		goto exit;
	}

	if (isp_frame_dequeue(&buf_desc->zsl_queue, &frame)) {
		pr_err("fail to deque offline_buf bin_path frame\n");
		ret = -1;
		goto exit;
	}

	p_offline_frame = &dev->offline_frame[ISP_OFF_BUF_BIN];
	memcpy(p_offline_frame, &frame, sizeof(struct camera_frame));

	dev->pre_state = ISP_ST_START;
	complete(&dev->offline_bin_thread_com);

exit:
	pr_debug("exit\n");

	return ret;
}

static int wait_for_full_tx_done(struct isp_pipe_dev *dev)
{
	int ret = 0;
	unsigned int flag;
	int loop = 0;

	dev->wait_full_tx_done = WAIT_BEGIN;
	flag = atomic_read(&dcam_full_path_time_flag);
	pr_debug("waiting for full tx done, cur_flag:%d ...", flag);
	while (flag != DCAM_TIME_TX_DONE) {
		if (flag == DCAM_TIME_NULL) {
			pr_err("fail to wait for full path tx done!\n");
			return -EFAULT;
		}
		usleep_range(1000, 1100);
		flag = atomic_read(&dcam_full_path_time_flag);
		if (loop++ > WAIT_CNT_MS) {
			pr_err("fail to wait for full tx done TimeOut!\n");
			/*
			 * FIXME: maybe need exception process here,
			 * when waiting time_out
			 * <goto time_out_exit;>
			 */
			return -EFAULT;
		}
	}

	dev->wait_full_tx_done = WAIT_DONE;
	atomic_set(&dcam_full_path_time_flag, DCAM_TIME_NULL);
	pr_debug("waiting for full tx done successfully!!");

	return ret;
}

static int sprd_isp_sel_cap_frm(struct isp_pipe_dev *dev)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev_other = NULL;
	enum isp_id iid = ISP_ID_0, iid_other = ISP_ID_1;
	struct isp_module *module = NULL;
	struct isp_offline_desc *off_desc = NULL;
	struct offline_buf_desc *buf_desc = NULL;
	struct camera_frame frame;
	struct camera_frame *p_offline_frame = NULL;

	if (!dev) {
		pr_err("FATAL: fail to get valid parameters\n");
		return -EPERM;
	}

	iid = ISP_GET_IID(dev->com_idx);
	module = &dev->module_info;
	off_desc = &module->off_desc;
	buf_desc = &off_desc->buf_desc_full;

	if (dev->delta_full == 0) {
		/* the dev streaming first, get its frame first (for the ID) */
#define ISP_MULTICAM_START_FROM_OLDEST
#ifdef ISP_MULTICAM_START_FROM_OLDEST /* get the oldest frame */
		ret = isp_frame_dequeue(&buf_desc->zsl_queue, &frame);
		if (ret) {
			pr_err("fail to dequeue offline\n");
			goto err_exit;
		}
		ret = isp_buf_recycle(buf_desc,
			&buf_desc->tmp_buf_queue,
			&buf_desc->zsl_queue,
			buf_desc->zsl_queue.valid_cnt - 1);
		if (ret) {
			pr_err("fail to dequeue offline\n");
			goto err_exit;
		}
		ret = isp_frame_enqueue(&buf_desc->zsl_queue, &frame);
		if (ret) {
			pr_err("fail to enqueue offline\n");
			goto err_exit;
		}
#else /* get the latest frame */
		ret = isp_buf_recycle(buf_desc,
			&buf_desc->tmp_buf_queue,
			&buf_desc->zsl_queue,
			buf_desc->zsl_queue.valid_cnt - 1);
		if (ret) {
			pr_err("fail to dequeue offline\n");
			goto err_exit;
		}
#endif
	} else {
		int i;
		int c = buf_desc->zsl_queue.valid_cnt;
		bool match = false;
		int target;

		iid_other = ISP_OTHER_ID(iid);
		dev_other = g_isp_dev_parray[iid_other];
		if (dev_other) {
			p_offline_frame =
				&dev_other->offline_frame[ISP_OFF_BUF_FULL];
			target = p_offline_frame->fid - dev->delta_full;
		} else {
			pr_err("FATAL: the other ISP is supposed to run.\n");
			goto err_exit;
		}
		for (i = 0; i < c; i++) {
			ret = isp_frame_dequeue(&buf_desc->zsl_queue,
				&frame);
			if (ret) {
				pr_err("fail to dequeue offline\n");
				goto err_exit;
			}
			if (frame.fid == target) {
				pr_info("match found %d\n",
					target);
				match = true;
				break;
			}
			if (isp_buf_queue_write(&buf_desc->tmp_buf_queue,
				&frame)) {
				pr_err("FATAL: fail to return off buf full\n");
				goto err_exit;
			}
		}
		if (!match)
			pr_warn("fail to find match %d, use last frame %d\n",
				target, frame.fid);
		else {
			/* clean the queue */
			ret = isp_buf_recycle(buf_desc,
				&buf_desc->tmp_buf_queue,
				&buf_desc->zsl_queue,
				buf_desc->zsl_queue.valid_cnt);
			if (ret) {
				pr_err("fail to dequeue offline\n");
				goto err_exit;
			}
		}
		/* put the matched frame into the queue */
		if (isp_frame_enqueue(&buf_desc->zsl_queue, &frame)) {
			pr_err("fail to enqueue offline\n");
			goto err_exit;
		}
	}

	return ret;

err_exit:
	pr_err("fail to exit, ret = %d\n", ret);
	return ret;
}

int sprd_isp_start_pipeline_full(void *handle, unsigned int cap_flag)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	enum isp_id iid = ISP_ID_0;
	struct isp_offline_desc *off_desc = NULL;
	struct offline_buf_desc *buf_desc = NULL;
	struct camera_frame frame;
	struct isp_module *module = NULL;
	struct camera_frame *p_offline_frame = NULL;
	unsigned int max_zsl_num;
	int consumer_q_size = is_dual_cam ? ISP_OFF_CONSUMER_Q_SIZE_MULTI :
		ISP_OFF_CONSUMER_Q_SIZE;

	if (!handle) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	pr_debug("enter, cap_flag:%d\n", cap_flag);

	dev = (struct isp_pipe_dev *)handle;
	iid = ISP_GET_IID(dev->com_idx);
	module = &dev->module_info;
	off_desc = &module->off_desc;
	buf_desc = &off_desc->buf_desc_full;

	/*
	 * dev->cap_on can be used to check if the capture button
	 * is "ON" or "OFF".
	 * In NON-ZSL scenario, this flag will be set first, then
	 * frame data coming, but
	 * In ZSL scenario, this flag will be set later than frame
	 * data coming.
	 */
	if (cap_flag == DCAM_CAPTURE_NONE)
		dev->frm_cnt++;
	else {
		if (is_dual_cam) {
			/*
			 * For dual-cam, start capture comes together and we
			 * want the timing very close, so drop frames before.
			 * need to recycle the frames before starting capture,
			 * using dev->frm_cnt to mark down the previous frame.
			 */
			pr_info("dual-cam frm_cnt %d, valid_cnt:%d\n",
				 dev->frm_cnt, buf_desc->zsl_queue.valid_cnt);

			ret = sprd_isp_sel_cap_frm(dev);
			if (ret)
				goto err_exit;
		}

		if (cap_flag == DCAM_CAPTURE_START_HDR && dev->is_hdr) {
			dev->cap_flag = DCAM_CAPTURE_START_HDR;

			/*
			 * start_capture may come during sof and tx_done,
			 * here make sure hdr process hold on until tx_done
			 */
			if (wait_for_full_tx_done(dev))
				goto err_exit;

			/*
			 * For hdr, only needs the frame after start capture,
			 * need to recycle the frames before starting capture,
			 * using dev->frm_cnt to mark down the previous frame.
			 */
			pr_debug("hdr frm_cnt %d, valid_cnt:%d\n",
				 dev->frm_cnt, buf_desc->zsl_queue.valid_cnt);
			ret = isp_buf_recycle(buf_desc,
					      &buf_desc->tmp_buf_queue,
					      &buf_desc->zsl_queue,
					      dev->frm_cnt);
			if (ret)
				goto err_exit;
		}

		if (cap_flag == DCAM_CAPTURE_START_3DNR) {
			if (dev->is_3dnr) {
				dev->frm_cnt_3dnr = ISP_3DNR_NUM;
			} else {
				pr_err("fail to start 3dnr\n");
				dev->frm_cnt_3dnr = 0;
				ret = -EFAULT;
				goto err_exit;
			}
		}

                if (cap_flag == DCAM_CAPTURE_START_WITH_TIMESTAMP) {
			dev->cap_flag = cap_flag;
		}

		dev->cap_on = 1;
	}

	/*
	 * 1, hdr requires 3 consecutive frames
	 * 2, 3dnr no need to set max_zsl_num to 5, if not necessary
	 *    to be continuous frames.
	 */
	if (dev->cap_flag == DCAM_CAPTURE_START_HDR)
		max_zsl_num = ISP_HDR_FRM_NUM;
	else
		max_zsl_num = consumer_q_size;

	pr_debug("iid%d frm_cnt %d, valid_cnt:%d\n", iid, dev->frm_cnt,
		 buf_desc->zsl_queue.valid_cnt);

	if (buf_desc->zsl_queue.valid_cnt > max_zsl_num) {
		ret = isp_buf_recycle(buf_desc, &buf_desc->tmp_buf_queue,
				      &buf_desc->zsl_queue, 1);
		if (ret)
			goto err_exit;
	}

	if (unlikely(dev->is_raw_capture)) {
		if (cap_flag != DCAM_CAPTURE_NONE) {
			pr_info("full path raw cap\n");
			goto normal_exit_raw;
		} else {
			pr_info("full raw cap:skip this cap request");
			goto normal_exit;
		}
	}

	if (cap_flag == DCAM_CAPTURE_START_WITH_FLASH) {
		dev->cap_flag = DCAM_CAPTURE_START_WITH_FLASH;
		/*
		 * start_capture may come during sof and tx_done,
		 * here make sure flash process hold until tx_done
		 */
		if (wait_for_full_tx_done(dev))
			goto err_exit;
		pr_debug("flash frm_cnt %d, valid_cnt:%d\n",
			 dev->frm_cnt, buf_desc->zsl_queue.valid_cnt);

		ret = isp_buf_recycle(buf_desc, &buf_desc->tmp_buf_queue,
				      &buf_desc->zsl_queue,
				      buf_desc->zsl_queue.valid_cnt);
		if (ret)
			goto err_exit;
	}

	if (dev->cap_flag == DCAM_CAPTURE_START_WITH_FLASH &&
	    dev->wait_full_tx_done == WAIT_BEGIN)
		goto normal_exit;

	if (buf_desc->zsl_queue.valid_cnt == 0) {
		pr_debug("wait new zsl frame\n");
		goto normal_exit;
	}

	/*
	 * For zsl,
	 * 1) maybe full path tx done, but user not start
	 * capture yet, in this scenario just return and again.
	 * 2) maybe pipeline_proc_full will be interrupted by
	 * new full path tx done, to avoid this situation, check
	 * if capture_state ISP_ST_START or not. capture_state will
	 * be ISP_ST_START until fmcu_config_done.
	 */
	if ((dev->cap_on == 0) ||
	    (fmcu_slice_capture_state == ISP_ST_START)) {
		pr_debug("not ready for cap yet, cap_on:%d\n", dev->cap_on);
		goto normal_exit;
	}

	if (dev->is_3dnr) {
		if (dev->frm_cnt_3dnr == 0) {
			pr_debug("frm_cnt_3dnr is zero, no need of capture\n");
			goto normal_exit;
		} else {
			pr_debug("dev->frm_cnt_3dnr%d\n", dev->frm_cnt_3dnr);
			dev->frm_cnt_3dnr--;
		}
	}

	if (isp_frame_dequeue(&buf_desc->zsl_queue, &frame)) {
		pr_err("fail to get offline_buf full_path frame\n");
		ret = -EPERM;
		goto err_exit;
	}

	pr_debug("[%d] cap frm cnt = %d, time = %ld.%06ld\n",
		iid, frame.fid, frame.t.tv_sec, frame.t.tv_usec);

	p_offline_frame = &dev->offline_frame[ISP_OFF_BUF_FULL];
	memcpy(p_offline_frame, &frame, sizeof(struct camera_frame));
	pr_debug("to be processed frame_id:%d\n", p_offline_frame->fid);

normal_exit_raw:
	spin_lock(&isp_mod_lock);
	if (fmcu_slice_capture_state == ISP_ST_START) {
		pr_info("capture already started cap_flag %d, skip this req\n",
			cap_flag);
		spin_unlock(&isp_mod_lock);
		goto normal_exit;
	}
	fmcu_slice_capture_state = ISP_ST_START;
	spin_unlock(&isp_mod_lock);
	complete(&dev->offline_full_thread_com);
	dev->wait_full_tx_done = WAIT_CLEAR;

normal_exit:
	pr_debug("normal exit, ret = %d\n", ret);
	return ISP_RTN_SUCCESS;

err_exit:
	dev->cap_on = 0;
	pr_err("fail to exit, ret = %d\n", ret);
	return ret;
}


static void do_shadow_clr(struct isp_pipe_dev *dev,
			unsigned int idx, int is_cap)
{
	struct isp_module *module = &dev->module_info;
	struct isp_path_desc *pre_path = &module->isp_path[ISP_SCL_PRE];
	struct isp_path_desc *vid_path = &module->isp_path[ISP_SCL_VID];
	struct isp_path_desc *cap_path = &module->isp_path[ISP_SCL_CAP];

	/*
	 * In CFG MODE, if both shadow_pre_cap done and shadow_vid_done
	 * are high-level, then cfg will start isp core configuration
	 * for next frame. If vid_path is closed or no used now, but clr
	 * the shadow_vid_done, then (shadow_pre_cap_done & shadow_vid_done)=0,
	 * and cfg will not poll isp shadow done, so cfg will not start
	 * the next frame configuration. To aviod this situation, we
	 * should do shadow clr before starting this path.
	 */
	pr_debug("idx 0x%x, pre_path%d, cap_path%d, vid_path%d, is_cap%d\n",
			dev->com_idx, pre_path->valid,
			cap_path->valid, vid_path->valid, is_cap);

	if (pre_path->valid || cap_path->valid) {
		ISP_REG_WR(idx,
			(ISP_STORE_PRE_CAP_BASE + ISP_STORE_SHADOW_CLR),
			0x1);

		if (cap_path->valid && dev->is_3dnr) {
			ISP_REG_WR(idx, (ISP_STORE_VID_BASE +
				ISP_STORE_SHADOW_CLR), 0x1);
			pr_debug("shadow clear for 3dnr path\n");
		}
	}

	if (vid_path->valid && !is_cap) {
		ISP_REG_WR(idx,
			(ISP_STORE_VID_BASE + ISP_STORE_SHADOW_CLR),
			0x1);
	}
}


static int prepare_fmcu_for_slice_cap(struct isp_pipe_dev *dev,
				      struct camera_frame *valid_frame,
				      unsigned int idx)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	struct slice_param_in slice_in = {0};

	if (!dev) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}
	if (!dev->fmcu_addr_vir || !dev->fmcu_addr_phy) {
		pr_err("fail to get fmcu_addr\n");
		return -EFAULT;
	}

	fmcu_slice = &dev->fmcu_slice;
	ret = isp_update_offline_path_param(dev, valid_frame, idx);
	if (ret) {
		pr_err("fail to update offline path param error!\n");
		ret = -ISP_RTN_PARA_ERR;
		goto exit;
	}

	if (!IS_ALIGNED(dev->fmcu_addr_phy, 8)) {
		pr_err("fail to aliged fmcu_addr_phy with 8!\n");
		ret = -ISP_RTN_PARA_ERR;
		goto exit;
	}
	ISP_HREG_WR(idx, ISP_FMCU_DDR_ADR, dev->fmcu_addr_phy);

	ret = get_slice_init_param(&slice_in, dev, valid_frame, idx);
	if (ret) {
		ret = ISP_RTN_SUCCESS;
		pr_err("fail to get slice init param need new frame\n");
		goto exit;
	}

	ret = isp_fmcu_slice_cfg(fmcu_slice->slice_handle,
		&slice_in, &fmcu_slice->fmcu_num);
	if (ret) {
		pr_err("fail to get fmcu slice cfg\n");
		goto exit;
	}

	ISP_HREG_MWR(idx, ISP_FMCU_CTRL, 0xFFFF0000,
		    fmcu_slice->fmcu_num << 16);

	/* should flush dcache before set fmcu addr to HW */
#ifdef CONFIG_64BIT
	__flush_dcache_area(dev->fmcu_addr_vir, fmcu_slice->fmcu_num << 3);
#else

	flush_kernel_vmap_range(dev->fmcu_addr_vir, fmcu_slice->fmcu_num << 3);
#endif

exit:
	isp_dbg_dump_fmcu_cmd_q(dev);
	return ret;
}

static void isp_cfg_subbock_reserved_buf(void *handle, unsigned int idx)
{
	struct isp_statis_buf *reserved_buf = NULL;
	struct isp_pipe_dev *dev = NULL;

	dev = (struct isp_pipe_dev *)handle;

	reserved_buf = &dev->statis_module_info.afl_buf_reserved;
	ISP_REG_WR(idx, ISP_ANTI_FLICKER_NEW_DDR_INIT_ADDR,
		   reserved_buf->phy_addr);
	ISP_REG_WR(idx, ISP_ANTI_FLICKER_NEW_REGION3,
		   (reserved_buf->phy_addr + reserved_buf->buf_size / 2));
	ISP_REG_MWR(idx, ISP_ANTI_FLICKER_NEW_CFG_READY, BIT_0, 1);

	reserved_buf = &dev->statis_module_info.binning_buf_reserved;
	ISP_REG_WR(idx, ISP_BINNING_MEM_ADDR, reserved_buf->phy_addr);
	ISP_REG_WR(idx, ISP_BINNING_CFG_READY, 1);

	reserved_buf = &dev->statis_module_info.aem_buf_reserved;
	ISP_REG_WR(idx, ISP_AEM_DDR_ADDR, reserved_buf->phy_addr);
	ISP_REG_MWR(idx, ISP_AEM_CFG_READY, BIT_0, 1);
}

static int sprd_isp_pipeline_proc_bin(void *handle)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module_info = NULL;
	struct isp_cctx_desc *cctx_desc = NULL;
	struct camera_frame *valid_frame = NULL;
	enum isp_id iid = ISP_ID_0;
	enum isp_scene_id sid = ISP_SCENE_PRE;
	unsigned int idx = 0;
	unsigned long flag;

	if (unlikely(!handle)) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	pr_debug("enter, idx:0x%x\n", idx);
	dev = (struct isp_pipe_dev *)handle;
	idx = dev->com_idx;
	ISP_SET_SID(idx, sid);
	iid = ISP_GET_IID(idx);
	module_info = &dev->module_info;
	if (unlikely(!module_info)) {
		pr_err("fail to get module_info is NULL iid %d\n", iid);
		return -EFAULT;
	}

	cctx_desc = module_info->cctx_desc;
	if (unlikely(ISP_GET_MID(idx) == ISP_CFG_MODE &&
		cctx_desc == NULL)) {
		pr_err("fail to get cctx_desc iid %d\n", iid);
		return -EFAULT;
	}

	spin_lock_irqsave(&dev->pre_lock, flag);

	if (module_info->off_desc.valid == 0) {
		pr_info("isp%d stopped, can not process this frame\n", iid);
		goto exit;
	}
	valid_frame = &dev->offline_frame[ISP_OFF_BUF_BIN];
	ISP_REG_WR(idx, ISP_FETCH_SLICE_Y_ADDR,
		   valid_frame->yaddr_vir);
	pr_debug("valid bin path frame, fetch_addr:0x%08x\n",
		 valid_frame->yaddr_vir);

	ISP_REG_WR(idx, ISP_STORE_BASE + ISP_STORE_PARAM, 1);

	do_shadow_clr(dev, idx, 0);
	isp_dbg_bypass_sblk(dev, idx);

	ISP_REG_MWR(idx, ISP_HIST_PARAM, BIT_0 | BIT_1, 0x2);
	ISP_REG_MWR(idx, ISP_HIST_CFG_READY, BIT_0, 1);
	pr_debug("update hist bypass\n");

	/*
	 * In CFG mode, init_hw will init isp CFG module and
	 * flush dcache for cfg cmd buf, if you config registers
	 * make sure the ISP_REG_xWR operation done before
	 * init_hw, while using ISP_HREG_xWR, ignore this limit.
	 * In AP mode, ignore this limit.
	 */
	if (likely(ISP_GET_MID(idx) == ISP_CFG_MODE)) {
		ret = cctx_desc->intf->init_hw(cctx_desc, iid, sid);
		if (ret) {
			pr_err("fail to init hw\n");
			goto exit;
		}
		pr_debug("init CFG hw done, iid %d, sid %d\n", iid, sid);
	}
	/*
	 * bit8 in ISP_INT_ALL_DONE_CTRL used to enable
	 * dispatch_done for all done ctrl, it means isp
	 * all done interrupt will not occur if dispatch
	 * not done.
	 */
	ISP_HREG_WR(idx, ISP_INT_ALL_DONE_CTRL + int_reg_base[iid][sid], 0xf8);

	if (likely(ISP_GET_MID(idx) == ISP_CFG_MODE)) {
		pr_debug("start isp%d in CFG mode, com_idx 0x%x\n",
			 iid, idx);
		isp_clk_resume(iid, ISP_CLK_P_P);
		cctx_desc->intf->buf_ready(cctx_desc, iid, sid);
		isp_dbg_reg_trace(dev, dev->com_idx);
	} else {
		pr_debug("start isp%d in AP mode, com_idx 0x%x\n",
			 iid, idx);
		isp_clk_resume(iid, ISP_CLK_P_S);
		ISP_REG_WR(idx, ISP_FETCH_START, 1);
	}

exit:
	spin_unlock_irqrestore(&dev->pre_lock, flag);
	pr_debug("exit\n");
	return ret;
}

static int sprd_isp_pipeline_proc_full(void *handle)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module_info = NULL;
	struct isp_cctx_desc *cctx_desc = NULL;
	struct camera_frame *valid_frame = NULL;
	enum isp_id iid = ISP_ID_0;
	enum isp_scene_id sid = ISP_SCENE_CAP;
	unsigned int idx = 0;
	unsigned long flag;
	struct offline_buf_desc *buf_desc = NULL;

	if (unlikely(!handle)) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)handle;
	idx = dev->com_idx;
	ISP_SET_SID(idx, sid);
	iid = ISP_GET_IID(idx);
	module_info = &dev->module_info;
	if (unlikely(!module_info)) {
		pr_err("fail to get ptr, iid %d module_info is NULL\n", iid);
		return -EFAULT;
	}

	cctx_desc = module_info->cctx_desc;
	if (unlikely(ISP_GET_MID(idx) == ISP_CFG_MODE &&
		cctx_desc == NULL)) {
		pr_err("fail to get ptr, iid %d cctx_desc is NULL\n", iid);
		return -EFAULT;
	}

	pr_debug("enter, cap_on:%d, idx: 0x%x\n", dev->cap_on, idx);

	spin_lock_irqsave(&dev->cap_lock, flag);

	if (module_info->off_desc.valid == 0) {
		pr_info("isp%d stopped, can not process this frame\n", iid);
		goto exit;
	}

	valid_frame = &dev->offline_frame[ISP_OFF_BUF_FULL];
	if (dev->cap_on == 0) {
		pr_err("capture stopped, skip this frame\n");
		fmcu_slice_capture_state = ISP_ST_STOP;
		dev->is_wait_fmcu = 0;
		complete(&dev->fmcu_com);
		/* release offline buffer */
		buf_desc = isp_offline_sel_buf(&module_info->off_desc,
			       ISP_OFF_BUF_FULL);
		if (isp_buf_queue_write(&buf_desc->tmp_buf_queue,
				valid_frame))
			pr_err("fail to retrieve off buf full_path\n");
		goto exit;
	}

	ret = prepare_fmcu_for_slice_cap(dev, valid_frame, idx);
	if (unlikely(ret)) {
		pr_err("fail to prepare_fmcu_for_slice_cap error, %d\n", ret);
		goto exit;
	}
	isp_cfg_subbock_reserved_buf(dev, idx);
#if 0 /* TODO: capture fetch delay control */
	sprd_isp_cap_hblank_cfg(handle);
#endif

	/* set h_blank value:0x800 */
	ISP_REG_MWR(idx, ISP_FETCH_LINE_DLY_CTRL, 0xffff,
			0x800);
	ISP_REG_WR(idx, ISP_STORE_BASE + ISP_STORE_PARAM, 1);

	do_shadow_clr(dev, idx, 1);
	isp_dbg_bypass_sblk(dev, idx);
	ISP_REG_MWR(idx, ISP_HIST_PARAM, BIT_0 | BIT_1, 0x2);
	ISP_REG_MWR(idx, ISP_HIST_CFG_READY, BIT_0, 1);
	pr_debug("update hist bypass\n");

	/*
	 * In CFG mode, init_hw will init isp CFG module and
	 * flush dcache for cfg cmd buf, if you config registers
	 * make sure the ISP_REG_xWR operation done before
	 * init_hw, while using ISP_HREG_xWR, ignore this limit.
	 * In AP mode, ignore this limit.
	 */
	if (likely(ISP_GET_MID(idx) == ISP_CFG_MODE)) {
		ret = cctx_desc->intf->init_hw(cctx_desc, iid, sid);
		if (unlikely(ret)) {
			pr_err("fail to init CFG_MODE hw!\n");
			goto exit;
		}
		pr_debug("init CFG hw done, iid %d, sid %d\n", iid, sid);
	}

	/*
	 * bit8 in ISP_INT_ALL_DONE_CTRL used to enable
	 * dispatch_done for all done ctrl, it means isp
	 * all done interrupt will not occur if dispatch
	 * not done.
	 */
	ISP_HREG_WR(idx, ISP_INT_ALL_DONE_CTRL + int_reg_base[iid][sid], 0xf8);

	pr_debug("start isp%d using fmcu, com_idx 0x%x\n", iid, idx);
	isp_clk_resume(iid, ISP_CLK_P_C);
	ISP_HREG_WR(idx, ISP_FMCU_START, 1);

exit:
	spin_unlock_irqrestore(&dev->cap_lock, flag);
	pr_debug("exit\n");

	return ret;
}

int sprd_isp_force_stop_pipeline(void *handle)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	unsigned long flag;

	if (!handle) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)handle;

	dev->frm_cnt = 0;
	dev->cap_on = 0;
	dev->wait_full_tx_done = WAIT_CLEAR;

	spin_lock_irqsave(&isp_mod_lock, flag);
	if (dev->is_wait_fmcu == 1) {
		complete(&dev->fmcu_com);
		dev->is_wait_fmcu = 0;
	}
	fmcu_slice_capture_state = ISP_ST_STOP;
	spin_unlock_irqrestore(&isp_mod_lock, flag);

	if (dev->is_3dnr) {
		pr_debug("disabling 3dnr\n");
		dev->is_3dnr = 0;
		dev->frm_cnt_3dnr = 0;
	}

	return ret;
}


/*
 * Note: this function use msleep,
 * can not be called in interrupt context.
 */
int sprd_isp_stop_pipeline(void *handle)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;

	if (!handle) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)handle;
	dev->cap_on = 0;
	if (dev->cap_flag != DCAM_CAPTURE_START_WITH_TIMESTAMP)
		wait_for_pipeline_proc_stop(handle);

	ret = sprd_isp_force_stop_pipeline(handle);

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

static int isp_calc_sc_size(struct isp_path_desc *path)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	unsigned int tmp_dstsize = 0;
	unsigned int align_size = 0;
	struct isp_trim_info *in_trim;
	struct camera_size *in_size;
	struct camera_size *out_size;
	unsigned int d_max = CAMERA_SC_COEFF_DOWN_MAX;
	unsigned int u_max = CAMERA_SC_COEFF_UP_MAX;
	unsigned int f_max = CAMERA_PATH_DECI_FAC_MAX;

	if (path == NULL) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	in_trim = &path->trim0_info;
	in_size = &path->src;
	out_size = &path->dst;
	pr_debug("in_trim_size_x:%d, in_trim_size_y:%d, out_size_w:%d,out_size_h:%d\n",
		in_trim->size_x, in_trim->size_y, out_size->w, out_size->h);
	pr_debug("in_size_x:%d, in_size_y:%d\n", in_size->w, in_size->h);

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
	enum isp_id iid = 0;
	enum isp_scl_id scl_path_id;
	struct isp_pipe_dev *dev = NULL;

	if (!module || !path || !coeff) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	dev = container_of(module, struct isp_pipe_dev, module_info);
	iid = ISP_GET_IID(dev->com_idx);

	if (module->scl_array == NULL) {
		pr_err("fail to get ptr\n");
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
		pr_err("fail to call dcam_gen_scale_coeff\n");
		return -DCAM_RTN_PATH_GEN_COEFF_ERR;
	}

	path->scaler_info.scaler_y_ver_tap = y_tap;
	path->scaler_info.scaler_uv_ver_tap = uv_tap;

	pr_debug("Scaler y_tap %d, uv_tap %d\n", y_tap, uv_tap);

	return ISP_RTN_SUCCESS;
}

int isp_path_scaler(struct isp_module *module,
		    enum isp_path_index path_index,
		    struct isp_path_desc *path,
		    struct isp_sc_coeff *coeff)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	unsigned long cfg_reg = 0;
	struct isp_pipe_dev *dev = NULL;
	enum isp_scene_id sid = ISP_SCENE_PRE;
	unsigned int idx = 0;

	if (!module || !path || !coeff) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	dev = container_of(module, struct isp_pipe_dev, module_info);
	idx = dev->com_idx;

	if (module->scl_array == NULL) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	if (path_index == ISP_PATH_IDX_PRE) {
		sid = ISP_SCENE_PRE;
		cfg_reg = ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_CFG;
	} else if (path_index == ISP_PATH_IDX_VID) {
		if (module->isp_path[ISP_SCL_CAP].valid && dev->is_3dnr)
			sid = ISP_SCENE_CAP;
		else
			sid = ISP_SCENE_PRE;
		cfg_reg = ISP_SCALER_VID_BASE + ISP_SCALER_CFG;
	} else if (path_index == ISP_PATH_IDX_CAP) {
		sid = ISP_SCENE_CAP;
		cfg_reg = ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_CFG;
	} else
		return -EFAULT;

	ISP_SET_SID(idx, sid);

	if (path->output_format == DCAM_RAWRGB ||
	    path->output_format == DCAM_JPEG) {
		pr_debug("out format is %d, no need scaler\n",
			   path->output_format);
		return ISP_RTN_SUCCESS;
	}

	rtn = isp_calc_sc_size(path);
	if (rtn)
		return rtn;

	if (path->scaler_info.scaler_factor_in ==
	    path->scaler_info.scaler_factor_out &&
	    path->scaler_info.scaler_ver_factor_in ==
	    path->scaler_info.scaler_ver_factor_out &&
	    path->output_format == DCAM_YUV422) {
		path->scaler_bypass = 1;
		ISP_REG_MWR(idx, cfg_reg, BIT_20, 1 << 20);
	} else {
		path->scaler_bypass = 0;
		ISP_REG_MWR(idx, cfg_reg, BIT_20, 0 << 20);
		rtn = isp_set_sc_coeff(module, path_index, path, coeff);
	}

	return rtn;
}

static int isp_raw_cap_proc(struct isp_pipe_dev *dev,
	struct isp_raw_proc_info *raw_cap)
{
	enum isp_drv_rtn ret = ISP_RTN_SUCCESS;
	struct isp_module *module = NULL;
	struct isp_path_desc *cap = NULL;
	struct isp_offline_desc *off_desc = NULL;
	struct camera_addr frm_addr = {0};
	struct camera_frame *frame = NULL;
	unsigned int cap_flag;
	int isp_fetch_fmt = ISP_FETCH_CSI2_RAW_10;

	if (!dev || !raw_cap) {
		ret = -EFAULT;
		pr_err("isp dev handle is NULL\n");
		goto exit;
	}

	frame = vzalloc(sizeof(struct camera_frame) * 2);
	if (frame == NULL) {
		ret = -ENOMEM;
		goto exit;
	}

	module = &dev->module_info;
	cap = &module->isp_path[ISP_SCL_CAP];
	off_desc = &module->off_desc;

	ISP_SET_SID(dev->com_idx, ISP_SCENE_CAP);

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
	off_desc->buf_desc_full.output_format = DCAM_YUV420;
	dev->is_raw_capture = 1;

	frm_addr.yaddr = raw_cap->img_offset.chn0;
	frm_addr.uaddr = raw_cap->img_offset.chn1;
	frm_addr.vaddr = raw_cap->img_offset.chn2;
	frm_addr.yaddr_vir = raw_cap->img_vir.chn0;
	frm_addr.uaddr_vir = raw_cap->img_vir.chn1;
	frm_addr.vaddr_vir = raw_cap->img_vir.chn2;
	frm_addr.mfd_y = raw_cap->img_fd;
	pr_info("start raw path config\n");
	ret = set_isp_path_cfg((void *)dev, ISP_PATH_IDX_CAP,
		ISP_PATH_OUTPUT_ADDR, &frm_addr);
	if (ret)
		return ret;

	ret = set_isp_path_cfg((void *)dev, ISP_PATH_IDX_CAP,
		ISP_PATH_INPUT_FORMAT, &isp_fetch_fmt);
	if (ret)
		return ret;

	pr_info("start isp path\n");
	ret = sprd_isp_start((void *)dev, frame);
	if (ret) {
		pr_err("fail to start isp path\n");
		return ret;
	}

	cap_flag = DCAM_CAPTURE_START;
	complete(&dev->fmcu_com);
	ret = sprd_isp_start_pipeline_full((void *)dev, cap_flag);
	if (ret) {
		pr_err("fail to start isp pipeline error, cap_flag = %d\n",
			       cap_flag);
		return ret;
	}

	pr_info("end isp path\n");

exit:
	if (frame)
		vfree(frame);
	return ret;
}

static int ispdrv_mask_3a_req(uint32_t idx)
{
	sprd_isp_glb_reg_awr(idx,
			     ISP_P0_INT_BASE + ISP_INT_EN2,
			     ~ISP_AF_INT_MASK,
			     ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_awr(idx,
			     ISP_P1_INT_BASE + ISP_INT_EN2,
			     ~ISP_AF_INT_MASK,
			     ISP_INIT_MASK_REG);

	sprd_isp_glb_reg_awr(idx,
			     ISP_P0_INT_BASE + ISP_INT_EN0,
			     ~ISP_3A_INT0_MASK,
			     ISP_INIT_MASK_REG);
	sprd_isp_glb_reg_awr(idx,
			     ISP_P1_INT_BASE + ISP_INT_EN0,
			     ~ISP_3A_INT0_MASK,
			     ISP_INIT_MASK_REG);

	return 0;
}

int sprd_isp_k_ioctl(void *isp_pipe_dev_handle,
	unsigned int cmd, unsigned long param)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct sprd_isp_irq irq_param;
	struct isp_node node = {0};
#if 0
	struct isp_statis_buf_input parm_inptr;
#endif
	struct isp_k_block *isp_k_param = NULL;
	struct isp_raw_proc_info raw_cap;
	enum isp_id iid = ISP_ID_0;
	unsigned int idx = 0;

	if (!isp_pipe_dev_handle) {
		ret = -EFAULT;
		pr_err("isp dev handle is NULL\n");
		goto exit;
	}
	dev = (struct isp_pipe_dev *)isp_pipe_dev_handle;
	idx = dev->com_idx;
	iid = ISP_GET_IID(idx);

	pr_debug("cmd: 0x%x, %d\n", cmd, _IOC_NR(cmd));
	isp_k_param = &dev->isp_k_param;

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
				pr_err("fail to copy_to_user, ret = %d\n",
					(uint32_t)ret);
			ret = -ERESTARTSYS;
			return ret;
		}

		ret = isp_queue_read(&dev->queue, &node);
		if (ret != 0) {
			ret = -EFAULT;
			pr_err("fail to read isp queue, ret = 0x%x\n",
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
			pr_err("fail to copy_to_user, ret = 0x%x",
				(uint32_t)ret);
		}
		break;
	}
	case SPRD_ISP_IO_READ:
		break;
	case SPRD_ISP_IO_RAW_CAP:
		mutex_lock(&dev->isp_mutex);
		pr_info("start isp_raw_cap_proc %p\n", dev);
		memset((void *)&raw_cap, 0x0, sizeof(struct isp_raw_proc_info));
		ret = copy_from_user(&raw_cap,
				     (const void __user *)param,
				     sizeof(struct isp_raw_proc_info));
		if (ret != 0) {
			pr_err("fail to copy_from_user\n");
			mutex_unlock(&dev->isp_mutex);
			goto exit;
		}
		ret = isp_raw_cap_proc(dev, &raw_cap);
		mutex_unlock(&dev->isp_mutex);
		break;
	case SPRD_ISP_IO_WRITE:
		break;
	case SPRD_ISP_IO_RST:
		isp_k_param->lsc_2d_weight_en = 0;
		break;
	case SPRD_ISP_IO_STOP:
		break;
	case SPRD_ISP_IO_INT:
		break;
	case SPRD_ISP_IO_CFG_PARAM:
		ret = isp_cfg_param((void *)param, isp_k_param, dev);
		break;
	case SPRD_ISP_IO_SET_STATIS_BUF:
#if 0
		mutex_lock(&dev->isp_mutex);
		ret = copy_from_user(&parm_inptr,
				     (const void __user *)param,
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
#endif
		break;
	case SPRD_ISP_IO_CAPABILITY:
		ret = isp_k_capability((void __user *)param);
		break;
	case SPRD_ISP_REG_READ:
		break;
	case SPRD_ISP_IO_MASK_3A:
		ret = ispdrv_mask_3a_req(idx);
		break;
	default:
		pr_err("fail to get cmd,it is unsupported, cmd = %x\n",
			(int32_t)cmd);
		return -EFAULT;
	}

exit:
	return ret;
}

static void isp_scl_size_print(enum isp_id iid, struct isp_path_desc *path,
			       unsigned int addr)
{
	if (!path) {
		pr_err("Input path ptr is NULL\n");
		return;
	}

	pr_debug("isp%d scl base addr %x\n", iid, addr);
	pr_debug("set_scl src %d %d dst %d %d trim0 %d %d %d %d trim1 %d %d %d %d\n",
		path->src.w, path->src.h,
		path->dst.w, path->dst.h,
		path->trim0_info.start_x,
		path->trim0_info.start_y,
		path->trim0_info.size_x,
		path->trim0_info.size_y,
		path->trim1_info.start_x,
		path->trim1_info.start_y,
		path->trim1_info.size_x,
		path->trim1_info.size_y);
}

int sprd_isp_update_zoom_param(void *isp_handle,
			       enum isp_path_index path_index,
			       struct camera_size *in_size,
			       struct camera_rect *in_rect,
			       struct camera_size *out_size)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_path_desc *pre = NULL;
	struct isp_path_desc *vid = NULL;
	struct isp_path_desc *cap = NULL;
	struct isp_offline_desc *off_desc = NULL;
	struct isp_sc_coeff *pre_coeff = NULL;
	struct isp_sc_coeff *vid_coeff = NULL;
	struct isp_sc_coeff *cap_coeff = NULL;
	struct isp_offline_desc *scl_off_desc = NULL;
	struct isp_sc_array *scl_array = NULL;
	enum isp_id iid = 0;

	if (isp_handle == NULL) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	iid = ISP_GET_IID(dev->com_idx);
	if (unlikely(!module)) {
		pr_err("error, iid %d module is NULL\n", iid);
		return -EFAULT;
	}

	scl_array = module->scl_array;
	if (scl_array == NULL) {
		pr_err("iid%d scl_array is NULL\n", iid);
		return -EFAULT;
	}
	pr_debug("ISP%d: update path\n", iid);

	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];
	cap = &module->isp_path[ISP_SCL_CAP];
	off_desc = &module->off_desc;

	rtn = isp_coeff_get_new_node(&scl_array->pre_queue, &pre_coeff, 0);
	if (rtn) {
		pr_err("fail to get isp coeff new node\n");
		return -rtn;
	}
	rtn = isp_coeff_get_new_node(&scl_array->vid_queue, &vid_coeff, 0);
	if (rtn) {
		pr_err("fail to get isp coeff new node\n");
		return -rtn;
	}
	rtn = isp_coeff_get_new_node(&scl_array->cap_queue, &cap_coeff, 0);
	if (rtn) {
		pr_err("fail to get isp coeff new node\n");
		return -rtn;
	}
	/* This time has 2 block memery, If need 4, please change */
	scl_off_desc = &scl_array->scl_off_desc;

	if (ISP_PATH_IDX_PRE & path_index && pre->valid) {
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

		if (!vid->valid && !cap->valid) {
			rtn = isp_start_pre_proc(&pre_coeff->path,
				&vid_coeff->path,
				&cap_coeff->path,
				scl_off_desc);
			if (rtn) {
				pr_err("isp start pre proc failed\n");
				return -rtn;
			}
			isp_scl_size_print(iid, &pre_coeff->path,
					   ISP_SCALER_PRE_CAP_BASE);

			pr_debug("ISP%d: to update path1\n", iid);
			rtn = isp_path_scaler(module, ISP_PATH_IDX_PRE,
					      &pre_coeff->path,
					      pre_coeff);
			if (rtn) {
				pr_err("err, code %d\n", rtn);
				return -(rtn);
			}
			rtn = isp_coeff_get_new_node(&scl_array->pre_queue,
						     &pre_coeff, 1);
			if (rtn) {
				pr_err("fail to get isp coeff new node\n");
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

		if (!cap->valid) {
			rtn = isp_start_pre_proc(&pre_coeff->path,
				&vid_coeff->path,
				&cap_coeff->path,
				scl_off_desc);
			if (rtn) {
				pr_err("fail to start isp pre proc\n");
				return -rtn;
			}

			pr_debug("ISP%d: to update path vid\n", iid);
			rtn = isp_path_scaler(module, ISP_PATH_IDX_VID,
					      &vid_coeff->path, vid_coeff);
			if (rtn) {
				pr_err("fail to scaler pre path rtn %d\n", rtn);
				return -(rtn);
			}
			rtn = isp_coeff_get_new_node(&scl_array->vid_queue,
						     &vid_coeff, 1);
			if (rtn) {
				pr_err("fail to get isp coeff new node\n");
				return -rtn;
			}
			if (pre->valid) {
				pr_debug("ISP%d: to update path1\n", iid);
				rtn = isp_path_scaler(module, ISP_PATH_IDX_PRE,
						      &pre_coeff->path,
						      pre_coeff);
				if (rtn) {
					pr_err("fail to scaler pre path\n");
					return -(rtn);
				}
				rtn = isp_coeff_get_new_node(
							&scl_array->pre_queue,
							&pre_coeff, 1);
				if (rtn) {
					pr_err("fail to get coeff new node\n");
					return -rtn;
				}
				isp_scl_size_print(iid, &pre_coeff->path,
						   ISP_SCALER_PRE_CAP_BASE);
				isp_scl_size_print(iid, &vid_coeff->path,
						   ISP_SCALER_VID_BASE);
			}
		}
	}
	if (ISP_PATH_IDX_CAP & path_index && cap->valid) {
		/* pre,vid can zoom when working, but cap can't
		 * so set all size to cap(module->isp_path[ISP_SCL_CAP])
		 * not coeff path
		 */
		cap->in_size.w = in_size->w;
		cap->in_size.h = in_size->h;
		cap->in_rect.x = in_rect->x;
		cap->in_rect.y = in_rect->y;
		cap->in_rect.w = in_rect->w;
		cap->in_rect.h = in_rect->h;
		cap->out_size.w = out_size->w;
		cap->out_size.h = out_size->h;

		rtn = isp_start_pre_proc(&pre_coeff->path,
				&vid_coeff->path,
				cap,
				scl_off_desc);
		if (rtn) {
			pr_err("fail to start isp pre proc\n");
			return -rtn;
		}

		memcpy(&cap_coeff->path, cap,
		       sizeof(struct isp_path_desc));
		memcpy(scl_off_desc, off_desc,
		       sizeof(struct isp_offline_desc));
		pr_debug("cap in %d %d rect %d %d %d %d out %d %d\n",
			 cap_coeff->path.in_size.w,
			 cap_coeff->path.in_size.h,
			 cap_coeff->path.in_rect.x,
			 cap_coeff->path.in_rect.y,
			 cap_coeff->path.in_rect.w,
			 cap_coeff->path.in_rect.h,
			 cap_coeff->path.out_size.w,
			 cap_coeff->path.out_size.h);

		pr_debug("ISP%d: to update path3\n", iid);
		rtn = isp_path_scaler(module, ISP_PATH_IDX_CAP,
				      &cap_coeff->path, cap_coeff);
		if (rtn) {
			pr_err("fail to scale cap path,rtn%d\n", rtn);
			return -(rtn);
		}

		if (vid->valid || dev->is_3dnr) {
			pr_debug("ISP%d: to update path vid\n", iid);
			rtn = isp_path_scaler(module, ISP_PATH_IDX_VID,
					      &vid_coeff->path, vid_coeff);
			if (rtn) {
				pr_err("fail to scale vid path rtn%d\n", rtn);
				return -(rtn);
			}
			rtn = isp_coeff_get_new_node(&scl_array->vid_queue,
						     &vid_coeff, 1);
			if (rtn) {
				pr_err("isp coeff get new node\n");
				return -rtn;
			}
		}

		if (pre->valid) {
			pr_debug("ISP%d: to update path1\n", iid);
			rtn = isp_path_scaler(module, ISP_PATH_IDX_PRE,
					      &pre_coeff->path, pre_coeff);
			if (rtn) {
				pr_err("fail to scale pre path rtn%d\n", rtn);
				return -(rtn);
			}
			rtn = isp_coeff_get_new_node(&scl_array->pre_queue,
						     &pre_coeff, 1);
			if (rtn) {
				pr_err("fail to get new coeff node\n");
				return -rtn;
			}
		}
		if (pre->valid) {
			isp_scl_size_print(iid, &pre_coeff->path,
					   ISP_SCALER_PRE_CAP_BASE);
		}
		if (vid->valid)
			isp_scl_size_print(iid, &vid_coeff->path,
					   ISP_SCALER_VID_BASE);
		if (cap->valid)
			isp_scl_size_print(iid, &cap_coeff->path,
					   ISP_SCALER_PRE_CAP_BASE);
	}

	pr_debug("update path done\n");

	return -rtn;
}


static void isp_common_cfg(unsigned int idx)
{

	/* isp outstanding setting value:4 */
	ISP_HREG_MWR(idx, ISP_AXI_ITI2AXIM_CTRL,
			ISP_AXI_ITI2AXIM_ROSTD_MASK,
			1 << 10);
	ISP_HREG_WR(idx, ISP_AXI_ARBITER_WQOS, 0x04a8);
	ISP_HREG_WR(idx, ISP_AXI_ARBITER_RQOS, 0x00a8);
	/* out closed */
	ISP_HREG_MWR(idx,
		ISP_COMMON_SCL_PATH_SEL, (BIT_6 | BIT_7), 3 << 6);
}

static int isp_set_statis_buf(struct isp_pipe_dev *dev)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_statis_module *statis_module = NULL;

	statis_module = &dev->statis_module_info;

	rtn = isp_set_next_statis_buf(dev->com_idx,
				statis_module, ISP_AFL_BLOCK);
	if (rtn) {
		pr_err("err, code %d\n", rtn);
		return -(rtn);
	}
	rtn = isp_set_next_statis_buf(dev->com_idx,
				statis_module, ISP_AFM_BLOCK);
	if (rtn) {
		pr_err("err, code %d\n", rtn);
		return -(rtn);
	}
	rtn = isp_set_next_statis_buf(dev->com_idx,
				statis_module, ISP_BINNING_BLOCK);
	if (rtn) {
		pr_err("err, code %d\n", rtn);
		return -(rtn);
	}
	rtn = isp_set_next_statis_buf(dev->com_idx,
				statis_module, ISP_HIST_BLOCK);
	if (rtn) {
		pr_err("err, code %d\n", rtn);
		return -(rtn);
	}

	return rtn;
}

static int sprd_isp_start_path(void *isp_handle,
	enum isp_path_index path_index,
	struct camera_frame *frame)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_path_desc *pre = NULL;
	struct isp_path_desc *vid = NULL;
	struct isp_path_desc *cap = NULL;
	struct isp_offline_desc *off_desc = NULL;
	struct isp_sc_array *scl_array = NULL;
	enum isp_id iid = 0;
	bool pre_to_run = false, cap_to_run = false, vid_to_run = false;
	uint8_t off_type = ISP_OFF_BUF_BIN;
	unsigned int idx = 0;
	unsigned long flag;

	if (!isp_handle) {
		pr_err("fail to get ptr\n");
		return -ISP_RTN_PARA_ERR;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	idx = dev->com_idx;
	dev->frm_cnt = 0;
	iid = ISP_GET_IID(idx);
	module = &dev->module_info;
	if (unlikely(!module)) {
		pr_err("error, iid %d module is NULL\n", iid);
		return -EFAULT;
	}

	pre = &module->isp_path[ISP_SCL_PRE];
	vid = &module->isp_path[ISP_SCL_VID];
	cap = &module->isp_path[ISP_SCL_CAP];
	off_desc = &module->off_desc;
	scl_array = module->scl_array;

	if (scl_array == NULL) {
		pr_err("Null pointer of scl_array iid%d\n", iid);
		return -EFAULT;
	}
	/* init for isp clk pause and resume */
	spin_lock_irqsave(&isp_clk_pr, flag);
	isp_clk_flag = 0;
	spin_unlock_irqrestore(&isp_clk_pr, flag);
	rtn = isp_start_pre_proc(pre, vid, cap, off_desc);
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

	pre_to_run = (ISP_PATH_IDX_PRE & path_index) &&
		module->isp_path[ISP_SCL_PRE].valid;
	cap_to_run = (ISP_PATH_IDX_CAP & path_index) &&
		module->isp_path[ISP_SCL_CAP].valid;
	vid_to_run = (ISP_PATH_IDX_VID & path_index) &&
		module->isp_path[ISP_SCL_VID].valid;

	pr_info("idx%d, path%d, pre_to_run %d, cap_to_run %d, vid_to_run%d\n",
		idx, path_index, pre_to_run, cap_to_run, vid_to_run);
	if (module->off_desc.valid) {
		if (pre_to_run || vid_to_run) {
			off_type = ISP_OFF_BUF_BIN;
			rtn = isp_offline_get_buf(&module->off_desc, off_type);
			if (rtn) {
				pr_err("fail to get binning buf\n");
				return -(rtn);
			}

			rtn = isp_offline_buf_iommu_map(&module->off_desc,
							off_type);
			if (rtn) {
				pr_err("fail to map binning buf\n");
				return -(rtn);
			}

			rtn = isp_offline_set_next_frm(module, off_type, frame);
			if (rtn) {
				pr_err("code %d\n", rtn);
				return -(rtn);
			}

			dev->pre_state = ISP_ST_STOP;
			dev->bin_path_miss_cnt = 0;

			pr_debug("offline_buf for bin_path is ready\n");
		}
		if (cap_to_run && !dev->is_raw_capture) {
			off_type = ISP_OFF_BUF_FULL;
			rtn = isp_offline_get_buf(&module->off_desc, off_type);
			if (rtn) {
				pr_err("fail to get full buf\n");
				return -(rtn);
			}

			rtn = isp_offline_buf_iommu_map(&module->off_desc,
							off_type);
			if (rtn) {
				pr_err("fail to map full buf\n");
				return -(rtn);
			}

			rtn = isp_offline_set_next_frm(module, off_type,
				frame+1);
			if (rtn) {
				pr_err("fail to set next frm rtn %d\n", rtn);
				return -(rtn);
			}

			pr_debug("offline_buf for full_path is ready\n");
		}

		module->off_desc.status = ISP_ST_START;
		module->off_desc.shadow_done_cnt = 0;
	}

	if (pre_to_run) {
		rtn = isp_path_scaler(module, ISP_PATH_IDX_PRE, pre,
			&scl_array->coeff[ISP_SCL_PRE]);
		if (rtn) {
			pr_err("err, code %d\n", rtn);
			return -(rtn);
		}
		isp_path_set(module, pre, ISP_PATH_IDX_PRE);
		rtn = isp_path_set_next_frm(module, ISP_PATH_IDX_PRE, NULL);
		if (rtn) {
			pr_err("err, code %d\n", rtn);
			return -(rtn);
		}

		module->isp_path[ISP_SCL_PRE].status = ISP_ST_START;
		module->isp_path[ISP_SCL_PRE].need_wait = 0;
		module->isp_path[ISP_SCL_PRE].shadow_done_cnt = 0;
	}

	if (vid_to_run) {
		rtn = isp_path_scaler(module, ISP_PATH_IDX_VID, vid,
			&scl_array->coeff[ISP_SCL_VID]);
		if (rtn) {
			pr_err("err, code %d\n", rtn);
			return -(rtn);
		}
		isp_path_set(module, vid, ISP_PATH_IDX_VID);

		if (dev->fmcu_slw.slw_flags == ISP_SLW_VIDEO) {
			rtn = set_fmcu_slw_cfg(isp_handle);
			if (rtn) {
				pr_err("err, code %d\n", rtn);
				return -(rtn);
			}
			rtn = isp_fmcu_slw_start(ISP_SCL_VID, isp_handle);
			if (rtn) {
				pr_err("err, code %d\n", rtn);
				return -(rtn);
			}
		} else {
			rtn = isp_path_set_next_frm(module,
						    ISP_PATH_IDX_VID, NULL);
			if (rtn) {
				pr_err("err, code %d\n", rtn);
				return -(rtn);
			}
		}

		module->isp_path[ISP_SCL_VID].status = ISP_ST_START;
		module->isp_path[ISP_SCL_VID].need_wait = 0;
		module->isp_path[ISP_SCL_VID].shadow_done_cnt = 0;
	}

	if (cap_to_run && fmcu_slice_capture_state == ISP_ST_START) {
		rtn = isp_path_scaler(module, ISP_PATH_IDX_CAP, cap,
			&scl_array->coeff[ISP_SCL_CAP]);
		if (rtn) {
			pr_err("err, code %d\n", rtn);
			return -(rtn);
		}
		isp_path_set(module, cap, ISP_PATH_IDX_CAP);

		module->isp_path[ISP_SCL_CAP].status = ISP_ST_START;
		module->isp_path[ISP_SCL_CAP].need_wait = 0;
		module->isp_path[ISP_SCL_CAP].shadow_done_cnt = 0;
	}

	if (pre_to_run || cap_to_run) {
		/* yuv pipeline */
		ISP_REG_MWR(idx, ISP_STORE_PRE_CAP_BASE + ISP_STORE_PARAM,
			BIT_0, 0 << 0);
		ISP_HREG_MWR(idx,
			ISP_COMMON_SCL_PATH_SEL, (BIT_0 | BIT_1), 0);

		if (cap_to_run && dev->is_3dnr) {
			ISP_REG_MWR(idx, ISP_STORE_VID_BASE + ISP_STORE_PARAM,
				BIT_0, 0 << 0);
			ISP_HREG_MWR(idx,
				ISP_COMMON_SCL_PATH_SEL,
				(BIT_2 | BIT_3), 0 << 2);
			pr_info("video path opened for 3dnr\n");
		}
	} else {
		ISP_REG_MWR(idx, ISP_STORE_PRE_CAP_BASE + ISP_STORE_PARAM,
			BIT_0, 1 << 0);
		ISP_HREG_MWR(idx,
			ISP_COMMON_SCL_PATH_SEL, (BIT_0 | BIT_1), 3 << 0);
	}

	if (vid_to_run) {
		 /* yuv pipeline */
		ISP_REG_MWR(idx, ISP_STORE_VID_BASE + ISP_STORE_PARAM,
			BIT_0, 0 << 0);
		ISP_HREG_MWR(idx,
			ISP_COMMON_SCL_PATH_SEL, (BIT_2 | BIT_3), 0 << 2);

		/*
		 * TODO: need to check the proper place
		 * for fixing scaler frame skip setting
		 */
#if 0
		ISP_REG_MWR(idx,
			ISP_SCALER_VID_BASE + ISP_SCALER_CFG,
			ISP_SCALER_CFG_MASK, 0x0);
#endif
	} else if (!dev->is_3dnr) {
		/* video closed */
		ISP_REG_MWR(idx, ISP_STORE_VID_BASE + ISP_STORE_PARAM,
			BIT_0, 1 << 0);
		ISP_HREG_MWR(idx,
			ISP_COMMON_SCL_PATH_SEL, (BIT_2 | BIT_3), 3 << 2);
	}

	if (path_index != ISP_PATH_IDX_ALL) {
		if (ISP_PATH_IDX_PRE & path_index)
			isp_wait_path_done(module, ISP_SCL_PRE, NULL);
		else if (ISP_PATH_IDX_VID & path_index)
			isp_wait_path_done(module, ISP_SCL_VID, NULL);
		else if (ISP_PATH_IDX_CAP & path_index)
			isp_wait_path_done(module, ISP_SCL_CAP, NULL);
	}

	isp_common_cfg(idx);
	isp_irq_ctrl(dev, true);

	return rtn;
}

int sprd_isp_start(void *isp_handle, struct camera_frame *frame)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;

	if (!isp_handle) {
		pr_err("fail to get ptr\n");
		return -ISP_RTN_PARA_ERR;
	}

	rtn = sprd_isp_start_path(isp_handle, ISP_PATH_IDX_ALL, frame);

	return rtn;
}

int set_isp_path_cfg(void *isp_handle, enum isp_path_index path_index,
	enum isp_cfg_id id, void *param)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_path_desc *path = NULL;
	struct isp_offline_desc *off_desc = NULL;
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
	enum isp_id iid  = 0;
	uint8_t off_type;
	struct offline_buf_desc *buf_desc = NULL;

	if (!isp_handle || !param) {
		pr_err("parm is null\n");
		return -ISP_RTN_PARA_ERR;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	iid = ISP_GET_IID(dev->com_idx);
	if (ISP_PATH_IDX_PRE & path_index) {
		path_id = ISP_SCL_PRE;
	} else if (ISP_PATH_IDX_VID & path_index) {
		path_id = ISP_SCL_VID;
	} else if (ISP_PATH_IDX_CAP & path_index) {
		path_id = ISP_SCL_CAP;
	} else {
		pr_err("error\n");
		return -ISP_RTN_PARA_ERR;
	}

	module = &dev->module_info;
	if (unlikely(!module)) {
		pr_err("error, iid %d module is NULL\n", iid);
		return -EFAULT;
	}

	path = &module->isp_path[path_id];
	off_desc = &module->off_desc;

	switch (id) {
	case ISP_PATH_INPUT_SIZE:
		size = (struct camera_size *)param;

		pr_info("ISP path%d input size:{%d %d}\n", path_id,
			size->w, size->h);
		if (size->w > ISP_PATH_FRAME_WIDTH_MAX ||
		    size->h > ISP_PATH_FRAME_HEIGHT_MAX) {
			rtn = ISP_RTN_PATH_IN_SIZE_ERR;
		} else {
			path->in_size.w = size->w;
			path->in_size.h = size->h;
		}
		break;

	case ISP_PATH_INPUT_RECT:
		rect = (struct camera_rect *)param;

		pr_info("ISP path%d trim size:{%d %d} {%d %d}\n", path_id,
			rect->x, rect->y, rect->w, rect->h);
		if (rect->x > ISP_PATH_FRAME_WIDTH_MAX ||
		    rect->y > ISP_PATH_FRAME_HEIGHT_MAX ||
		    rect->w > ISP_PATH_FRAME_WIDTH_MAX ||
		    rect->h > ISP_PATH_FRAME_HEIGHT_MAX) {
			rtn = ISP_RTN_PATH_TRIM_SIZE_ERR;
		} else {
			path->in_rect.x = rect->x;
			path->in_rect.y = rect->y;
			path->in_rect.w = rect->w;
			path->in_rect.h = rect->h;
		}
		break;

	case ISP_PATH_INPUT_FORMAT:
		format = *(unsigned int *)param;
		path->input_format = format;
		pr_info("ISP path%d input format:%d\n", path_id, format);
		break;

	case ISP_PATH_OUTPUT_SIZE:
		size = (struct camera_size *)param;
		pr_info("ISP path%d output size:{%d %d}\n", path_id,
			size->w, size->h);
		if (size->w > ISP_PATH_FRAME_WIDTH_MAX ||
		    size->h > ISP_PATH_FRAME_HEIGHT_MAX) {
			rtn = ISP_RTN_PATH_OUT_SIZE_ERR;
		} else {
			path->out_size.w = size->w;
			path->out_size.h = size->h;
		}
		break;

	case ISP_PATH_OUTPUT_FORMAT:
		format = *(unsigned int *)param;

		pr_info("ISP path%d isp out format %d\n", path_id, format);
		if ((format == DCAM_YUV422) || (format == DCAM_YUV420) ||
		    (format == DCAM_YVU420) || (format == DCAM_YUV420_3FRAME)) {
			path->output_format = format;
		} else {
			rtn = ISP_RTN_OUT_FMT_ERR;
			path->output_format = DCAM_FTM_MAX;
		}
		break;

	case ISP_PATH_OUTPUT_ADDR:
		p_addr = (struct camera_addr *)param;

		if (DCAM_YUV_ADDR_INVALID(p_addr->yaddr, p_addr->uaddr,
					  p_addr->vaddr) &&
		    p_addr->mfd_y == 0) {
			pr_err("FAILED: yuv output addr invalid\n");
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

			frame.pfinfo.dev = &s_isp_pdev->dev;
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

			pr_debug("y=0x%x u=0x%x v=0x%x mfd=0x%x 0x%x",
				 p_addr->yaddr, p_addr->uaddr, p_addr->vaddr,
				 frame.pfinfo.mfd[0], frame.pfinfo.mfd[1]);
			pr_debug("iova0=%lx, iova1=%lx\n",
				frame.pfinfo.iova[0], frame.pfinfo.iova[1]);
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

			frame->pfinfo.dev = &s_isp_pdev->dev;
			frame->pfinfo.mfd[0] = p_addr->mfd_y;
			frame->pfinfo.mfd[1] = p_addr->mfd_u;
			frame->pfinfo.mfd[2] = p_addr->mfd_u;
			/*may need update iommu here*/
			rtn = pfiommu_get_sg_table(&frame->pfinfo);
			if (rtn) {
				pr_err("cfg reserved output addr failed!\n");
				rtn = DCAM_RTN_PATH_ADDR_ERR;
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
		break;

	case ISP_PATH_MODE:
		path_mode = *(unsigned int *)param;
		path->path_mode = path_mode;
		pr_info("ISP path%d isp path mode %d\n", path_id, path_mode);
		break;

	case ISP_PATH_ZOOM_MODE:
		zoom_mode = *(unsigned int *)param;

		if (module->scl_array == NULL) {
			pr_err("Null pointer of scl_array\n");
			return -EFAULT;
		}
		module->scl_array->is_smooth_zoom = zoom_mode;
		break;
	case ISP_PATH_SN_MAX_SIZE:
		size = (struct camera_size *)param;
		/*
		 * TODO: so far we allocate binning buffer as full size
		 * It should be varied with binning (size'd be much smaller)
		 * FIXME: need to check in zsl scenario.
		 */
		if (path_id == ISP_SCL_CAP)
			off_type = ISP_OFF_BUF_FULL;
		else
			off_type = ISP_OFF_BUF_BIN;

		buf_desc = isp_offline_sel_buf(off_desc, off_type);
		buf_desc->buf_len =
			ALIGN(((unsigned long)((size->w + 3) / 4) * 5 + 3),
			ISP_PIXEL_ALIGN_WIDTH);
		buf_desc->buf_len *= size->h;
		break;

	case ISP_PATH_ENABLE:
		path->valid = *(unsigned int *)param;
		pr_info("ISP path%d isp path enable %d\n",
			path_id, path->valid);
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
			off_desc->data_endian.y_endian = endian->y_endian;
			if ((ISP_PATH_IDX_PRE |
			     ISP_PATH_IDX_CAP |
			     ISP_PATH_IDX_VID) & path_index) {
				path->data_endian.uv_endian = endian->uv_endian;
				off_desc->data_endian.uv_endian =
					endian->uv_endian;
				pr_debug("isp path data endian %d\n",
					path->data_endian.uv_endian);
			}
		}
		break;

	default:
		pr_err("fail to get path cfg type %d\n", id);
		break;
	}

	return -rtn;
}

int sprd_isp_get_afm_frame_info(void *isp_handle,
				struct camera_frame **out_frame)
{
	int rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_statis_module *module = NULL;

	if (!isp_handle) {
		rtn = -ISP_RTN_PARA_ERR;
		goto exit;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->statis_module_info;

	*out_frame = &module->afm_frame_info;

exit:
	return rtn;
}

int sprd_isp_get_offline_buffer(void *isp_handle,
	uint8_t off_type, struct camera_frame *out_frame)
{
	int rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_offline_desc *off_desc = NULL;
	struct offline_buf_desc *buf_desc = NULL;

	if (!isp_handle) {
		pr_err("fail to get ptr\n");
		rtn = ISP_RTN_PARA_ERR;
		goto _exit;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	if (unlikely(!module)) {
		pr_err("fail to get module,is NULL\n");
		rtn = ISP_RTN_PARA_ERR;
		goto _exit;
	}
	off_desc = &dev->module_info.off_desc;
	buf_desc = isp_offline_sel_buf(off_desc, off_type);

	if (buf_desc->frame_queue.valid_cnt < ISP_OFF_PRODUCER_Q_SIZE_MAX) {
		if (isp_offline_set_next_frm(module, off_type, out_frame)) {
			pr_err("fail to get new frame off_type %d\n", off_type);
			rtn = ISP_RTN_PATH_ADDR_ERR;
			goto _exit;
		}
	} else {
		pr_warn_ratelimited("type %d have %d frames in frame_queue\n",
			off_type, buf_desc->frame_queue.valid_cnt);
	}

_exit:
	pr_debug("Isp offline down end.\n");
	return -rtn;
}

int sprd_isp_set_offline_buffer(void *isp_handle, uint8_t off_type)
{
	int rtn = ISP_RTN_SUCCESS;
	struct camera_frame frame;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_offline_desc *off_desc = NULL;
	struct offline_buf_desc *buf_desc = NULL;

	if (!isp_handle) {
		rtn = ISP_RTN_PARA_ERR;
		goto _exit;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	off_desc = &dev->module_info.off_desc;
	module = &dev->module_info;

	buf_desc = isp_offline_sel_buf(off_desc, off_type);
	if (off_desc->read_buf_err == 1) {
		off_desc->read_buf_err = 0;
		goto _exit;
	}

	off_desc->shadow_done_cnt = 0;


	if (buf_desc->frame_queue.valid_cnt <=
			ISP_OFF_PRODUCER_Q_SIZE_MIN) {
		pr_warn_ratelimited("type %d need > 1 frames, %d\n",
			off_type, buf_desc->frame_queue.valid_cnt);
		goto _exit;
	}

	if (isp_frame_dequeue(&buf_desc->frame_queue, &frame)) {
		pr_err("fail to dequeue offline producer (%d)\n",
			off_type);
		rtn = ISP_RTN_PATH_ADDR_ERR;
		goto _exit;
	}

	pr_debug("enqueued into zsl q, frame_id: %d\n", frame.fid);
	if (unlikely(is_dual_cam && has_dual_cap_started &&
		off_type == ISP_OFF_BUF_FULL)) {
		/* do not update ZSL queue, please note this is not 100%
		 * precise that some frames can still slip into the queue
		 */
		if (isp_buf_queue_write(&buf_desc->tmp_buf_queue, &frame))
			pr_err("FATAL: fail to return off buf full_path\n");
		goto _exit;
	} else if (isp_frame_enqueue(&buf_desc->zsl_queue, &frame)) {
		pr_err("fail to enqueue offline consumer (%d)\n",
			off_type);
		rtn = ISP_RTN_PATH_ADDR_ERR;
		goto _exit;
	}

	pr_debug("start isp pipeline for %s\n",
		((off_type == ISP_OFF_BUF_FULL) ?
		"full_path" : "bin_path"));

	if (off_type == ISP_OFF_BUF_BIN) {
		rtn = sprd_isp_start_pipeline_bin(isp_handle,
						  DCAM_CAPTURE_NONE);
		if (rtn) {
			pr_err("fail to start isp pipeline bin path\n");
			rtn = ISP_RTN_PATH_ADDR_ERR;
			goto _exit;
		}
	} else if (off_type == ISP_OFF_BUF_FULL) {
		rtn = sprd_isp_start_pipeline_full(isp_handle,
						   DCAM_CAPTURE_NONE);
		if (rtn) {
			pr_err("fail to start isp pipeline full path\n");
			rtn = ISP_RTN_PATH_ADDR_ERR;
			goto _exit;
		}
	} else {
		pr_err("fail to get off_type\n");
		rtn = EINVAL;
		goto _exit;
	}

_exit:
	pr_debug("Isp offline down end.\n");
	return -rtn;
}

int sprd_isp_slw_flags_init(void *isp_handle, struct isp_path_info *info)
{
	pr_debug("+\n");
	return isp_slw_flags_init(isp_handle, info);
}

static int isp_enable_clk(void)
{
	int ret = 0;

	if (atomic_inc_return(&isp_enable_lock) == 1) {
		/*set isp clock to max value*/
		ret = clk_set_parent(isp0_clk, isp0_clk_parent);
		if (ret) {
			pr_info("isp0_clk_parent error.\n");
			clk_set_parent(isp0_clk, isp0_clk_default);
			clk_disable_unprepare(isp0_clk);
			goto exit;
		}

		ret = clk_prepare_enable(isp0_clk);
		if (ret) {
			pr_info("isp0_clk error.\n");
			clk_set_parent(isp0_clk, isp0_clk_default);
			clk_disable_unprepare(isp0_clk);
			goto exit;
		}

		/*isp enable*/
		ret = clk_prepare_enable(isp0_eb);
		if (ret) {
			pr_info("isp0_eb error.\n");
			goto exit;
		}

		ret = clk_prepare_enable(isp0_axi_eb);
		if (ret) {
			pr_info("isp0_axi_eb error.\n");
			goto exit;
		}
	} else
		pr_info("ISP clock already enabled\n");

exit:
	pr_info("isp enable clk done, %d\n",
		atomic_read(&isp_enable_lock));

	return ret;
}

static int isp_disable_clk(void)
{
	if (atomic_dec_return(&isp_enable_lock) == 0) {
		/*cut off the isp colck source*/
		clk_disable_unprepare(isp0_eb);
		clk_disable_unprepare(isp0_axi_eb);

		/*set isp clock to default value before power off*/
		clk_set_parent(isp0_clk, isp0_clk_default);
		clk_disable_unprepare(isp0_clk);
	}

	pr_info("isp disable clk done, %d\n",
		atomic_read(&isp_enable_lock));

	return 0;
}

void isp_clk_pause(enum isp_id iid, int i)
{
	int j;
	unsigned long flag;

	/* disable clk for pause */
	spin_lock_irqsave(&isp_clk_pr, flag);
	if (iid == ISP_ID_1)
		isp_clk_flag &= (~(i << 16));
	else
		isp_clk_flag &= (~i);
	j = isp_clk_flag;
	spin_unlock_irqrestore(&isp_clk_pr, flag);
	if (j == 0) {
		/* clk_disable_unprepare(isp0_clk); */
		ISP_HREG_WR(0, ISP_COMMON_GCLK_CTRL_0, 0);
		ISP_HREG_WR(0, ISP_COMMON_GCLK_CTRL_1, 0);
		ISP_HREG_WR(0, ISP_COMMON_GCLK_CTRL_2, 0);
		ISP_HREG_WR(0, ISP_COMMON_GCLK_CTRL_3, 0);
	}
	pr_debug("clk_p_pause, %d\n", j);
}

void isp_clk_resume(enum isp_id iid, int i)
{
	int j;
	unsigned long flag;

	spin_lock_irqsave(&isp_clk_pr, flag);
	/* enable clk */
	j = isp_clk_flag;
	if (iid == ISP_ID_1)
		isp_clk_flag |= (i << 16);
	else
		isp_clk_flag |= i;
	if (isp_clk_flag == j) {
		spin_unlock_irqrestore(&isp_clk_pr, flag);
		pr_debug("not write, %x\n", i);
		return;
	}
	spin_unlock_irqrestore(&isp_clk_pr, flag);
	if (!isp_clk_gt.g0) {
		/* use reg default valume */
		isp_clk_gt.g0 = 0xFFFF0000;
		isp_clk_gt.g1 = 0xFFFF0000;
		isp_clk_gt.g2 = 0xFFFF0000;
		isp_clk_gt.g3 = 0x0000FF00;
	}

	ISP_HREG_WR(0, ISP_COMMON_GCLK_CTRL_0, isp_clk_gt.g0);
	ISP_HREG_WR(0, ISP_COMMON_GCLK_CTRL_1, isp_clk_gt.g1);
	ISP_HREG_WR(0, ISP_COMMON_GCLK_CTRL_2, isp_clk_gt.g2);
	ISP_HREG_WR(0, ISP_COMMON_GCLK_CTRL_3, isp_clk_gt.g3);
	pr_debug("clk_p_resume\n");
}

/*register the callback func from the dcam_core*/
int sprd_isp_reg_isr(enum isp_id iid, enum isp_irq_id irq_id,
		isp_isr_func user_func, void *user_data)
{
	int ret = 0;

	pr_debug("irq_id %d\n", irq_id);
	if (irq_id >= ISP_IMG_MAX) {
		pr_err("isp IRQ is error.\n");
		ret = ISP_RTN_IRQ_NUM_ERR;
	} else {
		pr_debug("irq user_data %p\n", user_data);
		ret = isp_irq_callback(iid, irq_id, user_func, user_data);
		if (ret)
			pr_err("Register isp callback error\n");
	}

	return ret;
}

static int isp_module_init(struct isp_module *module_info, enum isp_id iid)
{
	int ret = 0;
	unsigned int i = 0;
	struct isp_cctx_desc *cctx_desc = NULL;

	if (!module_info) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	cctx_desc = isp_cctx_get_desc(iid);
	if (cctx_desc == NULL) {
		pr_err("CFG_MODE: cctx_desc %d is null!\n", iid);
		return -EFAULT;
	}

	module_info->cctx_desc = cctx_desc;

	for (i = ISP_SCL_PRE; i <= ISP_SCL_CAP; i++) {
		isp_buf_queue_init(&module_info->isp_path[i].buf_queue);
		init_completion(&module_info->isp_path[i].sof_com);
	}

	isp_offline_init_buf(&module_info->off_desc, ISP_OFF_BUF_BIN, true);
	isp_offline_init_buf(&module_info->off_desc, ISP_OFF_BUF_FULL, true);

	module_info->scl_array = vzalloc(sizeof(struct isp_sc_array));
	if (module_info->scl_array == NULL)
		return -ENOMEM;

	ret = isp_coeff_queue_init(module_info->scl_array);

	return ret;
}

static int isp_module_deinit(struct isp_module *module_info, enum isp_id iid)
{
	int ret = 0;
	int i = 0;

	if (!module_info) {
		pr_err("fail to get ptr\n");
		return -EFAULT;
	}

	for (i = ISP_SCL_PRE; i <= ISP_SCL_CAP; i++)
		isp_buf_queue_init(&module_info->isp_path[i].buf_queue);

	if (module_info->scl_array) {
		vfree(module_info->scl_array);
		module_info->scl_array = NULL;
	}

	return ret;
}

int sprd_isp_external_unmap(void *isp_handle)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_statis_module *statis_module = NULL;
	struct isp_k_block *isp_k_param = NULL;

	if (!isp_handle) {
		pr_err("Incorrect parameter.\n");
		return -EPERM;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	statis_module = &dev->statis_module_info;
	isp_k_param = &dev->isp_k_param;

	pfiommu_free_addr(&isp_k_param->lsc_pfinfo);
	memset(&isp_k_param->lsc_pfinfo, 0, sizeof(struct pfiommu_info));

	isp_offline_buf_iommu_unmap_external(
		&module->off_desc, ISP_OFF_BUF_BIN);
	isp_offline_buf_iommu_unmap_external(
		&module->off_desc, ISP_OFF_BUF_FULL);

	return ret;
}

static int isp_block_buf_alloc(struct isp_pipe_dev *dev)
{
	int32_t ret = 0;
	uint32_t buf_len = 0;
	struct isp_k_block *isp_k_param = NULL;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	enum isp_id iid;

	if (!dev) {
		pr_err("fail to get ptr dev is NULL\n");
		return -EFAULT;
	}

	isp_k_param = &dev->isp_k_param;
	fmcu_slice = &dev->fmcu_slice;
	iid = ISP_GET_IID(dev->com_idx);

	buf_len = ISP_FRGB_GAMMA_BUF_SIZE;

	isp_k_param->full_gamma_buf_addr = (unsigned long)vzalloc(buf_len);
	if (isp_k_param->full_gamma_buf_addr == 0) {
		pr_err("fail to alloc full gamma buf!\n");
		return -EPERM;
	}

	buf_len = ISP_NLM_BUF_SIZE;
	isp_k_param->nlm_vst_addr = vzalloc(buf_len);
	if (!isp_k_param->nlm_vst_addr) {
		pr_err("fail to alloc nlm vst buf!\n");
		return -EPERM;
	}

	isp_k_param->nlm_ivst_addr = vzalloc(buf_len);
	if (!isp_k_param->nlm_ivst_addr) {
		pr_err("fail to alloc nlm ivst buf!\n");
		return -EPERM;
	}

	buf_len = LENS_W_BUF_SIZE;
	isp_k_param->isp_lens_w_addr = vzalloc(buf_len);
	if (!isp_k_param->isp_lens_w_addr) {
		pr_err("fail to alloc isp lens weight buf!");
		return -EPERM;
	}

	isp_k_param->dcam_lens_w_addr = vzalloc(buf_len);
	if (!isp_k_param->dcam_lens_w_addr) {
		pr_err("fail to alloc dcam lens weight buf!");
		return -EPERM;
	}

	if (!isp_k_param->lsc_buf_info.sw_addr) {
		memset(isp_k_param->lsc_buf_info.name, '\0',
			(ISP_BUF_SHORT_NAME_LEN+1));
		strncpy(isp_k_param->lsc_buf_info.name, LSC_BUF_NAME,
			strlen(LSC_BUF_NAME)+1);
		isp_k_param->lsc_buf_info.size = ISP_LSC_BUF_SIZE;
		ret = isp_gen_buf_alloc(&isp_k_param->lsc_buf_info);
		if (ret != 0) {
			pr_err("can't alloc lsc buf, ret %d\n", ret);
			return -EPERM;
		}
		isp_gen_buf_hw_map(&isp_k_param->lsc_buf_info);
		if (ret) {
			pr_err("iid%d can't map lsc buf, ret %d\n",
			       iid, ret);
			return -EPERM;
		}
	}

	if (!fmcu_slice->cmdq_buf_info.sw_addr) {
		sprintf(fmcu_slice->cmdq_buf_info.name, "iid%d_fmcu", iid);
		fmcu_slice->cmdq_buf_info.size = ISP_FMCU_CMD_Q_SIZE;
		ret = isp_gen_buf_alloc(&fmcu_slice->cmdq_buf_info);
		if (ret != 0) {
			pr_err("iid%d can't alloc buf for fmcu cmdq, ret %d\n",
			       iid, ret);
			return -EPERM;
		}
		ret = isp_gen_buf_hw_map(&fmcu_slice->cmdq_buf_info);
		if (ret) {
			pr_err("iid%d can't map buf for fmcu cmdq, ret %d\n",
			       iid, ret);
			return -EPERM;
		}

		dev->fmcu_addr_phy =
			(unsigned long)fmcu_slice->cmdq_buf_info.hw_addr;
		dev->fmcu_addr_vir = fmcu_slice->cmdq_buf_info.sw_addr;
	}

	return ret;
}

static int isp_block_buf_free(struct isp_pipe_dev *dev)
{
	struct isp_k_block *isp_k_param = NULL;
	struct isp_fmcu_slice_desc *fmcu_slice = NULL;
	enum isp_id iid;

	if (!dev) {
		pr_err("fail to get ptr dev is NULL\n");
		return -EFAULT;
	}

	isp_k_param = &dev->isp_k_param;
	fmcu_slice = &dev->fmcu_slice;
	iid = ISP_GET_IID(dev->com_idx);

	if (isp_k_param->full_gamma_buf_addr != 0x00) {
		vfree((void *)isp_k_param->full_gamma_buf_addr);
		isp_k_param->full_gamma_buf_addr = 0x00;
	}

	if (isp_k_param->nlm_vst_addr) {
		vfree(isp_k_param->nlm_vst_addr);
		isp_k_param->nlm_vst_addr = NULL;
	}

	if (isp_k_param->nlm_ivst_addr) {
		vfree(isp_k_param->nlm_ivst_addr);
		isp_k_param->nlm_ivst_addr = NULL;
	}

	if (isp_k_param->isp_lens_w_addr) {
		vfree((void *)isp_k_param->isp_lens_w_addr);
		isp_k_param->isp_lens_w_addr = NULL;
	}

	if (isp_k_param->dcam_lens_w_addr) {
		vfree((void *)isp_k_param->dcam_lens_w_addr);
		isp_k_param->dcam_lens_w_addr = NULL;
	}

	if (isp_k_param->lsc_buf_info.sw_addr) {
		isp_gen_buf_hw_unmap(&isp_k_param->lsc_buf_info);
		isp_gen_buf_free(&isp_k_param->lsc_buf_info);
	}

	if (fmcu_slice->cmdq_buf_info.sw_addr) {
		isp_gen_buf_hw_unmap(&fmcu_slice->cmdq_buf_info);
		isp_gen_buf_free(&fmcu_slice->cmdq_buf_info);
	}

	return 0;
}

int sprd_isp_module_en(void *isp_handle, enum isp_id iid)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_cctx_desc *cctx_desc = NULL;
	struct isp_k_block *isp_k_param = NULL;

	if (!isp_handle) {
		pr_err("fail to get ptr dev is NULL\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	cctx_desc = dev->module_info.cctx_desc;
	isp_k_param = &dev->isp_k_param;

	ISP_SET_IID(dev->com_idx, iid);
	pr_info("ISP%d: enable isp module in: %d\n",
		iid, atomic_read(&s_isp_users[iid]));

	mutex_lock(&isp_module_sema[iid]);
	if (atomic_inc_return(&s_isp_users[iid]) == 1) {
		/*sprd_isp_pw_on */
		ret = sprd_cam_pw_on();
		if (ret) {
			pr_err("iid%d fail to power on cam sys %d\n",
			       iid, ret);
			goto exit;
		}

		ret = sprd_cam_domain_eb();
		if (ret) {
			pr_err("fail to sprd camera enable domain iid%d %d\n",
			       iid, ret);
			goto exit;
		}

		ret = isp_enable_clk();
		if (ret) {
			pr_err("fail to iid%d enable isp clk %d\n",
			       iid, ret);
			goto exit;
		}

		ret = sprd_isp_reset(dev);
		if (ret) {
			pr_err("fail to iid%d reset isp %d\n",
			       iid, ret);
			goto exit;
		}

		if (cctx_desc) {
			ret = cctx_desc->intf->init_cctx(cctx_desc, iid);
			if (ret) {
				pr_err("fail to iid%d init cctx_desc %d\n",
				       iid, ret);
				goto exit;
			}
		}

		ret = isp_block_buf_alloc(dev);
		if (ret) {
			pr_err("fail to alloc block buf iid%d %d\n",
			       iid, ret);
			isp_block_buf_free(dev);
		}

		ret = isp_irq_request(&s_isp_pdev->dev, &s_isp_irq[iid], dev);
		if (ret)
			pr_err("fail to install isp IRQ iid%d %d\n",
			       iid, ret);
	}

exit:
	mutex_unlock(&isp_module_sema[iid]);

	pr_info("ISP%d: enable isp module end: %d\n", iid,
		atomic_read(&s_isp_users[iid]));

	return ret;
}

int sprd_isp_module_dis(void *isp_handle, enum isp_id iid)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	struct isp_cctx_desc *cctx_desc = NULL;
	struct isp_k_block *isp_k_param = NULL;

	if (!isp_handle) {
		pr_err("fail to get ptr dev is NULL\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	module = &dev->module_info;
	if (module) {
		cctx_desc = module->cctx_desc;
	} else {
		pr_err("fail to get module,idx %d\n", dev->com_idx);
		/* Do not return here, cleanup other things */
	}

	isp_k_param = &dev->isp_k_param;

	pr_info("ISP%d: disable isp moduel, in %d\n",
		iid, atomic_read(&s_isp_users[iid]));

	if (atomic_read(&s_isp_users[iid]) == 0)
		return rtn;

	mutex_lock(&isp_module_sema[iid]);
	if (atomic_dec_return(&s_isp_users[iid]) == 0) {
		rtn = isp_irq_free(&s_isp_irq[iid], dev);
		if (rtn) {
			pr_err("fail to free isp IRQ iid%d %d\n",
			       iid, rtn);
			goto exit;
		}

		rtn = isp_block_buf_free(dev);
		if (rtn) {
			pr_err("fail to free block buf iid%d %d\n",
			       iid, rtn);
			goto exit;
		}

		if (cctx_desc) {
			rtn = cctx_desc->intf->deinit_cctx(cctx_desc, iid);
			if (rtn) {
				pr_err("fail to deinit cctx_desc iid%d %d\n",
				       iid, rtn);
				goto exit;
			}
		} else {
			pr_err("fail to get cctx_desc,idx%d\n", dev->com_idx);
		}

		rtn = isp_disable_clk();
		if (rtn) {
			pr_err("fail to disable isp clk iid%d %d\n",
			       iid, rtn);
			goto exit;
		}

		rtn = sprd_cam_domain_disable();
		if (rtn) {
			pr_err("fail to disable cam domain iid%d\n %d",
			       iid, rtn);
			goto exit;
		}

		rtn = sprd_cam_pw_off();
		if (rtn) {
			pr_err("fail to power off cam sys iid%d %d\n",
			       iid, rtn);
			goto exit;
		}
	}

exit:
	mutex_unlock(&isp_module_sema[iid]);

	pr_info("ISP%d: disable isp moduel end. %d\n", iid,
		   atomic_read(&s_isp_users[iid]));

	return rtn;
}

int sprd_isp_dev_init(void **isp_pipe_dev_handle, enum isp_id iid)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_statis_module *statis_module = NULL;

	if (!isp_pipe_dev_handle) {
		pr_err("fail to get input handle is NULL iid:%d\n", iid);
		return -EINVAL;
	}

	dev = vzalloc(sizeof(*dev));

	dev->is_raw_capture = 0;
	ISP_SET_IID(dev->com_idx, iid);
	mutex_init(&dev->isp_mutex);
	init_completion(&dev->irq_com);
	init_completion(&dev->isr_done_lock);
	dev->pre_lock = __SPIN_LOCK_UNLOCKED(pre_lock);
	dev->cap_lock = __SPIN_LOCK_UNLOCKED(cap_lock);

	dev->isr_count = 0;
	dev->isr_last_time = 0;

	statis_module = &dev->statis_module_info;

	ret = isp_queue_init(&dev->queue);
	if (unlikely(ret != 0)) {
		pr_err("fail to init queue\n");
		ret = -EIO;
		goto queue_exit;
	}

	ret = isp_statis_queue_init(&statis_module->aem_statis_queue);
	ret = isp_statis_queue_init(&statis_module->afl_statis_queue);
	ret = isp_statis_queue_init(&statis_module->afm_statis_queue);
	ret = isp_statis_queue_init(&statis_module->binning_statis_queue);
	ret = isp_statis_queue_init(&statis_module->hist_statis_queue);
	ret = isp_statis_queue_init(&statis_module->hist2_statis_queue);

	isp_statis_frm_queue_init(&statis_module->afl_statis_frm_queue);
	isp_statis_frm_queue_init(&statis_module->afm_statis_frm_queue);
	isp_statis_frm_queue_init(&statis_module->binning_statis_frm_queue);
	isp_statis_frm_queue_init(&statis_module->hist_statis_frm_queue);
	isp_statis_frm_queue_init(&statis_module->hist2_statis_frm_queue);

	ret = isp_module_init(&dev->module_info, iid);
	if (unlikely(ret != 0)) {
		pr_err("fail to init isp module info\n");
		ret = -EIO;
		goto queue_exit;
	}

	ret = isp_fmcu_slice_init(&dev->fmcu_slice.slice_handle);
	if (unlikely(ret != 0)) {
		pr_err("fail to init fmcu slice\n");
		ret = -EIO;
	}

	ret = isp_fmcu_slw_init(&dev->fmcu_slw.slw_handle);
	if (unlikely(ret != 0)) {
		pr_err("fail to init fmcu slw\n");
		ret = -EIO;
	}

	ret = isp_create_offline_thread(dev);
	if (unlikely(ret != 0)) {
		pr_err("fail to create offline thread\n");
		isp_stop_offline_thread(dev);
		ret = -EINVAL;
	}
	pr_info("dev init end! %p\n", dev);

queue_exit:
	if (unlikely(ret)) {
		vfree(dev);
		g_isp_dev_parray[iid] = NULL;
		pr_err("fait to init,dev[%d]\n", iid);
	} else {
		*isp_pipe_dev_handle = (void *)dev;
		g_isp_dev_parray[iid] = dev;
	}
	return ret;
}

int sprd_isp_dev_deinit(void *isp_pipe_dev_handle, enum isp_id iid)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;

	if (!isp_pipe_dev_handle) {
		pr_err("fail to get input handle is NULL iid:%d\n", iid);
		return -EINVAL;
	}

	dev = (struct isp_pipe_dev *)isp_pipe_dev_handle;
	mutex_lock(&dev->isp_mutex);
	ret = isp_queue_init(&dev->queue);
	if (unlikely(ret != 0)) {
		pr_err("fail to init queue\n");
		mutex_unlock(&dev->isp_mutex);
		vfree(dev);
		return -EIO;
	}

	ret = isp_module_deinit(&dev->module_info, iid);
	if (unlikely(ret != 0)) {
		pr_err("fail to init queue\n");
		mutex_unlock(&dev->isp_mutex);
		vfree(dev);
		return -EIO;
	}

	ret = isp_fmcu_slice_deinit(dev->fmcu_slice.slice_handle);
	if (unlikely(ret != 0)) {
		pr_err("fail to deinit fmcu slice\n");
		mutex_unlock(&dev->isp_mutex);
		vfree(dev);
		return -EIO;
	}

	ret = isp_fmcu_slw_deinit(dev->fmcu_slw.slw_handle);
	if (unlikely(ret != 0)) {
		pr_err("fail to deinit fmcu slw\n");
		mutex_unlock(&dev->isp_mutex);
		vfree(dev);
		return -EIO;
	}
	ret = isp_stop_offline_thread(dev);
	if (unlikely(ret != 0)) {
		pr_err("fail to stop offline thread\n");
		mutex_unlock(&dev->isp_mutex);
		vfree(dev);
		return -EIO;
	}

	dev->pre_lock = __SPIN_LOCK_UNLOCKED(pre_lock);
	dev->cap_lock = __SPIN_LOCK_UNLOCKED(cap_lock);

	init_completion(&dev->irq_com);
	init_completion(&dev->isr_done_lock);

	g_isp_dev_parray[iid] = NULL;

	mutex_unlock(&dev->isp_mutex);
	mutex_destroy(&dev->isp_mutex);

	vfree(dev);

	pr_info("dev[%d] deinit OK!\n", iid);

	return ret;
}

int sprd_isp_drv_init(struct platform_device *pdev)
{
	int i;

	isp_mod_lock = __SPIN_LOCK_UNLOCKED(isp_mod_lock);

	for (i = 0; i < s_isp_count; i++) {
		atomic_set(&s_isp_users[i], 0);
		mutex_init(&isp_module_sema[i]);
		isp_glb_reg_axi_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_axi_lock[i]);
		isp_glb_reg_mask_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_mask_lock[i]);
		isp_glb_reg_clr_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_clr_lock[i]);
	}

	return 0;
}

void sprd_isp_drv_deinit(void)
{
	int i;

	isp_mod_lock = __SPIN_LOCK_UNLOCKED(isp_mod_lock);

	for (i = 0; i < s_isp_count; i++) {
		atomic_set(&s_isp_users[i], 0);
		s_isp_irq[i].ch0 = 0;
		s_isp_irq[i].ch1 = 0;
		mutex_destroy(&isp_module_sema[i]);
		isp_glb_reg_axi_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_axi_lock[i]);
		isp_glb_reg_mask_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_mask_lock[i]);
		isp_glb_reg_clr_lock[i] =
			__SPIN_LOCK_UNLOCKED(&isp_glb_reg_clr_lock[i]);
	}
}

int sprd_isp_parse_dt(struct device_node *dn, unsigned int *isp_count)
{
	int i = 0;
	unsigned int count = 0;
	unsigned int offbuf_count = ISP_FRM_QUEUE_LENGTH;
	void __iomem *reg_base;
	struct device_node *isp_node = NULL;
	struct resource res = {0};

	pr_info("isp dev device node %s, full name %s\n",
		dn->name, dn->full_name);
	isp_node = of_parse_phandle(dn, "sprd,isp", 0);
	if (isp_node == NULL) {
		pr_err("fail to parse the property of sprd,isp\n");
		return -EFAULT;
	}

	pr_info("after isp dev device node %s, full name %s\n",
		isp_node->name, isp_node->full_name);
	s_isp_pdev = of_find_device_by_node(isp_node);
	pr_info("sprd s_isp_pdev name %s\n", s_isp_pdev->name);
	if (of_device_is_compatible(isp_node, "sprd,isp")) {
		if (of_property_read_u32_index(isp_node,
			"sprd,isp-count", 0, &count)) {
			pr_err("fail to parse the property of sprd,isp-count\n");
			return -EINVAL;
		}

		/*
		 * Reduce isp offline buffer count from 5 to 4
		 * considering HDR not supported and alleviate memory usage
		 * for low-end device.
		 */
		if (of_property_read_u32_index(isp_node,
			"sprd,isp-offbuf-count", 0, &offbuf_count)) {
			pr_info("fail to parse. set offline buffer count to 5.\n");
		}

		isp_frm_queue_len = offbuf_count;

		/*
		 * SharkLE, only one isp in HW, but used as 2 isp in SW.
		 * The isp count got from dts only used to check, will not
		 * be used.
		 * Change isp count here, avoiding to chang count in dts,
		 * thus dts still keeps the real isp count in HW.
		 *
		 */
		if (count > ISP_MAX_COUNT)
			pr_warn("isp-count got from dts maybe wrong\n");
		else
			count = ISP_MAX_COUNT;

		s_isp_count = count;
		*isp_count = count;
		isp0_eb = of_clk_get_by_name(isp_node, "isp_eb");
		if (IS_ERR(isp0_eb)) {
			pr_err("fail to get isp0_eb\n");
			return PTR_ERR(isp0_eb);
		}

		isp0_axi_eb = of_clk_get_by_name(isp_node,
			"isp_axi_eb");
		if (IS_ERR(isp0_axi_eb)) {
			pr_err("fail to get isp0_axi_eb\n");
			return PTR_ERR(isp0_axi_eb);
		}

		isp0_clk = of_clk_get_by_name(isp_node, "isp_clk");
		if (IS_ERR(isp0_clk)) {
			pr_err("fail to get isp0_clk\n");
			return PTR_ERR(isp0_clk);
		}

		isp0_clk_parent = of_clk_get_by_name(isp_node,
			"isp_clk_parent");
		if (IS_ERR(isp0_clk_parent)) {
			pr_err("fail to get isp0_clk_parent\n");
			return PTR_ERR(isp0_clk_parent);
		}

		cam_ahb_gpr = syscon_regmap_lookup_by_phandle(isp_node,
			"sprd,cam-ahb-syscon");
		if (IS_ERR(cam_ahb_gpr))
			return PTR_ERR(cam_ahb_gpr);

		isp0_clk_default = clk_get_parent(isp0_clk);
		if (IS_ERR(isp0_clk_default)) {
			pr_err("fail to get isp0_clk_default\n");
			return PTR_ERR(isp0_clk_default);
		}

		for (i = 0; i < count; i++) {
			/*
			 * In SharkLE, only one isp in HW, but used as
			 * two isp in SW, thus both of_address_to_resource
			 * and of_iomap use index 0 to get res and reg_base,
			 * avoiding to change isp reg property in dts.
			 */
			if (of_address_to_resource(isp_node, 0, &res))
				pr_err("fail to get isp phys addr\n");

			isp_phys_base[i] = (unsigned long)res.start;
			pr_info("isp phys reg base is %lx\n", isp_phys_base[i]);

			reg_base = of_iomap(isp_node, 0);
			if (!reg_base) {
				pr_err("fail to get isp reg_base %d\n", i);
				return -ENXIO;
			}

			s_isp_regbase[i] = (unsigned long)reg_base;

			s_isp_irq[i].ch0 = irq_of_parse_and_map(isp_node, i);
			if (s_isp_irq[i].ch0 <= 0) {
				pr_err("fail to get isp irq %d\n", i);
				return -EFAULT;
			}

			pr_info("ISP%d dts OK! base %lx, irq %d\n",
				i, s_isp_regbase[i], s_isp_irq[i].ch0);
		}
	} else {
		pr_err("fail to match isp device node\n");
		return -EINVAL;
	}

	return 0;
}

int isp_path_cap_with_vid_set_next_frm(struct isp_pipe_dev *dev)
{
	enum isp_drv_rtn rtn = ISP_RTN_SUCCESS;
	struct isp_module *module = NULL;
	struct isp_path_desc *path = NULL;
	struct isp_frm_queue *p_heap = NULL;
	struct camera_frame frame;
	struct isp_buf_queue *p_buf_queue = NULL;
	int size = 0;
	void *kaddr;

	module = &dev->module_info;
	path = &module->isp_path[ISP_SCL_CAP];
	p_heap = &path->frame_queue;
	p_buf_queue = &path->buf_queue;

	if (isp_buf_queue_read(p_buf_queue, &frame) == 0 &&
	    (frame.pfinfo.mfd[0] != 0)) {
		path->output_frame_count--;
	}
	if (frame.pfinfo.buf[0] != NULL) {
		kaddr = ion_heap_map_kernel(NULL,
				(struct ion_buffer *)frame.pfinfo.buf[0]);
		size = ((struct ion_buffer *)(frame.pfinfo.buf[0]))->size;
		if (IS_ERR(kaddr))
			pr_err("fail to map kernel memory");
		else {
			memset(kaddr, 0, size);
			vunmap((const void *)kaddr);
		}
	}
	if (isp_frame_enqueue(p_heap, &frame) == 0)
		pr_debug("success to enq frame buf\n");
	else {
		rtn = ISP_RTN_PATH_FRAME_LOCKED;
		pr_err("fail to enq frame buf\n");
	}
	return rtn;
}

void isp_handle_dcam_err(void *data)
{
	struct camera_dev *dev = (struct camera_dev *)data;
	struct isp_pipe_dev *isp_dev = NULL;

	pr_info("\n");
	if (dev == NULL) {
		pr_info("fail to get ptr\n");
		return;
	}

	isp_dev = (struct isp_pipe_dev *)dev->isp_dev_handle;
	if (isp_dev == NULL) {
		pr_info("incorrect isp_dev\n");
		return;
	}

	isp_quickstop(&isp_dev->module_info);
	if (fmcu_slice_capture_state == ISP_ST_START) {
		pr_info("force stop pipeline\n");
		sprd_isp_force_stop_pipeline(isp_dev);
	}
}
