/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
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

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include <video/sprd_mm.h>

#include "cam_types.h"

#include "isp_reg.h"
#include "isp_fmcu.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_FMCU %d: %d %s:" \
	fmt, current->pid, __LINE__, __func__


static int isp_fmcu_push_cmd(
			struct isp_fmcu_ctx_desc *fmcu_ctx,
			uint32_t addr, uint32_t cmd)
{
	int ret = 0;
	uint32_t *ptr;

	if (!fmcu_ctx) {
		pr_err("null fmcu_ctx pointer\n");
		return -EFAULT;
	}
	if (fmcu_ctx->cmdq_pos > (fmcu_ctx->cmdq_size/sizeof(uint32_t))) {
		pr_err("error: fmcu%d cmdq overflow.\n", fmcu_ctx->fid);
		return -EFAULT;
	}

	ptr = fmcu_ctx->cmd_buf + fmcu_ctx->cmdq_pos;
	*ptr++ = cmd;
	*ptr++ = addr;
	fmcu_ctx->cmdq_pos += 2;

	return ret;
}


static int isp_fmcu_start(struct isp_fmcu_ctx_desc *fmcu_ctx)
{
	int ret = 0;
	int cmd_num;
	unsigned long base;

	if (!fmcu_ctx) {
		pr_err("null fmcu_ctx pointer\n");
		return -EFAULT;
	}
	pr_debug("start fmcu%d\n", fmcu_ctx->fid);

	if (fmcu_ctx->fid == 0)
		base =  ISP_FMCU0_BASE;
	else
		base =  ISP_FMCU1_BASE;

	if (fmcu_ctx->cmdq_pos > (fmcu_ctx->cmdq_size/sizeof(uint32_t))) {
		pr_err("error: fmcu%d cmdq overflow.\n", fmcu_ctx->fid);
		return -EFAULT;
	}
	cmd_num = (int) fmcu_ctx->cmdq_pos / 2;

#if 0
	{
		unsigned int i = 0;
		unsigned long addr = (unsigned long)fmcu_ctx->cmd_buf;

		pr_info("fmcu %d  cmd num %d\n",
				(int)fmcu_ctx->fid,  cmd_num);

		for (i = 0; i <= cmd_num; i += 2) {
			pr_info("a:0x%08x c: 0x%08x | a:0x%08x c: 0x%08x\n",
				*(uint32_t *)(addr + 4),
				*(uint32_t *)(addr),
				*(uint32_t *)(addr + 12),
				*(uint32_t *)(addr + 8));
			addr += 16;
		}
	}
#endif

	FLUSH_DCACHE(fmcu_ctx->cmd_buf, fmcu_ctx->cmdq_pos*sizeof(uint32_t));

	ISP_HREG_WR(base + ISP_FMCU_DDR_ADDR, fmcu_ctx->hw_addr);
	ISP_HREG_MWR(base + ISP_FMCU_CTRL, 0xFFFF0000, cmd_num << 16);
	ISP_HREG_WR(base + ISP_FMCU_START, 1);

	pr_debug("fmcu%d start done\n", fmcu_ctx->fid);

	return ret;
}


static int isp_fmcu_ctx_reset(struct isp_fmcu_ctx_desc *fmcu_ctx)
{
	int ret = 0;

	if (!fmcu_ctx) {
		pr_err("null fmcu_ctx pointer\n");
		return -EFAULT;
	}

	pr_debug("Enter\n");

	fmcu_ctx->cmdq_pos = 0;
	memset(fmcu_ctx->cmd_buf, 0, fmcu_ctx->cmdq_size);

	pr_debug("Done\n");
	return ret;
}

static int isp_fmcu_ctx_init(struct isp_fmcu_ctx_desc *fmcu_ctx, void *arg)
{
	int ret = 0;
	int iommu_enable = 0;
	unsigned long hw_addr = 0;
	struct camera_buf *ion_buf = NULL;

	if (!fmcu_ctx || !arg) {
		pr_err("null fmcu_ctx pointer\n");
		return -EFAULT;
	}
	pr_debug("Enter\n");

	fmcu_ctx->owner = (struct platform_device *)arg;
	fmcu_ctx->cmdq_pos = 0;
	fmcu_ctx->cmdq_size = ISP_FMCU_CMDQ_SIZE;
	fmcu_ctx->lock = __SPIN_LOCK_UNLOCKED(&fmcu_ctx->lock);

	/*alloc cmd queue buffer*/
	ion_buf = &fmcu_ctx->ion_pool;
	memset(ion_buf, 0, sizeof(fmcu_ctx->ion_pool));
	sprintf(ion_buf->name, "isp_fmcu_ctx");

	if (sprd_iommu_attach_device(&fmcu_ctx->owner->dev) == 0) {
		pr_info("isp iommu enable\n");
		iommu_enable = 1;
	} else {
		pr_info("isp iommu disable\n");
		iommu_enable = 0;
	}
	ret = cambuf_alloc(ion_buf,
				fmcu_ctx->cmdq_size, 0, iommu_enable);
	if (ret) {
		pr_err("fail to get fmcu buffer\n");
		ret = -EFAULT;
		goto err_alloc_fmcu;
	}

	ret = cambuf_kmap(ion_buf);
	if (ret) {
		pr_err("fail to kmap fmcu buffer\n");
		ret = -EFAULT;
		goto err_kmap_fmcu;
	}
	ret = cambuf_iommu_map(ion_buf,
				&fmcu_ctx->owner->dev);
	if (ret) {
		pr_err("fail to map fmcu buffer\n");
		ret = -EFAULT;
		goto err_hwmap_fmcu;
	}
	fmcu_ctx->cmd_buf = (uint32_t *)ion_buf->addr_k[0];
	fmcu_ctx->hw_addr = ion_buf->iova[0];

	pr_info("fmcu cmd buf hw_addr:0x%lx, sw_addr:%p, size:0x%x\n",
			hw_addr, fmcu_ctx->cmd_buf, (int)ion_buf->size[0]);

	return 0;

err_hwmap_fmcu:
	cambuf_kunmap(ion_buf);
err_kmap_fmcu:
	cambuf_free(ion_buf);
err_alloc_fmcu:
	pr_err("fmcu%d init failed.\n", fmcu_ctx->fid);
	return ret;
}

static int isp_fmcu_ctx_deinit(struct isp_fmcu_ctx_desc *fmcu_ctx)
{
	int ret = 0;
	struct camera_buf *ion_buf = NULL;

	if (!fmcu_ctx) {
		pr_err("null fmcu_ctx pointer\n");
		return -EFAULT;
	}

	pr_debug("Enter\n");
	ion_buf = &fmcu_ctx->ion_pool;
	cambuf_iommu_unmap(ion_buf);
	cambuf_kunmap(ion_buf);
	cambuf_free(ion_buf);

	pr_debug("Done\n");
	return ret;
}


struct isp_fmcu_ops fmcu_ops = {
	.ctx_init = isp_fmcu_ctx_init,
	.ctx_deinit = isp_fmcu_ctx_deinit,
	.ctx_reset = isp_fmcu_ctx_reset,
	.push_cmdq = isp_fmcu_push_cmd,
	.hw_start = isp_fmcu_start,
};

static struct isp_fmcu_ctx_desc s_fmcu_desc[2] = {
	{
		.fid = ISP_FMCU_0,
		.ops = &fmcu_ops,
	},
	{
		.fid = ISP_FMCU_1,
		.ops = &fmcu_ops,
	},
};

struct isp_fmcu_ctx_desc *get_isp_fmcu_ctx_desc(void)
{
	int i;
	struct isp_fmcu_ctx_desc *fmcu = NULL;

	for (i = 0; i < ISP_FMCU_NUM; i++) {
		if (atomic_inc_return(&s_fmcu_desc[i].user_cnt) == 1) {
			fmcu = &s_fmcu_desc[i];
			pr_info("fmcu %d , %p\n", fmcu->fid, fmcu);
			break;
		}
		atomic_dec(&s_fmcu_desc[i].user_cnt);
	}

	return fmcu;
}

int put_isp_fmcu_ctx_desc(struct isp_fmcu_ctx_desc *fmcu)
{
	int i;

	pr_info("fmcu %d. %p\n", fmcu->fid, fmcu);
	for (i = 0; i < ISP_FMCU_NUM; i++) {
		if (fmcu == &s_fmcu_desc[i]) {
			fmcu = NULL;
			atomic_dec(&s_fmcu_desc[i].user_cnt);
			break;
		}
	}

	if (fmcu != NULL)
		pr_err("error: ptr %p not matched original.\n", fmcu);

	return 0;
}
