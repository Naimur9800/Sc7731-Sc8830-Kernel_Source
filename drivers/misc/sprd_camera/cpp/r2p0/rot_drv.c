/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
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
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include <video/sprd_cpp.h>

#include "../../common/cam_common.h"
#include "cpp_reg.h"
#include "rot_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ROT_DRV: %d: %d " fmt, current->pid, __LINE__

#define ROT_ADDR_ALIGN 0x03

enum {
	ROT_ONE_BYTE = 0,
	ROT_TWO_BYTES,
	ROT_BYTE_MAX
};

enum {
	ROT_NORMAL = 0,
	ROT_UV422,
	ROT_DATA_FORMAT_MAX
};

static unsigned int rot_k_get_rot_format(struct sprd_cpp_rot_cfg_parm *parm)
{
	unsigned int fmt = ROT_ONE_BYTE;

	switch (parm->format) {
	case ROT_YUV422:
	case ROT_YUV420:
		fmt = ROT_ONE_BYTE;
		break;
	case ROT_RGB565:
		fmt = ROT_TWO_BYTES;
		break;
	default:
		break;
	}

	return fmt;
}

int cpp_rot_check_parm(struct sprd_cpp_rot_cfg_parm *parm)
{
	if (!parm)
		return -EINVAL;

	pr_debug("ROT:\n");
	pr_debug("w %d h %d\n", parm->size.w, parm->size.h);
	pr_debug("format %d angle %d\n", parm->format, parm->angle);
	pr_debug("src y:u:v 0x%x 0x%x 0x%x\n", parm->src_addr.y,
		 parm->src_addr.u, parm->src_addr.v);
	pr_debug("dst y:u:v 0x%x 0x%x 0x%x\n", parm->dst_addr.y,
		 parm->dst_addr.u, parm->dst_addr.v);

	if ((parm->src_addr.y & ROT_ADDR_ALIGN) ||
	    (parm->src_addr.u & ROT_ADDR_ALIGN) ||
	    (parm->src_addr.v & ROT_ADDR_ALIGN) ||
	    (parm->dst_addr.y & ROT_ADDR_ALIGN) ||
	    (parm->dst_addr.u & ROT_ADDR_ALIGN) ||
	    (parm->dst_addr.v & ROT_ADDR_ALIGN)) {
		pr_err("addr not aligned\n");
		return -EINVAL;
	}

	if (ROT_YUV422 != parm->format && ROT_YUV420 != parm->format &&
	    parm->format != ROT_RGB565) {
		pr_err("invalid image format %d\n", parm->format);
		return -EINVAL;
	}

	if (parm->angle > ROT_MIRROR) {
		pr_err("invalid rotation angle %d\n", parm->angle);
		return -EINVAL;
	}

	return 0;
}

int cpp_rot_is_end(struct sprd_cpp_rot_cfg_parm *parm)
{
	int ret = 1;

	switch (parm->format) {
	case ROT_YUV422:
	case ROT_YUV420:
		ret = 0;
		break;
	case ROT_RGB565:
		ret = 1;
		break;
	default:
		break;
	}

	return ret;
}

void cpp_rot_set_y_parm(struct sprd_cpp_rot_cfg_parm *parm,
		      struct rot_drv_private *p)
{
	if (!parm || !p)
		return;

	memcpy((void *)&p->cfg_parm, (void *)parm,
	       sizeof(struct sprd_cpp_rot_cfg_parm));

	memcpy(p->iommu_src.mfd, parm->src_addr.mfd, 3 * sizeof(unsigned int));
	memcpy(p->iommu_dst.mfd, parm->dst_addr.mfd, 3 * sizeof(unsigned int));

	#if 1
	cpp_get_sg_table(&p->iommu_src);
	p->iommu_src.offset[0] = p->cfg_parm.src_addr.y;
	p->iommu_src.offset[1] = p->cfg_parm.src_addr.u;
	cpp_get_addr(&p->iommu_src, SPRD_IOMMU_FM_CH_RW);

	cpp_get_sg_table(&p->iommu_dst);
	p->iommu_dst.offset[0] = p->cfg_parm.dst_addr.y;
	p->iommu_dst.offset[1] = p->cfg_parm.dst_addr.u;
	cpp_get_addr(&p->iommu_dst, SPRD_IOMMU_FM_CH_RW);
	#else
	p->iommu_src.iova[0] = p->cfg_parm.src_addr.y;
	p->iommu_src.iova[1] = p->cfg_parm.src_addr.u;

	p->iommu_dst.iova[0] = p->cfg_parm.dst_addr.y;
	p->iommu_dst.iova[1] = p->cfg_parm.dst_addr.u;
	#endif

	p->rot_fmt = rot_k_get_rot_format(parm);
	p->uv_mode = ROT_NORMAL;
	p->rot_src_addr = p->iommu_src.iova[0];
	p->rot_dst_addr = p->iommu_dst.iova[0];
	p->rot_size.w = parm->size.w;
	p->rot_size.h = parm->size.h;
	p->rot_src_endian = 1;
	p->rot_dst_endian = 1;
}

void cpp_rot_set_y_parm_t32_load_image(struct sprd_cpp_rot_cfg_parm *parm,
		      struct rot_drv_private *p)
{
	if (!parm || !p)
		return;

	memcpy((void *)&p->cfg_parm, (void *)parm,
	       sizeof(struct sprd_cpp_rot_cfg_parm));

	p->rot_fmt = rot_k_get_rot_format(parm);
	p->uv_mode = ROT_NORMAL;
	p->rot_src_addr = parm->src_addr.y; /*p->iommu_src.iova[0];*/
	p->rot_dst_addr = parm->dst_addr.y; /*p->iommu_dst.iova[0];*/
	p->rot_size.w = parm->size.w;
	p->rot_size.h = parm->size.h;
	p->rot_src_endian = 1;
	p->rot_dst_endian = 1;
}

void cpp_rot_set_uv_parm_t32_load_image(struct rot_drv_private *p)
{
	struct sprd_cpp_rot_cfg_parm *parm = &(p->cfg_parm);

	/*p->iommu_src.iova[1];*/
	p->rot_src_addr = parm->src_addr.y + (parm->size.w * parm->size.h);
	/*p->iommu_dst.iova[1];*/
	p->rot_dst_addr = parm->dst_addr.y + (parm->size.w * parm->size.h);
	p->rot_size.w >>= 0x01;
	p->rot_fmt = ROT_TWO_BYTES;

	if (p->cfg_parm.format == ROT_YUV422)
		p->uv_mode = ROT_UV422;
	else if (p->cfg_parm.format == ROT_YUV420) {
		p->uv_mode = ROT_NORMAL;
		p->rot_size.h >>= 0x01;
	}
}

void cpp_rot_set_uv_parm(struct rot_drv_private *p)
{
	p->rot_src_addr = p->iommu_src.iova[1];
	p->rot_dst_addr = p->iommu_dst.iova[1];
	p->rot_size.w >>= 0x01;
	p->rot_fmt = ROT_TWO_BYTES;

	if (p->cfg_parm.format == ROT_YUV422)
		p->uv_mode = ROT_UV422;
	else if (p->cfg_parm.format == ROT_YUV420) {
		p->uv_mode = ROT_NORMAL;
		p->rot_size.h >>= 0x01;
	}
}

void cpp_rot_start(struct rot_drv_private *p)
{
	reg_awr(p, CPP_PATH_START, (~CPP_ROT_START_BIT));

	/* set src addr */
	reg_wr(p, CPP_PATH1_SRC_ADDR, p->rot_src_addr);

	/* set dst addr */
	reg_wr(p, CPP_PATH1_DES_ADDR, p->rot_dst_addr);

	/* set image size */
	reg_wr(p, CPP_PATH1_SRC_OFFSET, 0x00000000);
	reg_wr(p, CPP_PATH1_ROT_SIZE,
	       ((p->rot_size.w & 0x1FFF) | ((p->rot_size.h & 0x1FFF) << 16)));
	reg_wr(p, CPP_PATH1_SRC_PITCH, (p->rot_size.w & 0xFFFF));

	/* set uv mode */
	reg_awr(p, CPP_ROTATION_PATH_CFG, (~CPP_ROT_UV_MODE_BIT));
	reg_owr(p, CPP_ROTATION_PATH_CFG, ((p->uv_mode & 0x1) << 3));

	/* set rot mode */
	reg_awr(p, CPP_ROTATION_PATH_CFG, (~CPP_ROT_MODE_MASK));
	reg_owr(p, CPP_ROTATION_PATH_CFG, ((p->cfg_parm.angle & 0x3) << 1));

	/* set rot format */
	reg_awr(p, CPP_ROTATION_PATH_CFG, (~CPP_ROT_PIXEL_FORMAT_BIT));
	reg_owr(p, CPP_ROTATION_PATH_CFG, ((p->rot_fmt & 0x1) << 0));

	/* set endian */
	reg_awr(p, CPP_AXIM_CHN_SET, (~CPP_ROT_AXI_WR_ENDIAN_MASK));
	reg_wr(p, CPP_AXIM_CHN_SET, (CPP_ROT_AXI_WR_ENDIAN_MASK & (0x5 << 8)));

	/* enable rotate */
	reg_owr(p, CPP_PATH_EB, CPP_ROT_EB_BIT);

	/* start rotate */
	reg_owr(p, CPP_PATH_START, CPP_ROT_START_BIT);
}

void cpp_rot_stop(struct rot_drv_private *p)
{
	/* disable rotate */
	reg_awr(p, CPP_PATH_START, (~CPP_ROT_START_BIT));
	udelay(1);
	reg_awr(p, CPP_PATH_EB, (~CPP_ROT_EB_BIT));

	cpp_free_addr(&p->iommu_src, SPRD_IOMMU_FM_CH_RW);
	cpp_free_addr(&p->iommu_dst, SPRD_IOMMU_FM_CH_RW);
}
