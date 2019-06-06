/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#include <linux/semaphore.h>

#include <uapi/video/sprd_cpp.h>

#include "../../common/cam_common.h"
#include "cpp_reg.h"
#include "cpp_core.h"
#include "scale_drv.h"
#include "scaler_coef_gen.h"

/* Macro Definition */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "SCALE_DRV: %d: %d: " fmt, current->pid, __LINE__

#define SCALE_LOWEST_ADDR              0x800
#define SCALE_ADDR_INVALID(addr) \
	((unsigned long)(addr) < SCALE_LOWEST_ADDR)
#define SCALE_YUV_ADDR_INVALID(y, u, v) \
	(SCALE_ADDR_INVALID(y) && \
	SCALE_ADDR_INVALID(u) && \
	SCALE_ADDR_INVALID(v))

#define SCALE_FRAME_WIDTH_MAX          8192
#define SCALE_FRAME_HEIGHT_MAX         8192
#define SCALE_FRAME_OUT_WIDTH_MAX	   2432
#define SCALE_SC_COEFF_MAX             8
#define SCALE_DECI_FAC_MAX             4
#define SCALE_LINE_BUF_LENGTH          4096

#define SC_COEFF_H_TAB_OFFSET          0x0400
#define SC_COEFF_H_CHROMA_TAB_OFFSET   0x0480
#define SC_COEFF_V_TAB_OFFSET          0x04F0

#define SC_H_COEF_SIZE                 0x80
#define SC_H_CHROM_COEF_SIZE           0x40
#define SC_V_COEF_SIZE                 0x210

#define SC_COEFF_H_NUM                 (SC_H_COEF_SIZE / 4)
#define SC_COEFF_H_CHROMA_NUM          (SC_H_CHROM_COEF_SIZE / 4)
#define SC_COEFF_V_NUM                 (SC_V_COEF_SIZE / 4)

#define SC_COEFF_COEF_SIZE             (1 << 10)
#define SC_COEFF_TMP_SIZE \
	(SC_COEFF_BUF_SIZE - (SC_COEFF_COEF_SIZE * 3))

#define SCALE_PIXEL_ALIGNED            4
#define ALIGNED_DOWN_2(w) ((w) & ~(2 - 1))
#define ALIGNED_DOWN_4(w) ((w) & ~(4 - 1))
#define ALIGNED_DOWN_8(w) ((w) & ~(8 - 1))

/*#define SCALE_DRV_DEBUG*/

/* Internal Function Implementation */

static void scale_dev_stop(struct scale_drv_private *p)
{
	unsigned long flags;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return;
	}

	spin_lock_irqsave(p->hw_lock, flags);
	reg_awr(p, CPP_PATH_START, (~CPP_SCALE_START_BIT));
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static void scale_dev_start(struct scale_drv_private *p)
{
	unsigned long flags;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return;
	}
	spin_lock_irqsave(p->hw_lock, flags);
	reg_owr(p, CPP_PATH_START, CPP_SCALE_START_BIT);
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static int scale_k_check_param(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_size.w > SCALE_FRAME_WIDTH_MAX ||
		cfg_parm->input_size.h > SCALE_FRAME_HEIGHT_MAX ||
		cfg_parm->output_size.w > SCALE_FRAME_OUT_WIDTH_MAX ||
		cfg_parm->output_size.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_err("fail to get valid size:%d %d %d %d\n",
			cfg_parm->input_size.w, cfg_parm->input_size.h,
			cfg_parm->output_size.w, cfg_parm->output_size.h);
		return -1;
	} else if (cfg_parm->input_size.w < cfg_parm->input_rect.w +
		cfg_parm->input_rect.x ||
		cfg_parm->input_size.h < cfg_parm->input_rect.h +
		cfg_parm->input_rect.y) {
		pr_err("fail to get valid in size %d %d %d %d %d %d\n",
			cfg_parm->input_size.w, cfg_parm->input_size.h,
			cfg_parm->input_rect.x, cfg_parm->input_rect.y,
			cfg_parm->input_rect.w, cfg_parm->input_rect.h);
		return -1;
	} else {
		if (cfg_parm->output_size.w % 8 != 0) {
			pr_info("adjust dst width align 8: %d\n",
					cfg_parm->input_rect.x);
			return -1;
		}
		if (cfg_parm->output_format == SCALE_YUV420)
			if (cfg_parm->output_size.h % 2 != 0) {
			pr_info("adjust dst height align 2: %d\n",
					cfg_parm->input_rect.x);
			return -1;
			}
		if (cfg_parm->input_size.w % 8 != 0) {
			pr_err("fail to get src scale pitch size %d\n",
				cfg_parm->input_size.w);
			return -1;
		}
		if (cfg_parm->input_format == SCALE_YUV420) {
			if (cfg_parm->input_rect.h % 2 != 0) {
				cfg_parm->input_rect.h =
					ALIGNED_DOWN_2(cfg_parm->input_rect.h);
				pr_info("adjust src height align 2: %d\n",
					cfg_parm->input_rect.y);
			}
			if (cfg_parm->input_rect.y % 2 != 0) {
				cfg_parm->input_rect.y =
					ALIGNED_DOWN_2(cfg_parm->input_rect.y);
				pr_info("adjust src offset y align 2: %d\n",
					cfg_parm->input_rect.y);
			}
		}
		if (cfg_parm->input_rect.w % 4 != 0) {
			cfg_parm->input_rect.w =
				ALIGNED_DOWN_4(cfg_parm->input_rect.w);
			pr_info("adjust src width align 4: %d\n",
					cfg_parm->input_rect.y);
		}
		if (cfg_parm->input_rect.x % 2 != 0) {
			cfg_parm->input_rect.x =
				ALIGNED_DOWN_2(cfg_parm->input_rect.x);
			pr_info("adjust src offset x align 2: %d\n",
					cfg_parm->input_rect.x);
		}
	}

	return 0;
}

static void scale_k_set_input_size(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return;
	}
	cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH0_CFG3, CPP_SCALE_SRC_PITCH_MASK,
		cfg_parm->input_size.w & CPP_SCALE_SRC_PITCH_MASK);
}

static void scale_k_set_input_rect(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return;
	}
	cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH0_CFG1,
		CPP_SCALE_SRC_HEIGHT_MASK,
		(cfg_parm->input_rect.h << 16) &
		CPP_SCALE_SRC_HEIGHT_MASK);

	reg_mwr(p, CPP_PATH0_CFG4,
		CPP_SCALE_SRC_OFFSET_Y_MASK,
		cfg_parm->input_rect.y &
		CPP_SCALE_SRC_OFFSET_Y_MASK);

	reg_mwr(p, CPP_PATH0_CFG1, CPP_SCALE_SRC_WIDTH_MASK,
		cfg_parm->input_rect.w & 0x1fff);

	reg_mwr(p, CPP_PATH0_CFG4, CPP_SCALE_SRC_OFFSET_X_MASK,
		(cfg_parm->input_rect.x << 16) &
		CPP_SCALE_SRC_OFFSET_X_MASK);
}

static int scale_k_set_input_format(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_format == SCALE_YUV422 ||
	    cfg_parm->input_format == SCALE_YUV420) {
		reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_INPUT_FORMAT,
			(cfg_parm->input_format << 2));
	} else {
		pr_err("fail to get valid input format %d\n",
			cfg_parm->input_format);
		return -1;
	}

	return 0;
}

static int scale_k_set_addr(struct scale_drv_private *p)
{
	int ret = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_addr.mfd[0] == 0 ||
		cfg_parm->input_addr.mfd[1] == 0 ||
		cfg_parm->output_addr.mfd[0] == 0 ||
		cfg_parm->output_addr.mfd[1] == 0) {
		pr_info("fail to get valid input mfd %d %d %d\n",
			cfg_parm->input_addr.mfd[0],
			cfg_parm->input_addr.mfd[1],
			cfg_parm->input_addr.mfd[2]);
		pr_info("fail to get valid output mfd %d %d %d\n",
			cfg_parm->output_addr.mfd[0],
			cfg_parm->output_addr.mfd[1],
			cfg_parm->output_addr.mfd[2]);
		return -1;
	} else {
		memcpy(p->iommu_src.mfd, cfg_parm->input_addr.mfd,
			3 * sizeof(unsigned int));
		memcpy(p->iommu_dst.mfd, cfg_parm->output_addr.mfd,
			3 * sizeof(unsigned int));

		ret = cpp_get_sg_table(&p->iommu_src);
		if (ret) {
			pr_err("fail to get cpp src sg table\n");
			return -1;
		}
		p->iommu_src.offset[0] = cfg_parm->input_addr.y;
		p->iommu_src.offset[1] = cfg_parm->input_addr.u;
		p->iommu_src.offset[2] = cfg_parm->input_addr.v;
		ret = cpp_get_addr(&p->iommu_src);
		if (ret) {
			pr_err("fail to get cpp src addr\n");
			return -1;
		}

		ret = cpp_get_sg_table(&p->iommu_dst);
		if (ret) {
			pr_err("fail to get cpp dst sg table\n");
			cpp_free_addr(&p->iommu_src);
			return ret;
		}
		p->iommu_dst.offset[0] = cfg_parm->output_addr.y;
		p->iommu_dst.offset[1] = cfg_parm->output_addr.u;
		p->iommu_dst.offset[2] = cfg_parm->output_addr.v;
		ret = cpp_get_addr(&p->iommu_dst);
		if (ret) {
			pr_err("fail to get cpp dst addr\n");
			cpp_free_addr(&p->iommu_src);
			return ret;
		}

		reg_wr(p, CPP_PATH0_SRC_ADDR_Y,
			p->iommu_src.iova[0]);
		reg_wr(p, CPP_PATH0_SRC_ADDR_UV,
			p->iommu_src.iova[1]);
		reg_wr(p, CPP_PATH0_DES_ADDR_Y,
		       p->iommu_dst.iova[0]);
		reg_wr(p, CPP_PATH0_DES_ADDR_UV,
		       p->iommu_dst.iova[1]);

		pr_debug("iommu_src y:0x%lx, uv:0x%lx\n",
			p->iommu_src.iova[0], p->iommu_src.iova[1]);
	}

	return ret;
}

static int scale_k_set_input_endian(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;
	unsigned int y_endian = 0;
	unsigned int uv_endian = 0;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_endian.y_endian >= SCALE_ENDIAN_MAX ||
		cfg_parm->input_endian.uv_endian >= SCALE_ENDIAN_MAX) {
		pr_info("fail to get valid input endian %d %d\n",
			cfg_parm->input_endian.y_endian,
			cfg_parm->input_endian.uv_endian);
		return -1;
	} else {
		if (cfg_parm->input_endian.y_endian == SCALE_ENDIAN_LITTLE)
			y_endian = 0;

		if (cfg_parm->input_endian.y_endian == SCALE_ENDIAN_LITTLE &&
		cfg_parm->input_endian.uv_endian == SCALE_ENDIAN_HALFBIG)
			uv_endian = 1;

		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_INPUT_Y_ENDIAN),
			(y_endian & 0x7));
		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_INPUT_UV_ENDIAN),
			(uv_endian & 0x1) << 3);
		pr_debug("CPP:input endian y:%d, uv:%d\n", y_endian, uv_endian);
	}

	return 0;
}

static void scale_k_set_output_size(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return;
	}
	cfg_parm = &p->cfg_parm;

	reg_wr(p, CPP_PATH0_CFG5, 0);
	reg_mwr(p, CPP_PATH0_CFG3, CPP_SCALE_DES_PITCH_MASK,
		(cfg_parm->output_size.w << 16) &
		CPP_SCALE_DES_PITCH_MASK);
	reg_mwr(p, CPP_PATH0_CFG2, CPP_SCALE_DES_WIDTH_MASK,
		cfg_parm->output_size.w &
		CPP_SCALE_DES_WIDTH_MASK);
	reg_mwr(p, CPP_PATH0_CFG2, CPP_SCALE_DES_HEIGHT_MASK,
		(cfg_parm->output_size.h << 16) &
		CPP_SCALE_DES_HEIGHT_MASK);
}

static int scale_k_set_output_format(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

	if (cfg_parm->output_format == SCALE_YUV422 ||
	    cfg_parm->output_format == SCALE_YUV420) {
		reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_OUTPUT_FORMAT,
			(cfg_parm->output_format << 4));
	} else {
		pr_err("fail to get valid output format %d\n",
			cfg_parm->output_format);
		return -1;
	}

	return 0;
}

static int scale_k_set_output_endian(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;
	unsigned int y_endian = 0;
	unsigned int uv_endian = 0;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (cfg_parm->output_endian.y_endian >= SCALE_ENDIAN_MAX ||
	    cfg_parm->output_endian.uv_endian >= SCALE_ENDIAN_MAX) {
		pr_info("fail to get valid output endian %d %d\n",
			cfg_parm->output_endian.y_endian,
			cfg_parm->output_endian.uv_endian);
		return -1;
	} else {
		if (cfg_parm->output_endian.y_endian == SCALE_ENDIAN_LITTLE)
			y_endian = 0;

		if (cfg_parm->output_endian.y_endian == SCALE_ENDIAN_LITTLE &&
		    cfg_parm->output_endian.uv_endian == SCALE_ENDIAN_HALFBIG)
			uv_endian = 1;

		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_OUTPUT_Y_ENDIAN),
			y_endian << 4);
		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_OUTPUT_UV_ENDIAN),
			uv_endian << 7);
		pr_debug("CPP:output endian y:%d, uv:%d\n",
			y_endian, uv_endian);
	}

	return 0;
}

static void scale_dev_enable(struct scale_drv_private *p)
{
	unsigned long flags;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return;
	}
	cfg_parm = &p->cfg_parm;
	spin_lock_irqsave(p->hw_lock, flags);
	reg_owr(p, CPP_PATH_EB, CPP_SCALE_PATH_EB_BIT);
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static void scale_dev_disable(struct scale_drv_private *p)
{
	unsigned long flags;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return;
	}
	cfg_parm = &p->cfg_parm;

	spin_lock_irqsave(p->hw_lock, flags);
	reg_awr(p, CPP_PATH_EB, (~CPP_SCALE_PATH_EB_BIT));
	spin_unlock_irqrestore(p->hw_lock, flags);
}

static int scale_k_calc_sc_size(struct scale_drv_private *p)
{
	int i = 0;
	unsigned int div_factor = 1;
	unsigned int deci_val = 0;
	unsigned int pixel_aligned_num = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_DEC_H_MASK, 0 << 6);
	reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_DEC_V_MASK, 0 << 8);

	if (cfg_parm->input_rect.w >
	    (cfg_parm->output_size.w * SCALE_SC_COEFF_MAX *
	     (1 << SCALE_DECI_FAC_MAX))
	    || cfg_parm->input_rect.h >
	    (cfg_parm->output_size.h * SCALE_SC_COEFF_MAX *
	     (1 << SCALE_DECI_FAC_MAX))
	    || cfg_parm->input_rect.w * SCALE_SC_COEFF_MAX <
	    cfg_parm->output_size.w
	    || cfg_parm->input_rect.h * SCALE_SC_COEFF_MAX <
	    cfg_parm->output_size.h) {
		pr_info("fail to get valid input rect %d %d, output size %d %d",
			cfg_parm->input_rect.w, cfg_parm->input_rect.h,
			cfg_parm->output_size.w, cfg_parm->output_size.h);
		return -1;
	} else {
		p->sc_input_size.w = cfg_parm->input_rect.w;
		p->sc_input_size.h = cfg_parm->input_rect.h;
		if (cfg_parm->input_rect.w >
		    cfg_parm->output_size.w * SCALE_SC_COEFF_MAX
		    || cfg_parm->input_rect.h >
		    cfg_parm->output_size.h * SCALE_SC_COEFF_MAX) {
			for (i = 1; i < SCALE_DECI_FAC_MAX; i++) {
				div_factor =
				    (unsigned int)(SCALE_SC_COEFF_MAX *
						   (1 << i));
				if (cfg_parm->input_rect.w <=
				    (cfg_parm->output_size.w * div_factor)
				    && cfg_parm->input_rect.h <=
				    (cfg_parm->output_size.h * div_factor)) {
					break;
				}
			}
			deci_val = (1 << i);
			pixel_aligned_num =
			    (deci_val >= SCALE_PIXEL_ALIGNED) ?
			    deci_val : SCALE_PIXEL_ALIGNED;
			p->sc_input_size.w = cfg_parm->input_rect.w >> i;
			p->sc_input_size.h = cfg_parm->input_rect.h >> i;
			if ((p->sc_input_size.w % pixel_aligned_num) ||
			    (p->sc_input_size.h % pixel_aligned_num)) {
				p->sc_input_size.w =
				    p->sc_input_size.w / pixel_aligned_num *
				    pixel_aligned_num;
				p->sc_input_size.h =
				    p->sc_input_size.h / pixel_aligned_num *
				    pixel_aligned_num;
				cfg_parm->input_rect.w =
				    p->sc_input_size.w << i;
				cfg_parm->input_rect.h =
				    p->sc_input_size.h << i;
			}
			p->sc_deci_val = i;
			reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_DEC_H_MASK,
				i << 6);
			reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_DEC_V_MASK,
				i << 8);
		}
	}
	pr_debug("sc_input_size %d %d, deci %d input_rect %d %d",
			p->sc_input_size.w, p->sc_input_size.h, i,
			cfg_parm->input_rect.w, cfg_parm->input_rect.h);

	return 0;
}

static int scale_k_set_sc_coeff(struct scale_drv_private *p)
{
	unsigned int i = 0, j = 0;
	unsigned long h_coeff_addr = CPP_BASE;
	unsigned long h_chroma_coeff_addr = CPP_BASE;
	unsigned long v_coeff_addr = CPP_BASE;
	unsigned int *tmp_buf = NULL;
	unsigned int *h_coeff = NULL;
	unsigned int *h_chroma_coeff = NULL;
	unsigned int *v_coeff = NULL;
	unsigned int scale2yuv420 = 0;
	unsigned char y_tap = 0;
	unsigned char uv_tap = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	h_coeff_addr += SC_COEFF_H_TAB_OFFSET;
	h_chroma_coeff_addr += SC_COEFF_H_CHROMA_TAB_OFFSET;
	v_coeff_addr += SC_COEFF_V_TAB_OFFSET;

	if (cfg_parm->output_format == SCALE_YUV420)
		scale2yuv420 = 1;

	tmp_buf = (unsigned int *)p->coeff_addr;
	if (tmp_buf == NULL) {
		pr_err("coeff mem is null\n");
		return -1;
	}

	h_coeff = tmp_buf;
	h_chroma_coeff = tmp_buf + (SC_COEFF_COEF_SIZE / 4);
	v_coeff = h_chroma_coeff + (SC_COEFF_COEF_SIZE / 4);

	memset(h_coeff, 0xff, SC_H_COEF_SIZE);
	memset(h_chroma_coeff, 0xff, SC_H_CHROM_COEF_SIZE);
	memset(v_coeff, 0xff, SC_V_COEF_SIZE);

	if (!(Scale_GenScaleCoeff((short)cfg_parm->input_rect.w,
				  (short)cfg_parm->input_rect.h,
				  (short)cfg_parm->output_size.w,
				  (short)cfg_parm->output_size.h,
				  h_coeff,
				  h_chroma_coeff,
				  v_coeff,
				  scale2yuv420,
				  &y_tap,
				  &uv_tap,
				  tmp_buf + SC_COEFF_COEF_SIZE,
				  SC_COEFF_TMP_SIZE,
				  p->sc_deci_val,
				  cfg_parm->input_format,
				  cfg_parm->output_format))) {
		pr_err("scale_k_set_sc_coeff err\n");
		return -1;
	}

	reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_CLK_SWITCH, CPP_SCALE_CLK_SWITCH);

	for (i = 0; i < 8; i++)
		for (j = 0; j < 4; j++)
			reg_wr(p, h_coeff_addr + 4 * (4 * i + j),
			       h_coeff[4 * i + 3 - j]);

	for (i = 0; i < 8; i++)
		for (j = 0; j < 2; j++)
			reg_wr(p, h_chroma_coeff_addr + 4 * (2 * i + j),
			       h_chroma_coeff[2 * i + 1 - j]);

	for (i = 0; i < 132; i++)
		reg_wr(p, v_coeff_addr + 4 * i, v_coeff[i]);

	reg_awr(p, CPP_PATH0_CFG0, ~CPP_SCALE_CLK_SWITCH);

	reg_mwr(p, CPP_SC_TAP, CPP_SCALE_Y_VER_TAP, ((y_tap & 0x0F) << 5));
	reg_mwr(p, CPP_SC_TAP, CPP_SCALE_UV_VER_TAP, ((uv_tap & 0x1F)));

	pr_debug("y_tap %d uv_tap %d\n", y_tap, uv_tap);

	return 0;
}

static int scale_k_cfg_scaler(struct scale_drv_private *p)
{
	int ret = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}
	cfg_parm = &p->cfg_parm;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}

	ret = scale_k_calc_sc_size(p);
	if (ret) {
		pr_err("fail to calc sc size\n");
		return -EINVAL;
	}

	ret = scale_k_set_sc_coeff(p);
	if (ret) {
		pr_err("fail to set sc coeff\n");
		return -EINVAL;
	}

	return ret;
}

static void cpp_reg_trace(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return;
	}
	cfg_parm = &p->cfg_parm;

#ifdef SCALE_DRV_DEBUG
	unsigned long addr = 0;

	pr_info("CPP: Register list:\n");
	for (addr = CPP_BASE; addr <= CPP_END; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			reg_rd(p, addr), reg_rd(p, addr + 4),
			reg_rd(p, addr + 8), reg_rd(p, addr + 12));
	}
#endif
}
void get_cpp_max_size(unsigned int *max_width, unsigned int *max_height)
{
	*max_width = SCALE_FRAME_OUT_WIDTH_MAX;
	*max_height = SCALE_FRAME_HEIGHT_MAX;
}

int cpp_scale_start(struct sprd_cpp_scale_cfg_parm *parm,
		  struct scale_drv_private *p)
{
	int ret = 0;

	if (!parm || !p) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}

	memset(&p->sc_input_size, 0, sizeof(struct sprd_cpp_size));
	p->slice_in_height = 0;
	p->slice_out_height = 0;
	p->sc_deci_val = 0;

	memcpy((void *)&p->cfg_parm, (void *)parm,
	       sizeof(struct sprd_cpp_scale_cfg_parm));

	scale_dev_stop(p);
	scale_dev_enable(p);

	ret = scale_k_check_param(p);
	if (ret) {
		pr_err("fail to check size param\n");
		return ret;
	}
	scale_k_set_input_size(p);

	ret = scale_k_cfg_scaler(p);
	if (ret) {
		pr_err("fail to start scaler\n");
		return ret;
	}
	scale_k_set_output_size(p);
	scale_k_set_input_rect(p);

	ret = scale_k_set_input_format(p);
	if (ret) {
		pr_err("fail to set input format\n");
		return ret;
	}
	ret = scale_k_set_output_format(p);
	if (ret) {
		pr_err("fail to set output format\n");
		return ret;
	}
	ret = scale_k_set_input_endian(p);
	if (ret) {
		pr_err("fail to set input endian\n");
		return ret;
	}
	ret = scale_k_set_output_endian(p);
	if (ret) {
		pr_err("fail to set output endian\n");
		return ret;
	}
	ret = scale_k_set_addr(p);
	if (ret) {
		pr_err("fail to set addr\n");
		return ret;
	}

	pr_debug("in_size %d %d in_rect %d %d %d %d out_size %d %d dc_val %d\n",
		p->cfg_parm.input_size.w, p->cfg_parm.input_size.h,
		p->cfg_parm.input_rect.x, p->cfg_parm.input_rect.y,
		p->cfg_parm.input_rect.w, p->cfg_parm.input_rect.h,
		p->cfg_parm.output_size.w, p->cfg_parm.output_size.h,
		p->sc_deci_val);
	pr_debug("in_addr 0x%x 0x%x out_addr 0x%x 0x%x\n",
		p->cfg_parm.input_addr.y, p->cfg_parm.input_addr.u,
		p->cfg_parm.output_addr.y, p->cfg_parm.output_addr.u);

	cpp_reg_trace(p);

	scale_dev_start(p);

	return 0;
}

void cpp_scale_stop(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = NULL;

	if (!p) {
		pr_err("Input ptr is NULL\n");
		return;
	}
	cfg_parm = &p->cfg_parm;
	scale_dev_stop(p);
	scale_dev_disable(p);
	cpp_free_addr(&p->iommu_src);
	cpp_free_addr(&p->iommu_dst);
}
