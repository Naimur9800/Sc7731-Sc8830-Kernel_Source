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
#define pr_fmt(fmt) "SCALE,%s,%d: " fmt, __func__, __LINE__

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/semaphore.h>

#include <video/sprd_cpp.h>

#include "../../common/cam_common.h"
#include "cpp_reg.h"
#include "cpp_core.h"
#include "cpp_path2_drv.h"
#include "scaler_coef_gen.h"
#include "cpp_common.h"

/*#define CPP_TEST_DRIVER*/

/*#define SC_COEFF_H_TAB_OFFSET          0x0800*/
/*#define SC_COEFF_H_CHROMA_TAB_OFFSET   0x0880*/
/*#define SC_COEFF_V_TAB_OFFSET          0x08F0*/


/* Internal Function Implementation */

static void cpp_path2_k_ahb_reset(struct cpp_path2_drv_private *p)
{
	reg_awr(p, CPP_PATH_START, (~CPP_PATH2_START_BIT));
}

static void cpp_path2_k_set_input_size(struct cpp_path2_drv_private *p)
{
	struct sprd_cpp_path2_cfg_parm *cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_size.w > SCALE_FRAME_WIDTH_MAX ||
	    cfg_parm->input_size.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_info("invalid input size %d %d\n",
			cfg_parm->input_size.w, cfg_parm->input_size.h);
	}
	reg_mwr(p, CPP_PATH2_CFG3, CPP_PATH2_SRC_PITCH_MASK,
		cfg_parm->input_size.w & CPP_PATH2_SRC_PITCH_MASK);
}

static void cpp_path2_k_set_input_rect(struct cpp_path2_drv_private *p)
{
	struct sprd_cpp_path2_cfg_parm *cfg_parm = &p->cfg_parm;
	unsigned int val = 0x0;

	if (cfg_parm->input_rect.x > SCALE_FRAME_WIDTH_MAX ||
	    cfg_parm->input_rect.x > SCALE_FRAME_HEIGHT_MAX ||
	    cfg_parm->input_rect.w > SCALE_FRAME_WIDTH_MAX ||
	    cfg_parm->input_rect.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_info("invalid input rect %d %d %d %d\n",
			cfg_parm->input_rect.x, cfg_parm->input_rect.y,
			cfg_parm->input_rect.w, cfg_parm->input_rect.h);
	} else {
		val = cfg_parm->input_rect.y | (cfg_parm->input_rect.x << 16);
		reg_wr(p, CPP_PATH2_CFG4, val);
		val = cfg_parm->input_rect.w | (cfg_parm->input_rect.h << 16);
		reg_wr(p, CPP_PATH2_CFG1, val);
	}
}

static void cpp_path2_k_set_input_format(struct cpp_path2_drv_private *p)
{
	struct sprd_cpp_path2_cfg_parm *cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_format == SCALE_YUV422 ||
	    cfg_parm->input_format == SCALE_YUV420) {
		reg_mwr(p, CPP_PATH2_CFG0, CPP_PATH2_INPUT_FORMAT,
			cfg_parm->input_format);
		reg_mwr(p, CPP_PATH0_CFG0, BIT_1, 0);
	} else {
		pr_info("invalid input format %d\n", cfg_parm->input_format);
	}
}

#if !defined(CPP_TEST_DRIVER)
static void cpp_path2_k_set_src_addr(struct cpp_path2_drv_private *p)
{
	struct sprd_cpp_path2_cfg_parm *cfg_parm = &p->cfg_parm;

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	if (p->is_from_isp) {
		if (!cfg_parm->input_addr.y || !cfg_parm->input_addr.u) {
			pr_err("cpp path2 invalid cfg param\n");
			return;
		}
		reg_wr(p, CPP_PATH2_SRC_Y_BASE_ADDR,
			cfg_parm->input_addr.y);
		reg_wr(p, CPP_PATH2_SRC_UV_BASE_ADDR,
			cfg_parm->input_addr.u);
	} else
#endif
	{
		if (cfg_parm->input_addr.mfd[0] == 0 ||
			cfg_parm->input_addr.mfd[1] == 0) {
			pr_info("invalid input mfd %d %d %d\n",
				cfg_parm->input_addr.mfd[0],
				cfg_parm->input_addr.mfd[1],
				cfg_parm->input_addr.mfd[2]);
		} else {
			memcpy(p->iommu_src.mfd, cfg_parm->input_addr.mfd,
				3 * sizeof(unsigned int));
			cpp_get_sg_table(&p->iommu_src);
			p->iommu_src.offset[0] = cfg_parm->input_addr.y;
			p->iommu_src.offset[1] = cfg_parm->input_addr.u;
			p->iommu_src.offset[2] = cfg_parm->input_addr.v;
			cpp_get_addr(&p->iommu_src, SPRD_IOMMU_FM_CH_RW);

			reg_wr(p, CPP_PATH2_SRC_Y_BASE_ADDR,
				p->iommu_src.iova[0]);
			reg_wr(p, CPP_PATH2_SRC_UV_BASE_ADDR,
				p->iommu_src.iova[1]);
		}
	}
}
#else
static void cpp_path2_k_set_src_addr_t32_load_image
	(struct cpp_path2_drv_private *p)
{
	struct sprd_cpp_path2_cfg_parm *cfg_parm = &p->cfg_parm;

	pr_info("path2 set input y,u,v=(0x%x,0x%x,0x%x)\n",
		cfg_parm->input_addr.y, cfg_parm->input_addr.u,
		cfg_parm->input_addr.v);
	reg_wr(p, CPP_PATH2_SRC_Y_BASE_ADDR,
	       cfg_parm->input_addr.y);
	reg_wr(p, CPP_PATH2_SRC_UV_BASE_ADDR,
	       cfg_parm->input_addr.u);
}
#endif

static void cpp_path2_k_set_input_endian(struct cpp_path2_drv_private *p)
{
	struct sprd_cpp_path2_cfg_parm *cfg_parm = &p->cfg_parm;
	unsigned int y_endian = 0;
	unsigned int uv_endian = 0;

	if (cfg_parm->input_endian.y_endian >= PATH2_ENDIAN_MAX ||
		cfg_parm->input_endian.uv_endian >= PATH2_ENDIAN_MAX) {
		pr_info("invalid input endian %d %d\n",
			cfg_parm->input_endian.y_endian,
			cfg_parm->input_endian.uv_endian);
	} else {
		if (cfg_parm->input_endian.y_endian == PATH2_ENDIAN_LITTLE)
			y_endian = 0;

		if (cfg_parm->input_endian.y_endian == PATH2_ENDIAN_LITTLE &&
		    cfg_parm->input_endian.uv_endian == PATH2_ENDIAN_HALFBIG)
			uv_endian = 1;

		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_PATH2_INPUT_Y_ENDIAN),
			    y_endian << 16);
		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_PATH2_INPUT_UV_ENDIAN),
			    uv_endian << 19);
	}
}

static void cpp_path2_k_set_output_size(struct cpp_path2_drv_private *p)
{
	struct sprd_cpp_path2_cfg_parm *cfg_parm = &p->cfg_parm;
	unsigned int val = 0x0;

	if (cfg_parm->output_size.w > SCALE_FRAME_WIDTH_MAX ||
	    cfg_parm->output_size.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_info("invalid output size %d %d\n",
			cfg_parm->output_size.w, cfg_parm->output_size.h);
	} else {
		val = cfg_parm->output_size.w | (cfg_parm->output_size.h << 16);
		reg_wr(p, CPP_PATH2_CFG5, 0);
		reg_wr(p, CPP_PATH2_CFG2, val);
		reg_mwr(p, CPP_PATH2_CFG3, CPP_PATH2_DES_PITCH_MASK,
			(cfg_parm->output_size.
			 w << 16) & CPP_PATH2_DES_PITCH_MASK);
	}
}

static void cpp_path2_k_set_output_format(struct cpp_path2_drv_private *p)
{
	struct sprd_cpp_path2_cfg_parm *cfg_parm = &p->cfg_parm;

	if (cfg_parm->output_format == SCALE_YUV422 ||
	    cfg_parm->output_format == SCALE_YUV420) {
		reg_mwr(p, CPP_PATH2_CFG0, CPP_PATH2_OUTPUT_FORMAT,
			(cfg_parm->output_format << 2));
	} else {
		pr_info("invalid output format %d\n", cfg_parm->output_format);
	}
}

#if !defined(CPP_TEST_DRIVER)
static void cpp_path2_k_set_dst_addr(struct cpp_path2_drv_private *p)
{
	struct sprd_cpp_path2_cfg_parm *cfg_parm = &p->cfg_parm;

#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	if (p->is_from_isp) {
		if (!cfg_parm->output_addr.y || !cfg_parm->output_addr.u) {
			pr_err("cpp invalid output param\n");
			return;
		}
		reg_wr(p, CPP_PATH2_DES_Y_BASE_ADDR,
			cfg_parm->output_addr.y);
		reg_wr(p, CPP_PATH2_DES_UV_BASE_ADDR,
			cfg_parm->output_addr.u);
	} else
#endif
	{

		if (cfg_parm->output_addr.mfd[0] == 0 ||
			cfg_parm->output_addr.mfd[1] == 0) {
			pr_info("invalid output mfd %d %d %d\n",
			    cfg_parm->output_addr.mfd[0],
			    cfg_parm->output_addr.mfd[1],
			    cfg_parm->output_addr.mfd[2]);
		} else {
			memcpy(p->iommu_dst.mfd, cfg_parm->output_addr.mfd,
				3 * sizeof(unsigned int));
			cpp_get_sg_table(&p->iommu_dst);
			p->iommu_dst.offset[0] = cfg_parm->output_addr.y;
			p->iommu_dst.offset[1] = cfg_parm->output_addr.u;
			p->iommu_dst.offset[2] = cfg_parm->output_addr.v;
			cpp_get_addr(&p->iommu_dst, SPRD_IOMMU_FM_CH_RW);

			reg_wr(p, CPP_PATH2_DES_Y_BASE_ADDR,
				p->iommu_dst.iova[0]);
			reg_wr(p, CPP_PATH2_DES_UV_BASE_ADDR,
				p->iommu_dst.iova[1]);
		}
	}
}
#else
static void cpp_path2_k_set_dst_addr_t32_load_image
	(struct cpp_path2_drv_private *p)
{
	struct sprd_cpp_path2_cfg_parm *cfg_parm = &p->cfg_parm;

	pr_info("path2 set dst y,u,v=(0x%x,0x%x,0x%x)\n",
		cfg_parm->output_addr.y, cfg_parm->output_addr.u,
		cfg_parm->output_addr.v);
	reg_wr(p, CPP_PATH2_DES_Y_BASE_ADDR,
	       cfg_parm->output_addr.y);
	reg_wr(p, CPP_PATH2_DES_UV_BASE_ADDR,
	       cfg_parm->output_addr.u);
}
#endif

static void cpp_path2_k_set_output_endian(struct cpp_path2_drv_private *p)
{
	struct sprd_cpp_path2_cfg_parm *cfg_parm = &p->cfg_parm;
	unsigned int y_endian = 0;
	unsigned int uv_endian = 0;

	if (cfg_parm->output_endian.y_endian >= PATH2_ENDIAN_MAX ||
	    cfg_parm->output_endian.uv_endian >= PATH2_ENDIAN_MAX) {
		pr_info("invalid output endian %d %d\n",
			cfg_parm->output_endian.y_endian,
			cfg_parm->output_endian.uv_endian);
	} else {
		if (cfg_parm->input_endian.y_endian == PATH2_ENDIAN_LITTLE)
			y_endian = 0;

		if (cfg_parm->input_endian.y_endian == PATH2_ENDIAN_LITTLE &&
		    cfg_parm->input_endian.uv_endian == PATH2_ENDIAN_HALFBIG)
			uv_endian = 1;

		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_PATH2_OUTPUT_Y_ENDIAN),
			y_endian << 20);
		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_PATH2_OUTPUT_UV_ENDIAN),
			uv_endian << 23);
	}
}


#if 0
static void scale_k_srmode_enable(struct scale_drv_private *p)
{
	reg_owr(p, CPP_PATH0_CFG0, CPP_SCALE_SR_MODE_EB);
}
#endif

static void cpp_path2_k_enable(struct cpp_path2_drv_private *p)
{
	reg_owr(p, CPP_PATH_EB, CPP_PATH2_EB_BIT);
}

static void cpp_path2_k_disable(struct cpp_path2_drv_private *p)
{
	reg_awr(p, CPP_PATH_EB, (~CPP_PATH2_EB_BIT));
}

static int cpp_path2_k_calc_sc_size(struct cpp_path2_drv_private *p)
{
	int i = 0;
	unsigned int div_factor = 1;
	unsigned int deci_val = 0;
	unsigned int pixel_aligned_num = 0;
	struct sprd_cpp_path2_cfg_parm *cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH2_CFG0, CPP_PATH2_DEC_H_MASK, 0 << 4);
	reg_mwr(p, CPP_PATH2_CFG0, CPP_PATH2_DEC_V_MASK, 0 << 6);

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
		pr_info("invalid input rect %d %d, output size %d %d",
			cfg_parm->input_rect.w, cfg_parm->input_rect.h,
			cfg_parm->output_size.w, cfg_parm->output_size.h);
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
			reg_mwr(p, CPP_PATH2_CFG0, CPP_PATH2_DEC_H_MASK,
				i << 4);
			reg_mwr(p, CPP_PATH2_CFG0, CPP_PATH2_DEC_V_MASK,
				i << 6);
		}
	}
	pr_debug("sc_input_size %d %d, deci %d input_rect %d %d",
			p->sc_input_size.w, p->sc_input_size.h, i,
			cfg_parm->input_rect.w, cfg_parm->input_rect.h);

	return 0;
}

static int cpp_path2_k_set_sc_coeff(struct cpp_path2_drv_private *p)
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
	struct sprd_cpp_path2_cfg_parm *cfg_parm = &p->cfg_parm;

	h_coeff_addr += CPP_PATH2_SCALE_TABLE_HCOEF_LUMA;
	h_chroma_coeff_addr += CPP_PATH2_SCALE_TABLE_HCOEF_CHROMA;
	v_coeff_addr += CPP_PATH2_SCALE_TABLE_VCOEF;

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
				SC_COEFF_TMP_SIZE, p->sc_deci_val, false, 2))) {
		pr_err("scale_k_set_sc_coeff err\n");
		return -1;
	}

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

	reg_mwr(p, CPP_PATH2_CFG0,
		CPP_PATH2_Y_VER_TAP, ((y_tap & 0x0F) << 21));
	reg_mwr(p, CPP_PATH2_CFG0,
		CPP_PATH2_UV_VER_TAP, ((uv_tap & 0x1F) << 16));

	pr_debug("y_tap %d uv_tap %d\n", y_tap, uv_tap);
	#if 0
	cpp_print_coeff(p, h_coeff_addr, h_chroma_coeff_addr, v_coeff_addr);
	#endif

	return 0;
}

static int cpp_path2_k_cfg_scaler(struct cpp_path2_drv_private *p)
{
	int ret = 0;

	if (!p)
		return -EINVAL;

	ret = cpp_path2_k_calc_sc_size(p);
	if (ret)
		return -EINVAL;

	if (p->sc_input_size.w != p->cfg_parm.output_size.w ||
	    p->sc_input_size.h != p->cfg_parm.output_size.h ||
	    p->cfg_parm.input_format == SCALE_YUV420) {
		ret = cpp_path2_k_set_sc_coeff(p);
		if (ret)
			return -EINVAL;
	}

	return ret;
}

static void cpp_reg_trace(struct cpp_path2_drv_private *p)
{
#if 0 /*def SCALE_DRV_DEBUG*/
	unsigned long addr = 0;

	pr_info("CPP path2: Register list");
	for (addr = CPP_BASE; addr <= CPP_END; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			reg_rd(p, addr), reg_rd(p, addr + 4),
			reg_rd(p, addr + 8), reg_rd(p, addr + 12));
	}
#endif
}

int cpp_path2_start(struct sprd_cpp_path2_cfg_parm *parm,
		  struct cpp_path2_drv_private *p)
{
	int ret = 0;

	if (!parm || !p)
		return -EINVAL;

	memset(&p->sc_input_size, 0, sizeof(struct sprd_cpp_size));
	p->slice_in_height = 0;
	p->slice_out_height = 0;
	p->sc_deci_val = 0;

	memcpy((void *)&p->cfg_parm, (void *)parm,
	       sizeof(struct sprd_cpp_path2_cfg_parm));

	cpp_path2_k_ahb_reset(p);
	cpp_path2_k_enable(p);
	cpp_path2_k_set_input_size(p);
	cpp_path2_k_set_input_format(p);
#ifdef CPP_TEST_DRIVER
	cpp_path2_k_set_src_addr_t32_load_image(p);
#else
	cpp_path2_k_set_src_addr(p);
#endif
	cpp_path2_k_set_input_endian(p);
	cpp_path2_k_set_output_size(p);
	cpp_path2_k_set_output_format(p);
#ifdef CPP_TEST_DRIVER
	cpp_path2_k_set_dst_addr_t32_load_image(p);
#else
	cpp_path2_k_set_dst_addr(p);
#endif
	cpp_path2_k_set_output_endian(p);

	reg_mwr(p, CPP_PATH2_CFG0, CPP_PATH2_CLK_SWITCH, CPP_PATH2_CLK_SWITCH);

	ret = cpp_path2_k_cfg_scaler(p);
	if (ret) {
		pr_err("failed to start scaler\n");
		return ret;
	}
	cpp_path2_k_set_input_rect(p);
	pr_debug("in_size %d %d in_rect %d %d %d %d out_size %d %d sc_deci_val %d\n",
		p->cfg_parm.input_size.w, p->cfg_parm.input_size.h,
		p->cfg_parm.input_rect.x, p->cfg_parm.input_rect.y,
		p->cfg_parm.input_rect.w, p->cfg_parm.input_rect.h,
		p->cfg_parm.output_size.w, p->cfg_parm.output_size.h,
		p->sc_deci_val);
	pr_debug("in_addr 0x%x 0x%x out_addr 0x%x 0x%x\n",
		p->cfg_parm.input_addr.y, p->cfg_parm.input_addr.u,
		p->cfg_parm.output_addr.y, p->cfg_parm.output_addr.u);

	reg_awr(p, CPP_PATH2_CFG0, ~CPP_PATH2_CLK_SWITCH);
	udelay(1);

	reg_owr(p, CPP_PATH_START, CPP_PATH2_START_BIT);
	cpp_reg_trace(p);
	udelay(1);

	return 0;
}

void cpp_path2_stop(struct cpp_path2_drv_private *p)
{
	reg_awr(p, CPP_PATH_START, (~CPP_PATH2_START_BIT));
	cpp_path2_k_disable(p);
#ifdef CONFIG_ISP_CPP_COWORK_SUPPORT
	if (!p->is_from_isp)
#endif
	{
	    cpp_free_addr(&p->iommu_src, SPRD_IOMMU_FM_CH_RW);
	    cpp_free_addr(&p->iommu_dst, SPRD_IOMMU_FM_CH_RW);
	}
}
