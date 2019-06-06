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
#include "scale_drv.h"
#include "scaler_coef_gen.h"

/*#define CPP_TEST_DRIVER*/
/* Macro Definition */

#define SCALE_LOWEST_ADDR              0x800
#define SCALE_ADDR_INVALID(addr) \
	((unsigned long)(addr) < SCALE_LOWEST_ADDR)
#define SCALE_YUV_ADDR_INVALID(y, u, v) \
	(SCALE_ADDR_INVALID(y) && \
	SCALE_ADDR_INVALID(u) && \
	SCALE_ADDR_INVALID(v))

#define SCALE_FRAME_WIDTH_MAX          8192
#define SCALE_FRAME_HEIGHT_MAX         8192
#define SCALE_SC_COEFF_MAX             4
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
	(SC_COEFF_BUF_SIZE - (SC_COEFF_COEF_SIZE*3))

#define SCALE_PIXEL_ALIGNED            4

/*#define SCALE_DRV_DEBUG*/

/* Internal Function Implementation */

static void scale_k_ahb_reset(struct scale_drv_private *p)
{
	reg_awr(p, CPP_PATH_START, (~CPP_SCALE_START_BIT));
}
/* set pitch */
static void scale_k_set_input_size(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_size.w > SCALE_FRAME_WIDTH_MAX ||
	    cfg_parm->input_size.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_info("invalid input size %d %d\n",
			cfg_parm->input_size.w, cfg_parm->input_size.h);
	}
	reg_mwr(p, CPP_PATH0_CFG3, CPP_SCALE_SRC_PITCH_MASK,
		cfg_parm->input_size.w & CPP_SCALE_SRC_PITCH_MASK);
}
/* set input rect(x,y) and (w,h) */
static void scale_k_set_input_rect(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;
	unsigned int val = 0x0;

	if (cfg_parm->input_rect.x > SCALE_FRAME_WIDTH_MAX ||
	    cfg_parm->input_rect.y > SCALE_FRAME_HEIGHT_MAX ||
	    cfg_parm->input_rect.w > SCALE_FRAME_WIDTH_MAX ||
	    cfg_parm->input_rect.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_info("invalid input rect %d %d %d %d\n",
			cfg_parm->input_rect.x, cfg_parm->input_rect.y,
			cfg_parm->input_rect.w, cfg_parm->input_rect.h);
	} else {
		val = cfg_parm->input_rect.y | (cfg_parm->input_rect.x << 16);
		reg_wr(p, CPP_PATH0_CFG4, val);
		val = cfg_parm->input_rect.w | (cfg_parm->input_rect.h << 16);
		reg_wr(p, CPP_PATH0_CFG1, val);
	}
}

static void scale_k_set_input_format(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_format == SCALE_YUV422 ||
	    cfg_parm->input_format == SCALE_YUV420 ||
	    cfg_parm->input_format == SCALE_YUV420_3FRAME) {
		reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_INPUT_FORMAT,
			(cfg_parm->input_format << 2));
		reg_mwr(p, CPP_PATH0_CFG0, BIT_1, 0);
	} else if (cfg_parm->input_format == SCALE_JPEG_LS) {
		/* JPEGLS mode enable */
		reg_mwr(p, CPP_PATH0_CFG0, BIT_10, (1 << 10));
		/* split left width, DCAM supply */
		reg_mwr(p, CPP_PATH0_CFG0, 0x1FFF,
			(cfg_parm->split_left_block_w << 16));
	} else {
		pr_err("invalid input format %d\n", cfg_parm->input_format);
	}
}
#if !defined(CPP_TEST_DRIVER)
static int scale_k_set_src_addr(struct scale_drv_private *p)
{
	int ret = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

	if (cfg_parm->input_addr.mfd[0] == 0
	    || cfg_parm->input_addr.mfd[1] == 0) {
		pr_info("invalid input mfd %d %d %d\n",
			cfg_parm->input_addr.mfd[0],
			cfg_parm->input_addr.mfd[1],
			cfg_parm->input_addr.mfd[2]);
	} else {
		memcpy(p->iommu_src.mfd, cfg_parm->input_addr.mfd,
		       3 * sizeof(unsigned int));
		ret = cpp_get_sg_table(&p->iommu_src);
		if (ret) {
			pr_err("failed to get src sg table %d", ret);
			return ret;
		}
		p->iommu_src.offset[0] = cfg_parm->input_addr.y;
		p->iommu_src.offset[1] = cfg_parm->input_addr.u;
		p->iommu_src.offset[2] = cfg_parm->input_addr.v;
		cpp_get_addr(&p->iommu_src, SPRD_IOMMU_FM_CH_RW);
		CPP_LOG("scale set input y,u,v=(0x%lx,0x%lx,0x%lx)\n",
			p->iommu_src.iova[0], p->iommu_src.iova[1],
			p->iommu_src.iova[2]);
		reg_wr(p, CPP_PATH0_SRC_ADDR_Y,
		       p->iommu_src.iova[0]);
		reg_wr(p, CPP_PATH0_SRC_ADDR_UV,
		       p->iommu_src.iova[1]);
		reg_wr(p, CPP_PATH0_SRC_ADDR_V,
		       p->iommu_src.iova[2]);
	}

	return ret;
}
#else
static void scale_k_set_src_addr_t32_load_image(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

	pr_info("scale set input y,u,v=(0x%x,0x%x,0x%x)\n",
		cfg_parm->input_addr.y, cfg_parm->input_addr.u,
		cfg_parm->input_addr.v);
	reg_wr(p, CPP_PATH0_SRC_ADDR_Y,
	       cfg_parm->input_addr.y);
	reg_wr(p, CPP_PATH0_SRC_ADDR_UV,
	       cfg_parm->input_addr.u);
	reg_wr(p, CPP_PATH0_SRC_ADDR_V,
	       cfg_parm->input_addr.v);
}
#endif



#if 0
static void scale_k_set_r_src_addr(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

	if (SCALE_YUV_ADDR_INVALID(cfg_parm->input_addr.y,
				   cfg_parm->input_addr.u,
				   cfg_parm->input_addr.v)) {
		pr_info("invalid r src addr 0x%x 0x%x 0x%x\n",
			cfg_parm->input_addr.y,
			cfg_parm->input_addr.u, cfg_parm->input_addr.v);
	} else {
		reg_wr(p, CPP_PATH0_R_SRC_ADDR_Y,
		       cfg_parm->input_r_block_addr.y);
		reg_wr(p, CPP_PATH0_R_SRC_ADDR_UV,
		       cfg_parm->input_r_block_addr.u);
		reg_wr(p, CPP_PATH0_R_SRC_ADDR_V,
		       cfg_parm->input_r_block_addr.v);
	}
}
#endif

/* endian is error,8 endian,but there is 4 */
static void scale_k_set_input_endian(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;
	unsigned int y_endian = 0;
	unsigned int uv_endian = 0;

	if (cfg_parm->input_endian.y_endian >= SCALE_ENDIAN_MAX ||
	    cfg_parm->input_endian.uv_endian >= SCALE_ENDIAN_MAX) {
		pr_err("invalid input endian %d %d\n",
			cfg_parm->input_endian.y_endian,
			cfg_parm->input_endian.uv_endian);
	} else {
		if (cfg_parm->input_endian.y_endian == SCALE_ENDIAN_LITTLE)
			y_endian = 0;

		if (cfg_parm->input_endian.y_endian == SCALE_ENDIAN_LITTLE &&
		    cfg_parm->input_endian.uv_endian == SCALE_ENDIAN_HALFBIG)
			uv_endian = 1;

		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_INPUT_Y_ENDIAN),
			y_endian);
		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_INPUT_UV_ENDIAN),
			uv_endian << 3);
	}
}

static void scale_k_set_output_size(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;
	unsigned int val = 0x0;

	if (cfg_parm->output_size.w > SCALE_FRAME_WIDTH_MAX ||
	    cfg_parm->output_size.h > SCALE_FRAME_HEIGHT_MAX) {
		pr_err("invalid output size %d %d\n",
			cfg_parm->output_size.w, cfg_parm->output_size.h);
	} else {
		val = cfg_parm->output_size.w | (cfg_parm->output_size.h << 16);
		reg_wr(p, CPP_PATH0_CFG5, 0);
		reg_wr(p, CPP_PATH0_CFG2, val);
		reg_mwr(p, CPP_PATH0_CFG3, CPP_SCALE_DES_PITCH_MASK,
			(cfg_parm->output_size.
			 w << 16) & CPP_SCALE_DES_PITCH_MASK);
	}
}

static void scale_k_set_output_format(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

	if (cfg_parm->output_format == SCALE_YUV422 ||
	    cfg_parm->output_format == SCALE_YUV420 ||
	    cfg_parm->output_format == SCALE_YUV420_3FRAME ||
	    cfg_parm->output_format == SCALE_JPEG_LS) {
		reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_OUTPUT_FORMAT,
			(cfg_parm->output_format << 4));
	} else {
		pr_info("invalid output format %d\n", cfg_parm->output_format);
	}
}
#if !defined(CPP_TEST_DRIVER)
static int scale_k_set_dst_addr(struct scale_drv_private *p)
{
	int ret = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

	if (cfg_parm->output_addr.mfd[0] == 0 ||
	    cfg_parm->output_addr.mfd[1] == 0) {
		CPP_LOG("invalid output mfd %d %d %d\n",
			cfg_parm->output_addr.mfd[0],
			cfg_parm->output_addr.mfd[1],
			cfg_parm->output_addr.mfd[2]);
	} else {
		memcpy(p->iommu_dst.mfd, cfg_parm->output_addr.mfd,
		       3 * sizeof(unsigned int));
		ret = cpp_get_sg_table(&p->iommu_dst);
		if (ret) {
			pr_err("failed to get dst sg table %d", ret);
			return ret;
		}
		p->iommu_dst.offset[0] = cfg_parm->output_addr.y;
		p->iommu_dst.offset[1] = cfg_parm->output_addr.u;
		p->iommu_dst.offset[2] = cfg_parm->output_addr.v;
		cpp_get_addr(&p->iommu_dst, SPRD_IOMMU_FM_CH_RW);
		CPP_LOG("scale set dst y,u,v=(0x%lx,0x%lx,0x%lx)\n",
			p->iommu_dst.iova[0],
			p->iommu_dst.iova[1],
			p->iommu_dst.iova[2]);
		reg_wr(p, CPP_PATH0_R_DES_ADDR_Y,
		       p->iommu_dst.iova[0]);
		reg_wr(p, CPP_PATH0_R_DES_ADDR_UV,
		       p->iommu_dst.iova[1]);
		reg_wr(p, CPP_PATH0_R_DES_ADDR_V,
		       p->iommu_dst.iova[2]);
	}

	return ret;
}
#else
static void scale_k_set_dst_addr_t32_load_image(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

	pr_info("scale set dst y,u,v=(0x%x,0x%x,0x%x)\n",
		cfg_parm->output_addr.y, cfg_parm->output_addr.u,
		cfg_parm->output_addr.v);
	reg_wr(p, CPP_PATH0_R_DES_ADDR_Y,
	       cfg_parm->output_addr.y);
	reg_wr(p, CPP_PATH0_R_DES_ADDR_UV,
	       cfg_parm->output_addr.u);
	reg_wr(p, CPP_PATH0_R_DES_ADDR_V,
	       cfg_parm->output_addr.v);
}
#endif

static void scale_k_set_output_endian(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;
	unsigned int y_endian = 0;
	unsigned int uv_endian = 0;

	if (cfg_parm->output_endian.y_endian >= SCALE_ENDIAN_MAX ||
	    cfg_parm->output_endian.uv_endian >= SCALE_ENDIAN_MAX) {
		pr_err("invalid output endian %d %d\n",
			cfg_parm->output_endian.y_endian,
			cfg_parm->output_endian.uv_endian);
	} else {
		if (cfg_parm->input_endian.y_endian == SCALE_ENDIAN_LITTLE)
			y_endian = 0;

		if (cfg_parm->input_endian.y_endian == SCALE_ENDIAN_LITTLE &&
		    cfg_parm->input_endian.uv_endian == SCALE_ENDIAN_HALFBIG)
			uv_endian = 1;

		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_OUTPUT_Y_ENDIAN),
			y_endian << 4);
		reg_mwr(p, CPP_AXIM_CHN_SET, (CPP_SCALE_OUTPUT_UV_ENDIAN),
			uv_endian << 7);
	}
}

#if 0
static void scale_k_set_src_regulate(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH0_SRC_YUV_REGULATE_0, (BIT_1 | BIT_0),
		cfg_parm->regualte_mode_src);

	if (cfg_parm->regualte_mode == SCALE_REGULATE_MODE_SPECIAL_EFFECT) {
		reg_mwr(p, CPP_PATH0_SRC_YUV_REGULATE_0, 0x80,
			(cfg_parm->regulate_threshold.effect_threshold_y << 8));
		reg_mwr(p, CPP_PATH0_SRC_YUV_REGULATE_1, 0x80,
			cfg_parm->regulate_threshold.effect_threshold_uv);
	} else if (cfg_parm->regualte_mode == SCALE_REGULATE_MODE_CUT) {
		reg_mwr(p, CPP_PATH0_SRC_YUV_REGULATE_0, 0x10,
			(cfg_parm->regulate_threshold.down_threshold_y << 16));
		reg_mwr(p, CPP_PATH0_SRC_YUV_REGULATE_0, 0xec,
			(cfg_parm->regulate_threshold.up_threshold_y << 24));
		reg_mwr(p, CPP_PATH0_SRC_YUV_REGULATE_1, 0x10,
			(cfg_parm->regulate_threshold.down_threshold_uv << 8));
		reg_mwr(p, CPP_PATH0_SRC_YUV_REGULATE_1, 0xf1,
			(cfg_parm->regulate_threshold.up_threshold_uv << 16));
	}
}

static void scale_k_set_des_regulate(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

	reg_mwr(p, CPP_PATH0_DES_YUV_REGULATE_0, (BIT_1 | BIT_0),
		cfg_parm->regualte_mode_des);

	if (cfg_parm->regualte_mode == SCALE_REGULATE_MODE_SPECIAL_EFFECT) {
		reg_mwr(p, CPP_PATH0_DES_YUV_REGULATE_0, 0x80,
			(cfg_parm->regulate_threshold.effect_threshold_y << 8));
		reg_mwr(p, CPP_PATH0_DES_YUV_REGULATE_1, 0x80,
			cfg_parm->regulate_threshold.effect_threshold_uv);
	} else if (cfg_parm->regualte_mode == SCALE_REGULATE_MODE_CUT) {
		reg_mwr(p, CPP_PATH0_DES_YUV_REGULATE_0, 0x10,
			(cfg_parm->regulate_threshold.down_threshold_y << 16));
		reg_mwr(p, CPP_PATH0_DES_YUV_REGULATE_0, 0xec,
			(cfg_parm->regulate_threshold.up_threshold_y << 24));
		reg_mwr(p, CPP_PATH0_DES_YUV_REGULATE_1, 0x10,
			(cfg_parm->regulate_threshold.down_threshold_uv << 8));
		reg_mwr(p, CPP_PATH0_DES_YUV_REGULATE_1, 0xf1,
			(cfg_parm->regulate_threshold.up_threshold_uv << 16));
	}
}
#endif

#if 0
static void scale_k_set_jpegls_infor(struct scale_drv_private *p)
{
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

	if (cfg_parm->regualte_mode < 4208) {
		/* no split mode */
		reg_wr(p, CPP_PATH0_JPEGLS_INFOR_Y, cfg_parm->jpegls_info.y);
		reg_wr(p, CPP_PATH0_JPEGLS_INFOR_U, cfg_parm->jpegls_info.u);
		reg_wr(p, CPP_PATH0_JPEGLS_INFOR_V, cfg_parm->jpegls_info.v);
	} else {
		/* split mode */
		reg_wr(p, CPP_PATH0_R_JPEGLS_INFOR_Y, cfg_parm->jpegls_info.y);
		reg_wr(p, CPP_PATH0_R_JPEGLS_INFOR_U, cfg_parm->jpegls_info.u);
		reg_wr(p, CPP_PATH0_R_JPEGLS_INFOR_V, cfg_parm->jpegls_info.v);
	}
}

static void scale_k_srmode_enable(struct scale_drv_private *p)
{
	reg_owr(p, CPP_PATH0_CFG0, CPP_SCALE_SR_MODE_EB);
}
#endif

static void scale_k_enable(struct scale_drv_private *p)
{
	reg_owr(p, CPP_PATH_EB, CPP_SCALE_PATH_EB_BIT);
}

static void scale_k_disable(struct scale_drv_private *p)
{
	reg_awr(p, CPP_PATH_EB, (~CPP_SCALE_PATH_EB_BIT));
}

static int scale_k_calc_sc_size(struct scale_drv_private *p)
{
	int i = 0;
	unsigned int div_factor = 1;
	unsigned int deci_val = 0;
	unsigned int pixel_aligned_num = 0;
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

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
		CPP_LOG("invalid input rect %d %d, output size %d %d",
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
			reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_DEC_H_MASK,
				i << 6);
			reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_DEC_V_MASK,
				i << 8);
		}
	}
	CPP_LOG("sc_input_size %d %d, deci %d input_rect %d %d",
			p->sc_input_size.w, p->sc_input_size.h, i,
			cfg_parm->input_rect.w, cfg_parm->input_rect.h);

	return 0;
}

static void cpp_print_coeff(struct scale_drv_private *p,
			    unsigned long h_coeff_addr,
			    unsigned long h_chroma_coeff_addr,
			    unsigned long v_coeff_addr)
{
#ifdef SCALE_DRV_DEBUG
	int i = 0;
	int j = 0;
	unsigned long addr = 0;

	pr_info("CPP: h_coeff_addr\n");
	for (i = 0; i < 8; i++) {
		pr_info("0x%lx: ", h_coeff_addr + 4 * (4 * i));
		for (j = 0; j < 4; j++)
			pr_info("0x%x ",
				reg_rd(p, h_coeff_addr + 4 * (4 * i + j)));
		pr_info("\n");
	}

	pr_info("CPP: h_chroma_coeff_addr\n");
	for (i = 0; i < 8; i++) {
		pr_info("0x%lx: ", h_chroma_coeff_addr + 4 * (2 * i));
		for (j = 0; j < 2; j++)
			pr_info("0x%x ",
				reg_rd(p,
				       h_chroma_coeff_addr + 4 * (2 * i + j)));
		pr_info("\n");
	}

	pr_info("CPP: v_coeff_addr\n");
	for (addr = v_coeff_addr; addr <= 0x6FC; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			reg_rd(p, addr), reg_rd(p, addr + 4),
			reg_rd(p, addr + 8), reg_rd(p, addr + 12));
	}
#endif
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
	struct sprd_cpp_scale_cfg_parm *cfg_parm = &p->cfg_parm;

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
	h_chroma_coeff = tmp_buf + (SC_COEFF_COEF_SIZE / 4);/* offset 64 */
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
				SC_COEFF_TMP_SIZE, p->sc_deci_val, false, 0))) {
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

	reg_mwr(p, CPP_SC_TAP, CPP_SCALE_Y_VER_TAP, ((y_tap & 0x0F) << 5));
	reg_mwr(p, CPP_SC_TAP, CPP_SCALE_UV_VER_TAP, ((uv_tap & 0x1F)));

	CPP_LOG("y_tap %d uv_tap %d\n", y_tap, uv_tap);
	cpp_print_coeff(p, h_coeff_addr, h_chroma_coeff_addr, v_coeff_addr);

	return 0;
}

static int scale_k_cfg_scaler(struct scale_drv_private *p)
{
	int ret = 0;

	if (!p)
		return -EINVAL;

	ret = scale_k_calc_sc_size(p);
	if (ret)
		return -EINVAL;

	if (p->sc_input_size.w != p->cfg_parm.output_size.w ||
	    p->sc_input_size.h != p->cfg_parm.output_size.h ||
	    p->cfg_parm.input_format == SCALE_YUV420) {
		ret = scale_k_set_sc_coeff(p);
		if (ret)
			return -EINVAL;
	}

	return ret;
}
#if 1
static void cpp_reg_trace(struct scale_drv_private *p)
{
#if 1 /*def SCALE_DRV_DEBUG*/
	unsigned long addr = 0;

	CPP_LOG("CPP scale: Register list");
	for (addr = CPP_BASE; addr <= CPP_END; addr += 16) {
		CPP_LOG("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			reg_rd(p, addr), reg_rd(p, addr + 4),
			reg_rd(p, addr + 8), reg_rd(p, addr + 12));
	}
#endif
}
#endif

void get_cpp_max_size(unsigned int *max_width, unsigned int *max_height)
{
	*max_width = SCALE_FRAME_WIDTH_MAX;
	*max_height = SCALE_FRAME_HEIGHT_MAX;
}

int cpp_scale_start(struct sprd_cpp_scale_cfg_parm *parm,
		  struct scale_drv_private *p)
{
	int ret = -1;

	if (!parm || !p)
		return -EINVAL;
	CPP_LOG("cpp_scale_start s\n");
	memset(&p->sc_input_size, 0, sizeof(struct sprd_cpp_size));
	p->slice_in_height = 0;
	p->slice_out_height = 0;
	p->sc_deci_val = 0;

	memcpy((void *)&p->cfg_parm, (void *)parm,
	       sizeof(struct sprd_cpp_scale_cfg_parm));

	scale_k_ahb_reset(p);
	scale_k_enable(p);
	scale_k_set_input_size(p);
	scale_k_set_input_format(p);
#ifdef CPP_TEST_DRIVER
	scale_k_set_src_addr_t32_load_image(p);
#else
	if (scale_k_set_src_addr(p)) {
		pr_err("set src addr failed");
		return -1;
	}
#endif
	scale_k_set_input_endian(p);
	scale_k_set_output_size(p);
	scale_k_set_output_format(p);
#ifdef CPP_TEST_DRIVER
	scale_k_set_dst_addr_t32_load_image(p);
#else
	if (scale_k_set_dst_addr(p)) {
		pr_err("set dst addr failed");
		return -1;
	}
#endif
	scale_k_set_output_endian(p);

	if (parm->input_format == SCALE_JPEG_LS)
		pr_err("input format SCALE_JPEG_LS is not supported on iwhale2!\n");
		/*scale_k_set_jpegls_infor(p);*/

	reg_mwr(p, CPP_PATH0_CFG0, CPP_SCALE_CLK_SWITCH, CPP_SCALE_CLK_SWITCH);

	ret = scale_k_cfg_scaler(p);
	if (ret) {
		pr_err("failed to start scaler\n");
		return ret;
	}
	scale_k_set_input_rect(p);
	CPP_LOG("in_size%d %d in_rect%d %d %d %d out_size%d %d sc_deci_val%d\n",
		p->cfg_parm.input_size.w, p->cfg_parm.input_size.h,
		p->cfg_parm.input_rect.x, p->cfg_parm.input_rect.y,
		p->cfg_parm.input_rect.w, p->cfg_parm.input_rect.h,
		p->cfg_parm.output_size.w, p->cfg_parm.output_size.h,
		p->sc_deci_val);
	CPP_LOG("in_addr 0x%x 0x%x out_addr 0x%x 0x%x\n",
		p->cfg_parm.input_addr.y, p->cfg_parm.input_addr.u,
		p->cfg_parm.output_addr.y, p->cfg_parm.output_addr.u);

	reg_awr(p, CPP_PATH0_CFG0, ~CPP_SCALE_CLK_SWITCH);
	udelay(1);

	reg_owr(p, CPP_PATH_START, CPP_SCALE_START_BIT);
	CPP_LOG("cpp_scale_start e\n");
	cpp_reg_trace(p);
	udelay(1);

	return 0;
}

void cpp_scale_stop(struct scale_drv_private *p)
{
	reg_awr(p, CPP_PATH_START, (~CPP_SCALE_START_BIT));
	scale_k_disable(p);
	cpp_free_addr(&p->iommu_src, SPRD_IOMMU_FM_CH_RW);
	cpp_free_addr(&p->iommu_dst, SPRD_IOMMU_FM_CH_RW);
	CPP_LOG("cpp_scale_stop end\n");
}
