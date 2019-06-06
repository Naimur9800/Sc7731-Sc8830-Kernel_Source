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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <video/gsp_r3p0_cfg.h>
#include "../gsp_core.h"
#include "../gsp_kcfg.h"
#include "../gsp_debug.h"
#include "../gsp_dev.h"
#include "../gsp_workqueue.h"
#include "../gsp_layer.h"
#include "gsp_r3p0_core.h"
#include "gsp_r3p0_reg.h"
#include "gsp_r3p0_coef_cal.h"
#include "../gsp_interface.h"
#include "../gsp_interface/gsp_interface_whale2.h"

static int gsp_r3p0_core_cfg_is_err(struct gsp_core *core)
{
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod1_cfg_value;

	gsp_mod1_cfg_value.value =
		gsp_core_reg_read(R3P0_GSP_MOD1_CFG(core->base));
	return gsp_mod1_cfg_value.ERR1_FLG;
}

static void gsp_r3p0_core_err_code_print(struct gsp_core *core)
{
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod1_cfg_value;

	gsp_mod1_cfg_value.value =
		gsp_core_reg_read(R3P0_GSP_MOD1_CFG(core->base));

	GSP_ERR("register configure error code: %d.\n",
		gsp_mod1_cfg_value.ERR1_CODE);
}

static void print_image_layer_cfg(struct gsp_r3p0_img_layer *layer)
{
	struct gsp_r3p0_img_layer_params *params = NULL;

	GSP_DEBUG("\ngsp_r3p0 image layer configure information begin\n");

	params = &layer->params;
	gsp_layer_common_print(&layer->common);

	GSP_DEBUG("pitch: %d, clip_rect: [%d, %d, %d, %d]\n",
		  params->pitch, params->clip_rect.st_x, params->clip_rect.st_y,
		  params->clip_rect.rect_w, params->clip_rect.rect_h);

	GSP_DEBUG("des_rect: [%d, %d, %d, %d],  grey: [%d, %d, %d, %d]\n",
		  params->des_rect.st_x, params->des_rect.st_y,
		  params->des_rect.rect_w, params->des_rect.rect_h,
		  params->grey.r_val, params->grey.g_val, params->grey.b_val,
		  params->grey.a_val);

	GSP_DEBUG("colorkey: [%d, %d, %d, %d], endian: [%d, %d, %d, %d, %d]\n",
		  params->colorkey.r_val, params->colorkey.g_val,
		  params->colorkey.b_val, params->colorkey.a_val,
		  params->endian.y_rgb_word_endn, params->endian.uv_word_endn,
		  params->endian.va_word_endn, params->endian.rgb_swap_mode,
		  params->endian.a_swap_mode);

	GSP_DEBUG("img_format: %d\n", params->img_format);

	GSP_DEBUG("alpha: %d, colorkey_en: %d, pallet_en: %d\n",
		  params->alpha, params->colorkey_en, params->pallet_en);

	GSP_DEBUG("scaling_en: %d, pmargb_en: %d, pmargb_mod: %d\n",
		  params->scaling_en, params->pmargb_en, params->pmargb_mod);

	GSP_DEBUG("gsp_r3p0 image layer configure information end\n");
}

static void print_osd_layer_cfg(struct gsp_r3p0_osd_layer *layer)
{
	struct gsp_r3p0_osd_layer_params *params = NULL;

	GSP_DEBUG("\ngsp_r3p0 osd layer configure information begin\n");

	params = &layer->params;
	gsp_layer_common_print(&layer->common);

	GSP_DEBUG("pitch: %d, clip_rect: [%d, %d, %d, %d]\n",
		  params->pitch, params->clip_rect.st_x, params->clip_rect.st_y,
		  params->clip_rect.rect_w, params->clip_rect.rect_h);

	GSP_DEBUG("des_pos: [%d, %d],  grey: [%d, %d, %d, %d]\n",
		  params->des_pos.pt_x, params->des_pos.pt_y,
		  params->grey.r_val, params->grey.g_val, params->grey.b_val,
		  params->grey.a_val);

	GSP_DEBUG("colorkey: [%d, %d, %d, %d], endian: [%d, %d, %d, %d, %d]\n",
		  params->colorkey.r_val, params->colorkey.g_val,
		  params->colorkey.b_val, params->colorkey.a_val,
		  params->endian.y_rgb_word_endn, params->endian.uv_word_endn,
		  params->endian.va_word_endn, params->endian.rgb_swap_mode,
		  params->endian.a_swap_mode);

	GSP_DEBUG("osd_format: %d\n", params->osd_format);

	GSP_DEBUG("alpha: %d, colorkey_en: %d, pallet_en: %d\n",
		  params->alpha, params->colorkey_en, params->pallet_en);

	GSP_DEBUG("pmargb_en: %d, pmargb_mod: %d\n",
		  params->pmargb_en, params->pmargb_mod);

	GSP_DEBUG("gsp_r3p0 osd layer configure information end\n");
}

static void print_des_layer_cfg(struct gsp_r3p0_des_layer *layer)
{
	struct gsp_r3p0_des_layer_params *params = NULL;

	GSP_DEBUG("\ngsp_r3p0 des layer configure information begin\n");

	params = &layer->params;
	gsp_layer_common_print(&layer->common);

	GSP_DEBUG("pitch: %d, height: %d, endian: [%d, %d, %d, %d, %d]\n",
		  params->pitch, params->height,
		  params->endian.y_rgb_word_endn, params->endian.uv_word_endn,
		  params->endian.va_word_endn, params->endian.rgb_swap_mode,
		  params->endian.a_swap_mode);

	GSP_DEBUG("img_format: %d, compress_r8_en: %d\n",
		  params->img_format, params->compress_r8_en);

	GSP_DEBUG("gsp_r3p0 des layer configure information end\n");
}

static void gsp_r3p0_core_cfg_print(struct gsp_r3p0_cfg *cfg)
{
	print_image_layer_cfg(&cfg->l0);
	print_osd_layer_cfg(&cfg->l1);
	print_osd_layer_cfg(&cfg->l2);
	print_osd_layer_cfg(&cfg->l3);
	print_des_layer_cfg(&cfg->ld1);
	print_des_layer_cfg(&cfg->ld2);
}

void gsp_r3p0_core_dump(struct gsp_core *c)
{
	struct R3P0_GSP_CTL_REG_T reg_struct;

	reg_struct.glb_cfg.value = gsp_core_reg_read(R3P0_GSP_GLB_CFG(c->base));
	reg_struct.mod1_cfg.value =
		gsp_core_reg_read(R3P0_GSP_MOD1_CFG(c->base));
	reg_struct.mod2_cfg.value =
		gsp_core_reg_read(R3P0_GSP_MOD2_CFG(c->base));
	reg_struct.cmd_sts0.value =
		gsp_core_reg_read(R3P0_GSP_CMD_STS0(c->base));
	reg_struct.cmd_sts1.value =
		gsp_core_reg_read(R3P0_GSP_CMD_STS1(c->base));
	reg_struct.cmd_addr.value =
	gsp_core_reg_read(R3P0_GSP_CMD_ADDR(c->base));

	reg_struct.des1_data_cfg.value =
		 gsp_core_reg_read(R3P0_DES_DATA_CFG1(c->base));
	reg_struct.des1_y_addr.value =
		 gsp_core_reg_read(R3P0_DES_Y_ADDR1(c->base));
	reg_struct.des1_u_addr.value =
		 gsp_core_reg_read(R3P0_DES_U_ADDR1(c->base));
	reg_struct.des1_v_addr.value =
		 gsp_core_reg_read(R3P0_DES_V_ADDR1(c->base));
	reg_struct.des1_pitch.value =
		 gsp_core_reg_read(R3P0_DES_PITCH1(c->base));
	reg_struct.bg_rgb.value =
		 gsp_core_reg_read(R3P0_BACK_RGB(c->base));
	reg_struct.des_scl_size.value =
		 gsp_core_reg_read(R3P0_DES_SCL_SIZE(c->base));
	reg_struct.work_des_start.value =
		 gsp_core_reg_read(R3P0_WORK_DES_START(c->base));
	reg_struct.work_src_size.value =
		 gsp_core_reg_read(R3P0_WORK_SCR_SIZE(c->base));
	reg_struct.work_src_start.value =
		 gsp_core_reg_read(R3P0_WORK_SCR_START(c->base));

	reg_struct.des2_data_cfg.value =
		 gsp_core_reg_read(R3P0_DES_DATA_CFG2(c->base));
	reg_struct.des2_y_addr.value =
		 gsp_core_reg_read(R3P0_DES_Y_ADDR2(c->base));
	reg_struct.des2_u_addr.value =
		 gsp_core_reg_read(R3P0_DES_U_ADDR2(c->base));
	reg_struct.des2_v_addr.value =
		 gsp_core_reg_read(R3P0_DES_V_ADDR2(c->base));
	reg_struct.des2_pitch.value =
		 gsp_core_reg_read(R3P0_DES_PITCH2(c->base));
	reg_struct.axi_debug.value =
		 gsp_core_reg_read(R3P0_AXI_DEBUG(c->base));
	reg_struct.int_ctl.value =
		 gsp_core_reg_read(R3P0_GSP_INT(c->base));

	reg_struct.l0_cfg.value =
		 gsp_core_reg_read(R3P0_LAYER0_CFG(c->base));
	reg_struct.l0_y_addr.value =
		 gsp_core_reg_read(R3P0_LAYER0_Y_ADDR(c->base));
	reg_struct.l0_u_addr.value =
		 gsp_core_reg_read(R3P0_LAYER0_U_ADDR(c->base));
	reg_struct.l0_v_addr.value =
		 gsp_core_reg_read(R3P0_LAYER0_V_ADDR(c->base));
	reg_struct.l0_pitch.value =
		 gsp_core_reg_read(R3P0_LAYER0_PITCH(c->base));
	reg_struct.l0_clip_start.value =
		 gsp_core_reg_read(R3P0_LAYER0_CLIP_START(c->base));
	reg_struct.l0_clip_size.value =
		 gsp_core_reg_read(R3P0_LAYER0_CLIP_SIZE(c->base));
	reg_struct.l0_des_start.value =
		 gsp_core_reg_read(R3P0_LAYER0_DES_START(c->base));
	reg_struct.l0_pallet_rgb.value =
		 gsp_core_reg_read(R3P0_LAYER0_PALLET_RGB(c->base));
	reg_struct.l0_ck.value =
		 gsp_core_reg_read(R3P0_LAYER0_CK(c->base));
	reg_struct.y2r_y_param.value =
		 gsp_core_reg_read(R3P0_Y2R_Y_PARAM(c->base));
	reg_struct.y2r_u_param.value =
		 gsp_core_reg_read(R3P0_Y2R_U_PARAM(c->base));
	reg_struct.y2r_v_param.value =
		 gsp_core_reg_read(R3P0_Y2R_V_PARAM(c->base));

	reg_struct.l1_cfg.value =
		 gsp_core_reg_read(R3P0_LAYER1_CFG(c->base));
	reg_struct.l1_r_addr.value =
		 gsp_core_reg_read(R3P0_LAYER1_R_ADDR(c->base));
	reg_struct.l1_pitch.value =
		 gsp_core_reg_read(R3P0_LAYER1_PITCH(c->base));
	reg_struct.l1_clip_start.value =
		 gsp_core_reg_read(R3P0_LAYER1_CLIP_START(c->base));
	reg_struct.l1_clip_size.value =
		 gsp_core_reg_read(R3P0_LAYER1_CLIP_SIZE(c->base));
	reg_struct.l1_des_start.value =
		 gsp_core_reg_read(R3P0_LAYER1_DES_START(c->base));
	reg_struct.l1_pallet_rgb.value =
		 gsp_core_reg_read(R3P0_LAYER1_PALLET_RGB(c->base));
	reg_struct.l1_ck.value =
		 gsp_core_reg_read(R3P0_LAYER1_CK(c->base));

	reg_struct.l2_cfg.value =
		 gsp_core_reg_read(R3P0_LAYER2_CFG(c->base));
	reg_struct.l2_r_addr.value =
		 gsp_core_reg_read(R3P0_LAYER2_R_ADDR(c->base));
	reg_struct.l2_pitch.value =
		 gsp_core_reg_read(R3P0_LAYER2_PITCH(c->base));
	reg_struct.l2_clip_start.value =
		 gsp_core_reg_read(R3P0_LAYER2_CLIP_START(c->base));
	reg_struct.l2_clip_size.value =
		 gsp_core_reg_read(R3P0_LAYER2_CLIP_SIZE(c->base));
	reg_struct.l2_des_start.value =
		 gsp_core_reg_read(R3P0_LAYER2_DES_START(c->base));
	reg_struct.l2_pallet_rgb.value =
		 gsp_core_reg_read(R3P0_LAYER2_PALLET_RGB(c->base));
	reg_struct.l2_ck.value =
		 gsp_core_reg_read(R3P0_LAYER2_CK(c->base));

	reg_struct.l3_cfg.value =
		 gsp_core_reg_read(R3P0_LAYER3_CFG(c->base));
	reg_struct.l3_r_addr.value =
		 gsp_core_reg_read(R3P0_LAYER3_R_ADDR(c->base));
	reg_struct.l3_pitch.value =
		 gsp_core_reg_read(R3P0_LAYER3_PITCH(c->base));
	reg_struct.l3_clip_start.value =
		 gsp_core_reg_read(R3P0_LAYER3_CLIP_START(c->base));
	reg_struct.l3_clip_size.value =
		 gsp_core_reg_read(R3P0_LAYER3_CLIP_SIZE(c->base));
	reg_struct.l3_des_start.value =
		 gsp_core_reg_read(R3P0_LAYER3_DES_START(c->base));
	reg_struct.l3_pallet_rgb.value =
		 gsp_core_reg_read(R3P0_LAYER3_PALLET_RGB(c->base));
	reg_struct.l3_ck.value =
		 gsp_core_reg_read(R3P0_LAYER3_CK(c->base));

	/*core's ctl reg parsed*/
	GSP_INFO("run_mode%d,scale_seq%d,cmd_en%d",
				reg_struct.glb_cfg.RUN_MOD,
				reg_struct.mod1_cfg.SCALE_SEQ,
				reg_struct.glb_cfg.CMD_EN);
	if (reg_struct.glb_cfg.CMD_EN) {
		GSP_INFO(",up_m1%d,up_d%d,up_l3%d,up_l2%d,up_l1%d,up_l0%d,",
			reg_struct.cmd_sts0.NEW_M1,
			reg_struct.cmd_sts0.NEW_D,
			reg_struct.cmd_sts0.NEW_L3,
			reg_struct.cmd_sts0.NEW_L2,
			reg_struct.cmd_sts0.NEW_L1,
			reg_struct.cmd_sts0.NEW_L0);
		GSP_INFO("up_coef%d,stopc%d,cnt%d\n",
			reg_struct.cmd_sts0.NEW_C,
			reg_struct.cmd_sts0.STOPC,
			reg_struct.cmd_sts0.STAT_CMD_NUM);
	} else {
		GSP_INFO("\n");
	}

	if (reg_struct.glb_cfg.RUN_MOD == 0) {
		GSP_INFO("mod1: bk_en%d,pm_en%d,scl_en%d,dither_en%d,",
			reg_struct.mod1_cfg.BK_EN,
			reg_struct.mod1_cfg.PMARGB_EN,
			reg_struct.mod1_cfg.SCALE_EN,
			reg_struct.mod1_cfg.DITHER1_EN);
		GSP_INFO("scl_seq%d,l3_en%d,l2_en%d,l1_en%d,l0_en%d\n",
			reg_struct.mod1_cfg.SCALE_SEQ,
			reg_struct.mod1_cfg.L3_EN,
			reg_struct.mod1_cfg.L2_EN,
			reg_struct.mod1_cfg.L1_EN,
			reg_struct.mod1_cfg.L0_EN);
		GSP_INFO("mod1: err_flg%d,err_code%d,",
			reg_struct.mod1_cfg.ERR1_FLG,
			reg_struct.mod1_cfg.ERR1_CODE);
		GSP_INFO("w_busy%d,r_busy%d,bld_busy%d, bld_run: %d\n",
			reg_struct.mod1_cfg.WCH_BUSY,
			reg_struct.mod1_cfg.RCH_BUSY,
			reg_struct.mod1_cfg.BLD_BUSY,
			reg_struct.mod1_cfg.BLD_RUN);
		GSP_INFO("seq0 des1:fmt%d,compress%d,%s,rot%d,",
			reg_struct.des1_data_cfg.DES_IMG_FORMAT1,
			reg_struct.des1_data_cfg.COMPRESS_R8,
			reg_struct.des1_data_cfg.R2Y_MOD1 ? "reduce" : "full",
			reg_struct.des1_data_cfg.ROT_MOD1);
		GSP_INFO("{pitch%dx%d (%d,%d)[%dx%d]->(%d,%d)",
			reg_struct.des1_pitch.DES_PITCH1,
			reg_struct.des1_pitch.DES_HEIGHT1,
			reg_struct.work_src_start.WORK_SCR_X,
			reg_struct.work_src_start.WORK_SCR_Y,
			reg_struct.work_src_size.WORK_SCR_W,
			reg_struct.work_src_size.WORK_SCR_H,
			reg_struct.work_des_start.WORK_DES_X,
			reg_struct.work_des_start.WORK_DES_Y);
		if (reg_struct.mod1_cfg.SCALE_SEQ == 0)
			GSP_INFO("}\n");
		else
			GSP_INFO("[%dx%d]}\n",
				reg_struct.des_scl_size.DES_SCL_W,
				reg_struct.des_scl_size.DES_SCL_H);

		/*layer0 info*/
		if (reg_struct.mod1_cfg.L0_EN) {
			GSP_INFO("L0:fmt%d,y2y%d,%s,zorder%d ",
				reg_struct.l0_cfg.IMG_FORMAT0,
				reg_struct.l0_cfg.Y2R_MOD0,
				(reg_struct.l0_cfg.Y2R_MOD0 & 1) ? "reduce"
					: "full",
				reg_struct.l0_cfg.LAYER_NUM);
			GSP_INFO("{pitch%04d (%d,%d)[%dx%d]->(%d,%d)}\n",
				reg_struct.l0_pitch.PITCH0,
				reg_struct.l0_clip_start.CLIP_START_X0,
				reg_struct.l0_clip_start.CLIP_START_Y0,
				reg_struct.l0_clip_size.CLIP_SIZE_X0,
				reg_struct.l0_clip_size.CLIP_SIZE_Y0,
				reg_struct.l0_des_start.DES_START_X0,
				reg_struct.l0_des_start.DES_START_Y0);
		}

		/*layer1 info*/
		if (reg_struct.mod1_cfg.L1_EN) {
			GSP_INFO("L1:fmt%d,",
				reg_struct.l1_cfg.IMG_FORMAT1);
			GSP_INFO("{pitch%04d (%d,%d)[%dx%d]->(%d,%d)}\n",
				reg_struct.l1_pitch.PITCH1,
				reg_struct.l1_clip_start.CLIP_START_X1,
				reg_struct.l1_clip_start.CLIP_START_Y1,
				reg_struct.l1_clip_size.CLIP_SIZE_X1,
				reg_struct.l1_clip_size.CLIP_SIZE_Y1,
				reg_struct.l1_des_start.DES_START_X1,
				reg_struct.l1_des_start.DES_START_Y1);
		}

		/*layer2 info*/
		if (reg_struct.mod1_cfg.L2_EN) {
			GSP_INFO("L2:fmt%d,",
				reg_struct.l2_cfg.IMG_FORMAT2);
			GSP_INFO("{pitch%04d (%d,%d)[%dx%d]->(%d,%d)}\n",
				reg_struct.l2_pitch.PITCH2,
				reg_struct.l2_clip_start.CLIP_START_X2,
				reg_struct.l2_clip_start.CLIP_START_Y2,
				reg_struct.l2_clip_size.CLIP_SIZE_X2,
				reg_struct.l2_clip_size.CLIP_SIZE_Y2,
				reg_struct.l2_des_start.DES_START_X2,
				reg_struct.l2_des_start.DES_START_Y2);
		}
		/*layer3 info*/
		if (reg_struct.mod1_cfg.L3_EN) {
			GSP_INFO("L3:fmt%d,",
				reg_struct.l3_cfg.IMG_FORMAT3);
			GSP_INFO("{pitch%04d (%d,%d)[%dx%d]->(%d,%d)}\n",
				reg_struct.l3_pitch.PITCH3,
				reg_struct.l3_clip_start.CLIP_START_X3,
				reg_struct.l3_clip_start.CLIP_START_Y3,
				reg_struct.l3_clip_size.CLIP_SIZE_X3,
				reg_struct.l3_clip_size.CLIP_SIZE_Y3,
				reg_struct.l3_des_start.DES_START_X3,
				reg_struct.l3_des_start.DES_START_Y3);
		}
	} else {
		GSP_INFO("GSP r3p0 run_mode=1 not implemented yet !\n");
	}

}

static void gsp_r3p0_core_cfg_reinit(struct gsp_r3p0_cfg *cfg)
{
	struct gsp_layer *layer;
	struct gsp_kcfg *kcfg = NULL;

	if (IS_ERR_OR_NULL(cfg)) {
		GSP_ERR("cfg init params error\n");
		return;
	}

	kcfg = cfg->common.kcfg;
	GSP_DEBUG("gsp_r3p0 cfg parent kcfg[%d]\n", gsp_kcfg_to_tag(kcfg));
	if (cfg->common.init != 1) {
		GSP_ERR("gsp_r3p0 cfg[%d] has not been initialized\n",
			cfg->common.tag);
		return;
	}

	/* first to reset layer common attributes*/
	list_for_each_entry(layer, &cfg->common.layers, list) {
		layer->type = -1;
		layer->enable = -1;
		layer->wait_fd = -1;
		layer->sig_fd = -1;
		memset(&layer->src_addr, 0, sizeof(struct gsp_addr_data));
		memset(&layer->mem_data, 0, sizeof(struct gsp_mem_data));
	}
	GSP_DEBUG("gsp layer common reinit done\n");

	/* second to reset layer params*/
	memset(&cfg->l0.params, 0, sizeof(struct gsp_r3p0_img_layer_params));
	memset(&cfg->l1.params, 0, sizeof(struct gsp_r3p0_osd_layer_params));
	memset(&cfg->l2.params, 0, sizeof(struct gsp_r3p0_osd_layer_params));
	memset(&cfg->l3.params, 0, sizeof(struct gsp_r3p0_osd_layer_params));
	memset(&cfg->ld1.params, 0, sizeof(struct gsp_r3p0_des_layer_params));
	memset(&cfg->ld2.params, 0, sizeof(struct gsp_r3p0_des_layer_params));
}


static int gsp_r3p0_core_capa_init(struct gsp_core *core)
{
	struct gsp_r3p0_capability *capa = NULL;
	struct gsp_rect max = {0, 0, 8191, 8191};
	struct gsp_rect min = {0, 0, 4, 4};

	capa = (struct gsp_r3p0_capability *)core->capa;

	/* common information initialize */
	capa->common.magic = GSP_CAPABILITY_MAGIC;
	strcpy(capa->common.version, "R3P0");
	capa->common.capa_size = sizeof(struct gsp_r3p0_capability);

	capa->common.max_layer = 4;
	capa->common.max_img_layer = 1;

	capa->common.crop_max = max;
	capa->common.crop_min = min;
	capa->common.out_max = max;
	capa->common.out_min = min;

	capa->common.buf_type = GSP_ADDR_TYPE_IOVIRTUAL;

	/* private information initialize*/
	capa->scale_range_up = 256;
	capa->scale_range_down = 1;
	capa->yuv_xywh_even = 1;
	capa->scale_updown_sametime = 1;
	capa->OSD_scaling = 0;
	capa->blend_video_with_OSD = 1;
	capa->max_yuvLayer_cnt = 1;
	capa->max_scaleLayer_cnt = 1;
	capa->seq0_scale_range_up = 256;
	capa->seq0_scale_range_down = 1;
	capa->seq1_scale_range_up = 64;
	capa->seq1_scale_range_down = 4;
	capa->src_yuv_xywh_even_limit = 1;
	capa->max_video_size = 2;
	capa->csc_matrix_in = 0x3;
	capa->csc_matrix_out = 0x3;

	capa->block_alpha_limit = 0;
	capa->max_throughput = 256;

	capa->max_gspmmu_size = 80 * 1024 * 1024;
	capa->max_gsp_bandwidth = 1920 * 1080 * 4 * 5 / 2;

	return 0;
}

static void gsp_r3p0_int_clear_and_disable(struct gsp_core *core)
{
	struct gsp_kcfg *kcfg = NULL;
	struct gsp_r3p0_cfg *cmd = NULL;
	struct R3P0_GSP_INT_REG gsp_int_value;
	struct R3P0_GSP_INT_REG gsp_int_mask;

	kcfg = core->current_kcfg;
	cmd = (struct gsp_r3p0_cfg *)kcfg->cfg;

	if (core == NULL) {
		GSP_ERR("r3p0 interrupt clear with null core\n");
		return;
	}

	if (cmd->ld1.common.enable) {
		gsp_int_value.value = 0;
		gsp_int_value.INT_BLD_EN = 0;
		gsp_int_value.INT_BERR_EN = 0;
		gsp_int_value.INT_BLD_CLR = 1;
		gsp_int_value.INT_BERR_CLR = 1;
		gsp_int_mask.value = 0;
		gsp_int_mask.INT_BLD_EN = 1;
		gsp_int_mask.INT_BERR_EN = 1;
		gsp_int_mask.INT_BLD_CLR = 1;
		gsp_int_mask.INT_BERR_CLR = 1;
		gsp_core_reg_update(R3P0_GSP_INT(core->base),
				gsp_int_value.value, gsp_int_mask.value);
	}

	if (cmd->ld2.common.enable) {
		gsp_int_value.value = 0;
		gsp_int_value.INT_BLD_EN = 0;
		gsp_int_value.INT_BERR_EN = 0;
		gsp_int_value.INT_SCL_EN = 0;
		gsp_int_value.INT_SERR_EN = 0;
		gsp_int_value.INT_BLD_CLR = 1;
		gsp_int_value.INT_BERR_CLR = 1;
		gsp_int_value.INT_SCL_CLR = 1;
		gsp_int_value.INT_SERR_CLR = 1;
		gsp_int_mask.value = 0;
		gsp_int_mask.INT_BLD_EN = 1;
		gsp_int_mask.INT_BERR_EN = 1;
		gsp_int_mask.INT_SCL_EN = 1;
		gsp_int_mask.INT_SERR_EN = 1;
		gsp_int_mask.INT_BLD_CLR = 1;
		gsp_int_mask.INT_BERR_CLR = 1;
		gsp_int_mask.INT_SCL_CLR = 1;
		gsp_int_mask.INT_SERR_CLR = 1;
		gsp_core_reg_update(R3P0_GSP_INT(core->base),
				gsp_int_value.value, gsp_int_mask.value);
	}
}

static void gsp_r3p0_coef_cache_init(struct gsp_r3p0_core *core)
{
	uint32_t i = 0;

	if (core->cache_coef_init_flag == 0) {
		i = 0;
		INIT_LIST_HEAD(&core->coef_list);
		while (i < R3P0_GSP_COEF_CACHE_MAX) {
			list_add_tail(&core->coef_cache[i].list,
				&core->coef_list);
			i++;
		}
		core->cache_coef_init_flag = 1;
	}
}

static void gsp_r3p0_core_cfg_init(struct gsp_r3p0_cfg *cfg,
					struct gsp_kcfg *kcfg)
{
	/* to work around ERROR: do not initialise statics to 0 or NULL */
	static int tag = 1;

	if (IS_ERR_OR_NULL(cfg)) {
		GSP_ERR("cfg init params error\n");
		return;
	}

	INIT_LIST_HEAD(&cfg->common.layers);
	INIT_LIST_HEAD(&cfg->l0.common.list);
	INIT_LIST_HEAD(&cfg->l1.common.list);
	INIT_LIST_HEAD(&cfg->l2.common.list);
	INIT_LIST_HEAD(&cfg->l3.common.list);
	INIT_LIST_HEAD(&cfg->ld1.common.list);
	INIT_LIST_HEAD(&cfg->ld2.common.list);

	cfg->common.layer_num = 4;
	cfg->common.init = 1;
	cfg->common.kcfg = kcfg;
	cfg->common.tag = tag++ - 1;

	list_add_tail(&cfg->l0.common.list, &cfg->common.layers);
	list_add_tail(&cfg->l1.common.list, &cfg->common.layers);
	list_add_tail(&cfg->l2.common.list, &cfg->common.layers);
	list_add_tail(&cfg->l3.common.list, &cfg->common.layers);
	list_add_tail(&cfg->ld1.common.list, &cfg->common.layers);
	list_add_tail(&cfg->ld2.common.list, &cfg->common.layers);

	GSP_INFO("gsp_r3p0 cfg[%d] initialized done\n", cfg->common.tag);
}

int gsp_r3p0_core_init(struct gsp_core *core)
{
	int ret = 0;
	struct gsp_kcfg *kcfg = NULL;
	struct gsp_r3p0_core *c = (struct gsp_r3p0_core *)core;

	if (gsp_core_verify(core)) {
		GSP_ERR("core init params error");
		ret = -1;
		return ret;
	}

	gsp_r3p0_core_capa_init(core);

	gsp_r3p0_coef_cache_init(c);

	/* to do some specific initialize operations here */
	list_for_each_entry(kcfg, &core->kcfgs, sibling) {
		gsp_r3p0_core_cfg_init((struct gsp_r3p0_cfg *)kcfg->cfg, kcfg);
	}

	return ret;
}

int gsp_r3p0_core_alloc(struct gsp_core **core, struct device_node *node)
{
	struct gsp_capability *capa = NULL;
	struct platform_device *pdev = NULL;

	if (IS_ERR_OR_NULL(core)) {
		GSP_ERR("r3p0 core alloc params error\n");
		return -1;
	}

	/* device utility has been registered at of_device_add() */
	pdev = of_find_device_by_node(node);
	if (IS_ERR_OR_NULL(pdev)) {
		GSP_ERR("find gsp_r2p0 device by child node failed\n");
		return -1;
	}
	*core = devm_kzalloc(&pdev->dev, sizeof(struct gsp_r3p0_core),
						GFP_KERNEL);
	if (IS_ERR_OR_NULL(*core)) {
		GSP_ERR("no enough memory for core alloc\n");
		return -1;
	}

	memset(*core, 0, sizeof(struct gsp_r3p0_core));
	(*core)->cfg_size = sizeof(struct gsp_r3p0_cfg);
	(*core)->dev = &pdev->dev;
	if (dev_name(&pdev->dev))
		GSP_INFO("core[%d] device name: %s\n", (*core)->id,
			 dev_name(&pdev->dev));

	capa = kzalloc(sizeof(struct gsp_r3p0_capability), GFP_KERNEL);
	if (IS_ERR_OR_NULL(capa)) {
		GSP_ERR("gsp_r3p0 capability memory allocated failed\n");
		kfree(*core);
		return -1;
	}
	(*core)->capa = capa;
	(*core)->capa->capa_size = sizeof(struct gsp_r3p0_capability);
	(*core)->node = node;
	GSP_ERR("gsp_r3p0_cap_size = %d\n", (int)(*core)->capa->capa_size);
	GSP_INFO("gsp_r3p0 core allocate success\n");
	return 0;
}


void gsp_r3p0_core_irq_enable(struct gsp_core *core)
{
	struct R3P0_GSP_INT_REG gsp_int_value;
	struct R3P0_GSP_INT_REG gsp_int_mask;

	gsp_int_value.value = 0;
	gsp_int_value.INT_BLD_EN = 1;
	gsp_int_value.INT_SCL_EN = 1;
	gsp_int_value.INT_ROT_EN = 1;
	gsp_int_value.INT_BERR_EN = 1;
	gsp_int_value.INT_SERR_EN = 1;
	gsp_int_mask.value = 0;
	gsp_int_mask.INT_BLD_EN = 0x1;
	gsp_int_mask.INT_SCL_EN = 0x1;
	gsp_int_mask.INT_ROT_EN = 0x1;
	gsp_int_mask.INT_BERR_EN = 0x1;
	gsp_int_mask.INT_SERR_EN = 0x1;
	gsp_core_reg_update(R3P0_GSP_INT(core->base),
			gsp_int_value.value, gsp_int_mask.value);
}

void gsp_r3p0_core_irq_disable(struct gsp_core *core)
{
	struct R3P0_GSP_INT_REG gsp_int_value;
	struct R3P0_GSP_INT_REG gsp_int_mask;

	gsp_int_value.value = 0;
	gsp_int_value.INT_BLD_EN = 0;
	gsp_int_value.INT_SCL_EN = 0;
	gsp_int_value.INT_ROT_EN = 0;
	gsp_int_value.INT_BERR_EN = 0;
	gsp_int_value.INT_SERR_EN = 0;
	gsp_int_mask.value = 0;
	gsp_int_mask.INT_BLD_EN = 0x1;
	gsp_int_mask.INT_SCL_EN = 0x1;
	gsp_int_mask.INT_ROT_EN = 0x1;
	gsp_int_mask.INT_BERR_EN = 0x1;
	gsp_int_mask.INT_SERR_EN = 0x1;
	gsp_core_reg_update(R3P0_GSP_INT(core->base),
			gsp_int_value.value, gsp_int_mask.value);
}

irqreturn_t gsp_r3p0_core_irq_handler(int irq, void *data)
{
	struct gsp_core *core = NULL;

	core = (struct gsp_core *)data;
	if (gsp_core_verify(core)) {
		GSP_ERR("core irq handler not match\n");
		return IRQ_NONE;
	}

	gsp_r3p0_int_clear_and_disable(core);

	gsp_core_state_set(core, CORE_STATE_IRQ_HANDLED);
	queue_kthread_work(&core->kworker, &core->release);

	return IRQ_HANDLED;
}

int gsp_r3p0_core_enable(struct gsp_core *c)
{
	int ret = -1;
	struct gsp_r3p0_core *core = NULL;

	core = (struct gsp_r3p0_core *)c;

	clk_set_parent(core->gsp_clk, NULL);
	ret = clk_set_parent(core->gsp_clk, core->gsp_clk_parent);
	if (ret) {
		GSP_ERR("select GSP clk fail !\n");
		goto exit;
	}

	ret = clk_prepare_enable(core->gsp_clk);
	if (ret) {
		GSP_ERR("enable core[%d] gsp_clk failed\n",
			gsp_core_to_id(c));
		goto gsp_clk_disable;
	}

	ret = clk_prepare_enable(core->gsp_mmu_eb_clk);
	if (ret) {
		GSP_ERR("enable core[%d] gsp_mmu_eb failed\n",
			gsp_core_to_id(c));
		goto gsp_mmu_eb_clk_disable;
	}

	ret = clk_prepare_enable(core->gsp_eb_clk);
	if (ret) {
		GSP_ERR("enable core[%d] gsp_eb clk failed\n",
			gsp_core_to_id(c));
		goto gsp_eb_clk_disable;
	}

	ret = clk_prepare_enable(core->gsp_force_gate_clk);
	if (ret) {
		GSP_ERR("enable core[%d] gsp_force_gate clk failed\n",
			gsp_core_to_id(c));
		goto gsp_force_en_clk_disable;
	}

	gsp_r3p0_core_irq_enable(c);
	goto exit;

gsp_force_en_clk_disable:
	clk_disable_unprepare(core->gsp_force_gate_clk);
gsp_eb_clk_disable:
	clk_disable_unprepare(core->gsp_eb_clk);
gsp_mmu_eb_clk_disable:
	clk_disable_unprepare(core->gsp_mmu_eb_clk);
gsp_clk_disable:
	clk_disable_unprepare(core->gsp_clk);

exit:
	return ret;
}

void gsp_r3p0_core_disable(struct gsp_core *c)
{
	struct gsp_r3p0_core *core = NULL;

	core = (struct gsp_r3p0_core *)c;
	gsp_r3p0_core_irq_disable(c);

	clk_disable_unprepare(core->gsp_force_gate_clk);
	clk_disable_unprepare(core->gsp_eb_clk);
	clk_disable_unprepare(core->gsp_mmu_eb_clk);
	clk_disable_unprepare(core->gsp_clk);
}

int gsp_r3p0_core_parse_clk(struct gsp_r3p0_core *core)
{
	int status = 0;

	if (core->common.id == 0) {
		core->gsp_clk = of_clk_get_by_name(core->common.node,
				R3P0_GSP0_CLOCK_NAME);
		core->gsp_mmu_eb_clk = of_clk_get_by_name(core->common.node,
				R3P0_GSP0_MMU_EB_CLOCK_NAME);
		core->gsp_eb_clk = of_clk_get_by_name(core->common.node,
				R3P0_GSP0_EB_CLOCK_NAME);
		core->gsp_force_gate_clk = of_clk_get_by_name(core->common.node,
				R3P0_GSP0_FORCE_GATE_CLOCK_NAME);
	} else {
		core->gsp_clk = of_clk_get_by_name(core->common.node,
				R3P0_GSP1_CLOCK_NAME);
		core->gsp_mmu_eb_clk = of_clk_get_by_name(core->common.node,
				R3P0_GSP1_MMU_EB_CLOCK_NAME);
		core->gsp_eb_clk = of_clk_get_by_name(core->common.node,
				R3P0_GSP1_EB_CLOCK_NAME);
		core->gsp_force_gate_clk = of_clk_get_by_name(core->common.node,
				R3P0_GSP1_FORCE_GATE_CLOCK_NAME);
	}
	core->gsp_clk_parent = of_clk_get_by_name(core->common.node,
				R3P0_GSP_CLOCK_PARENT);

	if (IS_ERR_OR_NULL(core->gsp_clk)
		|| IS_ERR_OR_NULL(core->gsp_clk_parent)) {
		GSP_ERR("core[%d] parse gsp clk failed\n", core->common.id);
		status = -1;
	}
	if (IS_ERR_OR_NULL(core->gsp_mmu_eb_clk)) {
		GSP_ERR("core[%d] parse mmu_eb_clk failed\n", core->common.id);
		status |= -1;
	}
	if (IS_ERR_OR_NULL(core->gsp_eb_clk)) {
		GSP_ERR("core[%d] parse enable clk failed\n", core->common.id);
		status |= -1;
	}
	if (IS_ERR_OR_NULL(core->gsp_force_gate_clk)) {
		GSP_ERR("core[%d] parse force gate failed\n", core->common.id);
		status |= -1;
	}

	return status;
}

int gsp_r3p0_core_parse_irq(struct gsp_core *core)
{
	int ret = -1;
	struct device *dev = NULL;

	dev = gsp_core_to_device(core);
	core->irq = irq_of_parse_and_map(core->node, 0);

	if (core->irq)
		ret = devm_request_irq(dev, core->irq,
				gsp_r3p0_core_irq_handler,
				IRQF_SHARED, "GSP0", core);

	if (ret)
		GSP_ERR("r3p0 core[%d] request irq failed!\n", core->id);

	return ret;
}

int gsp_r3p0_core_parse_dt(struct gsp_core *core)
{
	int ret = -1;
	struct device *dev = NULL;
	struct gsp_r3p0_core *r3p0_core = NULL;

	dev = container_of(&core->node, struct device, of_node);
	r3p0_core = (struct gsp_r3p0_core *)core;

	r3p0_core->gsp_ctl_reg_base = core->base;
	r3p0_core->disp_ahb =
			syscon_regmap_lookup_by_compatible("sprd,sys-disp-ahb");
	GSP_INFO("gsp_ctl_reg_base: 0x%p\n", core->base);

	ret = gsp_r3p0_core_parse_irq(core);
	if (ret) {
		GSP_ERR("core[%d] parse irq failed\n", core->id);
		return ret;
	}

	gsp_r3p0_core_parse_clk(r3p0_core);

	return ret;
}


void gsp_r3p0_core_misc_reg_set(struct gsp_core *core, struct gsp_r3p0_cfg *cfg)
{
	void __iomem *base = NULL;
	struct R3P0_DES_SCL_SIZE_REG des_scl_size_value;
	struct R3P0_DES_SCL_SIZE_REG des_scl_size_mask;
	struct R3P0_WORK_DES_START_REG work_des_start_value;
	struct R3P0_WORK_DES_START_REG work_des_start_mask;
	struct R3P0_WORK_SCR_SIZE_REG  work_src_size_value;
	struct R3P0_WORK_SCR_SIZE_REG work_src_size_mask;
	struct R3P0_WORK_SCR_START_REG work_src_start_value;
	struct R3P0_WORK_SCR_START_REG work_src_start_mask;
	struct R3P0_GSP_GLB_CFG_REG  gsp_glb_cfg_value;
	struct R3P0_GSP_GLB_CFG_REG  gsp_glb_cfg_mask;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod1_cfg_value;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod1_cfg_mask;

	base = core->base;

	des_scl_size_value.value = 0;
	des_scl_size_value.DES_SCL_W =
		cfg->misc.scale_para.scale_rect_out.rect_w;
	des_scl_size_value.DES_SCL_H =
		cfg->misc.scale_para.scale_rect_out.rect_h;
	des_scl_size_mask.value = 0;
	des_scl_size_mask.DES_SCL_W = 0x1FFF;
	des_scl_size_mask.DES_SCL_H = 0x1FFF;
	gsp_core_reg_update(R3P0_DES_SCL_SIZE(base),
		des_scl_size_value.value, des_scl_size_mask.value);

	work_src_start_value.value = 0;
	work_src_start_value.WORK_SCR_X = cfg->misc.workarea_src_rect.st_x;
	work_src_start_value.WORK_SCR_Y = cfg->misc.workarea_src_rect.st_y;
	work_src_start_mask.value = 0;
	work_src_start_mask.WORK_SCR_X = 0x1FFF;
	work_src_start_mask.WORK_SCR_Y = 0x1FFF;
	gsp_core_reg_update(R3P0_WORK_SCR_START(base),
		work_src_start_value.value, work_src_start_mask.value);

	work_src_size_value.value = 0;
	work_src_size_value.WORK_SCR_W =
		cfg->misc.workarea_src_rect.rect_w;
	work_src_size_value.WORK_SCR_H =
		cfg->misc.workarea_src_rect.rect_h;
	work_src_size_mask.value = 0;
	work_src_size_mask.WORK_SCR_W = 0x1FFF;
	work_src_size_mask.WORK_SCR_H = 0x1FFF;
	gsp_core_reg_update(R3P0_WORK_SCR_SIZE(base),
		work_src_size_value.value, work_src_size_mask.value);

	work_des_start_value.value = 0;
	work_des_start_value.WORK_DES_X = cfg->misc.workarea_dst_rect.st_x;
	work_des_start_value.WORK_DES_Y = cfg->misc.workarea_dst_rect.st_y;
	work_des_start_mask.value = 0;
	work_des_start_mask.WORK_DES_X = 0x1FFF;
	work_des_start_mask.WORK_DES_Y = 0x1FFF;
	gsp_core_reg_update(R3P0_WORK_DES_START(base),
		work_des_start_value.value, work_des_start_mask.value);

	gsp_glb_cfg_value.value = 0;
	switch (cfg->misc.scale_para.htap_mod) {
	case 8:
		gsp_glb_cfg_value.HTAP_MOD = 0;
		break;
	case 6:
		gsp_glb_cfg_value.HTAP_MOD = 1;
		break;
	case 4:
		gsp_glb_cfg_value.HTAP_MOD = 2;
		break;
	case 2:
		gsp_glb_cfg_value.HTAP_MOD = 3;
		break;
	default:
		break;
	}
	switch (cfg->misc.scale_para.vtap_mod) {
	case 4:
		gsp_glb_cfg_value.VTAP_MOD = 0;
		break;
	case 2:
		gsp_glb_cfg_value.VTAP_MOD = 1;
		break;
	default:
		break;
	}
	gsp_glb_cfg_value.RUN_MOD = cfg->misc.run_mod;
	gsp_glb_cfg_mask.value = 0;
	gsp_glb_cfg_mask.HTAP_MOD = 0x3;
	gsp_glb_cfg_mask.VTAP_MOD = 0x1;
	gsp_glb_cfg_mask.RUN_MOD = 0x1;
	gsp_core_reg_update(R3P0_GSP_GLB_CFG(base),
		gsp_glb_cfg_value.value, gsp_glb_cfg_mask.value);

	gsp_mod1_cfg_value.value = 0;
	gsp_mod1_cfg_value.SCALE_EN = cfg->misc.scale_para.scale_en;
	gsp_mod1_cfg_value.SCALE_SEQ = cfg->misc.scale_seq;
	gsp_mod1_cfg_mask.value = 0;
	gsp_mod1_cfg_mask.SCALE_EN = 0x1;
	gsp_mod1_cfg_mask.SCALE_SEQ = 0x1;
	gsp_core_reg_update(R3P0_GSP_MOD1_CFG(base),
		gsp_mod1_cfg_value.value, gsp_mod1_cfg_mask.value);
}

void gsp_r3p0_core_l0_reg_set(void __iomem *base,
			   struct gsp_r3p0_img_layer *layer)
{
	struct R3P0_LAYER0_Y_ADDR_REG layer0_y_addr_value;
	struct R3P0_LAYER0_Y_ADDR_REG layer0_y_addr_mask;
	struct R3P0_LAYER0_U_ADDR_REG layer0_u_addr_value;
	struct R3P0_LAYER0_U_ADDR_REG layer0_u_addr_mask;
	struct R3P0_LAYER0_V_ADDR_REG layer0_v_addr_value;
	struct R3P0_LAYER0_V_ADDR_REG layer0_v_addr_mask;
	struct R3P0_LAYER0_PITCH_REG layer0_pitch_value;
	struct R3P0_LAYER0_PITCH_REG layer0_pitch_mask;
	struct R3P0_LAYER0_CLIP_SIZE_REG layer0_clip_size_value;
	struct R3P0_LAYER0_CLIP_SIZE_REG layer0_clip_size_mask;
	struct R3P0_LAYER0_CLIP_START_REG layer0_clip_start_value;
	struct R3P0_LAYER0_CLIP_START_REG layer0_clip_start_mask;
	struct R3P0_LAYER0_DES_START_REG layer0_des_start_value;
	struct R3P0_LAYER0_DES_START_REG layer0_des_start_mask;
	struct R3P0_LAYER0_CK_REG layer0_ck_value;
	struct R3P0_LAYER0_CK_REG layer0_ck_mask;
	struct R3P0_LAYER0_CFG_REG layer0_cfg_value;
	struct R3P0_LAYER0_CFG_REG layer0_cfg_mask;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod_cfg_value;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod_cfg_mask;
	struct gsp_r3p0_img_layer_params	*l0_params = NULL;

	if (IS_ERR_OR_NULL(base) || IS_ERR_OR_NULL(layer)) {
		GSP_ERR("layer0 reg set params error\n");
		return;
	}

	l0_params = &layer->params;

	if (layer->common.enable != 1) {
		gsp_mod_cfg_value.value = 0;
		gsp_mod_cfg_value.L0_EN = 0;
		gsp_mod_cfg_mask.value = 0;
		gsp_mod_cfg_mask.L0_EN = 0x1;
		gsp_core_reg_update(R3P0_GSP_MOD1_CFG(base),
			gsp_mod_cfg_value.value, gsp_mod_cfg_mask.value);
		GSP_DEBUG("no need to set layer0\n");
		return;
	}

	/* layer0 address set */
	layer0_y_addr_value.value = 0;
	layer0_y_addr_value.Y_BASE_ADDR0 = layer->common.src_addr.addr_y;
	layer0_y_addr_mask.value = 0;
	layer0_y_addr_mask.Y_BASE_ADDR0 = 0xFFFFFFFF;
	if (l0_params->img_format == GSP_R3P0_IMG_FMT_RGB888)
		layer0_y_addr_value.Y_BASE_ADDR0 += 1;

	gsp_core_reg_update(R3P0_LAYER0_Y_ADDR(base),
		layer0_y_addr_value.value, layer0_y_addr_mask.value);

	layer0_u_addr_value.value = 0;
	layer0_u_addr_value.U_BASE_ADDR0 = layer->common.src_addr.addr_uv;
	layer0_u_addr_mask.value = 0;
	layer0_u_addr_mask.U_BASE_ADDR0 = 0xFFFFFFFF;
	gsp_core_reg_update(R3P0_LAYER0_U_ADDR(base),
		layer0_u_addr_value.value, layer0_u_addr_mask.value);

	layer0_v_addr_value.value = 0;
	layer0_v_addr_value.V_BASE_ADDR0 = layer->common.src_addr.addr_va;
	layer0_v_addr_mask.value = 0;
	layer0_v_addr_mask.V_BASE_ADDR0 = 0xFFFFFFFF;
	gsp_core_reg_update(R3P0_LAYER0_V_ADDR(base),
		layer0_v_addr_value.value, layer0_v_addr_mask.value);

	/* layer0 pitch and height set */
	layer0_pitch_value.value = 0;
	layer0_pitch_value.PITCH0 = l0_params->pitch;
	layer0_pitch_mask.value = 0;
	layer0_pitch_mask.PITCH0 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER0_PITCH(base),
		layer0_pitch_value.value, layer0_pitch_mask.value);

	/* layer0 clip rect size set */
	layer0_clip_size_value.value = 0;
	layer0_clip_size_value.CLIP_SIZE_X0 = l0_params->clip_rect.rect_w;
	layer0_clip_size_value.CLIP_SIZE_Y0 = l0_params->clip_rect.rect_h;
	layer0_clip_size_mask.value = 0;
	layer0_clip_size_mask.CLIP_SIZE_X0 = 0x1FFF;
	layer0_clip_size_mask.CLIP_SIZE_Y0 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER0_CLIP_SIZE(base),
		layer0_clip_size_value.value, layer0_clip_size_mask.value);

	/* layer0 clip start position set */
	layer0_clip_start_value.value = 0;
	layer0_clip_start_value.CLIP_START_X0 = l0_params->clip_rect.st_x;
	layer0_clip_start_value.CLIP_START_Y0 = l0_params->clip_rect.st_y;
	layer0_clip_start_mask.value = 0;
	layer0_clip_start_mask.CLIP_START_X0 = 0x1FFF;
	layer0_clip_start_mask.CLIP_START_Y0 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER0_CLIP_START(base),
		layer0_clip_start_value.value, layer0_clip_start_mask.value);

	/* layer0 destination clip start position set*/
	layer0_des_start_value.value = 0;
	layer0_des_start_value.DES_START_X0 = l0_params->des_rect.st_x;
	layer0_des_start_value.DES_START_Y0 = l0_params->des_rect.st_y;
	layer0_des_start_mask.value = 0;
	layer0_des_start_mask.DES_START_X0 = 0x1FFF;
	layer0_des_start_mask.DES_START_Y0 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER0_DES_START(base),
		layer0_des_start_value.value, layer0_des_start_mask.value);

	/* layer0 pallet rgb set */
	if (l0_params->pallet_en) {
		struct R3P0_LAYER0_PALLET_RGB_REG layer0_pallet_value;
		struct R3P0_LAYER0_PALLET_RGB_REG layer0_pallet_mask;

		layer0_pallet_value.value = 0;
		layer0_pallet_value.PALLET_A0 = l0_params->pallet.a_val;
		layer0_pallet_value.PALLET_B0 = l0_params->pallet.b_val;
		layer0_pallet_value.PALLET_G0 = l0_params->pallet.g_val;
		layer0_pallet_value.PALLET_R0 = l0_params->pallet.r_val;
		layer0_pallet_mask.value = 0;
		layer0_pallet_mask.PALLET_A0 = 0xFF;
		layer0_pallet_mask.PALLET_B0 = 0xFF;
		layer0_pallet_mask.PALLET_G0 = 0xFF;
		layer0_pallet_mask.PALLET_R0 = 0xFF;
		gsp_core_reg_update(R3P0_LAYER0_PALLET_RGB(base),
			layer0_pallet_value.value, layer0_pallet_mask.value);
	}

	/* layer0 block alpha set */
	layer0_ck_value.value = 0;
	layer0_ck_value.BLOCK_ALPHA0 = l0_params->alpha;
	layer0_ck_mask.value = 0;
	layer0_ck_mask.BLOCK_ALPHA0 = 0xFF;
	gsp_core_reg_update(R3P0_LAYER0_CK(base),
		layer0_ck_value.value, layer0_ck_mask.value);

	/* layer0 color key set*/
	if (l0_params->colorkey_en) {
		layer0_ck_value.value = 0;
		layer0_ck_value.CK_B0 = l0_params->colorkey.b_val;
		layer0_ck_value.CK_G0 = l0_params->colorkey.g_val;
		layer0_ck_value.CK_R0 = l0_params->colorkey.r_val;
		layer0_ck_mask.value = 0;
		layer0_ck_mask.CK_B0 = 0xFF;
		layer0_ck_mask.CK_G0 = 0xFF;
		layer0_ck_mask.CK_R0 = 0xFF;
		gsp_core_reg_update(R3P0_LAYER0_CK(base),
			layer0_ck_value.value, layer0_ck_mask.value);
	}

	/*  yuv2yuv adjust*/
	if ((l0_params->y2r_mod == 0) || (l0_params->y2r_mod == 1)) {
		struct R3P0_Y2R_Y_PARAM0_REG y2r_y_para_value;
		struct R3P0_Y2R_Y_PARAM0_REG y2r_y_para_mask;
		struct R3P0_Y2R_U_PARAM0_REG y2r_u_para_value;
		struct R3P0_Y2R_U_PARAM0_REG y2r_u_para_mask;
		struct R3P0_Y2R_V_PARAM0_REG y2r_v_para_value;
		struct R3P0_Y2R_V_PARAM0_REG y2r_v_para_mask;

		y2r_y_para_value.value = 0;
		y2r_y_para_value.Y_BRIGHTNESS0 =
			l0_params->yuv_adjust.y_brightness;
		y2r_y_para_value.Y_CONTRAST0 = l0_params->yuv_adjust.y_contrast;
		y2r_y_para_mask.value = 0;
		y2r_y_para_mask.Y_BRIGHTNESS0 = 0x1FF;
		y2r_y_para_mask.Y_CONTRAST0 = 0x3FF;
		gsp_core_reg_update(R3P0_Y2R_Y_PARAM(base),
			y2r_y_para_value.value, y2r_y_para_mask.value);

		y2r_u_para_value.value = 0;
		y2r_u_para_value.U_OFFSET0 = l0_params->yuv_adjust.u_offset;
		y2r_u_para_value.U_SATURATION0 =
			l0_params->yuv_adjust.u_saturation;
		y2r_u_para_mask.value = 0;
		y2r_u_para_mask.U_OFFSET0 = 0xFF;
		y2r_u_para_mask.U_SATURATION0 = 0x3FF;
		gsp_core_reg_update(R3P0_Y2R_U_PARAM(base),
			y2r_u_para_value.value, y2r_u_para_mask.value);

		y2r_v_para_value.value = 0;
		y2r_v_para_value.V_OFFSET0 = l0_params->yuv_adjust.v_offset;
		y2r_v_para_value.V_SATURATION0 =
			l0_params->yuv_adjust.v_saturation;
		y2r_v_para_mask.value = 0;
		y2r_v_para_mask.V_OFFSET0 = 0xFF;
		y2r_v_para_mask.V_SATURATION0 = 0x3FF;
		gsp_core_reg_update(R3P0_Y2R_V_PARAM(base),
			y2r_v_para_value.value, y2r_v_para_mask.value);
	}

	/* layer0 cfg set*/
	layer0_cfg_value.value = 0;
	layer0_cfg_value.Y_ENDIAN_MOD0 = l0_params->endian.y_rgb_word_endn;
	layer0_cfg_value.UV_ENDIAN_MOD0 = l0_params->endian.uv_word_endn;
	layer0_cfg_value.RGB_SWAP_MOD0 = l0_params->endian.rgb_swap_mode;
	layer0_cfg_value.A_SWAP_MOD0 = l0_params->endian.a_swap_mode;
	layer0_cfg_value.IMG_FORMAT0 = l0_params->img_format;
	layer0_cfg_value.CK_EN0 = l0_params->colorkey_en;
	layer0_cfg_value.PALLET_EN0 = l0_params->pallet_en;
	layer0_cfg_value.Y2R_MOD0 = l0_params->y2r_mod;
	layer0_cfg_value.LAYER_NUM = l0_params->zorder;
	layer0_cfg_mask.value = 0;
	layer0_cfg_mask.Y_ENDIAN_MOD0 = 0x7;
	layer0_cfg_mask.UV_ENDIAN_MOD0 = 0x7;
	layer0_cfg_mask.RGB_SWAP_MOD0 = 0x7;
	layer0_cfg_mask.A_SWAP_MOD0 = 0x1;
	layer0_cfg_mask.IMG_FORMAT0 = 0x7;
	layer0_cfg_mask.CK_EN0 = 0x1;
	layer0_cfg_mask.PALLET_EN0 = 0x1;
	layer0_cfg_mask.Y2R_MOD0 = 0xF;
	layer0_cfg_mask.LAYER_NUM = 0x3;
	gsp_core_reg_update(R3P0_LAYER0_CFG(base),
		layer0_cfg_value.value, layer0_cfg_mask.value);

	gsp_mod_cfg_value.value = 0;
	gsp_mod_cfg_value.L0_EN = layer->common.enable;
	gsp_mod_cfg_value.PMARGB_MOD = l0_params->pmargb_mod ? 0x1 : 0;
	gsp_mod_cfg_mask.value = 0;
	gsp_mod_cfg_mask.L0_EN = 0x1;
	gsp_mod_cfg_mask.PMARGB_MOD = 0x1;
	gsp_core_reg_update(R3P0_GSP_MOD1_CFG(base),
		gsp_mod_cfg_value.value, gsp_mod_cfg_mask.value);

}

void gsp_r3p0_core_l1_reg_set(void __iomem *base,
			   struct gsp_r3p0_osd_layer *layer)
{
	struct R3P0_LAYER1_R_ADDR_REG layer1_r_addr_value;
	struct R3P0_LAYER1_R_ADDR_REG layer1_r_addr_mask;
	struct R3P0_LAYER1_PITCH_REG layer1_pitch_value;
	struct R3P0_LAYER1_PITCH_REG layer1_pitch_mask;
	struct R3P0_LAYER1_CLIP_SIZE_REG layer1_clip_size_value;
	struct R3P0_LAYER1_CLIP_SIZE_REG layer1_clip_size_mask;
	struct R3P0_LAYER1_CLIP_START_REG layer1_clip_start_value;
	struct R3P0_LAYER1_CLIP_START_REG layer1_clip_start_mask;
	struct R3P0_LAYER1_DES_START_REG layer1_des_start_value;
	struct R3P0_LAYER1_DES_START_REG layer1_des_start_mask;
	struct R3P0_LAYER1_CFG_REG layer1_cfg_value;
	struct R3P0_LAYER1_CFG_REG layer1_cfg_mask;
	struct R3P0_LAYER1_CK_REG layer1_ck_value;
	struct R3P0_LAYER1_CK_REG layer1_ck_mask;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod_cfg_value;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod_cfg_mask;
	struct gsp_r3p0_osd_layer_params	*l1_params = NULL;


	if (IS_ERR_OR_NULL(base) || IS_ERR_OR_NULL(layer)) {
		GSP_ERR("LAYER1 reg set params error\n");
		return;
	}

	l1_params = &layer->params;

	if (layer->common.enable != 1) {
		gsp_mod_cfg_value.value = 0;
		gsp_mod_cfg_value.L1_EN = 0;
		gsp_mod_cfg_mask.value = 0;
		gsp_mod_cfg_mask.L1_EN = 0x1;
		gsp_core_reg_update(R3P0_GSP_MOD1_CFG(base),
			gsp_mod_cfg_value.value, gsp_mod_cfg_mask.value);
		GSP_DEBUG("no need to set LAYER1\n");
		return;
	}

	/* layer1 address set */
	layer1_r_addr_value.value = 0;
	layer1_r_addr_value.R_BASE_ADDR1 = layer->common.src_addr.addr_y;
	layer1_r_addr_mask.value = 0;
	layer1_r_addr_mask.R_BASE_ADDR1 = 0xFFFFFFFF;
	if (l1_params->osd_format == GSP_R3P0_OSD_FMT_RGB888)
		layer1_r_addr_value.R_BASE_ADDR1 += 1;

	gsp_core_reg_update(R3P0_LAYER1_R_ADDR(base),
		layer1_r_addr_value.value, layer1_r_addr_mask.value);

	/* layer1 pitch set */
	layer1_pitch_value.value = 0;
	layer1_pitch_value.PITCH1 = l1_params->pitch;
	layer1_pitch_mask.value = 0;
	layer1_pitch_mask.PITCH1 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER1_PITCH(base),
		layer1_pitch_value.value, layer1_pitch_mask.value);

	/* layer1 clip rect size set */
	layer1_clip_size_value.value = 0;
	layer1_clip_size_value.CLIP_SIZE_X1 = l1_params->clip_rect.rect_w;
	layer1_clip_size_value.CLIP_SIZE_Y1 = l1_params->clip_rect.rect_h;
	layer1_clip_size_mask.value = 0;
	layer1_clip_size_mask.CLIP_SIZE_X1 = 0x1FFF;
	layer1_clip_size_mask.CLIP_SIZE_Y1 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER1_CLIP_SIZE(base),
		layer1_clip_size_value.value, layer1_clip_size_mask.value);

	/* layer1 clip start position set */
	layer1_clip_start_value.value = 0;
	layer1_clip_start_value.CLIP_START_X1 = l1_params->clip_rect.st_x;
	layer1_clip_start_value.CLIP_START_Y1 = l1_params->clip_rect.st_y;
	layer1_clip_start_mask.value = 0;
	layer1_clip_start_mask.CLIP_START_X1 = 0x1FFF;
	layer1_clip_start_mask.CLIP_START_Y1 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER1_CLIP_START(base),
		layer1_clip_start_value.value, layer1_clip_start_mask.value);

	/* LAYER1 destination start position set*/
	layer1_des_start_value.value = 0;
	layer1_des_start_value.DES_START_X1 = l1_params->des_pos.pt_x;
	layer1_des_start_value.DES_START_Y1 = l1_params->des_pos.pt_y;
	layer1_des_start_mask.value = 0;
	layer1_des_start_mask.DES_START_X1 = 0x1FFF;
	layer1_des_start_mask.DES_START_Y1 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER1_DES_START(base),
		layer1_des_start_value.value, layer1_des_start_mask.value);

	/* layer1 pallet set */
	if (l1_params->pallet_en) {
		struct R3P0_LAYER1_PALLET_RGB_REG layer1_pallet_value;
		struct R3P0_LAYER1_PALLET_RGB_REG layer1_pallet_mask;

		layer1_pallet_value.value = 0;
		layer1_pallet_value.PALLET_A1 = l1_params->pallet.a_val;
		layer1_pallet_value.PALLET_B1 = l1_params->pallet.b_val;
		layer1_pallet_value.PALLET_G1 = l1_params->pallet.g_val;
		layer1_pallet_value.PALLET_R1 = l1_params->pallet.r_val;
		layer1_pallet_mask.value = 0;
		layer1_pallet_mask.PALLET_A1 = 0xFF;
		layer1_pallet_mask.PALLET_B1 = 0xFF;
		layer1_pallet_mask.PALLET_G1 = 0xFF;
		layer1_pallet_mask.PALLET_R1 = 0xFF;
		gsp_core_reg_update(R3P0_LAYER1_PALLET_RGB(base),
			layer1_pallet_value.value, layer1_pallet_mask.value);
	}

	/* LAYER1 alpha set */
	layer1_ck_value.value = 0;
	layer1_ck_value.BLOCK_ALPHA1 = l1_params->alpha;
	layer1_ck_mask.value = 0;
	layer1_ck_mask.BLOCK_ALPHA1 = 0xFF;
	gsp_core_reg_update(R3P0_LAYER1_CK(base),
		layer1_ck_value.value, layer1_ck_mask.value);

	/* LAYER1 color key set*/
	if (l1_params->colorkey_en) {
		layer1_ck_value.value = 0;
		layer1_ck_value.CK_B1 = l1_params->colorkey.b_val;
		layer1_ck_value.CK_G1 = l1_params->colorkey.g_val;
		layer1_ck_value.CK_R1 = l1_params->colorkey.r_val;
		layer1_ck_mask.value = 0;
		layer1_ck_mask.CK_B1 = 0xFF;
		layer1_ck_mask.CK_G1 = 0xFF;
		layer1_ck_mask.CK_R1 = 0xFF;
		gsp_core_reg_update(R3P0_LAYER1_CK(base),
			layer1_ck_value.value, layer1_ck_mask.value);
	}

	/* layer1 cfg  */
	layer1_cfg_value.value = 0;
	layer1_cfg_value.ENDIAN1 = l1_params->endian.y_rgb_word_endn;
	layer1_cfg_value.RGB_SWAP1 = l1_params->endian.rgb_swap_mode;
	layer1_cfg_value.A_RGB_SWAP1 = l1_params->endian.a_swap_mode;
	layer1_cfg_value.IMG_FORMAT1 = l1_params->osd_format;
	layer1_cfg_value.CK_EN1 = l1_params->colorkey_en;
	layer1_cfg_value.PALLET_EN1 = l1_params->pallet_en;
	layer1_cfg_mask.value = 0;
	layer1_cfg_mask.ENDIAN1 = 0xF;
	layer1_cfg_mask.RGB_SWAP1 = 0x7;
	layer1_cfg_mask.A_RGB_SWAP1 = 0x1;
	layer1_cfg_mask.IMG_FORMAT1 = 0x3;
	layer1_cfg_mask.CK_EN1 = 0x1;
	layer1_cfg_mask.PALLET_EN1 = 0x1;
	gsp_core_reg_update(R3P0_LAYER1_CFG(base),
		layer1_cfg_value.value, layer1_cfg_mask.value);

	gsp_mod_cfg_value.value = 0;
	gsp_mod_cfg_value.L1_EN = layer->common.enable;
	gsp_mod_cfg_value.PMARGB_MOD = l1_params->pmargb_mod ? 0x2 : 0;
	gsp_mod_cfg_mask.value = 0;
	gsp_mod_cfg_mask.L1_EN = 0x1;
	gsp_mod_cfg_mask.PMARGB_MOD = 0x2;
	gsp_core_reg_update(R3P0_GSP_MOD1_CFG(base),
		gsp_mod_cfg_value.value, gsp_mod_cfg_mask.value);

}

void gsp_r3p0_core_l2_reg_set(void __iomem *base,
			   struct gsp_r3p0_osd_layer *layer)
{
	struct R3P0_LAYER2_R_ADDR_REG layer2_r_addr_value;
	struct R3P0_LAYER2_R_ADDR_REG layer2_r_addr_mask;
	struct R3P0_LAYER2_PITCH_REG layer2_pitch_value;
	struct R3P0_LAYER2_PITCH_REG layer2_pitch_mask;
	struct R3P0_LAYER2_CLIP_SIZE_REG layer2_clip_size_value;
	struct R3P0_LAYER2_CLIP_SIZE_REG layer2_clip_size_mask;
	struct R3P0_LAYER2_CLIP_START_REG layer2_clip_start_value;
	struct R3P0_LAYER2_CLIP_START_REG layer2_clip_start_mask;
	struct R3P0_LAYER2_DES_START_REG layer2_des_start_value;
	struct R3P0_LAYER2_DES_START_REG layer2_des_start_mask;
	struct R3P0_LAYER2_CFG_REG layer2_cfg_value;
	struct R3P0_LAYER2_CFG_REG layer2_cfg_mask;
	struct R3P0_LAYER2_CK_REG layer2_ck_value;
	struct R3P0_LAYER2_CK_REG layer2_ck_mask;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod_cfg_value;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod_cfg_mask;
	struct gsp_r3p0_osd_layer_params	*l2_params = NULL;


	if (IS_ERR_OR_NULL(base) || IS_ERR_OR_NULL(layer)) {
		GSP_ERR("LAYER2 reg set params error\n");
		return;
	}

	l2_params = &layer->params;

	if (layer->common.enable != 1) {
		gsp_mod_cfg_value.value = 0;
		gsp_mod_cfg_value.L2_EN = 0;
		gsp_mod_cfg_mask.value = 0;
		gsp_mod_cfg_mask.L2_EN = 0x1;
		gsp_core_reg_update(R3P0_GSP_MOD1_CFG(base),
			gsp_mod_cfg_value.value, gsp_mod_cfg_mask.value);
		GSP_DEBUG("no need to set LAYER2\n");
		return;
	}

	/* layer2 address set */
	layer2_r_addr_value.value = 0;
	layer2_r_addr_value.R_BASE_ADDR2 = layer->common.src_addr.addr_y;
	layer2_r_addr_mask.value = 0;
	layer2_r_addr_mask.R_BASE_ADDR2 = 0xFFFFFFFF;
	if (l2_params->osd_format == GSP_R3P0_OSD_FMT_RGB888)
		layer2_r_addr_value.R_BASE_ADDR2 += 1;

	gsp_core_reg_update(R3P0_LAYER2_R_ADDR(base),
		layer2_r_addr_value.value, layer2_r_addr_mask.value);

	/* layer2 pitch set */
	layer2_pitch_value.value = 0;
	layer2_pitch_value.PITCH2 = l2_params->pitch;
	layer2_pitch_mask.value = 0;
	layer2_pitch_mask.PITCH2 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER2_PITCH(base),
		layer2_pitch_value.value, layer2_pitch_mask.value);

	/* layer2 clip rect size set */
	layer2_clip_size_value.value = 0;
	layer2_clip_size_value.CLIP_SIZE_X2 = l2_params->clip_rect.rect_w;
	layer2_clip_size_value.CLIP_SIZE_Y2 = l2_params->clip_rect.rect_h;
	layer2_clip_size_mask.value = 0;
	layer2_clip_size_mask.CLIP_SIZE_X2 = 0x1FFF;
	layer2_clip_size_mask.CLIP_SIZE_Y2 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER2_CLIP_SIZE(base),
		layer2_clip_size_value.value, layer2_clip_size_mask.value);

	/* layer2 clip start position set */
	layer2_clip_start_value.value = 0;
	layer2_clip_start_value.CLIP_START_X2 = l2_params->clip_rect.st_x;
	layer2_clip_start_value.CLIP_START_Y2 = l2_params->clip_rect.st_y;
	layer2_clip_start_mask.value = 0;
	layer2_clip_start_mask.CLIP_START_X2 = 0x1FFF;
	layer2_clip_start_mask.CLIP_START_Y2 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER2_CLIP_START(base),
		layer2_clip_start_value.value, layer2_clip_start_mask.value);

	/* LAYER2 destination start position set*/
	layer2_des_start_value.value = 0;
	layer2_des_start_value.DES_START_X2 = l2_params->des_pos.pt_x;
	layer2_des_start_value.DES_START_Y2 = l2_params->des_pos.pt_y;
	layer2_des_start_mask.value = 0;
	layer2_des_start_mask.DES_START_X2 = 0x1FFF;
	layer2_des_start_mask.DES_START_Y2 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER2_DES_START(base),
		layer2_des_start_value.value, layer2_des_start_mask.value);

	/* layer2 pallet set */
	if (l2_params->pallet_en) {
		struct R3P0_LAYER2_PALLET_RGB_REG layer2_pallet_value;
		struct R3P0_LAYER2_PALLET_RGB_REG layer2_pallet_mask;

		layer2_pallet_value.value = 0;
		layer2_pallet_value.PALLET_A2 = l2_params->pallet.a_val;
		layer2_pallet_value.PALLET_B2 = l2_params->pallet.b_val;
		layer2_pallet_value.PALLET_G2 = l2_params->pallet.g_val;
		layer2_pallet_value.PALLET_R2 = l2_params->pallet.r_val;
		layer2_pallet_mask.value = 0;
		layer2_pallet_mask.PALLET_A2 = 0xFF;
		layer2_pallet_mask.PALLET_B2 = 0xFF;
		layer2_pallet_mask.PALLET_G2 = 0xFF;
		layer2_pallet_mask.PALLET_R2 = 0xFF;
		gsp_core_reg_update(R3P0_LAYER2_PALLET_RGB(base),
			layer2_pallet_value.value, layer2_pallet_mask.value);
	}

	/* LAYER2 alpha set */
	layer2_ck_value.value = 0;
	layer2_ck_value.BLOCK_ALPHA2 = l2_params->alpha;
	layer2_ck_mask.value = 0;
	layer2_ck_mask.BLOCK_ALPHA2 = 0xFF;
	gsp_core_reg_update(R3P0_LAYER2_CK(base),
		layer2_ck_value.value, layer2_ck_mask.value);

	/* LAYER2 color key set*/
	if (l2_params->colorkey_en) {
		layer2_ck_value.value = 0;
		layer2_ck_value.CK_B2 = l2_params->colorkey.b_val;
		layer2_ck_value.CK_G2 = l2_params->colorkey.g_val;
		layer2_ck_value.CK_R2 = l2_params->colorkey.r_val;
		layer2_ck_mask.value = 0;
		layer2_ck_mask.CK_B2 = 0xFF;
		layer2_ck_mask.CK_G2 = 0xFF;
		layer2_ck_mask.CK_R2 = 0xFF;
		gsp_core_reg_update(R3P0_LAYER2_CK(base),
			layer2_ck_value.value, layer2_ck_mask.value);
	}

	/* layer2 cfg  */
	layer2_cfg_value.value = 0;
	layer2_cfg_value.ENDIAN2 = l2_params->endian.y_rgb_word_endn;
	layer2_cfg_value.RGB_SWAP2 = l2_params->endian.rgb_swap_mode;
	layer2_cfg_value.A_RGB_SWAP2 = l2_params->endian.a_swap_mode;
	layer2_cfg_value.IMG_FORMAT2 = l2_params->osd_format;
	layer2_cfg_value.CK_EN2 = l2_params->colorkey_en;
	layer2_cfg_value.PALLET_EN2 = l2_params->pallet_en;
	layer2_cfg_mask.value = 0;
	layer2_cfg_mask.ENDIAN2 = 0xF;
	layer2_cfg_mask.RGB_SWAP2 = 0x7;
	layer2_cfg_mask.A_RGB_SWAP2 = 0x1;
	layer2_cfg_mask.IMG_FORMAT2 = 0x3;
	layer2_cfg_mask.CK_EN2 = 0x1;
	layer2_cfg_mask.PALLET_EN2 = 0x1;
	gsp_core_reg_update(R3P0_LAYER2_CFG(base),
		layer2_cfg_value.value, layer2_cfg_mask.value);

	gsp_mod_cfg_value.value = 0;
	gsp_mod_cfg_value.L2_EN = layer->common.enable;
	gsp_mod_cfg_value.PMARGB_MOD = l2_params->pmargb_mod ? 0x4 : 0;
	gsp_mod_cfg_mask.value = 0;
	gsp_mod_cfg_mask.L2_EN = 0x1;
	gsp_mod_cfg_mask.PMARGB_MOD = 0x4;
	gsp_core_reg_update(R3P0_GSP_MOD1_CFG(base),
		gsp_mod_cfg_value.value, gsp_mod_cfg_mask.value);

}

void gsp_r3p0_core_l3_reg_set(void __iomem *base,
			   struct gsp_r3p0_osd_layer *layer)
{
	struct R3P0_LAYER3_R_ADDR_REG layer3_r_addr_value;
	struct R3P0_LAYER3_R_ADDR_REG layer3_r_addr_mask;
	struct R3P0_LAYER3_PITCH_REG layer3_pitch_value;
	struct R3P0_LAYER3_PITCH_REG layer3_pitch_mask;
	struct R3P0_LAYER3_CLIP_SIZE_REG layer3_clip_size_value;
	struct R3P0_LAYER3_CLIP_SIZE_REG layer3_clip_size_mask;
	struct R3P0_LAYER3_CLIP_START_REG layer3_clip_start_value;
	struct R3P0_LAYER3_CLIP_START_REG layer3_clip_start_mask;
	struct R3P0_LAYER3_DES_START_REG layer3_des_start_value;
	struct R3P0_LAYER3_DES_START_REG layer3_des_start_mask;
	struct R3P0_LAYER3_CFG_REG layer3_cfg_value;
	struct R3P0_LAYER3_CFG_REG layer3_cfg_mask;
	struct R3P0_LAYER3_CK_REG layer3_ck_value;
	struct R3P0_LAYER3_CK_REG layer3_ck_mask;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod_cfg_value;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod_cfg_mask;
	struct gsp_r3p0_osd_layer_params	*l3_params = NULL;


	if (IS_ERR_OR_NULL(base) || IS_ERR_OR_NULL(layer)) {
		GSP_ERR("LAYER3 reg set params error\n");
		return;
	}

	l3_params = &layer->params;

	if (layer->common.enable != 1) {
		gsp_mod_cfg_value.value = 0;
		gsp_mod_cfg_value.L3_EN = 0;
		gsp_mod_cfg_mask.value = 0;
		gsp_mod_cfg_mask.L3_EN = 0x1;
		gsp_core_reg_update(R3P0_GSP_MOD1_CFG(base),
			gsp_mod_cfg_value.value, gsp_mod_cfg_mask.value);
		GSP_DEBUG("no need to set LAYER3\n");
		return;
	}

	/* layer3 address set */
	layer3_r_addr_value.value = 0;
	layer3_r_addr_value.R_BASE_ADDR3 = layer->common.src_addr.addr_y;
	layer3_r_addr_mask.value = 0;
	layer3_r_addr_mask.R_BASE_ADDR3 = 0xFFFFFFFF;
	if (l3_params->osd_format == GSP_R3P0_OSD_FMT_RGB888)
		layer3_r_addr_value.R_BASE_ADDR3 += 1;

	gsp_core_reg_update(R3P0_LAYER3_R_ADDR(base),
		layer3_r_addr_value.value, layer3_r_addr_mask.value);

	/* layer3 pitch set */
	layer3_pitch_value.value = 0;
	layer3_pitch_value.PITCH3 = l3_params->pitch;
	layer3_pitch_mask.value = 0;
	layer3_pitch_mask.PITCH3 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER3_PITCH(base),
		layer3_pitch_value.value, layer3_pitch_mask.value);

	/* layer3 clip rect size set */
	layer3_clip_size_value.value = 0;
	layer3_clip_size_value.CLIP_SIZE_X3 = l3_params->clip_rect.rect_w;
	layer3_clip_size_value.CLIP_SIZE_Y3 = l3_params->clip_rect.rect_h;
	layer3_clip_size_mask.value = 0;
	layer3_clip_size_mask.CLIP_SIZE_X3 = 0x1FFF;
	layer3_clip_size_mask.CLIP_SIZE_Y3 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER3_CLIP_SIZE(base),
		layer3_clip_size_value.value, layer3_clip_size_mask.value);

	/* layer3 clip start position set */
	layer3_clip_start_value.value = 0;
	layer3_clip_start_value.CLIP_START_X3 = l3_params->clip_rect.st_x;
	layer3_clip_start_value.CLIP_START_Y3 = l3_params->clip_rect.st_y;
	layer3_clip_start_mask.value = 0;
	layer3_clip_start_mask.CLIP_START_X3 = 0x1FFF;
	layer3_clip_start_mask.CLIP_START_Y3 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER3_CLIP_START(base),
		layer3_clip_start_value.value, layer3_clip_start_mask.value);

	/* LAYER3 destination start position set*/
	layer3_des_start_value.value = 0;
	layer3_des_start_value.DES_START_X3 = l3_params->des_pos.pt_x;
	layer3_des_start_value.DES_START_Y3 = l3_params->des_pos.pt_y;
	layer3_des_start_mask.value = 0;
	layer3_des_start_mask.DES_START_X3 = 0x1FFF;
	layer3_des_start_mask.DES_START_Y3 = 0x1FFF;
	gsp_core_reg_update(R3P0_LAYER3_DES_START(base),
		layer3_des_start_value.value, layer3_des_start_mask.value);

	/* layer3 pallet set */
	if (l3_params->pallet_en) {
		struct R3P0_LAYER3_PALLET_RGB_REG layer3_pallet_value;
		struct R3P0_LAYER3_PALLET_RGB_REG layer3_pallet_mask;

		layer3_pallet_value.value = 0;
		layer3_pallet_value.PALLET_A3 = l3_params->pallet.a_val;
		layer3_pallet_value.PALLET_B3 = l3_params->pallet.b_val;
		layer3_pallet_value.PALLET_G3 = l3_params->pallet.g_val;
		layer3_pallet_value.PALLET_R3 = l3_params->pallet.r_val;
		layer3_pallet_mask.value = 0;
		layer3_pallet_mask.PALLET_A3 = 0xFF;
		layer3_pallet_mask.PALLET_B3 = 0xFF;
		layer3_pallet_mask.PALLET_G3 = 0xFF;
		layer3_pallet_mask.PALLET_R3 = 0xFF;
		gsp_core_reg_update(R3P0_LAYER3_PALLET_RGB(base),
			layer3_pallet_value.value, layer3_pallet_mask.value);
	}

	/* LAYER3 alpha set */
	layer3_ck_value.value = 0;
	layer3_ck_value.BLOCK_ALPHA3 = l3_params->alpha;
	layer3_ck_mask.value = 0;
	layer3_ck_mask.BLOCK_ALPHA3 = 0xFF;
	gsp_core_reg_update(R3P0_LAYER3_CK(base),
		layer3_ck_value.value, layer3_ck_mask.value);

	/* LAYER3 color key set*/
	if (l3_params->colorkey_en) {
		layer3_ck_value.value = 0;
		layer3_ck_value.CK_B3 = l3_params->colorkey.b_val;
		layer3_ck_value.CK_G3 = l3_params->colorkey.g_val;
		layer3_ck_value.CK_R3 = l3_params->colorkey.r_val;
		layer3_ck_mask.value = 0;
		layer3_ck_mask.CK_B3 = 0xFF;
		layer3_ck_mask.CK_G3 = 0xFF;
		layer3_ck_mask.CK_R3 = 0xFF;
		gsp_core_reg_update(R3P0_LAYER3_CK(base),
			layer3_ck_value.value, layer3_ck_mask.value);
	}

	/* layer3 cfg  */
	layer3_cfg_value.value = 0;
	layer3_cfg_value.ENDIAN3 = l3_params->endian.y_rgb_word_endn;
	layer3_cfg_value.RGB_SWAP3 = l3_params->endian.rgb_swap_mode;
	layer3_cfg_value.A_RGB_SWAP3 = l3_params->endian.a_swap_mode;
	layer3_cfg_value.IMG_FORMAT3 = l3_params->osd_format;
	layer3_cfg_value.CK_EN3 = l3_params->colorkey_en;
	layer3_cfg_value.PALLET_EN3 = l3_params->pallet_en;
	layer3_cfg_mask.value = 0;
	layer3_cfg_mask.ENDIAN3 = 0xF;
	layer3_cfg_mask.RGB_SWAP3 = 0x7;
	layer3_cfg_mask.A_RGB_SWAP3 = 0x1;
	layer3_cfg_mask.IMG_FORMAT3 = 0x3;
	layer3_cfg_mask.CK_EN3 = 0x1;
	layer3_cfg_mask.PALLET_EN3 = 0x1;
	gsp_core_reg_update(R3P0_LAYER3_CFG(base),
		layer3_cfg_value.value, layer3_cfg_mask.value);

	gsp_mod_cfg_value.value = 0;
	gsp_mod_cfg_value.L3_EN = layer->common.enable;
	gsp_mod_cfg_value.PMARGB_MOD = l3_params->pmargb_mod ? 0x8 : 0;
	gsp_mod_cfg_mask.value = 0;
	gsp_mod_cfg_mask.L3_EN = 0x1;
	gsp_mod_cfg_mask.PMARGB_MOD = 0x8;
	gsp_core_reg_update(R3P0_GSP_MOD1_CFG(base),
		gsp_mod_cfg_value.value, gsp_mod_cfg_mask.value);

}

void gsp_r3p0_core_ld1_reg_set(void __iomem *base,
			   struct gsp_r3p0_des_layer *layer)
{
	struct R3P0_DES_Y_ADDR1_REG des_y_addr_value;
	struct R3P0_DES_Y_ADDR1_REG des_y_addr_mask;
	struct R3P0_DES_U_ADDR1_REG des_u_addr_value;
	struct R3P0_DES_U_ADDR1_REG des_u_addr_mask;
	struct R3P0_DES_V_ADDR1_REG des_v_addr_value;
	struct R3P0_DES_V_ADDR1_REG des_v_addr_mask;
	struct R3P0_DES_PITCH1_REG des_pitch_value;
	struct R3P0_DES_PITCH1_REG des_pitch_mask;
	struct R3P0_DES_DATA_CFG1_REG des_cfg_value;
	struct R3P0_DES_DATA_CFG1_REG des_cfg_mask;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod1_cfg_value;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod1_cfg_mask;
	struct gsp_r3p0_des_layer_params	*ld1_params = NULL;


	if (IS_ERR_OR_NULL(base) || IS_ERR_OR_NULL(layer)) {
		GSP_ERR("LAYER Dest reg set params error\n");
		return;
	}

	ld1_params = &layer->params;

	if (layer->common.enable != 1)
		GSP_WARN("do no need to set LAYERD ? force set dest layer\n");

	/* dest layer address set */
	des_y_addr_value.value = 0;
	des_y_addr_value.DES_Y_BASE_ADDR1 = layer->common.src_addr.addr_y;
	des_y_addr_mask.value = 0;
	des_y_addr_mask.DES_Y_BASE_ADDR1 = 0xFFFFFFFF;
	gsp_core_reg_update(R3P0_DES_Y_ADDR1(base),
		des_y_addr_value.value, des_y_addr_mask.value);

	des_u_addr_value.value = 0;
	des_u_addr_value.DES_U_BASE_ADDR1 = layer->common.src_addr.addr_uv;
	des_u_addr_mask.value = 0;
	des_u_addr_mask.DES_U_BASE_ADDR1 = 0xFFFFFFFF;
	gsp_core_reg_update(R3P0_DES_U_ADDR1(base),
		des_u_addr_value.value, des_u_addr_mask.value);

	des_v_addr_value.value = 0;
	des_v_addr_value.DES_V_BASE_ADDR1 = layer->common.src_addr.addr_va;
	des_v_addr_mask.value = 0;
	des_v_addr_mask.DES_V_BASE_ADDR1 = 0xFFFFFFFF;
	gsp_core_reg_update(R3P0_DES_V_ADDR1(base),
		des_v_addr_value.value, des_v_addr_mask.value);

	/* layerd pitch set - work plane configure*/
	des_pitch_value.value = 0;
	des_pitch_value.DES_PITCH1 = ld1_params->pitch;
	des_pitch_value.DES_HEIGHT1 = ld1_params->height;
	des_pitch_mask.value = 0;
	des_pitch_mask.DES_PITCH1 = 0x1FFF;
	des_pitch_mask.DES_HEIGHT1 = 0x1FFF;
	gsp_core_reg_update(R3P0_DES_PITCH1(base),
		des_pitch_value.value, des_pitch_mask.value);

	if (ld1_params->bk_para.bk_enable) {
		struct R3P0_BACK_RGB_REG back_rgb_value;
		struct R3P0_BACK_RGB_REG back_rgb_mask;

		back_rgb_value.value = 0;
		back_rgb_value.BACKGROUND_A =
			ld1_params->bk_para.background_rgb.a_val;
		back_rgb_value.BACKGROUND_B =
			ld1_params->bk_para.background_rgb.b_val;
		back_rgb_value.BACKGROUND_G =
			ld1_params->bk_para.background_rgb.g_val;
		back_rgb_value.BACKGROUND_R =
			ld1_params->bk_para.background_rgb.r_val;
		back_rgb_mask.value = 0;
		back_rgb_mask.BACKGROUND_A = 0xFF;
		back_rgb_mask.BACKGROUND_B = 0xFF;
		back_rgb_mask.BACKGROUND_G = 0xFF;
		back_rgb_mask.BACKGROUND_R = 0xFF;
		gsp_core_reg_update(R3P0_BACK_RGB(base),
			back_rgb_value.value, back_rgb_mask.value);
	}

	/* layerd cfg set */
	des_cfg_value.value = 0;
	des_cfg_value.Y_ENDIAN_MOD1 = ld1_params->endian.y_rgb_word_endn;
	des_cfg_value.UV_ENDIAN_MOD1 = ld1_params->endian.uv_word_endn;
	if (ld1_params->img_format == GSP_R3P0_DST_FMT_RGB565) {
		des_cfg_value.R5_RGB_SWAP_MOD1 =
			ld1_params->endian.rgb_swap_mode;
	} else if ((ld1_params->img_format == GSP_R3P0_DST_FMT_RGB888)
					&& (ld1_params->compress_r8_en)) {
		des_cfg_value.CR8_SWAP_MOD1 =
			ld1_params->endian.rgb_swap_mode;
	}
	des_cfg_value.A_SWAP_MOD1 = ld1_params->endian.a_swap_mode;
	des_cfg_value.ROT_MOD1 = ld1_params->rot_angle;
	des_cfg_value.R2Y_MOD1 = ld1_params->r2y_mod;
	des_cfg_value.DES_IMG_FORMAT1 = ld1_params->img_format;
	des_cfg_value.COMPRESS_R8 = ld1_params->compress_r8_en;
	des_cfg_mask.value = 0;
	des_cfg_mask.Y_ENDIAN_MOD1 = 0x7;
	des_cfg_mask.UV_ENDIAN_MOD1 = 0x7;
	des_cfg_mask.A_SWAP_MOD1 = 0x1;
	des_cfg_mask.R5_RGB_SWAP_MOD1 = 0x7;
	des_cfg_mask.ROT_MOD1 = 0x7;
	des_cfg_mask.R2Y_MOD1 = 0x7;
	des_cfg_mask.DES_IMG_FORMAT1 = 0x7;
	des_cfg_mask.COMPRESS_R8 = 0x1;
	des_cfg_mask.CR8_SWAP_MOD1 = 0x7;
	gsp_core_reg_update(R3P0_DES_DATA_CFG1(base),
		des_cfg_value.value, des_cfg_mask.value);

	/* background enable, dither enable cfg */
	gsp_mod1_cfg_value.value = 0;
	gsp_mod1_cfg_value.BK_EN = ld1_params->bk_para.bk_enable;
	gsp_mod1_cfg_value.DITHER1_EN = ld1_params->dither_en;
	gsp_mod1_cfg_mask.value = 0;
	gsp_mod1_cfg_mask.BK_EN = 0x1;
	gsp_mod1_cfg_mask.DITHER1_EN = 0x1;
	gsp_core_reg_update(R3P0_GSP_MOD1_CFG(base),
		gsp_mod1_cfg_value.value, gsp_mod1_cfg_mask.value);

}

void gsp_r3p0_core_ld2_reg_set(void __iomem *base,
			   struct gsp_r3p0_des_layer *layer)
{
	struct R3P0_DES_Y_ADDR2_REG des_y_addr_value;
	struct R3P0_DES_Y_ADDR2_REG des_y_addr_mask;
	struct R3P0_DES_U_ADDR2_REG des_u_addr_value;
	struct R3P0_DES_U_ADDR2_REG des_u_addr_mask;
	struct R3P0_DES_V_ADDR2_REG des_v_addr_value;
	struct R3P0_DES_V_ADDR2_REG des_v_addr_mask;
	struct R3P0_DES_PITCH2_REG des_pitch_value;
	struct R3P0_DES_PITCH2_REG des_pitch_mask;
	struct R3P0_DES_DATA_CFG2_REG des_cfg_value;
	struct R3P0_DES_DATA_CFG2_REG des_cfg_mask;
	struct R3P0_GSP_MOD2_CFG_REG gsp_mod2_cfg_value;
	struct R3P0_GSP_MOD2_CFG_REG gsp_mod2_cfg_mask;
	struct gsp_r3p0_des_layer_params	*ld2_params = NULL;

	if (IS_ERR_OR_NULL(base) || IS_ERR_OR_NULL(layer)) {
		GSP_ERR("LAYER Dest reg set params error\n");
		return;
	}
	ld2_params = &layer->params;

	if (layer->common.enable != 1) {
		GSP_DEBUG("no need to set LAYERD2\n");
		return;
	}

	/* dest layer 2 address set */
	des_y_addr_value.value = 0;
	des_y_addr_value.DES_Y_BASE_ADDR2 = layer->common.src_addr.addr_y;
	des_y_addr_mask.value = 0;
	des_y_addr_mask.DES_Y_BASE_ADDR2 = 0xFFFFFFFF;
	gsp_core_reg_update(R3P0_DES_Y_ADDR2(base),
		des_y_addr_value.value, des_y_addr_mask.value);

	des_u_addr_value.value = 0;
	des_u_addr_value.DES_U_BASE_ADDR2 = layer->common.src_addr.addr_uv;
	des_u_addr_mask.value = 0;
	des_u_addr_mask.DES_U_BASE_ADDR2 = 0xFFFFFFFF;
	gsp_core_reg_update(R3P0_DES_U_ADDR2(base),
		des_u_addr_value.value, des_u_addr_mask.value);

	des_v_addr_value.value = 0;
	des_v_addr_value.DES_V_BASE_ADDR2 = layer->common.src_addr.addr_va;
	des_v_addr_mask.value = 0;
	des_v_addr_mask.DES_V_BASE_ADDR2 = 0xFFFFFFFF;
	gsp_core_reg_update(R3P0_DES_V_ADDR2(base),
		des_v_addr_value.value, des_v_addr_mask.value);

	/* layer d2 pitch set - work plane configure*/
	des_pitch_value.value = 0;
	des_pitch_value.DES_PITCH2 = ld2_params->pitch;
	des_pitch_value.DES_HEIGHT2 = ld2_params->height;
	des_pitch_mask.value = 0;
	des_pitch_mask.DES_PITCH2 = 0x1FFF;
	des_pitch_mask.DES_HEIGHT2 = 0x1FFF;
	gsp_core_reg_update(R3P0_DES_PITCH2(base),
		des_pitch_value.value, des_pitch_mask.value);

	/* layer d2 cfg set */
	des_cfg_value.value = 0;
	des_cfg_value.Y_ENDIAN_MOD2 = ld2_params->endian.y_rgb_word_endn;
	des_cfg_value.UV_ENDIAN_MOD2 = ld2_params->endian.uv_word_endn;
	des_cfg_value.A_SWAP_MOD2 = ld2_params->endian.a_swap_mode;
	if (ld2_params->img_format == GSP_R3P0_DST_FMT_RGB565) {
		des_cfg_value.R5_RGB_SWAP_MOD2 =
			ld2_params->endian.rgb_swap_mode;
	} else if ((ld2_params->img_format == GSP_R3P0_DST_FMT_RGB888)
					&& (ld2_params->compress_r8_en)) {
		des_cfg_value.CR8_SWAP_MOD2 =
			ld2_params->endian.rgb_swap_mode;
	}
	des_cfg_value.ROT_MOD2 = ld2_params->rot_angle;
	des_cfg_value.R2Y_MOD2 = ld2_params->r2y_mod;
	des_cfg_value.DES_IMG_FORMAT2 = ld2_params->img_format;
	des_cfg_value.COMPRESS2_R8 = ld2_params->compress_r8_en;
	des_cfg_mask.value = 0;
	des_cfg_mask.Y_ENDIAN_MOD2 = 0x7;
	des_cfg_mask.UV_ENDIAN_MOD2 = 0x7;
	des_cfg_mask.A_SWAP_MOD2 = 0x1;
	des_cfg_mask.R5_RGB_SWAP_MOD2 = 0x7;
	des_cfg_mask.ROT_MOD2 = 0x7;
	des_cfg_mask.R2Y_MOD2 = 0x7;
	des_cfg_mask.DES_IMG_FORMAT2 = 0x7;
	des_cfg_mask.COMPRESS2_R8 = 0x1;
	des_cfg_mask.CR8_SWAP_MOD2 = 0x7;
	gsp_core_reg_update(R3P0_DES_DATA_CFG2(base),
		des_cfg_value.value, des_cfg_mask.value);

	/* dither2 enable cfg */
	gsp_mod2_cfg_value.value = 0;
	gsp_mod2_cfg_value.DITHER2_EN = ld2_params->dither_en;
	gsp_mod2_cfg_mask.value = 0;
	gsp_mod2_cfg_mask.DITHER2_EN = 0x1;
	gsp_core_reg_update(R3P0_GSP_MOD2_CFG(base),
		gsp_mod2_cfg_value.value, gsp_mod2_cfg_mask.value);

}
static void gsp_r3p0_coef_gen_and_cfg(struct gsp_r3p0_core *core,
						struct gsp_r3p0_cfg *cmd)
{
	int i = 0;
	uint32_t src_w = 0, src_h = 0;
	uint32_t dst_w = 0, dst_h = 0;
	uint32_t order_v = 0, order_h = 0;
	uint32_t ver_tap, hor_tap;
	uint32_t *ret_coef = NULL;
	uint32_t *pSrcCoef  = NULL;
	uint32_t *pDstCoef = NULL;
	struct R3P0_GSP_CTL_REG_T *ctl_reg_base =
		(struct R3P0_GSP_CTL_REG_T *) core->gsp_ctl_reg_base;

	if (cmd->misc.scale_para.scale_en) {
		src_w = cmd->misc.scale_para.scale_rect_in.rect_w;
		src_h = cmd->misc.scale_para.scale_rect_in.rect_h;
		dst_w = cmd->misc.scale_para.scale_rect_out.rect_w;
		dst_h = cmd->misc.scale_para.scale_rect_out.rect_h;
		hor_tap = cmd->misc.scale_para.htap_mod;
		ver_tap = cmd->misc.scale_para.vtap_mod;

		if (dst_h*4 < src_h && src_h <= dst_h*8)
			order_v = 1;
		else if (dst_h*8 < src_h && src_h <= dst_h*16)
			order_v = 2;

		if (dst_w*4 < src_w && src_w <= dst_w*8)
			order_h = 1;
		else if (dst_w*8 < src_w && src_w <= dst_w*16)
			order_h = 2;

		src_w = src_w >> order_h;
		src_h = src_h >> order_v;
		ret_coef = gsp_r3p0_gen_block_scaler_coef(core,
							src_w, src_h,
							dst_w, dst_h,
							hor_tap, ver_tap);
		/*config the coef to register of the bind core of cmd*/
		if (core) {
			pSrcCoef  = ret_coef;
			pDstCoef = (uint32_t *)&(ctl_reg_base->coef_tab);
			for (i = 0; i < 48 ; i++)
				*pDstCoef++ = *pSrcCoef++;

		} else {
			GSP_ERR("kcmd core is NULL, fail get coef addr!");
		}
	}
}

void gsp_r3p0_core_run(struct gsp_core *core)
{
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod1_cfg_value;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod1_cfg_mask;

	gsp_mod1_cfg_value.value = 0;
	gsp_mod1_cfg_value.BLD_RUN = 1;
	gsp_mod1_cfg_mask.value = 0;
	gsp_mod1_cfg_mask.BLD_RUN = 0x1;
	gsp_core_reg_update(R3P0_GSP_MOD1_CFG(core->base),
		gsp_mod1_cfg_value.value, gsp_mod1_cfg_mask.value);
}

int gsp_r3p0_core_run_precheck(struct gsp_core *c)
{
	struct gsp_r3p0_core *core = (struct gsp_r3p0_core *)c;
	struct WHALE2_AHB_EB_REG whale2_ahb_reg;
	struct WHALE2_GEN_CKG_CFG_REG  whale2_gen_clk_cfg_reg;
	int ret = 0;
	int err_code = 0;

	regmap_read(core->disp_ahb, 0x0, &whale2_ahb_reg.value);
	regmap_read(core->disp_ahb, 0x8, &whale2_gen_clk_cfg_reg.value);
	if (c->id == 0) {
		if (whale2_ahb_reg.GSP0_EB == 0)
			err_code = 0x1;

		if (whale2_ahb_reg.GSP0_MMU_EB == 0)
			err_code |= 0x2;

		if (whale2_ahb_reg.GSP_MTX_EB == 0)
			err_code |= 0x4;

		if (whale2_gen_clk_cfg_reg.GSP0_AUTO_CKG_EN == 0
			 && whale2_gen_clk_cfg_reg.GSP0_FORCE_CKG_EN == 0)
			err_code |= 0x8;

		if (whale2_gen_clk_cfg_reg.GSP_MTX_AUTO_CKG_EN == 0
			 && whale2_gen_clk_cfg_reg.GSP_MTX_FORCE_CKG_EN == 0)
			err_code |= 0x10;

		if (whale2_gen_clk_cfg_reg.GSP_NOC_AUTO_CKG_EN == 0
			 && whale2_gen_clk_cfg_reg.GSP_NOC_FORCE_CKG_EN == 0)
			err_code |= 0x20;
	} else {
		if (whale2_ahb_reg.GSP1_EB == 0)
			err_code = 0x100;

		if (whale2_ahb_reg.GSP1_MMU_EB == 0)
			err_code |= 0x200;

		if (whale2_ahb_reg.GSP_MTX_EB == 0)
			err_code |= 0x400;

		if (whale2_gen_clk_cfg_reg.GSP1_AUTO_CKG_EN == 0
			 && whale2_gen_clk_cfg_reg.GSP1_FORCE_CKG_EN == 0)
			err_code |= 0x800;

		if (whale2_gen_clk_cfg_reg.GSP_MTX_AUTO_CKG_EN == 0
			 && whale2_gen_clk_cfg_reg.GSP_MTX_FORCE_CKG_EN == 0)
			err_code |= 0x1000;

		if (whale2_gen_clk_cfg_reg.GSP_NOC_AUTO_CKG_EN == 0
			 && whale2_gen_clk_cfg_reg.GSP_NOC_FORCE_CKG_EN == 0)
			err_code |= 0x2000;
	}

	if (err_code) {
		ret = -1;
		GSP_ERR("err_code: 0x%x, 0x63100000: 0x%x, 0x63100008: 0x%x\n",
			 err_code, whale2_ahb_reg.value,
			 whale2_gen_clk_cfg_reg.value);
	} else {
		ret = 0;
	}

	return ret;
}

int gsp_r3p0_core_trigger(struct gsp_core *c)
{
	int ret = -1;
	void __iomem *base = NULL;
	struct gsp_kcfg *kcfg = NULL;
	struct gsp_r3p0_cfg *cfg = NULL;
	struct gsp_r3p0_core *core = (struct gsp_r3p0_core *)c;
	struct R3P0_GSP_MOD1_CFG_REG gsp_mod1_cfg_value;

	if (gsp_core_verify(c)) {
		GSP_ERR("gsp_r3p0 core trigger params error\n");
		return ret;
	}

	kcfg = c->current_kcfg;
	if (gsp_kcfg_verify(kcfg)) {
		GSP_ERR("gsp_r3p0 trigger invalidate kcfg\n");
		return ret;
	}

	/*hardware status check*/
	gsp_mod1_cfg_value.value =
		gsp_core_reg_read(R3P0_GSP_MOD1_CFG(c->base));
	if (gsp_mod1_cfg_value.BLD_BUSY
		|| gsp_mod1_cfg_value.RCH_BUSY
		|| gsp_mod1_cfg_value.WCH_BUSY) {
		GSP_ERR("core is still busy, can't trigger\n");
		return GSP_K_HW_BUSY_ERR;
	}
	base = c->base;
	cfg = (struct gsp_r3p0_cfg *)kcfg->cfg;

	gsp_r3p0_coef_gen_and_cfg(core, cfg);

	gsp_r3p0_core_misc_reg_set(c, cfg);
	gsp_r3p0_core_l0_reg_set(base, &cfg->l0);
	gsp_r3p0_core_l1_reg_set(base, &cfg->l1);
	gsp_r3p0_core_l2_reg_set(base, &cfg->l2);
	gsp_r3p0_core_l3_reg_set(base, &cfg->l3);
	gsp_r3p0_core_ld1_reg_set(base, &cfg->ld1);
	if (cfg->misc.run_mod)
		gsp_r3p0_core_ld2_reg_set(base, &cfg->ld2);

	if (gsp_r3p0_core_run_precheck(c)) {
		GSP_ERR("r3p0 core run precheck fail !\n");
		return GSP_K_CLK_CHK_ERR;
	}

	if (gsp_r3p0_core_cfg_is_err(c)) {
		gsp_r3p0_core_err_code_print(c);
		return GSP_K_CTL_CODE_ERR;
	}

	gsp_r3p0_core_run(c);

	return GSP_NO_ERR;
};


int gsp_r3p0_core_release(struct gsp_core *c)
{

	return 0;
}

int gsp_r3p0_core_copy_cfg(struct gsp_kcfg *kcfg,
			void *arg, int index)
{
	struct gsp_r3p0_cfg_user *cfg_user_arr = NULL;
	struct gsp_r3p0_cfg_user *cfg_user;
	struct gsp_r3p0_cfg *cfg = NULL;

	if (IS_ERR_OR_NULL(arg)
		|| 0 > index) {
		GSP_ERR("core copy params error\n");
		return -1;
	}

	if (gsp_kcfg_verify(kcfg)) {
		GSP_ERR("core copy kcfg param error\n");
		return -1;
	}

	cfg_user_arr = (struct gsp_r3p0_cfg_user *)arg;
	cfg_user = &cfg_user_arr[index];

	cfg = (struct gsp_r3p0_cfg *)kcfg->cfg;

	/* we must reinitialize cfg again in case data is residual */
	gsp_r3p0_core_cfg_reinit(cfg);

	/* first copy common gsp layer params from user */
	gsp_layer_common_copy_from_user(&cfg->l0, &cfg_user->l0);
	gsp_layer_common_copy_from_user(&cfg->l1, &cfg_user->l1);
	gsp_layer_common_copy_from_user(&cfg->l2, &cfg_user->l2);
	gsp_layer_common_copy_from_user(&cfg->l3, &cfg_user->l3);
	gsp_layer_common_copy_from_user(&cfg->ld1, &cfg_user->ld1);
	gsp_layer_common_copy_from_user(&cfg->ld2, &cfg_user->ld2);

	/* second copy specific gsp layer params from user */
	memcpy(&cfg->l0.params, &cfg_user->l0.params,
		   sizeof(struct gsp_r3p0_img_layer_params));
	gsp_layer_set_filled(&cfg->l0.common);

	memcpy(&cfg->l1.params, &cfg_user->l1.params,
		   sizeof(struct gsp_r3p0_osd_layer_params));
	gsp_layer_set_filled(&cfg->l1.common);

	memcpy(&cfg->l2.params, &cfg_user->l2.params,
		   sizeof(struct gsp_r3p0_osd_layer_params));
	gsp_layer_set_filled(&cfg->l2.common);

	memcpy(&cfg->l3.params, &cfg_user->l3.params,
		   sizeof(struct gsp_r3p0_osd_layer_params));
	gsp_layer_set_filled(&cfg->l3.common);

	memcpy(&cfg->ld1.params, &cfg_user->ld1.params,
		   sizeof(struct gsp_r3p0_des_layer_params));
	gsp_layer_set_filled(&cfg->ld1.common);

	memcpy(&cfg->ld2.params, &cfg_user->ld2.params,
		   sizeof(struct gsp_r3p0_des_layer_params));
	gsp_layer_set_filled(&cfg->ld2.common);

	memcpy(&cfg->misc, &cfg_user->misc,
		   sizeof(struct gsp_r3p0_misc_cfg));

	gsp_r3p0_core_cfg_print(cfg);

	return 0;

}

int __user *gsp_r3p0_core_intercept(void __user *arg, int index)
{
	struct gsp_r3p0_cfg_user  __user *cfgs = NULL;

	cfgs = (struct gsp_r3p0_cfg_user  *)arg;
	return &cfgs[index].ld1.common.sig_fd;
}

void gsp_r3p0_core_reset(struct gsp_core *core)
{
	struct gsp_r3p0_core *c = (struct gsp_r3p0_core *)core;
	struct WHALE2_AHB_RST_REG whale2_reset_reg;

	if (core->id == 0) {
		regmap_read(c->disp_ahb, 0x4, &whale2_reset_reg.value);
		whale2_reset_reg.GSP0_SOFT_RST = 1;
		whale2_reset_reg.GSP_MTX_SOFT_RST = 1;
		regmap_write(c->disp_ahb, 0x4, whale2_reset_reg.value);

		mdelay(1);

		whale2_reset_reg.GSP0_SOFT_RST = 0;
		whale2_reset_reg.GSP_MTX_SOFT_RST = 0;
		regmap_write(c->disp_ahb, 0x4, whale2_reset_reg.value);
	} else {
		regmap_read(c->disp_ahb, 0x4, &whale2_reset_reg.value);
		whale2_reset_reg.GSP1_SOFT_RST = 1;
		whale2_reset_reg.GSP_MTX_SOFT_RST = 1;
		regmap_write(c->disp_ahb, 0x4, whale2_reset_reg.value);

		mdelay(1);

		whale2_reset_reg.GSP1_SOFT_RST = 0;
		whale2_reset_reg.GSP_MTX_SOFT_RST = 0;
		regmap_write(c->disp_ahb, 0x4, whale2_reset_reg.value);
	}

	gsp_r3p0_core_disable(core);
	mdelay(1);
	gsp_r3p0_core_enable(core);
}


