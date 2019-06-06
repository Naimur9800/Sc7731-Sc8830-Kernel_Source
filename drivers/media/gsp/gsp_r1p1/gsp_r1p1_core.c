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
#include <linux/io.h>
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
#include <video/gsp_cfg.h>
#include <video/gsp_r1p1_cfg.h>
#include "../gsp_core.h"
#include "../gsp_debug.h"
#include "../gsp_dev.h"
#include "../gsp_layer.h"
#include "../gsp_kcfg.h"
#include "../gsp_workqueue.h"
#include "gsp_r1p1_core.h"

#define gsp_r1p1_layer_common_copy_from_user(layer, layer_user) \
do { \
	(layer)->common.type = (layer_user)->common.type; \
	(layer)->common.enable = (layer_user)->common.enable; \
	(layer)->common.wait_fd = (layer_user)->common.wait_fd; \
	(layer)->common.sig_fd = (layer_user)->common.sig_fd; \
	(layer)->common.mem_data.share_fd = (layer_user)->common.share_fd; \
	(layer)->common.mem_data.uv_offset = \
				(layer_user)->common.offset.uv_offset; \
	(layer)->common.mem_data.v_offset = \
				(layer_user)->common.offset.v_offset; \
	(layer)->common.src_addr = (layer_user)->common.src_addr; \
} while(0)

static int gsp_r1p1_core_cfg_is_err(struct gsp_core *core)
{
	u32 val = 0;

	val = gsp_core_reg_read(GSP_CFG(core->base));
	return (val & ERR_FLG) >> ERR_FLG_OFFSET;
}

static void gsp_r1p1_core_err_code_print(struct gsp_core *core)
{
	u32 val = 0;

	val = gsp_core_reg_read(GSP_CFG(core->base));
	GSP_ERR("register configure error code: %d.\n",
		(val & ERR_CODE) >> ERR_CODE_OFFSET);
}

static void gsp_r1p1_core_cfg_init(struct gsp_r1p1_cfg *cfg,
				struct gsp_kcfg *kcfg)
{
	static int tag = 0;

	if (IS_ERR_OR_NULL(cfg)) {
		GSP_ERR("cfg init params error\n");
		return;
	}

	INIT_LIST_HEAD(&cfg->common.layers);
	INIT_LIST_HEAD(&cfg->l0.common.list);
	INIT_LIST_HEAD(&cfg->l1.common.list);
	INIT_LIST_HEAD(&cfg->ld.common.list);

	cfg->common.layer_num = 2;
	cfg->common.init = 1;
	cfg->common.kcfg = kcfg;
	cfg->common.tag = tag++;

	list_add_tail(&cfg->l0.common.list, &cfg->common.layers);
	list_add_tail(&cfg->l1.common.list, &cfg->common.layers);
	list_add_tail(&cfg->ld.common.list, &cfg->common.layers);

	GSP_INFO("gsp_r1p1 cfg[%d] initialized done\n", cfg->common.tag);
}

static void print_image_layer_cfg(struct gsp_r1p1_img_layer *layer)
{
	struct gsp_r1p1_img_layer_params *params= NULL;

	GSP_DEBUG("\ngsp_r1p1 image layer configure information begin\n");

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
		  params->endian.y_word_endn, params->endian.uv_word_endn,
		  params->endian.va_word_endn, params->endian.rgb_swap_mode,
		  params->endian.a_swap_mode);

	GSP_DEBUG("img_format: %d, rot_angle: %d\n",
		  params->img_format, params->rot_angle);

	GSP_DEBUG("row_tap_mode: %d, col_tap_mode: %d\n",
		  params->row_tap_mode, params->col_tap_mode);

	GSP_DEBUG("alpha: %d, colorkey_en: %d, pallet_en: %d\n",
		  params->alpha, params->colorkey_en, params->pallet_en);

	GSP_DEBUG("scaling_en: %d, pmargb_en: %d, pmargb_mod: %d\n",
		  params->scaling_en, params->pmargb_en, params->pmargb_mod);

	GSP_DEBUG("gsp_r1p1 image layer configure information end\n");
}

static void print_osd_layer_cfg(struct gsp_r1p1_osd_layer *layer)
{
	struct gsp_r1p1_osd_layer_params *params= NULL;

	GSP_DEBUG("\ngsp_r1p1 osd layer configure information begin\n");

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
		  params->endian.y_word_endn, params->endian.uv_word_endn,
		  params->endian.va_word_endn, params->endian.rgb_swap_mode,
		  params->endian.a_swap_mode);

	GSP_DEBUG("osd_format: %d, rot_angle: %d\n",
		  params->osd_format, params->rot_angle);

	GSP_DEBUG("row_tap_mode: %d, col_tap_mode: %d\n",
		  params->row_tap_mode, params->col_tap_mode);

	GSP_DEBUG("alpha: %d, colorkey_en: %d, pallet_en: %d\n",
		  params->alpha, params->colorkey_en, params->pallet_en);

	GSP_DEBUG("pmargb_en: %d, pmargb_mod: %d\n",
		  params->pmargb_en, params->pmargb_mod);

	GSP_DEBUG("gsp_r1p1 osd layer configure information end\n");
}

static void print_des_layer_cfg(struct gsp_r1p1_des_layer *layer)
{
	struct gsp_r1p1_des_layer_params *params= NULL;

	GSP_DEBUG("\ngsp_r1p1 des layer configure information begin\n");

	params = &layer->params;
	gsp_layer_common_print(&layer->common);

	GSP_DEBUG("pitch: %d, endian: [%d, %d, %d, %d, %d]\n",
		  params->pitch,
		  params->endian.y_word_endn, params->endian.uv_word_endn,
		  params->endian.va_word_endn, params->endian.rgb_swap_mode,
		  params->endian.a_swap_mode);

	GSP_DEBUG("img_format: %d, compress_r8_en: %d\n",
		  params->img_format, params->compress_r8_en);

	GSP_DEBUG("gsp_r1p1 des layer configure information end\n");
}

static void gsp_r1p1_core_cfg_print(struct gsp_r1p1_cfg *cfg)
{
	print_image_layer_cfg(&cfg->l0);
	print_osd_layer_cfg(&cfg->l1);
	print_des_layer_cfg(&cfg->ld);
}

static void gsp_r1p1_core_cfg_reinit(struct gsp_r1p1_cfg *cfg)
{
	struct gsp_layer *layer;
	struct gsp_kcfg *kcfg = NULL;

	if (IS_ERR_OR_NULL(cfg)) {
		GSP_ERR("cfg init params error\n");
		return;
	}

	kcfg = cfg->common.kcfg;
	GSP_DEBUG("gsp_r1p1 cfg parent kcfg[%d]\n", gsp_kcfg_to_tag(kcfg));
	if (cfg->common.init != 1) {
		GSP_ERR("gsp_r1p1 cfg[%d] has not been initialized\n",
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
	memset(&cfg->l0.params, 0, sizeof(struct gsp_r1p1_img_layer_params));
	memset(&cfg->l1.params, 0, sizeof(struct gsp_r1p1_osd_layer_params));
	memset(&cfg->ld.params, 0, sizeof(struct gsp_r1p1_des_layer_params));

	/* last to reset misc information */
	memset(&cfg->misc, 0, sizeof(struct gsp_r1p1_misc_cfg));
}

static void gsp_r1p1_core_capability_init(struct gsp_core *core)
{
	struct gsp_r1p1_capability *capa = NULL;
	struct gsp_rect max = {0, 0, 4095, 4095};
	struct gsp_rect min = {0, 0, 4, 4};

	capa = (struct gsp_r1p1_capability *)core->capa;

	/* common information initialize */
	capa->common.magic = GSP_CAPABILITY_MAGIC;
	strcpy(capa->common.version, "R1P1");
	capa->common.capa_size = sizeof(*capa);

	capa->common.max_layer = 2;
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
	capa->max_video_size = 1;
}

void gsp_r1p1_core_coef_init(struct gsp_core *c)
{
	int i = 0;
	struct gsp_r1p1_core *core = NULL;

	core = (struct gsp_r1p1_core *)c;
	INIT_LIST_HEAD(&core->coef_list);
	for (i = 0; i < COEF_CACHE_MAX; i++) {
		list_add_tail(&core->coef_cache[i].entry_list,
			      &core->coef_list);
	}

	c->coef_init = 1;
}

int gsp_r1p1_core_init(struct gsp_core *core)
{
	struct gsp_kcfg *kcfg = NULL;

	if (gsp_core_verify(core)) {
		GSP_ERR("core init params error");
		return -1;
	}

	gsp_r1p1_core_capability_init(core);

	gsp_r1p1_core_coef_init(core);

	/* to do some specific initialize operations here */
	list_for_each_entry(kcfg, &core->kcfgs, sibling) {
		gsp_r1p1_core_cfg_init((struct gsp_r1p1_cfg *)kcfg->cfg, kcfg);
	}

	return 0;
}

int gsp_r1p1_core_alloc(struct gsp_core **core, struct device_node *node)
{
	struct gsp_capability *capa = NULL;
	struct platform_device *pdev = NULL;

	if (IS_ERR_OR_NULL(core)) {
		GSP_ERR("core alloc params error\n");
		return -1;
	}

	/* device utility has been registered at of_device_add() */
	pdev = of_find_device_by_node(node);
	if (IS_ERR_OR_NULL(pdev)) {
		GSP_ERR("find gsp_r1p1 device by child node failed\n");
		return -1;
	}

	*core = devm_kzalloc(&pdev->dev, sizeof(struct gsp_r1p1_core),
			     GFP_KERNEL);
	if (IS_ERR_OR_NULL(*core)) {
		GSP_ERR("no enough memory for core alloc\n");
		return -1;
	}

	memset(*core, 0, sizeof(struct gsp_r1p1_core));
	(*core)->cfg_size = sizeof(struct gsp_r1p1_cfg);
	(*core)->dev = &pdev->dev;
	if (dev_name(&pdev->dev))
		GSP_INFO("core[%d] device name: %s\n", (*core)->id,
			 dev_name(&pdev->dev));

	capa = kzalloc(sizeof(struct gsp_r1p1_capability), GFP_KERNEL);
	if (IS_ERR_OR_NULL(capa)) {
		GSP_ERR("gsp_r1p1 capability memory allocated failed\n");
		return -1;
	}
	(*core)->capa = capa;
	(*core)->node = node;
	GSP_INFO("gsp_r1p1 core allocate success\n");

	return 0;
}

void gsp_r1p1_core_irqstate_clear(struct gsp_core *core)
{
	gsp_core_reg_update(GSP_INT_CFG(core->base),
			    INT_CLR, INT_CLR);
	udelay(10);
	gsp_core_reg_update(GSP_INT_CFG(core->base),
			    0, INT_CLR);

}

void gsp_r1p1_core_irq_enable(struct gsp_core *core)
{
	gsp_core_reg_update(GSP_INT_CFG(core->base),
			    INT_EN, INT_EN);
}

void gsp_r1p1_core_irq_mode_set(struct gsp_core *core)
{
	u32 value = 0;

	value = GSP_IRQ_MODE_LEVEL << INT_MOD_OFFSET;
	gsp_core_reg_update(GSP_INT_CFG(core->base),
			    value, INT_MOD);
}

void gsp_r1p1_core_irq_disable(struct gsp_core *core)
{
	gsp_core_reg_update(GSP_INT_CFG(core->base),
			    0, INT_EN);
}

irqreturn_t gsp_r1p1_core_irq_handler(int irq, void *data)
{
	struct gsp_core *core = NULL;

	core = (struct gsp_core *)data;
	if (gsp_core_verify(core)) {
		GSP_ERR("core irq handler not match\n");
		return IRQ_NONE;
	}

	gsp_r1p1_core_irqstate_clear(core);
	gsp_r1p1_core_irq_disable(core);

	gsp_core_state_set(core, CORE_STATE_IRQ_HANDLED);
	queue_kthread_work(&core->kworker, &core->release);

	return IRQ_HANDLED;
}

int gsp_r1p1_core_enable(struct gsp_core *c)
{
	int ret = -1;
	struct gsp_r1p1_core *core = NULL;

	core = (struct gsp_r1p1_core *)c;
	gsp_r1p1_core_irq_mode_set(c);
	gsp_r1p1_core_irq_enable(c);

	ret = clk_prepare_enable(core->core_en_clk);
	if (ret) {
		GSP_ERR("enable core[%d] switch clock failed\n",
			gsp_core_to_id(c));
		goto clk_en_disable;
	}

	ret = clk_prepare_enable(core->core_emc_clk);
	if (ret) {
		GSP_ERR("enable core[%d] emc clock failed\n",
			gsp_core_to_id(c));
		goto clk_emc_disable;
	}

	ret = clk_prepare_enable(core->core_clk);
	if (ret) {
		GSP_ERR("enable core[%d] clock failed\n",
			gsp_core_to_id(c));
		goto clk_disable;
	}

	goto exit;

clk_disable:
	clk_disable_unprepare(core->core_clk);
clk_emc_disable:
	clk_disable_unprepare(core->core_emc_clk);
clk_en_disable:
	clk_disable_unprepare(core->core_en_clk);

exit:
	return ret;
}

void gsp_r1p1_core_disable(struct gsp_core *c)
{
	struct gsp_r1p1_core *core = NULL;

	core = (struct gsp_r1p1_core *)c;
	gsp_r1p1_core_irq_disable(c);

	clk_disable_unprepare(core->core_clk);
	clk_disable_unprepare(core->core_emc_clk);
	clk_disable_unprepare(core->core_en_clk);

}

int gsp_r1p1_core_parse_clk(struct gsp_core *c)
{
	struct gsp_r1p1_core *core = (struct gsp_r1p1_core *)c;

	core->core_clk = of_clk_get_by_name(c->node, CORE_CLK);
	if (IS_ERR_OR_NULL(core->core_clk)) {
		GSP_ERR("core[%d] read module clk failed\n",
			gsp_core_to_id(c));
		return -1;
	}

	core->core_pclk = of_clk_get_by_name(c->node, CORE_PARENT_CLK);
	if (IS_ERR_OR_NULL(core->core_pclk)) {
		GSP_ERR("core[%d] read module parent clk failed\n",
			gsp_core_to_id(c));
		return -1;
	}
	clk_set_parent(core->core_clk, core->core_pclk);

	core->core_emc_clk = of_clk_get_by_name(c->node, CORE_EMC_CLK);
	if (IS_ERR_OR_NULL(core->core_emc_clk)) {
		GSP_ERR("core[%d] read module emc clk failed\n",
			gsp_core_to_id(c));
		return -1;
	}

	core->core_en_clk = of_clk_get_by_name(c->node, CORE_EN_CLK);
	if (IS_ERR_OR_NULL(core->core_en_clk)) {
		GSP_ERR("core[%d] read module enable clk failed\n",
			gsp_core_to_id(c));
		return -1;
	}

	return 0;
}

int gsp_r1p1_core_parse_irq(struct gsp_core *core)
{
	int ret = -1;
	struct device *dev = NULL;

	dev = gsp_core_to_device(core);
	core->irq = irq_of_parse_and_map(core->node, 0);
	if (core->irq)
		ret = devm_request_irq(dev, core->irq,
				       gsp_r1p1_core_irq_handler,
				       IRQF_SHARED, "gsp-core", core);

	return ret;
}

int gsp_r1p1_core_parse_misc(struct gsp_core *core)
{
	/* to parse misc information here */
	return 0;
}

int gsp_r1p1_core_parse_dt(struct gsp_core *core)
{
	int ret = -1;

	ret = gsp_r1p1_core_parse_misc(core);
	if (ret) {
		GSP_ERR("core[%d] parse misc info failed\n", core->id);
		return ret;
	}

	ret = gsp_r1p1_core_parse_irq(core);
	if (ret) {
		GSP_ERR("core[%d] parse irq failed\n", core->id);
		return ret;
	}

	ret = gsp_r1p1_core_parse_clk(core);
	if (ret)
		GSP_ERR("core[%d] parse clk failed\n", core->id);

	return ret;
}

void gsp_r1p1_core_misc_reg_set(struct gsp_core *core, struct gsp_r1p1_cfg *cfg)
{
	u32 value = 0;
	void __iomem *base = NULL;

	if (gsp_core_verify(core)
	    || IS_ERR_OR_NULL(cfg)) {
		GSP_ERR("misc reg set params error\n");
		return;
	}

	base = core->base;

	/* layer enable set */
	value = cfg->l0.common.enable << L0_EN_OFFSET & L0_EN;
	gsp_core_reg_update(GSP_CFG(base), value, L0_EN);

	value = cfg->l1.common.enable << L1_EN_OFFSET & L1_EN;
	gsp_core_reg_update(GSP_CFG(base), value, L1_EN);

	/* have to reset scale enable bit because of hardware request */
	if (cfg->l0.params.scaling_en == 1) {
		gsp_core_reg_update(GSP_CFG(base), SCL_CLR, SCL_CLR);
		udelay(10);
		gsp_core_reg_update(GSP_CFG(base), ~SCL_CLR, SCL_CLR);
	}

	/* layer0 scaling enable set */
	value = cfg->l0.params.scaling_en << SCALE_EN_OFFSET & SCALE_EN;
	gsp_core_reg_update(GSP_CFG(base), value, SCALE_EN);

	/* pmargb enable set */
	value = (cfg->l0.params.pmargb_en || cfg->l1.params.pmargb_en)
		<< PMARGB_EN_OFFSET & PMARGB_EN;
	gsp_core_reg_update(GSP_CFG(base), value, PMARGB_EN);

	/* pmargb mod set: bit0->layer0, bit1->layer1 */
	value = (cfg->l0.params.pmargb_en + (cfg->l1.params.pmargb_en << 1))
		<< PMARGB_MOD_OFFSET & PMARGB_MOD;
	gsp_core_reg_update(GSP_CFG(base), value, PMARGB_MOD);

	/* dithering enable set */
	value = cfg->misc.dither_en << DITHER_EN_OFFSET & DITHER_EN;
	gsp_core_reg_update(GSP_CFG(base), value, DITHER_EN);

	/* gsp clk number of AXI READ burst interval */
	value = cfg->misc.gsp_gap << DIST_RB_OFFSET & DIST_RB;
	gsp_core_reg_update(GSP_CFG(base), value, DIST_RB);

	/* y2r opt for narrow yuv format */
	value = cfg->l0.params.y2r_opt << Y2R_OPT_OFFSET & Y2R_OPT;
	gsp_core_reg_update(GSP_CFG(base), value, Y2R_OPT);

	/* bnd bypass for 4k boundary */
	value = cfg->l0.params.bnd_bypass << BND_BYPASS_OFFSET & BND_BYPASS;
	gsp_core_reg_update(GSP_CFG(base), value, BND_BYPASS);
}

void gsp_r1p1_core_l0_reg_set(void __iomem *base,
			   struct gsp_r1p1_img_layer *layer)
{
	u32 value = 0;
	struct gsp_r1p1_img_layer_params *params = NULL;

	if (IS_ERR_OR_NULL(base)
	    || IS_ERR_OR_NULL(layer)) {
		GSP_ERR("layer0 reg set params error\n");
		return;
	}

	if (layer->common.enable != 1) {
		GSP_DEBUG("no need to set layer0\n");
		return;
	}

	params = &layer->params;

	/* layer0 address set */
	value = layer->common.src_addr.addr_y;
	gsp_core_reg_write(LAYER0_Y_ADDR(base), value);

	value = layer->common.src_addr.addr_uv;
	gsp_core_reg_write(LAYER0_UV_ADDR(base), value);

	value = layer->common.src_addr.addr_va;
	gsp_core_reg_write(LAYER0_VA_ADDR(base), value);

	/* layer0 pitch set */
	value = params->pitch << PITCH0_OFFSET & PITCH0;
	gsp_core_reg_update(LAYER0_PITCH(base), value, PITCH0);

	/* layer0 clip rect size set */
	value = (params->clip_rect.rect_w & CLIP_SIZE_X_L0) +
		(params->clip_rect.rect_h << CLIP_SIZE_Y_L0_OFFSET
		 & CLIP_SIZE_Y_L0);
	gsp_core_reg_write(LAYER0_CLIP_SIZE(base), value);

	/* layer0 clip start position set */
	value = (params->clip_rect.st_x & CLIP_START_X_L0) +
		(params->clip_rect.st_y << CLIP_START_Y_L0_OFFSET
		 & CLIP_START_Y_L0);
	gsp_core_reg_write(LAYER0_CLIP_START(base), value);

	/* layer0 destination clip rect size set */
	value = (params->des_rect.rect_w & DES_SIZE_X_L0) +
		(params->des_rect.rect_h << DES_SIZE_Y_L0_OFFSET
		 & DES_SIZE_Y_L0);
	gsp_core_reg_write(LAYER0_DES_SIZE(base), value);

	/* layer0 destination clip start position set*/
	value = (params->des_rect.st_x & DES_START_X_L0) +
		(params->des_rect.st_y << DES_START_Y_L0_OFFSET
		 & DES_START_Y_L0);
	gsp_core_reg_write(LAYER0_DES_START(base), value);

	/* layer0 grey color set */
	value = (params->grey.r_val << LAYER0_GREY_R_OFFSET
		 & LAYER0_GREY_R) +
		(params->grey.r_val << LAYER0_GREY_G_OFFSET
		 & LAYER0_GREY_G) +
		(params->grey.r_val << LAYER0_GREY_B_OFFSET
		 & LAYER0_GREY_B);
	gsp_core_reg_write(LAYER0_GREY_RGB(base), value);

	/* layer0 endian set */
	value = (params->endian.y_word_endn << Y_ENDIAN_MOD_L0_OFFSET
		 & Y_ENDIAN_MOD_L0) +
		(params->endian.uv_word_endn << UV_ENDIAN_MOD_L0_OFFSET
		 & UV_ENDIAN_MOD_L0) +
		(params->endian.va_word_endn << VA_ENDIAN_MOD_L0_OFFSET
		 & VA_ENDIAN_MOD_L0) +
		(params->endian.rgb_swap_mode << RGB_SWAP_MOD_L0_OFFSET
		 & RGB_SWAP_MOD_L0) +
		(params->endian.a_swap_mode << A_SWAP_MOD_L0_OFFSET
		 & A_SWAP_MOD_L0);
	gsp_core_reg_write(LAYER0_ENDIAN(base), value);

	/* layer0 alpha set */
	value = params->alpha << ALPHA_L0_OFFSET & ALPHA_L0;
	gsp_core_reg_write(LAYER0_ALPHA(base), value);

	/* layer0 color key set*/
	if (params->colorkey_en) {
		value = (params->colorkey.r_val << CK_R_L0_OFFSET & CK_R_L0) +
			(params->colorkey.g_val << CK_G_L0_OFFSET & CK_G_L0) +
			(params->colorkey.b_val << CK_B_L0_OFFSET & CK_B_L0);
		gsp_core_reg_write(LAYER0_CK(base), value);
	}

	/* layer0 image format set*/
	value = params->img_format << IMG_FORMAT_L0_OFFSET
		& IMG_FORMAT_L0;
	gsp_core_reg_update(LAYER0_CFG(base), value, IMG_FORMAT_L0);

	/* layer0 rotation angle set*/
	value = params->rot_angle << ROT_MOD_L0_OFFSET & ROT_MOD_L0;
	gsp_core_reg_update(LAYER0_CFG(base), value, ROT_MOD_L0);

	/* layer0 color key enbale set */
	value = params->colorkey_en << CK_EN_L0_OFFSET & CK_EN_L0;
	gsp_core_reg_update(LAYER0_CFG(base), value, CK_EN_L0);

	/* layer0 pallet enable set */
	value = params->pallet_en << PALLET_EN_L0_OFFSET & PALLET_EN_L0;
	gsp_core_reg_update(LAYER0_CFG(base), value, PALLET_EN_L0);
}

void gsp_r1p1_core_l1_reg_set(void __iomem *base,
			   struct gsp_r1p1_osd_layer *layer)
{
	u32 value = 0;
	struct gsp_r1p1_osd_layer_params *params = NULL;

	if (IS_ERR_OR_NULL(base)
	    || IS_ERR_OR_NULL(layer)) {
		GSP_ERR("LAYER1 reg set params error\n");
		return;
	}

	if (layer->common.enable != 1) {
		GSP_DEBUG("no need to set LAYER1\n");
		return;
	}

	params = &layer->params;

	/* layer1 address set */
	value = layer->common.src_addr.addr_y;
	gsp_core_reg_write(LAYER1_Y_ADDR(base), value);

	value = layer->common.src_addr.addr_uv;
	gsp_core_reg_write(LAYER1_UV_ADDR(base), value);

	value = layer->common.src_addr.addr_va;
	gsp_core_reg_write(LAYER1_VA_ADDR(base), value);

	/* layer1 pitch set */
	value = params->pitch << PITCH1_OFFSET & PITCH1;
	gsp_core_reg_update(LAYER1_PITCH(base), value, PITCH1);

	/* layer1 clip rect size set */
	value = (params->clip_rect.rect_w & CLIP_SIZE_X_L1) +
		(params->clip_rect.rect_h << CLIP_SIZE_Y_L1_OFFSET
		 & CLIP_SIZE_Y_L1);
	gsp_core_reg_write(LAYER1_CLIP_SIZE(base), value);

	/* layer1 clip start position set */
	value = (params->clip_rect.st_x & CLIP_START_X_L1) +
		(params->clip_rect.st_y << CLIP_START_Y_L1_OFFSET
		 & CLIP_START_Y_L1);
	gsp_core_reg_write(LAYER1_CLIP_START(base), value);

	/* LAYER1 destination start position set*/
	value = (params->des_pos.pt_x & DES_START_X_L1) +
		(params->des_pos.pt_y << DES_START_Y_L1_OFFSET
		 & DES_START_Y_L1);
	gsp_core_reg_write(LAYER1_DES_START(base), value);

	/* LAYER1 grey color set */
	value = (params->grey.r_val << GREY_R_L1_OFFSET & GREY_R_L1) +
		(params->grey.r_val << GREY_G_L1_OFFSET & GREY_G_L1) +
		(params->grey.r_val << GREY_B_L1_OFFSET & GREY_B_L1);
	gsp_core_reg_write(LAYER1_GREY_RGB(base), value);

	/* LAYER1 endian set */
	value = (params->endian.y_word_endn << Y_ENDIAN_MOD_L1_OFFSET
		 & Y_ENDIAN_MOD_L1) +
		(params->endian.uv_word_endn << UV_ENDIAN_MOD_L1_OFFSET
		 & UV_ENDIAN_MOD_L1) +
		(params->endian.va_word_endn << VA_ENDIAN_MOD_L1_OFFSET
		 & VA_ENDIAN_MOD_L1) +
		(params->endian.rgb_swap_mode << RGB_SWAP_MOD_L1_OFFSET
		 & RGB_SWAP_MOD_L1) +
		(params->endian.a_swap_mode << A_SWAP_MOD_L1_OFFSET
		 & A_SWAP_MOD_L1);
	gsp_core_reg_write(LAYER1_ENDIAN(base), value);

	/* LAYER1 alpha set */
	value = params->alpha << ALPHA_L1_OFFSET & ALPHA_L1;
	gsp_core_reg_write(LAYER1_ALPHA(base), value);

	/* LAYER1 color key set*/
	if (params->colorkey_en) {
		value = (params->colorkey.r_val << CK_R_L1_OFFSET & CK_R_L1) +
			(params->colorkey.g_val << CK_G_L1_OFFSET & CK_G_L1) +
			(params->colorkey.b_val << CK_B_L1_OFFSET & CK_B_L1);
		gsp_core_reg_write(LAYER1_CK(base), value);
	}

	/* LAYER1 image format set*/
	value = params->osd_format << IMG_FORMAT_L1_OFFSET & IMG_FORMAT_L1;
	gsp_core_reg_update(LAYER1_CFG(base), value, IMG_FORMAT_L1);

	/* LAYER1 rotation angle set*/
	value = params->rot_angle << ROT_MOD_L1_OFFSET & ROT_MOD_L1;
	gsp_core_reg_update(LAYER1_CFG(base), value, ROT_MOD_L1);

	/* LAYER1 color key enbale set */
	value = params->colorkey_en << CK_EN_L1_OFFSET & CK_EN_L1;
	gsp_core_reg_update(LAYER1_CFG(base), value, CK_EN_L1);

	/* layer1 pallet enable set */
	value = params->pallet_en << PALLET_EN_L1_OFFSET & PALLET_EN_L1;
	gsp_core_reg_update(LAYER1_CFG(base), value, PALLET_EN_L1);
}

void gsp_r1p1_core_ld_reg_set(void __iomem *base,
			   struct gsp_r1p1_des_layer *layer)
{
	u32 value = 0;
	struct gsp_r1p1_des_layer_params *params = NULL;

	if (IS_ERR_OR_NULL(base)
	    || IS_ERR_OR_NULL(layer)) {
		GSP_ERR("LAYER1 reg set params error\n");
		return;
	}

	if (layer->common.enable != 1) {
		GSP_ERR("layerd is not enable\n");
		return;
	}

	params = &layer->params;

	/* layer1 address set */
	value = layer->common.src_addr.addr_y;
	gsp_core_reg_write(DES_Y_ADDR(base), value);

	value = layer->common.src_addr.addr_uv;
	gsp_core_reg_write(DES_UV_ADDR(base), value);

	value = layer->common.src_addr.addr_va;
	gsp_core_reg_write(DES_V_ADDR(base), value);

	/* layerd pitch set */
	value = params->pitch << DES_PITCH_OFFSET & DES_PITCH;
	gsp_core_reg_update(DES_PITCH_REG(base), value, DES_PITCH);

	/* layerd endian set */
	value = (params->endian.y_word_endn << Y_ENDIAN_MOD_OFFSET &
		Y_ENDIAN_MOD) +
		(params->endian.uv_word_endn << UV_ENDIAN_MOD_OFFSET &
		 UV_ENDIAN_MOD) +
		(params->endian.va_word_endn << V_ENDIAN_MOD_OFFSET &
		 V_ENDIAN_MOD) +
		(params->endian.rgb_swap_mode << RGB_SWAP_MOD_OFFSET &
		 RGB_SWAP_MOD) +
		(params->endian.a_swap_mode << A_SWAP_MOD_OFFSET &
		 A_SWAP_MOD);
	gsp_core_reg_write(DES_DATA_ENDIAN(base), value);

	/* layerd image format set*/
	value = params->img_format << DES_IMG_FORMAT_OFFSET & DES_IMG_FORMAT;
	gsp_core_reg_update(DES_DATA_CFG(base), value, DES_IMG_FORMAT);

	/* layerd compress rgb888 set*/
	value = params->compress_r8_en << COMPRESS_R8_OFFSET & COMPRESS_R8;
	gsp_core_reg_update(DES_DATA_CFG(base), value, COMPRESS_R8);
}

void gsp_r1p1_core_run(struct gsp_core *core)
{
	gsp_core_reg_update(GSP_CFG(core->base), GSP_RUN, GSP_RUN);
}

int gsp_r1p1_core_trigger(struct gsp_core *c)
{
	int ret = -1;
	void __iomem *base = NULL;
	struct gsp_kcfg *kcfg = NULL;
	struct gsp_r1p1_core *core = NULL;
	struct gsp_r1p1_cfg *cfg = NULL;

	if (gsp_core_verify(c)) {
		GSP_ERR("gsp_r1p1 core trigger params error\n");
		goto exit;
	}
	core = (struct gsp_r1p1_core *)c;

	kcfg = c->current_kcfg;
	if (gsp_kcfg_verify(kcfg)) {
		GSP_ERR("gsp_r1p1 trigger invalidate kcfg\n");
		goto exit;
	}

	base = c->base;
	/*hardware status check*/
	if (readl(GSP_CFG(base)) & GSP_BUSY) {
		GSP_ERR("core is still busy, can't trigger\n");
		goto exit;
	}

	cfg = (struct gsp_r1p1_cfg *)kcfg->cfg;

	gsp_coef_gen_and_cfg(&c->force_calc, cfg, &c->coef_init,
			     (unsigned long)base, &core->coef_list);
	/*register set*/
	gsp_r1p1_core_misc_reg_set(c, cfg);
	gsp_r1p1_core_l0_reg_set(base, &cfg->l0);
	gsp_r1p1_core_l1_reg_set(base, &cfg->l1);
	gsp_r1p1_core_ld_reg_set(base, &cfg->ld);

	if (gsp_r1p1_core_cfg_is_err(c)) {
		gsp_r1p1_core_err_code_print(c);
		goto exit;
	}

	GSP_DEBUG("gsp_r1p1 configure success\n");
	gsp_r1p1_core_run(c);

	ret = 0;
exit:
	return ret;
};

int gsp_r1p1_core_release(struct gsp_core *c)
{
	/*
	 * there is nothing specific for gsp_r1p1
	 * to do, so here only return zero to
	 * indicate release success.
	 */
	return 0;
}

int __user *gsp_r1p1_core_intercept(void __user *arg, int index)
{
	struct gsp_r1p1_cfg_user __user *cfgs = NULL;

	cfgs = (struct gsp_r1p1_cfg_user __user *)arg;
	return &cfgs[index].ld.common.sig_fd;
}

void gsp_r1p1_core_reset(struct gsp_core *c)
{
	if (IS_ERR_OR_NULL(c)) {
		GSP_ERR("reset parames error\n");
		return;
	}

	gsp_r1p1_core_disable(c);
	gsp_r1p1_core_irq_disable(c);
}

int gsp_r1p1_core_copy_cfg(struct gsp_kcfg *kcfg,
			void *arg, int index)
{
	struct gsp_r1p1_cfg_user *cfg_user_arr = NULL;
	struct gsp_r1p1_cfg_user *cfg_user;
	struct gsp_r1p1_cfg *cfg = NULL;

	if (IS_ERR_OR_NULL(arg)
	    || 0 > index) {
		GSP_ERR("core copy params error\n");
		return -1;
	}

	if (gsp_kcfg_verify(kcfg)) {
		GSP_ERR("core copy kcfg param error\n");
		return -1;
	}

	cfg_user_arr = (struct gsp_r1p1_cfg_user *)arg;
	cfg_user = &cfg_user_arr[index];

	cfg = (struct gsp_r1p1_cfg *)kcfg->cfg;

	/* we must reinitialize cfg again in case data is residual */
	gsp_r1p1_core_cfg_reinit(cfg);

	/* first copy common gsp layer params from user */
	gsp_r1p1_layer_common_copy_from_user(&cfg->l0, &cfg_user->l0);
	gsp_r1p1_layer_common_copy_from_user(&cfg->l1, &cfg_user->l1);
	gsp_r1p1_layer_common_copy_from_user(&cfg->ld, &cfg_user->ld);

	/* second copy specific gsp layer params from user */
	memcpy(&cfg->l0.params, &cfg_user->l0.params,
	       sizeof(struct gsp_r1p1_img_layer_params));
	gsp_layer_set_filled(&cfg->l0.common);

	memcpy(&cfg->l1.params, &cfg_user->l1.params,
	       sizeof(struct gsp_r1p1_osd_layer_params));
	gsp_layer_set_filled(&cfg->l1.common);

	memcpy(&cfg->ld.params, &cfg_user->ld.params,
	       sizeof(struct gsp_r1p1_des_layer_params));
	gsp_layer_set_filled(&cfg->ld.common);

	/* last to copy misc information */
	memcpy(&cfg->misc, &cfg_user->misc, sizeof(struct gsp_r1p1_misc_cfg));

	gsp_r1p1_core_cfg_print(cfg);
	return 0;
}

void gsp_r1p1_core_misc_dump(void __iomem *base)
{
	u32 val = 0;

	GSP_DEBUG("misc dump begin.\n");

	/* dump GSP_CFG */
	val = gsp_core_reg_read(GSP_CFG(base));
	GSP_DEBUG("DIST_RB: %d, L0_EN: %d, L1_EN: %d, SCALE_EN: %d.\n",
		  (val & DIST_RB) >> DIST_RB_OFFSET,
		  (val & L0_EN) >> L0_EN_OFFSET,
		  (val & L1_EN) >> L1_EN_OFFSET,
		  (val & SCALE_EN) >> SCALE_EN_OFFSET);
	GSP_DEBUG("Y2R_OPT: %d, BND_BYPASS: %d\n",
		  (val & Y2R_OPT) >> Y2R_OPT_OFFSET,
		  (val & BND_BYPASS) >> BND_BYPASS_OFFSET);
	GSP_DEBUG("PMARGB_EN: %d, PMARGB_MOD: %d, DITHER_EN: %d.\n",
		  (val & PMARGB_EN) >> PMARGB_EN_OFFSET,
		  (val & PMARGB_MOD) >> PMARGB_MOD_OFFSET,
		  (val & DITHER_EN) >> DITHER_EN_OFFSET);
	GSP_DEBUG("ERR_FLG: %d, ERR_CODE: %d, GSP_BUSY: %d, GSP_RUN: %d\n",
		  (val & ERR_FLG) >> ERR_FLG_OFFSET,
		  (val & ERR_CODE) >> ERR_CODE_OFFSET,
		  (val & GSP_BUSY) >> GSP_BUSY_OFFSET,
		  (val & GSP_RUN) >> GSP_RUN_OFFSET);

	/* dump INT_CFG */
	val = gsp_core_reg_read(GSP_INT_CFG(base));
	GSP_DEBUG("INT_CLR: %d, INT_MOD: %d, INT_EN: %d.\n",
		  (val & INT_CLR) >> INT_CLR_OFFSET,
		  (val & INT_MOD) >> INT_MOD_OFFSET,
		  (val & INT_EN) >> INT_EN_OFFSET);

	GSP_DEBUG("misc dump end.\n");
}

void gsp_r1p1_core_layer0_dump(void __iomem *base)
{
	u32 val = 0;

	GSP_DEBUG("layer0 dump begin.\n");

	/* dump LAYER0_DES_SIZE */
	val = gsp_core_reg_read(LAYER0_DES_SIZE(base));
	GSP_DEBUG("DES_SIZE_Y_L0: %d, DES_SIZE_X_L0: %d.\n",
		  (val & DES_SIZE_Y_L0) >> DES_SIZE_Y_L0_OFFSET,
		  (val & DES_SIZE_X_L0) >> DES_SIZE_X_L0_OFFSET);


	/* dump LAYER0_CFG */
	val = gsp_core_reg_read(LAYER0_CFG(base));
	GSP_DEBUG("COL_TAP_MOD: %d, ROW_TAP_MOD: %d, PALLET_EN_L0: %d.\n",
		  (val & COL_TAP_MOD) >> COL_TAP_MOD_OFFSET,
		  (val & ROW_TAP_MOD) >> ROW_TAP_MOD_OFFSET,
		  (val & PALLET_EN_L0) >> PALLET_EN_L0_OFFSET);
	GSP_DEBUG("CK_EN_L0: %d, ROT_MOD_L0: %d, IMG_FORMAT_L0: %d.\n",
		  (val & CK_EN_L0) >> CK_EN_L0_OFFSET,
		  (val & ROT_MOD_L0) >> ROT_MOD_L0_OFFSET,
		  (val & IMG_FORMAT_L0) >> IMG_FORMAT_L0_OFFSET);


	/* dump LAYER0_Y_ADDR */
	val = gsp_core_reg_read(LAYER0_Y_ADDR(base));
	GSP_DEBUG("Y_BASE_ADDR_L0: %x.\n", val & Y_BASE_ADDR_L0);


	/* dump LAYER0_UV_ADDR */
	val = gsp_core_reg_read(LAYER0_UV_ADDR(base));
	GSP_DEBUG("UV_BASE_ADDR_L0: %x.\n", val & UV_BASE_ADDR_L0);

	/* dump LAYER0_VA_ADDR */
	val = gsp_core_reg_read(LAYER0_VA_ADDR(base));
	GSP_DEBUG("VA_BASE_ADDR_L0: %x.\n", val & VA_BASE_ADDR_L0);

	/* dump LAYER0_CLIP_START */
	val = gsp_core_reg_read(LAYER0_CLIP_START(base));
	GSP_DEBUG("CLIP_START_Y_L0: %d, CLIP_START_X_L0: %d.\n",
		  (val & CLIP_START_Y_L0) >> CLIP_START_Y_L0_OFFSET,
		  (val & CLIP_START_X_L0) >> CLIP_START_X_L0_OFFSET);

	/* dump LAYER0_CLIP_SIZE */
	val = gsp_core_reg_read(LAYER0_CLIP_SIZE(base));
	GSP_DEBUG("CLIP_SIZE_Y_L0: %d, CLIP_SIZE_X_L0: %d.\n",
		  (val & CLIP_SIZE_Y_L0) >> CLIP_SIZE_Y_L0_OFFSET,
		  (val & CLIP_SIZE_X_L0) >> CLIP_SIZE_X_L0_OFFSET);

	/* dump LAYER0_DES_START */
	val = gsp_core_reg_read(LAYER0_DES_START(base));
	GSP_DEBUG("DES_START_Y_L0: %d, DES_START_X_L0: %d.\n",
		  (val & DES_START_Y_L0) >> DES_START_Y_L0_OFFSET,
		  (val & DES_START_X_L0) >> DES_START_X_L0_OFFSET);

	/* dump LAYER0_GREY_RGB */
	val = gsp_core_reg_read(LAYER0_GREY_RGB(base));
	GSP_DEBUG("LAYER0_GREY_R: %d, LAYER0_GREY_G: %d, LAYER0_GREY_B: %d.\n",
		  (val & LAYER0_GREY_R) >> LAYER0_GREY_R_OFFSET,
		  (val & LAYER0_GREY_G) >> LAYER0_GREY_G_OFFSET,
		  (val & LAYER0_GREY_B) >> LAYER0_GREY_B_OFFSET);

	/* dump LAYER0_ENDIAN */
	val = gsp_core_reg_read(LAYER0_ENDIAN(base));
	GSP_DEBUG("A_SWAP_MOD_L0:%d,RGB_SWAP_MOD_L0:%d,VA_ENDIAN_MOD_L0:%d\n",
		  (val & A_SWAP_MOD_L0) >> A_SWAP_MOD_L0_OFFSET,
		  (val & RGB_SWAP_MOD_L0) >> RGB_SWAP_MOD_L0_OFFSET,
		  (val & VA_ENDIAN_MOD_L0) >> VA_ENDIAN_MOD_L0_OFFSET);

	/* dump LAYER0_ALPHA */
	val = gsp_core_reg_read(LAYER0_ALPHA(base));
	GSP_DEBUG("ALPHA_L0: %d.\n", (val & ALPHA_L0) >> ALPHA_L0_OFFSET);

	/* dump LAYER0_CK */
	val = gsp_core_reg_read(LAYER0_CK(base));
	GSP_DEBUG("CK_R_L0: %d, CK_G_L0: %d, CK_B_L0: %d.\n",
		  (val & CK_R_L0) >> CK_R_L0_OFFSET,
		  (val & CK_G_L0) >> CK_G_L0_OFFSET,
		  (val & CK_B_L0) >> CK_B_L0_OFFSET);

	GSP_DEBUG("layer0 dump end.\n");
}

void gsp_r1p1_core_layer1_dump(void __iomem *base)
{
	u32 val = 0;

	GSP_DEBUG("layer1 dump begin.\n");

	/* dump LAYER1_CFG */
	val = gsp_core_reg_read(LAYER1_CFG(base));
	GSP_DEBUG("PALLET_EN_L1: %d, CK_EN_L1: %d, ROT_MOD_L1: %d.\n",
		  (val & PALLET_EN_L1) >> PALLET_EN_L1_OFFSET,
		  (val & CK_EN_L1) >> CK_EN_L1_OFFSET,
		  (val & ROT_MOD_L1) >> ROT_MOD_L1_OFFSET);
	GSP_DEBUG("IMG_FORMAT_L1: %d.\n",
		  (val & IMG_FORMAT_L1) >> IMG_FORMAT_L1_OFFSET);

	/* dump LAYER1_Y_ADDR */
	val = gsp_core_reg_read(LAYER1_Y_ADDR(base));
	GSP_DEBUG("Y_BASE_ADDR_L1: %x.\n", val & Y_BASE_ADDR_L1);

	/* dump LAYER1_UV_BASE */
	val = gsp_core_reg_read(LAYER1_UV_ADDR(base));
	GSP_DEBUG("UV_BASE_ADDR_L1: %x.\n", val & UV_BASE_ADDR_L1);

	/* dump LAYER1_VA_ADDR */
	val = gsp_core_reg_read(LAYER1_VA_ADDR(base));
	GSP_DEBUG("UV_BASE_ADDR_L1: %x.\n", val & VA_BASE_ADDR_L1);

	/* dump LAYER1_PITCH */
	val = gsp_core_reg_read(LAYER1_PITCH(base));
	GSP_DEBUG("PITCH1: %d.\n", (val & PITCH1) >> PITCH1_OFFSET);

	/* dump LAYER1_CLIP_START */
	val = gsp_core_reg_read(LAYER1_CLIP_START(base));
	GSP_DEBUG("CLIP_START_Y_L1: %d, CLIP_START_X_L1: %d.\n",
		  (val & CLIP_START_Y_L1) >> CLIP_START_Y_L1_OFFSET,
		  (val & CLIP_START_X_L1) >> CLIP_START_X_L1_OFFSET);

	/* dump LAYER1_CLIP_SIZE */
	val = gsp_core_reg_read(LAYER1_CLIP_SIZE(base));
	GSP_DEBUG("CLIP_SIZE_Y_L1: %d, CLIP_SIZE_X_L1: %d.\n",
		  (val & CLIP_SIZE_Y_L1) >> CLIP_SIZE_Y_L1_OFFSET,
		  (val & CLIP_SIZE_X_L1) >> CLIP_SIZE_X_L1_OFFSET);

	/* dump LAYER1_DES_START */
	val = gsp_core_reg_read(LAYER1_DES_START(base));
	GSP_DEBUG("DES_START_Y_L1: %d, DES_START_X_L1: %d.\n",
		  (val & DES_START_Y_L1) >> DES_START_Y_L1_OFFSET,
		  (val & DES_START_X_L1) >> DES_START_X_L1_OFFSET);


	/* dump LAYER1_GREY_RGB */
	val = gsp_core_reg_read(LAYER1_GREY_RGB(base));
	GSP_DEBUG("GREY_R_L1: %d, GREY_G_L1: %d, GREY_B_L1: %d.\n",
		  (val & GREY_R_L1) >> GREY_R_L1_OFFSET,
		  (val & GREY_G_L1) >> GREY_G_L1_OFFSET,
		  (val & GREY_B_L1) >> GREY_B_L1_OFFSET);

	/* dump LAYER1_ENDIAN */
	val = gsp_core_reg_read(LAYER1_ENDIAN(base));
	GSP_DEBUG("A_SWAP_MOD_L1: %d, RGB_SWAP_MOD_L1: %d.\n",
		  (val & A_SWAP_MOD_L1) >> A_SWAP_MOD_L1_OFFSET,
		  (val & RGB_SWAP_MOD_L1) >> RGB_SWAP_MOD_L1_OFFSET);
	GSP_DEBUG("VA_ENDIAN_MOD_L1: %d, UV_ENDIAN_MOD_L1: %d.\n",
		  (val & VA_ENDIAN_MOD_L1) >> VA_ENDIAN_MOD_L1_OFFSET,
		  (val & UV_ENDIAN_MOD_L1) >> UV_ENDIAN_MOD_L1_OFFSET);
	GSP_DEBUG("Y_ENDIAN_MOD_L1: %d.\n",
		  (val & Y_ENDIAN_MOD_L1) >> Y_ENDIAN_MOD_L1_OFFSET);

	/* LAYER1_ALPHA */
	val = gsp_core_reg_read(LAYER1_ALPHA(base));
	GSP_DEBUG("ALPHA_L1: %d.\n", (val & ALPHA_L1) >> ALPHA_L1_OFFSET);

	/* LAYER1_CK */
	val = gsp_core_reg_read(LAYER1_CK(base));
	GSP_DEBUG("CK_R_L1: %d, CK_G_L1: %d, CK_B_L1: %d.\n",
		  (val & CK_R_L1) >> CK_R_L1_OFFSET,
		  (val & CK_G_L1) >> CK_G_L1_OFFSET,
		  (val & CK_B_L1) >> CK_B_L1_OFFSET);

	GSP_DEBUG("layer1 dump end.\n");
}

void gsp_r1p1_core_layerd_dump(void __iomem *base)
{
	u32 val = 0;

	GSP_DEBUG("layerd dump begin.\n ");

	/* dump DES_DATA_CFG */
	val = gsp_core_reg_read(DES_DATA_CFG(base));
	GSP_DEBUG("COMPRESS_R8: %d, DES_IMG_FORMAT: %d.\n",
		  (val & COMPRESS_R8) >> COMPRESS_R8_OFFSET,
		  (val & DES_IMG_FORMAT) >> DES_IMG_FORMAT_OFFSET);

	/* dump DES_Y_ADDR */
	val = gsp_core_reg_read(DES_Y_ADDR(base));
	GSP_DEBUG("DES_Y_BASE_ADDR: %x.\n", val);

	/* dump DES_UV_ADDR */
	val = gsp_core_reg_read(DES_UV_ADDR(base));
	GSP_DEBUG("DES_UV_BASE_ADDR: %x.\n", val);

	/* dump DES_V_ADDR */
	val = gsp_core_reg_read(DES_V_ADDR(base));
	GSP_DEBUG("DES_V_BASE_ADDR: %x.\n", val);

	/* dump DES_PITCH */
	val = gsp_core_reg_read(DES_PITCH_REG(base));
	GSP_DEBUG("DES_PITCH: %d.\n", (val & DES_PITCH) >> DES_PITCH_OFFSET);

	/* dump DES_DATA_ENDIAN */
	val = gsp_core_reg_read(DES_DATA_ENDIAN(base));
	GSP_DEBUG("A_SWAP_MOD: %d, RGB_SWAP_MOD: %d, V_ENDIAN_MOD: %d.\n",
		  (val & A_SWAP_MOD) >> A_SWAP_MOD_OFFSET,
		  (val & RGB_SWAP_MOD) >> RGB_SWAP_MOD_OFFSET,
		  (val & V_ENDIAN_MOD) >> V_ENDIAN_MOD_OFFSET);
	GSP_DEBUG("UV_ENDIAN_MOD: %d, Y_ENDIAN_MOD: %d.\n",
		  (val & UV_ENDIAN_MOD) >> UV_ENDIAN_MOD_OFFSET,
		  (val & Y_ENDIAN_MOD) >> Y_ENDIAN_MOD_OFFSET);

	GSP_DEBUG("layerd dump end.\n ");
}

void gsp_r1p1_core_dump(struct gsp_core *core)
{
	void __iomem *base = NULL;

	base = core->base;
	gsp_r1p1_core_misc_dump(base);
	gsp_r1p1_core_layer0_dump(base);
	gsp_r1p1_core_layer1_dump(base);
	gsp_r1p1_core_layerd_dump(base);
}
