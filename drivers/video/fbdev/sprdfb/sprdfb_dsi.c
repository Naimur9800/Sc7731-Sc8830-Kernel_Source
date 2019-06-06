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

#define pr_fmt(fmt) "sprdfb: " fmt

#include "sprdfb_chip_common.h"
#include "sprdfb.h"
#include "sprdfb_panel.h"
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/kgdb.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include "dsi_1_21a/mipi_dsih_local.h"
#include "dsi_1_21a/mipi_dsih_dphy.h"
#include "dsi_1_21a/mipi_dsih_hal.h"
#include "dsi_1_21a/mipi_dsih_api.h"

struct sprd_dsi_data {
	struct clk *clk_dsi;
	bool is_inited;
	/* 0- normal, 1- uninit, 2-abnormal */
	uint32_t status;
	struct sprd_fbdev *dev;
	struct device_node *dsi_node;
	int dsi_irq1;
	int dsi_irq2;
	int dsi_irq3;
	dsih_ctrl_t dsi_inst;
};

static struct sprd_dsi_data dsi_data;
unsigned long sprd_dsi_base = 0;

static int32_t sprd_dsi_set_lp_mode(void);
static int32_t sprd_dsi_set_ulps_mode(void);

static uint32_t dsi_read(unsigned long addr, uint32_t offset)
{
	return readl_relaxed((void __iomem *)(addr + offset));
}

static void dsi_write(unsigned long addr, uint32_t offset,
				    uint32_t data)
{
	writel_relaxed(data, (void __iomem *)(addr + offset));
}

void dsi_write_or(unsigned long addr, unsigned int data)
{
	writel_relaxed((readl_relaxed((void __iomem *)addr) | data),
		     (void __iomem *)addr);
}

void dsi_write_and(unsigned long addr, unsigned int data)
{
	writel_relaxed((readl_relaxed((void __iomem *)addr) & data),
		     (void __iomem *)addr);
}

void dsi_irq_trick(void)
{
	/* enable ack_with_err_0 interrupt */
	DSI_INT_MASK0_SET(0, 0);
}

static irqreturn_t sprd_dsi_isr0(int irq, void *data)
{
	uint32_t reg_val;

	reg_val = dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_ST0);
	pr_debug("[%s](0x%x)!\n", __func__, reg_val);
	if (reg_val & 0x1)
		DSI_INT_MASK0_SET(0, 1);

	return IRQ_HANDLED;
}

static irqreturn_t sprd_dsi_isr1(int irq, void *data)
{
	uint32_t reg_val;
	struct sprd_dsi_data *dsi_data = (struct sprd_dsi_data *)data;
	struct sprd_fbdev *dev = dsi_data->dev;
	dsih_ctrl_t *dsi_instance = &(dsi_data->dsi_inst);
	dphy_t *phy = &(dsi_instance->phy_instance);
	struct panel_spec *panel = dev->panel;
	struct info_mipi *mipi = panel->info.mipi;

	reg_val = dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_ST1);
	pr_debug("[%s](0x%x)!\n", __func__, reg_val);
	if (reg_val & BIT(7)) {
		dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_PWR_UP, 0);
		/* need delay 1us */
		if (NULL == phy) {
			pr_info("[%s]:the phy is null\n", __func__);
			return IRQ_NONE;
		}

		mipi_dsih_dphy_configure(phy, mipi->lan_number, mipi->phy_feq);

		dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_PWR_UP, 1);
	}

	return IRQ_HANDLED;
}

static irqreturn_t sprd_dsi_isr2(int irq, void *data)
{
	struct sprd_dsi_data *dsi_data = (struct sprd_dsi_data *)data;
	struct sprd_fbdev *dev = dsi_data->dev;

	pr_debug("[%s]\n", __func__);

	if (MIPI_PLL_TYPE_2 == dev->capability.mipi_pll_type) {
		dsi_write_or(SPRD_MIPI_DSIC_BASE +
				     R_DSI_HOST_MIPI_PLL_INT_CLR, BIT(0));
		pr_info("[%s], mipi pll interrupt!\n", __func__);
		dsi_write_or(SPRD_MIPI_DSIC_BASE +
				     R_DSI_HOST_MIPI_PLL_INT_EN, BIT(0));
	}
	return IRQ_HANDLED;
}

static void dsi_enable(struct sprd_fbdev *dev)
{
	pr_debug("[%s]\n", __func__);

	regmap_update_bits(dev->aon_apb_gpr, REG_AON_APB_PWR_CTRL,
	    BIT_AON_APB_MIPI_DSI_PS_PD_S, (u32)(~BIT_AON_APB_MIPI_DSI_PS_PD_S));
	regmap_update_bits(dev->aon_apb_gpr, REG_AON_APB_PWR_CTRL,
	    BIT_AON_APB_MIPI_DSI_PS_PD_L, (u32)(~BIT_AON_APB_MIPI_DSI_PS_PD_L));
	regmap_update_bits(dev->ap_ahb_gpr, REG_AP_AHB_MISC_CKG_EN,
	    BIT_AP_AHB_DPHY_REF_CKG_EN, (u32)BIT_AP_AHB_DPHY_REF_CKG_EN);
	regmap_update_bits(dev->ap_ahb_gpr, REG_AP_AHB_MISC_CKG_EN,
	    BIT_AP_AHB_DPHY_CFG_CKG_EN, (u32)BIT_AP_AHB_DPHY_CFG_CKG_EN);
	regmap_update_bits(dev->ap_ahb_gpr, REG_AP_AHB_AHB_EB,
		BIT_AP_AHB_DSI_EB, (u32)BIT_AP_AHB_DSI_EB);
}

static void dsi_disable(struct sprd_fbdev *dev)
{
	pr_debug("[%s]\n", __func__);

	regmap_update_bits(dev->aon_apb_gpr, REG_AON_APB_PWR_CTRL,
	    BIT_AON_APB_MIPI_DSI_PS_PD_S, (u32)(BIT_AON_APB_MIPI_DSI_PS_PD_S));
	regmap_update_bits(dev->aon_apb_gpr, REG_AON_APB_PWR_CTRL,
	    BIT_AON_APB_MIPI_DSI_PS_PD_L, (u32)(BIT_AON_APB_MIPI_DSI_PS_PD_L));
	regmap_update_bits(dev->ap_ahb_gpr, REG_AP_AHB_MISC_CKG_EN,
	    BIT_AP_AHB_DPHY_REF_CKG_EN, (u32)(~BIT_AP_AHB_DPHY_REF_CKG_EN));
	regmap_update_bits(dev->ap_ahb_gpr, REG_AP_AHB_MISC_CKG_EN,
	    BIT_AP_AHB_DPHY_CFG_CKG_EN, (u32)(~BIT_AP_AHB_DPHY_CFG_CKG_EN));
	regmap_update_bits(dev->ap_ahb_gpr, REG_AP_AHB_AHB_EB,
	    BIT_AP_AHB_DSI_EB, (u32)(~BIT_AP_AHB_DSI_EB));
}

static void sprd_dsi_print_global_config(struct sprd_fbdev *dev)
{
	u32 reg_val0, reg_val1, reg_val2;

	regmap_read(dev->ap_ahb_gpr, REG_AP_AHB_MISC_CKG_EN, &reg_val0);
	regmap_read(dev->ap_ahb_gpr, REG_AP_AHB_AHB_EB, &reg_val1);
	regmap_read(dev->aon_apb_gpr, REG_AON_APB_PWR_CTRL, &reg_val2);
	pr_debug(
	    "dsi: 0x20E00040 = 0x%x, 0x20E00000 = 0x%x, 0x402E0024 = 0x%x\n",
	     reg_val0, reg_val1, reg_val2);
}

static void dsi_reset(struct sprd_fbdev *dev)
{
	regmap_update_bits(dev->ap_ahb_gpr, REG_AP_AHB_AHB_RST,
		BIT_AP_AHB_DSI_SOFT_RST, BIT_AP_AHB_DSI_SOFT_RST);
	udelay(10);
	regmap_update_bits(dev->ap_ahb_gpr, REG_AP_AHB_AHB_RST,
		BIT_AP_AHB_DSI_SOFT_RST, (u32)(~BIT_AP_AHB_DSI_SOFT_RST));
}

static int32_t dsi_edpi_setbuswidth(struct info_mipi *mipi)
{
	dsih_color_coding_t color_coding = 0;

	pr_debug("[%s]\n", __func__);

	switch (mipi->video_bus_width) {
	case 16:
		color_coding = COLOR_CODE_16BIT_CONFIG1;
		break;
	case 18:
		color_coding = COLOR_CODE_18BIT_CONFIG1;
		break;
	case 24:
		color_coding = COLOR_CODE_24BIT;
		break;
	default:
		pr_err("[%s] fail, invalid video_bus_width\n", __func__);
		return 0;
	}
	dsi_write_and((SPRD_MIPI_DSIC_BASE + R_DSI_HOST_DPI_COLOR_CODE),
			      0xfffffff0);
	dsi_write_or((SPRD_MIPI_DSIC_BASE + R_DSI_HOST_DPI_COLOR_CODE),
			     color_coding);
	return 0;
}

static int32_t dsi_edpi_init(void)
{
	dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_EDPI_CMD_SIZE, 0x500);

	return 0;
}

int32_t dsi_dpi_init(struct sprd_fbdev *dev)
{
	dsih_dpi_video_t dpi_param = {
		0
	};
	dsih_error_t result;
	struct panel_spec *panel = dev->panel;
	struct info_mipi *mipi = panel->info.mipi;

	dpi_param.no_of_lanes = mipi->lan_number;
	dpi_param.byte_clock = mipi->phy_feq / 8;

	if (dev->dev_id == SPRD_FB_FPGA)
		dpi_param.pixel_clock = 6500;
	else
		dpi_param.pixel_clock = dev->dpi_clock / 1000;

	dpi_param.max_hs_to_lp_cycles = 4;
	dpi_param.max_lp_to_hs_cycles = 15;
	dpi_param.max_clk_hs_to_lp_cycles = 4;
	dpi_param.max_clk_lp_to_hs_cycles = 15;

	switch (mipi->video_bus_width) {
	case 16:
		dpi_param.color_coding = COLOR_CODE_16BIT_CONFIG1;
		break;
	case 18:
		dpi_param.color_coding = COLOR_CODE_18BIT_CONFIG1;
		break;
	case 24:
		dpi_param.color_coding = COLOR_CODE_24BIT;
		break;
	default:
		pr_err("[%s] fail, invalid video_bus_width\n", __func__);
		break;
	}

	if (SPRD_POLARITY_POS == mipi->h_sync_pol)
		dpi_param.h_polarity = 1;

	if (SPRD_POLARITY_POS == mipi->v_sync_pol)
		dpi_param.v_polarity = 1;

	if (SPRD_POLARITY_POS == mipi->de_pol)
		dpi_param.data_en_polarity = 1;

	dpi_param.h_active_pixels = panel->width;
	dpi_param.h_sync_pixels = mipi->timing->hsync;
	dpi_param.h_back_porch_pixels = mipi->timing->hbp;
	dpi_param.h_total_pixels = panel->width + mipi->timing->hsync +
				mipi->timing->hbp + mipi->timing->hfp;

	dpi_param.v_active_lines = panel->height;
	dpi_param.v_sync_lines = mipi->timing->vsync;
	dpi_param.v_back_porch_lines = mipi->timing->vbp;
	dpi_param.v_total_lines = panel->height + mipi->timing->vsync +
				mipi->timing->vbp + mipi->timing->vfp;

	dpi_param.receive_ack_packets = 0;
	dpi_param.video_mode = VIDEO_BURST_WITH_SYNC_PULSES;
	dpi_param.virtual_channel = 0;
	dpi_param.is_18_loosely = 0;

	result = mipi_dsih_dpi_video(&(dsi_data.dsi_inst), &dpi_param);
	if (result != OK) {
		pr_err("mipi_dsih_dpi_video fail (%d)!\n", result);
		return -1;
	}

	return 0;
}

static void dsi_log_error(const char *string)
{
	pr_err("%s\n", string);
}

static int32_t dsi_module_init(struct sprd_fbdev *dev)
{
	int ret = 0, phy_clk;
	dsih_ctrl_t *dsi_instance = &(dsi_data.dsi_inst);
	dphy_t *phy = &(dsi_instance->phy_instance);

	pr_debug("[%s]\n", __func__);

	if (dsi_data.is_inited) {
		pr_info("dsi_module_init. is_inited = true!");
		return 0;
	}

	ret = of_property_read_u32(dsi_data.dsi_node,
			"sprd,phy-clk", &phy_clk);
	if (ret)
		pr_err("[%s] read sprd,phy-clk fail from dt\n",
			__func__);

	phy->address = SPRD_MIPI_DSIC_BASE;
	phy->core_read_function = dsi_read;
	phy->core_write_function = dsi_write;
	phy->log_error = dsi_log_error;
	phy->log_info = NULL;
	phy->reference_freq = phy_clk;
	dsi_instance->address = SPRD_MIPI_DSIC_BASE;
	dsi_instance->core_read_function = dsi_read;
	dsi_instance->core_write_function = dsi_write;
	dsi_instance->log_error = dsi_log_error;
	dsi_instance->log_info = NULL;
	/* in our rtl implementation, this is max rd time,
	  * not bta time and use 15bits */
	dsi_instance->max_bta_cycles = 0x6000;

	dsi_data.dsi_irq1 = irq_of_parse_and_map(dsi_data.dsi_node, 0);
	pr_debug("dsi irq_num_1 = %d\n",  dsi_data.dsi_irq1);

	ret = devm_request_irq(dev->of_dev, dsi_data.dsi_irq1, sprd_dsi_isr0,
			IRQF_DISABLED, "DSI_INT0", &dsi_data);
	if (ret)
		pr_err("dsi failed to request irq int0!\n");
	else
		pr_debug("dsi request irq int0 OK!\n");

	dsi_data.dsi_irq2 = irq_of_parse_and_map(dsi_data.dsi_node, 1);
	pr_debug("dsi irq_num_2 = %d\n", dsi_data.dsi_irq2);

	ret = devm_request_irq(dev->of_dev, dsi_data.dsi_irq2, sprd_dsi_isr1,
			IRQF_DISABLED, "DSI_INT1", &dsi_data);
	if (ret)
		pr_err("dsi failed to request irq int1!\n");
	else
		pr_debug("dsi request irq int1 OK!\n");

	if ((dev->capability.mipi_pll_type > MIPI_PLL_TYPE_NONE)
	    && (dev->capability.mipi_pll_type < MIPI_PLL_TYPE_LIMIT)) {
		dsi_data.dsi_irq3 = irq_of_parse_and_map(dsi_data.dsi_node, 2);
		pr_debug("dsi irq_num_3 = %d\n", dsi_data.dsi_irq3);

		ret = devm_request_irq(dev->of_dev, dsi_data.dsi_irq3,
			sprd_dsi_isr2, IRQF_DISABLED, "DSI_INT2", &dsi_data);
		if (ret)
			pr_err("dsi failed to request irq int2!\n");
		else
			pr_debug("dsi request irq int2 OK!\n");

		if (MIPI_PLL_TYPE_1 == dev->capability.mipi_pll_type) {
			dsi_write_and(SPRD_MIPI_DSIC_BASE +
					      R_DSI_HOST_MIPI_PLL_INT_MSK,
					      0xFFFFFFFE);
		} else if (MIPI_PLL_TYPE_2 == dev->capability.mipi_pll_type) {
			dsi_write_and(SPRD_MIPI_DSIC_BASE +
					      R_DSI_HOST_MIPI_PLL_INT_MSK,
					      0xFFFFFFFE);
			dsi_write_or(SPRD_MIPI_DSIC_BASE +
					     R_DSI_HOST_MIPI_PLL_INT_EN,
					     BIT(0));
		}
	}

	dsi_data.is_inited = true;

	pr_debug("dsi_module_init done !!\n");

	return 0;
}

#ifdef CONFIG_FB_DYNAMIC_FREQ_SCALING
int sprd_dsi_chg_dphy_freq(struct sprd_fbdev *fb_dev, u32 dphy_freq)
{
	dsih_error_t result = OK;
	dsih_ctrl_t *ctrl = &(dsi_data.dsi_inst);
	struct info_mipi *mipi = fb_dev->panel->info.mipi;

	if (ctrl->status == INITIALIZED) {
		pr_info("Let's do update D-PHY frequency(%u)\n", dphy_freq);
		ctrl->phy_instance.phy_keep_work = true;
		result = mipi_dsih_dphy_configure(&ctrl->phy_instance,
						  mipi->lan_number, dphy_freq);
		if (result != OK) {
			pr_err("[%s]: mipi_dsih_dphy_configure fail (%d)\n",
			       __func__, result);
			ctrl->phy_instance.phy_keep_work = false;
			return -1;
		}
		ctrl->phy_instance.phy_keep_work = false;
	}

	mipi->phy_feq = dphy_freq;

	return 0;
}
#else
int sprd_dsi_chg_dphy_freq(struct sprd_fbdev *fb_dev, u32 dphy_freq)
{
	return -EINVAL;
}
#endif

int32_t sprd_dsih_init(struct sprd_fbdev *dev)
{
	dsih_error_t result = OK;
	dsih_ctrl_t *dsi_instance = &(dsi_data.dsi_inst);
	dphy_t *phy = &(dsi_instance->phy_instance);
	struct info_mipi *mipi = dev->panel->info.mipi;
	int wait_count = 0;

	pr_debug("[%s]\n", __func__);
	dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_MSK0,
				0x1fffff);
	dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_MSK1,
				0x3ffff);

	if (SPRD_MIPI_MODE_CMD == mipi->work_mode)
		dsi_edpi_init();

	dsi_instance->phy_feq = dev->panel->info.mipi->phy_feq;
	dsi_instance->max_lanes = dev->panel->info.mipi->lan_number;
	dsi_instance->color_mode_polarity =
	    dev->panel->info.mipi->color_mode_pol;
	dsi_instance->shut_down_polarity = dev->panel->info.mipi->shut_down_pol;

	result = mipi_dsih_open(dsi_instance);
	if (OK != result) {
		pr_err("[%s]: mipi_dsih_open fail (%d)!\n", __func__, result);
		dsi_data.status = 1;
		return -1;
	}

	result = mipi_dsih_dphy_configure(phy, mipi->lan_number, mipi->phy_feq);
	if (OK != result) {
		pr_err("[%s]: mipi_dsih_dphy_configure fail (%d)!\n",
				__func__, result);
		dsi_data.status = 1;
		return -1;
	}

	while (5 != (dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_PHY_STATUS) & 5)
	       && (wait_count < 100000)) {
		udelay(3);
		if (0x0 == ++wait_count % 1000) {
			pr_warn("[%s] warning: busy waiting!\n",
			       __func__);
		}
	}
	if (wait_count >= 100000) {
		pr_err("[%s] Errior: dsi phy can't be locked!!!\n",
		       __func__);
	}

	if (SPRD_MIPI_MODE_CMD == mipi->work_mode)
		dsi_edpi_setbuswidth(mipi);

	result = mipi_dsih_enable_rx(dsi_instance, 1);
	if (OK != result) {
		pr_err("[%s]: mipi_dsih_enable_rx fail (%d)!\n",
		       __func__, result);
		dsi_data.status = 1;
		return -1;
	}

	result = mipi_dsih_ecc_rx(dsi_instance, 1);
	if (OK != result) {
		pr_err("[%s]: mipi_dsih_ecc_rx fail (%d)!\n", __func__,
		       result);
		dsi_data.status = 1;
		return -1;
	}

	result = mipi_dsih_eotp_rx(dsi_instance, 1);
	if (OK != result) {
		pr_err("[%s]: mipi_dsih_eotp_rx fail (%d)!\n", __func__,
		       result);
		dsi_data.status = 1;
		return -1;
	}

	result = mipi_dsih_eotp_tx(dsi_instance, 1);
	if (OK != result) {
		pr_err("[%s]: mipi_dsih_eotp_tx fail (%d)!\n", __func__,
		       result);
		dsi_data.status = 1;
		return -1;
	}

	if (SPRD_MIPI_MODE_VIDEO == mipi->work_mode)
		dsi_dpi_init(dev);
	mipi_dsih_dphy_enable_nc_clk(&(dsi_instance->phy_instance), false);
	dsi_data.status = 0;

	return 0;
}

int32_t sprd_dsi_init(struct sprd_fbdev *dev)
{
	dsih_error_t result = OK;
	struct resource r;
	struct device_node *node;
	struct device_node *ep, *np;

	dsi_data.dev = dev;

	pr_debug("[%s]\n", __func__);

	dsi_data.dsi_node = of_find_compatible_node(NULL, NULL, "sprd,dsi-21a");
	node = dsi_data.dsi_node;
	do {
		np = of_get_next_child(node, NULL);
		if (!np) {
			pr_err("Unable to find dsi child port\n");
			break;
		}
		node = np;
	} while (of_node_cmp(np->name, "endpoint") != 0);

	if (np) {
		ep = of_parse_phandle(np, "remote-endpoint", 0);
		if (!ep)
			pr_err("Unable to find dsi remote-endpoint\n");
	}
	of_node_put(ep);
	of_node_put(np);

	if (0 != of_address_to_resource(dsi_data.dsi_node, 0, &r)) {
		pr_err(
			"sprd_dsi_init fail. (can't get register base address)\n");
		return -ENODEV;
	}

	sprd_dsi_base = (unsigned long)devm_ioremap_nocache(dev->of_dev,
				r.start, resource_size(&r));
	if (0 == sprd_dsi_base) {
		pr_err("sprd_dsi_base = 0!(0x%lx)\n",
			(unsigned long)r.start);
		return -ENOMEM;
	}

	pr_debug("sprd_dsi_base = (0x%lx)\n", (unsigned long)r.start);

	if (!dsi_data.is_inited) {
		/* Initialize DSI */
		pr_info("[%s]: Initializing dsi\n",
			__func__);
		dsi_enable(dev);
		dsi_reset(dev);
		dsi_module_init(dev);
		result = sprd_dsih_init(dev);
	} else {
		/* Resume DSI*/
		pr_info("[%s]: resume\n", __func__);
		dsi_enable(dev);
		dsi_reset(dev);
		result = sprd_dsih_init(dev);
	}

	sprd_dsi_print_global_config(dev);

	return result;
}

int32_t sprd_dsi_uninit(struct sprd_fbdev *dev)
{
	dsih_error_t result;
	dsih_ctrl_t *dsi_instance = &(dsi_data.dsi_inst);

	pr_info("[%s]\n", __func__);

	mipi_dsih_dphy_enable_hs_clk(&(dsi_instance->phy_instance), false);
	result = mipi_dsih_close(&(dsi_data.dsi_inst));
	dsi_instance->status = NOT_INITIALIZED;
	dsi_data.status = 1;

	if (OK != result) {
		pr_err("[%s]: sprd_dsi_uninit fail (%d)!\n", __func__, result);
		return -1;
	}

	msleep(10);
	dsi_disable(dev);

	return 0;
}

int32_t sprd_dsi_suspend(struct sprd_fbdev *dev)
{
	dsih_ctrl_t *dsi_instance = &(dsi_data.dsi_inst);

	pr_info("[%s]\n", __func__);
	/* sprd_dsi_uninit(dev); */
	/* mipi_dsih_dphy_close(&(dsi_instance->phy_instance)); */
	/* mipi_dsih_dphy_shutdown(&(dsi_instance->phy_instance), 0); */
	mipi_dsih_hal_power(dsi_instance, 0);

	return 0;
}

int32_t sprd_dsi_resume(struct sprd_fbdev *dev)
{
	dsih_ctrl_t *dsi_instance = &(dsi_data.dsi_inst);

	pr_info("[%s]\n", __func__);

	mipi_dsih_hal_power(dsi_instance, 1);

	return 0;
}

int32_t sprd_dsi_ready(struct sprd_fbdev *dev)
{
	struct info_mipi *mipi = dev->panel->info.mipi;

	if (SPRD_MIPI_MODE_CMD == mipi->work_mode) {
		mipi_dsih_cmd_mode(&(dsi_data.dsi_inst), 1);
		mipi_dsih_dphy_enable_hs_clk(&(dsi_data.dsi_inst.phy_instance),
					     true);
		dsi_write(SPRD_MIPI_DSIC_BASE,
					R_DSI_HOST_CMD_MODE_CFG, 0x0);
	} else {
		mipi_dsih_dphy_enable_hs_clk(&(dsi_data.dsi_inst.phy_instance),
					     true);
		mipi_dsih_video_mode(&(dsi_data.dsi_inst), 1);
		dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_PWR_UP, 0);
		udelay(100);
		dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_PWR_UP, 1);
		usleep_range(3000, 3500);
		dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_ST0);
		dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_ST1);
	}

	dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_MSK0, 0x0);
	dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_MSK1,
				0x800);
	return 0;
}

int32_t sprd_dsi_before_panel_reset(struct sprd_fbdev *dev)
{
	sprd_dsi_set_lp_mode();
	return 0;
}

int32_t sprd_dsi_enter_ulps(struct sprd_fbdev *dev)
{
	sprd_dsi_set_ulps_mode();
	return 0;
}

uint32_t sprd_dsi_get_status(struct sprd_fbdev *dev)
{
	return dsi_data.status;
}

static int32_t sprd_dsi_set_cmd_mode(void)
{
	mipi_dsih_cmd_mode(&(dsi_data.dsi_inst), 1);
	return 0;
}

static int32_t sprd_dsi_set_video_mode(void)
{
	mipi_dsih_video_mode(&(dsi_data.dsi_inst), 1);
	return 0;
}

static int32_t sprd_dsi_set_lp_mode(void)
{
	pr_debug("[%s]\n", __func__);
	mipi_dsih_cmd_mode(&(dsi_data.dsi_inst), 1);
	dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_CMD_MODE_CFG,
				0x01ffff00);
	mipi_dsih_dphy_enable_hs_clk(&(dsi_data.dsi_inst.phy_instance), false);

	return 0;
}

static int32_t sprd_dsi_set_hs_mode(void)
{
	pr_debug("[%s]\n", __func__);
	mipi_dsih_cmd_mode(&(dsi_data.dsi_inst), 1);
	dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_CMD_MODE_CFG, 0x0);
	mipi_dsih_dphy_enable_hs_clk(&(dsi_data.dsi_inst.phy_instance), true);

	return 0;
}

static int32_t sprd_dsi_set_ulps_mode(void)
{
	pr_debug("[%s]\n", __func__);
	mipi_dsih_ulps_mode(&(dsi_data.dsi_inst), 1);

	return 0;
}

static int32_t sprd_dsi_set_data_lp_mode(void)
{
	pr_debug("[%s]\n", __func__);

	dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_CMD_MODE_CFG, 0x1fff);

	return 0;
}

static int32_t sprd_dsi_set_data_hs_mode(void)
{
	int active_mode = mipi_dsih_active_mode(&(dsi_data.dsi_inst));

	pr_debug("[%s]\n", __func__);

	if (1 == active_mode)
		dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_CMD_MODE_CFG, 0x1);
	else
		dsi_write(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_CMD_MODE_CFG, 0x0);

	return 0;
}

static int32_t sprd_dsi_gen_write(uint8_t *param, uint16_t param_length)
{
	dsih_error_t result;

	result =
	    mipi_dsih_gen_wr_cmd(&(dsi_data.dsi_inst), 0, param, param_length);

	if (OK != result) {
		pr_err("[%s] error (%d)\n", __func__, result);
		return -1;
	}

	return 0;
}

static int32_t sprd_dsi_gen_read(uint8_t *param, uint16_t param_length,
				   uint8_t bytes_to_read, uint8_t *read_buffer)
{
	uint16_t result;
	uint32_t reg_val, reg_val_1, reg_val_2;

	result =
	    mipi_dsih_gen_rd_cmd(&(dsi_data.dsi_inst), 0, param, param_length,
				 bytes_to_read, read_buffer);

	reg_val = dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_PHY_STATUS);
	reg_val_1 = dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_ST0);
	reg_val_2 = dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_ST1);

	if (0 != (reg_val & 0x2)) {
		pr_err("[%s] mipi read hang (0x%x)!\n", __func__,
		       reg_val);
		dsi_data.status = 2;
		result = 0;
	}

	if (0 != (reg_val_1 & 0x701)) {
		pr_err("[%s] mipi read status error!(0x%x, 0x%x)\n",
		       __func__, reg_val_1, reg_val_2);
		result = 0;
	}

	if (0 == result) {
		pr_err("[%s] return error (%d)\n", __func__, result);
		return -1;
	}
	return 0;
}

static int32_t sprd_dsi_dcs_write(uint8_t *param, uint16_t param_length)
{
	dsih_error_t result;

	result =
	    mipi_dsih_dcs_wr_cmd(&(dsi_data.dsi_inst), 0, param, param_length);

	if (OK != result) {
		pr_err("[%s] error (%d)\n", __func__, result);
		return -1;
	}
	return 0;
}

static int32_t sprd_dsi_dcs_read(uint8_t command, uint8_t bytes_to_read,
				   uint8_t *read_buffer)
{
	uint16_t result;
	uint32_t reg_val, reg_val_1, reg_val_2;

	result = mipi_dsih_dcs_rd_cmd(&(dsi_data.dsi_inst), 0, command,
			bytes_to_read, read_buffer);
	reg_val = dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_PHY_STATUS);
	reg_val_1 = dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_ST0);
	reg_val_2 = dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_ST1);

	if (0 != (reg_val & 0x2)) {
		pr_err("[%s] mipi read hang (0x%x)!\n", __func__, reg_val);
		dsi_data.status = 2;
		result = 0;
	}

	if (0 != (reg_val_1 & 0x701)) {
		pr_err("[%s] mipi read status error!(0x%x, 0x%x)\n",
		       __func__, reg_val_1, reg_val_2);
		result = 0;
	}

	if (0 == result) {
		pr_err("[%s] return error (%d)\n", __func__, result);
		return -1;
	}
	return 0;
}

static int32_t sprd_dsi_force_write(uint8_t data_type, uint8_t *p_params,
				    uint16_t param_length)
{
	int32_t ret = 0;

	ret = mipi_dsih_gen_wr_packet(&(dsi_data.dsi_inst), 0, data_type,
			p_params, param_length);

	return ret;
}

static int32_t sprd_dsi_force_read(uint8_t command, uint8_t bytes_to_read,
				   uint8_t *read_buffer)
{
	int32_t ret = 0;
	uint32_t reg_val, reg_val_1, reg_val_2;

	ret =
	    mipi_dsih_gen_rd_packet(&(dsi_data.dsi_inst), 0, 6, 0, command,
				    bytes_to_read, read_buffer);

	reg_val = dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_PHY_STATUS);
	reg_val_1 = dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_ST0);
	reg_val_2 = dsi_read(SPRD_MIPI_DSIC_BASE, R_DSI_HOST_INT_ST1);

	if (0 != (reg_val & 0x2)) {
		pr_err("[%s] mipi read hang (0x%x)!\n", __func__, reg_val);
		dsi_data.status = 2;
		ret = 0;
	}

	if (0 != (reg_val_1 & 0x701)) {
		pr_err("[%s] mipi read status error!(0x%x, 0x%x)\n",
		       __func__, reg_val_1, reg_val_2);
		ret = 0;
	}

	if (0 == ret) {
		pr_err("[%s] return error (%d)\n", __func__, ret);
		return -1;
	}

	return 0;
}

static int32_t sprd_dsi_eotp_set(uint8_t rx_en, uint8_t tx_en)
{
	dsih_ctrl_t *curInstancePtr = &(dsi_data.dsi_inst);

	if (0 == rx_en)
		mipi_dsih_eotp_rx(curInstancePtr, 0);
	else if (1 == rx_en)
		mipi_dsih_eotp_rx(curInstancePtr, 1);
	if (0 == tx_en)
		mipi_dsih_eotp_tx(curInstancePtr, 0);
	else if (1 == tx_en)
		mipi_dsih_eotp_tx(curInstancePtr, 1);
	return 0;
}

struct ops_mipi sprd_fb_mipi_ops = {
	.mipi_set_cmd_mode = sprd_dsi_set_cmd_mode,
	.mipi_set_video_mode = sprd_dsi_set_video_mode,
	.mipi_set_lp_mode = sprd_dsi_set_lp_mode,
	.mipi_set_hs_mode = sprd_dsi_set_hs_mode,
	.mipi_set_data_lp_mode = sprd_dsi_set_data_lp_mode,
	.mipi_set_data_hs_mode = sprd_dsi_set_data_hs_mode,
	.mipi_gen_write = sprd_dsi_gen_write,
	.mipi_gen_read = sprd_dsi_gen_read,
	.mipi_dcs_write = sprd_dsi_dcs_write,
	.mipi_dcs_read = sprd_dsi_dcs_read,
	.mipi_force_write = sprd_dsi_force_write,
	.mipi_force_read = sprd_dsi_force_read,
	.mipi_eotp_set = sprd_dsi_eotp_set,
};
