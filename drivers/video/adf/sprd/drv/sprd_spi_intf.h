#ifndef _SPRD_SPI_H_
#define _SPRD_SPI_H_

#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <asm/unaligned.h>
#include "disp_lib.h"
#include "sprd_adf_adapter.h"
#include "sprd_dispc.h"
#include "sprd_panel.h"

enum spi_color_coding {
	COLOR_CODE_16BIT_CONFIG1	=  0,
	COLOR_CODE_16BIT_CONFIG2	=  1,
	COLOR_CODE_16BIT_CONFIG3	=  2,
	COLOR_CODE_MAX
};

struct spi_context {
	/* SPI CLK frequency */
	u32 freq;
	void *current_base;
	uint32_t refresh_len;
	uint32_t lcm_id;
};


struct edpi_video_param {
	int te;
	/* packet size of write memory command */
};

struct spi_core_ops {
	bool (*check_version)(struct spi_context *ctx);
};

struct spi_glb_ops {
	int (*parse_dt)(struct spi_context *ctx,
			struct device_node *np);
};


struct sprd_spi {
	struct device dev;
	struct spi_device *spi_dev;
	struct spi_context ctx;
	struct spi_core_ops *core;
	struct spi_glb_ops *glb;
	struct notifier_block nb;
	struct panel_info *panel;
	struct completion spi_complete;
	void *current_base;
	uint32_t refresh_len;
};

extern int sprd_spi_refresh(struct sprd_adf_hwlayer *hwlayer, uint8_t bg_flag);

#endif /* _SPRD_DSI_H_ */
