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

#ifndef _ISP_INTERFACE_H_
#define _ISP_INTERFACE_H_

#include <linux/of.h>
#include <linux/platform_device.h>

#include "cam_hw.h"
#include "cam_types.h"


#define ISP_MAX_LINE_WIDTH		2560

enum isp_path_cfg_cmd {
	ISP_PATH_CFG_COMMON,
	ISP_PATH_CFG_OUTPUT_BUF,
	ISP_PATH_CFG_OUTPUT_RESERVED_BUF,
	ISP_PATH_CFG_INPUT_SIZE,
	ISP_PATH_CFG_INPUT_RECT,
	ISP_PATH_CFG_OUTPUT_SIZE,
};


struct isp_path_cfg_param {
	struct img_endian endian;
	struct img_size input_size;
	struct img_trim input_trim;
	struct img_size output_size;
};


struct isp_pipe_ops {
	int (*open)(void *isp_handle, void *arg);
	int (*close)(void *isp_handle);
	int (*reset)(void *isp_handle, void *arg);

	int (*get_path)(void *isp_handle, void *arg);
	int (*put_path)(void *isp_handle, uint32_t put_path_id);
	int (*cfg_path)(void *isp_handle, enum isp_path_cfg_cmd cfg_cmd,
				uint32_t cfg_path_id, void *param);
	int (*proc_frame)(void *isp_handle, void *param, uint32_t in_path_id);
	int (*set_callback)(void *isp_handle, uint32_t path_id,
					isp_dev_callback cb, void *priv_data);
	int (*update_clk)(void *isp_handle, void *arg);
};

struct isp_pipe_ops *get_isp_ops(void);

void *get_isp_pipe_dev(void);
int put_isp_pipe_dev(void *isp_handle);

int sprd_isp_debugfs_init(void);

int sprd_isp_parse_dt(struct device_node *dn,
		struct sprd_cam_hw_info **isp_hw,
		uint32_t *isp_count);
#endif
