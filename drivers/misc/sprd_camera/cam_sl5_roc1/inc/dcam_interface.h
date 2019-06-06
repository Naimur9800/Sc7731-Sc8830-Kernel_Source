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

#ifndef _DCAM_INTERFACE_H_
#define _DCAM_INTERFACE_H_

#include <linux/of.h>
#include <linux/platform_device.h>

#include "cam_hw.h"
#include "cam_types.h"


#define DCAM_SCALE_DOWN_MAX 8


enum dcam_id {
	DCAM_ID_0 = 0,
	DCAM_ID_1,
	DCAM_ID_2,
	DCAM_ID_MAX,
};

enum dcam_path_cfg_cmd {
	DCAM_PATH_CFG_COMMON,
	DCAM_PATH_CFG_OUTPUT_BUF,
	DCAM_PATH_CFG_OUTPUT_RESERVED_BUF,
	DCAM_PATH_CFG_INPUT_SIZE,
	DCAM_PATH_CFG_INPUT_RECT,
	DCAM_PATH_CFG_OUTPUT_SIZE,
};


enum dcam_ioctrl_cmd {
	DCAM_IOCTL_CFG_CAP,
	DCAM_IOCTL_CFG_STATIS_BUF,
	DCAM_IOCTL_INIT_STATIS_Q,
	DCAM_IOCTL_DEINIT_STATIS_Q,
};


struct dcam_cap_cfg {
	uint32_t sensor_if;  /* MIPI CSI-2 */
	uint32_t format;  /* input color format */
	uint32_t mode;   /* single or multi mode. */
	uint32_t data_bits;
	uint32_t pattern; /* bayer mode for rgb, yuv pattern for yuv */
	uint32_t href;
	uint32_t frm_deci;
	uint32_t frm_skip;
	uint32_t x_factor;
	uint32_t y_factor;
	uint32_t is_4in1;
	struct img_trim cap_size;
};


struct dcam_path_cfg_param {
	uint32_t is_raw;
	uint32_t is_loose;
	uint32_t frm_deci;
	uint32_t frm_skip;

	struct img_endian endian;

	struct img_size input_size;
	struct img_trim input_trim;
	struct img_size output_size;
};

struct dcam_pipe_ops {
	int (*open)(void *handle, void *arg);
	int (*close)(void *handle);
	int (*reset)(void *handle);
	int (*start)(void *handle);
	int (*stop)(void *handle);
	int (*get_path)(void *handle, void *arg);
	int (*put_path)(void *handle, uint32_t path_id);
	int (*cfg_path)(void *dcam_handle,
				enum dcam_path_cfg_cmd cfg_cmd,
				uint32_t path_id, void *param);
	int (*ioctl)(void *handle, enum dcam_ioctrl_cmd cmd, void *arg);
	int (*proc_frame)(void *handle, void *param);
	int (*set_callback)(void *handle,
			dcam_dev_callback cb, void *priv_data);
	int (*update_clk)(void *handle, void *arg);
};

struct dcam_pipe_ops *get_dcam_ops(void);

void *get_dcam_pipe_dev(int idx);
int put_dcam_pipe_dev(void *dcam_handle);

int sprd_dcam_debugfs_init(void);

int sprd_dcam_parse_dt(struct device_node *dn,
		struct sprd_cam_hw_info **dcam_hw,
		uint32_t *dcam_count);

#endif  /* _DCAM_INTERFACE_H_ */
