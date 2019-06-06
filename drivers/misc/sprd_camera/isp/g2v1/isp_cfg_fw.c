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

#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <video/sprd_isp_altek.h>
#include "altek_isp/altek_ap3a_info.h"
#include "altek_isp/altek_isp_drv.h"
#include "altek_isp/altek_iq_info.h"

#include "isp_drv.h"

typedef int32_t(*isp_cfg_fun_ptr) (struct isp_io_param *isp_param,
				struct isp_pipe_dev *isp_pipeline);

struct isp_cfg_fw {
	uint32_t sub_block;
	isp_cfg_fun_ptr cfg_fun;
};

static int32_t isp_k_cfg_iso_speed(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	uint32_t iso_value = 0;

	if (!param) {
		pr_err("isp_k_cfg_iso_speed: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_iso_speed: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&iso_value, param->property_param,
							sizeof(iso_value));

	if (ret != 0) {
		pr_err("isp_k_cfg_iso_mode: copy error, ret=0x%x\n",
				(int32_t) ret);

		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_iso_speed(sns_id, (uint16_t)iso_value);
	if (ret)
		pr_err("ISPDRV_SetISOMode:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_awb_gain(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct isp_awb_gain_info awb_gain_info;
	struct awb_gain_info awb_gain;

	if (!param) {
		pr_err("isp_k_cfg_awb_gain: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_awb_gain: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&awb_gain_info, param->property_param,
			     sizeof(awb_gain_info));
	if (ret != 0) {
		pr_err("isp_k_cfg_awb_gain: ret=0x%x\n", (int32_t) ret);
		return -1;
	}

	awb_gain.detected_awb_r = awb_gain_info.r;
	awb_gain.detected_awb_gr = awb_gain_info.g;
	awb_gain.detected_awb_gb = awb_gain_info.g;
	awb_gain.detected_awb_b = awb_gain_info.b;
	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_awb_gain(sns_id, &awb_gain);
	if (ret)
		pr_err("ispdrv_set_awb_gain:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_awb_gain_balanced(struct isp_io_param *param,
	struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct isp_awb_gain_info awb_gain_info;
	struct awb_gain_info awb_gain;

	if (!param) {
		pr_err("isp_k_cfg_awb_gain: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_awb_gain: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&awb_gain_info,
		param->property_param, sizeof(awb_gain_info));
	if (ret != 0) {
		pr_err("isp_k_cfg_awb_gain: copy error, ret=0x%x\n",
			(int32_t) ret);
		return -1;
	}

	awb_gain.detected_awb_r = awb_gain_info.r;
	awb_gain.detected_awb_gr = awb_gain_info.g;
	awb_gain.detected_awb_gb = awb_gain_info.g;
	awb_gain.detected_awb_b = awb_gain_info.b;
	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_balanced_awb_gain(sns_id, &awb_gain);
	if (ret)
		pr_err("ispdrv_set_awb_gain:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_dld_sequence(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct dld_sequence dld_seq;

	if (!param) {
		pr_err("isp_k_cfg_dld_sequence: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_dld_sequence: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&dld_seq, param->property_param,
			     sizeof(dld_seq));
	if (ret != 0) {
		pr_err("isp_k_cfg_dld_sequence:ret=0x%x\n", (int32_t) ret);
		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_dld_sequence(sns_id, &dld_seq);
	if (ret)
		pr_err("ispdrv_set_dld_sequence:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_3a_param(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct cfg_3a_info cfg_3a_info;

	if (!param) {
		pr_err("isp_k_cfg_3a_param: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_3a_param: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&cfg_3a_info, param->property_param,
							sizeof(cfg_3a_info));

	if (ret != 0) {
		pr_err("isp_k_cfg_3a_param: copy error, ret=0x%x\n",
				(int32_t) ret);

		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_3a_cfg(sns_id, &cfg_3a_info);
	if (ret)
		pr_err("ispdrv_set_3a_cfg:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_ae_param(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct ae_cfg_info cfg_ae_info;

	if (!param) {
		pr_err("isp_k_cfg_ae_param: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_ae_param: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&cfg_ae_info, param->property_param,
							sizeof(cfg_ae_info));

	if (ret != 0) {
		pr_err("isp_k_cfg_ae_param: copy error, ret=0x%x\n",
				(int32_t) ret);

		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_ae_cfg(sns_id, &cfg_ae_info);
	if (ret)
		pr_err("ispdrv_set_ae_cfg:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_af_param(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct af_cfg_info cfg_af_info;

	if (!param) {
		pr_err("isp_k_cfg_af_param: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_af_param: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&cfg_af_info, param->property_param,
							sizeof(cfg_af_info));

	if (ret != 0) {
		pr_err("isp_k_cfg_af_param: copy error, ret=0x%x\n",
				(int32_t) ret);
		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_af_cfg(sns_id, &cfg_af_info);
	if (ret)
		pr_err("ispdrv_set_af_cfg:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_awb_param(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct awb_cfg_info cfg_awb_info;

	if (!param) {
		pr_err("isp_k_cfg_awb_param: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_awb_param: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&cfg_awb_info, param->property_param,
							sizeof(cfg_awb_info));
	if (ret != 0) {
		pr_err("isp_k_cfg_awb_param: copy error, ret=0x%x\n",
				(int32_t) ret);

		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_awb_cfg(sns_id, &cfg_awb_info);
	if (ret)
		pr_err("ispdrv_set_awb_cfg:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_yhis_param(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct yhis_cfg_info cfg_yhis_info;

	if (!param) {
		pr_err("isp_k_cfg_yhis_param: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_yhis_param: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&cfg_yhis_info, param->property_param,
							sizeof(cfg_yhis_info));

	if (ret != 0) {
		pr_err("isp_k_cfg_yhis_param: copy error, ret=0x%x\n",
				(int32_t) ret);

		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_yhis_cfg(sns_id, &cfg_yhis_info);
	if (ret)
		pr_err("ispdrv_set_yhis_cfg:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_sub_sample(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct subsample_cfg_info sub_sample_info;

	if (!param) {
		pr_err("isp_k_cfg_sub_sample: param is null error\n");
		ret = -EINVAL;
		goto exit;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_sub_sample: property_param is null error\n");
		ret = -EINVAL;
		goto exit;
	}

	ret = copy_from_user((void *)&sub_sample_info, param->property_param,
						sizeof(sub_sample_info));

	if (ret != 0) {
		pr_err("isp_k_cfg_sub_sample: copy error, ret=0x%x\n",
				(int32_t) ret);

		ret = -EFAULT;
		goto exit;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t)ispdrv_set_subsample_cfg(sns_id, &sub_sample_info);
	if (ret)
		pr_err("ispdrv_set_subsample_cfg:set param error\n");

exit:

	return ret;
}

static int32_t isp_k_cfg_anti_flicker(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct antiflicker_cfg_info afl_info;

	if (!param) {
		pr_err("isp_k_cfg_anti_flicker: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_anti_flicker: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&afl_info, param->property_param,
							sizeof(afl_info));

	if (ret != 0) {
		pr_err("isp_k_cfg_anti_flicker: copy error, ret=0x%x\n",
				(int32_t) ret);

		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_antiflicker_cfg(sns_id, &afl_info);
	if (ret)
		pr_err("ispdrv_set_antiflicker_cfg:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_dld_seq_basic_prev(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	uint32_t size;
	uint8_t *data_ptr;

	if (!param) {
		pr_err("isp_k_cfg_dld_seq_basic_prev: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_dld_seq_basic_prev: property_param is null error\n");
		return -1;
	}

	size = param->reserved;
	data_ptr = vmalloc(size);
	if (!data_ptr) {
		pr_err("isp_k_cfg_dld_seq_basic_prev: vmalloc error\n");
		return -1;
	}
	memset(data_ptr, 0, size);
	ret = copy_from_user((void *)&data_ptr, param->property_param, size);
	if (ret != 0) {
		pr_err("dld_seq_basic_prev:ret=0x%x\n", (int32_t) ret);
		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t)ispdrv_set_dld_sequence_basic_preview(sns_id, size,
							     data_ptr);
	if (ret)
		pr_err("isp_k_cfg_dld_seq_basic_prev:set param error\n");

	vfree(data_ptr);

	return ret;
}

static int32_t isp_k_cfg_dld_seq_adv_prev(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	uint32_t size;
	uint8_t *data_ptr;

	if (!param) {
		pr_err("isp_k_cfg_dld_seq_adv_prev: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_dld_seq_adv_prev: property_param is null error\n");
		return -1;
	}

	size = param->reserved;
	data_ptr = vmalloc(size);
	if (!data_ptr) {
		pr_err("isp_k_cfg_dld_seq_adv_prev: vmalloc error\n");
		return -1;
	}
	memset(data_ptr, 0, size);
	ret = copy_from_user((void *)&data_ptr, param->property_param, size);
	if (ret != 0) {
		pr_err("isp_k_cfg_dld_seq_adv_prev: copy error, ret=0x%x\n",
					(int32_t) ret);

		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t)ispdrv_set_dld_sequence_adv_preview(sns_id, size,
							   data_ptr);
	if (ret)
		pr_err("isp_k_cfg_dld_seq_adv_prev:set param error\n");

	vfree(data_ptr);

	return ret;
}

static int32_t isp_k_cfg_dld_seq_fast_converge(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	uint32_t size;
	uint8_t *data_ptr;

	if (!param) {
		pr_err("isp_k_cfg_dld_seq_fast_converge: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_dld_seq_fast_converge: property_param is null error\n");
		return -1;
	}

	size = param->reserved;
	data_ptr = vmalloc(size);
	if (!data_ptr) {
		pr_err("isp_k_cfg_dld_seq_fast_converge: vmalloc error\n");
		return -1;
	}
	memset(data_ptr, 0, size);
	ret = copy_from_user((void *)&data_ptr, param->property_param, size);
	if (ret != 0) {
		pr_err("dld_seq_fast_converge ret=0x%x\n", (int32_t) ret);
		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t)ispdrv_set_dld_sequence_basic_fast_converge(sns_id, size,
								   data_ptr);
	if (ret)
		pr_err("isp_k_cfg_dld_seq_fast_converge:set param error\n");

	vfree(data_ptr);

	return ret;
}

static int32_t isp_k_cfg_scenario_info(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct scenario_info_ap scenario_info;
	struct s_scenario_info_ap scen_info;
	uint32_t isp_id = 0;

	if (!param) {
		pr_err("isp_k_cfg_scenario_info: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_scenario_info: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&scenario_info, param->property_param,
			sizeof(scenario_info));
	if (ret != 0) {
		pr_err("isp_k_cfg_scenario_info: copy error, ret=0x%x\n",
			(int32_t) ret);
		return -1;
	}

	sns_id = (uint8_t) isp_scenario_id_map(isp_pipeline->scenario_id);
	isp_id = isp_pipeline->isp_id;

	memcpy((uint8_t *)&scen_info, (uint8_t *)&scenario_info,
	       sizeof(scenario_info));
	if (scen_info.out_bypass_flag.bypass_lv)
		scen_info.iq_param_idx_info.iq_param_idx_lv = 0xff;
	if (scen_info.out_bypass_flag.bypass_video)
		scen_info.iq_param_idx_info.iq_param_idx_video = 0xff;
	if (scen_info.out_bypass_flag.bypass_still)
		scen_info.iq_param_idx_info.iq_param_idx_still = 0xff;
	if ((isp_pipeline->isp_id == ISP_DEV0) &&
	    ((sns_id == V4L2_ISP_SCENARIO_PREVIEW_SENSOR_1) ||
	     (sns_id == V4L2_ISP_SCENARIO_PREVIEW_STRIPE) ||
	     (sns_id == V4L2_ISP_SCENARIO_PREVIEW_STILL_SS)))
		ret = (uint32_t) ispdrv_set_scenario_info(sns_id,
							&scen_info, NULL, NULL);
	if ((isp_pipeline->isp_id == ISP_DEV1) &&
	     (sns_id == V4L2_ISP_SCENARIO_PREVIEW_SENSOR_2))
		ret = (uint32_t) ispdrv_set_scenario_info(sns_id,
							NULL, &scen_info, NULL);
	if ((isp_pipeline->isp_id == ISP_DEV2) &&
	     (sns_id == V4L2_ISP_SCENARIO_PREVIEW_SENSOR_LITE))
		ret = (uint32_t) ispdrv_set_scenario_info(sns_id,
							NULL, NULL, &scen_info);
	if (ret)
		pr_err("ispdrv_set_scenario_info:set param error\n");


	return ret;
}

static int32_t isp_k_cfg_sharpness(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	uint32_t sharpness_val = 0;

	if (!param) {
		pr_err("isp_k_cfg_sharpness: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_sharpness: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&sharpness_val, param->property_param,
							sizeof(sharpness_val));

	if (ret != 0) {
		pr_err("isp_k_cfg_sharpness: copy error, ret=0x%x\n",
				(int32_t) ret);

		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_sharpness_mode(sns_id,
						  (uint8_t)sharpness_val);
	if (ret)
		pr_err("ispdrv_set_sharpness_mode:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_saturation(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	uint32_t saturation_val = 0;

	if (!param) {
		pr_err("isp_k_cfg_saturation: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_saturation: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&saturation_val, param->property_param,
							sizeof(saturation_val));
	if (ret != 0) {
		pr_err("isp_k_cfg_saturation: copy error, ret=0x%x\n",
				(int32_t) ret);
		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_saturation_mode(sns_id,
						   (uint8_t)saturation_val);
	if (ret)
		pr_err("ispdrv_set_saturation_mode:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_contrast(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	uint32_t contrast_val = 0;

	if (!param) {
		pr_err("isp_k_cfg_contrast: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_contrast: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&contrast_val, param->property_param,
							sizeof(contrast_val));

	if (ret != 0) {
		pr_err("isp_k_cfg_contrast: copy error, ret=0x%x\n",
				(int32_t) ret);

		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_contrast_mode(sns_id, (uint8_t)contrast_val);
	if (ret)
		pr_err("ispdrv_set_contrast_mode:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_special_effect(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	uint32_t effect_val = 0;

	if (!param) {
		pr_err("isp_k_cfg_special_effect: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_special_effect: property_param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&effect_val, param->property_param,
							sizeof(effect_val));

	if (ret != 0) {
		pr_err("isp_k_cfg_special_effect: copy error, ret=0x%x\n",
				(int32_t) ret);

		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_special_effect_mode(sns_id,
						       (uint8_t)effect_val);

	if (ret)
		pr_err("ispdrv_set_special_effect_mode:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_brightness_gain(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct isp_brightness_gain brightness_gain;
	struct ispdrv_brightness_gain_setting br_gain;

	if (!param) {
		pr_err("isp_k_cfg_brightness_gain: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_brightness_gain: param is null error\n");
		return -1;
	}

	memset((void *)&brightness_gain, 0, sizeof(brightness_gain));
	ret = copy_from_user((void *)&brightness_gain,
			param->property_param, sizeof(brightness_gain));
	if (ret != 0) {
		pr_err("isp_k_cfg_brightness_gain: copy error, ret=0x%x\n",
			(int32_t) ret);
		return -1;
	}

	memcpy(&br_gain, &brightness_gain, sizeof(br_gain));
	sns_id = (uint8_t) isp_pipeline->isp_id;
	ret = (int32_t) ispdrv_set_brightness_gain_setting(sns_id, &br_gain);
	if (ret)
		pr_err("isp_k_cfg_brightness_gain:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_brightness_mode(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	uint32_t br_mode = 0;
	uint8_t brightness = 0;

	if (!param) {
		pr_err("isp_k_cfg_brightness_mode: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_brightness_mode: param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&br_mode, param->property_param,
			sizeof(br_mode));
	if (ret != 0) {
		pr_err("isp_k_cfg_brightness_mode: copy error, ret=0x%x\n",
			(int32_t) ret);
		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;
	brightness = (uint8_t) br_mode;

	ret = (int32_t) ispdrv_set_brightness_mode(sns_id, brightness);
	if (ret)
		pr_err("isp_k_cfg_brightness_mode:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_color_temp(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	uint32_t color_t = 0;

	if (!param) {
		pr_err("isp_k_cfg_color_temperature: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_color_temperature: param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&color_t, param->property_param,
			sizeof(color_t));
	if (ret != 0) {
		pr_err("isp_k_cfg_color_temperature: copy error, ret=0x%x\n",
			(int32_t) ret);
		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_color_temperature(sns_id, color_t);
	if (ret)
		pr_err("isp_k_cfg_color_temperature:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_ccm(struct isp_io_param *param,
				struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct isp_iq_ccm_info ccm_info_in;
	struct iq_ccm_info ccm_info;

	if (!param) {
		pr_err("isp_k_cfg_ccm: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_ccm: param is null error\n");
		return -1;
	}

	memset((void *)&ccm_info_in, 0, sizeof(ccm_info_in));
	ret = copy_from_user((void *)&ccm_info_in,
			param->property_param, sizeof(ccm_info_in));
	if (ret != 0) {
		pr_err("isp_k_cfg_ccm: copy error, ret=0x%x\n",
			(int32_t) ret);
		return -1;
	}

	memcpy(&ccm_info, &ccm_info_in, sizeof(ccm_info_in));
	sns_id = (uint8_t) isp_pipeline->isp_id;
	ret = (int32_t) ispdrv_set_ccm(sns_id, &ccm_info);
	if (ret)
		pr_err("isp_k_cfg_ccm:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_valid_adgain(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	uint32_t ad_gain = 0;

	if (!param) {
		pr_err("isp_k_cfg_valid_adgain: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_valid_adgain: param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&ad_gain, param->property_param,
			sizeof(ad_gain));
	if (ret != 0) {
		pr_err("isp_k_cfg_valid_adgain: copy error, ret=0x%x\n",
			(int32_t) ret);
		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_valid_ad_gain(sns_id, ad_gain);
	if (ret)
		pr_err("isp_k_cfg_valid_adgain:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_valid_exp_time(struct isp_io_param *param,
					struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	uint32_t exp_time = 0;

	if (!param) {
		pr_err("isp_k_cfg_valid_exp_time: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_valid_exp_time: param is null error\n");
		return -1;
	}

	ret = copy_from_user((void *)&exp_time, param->property_param,
			sizeof(exp_time));
	if (ret != 0) {
		pr_err("isp_k_cfg_valid_exp_time: copy error, ret=0x%x\n",
			(int32_t) ret);
		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	ret = (int32_t) ispdrv_set_valid_exp_time(sns_id, exp_time);
	if (ret)
		pr_err("isp_k_cfg_valid_exp_time:set param error\n");

	return ret;
}

static int32_t isp_k_cfg_iq_otp_info(struct isp_io_param *param,
				struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	struct isp_iq_otp_info *otp_info_in = NULL;
	struct iq_otp_info *otp_info = NULL;

	if (!param) {
		pr_err("isp_k_cfg_iq_otp_info: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_iq_otp_info: param is null error\n");
		return -1;
	}

	otp_info_in = kmalloc(sizeof(struct isp_iq_otp_info), GFP_KERNEL);
	if (!otp_info_in)
		return -1;

	otp_info = kmalloc(sizeof(struct iq_otp_info), GFP_KERNEL);
	if (!otp_info) {
		kfree(otp_info_in);
		return -1;
	}

	memset((void *)otp_info_in, 0, sizeof(struct isp_iq_otp_info));
	ret = copy_from_user((void *)otp_info_in,
			     param->property_param,
			     sizeof(struct isp_iq_otp_info));
	if (ret != 0) {
		pr_err("isp_k_cfg_iq_otp_info: copy error, ret=0x%x\n",
		       (int32_t) ret);
		goto exit;
	}

	memcpy(otp_info, otp_info_in, sizeof(struct iq_otp_info));
	sns_id = (uint8_t) isp_pipeline->isp_id;
	ret = ispdrv_set_iq_otp_info(sns_id, otp_info);
	if (ret)
		pr_err("isp_k_cfg_iq_otp_info:set param error\n");

exit:
	kfree(otp_info_in);
	kfree(otp_info);
	return ret;
}

static int32_t isp_k_cfg_sof_param(struct isp_io_param *param,
				struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint8_t sns_id = 0;
	uint32_t color_t = 0;
	struct isp_sof_cfg_info sof_info;
	struct awb_gain_info awb_gain;

	if (!param) {
		pr_err("isp_k_cfg_sof_param: param is null error\n");
		return -1;
	}
	if (!param->property_param) {
		pr_err("isp_k_cfg_sof_param: param is null error\n");
		return -1;
	}

	memset((void *)&sof_info, 0, sizeof(sof_info));
	ret = copy_from_user((void *)&sof_info,
			     param->property_param, sizeof(sof_info));
	if (ret != 0) {
		pr_err("isp_k_cfg_sof_param: copy error, ret=0x%x\n",
		       (int32_t) ret);
		return -1;
	}

	sns_id = (uint8_t) isp_pipeline->isp_id;

	if (sof_info.is_update) {
		awb_gain.detected_awb_r = sof_info.awb_gain.r;
		awb_gain.detected_awb_gr = sof_info.awb_gain.g;
		awb_gain.detected_awb_gb = sof_info.awb_gain.g;
		awb_gain.detected_awb_b = sof_info.awb_gain.b;
		ret = (int32_t) ispdrv_set_awb_gain(sns_id, &awb_gain);
		if (ret)
			pr_err("ISPDrv_SetAWBGain:set param %d\n", ret);

		awb_gain.detected_awb_r = sof_info.awb_b_gain.r;
		awb_gain.detected_awb_gr = sof_info.awb_b_gain.g;
		awb_gain.detected_awb_gb = sof_info.awb_b_gain.g;
		awb_gain.detected_awb_b = sof_info.awb_b_gain.b;

		ret = (int32_t) ispdrv_set_balanced_awb_gain(sns_id, &awb_gain);
		if (ret)
			pr_err("ISPDrv_SetBalancedAWBGain:set param %d\n", ret);

		color_t = sof_info.color_temp;
		ret = (int32_t) ispdrv_set_color_temperature(sns_id, color_t);
		if (ret)
			pr_err("set_color_temperature:set param %d\n", ret);
	}

	ret = (int32_t) ispdrv_set_iso_speed(sns_id,
					     (uint16_t)sof_info.iso_val);
	if (ret)
		pr_err("ispdrv_set_iso_speed:set param error\n");

	ret = (int32_t) ispdrv_set_iq_info(sns_id,
					ISP_FUNC_ID_AE_INFO,
					(void *)&sof_info.iq_ae_info,
					sizeof(struct isp_func_ae_info));
	if (ret)
		pr_err("ispdrv_set_iq_info:set param error\n");

	return ret;
}

static struct isp_cfg_fw isp_cfg_fun_tab[] = {
	{ISP_CFG_SET_ISO_SPEED,               isp_k_cfg_iso_speed},
	{ISP_CFG_SET_AWB_GAIN,                isp_k_cfg_awb_gain},
	{ISP_CFG_SET_DLD_SEQUENCE,            isp_k_cfg_dld_sequence},
	{ISP_CFG_SET_3A_CFG,                  isp_k_cfg_3a_param},
	{ISP_CFG_SET_AE_CFG,                  isp_k_cfg_ae_param},
	{ISP_CFG_SET_AF_CFG,                  isp_k_cfg_af_param},
	{ISP_CFG_SET_AWB_CFG,                 isp_k_cfg_awb_param},
	{ISP_CFG_SET_YHIS_CFG,                isp_k_cfg_yhis_param},
	{ISP_CFG_SET_SUB_SAMP_CFG,            isp_k_cfg_sub_sample},
	{ISP_CFG_SET_AFL_CFG,                 isp_k_cfg_anti_flicker},
	{ISP_CFG_SET_DLD_SEQ_BASIC_PREV,      isp_k_cfg_dld_seq_basic_prev},
	{ISP_CFG_SET_DLD_SEQ_ADV_PREV,        isp_k_cfg_dld_seq_adv_prev},
	{ISP_CFG_SET_DLD_SEQ_BASIC_FAST_CONV, isp_k_cfg_dld_seq_fast_converge},
	{ISP_CFG_SET_SCENARIO_INFO,           isp_k_cfg_scenario_info},
	{ISP_CFG_SET_SHARPNESS,               isp_k_cfg_sharpness},
	{ISP_CFG_SET_SATURATION,              isp_k_cfg_saturation},
	{ISP_CFG_SET_CONTRAST,                isp_k_cfg_contrast},
	{ISP_CFG_SET_SPECIAL_EFFECT,          isp_k_cfg_special_effect},
	{ISP_CFG_SET_AWB_GAIN_BALANCED,       isp_k_cfg_awb_gain_balanced},
	{ISP_CFG_SET_BRIGHTNESS_GAIN,         isp_k_cfg_brightness_gain},
	{ISP_CFG_SET_BRIGHTNESS_MODE,         isp_k_cfg_brightness_mode},
	{ISP_CFG_SET_COLOR_TEMPERATURE,       isp_k_cfg_color_temp},
	{ISP_CFG_SET_CCM,                     isp_k_cfg_ccm},
	{ISP_CFG_SET_VALID_ADGAIN,            isp_k_cfg_valid_adgain},
	{ISP_CFG_SET_VALID_EXP_TIME,          isp_k_cfg_valid_exp_time},
	{ISP_CFG_SET_IQ_OTP_INFO,             isp_k_cfg_iq_otp_info},
	{ISP_CFG_SET_SOF_PARAM,               isp_k_cfg_sof_param}
};

int32_t isp_cfg_fw_param(void *param, struct isp_pipe_dev *isp_pipeline)
{
	int32_t ret = 0;
	uint32_t i = 0, cnt = 0;
	isp_cfg_fun_ptr cfg_fun_ptr = NULL;
	struct isp_io_param isp_param = { 0, NULL, 0};

	if (!param) {
		pr_err("isp_cfg_fw_param: param is null error.\n");
		return -1;
	}

	ret = copy_from_user((void *)&isp_param, (void *)param,
		sizeof(isp_param));
	if (ret) {
		pr_err("isp_cfg_param: copy error, ret = 0x%x\n",
			(uint32_t) ret);
		return -1;
	}

	cnt = ARRAY_SIZE(isp_cfg_fun_tab);
	for (i = 0; i < cnt; i++) {
		if (isp_param.sub_id == isp_cfg_fun_tab[i].sub_block) {
			cfg_fun_ptr = isp_cfg_fun_tab[i].cfg_fun;
			break;
		}
	}

	if (cfg_fun_ptr != NULL) {
		/* pr_err("isp_cfg_fw_param: sub_id %d\n", isp_param.sub_id); */
		ret = cfg_fun_ptr(&isp_param, isp_pipeline);
	}

	return ret;
}

