/*
* File: altek_ap3a_info.c                                 *
* Description: Implementaion of ap3a api                  *
*                                                         *
* (C)Copyright altek Corporation 2016                     *
*                                                         *
* History:                                                *
*   2016/03/012; Caedmon Lai; Initial version             *
*/

#include <linux/semaphore.h>

#include "altek_ahb_drv.h"
#include "altek_sensor_info.h"
#include "altek_common.h"
#include "altek_ap3a_info_local.h"
#include "altek_ap3a_info.h"
#include "altek_dump_utility.h"
#include "altek_log_local.h"

static struct dld_sequence *g_kva_dld_seq_info_ptr[SENSOR_TOTAL];

static struct cfg_3a_info *g_kva_cfg_3a_info_ptr[SENSOR_TOTAL];
/************************************************************
*	  ap-side 3A configuration apI			    *
*************************************************************/
/*
*\brief Set fast converge mode
*\param a_uc_sensor [in], Sensor ID
*\param a_uc_mode   [in], mode, refer to ispdrv_fast_converge_mode
*\return error code
*/
u32 ispdrv_set_fast_converge_mode(u8 a_uc_sensor, u8 a_uc_mode)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto fast_converge_end;
	}

	if (a_uc_mode >= V4L2_ISP_FASTCON_TOTAL) {
		err = EINVAL;
		goto fast_converge_semapost;
	}

	if (a_uc_sensor == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg =
		    AHB_ITEM_CHANGE_SENSOR1_FAST_CONVERGE_SETTING;
		isp_set_field_value(AHB_ITEM_SENSOR1_FAST_CONVERGE_SETTING,
				    a_uc_mode);
	} else if (a_uc_sensor == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg =
		    AHB_ITEM_CHANGE_SENSOR2_FAST_CONVERGE_SETTING;
		isp_set_field_value(AHB_ITEM_SENSOR2_FAST_CONVERGE_SETTING,
				    a_uc_mode);

	} else if (a_uc_sensor == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg =
		    AHB_ITEM_CHANGE_SENSOR3_FAST_CONVERGE_SETTING;
		isp_set_field_value(AHB_ITEM_SENSOR3_FAST_CONVERGE_SETTING,
				    a_uc_mode);
	} else {
		err = EINVAL;
		goto fast_converge_semapost;
	}

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* opstart*/
	err = command_start(__func__);
	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

fast_converge_semapost:
	SEMAPOST(get_command_semaphore());
fast_converge_end:

	isp_debug_err("%d. a_ucMode: %d", err, a_uc_mode);
	isp_debug_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_set_fast_converge_mode);


/*
*\brief Set download sequence
*\param sensor_id [in], Sensor ID
*\param a_ptDldSequence   [in], download sequence information
*\return error code
*/
u32 ispdrv_set_dld_sequence(u8 sensor_id,
	struct dld_sequence *dld_sequence)
{
	u32 err = 0;

	err = ispdrv_set_dld_sequence_basic_preview(sensor_id,
			dld_sequence->preview_baisc_dld_seq_length,
			dld_sequence->preview_baisc_dld_seq);
	if (err != 0)
		goto set_dld_sequence_end;

	err = ispdrv_set_dld_sequence_adv_preview(sensor_id,
			dld_sequence->preview_adv_dld_seq_length,
			dld_sequence->preview_adv_dld_seq);

	if (err != 0)
		goto set_dld_sequence_end;

	err = ispdrv_set_dld_sequence_basic_fast_converge(sensor_id,
			dld_sequence->fast_converge_baisc_dld_seq_length,
			dld_sequence->fast_converge_baisc_dld_seq);
	if (err != 0)
		goto set_dld_sequence_end;

set_dld_sequence_end:

	return err;
}
EXPORT_SYMBOL(ispdrv_set_dld_sequence);

/*
*\brief Set 3A configuration
*\param sensor_id [in], Sensor ID
*\param 3a_info	[in], 3A configuration
*\return error code
*/
u32 ispdrv_set_3a_cfg(u8 sensor_id, struct cfg_3a_info *a3_info)
{
	u32 err = 0;

	err = ispdrv_set_ae_cfg(sensor_id, &a3_info->ae_info);
	if (err != 0)
		goto set_3a_cfg_end;
	err = ispdrv_set_af_cfg(sensor_id, &a3_info->af_info);
	if (err != 0)
		goto set_3a_cfg_end;

	err = ispdrv_set_awb_cfg(sensor_id, &a3_info->awb_info);
	if (err != 0)
		goto set_3a_cfg_end;

	err = ispdrv_set_yhis_cfg(sensor_id, &a3_info->yhis_info);
	if (err != 0)
		goto set_3a_cfg_end;

	err = ispdrv_set_subsample_cfg(sensor_id,
				&a3_info->subsample_info);
	if (err != 0)
		goto set_3a_cfg_end;

	err = ispdrv_set_antiflicker_cfg(sensor_id,
				&a3_info->antiflicker_info);
	if (err != 0)
		goto set_3a_cfg_end;

set_3a_cfg_end:

	return err;

}
EXPORT_SYMBOL(ispdrv_set_3a_cfg);

/*
*\brief Set AE configuration
*\param sensor_id [in], Sensor ID
*\param ae_info   [in], AE configuration
*\return error code
*/
u32 ispdrv_set_ae_cfg(u8 sensor_id, struct ae_cfg_info *ae_info)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	struct cfg_3a_info *cfg_3a_info_to_be_set = NULL;

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_ae_cfg_end;
	}

	cfg_3a_info_to_be_set = get_shm_cfg_3a_info_ptr(sensor_id);

	MEMCPY((u8 *) &cfg_3a_info_to_be_set->ae_info, (u8 *) ae_info,
	       sizeof(struct ae_cfg_info));

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_CONFIG_AE;
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_CONFIG_AE;
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_CONFIG_AE;
	} else {
		err = EINVAL;
		goto set_ae_cfg_semapost;
	}

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* opstart*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_ae_cfg_semapost:
	SEMAPOST(get_command_semaphore());
set_ae_cfg_end:

	return err;

}
EXPORT_SYMBOL(ispdrv_set_ae_cfg);

/*
*\brief Set AF configuration
*\param sensor_id [in], Sensor ID
*\param af_info	[in], AF configuration
*\return error code
*/
u32 ispdrv_set_af_cfg(u8 sensor_id, struct af_cfg_info *af_info)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	struct cfg_3a_info *cfg_3a_info_to_be_set = NULL;

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_af_cfg_end;
	}

	cfg_3a_info_to_be_set = get_shm_cfg_3a_info_ptr(sensor_id);

	MEMCPY((u8 *) &cfg_3a_info_to_be_set->af_info, (u8 *) af_info,
	sizeof(struct af_cfg_info));

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_CONFIG_AF;
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_CONFIG_AF;
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_CONFIG_AF;
	} else {
		err = EINVAL;
		goto set_af_cfg_semapost;
	}

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* opstart*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_af_cfg_semapost:
	SEMAPOST(get_command_semaphore());
set_af_cfg_end:

	return err;

}
EXPORT_SYMBOL(ispdrv_set_af_cfg);

/*
*\brief Set AWB configuration
*\param sensor_id [in], Sensor ID
*\param awb_info	 [in], AWB configuration
*\return error code
*/
u32 ispdrv_set_awb_cfg(u8 sensor_id, struct awb_cfg_info *awb_info)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	struct cfg_3a_info *cfg_3a_info_to_be_set = NULL;

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_awb_cfg_end;
	}

	cfg_3a_info_to_be_set = get_shm_cfg_3a_info_ptr(sensor_id);

	MEMCPY((u8 *) &(cfg_3a_info_to_be_set->awb_info),
	       (u8 *) awb_info, sizeof(struct awb_cfg_info));

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_CONFIG_AWB;
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_CONFIG_AWB;
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_CONFIG_AWB;
	} else {
		err = EINVAL;
		goto set_awb_cfg_semapost;
	}

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* opstart*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_awb_cfg_semapost:
	SEMAPOST(get_command_semaphore());
set_awb_cfg_end:

	return err;

}
EXPORT_SYMBOL(ispdrv_set_awb_cfg);

/*
*\brief Set YHis configuration
*\param sensor_id [in], Sensor ID
*\param yhis_info   [in], YHis configuration
*\return error code
*/
u32 ispdrv_set_yhis_cfg(u8 sensor_id, struct yhis_cfg_info *yhis_info)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	struct cfg_3a_info *cfg_3a_info_to_be_set = NULL;

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_yhis_cfg_end;
	}

	cfg_3a_info_to_be_set = get_shm_cfg_3a_info_ptr(sensor_id);

	MEMCPY((u8 *) &cfg_3a_info_to_be_set->yhis_info,
	       (u8 *) yhis_info, sizeof(struct yhis_cfg_info));

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_CONFIG_YHIS;
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_CONFIG_YHIS;
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_CONFIG_YHIS;
	} else {
		err = EINVAL;
		goto set_yhis_cfg_semapost;
	}

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* opstart*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_yhis_cfg_semapost:
	SEMAPOST(get_command_semaphore());
set_yhis_cfg_end:

	return err;

}
EXPORT_SYMBOL(ispdrv_set_yhis_cfg);

/*
*\brief Set SubSample configuration
*\param sensor_id [in], Sensor ID
*\param subsample_info   [in], SubSample configuration
*\return error code
*/
u32 ispdrv_set_subsample_cfg(u8 sensor_id,
			     struct subsample_cfg_info *subsample_info)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	struct cfg_3a_info *cfg_3a_info_to_be_set = NULL;

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_subsample_cfg_end;
	}

	cfg_3a_info_to_be_set = get_shm_cfg_3a_info_ptr(sensor_id);

	MEMCPY((u8 *) &cfg_3a_info_to_be_set->subsample_info,
	       (u8 *) subsample_info, sizeof(struct subsample_cfg_info));

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_CONFIG_SUBSAMPLE;
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_CONFIG_SUBSAMPLE;
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_CONFIG_SUBSAMPLE;
	} else {
		err = EINVAL;
		goto set_subsample_cfg_semapost;
	}

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* opstart*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_subsample_cfg_semapost:
	SEMAPOST(get_command_semaphore());
set_subsample_cfg_end:

	return err;

}
EXPORT_SYMBOL(ispdrv_set_subsample_cfg);

/*
*\brief Set antiflicker configuration
*\param sensor_id [in], Sensor ID
*\param antiflicker_info	 [in], AntiFlicker configuration
*\return error code
*/
u32 ispdrv_set_antiflicker_cfg(u8 sensor_id,
			       struct antiflicker_cfg_info *
			       antiflicker_info)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	struct cfg_3a_info *cfg_3a_info_to_be_set = NULL;

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_antiflicker_cfg_end;
	}

	cfg_3a_info_to_be_set = get_shm_cfg_3a_info_ptr(sensor_id);

	MEMCPY((u8 *) &cfg_3a_info_to_be_set->antiflicker_info,
	       (u8 *) antiflicker_info,
	       sizeof(struct antiflicker_cfg_info));

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_CONFIG_ANTIFLICKER;
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_CONFIG_ANTIFLICKER;
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_CONFIG_ANTIFLICKER;
	} else {
		err = EINVAL;
		goto set_antiflicker_cfg_semapost;
	}

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* opstart*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_antiflicker_cfg_semapost:
	SEMAPOST(get_command_semaphore());
set_antiflicker_cfg_end:

	return err;
}
EXPORT_SYMBOL(ispdrv_set_antiflicker_cfg);

/*
*\brief Set download sequence of basic preview
*\param sensor_id [in], Sensor ID
*\param preview_baisc_dld_seq_length[in], length of the sequence
*\param preview_baisc_dld_seq [in], the sequence
*\return error code
*/
u32 ispdrv_set_dld_sequence_basic_preview(u8 sensor_id,
					  u8 preview_baisc_dld_seq_length,
					  u8 *preview_baisc_dld_seq)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;
	struct dld_sequence *t_dld_sequence_to_be_set = NULL;
	u8 i = 0;

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_dld_sequence_basic_preview_end;
	}

	t_dld_sequence_to_be_set = get_shm_dld_seq_info_ptr(sensor_id);

	t_dld_sequence_to_be_set->preview_baisc_dld_seq_length =
	    preview_baisc_dld_seq_length;

	for (i = 0; i < preview_baisc_dld_seq_length; i++)
		t_dld_sequence_to_be_set->preview_baisc_dld_seq[i] =
		    preview_baisc_dld_seq[i];

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg =
		    AHB_ITEM_CHANGE_SENSOR1_BASICDLDSEQ_PREVIEW;
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg =
		    AHB_ITEM_CHANGE_SENSOR2_BASICDLDSEQ_PREVIEW;
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg =
		    AHB_ITEM_CHANGE_SENSOR3_BASICDLDSEQ_PREVIEW;
	} else {
		err = EINVAL;
		goto set_dld_sequence_basic_preview_semapost;
	}

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* opstart*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_dld_sequence_basic_preview_semapost:
	SEMAPOST(get_command_semaphore());
set_dld_sequence_basic_preview_end:

	return err;

}
EXPORT_SYMBOL(ispdrv_set_dld_sequence_basic_preview);

/*
*\brief Set download sequence of advanced preview
*\param sensor_id [in], Sensor ID
*\param preview_adv_dld_seq_length   [in], length of the sequence
*\param preview_adv_dld_seq[in], the sequence
*\return error code
*/
u32 ispdrv_set_dld_sequence_adv_preview(u8 sensor_id,
					u8 preview_adv_dld_seq_length,
					u8 *preview_adv_dld_seq)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;
	struct dld_sequence *t_dld_sequence_to_be_set = NULL;
	u8 i = 0;

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_dld_sequence_adv_preview_end;
	}

	t_dld_sequence_to_be_set = get_shm_dld_seq_info_ptr(sensor_id);

	t_dld_sequence_to_be_set->preview_adv_dld_seq_length =
	    preview_adv_dld_seq_length;

	for (i = 0; i < preview_adv_dld_seq_length; i++)
		t_dld_sequence_to_be_set->preview_adv_dld_seq[i] =
		    preview_adv_dld_seq[i];

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_ADVDLDSEQ_PREVIEW;
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_ADVDLDSEQ_PREVIEW;
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_ADVDLDSEQ_PREVIEW;
	} else {
		err = EINVAL;
		goto set_dld_sequence_adv_preview_semapost;
	}

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* opstart*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_dld_sequence_adv_preview_semapost:
	SEMAPOST(get_command_semaphore());
set_dld_sequence_adv_preview_end:

	return err;

}
EXPORT_SYMBOL(ispdrv_set_dld_sequence_adv_preview);

/*
*\brief Set download sequence of basic fast converge
*\param sensor_id [in], Sensor ID
*\param fast_converge_baisc_dld_seq_length   [in], length of the sequence
*\param fast_converge_baisc_dld_seq [in], the sequence
*\return error code
*/
u32 ispdrv_set_dld_sequence_basic_fast_converge(u8 sensor_id,
			u8 fast_converge_baisc_dld_seq_length,
			u8 *fast_converge_baisc_dld_seq)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;
	struct dld_sequence *t_dld_sequence_to_be_set = NULL;
	u8 i = 0;

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_dld_sequence_basic_fast_converge_end;
	}

	t_dld_sequence_to_be_set = get_shm_dld_seq_info_ptr(sensor_id);

	t_dld_sequence_to_be_set->fast_converge_baisc_dld_seq_length =
	    fast_converge_baisc_dld_seq_length;

	for (i = 0; i < fast_converge_baisc_dld_seq_length; i++)
		t_dld_sequence_to_be_set->fast_converge_baisc_dld_seq[i] =
		    fast_converge_baisc_dld_seq[i];

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg =
		    AHB_ITEM_CHANGE_SENSOR1_BASICDLDSEQ_FASTCONVERGE;
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg =
		    AHB_ITEM_CHANGE_SENSOR2_BASICDLDSEQ_FASTCONVERGE;
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg =
		    AHB_ITEM_CHANGE_SENSOR3_BASICDLDSEQ_FASTCONVERGE;
	} else {
		err = EINVAL;
		goto set_dld_sequence_basic_fast_converge_semapost;
	}

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* opstart*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_dld_sequence_basic_fast_converge_semapost:
	SEMAPOST(get_command_semaphore());
set_dld_sequence_basic_fast_converge_end:

	return err;

}
EXPORT_SYMBOL(ispdrv_set_dld_sequence_basic_fast_converge);

/************************************************************
*		local public function		            *
*************************************************************/
void set_shm_dld_seq_info_ptr(u8 sensor_id, struct dld_sequence *ptr)
{
	isp_debug_lo_start_tag();

	isp_debug_item_tag("ptr:%p", ptr);
	g_kva_dld_seq_info_ptr[sensor_id] = ptr;

	isp_debug_lo_end_tag();
}

struct dld_sequence *get_shm_dld_seq_info_ptr(u8 sensor_id)
{
	return g_kva_dld_seq_info_ptr[sensor_id];
}

void **share_buff_get_shm_dld_seq_info_ptr_addr(u8 sensor_id)
{
	void **ptr = (void **)&g_kva_dld_seq_info_ptr[sensor_id];

	return ptr;
}

void set_shm_cfg_3a_info_ptr(u8 sensor_id, struct cfg_3a_info *ptr)
{
	isp_debug_lo_start_tag();

	isp_debug_item_tag("ptr:%p", ptr);
	g_kva_cfg_3a_info_ptr[sensor_id] = ptr;

	isp_debug_lo_end_tag();
}

struct cfg_3a_info *get_shm_cfg_3a_info_ptr(u8 sensor_id)
{
	return g_kva_cfg_3a_info_ptr[sensor_id];
}

void **share_buff_get_shm_cfg_3a_info_ptr_addr(u8 sensor_id)
{
	void **ptr = (void **)&g_kva_cfg_3a_info_ptr[sensor_id];

	return ptr;
}
