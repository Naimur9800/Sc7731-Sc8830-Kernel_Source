/*
* File: altek_iq_info.c                                   *
* Description: Implementaion of iq info api               *
*                                                         *
* (C)Copyright altek Corporation 2016                     *
*                                                         *
* History:                                                *
*   2016/03/012; Caedmon Lai; Initial version             *
*/

#include <linux/semaphore.h>

#include "altek_ahb_drv.h"
#include "altek_iq_info.h"
#include "altek_iq_info_func.h"
#include "altek_iq_info_local.h"
#include "altek_common.h"
#include "altek_sensor_info.h"
#include "altek_dump_utility.h"
#include "altek_log_local.h"
#include "altek_isp_drv.h"

/* RGB to YUV */
static u32 g_rgb2yuv_iva[SENSOR_TOTAL];
static struct ispdrv_rgb2yuv_martix *g_rgb2yuv[SENSOR_TOTAL];
/* Tone map */
static u32 g_tonemap_iva[SENSOR_TOTAL];
static struct ispdrv_tonemap_curve *g_tonemap[SENSOR_TOTAL];
/* Brightness */
static u32 g_brightness_setting_iva[SENSOR_TOTAL];
static struct ispdrv_brightness_gain_setting
	*g_brightness_setting[SENSOR_TOTAL];

/* iq ccm */
static u32 g_ccm_addr_iva[SENSOR_TOTAL];

/* iq otp */
static u32 g_otp_info_addr_iva[SENSOR_TOTAL];

/* isp d gain */
static u32 g_d_gain_addr_iva[SENSOR_TOTAL];

/* ccm info for each sensor */
static struct iq_ccm_info *g_kva_iq_info_ccm_addr[SENSOR_TOTAL];

/* IQ OTP info for each sensor */
static struct iq_otp_info *g_kva_iq_otp_info_ptr[SENSOR_TOTAL];

/* isp d gain for each sensor */
static struct isp_d_gain_info *g_kva_isp_d_gain_addr[SENSOR_TOTAL];

/* SS HQ capture, supporting first 2 sensors so far */
struct iq_raw_info_ss {
#if 0
	u32 raw_info_addr_iva[SENSOR_TOTAL];
	u8 *raw_info_addr_kva[SENSOR_TOTAL];
#endif
	u32 proc_still_arg_iva[SENSOR_TOTAL];
	u8 *proc_still_arg_kva[SENSOR_TOTAL];
};
static struct iq_raw_info_ss g_raw_info_ss;

struct iq_info_func_info {
	u64 buf_size;
	u32 fid_cnt;
	struct iq_addr_buf start_addr;
	struct iq_info_func_info_sns sns[SENSOR_TOTAL];
	struct iq_info_func_info_isp *info_isp[SENSOR_TOTAL];
};

struct iq_info_func_info g_iq_func_info;

/************************************************************
*               private api                                 *
*************************************************************/
/*
 *\brief allocate a mem space for a single specific func_id
 *\param *offest [out]
 *\param buf_size [in]
 *\param ap_tbl_addr [out] record working buf address on ap side
 *\param isp_fid_tbl [out] calculate each table's address on isp side
 */
static u32 iq_info_init_single_func_tbl(u32 *offset, u64 buf_size,
		struct iq_addr_buf *ap_tbl_addr,
		struct iq_info_func_tbl *isp_fid_tbl)
{
	u32 err = 0;
	int k;
	u32 temp_iva;
	u8  *temp_kva;
	u32 start_iva;
	u8  *start_kva;

	start_iva = g_iq_func_info.start_addr.iva;
	start_kva = g_iq_func_info.start_addr.kva;
	for (k = 0; k < IQ_INFO_E_FUNC_TBL_MAX; k++) {
		temp_iva = start_iva + *offset;
		temp_kva = start_kva + *offset;
		*offset = *offset + isp_fid_tbl->size;
		if (*offset > buf_size) {
			err = ENOMEM;
			break;
		}
		memset(temp_kva, 0, isp_fid_tbl->size);
		isp_fid_tbl->tbl_iva[k] = temp_iva;
		if (k == IQ_INFO_E_FUNC_TBL_TMP) {
			ap_tbl_addr->iva = temp_iva;
			ap_tbl_addr->kva = temp_kva;
		}
	}
	return err;
}

/*
 *\brief allocate struct iq_info_func_tbl which number is defined dynamically
 *\param *info_isp [out]
 *\param **func_tbl_kva [out]
 */
static u32 iq_info_alloc_func_tbl(u8 sensor_id __attribute__((unused)),
		u32 *offset, u64 buf_size,
		struct iq_info_func_info_isp *info_isp,
		u8 **func_tbl_kva)
{
	u32 err = 0;
	struct iq_addr_buf temp;
	u32 temp_size = 0;
	u32 fid_cnt = g_iq_func_info.fid_cnt;

	if (fid_cnt == 0)
		return err;
	temp.iva = g_iq_func_info.start_addr.iva + *offset;
	temp.kva = g_iq_func_info.start_addr.kva + *offset;
	temp_size = sizeof(struct iq_info_func_tbl) * fid_cnt;
	*offset = *offset + temp_size;
	if (*offset > buf_size) {
		err = ENOMEM;
		return err;
	}
	*func_tbl_kva = temp.kva;
	info_isp->tbl_iva = temp.iva;
	return err;
}

/*
 * \brief allocate function id tables and init their iva
 * \param sensor_id [in]
 * \param buf_size [in], the max size could be used
 * \param *offset [out]
 * \param *info_isp [out]
 */
static u32 iq_info_init_func_tbl(u8 sensor_id, u32 *offset,
		u64 buf_size, struct iq_info_func_info_isp *info_isp)
{
	u32 err = 0;
	int j;
	u32 fid_max = g_iq_func_info.fid_cnt;
	struct iq_info_func_tbl *p_tbl;

	err = iq_info_alloc_func_tbl(sensor_id, offset, buf_size, info_isp,
			(u8 **)&p_tbl);
	if (err)
		return err;
	for (j = 0; j < fid_max; j++) {
		p_tbl[j].size = iq_info_func_tbl_size_of(j);
		if (p_tbl[j].size == 0) {
			isp_info("%s - undefined func_id: %d",
					__func__,
					j);
			continue;
		}
		/* init all tables' addr */
		err = iq_info_init_single_func_tbl(
				offset, buf_size,
				&g_iq_func_info.sns[sensor_id].ap_tbl_addr[j],
				&p_tbl[j]);
		if (err)
			break;
	}
	return err;
}

static void iq_info_set_func_info_ahb(u8 sensor_id, u32 addr_iva)
{
	u32 ahb_value_reg = 0;

	switch (sensor_id) {
	case SENSOR1:
		ahb_value_reg = AHB_ITEM_SENSOR1_IQ_INFO_ADDR;
		break;
	case SENSOR2:
		ahb_value_reg = AHB_ITEM_SENSOR2_IQ_INFO_ADDR;
		break;
	case SENSOR3:
		ahb_value_reg = AHB_ITEM_SENSOR3_IQ_INFO_ADDR;
		break;
	default:
		break;
	}
	if (ahb_value_reg)
		isp_set_field_value(ahb_value_reg, addr_iva);
	isp_pr_debug("sensor[%d]: 0x%x", sensor_id, addr_iva);
}
/************************************************************
*		local public function		            *
*************************************************************/

void iq_info_set_rgb2yuv_iva(u8 sensor_id, u32 rgb2yuv_iva)
{
	g_rgb2yuv_iva[sensor_id] = rgb2yuv_iva;
}

u32 iq_info_get_rgb2yuv_iva(u8 sensor_id)
{
	return g_rgb2yuv_iva[sensor_id];
}

u32 *share_buff_iq_info_get_rgb2yuv_iva(u8 sensor_id)
{
	u32 *ptr = &g_rgb2yuv_iva[sensor_id];

	return ptr;
}

void iq_info_set_rgb2yuv(u8 sensor_id, struct ispdrv_rgb2yuv_martix *rgb2yuv)
{
	g_rgb2yuv[sensor_id] = rgb2yuv;
}

struct ispdrv_rgb2yuv_martix *iq_info_get_rgb2yuv(u8 sensor_id)
{
	return g_rgb2yuv[sensor_id];
}

void **share_buff_iq_info_get_rgb2yuv(u8 sensor_id)
{
	void **ptr = (void **)&(g_rgb2yuv[sensor_id]);

	return ptr;
}

void iq_info_set_tonemap_iva(u8 sensor_id, u32 tonemap_iva)
{
	g_tonemap_iva[sensor_id] = tonemap_iva;
}

u32 iq_info_get_tonemap_iva(u8 sensor_id)
{
	return g_tonemap_iva[sensor_id];
}

u32 *share_buff_iq_info_get_tonemap_iva(u8 sensor_id)
{
	u32 *ptr = &g_tonemap_iva[sensor_id];

	return ptr;
}

void iq_info_set_tonemap(u8 sensor_id, struct ispdrv_tonemap_curve *tonemap)
{
	g_tonemap[sensor_id] = tonemap;
}

struct ispdrv_tonemap_curve *iq_info_get_tonemap(u8 sensor_id)
{
	return g_tonemap[sensor_id];
}

void **share_buff_iq_info_get_tonemap(u8 sensor_id)
{
	void **ptr = (void **)&g_tonemap[sensor_id];

	return ptr;
}

void iq_info_set_brightness_setting_iva(u8 sensor_id
	, u32 brightness_setting_iva)
{
	g_brightness_setting_iva[sensor_id] = brightness_setting_iva;
}

u32 iq_info_get_brightness_setting_iva(u8 sensor_id)
{
	return g_brightness_setting_iva[sensor_id];
}

u32 *share_buff_iq_info_get_brightness_setting_iva(u8 sensor_id)
{
	u32 *ptr = &g_brightness_setting_iva[sensor_id];

	return ptr;
}

void iq_info_set_brightness_setting(u8 sensor_id,
	struct ispdrv_brightness_gain_setting *brightness_setting)
{
	g_brightness_setting[sensor_id] = brightness_setting;
}

struct ispdrv_brightness_gain_setting *iq_info_get_brightness_setting(
	u8 sensor_id)
{
	return g_brightness_setting[sensor_id];
}

void **share_buff_iq_info_brightness_setting(u8 sensor_id)
{
	void **ptr = (void **)&g_brightness_setting[sensor_id];

	return ptr;
}

void iq_info_set_ccm_addr_iva(u8 sensor_id, u32 ccm_iva)
{
	g_ccm_addr_iva[sensor_id] = ccm_iva;
}

u32 iq_info_get_ccm_addr_iva(u8 sensor_id)
{
	return g_ccm_addr_iva[sensor_id];
}

u32 *share_buff_iq_info_get_ccm_addr_iva(u8 sensor_id)
{
	u32 *ptr = &g_ccm_addr_iva[sensor_id];

	return ptr;
}

void iq_info_set_ccm_addr_kva(u8 sensor_id, struct iq_ccm_info *ccm_addr_kva)
{
	g_kva_iq_info_ccm_addr[sensor_id] = ccm_addr_kva;
}

struct iq_ccm_info *iq_info_get_ccm_addr_kva(u8 sensor_id)
{
	return g_kva_iq_info_ccm_addr[sensor_id];
}

void **share_buff_iq_info_get_ccm_addr_kva(u8 sensor_id)
{
	void **ptr = (void **)&g_kva_iq_info_ccm_addr[sensor_id];

	return ptr;
}

void iq_info_set_otp_addr_iva(u8 sensor_id, u32 otp_iva)
{
	g_otp_info_addr_iva[sensor_id] = otp_iva;
}

u32 iq_info_get_otp_addr_iva(u8 sensor_id)
{
	return g_otp_info_addr_iva[sensor_id];
}

u32 *share_buff_iq_info_get_otp_addr_iva(u8 sensor_id)
{
	u32 *ptr = &g_otp_info_addr_iva[sensor_id];

	return ptr;
}

void iq_info_set_otp_addr_kva(u8 sensor_id, struct iq_otp_info *otp_addr_kva)
{
	g_kva_iq_otp_info_ptr[sensor_id] = otp_addr_kva;
}

struct iq_otp_info *iq_info_get_otp_addr_kva(u8 sensor_id)
{
	return g_kva_iq_otp_info_ptr[sensor_id];
}

void **share_buff_iq_info_get_otp_addr_kva(u8 sensor_id)
{
	void **ptr = (void **)&g_kva_iq_otp_info_ptr[sensor_id];

	return ptr;
}

void iq_info_set_isp_d_gain_addr_iva(u8 sensor_id, u32 d_gain_iva)
{
	g_d_gain_addr_iva[sensor_id] = d_gain_iva;
}

u32 iq_info_get_isp_d_gain_addr_iva(u8 sensor_id)
{
	return g_d_gain_addr_iva[sensor_id];
}

u32 *share_buff_iq_info_get_isp_d_gain_addr_iva(u8 sensor_id)
{
	u32 *ptr = &g_d_gain_addr_iva[sensor_id];

	return ptr;
}

void iq_info_set_isp_d_gain_addr_kva(u8 sensor_id,
	struct isp_d_gain_info *isp_d_gain_addr_kva)
{
	g_kva_isp_d_gain_addr[sensor_id] = isp_d_gain_addr_kva;
}

struct isp_d_gain_info *iq_info_get_isp_d_gain_addr_kva(u8 sensor_id)
{
	return g_kva_isp_d_gain_addr[sensor_id];
}

void **share_buff_iq_info_get_isp_d_gain_addr_kva(u8 sensor_id)
{
	void **ptr = (void **)&g_kva_isp_d_gain_addr[sensor_id];

	return ptr;
}

#if 0
void iq_info_set_raw_info_addr_iva(u8 sensor_id, u32 raw_iva)
{
	g_raw_info_ss.raw_info_addr_iva[sensor_id] = raw_iva;
}

u32 iq_info_get_raw_info_addr_iva(u8 sensor_id)
{
	return g_raw_info_ss.raw_info_addr_iva[sensor_id];
}

void iq_info_set_raw_info_addr_kva(u8 sensor_id, u8 *raw_info_kva)
{
	g_raw_info_ss.raw_info_addr_kva[sensor_id] = raw_info_kva;
}

u8 *iq_info_get_raw_info_addr_kva(u8 sensor_id)
{
	return g_raw_info_ss.raw_info_addr_kva[sensor_id];
}
#endif
void iq_info_set_proc_still_arg_iva(u8 sensor_id, u32 arg_iva)
{
	g_raw_info_ss.proc_still_arg_iva[sensor_id] = arg_iva;
}

u32 iq_info_get_proc_still_arg_iva(u8 sensor_id)
{
	return g_raw_info_ss.proc_still_arg_iva[sensor_id];
}

u32 *share_buff_iq_info_get_proc_still_arg_iva(u8 sensor_id)
{
	u32 *ptr = &(g_raw_info_ss.proc_still_arg_iva[sensor_id]);

	return ptr;
}

void iq_info_set_proc_still_arg_kva(u8 sensor_id, u8 *pref)
{
	g_raw_info_ss.proc_still_arg_kva[sensor_id] = pref;
}

u8 *iq_info_get_proc_still_arg_kva(u8 sensor_id)
{
	return g_raw_info_ss.proc_still_arg_kva[sensor_id];
}

void **share_buff_iq_info_get_proc_still_arg_kva(u8 sensor_id)
{
	void **ptr = (void **)&(g_raw_info_ss.proc_still_arg_kva[sensor_id]);

	return ptr;
}

u32 *share_buff_iq_function_addr(void)
{
	u32 *ptr = &(g_iq_func_info.start_addr.iva);

	return ptr;
}
/*
 * set & init IQ function info, such as iva, kva address.
 */
u32 iq_info_init_func_info(u32 isp_base_iva, u8 *isp_base_kva)
{
	u32 err = 0;
	u32 temp_iva;
	u8  *temp_kva;
	u32 offset = 0;
	u32 temp_size = 0;
	int i;
	u64 buf_size;
	u32 start_iva;
	u8  *start_kva;

	start_iva = g_iq_func_info.start_addr.iva;
	buf_size = g_iq_func_info.buf_size;
	offset = start_iva - isp_base_iva;
	start_kva = (isp_base_kva + offset);
	offset = 0;
	g_iq_func_info.start_addr.kva = start_kva;
	for (i = SENSOR1; i < SENSOR_TOTAL; i++) {
		struct iq_info_func_info_isp *p_info_isp;

		/* mem access must be aligned 4-byte boundary.
		 * It'd crash on fw-side while non-4-byte boundary access
		 */
		offset = ALIGN(offset, 0x4);
		temp_iva = start_iva + offset;
		temp_kva = (start_kva + offset);
		temp_size = sizeof(struct iq_info_func_info_isp);
		offset += temp_size;

		/* init func table info for each sensor */
		p_info_isp = (struct iq_info_func_info_isp *) temp_kva;
		memset(p_info_isp, 0, temp_size);
		g_iq_func_info.sns[i].info_addr.iva = temp_iva;
		g_iq_func_info.sns[i].info_addr.kva = temp_kva;
		g_iq_func_info.info_isp[i] = p_info_isp;
		iq_info_set_func_info_ahb(i, temp_iva);

		/* allocate func id table */
		err = iq_info_init_func_tbl(i, &offset, buf_size, p_info_isp);
		if (err > 0)
			goto func_end;
	}

func_end:
	if (err > 0)
		isp_info_err("%d, offset: 0x%x, limit: 0x%llx",
				err, offset, buf_size);
	return err;
}

/*
 *\brief init global variables here
 */
void iq_info_init(void *pri_data)
{

}

void iq_info_deinit(void *pri_data)
{
	memset(g_rgb2yuv_iva, 0x0, sizeof(g_rgb2yuv_iva));
	memset(g_rgb2yuv, 0x0, sizeof(g_rgb2yuv));
	memset(g_tonemap_iva, 0x0, sizeof(g_tonemap_iva));
	memset(g_tonemap, 0x0, sizeof(g_tonemap));
	memset(g_brightness_setting_iva, 0x0, sizeof(g_brightness_setting_iva));
	memset(g_brightness_setting, 0x0, sizeof(g_brightness_setting));
	memset(g_ccm_addr_iva, 0x0, sizeof(g_ccm_addr_iva));
	memset(g_otp_info_addr_iva, 0x0, sizeof(g_otp_info_addr_iva));
	memset(g_kva_iq_info_ccm_addr, 0x0, sizeof(g_kva_iq_info_ccm_addr));
	memset(g_kva_iq_otp_info_ptr, 0x0, sizeof(g_kva_iq_otp_info_ptr));
	memset(&g_raw_info_ss, 0x0, sizeof(g_raw_info_ss));
	memset(&g_iq_func_info, 0x0, sizeof(g_iq_func_info));
}
/***********************************************************
*		IQ Info APIs		  *
*************************************************************/
/*
*\brief Set AWB gain value
*\param sensor_id [in], Sensor ID
*\param awb_gain_info   [in], AWB gain value
*\return error code
*/
u32 ispdrv_set_awb_gain(u8 sensor_id,
			struct awb_gain_info *awb_gain_info)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_awb_gain_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_DETECTED_AWB;
		isp_set_field_value(AHB_ITEM_SENSOR1_DETECTEDAWB_R,
		awb_gain_info->detected_awb_r);
		isp_set_field_value(AHB_ITEM_SENSOR1_DETECTEDAWB_GR,
		awb_gain_info->detected_awb_gr);
		isp_set_field_value(AHB_ITEM_SENSOR1_DETECTEDAWB_GB,
				    awb_gain_info->detected_awb_gb);
		isp_set_field_value(AHB_ITEM_SENSOR1_DETECTEDAWB_B,
		awb_gain_info->detected_awb_b);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_DETECTED_AWB;
		isp_set_field_value(AHB_ITEM_SENSOR2_DETECTEDAWB_R,
		awb_gain_info->detected_awb_r);
		isp_set_field_value(AHB_ITEM_SENSOR2_DETECTEDAWB_GR,
		awb_gain_info->detected_awb_gr);
		isp_set_field_value(AHB_ITEM_SENSOR2_DETECTEDAWB_GB,
		awb_gain_info->detected_awb_gb);
		isp_set_field_value(AHB_ITEM_SENSOR2_DETECTEDAWB_B,
		awb_gain_info->detected_awb_b);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_DETECTED_AWB;
		isp_set_field_value(AHB_ITEM_SENSOR3_DETECTEDAWB_R,
		awb_gain_info->detected_awb_r);
		isp_set_field_value(AHB_ITEM_SENSOR3_DETECTEDAWB_GR,
		awb_gain_info->detected_awb_gr);
		isp_set_field_value(AHB_ITEM_SENSOR3_DETECTEDAWB_GB,
		awb_gain_info->detected_awb_gb);
		isp_set_field_value(AHB_ITEM_SENSOR3_DETECTEDAWB_B,
		awb_gain_info->detected_awb_b);
	} else {
		err = EINVAL;
		goto set_awb_gain_semapost;
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

set_awb_gain_semapost:
	SEMAPOST(get_command_semaphore());
set_awb_gain_end:

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_awb_gain);


/*
*\brief Set Balanced AWB gain value
*\param sensor_id [in], Sensor ID
*\param awb_gain_info   [in], AWB gain value
*\return error code
*/
u32 ispdrv_set_balanced_awb_gain(u8 sensor_id,
				 struct awb_gain_info *awb_gain_info)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_balanced_awb_gain_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_BALANCED_AWB;
		isp_set_field_value(AHB_ITEM_SENSOR1_BALANCEDAWB_R,
		awb_gain_info->detected_awb_r);
		isp_set_field_value(AHB_ITEM_SENSOR1_BALANCEDAWB_GR,
		awb_gain_info->detected_awb_gr);
		isp_set_field_value(AHB_ITEM_SENSOR1_BALANCEDAWB_GB,
		awb_gain_info->detected_awb_gb);
		isp_set_field_value(AHB_ITEM_SENSOR1_BALANCEDAWB_B,
		awb_gain_info->detected_awb_b);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_BALANCED_AWB;
		isp_set_field_value(AHB_ITEM_SENSOR2_BALANCEDAWB_R,
		awb_gain_info->detected_awb_r);
		isp_set_field_value(AHB_ITEM_SENSOR2_BALANCEDAWB_GR,
		awb_gain_info->detected_awb_gr);
		isp_set_field_value(AHB_ITEM_SENSOR2_BALANCEDAWB_GB,
		awb_gain_info->detected_awb_gb);
		isp_set_field_value(AHB_ITEM_SENSOR2_BALANCEDAWB_B,
		awb_gain_info->detected_awb_b);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_BALANCED_AWB;
		isp_set_field_value(AHB_ITEM_SENSOR3_BALANCEDAWB_R,
		awb_gain_info->detected_awb_r);
		isp_set_field_value(AHB_ITEM_SENSOR3_BALANCEDAWB_GR,
		awb_gain_info->detected_awb_gr);
		isp_set_field_value(AHB_ITEM_SENSOR3_BALANCEDAWB_GB,
		awb_gain_info->detected_awb_gb);
		isp_set_field_value(AHB_ITEM_SENSOR3_BALANCEDAWB_B,
		awb_gain_info->detected_awb_b);
	} else {
		err = EINVAL;
		goto set_balanced_awb_gain_semapost;
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

set_balanced_awb_gain_semapost:
	SEMAPOST(get_command_semaphore());
set_balanced_awb_gain_end:

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_balanced_awb_gain);

/*
*\brief Set detected ISO value
*\param sensor_id [in], Sensor ID
*\param detected_iso	 [in], detected ISO value
*\return error code
*/
u32 ispdrv_set_iso_speed(u8 sensor_id, u16 detected_iso)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_iso_speed_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_DETECTED_ISO;
		isp_set_field_value(AHB_ITEM_SENSOR1_DETECTEDGAIN_ISO,
		detected_iso);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_DETECTED_ISO;
		isp_set_field_value(AHB_ITEM_SENSOR2_DETECTEDGAIN_ISO,
		detected_iso);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_DETECTED_ISO;
		isp_set_field_value(AHB_ITEM_SENSOR3_DETECTEDGAIN_ISO,
		detected_iso);
	} else {
		err = EINVAL;
		goto set_iso_speed_semapost;
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

set_iso_speed_semapost:
	SEMAPOST(get_command_semaphore());
set_iso_speed_end:

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_set_iso_speed);

/*
*\brief Set Shading correct mode
*\param sensor [in], Sensor ID
*\param mode [in], mode (ISPDRV_SHADING_MODE)
*\return error code
*/
u32 ispdrv_set_shading_correct_mode(u8 sensor_id, u8 mode)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (mode >= V4L2_ISP_SHADING_TOTAL) {
		err = EINVAL;
		goto set_shading_correct_mode_end;
	}

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_shading_correct_mode_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_SHADING;
		isp_set_field_value(AHB_ITEM_SENSOR1_SHADING, mode);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_SHADING;
		isp_set_field_value(AHB_ITEM_SENSOR2_SHADING, mode);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_SHADING;
		isp_set_field_value(AHB_ITEM_SENSOR3_SHADING, mode);
	} else {
		err = EINVAL;
		goto set_shading_correct_mode_semapost;
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

set_shading_correct_mode_semapost:
	SEMAPOST(get_command_semaphore());
set_shading_correct_mode_end:

	isp_debug_err("%d. mode: %d", err, mode);
	isp_debug_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_set_shading_correct_mode);

/*
*\brief Set RGB2YUY format mode
*\param sensor_id [in], Sensor ID
*\param mode   [in], RGB2YUV mode.(BT601/BT709/User define)
*\return error code
*/
u32 ispdrv_set_rgb2yuv_format_mode(u8 sensor_id, u8 mode)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (mode >= V4L2_ISP_RGB2YUV_TOTAL) {
		err = EINVAL;
		goto set_rgb2yuv_format_mode_end;
	}

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_rgb2yuv_format_mode_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_YUY2_FORMAT;
		isp_set_field_value(AHB_ITEM_SENSOR1_YUY2_FORMAT, mode);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_YUY2_FORMAT;
		isp_set_field_value(AHB_ITEM_SENSOR2_YUY2_FORMAT, mode);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_YUY2_FORMAT;
		isp_set_field_value(AHB_ITEM_SENSOR3_YUY2_FORMAT, mode);
	} else {
		err = EINVAL;
		goto set_rgb2yuv_format_mode_semapost;
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

set_rgb2yuv_format_mode_semapost:
	SEMAPOST(get_command_semaphore());
set_rgb2yuv_format_mode_end:

	isp_debug_err("%d mode: %d", err, mode);
	isp_debug_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_set_rgb2yuv_format_mode);

/*
*\brief Set RGB2YUV user define matrix
*\param sensor_id [in], Sensor ID
*\param rgb2yuv_matrix [in], RGB2YUV martix array
*\return EINVAL.(Incorrect sensor ID)
*/
u32 ispdrv_set_rgb2yuv_user_defined_matrix(u8 sensor_id,
	struct ispdrv_rgb2yuv_martix *rgb2yuv_matrix)
{
	u32 err = 0;

	isp_debug_pu_start_tag();

	if (sensor_id == SENSOR1)
		isp_set_field_value(AHB_ITEM_SENSOR1_YUY2_USER_DEF_ADDR,
		iq_info_get_rgb2yuv_iva(SENSOR1));
	else if (sensor_id == SENSOR2)
		isp_set_field_value(AHB_ITEM_SENSOR2_YUY2_USER_DEF_ADDR,
		iq_info_get_rgb2yuv_iva(SENSOR2));
	else if (sensor_id == SENSOR3)
		isp_set_field_value(AHB_ITEM_SENSOR3_YUY2_USER_DEF_ADDR,
		iq_info_get_rgb2yuv_iva(SENSOR3));
	else {
		err = EINVAL;
		isp_err_pu_combo_item_tag("error code:%d", err);
		goto set_rgb2yuv_user_defined_matrix_end;
	}

	/* copy the data to rgb2yuv data structure*/
	MEMCPY((u8 *) iq_info_get_rgb2yuv(sensor_id),
	(u8 *) rgb2yuv_matrix, sizeof(struct ispdrv_rgb2yuv_martix));

set_rgb2yuv_user_defined_matrix_end:

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_set_rgb2yuv_user_defined_matrix);

/*
*\brief Set Tone map Curve (gamma tone)
*\param sensor_id [in], Sensor ID
*\param a_pt_tone_addr  [in], address of the tone map
*\return EINVAL.(Incorrect sensor ID)
*/
u32 ispdrv_set_tonemap_curve(u8 sensor_id,
	struct ispdrv_tonemap_curve *tone_addr)
{
	u32 err = 0;

	isp_debug_pu_start_tag();

	if (sensor_id == SENSOR1)
		isp_set_field_value(AHB_ITEM_SENSOR1_TONEMAP_DATA_ADDR,
		iq_info_get_tonemap_iva(SENSOR1));
	else if (sensor_id == SENSOR2)
		isp_set_field_value(AHB_ITEM_SENSOR2_TONEMAP_DATA_ADDR,
		iq_info_get_tonemap_iva(SENSOR2));
	else if (sensor_id == SENSOR3)
		isp_set_field_value(AHB_ITEM_SENSOR3_TONEMAP_DATA_ADDR,
		iq_info_get_tonemap_iva(SENSOR3));
	else {
		err = EINVAL;
		isp_err_pu_combo_item_tag("error code:%d", err);
		goto set_tonemap_curve_end;
	}

	/* copy the data to tone map structure*/
	MEMCPY((u8 *) iq_info_get_tonemap(sensor_id), (u8 *) tone_addr,
	sizeof(struct ispdrv_tonemap_curve));

	/* opstart*/
	/* err = command_start(__func__);*/

set_tonemap_curve_end:

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_set_tonemap_curve);

/*
*\brief Set Tone map mode
*\param sensor_id [in], Sensor ID
*\param mode	 [in], mode (ispdrv_tonemap_mode)
*\return error code
*/
u32 ispdrv_set_tonemap_mode(u8 sensor_id, u8 mode)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (mode >= V4L2_ISP_TONEMAP_CURVE_MODE_TOTAL) {
		err = EINVAL;
		goto set_tonemap_mode_end;
	}

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_tonemap_mode_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_TONEMAP_MODE;
		isp_set_field_value(AHB_ITEM_SENSOR1_TONEMAP_MODE, mode);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_TONEMAP_MODE;
		isp_set_field_value(AHB_ITEM_SENSOR2_TONEMAP_MODE, mode);

	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_TONEMAP_MODE;
		isp_set_field_value(AHB_ITEM_SENSOR3_TONEMAP_MODE, mode);
	} else {
		err = EINVAL;
		goto set_tonemap_mode_semapost;
	}

	/* for test*/
#if 0
	if (a_ucMode == V4L2_ISP_TONEMap_CURVE_USER_DEFINE) {
		memset((u8 *) &g_tonemap.r[0], 0x80, TONESIZE);
		ISPDrv_SetTonemapCurve(a_uc_sensor, &g_tonemap);
	}
#endif

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* opstart*/
	err = command_start(__func__);
	/* err = command_start_poll(__func__);*/
	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_tonemap_mode_semapost:
	SEMAPOST(get_command_semaphore());
set_tonemap_mode_end:

	isp_debug_err("%d. mode: %d", err, mode);
	isp_debug_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_set_tonemap_mode);

/*
*\brief Set Sharpness mode
*\param sensor_id [in], Sensor ID
*\param mode	 [in], mode
*\return error code
*/
u32 ispdrv_set_sharpness_mode(u8 sensor_id, u8 mode)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (mode >= V4L2_ISP_SHARPNESS_TOTAL) {
		err = EINVAL;
		goto set_sharpness_mode_end;
	}
	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_sharpness_mode_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_SHARPNESS;
		isp_set_field_value(AHB_ITEM_SENSOR1_SHARPNESS, mode);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_SHARPNESS;
		isp_set_field_value(AHB_ITEM_SENSOR2_SHARPNESS, mode);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_SHARPNESS;
		isp_set_field_value(AHB_ITEM_SENSOR3_SHARPNESS, mode);
	} else {
		err = EINVAL;
		goto set_sharpness_mode_semapost;
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

set_sharpness_mode_semapost:
	SEMAPOST(get_command_semaphore());
set_sharpness_mode_end:
	isp_debug_err("%d. mode: %d", err, mode);
	isp_debug_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_set_sharpness_mode);

/*
*\brief Set Saturation Mode
*\param sensor_id [in], Sensor ID
*\param mode	 [in], mode, refer to ispdrv_saturation_mode
*\return error code
*/
u32 ispdrv_set_saturation_mode(u8 sensor_id, u8 mode)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (mode >= V4L2_ISP_SATURATION_TOTAL) {
		err = EINVAL;
		goto set_saturation_mode_end;
	}

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_saturation_mode_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_SATURATION;
		isp_set_field_value(AHB_ITEM_SENSOR1_SATURATION, mode);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_SATURATION;
		isp_set_field_value(AHB_ITEM_SENSOR2_SATURATION, mode);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_SATURATION;
		isp_set_field_value(AHB_ITEM_SENSOR3_SATURATION, mode);
	} else {
		err = EINVAL;
		goto set_saturation_mode_semapost;
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

set_saturation_mode_semapost:
	SEMAPOST(get_command_semaphore());
set_saturation_mode_end:

	isp_debug_err("%d. mode: %d", err, mode);
	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_saturation_mode);

/*
*\brief Set Contrast Mode
*\param sensor_id [in], Sensor ID
*\param mode	 [in], mode, refer to ispdrv_contrast_mode
*\return error code
*/
u32 ispdrv_set_contrast_mode(u8 sensor_id, u8 mode)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (mode >= V4L2_ISP_CONTRAST_TOTAL) {
		err = EINVAL;
		goto set_contrast_mode_end;
	}
	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_contrast_mode_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_CONTRAST;
		isp_set_field_value(AHB_ITEM_SENSOR1_CONTRAST, mode);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_CONTRAST;
		isp_set_field_value(AHB_ITEM_SENSOR2_CONTRAST, mode);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_CONTRAST;
		isp_set_field_value(AHB_ITEM_SENSOR3_CONTRAST, mode);
	} else {
		err = EINVAL;
		goto set_contrast_mode_semapost;
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

set_contrast_mode_semapost:
	SEMAPOST(get_command_semaphore());
set_contrast_mode_end:

	isp_debug_err("%d. mode: %d", err, mode);
	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_contrast_mode);

/*
*\brief Set Special Effect Mode
*\param sensor_id [in], Sensor ID
*\param mode	 [in], mode, refer to ispdrv_special_effect_mode
*\return error code
*/
u32 ispdrv_set_special_effect_mode(u8 sensor_id, u8 mode)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (mode >= V4L2_ISP_SPECIAL_EFFECT_TOTAL) {
		err = EINVAL;
		goto set_special_effect_mode_end;
	}
	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_special_effect_mode_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_SPECIAL_EFFECT;
		isp_set_field_value(AHB_ITEM_SENSOR1_SPECIAL_EFFECT, mode);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_SPECIAL_EFFECT;
		isp_set_field_value(AHB_ITEM_SENSOR2_SPECIAL_EFFECT, mode);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_SPECIAL_EFFECT;
		isp_set_field_value(AHB_ITEM_SENSOR3_SPECIAL_EFFECT, mode);
	} else {
		err = EINVAL;
		goto set_special_effect_mode_semapost;
	}

	/* set the command register*/
	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	/* opstart*/
	err = command_start(__func__);
	/*  err = command_start_poll(__func__);*/
	/* clear the command register*/
	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_special_effect_mode_semapost:
	SEMAPOST(get_command_semaphore());
set_special_effect_mode_end:

	isp_debug_err("%d. mode: %d", err, mode);
	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_special_effect_mode);

/*
 * \brief Set Brightness Mode.
 *	Function ispdrv_set_brightness_gain_setting should be called
 *	before call this function.
 * \param sensor_id [in], Sensor ID
 * \param mode   [in], mode 0 ~ 6, gain was defined at btightness gain setting
 * \return error code
 */
u32 ispdrv_set_brightness_mode(u8 sensor_id, u8 mode)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (mode >= BRIGHTNESS_ARRAY_SIZE) {
		err = EINVAL;
		goto set_brightness_mode_end;
	}
	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_brightness_mode_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_BRIGHTNESS;
		isp_set_field_value(AHB_ITEM_SENSOR1_BRIGHTNESS, mode);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_BRIGHTNESS;
		isp_set_field_value(AHB_ITEM_SENSOR2_BRIGHTNESS, mode);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_BRIGHTNESS;
		isp_set_field_value(AHB_ITEM_SENSOR3_BRIGHTNESS, mode);
	} else {
		err = EINVAL;
		goto set_brightness_mode_semapost;
	}

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	err = command_start(__func__);

	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_brightness_mode_semapost:
	SEMAPOST(get_command_semaphore());
set_brightness_mode_end:

	isp_debug_err("%d. mode: %d", err, mode);
	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_brightness_mode);

/**
 * \brief Set brightness gain setting. AP could set 7 brightness gains to ISP.
	The value range is 0 ~ 200. Value 150 means gain 1.50.
 * \param a_uc_sensor [in], Sensor ID
 * \param brghtness_gain_addr  [in], address of the brightness gain array
 * \return EINVAL.(Incorrect sensor ID)
 */
u32 ispdrv_set_brightness_gain_setting(u8 sensor_id,
	struct ispdrv_brightness_gain_setting *brghtness_gain_addr)
{
	u32 err = 0;

	isp_debug_pu_start_tag();

	if (sensor_id == SENSOR1)
		isp_set_field_value(AHB_ITEM_SENSOR1_BRIGHTNESS_SETTING_ADDR,
			iq_info_get_brightness_setting_iva(SENSOR1));
	else if (sensor_id == SENSOR2)
		isp_set_field_value(AHB_ITEM_SENSOR2_BRIGHTNESS_SETTING_ADDR,
			iq_info_get_brightness_setting_iva(SENSOR2));
	else if (sensor_id == SENSOR3)
		isp_set_field_value(AHB_ITEM_SENSOR3_BRIGHTNESS_SETTING_ADDR,
			iq_info_get_brightness_setting_iva(SENSOR3));
	else {
		err = EINVAL;
		goto set_brightness_gain_setting_end;
	}

	MEMCPY((u8 *) iq_info_get_brightness_setting(sensor_id),
			(u8 *) brghtness_gain_addr,
			sizeof(struct ispdrv_brightness_gain_setting));


set_brightness_gain_setting_end:

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_set_brightness_gain_setting);


u32 ispdrv_set_color_temperature(u8 sensor_id, u32 color_tmp)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_color_temperature_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_COLOR_TEMPERATURE;
		isp_set_field_value(AHB_ITEM_SENSOR1_COLOR_TEMPERATURE,
				color_tmp);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_COLOR_TEMPERATURE;
		isp_set_field_value(AHB_ITEM_SENSOR2_COLOR_TEMPERATURE,
				color_tmp);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_COLOR_TEMPERATURE;
		isp_set_field_value(AHB_ITEM_SENSOR3_COLOR_TEMPERATURE,
				color_tmp);
	} else {
		err = EINVAL;
		goto set_color_temperature_semapost;
	}

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	err = command_start(__func__);

	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_color_temperature_semapost:
	SEMAPOST(get_command_semaphore());
set_color_temperature_end:

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_color_temperature);

/*total length = 9, an array for hardware, each has 32 bits*/
u32 ispdrv_set_ccm(u8 sensor_id, struct iq_ccm_info *ccm)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_ccm_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_CCM;
		memcpy((u8 *)iq_info_get_ccm_addr_kva(sensor_id),
				(u8 *)ccm, sizeof(struct iq_ccm_info));
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_CCM;
		memcpy((u8 *)iq_info_get_ccm_addr_kva(sensor_id),
				(u8 *)ccm, sizeof(struct iq_ccm_info));
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_CCM;
		memcpy((u8 *)iq_info_get_ccm_addr_kva(sensor_id),
				(u8 *)ccm, sizeof(struct iq_ccm_info));
	} else {
		err = EINVAL;
		goto set_ccm_semapost;
	}

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	err = command_start(__func__);

	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_ccm_semapost:
	SEMAPOST(get_command_semaphore());
set_ccm_end:

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_ccm);

/**
 * \brief Set IQ otp information
 * \param a_ucSensor [In], the sensor ID
 * \param p_uc_otp_info [In],IQ otp information value
 * \return EINVAL
*/
u32 ispdrv_set_iq_otp_info(u8 sensor_id, struct iq_otp_info *otp_info)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (otp_info == NULL) {
		err = EINVAL;
		goto set_iq_otp_info_end;
	}

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_iq_otp_info_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_IQ_OTP_INFO;
		memcpy((u8 *)iq_info_get_otp_addr_kva(sensor_id),
				(u8 *)otp_info, sizeof(struct iq_otp_info));
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_IQ_OTP_INFO;
		memcpy((u8 *)iq_info_get_otp_addr_kva(sensor_id),
				(u8 *)otp_info, sizeof(struct iq_otp_info));
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_IQ_OTP_INFO;
		memcpy((u8 *)iq_info_get_otp_addr_kva(sensor_id),
				(u8 *)otp_info, sizeof(struct iq_otp_info));
	} else {
		err = EINVAL;
		goto set_iq_otp_info_semapost;
	}

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	err = command_start(__func__);

	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_iq_otp_info_semapost:
	SEMAPOST(get_command_semaphore());
set_iq_otp_info_end:

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_iq_otp_info);

u32 ispdrv_set_valid_ad_gain(u8 sensor_id, u32 gain)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_valid_ad_gain_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_VALID_AD_GAIN;
		isp_set_field_value(AHB_ITEM_SENSOR1_VALID_AD_GAIN, gain);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_VALID_AD_GAIN;
		isp_set_field_value(AHB_ITEM_SENSOR2_VALID_AD_GAIN, gain);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_VALID_AD_GAIN;
		isp_set_field_value(AHB_ITEM_SENSOR3_VALID_AD_GAIN, gain);
	} else {
		err = EINVAL;
		goto set_valid_ad_gain_semapost;
	}

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	err = command_start(__func__);

	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_valid_ad_gain_semapost:
	SEMAPOST(get_command_semaphore());
set_valid_ad_gain_end:

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_valid_ad_gain);

u32 ispdrv_set_valid_exp_time(u8 sensor_id, u32 exptime)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_valid_exp_time_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg =
				AHB_ITEM_CHANGE_SENSOR1_VALID_EXPOSURE_TIME;
		isp_set_field_value(AHB_ITEM_SENSOR1_VALID_EXPOSURE_TIME,
				exptime);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg =
				AHB_ITEM_CHANGE_SENSOR2_VALID_EXPOSURE_TIME;
		isp_set_field_value(AHB_ITEM_SENSOR2_VALID_EXPOSURE_TIME,
				exptime);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg =
				AHB_ITEM_CHANGE_SENSOR3_VALID_EXPOSURE_TIME;
		isp_set_field_value(AHB_ITEM_SENSOR3_VALID_EXPOSURE_TIME,
				exptime);
	} else {
		err = EINVAL;
		goto set_valid_exp_time_semapost;
	}

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	err = command_start(__func__);

	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_valid_exp_time_semapost:
	SEMAPOST(get_command_semaphore());
set_valid_exp_time_end:

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_set_valid_exp_time);

u32 ispdrv_set_iq_info(u8 sensor_id, u32 func_id, void *in_data, u32 length)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;
	u8 *func_tbl_kva;
	struct iq_info_func_info_isp *info_isp;

	isp_debug_pu_start_tag();
	isp_debug_item_tag("Sensor: %d, func_id: %d, leng: %d",
			sensor_id, func_id, length);
	if (func_id >= g_iq_func_info.fid_cnt) {
		err = EINVAL;
		goto func_end;
	}
	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert("%s - interrupted.", __func__);
		err = SEMAERR;
		goto func_sema_post;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_IQ_INFO;
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_IQ_INFO;
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_IQ_INFO;
	} else {
		err = EINVAL;
		goto func_sema_post;
	}
	func_tbl_kva = g_iq_func_info.sns[sensor_id].ap_tbl_addr[func_id].kva;
	if (func_tbl_kva != NULL) {
		info_isp = g_iq_func_info.info_isp[sensor_id];
		info_isp->func_id = func_id;
		memcpy(func_tbl_kva, in_data, length);
		isp_pr_debug("data: 0x%x 0x%x 0x%x 0x%x 0x%x",
				*func_tbl_kva, *(func_tbl_kva + 1),
				*(func_tbl_kva + 2), *(func_tbl_kva + 3),
				*(func_tbl_kva + 4));
	}

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	err = command_start(__func__);

	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

func_sema_post:
	SEMAPOST(get_command_semaphore());
func_end:
	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();
	return err;
}
EXPORT_SYMBOL(ispdrv_set_iq_info);

u32 ispdrv_set_isp_d_gain(u8 sensor_id, struct isp_d_gain_info *isp_d_gain)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_isp_d_gain_end;
	}

	/* check the value of r of d gain*/
	if (isp_d_gain->isp_d_gain_r < 100000)
		isp_d_gain->isp_d_gain_r = 100000;
	else if (isp_d_gain->isp_d_gain_r > 1596875)
		isp_d_gain->isp_d_gain_r = 1596875;
	/* check the value of gr of d gain*/
	if (isp_d_gain->isp_d_gain_gr < 100000)
		isp_d_gain->isp_d_gain_gr = 100000;
	else if (isp_d_gain->isp_d_gain_gr > 1596875)
		isp_d_gain->isp_d_gain_gr = 1596875;
	/* check the value of gb of d gain*/
	if (isp_d_gain->isp_d_gain_gb < 100000)
		isp_d_gain->isp_d_gain_gb = 100000;
	else if (isp_d_gain->isp_d_gain_gb > 1596875)
		isp_d_gain->isp_d_gain_gb = 1596875;
	/* check the value of b of d gain*/
	if (isp_d_gain->isp_d_gain_b < 100000)
		isp_d_gain->isp_d_gain_b = 100000;
	else if (isp_d_gain->isp_d_gain_b > 1596875)
		isp_d_gain->isp_d_gain_b = 1596875;

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_ISP_D_GAIN;
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_ISP_D_GAIN;
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_ISP_D_GAIN;
	} else {
		err = EINVAL;
		goto set_isp_d_gain_semapost;
	}

	memcpy((u8 *)iq_info_get_isp_d_gain_addr_kva(sensor_id),
	(u8 *)isp_d_gain, sizeof(struct isp_d_gain_info));

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	err = command_start(__func__);

	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_isp_d_gain_semapost:
	SEMAPOST(get_command_semaphore());
set_isp_d_gain_end:

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_set_isp_d_gain);

u32 ispdrv_set_y_offset(u8 sensor_id, u32 y_offset)
{
	u32 err = 0, command_sensor_reg = 0, command_system_reg = 0;

	isp_debug_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_y_offset_end;
	}

	if (sensor_id == SENSOR1) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_Y_OFFSET;
		isp_set_field_value(AHB_ITEM_SENSOR1_Y_OFFSET, y_offset);
	} else if (sensor_id == SENSOR2) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_Y_OFFSET;
		isp_set_field_value(AHB_ITEM_SENSOR2_Y_OFFSET, y_offset);
	} else if (sensor_id == SENSOR3) {
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_Y_OFFSET;
		isp_set_field_value(AHB_ITEM_SENSOR3_Y_OFFSET, y_offset);
	} else {
		err = EINVAL;
		goto set_y_offset_semapost;
	}

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	err = command_start(__func__);

	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);

set_y_offset_semapost:
	SEMAPOST(get_command_semaphore());
set_y_offset_end:

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_set_y_offset);


/*
 * set base iva & buffer size of general iq info
 */
void ispdrv_set_iq_buffer_mem_info(u32 start_iva, u64 buf_size)
{
	isp_pr_debug("start_iva: 0x%x, size: 0x%llx", start_iva, buf_size);
	g_iq_func_info.start_addr.iva = start_iva;
	g_iq_func_info.buf_size = buf_size;
}
EXPORT_SYMBOL(ispdrv_set_iq_buffer_mem_info);

u32 ispdrv_set_iq_func_info(u32 fid_cnt, u32 *fid_size)
{
	u32 err = 0;
	int i;

	isp_info_pu_start_tag();
	if (fid_cnt >= IQ_INFO_FUNC_ID_MAX) {
		err = EINVAL;
		goto func_end;
	}
	g_iq_func_info.fid_cnt = fid_cnt;
	for (i = 0; i < fid_cnt; i++) {
		isp_pr_debug("fid: %d, f_size: 0x%x", i, *(fid_size + i));
		iq_info_set_func_tbl_size(i, fid_size[i]);
	}

func_end:
	if (err != 0)
		isp_info_err("%s - err: %d, fid_cnt: %d",
				__func__, err, fid_cnt);
	isp_info_pu_end_tag();
	return err;
}
EXPORT_SYMBOL(ispdrv_set_iq_func_info);

u32 ispdrv_set_iq_param_index(u8 sensor_id,
	u8 change_flag,
	u8 *iq_param_index)
{
	u32 err = 0;
	int i;
	u32 command_sensor_reg = 0, command_system_reg = 0;
	u32 iq_param_idx_reg[3] = {0, 0, 0};

	isp_info_pu_start_tag();

	if (SEMAWAIT(get_command_semaphore())) {
		isp_alert_pu_combo_desc_tag("interrupted.");
		err = SEMAERR;
		goto set_quality_index_end;
	}

	if (sensor_id == SENSOR1) {
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR1_SETTINGS;
		command_system_reg = AHB_ITEM_CHANGE_SENSOR1_IQ_PARAM_IDX;
		iq_param_idx_reg[V4L2_ISP_IMG1] =
			AHB_ITEM_SENSOR1_IQ_PARAM_IDX_LV;
		iq_param_idx_reg[V4L2_ISP_IMG2] =
			AHB_ITEM_SENSOR1_IQ_PARAM_IDX_VIDEO;
		iq_param_idx_reg[V4L2_ISP_IMG3] =
			AHB_ITEM_SENSOR1_IQ_PARAM_IDX_STILL;
	} else if (sensor_id == SENSOR2) {
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR2_SETTINGS;
		command_system_reg = AHB_ITEM_CHANGE_SENSOR2_IQ_PARAM_IDX;
		iq_param_idx_reg[V4L2_ISP_IMG1] =
			AHB_ITEM_SENSOR2_IQ_PARAM_IDX_LV;
		iq_param_idx_reg[V4L2_ISP_IMG2] =
			AHB_ITEM_SENSOR2_IQ_PARAM_IDX_VIDEO;
		iq_param_idx_reg[V4L2_ISP_IMG3] =
			AHB_ITEM_SENSOR2_IQ_PARAM_IDX_STILL;
	} else if (sensor_id == SENSOR3) {
		command_sensor_reg = AHB_ITEM_CHANGE_SENSOR3_SETTINGS;
		command_system_reg = AHB_ITEM_CHANGE_SENSOR3_IQ_PARAM_IDX;
		iq_param_idx_reg[V4L2_ISP_IMG1] =
			AHB_ITEM_SENSOR3_IQ_PARAM_IDX_LV;
	} else {
		err = EINVAL;
		goto set_quality_index_semapost;
	}

	/* set the iq parameter index into AHB registers for 3 paths */
	for (i = V4L2_ISP_IMG1; i <= V4L2_ISP_IMG3; i++) {
		if (sensor_id == SENSOR3 && i > V4L2_ISP_IMG1)
			break;
		if (change_flag & (1 << i))
			isp_set_field_value(iq_param_idx_reg[i],
				iq_param_index[i]);
		else
			isp_set_field_value(iq_param_idx_reg[i], 0xff);
	}

	isp_set_field_value(command_system_reg, 1);
	isp_set_field_value(command_sensor_reg, 1);

	err = command_start(__func__);

	isp_set_field_value(command_system_reg, 0);
	isp_set_field_value(command_sensor_reg, 0);


set_quality_index_semapost:
	SEMAPOST(get_command_semaphore());

set_quality_index_end:
	if (err != 0)
		isp_info_err("%s - err: %d", __func__, err);

	isp_info_pu_end_tag();
	return err;
}
EXPORT_SYMBOL(ispdrv_set_iq_param_index);

