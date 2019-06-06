/*==============================================================================
 *
 * Copyright (c) 2006-2012 MStar Semiconductor, Inc.
 * All rights reserved.
 *
 * Unless otherwise stipulated in writing, any and all information contained
 * herein regardless in any format shall remain the sole proprietary of
 * MStar Semiconductor Inc. and be kept in strict confidence
 * (??MStar Confidential Information??) by the recipient.
 * Any unauthorized act including without limitation unauthorized disclosure,
 * copying, use, reproduction, sale, distribution, modification, disassembling,
 * reverse engineering and compiling of the contents of MStar Confidential
 * Information is unlawful and strictly prohibited. MStar hereby reserves the
 * rights to any and all damages, losses, costs and expenses resulting therefrom
.
 *
==============================================================================*/

/**
 *
 * @file    mstar_drv_main.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

/*=============================================================*/
/* INCLUDE FILE */
/*=============================================================*/

#include "mstar_drv_main.h"
#include "mstar_drv_utility_adaption.h"
#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_ic_fw_porting_layer.h"

/*=============================================================*/
/* CONSTANT VALUE DEFINITION */
/*=============================================================*/

/*=============================================================*/
/* LOCAL VARIABLE DEFINITION */
/*=============================================================*/

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
static u16 _g_debug_reg[MAX_DEBUG_REGISTER_NUM] = { 0 };

static u32 _g_debug_reg_cnt; /* = 0; */
/* internal use firmware version for MStar */
static u8 *_g_platform_fwversion; /* = NULL; */
#endif

#ifdef CONFIG_ENABLE_ITO_MP_TEST
static enum ito_testmode_e _g_ito_testmode; /* = 0; */
#endif/* CONFIG_ENABLE_ITO_MP_TEST */

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
static u32 _g_isupdate_complete; /* = 0; */

static u8 *_g_fw_version; /* = NULL; *//* customer firmware version */
#endif

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
static struct class *_g_firmware_class; /* = NULL; */
static struct device *_g_firmware_cmd_dev; /* = NULL; */
#endif

/*=============================================================*/
/* GLOBAL VARIABLE DEFINITION */
/*=============================================================*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
struct kset *g_touch_kset; /*  = NULL;  */
struct kobject *g_touch_kobj; /*  = NULL; */
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

u8 g_fwdata[94][1024];
u32 g_fwdata_count; /* = 0; */

/*=============================================================*/
/* LOCAL FUNCTION DEFINITION */
/*=============================================================*/

/*=============================================================*/
/* GLOBAL FUNCTION DEFINITION */
/*=============================================================*/

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
ssize_t main_firmware_chiptype_show(struct device *p_device,
	struct device_attribute *p_attr, char *p_buf)
{
	LOGTP_FUNC();

	return sprintf(p_buf, "%d", g_chiptype);
}

ssize_t main_firmware_chiptype_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	LOGTP_FUNC();

/*    g_chiptype = fwic_get_chiptype(); */

	return n_size;
}

static DEVICE_ATTR(chip_type, SYSFS_AUTHORITY, main_firmware_chiptype_show,
	main_firmware_chiptype_store);

ssize_t main_firmware_driverversion_show(struct device *p_device,
	struct device_attribute *p_attr,
	char *p_buf)
{
	LOGTP_FUNC();

	return sprintf(p_buf, "%s", DEVICE_DRIVER_RELEASE_VERSION);
}

ssize_t main_firmware_driverversion_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	LOGTP_FUNC();

	return n_size;
}

static DEVICE_ATTR(driver_version, SYSFS_AUTHORITY,
	main_firmware_driverversion_show,
	main_firmware_driverversion_store);
#endif

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
ssize_t main_firmware_update_show(struct device *p_device,
	struct device_attribute *p_attr, char *p_buf)
{
	LOGTP_DBUG("%s() _g_fw_version = %s\n", __func__, _g_fw_version);

	return sprintf(p_buf, "%s\n", _g_fw_version);
}

ssize_t main_firmware_update_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	platform_disable_finger_touchreport();

	LOGTP_DBUG("%s() g_fwdata_count = %d\n", __func__, g_fwdata_count);

	if (0 != fwic_update_firmware(g_fwdata, EMEM_ALL)) {
		_g_isupdate_complete = 0;
		LOGTP_DBUG("Update FAILED\n");
	} else {
		_g_isupdate_complete = 1;
		LOGTP_DBUG("Update SUCCESS\n");
	}

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
	fwic_restore_firmwaremode_tologdata();
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

	platform_enable_finger_touchreport();

	return n_size;
}

static DEVICE_ATTR(update, SYSFS_AUTHORITY, main_firmware_update_show,
	main_firmware_update_store);

ssize_t main_firmware_versionshow(struct device *p_device,
	struct device_attribute *p_attr, char *p_buf)
{
	LOGTP_DBUG("%s() _g_fw_version = %s\n", __func__, _g_fw_version);

	return sprintf(p_buf, "%s\n", _g_fw_version);
}

ssize_t main_firmware_version_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	u16 n_major = 0, n_minor = 0;

	fwic_getcustomer_firmware_version(&n_major, &n_minor, &_g_fw_version);

	LOGTP_DBUG("%s() _g_fw_version = %s\n", __func__, _g_fw_version);

	return n_size;
}

static DEVICE_ATTR(version, SYSFS_AUTHORITY, main_firmware_versionshow,
	main_firmware_version_store);

ssize_t main_firmware_datashow(struct device *p_device,
				struct device_attribute *p_attr, char *p_buf)
{
	LOGTP_DBUG("%s() g_fwdata_count = %d\n", __func__, g_fwdata_count);

	return g_fwdata_count;
}

ssize_t main_firmware_data_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	u32 n_count = n_size / 1024;
	u32 i;

	LOGTP_FUNC();

	if (n_count > 0)		 {/* n_size >= 1024 */
		for (i = 0; i < n_count; i++) {
			memcpy(g_fwdata[g_fwdata_count], p_buf + (i * 1024),
	1024);

			g_fwdata_count++;
		}
		} else {/* n_size < 1024 */
		if (n_size > 0) {
			memcpy(g_fwdata[g_fwdata_count], p_buf, n_size);

			g_fwdata_count++;
		}
	}

	LOGTP_DBUG("g_fwdata_count = %d\n", g_fwdata_count);

	if (p_buf != NULL)
		LOGTP_DBUG("buf[0] = %c\n", p_buf[0]);


	return n_size;
}

static DEVICE_ATTR(data, SYSFS_AUTHORITY, main_firmware_datashow,
	main_firmware_data_store);
#endif

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_ITO_MP_TEST
ssize_t main_firmware_test_show(struct device *p_device,
				struct device_attribute *p_attr, char *p_buf)
{
	LOGTP_FUNC();
	LOGTP_DBUG("ctp mp test status = %d\n", fw_get_mptest_result());

	return sprintf(p_buf, "%d", fw_get_mptest_result());
}

ssize_t main_firmware_test_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	u32 n_mode = 0;

	LOGTP_FUNC();

	if (p_buf != NULL) {
		sscanf(p_buf, "%x", &n_mode);

		LOGTP_DBUG("Mp Test Mode = 0x%x\n", n_mode);

		if (n_mode == ITO_TEST_MODE_OPEN_TEST)	 {/* open test */
			_g_ito_testmode = ITO_TEST_MODE_OPEN_TEST;
			fwic_schedule_mptest_work(ITO_TEST_MODE_OPEN_TEST);
		/* short test */
		} else if (n_mode == ITO_TEST_MODE_SHORT_TEST)	 {
			_g_ito_testmode = ITO_TEST_MODE_SHORT_TEST;
			fwic_schedule_mptest_work(ITO_TEST_MODE_SHORT_TEST);
		} else {
			LOGTP_DBUG("Undefined MP Test Mode\n");
		}
	}

	return n_size;
}

static DEVICE_ATTR(test, SYSFS_AUTHORITY, main_firmware_test_show,
	main_firmware_test_store);

ssize_t main_firmware_log_show(struct device *p_device,
	struct device_attribute *p_attr, char *p_buf)
{
	u32 n_len = 0;

	LOGTP_FUNC();

	fw_get_mptest_datalog(_g_ito_testmode, p_buf, &n_len);

	return n_len;
}

ssize_t main_firmware_log_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	LOGTP_FUNC();

	return n_size;
}

static DEVICE_ATTR(test_log, SYSFS_AUTHORITY, main_firmware_log_show,
	main_firmware_log_store);

ssize_t main_firmware_failchannel_show(struct device *p_device,
	struct device_attribute *p_attr,
	char *p_buf)
{
	u32 n_count = 0;

	LOGTP_FUNC();

	fw_get_testfail_channel(_g_ito_testmode, p_buf, &n_count);

	return n_count;
}

ssize_t main_firmware_failchannel_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	LOGTP_FUNC();

	return n_size;
}

static DEVICE_ATTR(test_fail_channel, SYSFS_AUTHORITY,
	main_firmware_failchannel_show,
	main_firmware_failchannel_store);
#endif/* CONFIG_ENABLE_ITO_MP_TEST */

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP

ssize_t main_firmware_gesture_wakeup_modeshow(struct device *p_device,
	struct device_attribute *p_attr,
	char *p_buf)
{
	LOGTP_FUNC();
	LOGTP_DBUG("g_gesture_wakeup_mode = 0x%x\n", g_gesture_wakeup_mode);

	return sprintf(p_buf, "%x", g_gesture_wakeup_mode);
}

ssize_t main_firmware_gesture_wakeupmode_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	u32 n_len, n_wakeup_mode;

	LOGTP_FUNC();

	if (p_buf != NULL) {
		sscanf(p_buf, "%x", &n_wakeup_mode);
		LOGTP_DBUG("n_wakeup_mode = 0x%x\n", n_wakeup_mode);

		n_len = n_size;
		LOGTP_DBUG("n_len = %d\n", n_len);

		if ((n_wakeup_mode & GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG) ==
	GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG) {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode |
	GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG;
		} else {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode &
	(~GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG);
		}

		if ((n_wakeup_mode & GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG) ==
	GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG) {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode |
	GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG;
		} else {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode &
	(~GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG);
		}

		if ((n_wakeup_mode & GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG) ==
	GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG) {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode |
	GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG;
		} else {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode &
	(~GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG);
		}

		if ((n_wakeup_mode & GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG) ==
	GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG) {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode |
	GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG;
		} else {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode &
	(~GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG);
		}

		if ((n_wakeup_mode & GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG) ==
	GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG) {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode |
	GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG;
		} else {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode &
	(~GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG);
		}

		if ((n_wakeup_mode & GESTURE_WAKEUP_MODE_M_CHARACTER_FLAG) ==
	GESTURE_WAKEUP_MODE_M_CHARACTER_FLAG) {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode |
	GESTURE_WAKEUP_MODE_M_CHARACTER_FLAG;
		} else {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode &
	(~GESTURE_WAKEUP_MODE_M_CHARACTER_FLAG);
		}

		if ((n_wakeup_mode & GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG) ==
	GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG) {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode |
	GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG;
		} else {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode &
	(~GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG);
		}

		if ((n_wakeup_mode & GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG) ==
	GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG) {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode |
	GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG;
		} else {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode &
	(~GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG);
		}

		if ((n_wakeup_mode & GESTURE_WAKEUP_MODE_E_CHARACTER_FLAG) ==
	GESTURE_WAKEUP_MODE_E_CHARACTER_FLAG) {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode |
	GESTURE_WAKEUP_MODE_E_CHARACTER_FLAG;
		} else {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode &
	(~GESTURE_WAKEUP_MODE_E_CHARACTER_FLAG);
		}

		if ((n_wakeup_mode & GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG) ==
	GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG) {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode |
	GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG;
		} else {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode &
	(~GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG);
		}

		if ((n_wakeup_mode & GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG) ==
	GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG) {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode |
	GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG;
		} else {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode &
	(~GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG);
		}

		if ((n_wakeup_mode & GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG) ==
	GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG) {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode |
	GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG;
		} else {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode &
	(~GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG);
		}

		if ((n_wakeup_mode & GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG) ==
	GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG) {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode |
	GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG;
		} else {
			g_gesture_wakeup_mode =
	g_gesture_wakeup_mode &
	(~GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG);
		}

		LOGTP_DBUG("wakeup_mode = 0x%x\n", g_gesture_wakeup_mode);
	}

	return n_size;
}

static DEVICE_ATTR(gesture_wakeup_mode, SYSFS_AUTHORITY,
	main_firmware_gesture_wakeup_modeshow,
	main_firmware_gesture_wakeupmode_store);

#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
ssize_t main_firmware_debug_show(struct device *p_device,
	struct device_attribute *p_attr, char *p_buf)
{
	u32 i;
	u8 n_bank, n_addr;
	u16 sz_reg_data[MAX_DEBUG_REGISTER_NUM] = { 0 };
	u8 sz_out[MAX_DEBUG_REGISTER_NUM * 25] = { 0 }, sz_value[10] = {
	0};

	LOGTP_FUNC();

	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();
	mdelay(300);

	for (i = 0; i < _g_debug_reg_cnt; i++)
		sz_reg_data[i] = reg_get16bit_value(_g_debug_reg[i]);


	bus_i2c_notuse_buf();
	buf_not_stop_mcu();
	bus_exit_serial_debugmode();

	for (i = 0; i < _g_debug_reg_cnt; i++) {
		n_bank = (_g_debug_reg[i] >> 8) & 0xFF;
		n_addr = _g_debug_reg[i] & 0xFF;

		LOGTP_DBUG("reg(0x%X,0x%X)=0x%04X\n", n_bank,
				n_addr, sz_reg_data[i]);

		strcat(sz_out, "reg(");
		sprintf(sz_value, "0x%X", n_bank);
		strcat(sz_out, sz_value);
		strcat(sz_out, ",");
		sprintf(sz_value, "0x%X", n_addr);
		strcat(sz_out, sz_value);
		strcat(sz_out, ")=");
		sprintf(sz_value, "0x%04X", sz_reg_data[i]);
		strcat(sz_out, sz_value);
		strcat(sz_out, "\n");
	}

	return sprintf(p_buf, "%s\n", sz_out);
}

ssize_t main_firmware_debug_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	u32 i;
	char *p_chr;

	LOGTP_FUNC();

	if (p_buf != NULL) {
		LOGTP_DBUG("%s() p_buf[0] = %c\n", __func__, p_buf[0]);
		LOGTP_DBUG("%s() p_buf[1] = %c\n", __func__, p_buf[1]);
		LOGTP_DBUG("%s() p_buf[2] = %c\n", __func__, p_buf[2]);
		LOGTP_DBUG("%s() p_buf[3] = %c\n", __func__, p_buf[3]);
		LOGTP_DBUG("%s() p_buf[4] = %c\n", __func__, p_buf[4]);
		LOGTP_DBUG("%s() p_buf[5] = %c\n", __func__, p_buf[5]);

		LOGTP_DBUG("n_size = %d\n", n_size);

		i = 0;
		while ((p_chr = strsep((char **)&p_buf, " ,"))
	&& (i < MAX_DEBUG_REGISTER_NUM)) {
			LOGTP_DBUG("p_chr = %s\n", p_chr);

			_g_debug_reg[i] =
	common_char_tohexdigit(p_chr, strlen(p_chr));

			LOGTP_DBUG("_g_debug_reg[%d] = 0x%04X\n", i,
					_g_debug_reg[i]);
			i++;
		}
		_g_debug_reg_cnt = i;

		LOGTP_DBUG("_g_debug_reg_cnt = %d\n", _g_debug_reg_cnt);
	}

	return n_size;
}

static DEVICE_ATTR(debug, SYSFS_AUTHORITY, main_firmware_debug_show,
	main_firmware_debug_store);

/*--------------------------------------------------------------------------*/

ssize_t main_firmware_platform_version_show(struct device *p_device,
	struct device_attribute *p_attr,
	char *p_buf)
{
	LOGTP_DBUG("%s() _g_platform_fwversion = %s\n", __func__,
	_g_platform_fwversion);

	return sprintf(p_buf, "%s\n", _g_platform_fwversion);
}

ssize_t main_firmware_platform_version_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	fwic_getplatform_firmware_version(&_g_platform_fwversion);

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
	fwic_restore_firmwaremode_tologdata();
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

	LOGTP_DBUG("%s() _g_platform_fwversion = %s\n", __func__,
	_g_platform_fwversion);

	return n_size;
}

static DEVICE_ATTR(platform_version, SYSFS_AUTHORITY,
	main_firmware_platform_version_show,
	main_firmware_platform_version_store);
#endif

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
ssize_t main_firmware_modeshow(struct device *p_device,
				struct device_attribute *p_attr, char *p_buf)
{
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
	g_firmware_mode = fwic_getfirmware_mode();

	LOGTP_DBUG("%s() firmware mode = 0x%x\n", __func__, g_firmware_mode);

	return sprintf(p_buf, "%x", g_firmware_mode);
#elif (defined(CONFIG_ENABLE_CHIP_MSG21XXA) \
	|| defined(CONFIG_ENABLE_CHIP_MSG22XX))
	fwic_getfirmware_info(&g_firmware_info);
	g_firmware_mode = g_firmware_info.n_firmware_mode;

	LOGTP_DBUG("%s()firmware mode = 0x%x, can change firmware mode = %d\n",
		__func__, g_firmware_info.n_firmware_mode,
		g_firmware_info.n_iscan_change_firmwaremode);

	return sprintf(p_buf, "%x,%d", g_firmware_info.n_firmware_mode,
	g_firmware_info.n_iscan_change_firmwaremode);
#endif
}

ssize_t main_firmwaremode_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	LOGTP_FUNC();

	u32 n_mode;

	if (p_buf != NULL) {
		sscanf(p_buf, "%x", &n_mode);
		LOGTP_DBUG("firmware mode = 0x%x\n", n_mode);

		if (n_mode == FIRMWARE_MODE_DEMO_MODE)/* demo mode */
			g_firmware_mode =
	fwic_change_firmwaremode
	(FIRMWARE_MODE_DEMO_MODE);
		/* debug mode */
		 else if (n_mode == FIRMWARE_MODE_DEBUG_MODE)
			g_firmware_mode =
	fwic_change_firmwaremode
	(FIRMWARE_MODE_DEBUG_MODE);

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
		/* raw data mode */
		else if (n_mode == FIRMWARE_MODE_RAW_DATA_MODE)
			g_firmware_mode =
	fwic_change_firmwaremode
	(FIRMWARE_MODE_RAW_DATA_MODE);

#endif
		else
			LOGTP_DBUG("Undefined Firmware Mode\n");

	}

	LOGTP_DBUG("g_firmware_mode = 0x%x\n", g_firmware_mode);

	return n_size;
}

static DEVICE_ATTR(mode, SYSFS_AUTHORITY, main_firmware_modeshow,
	main_firmwaremode_store);

ssize_t main_firmware_sensor_show(struct device *p_device,
	struct device_attribute *p_attr, char *p_buf)
{
	LOGTP_FUNC();

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
	if (g_firmware_info.n_logmode_packet_header == 0xA5
	|| g_firmware_info.n_logmode_packet_header == 0xAB) {
		return sprintf(p_buf, "%d,%d", g_firmware_info.n_mx,
	g_firmware_info.n_my);
	} else if (g_firmware_info.n_logmode_packet_header == 0xA7) {
		return sprintf(p_buf, "%d,%d,%d,%d", g_firmware_info.n_mx,
	g_firmware_info.n_my, g_firmware_info.n_ss,
	g_firmware_info.n_sd);
	} else {
		LOGTP_DBUG("Undefined debug mode packet format : 0x%x\n",
				g_firmware_info.n_logmode_packet_header);
		return 0;
	}
#elif (defined(CONFIG_ENABLE_CHIP_MSG21XXA) \
	|| defined(CONFIG_ENABLE_CHIP_MSG22XX))
	return sprintf(p_buf, "%d", g_firmware_info.n_logmode_packet_len);
#endif
}

ssize_t main_firmware_sensor_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	LOGTP_FUNC();
/*
    fwic_getfirmware_info(&g_firmware_info);
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    g_firmware_mode = g_firmware_info.n_firmware_mode;
#endif
*/
	return n_size;
}

static DEVICE_ATTR(sensor, SYSFS_AUTHORITY, main_firmware_sensor_show,
	main_firmware_sensor_store);

ssize_t main_firmware_headershow(struct device *p_device,
	struct device_attribute *p_attr, char *p_buf)
{
	LOGTP_FUNC();

	return sprintf(p_buf, "%d", g_firmware_info.n_logmode_packet_header);
}

ssize_t main_firmware_header_store(struct device *p_device,
	struct device_attribute *p_attr,
	const char *p_buf, size_t n_size)
{
	LOGTP_FUNC();
/*
    fwic_getfirmware_info(&g_firmware_info);
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    g_firmware_mode = g_firmware_info.n_firmware_mode;
#endif
*/
	return n_size;
}

static DEVICE_ATTR(header, SYSFS_AUTHORITY, main_firmware_headershow,
	main_firmware_header_store);

/* ----------------------------------------
--------------------------------------// */

ssize_t main_kobject_packet_show(struct kobject *p_kobj,
	struct kobj_attribute *p_attr, char *p_buf)
{
	u32 i = 0;
	u32 n_len = 0;

	LOGTP_FUNC();

	if (strcmp(p_attr->attr.name, "packet") == 0) {
		if (g_logmode_packet != NULL) {
			LOGTP_DBUG("FMode=%x, LMPacket[0]=%x, LMPacket[1]=%x\n",
			g_firmware_mode, g_logmode_packet[0],
			g_logmode_packet[1]);
			LOGTP_DBUG("g_logmode_packet[2]=%x, packet[3]=%x\n",
				g_logmode_packet[2], g_logmode_packet[3]);
			LOGTP_DBUG("g_logmode_packet[4]=%x, packet[5]=%x\n",
				g_logmode_packet[4], g_logmode_packet[5]);

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
			if ((g_firmware_mode == FIRMWARE_MODE_DEBUG_MODE)
	&& (g_logmode_packet[0] == 0xA5
				|| g_logmode_packet[0] == 0xAB
				|| g_logmode_packet[0] == 0xA7))
#elif (defined(CONFIG_ENABLE_CHIP_MSG21XXA) \
	|| defined(CONFIG_ENABLE_CHIP_MSG22XX))

			if ((g_firmware_mode == FIRMWARE_MODE_DEBUG_MODE ||
				g_firmware_mode == FIRMWARE_MODE_RAW_DATA_MODE)
				&& (g_logmode_packet[0] == 0x62))
#endif
				{
				for (i = 0;
				i < g_firmware_info.n_logmode_packet_len;
				i++) {
					p_buf[i] = g_logmode_packet[i];
				}

				n_len = g_firmware_info.n_logmode_packet_len;
				LOGTP_DBUG("n_len = %d\n", n_len);
			} else {
				LOGTP_DBUG("Current is mot DEBUG mode\n");
			}
		} else {
			LOGTP_DBUG("g_logmode_packet is NULL\n");
		}
	} else {
		LOGTP_DBUG("p_attr->attr.name=%s\n", p_attr->attr.name);
	}

	return n_len;
}

ssize_t main_kobj_packet_store(struct kobject *p_kobj,
	struct kobj_attribute *p_attr,
	const char *p_buf, size_t n_count)
{
	LOGTP_FUNC();
/*
    if (strcmp(p_attr->attr.name, "packet") == 0)
    {

    }
*/
	return n_count;
}

static struct kobj_attribute packet_attr =
__ATTR(packet, 0666, main_kobject_packet_show, main_kobj_packet_store);

/* Create a group of attributes so that we can create and destroy them all at
 once. */
static struct attribute *attrs[] = {
	&packet_attr.attr,
	NULL,/* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory. If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
struct attribute_group attr_group = {
	.attrs = attrs,
};
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

/* ----------------------------------------
--------------------------------------// */

s32 main_device_initialize(void)
{
	s32 n_retval = 0;
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
	u8 *p_device_path = NULL;
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

	LOGTP_FUNC();

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
	/* set sysfs for firmware */
	_g_firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
	if (IS_ERR(_g_firmware_class))
		LOGTP_ERRO("Failed to create class(firmware)!\n");

	_g_firmware_cmd_dev =
	device_create(_g_firmware_class, NULL, 0, NULL, "device");
	if (IS_ERR(_g_firmware_cmd_dev))
		LOGTP_ERRO("Failed to create device(_g_firmware_cmd_dev)!\n");

	/* version */
	if (device_create_file(_g_firmware_cmd_dev, &dev_attr_version) < 0)
		LOGTP_ERRO("Failed to create device file(%s)!\n",
				dev_attr_version.attr.name);
	/* update */
	if (device_create_file(_g_firmware_cmd_dev, &dev_attr_update) < 0)
		LOGTP_ERRO("Failed to create device file(%s)!\n",
				dev_attr_update.attr.name);
	/* data */
	if (device_create_file(_g_firmware_cmd_dev, &dev_attr_data) < 0)
		LOGTP_ERRO("Failed to create device file(%s)!\n",
				dev_attr_data.attr.name);
#ifdef CONFIG_ENABLE_ITO_MP_TEST
	/* test */
	if (device_create_file(_g_firmware_cmd_dev, &dev_attr_test) < 0)
		LOGTP_ERRO("Failed to create device file(%s)!\n",
	dev_attr_test.attr.name);
	/* test_log */
	if (device_create_file(_g_firmware_cmd_dev, &dev_attr_test_log) < 0)
		LOGTP_ERRO("Failed to create device file(%s)!\n",
				dev_attr_test_log.attr.name);
	/* test_fail_channel */
	if (device_create_file(_g_firmware_cmd_dev,
		&dev_attr_test_fail_channel) < 0)
		LOGTP_ERRO("Failed to create device file(%s)!\n",
			dev_attr_test_fail_channel.attr.name);
#endif/* CONFIG_ENABLE_ITO_MP_TEST */

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
	/* mode */
	if (device_create_file(_g_firmware_cmd_dev, &dev_attr_mode) < 0)
		LOGTP_DBUG("Failed to create device file(%s)!\n",
	dev_attr_mode.attr.name);
	/* packet */
/*    if (device_create_file(_g_firmware_cmd_dev, &dev_attr_packet) < 0) */
/*        LOGTP_DBUG("Failed to create device file(%s)!\n",
			dev_attr_packet.attr.name); */
	/* sensor */
	if (device_create_file(_g_firmware_cmd_dev, &dev_attr_sensor) < 0)
		LOGTP_DBUG("Failed to create device file(%s)!\n",
				dev_attr_sensor.attr.name);
	/* header */
	if (device_create_file(_g_firmware_cmd_dev, &dev_attr_header) < 0)
		LOGTP_DBUG("Failed to create device file(%s)!\n",
				dev_attr_header.attr.name);
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

	/* debug */
	if (device_create_file(_g_firmware_cmd_dev, &dev_attr_debug) < 0)
		LOGTP_DBUG"Failed to create device file(%s)!\n",
				dev_attr_debug.attr.name);
	/* platform version */
	if (device_create_file(_g_firmware_cmd_dev,
		&dev_attr_platform_version) < 0)
		LOGTP_DBUG("Failed to create device file(%s)!\n",
				dev_attr_platform_version.attr.name);
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	/* gesture wakeup mode */
	if (device_create_file(_g_firmware_cmd_dev,
		&dev_attr_gesture_wakeup_mode) < 0)
		LOGTP_DBUG("Failed to create device file(%s)!\n",
				dev_attr_gesture_wakeup_mode.attr.name);
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */
	/* chip type */
	if (device_create_file(_g_firmware_cmd_dev, &dev_attr_chip_type) < 0)
		LOGTP_DBUG("Failed to create device file(%s)!\n",
				dev_attr_chip_type.attr.name);
	/* driver version */
	if (device_create_file(_g_firmware_cmd_dev,
			&dev_attr_driver_version) < 0)
		LOGTP_DBUG("Failed to create device file(%s)!\n",
				dev_attr_driver_version.attr.name);

	dev_set_drvdata(_g_firmware_cmd_dev, NULL);
#endif/*CONFIG_UPDATE_FIRMWARE_BY_SW_ID */

#ifdef CONFIG_ENABLE_ITO_MP_TEST
	fwic_create_workqueue();
#endif/* CONFIG_ENABLE_ITO_MP_TEST */

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
/* create a kset with the name of "kset_example" which is located under
 /sys/kernel/ */
	g_touch_kset = kset_create_and_add("kset_example", NULL, kernel_kobj);
	if (!g_touch_kset) {
		LOGTP_DBUG("kset_create_and_add() failed, n_retval = %d\n",
				n_retval);

		n_retval = -ENOMEM;
	}

	g_touch_kobj = kobject_create();
	if (!g_touch_kobj) {
		LOGTP_DBUG("kobject_create() failed, retval = %d\n", n_retval);

		n_retval = -ENOMEM;
		kset_unregister(g_touch_kset);
		g_touch_kset = NULL;
	}

	g_touch_kobj->kset = g_touch_kset;

	n_retval = kobject_add(g_touch_kobj, NULL, "%s", "kobject_example");
	if (n_retval != 0) {
		LOGTP_DBUG("kobject_add() failed, n_retval = %d\n", n_retval);

		kobject_put(g_touch_kobj);
		g_touch_kobj = NULL;
		kset_unregister(g_touch_kset);
		g_touch_kset = NULL;
	}

	/* create the files associated with this kobject */
	n_retval = sysfs_create_group(g_touch_kobj, &attr_group);
	if (n_retval != 0) {
		LOGTP_DBUG("sysfs_create_file() failed, n_retval = %d\n",
				n_retval);

		kobject_put(g_touch_kobj);
		g_touch_kobj = NULL;
		kset_unregister(g_touch_kset);
		g_touch_kset = NULL;
	}

	p_device_path = kobject_get_path(g_touch_kobj, GFP_KERNEL);
	LOGTP_DBUG("DEVPATH = %s\n", p_device_path);
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

	g_chiptype = fwic_get_chiptype();
	if (g_chiptype != 2) {
		LOGTP_ERRO("[Mstar] the chip id error %x\n", g_chiptype);
		return -ENODEV;
	}
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
	/* get firmware mode for parsing packet judgement. */
	g_firmware_mode = fwic_getfirmware_mode();
#endif/* CONFIG_ENABLE_CHIP_MSG26XXM */

	memset(&g_firmware_info, 0x0, sizeof(struct firmware_info_t));

	fwic_getfirmware_info(&g_firmware_info);

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
	g_firmware_mode = g_firmware_info.n_firmware_mode;
#endif
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
	fwic_checkfirmware_byswid();
#endif/* CONFIG_UPDATE_FIRMWARE_BY_SW_ID */

	return n_retval;
}

/* ----------------------------------------
--------------------------------------// */
