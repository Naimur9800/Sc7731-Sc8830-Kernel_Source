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
 * rights to any and all damages, losses, costs and expenses resulting
 therefrom.
 *
==============================================================================*/

/**
 *
 * @file    mstar_drv_self_mp_test.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

/*=============================================================*/
/* INCLUDE FILE */
/*=============================================================*/

#include "mstar_drv_self_mp_test.h"
#include "mstar_drv_utility_adaption.h"
#include "mstar_drv_self_fw_control.h"
#include "mstar_drv_platform_porting_layer.h"

#ifdef CONFIG_ENABLE_ITO_MP_TEST

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
/* Modify. */
#include "open_test_ANA1_X.h"
#include "open_test_ANA2_X.h"
#include "open_test_ANA1_B_X.h"
#include "open_test_ANA2_B_X.h"
#include "open_test_ANA3_X.h"

#include "open_test_ANA1_Y.h"
#include "open_test_ANA2_Y.h"
#include "open_test_ANA1_B_Y.h"
#include "open_test_ANA2_B_Y.h"
#include "open_test_ANA3_Y.h"

/* Modify. */
#include "short_test_ANA1_X.h"
#include "short_test_ANA2_X.h"
#include "short_test_ANA3_X.h"
#include "short_test_ANA4_X.h"

#include "short_test_ANA1_Y.h"
#include "short_test_ANA2_Y.h"
#include "short_test_ANA3_Y.h"
#include "short_test_ANA4_Y.h"
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
/* Modify. */
#include "open_test_RIU1_X.h"
#include "open_test_RIU2_X.h"
#include "open_test_RIU3_X.h"

#include "open_test_RIU1_Y.h"
#include "open_test_RIU2_Y.h"
#include "open_test_RIU3_Y.h"

/* Modify. */
#include "short_test_RIU1_X.h"
#include "short_test_RIU2_X.h"
#include "short_test_RIU3_X.h"
#include "short_test_RIU4_X.h"

#include "short_test_RIU1_Y.h"
#include "short_test_RIU2_Y.h"
#include "short_test_RIU3_Y.h"
#include "short_test_RIU4_Y.h"
#endif

/*=============================================================*/
/* PREPROCESSOR CONSTANT DEFINITION */
/*=============================================================*/

/* Modify. */
#define TP_OF_X    (1)/* (2) */
#define TP_OF_Y    (4)


/*=============================================================*/
/* LOCAL VARIABLE DEFINITION */
/*=============================================================*/

static u32 _g_is_in_mptest; /* = 0; */
static u32 _gtest_rery_cnt = CTP_MP_TEST_RETRY_COUNT;
static enum ito_testmode_e _g_ito_testmode; /* = 0; */

static s32 _gctp_mptest_status = ITO_TEST_UNDER_TESTING;
static u8 _g_testfail_channel[MAX_CHANNEL] = { 0 };

static u32 _g_testfail_channel_cnt; /* = 0; */

static struct work_struct _g__ito_test_work;
static struct workqueue_struct *_g_ctp_mptest_workqueue; /* = NULL; */

static s16 _g_rawdata1[MAX_CHANNEL] = { 0 };
static s16 _g_rawdata2[MAX_CHANNEL] = { 0 };
static s16 _g_rawdata3[MAX_CHANNEL] = { 0 };
static s16 _g_rawdata4[MAX_CHANNEL] = { 0 };
static s8 _g_dataflag1[MAX_CHANNEL] = { 0 };
static s8 _g_dataflag2[MAX_CHANNEL] = { 0 };
static s8 _g_dataflag3[MAX_CHANNEL] = { 0 };
static s8 _g_dataflag4[MAX_CHANNEL] = { 0 };

static u8 _g_ito_test_keynum; /* = 0; */
static u8 _g_ito_dummy_num; /* = 0; */
static u8 _g_ito_triangle_num; /* = 0; */
static u8 _g_is_enable2r; /* = 0; */

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)

static u8 _g_ltp = 1;

/* _g_open1~_gopen3 are for MSG21XXA */
static u16 *_g_open1; /* = NULL; */
static u16 *_g_open1b; /* = NULL; */
static u16 *_g_open2; /* = NULL; */
static u16 *_g_open2b; /* = NULL; */
static u16 *_gopen3; /* = NULL; */

/* _g_short_1~_g_short_4 are for MSG21XXA */
static u16 *_g_short_1; /* = NULL; */
static u16 *_g_short_2; /* = NULL; */
static u16 *_g_short_3; /* = NULL; */
static u16 *_g_short_4; /* = NULL; */

/* _g_short_1_gpo~_g_short_4_gpo are for MSG21XXA */
static u16 *_g_short_1_gpo; /* = NULL; */
static u16 *_g_short_2_gpo; /* = NULL; */
static u16 *_g_short_3_gpo; /* = NULL; */
static u16 *_g_short_4_gpo; /* = NULL; */

#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)

/* _g_open_riu1~_g_open_riu3 are for MSG22XX */
static u32 *_g_open_riu1; /* = NULL; */
static u32 *_g_open_riu2; /* = NULL; */
static u32 *_g_open_riu3; /* = NULL; */

/* _gshort_riu1~_gshort_riu4 are for MSG22XX */
static u32 *_gshort_riu1; /* = NULL; */
static u32 *_gshort_riu2; /* = NULL; */
static u32 *_gshort_riu3; /* = NULL; */
static u32 *_gshort_riu4; /* = NULL; */

/* _gopen_sub_framenum1~_gshortsub_framenum4 are for MSG22XX */
static u8 _gopen_sub_framenum1; /* = 0; */
static u8 _gopen_sub_framenum2; /* = 0; */
static u8 _gopen_sub_framenum3; /* = 0; */
static u8 _g_shortsub_num; /* = 0; */
static u8 _gshortsub_framenum2; /* = 0; */
static u8 _gshortsub_framenum3; /* = 0; */
static u8 _gshortsub_framenum4; /* = 0; */

#endif

static u8 *_gmap1; /* = NULL; */
static u8 *_gmap2; /* = NULL; */
static u8 *_gmap3; /* = NULL; */
static u8 *_gmap40_1; /* = NULL; */
static u8 *_gmap40_2; /* = NULL; */
static u8 *_gmap40_3; /* = NULL; */
static u8 *_gmap40_4; /* = NULL; */
static u8 *_gmap41_1; /* = NULL; */
static u8 *_gmap41_2; /* = NULL; */
static u8 *_gmap41_3; /* = NULL; */
static u8 *_gmap41_4; /* = NULL; */

static u8 *_gshort_map1; /* = NULL; */
static u8 *_gshort_map2; /* = NULL; */
static u8 *_gshort_map3; /* = NULL; */
static u8 *_gshort_map4; /* = NULL; */

/* static struct proc_dir_entry *_gMsgItoTest; /* = NULL; */ */
/* static struct proc_dir_entry *_gDebug = NULL;  */
enum ito_testresult_e _g_ito_result = ITO_TEST_OK;

/*=============================================================*/
/* EXTERN FUNCTION DECLARATION */
/*=============================================================*/

/*=============================================================*/
/* LOCAL FUNCTION DEFINITION */
/*=============================================================*/

static u16 _mptest_ito_get_tptype(void)
{
	u16 n_major = 0, n_minor = 0;

	LOGTP_FUNC();

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
	{
		u8 sz_dbbus_txdata[3] = { 0 };
		u8 sz_dbbus_rxdata[4] = { 0 };

		sz_dbbus_txdata[0] = 0x53;
		sz_dbbus_txdata[1] = 0x00;
		sz_dbbus_txdata[2] = 0x2A;

		i2c_write_data(SLAVE_I2C_ID_DWI2C, &sz_dbbus_txdata[0], 3);
		i2c_read_data(SLAVE_I2C_ID_DWI2C, &sz_dbbus_rxdata[0], 4);

		n_major = (sz_dbbus_rxdata[1] << 8) + sz_dbbus_rxdata[0];
		n_minor = (sz_dbbus_rxdata[3] << 8) + sz_dbbus_rxdata[2];
	}
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
	{
		u16 n_reg_data1, n_reg_data2;

		platform_device_reset_hw();

		bus_enter_serial_debugmode();
		bus_stop_mcu();
		bus_i2c_use_bus();
		bus_i2c_reshape();
		mdelay(100);

		/* Stop mcu */
		reg_set_lowbyte(0x0FE6, 0x01);

		/* Stop watchdog */
		reg_set16bit_value(0x3C60, 0xAA55);

		/* RIU password */
		reg_set16bit_value(0x161A, 0xABBA);

		/* Clear pce */
		reg_set16bit_value(0x1618, (reg_get16bit_value(0x1618) | 0x80));

reg_set16bit_value(0x1600, 0xBFF4);

		/* Enable burst mode */
/*        reg_set16bit_value(0x160C, (reg_get16bit_value(0x160C) | 0x01)); */

		/* Set pce */
		reg_set16bit_value(0x1618, (reg_get16bit_value(0x1618) | 0x40));

		reg_set_lowbyte(0x160E, 0x01);

		n_reg_data1 = reg_get16bit_value(0x1604);
		n_reg_data2 = reg_get16bit_value(0x1606);

		n_major = (((n_reg_data1 >> 8) & 0xFF) << 8) +
				(n_reg_data1 & 0xFF);
		n_minor = (((n_reg_data2 >> 8) & 0xFF) << 8) +
				(n_reg_data2 & 0xFF);

		/* Clear burst mode */
/*        reg_set16bit_value(0x160C, reg_get16bit_value(0x160C) & (~0x01)); */

		reg_set16bit_value(0x1600, 0x0000);

		/* Clear RIU password */
		reg_set16bit_value(0x161A, 0x0000);

		bus_i2c_notuse_buf();
		buf_not_stop_mcu();
		bus_exit_serial_debugmode();

		platform_device_reset_hw();
		mdelay(100);
	}
#endif

	LOGTP_DBUG("*** major = %d ***\n", n_major);
	LOGTP_DBUG("*** minor = %d ***\n", n_minor);

	return n_major;
}

static u16 _mptest_ito_choose_tp_type(void)
{
	u16 n_tp_type = 0;
	u32 i = 0;

	LOGTP_FUNC();

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
	/* _g_open1~_gopen3 are for MSG21XXA */
	_g_open1 = NULL;
	_g_open1b = NULL;
	_g_open2 = NULL;
	_g_open2b = NULL;
	_gopen3 = NULL;

	/* _g_short_1~_g_short_4 are for MSG21XXA */
	_g_short_1 = NULL;
	_g_short_2 = NULL;
	_g_short_3 = NULL;
	_g_short_4 = NULL;

	_g_short_1_gpo = NULL;
	_g_short_2_gpo = NULL;
	_g_short_3_gpo = NULL;
	_g_short_4_gpo = NULL;
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
	/* _g_open_riu1~_g_open_riu3 are for MSG22XX */
	_g_open_riu1 = NULL;
	_g_open_riu2 = NULL;
	_g_open_riu3 = NULL;

	/* _gshort_riu1~_gshort_riu4 are for MSG22XX */
	_gshort_riu1 = NULL;
	_gshort_riu2 = NULL;
	_gshort_riu3 = NULL;
	_gshort_riu4 = NULL;

	_gopen_sub_framenum1 = 0;
	_gopen_sub_framenum2 = 0;
	_gopen_sub_framenum3 = 0;
	_g_shortsub_num = 0;
	_gshortsub_framenum2 = 0;
	_gshortsub_framenum3 = 0;
	_gshortsub_framenum4 = 0;
#endif

	_gmap1 = NULL;
	_gmap2 = NULL;
	_gmap3 = NULL;
	_gmap40_1 = NULL;
	_gmap40_2 = NULL;
	_gmap40_3 = NULL;
	_gmap40_4 = NULL;
	_gmap41_1 = NULL;
	_gmap41_2 = NULL;
	_gmap41_3 = NULL;
	_gmap41_4 = NULL;

	_gshort_map1 = NULL;
	_gshort_map2 = NULL;
	_gshort_map3 = NULL;
	_gshort_map4 = NULL;

	_g_ito_test_keynum = 0;
	_g_ito_dummy_num = 0;
	_g_ito_triangle_num = 0;
	_g_is_enable2r = 0;

	for (i = 0; i < 10; i++) {
		n_tp_type = _mptest_ito_get_tptype();
		LOGTP_DBUG("n_tp_type = %d, i = %d\n", n_tp_type, i);

		if (TP_OF_X == n_tp_type || TP_OF_Y == n_tp_type) {/* Modify. */
			break;
		} else if (i < 5) {
			mdelay(100);
		} else {
			platform_device_reset_hw();
		}
	}

		if (TP_OF_X == n_tp_type) {/* Modify. */
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
		_g_open1 = open_1_X;
		_g_open1b = open_1B_X;
		_g_open2 = open_2_X;
		_g_open2b = open_2B_X;
		_gopen3 = open_3_X;

		_g_short_1 = short_1_X;
		_g_short_2 = short_2_X;
		_g_short_3 = short_3_X;
		_g_short_4 = short_4_X;

		_g_short_1_gpo = short_1_X_GPO;
		_g_short_2_gpo = short_2_X_GPO;
		_g_short_3_gpo = short_3_X_GPO;
		_g_short_4_gpo = short_4_X_GPO;
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
		_g_open_riu1 = open_1_X;
		_g_open_riu2 = open_2_X;
		_g_open_riu3 = open_3_X;

		_gshort_riu1 = short_1_X;
		_gshort_riu2 = short_2_X;
		_gshort_riu3 = short_3_X;
		_gshort_riu4 = short_4_X;

		_gopen_sub_framenum1 = NUM_OPEN_1_SENSOR_X;
		_gopen_sub_framenum2 = NUM_OPEN_2_SENSOR_X;
		_gopen_sub_framenum3 = NUM_OPEN_3_SENSOR_X;
		_g_shortsub_num = NUM_SHORT_1_SENSOR_X;
		_gshortsub_framenum2 = NUM_SHORT_2_SENSOR_X;
		_gshortsub_framenum3 = NUM_SHORT_3_SENSOR_X;
		_gshortsub_framenum4 = NUM_SHORT_4_SENSOR_X;
#endif

		_gmap1 = MAP1_X;
		_gmap2 = MAP2_X;
		_gmap3 = MAP3_X;
		_gmap40_1 = MAP40_1_X;
		_gmap40_2 = MAP40_2_X;
		_gmap40_3 = MAP40_3_X;
		_gmap40_4 = MAP40_4_X;
		_gmap41_1 = MAP41_1_X;
		_gmap41_2 = MAP41_2_X;
		_gmap41_3 = MAP41_3_X;
		_gmap41_4 = MAP41_4_X;

		_gshort_map1 = SHORT_MAP1_X;
		_gshort_map2 = SHORT_MAP2_X;
		_gshort_map3 = SHORT_MAP3_X;
		_gshort_map4 = SHORT_MAP4_X;

		_g_ito_test_keynum = NUM_KEY_X;
		_g_ito_dummy_num = NUM_DUMMY_X;
		_g_ito_triangle_num = NUM_SENSOR_X;
		_g_is_enable2r = ENABLE_2R_X;
		} else if (TP_OF_Y == n_tp_type) {/* Modify. */
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
		_g_open1 = open_1_Y;
		_g_open1b = open_1B_Y;
		_g_open2 = open_2_Y;
		_g_open2b = open_2B_Y;
		_gopen3 = open_3_Y;

		_g_short_1 = short_1_Y;
		_g_short_2 = short_2_Y;
		_g_short_3 = short_3_Y;
		_g_short_4 = short_4_Y;

		_g_short_1_gpo = short_1_Y_GPO;
		_g_short_2_gpo = short_2_Y_GPO;
		_g_short_3_gpo = short_3_Y_GPO;
		_g_short_4_gpo = short_4_Y_GPO;
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
		_g_open_riu1 = open_1_Y;
		_g_open_riu2 = open_2_Y;
		_g_open_riu3 = open_3_Y;

		_gshort_riu1 = short_1_Y;
		_gshort_riu2 = short_2_Y;
		_gshort_riu3 = short_3_Y;
		_gshort_riu4 = short_4_Y;

		_gopen_sub_framenum1 = NUM_OPEN_1_SENSOR_Y;
		_gopen_sub_framenum2 = NUM_OPEN_2_SENSOR_Y;
		_gopen_sub_framenum3 = NUM_OPEN_3_SENSOR_Y;
		_g_shortsub_num = NUM_SHORT_1_SENSOR_Y;
		_gshortsub_framenum2 = NUM_SHORT_2_SENSOR_Y;
		_gshortsub_framenum3 = NUM_SHORT_3_SENSOR_Y;
		_gshortsub_framenum4 = NUM_SHORT_4_SENSOR_Y;
#endif

		_gmap1 = MAP1_Y;
		_gmap2 = MAP2_Y;
		_gmap3 = MAP3_Y;
		_gmap40_1 = MAP40_1_Y;
		_gmap40_2 = MAP40_2_Y;
		_gmap40_3 = MAP40_3_Y;
		_gmap40_4 = MAP40_4_Y;
		_gmap41_1 = MAP41_1_Y;
		_gmap41_2 = MAP41_2_Y;
		_gmap41_3 = MAP41_3_Y;
		_gmap41_4 = MAP41_4_Y;

		_gshort_map1 = SHORT_MAP1_Y;
		_gshort_map2 = SHORT_MAP2_Y;
		_gshort_map3 = SHORT_MAP3_Y;
		_gshort_map4 = SHORT_MAP4_Y;

		_g_ito_test_keynum = NUM_KEY_Y;
		_g_ito_dummy_num = NUM_DUMMY_Y;
		_g_ito_triangle_num = NUM_SENSOR_Y;
		_g_is_enable2r = ENABLE_2R_Y;
	} else {
		n_tp_type = 0;
	}

	return n_tp_type;
}

static void _mptest_ito_swreset(void)
{
	LOGTP_FUNC();

	reg_set16bit_value(0x1100, 0xFFFF);/* bank:ana, addr:h0000 */
	reg_set16bit_value(0x1100, 0x0000);
	mdelay(50);/* mdelay(15) */
}

/*----------------------------------------------------------------------------*/

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)

static u16 _mptest_ito_getnum(void)
{
	u32 i;
	u16 n_sensornum = 0;
	u16 n_regval1, n_regval2;

	LOGTP_FUNC();

	n_regval1 = reg_get16bit_value(0x114A);/* bank:ana, addr:h0025 */
	LOGTP_DBUG("n_regvalue1 = %d\n", n_regval1);

	if ((n_regval1 & BIT1) == BIT1) {
		/* bank:ana2, addr:h0005 */
		n_regval1 = reg_get16bit_value(0x120A);
		n_regval1 = n_regval1 & 0x0F;
		/* bank:ana2, addr:h000b */
		n_regval2 = reg_get16bit_value(0x1216);
		n_regval2 = ((n_regval2 >> 1) & 0x0F) + 1;

		n_sensornum = n_regval1 * n_regval2;
	} else {
		for (i = 0; i < 4; i++) {
			n_sensornum += (reg_get16bit_value(0x120A) >>
			(4 * i)) & 0x0F;
		}
	}
	LOGTP_DBUG("n_sensornum = %d\n", n_sensornum);

	return n_sensornum;
}

static void _mptest_disable_filternoise_detect(void)
{
	u16 n_regvalue;

	LOGTP_FUNC();

	/* Disable DIG/ANA drop */
	n_regvalue = reg_get16bit_value(0x1302);

	reg_set16bit_value(0x1302, n_regvalue & (~(BIT2 | BIT1 | BIT0)));
}

static void _mptest_ito_testpolling(void)
{
	u16 n_regint = 0x0000;
	u16 n_regval;

	LOGTP_FUNC();

	reg_set16bit_value(0x130C, BIT15);/* bank:fir, addr:h0006 */
	/* bank:ana2, addr:h000a */
	reg_set16bit_value(0x1214, (reg_get16bit_value(0x1214) | BIT0));

	LOGTP_DBUG("polling start\n");

	do {
		/* bank:intr_ctrl, addr:h000c */
		n_regint = reg_get16bit_value(0x3D18);
	} while ((n_regint & FIQ_E_FRAME_READY_MASK) == 0x0000);

	LOGTP_DBUG("polling end\n");

	n_regval = reg_get16bit_value(0x3D18);
	reg_set16bit_value(0x3D18, n_regval & (~FIQ_E_FRAME_READY_MASK));
}

static void _mptest_ito_opentest_setv(u8 n_enable, u8 n_prs)
{
	u16 n_regval;

	LOGTP_DBUG("*** %s,n_enable=%d, n_prs=%d ***\n",
			__func__, n_enable, n_prs);

	n_regval = reg_get16bit_value(0x1208);/* bank:ana2, addr:h0004 */
	n_regval = n_regval & 0xF1;

	if (n_prs == 0)
		reg_set16bit_value(0x1208, n_regval | 0x0C);
	else if (n_prs == 1)
		reg_set16bit_value(0x1208, n_regval | 0x0E);
	else
		reg_set16bit_value(0x1208, n_regval | 0x02);


	if (n_enable) {
		n_regval = reg_get16bit_value(0x1106);/* bank:ana, addr:h0003 */
		reg_set16bit_value(0x1106, n_regval | 0x03);
	} else {
		n_regval = reg_get16bit_value(0x1106);
		n_regval = n_regval & 0xFC;
		reg_set16bit_value(0x1106, n_regval);
	}
}

static u16 _mptest_ito_msg21xxa_getdataout(s16 *p_rawd_ata)
{
	u32 i;
	u16 sz_rawdata[MAX_CHANNEL] = { 0 };
	u16 n_sensornum;
	u16 n_regint;
	u8 sz_dbbus_txdata[8] = { 0 };
	u8 sz_dbbus_rxdata[MAX_CHANNEL * 2] = { 0 };

	LOGTP_FUNC();

	n_sensornum = _mptest_ito_getnum();

	if ((n_sensornum * 2) > (MAX_CHANNEL * 2)) {
		LOGTP_DBUG("Danger. n_sensornum = %d\n", n_sensornum);
		return n_sensornum;
	}

	n_regint = reg_get16bit_value((0x3d << 8) | (REG_INTR_FIQ_MASK << 1));
	reg_set16bit_value((0x3d << 8) | (REG_INTR_FIQ_MASK << 1),
	(n_regint & (u16) (~FIQ_E_FRAME_READY_MASK)));

	_mptest_ito_testpolling();

	sz_dbbus_txdata[0] = 0x10;
	sz_dbbus_txdata[1] = 0x13;/* bank:fir, addr:h0020 */
	sz_dbbus_txdata[2] = 0x40;
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 3);
	mdelay(20);
	i2c_read_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_rxdata[0],
			(n_sensornum * 2));
	mdelay(100);

	for (i = 0; i < n_sensornum * 2; i++)
		LOGTP_DBUG("sz_dbbus_rxdata[%d] = %d\n", i, sz_dbbus_rxdata[i]);


	n_regint = reg_get16bit_value((0x3d << 8) | (REG_INTR_FIQ_MASK << 1));
	reg_set16bit_value((0x3d << 8) | (REG_INTR_FIQ_MASK << 1),
	(n_regint | (u16) FIQ_E_FRAME_READY_MASK));

	for (i = 0; i < n_sensornum; i++) {
		sz_rawdata[i] =
	(sz_dbbus_rxdata[2 * i + 1] << 8) | (sz_dbbus_rxdata[2 * i]);
		p_rawd_ata[i] = (s16) sz_rawdata[i];
	}

	return n_sensornum;
}

static void _mptest_ito_msg21xxa_getdatain(u8 n_step)
{
	u32 i;
	u16 *p_type = NULL;
	u8 sz_dbbus_txdata[512] = { 0 };

	LOGTP_DBUG("*** %s() n_step = %d ***\n", __func__, n_step);

		if (n_step == 0) {/* 39-4 (2R) */
		p_type = &_g_short_4[0];
		} else if (n_step == 1) {/* 39-1 */
		p_type = &_g_short_1[0];
		} else if (n_step == 2) {/* 39-2 */
		p_type = &_g_short_2[0];
		} else if (n_step == 3) {/* 39-3 */
		p_type = &_g_short_3[0];
	} else if (n_step == 4) {
		p_type = &_g_open1[0];
	} else if (n_step == 5) {
		p_type = &_g_open2[0];
	} else if (n_step == 6) {
		p_type = &_gopen3[0];
	} else if (n_step == 9) {
		p_type = &_g_open1b[0];
	} else if (n_step == 10) {
		p_type = &_g_open2b[0];
	}

	sz_dbbus_txdata[0] = 0x10;
	sz_dbbus_txdata[1] = 0x11;/* bank:ana, addr:h0000 */
	sz_dbbus_txdata[2] = 0x00;
	for (i = 0; i <= 0x3E; i++) {
		sz_dbbus_txdata[3 + 2 * i] = p_type[i] & 0xFF;
		sz_dbbus_txdata[4 + 2 * i] = (p_type[i] >> 8) & 0xFF;
	}
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 3 + 0x3F * 2);

	sz_dbbus_txdata[2] = 0x7A * 2;/* bank:ana, addr:h007a */
	for (i = 0x7A; i <= 0x7D; i++) {
		sz_dbbus_txdata[3 + 2 * (i - 0x7A)] = 0;
		sz_dbbus_txdata[4 + 2 * (i - 0x7A)] = 0;
	}
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 3 + 8);

	sz_dbbus_txdata[0] = 0x10;
	sz_dbbus_txdata[1] = 0x12;/* bank:ana2, addr:h0005 */

	sz_dbbus_txdata[2] = 0x05 * 2;
	sz_dbbus_txdata[3] = p_type[128 + 0x05] & 0xFF;
	sz_dbbus_txdata[4] = (p_type[128 + 0x05] >> 8) & 0xFF;
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 5);

	sz_dbbus_txdata[2] = 0x0B * 2;/* bank:ana2, addr:h000b */
	sz_dbbus_txdata[3] = p_type[128 + 0x0B] & 0xFF;
	sz_dbbus_txdata[4] = (p_type[128 + 0x0B] >> 8) & 0xFF;
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 5);

	sz_dbbus_txdata[2] = 0x12 * 2;/* bank:ana2, addr:h0012 */
	sz_dbbus_txdata[3] = p_type[128 + 0x12] & 0xFF;
	sz_dbbus_txdata[4] = (p_type[128 + 0x12] >> 8) & 0xFF;
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 5);

	sz_dbbus_txdata[2] = 0x15 * 2;/* bank:ana2, addr:h0015 */
	sz_dbbus_txdata[3] = p_type[128 + 0x15] & 0xFF;
	sz_dbbus_txdata[4] = (p_type[128 + 0x15] >> 8) & 0xFF;
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 5);
/*
//#if 1 //for AC mod --showlo
    sz_dbbus_txdata[1] = 0x13;
    sz_dbbus_txdata[2] = 0x12 * 2;
    sz_dbbus_txdata[3] = 0x30;
    sz_dbbus_txdata[4] = 0x30;
    i2c_write_data(SLAVE_I2C_ID_DBBUS, sz_dbbus_txdata, 5);

    sz_dbbus_txdata[2] = 0x14 * 2;
    sz_dbbus_txdata[3] = 0X30;
    sz_dbbus_txdata[4] = 0X30;
    i2c_write_data(SLAVE_I2C_ID_DBBUS, sz_dbbus_txdata, 5);

    sz_dbbus_txdata[1] = 0x12;
    for (i = 0x0D; i <= 0x10; i ++) //for AC noise(++)
    {
	      sz_dbbus_txdata[2] = i * 2;
	      sz_dbbus_txdata[3] = p_type[128+i] & 0xFF;
	      sz_dbbus_txdata[4] = (p_type[128+i] >> 8) & 0xFF;
	      i2c_write_data(SLAVE_I2C_ID_DBBUS, sz_dbbus_txdata, 5);
    }

    for (i = 0x16; i <= 0x18; i ++) //for AC noise
    {
	      sz_dbbus_txdata[2] = i * 2;
	      sz_dbbus_txdata[3] = p_type[128+i] & 0xFF;
	      sz_dbbus_txdata[4] = (p_type[128+i] >> 8) & 0xFF;
	      i2c_write_data(SLAVE_I2C_ID_DBBUS, sz_dbbus_txdata, 5);
    }
//#endif
*/
}

static void _mptest_ito_opentest_msg21_setc(u8 n_csub_step)
{
	u32 i;
	u8 sz_dbbus_txdata[MAX_CHANNEL + 3];
	u8 n_highlevel_scub = false;
	u8 n_csubnew;

	LOGTP_DBUG("*** %s() n_csub_step = %d ***\n", __func__, n_csub_step);

	sz_dbbus_txdata[0] = 0x10;
	sz_dbbus_txdata[1] = 0x11;/* bank:ana, addr:h0042 */
	sz_dbbus_txdata[2] = 0x84;

	for (i = 0; i < MAX_CHANNEL; i++) {
		n_csubnew = n_csub_step;
		n_highlevel_scub = false;

		if (n_csubnew > 0x1F) {
			n_csubnew = n_csubnew - 0x14;
			n_highlevel_scub = true;
		}

		sz_dbbus_txdata[3 + i] = n_csubnew & 0x1F;
		if (n_highlevel_scub == true)
			sz_dbbus_txdata[3 + i] |= BIT5;

	}
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0],
	MAX_CHANNEL + 3);

	sz_dbbus_txdata[2] = 0xB4;/* bank:ana, addr:h005a */
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0],
	MAX_CHANNEL + 3);
}

static void _mptest_ito_opentest_msg21_first(u8 n_item_id, s16 *p_rawd_ata,
	s8 *p_dataflag)
{
	u32 i, j;
	s16 sz_tmp_rawdata[MAX_CHANNEL] = { 0 };
	u16 n_regval;
	u8 n_loop;
	u8 n_sensornum1 = 0, n_sensornum2 = 0, n_totalsensor = 0;
	u8 *p_mapping = NULL;

/*    n_sensornum1 = 0; */
/*    n_sensornum2 = 0; */

	LOGTP_DBUG("*** %s() n_item_id = %d ***\n", __func__, n_item_id);

	/* Stop cpu */
	reg_set16bit_value(0x0FE6, 0x0001);/* bank:mheg5, addr:h0073 */

	reg_set16bit_value(0x1E24, 0x0500);/* bank:chip, addr:h0012 */
	reg_set16bit_value(0x1E2A, 0x0000);/* bank:chip, addr:h0015 */
	reg_set16bit_value(0x1EE6, 0x6E00);/* bank:chip, addr:h0073 */
	reg_set16bit_value(0x1EE8, 0x0071);/* bank:chip, addr:h0074 */

	if (n_item_id == 40) {
		p_mapping = &_gmap1[0];
		if (_g_is_enable2r) {
			n_totalsensor = _g_ito_triangle_num / 2;
		} else {
			n_totalsensor =
	_g_ito_triangle_num / 2 + _g_ito_test_keynum +
	_g_ito_dummy_num;
		}
	} else if (n_item_id == 41) {
		p_mapping = &_gmap2[0];
		if (_g_is_enable2r) {
			n_totalsensor = _g_ito_triangle_num / 2;
		} else {
			n_totalsensor =
	_g_ito_triangle_num / 2 + _g_ito_test_keynum +
	_g_ito_dummy_num;
		}
	} else if (n_item_id == 42) {
		p_mapping = &_gmap3[0];
		n_totalsensor =
	_g_ito_triangle_num + _g_ito_test_keynum + _g_ito_dummy_num;
	}

	n_loop = 1;
	if (n_item_id != 42) {
		if (n_totalsensor > 11)
			n_loop = 2;

	}

	LOGTP_DBUG("n_loop = %d\n", n_loop);

	for (i = 0; i < n_loop; i++) {
		if (i == 0) {
			_mptest_ito_msg21xxa_getdatain(n_item_id - 36);
		} else {
			if (n_item_id == 40)
				_mptest_ito_msg21xxa_getdatain(9);
			else
				_mptest_ito_msg21xxa_getdatain(10);
			}
		}

		_mptest_disable_filternoise_detect();

		_mptest_ito_opentest_setv(1, 0);
		n_regval = reg_get16bit_value(0x110E);/* bank:ana, addr:h0007 */
		reg_set16bit_value(0x110E, n_regval | BIT11);

		if (_g_ltp == 1)
			_mptest_ito_opentest_msg21_setc(32);
		else
			_mptest_ito_opentest_msg21_setc(0);


		_mptest_ito_swreset();

		if (i == 0) {
			n_sensornum1 =
	_mptest_ito_msg21xxa_getdataout(sz_tmp_rawdata);
			LOGTP_DBUG("n_sensornum1 = %d\n", n_sensornum1);
		} else {
			n_sensornum2 =
	_mptest_ito_msg21xxa_getdataout(&sz_tmp_rawdata
				[n_sensornum1]);
	LOGTP_DBUG("n_sensornum1 = %d, n_sensornum2 = %d\n", n_sensornum1,
			n_sensornum2);
		}
	}

	for (j = 0; j < n_totalsensor; j++) {
		if (_g_ltp == 1) {
			p_rawd_ata[p_mapping[j]] = sz_tmp_rawdata[j] + 4096;
			p_dataflag[p_mapping[j]] = 1;
		} else {
			p_rawd_ata[p_mapping[j]] = sz_tmp_rawdata[j];
			p_dataflag[p_mapping[j]] = 1;
		}
	}
}

static void _mptest_itoshort_test_gpo_setting(u8 n_item_id)
{
	u8 sz_dbbus_txdata[3 + GPO_SETTING_SIZE * 2] = { 0 };
	u16 sz_gpo_setting[3] = { 0 };
	u32 i;

	LOGTP_DBUG("*** %s() n_item_id = %d ***\n", __func__, n_item_id);

		if (n_item_id == 0) {/* 39-4 */
		sz_gpo_setting[0] = _g_short_4_gpo[0];
		sz_gpo_setting[1] = _g_short_4_gpo[1];
		sz_gpo_setting[2] = _g_short_4_gpo[2];
		sz_gpo_setting[2] |= (1 << (int)(PIN_GUARD_RING % 16));
		} else if (n_item_id == 1) {/* 39-1 */
		sz_gpo_setting[0] = _g_short_1_gpo[0];
		sz_gpo_setting[1] = _g_short_1_gpo[1];
		sz_gpo_setting[2] = _g_short_1_gpo[2];
		sz_gpo_setting[2] |= (1 << (int)(PIN_GUARD_RING % 16));
		} else if (n_item_id == 2) {/* 39-2 */
		sz_gpo_setting[0] = _g_short_2_gpo[0];
		sz_gpo_setting[1] = _g_short_2_gpo[1];
		sz_gpo_setting[2] = _g_short_2_gpo[2];
		sz_gpo_setting[2] |= (1 << (int)(PIN_GUARD_RING % 16));
		} else if (n_item_id == 3) {/* 39-3 */
		sz_gpo_setting[0] = _g_short_3_gpo[0];
		sz_gpo_setting[1] = _g_short_3_gpo[1];
		sz_gpo_setting[2] = _g_short_3_gpo[2];
		sz_gpo_setting[2] |= (1 << (int)(PIN_GUARD_RING % 16));
	} else {
		LOGTP_DBUG("Invalid id changing GPIO setting of short test.\n");

		return;
	}

	sz_dbbus_txdata[0] = 0x10;
	sz_dbbus_txdata[1] = 0x12;
	sz_dbbus_txdata[2] = 0x48;

	for (i = 0; i < GPO_SETTING_SIZE; i++) {
		sz_dbbus_txdata[3 + 2 * i] = sz_gpo_setting[i] & 0xFF;
		sz_dbbus_txdata[4 + 2 * i] = (sz_gpo_setting[i] >> 8) & 0xFF;
	}

	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0],
	3 + GPO_SETTING_SIZE * 2);
}

static void _mptest_itoshort_test_rmode_setting(u8 n_mode)
{
	u8 sz_dbbus_txdata[6] = { 0 };

	LOGTP_DBUG("*** %s() n_mode = %d ***\n", __func__, n_mode);

	/* AFE R-mode enable(Bit-12) */
	reg_set_lowbyte(0x1103, 0x10);

	/* drv_mux_OV (Bit-8 1:enable) */
	reg_set_lowbyte(0x1107, 0x55);

		if (n_mode == 1) {/* P_CODE: 0V */
		reg_set16bit_value(0x110E, 0x073A);
		} else if (n_mode == 0) {/* N_CODE: 2.4V */
		reg_set16bit_value(0x110E, 0x073B);
	}
	/* SW2 rising & SW3 rising return to 0 */
	reg_set_lowbyte(0x1227, 0x01);
	/* turn off the chopping */
	reg_set_lowbyte(0x1208, 0x0C);
	/* idle driver ov */
	reg_set_lowbyte(0x1241, 0xC0);

	/* AFE ov */
	sz_dbbus_txdata[0] = 0x10;
	sz_dbbus_txdata[1] = 0x12;
	sz_dbbus_txdata[2] = 0x44;
	sz_dbbus_txdata[3] = 0xFF;
	sz_dbbus_txdata[4] = 0xFF;
	sz_dbbus_txdata[5] = 0xFF;

	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 6);
}

static void _mptest_itoshort_testmsg21_first(u8 n_item_id, s16 *p_rawd_ata,
						s8 *p_dataflag)
{
	u32 i;
	s16 sz_tmp_rawdata[MAX_CHANNEL] = { 0 };
	s16 sz_tmp_rawdata2[MAX_CHANNEL] = { 0 };
	u8 n_sensornum, n_sensornum2, n_numofsensor_mapping1;
	u8 n_numofsensor_mapping2, n_sensor_cnt = 0;
	u8 *p_mapping = NULL;

	LOGTP_DBUG("*** %s() n_item_id = %d ***\n", __func__, n_item_id);

	/* Stop cpu */
	reg_set16bit_value(0x0FE6, 0x0001);/* bank:mheg5, addr:h0073 */
	/* chip top op0 */
	reg_set16bit_value(0x1E24, 0x0500);/* bank:chip, addr:h0012 */
	reg_set16bit_value(0x1E2A, 0x0000);/* bank:chip, addr:h0015 */
	reg_set16bit_value(0x1EE6, 0x6E00);/* bank:chip, addr:h0073 */
	reg_set16bit_value(0x1EE8, 0x0071);/* bank:chip, addr:h0074 */

	if ((_g_ito_triangle_num + _g_ito_test_keynum + _g_ito_dummy_num) % 2 !=
	0) {
		n_numofsensor_mapping1 =
	(_g_ito_triangle_num + _g_ito_test_keynum +
	_g_ito_dummy_num) / 2 + 1;
		n_numofsensor_mapping2 = n_numofsensor_mapping1;
	} else {
		n_numofsensor_mapping1 =
	(_g_ito_triangle_num + _g_ito_test_keynum +
	_g_ito_dummy_num) / 2;
		n_numofsensor_mapping2 = n_numofsensor_mapping1;
		if (n_numofsensor_mapping2 % 2 != 0)
			n_numofsensor_mapping2++;

	}

		if (n_item_id == 0) {/* 39-4 (2R) */
		p_mapping = &_gshort_map4[0];
		n_sensor_cnt = _g_ito_triangle_num / 2;
		} else if (n_item_id == 1) {/* 39-1 */
		p_mapping = &_gshort_map1[0];
		n_sensor_cnt = n_numofsensor_mapping1;
		} else if (n_item_id == 2) {/* 39-2 */
		p_mapping = &_gshort_map2[0];
		n_sensor_cnt = n_numofsensor_mapping2;
		} else if (n_item_id == 3) {/* 39-3 */
		p_mapping = &_gshort_map3[0];
		n_sensor_cnt = _g_ito_triangle_num;
	}
	LOGTP_DBUG("n_sensor_cnt = %d\n", n_sensor_cnt);

	_mptest_ito_msg21xxa_getdatain(n_item_id);

	_mptest_disable_filternoise_detect();

	_mptest_itoshort_test_rmode_setting(1);
	_mptest_itoshort_test_gpo_setting(n_item_id);
	_mptest_ito_swreset();
	n_sensornum = _mptest_ito_msg21xxa_getdataout(sz_tmp_rawdata);
	LOGTP_DBUG("n_sensornum = %d\n", n_sensornum);

	_mptest_itoshort_test_rmode_setting(0);
	_mptest_itoshort_test_gpo_setting(n_item_id);
	_mptest_ito_swreset();
	n_sensornum2 = _mptest_ito_msg21xxa_getdataout(sz_tmp_rawdata2);
	LOGTP_DBUG("n_sensornum2 = %d\n", n_sensornum2);

	for (i = 0; i < n_sensor_cnt; i++) {
		p_rawd_ata[p_mapping[i]] = sz_tmp_rawdata[i] -
				sz_tmp_rawdata2[i];
		p_dataflag[p_mapping[i]] = 1;
	}
}

#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)

static u16 _mptest_ito_msg22_getdataout(s16 *p_rawd_ata,
				u16 n_sub_framenum)
{
	u32 i;
	u16 sz_rawdata[MAX_CHANNEL * 2] = { 0 };
	u16 n_regint = 0x0000;
	u16 n_size = n_sub_framenum * 4;
	u8 sz_dbbus_txdata[8] = { 0 };
	u8 sz_dbbus_rxdata[MAX_CHANNEL * 4] = { 0 };

	LOGTP_FUNC();

	reg_set16bit_valueoff(0x120A, BIT1);/* one-shot mode */
	reg_set16bit_valueoff(0x3D08, BIT8);/* FIQ_E_FRAME_READY_MASK */
	/* reg_set16bit_valueon(0x130C, BIT15); //MCU read done */
	reg_set16bit_valueon(0x120A, BIT0);/* trigger one-shot */

	LOGTP_DBUG("polling start\n");

	do {
		/* bank:intr_ctrl, addr:h000c */
		n_regint = reg_get16bit_value(0x3D18);
	} while ((n_regint & FIQ_E_FRAME_READY_MASK) == 0x0000);

	LOGTP_DBUG("polling end\n");

	/* Clear frame-ready interrupt status */
	reg_set16bit_valueoff(0x3D18, BIT8);

	/* ReadRegBurst start */
	sz_dbbus_txdata[0] = 0x10;
	sz_dbbus_txdata[1] = 0x15;/* bank:fir, addr:h0000 */
	sz_dbbus_txdata[2] = 0x00;
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 3);
	mdelay(20);
/*    i2c_read_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_rxdata[0], (MAX_CHANNEL *
 4)); */
	i2c_read_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_rxdata[0],
	(n_sub_framenum * 4 * 2));
	mdelay(100);

/*    for (i = 0; i < (MAX_CHANNEL * 4); i ++) */
	for (i = 0; i < (n_sub_framenum * 4 * 2); i++)
		LOGTP_DBUG("sz_dbbus_rxdata[%d] = %d\n",
				i, sz_dbbus_rxdata[i]);

	/* ReadRegBurst stop */

	reg_set16bit_valueon(0x3D08, BIT8);/* FIQ_E_FRAME_READY_MASK */

/*    for (i = 0; i < (MAX_CHANNEL * 2); i ++) */
	for (i = 0; i < n_size; i++) {
		sz_rawdata[i] =
	(sz_dbbus_rxdata[2 * i + 1] << 8) | (sz_dbbus_rxdata[2 * i]);
		p_rawd_ata[i] = (s16) sz_rawdata[i];
	}

/*    return (MAX_CHANNEL * 2); */
	return n_size;
}

static void _mptest_ito_msg22_senddatain(u8 n_step, u16 n_sub_framenum)
{
	u32 i;
	u32 *p_type = NULL;
	u16 n_riu_write_len = n_sub_framenum * 6;

	LOGTP_DBUG("*** %s() n_step = %d, n_sub_framenum = %d ***\n",
			 __func__, n_step, n_sub_framenum);

		if (n_step == 0) {/* 39-4 (2R) */
		p_type = &_gshort_riu4[0];
		} else if (n_step == 1) {/* 39-1 */
		p_type = &_gshort_riu1[0];
		} else if (n_step == 2) {/* 39-2 */
		p_type = &_gshort_riu2[0];
		} else if (n_step == 3) {/* 39-3 */
		p_type = &_gshort_riu3[0];
	} else if (n_step == 4) {
		p_type = &_g_open_riu1[0];
	} else if (n_step == 5) {
		p_type = &_g_open_riu2[0];
	} else if (n_step == 6) {
		p_type = &_g_open_riu3[0];
	}

	/* force on enable sensor mux and csub sel sram clock */
	reg_set16bit_valueon(0x1192, BIT4);
	reg_set16bit_valueoff(0x1192, BIT5);/* mem clk sel */
	reg_set16bit_valueoff(0x1100, BIT3);/* tgen soft rst */
	/* sensor mux sram read/write base address */
	reg_set16bit_value(0x1180, RIU_BASE_ADDR);
	/* sensor mux sram write length */
	reg_set16bit_value(0x1182, n_riu_write_len);
	reg_set16bit_valueon(0x1188, BIT0);/* reg_mem0_w_start */

	for (i = RIU_BASE_ADDR; i < (RIU_BASE_ADDR + n_riu_write_len); i++) {
		reg_set16bit_value(0x118A, (u16) (p_type[i]));
		reg_set16bit_value(0x118C, (u16) (p_type[i] >> 16));
	}
}

static void _mptest_ito_msg22_setc(u8 n_csub_step)
{
	u32 i;
	u16 n_csubnew, n_regval;

	LOGTP_DBUG("*** %s() n_csub_step = %d ***\n", __func__, n_csub_step);

	/* 6 bits */
	n_csubnew = (n_csub_step > CSUB_REF_MAX) ? CSUB_REF_MAX : n_csub_step;
	n_csubnew = (n_csubnew | (n_csubnew << 8));

	/* csub sel overwrite enable, will referance value of 11C0 */
	n_regval = reg_get16bit_value(0x11C8);

	if (n_regval == 0x000F) {
		reg_set16bit_value(0x11C0, n_csubnew);
	} else {
		/* force on enable sensor mux  and csub sel sram clock */
		reg_set16bit_valueon(0x1192, BIT4);
		reg_set16bit_valueoff(0x1192, BIT5);/* mem clk sel */
		reg_set16bit_valueoff(0x1100, BIT3);/* tgen soft rst */
		reg_set16bit_value(0x1184, 0);/* n_addr */
		reg_set16bit_value(0x1186, MAX_CHANNEL);/* nLen */
		reg_set16bit_valueon(0x1188, BIT2);/* reg_mem0_w_start */

		for (i = 0; i < MAX_CHANNEL; i++) {
			reg_set16bit_value(0x118E, n_csubnew);
			reg_set16bit_value(0x1190, n_csubnew);
		}
	}
}

static void _mptest_ito_msg22_reg_reset(void)
{
	LOGTP_FUNC();

	reg_set16bit_valueon(0x1102, (BIT3 | BIT4 | BIT5 | BIT6 | BIT7));
	reg_set16bit_valueoff(0x1130, (BIT0 | BIT1 | BIT2 | BIT3 | BIT8));

	reg_set16bit_valueon(0x1112, BIT15);
	reg_set16bit_valueoff(0x1112, BIT13);

	reg_set16bit_valueon(0x1250, ((5 << 0) & 0x007F));
	reg_set16bit_valueon(0x1250, ((1 << 8) & 0x7F00));
	reg_set16bit_valueoff(0x1250, 0x8080);

	reg_set16bit_valueoff(0x1312, (BIT12 | BIT14));

	reg_set16bit_valueon(0x1300, BIT15);
	reg_set16bit_valueoff(0x1300, (BIT10 | BIT11 | BIT12 | BIT13 | BIT14));

	reg_set16bit_valueon(0x1130, BIT9);

	reg_set16bit_valueon(0x1318, (BIT12 | BIT13));

/* reg_set16bit_value(0x121A, ((8 - 1) & 0x01FF));       // sampling
 number group A */
	/* sampling number group B */
	reg_set16bit_value(0x121C, ((8 - 1) & 0x01FF));
	reg_set16bit_valueon(0x131A, 0x2000);
	reg_set16bit_valueon(0x131C, BIT9);
	reg_set16bit_valueoff(0x131C, (BIT8 | BIT10));

	reg_set16bit_valueoff(0x1174, 0x0F00);

	reg_set16bit_value(0x1240, 0x0000);
	reg_set16bit_value(0x1242, 0x0000);

	reg_set16bit_value(0x1212, 0xFFFF);
	reg_set16bit_value(0x1214, 0x00FF);
/* reg_set16bit_value( 0x1212, 0);       // timing group A/B selection
 for total 24 subframes, 0: group A, 1: group B */
/* reg_set16bit_value( 0x1214, 0);       // timing group A/B selection
 for total 24 subframes, 0: group A, 1: group B */

	reg_set16bit_value(0x121E, 0xFFFF);
	reg_set16bit_value(0x1220, 0x00FF);
/* reg_set16bit_value( 0x121E, 0);   //sample number group A/B selection
 for total 24 subframes, 0: group A, 1: group B */
/* reg_set16bit_value( 0x1220, 0);   //sample number group A/B selection
 for total 24 subframes, 0: group A, 1: group B */

	/* noise sense mode selection for total 24 subframes */
	reg_set16bit_value(0x120E, 0x0);
	/* noise sense mode selection for total 24 subframes */
	reg_set16bit_value(0x1210, 0x0);

	reg_set16bit_value(0x128C, 0x0F);/* ADC afe gain correction bypass */
	mdelay(50);
}

static void _mptest_ito_msg22_getcharge_dumptime(u16 n_mode,
	u16 *p_chargetime,
	u16 *p_dumtime)
{
	u16 n_chargetime = 0, n_dumptime = 0;
	u16 n_min_chargetime = 0xFFFF, n_min_dumptime = 0xFFFF,
	u16 n_max_chargetime = 0x0000, n_max_dumptime = 0x0000;

	LOGTP_FUNC();

	n_chargetime = reg_get16bit_value(0x1226);
	n_dumptime = reg_get16bit_value(0x122A);

	if (n_min_chargetime > n_chargetime)
		n_min_chargetime = n_chargetime;


	if (n_max_chargetime < n_chargetime)
		n_max_chargetime = n_chargetime;


	if (n_min_dumptime > n_dumptime)
		n_min_dumptime = n_dumptime;


	if (n_max_dumptime < n_dumptime)
		n_max_dumptime = n_dumptime;


	LOGTP_DBUG("n_chargetime = %d, n_dumptime = %d\n",
			n_chargetime, n_dumptime);

	if (n_mode == 1) {
		*p_chargetime = n_max_chargetime;
		*p_dumtime = n_max_dumptime;
	} else {
		*p_chargetime = n_min_chargetime;
		*p_dumtime = n_min_dumptime;
	}
}

static void _mptest_ito_opentest_msg22first(u8 n_item_id, s16 *p_rawd_ata,
	s8 *p_dataflag)
{
	u32 i;
	s16 sz_tmp_rawdata[MAX_CHANNEL * 2] = { 0 };
	u16 n_sub_framenum = 0;
	u16 n_chargetime, n_dumptime;
	u8 *p_mapping = NULL;

	LOGTP_DBUG("*** %s() n_item_id = %d ***\n", __func__, n_item_id);

	/* Stop cpu */
	reg_set16bit_value(0x0FE6, 0x0001);/* bank:mheg5, addr:h0073 */

	_mptest_ito_msg22_reg_reset();

	switch (n_item_id) {
	case 40:
		p_mapping = &_gmap1[0];
		n_sub_framenum = _gopen_sub_framenum1;
		break;
	case 41:
		p_mapping = &_gmap2[0];
		n_sub_framenum = _gopen_sub_framenum2;
		break;
	case 42:
		p_mapping = &_gmap3[0];
		n_sub_framenum = _gopen_sub_framenum3;
		break;
	}

		if (n_sub_framenum > 24) {/* MAX_CHANNEL/2 */
		n_sub_framenum = 24;
	}

	_mptest_ito_msg22_senddatain(n_item_id - 36, n_sub_framenum * 6);

	if (true)
		reg_set16bit_value(0x1110, 0x0060);/* 2.4V -> 1.2V */
	else
		reg_set16bit_value(0x1110, 0x0020);/* 3.0V -> 0.6V */


	/* csub sel overwrite enable, will referance value of 11C0 */
	reg_set16bit_value(0x11C8, 0x000F);
	/* 1 : sel idel driver for sensor pad that connected to AFE */
	reg_set16bit_value(0x1174, 0x0F06);
	reg_set16bit_value(0x1208, 0x0006);/* PRS1 */
	reg_set16bit_value(0x1240, 0xFFFF);
	reg_set16bit_value(0x1242, 0x00FF);
	/* subframe numbers, 0:1subframe, 1:2subframe */
	reg_set16bit_value(0x1216, (n_sub_framenum - 1) << 1);

	_mptest_ito_msg22_getcharge_dumptime(1, &n_chargetime,
					&n_dumptime);

	reg_set16bit_value(0x1226, n_chargetime);
	reg_set16bit_value(0x122A, n_dumptime);

	_mptest_ito_msg22_setc(CSUB_REF);
	_mptest_ito_swreset();
	_mptest_ito_msg22_getdataout(sz_tmp_rawdata, n_sub_framenum);

	/*for (i = 0; i < (MAX_CHANNEL/2) ; i ++) */
	for (i = 0; i < n_sub_framenum; i++) {
		/*LOGTP_DBUG("sz_tmp_rawdata[%d * 4] >> 3 = %d\n", i,
		sz_tmp_rawdata[i * 4] >> 3);*/
		/*LOGTP_DBUG("p_mapping[%d] = %d\n", i, p_mapping[i]);*/

		/* Filter to ADC */
		p_rawd_ata[p_mapping[i]] = (sz_tmp_rawdata[i * 4] >> 3);
		p_dataflag[p_mapping[i]] = 1;
	}
}

static void _mptest_itoshort_test_msg22first(u8 n_item_id, s16 *p_rawd_ata,
	s8 *p_dataflag)
{
	u32 i, j;
	s16 sz_rawdata[MAX_CHANNEL * 2] = { 0 };
	s16 sz_tmp_rawdata[MAX_CHANNEL * 2] = { 0 };
	u16 n_sensornum = 0, n_sub_framenum = 0;
	u8 *p_mapping = NULL;

	LOGTP_DBUG("*** %s() n_item_id = %d ***\n", __func__, n_item_id);

	/* Stop cpu */
	reg_set16bit_value(0x0FE6, 0x0001);/* bank:mheg5, addr:h0073 */

	_mptest_ito_msg22_reg_reset();

	switch (n_item_id) {
	case 0:/* 39-4 (2R) */
		p_mapping = &_gshort_map4[0];
		n_sensornum = _gshortsub_framenum4;
		break;
	case 1:/* 39-1 */
		p_mapping = &_gshort_map1[0];
		n_sensornum = _g_shortsub_num;
		break;
	case 2:/* 39-2 */
		p_mapping = &_gshort_map2[0];
		n_sensornum = _gshortsub_framenum2;
		break;
	case 3:/* 39-3 */
		p_mapping = &_gshort_map3[0];
		n_sensornum = _gshortsub_framenum3;
		break;
	}

		if (n_sensornum > 24) {/* MAX_CHANNEL/2 */
		n_sub_framenum = 24;
	} else {
		n_sub_framenum = n_sensornum;
	}

	_mptest_ito_msg22_senddatain(n_item_id, n_sub_framenum * 6);

	/* reg_set16bit_value(0x1110, 0x0030); // [6:4}  011 : 2.1V -> 1.5V */
	reg_set16bit_value(0x1110, 0x0060);/* 2.4V -> 1.2V */
	/* csub sel overwrite enable, will referance value of 11C0 */
	reg_set16bit_value(0x11C8, 0x000F);
	reg_set16bit_value(0x1174, 0x0006);
	reg_set16bit_value(0x1208, 0x0006);/* PRS1 */
	reg_set16bit_valueon(0x1104, BIT14);/* R mode */
	/* subframe numbers, 0:1subframe, 1:2subframe */
	reg_set16bit_value(0x1216, (n_sub_framenum - 1) << 1);
	reg_set16bit_value(0x1176, 0x0000);/* CFB 10p */
	reg_set16bit_value(0x1226, 0x000B);/* Charge 4ns */
	reg_set16bit_value(0x122A, 0x000B);/* Dump 4ns */

	_mptest_ito_msg22_setc(CSUB_REF);
	_mptest_ito_swreset();
	_mptest_ito_msg22_getdataout(sz_tmp_rawdata, n_sub_framenum);

	if (n_sensornum > 24) {
		j = 0;

		/* for (i = 0; i < (MAX_CHANNEL/2); i ++) */
		for (i = 0; i < n_sub_framenum; i++) {
			sz_rawdata[j] = (sz_tmp_rawdata[i * 4] >> 2);
			j++;

			if ((n_sensornum - 24) > i) {
				sz_rawdata[j] = (sz_tmp_rawdata[i * 4 + 1]
							>> 2);
				j++;
			}
		}

		/* for (i = 0; i < (MAX_CHANNEL/2); i ++) */
		for (i = 0; i < n_sub_framenum; i++) {
			/*LOGTP_DBUG("rawdata[%d] = %d\n", i, sz_rawdata[i]);*/
			/*LOGTP_DBUG("p_mapping[%d] = %d\n", i, p_mapping[i]);*/

			p_rawd_ata[p_mapping[i]] = sz_rawdata[i];
			p_dataflag[p_mapping[i]] = 1;
		}
	} else {
		/*for (i = 0; i < (MAX_CHANNEL/2); i ++) */
		for (i = 0; i < n_sub_framenum; i++) {
			/*LOGTP_DBUG("tmp_rawdata[%d * 4] >> 2 = %d\n", i,
			sz_tmp_rawdata[i * 4] >> 2); */
			/*LOGTP_DBUG("mapping[%d] = %d\n", i, p_mapping[i]);*/

			p_rawd_ata[p_mapping[i]] = (sz_tmp_rawdata[i * 4] >> 2);
			p_dataflag[p_mapping[i]] = 1;
		}
	}
}

#endif

/*
 ------------------------------------------------------------------------------/
/ */

static enum ito_testresult_e _mptest_ito_opentest_second(u8 n_item_id)
{
	enum ito_testresult_e n_retval = ITO_TEST_OK;
	u32 i;
	s32 n_temp_rawdata_jg1 = 0;
	s32 n_temp_rawdata_jg2 = 0;
	s32 ntmp_jgavgth_max1 = 0;
	s32 mtmp_jgavgth_min1 = 0;
	s32 ntmp_jgavgth_max2 = 0;
	s32 mtmp_jgavgth_min2 = 0;

	LOGTP_DBUG("*** %s() n_item_id = %d ***\n", __func__, n_item_id);

	if (n_item_id == 40) {
		for (i = 0; i < (_g_ito_triangle_num / 2) - 2; i++)
			n_temp_rawdata_jg1 += _g_rawdata1[_gmap40_1[i]];


		for (i = 0; i < 2; i++)
			n_temp_rawdata_jg2 += _g_rawdata1[_gmap40_2[i]];

	} else if (n_item_id == 41) {
		for (i = 0; i < (_g_ito_triangle_num / 2) - 2; i++)
			n_temp_rawdata_jg1 += _g_rawdata2[_gmap41_1[i]];


		for (i = 0; i < 2; i++)
			n_temp_rawdata_jg2 += _g_rawdata2[_gmap41_2[i]];

	}

	ntmp_jgavgth_max1 =
	(n_temp_rawdata_jg1 / ((_g_ito_triangle_num / 2) - 2)) * (100 +
	OPEN_TEST_NON_BORDER_AREA_THRESHOLD)
	/ 100;
	mtmp_jgavgth_min1 =
	(n_temp_rawdata_jg1 / ((_g_ito_triangle_num / 2) - 2)) * (100 -
	OPEN_TEST_NON_BORDER_AREA_THRESHOLD)
	/ 100;
	ntmp_jgavgth_max2 =
	(n_temp_rawdata_jg2 / 2) * (100 +
	OPEN_TEST_BORDER_AREA_THRESHOLD) / 100;
	mtmp_jgavgth_min2 =
	(n_temp_rawdata_jg2 / 2) * (100 -
	OPEN_TEST_BORDER_AREA_THRESHOLD) / 100;

	LOGTP_DBUG("item_id = %d, rawdata_jg1 = %d, jgavgth_max1 = %d\n",
		n_item_id, n_temp_rawdata_jg1, ntmp_jgavgth_max1);
	LOGTP_DBUG("jgavgth_min1=%d, rawdataJg2 = %d, jgavgth_max2 = %d\n",
		mtmp_jgavgth_min1, n_temp_rawdata_jg2, ntmp_jgavgth_max2);
	LOGTP_DBUG("mtmp_jgavgth_min2 = %d\n", mtmp_jgavgth_min2);

	if (n_item_id == 40) {
		for (i = 0; i < (_g_ito_triangle_num / 2) - 2; i++) {
			if (_g_rawdata1[_gmap40_1[i]] > ntmp_jgavgth_max1
	|| _g_rawdata1[_gmap40_1[i]] < mtmp_jgavgth_min1) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gmap40_1[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}

		for (i = 0; i < 2; i++) {
			if (_g_rawdata1[_gmap40_2[i]] > ntmp_jgavgth_max2
	|| _g_rawdata1[_gmap40_2[i]] < mtmp_jgavgth_min2) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gmap40_2[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}
	} else if (n_item_id == 41) {
		for (i = 0; i < (_g_ito_triangle_num / 2) - 2; i++) {
			if (_g_rawdata2[_gmap41_1[i]] > ntmp_jgavgth_max1
	|| _g_rawdata2[_gmap41_1[i]] < mtmp_jgavgth_min1) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gmap41_1[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}

		for (i = 0; i < 2; i++) {
			if (_g_rawdata2[_gmap41_2[i]] > ntmp_jgavgth_max2
	|| _g_rawdata2[_gmap41_2[i]] < mtmp_jgavgth_min2) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gmap41_2[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}
	}

	return n_retval;
}

static enum ito_testresult_e _mptest_ito_opentest_second2r(u8 n_item_id)
{
	enum ito_testresult_e n_retval = ITO_TEST_OK;
	u32 i;
	s32 n_temp_rawdata_jg1 = 0;
	s32 n_temp_rawdata_jg2 = 0;
	s32 n_temp_rawdata_jg3 = 0;
	s32 n_temp_rawdata_jg4 = 0;
	s32 ntmp_jgavgth_max1 = 0;
	s32 mtmp_jgavgth_min1 = 0;
	s32 ntmp_jgavgth_max2 = 0;
	s32 mtmp_jgavgth_min2 = 0;
	s32 ntmp_jgavgth_max3 = 0;
	s32 mtmp_jgavgth_min3 = 0;
	s32 ntmp_jgavgth_max4 = 0;
	s32 mtmp_jgavgth_min4 = 0;

	LOGTP_INFO("%s() n_item_id = %d\n", __func__, n_item_id);

	if (n_item_id == 40) {
		for (i = 0; i < (_g_ito_triangle_num / 4) - 2; i++) {
			/* first region: non-border */
			n_temp_rawdata_jg1 += _g_rawdata1[_gmap40_1[i]];
		}

		for (i = 0; i < 2; i++) {
			/* first region: border */
			n_temp_rawdata_jg2 += _g_rawdata1[_gmap40_2[i]];
		}

		for (i = 0; i < (_g_ito_triangle_num / 4) - 2; i++) {
			/* second region: non-border */
			n_temp_rawdata_jg3 += _g_rawdata1[_gmap40_3[i]];
		}

		for (i = 0; i < 2; i++) {
			/* second region: border */
			n_temp_rawdata_jg4 += _g_rawdata1[_gmap40_4[i]];
		}
	} else if (n_item_id == 41) {
		for (i = 0; i < (_g_ito_triangle_num / 4) - 2; i++) {
			/* first region: non-border */
			n_temp_rawdata_jg1 += _g_rawdata2[_gmap41_1[i]];
		}

		for (i = 0; i < 2; i++) {
			/* first region: border */
			n_temp_rawdata_jg2 += _g_rawdata2[_gmap41_2[i]];
		}

		for (i = 0; i < (_g_ito_triangle_num / 4) - 2; i++) {
			/* second region: non-border */
			n_temp_rawdata_jg3 += _g_rawdata2[_gmap41_3[i]];
		}

		for (i = 0; i < 2; i++) {
			/* second region: border */
			n_temp_rawdata_jg4 += _g_rawdata2[_gmap41_4[i]];
		}
	}

	ntmp_jgavgth_max1 =
	(n_temp_rawdata_jg1 / ((_g_ito_triangle_num / 4) - 2)) * (100 +
	OPEN_TEST_NON_BORDER_AREA_THRESHOLD)
	/ 100;
	mtmp_jgavgth_min1 =
	(n_temp_rawdata_jg1 / ((_g_ito_triangle_num / 4) - 2)) * (100 -
	OPEN_TEST_NON_BORDER_AREA_THRESHOLD)
	/ 100;
	ntmp_jgavgth_max2 =
	(n_temp_rawdata_jg2 / 2) * (100 +
	OPEN_TEST_BORDER_AREA_THRESHOLD) / 100;
	mtmp_jgavgth_min2 =
	(n_temp_rawdata_jg2 / 2) * (100 -
	OPEN_TEST_BORDER_AREA_THRESHOLD) / 100;
	ntmp_jgavgth_max3 =
	(n_temp_rawdata_jg3 / ((_g_ito_triangle_num / 4) - 2)) * (100 +
	OPEN_TEST_NON_BORDER_AREA_THRESHOLD)
	/ 100;
	mtmp_jgavgth_min3 =
	(n_temp_rawdata_jg3 / ((_g_ito_triangle_num / 4) - 2)) * (100 -
	OPEN_TEST_NON_BORDER_AREA_THRESHOLD)
	/ 100;
	ntmp_jgavgth_max4 =
	(n_temp_rawdata_jg4 / 2) * (100 +
	OPEN_TEST_BORDER_AREA_THRESHOLD) / 100;
	mtmp_jgavgth_min4 =
	(n_temp_rawdata_jg4 / 2) * (100 -
	OPEN_TEST_BORDER_AREA_THRESHOLD) / 100;


	LOGTP_DBUG("item_id = %d, temp_rawdata_jg1 = %d, jgavgth_max1 = %d\n",
		n_item_id, n_temp_rawdata_jg1, ntmp_jgavgth_max1);
	LOGTP_DBUG("jgavgth_min1=%d, rawdata_jg2=%d, jgavgth_max2=%d\n",
		mtmp_jgavgth_min1, n_temp_rawdata_jg2, ntmp_jgavgth_max2);
	LOGTP_DBUG("jgavgth_min2 = %d\n", mtmp_jgavgth_min2);

	LOGTP_DBUG("rawdata_jg3=%d, jgavgth_max3=%d, jgavgth_min3=%d\n",
	n_temp_rawdata_jg3, ntmp_jgavgth_max3, mtmp_jgavgth_min3);
	LOGTP_DBUG("rawdata_jg4=%d, jgavgth_max4=%d, jgavgth_min4=%d\n",
		n_temp_rawdata_jg4, ntmp_jgavgth_max4, mtmp_jgavgth_min4);

	if (n_item_id == 40) {
		for (i = 0; i < (_g_ito_triangle_num / 4) - 2; i++) {
			if (_g_rawdata1[_gmap40_1[i]] > ntmp_jgavgth_max1
	|| _g_rawdata1[_gmap40_1[i]] < mtmp_jgavgth_min1) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gmap40_1[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}

		for (i = 0; i < 2; i++) {
			if (_g_rawdata1[_gmap40_2[i]] > ntmp_jgavgth_max2
	|| _g_rawdata1[_gmap40_2[i]] < mtmp_jgavgth_min2) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gmap40_2[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}

		for (i = 0; i < (_g_ito_triangle_num / 4) - 2; i++) {
			if (_g_rawdata1[_gmap40_3[i]] > ntmp_jgavgth_max3
	|| _g_rawdata1[_gmap40_3[i]] < mtmp_jgavgth_min3) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gmap40_3[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}

		for (i = 0; i < 2; i++) {
			if (_g_rawdata1[_gmap40_4[i]] > ntmp_jgavgth_max4
	|| _g_rawdata1[_gmap40_4[i]] < mtmp_jgavgth_min4) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gmap40_4[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}
	} else if (n_item_id == 41) {
		for (i = 0; i < (_g_ito_triangle_num / 4) - 2; i++) {
			if (_g_rawdata2[_gmap41_1[i]] > ntmp_jgavgth_max1
	|| _g_rawdata2[_gmap41_1[i]] < mtmp_jgavgth_min1) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gmap41_1[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}

		for (i = 0; i < 2; i++) {
			if (_g_rawdata2[_gmap41_2[i]] > ntmp_jgavgth_max2
	|| _g_rawdata2[_gmap41_2[i]] < mtmp_jgavgth_min2) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gmap41_2[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}

		for (i = 0; i < (_g_ito_triangle_num / 4) - 2; i++) {
			if (_g_rawdata2[_gmap41_3[i]] > ntmp_jgavgth_max3
	|| _g_rawdata2[_gmap41_3[i]] < mtmp_jgavgth_min3) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gmap41_3[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}

		for (i = 0; i < 2; i++) {
			if (_g_rawdata2[_gmap41_4[i]] > ntmp_jgavgth_max4
	|| _g_rawdata2[_gmap41_4[i]] < mtmp_jgavgth_min4) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gmap41_4[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}
	}

	return n_retval;
}

static enum ito_testresult_e _mptest_itoshort_test_second(u8 n_item_id)
{
	enum ito_testresult_e n_retval = ITO_TEST_OK;
	u32 i;
	u8 n_sensor_cnt = 0;
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
	u8 n_numofsensor_mapping1, n_numofsensor_mapping2;
#endif/* CONFIG_ENABLE_CHIP_MSG21XXA */

	LOGTP_DBUG("%s() n_item_id = %d\n", __func__, n_item_id);

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
	if ((_g_ito_triangle_num + _g_ito_test_keynum + _g_ito_dummy_num) % 2 !=
	0) {
		n_numofsensor_mapping1 =
	(_g_ito_triangle_num + _g_ito_test_keynum +
	_g_ito_dummy_num) / 2 + 1;
		n_numofsensor_mapping2 = n_numofsensor_mapping1;
	} else {
		n_numofsensor_mapping1 =
	(_g_ito_triangle_num + _g_ito_test_keynum +
	_g_ito_dummy_num) / 2;
		n_numofsensor_mapping2 = n_numofsensor_mapping1;
		if (n_numofsensor_mapping2 % 2 != 0)
			n_numofsensor_mapping2++;

	}
#endif/* CONFIG_ENABLE_CHIP_MSG21XXA */

		if (n_item_id == 0) {/* 39-4 (2R) */
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
		n_sensor_cnt = _g_ito_triangle_num / 2;
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
		n_sensor_cnt = _gshortsub_framenum4;
#endif
		for (i = 0; i < n_sensor_cnt; i++) {
			if (_g_rawdata4[_gshort_map4[i]] >
				SHORT_TEST_THRESHOLD) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gshort_map4[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}
		} else if (n_item_id == 1) {/* 39-1 */
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
		n_sensor_cnt = n_numofsensor_mapping1;
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
		n_sensor_cnt = _g_shortsub_num;
#endif
		for (i = 0; i < n_sensor_cnt; i++) {
			if (_g_rawdata1[_gshort_map1[i]] >
				SHORT_TEST_THRESHOLD) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gshort_map1[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}
		} else if (n_item_id == 2) {/* 39-2 */
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
		n_sensor_cnt = n_numofsensor_mapping2;
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
		n_sensor_cnt = _gshortsub_framenum2;
#endif
		for (i = 0; i < n_sensor_cnt; i++) {
			if (_g_rawdata2[_gshort_map2[i]] >
				SHORT_TEST_THRESHOLD) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gshort_map2[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}
		} else if (n_item_id == 3) {/* 39-3 */
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
		n_sensor_cnt = _g_ito_triangle_num;
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
		n_sensor_cnt = _gshortsub_framenum3;
#endif
		for (i = 0; i < n_sensor_cnt; i++) {
			if (_g_rawdata3[_gshort_map3[i]] >
				SHORT_TEST_THRESHOLD) {
				_g_testfail_channel[_g_testfail_channel_cnt] =
	_gshort_map3[i];
				_g_testfail_channel_cnt++;
				n_retval = ITO_TEST_FAIL;
			}
		}
	}
	LOGTP_DBUG("n_sensor_cnt = %d\n", n_sensor_cnt);

	return n_retval;
}

s32 _mptest_ito_opentest(void)
{
	enum ito_testresult_e n_retval1 = ITO_TEST_OK, n_retval2 =
	ITO_TEST_OK, n_retval3 = ITO_TEST_OK;
	u32 i;

	LOGTP_FUNC();

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_alloc();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	LOGTP_DBUG("open test start\n");

	platform_set_i2crate(gi2c_client, 50000);/* 50 KHZ */

	platform_disable_finger_touchreport();
	platform_device_reset_hw();

	if (!_mptest_ito_choose_tp_type()) {
		LOGTP_DBUG("Choose Tp Type failed\n");
		n_retval1 = ITO_GET_TP_TYPE_ERROR;
		goto ITO_TEST_END;
	}

	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();
	mdelay(100);

	/* Stop cpu */
	reg_set16bit_value(0x0FE6, 0x0001);/* bank:mheg5, addr:h0073 */

	/* Stop watchdog */
	reg_set16bit_value(0x3C60, 0xAA55);/* bank:reg_PIU_MISC_0, addr:h0030 */

	mdelay(50);

	for (i = 0; i < MAX_CHANNEL; i++) {
		_g_rawdata1[i] = 0;
		_g_rawdata2[i] = 0;
		_g_rawdata3[i] = 0;
		_g_dataflag1[i] = 0;
		_g_dataflag2[i] = 0;
		_g_dataflag3[i] = 0;
	}

	/* Reset _g_testfail_channel_cnt to 0 before test start */
	_g_testfail_channel_cnt = 0;

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
	_mptest_ito_opentest_msg21_first(40, _g_rawdata1, _g_dataflag1);
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
	_mptest_ito_opentest_msg22first(40, _g_rawdata1, _g_dataflag1);
#endif

	if (_g_is_enable2r)
		n_retval2 = _mptest_ito_opentest_second2r(40);
	else
		n_retval2 = _mptest_ito_opentest_second(40);


#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
	_mptest_ito_opentest_msg21_first(41, _g_rawdata2, _g_dataflag2);
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
	_mptest_ito_opentest_msg22first(41, _g_rawdata2, _g_dataflag2);
#endif

	if (_g_is_enable2r)
		n_retval3 = _mptest_ito_opentest_second2r(41);
	else
		n_retval3 = _mptest_ito_opentest_second(41);

/*
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
    _mptest_ito_opentest_msg21_first(42, _g_rawdata3, _g_dataflag3);
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
    _mptest_ito_opentest_msg22first(42, _g_rawdata3, _g_dataflag3);
#endif
*/

ITO_TEST_END:
#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_free();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	platform_set_i2crate(gi2c_client, 100000);/* 100 KHZ */

	platform_device_reset_hw();
	platform_enable_finger_touchreport();

	LOGTP_DBUG("open test end\n");

	if ((n_retval1 != ITO_TEST_OK) && (n_retval2 == ITO_TEST_OK)
	&& (n_retval3 == ITO_TEST_OK)) {
		return ITO_GET_TP_TYPE_ERROR;
	} else if ((n_retval1 == ITO_TEST_OK)
	&& ((n_retval2 != ITO_TEST_OK)
	|| (n_retval3 != ITO_TEST_OK))) {
		return ITO_TEST_FAIL;
	} else {
		return ITO_TEST_OK;
	}
}

static enum ito_testresult_e _mptest_itoshort_test(void)
{
	enum ito_testresult_e n_retval1 = ITO_TEST_OK, n_retval2 =
	ITO_TEST_OK, n_retval3 = ITO_TEST_OK, n_retval4 =
	ITO_TEST_OK, n_retval5 = ITO_TEST_OK;
	u32 i = 0;

	LOGTP_FUNC();

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_alloc();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	LOGTP_DBUG("short test start\n");

	platform_set_i2crate(gi2c_client, 50000);/* 50 KHZ */

	platform_disable_finger_touchreport();
	platform_device_reset_hw();

	if (!_mptest_ito_choose_tp_type()) {
		LOGTP_DBUG("Choose Tp Type failed\n");
		n_retval1 = ITO_GET_TP_TYPE_ERROR;
		goto ITO_TEST_END;
	}

	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();
	mdelay(100);

	/* Stop cpu */
	reg_set16bit_value(0x0FE6, 0x0001);/* bank:mheg5, addr:h0073 */

	/* Stop watchdog */
	reg_set16bit_value(0x3C60, 0xAA55);/* bank:reg_PIU_MISC_0, addr:h0030 */

	mdelay(50);

	for (i = 0; i < MAX_CHANNEL; i++) {
		_g_rawdata1[i] = 0;
		_g_rawdata2[i] = 0;
		_g_rawdata3[i] = 0;
		_g_rawdata4[i] = 0;
		_g_dataflag1[i] = 0;
		_g_dataflag2[i] = 0;
		_g_dataflag3[i] = 0;
		_g_dataflag4[i] = 0;
	}

	/* Reset _g_testfail_channel_cnt to 0 before test start */
	_g_testfail_channel_cnt = 0;

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
	_mptest_itoshort_testmsg21_first(1, _g_rawdata1, _g_dataflag1);
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
	_mptest_itoshort_test_msg22first(1, _g_rawdata1, _g_dataflag1);
#endif

	n_retval2 = _mptest_itoshort_test_second(1);

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
	_mptest_itoshort_testmsg21_first(2, _g_rawdata2, _g_dataflag2);
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
	_mptest_itoshort_test_msg22first(2, _g_rawdata2, _g_dataflag2);
#endif

	n_retval3 = _mptest_itoshort_test_second(2);

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
	_mptest_itoshort_testmsg21_first(3, _g_rawdata3, _g_dataflag3);
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
	_mptest_itoshort_test_msg22first(3, _g_rawdata3, _g_dataflag3);
#endif

	n_retval4 = _mptest_itoshort_test_second(3);

	if (_g_is_enable2r) {
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA)
		_mptest_itoshort_testmsg21_first(0, _g_rawdata4, _g_dataflag4);
#elif defined(CONFIG_ENABLE_CHIP_MSG22XX)
		_mptest_itoshort_test_msg22first(0, _g_rawdata4, _g_dataflag4);
#endif

		n_retval5 = _mptest_itoshort_test_second(0);
	}

ITO_TEST_END:
#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_free();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	platform_set_i2crate(gi2c_client, 100000);/* 100 KHZ */

	platform_device_reset_hw();
	platform_enable_finger_touchreport();

	LOGTP_DBUG("short test end\n");

	if ((n_retval1 != ITO_TEST_OK) && (n_retval2 == ITO_TEST_OK)
	&& (n_retval3 == ITO_TEST_OK) && (n_retval4 == ITO_TEST_OK)
	&& (n_retval5 == ITO_TEST_OK)) {
		return ITO_GET_TP_TYPE_ERROR;
	} else if ((n_retval1 == ITO_TEST_OK)
	&& ((n_retval2 != ITO_TEST_OK) || (n_retval3 != ITO_TEST_OK)
	|| (n_retval4 != ITO_TEST_OK)
	|| (n_retval5 != ITO_TEST_OK))) {
		return ITO_TEST_FAIL;
	} else {
		return ITO_TEST_OK;
	}
}

/*
static s32 DrvMpTestProcReadDebug(char *page, char **start, off_t off, int
 count, int *eof, void *data)
{
	s32 n_count = 0;

	LOGTP_FUNC();

	_g_ito_result = _mptest_ito_opentest();

	if (ITO_TEST_OK == _g_ito_result)
		LOGTP_DBUG("ITO_TEST_OK");

	else if (ITO_TEST_FAIL == _g_ito_result)
		LOGTP_DBUG("ITO_TEST_FAIL");

	else if (ITO_GET_TP_TYPE_ERROR == _g_ito_result)
		LOGTP_DBUG("ITO_GET_TP_TYPE_ERROR");


	*eof = 1;

	return n_count;
}

static s32 DrvMpTestProcWriteDebug(struct file *file, const char *buffer,
 unsigned long count, void *data)
{
	u32 i;

	LOGTP_FUNC();

	LOGTP_DBUG("ito test result : %d", _g_ito_result);

	for (i = 0; i < MAX_CHANNEL; i ++)
		LOGTP_DBUG("_g_rawdata1[%d]=%d\n", i, _g_rawdata1[i]);

	mdelay(5);

	for (i = 0; i < MAX_CHANNEL; i ++)
		LOGTP_DBUG("_g_rawdata2[%d]=%d\n", i, _g_rawdata2[i]);
	mdelay(5);

	for (i = 0; i < MAX_CHANNEL; i ++)
		LOGTP_DBUG("_g_rawdata3[%d]=%d;\n",i , _g_rawdata3[i]);
	mdelay(5);

	return count;
}
*/

static void _mptest_itotest_dowork(struct work_struct *p_work)
{
	s32 n_retval = ITO_TEST_OK;

	LOGTP_DBUG("%s(),g_is_in_mptest = %d, gtest_rery_cnt = %d\n", __func__,
	_g_is_in_mptest, _gtest_rery_cnt);

	if (_g_ito_testmode == ITO_TEST_MODE_OPEN_TEST) {
		n_retval = _mptest_ito_opentest();
	} else if (_g_ito_testmode == ITO_TEST_MODE_SHORT_TEST) {
		n_retval = _mptest_itoshort_test();
	} else {
		LOGTP_DBUG("Undefined Mp Test Mode = %d\n", _g_ito_testmode);
		return;
	}

	LOGTP_DBUG("ctp mp test result = %d\n", n_retval);

	if (n_retval == ITO_TEST_OK) {
		_gctp_mptest_status = ITO_TEST_OK;
		_g_is_in_mptest = 0;
		LOGTP_DBUG("mp test success\n");

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
		fw_restore_firmwaremode_tologdata();
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */
	} else {
		_gtest_rery_cnt--;
		if (_gtest_rery_cnt > 0) {
			LOGTP_DBUG("_gtest_rery_cnt = %d\n", _gtest_rery_cnt);
			queue_work(_g_ctp_mptest_workqueue, &_g__ito_test_work);
		} else {
			if (n_retval == ITO_TEST_FAIL)
				_gctp_mptest_status = ITO_TEST_FAIL;
			else if (n_retval == ITO_GET_TP_TYPE_ERROR)
				_gctp_mptest_status = ITO_GET_TP_TYPE_ERROR;
			else
				_gctp_mptest_status = ITO_TEST_UNDEFINED_ERROR;


			_g_is_in_mptest = 0;
			LOGTP_DBUG("mp test failed\n");

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
			fw_restore_firmwaremode_tologdata();
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */
		}
	}
}

/*=============================================================*/
/* GLOBAL FUNCTION DEFINITION */
/*=============================================================*/

/*
void DrvMpTestCreateProcEntry(void)
{
	_gMsgItoTest = proc_mkdir(PROC_MSG_ITO_TEST, NULL);
	_gDebug = create_proc_entry(PROC_ITO_TEST_DEBUG, 0777, _gMsgItoTest);

	if (NULL == _gDebug)
	{
		LOGTP_DBUG("create_proc_entry PROC_ITO_TEST_DEBUG FAIL\n");
	}
	else
	{
		_gDebug->read_proc = DrvMpTestProcReadDebug;
		_gDebug->write_proc = DrvMpTestProcWriteDebug;
		LOGTP_DBUG("create_proc_entry PROC_ITO_TEST_DEBUG OK\n");
	}
}
*/

s32 mp_get_test_result(void)
{
	LOGTP_FUNC();
	LOGTP_DBUG("_gctp_mptest_status = %d\n", _gctp_mptest_status);

	return _gctp_mptest_status;
}

void mptest_get_failchannel(enum ito_testmode_e e_ito_testmode,
	u8 *p_fail_channel, u32 *p_failchannel_count)
{
	u32 i;

	LOGTP_FUNC();
	LOGTP_DBUG("_g_testfail_channel_cnt = %d\n", _g_testfail_channel_cnt);

	for (i = 0; i < _g_testfail_channel_cnt; i++)
		p_fail_channel[i] = _g_testfail_channel[i];


	*p_failchannel_count = _g_testfail_channel_cnt;
}

void mptest_get_datalog(enum ito_testmode_e e_ito_testmode, u8 *p_datalog,
	u32 *p_len)
{
	u32 i;
	u8 n_highbyte, n_lowbyte;

	LOGTP_FUNC();

	if (e_ito_testmode == ITO_TEST_MODE_OPEN_TEST) {
		for (i = 0; i < MAX_CHANNEL; i++) {
			n_highbyte = (_g_rawdata1[i] >> 8) & 0xFF;
			n_lowbyte = (_g_rawdata1[i]) & 0xFF;

			if (_g_dataflag1[i] == 1)
				/* indicate it is a on-use channel number */
				p_datalog[i * 4] = 1;
			else
				/* indicate it is a non-use channel number */
				p_datalog[i * 4] = 0;


			if (_g_rawdata1[i] >= 0)
				/* + : a positive number */
				p_datalog[i * 4 + 1] = 0;
			else
				/* - : a negative number */
				p_datalog[i * 4 + 1] = 1;


			p_datalog[i * 4 + 2] = n_highbyte;
			p_datalog[i * 4 + 3] = n_lowbyte;
		}

		for (i = 0; i < MAX_CHANNEL; i++) {
			n_highbyte = (_g_rawdata2[i] >> 8) & 0xFF;
			n_lowbyte = (_g_rawdata2[i]) & 0xFF;

			if (_g_dataflag2[i] == 1)
				/* indicate it is a on-use channel number */
				p_datalog[i * 4 + MAX_CHANNEL * 4] = 1;
			else
				/* indicate it is a non-use channel number */
				p_datalog[i * 4 + MAX_CHANNEL * 4] = 0;


			if (_g_rawdata2[i] >= 0)
				p_datalog[(i * 4 + 1) + MAX_CHANNEL * 4] = 0;
			else
				p_datalog[(i * 4 + 1) + MAX_CHANNEL * 4] = 1;


			p_datalog[(i * 4 + 2) + MAX_CHANNEL * 4] = n_highbyte;
			p_datalog[(i * 4 + 3) + MAX_CHANNEL * 4] = n_lowbyte;
		}

		*p_len = MAX_CHANNEL * 8;
	} else if (e_ito_testmode == ITO_TEST_MODE_SHORT_TEST) {
		for (i = 0; i < MAX_CHANNEL; i++) {
			n_highbyte = (_g_rawdata1[i] >> 8) & 0xFF;
			n_lowbyte = (_g_rawdata1[i]) & 0xFF;

			if (_g_dataflag1[i] == 1)
				/* indicate it is a on-use channel number */
				p_datalog[i * 4] = 1;
			else
				/* indicate it is a non-use channel number */
				p_datalog[i * 4] = 0;


			if (_g_rawdata1[i] >= 0)
				/* + : a positive number */
				p_datalog[i * 4 + 1] = 0;
			else
				/* - : a negative number */
				p_datalog[i * 4 + 1] = 1;


			p_datalog[i * 4 + 2] = n_highbyte;
			p_datalog[i * 4 + 3] = n_lowbyte;
		}

		for (i = 0; i < MAX_CHANNEL; i++) {
			n_highbyte = (_g_rawdata2[i] >> 8) & 0xFF;
			n_lowbyte = (_g_rawdata2[i]) & 0xFF;

			if (_g_dataflag2[i] == 1)
				/* indicate it is a on-use channel number */
				p_datalog[i * 4 + MAX_CHANNEL * 4] = 1;
			else
				/* indicate it is a non-use channel number */
				p_datalog[i * 4 + MAX_CHANNEL * 4] = 0;


			if (_g_rawdata2[i] >= 0)
				p_datalog[(i * 4 + 1) +
				MAX_CHANNEL * 4] = 0;
			else
				p_datalog[(i * 4 + 1) +
				MAX_CHANNEL * 4] = 1;


			p_datalog[i * 4 + 2 + MAX_CHANNEL * 4] = n_highbyte;
			p_datalog[i * 4 + 3 + MAX_CHANNEL * 4] = n_lowbyte;
		}

		for (i = 0; i < MAX_CHANNEL; i++) {
			n_highbyte = (_g_rawdata3[i] >> 8) & 0xFF;
			n_lowbyte = (_g_rawdata3[i]) & 0xFF;

			if (_g_dataflag3[i] == 1) {
				/* indicate it is a on-use channel number */
				p_datalog[i * 4 + MAX_CHANNEL * 8] = 1;
			} else {
				/* indicate it is a non-use channel number */
				p_datalog[i * 4 + MAX_CHANNEL * 8] = 0;
			}

			if (_g_rawdata3[i] >= 0) {
				p_datalog[(i * 4 + 1) +
				MAX_CHANNEL * 8] = 0;
			} else {
				p_datalog[(i * 4 + 1) +
				MAX_CHANNEL * 8] = 1;
			}

			p_datalog[(i * 4 + 2) + MAX_CHANNEL * 8] = n_highbyte;
			p_datalog[(i * 4 + 3) + MAX_CHANNEL * 8] = n_lowbyte;
		}

		if (_g_is_enable2r) {
			for (i = 0; i < MAX_CHANNEL; i++) {
				n_highbyte = (_g_rawdata4[i] >> 8) & 0xFF;
				n_lowbyte = (_g_rawdata4[i]) & 0xFF;

				if (_g_dataflag4[i] == 1)
					p_datalog[i * 4 +
					MAX_CHANNEL * 12] = 1;
				else
					p_datalog[i * 4 +
					MAX_CHANNEL * 12] = 0;


				if (_g_rawdata4[i] >= 0)
					p_datalog[(i * 4 + 1) +
					MAX_CHANNEL * 12] = 0;
				else
					p_datalog[(i * 4 + 1) +
					MAX_CHANNEL * 12] = 1;


				p_datalog[(i * 4 + 2) + MAX_CHANNEL * 12] =
					n_highbyte;
				p_datalog[(i * 4 + 3) + MAX_CHANNEL * 12] =
					n_lowbyte;
			}
		}

		*p_len = MAX_CHANNEL * 16;
	} else {
		LOGTP_INFO("*** Undefined MP Test Mode ***\n");
	}
}

void mptest_schedule_work(enum ito_testmode_e e_ito_testmode)
{
	LOGTP_FUNC();

	if (_g_is_in_mptest == 0) {
		LOGTP_INFO("ctp mp test start\n");

		_g_ito_testmode = e_ito_testmode;
		_g_is_in_mptest = 1;
		_gtest_rery_cnt = CTP_MP_TEST_RETRY_COUNT;
		_gctp_mptest_status = ITO_TEST_UNDER_TESTING;

		queue_work(_g_ctp_mptest_workqueue, &_g__ito_test_work);
	}
}

void mptest_create_workqueue(void)
{
	LOGTP_FUNC();

	_g_ctp_mptest_workqueue = create_singlethread_workqueue("ctp_mp_test");
	INIT_WORK(&_g__ito_test_work, _mptest_itotest_dowork);
}

#endif/* CONFIG_ENABLE_ITO_MP_TEST */
