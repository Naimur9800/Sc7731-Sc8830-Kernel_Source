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
 * @file    mstar_drv_self_fw_control.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

/*=============================================================*/
/* INCLUDE FILE */
/*=============================================================*/

#include "mstar_drv_self_fw_control.h"
#include "mstar_drv_utility_adaption.h"
#include "mstar_drv_platform_porting_layer.h"

/*=============================================================*/
/* EXTERN VARIABLE DECLARATION */
/*=============================================================*/
/* virtualkey 1 MENU */
static const int g_virtualkey_dimlocal[3][4] = { {256, 1000, 70, 70},
				{128, 1000, 70, 70},/* virtualkey 4  HOME */
				{192, 1000, 70, 70}
				};/* virtualkey 2  BACK */

struct point_node_t point_slot[MAX_FINGER_NUM * 2];
/*=============================================================*/
/* LOCAL VARIABLE DEFINITION */
/*=============================================================*/

static u8 _g_dwi2c_info_data[1024];
static u8 _g_one_fwdata[MSG22XX_FM_BLOCKSIZE * 1024 +
			MSG22XX_FIRMWARE_INFO_BLOCK_SIZE] = { 0 };

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
/*
 * Note.
 * Please modify the name of the below .h depends on the vendor TP that you are
 using.
 */
static unsigned char msg2xxx_yyyy_update_bin[] = {
};

static unsigned char msg2xxx_update_bin[] = {
#include "msg2xxx_update_bin.h"
};

static u32 _g_update_retry_cnt = UPDATE_FIRMWARE_RETRY_COUNT;
static struct work_struct _g_update_byswid_work;
static struct workqueue_struct *_g_update_byswid_workqueue; /* = NULL; */

static u32 _g_isupdateinfo_blockfirst; /* = 0; */
static u8 _g_isupdate_firmware; /* = 0; */

#endif/* CONFIG_UPDATE_FIRMWARE_BY_SW_ID */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
static u16 _g_gesture_wakeup_value; /* = 0; */
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

/*=============================================================*/
/* GLOBAL VARIABLE DEFINITION */
/*=============================================================*/

u8 g_chiptype; /* = 0; */
u8 g_demomode_packet[DEMO_MODE_PACKET_LENGTH] = { 0 };

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
firmware_info_t g_firmware_info;

u8 *g_logmode_packet; /* = NULL; */
u16 g_firmware_mode = FIRMWARE_MODE_DEMO_MODE;
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
u8 *_g_gesturewakeup_packet; /* = NULL; */

u16 g_gesture_wakeup_mode; /* = 0x0000; */
u8 g_gesture_wakeup_flag; /* = 0; */
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

/*=============================================================*/
/* LOCAL FUNCTION DECLARATION */
/*=============================================================*/

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
static void _fw_update_firmware_byswid_dowork(struct work_struct *p_work);
#endif/* CONFIG_UPDATE_FIRMWARE_BY_SW_ID */

/*=============================================================*/
/* LOCAL FUNCTION DEFINITION */
/*=============================================================*/

unsigned char tpd_check_sum(unsigned char *pval)
{
	int i, sum = 0;
	for (i = 0; i < 7; i++)
		sum += pval[i];
	return (unsigned char)((-sum) & 0xFF);
}

static void _fwctrl_erase_ememc32(void)
{
	LOGTP_FUNC();

	/* /////////////////////// */
	/* Erase  all */
	/* /////////////////////// */

	/* Enter gpio mode */
	reg_set16bit_value(0x161E, 0xBEAF);

	/* Before gpio mode, set the control pin as the orginal status */
	reg_set16bit_value(0x1608, 0x0000);
	reg_set_lowbyte(0x160E, 0x10);
	mdelay(10);

	/* ptrim = 1, h'04[2] */
	reg_set_lowbyte(0x1608, 0x04);
	reg_set_lowbyte(0x160E, 0x10);
	mdelay(10);

	/* ptm = 6, h'04[12:14] = b'110 */
	reg_set_lowbyte(0x1609, 0x60);
	reg_set_lowbyte(0x160E, 0x10);

	/* pmasi = 1, h'04[6] */
	reg_set_lowbyte(0x1608, 0x44);
	/* pce = 1, h'04[11] */
	reg_set_lowbyte(0x1609, 0x68);
	/* perase = 1, h'04[7] */
	reg_set_lowbyte(0x1608, 0xC4);
	/* pnvstr = 1, h'04[5] */
	reg_set_lowbyte(0x1608, 0xE4);
	/* pwe = 1, h'04[9] */
	reg_set_lowbyte(0x1609, 0x6A);
	/* trigger gpio load */
	reg_set_lowbyte(0x160E, 0x10);
}

static void _fwctrl_erase_ememc33(enum emem_type_e e_mem_type)
{
	LOGTP_FUNC();

	/* Stop mcu */
	reg_set16bit_value(0x0FE6, 0x0001);

	/* Disable watchdog */
	reg_set_lowbyte(0x3C60, 0x55);
	reg_set_lowbyte(0x3C61, 0xAA);

	/* Set PROGRAM password */
	reg_set_lowbyte(0x161A, 0xBA);
	reg_set_lowbyte(0x161B, 0xAB);

	/* Clear pce */
	reg_set_lowbyte(0x1618, 0x80);

	if (e_mem_type == EMEM_ALL)
		reg_set_lowbyte(0x1608, 0x10);/* mark */


	reg_set_lowbyte(0x1618, 0x40);
	mdelay(10);

	reg_set_lowbyte(0x1618, 0x80);

	/* erase trigger */
	if (e_mem_type == EMEM_MAIN)
		reg_set_lowbyte(0x160E, 0x04);/* erase main */
	else
		reg_set_lowbyte(0x160E, 0x08);/* erase all block */

}

#if 0
static void _fwctrl_msg22_get_vendorcode(u8 *p_tpvendor_code)
{
	LOGTP_FUNC();

	if (g_chiptype == CHIP_TYPE_MSG22XX) {
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

reg_set16bit_value(0x1600, 0xC1E9);

		/* Enable burst mode */
		/*reg_set16bit_value(0x160C,
		(reg_get16bit_value(0x160C) | 0x01)); */

		/* Set pce */
		reg_set16bit_value(0x1618, (reg_get16bit_value(0x1618) | 0x40));

		reg_set_lowbyte(0x160E, 0x01);

		n_reg_data1 = reg_get16bit_value(0x1604);
		n_reg_data2 = reg_get16bit_value(0x1606);

		p_tpvendor_code[0] = ((n_reg_data1 >> 8) & 0xFF);
		p_tpvendor_code[1] = (n_reg_data2 & 0xFF);
		p_tpvendor_code[2] = ((n_reg_data2 >> 8) & 0xFF);

		/* Clear burst mode */
		/*reg_set16bit_value(0x160C,
		reg_get16bit_value(0x160C) & (~0x01)); */

		reg_set16bit_value(0x1600, 0x0000);

		/* Clear RIU password */
		reg_set16bit_value(0x161A, 0x0000);

		bus_i2c_notuse_buf();
		buf_not_stop_mcu();
		bus_exit_serial_debugmode();

		platform_device_reset_hw();
		mdelay(100);
	}
}

static void _fwctrl_msg22erase_emem(enum emem_type_e e_mem_type)
{
	u32 i;
	u16 n_reg_data = 0;

	LOGTP_DBUG(" %s() e_mem_type = %d\n", __func__, e_mem_type);

	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();

	LOGTP_DBUG("Erase start\n");

	/* Stop mcu */
	reg_set16bit_value(0x0FE6, 0x0001);

	/* Disable watchdog */
	reg_set_lowbyte(0x3C60, 0x55);
	reg_set_lowbyte(0x3C61, 0xAA);

	/* Set PROGRAM password */
	reg_set_lowbyte(0x161A, 0xBA);
	reg_set_lowbyte(0x161B, 0xAB);

		if (e_mem_type == EMEM_ALL) {/* 48KB + 512Byte */
		LOGTP_DBUG("Erase all block\n");

		/* Clear pce */
		reg_set_lowbyte(0x1618, 0x80);
		mdelay(100);

		/* Chip erase */
		reg_set16bit_value(0x160E, BIT3);

		LOGTP_DBUG("Wait erase done flag\n");

		do  {/* Wait erase done flag */
			/* Memory status */
			n_reg_data = reg_get16bit_value(0x1610);
			mdelay(50);
		} while ((n_reg_data & BIT1) != BIT1);
		} else if (e_mem_type == EMEM_MAIN) {/* 48KB (32+8+8) */
		LOGTP_DBUG("Erase main block\n");

		for (i = 0; i < 3; i++) {
			/* Clear pce */
			reg_set_lowbyte(0x1618, 0x80);
			mdelay(10);

			if (i == 0)
				reg_set16bit_value(0x1600, 0x0000);
			else if (i == 1)
				reg_set16bit_value(0x1600, 0x8000);
			else if (i == 2)
				reg_set16bit_value(0x1600, 0xA000);

			/* Sector erase */
			reg_set16bit_value(0x160E,
				(reg_get16bit_value(0x160E) | BIT2));

			LOGTP_DBUG("Wait erase done flag\n");

			do  {/* Wait erase done flag */
				/* Memory status */
				n_reg_data = reg_get16bit_value(0x1610);
				mdelay(50);
			} while ((n_reg_data & BIT1) != BIT1);
		}
		} else if (e_mem_type == EMEM_INFO) {/* 512Byte */
		LOGTP_DBUG("Erase info block\n");

		/* Clear pce */
		reg_set_lowbyte(0x1618, 0x80);
		mdelay(10);

		reg_set16bit_value(0x1600, 0xC000);

		/* Sector erase */
		reg_set16bit_value(0x160E, (reg_get16bit_value(0x160E) | BIT2));

		LOGTP_DBUG("Wait erase done flag\n");

		do  {/* Wait erase done flag */
			/* Memory status */
			n_reg_data = reg_get16bit_value(0x1610);
			mdelay(50);
		} while ((n_reg_data & BIT1) != BIT1);
	}

	LOGTP_DBUG("Erase end\n");

	bus_i2c_notuse_buf();
	buf_not_stop_mcu();
	bus_exit_serial_debugmode();
}

/* For MSG22XX */
static void _fwctrl_msg22_programemem(enum emem_type_e e_mem_type)
{
	u32 i, j;
	u32 n_remain_size = 0, n_blocksize = 0, n_size = 0, index = 0;
	u16 n_reg_data;
	u8 sz_dbbus_txdata[1024] = { 0 };
#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM))
	u32 n_size_perwrite = 125;
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
	u32 n_size_perwrite = 1021;
#endif

	LOGTP_DBUG(" %s() e_mem_type = %d\n", __func__, e_mem_type);

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_alloc();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();

	/* Hold reset pin before program */
	reg_set_lowbyte(0x1E06, 0x00);

	LOGTP_DBUG("Program start\n");

	reg_set16bit_value(0x161A, 0xABBA);
	reg_set16bit_value(0x1618, (reg_get16bit_value(0x1618) | 0x80));

	if (e_mem_type == EMEM_MAIN) {
		LOGTP_DBUG("Program main block\n");

		/* Set start address of main block */
		reg_set16bit_value(0x1600, 0x0000);
		/* 48KB */
		n_remain_size = MSG22XX_FM_BLOCKSIZE * 1024;
		index = 0;
	} else if (e_mem_type == EMEM_INFO) {
		LOGTP_DBUG("Program info block\n");

		/* Set start address of info block */
		reg_set16bit_value(0x1600, 0xC000);
		n_remain_size = MSG22XX_FIRMWARE_INFO_BLOCK_SIZE;/* 512Byte */
		index = MSG22XX_FM_BLOCKSIZE * 1024;
	} else {
		LOGTP_DBUG("e_mem_type = %d is not supported.\n",
	e_mem_type);
		return;
	}

	/* Enable burst mode */
	reg_set16bit_value(0x160C, (reg_get16bit_value(0x160C) | 0x01));

	/* Program start */
	sz_dbbus_txdata[0] = 0x10;
	sz_dbbus_txdata[1] = 0x16;
	sz_dbbus_txdata[2] = 0x02;

	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 3);

	sz_dbbus_txdata[0] = 0x20;

	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 1);

	i = 0;

	while (n_remain_size > 0) {
		if (n_remain_size > n_size_perwrite)
			n_blocksize = n_size_perwrite;
		else
			n_blocksize = n_remain_size;


		sz_dbbus_txdata[0] = 0x10;
		sz_dbbus_txdata[1] = 0x16;
		sz_dbbus_txdata[2] = 0x02;

		n_size = 3;

		for (j = 0; j < n_blocksize; j++) {
			sz_dbbus_txdata[3 + j] =
	_g_one_fwdata[index + (i * n_size_perwrite) + j];
			n_size++;
		}
		i++;

		i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], n_size);

		n_remain_size = n_remain_size - n_blocksize;
	}

	/* Program end */
	sz_dbbus_txdata[0] = 0x21;

	i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 1);

	n_reg_data = reg_get16bit_value(0x160C);
	reg_set16bit_value(0x160C, n_reg_data & (~0x01));

	LOGTP_DBUG("Wait write done flag\n");

	/* Polling 0x1610 is 0x0002 */
	do {
		n_reg_data = reg_get16bit_value(0x1610);
		n_reg_data = n_reg_data & BIT1;
		mdelay(10);

	} while (n_reg_data != BIT1);/* Wait write done flag */

	LOGTP_DBUG("Program end\n");

	bus_i2c_notuse_buf();
	buf_not_stop_mcu();
	bus_exit_serial_debugmode();

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_free();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */
}
#endif

/* For MSG22XX */
static u32 _fwctrl_msg22xx_getcrc_byhardware(enum emem_type_e e_mem_type)
{
	u16 n_crc_down = 0;
	u32 n_retval = 0;

	LOGTP_DBUG(" %s() e_mem_type = %d\n", __func__, e_mem_type);

	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();
	mdelay(100);

	/* RIU password */
	reg_set16bit_value(0x161A, 0xABBA);

	/* Set PCE high */
	reg_set_lowbyte(0x1618, 0x40);

	if (e_mem_type == EMEM_MAIN) {
		/* Set start address and end address for main block */
		reg_set16bit_value(0x1600, 0x0000);
		reg_set16bit_value(0x1640, 0xBFF8);
	} else if (e_mem_type == EMEM_INFO) {
		/* Set start address and end address for info block */
		reg_set16bit_value(0x1600, 0xC000);
		reg_set16bit_value(0x1640, 0xC1F8);
	}
	/* CRC reset */
	reg_set16bit_value(0x164E, 0x0001);

	reg_set16bit_value(0x164E, 0x0000);

	/* Trigger CRC check */
	reg_set_lowbyte(0x160E, 0x20);
	mdelay(10);

	n_crc_down = reg_get16bit_value(0x164E);

	while (n_crc_down != 2) {
		LOGTP_DBUG("Wait CRC down\n");
		mdelay(10);
		n_crc_down = reg_get16bit_value(0x164E);
	}

	n_retval = reg_get16bit_value(0x1652);
	n_retval = (n_retval << 16) | reg_get16bit_value(0x1650);

	LOGTP_DBUG("Hardware CRC = 0x%x\n", n_retval);

	bus_i2c_notuse_buf();
	buf_not_stop_mcu();
	bus_exit_serial_debugmode();

	return n_retval;
}

static void _fwctrl_msg22_convert_twodimento_one(u8
	sz_twedimen_fwdata[]
	[1024],
	u8 *p_onedimen_fwdata)
{
	u32 i, j;

	LOGTP_FUNC();

	for (i = 0; i < (MSG22XX_FM_BLOCKSIZE + 1); i++) {
		if (i < MSG22XX_FM_BLOCKSIZE) {/* i < 48 */
			for (j = 0; j < 1024; j++)
				p_onedimen_fwdata[i * 1024 + j] =
					sz_twedimen_fwdata[i][j];

		} else {/* i == 48 */
			for (j = 0; j < 512; j++)
				p_onedimen_fwdata[i * 1024 + j] =
					sz_twedimen_fwdata[i][j];

		}
	}
}

static s32 _fw_ctrlparse_packet(u8 *p_packet, u16 n_len,
			struct touchinfo_t *p_info)
{
	u8 n_checksum = 0;
	u32 n_delta_x = 0, n_delta_y = 0;
	u32 n_x = 0;
	u32 n_y = 0;
#ifdef CONFIG_SWAP_X_Y
	u32 n_tmp_x;
	u32 n_tmp_y;
#endif
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
	u8 n_checksum_index = 0;
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

	LOGTP_FUNC();

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
	if (g_firmware_mode == FIRMWARE_MODE_DEMO_MODE)
		n_checksum_index = 7;
	else if (g_firmware_mode == FIRMWARE_MODE_DEBUG_MODE
	|| g_firmware_mode == FIRMWARE_MODE_RAW_DATA_MODE)
		n_checksum_index = 31;

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	if (g_gesture_wakeup_flag == 1)
		n_checksum_index = n_len - 1;

#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
	n_checksum = common_cal_checksum(&p_packet[0], n_checksum_index);
	LOGTP_DBUG("check sum[%x]==[%x]?\n", p_packet[n_checksum_index],
					 n_checksum);
#else
	n_checksum = common_cal_checksum(&p_packet[0], (n_len - 1));

	LOGTP_DBUG("check ksum[%x]==[%x]?\n", p_packet[n_len - 1], n_checksum);
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	if (g_gesture_wakeup_flag == 1) {
		u8 n_wakeup_mode = 0;
		u8 is_correct_format = 0;

		LOGTP_DBUG("raw data:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
		p_packet[0], p_packet[1], p_packet[2], p_packet[3],
		p_packet[4], p_packet[5], p_packet[6], p_packet[7]);

		if (g_chiptype == CHIP_TYPE_MSG22XX && p_packet[0] == 0xA7
			&& p_packet[1] == 0x00 && p_packet[2] == 0x06
			&& p_packet[3] == 0x50) {
			n_wakeup_mode = p_packet[4];
			is_correct_format = 1;
		} else if (g_chiptype == CHIP_TYPE_MSG21XXA
			&& p_packet[0] == 0x52 && p_packet[1] == 0xFF
			&& p_packet[2] == 0xFF && p_packet[3] == 0xFF
			&& p_packet[4] == 0xFF && p_packet[6] == 0xFF) {
			n_wakeup_mode = p_packet[5];
			is_correct_format = 1;
		}

		if (is_correct_format) {
			LOGTP_DBUG("n_wakeup_mode = 0x%x\n", n_wakeup_mode);

			switch (n_wakeup_mode) {
			case 0x58:
				_g_gesture_wakeup_value =
					GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG;

				LOGTP_DBUG("Light up screen by DOUBLE_CLICK\n");

				input_report_key(g_input_device, KEY_POWER, 1);
				input_sync(g_input_device);
				input_report_key(g_input_device, KEY_POWER, 0);
				input_sync(g_input_device);
				break;
			case 0x60:
				_g_gesture_wakeup_value =
					GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG;

				LOGTP_DBUG("Light up screen by UP_DIRECTp.\n");

				/*input_report_key(g_input_device,
				KEY_UP, 1);*/
				input_report_key(g_input_device, KEY_POWER, 1);
				input_sync(g_input_device);
				/* input_report_key(g_input_device,
				KEY_UP, 0);*/
				input_report_key(g_input_device, KEY_POWER, 0);
				input_sync(g_input_device);
				break;
			case 0x61:
				_g_gesture_wakeup_value =
					GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG;

				LOGTP_DBUG("Light up screen by DOWN_DIRECT\n");

				/*input_report_key(g_input_device,
				KEY_DOWN, 1);*/
				input_report_key(g_input_device, KEY_POWER, 1);
				input_sync(g_input_device);
				/*input_report_key(g_input_device,
				KEY_DOWN, 0);*/
				input_report_key(g_input_device, KEY_POWER, 0);
				input_sync(g_input_device);
				break;
			case 0x62:
				_g_gesture_wakeup_value =
					GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG;

				LOGTP_DBUG("Light up screen by LEFT_DIRECT\n");

				/*input_report_key(g_input_device,
				KEY_LEFT, 1); */
				input_report_key(g_input_device, KEY_POWER, 1);
				input_sync(g_input_device);
				/*input_report_key(g_input_device,
				 KEY_LEFT, 0); */
				input_report_key(g_input_device, KEY_POWER, 0);
				input_sync(g_input_device);
				break;
			case 0x63:
				_g_gesture_wakeup_value =
	GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG;

				LOGTP_DBUG("Light up screen by LEFT_DIRECT\n");

				/*input_report_key(g_input_device,
				 KEY_RIGHT, 1); */
				input_report_key(g_input_device, KEY_POWER, 1);
				input_sync(g_input_device);
				/*input_report_key(g_input_device,
				 KEY_RIGHT, 0); */
				input_report_key(g_input_device, KEY_POWER, 0);
				input_sync(g_input_device);
				break;
			case 0x64:
				_g_gesture_wakeup_value =
	GESTURE_WAKEUP_MODE_M_CHARACTER_FLAG;

				LOGTP_DBUG("Light up screen by m_CHARACTER.\n");

				/*input_report_key(g_input_device,
				 KEY_M, 1); */
				input_report_key(g_input_device, KEY_POWER, 1);
				input_sync(g_input_device);
				/*input_report_key(g_input_device,
				 KEY_M, 0); */
				input_report_key(g_input_device, KEY_POWER, 0);
				input_sync(g_input_device);
				break;
			case 0x65:
				_g_gesture_wakeup_value =
	GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG;

				LOGTP_DBUG("Light up screen by W_CHARACTER.\n");

				/*input_report_key(g_input_device,
				 KEY_W, 1); */
				input_report_key(g_input_device, KEY_POWER, 1);
				input_sync(g_input_device);
				/*input_report_key(g_input_device,
				 KEY_W, 0); */
				input_report_key(g_input_device, KEY_POWER, 0);
				input_sync(g_input_device);
				break;
			case 0x66:
				_g_gesture_wakeup_value =
	GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG;

				LOGTP_DBUG("Light up screen by C_CHARACTER.\n");

				/*input_report_key(g_input_device,
				 KEY_C, 1); */
				input_report_key(g_input_device, KEY_POWER, 1);
				input_sync(g_input_device);
				/*input_report_key(g_input_device,
				 KEY_C, 0); */
				input_report_key(g_input_device, KEY_POWER, 0);
				input_sync(g_input_device);
				break;
			case 0x67:
				_g_gesture_wakeup_value =
	GESTURE_WAKEUP_MODE_E_CHARACTER_FLAG;

				LOGTP_DBUG("Light up screen by e_CHARACTER.\n");

				/*input_report_key(g_input_device,
				 KEY_E, 1); */
				input_report_key(g_input_device, KEY_POWER, 1);
				input_sync(g_input_device);
				/*input_report_key(g_input_device,
				 KEY_E, 0); */
				input_report_key(g_input_device, KEY_POWER, 0);
				input_sync(g_input_device);
				break;
			case 0x68:
				_g_gesture_wakeup_value =
	GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG;

				LOGTP_DBUG("Light up screen by V_CHARACTER.\n");

				/*input_report_key(g_input_device,
				 KEY_V, 1); */
				input_report_key(g_input_device, KEY_POWER, 1);
				input_sync(g_input_device);
				/*input_report_key(g_input_device,
				 KEY_V, 0); */
				input_report_key(g_input_device, KEY_POWER, 0);
				input_sync(g_input_device);
				break;
			case 0x69:
				_g_gesture_wakeup_value =
	GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG;

				LOGTP_DBUG("Light up screen by O_CHARACTER.\n");

				/*input_report_key(g_input_device,
				 KEY_O, 1); */
				input_report_key(g_input_device, KEY_POWER, 1);
				input_sync(g_input_device);
				/*input_report_key(g_input_device,
				 KEY_O, 0); */
				input_report_key(g_input_device, KEY_POWER, 0);
				input_sync(g_input_device);
				break;
			case 0x6A:
				_g_gesture_wakeup_value =
	GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG;

				LOGTP_DBUG("Light up screen by S_CHARACTER.\n");

				/*input_report_key(g_input_device,
				 KEY_S, 1); */
				input_report_key(g_input_device, KEY_POWER, 1);
				input_sync(g_input_device);
				/*input_report_key(g_input_device,
				 KEY_S, 0); */
				input_report_key(g_input_device, KEY_POWER, 0);
				input_sync(g_input_device);
				break;
			case 0x6B:
				_g_gesture_wakeup_value =
	GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG;

				LOGTP_DBUG("Light up screen by Z_CHARACTER.\n");

				/*input_report_key(g_input_device,
				 KEY_Z, 1); */
				input_report_key(g_input_device, KEY_POWER, 1);
				input_sync(g_input_device);
				/*input_report_key(g_input_device,
				 KEY_Z, 0); */
				input_report_key(g_input_device, KEY_POWER, 0);
				input_sync(g_input_device);
				break;
			default:
				_g_gesture_wakeup_value = 0;
				break;
			}

			LOGTP_DBUG("_g_gesture_wakeup_value = 0x%x\n",
	_g_gesture_wakeup_value);
		} else {
			LOGTP_DBUG("gesture wakeup packet format incorrect\n");
		}

		return -1;
	}
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

	LOGTP_DBUG("raw data:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
		p_packet[0], p_packet[1], p_packet[2], p_packet[3],
		p_packet[4], p_packet[5], p_packet[6], p_packet[7]);

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
	/* check the checksum of packet */
	if ((p_packet[n_checksum_index] == n_checksum)
		&& (p_packet[0] == 0x52)) {
#else
		/* check the checksum of packet */
	if ((p_packet[n_len - 1] == n_checksum)
			&& (p_packet[0] == 0x52)) {
#endif			/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

		/* parse the packet to coordinate */
		n_x = (((p_packet[1] & 0xF0) << 4) | p_packet[2]);
		n_y = (((p_packet[1] & 0x0F) << 8) | p_packet[3]);

		n_delta_x = (((p_packet[4] & 0xF0) << 4) | p_packet[5]);
		n_delta_y = (((p_packet[4] & 0x0F) << 8) | p_packet[6]);

		LOGTP_DBUG("[x,y]=[%d,%d]\n", n_x, n_y);
		LOGTP_DBUG("[delta_x,delta_y]=[%d,%d]\n",
			n_delta_x, n_delta_y);

#ifdef CONFIG_SWAP_X_Y
		n_tmp_y = n_x;
		n_tmp_x = n_y;
		n_x = n_tmp_x;
		n_y = n_tmp_y;

		n_tmp_y = n_delta_x;
		n_tmp_x = n_delta_y;
		n_delta_x = n_tmp_x;
		n_delta_y = n_tmp_y;
#endif

#ifdef CONFIG_REVERSE_X
		n_x = 2047 - n_x;
		n_delta_x = 4095 - n_delta_x;
#endif

#ifdef CONFIG_REVERSE_Y
		n_y = 2047 - n_y;
		n_delta_y = 4095 - n_delta_y;
#endif

/*
* p_packet[0]:id,p_packet[1]~p_packet[3]:the first point abs,
p_packet[4]~p_packet[6]:the relative distance between the first
point abs and the second point abs
* when p_packet[1]~p_packet[4], p_packet[6] is 0xFF,keyevent,
p_packet[5] to judge which key press.
* p_packet[1]~p_packet[6] all are 0xFF, release touch*/
		if ((p_packet[1] == 0xFF) && (p_packet[2] == 0xFF)
			&& (p_packet[3] == 0xFF)
			&& (p_packet[4] == 0xFF)
			&& (p_packet[6] == 0xFF)) {
			p_info->t_point[0].n_x = 0;/* final X */
			p_info->t_point[0].n_y = 0;/* final Y */
			/* p_packet[5] is key value 0x00 is key up,
			 0xff is touch screen up */
			if ((p_packet[5] != 0x00) && (p_packet[5] !=
					0xFF)) {
				LOGTP_DBUG("touch key down p_packet[5]=%d\n",
					p_packet[5]);
				p_info->n_fingernum = 1;
				p_info->n_touch_keycode = p_packet[5];
				p_info->n_touch_keymode = 1;

#if  1
				p_info->n_fingernum = 1;
				p_info->n_touch_keycode = 0;
				p_info->n_touch_keymode = 0;

	if (p_packet[5] == 1) {/* TOUCH_KEY_MENU */
					p_info->t_point[0].n_x =
	g_virtualkey_dimlocal[0][0];
					p_info->t_point[0].n_y =
	g_virtualkey_dimlocal[0][1];
				/* TOUCH_KEY_BACK */
	} else if (p_packet[5] == 2) {
					p_info->t_point[0].n_x =
	g_virtualkey_dimlocal[2][0];
					p_info->t_point[0].n_y =
	g_virtualkey_dimlocal[2][1];
				/* TOUCH_KEY_HOME */
	} else if (p_packet[5] == 4) {
					p_info->t_point[0].n_x =
	g_virtualkey_dimlocal[1][0];
					p_info->t_point[0].n_y =
	g_virtualkey_dimlocal[1][1];
				} else {
					LOGTP_DBUG("multi-key is pressed.\n");
					return -1;
				}
#endif/* CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE */
			} else {/* key up or touch up */
				LOGTP_DBUG("touch end\n");
				p_info->n_fingernum = 0;/* touch end*/
				p_info->n_touch_keycode = 0;
				p_info->n_touch_keymode = 0;
			}
		} else {
			p_info->n_touch_keymode = 0;/*Touch on screen*/

			/* if ((n_delta_x == 0) && (n_delta_y == 0)) */
			if (
#ifdef CONFIG_REVERSE_X
			(n_delta_x == 4095)
#else
			(n_delta_x == 0)
#endif
				&&
#ifdef CONFIG_REVERSE_Y
			(n_delta_y == 4095)
#else
			(n_delta_y == 0)
#endif
			) {/* one touch point */
				p_info->n_fingernum = 1;/* one touch */
				p_info->t_point[0].n_x =
	(n_x * TOUCH_SCREEN_X_MAX) / TPD_WIDTH;
				p_info->t_point[0].n_y =
	(n_y * TOUCH_SCREEN_Y_MAX) / TPD_HEIGHT;
				LOGTP_DBUG("[%s]: [x,y]=[%d,%d]\n",
					__func__, n_x, n_y);
				LOGTP_DBUG("[%s]: point[x,y]=[%d,%d]\n",
				__func__, p_info->t_point[0].n_x,
				p_info->t_point[0].n_y);
			} else {/* two touch points */
				u32 n_x2, n_y2;

				p_info->n_fingernum = 2;/* two touch */
				/* Finger 1 */
				p_info->t_point[0].n_x =
	(n_x * TOUCH_SCREEN_X_MAX) / TPD_WIDTH;
				p_info->t_point[0].n_y =
	(n_y * TOUCH_SCREEN_Y_MAX) / TPD_HEIGHT;
				LOGTP_DBUG("[%s]: point1[x,y]=[%d,%d]\n",
				__func__, p_info->t_point[0].n_x,
				p_info->t_point[0].n_y);
				/* Finger 2 */
				if (n_delta_x > 2048)
					n_delta_x -= 4096;


				if (n_delta_y > 2048)
					n_delta_y -= 4096;


				n_x2 = (u32) (n_x + n_delta_x);
				n_y2 = (u32) (n_y + n_delta_y);

				p_info->t_point[1].n_x =
	(n_x2 * TOUCH_SCREEN_X_MAX) / TPD_WIDTH;
				p_info->t_point[1].n_y =
	(n_y2 * TOUCH_SCREEN_Y_MAX) / TPD_HEIGHT;
				LOGTP_DBUG("[%s]: point2[x,y]=[%d,%d]\n",
				__func__, p_info->t_point[1].n_x,
				p_info->t_point[1].n_y);
			}
		}
	}
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
		else if (p_packet[n_checksum_index] == n_checksum
			&& p_packet[0] == 0x62) {
			n_x = ((p_packet[1] << 8) | p_packet[2]);/*Position_X*/
			n_y = ((p_packet[3] << 8) | p_packet[4]);/*Position_Y*/
			/* Distance_X */
			n_delta_x = ((p_packet[13] << 8) | p_packet[14]);
			/* Distance_Y */
			n_delta_y = ((p_packet[15] << 8) | p_packet[16]);

			LOGTP_DBUG("[x,y]=[%d,%d]\n", n_x, n_y);
			LOGTP_DBUG("[delta_x,delta_y]=[%d,%d]\n",
				n_delta_x, n_delta_y);

#ifdef CONFIG_SWAP_X_Y
			n_tmp_y = n_x;
			n_tmp_x = n_y;
			n_x = n_tmp_x;
			n_y = n_tmp_y;

			n_tmp_y = n_delta_x;
			n_tmp_x = n_delta_y;
			n_delta_x = n_tmp_x;
			n_delta_y = n_tmp_y;
#endif

#ifdef CONFIG_REVERSE_X
			n_x = 2047 - n_x;
			n_delta_x = 4095 - n_delta_x;
#endif

#ifdef CONFIG_REVERSE_Y
			n_y = 2047 - n_y;
			n_delta_y = 4095 - n_delta_y;
#endif

/*
 * p_packet[0]:id, p_packet[1]~p_packet[4]:the first point abs,
   p_packet[13]~p_packet[16]:the relative distance between the
    first point abs and the second point abs
 * when p_packet[1]~p_packet[7] is 0xFF, keyevent, p_packet[8]
   to judge which key press.
* p_packet[1]~p_packet[8] all are 0xFF, release touch
*/
			if ((p_packet[1] == 0xFF) && (p_packet[2] == 0xFF)
		&& (p_packet[3] == 0xFF) && (p_packet[4] == 0xFF)
		&& (p_packet[5] == 0xFF) && (p_packet[6] == 0xFF)
		&& (p_packet[7] == 0xFF)) {
				p_info->t_point[0].n_x = 0;
				p_info->t_point[0].n_y = 0;
				/* p_packet[8] is key value 0x00 is key up,
				0xff is touch screen up */
				if ((p_packet[8] != 0x00) &&
					(p_packet[8] != 0xFF)) {
					LOGTP_DBUG("packet[8]=%d\n",
							p_packet[8]);
					p_info->n_fingernum = 1;
					p_info->n_touch_keycode = p_packet[8];
					p_info->n_touch_keymode = 1;

	#if  1
					p_info->n_fingernum = 1;
					p_info->n_touch_keycode = 0;
					p_info->n_touch_keymode = 0;

		if (p_packet[8] == 1) {/* TOUCH_KEY_MENU */
						p_info->t_point[0].n_x =
		g_virtualkey_dimlocal[0][0];
						p_info->t_point[0].n_y =
		g_virtualkey_dimlocal[0][1];
					/* TOUCH_KEY_BACK */
		} else if (p_packet[8] == 2) {
						p_info->t_point[0].n_x =
		g_virtualkey_dimlocal[2][0];
						p_info->t_point[0].n_y =
		g_virtualkey_dimlocal[2][1];
					/* TOUCH_KEY_HOME */
		} else if (p_packet[8] == 4) {
						p_info->t_point[0].n_x =
		g_virtualkey_dimlocal[1][0];
						p_info->t_point[0].n_y =
		g_virtualkey_dimlocal[1][1];
					} else {
						LOGTP_DBUG("multi-k pressed\n");
						return -1;
					}
	#endif/* CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE */
				} else {/* key up or touch up */
					LOGTP_DBUG("touch end\n");
					p_info->n_fingernum = 0;/* touch end */
					p_info->n_touch_keycode = 0;
					p_info->n_touch_keymode = 0;
				}
			} else {
				/* Touch on screen... */
				p_info->n_touch_keymode = 0;

				/*if ((n_delta_x == 0) && (n_delta_y == 0)) */
				if (
#ifdef CONFIG_REVERSE_X
				(n_delta_x == 4095)
#else
				(n_delta_x == 0)
#endif
					&&
#ifdef CONFIG_REVERSE_Y
				(n_delta_y == 4095)
#else
				(n_delta_y == 0)
#endif
				) {/* one touch point */
					p_info->n_fingernum = 1;/* one touch */
					p_info->t_point[0].n_x =
		(n_x * TOUCH_SCREEN_X_MAX) / TPD_WIDTH;
					p_info->t_point[0].n_y =
		(n_y * TOUCH_SCREEN_Y_MAX) / TPD_HEIGHT;
					LOGTP_DBUG("[%s]: [x,y]=[%d,%d]\n",
						 __func__, n_x, n_y);
					LOGTP_DBUG("[%s]: point[x,y]=[%d,%d]\n",
					__func__, p_info->t_point[0].n_x,
						p_info->t_point[0].n_y);
				} else {/* two touch points */
					u32 n_x2, n_y2;

					p_info->n_fingernum = 2;/* two touch */
					/* Finger 1 */
					p_info->t_point[0].n_x =
		(n_x * TOUCH_SCREEN_X_MAX) / TPD_WIDTH;
					p_info->t_point[0].n_y =
		(n_y * TOUCH_SCREEN_Y_MAX) / TPD_HEIGHT;
					LOGTP_DBUG("[%s]:point1[x,y]=[%d,%d]\n",
					__func__, p_info->t_point[0].n_x,
					p_info->t_point[0].n_y);
					/* Finger 2 */
					if (n_delta_x > 2048)
						n_delta_x -= 4096;


					if (n_delta_y > 2048)
						n_delta_y -= 4096;


					n_x2 = (u32) (n_x + n_delta_x);
					n_y2 = (u32) (n_y + n_delta_y);

					p_info->t_point[1].n_x =
		(n_x2 * TOUCH_SCREEN_X_MAX) / TPD_WIDTH;
					p_info->t_point[1].n_y =
		(n_y2 * TOUCH_SCREEN_Y_MAX) / TPD_HEIGHT;
					LOGTP_DBUG("[%s]:point2[x,y]=[%d,%d]\n",
					__func__, p_info->t_point[1].n_x,
					p_info->t_point[1].n_y);
				}

	/* Notify android application to retrieve log data mode
	 packet from device driver by sysfs. */
				if (g_touch_kobj != NULL) {
					char *p_envp[2];
					s32 n_retval = 0;

					p_envp[0] = "STATUS=GET_PACKET";
					p_envp[1] = NULL;

					n_retval =
				kobject_uevent_env(g_touch_kobj, KOBJ_CHANGE,
					p_envp);
					LOGTP_DBUG("n_retval=%d\n", n_retval);
				}
			}
		} else {
			if (p_packet[n_checksum_index] != n_checksum) {
				LOGTP_DBUG("WRONG CHECKSUM\n");
				return -1;
			}

			if (g_firmware_mode ==
				FIRMWARE_MODE_DEMO_MODE
				&& p_packet[0] != 0x52) {
				LOGTP_DBUG("WRONG DEMO MODE HEADER\n");
				return -1;
			} else if (g_firmware_mode ==
				FIRMWARE_MODE_DEBUG_MODE
				&& p_packet[0] != 0x62) {
				LOGTP_DBUG("WRONG DEBUG MODE HEADER\n");
				return -1;
			} else if (g_firmware_mode ==
				FIRMWARE_MODE_RAW_DATA_MODE
				&& p_packet[0] != 0x62) {
				LOGTP_DBUG("WRONG RAW DATA MODE HEADER\n");
				return -1;
			}
		}
#else
	else {
		LOGTP_DBUG("packet[0]=0x%x, packet[7]=0x%x, checksum=0x%x\n",
				p_packet[0], p_packet[7], n_checksum);

		if (p_packet[n_len - 1] != n_checksum) {
			LOGTP_DBUG("WRONG CHECKSUM\n");
			return -1;
		}

		if (p_packet[0] != 0x52) {
			LOGTP_DBUG("WRONG DEMO MODE HEADER\n");
			return -1;
		}
	}
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

	return 0;
}

/*
 ------------------------------------------------------------------------------/
/ */

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID

static void _fw_store_fm_data(u8 *p_buf, u32 n_size)
{
	u32 n_count = n_size / 1024;
	u32 i;

	LOGTP_FUNC();

		if (n_count > 0) {/* n_size >= 1024 */
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

	LOGTP_DBUG(" g_fwdata_count = %d\n", g_fwdata_count);

	if (p_buf != NULL)
		LOGTP_DBUG(" buf[0] = %c\n", p_buf[0]);

}

/* -------------------------Start of SW ID for
 MSG22XX----------------------------// */

/* For MSG22XX */
static u32 _fwctrl_msg22xx_retrieve_crc_eflash(enum emem_type_e e_mem_type)
{
	u32 n_retval = 0;
	u16 n_reg_data1 = 0, n_reg_data2 = 0;

	LOGTP_DBUG(" %s() e_mem_type = %d\n", __func__, e_mem_type);

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

	/* Read main block CRC(48KB-4) from main block */
	if (e_mem_type == EMEM_MAIN) {
		/* Set start address for main block CRC */
		reg_set16bit_value(0x1600, 0xBFFC);
	/* Read info block CRC(512Byte-4) from info block */
	} else if (e_mem_type == EMEM_INFO) {
		/* Set start address for info block CRC */
		reg_set16bit_value(0x1600, 0xC1FC);
	}
	/* Enable burst mode */
	reg_set16bit_value(0x160C, (reg_get16bit_value(0x160C) | 0x01));

	/* Set pce */
	reg_set16bit_value(0x1618, (reg_get16bit_value(0x1618) | 0x40));

	reg_set_lowbyte(0x160E, 0x01);

	n_reg_data1 = reg_get16bit_value(0x1604);
	n_reg_data2 = reg_get16bit_value(0x1606);

	n_retval = ((n_reg_data2 >> 8) & 0xFF) << 24;
	n_retval |= (n_reg_data2 & 0xFF) << 16;
	n_retval |= ((n_reg_data1 >> 8) & 0xFF) << 8;
	n_retval |= (n_reg_data1 & 0xFF);

	LOGTP_DBUG("CRC = 0x%x\n", n_retval);

	/* Clear burst mode */
	reg_set16bit_value(0x160C, reg_get16bit_value(0x160C) & (~0x01));

	reg_set16bit_value(0x1600, 0x0000);

	/* Clear RIU password */
	reg_set16bit_value(0x161A, 0x0000);

	bus_i2c_notuse_buf();
	buf_not_stop_mcu();
	bus_exit_serial_debugmode();

	return n_retval;
}

static u32 _fwctrl_msg22xx_retrieve_crc_binfile(u8 sz_tmpbuf[],
				enum emem_type_e e_mem_type)
{
	u32 n_retval = 0;

	LOGTP_DBUG(" %s() e_mem_type = %d\n", __func__, e_mem_type);

	if (sz_tmpbuf != NULL) {
		/* Read main block CRC(48KB-4) from bin file */
		if (e_mem_type == EMEM_MAIN) {
			n_retval = sz_tmpbuf[0xBFFF] << 24;
			n_retval |= sz_tmpbuf[0xBFFE] << 16;
			n_retval |= sz_tmpbuf[0xBFFD] << 8;
			n_retval |= sz_tmpbuf[0xBFFC];
		/* Read info block CRC(512Byte-4) from bin file */
		} else if (e_mem_type == EMEM_INFO) {
			n_retval = sz_tmpbuf[0xC1FF] << 24;
			n_retval |= sz_tmpbuf[0xC1FE] << 16;
			n_retval |= sz_tmpbuf[0xC1FD] << 8;
			n_retval |= sz_tmpbuf[0xC1FC];
		}
	}

	return n_retval;
}

/* For MSG22XX */
static u16 _fw_msg22xx_getswid(enum emem_type_e e_mem_type)
{
	u16 n_retval = 0;
	u16 n_reg_data1 = 0;

	LOGTP_DBUG(" %s() e_mem_type = %d\n", __func__, e_mem_type);

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

	if (e_mem_type == EMEM_MAIN) {/* Read SW ID from main block */
		/* Set start address for main block SW ID */
		reg_set16bit_value(0x1600, 0xBFF4);
		/* Read SW ID from info block */
	} else if (e_mem_type == EMEM_INFO) {
		/* Set start address for info block SW ID */
		reg_set16bit_value(0x1600, 0xC1EC);
	}

	/*
			   Ex. SW ID in Main Block :
			   Major low byte at address 0xBFF4
			   Major high byte at address 0xBFF5

			   SW ID in Info Block :
			   Major low byte at address 0xC1EC
			   Major high byte at address 0xC1ED
			 */

	/* Enable burst mode */
/*    reg_set16bit_value(0x160C, (reg_get16bit_value(0x160C) | 0x01)); */

	/* Set pce */
	reg_set16bit_value(0x1618, (reg_get16bit_value(0x1618) | 0x40));

	reg_set_lowbyte(0x160E, 0x01);

	n_reg_data1 = reg_get16bit_value(0x1604);
/*    n_reg_data2 = reg_get16bit_value(0x1606); */

	n_retval = ((n_reg_data1 >> 8) & 0xFF) << 8;
	n_retval |= (n_reg_data1 & 0xFF);

	/* Clear burst mode */
/*    reg_set16bit_value(0x160C, reg_get16bit_value(0x160C) & (~0x01)); */

	reg_set16bit_value(0x1600, 0x0000);

	/* Clear RIU password */
	reg_set16bit_value(0x161A, 0x0000);

	LOGTP_DBUG("SW ID = 0x%x\n", n_retval);

	bus_i2c_notuse_buf();
	buf_not_stop_mcu();
	bus_exit_serial_debugmode();

	return n_retval;
}

static s32 _fwctrl_msg22_update_firmwareBySwId(void)/* For MSG22XX */
{
	s32 n_retval = -1;
	u32 ncrc_infoa = 0, ncrc_infob = 0, n_crc_maina = 0, n_crc_mainb = 0;

	LOGTP_FUNC();

	LOGTP_DBUG("isupdateinfo_blockfirst = %d, isupdate_firmware = 0x%x\n",
			_g_isupdateinfo_blockfirst, _g_isupdate_firmware);

	_fwctrl_msg22_convert_twodimento_one(g_fwdata,
	_g_one_fwdata);

	if (_g_isupdateinfo_blockfirst == 1) {
		if ((_g_isupdate_firmware & 0x10) == 0x10) {
			_fwctrl_msg22erase_emem(EMEM_INFO);
			_fwctrl_msg22_programemem(EMEM_INFO);

			ncrc_infoa =
	_fwctrl_msg22xx_retrieve_crc_binfile
	(_g_one_fwdata, EMEM_INFO);
			ncrc_infob =
	_fwctrl_msg22xx_getcrc_byhardware
	(EMEM_INFO);

			LOGTP_DBUG("ncrc_infoa = 0x%x, ncrc_infob = 0x%x\n",
				ncrc_infoa, ncrc_infob);

			if (ncrc_infoa == ncrc_infob) {
				_fwctrl_msg22erase_emem(EMEM_MAIN);
				_fwctrl_msg22_programemem(EMEM_MAIN);

				n_crc_maina =
	_fwctrl_msg22xx_retrieve_crc_binfile
	(_g_one_fwdata, EMEM_MAIN);
				n_crc_mainb =
	_fwctrl_msg22xx_getcrc_byhardware
	(EMEM_MAIN);

				LOGTP_DBUG("n_crc_maina = 0x%x, mainb = 0x%x\n",
						n_crc_maina, n_crc_mainb);

				if (n_crc_maina == n_crc_mainb) {
					_g_isupdate_firmware = 0x00;
					n_retval = 0;
				} else {
					_g_isupdate_firmware = 0x01;
				}
			} else {
				_g_isupdate_firmware = 0x11;
			}
		} else if ((_g_isupdate_firmware & 0x01) == 0x01) {
			_fwctrl_msg22erase_emem(EMEM_MAIN);
			_fwctrl_msg22_programemem(EMEM_MAIN);

			n_crc_maina =
	_fwctrl_msg22xx_retrieve_crc_binfile
	(_g_one_fwdata, EMEM_MAIN);
			n_crc_mainb =
	_fwctrl_msg22xx_getcrc_byhardware
	(EMEM_MAIN);

			LOGTP_DBUG("n_crc_maina=0x%x, n_crc_mainb=0x%x\n",
					n_crc_maina, n_crc_mainb);

			if (n_crc_maina == n_crc_mainb) {
				_g_isupdate_firmware = 0x00;
				n_retval = 0;
			} else {
				_g_isupdate_firmware = 0x01;
			}
		}
		} else {/* _g_isupdateinfo_blockfirst == 0 */
		if ((_g_isupdate_firmware & 0x10) == 0x10) {
			_fwctrl_msg22erase_emem(EMEM_MAIN);
			_fwctrl_msg22_programemem(EMEM_MAIN);

			n_crc_maina =
	_fwctrl_msg22xx_retrieve_crc_binfile
	(_g_one_fwdata, EMEM_MAIN);
			n_crc_mainb =
	_fwctrl_msg22xx_getcrc_byhardware
	(EMEM_MAIN);

			LOGTP_DBUG("n_crc_maina=0x%x, n_crc_mainb=0x%x\n",
					n_crc_maina, n_crc_mainb);

			if (n_crc_maina == n_crc_mainb) {
				_fwctrl_msg22erase_emem(EMEM_INFO);
				_fwctrl_msg22_programemem(EMEM_INFO);

				ncrc_infoa =
	_fwctrl_msg22xx_retrieve_crc_binfile
	(_g_one_fwdata, EMEM_INFO);
				ncrc_infob =
	_fwctrl_msg22xx_getcrc_byhardware
	(EMEM_INFO);

				LOGTP_DBUG("ncrc_infoa=0x%x, ncrc_infob=0x%x\n",
	ncrc_infoa, ncrc_infob);

				if (ncrc_infoa == ncrc_infob) {
					_g_isupdate_firmware = 0x00;
					n_retval = 0;
				} else {
					_g_isupdate_firmware = 0x01;
				}
			} else {
				_g_isupdate_firmware = 0x11;
			}
		} else if ((_g_isupdate_firmware & 0x01) == 0x01) {
			_fwctrl_msg22erase_emem(EMEM_INFO);
			_fwctrl_msg22_programemem(EMEM_INFO);

			ncrc_infoa =
	_fwctrl_msg22xx_retrieve_crc_binfile
	(_g_one_fwdata, EMEM_INFO);
			ncrc_infob =
	_fwctrl_msg22xx_getcrc_byhardware
	(EMEM_INFO);

			LOGTP_DBUG("ncrc_infoa=0x%x, ncrc_infob=0x%x\n",
				ncrc_infoa, ncrc_infob);

			if (ncrc_infoa == ncrc_infob) {
				_g_isupdate_firmware = 0x00;
				n_retval = 0;
			} else {
				_g_isupdate_firmware = 0x01;
			}
		}
	}

	return n_retval;
}

void _fwctrl_msg22_check_swid(void)/* For MSG22XX */
{
	u32 n_crc_maina, ncrc_infoa, n_crc_mainb, ncrc_infob;
	u32 i;
	u16 n_update_major = 0, n_update_minor = 0;
	u16 n_major = 0, n_minor = 0;
	u8 *p_version = NULL;
	Msg22xxSwId_e e_swid = MSG22XX_SW_ID_UNDEFINED;

	LOGTP_FUNC();

	platform_disable_finger_touchreport();

	fw_get_customer_firmware_version(&n_major, &n_minor, &p_version);

	n_crc_maina = _fwctrl_msg22xx_getcrc_byhardware(EMEM_MAIN);
	n_crc_mainb = _fwctrl_msg22xx_retrieve_crc_eflash(EMEM_MAIN);

	ncrc_infoa = _fwctrl_msg22xx_getcrc_byhardware(EMEM_INFO);
	ncrc_infob = _fwctrl_msg22xx_retrieve_crc_eflash(EMEM_INFO);

	_g_update_byswid_workqueue =
	create_singlethread_workqueue("update_firmware_by_sw_id");
	INIT_WORK(&_g_update_byswid_work,
	_fw_update_firmware_byswid_dowork);

	LOGTP_DBUG("n_crc_maina=0x%x, infoa=0x%x, mainb=0x%x, infob=0x%x\n",
	n_crc_maina, ncrc_infoa, n_crc_mainb, ncrc_infob);

	/* Case 1. Main Block:OK, Info Block:OK */
	if (n_crc_maina == n_crc_mainb && ncrc_infoa == ncrc_infob) {
		e_swid = _fw_msg22xx_getswid(EMEM_MAIN);

		if (e_swid == MSG22XX_SW_ID_XXXX) {
			n_update_major =
				msg2xxx_update_bin[0xBFF5] <<
				8 | msg2xxx_update_bin[0xBFF4];
			n_update_minor =
				msg2xxx_update_bin[0xBFF7] <<
				8 | msg2xxx_update_bin[0xBFF6];
		} else if (e_swid == MSG22XX_SW_ID_YYYY) {
			n_update_major =
				msg2xxx_yyyy_update_bin[0xBFF5] <<
				8 | msg2xxx_yyyy_update_bin[0xBFF4];
			n_update_minor =
				msg2xxx_yyyy_update_bin[0xBFF7] <<
				 8 | msg2xxx_yyyy_update_bin[0xBFF6];
				} else {/* e_swid == MSG22XX_SW_ID_UNDEFINED */
			LOGTP_DBUG("e_swid = 0x%x is an undefined SW ID.\n",
					e_swid);

			e_swid = MSG22XX_SW_ID_UNDEFINED;
			n_update_major = 0;
			n_update_minor = 0;
		}

		LOGTP_DBUG("e_swid=0x%x, n_major=%d, n_minor=%d\n",
			e_swid, n_major, n_minor);
		LOGTP_DBUG("n_update_major=%d, n_update_minor=%d\n",
			n_update_major, n_update_minor);

		if (n_update_minor > n_minor) {
			if (e_swid < MSG22XX_SW_ID_UNDEFINED &&
				e_swid != 0x0000 && e_swid != 0xFFFF) {
				if (e_swid == MSG22XX_SW_ID_XXXX &&
					i < MSG22XX_FM_BLOCKSIZE) {
					for (i = 0; i < (MSG22XX_FM_BLOCKSIZE
						+ 1); i++)
						_fw_store_fm_data
					(&(msg2xxx_update_bin[i * 1024]), 1024);
				} else if (e_swid == MSG22XX_SW_ID_XXXX) {
					/* i == 48 */
					for (i = 0; i < (MSG22XX_FM_BLOCKSIZE
						+ 1); i++)
						_fw_store_fm_data
					(&(msg2xxx_update_bin[i * 1024]), 512);
				} else if (e_swid == MSG22XX_SW_ID_YYYY &&
					i < MSG22XX_FM_BLOCKSIZE) {
					for (i = 0; i <	(MSG22XX_FM_BLOCKSIZE
						+ 1); i++)
						_fw_store_fm_data
					(&(msg2xxx_yyyy_update_bin[i * 1024]),
						1024);
				} else if (e_swid == MSG22XX_SW_ID_YYYY) {
					/* i == 48 */
					for (i = 0; i <	(MSG22XX_FM_BLOCKSIZE
						+ 1); i++)
						_fw_store_fm_data
					(&(msg2xxx_yyyy_update_bin[i * 1024]),
						512);
				}



				g_fwdata_count = 0;

				_g_update_retry_cnt =
					UPDATE_FIRMWARE_RETRY_COUNT;
				_g_isupdateinfo_blockfirst = 1;
				_g_isupdate_firmware = 0x11;
				queue_work(_g_update_byswid_workqueue,
					&_g_update_byswid_work);
				return;
			} else {
				LOGTP_DBUG("The sw id is invalid.\n");
				LOGTP_DBUG("Go to normal boot up process.\n");
			}
		} else {
			LOGTP_DBUG("The update bin version is older\n");
			LOGTP_DBUG("Go to normal boot up process.\n");
		}
	/* Case 2. Main Block:OK, Info Block:FAIL */
	} else if (n_crc_maina == n_crc_mainb &&
			ncrc_infoa != ncrc_infob) {
		e_swid = _fw_msg22xx_getswid(EMEM_MAIN);

		LOGTP_DBUG("e_swid=0x%x\n", e_swid);

		if (e_swid < MSG22XX_SW_ID_UNDEFINED && e_swid != 0x0000
	&& e_swid != 0xFFFF) {
			if (e_swid == MSG22XX_SW_ID_XXXX) {
				for (i = 0;
	i < (MSG22XX_FM_BLOCKSIZE + 1);
	i++) {
					if (i < MSG22XX_FM_BLOCKSIZE) {
						_fw_store_fm_data(&
	(msg2xxx_update_bin
	[i
	*
	1024]),
	1024);
			} else {/* i == 48 */
						_fw_store_fm_data(&
	(msg2xxx_update_bin
	[i
	*
	1024]),
	512);
					}
				}
			} else if (e_swid == MSG22XX_SW_ID_YYYY) {
				for (i = 0;
	i < (MSG22XX_FM_BLOCKSIZE + 1);
	i++) {
					if (i < MSG22XX_FM_BLOCKSIZE) {
						_fw_store_fm_data(&
	(msg2xxx_yyyy_update_bin
	[i
	*
	1024]),
	1024);
			} else {/* i == 48 */
						_fw_store_fm_data(&
	(msg2xxx_yyyy_update_bin
	[i
	*
	1024]),
	512);
					}
				}
			}

g_fwdata_count = 0;

			_g_update_retry_cnt = UPDATE_FIRMWARE_RETRY_COUNT;
			/* Set 1 for indicating main block is complete */
			_g_isupdateinfo_blockfirst = 1;
			_g_isupdate_firmware = 0x11;
			queue_work(_g_update_byswid_workqueue,
	&_g_update_byswid_work);
			return;
		} else {
			LOGTP_DBUG("The sw id is invalid.\n");
			LOGTP_DBUG("Go to normal boot up process.\n");
		}
	/* Case 3. Main Block:FAIL, Info Block:OK */
	} else if (n_crc_maina != n_crc_mainb &&
		ncrc_infoa == ncrc_infob) {
		e_swid = _fw_msg22xx_getswid(EMEM_INFO);

		LOGTP_DBUG("e_swid=0x%x\n", e_swid);

		if (e_swid < MSG22XX_SW_ID_UNDEFINED && e_swid != 0x0000
	&& e_swid != 0xFFFF) {
			if (e_swid == MSG22XX_SW_ID_XXXX) {
				for (i = 0;
	i < (MSG22XX_FM_BLOCKSIZE + 1);
	i++) {
					if (i < MSG22XX_FM_BLOCKSIZE) {
						_fw_store_fm_data(&
	(msg2xxx_update_bin
	[i
	*
	1024]),
	1024);
			} else {/* i == 48 */
						_fw_store_fm_data(&
	(msg2xxx_update_bin
	[i
	*
	1024]),
	512);
					}
				}
			} else if (e_swid == MSG22XX_SW_ID_YYYY) {
				for (i = 0;
	i < (MSG22XX_FM_BLOCKSIZE + 1);
	i++) {
					if (i < MSG22XX_FM_BLOCKSIZE) {
						_fw_store_fm_data(&
	(msg2xxx_yyyy_update_bin
	[i
	*
	1024]),
	1024);
			} else {/* i == 48 */
						_fw_store_fm_data(&
	(msg2xxx_yyyy_update_bin
	[i
	*
	1024]),
	512);
					}
				}
			}

g_fwdata_count = 0;

			_g_update_retry_cnt = UPDATE_FIRMWARE_RETRY_COUNT;
			/* Set 0 for indicating main block is broken */
			_g_isupdateinfo_blockfirst = 0;
			_g_isupdate_firmware = 0x11;
			queue_work(_g_update_byswid_workqueue,
	&_g_update_byswid_work);
			return;
		} else {
			LOGTP_DBUG("The sw id is invalid.\n");
			LOGTP_DBUG("Go to normal boot up process.\n");
		}
	} else {/* Case 4. Main Block:FAIL, Info Block:FAIL */
		LOGTP_DBUG("Main block and Info block are broken.\n");
		LOGTP_DBUG("Go to normal boot up process.\n");
	}

	platform_device_reset_hw();

	platform_enable_finger_touchreport();
}

/* -------------------------End of SW ID for
 MSG22XX----------------------------// */

/* -------------------------Start of SW ID for
 MSG21XXA----------------------------// */

static u32 fwctrl_msg21xxa_cal_crc_eflash(void)/* For MSG21XXA */
{
	u32 n_retval = 0;
	u16 n_reg_data = 0;

	LOGTP_FUNC();

	platform_device_reset_hw();
	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();
	mdelay(20);

	/* Stop mcu */
	reg_set_lowbyte(0x0FE6, 0x01);/* bank:mheg5, addr:h0073 */

	/* Stop Watchdog */
	reg_set16bit_value(0x3C60, 0xAA55);/* bank:reg_PIU_MISC_0, addr:h0030 */

	/* cmd */
	reg_set16bit_value(0x3CE4, 0xDF4C);/* bank:reg_PIU_MISC_0, addr:h0072 */

	/* TP SW reset */
	reg_set16bit_value(0x1E04, 0x7d60);/* bank:chip, addr:h0002 */
	reg_set16bit_value(0x1E04, 0x829F);

	/* Start mcu */
	reg_set_lowbyte(0x0FE6, 0x00);/* bank:mheg5, addr:h0073 */

	mdelay(20);

	/* Polling 0x3CE4 is 0x9432 */
	do {
		/* bank:reg_PIU_MISC_0, addr:h0072 */
		n_reg_data = reg_get16bit_value(0x3CE4);
	} while (n_reg_data != 0x9432);

	/* Read calculated main block CRC from register */
	n_retval = reg_get16bit_value(0x3C80);
	n_retval = (n_retval << 16) | reg_get16bit_value(0x3C82);

	LOGTP_DBUG("Main Block CRC = 0x%x\n", n_retval);

	return n_retval;
}

/* For MSG21XXA */
static u32 _DrvFwCtrlMsg21xxaRetrieveMainCrcFromMainBlock(void)
{
	u32 n_retval = 0;
	u16 n_reg_data = 0;
	u8 sz_dbbus_txdata[5] = { 0 };
	u8 szDbBusRxData[4] = { 0 };

	LOGTP_FUNC();

	platform_device_reset_hw();
	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();
	mdelay(20);

	/* Stop mcu */
	reg_set_lowbyte(0x0FE6, 0x01);/* bank:mheg5, addr:h0073 */

	/* Stop watchdog */
	reg_set16bit_value(0x3C60, 0xAA55);/* bank:reg_PIU_MISC_0, addr:h0030 */

	/* cmd */
	reg_set16bit_value(0x3CE4, 0xA4AB);/* bank:reg_PIU_MISC_0, addr:h0072 */

	/* TP SW reset */
	reg_set16bit_value(0x1E04, 0x7d60);/* bank:chip, addr:h0002 */
	reg_set16bit_value(0x1E04, 0x829F);

	/* Start mcu */
	reg_set_lowbyte(0x0FE6, 0x00);/* bank:mheg5, addr:h0073 */

	mdelay(20);

	/* Polling 0x3CE4 is 0x5B58 */
	do {
		/* bank:reg_PIU_MISC_0, addr:h0072 */
		n_reg_data = reg_get16bit_value(0x3CE4);
	} while (n_reg_data != 0x5B58);

	/* Read main block CRC from main block */
	sz_dbbus_txdata[0] = 0x72;
	sz_dbbus_txdata[1] = 0x7F;
	sz_dbbus_txdata[2] = 0xFC;
	sz_dbbus_txdata[3] = 0x00;
	sz_dbbus_txdata[4] = 0x04;

	i2c_write_data(SLAVE_I2C_ID_DWI2C, &sz_dbbus_txdata[0], 5);
	i2c_read_data(SLAVE_I2C_ID_DWI2C, &szDbBusRxData[0], 4);

	n_retval = szDbBusRxData[0];
	n_retval = (n_retval << 8) | szDbBusRxData[1];
	n_retval = (n_retval << 8) | szDbBusRxData[2];
	n_retval = (n_retval << 8) | szDbBusRxData[3];

	LOGTP_DBUG("CRC = 0x%x\n", n_retval);

	return n_retval;
}

/* For MSG21XXA */
static u16 _DrvFwCtrlMsg21xxaGetSwId(enum emem_type_e e_mem_type)
{
	u16 n_retval = 0;
	u16 n_reg_data = 0;
	u8 sz_dbbus_txdata[5] = { 0 };
	u8 szDbBusRxData[4] = { 0 };

	LOGTP_DBUG(" %s() e_mem_type = %d\n", __func__, e_mem_type);

	platform_device_reset_hw();
	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();
	mdelay(20);

	/* Stop mcu */
	reg_set_lowbyte(0x0FE6, 0x01);/* bank:mheg5, addr:h0073 */

	/* Stop watchdog */
	reg_set16bit_value(0x3C60, 0xAA55);/* bank:reg_PIU_MISC_0, addr:h0030 */

	/* cmd */
	reg_set16bit_value(0x3CE4, 0xA4AB);/* bank:reg_PIU_MISC_0, addr:h0072 */

	/* TP SW reset */
	reg_set16bit_value(0x1E04, 0x7d60);/* bank:chip, addr:h0002 */
	reg_set16bit_value(0x1E04, 0x829F);

	/* Start mcu */
	reg_set_lowbyte(0x0FE6, 0x00);/* bank:mheg5, addr:h0073 */

	mdelay(20);

	/* Polling 0x3CE4 is 0x5B58 */
	do {
		/* bank:reg_PIU_MISC_0, addr:h0072 */
		n_reg_data = reg_get16bit_value(0x3CE4);
	} while (n_reg_data != 0x5B58);

	sz_dbbus_txdata[0] = 0x72;
		if (e_mem_type == EMEM_MAIN) {/* Read SW ID from main block */
		sz_dbbus_txdata[1] = 0x7F;
		sz_dbbus_txdata[2] = 0x55;
		/* Read SW ID from info block */
		} else if (e_mem_type == EMEM_INFO) {
			sz_dbbus_txdata[1] = 0x83;
			sz_dbbus_txdata[2] = 0x00;
	}
	sz_dbbus_txdata[3] = 0x00;
	sz_dbbus_txdata[4] = 0x04;

	i2c_write_data(SLAVE_I2C_ID_DWI2C, &sz_dbbus_txdata[0], 5);
	i2c_read_data(SLAVE_I2C_ID_DWI2C, &szDbBusRxData[0], 4);

	LOGTP_DBUG("szDbBusRxData[0,1,2,3] = 0x%x,0x%x,0x%x,0x%x\n",
		szDbBusRxData[0], szDbBusRxData[1],
		szDbBusRxData[2], szDbBusRxData[3]);

	if ((szDbBusRxData[0] >= 0x30 && szDbBusRxData[0] <= 0x39)
	&& (szDbBusRxData[1] >= 0x30 && szDbBusRxData[1] <= 0x39)
	&& (szDbBusRxData[2] >= 0x31 && szDbBusRxData[2] <= 0x39)) {
		n_retval =
	(szDbBusRxData[0] - 0x30) * 100 + (szDbBusRxData[1] -
	0x30) * 10 +
	(szDbBusRxData[2] - 0x30);
	}

	LOGTP_DBUG("SW ID = 0x%x\n", n_retval);

	return n_retval;
}

static s32 _fwstrl_msg21xxa_update_byswid(u8 sz_fw_data[][1024],
			enum emem_type_e e_mem_type)
{
	u32 i, j, n_cal_crc_size;
	u32 n_crc_main = 0, ncrc_main_tp = 0;
	u32 ncrc_info = 0, n_crc_info_tp = 0;
	u32 n_crc_tmp = 0;
	u16 n_reg_data = 0;

	LOGTP_FUNC();

	n_crc_main = 0xffffffff;
	ncrc_info = 0xffffffff;

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_alloc();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	/* erase main */
	_fwctrl_erase_ememc33(EMEM_MAIN);
	mdelay(1000);

	platform_device_reset_hw();

	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();
	mdelay(300);

	/* /////////////////////// */
	/* Program */
	/* /////////////////////// */

	/* Polling 0x3CE4 is 0x1C70 */
	if ((e_mem_type == EMEM_ALL) || (e_mem_type == EMEM_MAIN)) {
		do {
			n_reg_data = reg_get16bit_value(0x3CE4);
		} while (n_reg_data != 0x1C70);
	}

	switch (e_mem_type) {
	case EMEM_ALL:
		reg_set16bit_value(0x3CE4, 0xE38F);/* for all blocks */
		break;
	case EMEM_MAIN:
		reg_set16bit_value(0x3CE4, 0x7731);/* for main block */
		break;
	case EMEM_INFO:
		reg_set16bit_value(0x3CE4, 0x7731);/* for info block */

		reg_set_lowbyte(0x0FE6, 0x01);

		reg_set_lowbyte(0x3CE4, 0xC5);
		reg_set_lowbyte(0x3CE5, 0x78);

		reg_set_lowbyte(0x1E04, 0x9F);
		reg_set_lowbyte(0x1E05, 0x82);

		reg_set_lowbyte(0x0FE6, 0x00);
		mdelay(100);
		break;
	}

	/* Polling 0x3CE4 is 0x2F43 */
	do {
		n_reg_data = reg_get16bit_value(0x3CE4);
	} while (n_reg_data != 0x2F43);

	/* Calculate CRC 32 */
	common_crc_inittable();

	if (e_mem_type == EMEM_ALL)
		n_cal_crc_size = MSG21XXA_FIRMWARE_WHOLE_SIZE;
	else if (e_mem_type == EMEM_MAIN)
		n_cal_crc_size = MSG21XXA_BLOCKSIZE;
	else if (e_mem_type == EMEM_INFO)
		n_cal_crc_size = MSG21XXA_FIRMWARE_INFO_BLOCK_SIZE;
	else
		n_cal_crc_size = 0;


	for (i = 0; i < n_cal_crc_size; i++) {
		if (e_mem_type == EMEM_INFO)
			i = 32;


		if (i < 32) {/* emem_main */
			if (i == 31) {
				sz_fw_data[i][1014] = 0x5A;
				sz_fw_data[i][1015] = 0xA5;

				for (j = 0; j < 1016; j++)
					n_crc_main =
					common_crc_getvalue(sz_fw_data[i][j],
					n_crc_main);


				n_crc_tmp = n_crc_main;
				n_crc_tmp = n_crc_tmp ^ 0xffffffff;

				/* add for debug */
				LOGTP_DBUG("n_crc_tmp=%x\n", n_crc_tmp);

				for (j = 0; j < 4; j++)
					sz_fw_data[i][1023 - j] =
					((n_crc_tmp >> (8 * j)) & 0xFF);


			} else {
				for (j = 0; j < 1024; j++)
					n_crc_main =
					common_crc_getvalue(sz_fw_data[i][j],
					n_crc_main);

			}
				} else {/* emem_info */
			for (j = 0; j < 1024; j++)
				ncrc_info =
				common_crc_getvalue(sz_fw_data[i][j],
				ncrc_info);


			if (e_mem_type == EMEM_MAIN)
				break;

		}

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM))
		for (j = 0; j < 8; j++)
			i2c_write_data(SLAVE_I2C_ID_DWI2C,
				&sz_fw_data[i][j * 128], 128);

#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
		i2c_write_data(SLAVE_I2C_ID_DWI2C, sz_fw_data[i], 1024);
#endif

		/* Polling 0x3CE4 is 0xD0BC */
		do {
			n_reg_data = reg_get16bit_value(0x3CE4);
		} while (n_reg_data != 0xD0BC);

		reg_set16bit_value(0x3CE4, 0x2F43);
	}

	if ((e_mem_type == EMEM_ALL) || (e_mem_type == EMEM_MAIN)) {
		/* write file done and check crc */
		reg_set16bit_value(0x3CE4, 0x1380);
	}
	mdelay(10);

	if ((e_mem_type == EMEM_ALL) || (e_mem_type == EMEM_MAIN)) {
		/* Polling 0x3CE4 is 0x9432 */
		do {
			n_reg_data = reg_get16bit_value(0x3CE4);
		} while (n_reg_data != 0x9432);
	}

	n_crc_main = n_crc_main ^ 0xffffffff;
	ncrc_info = ncrc_info ^ 0xffffffff;

	if ((e_mem_type == EMEM_ALL) || (e_mem_type == EMEM_MAIN)) {
		/* CRC Main from TP */
		ncrc_main_tp = reg_get16bit_value(0x3C80);
		ncrc_main_tp = (ncrc_main_tp << 16) |
			reg_get16bit_value(0x3C82);
	}

	if (e_mem_type == EMEM_ALL) {
		/* CRC Info from TP */
		n_crc_info_tp = reg_get16bit_value(0x3CA0);
		n_crc_info_tp = (n_crc_info_tp << 16) |
			reg_get16bit_value(0x3CA2);
	}

	LOGTP_DBUG("n_crc_main=0x%x, info=0x%x, maintp=0x%x, infotp=0x%x\n",
	n_crc_main, ncrc_info, ncrc_main_tp, n_crc_info_tp);

	/* Reset g_fwdata_count to 0 after update firmware */
	g_fwdata_count = 0;

	platform_device_reset_hw();

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_free();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	if ((e_mem_type == EMEM_ALL) || (e_mem_type == EMEM_MAIN)) {
		if (ncrc_main_tp != n_crc_main) {
			LOGTP_DBUG("Update FAILED\n");

			return -1;
		}
	}

	if (e_mem_type == EMEM_ALL) {
		if (n_crc_info_tp != ncrc_info) {
			LOGTP_DBUG("Update FAILED\n");

			return -1;
		}
	}

	LOGTP_DBUG("Update SUCCESS\n");

	return 0;
}

void _DrvFwCtrlMsg21xxaCheckFirmwareUpdateBySwId(void)/* For MSG21XXA */
{
	u32 n_crc_maina, n_crc_mainb;
	u32 i;
	u16 n_update_major = 0, n_update_minor = 0;
	u16 n_major = 0, n_minor = 0;
	u8 n_iscomare_version = 0;
	u8 *p_version = NULL;
	Msg21xxaSwId_e e_main_swid = MSG21XXA_SW_ID_UNDEFINED, e_info_swid =
	MSG21XXA_SW_ID_UNDEFINED, e_swid = MSG21XXA_SW_ID_UNDEFINED;

	LOGTP_FUNC();

	platform_disable_finger_touchreport();
	msleep(100);
	fw_get_customer_firmware_version(&n_major, &n_minor, &p_version);

	n_crc_maina = fwctrl_msg21xxa_cal_crc_eflash();
	n_crc_mainb = _DrvFwCtrlMsg21xxaRetrieveMainCrcFromMainBlock();

	_g_update_byswid_workqueue =
	create_singlethread_workqueue("update_firmware_by_sw_id");
	INIT_WORK(&_g_update_byswid_work,
	_fw_update_firmware_byswid_dowork);

	LOGTP_DBUG("n_crc_maina=0x%x, n_crc_mainb=0x%x\n",
			n_crc_maina, n_crc_mainb);

	if (n_crc_maina == n_crc_mainb) {
		e_main_swid = _DrvFwCtrlMsg21xxaGetSwId(EMEM_MAIN);
		e_info_swid = _DrvFwCtrlMsg21xxaGetSwId(EMEM_INFO);

		LOGTP_DBUG("Check firmware integrity success\n");
		LOGTP_DBUG("e_main_swid=0x%x, e_info_swid=0x%x\n",
			e_main_swid, e_info_swid);

		if (e_main_swid == e_info_swid) {
			e_swid = e_main_swid;
			n_iscomare_version = 1;
		} else {
			e_swid = e_info_swid;
			n_iscomare_version = 0;
		}

		if (e_swid == MSG21XXA_SW_ID_XXXX) {

#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY
			/* By two dimensional array */
			n_update_major =
	msg2xxx_update_bin[31][0x34F] << 8 |
	msg2xxx_update_bin[31][0x34E];
			n_update_minor =
	msg2xxx_update_bin[31][0x351] << 8 |
	msg2xxx_update_bin[31][0x350];
#else/* By one dimensional array */
			n_update_major =
	msg2xxx_update_bin[0x7F4F] << 8 |
	msg2xxx_update_bin[0x7F4E];
			n_update_minor =
	msg2xxx_update_bin[0x7F51] << 8 |
	msg2xxx_update_bin[0x7F50];
#endif
		} else if (e_swid == MSG21XXA_SW_ID_YYYY) {

#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY
			/* By two dimensional array */
			n_update_major =
	msg2xxx_yyyy_update_bin[31][0x34F] << 8 |
	msg2xxx_yyyy_update_bin[31][0x34E];
			n_update_minor =
	msg2xxx_yyyy_update_bin[31][0x351] << 8 |
	msg2xxx_yyyy_update_bin[31][0x350];
#else/* By one dimensional array */
			n_update_major =
	msg2xxx_yyyy_update_bin[0x7F4F] << 8 |
	msg2xxx_yyyy_update_bin[0x7F4E];
			n_update_minor =
	msg2xxx_yyyy_update_bin[0x7F51] << 8 |
	msg2xxx_yyyy_update_bin[0x7F50];
#endif
				} else {/* e_swid == MSG21XXA_SW_ID_UNDEFINED */
			LOGTP_DBUG("e_swid = 0x%x is an undefined SW ID.\n",
					e_swid);

			e_swid = MSG21XXA_SW_ID_UNDEFINED;
			n_update_major = 0;
			n_update_minor = 0;
		}
		/*
		LOGTP_DBUG("e_swid=0x%x, n_major=%d, n_minor=%d,
		n_update_major=%d, n_update_minor=%d\n",
		e_swid, n_major, n_minor, n_update_major,
		n_update_minor);
		*/
		if ((n_update_minor > n_minor && n_iscomare_version == 1)
			|| (n_iscomare_version == 0)) {
			if (e_swid < MSG21XXA_SW_ID_UNDEFINED &&
				e_swid != 0xFFFF) {
				if (e_swid == MSG21XXA_SW_ID_XXXX) {
					for (i = 0; i < MSG21XXA_BLOCKSIZE;
						i++) {

#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY
						/* By two dimensional array */
						_fw_store_fm_data
	(msg2xxx_update_bin[i],
	1024);
#else/* By one dimensional array */
						_fw_store_fm_data(&
	(msg2xxx_update_bin
	[i
	*
	1024]),
	1024);
#endif
					}
				} else if (e_swid == MSG21XXA_SW_ID_YYYY) {
					for (i = 0; i <	MSG21XXA_BLOCKSIZE;
						i++) {

#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY
						/* By two dimensional array */
						_fw_store_fm_data
	(msg2xxx_yyyy_update_bin[i],
	1024);
#else/* By one dimensional array */
						_fw_store_fm_data(&
	(msg2xxx_yyyy_update_bin
	[i
	*
	1024]),
	1024);
#endif
					}
				}

g_fwdata_count = 0;

				_g_update_retry_cnt =
	UPDATE_FIRMWARE_RETRY_COUNT;
				queue_work(_g_update_byswid_workqueue,
	&_g_update_byswid_work);
				return;
			} else {
				LOGTP_DBUG("The sw id is invalid.\n");
				LOGTP_DBUG("Go to normal boot up process.\n");
			}
		} else {
			LOGTP_DBUG("The update bin version is older\n");
			LOGTP_DBUG("Go to normal boot up process.\n");
		}
	} else {
		e_swid = _DrvFwCtrlMsg21xxaGetSwId(EMEM_INFO);

		LOGTP_DBUG("Check firmware integrity failed\n");
		LOGTP_DBUG("e_swid=0x%x\n", e_swid);

		if (e_swid < MSG21XXA_SW_ID_UNDEFINED && e_swid != 0xFFFF) {
			if (e_swid == MSG21XXA_SW_ID_XXXX) {
				for (i = 0; i < MSG21XXA_BLOCKSIZE; i++) {

#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY
					/* By two dimensional array */
					_fw_store_fm_data
					(msg2xxx_update_bin[i], 1024);
#else/* By one dimensional array */
					_fw_store_fm_data(&
						(msg2xxx_update_bin
						[i * 1024]), 1024);
#endif
				}
			} else if (e_swid == MSG21XXA_SW_ID_YYYY) {
				for (i = 0; i < MSG21XXA_BLOCKSIZE; i++) {

#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY
					/* By two dimensional array */
					_fw_store_fm_data
					(msg2xxx_yyyy_update_bin[i], 1024);
#else/* By one dimensional array */
					_fw_store_fm_data(&
						(msg2xxx_yyyy_update_bin
						[i * 1024]), 1024);
#endif
				}
			}

g_fwdata_count = 0;

			_g_update_retry_cnt = UPDATE_FIRMWARE_RETRY_COUNT;
			queue_work(_g_update_byswid_workqueue,
	&_g_update_byswid_work);
			return;
		} else {
			LOGTP_DBUG("The sw id is invalid.\n");
			LOGTP_DBUG("Go to normal boot up process.\n");
		}
	}

	platform_device_reset_hw();

	platform_enable_finger_touchreport();
}

/* -------------------------End of SW ID for
 MSG21XXA----------------------------// */

static void _fw_update_firmware_byswid_dowork(struct work_struct *p_work)
{
	s32 n_retval = 0;

	LOGTP_DBUG(" %s() _g_update_retry_cnt = %d\n", __func__,
	_g_update_retry_cnt);

	if (g_chiptype == CHIP_TYPE_MSG21XXA) {
		n_retval =
	_fwstrl_msg21xxa_update_byswid(g_fwdata, EMEM_MAIN);
	} else if (g_chiptype == CHIP_TYPE_MSG22XX) {
		n_retval = _fwctrl_msg22_update_firmwareBySwId();
	} else {
		LOGTP_DBUG("Chip type (%d) does not support\n", g_chiptype);

		platform_device_reset_hw();

		platform_enable_finger_touchreport();

		n_retval = -1;
		return;
	}

	LOGTP_DBUG(" update firmware by sw id result = %d\n", n_retval);

	if (n_retval == 0) {
		LOGTP_DBUG("update firmware by sw id success\n");

		platform_device_reset_hw();

		platform_enable_finger_touchreport();

		if (g_chiptype == CHIP_TYPE_MSG22XX) {
			_g_isupdateinfo_blockfirst = 0;
			_g_isupdate_firmware = 0x00;
		}
		} else {/* n_retval == -1 */
		_g_update_retry_cnt--;
		if (_g_update_retry_cnt > 0) {
			LOGTP_DBUG("_g_update_retry_cnt = %d\n",
					_g_update_retry_cnt);
			queue_work(_g_update_byswid_workqueue,
	&_g_update_byswid_work);
		} else {
			LOGTP_DBUG("update firmware by sw id failed\n");

			platform_device_reset_hw();

			platform_enable_finger_touchreport();

			if (g_chiptype == CHIP_TYPE_MSG22XX) {
				_g_isupdateinfo_blockfirst = 0;
				_g_isupdate_firmware = 0x00;
			}
		}
	}
}

#endif/* CONFIG_UPDATE_FIRMWARE_BY_SW_ID */

/*
 ------------------------------------------------------------------------------/
/ */

static void _DrvFwCtrlReadInfoC33(void)
{
	u8 sz_dbbus_txdata[5] = { 0 };
	u16 n_reg_data = 0;
#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM))
	u32 i;
#endif

	LOGTP_FUNC();

	mdelay(300);

	/* Stop Watchdog */
	reg_set_lowbyte(0x3C60, 0x55);
	reg_set_lowbyte(0x3C61, 0xAA);

	reg_set16bit_value(0x3CE4, 0xA4AB);

	reg_set16bit_value(0x1E04, 0x7d60);

	/* TP SW reset */
	reg_set16bit_value(0x1E04, 0x829F);
	mdelay(1);

	sz_dbbus_txdata[0] = 0x10;
	sz_dbbus_txdata[1] = 0x0F;
	sz_dbbus_txdata[2] = 0xE6;
	sz_dbbus_txdata[3] = 0x00;
	i2c_write_data(SLAVE_I2C_ID_DBBUS, sz_dbbus_txdata, 4);
	mdelay(100);

	do {
		n_reg_data = reg_get16bit_value(0x3CE4);
	} while (n_reg_data != 0x5B58);

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM))
	sz_dbbus_txdata[0] = 0x72;
	sz_dbbus_txdata[3] = 0x00;
	sz_dbbus_txdata[4] = 0x80;/* read 128 bytes */

	for (i = 0; i < 8; i++) {
		sz_dbbus_txdata[1] = 0x80 + (((i * 128) & 0xff00) >> 8);
		sz_dbbus_txdata[2] = (i * 128) & 0x00ff;

		i2c_write_data(SLAVE_I2C_ID_DWI2C, &sz_dbbus_txdata[0], 5);

		mdelay(50);

		/* Receive info data */
i2c_read_data(SLAVE_I2C_ID_DWI2C, &_g_dwi2c_info_data[i * 128], 128);
	}
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
	sz_dbbus_txdata[0] = 0x72;
	sz_dbbus_txdata[1] = 0x80;
	sz_dbbus_txdata[2] = 0x00;
	sz_dbbus_txdata[3] = 0x04;/* read 1024 bytes */
	sz_dbbus_txdata[4] = 0x00;

	i2c_write_data(SLAVE_I2C_ID_DWI2C, sz_dbbus_txdata, 5);

	mdelay(50);

	/* Receive info data */
	i2c_read_data(SLAVE_I2C_ID_DWI2C, &_g_dwi2c_info_data[0], 1024);
#endif
}

static s32 _fw_update_firmwareC32(u8 sz_fw_data[][1024],
	enum emem_type_e e_mem_type)
{
	u32 i, j;
	u32 n_crc_main, ncrc_main_tp;
	u32 ncrc_info, n_crc_info_tp;
	u32 n_crc_tmp;
	u16 n_reg_data = 0;

	LOGTP_FUNC();

	n_crc_main = 0xffffffff;
	ncrc_info = 0xffffffff;

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_alloc();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	/* /////////////////////// */
	/* Erase  all */
	/* /////////////////////// */
	_fwctrl_erase_ememc32();
	mdelay(1000);

	platform_device_reset_hw();

	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();
	mdelay(300);

	/* Reset watch dog */
	reg_set_lowbyte(0x3C60, 0x55);
	reg_set_lowbyte(0x3C61, 0xAA);

	/* /////////////////////// */
	/* Program */
	/* /////////////////////// */

	/* Polling 0x3CE4 is 0x1C70 */
	do {
		n_reg_data = reg_get16bit_value(0x3CE4);
	} while (n_reg_data != 0x1C70);

	reg_set16bit_value(0x3CE4, 0xE38F);/* for all-blocks */

	/* Polling 0x3CE4 is 0x2F43 */
	do {
		n_reg_data = reg_get16bit_value(0x3CE4);
	} while (n_reg_data != 0x2F43);

	/* Calculate CRC 32 */
	common_crc_inittable();

		for (i = 0; i < 33; i++) {/* total  33 KB : 2 byte per R/W */
				if (i < 32) {/* emem_main */
			if (i == 31) {
				sz_fw_data[i][1014] = 0x5A;
				sz_fw_data[i][1015] = 0xA5;

				for (j = 0; j < 1016; j++) {
					n_crc_main =
	common_crc_getvalue(sz_fw_data[i][j],
	n_crc_main);
				}

				n_crc_tmp = n_crc_main;
				n_crc_tmp = n_crc_tmp ^ 0xffffffff;

				/* add for debug */
				LOGTP_DBUG("n_crc_tmp=%x\n", n_crc_tmp);

				for (j = 0; j < 4; j++) {
					sz_fw_data[i][1023 - j] =
					((n_crc_tmp >> (8 * j)) & 0xFF);
/*
LOGTP_DBUG("((n_crc_tmp>>(8*%d)) & 0xFF)=%x\n",
j, ((n_crc_tmp >> (8 * j)) & 0xFF));
LOGTP_DBUG("Update main clock crc32 into bin buffer sz_fw_data[%d][%d]=%x\n",
i, (1020 + j), sz_fw_data[i][1020 + j]); */
				}
			} else {
				for (j = 0; j < 1024; j++) {
					n_crc_main =
	common_crc_getvalue(sz_fw_data[i][j],
	n_crc_main);
				}
			}
				} else {/* emem_info */
			for (j = 0; j < 1024; j++) {
				ncrc_info =
	common_crc_getvalue(sz_fw_data[i][j],
	ncrc_info);
			}
		}

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM))

		for (j = 0; j < 8; j++) {
			i2c_write_data(SLAVE_I2C_ID_DWI2C,
				&sz_fw_data[i][j * 128], 128);
		}
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
		i2c_write_data(SLAVE_I2C_ID_DWI2C, sz_fw_data[i], 1024);
#endif

		/* Polling 0x3CE4 is 0xD0BC */
		do {
			n_reg_data = reg_get16bit_value(0x3CE4);
		} while (n_reg_data != 0xD0BC);

		reg_set16bit_value(0x3CE4, 0x2F43);
	}

	/* Write file done */
	reg_set16bit_value(0x3CE4, 0x1380);

	mdelay(10);
	/* Polling 0x3CE4 is 0x9432 */
	do {
		n_reg_data = reg_get16bit_value(0x3CE4);
	} while (n_reg_data != 0x9432);

	n_crc_main = n_crc_main ^ 0xffffffff;
	ncrc_info = ncrc_info ^ 0xffffffff;

	/* CRC Main from TP */
	ncrc_main_tp = reg_get16bit_value(0x3C80);
	ncrc_main_tp = (ncrc_main_tp << 16) | reg_get16bit_value(0x3C82);

	/* CRC Info from TP */
	n_crc_info_tp = reg_get16bit_value(0x3CA0);
	n_crc_info_tp = (n_crc_info_tp << 16) | reg_get16bit_value(0x3CA2);

	LOGTP_DBUG("ncrc_main=0x%x,info=0x%x,maintp=0x%x,infotp=0x%x\n",
			n_crc_main, ncrc_info, ncrc_main_tp, n_crc_info_tp);

	/* Reset g_fwdata_count to 0 after update firmware */
	g_fwdata_count = 0;

	platform_device_reset_hw();

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_free();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	if ((ncrc_main_tp != n_crc_main) || (n_crc_info_tp != ncrc_info)) {
		LOGTP_DBUG("Update FAILED\n");

		return -1;
	}

	LOGTP_DBUG("Update SUCCESS\n");

	return 0;
}

static s32 _fw_update_firmwareC33(u8 sz_fw_data[][1024],
	enum emem_type_e e_mem_type)
{
	u8 szLifeCounter[2];
	u32 i, j;
	u32 n_crc_main, ncrc_main_tp;
	u32 ncrc_info, n_crc_info_tp;
	u32 n_crc_tmp;
	u16 n_reg_data = 0;

	LOGTP_FUNC();

	n_crc_main = 0xffffffff;
	ncrc_info = 0xffffffff;

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_alloc();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	_DrvFwCtrlReadInfoC33();

	if (_g_dwi2c_info_data[0] == 'M' && _g_dwi2c_info_data[1] == 'S'
	&& _g_dwi2c_info_data[2] == 'T' && _g_dwi2c_info_data[3] == 'A'
	&& _g_dwi2c_info_data[4] == 'R' && _g_dwi2c_info_data[5] == 'T'
	&& _g_dwi2c_info_data[6] == 'P' && _g_dwi2c_info_data[7] == 'C') {
		_g_dwi2c_info_data[8] = sz_fw_data[32][8];
		_g_dwi2c_info_data[9] = sz_fw_data[32][9];
		_g_dwi2c_info_data[10] = sz_fw_data[32][10];
		_g_dwi2c_info_data[11] = sz_fw_data[32][11];
		/* updata life counter */
		szLifeCounter[1] =
	((((_g_dwi2c_info_data[13] << 8) | _g_dwi2c_info_data[12]) +
	1) >> 8) & 0xFF;
		szLifeCounter[0] =
	(((_g_dwi2c_info_data[13] << 8) | _g_dwi2c_info_data[12]) +
	1) & 0xFF;
		_g_dwi2c_info_data[12] = szLifeCounter[0];
		_g_dwi2c_info_data[13] = szLifeCounter[1];

		reg_set16bit_value(0x3CE4, 0x78C5);
		reg_set16bit_value(0x1E04, 0x7d60);
		/* TP SW reset */
		reg_set16bit_value(0x1E04, 0x829F);

		mdelay(50);

		/* Polling 0x3CE4 is 0x2F43 */
		do {
			n_reg_data = reg_get16bit_value(0x3CE4);
		} while (n_reg_data != 0x2F43);

		/* Transmit lk info data */
#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM))
		for (j = 0; j < 8; j++) {
			i2c_write_data(SLAVE_I2C_ID_DWI2C,
	&_g_dwi2c_info_data[j * 128], 128);
		}
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
		i2c_write_data(SLAVE_I2C_ID_DWI2C,
				&_g_dwi2c_info_data[0], 1024);
#endif

		/* Polling 0x3CE4 is 0xD0BC */
		do {
			n_reg_data = reg_get16bit_value(0x3CE4);
		} while (n_reg_data != 0xD0BC);
	}
	/* erase main */
	_fwctrl_erase_ememc33(EMEM_MAIN);
	mdelay(1000);

	platform_device_reset_hw();

	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();
	mdelay(300);

	/* /////////////////////// */
	/* Program */
	/* /////////////////////// */

	/* Polling 0x3CE4 is 0x1C70 */
	if ((e_mem_type == EMEM_ALL) || (e_mem_type == EMEM_MAIN)) {
		do {
			n_reg_data = reg_get16bit_value(0x3CE4);
		} while (n_reg_data != 0x1C70);
	}

	switch (e_mem_type) {
	case EMEM_ALL:
		reg_set16bit_value(0x3CE4, 0xE38F);/* for all blocks */
		break;
	case EMEM_MAIN:
		reg_set16bit_value(0x3CE4, 0x7731);/* for main block */
		break;
	case EMEM_INFO:
		reg_set16bit_value(0x3CE4, 0x7731);/* for info block */

		reg_set_lowbyte(0x0FE6, 0x01);

		reg_set_lowbyte(0x3CE4, 0xC5);
		reg_set_lowbyte(0x3CE5, 0x78);

		reg_set_lowbyte(0x1E04, 0x9F);
		reg_set_lowbyte(0x1E05, 0x82);

		reg_set_lowbyte(0x0FE6, 0x00);
		mdelay(100);
		break;
	}

	/* Polling 0x3CE4 is 0x2F43 */
	do {
		n_reg_data = reg_get16bit_value(0x3CE4);
	} while (n_reg_data != 0x2F43);

	/* Calculate CRC 32 */
	common_crc_inittable();

		for (i = 0; i < 33; i++) {/* total 33 KB : 2 byte per R/W */
		if (e_mem_type == EMEM_INFO)
			i = 32;


				if (i < 32) {/* emem_main */
			if (i == 31) {
				sz_fw_data[i][1014] = 0x5A;
				sz_fw_data[i][1015] = 0xA5;

				for (j = 0; j < 1016; j++) {
					n_crc_main =
					common_crc_getvalue(sz_fw_data[i][j],
					n_crc_main);
				}

				n_crc_tmp = n_crc_main;
				n_crc_tmp = n_crc_tmp ^ 0xffffffff;

				/* add for debug */
				LOGTP_DBUG("n_crc_tmp=%x\n", n_crc_tmp);

				for (j = 0; j < 4; j++) {
					sz_fw_data[i][1023 - j] =
					((n_crc_tmp >> (8 * j)) & 0xFF);

				}
			} else {
				for (j = 0; j < 1024; j++)
					n_crc_main =
					common_crc_getvalue(sz_fw_data[i][j],
					n_crc_main);

			}
				} else {/* emem_info */
			for (j = 0; j < 1024; j++)
				ncrc_info =
				common_crc_getvalue(_g_dwi2c_info_data[j],
							ncrc_info);


			if (e_mem_type == EMEM_MAIN)
				break;

		}

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM))

		for (j = 0; j < 8; j++)
			i2c_write_data(SLAVE_I2C_ID_DWI2C,
				&sz_fw_data[i][j * 128], 128);

#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
		i2c_write_data(SLAVE_I2C_ID_DWI2C, sz_fw_data[i], 1024);
#endif

		/* Polling 0x3CE4 is 0xD0BC */
		do {
			n_reg_data = reg_get16bit_value(0x3CE4);
		} while (n_reg_data != 0xD0BC);

		reg_set16bit_value(0x3CE4, 0x2F43);
	}

	if ((e_mem_type == EMEM_ALL) || (e_mem_type == EMEM_MAIN)) {
		/* write file done and check crc */
		reg_set16bit_value(0x3CE4, 0x1380);
	}
	mdelay(10);

	if ((e_mem_type == EMEM_ALL) || (e_mem_type == EMEM_MAIN)) {
		/* Polling 0x3CE4 is 0x9432 */
		do {
			n_reg_data = reg_get16bit_value(0x3CE4);
		} while (n_reg_data != 0x9432);
	}

	n_crc_main = n_crc_main ^ 0xffffffff;
	ncrc_info = ncrc_info ^ 0xffffffff;

	if ((e_mem_type == EMEM_ALL) || (e_mem_type == EMEM_MAIN)) {
		/* CRC Main from TP */
		ncrc_main_tp = reg_get16bit_value(0x3C80);
		ncrc_main_tp = (ncrc_main_tp << 16) |
				reg_get16bit_value(0x3C82);

		/* CRC Info from TP */
		n_crc_info_tp = reg_get16bit_value(0x3CA0);
		n_crc_info_tp = (n_crc_info_tp << 16) |
				reg_get16bit_value(0x3CA2);
	}
	LOGTP_DBUG("ncrc_main=0x%x,info=0x%x,maintp=0x%x,infotp=0x%x\n",
			n_crc_main, ncrc_info, ncrc_main_tp, n_crc_info_tp);

	/* Reset g_fwdata_count to 0 after update firmware */
	g_fwdata_count = 0;

	platform_device_reset_hw();

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_free();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	if ((e_mem_type == EMEM_ALL) || (e_mem_type == EMEM_MAIN)) {
		if ((ncrc_main_tp != n_crc_main) ||
			 (n_crc_info_tp != ncrc_info)) {
			LOGTP_DBUG("Update FAILED\n");

			return -1;
		}
	}

	LOGTP_DBUG("Update SUCCESS\n");

	return 0;
}

static s32 _fwctrl_msg22_update_firmware(u8 sz_fw_data[][1024],
	enum emem_type_e e_mem_type)
{
	int ret = 0;
	u32 i, index;
	u32 n_crc_main, ncrc_main_tp;
	u32 ncrc_info, n_crc_info_tp;
	u32 n_remain_size, n_blocksize, n_size;
	u16 n_reg_data = 0;
	u8 *sz_dbbus_txdata = NULL;
	/* u8 sz_dbbus_txdata[1024] = { 0 }; */
#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM))
	u32 n_size_perwrite = 125;
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
	u32 n_size_perwrite = 1021;
#endif

	LOGTP_FUNC();

    sz_dbbus_txdata = kzalloc(1024, GFP_KERNEL);

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_alloc();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	_fwctrl_msg22_convert_twodimento_one(sz_fw_data,
	_g_one_fwdata);

	platform_device_reset_hw();

	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();

	LOGTP_DBUG("Erase start\n");

	/* Stop mcu */
	reg_set16bit_value(0x0FE6, 0x0001);

	/* Disable watchdog */
	reg_set_lowbyte(0x3C60, 0x55);
	reg_set_lowbyte(0x3C61, 0xAA);

	/* Set PROGRAM password */
	reg_set_lowbyte(0x161A, 0xBA);
	reg_set_lowbyte(0x161B, 0xAB);

	if (e_mem_type == EMEM_ALL) {/* 48KB + 512Byte */
		LOGTP_DBUG("Erase all block\n");

		/* Clear pce */
		reg_set_lowbyte(0x1618, 0x80);
		mdelay(100);

		/* Chip erase */
		reg_set16bit_value(0x160E, BIT3);

		LOGTP_DBUG("Wait erase done flag\n");

		do  {/* Wait erase done flag */
			/* Memory status*/
			n_reg_data = reg_get16bit_value(0x1610);
			mdelay(50);
		} while ((n_reg_data & BIT1) != BIT1);
	} else if (e_mem_type == EMEM_MAIN) {/* 48KB (32+8+8) */
		LOGTP_DBUG("Erase main block\n");

		for (i = 0; i < 3; i++) {
			/* Clear pce */
			reg_set_lowbyte(0x1618, 0x80);
			mdelay(10);

			if (i == 0)
				reg_set16bit_value(0x1600, 0x0000);
			else if (i == 1)
				reg_set16bit_value(0x1600, 0x8000);
			else if (i == 2)
				reg_set16bit_value(0x1600, 0xA000);

			/* Sector erase */
			reg_set16bit_value(0x160E,
			(reg_get16bit_value(0x160E) | BIT2));

			LOGTP_DBUG("Wait erase done flag\n");

			do  {/* Wait erase done flag */
				/* Memory status */
				n_reg_data = reg_get16bit_value(0x1610);
				mdelay(50);
			} while ((n_reg_data & BIT1) != BIT1);
		}
	} else if (e_mem_type == EMEM_INFO) {/* 512Byte */
		LOGTP_DBUG("Erase info block\n");

		/* Clear pce */
		reg_set_lowbyte(0x1618, 0x80);
		mdelay(10);

		reg_set16bit_value(0x1600, 0xC000);

		/* Sector erase */
		reg_set16bit_value(0x160E, (reg_get16bit_value(0x160E) | BIT2));

		LOGTP_DBUG("Wait erase done flag\n");

		do  {/* Wait erase done flag */
			/*Memory status*/
			n_reg_data = reg_get16bit_value(0x1610);
			mdelay(50);
		} while ((n_reg_data & BIT1) != BIT1);
	}

	LOGTP_DBUG("Erase end\n");

	/* Hold reset pin before program */
	reg_set_lowbyte(0x1E06, 0x00);

	/* /////////////////////// */
	/* Program */
	/* /////////////////////// */

		if (e_mem_type == EMEM_ALL || e_mem_type == EMEM_MAIN) {/*48KB*/
		LOGTP_DBUG("Program main block start\n");

		/* Program main block */
		reg_set16bit_value(0x161A, 0xABBA);
		reg_set16bit_value(0x1618, (reg_get16bit_value(0x1618) | 0x80));

		/* Set start address of main block */
		reg_set16bit_value(0x1600, 0x0000);
		/* Enable burst mode */
		reg_set16bit_value(0x160C, (reg_get16bit_value(0x160C) | 0x01));

		/* Program start */
		sz_dbbus_txdata[0] = 0x10;
		sz_dbbus_txdata[1] = 0x16;
		sz_dbbus_txdata[2] = 0x02;

		i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 3);

		sz_dbbus_txdata[0] = 0x20;

		i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 1);

		/* 48KB */
		n_remain_size = MSG22XX_FM_BLOCKSIZE * 1024;
		index = 0;

		while (n_remain_size > 0) {
			if (n_remain_size > n_size_perwrite)
				n_blocksize = n_size_perwrite;
			else
				n_blocksize = n_remain_size;


			sz_dbbus_txdata[0] = 0x10;
			sz_dbbus_txdata[1] = 0x16;
			sz_dbbus_txdata[2] = 0x02;

			n_size = 3;

			for (i = 0; i < n_blocksize; i++) {
				sz_dbbus_txdata[3 + i] =
	_g_one_fwdata[index * n_size_perwrite + i];
				n_size++;
			}
			index++;

			i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0],
	n_size);

			n_remain_size = n_remain_size - n_blocksize;
		}

		/* Program end */
		sz_dbbus_txdata[0] = 0x21;

		i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 1);

		n_reg_data = reg_get16bit_value(0x160C);
		reg_set16bit_value(0x160C, n_reg_data & (~0x01));

		LOGTP_DBUG("Wait main block write done flag\n");

		/* Polling 0x1610 is 0x0002 */
		do {
			n_reg_data = reg_get16bit_value(0x1610);
			n_reg_data = n_reg_data & BIT1;
			mdelay(10);

		} while (n_reg_data != BIT1);/* Wait write done flag */

		LOGTP_DBUG("Program main block end\n");
	}
	/* 512 Byte */
	if (e_mem_type == EMEM_ALL || e_mem_type == EMEM_INFO) {
		LOGTP_DBUG("Program info block start\n");

		/* Program info block */
		reg_set16bit_value(0x161A, 0xABBA);
		reg_set16bit_value(0x1618, (reg_get16bit_value(0x1618) | 0x80));

		/* Set start address of info block */
		reg_set16bit_value(0x1600, 0xC000);
		/* Enable burst mode */
		reg_set16bit_value(0x160C, (reg_get16bit_value(0x160C) | 0x01));

		/* Program start */
		sz_dbbus_txdata[0] = 0x10;
		sz_dbbus_txdata[1] = 0x16;
		sz_dbbus_txdata[2] = 0x02;

		i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 3);

		sz_dbbus_txdata[0] = 0x20;

		i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 1);

		n_remain_size = MSG22XX_FIRMWARE_INFO_BLOCK_SIZE;/* 512Byte */
		index = 0;

		while (n_remain_size > 0) {
			if (n_remain_size > n_size_perwrite)
				n_blocksize = n_size_perwrite;
			else
				n_blocksize = n_remain_size;


			sz_dbbus_txdata[0] = 0x10;
			sz_dbbus_txdata[1] = 0x16;
			sz_dbbus_txdata[2] = 0x02;

			n_size = 3;

			for (i = 0; i < n_blocksize; i++) {
				sz_dbbus_txdata[3 + i] =
				_g_one_fwdata[(MSG22XX_FM_BLOCKSIZE
				* 1024) + (index * n_size_perwrite) + i];
				n_size++;
			}
			index++;

			i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0],
					n_size);

			n_remain_size = n_remain_size - n_blocksize;
		}

		/* Program end */
		sz_dbbus_txdata[0] = 0x21;

		i2c_write_data(SLAVE_I2C_ID_DBBUS, &sz_dbbus_txdata[0], 1);

		n_reg_data = reg_get16bit_value(0x160C);
		reg_set16bit_value(0x160C, n_reg_data & (~0x01));

		LOGTP_DBUG("Wait info block write done flag\n");

		/* Polling 0x1610 is 0x0002 */
		do {
			n_reg_data = reg_get16bit_value(0x1610);
			n_reg_data = n_reg_data & BIT1;
			mdelay(10);

		} while (n_reg_data != BIT1);/* Wait write done flag */

		LOGTP_DBUG("Program info block end\n");
	}

	if (e_mem_type == EMEM_ALL || e_mem_type == EMEM_MAIN) {
		/* Get CRC 32 from updated firmware bin file */
		n_crc_main = _g_one_fwdata[0xBFFF] << 24;
		n_crc_main |= _g_one_fwdata[0xBFFE] << 16;
		n_crc_main |= _g_one_fwdata[0xBFFD] << 8;
		n_crc_main |= _g_one_fwdata[0xBFFC];

		/* CRC Main from TP */
		LOGTP_DBUG("Get Main CRC from TP\n");

		ncrc_main_tp =
	_fwctrl_msg22xx_getcrc_byhardware(EMEM_MAIN);

		LOGTP_DBUG("n_crc_main=0x%x, ncrc_main_tp=0x%x\n", n_crc_main,
					ncrc_main_tp);
	}

	if (e_mem_type == EMEM_ALL || e_mem_type == EMEM_INFO) {
		ncrc_info = _g_one_fwdata[0xC1FF] << 24;
		ncrc_info |= _g_one_fwdata[0xC1FE] << 16;
		ncrc_info |= _g_one_fwdata[0xC1FD] << 8;
		ncrc_info |= _g_one_fwdata[0xC1FC];

		/* CRC Info from TP */
		LOGTP_DBUG("Get Info CRC from TP\n");

		n_crc_info_tp =
	_fwctrl_msg22xx_getcrc_byhardware(EMEM_INFO);

		LOGTP_DBUG("ncrc_info=0x%x, ncrc_infotp=0x%x\n", ncrc_info,
				n_crc_info_tp);
	}

	/* Reset g_fwdata_count to 0 after update firmware */
	g_fwdata_count = 0;

	bus_i2c_notuse_buf();
	buf_not_stop_mcu();
	bus_exit_serial_debugmode();

	platform_device_reset_hw();

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_free();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif/* CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	if (e_mem_type == EMEM_ALL) {
		if ((ncrc_main_tp != n_crc_main)
			|| (n_crc_info_tp != ncrc_info)) {
			LOGTP_ERRO("Update FAILED\n");

			ret = -1;
			goto FAIL;
		}
	} else if (e_mem_type == EMEM_MAIN) {
		if (ncrc_main_tp != n_crc_main) {
			LOGTP_ERRO("Update FAILED\n");

			ret = -1;
			goto FAIL;
		}
	} else if (e_mem_type == EMEM_INFO) {
		if (n_crc_info_tp != ncrc_info) {
			LOGTP_ERRO("Update FAILED\n");

			ret = -1;
			goto FAIL;
		}
	}

	LOGTP_DBUG("Update SUCCESS\n");

FAIL:
    kfree(sz_dbbus_txdata);
	return ret;
}

static s32 _fw_update_firmware_cash(u8 sz_fw_data[][1024])
{
	LOGTP_FUNC();

	LOGTP_DBUG("chip type = 0x%x\n", g_chiptype);

	if (g_chiptype == CHIP_TYPE_MSG21XXA) {/* (0x02) */
/*        u16 n_chiptype; */
		u8 n_chipversion = 0;

		platform_device_reset_hw();

		/* Erase TP Flash first */
		bus_enter_serial_debugmode();
		bus_stop_mcu();
		bus_i2c_use_bus();
		bus_i2c_reshape();
		mdelay(300);

		/* Stop MCU */
		reg_set_lowbyte(0x0FE6, 0x01);

		/* Disable watchdog */
		reg_set16bit_value(0x3C60, 0xAA55);

		/* /////////////////////// */
		/* Difference between C2 and C3 */
		/* /////////////////////// */
		/* c2:MSG2133(1) c32:MSG2133A(2) c33:MSG2138A(2) */
		/* check ic type */
/*        n_chiptype = reg_get16bit_value(0x1ECC) & 0xFF; */

		/* check ic version */
		n_chipversion = reg_get16bit_value(0x3CEA) & 0xFF;

		LOGTP_DBUG("chip version = 0x%x\n", n_chipversion);

		if (n_chipversion == 3) {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
			return _fwstrl_msg21xxa_update_byswid(
				sz_fw_data, EMEM_MAIN);
#else
			return _fw_update_firmwareC33(sz_fw_data, EMEM_MAIN);
#endif
		} else {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
return _fwstrl_msg21xxa_update_byswid(sz_fw_data,
	EMEM_MAIN);
#else
			return _fw_update_firmwareC32(sz_fw_data, EMEM_ALL);
#endif
		}
		} else if (g_chiptype == CHIP_TYPE_MSG22XX) {/* (0x7A) */
			return _fwctrl_msg22_update_firmware(sz_fw_data,
						 EMEM_ALL);
		} else {/* CHIP_TYPE_MSG21XX (0x01) */
			LOGTP_DBUG("Can not update firmware.\n");
			/* Reset g_fwdata_count to 0 after update firmware */
			g_fwdata_count = 0;

		return -1;
	}
}

/*=============================================================*/
/* GLOBAL FUNCTION DEFINITION */
/*=============================================================*/

u8 fw_ctrl_get_chiptype(void)
{
	u8 n_chiptype = 0;

	LOGTP_FUNC();

	/* platform_device_reset_hw(); */

	/* Erase TP Flash first */
	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();
	mdelay(20);

	/* Stop MCU */
	reg_set_lowbyte(0x0FE6, 0x01);

	/* Disable watchdog */
	reg_set16bit_value(0x3C60, 0xAA55);

	/* /////////////////////// */
	/* Difference between C2 and C3 */
	/* /////////////////////// */
	/* c2:MSG2133(1) c32:MSG2133A(2) c33:MSG2138A(2) */
	/* check ic type */
	n_chiptype = reg_get16bit_value(0x1ECC) & 0xFF;

	if (n_chiptype != CHIP_TYPE_MSG21XX &&/* (0x01) */
	n_chiptype != CHIP_TYPE_MSG21XXA &&/* (0x02) */
	n_chiptype != CHIP_TYPE_MSG26XXM &&/* (0x03) */
	n_chiptype != CHIP_TYPE_MSG22XX)/* (0x7A) */
		n_chiptype = 0;

	LOGTP_DBUG(" Chip Type = 0x%x\n", n_chiptype);

	platform_device_reset_hw();

	return n_chiptype;
}

void fw_get_customer_firmware_version(u16 *p_major, u16 *p_minor,
	u8 **pp_version)
{
	LOGTP_FUNC();

	if (g_chiptype == CHIP_TYPE_MSG21XXA
		|| g_chiptype == CHIP_TYPE_MSG21XX) {
		u8 sz_dbbus_txdata[3] = { 0 };
		u8 szDbBusRxData[4] = { 0 };

		sz_dbbus_txdata[0] = 0x53;
		sz_dbbus_txdata[1] = 0x00;

		if (g_chiptype == CHIP_TYPE_MSG21XXA)
			sz_dbbus_txdata[2] = 0x2A;
		else if (g_chiptype == CHIP_TYPE_MSG21XX)
			sz_dbbus_txdata[2] = 0x74;
		else
			sz_dbbus_txdata[2] = 0x2A;


		mutex_lock(&g_mutex);

		i2c_write_data(SLAVE_I2C_ID_DWI2C, &sz_dbbus_txdata[0], 3);
		i2c_read_data(SLAVE_I2C_ID_DWI2C, &szDbBusRxData[0], 4);

		mutex_unlock(&g_mutex);

		*p_major = (szDbBusRxData[1] << 8) + szDbBusRxData[0];
		*p_minor = (szDbBusRxData[3] << 8) + szDbBusRxData[2];
	} else if (g_chiptype == CHIP_TYPE_MSG22XX) {
		u16 n_reg_data1, n_reg_data2;

		mutex_lock(&g_mutex);

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

		*p_major = (((n_reg_data1 >> 8) & 0xFF) << 8) +
				(n_reg_data1 & 0xFF);
		*p_minor = (((n_reg_data2 >> 8) & 0xFF) << 8) +
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

		mutex_unlock(&g_mutex);
	}

	LOGTP_DBUG(" major = %d\n", *p_major);
	LOGTP_DBUG(" minor = %d\n", *p_minor);

	if (*pp_version == NULL)
		*pp_version = kzalloc(sizeof(u8) * 6, GFP_KERNEL);


	sprintf(*pp_version, "%03d%03d", *p_major, *p_minor);
}

void fw_get_plattform_firmwareversion(u8 **pp_version)
{
	u32 i;
	u16 n_reg_data1, n_reg_data2;
	u8 szDbBusRxData[12] = { 0 };

	LOGTP_FUNC();

	mutex_lock(&g_mutex);

	platform_device_reset_hw();

	bus_enter_serial_debugmode();
	bus_stop_mcu();
	bus_i2c_use_bus();
	bus_i2c_reshape();
	mdelay(100);

	/* Only MSG22XX support platform firmware version */
	if (g_chiptype == CHIP_TYPE_MSG22XX) {
		/* Stop mcu */
		reg_set_lowbyte(0x0FE6, 0x01);

		/* Stop watchdog */
		reg_set16bit_value(0x3C60, 0xAA55);

		/* RIU password */
		reg_set16bit_value(0x161A, 0xABBA);

		/* Clear pce */
		reg_set16bit_value(0x1618, (reg_get16bit_value(0x1618) | 0x80));

		reg_set16bit_value(0x1600, 0xC1F2);

		/* Enable burst mode */
		reg_set16bit_value(0x160C, (reg_get16bit_value(0x160C) | 0x01));

		/* Set pce */
		reg_set16bit_value(0x1618, (reg_get16bit_value(0x1618) | 0x40));

		for (i = 0; i < 3; i++) {
			reg_set_lowbyte(0x160E, 0x01);

			n_reg_data1 = reg_get16bit_value(0x1604);
			n_reg_data2 = reg_get16bit_value(0x1606);

			szDbBusRxData[i * 4 + 0] = (n_reg_data1 & 0xFF);
			szDbBusRxData[i * 4 + 1] = ((n_reg_data1 >> 8) & 0xFF);



			szDbBusRxData[i * 4 + 2] = (n_reg_data2 & 0xFF);
			szDbBusRxData[i * 4 + 3] = ((n_reg_data2 >> 8) & 0xFF);

		}

		/* Clear burst mode */
		reg_set16bit_value(0x160C,
			reg_get16bit_value(0x160C) & (~0x01));

		reg_set16bit_value(0x1600, 0x0000);

		/* Clear RIU password */
		reg_set16bit_value(0x161A, 0x0000);

		if (*pp_version == NULL)
			*pp_version = kzalloc(sizeof(u8) * 10, GFP_KERNEL);


		sprintf(*pp_version, "%c%c%c%c%c%c%c%c%c%c", szDbBusRxData[2],
			szDbBusRxData[3], szDbBusRxData[4], szDbBusRxData[5],
			szDbBusRxData[6], szDbBusRxData[7], szDbBusRxData[8],
			szDbBusRxData[9], szDbBusRxData[10], szDbBusRxData[11]);
	} else {
		if (*pp_version == NULL)
			*pp_version = kzalloc(sizeof(u8) * 10, GFP_KERNEL);

		sprintf(*pp_version, "%s", "N/A");
	}

	bus_i2c_notuse_buf();
	buf_not_stop_mcu();
	bus_exit_serial_debugmode();

	platform_device_reset_hw();
	mdelay(100);

	mutex_unlock(&g_mutex);

	LOGTP_DBUG(" platform firmware version = %s\n", *pp_version);
}

s32 fw_update_firmware(u8 sz_fw_data[][1024], enum emem_type_e e_mem_type)
{
	LOGTP_FUNC();

	return _fw_update_firmware_cash(sz_fw_data);
}

void fw_handle_finger_touch(void)
{
	struct touchinfo_t t_info;
	u32 i;
#ifdef CONFIG_TP_HAVE_KEY
	u8 nn_touch_keycode = 0;
#endif
	static u32 n_last_keycode; /* = 0; */
	u8 *p_packet = NULL;
	u16 n_report_packet_len = 0;

	/* LOGTP_FUNC(); */

	memset(&t_info, 0x0, sizeof(t_info));

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
	if (g_firmware_mode == FIRMWARE_MODE_DEMO_MODE) {
		LOGTP_DBUG("FIRMWARE_MODE_DEMO_MODE\n");

		n_report_packet_len = DEMO_MODE_PACKET_LENGTH;
		p_packet = g_demomode_packet;
	} else if (g_firmware_mode == FIRMWARE_MODE_DEBUG_MODE) {
		LOGTP_DBUG("FIRMWARE_MODE_DEBUG_MODE\n");

		if (g_firmware_info.n_logmode_packet_header != 0x62) {
			LOGTP_DBUG("WRONG DEBUG MODE HEADER : 0x%x\n",
				g_firmware_info.n_logmode_packet_header);
			return;
		}

		if (g_logmode_packet == NULL) {
			g_logmode_packet =
			kzalloc(sizeof(u8) *
			g_firmware_info.n_logmode_packet_len, GFP_KERNEL);
		}

		n_report_packet_len = g_firmware_info.n_logmode_packet_len;
		p_packet = g_logmode_packet;
	} else if (g_firmware_mode == FIRMWARE_MODE_RAW_DATA_MODE) {
		LOGTP_DBUG("FIRMWARE_MODE_RAW_DATA_MODE\n");

		if (g_firmware_info.n_logmode_packet_header != 0x62) {
			LOGTP_DBUG("WRONG RAW DATA MODE HEADER : 0x%x\n",
				g_firmware_info.n_logmode_packet_header);
			return;
		}

		if (g_logmode_packet == NULL) {
			g_logmode_packet =
				kzalloc(sizeof(u8) *
				g_firmware_info.n_logmode_packet_len,
				GFP_KERNEL);
		}

		n_report_packet_len = g_firmware_info.n_logmode_packet_len;
		p_packet = g_logmode_packet;
	} else {
		LOGTP_DBUG("WRONG FIRMWARE MODE : 0x%x\n", g_firmware_mode);
		return;
	}
#else
	LOGTP_DBUG("FIRMWARE_MODE_DEMO_MODE\n");

	n_report_packet_len = DEMO_MODE_PACKET_LENGTH;
	p_packet = g_demomode_packet;
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	if (g_gesture_wakeup_flag == 1) {
		LOGTP_DBUG("Set gesture wakeup packet length, g_chiptype=%d\n",
				g_chiptype);

		if (g_chiptype == CHIP_TYPE_MSG22XX) {
			if (_g_gesturewakeup_packet == NULL) {
				_g_gesturewakeup_packet =
					kzalloc(sizeof(u8) *
					GESTURE_WAKEUP_PACKET_LENGTH,
					GFP_KERNEL);
			}

			n_report_packet_len = GESTURE_WAKEUP_PACKET_LENGTH;
			p_packet = _g_gesturewakeup_packet;
		} else if (g_chiptype == CHIP_TYPE_MSG21XXA) {
			if (_g_gesturewakeup_packet == NULL) {
				_g_gesturewakeup_packet =
					kzalloc(sizeof(u8) *
					DEMO_MODE_PACKET_LENGTH,
					GFP_KERNEL);
			}

			n_report_packet_len = DEMO_MODE_PACKET_LENGTH;
			p_packet = _g_gesturewakeup_packet;
		} else {
			LOGTP_DBUG("chip does not support gesture wakeup.\n");
			return;
		}
	}
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	if (g_gesture_wakeup_flag == 1) {
		u32 i = 0, rc;

		while (i < 5) {
			mdelay(50);

			rc = i2c_read_data(SLAVE_I2C_ID_DWI2C, &p_packet[0],
					n_report_packet_len);

			if (rc > 0)
				break;

			i++;
		}
	} else {
		i2c_read_data(SLAVE_I2C_ID_DWI2C, &p_packet[0],
			n_report_packet_len);
	}
#else
	i2c_read_data(SLAVE_I2C_ID_DWI2C, &p_packet[0], n_report_packet_len);
#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
	i2c_read_data(SLAVE_I2C_ID_DWI2C, &p_packet[0], n_report_packet_len);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_alloc();
#endif/* CONFIG_ENABLE_DMA_IIC */
	i2c_read_data(SLAVE_I2C_ID_DWI2C, &p_packet[0], n_report_packet_len);
#ifdef CONFIG_ENABLE_DMA_IIC
	dma_free();
#endif/* CONFIG_ENABLE_DMA_IIC */
#endif

	if (0 == _fw_ctrlparse_packet(p_packet, n_report_packet_len, &t_info)) {
		/* report... */
				if ((t_info.n_fingernum) == 0) {/* touch end */
			if (n_last_keycode != 0) {
				LOGTP_INFO("key touch released\n");

				input_report_key(g_input_device, BTN_TOUCH, 0);
				input_report_key(g_input_device, n_last_keycode,
	0);

				n_last_keycode = 0;/* clear key status.. */
			} else {
				platform_finger_touch_released(0, 0);
			}
				} else {/* touch on screen */
			if (t_info.n_touch_keycode != 0) {
#ifdef CONFIG_TP_HAVE_KEY
				/* TOUCH_KEY_HOME */
	if (t_info.n_touch_keycode == 4) {
					nn_touch_keycode = g_TpVirtualKey[1];
				/* TOUCH_KEY_MENU */
	} else if (t_info.n_touch_keycode == 1) {
					nn_touch_keycode = g_TpVirtualKey[0];
				/* TOUCH_KEY_BACK */
	} else if (t_info.n_touch_keycode == 2) {
					nn_touch_keycode = g_TpVirtualKey[2];
				/* TOUCH_KEY_SEARCH */
	} else if (t_info.n_touch_keycode == 8) {
					nn_touch_keycode = g_TpVirtualKey[3];
				}

				if (n_last_keycode != nn_touch_keycode) {
					LOGTP_DBUG("key touch pressed\n");
					LOGTP_DBUG("kcode=%d,LastKeyCode=%d\n",
					nn_touch_keycode, n_last_keycode);

					n_last_keycode = nn_touch_keycode;

					input_report_key(g_input_device,
						BTN_TOUCH, 1);
					input_report_key(g_input_device,
						nn_touch_keycode, 1);
				}
#endif/* CONFIG_TP_HAVE_KEY */
			} else {
				LOGTP_DBUG("t_info->n_fingernum = %d\n",
					t_info.n_fingernum);

				for (i = 0; i < t_info.n_fingernum; i++) {
					platform_finger_pressed(t_info.
							t_point[i].n_x,
							t_info.t_point[i].n_y,
							0, 0);
				}
			}
		}

		input_sync(g_input_device);
	}
}

/*
 ------------------------------------------------------------------------------/
/ */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP

void fw_open_getsture_wakeup(u16 n_mode)
{
	u8 sz_dbbus_txdata[3] = { 0 };
	u32 i = 0;
	s32 rc;

	LOGTP_FUNC();

	LOGTP_DBUG("wakeup mode = 0x%x\n", n_mode);

	sz_dbbus_txdata[0] = 0x58;
	sz_dbbus_txdata[1] = (n_mode >> 8) & 0xFF;
	sz_dbbus_txdata[2] = n_mode & 0xFF;

	while (i < 5) {
		rc = i2c_write_data(SLAVE_I2C_ID_DWI2C, &sz_dbbus_txdata[0], 3);

		if (rc > 0) {
			LOGTP_INFO("Enable gesture wakeup success\n");
			break;
		}

		i++;
	}

	if (i == 5)
		LOGTP_ERRO("Enable gesture wakeup failed\n");

/*
	rc = i2c_write_data(SLAVE_I2C_ID_DWI2C, &sz_dbbus_txdata[0], 3);
	if (rc < 0)
		LOGTP_ERRO("Enable gesture wakeup failed\n");
	else
		LOGTP_INFO("Enable gesture wakeup success\n");
*/
	g_gesture_wakeup_flag = 1;/* gesture wakeup is enabled */
}

void fw_close_getsture_wakeup(void)
{
/*
	u8 sz_dbbus_txdata[3] = {0};
	s32 rc; */

	LOGTP_FUNC();
/*
	sz_dbbus_txdata[0] = 0x58;
	sz_dbbus_txdata[1] = 0x00;
	sz_dbbus_txdata[2] = 0x00;

	rc = i2c_write_data(SLAVE_I2C_ID_DWI2C, &sz_dbbus_txdata[0], 3);
	if (rc < 0)
		LOGTP_ERRO("Disable gesture wakeup failed\n");
	else
		LOGTP_DINFO("Disable gesture wakeup success\n");

*/
	g_gesture_wakeup_flag = 0;/* gesture wakeup is disabled */
}

#endif/* CONFIG_ENABLE_GESTURE_WAKEUP */

/*
 ------------------------------------------------------------------------------/
/ */

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG

u16 fw_change_firmware_mode(u16 n_mode)
{
	u8 sz_dbbus_txdata[2] = { 0 };

	LOGTP_FUNC();

	sz_dbbus_txdata[0] = 0x02;
	sz_dbbus_txdata[1] = (u8) n_mode;

	mdelay(20);

	mutex_lock(&g_mutex);

	i2c_write_data(SLAVE_I2C_ID_DWI2C, &sz_dbbus_txdata[0], 2);

	mutex_unlock(&g_mutex);

	return n_mode;
}

void fw_ctrl_getfirmware_info(firmware_info_t *p_info)
{
	u8 sz_dbbus_txdata[1] = { 0 };
	u8 szDbBusRxData[8] = { 0 };

	LOGTP_FUNC();

	sz_dbbus_txdata[0] = 0x01;

	mutex_lock(&g_mutex);

	mdelay(300);
	i2c_write_data(SLAVE_I2C_ID_DWI2C, &sz_dbbus_txdata[0], 1);
	mdelay(20);
	i2c_read_data(SLAVE_I2C_ID_DWI2C, &szDbBusRxData[0], 8);

	mutex_unlock(&g_mutex);

	if ((szDbBusRxData[1] & 0x80) == 0x80)
		p_info->n_iscan_change_firmwaremode = 0;
	else
		p_info->n_iscan_change_firmwaremode = 1;


	p_info->n_firmware_mode = szDbBusRxData[1] & 0x7F;
	p_info->n_logmode_packet_header = szDbBusRxData[2];
	p_info->n_logmode_packet_len =
	(szDbBusRxData[3] << 8) + szDbBusRxData[4];

	LOGTP_DBUG("p_info->firmware_mode=0x%x, p_info->packet_header=0x%x\n",
		p_info->n_firmware_mode, p_info->n_logmode_packet_header);
	LOGTP_DBUG("p_info->logmodepacketlen=%d, info->firmwaremode=%d\n",
		p_info->n_logmode_packet_len,
		p_info->n_iscan_change_firmwaremode);
}

void fw_restore_firmwaremode_tologdata(void)
{
	firmware_info_t t_info;

	LOGTP_FUNC();

	memset(&t_info, 0x0, sizeof(firmware_info_t));

	fw_ctrl_getfirmware_info(&t_info);

	LOGTP_DBUG("g_firmware_mode = 0x%x, t_info.n_firmware_mode = 0x%x\n",
			g_firmware_mode, t_info.n_firmware_mode);

/* Since reset_hw() will reset the firmware mode to demo mode, we must
 reset the firmware mode again after reset_hw(). */
	if (g_firmware_mode == FIRMWARE_MODE_DEBUG_MODE
	&& FIRMWARE_MODE_DEBUG_MODE != t_info.n_firmware_mode)
		g_firmware_mode =
	fw_change_firmware_mode(FIRMWARE_MODE_DEBUG_MODE);
	 else if (g_firmware_mode == FIRMWARE_MODE_RAW_DATA_MODE
	&& FIRMWARE_MODE_RAW_DATA_MODE != t_info.n_firmware_mode)
		g_firmware_mode =
	fw_change_firmware_mode(FIRMWARE_MODE_RAW_DATA_MODE);
	else
		LOGTP_DBUG("firmware mode is not restored\n");

}
#endif/* CONFIG_ENABLE_FIRMWARE_DATA_LOG */

/*
 ------------------------------------------------------------------------------/
/ */

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID

void fw_checkfirmware_byswid(void)
{
	if (g_chiptype == CHIP_TYPE_MSG21XXA)
		_DrvFwCtrlMsg21xxaCheckFirmwareUpdateBySwId();
	else if (g_chiptype == CHIP_TYPE_MSG22XX)
		_fwctrl_msg22_check_swid();
	else
		LOGTP_DBUG("chip(%d) does not support update\n", g_chiptype);

}

#endif/* CONFIG_UPDATE_FIRMWARE_BY_SW_ID */

/*
 ------------------------------------------------------------------------------/
/ */
