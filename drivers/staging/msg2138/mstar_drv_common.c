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
 * @file    mstar_drv_common.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

/*=============================================================*/
/* INCLUDE FILE */
/*=============================================================*/

#include "mstar_drv_common.h"

/*=============================================================*/
/* MACRO DEFINITION */
/*=============================================================*/

/*=============================================================*/
/* CONSTANT VALUE DEFINITION */
/*=============================================================*/

/*=============================================================*/
/* VARIABLE DEFINITION */
/*=============================================================*/

static u32 _gcrc32_table[256];

/*=============================================================*/
/* DATA TYPE DEFINITION */
/*=============================================================*/

/*=============================================================*/
/* GLOBAL FUNCTION DEFINITION */
/*=============================================================*/

/* / CRC */
u32 common_crc_reflect(u32 n_ref, s8 n_chr)
{
	u32 n_value = 0;
	u32 i = 0;

	for (i = 1; i < (n_chr + 1); i++) {
		if (n_ref & 1)
			n_value |= 1 << (n_chr - i);

		n_ref >>= 1;
	}

	return n_value;
}

u32 common_crc_getvalue(u32 n_text, u32 n_prevcrc)
{
	u32 n_crc = n_prevcrc;

	n_crc = (n_crc >> 8) ^ _gcrc32_table[(n_crc & 0xFF) ^ n_text];

	return n_crc;
}

void common_crc_inittable(void)
{
	u32 n_magicnumber = 0x04c11db7;
	u32 i, j;

	for (i = 0; i <= 0xFF; i++) {
		_gcrc32_table[i] = common_crc_reflect(i, 8) << 24;
		for (j = 0; j < 8; j++)
			_gcrc32_table[i] =
	(_gcrc32_table[i] << 1) ^ (_gcrc32_table[i] &
	(0x80000000L) ?
	n_magicnumber : 0);

		_gcrc32_table[i] = common_crc_reflect(_gcrc32_table[i], 32);
	}
}

u8 common_cal_checksum(u8 *p_msg, u32 n_len)
{
	s32 n_checksum = 0;
	u32 i;

	for (i = 0; i < n_len; i++)
		n_checksum += p_msg[i];


	return (u8) ((-n_checksum) & 0xFF);
}

u32 common_char_tohexdigit(char *p_chr, u32 n_len)
{
	u32 n_retval = 0;
	u32 i;

	LOGTP_DBUG("n_len = %d\n", n_len);

	for (i = 0; i < n_len; i++) {
		char ch = *p_chr++;
		u32 n = 0;

		if ((i == 0 && ch == '0') || (i == 1 && ch == 'x'))
			continue;


		if ('0' <= ch && ch <= '9')
			n = ch - '0';
		 else if ('a' <= ch && ch <= 'f')
			n = 10 + ch - 'a';
		else if ('A' <= ch && ch <= 'F')
			n = 10 + ch - 'A';


		if (i < 6)
			n_retval = n + n_retval * 16;

	}

	return n_retval;
}

/*
 ------------------------------------------------------------------------------/
/ */
