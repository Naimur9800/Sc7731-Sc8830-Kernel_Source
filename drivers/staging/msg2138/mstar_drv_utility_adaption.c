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
 * @file    mstar_drv_utility_adaption.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

/* ////////////////////////////////////////////////////////// */
/* / Included Files */
/* ////////////////////////////////////////////////////////// */
#include "mstar_drv_utility_adaption.h"

/* ////////////////////////////////////////////////////////// */
/* / Data Types */
/* ////////////////////////////////////////////////////////// */

/* ////////////////////////////////////////////////////////// */
/* / Constant */
/* ////////////////////////////////////////////////////////// */

/* ////////////////////////////////////////////////////////// */
/* / Macro */
/* ////////////////////////////////////////////////////////// */

/* ////////////////////////////////////////////////////////// */
/* / Function Prototypes */
/* ////////////////////////////////////////////////////////// */

/* ////////////////////////////////////////////////////////// */
/* / Function Implementation */
/* ////////////////////////////////////////////////////////// */

#ifdef CONFIG_ENABLE_DMA_IIC
#include <linux/dma - mapping.h>
#include <linux/mm_types.h>
#include <linux/mm.h>
/* #include <asm/uaccess.h>*/
#include <linux/uaccess.h>
#include <asm/page.h>
#include <linux/vmalloc.h>

static unsigned char *i2c_dmabuf_va;	/* = NULL;*/
static unsigned int i2c_dmabuf_pa;	/*volatile = NULL;*/

void dma_alloc(void)
{
	if (NULL == i2c_dmabuf_va)
		i2c_dmabuf_va =
	(u8 *) dma_alloc_coherent(NULL, 4096, &i2c_dmabuf_pa,
	GFP_KERNEL);


	if (NULL == i2c_dmabuf_va)
		LOGTP_INFO("DrvCommondma_alloc FAILED!");
	else
		LOGTP_INFO("DrvCommondma_alloc SUCCESS!");

}

void dma_free(void)
{
	if (NULL != i2c_dmabuf_va) {
		dma_free_coherent(NULL, 4096, i2c_dmabuf_va, i2c_dmabuf_pa);
		i2c_dmabuf_va = NULL;
		i2c_dmabuf_pa = 0;
	}
}
#endif/* CONFIG_ENABLE_DMA_IIC */

/*
 ------------------------------------------------------------------------------/
/ */

u16 reg_get16bit_value(u16 n_addr)
{
	u8 tx_data[3] = {0x10, (n_addr >> 8) & 0xFF, n_addr & 0xFF};
	u8 rx_data[2] = {0};
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &tx_data[0], 3);
	i2c_read_data(SLAVE_I2C_ID_DBBUS, &rx_data[0], 2);

	return rx_data[1] << 8 | rx_data[0];
}

u8 reg_get_lowbyte(u16 n_addr)
{
	u8 tx_data[3] = {0x10, (n_addr >> 8) & 0xFF, n_addr & 0xFF};
	u8 rx_data = {0};

	i2c_write_data(SLAVE_I2C_ID_DBBUS, &tx_data[0], 3);
	i2c_read_data(SLAVE_I2C_ID_DBBUS, &rx_data, 1);

	return rx_data;
}

u8 reg_get_hbyte(u16 n_addr)
{
	u8 tx_data[3] = {0x10, (n_addr >> 8) & 0xFF, (n_addr & 0xFF) + 1};
	u8 rx_data = {0};

	i2c_write_data(SLAVE_I2C_ID_DBBUS, &tx_data[0], 3);
	i2c_read_data(SLAVE_I2C_ID_DBBUS, &rx_data, 1);

	return rx_data;
}

void reg_set16bit_value(u16 n_addr, u16 n_data)
{
	u8 tx_data[5] = {0x10, (n_addr >> 8) & 0xFF,
		n_addr & 0xFF, n_data & 0xFF, n_data >> 8};
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &tx_data[0], 5);
}

void reg_set_lowbyte(u16 n_addr, u8 n_data)
{
	u8 tx_data[4] = {0x10, (n_addr >> 8) & 0xFF, n_addr & 0xFF, n_data};
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &tx_data[0], 4);
}

void reg_set_hbyte(u16 n_addr, u8 n_data)
{
	u8 tx_data[4] = {0x10, (n_addr >> 8) & 0xFF,
			(n_addr & 0xFF) + 1, n_data};
	i2c_write_data(SLAVE_I2C_ID_DBBUS, &tx_data[0], 4);
}

void reg_set16bit_valueon(u16 n_addr, u16 n_data)
{/* Set bit on n_data from 0 to 1 */
	u16 r_data = reg_get16bit_value(n_addr);
	r_data |= n_data;
	reg_set16bit_value(n_addr, r_data);
}

/* Set bit on n_data from 1 to 0 */
void reg_set16bit_valueoff(u16 n_addr, u16 n_data)
{
	u16 r_data = reg_get16bit_value(n_addr);
	r_data &= (~n_data);
	reg_set16bit_value(n_addr, r_data);
}

void bus_enter_serial_debugmode(void)
{
	u8 data[5];

	/* Enter the Serial Debug Mode */
	data[0] = 0x53;
	data[1] = 0x45;
	data[2] = 0x52;
	data[3] = 0x44;
	data[4] = 0x42;

	i2c_write_data(SLAVE_I2C_ID_DBBUS, data, 5);
}

void bus_exit_serial_debugmode(void)
{
	u8 data[1];

	/* Exit the Serial Debug Mode */
	data[0] = 0x45;

	i2c_write_data(SLAVE_I2C_ID_DBBUS, data, 1);

	/* Delay some interval to guard the next transaction */
/*    udelay(200);        // delay about 0.2ms */
}

void bus_i2c_use_bus(void)
{
	u8 data[1];

	/* IIC Use Bus */
	data[0] = 0x35;

	i2c_write_data(SLAVE_I2C_ID_DBBUS, data, 1);
}

void bus_i2c_notuse_buf(void)
{
	u8 data[1];

	/* IIC Not Use Bus */
	data[0] = 0x34;

	i2c_write_data(SLAVE_I2C_ID_DBBUS, data, 1);
}

void bus_i2c_reshape(void)
{
	u8 data[1];

	/* IIC Re-shape */
	data[0] = 0x71;

	i2c_write_data(SLAVE_I2C_ID_DBBUS, data, 1);
}

void bus_stop_mcu(void)
{
	u8 data[1];

	/* Stop the MCU */
	data[0] = 0x37;

	i2c_write_data(SLAVE_I2C_ID_DBBUS, data, 1);
}

void buf_not_stop_mcu(void)
{
	u8 data[1];

	/* Not Stop the MCU */
	data[0] = 0x36;

	i2c_write_data(SLAVE_I2C_ID_DBBUS, data, 1);
}

s32 i2c_write_data(u8 n_slave_id, u8 *p_buf, u16 n_size)
{
	s32 rc = 0;

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
	struct i2c_msg msgs[] = {
		{
	.addr = n_slave_id,
	.flags = 0,/* if read flag is undefined, then it means write flag. */
	.len = n_size,
	.buf = p_buf,
	},
	};

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
		   transmitted, else error code. */
	if (gi2c_client != NULL) {
		rc = i2c_transfer(gi2c_client->adapter, msgs, 1);
		if (rc < 0)
			LOGTP_ERRO("i2c_write_data() error %d\n", rc);

	} else {
		LOGTP_ERRO("i2c client is NULL\n");
	}
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
	if (gi2c_client != NULL) {
		u8 n_addr_before = gi2c_client->addr;
		gi2c_client->addr = n_slave_id;
/*        gi2c_client->addr = (gi2c_client->addr & I2C_MASK_FLAG ) |
 (I2C_ENEXT_FLAG); */

#ifdef CONFIG_ENABLE_DMA_IIC
		if (n_size > 8 && NULL != i2c_dmabuf_va) {
			s32 i = 0;

			for (i = 0; i < n_size; i++)
				i2c_dmabuf_va[i] = p_buf[i];

			gi2c_client->ext_flag =
	gi2c_client->ext_flag | I2C_DMA_FLAG;
			rc = i2c_master_send(gi2c_client,
	(unsigned char *)i2c_dmabuf_pa,
	n_size);
		} else {
			gi2c_client->ext_flag =
	gi2c_client->ext_flag & (~I2C_DMA_FLAG);
			rc = i2c_master_send(gi2c_client, p_buf, n_size);
		}
#else
		rc = i2c_master_send(gi2c_client, p_buf, n_size);
#endif/* CONFIG_ENABLE_DMA_IIC */
		gi2c_client->addr = n_addr_before;

		if (rc < 0) {
			LOGTP_ERRO("error%d,slave_id=%d, size=%d\n",
				rc, n_slave_id, n_size);
		}
	} else {
		LOGTP_ERRO("i2c client is NULL\n");
	}
#endif

	return rc;
}

s32 i2c_read_data(u8 n_slave_id, u8 *p_buf, u16 n_size)
{
	s32 rc = 0;

#if (defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) \
	|| defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM))
	struct i2c_msg msgs[] = {
		{
	.addr = n_slave_id,
	.flags = I2C_M_RD,/* read flag */
	.len = n_size,
	.buf = p_buf,
	},
	};

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
		   transmitted, else error code. */
	if (gi2c_client != NULL) {
		rc = i2c_transfer(gi2c_client->adapter, msgs, 1);
		if (rc < 0)
			LOGTP_ERRO("i2c_read_data() error %d\n", rc);

	} else {
		LOGTP_ERRO("i2c client is NULL\n");
	}
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
	if (gi2c_client != NULL) {
		u8 n_addr_before = gi2c_client->addr;
		gi2c_client->addr = n_slave_id;
/*        gi2c_client->addr = (gi2c_client->addr & I2C_MASK_FLAG) |
 (I2C_ENEXT_FLAG); */

#ifdef CONFIG_ENABLE_DMA_IIC
		if (n_size > 8 && NULL != i2c_dmabuf_va) {
			s32 i = 0;

			gi2c_client->ext_flag =
	gi2c_client->ext_flag | I2C_DMA_FLAG;
			rc = i2c_master_recv(gi2c_client,
	(unsigned char *)i2c_dmabuf_pa,
	n_size);

			for (i = 0; i < n_size; i++)
				p_buf[i] = i2c_dmabuf_va[i];

		} else {
			gi2c_client->ext_flag =
	gi2c_client->ext_flag & (~I2C_DMA_FLAG);
			rc = i2c_master_recv(gi2c_client, p_buf, n_size);
		}
#else
		rc = i2c_master_recv(gi2c_client, p_buf, n_size);
#endif/* CONFIG_ENABLE_DMA_IIC */
		gi2c_client->addr = n_addr_before;

		if (rc < 0) {
			LOGTP_ERRO("error %d, n_slave_id=%d, n_size=%d\n",
					rc, n_slave_id, n_size);
		}
	} else {
		LOGTP_ERRO("i2c client is NULL\n");
	}
#endif

	return rc;
}

void mstp_memset(void *p_dst, s8 n_nval, u32 n_size)
{
	memset(p_dst, n_nval, n_size);
}

void mstp_memcopy(void *p_dst, void *p_source, u32 n_size)
{
	memcpy(p_dst, p_source, n_size);
}

void mstp_delay(u32 n_time)
{
	mdelay(n_time);
}

/*
 ------------------------------------------------------------------------------/
/ */
