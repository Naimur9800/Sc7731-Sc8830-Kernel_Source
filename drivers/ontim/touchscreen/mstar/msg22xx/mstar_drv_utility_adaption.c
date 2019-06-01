////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_utility_adaption.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.3.0.0
 *
 */

////////////////////////////////////////////////////////////
/// Included Files
////////////////////////////////////////////////////////////
#include "mstar_drv_utility_adaption.h"
#include "mstar_drv_common.h"
#include "mstar_drv_platform_interface.h"


////////////////////////////////////////////////////////////
/// Function Implementation
////////////////////////////////////////////////////////////

#include <linux/dma-mapping.h>
#include <linux/mm_types.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <linux/vmalloc.h>


static u32 _gCrc32Table[256]; 

/// CRC
u32 DrvCommonCrcDoReflect(u32 nRef, s8 nCh)
{
    u32 nValue = 0;
    u32 i = 0;

    for (i = 1; i < (nCh + 1); i ++)
    {
        if (nRef & 1)
        {
            nValue |= 1 << (nCh - i);
        }
        nRef >>= 1;
    }

    return nValue;
}

u32 DrvCommonCrcGetValue(u32 nText, u32 nPrevCRC)
{
    u32 nCRC = nPrevCRC;

    nCRC = (nCRC >> 8) ^ _gCrc32Table[(nCRC & 0xFF) ^ nText];

    return nCRC;
}

void DrvCommonCrcInitTable(void)
{
    u32 nMagicNumber = 0x04c11db7;
    u32 i, j;

    for (i = 0; i <= 0xFF; i ++)
    {
        _gCrc32Table[i] = DrvCommonCrcDoReflect(i, 8) << 24;
        for (j = 0; j < 8; j ++)
        {
            _gCrc32Table[i] = (_gCrc32Table[i] << 1) ^ (_gCrc32Table[i] & (0x80000000L) ? nMagicNumber : 0);
        }
        _gCrc32Table[i] = DrvCommonCrcDoReflect(_gCrc32Table[i], 32);
    }
}

u8 DrvCommonCalculateCheckSum(u8 *pMsg, u32 nLength)
{
    s32 nCheckSum = 0;
    u32 i;

    for (i = 0; i < nLength; i ++)
    {
        nCheckSum += pMsg[i];
    }

    return (u8)((-nCheckSum) & 0xFF);
}

u32 DrvCommonConvertCharToHexDigit(char *pCh, u32 nLength)
{
    u32 nRetVal = 0;
    u32 i;
    
    DBG("nLength = %d\n", nLength);

    for (i = 0; i < nLength; i ++)
    {
        char ch = *pCh++;
        u32 n = 0;
        
        if ((i == 0 && ch == '0') || (i == 1 && ch == 'x'))
        {
            continue;        
        }
        
        if ('0' <= ch && ch <= '9')
        {
            n = ch-'0';
        }
        else if ('a' <= ch && ch <= 'f')
        {
            n = 10 + ch-'a';
        }
        else if ('A' <= ch && ch <= 'F')
        {
            n = 10 + ch-'A';
        }
        
        if (i < 6)
        {
            nRetVal = n + nRetVal*16;
        }
    }
    
    return nRetVal;
}

//------------------------------------------------------------------------------//
#ifdef CONFIG_ENABLE_DMA_IIC

void DmaAlloc(void)
{
    if (NULL == I2CDMABuf_va)
    {
        I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);
    }
    
    if (NULL == I2CDMABuf_va)
    {
        DBG("DrvCommonDmaAlloc FAILED!");
    }
    else
    {
        DBG("DrvCommonDmaAlloc SUCCESS!");
    }
}

void DmaFree(void)
{
    if (NULL != I2CDMABuf_va)
    {
        dma_free_coherent(NULL, 4096, I2CDMABuf_va, I2CDMABuf_pa);
          I2CDMABuf_va = NULL;
          I2CDMABuf_pa = 0;
    }
}


//------------------------------------------------------------------------------//


static unsigned char *I2CDMABuf_va = NULL;
static volatile unsigned int I2CDMABuf_pa = NULL;
#endif //CONFIG_ENABLE_DMA_IIC

u16 RegGet16BitValue(u16 nAddr)
{
    u8 tx_data[3] = {0x10, (nAddr >> 8) & 0xFF, nAddr & 0xFF};
    u8 rx_data[2] = {0};
    //u16 i2c_timing = g_I2cClient ->timing;//showlo
    //g_I2cClient->timing = 50;//showlo
    IicWriteData(msg2xxx_info->i2c_id_dbbus, &tx_data[0], 3);
    IicReadData(msg2xxx_info->i2c_id_dbbus, &rx_data[0], 2);
    //g_I2cClient->timing = i2c_timing;//showlo

    return (rx_data[1] << 8 | rx_data[0]);
}

u8 RegGetLByteValue(u16 nAddr)
{
    u8 tx_data[3] = {0x10, (nAddr >> 8) & 0xFF, nAddr & 0xFF};
    u8 rx_data = {0};
    //u16 i2c_timing = g_I2cClient ->timing;//showlo
    //g_I2cClient->timing = 50;//showlo
    IicWriteData(msg2xxx_info->i2c_id_dbbus, &tx_data[0], 3);
    IicReadData(msg2xxx_info->i2c_id_dbbus, &rx_data, 1);
    //g_I2cClient->timing = i2c_timing;//showlo

    return (rx_data);
}

u8 RegGetHByteValue(u16 nAddr)
{
    u8 tx_data[3] = {0x10, (nAddr >> 8) & 0xFF, (nAddr & 0xFF) + 1};
    u8 rx_data = {0};
    //u16 i2c_timing = g_I2cClient ->timing;//showlo
    //g_I2cClient->timing = 50;//showlo

    IicWriteData(msg2xxx_info->i2c_id_dbbus, &tx_data[0], 3);
    IicReadData(msg2xxx_info->i2c_id_dbbus, &rx_data, 1);
    //g_I2cClient->timing = i2c_timing;//showlo
    return (rx_data);
}

void RegSet16BitValue(u16 nAddr, u16 nData)
{
    u8 tx_data[5] = {0x10, (nAddr >> 8) & 0xFF, nAddr & 0xFF, nData & 0xFF, nData >> 8};
    //u16 i2c_timing = g_I2cClient ->timing;//showlo
    //g_I2cClient->timing = 50;//showlo
    IicWriteData(msg2xxx_info->i2c_id_dbbus, &tx_data[0], 5);
    //g_I2cClient->timing = i2c_timing;//showlo
}

void RegSetLByteValue(u16 nAddr, u8 nData)
{
    u8 tx_data[4] = {0x10, (nAddr >> 8) & 0xFF, nAddr & 0xFF, nData}; 
    //u16 i2c_timing = g_I2cClient ->timing;//showlo
    //g_I2cClient->timing = 50;//showlo
    IicWriteData(msg2xxx_info->i2c_id_dbbus, &tx_data[0], 4);
    //g_I2cClient->timing = i2c_timing;//showlo
}

void RegSetHByteValue(u16 nAddr, u8 nData)
{
    u8 tx_data[4] = {0x10, (nAddr >> 8) & 0xFF, (nAddr & 0xFF) + 1, nData};
    IicWriteData(msg2xxx_info->i2c_id_dbbus, &tx_data[0], 4);
}

void RegSet16BitValueOn(u16 nAddr, u16 nData) //Set bit on nData from 0 to 1
{
    u16 rData = RegGet16BitValue(nAddr);
    rData |= nData;
    RegSet16BitValue(nAddr, rData);
}

void RegSet16BitValueOff(u16 nAddr, u16 nData) //Set bit on nData from 1 to 0
{
    u16 rData = RegGet16BitValue(nAddr);
    rData &= (~nData);
    RegSet16BitValue(nAddr, rData);
}

u16 RegGet16BitValueByAddressMode(u16 nAddr, AddressMode_e eAddressMode)
{
    u16 nData = 0;
    
    if (eAddressMode == ADDRESS_MODE_16BIT)
    {
        nAddr = nAddr - (nAddr & 0xFF) + ((nAddr & 0xFF) << 1);
    }
    
    nData = RegGet16BitValue(nAddr);
    
    return nData;
}
    
void RegSet16BitValueByAddressMode(u16 nAddr, u16 nData, AddressMode_e eAddressMode)
{
    if (eAddressMode == ADDRESS_MODE_16BIT)
    {
        nAddr = nAddr - (nAddr & 0xFF) + ((nAddr & 0xFF) << 1);
    }
    
    RegSet16BitValue(nAddr, nData);
}

void RegMask16BitValue(u16 nAddr, u16 nMask, u16 nData, AddressMode_e eAddressMode) 
{
    u16 nTmpData = 0;
    
    if (nData > nMask)
    {
        return;
    }

    nTmpData = RegGet16BitValueByAddressMode(nAddr, eAddressMode);
    nTmpData = (nTmpData & (~nMask));
    nTmpData = (nTmpData | nData);
    RegSet16BitValueByAddressMode(nAddr, nTmpData, eAddressMode);
}

void DbBusEnterSerialDebugMode(void)
{
    u8 data[5];

    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;

    IicWriteData(msg2xxx_info->i2c_id_dbbus, data, 5);
}

void DbBusExitSerialDebugMode(void)
{
    u8 data[1];

    // Exit the Serial Debug Mode
    data[0] = 0x45;

    IicWriteData(msg2xxx_info->i2c_id_dbbus, data, 1);
    
    // Delay some interval to guard the next transaction
//    udelay(200);        // delay about 0.2ms
}

void DbBusIICUseBus(void)
{
    u8 data[1];

    // IIC Use Bus
    data[0] = 0x35;

    IicWriteData(msg2xxx_info->i2c_id_dbbus, data, 1);
}

void DbBusIICNotUseBus(void)
{
    u8 data[1];

    // IIC Not Use Bus
    data[0] = 0x34;

    IicWriteData(msg2xxx_info->i2c_id_dbbus, data, 1);
}

void DbBusIICReshape(void)
{
    u8 data[1];

    // IIC Re-shape
    data[0] = 0x71;

    IicWriteData(msg2xxx_info->i2c_id_dbbus, data, 1);
}

void DbBusStopMCU(void)
{
    u8 data[1];

    // Stop the MCU
    data[0] = 0x37;

    IicWriteData(msg2xxx_info->i2c_id_dbbus, data, 1);
}

void DbBusNotStopMCU(void)
{
    u8 data[1];

    // Not Stop the MCU
    data[0] = 0x36;

    IicWriteData(msg2xxx_info->i2c_id_dbbus, data, 1);
}

s32 IicWriteData(u8 nSlaveId, u8* pBuf, u16 nSize)
{
    s32 rc = 0;

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
    struct i2c_msg msgs[] =
    {
        {
            .addr = nSlaveId,
            .flags = 0, // if read flag is undefined, then it means write flag.
            .len = nSize,
            .buf = pBuf,
        },
    };

    /* If everything went ok (i.e. 1 msg transmitted), return #bytes
       transmitted, else error code. */
    if (msg2xxx_info->i2c != NULL)
    {
        rc = i2c_transfer(msg2xxx_info->i2c->adapter, msgs, 1);
        if (rc < 0)
        {
            PRINTF_ERR("IicWriteData() error %d\n", rc);
        }
    }
    else
    {
        PRINTF_ERR("i2c client is NULL\n");
    }
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    if (msg2xxx_info->i2c != NULL)
    {
        u8 nAddrBefore = msg2xxx_info->i2c->addr;
        msg2xxx_info->i2c->addr = nSlaveId;
//        g_I2cClient->addr = (g_I2cClient->addr & I2C_MASK_FLAG ) | (I2C_ENEXT_FLAG);

#ifdef CONFIG_ENABLE_DMA_IIC
        if (nSize > 8 && NULL != I2CDMABuf_va)
        {
            s32 i = 0;
              
            for (i = 0; i < nSize; i ++)
            {
                I2CDMABuf_va[i] = pBuf[i];
            }
            msg2xxx_info->i2c->ext_flag = msg2xxx_info->i2c->ext_flag | I2C_DMA_FLAG;
            rc = i2c_master_send(msg2xxx_info->i2c, (unsigned char *)I2CDMABuf_pa, nSize);
        }
        else
        {
            msg2xxx_info->i2c->ext_flag = msg2xxx_info->i2c->ext_flag & (~I2C_DMA_FLAG);    
            rc = i2c_master_send(msg2xxx_info->i2c, pBuf, nSize);
        }
#else
        rc = i2c_master_send(msg2xxx_info->i2c, pBuf, nSize);
#endif //CONFIG_ENABLE_DMA_IIC
        msg2xxx_info->i2c->addr = nAddrBefore;

        if (rc < 0)
        {
            PRINTF_ERR("IicWriteData() error %d, nSlaveId=%d, nSize=%d\n", rc, nSlaveId, nSize);
        }
    }
    else
    {
        PRINTF_ERR("i2c client is NULL\n");
    }
#endif
    
    return rc;
}

s32 IicReadData(u8 nSlaveId, u8* pBuf, u16 nSize)
{
    s32 rc = 0;

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
    struct i2c_msg msgs[] =
    {
        {
            .addr = nSlaveId,
            .flags = I2C_M_RD, // read flag
            .len = nSize,
            .buf = pBuf,
        },
    };

    /* If everything went ok (i.e. 1 msg transmitted), return #bytes
       transmitted, else error code. */
    if (msg2xxx_info->i2c != NULL)
    {
        rc = i2c_transfer(msg2xxx_info->i2c->adapter, msgs, 1);
        if (rc < 0)
        {
            PRINTF_ERR("IicReadData() error %d\n", rc);
        }
    }
    else
    {
        PRINTF_ERR("i2c client is NULL\n");
    }
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    if (msg2xxx_info->i2c != NULL)
    {
        u8 nAddrBefore = g_I2cClient->addr;
        msg2xxx_info->i2c->addr = nSlaveId;
//        g_I2cClient->addr = (g_I2cClient->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG);

#ifdef CONFIG_ENABLE_DMA_IIC
        if (nSize > 8 && NULL != I2CDMABuf_va)
        {
            s32 i = 0;
        
            msg2xxx_info->i2c->ext_flag = msg2xxx_info->i2c->ext_flag | I2C_DMA_FLAG;
            rc = i2c_master_recv(msg2xxx_info->i2c, (unsigned char *)I2CDMABuf_pa, nSize);
        
            for (i = 0; i < nSize; i ++)
            {
                pBuf[i] = I2CDMABuf_va[i];
            }
        }
        else
        {
            msg2xxx_info->i2c->ext_flag = msg2xxx_info->i2c->ext_flag & (~I2C_DMA_FLAG);    
            rc = i2c_master_recv(msg2xxx_info->i2c, pBuf, nSize);
        }
#else
        rc = i2c_master_recv(msg2xxx_info->i2c, pBuf, nSize);
#endif //CONFIG_ENABLE_DMA_IIC
        msg2xxx_info->i2c->addr = nAddrBefore;

        if (rc < 0)
        {
            PRINTF_ERR("IicReadData() error %d, nSlaveId=%d, nSize=%d\n", rc, nSlaveId, nSize);
        }
    }
    else
    {
        PRINTF_ERR("i2c client is NULL\n");
    }
#endif
    
    return rc;
}

void mstpMemSet(void *pDst, s8 nVal, u32 nSize)
{
    memset(pDst, nVal, nSize);
}

void mstpMemCopy(void *pDst, void *pSource, u32 nSize)
{
    memcpy(pDst, pSource, nSize);
}

void mstpDelay(u32 nTime)
{
    mdelay(nTime);
}

//------------------------------------------------------------------------------//
