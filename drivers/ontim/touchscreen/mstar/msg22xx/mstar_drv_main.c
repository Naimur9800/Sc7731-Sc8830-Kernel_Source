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
 * @file    mstar_drv_main.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.3.0.0
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_interface.h"
#include "mstar_drv_main.h"
#include "mstar_drv_utility_adaption.h"
#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_self_fw_control.h"


/*=============================================================*/
// CONSTANT VALUE DEFINITION
/*=============================================================*/


/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

static u16 _gDebugReg[MAX_DEBUG_REGISTER_NUM] = {0};
static u32 _gDebugRegCount = 0;

static u8 *_gPlatformFwVersion = NULL; // internal use firmware version for MStar

#ifdef CONFIG_ENABLE_ITO_MP_TEST
static ItoTestMode_e _gItoTestMode = 0;
#endif //CONFIG_ENABLE_ITO_MP_TEST

static u32 _gIsUpdateComplete = 0;

static u8 *_gFwVersion = NULL; // customer firmware version

static struct class *_gFirmwareClass = NULL;
static struct device *_gFirmwareCmdDev = NULL;

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
struct kset *g_TouchKSet = NULL;
struct kobject *g_TouchKObj = NULL;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

u8 g_FwData[94][1024];
unsigned int g_FwDataCount = 0;

/*=============================================================*/
// LOCAL FUNCTION DEFINITION
/*=============================================================*/


/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

ssize_t DrvMainFirmwareChipTypeShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);

    return sprintf(pBuf, "%d", msg2xxx_info->ChipType);
}

ssize_t DrvMainFirmwareChipTypeStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(chip_type, SYSFS_AUTHORITY, DrvMainFirmwareChipTypeShow, DrvMainFirmwareChipTypeStore);

ssize_t DrvMainFirmwareDriverVersionShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);

    return sprintf(pBuf, "%s", DEVICE_DRIVER_RELEASE_VERSION);
}

ssize_t DrvMainFirmwareDriverVersionStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(driver_version, SYSFS_AUTHORITY, DrvMainFirmwareDriverVersionShow, DrvMainFirmwareDriverVersionStore);

/////////ontim////////////    
ssize_t DrvMainFirmwareupShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DrvFwCtrlFwForceUpdate();
    return (msg2xxx_info->major_version << 16) | msg2xxx_info->minor_version  ;
}

ssize_t DrvMainFirmwareupStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    //DrvFwCtrlFwForceUpdate();
    return 3 ;
}

static DEVICE_ATTR(up, SYSFS_AUTHORITY, DrvMainFirmwareupShow, DrvMainFirmwareupStore);

/*--------------------------------------------------------------------------*/

ssize_t DrvMainFirmwareUpdateShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() _gFwVersion = %s ***\n", __func__, _gFwVersion);

    return sprintf(pBuf, "%s\n", _gFwVersion);
}

ssize_t DrvMainFirmwareUpdateStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DrvPlatformLyrDisableFingerTouchReport();

    DBG("*** %s() g_FwDataCount = %d ***\n", __func__, g_FwDataCount);
    g_FwDataCount = 0 ;

    if (0 != DrvFwCtrlUpdateFirmware(g_FwData, EMEM_ALL))
    {
        _gIsUpdateComplete = 0;
        DBG("Update FAILED\n");
    }
    else
    {
        _gIsUpdateComplete = 1;
        DBG("Update SUCCESS\n");
    }

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    DrvFwCtrlRestoreFirmwareModeToLogDataMode();    
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    DrvPlatformLyrEnableFingerTouchReport();
  
    return nSize;
}

static DEVICE_ATTR(update, SYSFS_AUTHORITY, DrvMainFirmwareUpdateShow, DrvMainFirmwareUpdateStore);

ssize_t DrvMainFirmwareVersionShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u16 nMajor = 0, nMinor = 0, nVendor=0;
    DrvFwCtrlGetCustomerFirmwareVersion(&nMajor, &nMinor, &nVendor);

    return sprintf(pBuf, "%03d%03d\n", nMajor,nMinor);
}

ssize_t DrvMainFirmwareVersionStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u16 nMajor = 0, nMinor = 0, nVendor=0;
    DBG("*** %s() _gFwVersion = %d.%d  Vendor %d***\n", __func__, nMajor, nMinor, nVendor);

    return nSize;
}

static DEVICE_ATTR(version, SYSFS_AUTHORITY, DrvMainFirmwareVersionShow, DrvMainFirmwareVersionStore);

ssize_t DrvMainFirmwareDataShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    printk("*** %s() g_FwDataCount = %d ***\n", __func__, g_FwDataCount);
    
    return g_FwDataCount;
}

ssize_t DrvMainFirmwareDataStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u32 nCount = nSize / 1024;
    u32 i;

    DBG("*** %s() ***\n", __func__);

    printk("******count %d nSize %d *******\n",nCount,nSize);
    if (nCount > 0) // nSize >= 1024
    {
        for (i = 0; i < nCount; i ++)
        {
            memcpy(g_FwData[g_FwDataCount], pBuf+(i*1024), 1024);
            g_FwDataCount ++;
        }
    }
    else // nSize < 1024
    {
        if (nSize > 0)
        {
            memcpy(g_FwData[g_FwDataCount], pBuf, nSize);
            g_FwDataCount ++;
        }
    }

    printk("*** g_FwDataCount = %d ***\n", g_FwDataCount);

    if (pBuf != NULL)
    {
        DBG("*** buf[0] = %c ***\n", pBuf[0]);
    }
   
    return nSize;
}

static DEVICE_ATTR(data, SYSFS_AUTHORITY, DrvMainFirmwareDataShow, DrvMainFirmwareDataStore);

#ifdef CONFIG_ENABLE_ITO_MP_TEST
ssize_t DrvMainFirmwareTestShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);
    DBG("*** ctp mp test status = %d ***\n", g_ChipType);
    
    return sprintf(pBuf, "%d", DrvIcFwLyrGetMpTestResult());
}

ssize_t DrvMainFirmwareTestStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u32 nMode = 0;

    DBG("*** %s() ***\n", __func__);
    
    if (pBuf != NULL)
    {
        sscanf(pBuf, "%x", &nMode);   

        DBG("Mp Test Mode = 0x%x\n", nMode);

        if (nMode == ITO_TEST_MODE_OPEN_TEST) //open test
        {
            _gItoTestMode = ITO_TEST_MODE_OPEN_TEST;
            DrvIcFwLyrScheduleMpTestWork(ITO_TEST_MODE_OPEN_TEST);
        }
        else if (nMode == ITO_TEST_MODE_SHORT_TEST) //short test
        {
            _gItoTestMode = ITO_TEST_MODE_SHORT_TEST;
            DrvIcFwLyrScheduleMpTestWork(ITO_TEST_MODE_SHORT_TEST);
        }
        else
        {
            DBG("*** Undefined MP Test Mode ***\n");
        }
    }

    return nSize;
}

static DEVICE_ATTR(test, SYSFS_AUTHORITY, DrvMainFirmwareTestShow, DrvMainFirmwareTestStore);

ssize_t DrvMainFirmwareTestLogShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u32 nLength = 0;
    
    DBG("*** %s() ***\n", __func__);
    
    DrvIcFwLyrGetMpTestDataLog(_gItoTestMode, pBuf, &nLength);
    
    return nLength;
}

ssize_t DrvMainFirmwareTestLogStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(test_log, SYSFS_AUTHORITY, DrvMainFirmwareTestLogShow, DrvMainFirmwareTestLogStore);

ssize_t DrvMainFirmwareTestFailChannelShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u32 nCount = 0;

    DBG("*** %s() ***\n", __func__);
    
    DrvIcFwLyrGetMpTestFailChannel(_gItoTestMode, pBuf, &nCount);
    
    return nCount;
}

ssize_t DrvMainFirmwareTestFailChannelStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(test_fail_channel, SYSFS_AUTHORITY, DrvMainFirmwareTestFailChannelShow, DrvMainFirmwareTestFailChannelStore);
#endif //CONFIG_ENABLE_ITO_MP_TEST

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP

ssize_t DrvMainFirmwareGestureWakeupModeShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);
    DBG("g_GestureWakeupMode = 0x%x\n", g_GestureWakeupMode);

    return sprintf(pBuf, "%x", g_GestureWakeupMode);
}

ssize_t DrvMainFirmwareGestureWakeupModeStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u32 nLength, nWakeupMode;
    
    DBG("*** %s() ***\n", __func__);

    if (pBuf != NULL)
    {
        sscanf(pBuf, "%x", &nWakeupMode);   
        DBG("nWakeupMode = 0x%x\n", nWakeupMode);

        nLength = nSize;
        DBG("nLength = %d\n", nLength);

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG) == GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG);
        }
        
        if ((nWakeupMode & GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG);
        }
       
        if ((nWakeupMode & GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG);
        }

        DBG("g_GestureWakeupMode = 0x%x\n", g_GestureWakeupMode);
    }
        
    return nSize;
}

static DEVICE_ATTR(gesture_wakeup_mode, SYSFS_AUTHORITY, DrvMainFirmwareGestureWakeupModeShow, DrvMainFirmwareGestureWakeupModeStore);

#endif //CONFIG_ENABLE_GESTURE_WAKEUP

/*--------------------------------------------------------------------------*/

ssize_t DrvMainFirmwareDebugShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u32 i;
    u8 nBank, nAddr;
    u16 szRegData[MAX_DEBUG_REGISTER_NUM] = {0};
    u8 szOut[MAX_DEBUG_REGISTER_NUM*25] = {0}, szValue[10] = {0};

    DBG("*** %s() ***\n", __func__);
    
    DbBusEnterSerialDebugMode();
    DbBusStopMCU();
    DbBusIICUseBus();
    DbBusIICReshape();
    mdelay(300);
    
    for (i = 0; i < _gDebugRegCount; i ++)
    {
        szRegData[i] = RegGet16BitValue(_gDebugReg[i]);
    }

    DbBusIICNotUseBus();
    DbBusNotStopMCU();
    DbBusExitSerialDebugMode();

    for (i = 0; i < _gDebugRegCount; i ++)
    {
          nBank = (_gDebugReg[i] >> 8) & 0xFF;
          nAddr = _gDebugReg[i] & 0xFF;
          
        DBG("reg(0x%X,0x%X)=0x%04X\n", nBank, nAddr, szRegData[i]);

        strcat(szOut, "reg(");
        sprintf(szValue, "0x%X", nBank);
        strcat(szOut, szValue);
        strcat(szOut, ",");
        sprintf(szValue, "0x%X", nAddr);
        strcat(szOut, szValue);
        strcat(szOut, ")=");
        sprintf(szValue, "0x%04X", szRegData[i]);
        strcat(szOut, szValue);
        strcat(szOut, "\n");
    }

    return sprintf(pBuf, "%s\n", szOut);
}

ssize_t DrvMainFirmwareDebugStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u32 i;
    char *pCh;

    DBG("*** %s() ***\n", __func__);

    if (pBuf != NULL)
    {
        DBG("*** %s() pBuf[0] = %c ***\n", __func__, pBuf[0]);
        DBG("*** %s() pBuf[1] = %c ***\n", __func__, pBuf[1]);
        DBG("*** %s() pBuf[2] = %c ***\n", __func__, pBuf[2]);
        DBG("*** %s() pBuf[3] = %c ***\n", __func__, pBuf[3]);
        DBG("*** %s() pBuf[4] = %c ***\n", __func__, pBuf[4]);
        DBG("*** %s() pBuf[5] = %c ***\n", __func__, pBuf[5]);

        DBG("nSize = %d\n", nSize);
       
        i = 0;
        while ((pCh = strsep((char **)&pBuf, " ,")) && (i < MAX_DEBUG_REGISTER_NUM))
        {
            DBG("pCh = %s\n", pCh);
            
            _gDebugReg[i] = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));

            DBG("_gDebugReg[%d] = 0x%04X\n", i, _gDebugReg[i]);
            i ++;
        }
        _gDebugRegCount = i;
        
        DBG("_gDebugRegCount = %d\n", _gDebugRegCount);
    }

    return nSize;
}

static DEVICE_ATTR(debug, SYSFS_AUTHORITY, DrvMainFirmwareDebugShow, DrvMainFirmwareDebugStore);

/*--------------------------------------------------------------------------*/

ssize_t DrvMainFirmwarePlatformVersionShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DrvFwCtrlGetPlatformFirmwareVersion(&_gPlatformFwVersion);
    printk("*** %s() _gPlatformFwVersion = %s ***\n", __func__, _gPlatformFwVersion);

    return sprintf(pBuf, "%s\n", _gPlatformFwVersion);
}

ssize_t DrvMainFirmwarePlatformVersionStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DrvFwCtrlGetPlatformFirmwareVersion(&_gPlatformFwVersion);

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    DrvFwCtrlRestoreFirmwareModeToLogDataMode();    
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

   printk("*** %s() _gPlatformFwVersion = %s ***\n", __func__, _gPlatformFwVersion);

    return nSize;
}

static DEVICE_ATTR(platform_version, SYSFS_AUTHORITY, DrvMainFirmwarePlatformVersionShow, DrvMainFirmwarePlatformVersionStore);

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
ssize_t DrvMainFirmwareModeShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    g_FirmwareMode = DrvFwCtrlGetFirmwareMode();
    
    DBG("%s() firmware mode = 0x%x\n", __func__, g_FirmwareMode);

    return sprintf(pBuf, "%x", g_FirmwareMode);
#elif defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    DrvFwCtrlGetFirmwareInfo(&g_FirmwareInfo);
    g_FirmwareMode = g_FirmwareInfo.nFirmwareMode;

    DBG("%s() firmware mode = 0x%x, can change firmware mode = %d\n", __func__, g_FirmwareInfo.nFirmwareMode, g_FirmwareInfo.nIsCanChangeFirmwareMode);

    return sprintf(pBuf, "%x,%d", g_FirmwareInfo.nFirmwareMode, g_FirmwareInfo.nIsCanChangeFirmwareMode);
#endif
}

ssize_t DrvMainFirmwareModeStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    u32 nMode;
    
    if (pBuf != NULL)
    {
        sscanf(pBuf, "%x", &nMode);   
        DBG("firmware mode = 0x%x\n", nMode);

        if (nMode == FIRMWARE_MODE_DEMO_MODE) //demo mode
        {
            g_FirmwareMode = DrvFwCtrlChangeFirmwareMode(FIRMWARE_MODE_DEMO_MODE);
        }
        else if (nMode == FIRMWARE_MODE_DEBUG_MODE) //debug mode
        {
            g_FirmwareMode = DrvFwCtrlChangeFirmwareMode(FIRMWARE_MODE_DEBUG_MODE);
        }
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
        else if (nMode == FIRMWARE_MODE_RAW_DATA_MODE) //raw data mode
        {
            g_FirmwareMode = DrvFwCtrlChangeFirmwareMode(FIRMWARE_MODE_RAW_DATA_MODE);
        }
#endif //CONFIG_ENABLE_CHIP_MSG21XXA || CONFIG_ENABLE_CHIP_MSG22XX
        else
        {
            DBG("*** Undefined Firmware Mode ***\n");
        }
    }

    DBG("*** g_FirmwareMode = 0x%x ***\n", g_FirmwareMode);

    return nSize;
}

static DEVICE_ATTR(mode, SYSFS_AUTHORITY, DrvMainFirmwareModeShow, DrvMainFirmwareModeStore);
/*
ssize_t DrvMainFirmwarePacketShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u32 i = 0;
    u32 nLength = 0;
    
    DBG("*** %s() ***\n", __func__);
    
    if (g_LogModePacket != NULL)
    {
        DBG("g_FirmwareMode=%x, g_LogModePacket[0]=%x, g_LogModePacket[1]=%x\n", g_FirmwareMode, g_LogModePacket[0], g_LogModePacket[1]);
        DBG("g_LogModePacket[2]=%x, g_LogModePacket[3]=%x\n", g_LogModePacket[2], g_LogModePacket[3]);
        DBG("g_LogModePacket[4]=%x, g_LogModePacket[5]=%x\n", g_LogModePacket[4], g_LogModePacket[5]);

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
        if ((g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE) && (g_LogModePacket[0] == 0xA5 || g_LogModePacket[0] == 0xAB || g_LogModePacket[0] == 0xA7))
#elif defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
        if ((g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE || g_FirmwareMode == FIRMWARE_MODE_RAW_DATA_MODE) && (g_LogModePacket[0] == 0x62))
#endif
        {
            for (i = 0; i < g_FirmwareInfo.nLogModePacketLength; i ++)
            {
                pBuf[i] = g_LogModePacket[i];
            }

            nLength = g_FirmwareInfo.nLogModePacketLength;
            DBG("nLength = %d\n", nLength);
        }
        else
        {
            DBG("CURRENT MODE IS NOT DEBUG MODE/WRONG DEBUG MODE HEADER\n");
//        nLength = 0;
        }
    }
    else
    {
        DBG("g_LogModePacket is NULL\n");
//        nLength = 0;
    }

    return nLength;
}

ssize_t DrvMainFirmwarePacketStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(packet, SYSFS_AUTHORITY, DrvMainFirmwarePacketShow, DrvMainFirmwarePacketStore);
*/
ssize_t DrvMainFirmwareSensorShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    if (g_FirmwareInfo.nLogModePacketHeader == 0xA5 || g_FirmwareInfo.nLogModePacketHeader == 0xAB)
    {
        return sprintf(pBuf, "%d,%d", g_FirmwareInfo.nMx, g_FirmwareInfo.nMy);
    }
    else if (g_FirmwareInfo.nLogModePacketHeader == 0xA7)
    {
        return sprintf(pBuf, "%d,%d,%d,%d", g_FirmwareInfo.nMx, g_FirmwareInfo.nMy, g_FirmwareInfo.nSs, g_FirmwareInfo.nSd);
    }
    else
    {
        DBG("Undefined debug mode packet format : 0x%x\n", g_FirmwareInfo.nLogModePacketHeader);
        return 0;
    }
#elif defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    return sprintf(pBuf, "%d", g_FirmwareInfo.nLogModePacketLength);
#endif
}

ssize_t DrvMainFirmwareSensorStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);
  
    return nSize;
}

static DEVICE_ATTR(sensor, SYSFS_AUTHORITY, DrvMainFirmwareSensorShow, DrvMainFirmwareSensorStore);

ssize_t DrvMainFirmwareHeaderShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);

    return sprintf(pBuf, "%d", g_FirmwareInfo.nLogModePacketHeader);
}

ssize_t DrvMainFirmwareHeaderStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(header, SYSFS_AUTHORITY, DrvMainFirmwareHeaderShow, DrvMainFirmwareHeaderStore);

//------------------------------------------------------------------------------//

ssize_t DrvMainKObjectPacketShow(struct kobject *pKObj, struct kobj_attribute *pAttr, char *pBuf)
{
    u32 i = 0;
    u32 nLength = 0;

    DBG("*** %s() ***\n", __func__);

    if (strcmp(pAttr->attr.name, "packet") == 0)
    {
        if (g_LogModePacket != NULL)
        {
            DBG("g_FirmwareMode=%x, g_LogModePacket[0]=%x, g_LogModePacket[1]=%x\n", g_FirmwareMode, g_LogModePacket[0], g_LogModePacket[1]);
            DBG("g_LogModePacket[2]=%x, g_LogModePacket[3]=%x\n", g_LogModePacket[2], g_LogModePacket[3]);
            DBG("g_LogModePacket[4]=%x, g_LogModePacket[5]=%x\n", g_LogModePacket[4], g_LogModePacket[5]);

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
            if ((g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE) && (g_LogModePacket[0] == 0xA5 || g_LogModePacket[0] == 0xAB || g_LogModePacket[0] == 0xA7))
#elif defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
            if ((g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE || g_FirmwareMode == FIRMWARE_MODE_RAW_DATA_MODE) && (g_LogModePacket[0] == 0x62))
#endif
            {
                for (i = 0; i < g_FirmwareInfo.nLogModePacketLength; i ++)
                {
                    pBuf[i] = g_LogModePacket[i];
                }

                nLength = g_FirmwareInfo.nLogModePacketLength;
                DBG("nLength = %d\n", nLength);
            }
            else
            {
                DBG("CURRENT MODE IS NOT DEBUG MODE/WRONG DEBUG MODE HEADER\n");
//            nLength = 0;
            }
        }
        else
        {
            DBG("g_LogModePacket is NULL\n");
//            nLength = 0;
        }
    }
    else
    {
        DBG("pAttr->attr.name = %s \n", pAttr->attr.name);
//        nLength = 0;
    }

    return nLength;
}

ssize_t DrvMainKObjectPacketStore(struct kobject *pKObj, struct kobj_attribute *pAttr, const char *pBuf, size_t nCount)
{
    DBG("*** %s() ***\n", __func__);
/*
    if (strcmp(pAttr->attr.name, "packet") == 0)
    {

    }
*/        
    return nCount;
}

static struct kobj_attribute packet_attr = __ATTR(packet, 0666, DrvMainKObjectPacketShow, DrvMainKObjectPacketStore);

/* Create a group of attributes so that we can create and destroy them all at once. */
static struct attribute *attrs[] = {
    &packet_attr.attr,
    NULL,    /* need to NULL terminate the list of attributes */
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
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG
//------------------------------------------------------------------------------//
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
#define CHAR_NB 40
    int i,virtual_key_number;
    struct touchpanel_virtual_key_data *vkey;

    char *buffer;
    int     char_num;
    int     key_code;
    int     logic_x;
    int     logic_y;
    int     x_correction;
    int     y_correction;

    if ( msg2xxx_info == NULL ) goto error;
    if ( msg2xxx_info->pdata == NULL ) goto error;

    virtual_key_number = msg2xxx_info->panel_parm->virtual_key_num;
    vkey =  msg2xxx_info->panel_parm->virtual_key;
    if (( virtual_key_number == 0 ) || ( vkey == NULL )) goto error;

    buffer = kcalloc(virtual_key_number,CHAR_NB, GFP_KERNEL);
    if ( buffer == NULL ) goto error;

    memset(buffer,0,CHAR_NB * virtual_key_number);

    for( i=0 ,char_num=0; i < virtual_key_number ; i++)
    {
        if ( vkey[i].key_code)
        {
            key_code = vkey[i].key_code;
            {
                logic_x = vkey[i].logic_x;
                logic_y = vkey[i].logic_y;
                x_correction = vkey[i].x_correction;
                y_correction = vkey[i].y_correction;
            }

            if ( char_num )
            {
                sprintf(buffer+char_num++,":");
            }
            char_num +=sprintf( buffer+char_num , __stringify(EV_KEY)":");
            char_num +=sprintf( buffer+char_num , "%d", key_code);
            char_num +=sprintf( buffer+char_num , ":%d", logic_x);
            char_num +=sprintf( buffer+char_num , ":%d", logic_y);
            char_num +=sprintf( buffer+char_num , ":%d", x_correction);
            char_num +=sprintf( buffer+char_num , ":%d", y_correction);
        }
    }
    if ( char_num )
    {
        sprintf(buffer+char_num++,"\n");
    }
    buffer[char_num]=0;
    printk(KERN_INFO "[kernel][%s %d]%s   ;  len = %d\n",
           __func__,__LINE__,buffer,char_num);
    memcpy(buf,buffer,char_num);
    kfree(buffer);
    return char_num;
error:
    printk(KERN_INFO "[kernel][%s %d] : No virtualbutton\n",__func__,__LINE__);
    return 0;

}

static struct kobj_attribute virtual_keys_attr =
{
    .attr = {
        .name = "virtualkeys.msg2xxx",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] =
{
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group =
{
    .attrs = properties_attrs,
};
//----proximity---------------------------------------------------------
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION 
static ssize_t msg2xxx_ps_onoff_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    if(msg2xxx_info->ps_onoff)
        return sprintf(buf, "%s\n","1");
    else
        return sprintf(buf, "%s\n","0");
}

static ssize_t msg2xxx_ps_onoff_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    printk( "[kernel][%s]***buf=%s enter \n",__func__,buf);
	mutex_lock(&msg2xxx_info->ts_suspend_lock);
	mutex_lock(&msg2xxx_info->ts_lock);

    if(strncmp(buf,"1",1)==0)
    {
        if((msg2xxx_info->ps_onoff==0) && (msg2xxx_info->ps_input_dev) )
        {
            msg2xxx_info->ps_onoff=1;
			wake_lock_timeout(&msg2xxx_info->wlock,msecs_to_jiffies(300));
            schedule_delayed_work(&msg2xxx_info->ps_work,msecs_to_jiffies(100));
        }
    }
    else if(strncmp(buf,"0",1)==0)
    {
        int need_disable_irq=0;
        
        msg2xxx_info->ps_onoff=0;
		wake_unlock(&msg2xxx_info->wlock);
		cancel_delayed_work(&msg2xxx_info->ps_work);
        DrvPlatformLyrTpPsEnable(0);
        if (msg2xxx_info->suspend_state == 1)
        {
            printk("[kernel][%s]: [FTS]msg2xxx suspend [1]\n",__func__);
            msg2xxx_info->suspend_state=2;
            need_disable_irq = 1;
        }

        if (need_disable_irq)
        {
            printk("[kernel][%s]: [FTS]msg2xxx suspend [2]\n",__func__);
            disable_irq(msg2xxx_info->irq);
            //msg2xxx_info->run_state = 0;
        }
    }
	mutex_unlock(&msg2xxx_info->ts_lock);
	mutex_unlock(&msg2xxx_info->ts_suspend_lock);
    return count;
}
static ssize_t msg2xxx_ps_state_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    if(msg2xxx_info->ps_state)
        return sprintf(buf, "%s\n","1");
    else
        return sprintf(buf, "%s\n","0");
}
static ssize_t msg2xxx_ps_state_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
        return 0;
}

static ssize_t msg2xxx_ps_vendor_name_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
     return sprintf(buf, "msg2xxx\n");
}

static DEVICE_ATTR(proximity_onoff, S_IRUGO|S_IWUSR|S_IWGRP,
        msg2xxx_ps_onoff_show, msg2xxx_ps_onoff_store);

static DEVICE_ATTR(ps_state, S_IRUGO|S_IWUSR|S_IWGRP,
        msg2xxx_ps_state_show, msg2xxx_ps_state_store);

static DEVICE_ATTR(vendor_name, S_IRUGO,
        msg2xxx_ps_vendor_name_show, NULL);

static struct attribute *msg2xxx_ps_attributes[] = {
    &dev_attr_proximity_onoff.attr,
    &dev_attr_ps_state.attr,
    &dev_attr_vendor_name.attr,
    NULL
};

static struct attribute_group msg2xxx_ps_attribute_group = {
    .attrs = msg2xxx_ps_attributes
};
#endif
//---------------end----------------------------------------------------------

//------------------------------------------------------------------------------//
s32 DrvMainTouchDeviceInitialize(void)
{
    s32 nRetVal = 0;

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    u8 *pDevicePath = NULL;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    DBG("*** %s() ***\n", __func__);

    /* set sysfs for firmware */
    _gFirmwareClass = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
    if (IS_ERR(_gFirmwareClass))
        DBG("Failed to create class(firmware)!\n");

    _gFirmwareCmdDev = device_create(_gFirmwareClass, NULL, 0, NULL, "device");
    if (IS_ERR(_gFirmwareCmdDev))
        DBG("Failed to create device(_gFirmwareCmdDev)!\n");

    // version
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_version) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
    // update
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_update) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
    // data
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_data) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_data.attr.name);
#ifdef CONFIG_ENABLE_ITO_MP_TEST
    // test
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_test) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_test.attr.name);
    // test_log
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_test_log) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_test_log.attr.name);
    // test_fail_channel
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_test_fail_channel) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_test_fail_channel.attr.name);
#endif //CONFIG_ENABLE_ITO_MP_TEST

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    // mode
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_mode) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_mode.attr.name);
    // packet
//    if (device_create_file(_gFirmwareCmdDev, &dev_attr_packet) < 0)
//        DBG("Failed to create device file(%s)!\n", dev_attr_packet.attr.name);
    // sensor
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_sensor) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_sensor.attr.name);
    // header
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_header) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_header.attr.name);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    // debug
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_debug) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_debug.attr.name);
    // platform version
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_platform_version) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_platform_version.attr.name);
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    // gesture wakeup mode
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_gesture_wakeup_mode) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_gesture_wakeup_mode.attr.name);
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
    // chip type
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_chip_type) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_chip_type.attr.name);
    // driver version
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_driver_version) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_driver_version.attr.name);
    /////////ontim////////////    
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_up) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_up.attr.name);

    if ( msg2xxx_info->panel_parm->virtual_key && msg2xxx_info->panel_parm->virtual_key_num)
    {
        int retval=0;
        struct kobject *properties_kobj;
        /*virtual key init here begin*/
        properties_kobj = kobject_create_and_add("board_properties", NULL);
        if (properties_kobj)
            retval = sysfs_create_group(properties_kobj,
                                        &properties_attr_group);
        if (!properties_kobj || retval)
            printk(KERN_ERR "[kernel][%s %d]/-/-/-/-/-/failed to create board_properties\n",
                             __func__, __LINE__);
        /*virtual key init end*/
    }
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
    if( msg2xxx_info->ps_input_dev )
    {
        if( sysfs_create_group(&msg2xxx_info->ps_input_dev->dev.kobj,&msg2xxx_ps_attribute_group) < 0 )      
        {         
            printk(KERN_ERR "[kernel][%s %d] create ps_input_dev sysfs have a error \n",__func__,__LINE__);  
            return -1;      
        }    
    }
#endif
    dev_set_drvdata(_gFirmwareCmdDev, NULL);

#ifdef CONFIG_ENABLE_ITO_MP_TEST
    DrvIcFwLyrCreateMpTestWorkQueue();
#endif //CONFIG_ENABLE_ITO_MP_TEST
    
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    /* create a kset with the name of "kset_example" which is located under /sys/kernel/ */
    g_TouchKSet = kset_create_and_add("kset_example", NULL, kernel_kobj);
    if (!g_TouchKSet)
    {
        DBG("*** kset_create_and_add() failed, nRetVal = %d ***\n", nRetVal);

        nRetVal = -ENOMEM;
    }

    g_TouchKObj = kobject_create();
    if (!g_TouchKObj)
    {
        DBG("*** kobject_create() failed, nRetVal = %d ***\n", nRetVal);

        nRetVal = -ENOMEM;
            kset_unregister(g_TouchKSet);
            g_TouchKSet = NULL;
    }

    g_TouchKObj->kset = g_TouchKSet;

    nRetVal = kobject_add(g_TouchKObj, NULL, "%s", "kobject_example");
    if (nRetVal != 0)
    {
        DBG("*** kobject_add() failed, nRetVal = %d ***\n", nRetVal);

            kobject_put(g_TouchKObj);
            g_TouchKObj = NULL;
            kset_unregister(g_TouchKSet);
            g_TouchKSet = NULL;
    }
    
    /* create the files associated with this kobject */
    nRetVal = sysfs_create_group(g_TouchKObj, &attr_group);
    if (nRetVal != 0)
    {
        DBG("*** sysfs_create_file() failed, nRetVal = %d ***\n", nRetVal);

        kobject_put(g_TouchKObj);
            g_TouchKObj = NULL;
            kset_unregister(g_TouchKSet);
            g_TouchKSet = NULL;
    }
    
    pDevicePath = kobject_get_path(g_TouchKObj, GFP_KERNEL);
    DBG("DEVPATH = %s\n", pDevicePath);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
        memset(&g_FirmwareInfo, 0x0, sizeof(FirmwareInfo_t));
        DrvFwCtrlGetFirmwareInfo(&g_FirmwareInfo);
        g_FirmwareMode = g_FirmwareInfo.nFirmwareMode;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    return nRetVal;
}


//------------------------------------------------------------------------------//
