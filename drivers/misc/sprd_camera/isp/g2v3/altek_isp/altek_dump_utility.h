/*
* File: altek_dump_utility.h                                                *
* Description: delcalration of dump utility api                             *
*                                                                           *
* (C)Copyright altek Corporation 2016                                       *
*                                                                           *
* History:                                                                  *
*   2016/03/012; Caedmon Lai; Initial version                               *
*/

#ifndef _ALTEK_DUMP_UTILITY
#define _ALTEK_DUMP_UTILITY
/*
 * ///////////////////////////////////////////////////////////
 *             Debug Information API          *
 * ///////////////////////////////////////////////////////////
 */

 /* Dump utility*/
#define MAX_FIFO_REG_NUM 10
#define INTERNAL_HW_NUM \
	(sizeof(hw_info_table) / sizeof(struct hw_reg_info_item))
#define DUMP_ARY_SIZE   512
#define DEFAULT_DUMP_PATH "/data/misc/media/"
#define auto_dump_on_error(err) \
	do { \
		if ((auto_dump == 3 /* always */) || \
		    (((err) != 0) && (auto_dump == 1 /* on-error */))) { \
			ispdrv_dump_firmware_memory(); \
			ispdrv_dump_irpbin(); \
			ispdrv_dump_shadingbin(); \
			ispdrv_dump_ahb_reg(); \
			ispdrv_dump_isp_internal_reg(); \
			isp_dev_append_response( \
				"auto dump on error 0x%08x: OK, " \
				"output to " DEFAULT_DUMP_PATH "\n", err); \
		} \
	} while (0)

/*
 * IRP and Shading Bin buffer size.
 *
 * WARNING:
 * These two macros are copyed from file
 * kernel/include/video/sprd_isp_altek.h
 * Make sure these values are consistent with the original values before
 * using them.
 */
#define ISP_IRP_BIN_BUF_SIZE                    0x100000
#define ISP_SHADING_BIN_BUF_SIZE                0x4b000

struct hw_reg_info_item {
	/* Start address of registers*/
	u32 ul_reg_addr_start;
	/* Offset range of registers*/
	u32 ul_reg_addr_range;
	/* Number of FIFO registers*/
	u32 ul_fifo_reg_num;
	/*Array to store addresses of FIFO registers*/
	u32 aul_fifo_reg_addr_offset[MAX_FIFO_REG_NUM];
};

extern u32 auto_dump;
extern char *str_auto_dump[];

/*
 *\brief Dump ISP internal reg to /home/InternalReg_%s.bin
 *\return error code
 */
extern u32 ispdrv_dump_isp_internal_reg(void);

/*
 *\brief Dump AHB Reg to /home/AHBReg_%s.bin
 *\return error code
 */
extern u32 ispdrv_dump_ahb_reg(void);

/*
 *\brief Dump Firmware memory to /home/DumpFirmwareMemory_(timestamp).bin
 *\return error code
 */
extern u32 ispdrv_dump_firmware_memory(void);

/*
 *\brief Dump ISP QMerge to /data/misc/media/[XXX]SensorXIrp.bin
 *\return error code
 */
extern u32 ispdrv_dump_irpbin(void);

/*
 *\brief Dump ISP shading to /data/misc/media/[XXX]SensorXShading.bin
 *\return error code
 */
extern u32 ispdrv_dump_shadingbin(void);

#endif
