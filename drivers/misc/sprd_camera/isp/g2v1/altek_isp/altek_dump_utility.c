/*
* File: altek_dump_utility.c                                                *
* Description: implementation of dump utility api                           *
*                                                                           *
* (C)Copyright altek Corporation 2016                                       *
*                                                                           *
* History:                                                                  *
*   2016/03/012; Caedmon Lai; Initial version                               *
*/
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/uaccess.h>

#include "altek_sensor_info.h"
#include "altek_ahb_drv.h"
#include "altek_common.h"
#include "altek_isp_drv.h"
#include "altek_isp_drv_local.h"
#include "altek_dump_utility.h"
#include "altek_log_local.h"
#define DUMP_IRP_BIN_WITH_EXACT_SIZE	0

static struct hw_reg_info_item hw_info_table[] = {
/*	StrartAddr, Range, FIFO_Num, FIFO_Offset */
	{0xffe13bc0, 0x38,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffe15d80, 0x38,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffe119c0, 0x38,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffe40000, 0x4c,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffe60200, 0x1dfc,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffe645a0, 0x15d8,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffe66640, 0x1738,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffe6e460, 0x1918,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffe80000, 0x2c8,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffed0000, 0xc0,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffed2000, 0xc0,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffed5000, 0x25c,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffed6000, 0x25c,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffed7000, 0x68, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffed8000, 0x68,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffedb000, 0x134, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffedc000, 0x134, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffedd000, 0x134, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xffef0000, 0x108,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff10000, 0xf90,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff18000, 0xf90,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff20000, 0x2bc,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff21000, 0x2bc,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff22000, 0x594,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff23000, 0xf50,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff24000, 0xd0,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff25000, 0xd0,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff26000, 0x134,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff27000, 0x134,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff28000, 0x134,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff30000, 0x2130,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff38000, 0x2130,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff40000, 0x5c,	1, {0x18, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff41000, 0x5c,	1, {0x18, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff42000, 0x5c,	1, {0x18, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff43000, 0xf4,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff44000, 0xf4,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff50000, 0x49fc,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff6b000, 0x248,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff6c000, 0x248,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff6e000, 0x6b8,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff70000, 0x3e0c,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
/*	{0xfff70000, 0x49fc,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} }, */
/*	{0xfff74974, 0x88,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} }, */
	{0xfff86000, 0xa5c,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfff89000, 0xa5c,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffa0000, 0x58,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffa3000, 0x120,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffa4000, 0x120,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffa5000, 0x120,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffa6000, 0x34,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffa7000, 0x34,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffa8000, 0x300,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffa9000, 0xe0,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffaa000, 0xe0,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffac000, 0x58,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffad000, 0x58,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffae000, 0x908,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffaf000, 0x300,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffb0000, 0x814,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffb1000, 0x814,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffb2000, 0x814,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffb8000, 0x5c,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffb9000, 0x5c,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffba000, 0x5c,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffbb000, 0x5c,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffc0000, 0x134, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffc1000, 0x134, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffc2000, 0x134, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffc3000, 0x134, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffc4000, 0x134, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffc5000, 0x134, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffc6000, 0x25c,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffc7000, 0x4e4,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffc8000, 0x4e4, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffc9000, 0x68, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffca000, 0x134, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffcb000, 0x4e4,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffd0740, 0x1438,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffd4000, 0x34,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffd5000, 0xe0,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffd6000, 0xf50,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffd7000, 0xc0,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffd8000, 0xf90,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
	{0xfffe0000, 0x49fc,	0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} },
};

#if DUMP_IRP_BIN_WITH_EXACT_SIZE
static u32 calc_shading_bin_size(u32 bin_addr);
static u32 calc_irp_bin_size(u32 bin_addr);
#endif

u32 auto_dump = NUMBER_ZERO;
char *str_auto_dump[] = {
	/* 0 */ "none",
	/* 1 */ "on-error",
	/* 2 */ "on-success",
	/* 3 */ "always",
	NULL,
};

/*
*\brief Dump Firmware memory to /home/DumpFirmwareMemory_(timestamp).bin
*\return error code
*/
u32 ispdrv_dump_firmware_memory(void)
{
	u32 err = 0, size = 20 * 1024 * 1024;	/* 5M*/
	char filepath[128];
	struct timespec ut;

	isp_debug_pu_start_tag();

	get_monotonic_boottime(&ut);
	sprintf(filepath, DEFAULT_DUMP_PATH "[%06lu.%09lu]ISPfirmware.bin",
		ut.tv_sec, ut.tv_nsec);

	isp_debug_item_tag("filepath: %s", filepath);

	dump_memory(filepath, common_get_iva_fw_addr(), size);

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_dump_firmware_memory);

/*
*\brief Dump AHB Reg to /home/AHBReg_%s.bin
*\return error code
*/
u32 ispdrv_dump_ahb_reg(void)
{
	u32 err = 0;
	char filepath[128];
	struct timespec ut;

	isp_debug_pu_start_tag();

	get_monotonic_boottime(&ut);
	sprintf(filepath, DEFAULT_DUMP_PATH "[%06lu.%09lu]AHBReg.bin",
		ut.tv_sec, ut.tv_nsec);

	isp_debug_item_tag("filepath: %s", filepath);

	dump_isp_ahb(filepath, common_get_ahb_size());

	isp_debug_err("%d", err);
	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_dump_ahb_reg);

/*
*\brief Dump ISP internal reg to /home/InternalReg_%s.bin
*\return error code
*/
u32 ispdrv_dump_isp_internal_reg(void)
{
	u32 err = 0;
	char filepath[128];
	u32 start;
	u32 size;
	u8 i;
	struct timespec ut;

	isp_debug_pu_start_tag();

	get_monotonic_boottime(&ut);

	for (i = 0; i < INTERNAL_HW_NUM; i++) {
		start = hw_info_table[i].ul_reg_addr_start;
		/* the range represents the start of the last reg */
		size = hw_info_table[i].ul_reg_addr_range + 4;

		if (start == 0x0)
			continue;
		memset((u8 *) filepath, 0, 128);
		sprintf(filepath,
			DEFAULT_DUMP_PATH "[%06lu.%09lu]InternalReg_0x%x.bin",
			ut.tv_sec, ut.tv_nsec, start);
		isp_debug_item_tag("Dumping addr: 0x%x, size: 0x%x\n",
			start, size);
		isp_debug_item_tag("filepath: %s", filepath);
		dump_isp_internal_reg(filepath, start, size, i);
		isp_debug_err("%d", err);
	}

	isp_debug_pu_end_tag();

	return err;

}
EXPORT_SYMBOL(ispdrv_dump_isp_internal_reg);

/*
 *\brief Dump ISP irp bin to /data/misc/media/[XXX]SensorXIrp.bin
 *\return error code
 */
u32 ispdrv_dump_irpbin(void)
{
	u32 err = 0;
	u32 sensor;
	u32 size = 0;
	char filepath[128];
	u32 irp_addr = 0;
	struct timespec ut;

	isp_debug_pu_start_tag();

	get_monotonic_boottime(&ut);

	for (sensor = SENSOR1; sensor < SENSOR_TOTAL; sensor++) {
		irp_addr =
			common_get_qmerge_data_addr_by_sensor_id(sensor);

		if (irp_addr == 0) {
			isp_debug_desc_tag("addr. of sensor %u irp bin is 0",
				sensor+1);
			continue;
		}

#if DUMP_IRP_BIN_WITH_EXACT_SIZE
		/* Calculate the size of the irp bin. */
		size = calc_irp_bin_size(irp_addr);
#else
		size = ISP_IRP_BIN_BUF_SIZE;
#endif
		isp_debug_item_tag(
			"sensor: %u, addr: 0x%x, size: %u",
			sensor+1, irp_addr, size);
		if (size == 0) {
			isp_debug_desc_tag(
				"irp bin size for sensor %u is 0",
				sensor+1);
			continue;
		}

		/* Dump the irp bin */
		sprintf(filepath,
			DEFAULT_DUMP_PATH "[%06lu.%09lu]Sensor%uIrp.bin",
			ut.tv_sec, ut.tv_nsec, sensor+1);
		err = dump_memory(filepath, irp_addr, size);
		isp_debug_item_tag("irp bin for sensor %u, dump result: %u",
			sensor+1, err);
	}

	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_dump_irpbin);


/*
 *\brief Dump ISP shading bin to /data/misc/media/[XXX]SensorXShading.bin
 *\return error code
 */
u32 ispdrv_dump_shadingbin(void)
{
	u32 err = 0;
	u32 sensor;
	u32 size = 0;
	char filepath[128];
	u32 shading_addr = 0;
	struct timespec ut;

	isp_debug_pu_start_tag();

	get_monotonic_boottime(&ut);

	for (sensor = SENSOR1; sensor < SENSOR_TOTAL; sensor++) {

		/* Get the address of the shading bin. */
		shading_addr =
			common_get_shading_data_addr_by_sensor_id(sensor);

		if (shading_addr == 0) {
			isp_debug_desc_tag(
				"addr. of sensor %u shading bin is 0",
				sensor+1);
			continue;
		}

#if DUMP_IRP_BIN_WITH_EXACT_SIZE
		/* Calculate the size of the shading bin. */
		size = calc_shading_bin_size(shading_addr);
#else
		size = ISP_SHADING_BIN_BUF_SIZE;
#endif
		isp_debug_item_tag(
			"sensor: %u, addr: 0x%x, size: %u",
			sensor+1, shading_addr, size);
		if (size == 0) {
			isp_debug_desc_tag("irp bin size for sensor %u is 0",
				sensor+1);
			continue;
		}

		/* Dump the shading bin */
		sprintf(filepath,
			DEFAULT_DUMP_PATH "[%06lu.%09lu]Sensor%uShading.bin",
			ut.tv_sec, ut.tv_nsec, sensor+1);
		err = dump_memory(filepath, shading_addr, size);
		isp_debug_item_tag("sensor: %u, dump result: %u",
			sensor+1, err);
	}

	isp_debug_pu_end_tag();

	return err;
}
EXPORT_SYMBOL(ispdrv_dump_shadingbin);
/*
 * /////////////////////////////////////////////////////////
 *		Local Function		*
 * /////////////////////////////////////////////////////////
*/
int dump_kva(char *filepath, void *_addr, u64 size) /* YuMJ */
{
	int err = 0;
	char *addr = (char *)_addr;
	mm_segment_t orig_fs = {0};
	struct file *filp = NULL;

	isp_debug_lo_start_tag();

	if (!filepath) {
		err = EINVAL;
		isp_alert("Null file path");
		goto dump_kva_end;
	}

	orig_fs = get_fs();
	set_fs(KERNEL_DS);

	isp_alert("filepath: %s", filepath);

	filp = filp_open(filepath, O_RDWR | O_CREAT, 0666);

	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		isp_alert("open file failed. err: %x", err);
		goto dump_kva_end_1;
	} else
		isp_alert("open file success!");

	isp_alert("size of the img :%llu", size);

	err = vfs_write(filp, addr, size, &filp->f_pos);
	if (err < 0)
		isp_alert("Write file failed.");

	filp_close(filp, NULL);

dump_kva_end_1:
	set_fs(orig_fs);

dump_kva_end:
	isp_alert("%d", err);
	isp_debug_lo_end_tag();

return err;
}

int dump_memory(char *filepath, u64 addr, u64 size)
{

	mm_segment_t oldfs = {0};
	loff_t offset = 0;
	struct file *filp = NULL;
	int err = 0;
	u64 i = 0, write_size = 0, start_addr;
	u8 *p_map;

	isp_debug_lo_start_tag();

	if (!filepath) {
		err = EINVAL;
		isp_err_lo_combo_desc_tag("Null file path");
		goto dump_memory_end;
	}

	addr += PHY_BASE_ADDR;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	isp_debug_item_tag("filepath: %s", filepath);

	filp = filp_open(filepath, O_RDWR | O_CREAT, 0666);

	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		isp_err_lo_combo_desc_tag("open file failed. It: err: %d", err);
		goto dump_memory_end;
	} else
		isp_debug_desc_tag("open file success!");

	isp_debug_item_tag("size of the img: %llu", size);

	/* write page size(4k) each time*/
	for (i = 0; i < size; i += PAGE_SIZE_4K) {
		write_size =
		    ((size - i) > PAGE_SIZE_4K) ? PAGE_SIZE_4K : (size - i);
		start_addr = addr + i;

		p_map = MAP_IVA_TO_KVA(start_addr, PAGE_SIZE_4K);

		if (p_map) {
			offset = filp->f_pos;
			err = vfs_write(filp, p_map, write_size, &offset);
			if (err < 0) {
				isp_alert_lo_combo_desc_tag(
					"Write file failed. err: %d", err);
				UNMAP_IVA_TO_KVA(p_map);
				goto dump_memory_end;
			}

			filp->f_pos = offset;
			UNMAP_IVA_TO_KVA(p_map);
		} else {
			isp_err_lo_combo_desc_tag("Mapping memory failed.");
			err = EINVAL;
			goto dump_memory_end;
		}
	}

dump_memory_end:
	isp_debug_err("%d", err);
	set_fs(oldfs);

	if (!IS_ERR(filp))
		filp_close(filp, NULL);

	isp_debug_lo_end_tag();

	return err;

}



/* dump specific size of ahb register from start*/
int dump_isp_ahb(char *filepath, u32 size)
{

#if 1
	mm_segment_t oldfs;
	loff_t offset;
	struct file *filp;
#endif

	int err = 0;
	u32 i = 0;
	/* u8 ahb_buffer[4096];*/
	u8 *ahb_buffer = NULL;
	u32 *write_ptr;

	isp_debug_lo_start_tag();

	/* ahb_buffer = kmalloc(size,GFP_KERNEL);*/

	ahb_buffer = kmalloc(size, GFP_KERNEL);

	if (!ahb_buffer) {
		err = EINVAL;
		goto dump_isp_ahb_end;
	}

	write_ptr = (u32 *) (&ahb_buffer[0]);
#if 1
	oldfs = get_fs();

	set_fs(KERNEL_DS);

	isp_debug_item_tag("filepath: %s", filepath);

	filp = filp_open(filepath, O_RDWR | O_CREAT, 0666);	/* 666*/

	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		isp_err_lo_combo_desc_tag("open file failed. It: err: %x",
			err);
		goto dump_isp_ahb_end;
	} else
		isp_debug_desc_tag("open file success!");

	isp_debug_item_tag("size of the img: %d", size);

#endif

	/* write page size(4k) each time*/
	for (i = 0; i < size && i < 0x1000; i += 4) {
		/* 0xa8 ~ 0xac is protected range which ap cannot access*/
		if (i == 0xa8 || i == 0xac)
			*write_ptr = 0xFFFFFFFF;
		else
			*write_ptr = isp_get_reg_value(i);
		write_ptr++;
	}
#if 1
	offset = filp->f_pos;
	err = vfs_write(filp, ahb_buffer, size, &offset);
	if (err == -1) {
		isp_err_lo_combo_desc_tag("Write file failed.");
		goto dump_isp_ahb_end;
	}
	filp->f_pos = offset;

	set_fs(oldfs);

	if (!IS_ERR(filp))
		filp_close(filp, NULL);
#endif

dump_isp_ahb_end:

	kfree((void *)ahb_buffer);

	isp_debug_lo_end_tag();

	return err;

}


/* dump specific size of ahb register from start*/
int dump_isp_internal_reg(char *filepath, u32 start, u32 size, u8 hwid)
{

#if 1
	mm_segment_t oldfs = {0};
	loff_t offset;
	struct file *filp = NULL;
#endif
	int err = 0;
	u32 reg_addr = 0;
	/* u8 buffer[DUMP_ARY_SIZE];*/
	u8 *buffer = NULL;
	u32 *write_ptr;
	u32 j, write_size;
	u32 reg_cnt;
#if 0
	struct timespec ut0, ut1;
	long long int t0, t1;
#endif

	isp_debug_lo_start_tag();

	buffer = kmalloc(sizeof(u8) * 512, GFP_KERNEL);
	if (!buffer) {
		err = EINVAL;
		goto dump_isp_internal_reg_end;
	}

	write_ptr = (u32 *) (&buffer[0]);

	if (!filepath) {
		err = EINVAL;
		isp_err_lo_combo_desc_tag("Null file path");
	} else {
#if 1
		oldfs = get_fs();
		set_fs(KERNEL_DS);

		isp_debug_item_tag("filepath: %s", filepath);

		filp = filp_open(filepath, O_RDWR | O_CREAT, 0666);

		if (IS_ERR(filp)) {
			err = PTR_ERR(filp);
			isp_err_lo_combo_desc_tag(
				"open file failed. It: err: %x",
				err);
			goto dump_isp_internal_reg_end;
		} else
			isp_debug_desc_tag("open file success!");

		isp_debug_item_tag("size of the img :%d", size);


		/* Write header, start address */
		offset = filp->f_pos;
		err = vfs_write(filp, (u8 *)(&start), 4, &offset);
		if (err == -1) {
			isp_err_lo_combo_desc_tag("Write file failed.");
			goto dump_isp_internal_reg_end;
		}
		filp->f_pos = offset;


		/* Write header, size */
		offset = filp->f_pos;
		reg_cnt = size/4;
		err = vfs_write(filp, (u8 *)(&reg_cnt), 4, &offset);
		if (err == -1) {
			isp_err_lo_combo_desc_tag("Write file failed.");
			goto dump_isp_internal_reg_end;
			return err;
		}
		filp->f_pos = offset;
#endif
		/*  Write DUMP_ARY_SIZE each time*/
		for (j = 0; size != 0; j++) {
			/*  To Get DUMP_ARY_SIZE or remain size*/
			write_size = (
			(size > DUMP_ARY_SIZE) ? DUMP_ARY_SIZE : size);
			write_ptr = (u32 *) &buffer[0];

			for (reg_addr = start + j * DUMP_ARY_SIZE;
			reg_addr < start + j * DUMP_ARY_SIZE + write_size;
			reg_addr += 4) {
				/* FIFO of I2C*/
				if (hw_info_table[hwid].ul_fifo_reg_num > 0) {
					if (reg_addr ==
					(start + j * DUMP_ARY_SIZE) + 0x18)
						*write_ptr = 0xF1F0F1F0;
				}
#if 0
				get_monotonic_boottime(&ut0);
#endif
				*write_ptr = get_ahb_indirect(reg_addr);
#if 0
				get_monotonic_boottime(&ut1);
				t0 = ut0.tv_sec * 1000000000 + ut0.tv_nsec;
				t1 = ut1.tv_sec * 1000000000 + ut1.tv_nsec;
				isp_dev_append_response("%x, %lu\n",
							reg_addr, t1 - t0);
#endif
				write_ptr++;
			}
			#if 1
			offset = filp->f_pos;
			err = vfs_write(filp, buffer, write_size, &offset);
			if (err == -1) {
				isp_err_lo_combo_desc_tag(
					"Write file failed.");
				goto dump_isp_internal_reg_end;
			}
			filp->f_pos = offset;
			#endif
			/*  Update size*/
			size -= write_size;
		}
	}

    #if 1
	set_fs(oldfs);

	if (!IS_ERR(filp))
		filp_close(filp, NULL);
    #endif
	kfree((void *)buffer);
dump_isp_internal_reg_end:

	isp_debug_lo_end_tag();

	return err;

}


/* dump the data when the format is raw10*/
void dump_raw(void)
{
	int i = 0;
	u8 format;
	u8 buffer_number[SENSOR_TOTAL][S_OUTPUT_IMAGE_TOTAL];

	isp_debug_lo_start_tag();

	memset((u8 *) buffer_number, 0, sizeof(buffer_number));

	common_get_format(SENSOR1, V4L2_ISP_IMG1, &format);
	common_get_buff_number(&buffer_number[0][0]);

	if (format == V4L2_ISP_OUTPUT_RAW10) {
#if 0
		if (g_uc_rear_scenario_id ==
		    V4L2_ISP_SCENARIO_DUAL_SHOT_FRONT_REAR) {
			for (i = 0; i < g_buffer_number[SENSOR1]; i++) {
				dump_img_by_index(SENSOR1, V4L2_ISP_IMG1, i);
				SLEEPRANGE(1000, 1500);
			}
			for (i = 0; i < g_buffer_number[SENSOR2]; i++) {
				dump_img_by_index(SENSOR2, V4L2_ISP_IMG1, i);
				SLEEPRANGE(1000, 1500);
			}
		} else if (g_uc_front_scenario_id ==
			   V4L2_ISP_SCENARIO_FRONT_PREVIEW) {
			for (i = 0; i < g_buffer_number[SENSOR2]; i++) {
				dump_img_by_index(SENSOR2, V4L2_ISP_IMG1, i);
				SLEEPRANGE(1000, 1500);
			}
		} else {
			for (i = 0; i < g_buffer_number[SENSOR1]; i++) {
				dump_img_by_index(SENSOR1, V4L2_ISP_IMG1, i);
				SLEEPRANGE(1000, 1500);
			}
		}
#endif
		for (i = 0; i < buffer_number[SENSOR1][V4L2_ISP_IMG1]; i++) {
			dump_img_by_index(SENSOR1, V4L2_ISP_IMG1, i);
			SLEEPRANGE(1000, 1500);
		}
	}

	isp_debug_lo_end_tag();
}

/* Dump output Img*/
void dump_output_img(void)
{
	int i = 0;
	u8 sensor_id;
	int img_count;
	int img_type;
	u8 buffer_number[SENSOR_TOTAL][S_OUTPUT_IMAGE_TOTAL];

	isp_debug_lo_start_tag();

	memset((u8 *) buffer_number, 0, sizeof(buffer_number));

	common_get_sensor_id(&sensor_id);
	common_get_buff_number(&buffer_number[0][0]);

	if (sensor_id == 0)	/* Sensor 1*/
		img_type = V4L2_ISP_IMG2;	/*  video*/
	else			/*  Sensor 2 and Sensor Lite*/
		img_type = V4L2_ISP_IMG1;	/*  LV*/

	img_count = buffer_number[sensor_id][img_type];

	for (i = 0; i < img_count; i++) {
		isp_debug_item_tag("DumpImgByIdx: %d", i);
		dump_img_by_index(sensor_id, img_type, i);
		SLEEPRANGE(1000, 1500);
	}

	/*  Dump statistics*/
	for (i = 0; i < img_count; i++) {
		isp_debug_item_tag("DumpStatistisByIdx: %d", i);
		dump_img_by_index(sensor_id, V4L2_ISP_STATITISTICS, i);
		SLEEPRANGE(1000, 1500);
	}

	isp_debug_lo_end_tag();
}

u32 dump_img_by_index(u8 a_uc_sensor, u8 a_uc_output_image,
			     u8 a_uc_index)
{
	u32 err = 0, size = 0, img_addr, field;
	char filepath[128];
	u16 width, height;
	u8 img_index, field_index;
	u32 output_field_table[48] = {0};

	isp_debug_lo_start_tag();

#ifdef SENSORLITE_SENSOR2
	if (a_uc_sensor == SENSOR3)
		a_uc_sensor = SENSOR2;
#endif

	/* isp_info("%s - start", __func__);*/
	common_get_out_field_table(&output_field_table[0]);
	img_index = a_uc_index;

	/* get the output buffer address*/
	field_index =
	    a_uc_sensor * V4L2_ISP_OUTPUT_IMAGE_TOTAL * V4L2_ISP_INDEX_TOTAL +
	    a_uc_output_image * V4L2_ISP_INDEX_TOTAL + img_index;
	if (field_index >= (sizeof(output_field_table) / sizeof(u32))) {
		err = EINVAL;
		goto dump_img_by_index_end;
	}

	field = output_field_table[field_index];
	img_addr = isp_get_field_value(field);

	if (a_uc_sensor == SENSOR1) {
		if (a_uc_output_image == V4L2_ISP_IMG1) {
			width =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_WIDTH);
			height =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG1_HEIGHT);
		} else if (a_uc_output_image == V4L2_ISP_IMG2) {
			width =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_WIDTH);
			height =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG2_HEIGHT);
		} else if (a_uc_output_image == V4L2_ISP_IMG3) {
			width =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_WIDTH);
			height =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR1_OUT_IMG3_HEIGHT);
		} else {	/*  statistic data*/
			/* 200k statistic report*/
			width = 100;
			height = 1024;
		}
	} else if (a_uc_sensor == SENSOR2) {
		if (a_uc_output_image == V4L2_ISP_IMG1) {
			width =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG1_WIDTH);
			height =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG1_HEIGHT);
		} else if (a_uc_output_image == V4L2_ISP_IMG2) {
			width =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG2_WIDTH);
			height =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG2_HEIGHT);
		} else if (a_uc_output_image == V4L2_ISP_IMG3) {
			width =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG3_WIDTH);
			height =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR2_OUT_IMG3_HEIGHT);
		} else {	/*  statistic data*/
			/* 200k statistic report*/
			width = 100;
			height = 1024;
		}
	} else {
		if (a_uc_output_image == V4L2_ISP_IMG1) {
			width =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR3_OUT_IMG1_WIDTH);
			height =
			    isp_get_field_value
			    (AHB_ITEM_SENSOR3_OUT_IMG1_HEIGHT);
		} else {	/*  statistic data*/
			/* 200k statistic report*/
			width = 100;
			height = 1024;
		}
	}

	size = width * height * 2;

	if (a_uc_output_image == V4L2_ISP_IMG1)
		sprintf(filepath, "/home/DumpImgLV%d.raw", a_uc_index);
	else if (a_uc_output_image == V4L2_ISP_IMG2)
		sprintf(filepath, "/home/DumpImgVIDEO%d.raw", a_uc_index);
	else if (a_uc_output_image == V4L2_ISP_IMG3)
		sprintf(filepath, "/home/DumpImgSTILL%d.raw", a_uc_index);
	else {
		sprintf(filepath, "/home/StatisticReport%d.raw", a_uc_index);
		isp_debug_item_tag("Img addr 0x%x\n", img_addr);
	}

	dump_memory(filepath, img_addr, size);

dump_img_by_index_end:

	isp_debug_err("%d", err);
	isp_debug_lo_end_tag();

	return err;
}
/*
 * /////////////////////////////////////////////////////////
 *		Private Function		*
 * /////////////////////////////////////////////////////////
*/

#if DUMP_IRP_BIN_WITH_EXACT_SIZE
static u32 calc_shading_bin_size(u32 bin_addr)
{
	return *((u32 *) MAP_IVA_TO_KVA(bin_addr + 20, 0));
}

static u32 calc_irp_bin_size(u32 bin_addr)
{
	u32 *map_addr1, *map_addr2;
	u32 mode_bin_add;
	u32 ui_bin_add;
	u32 pj_bin_add;

	map_addr1 = iva32_to_ka64(bin_addr + 4, 0);
	map_addr2 = iva32_to_ka64(bin_addr + 8, 0);
	mode_bin_add = bin_addr + *map_addr1 + *map_addr2;

	map_addr1 = iva32_to_ka64(mode_bin_add + 4, 0);
	ui_bin_add = mode_bin_add + *map_addr1;

	map_addr1 = iva32_to_ka64(ui_bin_add + 12, 0);
	pj_bin_add = ui_bin_add + *map_addr1;

	return (pj_bin_add + 42) - bin_addr;
}
#endif

