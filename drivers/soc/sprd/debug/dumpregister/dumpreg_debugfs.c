/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/debugfs.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <asm/io.h>
#include "bia_reg_dump.h"

#define DUMP_FILE "/system/dumpreg.bin"

#define BYTE16                     (UINT32_TYPE)(16)
#define BYTE32                     (UINT32_TYPE)(32)
#define BYTE64                     (UINT32_TYPE)(64)
#define BYTE128                    (UINT32_TYPE)(128)
#define WORD3                      (UINT32_TYPE)(3)
#define WORD4                      (UINT32_TYPE)(4)
#define DOUBLE_WORD2               (UINT32_TYPE)(2)
#define DOUBLE_WORD3               (UINT32_TYPE)(3)
#define DOUBLE_WORD4               (UINT32_TYPE)(4)

#define AP_AON_APB_BASE_ADDR       (UINT32_TYPE)(0xE42E0000)
#define AON_CHIP_ID0               (UINT32_TYPE)(AP_AON_APB_BASE_ADDR + 0xF8)
#define AON_CHIP_ID1               (UINT32_TYPE)(AP_AON_APB_BASE_ADDR + 0xFC)
#define MTRR_REGION_NUM_MAX 16

/******************** HEADER Definition ************************************/
struct FILE_HEADER_T {
	UINT64_TYPE chip_id;           /* chip id info, read from aon apb. */
	UINT8_TYPE  sys_name[BYTE16];  /* ap/pub_cp/cpag/cpwtl. */
	UINT8_TYPE  env_type[BYTE16];  /* get from C code / get from task. */
	UINT8_TYPE  ver_info[BYTE128]; /* record the version information. */
};

struct RECORD_HEADER_T {
	UINT8_TYPE  core_loc[BYTE16];        /*Incore Offcore.*/
	UINT8_TYPE  modu_name[BYTE64];       /*modu_name.*/
	UINT64_TYPE base_addr;               /*modu base address.*/
	UINT64_TYPE length;                  /*range length.*/
	UINT64_TYPE spare_addr;              /*modu spare address.*/
	UINT64_TYPE check_code;              /*check_code*/
	UINT64_TYPE reserved[DOUBLE_WORD2];  /*reserved.*/
};

/******************** IN Core Definition ***********************************/
struct BIA_INCORE_CFG_VAL_T {
	/*each modu header*/
	struct RECORD_HEADER_T record_header;
	/* AUNIT register */
	UINT32_TYPE io_ackgate;
	UINT32_TYPE io_aisochctl;
	UINT32_TYPE io_avcctl;
	UINT32_TYPE io_aarbctl0;
	/* BUNIT register */
	UINT32_TYPE io_barbctrl0;
	UINT32_TYPE io_barbctrl1;
	UINT32_TYPE io_barbctrl2;
	UINT32_TYPE io_barbctrl3;
	UINT32_TYPE io_bwflush;
	UINT32_TYPE io_bbankmask;
	UINT32_TYPE io_browmask;
	UINT32_TYPE io_brankmask;
	UINT32_TYPE io_balimit0;
	UINT32_TYPE io_balimit1;
	UINT32_TYPE io_balimit2;
	UINT32_TYPE io_balimit3;
	UINT32_TYPE io_bcoscat;
	UINT32_TYPE io_bmisc;
	UINT32_TYPE io_bc0ahashcfg;
	UINT32_TYPE io_bdebug0;
	UINT32_TYPE io_bctrl;
	UINT32_TYPE io_bthctrl;
	UINT32_TYPE io_bthmask;
	/* CUNIT register */
	UINT32_TYPE io_cunit_ssa;
	/* TUNIT register */
	UINT32_TYPE io_t_ctl;
	UINT32_TYPE io_t_misc_ctl;
	UINT32_TYPE io_t_clkgate_ctl;
	/* PMI2AXI register */
	UINT32_TYPE io_bmode0;
	UINT32_TYPE io_bdbug0;
	/* OCP2AXI register */
	UINT32_TYPE io_ocp_ctl;
	/* DVFS MSR register */
	UINT64_TYPE msr_punit_667h;                   /*0x667*/
	UINT64_TYPE msr_BIOS_SIGN_ID;                 /*0x8B*/
	UINT64_TYPE msr_CR_MISC_ENABLES;              /*0x1A0, b16: gs3_en*/
	UINT64_TYPE msr_CR_CST_CONFIG;                /*0xE2*/
	UINT64_TYPE msr_CR_GV3_CTRL;                  /*0x199*/
	UINT64_TYPE msr_CR_GV3_STAT;                  /*0x198*/
	UINT64_TYPE core_id;                          /*core id type, 0~7*/
	UINT32_TYPE cache_L1I_size;
	UINT32_TYPE cache_L1D_size;
	UINT32_TYPE cache_L2_size;
	UINT32_TYPE cache_L2_ways;
	UINT32_TYPE mtrr_regions;                     /*region numbers*/
	/*region enable flag*/
	UINT32_TYPE mtrr_region_en[MTRR_REGION_NUM_MAX];
	/*memory type, 0:UC, 1:WC, 4:WT, 5:WP, 6:WB*/
	UINT32_TYPE mtrr_region_attr[MTRR_REGION_NUM_MAX];
	/*region size flag*/
	UINT64_TYPE mtrr_region_size[MTRR_REGION_NUM_MAX];
	/*base address, 16 bytes aligned*/
	UINT64_TYPE mtrr_region_base[MTRR_REGION_NUM_MAX];
	/*default mtrr type*/
	UINT64_TYPE mtrr_def_type;
	UINT32_TYPE cpuid1_ecx;
	UINT32_TYPE cpuid1_edx;

};

/******************** OFF Core Definition **********************************/
#define VSP_SYS_AHB_RF_START_ADDR			(0xD1100000)
#define VSP_SYS_AHB_RF_SPARE_ADDR			(0xD1100000)
#define VSP_SYS_AHB_RF_LEN				(0x5C)

#define AP_APB_RF_START_ADDR				(0xE7B00000)
#define AP_APB_RF_SPARE_ADDR				(0xE7B00000)
#define AP_APB_RF_LEN					(0xC)

#define AP_AHB_RF_SEG0_START_ADDR			(0xE2210000)
#define AP_AHB_RF_SEG0_SPARE_ADDR			(0xE2210000)
#define AP_AHB_RF_SEG0_LEN				(0x30)

#define AP_AHB_RF_SEG1_START_ADDR			(0xE2213000)
#define AP_AHB_RF_SEG1_SPARE_ADDR			(0xE2213000)
#define AP_AHB_RF_SEG1_LEN				(0x9c)

#define AP_CLK_CORE_START_ADDR				(0xE2000000)
#define AP_CLK_CORE_SPARE_ADDR				(0xE2000000)
#define AP_CLK_CORE_LEN					(0x88)

#define CAM_AHB_RF_START_ADDR				(0xD2100000)
#define CAM_AHB_RF_SPARE_ADDR				(0xD2100000)
#define CAM_AHB_RF_LEN					(0x78)

#define GPU_SYS_APB_START_ADDR				(0xD0100000)
#define GPU_SYS_APB_SPARE_ADDR				(0xD0100000)
#define GPU_SYS_APB_LEN					(0x78)


#define DISP_AHP_RF_START_ADDR				(0xD3100000)
#define DISP_AHP_RF_SPARE_ADDR				(0xD3100000)
#define DISP_AHB_RF_LEN					(0xc)

#define CAM_CLK_CORE_START_ADDR				(0xD3000000)
#define CAM_CLK_CORE_SPARE_ADDR				(0xD3000000)
#define CAM_CLK_CORE_LEN				(0x78)


#define VSP_CLK_CORE_START_ADDR				(0xD1000000)
#define VSP_CLK_CORE_SPARE_ADDR				(0xD1000000)
#define VSP_CLK_CORE_LEN				(0x30)

#define AON_CLK_RF_START_ADDR				(0xE42D0200)
#define AON_CLK_RF_SPARE_ADDR				(0xE42D0200)
#define AON_CLK_RF_LEN					(0x160)

#define AON_APB_RF_SEG0_START_ADDR			(0xE42E0000)
#define AON_APB_RF_SEG0_SPARE_ADDR			(0xE42E0000)
#define AON_APB_RF_SEG0_LEN				(0x364)

#define AON_APB_RF_SEG1_START_ADDR			(0xE42E3000)
#define AON_APB_RF_SEG1_SPARE_ADDR			(0xE42E3000)
#define AON_APB_RF_SEG1_LEN				(0x318)

#define AON_LPC_APB_RF_START_ADDR			(0xE4290000)
#define AON_LPC_APB_RF_SPARE_ADDR			(0xE4290000)
#define AON_LPC_APB_RF_LEN				(0xA8)

#define PMU_APB_RF_START_ADDR				(0xE42B0000)
#define PMU_APB_RF_SPARE_ADDR				(0xE42B0000)
#define PMU_APB_RF_LEN					(0x2E8)

#define COM_PMU_APB_RF_START_ADDR			(0xE42C0000)
#define COM_PMU_APB_RF_SPARE_ADDR			(0xE42C0000)
#define COM_PMU_APB_RF_LEN				(0x228)

#define CADENCE_MC_AXI_SEG0_START_ADDR		(0xC0000000)
#define CADENCE_MC_AXI_SEG0_SPARE_ADDR		(0xC0000000)
#define CADENCE_MC_AXI_SEG0_LEN			(0x784)

#define CADENCE_MC_AXI_SEG1_START_ADDR		(0xC0002000)
#define CADENCE_MC_AXI_SEG1_SPARE_ADDR		(0xC0002000)
#define CADENCE_MC_AXI_SEG1_LEN			(0x1150)

#define PUB_APB_RF_PUB0_SEG0_START_ADDR		(0xC0010000)
#define PUB_APB_RF_PUB0_SEG0_SPARE_ADDR		(0xC0010000)
#define PUB_APB_RF_PUB0_SEG0_LEN		(0x404)

#define PUB_APB_RF_PUB0_SEG1_START_ADDR		(0xC0014000)
#define PUB_APB_RF_PUB0_SEG1_SPARE_ADDR		(0xC0014000)
#define PUB_APB_RF_PUB0_SEG1_LEN		(0x58)

#define PUB_APB_RF_PUB0_SEG2_START_ADDR		(0xC0018000)
#define PUB_APB_RF_PUB0_SEG2_SPARE_ADDR		(0xC0018000)
#define PUB_APB_RF_PUB0_SEG2_LEN		(0xCC)

struct AP_OFFCORE_CFG_VAL_T {
	struct RECORD_HEADER_T      vsp_sys_ahb_rf_record_header;
	UINT32_TYPE          vsp_sys_ahb_rf_list[VSP_SYS_AHB_RF_LEN / 4];

	struct RECORD_HEADER_T      ap_apb_rf_record_header;
	UINT32_TYPE          ap_apb_rf_list[AP_APB_RF_LEN / 4];

	struct RECORD_HEADER_T      ap_ahb_rf_seg0_record_header;
	UINT32_TYPE          ap_ahb_rf_seg0_list[AP_AHB_RF_SEG0_LEN / 4];

	struct RECORD_HEADER_T      ap_ahb_rf_seg1_record_header;
	UINT32_TYPE          ap_ahb_rf_seg1_list[AP_AHB_RF_SEG1_LEN / 4];

	struct RECORD_HEADER_T      ap_clk_core_record_header;
	UINT32_TYPE          ap_clk_core_list[AP_CLK_CORE_LEN / 4];

	struct RECORD_HEADER_T      cam_ahb_rf_record_header;
	UINT32_TYPE          cam_ahb_rf_list[CAM_AHB_RF_LEN / 4];

	struct RECORD_HEADER_T      gpu_sys_apb_record_header;
	UINT32_TYPE          gpu_sys_apb_list[GPU_SYS_APB_LEN / 4];

	struct RECORD_HEADER_T      disp_ahb_rf_record_header;
	UINT32_TYPE          disp_ahb_rf_list[DISP_AHB_RF_LEN / 4];

	struct RECORD_HEADER_T      cam_clk_core_record_header;
	UINT32_TYPE          cam_clk_core_list[CAM_CLK_CORE_LEN / 4];

	struct RECORD_HEADER_T      vsp_clk_core_record_header;
	UINT32_TYPE          vsp_clk_core_list[VSP_CLK_CORE_LEN / 4];

	struct RECORD_HEADER_T      aon_clk_rf_record_header;
	UINT32_TYPE          aon_clk_rf_list[AON_CLK_RF_LEN / 4];

	struct RECORD_HEADER_T      aon_apb_rf_seg0_record_header;
	UINT32_TYPE          aon_apb_rf_seg0_list[AON_APB_RF_SEG0_LEN / 4];

	struct RECORD_HEADER_T      aon_apb_rf_seg1_record_header;
	UINT32_TYPE          aon_apb_rf_seg1_list[AON_APB_RF_SEG1_LEN / 4];

	struct RECORD_HEADER_T      aon_lpc_apb_rf_record_header;
	UINT32_TYPE          aon_lpc_apb_rf_list[AON_LPC_APB_RF_LEN / 4];

	struct RECORD_HEADER_T      pmu_apb_rf_record_header;
	UINT32_TYPE          pmu_apb_rf_list[PMU_APB_RF_LEN / 4];

	struct RECORD_HEADER_T      com_pmu_apb_rf_record_header;
	UINT32_TYPE          com_pmu_apb_rf_list[COM_PMU_APB_RF_LEN / 4];

	struct RECORD_HEADER_T      cadence_mc_axi_seg0_record_header;
	UINT32_TYPE   cadence_mc_axi_seg0_list[CADENCE_MC_AXI_SEG0_LEN / 4];

	struct RECORD_HEADER_T      cadence_mc_axi_seg1_record_header;
	UINT32_TYPE   cadence_mc_axi_seg1_list[CADENCE_MC_AXI_SEG1_LEN / 4];

	struct RECORD_HEADER_T      pub_apb_rf_pub0_seg0_record_header;
	UINT32_TYPE   pub_apb_rf_pub0_seg0_list[PUB_APB_RF_PUB0_SEG0_LEN / 4];

	struct RECORD_HEADER_T      pub_apb_rf_pub0_seg1_record_header;
	UINT32_TYPE   pub_apb_rf_pub0_seg1_list[PUB_APB_RF_PUB0_SEG1_LEN / 4];

	struct RECORD_HEADER_T      pub_apb_rf_pub0_seg2_record_header;
	UINT32_TYPE   pub_apb_rf_pub0_seg2_list[PUB_APB_RF_PUB0_SEG2_LEN / 4];
};

void __iomem *IOMAP(UINT32_TYPE address, UINT32_TYPE length)
{
	void __iomem      *base;

	pr_info("iomap address=%x,iolength=%x\n", address, length);
	base = ioremap_nocache(address, length);
	return  base;
}

#define REG32_VAL_DUMP(base, offset)   (readl(base + offset))

struct AP_CFG_HEADER_T {
	struct FILE_HEADER_T file_dump_header;
	struct BIA_INCORE_CFG_VAL_T ap_dump_incore;
	struct AP_OFFCORE_CFG_VAL_T ap_dump_offcore;
};
/******************** Common Func ******************************************/
static const UINT32_TYPE crc32tab[] = {
	0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL,
	0x076dc419L, 0x706af48fL, 0xe963a535L, 0x9e6495a3L,
	0x0edb8832L, 0x79dcb8a4L, 0xe0d5e91eL, 0x97d2d988L,
	0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L, 0x90bf1d91L,

	0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
	0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L,
	0x136c9856L, 0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL,
	0x14015c4fL, 0x63066cd9L, 0xfa0f3d63L, 0x8d080df5L,

	0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L, 0xa2677172L,
	0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
	0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L,
	0x32d86ce3L, 0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L,

	0x26d930acL, 0x51de003aL, 0xc8d75180L, 0xbfd06116L,
	0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L, 0xb8bda50fL,
	0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
	0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL,

	0x76dc4190L, 0x01db7106L, 0x98d220bcL, 0xefd5102aL,
	0x71b18589L, 0x06b6b51fL, 0x9fbfe4a5L, 0xe8b8d433L,
	0x7807c9a2L, 0x0f00f934L, 0x9609a88eL, 0xe10e9818L,
	0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,

	0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL,
	0x6c0695edL, 0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L,
	0x65b0d9c6L, 0x12b7e950L, 0x8bbeb8eaL, 0xfcb9887cL,
	0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L, 0xfbd44c65L,

	0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
	0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL,
	0x4369e96aL, 0x346ed9fcL, 0xad678846L, 0xda60b8d0L,
	0x44042d73L, 0x33031de5L, 0xaa0a4c5fL, 0xdd0d7cc9L,

	0x5005713cL, 0x270241aaL, 0xbe0b1010L, 0xc90c2086L,
	0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
	0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L,
	0x59b33d17L, 0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL,

	0xedb88320L, 0x9abfb3b6L, 0x03b6e20cL, 0x74b1d29aL,
	0xead54739L, 0x9dd277afL, 0x04db2615L, 0x73dc1683L,
	0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
	0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L,

	0xf00f9344L, 0x8708a3d2L, 0x1e01f268L, 0x6906c2feL,
	0xf762575dL, 0x806567cbL, 0x196c3671L, 0x6e6b06e7L,
	0xfed41b76L, 0x89d32be0L, 0x10da7a5aL, 0x67dd4accL,
	0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,

	0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L,
	0xd1bb67f1L, 0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL,
	0xd80d2bdaL, 0xaf0a1b4cL, 0x36034af6L, 0x41047a60L,
	0xdf60efc3L, 0xa867df55L, 0x316e8eefL, 0x4669be79L,

	0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
	0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL,
	0xc5ba3bbeL, 0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L,
	0xc2d7ffa7L, 0xb5d0cf31L, 0x2cd99e8bL, 0x5bdeae1dL,

	0x9b64c2b0L, 0xec63f226L, 0x756aa39cL, 0x026d930aL,
	0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
	0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L,
	0x92d28e9bL, 0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L,

	0x86d3d2d4L, 0xf1d4e242L, 0x68ddb3f8L, 0x1fda836eL,
	0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L, 0x18b74777L,
	0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
	0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L,

	0xa00ae278L, 0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L,
	0xa7672661L, 0xd06016f7L, 0x4969474dL, 0x3e6e77dbL,
	0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L, 0x37d83bf0L,
	0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,

	0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L,
	0xbad03605L, 0xcdd70693L, 0x54de5729L, 0x23d967bfL,
	0xb3667a2eL, 0xc4614ab8L, 0x5d681b02L, 0x2a6f2b94L,
	0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL, 0x2d02ef8dL,
};

static struct AP_CFG_HEADER_T ap_cfg_header_p;
/******************** Header Func ******************************************/
UINT32_TYPE  get_check_code(struct BIA_INCORE_CFG_VAL_T *incore_cfg_p)
{
	UINT32_TYPE crc;
	UINT32_TYPE size;
	UINT32_TYPE i;
	unsigned char *p;

	i = 0;
	crc = 0xFFFFFFFF;
	size = incore_cfg_p->record_header.length;
	p = (unsigned char *)incore_cfg_p;

	p += 0x80;
	for (i = 0; i < size; i++)
		crc = crc32tab[(crc^(*(p+i))) & 0xFF] ^ (crc >> 8);

	return (crc ^ 0xFFFFFFFF);
}

void AP_Cfg_Header_Val(void)
{
	void __iomem *base0;
	void __iomem *base1;
	struct FILE_HEADER_T *dump_header;
	UINT64_TYPE value0;
	UINT64_TYPE value1;

	pr_info("start to dump header\n");

	base0 = IOMAP(AON_CHIP_ID1, 0x04);
	base1 = IOMAP(AON_CHIP_ID0, 0x04);
	dump_header = &(ap_cfg_header_p.file_dump_header);
	value0 = (UINT64_TYPE)REG32_VAL_DUMP(base0, 0);
	value1 = (UINT64_TYPE)REG32_VAL_DUMP(base1, 0);
	dump_header->chip_id = (value1 << 32) | value0;
	strcpy(dump_header->sys_name, "AP");
	strcpy(dump_header->env_type, "C");
	strcpy(dump_header->ver_info, "v0.1.2");
}

/******************** Incore Func ******************************************/
void dump_mtrr_config(void)
{
	UINT32_TYPE i;
	UINT64_TYPE reg_base;
	UINT64_TYPE reg_mask;
	struct BIA_INCORE_CFG_VAL_T *incore_cfg;

	incore_cfg = &(ap_cfg_header_p.ap_dump_incore);
	incore_cfg->mtrr_def_type = perf_read_msr(0x2FF);
	incore_cfg->mtrr_regions = perf_read_msr(0xFE) & 0xFF;

	for (i = 0; i < incore_cfg->mtrr_regions; i++) {
		UINT64_TYPE size_temp;

		reg_base = perf_read_msr(0x200 + i * 2);
		reg_mask = perf_read_msr(0x200 + i * 2 + 1);

		incore_cfg->mtrr_region_en[i] = (reg_mask >> 11) & 0x1;
		incore_cfg->mtrr_region_base[i] = (reg_base >> 12) << 12;
		incore_cfg->mtrr_region_attr[i] = reg_base & 0xFF;
		size_temp = ((~(reg_mask & ~0xFFF) + 1) & 0xFFFFFF000LL);
		incore_cfg->mtrr_region_size[i] = size_temp;
	}
}

void dump_dvfs_config(void)
{
	UINT32_TYPE msr_no;
	struct BIA_INCORE_CFG_VAL_T *incore_cfg;

	incore_cfg = &(ap_cfg_header_p.ap_dump_incore);

	msr_no = 0x667;
	incore_cfg->msr_punit_667h = perf_read_msr(msr_no);

	msr_no = 0x8B;
	incore_cfg->msr_BIOS_SIGN_ID = perf_read_msr(msr_no);

	msr_no = 0x1A0;
	incore_cfg->msr_CR_MISC_ENABLES = perf_read_msr(msr_no);

	msr_no = 0xE2;
	incore_cfg->msr_CR_CST_CONFIG = perf_read_msr(msr_no);

	msr_no = 0x199;
	incore_cfg->msr_CR_GV3_CTRL = perf_read_msr(msr_no);

	msr_no = 0x198;
	incore_cfg->msr_CR_GV3_STAT = perf_read_msr(msr_no);
}

void dump_cache_config(void)
{
	UINT32_TYPE ways[3], part, line, sets, level, size[3];
	CPUID_regs pr[3];
	struct BIA_INCORE_CFG_VAL_T *incore_cfg;

	incore_cfg = &(ap_cfg_header_p.ap_dump_incore);

	for (level = 0; level < 3; level++) {
		perf_get_cpuid(4, &pr[level], level);
		ways[level] = 1 + ((pr[level].ebx >> 22) & 0x3FF);
		part = 1 + ((pr[level].ebx >> 12) & 0x3FF);
		line = 1 + ((pr[level].ebx) & 0xFFF);
		sets = 1 + pr[level].ecx;

		size[level] = ways[level] * part * line * sets;
	}

	incore_cfg->cache_L1I_size = size[1];
	incore_cfg->cache_L1D_size = size[0];
	incore_cfg->cache_L2_size = size[2];
	incore_cfg->cache_L2_ways = ways[2];
}

void dump_cpuid_info(void)
{
	CPUID_regs pr;

	perf_get_cpuid(1, &pr, 0);
	ap_cfg_header_p.ap_dump_incore.cpuid1_ecx = pr.ecx;
	ap_cfg_header_p.ap_dump_incore.cpuid1_edx = pr.edx;
}

static UINT32_TYPE iosf_reg_read(UINT32_TYPE portid, UINT32_TYPE offset)
{
	UINT32_TYPE mcr;
	UINT32_TYPE mdr;

	mcr = (UINT32_TYPE)(0x06000000 + (portid << 16) + (offset << 8) + 0xFF);
	asm __volatile__(
		"mov $0x0cf8, %%dx\n\t"
		"mov $0x0800000d0, %%eax\n\t"
		"out %%eax, %%dx\n\t"
		"mov $0x0cfc, %%edx\n\t"
		"mov %1, %%eax\n\t"
		"out %%eax, %%dx\n\t"
		"mov $0x0cf8, %%dx\n\t"
		"mov $0x0800000d4, %%eax\n\t"
		"out %%eax, %%dx\n\t"
		"mov $0x0cfc, %%dx\n\t"
		"in %%dx, %%eax\n\t"
		"mov %%eax, %0"
		: "=r"(mdr)
		: "r"(mcr)
		: "%eax", "%edx"
		);

	return mdr;
}

void dump_uncore_config(void)
{
	struct BIA_INCORE_CFG_VAL_T *incore_cfg;

	incore_cfg = &(ap_cfg_header_p.ap_dump_incore);

	incore_cfg->io_ackgate = iosf_reg_read(AUNIT_PORT, ACKGATE);
	incore_cfg->io_aisochctl = iosf_reg_read(AUNIT_PORT, AISOCHCTL);
	incore_cfg->io_avcctl = iosf_reg_read(AUNIT_PORT, AVCCTL);
	incore_cfg->io_aarbctl0 = iosf_reg_read(AUNIT_PORT, AARBCTL0);

	incore_cfg->io_barbctrl0 = iosf_reg_read(BUNIT_PORT, BARBCTRL0);
	incore_cfg->io_barbctrl1 = iosf_reg_read(BUNIT_PORT, BARBCTRL1);
	incore_cfg->io_barbctrl2 = iosf_reg_read(BUNIT_PORT, BARBCTRL2);
	incore_cfg->io_barbctrl3 = iosf_reg_read(BUNIT_PORT, BARBCTRL3);

	incore_cfg->io_bwflush = iosf_reg_read(BUNIT_PORT, BWFLUSH);
	incore_cfg->io_bbankmask = iosf_reg_read(BUNIT_PORT, BBANKMASK);
	incore_cfg->io_browmask = iosf_reg_read(BUNIT_PORT, BROWMASK);
	incore_cfg->io_brankmask = iosf_reg_read(BUNIT_PORT, BRANKMASK);
	incore_cfg->io_balimit0 = iosf_reg_read(BUNIT_PORT, BALIMIT0);
	incore_cfg->io_balimit1 = iosf_reg_read(BUNIT_PORT, BALIMIT1);
	incore_cfg->io_balimit2 = iosf_reg_read(BUNIT_PORT, BALIMIT2);
	incore_cfg->io_balimit3 = iosf_reg_read(BUNIT_PORT, BALIMIT3);

	incore_cfg->io_bcoscat = iosf_reg_read(BUNIT_PORT, BCOSCAT);
	incore_cfg->io_bmisc = iosf_reg_read(BUNIT_PORT, BMISC);
	incore_cfg->io_bc0ahashcfg = iosf_reg_read(AUNIT_PORT, BC0AHASHCFG);
	incore_cfg->io_bdebug0 = iosf_reg_read(BUNIT_PORT, BDEBUG0);
	incore_cfg->io_bctrl = iosf_reg_read(BUNIT_PORT, BCTRL);
	incore_cfg->io_bthctrl = iosf_reg_read(BUNIT_PORT, BTHCTRL);
	incore_cfg->io_bthmask = iosf_reg_read(BUNIT_PORT, BTHMASK);

	incore_cfg->io_cunit_ssa = iosf_reg_read(CUNIT_PORT, CUNIT_SSA_REG);

	incore_cfg->io_t_ctl = iosf_reg_read(TUNIT_PORT, T_CTL);
	incore_cfg->io_t_misc_ctl = iosf_reg_read(TUNIT_PORT, T_MISC_CTL);
	incore_cfg->io_t_clkgate_ctl = iosf_reg_read(TUNIT_PORT, T_CLKGATE_CTL);

	incore_cfg->io_bmode0 = iosf_reg_read(PMI2AXI_PORT, BMODE0);
	incore_cfg->io_bdbug0 = iosf_reg_read(PMI2AXI_PORT, BDBUG0);
	incore_cfg->io_ocp_ctl = iosf_reg_read(IOSF2OCP_PORT, OCP_CTL);
}

#define LAPIC_PADDR_BASE   0xFEE00000
#define LAPIC_LEN  0x4

static UINT32_TYPE get_coreid(void)
{
	/* LAPIC_IDR*/
	void __iomem *base;
	UINT32_TYPE apicID;

	base = IOMAP(LAPIC_PADDR_BASE, LAPIC_LEN);
	/*Read the cores APIC ID*/
	apicID = (REG32_VAL_DUMP(base, 0x20)) >> 25;
	return apicID;
}

static void AP_Dump_Incore_Cfg_Val(void)
{
	struct BIA_INCORE_CFG_VAL_T *incore_cfg;
	size_t incore_cfg_len, recore_header_len;

	incore_cfg = &(ap_cfg_header_p.ap_dump_incore);
	pr_info("start to dump incore reg\n");

	strcpy(incore_cfg->record_header.core_loc, "Incore");
	strcpy(incore_cfg->record_header.modu_name, "Incore");
	incore_cfg_len = sizeof(struct BIA_INCORE_CFG_VAL_T);
	recore_header_len = sizeof(struct RECORD_HEADER_T);
	incore_cfg->record_header.base_addr = 0xFF000000;
	incore_cfg->record_header.length = incore_cfg_len - recore_header_len;
	incore_cfg->record_header.spare_addr = 0xFF000000;
	incore_cfg->core_id = get_coreid();

	pr_info("start to dump cpuid info\n");
	dump_cpuid_info();
	pr_info("start to dump dvfs reg\n");
	dump_dvfs_config();
	pr_info("start to dump cache reg\n");
	dump_cache_config();
	pr_info("start to dump mtrr reg\n");
	dump_mtrr_config();
	pr_info("start to dump uncore reg\n");
	dump_uncore_config();
	pr_info("start  to dump caculate the crc\n");
	incore_cfg->record_header.check_code = get_check_code(incore_cfg);
	pr_info("end to dump incore value\n");
}

/******************** OFFCore Func *****************************************/
void AP_Dump_Offcore_Cfg_Val(void)
{
	UINT32_TYPE  i, j, k, crc;
	UINT32_TYPE tmp_value;
	UINT8_TYPE tmp_char;
	void __iomem *base;
	struct AP_OFFCORE_CFG_VAL_T *offcore_cfg_p;
	struct RECORD_HEADER_T *header_tmp;

	pr_info("start to dump OffCore reg\n");
	offcore_cfg_p = &(ap_cfg_header_p.ap_dump_offcore);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->vsp_sys_ahb_rf_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "vsp_sys_ahb_rf");
	header_tmp->base_addr = VSP_SYS_AHB_RF_START_ADDR;
	header_tmp->length = VSP_SYS_AHB_RF_LEN;
	header_tmp->spare_addr = VSP_SYS_AHB_RF_SPARE_ADDR;
	base = IOMAP(VSP_SYS_AHB_RF_SPARE_ADDR, VSP_SYS_AHB_RF_LEN);
	for (i = 0; i < VSP_SYS_AHB_RF_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->vsp_sys_ahb_rf_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->ap_apb_rf_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "ap_apb_rf");
	header_tmp->base_addr = AP_APB_RF_START_ADDR;
	header_tmp->length = AP_APB_RF_LEN;
	header_tmp->spare_addr = AP_APB_RF_SPARE_ADDR;
	base = IOMAP(AP_APB_RF_SPARE_ADDR, AP_APB_RF_LEN);
	for (i = 0; i < AP_APB_RF_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->ap_apb_rf_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->ap_ahb_rf_seg0_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "ap_ahb_rf_seg0");
	header_tmp->base_addr = AP_AHB_RF_SEG0_START_ADDR;
	header_tmp->length = AP_AHB_RF_SEG0_LEN;
	header_tmp->spare_addr = AP_AHB_RF_SEG0_SPARE_ADDR;
	base = IOMAP(AP_AHB_RF_SEG0_SPARE_ADDR, AP_AHB_RF_SEG0_LEN);
	for (i = 0; i < AP_AHB_RF_SEG0_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->ap_ahb_rf_seg0_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->ap_ahb_rf_seg1_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "ap_ahb_rf_seg1");
	header_tmp->base_addr = AP_AHB_RF_SEG1_START_ADDR;
	header_tmp->length = AP_AHB_RF_SEG1_LEN;
	header_tmp->spare_addr = AP_AHB_RF_SEG1_SPARE_ADDR;
	base = IOMAP(AP_AHB_RF_SEG1_SPARE_ADDR, AP_AHB_RF_SEG1_LEN);
	for (i = 0; i < AP_AHB_RF_SEG1_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->ap_ahb_rf_seg1_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->ap_clk_core_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "ap_clk_core");
	header_tmp->base_addr = AP_CLK_CORE_START_ADDR;
	header_tmp->length = AP_CLK_CORE_LEN;
	header_tmp->spare_addr = AP_CLK_CORE_SPARE_ADDR;
	base = IOMAP(AP_CLK_CORE_SPARE_ADDR, AP_CLK_CORE_LEN);
	for (i = 0; i < AP_CLK_CORE_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->ap_clk_core_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->cam_ahb_rf_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "cam_ahb_rf");
	header_tmp->base_addr = CAM_AHB_RF_START_ADDR;
	header_tmp->length = CAM_AHB_RF_LEN;
	header_tmp->spare_addr = CAM_AHB_RF_SPARE_ADDR;
	base = IOMAP(CAM_AHB_RF_SPARE_ADDR, CAM_AHB_RF_LEN);
	for (i = 0; i < CAM_AHB_RF_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->cam_ahb_rf_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->gpu_sys_apb_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "gpu_sys_apb");
	header_tmp->base_addr = GPU_SYS_APB_START_ADDR;
	header_tmp->length = GPU_SYS_APB_LEN;
	header_tmp->spare_addr = GPU_SYS_APB_SPARE_ADDR;
	base = IOMAP(GPU_SYS_APB_SPARE_ADDR, GPU_SYS_APB_LEN);
	for (i = 0; i < GPU_SYS_APB_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->gpu_sys_apb_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->disp_ahb_rf_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "disp_ahb_rf");
	header_tmp->base_addr = DISP_AHP_RF_START_ADDR;
	header_tmp->length = DISP_AHB_RF_LEN;
	header_tmp->spare_addr = DISP_AHP_RF_SPARE_ADDR;
	base = IOMAP(DISP_AHP_RF_SPARE_ADDR, DISP_AHB_RF_LEN);
	for (i = 0; i < DISP_AHB_RF_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->disp_ahb_rf_list[j] = tmp_value;
		for (k = 0; k < 4; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->cam_clk_core_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "cam_clk_core");
	header_tmp->base_addr = CAM_CLK_CORE_START_ADDR;
	header_tmp->length = CAM_CLK_CORE_LEN;
	header_tmp->spare_addr = CAM_CLK_CORE_SPARE_ADDR;
	base = IOMAP(CAM_CLK_CORE_SPARE_ADDR, CAM_CLK_CORE_LEN);
	for (i = 0; i < CAM_CLK_CORE_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->cam_clk_core_list[j] = tmp_value;
		for (k = 0; k < 4; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->vsp_clk_core_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "vsp_clk_core");
	header_tmp->base_addr = VSP_CLK_CORE_START_ADDR;
	header_tmp->length = VSP_CLK_CORE_LEN;
	header_tmp->spare_addr = VSP_CLK_CORE_SPARE_ADDR;
	base = IOMAP(VSP_CLK_CORE_SPARE_ADDR, VSP_CLK_CORE_LEN);
	for (i = 0; i < VSP_CLK_CORE_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->vsp_clk_core_list[j] = tmp_value;
		for (k = 0; k < 4; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->aon_clk_rf_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "aon_clk_rf");
	header_tmp->base_addr = AON_CLK_RF_START_ADDR;
	header_tmp->length = AON_CLK_RF_LEN;
	header_tmp->spare_addr = AON_CLK_RF_SPARE_ADDR;
	base = IOMAP(AON_CLK_RF_SPARE_ADDR, AON_CLK_RF_LEN);
	for (i = 0; i < AON_CLK_RF_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->aon_clk_rf_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);


	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->aon_apb_rf_seg0_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "aon_apb_rf_seg0");
	header_tmp->base_addr = AON_APB_RF_SEG0_START_ADDR;
	header_tmp->length = AON_APB_RF_SEG0_LEN;
	header_tmp->spare_addr = AON_APB_RF_SEG0_SPARE_ADDR;
	base = IOMAP(AON_APB_RF_SEG0_SPARE_ADDR, AON_APB_RF_SEG0_LEN);
	for (i = 0; i < AON_APB_RF_SEG0_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->aon_apb_rf_seg0_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->aon_apb_rf_seg1_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "aon_apb_rf_seg1");
	header_tmp->base_addr = AON_APB_RF_SEG1_START_ADDR;
	header_tmp->length = AON_APB_RF_SEG1_LEN;
	header_tmp->spare_addr = AON_APB_RF_SEG1_SPARE_ADDR;
	base = IOMAP(AON_APB_RF_SEG1_SPARE_ADDR, AON_APB_RF_SEG1_LEN);
	for (i = 0; i < AON_APB_RF_SEG1_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->aon_apb_rf_seg1_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->aon_lpc_apb_rf_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "aon_lpc_apb_rf");
	header_tmp->base_addr = AON_LPC_APB_RF_START_ADDR;
	header_tmp->length = AON_LPC_APB_RF_LEN;
	header_tmp->spare_addr = AON_LPC_APB_RF_SPARE_ADDR;
	base = IOMAP(AON_LPC_APB_RF_SPARE_ADDR, AON_LPC_APB_RF_LEN);
	for (i = 0; i < AON_LPC_APB_RF_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->aon_lpc_apb_rf_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->pmu_apb_rf_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "pmu_apb_rf");
	header_tmp->base_addr = PMU_APB_RF_START_ADDR;
	header_tmp->length = PMU_APB_RF_LEN;
	header_tmp->spare_addr = PMU_APB_RF_SPARE_ADDR;
	base = IOMAP(PMU_APB_RF_SPARE_ADDR, PMU_APB_RF_LEN);
	for (i = 0; i < PMU_APB_RF_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->pmu_apb_rf_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->com_pmu_apb_rf_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "com_pmu_apb_rf");
	header_tmp->base_addr = COM_PMU_APB_RF_START_ADDR;
	header_tmp->length = COM_PMU_APB_RF_LEN;
	header_tmp->spare_addr = COM_PMU_APB_RF_SPARE_ADDR;
	base = IOMAP(COM_PMU_APB_RF_SPARE_ADDR, COM_PMU_APB_RF_LEN);
	for (i = 0; i < COM_PMU_APB_RF_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->com_pmu_apb_rf_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->cadence_mc_axi_seg0_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "cadence_mc_axi_reg_seg0");
	header_tmp->base_addr = CADENCE_MC_AXI_SEG0_START_ADDR;
	header_tmp->length = CADENCE_MC_AXI_SEG0_LEN;
	header_tmp->spare_addr = CADENCE_MC_AXI_SEG0_SPARE_ADDR;
	base = IOMAP(CADENCE_MC_AXI_SEG0_SPARE_ADDR, CADENCE_MC_AXI_SEG0_LEN);
	for (i = 0; i < CADENCE_MC_AXI_SEG0_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->cadence_mc_axi_seg0_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->cadence_mc_axi_seg1_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "cadence_mc_axi_reg_seg1");
	header_tmp->base_addr = CADENCE_MC_AXI_SEG1_START_ADDR;
	header_tmp->length = CADENCE_MC_AXI_SEG1_LEN;
	header_tmp->spare_addr = CADENCE_MC_AXI_SEG1_SPARE_ADDR;
	base = IOMAP(CADENCE_MC_AXI_SEG1_SPARE_ADDR, CADENCE_MC_AXI_SEG1_LEN);
	for (i = 0; i < CADENCE_MC_AXI_SEG1_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->cadence_mc_axi_seg1_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->pub_apb_rf_pub0_seg0_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "pub_apb_rf_pub0_within_fw_top_seg0");
	header_tmp->base_addr = PUB_APB_RF_PUB0_SEG0_START_ADDR;
	header_tmp->length = PUB_APB_RF_PUB0_SEG0_LEN;
	header_tmp->spare_addr = PUB_APB_RF_PUB0_SEG0_SPARE_ADDR;
	base = IOMAP(PUB_APB_RF_PUB0_SEG0_SPARE_ADDR, PUB_APB_RF_PUB0_SEG0_LEN);
	for (i = 0; i < PUB_APB_RF_PUB0_SEG0_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->pub_apb_rf_pub0_seg0_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->pub_apb_rf_pub0_seg1_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "pub_apb_rf_pub0_within_fw_top_seg1");
	header_tmp->base_addr = PUB_APB_RF_PUB0_SEG1_START_ADDR;
	header_tmp->length = PUB_APB_RF_PUB0_SEG1_LEN;
	header_tmp->spare_addr = PUB_APB_RF_PUB0_SEG1_SPARE_ADDR;
	base = IOMAP(PUB_APB_RF_PUB0_SEG1_SPARE_ADDR, PUB_APB_RF_PUB0_SEG1_LEN);
	for (i = 0; i < PUB_APB_RF_PUB0_SEG1_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->pub_apb_rf_pub0_seg1_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

	j = 0;
	crc = 0xFFFFFFFF;
	header_tmp = &(offcore_cfg_p->pub_apb_rf_pub0_seg2_record_header);
	strcpy(header_tmp->core_loc, "OffCore");
	strcpy(header_tmp->modu_name, "pub_apb_rf_pub0_within_fw_top_seg2");
	header_tmp->base_addr = PUB_APB_RF_PUB0_SEG2_START_ADDR;
	header_tmp->length = PUB_APB_RF_PUB0_SEG2_LEN;
	header_tmp->spare_addr = PUB_APB_RF_PUB0_SEG2_SPARE_ADDR;
	base = IOMAP(PUB_APB_RF_PUB0_SEG2_SPARE_ADDR, PUB_APB_RF_PUB0_SEG2_LEN);
	for (i = 0; i < PUB_APB_RF_PUB0_SEG2_LEN; i += 4) {
		tmp_value = REG32_VAL_DUMP(base, i);
		offcore_cfg_p->pub_apb_rf_pub0_seg2_list[j] = tmp_value;
		for (k = 0; k < 4 ; k++) {
			tmp_char = (tmp_value >> (8 * k)) & 0xff;
			crc = crc32tab[(crc ^ tmp_char) & 0xff] ^ (crc >> 8);
		}
		j++;
	}
	header_tmp->check_code = (crc ^ 0xFFFFFFFF);

}

/******************** entring function and save file ***********************/
static int write_fs(void)
{
	struct file *file = NULL;
	mm_segment_t old_fs;
	size_t file_size;

	pr_info("the module that intends to write messages to file.\n");

	if (file == NULL)
		file = filp_open(DUMP_FILE, O_RDWR | O_CREAT, 0644);

	if (IS_ERR(file)) {
		pr_info("error while opening file %s, exiting\n", DUMP_FILE);
		return 0;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	file_size = sizeof(struct AP_CFG_HEADER_T);
	__vfs_write(file, (char *)&ap_cfg_header_p, file_size, &file->f_pos);
	set_fs(old_fs);
	filp_close(file, NULL);
	file = NULL;
	return 0;
}

static int dump_register(void)
{
	pr_info("dump register start\n");
	memset(&ap_cfg_header_p, 0, sizeof(struct AP_CFG_HEADER_T));

	AP_Cfg_Header_Val();
	pr_info("dump header end\n");
	AP_Dump_Offcore_Cfg_Val();
	pr_info("dump Offcore end\n");
	AP_Dump_Incore_Cfg_Val();
	pr_info("dump Incore end\n");

	write_fs();
	pr_info("write file completed.\n");
	return 0;
}

#define DEVICE_NAME ("perfchecker")
#define NODE_NAME ("dumpreg_trigger")
struct dentry *root_dentry;
struct dentry *sub_dentry;

static int dumpreg_func(void *data, u64 value)
{
	if (value == 1) {
		pr_info("start to dump\n");
		dump_register();
	} else if (value == 0)
		pr_info("end to show\n");
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debugfs_fops, NULL, dumpreg_func, "%llu\n");

#define PARAM1   (NODE_NAME, 0644, root_dentry, NULL, &debugfs_fops)
static int __init dumpreg_debugfs_init(void)
{
	pr_info("dumpreg_debugfs_init\n");
	root_dentry = debugfs_create_dir(DEVICE_NAME, NULL);
	if (root_dentry != NULL)
		sub_dentry = debugfs_create_file PARAM1;

	return 0;
}

static void __exit dumpreg_debugfs_exit(void)
{
	pr_info("dumpreg_debugfs_exit\n");
	debugfs_remove(sub_dentry);
	debugfs_remove(root_dentry);
}

MODULE_LICENSE("GPL");
module_init(dumpreg_debugfs_init);
module_exit(dumpreg_debugfs_exit);
