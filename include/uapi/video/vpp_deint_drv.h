/*
 * Copyright (C) 2014 Spreadtrum Communications Inc.
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

#ifndef _SPRD_VPP_H
#define _SPRD_VPP_H

#include <linux/ioctl.h>

/*----------------------------------------------------------------------------*
**                            Macro Definitions                               *
**---------------------------------------------------------------------------*/

#define DEINT_CLK_LEVEL_NUM 4
#define SPRD_VPP_MAP_SIZE 0x4000

#define SPRD_VPP_IOCTL_MAGIC 'd'
#define SPRD_VPP_ALLOCATE_PHYSICAL_MEMORY _IO(SPRD_VPP_IOCTL_MAGIC, 1)
#define SPRD_VPP_FREE_PHYSICAL_MEMORY     _IO(SPRD_VPP_IOCTL_MAGIC, 2)
#define SPRD_VPP_DEINT_COMPLETE           _IO(SPRD_VPP_IOCTL_MAGIC, 3)
#define SPRD_VPP_DEINT_ACQUIRE            _IO(SPRD_VPP_IOCTL_MAGIC, 4)
#define SPRD_VPP_DEINT_RELEASE            _IO(SPRD_VPP_IOCTL_MAGIC, 5)
#define SPRD_VPP_RESET                    _IO(SPRD_VPP_IOCTL_MAGIC, 6)
#define SPRD_VPP_GET_IOVA _IOWR(SPRD_VPP_IOCTL_MAGIC, 7, struct vsp_iommu_map_data)
#define SPRD_VPP_FREE_IOVA _IOW(SPRD_VPP_IOCTL_MAGIC, 8, struct vsp_iommu_map_data)
#define SPRD_VPP_GET_IOMMU_STATUS         _IO(SPRD_VPP_IOCTL_MAGIC, 9)

#ifdef CONFIG_COMPAT
#define COMPAT_VPP_GET_IOVA _IOWR(SPRD_VPP_IOCTL_MAGIC, 7, struct compat_vsp_iommu_map_data)
#define COMPAT_VPP_FREE_IOVA _IOW(SPRD_VPP_IOCTL_MAGIC, 8, struct compat_vsp_iommu_map_data)

struct compat_vsp_iommu_map_data {
	compat_int_t fd;
	compat_size_t size;
	compat_ulong_t iova_addr;
};
#endif

struct vsp_iommu_map_data {
	int fd;
	size_t size;
	unsigned long iova_addr;
};

#define VPP_CFG     (0x0000)
#define VPP_INT_STS (0x0010)
#define VPP_INT_MSK (0x0014)
#define VPP_INT_CLR (0x0018)
#define VPP_INT_RAW (0x001C)
#define DEINT_PATH_CFG   (0x2000)
#define DEINT_PATH_START (0x2004)
#define DEINT_IMG_SIZE   (0x2008)

typedef enum {
	SHARK = 0,
	DOLPHIN = 1,
	TSHARK = 2,
	SHARKL = 3,
	PIKE = 4,
	PIKEL = 5,
	SHARKL64 = 6,
	SHARKLT8 = 7,
	WHALE = 8,
	WHALE2 = 9,
	IWHALE2 = 10,
	MAX_VERSIONS,
} CHIP_VERSION_E;

typedef struct vpp_drv_buffer_t {
	unsigned int size;
	unsigned int phys_addr;
	unsigned long base; /*kernel logical address in use kernel */
	unsigned long virt_addr; /* virtual user space address */
} VPP_DRV_BUFFER_T;

enum sprd_vpp_frequency_e {
	VPP_FREQENCY_LEVEL_0 = 0,
	VPP_FREQENCY_LEVEL_1 = 1,
	VPP_FREQENCY_LEVEL_2 = 2,
	VPP_FREQENCY_LEVEL_3 = 3
};

#endif
