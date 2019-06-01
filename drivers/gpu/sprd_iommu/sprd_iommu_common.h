#ifndef __SPRD_IOMMU_COMMON_H__
#define __SPRD_IOMMU_COMMON_H__

#include <linux/module.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/ion.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/genalloc.h>
#include <linux/sprd_iommu.h>
#include <linux/scatterlist.h>
#if 0
#ifdef CONFIG_ARCH_SCX35L64
#include <soc/sprd/sci_glb_regs.h>
#else
#include <soc/sprd/hardware.h>
#include <soc/sprd/sci.h>
#include <soc/sprd/sci_glb_regs.h>
#endif
#endif
#include "../ion/ion_priv.h"
#include <linux/of.h>

#define SPRD_IOMMU_PAGE_SIZE	PAGE_SIZE//0x1000  //need check
/**
 * Page table index from address
 * Calculates the page table index from the given address
*/
#define SPRD_IOMMU_PTE_ENTRY(address) (((address)>>12) & 0x7FFF)

/*64-bit virtual addr and 32bit phy addr in arm 64-bit of TSharkL.
  *So we use (sizeof(unsigned int)).
  */
#define SPRD_IOMMU_IOVA_SIZE_TO_PGT(size)  ((sizeof(unsigned int))*(size)/SPRD_IOMMU_PAGE_SIZE)

extern const struct of_device_id iommu_ids[];

#if 0
extern u32 emc_clk_get(void);
#endif

void mmu_reg_write(unsigned long reg, unsigned long val, unsigned long msk);

int sprd_iommu_init(struct sprd_iommu_dev *dev, struct sprd_iommu_init_data *data);

int sprd_iommu_exit(struct sprd_iommu_dev *dev);

unsigned long sprd_iommu_iova_alloc(struct sprd_iommu_dev *dev, size_t iova_length);

void sprd_iommu_iova_free(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length);

int sprd_iommu_iova_map(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length, struct ion_buffer *handle);

int sprd_iommu_iova_unmap(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length, struct ion_buffer *handle);

int sprd_iommu_backup(struct sprd_iommu_dev *dev);

int sprd_iommu_restore(struct sprd_iommu_dev *dev);

void sprd_iommu_reset(struct sprd_iommu_dev *dev);

int sprd_iommu_disable(struct sprd_iommu_dev *dev);
int sprd_iommu_enable(struct sprd_iommu_dev *dev);
int sprd_iommu_dump(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length);

#endif
