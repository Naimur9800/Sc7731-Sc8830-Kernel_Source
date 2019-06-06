#ifndef _SPRD_COM_H_
#define _SPRD_COM_H_

#include "../inc/sprd_defs.h"

#define FULL_MASK 0xFFFFFFFF

#define reg_write_byte(addr, value)  __raw_writeb(value, (void __iomem *)addr)
#define reg_write_word(addr, value)  __raw_writew(value, (void __iomem *)addr)
#define reg_write_dword(addr, value)  __raw_writel(value, (void __iomem *)addr)

#define reg_read_byte(addr)  __raw_readb((void __iomem *)addr)
#define reg_read_word(addr)   __raw_readw((void __iomem *)addr)
#define reg_read_dword(addr)   __raw_readl((void __iomem *)addr)


/*page is 4k alignment, left shift 12 bit*/
#define MMU_MAPING_PAGESIZE_SHIFFT   12

/*virt address must be 1M byte alignment, left shift 20 bit*/
#define MMU_MAPING_VIRT_ADDR_SHIFFT   20

/*page size is 4K */
#define MMU_MAPING_PAGESIZE   (1<<MMU_MAPING_PAGESIZE_SHIFFT)
#define MMU_MAPING_PAGE_MASK  (MMU_MAPING_PAGESIZE - 1)

#define MMU_VIRT_ADDR_ALIGN  (1<<MMU_MAPING_VIRT_ADDR_SHIFFT)
#define MMU_VIRT_ADDR_MASK  (MMU_VIRT_ADDR_ALIGN - 1)
#define ADDR_TO_PAGE(addr)  (addr >> MMU_MAPING_PAGESIZE_SHIFFT)

/*suppose each channel valid use 128M, actually support 1G above*/
#define MMU_CH_MAX_MAP_SIZE (100*1024*1024)
#define MMU_EX_128M_SIZE (128*1024*1024)
#define MMU_EX_256M_SIZE (256*1024*1024)
#define MMU_EX_64M_SIZE (64*1024*1024)

#define VIR_TO_ENTRY_IDX(virt_addr, base_addr) \
	((virt_addr-base_addr)/MMU_MAPING_PAGESIZE)
#define MAP_SIZE_PAGE_ALIGN_UP(length) \
	((length + MMU_MAPING_PAGE_MASK) & (~MMU_MAPING_PAGE_MASK))
#define SIZE_TO_ENTRYS(size) (size/MMU_MAPING_PAGESIZE)


/*-----------------------------------------------------------------------*/
/*                          FUNCTIONS HEADERS                            */
/*-----------------------------------------------------------------------*/

void putbit(ulong reg_addr, u32 dst_value , u8 pos);
void putbits(ulong reg_addr, u32 dst_value , u8 highbitoffset, u8 lowbitoffset);
ulong sg_to_phys(struct scatterlist *sg);


#endif  /*END OF : define  _SPRD_COM_H_ */
