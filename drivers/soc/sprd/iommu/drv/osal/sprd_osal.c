#include "sprd_osal.h"

void *sprd_malloc(u32 size)
{
	return kmalloc(size, GFP_KERNEL);
}

void sprd_free(void *addr)
{
	kfree(addr);
}

void sprd_memset(void *addr , u32 value, u32 size)
{
	memset((void *)addr, value, size);
}

#ifdef CONFIG_X86
extern void *memset_orig(void *, int, __kernel_size_t);
#endif

void sprd_memset_orig(void *addr, u32 value, u32 size)
{
#ifdef CONFIG_X86
	memset_orig((void *)addr, value, size);
#else
	memset((void *)addr, value, size);
#endif
}

void sprd_memcpy(void *dstaddr , void *srcaddr, u32 size)
{
	memcpy((void *)dstaddr, (void *)srcaddr, size);
}

void sprd_sleep(u32 mstime)
{
}

void *sprd_aligned_malloc(u32 size  , u32 align)
{
	return kmalloc(size, GFP_KERNEL);
}


void sprd_aligned_free(void *addr)
{

}
