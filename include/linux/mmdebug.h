#ifndef LINUX_MM_DEBUG_H
#define LINUX_MM_DEBUG_H 1

#ifdef CONFIG_DEBUG_VM
#define VM_BUG_ON(cond) BUG_ON(cond)
#define VM_BUG_ON_PAGE(cond, page)					\
	do {								\
		if (unlikely(cond)) {					\
			dump_page(page, "VM_BUG_ON_PAGE(" __stringify(cond)")");\
			BUG();						\
		}							\
	} while (0)
#define VM_BUG_ON_VMA(cond, vma)					\
	do {								\
		if (unlikely(cond)) {					\
			dump_vma(vma);					\
			BUG();						\
		}							\
	} while (0)
#define VM_BUG_ON_MM(cond, mm)						\
	do {								\
		if (unlikely(cond)) {					\
			dump_mm(mm);					\
			BUG();						\
		}							\
	} while (0)
#define VM_WARN_ON(cond) WARN_ON(cond)
#define VM_WARN_ON_ONCE(cond) WARN_ON_ONCE(cond)
#define VM_WARN_ONCE(cond, format...) WARN_ONCE(cond, format)
#define VM_WARN(cond, format...) WARN(cond, format)
#else
#define VM_BUG_ON(cond) BUILD_BUG_ON_INVALID(cond)
#define VM_BUG_ON_PAGE(cond, page) VM_BUG_ON(cond)
#define VM_BUG_ON_VMA(cond, vma) VM_BUG_ON(cond)
#define VM_BUG_ON_MM(cond, mm) VM_BUG_ON(cond)
#define VM_WARN_ON(cond) BUILD_BUG_ON_INVALID(cond)
#define VM_WARN_ON_ONCE(cond) BUILD_BUG_ON_INVALID(cond)
#define VM_WARN_ONCE(cond, format...) BUILD_BUG_ON_INVALID(cond)
#define VM_WARN(cond, format...) BUILD_BUG_ON_INVALID(cond)
#endif

#ifdef CONFIG_DEBUG_VIRTUAL
#define VIRTUAL_BUG_ON(cond) BUG_ON(cond)
#else
#define VIRTUAL_BUG_ON(cond) do { } while (0)
#endif

#endif
