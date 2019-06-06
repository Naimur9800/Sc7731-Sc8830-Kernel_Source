#ifndef __SDIOM_TYPE_H__
#define __SDIOM_TYPE_H__

#ifndef NULL
#define NULL (0)
#endif

#ifndef OK
#define OK                                   (0)
#endif

#ifndef ERROR
#define ERROR                                (-1)
#endif

#define SDIOM_ALIGN_32BIT(a)  ((((a)+3)&(~3)))
#define SDIOM_ALIGN_512BYTE(a)  ((((a)+511)&(~511)))

typedef unsigned int (*SDIOM_PT_TX_RELEASE_CALLBACK) (void *addr);
typedef unsigned int (*SDIOM_PT_RX_PROCESS_CALLBACK) (void *addr,
						      unsigned int len,
						      unsigned int fifo_id);

typedef int (*SDIOM_RESCAN_CALLBACK) (void);

#endif /* __SDIOM_TYPE_H__ */
