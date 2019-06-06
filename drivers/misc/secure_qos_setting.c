#include <asm/compiler.h>
#include <linux/module.h>
#include <linux/of.h>
#include <misc/secure_qos_setting.h>

#ifdef CONFIG_ARM64
#define SMC_ARG0		"x0"
#define SMC_ARG1		"x1"
#define SMC_ARG2		"x2"
#define SMC_ARG3		"x3"
#define SMC_ARCH_EXTENSION	""
#define SMC_REGISTERS_TRASHED "x4", "x5", "x6", "x7", "x8", "x9", "x10", \
				"x11", "x12", "x13", "x14", "x15", "x16", "x17"
#else
#define SMC_ARG0		"r0"
#define SMC_ARG1		"r1"
#define SMC_ARG2		"r2"
#define SMC_ARG3		"r3"
#define SMC_ARCH_EXTENSION	".arch_extension sec\n"
#define SMC_REGISTERS_TRASHED	"ip"
#endif

static ulong smc(ulong r0, ulong r1, ulong r2, ulong r3)
{
	register ulong _r0 asm(SMC_ARG0) = r0;
	register ulong _r1 asm(SMC_ARG1) = r1;
	register ulong _r2 asm(SMC_ARG2) = r2;
	register ulong _r3 asm(SMC_ARG3) = r3;
	asm volatile(
		__asmeq("%0", SMC_ARG0)
		__asmeq("%1", SMC_ARG1)
		__asmeq("%2", SMC_ARG2)
		__asmeq("%3", SMC_ARG3)
		__asmeq("%4", SMC_ARG0)
		__asmeq("%5", SMC_ARG1)
		__asmeq("%6", SMC_ARG2)
		__asmeq("%7", SMC_ARG3)
		SMC_ARCH_EXTENSION
		"smc    #0"  /* switch to secure world */
		: "=r" (_r0), "=r" (_r1), "=r" (_r2), "=r" (_r3)
		: "r" (_r0), "r" (_r1), "r" (_r2), "r" (_r3)
		: SMC_REGISTERS_TRASHED);
		return _r0;
}

ulong qos_register_read(ulong phyaddr)
{
	return  smc(SECURE_FN_REG_READ, phyaddr, 0, 0);
}
EXPORT_SYMBOL(qos_register_read);

/* 0 for success, otherwise fail */
ulong qos_register_write(ulong phyaddr, ulong val)
{
	return smc(SECURE_FN_REG_WRITE, phyaddr, val, 0);
}
EXPORT_SYMBOL(qos_register_write);
