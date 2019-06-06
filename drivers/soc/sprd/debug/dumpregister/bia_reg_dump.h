/************************************************************************
 ** File Name: bia_reg_dump.h                                           *
 ** Author:    Yanan.Luo                                                *
 ** DATE:      11/01/2015                                               *
 ** Copyright: 2003 Spreatrum, Incoporated. All Rights Reserved.        *
 ** Description:define trace interface just for testing usage           *
 ************************************************************************

 ************************************************************************
 **                        Edit History                                 *
 ** ------------------------------------------------------------------- *
 ** DATE           NAME            DESCRIPTION                          *
 ** 11/01/2015     Yanan.Luo       Create                               *
 ** 12/01/2016     rui.chen        up to standard of kernel             *
 ************************************************************************/
#ifndef _PERFMON_DRV_H_
#define _PERFMON_DRV_H_

#include <asm/processor.h>
#include <asm/msr.h>

#define UINT8_TYPE      unsigned char
#define UINT16_TYPE     unsigned short
#define UINT32_TYPE     unsigned int
#define UINT64_TYPE     unsigned long long

#define OPCODE_10	(0x10 >> 1)
#define OPCODE_6	(0x6 >> 1)

/* port id */
#define AUNIT_PORT	0x0
#define TUNIT_PORT	0x2
#define BUNIT_PORT	0x3
#define CUNIT_PORT	0x8
#define IOSF2OCP_PORT	0x14
#define PMI2AXI_PORT	0x1B

/* reg offset */
#define ACKGATE		0x18
#define AISOCHCTL	0x20
#define AVCCTL		0x21
#define AARBCTL0	0xC0

#define BARBCTRL0	0x3
#define BARBCTRL1	0x4
#define BARBCTRL2	0x5
#define BARBCTRL3	0x6
#define BWFLUSH		0x7
#define BBANKMASK	0x8
#define BROWMASK	0x9
#define BRANKMASK	0xA
#define BALIMIT0	0xB
#define BALIMIT1	0xC
#define BALIMIT2	0xD
#define BALIMIT3	0xE
#define BCOSCAT		0x12
#define BMISC		0x28
#define BC0AHASHCFG	0x30
#define BC1AHASHCFG	0x31
#define BDEBUG0		0x3B
#define BCTRL		0x3D
#define BTHCTRL		0x3E
#define BTHMASK		0x3F

#define CUNIT_SSA_REG	0x43

#define T_CTL		0x3
#define T_MISC_CTL	0x4
#define T_CLKGATE_CTL	0x5

#define BMODE0		0x0
#define BDBUG0		0x1

#define OCP_CTL		0x10


struct CPUID_regs_T {
	UINT32_TYPE	eax;
	UINT32_TYPE	ebx;
	UINT32_TYPE	ecx;
	UINT32_TYPE	edx;
};

#define CPUID_regs        struct CPUID_regs_T

void perf_get_cpuid(UINT32_TYPE op, CPUID_regs *id, UINT32_TYPE count)
{
	cpuid_count(op, count, &id->eax, &id->ebx, &id->ecx, &id->edx);
}

#define perf_read_msr(_ecx) native_read_msr(_ecx)

#endif
