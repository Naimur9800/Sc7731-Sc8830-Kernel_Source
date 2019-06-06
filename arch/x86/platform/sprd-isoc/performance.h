/*
 * Copyright (C) 2016, Intel Corporation
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */

#ifndef _SPRD_PERFORMANCE_H
#define _SPRD_PERFORMANCE_H

#define PCI_REG_ADDR	0xcf8
#define PCI_REG_DATA	0xcfc

#define MSG_DEV		0x0 /* B=0/D=0/F=0 */
#define PCI_CFG_EN	0x80000000

#define MCR_RD		0x6
#define MCR_WR		0x7

#define MCR		0xd0
#define MDR		0xd4
#define MCRX		0xd8

#define BDF(x)		(x << 8)

#define ENA_BYTE(x)	(x << 4)
#define OFFSET(x)	((x & 0xff) << 8)
#define OFFSETX(x)	(x & ~0xff)
#define PORT(x)		(x << 16)
#define OPCODE(x)	(x << 24)

#define RORW_HOST_IOSF(p, o, m, v)	\
	({SET_HOST_IOSF(p, o, ((GET_HOST_IOSF(p, o) & (~m)) | ((v) & (m)))); })

#define RANDW_HOST_IOSF(p, o, v)	\
	({SET_HOST_IOSF(p, o, (GET_HOST_IOSF(p, o) & (v))); })

#define GET_HOST_IOSF(p, o)	\
	({outl(PCI_CFG_EN | BDF(MSG_DEV) | MCRX, PCI_REG_ADDR);		  \
	  outl(OFFSETX(o), PCI_REG_DATA);				  \
	  outl(PCI_CFG_EN | BDF(MSG_DEV) | MCR, PCI_REG_ADDR);		  \
	  outl(OPCODE(MCR_RD) | PORT(p) | OFFSET(o) | 0xf0, PCI_REG_DATA);\
	  outl(PCI_CFG_EN | BDF(MSG_DEV) | MDR, PCI_REG_ADDR);		  \
	  inl(PCI_REG_DATA); })

#define SET_HOST_IOSF(p, o, v)	\
	({outl(PCI_CFG_EN | BDF(MSG_DEV) | MDR, PCI_REG_ADDR);	\
	  outl(v, PCI_REG_DATA);				\
	  outl(PCI_CFG_EN | BDF(MSG_DEV) | MCRX, PCI_REG_ADDR);	\
	  outl(OFFSETX(o), PCI_REG_DATA);			\
	  outl(PCI_CFG_EN | BDF(MSG_DEV) | MCR, PCI_REG_ADDR);	\
	  outl(OPCODE(MCR_WR) | PORT(p) | OFFSET(o) | 0xf0, PCI_REG_DATA); })

#endif /*_SPRD_PERFORMANCE_H*/
