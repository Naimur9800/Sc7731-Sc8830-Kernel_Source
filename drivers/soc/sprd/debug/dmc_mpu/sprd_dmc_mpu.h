/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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

#ifndef __SPRD_DMC_MPU_H__
#define __SPRD_DMC_MPU_H__

#ifdef CONFIG_X86
extern void ap_light_enable_disable(unsigned int req, const char *name);

static inline void close_smart_lightsleep(void)
{
	ap_light_enable_disable(0, "dmc_mpu");
}

static inline void open_smart_lightsleep(void)
{
	ap_light_enable_disable(1, "dmc_mpu");
}
#else
static inline void close_smart_lightsleep(void)
{
}

static inline void open_smart_lightsleep(void)
{
}
#endif
enum chip_id {
	WHILE2,
	ISHARKL2,
	SHARKL2,
	SHARKLJ1,
};
struct chip_id_data {
	enum chip_id id;
};

static struct chip_id_data whale2 = {
	WHILE2
};

static struct chip_id_data isharkl2 = {
	ISHARKL2
};

static struct chip_id_data sharkl2 = {
	SHARKL2
};

static struct chip_id_data sharklj1 = {
	SHARKLJ1
};

struct reg_addr_offset {
	enum chip_id chip;
	u32 mpu_int;
	u32 mpu_int_set;
	u32 mpu_int_clr;
	u32 mpu_en;
	u32 mpu_reset;
	u32 mpu_addr_range;
	u32 mpu_addr_dump;
	u32 mpu_sel;
	u32 mpu_vio_waddr;
	u32 mpu_vio_raddr;
	u32 mpu_vio_wcmd;
	u32 mpu_vio_rcmd;
	u32 mpu_addr_mask;
	u32 mpu_ddr_addr_mask;
};

static struct reg_addr_offset reg_offset[] = {
	{WHILE2, 0x0, BIT(20), BIT(21), 0x4, 0xc, 0x3c,
		0x68, 0xA0, 0x100, 0x104, 0x180, 0x184,
		0x80000000, 0x80008000},
	{ISHARKL2, 0x0, BIT(20), BIT(21), 0x4, 0xc, 0x28,
		0x40, 0x58, 0x5c, 0x60, 0x8c, 0x90,
		0, 0},
	{SHARKL2, 0x31d8, BIT(0), BIT(1), 0x4, 0x3004, 0x3030,
		0x305c, 0x31B4, 0x3200, 0x3204, 0x3240, 0x3244,
		0x0, 0x0},
	{SHARKLJ1, 0x31d8, BIT(0), BIT(1), 0x4, 0x3004, 0x3030,
		0x305c, 0x31B4, 0x3200, 0x3204, 0x3240, 0x3244,
		0x80000000, 0x80008000},
};

static struct reg_addr_offset *this_offset;
#define MPU_INT			(this_offset->mpu_int)
#define MPU_EN			(this_offset->mpu_en)
#define MPU_RESET		(this_offset->mpu_reset)
#define MPU_RANGE(n)		((this_offset->mpu_addr_range) +\
((this_offset->chip == SHARKL2) ? (((n > 6) ? (n + 1) : n) * 4) : (n * 4)))
#define ADDR_DUMP(n)		((this_offset->mpu_addr_dump) + (n * 4))
#define MPU_SEL			(this_offset->mpu_sel)
#define MPU_VIO_WADDR(n)		((this_offset->mpu_vio_waddr) + (n * 8))
#define MPU_VIO_RADDR(n)		((this_offset->mpu_vio_raddr) + (n * 8))
#define MPU_VIO_WCMD(n)		((this_offset->mpu_vio_wcmd)  + (n * 8))
#define MPU_VIO_RCMD(n)		((this_offset->mpu_vio_rcmd)  + (n * 8))

#define BIT_MPU_EB		BIT(29)
#define BIT_MPU_INT		(this_offset->mpu_int_set)
#define BIT_MPU_INT_CLR		(this_offset->mpu_int_clr)
#define BIT_MPU_RST(n)		BIT(n)

#define MPU_ADDR_MASK		(this_offset->mpu_addr_mask)
#define MPU_DDR_ADDR_MASK	(this_offset->mpu_ddr_addr_mask)
#define MPU_ADDR_ALIGN		0xFFFF0000

enum sprd_mpu_mode {
	W_MODE,
	R_MODE,
	RW_MODE,
	NONE_MODE,
};

struct sprd_mpu_chn {
	int pub;
	int chn;
	enum sprd_mpu_mode flag;
	u32 range_addr;
	bool panic;
	bool incld_mod;
	bool sticky_enable;
};

struct sprd_mpu_dbg_info {
	struct sprd_mpu_chn chn;
	u32 dump_addr;
	u32 vio_waddr;
	u32 vio_raddr;
	u32 vio_wcmd;
	u32 vio_rcmd;
	u32 dump_data[16];
};

struct sprd_dmc_mpu_dev {
	struct miscdevice			misc;
	void __iomem				*mpu_pub0_base;
	void __iomem				*mpu_pub1_base;
	int							pub0_irq;
	int							pub1_irq;
	int							pub_chn;
	int							interleaved_mod;
	unsigned long				pub0_dump_addr_p;
	unsigned long				pub1_dump_addr_p;
	void __iomem				*pub0_dump_addr_v;
	void __iomem				*pub1_dump_addr_v;
	struct sprd_mpu_dbg_info	dbg_info;
	struct sprd_mpu_chn			chns[0];
};
#endif
