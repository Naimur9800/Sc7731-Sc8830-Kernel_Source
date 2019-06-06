/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
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
#ifndef _CPP_REG_H_
#define _CPP_REG_H_

#define MM_AHB_RESET                  (0x0004UL)
#define CPP_AHB_RESET_BIT              (1 << 9)

#define CPP_BASE                       0x00
#define CPP_PATH_EB                    0x00
#define CPP_ROT_EB_BIT                 BIT(1)
#define CPP_SCALE_PATH_EB_BIT          BIT(0)

#define CPP_PATH_START                 0x04
#define CPP_ROT_START_BIT              BIT(1)
#define CPP_SCALE_START_BIT            BIT(0)

#define CPP_PATH_STS                   0x08
#define CPP_INT_STS                    0x0c
#define CPP_INT_MASK                   0x10
#define ROT_PATH_INT_BIT               BIT(0)
#define SC_PATH_INT_BIT                BIT(1)
#define MMU_VAOR_RD_INT_BIT            BIT(2)
#define MMU_VAOR_WR_INT_BIT            BIT(3)
#define MMU_INV_RD_INT_BIT             BIT(4)
#define MMU_INV_WR_INT_BIT             BIT(5)
#define MMU_UNS_RD_INT_BIT             BIT(6)
#define MMU_UNS_WR_INT_BIT             BIT(7)
#define MMU_PAOR_RD_INT_BIT            BIT(8)
#define MMU_PAOR_WR_INT_BIT            BIT(9)
#define CPP_PATH_DONE \
		(ROT_PATH_INT_BIT | SC_PATH_INT_BIT)
#define CPP_MMU_ERROR_INT \
	((MMU_VAOR_RD_INT_BIT | MMU_VAOR_WR_INT_BIT) | \
	(MMU_INV_RD_INT_BIT | MMU_INV_WR_INT_BIT) | \
	(MMU_UNS_RD_INT_BIT | MMU_UNS_RD_INT_BIT) | \
	(MMU_PAOR_RD_INT_BIT | MMU_PAOR_WR_INT_BIT))

#define CPP_INT_CLR                    0x14
#define CPP_INT_RAW                    0x18
#define CPP_ROT_IRQ_BIT                BIT(1)
#define CPP_SCALE_IRQ_BIT              BIT(0)

#define CPP_AXIM_CHN_SET               0x1c
#define CPP_ROT_AXI_WR_ENDIAN_MASK     (7 << 8)
#define CPP_SCALE_OUTPUT_UV_ENDIAN     (1 << 7)
#define CPP_SCALE_OUTPUT_Y_ENDIAN      (7 << 4)
#define CPP_SCALE_INPUT_UV_ENDIAN      (1 << 3)
#define CPP_SCALE_INPUT_Y_ENDIAN       (7 << 0)

#define CPP_PATH0_SRC_ADDR_Y           0x40
#define CPP_PATH0_SRC_ADDR_UV          0x44

#define CPP_PATH0_DES_ADDR_Y           0x48
#define CPP_PATH0_DES_ADDR_UV          0x4c


#define CPP_PATH0_CFG0                 0x50
#define CPP_SCALE_CLK_SWITCH           (1 << 11)

#define CPP_SCALE_DEC_V_MASK           (3 << 8)
#define CPP_SCALE_DEC_H_MASK           (3 << 6)
#define CPP_SCALE_OUTPUT_FORMAT        (3 << 4)
#define CPP_SCALE_INPUT_FORMAT         (3 << 2)

#define CPP_PATH0_CFG1                 0x54
#define CPP_SCALE_SRC_HEIGHT_MASK      (0x1fff << 16)
#define CPP_SCALE_SRC_WIDTH_MASK       0x1fff

#define CPP_PATH0_CFG2                 0x58
#define CPP_SCALE_DES_HEIGHT_MASK      (0x1fff << 16)
#define CPP_SCALE_DES_WIDTH_MASK       (0x1fff)

#define CPP_PATH0_CFG3                 0x5c
#define CPP_SCALE_DES_PITCH_MASK       (0x1fff << 16)
#define CPP_SCALE_SRC_PITCH_MASK       0x1fff

#define CPP_PATH0_CFG4                 0x60
#define CPP_SCALE_SRC_OFFSET_X_MASK    (0x1fff << 16)
#define CPP_SCALE_SRC_OFFSET_Y_MASK    0x1fff

#define CPP_PATH0_CFG5                 0x64
#define CPP_SCALE_DES_OFFSET_X_MASK    (0x1fff << 16)
#define CPP_SCALE_DES_OFFSET_Y_MASK    0x1fff

#define CPP_SC_TAP                     0x68
#define CPP_SCALE_Y_VER_TAP            (0xf << 5)
#define CPP_SCALE_UV_VER_TAP           (0x1f << 0)

#define CPP_PATH0_SRC_YUV_REGULATE_0   0x80
#define CPP_PATH0_SRC_YUV_REGULATE_1   0x84
#define CPP_PATH0_SRC_YUV_REGULATE_2   0x88

#define CPP_PATH0_DES_YUV_REGULATE_0   0x8c
#define CPP_PATH0_DES_YUV_REGULATE_1   0x90
#define CPP_PATH0_DES_YUV_REGULATE_2   0x94


#define CPP_ROTATION_SRC_ADDR          0x100
#define CPP_ROTATION_DES_ADDR          0x104
#define CPP_ROTATION_IMG_SIZE          0x108
#define CPP_ROTATION_SRC_PITCH         0x10c
#define CPP_ROTATION_OFFSET_START      0x110
#define CPP_ROTATION_PATH_CFG          0x114
#define CPP_ROT_UV_MODE_BIT            (1 << 4)
#define CPP_ROT_MODE_MASK              (3 << 2)
#define CPP_ROT_PIXEL_FORMAT_BIT       (3 << 0)

#define CPP_END                        0x124

#endif
