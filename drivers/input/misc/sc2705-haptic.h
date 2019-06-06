/*
 * registers.h - Haptics REGISTERS H for SC2705
 * Copyright (c) 2017 Dialog Semiconductor.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SC2705_HAPTIC_REGISTERS_H
#define __SC2705_HAPTIC_REGISTERS_H

#include <linux/bitops.h>

/* Registers */

#define SC2705_CHIP_ID			0x01
#define SC2705_IRQ_EVENT1		0x03
#define SC2705_IRQ_EVENT_WARNING_DIAG	0x04
#define SC2705_IRQ_EVENT_PAT_DIAG	0x05
#define SC2705_IRQ_STATUS1		0x06
#define SC2705_IRQ_MASK1		0x07
#define SC2705_CIF_I2C1			0x08
#define SC2705_CIF_I2C2			0x09
#define SC2705_FRQ_LRA_PER_H		0x0A
#define SC2705_FRQ_LRA_PER_L		0x0B
#define SC2705_ACTUATOR1		0x0C
#define SC2705_ACTUATOR2		0x0D
#define SC2705_ACTUATOR3		0x0E
#define SC2705_CALIB_V2I_H		0x0F
#define SC2705_CALIB_V2I_L		0x10
#define SC2705_TOP_CFG1			0x13
#define SC2705_TOP_CFG2			0x14
#define SC2705_TOP_CFG3			0x15
#define SC2705_TOP_CFG4			0x16
#define SC2705_TOP_CTL1			0x21
#define SC2705_TOP_CTL2			0x22
#define SC2705_SEQ_CTL1			0x23
#define SC2705_SEQ_CTL2			0x24
#define SC2705_MEM_CTL1			0x25
#define SC2705_MEM_CTL2			0x26
#define SC2705_ADC_DATA_H1		0x27
#define SC2705_ADC_DATA_L1		0x28
#define SC2705_H_TRIM1			0x53
#define SC2705_H_TRIM2			0x54
#define SC2705_IRQ_STATUS2		0x76
#define SC2705_IRQ_MASK2		0x77
#define SC2705_SNP_MEM_0		0x78
#define SC2705_SNP_MEM_1		0x79
#define SC2705_SNP_MEM_2		0x7A
#define SC2705_SNP_MEM_3		0x7B
#define SC2705_SNP_MEM_4		0x7C
#define SC2705_SNP_MEM_5		0x7D
#define SC2705_SNP_MEM_6		0x7E
#define SC2705_SNP_MEM_7		0x7F
#define SC2705_SNP_MEM_8		0x80
#define SC2705_SNP_MEM_9		0x81
#define SC2705_SNP_MEM_10		0x82
#define SC2705_SNP_MEM_11		0x83
#define SC2705_SNP_MEM_12		0x84
#define SC2705_SNP_MEM_13		0x85
#define SC2705_SNP_MEM_14		0x86
#define SC2705_SNP_MEM_15		0x87
#define SC2705_SNP_MEM_16		0x88
#define SC2705_SNP_MEM_17		0x89
#define SC2705_SNP_MEM_18		0x8A
#define SC2705_SNP_MEM_19		0x8B
#define SC2705_SNP_MEM_20		0x8C
#define SC2705_SNP_MEM_21		0x8D
#define SC2705_SNP_MEM_22		0x8E
#define SC2705_SNP_MEM_23		0x8F
#define SC2705_SNP_MEM_24		0x90
#define SC2705_SNP_MEM_25		0x91
#define SC2705_SNP_MEM_26		0x92
#define SC2705_SNP_MEM_27		0x93
#define SC2705_SNP_MEM_28		0x94
#define SC2705_SNP_MEM_29		0x95
#define SC2705_SNP_MEM_30		0x96
#define SC2705_SNP_MEM_31		0x97
#define SC2705_SNP_MEM_32		0x98
#define SC2705_SNP_MEM_33		0x99
#define SC2705_SNP_MEM_34		0x9A
#define SC2705_SNP_MEM_35		0x9B
#define SC2705_SNP_MEM_36		0x9C
#define SC2705_SNP_MEM_37		0x9D
#define SC2705_SNP_MEM_38		0x9E
#define SC2705_SNP_MEM_39		0x9F
#define SC2705_SNP_MEM_40		0xA0
#define SC2705_SNP_MEM_41		0xA1
#define SC2705_SNP_MEM_42		0xA2
#define SC2705_SNP_MEM_43		0xA3
#define SC2705_SNP_MEM_44		0xA4
#define SC2705_SNP_MEM_45		0xA5
#define SC2705_SNP_MEM_46		0xA6
#define SC2705_SNP_MEM_47		0xA7
#define SC2705_SNP_MEM_48		0xA8
#define SC2705_SNP_MEM_49		0xA9
#define SC2705_SNP_MEM_50		0xAA
#define SC2705_SNP_MEM_51		0xAB
#define SC2705_SNP_MEM_52		0xAC
#define SC2705_SNP_MEM_53		0xAD
#define SC2705_SNP_MEM_54		0xAE
#define SC2705_SNP_MEM_55		0xAF
#define SC2705_SNP_MEM_56		0xB0
#define SC2705_SNP_MEM_57		0xB1
#define SC2705_SNP_MEM_58		0xB2
#define SC2705_SNP_MEM_59		0xB3
#define SC2705_SNP_MEM_60		0xB4
#define SC2705_SNP_MEM_61		0xB5
#define SC2705_SNP_MEM_62		0xB6
#define SC2705_SNP_MEM_63		0xB7
#define SC2705_SNP_MEM_64		0xB8
#define SC2705_SNP_MEM_65		0xB9
#define SC2705_SNP_MEM_66		0xBA
#define SC2705_SNP_MEM_67		0xBB
#define SC2705_SNP_MEM_68		0xBC
#define SC2705_SNP_MEM_69		0xBD
#define SC2705_SNP_MEM_70		0xBE
#define SC2705_SNP_MEM_71		0xBF
#define SC2705_SNP_MEM_72		0xC0
#define SC2705_SNP_MEM_73		0xC1
#define SC2705_SNP_MEM_74		0xC2
#define SC2705_SNP_MEM_75		0xC3
#define SC2705_SNP_MEM_76		0xC4
#define SC2705_SNP_MEM_77		0xC5
#define SC2705_SNP_MEM_78		0xC6
#define SC2705_SNP_MEM_79		0xC7
#define SC2705_SNP_MEM_80		0xC8
#define SC2705_SNP_MEM_81		0xC9
#define SC2705_SNP_MEM_82		0xCA
#define SC2705_SNP_MEM_83		0xCB
#define SC2705_SNP_MEM_84		0xCC
#define SC2705_SNP_MEM_85		0xCD
#define SC2705_SNP_MEM_86		0xCE
#define SC2705_SNP_MEM_87		0xCF
#define SC2705_SNP_MEM_88		0xD0
#define SC2705_SNP_MEM_89		0xD1

/* SC2705_CHIP_REV = 0x00 */
#define SC2705_CHIP_REV_MINOR_SHIFT           4
#define SC2705_CHIP_REV_MINOR_MASK            (0xF << 4)
#define SC2705_CHIP_REV_MAJOR_SHIFT           0
#define SC2705_CHIP_REV_MAJOR_MASK            (0xF << 0)

/* SC2705_CHIP_ID = 0x01 */
#define SC2705_CHIP_ID_SHIFT                  0
#define SC2705_CHIP_ID_MASK                   (0xFF << 0)

/* SC2705_IRQ_EVENT1 = 0x03 */
#define SC2705_E_OC_FAULT_SHIFT               7
#define SC2705_E_OC_FAULT_MASK                BIT(7)
#define SC2705_E_ACTUATOR_FAULT_SHIFT         6
#define SC2705_E_ACTUATOR_FAULT_MASK          BIT(6)
#define SC2705_E_WARNING_SHIFT                5
#define SC2705_E_WARNING_MASK                 BIT(5)
#define SC2705_E_PAT_FAULT_SHIFT              4
#define SC2705_E_PAT_FAULT_MASK               BIT(4)
#define SC2705_E_CALIB_DONE_SHIFT             3
#define SC2705_E_CALIB_DONE_MASK              BIT(3)
#define SC2705_E_PAT_DONE_SHIFT               2
#define SC2705_E_PAT_DONE_MASK                BIT(2)
#define SC2705_E_SOFT_SHUTDOWN_SHIFT          1
#define SC2705_E_SOFT_SHUTDOWN_MASK           BIT(1)
#define SC2705_E_SEQ_CONTINUE_SHIFT           0
#define SC2705_E_SEQ_CONTINUE_MASK            BIT(0)

/* SC2705_IRQ_EVENT_WARNING_DIAG = 0x04 */
#define SC2705_E_LIM_DRIVE_SHIFT              7
#define SC2705_E_LIM_DRIVE_MASK               BIT(7)
#define SC2705_E_LIM_DRIVE_ACC_SHIFT          6
#define SC2705_E_LIM_DRIVE_ACC_MASK           BIT(6)
#define SC2705_E_LIM_CALIB_SHIFT              5
#define SC2705_E_LIM_CALIB_MASK               BIT(5)
#define SC2705_E_MEM_TYPE_SHIFT               4
#define SC2705_E_MEM_TYPE_MASK                BIT(4)

/* SC2705_IRQ_EVENT_PAT_DIAG = 0x05 */
#define SC2705_E_SEQ_ID_FAULT_SHIFT           7
#define SC2705_E_SEQ_ID_FAULT_MASK            BIT(7)
#define SC2705_E_MEM_FAULT_SHIFT              6
#define SC2705_E_MEM_FAULT_MASK               BIT(6)
#define SC2705_E_PWM_FAULT_SHIFT              5
#define SC2705_E_PWM_FAULT_MASK               BIT(5)

/* SC2705_IRQ_STATUS1 = 0x06 */
#define SC2705_STA_OC_SHIFT                   7
#define SC2705_STA_OC_MASK                    BIT(7)
#define SC2705_STA_ACTUATOR_SHIFT             6
#define SC2705_STA_ACTUATOR_MASK              BIT(6)
#define SC2705_STA_WARNING_SHIFT              5
#define SC2705_STA_WARNING_MASK               BIT(5)
#define SC2705_STA_PAT_FAULT_SHIFT            4
#define SC2705_STA_PAT_FAULT_MASK             BIT(4)
#define SC2705_STA_CALIB_DONE_SHIFT           3
#define SC2705_STA_CALIB_DONE_MASK            BIT(3)
#define SC2705_STA_PAT_DONE_SHIFT             2
#define SC2705_STA_PAT_DONE_MASK              BIT(2)
#define SC2705_STA_SOFT_SHUTDOWN_SHIFT        1
#define SC2705_STA_SOFT_SHUTDOWN_MASK         BIT(1)
#define SC2705_STA_SEQ_CONTINUE_SHIFT         0
#define SC2705_STA_SEQ_CONTINUE_MASK          BIT(0)

/* SC2705_IRQ_MASK1 = 0x07 */
#define SC2705_OC_M_SHIFT                     7
#define SC2705_OC_M_MASK                      BIT(7)
#define SC2705_ACTUATOR_M_SHIFT               6
#define SC2705_ACTUATOR_M_MASK                BIT(6)
#define SC2705_WARNING_M_SHIFT                5
#define SC2705_WARNING_M_MASK                 BIT(5)
#define SC2705_PAT_FAULT_M_SHIFT              4
#define SC2705_PAT_FAULT_M_MASK               BIT(4)
#define SC2705_CALIB_DONE_M_SHIFT             3
#define SC2705_CALIB_DONE_M_MASK              BIT(3)
#define SC2705_PAT_DONE_M_SHIFT               2
#define SC2705_PAT_DONE_M_MASK                BIT(2)
#define SC2705_SOFT_SHUTDOWN_M_SHIFT          1
#define SC2705_SOFT_SHUTDOWN_M_MASK           BIT(1)
#define SC2705_SEQ_CONTINUE_M_SHIFT           0
#define SC2705_SEQ_CONTINUE_M_MASK            BIT(0)

/* SC2705_CIF_I2C1 = 0x08 */
#define SC2705_I2C_WR_MODE_SHIFT              7
#define SC2705_I2C_WR_MODE_MASK               BIT(7)
#define SC2705_I2C_TO_ENABLE_SHIFT            6
#define SC2705_I2C_TO_ENABLE_MASK             BIT(6)
#define SC2705_PC_DONE_SHIFT                  5
#define SC2705_PC_DONE_MASK                   BIT(5)
#define SC2705_PC_MODE_SHIFT                  4
#define SC2705_PC_MODE_MASK                   BIT(4)

/* SC2705_CIF_I2C2 = 0x09 */
#define SC2705_IF_BASE_ADDR_SHIFT             0
#define SC2705_IF_BASE_ADDR_MASK              (0x7F << 0)

/* SC2705_FRQ_LRA_PER_H = 0x0A */
#define SC2705_LRA_PER_H_SHIFT                0
#define SC2705_LRA_PER_H_MASK                 (0xFF << 0)

/* SC2705_FRQ_LRA_PER_L = 0x0B */
#define SC2705_LRA_PER_L_SHIFT                0
#define SC2705_LRA_PER_L_MASK                 (0x7F << 0)

/* SC2705_ACTUATOR1 = 0x0C */
#define SC2705_ACTUATOR_NOMMAX_SHIFT          0
#define SC2705_ACTUATOR_NOMMAX_MASK           (0xFF << 0)

/* SC2705_ACTUATOR2 = 0x0D */
#define SC2705_ACTUATOR_ABSMAX_SHIFT          0
#define SC2705_ACTUATOR_ABSMAX_MASK           (0xFF << 0)

/* SC2705_ACTUATOR3 = 0x0E */
#define SC2705_IMAX_SHIFT                     0
#define SC2705_IMAX_MASK                      (0x1F << 0)

/* SC2705_TOP_CFG1 = 0x13 */
#define SC2705_ACTUATOR_TYPE_SHIFT            4
#define SC2705_ACTUATOR_TYPE_MASK             BIT(4)
#define SC2705_FREQ_TRACK_EN_SHIFT            3
#define SC2705_FREQ_TRACK_EN_MASK             BIT(3)
#define SC2705_ACCELERATION_EN_SHIFT          2
#define SC2705_ACCELERATION_EN_MASK           BIT(2)
#define SC2705_RAPID_STOP_EN_SHIFT            1
#define SC2705_RAPID_STOP_EN_MASK             BIT(1)
#define SC2705_AMP_PID_EN_SHIFT               0
#define SC2705_AMP_PID_EN_MASK                BIT(0)
#define SC2705_TOP_CFG1_ALL_MASK                0x1F

/* SC2705_TOP_CFG2 = 0x14 */
#define SC2705_MEM_DATA_SIGNED_SHIFT          4
#define SC2705_MEM_DATA_SIGNED_MASK           BIT(4)
#define SC2705_FULL_BRAKE_THR_SHIFT           0
#define SC2705_FULL_BRAKE_THR_MASK            (0xF << 0)

/* SC2705_TOP_CFG3 = 0x15 */
#define SC2705_VBAT_MARGIN_SHIFT              0
#define SC2705_VBAT_MARGIN_MASK               (0xF << 0)

/* SC2705_TOP_CFG4 = 0x16 */
#define SC2705_V2I_FACTOR_FREEZE_SHIFT        7
#define SC2705_V2I_FACTOR_FREEZE_MASK         BIT(7)
#define SC2705_TST_CALIB_IMPEDANCE_DIS_SHIFT  6
#define SC2705_TST_CALIB_IMPEDANCE_DIS_MASK   BIT(6)
#define SC2705_ERM_CALIB_IDAC_LEVEL_SHIFT     0
#define SC2705_ERM_CALIB_IDAC_LEVEL_MASK      (0x3F << 0)

/* SC2705_TOP_CTL1 = 0x21 */
#define SC2705_CALIB_REQ_SHIFT                5
#define SC2705_CALIB_REQ_MASK                 BIT(5)
#define SC2705_SEQ_START_SHIFT                4
#define SC2705_SEQ_START_MASK                 BIT(4)
#define SC2705_PATTERN_INACTIVE_MODE_SHIFT    3
#define SC2705_PATTERN_INACTIVE_MODE_MASK     BIT(3)
#define SC2705_OPERATION_MODE_SHIFT           0
#define SC2705_OPERATION_MODE_MASK            (0x7 << 0)

/* SC2705_TOP_CTL2 = 0x22 */
#define SC2705_OVERRIDE_VAL_SHIFT             0
#define SC2705_OVERRIDE_VAL_MASK              (0xFF << 0)

/* SC2705_SEQ_CTL1 = 0x23 */
#define SC2705_SEQ_CONTINUE_SHIFT             0
#define SC2705_SEQ_CONTINUE_MASK              BIT(0)

/* SC2705_SEQ_CTL2 = 0x24 */
#define SC2705_PS_SEQ_LOOP_SHIFT              4
#define SC2705_PS_SEQ_LOOP_MASK               (0xF << 4)
#define SC2705_PS_SEQ_ID_SHIFT                0
#define SC2705_PS_SEQ_ID_MASK                 (0xF << 0)

/* SC2705_MEM_CTL1 = 0x25 */
#define SC2705_PATTERN_BASE_ADDR_SHIFT        0
#define SC2705_PATTERN_BASE_ADDR_MASK         (0xFF << 0)

/* SC2705_MEM_CTL2 = 0x26 */
#define SC2705_PATTERN_MEM_LOCK_SHIFT         7
#define SC2705_PATTERN_MEM_LOCK_MASK          BIT(7)

/* SC2705_ADC_DATA_H1 = 0x27 */
#define SC2705_ADC_VBAT_H_SHIFT               0
#define SC2705_ADC_VBAT_H_MASK                (0xFF << 0)

/* SC2705_ADC_DATA_L1 = 0x28 */
#define SC2705_ADC_VBAT_L_SHIFT               0
#define SC2705_ADC_VBAT_L_MASK                (0x7F << 0)

/* SC2705_IRQ_STATUS2 = 0x76 */
#define SC2705_STA_TEST_ADC_SAT_SHIFT         7
#define SC2705_STA_TEST_ADC_SAT_MASK          BIT(7)

/* SC2705_IRQ_MASK2 = 0x77 */
#define SC2705_TEST_ADC_SAT_M_SHIFT           7
#define SC2705_TEST_ADC_SAT_M_MASK            BIT(7)

/* SC2705_SNP_MEM_0 - SC2705_SNP_MEM_89 = 0x78 - 0xD1 */
#define SC2705_SNP_MEM_SHIFT                  0
#define SC2705_SNP_MEM_MASK                   (0xFF << 0)

#endif /* __SC2705_HAPTIC_REGISTERS_H */
