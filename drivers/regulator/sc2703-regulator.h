/*
 * SPDX-License-Identifier: GPL-2.0
 * Regulator device driver registers for SC2703
 *
 * Copyright (c) 2018 Dialog Semiconductor.
 */

#ifndef _SC2703_COREBUCK_REGISTERS_H__
#define _SC2703_COREBUCK_REGISTERS_H__

#include <linux/bitops.h>

#define SC2703_COREBUCK_EVENT                  0x00
#define SC2703_COREBUCK_STATUS                 0x01
#define SC2703_COREBUCK_IRQ_MASK               0x02
#define SC2703_COREBUCK_BUCK1_0                0x03
#define SC2703_COREBUCK_BUCK1_1                0x04
#define SC2703_COREBUCK_BUCK1_2                0x05
#define SC2703_COREBUCK_BUCK1_3                0x06
#define SC2703_COREBUCK_BUCK1_4                0x07
#define SC2703_COREBUCK_BUCK1_5                0x08
#define SC2703_COREBUCK_BUCK1_6                0x09
#define SC2703_COREBUCK_BUCK1_7                0x0a

/* SC2703_COREBUCK_EVENT = 0x0100 */
#define SC2703_EVT_PWRGOOD1_SHIFT                    0
#define SC2703_EVT_PWRGOOD1_MASK                     BIT(0)
#define SC2703_EVT_OVCURR1_SHIFT                     4
#define SC2703_EVT_OVCURR1_MASK                      BIT(4)
#define SC2703_EVT_VSYS_UV_OT_VREF_FLT_SHIFT         5
#define SC2703_EVT_VSYS_UV_OT_VREF_FLT_MASK          BIT(5)

/* SC2703_COREBUCK_STATUS = 0x0101 */
#define SC2703_PWRGOOD1_SHIFT                        0
#define SC2703_PWRGOOD1_MASK                         BIT(0)
#define SC2703_OVCURR1_SHIFT                         4
#define SC2703_OVCURR1_MASK                          BIT(4)
#define SC2703_VSYS_UV_OT_VREF_FLT_SHIFT             5
#define SC2703_VSYS_UV_OT_VREF_FLT_MASK              BIT(5)

/* SC2703_COREBUCK_IRQ_MASK = 0x0102 */
#define SC2703_M_PWRGOOD1_SHIFT                      0
#define SC2703_M_PWRGOOD1_MASK                       BIT(0)
#define SC2703_M_OVCURR1_SHIFT                       4
#define SC2703_M_OVCURR1_MASK                        BIT(4)
#define SC2703_M_VSYS_UV_OT_VREF_FLT_SHIFT           5
#define SC2703_M_VSYS_UV_OT_VREF_FLT_MASK            BIT(5)

/* SC2703_COREBUCK_BUCK1_0 = 0x0103 */
#define SC2703_CH1_A_VOUT_SHIFT                      0
#define SC2703_CH1_A_VOUT_MASK                       GENMASK(6, 0)

/* SC2703_COREBUCK_BUCK1_1 = 0x0104 */
#define SC2703_CH1_SLEW_VU_SHIFT                     0
#define SC2703_CH1_SLEW_VU_MASK                      GENMASK(2, 0)
#define SC2703_CH1_SLEW_VD_SHIFT                     4
#define SC2703_CH1_SLEW_VD_MASK                      GENMASK(6, 4)

/* SC2703_COREBUCK_BUCK1_2 = 0x0105 */
#define SC2703_CH1_PD_DIS_SHIFT                      0
#define SC2703_CH1_PD_DIS_MASK                       BIT(0)
#define SC2703_CH1_SLEW_PU_SHIFT                     1
#define SC2703_CH1_SLEW_PU_MASK                      GENMASK(3, 1)
#define SC2703_CH1_SLEW_PD_SHIFT                     4
#define SC2703_CH1_SLEW_PD_MASK                      GENMASK(6, 4)

/* SC2703_COREBUCK_BUCK1_3 = 0x0106 */
#define SC2703_CH1_ILIM_SHIFT                        0
#define SC2703_CH1_ILIM_MASK                         GENMASK(3, 0)
#define SC2703_PG1_MASK_SHIFT                        4
#define SC2703_PG1_MASK_MASK                         GENMASK(5, 4)
#define SC2703_OC1_MASK_SHIFT                        6
#define SC2703_OC1_MASK_MASK                         BIT(6)

/* SC2703_COREBUCK_BUCK1_4 = 0x0107 */
#define SC2703_CH1_VMAX_SHIFT                        0
#define SC2703_CH1_VMAX_MASK                         GENMASK(6, 0)

/* SC2703_COREBUCK_BUCK1_5 = 0x0108 */
#define SC2703_CH1_EN_SHIFT                          0
#define SC2703_CH1_EN_MASK                           GENMASK(0, 0)

/* SC2703_COREBUCK_BUCK1_6 = 0x0109 */
#define SC2703_CH1_B_VOUT_SHIFT                      0
#define SC2703_CH1_B_VOUT_MASK                       GENMASK(6, 0)

/* SC2703_COREBUCK_BUCK1_7 = 0x010a */
#define SC2703_CH1_A_CMD_SHIFT                       0
#define SC2703_CH1_A_CMD_MASK                        GENMASK(1, 0)

#endif	/* _SC2703_COREBUCK_REGISTERS_H__ */
