/**************************************************************
* Copyright 2014 (c) Discretix Technologies Ltd.              *
* This software is protected by copyright, international      *
* treaties and various patents. Any copy or reproduction of   *
* the software as permitted below, must include this          *
* Copyright Notice as well as any other notices provided      *
* under the relevant license.                                 *
*                                                             *
* This software shall be governed by, and may be used and     *
* redistributed under the terms and conditions of the GNU     *
* General Public License version 2, as published by the       *
* Free Software Foundation.                                   *
* This software is distributed in the hope that it will be    *
* useful, but WITHOUT ANY liability and WARRANTY; without     *
* even the implied warranty of MERCHANTABILITY or FITNESS     *
* FOR A PARTICULAR PURPOSE. See the GNU General Public        *
* License for more details.                                   *
* You should have received a copy of the GNU General          *
* Public License along with this software; if not, please     *
* write to the Free Software Foundation, Inc.,                *
* 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.   *
**************************************************************/

/* pseudo dx_hal.h for cc63_perf_test_driver (to be able to include code from CC drivers) */

#ifndef __DX_HAL_H__
#define __DX_HAL_H__

#include <linux/io.h>

#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
/* CC registers are always 32 bit wide (even on 64 bit platforms) */
#define READ_REGISTER(_addr) ioread32((_addr))
#define WRITE_REGISTER(_addr, _data)  iowrite32((_data), (_addr))
#else
#error Unsupported platform
#endif

#define DX_HAL_WriteCcRegister(offset, val) WRITE_REGISTER(cc_base + offset, val)
#define DX_HAL_ReadCcRegister(offset) READ_REGISTER(cc_base + offset)

#endif
