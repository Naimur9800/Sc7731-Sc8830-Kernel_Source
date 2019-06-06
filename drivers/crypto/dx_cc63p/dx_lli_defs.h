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


#ifndef _DX_LLI_DEFS_H_
#define _DX_LLI_DEFS_H_
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif
#include "dx_bitops.h"


#define DX_MAX_MLLI_ENTRY_SIZE 0x10000

#define LLI_SET_ADDR(lli_p, addr) \
		BITFIELD_SET(((uint32_t *)(lli_p))[LLI_WORD0_OFFSET], LLI_LADDR_BIT_OFFSET, LLI_LADDR_BIT_SIZE, (addr & UINT32_MAX)); \
		BITFIELD_SET(((uint32_t *)(lli_p))[LLI_WORD1_OFFSET], LLI_HADDR_BIT_OFFSET, LLI_HADDR_BIT_SIZE, ((addr >> 32) & UINT16_MAX));

#define LLI_SET_SIZE(lli_p, size) \
		BITFIELD_SET(((uint32_t *)(lli_p))[LLI_WORD1_OFFSET], LLI_SIZE_BIT_OFFSET, LLI_SIZE_BIT_SIZE, size)


/* Size of entry */
#define LLI_ENTRY_WORD_SIZE 2
#define LLI_ENTRY_BYTE_SIZE (LLI_ENTRY_WORD_SIZE * sizeof(uint32_t))

/* Word0[31:0] = ADDR[31:0] */
#define LLI_WORD0_OFFSET 0
#define LLI_LADDR_BIT_OFFSET 0
#define LLI_LADDR_BIT_SIZE 32
/* Word1[31:16] = ADDR[47:32]; Word1[15:0] = SIZE */
#define LLI_WORD1_OFFSET 1
#define LLI_SIZE_BIT_OFFSET 0
#define LLI_SIZE_BIT_SIZE 16
#define LLI_HADDR_BIT_OFFSET 16
#define LLI_HADDR_BIT_SIZE 16

#endif /*_DX_LLI_DEFS_H_*/
