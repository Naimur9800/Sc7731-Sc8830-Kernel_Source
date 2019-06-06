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

/*!
 * \file dx_bitops.h
 * Bit fields operations macros.
 */
#ifndef _DX_BITOPS_H_
#define _DX_BITOPS_H_

#define BITMASK(mask_size) (((mask_size) < 32) ?	\
	((1UL << (mask_size)) - 1) : 0xFFFFFFFFUL)
#define BITMASK_AT(mask_size, mask_offset) (BITMASK(mask_size) << (mask_offset))

#define BITFIELD_GET(word, bit_offset, bit_size) \
	(((word) >> (bit_offset)) & BITMASK(bit_size))
#define BITFIELD_SET(word, bit_offset, bit_size, new_val)   do {    \
	word = ((word) & ~BITMASK_AT(bit_size, bit_offset)) |	    \
		(((new_val) & BITMASK(bit_size)) << (bit_offset));  \
} while (0)

/* Is val aligned to "align" ("align" must be power of 2) */
#ifndef IS_ALIGNED
#define IS_ALIGNED(val, align)		\
	(((uint32_t)(val) & ((align) - 1)) == 0)
#endif

#define SWAP_ENDIAN(word)		\
	(((word) >> 24) | (((word) & 0x00FF0000) >> 8) | \
	(((word) & 0x0000FF00) << 8) | (((word) & 0x000000FF) << 24))

#ifdef BIG__ENDIAN
#define SWAP_TO_LE(word) SWAP_ENDIAN(word)
#define SWAP_TO_BE(word) word
#else
#define SWAP_TO_LE(word) word
#define SWAP_TO_BE(word) SWAP_ENDIAN(word)
#endif



/* Is val a multiple of "mult" ("mult" must be power of 2) */
#define IS_MULT(val, mult)              \
	(((val) & ((mult) - 1)) == 0)

#define IS_NULL_ADDR(adr)		\
	(!(adr))

#endif /*_DX_BITOPS_H_*/
