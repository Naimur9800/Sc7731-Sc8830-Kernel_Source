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

#ifndef __DX_SRAM_MGR_H__
#define __DX_SRAM_MGR_H__


#ifndef DX_CC_SRAM_SIZE
#define DX_CC_SRAM_SIZE 4096
#endif

struct dx_drvdata;

/**
 * Address (offset) within CC internal SRAM
 */

typedef uint64_t dx_sram_addr_t;

#define NULL_SRAM_ADDR ((dx_sram_addr_t)-1)

/*!
 * Initializes SRAM pool. 
 * The first X bytes of SRAM are reserved for ROM usage, hence, pool 
 * starts right after X bytes. 
 *  
 * \param drvdata 
 *  
 * \return int Zero for success, negative value otherwise.
 */
int dx_sram_mgr_init(struct dx_drvdata *drvdata);

/*!
 * Uninits SRAM pool.
 * 
 * \param drvdata 
 */
void dx_sram_mgr_fini(struct dx_drvdata *drvdata);

/*!
 * Allocated buffer from SRAM pool. 
 * Note: Caller is responsible to free the LAST allocated buffer. 
 * This function does not taking care of any fragmentation may occur 
 * by the order of calls to alloc/free. 
 * 
 * \param drvdata 
 * \param size The requested bytes to allocate
 */
dx_sram_addr_t dx_sram_mgr_alloc(struct dx_drvdata *drvdata, uint32_t size);

/**
 * dx_sram_mgr_const2sram_desc() - Create const descriptors sequence to
 *	set values in given array into SRAM. 
 * Note: each const value can't exceed word size.
 * 
 * @src:	  A pointer to array of words to set as consts.
 * @dst:	  The target SRAM buffer to set into
 * @nelements:	  The number of words in "src" array
 * @seq:	  A pointer to the given IN/OUT descriptor sequence
 * @seq_len:	  A pointer to the given IN/OUT sequence length
 */
void dx_sram_mgr_const2sram_desc(
	const uint32_t *src, dx_sram_addr_t dst,
	unsigned int nelement,
	HwDesc_s *seq, unsigned int *seq_len);

#endif /*__DX_SRAM_MGR_H__*/
