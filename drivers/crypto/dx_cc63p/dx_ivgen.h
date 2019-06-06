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

#ifndef __DX_IVGEN_H__
#define __DX_IVGEN_H__

#include "hw_queue_defs.h"


#define DX_IVPOOL_SEQ_LEN 8

/*!
 * Allocates iv-pool and maps resources. 
 * This function generates the first IV pool.  
 * 
 * \param drvdata Driver's private context
 * 
 * \return int Zero for success, negative value otherwise.
 */
int dx_ivgen_init(struct dx_drvdata *drvdata);

/*!
 * Free iv-pool and ivgen context.
 *  
 * \param drvdata 
 */
void dx_ivgen_fini(struct dx_drvdata *drvdata);

/*!
 * Generates the initial pool in SRAM. 
 * This function should be invoked when resuming DX driver. 
 * 
 * \param drvdata 
 *  
 * \return int Zero for success, negative value otherwise.
 */
int dx_ivgen_init_sram_pool(struct dx_drvdata *drvdata);

/*!
 * Acquires 16 Bytes IV from the iv-pool
 * 
 * \param drvdata Driver private context
 * \param iv_out_dma Array of physical IV out addresses
 * \param iv_out_dma_len Length of iv_out_dma array (additional elements of iv_out_dma array are ignore)
 * \param iv_out_size May be 8 or 16 bytes long 
 * \param iv_seq IN/OUT array to the descriptors sequence
 * \param iv_seq_len IN/OUT pointer to the sequence length 
 *  
 * \return int Zero for success, negative value otherwise. 
 */
int dx_ivgen_getiv(
	struct dx_drvdata *drvdata,
	dma_addr_t iv_out_dma[],
	unsigned int iv_out_dma_len,
	unsigned int iv_out_size,
	HwDesc_s iv_seq[],
	unsigned int *iv_seq_len);

#endif /*__DX_IVGEN_H__*/
