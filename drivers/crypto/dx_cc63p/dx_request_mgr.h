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

/* \file request_mgr.h
   Request Manager
 */

#ifndef __REQUEST_MGR_H__
#define __REQUEST_MGR_H__

#include "hw_queue_defs.h"

int request_mgr_init(struct dx_drvdata *drvdata);

/*!
 * Enqueue caller request to crypto hardware.
 * 
 * \param drvdata 
 * \param dx_req The request to enqueue
 * \param desc The crypto sequence
 * \param len The crypto sequence length
 * \param is_dout If "true": completion is handled by the caller 
 *      	  If "false": this function adds a dummy descriptor completion
 *      	  and waits upon completion signal.
 * 
 * \return int Returns -EINPROGRESS if "is_dout=ture"; "0" if "is_dout=false"
 */
int send_request(
	struct dx_drvdata *drvdata, struct dx_crypto_req *dx_req,
	HwDesc_s *desc, unsigned int len, bool is_dout);

int send_request_init(
	struct dx_drvdata *drvdata, HwDesc_s *desc, unsigned int len);

void complete_request(struct dx_drvdata *drvdata);

void request_mgr_fini(struct dx_drvdata *drvdata);

#ifdef CONFIG_PM_RUNTIME
int dx_request_mgr_runtime_resume_queue(struct dx_drvdata *drvdata);

int dx_request_mgr_runtime_suspend_queue(struct dx_drvdata *drvdata);

bool dx_request_mgr_is_queue_runtime_suspend(struct dx_drvdata *drvdata);
#endif

#endif /*__REQUEST_MGR_H__*/
