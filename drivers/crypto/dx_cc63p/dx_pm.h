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

/* \file dx_power_mgr.h
    */

#ifndef __POWER_MGR_H__
#define __POWER_MGR_H__


#include "dx_config.h"
#include "dx_driver.h"


#define DX_SUSPEND_TIMEOUT 3000


int dx_power_mgr_init(struct dx_drvdata *drvdata);

void dx_power_mgr_fini(struct dx_drvdata *drvdata);

#ifdef CONFIG_PM_RUNTIME
int dx_power_mgr_runtime_suspend(struct device *dev);

int dx_power_mgr_runtime_resume(struct device *dev);

int dx_power_mgr_runtime_get(struct device *dev);

int dx_power_mgr_runtime_put_suspend(struct device *dev);
#endif
#endif /*__POWER_MGR_H__*/

