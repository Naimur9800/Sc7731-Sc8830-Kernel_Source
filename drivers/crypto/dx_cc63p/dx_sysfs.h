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

/* \file dx_sysfs.h
   Discretix sysfs APIs
 */

#ifndef __DX_SYSFS_H__
#define __DX_SYSFS_H__

#include <asm/timex.h>

/* forward declaration */
struct dx_drvdata;

enum stat_phase {
	STAT_PHASE_0 = 0,
	STAT_PHASE_1,
	STAT_PHASE_2,
	STAT_PHASE_3,
	STAT_PHASE_4,
	STAT_PHASE_5,
	STAT_PHASE_6,
	MAX_STAT_PHASES,
};
enum stat_op {
	STAT_OP_TYPE_NULL = 0,
	STAT_OP_TYPE_ENCODE,
	STAT_OP_TYPE_DECODE,
	STAT_OP_TYPE_SETKEY,
	STAT_OP_TYPE_GENERIC,
	MAX_STAT_OP_TYPES,
};

int dx_sysfs_init(struct kobject *sys_dev_obj, struct dx_drvdata *drvdata);
void dx_sysfs_fini(void);
void update_host_stat(unsigned int op_type, unsigned int phase, cycles_t result);
void update_cc_stat(unsigned int op_type, unsigned int phase, unsigned int elapsed_cycles);
void display_all_stat_db(void);

#endif /*__DX_SYSFS_H__*/
