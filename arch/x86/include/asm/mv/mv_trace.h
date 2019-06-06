/* ----------------------------------------------------------------------------
 *  Copyright (C) 2016 Intel Mobile Communications GmbH

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

  ---------------------------------------------------------------------------*/

#ifndef __MV_TRACE_H__
#define __MV_TRACE_H__


#define STR_SIZE 16

struct trace_shared_head {
	char name[STR_SIZE];
	char func[STR_SIZE];
	char file[STR_SIZE];
	uint16_t line;
	uint16_t enabled;
};

#define TRACE_SHARED_HEAD_SIZE sizeof(struct trace_shared_head)


struct trace_info {
	uint16_t size;
	uint16_t ntrace;
	uint16_t idx;
	uint16_t reserved;
	char name[16];
};


enum vmm_trace_ops {
	VTO_GET_TRACE_ADDR,
	VTO_SET_TRACE,
};


void mv_vmm_trace(enum vmm_trace_ops ops, uint32_t *parm1,
		uint32_t *parm2, uint32_t *parm3);


#endif /* __MV_TRACE_H__ */

