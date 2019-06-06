/* ----------------------------------------------------------------------------
   Copyright (C) 2013 Intel Mobile Communications GmbH

	Sec Class: Intel Confidential (IC)

   All rights reserved.
   ----------------------------------------------------------------------------
   This document contains proprietary information belonging to IMC.
   Passing on and copying of this document, use
   and communication of its contents is not permitted without prior written
   authorisation.
  ---------------------------------------------------------------------------*/
#ifndef _MV_SVC_VMMLOG_H
#define _MV_SVC_VMMLOG_H

#define VMMLOG_API_VER			1
#define VMMLOG_API_MIN_VER		1

/**
  @typedef vmm_log_opcode
  @brief   enumeration containing the operation of vmm log service
**/
enum vmm_log_opcode {
	VMM_PRINT_LOG = 0,
	VMM_CONNECT_LOG,
	VMM_DISCONNECT_LOG,
	VMM_LOG_LEVEL,
	VMM_LOG_SYNC_TIMESTAMP
};

enum vmmlog_level_value {
	VMM_EMERG_VALUE = 0,
	VMM_ALERT_VALUE,	/* VMM alert log level */
	VMM_CRIT_VALUE,		/* VMM critical log level */
	VMM_ERR_VALUE,		/* VMM Error log level */
	VMM_WARNING_VALUE,	/* VMM warning log level */
	VMM_NOTITY_VALUE,	/* VMM notity log level */
	VMM_INFO_VALUE,		/* VMM informative log level */
	VMM_DEBUG_VALUE		/* VMM debug log level */
};

int32_t mv_svc_vmm_logs_print(char *msg, int msg_len);
int32_t mv_svc_vmm_logs_connect(uint32_t parm);
int32_t mv_svc_vmm_logs_disconnect(uint32_t parm);
int32_t mv_svc_vmm_logs_level(uint32_t uart_level, uint32_t kernel_level);
int32_t mv_svc_vmm_logs_sync_timestamp(uint64_t ts_nsec);
bool    mv_svc_vmm_logs_is_initialized(void);

#endif /* _MV_SVC_VMMLOG */
