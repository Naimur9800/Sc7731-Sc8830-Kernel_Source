/*
 * File: altek_log.c							*
 * Description: log variable						*
 *									*
 * (C)Copyright Altek Corporation 2016					*
 *									*
 * History:								*
 *   2016/05/27; Caedmon Lai; Initial version				*
 */
#include "altek_log_local.h"
#include "altek_common.h"

u32 drvlog_threshold = 6;
u32 fwlog_threshold = NUMBER_ZERO;
u8 fwlog_force_switch = NUMBER_ZERO;
