/*
 * File: altek_cmd.h							*
 * Description: ISP Device Node Cmd Parser				*
 *									*
 * (C)Copyright Altek Corporation 2016					*
 *									*
 * History:								*
 *   2016/06/17; ; Initial version					*
 */

#ifndef _ALTEK_PARSE_CMD_H_
#define _ALTEK_PARSE_CMD_H_

#if 1 /* declarations copied from lib/kstrtox.h */
#define KSTRTOX_OVERFLOW (1U << 31)
extern const char *_parse_integer_fixup_radix(
			const char *s, unsigned int *base);
extern unsigned int _parse_integer(
			const char *s, unsigned int base,
			unsigned long long *res);
#endif

extern ssize_t isp_cmd(char *cmd);

#endif /* _ALTEK_PARSE_CMD_H_ */
