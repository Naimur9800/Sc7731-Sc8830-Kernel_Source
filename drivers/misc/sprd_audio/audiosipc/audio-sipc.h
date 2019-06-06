/*
 * include/sound/audio-sipc.h
 *
 * SPRD SoC VBC -- SpreadTrum SOC SIPC for audio Common function.
 *
 * Copyright (C) 2015 SpreadTrum Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY ork FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __AUDIO_SIPC_H
#define __AUDIO_SIPC_H

#include "audio-smsg.h"

#define AUDIO_SIPC_WAIT_FOREVER	(-1)

#define SND_VBC_DSP_FUNC_STARTUP 4
#define SND_VBC_DSP_FUNC_SHUTDOWN 5
#define SND_VBC_DSP_FUNC_HW_PARAMS 6
#define SND_VBC_DSP_FUNC_HW_FREE 7
#define SND_VBC_DSP_FUNC_HW_PREPARE 8
#define SND_VBC_DSP_FUNC_HW_TRIGGER 9
#define SND_VBC_DSP_IO_KCTL_GET 10
#define SND_VBC_DSP_IO_KCTL_SET 11
#define SND_VBC_DSP_IO_SHAREMEM_GET 12
#define SND_VBC_DSP_IO_SHAREMEM_SET 13

int aud_ipc_ch_open(uint16_t channel);
int aud_ipc_ch_close(uint16_t channel);
int aud_send_cmd(uint16_t channel, int id, int stream, uint32_t cmd,
		 void *para, size_t n, int32_t timeout);
int aud_send_block_param(uint16_t channel, int id, int stream, int type,
			 void *buf, size_t n, int32_t timeout);
int aud_recv_block_param(uint16_t channel, int id, int stream, int type,
			 void *buf, uint32_t size, int32_t timeout);
int aud_recv_cmd(uint16_t channel, uint32_t cmd, int *ret, int32_t timeout);
int aud_send_use_noreplychan(uint32_t cmd, uint32_t value0, uint32_t value1,
			     uint32_t value2, int32_t value3);
int aud_send_cmd_no_wait(uint16_t channel, uint32_t cmd, uint32_t value0,
			 uint32_t value1, uint32_t value2, int32_t value3);
int aud_send_cmd_no_param(uint16_t channel, uint32_t cmd,
			  uint32_t value0, uint32_t value1,
			  uint32_t value2, int32_t value3, int32_t timeout);
uint32_t aud_ipc_dump(void *buf, uint32_t buf_bytes);

#endif /* __AUDIO_SIPC_H */
