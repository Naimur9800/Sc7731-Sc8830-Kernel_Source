/*
 * File:shub_protocol.c
 * Author:Sensor Hub Team
 * Created:2015-10-21
 * Description:SPRD Sensor Hub Driver
 *
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#include <linux/string.h>
#include <linux/kernel.h>
#include "shub_common.h"
#include "shub_protocol.h"

struct shub_data_processor shub_stream_processor;
struct shub_data_processor shub_stream_processor_nwu;

void debuginfor(void *src, int len)
{
	unsigned char *ptmp = src;

	while (len) {
		SH_LOG("0x%02x\n", *ptmp++);
		len--;
	}
}

/*  Function :  shub_search_flag
  * Description:
  * it find the matic word form the data buffer
  *  Parameters:
  *  stream : point the current  parse  data
  *  data : point the uart buffer data
  *  len       : the receive data length
  *  processed_len : it deal with data len
  *  Return : void
  */
static void shub_search_flag(struct shub_data_processor *stream,
			     unsigned char *data, unsigned short len,
			     unsigned short *processed_len)
{
	unsigned short headsize = stream->head_size;
	unsigned char *start_data = data;
	int i = 0;
	/* unsigned char *pEndData = data + Len; */

	/* the magic number is 4 '~' */
	for (i = 0; i < len; i++) {
		if (*start_data == 0x7E) {
			headsize++;
			/* we got the 4 magic '~'  */
			if (headsize == SHUB_MAGIC_NUMBER_LEN) {
				start_data++;
				memset(stream->cur_header, SHUB_MAGIC_NUMBER,
				       SHUB_MAGIC_NUMBER_LEN);
				stream->state = SHUB_RECV_COLLECT_HEAD;
				break;
			}
		} else {
			headsize = 0;
		}
		start_data++;
	}
	stream->head_size = headsize;
	*processed_len = start_data - data;

}


/*   Function :  shub_checksum
*   Description :
*  it Calculate the CRC  for the 8 bytes in head buffer
*   Parameters :
*   pHeadData: point the head  data
*   Return : void
*/
static unsigned short shub_checksum(unsigned char *head_data)
{
	/* The first 4 octet is 0x7e
	 * 0x7e7e + 0x7e7e = 0xfcfc
	 */
	unsigned int sum = 0xfcfc;
	unsigned short nAdd;

	head_data += 4;
	nAdd = *head_data++;
	nAdd <<= 8;
	nAdd += *head_data++;
	sum += nAdd;

	nAdd = *head_data++;
	nAdd <<= 8;
	nAdd += *head_data++;
	sum += nAdd;

	/* The carry is 2 at most, so we need 2 additions at most. */
	sum = (sum & 0xFFFF) + (sum >> 16);
	sum = (sum & 0xFFFF) + (sum >> 16);

	return (unsigned short)(~sum);
}

 /*   Function :  shub_data_checksum
   *   Description :
   *   it auto fill the encode head context in one packet
   *   Parameters:
   *   in_data : point the send data context
   *   out_data : the one packet head address
   */
static unsigned short shub_data_checksum(unsigned char *data,
	 unsigned short out_len)
{
	unsigned int sum = 0;
	unsigned short nAdd;

	if ((data == NULL) || (out_len == 0))
		return 0;

	while (out_len > 2) {
		nAdd = *data++;
		nAdd <<= 8;
		nAdd += *data++;
		sum += nAdd;
		out_len -= 2;
	}
	if (out_len == 2) {
		nAdd = *data++;
		nAdd <<= 8;
		nAdd += *data++;
		sum += nAdd;
	} else {
		nAdd = *data++;
		nAdd <<= 8;
		sum += nAdd;
	}
	/*The carry is 2 at most, so we need 2 additions at most. */
	sum = (sum & 0xFFFF) + (sum >> 16);
	sum = (sum & 0xFFFF) + (sum >> 16);

	return (unsigned short)(~sum);
}

static void research_flag(struct shub_data_processor *stream)
{
	unsigned char *start = stream->cur_header;
	/* slip the first '~'  */
	if (start[4] == 0x7e) {
		memmove(start + 4, start + 5, 5);
		stream->head_size = 9;
		stream->state = SHUB_RECV_COLLECT_HEAD;
	} else {
		unsigned int value4Byte;

		value4Byte = start[5];
		value4Byte <<= 8;
		value4Byte |= start[6];
		value4Byte <<= 8;
		value4Byte |= start[7];
		value4Byte <<= 8;
		value4Byte |= start[8];
		 /* get the [5--8] is magic head  */
		if (value4Byte == 0x7e7e7e7e) {
			start[4] = start[9];
			stream->head_size = 5;
			stream->state = SHUB_RECV_COLLECT_HEAD;
		} else {	/* get [6--9] */
			value4Byte <<= 8;
			value4Byte |= start[9];
			if (value4Byte == 0x7e7e7e7e) {
				stream->head_size = 4;
				stream->state = SHUB_RECV_COLLECT_HEAD;
			} else if (0x7e7e7e == (0xffffff & value4Byte)) {
				stream->head_size = 3;
				stream->state = SHUB_RECV_SEARCH_FLAG;
			} else if (0x7e7e == (0xffff & value4Byte)) {
				stream->head_size = 2;
				stream->state = SHUB_RECV_SEARCH_FLAG;
			} else if (0x7e == (0xff & value4Byte)) {
				stream->head_size = 1;
				stream->state = SHUB_RECV_SEARCH_FLAG;
			} else {
				stream->head_size = 0;
				stream->state = SHUB_RECV_SEARCH_FLAG;
			}
		}
	}
}

static void shub_collect_header(struct shub_data_processor *stream,
				unsigned char *data, unsigned short len,
				unsigned short *processed_len)
{
	unsigned short headsize = stream->head_size;
	unsigned short remain_len = SHUB_MAX_HEAD_LEN - headsize;
	unsigned short processed_length;
	unsigned short crc;
	unsigned short crc_inframe;

	processed_length = remain_len > len ? len : remain_len;
	memcpy(stream->cur_header + headsize, data, processed_length);
	headsize += processed_length;
	*processed_len = processed_length;
	stream->head_size = headsize;

	if (headsize != SHUB_MAX_HEAD_LEN) /* We have not got 10 bytes*/
		return;

	/* We have got 10 bytes
	 * Calculate the checksum (only 8 bytes in head buffer)
	 */
	crc = shub_checksum(stream->cur_header);
	crc_inframe = stream->cur_header[8];
	crc_inframe <<= 8;
	crc_inframe |= stream->cur_header[9];
	if (crc == crc_inframe)	{	/* We have got a right header*/
		unsigned short data_len;

		/* Set the frame length here*/
		data_len = stream->cur_header[6];
		data_len <<= 8;
		data_len |= stream->cur_header[7];
		stream->data_len = data_len;
		stream->cmd_data.type = stream->cur_header[4];
		stream->cmd_data.subtype = stream->cur_header[5];
		stream->cmd_data.length = data_len;
		if (data_len == 0) {
			shub_dispatch(&stream->cmd_data);
			stream->state = SHUB_RECV_SEARCH_FLAG;
			stream->head_size = 0;
			stream->received_data_len = 0;
			stream->data_len = 0;
		} else {
			if (data_len <
			    (MAX_MSG_BUFF_SIZE - SHUB_MAX_HEAD_LEN -
			     SHUB_MAX_DATA_CRC)) {
				stream->state = SHUB_RECV_DATA;
			} else {
				stream->error_num++;
				research_flag(stream);
				SH_ERR("error=%d dataLen=%d\n",
				       stream->error_num, data_len);
			}
		}

	} else {
		stream->error_num++;
		SH_ERR("crc_inframe=0x%x crc=0x%x\n", crc_inframe, crc);
		research_flag(stream);
		SH_ERR("error = %d\n", stream->error_num);
	}
}

static int shub_collect_data(struct shub_data_processor *stream,
			     unsigned char *data, unsigned short len,
			     unsigned short *processed_len)
{
	unsigned short nFrameRemain =
	    stream->data_len - stream->received_data_len + SHUB_MAX_DATA_CRC;
	struct cmd_data *pPacket = &stream->cmd_data;
	unsigned short nCopy = nFrameRemain > len ? len : nFrameRemain;
	unsigned short data_crc;
	unsigned short crc_inframe;

	memcpy(pPacket->buff + stream->received_data_len, data, nCopy);
	stream->received_data_len += nCopy;

	*processed_len = nCopy;
	/*  Have we got the whole frame? */
	if (stream->received_data_len ==
		(stream->data_len + SHUB_MAX_DATA_CRC)) {
		data_crc = shub_data_checksum(pPacket->buff, pPacket->length);
		crc_inframe = pPacket->buff[pPacket->length];
		crc_inframe <<= 8;
		crc_inframe |= pPacket->buff[(pPacket->length + 1)];
		if (data_crc == crc_inframe) {
			shub_dispatch(&stream->cmd_data);
		} else {
			if (((pPacket->subtype == SHUB_MEMDUMP_DATA_SUBTYPE) ||
			     (pPacket->subtype == SHUB_MEMDUMP_CODE_SUBTYPE))) {
				/* gps_dispatch(&stream->cmd_data); */
			} else {
				SH_LOG
					("err type=%d,subtype=%d len=%d\n",
				     pPacket->type, pPacket->subtype,
				     pPacket->length);
				SH_LOG
				    ("err CRC=%d crc_inframe=%d\n",
				     data_crc, crc_inframe);
			}
		}
		stream->state = SHUB_RECV_Complete;
		stream->state = SHUB_RECV_SEARCH_FLAG;
		stream->head_size = 0;
		stream->received_data_len = 0;
		stream->data_len = 0;
/*		SH_LOG("recv one packet complete\n");*/
	}

	return 0;
}


/*Function:  shub_InitOnePacket
* Description :
* the init SHUB parse data
* Parameters :
* stream : point the current  parse  data
* Return :
* TRUE   One  frame completed
* FALSE  One  frame not completed
*  Negetive Error
*/
void shub_init_parse_packet(struct shub_data_processor *stream)
{
	stream->state = SHUB_RECV_SEARCH_FLAG;
	stream->head_size = 0;
	stream->received_data_len = 0;
	stream->data_len = 0;
	stream->error_num = 0;
}

/*Function:  shub_parse_one_packet
* Description :
* Parse the input uart  data
* Parameters :
* stream : point the current  parse  data
* UseData : point the uart buffer data
* len       : the receive data length
* Return :
* TRUE     One  frame completed
* FALSE    One  frame not completed
* Negetive  Error
*/
int shub_parse_one_packet(
	struct shub_data_processor *stream,
	unsigned char *data,
	unsigned short len)
{
	unsigned char *input;
	unsigned short remain_len = 0;
	unsigned short processed_len = 0;
	int nRet = 1;

	remain_len = len;
	input = data;

	if ((stream == NULL) || (data == NULL))
		return -1;

	while (remain_len) {
		switch (stream->state) {
		case SHUB_RECV_SEARCH_FLAG:
			shub_search_flag(stream, input, remain_len,
					 &processed_len);
			break;
		case SHUB_RECV_COLLECT_HEAD:
			shub_collect_header(stream, input, remain_len,
					    &processed_len);
			break;
		case SHUB_RECV_DATA:
			shub_collect_data(stream, input, remain_len,
					  &processed_len);
			break;
		default:
			break;
		}
		remain_len -= processed_len;
		input += processed_len;
	}

	return nRet;
}

void shub_fill_head(struct cmd_data *in_data, unsigned char *out_data)
{
	unsigned char *data = out_data;
	unsigned short crc = 0;

	*data++ = SHUB_MAGIC_NUMBER;
	*data++ = SHUB_MAGIC_NUMBER;
	*data++ = SHUB_MAGIC_NUMBER;
	*data++ = SHUB_MAGIC_NUMBER;
	*data++ = in_data->type;
	*data++ = in_data->subtype;
	*data++ = SHUB_GET_HIGH_BYTE(in_data->length);
	*data++ = SHUB_GET_LOW_BYTE(in_data->length);
	/*calc crc */
	crc = shub_checksum(out_data);
	*data++ = SHUB_GET_HIGH_BYTE(crc);
	*data++ = SHUB_GET_LOW_BYTE(crc);
}

int shub_encode_one_packet(
			struct cmd_data *in_data,
			unsigned char *out_data,
			unsigned short out_len)
{
	int len = 0;
	unsigned char *crc_data = NULL;
	unsigned short data_checksum;

	if (in_data == NULL) {
		SH_LOG("NULL == in_data");
		return -1;
	}

	if (out_len <= SHUB_MAX_HEAD_LEN) {
		SH_LOG("  out_len == %d", out_len);
		return -1;
	}

	len = in_data->length;
	/* First fill the SHUB head context */
	shub_fill_head(in_data, out_data);
	if (len) {
		memcpy((out_data + SHUB_MAX_HEAD_LEN), in_data->buff, len);
		data_checksum = shub_data_checksum(in_data->buff, len);
		crc_data = out_data + SHUB_MAX_HEAD_LEN + len;
		*crc_data++ = SHUB_GET_HIGH_BYTE(data_checksum);
		*crc_data++ = SHUB_GET_LOW_BYTE(data_checksum);
		len += SHUB_MAX_HEAD_LEN;
		len += SHUB_MAX_DATA_CRC;
	}

	/*      SH_LOG("yuebao --exit "); */
	return len;
}
