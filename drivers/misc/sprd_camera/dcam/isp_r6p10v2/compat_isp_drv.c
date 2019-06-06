/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "compat_isp_drv.h"

#ifdef ISP_COMPAT_SUPPORT
static int compat_get_set_statis_buf(
		struct compat_isp_statis_buf_input __user *data32,
		struct isp_statis_buf_input __user *data)
{
	int                                err = 0;
	uint32_t                           tmp;
	compat_ulong_t                     val;

	err  = get_user(tmp, &data32->buf_size);
	err |= put_user(tmp, &data->buf_size);
	compat_isp_trace("buf_size [0x%x]\n", tmp);

	err |= get_user(tmp, &data32->buf_num);
	err |= put_user(tmp, &data->buf_num);
	compat_isp_trace("buf_num [%d]\n", tmp);

	err |= get_user(val, &data32->phy_addr);
	err |= put_user(val, &data->phy_addr);
	compat_isp_trace("phy_addr [0x%x]\n", val);

	err |= get_user(val, &data32->vir_addr);
	err |= put_user(val, &data->vir_addr);
	compat_isp_trace("vir_addr [0x%x]\n", val);

	err |= get_user(tmp, &data32->kaddr[0]);
	err |= put_user(tmp, &data->kaddr[0]);
	compat_isp_trace("kaddr [%d]\n", tmp);

	err |= get_user(tmp, &data32->kaddr[1]);
	err |= put_user(tmp, &data->kaddr[1]);
	compat_isp_trace("kaddr [%d]\n", tmp);

	err |= get_user(val, &data32->mfd);
	err |= put_user(val, &data->mfd);
	compat_isp_trace("mfd [0x%x]\n", val);

	err |= get_user(val, &data32->dev_fd);
	err |= put_user(val, &data->dev_fd);
	compat_isp_trace("dev_fd [0x%x]\n", val);

	err |= get_user(tmp, &data32->buf_property);
	err |= put_user(tmp, &data->buf_property);
	compat_isp_trace("buf_property [%d]\n", tmp);

	err |= get_user(tmp, &data32->buf_flag);
	err |= put_user(tmp, &data->buf_flag);
	compat_isp_trace("buf_flag [%d]\n", tmp);

	err |= get_user(tmp, &data32->is_statis_buf_reserved);
	err |= put_user(tmp, &data->is_statis_buf_reserved);
	compat_isp_trace("is_statis_buf_reserved [%d]\n", tmp);

	err |= get_user(tmp, &data32->reserved[0]);
	err |= put_user(tmp, &data->reserved[0]);
	compat_isp_trace("reserved[0] [%d]\n", tmp);

	err |= get_user(tmp, &data32->reserved[1]);
	err |= put_user(tmp, &data->reserved[1]);
	compat_isp_trace("reserved[1] [%d]\n", tmp);

	err |= get_user(tmp, &data32->reserved[2]);
	err |= put_user(tmp, &data->reserved[2]);
	compat_isp_trace("reserved[2] [%d]\n", tmp);

	err |= get_user(tmp, &data32->reserved[3]);
	err |= put_user(tmp, &data->reserved[3]);
	compat_isp_trace("reserved[3] [%d]\n", tmp);

	return err;
}

static int compat_put_set_statis_buf(
		struct compat_isp_statis_buf_input __user *data32,
		struct isp_statis_buf_input __user *data)
{
	int                                err = 0;
	uint32_t                           tmp;
	compat_ulong_t                     val;

	err  = get_user(tmp, &data->buf_size);
	err |= put_user(tmp, &data32->buf_size);

	err |= get_user(tmp, &data->buf_num);
	err |= put_user(tmp, &data32->buf_num);
	err |= get_user(val, &data->phy_addr);
	err |= put_user(val, &data32->phy_addr);
	err |= get_user(val, &data->vir_addr);
	err |= put_user(val, &data32->vir_addr);
	err |= get_user(tmp, &data->kaddr[0]);
	err |= put_user(tmp, &data32->kaddr[0]);
	err |= get_user(tmp, &data->kaddr[1]);
	err |= put_user(tmp, &data32->kaddr[1]);
	err |= get_user(val, &data->mfd);
	err |= put_user(val, &data32->mfd);
	err |= get_user(val, &data->dev_fd);
	err |= put_user(val, &data32->dev_fd);
	err |= get_user(tmp, &data->buf_property);
	err |= put_user(tmp, &data32->buf_property);
	err |= get_user(tmp, &data->buf_flag);
	err |= put_user(tmp, &data32->buf_flag);
	err |= get_user(tmp, &data->is_statis_buf_reserved);
	err |= put_user(tmp, &data32->is_statis_buf_reserved);
	err |= get_user(tmp, &data->reserved[0]);
	err |= put_user(tmp, &data32->reserved[0]);
	err |= get_user(tmp, &data->reserved[1]);
	err |= put_user(tmp, &data32->reserved[1]);
	err |= get_user(tmp, &data->reserved[2]);
	err |= put_user(tmp, &data32->reserved[2]);
	err |= get_user(tmp, &data->reserved[3]);
	err |= put_user(tmp, &data32->reserved[3]);

	return err;
}

static int compat_get_raw_proc_info(
		struct compat_isp_raw_proc_info __user *data32,
		struct isp_raw_proc_info __user *data)
{
	int                                err = 0;
	uint32_t                           tmp;
	compat_ulong_t                     val;

	err  = get_user(tmp, &data32->in_size.width);
	err |= put_user(tmp, &data->in_size.width);
	compat_isp_trace("in_size.width [%d]\n", tmp);

	err  = get_user(tmp, &data32->in_size.height);
	err |= put_user(tmp, &data->in_size.height);
	compat_isp_trace("in_size.height [%d]\n", tmp);

	err  = get_user(tmp, &data32->out_size.width);
	err |= put_user(tmp, &data->out_size.width);
	compat_isp_trace("out_size.width [%d]\n", tmp);

	err  = get_user(tmp, &data32->out_size.height);
	err |= put_user(tmp, &data->out_size.height);
	compat_isp_trace("out_size.height [%d]\n", tmp);

	err  = get_user(val, &data32->img_vir.chn0);
	err |= put_user(val, &data->img_vir.chn0);
	compat_isp_trace("img_vir.chn0 [%d]\n", tmp);

	err  = get_user(val, &data32->img_vir.chn1);
	err |= put_user(val, &data->img_vir.chn1);
	compat_isp_trace("img_vir.chn1 [%d]\n", tmp);

	err  = get_user(val, &data32->img_vir.chn2);
	err |= put_user(val, &data->img_vir.chn2);
	compat_isp_trace("img_vir.chn2 [%d]\n", tmp);

	err  = get_user(val, &data32->img_offset.chn0);
	err |= put_user(val, &data->img_offset.chn0);
	compat_isp_trace("img_offset.chn0 [%d]\n", tmp);

	err  = get_user(val, &data32->img_offset.chn1);
	err |= put_user(val, &data->img_offset.chn1);
	compat_isp_trace("img_offset.chn1 [%d]\n", tmp);

	err  = get_user(val, &data32->img_offset.chn2);
	err |= put_user(val, &data->img_offset.chn2);
	compat_isp_trace("img_offset.chn2 [%d]\n", tmp);

	err  = get_user(tmp, &data32->img_fd);
	err |= put_user(tmp, &data->img_fd);
	compat_isp_trace("img_fd [%d]\n", tmp);

	return err;
}

static int compat_put_raw_proc_info(
		struct compat_isp_raw_proc_info __user *data32,
		struct isp_raw_proc_info __user *data)
{
	int                                err = 0;
	uint32_t                           tmp;
	compat_ulong_t                     val;

	err  = get_user(tmp, &data->in_size.width);
	err |= put_user(tmp, &data32->in_size.width);
	err |= get_user(tmp, &data->in_size.height);
	err |= put_user(tmp, &data32->in_size.height);

	err |= get_user(tmp, &data->out_size.width);
	err |= put_user(tmp, &data32->out_size.width);
	err |= get_user(tmp, &data->out_size.height);
	err |= put_user(tmp, &data32->out_size.height);

	err |= get_user(val, &data->img_vir.chn0);
	err |= put_user(val, &data32->img_vir.chn0);
	err |= get_user(val, &data->img_vir.chn1);
	err |= put_user(val, &data32->img_vir.chn1);
	err |= get_user(val, &data->img_vir.chn2);
	err |= put_user(val, &data32->img_vir.chn2);

	err |= get_user(val, &data->img_offset.chn0);
	err |= put_user(val, &data32->img_offset.chn0);
	err |= get_user(val, &data->img_offset.chn1);
	err |= put_user(val, &data32->img_offset.chn1);
	err |= get_user(val, &data->img_offset.chn2);
	err |= put_user(val, &data32->img_offset.chn2);

	err |= get_user(tmp, &data->img_fd);
	err |= put_user(tmp, &data32->img_fd);

	return err;
}

static int compat_get_isp_io_param(
		struct compat_isp_io_param __user *data32,
		struct isp_io_param __user *data)
{
	int                                err = 0;
	uint32_t                           tmp;
	unsigned long					   parm;

	err  = get_user(tmp, &data32->isp_id);
	err |= put_user(tmp, &data->isp_id);
	compat_isp_trace("isp_id [%d]\n", tmp);

	err |= get_user(tmp, &data32->sub_block);
	err |= put_user(tmp, &data->sub_block);
	compat_isp_trace("sub_block [%d]\n", tmp);

	err |= get_user(tmp, &data32->property);
	err |= put_user(tmp, &data->property);
	compat_isp_trace("property [%d]\n", tmp);

	err |= get_user(parm, &data32->property_param);
	err |= put_user(((void *)parm), &data->property_param);
	compat_isp_trace("property_param [0x%lx]\n", parm);

	return err;
}

static int compat_put_isp_io_param(
		struct compat_isp_io_param __user *data32,
		struct isp_io_param __user *data)
{
	int                                err = 0;
	uint32_t                           tmp;
	compat_caddr_t					   parm;

	err  = get_user(tmp, &data->isp_id);
	err |= put_user(tmp, &data32->isp_id);
	compat_isp_trace("isp_id [%d]\n", tmp);

	err |= get_user(tmp, &data->sub_block);
	err |= put_user(tmp, &data32->sub_block);
	compat_isp_trace("sub_block [%d]\n", tmp);

	err |= get_user(tmp, &data->property);
	err |= put_user(tmp, &data32->property);
	compat_isp_trace("property [%d]\n", tmp);

	err |= get_user(parm, (compat_caddr_t *)&data->property_param);
	err |= put_user(parm, &data32->property_param);

	compat_isp_trace("property_param [0x%x]\n", parm);

	return err;
}

static int compat_get_isp_capability(
		struct compat_sprd_isp_capability __user *data32,
		struct sprd_isp_capability __user *data)
{
	int				err = 0;
	uint32_t			tmp;
	unsigned long			parm;

	err  = get_user(tmp, &data32->isp_id);
	err |= put_user(tmp, &data->isp_id);
	compat_isp_trace("isp_id [%d]\n", tmp);

	err |= get_user(tmp, &data32->index);
	err |= put_user(tmp, &data->index);
	compat_isp_trace("index [%d]\n", tmp);

	err |= get_user(parm, &data32->property_param);
	err |= put_user(((void *)parm), &data->property_param);
	compat_isp_trace("property_param [0x%lx]\n", parm);

	return err;
}

static int compat_put_isp_capability(
		struct compat_sprd_isp_capability __user *data32,
		struct sprd_isp_capability __user *data)
{
	int				err = 0;
	uint32_t			tmp;
	compat_caddr_t			parm;

	err  = get_user(tmp, &data->isp_id);
	err |= put_user(tmp, &data32->isp_id);
	compat_isp_trace("isp_id [%d]\n", tmp);

	err |= get_user(tmp, &data->index);
	err |= put_user(tmp, &data32->index);
	compat_isp_trace("index [%d]\n", tmp);

	err |= get_user(parm, (compat_caddr_t *)&data->property_param);
	err |= put_user(parm, &data32->property_param);
	compat_isp_trace("property_param [0x%x]\n", parm);

	return err;
}

static int compat_get_isp_dev_block_addr(
		struct compat_isp_dev_block_addr __user *data32,
		struct isp_dev_block_addr __user *data)
{
	int				err = 0;
	compat_ulong_t			val;

	err  = get_user(val, &data32->img_vir.chn0);
	err |= put_user(val, &data->img_vir.chn0);
	compat_isp_trace("img_vir.chn0 [%d]\n", val);

	err |= get_user(val, &data32->img_vir.chn1);
	err |= put_user(val, &data->img_vir.chn1);
	compat_isp_trace("img_vir.chn1 [%d]\n", val);

	err |= get_user(val, &data32->img_vir.chn2);
	err |= put_user(val, &data->img_vir.chn2);
	compat_isp_trace("img_vir.chn2 [%d]\n", val);

	err |= get_user(val, &data32->img_offset.chn0);
	err |= put_user(val, &data->img_offset.chn0);
	compat_isp_trace("img_offset.chn0 [%d]\n", val);

	err |= get_user(val, &data32->img_offset.chn1);
	err |= put_user(val, &data->img_offset.chn1);
	compat_isp_trace("img_offset.chn1 [%d]\n", val);

	err |= get_user(val, &data32->img_offset.chn2);
	err |= put_user(val, &data->img_offset.chn2);
	compat_isp_trace("img_offset.chn2 [%d]\n", val);

	err |= get_user(val, &data32->img_fd);
	err |= put_user(val, &data->img_fd);
	compat_isp_trace("img_fd [%d]\n", val);

	return err;
}

static int compat_put_isp_dev_block_addr(
		struct compat_isp_dev_block_addr __user *data32,
		struct isp_dev_block_addr __user *data)
{
	int				err = 0;
	compat_ulong_t			val;

	err  = get_user(val, &data->img_vir.chn0);
	err |= put_user(val, &data32->img_vir.chn0);
	compat_isp_trace("img_vir.chn0 [%d]\n", val);

	err |= get_user(val, &data->img_vir.chn1);
	err |= put_user(val, &data32->img_vir.chn1);
	compat_isp_trace("img_vir.chn1 [%d]\n", val);

	err |= get_user(val, &data->img_vir.chn2);
	err |= put_user(val, &data32->img_vir.chn2);
	compat_isp_trace("img_vir.chn2 [%d]\n", val);

	err |= get_user(val, &data->img_offset.chn0);
	err |= put_user(val, &data32->img_offset.chn0);
	compat_isp_trace("img_offset.chn0 [%d]\n", val);

	err |= get_user(val, &data->img_offset.chn1);
	err |= put_user(val, &data32->img_offset.chn1);
	compat_isp_trace("img_offset.chn1 [%d]\n", val);

	err |= get_user(val, &data->img_offset.chn2);
	err |= put_user(val, &data32->img_offset.chn2);
	compat_isp_trace("img_offset.chn2 [%d]\n", val);

	err |= get_user(val, &data->img_fd);
	err |= put_user(val, &data32->img_fd);
	compat_isp_trace("img_fd [%d]\n", val);

	return err;
}

static int compat_get_isp_dev_fetch_info_v1(
		struct compat_isp_dev_fetch_info_v1 __user *data32,
		struct isp_dev_fetch_info __user *data)
{
	int				err = 0;
	uint32_t			tmp;
	compat_ulong_t			val;

	err  = get_user(tmp, &data32->bypass);
	err |= put_user(tmp, &data->bypass);
	compat_isp_trace_high("bypass [%d]\n", tmp);

	err |= get_user(tmp, &data32->subtract);
	err |= put_user(tmp, &data->subtract);
	compat_isp_trace_high("subtract [%d]\n", tmp);

	err |= get_user(tmp, &data32->color_format);
	err |= put_user(tmp, &data->color_format);
	compat_isp_trace_high("color_format [%d]\n", tmp);

	err |= get_user(tmp, &data32->start_isp);
	err |= put_user(tmp, &data->start_isp);
	compat_isp_trace_high("start_isp [%d]\n", tmp);

	err |= get_user(tmp, &data32->size.width);
	err |= put_user(tmp, &data->size.width);
	compat_isp_trace_high("size.width [%d]\n", tmp);

	err |= get_user(tmp, &data32->size.height);
	err |= put_user(tmp, &data->size.height);
	compat_isp_trace_high("size.height [%d]\n", tmp);

	err |= get_user(tmp, &data32->addr.chn0);
	err |= put_user(tmp, &data->addr.chn0);
	compat_isp_trace_high("addr.chn0 [%d]\n", tmp);

	err |= get_user(tmp, &data32->addr.chn1);
	err |= put_user(tmp, &data->addr.chn1);
	compat_isp_trace_high("addr.chn1 [%d]\n", tmp);

	err |= get_user(tmp, &data32->addr.chn2);
	err |= put_user(tmp, &data->addr.chn2);
	compat_isp_trace_high("addr.chn2 [%d]\n", tmp);

	err |= get_user(tmp, &data32->pitch.chn0);
	err |= put_user(tmp, &data->pitch.chn0);
	compat_isp_trace_high("pitch.chn0 [%d]\n", tmp);

	err |= get_user(tmp, &data32->pitch.chn1);
	err |= put_user(tmp, &data->pitch.chn1);
	compat_isp_trace_high("pitch.chn1 [%d]\n", tmp);

	err |= get_user(tmp, &data32->pitch.chn2);
	err |= put_user(tmp, &data->pitch.chn2);
	compat_isp_trace_high("pitch.chn2 [%d]\n", tmp);

	err |= get_user(tmp, &data32->mipi_word_num);
	err |= put_user(tmp, &data->mipi_word_num);
	compat_isp_trace_high("mipi_word_num [%d]\n", tmp);

	err |= get_user(tmp, &data32->mipi_byte_rel_pos);
	err |= put_user(tmp, &data->mipi_byte_rel_pos);
	compat_isp_trace_high("mipi_byte_rel_pos [%d]\n", tmp);

	err |= get_user(tmp, &data32->no_line_dly_ctrl);
	err |= put_user(tmp, &data->no_line_dly_ctrl);
	compat_isp_trace_high("no_line_dly_ctrl [%d]\n", tmp);

	err |= get_user(tmp, &data32->req_cnt_num);
	err |= put_user(tmp, &data->req_cnt_num);
	compat_isp_trace_high("req_cnt_num [%d]\n", tmp);

	err |= get_user(tmp, &data32->line_dly_num);
	err |= put_user(tmp, &data->line_dly_num);
	compat_isp_trace_high("pitch.chn2 [%d]\n", tmp);

	err |= get_user(val, &data32->fetch_addr.img_vir.chn0);
	err |= put_user(val, &data->fetch_addr.img_vir.chn0);
	compat_isp_trace_high("img_vir.chn0 [0x%x]\n", val);

	err |= get_user(val, &data32->fetch_addr.img_vir.chn1);
	err |= put_user(val, &data->fetch_addr.img_vir.chn1);
	compat_isp_trace_high("img_vir.chn1 [0x%x]\n", val);

	err |= get_user(val, &data32->fetch_addr.img_vir.chn2);
	err |= put_user(val, &data->fetch_addr.img_vir.chn2);
	compat_isp_trace_high("img_vir.chn2 [0x%x}\n", val);

	err |= get_user(val, &data32->fetch_addr.img_offset.chn0);
	err |= put_user(val, &data->fetch_addr.img_offset.chn0);
	compat_isp_trace_high("img_offset.chn0 [0x%x]\n", val);

	err |= get_user(val, &data32->fetch_addr.img_offset.chn1);
	err |= put_user(val, &data->fetch_addr.img_offset.chn1);
	compat_isp_trace_high("img_offset.chn1 [0x%x]\n", val);

	err |= get_user(val, &data32->fetch_addr.img_offset.chn2);
	err |= put_user(val, &data->fetch_addr.img_offset.chn2);
	compat_isp_trace_high("img_offset.chn2 [0x%x]\n", val);

	err |= get_user(val, &data32->fetch_addr.img_fd);
	err |= put_user(val, &data->fetch_addr.img_fd);
	compat_isp_trace_high("img_fd [%d]\n", val);

	return err;
}

static int compat_put_isp_dev_fetch_info_v1(
		struct compat_isp_dev_fetch_info_v1 __user *data32,
		struct isp_dev_fetch_info __user *data)
{
	int				err = 0;
	uint32_t			tmp;
	compat_ulong_t			val;

	err  = get_user(tmp, &data->bypass);
	err |= put_user(tmp, &data32->bypass);
	compat_isp_trace_high("bypass [%d]\n", tmp);

	err |= get_user(tmp, &data->subtract);
	err |= put_user(tmp, &data32->subtract);
	compat_isp_trace_high("subtract [%d]\n", tmp);

	err |= get_user(tmp, &data->color_format);
	err |= put_user(tmp, &data32->color_format);
	compat_isp_trace_high("color_format [%d]\n", tmp);

	err |= get_user(tmp, &data->start_isp);
	err |= put_user(tmp, &data32->start_isp);
	compat_isp_trace_high("start_isp [%d]\n", tmp);

	err |= get_user(tmp, &data->size.width);
	err |= put_user(tmp, &data32->size.width);
	compat_isp_trace_high("size.width [%d]\n", tmp);

	err |= get_user(tmp, &data->size.height);
	err |= put_user(tmp, &data32->size.height);
	compat_isp_trace_high("size.height [%d]\n", tmp);

	err |= get_user(tmp, &data->addr.chn0);
	err |= put_user(tmp, &data32->addr.chn0);
	compat_isp_trace_high("addr.chn0 [%d]\n", tmp);

	err |= get_user(tmp, &data->addr.chn1);
	err |= put_user(tmp, &data32->addr.chn1);
	compat_isp_trace_high("addr.chn1 [%d]\n", tmp);

	err |= get_user(tmp, &data->addr.chn2);
	err |= put_user(tmp, &data32->addr.chn2);
	compat_isp_trace_high("addr.chn2 [%d]\n", tmp);

	err |= get_user(tmp, &data->pitch.chn0);
	err |= put_user(tmp, &data32->pitch.chn0);
	compat_isp_trace_high("pitch.chn0 [%d]\n", tmp);

	err |= get_user(tmp, &data->pitch.chn1);
	err |= put_user(tmp, &data32->pitch.chn1);
	compat_isp_trace_high("pitch.chn1 [%d]\n", tmp);

	err |= get_user(tmp, &data->pitch.chn2);
	err |= put_user(tmp, &data32->pitch.chn2);
	compat_isp_trace_high("pitch.chn2 [%d]\n", tmp);

	err |= get_user(tmp, &data->mipi_word_num);
	err |= put_user(tmp, &data32->mipi_word_num);
	compat_isp_trace_high("mipi_word_num [%d]\n", tmp);

	err |= get_user(tmp, &data->mipi_byte_rel_pos);
	err |= put_user(tmp, &data32->mipi_byte_rel_pos);
	compat_isp_trace_high("mipi_byte_rel_pos [%d]\n", tmp);

	err |= get_user(tmp, &data->no_line_dly_ctrl);
	err |= put_user(tmp, &data32->no_line_dly_ctrl);
	compat_isp_trace_high("no_line_dly_ctrl [%d]\n", tmp);

	err |= get_user(tmp, &data->req_cnt_num);
	err |= put_user(tmp, &data32->req_cnt_num);
	compat_isp_trace_high("req_cnt_num [%d]\n", tmp);

	err |= get_user(tmp, &data->line_dly_num);
	err |= put_user(tmp, &data32->line_dly_num);
	compat_isp_trace_high("pitch.chn2 [%d]\n", tmp);

	err  = get_user(val, &data->fetch_addr.img_vir.chn0);
	err |= put_user(val, &data32->fetch_addr.img_vir.chn0);
	compat_isp_trace_high("img_vir.chn0 [%d]\n", val);

	err |= get_user(val, &data->fetch_addr.img_vir.chn1);
	err |= put_user(val, &data32->fetch_addr.img_vir.chn1);
	compat_isp_trace_high("img_vir.chn1 [%d]\n", val);

	err |= get_user(val, &data->fetch_addr.img_vir.chn2);
	err |= put_user(val, &data32->fetch_addr.img_vir.chn2);
	compat_isp_trace_high("img_vir.chn2 [%d]\n", val);

	err |= get_user(val, &data->fetch_addr.img_offset.chn0);
	err |= put_user(val, &data32->fetch_addr.img_offset.chn0);
	compat_isp_trace_high("img_offset.chn0 [%d]\n", val);

	err |= get_user(val, &data->fetch_addr.img_offset.chn1);
	err |= put_user(val, &data32->fetch_addr.img_offset.chn1);
	compat_isp_trace_high("img_offset.chn1 [%d]\n", val);

	err |= get_user(val, &data->fetch_addr.img_offset.chn2);
	err |= put_user(val, &data32->fetch_addr.img_offset.chn2);
	compat_isp_trace_high("img_offset.chn2 [%d]\n", val);

	err |= get_user(val, &data->fetch_addr.img_fd);
	err |= put_user(val, &data32->fetch_addr.img_fd);
	compat_isp_trace_high("img_fd [%d]\n", val);

	return err;
}


long compat_sprd_isp_k_ioctl(
		void *isp_pipe_dev_handle,
		unsigned int cmd,
		unsigned long param)
{
	long ret = 0;
	void __user *data32 = compat_ptr(param);

	compat_isp_trace("compat_sprd_isp_k_ioctl cmd [0x%x][%d]\n",
			 cmd, _IOC_NR(cmd));

	switch (cmd) {
	case COMPAT_SPRD_ISP_IO_SET_STATIS_BUF:
	{
		struct compat_isp_statis_buf_input __user *data32;
		struct isp_statis_buf_input __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(
			sizeof(struct isp_statis_buf_input));

		compat_get_set_statis_buf(data32, data);
		sprd_isp_k_ioctl(isp_pipe_dev_handle,
			SPRD_ISP_IO_SET_STATIS_BUF, (unsigned long)data);
		compat_put_set_statis_buf(data32, data);

		break;
	}
	case COMPAT_SPRD_ISP_IO_CAPABILITY:
	{
		struct compat_sprd_isp_capability __user *data32;
		struct sprd_isp_capability __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(
			sizeof(struct sprd_isp_capability));

		compat_get_isp_capability(data32, data);
		sprd_isp_k_ioctl(isp_pipe_dev_handle,
			SPRD_ISP_IO_CAPABILITY, (unsigned long)data);
		compat_put_isp_capability(data32, data);

		break;
	}
	case COMPAT_SPRD_ISP_IO_CFG_PARAM:
	{
		struct compat_isp_io_param __user *data32;
		struct isp_io_param __user *data;
		compat_caddr_t userpoint;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(struct isp_io_param));
		compat_get_isp_io_param(data32, data);

		get_user(userpoint, &data32->property_param);
		compat_isp_trace("ispid = %d, sub_block = %d, property = %d\n",
				 data->isp_id,
				 data->sub_block,
				 data->property);

		switch (data->sub_block) {
		case ISP_BLOCK_2D_LSC:
		{
			struct isp_dev_block_addr __user *addr;

			if (data->property == ISP_PRO_2D_LSC_TRANSADDR) {
				addr = compat_alloc_user_space(
					sizeof(struct isp_dev_block_addr) +
					sizeof(struct isp_io_param));

				compat_get_isp_dev_block_addr(
					(struct compat_isp_dev_block_addr *)
					data->property_param, addr);
				put_user(((void *)addr), &data->property_param);
			}

			sprd_isp_k_ioctl(isp_pipe_dev_handle,
				SPRD_ISP_IO_CFG_PARAM, (unsigned long)data);

			if (data->property == ISP_PRO_2D_LSC_TRANSADDR) {
				put_user(userpoint, &data32->property_param);
				compat_put_isp_dev_block_addr(
					(struct compat_isp_dev_block_addr *)
					data->property_param,
					data->property_param);
				}
			break;
		}
		case ISP_BLOCK_FETCH:
		{
			switch (data->property) {
			case ISP_PRO_FETCH_TRANSADDR:
			{
				struct isp_dev_block_addr __user *addr;

				addr = compat_alloc_user_space(
					sizeof(struct isp_dev_block_addr) +
					sizeof(struct isp_io_param));

				compat_get_isp_dev_block_addr(
					(struct compat_isp_dev_block_addr *)
					data->property_param, addr);
				put_user(((void *)addr), &data->property_param);

				sprd_isp_k_ioctl(isp_pipe_dev_handle,
					SPRD_ISP_IO_CFG_PARAM,
					(unsigned long)data);

				put_user(userpoint, &data32->property_param);
				compat_put_isp_dev_block_addr(
					(struct compat_isp_dev_block_addr *)
					data->property_param,
					data->property_param);
				break;
			}
			case ISP_PRO_FETCH_RAW_BLOCK:
			{
				struct isp_dev_fetch_info __user *addr;

				addr = compat_alloc_user_space(
					sizeof(struct isp_dev_fetch_info) +
					sizeof(struct isp_io_param));

				compat_get_isp_dev_fetch_info_v1(
					(struct compat_isp_dev_fetch_info_v1 *)
					data->property_param, addr);
				put_user(((void *)addr), &data->property_param);
				sprd_isp_k_ioctl(isp_pipe_dev_handle,
						 SPRD_ISP_IO_CFG_PARAM,
						 (unsigned long)data);
				put_user(userpoint, &data32->property_param);
				compat_put_isp_dev_fetch_info_v1(
					(struct compat_isp_dev_fetch_info_v1 *)
					data->property_param,
					data->property_param);
				break;
			}
			default:
				sprd_isp_k_ioctl(isp_pipe_dev_handle,
						 SPRD_ISP_IO_CFG_PARAM,
						 (unsigned long)data);
				break;
			}

			break;
		}
		default:
			sprd_isp_k_ioctl(isp_pipe_dev_handle,
					 SPRD_ISP_IO_CFG_PARAM,
					 (unsigned long)data);
			break;
		}

		compat_put_isp_io_param(data32, data);
		break;
	}
	case COMPAT_SPRD_ISP_IO_RAW_CAP:
	{
		struct compat_isp_raw_proc_info __user *data32;
		struct isp_raw_proc_info __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(
			sizeof(struct isp_raw_proc_info));

		compat_get_raw_proc_info(data32, data);
		sprd_isp_k_ioctl(isp_pipe_dev_handle,
				 SPRD_ISP_IO_RAW_CAP,
				 (unsigned long)data);

		compat_put_raw_proc_info(data32, data);

		break;
	}
	default:
		sprd_isp_k_ioctl(isp_pipe_dev_handle,
			cmd,
			(unsigned long)data32);
		break;
	}

	return ret;
}

#endif /* ISP_COMPAT_SUPPORT */
