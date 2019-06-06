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

#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <video/sprd_isp_altek.h>
#include "compat_isp_drv.h"

struct compat_isp_init_mem_param {
	uint32_t                           fw_buf_size;
	int32_t                            fw_buf_mfd;
	int32_t                            fw_buf_dev_fd;
	compat_ulong_t                     fw_buf_vir_addr;
	/* full mode channel */
	compat_u64                         fw_buf_phy_addr;
	uint32_t                           shading_bin_offset;
	compat_ulong_t                     dram_buf_vir_addr;
	/* full mode channel */
	compat_ulong_t                     dram_buf_phy_addr;
	uint32_t                           irp_bin_offset;
	uint32_t                           cbc_bin_offset;
	uint32_t                           pdaf_supported;
	compat_ulong_t                     high_iso_buf_vir_addr;
	/* full mode channel */
	compat_ulong_t                     high_iso_phy_addr;
	uint32_t                           af_stats_independence;
};

struct compat_isp_statis_buf {
	uint32_t                           buf_size;
	compat_ulong_t                     phy_addr;
	compat_ulong_t                     vir_addr;
};

struct compat_isp_cfg_img_buf {
	uint32_t                         format;
	uint32_t                         img_id;
	uint32_t                         width;
	uint32_t                         height;
	compat_ulong_t                   yaddr;
	compat_ulong_t                   uaddr;
	compat_ulong_t                   vaddr;
	compat_ulong_t                   yaddr_vir;
	compat_ulong_t                   uaddr_vir;
	compat_ulong_t                   vaddr_vir;
};

struct compat_isp_addr {
	compat_ulong_t                   chn0;
	compat_ulong_t                   chn1;
	compat_ulong_t                   chn2;
};

struct compat_isp_cfg_img_param {
	/* img_id : 0-preview, 1-video, 2-still capture 3-statistics */
	uint32_t                    img_id;
	uint32_t                    dram_eb;
	uint32_t                    format;
	uint32_t                    width;
	uint32_t                    height;
	uint32_t                    buf_num;
	struct compat_isp_addr      addr[IMG_BUF_NUM_MAX];
	struct compat_isp_addr      addr_vir[IMG_BUF_NUM_MAX];
	int32_t                     addr_mfd[IMG_BUF_NUM_MAX]; /* iommu fd */
	uint32_t                    line_offset;
};

struct compat_isp_statis_frame {
	uint32_t                         format;
	uint32_t                         evt;
	uint32_t                         buf_size;
	compat_ulong_t                   phy_addr;
	compat_ulong_t                   vir_addr;
	struct sprd_isp_time             time_stamp;
};

struct compat_isp_io_param {
	uint32_t sub_id;
	compat_uptr_t property_param;
	uint32_t reserved;
};
struct compat_isp_capability {
	uint32_t index;
	compat_uptr_t property_param;
};

struct compat_isp_irq_info {
	uint32_t irq_type;
	uint32_t irq_flag;
	uint32_t format;
	uint32_t channel_id;
	uint32_t base_id;
	uint32_t img_id;
	uint32_t irq_id;
	uint32_t sensor_id;
	compat_ulong_t yaddr;
	compat_ulong_t uaddr;
	compat_ulong_t vaddr;
	compat_ulong_t yaddr_vir;
	compat_ulong_t uaddr_vir;
	compat_ulong_t vaddr_vir;
	uint32_t img_y_fd;
	uint32_t img_u_fd;
	uint32_t img_v_fd;
	compat_ulong_t length;
	struct isp_img_size buf_size;
	struct sprd_isp_time time_stamp;
	uint32_t frm_index;
};

struct compat_isp_img_mem {
	uint32_t                    img_fmt;
	uint32_t                    channel_id;
	uint32_t                    base_id;
	uint32_t                    width;
	uint32_t                    height;
	uint32_t                    is_reserved_buf;
	uint32_t                    img_y_fd;
	uint32_t                    img_u_fd;
	uint32_t                    img_v_fd;
	compat_ulong_t              yaddr;
	compat_ulong_t              uaddr;
	compat_ulong_t              vaddr;
	compat_ulong_t              yaddr_vir;
	compat_ulong_t              uaddr_vir;
	compat_ulong_t              vaddr_vir;
};

struct compat_isp_raw_data {
	int32_t                     fd[ISP_RAWBUF_NUM];
	unsigned int                phy_addr[ISP_RAWBUF_NUM];
	compat_u64                  virt_addr[ISP_RAWBUF_NUM];
	uint32_t                    size;
	uint32_t                    width;
	uint32_t                    height;
	uint32_t                    fmt;
	uint32_t                    cnt;
};


#define COMPAT_ISP_IO_LOAD_FW          _IOW(ISP_IO_MAGIC, 0, \
	struct compat_isp_init_mem_param)
#define COMPAT_ISP_IO_IRQ              _IOR(ISP_IO_MAGIC, 1, \
	struct compat_isp_irq_info)
#define COMPAT_ISP_IO_SET_STATIS_BUF   _IOW(ISP_IO_MAGIC, 2, \
	struct compat_isp_statis_buf)
#define COMPAT_ISP_IO_SET_IMG_BUF      _IOW(ISP_IO_MAGIC, 3, \
	struct compat_isp_cfg_img_buf)
#define COMPAT_ISP_IO_SET_IMG_PARAM    _IOW(ISP_IO_MAGIC, 4, \
	struct compat_isp_cfg_img_param)
#define COMPAT_ISP_IO_CAPABILITY       _IOR(ISP_IO_MAGIC, 9, \
	struct compat_isp_capability)
#define COMPAT_ISP_IO_CFG_PARAM        _IOWR(ISP_IO_MAGIC, 10, \
	struct compat_isp_io_param)
#define COMPAT_ISP_IO_GET_STATIS_BUF   _IOR(ISP_IO_MAGIC, 12, \
	struct compat_isp_statis_frame)
#define COMPAT_ISP_IO_SET_RAW10        _IOW(ISP_IO_MAGIC, 15, \
	struct compat_isp_raw_data)
#define COMPAT_ISP_IO_SET_POST_PROC_YUV   _IOW(ISP_IO_MAGIC, 16, \
	struct compat_isp_img_mem)
#define COMPAT_ISP_IO_SET_FETCH_SRC_BUF   _IOW(ISP_IO_MAGIC, 17, \
	struct compat_isp_img_mem)
#define COMPAT_ISP_IO_CFG_CAP_BUF   _IOW(ISP_IO_MAGIC, 21, \
	struct compat_isp_img_mem)

static int compat_get_init_mem_param(
				struct compat_isp_init_mem_param __user *data32,
				struct isp_init_mem_param __user *data)
{
	int            err = 0;
	uint32_t       temp;
	compat_ulong_t val;
	compat_u64  val_64;

	err = get_user(temp, &data32->fw_buf_size);
	err |= put_user(temp, &data->fw_buf_size);
	err |= get_user(val, &data32->fw_buf_mfd);
	err |= put_user(val, &data->fw_buf_mfd);
	err |= get_user(val, &data32->fw_buf_dev_fd);
	err |= put_user(val, &data->fw_buf_dev_fd);
	err |= get_user(val, &data32->fw_buf_vir_addr);
	err |= put_user(val, &data->fw_buf_vir_addr);
	err |= get_user(val_64, &data32->fw_buf_phy_addr);
	err |= put_user(val_64, &data->fw_buf_phy_addr);

	err |= get_user(temp, &data32->shading_bin_offset);
	err |= put_user(temp, &data->shading_bin_offset);
	err |= get_user(temp, &data32->cbc_bin_offset);
	err |= put_user(temp, &data->cbc_bin_offset);
	err |= get_user(val, &data32->dram_buf_vir_addr);
	err |= put_user(val, &data->dram_buf_vir_addr);
	err |= get_user(val, &data32->dram_buf_phy_addr);
	err |= put_user(val, &data->dram_buf_phy_addr);

	err |= get_user(temp, &data32->irp_bin_offset);
	err |= put_user(temp, &data->irp_bin_offset);
	err |= get_user(temp, &data32->cbc_bin_offset);
	err |= put_user(temp, &data->cbc_bin_offset);
	err |= get_user(temp, &data32->pdaf_supported);
	err |= put_user(temp, &data->pdaf_supported);
	err |= get_user(val, &data32->high_iso_buf_vir_addr);
	err |= put_user(val, &data->high_iso_buf_vir_addr);
	err |= get_user(val, &data32->high_iso_phy_addr);
	err |= put_user(val, &data->high_iso_phy_addr);
	err |= get_user(temp, &data32->af_stats_independence);
	err |= put_user(temp, &data->af_stats_independence);

	return err;
}

static int compat_set_statis_buf(struct compat_isp_statis_buf __user *data32,
				 struct isp_statis_buf __user *data)
{
	int            err = 0;
	uint32_t       tmp;
	compat_ulong_t val;

	err = get_user(tmp, &data32->buf_size);
	err |= put_user(tmp, &data->buf_size);
	err |= get_user(val, &data32->phy_addr);
	err |= put_user(val, &data->phy_addr);
	err |= get_user(val, &data32->vir_addr);
	err |= put_user(val, &data->vir_addr);

	return err;
}

static int compat_set_img_buf(struct compat_isp_cfg_img_buf __user *data32,
			      struct isp_cfg_img_buf __user *data)
{
	int                   err = 0;
	uint32_t              tmp;
	compat_ulong_t        parm;

	err = get_user(tmp, &data32->format);
	err |= put_user(tmp, &data->format);
	err |= get_user(tmp, &data32->img_id);
	err |= put_user(tmp, &data->img_id);
	err |= get_user(tmp, &data32->width);
	err |= put_user(tmp, &data->width);
	err |= get_user(tmp, &data32->height);
	err |= put_user(tmp, &data->height);
	err |= get_user(parm, &data32->yaddr);
	err |= put_user(parm, &data->yaddr);
	err |= get_user(parm, &data32->uaddr);
	err |= put_user(parm, &data->uaddr);
	err |= get_user(parm, &data32->vaddr);
	err |= put_user(parm, &data->vaddr);
	err |= get_user(parm, &data32->yaddr_vir);
	err |= put_user(parm, &data->yaddr_vir);
	err |= get_user(parm, &data32->uaddr_vir);
	err |= put_user(parm, &data->uaddr_vir);
	err |= get_user(parm, &data32->vaddr_vir);
	err |= put_user(parm, &data->vaddr_vir);

	return err;
}

static int compat_set_img_param(struct compat_isp_cfg_img_param __user *data32,
				struct isp_cfg_img_param __user *data)
{
	int                             err = 0;
	int                             cnt = 0;
	uint32_t                        tmp;
	int32_t                         tmp_int32;
	compat_ulong_t                  parm;

	err = get_user(tmp, &data32->img_id);
	err |= put_user(tmp, &data->img_id);
	err |= get_user(tmp, &data32->dram_eb);
	err |= put_user(tmp, &data->dram_eb);
	err |= get_user(tmp, &data32->format);
	err |= put_user(tmp, &data->format);
	err |= get_user(tmp, &data32->width);
	err |= put_user(tmp, &data->width);
	err |= get_user(tmp, &data32->height);
	err |= put_user(tmp, &data->height);
	err |= get_user(tmp, &data32->buf_num);
	err |= put_user(tmp, &data->buf_num);

	for (cnt = 0; cnt < IMG_BUF_NUM_MAX; cnt++) {
		err |= get_user(parm, &data32->addr[cnt].chn0);
		err |= put_user(parm, &data->addr[cnt].chn0);
		err |= get_user(parm, &data32->addr[cnt].chn1);
		err |= put_user(parm, &data->addr[cnt].chn1);
		err |= get_user(parm, &data32->addr[cnt].chn2);
		err |= put_user(parm, &data->addr[cnt].chn2);
	}

	for (cnt = 0; cnt < IMG_BUF_NUM_MAX; cnt++) {
		err |= get_user(parm, &data32->addr_vir[cnt].chn0);
		err |= put_user(parm, &data->addr_vir[cnt].chn0);
		err |= get_user(parm, &data32->addr_vir[cnt].chn1);
		err |= put_user(parm, &data->addr_vir[cnt].chn1);
		err |= get_user(parm, &data32->addr_vir[cnt].chn2);
		err |= put_user(parm, &data->addr_vir[cnt].chn2);
	}

	for (cnt = 0; cnt < IMG_BUF_NUM_MAX; cnt++) {
		err |= get_user(tmp_int32, &data32->addr_mfd[cnt]);
		err |= put_user(tmp_int32, &data->addr_mfd[cnt]);
	}

	err |= get_user(tmp, &data32->line_offset);
	err |= put_user(tmp, &data->line_offset);

	return err;
}

static int compat_get_capability_param(
				struct compat_isp_capability __user *data32,
				struct isp_capability __user *data)
{
	int err = 0;
	uint32_t tmp;
	unsigned long parm;

	err = get_user(tmp, &data32->index);
	err |= put_user(tmp, &data->index);
	err |= get_user(parm, &data32->property_param);
	err |= put_user(((void *)parm), &data->property_param);

	return err;
}

static int compat_set_cfg_param(struct compat_isp_io_param __user *data32,
				struct isp_io_param __user *data)
{
	int err = 0;
	uint32_t tmp;
	unsigned long parm;

	err = get_user(tmp, &data32->sub_id);
	err |= put_user(tmp, &data->sub_id);
	err |= get_user(parm, &data32->property_param);
	err |= put_user(((void *)parm), &data->property_param);
	err |= get_user(tmp, &data32->reserved);
	err |= put_user(tmp, &data->reserved);

	return err;
}

static int compat_get_irq_info(struct compat_isp_irq_info __user *data32,
	struct isp_irq_info __user *data)
{
	int                                err = 0;
	uint32_t                           tmp;
	compat_ulong_t                     val;

	err = get_user(tmp, &data32->irq_type);
	err |= put_user(tmp, &data->irq_type);
	err |= get_user(tmp, &data32->irq_flag);
	err |= put_user(tmp, &data->irq_flag);
	err |= get_user(tmp, &data32->format);
	err |= put_user(tmp, &data->format);
	err |= get_user(tmp, &data32->channel_id);
	err |= put_user(tmp, &data->channel_id);
	err |= get_user(tmp, &data32->base_id);
	err |= put_user(tmp, &data->base_id);
	err |= get_user(tmp, &data32->img_id);
	err |= put_user(tmp, &data->img_id);
	err |= get_user(tmp, &data32->irq_id);
	err |= put_user(tmp, &data->irq_id);
	err |= get_user(tmp, &data32->sensor_id);
	err |= put_user(tmp, &data->sensor_id);
	err |= get_user(val, &data32->yaddr);
	err |= put_user(val, &data->yaddr);
	err |= get_user(val, &data32->uaddr);
	err |= put_user(val, &data->uaddr);
	err |= get_user(val, &data32->vaddr);
	err |= put_user(val, &data->vaddr);
	err |= get_user(val, &data32->yaddr_vir);
	err |= put_user(val, &data->yaddr_vir);
	err |= get_user(val, &data32->uaddr_vir);
	err |= put_user(val, &data->uaddr_vir);
	err |= get_user(val, &data32->vaddr_vir);
	err |= put_user(val, &data->vaddr_vir);
	err |= get_user(val, &data32->img_y_fd);
	err |= put_user(val, &data->img_y_fd);
	err |= get_user(val, &data32->img_u_fd);
	err |= put_user(val, &data->img_u_fd);
	err |= get_user(val, &data32->img_v_fd);
	err |= put_user(val, &data->img_v_fd);
	err |= get_user(val, &data32->length);
	err |= put_user(val, &data->length);
	err |= get_user(val, &data32->buf_size.width);
	err |= put_user(val, &data->buf_size.width);
	err |= get_user(val, &data32->buf_size.height);
	err |= put_user(val, &data->buf_size.height);
	err |= get_user(tmp, &data32->time_stamp.sec);
	err |= put_user(tmp, &data->time_stamp.sec);
	err |= get_user(tmp, &data32->time_stamp.usec);
	err |= put_user(tmp, &data->time_stamp.usec);
	err |= get_user(tmp, &data32->frm_index);
	err |= put_user(tmp, &data->frm_index);

	return err;
}

static int compat_put_irq_info(struct compat_isp_irq_info __user *data32,
	struct isp_irq_info __user *data)
{
	int                                err = 0;
	uint32_t                           tmp;
	compat_ulong_t                     val;

	err = get_user(tmp, &data->irq_type);
	err |= put_user(tmp, &data32->irq_type);
	err |= get_user(tmp, &data->irq_flag);
	err |= put_user(tmp, &data32->irq_flag);
	err |= get_user(tmp, &data->format);
	err |= put_user(tmp, &data32->format);
	err |= get_user(tmp, &data->channel_id);
	err |= put_user(tmp, &data32->channel_id);
	err |= get_user(tmp, &data->base_id);
	err |= put_user(tmp, &data32->base_id);
	err |= get_user(tmp, &data->img_id);
	err |= put_user(tmp, &data32->img_id);
	err |= get_user(tmp, &data->irq_id);
	err |= put_user(tmp, &data32->irq_id);
	err |= get_user(tmp, &data->sensor_id);
	err |= put_user(tmp, &data32->sensor_id);
	err |= get_user(val, &data->yaddr);
	err |= put_user(val, &data32->yaddr);
	err |= get_user(val, &data->uaddr);
	err |= put_user(val, &data32->uaddr);
	err |= get_user(val, &data->vaddr);
	err |= put_user(val, &data32->vaddr);
	err |= get_user(val, &data->yaddr_vir);
	err |= put_user(val, &data32->yaddr_vir);
	err |= get_user(val, &data->uaddr_vir);
	err |= put_user(val, &data32->uaddr_vir);
	err |= get_user(val, &data->vaddr_vir);
	err |= put_user(val, &data32->vaddr_vir);
	err |= get_user(tmp, &data->img_y_fd);
	err |= put_user(tmp, &data32->img_y_fd);
	err |= get_user(tmp, &data->img_u_fd);
	err |= put_user(tmp, &data32->img_u_fd);
	err |= get_user(tmp, &data->img_v_fd);
	err |= put_user(tmp, &data32->img_v_fd);
	err |= get_user(val, &data->length);
	err |= put_user(val, &data32->length);
	err |= get_user(val, &data->buf_size.width);
	err |= put_user(val, &data32->buf_size.width);
	err |= get_user(val, &data->buf_size.height);
	err |= put_user(val, &data32->buf_size.height);
	err |= get_user(tmp, &data->time_stamp.sec);
	err |= put_user(tmp, &data32->time_stamp.sec);
	err |= get_user(tmp, &data->time_stamp.usec);
	err |= put_user(tmp, &data32->time_stamp.usec);
	err |= get_user(tmp, &data->frm_index);
	err |= put_user(tmp, &data32->frm_index);

	return err;
}

static int compat_get_statis_buf(struct compat_isp_statis_frame __user *data32,
				 struct isp_statis_frame __user *data)
{
	int                                err = 0;
	uint32_t                           tmp;
	compat_ulong_t                     val;

	err = get_user(tmp, &data32->format);
	err |= put_user(tmp, &data->format);
	err |= get_user(tmp, &data32->evt);
	err |= put_user(tmp, &data->evt);
	err |= get_user(tmp, &data32->buf_size);
	err |= put_user(tmp, &data->buf_size);
	err |= get_user(val, &data32->phy_addr);
	err |= put_user(val, &data->phy_addr);
	err |= get_user(val, &data32->vir_addr);
	err |= put_user(val, &data->vir_addr);
	err |= get_user(tmp, &data32->time_stamp.sec);
	err |= put_user(tmp, &data->time_stamp.sec);
	err |= get_user(tmp, &data32->time_stamp.usec);
	err |= put_user(tmp, &data->time_stamp.usec);

	return err;
}

static int compat_put_statis_buf(struct compat_isp_statis_frame __user *data32,
				 struct isp_statis_frame __user *data)
{
	int                                err = 0;
	uint32_t                           tmp;
	compat_ulong_t                     val;

	err = get_user(tmp, &data->format);
	err |= put_user(tmp, &data32->format);
	err |= get_user(tmp, &data->evt);
	err |= put_user(tmp, &data32->evt);
	err |= get_user(tmp, &data->buf_size);
	err |= put_user(tmp, &data32->buf_size);
	err |= get_user(val, &data->phy_addr);
	err |= put_user(val, &data32->phy_addr);
	err |= get_user(val, &data->vir_addr);
	err |= put_user(val, &data32->vir_addr);
	err |= get_user(tmp, &data->time_stamp.sec);
	err |= put_user(tmp, &data32->time_stamp.sec);
	err |= get_user(tmp, &data->time_stamp.usec);
	err |= put_user(tmp, &data32->time_stamp.usec);

	return err;
}

static int compat_set_isp_img_mem(struct compat_isp_img_mem __user
					*data32,
					struct isp_img_mem __user *data)
{
	int                                err = 0;
	uint32_t                           tmp;
	compat_ulong_t                     val;

	err = get_user(tmp, &data32->img_fmt);
	err |= put_user(tmp, &data->img_fmt);
	err |= get_user(tmp, &data32->channel_id);
	err |= put_user(tmp, &data->channel_id);
	err |= get_user(tmp, &data32->base_id);
	err |= put_user(tmp, &data->base_id);
	err |= get_user(tmp, &data32->width);
	err |= put_user(tmp, &data->width);
	err |= get_user(tmp, &data32->height);
	err |= put_user(tmp, &data->height);
	err |= get_user(tmp, &data32->is_reserved_buf);
	err |= put_user(tmp, &data->is_reserved_buf);
	err |= get_user(tmp, &data32->img_y_fd);
	err |= put_user(tmp, &data->img_y_fd);
	err |= get_user(tmp, &data32->img_u_fd);
	err |= put_user(tmp, &data->img_u_fd);
	err |= get_user(tmp, &data32->img_v_fd);
	err |= put_user(tmp, &data->img_v_fd);
	err |= get_user(val, &data32->yaddr);
	err |= put_user(val, &data->yaddr);
	err |= get_user(val, &data32->uaddr);
	err |= put_user(val, &data->uaddr);
	err |= get_user(val, &data32->vaddr);
	err |= put_user(val, &data->vaddr);
	err |= get_user(val, &data32->yaddr_vir);
	err |= put_user(val, &data->yaddr_vir);
	err |= get_user(val, &data32->uaddr_vir);
	err |= put_user(val, &data->uaddr_vir);
	err |= get_user(val, &data32->vaddr_vir);
	err |= put_user(val, &data->vaddr_vir);

	return err;
}

static int compat_get_isp_raw_data(struct compat_isp_raw_data __user *data32,
	struct isp_raw_data __user *data)
{
	int            err = 0;
	int            cnt = 0;
	uint32_t       temp;
	compat_u64  val_64;

	for (cnt = 0; cnt < ISP_RAWBUF_NUM; cnt++) {
		err |= get_user(temp, &data32->fd[cnt]);
		err |= put_user(temp, &data->fd[cnt]);
	}

	for (cnt = 0; cnt < ISP_RAWBUF_NUM; cnt++) {
		err |= get_user(temp, &data32->phy_addr[cnt]);
		err |= put_user(temp, &data->phy_addr[cnt]);
	}

	for (cnt = 0; cnt < ISP_RAWBUF_NUM; cnt++) {
		err |= get_user(val_64, &data32->virt_addr[cnt]);
		err |= put_user(val_64, &data->virt_addr[cnt]);
	}

	err = get_user(temp, &data32->size);
	err |= put_user(temp, &data->size);
	err |= get_user(temp, &data32->width);
	err |= put_user(temp, &data->width);
	err |= get_user(temp, &data32->height);
	err |= put_user(temp, &data->height);
	err |= get_user(temp, &data32->fmt);
	err |= put_user(temp, &data->fmt);
	err |= get_user(temp, &data32->cnt);
	err |= put_user(temp, &data->cnt);

	return err;
}

long compat_isp_ioctl(struct file *file, unsigned int cmd, unsigned long param)
{
	long ret = 0;
	int err = 0;
	void __user *up = compat_ptr(param);
	struct compat_isp_img_mem __user *data32;
	struct isp_img_mem __user *data;

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_ISP_IO_LOAD_FW:
	{
		struct compat_isp_init_mem_param __user *data32;
		struct isp_init_mem_param __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("compat_isp_ioctl: failed to compat_alloc_user_space.\n");
			return -EFAULT;
		}

		err = compat_get_init_mem_param(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get compat_get_init_mem_param detail.\n");
			return err;
		}

		ret = file->f_op->unlocked_ioctl(file, ISP_IO_LOAD_FW,
						 (unsigned long)data);

		break;
	}
	case COMPAT_ISP_IO_IRQ:
	{
		struct compat_isp_irq_info __user *data32;
		struct isp_irq_info __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("compat_isp_ioctl: failed to compat_alloc_user_space.\n");
			return -EFAULT;
		}

		err = compat_get_irq_info(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get compat_isp_irq_info detail.\n");
			return err;
		}

		ret = file->f_op->unlocked_ioctl(file, ISP_IO_IRQ,
			(unsigned long)data);

		err = compat_put_irq_info(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get compat_isp_irq_info detail.\n");
			return err;
		}
		break;
	}
	case ISP_IO_STREAM_ON:
	case ISP_IO_STREAM_OFF:
	case ISP_IO_SET_INIT_PARAM:
	case ISP_IO_STOP:
	case ISP_IO_GET_TIME:
	case ISP_IO_GET_ISP_ID:
	case ISP_IO_GET_IQ_PARAM:
	case ISP_IO_SET_CAP_MODE:
	case ISP_IO_SET_HISO:
	case ISP_IO_SET_SKIP_NUM:
	case ISP_IO_GET_USER_CNT:
	case ISP_IO_MATCH_DATA_CTRL:
	case ISP_IO_PROC_STILL:
	case ISP_IO_SET_DECI_NUM:
	case ISP_IO_SEL_TUNING_IQ:
		ret = file->f_op->unlocked_ioctl(file, cmd, (unsigned long)up);
		break;
	case COMPAT_ISP_IO_SET_STATIS_BUF:
	{
		struct compat_isp_statis_buf __user *data32;
		struct isp_statis_buf __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("COMPAT_ISP_IO_SET_STATIS_BUF: failed to compat_alloc_user_space.\n");
			return -EFAULT;
		}

		err = compat_set_statis_buf(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get compat_set_statis_buf detail.\n");
			return err;
		}

		ret = file->f_op->unlocked_ioctl(file,
						 ISP_IO_SET_STATIS_BUF,
						 (unsigned long)data);

		break;
	}

	case COMPAT_ISP_IO_SET_IMG_BUF:
	{
		struct compat_isp_cfg_img_buf __user *data32;
		struct isp_cfg_img_buf __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("compat_isp_ioctl: failed to compat_alloc_user_space.\n");
			return -EFAULT;
		}

		err = compat_set_img_buf(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get COMPAT_ISP_IO_SET_IMG_BUF detail.\n");
			return err;
		}

		ret = file->f_op->unlocked_ioctl(file,
						 ISP_IO_SET_IMG_BUF,
						 (unsigned long)data);

		break;
	}

	case COMPAT_ISP_IO_SET_IMG_PARAM:
	{
		struct compat_isp_cfg_img_param __user *data32;
		struct isp_cfg_img_param __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("compat_isp_ioctl: failed to compat_alloc_user_space.\n");
			return -EFAULT;
		}

		err = compat_set_img_param(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get compat_set_img_param detail.\n");
			return err;
		}

		ret = file->f_op->unlocked_ioctl(file,
						 ISP_IO_SET_IMG_PARAM,
						 (unsigned long)data);

		break;
	}
	case COMPAT_ISP_IO_CAPABILITY:
	{
		struct compat_isp_capability __user *data32;
		struct isp_capability __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("compat_isp_ioctl: failed to compat_alloc_user_space.\n");
			return -EFAULT;
		}

		err = compat_get_capability_param(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get compat_get_capability_param detail.\n");
			return err;
		}

		ret = file->f_op->unlocked_ioctl(file,
						 ISP_IO_CAPABILITY,
						 (unsigned long)data);

		break;
	}
	case COMPAT_ISP_IO_CFG_PARAM:
	{
		struct compat_isp_io_param __user *data32;
		struct isp_io_param __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("compat_isp_ioctl: failed to compat_alloc_user_space.\n");
			return -EFAULT;
		}

		err = compat_set_cfg_param(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get isp_io_param detail.\n");
			return err;
		}

		ret = file->f_op->unlocked_ioctl(file,
						 ISP_IO_CFG_PARAM,
						 (unsigned long)data);

		break;
	}

	case COMPAT_ISP_IO_GET_STATIS_BUF:
	{
		struct compat_isp_statis_frame __user *data32;
		struct isp_statis_frame __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("compat_isp_ioctl: failed to compat_alloc_user_space.\n");
			return -EFAULT;
		}

		err = compat_get_statis_buf(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get compat_get_statis_buf detail.\n");
			return err;
		}

		ret = file->f_op->unlocked_ioctl(file,
						 ISP_IO_GET_STATIS_BUF,
						 (unsigned long)data);

		err = compat_put_statis_buf(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get compat_put_statis_buf detail.\n");
			return err;
		}

		break;
	}

	case COMPAT_ISP_IO_SET_POST_PROC_YUV:
		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("compat_isp_ioctl: failed to compat_alloc_user_space.\n");
			return -EFAULT;
		}

		err = compat_set_isp_img_mem(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get isp_io_param detail.\n");
			return err;
		}

		ret = file->f_op->unlocked_ioctl(file, ISP_IO_SET_POST_PROC_YUV,
						(unsigned long)data);
		break;
	case COMPAT_ISP_IO_SET_FETCH_SRC_BUF:
		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("compat_isp_ioctl: failed to compat_alloc_user_space.\n");
			return -EFAULT;
		}

		err = compat_set_isp_img_mem(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get isp_io_param detail.\n");
			return err;
		}

		ret = file->f_op->unlocked_ioctl(file, ISP_IO_SET_FETCH_SRC_BUF,
			(unsigned long)data);

		break;

	case COMPAT_ISP_IO_CFG_CAP_BUF:
		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("compat_isp_ioctl: failed to compat_alloc_user_space.\n");
			return -EFAULT;
		}

		err = compat_set_isp_img_mem(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get isp_io_param detail.\n");
			return err;
		}

		ret = file->f_op->unlocked_ioctl(file, ISP_IO_CFG_CAP_BUF,
						 (unsigned long)data);
		break;

	case COMPAT_ISP_IO_SET_RAW10:
	{

		struct compat_isp_raw_data __user *data32;
		struct isp_raw_data __user *data;

		data32 = compat_ptr(param);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL) {
			pr_err("compat_isp_ioctl: failed to compat_alloc_user_space.\n");
			return -EFAULT;
		}

		err = compat_get_isp_raw_data(data32, data);
		if (err) {
			pr_err("compat_isp_ioctl: failed to get isp_raw_data detail.\n");
			return err;
		}

		ret = file->f_op->unlocked_ioctl(file, ISP_IO_SET_RAW10,
			(unsigned long)data);
		break;
	}

	default:
		pr_err("compat_isp_ioctl, don't support cmd 0x%x\n", cmd);
		break;
	}

	return ret;
}
