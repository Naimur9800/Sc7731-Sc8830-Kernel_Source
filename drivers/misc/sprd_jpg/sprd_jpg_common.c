/*
 * Copyright (C) 2012--2015 Spreadtrum Communications Inc.
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
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/platform_device.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include <uapi/video/sprd_jpg.h>
#include "sprd_jpg_common.h"

#if IS_ENABLED(CONFIG_QOS_SECURE_SETTING)
#include <misc/secure_qos_setting.h>
#endif

#if IS_ENABLED(CONFIG_SPRD_VSP_PW_DOMAIN_R7P0)
#include <uapi/video/sprd_vsp_pw_domain.h>

int sprd_jpg_pw_on(void)
{
	int ret = 0;

	ret = vsp_pw_on(VSP_PW_DOMAIN_VSP_JPG);
	return ret;
}

int sprd_jpg_pw_off(void)
{
	int ret = 0;

	ret = vsp_pw_off(VSP_PW_DOMAIN_VSP_JPG);
	return ret;
}


#elif (IS_ENABLED(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0) \
	|| IS_ENABLED(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0V2) \
	|| IS_ENABLED(CONFIG_SPRD_CAM_PW_DOMAIN_R4P0) \
	|| IS_ENABLED(CONFIG_SPRD_CAM_PW_DOMAIN_R5P0))
#include "cam_pw_domain.h"

int sprd_jpg_pw_on(void)
{
	int ret = 0;

	ret = sprd_cam_pw_on();
	return ret;
}

int sprd_jpg_pw_off(void)
{
	int ret = 0;

	ret = sprd_cam_pw_off();
	return ret;
}

#else
int sprd_jpg_pw_on(void)
{
	return 0;
}

int sprd_jpg_pw_off(void)
{
	return 0;
}
#endif

struct clk *jpg_get_clk_src_name(struct clock_name_map_t clock_name_map[],
				unsigned int freq_level,
				unsigned int max_freq_level)
{
	if (freq_level >= max_freq_level) {
		pr_info("set freq_level to 0\n");
		freq_level = 0;
	}

	pr_debug(" freq_level %d %s\n", freq_level,
		 clock_name_map[freq_level].name);
	return clock_name_map[freq_level].clk_parent;
}

int find_jpg_freq_level(struct clock_name_map_t clock_name_map[],
			unsigned long freq,
			unsigned int max_freq_level)
{
	int level = 0;
	int i;

	for (i = 0; i < max_freq_level; i++) {
		if (clock_name_map[i].freq == freq) {
			level = i;
			break;
		}
	}
	return level;
}

int jpg_get_mm_clk(struct jpg_dev_t *jpg_hw_dev)
{
	int ret = 0;
	struct clk *clk_mm_eb;
	struct clk *clk_axi_gate_jpg;
	struct clk *clk_ahb_gate_jpg_eb;
	struct clk *clk_vsp_mq_ahb_eb;
	struct clk *clk_jpg;
	struct clk *clk_parent;
	struct clk *clk_aon_jpg_emc_eb;
	struct clk *clk_aon_jpg_eb;
	struct clk *clk_ahb_vsp;
	struct clk *clk_emc_vsp;

	clk_mm_eb = devm_clk_get(jpg_hw_dev->jpg_dev, "clk_mm_eb");

	if (IS_ERR_OR_NULL(clk_mm_eb)) {
		pr_err("Failed : Can't get clock [%s]!\n", "clk_mm_eb");
		pr_err("clk_mm_eb =  %p\n", clk_mm_eb);
		jpg_hw_dev->clk_mm_eb = NULL;
		ret = -EINVAL;
	} else {
		jpg_hw_dev->clk_mm_eb = clk_mm_eb;
	}

	if (jpg_hw_dev->version == SHARKL3) {
		clk_aon_jpg_emc_eb = devm_clk_get(jpg_hw_dev->jpg_dev,
					"clk_aon_jpg_emc_eb");
		if (IS_ERR_OR_NULL(clk_aon_jpg_emc_eb)) {
			pr_err("Can't get clock [%s]!\n", "clk_aon_jpg_emc_eb");
			pr_err("clk_aon_jpg_emc_eb = %p\n", clk_aon_jpg_emc_eb);
			jpg_hw_dev->clk_aon_jpg_emc_eb = NULL;
		} else {
			jpg_hw_dev->clk_aon_jpg_emc_eb = clk_aon_jpg_emc_eb;
		}

		clk_aon_jpg_eb = devm_clk_get(jpg_hw_dev->jpg_dev,
					"clk_aon_jpg_eb");
		if (IS_ERR_OR_NULL(clk_aon_jpg_eb)) {
			pr_err("Can't get clock [%s]!\n", "clk_aon_jpg_eb");
			pr_err("clk_aon_jpg_eb = %p\n", clk_aon_jpg_eb);
			jpg_hw_dev->clk_aon_jpg_eb = NULL;
		} else {
			jpg_hw_dev->clk_aon_jpg_eb = clk_aon_jpg_eb;
		}
	}

	clk_axi_gate_jpg =
	    devm_clk_get(jpg_hw_dev->jpg_dev, "clk_axi_gate_jpg");
	if (IS_ERR_OR_NULL(clk_axi_gate_jpg)) {
		pr_err("Failed : Can't get clock [%s]!\n", "clk_axi_gate_jpg");
		pr_err("clk_axi_gate_jpg =  %p\n", clk_axi_gate_jpg);
		jpg_hw_dev->clk_axi_gate_jpg = NULL;
	} else {
		jpg_hw_dev->clk_axi_gate_jpg = clk_axi_gate_jpg;
	}

	clk_ahb_gate_jpg_eb =
	    devm_clk_get(jpg_hw_dev->jpg_dev, "clk_ahb_gate_jpg_eb");

	if (IS_ERR_OR_NULL(clk_ahb_gate_jpg_eb)) {
		pr_err("Failed : Can't get clock [%s]!\n",
		       "clk_ahb_gate_jpg_eb");
		pr_err("clk_ahb_gate_jpg_eb =  %p\n", clk_ahb_gate_jpg_eb);
		ret = -EINVAL;
		jpg_hw_dev->clk_ahb_gate_jpg_eb = NULL;
	} else {
		jpg_hw_dev->clk_ahb_gate_jpg_eb = clk_ahb_gate_jpg_eb;
	}

	if (jpg_hw_dev->version == PIKE2) {
		clk_vsp_mq_ahb_eb =
		    devm_clk_get(jpg_hw_dev->jpg_dev, "clk_vsp_mq_ahb_eb");

		if (IS_ERR_OR_NULL(clk_vsp_mq_ahb_eb)) {
			pr_err("Failed: Can't get clock [%s]! %p\n",
			       "clk_vsp_mq_ahb_eb", clk_vsp_mq_ahb_eb);
			jpg_hw_dev->clk_vsp_mq_ahb_eb = NULL;
			ret = -EINVAL;
		} else
			jpg_hw_dev->clk_vsp_mq_ahb_eb = clk_vsp_mq_ahb_eb;
	}

	if (jpg_hw_dev->version == SHARKL3) {
		clk_ahb_vsp =
		    devm_clk_get(jpg_hw_dev->jpg_dev, "clk_ahb_vsp");

		if (IS_ERR_OR_NULL(clk_ahb_vsp)) {
			pr_err("Failed: Can't get clock [%s]! %p\n",
			       "clk_ahb_vsp", clk_ahb_vsp);
			ret = -EINVAL;
		} else
			jpg_hw_dev->clk_ahb_vsp = clk_ahb_vsp;

		clk_parent = devm_clk_get(jpg_hw_dev->jpg_dev,
				"clk_ahb_vsp_parent");
		if (IS_ERR_OR_NULL(clk_parent)) {
			pr_err("clock[%s]: failed to get parent in probe!\n",
				 "clk_ahb_vsp_parent");
			ret = -EINVAL;
		} else
			jpg_hw_dev->ahb_parent_clk = clk_parent;

		clk_emc_vsp =
			devm_clk_get(jpg_hw_dev->jpg_dev, "clk_emc_vsp");

		if (IS_ERR_OR_NULL(clk_emc_vsp)) {
			pr_err("Failed: Can't get clock [%s]! %p\n",
				   "clk_emc_vsp", clk_emc_vsp);
			ret = -EINVAL;
		} else
			jpg_hw_dev->clk_emc_vsp = clk_emc_vsp;

		clk_parent = devm_clk_get(jpg_hw_dev->jpg_dev,
				"clk_emc_vsp_parent");
		if (IS_ERR_OR_NULL(clk_parent)) {
			pr_err("clock[%s]: failed to get parent in probe!\n",
				 "clk_emc_vsp_parent");
			ret = -EINVAL;
		} else
			jpg_hw_dev->emc_parent_clk = clk_parent;
	}

	clk_jpg = devm_clk_get(jpg_hw_dev->jpg_dev, "clk_jpg");

	if (IS_ERR_OR_NULL(clk_jpg)) {
		pr_err("Failed : Can't get clock [%s}!\n", "clk_jpg");
		pr_err("jpg_clk =  %p\n", clk_jpg);
		ret = -EINVAL;
		jpg_hw_dev->jpg_clk = NULL;
	} else {
		jpg_hw_dev->jpg_clk = clk_jpg;
	}

	clk_parent = jpg_get_clk_src_name(jpg_hw_dev->clock_name_map, 0,
			jpg_hw_dev->max_freq_level);
	jpg_hw_dev->jpg_parent_clk_df = clk_parent;

	return ret;
}

#ifdef CONFIG_COMPAT
int compat_get_mmu_map_data(struct compat_jpg_iommu_map_data __user *
				   data32,
				   struct jpg_iommu_map_data __user *data)
{
	compat_int_t i;
	compat_size_t s;
	compat_ulong_t ul;
	int err;

	err = get_user(i, &data32->fd);
	err |= put_user(i, &data->fd);
	err |= get_user(s, &data32->size);
	err |= put_user(s, &data->size);
	err |= get_user(ul, &data32->iova_addr);
	err |= put_user(ul, &data->iova_addr);

	return err;
};

int compat_put_mmu_map_data(struct compat_jpg_iommu_map_data __user *
				   data32,
				   struct jpg_iommu_map_data __user *data)
{
	compat_int_t i;
	compat_size_t s;
	compat_ulong_t ul;
	int err;

	err = get_user(i, &data->fd);
	err |= put_user(i, &data32->fd);
	err |= get_user(s, &data->size);
	err |= put_user(s, &data32->size);
	err |= get_user(ul, &data->iova_addr);
	err |= put_user(ul, &data32->iova_addr);

	return err;
};

long compat_jpg_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	long ret = 0;
	struct jpg_fh *jpg_fp = filp->private_data;

	if (!filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	if (jpg_fp == NULL) {
		pr_err("%s, jpg_ioctl error occurred, jpg_fp == NULL\n",
		       __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case COMPAT_JPG_GET_IOVA:
		{
			struct compat_jpg_iommu_map_data __user *data32;
			struct jpg_iommu_map_data __user *data;
			int err;

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL) {
				pr_err("%s %d, compat_alloc_user_space failed",
				       __func__, __LINE__);
				return -EFAULT;
			}

			err = compat_get_mmu_map_data(data32, data);
			if (err) {
				pr_err("%s %d, compat_get_mmu_map_data failed",
				       __func__, __LINE__);
				return err;
			}
			ret = filp->f_op->unlocked_ioctl(filp, JPG_GET_IOVA,
							 (unsigned long)data);
			err = compat_put_mmu_map_data(data32, data);
			return ret ? ret : err;
		}
	case COMPAT_JPG_FREE_IOVA:
		{
			struct compat_jpg_iommu_map_data __user *data32;
			struct jpg_iommu_map_data __user *data;
			int err;

			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL) {
				pr_err("%s %d, compat_alloc_user_space failed",
				       __func__, __LINE__);
				return -EFAULT;
			}

			err = compat_get_mmu_map_data(data32, data);
			if (err) {
				pr_err("%s %d, compat_get_mmu_map_data failed",
				       __func__, __LINE__);
				return err;
			}
			ret = filp->f_op->unlocked_ioctl(filp, JPG_FREE_IOVA,
							 (unsigned long)data);
			err = compat_put_mmu_map_data(data32, data);
			return ret ? ret : err;
		}
	default:
		return filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)
						  compat_ptr(arg));
	}

	return ret;
}
#endif
int jpg_get_iova(struct jpg_dev_t *jpg_hw_dev,
		 struct jpg_iommu_map_data *mapdata, void __user *arg)
{
	int ret = 0;
	/*struct jpg_iommu_map_data mapdata; */
	struct sprd_iommu_map_data iommu_map_data;

	if (sprd_iommu_attach_device(jpg_hw_dev->jpg_dev) == 0) {
		ret = sprd_ion_get_buffer(mapdata->fd, NULL,
					    &(iommu_map_data.buf),
					    &iommu_map_data.iova_size);
		if (ret) {
			pr_err("get_sg_table failed, ret %d\n", ret);
			return ret;
		}

		iommu_map_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		ret =
		    sprd_iommu_map(jpg_hw_dev->jpg_dev, &iommu_map_data);
		if (!ret) {
			mapdata->iova_addr = iommu_map_data.iova_addr;
			mapdata->size = iommu_map_data.iova_size;
			ret =
			    copy_to_user((void __user *)arg,
					 (void *)mapdata,
					 sizeof(struct jpg_iommu_map_data));
			if (ret) {
				pr_err("copy_to_user failed, ret %d\n", ret);
				return -EFAULT;
			}

		} else {
			pr_err("vsp iommu map failed, ret %d\n", ret);
			pr_err("map size 0x%zx\n", iommu_map_data.iova_size);
		}
	} else {
		ret =
		    sprd_ion_get_phys_addr(mapdata->fd, NULL,
					   &mapdata->iova_addr, &mapdata->size);
		if (ret) {
			pr_err
			    ("jpg sprd_ion_get_phys_addr failed, ret %d\n",
			     ret);
			return ret;
		}

		ret =
		    copy_to_user((void __user *)arg,
				 (void *)mapdata,
				 sizeof(struct jpg_iommu_map_data));
		if (ret) {
			pr_err("copy_to_user failed, ret %d\n", ret);
			return -EFAULT;
		}
	}
	return ret;
}

int jpg_free_iova(struct jpg_dev_t *jpg_hw_dev,
		  struct jpg_iommu_map_data *ummapdata)
{

	int ret = 0;
	struct sprd_iommu_unmap_data iommu_ummap_data;

	if (sprd_iommu_attach_device(jpg_hw_dev->jpg_dev) == 0) {
		iommu_ummap_data.iova_addr = ummapdata->iova_addr;
		iommu_ummap_data.iova_size = ummapdata->size;
		iommu_ummap_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		iommu_ummap_data.buf = NULL;
		ret =
		    sprd_iommu_unmap(jpg_hw_dev->jpg_dev,
					  &iommu_ummap_data);

		if (ret) {
			pr_err("jpg iommu unmap failed ret %d\n", ret);
			pr_err("unmap addr&size 0x%lx 0x%zx\n",
			       ummapdata->iova_addr, ummapdata->size);
		}
	}

	return ret;
}

static int poll_mbio_vlc_done(struct jpg_dev_t *jpg_hw_dev, int cmd0)
{
	int ret = 0;

	pr_debug("jpg_poll_begin\n");
	if (cmd0 == INTS_MBIO) {
		/* JPG_ACQUAIRE_MBIO_DONE */
		ret = wait_event_interruptible_timeout(
				jpg_hw_dev->wait_queue_work_MBIO,
				jpg_hw_dev->condition_work_MBIO,
				msecs_to_jiffies(JPG_TIMEOUT_MS));

		if (ret == -ERESTARTSYS) {
			pr_err("jpg error start -ERESTARTSYS\n");
			ret = -EINVAL;
		} else if (ret == 0) {
			pr_err("jpg error start  timeout\n");
			ret = readl_relaxed(
				(void __iomem *)(jpg_hw_dev->sprd_jpg_virt +
				GLB_INT_STS_OFFSET));
			pr_err("jpg_int_status %x", ret);
			ret = -ETIMEDOUT;
		} else {
			ret = 0;
		}

		if (ret) {
			/* timeout, clear jpg int */
			writel_relaxed((1 << 3) | (1 << 2) | (1 << 1) |
				(1 << 0),
				(void __iomem *)(jpg_hw_dev->sprd_jpg_virt +
				GLB_INT_CLR_OFFSET));
			ret = 1;
		} else {
			/* poll successful */
			ret = 0;
		}

		jpg_hw_dev->jpg_int_status &= (~0x8);
		jpg_hw_dev->condition_work_MBIO = 0;
	} else if (cmd0 == INTS_VLC) {
		/* JPG_ACQUAIRE_VLC_DONE */
		ret = wait_event_interruptible_timeout
			    (jpg_hw_dev->wait_queue_work_VLC,
			     jpg_hw_dev->condition_work_VLC,
			     msecs_to_jiffies(JPG_TIMEOUT_MS));

		if (ret == -ERESTARTSYS) {
			pr_err("jpg error start -ERESTARTSYS\n");
			ret = -EINVAL;
		} else if (ret == 0) {
			pr_err("jpg error start  timeout\n");
			ret = readl_relaxed(
				(void __iomem *)(jpg_hw_dev->sprd_jpg_virt +
							   GLB_INT_STS_OFFSET));
			pr_err("jpg_int_status %x", ret);
			ret = -ETIMEDOUT;
		} else {
			ret = 0;
		}

		if (ret) {
			/* timeout, clear jpg int */
			writel_relaxed((1 << 3) | (1 << 2) | (1 << 1) |
				(1 << 0),
				(void __iomem *)(jpg_hw_dev->sprd_jpg_virt +
				GLB_INT_CLR_OFFSET));
			ret = 1;
		} else {
			/* poll successful */
			ret = 4;
		}

		jpg_hw_dev->jpg_int_status &= (~0x2);
		jpg_hw_dev->condition_work_VLC = 0;
	} else {
		pr_err("JPG_ACQUAIRE_MBIO_DONE error arg");
		ret = -1;
	}
	pr_debug("jpg_poll_end\n");
	return ret;
}

/* ONLY PIKE2 */
#if IS_ENABLED(CONFIG_QOS_SECURE_SETTING)
#define JPG_INTERNAL_DDR_QOS_BASE	(0x61043000UL)
static void jpg_get_ddr_qos_all(void)
{
	ulong val;

	val = qos_register_read(JPG_INTERNAL_DDR_QOS_BASE + 0x130);
	pr_info("%s read jpg 0x130:0x%lx\n", __func__, val);
	val = qos_register_read(JPG_INTERNAL_DDR_QOS_BASE + 0x134);
	pr_info("%s read jpg 0x134:0x%lx\n", __func__, val);
	val = qos_register_read(JPG_INTERNAL_DDR_QOS_BASE + 0x138);
	pr_info("%s read jpg 0x138:0x%lx\n", __func__, val);
	val = qos_register_read(JPG_INTERNAL_DDR_QOS_BASE + 0x10c);
	pr_info("%s read jpg 0x10c:0x%lx\n", __func__, val);
}

static void jpg_set_ddr_qos(u32 addr, u32 mask, u32 bit, u32 val)
{
	ulong tmp;

	tmp = qos_register_read(addr);
	tmp = (tmp & (~(mask << bit))) | (val << bit);
	qos_register_write(addr, tmp);
}

static void jpg_set_ddr_qos_all(void)
{
	/* WR latency */
	jpg_set_ddr_qos(JPG_INTERNAL_DDR_QOS_BASE + 0x130, 0xfff, 0, 70);
	/* WR step */
	jpg_set_ddr_qos(JPG_INTERNAL_DDR_QOS_BASE + 0x134, 0x7, 0, 5);
	/* WR min_qos */
	jpg_set_ddr_qos(JPG_INTERNAL_DDR_QOS_BASE + 0x138, 0xf, 0, 4);
	/* WR max_qos */
	jpg_set_ddr_qos(JPG_INTERNAL_DDR_QOS_BASE + 0x138, 0xf, 8, 12);
	/* WR addr_latency */
	jpg_set_ddr_qos(JPG_INTERNAL_DDR_QOS_BASE + 0x10c, 0x1, 16, 0);
	/* WR enable */
	jpg_set_ddr_qos(JPG_INTERNAL_DDR_QOS_BASE + 0x10c, 0x1, 3, 1);

	/* RD latency */
	jpg_set_ddr_qos(JPG_INTERNAL_DDR_QOS_BASE + 0x130, 0xfff, 16, 170);
	/* RD step */
	jpg_set_ddr_qos(JPG_INTERNAL_DDR_QOS_BASE + 0x134, 0x7, 8, 5);
	/* RD min_qos */
	jpg_set_ddr_qos(JPG_INTERNAL_DDR_QOS_BASE + 0x138, 0xf, 16, 4);
	/* RD max_qos */
	jpg_set_ddr_qos(JPG_INTERNAL_DDR_QOS_BASE + 0x138, 0xf, 24, 12);
	/* RD addr_latency */
	jpg_set_ddr_qos(JPG_INTERNAL_DDR_QOS_BASE + 0x10c, 0x1, 20, 0);
	/* RD enable */
	jpg_set_ddr_qos(JPG_INTERNAL_DDR_QOS_BASE + 0x10c, 0x1, 4, 1);
}
#endif

int jpg_clk_enable(struct jpg_dev_t *jpg_hw_dev)
{
	int ret = 0;

	pr_info("jpg JPG_ENABLE\n");
	if (jpg_hw_dev->version != IWHALE2) {
		ret = clk_prepare_enable(jpg_hw_dev->clk_mm_eb);
		if (ret) {
			pr_err("jpg clk_mm_eb clk_prepare_enable");
			pr_err(" failed!\n");
			return ret;
		}
		pr_debug("jpg clk_mm_eb clk_prepare_enable ok.\n");
	}

	if (jpg_hw_dev->clk_aon_jpg_emc_eb) {
		ret = clk_prepare_enable(jpg_hw_dev->clk_aon_jpg_emc_eb);
		if (ret) {
			pr_err("clk_aon_jpg_emc_eb clk_enable failed!\n");
			return ret;
		}
		pr_debug("clk_aon_jpg_emc_eb clk_prepare_enable ok.\n");
	}

	if (jpg_hw_dev->clk_aon_jpg_eb) {
		ret = clk_prepare_enable(jpg_hw_dev->clk_aon_jpg_eb);
		if (ret) {
			pr_err("clk_aon_jpg_eb clk_enable failed!\n");
			return ret;
		}
		pr_debug("clk_aon_jpg_eb clk_prepare_enable ok.\n");
	}

	ret = clk_prepare_enable(jpg_hw_dev->clk_axi_gate_jpg);
	if (ret) {
		pr_err("clk_axi_gate_jpg clk_prepare_enable failed!\n");
		return ret;
	}
	pr_debug("clk_axi_gate_jpg clk_prepare_enable ok.\n");

	ret = clk_prepare_enable(jpg_hw_dev->clk_ahb_gate_jpg_eb);
	if (ret) {
		pr_err("clk_ahb_gate_jpg_eb prepare_enable failed!\n");
		return ret;
	}
	pr_debug("clk_ahb_gate_jpg_eb clk_prepare_enable ok.\n");

	if (jpg_hw_dev->clk_vsp_mq_ahb_eb) {
		ret = clk_prepare_enable(jpg_hw_dev->clk_vsp_mq_ahb_eb);
		if (ret) {
			pr_err("jpg clk_vsp_mq_ahb_eb clk_prepare_enable");
			pr_err(" failed!\n");
			return ret;
		}
		pr_debug("jpg clk_vsp_mq_ahb_eb clk_prepare_enable ok.\n");
	}

	ret = clk_set_parent(jpg_hw_dev->jpg_clk,
			jpg_hw_dev->jpg_parent_clk_df);
	if (ret) {
		pr_err("clock[%s]: clk_set_parent() failed!",
			"clk_jpg");
		return -EINVAL;
	}

	ret = clk_set_parent(jpg_hw_dev->jpg_clk,
			jpg_hw_dev->jpg_parent_clk);
	if (ret) {
		pr_err("clock[%s]: clk_set_parent() failed!",
			"clk_jpg");
		return -EINVAL;
	}

	ret = clk_prepare_enable(jpg_hw_dev->jpg_clk);
	if (ret) {
		pr_err("jpg_clk clk_prepare_enable failed!\n");
		return ret;
	}

	if (jpg_hw_dev->version == SHARKL3) {
		ret = clk_set_parent(jpg_hw_dev->clk_ahb_vsp,
				   jpg_hw_dev->ahb_parent_clk);
		if (ret) {
			pr_err("clock[%s]: clk_set_parent() failed!",
			       "ahb_parent_clk");
			return -EINVAL;
		}

		ret = clk_prepare_enable(jpg_hw_dev->clk_ahb_vsp);
		if (ret) {
			pr_err("clk_ahb_vsp: clk_prepare_enable failed!\n");
			return ret;
		}
		pr_debug("clk_ahb_vsp: clk_prepare_enable ok.\n");

		ret = clk_set_parent(jpg_hw_dev->clk_emc_vsp,
				   jpg_hw_dev->emc_parent_clk);
		if (ret) {
			pr_err("clock[%s]: clk_set_parent() failed!",
				   "emc_parent_clk");
			return -EINVAL;
		}

		ret = clk_prepare_enable(jpg_hw_dev->clk_emc_vsp);
		if (ret) {
			pr_err("clk_emc_vsp: clk_prepare_enable failed!\n");
			return ret;
		}
		pr_debug("clk_emc_vsp: clk_prepare_enable ok.\n");
	}

	pr_info("jpg_clk clk_prepare_enable ok.\n");
	return ret;
}


int jpg_clk_disable(struct jpg_dev_t *jpg_hw_dev)
{
	int ret = 0;

	if (jpg_hw_dev->version == SHARKL3) {
		if (jpg_hw_dev->clk_ahb_vsp)
			clk_disable_unprepare(jpg_hw_dev->clk_ahb_vsp);
		if (jpg_hw_dev->clk_emc_vsp)
			clk_disable_unprepare(jpg_hw_dev->clk_emc_vsp);
	}
	clk_disable_unprepare(jpg_hw_dev->jpg_clk);
	clk_disable_unprepare(jpg_hw_dev->clk_ahb_gate_jpg_eb);
	if (jpg_hw_dev->clk_vsp_mq_ahb_eb)
		clk_disable_unprepare(jpg_hw_dev->clk_vsp_mq_ahb_eb);
	clk_disable_unprepare(jpg_hw_dev->clk_axi_gate_jpg);
	if (jpg_hw_dev->clk_aon_jpg_eb)
		clk_disable_unprepare(jpg_hw_dev->clk_aon_jpg_eb);
	if (jpg_hw_dev->clk_aon_jpg_emc_eb)
		clk_disable_unprepare(jpg_hw_dev->clk_aon_jpg_emc_eb);
	if (jpg_hw_dev->version != IWHALE2)
		clk_disable_unprepare(jpg_hw_dev->clk_mm_eb);

	return ret;
}

long jpg_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int cmd0;
	struct jpg_iommu_map_data mapdata;
	struct jpg_iommu_map_data ummapdata;
	struct clk *clk_parent;
	unsigned long frequency;
	struct jpg_fh *jpg_fp = filp->private_data;
	struct jpg_dev_t *jpg_hw_dev = NULL;

	if (!jpg_fp) {
		pr_err("%s,%d jpg_fp NULL !\n", __func__, __LINE__);
		return -EINVAL;
	}

	jpg_hw_dev = jpg_fp->jpg_hw_dev;
	switch (cmd) {
	case JPG_CONFIG_FREQ:
		get_user(jpg_hw_dev->freq_div, (int __user *)arg);
		clk_parent = jpg_get_clk_src_name(jpg_hw_dev->clock_name_map,
			jpg_hw_dev->freq_div, jpg_hw_dev->max_freq_level);

		jpg_hw_dev->jpg_parent_clk = clk_parent;

		pr_debug("%s,%d clk_set_parent() !\n", __func__, __LINE__);
		pr_info("JPG_CONFIG_FREQ %d\n", jpg_hw_dev->freq_div);
		break;

	case JPG_GET_FREQ:
		frequency = clk_get_rate(jpg_hw_dev->jpg_clk);
		ret = find_jpg_freq_level(jpg_hw_dev->clock_name_map,
			frequency, jpg_hw_dev->max_freq_level);
		put_user(ret, (int __user *)arg);
		pr_info("jpg ioctl JPG_GET_FREQ %d\n", ret);
		break;

	case JPG_ENABLE:
		pr_debug("jpg ioctl JPG_ENABLE\n");
		ret = jpg_clk_enable(jpg_hw_dev);
		if (ret == 0)
			jpg_fp->is_clock_enabled = 1;

		break;
	case JPG_DISABLE:
		if (jpg_fp->is_clock_enabled == 1)
			jpg_clk_disable(jpg_hw_dev);

		jpg_fp->is_clock_enabled = 0;

		pr_info("jpg ioctl JPG_DISABLE\n");
		break;
	case JPG_ACQUAIRE:
		pr_debug("jpg ioctl JPG_ACQUAIRE begin\n");
		ret = down_timeout(&jpg_hw_dev->jpg_mutex,
				   msecs_to_jiffies(JPG_TIMEOUT_MS));
		if (ret) {
			pr_err("jpg error timeout\n");
			return ret;
		}

		jpg_fp->is_jpg_acquired = 1;
		jpg_hw_dev->jpg_fp = jpg_fp;
		pr_debug("jpg ioctl JPG_ACQUAIRE end\n");
		break;
	case JPG_RELEASE:
		pr_debug("jpg ioctl JPG_RELEASE\n");
		if (jpg_fp->is_jpg_acquired == 1) {
			jpg_fp->is_jpg_acquired = 0;
			up(&jpg_hw_dev->jpg_mutex);
		}
		break;

	case JPG_START:
		pr_debug("jpg ioctl JPG_START\n");
		ret =
		    wait_event_interruptible_timeout
		    (jpg_hw_dev->wait_queue_work_VLC,
		     jpg_hw_dev->condition_work_VLC,
		     msecs_to_jiffies(JPG_TIMEOUT_MS));
		if (ret == -ERESTARTSYS) {
			pr_err("jpg error start -ERESTARTSYS\n");
			jpg_hw_dev->jpg_int_status |= 1 << 30;
			ret = -EINVAL;
		} else if (ret == 0) {
			pr_err("jpg error start  timeout\n");
			jpg_hw_dev->jpg_int_status |= 1 << 31;
			ret = -ETIMEDOUT;
		} else {
			ret = 0;
		}
		if (ret) {
			/* clear jpg int */
			writel_relaxed((1 << 3) | (1 << 2) | (1 << 1) |
				       (1 << 0),
				(void __iomem *)(jpg_hw_dev->sprd_jpg_virt +
						GLB_INT_CLR_OFFSET));
		}
		put_user(jpg_hw_dev->jpg_int_status, (int __user *)arg);
		jpg_hw_dev->condition_work_MBIO = 0;
		jpg_hw_dev->condition_work_VLC = 0;
		jpg_hw_dev->condition_work_BSM = 0;
		jpg_hw_dev->jpg_int_status = 0;
		pr_debug("jpg ioctl JPG_START end\n");
		break;

	case JPG_RESET:
		pr_debug("jpg ioctl JPG_RESET\n");
		regmap_update_bits(jpg_hw_dev->gpr_jpg_ahb,
			jpg_hw_dev->jpg_softreset_reg_offset,
			 jpg_hw_dev->jpg_reset_mask,
			 jpg_hw_dev->jpg_reset_mask);
		regmap_update_bits(jpg_hw_dev->gpr_jpg_ahb,
			jpg_hw_dev->jpg_softreset_reg_offset,
			 jpg_hw_dev->jpg_reset_mask, 0);

#if IS_ENABLED(CONFIG_QOS_SECURE_SETTING)
		jpg_get_ddr_qos_all();
		jpg_set_ddr_qos_all();
		jpg_get_ddr_qos_all();
#endif
		break;

	case JPG_ACQUAIRE_MBIO_VLC_DONE:
		cmd0 = (int)arg;
		ret = poll_mbio_vlc_done(jpg_hw_dev, cmd0);
		break;

	case JPG_GET_IOMMU_STATUS:
		ret = sprd_iommu_attach_device(jpg_hw_dev->jpg_dev);

		break;

	case JPG_GET_IOVA:

		ret =
		    copy_from_user((void *)&mapdata,
				   (const void __user *)arg,
				   sizeof(struct jpg_iommu_map_data));
		if (ret) {
			pr_err("copy mapdata failed, ret %d\n", ret);
			return -EFAULT;
		}

		ret = jpg_get_iova(jpg_hw_dev, &mapdata, (void __user *)arg);
		pr_info("JPG_GET_IOVA end\n");
		break;

	case JPG_FREE_IOVA:

		ret =
		    copy_from_user((void *)&ummapdata,
				   (const void __user *)arg,
				   sizeof(struct jpg_iommu_map_data));
		if (ret) {
			pr_err("copy ummapdata failed, ret %d\n", ret);
			return -EFAULT;
		}

		ret = jpg_free_iova(jpg_hw_dev, &ummapdata);
		pr_info("JPG_FREE_IOVA end\n");
		break;

	case JPG_VERSION:

		pr_debug("jpg version -enter\n");
		put_user(jpg_hw_dev->version, (int __user *)arg);

		break;
	default:
		return -EINVAL;
	}

	return ret;
}


