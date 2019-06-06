/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#include <video/sprd_cpp.h>

#include "cam_common.h"
#include "cpp_reg.h"
#include "rot_drv.h"
#include "scale_drv.h"
#include "cam_pw_domain.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CPP_CORE: %d: %d: " fmt, current->pid, __LINE__

#define CPP_DEVICE_NAME                "sprd_cpp"
#define ROT_TIMEOUT                    5000
#define SCALE_TIMEOUT                  5000
#define CPP_IRQ_LINE_MASK              CPP_PATH_DONE
#define CPP_MMU_IRQ_LINE_MASK          CPP_MMU_ERROR_INT

enum cpp_irq_id {
	CPP_SCALE_DONE = 0,
	CPP_ROT_DONE,
	CPP_IRQ_NUMBER
};

struct rotif_device {
	atomic_t count;
	struct mutex rot_mutex;
	struct completion done_com;
	struct rot_drv_private drv_priv;
};

struct scif_device {
	atomic_t count;
	struct mutex sc_mutex;
	struct completion done_com;
	struct scale_drv_private drv_priv;
};

typedef void (*cpp_isr_func) (void *);

struct cpp_device {
	atomic_t users;
	spinlock_t hw_lock;
	struct mutex lock;
	spinlock_t slock;

	struct rotif_device *rotif;
	struct scif_device *scif;
	struct miscdevice md;

	int irq;
	cpp_isr_func isr_func[CPP_IRQ_NUMBER];
	void *isr_data[CPP_IRQ_NUMBER];

	void __iomem *io_base;

	struct platform_device *pdev;

	struct clk *cpp_clk;
	struct clk *cpp_clk_parent;
	struct clk *cpp_clk_default;

	struct clk *cpp_eb;
	struct clk *cpp_axi_eb;
	struct clk *clk_mm_eb;

	struct regmap *cam_ahb_gpr;
};

typedef void (*cpp_isr) (struct cpp_device *dev);

static void cpp_scale_done(struct cpp_device *dev)
{
	cpp_isr_func user_func = dev->isr_func[CPP_SCALE_DONE];
	void *priv = dev->isr_data[CPP_SCALE_DONE];

	if (user_func)
		(*user_func) (priv);
}

static void cpp_rot_done(struct cpp_device *dev)
{
	cpp_isr_func user_func = dev->isr_func[CPP_ROT_DONE];
	void *priv = dev->isr_data[CPP_ROT_DONE];

	if (user_func)
		(*user_func) (priv);
}

static const cpp_isr cpp_isr_list[CPP_IRQ_NUMBER] = {
	cpp_scale_done,
	cpp_rot_done,
};

static void cpp_mmu_error(struct cpp_device *dev)
{
	unsigned long addr = 0;

	if (!dev) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	pr_info("CPP MMU INT ERROR:register list\n");
	for (addr = 0x200; addr <= 0x25C ; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			reg_rd(dev, addr),
			reg_rd(dev, addr + 4),
			reg_rd(dev, addr + 8),
			reg_rd(dev, addr + 12));
	}
}

static int mmu_err_pre_proc(struct cpp_device *dev)
{
	if (!dev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	cpp_mmu_error(dev);

	return 0;
}

static irqreturn_t cpp_isr_root(int irq, void *priv)
{
	int i;
	unsigned int path_irq_line, status, mmu_irq_line;
	unsigned long flag;
	struct cpp_device *dev = (struct cpp_device *)priv;

	status = reg_rd(dev, CPP_INT_STS);

	mmu_irq_line = status & CPP_MMU_IRQ_LINE_MASK;
	if (unlikely(mmu_irq_line != 0)) {
		pr_err("cpp mmu error int 0x%x\n", mmu_irq_line);
		if (mmu_err_pre_proc(dev))
			return IRQ_HANDLED;
	}
	path_irq_line = status & CPP_IRQ_LINE_MASK;
	if (unlikely(path_irq_line == 0))
		return IRQ_NONE;

	reg_wr(dev, CPP_INT_CLR, status);

	spin_lock_irqsave(&dev->slock, flag);
	for (i = 0; i < CPP_IRQ_NUMBER; i++) {
		if (path_irq_line & (1 << (unsigned int)i)) {
			if (cpp_isr_list[i])
				cpp_isr_list[i](dev);
		}
		path_irq_line &= ~(unsigned int)(1 << (unsigned int)i);
		if (!path_irq_line)
			break;
	}
	spin_unlock_irqrestore(&dev->slock, flag);

	return IRQ_HANDLED;
}

static int cpp_module_enable(struct cpp_device *dev)
{
	int ret = 0;

	mutex_lock(&dev->lock);

	if (atomic_read(&dev->users) == 1) {
		ret = clk_prepare_enable(dev->cpp_eb);
		if (ret) {
			pr_err("fail to enable cpp\n");
			goto fail;
		}

		ret = clk_prepare_enable(dev->cpp_axi_eb);
		if (ret) {
			pr_err("fail to enable cpp axi\n");
			goto fail;
		}

		ret = clk_set_parent(dev->cpp_clk, dev->cpp_clk_parent);
		if (ret) {
			pr_err("fail to set cpp parent clk\n");
			clk_disable_unprepare(dev->cpp_eb);
			goto fail;
		}

		ret = clk_prepare_enable(dev->cpp_clk);
		if (ret) {
			pr_err("fail to enable cpp clk\n");
			clk_disable_unprepare(dev->cpp_eb);
			goto fail;
		}

		regmap_update_bits(dev->cam_ahb_gpr, MM_AHB_RESET,
				CPP_AHB_RESET_BIT,
				(unsigned int)CPP_AHB_RESET_BIT);
		udelay(2);

		regmap_update_bits(dev->cam_ahb_gpr, MM_AHB_RESET,
				CPP_AHB_RESET_BIT,
				~(unsigned int)CPP_AHB_RESET_BIT);
	}
	mutex_unlock(&dev->lock);

	return ret;

fail:
	mutex_unlock(&dev->lock);

	return ret;
}

static void cpp_module_disable(struct cpp_device *dev)
{
	mutex_lock(&dev->lock);

	clk_set_parent(dev->cpp_clk, dev->cpp_clk_default);
	clk_disable_unprepare(dev->cpp_clk);
	clk_disable_unprepare(dev->cpp_axi_eb);
	clk_disable_unprepare(dev->cpp_eb);

	mutex_unlock(&dev->lock);
}

static void cpp_register_isr(struct cpp_device *dev, enum cpp_irq_id id,
			     cpp_isr_func user_func, void *priv)
{
	unsigned long flag;

	if (id < CPP_IRQ_NUMBER) {
		spin_lock_irqsave(&dev->slock, flag);
		dev->isr_func[id] = user_func;
		dev->isr_data[id] = priv;
		if (user_func) {
			reg_mwr(dev, CPP_INT_MASK, (1 << id), ~(1 << id));
			reg_awr(dev, CPP_INT_MASK, ~CPP_MMU_ERROR_INT);
		} else {
			reg_mwr(dev, CPP_INT_MASK, (1 << id), (1 << id));
			reg_owr(dev, CPP_INT_MASK, CPP_MMU_ERROR_INT);
		}
		spin_unlock_irqrestore(&dev->slock, flag);
	} else {
		pr_err("cpp isr irq error\n");
	}
}

static void rot_isr(void *priv)
{
	struct rotif_device *rotif = (struct rotif_device *)priv;

	if (!rotif)
		return;

	complete(&rotif->done_com);
}

static void scale_isr(void *priv)
{
	struct scif_device *scif = (struct scif_device *)priv;

	if (!scif)
		return;

	complete(&scif->done_com);
}

int cpp_get_sg_table(struct cpp_iommu_info *pfinfo)
{
	int i = 0, ret = 0;

	if (!pfinfo) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}

	for (i = 0; i < 2; i++) {
		if (pfinfo->mfd[i] > 0) {
			ret = sprd_ion_get_buffer(pfinfo->mfd[i],
				NULL,
				&pfinfo->buf[i],
				&pfinfo->size[i]);
			if (ret) {
				pr_err("fail to get sg table\n");
				return -EFAULT;
			}
		}
	}
	return ret;
}

int cpp_get_addr(struct cpp_iommu_info *pfinfo)
{
	int i = 0, ret = 0;
	struct sprd_iommu_map_data iommu_data;

	if (!pfinfo) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}

	for (i = 0; i < 2; i++) {
		if (pfinfo->size[i] <= 0)
			continue;

		if (sprd_iommu_attach_device(pfinfo->dev) == 0) {
			memset(&iommu_data, 0,
				sizeof(struct sprd_iommu_map_data));
			iommu_data.buf = pfinfo->buf[i];
			iommu_data.iova_size = pfinfo->size[i];
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			iommu_data.sg_offset = pfinfo->offset[i];

			ret = sprd_iommu_map(pfinfo->dev, &iommu_data);
			if (ret) {
				pr_err("fail to get iommu kaddr %d\n", i);
				return -EFAULT;
			}
			pfinfo->iova[i] = iommu_data.iova_addr
					+ pfinfo->offset[i];
		} else {
			ret = sprd_ion_get_phys_addr(pfinfo->mfd[i],
					NULL,
					&pfinfo->iova[i],
					&pfinfo->size[i]);
			if (ret) {
				pr_err("fail to get iommu phy addr %d\n", i);
				return -EFAULT;
			}
			pfinfo->iova[i] += pfinfo->offset[i];
		}
	}
	return ret;
}

int cpp_free_addr(struct cpp_iommu_info *pfinfo)
{
	int i, ret;
	struct sprd_iommu_unmap_data iommu_data;

	if (!pfinfo) {
		pr_err("Input ptr is NULL\n");
		return -EINVAL;
	}

	for (i = 0; i < 2; i++) {
		if (pfinfo->size[i] <= 0)
			continue;

		if (sprd_iommu_attach_device(pfinfo->dev) == 0) {
			iommu_data.iova_addr = pfinfo->iova[i]
						- pfinfo->offset[i];
			iommu_data.iova_size = pfinfo->size[i];
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			iommu_data.buf = NULL;

			ret = sprd_iommu_unmap(pfinfo->dev, &iommu_data);
			if (ret) {
				pr_err("failed to free iommu %d\n", i);
				return -EFAULT;
			}
		}
	}

	return 0;
}

static long sprd_cpp_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	int ret = 0;
	unsigned long rtn = 0;
	struct cpp_device *dev = NULL;
	struct rotif_device *rotif = NULL;
	struct scif_device *scif = NULL;
	struct sprd_cpp_rot_cfg_parm rot_parm;
	struct sprd_cpp_scale_cfg_parm sc_parm;
	struct sprd_cpp_size s_sc_cap = {0, 0};
	int cpp_dimension = 0;

	dev = file->private_data;
	if (!dev) {
		pr_err("fail to get cpp dev\n");
		return -EFAULT;
	}

	switch (cmd) {
	case SPRD_CPP_IO_OPEN_ROT:
		break;
	case SPRD_CPP_IO_START_ROT:
		{
			pr_debug("cpp ioctl start rot path rotif\n");
			rotif = dev->rotif;
			if (!rotif) {
				pr_err("rot start: rotif is null!\n");
				return -EINVAL;
			}

			mutex_lock(&rotif->rot_mutex);

			ret = copy_from_user(&rot_parm,
					(void __user *)arg, sizeof(rot_parm));
			if (ret) {
				pr_err("fail to get rot param form user\n");
				mutex_unlock(&rotif->rot_mutex);
				return -EFAULT;
			}

			ret = cpp_rot_check_parm(&rot_parm);
			if (ret) {
				pr_err("fail to check rot parm\n");
				mutex_unlock(&rotif->rot_mutex);
				return -EINVAL;
			}

			rotif->drv_priv.iommu_src.dev = &dev->pdev->dev;
			rotif->drv_priv.iommu_dst.dev = &dev->pdev->dev;

			cpp_register_isr(dev, CPP_ROT_DONE, rot_isr,
				(void *)rotif);

			pr_debug("start to rot y\n");
			ret = cpp_rot_set_y_parm(&rot_parm, &rotif->drv_priv);
			if (ret) {
				pr_err("fail to set rot y parm\n");
				mutex_unlock(&rotif->rot_mutex);
				return -EINVAL;
			}
			cpp_rot_start(&rotif->drv_priv);

			if (cpp_rot_is_end(&rot_parm) == 0) {
				rtn = wait_for_completion_timeout
						(&rotif->done_com,
						msecs_to_jiffies(ROT_TIMEOUT));
				if (rtn == 0) {
					cpp_register_isr(dev, CPP_ROT_DONE,
						NULL, NULL);
					cpp_rot_stop(&rotif->drv_priv);
					mutex_unlock(&rotif->rot_mutex);
					return -EBUSY;
				}

				ret = cpp_rot_set_uv_parm(&rotif->drv_priv);
				if (ret) {
					pr_err("fail to set rot uv parm\n");
					mutex_unlock(&rotif->rot_mutex);
					return -EINVAL;
				}
				cpp_rot_start(&rotif->drv_priv);
			}

			rtn = wait_for_completion_timeout
						(&rotif->done_com,
						msecs_to_jiffies(ROT_TIMEOUT));
			if (rtn == 0) {
				cpp_register_isr(dev, CPP_ROT_DONE,
					NULL, NULL);
				cpp_rot_stop(&rotif->drv_priv);
				mutex_unlock(&rotif->rot_mutex);
				return -EBUSY;
			}
			cpp_register_isr(dev, CPP_ROT_DONE, NULL, NULL);
			cpp_rot_stop(&rotif->drv_priv);
			mutex_unlock(&rotif->rot_mutex);
			pr_debug("cpp ioctl rot over\n");
			break;
		}

	case SPRD_CPP_IO_OPEN_SCALE:
		{
			get_cpp_max_size(&s_sc_cap.w, &s_sc_cap.h);
			cpp_dimension = (s_sc_cap.h << 16) | s_sc_cap.w;
			if (arg)
				ret = copy_to_user((int *)arg, &cpp_dimension,
				sizeof(cpp_dimension));
			if (ret) {
				pr_err("fail to get max size form user");
				return -EFAULT;
			}
			break;
		}
	case SPRD_CPP_IO_START_SCALE:
		{
			pr_debug("cpp ioctl start scale path scale_if\n");
			scif = dev->scif;
			if (!scif) {
				pr_err("scale start:scif is null!\n");
				return -EFAULT;
			}

			mutex_lock(&scif->sc_mutex);

			ret = copy_from_user(&sc_parm,
					(void __user *)arg, sizeof(sc_parm));
			if (ret) {
				pr_err("fail to get scale input param\n");
				mutex_unlock(&scif->sc_mutex);
				return -EFAULT;
			}

			scif->drv_priv.iommu_src.dev = &dev->pdev->dev;
			scif->drv_priv.iommu_dst.dev = &dev->pdev->dev;

			cpp_register_isr(dev, CPP_SCALE_DONE, scale_isr,
				(void *)scif);

			ret = cpp_scale_start(&sc_parm, &scif->drv_priv);
			if (ret) {
				pr_err("fail to start scaler\n");
				mutex_unlock(&scif->sc_mutex);
				return -EFAULT;
			}

			rtn = wait_for_completion_timeout(&scif->done_com,
						msecs_to_jiffies
						(SCALE_TIMEOUT));
			if (rtn == 0) {
				cpp_register_isr(dev, CPP_SCALE_DONE,
					NULL, NULL);
				cpp_scale_stop(&scif->drv_priv);
				mutex_unlock(&scif->sc_mutex);
				pr_err("fail to get scaling done com\n");
				return -EBUSY;
			}
			cpp_register_isr(dev, CPP_SCALE_DONE, NULL, NULL);
			cpp_scale_stop(&scif->drv_priv);
			mutex_unlock(&scif->sc_mutex);
			pr_debug("cpp ioctl scale over\n");
			break;
		}
	case SPRD_CPP_IO_STOP_SCALE:
		break;
	default:
		pr_err("cpp invalid cmd %d\n", cmd);
		break;
	}

	return ret;
}

static int sprd_cpp_open(struct inode *node, struct file *file)
{
	int ret = 0;
	struct cpp_device *dev = NULL;
	struct miscdevice *md = NULL;
	struct rotif_device *rotif = NULL;
	struct scif_device *scif = NULL;
	void *coeff_addr = NULL;

	md = (struct miscdevice *)file->private_data;
	if (!md) {
		pr_err("miscdevice is NULL\n");
		return -EFAULT;
	}

	dev = md->this_device->platform_data;

	file->private_data = (void *)dev;
	if (atomic_inc_return(&dev->users) != 1) {
		pr_info("cpp device node has been opened %d\n",
			atomic_read(&dev->users));
		return ret;
	}

	ret = sprd_cam_pw_on();
	if (ret) {
		pr_err("%s: fail to camera power on\n", __func__);
		goto exit;
	}
	sprd_cam_domain_eb();

	ret = cpp_module_enable(dev);
	if (ret) {
		pr_err("fail to cpp module enable\n");
		goto en_cpp_exit;
	}

	rotif = vzalloc(sizeof(*rotif));
	if (unlikely(!rotif)) {
		ret = -EFAULT;
		pr_err("failed to rotif vzalloc\n");
		goto vzalloc_exit;
	}

	memset(rotif, 0, sizeof(*rotif));

	rotif->drv_priv.io_base = dev->io_base;
	rotif->drv_priv.priv = (void *)rotif;
	dev->rotif = rotif;
	rotif->drv_priv.hw_lock = &dev->hw_lock;
	init_completion(&rotif->done_com);
	mutex_init(&rotif->rot_mutex);

	scif = vzalloc(sizeof(*scif));
	if (unlikely(!scif)) {
		ret = -EFAULT;
		pr_err("fail to scif vzalloc\n");
		goto vzalloc_exit;
	}

	memset(scif, 0, sizeof(*scif));

	coeff_addr = vzalloc(SC_COEFF_BUF_SIZE);
	if (unlikely(!coeff_addr)) {
		ret = -EFAULT;
		pr_err("fail to coeff_addr vzalloc\n");
		goto vzalloc_exit;
	}

	memset(coeff_addr, 0, SC_COEFF_BUF_SIZE);

	scif->drv_priv.io_base = dev->io_base;
	scif->drv_priv.pdev = dev->pdev;
	scif->drv_priv.hw_lock = &dev->hw_lock;
	scif->drv_priv.priv = (void *)scif;
	scif->drv_priv.coeff_addr = coeff_addr;
	dev->scif = scif;
	init_completion(&scif->done_com);
	mutex_init(&scif->sc_mutex);
	spin_lock_init(&dev->hw_lock);

	ret = devm_request_irq(&dev->pdev->dev, dev->irq, cpp_isr_root,
			IRQF_SHARED, "CPP", (void *)dev);
	if (ret < 0) {
		pr_err("fail to install IRQ %d\n", ret);
		goto vzalloc_exit;
	}

	pr_info("open sprd_cpp success\n");

	return ret;

vzalloc_exit:
	if (coeff_addr) {
		vfree(coeff_addr);
		dev->scif->drv_priv.coeff_addr = NULL;
	}
	if (scif) {
		vfree(scif);
		dev->scif = NULL;
	}
	if (rotif) {
		vfree(rotif);
		dev->rotif = NULL;
	}
	cpp_module_disable(dev);
en_cpp_exit:
	sprd_cam_domain_disable();
	ret = sprd_cam_pw_off();
	if (ret) {
		pr_err("%s: failed to camera power off\n", __func__);
		return ret;
	}
exit:
	if (atomic_dec_return(&dev->users) != 0)
		pr_info("others is using cpp device\n");
	file->private_data = NULL;

	return ret;
}

static int sprd_cpp_release(struct inode *node, struct file *file)
{
	int ret = 0;
	struct cpp_device *dev = NULL;

	dev = file->private_data;

	if (dev == NULL) {
		pr_err("fail to close cpp device\n");
		return -EFAULT;
	}

	if (atomic_dec_return(&dev->users) != 0) {
		pr_info("others is using cpp device\n");
		return ret;
	}

	if (dev->rotif) {
		mutex_destroy(&dev->rotif->rot_mutex);
		vfree(dev->rotif);
		dev->rotif = NULL;
	}
	if (dev->scif) {
		if (dev->scif->drv_priv.coeff_addr) {
			vfree(dev->scif->drv_priv.coeff_addr);
			dev->scif->drv_priv.coeff_addr = NULL;
		}
		mutex_destroy(&dev->scif->sc_mutex);
		vfree(dev->scif);
		dev->scif = NULL;
	}

	devm_free_irq(&dev->pdev->dev, dev->irq, (void *)dev);
	cpp_module_disable(dev);
	sprd_cam_domain_disable();
	ret = sprd_cam_pw_off();
	if (ret) {
		pr_err("%s: failed to camera power off\n", __func__);
		return ret;
	}

	file->private_data = NULL;
	pr_info("cpp release success\n");

	return ret;
}

static const struct file_operations cpp_fops = {
	.open = sprd_cpp_open,
	.unlocked_ioctl = sprd_cpp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = sprd_cpp_ioctl,
#endif
	.release = sprd_cpp_release,
};

static int sprd_cpp_probe(struct platform_device *pdev)
{
	int ret = 0;
	int irq = 0;
	struct cpp_device *dev = NULL;
	struct resource *res = NULL;
	struct regmap *cam_ahb_gpr = NULL;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		pr_err("fail to alloc dev!\n");
		goto exit;
	}

	mutex_init(&dev->lock);
	spin_lock_init(&dev->slock);
	atomic_set(&dev->users, 0);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		pr_err("fail to get io source\n");
		goto exit;
	}

	dev->io_base = devm_ioremap_nocache(&pdev->dev, res->start,
					    resource_size(res));
	if (IS_ERR_OR_NULL(dev->io_base))
		return PTR_ERR(dev->io_base);

	dev->cpp_eb = devm_clk_get(&pdev->dev, "cpp_eb");
	if (IS_ERR_OR_NULL(dev->cpp_eb))
		return PTR_ERR(dev->cpp_eb);

	dev->cpp_axi_eb = devm_clk_get(&pdev->dev, "cpp_axi_eb");
	if (IS_ERR_OR_NULL(dev->cpp_axi_eb))
		return PTR_ERR(dev->cpp_axi_eb);

	cam_ahb_gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
					"sprd,cam-ahb-syscon");
	if (IS_ERR_OR_NULL(cam_ahb_gpr))
		return PTR_ERR(cam_ahb_gpr);
	dev->cam_ahb_gpr = cam_ahb_gpr;

	dev->cpp_clk = devm_clk_get(&pdev->dev, "cpp_clk");
	if (IS_ERR_OR_NULL(dev->cpp_clk))
		return PTR_ERR(dev->cpp_clk);

	dev->cpp_clk_parent = devm_clk_get(&pdev->dev, "cpp_clk_parent");
	if (IS_ERR_OR_NULL(dev->cpp_clk_parent))
		return PTR_ERR(dev->cpp_clk_parent);

	dev->cpp_clk_default = clk_get_parent(dev->cpp_clk);
	if (IS_ERR_OR_NULL(dev->cpp_clk_default))
		return PTR_ERR(dev->cpp_clk_default);

	dev->md.minor = MISC_DYNAMIC_MINOR;
	dev->md.name = CPP_DEVICE_NAME;
	dev->md.fops = &cpp_fops;
	dev->md.parent = NULL;
	ret = misc_register(&dev->md);
	if (ret) {
		pr_err("fail to register misc devices\n");
		goto exit;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		pr_err("fail to get IRQ\n");
		ret = -ENXIO;
		goto fail_irq;
	}

	dev->irq = irq;

	dev->pdev = pdev;
	dev->md.this_device->platform_data = (void *)dev;
	platform_set_drvdata(pdev, (void *)dev);

	pr_info("cpp probe OK\n");

	return ret;

fail_irq:
	misc_deregister(&dev->md);
exit:
	return ret;
}

static int sprd_cpp_remove(struct platform_device *pdev)
{
	struct cpp_device *dev = platform_get_drvdata(pdev);

	if (!dev) {
		pr_err("fail to get dev to remove cpp device!\n");
		return -1;
	}

	mutex_destroy(&dev->lock);
	misc_deregister(&dev->md);

	return 0;
}

static const struct of_device_id of_match_table_rot[] = {
	{ .compatible = "sprd,cpp", },
	{},
};

static struct platform_driver sprd_cpp_driver = {
	.probe = sprd_cpp_probe,
	.remove = sprd_cpp_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = CPP_DEVICE_NAME,
		   .of_match_table = of_match_ptr(of_match_table_rot),
		   },
};

struct file *sprd_cpp_k_open(void)
{
	int i = 0;
	struct file *cpp_file = NULL;

	for (i = 0; i < 4; i++) {
		cpp_file = filp_open("/dev/sprd_cpp", O_RDWR, 0);
		if (cpp_file == NULL) {
			msleep(50);
		} else {
			pr_info("success to open sprd_cpp device node\n");
			break;
		}
	}

	return cpp_file;
}

int sprd_cpp_k_close(struct file *filp)
{
	int ret = 0;

	if (filp == NULL) {
		pr_err("Input ptr is NULL\n");
		return -EFAULT;
	}

	ret = filp_close(filp, NULL);

	return ret;
}

int rot_k_start(struct file *file, void *rot_param)
{
	int ret = 0;
	unsigned long rtn = 0;
	struct rotif_device *rotif = NULL;
	struct cpp_device *dev = NULL;
	struct sprd_cpp_rot_cfg_parm *rot_pm = rot_param;

	dev = file->private_data;
	if (!dev) {
		pr_err("others module fail to get cpp dev fot rot\n");
		return -EFAULT;
	}

	rotif = dev->rotif;
	if (!rotif) {
		pr_err("rot start: rotif is null!\n");
		return -EINVAL;
	}

	mutex_lock(&rotif->rot_mutex);

	rotif->drv_priv.iommu_src.dev = &dev->pdev->dev;
	rotif->drv_priv.iommu_dst.dev = &dev->pdev->dev;

	ret = cpp_rot_check_parm(rot_pm);
	if (ret) {
		pr_err("fail to check rot parm\n");
		mutex_unlock(&rotif->rot_mutex);
		return -EINVAL;
	}

	pr_info("start to rot y\n");
	ret = cpp_rot_set_y_parm(rot_pm, &rotif->drv_priv);
	if (ret) {
		pr_err("fail to set rot y parm\n");
		mutex_unlock(&rotif->rot_mutex);
		return -EINVAL;
	}
	cpp_rot_start(&rotif->drv_priv);


	if (cpp_rot_is_end(rot_pm) == 0) {
		rtn = wait_for_completion_timeout(&rotif->done_com,
				msecs_to_jiffies(ROT_TIMEOUT));
		if (rtn == 0) {
			cpp_rot_stop(&rotif->drv_priv);
			mutex_unlock(&rotif->rot_mutex);
			return -EBUSY;
		}

		pr_debug("start to rot uv\n");
		ret = cpp_rot_set_uv_parm(&rotif->drv_priv);
		if (ret) {
			pr_err("fail to set rot uv parm\n");
			mutex_unlock(&rotif->rot_mutex);
			return -EINVAL;
		}
		cpp_rot_start(&rotif->drv_priv);
	}

	rtn = wait_for_completion_timeout(&rotif->done_com,
				msecs_to_jiffies(ROT_TIMEOUT));
	if (rtn == 0) {
		cpp_rot_stop(&rotif->drv_priv);
		mutex_unlock(&rotif->rot_mutex);
		return -EBUSY;
	}

	cpp_rot_stop(&rotif->drv_priv);
	mutex_unlock(&rotif->rot_mutex);

	return ret;
}

int scale_k_start(struct file *file, void *scale_param)
{
	int ret = 0;
	unsigned long rtn = 0;
	struct scif_device *scif = NULL;
	struct cpp_device *dev = NULL;
	struct sprd_cpp_scale_cfg_parm *sc_parm = NULL;

	dev = file->private_data;
	if (!dev) {
		pr_err("others module fail to get cpp dev for scale\n");
		return -EFAULT;
	}
	scif = dev->scif;
	sc_parm = scale_param;

	if (!scif || !sc_parm) {
		pr_err("scale start:scif is null!\n");
		return -EFAULT;
	}

	mutex_lock(&scif->sc_mutex);

	scif->drv_priv.iommu_src.dev = &dev->pdev->dev;
	scif->drv_priv.iommu_dst.dev = &dev->pdev->dev;

	pr_info("start to scale\n");
	ret = cpp_scale_start(sc_parm, &scif->drv_priv);
	if (ret) {
		pr_err("fail to start scaler\n");
		mutex_unlock(&scif->sc_mutex);
		return -EFAULT;
	}

	rtn = wait_for_completion_timeout(&scif->done_com,
				msecs_to_jiffies
				(SCALE_TIMEOUT));
	if (rtn == 0) {
		cpp_scale_stop(&scif->drv_priv);
		mutex_unlock(&scif->sc_mutex);
		pr_err("fail to get scaling done com\n");
		return -EBUSY;
	}

	cpp_scale_stop(&scif->drv_priv);
	mutex_unlock(&scif->sc_mutex);

	return ret;
}

static int __init sprd_cpp_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&sprd_cpp_driver);
	pr_info("init ret %d\n", ret);

	return ret;
}

static void sprd_cpp_exit(void)
{
	platform_driver_unregister(&sprd_cpp_driver);
}

module_init(sprd_cpp_init);
module_exit(sprd_cpp_exit);
MODULE_DESCRIPTION("Sharkl2 Cpp Driver");
MODULE_LICENSE("GPL");
