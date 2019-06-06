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
#include <linux/mfd/syscon/sprd-glb.h>
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
#include "cam_pw_domain.h"
#include "cpp_reg.h"
#include "rot_drv.h"
#include "scale_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CPP_CORE: %d: %d " fmt, current->pid, __LINE__

#define CPP_DEVICE_NAME                "sprd_cpp"
#define ROT_TIMEOUT                    5000
#define SCALE_TIMEOUT                  5000
#define CPP_IRQ_LINE_MASK              0x3UL

enum cpp_irq_id {
	CPP_SCALE_DONE = 0,
	CPP_ROT_DONE,
	CPP_IRQ_NUMBER
};

struct rotif_device {
	atomic_t count;
	struct semaphore start_sem;
	struct completion done_com;
	struct rot_drv_private drv_priv;
};

struct scif_device {
	atomic_t count;
	struct semaphore start_sem;
	struct completion done_com;
	struct scale_drv_private drv_priv;
};

typedef void (*cpp_isr_func) (void *);

struct cpp_device {
	atomic_t users;
	struct mutex hw_lock;
	struct mutex lock;
	spinlock_t slock;

	struct rotif_device *rotif;
	struct scif_device *scif;
	struct miscdevice md;

	cpp_isr_func isr_func[CPP_IRQ_NUMBER];
	void *isr_data[CPP_IRQ_NUMBER];

	void __iomem *io_base;

	struct platform_device *pdev;

	struct clk *cpp_clk;
	struct clk *cpp_clk_parent;
	struct clk *cpp_clk_default;

	struct clk *cpp_eb;
	struct clk *cpp_axi_eb;

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

static irqreturn_t cpp_isr_root(int irq, void *priv)
{
	int i;
	unsigned int irq_line, status;
	unsigned long flag;
	struct cpp_device *dev = (struct cpp_device *)priv;

	status = reg_rd(dev, CPP_INT_STS) & CPP_IRQ_LINE_MASK;
	if (unlikely(status == 0))
		return IRQ_NONE;

	irq_line = status;

	spin_lock_irqsave(&dev->slock, flag);
	for (i = 0; i < CPP_IRQ_NUMBER; i++) {
		if (irq_line & (1 << (unsigned int)i)) {
			if (cpp_isr_list[i])
				cpp_isr_list[i] (dev);
		}
		irq_line &= ~(unsigned int)(1 << (unsigned int)i);
		if (!irq_line)
			break;
	}

	reg_wr(dev, CPP_INT_CLR, status);
	spin_unlock_irqrestore(&dev->slock, flag);

	return IRQ_HANDLED;
}

static int cpp_module_enable(struct cpp_device *dev)
{
	int ret = 0;

	mutex_lock(&dev->lock);
	if (atomic_inc_return(&dev->users) == 1) {
		/* enable clk */
		sprd_cam_pw_on();

		ret = clk_prepare_enable(dev->cpp_eb);
		if (ret)
			goto fail;

		ret = clk_prepare_enable(dev->cpp_axi_eb);
		if (ret) {
			clk_disable_unprepare(dev->cpp_eb);
			goto fail;
		}

		ret = clk_set_parent(dev->cpp_clk, dev->cpp_clk_parent);
		if (ret) {
			clk_disable_unprepare(dev->cpp_axi_eb);
			clk_disable_unprepare(dev->cpp_eb);
			goto fail;
		}

		ret = clk_prepare_enable(dev->cpp_clk);
		if (ret) {
			clk_disable_unprepare(dev->cpp_axi_eb);
			clk_disable_unprepare(dev->cpp_eb);
			goto fail;
		}

		regmap_update_bits(dev->cam_ahb_gpr,
				   REG_CAM_AHB_AHB_RST,
				   BIT_CAM_AHB_CPP_SOFT_RST,
				   BIT_CAM_AHB_CPP_SOFT_RST);
		udelay(1);
		regmap_update_bits(dev->cam_ahb_gpr,
				   REG_CAM_AHB_AHB_RST,
				   BIT_CAM_AHB_CPP_SOFT_RST,
				   ~(unsigned int)BIT_CAM_AHB_CPP_SOFT_RST);
	}
	mutex_unlock(&dev->lock);

	return 0;

fail:
	mutex_unlock(&dev->lock);
	return ret;
}

static int cpp_module_disable(struct cpp_device *dev)
{
	int ret = 0;

	mutex_lock(&dev->lock);
	if (atomic_dec_return(&dev->users) == 0) {
		/* disable clk */
		regmap_update_bits(dev->cam_ahb_gpr,
				   REG_CAM_AHB_AHB_RST,
				   BIT_CAM_AHB_CPP_SOFT_RST,
				   BIT_CAM_AHB_CPP_SOFT_RST);
		udelay(1);
		regmap_update_bits(dev->cam_ahb_gpr,
				   REG_CAM_AHB_AHB_RST,
				   BIT_CAM_AHB_CPP_SOFT_RST,
				   ~(unsigned int)BIT_CAM_AHB_CPP_SOFT_RST);

		clk_set_parent(dev->cpp_clk, dev->cpp_clk_default);
		clk_disable_unprepare(dev->cpp_clk);
		clk_disable_unprepare(dev->cpp_axi_eb);
		clk_disable_unprepare(dev->cpp_eb);

		sprd_cam_pw_off();
	}
	mutex_unlock(&dev->lock);

	return ret;
}

static void cpp_register_isr(struct cpp_device *dev, enum cpp_irq_id id,
			     cpp_isr_func user_func, void *priv)
{
	unsigned long flag;

	if (id >= CPP_IRQ_NUMBER)
		return;
	spin_lock_irqsave(&dev->slock, flag);
	dev->isr_func[id] = user_func;
	dev->isr_data[id] = priv;
	if (user_func)
		reg_mwr(dev, CPP_INT_MASK, (1 << id), ~(1 << id));
	else
		reg_mwr(dev, CPP_INT_MASK, (1 << id), (1 << id));
	spin_unlock_irqrestore(&dev->slock, flag);
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

static void cpp_read_reg(struct cpp_device *dev, unsigned int *reg_buf,
			 unsigned int *buf_len)
{
	unsigned int offset = 0;

	while (buf_len != 0 && offset < CPP_END) {
		*reg_buf++ = reg_rd(dev, offset);
		offset += 4;
		*buf_len -= 4;
	}

	*buf_len = offset;
}

void cpp_print_reg(void *priv)
{
	unsigned int *reg_buf = NULL;
	unsigned int reg_buf_len = 0x400;
	unsigned int print_len = 0, print_cnt = 0;
	struct cpp_device *dev = (struct cpp_device *)priv;

	reg_buf = vzalloc(reg_buf_len);
	if (!reg_buf)
		return;

	cpp_read_reg(dev, reg_buf, &reg_buf_len);

	pr_info("scale registers\n");
	while (print_len < reg_buf_len) {
		pr_info("offset 0x%03x : 0x%08x, 0x%08x, 0x%08x, 0x%08x\n",
			print_len,
			reg_buf[print_cnt],
			reg_buf[print_cnt + 1],
			reg_buf[print_cnt + 2], reg_buf[print_cnt + 3]);
		print_cnt += 4;
		print_len += 16;
	}
	vfree(reg_buf);
}

int cpp_get_sg_table(struct cpp_iommu_info *pfinfo)
{
	int i, ret;

	for (i = 0; i < 2; i++) {
		if (pfinfo->mfd[i] > 0) {
			ret = sprd_ion_get_sg_table(pfinfo->mfd[i],
				NULL,
				&pfinfo->table[i],
				&pfinfo->size[i]);
			if (ret) {
				pr_err("failed to get sg table\n");
				return -EFAULT;
			}
		}
	}
	return 0;
}

int cpp_get_addr(struct cpp_iommu_info *pfinfo)
{
	int i, ret;
	struct sprd_iommu_map_data iommu_data;

	for (i = 0; i < 2; i++) {
		if (pfinfo->size[i] <= 0)
			continue;

		if (sprd_iommu_attach_device(pfinfo->dev) == 0) {
			memset(&iommu_data, 0,
				sizeof(struct sprd_iommu_map_data));
			iommu_data.table = pfinfo->table[i];
			iommu_data.iova_size = pfinfo->size[i];
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			iommu_data.sg_offset = pfinfo->offset[i];

			ret = sprd_iommu_get_kaddr(pfinfo->dev, &iommu_data);
			if (ret) {
				pr_err("failed to get iommu kaddr %d\n", i);
				return -EFAULT;
			}

			pfinfo->iova[i] = iommu_data.iova_addr
					+ pfinfo->offset[i];
		} else {
			sprd_ion_get_phys_addr(pfinfo->mfd[i],
					NULL,
					&pfinfo->iova[i],
					&pfinfo->size[i]);
			pfinfo->iova[i] += pfinfo->offset[i];
		}
	}
	return 0;
}

int cpp_free_addr(struct cpp_iommu_info *pfinfo)
{
	int i, ret;
	struct sprd_iommu_unmap_data iommu_data;

	for (i = 0; i < 2; i++) {
		if (pfinfo->size[i] <= 0)
			continue;

		if (sprd_iommu_attach_device(pfinfo->dev) == 0) {
			iommu_data.iova_addr = pfinfo->iova[i]
						- pfinfo->offset[i];
			iommu_data.iova_size = pfinfo->size[i];
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;

			ret = sprd_iommu_free_kaddr(pfinfo->dev, &iommu_data);
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
	struct cpp_device *dev = NULL;
	unsigned long timeleft = 0;

	dev = file->private_data;
	if (!dev)
		return -EFAULT;

	switch (cmd) {
	case SPRD_CPP_IO_OPEN_ROT:
		{
			struct rotif_device *rotif = NULL;

			if (!dev->rotif) {
				rotif = vzalloc(sizeof(*rotif));
				if (unlikely(!rotif)) {
					pr_err("rot open: vzalloc fail!\n");
					return -ENOMEM;
				}
				init_completion(&rotif->done_com);

				rotif->drv_priv.io_base = dev->io_base;
				rotif->drv_priv.priv = (void *)rotif;
				dev->rotif = rotif;

				cpp_register_isr(dev, CPP_ROT_DONE, rot_isr,
						 (void *)rotif);
				rotif->drv_priv.iommu_src.dev = &dev->pdev->dev;
				rotif->drv_priv.iommu_dst.dev = &dev->pdev->dev;
			}
			atomic_inc(&dev->rotif->count);
			break;
		}

	case SPRD_CPP_IO_START_ROT:
		{
			struct rotif_device *rotif = NULL;
			struct sprd_cpp_rot_cfg_parm parm;

			rotif = dev->rotif;
			if (!rotif) {
				pr_err("rot start: rotif is null!\n");
				return -EINVAL;
			}

			mutex_lock(&dev->hw_lock);

			ret = copy_from_user(&parm,
					     (struct sprd_cpp_rot_cfg_parm *)
					     arg, sizeof(parm));
			if (ret) {
				mutex_unlock(&dev->hw_lock);
				return -EFAULT;
			}

			ret = cpp_rot_check_parm(&parm);
			if (ret) {
				pr_err("failed to check rot parm\n");
				mutex_unlock(&dev->hw_lock);
				return -EINVAL;
			}

			pr_debug("start to rot y\n");
			cpp_rot_set_y_parm(&parm, &rotif->drv_priv);
			cpp_rot_start(&rotif->drv_priv);

			if (cpp_rot_is_end(&parm) == 0) {
				timeleft = wait_for_completion_timeout
						(&rotif->done_com,
						msecs_to_jiffies(ROT_TIMEOUT));
				if (timeleft == 0) {
					cpp_rot_stop(&rotif->drv_priv);
					mutex_unlock(&dev->hw_lock);
					return -1;
				}

				pr_debug("start to rot uv\n");
				cpp_rot_set_uv_parm(&rotif->drv_priv);
				cpp_rot_start(&rotif->drv_priv);
			}

			timeleft = wait_for_completion_timeout
						(&rotif->done_com,
						msecs_to_jiffies(ROT_TIMEOUT));
			if (timeleft == 0) {
				cpp_rot_stop(&rotif->drv_priv);
				mutex_unlock(&dev->hw_lock);
				return -1;
			}

			cpp_rot_stop(&rotif->drv_priv);
			mutex_unlock(&dev->hw_lock);
			break;
		}

	case SPRD_CPP_IO_OPEN_SCALE:
		{
			struct scif_device *scif = NULL;
			void *coeff_addr = NULL;

			if (!dev->scif) {
				scif = vzalloc(sizeof(*scif));
				if (unlikely(!scif)) {
					pr_err("scale open: vzalloc fail!\n");
					return -ENOMEM;
				}
				coeff_addr = vzalloc(SC_COEFF_BUF_SIZE);
				if (unlikely(!coeff_addr)) {
					pr_err("scale open: vzalloc coeff_addr fail!\n");
					vfree(scif);
					return -ENOMEM;
				}
				sema_init(&scif->start_sem, 1);
				init_completion(&scif->done_com);

				scif->drv_priv.io_base = dev->io_base;
				scif->drv_priv.pdev = dev->pdev;
				scif->drv_priv.priv = (void *)scif;
				scif->drv_priv.coeff_addr = coeff_addr;
				dev->scif = scif;
				cpp_register_isr(dev, CPP_SCALE_DONE, scale_isr,
				 (void *)scif);
				scif->drv_priv.iommu_src.dev = &dev->pdev->dev;
				scif->drv_priv.iommu_dst.dev = &dev->pdev->dev;
			}
			atomic_inc(&dev->scif->count);
			break;
		}

	case SPRD_CPP_IO_START_SCALE:
		{
			struct scif_device *scif = dev->scif;
			struct sprd_cpp_scale_cfg_parm parm;

			WARN_ON(!scif);

			mutex_lock(&dev->hw_lock);
			down(&scif->start_sem);

			ret = copy_from_user(&parm,
					     (struct sprd_cpp_scale_cfg_parm *)
					     arg,
					     sizeof(struct
						    sprd_cpp_scale_cfg_parm));
			if (ret) {
				up(&scif->start_sem);
				mutex_unlock(&dev->hw_lock);
				return -EFAULT;
			}

			pr_debug("start to scale\n");
			ret = cpp_scale_start(&parm, &scif->drv_priv);
			if (ret) {
				pr_err("failed to start scaler\n");
				up(&scif->start_sem);
				mutex_unlock(&dev->hw_lock);
				return -1;
			}
			break;
		}

	case SPRD_CPP_IO_CONTINUE_SCALE:
		{
			pr_info("not supported by current driver\n");
			break;
		}

	case SPRD_CPP_IO_STOP_SCALE:
		{
			struct scif_device *scif = dev->scif;

			WARN_ON(!scif);

			scif->drv_priv.iommu_src.dev = &dev->pdev->dev;
			scif->drv_priv.iommu_dst.dev = &dev->pdev->dev;

			timeleft = wait_for_completion_timeout(&scif->done_com,
						msecs_to_jiffies
						(SCALE_TIMEOUT));
			if (timeleft == 0) {
				pr_err("failed to get scaling done com\n");
				ret = -1;
			}

			cpp_scale_stop(&scif->drv_priv);

			up(&scif->start_sem);
			mutex_unlock(&dev->hw_lock);
			break;
		}

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
	struct miscdevice *md = (struct miscdevice *)file->private_data;

	if (!md)
		return -EFAULT;

	dev = md->this_device->platform_data;

	file->private_data = (void *)dev;
	ret = cpp_module_enable(dev);
	if (ret) {
		pr_err("failed to enable cpp\n");
		return -1;
	}

	return 0;
}

static int sprd_cpp_release(struct inode *node, struct file *file)
{
	struct cpp_device *dev = NULL;
	struct rotif_device *rotif = NULL;
	struct scif_device *scif = NULL;

	dev = file->private_data;
	if (!dev)
		return -EFAULT;

	rotif = dev->rotif;
	if (unlikely(rotif) && atomic_dec_return(&rotif->count) == 0) {
		cpp_register_isr(dev, CPP_ROT_DONE, NULL, NULL);
		vfree(rotif);
		dev->rotif = NULL;
	}

	scif = dev->scif;
	if (unlikely(scif) && atomic_dec_return(&scif->count) == 0) {
		down(&scif->start_sem);
		up(&scif->start_sem);
		cpp_register_isr(dev, CPP_SCALE_DONE, NULL, NULL);
		vfree(scif->drv_priv.coeff_addr);
		vfree(scif);
		dev->scif = NULL;
	}

	cpp_module_disable(dev);

	return 0;
}

static const struct file_operations cpp_fops = {
	.owner = THIS_MODULE,
	.open = sprd_cpp_open,
	.unlocked_ioctl = sprd_cpp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = sprd_cpp_ioctl,
#endif
	.release = sprd_cpp_release,
};

static int sprd_cpp_probe(struct platform_device *pdev)
{
	int ret = 0, irq;
	struct cpp_device *dev = NULL;
	struct resource *res = NULL;
	struct regmap *cam_ahb_gpr;
	struct regmap *pmu_apb_gpr;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->lock);
	spin_lock_init(&dev->slock);
	mutex_init(&dev->hw_lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	dev->io_base = devm_ioremap_nocache(&pdev->dev, res->start,
					    resource_size(res));
	if (IS_ERR(dev->io_base))
		return PTR_ERR(dev->io_base);

	dev->cpp_clk = devm_clk_get(&pdev->dev, "clk_cpp");
	if (IS_ERR(dev->cpp_clk))
		return PTR_ERR(dev->cpp_clk);

	dev->cpp_clk_default = clk_get_parent(dev->cpp_clk);
	if (IS_ERR(dev->cpp_clk_default))
		return PTR_ERR(dev->cpp_clk_default);

	dev->cpp_clk_parent = devm_clk_get(&pdev->dev, "clk_cpp_parent");
	if (IS_ERR(dev->cpp_clk_parent))
		return PTR_ERR(dev->cpp_clk_parent);

	dev->cpp_eb = devm_clk_get(&pdev->dev, "cpp_eb");
	if (IS_ERR(dev->cpp_eb))
		return PTR_ERR(dev->cpp_eb);

	dev->cpp_axi_eb = devm_clk_get(&pdev->dev, "cpp_axi_eb");
	if (IS_ERR(dev->cpp_axi_eb))
		return PTR_ERR(dev->cpp_axi_eb);

	cam_ahb_gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						      "sprd,syscon-cam-ahb");
	if (IS_ERR(cam_ahb_gpr))
		return PTR_ERR(cam_ahb_gpr);
	dev->cam_ahb_gpr = cam_ahb_gpr;

	pmu_apb_gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						      "sprd,syscon-pmu-apb");
	if (IS_ERR(pmu_apb_gpr))
		return PTR_ERR(pmu_apb_gpr);

	dev->md.minor = MISC_DYNAMIC_MINOR;
	dev->md.name = CPP_DEVICE_NAME;
	dev->md.fops = &cpp_fops;
	dev->md.parent = NULL;
	ret = misc_register(&dev->md);
	if (ret) {
		pr_err("failed to register misc devices\n");
		return ret;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		pr_err("failed to get IRQ\n");
		ret = -ENXIO;
		goto fail;
	}

	ret = devm_request_irq(&pdev->dev,
			       irq,
			       cpp_isr_root, IRQF_SHARED, "CPP", (void *)dev);
	if (ret < 0) {
		pr_err("failed to install IRQ %d\n", ret);
		goto fail;
	}

	dev->pdev = pdev;
	dev->md.this_device->platform_data = (void *)dev;
	platform_set_drvdata(pdev, (void *)dev);

	/* workaround for power when start up */
	regmap_update_bits(pmu_apb_gpr,
			   REG_PMU_APB_PD_CAM_SYS_CFG,
			   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN,
			   ~(unsigned int)
			   BIT_PMU_APB_PD_CAM_SYS_AUTO_SHUTDOWN_EN);
	regmap_update_bits(pmu_apb_gpr,
			   REG_PMU_APB_PD_CAM_SYS_CFG,
			   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN,
			   BIT_PMU_APB_PD_CAM_SYS_FORCE_SHUTDOWN);

	return 0;

fail:
	misc_deregister(&dev->md);
	return ret;
}

static int sprd_cpp_remove(struct platform_device *pdev)
{
	struct cpp_device *dev = platform_get_drvdata(pdev);

	misc_deregister(&dev->md);

	return 0;
}

static const struct of_device_id of_match_table_rot[] = {
	{.compatible = "sprd,cpp",},
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
MODULE_DESCRIPTION("Cpp Driver");
MODULE_LICENSE("GPL");
