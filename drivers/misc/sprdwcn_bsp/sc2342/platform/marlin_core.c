/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
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
#include <linux/fs.h>
#include <linux/firmware.h>
#include <linux/file.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/marlin_platform.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/sdiom_rx_api.h>
#include <linux/sdiom_tx_api.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#include "mdbg_main.h"
#include "rf/rf.h"
#include "slp_mgr.h"
#include "wcn_bus.h"
#include "wcn_debug.h"
#include "wcn_glb_reg.h"
#include "wcn_op.h"
#include "wcn_parn_parser.h"

#ifndef REG_PMU_APB_XTL_WAIT_CNT0
#define REG_PMU_APB_XTL_WAIT_CNT0 0xe42b00ac
#endif

static char FIRMWARE_PATH[255];
struct marlin_device {
	int wakeup_ap;
	int reset;
	int chip_en;
	int int_ap;
	struct regulator *vddwcn;
	struct clk *clk_32k;
	struct regulator *vdd_marlin2_1v2;
	struct clk *clk_parent;
	struct clk *clk_enable;
	struct mutex power_lock;
	struct completion carddetect_done;
	struct completion download_done;
	unsigned long power_state;
	char *write_buffer;
	struct delayed_work power_wq;
	struct work_struct download_wq;
	bool power_ctl_disabled;
};

struct wifi_calibration {
	struct wifi_config_t config_data;
	struct wifi_cali_t cali_data;
};

static struct wifi_calibration wifi_data;
struct completion ge2_completion;
static int flag1;
marlin_reset_callback marlin_reset_func;
void *marlin_callback_para;

/* static struct semaphore marlin_chipup_sem; */
static struct marlin_device *marlin_dev;
int wcn_open_module;
int wcn_module_state_change;
unsigned char flag_download;
static unsigned char slp_mgr_init_done;
unsigned char  flag_reset;
char functionmask[8];
static unsigned int reg_val;
static unsigned int clk_wait_val;
static unsigned int cp_clk_wait_val;
static unsigned int marlin2_clk_wait_reg;

/* temp for rf pwm mode */
static struct regmap *pwm_regmap;

#define IMG_HEAD_MAGIC "WCNM"
#define IMG_MARLINAA_TAG "MLAA"
#define IMG_MARLINAB_TAG "MLAB"
#define MARLIN2_AA	1
#define MARLIN2_AB	2
#define MARLIN2_AA_CHIP 0x23490000

struct head {
	char magic[4];
	unsigned int version;
	unsigned int img_count;
};

struct imageinfo {
	char tag[4];
	unsigned int offset;
	unsigned int size;
};

static unsigned long int chip_id;

void marlin_get_wcn_chipid(void)
{
	int ret;

	ret = sprdwcn_bus_direct_read(CHIPID_REG, &chip_id, 4);
	if (ret < 0) {
		pr_err("marlin read chip ID fail\n");
		return;
	}
	pr_info("marlin: chipid=%lx, %s\n", chip_id, __func__);
}

static int marlin_judge_imagepack(char *buffer)
{
	int ret;
	struct head *imghead;

	if (buffer == NULL)
		return -1;
	imghead = vmalloc(sizeof(struct head));
	if (!imghead) {
		pr_err("%s no memory\n", __func__);
		return -1;
	}
	memcpy(imghead, buffer, sizeof(struct head));
	pr_info("marlin image  pack type:%s in the func %s:\n",
		imghead->magic, __func__);
	ret = strncmp(IMG_HEAD_MAGIC, imghead->magic, 4);
	vfree(imghead);

	return ret;
}


static struct imageinfo *marlin_judge_images(char *buffer)
{

	struct imageinfo *imginfo = NULL;

	if (buffer == NULL)
		return NULL;
	imginfo = vmalloc((sizeof(struct imageinfo)));
	if (!imginfo) {
		pr_err("%s no memory\n", __func__);
		return NULL;
	}

	if (chip_id == MARLIN2_AA_CHIPID) {
		pr_info("%s marlin2 is AA !!!!\n",  __func__);
		memcpy(imginfo, (buffer + sizeof(struct head)),
			sizeof(struct imageinfo));
		if (!strncmp(IMG_MARLINAA_TAG, imginfo->tag, 4)) {
			pr_info("marlin imginfo1 type is %s in the func %s:\n",
				imginfo->tag, __func__);
			return imginfo;
		}
		pr_err("Marlin can't find marlin AA image!!!\n");
		vfree(imginfo);
		return NULL;
	} else if (chip_id == MARLIN2_AB_CHIPID) {
		pr_info("%s marlin is AB !!!!\n", __func__);
		memcpy(imginfo,
			buffer + sizeof(struct imageinfo) + sizeof(struct head),
			sizeof(struct imageinfo));
		if (!strncmp(IMG_MARLINAB_TAG, imginfo->tag, 4)) {
			pr_info("marlin imginfo1 type is %s in the func %s:\n",
				imginfo->tag, __func__);
			return imginfo;
		}
		pr_err("Marlin can't find marlin AB image!!!\n");
		vfree(imginfo);
		return NULL;
	}
	pr_err("Marlin can't find marlin AB or AA image!!!\n");
	vfree(imginfo);

	return NULL;
}

static char *load_firmware_data(unsigned long int imag_size)
{
	int read_len, size, i, opn_num_max = 5;
	char *buffer = NULL;
	char *data = NULL;
	struct file *file;
	loff_t offset = 0;

	pr_info("%s entry\n", __func__);
	parse_firmware_path(FIRMWARE_PATH);
	pr_info("marlin parse path is %s\n", FIRMWARE_PATH);

	file = filp_open(FIRMWARE_PATH, O_RDONLY, 0);
	for (i = 1; i <= opn_num_max; i++) {
		if (IS_ERR(file)) {
			pr_info("try open file %s,count_num:%d,%s\n",
				FIRMWARE_PATH, i, __func__);
			ssleep(1);
			file = filp_open(FIRMWARE_PATH, O_RDONLY, 0);
		} else
			break;
	}
	if (IS_ERR(file)) {
		pr_err("%s open file %s error\n",
			FIRMWARE_PATH, __func__);
		return NULL;
	}
	pr_info("marlin %s open image file  successfully\n",
		__func__);
	size = imag_size;
	buffer = vmalloc(size);
	if (!buffer) {
		fput(file);
		pr_err("no memory\n");
		return NULL;
	}

	read_len = kernel_read(file, 0, functionmask, 8);
	if ((functionmask[0] == 0x00) && (functionmask[1] == 0x00))
		offset = offset + 8;
	else
		functionmask[7] = 0;

	data = buffer;
	do {
		read_len = kernel_read(file, offset, buffer, size);
		if (read_len > 0) {
			size -= read_len;
			buffer += read_len;
		}
	} while ((read_len > 0) && (size > 0));
	fput(file);
	pr_info("%s finish read_Len:%d\n", __func__, read_len);

	return data;
}

static int marlin_download_firmware(void)
{
	int err, len, trans_size, ret;
	unsigned long int imgpack_size, img_size;
	char *buffer = NULL;
	char *temp = NULL;
	struct imageinfo *imginfo = NULL;

	img_size = imgpack_size =  FIRMWARE_MAX_SIZE;

	pr_info("%s entry\n", __func__);
	temp = buffer = load_firmware_data(imgpack_size);
	if (!buffer) {
		pr_err("%s buff is NULL\n", __func__);
		return -1;
	}

	ret = marlin_judge_imagepack(buffer);
	if (!ret) {
		pr_info("marlin %s imagepack is WCNM type,need parse it\n",
			__func__);
		marlin_get_wcn_chipid();

		imginfo = marlin_judge_images(buffer);
		vfree(temp);
		if (!imginfo) {
			pr_err("marlin:%s imginfo is NULL\n", __func__);
			return -1;
		}
		imgpack_size = imginfo->offset + imginfo->size;
		temp = buffer = load_firmware_data(imgpack_size);
		if (!buffer) {
			pr_err("marlin:%s buffer is NULL\n", __func__);
			vfree(imginfo);
			return -1;
		}
		buffer += imginfo->offset;
		img_size = imginfo->size;
		vfree(imginfo);
	}
	len = 0;
	while (len < img_size) {
		trans_size = (img_size - len) > PACKET_SIZE ?
				PACKET_SIZE : (img_size - len);
		memcpy(marlin_dev->write_buffer, buffer + len, trans_size);
		err = sprdwcn_bus_direct_write(CP_START_ADDR + len,
			marlin_dev->write_buffer, trans_size);
		if (err < 0) {
			pr_err("marlin %s error:%d\n", __func__, err);
			vfree(temp);
			return -1;
		}
		len += PACKET_SIZE;
	}
	vfree(temp);
	pr_info("%s finish\n", __func__);

	return 0;
}

static int download_firmware(void)
{
	const struct firmware *firmware;
	char *buf;
	char *buffer = NULL;
	int err, ret;
	int i, len, count, trans_size;
	struct imageinfo *imginfo = NULL;
	unsigned long int img_size;

	pr_info("marlin %s start!\n", __func__);
	buf = marlin_dev->write_buffer;
	pr_info("marlin fs DL  begin to read wcnmodem.bin\n");
	err = request_firmware_direct(&firmware, "wcnmodem.bin", NULL);
	if (err < 0) {
		pr_err("marlin fs DL:%s no find wcnmodem.bin errno:(%d)(ignore!!)\n",
			__func__, err);
		err = marlin_download_firmware();

		return err;
	}

	/* add judge img from chip id according to fs */
	ret = marlin_judge_imagepack((char *)(firmware->data));
	if (!ret) {
		pr_info("marlin fs DL %s imagepack is WCNM type,need parse it\n",
			__func__);
		marlin_get_wcn_chipid();

		imginfo = marlin_judge_images((char *)(firmware->data));
		if (!imginfo) {
			pr_err("marlin fs DL:%s imginfo is NULL\n", __func__);
			return -1;
		}

		buffer = (char *)(firmware->data);
		if (!buffer) {
			pr_err("marlin fs DL:%s buffer is NULL\n", __func__);
			vfree(imginfo);
			return -1;
		}
		buffer += imginfo->offset;
		img_size = imginfo->size;
		vfree(imginfo);
	}

	count = (firmware->size + PACKET_SIZE - 1) / PACKET_SIZE;
	len = 0;

	for (i = 0; i < count; i++) {
		trans_size = (img_size - len) > PACKET_SIZE ?
				PACKET_SIZE : (img_size - len);
		memcpy(buf, buffer + len, trans_size);
		err = sprdwcn_bus_direct_write(CP_START_ADDR + len,
					       buf, trans_size);
		if (err < 0) {
			pr_err("marlin fs DL %s error:%d\n", __func__, err);
			release_firmware(firmware);
			return err;
		}
		len += trans_size;
	}

	release_firmware(firmware);
	pr_info("marlin fs DL %s successfully!\n", __func__);

	return 0;
}

static int marlin_parse_dt(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct regmap *pmu_apb_gpr;
	int ret;

	if (!marlin_dev)
		return -1;

	marlin_dev->wakeup_ap = of_get_named_gpio(np,
			"m2-wakeup-ap-gpios", 0);
	if (!gpio_is_valid(marlin_dev->wakeup_ap))
		return -EINVAL;

	marlin_dev->reset = of_get_named_gpio(np,
			"rstn-gpios", 0);
	if (!gpio_is_valid(marlin_dev->reset))
		return -EINVAL;

	marlin_dev->chip_en = of_get_named_gpio(np,
			"chip-en-gpios", 0);
	if (!gpio_is_valid(marlin_dev->chip_en))
		return -EINVAL;

	marlin_dev->int_ap = of_get_named_gpio(np,
			"m2-to-ap-irq-gpios", 0);
	if (!gpio_is_valid(marlin_dev->int_ap))
		return -EINVAL;

	marlin_dev->vddwcn = devm_regulator_get(&pdev->dev, "vddwcn");
	if (IS_ERR(marlin_dev->vddwcn)) {
		pr_err("Get regulator of vddwcn error!\n");
		return -1;
	}

	marlin_dev->vdd_marlin2_1v2 = devm_regulator_get(&pdev->dev,
	"vdd_marlin2_1v2");

	marlin_dev->clk_32k = devm_clk_get(&pdev->dev, "clk_32k");
	if (IS_ERR(marlin_dev->clk_32k)) {
		pr_err("can't get wcn clock dts config: clk_32k\n");
		return -1;
	}

	marlin_dev->clk_parent = devm_clk_get(&pdev->dev, "source");
	if (IS_ERR(marlin_dev->clk_parent)) {
		pr_err("can't get wcn clock dts config: source\n");
		return -1;
	}
	clk_set_parent(marlin_dev->clk_32k, marlin_dev->clk_parent);

	marlin_dev->clk_enable = devm_clk_get(&pdev->dev, "enable");
	if (IS_ERR(marlin_dev->clk_enable)) {
		pr_err("can't get wcn clock dts config: enable\n");
		return -1;
	}

	ret = gpio_request(marlin_dev->reset, "reset");
	if (ret)
		pr_err("gpio reset request err: %d\n",
				marlin_dev->reset);

	ret = gpio_request(marlin_dev->chip_en, "chip_en");
	if (ret)
		pr_err("gpio_rst request err: %d\n",
				marlin_dev->chip_en);

	ret = gpio_request(marlin_dev->int_ap, "int_ap");
	if (ret)
		pr_err("gpio_rst request err: %d\n",
				marlin_dev->int_ap);

	if (of_get_property(np, "power-ctl-disabled", NULL))
		marlin_dev->power_ctl_disabled = true;

	pmu_apb_gpr = syscon_regmap_lookup_by_phandle(np,
				"sprd,syscon-pmu-apb");
	if (IS_ERR(pmu_apb_gpr)) {
		pr_err("%s:failed to find pmu_apb_gpr\n", __func__);
		return -EINVAL;
	}
	ret = regmap_read(pmu_apb_gpr, REG_PMU_APB_XTL_WAIT_CNT0,
					&clk_wait_val);
	pr_info("marlin2 clk_wait value is 0x%x\n", clk_wait_val);

	ret = of_property_read_u32(np, "sprd,reg-m2-apb-xtl-wait-addr",
			&marlin2_clk_wait_reg);
	if (ret) {
		pr_err("Did not find reg-m2-apb-xtl-wait-addr\n");
		return -EINVAL;
	}
	pr_info("marlin2 clk reg is 0x%x\n", marlin2_clk_wait_reg);

	return 0;
}

static int marlin_clk_enable(bool enable)
{
	int ret = 0;

	if (enable) {
		ret = clk_prepare_enable(marlin_dev->clk_32k);
		ret = clk_prepare_enable(marlin_dev->clk_enable);
		pr_info("marlin %s successfully!\n", __func__);
	} else {
		clk_disable_unprepare(marlin_dev->clk_enable);
		clk_disable_unprepare(marlin_dev->clk_32k);
	}

	return ret;
}

/* temp for rf pwm mode */

static int marlin_power_enable(bool enable)
{
	int ret = 0;

	pr_info("marlin_power_enable 1v5 %d\n", enable);
	if (enable) {
		gpio_direction_output(marlin_dev->reset, 0);

		regulator_set_voltage(marlin_dev->vddwcn,
					      1500000, 1500000);
		ret = regulator_enable(marlin_dev->vddwcn);
	} else {
		if (regulator_is_enabled(marlin_dev->vddwcn))
			ret = regulator_disable(marlin_dev->vddwcn);
	}

	return ret;
}
static int marlin_power_enable_1v2(bool enable)
{
	int ret = 0;

	if (marlin_dev->vdd_marlin2_1v2 != NULL) {
		pr_info("marlin_power_enable 1v2 %d\n", enable);
		if (enable) {
			regulator_set_voltage(marlin_dev->vdd_marlin2_1v2,
			1200000, 1200000);
			ret = regulator_enable(marlin_dev->vdd_marlin2_1v2);
		} else {
			if (regulator_is_enabled(marlin_dev->vdd_marlin2_1v2))
				ret =
				regulator_disable(marlin_dev->vdd_marlin2_1v2);
		}
	}

		return ret;
}

void marlin_hold_cpu(void)
{
	int ret = 0;

	ret = sprdwcn_bus_direct_read(CP_RESET_REG, &reg_val, 4);
	if (ret < 0) {
		pr_err("%s read reset reg error:%d\n", __func__, ret);
		return;
	}
	pr_info("%s reset reg val:0x%x\n", __func__, reg_val);
	reg_val |= 1;
	ret = sprdwcn_bus_direct_write(CP_RESET_REG, &reg_val, 4);
	if (ret < 0) {
		pr_err("%s write reset reg error:%d\n", __func__, ret);
		return;
	}
}

void marlin_read_cali_data(void)
{
	int err;

	pr_info("marlin sync entry is_calibrated:%d\n",
		wifi_data.cali_data.cali_config.is_calibrated);

	if (!wifi_data.cali_data.cali_config.is_calibrated) {
		memset(&wifi_data.cali_data, 0x0,
			sizeof(struct wifi_cali_t));
		err = sprdwcn_bus_direct_read(CALI_OFSET_REG,
			&wifi_data.cali_data, sizeof(struct wifi_cali_t));
		if (err < 0) {
			pr_err("marlin read cali data fail:%d\n", err);
			return;
		}

		if (!marlin_dev->power_ctl_disabled) {
			dump_cali_file(&wifi_data.cali_data);
			pr_info("marlin save cali data end\n");
		}
	}

	if ((marlin2_clk_wait_reg > 0) && (clk_wait_val > 0)) {
		err = sprdwcn_bus_direct_read(marlin2_clk_wait_reg,
					&cp_clk_wait_val, 4);
		if (err < 0) {
			pr_err("marlin read clk_wait_reg fail:%d\n", err);
			return;
		}
		pr_info("marlin2 cp_clk_wait_val is 0x%x\n", cp_clk_wait_val);
		clk_wait_val = ((clk_wait_val & 0xFF00) >> 8);
		cp_clk_wait_val =
			((cp_clk_wait_val & 0xFFFFFC00) | clk_wait_val);
		pr_info("marlin2 cp_clk_wait_val is modifyed 0x%x\n",
					cp_clk_wait_val);
		err = sprdwcn_bus_direct_write(marlin2_clk_wait_reg,
					       &cp_clk_wait_val, 4);
		if (err < 0)
			pr_err("marlin2 write 26M error:%d\n", err);
	}

	/* write this flag to notify cp that ap read calibration data */
	reg_val = 0xbbbbbbbb;
	err = sprdwcn_bus_direct_write(CALI_REG, &reg_val, 4);
	if (err < 0) {
		pr_err("marlin write cali finish error:%d\n", err);
		return;
	}

	complete(&marlin_dev->download_done);
}

static int marlin_write_cali_data(void)
{
	int  ret;

	ret = sprdwcn_bus_direct_read(WIFI_REG, &reg_val, 4);
	if (ret < 0) {
		pr_err("marlin read ahb0 fail:%d\n", ret);
		return ret;
	}
	reg_val |= (0x1<<5);
	ret = sprdwcn_bus_direct_write(WIFI_REG, &reg_val, 4);
	if (ret < 0) {
		pr_err("marlin write ahb0 error:%d\n", ret);
		return ret;
	}

	get_connectivity_config_param(&wifi_data.config_data);
	get_connectivity_cali_param(&wifi_data.cali_data);

	ret = sprdwcn_bus_direct_write(CALI_OFSET_REG, &wifi_data,
		sizeof(struct wifi_config_t) + sizeof(struct wifi_cali_t));
	if (ret < 0) {
		pr_err("marlin write cali data error:%d\n", ret);
		return ret;
	}

	/* write this flag to notify cp start calibrate */
	reg_val = 0xaaaaaaaa;
	ret = sprdwcn_bus_direct_write(CALI_REG, &reg_val, 4);
	if (ret < 0) {
		pr_err("marlin write cali tag error:%d\n", ret);
		return ret;
	}
	pr_info("marlin send cali data finish\n");

	return 0;
}

static int marlin_start_run(void)
{
	int ret = 0;

	/* set sdio higher priority to visit iram */
	ret = sprdwcn_bus_direct_read(CP_SDIO_PRIORITY_ADDR, &reg_val, 4);
	if (ret < 0) {
		pr_err("%s read sdio priority fail:%d\n", __func__, ret);
		return ret;
	}
	pr_info("%s read sdio priority value:0x%x\n", __func__, reg_val);
	reg_val |= M6_TO_S0_HIGH_PRIORITY;
	ret = sprdwcn_bus_direct_write(CP_SDIO_PRIORITY_ADDR, &reg_val, 4);
	if (ret < 0) {
		pr_err("%s write sdio priority fail:%d\n", __func__, ret);
		return ret;
	}

	ret = sprdwcn_bus_direct_read(CP_RESET_REG, &reg_val, 4);
	if (ret < 0) {
		pr_err("%s read reset reg error:%d\n", __func__, ret);
		return ret;
	}
	pr_info("%s reset reg val:0x%x\n", __func__, reg_val);
	reg_val &= (~0) - 1;
	ret = sprdwcn_bus_direct_write(CP_RESET_REG, &reg_val, 4);
	if (ret < 0) {
		pr_err("%s write reset reg error:%d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int marlin_reset(int val)
{
	if (gpio_is_valid(marlin_dev->reset)) {
		gpio_direction_output(marlin_dev->reset, 0);
		mdelay(RESET_DELAY);
		gpio_direction_output(marlin_dev->reset, 1);
	}

	return 0;
}

void marlin_chip_en(bool enable, bool reset)
{
	static unsigned int chip_en_count;

	if (gpio_is_valid(marlin_dev->chip_en)) {
		if (reset) {
			gpio_direction_output(marlin_dev->chip_en, 0);
			pr_info("marlin gnss chip en reset\n");
			msleep(100);
			gpio_direction_output(marlin_dev->chip_en, 1);
		} else if (enable) {
			if (chip_en_count == 0) {
				gpio_direction_output(marlin_dev->chip_en, 0);
				msleep(100);
				gpio_direction_output(marlin_dev->chip_en, 1);
				mdelay(1);
				pr_info("marlin chip en pull up\n");
			}
			chip_en_count++;
		} else {
			chip_en_count--;
			if (chip_en_count == 0) {
				gpio_direction_output(marlin_dev->chip_en, 0);
				pr_info("marlin chip en pull down\n");
			}
		}
	}
}
EXPORT_SYMBOL_GPL(marlin_chip_en);

static void marlin_set_module_state(void)
{
	if (test_bit(MARLIN_BLUETOOTH, &marlin_dev->power_state) ||
		test_bit(MARLIN_FM, &marlin_dev->power_state) ||
		test_bit(MARLIN_WIFI, &marlin_dev->power_state) ||
		test_bit(MARLIN_MDBG, &marlin_dev->power_state))
		wcn_open_module = 1;
	else
		wcn_open_module = 0;
	wcn_module_state_change = 1;
	open_mdbg_loopcheck_interru();
}

static void marlin_scan_finish(void)
{
	pr_info("marlin_scan_finish!\n");
	complete(&marlin_dev->carddetect_done);
}

static void marlin_download_sdio(struct work_struct *work)
{
	marlin_chip_en(true, false);
	marlin_reset(1);
	msleep(20);
	sprdwcn_bus_rescan();
	if (wait_for_completion_timeout(&marlin_dev->carddetect_done,
		msecs_to_jiffies(CARD_DETECT_WAIT_MS))) {
		if (download_firmware() == 0 &&
			marlin_start_run() == 0) {
			slp_mgr_init(marlin_dev->int_ap);
			slp_mgr_init_done = TRUE;
			marlin_write_cali_data();
			wcn_debug_init();

			return;
		}
	}
	pr_err("wait scan marlin time out\n");
}

static int marlin_set_power(int subsys, int val)
{
	unsigned long timeleft;

	mutex_lock(&marlin_dev->power_lock);
	flag1++;
	if (flag1 == 1) {
		pr_info("enter init ge2_completion");
		timeleft = wait_for_completion_timeout(&ge2_completion, 12*HZ);
		if (!timeleft)
			pr_err("wait ge2 timeout\n");
	}
	flag1 = 2;
	pr_info("marlin power state:%lx, subsys %d power %d\n",
			marlin_dev->power_state, subsys, val);
	if (val) {
		if (marlin_dev->power_state != 0) {
			set_bit(subsys, &marlin_dev->power_state);
			marlin_set_module_state();
			pr_info("********wcn_open_module:%dn", wcn_open_module);
			mutex_unlock(&marlin_dev->power_lock);
			return 0;
		}

		marlin_clk_enable(true);
		marlin_power_enable(true);
		marlin_power_enable_1v2(true);
		init_completion(&marlin_dev->download_done);
		init_completion(&marlin_dev->carddetect_done);
		sdiom_driver_register();
		schedule_work(&marlin_dev->download_wq);
		if (wait_for_completion_timeout(&marlin_dev->download_done,
			msecs_to_jiffies(POWERUP_WAIT_MS)) <= 0) {

			if (slp_mgr_init_done) {
				slp_mgr_init_done = FALSE;
				slp_mgr_exit();
			}

			sdiom_driver_unregister();
			marlin_clk_enable(false);
			marlin_power_enable(false);
			marlin_power_enable_1v2(false);
			mutex_unlock(&marlin_dev->power_lock);
			return -1;
		}
		set_bit(subsys, &marlin_dev->power_state);
		set_bit(MARLIN_ALL, &marlin_dev->power_state);
		marlin_set_module_state();
		pr_info("********wcn_open_module:%dn", wcn_open_module);
		power_state_notify(true);
		pr_info("marlin power on!\n");
	} else {
		if (marlin_dev->power_state == 0) {
			mutex_unlock(&marlin_dev->power_lock);
			return 0;
		}

		clear_bit(subsys, &marlin_dev->power_state);
		marlin_set_module_state();
		pr_err("********wcn_open_module:%dn", wcn_open_module);
		if (flag_reset)
			marlin_dev->power_state = 0;

		if ((marlin_dev->power_state
			& (~BIT(MARLIN_ALL))) != 0) {
			mutex_unlock(&marlin_dev->power_lock);
			return 0;
		}

		if (marlin_dev->power_ctl_disabled) {
			if (!flag_reset) {
				mutex_unlock(&marlin_dev->power_lock);
				pr_info("marlin reset flag_reset3:%d\n",
					flag_reset);
				return 0;
			}
		}

		if (!flag_download) {
			mutex_unlock(&marlin_dev->power_lock);
			return 0;
		}

		clear_bit(MARLIN_ALL, &marlin_dev->power_state);
		power_state_notify(false);
		if (slp_mgr_init_done) {
			slp_mgr_init_done = FALSE;
			slp_mgr_exit();
		}
		sdiom_driver_unregister();
		marlin_clk_enable(false);
		marlin_power_enable(false);
		marlin_power_enable_1v2(false);
		marlin_chip_en(false, false);
		sdiom_remove_card();
		pr_info("marlin power off!\n");
		if (flag_reset)
			flag_reset = FALSE;
	}
	mutex_unlock(&marlin_dev->power_lock);

	return 0;
}

void marlin_power_off(enum marlin_sub_sys subsys)
{
	pr_info("%s all\n", __func__);

	marlin_dev->power_ctl_disabled = false;
	set_bit(subsys, &marlin_dev->power_state);
	marlin_set_power(subsys, false);
}

int marlin_get_power(enum marlin_sub_sys subsys)
{
	if (subsys == MARLIN_ALL)
		return marlin_dev->power_state != 0;
	else
		return test_bit(subsys, &marlin_dev->power_state);
}
EXPORT_SYMBOL_GPL(marlin_get_power);

bool marlin_get_download_status(void)
{
	return flag_download;
}
EXPORT_SYMBOL_GPL(marlin_get_download_status);

int marlin_get_module_status(void)
{
	return wcn_open_module;
}
EXPORT_SYMBOL_GPL(marlin_get_module_status);

int marlin_get_module_status_changed(void)
{
	return wcn_module_state_change;
}
EXPORT_SYMBOL_GPL(marlin_get_module_status_changed);

int marlin_set_wakeup(enum marlin_sub_sys subsys)
{
	int ret = 0;

	if (unlikely(flag_download != true))
		return ERROR;

	ret = slp_mgr_wakeup(subsys);

	return ret;
}
EXPORT_SYMBOL_GPL(marlin_set_wakeup);

int marlin_set_sleep(enum marlin_sub_sys subsys, bool enable)
{
	if (unlikely(flag_download != true))
		return ERROR;

	slp_mgr_drv_sleep(subsys, enable);
	return 0;
}
EXPORT_SYMBOL_GPL(marlin_set_sleep);

int marlin_reset_reg(void)
{
	init_completion(&marlin_dev->carddetect_done);
	marlin_reset(true);
	mdelay(1);
	sprdwcn_bus_rescan();
	if (wait_for_completion_timeout(&marlin_dev->carddetect_done,
		msecs_to_jiffies(POWERUP_WAIT_MS))) {
		return 0;
	}
	pr_err("marlin reset reg wait scan error!\n");

	return -1;
}
EXPORT_SYMBOL_GPL(marlin_reset_reg);

int start_marlin(u32 subsys)
{
	if (sprdwcn_bus_get_carddump_status() != 0)
		return 0;

	if (get_loopcheck_status())
		return 0;

	return marlin_set_power(subsys, true);
}
EXPORT_SYMBOL_GPL(start_marlin);

int stop_marlin(u32 subsys)
{
	if (sprdwcn_bus_get_carddump_status() != 0)
		return 0;

	if (get_loopcheck_status())
		return 0;

	return marlin_set_power(subsys, false);
}
EXPORT_SYMBOL_GPL(stop_marlin);

static void marlin_power_wq(struct work_struct *work)
{
	pr_info("%s start\n", __func__);
	marlin_set_power(MARLIN_ALL, true);
	if (marlin_dev->power_ctl_disabled) {
		msleep(30000);
		dump_cali_file(&wifi_data.cali_data);
		pr_info("marlin power_wq save cali data end\n");
	}
}

static int marlin_probe(struct platform_device *pdev)
{
	struct device_node *regmap_np;
	struct platform_device *pdev_regmap;

	marlin_dev = devm_kzalloc(&pdev->dev,
			sizeof(struct marlin_device), GFP_KERNEL);
	if (!marlin_dev)
		return -ENOMEM;
	marlin_dev->write_buffer = devm_kzalloc(&pdev->dev,
			PACKET_SIZE, GFP_KERNEL);
	if (marlin_dev->write_buffer == NULL) {
		devm_kfree(&pdev->dev, marlin_dev);
		pr_err("marlin_probe write buffer low memory\n");
		return -ENOMEM;
	}
	mutex_init(&(marlin_dev->power_lock));
	marlin_dev->power_state = 0;
	if (marlin_parse_dt(pdev) < 0)
		pr_info("marlin2 parse_dt some para not config\n");
	if (gpio_is_valid(marlin_dev->reset))
		gpio_direction_output(marlin_dev->reset, 0);
	init_completion(&ge2_completion);
	init_completion(&marlin_dev->carddetect_done);

	/* register ops */
	wcn_bus_init();
	/* sdiom_init or pcie_init */
	sprdwcn_bus_preinit();
	sprdwcn_bus_register_rescan_cb(marlin_scan_finish);

	mdbg_init();
	wcn_op_init();

	INIT_WORK(&marlin_dev->download_wq, marlin_download_sdio);
	if (marlin_dev->power_ctl_disabled) {
		INIT_DELAYED_WORK(&marlin_dev->power_wq, marlin_power_wq);
		schedule_delayed_work(&marlin_dev->power_wq,
				msecs_to_jiffies(5000));
	}

	/* temp for rf pwm mode */
	regmap_np = of_find_compatible_node(NULL, NULL, "sprd,pmic-glb");
	if (!regmap_np)
		return -ENOMEM;

	pdev_regmap = of_find_device_by_node(regmap_np);
	if (!pdev_regmap) {
		of_node_put(regmap_np);
		return -ENOMEM;
	}
	pwm_regmap = dev_get_regmap(pdev_regmap->dev.parent, NULL);
	if (!pwm_regmap) {
		pr_info("pwm regmap error\n");
		of_node_put(regmap_np);
		return PTR_ERR(pwm_regmap);
	}
	of_node_put(regmap_np);
	pr_info("marlin_probe ok!\n");

	return 0;
}

static int  marlin_remove(struct platform_device *pdev)
{
	mutex_lock(&marlin_dev->power_lock);
	if (marlin_dev->power_state != 0) {
		pr_err("marlin some subsys power is on, warning!\n");
		mutex_unlock(&marlin_dev->power_lock);
		return -1;
	}
	mutex_unlock(&marlin_dev->power_lock);
	cancel_work_sync(&marlin_dev->download_wq);
	cancel_delayed_work_sync(&marlin_dev->power_wq);
	gpio_free(marlin_dev->reset);
	gpio_free(marlin_dev->chip_en);
	gpio_free(marlin_dev->int_ap);
	mdbg_exit();
	wcn_op_exit();
	sdiom_exit();
	wcn_bus_deinit();
	kfree(marlin_dev->write_buffer);
	kfree(marlin_dev);

	return 0;
}

static int marlin_suspend(struct device *dev)
{

	pr_info("[%s]enter\n", __func__);
	pr_info("[%s]ok\n", __func__);

	return 0;
}

int marlin_reset_register_notify(void *callback_func, void *para)
{
	marlin_reset_func = (marlin_reset_callback)callback_func;
	marlin_callback_para = para;

	return 0;
}
EXPORT_SYMBOL_GPL(marlin_reset_register_notify);

int marlin_reset_unregister_notify(void)
{
	marlin_reset_func = NULL;
	marlin_callback_para = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(marlin_reset_unregister_notify);

static int marlin_resume(struct device *dev)
{
	pr_info("[%s]enter\n", __func__);
	pr_info("[%s]ok\n", __func__);

	return 0;
}

static const struct dev_pm_ops marlin_pm_ops = {
	.suspend = marlin_suspend,
	.resume	= marlin_resume,
};

static const struct of_device_id marlin_match_table[] = {
	{ .compatible = "sprd,marlin2", },
	{ },
};

static struct platform_driver marlin_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "marlin",
		.pm = &marlin_pm_ops,
		.of_match_table = marlin_match_table,
	},
	.probe = marlin_probe,
	.remove = marlin_remove,
};

static int __init marlin_init(void)
{
	pr_info("marlin_init entry!\n");

	return platform_driver_register(&marlin_driver);
}
late_initcall(marlin_init);

static void __exit marlin_exit(void)
{
	platform_driver_unregister(&marlin_driver);
}
module_exit(marlin_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Spreadtrum  WCN Marlin Driver");
MODULE_AUTHOR("Yufeng Yang <yufeng.yang@spreadtrum.com>");
