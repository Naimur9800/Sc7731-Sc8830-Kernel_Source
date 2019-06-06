/*
* Copyright (C) 2017 Spreadtrum Communications Inc.
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/

#include <linux/delay.h>
#include <linux/gnss.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/marlin_platform.h>
#include <linux/printk.h>
#include <linux/kthread.h>
#include <linux/sdiom_rx_api.h>
#include <linux/sdiom_tx_api.h>
#include <linux/sipc.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/mfd/syscon/sprd-glb.h>
#include <linux/unistd.h>
#include <linux/wait.h>
#include <linux/wcn_integrate_platform.h>

#include <mdbg_type.h>

#include "gnss_dump.h"

struct regmap_dump {
	int regmap_type;
	uint reg;
};

/* for sharkle ap reg dump, this order format can't change, just do it */
static struct regmap_dump gnss_sharkle_ap_reg[] = {
	{REGMAP_PMU_APB, 0x00cc}, /* REG_PMU_APB_SLEEP_CTRL */
	{REGMAP_PMU_APB, 0x00d4}, /* REG_PMU_APB_SLEEP_STATUS */
	{REGMAP_AON_APB, 0x057c}, /* REG_AON_APB_WCN_SYS_CFG2 */
	{REGMAP_AON_APB, 0x0578}, /* REG_AON_APB_WCN_SYS_CFG1 */
	{REGMAP_TYPE_NR, (AON_CLK_CORE + CGM_WCN_SHARKLE_CFG)}, /* clk */
	{REGMAP_PMU_APB, 0x0100}, /* REG_PMU_APB_PD_WCN_SYS_CFG */
	{REGMAP_PMU_APB, 0x0104}, /* REG_PMU_APB_PD_WIFI_WRAP_CFG */
	{REGMAP_PMU_APB, 0x0108}, /* REG_PMU_APB_PD_GNSS_WRAP_CFG */
};

/* for pike2 ap reg dump, this order format can't change, just do it */
static struct regmap_dump gnss_pike2_ap_reg[] = {
	{REGMAP_PMU_APB, 0x00cc}, /* REG_PMU_APB_SLEEP_CTRL */
	{REGMAP_PMU_APB, 0x00d4}, /* REG_PMU_APB_SLEEP_STATUS */
	{REGMAP_PMU_APB, 0x0338}, /* REG_PMU_APB_WCN_SYS_CFG_STATUS */
	{REGMAP_AON_APB, 0x00d8}, /* REG_AON_APB_WCN_CONFIG0 */
	{REGMAP_TYPE_NR, (AON_CLK_CORE + CGM_WCN_PIKE2_CFG)}, /* clk */
	{REGMAP_PMU_APB, 0x0050}, /* REG_PMU_APB_PD_WCN_TOP_CFG */
	{REGMAP_PMU_APB, 0x0054}, /* REG_PMU_APB_PD_WCN_WIFI_CFG */
	{REGMAP_PMU_APB, 0x0058}, /* REG_PMU_APB_PD_WCN_GNSS_CFG */
};
#define GNSS_DUMP_REG_NUMBER 8

static struct file *dump_file;
static  loff_t pos;
#define GNSS_MEMDUMP_PATH           "/data/gnss/gnssdump.mem"
static char gnss_dump_level; /* 0: default, all, 1: only data, pmu, aon */

static void gnss_write_data_to_phy_addr(phys_addr_t phy_addr,
					void *src_data, u32 size)
{
	void *virt_addr;

	pr_info("gnss_write_data_to_phy_addr entry\n");
	virt_addr = shmem_ram_vmap_nocache(phy_addr, size);
	if (virt_addr) {
		memcpy(virt_addr, src_data, size);
		shmem_ram_unmap(virt_addr);
	} else
		pr_err("%s shmem_ram_vmap_nocache fail\n", __func__);
}

static void gnss_read_data_from_phy_addr(phys_addr_t phy_addr,
					 void *tar_data, u32 size)
{
	void *virt_addr;

	pr_info("gnss_read_data_from_phy_addr\n");
	virt_addr = shmem_ram_vmap_nocache(phy_addr, size);
	if (virt_addr) {
		memcpy(tar_data, virt_addr, size);
		shmem_ram_unmap(virt_addr);
	} else
		pr_err("%s shmem_ram_vmap_nocache fail\n", __func__);
}

static void gnss_hold_cpu(void)
{
	struct regmap *regmap;
	u32 value;
	phys_addr_t base_addr;
	int i = 0;

	pr_info("gnss_hold_cpu entry\n");

	if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_SHARKL3)
		regmap = wcn_get_gnss_regmap(REGMAP_WCN_REG);
	else
		regmap = wcn_get_gnss_regmap(REGMAP_ANLG_WRAP_WCN);
	wcn_regmap_read(regmap, 0X20, &value);
	value |= 1 << 2;
	wcn_regmap_raw_write_bit(regmap, 0X20, value);

	wcn_regmap_read(regmap, 0X24, &value);
	value |= 1 << 3;
	wcn_regmap_raw_write_bit(regmap, 0X24, value);

	value = GNSS_CACHE_FLAG_VALUE;
	base_addr = wcn_get_gnss_base_addr();
	gnss_write_data_to_phy_addr(base_addr + GNSS_CACHE_FLAG_ADDR,
		(void *)&value, 4);

	value = 0;
	wcn_regmap_raw_write_bit(regmap, 0X20, value);
	wcn_regmap_raw_write_bit(regmap, 0X24, value);
	while (i < 3) {
		gnss_read_data_from_phy_addr(base_addr + GNSS_CACHE_FLAG_ADDR,
			(void *)&value, 4);
		if (value == GNSS_CACHE_END_VALUE)
			break;
		i++;
		msleep(50);
	}
	if (value != GNSS_CACHE_END_VALUE)
		pr_err("%s gnss cache failed value %d\n", __func__, value);
	msleep(200);
}

static int gnss_dump_cp_register_data(u32 addr, u32 len)
{
	struct regmap *regmap;
	u32 i;
	u8 *buf = NULL;
	u8 *ptr = NULL;
	long int ret;
	void  *iram_buffer = NULL;
	mm_segment_t fs;

	pr_info(" start dump cp register!addr:%x,len:%d\n", addr, len);
	buf = kzalloc(len, GFP_KERNEL);
	if (!buf) {
		pr_err("%s kzalloc buf error\n", __func__);
		return -ENOMEM;
	}

	if (IS_ERR(dump_file)) {
		dump_file = filp_open(GNSS_MEMDUMP_PATH,
			O_RDWR | O_CREAT | O_APPEND, 0666);
		if (IS_ERR(dump_file)) {
			pr_err("%s open file mem error\n", __func__);
			kfree(buf);
			return PTR_ERR(dump_file);
		}
	}

	iram_buffer = vmalloc(len);
	if (!iram_buffer) {
		kfree(buf);
		return -ENOMEM;
	}

	/* can't op cp reg when level is 1, just record 0 to buffer */
	memset(iram_buffer, 0, len);
	if (gnss_dump_level == 0) {
		if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_SHARKL3)
			regmap = wcn_get_gnss_regmap(REGMAP_WCN_REG);
		else
			regmap = wcn_get_gnss_regmap(REGMAP_ANLG_WRAP_WCN);
		wcn_regmap_raw_write_bit(regmap, ANLG_WCN_WRITE_ADDR, addr);
		for (i = 0; i < len / 4; i++) {
			ptr = buf + i * 4;
			wcn_regmap_read(regmap, ANLG_WCN_READ_ADDR, (u32 *)ptr);
		}
		memcpy(iram_buffer, buf, len);
	}
	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = dump_file->f_pos;
	ret = vfs_write(dump_file, iram_buffer, len, &pos);
	dump_file->f_pos = pos;
	kfree(buf);
	vfree(iram_buffer);
	set_fs(fs);
	if (ret != len) {
		pr_err("gnss_dump_cp_register_data failed  size is %ld\n", ret);
		return -1;
	}
	 pr_info("gnss_dump_cp_register_data finish  size is  %ld\n", ret);

	return ret;
}


static int gnss_dump_ap_register(void)
{
	struct regmap *regmap;
	u32 value[GNSS_DUMP_REG_NUMBER + 1] = {0}; /* [0]board+ [..]reg */
	u32 i = 0;
	mm_segment_t fs;
	u32 len = 0;
	u8 *ptr = NULL;
	int ret;
	void  *apreg_buffer = NULL;
	struct regmap_dump *gnss_ap_reg = NULL;

	pr_info("%s ap reg data\n", __func__);
	if (IS_ERR(dump_file)) {
		dump_file = filp_open(GNSS_MEMDUMP_PATH,
			O_RDWR | O_CREAT | O_APPEND, 0666);
		if (IS_ERR(dump_file)) {
			pr_err("%s	open file mem error\n", __func__);
			return -1;
		}
	}

	if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_SHARKLE) {
		gnss_ap_reg = gnss_sharkle_ap_reg;
		len = (GNSS_DUMP_REG_NUMBER + 1) * sizeof(u32);
	} else {
		gnss_ap_reg = gnss_pike2_ap_reg;
		len = (GNSS_DUMP_REG_NUMBER + 1) * sizeof(u32);
	}

	apreg_buffer = vmalloc(len);
	if (!apreg_buffer)
		return -2;

	ptr = (u8 *)&value[0];
	if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_SHARKLE)
		value[0] = 0xF1;
	else
		value[0] = 0xF2;

	for (i = 0; i < GNSS_DUMP_REG_NUMBER; i++) {
		if ((gnss_ap_reg + i)->regmap_type == REGMAP_TYPE_NR) {
			gnss_read_data_from_phy_addr((gnss_ap_reg + i)->reg,
				&value[i+1], 4);
		} else {
			regmap =
			wcn_get_gnss_regmap((gnss_ap_reg + i)->regmap_type);
			wcn_regmap_read(regmap, (gnss_ap_reg + i)->reg,
				&value[i+1]);
		}
	}
	memset(apreg_buffer, 0, len);
	memcpy(apreg_buffer, ptr, len);
	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = dump_file->f_pos;
	ret = vfs_write(dump_file, apreg_buffer, len, &pos);
	dump_file->f_pos = pos;
	vfree(apreg_buffer);
	set_fs(fs);
	if (ret != len)
		pr_err("%s not write completely,ret is 0x%x\n", __func__, ret);
	return 0;
}

static void gnss_dump_cp_register(void)
{
	u32 count;

	count = gnss_dump_cp_register_data(DUMP_REG_GNSS_APB_CTRL_ADDR,
			DUMP_REG_GNSS_APB_CTRL_LEN);
	pr_info("gnss dump gnss_apb_ctrl_reg %u ok!\n", count);

	count = gnss_dump_cp_register_data(DUMP_REG_GNSS_AHB_CTRL_ADDR,
			DUMP_REG_GNSS_AHB_CTRL_LEN);
	pr_info("gnss dump manu_clk_ctrl_reg %u ok!\n", count);

	count = gnss_dump_cp_register_data(DUMP_COM_SYS_CTRL_ADDR,
			DUMP_COM_SYS_CTRL_LEN);
	pr_info("gnss dump com_sys_ctrl_reg %u ok!\n", count);

	count = gnss_dump_cp_register_data(DUMP_WCN_CP_CLK_CORE_ADDR,
			DUMP_WCN_CP_CLK_LEN);
	pr_info("gnss dump manu_clk_ctrl_reg %u ok!\n", count);
}

static void gnss_dump_register(void)
{
	gnss_dump_ap_register();
	gnss_dump_cp_register();
	pr_info("gnss dump register ok!\n");
}

static void gnss_dump_iram(void)
{
	u32 count;

	count = gnss_dump_cp_register_data(DUMP_IRAM_START_ADDR,
			GNSS_CP_IRAM_DATA_NUM * 4);
	pr_info("gnss dump iram %u ok!\n", count);
}

static int gnss_dump_share_memory(u32 len)
{
	void *virt_addr;
	phys_addr_t base_addr;
	long int ret;
	mm_segment_t fs;
	void  *ddr_buffer = NULL;

	if (len == 0)
		return -1;
	pr_info("gnss_dump_share_memory\n");
	fs = get_fs();
	set_fs(KERNEL_DS);
	base_addr = wcn_get_gnss_base_addr();
	virt_addr = shmem_ram_vmap_nocache(base_addr, len);
	if (!virt_addr) {
		pr_err(" %s shmem_ram_vmap_nocache fail\n", __func__);
		return -1;
	}

	if (IS_ERR(dump_file)) {
		dump_file = filp_open(GNSS_MEMDUMP_PATH,
			O_RDWR | O_CREAT | O_APPEND, 0666);
		if (IS_ERR(dump_file)) {
			pr_err("%s open file mem error\n", __func__);
			return PTR_ERR(dump_file);
		}
	}

	ddr_buffer = vmalloc(len);
	if (!ddr_buffer)
		return -1;

	memset(ddr_buffer, 0, len);
	memcpy(ddr_buffer, virt_addr, len);
	pos = dump_file->f_pos;
	ret = vfs_write(dump_file, ddr_buffer, len, &pos);
	dump_file->f_pos = pos;
	shmem_ram_unmap(virt_addr);
	set_fs(fs);
	vfree(ddr_buffer);
	if (ret != len) {
		pr_err("%s dump ddr error,data len is %ld\n", __func__, ret);
		return -1;
	}

	pr_info("gnss dump share memory  size = %ld\n", ret);

	return 0;
}

static int gnss_creat_dump_file(void)
{
	dump_file = filp_open(GNSS_MEMDUMP_PATH,
		O_RDWR | O_CREAT | O_TRUNC, 0666);
	pr_info("gnss_creat_dump_file entry\n");
	if (IS_ERR(dump_file)) {
		pr_err("%s open mem dump file error dump_file is %p\n",
			__func__, dump_file);
		return -1;
	}
	if (sys_chmod(GNSS_MEMDUMP_PATH, 0666) != 0)
		pr_err("%s chmod  error\n", __func__);

	return 0;
}

int gnss_dump_mem(char flag)
{
	int ret = 0;

	gnss_dump_level = flag;
	pr_info("gnss_dump_mem entry\n");
	gnss_hold_cpu();
	ret = gnss_creat_dump_file();
	if (ret == -1) {
		pr_err("%s create mem dump file  error\n", __func__);
		return -1;
	}
	ret = gnss_dump_share_memory(GNSS_SHARE_MEMORY_SIZE);
	gnss_dump_iram();
	gnss_dump_register();
	if (dump_file != NULL)
		filp_close(dump_file, NULL);

	return ret;
}
