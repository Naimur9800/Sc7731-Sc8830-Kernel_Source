/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
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
#include <linux/file.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mfd/sprd/pmic_glb_reg.h>
#include <linux/mfd/syscon.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#include "linux/sipc.h"
#include "linux/sprd_otp.h"
#include "linux/wcn_integrate_platform.h"
#include "linux/gnss.h"
#include "gnss_firmware_bin.h"
#include "marlin_firmware_bin.h"
#include "mdbg_common.h"
#include "mdbg_main.h"
#include "rf/rf.h"
#include "wcn_misc.h"
#include "wcn_parn_parser.h"

#define WCN_BTWF_FILENAME "wcnmodem"
#define WCN_GNSS_FILENAME "gpsgl"
#define WCN_GNSS_BD_FILENAME "gpsbd"

/* NOTES:If DTS config more than REG_CTRL_CNT_MAX REGs */
#define REG_CTRL_CNT_MAX 8
/* NOTES:If DTS config more than REG_SHUTDOWN_CNT_MAX REGs */
#define REG_SHUTDOWN_CNT_MAX 4

#define WCN_INTEGRATE_PLATFORM_DEBUG 0
#define SUSPEND_RESUME_ENABLE 0
#define REGMAP_UPDATE_BITS_ENABLE 0	/* It can't work well. */

#define WCN_OPEN_MAX_CNT (0x10)

/* default VDDCON voltage is 1.6v, work voltage is 1.2v */
#define WCN_VDDCON_WORK_VOLTAGE (1200000)
/* default VDDCON voltage is 3.3v, work voltage is 3.0v */
#define WCN_VDDWIFIPA_WORK_VOLTAGE (3000000)

#define WCN_PROC_FILE_LENGTH_MAX (63)

#define WCN_CP_SOFT_RST_MIN_TIME (5000)	/* us */
#define WCN_CP_SOFT_RST_MAX_TIME (6000)	/* us */

struct mutex marlin_lock;
static int start_integrate_wcn_truely(u32 subsys);
static int stop_integrate_wcn_truely(u32 subsys);

#if WCN_INTEGRATE_PLATFORM_DEBUG
enum wcn_integrate_platform_debug_case {
	NORMAL_CASE = 0,
	WCN_START_MARLIN_DEBUG,
	WCN_STOP_MARLIN_DEBUG,
	WCN_START_MARLIN_DDR_FIRMWARE_DEBUG,
	/* Next for GNSS */
	WCN_START_GNSS_DEBUG,
	WCN_STOP_GNSS_DEBUG,
	WCN_START_GNSS_DDR_FIRMWARE_DEBUG,
	/* Print Info */
	WCN_PRINT_INFO,
	WCN_BRINGUP_DEBUG,
};
#endif

int wcn_open_module;
int wcn_module_state_change;
unsigned char flag_download;
char functionmask[8];
marlin_reset_callback marlin_reset_func;
void *marlin_callback_para;
static struct platform_chip_id g_platform_chip_id;
static u32 g_platform_chip_type;
struct wcn_device;
struct platform_proc_file_entry {
	char			*name;
	struct proc_dir_entry	*platform_proc_dir_entry;
	struct wcn_device	*wcn_dev;
	unsigned		flag;
};

#define MAX_PLATFORM_ENTRY_NUM		0x10
enum {
	BE_SEGMFG   = (0x1 << 4),
	BE_RDONLY   = (0x1 << 5),
	BE_WRONLY   = (0x1 << 6),
	BE_CPDUMP   = (0x1 << 7),
	BE_MNDUMP   = (0x1 << 8),
	BE_RDWDT    = (0x1 << 9),
	BE_RDWDTS   = (0x1 << 10),
	BE_RDLDIF   = (0x1 << 11),
	BE_LD	    = (0x1 << 12),
	BE_CTRL_ON  = (0x1 << 13),
	BE_CTRL_OFF	= (0x1 << 14),
};

enum {
	CP_NORMAL_STATUS = 0,
	CP_STOP_STATUS,
	CP_MAX_STATUS,
};

struct wcn_platform_fs {
	struct proc_dir_entry		*platform_proc_dir_entry;
	struct platform_proc_file_entry entrys[MAX_PLATFORM_ENTRY_NUM];
};

struct wcn_proc_data {
	int (*start)(void *arg);
	int (*stop)(void *arg);
};

struct wcn_init_data {
	char		*devname;
	phys_addr_t	base;		/* CP base addr */
	u32		maxsz;		/* CP max size */
	int		(*start)(void *arg);
	int		(*stop)(void *arg);
	int		(*suspend)(void *arg);
	int		(*resume)(void *arg);
	int		type;
};

/* CHIP if include GNSS */
#define WCN_INTERNAL_INCLUD_GNSS_VAL (0)
#define WCN_INTERNAL_NOINCLUD_GNSS_VAL (0xab520)

/* WIFI cali */
#define WIFI_CALIBRATION_FLAG_VALUE	(0xefeffefe)
#define WIFI_CALIBRATION_FLAG_CLEAR_VALUE	(0x12345678)
/*GNSS cali*/
#define GNSS_CALIBRATION_FLAG_CLEAR_ADDR (0x00150028)
#define GNSS_CALIBRATION_FLAG_CLEAR_ADDR_CP \
	(GNSS_CALIBRATION_FLAG_CLEAR_ADDR + 0x300000)
#define GNSS_CALIBRATION_FLAG_CLEAR_VALUE (0)
#define GNSS_WAIT_CP_INIT_COUNT	(256)
#define GNSS_CALI_DONE_FLAG (0x1314520)
#define GNSS_WAIT_CP_INIT_POLL_TIME_MS	(20)	/* 20ms */

struct wifi_calibration {
	struct wifi_config_t config_data;
	struct wifi_cali_t cali_data;
};
static struct wifi_calibration wifi_data;

struct wcn_chip_type {
	u32 chipid;
	enum wcn_aon_chip_id chiptype;
};
/* wifi efuse data, default value comes from PHY team */
#define WIFI_EFUSE_BLOCK_COUNT (3)
static const u32
s_wifi_efuse_id[WCN_PLATFORM_TYPE][WIFI_EFUSE_BLOCK_COUNT] = {
	{41, 42, 43},	/* SharkLE */
	{38, 39, 40},	/* PIKE2 */
	{41, 42, 43},	/* SharkL3 */
};
static const u32 s_wifi_efuse_default_value[WIFI_EFUSE_BLOCK_COUNT] = {
0x11111111, 0x22222222, 0x33333333};	/* Until now, the value is error */

static const struct wcn_chip_type wcn_chip_type[] = {
	{0x96360000, WCN_SHARKLE_CHIP_AA_OR_AB},
	{0x96360002, WCN_SHARKLE_CHIP_AC},
	{0x96360003, WCN_SHARKLE_CHIP_AD},
	/* WCN_PIKE2_CHIP_AA and WCN_PIKE2_CHIP_AB is the same */
	{0x96330000, WCN_PIKE2_CHIP},
};

#define MARLIN_CP_INIT_READY_MAGIC	(0xababbaba)
#define MARLIN_CP_INIT_START_MAGIC	(0x5a5a5a5a)
#define MARLIN_CP_INIT_SUCCESS_MAGIC	(0x13579bdf)
#define MARLIN_CP_INIT_FALIED_MAGIC	(0x88888888)

#define MARLIN_USE_FORCE_SHUTDOWN	(0xabcd250)
#define MARLIN_FORCE_SHUTDOWN_OK	(0x6B6B6B6B)

#define MARLIN_WAIT_CP_INIT_POLL_TIME_MS	(20)	/* 20ms */
#define MARLIN_WAIT_CP_INIT_COUNT	(256)
#define MARLIN_WAIT_CP_INIT_MAX_TIME (20000)
#define WCN_WAIT_SLEEP_MAX_COUNT (32)
static char gnss_firmware_parent_path[FIRMWARE_FILEPATHNAME_LENGTH_MAX];
static char firmware_file_name[FIRMWARE_FILEPATHNAME_LENGTH_MAX];

/* begin : for gnss module */

/* record efuse, GNSS_EFUSE_DATA_OFFSET is defined in gnss.h */
#define GNSS_EFUSE_BLOCK_COUNT (3)
static const u32 s_gnss_efuse_id[GNSS_EFUSE_BLOCK_COUNT] = {40, 42, 43};

#define WCN_SPECIAL_SHARME_MEM_ADDR	(0x0017c000)
struct wifi_special_share_mem {
	struct wifi_calibration calibration_data;
	u32 efuse[WIFI_EFUSE_BLOCK_COUNT];
	u32 calibration_flag;
};

struct marlin_special_share_mem {
	u32 init_status;
	u32 loopcheck_cnt;
};

struct gnss_special_share_mem {
	u32 calibration_flag;
	u32 efuse[GNSS_EFUSE_BLOCK_COUNT];
};

struct wcn_special_share_mem {
	/* 0x17c000 */
	struct wifi_special_share_mem wifi;
	/* 0x17cf54 */
	struct marlin_special_share_mem marlin;
	/* 0x17cf5c */
	u32 include_gnss;
	/* 0x17cf60 */
	u32 gnss_flag_addr;
	/* 0x17cf64 */
	u32 cp2_sleep_status;
	/* 0x17cf68 */
	u32 sleep_flag_addr;
	/* 0x17cf6c */
	u32 efuse_temper_magic;
	/* 0x17cf70 */
	u32 efuse_temper_val;
	/* 0x17cf74 */
	struct gnss_special_share_mem gnss;
};

static struct wcn_special_share_mem *s_wssm_phy_offset_p =
	(struct wcn_special_share_mem *)WCN_SPECIAL_SHARME_MEM_ADDR;

#define WCN_BOOT_CP2_OK 0
#define WCN_BOOT_CP2_ERR_DOWN_IMG 1
#define WCN_BOOT_CP2_ERR_BOOT 2
/* end: for gnss */
struct wcn_device {
	char	*name;
	/* DTS info: */

	/*
	 * wcn and gnss ctrl_reg num
	 * from ctrl-reg[0] to ctrl-reg[ctrl-probe-num - 1]
	 * need init in the driver probe stage
	 */
	u32	ctrl_probe_num;
	u32	ctrl_reg[REG_CTRL_CNT_MAX]; /* offset */
	u32	ctrl_mask[REG_CTRL_CNT_MAX];
	u32	ctrl_value[REG_CTRL_CNT_MAX];
	/*
	 * Some REGs Read and Write has about 0x1000 offset;
	 * REG_write - REG_read=0x1000, the DTS value is write value
	 */
	u32	ctrl_rw_offset[REG_CTRL_CNT_MAX];
	u32	ctrl_us_delay[REG_CTRL_CNT_MAX];
	u32	ctrl_type[REG_CTRL_CNT_MAX]; /* the value is pmu or apb */
	struct	regmap *rmap[REGMAP_TYPE_NR];
	u32	reg_nr;
	/* Shut down group */
	u32	ctrl_shutdown_reg[REG_SHUTDOWN_CNT_MAX];
	u32	ctrl_shutdown_mask[REG_SHUTDOWN_CNT_MAX];
	u32	ctrl_shutdown_value[REG_SHUTDOWN_CNT_MAX];
	u32	ctrl_shutdown_rw_offset[REG_SHUTDOWN_CNT_MAX];
	u32	ctrl_shutdown_us_delay[REG_SHUTDOWN_CNT_MAX];
	u32	ctrl_shutdown_type[REG_SHUTDOWN_CNT_MAX];
	/* struct regmap *rmap_shutdown[REGMAP_TYPE_NR]; */
	u32	reg_shutdown_nr;	/* REG_SHUTDOWN_CNT_MAX */
	phys_addr_t base_addr;
	char	firmware_path[FIRMWARE_FILEPATHNAME_LENGTH_MAX];
	char	firmware_path_ext[FIRMWARE_FILEPATHNAME_LENGTH_MAX];
	u32	file_length;
	/* FS OPS info: */
	struct	wcn_platform_fs platform_fs;
	int	status;
	u32	wcn_open_status;	/* marlin or gnss subsys status */
	u32	boot_cp_status;
	/* driver OPS */
	int	(*start)(void *arg);
	int	(*stop)(void *arg);
	u32	maxsz;
	struct	mutex power_lock;
	u32	power_state;
	struct regulator *vddwifipa;
	struct mutex vddwifipa_lock;
	char	*write_buffer;
	struct	delayed_work power_wq;
	struct	work_struct load_wq;
	struct	delayed_work cali_wq;
	struct	completion download_done;
};

struct wcn_device_manage {
	struct wcn_device *btwf_device;
	struct wcn_device *gnss_device;
	struct regulator *vddwcn;
	struct mutex vddwcn_lock;
	int vddwcn_en_count;
	int gnss_type;
	bool vddcon_voltage_setted;
	bool btwf_calibrated;
};
static struct wcn_device_manage s_wcn_device;

static inline bool wcn_dev_is_marlin(struct wcn_device *dev)
{
	return dev == s_wcn_device.btwf_device;
}

static inline bool wcn_dev_is_gnss(struct wcn_device *dev)
{
	return dev == s_wcn_device.gnss_device;
}

#define WCN_VMAP_RETRY_CNT (20)
static void *wcn_mem_ram_vmap(phys_addr_t start, size_t size,
			      int noncached, unsigned int *count)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;
	phys_addr_t addr;
	int retry = 0;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);
	*count = page_count;
	if (noncached)
		prot = pgprot_noncached(PAGE_KERNEL);
	else
		prot = PAGE_KERNEL;
retry1:
	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		if (retry++ < WCN_VMAP_RETRY_CNT) {
			usleep_range(8000, 10000);
			goto retry1;
		} else {
			WCN_ERR("malloc err\n");
			return NULL;
		}
	}

	for (i = 0; i < page_count; i++) {
		addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
retry2:
	vaddr = vm_map_ram(pages, page_count, -1, prot);
	if (!vaddr) {
		if (retry++ < WCN_VMAP_RETRY_CNT) {
			usleep_range(8000, 10000);
			goto retry2;
		} else {
			WCN_ERR("vmap err\n");
			goto out;
		}
	} else
		vaddr += offset_in_page(start);
out:
	kfree(pages);

	return vaddr;
}

void wcn_mem_ram_unmap(const void *mem, unsigned int count)
{
	vm_unmap_ram(mem - offset_in_page(mem), count);
}

void *wcn_mem_ram_vmap_nocache(phys_addr_t start, size_t size,
			       unsigned int *count)
{
	return wcn_mem_ram_vmap(start, size, 1, count);
}

int wcn_write_data_to_phy_addr(phys_addr_t phy_addr,
			       void *src_data, u32 size)
{
	int i;
	char *virt_addr, *src;
	unsigned int cnt;

	src = (char *)src_data;
	virt_addr = (char *)wcn_mem_ram_vmap_nocache(phy_addr, size, &cnt);
	if (virt_addr) {
		/* crash so remove the memcpy */
		for (i = 0; i < size; i++)
			virt_addr[i] = src[i];
		wcn_mem_ram_unmap(virt_addr, cnt);
		return 0;
	}

	WCN_ERR("wcn_mem_ram_vmap_nocache fail\n");
	return -1;
}

int wcn_read_data_from_phy_addr(phys_addr_t phy_addr,
				void *tar_data, u32 size)
{
	int i;
	char *virt_addr, *tar;
	unsigned int cnt;

	tar = (char *)tar_data;
	virt_addr = wcn_mem_ram_vmap_nocache(phy_addr, size, &cnt);
	if (virt_addr) {
		/* crash so remove the memcpy */
		for (i = 0; i < size; i++)
			tar[i] = virt_addr[i];
		wcn_mem_ram_unmap(virt_addr, cnt);
		return 0;
	}

	WCN_ERR("wcn_mem_ram_vmap_nocache fail\n");
	return -1;
}

enum wcn_aon_chip_id wcn_get_aon_chip_id(void)
{
	u32 aon_chip_id;
	u32 version_id;
	int i;
	struct regmap *regmap;

	if (unlikely(!s_wcn_device.btwf_device))
		return WCN_AON_CHIP_ID_INVALID;

	regmap = wcn_get_btwf_regmap(REGMAP_AON_APB);
	wcn_regmap_read(regmap, WCN_AON_CHIP_ID, &aon_chip_id);
	WCN_INFO("aon_chip_id=0x%08x\n", aon_chip_id);
	for (i = 0; i < ARRAY_SIZE(wcn_chip_type); i++) {
		if (wcn_chip_type[i].chipid == aon_chip_id) {
			if (wcn_chip_type[i].chiptype != WCN_PIKE2_CHIP)
				return wcn_chip_type[i].chiptype;
			wcn_regmap_read(regmap, WCN_AON_VERSION_ID,
					&version_id);
			WCN_INFO("aon_version_id=0x%08x\n", version_id);
			/* version_id:
			 * 0 for WCN_PIKE2_CHIP_AA
			 * others for WCN_PIKE2_CHIP_AB
			 */
			return (version_id == 0) ?
			       WCN_PIKE2_CHIP_AA : WCN_PIKE2_CHIP_AB;
		}
	}

	return WCN_AON_CHIP_ID_INVALID;
}
EXPORT_SYMBOL_GPL(wcn_get_aon_chip_id);

u32 wcn_platform_chip_id(void)
{
	return g_platform_chip_id.aon_chip_id;
}

u32 wcn_platform_chip_type(void)
{
	return g_platform_chip_type;
}

u32 wcn_get_cp2_comm_rx_count(void)
{
	u32 rx_count;
	phys_addr_t phy_addr;

	phy_addr = s_wcn_device.btwf_device->base_addr +
		   (phys_addr_t)&s_wssm_phy_offset_p->marlin.loopcheck_cnt;
	wcn_read_data_from_phy_addr(phy_addr,
				    &rx_count, sizeof(u32));
	WCN_INFO("cp2 comm rx count :%d\n", rx_count);

	return rx_count;
}

phys_addr_t wcn_get_btwf_base_addr(void)
{
	return s_wcn_device.btwf_device->base_addr;
}

phys_addr_t wcn_get_btwf_init_status_addr(void)
{
	return s_wcn_device.btwf_device->base_addr +
	       (phys_addr_t)&s_wssm_phy_offset_p->marlin.init_status;
}

phys_addr_t wcn_get_btwf_sleep_addr(void)
{
	return s_wcn_device.btwf_device->base_addr +
	       (phys_addr_t)&s_wssm_phy_offset_p->cp2_sleep_status;
}

struct regmap *wcn_get_btwf_regmap(u32 regmap_type)
{
	return s_wcn_device.btwf_device->rmap[regmap_type];
}

struct regmap *wcn_get_gnss_regmap(u32 regmap_type)
{
	return s_wcn_device.gnss_device->rmap[regmap_type];
}

phys_addr_t wcn_get_gnss_base_addr(void)
{
	return s_wcn_device.gnss_device->base_addr;
}

int wcn_get_gnss_power_status(void)
{
	u32 gnss_status;

	gnss_status = s_wcn_device.gnss_device->wcn_open_status &
		WCN_GNSS_ALL_MASK;
	WCN_INFO("gnss_device status:%d\n",
		 s_wcn_device.gnss_device->wcn_open_status);
	return gnss_status;
}

int wcn_get_btwf_power_status(void)
{
	WCN_INFO("btwf_device power_state:%d\n",
		 s_wcn_device.btwf_device->power_state);
	return s_wcn_device.btwf_device->power_state;
}

bool wcn_get_download_status(void)
{
	return flag_download;
}
EXPORT_SYMBOL_GPL(wcn_get_download_status);

int wcn_get_module_status(void)
{
	return wcn_open_module;
}
EXPORT_SYMBOL_GPL(wcn_get_module_status);

int wcn_get_module_status_changed(void)
{
	return wcn_module_state_change;
}
EXPORT_SYMBOL_GPL(wcn_get_module_status_changed);

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

void wcn_global_source_init(void)
{
	mutex_init(&marlin_lock);
	WCN_INFO("init finish!\n");
}

static void wcn_set_module_state(void)
{
	if (s_wcn_device.btwf_device->wcn_open_status & WCN_MARLIN_MASK)
		wcn_open_module = 1;
	else
		wcn_open_module = 0;
	wcn_module_state_change = 1;
	WCN_INFO("wcn_open_module:%d\n", wcn_open_module);
	open_mdbg_loopcheck_interru();
}

#ifdef CONFIG_PM_SLEEP
static int wcn_resume(struct device *dev)
{
	WCN_INFO("enter\n");
#if SUSPEND_RESUME_ENABLE
	slp_mgr_resume();
#endif
	WCN_INFO("ok\n");

	return 0;
}

static int wcn_suspend(struct device *dev)
{
	WCN_INFO("enter\n");
#if SUSPEND_RESUME_ENABLE
	slp_mgr_suspend();
#endif
	WCN_INFO("ok\n");

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

#if REGMAP_UPDATE_BITS_ENABLE
static void wcn_regmap_update_bit(struct wcn_device *ctrl,
					 u32 index,
					 u32 mask,
					 u32 val)
{
	u32 type;
	u32 reg;
	int ret;

	type = ctrl->ctrl_type[index];
	reg = ctrl->ctrl_reg[index];

	ret = regmap_update_bits(ctrl->rmap[type],
			   reg,
			   mask,
			   val);
	if (ret)
		WCN_ERR("regmap_update_bits ret=%d\n", ret);
}

static void wcn_regmap_write_bit(struct wcn_device *ctrl,
					 u32 index,
					 u32 mask,
					 u32 val)
{
	u32 type;
	u32 reg;
	int ret;

	type = ctrl->ctrl_type[index];
	reg = ctrl->ctrl_reg[index];

	ret = regmap_write_bits(ctrl->rmap[type],
			   reg,
			   mask,
			   val);
	if (ret)
		WCN_ERR("regmap_write_bits ret=%d\n", ret);
}
#endif

void wcn_regmap_raw_write_bit(struct regmap *cur_regmap,
				     u32 reg,
				     unsigned int val)
{
	int ret;
	u32 val_tmp = val;

	ret = regmap_raw_write(cur_regmap, reg, (const void *)&val_tmp, 4);
	if (ret)
		WCN_ERR("regmap_raw_write ret=%d\n", ret);
}

/* addr_offset:some REGs has twice group, one read and another write */
void wcn_regmap_read(struct regmap *cur_regmap,
			    u32 reg,
			    unsigned int *val)
{
	(void)regmap_read(cur_regmap, reg, val);
}

#if WCN_INTEGRATE_PLATFORM_DEBUG
static u32 s_wcn_debug_case;
static struct task_struct *s_thead_wcn_codes_debug;
static int wcn_codes_debug_thread(void *data)
{
	u32 i;
	static u32 is_first_time = 1;

	while (!kthread_should_stop()) {
		switch (s_wcn_debug_case) {
		case WCN_START_MARLIN_DEBUG:
			for (i = 0; i < 16; i++)
				start_integrate_wcn(i);

			s_wcn_debug_case = 0;
			break;

		case WCN_STOP_MARLIN_DEBUG:
			for (i = 0; i < 16; i++)
				stop_integrate_wcn(i);

			s_wcn_debug_case = 0;
			break;

		case WCN_START_MARLIN_DDR_FIRMWARE_DEBUG:
			for (i = 0; i < 16; i++)
				start_integrate_wcn(i);

			s_wcn_debug_case = 0;
			break;

		case WCN_START_GNSS_DEBUG:
			for (i = 16; i < 32; i++)
				start_integrate_wcn(i);

			s_wcn_debug_case = 0;
			break;

		case WCN_STOP_GNSS_DEBUG:
			for (i = 16; i < 32; i++)
				stop_integrate_wcn(i);

			s_wcn_debug_case = 0;
			break;

		case WCN_START_GNSS_DDR_FIRMWARE_DEBUG:
			for (i = 16; i < 32; i++)
				start_integrate_wcn(i);

			s_wcn_debug_case = 0;
			break;

		case WCN_PRINT_INFO:
			WCN_INFO(
				"cali[data=%p flag=%p]efuse=%p status=%p gnss=%p\n",
				&s_wssm_phy_offset_p->wifi.calibration_data,
				&s_wssm_phy_offset_p->wifi.calibration_flag,
				&s_wssm_phy_offset_p->wifi.efuse[0],
				&s_wssm_phy_offset_p->marlin.init_status,
				&s_wssm_phy_offset_p->include_gnss);
			break;

		case WCN_BRINGUP_DEBUG:
			if (is_first_time) {
				msleep(100000);
				is_first_time = 0;
			}

			for (i = 0; i < 16; i++) {
				msleep(5000);
				start_integrate_wcn(i);
			}

			for (i = 0; i < 16; i++) {
				msleep(5000);
				stop_integrate_wcn(i);
			}

			break;

		default:
			msleep(5000);
			break;
		}
	}

	kthread_stop(s_thead_wcn_codes_debug);

	return 0;
}

static void wcn_codes_debug(void)
{
	/* Check reg read */
	s_thead_wcn_codes_debug = kthread_create(wcn_codes_debug_thread, NULL,
			"wcn_codes_debug");
	wake_up_process(s_thead_wcn_codes_debug);
}
#endif

static void wcn_direct_regmap_read(struct regmap *regmap, u32 addr)
{
	u32 reg_value = 0;

	wcn_regmap_read(regmap, addr, &reg_value);
	WCN_INFO("ctrl_reg=0x%x,read=0x%x\n",
		 addr, reg_value);
}

static void wcn_wr_regmap_read(struct regmap *regmap, u32 addr)
{
	u32 reg_value = 0;

	wcn_regmap_raw_write_bit(regmap, 0XFF4, addr);
	wcn_regmap_read(regmap, 0XFFC, &reg_value);
	WCN_INFO("addr=0x%x,read=0x%x\n",
		 addr, reg_value);
}

static void wcn_show_debug_reg_info(struct wcn_device *wcn_dev)
{
	struct regmap *regmap;
	u32 reg_value = 0;
	u32 addr = 0;
	u32 iloop = 0;
	u32 pmu_reg[6] = {0x50, 0x54, 0x58, 0xb0, 0xcc, 0xd4};
	u32 wrap_reg[16] = {0xd0010000, 0xd0010004, 0xd0010008,
			    0xd001000c, 0xd0010018, 0xd0010020,
			    0xd0010024, 0xd0010028, 0xd001002c,
			    0xd0010088, 0xd00100a0, 0xd00100b0,
			    0xd00100b4, 0xd0020800, 0xd0020804,
			    0xd0020808};

	WCN_INFO("reg info print!\n");

	/* public aera */
	regmap = wcn_dev->rmap[REGMAP_PMU_APB];
	for (iloop = 0; iloop < 6; iloop++)
		wcn_direct_regmap_read(regmap, pmu_reg[iloop]);

	/* cp aera */
	regmap = wcn_dev->rmap[REGMAP_ANLG_WRAP_WCN];
	for (iloop = 0; iloop < 16; iloop++)
		wcn_wr_regmap_read(regmap, wrap_reg[iloop]);

	addr = 0X20;
	wcn_regmap_read(regmap, addr, &reg_value);
	WCN_INFO("addr=0x%x,read=0x%x\n",
		 addr, reg_value);

	addr = 0x24;
	wcn_regmap_read(regmap, addr, &reg_value);
	WCN_INFO("addr=0x%x,read=0x%x\n",
		 addr, reg_value);
}

static void wcn_config_ctrlreg(struct wcn_device *wcn_dev,
			u32 start, u32 end)
{
	u32 reg_read, type, i, val, utemp_val;

	for (i = start; i < end; i++) {
		val = 0;
		type = wcn_dev->ctrl_type[i];
		reg_read = wcn_dev->ctrl_reg[i] -
			   wcn_dev->ctrl_rw_offset[i];
		wcn_regmap_read(wcn_dev->rmap[type], reg_read, &val);
		WCN_INFO("ctrl_reg[%d]=0x%x,read=0x%x, set=%x\n",
			i, reg_read, val,
			wcn_dev->ctrl_value[i]);
		utemp_val = wcn_dev->ctrl_value[i];

		if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_PIKE2) {
			if (wcn_dev->ctrl_rw_offset[i] == 0x00)
				utemp_val = val | wcn_dev->ctrl_value[i];
		}

		WCN_INFO("rmap[%d]=%p,ctrl_reg=0x%x ctrl_us_delay=%u\n",
			type, wcn_dev->rmap[type],
			wcn_dev->ctrl_reg[i],
			wcn_dev->ctrl_us_delay[i]);
		wcn_regmap_raw_write_bit(wcn_dev->rmap[type],
					 wcn_dev->ctrl_reg[i],
					 utemp_val);
		if (wcn_dev->ctrl_us_delay[i] >= 10)
			usleep_range(wcn_dev->ctrl_us_delay[i],
				     wcn_dev->ctrl_us_delay[i] + 40);
		else
			udelay(wcn_dev->ctrl_us_delay[i]);
		wcn_regmap_read(wcn_dev->rmap[type], reg_read, &val);
		WCN_INFO("ctrl_reg[%d] = 0x%x, val=0x%x\n",
			i, reg_read, val);
	}
}

static void wcn_cpu_bootup(struct wcn_device *wcn_dev)
{
	u32 reg_nr;

	if (!wcn_dev)
		return;

	reg_nr = wcn_dev->reg_nr < REG_CTRL_CNT_MAX ?
		wcn_dev->reg_nr : REG_CTRL_CNT_MAX;
	wcn_config_ctrlreg(wcn_dev, wcn_dev->ctrl_probe_num, reg_nr);
}

/* return val: 1 for send the cmd to CP2 */
static int wcn_send_force_sleep_cmd(struct wcn_device *wcn_dev)
{
	u32 val;
	phys_addr_t phy_addr;

	phy_addr = wcn_dev->base_addr +
		   (phys_addr_t)&s_wssm_phy_offset_p->sleep_flag_addr;
	wcn_read_data_from_phy_addr(phy_addr, &val, sizeof(val));
	if  (val == MARLIN_USE_FORCE_SHUTDOWN) {
		mdbg_send("at+sleep_switch=2\r",
			  strlen("at+sleep_switch=2\r"), MDBG_SUBTYPE_AT);
		WCN_INFO("send sleep_switch=2\n");
		return 1;
	}

	return 0;
}

/*
 * WCN SYS include BTWF and GNSS sys, ret: 0 is sleep, else is not.
 * force_sleep: 1 for send CP2 shutdown cmd way, 0 for old way.
 */
static u32 wcn_get_sleep_status(struct wcn_device *wcn_dev, int force_sleep)
{
	u32 sleep_status = 0;
	u32 wcn_sleep_status_mask = 0xf000;
	u32 val;
	phys_addr_t phy_addr;

	if (wcn_dev_is_marlin(wcn_dev) && force_sleep) {
		phy_addr = wcn_dev->base_addr +
			   (phys_addr_t)&s_wssm_phy_offset_p->cp2_sleep_status;
		wcn_read_data_from_phy_addr(phy_addr, &val, sizeof(val));
		WCN_INFO("foce shut down val:0x%x\n", val);
		if (val == MARLIN_FORCE_SHUTDOWN_OK) {
			usleep_range(10000, 12000);
			return 0;
		}
		return 1;
	}

	wcn_regmap_read(wcn_dev->rmap[REGMAP_PMU_APB],
			0x00d4, &sleep_status);

	return (sleep_status & wcn_sleep_status_mask);
}

static struct wcn_proc_data g_proc_data;
static const struct of_device_id wcn_match_table[] = {
	{ .compatible = "sprd,integrate_marlin", .data = &g_proc_data},
	{ .compatible = "sprd,integrate_gnss", .data = &g_proc_data},
	{ },
};

static u32 wcn_parse_platform_chip_id(struct wcn_device *wcn_dev)
{
	wcn_regmap_read(wcn_dev->rmap[REGMAP_AON_APB],
					WCN_AON_CHIP_ID0,
					&g_platform_chip_id.aon_chip_id0);
	wcn_regmap_read(wcn_dev->rmap[REGMAP_AON_APB],
					WCN_AON_CHIP_ID1,
					&g_platform_chip_id.aon_chip_id1);
	wcn_regmap_read(wcn_dev->rmap[REGMAP_AON_APB],
					WCN_AON_PLATFORM_ID0,
					&g_platform_chip_id.aon_platform_id0);
	wcn_regmap_read(wcn_dev->rmap[REGMAP_AON_APB],
					WCN_AON_PLATFORM_ID1,
					&g_platform_chip_id.aon_platform_id1);
	wcn_regmap_read(wcn_dev->rmap[REGMAP_AON_APB],
					WCN_AON_CHIP_ID,
					&g_platform_chip_id.aon_chip_id);

	if (g_platform_chip_id.aon_chip_id0 == PIKE2_CHIP_ID0 &&
		g_platform_chip_id.aon_chip_id1 == PIKE2_CHIP_ID1)
		g_platform_chip_type = WCN_PLATFORM_TYPE_PIKE2;
	else if (g_platform_chip_id.aon_chip_id0 == SHARKLE_CHIP_ID0 &&
		g_platform_chip_id.aon_chip_id1 == SHARKLE_CHIP_ID1)
		g_platform_chip_type = WCN_PLATFORM_TYPE_SHARKLE;
	else if (g_platform_chip_id.aon_chip_id0 == SHARKL3_CHIP_ID0 &&
		g_platform_chip_id.aon_chip_id1 == SHARKL3_CHIP_ID1)
		g_platform_chip_type = WCN_PLATFORM_TYPE_SHARKL3;
	else
		WCN_ERR("aon_chip_id0:[%d],id1[%d]\n",
					g_platform_chip_id.aon_chip_id0,
					g_platform_chip_id.aon_chip_id1);

	WCN_INFO("platform chip type: [%d]\n",
							g_platform_chip_type);

	return 0;
}

static int wcn_parse_dt(struct platform_device *pdev,
	struct wcn_device *wcn_dev)
{
	struct device_node *np = pdev->dev.of_node;
	u32 cr_num;
	int index, ret;
	u32 i;
	struct resource res;
	const struct of_device_id *of_id =
		of_match_node(wcn_match_table, np);
	struct wcn_proc_data *pcproc_data;

	WCN_INFO("start!\n");

	if (of_id)
		pcproc_data = (struct wcn_proc_data *)of_id->data;
	else {
		WCN_ERR("not find matched id!");
		return -EINVAL;
	}

	if (!wcn_dev) {
		WCN_ERR("wcn_dev NULL\n");
		return -EINVAL;
	}

	/* get the wcn chip name */
	ret = of_property_read_string(np,
				      "sprd,name",
				      (const char **)&wcn_dev->name);

	/* get apb reg handle */
	wcn_dev->rmap[REGMAP_AON_APB] = syscon_regmap_lookup_by_phandle(np,
						"sprd,syscon-ap-apb");
	if (IS_ERR(wcn_dev->rmap[REGMAP_AON_APB])) {
		WCN_ERR("failed to find sprd,syscon-ap-apb\n");
		return -EINVAL;
	}

	wcn_parse_platform_chip_id(wcn_dev);

	/* get pmu reg handle */
	wcn_dev->rmap[REGMAP_PMU_APB] = syscon_regmap_lookup_by_phandle(np,
						"sprd,syscon-ap-pmu");
	if (IS_ERR(wcn_dev->rmap[REGMAP_PMU_APB])) {
		WCN_ERR("failed to find sprd,syscon-ap-pmu\n");
		return -EINVAL;
	}

	/* get pub apb reg handle:SHARKLE has it, but PIKE2 hasn't  */
	if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_SHARKLE) {
		wcn_dev->rmap[REGMAP_PUB_APB] =
				syscon_regmap_lookup_by_phandle(np,
						"sprd,syscon-ap-pub-apb");
		if (IS_ERR(wcn_dev->rmap[REGMAP_PUB_APB])) {
			WCN_ERR("failed to find sprd,syscon-ap-pub-apb\n");
			return -EINVAL;
		}
	}

	/* get  anlg wrap wcn reg handle */
	wcn_dev->rmap[REGMAP_ANLG_WRAP_WCN] =
					syscon_regmap_lookup_by_phandle(
					np, "sprd,syscon-anlg-wrap-wcn");
	if (IS_ERR(wcn_dev->rmap[REGMAP_ANLG_WRAP_WCN])) {
		WCN_ERR("failed to find sprd,anlg-wrap-wcn\n");
		return -EINVAL;
	}

	if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_SHARKLE) {
		/* get  anlg wrap wcn reg handle */
		wcn_dev->rmap[REGMAP_ANLG_PHY_G6] =
					syscon_regmap_lookup_by_phandle(
					np, "sprd,syscon-anlg-phy-g6");
		if (IS_ERR(wcn_dev->rmap[REGMAP_ANLG_PHY_G6])) {
			WCN_ERR("failed to find sprd,anlg-phy-g6\n");
			return -EINVAL;
		}
	}

	/* SharkL3:The base Reg changed which used by AP read CP2 Regs */
	if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_SHARKL3) {
		/* get  anlg wrap wcn reg handle */
		wcn_dev->rmap[REGMAP_WCN_REG] =
					syscon_regmap_lookup_by_phandle(
					np, "sprd,syscon-wcn-reg");
		if (IS_ERR(wcn_dev->rmap[REGMAP_WCN_REG])) {
			WCN_ERR("failed to find sprd,wcn-reg\n");
			return -EINVAL;
		}

		WCN_INFO("success to find sprd,wcn-reg for SharkL3 %p\n",
			   wcn_dev->rmap[REGMAP_WCN_REG]);
	}

	ret = of_property_read_u32(np, "sprd,ctrl-probe-num",
				   &wcn_dev->ctrl_probe_num);
	if (ret) {
		WCN_ERR("failed to find sprd,ctrl-probe-num\n");
		return -EINVAL;
	}

	/*
	 * get ctrl_reg offset, the ctrl-reg variable number, so need
	 * to start reading from the largest until success
	 */
	cr_num = of_property_count_elems_of_size(np, "sprd,ctrl-reg", 4);
	if (cr_num > REG_CTRL_CNT_MAX) {
		WCN_ERR("DTS config err. cr_num=%d\n", cr_num);
		return -EINVAL;
	}

	do {
		ret = of_property_read_u32_array(np, "sprd,ctrl-reg",
					(u32 *)wcn_dev->ctrl_reg, cr_num);
		if (ret)
			cr_num--;
		if (!cr_num)
			return -EINVAL;
	} while (ret);

	wcn_dev->reg_nr = cr_num;
	for (i = 0; i < cr_num; i++)
		WCN_INFO("ctrl_reg[%d] = 0x%x\n",
			i, wcn_dev->ctrl_reg[i]);

	/* get ctrl_mask */
	ret = of_property_read_u32_array(np, "sprd,ctrl-mask",
					(u32 *)wcn_dev->ctrl_mask, cr_num);
	if (ret)
		return -EINVAL;
	for (i = 0; i < cr_num; i++)
		WCN_INFO("ctrl_mask[%d] = 0x%08x\n",
			i, wcn_dev->ctrl_mask[i]);

	/* get ctrl_value */
	ret = of_property_read_u32_array(np,
					 "sprd,ctrl-value",
					 (u32 *)wcn_dev->ctrl_value,
					 cr_num);
	if (ret)
		return -EINVAL;
	for (i = 0; i < cr_num; i++)
		WCN_INFO("ctrl_value[%d] = 0x%08x\n",
			i, wcn_dev->ctrl_value[i]);

	/* get ctrl_rw_offset */
	ret = of_property_read_u32_array(np,
					 "sprd,ctrl-rw-offset",
					 (u32 *)wcn_dev->ctrl_rw_offset,
					 cr_num);
	if (ret)
		return -EINVAL;
	for (i = 0; i < cr_num; i++)
		WCN_INFO("ctrl_rw_offset[%d] = 0x%08x\n",
			i, wcn_dev->ctrl_rw_offset[i]);

	/* get ctrl_us_delay */
	ret = of_property_read_u32_array(np,
					 "sprd,ctrl-us-delay",
					 (u32 *)wcn_dev->ctrl_us_delay,
					 cr_num);
	if (ret)
		return -EINVAL;
	for (i = 0; i < cr_num; i++)
		WCN_INFO("ctrl_us_delay[%d] = 0x%08x\n",
			i, wcn_dev->ctrl_us_delay[i]);

	/* get ctrl_type */
	ret = of_property_read_u32_array(np, "sprd,ctrl-type",
					(u32 *)wcn_dev->ctrl_type, cr_num);
	if (ret)
		return -EINVAL;

	for (i = 0; i < cr_num; i++)
		WCN_INFO("ctrl_type[%d] = 0x%08x\n",
			i, wcn_dev->ctrl_type[i]);

	/*
	 * Add a new group to control shut down WCN
	 * get ctrl_reg offset, the ctrl-reg variable number, so need
	 * to start reading from the largest until success
	 */
	cr_num = of_property_count_elems_of_size(np,
				 "sprd,ctrl-shutdown-reg", 4);
	if (cr_num > REG_CTRL_CNT_MAX) {
		WCN_ERR("DTS config err. cr_num=%d\n", cr_num);
		return -EINVAL;
	}

	do {
		ret = of_property_read_u32_array(np,
				"sprd,ctrl-shutdown-reg",
				(u32 *)wcn_dev->ctrl_shutdown_reg,
				 cr_num);
		if (ret)
			cr_num--;
		if (!cr_num)
			return -EINVAL;
	} while (ret);

	wcn_dev->reg_shutdown_nr = cr_num;
	for (i = 0; i < cr_num; i++) {
		WCN_INFO("ctrl_shutdown_reg[%d] = 0x%x\n",
			i, wcn_dev->ctrl_shutdown_reg[i]);
	}

	/* get ctrl_shutdown_mask */
	ret = of_property_read_u32_array(np,
					 "sprd,ctrl-shutdown-mask",
					 (u32 *)wcn_dev->ctrl_shutdown_mask,
					 cr_num);
	if (ret)
		return -EINVAL;
	for (i = 0; i < cr_num; i++) {
		WCN_INFO("ctrl_shutdown_mask[%d] = 0x%08x\n",
			i, wcn_dev->ctrl_shutdown_mask[i]);
	}

	/* get ctrl_shutdown_value */
	ret = of_property_read_u32_array(np, "sprd,ctrl-shutdown-value",
				(u32 *)wcn_dev->ctrl_shutdown_value, cr_num);
	if (ret)
		return -EINVAL;
	for (i = 0; i < cr_num; i++) {
		WCN_INFO("ctrl_shutdown_value[%d] = 0x%08x\n",
			i, wcn_dev->ctrl_shutdown_value[i]);
	}

	/* get ctrl_shutdown_rw_offset */
	ret = of_property_read_u32_array(np,
			"sprd,ctrl-shutdown-rw-offset",
			(u32 *)wcn_dev->ctrl_shutdown_rw_offset, cr_num);
	if (ret)
		return -EINVAL;
	for (i = 0; i < cr_num; i++) {
		WCN_INFO("ctrl_shutdown_rw_offset[%d] = 0x%08x\n",
			i, wcn_dev->ctrl_shutdown_rw_offset[i]);
	}

	/* get ctrl_shutdown_us_delay */
	ret = of_property_read_u32_array(np,
			"sprd,ctrl-shutdown-us-delay",
			(u32 *)wcn_dev->ctrl_shutdown_us_delay, cr_num);
	if (ret)
		return -EINVAL;
	for (i = 0; i < cr_num; i++) {
		WCN_INFO("ctrl_shutdown_us_delay[%d] = 0x%08x\n",
			i, wcn_dev->ctrl_shutdown_us_delay[i]);
	}

	/* get ctrl_shutdown_type */
	ret = of_property_read_u32_array(np,
			"sprd,ctrl-shutdown-type",
			(u32 *)wcn_dev->ctrl_shutdown_type, cr_num);
	if (ret)
		return -EINVAL;

	for (i = 0; i < cr_num; i++)
		WCN_INFO("ctrl_shutdown_type[%d] = 0x%08x\n",
			i, wcn_dev->ctrl_shutdown_type[i]);

	/* get vddwcn */
	if (s_wcn_device.vddwcn == NULL) {
		s_wcn_device.vddwcn = devm_regulator_get(&pdev->dev,
						     "vddwcn");
		if (IS_ERR(s_wcn_device.vddwcn)) {
			WCN_ERR("Get regulator of vddwcn error!\n");
			return -EINVAL;
		}
	}

	/* get vddwifipa: only MARLIN has it */
	if (strcmp(wcn_dev->name, WCN_MARLIN_DEV_NAME) == 0) {
		wcn_dev->vddwifipa = devm_regulator_get(&pdev->dev,
							"vddwifipa");
		if (IS_ERR(wcn_dev->vddwifipa)) {
			WCN_ERR("Get regulator of vddwifipa error!\n");
			return -EINVAL;
		}
	}

	/* get cp base */
	index = 0;
	ret = of_address_to_resource(np, index, &res);
	if (ret)
		return -EINVAL;
	wcn_dev->base_addr = res.start;
	wcn_dev->maxsz = res.end - res.start + 1;
	WCN_INFO("cp base = %llu, size = 0x%x\n",
			(u64)wcn_dev->base_addr, wcn_dev->maxsz);

	/* get cp source file length */
	ret = of_property_read_u32_index(np,
					  "sprd,file-length",
					  0,
					  &wcn_dev->file_length);
	WCN_INFO("wcn_dev->file_length:%d\n", wcn_dev->file_length);
	if (ret)
		return -EINVAL;

	wcn_dev->start = pcproc_data->start;
	wcn_dev->stop = pcproc_data->stop;

	return 0;
}

static int wcn_get_firmware_path(char *firmwarename, char *firmware_path)
{
	if (firmwarename == NULL || firmware_path == NULL)
		return -EINVAL;

	memset(firmware_path, 0, FIRMWARE_FILEPATHNAME_LENGTH_MAX);
	if (strcmp(firmwarename, WCN_MARLIN_DEV_NAME) == 0) {
		if (parse_firmware_path(firmware_path))
			return -EINVAL;
	} else if (strcmp(firmwarename, WCN_GNSS_DEV_NAME) == 0) {
		int folder_path_length = 0;
		/*
		 * GNSS firmware path is the same as BTWF
		 * But the function parse_firmware_path return path
		 * includes filename of wcnmodem
		 */
		if (parse_firmware_path(firmware_path))
			return -EINVAL;
		folder_path_length = strlen(firmware_path)
			-strlen(WCN_BTWF_FILENAME);
		*(firmware_path + folder_path_length) = 0;
		strcpy(gnss_firmware_parent_path, firmware_path);

	} else
		return -EINVAL;

	WCN_INFO("wcn_dev->firmware_path:%s\n",
		firmware_path);

	return 0;
}

/* only wifi need it */
static void marlin_write_cali_data(void)
{
	phys_addr_t phy_addr;
	u32 cali_flag;

	/* get cali para from RF */
	get_connectivity_config_param(&wifi_data.config_data);
	get_connectivity_cali_param(&wifi_data.cali_data);

	/* copy calibration file data to target ddr address */
	phy_addr = s_wcn_device.btwf_device->base_addr +
		   (phys_addr_t)&s_wssm_phy_offset_p->wifi.calibration_data;
	wcn_write_data_to_phy_addr(phy_addr, &wifi_data, sizeof(wifi_data));

	/* notify CP to cali */
	cali_flag = WIFI_CALIBRATION_FLAG_VALUE;
	phy_addr = s_wcn_device.btwf_device->base_addr +
		   (phys_addr_t)&s_wssm_phy_offset_p->wifi.calibration_flag;
	wcn_write_data_to_phy_addr(phy_addr, &cali_flag, sizeof(cali_flag));

	WCN_INFO("finish\n");
}

/* only wifi need it: AP save the cali data to ini file */
static void marlin_save_cali_data(void)
{
	phys_addr_t phy_addr;

	if (s_wcn_device.btwf_device) {
		memset(&wifi_data.cali_data, 0x0,
					sizeof(struct wifi_cali_t));
		/* copy calibration file data to target ddr address */
		phy_addr = s_wcn_device.btwf_device->base_addr +
			   (phys_addr_t)&s_wssm_phy_offset_p->
			   wifi.calibration_data +
			   sizeof(struct wifi_config_t);
		wcn_read_data_from_phy_addr(phy_addr, &wifi_data.cali_data,
					    sizeof(struct wifi_cali_t));
		dump_cali_file(&wifi_data.cali_data);
		WCN_INFO("finish\n");
	}
}

#define WCN_EFUSE_TEMPERATURE_MAGIC 0x432ff678
#define WCN_EFUSE_TEMPERATURE_OFF 44
/* now just sharkle */
static void marlin_write_efuse_temperature(void)
{
	phys_addr_t phy_addr;
	u32 magic, val;

	magic = WCN_EFUSE_TEMPERATURE_MAGIC;
	val = sprd_efuse_double_read(WCN_EFUSE_TEMPERATURE_OFF, true);
	if (val == 0) {
		WCN_INFO("temperature efuse read err\n");
		magic += 1;
		goto out;
	}
	WCN_INFO("temperature efuse read 0x%x\n", val);
	phy_addr = s_wcn_device.btwf_device->base_addr +
		   (phys_addr_t)&s_wssm_phy_offset_p->efuse_temper_val;
	wcn_write_data_to_phy_addr(phy_addr, &val, sizeof(val));
out:
	phy_addr = s_wcn_device.btwf_device->base_addr +
		   (phys_addr_t)&s_wssm_phy_offset_p->efuse_temper_magic;
	wcn_write_data_to_phy_addr(phy_addr, &magic, sizeof(magic));
}

/* only wifi need it */
static void marlin_write_efuse_data(void)
{
	phys_addr_t phy_addr;
	u32 iloop = 0;
	u32 tmp_value[WIFI_EFUSE_BLOCK_COUNT];
	u32 chip_type;

	chip_type = wcn_platform_chip_type();
	/* get data from Efuse */
	memset(&tmp_value, 0, sizeof(tmp_value[0])*WIFI_EFUSE_BLOCK_COUNT);
	for (iloop = 0; iloop < WIFI_EFUSE_BLOCK_COUNT; iloop++) {
		tmp_value[iloop] =
		sprd_efuse_double_read(
				s_wifi_efuse_id[chip_type][iloop], true);
		if (tmp_value[iloop] == 0)
			tmp_value[iloop] = s_wifi_efuse_default_value[iloop];
	}

	for (iloop = 0; iloop < WIFI_EFUSE_BLOCK_COUNT; iloop++)
		WCN_INFO("s_wifi_efuse_id[%d][%d]=%d\n",
				chip_type,
				iloop,
				s_wifi_efuse_id[chip_type][iloop]);
	/* copy efuse data to target ddr address */
	phy_addr = s_wcn_device.btwf_device->base_addr +
		   (phys_addr_t)&s_wssm_phy_offset_p->wifi.efuse[0];
	wcn_write_data_to_phy_addr(phy_addr, &tmp_value,
				sizeof(tmp_value[0])*WIFI_EFUSE_BLOCK_COUNT);

	WCN_INFO("finish\n");
}

static void wcn_marlin_write_efuse(void)
{
	marlin_write_efuse_data();
	marlin_write_efuse_temperature();
}

/* used for provide efuse data to gnss */
static void gnss_write_efuse_data(void)
{
	phys_addr_t phy_addr;
	u32 iloop = 0;
	u32 tmp_value[GNSS_EFUSE_BLOCK_COUNT];

	/* get data from Efuse */
	memset(&tmp_value, 0, sizeof(tmp_value[0]) * GNSS_EFUSE_BLOCK_COUNT);
	for (iloop = 0; iloop < GNSS_EFUSE_BLOCK_COUNT; iloop++) {
		tmp_value[iloop] =
		sprd_efuse_double_read(s_gnss_efuse_id[iloop], true);
	}

	/* copy efuse data to target ddr address */
	phy_addr = s_wcn_device.gnss_device->base_addr +
				   GNSS_EFUSE_DATA_OFFSET;
	wcn_write_data_to_phy_addr(phy_addr, &tmp_value,
				sizeof(tmp_value[0]) * GNSS_EFUSE_BLOCK_COUNT);

	WCN_INFO("finish\n");
}

/* used for distinguish Pike2 or sharkle */
static void gnss_write_version_data(void)
{
	phys_addr_t phy_addr;
	u32 tmp_aon_id[2];

	tmp_aon_id[0] = g_platform_chip_id.aon_chip_id0;
	tmp_aon_id[1] = g_platform_chip_id.aon_chip_id1;
	phy_addr = wcn_get_gnss_base_addr() +
				   GNSS_REC_AON_CHIPID_OFFSET;
	wcn_write_data_to_phy_addr(phy_addr, &tmp_aon_id,
				GNSS_REC_AON_CHIPID_SIZE);

	WCN_INFO("finish\n");
}

/*  GNSS assert workaround */
#define GNSS_TEST_OFFSET 0x150050
#define GNSS_TEST_MAGIC 0x12345678
static void gnss_clear_boot_flag(void)
{
	phys_addr_t phy_addr;
	u32 magic_value;

	phy_addr = wcn_get_gnss_base_addr() + GNSS_TEST_OFFSET;
	wcn_read_data_from_phy_addr(phy_addr, &magic_value, sizeof(u32));
	WCN_INFO("value is 0x%x\n", magic_value);
	magic_value = 0;
	wcn_write_data_to_phy_addr(phy_addr, &magic_value, sizeof(u32));

	WCN_INFO("finish\n");
}

/* used for distinguish Pike2 or sharkle */
static void gnss_read_boot_flag(void)
{
	phys_addr_t phy_addr;
	u32 magic_value;
	u32 wait_count;

	phy_addr = wcn_get_gnss_base_addr() + GNSS_TEST_OFFSET;
	for (wait_count = 0; wait_count < MARLIN_WAIT_CP_INIT_COUNT;
	     wait_count++) {
		wcn_read_data_from_phy_addr(phy_addr,
					    &magic_value, sizeof(u32));
		if (magic_value == GNSS_TEST_MAGIC)
			break;

		msleep(MARLIN_WAIT_CP_INIT_POLL_TIME_MS);
		WCN_INFO("magic_value=%d, wait_count=%d\n",
			 magic_value, wait_count);
	}

	WCN_INFO("finish\n");
}

static bool wcn_load_firmware_data(struct wcn_device *wcn_dev)
{
	int read_len, size, i;
	char *data = NULL;
	char *wcn_image_buffer;
	struct file *file;
	unsigned int imag_size;
	bool is_gnss;

	WCN_INFO("entry\n");

	if (wcn_dev == NULL)
		return false;
	if (strlen(wcn_dev->firmware_path) == 0) {
		/* get firmware path */
		if (wcn_get_firmware_path(wcn_dev->name,
					  &wcn_dev->firmware_path[0]) < 0) {
			WCN_ERR("wcn_get_firmware path Failed!\n");
			return false;
		}
		WCN_INFO("firmware path=%s\n", &wcn_dev->firmware_path[0]);
	}
	is_gnss = wcn_dev_is_gnss(wcn_dev);
	if (is_gnss) {
		strcpy(&wcn_dev->firmware_path[0],
					gnss_firmware_parent_path);
		strcat(&wcn_dev->firmware_path[0],
					&wcn_dev->firmware_path_ext[0]);
		WCN_INFO("gnss path=%s\n", &wcn_dev->firmware_path[0]);
		gnss_file_path_set(&wcn_dev->firmware_path[0]);
	}
	/* try to open file */
	for (i = 1; i <= WCN_OPEN_MAX_CNT; i++) {
		file = filp_open(wcn_dev->firmware_path, O_RDONLY, 0);
		if (IS_ERR(file)) {
			WCN_ERR("try open file %s,count_num:%d, file=%p\n",
				wcn_dev->firmware_path, i, file);
			ssleep(1);
		} else
			break;
	}
	if (IS_ERR(file)) {
		WCN_ERR("open file %s error\n",
			wcn_dev->firmware_path);
		return false;
	}
	WCN_INFO("open image file %s  successfully\n",
		wcn_dev->firmware_path);

	/* read file to buffer */
	size = imag_size = wcn_dev->file_length;
	wcn_image_buffer = vmalloc(size);
	if (!wcn_image_buffer) {
		fput(file);
		WCN_ERR("no memory\n");
		return false;

	}
	WCN_INFO("wcn_image_buffer=%p\n", wcn_image_buffer);

	data = wcn_image_buffer;
	do {
		read_len = kernel_read(file, 0, wcn_image_buffer, size);
		if (read_len > 0) {
			size -= read_len;
			wcn_image_buffer += read_len;
		}
	} while ((read_len > 0) && (size > 0));
	fput(file);
	WCN_INFO("After read, wcn_image_buffer=%p\n", wcn_image_buffer);

	wcn_image_buffer = data;

#if WCN_INTEGRATE_PLATFORM_DEBUG
	if (s_wcn_debug_case == WCN_START_MARLIN_DDR_FIRMWARE_DEBUG)
		memcpy(wcn_image_buffer, &marlin_firmware_bin[0], imag_size);
	else if (s_wcn_debug_case == WCN_START_GNSS_DDR_FIRMWARE_DEBUG)
		memcpy(wcn_image_buffer, &gnss_firmware_bin[0], imag_size);
#endif

	/* copy file data to target ddr address */
	wcn_write_data_to_phy_addr(wcn_dev->base_addr, data, imag_size);

	vfree(wcn_image_buffer);

	WCN_INFO("finish\n");

	return true;

}

/*
 * This function is used to use the firmware subsystem
 * to load the wcn image.And at the same time support
 * for reading from the partition image.The first way
 * to use the first.
 */
static int wcn_download_image(struct wcn_device *wcn_dev)
{
	const struct firmware *firmware;
	bool load_fimrware_ret;
	bool is_marlin;
	int err;

	is_marlin = wcn_dev_is_marlin(wcn_dev);
	memset(firmware_file_name, 0, FIRMWARE_FILEPATHNAME_LENGTH_MAX);

	if (!is_marlin) {
		if (s_wcn_device.gnss_type == WCN_GNSS_TYPE_GL)
			strcpy(firmware_file_name, WCN_GNSS_FILENAME);
		else if (s_wcn_device.gnss_type == WCN_GNSS_TYPE_BD)
			strcpy(firmware_file_name, WCN_GNSS_BD_FILENAME);
		else
			return -EINVAL;
	}

	if (is_marlin)
		strcpy(firmware_file_name, WCN_BTWF_FILENAME);

	strcat(firmware_file_name, ".bin");
	WCN_INFO("loading image [%s] from firmware subsystem ...\n",
			firmware_file_name);
	err = request_firmware_direct(&firmware, firmware_file_name, NULL);
	if (err < 0) {
		WCN_ERR("no find image [%s] errno:(%d)(ignore!!)\n",
				firmware_file_name, err);
		load_fimrware_ret = wcn_load_firmware_data(wcn_dev);
		if (load_fimrware_ret == false) {
			WCN_ERR("wcn_load_firmware_data ERR!\n");
			return -EINVAL;
		}
	} else {
		WCN_INFO("image size = %d\n", (int)firmware->size);
		if (wcn_write_data_to_phy_addr(wcn_dev->base_addr,
					       (void *)firmware->data,
					       firmware->size)) {
			WCN_ERR("wcn_mem_ram_vmap_nocache fail\n");
			release_firmware(firmware);
			return -ENOMEM;
		}

		release_firmware(firmware);
		WCN_INFO("loading image [%s] successfully!\n",
				firmware_file_name);
	}

	return 0;
}

static void wcn_clean_marlin_ddr_flag(struct wcn_device *wcn_dev)
{
	phys_addr_t phy_addr;
	u32 tmp_value;

	tmp_value = MARLIN_CP_INIT_START_MAGIC;
	phy_addr = wcn_dev->base_addr +
		   (phys_addr_t)&s_wssm_phy_offset_p->marlin.init_status;
	wcn_write_data_to_phy_addr(phy_addr, &tmp_value, sizeof(tmp_value));

	tmp_value = 0;

	phy_addr = wcn_dev->base_addr +
		   (phys_addr_t)&s_wssm_phy_offset_p->cp2_sleep_status;
	wcn_write_data_to_phy_addr(phy_addr, &tmp_value, sizeof(tmp_value));
	phy_addr = wcn_dev->base_addr +
		   (phys_addr_t)&s_wssm_phy_offset_p->sleep_flag_addr;
	wcn_write_data_to_phy_addr(phy_addr, &tmp_value, sizeof(tmp_value));
}

static int wcn_wait_marlin_boot(struct wcn_device *wcn_dev)
{
	u32 wait_count = 0;
	u32 magic_value = 0;
	phys_addr_t phy_addr;

	phy_addr = wcn_dev->base_addr +
		   (phys_addr_t)&s_wssm_phy_offset_p->marlin.init_status;
	for (wait_count = 0; wait_count < MARLIN_WAIT_CP_INIT_COUNT;
	     wait_count++) {
		wcn_read_data_from_phy_addr(phy_addr,
					    &magic_value, sizeof(u32));
		if (magic_value == MARLIN_CP_INIT_READY_MAGIC)
			break;

		msleep(MARLIN_WAIT_CP_INIT_POLL_TIME_MS);
		WCN_INFO("BTWF: magic_value=0x%x, wait_count=%d\n",
			 magic_value, wait_count);
	}

	/* get CP ready flag failed */
	if (wait_count >= MARLIN_WAIT_CP_INIT_COUNT) {
		if (magic_value == MARLIN_CP_INIT_START_MAGIC &&
				wcn_get_gnss_power_status()) {
			WCN_ERR("more reg info:");
			wcn_show_debug_reg_info(wcn_dev);
		}
		WCN_ERR("MARLIN boot cp timeout!\n");
		magic_value = MARLIN_CP_INIT_FALIED_MAGIC;
		wcn_write_data_to_phy_addr(phy_addr, &magic_value, sizeof(u32));
		return -1;
	}

	return 0;
}

static void wcn_marlin_boot_finish(struct wcn_device *wcn_dev)
{
	phys_addr_t phy_addr;
	u32 magic_value = 0;

	/* save cali data to INI file */
	if (!s_wcn_device.btwf_calibrated) {
		u32 cali_flag;

		marlin_save_cali_data();
		/* clear notify CP calibration flag */
		cali_flag = WIFI_CALIBRATION_FLAG_CLEAR_VALUE;
		phy_addr = s_wcn_device.btwf_device->base_addr +
			   (phys_addr_t)&(s_wssm_phy_offset_p->
			   wifi.calibration_flag);
		wcn_write_data_to_phy_addr(phy_addr, &cali_flag,
					   sizeof(cali_flag));
		s_wcn_device.btwf_calibrated = true;
	}

	/* set success flag */
	phy_addr = wcn_dev->base_addr +
		   (phys_addr_t)&s_wssm_phy_offset_p->marlin.init_status;
	magic_value = MARLIN_CP_INIT_SUCCESS_MAGIC;
	wcn_write_data_to_phy_addr(phy_addr, &magic_value, sizeof(u32));
}

static int wcn_wait_gnss_boot(struct wcn_device *wcn_dev)
{
	static int cali_flag;
	u32 wait_count = 0;
	u32 magic_value = 0;
	phys_addr_t phy_addr;

	if (cali_flag) {
		gnss_read_boot_flag();
		return 0;
	}

	phy_addr = wcn_dev->base_addr +
		   GNSS_CALIBRATION_FLAG_CLEAR_ADDR;
	for (wait_count = 0; wait_count < GNSS_WAIT_CP_INIT_COUNT;
	     wait_count++) {
		wcn_read_data_from_phy_addr(phy_addr,
					    &magic_value, sizeof(u32));
		if (magic_value == GNSS_CALI_DONE_FLAG)
			break;
		msleep(GNSS_WAIT_CP_INIT_POLL_TIME_MS);
		WCN_INFO("GPS: magic_value=0x%x, wait_count=%d\n",
			 magic_value, wait_count);
	}

	if (wait_count < GNSS_WAIT_CP_INIT_COUNT)
		cali_flag = 1;

	return 0;
}

static void wcn_marlin_pre_boot(struct wcn_device *wcn_dev)
{
	if (!s_wcn_device.btwf_calibrated)
		marlin_write_cali_data();
}

static void wcn_gnss_pre_boot(struct wcn_device *wcn_dev)
{
	gnss_write_version_data();
}

/* load firmware and boot up sys. */
static int wcn_proc_native_start(void *arg)
{
	bool is_marlin;
	int err;
	struct wcn_device *wcn_dev = (struct wcn_device *)arg;

	if (!wcn_dev) {
		WCN_ERR("dev is NULL\n");
		return -ENODEV;
	}

	WCN_INFO("%s enter\n", wcn_dev->name);
	is_marlin = wcn_dev_is_marlin(wcn_dev);

	/* when hot restart handset, the DDR value is error */
	if (is_marlin)
		wcn_clean_marlin_ddr_flag(wcn_dev);

	wcn_dev->boot_cp_status = WCN_BOOT_CP2_OK;
	err = wcn_download_image(wcn_dev);
	if (err < 0) {
		WCN_ERR("wcn download image err!\n");
		wcn_dev->boot_cp_status = WCN_BOOT_CP2_ERR_DOWN_IMG;
		return -1;
	}

	if (is_marlin)
		/* wifi need calibrate */
		wcn_marlin_pre_boot(wcn_dev);
	else
		/* gnss need prepare some data before bootup */
		wcn_gnss_pre_boot(wcn_dev);

	/* boot up system */
	wcn_cpu_bootup(wcn_dev);

	wcn_dev->power_state = WCN_POWER_STATUS_ON;
	WCN_INFO("device power_state:%d\n",
		 wcn_dev->power_state);

	/* wifi need polling CP ready */
	if (is_marlin) {
		if (wcn_wait_marlin_boot(wcn_dev)) {
			wcn_dev->boot_cp_status = WCN_BOOT_CP2_ERR_BOOT;
			return -1;
		}
		wcn_marlin_boot_finish(wcn_dev);
	} else
		wcn_wait_gnss_boot(wcn_dev);

	return 0;
}

static int wcn_proc_native_stop(void *arg)
{
	struct wcn_device *wcn_dev = arg;
	u32 iloop_index;
	u32 reg_nr = 0;
	unsigned int val;
	u32 reg_read;
	u32 type;

	WCN_INFO("enter\n");
	if (!wcn_dev)
		return -EINVAL;

	reg_nr = wcn_dev->reg_shutdown_nr < REG_SHUTDOWN_CNT_MAX ?
		wcn_dev->reg_shutdown_nr : REG_SHUTDOWN_CNT_MAX;
	for (iloop_index = 0; iloop_index < reg_nr; iloop_index++) {
		val = 0;
		type = wcn_dev->ctrl_shutdown_type[iloop_index];
		reg_read = wcn_dev->ctrl_shutdown_reg[iloop_index] -
			wcn_dev->ctrl_shutdown_rw_offset[iloop_index];
		wcn_regmap_read(wcn_dev->rmap[type],
				   reg_read,
				   &val
				   );
		WCN_INFO("ctrl_shutdown_reg[%d] = 0x%x, val=0x%x\n",
				iloop_index, reg_read, val);

		wcn_regmap_raw_write_bit(wcn_dev->rmap[type],
				wcn_dev->ctrl_shutdown_reg[iloop_index],
				wcn_dev->ctrl_shutdown_value[iloop_index]);
		udelay(wcn_dev->ctrl_shutdown_us_delay[iloop_index]);
		wcn_regmap_read(wcn_dev->rmap[type],
				   reg_read,
				   &val
				   );
		WCN_INFO("ctrl_reg[%d] = 0x%x, val=0x%x\n",
				iloop_index, reg_read, val);
	}

	return 0;
}

static struct wcn_proc_data g_proc_data = {
	.start = wcn_proc_native_start,
	.stop  = wcn_proc_native_stop,
};

static int wcn_platform_open(struct inode *inode, struct file *filp)
{
	struct platform_proc_file_entry
	*entry = (struct platform_proc_file_entry *)PDE_DATA(inode);

	WCN_INFO("entry name:%s\n!", entry->name);

	filp->private_data = entry;

	return 0;
}

static ssize_t wcn_platform_read(struct file *filp,
			       char __user *buf,
			       size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t wcn_platform_write(struct file *filp,
				const char __user *buf,
				size_t count,
				loff_t *ppos)
{
	struct platform_proc_file_entry
		*entry = (struct platform_proc_file_entry *)filp->private_data;
	struct wcn_device *wcn_dev = entry->wcn_dev;
	char *type = entry->name;
	unsigned flag;
	char str[WCN_PROC_FILE_LENGTH_MAX + 1];
	u32 sub_sys = 0;

	flag = entry->flag;
	WCN_INFO("type = %s flag = 0x%x\n", type, flag);

	if ((flag & BE_WRONLY) == 0)
		return -EPERM;

	memset(&str[0], 0, WCN_PROC_FILE_LENGTH_MAX + 1);
	if (copy_from_user(&str[0], buf, WCN_PROC_FILE_LENGTH_MAX) == 0) {
		if (strncmp(str, "gnss", strlen("gnss")) == 0)
			sub_sys = WCN_GNSS;
		else
			sub_sys = str[0] - '0';
	} else {
		WCN_ERR("copy_from_user too length %s!\n", buf);
		return -EINVAL;
	}

	if ((flag & BE_CTRL_ON) != 0) {
		start_integrate_wcn(sub_sys);
		wcn_dev->status = CP_NORMAL_STATUS;
		WCN_INFO("start, str=%s!\n", str);

		return count;
	} else if ((flag & BE_CTRL_OFF) != 0) {
		stop_integrate_wcn(sub_sys);
		wcn_dev->status = CP_STOP_STATUS;
		WCN_INFO("stop, str=%s!\n", str);

		return count;
	}

	return 0;
}

static const struct file_operations wcn_platform_fs_fops = {
	.open		= wcn_platform_open,
	.read		= wcn_platform_read,
	.write		= wcn_platform_write,
};

static inline void wcn_platform_fs_init(struct wcn_device *wcn_dev)
{
	u8 i = 0;
	u8 ucnt = 0;
	unsigned flag = 0;
	umode_t mode = 0;

	wcn_dev->platform_fs.platform_proc_dir_entry =
		proc_mkdir(wcn_dev->name, NULL);

	memset(wcn_dev->platform_fs.entrys,
		0,
		sizeof(wcn_dev->platform_fs.entrys));

	for (flag = 0, ucnt = 0, i = 0;
		i < MAX_PLATFORM_ENTRY_NUM;
		i++, flag = 0, mode = 0) {
		switch (i) {
		case 0:
			wcn_dev->platform_fs.entrys[i].name = "start";
			flag |= (BE_WRONLY | BE_CTRL_ON);
			ucnt++;
			break;

		case 1:
			wcn_dev->platform_fs.entrys[i].name = "stop";
			flag |= (BE_WRONLY | BE_CTRL_OFF);
			ucnt++;
			break;

		case 2:
			wcn_dev->platform_fs.entrys[i].name = "status";
			flag |= (BE_RDONLY | BE_RDWDTS);
			ucnt++;
			break;

		default:
			return;		/* we didn't use it until now */
		}

		wcn_dev->platform_fs.entrys[i].flag = flag;

		mode |= (S_IRUSR | S_IWUSR);
		if (flag & (BE_CPDUMP | BE_MNDUMP))
			mode |= S_IROTH;

		WCN_INFO("entry name is %s type 0x%x addr: 0x%p\n",
			wcn_dev->platform_fs.entrys[i].name,
			wcn_dev->platform_fs.entrys[i].flag,
			&wcn_dev->platform_fs.entrys[i]);

		wcn_dev->platform_fs.entrys[i].platform_proc_dir_entry =
			proc_create_data(
				wcn_dev->platform_fs.entrys[i].name,
				mode,
				wcn_dev->platform_fs.platform_proc_dir_entry,
				&wcn_platform_fs_fops,
				&wcn_dev->platform_fs.entrys[i]);
		wcn_dev->platform_fs.entrys[i].wcn_dev = wcn_dev;
	}
}

static inline void wcn_platform_fs_exit(struct wcn_device *wcn_dev)
{
	u8 i = 0;

	for (i = 0; i < MAX_PLATFORM_ENTRY_NUM; i++) {
		if (!wcn_dev->platform_fs.entrys[i].name)
			break;

		if (wcn_dev->platform_fs.entrys[i].flag != 0) {
			remove_proc_entry(wcn_dev->platform_fs.entrys[i].name,
				wcn_dev->platform_fs.platform_proc_dir_entry);
		}
	}

	remove_proc_entry(wcn_dev->name, NULL);
}

static void wcn_power_domain_set(struct wcn_device *wcn_dev, u32 set_type)
{
	u32 offset0 = 0, offset1 = 0;
	u32 bitmap0 = 0, bitmap1 = 0;

	if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_PIKE2) {
		if (set_type == 1) {
			offset0 = 0X2050;
			offset1 = 0X1050;
			bitmap0 = 1 << 24;
			bitmap1 = 1 << 25;
		} else {
			offset0 = 0X2050;
			offset1 = 0X2050;
			bitmap0 = 1 << 24;
			bitmap1 = 1 << 25;
		}
	} else {
		if (set_type == 1) {
			offset0 = 0X2100;
			offset1 = 0X1100;
			bitmap0 = 1 << 24;
			bitmap1 = 1 << 25;
		} else {
			offset0 = 0X2100;
			offset1 = 0X2100;
			bitmap0 = 1 << 24;
			bitmap1 = 1 << 25;
		}
	}
	wcn_regmap_raw_write_bit(wcn_dev->rmap[REGMAP_PMU_APB],
					 offset0, bitmap0);
	wcn_regmap_raw_write_bit(wcn_dev->rmap[REGMAP_PMU_APB],
					 offset1, bitmap1);
}

static void wcn_xtl_auto_sel(bool enable)
{
	struct regmap *regmap;
	u32 value;

	regmap = wcn_get_btwf_regmap(REGMAP_PMU_APB);
	wcn_regmap_read(regmap, 0x338, &value);

	if (enable) {
		value |= 1 << 4;
		wcn_regmap_raw_write_bit(regmap, 0x338, value);
	} else {
		value &= ~(1 << 4);
		wcn_regmap_raw_write_bit(regmap, 0X338, value);
	}
}

static int wcn_power_enable_sys_domain(bool enable)
{
	int ret = 0;
	u32 btwf_open = false;
	u32 gnss_open = false;
	static u32 sys_domain;

	if (s_wcn_device.btwf_device &&
		s_wcn_device.btwf_device->wcn_open_status & WCN_MARLIN_MASK)
		btwf_open = true;
	if (s_wcn_device.gnss_device &&
		s_wcn_device.gnss_device->wcn_open_status & WCN_GNSS_ALL_MASK)
		gnss_open = true;

	if (enable && (sys_domain == false)) {
		if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_PIKE2)
			wcn_xtl_auto_sel(false);
		wcn_power_domain_set(s_wcn_device.btwf_device, 0);
		if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_PIKE2)
			wcn_xtl_auto_sel(true);
		sys_domain = true;
		WCN_INFO("clear WCN SYS TOP PD\n");
	} else if ((!btwf_open) && (!gnss_open) && sys_domain) {
		if (wcn_platform_chip_type() ==
				WCN_PLATFORM_TYPE_PIKE2)
			wcn_xtl_auto_sel(false);
		wcn_power_domain_set(s_wcn_device.btwf_device, 1);
		sys_domain = false;
		WCN_INFO("set WCN SYS TOP PD\n");
	}
	WCN_INFO("enable = %d, ret = %d, btwf_open=%d, gnss_open=%d\n",
		     enable, ret, btwf_open, gnss_open);

	return ret;
}

/*
 * wcn_sys_soft_reset was used by BTWF and GNSS together
 * both BTWF and GNSS not work, we should set it.
 */
static void wcn_sys_soft_reset(void)
{
	u32 btwf_open = false;
	u32 gnss_open = false;
	u32 offset = 0;
	u32 bitmap = 0;
	struct regmap *rmap = NULL;

	if (!s_wcn_device.btwf_device && !s_wcn_device.gnss_device)
		return;

	if (s_wcn_device.btwf_device &&
	    s_wcn_device.btwf_device->wcn_open_status)
		btwf_open = true;
	if (s_wcn_device.gnss_device &&
	    s_wcn_device.gnss_device->wcn_open_status)
		gnss_open = true;

	if (!btwf_open && !gnss_open) {
		if (wcn_platform_chip_type() ==
		    WCN_PLATFORM_TYPE_PIKE2) {
			bitmap = 1 << 7;
		} else if (wcn_platform_chip_type() ==
			   WCN_PLATFORM_TYPE_SHARKLE) {
			bitmap = 1 << 9;
		} else if (wcn_platform_chip_type() ==
			   WCN_PLATFORM_TYPE_SHARKL3) {
			bitmap = 1 << 16;
		} else {
			WCN_ERR("chip type err\n");
			return;
		}
		offset = 0X10b0;
		rmap = s_wcn_device.btwf_device ?
		       s_wcn_device.btwf_device->rmap[REGMAP_PMU_APB] :
		       s_wcn_device.gnss_device->rmap[REGMAP_PMU_APB];
		wcn_regmap_raw_write_bit(rmap, offset, bitmap);
		WCN_INFO("finish\n");
		usleep_range(WCN_CP_SOFT_RST_MIN_TIME,
			     WCN_CP_SOFT_RST_MAX_TIME);
	}
}

static void wcn_sys_ctrl_26m(bool enable)
{
	struct regmap *regmap;
	u32 value;

	regmap = wcn_get_btwf_regmap(REGMAP_ANLG_PHY_G6);
	wcn_regmap_read(regmap, 0x28, &value);

	if (enable) {
		value &= ~(1 << 2);
		wcn_regmap_raw_write_bit(regmap, 0X28, value);
	} else {
		value |= 1 << 2;
		wcn_regmap_raw_write_bit(regmap, 0X28, value);
	}
}

/*
 * wcn_sys_soft_release was used by BTWF and GNSS together
 * both BTWF and GNSS not work, we should set it.
 */
static void wcn_sys_soft_release(void)
{
	u32 btwf_open = false;
	u32 gnss_open = false;
	u32 offset = 0;
	u32 bitmap = 0;
	struct regmap *rmap = NULL;

	if (!s_wcn_device.btwf_device && !s_wcn_device.gnss_device)
		return;

	if (s_wcn_device.btwf_device &&
	    s_wcn_device.btwf_device->wcn_open_status)
		btwf_open = true;
	if (s_wcn_device.gnss_device &&
	    s_wcn_device.gnss_device->wcn_open_status)
		gnss_open = true;

	if (!btwf_open && !gnss_open) {
		if (wcn_platform_chip_type() ==
		    WCN_PLATFORM_TYPE_PIKE2) {
			bitmap = 1 << 7;
		} else if (wcn_platform_chip_type() ==
			   WCN_PLATFORM_TYPE_SHARKLE) {
			bitmap = 1 << 9;
		} else if (wcn_platform_chip_type() ==
			   WCN_PLATFORM_TYPE_SHARKL3) {
			bitmap = 1 << 16;
		} else {
			WCN_ERR("chip type err\n");
			return;
		}
		offset = 0X20b0;
		rmap = s_wcn_device.btwf_device ?
		       s_wcn_device.btwf_device->rmap[REGMAP_PMU_APB] :
		       s_wcn_device.gnss_device->rmap[REGMAP_PMU_APB];
		wcn_regmap_raw_write_bit(rmap, offset, bitmap);
		WCN_INFO("finish!\n");
		usleep_range(WCN_CP_SOFT_RST_MIN_TIME,
			     WCN_CP_SOFT_RST_MAX_TIME);
	}
}

/*
 * wcn_sys_deep_sleep_en was used by BTWF and GNSS together
 * both BTWF and GNSS not work, we should set it.
 */
static void wcn_sys_deep_sleep_en(void)
{
	struct regmap *rmap = NULL;

	if (wcn_platform_chip_type() != WCN_PLATFORM_TYPE_PIKE2) {
		if (s_wcn_device.btwf_device) {
			rmap = s_wcn_device.btwf_device->rmap[REGMAP_PMU_APB];
		} else if (s_wcn_device.gnss_device) {
			rmap = s_wcn_device.gnss_device->rmap[REGMAP_PMU_APB];
		} else {
			WCN_ERR("no devices\n");
			return;
		}
		wcn_regmap_raw_write_bit(rmap, 0x1244, 1 << 0);
		WCN_INFO("finish!\n");
	}
}

/* The VDDCON default value is 1.6V, we should set it to 1.2v */
static void wcn_power_set_vddcon(u32 value)
{
	if (s_wcn_device.vddwcn != NULL)
		regulator_set_voltage(s_wcn_device.vddwcn,
				      value, value);
}

/*
 * NOTES:regulator function has compute-counter
 * We needn't judge GNSS and BTWF coxist case now.
 * But we should reserve the open status to debug.
 */
static int wcn_power_enable_vddcon(bool enable)
{
	int ret = 0;
	u32 btwf_open = false;
	u32 gnss_open = false;

	if (s_wcn_device.btwf_device &&
		s_wcn_device.btwf_device->wcn_open_status & WCN_MARLIN_MASK)
		btwf_open = true;
	if (s_wcn_device.gnss_device &&
		s_wcn_device.gnss_device->wcn_open_status & WCN_GNSS_ALL_MASK)
		gnss_open = true;

	mutex_lock(&(s_wcn_device.vddwcn_lock));
	if (s_wcn_device.vddwcn != NULL) {
		if (enable) {
			ret = regulator_enable(s_wcn_device.vddwcn);
			s_wcn_device.vddwcn_en_count++;
			if (wcn_platform_chip_type() ==
				WCN_PLATFORM_TYPE_SHARKLE)
				wcn_sys_ctrl_26m(true);
		} else if (regulator_is_enabled(s_wcn_device.vddwcn)) {
			ret = regulator_disable(s_wcn_device.vddwcn);
			s_wcn_device.vddwcn_en_count--;
			if ((wcn_platform_chip_type() ==
				WCN_PLATFORM_TYPE_SHARKLE)
				&& (s_wcn_device.vddwcn_en_count == 0)) {
				wcn_sys_ctrl_26m(false);
			}
		}

		WCN_INFO("enable=%d,en_count=%d,ret=%d,btwf=%d,gnss=%d\n",
			     enable, s_wcn_device.vddwcn_en_count,
			     ret, btwf_open, gnss_open);
		if (s_wcn_device.vddwcn_en_count > 2 ||
		    s_wcn_device.vddwcn_en_count < 0)
			WCN_ERR("vddwcn_en_count=%d",
				s_wcn_device.vddwcn_en_count);
	}
	mutex_unlock(&(s_wcn_device.vddwcn_lock));

	return ret;
}

/* The VDDCON default value is 1.6V, we should set it to 1.2v */
static void wcn_power_set_vddwifipa(u32 value)
{
	struct wcn_device *btwf_device = s_wcn_device.btwf_device;

	if (btwf_device->vddwifipa != NULL)
		regulator_set_voltage(btwf_device->vddwifipa,
				      value, value);
	WCN_INFO("value %d\n", value);
}

/* NOTES: wifipa: only used by WIFI module */
static int wcn_marlin_power_enable_vddwifipa(bool enable)
{
	int ret = 0;
	struct wcn_device *btwf_device = s_wcn_device.btwf_device;

	mutex_lock(&(btwf_device->vddwifipa_lock));
	if (btwf_device->vddwifipa != NULL) {
		if (enable)
			ret = regulator_enable(btwf_device->vddwifipa);
		else if (regulator_is_enabled(btwf_device->vddwifipa))
			ret = regulator_disable(btwf_device->vddwifipa);

		WCN_INFO("enable = %d, ret = %d\n", enable, ret);
	}
	mutex_unlock(&(btwf_device->vddwifipa_lock));

	return ret;
}

static void wcn_power_wq(struct work_struct *pwork)
{
	bool is_marlin;
	struct wcn_device *wcn_dev;
	struct delayed_work *ppower_wq;
	int ret;

	ppower_wq = container_of(pwork, struct delayed_work, work);
	wcn_dev = container_of(ppower_wq, struct wcn_device, power_wq);

	WCN_INFO("start boot :%s\n", wcn_dev->name);
	is_marlin = wcn_dev_is_marlin(wcn_dev);
	if (!is_marlin)
		gnss_clear_boot_flag();

	wcn_power_enable_vddcon(true);
	if (is_marlin) {
		/* ASIC: enable vddcon and wifipa interval time > 1ms */
		usleep_range(VDDWIFIPA_VDDCON_MIN_INTERVAL_TIME,
			VDDWIFIPA_VDDCON_MAX_INTERVAL_TIME);
		wcn_marlin_power_enable_vddwifipa(true);
	}

	wcn_power_enable_sys_domain(true);
	ret = wcn_proc_native_start(wcn_dev);

	WCN_INFO("finish %s!\n", ret ? "ERR" : "OK");
	complete(&wcn_dev->download_done);
}

void wcn_clear_ddr_gnss_cali_bit(void)
{
	phys_addr_t phy_addr;
	u32 value;
	struct wcn_device *wcn_dev;

	wcn_dev = s_wcn_device.btwf_device;
	if (wcn_dev) {
		value = GNSS_CALIBRATION_FLAG_CLEAR_ADDR_CP;
		phy_addr = wcn_dev->base_addr +
			   (phys_addr_t)&s_wssm_phy_offset_p->gnss_flag_addr;
		wcn_write_data_to_phy_addr(phy_addr, &value, sizeof(u32));
		WCN_INFO("set gnss flag off:0x%x\n", value);
	}
	wcn_dev = s_wcn_device.gnss_device;
	value = GNSS_CALIBRATION_FLAG_CLEAR_VALUE;
	phy_addr = wcn_dev->base_addr + GNSS_CALIBRATION_FLAG_CLEAR_ADDR;
	wcn_write_data_to_phy_addr(phy_addr, &value, sizeof(u32));
	WCN_INFO("clear gnss ddr bit\n");
}

void wcn_set_nognss(u32 val)
{
	phys_addr_t phy_addr;
	struct wcn_device *wcn_dev;

	wcn_dev = s_wcn_device.btwf_device;
	if (wcn_dev) {
		phy_addr = wcn_dev->base_addr +
			   (phys_addr_t)&s_wssm_phy_offset_p->include_gnss;
		wcn_write_data_to_phy_addr(phy_addr, &val, sizeof(u32));
		WCN_INFO("gnss:%u\n", val);
	}
}

static struct wcn_device *wcn_get_dev_by_type(u32 subsys_bit)
{
	if (subsys_bit & WCN_MARLIN_MASK)
		return s_wcn_device.btwf_device;
	else if ((subsys_bit & WCN_GNSS_MASK) ||
		 (subsys_bit & WCN_GNSS_BD_MASK))
		return s_wcn_device.gnss_device;

	WCN_ERR("invalid subsys:0x%x\n", subsys_bit);
	return NULL;
}

/* pre_str shuold't NULL */
static void wcn_show_dev_status(const char *pre_str)
{
	u32 status;

	if (s_wcn_device.btwf_device) {
		status = s_wcn_device.btwf_device->wcn_open_status;
		WCN_INFO("%s malrin status[%d] BT:%d FM:%d WIFI:%d MDBG:%d\n",
			 pre_str, status,
			 status & (1 << WCN_MARLIN_BLUETOOTH),
			 status & (1 << WCN_MARLIN_FM),
			 status & (1 << WCN_MARLIN_WIFI),
			 status & (1 << WCN_MARLIN_MDBG));
	}
	if (s_wcn_device.gnss_device) {
		status = s_wcn_device.gnss_device->wcn_open_status;
		WCN_INFO("%s gnss status[%d] GPS:%d GNSS_BD:%d\n",
			 pre_str, status, status & (1 << WCN_GNSS),
			 status & (1 << WCN_GNSS_BD));
	}
}

static int start_integrate_wcn_truely(u32 subsys)
{
	bool is_marlin;
	struct wcn_device *wcn_dev;
	u32 subsys_bit = 1 << subsys;

	WCN_INFO("start subsys:%d\n", subsys);
	wcn_dev = wcn_get_dev_by_type(subsys_bit);
	if (!wcn_dev) {
		WCN_ERR("wcn dev null!\n");
		return -EINVAL;
	}

	wcn_show_dev_status("before start");
	mutex_lock(&wcn_dev->power_lock);

	/* Check whether opened already */
	if (wcn_dev->wcn_open_status) {
		WCN_INFO("%s opened already = %d, subsys=%d!\n",
			 wcn_dev->name, wcn_dev->wcn_open_status, subsys);
		wcn_dev->wcn_open_status |= subsys_bit;
		wcn_show_dev_status("after start1");
		mutex_unlock(&wcn_dev->power_lock);
		return 0;
	}

	is_marlin = wcn_dev_is_marlin(wcn_dev);
	if (!is_marlin) {
		if (subsys_bit & WCN_GNSS_MASK) {
			strcpy(&wcn_dev->firmware_path_ext[0],
			       WCN_GNSS_FILENAME);
			s_wcn_device.gnss_type = WCN_GNSS_TYPE_GL;
			WCN_INFO("wcn gnss path=%s\n",
				&wcn_dev->firmware_path_ext[0]);
		} else {
			strcpy(&wcn_dev->firmware_path_ext[0],
			       WCN_GNSS_BD_FILENAME);
			s_wcn_device.gnss_type = WCN_GNSS_TYPE_BD;
			WCN_INFO("wcn bd path=%s\n",
				&wcn_dev->firmware_path_ext[0]);
		}
	}

	/* Not opened, so first open */
	init_completion(&wcn_dev->download_done);
	schedule_delayed_work(&wcn_dev->power_wq, 0);

	if (wait_for_completion_timeout(&wcn_dev->download_done,
		msecs_to_jiffies(MARLIN_WAIT_CP_INIT_MAX_TIME)) <= 0) {
		/* marlin download fail dump memory */
		if (is_marlin)
			goto err_boot_marlin;
		mutex_unlock(&wcn_dev->power_lock);
		return -1;
	} else if (wcn_dev->boot_cp_status) {
		if (wcn_dev->boot_cp_status == WCN_BOOT_CP2_ERR_DOWN_IMG) {
			mutex_unlock(&wcn_dev->power_lock);
			return -1;
		}
		if (is_marlin)
			goto err_boot_marlin;
	}

	wcn_dev->wcn_open_status |= subsys_bit;

	if (is_marlin) {
		power_state_notify(true);
		wcn_set_module_state();
	}
	mutex_unlock(&wcn_dev->power_lock);

	wcn_show_dev_status("after start2");

	return 0;

err_boot_marlin:
	mdbg_assert_interface("MARLIN boot cp timeout");
	/* warnning! fake status for poweroff in usr mode */
	wcn_dev->wcn_open_status |= subsys_bit;
	mutex_unlock(&wcn_dev->power_lock);

	return -1;
}

int start_integrate_wcn(u32 subsys)
{
	static u32 first_time;
	u32 btwf_subsys;
	u32 ret = 0;

	WCN_INFO("subsys:%d\n", subsys);
	mutex_lock(&marlin_lock);
	if (unlikely(wcn_get_carddump_status() != 0)) {
		WCN_ERR("in dump status subsys=%d!\n", subsys);
		mutex_unlock(&marlin_lock);
		return -1;
	}
	if (unlikely(get_loopcheck_status() >= 2)) {
		WCN_ERR("loopcheck status error subsys=%d!\n", subsys);
		mutex_unlock(&marlin_lock);
		return -1;
	}

	if (unlikely(!first_time)) {
		if (s_wcn_device.gnss_device) {
			/* clear ddr gps cali bit */
			wcn_set_nognss(WCN_INTERNAL_INCLUD_GNSS_VAL);
			wcn_clear_ddr_gnss_cali_bit();
			ret = start_integrate_wcn_truely(WCN_GNSS);
			if (ret) {
				mutex_unlock(&marlin_lock);
				return ret;
			}
		} else {
			wcn_set_nognss(WCN_INTERNAL_NOINCLUD_GNSS_VAL);
			WCN_INFO("not include gnss\n");
		}

		/* after cali,gnss powerdown itself,AP sync state by stop op */
		if (s_wcn_device.gnss_device)
			stop_integrate_wcn_truely(WCN_GNSS);
		first_time = 1;

		if (s_wcn_device.btwf_device) {
			if (subsys == WCN_GNSS || subsys == WCN_GNSS_BD)
				btwf_subsys = WCN_MARLIN_MDBG;
			else
				btwf_subsys = subsys;
			ret = start_integrate_wcn_truely(btwf_subsys);
			if (ret) {
				mutex_unlock(&marlin_lock);
				return ret;
			}
		}
		WCN_INFO("first time, start gnss and btwf\n");

		if (s_wcn_device.btwf_device &&
		    (subsys == WCN_GNSS || subsys == WCN_GNSS_BD))
			stop_integrate_wcn_truely(btwf_subsys);
		else {
			mutex_unlock(&marlin_lock);
			return 0;
		}
	}
	ret = start_integrate_wcn_truely(subsys);
	mutex_unlock(&marlin_lock);

	return ret;
}

int start_marlin(u32 subsys)
{
	return start_integrate_wcn(subsys);
}
EXPORT_SYMBOL_GPL(start_marlin);

/*
 * Fixme 1. Pike2, Sharkle, Sharkl3
 * Fixme 2. both of wcn and gnss case
 * force_sleep: 1 for send cmd, 0 for the old way
 */
static int wcn_wait_wcn_deep_sleep(struct wcn_device *wcn_dev, int force_sleep)
{
	u32 wait_sleep_count = 0;

	for (wait_sleep_count = 0;
	     wait_sleep_count < WCN_WAIT_SLEEP_MAX_COUNT;
	     wait_sleep_count++) {
		if (wcn_get_sleep_status(wcn_dev, force_sleep) == 0)
			break;
		msleep(20);
		WCN_INFO("wait_sleep_count=%d!\n",
			 wait_sleep_count);
	}

	return 0;
}

static int stop_integrate_wcn_truely(u32 subsys)
{
	bool is_marlin;
	struct wcn_device *wcn_dev;
	u32 subsys_bit = 1 << subsys;
	int force_sleep = 0;

	/* Check Parameter whether valid */
	wcn_dev = wcn_get_dev_by_type(subsys_bit);
	if (!wcn_dev) {
		WCN_ERR("wcn dev NULL: subsys=%d!\n", subsys);
		return -EINVAL;
	}

	wcn_show_dev_status("before stop");
	if (unlikely(!(subsys_bit & wcn_dev->wcn_open_status))) {
		/* It wants to stop not opened device */
		WCN_ERR("%s not opend, err: subsys = %d\n",
			wcn_dev->name, subsys);
		return -EINVAL;
	}

	is_marlin = wcn_dev_is_marlin(wcn_dev);

	mutex_lock(&wcn_dev->power_lock);
	wcn_dev->wcn_open_status &= ~subsys_bit;
	if (wcn_dev->wcn_open_status) {
		/* FIXme if we need it here */
		if (is_marlin)
			wcn_set_module_state();
		WCN_INFO("%s subsys(%d) close, and subsys(%d) opend\n",
			 wcn_dev->name, subsys, wcn_dev->wcn_open_status);
		wcn_show_dev_status("after stop1");
		mutex_unlock(&wcn_dev->power_lock);
		return 0;
	}

	WCN_INFO("%s do stop\n", wcn_dev->name);
	/* btwf use the send shutdown cp2 cmd way */
	if (is_marlin && !wcn_get_carddump_status())
		force_sleep = wcn_send_force_sleep_cmd(wcn_dev);
	/* the last module will stop,AP should wait CP2 sleep */
	wcn_wait_wcn_deep_sleep(wcn_dev, force_sleep);

	/* only one module works: stop CPU */
	wcn_proc_native_stop(wcn_dev);
	wcn_power_enable_sys_domain(false);

	if (is_marlin) {
		/* stop common resources if can disable it */
		wcn_marlin_power_enable_vddwifipa(false);
		/* ASIC: disable vddcon, wifipa interval time > 1ms */
		usleep_range(VDDWIFIPA_VDDCON_MIN_INTERVAL_TIME,
			     VDDWIFIPA_VDDCON_MAX_INTERVAL_TIME);
	}
	wcn_power_enable_vddcon(false);

	if (is_marlin)
		power_state_notify(false);

	wcn_sys_soft_reset();
	wcn_sys_soft_release();
	wcn_sys_deep_sleep_en();
	wcn_dev->power_state = WCN_POWER_STATUS_OFF;

	WCN_INFO("%s open_status = %d,power_state=%d,stop subsys=%d!\n",
		 wcn_dev->name, wcn_dev->wcn_open_status,
		 wcn_dev->power_state, subsys);

	if (is_marlin)
		wcn_set_module_state();
	mutex_unlock(&(wcn_dev->power_lock));

	wcn_show_dev_status("after stop2");

	return 0;
}

int stop_integrate_wcn(u32 subsys)
{
	u32 ret;

	mutex_lock(&marlin_lock);
	if (unlikely(wcn_get_carddump_status() != 0)) {
		WCN_ERR("in dump status subsys=%d!\n", subsys);
		mutex_unlock(&marlin_lock);
		return -1;
	}
	if (unlikely(get_loopcheck_status() >= 2)) {
		WCN_ERR("loopcheck status error subsys=%d!\n", subsys);
		mutex_unlock(&marlin_lock);
		return -1;
	}

	ret = stop_integrate_wcn_truely(subsys);
	mutex_unlock(&marlin_lock);

	return ret;
}

int stop_marlin(u32 subsys)
{
	return stop_integrate_wcn(subsys);
}
EXPORT_SYMBOL_GPL(stop_marlin);

void wcn_device_poweroff(void)
{
	u32 i;

	mutex_lock(&marlin_lock);

	for (i = 0; i < WCN_GNSS_ALL; i++)
		stop_integrate_wcn_truely(i);

	if (marlin_reset_func != NULL)
		marlin_reset_func(marlin_callback_para);

	mutex_unlock(&marlin_lock);
	WCN_INFO("reset finish!\n");
}
static int wcn_probe(struct platform_device *pdev)
{
	struct wcn_device *wcn_dev;
	static int first = 1;

	WCN_INFO("start!\n");

	wcn_dev = kzalloc(sizeof(struct wcn_device), GFP_KERNEL);
	if (!wcn_dev)
		return -ENOMEM;

	if (wcn_parse_dt(pdev, wcn_dev) < 0) {
		WCN_ERR("wcn_parse_dt Failed!\n");
		kfree(wcn_dev);
		return -EINVAL;
	}

	/* init the regs which can be init in the driver probe */
	wcn_config_ctrlreg(wcn_dev, 0, wcn_dev->ctrl_probe_num);

	mutex_init(&(wcn_dev->power_lock));

	wcn_platform_fs_init(wcn_dev);

	platform_set_drvdata(pdev, (void *)wcn_dev);

	if (strcmp(wcn_dev->name, WCN_MARLIN_DEV_NAME) == 0)
		s_wcn_device.btwf_device = wcn_dev;
	else if (strcmp(wcn_dev->name, WCN_GNSS_DEV_NAME) == 0)
		s_wcn_device.gnss_device = wcn_dev;

	/* default vddcon is 1.6V, we should set it to 1.2v */
	if (s_wcn_device.vddcon_voltage_setted == false) {
		s_wcn_device.vddcon_voltage_setted = true;
		wcn_power_set_vddcon(WCN_VDDCON_WORK_VOLTAGE);
		mutex_init(&(s_wcn_device.vddwcn_lock));
	}

	if (strcmp(wcn_dev->name, WCN_MARLIN_DEV_NAME) == 0) {
		mutex_init(&(wcn_dev->vddwifipa_lock));
		if (wcn_platform_chip_id() == AON_CHIP_ID_AA)
			wcn_power_set_vddwifipa(WCN_VDDWIFIPA_WORK_VOLTAGE);
		wcn_global_source_init();
		mdbg_init();
		wcn_marlin_write_efuse();
	} else if (strcmp(wcn_dev->name, WCN_GNSS_DEV_NAME) == 0)
		gnss_write_efuse_data();

	INIT_DELAYED_WORK(&wcn_dev->power_wq, wcn_power_wq);

	if (first) {
		/* Transceiver can't get into LP, so force deep sleep */
		if ((wcn_platform_chip_type() == WCN_PLATFORM_TYPE_SHARKLE) ||
		    (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_SHARKL3)) {
			wcn_sys_soft_release();
			wcn_sys_deep_sleep_en();
		}
		first = 0;
	}

#if WCN_INTEGRATE_PLATFORM_DEBUG
	wcn_codes_debug();
#endif

	WCN_INFO("finish!\n");

	return 0;
}

static int  wcn_remove(struct platform_device *pdev)
{
	struct wcn_device *wcn_dev = platform_get_drvdata(pdev);

	if (wcn_dev)
		WCN_INFO("dev name %s\n", wcn_dev->name);

	wcn_platform_fs_exit(wcn_dev);
	kfree(wcn_dev);
	wcn_dev = NULL;

	return 0;
}

static void wcn_shutdown(struct platform_device *pdev)
{
	struct wcn_device *wcn_dev = platform_get_drvdata(pdev);

	if (wcn_dev && wcn_dev->wcn_open_status) {
		/* CPU hold on */
		wcn_proc_native_stop(wcn_dev);
		/* wifipa power off */
		if (wcn_dev_is_marlin(wcn_dev)) {
			wcn_marlin_power_enable_vddwifipa(false);
			/* ASIC: disable vddcon, wifipa interval time > 1ms */
			usleep_range(VDDWIFIPA_VDDCON_MIN_INTERVAL_TIME,
				VDDWIFIPA_VDDCON_MAX_INTERVAL_TIME);
		}
		/* vddcon power off */
		wcn_power_enable_vddcon(false);
		wcn_sys_soft_reset();
		wcn_sys_soft_release();
		wcn_sys_deep_sleep_en();
		WCN_INFO("dev name %s\n", wcn_dev->name);
	}
}

static SIMPLE_DEV_PM_OPS(wcn_pm_ops, wcn_suspend, wcn_resume);
static struct platform_driver wcn_driver = {
	.driver = {
		.name = "wcn_integrate_platform",
		.pm = &wcn_pm_ops,
		.of_match_table = wcn_match_table,
	},
	.probe = wcn_probe,
	.remove = wcn_remove,
	.shutdown = wcn_shutdown,
};

static int __init wcn_init(void)
{
	WCN_INFO("entry!\n");

	return platform_driver_register(&wcn_driver);
}
late_initcall(wcn_init);

static void __exit wcn_exit(void)
{
	platform_driver_unregister(&wcn_driver);
}
module_exit(wcn_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Spreadtrum  WCN Integrate Platform Driver");
MODULE_AUTHOR("YaoGuang Chen <yaoguang.chen@spreadtrum.com>");
