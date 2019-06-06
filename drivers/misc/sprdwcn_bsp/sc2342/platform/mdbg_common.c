/*
* Copyright (C) 2015 Spreadtrum Communications Inc.
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/

#include <linux/marlin_platform.h>
#include <linux/sdiom_rx_api.h>
#include <linux/sdiom_tx_api.h>

#include "mdbg_common.h"
#include "mdbg_ring.h"
#include "mdbg_main.h"
#include "wcn_bus.h"
#include "wcn_debug.h"
#include "wcn_glb_reg.h"
#include "wcn_misc.h"

#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
#include "mdbg_sipc.h"
#include "linux/wcn_integrate_platform.h"
#endif

struct ring_device *ring_dev;

static int long flag_smp;
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
static int g_dump_status;
gnss_dump_callback gnss_dump_handle;
#endif

bool mdbg_get_download_status(void)
{
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
	return wcn_get_download_status();
#else
	return marlin_get_download_status();
#endif
}

int mdbg_get_module_status(void)
{
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
	return wcn_get_module_status();
#else
	return marlin_get_module_status();
#endif
}

int mdbg_get_module_status_changed(void)
{
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
	return wcn_get_module_status_changed();
#else
	return marlin_get_module_status_changed();
#endif
}

void wcn_hold_cpu(void)
{
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
	mdbg_hold_cpu();
#else
	marlin_hold_cpu();
#endif
}

unsigned int wcn_set_carddump_status(unsigned int flag)
{
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
	g_dump_status = flag;
	return 0;
#else
	sprdwcn_bus_set_carddump_status(flag);
	return 0;
#endif
}

unsigned int wcn_get_carddump_status(void)
{
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
	return g_dump_status;
#else
	return sprdwcn_bus_get_carddump_status();
#endif
}

unsigned long long mdbg_get_rx_total_cnt(void)
{
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
	return wcn_get_cp2_comm_rx_count();
#else
	return sprdwcn_bus_get_rx_total_cnt();
#endif
}

int mdbg_read_release(unsigned int fifo_id)
{
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
#else
	sprdwcn_bus_pt_read_release(fifo_id);
#endif
	return 0;
}

long mdbg_content_len(void)
{
	if (unlikely(!ring_dev))
		return 0;

	return mdbg_ring_content_len(ring_dev->ring);
}

#ifndef CONFIG_INTEGRATED_MARLIN2_GE2
static unsigned int mdbg_tx_cb(void *addr)
{
	kfree(addr);

	return 0;
}
#endif

static int smp_calc_chsum(unsigned short *buf, unsigned int size)
{
	unsigned long int cksum = 0;
	unsigned short data;

	while (size > 1) {
		data = *buf;
		buf++;
		cksum += data;
		size -= sizeof(unsigned short);
	}

	if (size)
		cksum += *buf & 0xff;

	while (cksum >> 16)
		cksum = (cksum >> 16) + (cksum & 0xffff);

	return (unsigned short)(~cksum);
}

static int mdbg_write_smp_head(unsigned int len)
{
	struct smp_head *smp;
	unsigned char *smp_buf, *tmp;
	int smp_len;

	smp_len = sizeof(struct smp_head) + sizeof(struct sme_head_tag);
	smp_buf = kmalloc(smp_len, GFP_KERNEL);
	if (!smp_buf)
		return -ENOMEM;

	/* Smp header */
	smp = (struct smp_head *)smp_buf;
	smp->sync_code = SMP_HEADERFLAG;
	smp->length = smp_len + len - SYSNC_CODE_LEN;
	smp->channel_num = SMP_DSP_CHANNEL_NUM;
	smp->packet_type = SMP_DSP_TYPE;
	smp->reserved = SMP_RESERVEDFLAG;
	smp->check_sum = smp_calc_chsum(&smp->length, sizeof(struct smp_head)
		- SYSNC_CODE_LEN - CHKSUM_LEN);

	/* Diag header: Needs use these bytes for ARM log tool,
	 * And it need't 0x7e head and without 0x7e tail
	 */
	tmp = smp_buf + sizeof(struct smp_head);
	((struct sme_head_tag *)tmp)->seq_num = 0;
	((struct sme_head_tag *)tmp)->len = smp_len
		+ len - sizeof(struct smp_head);
	((struct sme_head_tag *)tmp)->type = SMP_DSP_TYPE;
	((struct sme_head_tag *)tmp)->subtype = SMP_DSP_DUMP_TYPE;

	mdbg_ring_write(ring_dev->ring, smp_buf, smp_len);

	kfree(smp_buf);

	return 0;
}

static long int mdbg_comm_write(char *buf,
				long int len, unsigned int subtype)
{
	unsigned char *send_buf = NULL;
	char *str = NULL;

#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
	if (unlikely(wcn_get_download_status() != true))
		return -EIO;
#else
	if (unlikely(marlin_get_download_status() != true))
		return -EIO;
#endif

	send_buf = kzalloc(MDBG_WRITE_SIZE, GFP_KERNEL);
	if (!send_buf)
		return -ENOMEM;
	memcpy(send_buf, buf, len);
	WCN_INFO("len:%ld,\n", len);
	str = strstr(send_buf, SMP_HEAD_STR);
	if (!str)
		str = strstr(send_buf + ARMLOG_HEAD, SMP_HEAD_STR);
	if (str) {
		int err;

		WCN_INFO("len:%ld,str:%s\n", len, str);
		str[sizeof(SMP_HEAD_STR)] = 0;
		err = kstrtol(&str[sizeof(SMP_HEAD_STR) - 1], 10, &flag_smp);
		WCN_INFO("err:%d, flag_smp:%ld\n", err, flag_smp);
		kfree(send_buf);
	} else {
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
		mdbg_sipc_type_write(send_buf, len, subtype);
		kfree(send_buf);
#else
		sprdwcn_bus_pt_write(send_buf, len, MDBG_SDIO_PACKER_TYPE,
				     subtype);
#endif
	}
	return len;
}

static void mdbg_ring_rx_task(unsigned long data)
{
	struct ring_rx_data *rx = NULL;
	struct mdbg_ring_t *ring = NULL;

	if (unlikely(!ring_dev)) {
		WCN_ERR("ring_dev is NULL\n");
		return;
	}

	spin_lock_bh(&ring_dev->rw_lock);
	rx = list_first_entry_or_null(&ring_dev->rx_head,
				      struct ring_rx_data, entry);
	if (rx) {
		list_del(&rx->entry);
	} else {
		WCN_ERR("tasklet something err\n");
		spin_unlock_bh(&ring_dev->rw_lock);
		return;
	}
	if (!list_empty(&ring_dev->rx_head))
		tasklet_schedule(&ring_dev->rx_task);
	ring = ring_dev->ring;
	spin_unlock_bh(&ring_dev->rw_lock);
	mdbg_ring_write(ring, rx->addr, rx->len);
	wake_up_interruptible(&mdbg_wait);
	wake_up_interruptible(&mdbg_dev->rxwait);
	mdbg_read_release(rx->fifo_id);
	kfree(rx);
}

void mdbg_log_read(void *addr, unsigned int len,
		   unsigned int fifo_id)
{
	struct ring_rx_data *rx;

	if ((functionmask[7] & CP2_FLAG_YLOG) == 1) {
		log_rx_callback(addr, len, fifo_id);
		return;
	}
	if (ring_dev) {
		mutex_lock(&ring_dev->mdbg_read_mutex);
		rx = kmalloc(sizeof(*rx), GFP_KERNEL);
		if (!rx) {
			WCN_ERR("mdbg ring low memory\n");
			mutex_unlock(&ring_dev->mdbg_read_mutex);
			mdbg_read_release(fifo_id);
			return;
		}
		mutex_unlock(&ring_dev->mdbg_read_mutex);
		spin_lock_bh(&ring_dev->rw_lock);
		rx->addr	= (unsigned char *)addr;
		rx->len		= len;
		rx->fifo_id	= fifo_id;
		list_add_tail(&rx->entry, &ring_dev->rx_head);
		tasklet_schedule(&ring_dev->rx_task);
		spin_unlock_bh(&ring_dev->rw_lock);
	}
}
EXPORT_SYMBOL_GPL(mdbg_log_read);

long int mdbg_send(char *buf, long int len, unsigned int subtype)
{
	long int sent_size = 0;

	MDBG_LOG("BYTE MODE");
	wake_lock(&ring_dev->rw_wake_lock);
	sent_size = mdbg_comm_write(buf, len, subtype);
	wake_unlock(&ring_dev->rw_wake_lock);

	return sent_size;
}

long int mdbg_receive(void *buf, long int len)
{
	return mdbg_ring_read(ring_dev->ring, buf, len);
}

#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
static int mdbg_snap_shoot_iram_data(void *buf, u32 addr, u32 len)
{
	struct regmap *regmap;
	u32 i;
	u8 *ptr = NULL;

	WCN_INFO("start snap_shoot iram data!addr:%x,len:%d", addr, len);
	if (mdbg_get_module_status() == 0) {
		WCN_ERR("module status off:can not get iram data!\n");
		return -1;
	}

	if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_SHARKL3)
		regmap = wcn_get_btwf_regmap(REGMAP_WCN_REG);
	else
		regmap = wcn_get_btwf_regmap(REGMAP_ANLG_WRAP_WCN);
	wcn_regmap_raw_write_bit(regmap, 0XFF4, addr);
	for (i = 0; i < len / 4; i++) {
		ptr = buf + i * 4;
		wcn_regmap_read(regmap, 0XFFC, (u32 *)ptr);
	}
	WCN_INFO("snap_shoot iram data success\n");

	return 0;
}

int mdbg_snap_shoot_iram(void *buf)
{
	u32 ret;

	ret = mdbg_snap_shoot_iram_data(buf,
			0x18000000, 1024 * 32);

	return ret;
}

void mdbg_hold_cpu(void)
{
	struct regmap *regmap;
	u32 value;
	phys_addr_t init_addr;

	if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_SHARKL3)
		regmap = wcn_get_btwf_regmap(REGMAP_WCN_REG);
	else
		regmap = wcn_get_btwf_regmap(REGMAP_ANLG_WRAP_WCN);
	wcn_regmap_read(regmap, 0X20, &value);
	value |= 1 << 3;
	wcn_regmap_raw_write_bit(regmap, 0X20, value);

	wcn_regmap_read(regmap, 0X24, &value);
	value |= 1 << 2;
	wcn_regmap_raw_write_bit(regmap, 0X24, value);

	value = MDBG_CACHE_FLAG_VALUE;
	init_addr = wcn_get_btwf_init_status_addr();
	wcn_write_data_to_phy_addr(init_addr, (void *)&value, 4);
	value = 0;
	wcn_regmap_raw_write_bit(regmap, 0X20, value);
	wcn_regmap_raw_write_bit(regmap, 0X24, value);
	msleep(200);
}

struct wcn_dump_mem_reg {
	/* some CP regs can't dump */
	bool do_dump;
	u32 addr;
	/* 4 btyes align */
	u32 len;
};

/* magic number, not change it */
#define WCN_DUMP_VERSION_NAME "WCN_DUMP_HEAD__"
/* SUB_NAME len not more than 15 bytes */
#define WCN_DUMP_VERSION_SUB_NAME "SIPC_23xx"
/* CP2 iram start and end */
#define WCN_DUMP_CP2_IRAM_START 1
#define WCN_DUMP_CP2_IRAM_END 2
/* AP regs start and end */
#define WCN_DUMP_AP_REGS_START (WCN_DUMP_CP2_IRAM_END + 1)
#define WCN_DUMP_AP_REGS_END 9
/* CP2 regs start and end */
#define WCN_DUMP_CP2_REGS_START (WCN_DUMP_AP_REGS_END + 1)
#define WCN_DUMP_CP2_REGS_END (ARRAY_SIZE(s_wcn_dump_regs) - 1)

#define WCN_DUMP_ALIGN(x) (((x) + 3) & ~3)
/* used for HEAD, so all dump mem in this array.
 * if new member added, please modify the macor XXX_START XXX_end above.
 */
static struct wcn_dump_mem_reg s_wcn_dump_regs[] = {
	/* share mem */
	{1, 0, 0x300000},
	/* iram mem */
	{1, 0x10000000, 0x8000}, /* wcn iram */
	{1, 0x18004000, 0x4000}, /* gnss iram */
	/* ap regs */
	{1, 0x402B00CC, 4}, /* PMU_SLEEP_CTRL */
	{1, 0x402B00D4, 4}, /* PMU_SLEEP_STATUS */
	{1, 0x402B0100, 4}, /* PMU_PD_WCN_SYS_CFG */
	{1, 0x402B0104, 4}, /* PMU_PD_WIFI_WRAP_CFG */
	{1, 0x402B0244, 4}, /* PMU_WCN_SYS_DSLP_ENA */
	{1, 0x402B0248, 4}, /* PMU_WIFI_WRAP_DSLP_ENA */
	{1, 0x402E057C, 4}, /* AON_APB_WCN_SYS_CFG2 */
	/* cp regs */
	{0, 0x40060000, 0x300}, /* BTWF_CTRL */
	{0, 0x60300000, 0x400}, /* BTWF_AHB_CTRL */
	{0, 0x40010000, 0x38},  /* BTWF_INTC */
	{0, 0x40020000, 0x10},  /* BTWF_SYSTEM_TIMER */
	{0, 0x40030000, 0x20},  /* BTWF_TIMER0 */
	{0, 0x40030020, 0x20},  /* BTWF_TIMER1 */
	{0, 0x40030040, 0x20},  /* BTWF_TIMER2 */
	{0, 0x40040000, 0x24},  /* BTWF_WATCHDOG */
	{0, 0xd0010000, 0xb4},  /* COM_AHB_CTRL */
	{0, 0xd0020800, 0xc},   /* MANU_CLK_CTRL */
	{0, 0x70000000, 0x10000},  /* WIFI */
	{0, 0x400b0000, 0x850},    /* FM */
	{0, 0x60700000, 0x400},    /* BT_CMD */
	{0, 0x60740000, 0xa400}    /* BT */
};

struct wcn_dump_section_info {
	/* cp load start addr */
	__le32 start;
	/* cp load end addr */
	__le32 end;
	/* load from file offset */
	__le32 off;
	__le32 reserv;
} __packed;

struct wcn_dump_head_info {
	/* WCN_DUMP_VERSION_NAME */
	u8 version[16];
	/* WCN_DUMP_VERSION_SUB_NAME */
	u8 sub_version[16];
	/* numbers of wcn_dump_section_info */
	__le32 n_sec;
	/* used to check if dump is full */
	__le32 file_size;
	u8 reserv[8];
	struct wcn_dump_section_info section[0];
} __packed;

static int wcn_fill_dump_head_info(struct wcn_dump_mem_reg *mem_cfg, int cnt)
{
	int i, len;
	struct wcn_dump_mem_reg *mem;
	struct wcn_dump_head_info *head;
	struct wcn_dump_section_info *sec;

	len = sizeof(*head) + sizeof(*sec) * cnt;
	head = kzalloc(len, GFP_KERNEL);
	if (unlikely(!head)) {
		WCN_ERR("system has no mem for dump mem\n");
		return -1;
	}

	strncpy(head->version, WCN_DUMP_VERSION_NAME,
		strlen(WCN_DUMP_VERSION_NAME));
	strncpy(head->sub_version, WCN_DUMP_VERSION_SUB_NAME,
		strlen(WCN_DUMP_VERSION_SUB_NAME));
	head->n_sec = cpu_to_le32(cnt);
	sec = head->section;
	for (i = 0; i < cnt; i++) {
		sec = head->section + i;
		mem = mem_cfg + i;
		sec->off = cpu_to_le32(WCN_DUMP_ALIGN(len));
		sec->start = cpu_to_le32(mem->addr);
		sec->end = cpu_to_le32(sec->start + mem->len - 1);
		len += mem->len;
		WCN_INFO("section[%d] [0x%x 0x%x 0x%x]\n",
			 i, le32_to_cpu(sec->start),
			 le32_to_cpu(sec->end), le32_to_cpu(sec->off));
	}
	head->file_size = cpu_to_le32(len + strlen(WCN_DUMP_END_STRING));

	mdbg_ring_write(ring_dev->ring, head,
			sizeof(*head) + sizeof(*sec) * cnt);
	wake_up_interruptible(&mdbg_wait);
	wake_up_interruptible(&mdbg_dev->rxwait);
	kfree(head);

	return 0;
}

static void mdbg_dump_str(char *str, int str_len)
{
	if (!str)
		return;

	mdbg_ring_write(ring_dev->ring, str, str_len);
	wake_up_interruptible(&mdbg_wait);
	wake_up_interruptible(&mdbg_dev->rxwait);
	WCN_INFO("dump str finish!");
}

static int mdbg_dump_ap_register_data(phys_addr_t addr, u32 len)
{
	u32 value = 0;
	u8 *ptr = NULL;

	ptr = (u8 *)&value;
	wcn_read_data_from_phy_addr(addr, &value, len);
	mdbg_ring_write(ring_dev->ring, ptr, len);
	wake_up_interruptible(&mdbg_wait);
	wake_up_interruptible(&mdbg_dev->rxwait);

	return 0;
}

static int mdbg_dump_cp_register_data(u32 addr, u32 len)
{
	struct regmap *regmap;
	u32 i;
	u32 count, trans_size;
	u8 *buf = NULL;
	u8 *ptr = NULL;

	WCN_INFO("start dump cp register!addr:%x,len:%d", addr, len);
	if (unlikely(!ring_dev)) {
		WCN_ERR("ring_dev is NULL\n");
		return -1;
	}

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (wcn_platform_chip_type() == WCN_PLATFORM_TYPE_SHARKL3)
		regmap = wcn_get_btwf_regmap(REGMAP_WCN_REG);
	else
		regmap = wcn_get_btwf_regmap(REGMAP_ANLG_WRAP_WCN);

	wcn_regmap_raw_write_bit(regmap, 0XFF4, addr);
	for (i = 0; i < len / 4; i++) {
		ptr = buf + i * 4;
		wcn_regmap_read(regmap, 0XFFC, (u32 *)ptr);
	}
	count = 0;
	while (count < len) {
		trans_size = (len - count) > DUMP_PACKET_SIZE ?
			DUMP_PACKET_SIZE : (len - count);
		mdbg_ring_write(ring_dev->ring, buf + count, trans_size);
		count += trans_size;
		wake_up_interruptible(&mdbg_wait);
		wake_up_interruptible(&mdbg_dev->rxwait);
	}

	kfree(buf);
	WCN_INFO("dump cp register finish count %u\n", count);

	return count;
}

static void mdbg_dump_ap_register(struct wcn_dump_mem_reg *mem)
{
	int i;

	for (i = WCN_DUMP_AP_REGS_START; i <= WCN_DUMP_AP_REGS_END; i++) {
		mdbg_dump_ap_register_data(mem[i].addr, mem[i].len);
		WCN_INFO("dump ap reg section[%d] ok!\n", i);
	}
}

static void mdbg_dump_cp_register(struct wcn_dump_mem_reg *mem)
{
	int i, count;

	for (i = WCN_DUMP_CP2_REGS_START; i <= WCN_DUMP_CP2_REGS_END; i++) {
		count = mdbg_dump_cp_register_data(mem[i].addr, mem[i].len);
		WCN_INFO("dump cp reg section[%d] %d ok!\n", i, count);
	}
}

static void mdbg_dump_iram(struct wcn_dump_mem_reg *mem)
{
	int i, count;

	for (i = WCN_DUMP_CP2_IRAM_START; i <= WCN_DUMP_CP2_IRAM_END; i++) {
		count = mdbg_dump_cp_register_data(mem[i].addr, mem[i].len);
		WCN_INFO("dump iram section[%d] %d ok!\n", i, count);
	}
}

static int mdbg_dump_share_memory(struct wcn_dump_mem_reg *mem)
{
	u32 count, len, trans_size;
	void *virt_addr;
	phys_addr_t base_addr;
	u32 time = 0;
	unsigned int cnt;

	if (unlikely(!ring_dev)) {
		WCN_ERR("ring_dev is NULL\n");
		return -1;
	}
	len = mem[0].len;
	base_addr = wcn_get_btwf_base_addr();
	WCN_INFO("dump sharememory start!");
	WCN_INFO("ring->pbuff=%p, ring->end=%p.\n",
		 ring_dev->ring->pbuff, ring_dev->ring->end);
	virt_addr = wcn_mem_ram_vmap_nocache(base_addr, len, &cnt);
	if (!virt_addr) {
		WCN_ERR("wcn_mem_ram_vmap_nocache fail\n");
		return -1;
	}
	count = 0;
	while (count < len) {
		trans_size = (len - count) > DUMP_PACKET_SIZE ?
			DUMP_PACKET_SIZE : (len - count);
		/* copy data from ddr to ring buf  */
		if (flag_smp == 1)
			mdbg_write_smp_head(trans_size);
		mdbg_ring_write(ring_dev->ring, virt_addr + count, trans_size);
		count += trans_size;
		wake_up_interruptible(&mdbg_wait);
		wake_up_interruptible(&mdbg_dev->rxwait);

		if (mdbg_ring_over_loop(
			ring_dev->ring, trans_size, MDBG_RING_W)) {
			WCN_INFO("ringbuf overloop:wait for read\n");
			while (!mdbg_ring_over_loop(
				ring_dev->ring, trans_size, MDBG_RING_R)) {
				msleep(DUMP_WAIT_TIMEOUT);
				time++;
				WCN_INFO("wait time %d\n", time);
				wake_up_interruptible(&mdbg_wait);
				wake_up_interruptible(&mdbg_dev->rxwait);
				if (time > DUMP_WAIT_COUNT) {
					WCN_INFO("ringbuf overloop timeout!\n");
					break;
				}
			}
		}
	}
	wcn_mem_ram_unmap(virt_addr, cnt);
	WCN_INFO("share memory dump finish! total count %u\n", count);

	return 0;
}

void mdbg_dump_gnss_register(gnss_dump_callback callback_func, void *para)
{
	gnss_dump_handle = (gnss_dump_callback)callback_func;
	WCN_INFO("gnss_dump register success!\n");
}

void mdbg_dump_gnss_unregister(void)
{
	gnss_dump_handle = NULL;
}

static int btwf_dump_mem(void)
{
	u32 cp2_status = 0;
	phys_addr_t sleep_addr;

	if (wcn_get_btwf_power_status() == WCN_POWER_STATUS_OFF) {
		WCN_INFO("wcn power status off:can not dump btwf!\n");
		return -1;
	}

	mdbg_send_atcmd("at+sleep_switch=0\r",
			strlen("at+sleep_switch=0\r"),
			WCN_ATCMD_KERNEL);
	msleep(500);
	sleep_addr = wcn_get_btwf_sleep_addr();
	wcn_read_data_from_phy_addr(sleep_addr, &cp2_status, sizeof(u32));
	mdbg_hold_cpu();
	msleep(100);
	mdbg_ring_reset(ring_dev->ring);
	mdbg_atcmd_clean();
	if (wcn_fill_dump_head_info(s_wcn_dump_regs,
				    ARRAY_SIZE(s_wcn_dump_regs)))
		return -1;
	mdbg_dump_share_memory(s_wcn_dump_regs);
	mdbg_dump_iram(s_wcn_dump_regs);
	mdbg_dump_ap_register(s_wcn_dump_regs);
	if (cp2_status == WCN_CP2_STATUS_DUMP_REG) {
		mdbg_dump_cp_register(s_wcn_dump_regs);
		WCN_INFO("dump cp2 register ok!\n");
	}

	mdbg_dump_str(WCN_DUMP_END_STRING, strlen(WCN_DUMP_END_STRING));

	return 0;
}
#else
static int mdbg_dump_data(unsigned int start_addr,
			  char *str, int len, int str_len)
{
	unsigned char *buf;
	int count, trans_size, err = 0, i, prin_temp = 0;

	if (unlikely(!ring_dev)) {
		WCN_ERR("mdbg_dump ring_dev is NULL\n");
		return -1;
	}

	if (str) {
		msleep(20);
		WCN_INFO("mdbg str_len:%d\n", str_len);
		if (flag_smp == 1)
			mdbg_write_smp_head(str_len);
		mdbg_ring_write(ring_dev->ring, str, str_len);
		wake_up_interruptible(&mdbg_wait);
		wake_up_interruptible(&mdbg_dev->rxwait);
	}

	if (len == 0)
		return 0;

	buf = kmalloc(DUMP_PACKET_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	count = 0;
	while (count < len) {
		trans_size = (len - count) > DUMP_PACKET_SIZE ?
			DUMP_PACKET_SIZE : (len - count);
		err = sprdwcn_bus_direct_read(start_addr + count, buf,
					      trans_size);
		if (err < 0) {
			WCN_ERR("dump memory error:%d\n", err);
			goto out;
		}
		if (prin_temp == 0) {
			prin_temp = 1;
			for (i = 0; i < 5; i++)
				WCN_ERR("mdbg *****buf[%d]:0x%x\n",
				       i, buf[i]);
			}
		if (flag_smp == 1)
			mdbg_write_smp_head(trans_size);
		mdbg_ring_write(ring_dev->ring, buf, trans_size);
		count += trans_size;
		wake_up_interruptible(&mdbg_wait);
		wake_up_interruptible(&mdbg_dev->rxwait);
	}

out:
	kfree(buf);

	return count;
}
#endif

void mdbg_clear_log(void)
{
	mdbg_ring_clear(ring_dev->ring);
}

int mdbg_dump_mem(void)
{
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
	/* dump gnss */
	if (gnss_dump_handle) {
		WCN_INFO("need dump gnss\n");
		gnss_dump_handle();
	}

	/* dump btwf */
	btwf_dump_mem();
#else
	long int count;
	int ret;

	wcn_hold_cpu();
	msleep(100);
	mdbg_clear_log();

	count = mdbg_dump_data(CP_START_ADDR, NULL, FIRMWARE_MAX_SIZE, 0);
	if (count <= 0) {
		WCN_INFO("mdbg start reset marlin reg!\n");
		ret = marlin_reset_reg();
		if (ret < 0)
			return 0;

		count = mdbg_dump_data(CP_START_ADDR, NULL,
				       FIRMWARE_MAX_SIZE, 0);

		WCN_INFO("mdbg only dump ram %ld ok!\n", count);

		goto end;
	}
	WCN_INFO("mdbg dump ram %ld ok!\n", count);

	count = mdbg_dump_data(DUMP_SDIO_ADDR, "start_dump_sdio_reg",
			       DUMP_SDIO_ADDR_SIZE,
			      strlen("start_dump_sdio_reg"));
	WCN_INFO("mdbg dump sdio %ld ok!\n", count);

	count = mdbg_dump_data(DUMP_INTC_ADDR, "start_dump_intc_reg",
			       DUMP_REG_SIZE,
			       strlen("start_dump_intc_reg"));
	WCN_INFO("mdbg dump intc %ld ok!\n", count);

	count = mdbg_dump_data(DUMP_SYSTIMER_ADDR, "start_dump_systimer_reg",
			       DUMP_REG_SIZE,
			       strlen("start_dump_systimer_reg"));
	WCN_INFO("mdbg dump systimer %ld ok!\n", count);

	count = mdbg_dump_data(DUMP_WDG_ADDR, "start_dump_wdg_reg",
		DUMP_REG_SIZE, strlen("start_dump_wdg_reg"));
	WCN_INFO("mdbg dump wdg %ld ok!\n", count);

	count = mdbg_dump_data(DUMP_APB_ADDR, "start_dump_apb_reg",
		DUMP_REG_SIZE, strlen("start_dump_apb_reg"));
	WCN_INFO("mdbg dump apb %ld ok!\n", count);

	count = mdbg_dump_data(DUMP_DMA_ADDR, "start_dump_dma_reg",
		DUMP_REG_SIZE, strlen("start_dump_dma_reg"));
	WCN_INFO("mdbg dump dma %ld ok!\n", count);

	count = mdbg_dump_data(DUMP_AHB_ADDR, "start_dump_ahb_reg",
		DUMP_REG_SIZE, strlen("start_dump_ahb_reg"));
	WCN_INFO("mdbg dump ahb %ld ok!\n", count);

	count = mdbg_dump_data(DUMP_FM_ADDR, "start_dump_fm_reg",
		DUMP_REG_SIZE, strlen("start_dump_fm_reg"));
	WCN_INFO("mdbg dump fm %ld ok!\n", count);

	count = mdbg_dump_data(DUMP_WIFI_ADDR, "start_dump_wifi_reg",
		DUMP_WIFI_ADDR_SIZE, strlen("start_dump_wifi_reg"));
	WCN_INFO("mdbg dump wifi %ld ok!\n", count);

	count = mdbg_dump_data(DUMP_BT_CMD_ADDR, "start_dump_bt_cmd_buf",
		DUMP_BT_CMD_ADDR_SIZE, strlen("start_dump_bt_cmd_buf"));
	WCN_INFO("mdbg dump bt cmd %ld ok!\n", count);

	count = mdbg_dump_data(DUMP_BT_ADDR, "start_dump_bt_reg",
		DUMP_BT_ADDR_SIZE, strlen("start_dump_bt_reg"));
	WCN_INFO("mdbg dump bt %ld ok!\n", count);

end:
	count = mdbg_dump_data(0, WCN_DUMP_END_STRING,
			       0, strlen(WCN_DUMP_END_STRING));

	WCN_INFO("mdbg dump memory finish\n");
#endif
	if ((functionmask[7] & CP2_FLAG_YLOG) == 1)
		complete(&dumpmem_complete);
	return 0;
}

static int mdbg_pt_ring_reg(void)
{
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
	mdbg_sipc_cb_register(MDBG_SUBTYPE_RING, mdbg_log_read);
#else
	sprdwcn_bus_register_pt_rx_process(MDBG_SDIO_PACKER_TYPE,
			MDBG_SUBTYPE_RING, mdbg_log_read);
	sprdwcn_bus_register_pt_tx_release(MDBG_SDIO_PACKER_TYPE,
			MDBG_SUBTYPE_RING, mdbg_tx_cb);
#endif

	return 0;
}

int mdbg_pt_common_reg(unsigned int subtype, void *func)
{
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
	mdbg_sipc_cb_register(subtype, func);
#else
	sprdwcn_bus_register_pt_rx_process(MDBG_SDIO_PACKER_TYPE,
			subtype, func);
	sprdwcn_bus_register_pt_tx_release(MDBG_SDIO_PACKER_TYPE,
			subtype, mdbg_tx_cb);
#endif
	return 0;
}

int mdbg_comm_init(void)
{
	int err = 0;

	ring_dev = kmalloc(sizeof(struct ring_device), GFP_KERNEL);
	if (!ring_dev)
		return -ENOMEM;

	ring_dev->ring = mdbg_ring_alloc(MDBG_RX_RING_SIZE);
	if (!(ring_dev->ring)) {
		WCN_ERR("Ring malloc error.");
		return -MDBG_ERR_MALLOC_FAIL;
	}

	wake_lock_init(&ring_dev->rw_wake_lock, WAKE_LOCK_SUSPEND,
			"mdbg_wake_lock");
	spin_lock_init(&ring_dev->rw_lock);
	mutex_init(&ring_dev->mdbg_read_mutex);
	INIT_LIST_HEAD(&ring_dev->rx_head);
	tasklet_init(&ring_dev->rx_task, mdbg_ring_rx_task,
		(unsigned long int)ring_dev);
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
	mdbg_sipc_init();
#endif
	mdbg_pt_ring_reg();
	WCN_INFO("mdbg_comm_init!");

	return err;
}

void mdbg_comm_remove(void)
{
	struct ring_rx_data *pos, *next;

	MDBG_FUNC_ENTERY;
#ifdef CONFIG_INTEGRATED_MARLIN2_GE2
	mdbg_sipc_destroy();
#endif
	wake_lock_destroy(&ring_dev->rw_wake_lock);
	mdbg_ring_destroy(ring_dev->ring);
	tasklet_kill(&ring_dev->rx_task);
	list_for_each_entry_safe(pos, next, &ring_dev->rx_head, entry) {
		if (pos) {
			list_del(&pos->entry);
			kfree(pos);
		}
	}
	kfree(ring_dev);

	ring_dev = NULL;
}
