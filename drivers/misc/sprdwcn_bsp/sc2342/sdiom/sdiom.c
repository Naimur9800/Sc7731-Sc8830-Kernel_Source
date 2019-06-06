/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * Filename : sdiom.c
 * Abstract : This file is a implementation for itm sipc command/event function
 *
 * Authors	: hai.zhang
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "sdiom.h"
#include "sdiom_depend.h"
#include "sdiom_packer_main.h"
#include "sdiom_rx_default.h"
#include "sdiom_type.h"
#include "sdiom_sleep.h"
#include <linux/version.h>
#include <linux/sdiom_rx_api.h>
#include "./../../../../mmc/host/sprd-sdhcr7.h"

#define CP_IRAM_ADDR 0X0
#define CP_AHB_ADDR 0x60300000
#define CP_AHB_EB_ADDR (CP_AHB_ADDR + 0x4)
#define CP_BT_DT_ADDR 0x51004000
#define CP_BT_CMD_BUF 0x50000000
#define CP_BT_WRITE_INDEX 0x50040610

#define SDIOM_PRINTF_LEN (10)
#define SDIOM_SLEEP_PROTECT

static int sdiom_probe(struct sdio_func *func, const struct sdio_device_id *id);
static void sdiom_remove(struct sdio_func *func);

#if defined(CONFIG_PM_SLEEP) || defined(CONFIG_PM)
static int sdiom_suspend(struct device *dev);
static int sdiom_resume(struct device *dev);
#endif
static unsigned int sdiom_gpio_num;
static unsigned int irq_num;
static unsigned int carddetect_indicator;
static unsigned int sdiom_init_indicator;
static unsigned int card_dump_flag;
static unsigned int sdiom_debug_level = LEVEL_0;

static unsigned long long sdiom_last_irq_cnt;
static unsigned long long sdiom_irq_cnt;

static struct sdio_func *sdio_func[SDIOM_MAX_FUNCS];
static struct sdio_func sdio_func_0;
static struct sdio_data *sdio_data;
static struct mmc_host *sdio_dev_host;
static unsigned int addr_val;
static unsigned int success_pac_num;

static SDIOM_RESCAN_CALLBACK s_rescan_cb;

static const struct sdio_device_id sdiom_ids[] = {
	{SDIO_DEVICE(0, 0)},
	{},
};

static const struct dev_pm_ops sdiom_pm_ops = {
	#ifdef CONFIG_PM_SLEEP
	SET_SYSTEM_SLEEP_PM_OPS(sdiom_suspend, sdiom_resume)
	#elif CONFIG_PM
	.suspend = sdiom_suspend,
	.resume = sdiom_resume,
	#endif
};

static struct sdio_driver sdiom_driver = {
	.probe = sdiom_probe,
	.remove = sdiom_remove,
	.name = "sdiom",
	.id_table = sdiom_ids,
	.drv = {
		.pm = &sdiom_pm_ops,
		},
};

static irqreturn_t sdiom_irq_handler(int irq, void *para)
{
	sdiom_print("%s enter\n", __func__);

	sdiom_lock_rx_wakelock();

	sdiom_irq_cnt++;
	disable_irq_nosync(irq);
	sdiom_rx_trans_up();

	return IRQ_HANDLED;
}

void sdiom_print_buf(unsigned char *buf, unsigned int len, const char *func)
{
	sdiom_info("%s print_buf:\n", func);
	print_hex_dump(KERN_WARNING, "sdiom hex dump:", DUMP_PREFIX_NONE, 16, 1,
		buf, (len < SDIOM_PRINTF_LEN ? len : SDIOM_PRINTF_LEN), true);
}

unsigned long long sdiom_get_status(void)
{
	unsigned long long cnt;

	cnt = sdiom_irq_cnt - sdiom_last_irq_cnt;
	sdiom_last_irq_cnt = sdiom_irq_cnt;

	return cnt;
}
EXPORT_SYMBOL_GPL(sdiom_get_status);

void sdiom_enable_rx_irq(void)
{
	if (irq_num == 0)
		return;

	sdiom_print("%s enter\n", __func__);
	irq_set_irq_type(irq_num, IRQF_TRIGGER_HIGH);
	enable_irq(irq_num);
}

int sdiom_enable_slave_int(void)
{
	int err;
	unsigned char reg_val;

#ifdef SDIOM_SLEEP_PROTECT
	sdiom_resume_check();
#endif
	sdiom_op_enter();
	sdio_claim_host(sdio_func[FUNC_0]);
	reg_val = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_DEINT_EN, &err);
	sdiom_print("default dedicated int enable: 0x%x\n", reg_val);
	sdio_writeb(sdio_func[FUNC_0],
		    reg_val | VAL_DEINT_ENABLE, SDIOM_FBR_DEINT_EN, &err);
	reg_val = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_DEINT_EN, &err);

	sdiom_print("write after dedicated int enable: 0x%x\n", reg_val);
	sdio_release_host(sdio_func[FUNC_0]);
	sdiom_op_leave();

	return 0;
}

static int sdiom_interrupt_init(void)
{
	int ret;

	sdiom_print("%s enter, wakelock init\n", __func__);
	sdiom_print("sdiom_gpio_num: %d\n", sdiom_gpio_num);

	ret = gpio_request(sdiom_gpio_num, "marlin2_irq");
	if (ret < 0) {
		sdiom_err("req gpio irq = %d fail!!!", sdiom_gpio_num);
		return ret;
	}

	ret = gpio_direction_input(sdiom_gpio_num);
	if (ret < 0) {
		sdiom_err("gpio:%d input set fail!!!", sdiom_gpio_num);
		return ret;
	}

	irq_num = gpio_to_irq(sdiom_gpio_num);

	ret =
	    request_irq(irq_num, sdiom_irq_handler,
			IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND, "sdiom_irq", NULL);
	if (ret != 0) {
		sdiom_err("request irq err!!!gpio is %d!!!", irq_num);
		return ret;
	}

	return 0;
}

static void sdiom_abort(void)
{
	int err;

	sdiom_print("sdiom_abort\n");

#ifdef SDIOM_SLEEP_PROTECT
	sdiom_resume_check();
#endif

	sdiom_op_enter();

	if (unlikely(carddetect_indicator != true)) {
		sdiom_op_leave();
		return;
	}

	sdio_claim_host(sdio_func[FUNC_0]);
	sdio_writeb(sdio_func[FUNC_0], VAL_ABORT_TRANS, SDIOM_CCCR_ABORT, &err);
	sdio_release_host(sdio_func[FUNC_0]);
	sdiom_op_leave();

}

static void sdiom_tx_abort(void)
{
	int err;

	sdiom_info("sdiom_abort\n");
	sdio_claim_host(sdio_func[FUNC_0]);
	sdio_writeb(sdio_func[FUNC_0], VAL_ABORT_TRANS,
		    SDIOM_CCCR_ABORT, &err);
	sdio_release_host(sdio_func[FUNC_0]);
}

/* Get Success Transfer pac num Before Abort */
static void sdiom_success_trans_pac_num(void)
{
	unsigned char reg_stbba_0;
	unsigned char reg_stbba_1;
	unsigned char reg_stbba_2;
	unsigned char reg_stbba_3;
	int err;

	sdio_claim_host(sdio_func[FUNC_0]);
	reg_stbba_0 = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_STBBA_0, &err);
	reg_stbba_1 = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_STBBA_1, &err);
	reg_stbba_2 = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_STBBA_2, &err);
	reg_stbba_3 = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_STBBA_3, &err);
	success_pac_num = reg_stbba_0 | (reg_stbba_1 << 8) |
	    (reg_stbba_2 << 16) | (reg_stbba_3 << 24);
	sdio_release_host(sdio_func[FUNC_0]);

	sdiom_info("stbba:[%d]\n", success_pac_num);
}

unsigned int sdiom_get_trans_pac_num(void)
{
	return success_pac_num;
}

int sdiom_sdio_pt_write(void *src, unsigned datalen)
{
	int ret = 0;

	if (unlikely(card_dump_flag == true))
		return ERROR;

#ifdef SDIOM_SLEEP_PROTECT
	sdiom_resume_check();
#endif

	sdiom_op_enter();

	if (unlikely(carddetect_indicator != true)) {
		sdiom_op_leave();
		return ERROR;
	}

	sdio_claim_host(sdio_func[FUNC_1]);
	sdiom_print("sdiom_sdio_pt_write buf[%p] len[%d]\n", src, datalen);
	ret = sdio_writesb(sdio_func[FUNC_1], SDIOM_PK_MODE_ADDR, src, datalen);
	if (ret)
		sdiom_err("sdiom_sdio_pt_write failed err:%d!\n", ret);
	sdio_release_host(sdio_func[FUNC_1]);
	if (ret) {
		sdiom_success_trans_pac_num();
		sdiom_tx_abort();
		sdiom_info("%s sdio_writesb err[%d]!\n", __func__, ret);
	}
	sdiom_op_leave();

	return ret;
}
EXPORT_SYMBOL_GPL(sdiom_sdio_pt_write);

int sdiom_sdio_pt_write_raw(unsigned char *src, unsigned int datalen, int retry)
{
	int ret = 0;

	if (unlikely(card_dump_flag == true))
		return ERROR;

#ifdef SDIOM_SLEEP_PROTECT
	sdiom_resume_check();
#endif
	sdiom_op_enter();

	if (unlikely(carddetect_indicator != true)) {
		sdiom_op_leave();
		return ERROR;
	}

	sdio_claim_host(sdio_func[FUNC_1]);
	ret = sdio_writesb(sdio_func[FUNC_1], SDIOM_PK_MODE_ADDR, src,
			SDIOM_ALIGN_512BYTE(datalen));
	sdio_release_host(sdio_func[FUNC_1]);
	if (ret) {
		sdiom_success_trans_pac_num();
		sdiom_tx_abort();
		sdiom_info("%s sdio_writesb err[%d]!\n", __func__, ret);
	}
	sdiom_op_leave();

	return ret;
}
EXPORT_SYMBOL_GPL(sdiom_sdio_pt_write_raw);

int sdiom_sdio_pt_read(void *src, unsigned datalen)
{
	int ret;


	if (unlikely(card_dump_flag == true))
		return ERROR;

#ifdef SDIOM_SLEEP_PROTECT
	sdiom_resume_check();
#endif

	sdiom_op_enter();

	if (unlikely(carddetect_indicator != true)) {
		sdiom_op_leave();
		return ERROR;
	}

	sdio_claim_host(sdio_func[FUNC_1]);
	ret = sdio_readsb(sdio_func[FUNC_1], src, SDIOM_PK_MODE_ADDR, datalen);
	sdio_release_host(sdio_func[FUNC_1]);
	sdiom_op_leave();

	if (ret) {
		sdiom_abort();
		sdiom_info("%s sdio_readsb err[%d]!\n", __func__, ret);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(sdiom_sdio_pt_read);

void sdiom_sdio_tx_status(void)
{
	unsigned char reg_stbba_0;
	unsigned char reg_stbba_1;
	unsigned char reg_stbba_2;
	unsigned char reg_stbba_3;

	unsigned char reg_pubint_raw4;

	unsigned char reg_apbrw0_0;
	unsigned char reg_apbrw0_1;
	unsigned char reg_apbrw0_2;
	unsigned char reg_apbrw0_3;

	int err;

#ifdef SDIOM_SLEEP_PROTECT
	sdiom_resume_check();
#endif

	sdiom_op_enter();
	sdio_claim_host(sdio_func[FUNC_0]);

	reg_stbba_0 = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_STBBA_0, &err);
	reg_stbba_1 = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_STBBA_1, &err);
	reg_stbba_2 = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_STBBA_2, &err);
	reg_stbba_3 = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_STBBA_3, &err);

	reg_pubint_raw4 = sdio_readb(sdio_func[FUNC_0],
				     SDIOM_FBR_PUBINT_RAW4_0, &err);

	reg_apbrw0_0 = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_APBRW0_0, &err);
	reg_apbrw0_1 = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_APBRW0_1, &err);
	reg_apbrw0_2 = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_APBRW0_2, &err);
	reg_apbrw0_3 = sdio_readb(sdio_func[FUNC_0], SDIOM_FBR_APBRW0_3, &err);

	sdiom_print
	    ("byte:[0x%x][0x%x][0x%x][0x%x];[0x%x];[0x%x][0x%x][0x%x][0x%x]\n",
	     reg_stbba_0, reg_stbba_1, reg_stbba_2, reg_stbba_3,
	     reg_pubint_raw4,
	     reg_apbrw0_0, reg_apbrw0_1, reg_apbrw0_2, reg_apbrw0_3);

	sdio_release_host(sdio_func[FUNC_0]);
	sdiom_op_leave();

}
EXPORT_SYMBOL_GPL(sdiom_sdio_tx_status);

int sdiom_aon_readb(unsigned int addr, unsigned char *val)
{
	int err = 0;
	unsigned char reg_val = 0;

#ifdef SDIOM_SLEEP_PROTECT
	sdiom_resume_check();
#endif

	sdiom_op_enter();

	if (unlikely(carddetect_indicator != true)) {
		sdiom_op_leave();
		return ERROR;
	}

	sdio_claim_host(sdio_func[FUNC_0]);

	reg_val = sdio_readb(sdio_func[FUNC_0], addr, &err);

	if (val)
		*val = reg_val;

	sdio_release_host(sdio_func[FUNC_0]);
	sdiom_op_leave();

	return err;
}
EXPORT_SYMBOL_GPL(sdiom_aon_readb);

int sdiom_aon_writeb(unsigned int addr, unsigned char val)
{
	int err = 0;

#ifdef SDIOM_SLEEP_PROTECT
	sdiom_resume_check();
#endif
	sdiom_op_enter();

	if (unlikely(carddetect_indicator != true)) {
		sdiom_op_leave();
		return ERROR;
	}

	sdio_claim_host(sdio_func[FUNC_0]);

	sdio_writeb(sdio_func[FUNC_0], val, addr, &err);

	sdio_release_host(sdio_func[FUNC_0]);
	sdiom_op_leave();

	return err;
}
EXPORT_SYMBOL_GPL(sdiom_aon_writeb);

void sdiom_dump_aon_reg(void)
{
	unsigned char reg_buf[16];
	unsigned char i, val = 0;
	int master = -1;

	sdiom_info("sdiom_dump_aon_reg entry\n");
	for (i = 0; i <= DUMP_MIN_REG; i++) {
		sdiom_aon_readb(DUMP_REG + i, &reg_buf[i]);
		sdiom_info("pmu sdio status:[0x%x]:0x%x\n",
			DUMP_REG + i, reg_buf[i]);
	}

	/*ap hready status*/
	sdiom_aon_readb(DUMP_REG_1a4, &val);

	val &= ~BIT(4);
	sdiom_aon_writeb(DUMP_REG_1a4, val);

	val &= ~(BIT(0) | BIT(1) | BIT(2) | BIT(3));
	sdiom_aon_writeb(DUMP_REG_1a4, val);

	val |= BIT(4);
	sdiom_aon_writeb(DUMP_REG_1a4, val);

	for (i = 0; i < DUMP_REG_4; i++) {
		sdiom_aon_readb(DUMP_REG_144 + i, &reg_buf[i]);
		sdiom_info("hready status:[0x%x]:0x%x\n",
			DUMP_REG_144 + i, reg_buf[i]);
	}

	/*read master address*/
	for (i = 0; i <= 7; i++)
		if (!(reg_buf[2] & (0x1 << i))) {
			master = 7 - i;
			break;
		}

	sdiom_info("master=%d\n", master);
	if (master < 0)
		return;

	sdiom_aon_readb(DUMP_REG_1a4, &val);

	val &= ~BIT(4);
	sdiom_aon_writeb(DUMP_REG_1a4, val);

	val |= BIT(3) | master;
	sdiom_aon_writeb(DUMP_REG_1a4, val);

	val |= BIT(4);
	sdiom_aon_writeb(DUMP_REG_1a4, val);

	for (i = 0; i < DUMP_REG_4; i++) {
		sdiom_aon_readb(DUMP_REG_1a4 + i, &reg_buf[i]);
		sdiom_info("master=%d address[0x%x]=0x%x\n",
			master, DUMP_REG_144 + i, reg_buf[i]);
	}
	sdiom_info("sdiom_dump_aon_reg end\n\n");
}
EXPORT_SYMBOL_GPL(sdiom_dump_aon_reg);

static int sdiom_dt_set_addr(unsigned int addr)
{
	unsigned char address[4];
	int err = 0;
	int i;

	for (i = 0; i < 4; i++) {
		address[i] = (addr >> (8 * i)) & 0xFF;
		sdiom_print("addr[%d]:0x%x, ", i, address[i]);
	}
	sdiom_print("\n");

	sdio_claim_host(sdio_func[FUNC_0]);

	sdio_writeb(sdio_func[FUNC_0], address[0], SDIOM_FBR_SYSADDR_0, &err);

	if (err != 0)
		goto sdiom_dt_set_addr_exit;

	sdio_writeb(sdio_func[FUNC_0], address[1], SDIOM_FBR_SYSADDR_1, &err);
	if (err != 0)
		goto sdiom_dt_set_addr_exit;

	sdio_writeb(sdio_func[FUNC_0], address[2], SDIOM_FBR_SYSADDR_2, &err);
	if (err != 0)
		goto sdiom_dt_set_addr_exit;

	sdio_writeb(sdio_func[FUNC_0], address[3], SDIOM_FBR_SYSADDR_3, &err);
	if (err != 0)
		goto sdiom_dt_set_addr_exit;

sdiom_dt_set_addr_exit:
	sdio_release_host(sdio_func[FUNC_0]);

	return err;
}

int sdiom_dt_write(unsigned int system_addr,
			    void *buf, unsigned int len)
{
	int ret = 0;

	if (!buf) {
		WARN_ON(1);
		return ERROR;
	}

#ifdef SDIOM_SLEEP_PROTECT
	sdiom_resume_check();
#endif

	sdiom_op_enter();

	if (unlikely(carddetect_indicator != true)) {
		sdiom_op_leave();
		return ERROR;
	}

	ret = sdiom_dt_set_addr(system_addr);
	if (ret != 0) {
		sdiom_op_leave();
		return ret;
	}

	sdio_claim_host(sdio_func[FUNC_1]);

	ret = sdio_memcpy_toio_no_incraddr(sdio_func[FUNC_1],
					   SDIOM_DT_MODE_ADDR, buf, len);

	sdio_release_host(sdio_func[FUNC_1]);
	sdiom_op_leave();
	if (ret != 0) {
		sdiom_err("dt write fail ret:%d\n", ret);
		sdiom_dump_aon_reg();
		sdiom_abort();
	}

	return ret;
}
EXPORT_SYMBOL_GPL(sdiom_dt_write);

static void sdiom_dummy_read(bool fail)
{
	int ret = 0;

	ret = sdiom_dt_read(CP_IRAM_ADDR, &addr_val, 4);
	if (ret < 0)
		sdiom_err("%s read iram error:%d\n", __func__, ret);
	if (fail)
		sdiom_err("read iram = 0x%x\n", addr_val);

	ret = sdiom_dt_read(CP_BT_CMD_BUF, &addr_val, 4);
	if (ret != 0)
		sdiom_err("read 0x50000000 fail, ret=%d\n", ret);
	if (fail)
		sdiom_err("read 0x50000000 = 0x%x\n", addr_val);

	ret = sdiom_dt_read(CP_BT_WRITE_INDEX, &addr_val, 4);
	if (ret != 0)
		sdiom_err("read 0x50040610 fail, ret=%d\n", ret);
	if (fail)
		sdiom_err("read 0x50040610 = 0x%x\n", addr_val);
}

static void sdiom_dt_fail_handle(void *buf, unsigned int len)
{
	int ret = 0, i = 0;

	sdiom_info("start write to iram\n");
	addr_val = 0x55555555;
	ret = sdiom_dt_write(0x0, &addr_val, 4);
	if (ret != 0)
		sdiom_err("start write to iram fail\n");

	sdiom_info("start write bt again\n");
	ret = sdiom_dt_write(CP_BT_DT_ADDR, (void *)buf, len);
	if (ret != 0)
		sdiom_err("start write bt again fail\n");

	/* rst */
	ret = sdiom_dt_read(CP_AHB_ADDR, &addr_val, 4);
	if (ret < 0)
		sdiom_err("%s read CP_BT_DT_ADDR error:%d\n", __func__, ret);
	sdiom_info("%s 0x60300000 addr val:0x%x\n", __func__, addr_val);

	addr_val |= BIT(4);
	ret = sdiom_dt_write(CP_AHB_ADDR, &addr_val, 4);
	if (ret < 0)
		sdiom_err("%s set CP_BT_DT_ADDR error:%d\n", __func__, ret);

	addr_val &= ~(BIT(4));
	ret = sdiom_dt_write(CP_AHB_ADDR, &addr_val, 4);
	if (ret < 0)
		sdiom_err("%s clear CP_BT_DT_ADDR 2 error:%d\n", __func__, ret);

	/* write bt again */
	sdiom_info("after rst write bt again\n");
	ret = sdiom_dt_write(CP_BT_DT_ADDR, (void *)buf, len);
	if (ret != 0)
		sdiom_err("after rst write bt again fail\n");

	/* sdio axi force enable */
	ret = sdiom_dt_read(CP_AHB_EB_ADDR, &addr_val, 4);
	if (ret < 0)
		sdiom_err("%s read CP_AHB_EB_ADDR error:%d\n", __func__, ret);
	sdiom_info("%s CP_AHB_EB_ADDR val:0x%x\n", __func__, addr_val);

	addr_val |= BIT(6);
	ret = sdiom_dt_write(CP_AHB_EB_ADDR, &addr_val, 4);
	if (ret < 0)
		sdiom_err("%s set CP_AHB_EB_ADDR error:%d\n", __func__, ret);

	mdbg_assert_interface("sdiom dt mode tx fail\n");

	for (i = 0; i < (len > 10 ? 10 : len); i++)
		sdiom_info("sdiom_dt_write_bt [%d] 0x%x\n",
			i, *((unsigned char *)buf + i));
}

int sdiom_dt_write_bt(unsigned int system_addr,
			       void *buf, unsigned int len)
{
	int ret = 0;

	if (!buf) {
		WARN_ON(1);
		return ERROR;
	}

	if (unlikely(card_dump_flag == true))
		return ERROR;

#ifdef SDIOM_SLEEP_PROTECT
	sdiom_resume_check();
#endif
	sdiom_dummy_read(false);

	sdiom_op_enter();

	if (unlikely(carddetect_indicator != true)) {
		sdiom_op_leave();
		return ERROR;
	}

	ret = sdiom_dt_set_addr(system_addr);
	if (ret != 0) {
		sdiom_op_leave();
		return ret;
	}

	sdio_claim_host(sdio_func[FUNC_1]);

	ret = sdio_memcpy_toio_no_incraddr(sdio_func[FUNC_1],
					   SDIOM_DT_MODE_ADDR, buf, len);

	sdio_release_host(sdio_func[FUNC_1]);
	sdiom_op_leave();
	if (ret != 0) {
		sdiom_err("dt write bt fail ret:%d\n", ret);
		sdiom_dump_aon_reg();
		sdiom_abort();
		sdiom_err("start read dummy data\n");
		sdiom_dummy_read(true);
		sdiom_dt_fail_handle(buf, len);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(sdiom_dt_write_bt);

int sdiom_dt_read(unsigned int system_addr, void *buf,
			   unsigned int len)
{
	int ret = 0;

	if (!buf) {
		WARN_ON(1);
		return ERROR;
	}

#ifdef SDIOM_SLEEP_PROTECT
	sdiom_resume_check();
#endif
	sdiom_op_enter();

	if (unlikely(carddetect_indicator != true)) {
		sdiom_op_leave();
		return ERROR;
	}

	ret = sdiom_dt_set_addr(system_addr);
	if (ret != 0) {
		sdiom_op_leave();
		return ret;
	}

	sdio_claim_host(sdio_func[FUNC_1]);

	ret =
	    sdio_memcpy_fromio_no_incraddr(sdio_func[FUNC_1],
					   (unsigned char *)buf, 0x0f, len);

	sdio_release_host(sdio_func[FUNC_1]);
	sdiom_op_leave();
	if (ret != 0) {
		sdiom_err("dt read fail ret:%d\n", ret);
		sdiom_dump_aon_reg();
		sdiom_abort();
	}

	return ret;
}
EXPORT_SYMBOL_GPL(sdiom_dt_read);

static int get_sdio_dev_func(struct sdio_func *func)
{
	if (func->num >= SDIOM_MAX_FUNCS) {
		sdiom_err("func num err!!! func num is %d!!!", func->num);
		return -1;
	}
	sdiom_print("func num is %d!!!", func->num);

	if (func->num == 1) {
		sdio_func_0.num = 0;
		sdio_func_0.card = func->card;
		sdio_func_0.max_blksize = SDIOM_BLK_SIZE;
		sdio_func[0] = &sdio_func_0;
	}

	sdio_func[func->num] = func;

	return 0;
}

static int sdiom_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
	int ret;

	sdiom_info("%s Enter\n", __func__);
	sdiom_info("func_class: func->class=%x\n", func->class);
	sdiom_info("vendor: 0x%04x\n", func->vendor);
	sdiom_info("device: 0x%04x\n", func->device);
	sdiom_info("func_num: 0x%04x\n", func->num);

	ret = get_sdio_dev_func(func);
	if (ret < 0) {
		sdiom_err("get func err!!!\n");
		return ret;
	}
	sdiom_info("get func ok!!!\n");

	/* Enable Function 1 */
	sdio_claim_host(sdio_func[1]);
	ret = sdio_enable_func(sdio_func[1]);
	sdio_set_block_size(sdio_func[1], SDIOM_BLK_SIZE);
	sdio_func[1]->max_blksize = SDIOM_BLK_SIZE;
	sdio_release_host(sdio_func[1]);
	if (ret < 0) {
		sdiom_err("enable func1 err!!! ret is %d", ret);
		return ret;
	}
	sdiom_info("enable func1 ok!!!");

	sdiom_enable_slave_int();

	sdiom_enable_rx_irq();

	carddetect_indicator = true;

	card_dump_flag = false;

	/* calling rescan callback to inform download */
	if (s_rescan_cb != NULL)
		s_rescan_cb();
	else
		sdiom_err("rescan callback not exist\n");

	return 0;
}

static void sdiom_remove(struct sdio_func *func)
{
	sdiom_info("[%s]enter\n", __func__);

	carddetect_indicator = false;

	if (irq_num != 0)
		disable_irq(irq_num);
}
#if defined(CONFIG_PM_SLEEP) || defined(CONFIG_PM)
static int sdiom_suspend(struct device *dev)
{
	struct sdio_func *func;

	sdiom_info("[%s]enter\n", __func__);

	func = container_of(dev, struct sdio_func, dev);
	func->card->host->pm_flags |= MMC_PM_KEEP_POWER;

	atomic_set(&sdiom_resume_flag, 0);

	return 0;
}

static int sdiom_resume(struct device *dev)
{
	sdiom_info("[%s]enter\n", __func__);

	atomic_set(&sdiom_resume_flag, 1);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static struct mmc_host *sdio_dev_get_host(struct device_node *np_node)
{
	struct sprd_sdhc_host *host;
	struct platform_device *pdev;

	pdev = of_find_device_by_node(np_node);
	if (pdev == NULL) {
		sdiom_err("dev get platform device failed!!!");
		return NULL;
	}

	host = platform_get_drvdata(pdev);

	return container_of((void *)host, struct mmc_host, private);
}

void sdiom_exit(void)
{
	sdiom_info("sdiom_exit ok");
	sdiom_packer_deinit();
	sdiom_debug_deinit();
}
EXPORT_SYMBOL_GPL(sdiom_exit);

void sdiom_remove_card(void)
{
	mmc_card_set_removed(sdio_dev_host->card);
	mmc_detect_change(sdio_dev_host, 0);
}
EXPORT_SYMBOL_GPL(sdiom_remove_card);

int sdiom_driver_register(void)
{
	int ret = 0;

	sdiom_info("[%s]enter\n", __func__);

	ret = sdio_register_driver(&sdiom_driver);
	if (ret != 0) {
		sdiom_err("register drv err!!!ret is %d!!!", ret);
		return -1;
	}
	sdiom_info("register drv succ!!!");

	return ret;
}
EXPORT_SYMBOL_GPL(sdiom_driver_register);

void sdiom_driver_unregister(void)
{
	sdiom_info("[%s]enter\n", __func__);
	sdiom_op_enter();

	carddetect_indicator = false;
	sdio_unregister_driver(&sdiom_driver);

	sdiom_op_leave();
}
EXPORT_SYMBOL_GPL(sdiom_driver_unregister);

int sdiom_init(void)
{
	struct device_node *np;
	struct device_node *sdio_node;

	if (sdiom_init_indicator == true) {
		sdiom_info("already initialized!\n");
		return 0;
	}
	carddetect_indicator = false;

	sdio_data = kzalloc(sizeof(*sdio_data), GFP_KERNEL);
	if (!sdio_data)
		return -ENOMEM;
	np = of_find_node_by_name(NULL, "sprd-marlin2");
	if (!np) {
		sdiom_err("sprd-marlin2 not found");
		return -1;
	}

	sdiom_gpio_num = of_get_named_gpio(np, "m2-wakeup-ap-gpios", 0);

	sdio_node = of_parse_phandle(np, "sdhci-name", 0);
	sdio_data->sdhci = sdio_node->name;
	if (sdio_node == NULL) {
		sdiom_err("get sdhci-name failed");
		return -1;
	}

	sdio_dev_host = sdio_dev_get_host(sdio_node);
	if (sdio_dev_host == NULL) {
		sdiom_err("get host failed!!!");
		return -1;
	}

	sdiom_info("get host ok!!!");

	sdiom_tx_mutex_init();
	sdiom_rx_cb_mutex_init();
	sdiom_rx_default();

	sdiom_packer_init();

	sdiom_sleep_flag_init();

	sdiom_wakelock_init();

	sdiom_sleep_mutex_init();

	sdiom_interrupt_init();

	if (irq_num != 0)
		disable_irq(irq_num);

	sdiom_init_indicator = true;

	sdiom_debug_init();

	return 0;
}
EXPORT_SYMBOL_GPL(sdiom_init);

int sdiom_sdio_rescan(void)
{
	sdiom_info("sdiom_sdio_rescan\n");

	if (sdio_dev_host == NULL) {
		sdiom_err("sdiom_sdio_rescan get host failed!");
		return -1;
	}
	mmc_detect_change(sdio_dev_host, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(sdiom_sdio_rescan);

void sdiom_register_rescan_cb(void *func)
{
	s_rescan_cb = (SDIOM_RESCAN_CALLBACK) func;

}
EXPORT_SYMBOL_GPL(sdiom_register_rescan_cb);

void sdiom_set_carddump_status(unsigned int flag)
{
	sdiom_info("carddump flag set[%d]\n", flag);
	if (flag == true) {
		disable_irq(irq_num);
		sdiom_info("disable rx int for dump\n");
	}
	card_dump_flag = flag;
}
EXPORT_SYMBOL_GPL(sdiom_set_carddump_status);

unsigned int sdiom_get_carddump_status(void)
{
	return card_dump_flag;
}
EXPORT_SYMBOL_GPL(sdiom_get_carddump_status);

void sdiom_set_debug_level(unsigned int level)
{
	if (level < LEVEL_MAX) {
		sdiom_debug_level = level;
		sdiom_info("debug level set[%d]\n", level);
	} else
		sdiom_info("debug level set error! [%d]\n", level);
}

unsigned int sdiom_get_debug_level(void)
{
	return sdiom_debug_level;
}

