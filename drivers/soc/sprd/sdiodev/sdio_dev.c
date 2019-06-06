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

#include <soc/sprd/sdio_dev.h>

#define SDIO_CHN_NUM			8
#define SDIO_CHN_START			8
#define SDIO_CHN_REG0			0x840
#define SDIO_CHN_REG1			0x841
#define SDIO_CHN_DATALEN_BASEREG0	0x800
#define SDIO_CHN_DATALEN_BASEREG1	0x801
#define SDIO_CHN_DATALEN_BASEREG2	0x802

static struct mutex g_sdio_func_lock;
static int ack_gpio_status;
struct sdio_func *sprd_sdio_func[SDIODEV_MAX_FUNCS];
struct sdio_func sdio_func_0;
struct sdio_chn_handler sdio_tran_handle[17];
static struct sdio_data *sdio_data;
static int sdio_irq_num;
static int marlinwake_irq_num;

static struct wake_lock marlin_wakelock;
static struct wake_lock bt_ap_wakelock;
static struct wake_lock marlinpub_wakelock;
static struct work_struct marlin_wq;
static struct work_struct marlinwake_wq;
static bool ap_sdio_ready;
static bool have_card;
static struct marlin_sdio_ready_t marlin_sdio_ready = { 0, 0 };
static const struct sdio_device_id marlin_sdio_ids[] = {
	{SDIO_DEVICE(MARLIN_VENDOR_ID, MARLIN_DEVICE_ID)},
	{},
};

static struct mmc_host *sdio_dev_host;
char *sync_data_ptr;
static int gpio_marlin_req_tag;
static int gpio_marlinwake_tag;
static bool sdio_w_flag;
atomic_t gpioreq_need_pulldown;
static bool bt_wake_flag;
static bool marlin_bt_wake_flag;
static struct completion marlin_ack;
static struct sleep_policy_t sleep_para;
unsigned int fwdownload_fin = 0;
unsigned int irq_count_change = 0;
unsigned int irq_count_change_last = 0;
spinlock_t sleep_spinlock;
struct device *marlin_dev = NULL;
MARLIN_PM_RESUME_WAIT_INIT(marlin_sdio_wait);

static void marlin_runtime_setting(struct platform_device *pdev)
{
	SDIOTRAN_DEBUG("marlin run PM init\n");
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
}

static int marlin_runtime_get(struct device *dev)
{
	return pm_runtime_get_sync(dev);
}

static int marlin_runtime_put(struct device *dev)
{
	pm_runtime_mark_last_busy(dev);
	return pm_runtime_put_autosuspend(dev);
}
#ifdef CONFIG_PM_RUNTIME
static int marlin_runtime_suspend(struct device *dev)
{
	return 0;
}

static int marlin_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static int get_sdio_dev_func(struct sdio_func *func)
{
	if (SDIODEV_MAX_FUNCS <= func->num) {
		SDIOTRAN_ERR("func num err!!! func num is %d!!!", func->num);
		return -1;
	}
	SDIOTRAN_DEBUG("func num is %d!!!", func->num);
	if (func->num == 1) {
		sdio_func_0.num = 0;
		sdio_func_0.card = func->card;
		sprd_sdio_func[0] = &sdio_func_0;
	}
	sprd_sdio_func[func->num] = func;

	return 0;
}

static int free_sdio_dev_func(struct sdio_func *func)
{
	if (SDIODEV_MAX_FUNCS <= func->num) {
		SDIOTRAN_ERR("func num err!!! func num is %d!!!", func->num);
		return -1;
	}

	sprd_sdio_func[0] = NULL;
	sprd_sdio_func[func->num] = NULL;
	sdio_func_0.num = 0;
	sdio_func_0.card = NULL;

	return 0;
}

static int wakeup_slave_pin_init(void)
{
	int ret;

	ret = gpio_request(sdio_data->wake_out, "marlin_wakeup");
	if (ret < 0) {
		SDIOTRAN_ERR("req gpio sdio_data->wake_out = %d fail!!!",
			     sdio_data->wake_out);
		return ret;
	}

	ret = gpio_direction_output(sdio_data->wake_out, 0);
	if (ret) {
		SDIOTRAN_ERR("sdio_data->wake_out = %d input set fail!!!",
			     sdio_data->wake_out);
		return ret;
	}

	SDIOTRAN_ERR("req sdio_data->wake_out = %d succ!!!",
		     sdio_data->wake_out);

	return ret;
}

static void gpio_timer_handler(unsigned long data)
{
	if (gpio_get_value(sdio_data->wake_ack)) {
		sleep_para.gpio_opt_tag = 1;
		mod_timer(&(sleep_para.gpio_timer),
			jiffies + msecs_to_jiffies(sleep_para.ack_high_time));
		SDIOTRAN_ERR("ack high");
	} else {
		sleep_para.gpio_opt_tag = 0;
		SDIOTRAN_ERR("gpio_opt_tag=0\n");
	}
}

int sprd_get_download_fin(void)
{
	return fwdownload_fin;
}
EXPORT_SYMBOL_GPL(sprd_get_download_fin);

void sprd_set_download_fin(int dl_tag)
{
	marlin_sdio_ready.marlin_sdio_init_end_tag = 1;
	fwdownload_fin = dl_tag;
}
EXPORT_SYMBOL_GPL(sprd_set_download_fin);

bool sprd_get_marlin_status(void)
{
	pr_info("irq_count_change:%d\n\tirq_count_change_last:%d\n",
		irq_count_change, irq_count_change_last);
	if ((irq_count_change == 0) && (irq_count_change_last == 0))
		return 1;
	else if (irq_count_change != irq_count_change_last) {
		irq_count_change_last = irq_count_change;
		return 1;
	} else
		return 0;
}
EXPORT_SYMBOL_GPL(sprd_get_marlin_status);

/* 0: wlan close,  1: wlan open */
atomic_t is_wlan_open;

/* 0: not complete,  1:  complete */
atomic_t set_marlin_cmplete;
int sprd_set_wlan_status(int status)
{
	atomic_set(&(is_wlan_open), status);

	return 0;
}
EXPORT_SYMBOL_GPL(sprd_set_wlan_status);

int sprd_set_marlin_wakeup(unsigned int chn, unsigned int user_id)
{
	/* user_id: wifi=0x1;bt=0x2 */

	unsigned long flags;
	int ret;

	SDIOTRAN_DEBUG("entry");

	if (sprd_get_download_fin() != 1) {
		SDIOTRAN_ERR("marlin unready");
		return -1;
	}

	/* when bt only, marlin will don't stay 2s wakeup;
	* so add for avoid bt ack 250ms high, then in
	* below cases ap don't req cp and send data directly;
	* case1: first open wlan, then user_id = 1;
	* case2: send AT cmd or loop check, then user_id = 3;
	* mdbg module has done re-try action
	*/
	if ((1 == atomic_read(&(is_wlan_open))) && (1 == user_id)) {
		if ((gpio_get_value(sdio_data->wake_ack))
		    && (0 == sleep_para.gpio_opt_tag))
			mdelay(sleep_para.ack_high_time);
		if (gpio_get_value(sdio_data->wake_ack))
			SDIOTRAN_ERR("err:bt req delay 300ms\n");
		atomic_set(&is_wlan_open, 0);
	}

	spin_lock_irqsave(&sleep_spinlock, flags);

	if (!sleep_para.gpio_opt_tag) {
		if (gpio_get_value(sdio_data->wake_ack)) {
			spin_unlock_irqrestore(&sleep_spinlock, flags);
			SDIOTRAN_ERR("%d-high,user_id-%d\n",
				     sdio_data->wake_ack, user_id);
			return -2;
		}
		ack_gpio_status = gpio_get_value(sdio_data->wake_ack);
	/* bt ack low,thenlose the marlinwake irq */
	if (ack_gpio_status) {
			spin_unlock_irqrestore(&sleep_spinlock, flags);
			SDIOTRAN_ERR("ack_gpio_status-%d\n", ack_gpio_status);
			return -2;
		}
	/* last wakeupdon't complete */
	if (!atomic_read(&(set_marlin_cmplete))) {
			spin_unlock_irqrestore(&sleep_spinlock, flags);
			SDIOTRAN_ERR("set_marlin_complete-0\n");
			return -3;
		}

		atomic_set(&(set_marlin_cmplete), 0);
		/* invoid re-entry this func */
		if ((0x1 == user_id) || (0x3 == user_id))
			sdio_w_flag = 1;

		if (0x2 == user_id)
			bt_wake_flag = 1;
		atomic_set(&gpioreq_need_pulldown, 1);

		gpio_direction_output(sdio_data->wake_out, 1);
		spin_unlock_irqrestore(&sleep_spinlock, flags);

		SDIOTRAN_ERR("%d-1\n", sdio_data->wake_out);

		if ((0x1 == user_id) || (0x3 == user_id)) {
			ret = wait_for_completion_timeout(&marlin_ack,
							msecs_to_jiffies(100));
			if (!ret) {
				atomic_set(&(set_marlin_cmplete), 1);
				SDIOTRAN_ERR
				    ("marlin chn %d ack timeout, user_id-%d!!!",
				     chn, user_id);
				/* timeout */
				return -ETIMEDOUT;
			}
		}
	} else
		spin_unlock_irqrestore(&sleep_spinlock, flags);

	atomic_set(&(set_marlin_cmplete), 1);

	return 0;
}
EXPORT_SYMBOL_GPL(sprd_set_marlin_wakeup);

int sprd_set_marlin_sleep(unsigned int chn, unsigned int user_id)
{
	/* user_id: wifi=0x1;bt=0x2 */
	SDIOTRAN_DEBUG("entry");

	if (atomic_read(&gpioreq_need_pulldown)) {
		gpio_direction_output(sdio_data->wake_out, 0);
		if ((user_id == 0x1) || (user_id == 0x3))
			sdio_w_flag = 0;
		if (user_id == 0x2)
			bt_wake_flag = 0;
		atomic_set(&gpioreq_need_pulldown, 0);

		SDIOTRAN_ERR("gpio%d-0,user_id-%d\n", sdio_data->wake_out,
			     user_id);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(sprd_set_marlin_sleep);

/* only idle chn can be used for ap write */
int sprd_sdio_dev_chn_idle(unsigned int chn)
{
	unsigned int active_status;
	unsigned char status0, status1;
	int err_ret;

	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);
	if (NULL == sprd_sdio_func[SDIODEV_FUNC_0]) {
		SDIOTRAN_ERR("func uninit!!!");
		return -1;
	}

	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_0]);

	status0 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],
					SDIO_CHN_REG0,
					&err_ret);

	status1 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],
					SDIO_CHN_REG1,
					&err_ret);

	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_0]);

	active_status = (unsigned int) (status0) +
			((unsigned int) (status1) << 8);

	if (BIT_0 & (active_status >> chn)) {
		SDIOTRAN_DEBUG("SDIO channel num = %x is active!!", chn);
		/* active */
		return 0;

	} else {
		SDIOTRAN_DEBUG("SDIO channel num = %x is idle!!", chn);
		/* idle */
		return 1;
	}
}
EXPORT_SYMBOL_GPL(sprd_sdio_dev_chn_idle);

int sprd_sdio_dev_get_read_chn(void)
{
	unsigned char chn_status;
	int err_ret;
	int i = 0;
	int timeout = 3;

	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);
	marlin_runtime_get(marlin_dev);

	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_0]);

	chn_status = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],
						SDIO_CHN_REG1,
						&err_ret);

	while (!chn_status) {
		chn_status = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],
						SDIO_CHN_REG1,
						&err_ret);
		timeout--;
		if (timeout == 0) {
			sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_0]);
			marlin_runtime_put(marlin_dev);
			SDIOTRAN_ERR("get sdio channel timeout!!!");

			return -1;
		}
	}

	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_0]);
	marlin_runtime_put(marlin_dev);
	SDIOTRAN_DEBUG("ap read: chn_status is 0x%x!!!", chn_status);

	if (chn_status == 0xff) {
		SDIOTRAN_ERR("Sdio Err!!!chn status 0x%x!!!", chn_status);
		return -1;
	}

	for (i = 0; i < SDIO_CHN_NUM; i++) {
		if (chn_status & (1 << i))
			return i + SDIO_CHN_START;
	}

	SDIOTRAN_ERR("Invalid sdio read chn!!!chn status 0x%x!!!", chn_status);

	return -1;
}
EXPORT_SYMBOL_GPL(sprd_sdio_dev_get_read_chn);

/* only wifi will use this function */
int sprd_sdio_chn_status(unsigned short chn, unsigned short *status)
{
	unsigned char status0 = 0;
	unsigned char status1 = 0;
	int err_ret;
	int ret = 0;

	SDIOTRAN_DEBUG("ENTRY");
	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);
	if (NULL == sprd_sdio_func[SDIODEV_FUNC_0]) {
		SDIOTRAN_ERR("func err");
		return -1;
	}
	marlin_runtime_get(marlin_dev);
	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_0]);

	if (0x00FF & chn) {
		status0 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],
						SDIO_CHN_REG0,
						&err_ret);
		if (status0 == 0xFF) {
			SDIOTRAN_ERR("sdio chn reg0 err");
			ret = -1;
		}
	}

	if (0xFF00 & chn) {
		status1 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],
						SDIO_CHN_REG1,
						&err_ret);
		if (status1 == 0xFF) {
			SDIOTRAN_ERR("sdio chn reg1 err");
			ret = -1;
		}
	}

	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_0]);
	marlin_runtime_put(marlin_dev);

	*status = ((status0 + (status1 << 8)) & chn);

	return ret;
}
EXPORT_SYMBOL_GPL(sprd_sdio_chn_status);

int sprd_sdio_dev_get_chn_datalen(unsigned int chn)
{
	unsigned int usedata;
	unsigned char status0, status1, status2;
	int err_ret;

	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);
	if (NULL == sprd_sdio_func[SDIODEV_FUNC_0]) {
		SDIOTRAN_ERR("func uninit!!!");
		return -1;
	}
	marlin_runtime_get(marlin_dev);

	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_0]);

	status0 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],
			(SDIO_CHN_DATALEN_BASEREG0 + 4 * chn),
						&err_ret);

	status1 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],
			(SDIO_CHN_DATALEN_BASEREG1 + 4 * chn),
						&err_ret);

	status2 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],
			(SDIO_CHN_DATALEN_BASEREG2 + 4 * chn),
						&err_ret);

	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_0]);
	marlin_runtime_put(marlin_dev);

	usedata = ((unsigned int) (status0)) + ((unsigned int) (status1) << 8) +
			((unsigned int) (status2) << 16);

	if (status0 == 0xff || status1 == 0xff || status2 == 0xff) {
		SDIOTRAN_ERR("read err!!!");
		return -1;
	}

	return usedata;
}
EXPORT_SYMBOL_GPL(sprd_sdio_dev_get_chn_datalen);

int sprd_sdio_dev_write(unsigned int chn, void *data_buf, unsigned int count)
{
	int ret, data_len;

	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);

	if (NULL == sprd_sdio_func[SDIODEV_FUNC_1]) {
		SDIOTRAN_ERR("func uninit!!!");
		return -1;
	}

	if (chn != 0 && chn != 1) {
		data_len = sprd_sdio_dev_get_chn_datalen(chn);
		if (data_len < count || data_len <= 0) {
			SDIOTRAN_ERR("chn %d, len %d, cnt %d, err!!!", chn,
				     data_len, count);
			return -1;
		}
	}
	marlin_runtime_get(marlin_dev);

	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_1]);

	ret = sdio_memcpy_toio(sprd_sdio_func[SDIODEV_FUNC_1],
				chn, data_buf, count);
	SDIOTRAN_DEBUG("chn %d send ok!!!", chn);

	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_1]);
	marlin_runtime_put(marlin_dev);

	return ret;
}
EXPORT_SYMBOL_GPL(sprd_sdio_dev_write);

int sprd_sdio_dev_read(unsigned int chn, void *read_buf, unsigned int *count)
{
	int err_ret, data_len;

	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);

	if (NULL == sprd_sdio_func[SDIODEV_FUNC_1]) {
		SDIOTRAN_ERR("func uninit!!!");
		return -1;
	}

	data_len = sprd_sdio_dev_get_chn_datalen(chn);
	if (data_len <= 0) {
		SDIOTRAN_ERR("chn %d,datelen %d err!!!", chn, data_len);
		return -1;
	}
	SDIOTRAN_DEBUG("ap recv chn %d: read_datalen is 0x%x!!!", chn,
		       data_len);

	marlin_runtime_get(marlin_dev);
	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_1]);
	err_ret = sdio_memcpy_fromio(sprd_sdio_func[SDIODEV_FUNC_1],
				     read_buf, chn, data_len);
	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_1]);
	marlin_runtime_put(marlin_dev);

	if (NULL != count)
		*count = data_len;

	return err_ret;
}
EXPORT_SYMBOL_GPL(sprd_sdio_dev_read);

int sprd_sdio_read_wlan(unsigned int chn, void *read_buf, unsigned int *count)
{
	int err_ret, data_len;

	data_len = 16384;
	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);
	if (NULL == sprd_sdio_func[SDIODEV_FUNC_1]) {
		SDIOTRAN_ERR("func uninit!!!");
		return -1;
	}
	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_1]);
	err_ret = sdio_memcpy_fromio(sprd_sdio_func[SDIODEV_FUNC_1],
					read_buf, chn, data_len);
	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_1]);
	if (NULL != count)
		*count = data_len;

	return err_ret;
}
EXPORT_SYMBOL_GPL(sprd_sdio_read_wlan);

int sprd_get_marlin_rx_status(void)
{
	int gpio_status;
	return gpio_status = gpio_get_value(sdio_data->data_irq);
}
EXPORT_SYMBOL_GPL(sprd_get_marlin_rx_status);

int sprd_sdiodev_readchn_init(unsigned int chn, void *callback, bool with_para)
{
	SDIOTRAN_DEBUG("Param, chn = %d, callback = %p", chn, callback);
	if (chn >= INVALID_SDIO_CHN) {
		SDIOTRAN_ERR("err input chn %d", chn);
		return -1;
	}

	sdio_tran_handle[chn].chn = chn;
	if (with_para == 1) {
		sdio_tran_handle[chn].tran_callback_para = callback;
		SDIOTRAN_DEBUG("sdio_tran_handle, chn = %d, callback = %p",
			       sdio_tran_handle[chn].chn,
			       sdio_tran_handle[chn].tran_callback_para);
	} else {
		sdio_tran_handle[chn].tran_callback = callback;
		SDIOTRAN_DEBUG("sdio_tran_handle, chn = %d, callback = %p",
			       sdio_tran_handle[chn].chn,
			       sdio_tran_handle[chn].tran_callback);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(sprd_sdiodev_readchn_init);

int sprd_sdiodev_readchn_uninit(unsigned int chn)
{
	if (chn >= INVALID_SDIO_CHN) {
		SDIOTRAN_ERR("err input chn %d", chn);
		return -1;
	}

	sdio_tran_handle[chn].chn = INVALID_SDIO_CHN;
	sdio_tran_handle[chn].tran_callback = NULL;
	sdio_tran_handle[chn].tran_callback_para = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(sprd_sdiodev_readchn_uninit);

void sprd_invalid_recv_flush(unsigned int chn)
{
	int len;
	char *pbuff = NULL;

	if (chn < 0 || chn > 15) {
		SDIOTRAN_ERR("err chn");
		return;
	}

	len = sprd_sdio_dev_get_chn_datalen(chn);
	if (len <= 0) {
		SDIOTRAN_ERR("chn %d, len err %d!!", chn, len);
		return;
	}
	SDIOTRAN_ERR("NO VALID DATA! CHN %d FLUSH! DATALEN %d!", chn, len);
	pbuff = kmalloc(len, GFP_KERNEL);
	if (pbuff) {
		SDIOTRAN_DEBUG("Read to Flush,chn=%d,pbuff=%p,len=%d", chn,
			       pbuff, len);
		sprd_sdio_dev_read(chn, pbuff, NULL);
		kfree(pbuff);
	} else {
		SDIOTRAN_ERR("Kmalloc %d failed!", len);
	}
}
EXPORT_SYMBOL_GPL(sprd_invalid_recv_flush);

void sprd_flush_blkchn(void)
{
	unsigned char rw_flag0, rw_flag1;
	unsigned char status0, status1;
	unsigned int chn_status;
	unsigned int chn_rw_flag;
	unsigned int tmp;
	int err_ret, read_chn;

	MARLIN_PM_RESUME_WAIT(marlin_sdio_wait);
	MARLIN_PM_RESUME_RETURN_ERROR(-1);
	sdio_claim_host(sprd_sdio_func[SDIODEV_FUNC_0]);

	status0 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],
					SDIO_CHN_REG0,
					&err_ret);
	if (status0 == 0xFF) {
		SDIOTRAN_ERR("sdio chn reg0 err");
		return;
	}

	status1 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0],
					SDIO_CHN_REG1,
					&err_ret);
	if (status1 == 0xFF) {
		SDIOTRAN_ERR("sdio chn reg1 err");
		return;
	}

	rw_flag0 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0], 0x842, &err_ret);
	if (rw_flag0 == 0xFF) {
		SDIOTRAN_ERR("sdio RW reg0 err");
		return;
	}

	rw_flag1 = sdio_readb(sprd_sdio_func[SDIODEV_FUNC_0], 0x843, &err_ret);
	if (rw_flag1 == 0xFF) {
		SDIOTRAN_ERR("sdio RW reg1 err");
		return;
	}

	sdio_release_host(sprd_sdio_func[SDIODEV_FUNC_0]);

	chn_status = (unsigned int) (status0) +
			((unsigned int) (status1) << 8);
	chn_rw_flag = (unsigned int) (rw_flag0) +
			((unsigned int) (rw_flag1) << 8);

	SDIOTRAN_ERR("chn_status is 0x%x!!!, chn_rw_flag is 0x%x!!!",
		chn_status, chn_rw_flag);
	tmp = (chn_status & (~chn_rw_flag)) & (0x0000ffff);

	for (read_chn = 0; read_chn < 16; read_chn++) {
		if (BIT_0 & (tmp >> read_chn)) {
			SDIOTRAN_ERR("BLK CHN is 0x%x!!!", read_chn);
			sprd_invalid_recv_flush(read_chn);
			return;
		}
	}
	SDIOTRAN_DEBUG("NO BLK CHN!!!");
}
EXPORT_SYMBOL_GPL(sprd_flush_blkchn);

static int sdiolog_handler(void)
{
	if (NULL != sdio_tran_handle[SDIOLOG_CHN].tran_callback) {
		SDIOTRAN_DEBUG("SDIOLOG_CHN tran_callback=%p",
			       sdio_tran_handle[SDIOLOG_CHN].tran_callback);
		sdio_tran_handle[SDIOLOG_CHN].tran_callback();
		return 0;
	} else
		return -1;
}

static int sdio_download_handler(void)
{
	if (NULL != sdio_tran_handle[DOWNLOAD_CHANNEL_READ].tran_callback) {
		SDIOTRAN_DEBUG("DOWNLOAD_CHN tran_callback=%p",
			       sdio_tran_handle
			       [DOWNLOAD_CHANNEL_READ].tran_callback);
		sdio_tran_handle[DOWNLOAD_CHANNEL_READ].tran_callback();
		return 0;
	} else
		return -1;
}

int sdio_pseudo_atc_handler(void)
{
	SDIOTRAN_ERR("ENTRY");

	if (NULL != sdio_tran_handle[PSEUDO_ATC_CHANNEL_READ].tran_callback) {
		SDIOTRAN_ERR("tran_callback=%p",
			     sdio_tran_handle
			     [PSEUDO_ATC_CHANNEL_READ].tran_callback);
		sdio_tran_handle[PSEUDO_ATC_CHANNEL_READ].tran_callback();
		return 0;
	} else
		return -1;
}

int sdio_fm_handler(void)
{
	SDIOTRAN_DEBUG("ENTRY");
	if (NULL != sdio_tran_handle[FM_CHANNEL_READ].tran_callback) {
		SDIOTRAN_ERR("tran_callback=%p",
			     sdio_tran_handle[FM_CHANNEL_READ].tran_callback);
		sdio_tran_handle[FM_CHANNEL_READ].tran_callback();
		return 0;
	} else
		return -1;
}

int sdio_pseudo_loopcheck_handler(void)
{
	SDIOTRAN_ERR("ENTRY");

	if (NULL !=
	    sdio_tran_handle[PSEUDO_ATC_CHANNEL_LOOPCHECK].tran_callback) {
		SDIOTRAN_ERR("tran_callback=%p",
			     sdio_tran_handle
			     [PSEUDO_ATC_CHANNEL_LOOPCHECK].tran_callback);
		sdio_tran_handle[PSEUDO_ATC_CHANNEL_LOOPCHECK].tran_callback();
		return 0;
	} else
		return -1;
}

int sdio_assertinfo_handler(void)
{
	SDIOTRAN_ERR("ENTRY");

	if (NULL != sdio_tran_handle[MARLIN_ASSERTINFO_CHN].tran_callback) {
		SDIOTRAN_ERR("tran_callback=%p",
			     sdio_tran_handle
			     [MARLIN_ASSERTINFO_CHN].tran_callback);
		sdio_tran_handle[MARLIN_ASSERTINFO_CHN].tran_callback();
		return 0;
	} else
		return -1;
}

static void marlin_workq(struct work_struct *work)
{
	int read_chn;
	int ret = 0;
	int gpio_status;

	SDIOTRAN_DEBUG("ENTRY");

	read_chn = sprd_sdio_dev_get_read_chn();
	switch (read_chn) {
	case SDIOLOG_CHN:
		ret = sdiolog_handler();
		break;
	case DOWNLOAD_CHANNEL_READ:
		ret = sdio_download_handler();
		break;
	case PSEUDO_ATC_CHANNEL_READ:
		ret = sdio_pseudo_atc_handler();
		break;
	case FM_CHANNEL_READ:
		ret = sdio_fm_handler();
		break;
	case PSEUDO_ATC_CHANNEL_LOOPCHECK:
		ret = sdio_pseudo_loopcheck_handler();
		break;
	case MARLIN_ASSERTINFO_CHN:
		ret = sdio_assertinfo_handler();
		break;
	case WIFI_CHN_8:
	case WIFI_CHN_9:
		break;
	default:
		SDIOTRAN_ERR("no handler for this chn %d", read_chn);
		sprd_invalid_recv_flush(read_chn);
	}

	if (-1 == ret)
		sprd_invalid_recv_flush(read_chn);

	gpio_status = sprd_get_marlin_rx_status();
	if (gpio_status)
		schedule_work(&marlin_wq);

	wake_unlock(&marlin_wakelock);
}

static irqreturn_t marlin_irq_handler(int irq, void *para)
{
	wake_lock(&marlin_wakelock);
	irq_set_irq_type(irq, IRQF_TRIGGER_RISING);

	if (sprd_get_download_fin() != 1) {
		schedule_work(&marlin_wq);

		return IRQ_HANDLED;
	}
	if (NULL != sdio_tran_handle[WIFI_CHN_8].tran_callback_para)
		sdio_tran_handle[WIFI_CHN_8].tran_callback_para(WIFI_CHN_8);

	schedule_work(&marlin_wq);

	return IRQ_HANDLED;
}

static int sdio_dev_intr_init(void)
{
	int ret;

	wake_lock_init(&marlin_wakelock, WAKE_LOCK_SUSPEND, "marlin_wakelock");

	INIT_WORK(&marlin_wq, marlin_workq);

	ret = gpio_request(sdio_data->data_irq, "marlin_irq");
	if (ret < 0) {
		SDIOTRAN_ERR("req gpio sdio_data->data_irq = %d fail!!!",
			     sdio_data->data_irq);
		return ret;
	}

	ret = gpio_direction_input(sdio_data->data_irq);
	if (ret < 0) {
		SDIOTRAN_ERR("sdio_data->data_irq = %d input set fail!!!",
			     sdio_data->data_irq);
		return ret;
	}

	sdio_irq_num = gpio_to_irq(sdio_data->data_irq);

	ret = request_irq(sdio_irq_num,
			marlin_irq_handler,
			IRQF_TRIGGER_RISING |
			IRQF_NO_SUSPEND,
			"sdio_dev_intr",
			NULL);

	if (ret != 0) {
		SDIOTRAN_ERR("request irq err!!!gpio is %d!!!",
			     sdio_data->data_irq);
		return ret;
	}
	SDIOTRAN_DEBUG("ok!!!irq is %d!!!", sdio_irq_num);

	return 0;
}

static void sdio_dev_intr_uninit(void)
{
	disable_irq_nosync(sdio_irq_num);
	free_irq(sdio_irq_num, NULL);
	gpio_free(sdio_data->data_irq);
	gpio_free(sdio_data->wake_out);
	cancel_work_sync(&marlin_wq);
	wake_lock_destroy(&marlin_wakelock);
	SDIOTRAN_DEBUG("ok!!!");
}

static void set_apsdiohal_ready(void)
{
	ap_sdio_ready = 1;
}

static void set_apsdiohal_unready(void)
{
	ap_sdio_ready = 0;
}

/* return 1 means ap sdiohal ready */
bool sprd_get_apsdiohal_status(void)
{
	return ap_sdio_ready;
}
EXPORT_SYMBOL_GPL(sprd_get_apsdiohal_status);

/* return 1 means marlin sdiohal ready */
bool sprd_get_sdiohal_status(void)
{
	return marlin_sdio_ready.marlin_sdio_init_end_tag;
}
EXPORT_SYMBOL_GPL(sprd_get_sdiohal_status);

static void clear_sdiohal_status(void)
{
	marlin_sdio_ready.marlin_sdio_init_start_tag = 0;
	marlin_sdio_ready.marlin_sdio_init_end_tag = 0;
}

static void marlinack_workq(struct work_struct *work)
{
	if (sdio_w_flag == 1)
		complete(&marlin_ack);
}

static irqreturn_t marlinwake_irq_handler(int irq, void *para)
{
	unsigned int gpio_wake_status = 0;

	gpio_wake_status = gpio_get_value(sdio_data->wake_ack);
	if (sprd_get_download_fin() != 1) {
		if (!marlin_sdio_ready.marlin_sdio_init_start_tag
		    && gpio_wake_status) {
			SDIOTRAN_ERR("start,g_val=%d", gpio_wake_status);
			irq_set_irq_type(irq, IRQF_TRIGGER_LOW);
			marlin_sdio_ready.marlin_sdio_init_start_tag = 1;
		} else if (!marlin_sdio_ready.marlin_sdio_init_end_tag
			   && (!gpio_wake_status)) {
			SDIOTRAN_ERR("end,g_val=%d", gpio_wake_status);
			irq_set_irq_type(irq, IRQF_TRIGGER_RISING);
			marlin_sdio_ready.marlin_sdio_init_end_tag = 1;
		} else {
			SDIOTRAN_ERR("marlin gpio0 err interrupt,g_val=%d",
				     gpio_wake_status);
		}
	} else {
		/* add count irq for loopcheck */
		irq_count_change++;

		/* avoid gpio jump , so need check the
		 * last and cur gpio value
		 */
		if (ack_gpio_status == gpio_wake_status) {
			SDIOTRAN_ERR("discard gpio%d irq\n",
				     sdio_data->wake_ack);
			return IRQ_HANDLED;
		}

		if (gpio_wake_status)
			irq_set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);
		else
			irq_set_irq_type(irq, IRQ_TYPE_EDGE_RISING);

		ack_gpio_status = gpio_wake_status;
		SDIOTRAN_ERR("%d-%d\n", sdio_data->wake_ack, gpio_wake_status);

		if (gpio_wake_status) {
			wake_lock(&marlinpub_wakelock);
			sleep_para.gpio_up_time = jiffies;
		} else {
			wake_unlock(&marlinpub_wakelock);
			sleep_para.gpio_down_time = jiffies;
		}

		if ((!gpio_wake_status)
		    && time_after(sleep_para.gpio_down_time,
				  sleep_para.gpio_up_time)) {
			if (jiffies_to_msecs
			    (sleep_para.gpio_down_time -
			     sleep_para.gpio_up_time) >
			    sleep_para.bt_req_time) {
				SDIOTRAN_ERR("BT REQ!!!");
				marlin_bt_wake_flag = 1;
				wake_lock_timeout(&bt_ap_wakelock,
				/* wsh */
				sleep_para.wake_lock_time);
			}

		}
	if (gpio_wake_status) {
		if (atomic_read(&gpioreq_need_pulldown)) {
				sleep_para.gpio_opt_tag = 1;
				SDIOTRAN_ERR("gpio_opt_tag-1\n");
				mod_timer(&(sleep_para.gpio_timer),
					  jiffies +
					  msecs_to_jiffies
					  (sleep_para.marlin_waketime));
				if (sdio_w_flag == 1) {
					complete(&marlin_ack);
					SDIOTRAN_ERR("ack-sem\n");
					sprd_set_marlin_sleep(0xff, 0x1);
				} else {
					SDIOTRAN_ERR("sdio_w_flag:%d\n",
						     sdio_w_flag);
					sprd_set_marlin_sleep(0xff, 0x2);
				}
			}
		}
	}

	return IRQ_HANDLED;
}

static int marlin_wake_intr_init(void)
{
	int ret;

	sdio_w_flag = 0;
	bt_wake_flag = 0;
	INIT_WORK(&marlinwake_wq, marlinack_workq);

	init_completion(&marlin_ack);

	wake_lock_init(&bt_ap_wakelock, WAKE_LOCK_SUSPEND,
		       "marlinbtup_wakelock");

	wake_lock_init(&marlinpub_wakelock, WAKE_LOCK_SUSPEND,
		       "marlinpub_wakelock");

	ret = gpio_request(sdio_data->wake_ack, "marlinwake_irq");
	if (ret < 0) {
		SDIOTRAN_ERR("req gpio sdio_data->wake_ack = %d fail!!!",
			     sdio_data->wake_ack);

		return ret;
	}
	ret = gpio_direction_input(sdio_data->wake_ack);
	if (ret < 0) {
		SDIOTRAN_ERR("sdio_data->wake_ack = %d input set fail!!!",
			     sdio_data->wake_ack);

		return ret;

	}
	marlinwake_irq_num = gpio_to_irq(sdio_data->wake_ack);
	ret = request_irq(marlinwake_irq_num, marlinwake_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
			"marlin_wake_intr", NULL);

	if (ret != 0) {
		SDIOTRAN_ERR("request irq err!!!gpio is %d!!!",
			     sdio_data->wake_ack);

		return ret;
	}
	SDIOTRAN_DEBUG("ok!!!irq is %d!!!", marlinwake_irq_num);

	return 0;
}

static void marlin_wake_intr_uninit(void)
{
	disable_irq_nosync(marlinwake_irq_num);
	free_irq(marlinwake_irq_num, NULL);
	gpio_free(sdio_data->wake_ack);
	cancel_work_sync(&marlinwake_wq);
	wake_lock_destroy(&bt_ap_wakelock);
	wake_lock_destroy(&marlinpub_wakelock);
	SDIOTRAN_DEBUG("ok!!!");
}

static void sdio_init_timer(void)
{
	init_timer(&sleep_para.gpio_timer);
	sleep_para.gpio_timer.function = gpio_timer_handler;
	sleep_para.marlin_waketime = 350;
	sleep_para.bt_req_time = 100;
	sleep_para.ack_high_time = 200;
	sleep_para.wake_lock_time = HZ / 2;
	sleep_para.gpio_opt_tag = 0;
	atomic_set(&gpioreq_need_pulldown, 1);
}

static void sdio_uninit_timer(void)
{
	del_timer(&sleep_para.gpio_timer);
	sleep_para.marlin_waketime = 0;
	sleep_para.bt_req_time = 0;
	sleep_para.ack_high_time = 0;
	sleep_para.wake_lock_time = 0;
	sleep_para.gpio_opt_tag = 0;
	atomic_set(&gpioreq_need_pulldown, 1);
}

static int marlin_sdio_probe(struct sdio_func *func,
			     const struct sdio_device_id *id)
{
	int ret;

	SDIOTRAN_DEBUG("sdio drv and dev match!!!");
	mutex_init(&g_sdio_func_lock);
	ret = get_sdio_dev_func(func);

	if (ret < 0) {
		SDIOTRAN_ERR("get func err!!!\n");
		return ret;
	}

	SDIOTRAN_DEBUG("get func ok!!!\n");
	/* Enable Function 1 */
	sdio_claim_host(sprd_sdio_func[1]);
	ret = sdio_enable_func(sprd_sdio_func[1]);
	sdio_set_block_size(sprd_sdio_func[1], 512);
	sdio_release_host(sprd_sdio_func[1]);

	if (ret < 0) {
		SDIOTRAN_ERR("enable func1 err!!! ret is %d", ret);
		return ret;
	}

	SDIOTRAN_DEBUG("enable func1 ok!!!");
	spin_lock_init(&sleep_spinlock);
	sdio_init_timer();
	clear_sdiohal_status();
	wakeup_slave_pin_init();
	marlin_wake_intr_init();
	sdio_dev_intr_init();

	set_apsdiohal_ready();
	marlin_runtime_setting(to_platform_device(marlin_dev));

	have_card = 1;

	return 0;
}

static void marlin_sdio_remove(struct sdio_func *func)
{
	SDIOTRAN_DEBUG("entry");
	sdio_uninit_timer();
	free_sdio_dev_func(func);
	sdio_dev_intr_uninit();
	marlin_wake_intr_uninit();
	set_apsdiohal_unready();
	clear_sdiohal_status();
	SDIOTRAN_DEBUG("ok");
}

static int marlin_sdio_suspend(struct device *dev)
{
	SDIOTRAN_DEBUG("[%s]enter\n", __func__);

	gpio_marlin_req_tag = gpio_get_value(sdio_data->data_irq);

	if (gpio_marlin_req_tag)
		SDIOTRAN_ERR("err marlin_req!!!");
	else
		irq_set_irq_type(sdio_irq_num, IRQF_TRIGGER_HIGH);

	gpio_marlinwake_tag = gpio_get_value(sdio_data->wake_ack);

	if (gpio_marlinwake_tag)
		SDIOTRAN_ERR("err marlinwake!!!");
	else
		irq_set_irq_type(marlinwake_irq_num, IRQF_TRIGGER_HIGH);
	/*SDIOTRAN_ERR("[%s]ok\n", __func__);*/
	smp_mb();

	return 0;
}

static int marlin_sdio_resume(struct device *dev)
{
	SDIOTRAN_ERR("[%s]enter\n", __func__);

	sleep_para.gpio_opt_tag = 0;
	sprd_set_marlin_wakeup(0, 1);
	mod_timer(&(sleep_para.gpio_timer),
		  jiffies + msecs_to_jiffies(sleep_para.marlin_waketime));
	/*SDIOTRAN_ERR("[%s]ok\n", __func__);*/
	smp_mb();

	return 0;
}

static void *sdio_dev_get_host(void)
{
	struct device *host;
	struct platform_device *pdev;

	marlin_dev = bus_find_device_by_name(&platform_bus_type,
					NULL,
					sdio_data->sdhci);
	if (marlin_dev == NULL) {
		SDIOTRAN_ERR("sdio find dev by name failed!!!");

		return NULL;
	}

	pdev = to_platform_device(marlin_dev);
	if (pdev == NULL) {
		SDIOTRAN_ERR("sdio dev get platform device failed!!!");
		return NULL;
	}

	host = platform_get_drvdata(pdev);

	return container_of(host, struct mmc_host, class_dev);
}

static const struct dev_pm_ops marlin_sdio_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(marlin_sdio_suspend, marlin_sdio_resume)
	SET_RUNTIME_PM_OPS(marlin_runtime_suspend, marlin_runtime_resume, NULL)
};

static struct sdio_driver marlin_sdio_driver = {
	.probe = marlin_sdio_probe,
	.remove = marlin_sdio_remove,
	.name = "marlin_sdio",
	.id_table = marlin_sdio_ids,
	.drv = {
		.pm = &marlin_sdio_pm_ops,
		},
};

void marlin_sdio_uninit(void)
{
	sprd_set_download_fin(0);
	sdio_unregister_driver(&marlin_sdio_driver);

	sdio_dev_host = NULL;
	have_card = 0;
	kfree(sdio_data);
	sdio_data = NULL;
	SDIOTRAN_DEBUG("ok");
}

int sprd_marlin_sdio_init(void)
{
	int ret;
	struct device_node *np;

	SDIOTRAN_DEBUG("entry");
	if (have_card == 1)
		marlin_sdio_uninit();

	atomic_set(&gpioreq_need_pulldown, 1);
	atomic_set(&is_wlan_open, 0);
	atomic_set(&set_marlin_cmplete, 1);

	sdio_data = kzalloc(sizeof(*sdio_data), GFP_KERNEL);
	if (!sdio_data)
		return -ENOMEM;
	np = of_find_node_by_name(NULL, "sprd-marlin");
	if (!np) {
		SDIOTRAN_ERR("sprd-marlin not found");
		return -1;
	}

	sdio_data->wake_ack = of_get_named_gpio(np,
				"bt-wakeup-ap-gpios", 0);
	if (sdio_data->wake_ack == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	sdio_data->data_irq = of_get_named_gpio(np,
				"ap-wakeup-cp-gpios", 0);
	if (sdio_data->data_irq == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	sdio_data->wake_out = of_get_named_gpio(np,
				"wifi-wakeup-ap-gpios", 0);
	if (sdio_data->wake_out == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	ret = of_property_read_string(np, "sdhci-name", &(sdio_data->sdhci));
	if (ret) {
		SDIOTRAN_ERR("get sdhci-name failed");

		return -1;
	}

	sdio_dev_host = sdio_dev_get_host();
	if (NULL == sdio_dev_host) {
		SDIOTRAN_ERR("get host failed!!!");
		return -1;
	}

	SDIOTRAN_DEBUG("sdio get host ok!!!");
	ret = sdio_register_driver(&marlin_sdio_driver);

	if (0 != ret) {
		SDIOTRAN_ERR("sdio register drv err!!!ret is %d!!!", ret);
		return -1;
	}

	SDIOTRAN_DEBUG("sdio register drv succ!!!");
	flush_delayed_work(&sdio_dev_host->detect);
	mmc_detect_change(sdio_dev_host, 0);

	return ret;
}
EXPORT_SYMBOL_GPL(sprd_marlin_sdio_init);
