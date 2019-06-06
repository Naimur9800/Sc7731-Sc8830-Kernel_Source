/************************************************************************
*
* File Name: himax_common_interface.c
*
*  *   Version: v1.0
*
************************************************************************/
#include "himax_platform.h"
#include "himax_common.h"
#include "himax_ic.h"
#include <linux/kernel.h>

#define TEST_RESULT_LENGTH (8 * 1200)
#define TEST_TEMP_LENGTH 8
#define MAX_ALLOC_BUFF 256
#define TEST_PASS	0
#define TEST_BEYOND_MAX_LIMIT		0x0001
#define TEST_BEYOND_MIN_LIMIT		0x0002
#ifdef TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM
#define HIMAX_PINCTRL_INIT_STATE "pmx_ts_init"
#endif
#define TP_TEST_INIT		1
#define TP_TEST_START	2
#define TP_TEST_END		3
#define MAX_NAME_LEN_50  50
#define MAX_NAME_LEN_20  20

extern struct himax_ts_data *private_ts;
extern struct himax_ic_data *ic_data;
extern struct himax_core_fp g_core_fp;
extern uint8_t HX_SMWP_EN;
extern bool fw_update_complete;
extern unsigned char IC_TYPE;
#if defined(HX_AUTO_UPDATE_FW) || defined(HX_ZERO_FLASH)
extern char *i_CTPM_firmware_name;
#endif
#if defined(HX_TP_PROC_SELF_TEST) || defined(CONFIG_TOUCHSCREEN_HIMAX_ITO_TEST)
extern int g_self_test_entered;
extern char *g_hx_crtra_file;
extern char *g_sensor_result_file;
#endif
extern int himax_read_Sensor_ID(struct i2c_client *client);

int himax_vendor_id = 0;
int himax_test_faied_buffer_length = 0;
int himax_test_failed_count = 0;
int himax_tptest_result = 0;
char CTPM_firmware_name[MAX_NAME_LEN_50] = {0};
char hx_criteria_csv_name[MAX_NAME_LEN_20] = {0};
char g_hx_save_file_path[MAX_NAME_LEN_20] = {0};
char g_hx_save_file_name[MAX_NAME_LEN_50] = {0};
char hx_vendor_name[MAX_NAME_LEN_20] = {0};

char *himax_test_failed_node_buffer = NULL;
char *himax_test_temp_buffer = NULL;
u8 *himax_test_failed_node = NULL;
char *g_file_path = NULL;
struct tpvendor_t himax_vendor_l[] = {
	{0x11, HXTS_VENDOR_11_NAME},
	{0x12, HXTS_VENDOR_12_NAME},
	{0x14, HXTS_VENDOR_14_NAME},
	{0x21, HXTS_VENDOR_21_NAME},
	{0x22, HXTS_VENDOR_22_NAME},
	{0x24, HXTS_VENDOR_24_NAME},
	{0x41, HXTS_VENDOR_41_NAME},
	{0x42, HXTS_VENDOR_42_NAME},
	{0x44, HXTS_VENDOR_44_NAME},
	{VENDOR_END, "Unknown"},
};

#ifdef TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM
int himax_platform_pinctrl_init(struct himax_i2c_platform_data *pdata)
{
	int ret = 0;
	struct i2c_client *client = private_ts->client;

	/* Get pinctrl if target uses pinctrl */
	pdata->ts_pinctrl = devm_pinctrl_get(&(client->dev));
	if (IS_ERR_OR_NULL(pdata->ts_pinctrl)) {
		ret = PTR_ERR(pdata->ts_pinctrl);
		E("Target does not use pinctrl %d\n", ret);
		goto err_pinctrl_get;
	}

	pdata->pinctrl_state_init = pinctrl_lookup_state(pdata->ts_pinctrl, HIMAX_PINCTRL_INIT_STATE);
	if (IS_ERR_OR_NULL(pdata->pinctrl_state_init)) {
		ret = PTR_ERR(pdata->pinctrl_state_init);
		E("Can not lookup %s pinstate %d\n", HIMAX_PINCTRL_INIT_STATE, ret);
		goto err_pinctrl_lookup;
	}

	ret = pinctrl_select_state(pdata->ts_pinctrl, pdata->pinctrl_state_init);
	if (ret < 0) {
		E("failed to select pin to init state");
		goto err_select_init_state;
	}

	return 0;

err_select_init_state:
err_pinctrl_lookup:
	devm_pinctrl_put(pdata->ts_pinctrl);
err_pinctrl_get:
	pdata->ts_pinctrl = NULL;
	return ret;
}
#endif

static int himax_get_chip_vendor(int vendor_id)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(himax_vendor_l); i++) {
		if (himax_vendor_l[i].vendor_id == vendor_id || himax_vendor_l[i].vendor_id == VENDOR_END) {
			strlcpy(hx_vendor_name,  himax_vendor_l[i].vendor_name, sizeof(hx_vendor_name));
			snprintf(CTPM_firmware_name, sizeof(CTPM_firmware_name),
				"Himax_firmware_%s.bin", hx_vendor_name);
			break;
		}
	}
	return 0;
}

int himax_get_tp_vendor_info(void)
{
	int vendor_sensor_id = 0;

	vendor_sensor_id = himax_read_Sensor_ID(private_ts->client);
	I("HXTP,vendor_sensor_id =%d\n", vendor_sensor_id);
#if defined(HX_AUTO_UPDATE_FW) || defined(HX_ZERO_FLASH)
	himax_get_chip_vendor(vendor_sensor_id);
	i_CTPM_firmware_name = CTPM_firmware_name;
	I("HXTP,firmware name is %s\n", i_CTPM_firmware_name);
#endif
	snprintf(hx_criteria_csv_name, sizeof(hx_criteria_csv_name), "hx_criteria_%02x.csv", vendor_sensor_id);
	g_hx_crtra_file = hx_criteria_csv_name;
	return 0;
}

static int tpd_init_tpinfo(struct tpd_classdev_t *cdev)
{
	I("tpd_init_tpinfo\n");
	himax_read_FW_ver(private_ts->client);
	strlcpy(cdev->ic_tpinfo.vendor_name, hx_vendor_name, sizeof(cdev->ic_tpinfo.vendor_name));
	cdev->ic_tpinfo.chip_model_id = TS_CHIP_HIMAX;
	cdev->ic_tpinfo.module_id = ic_data->vendor_sensor_id;
	cdev->ic_tpinfo.firmware_ver = ic_data->vendor_fw_ver;
	cdev->ic_tpinfo.i2c_addr = 0x48;
	switch (IC_TYPE) {
	case HX_85XX_D_SERIES_PWON:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_HX852xD");
		break;
	case HX_85XX_E_SERIES_PWON:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_HX852xE");
		break;
	case HX_85XX_ES_SERIES_PWON:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_HX852xES");
		break;
	case HX_85XX_F_SERIES_PWON:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_HX852xF");
		break;
	case HX_83100A_SERIES_PWON:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_HX83100A");
		break;
	case HX_83102A_SERIES_PWON:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_ X83102A");
		break;
	case HX_83102B_SERIES_PWON:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_HX83102B");
		break;
	case HX_83103A_SERIES_PWON:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_HX83103A");
		break;
	case HX_83110A_SERIES_PWON:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_HX83110A");
		break;
	case HX_83110B_SERIES_PWON:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_HX83110B");
		break;
	case HX_83111B_SERIES_PWON:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_HX83111B");
		break;
	case HX_83112A_SERIES_PWON:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_HX83112A");
		break;
	case HX_83112B_SERIES_PWON:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_HX83112B");
		break;
	default:
		snprintf(cdev->ic_tpinfo.tp_name, sizeof(cdev->ic_tpinfo.tp_name), "Himax_Type_error");
	}
	if (IC_TYPE < 8) {
		cdev->ic_tpinfo.config_ver = ic_data->vendor_config_ver;
	} else {
		cdev->ic_tpinfo.config_ver =  ic_data->vendor_touch_cfg_ver;
		cdev->ic_tpinfo.display_ver = ic_data->vendor_display_cfg_ver;
	}
	return 0;
}

#ifdef HX_SMART_WAKEUP
static int tpd_get_wakegesture(struct tpd_classdev_t *cdev)
{
	struct himax_ts_data *ts = private_ts;

	I("%s wakeup_gesture_enable val is:%d.\n", __func__, ts->SMWP_enable);
	cdev->b_gesture_enable = ts->SMWP_enable;
	return cdev->b_gesture_enable;
}

static int tpd_enable_wakegesture(struct tpd_classdev_t *cdev, int enable)
{
	struct himax_ts_data *ts = private_ts;

	ts->SMWP_enable = enable;

	g_core_fp.fp_set_SMWP_enable(ts->SMWP_enable, ts->suspended);
	HX_SMWP_EN = ts->SMWP_enable;
	I("%s: SMART_WAKEUP_enable = %d.\n", __func__, HX_SMWP_EN);

	return ts->SMWP_enable;
}
#endif

static int himax_i2c_reg_read(struct tpd_classdev_t *cdev, char addr, u8 *data, int len)
{
	if (i2c_himax_read(private_ts->client, addr, data, len, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return -EIO;
	}
	return 0;
}

static int himax_i2c_reg_write(struct tpd_classdev_t *cdev, char addr, u8 *data, int len)
{
	if (i2c_himax_write(private_ts->client, addr, data, len, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return -EIO;
	}
	return 0;
}

static int himax_tp_fw_upgrade(struct tpd_classdev_t *cdev, char *fw_name, int fwname_len)
{
	char fileName[128] = { 0 };
	int result = 0;
#ifndef HX_ZERO_FLASH
	int fw_type = 0;
	const struct firmware *fw = NULL;
#endif

	memset(fileName, 0, sizeof(fileName));
	snprintf(fileName, sizeof(fileName), "%s", fw_name);
	fileName[fwname_len - 1] = '\0';
	I("%s: upgrade from file(%s) start!\n", __func__, fileName);
	himax_int_enable(private_ts->client->irq, 0);
#ifdef HX_CHIP_STATUS_MONITOR
	g_chip_monitor_data->HX_CHIP_POLLING_COUNT = 0;
	g_chip_monitor_data->HX_CHIP_MONITOR_EN = 0;
	cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
#endif
	fw_update_complete = false;
	result = request_firmware(&fw, fileName, private_ts->dev);
	if (result < 0) {
		I("fail to request_firmware fwpath: %s (ret:%d)\n", fileName, result);
		return result;
	}
	I("%s: FW image: %02X, %02X, %02X, %02X\n", __func__, fw->data[0], fw->data[1], fw->data[2],
	  fw->data[3]);

	fw_type = (fw->size) / 1024;
	/* start to upgrade */
	himax_int_enable(private_ts->client->irq, 0);
	I("Now FW size is : %dk\n", fw_type);
	switch (fw_type) {
	case 32:
		if (fts_ctpm_fw_upgrade_with_sys_fs_32k(private_ts->client,
							(unsigned char *)fw->data, fw->size, false) == 0) {
			E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
			fw_update_complete = false;
		} else {
			I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
			fw_update_complete = true;
		}
		break;
	case 60:
		if (fts_ctpm_fw_upgrade_with_sys_fs_60k(private_ts->client,
							(unsigned char *)fw->data, fw->size, false) == 0) {
			E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
			fw_update_complete = false;
		} else {
			I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
			fw_update_complete = true;
		}
		break;
	case 64:
		if (fts_ctpm_fw_upgrade_with_sys_fs_64k(private_ts->client,
							(unsigned char *)fw->data, fw->size, false) == 0) {
			E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
			fw_update_complete = false;
		} else {
			I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
			fw_update_complete = true;
		}
		break;
	case 124:
		if (fts_ctpm_fw_upgrade_with_sys_fs_124k(private_ts->client,
							 (unsigned char *)fw->data, fw->size, false) == 0) {
			E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
			fw_update_complete = false;
		} else {
			I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
			fw_update_complete = true;
		}
		break;
	case 128:
		if (fts_ctpm_fw_upgrade_with_sys_fs_128k(private_ts->client,
							 (unsigned char *)fw->data, fw->size, false) == 0) {
			E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
			fw_update_complete = false;
		} else {
			I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
			fw_update_complete = true;
		}
		break;

	default:
		E("%s: Flash command fail: %d\n", __func__, __LINE__);
		fw_update_complete = false;
		break;
	}
	release_firmware(fw);
	himax_read_FW_ver(private_ts->client);
	himax_touch_information(private_ts->client);
#ifdef HX_RST_PIN_FUNC
	himax_ic_reset(true, false);
#else
	himax_sense_on(private_ts->client, 0);
#endif
	himax_int_enable(private_ts->client->irq, 1);

#ifdef HX_CHIP_STATUS_MONITOR
	g_chip_monitor_data->HX_CHIP_POLLING_COUNT = 0;
	g_chip_monitor_data->HX_CHIP_MONITOR_EN = 1;
	queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor,
			   g_chip_monitor_data->HX_POLLING_TIMES * HZ);
#endif
	return 0;
}

static int himax_gpio_shutdown_config(void)
{
#ifdef HX_RST_PIN_FUNC
	if (gpio_is_valid(private_ts->rst_gpio)) {
		I("%s\n", __func__);
		gpio_set_value(private_ts->rst_gpio, 0);
	}
#endif
	return 0;
}

/* himax TP slef test*/

static int himax_test_init(void)
{
	I("%s:enter\n", __func__);
	himax_test_failed_node_buffer = kzalloc(TEST_RESULT_LENGTH, GFP_KERNEL);
	himax_test_temp_buffer = kzalloc(TEST_TEMP_LENGTH, GFP_KERNEL);
	himax_test_failed_node = kzalloc((ic_data->HX_TX_NUM * ic_data->HX_RX_NUM), GFP_KERNEL);
	g_file_path = kzalloc(MAX_ALLOC_BUFF, GFP_KERNEL);
	if (himax_test_failed_node_buffer == NULL || himax_test_temp_buffer == NULL ||
		himax_test_failed_node == NULL || g_file_path == NULL) {
		if (himax_test_failed_node_buffer != NULL)
			kfree(himax_test_failed_node_buffer);
		if (himax_test_temp_buffer != NULL)
			kfree(himax_test_temp_buffer);
		if (himax_test_failed_node != NULL)
			kfree(himax_test_failed_node);
		if (g_file_path != NULL)
			kfree(g_file_path);
		E("%s:alloc memory failde!\n", __func__);
		return -ENOMEM;
	}
	himax_test_faied_buffer_length = 0;
	himax_test_failed_count = 0;
	himax_tptest_result = 0;
	return 0;
}

static void himax_test_buffer_free(void)
{
	I("%s:enter\n", __func__);
	if (himax_test_failed_node_buffer != NULL)
		kfree(himax_test_failed_node_buffer);
	if (himax_test_temp_buffer != NULL)
		kfree(himax_test_temp_buffer);
	if (himax_test_failed_node != NULL)
		kfree(himax_test_failed_node);
	if (g_file_path != NULL)
		kfree(g_file_path);
}

static int himax_save_failed_node_to_buffer(char *tmp_buffer, int length)
{

	if (himax_test_failed_node_buffer == NULL) {
		E("warning:himax_test_failed_node_buffer is null.");
		return -EPERM;
	}

	snprintf(himax_test_failed_node_buffer + himax_test_faied_buffer_length,
		 (TEST_RESULT_LENGTH - himax_test_faied_buffer_length), tmp_buffer);
	himax_test_faied_buffer_length += length;
	himax_test_failed_count++;

	return 0;
}

int himax_save_failed_node(int failed_node)
{
	int i_len = 0;
	int tx = 0;
	int rx = 0;

	tx = failed_node / ic_data->HX_RX_NUM;
	rx = failed_node % ic_data->HX_RX_NUM;
	if (himax_test_failed_node == NULL)
		return -EPERM;
	if (himax_test_failed_node[failed_node] == 0) {
		if (himax_test_temp_buffer != NULL) {
			i_len = snprintf(himax_test_temp_buffer, TEST_TEMP_LENGTH, ",%d,%d", tx, rx);
			himax_save_failed_node_to_buffer(himax_test_temp_buffer, i_len);
			himax_test_failed_node[failed_node] = 1;
			return 0;
		} else {
			return -EPERM;
		}
	} else {
		return 0;
	}
}

static int tpd_test_save_file_path_store(struct tpd_classdev_t *cdev, const char *buf)
{
	memset(g_hx_save_file_path, 0, sizeof(g_hx_save_file_path));
	snprintf(g_hx_save_file_path, sizeof(g_hx_save_file_path), "%s", buf);

	I("save file path:%s.", g_hx_save_file_path);

	return 0;
}

static int tpd_test_save_file_path_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_hx_save_file_path);

	return num_read_chars;
}

static int tpd_test_save_file_name_store(struct tpd_classdev_t *cdev, const char *buf)
{
	memset(g_hx_save_file_name, 0, sizeof(g_hx_save_file_name));
	snprintf(g_hx_save_file_name, sizeof(g_hx_save_file_name), "%s", buf);

	I("save file name:%s.", g_hx_save_file_name);

	return 0;
}

static int tpd_test_save_file_name_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_hx_save_file_name);

	return num_read_chars;
}

static int tpd_test_cmd_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;
	int i_len = 0;

	I("%s:enter\n", __func__);
	i_len = snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d", himax_tptest_result, ic_data->HX_TX_NUM,
			 ic_data->HX_RX_NUM, himax_test_failed_count);
	I("tpd test resutl:%d && rawdata node failed count:%d.\n", himax_tptest_result, himax_test_failed_count);

	if (himax_test_failed_node_buffer != NULL) {
		i_len += snprintf(buf + i_len, PAGE_SIZE - i_len, himax_test_failed_node_buffer);
	}
	E("tpd  test:%s.\n", buf);
	num_read_chars = i_len;
	return num_read_chars;
}

static int tpd_test_cmd_store(struct tpd_classdev_t *cdev, const char *buf)
{
	unsigned long command = 0;
	int retval = 0;
	int val = 0x00;

	I("%s:enter\n", __func__);
	retval = kstrtoul(buf, 10, &command);
	if (retval) {
		E("invalid param:%s", buf);
		return -EIO;
	}
	if (command == TP_TEST_INIT) {
		retval = himax_test_init();
		if (retval < 0) {
			E("%s:alloc memory failde!\n", __func__);
			return -ENOMEM;
		}
	} else if (command == TP_TEST_START) {
		I("%s:start TP test.\n", __func__);
		if (g_file_path != NULL) {
			snprintf(g_file_path, MAX_ALLOC_BUFF, "%s%s", g_hx_save_file_path, g_hx_save_file_name);
			g_sensor_result_file = g_file_path;
		}
		himax_int_enable(private_ts->client->irq, 0);	/* disable irq */
		g_self_test_entered = 1;
		val = himax_chip_self_test(private_ts->client);
#ifdef HX_ESD_RECOVERY
		HX_ESD_RESET_ACTIVATE = 1;
#endif
		himax_int_enable(private_ts->client->irq, 1);	/* enable irq */
		himax_sense_on(private_ts->client, 0x01);
		if (val == 0x00) {
			himax_tptest_result = TEST_PASS;
			I("Self_Test Pass\n");
		} else if (val >= 0x01 && val <= 0x03) {
			himax_tptest_result = himax_tptest_result | TEST_BEYOND_MAX_LIMIT | TEST_BEYOND_MIN_LIMIT;
			E("Self_Test Fail\n");
		} else {
			himax_tptest_result = -EIO;
			E("self test data init Fail\n");
		}
		g_self_test_entered = 0;

	} else if (command == TP_TEST_END) {
		himax_test_buffer_free();
	} else {
		E("invalid command %ld", command);
	}
	return 0;
}

static int tpd_test_channel_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf, PAGE_SIZE, "%d %d", ic_data->HX_TX_NUM, ic_data->HX_RX_NUM);

	return num_read_chars;
}

void hiamx_tpd_register_fw_class(void)
{
	I("tpd_register_fw_class\n");
	tpd_fw_cdev.get_tpinfo = tpd_init_tpinfo;
#ifdef HX_SMART_WAKEUP
	tpd_fw_cdev.get_gesture = tpd_get_wakegesture;
	tpd_fw_cdev.wake_gesture = tpd_enable_wakegesture;
#endif
	tpd_fw_cdev.tp_i2c_reg_read = himax_i2c_reg_read;
	tpd_fw_cdev.tp_i2c_reg_write = himax_i2c_reg_write;
	tpd_fw_cdev.reg_char_num = REG_CHAR_NUM_2;
	tpd_fw_cdev.tp_fw_upgrade = himax_tp_fw_upgrade;
	tpd_fw_cdev.tpd_gpio_shutdown = himax_gpio_shutdown_config;

	tpd_fw_cdev.tpd_test_set_save_filepath = tpd_test_save_file_path_store;
	tpd_fw_cdev.tpd_test_get_save_filepath = tpd_test_save_file_path_show;
	tpd_fw_cdev.tpd_test_set_save_filename = tpd_test_save_file_name_store;
	tpd_fw_cdev.tpd_test_get_save_filename = tpd_test_save_file_name_show;
	tpd_fw_cdev.tpd_test_set_cmd = tpd_test_cmd_store;
	tpd_fw_cdev.tpd_test_get_cmd = tpd_test_cmd_show;
	tpd_fw_cdev.tpd_test_get_channel_info = tpd_test_channel_show;
	snprintf(g_hx_save_file_path, sizeof(g_hx_save_file_path), "%s", "/sdcard/");
	snprintf(g_hx_save_file_name, sizeof(g_hx_save_file_name), "%s", "hx_test_result.txt");
}
