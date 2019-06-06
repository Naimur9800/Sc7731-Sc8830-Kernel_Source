#include "himax_ito_test.h"

extern struct himax_ic_data *ic_data;
extern struct himax_ts_data *private_ts;
extern struct himax_report_data *hx_touch_data;
extern int g_diag_command;
/* extern int Selftest_flag; */
extern uint16_t *mutual_bank;
extern uint16_t *self_bank;
extern uint8_t x_channel;
extern uint8_t y_channel;
struct himax_ito_data *hx_ito;
extern int g_self_test_entered;

/********* Get status of ito test*********/
void ito_set_step_status(uint8_t status)
{
	ito_test_step = status;
}

uint8_t ito_get_step_status(void)
{
	return ito_test_step;
}

void ito_set_result_status(uint8_t status)
{
	ito_test_result = status;
}

uint8_t ito_get_result_status(void)
{
	return ito_test_result;
}

/*****************************************/
/*********** Get thx setting ***********/

static void himax_get_thx_info(uint32_t *dataBuf)
{
	max_mutual_baseC_percent = (uint8_t) dataBuf[1];
	min_mutual_baseC_percent = (uint8_t) dataBuf[2];	/* 10% */

	max_self_baseC_percent = (uint8_t) dataBuf[3];	/* 90% */
	min_self_baseC_percent = (uint8_t) dataBuf[4];	/* 10% */

	mutual_dev_fail_upper = (uint8_t) dataBuf[5];	/* 30% */
	mutual_dev_fail_lower = -(uint8_t) dataBuf[6];	/* -30% */

	self_dev_fail_upper = (uint8_t) dataBuf[7];	/* 30% */
	self_dev_fail_lower = -(uint8_t) dataBuf[8];	/* -30% */

	noice_thx = (uint8_t) dataBuf[9];

	I("%s : max_mutual_baseC_percent = %d\n", __func__, max_mutual_baseC_percent);
	I("%s : min_mutual_baseC_percent = %d\n", __func__, min_mutual_baseC_percent);
	I("%s : max_self_baseC_percent = %d\n", __func__, max_self_baseC_percent);
	I("%s : min_self_baseC_percent = %d\n", __func__, min_self_baseC_percent);
	I("%s : mutual_dev_fail_upper = %d\n", __func__, mutual_dev_fail_upper);
	I("%s : mutual_dev_fail_lower = %d\n", __func__, mutual_dev_fail_lower);
	I("%s : self_dev_fail_upper = %d\n", __func__, self_dev_fail_upper);
	I("%s : self_dev_fail_lower = %d\n", __func__, self_dev_fail_lower);
	I("%s : noice_thx = %d\n", __func__, noice_thx);

}

/*****************************************/
/*********** Write config table***********/
static void himax_write_config_table(uint8_t type, uint32_t *dataBuf, int data_size)
{
	int i;
	uint8_t writeBuf[100];

	I("%s: Enter type = %d, dataBuf[0] = %02X, data_size = %d", __func__, type, dataBuf[0], data_size);
	for (i = 0; i < data_size; i++) {
		/* I("dataBuf[%d] = 0x%02X\n", i, dataBuf[i]); */
		writeBuf[i] = (uint8_t) dataBuf[i];
	}

	if (type == 1) {/* FE */

		switch (writeBuf[0]) {
		case 0x02:
			c8 =  kzalloc((data_size * sizeof(uint8_t)), GFP_KERNEL);
			memcpy(c8, writeBuf, data_size * sizeof(uint8_t));
			break;
		case 0x6C:
			c9 =  kzalloc((data_size * sizeof(uint8_t)), GFP_KERNEL);
			memcpy(c9, writeBuf, data_size * sizeof(uint8_t));
			break;
		case 0xE0:
			c10 = kzalloc((data_size * sizeof(uint8_t)), GFP_KERNEL);
			memcpy(c10, writeBuf, data_size * sizeof(uint8_t));
			break;
		case 0xE6:
			c11 = kzalloc((data_size * sizeof(uint8_t)), GFP_KERNEL);
			memcpy(c11, writeBuf, data_size * sizeof(uint8_t));
			break;
		}
		himax_register_write(private_ts->client, writeBuf, data_size - 1, &writeBuf[1], 1);
	} else {
		switch (writeBuf[0]) {
		case 0xE1:
			c1 = kzalloc((data_size * sizeof(uint8_t)), GFP_KERNEL);
			memcpy(c1, writeBuf, data_size * sizeof(uint8_t));
			break;
		case 0xBC:
			c2 = kzalloc((data_size * sizeof(uint8_t)), GFP_KERNEL);
			memcpy(c2, writeBuf, data_size * sizeof(uint8_t));
			break;
		case 0xC5:
			c3 = kzalloc((data_size * sizeof(uint8_t)), GFP_KERNEL);
			memcpy(c3, writeBuf, data_size * sizeof(uint8_t));
			break;
		case 0xC6:
			c4 = kzalloc((data_size * sizeof(uint8_t)), GFP_KERNEL);
			memcpy(c4, writeBuf, data_size * sizeof(uint8_t));
			break;
		case 0xEC:
			c5 = kzalloc((data_size * sizeof(uint8_t)), GFP_KERNEL);
			memcpy(c5, writeBuf, data_size * sizeof(uint8_t));
			break;
		case 0x3F:
			c6 = kzalloc((data_size * sizeof(uint8_t)), GFP_KERNEL);
			memcpy(c6, writeBuf, data_size * sizeof(uint8_t));
			break;
		case 0xA4:
			c7 = kzalloc((data_size * sizeof(uint8_t)), GFP_KERNEL);
			memcpy(c7, writeBuf, data_size * sizeof(uint8_t));
			break;
		}
		himax_register_write(private_ts->client, writeBuf, data_size - 1, &writeBuf[1], 0);
	}
}

/*****************************************/
/*********** Get golden baseC***********/
static void himax_get_golden_baseC(uint8_t type, uint32_t *dataBuf, int data_size)
{
	int ini_data_size = 0;

	I("%s: Enter type = %d, dataBuf[0] = %02X, data_size = %d", __func__, type, dataBuf[0], data_size);
	switch (type) {
	case GOLDEN_MUTUAL_DATA_TYPE:
		ini_data_size = data_size > mutual_num ? data_size : mutual_num;
		golden_mutual_baseC = kzalloc((ini_data_size * sizeof(uint32_t)), GFP_KERNEL);
		memcpy(golden_mutual_baseC, dataBuf, data_size * sizeof(uint32_t));
		break;
	case GOLDEN_SELF_RX_DATA_TYPE:
		ini_data_size = data_size > x_channel ? data_size : x_channel;
		golden_rx_self_baseC = kzalloc((ini_data_size * sizeof(uint32_t)), GFP_KERNEL);
		memcpy(golden_rx_self_baseC, dataBuf, data_size * sizeof(uint32_t));
		break;
	case GOLDEN_SELF_TX_DATA_TYPE:
		ini_data_size = data_size > y_channel ? data_size : y_channel;
		golden_tx_self_baseC = kzalloc((ini_data_size * sizeof(uint32_t)), GFP_KERNEL);
		memcpy(golden_tx_self_baseC, dataBuf, data_size * sizeof(uint32_t));
		break;
	}
}

/* int mutual_num = 240;//wdd for test */

typedef struct {
	uint8_t reg;
	char name[CONFIG_TABLE_CHAR_REG_NAME_SIZE];
} *PChARREG, CHARREG;
static CHARREG charRegArray[CONFIG_TABLE_CHAR_REG_NUM] = {
	{TEST_THX_SETTING, CONFIG_TEST_THX_SETTING},
	{EN_DEFAULT_CFG, CONFIG_TABLE_EN_DEFAULT_CFG},
	{CFG_TABLE_DATE, CONFIG_TABLE_CFG_TABLE_DATE},
	{CFG_TABLE_VERSION, CONFIG_TABLE_CFG_TABLE_VERSION},
	{FW_MAJOR_VERSION, CONFIG_TABLE_FW_MAJOR_VERSION},
	{FW_MINOR_VERSION, CONFIG_TABLE_FW_MINOR_VERSION},
	{CFG_FW_SIGN, CONFIG_TABLE_CFG_FW_SIGN},
	{CFG_TABLE_INFO, CONFIG_TABLE_CFG_TABLE_INFO},
	{CFG_UNUSED, CONFIG_TABLE_CFG_UNUSED},
	{CFB_ADC_SEQ, CONFIG_TABLE_CFB_ADC_SEQ},
	{CFB_FIT_CURVE, CONFIG_TABLE_CFB_FIT_CURVE}
};

static CHARREG charMutualSelfRefArray[CHAR_MUTUAL_SELF_REF_NUM] = {
	{GOLDEN_MUTUAL_DATA_TYPE, GOLDEN_MUTUAL_STR},
	{GOLDEN_SELF_RX_DATA_TYPE, GOLDEN_SELF_RX_STR},
	{GOLDEN_SELF_TX_DATA_TYPE, GOLDEN_SELF_TX_STR}
};

static uint8_t himax_char_to_int(char data)
{
	if ((data >= '0') && (data <= '9')) {
		return (data - '0');
	} else if ((data >= 'A') && (data <= 'F')) {
		return (data - 'A' + 10);
	} else if ((data >= 'a') && (data <= 'f')) {
		return (data - 'a' + 10);
	} else {
		return 0;
	}
}

static uint32_t himax_str_to_int(char *data, uint8_t length, uint8_t format)
{
	uint32_t iData = 0;
	uint8_t cData = 0;
	uint8_t ui = 0;

	if (data == NULL) {
		return -EPERM;
	}
	for (ui = 0; ui < length; ui++) {

		cData = himax_char_to_int(*(data + ui));
		if (format) {
			iData <<= 4;
		} else {
			iData *= 10;
		}

		iData += cData;
	}

	return iData;
}

static uint32_t himax_dec_str_to_int(char *data, uint8_t length)
{
	uint8_t format = HIMAX_STR_TO_DEC_FORMAT;

	return himax_str_to_int(data, length, format);
}

static uint32_t himax_hex_str_to_int(char *data, uint8_t length)
{
	uint8_t format = HIMAX_STR_TO_HEX_FORMAT;

	return himax_str_to_int(data, length, format);
}

/* deal one { .... } item */
static size_t himax_get_comma_data(char *strStart, size_t searchSize, uint32_t *data, size_t bufSize, int *type)
{
	int i = 0;
	size_t retval = 0;
	char *hexDataPos = NULL;
	char *commaPos = NULL;
	char *pDataPos = NULL;
	char *pTempDataPos = NULL;
	char *itemStart = strStart;
	uint8_t dataLength = 0;
	int itemOffset = 0;

	do {
		commaPos = strnchr(itemStart, searchSize - itemOffset, ',');

		hexDataPos = strnstr(itemStart, CONFIG_TABLE_DATA_MATCH_STR, searchSize - itemOffset);

		pDataPos = itemStart + searchSize - itemOffset;	/* item end postion " }" */
		for (i = 0; i < 10; i++) {/* find first num data location */
			pTempDataPos = strnchr(itemStart, searchSize - itemOffset, '0' + i);

			if (pTempDataPos) {
				pDataPos = pDataPos > pTempDataPos ? pTempDataPos : pDataPos;
			}
		}

		if (pDataPos == (itemStart + searchSize - itemOffset)) {	/* can't find data */
			I("%s:can not find more data.\n", __func__);
			break;
		}

		if (!commaPos) {/* data without comma == last data */
			/* *data = himax_char_to_int(*(hexDataPos+2))*16 + himax_char_to_int(*(hexDataPos+3)); */
			/* itemStart + serchSize  -itemOffset point to  ==> " }" ahead one valid data */
			dataLength = itemStart + searchSize - itemOffset - pDataPos + 1;
			if (pDataPos == hexDataPos)	 { /* hex data */
				/* move the point to the string "0x next char location */
				*data = himax_hex_str_to_int(pDataPos + 2, 2);
			} else {	/* dec data */
				/* I("%s:last data load dataLength = %d\n",__func__, dataLength); */
				*data = himax_dec_str_to_int(pDataPos, dataLength);
			}

			I("%s:last data load\n", __func__);
		} else if (commaPos > pDataPos) {/*data before comma as txt data"0x55," hexDataPos ->0x55,<-commaPos*/

			if (pDataPos == hexDataPos)	 {/* hex data */

				if ((commaPos - hexDataPos) == CONFIG_TABLE_NORMAL_REG_STR_SIZE) {
			/* *data = himax_char_to_int(*(hexDataPos+2))*16 + himax_char_to_int(*(hexDataPos+3)); */
					*data = himax_hex_str_to_int(hexDataPos + 2, 2);
					itemStart = commaPos + 1;	/* point to next item */
					data++;	/* point to next data buf */
				} else if ((commaPos - hexDataPos) == CONFIG_TABLE_FE_REG_STR_SIZE) {
					if (!(strncmp(itemStart + 2, CONFIG_TABLE_FE_REG_MATCH_STR,
						CONFIG_TABLE_FE_REG_MATCH_SIZE))) {
						/* as txt data "0xFE(55), */
						*type = 1;
			/* *data = himax_char_to_int(*(hexDataPos+2+3))*16 + himax_char_to_int(*(hexDataPos+3+3));*/
						*data = himax_hex_str_to_int(hexDataPos + 2 + 3, 2);
						itemStart = commaPos + 1;	/* point to next item */
						data++;	/* point to next data buf */
					} else {
						I("%s:Can not march the FE format\n", __func__);
						break;
					}
				} else {
					I("%s:Can not march the FE & NORMAL format\n", __func__);
					break;
				}
			} else {/* dec data */

				dataLength = commaPos - pDataPos;
				*data = himax_dec_str_to_int(pDataPos, dataLength);
				itemStart = commaPos + 1;	/* point to next item */
				data++;	/* point to next data buf */
			}
		} else {
		/* data after comma as txt data  "EN_DEFAULT_CFG, 0x55"   commaPos ->, 0x55<-hexDataPos */
			/* *data=0;//wdd:need add code */
			for (i = 0; i < CONFIG_TABLE_CHAR_REG_NUM; i++) {
				if (!(strncmp(itemStart + 2, charRegArray[i].name, strlen(charRegArray[i].name)))) {
					break;
				}
			}

			if (i < CONFIG_TABLE_CHAR_REG_NUM) {/* find char reg in array */

				/* *type = 0; */
				*data = charRegArray[i].reg;
				itemStart = commaPos + 1;	/* point to next item */
				data++;	/* point to next data buf */
			} else {
				for (i = 0; i < CHAR_MUTUAL_SELF_REF_NUM; i++) {
					if (!(strncmp(itemStart + 2, charMutualSelfRefArray[i].name,
						strlen(charMutualSelfRefArray[i].name)))) {
						/* Compare string form "{ " <= next char */
						break;
					}
				}

				if (i < CHAR_MUTUAL_SELF_REF_NUM) {
					/* find char reg in array */
					*type = charMutualSelfRefArray[i].reg;
					itemStart = commaPos + 1;	/* point to next item */
				} else {
					I("%s:Can not march the CHAR REG format\n", __func__);
					break;
				}
			}

		}

		itemOffset = itemStart - strStart + 1;

		retval++;
		if (retval > bufSize) {
			break;
		}
	} while (commaPos);/* can't find comma as have no more data */

	return retval;
}

int8_t himax_load_config_table(void)
{
	int i = 0;
	struct file *fn = NULL;
	mm_segment_t old_fs;
	char *result_buff = NULL;
	char *buff_cursor = NULL;
	char *findStartCharPtr = NULL;
	char *findEndCharPtr = NULL;
	int readRet = 0;
	loff_t fileOffset = 0;
	int itemStartOffset = 0;
	int itemEndOffset = 0;
	int iNextDo = 1;
	/* int iMatchItem=0; */
	int iDataBufLoadSize = 0;
	uint32_t *dataBuf = NULL;
	int type = 0;
	size_t maxItemStrSize = 0;
	size_t maxItemDataSize = 0;
	uint8_t data_check[6] = { 0 };
	uint8_t chk_cnt = 0;
	int8_t ret = 0;

	fn = filp_open(CONFIG_TABLE_FILE, O_RDWR | O_APPEND, 0);/* O_CREAT | */

	if (IS_ERR(fn)) {
		I("%s: Can not open file\n", __func__);
		ret = OPEN_FILE_FAIL;
		return ret;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	I("%s: Open file OK!\n", __func__);

	maxItemStrSize = mutual_num * 6 + CONFIG_TABLE_CHAR_REG_NAME_SIZE;
	maxItemStrSize = maxItemStrSize > CONFIG_TABLE_BUF_SIZE ? maxItemStrSize : CONFIG_TABLE_BUF_SIZE;

	maxItemDataSize = mutual_num > CONFIG_TABLE_DATA_BUF_SIZE ? mutual_num : CONFIG_TABLE_DATA_BUF_SIZE;

	result_buff = kzalloc(maxItemStrSize, GFP_KERNEL);

	if (!result_buff) {
		I("%s:Can not allocate result_buff Buffer\n", __func__);
		ret = OPEN_FILE_FAIL;
		goto allocate_result_buff;
	}

	dataBuf = kzalloc((maxItemDataSize * sizeof(uint32_t)), GFP_KERNEL);

	if (!dataBuf) {
		I("%s:Can not allocate dataBuf Buffer\n", __func__);
		ret = OPEN_FILE_FAIL;
		goto allocate_data_buff;
	}

	if (!IS_ERR(fn)) {
		/* fileOffset = CONFIG_TABLE_BUF_SIZE; */
		fileOffset = 0;	/* first read offset */

		if (fn->f_op->llseek == NULL) {

			I("%s: fn->f_op->llseek not support\n", __func__);
			goto llseek_not_support_error;
		}

		do {
			memset(result_buff, 0x00, maxItemStrSize);

			fn->f_op->llseek(fn, fileOffset, SEEK_SET);

			readRet = fn->f_op->read(fn, result_buff, maxItemStrSize, &fn->f_pos);
			if (readRet < 0) {
				I("%s: fn->f_op->read error\n", __func__);
				break;	/* goto file_read_error; */
			} else if (readRet == maxItemStrSize) {
				iNextDo = 1;
			} else {
				iNextDo = 0;	/* reach the file end */
			}

			buff_cursor = result_buff;

			do {
				findStartCharPtr = strnstr(buff_cursor, "{ ",
					maxItemStrSize - ((buff_cursor - result_buff) + 1));	/* find "{ " locarion */
				if (!findStartCharPtr) {
					/* point to the second end of result_buff      "....{" case */
					buff_cursor = result_buff + maxItemStrSize - 2;
					I("%s: invalid config table without {\n", __func__);
					break;
				}

				itemStartOffset = findStartCharPtr - buff_cursor;	/* "{ " offset from load data */
				/* find "{ " locarion */
				findEndCharPtr = strnstr(findStartCharPtr, " }",
					maxItemStrSize - ((buff_cursor - result_buff) + 1) - itemStartOffset);

				if (!findEndCharPtr) {
					if (itemStartOffset) {/* point to the "{ " ahead ont byte addr */
						buff_cursor = findStartCharPtr - 1;
					}
					I("%s: invalid config table without }\n", __func__);
					break;
				}

				itemEndOffset = findEndCharPtr - buff_cursor;	/* " }" offset from load data */

				iDataBufLoadSize =
				    himax_get_comma_data(findStartCharPtr, itemEndOffset - itemStartOffset, dataBuf,
							 maxItemDataSize, &type);

				/* wdd test block start */
				/* buff_cursor[itemEndOffset]='\0'; */
				/* printk("HIMAX [HXTP] wdd:%s\n", buff_cursor);//wdd log */
				if (type == 1) {
					pr_info("HIMAX [HXTP] wdd:0xFE%02x %02x %02x ... %02x %02x\n", dataBuf[0],
					       dataBuf[1], dataBuf[2], dataBuf[iDataBufLoadSize - 1],
					       dataBuf[iDataBufLoadSize]);
					himax_write_config_table(type, dataBuf, iDataBufLoadSize);
				} else if (type > 1) {
					if (iDataBufLoadSize > 1) {
						iDataBufLoadSize -= 1;	/* filter item name */
						himax_get_golden_baseC(type, dataBuf, iDataBufLoadSize);
				pr_info
		("HIMAX [HXTP]wdd:mutal self data%d %d %d ... %d last data=%d iDataBufLoadSize=%d\n",
						     dataBuf[0], dataBuf[1], dataBuf[2], dataBuf[iDataBufLoadSize - 2],
						     dataBuf[iDataBufLoadSize - 1], iDataBufLoadSize);
					} else {
						I("%s: mutal self ref without data\n", __func__);
					}
				} else {
					pr_info("HIMAX [HXTP] wdd:%02x %02x %02x ... %02x %02x\n", dataBuf[0],
					       dataBuf[1], dataBuf[2], dataBuf[iDataBufLoadSize - 1],
					       dataBuf[iDataBufLoadSize]);
					if (dataBuf[0] > 0x20)
						himax_write_config_table(type, dataBuf, iDataBufLoadSize);
					else if (dataBuf[0] == 0x10)
						himax_get_thx_info(dataBuf);
				}
				/* wdd test block end */
				data_check[type] = 1;

				type = 0;	/* clear the type */

				buff_cursor += itemEndOffset + 2;/* curser to next */
			}		while (1);

			fileOffset += buff_cursor - result_buff + 1;	/* load next file buf */

		}	while (iNextDo);
	} else {
		I("%s: Open file fail!\n", __func__);
		ret = OPEN_FILE_FAIL;
	}
	for (i = 0; i < 5; i++) {
		if (data_check[i] == 1)
			chk_cnt++;
	}
	if (chk_cnt != 5)
		ret = LENGTH_FAIL;

llseek_not_support_error:
	kfree(dataBuf);
allocate_data_buff:
	kfree(result_buff);
allocate_result_buff:
	filp_close(fn, NULL);
	return ret;
}

/*****************************************/
/********** Save log in /sdcard **********/
#ifdef HIMAX_ITO_TEST_LOG
void himax_ito_mul_baseC_log_func(void)
{
	/* int result_size = 0; */
	int loop_i = 0;
	int loop_j = 0;
	int len = 0;
	char *result_buff = NULL;
/* char log_type_name[10]; */
	char temp[1000] = {0};
	struct file *fn = NULL;

	result_buff = kzalloc((mutual_num + self_num) * 15 * sizeof(char), GFP_KERNEL);
	I("%s:Entering!\n", __func__);

	memset(temp, '\0', sizeof(temp));
	len += snprintf(temp + len, sizeof(temp), "****************************************************************");
	len += snprintf(temp + len, sizeof(temp), "**************************************************************\n");
	len += snprintf(temp + len, sizeof(temp),
	"#SORTING_MODE~Sorting~BaseCPer~Pass~BaseC~NORMAL~NORMAL~NORMAL~NORMAL~NORMAL\n Max mutual baseC = %d\n",
			mutual_base_c_max);
	snprintf(temp, sizeof(temp), "Max mutual baseC = %d\n", mutual_base_c_max);
	strlcat(result_buff, temp, sizeof(result_buff));

/* memset(temp,'\0',sizeof(temp)); */
/* snprintf(temp, sizeof(temp),"Max baseC = %d\n",  mutual_base_c_max); */
/* strlcat(result_buff,temp, sizeof(result_buff)); */

	for (loop_i = 0; loop_i < x_channel + 1; loop_i++) {
		memset(temp, '\0', sizeof(temp));
		if (loop_i != x_channel)
			snprintf(temp, sizeof(temp), ",RX%d", loop_i + 1);
		else
			snprintf(temp, sizeof(temp), "\n");
		strlcat(result_buff, temp, sizeof(result_buff));
	}

	for (loop_i = 0; loop_i < y_channel; loop_i++) {
		memset(temp, '\0', sizeof(temp));
		snprintf(temp, sizeof(temp), "TX%d,", loop_i + 1);
		strlcat(result_buff, temp, sizeof(result_buff));

		for (loop_j = 0; loop_j < x_channel; loop_j++) {
			if (loop_j != x_channel - 1)
				snprintf(temp, sizeof(temp), "%d,",
					 hx_ito->ito_log_baseC_mul_data[loop_i * x_channel + loop_j]);
			else
				snprintf(temp, sizeof(temp), "%d,\n",
					 hx_ito->ito_log_baseC_mul_data[loop_i * x_channel + loop_j]);
			strlcat(result_buff, temp, sizeof(result_buff));
		}
	}

	fn = filp_open(ITO_LOG_FILE, O_CREAT | O_WRONLY | O_APPEND, 0);
	if (!IS_ERR(fn)) {
		fn->f_op->write(fn, result_buff, (mutual_num + self_num) * 15 * sizeof(char), &fn->f_pos);
		filp_close(fn, NULL);
		I("%s: Store mutual data OK!\n", __func__);
	} else
		I("%s: Store mutual data fail!\n", __func__);
}

void himax_ito_self_baseC_log_func(void)
{
	/* int result_size = 0; */
	int loop_i = 0;
	char *result_buff;
	char temp[1000];
	struct file *fn = NULL;

	result_buff = kzalloc((mutual_num + self_num) * 10 * sizeof(char), GFP_KERNEL);
	I("%s:Entering!\n", __func__);

	memset(temp, '\0', sizeof(temp));
	snprintf(temp, sizeof(temp), "\n");
	strlcat(result_buff, temp, sizeof(result_buff));

	memset(temp, '\0', sizeof(temp));
	snprintf(temp, sizeof(temp), "Max self rx baseC = %d, Max self tx baseC = %d\n", self_rx_base_c_max,
		 self_tx_base_c_max);
	strlcat(result_buff, temp, sizeof(result_buff));

	for (loop_i = 0; loop_i < x_channel + 1; loop_i++) {
		memset(temp, '\0', sizeof(temp));
		if (loop_i != x_channel)
			snprintf(temp, sizeof(temp), ",RX%d", loop_i + 1);
		else
			snprintf(temp, sizeof(temp), "\n");
		strlcat(result_buff, temp, sizeof(result_buff));
	}
	for (loop_i = 0; loop_i < x_channel + 1; loop_i++) {
		memset(temp, '\0', sizeof(temp));
		if (loop_i != x_channel)
			snprintf(temp, sizeof(temp), ",%d", hx_ito->ito_log_baseC_self_data[loop_i]);
		else
			snprintf(temp, sizeof(temp), "\n");
		strlcat(result_buff, temp, sizeof(result_buff));
	}

	for (loop_i = 0; loop_i < y_channel + 1; loop_i++) {
		memset(temp, '\0', sizeof(temp));
		if (loop_i != y_channel)
			snprintf(temp, sizeof(temp), ",TX%d", loop_i + 1);
		else
			snprintf(temp, sizeof(temp), "\n");
		strlcat(result_buff, temp, sizeof(result_buff));
	}
	for (loop_i = 0; loop_i < y_channel + 1; loop_i++) {
		memset(temp, '\0', sizeof(temp));
		if (loop_i != y_channel)
			snprintf(temp, sizeof(temp), ",%d", hx_ito->ito_log_baseC_self_data[x_channel + loop_i]);
		else
			snprintf(temp, sizeof(temp), "\n");
		strlcat(result_buff, temp, sizeof(result_buff));
	}

	memset(temp, '\0', sizeof(temp));
	snprintf(temp, sizeof(temp), "\n**************************************************\n");
	strlcat(result_buff, temp, sizeof(result_buff));

	fn = filp_open(ITO_LOG_FILE, O_CREAT | O_WRONLY | O_APPEND, 0);
	if (!IS_ERR(fn)) {
		fn->f_op->write(fn, result_buff, (mutual_num + self_num) * 10 * sizeof(char), &fn->f_pos);
		filp_close(fn, NULL);
		I("%s: Store self data OK!\n", __func__);
	} else
		I("%s: Store self data fail!\n", __func__);
}

void himax_ito_mul_log_func(void)
{
	/* int result_size = 0; */
	int loop_i = 0;
	int loop_j = 0;
	char *result_buff;
	char log_type_name[10];
	char temp[1000];
	int len = 0;
	struct file *fn = NULL;

	result_buff = kzalloc((mutual_num + self_num) * 15 * sizeof(char), GFP_KERNEL);
	I("%s:Entering!\n", __func__);
	switch (hx_ito->test_log_type) {
	case BANK:
		strlcpy(log_type_name, "BANK", sizeof(log_type_name));
		break;
	case DC:
		strlcpy(log_type_name, "DC", sizeof(log_type_name));
		break;
	case BASEC:
		strlcpy(log_type_name, "BASEC", sizeof(log_type_name));
		break;
	case IIR:
		strlcpy(log_type_name, "IIR", sizeof(log_type_name));
		break;
	default:
		strlcpy(log_type_name, "NULL", sizeof(log_type_name));
	}

	memset(temp, '\0', sizeof(temp));
	len += snprintf(temp + len, sizeof(temp), "**************************************************************");
	len += snprintf(temp + len, sizeof(temp), "************************************************************\n");
	len += snprintf(temp + len, sizeof(temp),
		"#SORTING_MODE~Sorting~BaseCPer~Pass~%s~NORMAL~NORMAL~NORMAL~NORMAL~NORMAL\n",
		log_type_name);

	strlcat(result_buff, temp, sizeof(result_buff));

	if (hx_ito->test_log_type == BASEC) {
		memset(temp, '\0', sizeof(temp));
		snprintf(temp, sizeof(temp), "Max baseC = %d\n", mutual_base_c_max);
		strlcat(result_buff, temp, sizeof(result_buff));
	}

	for (loop_i = 0; loop_i < x_channel + 1; loop_i++) {
		memset(temp, '\0', sizeof(temp));
		if (loop_i != x_channel)
			snprintf(temp, sizeof(temp), ",RX%d", loop_i + 1);
		else
			snprintf(temp, sizeof(temp), "\n");
		strlcat(result_buff, temp, sizeof(result_buff));
	}

	for (loop_i = 0; loop_i < y_channel; loop_i++) {
		memset(temp, '\0', sizeof(temp));
		snprintf(temp, sizeof(temp), "TX%d,", loop_i + 1);
		strlcat(result_buff, temp, sizeof(result_buff));

		for (loop_j = 0; loop_j < x_channel; loop_j++) {
			if (loop_j != x_channel - 1)
				snprintf(temp, sizeof(temp), "%d,",
					 hx_ito->ito_log_mul_data[loop_i * x_channel + loop_j]);
			else
				snprintf(temp, sizeof(temp), "%d,\n",
					 hx_ito->ito_log_mul_data[loop_i * x_channel + loop_j]);
			strlcat(result_buff, temp, sizeof(result_buff));
		}
	}

	fn = filp_open(ITO_LOG_FILE, O_CREAT | O_WRONLY | O_APPEND, 0);
	if (!IS_ERR(fn)) {
		fn->f_op->write(fn, result_buff, (mutual_num + self_num) * 15 * sizeof(char), &fn->f_pos);
		filp_close(fn, NULL);
		I("%s: Store mutual data OK!\n", __func__);
	} else
		I("%s: Store mutual data fail!\n", __func__);
}

void himax_ito_self_log_func(void)
{
	/* int result_size = 0; */
	int loop_i = 0;
	char *result_buff;
	char temp[1000];
	struct file *fn = NULL;

	result_buff = kzalloc((mutual_num + self_num) * 10 * sizeof(char), GFP_KERNEL);
	I("%s:Entering!\n", __func__);

	memset(temp, '\0', sizeof(temp));
	snprintf(temp, sizeof(temp), "\n");
	strlcat(result_buff, temp, sizeof(result_buff));

	for (loop_i = 0; loop_i < x_channel + 1; loop_i++) {
		memset(temp, '\0', sizeof(temp));
		if (loop_i != x_channel)
			snprintf(temp, sizeof(temp), ",RX%d", loop_i + 1);
		else
			snprintf(temp, sizeof(temp), "\n");
		strlcat(result_buff, temp, sizeof(result_buff));
	}
	for (loop_i = 0; loop_i < x_channel + 1; loop_i++) {
		memset(temp, '\0', sizeof(temp));
		if (loop_i != x_channel)
			snprintf(temp, sizeof(temp), ",%d", hx_ito->ito_log_self_data[loop_i]);
		else
			snprintf(temp, sizeof(temp), "\n");
		strlcat(result_buff, temp, sizeof(result_buff));
	}

	for (loop_i = 0; loop_i < y_channel + 1; loop_i++) {
		memset(temp, '\0', sizeof(temp));
		if (loop_i != y_channel)
			snprintf(temp, sizeof(temp), ",TX%d", loop_i + 1);
		else
			snprintf(temp, sizeof(temp), "\n");
		strlcat(result_buff, temp, sizeof(result_buff));
	}
	for (loop_i = 0; loop_i < y_channel + 1; loop_i++) {
		memset(temp, '\0', sizeof(temp));
		if (loop_i != y_channel)
			snprintf(temp, sizeof(temp), ",%d", hx_ito->ito_log_self_data[x_channel + loop_i]);
		else
			snprintf(temp, sizeof(temp), "\n");
		strlcat(result_buff, temp, sizeof(result_buff));
	}

	memset(temp, '\0', sizeof(temp));
	snprintf(temp, sizeof(temp), "\n******************************************************************\n");
	strlcat(result_buff, temp, sizeof(result_buff));

	fn = filp_open(ITO_LOG_FILE, O_CREAT | O_WRONLY | O_APPEND, 0);
	if (!IS_ERR(fn)) {
		fn->f_op->write(fn, result_buff, (mutual_num + self_num) * 10 * sizeof(char), &fn->f_pos);
		filp_close(fn, NULL);
		I("%s: Store self data OK!\n", __func__);
	} else
		I("%s: Store self data fail!\n", __func__);
}
#endif

void himax_ito_fail_log_func(uint8_t test_type, int16_t *data, uint16_t test_result)
{
	int loop_i = 0;
	int loop_j = 0;
	char *result_buff;
	int buf_shift = 0;
	char log_type_name[50];
	char temp[900];
	struct file *fn = NULL;

	result_buff = kzalloc((mutual_num + self_num) * 15 * sizeof(char), GFP_KERNEL);
	I("%s:Entering!\n", __func__);

	switch (test_type) {
	case MUTUAL_PERCENT_LIMIT:
		strlcpy(log_type_name, "MUTUAL_PERCENT_LIMIT", sizeof(log_type_name));
		break;
	case SELF_PERCENT_LIMIT:
		strlcpy(log_type_name, "SELF_PERCENT_LIMIT", sizeof(log_type_name));
		buf_shift = x_channel;
		break;
	case MUTUAL_DEV_TX:
		strlcpy(log_type_name, "MUTUAL_DEV_TX", sizeof(log_type_name));
		break;
	case MUTUAL_DEV_RX:
		strlcpy(log_type_name, "MUTUAL_DEV_RX", sizeof(log_type_name));
		break;
	case SELF_DEV_TX:
		strlcpy(log_type_name, "SELF_DEV_TX", sizeof(log_type_name));
		break;
	case SELF_DEV_RX:
		strlcpy(log_type_name, "SELF_DEV_RX", sizeof(log_type_name));
		break;
	case MUTUAL_IIR_NOISE:
		strlcpy(log_type_name, "MUTUAL_IIR_NOISE", sizeof(log_type_name));
		break;
	case SELF_IIR_NOISE:
		strlcpy(log_type_name, "SELF_IIR_NOISE", sizeof(log_type_name));
		buf_shift = x_channel;
		break;
	default:
		strlcpy(log_type_name, "NULL", sizeof(log_type_name));
	}

	if (test_result == 0) {
		memset(temp, '\0', sizeof(temp));
		snprintf(temp, sizeof(temp), "\n***Test fail item : %s***\n", log_type_name);
		strlcat(result_buff, temp, sizeof(result_buff));
	}

	memset(temp, '\0', sizeof(temp));
	snprintf(temp, sizeof(temp), "\nTest item : %s\n", log_type_name);
	strlcat(result_buff, temp, sizeof(result_buff));

	if (test_type == MUTUAL_PERCENT_LIMIT || test_type == MUTUAL_DEV_TX || test_type == MUTUAL_DEV_RX
	    || test_type == MUTUAL_IIR_NOISE) {
		for (loop_i = 0; loop_i < x_channel + 1; loop_i++) {
			memset(temp, '\0', sizeof(temp));
			if (loop_i != x_channel)
				snprintf(temp, sizeof(temp), ",RX%d", loop_i + 1);
			else
				snprintf(temp, sizeof(temp), "\n");
			strlcat(result_buff, temp, sizeof(result_buff));
		}

		for (loop_i = 0; loop_i < y_channel; loop_i++) {
			memset(temp, '\0', sizeof(temp));
			snprintf(temp, sizeof(temp), "TX%d,", loop_i + 1);
			strlcat(result_buff, temp, sizeof(result_buff));

			for (loop_j = 0; loop_j < x_channel; loop_j++) {
				if (loop_j != x_channel - 1)
					snprintf(temp, sizeof(temp), "%d,", data[loop_i * x_channel + loop_j]);
				else
					snprintf(temp, sizeof(temp), "%d,\n", data[loop_i * x_channel + loop_j]);
				strlcat(result_buff, temp, sizeof(result_buff));
			}
		}
	}

	if (test_type == SELF_DEV_RX || test_type == SELF_PERCENT_LIMIT || test_type == SELF_IIR_NOISE) {
		for (loop_i = 0; loop_i < x_channel + 1; loop_i++) {
			memset(temp, '\0', sizeof(temp));

			if (loop_i == 0)
				snprintf(temp, sizeof(temp), "RX%d", loop_i + 1);
			else if (loop_i != x_channel)
				snprintf(temp, sizeof(temp), ",RX%d", loop_i + 1);
			else
				snprintf(temp, sizeof(temp), "\n");
			strlcat(result_buff, temp, sizeof(result_buff));
		}
		for (loop_i = 0; loop_i < x_channel + 1; loop_i++) {
			memset(temp, '\0', sizeof(temp));
			if (loop_i == 0)
				snprintf(temp, sizeof(temp), "%d", data[loop_i]);
			else if (loop_i != x_channel)
				snprintf(temp, sizeof(temp), ",%d", data[loop_i]);
			else
				snprintf(temp, sizeof(temp), "\n");
			strlcat(result_buff, temp, sizeof(result_buff));
		}
	}

	if (test_type == SELF_DEV_TX || test_type == SELF_PERCENT_LIMIT || test_type == SELF_IIR_NOISE) {
		for (loop_i = 0; loop_i < y_channel + 1; loop_i++) {
			memset(temp, '\0', sizeof(temp));
			if (loop_i == 0)
				snprintf(temp, sizeof(temp), "TX%d", loop_i + 1);
			else if (loop_i != y_channel)
				snprintf(temp, sizeof(temp), ",TX%d", loop_i + 1);
			else
				snprintf(temp, sizeof(temp), "\n");
			strlcat(result_buff, temp, sizeof(result_buff));
		}
		for (loop_i = 0; loop_i < y_channel + 1; loop_i++) {
			memset(temp, '\0', sizeof(temp));
			if (loop_i == 0)
				snprintf(temp, sizeof(temp), "%d", data[loop_i + buf_shift]);
			else if (loop_i != y_channel)
				snprintf(temp, sizeof(temp), ",%d", data[loop_i + buf_shift]);
			else
				snprintf(temp, sizeof(temp), "\n");
			strlcat(result_buff, temp, sizeof(result_buff));
		}
	}

	fn = filp_open(ITO_FAIL_LOG_FILE, O_CREAT | O_WRONLY | O_APPEND, 0);
	if (!IS_ERR(fn)) {
		fn->f_op->write(fn, result_buff, (mutual_num + self_num) * 15 * sizeof(char), &fn->f_pos);
		filp_close(fn, NULL);
		I("%s: Store fail data OK!\n", __func__);
	} else
		I("%s: Store fail data fail!\n", __func__);
}

/****************************************/
static uint16_t himax_noise_test(void)
{
	int i = 0;
	uint32_t test_result = 0;

	ito_set_step_status(NOISE_TEST);
	for (i = 0; i < mutual_num; i++) {
		if (hx_ito->mutual_iir_data[i] < noice_thx) {
			hx_ito->noise_test_result[i] = 0;
		} else {
			hx_ito->noise_test_result[i] = hx_ito->mutual_iir_data[i];
			test_result++;
		}
	}

	for (i = 0; i < self_num; i++) {
		if (hx_ito->self_iir_data[i] < noice_thx) {
			hx_ito->noise_test_result[mutual_num + i] = hx_ito->self_iir_data[i];
		} else {
			hx_ito->noise_test_result[mutual_num + i] = hx_ito->self_iir_data[i];
			test_result++;
		}
	}

	if (test_result == 0) {
		I("%s Pass!\n", __func__);
		himax_ito_fail_log_func(MUTUAL_IIR_NOISE, hx_ito->noise_test_result, 1);
		himax_ito_fail_log_func(SELF_IIR_NOISE, &hx_ito->noise_test_result[mutual_num], 1);
	} else {
		I("%s Fail! test_result = %d\n", __func__, test_result);
		himax_ito_fail_log_func(MUTUAL_IIR_NOISE, hx_ito->noise_test_result, 0);
		himax_ito_fail_log_func(SELF_IIR_NOISE, &hx_ito->noise_test_result[mutual_num], 0);
		ito_set_result_status(TEST_FAIL);
	}
	return test_result;
}

static uint16_t self_dev_rx_check(void)	/* (Self_RX_DeviationCheckItem) */
{
	int i = 0;
	int16_t *self_rx_dev;
	uint32_t test_result = 0;
	int32_t res = 0;

	self_rx_dev = kzalloc((x_channel * sizeof(int16_t)), GFP_KERNEL);

	for (i = 0; i < x_channel - 1; i++) {
		if (golden_rx_self_baseC[i] == 0) {
			self_rx_dev[i] = 0;
		} else {
			res =
			    ((int32_t)
			     ((hx_ito->self_baseC[i] - hx_ito->self_baseC[i + 1]) -
			      (golden_rx_self_baseC[i] -
			       golden_rx_self_baseC[i + 1])) * 100 / (int32_t) golden_rx_self_baseC[i]);
			self_rx_dev[i] = (int16_t) res;
		}
		if (self_rx_dev[i] <= self_dev_fail_upper && self_rx_dev[i] >= self_dev_fail_lower) {
			hx_ito->self_rx_dev_result[i] = self_rx_dev[i];
		} else {
			hx_ito->self_rx_dev_result[i] = self_rx_dev[i];
			test_result++;
		}
#ifdef ITO_DEBUG_DIS
		I("%s res[%d]= %5d\n", __func__, i, res);
		I("self_rx_dev[%d]= %5d\n", i, self_rx_dev[i]);
		I("self_rx_dev_result[%d] = %5d\n", i, hx_ito->self_rx_dev_result[i]);
#endif
	}

	if (test_result == 0) {
		I("%s Pass!\n", __func__);
		himax_ito_fail_log_func(SELF_DEV_RX, hx_ito->self_rx_dev_result, 1);
	} else {
		I("%s Fail! test_result = %d\n", __func__, test_result);
		himax_ito_fail_log_func(SELF_DEV_RX, hx_ito->self_rx_dev_result, 0);
		ito_set_result_status(TEST_FAIL);
	}

	kfree(self_rx_dev);
	return test_result;
}

static uint16_t self_dev_tx_check(void)	/* (Self_TX_DeviationCheckItem) */
{
	int i = 0;
	int32_t res = 0;
	int16_t *self_tx_dev;
	uint8_t y_channel_test = y_channel;
	uint32_t test_result = 0;

	self_tx_dev = kzalloc((y_channel * sizeof(int16_t)), GFP_KERNEL);

	if (hx_ito->mkey_num > 0)
		y_channel_test = y_channel - 1;
	for (i = 0; i < y_channel_test - 1; i++) {
		res = golden_tx_self_baseC[i];
		if (res == 0) {
			self_tx_dev[i] = 0;
		} else {
			res =
			    ((int32_t)
			     ((hx_ito->self_baseC[x_channel + i] - hx_ito->self_baseC[x_channel + i + 1]) -
			      (golden_tx_self_baseC[i] -
			       golden_tx_self_baseC[i + 1])) * 100 / (int32_t) golden_tx_self_baseC[i]);
			self_tx_dev[i] = (int16_t) res;
		}

		if (self_tx_dev[i] <= self_dev_fail_upper && self_tx_dev[i] >= self_dev_fail_lower) {
			hx_ito->self_tx_dev_result[i] = self_tx_dev[i];
		} else {
			hx_ito->self_tx_dev_result[i] = self_tx_dev[i];
			test_result++;
		}
#ifdef ITO_DEBUG_DIS
		I("%s res[%d]= %5d\n", __func__, i, res);
		I("self_tx_dev[%d]= %5d\n", i, self_tx_dev[i]);
		I("self_tx_dev_result[%d] = %5d\n", i, hx_ito->self_tx_dev_result[i]);
#endif
	}

	if (test_result == 0) {
		I("%s Pass!\n", __func__);
		himax_ito_fail_log_func(SELF_DEV_TX, hx_ito->self_tx_dev_result, 1);
	} else {
		I("%s Fail! test_result = %d\n", __func__, test_result);
		himax_ito_fail_log_func(SELF_DEV_TX, hx_ito->self_tx_dev_result, 0);
		ito_set_result_status(TEST_FAIL);
	}

	kfree(self_tx_dev);
	return test_result;
}

static uint16_t mutual_dev_rx_check(void)	/* (MutualDeviationRxSideCheck) */
{
	int i = 0;
	int j = 0;
	int32_t res = 0;
	uint32_t rx_channel_sum = 0;
	uint32_t rx_channel_sum_ref = 0;
	uint32_t *rx_channel_sum_arr;
	uint32_t *rx_channel_sum_ref_arr;
	uint16_t baseC_avg_except_pre = 0;
	uint16_t baseC_avg_except_pre_ref = 0;
	int16_t *mutual_dev;
	uint8_t y_channel_test = y_channel;
	uint32_t test_result = 0;

	rx_channel_sum_arr = kzalloc((x_channel * sizeof(uint32_t)), GFP_KERNEL);
	rx_channel_sum_ref_arr = kzalloc((x_channel * sizeof(uint32_t)), GFP_KERNEL);
	mutual_dev = kzalloc((mutual_num * sizeof(int16_t)), GFP_KERNEL);

	memset(mutual_dev, 0x00, sizeof(int16_t));
	memset(hx_ito->mutual_rx_dev_result, 0x00, sizeof(int16_t));

	if (hx_ito->mkey_num > 0)
		y_channel_test = y_channel - 1;

	for (i = 0; i < x_channel; i++) {/* Calculate aversge of RX per line */

		rx_channel_sum = 0;
		rx_channel_sum_ref = 0;
		for (j = 0; j < y_channel_test; j++) {
			rx_channel_sum += hx_ito->mutual_baseC[i + j * x_channel];
			rx_channel_sum_ref += golden_mutual_baseC[i + j * x_channel];
		}
		rx_channel_sum_arr[i] = rx_channel_sum;
		rx_channel_sum_ref_arr[i] = rx_channel_sum_ref;
	}

	for (i = 0; i < x_channel; i++) {
		for (j = 0; j < y_channel; j++)	{/* Calculate deviation of all block */

			if (hx_ito->mkey_num > 0 && j == y_channel - 1) {
				continue;
			} else {
				if (golden_mutual_baseC[i + j * x_channel] == 0)
					golden_mutual_baseC[i + j * x_channel] = 1;
				baseC_avg_except_pre =
				    (rx_channel_sum_arr[i] -
				     hx_ito->mutual_baseC[i + j * x_channel]) / (y_channel_test - 1);
				baseC_avg_except_pre_ref =
				    (rx_channel_sum_ref_arr[i] -
				     golden_mutual_baseC[i + j * x_channel]) / (y_channel_test - 1);

				res =
				    (int32_t) ((hx_ito->mutual_baseC[i + j * x_channel] - baseC_avg_except_pre) -
					       (golden_mutual_baseC[i + j * x_channel] -
						baseC_avg_except_pre_ref)) * 100 / (int32_t) golden_mutual_baseC[i +
						 j * x_channel];
				mutual_dev[i + j * x_channel] = (int16_t) res;

				if (mutual_dev[i + j * x_channel] < mutual_dev_fail_upper
				    && mutual_dev[i + j * x_channel] > mutual_dev_fail_lower)	/* Pass */
					hx_ito->mutual_rx_dev_result[i + j * x_channel] = mutual_dev[i + j * x_channel];
				else {	/* Fail */
					hx_ito->mutual_rx_dev_result[i + j * x_channel] = mutual_dev[i + j * x_channel];
					test_result++;
				}

#ifdef ITO_DEBUG_DIS
				I("rx baseC_avg_except_pre = %5d, baseC_avg_except_pre_ref = %5d\n",
				  baseC_avg_except_pre, baseC_avg_except_pre_ref);
				I("rx PER = %5d\n",
				  (hx_ito->mutual_baseC[i + j * x_channel] - baseC_avg_except_pre) -
				  (golden_mutual_baseC[i + j * x_channel] - baseC_avg_except_pre_ref));
				I("rx res = %5d\n", res);
				I("mutual_dev[%d][%d] = %5d\n", i, j, mutual_dev[i + j * x_channel]);
				I("mutual_rx_dev_result[%d][%d] = %5d\n", i, j,
				  hx_ito->mutual_rx_dev_result[i + j * x_channel]);
#endif
			}
		}
	}

	if (test_result == 0) {
		I("%s Pass!\n", __func__);
		himax_ito_fail_log_func(MUTUAL_DEV_RX, hx_ito->mutual_rx_dev_result, 1);
	} else {
		I("%s Fail! test_result = %d\n", __func__, test_result);
		himax_ito_fail_log_func(MUTUAL_DEV_RX, hx_ito->mutual_rx_dev_result, 0);
		ito_set_result_status(TEST_FAIL);
	}

	kfree(rx_channel_sum_arr);
	kfree(rx_channel_sum_ref_arr);
	kfree(mutual_dev);

	return test_result;
}

static uint16_t mutual_dev_tx_check(void)	/* (MutualDeviationTxSideCheck) */
{
	int i = 0;
	int j = 0;
	int32_t res = 0;
	uint32_t tx_channel_sum = 0;
	uint32_t tx_channel_sum_ref = 0;
	uint32_t *tx_channel_sum_arr;
	uint32_t *tx_channel_sum_ref_arr;
	uint16_t baseC_avg_except_pre = 0;
	uint16_t baseC_avg_except_pre_ref = 0;
	int16_t *mutual_dev;
	uint8_t location = 0;
	uint32_t test_result = 0;

	ito_set_step_status(DEV_TEST);

	tx_channel_sum_arr = kzalloc((y_channel * sizeof(uint32_t)), GFP_KERNEL);
	tx_channel_sum_ref_arr = kzalloc((y_channel * sizeof(uint32_t)), GFP_KERNEL);
	mutual_dev = kzalloc((mutual_num * sizeof(int16_t)), GFP_KERNEL);

	memset(mutual_dev, 0x00, sizeof(int16_t));
	memset(hx_ito->mutual_tx_dev_result, 0x00, sizeof(int16_t));

	for (i = 0; i < y_channel; i++)	{/* Calculate aversge of TX per line */

		tx_channel_sum = 0;
		tx_channel_sum_ref = 0;
		if ((hx_ito->mkey_num > 0) && (i == y_channel - 1)) {
			continue;
		} else {
			for (j = 0; j < x_channel; j++) {
				tx_channel_sum += hx_ito->mutual_baseC[i * x_channel + j];
				tx_channel_sum_ref += golden_mutual_baseC[i * x_channel + j];
			}
			tx_channel_sum_arr[i] = tx_channel_sum;
			tx_channel_sum_ref_arr[i] = tx_channel_sum_ref;
			/* I("tx_channel_sum_arr[i] = %5d\n", tx_channel_sum_arr[i]); */
			/* I("tx_channel_sum_ref_arr[i] = %5d\n", tx_channel_sum_ref_arr[i]); */
		}
	}

	for (i = 0; i < y_channel; i++)	{/* Calculate aversge of TX per line */

		if ((hx_ito->mkey_num > 0) && (i == y_channel - 1)) {
			for (j = 0; j < hx_ito->mkey_num; j++) {	/* Calculate m_key in last tx */

				location = hx_ito->m_key_location[j];
				if (golden_mutual_baseC[i * x_channel + location] == 0)
					golden_mutual_baseC[i * x_channel + location] = 1;
				res =
				    (int32_t) (hx_ito->mutual_baseC[i * x_channel + location] -
					   golden_mutual_baseC[i * x_channel + location]) * 100 /
					    (int32_t) golden_mutual_baseC[i * x_channel + location];
				mutual_dev[i * x_channel + location] = (int16_t) res;

				if (mutual_dev[i * x_channel + location] < mutual_dev_fail_upper
				    && mutual_dev[i * x_channel + location] > mutual_dev_fail_lower) {
					/* Pass */
					hx_ito->mutual_tx_dev_result[i * x_channel + location] =
						mutual_dev[i * x_channel + location];
				} else {
					/* Fail */
					hx_ito->mutual_tx_dev_result[i * x_channel + location] =
						mutual_dev[i * x_channel + location];
					test_result++;
				}
#ifdef ITO_DEBUG_DIS
				I("mutual_dev[%d][%d] = %5d\n", j, i, mutual_dev[i * x_channel + location]);
				I("mutual_tx_dev_result[%d][%d] = %5d\n", j, i,
				  hx_ito->mutual_tx_dev_result[i * x_channel + location]);
#endif
			}
		} else {
			for (j = 0; j < x_channel; j++)	 {/* Calculate deviation of all block */

				if (golden_mutual_baseC[i * x_channel + j] == 0)
					golden_mutual_baseC[i * x_channel + j] = 1;
				baseC_avg_except_pre =
				    (tx_channel_sum_arr[i] - hx_ito->mutual_baseC[i * x_channel + j]) / (x_channel - 1);
				baseC_avg_except_pre_ref =
				    (tx_channel_sum_ref_arr[i] - golden_mutual_baseC[i * x_channel + j]) / (x_channel -
													    1);

				res =
				    (int32_t) ((hx_ito->mutual_baseC[i * x_channel + j] - baseC_avg_except_pre) -
					       (golden_mutual_baseC[i * x_channel + j] -
						baseC_avg_except_pre_ref)) * 100 / (int32_t) golden_mutual_baseC[i *
							 x_channel + j];

				mutual_dev[i * x_channel + j] = (int16_t) res;

				if (mutual_dev[i * x_channel + j] < mutual_dev_fail_upper
				    && mutual_dev[i * x_channel + j] > mutual_dev_fail_lower) {
					/* Pass */
					hx_ito->mutual_tx_dev_result[i * x_channel + j] = mutual_dev[i * x_channel + j];
				} else {
					/* Fail */
					hx_ito->mutual_tx_dev_result[i * x_channel + j] = mutual_dev[i * x_channel + j];
					test_result++;
				}
#ifdef ITO_DEBUG_DIS
				I("baseC_avg_except_pre = %5d, baseC_avg_except_pre_ref = %5d\n", baseC_avg_except_pre,
				  baseC_avg_except_pre_ref);
				I("PER = %5d\n",
				  (hx_ito->mutual_baseC[i * x_channel + j] - baseC_avg_except_pre) -
				  (golden_mutual_baseC[i * x_channel + j] - baseC_avg_except_pre_ref));
				I("res = %5d\n", res);
				I("mutual_dev[%d][%d] = %5d\n", j, i, mutual_dev[i * x_channel + j]);
				I("mutual_tx_dev_result[%d][%d] = %5d\n", j, i,
				  hx_ito->mutual_tx_dev_result[i * x_channel + j]);
#endif
			}
		}
	}

	if (test_result == 0) {
		I("%s Pass!\n", __func__);
		himax_ito_fail_log_func(MUTUAL_DEV_TX, hx_ito->mutual_tx_dev_result, 1);
	} else {
		I("%s Fail! test_result = %d\n", __func__, test_result);
		himax_ito_fail_log_func(MUTUAL_DEV_TX, hx_ito->mutual_tx_dev_result, 0);
		ito_set_result_status(TEST_FAIL);
	}

	kfree(tx_channel_sum_arr);
	kfree(tx_channel_sum_ref_arr);
	kfree(mutual_dev);

	return test_result;
}

static uint16_t baseC_self_percent_limit_check(void)	/* (BaseC_PerLimitCheck)Check BaseC per block */
{
	int i = 0;
	uint32_t test_result = 0;
	uint32_t res = 0;
	uint32_t self_base_c_max = 0;

	memset(hx_ito->self_baseC_percent_result, 0x00, sizeof(int16_t));

	for (i = 0; i < x_channel + y_channel; i++) {	/* Self test */

		if (i < x_channel)
			self_base_c_max = self_rx_base_c_max;
		else
			self_base_c_max = self_tx_base_c_max;

		res = hx_ito->self_baseC[i] * 100 / self_base_c_max;
		hx_ito->self_baseC_percent_result[i] = res;
		if (hx_ito->self_baseC[i] * 10 < (min_self_baseC_percent * self_base_c_max / 10)
		    || hx_ito->self_baseC[i] * 10 > (max_self_baseC_percent * self_base_c_max / 10)) {
			test_result++;
			/* I("self_percent[%d] Fail\n", i); */
		}
#ifdef ITO_DEBUG_EN
		I("self_baseC[%d] = %5d  mutual_base_c_max = %5d\n", i, hx_ito->self_baseC[i], mutual_base_c_max);
		I("self_baseC_percent_result[%d] = %5d\n", i, hx_ito->self_baseC_percent_result[i]);
		if (i == x_channel)
			I("\n");
#endif
	}

	if (test_result == 0) {
		I("%s Pass!\n", __func__);
		himax_ito_fail_log_func(SELF_PERCENT_LIMIT, hx_ito->self_baseC_percent_result, 1);
	} else {
		I("%s Fail! test_result = %d\n", __func__, test_result);
		himax_ito_fail_log_func(SELF_PERCENT_LIMIT, hx_ito->self_baseC_percent_result, 0);
		ito_set_result_status(TEST_FAIL);
	}

	return test_result;
}

static uint16_t baseC_mutual_percent_limit_check(void)/* (BaseC_PerLimitCheck)Check BaseC per block */
{
	int i = 0;
	int j = 0;
	uint32_t test_result = 0;
	uint32_t res = 0;

	ito_set_step_status(PERCENT_TEST);
	memset(hx_ito->mutual_baseC_percent_result, 0x00, sizeof(int16_t));

	for (i = 0; i < x_channel; i++) {/* Mutual test */

		for (j = 0; j < y_channel; j++) {/* fail result : percent data */
			res = hx_ito->mutual_baseC[i + j * x_channel] * 100 / mutual_base_c_max;
			hx_ito->mutual_baseC_percent_result[i + j * x_channel] = res;

			if (hx_ito->mutual_baseC[i + j * x_channel] * 10 <
			    (min_mutual_baseC_percent * mutual_base_c_max / 10)
			    || hx_ito->mutual_baseC[i + j * x_channel] * 10 >
			    (max_mutual_baseC_percent * mutual_base_c_max / 10)) {
				test_result++;
				/* I("mutual_percent[%d][%d] fail\n", i, j); */
			} else {
				/* hx_ito->mutual_baseC_percent_result[i + j * x_channel] = 0; //Pass */
				/* I("mutual_percent[%d][%d] pass\n", i, j); */
			}
#ifdef ITO_DEBUG_DIS
			if (i < mutual_num) {
				I("mutual_baseC[%d][%d] = %5d  mutual_base_c_max = %5d\n", i, j,
				  hx_ito->mutual_baseC[i + j * x_channel], mutual_base_c_max);
				I("mutual_baseC_percent_result[%d][%d] = %5d\n", i, j,
				  hx_ito->mutual_baseC_percent_result[i + j * x_channel]);
			}
			if (i % 8 == 7)
				I("\n");
#endif
		}
	}

	if (test_result == 0) {
		I("%s Pass!\n", __func__);
		himax_ito_fail_log_func(MUTUAL_PERCENT_LIMIT, hx_ito->mutual_baseC_percent_result, 1);
	} else {
		I("%s Fail! test_result = %d\n", __func__, test_result);
		himax_ito_fail_log_func(MUTUAL_PERCENT_LIMIT, hx_ito->mutual_baseC_percent_result, 0);
		ito_set_result_status(TEST_FAIL);
	}

	return test_result;
}

static void cal_mutual_parameter(void)	/* (CalculateMutParameter) */
{
	/* -Mul_8_16 */
	if (MUL_8_16_Enable_Mutual == 1) {
		Mut_MUL_8_16 = 0;
	} else {
		Mut_MUL_8_16 = 1;
	}

	/* -TX Voltage */
	if (VR9_V > VR5_M_V) {
		TX_Voltage_P = VR5_M_V * 3;
	} else {
		TX_Voltage_P = VR9_V * 3;
	}

	TX_Voltage_N = VR8_V;

	/* Muutual Bank Cut,31 doesn't gave bank cut */
	Bank_Cut = 0;

	/* -Charge Discharge */
	Charge_Discharge = 8 * (Mut_MUL_8_16 + 1) * (ChrageDischarge_Mutual + 1);

	/* -P Pulse */
	if (P_PulseMask > P_PulseMutual) {
		Temp_P_PulseMask = P_PulseMask;
	} else {
		Temp_P_PulseMask = P_PulseMutual;
	}

	/* -Down Scale */
	Down_Scale = ((Charge_Discharge - Temp_P_PulseMask) * OSR_Mutual + 256 - 1) / 256;

	/* ceil((Charge_Discharge - Temp_P_PulseMask) * OSR_Mutual / 256); */

	/* -Base Cap */
	if (VR7_Enable_Mutual == 0) {
		if (Mutual_TwoSetVR && Is2T3R) {
			Mut_Sense_Cap_PerBank =
			    (VR1_M_V - VR3_M_V +
			     VR7_V * VR7_Enable_Mutual) * Bank_Cap_pf * MULITPLE_SCALE / (TX_Voltage_P - TX_Voltage_N);
		} else {
			Mut_Sense_Cap_PerBank =
			    (VR1_M_V - VR3_V +
			     VR7_V * VR7_Enable_Mutual) * Bank_Cap_pf * MULITPLE_SCALE / (TX_Voltage_P - TX_Voltage_N);
		}
	} else {
		if (Mutual_TwoSetVR && Is2T3R) {
			Mut_Sense_Cap_PerBank = (VR1_M_V - VR3_M_V + VR7_V * VR7_Enable_Mutual) *
				Bank_Cap_pf * 70 / (TX_Voltage_P - TX_Voltage_N);	/* 0.7 */
		} else {
			Mut_Sense_Cap_PerBank = (VR1_M_V - VR3_V + VR7_V * VR7_Enable_Mutual) *
				Bank_Cap_pf * 70 / (TX_Voltage_P - TX_Voltage_N);	/* 0.7 */
		}
	}
	if (Mutual_TwoSetVR && Is2T3R) {
		Mut_Sense_Cap_PerDAC = (VR4_M_V - VR3_M_V) * DAC_Ratio * DAC_Cap_pf / (TX_Voltage_P - TX_Voltage_N);
	} else {
		Mut_Sense_Cap_PerDAC = (VR4_M_V - VR3_V) * DAC_Ratio * DAC_Cap_pf / (TX_Voltage_P - TX_Voltage_N);
	}
	/* original scaling * 10000 => * 10000 / 10 = * 1000 */
	mutual_base_c_max = (255 * Mut_Sense_Cap_PerBank + Mutual_BaseLine * Mut_Sense_Cap_PerDAC * Down_Scale
			/ OSR_Mutual) / 10;	/* MPAP : mutual_bank_cap */
	I("%s : mutual_base_c_max = %d\n", __func__, mutual_base_c_max);
#ifdef MutPar_DEBUG
	I("%s : TX_Voltage_P = %d\n", __func__, TX_Voltage_P);
	I("%s : Charge_Discharge = %d\n", __func__, Charge_Discharge);
	I("%s : Down_Scale = %d\n", __func__, Down_Scale);
	I("%s : TX_Voltage_N = %d\n", __func__, TX_Voltage_N);
	I("%s : Mut_Sense_Cap_PerBank = %d\n", __func__, Mut_Sense_Cap_PerBank);
	I("%s : Mut_Sense_Cap_PerDAC = %d\n", __func__, Mut_Sense_Cap_PerDAC);

#endif
}

static void cal_self_rx_parameter(void)	/* CalculateSelfRXParameter */
{
	/* -Mul_8_16 */
	if (MUL_8_16_Enable_Mutual == 1) {
		SlfRX_MUL_8_16 = 0;
	} else {
		SlfRX_MUL_8_16 = 1;
	}

	/* -Charge Discharge */
	Charge_Discharge = 8 * (SlfRX_MUL_8_16 + 1) * (ChrageDischarge_SelfRX + 1);

	/* -P Pulse */
	if (P_PulseMask > P_PulseSelf) {
		Temp_P_PulseMask = P_PulseMask;
	} else {
		Temp_P_PulseMask = P_PulseSelf;
	}

	/* -Down Scale */
	Down_Scale = ((Charge_Discharge - Temp_P_PulseMask) * OSR_SelfRX + 256 - 1) / 256;

	/* -Base Cap */
	if (VR7_Enable_Self == 0) {
		SlfRX_Sense_Cap_PerBank =
		    (VR1_S_V - VR3_V + VR7_V * VR7_Enable_Self) * Bank_Cap_pf * 100 / (VR3_V - VR2_S_V);
	} else {
		SlfRX_Sense_Cap_PerBank = (VR1_S_V - VR3_V + VR7_V * VR7_Enable_Self) * Bank_Cap_pf * 70
			/ (VR3_V - VR2_S_V);	/* 0.7 */
	}
	SlfRX_Sense_Cap_PerDAC = (VR4_S_V - VR3_V) * DAC_Ratio * DAC_Cap_pf / (VR3_V - VR2_S_V);

	self_rx_base_c_max =
	    (SlfRX_Sense_Cap_PerBank * 255 + Self_BaseLineUp * SlfRX_Sense_Cap_PerDAC * Down_Scale / OSR_SelfRX) / 10;
	I("%s : self_rx_base_c_max = %d\n", __func__, self_rx_base_c_max);
#ifdef MutPar_DEBUG
	I("%s : Charge_Discharge = %d\n", __func__, Charge_Discharge);
	I("%s : Down_Scale = %d\n", __func__, Down_Scale);
	I("%s : SlfRX_Sense_Cap_PerBank = %d\n", __func__, SlfRX_Sense_Cap_PerBank);
	I("%s : SlfRX_Sense_Cap_PerDAC = %d\n", __func__, SlfRX_Sense_Cap_PerDAC);
#endif
}

static void cal_self_tx_parameter(void)	/* CalculateSelfTXParameter */
{
	/* -Mul_8_16 */
	if (MUL_8_16_Enable_Mutual == 1) {
		SlfTX_MUL_8_16 = 0;
	} else {
		SlfTX_MUL_8_16 = 1;
	}

	/* -Charge Discharge */
	Charge_Discharge = 8 * (SlfTX_MUL_8_16 + 1) * (ChrageDischarge_SelfTX + 1);

	/* -P Pulse */
	if (P_PulseMask > P_PulseSelf) {
		Temp_P_PulseMask = P_PulseMask;
	} else {
		Temp_P_PulseMask = P_PulseSelf;
	}

	/* -Down Scale */
	Down_Scale = ((Charge_Discharge - Temp_P_PulseMask) * OSR_SelfTX + 256 - 1) / 256;

	/* -Base Cap */
	if (VR7_Enable_Self == 0) {
		SlfTX_Sense_Cap_PerBank =
		    (VR1_S_V - VR3_V + VR7_V * VR7_Enable_Self) * Bank_Cap_pf * 100 / (VR3_V - VR2_S_V);
	} else {
		SlfTX_Sense_Cap_PerBank =
		    (VR1_S_V - VR3_V + VR7_V * VR7_Enable_Self) * Bank_Cap_pf * 70 / (VR3_V - VR2_S_V);
	}
	SlfTX_Sense_Cap_PerDAC = (VR4_S_V - VR3_V) * DAC_Ratio * DAC_Cap_pf / (VR3_V - VR2_S_V);

	self_tx_base_c_max =
	    (SlfTX_Sense_Cap_PerBank * 255 + Self_BaseLineUp * SlfTX_Sense_Cap_PerDAC * Down_Scale / OSR_SelfTX) / 10;
	I("%s : self_tx_base_c_max = %d\n", __func__, self_tx_base_c_max);
#ifdef MutPar_DEBUG
	I("%s : Charge_Discharge = %d\n", __func__, Charge_Discharge);
	I("%s : Down_Scale = %d\n", __func__, Down_Scale);
	I("%s : SlfTX_Sense_Cap_PerBank = %d\n", __func__, SlfTX_Sense_Cap_PerBank);
	I("%s : SlfTX_Sense_Cap_PerDAC = %d\n", __func__, SlfTX_Sense_Cap_PerDAC);
#endif
}

static void cal_vr_parameter(bool is_mutual, int tx_num, int rx_num)	/* (CalculateVRParameter) */
{
	uint8_t temp_data[4] = { 0x00 };
	uint8_t temp_cmd[4] = { 0x00 };
	uint8_t CycleNumber = 0;

	if (Self_TwoSetVR) {
		if (!is_mutual) {	/* Self Channel */

			if (rx_num != 0) {/* RX Channel */

				VR1_S_V = VRH_V * TwoSet_VR1_S_RX / 32;
				VR3_V = VRH_V * TwoSet_VR3_S_RX / 32;
				VR4_S_V = VRH_V * TwoSet_VR4_S_RX / 32;
				VR7_V = VR1_S_V;
			} else if (tx_num != 0) {/* TX Channel */

				VR1_S_V = VRH_V * TwoSet_VR1_S_TX / 32;
				VR3_V = VRH_V * TwoSet_VR3_S_TX / 32;
				VR4_S_V = VRH_V * TwoSet_VR4_S_TX / 32;
				VR7_V = VR1_S_V;
			} else {
				VR1_S_V = VRH_V * VR1_S / 32;
				VR3_V = VRH_V * VR3 / 32;
				VR4_S_V = VRH_V * VR4_S / 32;
				VR7_V = VR1_S_V;
			}
		} else {
			VR1_S_V = VRH_V * VR1_S / 32;
			VR3_V = VRH_V * VR3 / 32;
			VR4_S_V = VRH_V * VR4_S / 32;
			VR7_V = VR1_S_V;
		}
	} else {
		VR1_S_V = VRH_V * VR1_S / 32;
		VR3_V = VRH_V * VR3 / 32;
		VR4_S_V = VRH_V * VR4_S / 32;
		VR7_V = VRH_V * VR7 / 32;
	}

	VR5_M_V = VRH_V * VR5_M / 32;
	VR6_V = VRH_V * VR6 / 32;
	VR8_V = VRH_V * VR8 / 32;
	VR9_V = VRH_V * VR9 / 32;

	if (Mutual_TwoSetVR) {
		if (is_mutual) {/* Mutual Channel */

			/* CycleNumber = MappingObj.GetCycleNumber(rx_num); //Get Cycle number : 1~3 */
			temp_cmd[0] = 0x73;
			himax_register_read(private_ts->client, temp_cmd, 1, temp_data, true);
			I("%s Cycle number = %X", __func__, temp_data[0]);
			/* MappingObj.GetCycleNumber(rx_num); //Get Cycle number : 1~3 //FE(73) High byte */
			CycleNumber = (temp_data[0] & 0xF0) >> 4;
			I("%s Calculate cycle Number = %X", __func__, CycleNumber);

			if (TwoSet_MutualMap[CycleNumber - 1]) {/* 1 : 2nd VR         //VRSelRev[] == false */

				VR1_M_V = VRH_V * TwoSet_VR1_M_2 / 32;
				VR3_M_V = VRH_V * TwoSet_VR3_M_2 / 32;
				VR4_M_V = VRH_V * TwoSet_VR4_M_2 / 32;
			} else {	/* 0 : 1st VR */

				VR1_M_V = VRH_V * TwoSet_VR1_M_1 / 32;
				VR3_M_V = VRH_V * TwoSet_VR3_M_1 / 32;
				VR4_M_V = VRH_V * TwoSet_VR4_M_1 / 32;
			}
		} else {
			VR1_M_V = VRH_V * VR1_M / 32;
			VR3_M_V = VRH_V * VR3_M / 32;
			VR4_M_V = VRH_V * VR4_M / 32;
		}
	} else {
		VR1_M_V = VRH_V * VR1_M / 32;
		VR3_M_V = VRH_V * VR3_M / 32;
		VR4_M_V = VRH_V * VR4_M / 32;
	}
#ifdef VR_DEBUG
	I("%s : VR1_S_V = %d, VR3_V = %d, VR4_S_V = %d, VR7_V = %d,\n", __func__, VR1_S_V, VR3_V, VR4_S_V, VR7_V);
	I("%s : VR5_M_V = %d, VR6_V = %d, VR8_V = %d, VR9_V = %d,\n", __func__, VR5_M_V, VR6_V, VR8_V, VR9_V);
	I("%s : VR1_M_V = %d, VR3_M_V = %d, VR4_M_V = %d\n", __func__, VR1_M_V, VR3_M_V, VR4_M_V);
#endif
}

static void get_self_baseC(void)	/* (GetSelfRxBaseC + GetSelfTxBaseC) */
{
	uint32_t temp_double = 0;
	int i = 0;

	for (i = 0; i < x_channel + y_channel; i++) {
		if (i < x_channel) {	/* RX */

			cal_vr_parameter(false, 0, i + 1);
			cal_self_rx_parameter();

			/* original scaling 10000 */
			temp_double =
			    hx_ito->self_bank_data[i] * SlfRX_Sense_Cap_PerBank +
			    hx_ito->self_dc_data[i] * Down_Scale * SlfRX_Sense_Cap_PerDAC / OSR_SelfRX;
			hx_ito->self_baseC[i] = temp_double / 10;
			I("self_baseC[%d] = %d\t", i, temp_double / 10);
		} else {/* TX */

			cal_vr_parameter(false, i + 1, 0);
			cal_self_tx_parameter();

			/* original scaling 10000 */
			temp_double =
			    hx_ito->self_bank_data[i] * SlfTX_Sense_Cap_PerBank +
			    hx_ito->self_dc_data[i] * Down_Scale * SlfTX_Sense_Cap_PerDAC / OSR_SelfTX;
			hx_ito->self_baseC[i] = temp_double / 10;
			I("self_baseC[%d] = %d\t", i, temp_double / 10);
		}

	}
#ifdef HIMAX_ITO_TEST_LOG
	/* Self BaseC data */
	memcpy(hx_ito->ito_log_baseC_self_data, hx_ito->self_baseC, self_num * sizeof(uint32_t));
	himax_ito_self_baseC_log_func();
#endif
#ifdef ITO_DEBUG_EN
	for (i = 0; i < x_channel + y_channel; i++) {
		I("self_baseC[%d] = %5d\t", i, hx_ito->self_baseC[i]);
		if (i % 8 == 7)
			I("\n");
	}
#endif
}

static void get_mutual_baseC(void)	/* (GetMutualBaseC) */
{
	uint32_t temp_double = 0;
	int i = 0;
	int j = 0;

	for (i = 0; i < y_channel; i++) {
		for (j = 0; j < x_channel; j++) {
			cal_vr_parameter(true, i + 1, j + 1);
			cal_mutual_parameter();

			/* original scaling 10000 */
			temp_double =
			    hx_ito->mutual_bank_data[i * x_channel + j] * Mut_Sense_Cap_PerBank +
			    hx_ito->mutual_dc_data[i * x_channel + j] * Down_Scale * Mut_Sense_Cap_PerDAC / OSR_Mutual;
			hx_ito->mutual_baseC[i * x_channel + j] = temp_double / 10;
			/* hx_ito->mutual_baseC[i][j] = temp_double / 10; */
		}
	}
#ifdef HIMAX_ITO_TEST_LOG
	/* Mutual BaseC data */
	memcpy(hx_ito->ito_log_baseC_mul_data, hx_ito->mutual_baseC, mutual_num * sizeof(uint32_t));
	hx_ito->test_log_type = BASEC;
	himax_ito_mul_baseC_log_func();
#endif
#ifdef ITO_DEBUG_EN
	for (i = 0; i < y_channel; i++) {
		for (j = 0; j < x_channel; j++) {
			if (i < mutual_num)
				I("mutual_baseC[%d][%d] = %5d\n", i, j, hx_ito->mutual_baseC[i * x_channel + j]);
			if (i % 8 == 7)
				I("\n");
		}
	}
#endif
}

static void himax_cal_parameter(void)
{
	VRH_V = VRH * 10 + 2 * MULITPLE_SCALE;	/* VRH_V = VRH / 10.0 + 2 */
	VR2_S_V = VRH_V * VR2_S / 32;
	I("%s : VRH_V = %d, VR2_S_V = %d\n", __func__, VRH_V, VR2_S_V);
}

static void himax_parameter_initial(void)
{
    /***** C5 *****/
	VRH = c3[1];
	VR1_S = c3[2];
	VR2_S = c3[3];
	VR3 = c3[4];
	VR4_S = c3[5];
	VR5_M = c3[6];
	VR6 = c3[7];
	VR7 = c3[8];
	VR8 = c3[9];
	VR9 = c3[10];
	I("%s : VRH = %d, VR1_S = %d, VR2_S = %d, VR3 = %d,\n", __func__, VRH, VR1_S, VR2_S, VR3);
	I("%s : VR4_S = %d, VR5_M = %d, VR6 = %d, VR7 = %d,\n", __func__, VR4_S, VR5_M, VR6, VR7);
	I("%s : VR8 = %d, VR9 = %d\n", __func__, VR8, VR9);

    /***** C6 *****/
	VR1_M = c4[1];
	VR3_M = c4[2];
	VR4_M = c4[3];

	I("%s : VR1_M = %d, VR3_M = %d, VR4_M = %d,\n", __func__, VR1_M, VR3_M, VR4_M);

    /***** BaseLine - 0xFE(6C) *****/
	Mutual_BaseLine = c9[1];
	Self_BaseLineUp = c9[2];
	Self_BaseLineLow = c9[3];

	I("%s : Mutual_BaseLine = %d, Self_BaseLineUp = %d, Self_BaseLineLow = %d,\n", __func__, Mutual_BaseLine,
	  Self_BaseLineUp, Self_BaseLineLow);
    /***** VR7 - 0xA4 *****/
	if (c7[1] == 0x94 && c7[2] == 0x62)
		VR7_Enable_Self = 1;
	else
		VR7_Enable_Self = 0;
	if (c7[3] == 0x94 && c7[4] == 0x62)
		VR7_Enable_Mutual = 1;
	else
		VR7_Enable_Mutual = 0;
	I("%s : VR7_Enable_Self = %d, VR7_Enable_Mutual = %d\n", __func__, VR7_Enable_Self, VR7_Enable_Mutual);
    /***** OSR - 0xE1 *****/
	ChrageDischarge_Mutual = c1[2] & 0x0F;
	ChrageDischarge_SelfRX = c1[3] & 0x0F;
	ChrageDischarge_SelfTX = c1[4] & 0x0F;
	ChrageDischarge_SelfBT = c1[5] & 0x0F;
	OSR_Mutual = (c1[6] & 0x0F) * 16 + (c1[7] & 0x0F);
	OSR_SelfRX = (c1[8] & 0x0F) * 16 + (c1[9] & 0x0F);
	OSR_SelfTX = (c1[10] & 0x0F) * 16 + (c1[11] & 0x0F);
	OSR_SelfBT = (c1[12] & 0x0F) * 16 + (c1[13] & 0x0F);
	I("%s : ChrageDischarge_Mutual = %d, ChrageDischarge_SelfRX = %d,",
		__func__,  ChrageDischarge_Mutual, ChrageDischarge_SelfRX);
	I("ChrageDischarge_SelfTX = %d, ChrageDischarge_SelfBT = %d,\n",
		ChrageDischarge_SelfTX, ChrageDischarge_SelfBT);
	I("%s : OSR_Mutual = %d, OSR_SelfRX = %d, OSR_SelfTX = %d, OSR_SelfBT = %d,\n", __func__, OSR_Mutual,
	  OSR_SelfRX, OSR_SelfTX, OSR_SelfBT);
    /***** MUL_8_16 - 0x3F *****/
	MUL_8_16_Enable_Mutual = c6[1] & 0x02;
	MUL_8_16_Enable_Self = c6[1] & 0x02;
	I("%s : MUL_8_16_Enable_Mutual = %d, MUL_8_16_Enable_Self = %d\n", __func__, MUL_8_16_Enable_Mutual,
	  MUL_8_16_Enable_Self);
    /***** P_PULSE - 0xBC *****/
	P_PulseMask_Enable = 0;
	P_PulseMask = 0;
	P_PulseSelf = c2[1];
	P_PulseMutual = c2[3];
	I("%s : P_PulseMask_Enable = %d, P_PulseMask = %d, P_PulseSelf = %d, P_PulseMutual = %d,\n", __func__,
	  P_PulseMask_Enable, P_PulseMask, P_PulseSelf, P_PulseMutual);
    /***** Bank Cap/DAC Cap - 0xEC *****/
	Bank_Cap_pf = 25;	/* 0.25 */
	DAC_Cap_pf = 50;	/* 0.5 */
	if ((c5[1] & 0x80) == 0x80)
		DAC_Ratio = 50;	/* 0.5 */
	else
		DAC_Ratio = 25;	/* 0.25 */

	I("%s : Bank_Cap_pf = %d, DAC_Cap_pf = %d\n", __func__, Bank_Cap_pf, DAC_Cap_pf);
    /***** Check Auto Bank Cut and Two Set VR - 0xFE(02) *****/
	Is2T3R = true;		/* (fw_major_version & 0xFF) == 0xE5) */
	if ((c8[3] & 0x40) == 0x40)
		AutoBankCut = true;
	else
		AutoBankCut = false;
	/* -two set VR */
	if ((c8[3] & 0x08) == 0x08) {
		/* Mutual_TwoSetVR = true; */
		Self_TwoSetVR = true;
	} else {
		Mutual_TwoSetVR = false;
		Self_TwoSetVR = false;
	}

	I("%s : AutoBankCut = %d, Self_TwoSetVR = %d\n", __func__, AutoBankCut, Self_TwoSetVR);
	if (Mutual_TwoSetVR || Self_TwoSetVR) {
	/***** Get Two Set VR - 0xFE(E0) *****/
		TwoSet_MutualMap[0] = (c10[1] & 0x80) >> 7;	/* Cycle 1 */
		TwoSet_MutualMap[1] = (c10[1] & 0x40) >> 6;	/* Cycle 2 */
		TwoSet_MutualMap[2] = (c10[1] & 0x20) >> 5;	/* Cycle 3 */
		TwoSet_MutualMap[3] = (c10[2] & 0x80) >> 7;	/* Cycle 4 */

		TwoSet_VR1_M_1 = c10[3] & 0x1F;
		TwoSet_VR3_M_1 = c10[4] & 0x1F;
		TwoSet_VR4_M_1 = c10[5] & 0x1F;

		TwoSet_VR1_M_2 = c10[6] & 0x1F;
	/***** Get Two Set VR - 0xFE(E6) *****/
		TwoSet_VR3_M_2 = c11[1] & 0x1F;
		TwoSet_VR4_M_2 = c11[2] & 0x1F;

		TwoSet_VR1_S_RX = c11[3] & 0x1F;
		TwoSet_VR3_S_RX = c11[4] & 0x1F;
		TwoSet_VR4_S_RX = c11[5] & 0x1F;

		TwoSet_VR1_S_TX = c11[6] & 0x1F;
		TwoSet_VR3_S_TX = c11[7] & 0x1F;
		TwoSet_VR4_S_TX = c11[8] & 0x1F;

	}

I("%s : TwoSet_MutualMap[0] = %d, TwoSet_MutualMap[1] = %d, TwoSet_MutualMap[2] = %d, TwoSet_MutualMap[3] = %d,\n",
		__func__, TwoSet_MutualMap[0], TwoSet_MutualMap[1], TwoSet_MutualMap[2], TwoSet_MutualMap[3]);
	I("%s : TwoSet_VR1_M_1 = %d, TwoSet_VR3_M_1 = %d, TwoSet_VR4_M_1 = %d, TwoSet_VR1_M_2 = %d,\n", __func__,
	  TwoSet_VR1_M_1, TwoSet_VR3_M_1, TwoSet_VR4_M_1, TwoSet_VR1_M_2);
	I("%s : TwoSet_VR3_M_2 = %d, TwoSet_VR4_M_2 = %d, TwoSet_VR1_S_RX = %d, TwoSet_VR3_S_RX = %d,\n", __func__,
	  TwoSet_VR3_M_2, TwoSet_VR4_M_2, TwoSet_VR1_S_RX, TwoSet_VR3_S_RX);
	I("%s : TwoSet_VR4_S_RX = %d, TwoSet_VR1_S_TX = %d, TwoSet_VR3_S_TX = %d, TwoSet_VR4_S_TX = %d,\n", __func__,
	  TwoSet_VR4_S_RX, TwoSet_VR1_S_TX, TwoSet_VR3_S_TX, TwoSet_VR4_S_TX);
}

static void himax_get_test_raw_data(uint8_t diag_command, uint16_t mutual_num, uint16_t self_num,
				    uint16_t *mutual_data, uint16_t *self_data)
{
#ifdef HIMAX_ITO_TEST_LOG
#ifdef ITO_DEBUG_DIS
	int i = 0;
#endif
#endif

	ito_set_step_status(RAW_DATA);
	g_diag_command = diag_command;
	himax_get_raw_data(diag_command, mutual_num, self_num);
#ifdef ITO_DEBUG_DIS
	for (i = 0; i < mutual_num + self_num; i++) {
		if (i < mutual_num)
			I("mutual[%d] = %3d \t", i, mutual_bank[i]);
		else
			I("self[%d] = %3d \t", i - mutual_num, self_bank[i - mutual_num]);
		if (i % 8 == 7)
			I("\n");
	}
#endif

	if (diag_command == 3) {
		memcpy(hx_ito->mutual_bank_data, mutual_bank, mutual_num * sizeof(uint16_t));
		memcpy(hx_ito->self_bank_data, self_bank, self_num * sizeof(uint16_t));
#ifdef HIMAX_ITO_TEST_LOG
		/* run store data to sdcard */
		memcpy(hx_ito->ito_log_mul_data, hx_ito->mutual_bank_data, mutual_num * sizeof(uint16_t));
		memcpy(hx_ito->ito_log_self_data, hx_ito->self_bank_data, self_num * sizeof(uint16_t));
		hx_ito->test_log_type = BANK;
		himax_ito_mul_log_func();
		himax_ito_self_log_func();
#ifdef ITO_DEBUG_DIS
		for (i = 0; i < mutual_num + self_num; i++) {
			if (i < mutual_num)
				I("ito_log_mul_data mutual bank[%d] = %3d \t", i, hx_ito->ito_log_mul_data[i]);
			else
				I("ito_log_mul_data self bank[%d] = %3d \t", i - mutual_num,
				  hx_ito->ito_log_mul_data[i - mutual_num]);
			if (i % 8 == 7)
				I("\n");
		}
#endif
#endif
	}
	if (diag_command == 2) {
		memcpy(hx_ito->mutual_dc_data, mutual_bank, mutual_num * sizeof(uint16_t));
		memcpy(hx_ito->self_dc_data, self_bank, self_num * sizeof(uint16_t));
#ifdef HIMAX_ITO_TEST_LOG
		/* run store data to sdcard */
		memcpy(hx_ito->ito_log_mul_data, hx_ito->mutual_dc_data, mutual_num * sizeof(uint16_t));
		memcpy(hx_ito->ito_log_self_data, hx_ito->self_dc_data, self_num * sizeof(uint16_t));
		hx_ito->test_log_type = DC;
		himax_ito_mul_log_func();
		himax_ito_self_log_func();
#ifdef ITO_DEBUG_DIS
		for (i = 0; i < mutual_num + self_num; i++) {
			if (i < mutual_num)
				I("ito_log_mul_data mutual DC[%d] = %3d \t", i, hx_ito->ito_log_mul_data[i]);
			else
				I("ito_log_mul_data self DC[%d] = %3d \t", i - mutual_num,
				  hx_ito->ito_log_mul_data[i - mutual_num]);
			if (i % 8 == 7)
				I("\n");
		}
#endif
#endif
	}
	if (diag_command == 1) {
		memcpy(hx_ito->mutual_iir_data, mutual_bank, mutual_num * sizeof(uint16_t));
		memcpy(hx_ito->self_iir_data, self_bank, self_num * sizeof(uint16_t));
#ifdef HIMAX_ITO_TEST_LOG
		/* run store data to sdcard */
		memcpy(hx_ito->ito_log_mul_data, hx_ito->mutual_iir_data, mutual_num * sizeof(uint16_t));
		memcpy(hx_ito->ito_log_self_data, hx_ito->self_iir_data, self_num * sizeof(uint16_t));
		hx_ito->test_log_type = IIR;
		himax_ito_mul_log_func();
		himax_ito_self_log_func();
#ifdef ITO_DEBUG_DIS
		for (i = 0; i < mutual_num + self_num; i++) {
			if (i < mutual_num)
				I("ito_log_mul_data mutual IIR[%d] = %3d \t", i, hx_ito->ito_log_mul_data[i]);
			else
				I("ito_log_mul_data self IIR[%d] = %3d \t", i - mutual_num,
				  hx_ito->ito_log_mul_data[i - mutual_num]);
			if (i % 8 == 7)
				I("\n");
		}
#endif
#endif
	}
}

void himax_read_mkey(void)
{
	uint8_t temp_data[4] = { 0x00 };
	uint8_t temp_cmd[4] = { 0x00 };
	int i = 0;

    /***** Read location of m_key 0xFE(64) *****/

	temp_cmd[0] = 0x64;
	himax_register_read(private_ts->client, temp_cmd, 1, temp_data, true);

	hx_ito->mkey_num = temp_data[0] & 0x0F;
	I("%s mkey_num = %X", __func__, hx_ito->mkey_num);

	for (i = 0; i < hx_ito->mkey_num; i++) {
		temp_cmd[0] = 0x68 + i;
		himax_register_read(private_ts->client, temp_cmd, 1, temp_data, true);
		hx_ito->m_key_location[i] = temp_data[0];
		I("%s m_key_location[%d] = %d", __func__, i, hx_ito->m_key_location[i]);
	}

}

uint8_t himax_ito_test(void)
{
	int8_t test_result = 0;
	uint8_t temp_data[4];
#ifdef ITO_DEBUG_DIS
	int i = 0;
#endif

	I("%s Start", __func__);
	g_self_test_entered = 1;
	ito_set_step_status(START_TEST);
	ito_set_result_status(TEST_ONGOING);
    /********* ITO test data struct initial - Start*********/
	mutual_num = x_channel * y_channel;
	self_num = x_channel + y_channel;	/* don't add KEY_COUNT */

	mutual_bank = kzalloc((mutual_num * sizeof(uint16_t)), GFP_KERNEL);	/* initial for array of getting data */
	self_bank = kzalloc((self_num * sizeof(uint16_t)), GFP_KERNEL);

	hx_ito = kzalloc((sizeof(struct himax_ito_data)), GFP_KERNEL);	/* initial for data array */
	if (hx_ito == NULL) {
		return -ENOMEM;
	}
	hx_ito->mutual_bank_data = kzalloc((mutual_num * sizeof(uint16_t)), GFP_KERNEL);
	hx_ito->self_bank_data = kzalloc((self_num * sizeof(uint16_t)), GFP_KERNEL);
	hx_ito->mutual_dc_data = kzalloc((mutual_num * sizeof(uint16_t)), GFP_KERNEL);
	hx_ito->self_dc_data = kzalloc((self_num * sizeof(uint16_t)), GFP_KERNEL);
	hx_ito->mutual_iir_data = kzalloc((mutual_num * sizeof(uint16_t)), GFP_KERNEL);
	hx_ito->self_iir_data = kzalloc((self_num * sizeof(uint16_t)), GFP_KERNEL);
	hx_ito->mutual_baseC = kzalloc((mutual_num * sizeof(uint32_t)), GFP_KERNEL);
	hx_ito->self_baseC = kzalloc((self_num * sizeof(uint32_t)), GFP_KERNEL);
	hx_ito->mutual_baseC_percent_result = kzalloc((mutual_num * sizeof(int16_t)), GFP_KERNEL);
	hx_ito->self_baseC_percent_result = kzalloc((self_num * sizeof(int16_t)), GFP_KERNEL);
	hx_ito->mutual_tx_dev_result = kzalloc((mutual_num * sizeof(int16_t)), GFP_KERNEL);
	hx_ito->mutual_rx_dev_result = kzalloc((mutual_num * sizeof(int16_t)), GFP_KERNEL);
	hx_ito->self_tx_dev_result = kzalloc((self_num * sizeof(int16_t)), GFP_KERNEL);
	hx_ito->self_rx_dev_result = kzalloc((self_num * sizeof(int16_t)), GFP_KERNEL);
	hx_ito->noise_test_result = kzalloc(((mutual_num + self_num) * sizeof(int16_t)), GFP_KERNEL);

#ifdef HIMAX_ITO_TEST_LOG
	hx_ito->ito_log_mul_data = kzalloc((mutual_num * sizeof(uint16_t)), GFP_KERNEL);
	hx_ito->ito_log_self_data = kzalloc((mutual_num * sizeof(uint16_t)), GFP_KERNEL);
	hx_ito->ito_log_baseC_mul_data = kzalloc((mutual_num * sizeof(uint32_t)), GFP_KERNEL);
	hx_ito->ito_log_baseC_self_data = kzalloc((mutual_num * sizeof(uint32_t)), GFP_KERNEL);
#endif
    /**********************************************************************************/

	himax_int_enable(private_ts->client->irq, 0);
/* himax_sense_off(private_ts->client); //APK will reset */
/* msleep(120); */
	himax_read_mkey();
	temp_data[0] = 0x00;
	i2c_himax_write(private_ts->client, 0xE3, temp_data, 1, DEFAULT_RETRY_CNT);	/* disable reload */
	test_result = himax_load_config_table();
	if (test_result < 0) {
		I("%s: Test fail item = %3d \t", __func__, test_result);
		goto ito_test_end;
	}
	himax_sense_on(private_ts->client, 0x01);
	msleep(120);

    /*********Get Bank data*********/
	himax_get_test_raw_data(0x03, mutual_num, self_num, hx_ito->mutual_bank_data, hx_ito->self_bank_data);

#ifdef ITO_DEBUG_DIS
	for (i = 0; i < mutual_num + self_num; i++) {
		if (i < mutual_num)
			I("mutual_bank_data[%d] = %3d \t", i, hx_ito->mutual_bank_data[i]);
		else
			I("self_bank_data[%d] = %3d \t", i - mutual_num, hx_ito->self_bank_data[i - mutual_num]);
		if (i % 8 == 7)
			I("\n");
	}
#endif

    /*********Get DC data*********/
	himax_get_test_raw_data(0x02, mutual_num, self_num, hx_ito->mutual_dc_data, hx_ito->self_dc_data);
#ifdef ITO_DEBUG_DIS
	for (i = 0; i < mutual_num + self_num; i++) {
		if (i < mutual_num)
			I("mutual_dc_data[%d] = %3d \t", i, hx_ito->mutual_dc_data[i]);
		else
			I("self_dc_data[%d] = %3d \t", i - mutual_num, hx_ito->self_dc_data[i - mutual_num]);
		if (i % 8 == 7)
			I("\n");
	}
#endif

    /*********Get IIR data*********/
	himax_get_test_raw_data(0x01, mutual_num, self_num, hx_ito->mutual_dc_data, hx_ito->self_dc_data);
#ifdef ITO_DEBUG_DIS
	for (i = 0; i < mutual_num + self_num; i++) {
		if (i < mutual_num)
			I("mutual_dc_data[%d] = %3d \t", i, hx_ito->mutual_iir_data[i]);
		else
			I("self_dc_data[%d] = %3d \t", i - mutual_num, hx_ito->self_iir_data[i - mutual_num]);
		if (i % 8 == 7)
			I("\n");
	}
#endif

	/* initial and get parameter (VR,OSR...setting) */
	himax_parameter_initial();
	himax_cal_parameter();

	/* Calculate BaseC */
	get_mutual_baseC();
	get_self_baseC();
	/* Do ITO test */
	I("%s : mutual_base_c_max = %d\n", __func__, mutual_base_c_max);

	test_result += baseC_mutual_percent_limit_check();
/* if(test_result != 0) //PASS : result = 0 */
/* goto ito_test_end; */

	test_result += baseC_self_percent_limit_check();
/* if(test_result != 0) //PASS : result = 0 */
/* goto ito_test_end; */

	test_result += mutual_dev_tx_check();
/* if(test_result != 0) //PASS : result = 0 */
/* goto ito_test_end; */

	test_result += mutual_dev_rx_check();
/* if(test_result != 0) //PASS : result = 0 */
/* goto ito_test_end; */

	test_result += self_dev_tx_check();
/* if(test_result != 0) //PASS : result = 0 */
/* goto ito_test_end; */

	test_result += self_dev_rx_check();
/* if(test_result != 0) //PASS : result = 0 */
/* goto ito_test_end; */

	test_result += himax_noise_test();
/* if(test_result != 0) //PASS : result = 0 */
/* goto ito_test_end; */

	if (test_result == 0)
		ito_set_result_status(TEST_PASS);
ito_test_end:
	if (test_result < 0)
		ito_set_result_status(LOAD_CONFIG_FILE_FAIL);
	ito_set_step_status(END_TEST);
	himax_ic_reset(false, false);
	msleep(120);
	himax_sense_on(private_ts->client, 0x01);

	himax_int_enable(private_ts->client->irq, 1);

	kfree(hx_ito->mutual_bank_data);
	kfree(hx_ito->self_bank_data);
	kfree(hx_ito->mutual_dc_data);
	kfree(hx_ito->self_dc_data);
	kfree(hx_ito->mutual_iir_data);
	kfree(hx_ito->self_iir_data);
	kfree(hx_ito->mutual_baseC);
	kfree(hx_ito->self_baseC);
	kfree(hx_ito->mutual_baseC_percent_result);
	kfree(hx_ito->self_baseC_percent_result);
	kfree(hx_ito->mutual_tx_dev_result);
	kfree(hx_ito->mutual_rx_dev_result);
	kfree(hx_ito->self_tx_dev_result);
	kfree(hx_ito->self_rx_dev_result);
	kfree(hx_ito->noise_test_result);
#ifdef HIMAX_ITO_TEST_LOG
	kfree(hx_ito->ito_log_mul_data);
	kfree(hx_ito->ito_log_self_data);
	kfree(hx_ito->ito_log_baseC_mul_data);
	kfree(hx_ito->ito_log_baseC_self_data);
#endif
	kfree(mutual_bank);
	kfree(self_bank);
	kfree(hx_ito);

	g_self_test_entered = 0;
	return test_result;
}
