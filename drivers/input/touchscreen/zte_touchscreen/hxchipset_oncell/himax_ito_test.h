#ifndef H_HIMAX_ITO_TEST
#define H_HIMAX_ITO_TEST

#include "himax_ic.h"

#define HIMAX_ITO_TEST_LOG
#define ITO_DEBUG_EN
#define ITO_DEBUG_DIS
/* #define MutPar_DEBUG */
/* #define VR_DEBUG */

struct himax_ito_data {
	uint16_t *mutual_bank_data;
	uint16_t *self_bank_data;
	uint16_t *mutual_dc_data;
	uint16_t *self_dc_data;
	uint16_t *mutual_iir_data;
	uint16_t *self_iir_data;
	uint32_t *mutual_baseC;
	uint32_t *self_baseC;
	int16_t *mutual_baseC_percent_result;
	int16_t *self_baseC_percent_result;
	int16_t *mutual_tx_dev_result;
	int16_t *mutual_rx_dev_result;
	int16_t *self_tx_dev_result;
	int16_t *self_rx_dev_result;
	int16_t *noise_test_result;
	uint8_t mkey_num;
	uint8_t m_key_location[3];
#ifdef HIMAX_ITO_TEST_LOG
	uint16_t *ito_log_mul_data;
	uint16_t *ito_log_self_data;
	uint32_t *ito_log_baseC_mul_data;
	uint32_t *ito_log_baseC_self_data;
	uint8_t test_log_type;
	struct workqueue_struct *himax_ito_test_log_wq;
	struct work_struct ito_test_log_work;
#endif
};

enum data_type {
	BANK = 1,
	DC,
	IIR,
	BASEC,
};

enum ito_test_type {
	MUTUAL_PERCENT_LIMIT,
	SELF_PERCENT_LIMIT,
	MUTUAL_DEV_TX,
	MUTUAL_DEV_RX,
	SELF_DEV_TX,
	SELF_DEV_RX,
	MUTUAL_IIR_NOISE,
	SELF_IIR_NOISE,
};

/******* Test Thx setting*******/
uint8_t max_mutual_baseC_percent = 90;	/* 90% */
uint8_t min_mutual_baseC_percent = 10;	/* 10% */

uint8_t max_self_baseC_percent = 90;	/* 90% */
uint8_t min_self_baseC_percent = 10;	/* 10% */

int8_t mutual_dev_fail_upper = 30;	/* 30% */
int8_t mutual_dev_fail_lower = -30;	/* -30% */

int8_t self_dev_fail_upper = 30;	/* 30% */
int8_t self_dev_fail_lower = -30;	/* -30% */

int8_t noice_thx = 20;

/*******************************/

#ifdef HIMAX_ITO_TEST_LOG
#define ITO_LOG_FILE "/sdcard/HX_ITO_LOG.txt"
#endif
#define ITO_FAIL_LOG_FILE "/sdcard/HX_ITO_FAIL_LOG.txt"

uint32_t MULITPLE_SCALE = 100;
uint16_t mutual_num = 0;
uint16_t self_num = 0;

uint32_t mutual_base_c_max = 0;
uint32_t self_rx_base_c_max = 0;
uint32_t self_tx_base_c_max = 0;
int CycleNumber = 0;
bool TwoSet_MutualMap[4];

uint8_t TwoSet_VR1_S_RX = 0;
uint8_t TwoSet_VR3_S_RX = 0;
uint8_t TwoSet_VR4_S_RX = 0;

uint8_t TwoSet_VR1_S_TX = 0;
uint8_t TwoSet_VR3_S_TX = 0;
uint8_t TwoSet_VR4_S_TX = 0;

uint8_t TwoSet_VR1_M_1 = 0;
uint8_t TwoSet_VR3_M_1 = 0;
uint8_t TwoSet_VR4_M_1 = 0;

uint8_t TwoSet_VR1_M_2 = 0;
uint8_t TwoSet_VR3_M_2 = 0;
uint8_t TwoSet_VR4_M_2 = 0;

uint8_t VRH = 0;
uint8_t VR1_S = 0;
uint8_t VR2_S = 0;
uint8_t VR3 = 0;
uint8_t VR4_S = 0;
uint8_t VR5_M = 0;
uint8_t VR6 = 0;
uint8_t VR7 = 0;
uint8_t VR8 = 0;
uint8_t VR9 = 0;

uint8_t VR1_M = 0;
uint8_t VR3_M = 0;
uint8_t VR4_M = 0;

uint8_t VR7_Enable_Self = 0;
uint8_t VR7_Enable_Mutual = 0;

uint8_t OSR_SelfRX = 0;
uint8_t OSR_SelfTX = 0;
uint8_t OSR_SelfBT = 0;
uint8_t OSR_Mutual = 0;

uint8_t ChrageDischarge_SelfRX = 0;
uint8_t ChrageDischarge_SelfTX = 0;
uint8_t ChrageDischarge_SelfBT = 0;
uint8_t ChrageDischarge_Mutual = 0;

uint8_t MUL_8_16_Enable_Self = 0;
uint8_t MUL_8_16_Enable_Mutual = 0;

uint8_t P_PulseMask_Enable = 0;
uint8_t P_PulseMask = 0;
uint8_t P_PulseSelf = 0;
uint8_t P_PulseMutual = 0;

uint32_t Bank_Cap_pf = 0;
uint32_t DAC_Cap_pf = 0;
uint32_t DAC_Ratio = 0;

uint8_t SelfBankCut[200];
uint8_t MutualBankCut = 0;

uint8_t Mutual_BaseLine = 0;
uint8_t Self_BaseLineUp = 0;
uint8_t Self_BaseLineLow = 0;

uint32_t VRH_V = 0;
uint32_t VR1_S_V = 0;
uint32_t VR2_S_V = 0;
uint32_t VR3_V = 0;
uint32_t VR4_S_V = 0;
uint32_t VR5_M_V = 0;
uint32_t VR6_V = 0;
uint32_t VR7_V = 0;
uint32_t VR8_V = 0;
uint32_t VR9_V = 0;

uint32_t VR1_M_V = 0;
uint32_t VR3_M_V = 0;
uint32_t VR4_M_V = 0;

uint8_t Mut_MUL_8_16 = 0;
uint8_t SlfRX_MUL_8_16 = 0;
uint8_t SlfTX_MUL_8_16 = 0;
uint8_t SlfBT_MUL_8_16 = 0;

uint32_t TX_Voltage_P = 0;
uint32_t TX_Voltage_N = 0;

uint32_t Mut_Sense_Cap_PerBank = 0;
uint32_t Mut_Sense_Cap_PerDAC = 0;
uint32_t SlfRX_Sense_Cap_PerBank = 0;
uint32_t SlfRX_Sense_Cap_PerDAC = 0;
uint32_t SlfTX_Sense_Cap_PerBank = 0;
uint32_t SlfTX_Sense_Cap_PerDAC = 0;
uint32_t SlfBT_Sense_Cap_PerBank = 0;
uint32_t SlfBT_Sense_Cap_PerDAC = 0;
uint32_t Down_Scale = 0;
uint32_t Temp_P_PulseMask = 0;
uint32_t Charge_Discharge = 0;
uint32_t Bank_Cut = 0;

bool Is2T3R = false;
bool AutoBankCut = false;
bool Mutual_TwoSetVR = false;
bool Self_TwoSetVR = false;

int TX_Num = 0;
int RX_Num = 0;
int BT_Num = 0;
bool UsingMutKey = false;

uint8_t ito_test_step = 0;
uint8_t ito_test_result = 0;
enum ito_test_step {
	START_TEST,
	RAW_DATA,
	PERCENT_TEST,
	DEV_TEST,
	NOISE_TEST,
	END_TEST = 9,
};
enum ito_test_result {
	TEST_PASS,
	TEST_FAIL,
	LOAD_CONFIG_FILE_FAIL,
	TEST_ONGOING = 0xF,
};

/*************** Config table parameter ***************/
#define CONFIG_TABLE_FILE "/sdcard/HX_CONFIG_TABLE.txt"
#define CONFIG_TABLE_BUF_SIZE 512
#define CONFIG_TABLE_DATA_BUF_SIZE 128

#define CONFIG_TABLE_DATA_MATCH_STR "0x"

#define CONFIG_TABLE_NORMAL_REG_STR_SIZE 4

#define CONFIG_TABLE_FE_REG_STR_SIZE 8
#define CONFIG_TABLE_FE_REG_MATCH_STR "0xFE"
#define CONFIG_TABLE_FE_REG_MATCH_SIZE 4

#define CONFIG_TABLE_CHAR_REG_NAME_SIZE 32
#define CONFIG_TABLE_CHAR_REG_NUM 11

#define TEST_THX_SETTING 0x10
#define CONFIG_TEST_THX_SETTING "TEST_THX_SETTING"
#define EN_DEFAULT_CFG 0x11
#define CONFIG_TABLE_EN_DEFAULT_CFG "EN_DEFAULT_CFG"
#define CFG_TABLE_DATE 0x12
#define CONFIG_TABLE_CFG_TABLE_DATE "CFG_TABLE_DATE"
#define CFG_TABLE_VERSION 0x13
#define CONFIG_TABLE_CFG_TABLE_VERSION  "CFG_TABLE_VERSION"
#define FW_MAJOR_VERSION 0x14
#define CONFIG_TABLE_FW_MAJOR_VERSION "FW_MAJOR_VERSION"
#define FW_MINOR_VERSION 0x15
#define CONFIG_TABLE_FW_MINOR_VERSION "FW_MINOR_VERSION"
#define CFG_FW_SIGN 0x16
#define CONFIG_TABLE_CFG_FW_SIGN "CFG_FW_SIGN"
#define CFG_TABLE_INFO 0x17
#define CONFIG_TABLE_CFG_TABLE_INFO "CFG_TABLE_INFO"
#define CFG_UNUSED 0x18
#define CONFIG_TABLE_CFG_UNUSED "CFG_UNUSED"
#define CFB_ADC_SEQ 0x19
#define CONFIG_TABLE_CFB_ADC_SEQ "CFB_ADC_SEQ"
#define CFB_FIT_CURVE 0x20
#define CONFIG_TABLE_CFB_FIT_CURVE "CFB_FIT_CURVE"

/* ------------------------------------------------- */
#define CHAR_MUTUAL_SELF_REF_NUM 3

#define GOLDEN_MUTUAL_DATA_TYPE 2
#define GOLDEN_MUTUAL_STR  "GOLDEN_MUTUAL"

#define GOLDEN_SELF_RX_DATA_TYPE 3
#define GOLDEN_SELF_RX_STR  "GOLDEN_SELF_RX"

#define GOLDEN_SELF_TX_DATA_TYPE 4
#define GOLDEN_SELF_TX_STR  "GOLDEN_SELF_TX"

#define HIMAX_STR_TO_DEC_FORMAT  0
#define HIMAX_STR_TO_HEX_FORMAT  1

/******************************************************/
/* Test config */
uint8_t *c1;
uint8_t *c2;
uint8_t *c3;
uint8_t *c4;
uint8_t *c5;
uint8_t *c6;
uint8_t *c7;
uint8_t *c8;
uint8_t *c9;
uint8_t *c10;
uint8_t *c11;

uint32_t *golden_mutual_baseC;
uint32_t *golden_rx_self_baseC;
uint32_t *golden_tx_self_baseC;
/*
uint8_t c1[15] = { 0xE1, 0x0B, 0x01, 0x01, 0x01, 0x01, 0x03, 0x07, 0x03, 0x07, 0x03, 0x07, 0x03, 0x0F, 0x08};
uint8_t c2[5] = { 0xBC, 0x00, 0x00, 0x00, 0x00};
uint8_t c3[11] = { 0xC5, 0x05, 0x1E, 0x00, 0x10, 0x17, 0x1E, 0x0B, 0x14, 0x00, 0x1F};
uint8_t c4[4] = { 0xC6, 0x1A, 0x10, 0x16};
uint8_t c5[4] = { 0xEC, 0x10, 0xDF, 0x8D};
uint8_t c6[2] = { 0x3F, 0x00};
uint8_t c7[5] = { 0xA4, 0x94, 0x62, 0x94, 0x86};
uint8_t c8[7] = { 0x02, 0x56, 0x05, 0x28, 0x12, 0x40, 0x00};
uint8_t c9[5] = { 0x6C, 0xAA, 0x82, 0x2D, 0x00};
uint8_t c10[7] = { 0xE0, 0x00, 0x00, 0x1A, 0x10, 0x16, 0x1A};
uint8_t c11[9] = { 0xE6, 0x10, 0x16, 0x1E, 0x10, 0x17, 0x1E, 0x10, 0x15};
*/

uint8_t himax_ito_test(void);

#endif
