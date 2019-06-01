#ifndef __LINUX_FT6X06_TS_H__
#define __LINUX_FT6X06_TS_H__

#include <ontim/touchscreen/touchpanel.h>

/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS	5

#define PRESS_MAX	0xFF
#define FT_PRESS		0x7F

#define Proximity_Max	32

#define FT_FACE_DETECT_ON		0xc0
#define FT_FACE_DETECT_OFF		0xe0

#define FT_FACE_DETECT_ENABLE	1
#define FT_FACE_DETECT_DISABLE	0
#define FT_FACE_DETECT_REG		0xB0

#define FT6X06_NAME 	"focaltech-ts"
#define FT6x06_TS_ADDR    0x38

#define FT_MAX_ID	0x0F
#define FT_TOUCH_STEP	6
#define FT_FACE_DETECT_POS		1
#define FT_TOUCH_X_H_POS		3
#define FT_TOUCH_X_L_POS		4
#define FT_TOUCH_Y_H_POS		5
#define FT_TOUCH_Y_L_POS		6
#define FT_TOUCH_EVENT_POS		3
#define FT_TOUCH_ID_POS			5

#define POINT_READ_BUF	(3 + FT_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

/*register address*/
#define FT6x06_REG_FW_VER		0xA6
#define FT6x06_REG_FW_VENDOR	0xA8
#define FT6x06_REG_POINT_RATE	0x88
#define FT6x06_REG_THGROUP	0x80
#define FT6x06_REG_CHARGER_STATE	0x8B

#define FTS_EVENT_LIFT				(0x20)
#define FTS_EVENT_RIGHT			(0x21)
#define FTS_EVENT_UP				(0x22)
#define FTS_EVENT_DOWN			(0x23)
#define FTS_EVENT_DOUBLE_CLICK	(0x24)
void ft6x06_reset_tp(int HighOrLow);

int ft6x06_i2c_Read(struct i2c_client *client, char *writebuf, int writelen,
		    char *readbuf, int readlen);
int ft6x06_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);
 //+add by hzb

#define FTS_POINT_UP		0x01
#define FTS_POINT_DOWN		0x00
#define FTS_POINT_CONTACT	0x02
#define FTS_POINT_INVALID	0x03

#define FT6x06_REG_PMODE 0xa5 	

/* FT6X06_REG_PMODE. */
#define PMODE_ACTIVE	    	0x00
#define PMODE_MONITOR	    	0x01
#define PMODE_STANDBY	    	0x02
#define PMODE_HIBERNATE     	0x03

#define VENDOR_ID_NULL  0x79
#define VENDOR_ID_OFILM  0x51
#define VENDOR_ID_TRULY  0x5A
#define VENDOR_ID_YUSHUN  0x5F
#define VENDOR_ID_YUANSHENG  0x74
#define VENDOR_ID_CNT  0xB2
#define VENDOR_ID_CHENXING  0x90
#define VENDOR_ID_ZHILING  0x30
#define VENDOR_ID_TIANYI  0x71
#define VENDOR_ID_CHUANGSHI  0x39
#define VENDOR_ID_SUCCESS  0x5F
#define VENDOR_ID_SHENYUE 0xA0
#define VENDOR_ID_LAIBAO 0x55
#define VENDOR_ID_BONEN 0x3B

/*upgrade config of FT5606*/
#define FT5606_UPGRADE_AA_DELAY 		50
#define FT5606_UPGRADE_55_DELAY 		10
#define FT5606_UPGRADE_ID_1			0x79
#define FT5606_UPGRADE_ID_2			0x06
#define FT5606_UPGRADE_READID_DELAY 	100
#define FT5606_UPGRADE_EARSE_DELAY	2000


/*upgrade config of FT5316*/
#define FT5316_UPGRADE_AA_DELAY 		50
#define FT5316_UPGRADE_55_DELAY 		30
#define FT5316_UPGRADE_ID_1			0x79
#define FT5316_UPGRADE_ID_2			0x07
#define FT5316_UPGRADE_READID_DELAY 	1
#define FT5316_UPGRADE_EARSE_DELAY	1500


/*upgrade config of FT5x06(x=2,3,4)*/
#define FT5x06_UPGRADE_AA_DELAY 		50
#define FT5x06_UPGRADE_55_DELAY 		30
#define FT5x06_UPGRADE_ID_1			0x79
#define FT5x06_UPGRADE_ID_2			0x03
#define FT5x06_UPGRADE_READID_DELAY 	1
#define FT5x06_UPGRADE_EARSE_DELAY	2000

/*upgrade config of FT6208*/
#define FT6208_UPGRADE_AA_DELAY 		60
#define FT6208_UPGRADE_55_DELAY 		10
#define FT6208_UPGRADE_ID_1			0x79
#define FT6208_UPGRADE_ID_2			0x05
#define FT6208_UPGRADE_READID_DELAY 	10
#define FT6208_UPGRADE_EARSE_DELAY	2000

/*upgrade config of FT6x06(x=2,3,4)*/
#define FT6x06_UPGRADE_AA_DELAY 		100
#define FT6x06_UPGRADE_55_DELAY 		30
#define FT6x06_UPGRADE_ID_1			0x79
#define FT6x06_UPGRADE_ID_2			0x08
#define FT6x06_UPGRADE_READID_DELAY 	1
#define FT6x06_UPGRADE_EARSE_DELAY	2000

/*upgrade config of FT5X36*/
#define FT5x36_UPGRADE_AA_DELAY 		30
#define FT5x36_UPGRADE_55_DELAY 		30
#define FT5x36_UPGRADE_ID_1			0x79
#define FT5x36_UPGRADE_ID_2			0x11
#define FT5x36_UPGRADE_READID_DELAY 	10
#define FT5x36_UPGRADE_EARSE_DELAY	2000

/*upgrade config of FT6X36*/
#define FT6x36_UPGRADE_AA_DELAY 		100
#define FT6x36_UPGRADE_55_DELAY 		30
#define FT6x36_UPGRADE_ID_1			0x79
#define FT6x36_UPGRADE_ID_2			0x18
#define FT6x36_UPGRADE_READID_DELAY 	10
#define FT6x36_UPGRADE_EARSE_DELAY	2000
#define FT6x36_UPGRADE_ID			0x7918

#endif
