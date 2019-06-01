/*
 *drivers/input/touchscreen/ft5x06_ex_fun.c
 *
 *FocalTech ft6x06 expand function for debug.
 *
 *Copyright (c) 2010  Focal tech Ltd.
 *
 *This software is licensed under the terms of the GNU General Public
 *License version 2, as published by the Free Software Foundation, and
 *may be copied, distributed, and modified under those terms.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 *
 *Note:the error code of EIO is the general error in this file.
 */


#include "ft6x06_ex_fun.h"
#include <ontim/touchscreen/focaltech/ft6x06_ts.h>
#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>

struct Upgrade_Info {
	u16 delay_aa;		/*delay of write FT_UPGRADE_AA */
	u16 delay_55;		/*delay of write FT_UPGRADE_55 */
	u8 upgrade_id_1;	/*upgrade id 1 */
	u8 upgrade_id_2;	/*upgrade id 2 */
	u16 delay_readid;	/*delay of read id */
	u16 delay_earse_flash; /*delay of earse flash*/
};


enum{
     FW_UPGDRADE_SUCCESS=0,
     FW_UPGDRADE_PROGRAM,
     FW_UPGDRADE_FAIL,
     FW_UPGDRADE_LATEST,
     FW_UPGDRADE_FILE_ERR,
     FW_UPGDRADE_FILE_READ_ERR,
     FW_UPGDRADE_FILE_VER_ERR,
};
static unsigned char fw_upgrade_state=FW_UPGDRADE_PROGRAM;
static unsigned char tp_chip_type=0;

int fts_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			  u32 dw_lenth);
extern void ft6x06_ts_updata_version(void);
extern void ft6x06_reset_tp(int HighOrLow);

static unsigned char * CTPM_FW=NULL;
static int  CTPM_FW_LEN=0;

static struct mutex g_device_mutex;
static char projectcode[33]; 
static struct i2c_client * tp_client=NULL;

void  fts_ctpm_set_fw_infor(unsigned int fw_len,unsigned char *fw)
{
   
       if(fw_len && fw)
       {
           CTPM_FW = fw;
           CTPM_FW_LEN = fw_len;
	    printk(KERN_INFO "%s: fw_len = %d; CTPM_FW = 0x%x\n",__func__,CTPM_FW_LEN,CTPM_FW);
       }
	return ;
}

int ft6x06_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return ft6x06_i2c_Write(client, buf, sizeof(buf));
}


int ft6x06_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return ft6x06_i2c_Read(client, &regaddr, 1, regvalue, 1);
}


int fts_ctpm_auto_clb(struct i2c_client *client)
{
	unsigned char uc_temp = 0x00;
	unsigned char i = 0;

	/*start auto CLB */
	msleep(200);

	ft6x06_write_reg(client, 0, FTS_FACTORYMODE_VALUE);
	/*make sure already enter factory mode */
	msleep(100);
	/*write command to start calibration */
	ft6x06_write_reg(client, 2, 0x4);
	msleep(300);
	if (tp_chip_type == FT5x36_UPGRADE_ID_2) {
		for(i=0;i<100;i++)
		{
			ft6x06_read_reg(client, 0x02, &uc_temp);
			if (0x02 == uc_temp ||
				0xFF == uc_temp)
			{
				/*if 0x02, then auto clb ok, else 0xff, auto clb failure*/
			    break;
			}
			msleep(20);	    
		}
	} else {
		for(i=0;i<100;i++)
		{
			ft6x06_read_reg(client, 0, &uc_temp);
			if (0x0 == ((uc_temp&0x70)>>4))  /*return to normal mode, calibration finish*/
			{
			    break;
			}
			msleep(20);	    
		}
	}

	//msleep(200);
	/*calibration OK */
	msleep(300);
	ft6x06_write_reg(client, 0, FTS_FACTORYMODE_VALUE);	/*goto factory mode for store */
	msleep(100);	/*make sure already enter factory mode */
	ft6x06_write_reg(client, 2, 0x5);	/*store CLB result */
	msleep(300);
	ft6x06_write_reg(client, 0, FTS_WORKMODE_VALUE);	/*return to normal mode */
	msleep(300);

	/*store CLB result OK */
	return 0;
}

/*
upgrade with *.i file
*/
int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client)
{
	u8 *pbt_buf = NULL;
	int i_ret;
	int fw_len = CTPM_FW_LEN;

	/*judge the fw that will be upgraded
	* if illegal, then stop upgrade and return.
	*/
       printk(KERN_INFO "%s: fw_len = 0x%x\n",__func__,fw_len);
	if (fw_len < 8 || fw_len > 32 * 1024 || CTPM_FW==NULL) {
		dev_err(&client->dev, "%s:FW length error\n", __func__);
		return -EIO;
	}

	if (((CTPM_FW[fw_len - 8] ^ CTPM_FW[fw_len - 6]) == 0xFF
		&& (CTPM_FW[fw_len - 7] ^ CTPM_FW[fw_len - 5]) == 0xFF
		&& (CTPM_FW[fw_len - 3] ^ CTPM_FW[fw_len - 4]) == 0xFF) 
		|| (tp_chip_type==FT6x36_UPGRADE_ID_2) ) {
		/*FW upgrade */
		pbt_buf = CTPM_FW;
		/*call the upgrade function */
		i_ret = fts_ctpm_fw_upgrade(client, pbt_buf, CTPM_FW_LEN);
		if (i_ret != 0)
			dev_err(&client->dev, "%s:upgrade failed. err.\n",
					__func__);
#ifdef AUTO_CLB
		else
			fts_ctpm_auto_clb(client);	/*start auto CLB */
#endif
		ft6x06_reset_tp(0);
		mdelay(10);
		ft6x06_reset_tp(1);

	} else {
		dev_err(&client->dev, "%s:FW format error\n", __func__);
		return -EBADFD;
	}

	return i_ret;
}

u8 fts_ctpm_get_i_file_ver(void)
{
	u16 ui_sz;
	ui_sz = CTPM_FW_LEN;
	if (ui_sz > 2 &&  CTPM_FW!=NULL)
	{
		if (tp_chip_type==FT6x36_UPGRADE_ID_2)
			return CTPM_FW[0x10a];
		else
			return CTPM_FW[ui_sz - 2];
	}

	return 0x00;	/*default value */
}

static int  fts_ctpm_get_file_hw_ver(char *buf,int size)
{
    unsigned int ui_sz;
    ui_sz =size;
    if (ui_sz > 2)
    {
        return buf[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return -1; //default value
    }
}

int fts_ctpm_auto_upgrade(struct i2c_client *client, unsigned char force_updata)
{
	u8 uc_host_fm_ver = FT6x06_REG_FW_VER;
	u8 uc_tp_fm_ver;
	int i_ret;
	
	ft6x06_read_reg(client, FT6x06_REG_FW_VER, &uc_tp_fm_ver);
	uc_host_fm_ver = fts_ctpm_get_i_file_ver();
       printk(KERN_INFO "%s: host_ver = 0x%x ; tp_ver = 0x%x; force_updata =%d\n",__func__,uc_host_fm_ver,uc_tp_fm_ver,force_updata);
//	uc_tp_fm_ver = FT6x06_REG_FW_VER;
	if (/*the firmware in touch panel maybe corrupted */
		force_updata || (uc_tp_fm_ver == FT6x06_REG_FW_VER) ||
		/*the firmware in host flash is new, need upgrade */
	     (uc_tp_fm_ver < uc_host_fm_ver)
	    ) {
		msleep(100);
		dev_dbg(&client->dev, "[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
				uc_tp_fm_ver, uc_host_fm_ver);
		i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
		if (i_ret == 0)	{
			msleep(300);
			uc_host_fm_ver = fts_ctpm_get_i_file_ver();
			dev_dbg(&client->dev, "[FTS] upgrade to new version 0x%x\n",
					uc_host_fm_ver);
		} else {
			pr_err("[FTS] upgrade failed ret=%d.\n", i_ret);
			return -EIO;
		}
	}

	return 0;
}

/*
*get upgrade information depend on the ic type
*/
static void fts_get_upgrade_info(struct Upgrade_Info *upgrade_info)
{
	switch (tp_chip_type) {
	case FT5x06_UPGRADE_ID_2:
		upgrade_info->delay_55 = FT5x06_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5x06_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5x06_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5x06_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5x06_UPGRADE_READID_DELAY;
		upgrade_info->delay_earse_flash = FT5x06_UPGRADE_EARSE_DELAY;
		break;
	case FT5606_UPGRADE_ID_2:
		upgrade_info->delay_55 = FT5606_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5606_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5606_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5606_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5606_UPGRADE_READID_DELAY;
		upgrade_info->delay_earse_flash = FT5606_UPGRADE_EARSE_DELAY;
		break;
	case FT5316_UPGRADE_ID_2:
		upgrade_info->delay_55 = FT5316_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5316_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5316_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5316_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5316_UPGRADE_READID_DELAY;
		upgrade_info->delay_earse_flash = FT5316_UPGRADE_EARSE_DELAY;
		break;
	case FT6208_UPGRADE_ID_2:
		upgrade_info->delay_55 = FT6208_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT6208_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT6208_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT6208_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT6208_UPGRADE_READID_DELAY;
		upgrade_info->delay_earse_flash = FT6208_UPGRADE_EARSE_DELAY;
		break;
	case FT6x06_UPGRADE_ID_2:
		upgrade_info->delay_55 = FT6x06_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT6x06_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT6x06_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT6x06_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT6x06_UPGRADE_READID_DELAY;
		upgrade_info->delay_earse_flash = FT6x06_UPGRADE_EARSE_DELAY;
		break;
	case FT5x36_UPGRADE_ID_2:
		upgrade_info->delay_55 = FT5x36_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5x36_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5x36_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5x36_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5x36_UPGRADE_READID_DELAY;
		upgrade_info->delay_earse_flash = FT5x36_UPGRADE_EARSE_DELAY;
		break;
	default:
		break;
	}
}

int  fts_ctpm_get_fw_pid( struct i2c_client *client, unsigned char *vendor_id, unsigned char *product_name, int * panel_ic_id ,unsigned char *force_updata)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 j;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	int i_ret;
	u8 is_5336_new_bootloader = 0;
	//struct Upgrade_Info upgradeinfo;

	//fts_get_upgrade_info(&upgradeinfo);


       printk(KERN_INFO "%s: \n",__func__);
	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		ft6x06_reset_tp(0);
		mdelay(10);
		ft6x06_reset_tp(1);
		mdelay(i*10 + 30);
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		j=0;
		do {
			j++;
			i_ret = ft6x06_i2c_Write(client, auc_i2c_write_buf, 2);
			msleep(5);
		} while (i_ret <= 0 && j < 5);

		if (j>=5 ) printk("%s: Error!!!!!!!   j > 5\n",__func__);
		/*********Step 3:check READ-ID***********************/
		mdelay(10 /*upgradeinfo.delay_readid*/);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);


		if (reg_val[0] == 0x79 /*upgradeinfo.upgrade_id_1*/
			&& ((reg_val[1] == FT5x06_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */) /*FT5x06(x=2,3,4)*/
			||(reg_val[1] == FT6208_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */)    /*FT6208*/
			||(reg_val[1] == FT5606_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */)    /*FT5606*/
			||(reg_val[1] == FT5316_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */)    /*FT5316*/
			||(reg_val[1] == FT6x06_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */)    /*FT6306*/
			||(reg_val[1] == FT5x36_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */)    /*FT5336*/
			||(reg_val[1] == FT6x36_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */)    /*FT6x36*/
			)) {
			//dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				//reg_val[0], reg_val[1]);
			printk(KERN_INFO "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			*panel_ic_id= (reg_val[0]<<8) | reg_val[1];
			tp_chip_type=reg_val[1];
			break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3 error : CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
            mdelay(200);
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	auc_i2c_write_buf[0] = 0xcd;

	ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] <= 4)
		is_5336_new_bootloader = BL_VERSION_LZ4 ;
	else if(reg_val[0] == 7)
		is_5336_new_bootloader = BL_VERSION_Z7 ;	
	else if(reg_val[0] >= 0x0f)
		is_5336_new_bootloader = BL_VERSION_GZF ;
	printk(KERN_INFO "[FTS] bootloader version = 0x%x\n", reg_val[0]);
	
	/* --------- read current project setting  ---------- */
	//set read start address
	auc_i2c_write_buf[0] = 0x3;
	auc_i2c_write_buf[1] = 0x0;
	if (((is_5336_new_bootloader == BL_VERSION_Z7 || is_5336_new_bootloader == BL_VERSION_GZF) && (tp_chip_type==FT5x36_UPGRADE_ID_2))||(tp_chip_type==FT6x36_UPGRADE_ID_2))
	{
		auc_i2c_write_buf[2] = 0x07;
		auc_i2c_write_buf[3] = 0xb0;
	}
	else
	{
		auc_i2c_write_buf[2] = 0x78;
		auc_i2c_write_buf[3] = 0x00;
	}
	ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, packet_buf, FTS_PACKET_LENGTH);
	for (i=0;i<32;i++)
	{
		if (packet_buf[0x20 + i] == '\0')
			break;
	}
	memset(projectcode, 0, sizeof(projectcode));
	if (i) memcpy(projectcode,packet_buf+0x20,i);
	
	for (i = 0; i < FTS_PID_NAME_LENGTH; i++)
	{
		if ( packet_buf[0x42 + i] == 0xFF) packet_buf[0x42 + i] = 0;
		product_name[i]=packet_buf[0x42 + i];
	}
	product_name[FTS_PID_NAME_LENGTH-1]=0;
	printk(KERN_INFO "[%s] old setting: uc_i2c_addr = 0x%x, uc_io_voltage = %d, uc_panel_factory_id = 0x%x, PID is %s\n",__func__,
		packet_buf[0],  packet_buf[2], packet_buf[4],product_name);
	if (*vendor_id == 0 )
	{
		*vendor_id = packet_buf[4];
		printk(KERN_INFO "[FTS] Reset vendor_id = 0x%x\n", *vendor_id);
	}
	else if ((packet_buf[4] !=0)&&(packet_buf[4] !=*vendor_id))
	{
		*vendor_id = packet_buf[4];
		*force_updata = 1;
		printk(KERN_INFO "[FTS] Reset vendor_id = 0x%x; Need Updata FW!!\n", *vendor_id);
	}

    /********* reset the FW ***********************/
	auc_i2c_write_buf[0] = 0x07;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);

    msleep(300);  //make sure CTP startup normally
    
    return 0;
}


int fts_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			  u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u8 is_5336_new_bootloader = 0;
	u8 is_5336_fwsize_30 = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	u32 fw_length;
	u32 driver_ic_id;
	//struct Upgrade_Info upgradeinfo;

	//fts_get_upgrade_info(&upgradeinfo);
	printk(KERN_INFO "%s: \n",__func__);
	
	if(dw_lenth > 0x11f)
	{
		fw_length = ((u32)pbt_buf[0x100]<<8) + pbt_buf[0x101];
		if(dw_lenth < fw_length)
		{
			printk(KERN_ERR "[FTS] Fw length is invalid \n");
			return -1;
		}
	}
	else
	{
		printk(KERN_ERR "[FTS] Fw length is invalid \n");
		return -1;
	}

	if(pbt_buf[dw_lenth-12] == 30)
	{
		is_5336_fwsize_30 = 1;
	}
	else 
	{
		is_5336_fwsize_30 = 0;
	}
	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		ft6x06_reset_tp(0);
		mdelay(10);
		ft6x06_reset_tp(1);

		mdelay(i*10 + 30);
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		j=0;
		do {
			j++;
			i_ret = ft6x06_i2c_Write(client, auc_i2c_write_buf, 2);
			msleep(5);
		} while (i_ret <= 0 && j < 5);

		if (j>=5 ) printk("%s: Error!!!!!!!   j > 5\n",__func__);
		/*********Step 3:check READ-ID***********************/
		mdelay(10 /*upgradeinfo.delay_readid*/);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);


		if (reg_val[0] == 0x79 /*upgradeinfo.upgrade_id_1*/
			&& ((reg_val[1] == FT5x06_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */) /*FT5x06(x=2,3,4)*/
			||(reg_val[1] == FT6208_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */)    /*FT6208*/
			||(reg_val[1] == FT5606_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */)    /*FT5606*/
			||(reg_val[1] == FT5316_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */)    /*FT5316*/
			||(reg_val[1] == FT6x06_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */)    /*FT6306*/
			||(reg_val[1] == FT5x36_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */)    /*FT5336*/
			||(reg_val[1] == FT6x36_UPGRADE_ID_2/* upgradeinfo.upgrade_id_2 */)    /*FT6x36*/
			)) {
			//dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				//reg_val[0], reg_val[1]);
			driver_ic_id=(reg_val[0]<<8)|reg_val[1];
			printk(KERN_INFO "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x.driver_ic_id=0x%x\n",
				reg_val[0], reg_val[1],driver_ic_id);
			break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3 error : CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
            mdelay(200);
		}
	}
	if (i > FTS_UPGRADE_LOOP)
		return -EIO;
	auc_i2c_write_buf[0] = 0xcd;

	ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
    	printk(KERN_INFO "[FTS] bootloader version = 0x%x\n", reg_val[0]);


	/*Step 4:erase app and panel paramenter area*/
	if (reg_val[0] <= 4)
	{
		is_5336_new_bootloader = BL_VERSION_LZ4 ;
	}
	else if(reg_val[0] == 7)
	{
		is_5336_new_bootloader = BL_VERSION_Z7 ;
	}
	else if(reg_val[0] >= 0x0f)
	{
		is_5336_new_bootloader = BL_VERSION_GZF ;
	}

	printk(KERN_INFO "Step 4:erase app and panel paramenter area\n");
	if(is_5336_fwsize_30)
	{
		auc_i2c_write_buf[0] = 0x61;
		ft6x06_i2c_Write(client, auc_i2c_write_buf, 1); /*erase app area*/	
   		 msleep(2000); 

		 auc_i2c_write_buf[0] = 0x63;
		ft6x06_i2c_Write(client, auc_i2c_write_buf, 1); /*erase app area*/	
   		 msleep(100);
	}
	else
	{
		auc_i2c_write_buf[0] = 0x61;
		ft6x06_i2c_Write(client, auc_i2c_write_buf, 1); /*erase app area*/	
   		msleep(2000); 
	}

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	printk(KERN_INFO "Step 5:write firmware(FW) to ctpm flash\n");

	if(driver_ic_id == FT6x36_UPGRADE_ID)
	{
		dw_lenth = fw_length;
		printk(KERN_INFO "[kernel]:%s,test33.\n",__func__);
	}
	else if(is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 || (tp_chip_type!=FT5x36_UPGRADE_ID_2))
	{
		dw_lenth = dw_lenth - 8;
	}
	else if(is_5336_new_bootloader == BL_VERSION_GZF)
	{
		dw_lenth = dw_lenth - 14;
	}
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		
		ft6x06_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		msleep(FTS_PACKET_LENGTH / 6 + 1);
		//DBG("write bytes:0x%04x\n", (j+1) * FTS_PACKET_LENGTH);
		//delay_qt_ms(FTS_PACKET_LENGTH / 6 + 1);
		if(driver_ic_id == FT6x36_UPGRADE_ID)
		{
			for(i = 0;i < 30;i++)
			{
				auc_i2c_write_buf[0] = 0x6a;
				auc_i2c_write_buf[1] = 0x00;
				auc_i2c_write_buf[2] = 0x00;
				auc_i2c_write_buf[3] = 0x00;
				reg_val[0] = 0x00;
				reg_val[1] = 0x00;
				ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
				if(0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
				{
					//DBG("[FTS] write a block data finished \n");
					break;
				}
				msleep(1);
			}
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		ft6x06_i2c_Write(client, packet_buf, temp + 6);
		msleep(20);
		if(driver_ic_id == FT6x36_UPGRADE_ID)
		{
			for(i = 0;i < 30;i++)
			{
				auc_i2c_write_buf[0] = 0x6a;
				auc_i2c_write_buf[1] = 0x00;
				auc_i2c_write_buf[2] = 0x00;
				auc_i2c_write_buf[3] = 0x00;
				reg_val[0] = 0x00;
				reg_val[1] = 0x00;
				ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
				if(0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
				{
					//DBG("[FTS] write a block data finished \n");
					break;
				}
				msleep(1);
			}
		}
	}

	/*send the last six byte*/
	if(driver_ic_id != FT6x36_UPGRADE_ID)
	{
		if(is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7  || (tp_chip_type!=FT5x36_UPGRADE_ID_2 ))
		{
			for (i = 0; i<6; i++)
			{
				if (is_5336_new_bootloader  == BL_VERSION_Z7 && (tp_chip_type==FT5x36_UPGRADE_ID_2)) 
				{
					temp = 0x7bfa + i;
				}
				else //if(is_5336_new_bootloader == BL_VERSION_LZ4 || (tp_chip_type!=FT5x36_UPGRADE_ID_2))
				{
					temp = 0x6ffa + i;
				}
				//printk( "%s:Last 6 byte in 0x%x\n",__func__,temp);
				packet_buf[2] = (u8)(temp>>8);
				packet_buf[3] = (u8)temp;
				temp =1;
				packet_buf[4] = (u8)(temp>>8);
				packet_buf[5] = (u8)temp;
				packet_buf[6] = pbt_buf[ dw_lenth + i]; 
				bt_ecc ^= packet_buf[6];
				
				ft6x06_i2c_Write(client, packet_buf, 7);
				msleep(10);
			}
		}
		else if(is_5336_new_bootloader == BL_VERSION_GZF)
		{
			for (i = 0; i<12; i++)
			{
				if (is_5336_fwsize_30 && (tp_chip_type==FT5x36_UPGRADE_ID_2)) 
				{
					temp = 0x7ff4 + i;
				}
				else if (tp_chip_type==FT5x36_UPGRADE_ID_2) 
				{
					temp = 0x7bf4 + i;
				}
				//printk( "%s:Last 6 byte in 0x%x\n",__func__,temp);
				packet_buf[2] = (u8)(temp>>8);
				packet_buf[3] = (u8)temp;
				temp =1;
				packet_buf[4] = (u8)(temp>>8);
				packet_buf[5] = (u8)temp;
				packet_buf[6] = pbt_buf[ dw_lenth + i]; 
				bt_ecc ^= packet_buf[6];

				ft6x06_i2c_Write(client, packet_buf, 7);
				msleep(10);

			}
		}
	}
	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	printk(KERN_INFO "Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	printk(KERN_INFO "Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);	/*make sure CTP startup normally */

	return 0;
}

/*sysfs debug*/

/*
*get firmware size

@firmware_name:firmware name
*note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int ft6x06_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "%s", firmware_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}


/*
*read firmware buf for .bin file.

@firmware_name: fireware name
@firmware_buf: data buf of fireware

note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int ft6x06_ReadFirmware(char *firmware_name,
			       unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, firmware_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}



/*
upgrade with *.bin file
*/

int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client,
				       char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret; 
	u8 fwver;
	int fwsize = ft6x06_GetFirmwareSize(firmware_name);
	u8 uc_tp_fm_ver;

	if (fwsize <= 0) {
		dev_err(&client->dev, "%s ERROR:Get firmware size failed\n",
					__func__);
               printk(KERN_ERR "at %s line %d ERROR:Get firmware size failed\n",__func__,__LINE__);
		fw_upgrade_state=FW_UPGDRADE_FILE_ERR;
		return -2;
	}

	if (fwsize < 8 || fwsize > 32 * 1024) {
		dev_dbg(&client->dev, "%s:FW length error\n", __func__);
		fw_upgrade_state=FW_UPGDRADE_FILE_ERR;
		return -2;
	}

	/*=========FW upgrade========================*/
	pbt_buf = kmalloc(fwsize + 1, GFP_ATOMIC);

	if (ft6x06_ReadFirmware(firmware_name, pbt_buf)) {
		dev_err(&client->dev, "%s() - ERROR: request_firmware failed\n",
					__func__);
		kfree(pbt_buf);
                printk(KERN_ERR "at %s line %d ERROR: request_firmware failed\n",__func__,__LINE__);
		fw_upgrade_state=FW_UPGDRADE_FILE_READ_ERR;
		return -3;
	}
	
        fwver=fts_ctpm_get_file_hw_ver(pbt_buf,fwsize);
        if(fwver<= 0)
        {
             printk(KERN_ERR "at %s line %d ERROR: firmware wrong  fwver=0x%x\n",__func__,__LINE__,\
                      fwver);
		fw_upgrade_state=FW_UPGDRADE_FILE_VER_ERR;
		kfree(pbt_buf);
             return -4;
        }

	ft6x06_read_reg(client, FT6x06_REG_FW_VER, &uc_tp_fm_ver);

        if(uc_tp_fm_ver > fwver)
        {
             printk(KERN_ERR "at %s line %d ERROR: firmware wrong  fwver=0x%x curent_ver=0x%x\n",__func__,__LINE__,\
                      fwver,uc_tp_fm_ver);
		fw_upgrade_state=FW_UPGDRADE_LATEST;
		kfree(pbt_buf);
             return -5;
        }
		
	/*call the upgrade function */
	if ((pbt_buf[fwsize - 8] ^ pbt_buf[fwsize - 6]) == 0xFF
		&& (pbt_buf[fwsize - 7] ^ pbt_buf[fwsize - 5]) == 0xFF
		&& (pbt_buf[fwsize - 3] ^ pbt_buf[fwsize - 4]) == 0xFF) {
		/*call the upgrade function */
		fw_upgrade_state=FW_UPGDRADE_PROGRAM;
	
		/*call the upgrade function */
		i_ret = fts_ctpm_fw_upgrade(client, pbt_buf, fwsize);
		if (i_ret != 0)
		{
			dev_err(&client->dev, "%s() - ERROR:[FTS] upgrade failed..\n",
					__func__);
			printk(KERN_ERR "at %s line %d ERROR: xxxxxxxx\n",__func__,__LINE__);
			fw_upgrade_state=FW_UPGDRADE_FAIL;
			kfree(pbt_buf);
			return -6;
		}
		else
			fts_ctpm_auto_clb(client);
	} else {
		dev_dbg(&client->dev, "%s:FW format error\n", __func__);
		printk(KERN_ERR "at %s line %d check buffer ERROR: xxxxxxxx\n",__func__,__LINE__);
			fw_upgrade_state=FW_UPGDRADE_FAIL;
		kfree(pbt_buf);
		return -6;
	}
	
	ft6x06_ts_updata_version();

	kfree(pbt_buf);
	fw_upgrade_state=FW_UPGDRADE_SUCCESS;
   	return 0;
}

static ssize_t ft6x06_tpfwver_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	u8 fwver = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	mutex_lock(&g_device_mutex);

	if (ft6x06_read_reg(client, FT6x06_REG_FW_VER, &fwver) < 0)
		num_read_chars = sprintf(buf,
					"get tp fw version fail!\n");
	else
		num_read_chars = sprintf(buf, "%02X\n", fwver);

	mutex_unlock(&g_device_mutex);

	return num_read_chars;
}

static ssize_t ft6x06_tpfwver_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}


static ssize_t ft6x06_tprwreg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t ft6x06_tprwreg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg = 0;
	u8 regaddr = 0xff, regvalue = 0xff;
	u8 valbuf[5] = {0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&g_device_mutex);
	num_read_chars = count - 1;

	if (num_read_chars != 2) {
		if (num_read_chars != 4) {
			pr_info("please input 2 or 4 character\n");
			goto error_return;
		}
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);

	if (0 != retval) {
		dev_err(&client->dev, "%s() - ERROR: Could not convert the "\
						"given input to a number." \
						"The given input was: \"%s\"\n",
						__func__, buf);
		goto error_return;
	}

	if (2 == num_read_chars) {
		/*read register*/
		regaddr = wmreg;
		if (ft6x06_read_reg(client, regaddr, &regvalue) < 0)
			dev_err(&client->dev, "Could not read the register(0x%02x)\n",
						regaddr);
		else
			pr_info("the register(0x%02x) is 0x%02x\n",
					regaddr, regvalue);
	} else {
		regaddr = wmreg >> 8;
		regvalue = wmreg;
		if (ft6x06_write_reg(client, regaddr, regvalue) < 0)
			dev_err(&client->dev, "Could not write the register(0x%02x)\n",
							regaddr);
		else
			dev_err(&client->dev, "Write 0x%02x into register(0x%02x) successful\n",
							regvalue, regaddr);
	}

error_return:
	mutex_unlock(&g_device_mutex);

	return count;
}

static ssize_t ft6x06_fwupdate_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/*upgrade from *.i*/
static ssize_t ft6x06_fwupdate_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct ft6x06_ts_data *data = NULL;
	u8 uc_host_fm_ver;
	int i_ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	data = (struct ft6x06_ts_data *)i2c_get_clientdata(client);

	mutex_lock(&g_device_mutex);

	disable_irq(client->irq);
	i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
	if (i_ret == 0) {
		msleep(300);
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		pr_info("%s [FTS] upgrade to new version 0x%x\n", __func__,
					 uc_host_fm_ver);
	} else
		dev_err(&client->dev, "%s ERROR:[FTS] upgrade failed.\n",
					__func__);

	enable_irq(client->irq);
	mutex_unlock(&g_device_mutex);

	return count;
}

static ssize_t ft6x06_fwupgradeapp_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/* place holder for future use */
    switch(fw_upgrade_state)
    {
        case FW_UPGDRADE_FAIL:
             return sprintf(buf, "%s\0", "fail");
        case FW_UPGDRADE_PROGRAM:
             return sprintf(buf, "%s\0", "program");
        case FW_UPGDRADE_LATEST:
             return sprintf(buf, "%s\0", "latest");
        case FW_UPGDRADE_FILE_ERR:
             return sprintf(buf, "%s\0", "file error");
        case FW_UPGDRADE_FILE_READ_ERR:
             return sprintf(buf, "%s\0", "read file error");
        case FW_UPGDRADE_FILE_VER_ERR:
             return sprintf(buf, "%s\0", "file ver error , it`s 0");
        case FW_UPGDRADE_SUCCESS:
        default:
             return sprintf(buf, "%s\n", "success");
    }
    return 0;
}


/*upgrade from app.bin*/
static ssize_t ft6x06_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	int ret;

	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count - 1] = '\0';

       printk(KERN_INFO "at %s line %d file_name=%s\n",__func__,__LINE__,fwname);
	mutex_lock(&g_device_mutex);
	disable_irq(client->irq);

	ret = fts_ctpm_fw_upgrade_with_app_file(client, fwname);

	enable_irq(client->irq);
	mutex_unlock(&g_device_mutex);

	if (ret != 0 ) return ret; 
	return count;
}

static ssize_t ft6x06_ftsgetprojectcode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

//	memset(projectcode, 0, sizeof(projectcode));
	mutex_lock(&g_device_mutex);
//	if(ft6x06_read_project_code(client, projectcode) < 0)
//		num_read_chars = snprintf(buf, PAGE_SIZE, "get projcet code fail!\n");
//	else
		num_read_chars = sprintf(buf, "projcet code = %s\n", projectcode);

	mutex_unlock(&g_device_mutex);
	return num_read_chars;

	
}
//upgrade from app.bin
static ssize_t ft6x06_ftsgetprojectcode_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	/* place holder for future use */
    return -EPERM;
}


/* sysfs */
/*get the fw version
*example:cat ftstpfwver
*/
static DEVICE_ATTR(ftstpfwver, S_IRUGO | S_IWUSR, ft6x06_tpfwver_show,
			ft6x06_tpfwver_store);

/*upgrade from *.i
*example: echo 1 > ftsfwupdate
*/
static DEVICE_ATTR(ftsfwupdate, S_IRUGO | S_IWUSR, ft6x06_fwupdate_show,
			ft6x06_fwupdate_store);

/*read and write register
*read example: echo 88 > ftstprwreg ---read register 0x88
*write example:echo 8807 > ftstprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(ftstprwreg, S_IRUGO | S_IWUSR, ft6x06_tprwreg_show,
			ft6x06_tprwreg_store);


/*upgrade from app.bin
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(upgrade_file_on, 0664, ft6x06_fwupgradeapp_show, ft6x06_fwupgradeapp_store);

/*show project code
*example:cat ftsgetprojectcode
*/
static DEVICE_ATTR(ftsgetprojectcode, S_IRUGO|S_IWUSR, ft6x06_ftsgetprojectcode_show, ft6x06_ftsgetprojectcode_store);


/*add your attr in here*/
static struct attribute *ft6x06_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_upgrade_file_on.attr,
	&dev_attr_ftsgetprojectcode.attr,
	NULL
};

static struct attribute_group ft6x06_attribute_group = {
	.attrs = ft6x06_attributes
};

/*create sysfs for debug*/
int ft6x06_create_sysfs(struct i2c_client *client)
{
	int err;
	err = sysfs_create_group(&client->dev.kobj, &ft6x06_attribute_group);
	if (0 != err) {
		dev_err(&client->dev,
					 "%s() - ERROR: sysfs_create_group() failed.\n",
					 __func__);
		sysfs_remove_group(&client->dev.kobj, &ft6x06_attribute_group);
		return -EIO;
	} else {
		mutex_init(&g_device_mutex);
		pr_info("ft6x06:%s() - sysfs_create_group() succeeded.\n",
				__func__);
	}
	return err;
}

void ft6x06_release_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &ft6x06_attribute_group);
	mutex_destroy(&g_device_mutex);
}


/*create apk debug channel*/

#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER		2
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA			6
#define PROC_READ_DATA			7

#define PROC_NAME	"ft5x0x-debug"
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *ft6x06_proc_entry;
/*interface of write proc*/
static int ft6x06_debug_write(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)
{
	struct i2c_client *client = tp_client;
	unsigned char writebuf[FTS_PACKET_LENGTH];
	int buflen = len;
	int writelen = 0;
	int ret = 0;
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			DBG("%s\n", upgrade_file_path);
			disable_irq(client->irq);

			ret = fts_ctpm_fw_upgrade_with_app_file(client, upgrade_file_path);

			enable_irq(client->irq);
			if (ret < 0) {
				dev_err(&client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		DBG("%s:register addr=0x%02x\n", __func__, writebuf[1]);
		ret = ft6x06_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = ft6x06_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = len - 1;
		ret = ft6x06_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	

	return len;
}

/*interface of read proc*/
static int ft6x06_debug_read(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)  //( char *page, char **start,off_t off, int count, int *eof, void *data )
{
	struct i2c_client *client = tp_client;
	int ret = 0, err = 0;
	//u8 tx = 0, rx = 0;
	//int i, j;
	unsigned char buf[PAGE_SIZE];
	int num_read_chars = 0;
	int readlen = 0, writelen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		/*after calling ft6x06_debug_write to upgrade*/
		regaddr = 0xA6;
		ret = ft6x06_read_reg(client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = ft6x06_i2c_Read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		} else
			DBG("%s:value=0x%02x\n", __func__, buf[0]);
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = len;
		ret = ft6x06_i2c_Read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	//memcpy(page, buf, num_read_chars);
	copy_to_user(buff, buf, num_read_chars);

	return num_read_chars;
}

static const struct file_operations proc_fops = {
	.owner = THIS_MODULE,
	.read = ft6x06_debug_read,
	.write = ft6x06_debug_write,
};


int ft6x06_create_apk_debug_channel(struct i2c_client * client)
{
	ft6x06_proc_entry = proc_create_data(PROC_NAME, 0666, NULL,&proc_fops, NULL);
	if (NULL == ft6x06_proc_entry) {
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		return -ENOMEM;
	} else {
		dev_info(&client->dev, "Create proc entry success!\n");
		tp_client = client;
	//	ft6x06_proc_entry->write_proc = ft6x06_debug_write;
	//	ft6x06_proc_entry->read_proc = ft6x06_debug_read;
	}
	return 0;
}

void ft6x06_release_apk_debug_channel(void)
{
	if (ft6x06_proc_entry)
		remove_proc_entry(PROC_NAME, NULL);
}

