#ifndef __SPRDFB_DEV_H_
#define __SPRDFB_DEV_H_

enum {
	DSI_CMD_DCS = 0,
	DSI_CMD_GEN,
	DSI_CMD_DEALY,
	DSI_RD_CMD_DCS,
	DSI_CMD_UDEALY,//add by liuwei
	DSI_CMD_END=0xFF,
};

typedef  struct{
	u8 type;
	u8 len;
	u8 cmd[4];
	u8 *p_cmd;
}sprdfb_dev_dsi_cmds;

enum {
	GEN_SW_0P_TX = 1,
	GEN_SW_1P_TX,
	GEN_SW_2P_TX,
	GEN_SR_0P_TX,
	GEN_SR_1P_TX,
	GEN_SR_2P_TX,
	DCS_SW_0P_TX,
	DCS_SW_1P_TX,
	DCS_SR_0P_TX,
	MAX_RD_PKT_SIZE,
	GEN_LW_TX,
	DCS_LW_TX,
};
#endif
