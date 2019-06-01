#ifndef __ITM_CMD_H__
#define __ITM_CMD_H__

#include <linux/ieee80211.h>
#include "wlan_common.h"
#include "wlan_fifo.h"

#define ITM_PMKID_LEN     16

enum ITM_HOST_TROUT3_CMD_TYPE
{
	HOST_SC2331_CMD = 0,
	SC2331_HOST_RSP,
	HOST_SC2331_PKT,
	HOST_SC2331_WAPI,
};

enum ITM_HOST_TROUT3_CMD_LIST
{
	WIFI_CMD_GET_MODE = 1,
	WIFI_CMD_GET_RSSI,
	WIFI_CMD_GET_TXRATE_TXFAILED,
	WIFI_CMD_SET_SCAN,
	WIFI_CMD_SET_AUTH_TYPE,
	WIFI_CMD_SET_WPA_VERSION,
	WIFI_CMD_SET_PAIRWISE_CIPHER,
	WIFI_CMD_SET_GROUP_CIPHER,
	WIFI_CMD_SET_AKM_SUITE,
	WIFI_CMD_SET_CHANNEL,//10-0xA
	WIFI_CMD_SET_BSSID,
	WIFI_CMD_SET_ESSID,
	WIFI_CMD_KEY_ADD,
	WIFI_CMD_KEY_DEL,
	WIFI_CMD_KEY_SET,
	WIFI_CMD_SET_DISCONNECT,
	WIFI_CMD_SET_RTS_THRESHOLD,
	WIFI_CMD_SET_FRAG_THRESHOLD,
	WIFI_CMD_SET_PMKSA,
	WIFI_CMD_DEL_PMKSA,//20--0x14
	WIFI_CMD_FLUSH_PMKSA,
	WIFI_CMD_SET_DEV_OPEN,
	WIFI_CMD_SET_DEV_CLOSE,
	WIFI_CMD_SET_PSK,
	WIFI_CMD_START_BEACON,
	WIFI_CMD_SET_WPS_IE,
	WIFI_CMD_TX_MGMT,
	WIFI_CMD_REMAIN_CHAN,
	WIFI_CMD_CANCEL_REMAIN_CHAN,
	WIFI_CMD_P2P_IE,//30---0x1e
	WIFI_CMD_CHANGE_BEACON,
	WIFI_CMD_REGISTER_FRAME,
	WIFI_CMD_NPI_MSG,
	WIFI_CMD_NPI_GET,
	WIFI_CMD_SET_FT_IE,
	WIFI_CMD_UPDATE_FT_IE,
	WIFI_CMD_ASSERT,
	WIFI_CMD_SLEEP,
	WIFI_CMD_ADD_SOFTAP_BLACKLIST,
	WIFI_CMD_DEL_SOFTAP_BLACKLIST,
	WIFI_CMD_SCAN_NOR_CHANNELS,
	WIFI_CMD_GET_IP,
	WIFI_CMD_REQ_LTE_CONCUR,
	WIFI_CMD_SET_CQM_RSSI,
	WIFI_CMD_MAX,
	
	
	WIFI_EVENT_CONNECT = 128,
	WIFI_EVENT_DISCONNECT,
	WIFI_EVENT_SCANDONE,
	WIFI_EVENT_MGMT_DEAUTH,
	WIFI_EVENT_MGMT_DISASSOC,
	WIFI_EVENT_REMAIN_ON_CHAN_EXPIRED,
	WIFI_EVENT_NEW_STATION,
	WIFI_EVENT_REPORT_FRAME,
	WIFI_EVENT_CONNECT_AP,
	WIFI_EVENT_SDIO_SEQ_NUM,
	WIFI_EVENT_REPORT_SCAN_FRAME,
	WIFI_EVENT_REPORT_MIC_FAIL,
	WIFI_EVENT_REPORT_CQM_RSSI_LOW,
	WIFI_EVENT_REPORT_CQM_RSSI_HIGH,
	WIFI_EVENT_REPORT_CQM_RSSI_LOSS_BEACON,
	WIFI_EVENT_MAX,
};

/* The reason code is defined by CP2 */
enum wlan_cmd_disconnect_reason
{
	AP_LEAVING = 0xc1,
	AP_DEAUTH = 0xc4,
};

struct wlan_cmd_add_key
{
	unsigned char mac[6];
	unsigned char keyseq[8];
	unsigned char pairwise;
	unsigned char cypher_type;
	unsigned char key_index;
	unsigned char key_len;
	unsigned char value[0];
} __packed;

struct wlan_cmd_del_key
{
	unsigned char key_index;
	unsigned char pairwise;		/* unicase or group */
	unsigned char mac[6];
} __packed;

struct wlan_cmd_pmkid
{
	unsigned char bssid[ETH_ALEN];
	unsigned char pmkid[ITM_PMKID_LEN];
} __packed;

struct wlan_cmd_cqm_rssi {
	s32 rssih;
	u32 rssil;
} __packed;

struct wlan_cmd_beacon
{
	unsigned char len;
	unsigned char value[0];
} __packed;

struct wlan_cmd_mac_open
{
	__le16 mode;	/* AP or STATION mode */
	unsigned char mac[6];
} __packed;

struct wlan_cmd_mac_close
{
	unsigned char mode;	/* AP or STATION mode */
} __packed;

struct wlan_cmd_wps_ie
{
	unsigned char type;	/* probe req ie or assoc req ie */
	unsigned char len;		/* max ie len is 255 */
	unsigned char value[0];
} __packed;

struct wlan_cmd_scan_ssid
{
	unsigned char len;
	unsigned char ssid[0];
} __packed;

struct wlan_cmd_set_frag
{
	__le16 frag;
} __packed;

struct wlan_cmd_set_rts
{
	__le16 threshold;
} __packed;

struct wlan_cmd_set_key
{
	__le32 key_index;
} __packed;

struct wlan_cmd_disconnect
{
	__le16 reason_code;
} __packed;

struct wlan_cmd_set_essid
{
	__le16 len;
	unsigned char essid[0];
} __packed;

struct wlan_cmd_set_bssid
{
	unsigned char addr[6];
} __packed;

struct wlan_cmd_get_ip
{
	unsigned char ip[4];
} __packed;

struct wlan_cmd_set_channel
{
	__le32 channel;
} __packed;

struct wlan_cmd_set_psk
{
	__le16 len;
	unsigned char key[0];
} __packed;

struct wlan_cmd_set_key_management
{
	__le32 key_mgmt;
} __packed;

struct wlan_cmd_set_cipher
{
	__le32 cipher;
} __packed;

struct wlan_cmd_set_auth_type
{
	__le32 type;
} __packed;

struct wlan_cmd_set_wpa_version
{
	__le32 wpa_version;
} __packed;

struct wlan_cmd_assert_t
{
	__le32 reason_code;
	__le32 tx_cnt;
	__le32 rx_cnt;
} __packed;

struct wlan_cmd_scan
{
	__le32 len;
	unsigned char ssid[0];
} __packed;

struct wlan_cmd_get_txrate_txfailed
{
	__le32 rate;
	__le32 failed;
} __packed;

struct wlan_cmd_get_device_mode
{
	__le32 mode;
} __packed;

struct wlan_cmd_rsp_state_code
{
	__le32 code;
} __packed;

struct wlan_cmd_remain_chan_t {
	unsigned char chan;		/* send channel */
	unsigned char chan_type;
	__le32 duraion;
	__le64 cookie;/* cookie */
} __packed;

struct wlan_cmd_cancel_remain_chan_t {
	__le64 cookie;		/* cookie */
} __packed;

struct wlan_cmd_mgmt_tx_t {
	unsigned char chan;		/* send channel */
	__le32 wait;	/* wait time */
	__le32 len;		/* mac length*/
	unsigned char value[0];/* mac*/
} __packed;

/* wlan_sipc wps ie struct */
struct wlan_cmd_p2p_ie_t {
	unsigned char type;	/*  assoc req ie */
	__le16 len;		/* max ie len is 255 */
	unsigned char value[0];
} __packed;

struct wlan_cmd_register_frame_t {
	__le16 type;	/*  assoc req ie */
	unsigned char reg;    /* max ie len is 255 */
} __packed;

struct wlan_cmd_beacon_t {
	__le16 len;
	unsigned char value[0];
} __packed;

struct wlan_cmd_ft_ies_params
{
	__le16 md;
	__le16 ie_len;
	unsigned char  ie[0];
} __packed;

struct wlan_set_regdom_params
{
	__le32 len;
	unsigned char value[0];
} __packed;

struct wlan_event_report_frame_t {
    unsigned char channel;
    unsigned char frame_type;
	__le16 frame_len;
} __packed;

struct wlan_event_scan_rsp
{
	__le16 ops;
	__le16 channel;
	__le16 signal;
	__le16 frame_len;
} __packed;

struct wlan_event_mic_failure {
	unsigned char key_id;
	unsigned char is_mcast;
} __packed;

extern int wlan_cmd_send_recv(unsigned char vif_id, unsigned char *pData, int len, int type, int timeout);
extern int wlan_cmd_start_ap(unsigned char vif_id, unsigned char *beacon, unsigned short len);
extern int wlan_cmd_register_frame(unsigned char vif_id, struct  wlan_cmd_register_frame_t *data);
extern int wlan_cmd_set_p2p_ie(unsigned char vif_id, u8 type, const u8 *ie, u16 len);
extern int wlan_cmd_set_tx_mgmt(unsigned char vif_id, struct ieee80211_channel *channel, unsigned int wait, const u8 *mac, size_t mac_len);
extern int wlan_cmd_remain_chan(unsigned char vif_id, struct ieee80211_channel *channel,  enum nl80211_channel_type channel_type, unsigned int duration, u64 *cookie);
extern int wlan_cmd_cancel_remain_chan(unsigned char vif_id, u64 cookie);
extern int wlan_cmd_scan(unsigned char vif_id, const unsigned char *ssid, const unsigned char *channels, int len);;
extern int wlan_cmd_set_wpa_version(unsigned char vif_id,  unsigned int wpa_version);
extern int wlan_cmd_set_auth_type(unsigned char vif_id, unsigned int type);
extern int wlan_cmd_set_cipher(unsigned char vif_id, unsigned int cipher, unsigned char cmd_id);
extern int wlan_cmd_set_key_management(unsigned char vif_id, unsigned char key_mgmt);
extern int wlan_cmd_set_psk(unsigned char vif_id, const unsigned char *key, unsigned int key_len);
extern int wlan_cmd_set_channel(unsigned char vif_id, unsigned int channel);
extern int wlan_cmd_set_bssid(unsigned char vif_id, const unsigned char *addr);
extern int wlan_cmd_get_ip(unsigned char vif_id, u8 *ip);
extern int wlan_cmd_set_essid(unsigned char vif_id, const unsigned char *essid, int essid_len);
extern int wlan_cmd_pmksa(unsigned char vif_id, const unsigned char *bssid, const unsigned char *pmkid, unsigned char type);
extern int wlan_cmd_disconnect(unsigned char vif_id, unsigned short reason_code);
extern int wlan_cmd_add_key(unsigned char vif_id, const unsigned char *key_data, unsigned char key_len, unsigned char pairwise, unsigned char key_index, const unsigned char *key_seq, unsigned char cypher_type, const unsigned char *pmac);
extern int wlan_cmd_del_key(unsigned char vif_id, unsigned short key_index, const unsigned char *mac_addr);
extern int wlan_cmd_set_key(unsigned char vif_id, unsigned char key_index);
extern int wlan_cmd_set_rts(unsigned char vif_id, unsigned short rts_threshold);
extern int wlan_cmd_set_frag(unsigned char vif_id, unsigned short frag_threshold);
extern int wlan_cmd_set_wps_ie( unsigned char vif_id, unsigned char type, const unsigned char *ie, unsigned char len);
extern int
wlan_cmd_mac_open(unsigned char vif_id, unsigned short mode,
				  unsigned char *mac_addr);
extern int wlan_cmd_mac_close(unsigned char vif_id, unsigned char mode);
extern int wlan_cmd_get_rssi(unsigned char vif_id, unsigned char *signal, unsigned char *noise);
extern int wlan_cmd_get_txrate_txfailed(unsigned char vif_id, unsigned int *rate, unsigned int *failed);
extern int wlan_cmd_get_txrate(unsigned char vif_id, unsigned int *rate);
extern int wlan_rx_rsp_process(const unsigned char vif_id, r_msg_hdr_t *msg);
extern int wlan_rx_event_process(const unsigned char vif_id, unsigned char event, unsigned char *pData, unsigned short len);
extern int wlan_cmd_npi_send_recv(unsigned char *s_buf,unsigned short s_len, unsigned char *r_buf, unsigned short *r_len );
extern int wlan_cmd_init(void );
extern int wlan_cmd_deinit(void );
extern int wlan_cmd_set_ft_ie(unsigned char vif_id, const unsigned char *ies, unsigned short len);
extern int wlan_cmd_update_ft_ies( unsigned char vif_id, struct cfg80211_update_ft_ies_params *ft_ies);
extern int wlan_cmd_assert(unsigned char vif_id, unsigned int reason_code);
extern void cfg80211_report_scan_frame(unsigned char vif_id, unsigned char *pData, int len);
extern void cfg80211_report_mic_failure(unsigned char vif_id, unsigned char *pdata, int len);
extern int wlan_cmd_sleep(int ops);
extern int wlan_cmd_req_lte_concur(unsigned char vif_id, const unsigned char *val, int len);
extern int wlan_cmd_set_regdom(unsigned char vif_id, unsigned char *regdom, unsigned int len);
extern int wlan_cmd_cmq_rssi(unsigned char vif_id,
				s32 rssi_thold, u32 rssi_hyst,
				unsigned char type);
#endif

