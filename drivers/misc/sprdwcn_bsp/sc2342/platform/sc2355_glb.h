#define CP_START_ADDR		0x40500000
#define CP_RESET_REG		0x40088288
#define CP_SDIO_PRIORITY_ADDR 0x40130150

/* set sdio higher priority to visit iram */
#define M6_TO_S0_HIGH_PRIORITY 0X80000000

#define PACKET_SIZE		(32*1024)

/* time out in waiting wifi to come up */
#define POWERUP_WAIT_MS	30000000

#define POWERUP_DELAY		200
#define RESET_DELAY		1
#define FIRMWARE_PATH "/dev/block/platform/sdio_emmc/by-name/wcnmodem"
#define FIRMWARE_MAX_SIZE 0xf0c00
#define WIFI_REG 0x60300004
#define CHIPID_REG 0x603003fc
#define CALI_REG 0x70040000
#define CALI_OFSET_REG 0x70040010
#define MARLIN2_AA_CHIPID 0x23490000
#define MARLIN2_AB_CHIPID 0x23490001
