#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/driver.h>
#include <linux/i2c.h>
#include <media/v4l2-device.h>

#include <linux/clk.h>
//#include <video/sensor_drv_k.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <ontim/ontim_dev_dgb.h>

extern int set_mclk(uint32_t mclk , struct sensor_file_tag* p_file) ;
extern int set_clk_mm_i_eb(uint32_t enable);
extern bool camera_front_probe_ok;//bit1 add by liuwei
extern bool camera_back_probe_ok;//bit2add by liuwei
static char front_camera[50]="NULL_SENSOR";
static char back_camera[50]="NULL_SENSOR";


DEV_ATTR_DECLARE(camera)
DEV_ATTR_DEFINE("front_vendor",front_camera)
DEV_ATTR_DEFINE("back_vendor",back_camera)
DEV_ATTR_DECLARE_END;
ONTIM_DEBUG_DECLARE_AND_INIT(camera,camera,8);

struct sensor_power {
    struct regulator *af_2v8;
    struct regulator *avdd_2v8;
    struct regulator *dovdd_1v8;
    struct regulator *dvdd_1v2;
    char* af_2v8_name;
    char* avdd_2v8_name;
    char* dovdd_1v8_name;
    char* dvdd_1v2_name;
    int pwdn;
    int pwdn_value;
    int rst;
    int rst_value;
    int enable;
};

struct sensor_mem_tag {
	void                            *buf_ptr;
	size_t                          size;
};

struct sensor_module_tag {
	struct mutex                    sync_lock;
	atomic_t                        users;
	struct i2c_client               *cur_i2c_client;
};

struct sensor_gpio_tag {
	int                             pwn;
	int                             reset;
};

struct sensor_file_tag {
	struct sensor_module_tab_tag    *module_data;
	uint32_t                        sensor_id;
	uint32_t                        sensor_mclk;
	uint32_t                        iopower_on_count;
	uint32_t                        avddpower_on_count;
	uint32_t                        dvddpower_on_count;
	uint32_t                        motpower_on_count;
	uint32_t                        mipi_on;
	uint32_t                        padding;
	uint32_t                        phy_id;
	uint32_t                        if_type;
	struct sensor_gpio_tag          gpio_tab;
	struct clk                      *ccir_clk;
	struct clk                      *ccir_enable_clk;
	struct clk                      *mipi_clk;
	struct regulator                *camvio_regulator;
	struct regulator                *camavdd_regulator;
	struct regulator                *camdvdd_regulator;
	struct regulator                *cammot_regulator;
	struct sensor_mem_tag           sensor_mem;
};

struct b52_sensor {
    struct i2c_adapter *adapter;;
    struct device_node *dev_np;
    struct device *dev;
    struct sensor_power power;
    char* sensor_name ;
    char* modulehouse_name ;
    int id_modulehouse ;
    int reg;
    int id_regbit;
    int id_reg;
    int id_validvalue;
    u32 mclk;
};

#define SENSOR_I2C_OP_TRY_NUM             4
#define SENSOR_K_FAIL                     (-1)
#define SENSOR_CMD_BITS_8                 1
#define SENSOR_CMD_BITS_16                2
#define SENSOR_I2C_REG_8BIT               (0x00 << 1)
#define SENSOR_I2C_REG_16BIT              (0x01 << 1)
#define SENSOR_I2C_VAL_8BIT               0x00
#define SENSOR_I2C_VAL_16BIT              0x01
#define SENSOR_LOW_EIGHT_BIT              0xff
#define PLAT_CAM_DRV    "platform-cam" 

uint16_t ModuleHouseID = 0 ; 

static int32_t _Sensor_ReadReg( struct b52_sensor *sensor, u16 addr )
{
    uint8_t                       cmd[2] = { 0 }, buf_r[2] = { 0 };
    uint16_t                      w_cmd_num = 0,r_cmd_num = 0;
    int32_t                       ret = -1,i=0;
    struct i2c_msg                msg_r[2];
    uint16_t                      reg_addr ,data;

    reg_addr = addr;

    if (SENSOR_I2C_REG_16BIT ==(sensor->id_regbit & SENSOR_I2C_REG_16BIT)) {
        cmd[w_cmd_num++] = (uint8_t) ((reg_addr >> 8) & SENSOR_LOW_EIGHT_BIT);
        cmd[w_cmd_num++] = (uint8_t) (reg_addr & SENSOR_LOW_EIGHT_BIT);
        r_cmd_num = SENSOR_CMD_BITS_16;
    } else {
        cmd[w_cmd_num++] = (uint8_t) reg_addr;
        r_cmd_num = SENSOR_CMD_BITS_8;
    }

    for (i = 0; i < SENSOR_I2C_OP_TRY_NUM; i++) {
        msg_r[0].addr = sensor->reg ;
        msg_r[0].flags = 0;
        msg_r[0].buf = cmd;
        msg_r[0].len = w_cmd_num;
        msg_r[1].addr = sensor->reg ;
        msg_r[1].flags = I2C_M_RD;
        msg_r[1].buf = buf_r;
        msg_r[1].len = r_cmd_num;
        ret = i2c_transfer(sensor->adapter, msg_r, 2);
        if (ret != 2) {
            printk("SENSOR:read reg fail, ret %d, addr 0x%x, reg_addr 0x%x \n",
                    ret, sensor->adapter,reg_addr);
            msleep(20);
            ret = -1;
        } else {
            return (r_cmd_num == 1) ? (int32_t) buf_r[0] : (int32_t) ((buf_r[0] << 8) + buf_r[1]);
        }
    }

    return ret;
}


static int _Sensor_WriteReg( struct b52_sensor *sensor, u16 addr, u16 data )
{
    uint8_t                       cmd[4] = { 0 };
    uint32_t                      index = 0;
    uint32_t                      cmd_num = 0;
    struct i2c_msg                msg_w;
    int32_t                       ret = 0;
    uint16_t                      subaddr;
    int                           i;

    subaddr = addr;

    if ( SENSOR_I2C_REG_16BIT == sensor->id_regbit ) {
        cmd[cmd_num++] = (uint8_t) ((subaddr >> 8) & SENSOR_LOW_EIGHT_BIT);
        index++;
        cmd[cmd_num++] =  (uint8_t) (subaddr & SENSOR_LOW_EIGHT_BIT);
        index++;
    } else {
        cmd[cmd_num++] = (uint8_t) subaddr;
        index++;
    }

    if ( SENSOR_I2C_REG_16BIT == sensor->id_regbit ) {
        cmd[cmd_num++] = (uint8_t) ((data >> 8) & SENSOR_LOW_EIGHT_BIT);
        index++;
        cmd[cmd_num++] = (uint8_t) (data & SENSOR_LOW_EIGHT_BIT);
        index++;
    } else {
        cmd[cmd_num++] = (uint8_t) data;
        index++;
    }

    if (0xFFFF != subaddr) {
        for (i = 0; i < SENSOR_I2C_OP_TRY_NUM; i++) {
            msg_w.addr = sensor->reg;
            msg_w.flags = 0;
            msg_w.buf = cmd;
            msg_w.len = index;
            ret = i2c_transfer(sensor->adapter, &msg_w, 1);
            if (ret != 1) {
                printk("_Sensor_K_WriteReg failed:i2cAddr=%x, addr=%x, value=%x, bit=%d \n",
                        sensor->reg, addr, data, sensor->id_regbit);
                ret = SENSOR_K_FAIL;
                continue;
            } else {
                ret = 0;
                break;
            }
        }
    } else {
        msleep(data);
    }

    return ret;
}

//************read hi545 otp function begin*****************************************//
static uint16_t _hi545_Sensor_OTP_read(  struct b52_sensor *sensor, uint16_t otp_addr)
{
    uint16_t data;
    _Sensor_WriteReg(sensor,0x10a, otp_addr); //start address       
    _Sensor_WriteReg(sensor,0x102, 0x0000); //single read
    data = _Sensor_ReadReg(sensor,0x108); //OTP data read

    return data >>8;
}

static void _hi545_OTPSetting(struct b52_sensor *sensor)
{   
    _Sensor_WriteReg( sensor,0x0118, 0x0000); //sleep On

    //--- SRAM timing control---//
    _Sensor_WriteReg( sensor,0x0E00, 0x0101);
    _Sensor_WriteReg( sensor,0x0E02, 0x0101);
    _Sensor_WriteReg( sensor,0x0E04, 0x0101);
    _Sensor_WriteReg( sensor,0x0E06, 0x0101);
    _Sensor_WriteReg( sensor,0x0E08, 0x0101);
    _Sensor_WriteReg( sensor,0x0E0A, 0x0101);
    _Sensor_WriteReg( sensor,0x0E0C, 0x0101);
    _Sensor_WriteReg( sensor,0x0E0E, 0x0101);

//Firmware 2Lane v0.34, LB, OTP RW 20140715
    _Sensor_WriteReg( sensor,0x2000, 0x4031);
    _Sensor_WriteReg( sensor,0x2002, 0x83F8);
    _Sensor_WriteReg( sensor,0x2004, 0x4104);
    _Sensor_WriteReg( sensor,0x2006, 0x4307);
    _Sensor_WriteReg( sensor,0x2008, 0x430A);
    _Sensor_WriteReg( sensor,0x200a, 0x4382);
    _Sensor_WriteReg( sensor,0x200c, 0x80CC);
    _Sensor_WriteReg( sensor,0x200e, 0x4382);
    _Sensor_WriteReg( sensor,0x2010, 0x8070);
    _Sensor_WriteReg( sensor,0x2012, 0x43A2);
    _Sensor_WriteReg( sensor,0x2014, 0x0B80);
    _Sensor_WriteReg( sensor,0x2016, 0x0C0A);
    _Sensor_WriteReg( sensor,0x2018, 0x4382);
    _Sensor_WriteReg( sensor,0x201a, 0x0B90);
    _Sensor_WriteReg( sensor,0x201c, 0x0C0A);
    _Sensor_WriteReg( sensor,0x201e, 0x4382);
    _Sensor_WriteReg( sensor,0x2020, 0x0B9C);
    _Sensor_WriteReg( sensor,0x2022, 0x0C0A);
    _Sensor_WriteReg( sensor,0x2024, 0x93D2);
    _Sensor_WriteReg( sensor,0x2026, 0x003D);
    _Sensor_WriteReg( sensor,0x2028, 0x2002);
    _Sensor_WriteReg( sensor,0x202a, 0x4030);
    _Sensor_WriteReg( sensor,0x202c, 0xF69C);
    _Sensor_WriteReg( sensor,0x202e, 0x43C2);
    _Sensor_WriteReg( sensor,0x2030, 0x0F82);
    _Sensor_WriteReg( sensor,0x2032, 0x425F);
    _Sensor_WriteReg( sensor,0x2034, 0x0118);
    _Sensor_WriteReg( sensor,0x2036, 0xF37F);
    _Sensor_WriteReg( sensor,0x2038, 0x930F);
    _Sensor_WriteReg( sensor,0x203a, 0x2002);
    _Sensor_WriteReg( sensor,0x203c, 0x0CC8);
    _Sensor_WriteReg( sensor,0x203e, 0x3FF9);
    _Sensor_WriteReg( sensor,0x2040, 0x4F82);
    _Sensor_WriteReg( sensor,0x2042, 0x8098);
    _Sensor_WriteReg( sensor,0x2044, 0x43D2);
    _Sensor_WriteReg( sensor,0x2046, 0x0A80);
    _Sensor_WriteReg( sensor,0x2048, 0x43D2);
    _Sensor_WriteReg( sensor,0x204a, 0x0180);
    _Sensor_WriteReg( sensor,0x204c, 0x43D2);
    _Sensor_WriteReg( sensor,0x204e, 0x019A);
    _Sensor_WriteReg( sensor,0x2050, 0x40F2);
    _Sensor_WriteReg( sensor,0x2052, 0x0009);
    _Sensor_WriteReg( sensor,0x2054, 0x019B);
    _Sensor_WriteReg( sensor,0x2056, 0x12B0);
    _Sensor_WriteReg( sensor,0x2058, 0xFD70);
    _Sensor_WriteReg( sensor,0x205a, 0x93D2);
    _Sensor_WriteReg( sensor,0x205c, 0x003E);
    _Sensor_WriteReg( sensor,0x205e, 0x2002);
    _Sensor_WriteReg( sensor,0x2060, 0x4030);
    _Sensor_WriteReg( sensor,0x2062, 0xF580);
    _Sensor_WriteReg( sensor,0x2064, 0x4308);
    _Sensor_WriteReg( sensor,0x2066, 0x5038);
    _Sensor_WriteReg( sensor,0x2068, 0x0030);
    _Sensor_WriteReg( sensor,0x206a, 0x480F);
    _Sensor_WriteReg( sensor,0x206c, 0x12B0);
    _Sensor_WriteReg( sensor,0x206e, 0xFD82);
    _Sensor_WriteReg( sensor,0x2070, 0x403B);
    _Sensor_WriteReg( sensor,0x2072, 0x7606);
    _Sensor_WriteReg( sensor,0x2074, 0x4B29);
    _Sensor_WriteReg( sensor,0x2076, 0x5318);
    _Sensor_WriteReg( sensor,0x2078, 0x480F);
    _Sensor_WriteReg( sensor,0x207a, 0x12B0);
    _Sensor_WriteReg( sensor,0x207c, 0xFD82);
    _Sensor_WriteReg( sensor,0x207e, 0x4B2A);
    _Sensor_WriteReg( sensor,0x2080, 0x5318);
    _Sensor_WriteReg( sensor,0x2082, 0x480F);
    _Sensor_WriteReg( sensor,0x2084, 0x12B0);
    _Sensor_WriteReg( sensor,0x2086, 0xFD82);
    _Sensor_WriteReg( sensor,0x2088, 0x4A0D);
    _Sensor_WriteReg( sensor,0x208a, 0xF03D);
    _Sensor_WriteReg( sensor,0x208c, 0x000F);
    _Sensor_WriteReg( sensor,0x208e, 0x108D);
    _Sensor_WriteReg( sensor,0x2090, 0x4B2E);
    _Sensor_WriteReg( sensor,0x2092, 0x5E0E);
    _Sensor_WriteReg( sensor,0x2094, 0x5E0E);
    _Sensor_WriteReg( sensor,0x2096, 0x5E0E);
    _Sensor_WriteReg( sensor,0x2098, 0x5E0E);
    _Sensor_WriteReg( sensor,0x209a, 0x4A0F);
    _Sensor_WriteReg( sensor,0x209c, 0xC312);
    _Sensor_WriteReg( sensor,0x209e, 0x100F);
    _Sensor_WriteReg( sensor,0x20a0, 0x110F);
    _Sensor_WriteReg( sensor,0x20a2, 0x110F);
    _Sensor_WriteReg( sensor,0x20a4, 0x110F);
    _Sensor_WriteReg( sensor,0x20a6, 0x590D);
    _Sensor_WriteReg( sensor,0x20a8, 0x4D87);
    _Sensor_WriteReg( sensor,0x20aa, 0x5000);
    _Sensor_WriteReg( sensor,0x20ac, 0x5F0E);
    _Sensor_WriteReg( sensor,0x20ae, 0x4E87);
    _Sensor_WriteReg( sensor,0x20b0, 0x6000);
    _Sensor_WriteReg( sensor,0x20b2, 0x5327);
    _Sensor_WriteReg( sensor,0x20b4, 0x5038);
    _Sensor_WriteReg( sensor,0x20b6, 0xFFD1);
    _Sensor_WriteReg( sensor,0x20b8, 0x9038);
    _Sensor_WriteReg( sensor,0x20ba, 0x0300);
    _Sensor_WriteReg( sensor,0x20bc, 0x2BD4);
    _Sensor_WriteReg( sensor,0x20be, 0x0261);
    _Sensor_WriteReg( sensor,0x20c0, 0x0000);
    _Sensor_WriteReg( sensor,0x20c2, 0x43A2);
    _Sensor_WriteReg( sensor,0x20c4, 0x0384);
    _Sensor_WriteReg( sensor,0x20c6, 0x42B2);
    _Sensor_WriteReg( sensor,0x20c8, 0x0386);
    _Sensor_WriteReg( sensor,0x20ca, 0x43C2);
    _Sensor_WriteReg( sensor,0x20cc, 0x0180);
    _Sensor_WriteReg( sensor,0x20ce, 0x43D2);
    _Sensor_WriteReg( sensor,0x20d0, 0x003D);
    _Sensor_WriteReg( sensor,0x20d2, 0x40B2);
    _Sensor_WriteReg( sensor,0x20d4, 0x808B);
    _Sensor_WriteReg( sensor,0x20d6, 0x0B88);
    _Sensor_WriteReg( sensor,0x20d8, 0x0C0A);
    _Sensor_WriteReg( sensor,0x20da, 0x40B2);
    _Sensor_WriteReg( sensor,0x20dc, 0x1009);
    _Sensor_WriteReg( sensor,0x20de, 0x0B8A);
    _Sensor_WriteReg( sensor,0x20e0, 0x0C0A);
    _Sensor_WriteReg( sensor,0x20e2, 0x40B2);
    _Sensor_WriteReg( sensor,0x20e4, 0xC40C);
    _Sensor_WriteReg( sensor,0x20e6, 0x0B8C);
    _Sensor_WriteReg( sensor,0x20e8, 0x0C0A);
    _Sensor_WriteReg( sensor,0x20ea, 0x40B2);
    _Sensor_WriteReg( sensor,0x20ec, 0xC9E1);
    _Sensor_WriteReg( sensor,0x20ee, 0x0B8E);
    _Sensor_WriteReg( sensor,0x20f0, 0x0C0A);
    _Sensor_WriteReg( sensor,0x20f2, 0x40B2);
    _Sensor_WriteReg( sensor,0x20f4, 0x0C1E);
    _Sensor_WriteReg( sensor,0x20f6, 0x0B92);
    _Sensor_WriteReg( sensor,0x20f8, 0x0C0A);
    _Sensor_WriteReg( sensor,0x20fa, 0x43D2);
    _Sensor_WriteReg( sensor,0x20fc, 0x0F82);
    _Sensor_WriteReg( sensor,0x20fe, 0x0C3C);
    _Sensor_WriteReg( sensor,0x2100, 0x0C3C);
    _Sensor_WriteReg( sensor,0x2102, 0x0C3C);
    _Sensor_WriteReg( sensor,0x2104, 0x0C3C);
    _Sensor_WriteReg( sensor,0x2106, 0x421F);
    _Sensor_WriteReg( sensor,0x2108, 0x00A6);
    _Sensor_WriteReg( sensor,0x210a, 0x503F);
    _Sensor_WriteReg( sensor,0x210c, 0x07D0);
    _Sensor_WriteReg( sensor,0x210e, 0x3811);
    _Sensor_WriteReg( sensor,0x2110, 0x4F82);
    _Sensor_WriteReg( sensor,0x2112, 0x7100);
    _Sensor_WriteReg( sensor,0x2114, 0x0004);
    _Sensor_WriteReg( sensor,0x2116, 0x0C0D);
    _Sensor_WriteReg( sensor,0x2118, 0x0005);
    _Sensor_WriteReg( sensor,0x211a, 0x0C04);
    _Sensor_WriteReg( sensor,0x211c, 0x000D);
    _Sensor_WriteReg( sensor,0x211e, 0x0C09);
    _Sensor_WriteReg( sensor,0x2120, 0x003D);
    _Sensor_WriteReg( sensor,0x2122, 0x0C1D);
    _Sensor_WriteReg( sensor,0x2124, 0x003C);
    _Sensor_WriteReg( sensor,0x2126, 0x0C13);
    _Sensor_WriteReg( sensor,0x2128, 0x0004);
    _Sensor_WriteReg( sensor,0x212a, 0x0C09);
    _Sensor_WriteReg( sensor,0x212c, 0x0004);
    _Sensor_WriteReg( sensor,0x212e, 0x533F);
    _Sensor_WriteReg( sensor,0x2130, 0x37EF);
    _Sensor_WriteReg( sensor,0x2132, 0x4392);
    _Sensor_WriteReg( sensor,0x2134, 0x8094);
    _Sensor_WriteReg( sensor,0x2136, 0x4382);
    _Sensor_WriteReg( sensor,0x2138, 0x809C);
    _Sensor_WriteReg( sensor,0x213a, 0x4382);
    _Sensor_WriteReg( sensor,0x213c, 0x80B4);
    _Sensor_WriteReg( sensor,0x213e, 0x4382);
    _Sensor_WriteReg( sensor,0x2140, 0x80BA);
    _Sensor_WriteReg( sensor,0x2142, 0x4382);
    _Sensor_WriteReg( sensor,0x2144, 0x80A0);
    _Sensor_WriteReg( sensor,0x2146, 0x40B2);
    _Sensor_WriteReg( sensor,0x2148, 0x0028);
    _Sensor_WriteReg( sensor,0x214a, 0x7000);
    _Sensor_WriteReg( sensor,0x214c, 0x43A2);
    _Sensor_WriteReg( sensor,0x214e, 0x809E);
    _Sensor_WriteReg( sensor,0x2150, 0xB3E2);
    _Sensor_WriteReg( sensor,0x2152, 0x00B4);
    _Sensor_WriteReg( sensor,0x2154, 0x2402);
    _Sensor_WriteReg( sensor,0x2156, 0x4392);
    _Sensor_WriteReg( sensor,0x2158, 0x809E);
    _Sensor_WriteReg( sensor,0x215a, 0x4328);
    _Sensor_WriteReg( sensor,0x215c, 0xB3D2);
    _Sensor_WriteReg( sensor,0x215e, 0x00B4);
    _Sensor_WriteReg( sensor,0x2160, 0x2002);
    _Sensor_WriteReg( sensor,0x2162, 0x4030);
    _Sensor_WriteReg( sensor,0x2164, 0xF570);
    _Sensor_WriteReg( sensor,0x2166, 0x4308);
    _Sensor_WriteReg( sensor,0x2168, 0x4384);
    _Sensor_WriteReg( sensor,0x216a, 0x0002);
    _Sensor_WriteReg( sensor,0x216c, 0x4384);
    _Sensor_WriteReg( sensor,0x216e, 0x0006);
    _Sensor_WriteReg( sensor,0x2170, 0x4382);
    _Sensor_WriteReg( sensor,0x2172, 0x809A);
    _Sensor_WriteReg( sensor,0x2174, 0x4382);
    _Sensor_WriteReg( sensor,0x2176, 0x8096);
    _Sensor_WriteReg( sensor,0x2178, 0x40B2);
    _Sensor_WriteReg( sensor,0x217a, 0x0005);
    _Sensor_WriteReg( sensor,0x217c, 0x7320);
    _Sensor_WriteReg( sensor,0x217e, 0x4392);
    _Sensor_WriteReg( sensor,0x2180, 0x7326);
    _Sensor_WriteReg( sensor,0x2182, 0x12B0);
    _Sensor_WriteReg( sensor,0x2184, 0xF92E);
    _Sensor_WriteReg( sensor,0x2186, 0x4392);
    _Sensor_WriteReg( sensor,0x2188, 0x731C);
    _Sensor_WriteReg( sensor,0x218a, 0x9382);
    _Sensor_WriteReg( sensor,0x218c, 0x8094);
    _Sensor_WriteReg( sensor,0x218e, 0x200A);
    _Sensor_WriteReg( sensor,0x2190, 0x0B00);
    _Sensor_WriteReg( sensor,0x2192, 0x7302);
    _Sensor_WriteReg( sensor,0x2194, 0x02BC);
    _Sensor_WriteReg( sensor,0x2196, 0x4382);
    _Sensor_WriteReg( sensor,0x2198, 0x7004);
    _Sensor_WriteReg( sensor,0x219a, 0x430F);
    _Sensor_WriteReg( sensor,0x219c, 0x12B0);
    _Sensor_WriteReg( sensor,0x219e, 0xF72E);
    _Sensor_WriteReg( sensor,0x21a0, 0x12B0);
    _Sensor_WriteReg( sensor,0x21a2, 0xF92E);
    _Sensor_WriteReg( sensor,0x21a4, 0x4392);
    _Sensor_WriteReg( sensor,0x21a6, 0x80B8);
    _Sensor_WriteReg( sensor,0x21a8, 0x4382);
    _Sensor_WriteReg( sensor,0x21aa, 0x740E);
    _Sensor_WriteReg( sensor,0x21ac, 0xB3E2);
    _Sensor_WriteReg( sensor,0x21ae, 0x0080);
    _Sensor_WriteReg( sensor,0x21b0, 0x2402);
    _Sensor_WriteReg( sensor,0x21b2, 0x4392);
    _Sensor_WriteReg( sensor,0x21b4, 0x740E);
    _Sensor_WriteReg( sensor,0x21b6, 0x431F);
    _Sensor_WriteReg( sensor,0x21b8, 0x12B0);
    _Sensor_WriteReg( sensor,0x21ba, 0xF72E);
    _Sensor_WriteReg( sensor,0x21bc, 0x4392);
    _Sensor_WriteReg( sensor,0x21be, 0x7004);
    _Sensor_WriteReg( sensor,0x21c0, 0x4882);
    _Sensor_WriteReg( sensor,0x21c2, 0x7110);
    _Sensor_WriteReg( sensor,0x21c4, 0x9382);
    _Sensor_WriteReg( sensor,0x21c6, 0x8092);
    _Sensor_WriteReg( sensor,0x21c8, 0x2005);
    _Sensor_WriteReg( sensor,0x21ca, 0x9392);
    _Sensor_WriteReg( sensor,0x21cc, 0x7110);
    _Sensor_WriteReg( sensor,0x21ce, 0x2402);
    _Sensor_WriteReg( sensor,0x21d0, 0x4030);
    _Sensor_WriteReg( sensor,0x21d2, 0xF474);
    _Sensor_WriteReg( sensor,0x21d4, 0x9392);
    _Sensor_WriteReg( sensor,0x21d6, 0x7110);
    _Sensor_WriteReg( sensor,0x21d8, 0x2096);
    _Sensor_WriteReg( sensor,0x21da, 0x0B00);
    _Sensor_WriteReg( sensor,0x21dc, 0x7302);
    _Sensor_WriteReg( sensor,0x21de, 0x0032);
    _Sensor_WriteReg( sensor,0x21e0, 0x4382);
    _Sensor_WriteReg( sensor,0x21e2, 0x7004);
    _Sensor_WriteReg( sensor,0x21e4, 0x0B00);
    _Sensor_WriteReg( sensor,0x21e6, 0x7302);
    _Sensor_WriteReg( sensor,0x21e8, 0x03E8);
    _Sensor_WriteReg( sensor,0x21ea, 0x0800);
    _Sensor_WriteReg( sensor,0x21ec, 0x7114);
    _Sensor_WriteReg( sensor,0x21ee, 0x425F);
    _Sensor_WriteReg( sensor,0x21f0, 0x0C9C);
    _Sensor_WriteReg( sensor,0x21f2, 0x4F4E);
    _Sensor_WriteReg( sensor,0x21f4, 0x430F);
    _Sensor_WriteReg( sensor,0x21f6, 0x4E0D);
    _Sensor_WriteReg( sensor,0x21f8, 0x430C);
    _Sensor_WriteReg( sensor,0x21fa, 0x421F);
    _Sensor_WriteReg( sensor,0x21fc, 0x0C9A);
    _Sensor_WriteReg( sensor,0x21fe, 0xDF0C);
    _Sensor_WriteReg( sensor,0x2200, 0x1204);
    _Sensor_WriteReg( sensor,0x2202, 0x440F);
    _Sensor_WriteReg( sensor,0x2204, 0x532F);
    _Sensor_WriteReg( sensor,0x2206, 0x120F);
    _Sensor_WriteReg( sensor,0x2208, 0x1212);
    _Sensor_WriteReg( sensor,0x220a, 0x0CA2);
    _Sensor_WriteReg( sensor,0x220c, 0x403E);
    _Sensor_WriteReg( sensor,0x220e, 0x80BC);
    _Sensor_WriteReg( sensor,0x2210, 0x403F);
    _Sensor_WriteReg( sensor,0x2212, 0x8072);
    _Sensor_WriteReg( sensor,0x2214, 0x12B0);
    _Sensor_WriteReg( sensor,0x2216, 0xF796);
    _Sensor_WriteReg( sensor,0x2218, 0x4F09);
    _Sensor_WriteReg( sensor,0x221a, 0x425F);
    _Sensor_WriteReg( sensor,0x221c, 0x0CA0);
    _Sensor_WriteReg( sensor,0x221e, 0x4F4E);
    _Sensor_WriteReg( sensor,0x2220, 0x430F);
    _Sensor_WriteReg( sensor,0x2222, 0x4E0D);
    _Sensor_WriteReg( sensor,0x2224, 0x430C);
    _Sensor_WriteReg( sensor,0x2226, 0x421F);
    _Sensor_WriteReg( sensor,0x2228, 0x0C9E);
    _Sensor_WriteReg( sensor,0x222a, 0xDF0C);
    _Sensor_WriteReg( sensor,0x222c, 0x440F);
    _Sensor_WriteReg( sensor,0x222e, 0x522F);
    _Sensor_WriteReg( sensor,0x2230, 0x120F);
    _Sensor_WriteReg( sensor,0x2232, 0x532F);
    _Sensor_WriteReg( sensor,0x2234, 0x120F);
    _Sensor_WriteReg( sensor,0x2236, 0x1212);
    _Sensor_WriteReg( sensor,0x2238, 0x0CA4);
    _Sensor_WriteReg( sensor,0x223a, 0x403E);
    _Sensor_WriteReg( sensor,0x223c, 0x80A2);
    _Sensor_WriteReg( sensor,0x223e, 0x403F);
    _Sensor_WriteReg( sensor,0x2240, 0x8050);
    _Sensor_WriteReg( sensor,0x2242, 0x12B0);
    _Sensor_WriteReg( sensor,0x2244, 0xF796);
    _Sensor_WriteReg( sensor,0x2246, 0x4F0B);
    _Sensor_WriteReg( sensor,0x2248, 0x430D);
    _Sensor_WriteReg( sensor,0x224a, 0x441E);
    _Sensor_WriteReg( sensor,0x224c, 0x0004);
    _Sensor_WriteReg( sensor,0x224e, 0x442F);
    _Sensor_WriteReg( sensor,0x2250, 0x5031);
    _Sensor_WriteReg( sensor,0x2252, 0x000C);
    _Sensor_WriteReg( sensor,0x2254, 0x9E0F);
    _Sensor_WriteReg( sensor,0x2256, 0x2C01);
    _Sensor_WriteReg( sensor,0x2258, 0x431D);
    _Sensor_WriteReg( sensor,0x225a, 0x8E0F);
    _Sensor_WriteReg( sensor,0x225c, 0x930F);
    _Sensor_WriteReg( sensor,0x225e, 0x3402);
    _Sensor_WriteReg( sensor,0x2260, 0xE33F);
    _Sensor_WriteReg( sensor,0x2262, 0x531F);
    _Sensor_WriteReg( sensor,0x2264, 0x421E);
    _Sensor_WriteReg( sensor,0x2266, 0x0CA2);
    _Sensor_WriteReg( sensor,0x2268, 0xC312);
    _Sensor_WriteReg( sensor,0x226a, 0x100E);
    _Sensor_WriteReg( sensor,0x226c, 0x9E0F);
    _Sensor_WriteReg( sensor,0x226e, 0x2804);
    _Sensor_WriteReg( sensor,0x2270, 0x930D);
    _Sensor_WriteReg( sensor,0x2272, 0x2001);
    _Sensor_WriteReg( sensor,0x2274, 0x5319);
    _Sensor_WriteReg( sensor,0x2276, 0x5D0B);
    _Sensor_WriteReg( sensor,0x2278, 0x403D);
    _Sensor_WriteReg( sensor,0x227a, 0x0196);
    _Sensor_WriteReg( sensor,0x227c, 0x4D2F);
    _Sensor_WriteReg( sensor,0x227e, 0x490A);
    _Sensor_WriteReg( sensor,0x2280, 0x4F0C);
    _Sensor_WriteReg( sensor,0x2282, 0x12B0);
    _Sensor_WriteReg( sensor,0x2284, 0xFDA2);
    _Sensor_WriteReg( sensor,0x2286, 0x4E09);
    _Sensor_WriteReg( sensor,0x2288, 0xC312);
    _Sensor_WriteReg( sensor,0x228a, 0x1009);
    _Sensor_WriteReg( sensor,0x228c, 0x1109);
    _Sensor_WriteReg( sensor,0x228e, 0x1109);
    _Sensor_WriteReg( sensor,0x2290, 0x1109);
    _Sensor_WriteReg( sensor,0x2292, 0x1109);
    _Sensor_WriteReg( sensor,0x2294, 0x1109);
    _Sensor_WriteReg( sensor,0x2296, 0x4D2F);
    _Sensor_WriteReg( sensor,0x2298, 0x4B0A);
    _Sensor_WriteReg( sensor,0x229a, 0x4F0C);
    _Sensor_WriteReg( sensor,0x229c, 0x12B0);
    _Sensor_WriteReg( sensor,0x229e, 0xFDA2);
    _Sensor_WriteReg( sensor,0x22a0, 0x4E0B);
    _Sensor_WriteReg( sensor,0x22a2, 0xC312);
    _Sensor_WriteReg( sensor,0x22a4, 0x100B);
    _Sensor_WriteReg( sensor,0x22a6, 0x110B);
    _Sensor_WriteReg( sensor,0x22a8, 0x110B);
    _Sensor_WriteReg( sensor,0x22aa, 0x110B);
    _Sensor_WriteReg( sensor,0x22ac, 0x110B);
    _Sensor_WriteReg( sensor,0x22ae, 0x110B);
    _Sensor_WriteReg( sensor,0x22b0, 0x425F);
    _Sensor_WriteReg( sensor,0x22b2, 0x00BA);
    _Sensor_WriteReg( sensor,0x22b4, 0xC312);
    _Sensor_WriteReg( sensor,0x22b6, 0x104F);
    _Sensor_WriteReg( sensor,0x22b8, 0x114F);
    _Sensor_WriteReg( sensor,0x22ba, 0x114F);
    _Sensor_WriteReg( sensor,0x22bc, 0x114F);
    _Sensor_WriteReg( sensor,0x22be, 0xF37F);
    _Sensor_WriteReg( sensor,0x22c0, 0x5F0F);
    _Sensor_WriteReg( sensor,0x22c2, 0x5F0F);
    _Sensor_WriteReg( sensor,0x22c4, 0x5F0F);
    _Sensor_WriteReg( sensor,0x22c6, 0x5F0F);
    _Sensor_WriteReg( sensor,0x22c8, 0x503F);
    _Sensor_WriteReg( sensor,0x22ca, 0x0010);
    _Sensor_WriteReg( sensor,0x22cc, 0x92E2);
    _Sensor_WriteReg( sensor,0x22ce, 0x00A0);
    _Sensor_WriteReg( sensor,0x22d0, 0x2406);
    _Sensor_WriteReg( sensor,0x22d2, 0x990F);
    _Sensor_WriteReg( sensor,0x22d4, 0x2C01);
    _Sensor_WriteReg( sensor,0x22d6, 0x4F09);
    _Sensor_WriteReg( sensor,0x22d8, 0x9B0F);
    _Sensor_WriteReg( sensor,0x22da, 0x2C01);
    _Sensor_WriteReg( sensor,0x22dc, 0x4F0B);
    _Sensor_WriteReg( sensor,0x22de, 0x92B2);
    _Sensor_WriteReg( sensor,0x22e0, 0x80BA);
    _Sensor_WriteReg( sensor,0x22e2, 0x280C);
    _Sensor_WriteReg( sensor,0x22e4, 0x90B2);
    _Sensor_WriteReg( sensor,0x22e6, 0x0096);
    _Sensor_WriteReg( sensor,0x22e8, 0x80B4);
    _Sensor_WriteReg( sensor,0x22ea, 0x2408);
    _Sensor_WriteReg( sensor,0x22ec, 0x0900);
    _Sensor_WriteReg( sensor,0x22ee, 0x710E);
    _Sensor_WriteReg( sensor,0x22f0, 0x0B00);
    _Sensor_WriteReg( sensor,0x22f2, 0x7302);
    _Sensor_WriteReg( sensor,0x22f4, 0x0320);
    _Sensor_WriteReg( sensor,0x22f6, 0x12B0);
    _Sensor_WriteReg( sensor,0x22f8, 0xF6CE);
    _Sensor_WriteReg( sensor,0x22fa, 0x3F64);
    _Sensor_WriteReg( sensor,0x22fc, 0x4982);
    _Sensor_WriteReg( sensor,0x22fe, 0x0CAC);
    _Sensor_WriteReg( sensor,0x2300, 0x4B82);
    _Sensor_WriteReg( sensor,0x2302, 0x0CAE);
    _Sensor_WriteReg( sensor,0x2304, 0x3FF3);
    _Sensor_WriteReg( sensor,0x2306, 0x0B00);
    _Sensor_WriteReg( sensor,0x2308, 0x7302);
    _Sensor_WriteReg( sensor,0x230a, 0x0002);
    _Sensor_WriteReg( sensor,0x230c, 0x069A);
    _Sensor_WriteReg( sensor,0x230e, 0x0C1F);
    _Sensor_WriteReg( sensor,0x2310, 0x0403);
    _Sensor_WriteReg( sensor,0x2312, 0x0C05);
    _Sensor_WriteReg( sensor,0x2314, 0x0001);
    _Sensor_WriteReg( sensor,0x2316, 0x0C01);
    _Sensor_WriteReg( sensor,0x2318, 0x0003);
    _Sensor_WriteReg( sensor,0x231a, 0x0C03);
    _Sensor_WriteReg( sensor,0x231c, 0x000B);
    _Sensor_WriteReg( sensor,0x231e, 0x0C33);
    _Sensor_WriteReg( sensor,0x2320, 0x0003);
    _Sensor_WriteReg( sensor,0x2322, 0x0C03);
    _Sensor_WriteReg( sensor,0x2324, 0x0653);
    _Sensor_WriteReg( sensor,0x2326, 0x0C03);
    _Sensor_WriteReg( sensor,0x2328, 0x065B);
    _Sensor_WriteReg( sensor,0x232a, 0x0C13);
    _Sensor_WriteReg( sensor,0x232c, 0x065F);
    _Sensor_WriteReg( sensor,0x232e, 0x0C43);
    _Sensor_WriteReg( sensor,0x2330, 0x0657);
    _Sensor_WriteReg( sensor,0x2332, 0x0C03);
    _Sensor_WriteReg( sensor,0x2334, 0x0653);
    _Sensor_WriteReg( sensor,0x2336, 0x0C03);
    _Sensor_WriteReg( sensor,0x2338, 0x0643);
    _Sensor_WriteReg( sensor,0x233a, 0x0C0F);
    _Sensor_WriteReg( sensor,0x233c, 0x067D);
    _Sensor_WriteReg( sensor,0x233e, 0x0C01);
    _Sensor_WriteReg( sensor,0x2340, 0x077F);
    _Sensor_WriteReg( sensor,0x2342, 0x0C01);
    _Sensor_WriteReg( sensor,0x2344, 0x0677);
    _Sensor_WriteReg( sensor,0x2346, 0x0C01);
    _Sensor_WriteReg( sensor,0x2348, 0x0673);
    _Sensor_WriteReg( sensor,0x234a, 0x0C67);
    _Sensor_WriteReg( sensor,0x234c, 0x0677);
    _Sensor_WriteReg( sensor,0x234e, 0x0C03);
    _Sensor_WriteReg( sensor,0x2350, 0x077D);
    _Sensor_WriteReg( sensor,0x2352, 0x0C19);
    _Sensor_WriteReg( sensor,0x2354, 0x0013);
    _Sensor_WriteReg( sensor,0x2356, 0x0C27);
    _Sensor_WriteReg( sensor,0x2358, 0x0003);
    _Sensor_WriteReg( sensor,0x235a, 0x0C45);
    _Sensor_WriteReg( sensor,0x235c, 0x0675);
    _Sensor_WriteReg( sensor,0x235e, 0x0C01);
    _Sensor_WriteReg( sensor,0x2360, 0x0671);
    _Sensor_WriteReg( sensor,0x2362, 0x4392);
    _Sensor_WriteReg( sensor,0x2364, 0x7004);
    _Sensor_WriteReg( sensor,0x2366, 0x430F);
    _Sensor_WriteReg( sensor,0x2368, 0x9382);
    _Sensor_WriteReg( sensor,0x236a, 0x80B8);
    _Sensor_WriteReg( sensor,0x236c, 0x2001);
    _Sensor_WriteReg( sensor,0x236e, 0x431F);
    _Sensor_WriteReg( sensor,0x2370, 0x4F82);
    _Sensor_WriteReg( sensor,0x2372, 0x80B8);
    _Sensor_WriteReg( sensor,0x2374, 0x930F);
    _Sensor_WriteReg( sensor,0x2376, 0x2472);
    _Sensor_WriteReg( sensor,0x2378, 0x0B00);
    _Sensor_WriteReg( sensor,0x237a, 0x7302);
    _Sensor_WriteReg( sensor,0x237c, 0x033A);
    _Sensor_WriteReg( sensor,0x237e, 0x0675);
    _Sensor_WriteReg( sensor,0x2380, 0x0C02);
    _Sensor_WriteReg( sensor,0x2382, 0x0339);
    _Sensor_WriteReg( sensor,0x2384, 0xAE0C);
    _Sensor_WriteReg( sensor,0x2386, 0x0C01);
    _Sensor_WriteReg( sensor,0x2388, 0x003C);
    _Sensor_WriteReg( sensor,0x238a, 0x0C01);
    _Sensor_WriteReg( sensor,0x238c, 0x0004);
    _Sensor_WriteReg( sensor,0x238e, 0x0C01);
    _Sensor_WriteReg( sensor,0x2390, 0x0642);
    _Sensor_WriteReg( sensor,0x2392, 0x0B00);
    _Sensor_WriteReg( sensor,0x2394, 0x7302);
    _Sensor_WriteReg( sensor,0x2396, 0x0386);
    _Sensor_WriteReg( sensor,0x2398, 0x0643);
    _Sensor_WriteReg( sensor,0x239a, 0x0C05);
    _Sensor_WriteReg( sensor,0x239c, 0x0001);
    _Sensor_WriteReg( sensor,0x239e, 0x0C01);
    _Sensor_WriteReg( sensor,0x23a0, 0x0003);
    _Sensor_WriteReg( sensor,0x23a2, 0x0C03);
    _Sensor_WriteReg( sensor,0x23a4, 0x000B);
    _Sensor_WriteReg( sensor,0x23a6, 0x0C33);
    _Sensor_WriteReg( sensor,0x23a8, 0x0003);
    _Sensor_WriteReg( sensor,0x23aa, 0x0C03);
    _Sensor_WriteReg( sensor,0x23ac, 0x0653);
    _Sensor_WriteReg( sensor,0x23ae, 0x0C03);
    _Sensor_WriteReg( sensor,0x23b0, 0x065B);
    _Sensor_WriteReg( sensor,0x23b2, 0x0C13);
    _Sensor_WriteReg( sensor,0x23b4, 0x065F);
    _Sensor_WriteReg( sensor,0x23b6, 0x0C43);
    _Sensor_WriteReg( sensor,0x23b8, 0x0657);
    _Sensor_WriteReg( sensor,0x23ba, 0x0C03);
    _Sensor_WriteReg( sensor,0x23bc, 0x0653);
    _Sensor_WriteReg( sensor,0x23be, 0x0C03);
    _Sensor_WriteReg( sensor,0x23c0, 0x0643);
    _Sensor_WriteReg( sensor,0x23c2, 0x0C0F);
    _Sensor_WriteReg( sensor,0x23c4, 0x067D);
    _Sensor_WriteReg( sensor,0x23c6, 0x0C01);
    _Sensor_WriteReg( sensor,0x23c8, 0x077F);
    _Sensor_WriteReg( sensor,0x23ca, 0x0C01);
    _Sensor_WriteReg( sensor,0x23cc, 0x0677);
    _Sensor_WriteReg( sensor,0x23ce, 0x0C01);
    _Sensor_WriteReg( sensor,0x23d0, 0x0673);
    _Sensor_WriteReg( sensor,0x23d2, 0x0C67);
    _Sensor_WriteReg( sensor,0x23d4, 0x0677);
    _Sensor_WriteReg( sensor,0x23d6, 0x0C03);
    _Sensor_WriteReg( sensor,0x23d8, 0x077D);
    _Sensor_WriteReg( sensor,0x23da, 0x0C19);
    _Sensor_WriteReg( sensor,0x23dc, 0x0013);
    _Sensor_WriteReg( sensor,0x23de, 0x0C27);
    _Sensor_WriteReg( sensor,0x23e0, 0x0003);
    _Sensor_WriteReg( sensor,0x23e2, 0x0C45);
    _Sensor_WriteReg( sensor,0x23e4, 0x0675);
    _Sensor_WriteReg( sensor,0x23e6, 0x0C01);
    _Sensor_WriteReg( sensor,0x23e8, 0x0671);
    _Sensor_WriteReg( sensor,0x23ea, 0x12B0);
    _Sensor_WriteReg( sensor,0x23ec, 0xF6CE);
    _Sensor_WriteReg( sensor,0x23ee, 0x930F);
    _Sensor_WriteReg( sensor,0x23f0, 0x2405);
    _Sensor_WriteReg( sensor,0x23f2, 0x4292);
    _Sensor_WriteReg( sensor,0x23f4, 0x8094);
    _Sensor_WriteReg( sensor,0x23f6, 0x809C);
    _Sensor_WriteReg( sensor,0x23f8, 0x4382);
    _Sensor_WriteReg( sensor,0x23fa, 0x8094);
    _Sensor_WriteReg( sensor,0x23fc, 0x9382);
    _Sensor_WriteReg( sensor,0x23fe, 0x80B8);
    _Sensor_WriteReg( sensor,0x2400, 0x241D);
    _Sensor_WriteReg( sensor,0x2402, 0x0B00);
    _Sensor_WriteReg( sensor,0x2404, 0x7302);
    _Sensor_WriteReg( sensor,0x2406, 0x069E);
    _Sensor_WriteReg( sensor,0x2408, 0x0675);
    _Sensor_WriteReg( sensor,0x240a, 0x0C02);
    _Sensor_WriteReg( sensor,0x240c, 0x0339);
    _Sensor_WriteReg( sensor,0x240e, 0xAE0C);
    _Sensor_WriteReg( sensor,0x2410, 0x0C01);
    _Sensor_WriteReg( sensor,0x2412, 0x003C);
    _Sensor_WriteReg( sensor,0x2414, 0x0C01);
    _Sensor_WriteReg( sensor,0x2416, 0x0004);
    _Sensor_WriteReg( sensor,0x2418, 0x0C01);
    _Sensor_WriteReg( sensor,0x241a, 0x0642);
    _Sensor_WriteReg( sensor,0x241c, 0x0C01);
    _Sensor_WriteReg( sensor,0x241e, 0x06A1);
    _Sensor_WriteReg( sensor,0x2420, 0x0C03);
    _Sensor_WriteReg( sensor,0x2422, 0x06A0);
    _Sensor_WriteReg( sensor,0x2424, 0x9382);
    _Sensor_WriteReg( sensor,0x2426, 0x80CC);
    _Sensor_WriteReg( sensor,0x2428, 0x2003);
    _Sensor_WriteReg( sensor,0x242a, 0x930F);
    _Sensor_WriteReg( sensor,0x242c, 0x26CB);
    _Sensor_WriteReg( sensor,0x242e, 0x3EAD);
    _Sensor_WriteReg( sensor,0x2430, 0x43C2);
    _Sensor_WriteReg( sensor,0x2432, 0x0A80);
    _Sensor_WriteReg( sensor,0x2434, 0x0B00);
    _Sensor_WriteReg( sensor,0x2436, 0x7302);
    _Sensor_WriteReg( sensor,0x2438, 0xFFF0);
    _Sensor_WriteReg( sensor,0x243a, 0x3EC4);
    _Sensor_WriteReg( sensor,0x243c, 0x0B00);
    _Sensor_WriteReg( sensor,0x243e, 0x7302);
    _Sensor_WriteReg( sensor,0x2440, 0x069E);
    _Sensor_WriteReg( sensor,0x2442, 0x0675);
    _Sensor_WriteReg( sensor,0x2444, 0x0C02);
    _Sensor_WriteReg( sensor,0x2446, 0x0301);
    _Sensor_WriteReg( sensor,0x2448, 0xAE0C);
    _Sensor_WriteReg( sensor,0x244a, 0x0C01);
    _Sensor_WriteReg( sensor,0x244c, 0x0004);
    _Sensor_WriteReg( sensor,0x244e, 0x0C03);
    _Sensor_WriteReg( sensor,0x2450, 0x0642);
    _Sensor_WriteReg( sensor,0x2452, 0x0C01);
    _Sensor_WriteReg( sensor,0x2454, 0x06A1);
    _Sensor_WriteReg( sensor,0x2456, 0x0C03);
    _Sensor_WriteReg( sensor,0x2458, 0x06A0);
    _Sensor_WriteReg( sensor,0x245a, 0x3FE4);
    _Sensor_WriteReg( sensor,0x245c, 0x0B00);
    _Sensor_WriteReg( sensor,0x245e, 0x7302);
    _Sensor_WriteReg( sensor,0x2460, 0x033A);
    _Sensor_WriteReg( sensor,0x2462, 0x0675);
    _Sensor_WriteReg( sensor,0x2464, 0x0C02);
    _Sensor_WriteReg( sensor,0x2466, 0x0301);
    _Sensor_WriteReg( sensor,0x2468, 0xAE0C);
    _Sensor_WriteReg( sensor,0x246a, 0x0C01);
    _Sensor_WriteReg( sensor,0x246c, 0x0004);
    _Sensor_WriteReg( sensor,0x246e, 0x0C03);
    _Sensor_WriteReg( sensor,0x2470, 0x0642);
    _Sensor_WriteReg( sensor,0x2472, 0x3F8F);
    _Sensor_WriteReg( sensor,0x2474, 0x0B00);
    _Sensor_WriteReg( sensor,0x2476, 0x7302);
    _Sensor_WriteReg( sensor,0x2478, 0x0002);
    _Sensor_WriteReg( sensor,0x247a, 0x069A);
    _Sensor_WriteReg( sensor,0x247c, 0x0C1F);
    _Sensor_WriteReg( sensor,0x247e, 0x0402);
    _Sensor_WriteReg( sensor,0x2480, 0x0C05);
    _Sensor_WriteReg( sensor,0x2482, 0x0001);
    _Sensor_WriteReg( sensor,0x2484, 0x0C01);
    _Sensor_WriteReg( sensor,0x2486, 0x0003);
    _Sensor_WriteReg( sensor,0x2488, 0x0C03);
    _Sensor_WriteReg( sensor,0x248a, 0x000B);
    _Sensor_WriteReg( sensor,0x248c, 0x0C33);
    _Sensor_WriteReg( sensor,0x248e, 0x0003);
    _Sensor_WriteReg( sensor,0x2490, 0x0C03);
    _Sensor_WriteReg( sensor,0x2492, 0x0653);
    _Sensor_WriteReg( sensor,0x2494, 0x0C03);
    _Sensor_WriteReg( sensor,0x2496, 0x065B);
    _Sensor_WriteReg( sensor,0x2498, 0x0C13);
    _Sensor_WriteReg( sensor,0x249a, 0x065F);
    _Sensor_WriteReg( sensor,0x249c, 0x0C43);
    _Sensor_WriteReg( sensor,0x249e, 0x0657);
    _Sensor_WriteReg( sensor,0x24a0, 0x0C03);
    _Sensor_WriteReg( sensor,0x24a2, 0x0653);
    _Sensor_WriteReg( sensor,0x24a4, 0x0C03);
    _Sensor_WriteReg( sensor,0x24a6, 0x0643);
    _Sensor_WriteReg( sensor,0x24a8, 0x0C0F);
    _Sensor_WriteReg( sensor,0x24aa, 0x077D);
    _Sensor_WriteReg( sensor,0x24ac, 0x0C01);
    _Sensor_WriteReg( sensor,0x24ae, 0x067F);
    _Sensor_WriteReg( sensor,0x24b0, 0x0C01);
    _Sensor_WriteReg( sensor,0x24b2, 0x0677);
    _Sensor_WriteReg( sensor,0x24b4, 0x0C01);
    _Sensor_WriteReg( sensor,0x24b6, 0x0673);
    _Sensor_WriteReg( sensor,0x24b8, 0x0C5F);
    _Sensor_WriteReg( sensor,0x24ba, 0x0663);
    _Sensor_WriteReg( sensor,0x24bc, 0x0C6F);
    _Sensor_WriteReg( sensor,0x24be, 0x0667);
    _Sensor_WriteReg( sensor,0x24c0, 0x0C01);
    _Sensor_WriteReg( sensor,0x24c2, 0x0677);
    _Sensor_WriteReg( sensor,0x24c4, 0x0C01);
    _Sensor_WriteReg( sensor,0x24c6, 0x077D);
    _Sensor_WriteReg( sensor,0x24c8, 0x0C33);
    _Sensor_WriteReg( sensor,0x24ca, 0x0013);
    _Sensor_WriteReg( sensor,0x24cc, 0x0C27);
    _Sensor_WriteReg( sensor,0x24ce, 0x0003);
    _Sensor_WriteReg( sensor,0x24d0, 0x0C4F);
    _Sensor_WriteReg( sensor,0x24d2, 0x0675);
    _Sensor_WriteReg( sensor,0x24d4, 0x0C01);
    _Sensor_WriteReg( sensor,0x24d6, 0x0671);
    _Sensor_WriteReg( sensor,0x24d8, 0x0CFF);
    _Sensor_WriteReg( sensor,0x24da, 0x0C78);
    _Sensor_WriteReg( sensor,0x24dc, 0x0661);
    _Sensor_WriteReg( sensor,0x24de, 0x4392);
    _Sensor_WriteReg( sensor,0x24e0, 0x7004);
    _Sensor_WriteReg( sensor,0x24e2, 0x430F);
    _Sensor_WriteReg( sensor,0x24e4, 0x9382);
    _Sensor_WriteReg( sensor,0x24e6, 0x80B8);
    _Sensor_WriteReg( sensor,0x24e8, 0x2001);
    _Sensor_WriteReg( sensor,0x24ea, 0x431F);
    _Sensor_WriteReg( sensor,0x24ec, 0x4F82);
    _Sensor_WriteReg( sensor,0x24ee, 0x80B8);
    _Sensor_WriteReg( sensor,0x24f0, 0x12B0);
    _Sensor_WriteReg( sensor,0x24f2, 0xF6CE);
    _Sensor_WriteReg( sensor,0x24f4, 0x930F);
    _Sensor_WriteReg( sensor,0x24f6, 0x2405);
    _Sensor_WriteReg( sensor,0x24f8, 0x4292);
    _Sensor_WriteReg( sensor,0x24fa, 0x8094);
    _Sensor_WriteReg( sensor,0x24fc, 0x809C);
    _Sensor_WriteReg( sensor,0x24fe, 0x4382);
    _Sensor_WriteReg( sensor,0x2500, 0x8094);
    _Sensor_WriteReg( sensor,0x2502, 0x9382);
    _Sensor_WriteReg( sensor,0x2504, 0x80B8);
    _Sensor_WriteReg( sensor,0x2506, 0x2019);
    _Sensor_WriteReg( sensor,0x2508, 0x0B00);
    _Sensor_WriteReg( sensor,0x250a, 0x7302);
    _Sensor_WriteReg( sensor,0x250c, 0x0562);
    _Sensor_WriteReg( sensor,0x250e, 0x0665);
    _Sensor_WriteReg( sensor,0x2510, 0x0C02);
    _Sensor_WriteReg( sensor,0x2512, 0x0301);
    _Sensor_WriteReg( sensor,0x2514, 0xA60C);
    _Sensor_WriteReg( sensor,0x2516, 0x0204);
    _Sensor_WriteReg( sensor,0x2518, 0xAE0C);
    _Sensor_WriteReg( sensor,0x251a, 0x0C03);
    _Sensor_WriteReg( sensor,0x251c, 0x0642);
    _Sensor_WriteReg( sensor,0x251e, 0x0C13);
    _Sensor_WriteReg( sensor,0x2520, 0x06A1);
    _Sensor_WriteReg( sensor,0x2522, 0x0C03);
    _Sensor_WriteReg( sensor,0x2524, 0x06A0);
    _Sensor_WriteReg( sensor,0x2526, 0x9382);
    _Sensor_WriteReg( sensor,0x2528, 0x80CC);
    _Sensor_WriteReg( sensor,0x252a, 0x277F);
    _Sensor_WriteReg( sensor,0x252c, 0x43C2);
    _Sensor_WriteReg( sensor,0x252e, 0x0A80);
    _Sensor_WriteReg( sensor,0x2530, 0x0B00);
    _Sensor_WriteReg( sensor,0x2532, 0x7302);
    _Sensor_WriteReg( sensor,0x2534, 0xFFF0);
    _Sensor_WriteReg( sensor,0x2536, 0x4030);
    _Sensor_WriteReg( sensor,0x2538, 0xF1C4);
    _Sensor_WriteReg( sensor,0x253a, 0x0B00);
    _Sensor_WriteReg( sensor,0x253c, 0x7302);
    _Sensor_WriteReg( sensor,0x253e, 0x0562);
    _Sensor_WriteReg( sensor,0x2540, 0x0665);
    _Sensor_WriteReg( sensor,0x2542, 0x0C02);
    _Sensor_WriteReg( sensor,0x2544, 0x0339);
    _Sensor_WriteReg( sensor,0x2546, 0xA60C);
    _Sensor_WriteReg( sensor,0x2548, 0x023C);
    _Sensor_WriteReg( sensor,0x254a, 0xAE0C);
    _Sensor_WriteReg( sensor,0x254c, 0x0C01);
    _Sensor_WriteReg( sensor,0x254e, 0x0004);
    _Sensor_WriteReg( sensor,0x2550, 0x0C01);
    _Sensor_WriteReg( sensor,0x2552, 0x0642);
    _Sensor_WriteReg( sensor,0x2554, 0x0C13);
    _Sensor_WriteReg( sensor,0x2556, 0x06A1);
    _Sensor_WriteReg( sensor,0x2558, 0x0C03);
    _Sensor_WriteReg( sensor,0x255a, 0x06A0);
    _Sensor_WriteReg( sensor,0x255c, 0x9382);
    _Sensor_WriteReg( sensor,0x255e, 0x80CC);
    _Sensor_WriteReg( sensor,0x2560, 0x2764);
    _Sensor_WriteReg( sensor,0x2562, 0x43C2);
    _Sensor_WriteReg( sensor,0x2564, 0x0A80);
    _Sensor_WriteReg( sensor,0x2566, 0x0B00);
    _Sensor_WriteReg( sensor,0x2568, 0x7302);
    _Sensor_WriteReg( sensor,0x256a, 0xFFF0);
    _Sensor_WriteReg( sensor,0x256c, 0x4030);
    _Sensor_WriteReg( sensor,0x256e, 0xF1C4);
    _Sensor_WriteReg( sensor,0x2570, 0xB3E2);
    _Sensor_WriteReg( sensor,0x2572, 0x00B4);
    _Sensor_WriteReg( sensor,0x2574, 0x2002);
    _Sensor_WriteReg( sensor,0x2576, 0x4030);
    _Sensor_WriteReg( sensor,0x2578, 0xF168);
    _Sensor_WriteReg( sensor,0x257a, 0x4318);
    _Sensor_WriteReg( sensor,0x257c, 0x4030);
    _Sensor_WriteReg( sensor,0x257e, 0xF168);
    _Sensor_WriteReg( sensor,0x2580, 0x4392);
    _Sensor_WriteReg( sensor,0x2582, 0x760E);
    _Sensor_WriteReg( sensor,0x2584, 0x425F);
    _Sensor_WriteReg( sensor,0x2586, 0x0118);
    _Sensor_WriteReg( sensor,0x2588, 0xF37F);
    _Sensor_WriteReg( sensor,0x258a, 0x930F);
    _Sensor_WriteReg( sensor,0x258c, 0x2005);
    _Sensor_WriteReg( sensor,0x258e, 0x43C2);
    _Sensor_WriteReg( sensor,0x2590, 0x0A80);
    _Sensor_WriteReg( sensor,0x2592, 0x0B00);
    _Sensor_WriteReg( sensor,0x2594, 0x7302);
    _Sensor_WriteReg( sensor,0x2596, 0xFFF0);
    _Sensor_WriteReg( sensor,0x2598, 0x9382);
    _Sensor_WriteReg( sensor,0x259a, 0x760C);
    _Sensor_WriteReg( sensor,0x259c, 0x2002);
    _Sensor_WriteReg( sensor,0x259e, 0x0C64);
    _Sensor_WriteReg( sensor,0x25a0, 0x3FF1);
    _Sensor_WriteReg( sensor,0x25a2, 0x4F82);
    _Sensor_WriteReg( sensor,0x25a4, 0x8098);
    _Sensor_WriteReg( sensor,0x25a6, 0x12B0);
    _Sensor_WriteReg( sensor,0x25a8, 0xFD70);
    _Sensor_WriteReg( sensor,0x25aa, 0x421F);
    _Sensor_WriteReg( sensor,0x25ac, 0x760A);
    _Sensor_WriteReg( sensor,0x25ae, 0x903F);
    _Sensor_WriteReg( sensor,0x25b0, 0x0200);
    _Sensor_WriteReg( sensor,0x25b2, 0x2469);
    _Sensor_WriteReg( sensor,0x25b4, 0x930F);
    _Sensor_WriteReg( sensor,0x25b6, 0x2467);
    _Sensor_WriteReg( sensor,0x25b8, 0x903F);
    _Sensor_WriteReg( sensor,0x25ba, 0x0100);
    _Sensor_WriteReg( sensor,0x25bc, 0x23E1);
    _Sensor_WriteReg( sensor,0x25be, 0x40B2);
    _Sensor_WriteReg( sensor,0x25c0, 0x0005);
    _Sensor_WriteReg( sensor,0x25c2, 0x7600);
    _Sensor_WriteReg( sensor,0x25c4, 0x4382);
    _Sensor_WriteReg( sensor,0x25c6, 0x7602);
    _Sensor_WriteReg( sensor,0x25c8, 0x0262);
    _Sensor_WriteReg( sensor,0x25ca, 0x0000);
    _Sensor_WriteReg( sensor,0x25cc, 0x0222);
    _Sensor_WriteReg( sensor,0x25ce, 0x0000);
    _Sensor_WriteReg( sensor,0x25d0, 0x0262);
    _Sensor_WriteReg( sensor,0x25d2, 0x0000);
    _Sensor_WriteReg( sensor,0x25d4, 0x0260);
    _Sensor_WriteReg( sensor,0x25d6, 0x0000);
    _Sensor_WriteReg( sensor,0x25d8, 0x425F);
    _Sensor_WriteReg( sensor,0x25da, 0x0186);
    _Sensor_WriteReg( sensor,0x25dc, 0x4F4C);
    _Sensor_WriteReg( sensor,0x25de, 0x421B);
    _Sensor_WriteReg( sensor,0x25e0, 0x018A);
    _Sensor_WriteReg( sensor,0x25e2, 0x93D2);
    _Sensor_WriteReg( sensor,0x25e4, 0x018F);
    _Sensor_WriteReg( sensor,0x25e6, 0x244D);
    _Sensor_WriteReg( sensor,0x25e8, 0x425F);
    _Sensor_WriteReg( sensor,0x25ea, 0x018F);
    _Sensor_WriteReg( sensor,0x25ec, 0x4F4D);
    _Sensor_WriteReg( sensor,0x25ee, 0x4308);
    _Sensor_WriteReg( sensor,0x25f0, 0x431F);
    _Sensor_WriteReg( sensor,0x25f2, 0x480E);
    _Sensor_WriteReg( sensor,0x25f4, 0x930E);
    _Sensor_WriteReg( sensor,0x25f6, 0x2403);
    _Sensor_WriteReg( sensor,0x25f8, 0x5F0F);
    _Sensor_WriteReg( sensor,0x25fa, 0x831E);
    _Sensor_WriteReg( sensor,0x25fc, 0x23FD);
    _Sensor_WriteReg( sensor,0x25fe, 0xFC0F);
    _Sensor_WriteReg( sensor,0x2600, 0x242E);
    _Sensor_WriteReg( sensor,0x2602, 0x430F);
    _Sensor_WriteReg( sensor,0x2604, 0x9D0F);
    _Sensor_WriteReg( sensor,0x2606, 0x2C2B);
    _Sensor_WriteReg( sensor,0x2608, 0x4B82);
    _Sensor_WriteReg( sensor,0x260a, 0x7600);
    _Sensor_WriteReg( sensor,0x260c, 0x4882);
    _Sensor_WriteReg( sensor,0x260e, 0x7602);
    _Sensor_WriteReg( sensor,0x2610, 0x4C82);
    _Sensor_WriteReg( sensor,0x2612, 0x7604);
    _Sensor_WriteReg( sensor,0x2614, 0x0264);
    _Sensor_WriteReg( sensor,0x2616, 0x0000);
    _Sensor_WriteReg( sensor,0x2618, 0x0224);
    _Sensor_WriteReg( sensor,0x261a, 0x0000);
    _Sensor_WriteReg( sensor,0x261c, 0x0264);
    _Sensor_WriteReg( sensor,0x261e, 0x0000);
    _Sensor_WriteReg( sensor,0x2620, 0x0260);
    _Sensor_WriteReg( sensor,0x2622, 0x0000);
    _Sensor_WriteReg( sensor,0x2624, 0x0268);
    _Sensor_WriteReg( sensor,0x2626, 0x0000);
    _Sensor_WriteReg( sensor,0x2628, 0x0C18);
    _Sensor_WriteReg( sensor,0x262a, 0x02E8);
    _Sensor_WriteReg( sensor,0x262c, 0x0000);
    _Sensor_WriteReg( sensor,0x262e, 0x0C30);
    _Sensor_WriteReg( sensor,0x2630, 0x02A8);
    _Sensor_WriteReg( sensor,0x2632, 0x0000);
    _Sensor_WriteReg( sensor,0x2634, 0x0C30);
    _Sensor_WriteReg( sensor,0x2636, 0x0C30);
    _Sensor_WriteReg( sensor,0x2638, 0x0C30);
    _Sensor_WriteReg( sensor,0x263a, 0x0C30);
    _Sensor_WriteReg( sensor,0x263c, 0x0C30);
    _Sensor_WriteReg( sensor,0x263e, 0x0C30);
    _Sensor_WriteReg( sensor,0x2640, 0x0C30);
    _Sensor_WriteReg( sensor,0x2642, 0x0C30);
    _Sensor_WriteReg( sensor,0x2644, 0x0C00);
    _Sensor_WriteReg( sensor,0x2646, 0x02E8);
    _Sensor_WriteReg( sensor,0x2648, 0x0000);
    _Sensor_WriteReg( sensor,0x264a, 0x0C30);
    _Sensor_WriteReg( sensor,0x264c, 0x0268);
    _Sensor_WriteReg( sensor,0x264e, 0x0000);
    _Sensor_WriteReg( sensor,0x2650, 0x0C18);
    _Sensor_WriteReg( sensor,0x2652, 0x0260);
    _Sensor_WriteReg( sensor,0x2654, 0x0000);
    _Sensor_WriteReg( sensor,0x2656, 0x0C18);
    _Sensor_WriteReg( sensor,0x2658, 0x531F);
    _Sensor_WriteReg( sensor,0x265a, 0x9D0F);
    _Sensor_WriteReg( sensor,0x265c, 0x2BD5);
    _Sensor_WriteReg( sensor,0x265e, 0x5318);
    _Sensor_WriteReg( sensor,0x2660, 0x9238);
    _Sensor_WriteReg( sensor,0x2662, 0x2BC6);
    _Sensor_WriteReg( sensor,0x2664, 0x0261);
    _Sensor_WriteReg( sensor,0x2666, 0x0000);
    _Sensor_WriteReg( sensor,0x2668, 0x12B0);
    _Sensor_WriteReg( sensor,0x266a, 0xFD70);
    _Sensor_WriteReg( sensor,0x266c, 0x4A0F);
    _Sensor_WriteReg( sensor,0x266e, 0x12B0);
    _Sensor_WriteReg( sensor,0x2670, 0xFD82);
    _Sensor_WriteReg( sensor,0x2672, 0x421F);
    _Sensor_WriteReg( sensor,0x2674, 0x7606);
    _Sensor_WriteReg( sensor,0x2676, 0x4FC2);
    _Sensor_WriteReg( sensor,0x2678, 0x0188);
    _Sensor_WriteReg( sensor,0x267a, 0x4B0A);
    _Sensor_WriteReg( sensor,0x267c, 0x0261);
    _Sensor_WriteReg( sensor,0x267e, 0x0000);
    _Sensor_WriteReg( sensor,0x2680, 0x3F7F);
    _Sensor_WriteReg( sensor,0x2682, 0x432D);
    _Sensor_WriteReg( sensor,0x2684, 0x3FB4);
    _Sensor_WriteReg( sensor,0x2686, 0x421F);
    _Sensor_WriteReg( sensor,0x2688, 0x018A);
    _Sensor_WriteReg( sensor,0x268a, 0x12B0);
    _Sensor_WriteReg( sensor,0x268c, 0xFD82);
    _Sensor_WriteReg( sensor,0x268e, 0x421F);
    _Sensor_WriteReg( sensor,0x2690, 0x7606);
    _Sensor_WriteReg( sensor,0x2692, 0x4FC2);
    _Sensor_WriteReg( sensor,0x2694, 0x0188);
    _Sensor_WriteReg( sensor,0x2696, 0x0261);
    _Sensor_WriteReg( sensor,0x2698, 0x0000);
    _Sensor_WriteReg( sensor,0x269a, 0x3F72);
    _Sensor_WriteReg( sensor,0x269c, 0x4382);
    _Sensor_WriteReg( sensor,0x269e, 0x0B88);
    _Sensor_WriteReg( sensor,0x26a0, 0x0C0A);
    _Sensor_WriteReg( sensor,0x26a2, 0x4382);
    _Sensor_WriteReg( sensor,0x26a4, 0x0B8A);
    _Sensor_WriteReg( sensor,0x26a6, 0x0C0A);
    _Sensor_WriteReg( sensor,0x26a8, 0x40B2);
    _Sensor_WriteReg( sensor,0x26aa, 0x000C);
    _Sensor_WriteReg( sensor,0x26ac, 0x0B8C);
    _Sensor_WriteReg( sensor,0x26ae, 0x0C0A);
    _Sensor_WriteReg( sensor,0x26b0, 0x40B2);
    _Sensor_WriteReg( sensor,0x26b2, 0xB5E1);
    _Sensor_WriteReg( sensor,0x26b4, 0x0B8E);
    _Sensor_WriteReg( sensor,0x26b6, 0x0C0A);
    _Sensor_WriteReg( sensor,0x26b8, 0x40B2);
    _Sensor_WriteReg( sensor,0x26ba, 0x641C);
    _Sensor_WriteReg( sensor,0x26bc, 0x0B92);
    _Sensor_WriteReg( sensor,0x26be, 0x0C0A);
    _Sensor_WriteReg( sensor,0x26c0, 0x43C2);
    _Sensor_WriteReg( sensor,0x26c2, 0x003D);
    _Sensor_WriteReg( sensor,0x26c4, 0x4030);
    _Sensor_WriteReg( sensor,0x26c6, 0xF02E);
    _Sensor_WriteReg( sensor,0x26c8, 0x5231);
    _Sensor_WriteReg( sensor,0x26ca, 0x4030);
    _Sensor_WriteReg( sensor,0x26cc, 0xFD9E);
    _Sensor_WriteReg( sensor,0x26ce, 0xE3B2);
    _Sensor_WriteReg( sensor,0x26d0, 0x740E);
    _Sensor_WriteReg( sensor,0x26d2, 0x425F);
    _Sensor_WriteReg( sensor,0x26d4, 0x0118);
    _Sensor_WriteReg( sensor,0x26d6, 0xF37F);
    _Sensor_WriteReg( sensor,0x26d8, 0x4F82);
    _Sensor_WriteReg( sensor,0x26da, 0x8098);
    _Sensor_WriteReg( sensor,0x26dc, 0x930F);
    _Sensor_WriteReg( sensor,0x26de, 0x2005);
    _Sensor_WriteReg( sensor,0x26e0, 0x93C2);
    _Sensor_WriteReg( sensor,0x26e2, 0x0A82);
    _Sensor_WriteReg( sensor,0x26e4, 0x2402);
    _Sensor_WriteReg( sensor,0x26e6, 0x4392);
    _Sensor_WriteReg( sensor,0x26e8, 0x80CC);
    _Sensor_WriteReg( sensor,0x26ea, 0x9382);
    _Sensor_WriteReg( sensor,0x26ec, 0x8098);
    _Sensor_WriteReg( sensor,0x26ee, 0x2002);
    _Sensor_WriteReg( sensor,0x26f0, 0x4392);
    _Sensor_WriteReg( sensor,0x26f2, 0x8070);
    _Sensor_WriteReg( sensor,0x26f4, 0x421F);
    _Sensor_WriteReg( sensor,0x26f6, 0x710E);
    _Sensor_WriteReg( sensor,0x26f8, 0x93A2);
    _Sensor_WriteReg( sensor,0x26fa, 0x7110);
    _Sensor_WriteReg( sensor,0x26fc, 0x2411);
    _Sensor_WriteReg( sensor,0x26fe, 0x9382);
    _Sensor_WriteReg( sensor,0x2700, 0x710E);
    _Sensor_WriteReg( sensor,0x2702, 0x240C);
    _Sensor_WriteReg( sensor,0x2704, 0x5292);
    _Sensor_WriteReg( sensor,0x2706, 0x809E);
    _Sensor_WriteReg( sensor,0x2708, 0x7110);
    _Sensor_WriteReg( sensor,0x270a, 0x4382);
    _Sensor_WriteReg( sensor,0x270c, 0x740E);
    _Sensor_WriteReg( sensor,0x270e, 0x9382);
    _Sensor_WriteReg( sensor,0x2710, 0x80B6);
    _Sensor_WriteReg( sensor,0x2712, 0x2402);
    _Sensor_WriteReg( sensor,0x2714, 0x4392);
    _Sensor_WriteReg( sensor,0x2716, 0x740E);
    _Sensor_WriteReg( sensor,0x2718, 0x4392);
    _Sensor_WriteReg( sensor,0x271a, 0x80B8);
    _Sensor_WriteReg( sensor,0x271c, 0x430F);
    _Sensor_WriteReg( sensor,0x271e, 0x4130);
    _Sensor_WriteReg( sensor,0x2720, 0xF31F);
    _Sensor_WriteReg( sensor,0x2722, 0x27ED);
    _Sensor_WriteReg( sensor,0x2724, 0x40B2);
    _Sensor_WriteReg( sensor,0x2726, 0x0003);
    _Sensor_WriteReg( sensor,0x2728, 0x7110);
    _Sensor_WriteReg( sensor,0x272a, 0x431F);
    _Sensor_WriteReg( sensor,0x272c, 0x4130);
    _Sensor_WriteReg( sensor,0x272e, 0x4F0E);
    _Sensor_WriteReg( sensor,0x2730, 0x421D);
    _Sensor_WriteReg( sensor,0x2732, 0x8070);
    _Sensor_WriteReg( sensor,0x2734, 0x425F);
    _Sensor_WriteReg( sensor,0x2736, 0x0118);
    _Sensor_WriteReg( sensor,0x2738, 0xF37F);
    _Sensor_WriteReg( sensor,0x273a, 0x903E);
    _Sensor_WriteReg( sensor,0x273c, 0x0003);
    _Sensor_WriteReg( sensor,0x273e, 0x2405);
    _Sensor_WriteReg( sensor,0x2740, 0x931E);
    _Sensor_WriteReg( sensor,0x2742, 0x2403);
    _Sensor_WriteReg( sensor,0x2744, 0x0B00);
    _Sensor_WriteReg( sensor,0x2746, 0x7302);
    _Sensor_WriteReg( sensor,0x2748, 0x0384);
    _Sensor_WriteReg( sensor,0x274a, 0x930F);
    _Sensor_WriteReg( sensor,0x274c, 0x241A);
    _Sensor_WriteReg( sensor,0x274e, 0x930D);
    _Sensor_WriteReg( sensor,0x2750, 0x2018);
    _Sensor_WriteReg( sensor,0x2752, 0x9382);
    _Sensor_WriteReg( sensor,0x2754, 0x7308);
    _Sensor_WriteReg( sensor,0x2756, 0x2402);
    _Sensor_WriteReg( sensor,0x2758, 0x930E);
    _Sensor_WriteReg( sensor,0x275a, 0x2419);
    _Sensor_WriteReg( sensor,0x275c, 0x9382);
    _Sensor_WriteReg( sensor,0x275e, 0x7328);
    _Sensor_WriteReg( sensor,0x2760, 0x2402);
    _Sensor_WriteReg( sensor,0x2762, 0x931E);
    _Sensor_WriteReg( sensor,0x2764, 0x2414);
    _Sensor_WriteReg( sensor,0x2766, 0x9382);
    _Sensor_WriteReg( sensor,0x2768, 0x710E);
    _Sensor_WriteReg( sensor,0x276a, 0x2402);
    _Sensor_WriteReg( sensor,0x276c, 0x932E);
    _Sensor_WriteReg( sensor,0x276e, 0x240F);
    _Sensor_WriteReg( sensor,0x2770, 0x9382);
    _Sensor_WriteReg( sensor,0x2772, 0x7114);
    _Sensor_WriteReg( sensor,0x2774, 0x2402);
    _Sensor_WriteReg( sensor,0x2776, 0x922E);
    _Sensor_WriteReg( sensor,0x2778, 0x240A);
    _Sensor_WriteReg( sensor,0x277a, 0x903E);
    _Sensor_WriteReg( sensor,0x277c, 0x0003);
    _Sensor_WriteReg( sensor,0x277e, 0x23DA);
    _Sensor_WriteReg( sensor,0x2780, 0x3C06);
    _Sensor_WriteReg( sensor,0x2782, 0x43C2);
    _Sensor_WriteReg( sensor,0x2784, 0x0A80);
    _Sensor_WriteReg( sensor,0x2786, 0x0B00);
    _Sensor_WriteReg( sensor,0x2788, 0x7302);
    _Sensor_WriteReg( sensor,0x278a, 0xFFF0);
    _Sensor_WriteReg( sensor,0x278c, 0x3FD3);
    _Sensor_WriteReg( sensor,0x278e, 0x4F82);
    _Sensor_WriteReg( sensor,0x2790, 0x8098);
    _Sensor_WriteReg( sensor,0x2792, 0x431F);
    _Sensor_WriteReg( sensor,0x2794, 0x4130);
    _Sensor_WriteReg( sensor,0x2796, 0x120B);
    _Sensor_WriteReg( sensor,0x2798, 0x120A);
    _Sensor_WriteReg( sensor,0x279a, 0x1209);
    _Sensor_WriteReg( sensor,0x279c, 0x1208);
    _Sensor_WriteReg( sensor,0x279e, 0x1207);
    _Sensor_WriteReg( sensor,0x27a0, 0x1206);
    _Sensor_WriteReg( sensor,0x27a2, 0x1205);
    _Sensor_WriteReg( sensor,0x27a4, 0x1204);
    _Sensor_WriteReg( sensor,0x27a6, 0x8221);
    _Sensor_WriteReg( sensor,0x27a8, 0x403B);
    _Sensor_WriteReg( sensor,0x27aa, 0x0016);
    _Sensor_WriteReg( sensor,0x27ac, 0x510B);
    _Sensor_WriteReg( sensor,0x27ae, 0x4F08);
    _Sensor_WriteReg( sensor,0x27b0, 0x4E09);
    _Sensor_WriteReg( sensor,0x27b2, 0x4BA1);
    _Sensor_WriteReg( sensor,0x27b4, 0x0000);
    _Sensor_WriteReg( sensor,0x27b6, 0x4B1A);
    _Sensor_WriteReg( sensor,0x27b8, 0x0002);
    _Sensor_WriteReg( sensor,0x27ba, 0x4B91);
    _Sensor_WriteReg( sensor,0x27bc, 0x0004);
    _Sensor_WriteReg( sensor,0x27be, 0x0002);
    _Sensor_WriteReg( sensor,0x27c0, 0x4304);
    _Sensor_WriteReg( sensor,0x27c2, 0x4305);
    _Sensor_WriteReg( sensor,0x27c4, 0x4306);
    _Sensor_WriteReg( sensor,0x27c6, 0x4307);
    _Sensor_WriteReg( sensor,0x27c8, 0x9382);
    _Sensor_WriteReg( sensor,0x27ca, 0x80B2);
    _Sensor_WriteReg( sensor,0x27cc, 0x2425);
    _Sensor_WriteReg( sensor,0x27ce, 0x438A);
    _Sensor_WriteReg( sensor,0x27d0, 0x0000);
    _Sensor_WriteReg( sensor,0x27d2, 0x430B);
    _Sensor_WriteReg( sensor,0x27d4, 0x4B0F);
    _Sensor_WriteReg( sensor,0x27d6, 0x5F0F);
    _Sensor_WriteReg( sensor,0x27d8, 0x5F0F);
    _Sensor_WriteReg( sensor,0x27da, 0x580F);
    _Sensor_WriteReg( sensor,0x27dc, 0x4C8F);
    _Sensor_WriteReg( sensor,0x27de, 0x0000);
    _Sensor_WriteReg( sensor,0x27e0, 0x4D8F);
    _Sensor_WriteReg( sensor,0x27e2, 0x0002);
    _Sensor_WriteReg( sensor,0x27e4, 0x4B0F);
    _Sensor_WriteReg( sensor,0x27e6, 0x5F0F);
    _Sensor_WriteReg( sensor,0x27e8, 0x590F);
    _Sensor_WriteReg( sensor,0x27ea, 0x41AF);
    _Sensor_WriteReg( sensor,0x27ec, 0x0000);
    _Sensor_WriteReg( sensor,0x27ee, 0x531B);
    _Sensor_WriteReg( sensor,0x27f0, 0x923B);
    _Sensor_WriteReg( sensor,0x27f2, 0x2BF0);
    _Sensor_WriteReg( sensor,0x27f4, 0x430B);
    _Sensor_WriteReg( sensor,0x27f6, 0x4B0F);
    _Sensor_WriteReg( sensor,0x27f8, 0x5F0F);
    _Sensor_WriteReg( sensor,0x27fa, 0x5F0F);
    _Sensor_WriteReg( sensor,0x27fc, 0x580F);
    _Sensor_WriteReg( sensor,0x27fe, 0x5F34);
    _Sensor_WriteReg( sensor,0x2800, 0x6F35);
    _Sensor_WriteReg( sensor,0x2802, 0x4B0F);
    _Sensor_WriteReg( sensor,0x2804, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2806, 0x590F);
    _Sensor_WriteReg( sensor,0x2808, 0x4F2E);
    _Sensor_WriteReg( sensor,0x280a, 0x430F);
    _Sensor_WriteReg( sensor,0x280c, 0x5E06);
    _Sensor_WriteReg( sensor,0x280e, 0x6F07);
    _Sensor_WriteReg( sensor,0x2810, 0x531B);
    _Sensor_WriteReg( sensor,0x2812, 0x923B);
    _Sensor_WriteReg( sensor,0x2814, 0x2BF0);
    _Sensor_WriteReg( sensor,0x2816, 0x3C18);
    _Sensor_WriteReg( sensor,0x2818, 0x4A2E);
    _Sensor_WriteReg( sensor,0x281a, 0x4E0F);
    _Sensor_WriteReg( sensor,0x281c, 0x5F0F);
    _Sensor_WriteReg( sensor,0x281e, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2820, 0x580F);
    _Sensor_WriteReg( sensor,0x2822, 0x4C8F);
    _Sensor_WriteReg( sensor,0x2824, 0x0000);
    _Sensor_WriteReg( sensor,0x2826, 0x4D8F);
    _Sensor_WriteReg( sensor,0x2828, 0x0002);
    _Sensor_WriteReg( sensor,0x282a, 0x5E0E);
    _Sensor_WriteReg( sensor,0x282c, 0x590E);
    _Sensor_WriteReg( sensor,0x282e, 0x41AE);
    _Sensor_WriteReg( sensor,0x2830, 0x0000);
    _Sensor_WriteReg( sensor,0x2832, 0x4A2F);
    _Sensor_WriteReg( sensor,0x2834, 0x903F);
    _Sensor_WriteReg( sensor,0x2836, 0x0007);
    _Sensor_WriteReg( sensor,0x2838, 0x2404);
    _Sensor_WriteReg( sensor,0x283a, 0x531F);
    _Sensor_WriteReg( sensor,0x283c, 0x4F8A);
    _Sensor_WriteReg( sensor,0x283e, 0x0000);
    _Sensor_WriteReg( sensor,0x2840, 0x3FD9);
    _Sensor_WriteReg( sensor,0x2842, 0x438A);
    _Sensor_WriteReg( sensor,0x2844, 0x0000);
    _Sensor_WriteReg( sensor,0x2846, 0x3FD6);
    _Sensor_WriteReg( sensor,0x2848, 0x440C);
    _Sensor_WriteReg( sensor,0x284a, 0x450D);
    _Sensor_WriteReg( sensor,0x284c, 0x460A);
    _Sensor_WriteReg( sensor,0x284e, 0x470B);
    _Sensor_WriteReg( sensor,0x2850, 0x12B0);
    _Sensor_WriteReg( sensor,0x2852, 0xFDF4);
    _Sensor_WriteReg( sensor,0x2854, 0x4C08);
    _Sensor_WriteReg( sensor,0x2856, 0x4D09);
    _Sensor_WriteReg( sensor,0x2858, 0x4C0E);
    _Sensor_WriteReg( sensor,0x285a, 0x430F);
    _Sensor_WriteReg( sensor,0x285c, 0x4E0A);
    _Sensor_WriteReg( sensor,0x285e, 0x4F0B);
    _Sensor_WriteReg( sensor,0x2860, 0x460C);
    _Sensor_WriteReg( sensor,0x2862, 0x470D);
    _Sensor_WriteReg( sensor,0x2864, 0x12B0);
    _Sensor_WriteReg( sensor,0x2866, 0xFDB8);
    _Sensor_WriteReg( sensor,0x2868, 0x8E04);
    _Sensor_WriteReg( sensor,0x286a, 0x7F05);
    _Sensor_WriteReg( sensor,0x286c, 0x440E);
    _Sensor_WriteReg( sensor,0x286e, 0x450F);
    _Sensor_WriteReg( sensor,0x2870, 0xC312);
    _Sensor_WriteReg( sensor,0x2872, 0x100F);
    _Sensor_WriteReg( sensor,0x2874, 0x100E);
    _Sensor_WriteReg( sensor,0x2876, 0x110F);
    _Sensor_WriteReg( sensor,0x2878, 0x100E);
    _Sensor_WriteReg( sensor,0x287a, 0x110F);
    _Sensor_WriteReg( sensor,0x287c, 0x100E);
    _Sensor_WriteReg( sensor,0x287e, 0x411D);
    _Sensor_WriteReg( sensor,0x2880, 0x0002);
    _Sensor_WriteReg( sensor,0x2882, 0x4E8D);
    _Sensor_WriteReg( sensor,0x2884, 0x0000);
    _Sensor_WriteReg( sensor,0x2886, 0x480F);
    _Sensor_WriteReg( sensor,0x2888, 0x5221);
    _Sensor_WriteReg( sensor,0x288a, 0x4134);
    _Sensor_WriteReg( sensor,0x288c, 0x4135);
    _Sensor_WriteReg( sensor,0x288e, 0x4136);
    _Sensor_WriteReg( sensor,0x2890, 0x4137);
    _Sensor_WriteReg( sensor,0x2892, 0x4138);
    _Sensor_WriteReg( sensor,0x2894, 0x4139);
    _Sensor_WriteReg( sensor,0x2896, 0x413A);
    _Sensor_WriteReg( sensor,0x2898, 0x413B);
    _Sensor_WriteReg( sensor,0x289a, 0x4130);
    _Sensor_WriteReg( sensor,0x289c, 0x120A);
    _Sensor_WriteReg( sensor,0x289e, 0x4F0D);
    _Sensor_WriteReg( sensor,0x28a0, 0x4E0C);
    _Sensor_WriteReg( sensor,0x28a2, 0x425F);
    _Sensor_WriteReg( sensor,0x28a4, 0x00BA);
    _Sensor_WriteReg( sensor,0x28a6, 0x4F4A);
    _Sensor_WriteReg( sensor,0x28a8, 0x503A);
    _Sensor_WriteReg( sensor,0x28aa, 0x0010);
    _Sensor_WriteReg( sensor,0x28ac, 0x931D);
    _Sensor_WriteReg( sensor,0x28ae, 0x242B);
    _Sensor_WriteReg( sensor,0x28b0, 0x932D);
    _Sensor_WriteReg( sensor,0x28b2, 0x2421);
    _Sensor_WriteReg( sensor,0x28b4, 0x903D);
    _Sensor_WriteReg( sensor,0x28b6, 0x0003);
    _Sensor_WriteReg( sensor,0x28b8, 0x2418);
    _Sensor_WriteReg( sensor,0x28ba, 0x922D);
    _Sensor_WriteReg( sensor,0x28bc, 0x2413);
    _Sensor_WriteReg( sensor,0x28be, 0x903D);
    _Sensor_WriteReg( sensor,0x28c0, 0x0005);
    _Sensor_WriteReg( sensor,0x28c2, 0x2407);
    _Sensor_WriteReg( sensor,0x28c4, 0x903D);
    _Sensor_WriteReg( sensor,0x28c6, 0x0006);
    _Sensor_WriteReg( sensor,0x28c8, 0x2028);
    _Sensor_WriteReg( sensor,0x28ca, 0xC312);
    _Sensor_WriteReg( sensor,0x28cc, 0x100A);
    _Sensor_WriteReg( sensor,0x28ce, 0x110A);
    _Sensor_WriteReg( sensor,0x28d0, 0x3C24);
    _Sensor_WriteReg( sensor,0x28d2, 0x4A0E);
    _Sensor_WriteReg( sensor,0x28d4, 0xC312);
    _Sensor_WriteReg( sensor,0x28d6, 0x100E);
    _Sensor_WriteReg( sensor,0x28d8, 0x110E);
    _Sensor_WriteReg( sensor,0x28da, 0x4E0F);
    _Sensor_WriteReg( sensor,0x28dc, 0x110F);
    _Sensor_WriteReg( sensor,0x28de, 0x4E0A);
    _Sensor_WriteReg( sensor,0x28e0, 0x5F0A);
    _Sensor_WriteReg( sensor,0x28e2, 0x3C1B);
    _Sensor_WriteReg( sensor,0x28e4, 0xC312);
    _Sensor_WriteReg( sensor,0x28e6, 0x100A);
    _Sensor_WriteReg( sensor,0x28e8, 0x3C18);
    _Sensor_WriteReg( sensor,0x28ea, 0x4A0E);
    _Sensor_WriteReg( sensor,0x28ec, 0xC312);
    _Sensor_WriteReg( sensor,0x28ee, 0x100E);
    _Sensor_WriteReg( sensor,0x28f0, 0x4E0F);
    _Sensor_WriteReg( sensor,0x28f2, 0x110F);
    _Sensor_WriteReg( sensor,0x28f4, 0x3FF3);
    _Sensor_WriteReg( sensor,0x28f6, 0x4A0F);
    _Sensor_WriteReg( sensor,0x28f8, 0xC312);
    _Sensor_WriteReg( sensor,0x28fa, 0x100F);
    _Sensor_WriteReg( sensor,0x28fc, 0x4F0E);
    _Sensor_WriteReg( sensor,0x28fe, 0x110E);
    _Sensor_WriteReg( sensor,0x2900, 0x4F0A);
    _Sensor_WriteReg( sensor,0x2902, 0x5E0A);
    _Sensor_WriteReg( sensor,0x2904, 0x3C0A);
    _Sensor_WriteReg( sensor,0x2906, 0x4A0F);
    _Sensor_WriteReg( sensor,0x2908, 0xC312);
    _Sensor_WriteReg( sensor,0x290a, 0x100F);
    _Sensor_WriteReg( sensor,0x290c, 0x4F0E);
    _Sensor_WriteReg( sensor,0x290e, 0x110E);
    _Sensor_WriteReg( sensor,0x2910, 0x4F0A);
    _Sensor_WriteReg( sensor,0x2912, 0x5E0A);
    _Sensor_WriteReg( sensor,0x2914, 0x4E0F);
    _Sensor_WriteReg( sensor,0x2916, 0x110F);
    _Sensor_WriteReg( sensor,0x2918, 0x3FE3);
    _Sensor_WriteReg( sensor,0x291a, 0x12B0);
    _Sensor_WriteReg( sensor,0x291c, 0xFDA2);
    _Sensor_WriteReg( sensor,0x291e, 0x4E0F);
    _Sensor_WriteReg( sensor,0x2920, 0xC312);
    _Sensor_WriteReg( sensor,0x2922, 0x100F);
    _Sensor_WriteReg( sensor,0x2924, 0x110F);
    _Sensor_WriteReg( sensor,0x2926, 0x110F);
    _Sensor_WriteReg( sensor,0x2928, 0x110F);
    _Sensor_WriteReg( sensor,0x292a, 0x413A);
    _Sensor_WriteReg( sensor,0x292c, 0x4130);
    _Sensor_WriteReg( sensor,0x292e, 0x120B);
    _Sensor_WriteReg( sensor,0x2930, 0x120A);
    _Sensor_WriteReg( sensor,0x2932, 0x1209);
    _Sensor_WriteReg( sensor,0x2934, 0x1208);
    _Sensor_WriteReg( sensor,0x2936, 0x425F);
    _Sensor_WriteReg( sensor,0x2938, 0x0080);
    _Sensor_WriteReg( sensor,0x293a, 0xF36F);
    _Sensor_WriteReg( sensor,0x293c, 0x4F0E);
    _Sensor_WriteReg( sensor,0x293e, 0xF32E);
    _Sensor_WriteReg( sensor,0x2940, 0x4E82);
    _Sensor_WriteReg( sensor,0x2942, 0x80B6);
    _Sensor_WriteReg( sensor,0x2944, 0xB3D2);
    _Sensor_WriteReg( sensor,0x2946, 0x0786);
    _Sensor_WriteReg( sensor,0x2948, 0x2402);
    _Sensor_WriteReg( sensor,0x294a, 0x4392);
    _Sensor_WriteReg( sensor,0x294c, 0x7002);
    _Sensor_WriteReg( sensor,0x294e, 0x4392);
    _Sensor_WriteReg( sensor,0x2950, 0x80B8);
    _Sensor_WriteReg( sensor,0x2952, 0x4382);
    _Sensor_WriteReg( sensor,0x2954, 0x740E);
    _Sensor_WriteReg( sensor,0x2956, 0x9382);
    _Sensor_WriteReg( sensor,0x2958, 0x80B6);
    _Sensor_WriteReg( sensor,0x295a, 0x2402);
    _Sensor_WriteReg( sensor,0x295c, 0x4392);
    _Sensor_WriteReg( sensor,0x295e, 0x740E);
    _Sensor_WriteReg( sensor,0x2960, 0x93C2);
    _Sensor_WriteReg( sensor,0x2962, 0x00C6);
    _Sensor_WriteReg( sensor,0x2964, 0x2406);
    _Sensor_WriteReg( sensor,0x2966, 0xB392);
    _Sensor_WriteReg( sensor,0x2968, 0x732A);
    _Sensor_WriteReg( sensor,0x296a, 0x2403);
    _Sensor_WriteReg( sensor,0x296c, 0xB3D2);
    _Sensor_WriteReg( sensor,0x296e, 0x00C7);
    _Sensor_WriteReg( sensor,0x2970, 0x2412);
    _Sensor_WriteReg( sensor,0x2972, 0x4292);
    _Sensor_WriteReg( sensor,0x2974, 0x01A8);
    _Sensor_WriteReg( sensor,0x2976, 0x0688);
    _Sensor_WriteReg( sensor,0x2978, 0x4292);
    _Sensor_WriteReg( sensor,0x297a, 0x01AA);
    _Sensor_WriteReg( sensor,0x297c, 0x068A);
    _Sensor_WriteReg( sensor,0x297e, 0x4292);
    _Sensor_WriteReg( sensor,0x2980, 0x01AC);
    _Sensor_WriteReg( sensor,0x2982, 0x068C);
    _Sensor_WriteReg( sensor,0x2984, 0x4292);
    _Sensor_WriteReg( sensor,0x2986, 0x01AE);
    _Sensor_WriteReg( sensor,0x2988, 0x068E);
    _Sensor_WriteReg( sensor,0x298a, 0x4292);
    _Sensor_WriteReg( sensor,0x298c, 0x0190);
    _Sensor_WriteReg( sensor,0x298e, 0x0A92);
    _Sensor_WriteReg( sensor,0x2990, 0x4292);
    _Sensor_WriteReg( sensor,0x2992, 0x0192);
    _Sensor_WriteReg( sensor,0x2994, 0x0A94);
    _Sensor_WriteReg( sensor,0x2996, 0x430E);
    _Sensor_WriteReg( sensor,0x2998, 0x425F);
    _Sensor_WriteReg( sensor,0x299a, 0x00C7);
    _Sensor_WriteReg( sensor,0x299c, 0xF35F);
    _Sensor_WriteReg( sensor,0x299e, 0xF37F);
    _Sensor_WriteReg( sensor,0x29a0, 0xF21F);
    _Sensor_WriteReg( sensor,0x29a2, 0x732A);
    _Sensor_WriteReg( sensor,0x29a4, 0x200C);
    _Sensor_WriteReg( sensor,0x29a6, 0xB3D2);
    _Sensor_WriteReg( sensor,0x29a8, 0x00C7);
    _Sensor_WriteReg( sensor,0x29aa, 0x2003);
    _Sensor_WriteReg( sensor,0x29ac, 0xB392);
    _Sensor_WriteReg( sensor,0x29ae, 0x80A0);
    _Sensor_WriteReg( sensor,0x29b0, 0x2006);
    _Sensor_WriteReg( sensor,0x29b2, 0xB3A2);
    _Sensor_WriteReg( sensor,0x29b4, 0x732A);
    _Sensor_WriteReg( sensor,0x29b6, 0x2003);
    _Sensor_WriteReg( sensor,0x29b8, 0x9382);
    _Sensor_WriteReg( sensor,0x29ba, 0x8094);
    _Sensor_WriteReg( sensor,0x29bc, 0x2401);
    _Sensor_WriteReg( sensor,0x29be, 0x431E);
    _Sensor_WriteReg( sensor,0x29c0, 0x4E82);
    _Sensor_WriteReg( sensor,0x29c2, 0x80B2);
    _Sensor_WriteReg( sensor,0x29c4, 0x930E);
    _Sensor_WriteReg( sensor,0x29c6, 0x25B6);
    _Sensor_WriteReg( sensor,0x29c8, 0x4382);
    _Sensor_WriteReg( sensor,0x29ca, 0x80BA);
    _Sensor_WriteReg( sensor,0x29cc, 0x421F);
    _Sensor_WriteReg( sensor,0x29ce, 0x732A);
    _Sensor_WriteReg( sensor,0x29d0, 0xF31F);
    _Sensor_WriteReg( sensor,0x29d2, 0x4F82);
    _Sensor_WriteReg( sensor,0x29d4, 0x80A0);
    _Sensor_WriteReg( sensor,0x29d6, 0x425F);
    _Sensor_WriteReg( sensor,0x29d8, 0x008C);
    _Sensor_WriteReg( sensor,0x29da, 0x4FC2);
    _Sensor_WriteReg( sensor,0x29dc, 0x8092);
    _Sensor_WriteReg( sensor,0x29de, 0x43C2);
    _Sensor_WriteReg( sensor,0x29e0, 0x8093);
    _Sensor_WriteReg( sensor,0x29e2, 0x425F);
    _Sensor_WriteReg( sensor,0x29e4, 0x009E);
    _Sensor_WriteReg( sensor,0x29e6, 0x4F48);
    _Sensor_WriteReg( sensor,0x29e8, 0x425F);
    _Sensor_WriteReg( sensor,0x29ea, 0x009F);
    _Sensor_WriteReg( sensor,0x29ec, 0xF37F);
    _Sensor_WriteReg( sensor,0x29ee, 0x5F08);
    _Sensor_WriteReg( sensor,0x29f0, 0x1108);
    _Sensor_WriteReg( sensor,0x29f2, 0x1108);
    _Sensor_WriteReg( sensor,0x29f4, 0x425F);
    _Sensor_WriteReg( sensor,0x29f6, 0x00B2);
    _Sensor_WriteReg( sensor,0x29f8, 0x4F49);
    _Sensor_WriteReg( sensor,0x29fa, 0x425F);
    _Sensor_WriteReg( sensor,0x29fc, 0x00B3);
    _Sensor_WriteReg( sensor,0x29fe, 0xF37F);
    _Sensor_WriteReg( sensor,0x2a00, 0x5F09);
    _Sensor_WriteReg( sensor,0x2a02, 0x1109);
    _Sensor_WriteReg( sensor,0x2a04, 0x1109);
    _Sensor_WriteReg( sensor,0x2a06, 0x425F);
    _Sensor_WriteReg( sensor,0x2a08, 0x00BA);
    _Sensor_WriteReg( sensor,0x2a0a, 0x4F4C);
    _Sensor_WriteReg( sensor,0x2a0c, 0x407A);
    _Sensor_WriteReg( sensor,0x2a0e, 0x001C);
    _Sensor_WriteReg( sensor,0x2a10, 0x12B0);
    _Sensor_WriteReg( sensor,0x2a12, 0xFDD8);
    _Sensor_WriteReg( sensor,0x2a14, 0x934C);
    _Sensor_WriteReg( sensor,0x2a16, 0x257A);
    _Sensor_WriteReg( sensor,0x2a18, 0x403E);
    _Sensor_WriteReg( sensor,0x2a1a, 0x0080);
    _Sensor_WriteReg( sensor,0x2a1c, 0x4E0F);
    _Sensor_WriteReg( sensor,0x2a1e, 0xF37F);
    _Sensor_WriteReg( sensor,0x2a20, 0x108F);
    _Sensor_WriteReg( sensor,0x2a22, 0xD03F);
    _Sensor_WriteReg( sensor,0x2a24, 0x008B);
    _Sensor_WriteReg( sensor,0x2a26, 0x4F82);
    _Sensor_WriteReg( sensor,0x2a28, 0x0B88);
    _Sensor_WriteReg( sensor,0x2a2a, 0x0C0A);
    _Sensor_WriteReg( sensor,0x2a2c, 0x403F);
    _Sensor_WriteReg( sensor,0x2a2e, 0x00BA);
    _Sensor_WriteReg( sensor,0x2a30, 0x4F6E);
    _Sensor_WriteReg( sensor,0x2a32, 0x403C);
    _Sensor_WriteReg( sensor,0x2a34, 0x0813);
    _Sensor_WriteReg( sensor,0x2a36, 0x403D);
    _Sensor_WriteReg( sensor,0x2a38, 0x007F);
    _Sensor_WriteReg( sensor,0x2a3a, 0x90FF);
    _Sensor_WriteReg( sensor,0x2a3c, 0x0011);
    _Sensor_WriteReg( sensor,0x2a3e, 0x0000);
    _Sensor_WriteReg( sensor,0x2a40, 0x2D13);
    _Sensor_WriteReg( sensor,0x2a42, 0x403C);
    _Sensor_WriteReg( sensor,0x2a44, 0x1009);
    _Sensor_WriteReg( sensor,0x2a46, 0x430D);
    _Sensor_WriteReg( sensor,0x2a48, 0x425E);
    _Sensor_WriteReg( sensor,0x2a4a, 0x00BA);
    _Sensor_WriteReg( sensor,0x2a4c, 0x4E4F);
    _Sensor_WriteReg( sensor,0x2a4e, 0x108F);
    _Sensor_WriteReg( sensor,0x2a50, 0xDD0F);
    _Sensor_WriteReg( sensor,0x2a52, 0x4F82);
    _Sensor_WriteReg( sensor,0x2a54, 0x0B90);
    _Sensor_WriteReg( sensor,0x2a56, 0x0C0A);
    _Sensor_WriteReg( sensor,0x2a58, 0x4C82);
    _Sensor_WriteReg( sensor,0x2a5a, 0x0B8A);
    _Sensor_WriteReg( sensor,0x2a5c, 0x0C0A);
    _Sensor_WriteReg( sensor,0x2a5e, 0x425F);
    _Sensor_WriteReg( sensor,0x2a60, 0x0C87);
    _Sensor_WriteReg( sensor,0x2a62, 0x4F4E);
    _Sensor_WriteReg( sensor,0x2a64, 0x425F);
    _Sensor_WriteReg( sensor,0x2a66, 0x0C88);
    _Sensor_WriteReg( sensor,0x2a68, 0xF37F);
    _Sensor_WriteReg( sensor,0x2a6a, 0x12B0);
    _Sensor_WriteReg( sensor,0x2a6c, 0xF89C);
    _Sensor_WriteReg( sensor,0x2a6e, 0x4F82);
    _Sensor_WriteReg( sensor,0x2a70, 0x0C8C);
    _Sensor_WriteReg( sensor,0x2a72, 0x425F);
    _Sensor_WriteReg( sensor,0x2a74, 0x0C85);
    _Sensor_WriteReg( sensor,0x2a76, 0x4F4E);
    _Sensor_WriteReg( sensor,0x2a78, 0x425F);
    _Sensor_WriteReg( sensor,0x2a7a, 0x0C89);
    _Sensor_WriteReg( sensor,0x2a7c, 0xF37F);
    _Sensor_WriteReg( sensor,0x2a7e, 0x12B0);
    _Sensor_WriteReg( sensor,0x2a80, 0xF89C);
    _Sensor_WriteReg( sensor,0x2a82, 0x4F82);
    _Sensor_WriteReg( sensor,0x2a84, 0x0C8A);
    _Sensor_WriteReg( sensor,0x2a86, 0x425E);
    _Sensor_WriteReg( sensor,0x2a88, 0x00B7);
    _Sensor_WriteReg( sensor,0x2a8a, 0x5E4E);
    _Sensor_WriteReg( sensor,0x2a8c, 0x4EC2);
    _Sensor_WriteReg( sensor,0x2a8e, 0x0CB0);
    _Sensor_WriteReg( sensor,0x2a90, 0x425F);
    _Sensor_WriteReg( sensor,0x2a92, 0x00B8);
    _Sensor_WriteReg( sensor,0x2a94, 0x5F4F);
    _Sensor_WriteReg( sensor,0x2a96, 0x4FC2);
    _Sensor_WriteReg( sensor,0x2a98, 0x0CB1);
    _Sensor_WriteReg( sensor,0x2a9a, 0x480E);
    _Sensor_WriteReg( sensor,0x2a9c, 0x5E0E);
    _Sensor_WriteReg( sensor,0x2a9e, 0x5E0E);
    _Sensor_WriteReg( sensor,0x2aa0, 0x5E0E);
    _Sensor_WriteReg( sensor,0x2aa2, 0x5E0E);
    _Sensor_WriteReg( sensor,0x2aa4, 0x490F);
    _Sensor_WriteReg( sensor,0x2aa6, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2aa8, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2aaa, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2aac, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2aae, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2ab0, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2ab2, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2ab4, 0xDF0E);
    _Sensor_WriteReg( sensor,0x2ab6, 0x4E82);
    _Sensor_WriteReg( sensor,0x2ab8, 0x0A8E);
    _Sensor_WriteReg( sensor,0x2aba, 0xB229);
    _Sensor_WriteReg( sensor,0x2abc, 0x2401);
    _Sensor_WriteReg( sensor,0x2abe, 0x5339);
    _Sensor_WriteReg( sensor,0x2ac0, 0xB3E2);
    _Sensor_WriteReg( sensor,0x2ac2, 0x0080);
    _Sensor_WriteReg( sensor,0x2ac4, 0x2403);
    _Sensor_WriteReg( sensor,0x2ac6, 0x40F2);
    _Sensor_WriteReg( sensor,0x2ac8, 0x0003);
    _Sensor_WriteReg( sensor,0x2aca, 0x00B5);
    _Sensor_WriteReg( sensor,0x2acc, 0x40B2);
    _Sensor_WriteReg( sensor,0x2ace, 0x1000);
    _Sensor_WriteReg( sensor,0x2ad0, 0x7500);
    _Sensor_WriteReg( sensor,0x2ad2, 0x40B2);
    _Sensor_WriteReg( sensor,0x2ad4, 0x1001);
    _Sensor_WriteReg( sensor,0x2ad6, 0x7502);
    _Sensor_WriteReg( sensor,0x2ad8, 0x40B2);
    _Sensor_WriteReg( sensor,0x2ada, 0x0803);
    _Sensor_WriteReg( sensor,0x2adc, 0x7504);
    _Sensor_WriteReg( sensor,0x2ade, 0x40B2);
    _Sensor_WriteReg( sensor,0x2ae0, 0x080F);
    _Sensor_WriteReg( sensor,0x2ae2, 0x7506);
    _Sensor_WriteReg( sensor,0x2ae4, 0x40B2);
    _Sensor_WriteReg( sensor,0x2ae6, 0x6003);
    _Sensor_WriteReg( sensor,0x2ae8, 0x7508);
    _Sensor_WriteReg( sensor,0x2aea, 0x40B2);
    _Sensor_WriteReg( sensor,0x2aec, 0x0801);
    _Sensor_WriteReg( sensor,0x2aee, 0x750A);
    _Sensor_WriteReg( sensor,0x2af0, 0x40B2);
    _Sensor_WriteReg( sensor,0x2af2, 0x0800);
    _Sensor_WriteReg( sensor,0x2af4, 0x750C);
    _Sensor_WriteReg( sensor,0x2af6, 0x40B2);
    _Sensor_WriteReg( sensor,0x2af8, 0x1400);
    _Sensor_WriteReg( sensor,0x2afa, 0x750E);
    _Sensor_WriteReg( sensor,0x2afc, 0x403F);
    _Sensor_WriteReg( sensor,0x2afe, 0x0003);
    _Sensor_WriteReg( sensor,0x2b00, 0x12B0);
    _Sensor_WriteReg( sensor,0x2b02, 0xF72E);
    _Sensor_WriteReg( sensor,0x2b04, 0x421F);
    _Sensor_WriteReg( sensor,0x2b06, 0x0098);
    _Sensor_WriteReg( sensor,0x2b08, 0x821F);
    _Sensor_WriteReg( sensor,0x2b0a, 0x0092);
    _Sensor_WriteReg( sensor,0x2b0c, 0x531F);
    _Sensor_WriteReg( sensor,0x2b0e, 0xC312);
    _Sensor_WriteReg( sensor,0x2b10, 0x100F);
    _Sensor_WriteReg( sensor,0x2b12, 0x4F82);
    _Sensor_WriteReg( sensor,0x2b14, 0x0A86);
    _Sensor_WriteReg( sensor,0x2b16, 0x421F);
    _Sensor_WriteReg( sensor,0x2b18, 0x00AC);
    _Sensor_WriteReg( sensor,0x2b1a, 0x821F);
    _Sensor_WriteReg( sensor,0x2b1c, 0x00A6);
    _Sensor_WriteReg( sensor,0x2b1e, 0x531F);
    _Sensor_WriteReg( sensor,0x2b20, 0x4F82);
    _Sensor_WriteReg( sensor,0x2b22, 0x0A88);
    _Sensor_WriteReg( sensor,0x2b24, 0xB0B2);
    _Sensor_WriteReg( sensor,0x2b26, 0x0010);
    _Sensor_WriteReg( sensor,0x2b28, 0x0A84);
    _Sensor_WriteReg( sensor,0x2b2a, 0x248F);
    _Sensor_WriteReg( sensor,0x2b2c, 0x421E);
    _Sensor_WriteReg( sensor,0x2b2e, 0x068C);
    _Sensor_WriteReg( sensor,0x2b30, 0xC312);
    _Sensor_WriteReg( sensor,0x2b32, 0x100E);
    _Sensor_WriteReg( sensor,0x2b34, 0x4E82);
    _Sensor_WriteReg( sensor,0x2b36, 0x0782);
    _Sensor_WriteReg( sensor,0x2b38, 0x4292);
    _Sensor_WriteReg( sensor,0x2b3a, 0x068E);
    _Sensor_WriteReg( sensor,0x2b3c, 0x0784);
    _Sensor_WriteReg( sensor,0x2b3e, 0xB3D2);
    _Sensor_WriteReg( sensor,0x2b40, 0x0CB6);
    _Sensor_WriteReg( sensor,0x2b42, 0x2418);
    _Sensor_WriteReg( sensor,0x2b44, 0x421A);
    _Sensor_WriteReg( sensor,0x2b46, 0x0CB8);
    _Sensor_WriteReg( sensor,0x2b48, 0x430B);
    _Sensor_WriteReg( sensor,0x2b4a, 0x425F);
    _Sensor_WriteReg( sensor,0x2b4c, 0x0CBA);
    _Sensor_WriteReg( sensor,0x2b4e, 0x4F4E);
    _Sensor_WriteReg( sensor,0x2b50, 0x430F);
    _Sensor_WriteReg( sensor,0x2b52, 0x4E0F);
    _Sensor_WriteReg( sensor,0x2b54, 0x430E);
    _Sensor_WriteReg( sensor,0x2b56, 0xDE0A);
    _Sensor_WriteReg( sensor,0x2b58, 0xDF0B);
    _Sensor_WriteReg( sensor,0x2b5a, 0x421F);
    _Sensor_WriteReg( sensor,0x2b5c, 0x0CBC);
    _Sensor_WriteReg( sensor,0x2b5e, 0x4F0C);
    _Sensor_WriteReg( sensor,0x2b60, 0x430D);
    _Sensor_WriteReg( sensor,0x2b62, 0x421F);
    _Sensor_WriteReg( sensor,0x2b64, 0x0CBE);
    _Sensor_WriteReg( sensor,0x2b66, 0x430E);
    _Sensor_WriteReg( sensor,0x2b68, 0xDE0C);
    _Sensor_WriteReg( sensor,0x2b6a, 0xDF0D);
    _Sensor_WriteReg( sensor,0x2b6c, 0x12B0);
    _Sensor_WriteReg( sensor,0x2b6e, 0xFDF4);
    _Sensor_WriteReg( sensor,0x2b70, 0x4C82);
    _Sensor_WriteReg( sensor,0x2b72, 0x0194);
    _Sensor_WriteReg( sensor,0x2b74, 0xB2A2);
    _Sensor_WriteReg( sensor,0x2b76, 0x0A84);
    _Sensor_WriteReg( sensor,0x2b78, 0x2412);
    _Sensor_WriteReg( sensor,0x2b7a, 0x421E);
    _Sensor_WriteReg( sensor,0x2b7c, 0x0A96);
    _Sensor_WriteReg( sensor,0x2b7e, 0xC312);
    _Sensor_WriteReg( sensor,0x2b80, 0x100E);
    _Sensor_WriteReg( sensor,0x2b82, 0x110E);
    _Sensor_WriteReg( sensor,0x2b84, 0x110E);
    _Sensor_WriteReg( sensor,0x2b86, 0x43C2);
    _Sensor_WriteReg( sensor,0x2b88, 0x0A98);
    _Sensor_WriteReg( sensor,0x2b8a, 0x431D);
    _Sensor_WriteReg( sensor,0x2b8c, 0x4E0F);
    _Sensor_WriteReg( sensor,0x2b8e, 0x9F82);
    _Sensor_WriteReg( sensor,0x2b90, 0x0194);
    _Sensor_WriteReg( sensor,0x2b92, 0x2850);
    _Sensor_WriteReg( sensor,0x2b94, 0x5E0F);
    _Sensor_WriteReg( sensor,0x2b96, 0x531D);
    _Sensor_WriteReg( sensor,0x2b98, 0x903D);
    _Sensor_WriteReg( sensor,0x2b9a, 0x0009);
    _Sensor_WriteReg( sensor,0x2b9c, 0x2BF8);
    _Sensor_WriteReg( sensor,0x2b9e, 0x4292);
    _Sensor_WriteReg( sensor,0x2ba0, 0x0084);
    _Sensor_WriteReg( sensor,0x2ba2, 0x7524);
    _Sensor_WriteReg( sensor,0x2ba4, 0x4292);
    _Sensor_WriteReg( sensor,0x2ba6, 0x0088);
    _Sensor_WriteReg( sensor,0x2ba8, 0x7316);
    _Sensor_WriteReg( sensor,0x2baa, 0x9382);
    _Sensor_WriteReg( sensor,0x2bac, 0x8092);
    _Sensor_WriteReg( sensor,0x2bae, 0x2403);
    _Sensor_WriteReg( sensor,0x2bb0, 0x4292);
    _Sensor_WriteReg( sensor,0x2bb2, 0x008A);
    _Sensor_WriteReg( sensor,0x2bb4, 0x7316);
    _Sensor_WriteReg( sensor,0x2bb6, 0x430E);
    _Sensor_WriteReg( sensor,0x2bb8, 0x421F);
    _Sensor_WriteReg( sensor,0x2bba, 0x0086);
    _Sensor_WriteReg( sensor,0x2bbc, 0x822F);
    _Sensor_WriteReg( sensor,0x2bbe, 0x9F82);
    _Sensor_WriteReg( sensor,0x2bc0, 0x0084);
    _Sensor_WriteReg( sensor,0x2bc2, 0x2801);
    _Sensor_WriteReg( sensor,0x2bc4, 0x431E);
    _Sensor_WriteReg( sensor,0x2bc6, 0x4292);
    _Sensor_WriteReg( sensor,0x2bc8, 0x0086);
    _Sensor_WriteReg( sensor,0x2bca, 0x7314);
    _Sensor_WriteReg( sensor,0x2bcc, 0x93C2);
    _Sensor_WriteReg( sensor,0x2bce, 0x00BC);
    _Sensor_WriteReg( sensor,0x2bd0, 0x2007);
    _Sensor_WriteReg( sensor,0x2bd2, 0xB31E);
    _Sensor_WriteReg( sensor,0x2bd4, 0x2405);
    _Sensor_WriteReg( sensor,0x2bd6, 0x421F);
    _Sensor_WriteReg( sensor,0x2bd8, 0x0084);
    _Sensor_WriteReg( sensor,0x2bda, 0x522F);
    _Sensor_WriteReg( sensor,0x2bdc, 0x4F82);
    _Sensor_WriteReg( sensor,0x2bde, 0x7314);
    _Sensor_WriteReg( sensor,0x2be0, 0x425F);
    _Sensor_WriteReg( sensor,0x2be2, 0x00BC);
    _Sensor_WriteReg( sensor,0x2be4, 0xF37F);
    _Sensor_WriteReg( sensor,0x2be6, 0xFE0F);
    _Sensor_WriteReg( sensor,0x2be8, 0x2406);
    _Sensor_WriteReg( sensor,0x2bea, 0x421E);
    _Sensor_WriteReg( sensor,0x2bec, 0x0086);
    _Sensor_WriteReg( sensor,0x2bee, 0x503E);
    _Sensor_WriteReg( sensor,0x2bf0, 0xFFFB);
    _Sensor_WriteReg( sensor,0x2bf2, 0x4E82);
    _Sensor_WriteReg( sensor,0x2bf4, 0x7524);
    _Sensor_WriteReg( sensor,0x2bf6, 0x430E);
    _Sensor_WriteReg( sensor,0x2bf8, 0x421F);
    _Sensor_WriteReg( sensor,0x2bfa, 0x7524);
    _Sensor_WriteReg( sensor,0x2bfc, 0x9F82);
    _Sensor_WriteReg( sensor,0x2bfe, 0x809A);
    _Sensor_WriteReg( sensor,0x2c00, 0x2C07);
    _Sensor_WriteReg( sensor,0x2c02, 0x9382);
    _Sensor_WriteReg( sensor,0x2c04, 0x8094);
    _Sensor_WriteReg( sensor,0x2c06, 0x2004);
    _Sensor_WriteReg( sensor,0x2c08, 0x9382);
    _Sensor_WriteReg( sensor,0x2c0a, 0x8096);
    _Sensor_WriteReg( sensor,0x2c0c, 0x2001);
    _Sensor_WriteReg( sensor,0x2c0e, 0x431E);
    _Sensor_WriteReg( sensor,0x2c10, 0x40B2);
    _Sensor_WriteReg( sensor,0x2c12, 0x0032);
    _Sensor_WriteReg( sensor,0x2c14, 0x7522);
    _Sensor_WriteReg( sensor,0x2c16, 0x4292);
    _Sensor_WriteReg( sensor,0x2c18, 0x7524);
    _Sensor_WriteReg( sensor,0x2c1a, 0x809A);
    _Sensor_WriteReg( sensor,0x2c1c, 0x930E);
    _Sensor_WriteReg( sensor,0x2c1e, 0x249F);
    _Sensor_WriteReg( sensor,0x2c20, 0x421F);
    _Sensor_WriteReg( sensor,0x2c22, 0x7316);
    _Sensor_WriteReg( sensor,0x2c24, 0xC312);
    _Sensor_WriteReg( sensor,0x2c26, 0x100F);
    _Sensor_WriteReg( sensor,0x2c28, 0x832F);
    _Sensor_WriteReg( sensor,0x2c2a, 0x4F82);
    _Sensor_WriteReg( sensor,0x2c2c, 0x7522);
    _Sensor_WriteReg( sensor,0x2c2e, 0x53B2);
    _Sensor_WriteReg( sensor,0x2c30, 0x7524);
    _Sensor_WriteReg( sensor,0x2c32, 0x3C95);
    _Sensor_WriteReg( sensor,0x2c34, 0x431F);
    _Sensor_WriteReg( sensor,0x2c36, 0x4D0E);
    _Sensor_WriteReg( sensor,0x2c38, 0x533E);
    _Sensor_WriteReg( sensor,0x2c3a, 0x930E);
    _Sensor_WriteReg( sensor,0x2c3c, 0x2403);
    _Sensor_WriteReg( sensor,0x2c3e, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2c40, 0x831E);
    _Sensor_WriteReg( sensor,0x2c42, 0x23FD);
    _Sensor_WriteReg( sensor,0x2c44, 0x4FC2);
    _Sensor_WriteReg( sensor,0x2c46, 0x0A98);
    _Sensor_WriteReg( sensor,0x2c48, 0x3FAA);
    _Sensor_WriteReg( sensor,0x2c4a, 0x4292);
    _Sensor_WriteReg( sensor,0x2c4c, 0x0A86);
    _Sensor_WriteReg( sensor,0x2c4e, 0x0782);
    _Sensor_WriteReg( sensor,0x2c50, 0x421F);
    _Sensor_WriteReg( sensor,0x2c52, 0x0A88);
    _Sensor_WriteReg( sensor,0x2c54, 0x490E);
    _Sensor_WriteReg( sensor,0x2c56, 0x930E);
    _Sensor_WriteReg( sensor,0x2c58, 0x2404);
    _Sensor_WriteReg( sensor,0x2c5a, 0xC312);
    _Sensor_WriteReg( sensor,0x2c5c, 0x100F);
    _Sensor_WriteReg( sensor,0x2c5e, 0x831E);
    _Sensor_WriteReg( sensor,0x2c60, 0x23FC);
    _Sensor_WriteReg( sensor,0x2c62, 0x4F82);
    _Sensor_WriteReg( sensor,0x2c64, 0x0784);
    _Sensor_WriteReg( sensor,0x2c66, 0x3F6B);
    _Sensor_WriteReg( sensor,0x2c68, 0x90F2);
    _Sensor_WriteReg( sensor,0x2c6a, 0x0011);
    _Sensor_WriteReg( sensor,0x2c6c, 0x00BA);
    _Sensor_WriteReg( sensor,0x2c6e, 0x280A);
    _Sensor_WriteReg( sensor,0x2c70, 0x90F2);
    _Sensor_WriteReg( sensor,0x2c72, 0x0051);
    _Sensor_WriteReg( sensor,0x2c74, 0x00BA);
    _Sensor_WriteReg( sensor,0x2c76, 0x2C06);
    _Sensor_WriteReg( sensor,0x2c78, 0x403C);
    _Sensor_WriteReg( sensor,0x2c7a, 0x0A13);
    _Sensor_WriteReg( sensor,0x2c7c, 0x425E);
    _Sensor_WriteReg( sensor,0x2c7e, 0x00BA);
    _Sensor_WriteReg( sensor,0x2c80, 0x535E);
    _Sensor_WriteReg( sensor,0x2c82, 0x3EE4);
    _Sensor_WriteReg( sensor,0x2c84, 0x90F2);
    _Sensor_WriteReg( sensor,0x2c86, 0x0051);
    _Sensor_WriteReg( sensor,0x2c88, 0x00BA);
    _Sensor_WriteReg( sensor,0x2c8a, 0x2804);
    _Sensor_WriteReg( sensor,0x2c8c, 0x90F2);
    _Sensor_WriteReg( sensor,0x2c8e, 0xFF81);
    _Sensor_WriteReg( sensor,0x2c90, 0x00BA);
    _Sensor_WriteReg( sensor,0x2c92, 0x2808);
    _Sensor_WriteReg( sensor,0x2c94, 0x90F2);
    _Sensor_WriteReg( sensor,0x2c96, 0xFF81);
    _Sensor_WriteReg( sensor,0x2c98, 0x00BA);
    _Sensor_WriteReg( sensor,0x2c9a, 0x2807);
    _Sensor_WriteReg( sensor,0x2c9c, 0x90F2);
    _Sensor_WriteReg( sensor,0x2c9e, 0xFF91);
    _Sensor_WriteReg( sensor,0x2ca0, 0x00BA);
    _Sensor_WriteReg( sensor,0x2ca2, 0x2C03);
    _Sensor_WriteReg( sensor,0x2ca4, 0x403C);
    _Sensor_WriteReg( sensor,0x2ca6, 0x0813);
    _Sensor_WriteReg( sensor,0x2ca8, 0x3FE9);
    _Sensor_WriteReg( sensor,0x2caa, 0x90F2);
    _Sensor_WriteReg( sensor,0x2cac, 0xFF91);
    _Sensor_WriteReg( sensor,0x2cae, 0x00BA);
    _Sensor_WriteReg( sensor,0x2cb0, 0x280A);
    _Sensor_WriteReg( sensor,0x2cb2, 0x90F2);
    _Sensor_WriteReg( sensor,0x2cb4, 0xFFB1);
    _Sensor_WriteReg( sensor,0x2cb6, 0x00BA);
    _Sensor_WriteReg( sensor,0x2cb8, 0x2C06);
    _Sensor_WriteReg( sensor,0x2cba, 0x403D);
    _Sensor_WriteReg( sensor,0x2cbc, 0x0060);
    _Sensor_WriteReg( sensor,0x2cbe, 0x425E);
    _Sensor_WriteReg( sensor,0x2cc0, 0x00BA);
    _Sensor_WriteReg( sensor,0x2cc2, 0x536E);
    _Sensor_WriteReg( sensor,0x2cc4, 0x3EC3);
    _Sensor_WriteReg( sensor,0x2cc6, 0x90F2);
    _Sensor_WriteReg( sensor,0x2cc8, 0xFFB1);
    _Sensor_WriteReg( sensor,0x2cca, 0x00BA);
    _Sensor_WriteReg( sensor,0x2ccc, 0x2804);
    _Sensor_WriteReg( sensor,0x2cce, 0x90F2);
    _Sensor_WriteReg( sensor,0x2cd0, 0xFFC1);
    _Sensor_WriteReg( sensor,0x2cd2, 0x00BA);
    _Sensor_WriteReg( sensor,0x2cd4, 0x2808);
    _Sensor_WriteReg( sensor,0x2cd6, 0x90F2);
    _Sensor_WriteReg( sensor,0x2cd8, 0xFFC1);
    _Sensor_WriteReg( sensor,0x2cda, 0x00BA);
    _Sensor_WriteReg( sensor,0x2cdc, 0x280B);
    _Sensor_WriteReg( sensor,0x2cde, 0x90F2);
    _Sensor_WriteReg( sensor,0x2ce0, 0xFFD1);
    _Sensor_WriteReg( sensor,0x2ce2, 0x00BA);
    _Sensor_WriteReg( sensor,0x2ce4, 0x2C07);
    _Sensor_WriteReg( sensor,0x2ce6, 0x403D);
    _Sensor_WriteReg( sensor,0x2ce8, 0x0050);
    _Sensor_WriteReg( sensor,0x2cea, 0x425E);
    _Sensor_WriteReg( sensor,0x2cec, 0x00BA);
    _Sensor_WriteReg( sensor,0x2cee, 0x507E);
    _Sensor_WriteReg( sensor,0x2cf0, 0x0003);
    _Sensor_WriteReg( sensor,0x2cf2, 0x3EAC);
    _Sensor_WriteReg( sensor,0x2cf4, 0x403D);
    _Sensor_WriteReg( sensor,0x2cf6, 0x003C);
    _Sensor_WriteReg( sensor,0x2cf8, 0x403F);
    _Sensor_WriteReg( sensor,0x2cfa, 0x00BA);
    _Sensor_WriteReg( sensor,0x2cfc, 0x4F6E);
    _Sensor_WriteReg( sensor,0x2cfe, 0x526E);
    _Sensor_WriteReg( sensor,0x2d00, 0x90FF);
    _Sensor_WriteReg( sensor,0x2d02, 0xFFFB);
    _Sensor_WriteReg( sensor,0x2d04, 0x0000);
    _Sensor_WriteReg( sensor,0x2d06, 0x2AA2);
    _Sensor_WriteReg( sensor,0x2d08, 0x437E);
    _Sensor_WriteReg( sensor,0x2d0a, 0x3EA0);
    _Sensor_WriteReg( sensor,0x2d0c, 0x425F);
    _Sensor_WriteReg( sensor,0x2d0e, 0x00BA);
    _Sensor_WriteReg( sensor,0x2d10, 0x4F4C);
    _Sensor_WriteReg( sensor,0x2d12, 0x407A);
    _Sensor_WriteReg( sensor,0x2d14, 0x001C);
    _Sensor_WriteReg( sensor,0x2d16, 0x12B0);
    _Sensor_WriteReg( sensor,0x2d18, 0xFDD8);
    _Sensor_WriteReg( sensor,0x2d1a, 0x4E4F);
    _Sensor_WriteReg( sensor,0x2d1c, 0xC312);
    _Sensor_WriteReg( sensor,0x2d1e, 0x104F);
    _Sensor_WriteReg( sensor,0x2d20, 0x114F);
    _Sensor_WriteReg( sensor,0x2d22, 0xF37F);
    _Sensor_WriteReg( sensor,0x2d24, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2d26, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2d28, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2d2a, 0x5F0F);
    _Sensor_WriteReg( sensor,0x2d2c, 0x403E);
    _Sensor_WriteReg( sensor,0x2d2e, 0x00F0);
    _Sensor_WriteReg( sensor,0x2d30, 0x8F0E);
    _Sensor_WriteReg( sensor,0x2d32, 0x3E74);
    _Sensor_WriteReg( sensor,0x2d34, 0x421F);
    _Sensor_WriteReg( sensor,0x2d36, 0x80BA);
    _Sensor_WriteReg( sensor,0x2d38, 0x903F);
    _Sensor_WriteReg( sensor,0x2d3a, 0x0009);
    _Sensor_WriteReg( sensor,0x2d3c, 0x2C04);
    _Sensor_WriteReg( sensor,0x2d3e, 0x531F);
    _Sensor_WriteReg( sensor,0x2d40, 0x4F82);
    _Sensor_WriteReg( sensor,0x2d42, 0x80BA);
    _Sensor_WriteReg( sensor,0x2d44, 0x3E43);
    _Sensor_WriteReg( sensor,0x2d46, 0x421F);
    _Sensor_WriteReg( sensor,0x2d48, 0x80B4);
    _Sensor_WriteReg( sensor,0x2d4a, 0x903F);
    _Sensor_WriteReg( sensor,0x2d4c, 0x0098);
    _Sensor_WriteReg( sensor,0x2d4e, 0x2C04);
    _Sensor_WriteReg( sensor,0x2d50, 0x531F);
    _Sensor_WriteReg( sensor,0x2d52, 0x4F82);
    _Sensor_WriteReg( sensor,0x2d54, 0x80B4);
    _Sensor_WriteReg( sensor,0x2d56, 0x3E3A);
    _Sensor_WriteReg( sensor,0x2d58, 0x4382);
    _Sensor_WriteReg( sensor,0x2d5a, 0x80B4);
    _Sensor_WriteReg( sensor,0x2d5c, 0x3E37);
    _Sensor_WriteReg( sensor,0x2d5e, 0x4E82);
    _Sensor_WriteReg( sensor,0x2d60, 0x8096);
    _Sensor_WriteReg( sensor,0x2d62, 0xD392);
    _Sensor_WriteReg( sensor,0x2d64, 0x7102);
    _Sensor_WriteReg( sensor,0x2d66, 0x4138);
    _Sensor_WriteReg( sensor,0x2d68, 0x4139);
    _Sensor_WriteReg( sensor,0x2d6a, 0x413A);
    _Sensor_WriteReg( sensor,0x2d6c, 0x413B);
    _Sensor_WriteReg( sensor,0x2d6e, 0x4130);
    _Sensor_WriteReg( sensor,0x2d70, 0x0260);
    _Sensor_WriteReg( sensor,0x2d72, 0x0000);
    _Sensor_WriteReg( sensor,0x2d74, 0x0C18);
    _Sensor_WriteReg( sensor,0x2d76, 0x0240);
    _Sensor_WriteReg( sensor,0x2d78, 0x0000);
    _Sensor_WriteReg( sensor,0x2d7a, 0x0260);
    _Sensor_WriteReg( sensor,0x2d7c, 0x0000);
    _Sensor_WriteReg( sensor,0x2d7e, 0x0C05);
    _Sensor_WriteReg( sensor,0x2d80, 0x4130);
    _Sensor_WriteReg( sensor,0x2d82, 0x4382);
    _Sensor_WriteReg( sensor,0x2d84, 0x7602);
    _Sensor_WriteReg( sensor,0x2d86, 0x4F82);
    _Sensor_WriteReg( sensor,0x2d88, 0x7600);
    _Sensor_WriteReg( sensor,0x2d8a, 0x0270);
    _Sensor_WriteReg( sensor,0x2d8c, 0x0000);
    _Sensor_WriteReg( sensor,0x2d8e, 0x0C07);
    _Sensor_WriteReg( sensor,0x2d90, 0x0270);
    _Sensor_WriteReg( sensor,0x2d92, 0x0001);
    _Sensor_WriteReg( sensor,0x2d94, 0x421F);
    _Sensor_WriteReg( sensor,0x2d96, 0x7606);
    _Sensor_WriteReg( sensor,0x2d98, 0x4FC2);
    _Sensor_WriteReg( sensor,0x2d9a, 0x0188);
    _Sensor_WriteReg( sensor,0x2d9c, 0x4130);
    _Sensor_WriteReg( sensor,0x2d9e, 0xDF02);
    _Sensor_WriteReg( sensor,0x2da0, 0x3FFE);
    _Sensor_WriteReg( sensor,0x2da2, 0x430E);
    _Sensor_WriteReg( sensor,0x2da4, 0x930A);
    _Sensor_WriteReg( sensor,0x2da6, 0x2407);
    _Sensor_WriteReg( sensor,0x2da8, 0xC312);
    _Sensor_WriteReg( sensor,0x2daa, 0x100C);
    _Sensor_WriteReg( sensor,0x2dac, 0x2801);
    _Sensor_WriteReg( sensor,0x2dae, 0x5A0E);
    _Sensor_WriteReg( sensor,0x2db0, 0x5A0A);
    _Sensor_WriteReg( sensor,0x2db2, 0x930C);
    _Sensor_WriteReg( sensor,0x2db4, 0x23F7);
    _Sensor_WriteReg( sensor,0x2db6, 0x4130);
    _Sensor_WriteReg( sensor,0x2db8, 0x430E);
    _Sensor_WriteReg( sensor,0x2dba, 0x430F);
    _Sensor_WriteReg( sensor,0x2dbc, 0x3C08);
    _Sensor_WriteReg( sensor,0x2dbe, 0xC312);
    _Sensor_WriteReg( sensor,0x2dc0, 0x100D);
    _Sensor_WriteReg( sensor,0x2dc2, 0x100C);
    _Sensor_WriteReg( sensor,0x2dc4, 0x2802);
    _Sensor_WriteReg( sensor,0x2dc6, 0x5A0E);
    _Sensor_WriteReg( sensor,0x2dc8, 0x6B0F);
    _Sensor_WriteReg( sensor,0x2dca, 0x5A0A);
    _Sensor_WriteReg( sensor,0x2dcc, 0x6B0B);
    _Sensor_WriteReg( sensor,0x2dce, 0x930C);
    _Sensor_WriteReg( sensor,0x2dd0, 0x23F6);
    _Sensor_WriteReg( sensor,0x2dd2, 0x930D);
    _Sensor_WriteReg( sensor,0x2dd4, 0x23F4);
    _Sensor_WriteReg( sensor,0x2dd6, 0x4130);
    _Sensor_WriteReg( sensor,0x2dd8, 0xEE4E);
    _Sensor_WriteReg( sensor,0x2dda, 0x407B);
    _Sensor_WriteReg( sensor,0x2ddc, 0x0009);
    _Sensor_WriteReg( sensor,0x2dde, 0x3C05);
    _Sensor_WriteReg( sensor,0x2de0, 0x100D);
    _Sensor_WriteReg( sensor,0x2de2, 0x6E4E);
    _Sensor_WriteReg( sensor,0x2de4, 0x9A4E);
    _Sensor_WriteReg( sensor,0x2de6, 0x2801);
    _Sensor_WriteReg( sensor,0x2de8, 0x8A4E);
    _Sensor_WriteReg( sensor,0x2dea, 0x6C4C);
    _Sensor_WriteReg( sensor,0x2dec, 0x6D0D);
    _Sensor_WriteReg( sensor,0x2dee, 0x835B);
    _Sensor_WriteReg( sensor,0x2df0, 0x23F7);
    _Sensor_WriteReg( sensor,0x2df2, 0x4130);
    _Sensor_WriteReg( sensor,0x2df4, 0xEF0F);
    _Sensor_WriteReg( sensor,0x2df6, 0xEE0E);
    _Sensor_WriteReg( sensor,0x2df8, 0x4039);
    _Sensor_WriteReg( sensor,0x2dfa, 0x0021);
    _Sensor_WriteReg( sensor,0x2dfc, 0x3C0A);
    _Sensor_WriteReg( sensor,0x2dfe, 0x1008);
    _Sensor_WriteReg( sensor,0x2e00, 0x6E0E);
    _Sensor_WriteReg( sensor,0x2e02, 0x6F0F);
    _Sensor_WriteReg( sensor,0x2e04, 0x9B0F);
    _Sensor_WriteReg( sensor,0x2e06, 0x2805);
    _Sensor_WriteReg( sensor,0x2e08, 0x2002);
    _Sensor_WriteReg( sensor,0x2e0a, 0x9A0E);
    _Sensor_WriteReg( sensor,0x2e0c, 0x2802);
    _Sensor_WriteReg( sensor,0x2e0e, 0x8A0E);
    _Sensor_WriteReg( sensor,0x2e10, 0x7B0F);
    _Sensor_WriteReg( sensor,0x2e12, 0x6C0C);
    _Sensor_WriteReg( sensor,0x2e14, 0x6D0D);
    _Sensor_WriteReg( sensor,0x2e16, 0x6808);
    _Sensor_WriteReg( sensor,0x2e18, 0x8319);
    _Sensor_WriteReg( sensor,0x2e1a, 0x23F1);
    _Sensor_WriteReg( sensor,0x2e1c, 0x4130);
    _Sensor_WriteReg( sensor,0x2e1e, 0x0000);
    _Sensor_WriteReg( sensor,0x2ffe, 0xf000);
    _Sensor_WriteReg( sensor,0x3000, 0x00AE);
    _Sensor_WriteReg( sensor,0x3002, 0x00AE);
    _Sensor_WriteReg( sensor,0x3004, 0x00AE);
    _Sensor_WriteReg( sensor,0x3006, 0x00AE);
    _Sensor_WriteReg( sensor,0x3008, 0x00AE);
    _Sensor_WriteReg( sensor,0x4000, 0x0400);
    _Sensor_WriteReg( sensor,0x4002, 0x0400);
    _Sensor_WriteReg( sensor,0x4004, 0x0C04);
    _Sensor_WriteReg( sensor,0x4006, 0x0C04);
    _Sensor_WriteReg( sensor,0x4008, 0x0C04);
    
    //--- FW End ---//
    
    
    //--- Initial Set file ---//
    _Sensor_WriteReg( sensor,0x0B02, 0x0014);
    _Sensor_WriteReg( sensor,0x0B04, 0x07CB);
    _Sensor_WriteReg( sensor,0x0B06, 0x5ED7);
    _Sensor_WriteReg( sensor,0x0B14, 0x370B);//PLL Main Div[15:8]. 0x1b = Pclk(86.4mhz), 0x37 = Pclk(176mhz)
    _Sensor_WriteReg( sensor,0x0B16, 0x4A0B);
    _Sensor_WriteReg( sensor,0x0B18, 0x0000);
    _Sensor_WriteReg( sensor,0x0B1A, 0x1044);
    
    _Sensor_WriteReg( sensor,0x004C, 0x0100);//tg_enable.
    _Sensor_WriteReg( sensor,0x0032, 0x0101);//Normal
    _Sensor_WriteReg( sensor,0x0036, 0x0048);//ramp_rst_offset
    _Sensor_WriteReg( sensor,0x0038, 0x4800);//ramp_sig_poffset
    _Sensor_WriteReg( sensor,0x0138, 0x0004);//pxl_drv_pwr
    _Sensor_WriteReg( sensor,0x013A, 0x0100);//tx_idle
    _Sensor_WriteReg( sensor,0x0C00, 0x3BC1); //BLC_ctl1. Line BLC on = 0x3b, off = 0x2A //LBLC_ctl1. [0]en_blc, [1]en_lblc_dpc, [2]en_channel_blc, [3]en_adj_pxl_dpc, [4]en_adp_dead_pxl_th
    _Sensor_WriteReg( sensor,0x0C0E, 0x0500);//0x07  BLC display On, 0x05 BLC D off //BLC_ctl3. Frame BLC On = 0x05, off=0x04 //FBLC_ctl3. [0]en_fblc, [1]en_blc_bypass, [2]en_fblc_dpc, [5]en_fobp_dig_offset, [7]en_lobp_dpc_bypass 
    _Sensor_WriteReg( sensor,0x0C10, 0x0510);//dig_blc_offset_h
    _Sensor_WriteReg( sensor,0x0C16, 0x0000);//fobp_dig_b_offset. red b[7] sign(0:+,1:-), b[6:0] dc offset 128(max)
    _Sensor_WriteReg( sensor,0x0C18, 0x0000);//fobp_dig_Gb_offset. Gr b[7] sign(0:+,1:-), b[6:0] dc offset 128(max)
    _Sensor_WriteReg( sensor,0x0C36, 0x0100);//r_g_sum_ctl. [0]:g_sum_en. '1'enable, '0'disable.
    
    //--- MIPI blank time --------------//
    _Sensor_WriteReg( sensor,0x0902, 0x4101);//mipi_value_clk_trail. MIPI CLK mode [1]'1'cont(0x43),'0'non-cont(0x41) [6]'1'2lane(0x41), '0'1lane(0x01) 
    _Sensor_WriteReg( sensor,0x090A, 0x03E4);//mipi_vblank_delay_h.
    _Sensor_WriteReg( sensor,0x090C, 0x0020);//mipi_hblank_short_delay_h.
    _Sensor_WriteReg( sensor,0x090E, 0x0020);//mipi_hblank_long_delay_h.
    _Sensor_WriteReg( sensor,0x0910, 0x5D07);//05 mipi_LPX
    _Sensor_WriteReg( sensor,0x0912, 0x061e);//05 mipi_CLK_prepare
    _Sensor_WriteReg( sensor,0x0914, 0x0407);//02 mipi_clk_pre
    _Sensor_WriteReg( sensor,0x0916, 0x0b0a);//09 mipi_data_zero
    _Sensor_WriteReg( sensor,0x0918, 0x0e09);//0c mipi_clk_post
    //----------------------------------//
    
    //--- Pixel Array Addressing ------//
    _Sensor_WriteReg( sensor,0x000E, 0x0000);//x_addr_start_lobp_h.
    _Sensor_WriteReg( sensor,0x0014, 0x003F);//x_addr_end_lobp_h.
    _Sensor_WriteReg( sensor,0x0010, 0x0050);//x_addr_start_robp_h.
    _Sensor_WriteReg( sensor,0x0016, 0x008F);//x_addr_end_robp_h.
    _Sensor_WriteReg( sensor,0x0012, 0x00AA);
    _Sensor_WriteReg( sensor,0x0018, 0x0ACD);
    _Sensor_WriteReg( sensor,0x0020, 0x0700);//x_regin_sel
    _Sensor_WriteReg( sensor,0x0022, 0x0004);//y_addr_start_fobp_h.
    _Sensor_WriteReg( sensor,0x0028, 0x000B);//y_addr_end_fobp_h.  
    _Sensor_WriteReg( sensor,0x0024, 0xFFFA);//y_addr_start_dummy_h.
    _Sensor_WriteReg( sensor,0x002A, 0xFFFF);//y_addr_end_dummy_h.    
    _Sensor_WriteReg( sensor,0x0026, 0x0016);
    _Sensor_WriteReg( sensor,0x002C, 0x07b1);
    _Sensor_WriteReg( sensor,0x0034, 0x0700);//Y_region_sel
    //------------------------------//
    
    //--Crop size 2592x1944 ----///
    _Sensor_WriteReg( sensor,0x0128, 0x0002);// digital_crop_x_offset_l
    _Sensor_WriteReg( sensor,0x012A, 0x0000);// digital_crop_y_offset_l
    _Sensor_WriteReg( sensor,0x012C, 0x0A20);// digital_crop_image_width
    _Sensor_WriteReg( sensor,0x012E, 0x0798);// digital_crop_image_height
    //------------------------------//
    
    //----< Image FMT Size >--------------------//
    //Image size 2592x1944
    _Sensor_WriteReg( sensor,0x0110, 0x0A20);//X_output_size_h
    _Sensor_WriteReg( sensor,0x0112, 0x0798);//Y_output_size_h
    //------------------------------------------//
    
    //----< Frame / Line Length >--------------//
    _Sensor_WriteReg( sensor,0x0006, 0x07c0);//frame_length_h 1984
    _Sensor_WriteReg( sensor,0x0008, 0x0B40);//line_length_h 2880
    _Sensor_WriteReg( sensor,0x000A, 0x0DB0);//line_length for binning 3504
    //---------------------------------------//
    
    //--- ETC set ----//
    _Sensor_WriteReg( sensor,0x003C, 0x0000);//fixed frame off. b[0] '1'enable, '0'disable
    _Sensor_WriteReg( sensor,0x0000, 0x0100);//orientation. [0]:x-flip, [1]:y-flip.
    _Sensor_WriteReg( sensor,0x0500, 0x0000);//DGA_ctl.  b[1]'0'OTP_color_ratio_disable, '1' OTP_color_ratio_enable, b[2]'0'data_pedestal_en, '1'data_pedestal_dis.
    _Sensor_WriteReg( sensor,0x0700, 0x0590);//Scaler Normal 
    _Sensor_WriteReg( sensor,0x001E, 0x0101);
    _Sensor_WriteReg( sensor,0x0032, 0x0101);
    _Sensor_WriteReg( sensor,0x0A02, 0x0100);// Fast sleep Enable
    _Sensor_WriteReg( sensor,0x0116, 0x003B);// FBLC Ratio
    //----------------//
    
    //--- AG / DG control ----------------//
    //AG
    _Sensor_WriteReg( sensor,0x003A, 0x0000);//Analog Gain.  0x00=x1, 0x70=x8, 0xf0=x16.
    
    //DG
    _Sensor_WriteReg( sensor,0x0508, 0x0100);//DG_Gr_h.  0x01=x1, 0x07=x8.
    _Sensor_WriteReg( sensor,0x050a, 0x0100);//DG_Gb_h.  0x01=x1, 0x07=x8.
    _Sensor_WriteReg( sensor,0x050c, 0x0100);//DG_R_h.    0x01=x1, 0x07=x8.
    _Sensor_WriteReg( sensor,0x050e, 0x0100);//DG_B_h.    0x01=x1, 0x07=x8.
    //----------------------------------//
    
    
    //-----< Exp.Time >------------------------//
    // Pclk_88Mhz @ Line_length_pclk : 2880 @Exp.Time 33.33ms
    _Sensor_WriteReg( sensor,0x0002, 0x04b0);    //Fine_int : 33.33ms@Pclk88mhz@Line_length2880 
    _Sensor_WriteReg( sensor,0x0004, 0x07F4);//coarse_int : 33.33ms@Pclk88mhz@Line_length2880
    
    //--- ISP enable Selection ---------------//
    _Sensor_WriteReg( sensor,0x0A04, 0x011B); //isp_en. [9]s-gamma,[8]MIPI_en,[6]compresion10to8,[5]Scaler,[4]window,[3]DG,[2]LSC,[1]adpc,[0]tpg
    
    //----------------------------------------//

    _Sensor_WriteReg( sensor,0x0118, 0x0100);//sleep Off

    _Sensor_WriteReg( sensor,0x0118, 0x0000); //sleep On
    msleep(10);
    _Sensor_WriteReg( sensor,0x0F02, 0x0000); //pll disable
    _Sensor_WriteReg( sensor,0x011A, 0x0109); //CP TRI_H IPGM TRIM_H
    _Sensor_WriteReg( sensor,0x0D04, 0x0100); //Fsync Output enable
    _Sensor_WriteReg( sensor,0x0D00, 0x0007); //Fsync Output Drivability
    _Sensor_WriteReg( sensor,0x004C, 0x0100); //TG MCU enable
    _Sensor_WriteReg( sensor,0x003E, 0x0100); //OTP R/W
    _Sensor_WriteReg( sensor,0x0118, 0x0100); //sleep off
    msleep(10);
}
    
static void _get_hi545_sensor_otp_info( struct b52_sensor *sensor )
{
    uint16_t   Year = 0, 
    Month = 0, 
    Day = 0, 
    LensID = 0, 
    VCMID = 0, 
    DriverID = 0;
    uint16_t info_flag = 0, 
    infocheck = 0, 
    checksum = 0;

    printk("********************************************\n ");

    if (_hi545_Sensor_OTP_read(sensor,0x1801) == 0x40)
        {info_flag = 1;}
    else if (_hi545_Sensor_OTP_read(sensor,0x1801) == 0xd0)
        {info_flag = 2;}
    else if (_hi545_Sensor_OTP_read(sensor,0x1801) == 0xf4)
        {info_flag = 3;}
    
    switch (info_flag)
    {    
        case 1:
                ModuleHouseID = _hi545_Sensor_OTP_read(sensor,0x1802);
                Year   = _hi545_Sensor_OTP_read(sensor,0x1804);
                Month  = _hi545_Sensor_OTP_read(sensor,0x1805);
                Day    = _hi545_Sensor_OTP_read(sensor,0x1806);
                LensID = _hi545_Sensor_OTP_read(sensor,0x1808);
                VCMID  = _hi545_Sensor_OTP_read(sensor,0x1809);
                DriverID = _hi545_Sensor_OTP_read(sensor,0x180A);
                break;
        case 2:
                ModuleHouseID = _hi545_Sensor_OTP_read(sensor,0x1821);
                Year   = _hi545_Sensor_OTP_read(sensor,0x1814);
                Month  = _hi545_Sensor_OTP_read(sensor,0x1815);
                Day    = _hi545_Sensor_OTP_read(sensor,0x1816);
                LensID = _hi545_Sensor_OTP_read(sensor,0x1818);
                VCMID  = _hi545_Sensor_OTP_read(sensor,0x1819);
                DriverID = _hi545_Sensor_OTP_read(sensor,0x181A);
                break;
        case 3:
                ModuleHouseID = _hi545_Sensor_OTP_read(sensor,0x1822);
                Year   = _hi545_Sensor_OTP_read(sensor,0x1824);
                Month  = _hi545_Sensor_OTP_read(sensor,0x1825);
                Day    = _hi545_Sensor_OTP_read(sensor,0x1826);
                LensID = _hi545_Sensor_OTP_read(sensor,0x1828);
                VCMID  = _hi545_Sensor_OTP_read(sensor,0x1829);
                DriverID = _hi545_Sensor_OTP_read(sensor,0x182A);
                break;
        default:        
                printk("_hi545_Sensor: info_flag error value: 0x%x \n ",info_flag);
                break;
        
    }

    printk("ModuleHouseID = 0x%x \n", ModuleHouseID);
    printk("Year = 0x%x, Month = 0x%x, Day = 0x%x\n", Year, Month, Day);
    printk("LensID = 0x%x \n", LensID);
    printk("********************************************\n");
}
static int _hi545_update_otp(struct b52_sensor *sensor)
{
    if( !ModuleHouseID )
    {
        _hi545_OTPSetting(sensor);
        _get_hi545_sensor_otp_info(sensor);
    }

    if(  ModuleHouseID == sensor->id_modulehouse)
        return 2 ;
    else
        return 0 ;
}
//************read hi545 otp function end*****************************************//

static int b52_sensor_get_power(struct device * dcam,struct b52_sensor *sensor)
{
    if( sensor->power.af_2v8_name )
    {
        sensor->power.af_2v8 = regulator_get(dcam, "vddcammot");
        if (IS_ERR(sensor->power.af_2v8))
        {
            printk("Failed to get regulator vddcammot\n");
            sensor->power.af_2v8 = NULL;
            return -1 ;
        }
    }

    if(sensor->power.avdd_2v8_name)
    {
        sensor->power.avdd_2v8 = regulator_get(dcam, "vddcama");
        if (IS_ERR(sensor->power.avdd_2v8))
        {
            printk("Failed to get regulator vddcama\n");
            sensor->power.avdd_2v8 = NULL;
            return -1 ;
        }
    }
    if(sensor->power.dovdd_1v8_name)
    {
        sensor->power.dovdd_1v8 = regulator_get(dcam, "vddcamio");
        if (IS_ERR(sensor->power.dovdd_1v8))
        {
            printk("Failed to get regulator vddcamio\n");
            sensor->power.dovdd_1v8 = NULL;
            return -1 ;
        }
    }
    if(sensor->power.dvdd_1v2_name)
    {
        sensor->power.dvdd_1v2 = regulator_get(dcam, "vddcamd");
        if (IS_ERR(sensor->power.dvdd_1v2))
        {
            printk("Failed to get regulator vddcamd\n");
            sensor->power.dvdd_1v2 = NULL;
            return -1 ;
        }
    }
    return 0;
}

static int b52_sensor_put_power(struct device * dcam,struct b52_sensor *sensor)
{    
    if ( sensor->power.pwdn )
    {
        gpio_free(sensor->power.pwdn);
        sensor->power.pwdn = 0 ;
    }
    if ( sensor->power.rst )
    {
        gpio_free(sensor->power.rst);
        sensor->power.rst = 0 ;
    }
    if (sensor->power.af_2v8)
    {
        regulator_put(sensor->power.af_2v8);
        sensor->power.af_2v8_name = NULL ;
    }
    if (sensor->power.avdd_2v8)
    {
        regulator_put(sensor->power.avdd_2v8);
        sensor->power.avdd_2v8_name = NULL ;
    }
    if (sensor->power.dovdd_1v8)
        regulator_put(sensor->power.dovdd_1v8);
        sensor->power.dovdd_1v8_name = NULL ;

    if (sensor->power.dvdd_1v2)
    {
        regulator_put(sensor->power.dvdd_1v2);
        sensor->power.dvdd_1v2_name = NULL ;
    }
    return 0;
}

static int b52_sensor_s_power(struct b52_sensor *sensor,int on, struct sensor_file_tag* p_file)
{
    int ret = 0;
    struct sensor_power *power = (struct sensor_power *) &(sensor->power);

    gpio_request(power->pwdn, "CAM_PDWD");
    gpio_request(power->rst,  "CAM_RESET");

    if ( on )
    {
        if (power->enable++ > 0)
            return 0;
        
        if (power->af_2v8)
        {
            regulator_set_voltage(power->af_2v8,2800000, 2800000);
            ret = regulator_enable(power->af_2v8);
            if (ret < 0)
                goto af_err;
        }
        
        if (power->avdd_2v8)
        {
            regulator_set_voltage(power->avdd_2v8,2800000, 2800000);
            ret = regulator_enable(power->avdd_2v8);
            if (ret < 0)
                goto avdd_err;
        }

        if (power->dovdd_1v8)
        {
            regulator_set_voltage(power->dovdd_1v8,
                                  1800000, 1800000);
            ret = regulator_enable(power->dovdd_1v8);
            if (ret < 0)
                goto dovdd_err;
        }

        if (power->dvdd_1v2)
        {
            regulator_set_voltage(power->dvdd_1v2,
                                  1200000, 1200000);
            ret = regulator_enable(power->dvdd_1v2);
            if (ret < 0)
                goto dvdd_err;
        }

        if ( power->rst && power->pwdn )
        {
            if( !strcmp(sensor->sensor_name,"hi545") )
            {
                printk("[kernel]sensor config gpio %d name hi545----\n",power->pwdn);
                gpio_direction_output(power->pwdn, !power->pwdn_value);
                gpio_direction_output(power->rst,  !power->rst_value);
                gpio_direction_output(power->pwdn, power->pwdn_value);
                
                ret = set_mclk(sensor->mclk,p_file);
                if (ret < 0)
                    goto af_err;
                msleep(10);
                gpio_direction_output(power->rst, power->rst_value);
                msleep(16);
            }
            else if( !strcmp(sensor->sensor_name,"sp0a20") )
            {
                printk("[kernel]sensor config gpio %d sp0a20----\n",power->pwdn);
                ret = set_mclk(sensor->mclk,p_file);
                if (ret < 0)
                    goto af_err;
                gpio_direction_output(power->pwdn, 0);
                gpio_direction_output(power->pwdn, 1);
                msleep(1);
                gpio_direction_output(power->pwdn, 0);
                msleep(10);
            }
        }
        
    }
    else
    {
        if (WARN_ON(power->enable == 0))
            return -EINVAL;

        if (--power->enable > 0)
            return 0;

        if (power->rst)
            gpio_direction_output(power->rst, !power->rst_value);
        if (power->pwdn)
            gpio_direction_output(power->pwdn, !power->pwdn_value);
        
        if (power->dvdd_1v2)
            regulator_disable(power->dvdd_1v2);
        if (power->avdd_2v8)
            regulator_disable(power->avdd_2v8);
        if (power->dovdd_1v8)
            regulator_disable(power->dovdd_1v8);
        if (power->af_2v8)
            regulator_disable(power->af_2v8);

        ret = set_mclk(0,p_file);
    }
    return ret;
af_err:
    regulator_disable(power->dvdd_1v2);
dvdd_err:
    regulator_disable(power->dovdd_1v8);
dovdd_err:
    regulator_disable(power->avdd_2v8);
avdd_err:
    return ret;
}

static int b52_detect_sensor(struct device * dcam, char *name, char *camera_name)
{
    int ret ;
    u32 nr;
    uint16_t data = 0 ;
    struct device_node *subdev_np = NULL, *sensor_np = NULL , *power_np = NULL;
    struct b52_sensor *sensor;
    struct sensor_file_tag* p_file = NULL ;

    
    sensor_np = of_get_child_by_name(dcam->of_node, name);
    if (sensor_np == NULL)
    {
        printk("[kernel] %s No of_get_child_by_name\n",    __func__);
        return NULL;
    }
    
    sensor = kzalloc( sizeof(struct b52_sensor), GFP_KERNEL);
    if (!sensor)
        return -ENOMEM;

    do {
        subdev_np = of_get_next_available_child(sensor_np,subdev_np);
        if (subdev_np == NULL) {
            printk("[kernel] %s No sensor need to be registered\n",    __func__);
            return NULL;
        }
        sensor->dev_np = subdev_np ;

        ret = of_property_read_string(subdev_np,"sensor-name", &sensor->sensor_name);
        if (ret < 0) {
            printk("[kernel] %s Unable to get sensor full name\n", __func__);
            return NULL;
        }

        ret = of_property_read_string(subdev_np,"modulehouse-name", &sensor->modulehouse_name);
        if (ret < 0) {
            printk("[kernel] %s Unable to get modulehouse full name\n", __func__);
        }

        ret = of_property_read_u32(subdev_np, "id_modulehouse", &sensor->id_modulehouse);    
        if (ret < 0)    
        {        
            printk("[kernel]%d: get id modulehouse_name failed\n", __LINE__);        
        }    

        ret = of_property_read_string(subdev_np,"af_2v8-supply", &sensor->power.af_2v8_name);
        if (ret < 0) {
            printk("[kernel] %s Unable to get af_2v8-supply full name\n", __func__);
        }
        ret = of_property_read_string(subdev_np,"avdd_2v8-supply", &sensor->power.avdd_2v8_name);
        if (ret < 0) {
            printk("[kernel] %s Unable to get avdd_2v8-supply full name\n", __func__);
        }
        ret = of_property_read_string(subdev_np,"dovdd_1v8-supply", &sensor->power.dovdd_1v8_name);
        if (ret < 0) {
            printk("[kernel] %s Unable to get dovdd_1v8-supply full name\n", __func__);
        }
        ret = of_property_read_string(subdev_np,"dvdd_1v2-supply", &sensor->power.dvdd_1v2_name);
        if (ret < 0) {
            printk("[kernel] %s Unable to get dvdd_1v2-supply full name\n", __func__);
        }

        ret = of_property_read_u32(subdev_np,"adapter", &nr);
        if (ret < 0) {
            printk("[kernel] %s Unable to get I2C bus number\n", __func__);
        }

        sensor->adapter = i2c_get_adapter(nr);
        if (sensor->adapter == NULL) {
            printk("[kernel]%s:Unable to get I2C adapter %d for device\n",__func__,nr);
        }
        
        ret = of_property_read_u32(subdev_np,"mclk", &sensor->mclk);
        if (ret < 0) {
            printk("[kernel] %s Unable to get sensor full name\n", __func__);
        }
        
        ret = of_property_read_u32(subdev_np,"register", &sensor->reg);
        if (ret < 0) {
            printk("[kernel]%s Unable to get I2C address\n", __func__);
        }

        ret = of_property_read_u32(subdev_np, "id_regbit", &sensor->id_regbit);    
        if (ret < 0)    
        {        
            printk("[kernel]%x: of_get_id_regbit failed\n", __LINE__);        
        }    

        ret = of_property_read_u32(subdev_np, "id_reg", &sensor->id_reg);    
        if (ret < 0)    
        {        
            printk("[kernel]%x: of_get_id_reg failed\n", __LINE__);        
        }    
        else    
        {        
            ret = of_property_read_u32(subdev_np,"id_validvalue", &sensor->id_validvalue);        
            if (ret < 0)       
                printk("[kernel]%x: of_get_id_validvaule failed\n", __LINE__);           
        }

        power_np = of_get_child_by_name(subdev_np, "sensor_power");
        
        ret = of_property_read_u32(power_np, "pwdn-gpios", &sensor->power.pwdn);
        if (ret < 0)
            printk("%x: of_get_named_gpio failed\n", __LINE__);
        else
        {
            ret = of_property_read_u32(power_np,"pwdn-validvalue", &sensor->power.pwdn_value);
            if (ret < 0)
            {
                printk("%x: of_get_gpio_value failed\n", __LINE__);
            }
        }
        
        ret = of_property_read_u32(power_np, "reset-gpios", &sensor->power.rst);
        if (ret < 0)
            printk("%x: of_get_named_gpio failed\n", __LINE__);
        else
        {
            ret = of_property_read_u32(power_np,"reset-validvalue", &sensor->power.rst_value);
            if (ret < 0)
            {
                printk("%x: of_get_gpio_value failed\n", __LINE__);
            }
        }

        p_file = (struct sensor_file_tag *)vzalloc(sizeof(struct sensor_file_tag));

        b52_sensor_get_power(dcam,sensor);
        b52_sensor_s_power(sensor,1,p_file);

       data =  _Sensor_ReadReg(sensor,sensor->id_reg) ; 
       if( sensor->id_validvalue == data )
           ret = 2;

        if( data == 0x4505 ) //hi545 sensor
            ret = _hi545_update_otp( sensor );

        b52_sensor_s_power(sensor,0,p_file);
        b52_sensor_put_power(dcam,sensor);
        i2c_put_adapter(sensor->adapter);

        vfree(p_file);
        if (ret == 2){
            strcpy(camera_name, sensor->modulehouse_name);
            printk("[kernel] find %s sensor is %s \n",name,sensor->modulehouse_name);
            break;
        }
    } while (subdev_np != NULL);

    kfree(sensor);
    return ret;
}
static int plat_setup_sensor(struct device *dcam)
{
    int ret ;
    
    ret = b52_detect_sensor(dcam, "backsensor", back_camera);
    if (ret != 2)
        pr_info("plat detect back sensor failed\n");
    

    ret = b52_detect_sensor(dcam, "frontsensor", front_camera);
    if (ret != 2)
        pr_info("detect front sensor failed\n");

    if(!camera_back_probe_ok)//add by liuwei
        camera_back_probe_ok=1;//add by liuwei
        
    if(!camera_front_probe_ok)//add by liuwei
        camera_front_probe_ok=1;//add by liuwei

    return ret;
}

static int plat_cam_remove(struct platform_device *pdev)
{
    return 0;
}   
static int plat_cam_probe(struct platform_device *pdev)
{
    printk("[kernel]***start read Camera Info ***\n");
    if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
        return -EIO;
 
    plat_setup_sensor(&pdev->dev);
    
    REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
    printk("[kernel]***Stop read Camera Info ***\n");
    return 0;
}
static const struct of_device_id plat_cam_dt_match[] = {
    { .compatible = PLAT_CAM_DRV, .data = NULL },
    {},
};
MODULE_DEVICE_TABLE(of, plat_isp_dt_match);

static struct platform_driver plat_cam_driver = {
    .probe = plat_cam_probe,
    .remove = plat_cam_remove,
    .driver = {
        .owner    = THIS_MODULE,
        .name    = PLAT_CAM_DRV,
        .of_match_table = of_match_ptr(plat_cam_dt_match)
    },
};

module_platform_driver(plat_cam_driver);

MODULE_AUTHOR("ontim_camera");
MODULE_DESCRIPTION("Camera Platform Level Driver");
MODULE_LICENSE("GPL");
