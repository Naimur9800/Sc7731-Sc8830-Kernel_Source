#include "zte_lcd_common.h"

#define DTYPE_GEN_READ		0x04	/* long read, 0 parameter */
#define DTYPE_GEN_WRITE	0x03	/* short write, 0 parameter */

#define DTYPE_DCS_READ		0x06	/* read */
#define DTYPE_DCS_WRITE	0x05	/* short write, 0 parameter */

/*
*echo ff988100 > gwrite (0x13,0x29) or echo 51ff > dwrite (0x15,0x39)
*echo 5401 > gread(0x14,0x24), then cat gread
*dread (0x06) sometimes read nothing,return error
*file path: sys/lcd_reg_debug
*/

struct zte_lcd_reg_debug zte_lcd_reg_debug;
extern struct sprd_dispc *g_zte_ctrl_pdata;
#define SYSFS_FOLDER_NAME "reg_debug"

static void zte_lcd_reg_rw_func(struct sprd_dispc *ctrl, struct zte_lcd_reg_debug *reg_debug)
{
	int read_length = -1;
	struct dsi_cmd_desc write_lcd_cmd;
	struct sprd_dsi *dsi = dev_get_drvdata(g_zte_ctrl_pdata->zte_lcd_ctrl->panel_info->pd->intf);

	if ((!reg_debug) || (!ctrl))
		return;

	write_lcd_cmd.header.data_type = reg_debug->dtype;
	write_lcd_cmd.header.wait = 5;/*5ms*/
	write_lcd_cmd.header.len = reg_debug->length;
	write_lcd_cmd.payload = (char *)reg_debug->wbuf;

	
	#ifdef ZTE_LCD_REG_DEBUG
	for (i = 0; i < reg_debug->length; i++)
		ZTE_LCD_INFO("rwbuf[%d]= %x\n", i, reg_debug->wbuf[i]);
	#endif

	switch (reg_debug->is_read_mode) {
	case REG_READ_MODE:
		read_length = reg_debug->wbuf[1];
		mipi_dsi_set_max_return_size(dsi, read_length);
		if (zte_lcd_reg_debug.dtype == DTYPE_DCS_READ)
			mipi_dsi_dcs_read(dsi, reg_debug->wbuf[0], reg_debug->rbuf, read_length);
		else
			mipi_dsi_gen_read(dsi, &reg_debug->wbuf[0], 1, reg_debug->rbuf, read_length);

		break;
	case REG_WRITE_MODE:
		if (zte_lcd_reg_debug.dtype == DTYPE_DCS_WRITE)
			mipi_dsi_dcs_write(dsi, write_lcd_cmd.payload, write_lcd_cmd.header.len);
		else
			mipi_dsi_gen_write(dsi, write_lcd_cmd.payload, write_lcd_cmd.header.len);

		break;
	default:
		ZTE_LCD_ERROR("%s:rw error\n", __func__);
		break;
	}

}

static void get_user_sapce_data(const char *buf, size_t count)
{
	int i = 0, length = 0;
	char lcd_status[ZTE_REG_LEN*2] = { 0 };

	if (count >= sizeof(lcd_status)) {
		ZTE_LCD_INFO("count=%zu,sizeof(lcd_status)=%zu\n", count, sizeof(lcd_status));
		return;
	}

	strlcpy(lcd_status, buf, count);
	memset(zte_lcd_reg_debug.wbuf, 0, ZTE_REG_LEN);
	memset(zte_lcd_reg_debug.rbuf, 0, ZTE_REG_LEN);

	
	#ifdef ZTE_LCD_REG_DEBUG
	for (i = 0; i < count; i++)
		ZTE_LCD_INFO("lcd_status[%d]=%c  %d\n", i, lcd_status[i], lcd_status[i]);
	#endif
	for (i = 0; i < count; i++) {
		if (isdigit(lcd_status[i]))
			lcd_status[i] -= '0';
		else if (isalpha(lcd_status[i]))
			lcd_status[i] -= (isupper(lcd_status[i]) ? 'A' - 10 : 'a' - 10);
	}
	for (i = 0, length = 0; i < (count-1); i = i+2, length++) {
		zte_lcd_reg_debug.wbuf[length] = lcd_status[i]*16 + lcd_status[1+i];
	}

	zte_lcd_reg_debug.length = length; /*length is use space write data number*/
}

static ssize_t sysfs_show_read(struct device *d, struct device_attribute *attr, char *buf)
{
	int i = 0, len = 0, count = 0;
	char *s = NULL;
	char *data_buf = NULL;

	data_buf = kzalloc(ZTE_REG_LEN * REG_MAX_LEN, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	s = data_buf;
	for (i = 0; i < zte_lcd_reg_debug.length; i++) {
		len = snprintf(s, 20, "rbuf[%02d]=%02x ", i, zte_lcd_reg_debug.rbuf[i]);
		s += len;
		if ((i+1)%8 == 0) {
			len = snprintf(s, 20, "\n");
			s += len;
		}
	}

	count = snprintf(buf, PAGE_SIZE, "read back:\n%s\n", data_buf);
	kfree(data_buf);
	return count;
}
static ssize_t sysfs_store_dread(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i = 0, length = 0;

	get_user_sapce_data(buf, count);
	length = zte_lcd_reg_debug.wbuf[1];
	if (length < 1) {
		ZTE_LCD_ERROR("%s:read length is 0\n", __func__);
		return count;
	}

	zte_lcd_reg_debug.is_read_mode = REG_READ_MODE;
	zte_lcd_reg_debug.dtype = DTYPE_DCS_READ;

	ZTE_LCD_INFO("dtype = %x read cmd = %x length = %x\n", zte_lcd_reg_debug.dtype,
		zte_lcd_reg_debug.wbuf[0], length);
	zte_lcd_reg_rw_func(g_zte_ctrl_pdata, &zte_lcd_reg_debug);

	zte_lcd_reg_debug.length = length;
	for (i = 0; i < length; i++)
		ZTE_LCD_INFO("read zte_lcd_reg_debug.rbuf[%d]=0x%02x\n", i, zte_lcd_reg_debug.rbuf[i]);

	return count;
}

static ssize_t sysfs_store_gread(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i = 0, length = 0;

	get_user_sapce_data(buf, count);
	length = zte_lcd_reg_debug.wbuf[1];
	if (length < 1) {
		ZTE_LCD_INFO("%s:read length is 0\n", __func__);
		return count;
	}

	zte_lcd_reg_debug.is_read_mode = REG_READ_MODE; /* if 1 read ,0 write*/

	zte_lcd_reg_debug.dtype = DTYPE_GEN_READ;

	ZTE_LCD_INFO("dtype = %x read cmd = %x num = %x\n", zte_lcd_reg_debug.dtype, zte_lcd_reg_debug.wbuf[0], length);
	zte_lcd_reg_rw_func(g_zte_ctrl_pdata, &zte_lcd_reg_debug);

	zte_lcd_reg_debug.length = length;
	for (i = 0; i < length; i++)
		ZTE_LCD_INFO("read zte_lcd_reg_debug.rbuf[%d]=0x%02x\n", i, zte_lcd_reg_debug.rbuf[i]);

	return count;
}

static ssize_t sysfs_store_dwrite(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int length = 0;

	get_user_sapce_data(buf, count);
	length = zte_lcd_reg_debug.length;

	zte_lcd_reg_debug.is_read_mode = REG_WRITE_MODE; /* if 1 read ,0 write*/

	zte_lcd_reg_debug.dtype = DTYPE_DCS_WRITE;

	zte_lcd_reg_rw_func(g_zte_ctrl_pdata, &zte_lcd_reg_debug);
	ZTE_LCD_INFO("dtype = 0x%02x,write cmd = 0x%02x,length = 0x%02x\n", zte_lcd_reg_debug.dtype,
		zte_lcd_reg_debug.wbuf[0], length);

	return count;
}

static ssize_t sysfs_store_gwrite(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int length = 0;

	get_user_sapce_data(buf, count);
	length = zte_lcd_reg_debug.length;

	zte_lcd_reg_debug.is_read_mode = REG_WRITE_MODE;

	zte_lcd_reg_debug.dtype = DTYPE_GEN_WRITE;

	zte_lcd_reg_rw_func(g_zte_ctrl_pdata, &zte_lcd_reg_debug);
	ZTE_LCD_INFO("dtype = 0x%02x write cmd = 0x%02x length = 0x%02x\n", zte_lcd_reg_debug.dtype,
		zte_lcd_reg_debug.wbuf[0], length);

	return count;
}

static ssize_t sysfs_store_mipiclk(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value = 0, tmp = -1, ret = count;

	tmp = kstrtoint(buf, 10, &value);
	if (tmp) {
		ZTE_LCD_ERROR("kstrtouint error!\n");
		ret = tmp;
	} else {
		ZTE_LCD_INFO("you can read/write /sys/class/display/dphy0/freq to modify mipiclk\n");
		ZTE_LCD_INFO("count=%zu value=%d\n", count, value);
	}

	return ret;
}

static ssize_t sysfs_show_reserved(struct device *d, struct device_attribute *attr, char *buf)
{
	int i = 0, len = 0, count = 0;
	char *s = NULL;
	char *data_buf = NULL;

	data_buf = kzalloc(ZTE_REG_LEN * REG_MAX_LEN, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	s = data_buf;
	for (i = 0; i < zte_lcd_reg_debug.length; i++) {
		len = snprintf(s, 20, "rbuf[%02d]=%02x ", i, zte_lcd_reg_debug.wbuf[i]);
		s += len;
	if ((i+1)%8 == 0) {
			len = snprintf(s, 20, "\n");
			s += len;
		}
	}
	len = snprintf(s, 100, "\n%s", zte_lcd_reg_debug.reserved);
	s += len;

	count = snprintf(buf, PAGE_SIZE, "read back:\n%s\n", data_buf);
	kfree(data_buf);
	return count;
}

static ssize_t sysfs_store_reserved(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int i = 0, value = 0;
	int tmp = -1;

	get_user_sapce_data(buf, count);
	for (i = 0; i < zte_lcd_reg_debug.length; i++)
		ZTE_LCD_INFO("write data [%d]=0x%02x\n", i, zte_lcd_reg_debug.wbuf[i]);

	tmp = kstrtouint(buf, 10, &value);
	if (tmp) {
		ZTE_LCD_ERROR("kstrtouint error!\n");
	} else {
		ZTE_LCD_INFO("count=%zu value=%d\n", count, value);
		snprintf(zte_lcd_reg_debug.reserved, ZTE_REG_LEN, "reserved str=%d", value);
	}
/******************************* add code here ************************************************/
	return count;
}
static DEVICE_ATTR(dread, 0600, sysfs_show_read, sysfs_store_dread);
static DEVICE_ATTR(gread, 0600, sysfs_show_read, sysfs_store_gread);
static DEVICE_ATTR(dwrite, 0600, NULL, sysfs_store_dwrite);
static DEVICE_ATTR(gwrite, 0600, NULL, sysfs_store_gwrite);
static DEVICE_ATTR(mipiclk, 0600, NULL, sysfs_store_mipiclk);
static DEVICE_ATTR(reserved, 0600, sysfs_show_reserved, sysfs_store_reserved);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_dread.attr,
	&dev_attr_gread.attr,
	&dev_attr_dwrite.attr,
	&dev_attr_gwrite.attr,
	&dev_attr_mipiclk.attr,
	&dev_attr_reserved.attr,
	NULL,
};

static struct attribute_group sysfs_attr_group = {
	.attrs = sysfs_attrs,
};

void zte_lcd_reg_debug_func(void)
{
	int ret = -1;

	struct kobject *vkey_obj = NULL;

	vkey_obj = kobject_create_and_add(SYSFS_FOLDER_NAME, g_zte_ctrl_pdata->zte_lcd_ctrl->kobj);
	if (!vkey_obj) {
		ZTE_LCD_ERROR("%s:unable to create kobject\n", __func__);
		return;
	}

	ret = sysfs_create_group(vkey_obj, &sysfs_attr_group);
	if (ret) {
		ZTE_LCD_ERROR("%s:failed to create attributes\n", __func__);
	}
}
