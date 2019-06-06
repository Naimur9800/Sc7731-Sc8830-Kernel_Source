#include "zte_lcd_common.h"

#ifdef CONFIG_ZTE_LCD_REG_DEBUG
extern void zte_lcd_reg_debug_func(void);
#endif
extern int mipi_dsi_send_cmds(struct sprd_dsi *dsi, struct panel_cmds *in_cmds);
extern int of_parse_cmds(struct device_node *np,
			struct panel_cmds **in_pcmds, const char *name);

struct sprd_dispc *g_zte_ctrl_pdata = NULL;
const char *zte_get_lcd_panel_name(void)
{
	if (g_zte_ctrl_pdata == NULL)
		return "no_panel_info";
	else
		return g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_panel_name;
}

static int zte_lcd_proc_info_show(struct seq_file *m, void *v)
{
	if (g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_panel_name) {
		seq_printf(m, "panel name=%s\n", g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_panel_name);
	} else {
		seq_printf(m, "%s\n", "panel name is not detect");
	}

	if (g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_init_code_version) {
		seq_printf(m, "version=%s\n", g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_init_code_version);
	} else {
		seq_printf(m, "%s\n", "version is not detect");
	}

	return 0;
}
static int zte_lcd_proc_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, zte_lcd_proc_info_show, NULL);
}
static const struct file_operations zte_lcd_common_func_proc_fops = {
	.owner = THIS_MODULE,
	.open = zte_lcd_proc_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static int zte_lcd_proc_info_display(struct panel_info *panel_info)
{
	struct proc_dir_entry *proc_lcd_id = NULL;

	proc_lcd_id = proc_create_data("driver/lcd_id", 0, NULL, &zte_lcd_common_func_proc_fops, NULL);
	if (!proc_lcd_id) {
		ZTE_LCD_ERROR("%s:create driver/lcd_id error!\n", __func__);
		return -EPERM;
	}

	g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_panel_name = panel_info->lcd_name;
	ZTE_LCD_INFO("%s panel_name = %s\n", __func__, panel_info->lcd_name);

	g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_init_code_version = of_get_property(panel_info->of_node,
		"zte,lcd-init-code-version", NULL);

	ZTE_LCD_INFO("%s:Panel Name = %s,init code version=%s\n", __func__,
		g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_panel_name,
		g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_init_code_version);
	return 0;
}


#ifdef CONFIG_ZTE_LCD_CABC3_EXTREME_POWER_SAVE
static ssize_t lcd_cabc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int retval = 0;

	mutex_lock(&g_zte_ctrl_pdata->zte_lcd_ctrl->panel_sys_lock);
	retval = snprintf(buf, 32, "%d\n", g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_value);
	ZTE_LCD_INFO("%s:cabc_value: 0x%x\n", __func__, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_value);
	mutex_unlock(&g_zte_ctrl_pdata->zte_lcd_ctrl->panel_sys_lock);
	return retval;
}
static int zte_set_panel_cabc(int cabc_mode)
{
	struct sprd_dsi *dsi = dev_get_drvdata(g_zte_ctrl_pdata->zte_lcd_ctrl->panel_info->pd->intf);

	ZTE_LCD_INFO("%s:cabc_mode= 0x%x\n", __func__, cabc_mode);
	switch (cabc_mode) {
	mipi_dsi_lp_cmd_enable(dsi, true);
	case LCD_CABC_OFF_MODE:
		mipi_dsi_send_cmds(dsi, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_off_cmds);
		break;
	case LCD_CABC_LOW_MODE:
		mipi_dsi_send_cmds(dsi, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_low_cmds);
		break;
	case LCD_CABC_MEDIUM_MODE:
		mipi_dsi_send_cmds(dsi, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_medium_cmds);
		break;
	case LCD_CABC_HIGH_MODE:
		mipi_dsi_send_cmds(dsi, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_high_cmds);
		break;
	default:
		mipi_dsi_send_cmds(dsi, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_off_cmds);
		break;
	}
	return 0;
}
static ssize_t lcd_cabc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input = 0;

	if (kstrtoint(buf, 16, &input) != 0)
		return -EINVAL;

	if (input > LCD_CABC_HIGH_MODE)
		return -EINVAL;

	mutex_lock(&g_zte_ctrl_pdata->zte_lcd_ctrl->panel_sys_lock);
	g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_value = input;
	zte_set_panel_cabc(input);
	mutex_unlock(&g_zte_ctrl_pdata->zte_lcd_ctrl->panel_sys_lock);
	return count;
}
static DEVICE_ATTR(lcd_cabc, 0664,  lcd_cabc_show, lcd_cabc_store);
static struct attribute *lcd_cabc_attributes[] = {
	&dev_attr_lcd_cabc.attr,
	NULL
};
static struct attribute_group lcd_cabc_attribute_group = {
	.attrs = lcd_cabc_attributes
};
int zte_create_cabc_sys(struct sprd_dispc *dispc)
{
	int err;
	struct kobject *lcd_cabc_kobj;

	lcd_cabc_kobj = kobject_create_and_add("cabc", g_zte_ctrl_pdata->zte_lcd_ctrl->kobj);
	if (!lcd_cabc_kobj) {
		err = -EINVAL;
		ZTE_LCD_ERROR("%s:ERROR Unable to create lcd_cabc_kobj.\n", __func__);
		return -EIO;
	}
	err = sysfs_create_group(lcd_cabc_kobj, &lcd_cabc_attribute_group);
	if (err != 0) {
		ZTE_LCD_ERROR("%s:ERROR lcd_cabc_kobj failed.\n", __func__);
		kobject_put(lcd_cabc_kobj);
		return -EIO;
	}
	ZTE_LCD_INFO("%s:succeeded.\n", __func__);
	return err;
}
#endif
/********************extreme power save mode cabc end*****************/


/********************lcd backlight level curve begin*****************/
#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE

const int zte_backlight_curve_matrix_max_350_lux[256] = {
0, 3, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9,
10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 16, 16, 17, 17, 18,
18, 19, 19, 20, 21, 21, 22, 22, 23, 23, 24, 25, 25, 26, 26, 27,
28, 28, 29, 29, 30, 31, 31, 32, 33, 33, 34, 35, 35, 36, 36, 37,
38, 38, 39, 40, 40, 41, 42, 43, 43, 44, 45, 45, 46, 47, 47, 48,
49, 50, 50, 51, 52, 52, 53, 54, 55, 55, 56, 57, 58, 58, 59, 60,
61, 61, 62, 63, 64, 64, 65, 66, 67, 68, 68, 69, 70, 71, 72, 73,
74, 75, 76, 77, 77, 78, 79, 80, 81, 82, 82, 83, 84, 85, 86, 87,
88, 88, 89, 90, 91, 92, 93, 94, 95, 96, 96, 97, 98, 99, 100, 101,
102, 103, 104, 105, 106, 107, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116,
117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132,
133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 146, 147,
148, 149, 151, 152, 153, 155, 156, 157, 159, 160, 162, 163, 164, 166, 167, 169,
170, 172, 173, 175, 176, 178, 179, 181, 183, 184, 186, 187, 189, 191, 192, 194,
196, 197, 199, 201, 203, 204, 206, 208, 210, 212, 214, 215, 217, 219, 221, 223,
225, 227, 229, 231, 233, 235, 237, 239, 241, 243, 245, 248, 250, 252, 254, 255
};

const int zte_backlight_curve_matrix_max_400_lux[256] = {
0, 3, 3, 3, 3, 3, 4, 4, 5, 5, 6, 6, 6, 7, 7, 8,
8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16,
16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24,
25, 25, 26, 26, 27, 27, 28, 28, 29, 30, 30, 31, 31, 32, 32, 33,
34, 34, 35, 35, 36, 37, 37, 38, 38, 39, 40, 40, 41, 42, 42, 43,
43, 44, 45, 45, 46, 47, 47, 48, 49, 49, 50, 51, 51, 52, 53, 53,
54, 55, 55, 56, 57, 57, 58, 59, 59, 60, 61, 61, 62, 63, 64, 64,
65, 66, 66, 67, 68, 69, 69, 70, 71, 72, 72, 73, 74, 74, 75, 76,
77, 78, 78, 79, 80, 81, 81, 82, 83, 84, 84, 85, 86, 87, 88, 88,
89, 90, 91, 92, 92, 93, 94, 95, 96, 96, 97, 98, 99, 100, 101, 101,
102, 103, 104, 105, 106, 107, 107, 108, 109, 110, 111, 112, 113, 113, 114, 115,
116, 117, 118, 119, 120, 121, 121, 122, 123, 124, 125, 126, 127, 128, 129, 129,
130, 132, 133, 134, 136, 137, 139, 140, 142, 143, 145, 147, 148, 150, 151, 153,
155, 156, 158, 160, 162, 163, 165, 167, 169, 170, 172, 174, 176, 178, 180, 182,
184, 186, 188, 190, 192, 194, 196, 198, 200, 203, 205, 207, 209, 212, 214, 216,
219, 221, 223, 226, 228, 231, 233, 236, 238, 241, 243, 246, 249, 252, 254, 255
};

const int zte_backlight_curve_matrix_max_450_lux[256] = {
0, 3, 3, 3, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7,
8, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14,
14, 15, 15, 16, 16, 17, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21,
22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29,
30, 30, 31, 31, 32, 32, 33, 33, 34, 34, 35, 36, 36, 37, 37, 38,
38, 39, 39, 40, 41, 41, 42, 42, 43, 43, 44, 45, 45, 46, 46, 47,
48, 48, 49, 49, 50, 51, 51, 52, 52, 53, 54, 54, 55, 56, 56, 57,
57, 58, 59, 59, 60, 61, 61, 62, 63, 63, 64, 65, 65, 66, 67, 67,
68, 69, 69, 70, 71, 71, 72, 73, 73, 74, 75, 75, 76, 77, 78, 78,
79, 80, 80, 81, 82, 83, 83, 84, 85, 85, 86, 87, 88, 88, 89, 90,
91, 91, 92, 93, 94, 94, 95, 96, 97, 97, 98, 99, 100, 101, 101, 102,
103, 104, 105, 105, 106, 107, 108, 109, 109, 110, 111, 112, 112, 112, 113, 113,
114, 116, 117, 119, 120, 122, 123, 125, 126, 128, 130, 131, 133, 135, 136, 138,
140, 142, 143, 145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169,
171, 173, 176, 178, 180, 183, 185, 187, 190, 192, 194, 197, 199, 202, 205, 207,
210, 213, 215, 218, 221, 224, 226, 229, 232, 235, 238, 241, 244, 248, 251, 255
};
static int zte_convert_backlevel_function(int level, u32 bl_max)
{
	int bl, convert_level;

	if (level == 0)
		return 0;

	if (bl_max > 1023) {
		bl = level>>4;
	} else {
		bl = level;
	}

	if (!bl && level)
		bl = 1;/*ensure greater than 0 and less than 16 equal to 1*/

	switch (g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode) {
	case CURVE_MATRIX_MAX_350_LUX:
		convert_level = zte_backlight_curve_matrix_max_350_lux[bl];
		break;
	case CURVE_MATRIX_MAX_400_LUX:
		convert_level = zte_backlight_curve_matrix_max_400_lux[bl];
		break;
	case CURVE_MATRIX_MAX_450_LUX:
		convert_level = zte_backlight_curve_matrix_max_450_lux[bl];
		break;
	default:
		convert_level = zte_backlight_curve_matrix_max_450_lux[bl];
		break;
	}
	if (bl_max > 1023) {
		convert_level = (convert_level >= 255) ? 4095 : (convert_level<<4);
	}

	return convert_level;
}
#endif
/********************lcd backlight level curve end*****************/

/********************lcd common function start*****************/
static void zte_lcd_panel_parse_dt(struct sprd_dispc *dispc, struct device_node *node)
{
	int rc;
	u32 tmp;
#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
	const char *data;
#endif
#ifdef CONFIG_ZTE_LCD_CABC3_EXTREME_POWER_SAVE
	struct panel_cmds *pcmds = NULL;
#endif

#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
	data = of_get_property(node, "zte,lcm_backlight_curve_mode", NULL);
	if (data) {
		if (!strcmp(data, "lcd_brightness_max_350_lux"))
			g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode = CURVE_MATRIX_MAX_350_LUX;
		else if (!strcmp(data, "lcd_brightness_max_400_lux"))
			g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode = CURVE_MATRIX_MAX_400_LUX;
		else if (!strcmp(data, "lcd_brightness_max_450_lux"))
			g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode = CURVE_MATRIX_MAX_450_LUX;
		else
			g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode = CURVE_MATRIX_MAX_450_LUX;
	} else
		g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode = CURVE_MATRIX_MAX_450_LUX;

	ZTE_LCD_INFO("%s:dtsi_mode=%s matrix_mode=%d\n", __func__, data,
			g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_bl_curve_mode);
#endif


#ifdef CONFIG_ZTE_LCD_CABC3_EXTREME_POWER_SAVE
	rc = of_property_read_u32(node, "zte,lcd-cabc-default-value", &tmp);
	g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_value = (!rc ? tmp : 0x01);
	ZTE_LCD_INFO("%s:cabc_value =%d\n", __func__, g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_value);

	rc = of_parse_cmds(node, &pcmds, "zte,lcd-cabc-off-command");
	if (!rc)
		g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_off_cmds = pcmds;
	else
		ZTE_LCD_ERROR("init-data cmd not found\n");

	rc = of_parse_cmds(node, &pcmds, "zte,lcd-cabc-low-command");
	if (!rc)
		g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_low_cmds = pcmds;
	else
		ZTE_LCD_ERROR("init-data cmd not found\n");

	rc = of_parse_cmds(node, &pcmds, "zte,lcd-cabc-medium-command");
	if (!rc)
		g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_medium_cmds = pcmds;
	else
		ZTE_LCD_ERROR("init-data cmd not found\n");

	rc = of_parse_cmds(node, &pcmds, "zte,lcd-cabc-high-command");
	if (!rc)
		g_zte_ctrl_pdata->zte_lcd_ctrl->cabc_high_cmds = pcmds;
	else
		ZTE_LCD_ERROR("init-data cmd not found\n");

#endif

#ifdef CONFIG_ZTE_LCD_DCSBL_CABC_GRADIENT
	g_zte_ctrl_pdata->zte_lcd_ctrl->close_dynamic_dimming = of_property_read_bool(node,
		"zte,lcd-close-dynamic-dimming");
	ZTE_LCD_INFO("close_dynamic_dimming is 0x%x\n", g_zte_ctrl_pdata->zte_lcd_ctrl->close_dynamic_dimming);
#endif

	rc = of_property_read_u32(node, "zte,lcd-backlight-register-bit-length", &tmp);
	g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_bl_register_len = (!rc ? tmp : 8);
	ZTE_LCD_INFO("tmp=%d lcd bl length=%d\n", tmp, g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_bl_register_len);

#ifdef CONFIG_ZTE_LCD_DISABLE_SSC
	g_zte_ctrl_pdata->zte_lcd_ctrl->is_disable_ssc = of_property_read_bool(node,
		"zte,lcd-disable-ssc");
	ZTE_LCD_INFO("is_disable_ssc is 0x%x\n", g_zte_ctrl_pdata->zte_lcd_ctrl->is_disable_ssc);
#endif
}



static ssize_t zte_show_esd_num(struct device *d, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 80, "esd num:0x%x\n", g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_esd_num);
}
static DEVICE_ATTR(esd_num, 0660, zte_show_esd_num, NULL);

static struct attribute *zte_lcd_fs_attrs[] = {
	&dev_attr_esd_num.attr,
	NULL,
};

static struct attribute_group zte_lcd_attrs_group = {
	.attrs = zte_lcd_fs_attrs,
};

static void zte_lcd_common_init(struct sprd_dispc *dispc, struct panel_info *panel_info)
{
	int ret = 0;

	g_zte_ctrl_pdata->zte_lcd_ctrl = kzalloc(sizeof(struct zte_lcd_ctrl_data), GFP_KERNEL);
	if (!g_zte_ctrl_pdata->zte_lcd_ctrl) {
		ZTE_LCD_ERROR("no mem to save zte_lcd_ctrl_data info: kzalloc fail\n");
		return;
	}

	ZTE_LCD_INFO("%s:alloc zte_lcd_ctrl_data success!\n", __func__);

	/*create /sys/lcd_sys/ path to add other lcd ctrl point*/
	g_zte_ctrl_pdata->zte_lcd_ctrl->kobj = kobject_create_and_add("lcd_sys", NULL);
	if (!g_zte_ctrl_pdata->zte_lcd_ctrl->kobj) {
		ZTE_LCD_ERROR("%s:create lcd_sys error!\n", __func__);
	} else {
		ret = sysfs_create_group(g_zte_ctrl_pdata->zte_lcd_ctrl->kobj, &zte_lcd_attrs_group);
		if (ret)
			ZTE_LCD_ERROR("sysfs group creation failed, rc=%d\n", ret);
	}

	
	g_zte_ctrl_pdata->zte_lcd_ctrl->panel_info = panel_info;

	g_zte_ctrl_pdata->zte_lcd_ctrl->lcd_powerdown_for_shutdown = false;

}
void zte_lcd_common_func(struct sprd_dispc *dispc, struct panel_info *panel_info)
{
	if (g_zte_ctrl_pdata) {
		return;
	}

	g_zte_ctrl_pdata = dispc;

	zte_lcd_common_init(dispc, panel_info);
	zte_lcd_panel_parse_dt(dispc, panel_info->of_node);

	zte_lcd_proc_info_display(panel_info);
#ifdef CONFIG_ZTE_LCD_REG_DEBUG
	zte_lcd_reg_debug_func();
#endif
#ifdef CONFIG_ZTE_LCD_BACKLIGHT_LEVEL_CURVE
	g_zte_ctrl_pdata->zte_lcd_ctrl->zte_convert_brightness = zte_convert_backlevel_function;
#endif

#ifdef CONFIG_ZTE_LCD_CABC3_EXTREME_POWER_SAVE
	mutex_init(&g_zte_ctrl_pdata->zte_lcd_ctrl->panel_sys_lock);
	zte_create_cabc_sys(g_zte_ctrl_pdata);
	g_zte_ctrl_pdata->zte_lcd_ctrl->zte_set_cabc_mode = zte_set_panel_cabc;
#endif

}
