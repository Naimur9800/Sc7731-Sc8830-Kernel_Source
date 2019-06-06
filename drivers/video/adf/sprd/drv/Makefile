subdir-ccflags-y += -I$(src)
ccflags-y := -Idrivers/staging/android

obj-y += dispc/
obj-y += encoder/
obj-y += dsi/
obj-y += dphy/
obj-y += panel-if/
obj-y += sysfs/

obj-y += whale2/
obj-y += iwhale2/
obj-y += sharkl2/
obj-y += isharkl2/
obj-y += sharklj1/
obj-y += sharkle/
obj-y += sharkl3/
obj-y += pike2/
obj-y += sharkl5/

obj-y += sprd_panel.o
obj-y += sprd_spi_intf.o
obj-y += sprd_dphy.o
obj-y += sprd_dsi.o
obj-y += sprd_dispc.o
obj-y += sprd_adf.o sprd_adf_bridge.o
obj-y += disp_lib.o
obj-y += disp_notify.o
obj-y += sprd_round_corner.o
ifeq ($(CONFIG_ZTE_LCD_COMMON_FUNCTION),y)
obj-y += zte_lcd_common.o
endif
ifeq ($(CONFIG_ZTE_LCD_REG_DEBUG),y)
obj-y += zte_lcd_reg_debug.o
endif
