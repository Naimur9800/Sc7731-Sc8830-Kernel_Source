# Makefile for the Himax touchscreen drivers.

ccflags-y += -I$(srctree)/drivers/input/touchscreen/zte_touchscreen/hxchipset_oncell/firmware_config/$(CONFIG_TS_FIRMWARE)/
obj-$(CONFIG_TOUCHSCREEN_HIMAX_I2C)   	+= himax_platform.o himax_ic.o himax_common.o himax_debug.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_ITO_TEST)   	+= 	himax_ito_test.o
obj-y += himax_common_interface.o
