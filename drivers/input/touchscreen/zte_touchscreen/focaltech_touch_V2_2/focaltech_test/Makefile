#drivers/input/touchscreen/focaltech_touch/focaltech_test
#
#Makefile for test module
#

ccflags-y += -I$(srctree)/drivers/input/touchscreen/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/vendor/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/zte_touchscreen/focaltech_touch_V2_2/firmware_config/$(CONFIG_TS_FIRMWARE)/
obj-$(CONFIG_TOUCHSCREEN_FTS_V2_2) += focaltech_test.o
obj-$(CONFIG_TOUCHSCREEN_FTS_V2_2) += focaltech_test_ini.o
obj-$(CONFIG_TOUCHSCREEN_FTS_V2_2) += focaltech_test_detail_threshold.o
obj-$(CONFIG_TOUCHSCREEN_FTS_V2_2) += supported_ic/

