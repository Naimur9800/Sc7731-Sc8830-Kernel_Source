#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.
ccflags-y += -I$(srctree)/drivers/input/touchscreen/zte_touchscreen/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/zte_touchscreen/tlsc6x/firmware_config/$(CONFIG_TS_FIRMWARE)/
obj-$(CONFIG_TOUCHSCREEN_TLSC6X)  += tlsc6x_ts.o
tlsc6x_ts-objs := tlsc6x_main.o
tlsc6x_ts-objs += tlsc6x_comp.o
tlsc6x_ts-objs += tlsc6x_common_interface.o

