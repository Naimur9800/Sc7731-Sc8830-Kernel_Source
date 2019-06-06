#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.
obj-$(CONFIG_TOUCHSCREEN_MSG2XXX) += msg2133_ts.o
msg2133_ts-objs := mstar_drv_sprd.o
msg2133_ts-objs += mstar_drv_common.o
msg2133_ts-objs += mstar_drv_utility_adaption.o
msg2133_ts-objs += mstar_drv_self_fw_control.o
msg2133_ts-objs += mstar_drv_self_mp_test.o
msg2133_ts-objs += mstar_drv_ic_fw_porting_layer.o
msg2133_ts-objs += mstar_drv_main.o
msg2133_ts-objs += mstar_drv_platform_interface.o
msg2133_ts-objs += mstar_drv_platform_porting_layer.o
