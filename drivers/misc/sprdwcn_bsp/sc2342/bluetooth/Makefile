#
# makefile for SPREADTRUM BT devices
#
obj-$(CONFIG_MARLIN2_BT_SDIO) := mtty.o
mtty-c-objs += marlin_rfkill.o
mtty-c-objs += marlin_tty.o
mtty-c-objs += marlin2_sitm.o
mtty-c-objs += marlin_bt_lpm.o
mtty-objs += $(mtty-c-objs)
