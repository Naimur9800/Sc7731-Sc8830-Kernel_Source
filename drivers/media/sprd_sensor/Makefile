# SC88XX Platform Support

obj-y += sprd_sensor.o
ccflags-y += -Idrivers/media/sprd_dcam/common

ifeq ($(CONFIG_ARCH_SCX15),y)
DEFS = -DVERSION1=1
ccflags-y += $(DEFS)
ccflags-y += -Idrivers/media/sprd_dcam/scx15
else ifeq ($(CONFIG_ARCH_SCX30G),y)
DEFS = -DVERSION3T=1
ccflags-y += $(DEFS)
ccflags-y += -Idrivers/media/sprd_dcam/sc8830g
else ifeq ($(CONFIG_ARCH_SCX35L),y)
DEFS = -DVERSION3L=1
ccflags-y += $(DEFS)
ccflags-y += -Idrivers/media/sprd_dcam/sc9630
else
ccflags-y += -Idrivers/media/sprd_dcam/sc8830
endif

ifeq ($(CONFIG_ARCH_SCX35),y)
DEFS = -DVERSION3=1
ccflags-y += $(DEFS)
else ifeq ($(CONFIG_ARCH_SC8825),y)
DEFS = -DVERSION2=1
ccflags-y += $(DEFS)
else ifeq ($(CONFIG_ARCH_SC8810),y)
DEFS = -DVERSION2T=1
ccflags-y += $(DEFS)
endif

common-file := csi2/csi_access.o csi2/csi_driver.o csi2/csi_log.o csi2/csi_system.o
ifeq ($(CONFIG_ARCH_SCX35),y)
sprd_sensor-objs := csi2/csi_api.o
endif
ifeq ($(CONFIG_ARCH_SCX15),y)
sprd_sensor-objs := csi2/csi_api_null.o
endif
ifeq ($(CONFIG_64BIT),y)
sprd_sensor-objs += compat_sensor_drv_k.o $(common-file)
endif
sprd_sensor-objs += sensor_drv_k.o $(common-file)
obj-y += power/
