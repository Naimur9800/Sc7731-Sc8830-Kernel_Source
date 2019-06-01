# SC88XX Platform Support

ccflags-y += -Idrivers/media/sprd_dcam/common
ifeq ($(CONFIG_ARCH_SCX15),y)
ccflags-y += -Idrivers/media/sprd_dcam/scx15
else ifeq ($(CONFIG_ARCH_SCX30G),y)
ccflags-y += -Idrivers/media/sprd_dcam/sc8830g
else ifeq ($(CONFIG_ARCH_SCX35L),y)
ccflags-y += -Idrivers/media/sprd_dcam/sc9630
else
ccflags-y += -Idrivers/media/sprd_dcam/sc8830
endif

ifeq ($(CONFIG_ARCH_SCX35L),y)
DEFS = -DVERSION3L=1
ccflags-y += $(DEFS)
endif
ifeq ($(CONFIG_ARCH_SCX30G),y)
DEFS = -DVERSION3T=1
ccflags-y += $(DEFS)
endif



ifeq ($(CONFIG_ARCH_SCX35),y)
sprd_rotation-objs := sc8830/rot_drv.o sc8830/img_rot.o
obj-y += sprd_rotation.o
endif

