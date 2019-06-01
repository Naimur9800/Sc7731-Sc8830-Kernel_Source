ccflags-y += -Idrivers/media/sprd_scale/common -Idrivers/media/sprd_dcam/common
ifeq ($(CONFIG_ARCH_SCX15),y)
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
ccflags-y += -Idrivers/media/sprd_scale/sc8830
sprd_scale-objs := common/sin_cos.o common/gen_scale_coef.o common/img_scale.o sc8830/scale_drv.o
ifeq ($(CONFIG_64BIT),y)
sprd_scale-objs += common/compat_img_scale.o
endif
obj-y += sprd_scale.o
endif

