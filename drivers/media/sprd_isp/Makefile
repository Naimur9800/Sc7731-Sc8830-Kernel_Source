# SC88XX Platform Support

obj-y += sprd_isp.o
ccflags-y += -Idrivers/media/sprd_dcam/common


ifeq ($(CONFIG_ARCH_SCX35),y)
DEFS = -DVERSION3=1
ccflags-y += $(DEFS)

ifeq ($(CONFIG_ARCH_SCX15),y)
sprd_isp-objs := isp_drv_null.o
else
sprd_isp-objs := isp_drv_kernel.o
ifeq ($(CONFIG_64BIT),y)
sprd_isp-objs += compat_isp_drv_kernel.o
endif
endif

else ifeq ($(CONFIG_ARCH_SC8825),y)
sprd_isp-objs := isp_drv_kernel.o
endif

ifeq ($(CONFIG_ARCH_SCX15),y)
DEFS = -DVERSION1=1
ccflags-y += $(DEFS)
else ifeq ($(CONFIG_ARCH_SC8825),y)
DEFS = -DVERSION2=1
ccflags-y += $(DEFS)
else ifeq ($(CONFIG_ARCH_SCX30G),y)
DEFS = -DVERSION3T=1
ccflags-y += $(DEFS)
else ifeq ($(CONFIG_ARCH_SCX35L),y)
DEFS = -DVERSION3L=1
ccflags-y += $(DEFS)
endif

