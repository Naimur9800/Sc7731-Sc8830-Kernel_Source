
sprd_dcam-objs := dcam_drv.o

obj-y += sprd_dcam.o
obj-y += dcam_core.o

ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/sensor
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/dcam/dcam_if_r3p0v2

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R1P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r1p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R2P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r2p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r3p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0V2),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r3p0v2
endif

ifeq ($(CONFIG_SPRD_CAMERA_ISP_R6P10V2),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/dcam/isp_r6p10v2
endif

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0V2), y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r3p0v2 \
		-I$(srctree)/drivers/misc/sprd_camera/sensor \
		-I$(srctree)/drivers/misc/sprd_camera/flash
endif
