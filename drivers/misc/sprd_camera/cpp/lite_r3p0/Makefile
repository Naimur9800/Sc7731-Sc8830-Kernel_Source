obj-y := cpp_lite_r3p0.o

cpp_lite_r3p0-objs := cpp_core.o scale_drv.o rot_drv.o dma_drv.o

ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common
ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R1P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r1p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R2P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r2p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0), y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r3p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R5P0), y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r5p0
else ifeq ($(CONFIG_SPRD_CAMERA_DCAM_IF_R5P0_ISP_R6P12), y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/dcam_if_r5p0_isp_r6p12/inc
endif
