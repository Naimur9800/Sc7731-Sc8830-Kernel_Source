obj-$(CONFIG_SPRD_CAMERA_DCAM_R6P0) += dcam_r6p0.o

dcam_r6p0-objs := dcam_img.o dcam_drv.o

ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common \
		-I$(srctree)/drivers/misc/sprd_camera/sensor \
		-I$(srctree)/drivers/misc/sprd_camera/sensor/csi2/csi_sprd/r2p0 \
		-I$(srctree)/drivers/misc/sprd_camera/flash \
		-I$(srctree)/drivers/misc/sprd_camera/isp/isp_r6p91/inc
#		-I$(srctree)/drivers/misc/sprd_camera/isp/g2v1

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R1P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r1p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R2P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r2p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R4P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r4p0
endif
