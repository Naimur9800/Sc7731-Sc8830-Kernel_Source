
obj-$(CONFIG_SPRD_CAMERA_DCAM_IF_R2P0) += dcam_if_r2p0.o

dcam_if_r2p0-objs := dcam_img.o dcam_drv.o dcam2isp.o

ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common \
		-I$(srctree)/drivers/misc/sprd_camera/sensor \
		-I$(srctree)/drivers/misc/sprd_camera/sensor/csi2/csi_intel/11a \
		-I$(srctree)/drivers/misc/sprd_camera/flash \
		-I$(srctree)/drivers/misc/sprd_camera/isp/g2v3 \
		-I$(srctree)/drivers/misc/sprd_camera/cpp/r2p0

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R1P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r1p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R2P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r2p0
endif
