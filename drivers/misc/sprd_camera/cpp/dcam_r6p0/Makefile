obj-$(CONFIG_SPRD_CAMERA_DCAM_R6P0) += cpp_dcam_r6p0.o

cpp_dcam_r6p0-objs := img_rot.o img_scale.o scale_drv.o rot_drv.o

ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common
ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R1P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r1p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R2P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r2p0
endif

