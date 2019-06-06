sprd_dcam-objs := dcam_buf.o \
		dcam_drv.o \
		dcam_core.o

CAM_SRC = $(wildcard $(srctree)/$(src)/src/*.c)
CAM_DIR = $(notdir $(CAM_SRC))
CAM_OBJ = $(patsubst %c, src/%o, $(CAM_DIR))

obj-$(CONFIG_SPRD_CAMERA_DCAM_IF_R4P0) += sprd_dcam.o $(CAM_OBJ)

ccflags-$(CONFIG_SPRD_CAMERA) += \
		-I$(srctree)/drivers/misc/sprd_camera/common \
		-I$(srctree)/drivers/misc/sprd_camera/sensor \
		-I$(srctree)/drivers/misc/sprd_camera/flash

ccflags-$(CONFIG_SPRD_CAMERA_ISP_R6P11) += \
		-I$(srctree)/drivers/misc/sprd_camera/isp/isp_r6p11

ccflags-$(CONFIG_SPRD_CAMERA_DCAM_IF_R4P0) += \
		-I$(srctree)/drivers/misc/sprd_camera/dcam/dcam_if_r4p0

ccflags-$(CONFIG_SPRD_CAM_PW_DOMAIN_R5P0) += \
		-I$(srctree)/drivers/misc/sprd_camera/common/r5p0
