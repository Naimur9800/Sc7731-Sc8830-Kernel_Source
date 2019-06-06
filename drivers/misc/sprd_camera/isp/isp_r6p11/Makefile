isp_r6p11-objs := isp_drv.o \
		isp_int.o \
		isp_buf.o \
		isp_statis_buf.o \
		isp_cfg_param.o \
		isp_slw.o \
		isp_slice.o \
		isp_path.o \
		isp_cfg.o \
		isp_reg_default.o

CAM_SRC = $(wildcard $(srctree)/$(src)/src/*.c)
CAM_DIR = $(notdir $(CAM_SRC))
CAM_OBJ = $(patsubst %c, src/%o, $(CAM_DIR))

obj-$(CONFIG_COMPAT) += compat_isp_drv.o
obj-$(CONFIG_SPRD_CAMERA_ISP_R6P11) += isp_r6p11.o $(CAM_OBJ)

ccflags-y += -I$(srctree)/drivers/staging/android/ion

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
