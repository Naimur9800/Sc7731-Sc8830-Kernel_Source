dcam-objs := src/dcam_core.o \
		src/dcam_drv.o \
		src/dcam_int.o \
		src/dcam_path.o

isp-objs := src/isp_core.o \
		src/isp_drv.o \
		src/isp_int.o \
		src/isp_fmcu.o \
		src/isp_cfg.o \
		src/isp_path.o \
		src/isp_slice.o

#obj-$(CONFIG_COMPAT) = src/compat_isp_drv.o

CAM_SRC = $(wildcard $(srctree)/$(src)/block/*.c)
CAM_DIR = $(notdir $(CAM_SRC))
CAM_OBJ = $(patsubst %c, block/%o, $(CAM_DIR))

obj-$(CONFIG_SPRD_CAMERA_CAM_SL5_ROC1) += isp.o $(CAM_OBJ)
obj-$(CONFIG_SPRD_CAMERA_CAM_SL5_ROC1) += dcam.o
obj-$(CONFIG_SPRD_CAMERA_CAM_SL5_ROC1) += src/cam_core.o \
		src/cam_buf.o \
		src/cam_flash.o \
		src/cam_power.o \
		src/cam_queue.o

obj-$(CONFIG_SPRD_CAMERA_CSI_RECEIVER_R2P1) += src/cam_power.o

ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/cam_sl5_roc1 \
		-I$(srctree)/drivers/misc/sprd_camera/cam_sl5_roc1/inc \
		-I$(srctree)/drivers/misc/sprd_camera/common \
		-I$(srctree)/drivers/misc/sprd_camera/sensor \
		-I$(srctree)/drivers/misc/sprd_camera/flash \
		-I$(srctree)/drivers/staging/android/ion
#temp for HAPS
ccflags-y += -DTEST_ON_HAPS
