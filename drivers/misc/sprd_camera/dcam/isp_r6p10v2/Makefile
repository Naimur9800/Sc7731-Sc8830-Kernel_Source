
isp_r6p10v2-objs := isp_drv.o \
		isp_slice.o \
		isp_path.o \
		isp_cfg_param.o \
		isp_int.o \
		isp_buf.o \
		isp_3dnr.o \
		isp_slw.o \
		compat_isp_drv.o

CAM_SRC = $(wildcard $(srctree)/$(src)/src/*.c)
CAM_DIR = $(notdir $(CAM_SRC))
CAM_OBJ = $(patsubst %c, src/%o, $(CAM_DIR))

obj-y += isp_r6p10v2.o $(CAM_OBJ)

ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common \
		-I$(srctree)/drivers/staging/android/ion
ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R1P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r1p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R2P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r2p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r3p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0V2),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r3p0v2
endif
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/cpp/lite_r1p0

ifeq ($(CONFIG_SPRD_CAMERA_DCAM_IF_R3P0V2),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/dcam/dcam_if_r3p0v2/
endif

