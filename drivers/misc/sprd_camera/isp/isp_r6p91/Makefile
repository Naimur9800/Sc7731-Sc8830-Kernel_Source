obj-$(CONFIG_SPRD_CAMERA_ISP_R6P91) += sprd_isp.o

sprd_isp-objs := isp_drv.o \
		isp_int.o \
		isp_buf.o

CAM_SRC = $(wildcard $(srctree)/$(src)/src/*.c)
CAM_DIR = $(notdir $(CAM_SRC))
CAM_OBJ = $(patsubst %c, src/%o, $(CAM_DIR))

obj-y += sprd_isp.o $(CAM_OBJ)

ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common \
		-I$(srctree)/drivers/misc/sprd_camera/common/r4p0 \
		-I$(srctree)/drivers/misc/sprd_camera/isp/isp_r6p91/inc \
		-I$(srctree)/drivers/misc/sprd_camera/dcam/dcam_r6p0
