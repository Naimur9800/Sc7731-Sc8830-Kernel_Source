
isp_r6p10-objs := isp_drv.o \
		isp_slice.o \
		isp_path.o \
		isp_cfg_param.o \
		isp_int.o \
		isp_buf.o \
		isp_3dnr.o \
		isp_ynr.o \
		isp_slw.o

obj-$(CONFIG_COMPAT) = compat_isp_drv.o

CAM_SRC = $(wildcard $(srctree)/$(src)/src/*.c)
CAM_DIR = $(notdir $(CAM_SRC))
CAM_OBJ = $(patsubst %c, src/%o, $(CAM_DIR))

obj-y += isp_r6p10.o $(CAM_OBJ)

ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common \
		-I$(srctree)/drivers/staging/android/ion
ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R1P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r1p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R2P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r2p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r3p0
endif
