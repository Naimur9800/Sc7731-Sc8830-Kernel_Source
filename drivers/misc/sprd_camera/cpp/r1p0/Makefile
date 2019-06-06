obj-$(CONFIG_SPRD_CAMERA_CPP_R1P0) += cpp_r1p0.o

cpp_r1p0-objs := cpp_core.o scale_drv.o rot_drv.o scaler_coef_gen.o scaler_coef_cal.o

ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common
ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R1P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r1p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R2P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r2p0
endif
