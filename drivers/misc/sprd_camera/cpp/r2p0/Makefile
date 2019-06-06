
sprd_cpp-objs := cpp_core.o scale_drv.o rot_drv.o scaler_coef_gen.o scaler_coef_cal.o cpp_path2_drv.o cpp_path3_drv.o cpp_path0_path3_drv.o cpp_path2_path3_drv.o #preview_drv.o playback_drv.o

obj-$(CONFIG_SPRD_CAMERA_CPP_R2P0) += sprd_cpp.o

ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R1P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r1p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R2P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r2p0
endif
