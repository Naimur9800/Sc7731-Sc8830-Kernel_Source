
sprd_cpp-objs := cpp_core.o scale_drv.o rot_drv.o scaler_coef_gen.o scaler_coef_cal.o cpp_path2_drv.o cpp_path3_drv.o cpp_path0_path3_drv.o cpp_path2_path3_drv.o  #preview_drv.o playback_drv.o

obj-y += sprd_cpp.o
ifeq ($(CONFIG_ISP_CPP_COWORK_SUPPORT), y)
obj-y += sprd_k_cpp.o
endif
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/dcam/

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R1P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r1p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R2P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r2p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_LITE_R1P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/lite_r1p0
endif

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0V2), y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r3p0v2
endif
