obj-$(CONFIG_SPRD_CAMERA_ISP_G2V1) += g2v1.o

g2v1-objs := isp_drv.o \
		isp_cfg_fw.o \
		isp_drv_buf.o \
		compat_isp_drv.o \
		isp_drv_capability.o \
		altek_isp/altek_common.o \
		altek_isp/altek_isp_drv.o \
		altek_isp/altek_platform_info.o \
		altek_isp/altek_ahb_drv.o \
		altek_isp/altek_ap3a_info.o \
		altek_isp/altek_bist.o \
		altek_isp/altek_dump_utility.o \
		altek_isp/altek_iq_info.o \
		altek_isp/altek_cmd.o \
		altek_isp/altek_log.o \
		altek_isp/altek_iq_info_func.o \
		altek_isp/altek_cmd_bist.o

ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common \
	     -I$(srctree)/drivers/misc/sprd_camera/sensor/ \
	     -I$(srctree)/drivers/misc/sprd_camera/dcam/dcam_r5p0

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R1P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r1p0
else ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R2P0),y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r2p0
endif
