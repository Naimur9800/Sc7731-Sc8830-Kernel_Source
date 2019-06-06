
obj-$(CONFIG_SPRD_VSP) += sprd_vsp.o
obj-$(CONFIG_SPRD_VSP) += vsp_common.o
obj-$(CONFIG_SPRD_VSP) += sprd_vsp_pw_domain.o
obj-$(CONFIG_SPRD_VSP_ENC) += sprd_vsp_enc.o


ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0), y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r3p0
endif

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R4P0), y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r4p0
endif