
obj-$(CONFIG_SPRD_JPG) += sprd_jpg.o
obj-$(CONFIG_SPRD_JPG) += sprd_jpg_common.o
obj-$(CONFIG_SPRD_JPG1) += sprd_jpg1.o

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0), y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r3p0
endif

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R3P0V2), y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r3p0v2
endif

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R4P0), y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r4p0
endif

ifeq ($(CONFIG_SPRD_CAM_PW_DOMAIN_R5P0), y)
ccflags-y += -I$(srctree)/drivers/misc/sprd_camera/common/r5p0
endif
