# SPRD Sound Card Support

SPRD_SOUND_TREE				:= sound/soc/sprd/
SPRD_AUDIO_ASSISTANT_TREE		:= drivers/misc/sprd_audio/
ccflags-y				+= -I$(SPRD_SOUND_TREE)/codec/sprd/
subdir-ccflags-y			+= -I$(SPRD_SOUND_TREE)/include/ \
					   -I$(SPRD_AUDIO_ASSISTANT_TREE)/agdsp_access/

obj-$(CONFIG_SND_SOC_SPRD)		+= sprd-asoc-common.o sprd-asoc-card-utils.o \
					   sprd-asoc-card-utils-legacy.o \
					   sprd-asoc-card-utils-hook.o
obj-$(CONFIG_SND_SOC_SPRD)		+= platform/
obj-$(CONFIG_SND_SOC_SPRD)		+= codec/dummy-codec/

obj-$(CONFIG_SND_SOC_SPRD_VBC_R3P0)	+= dai/vbc/r3p0/
obj-$(CONFIG_SND_SOC_SPRD_VBC_R2P0)	+= dai/vbc/r2p0/
obj-$(CONFIG_SND_SOC_SPRD_VBC_R1P0V3)	+= dai/vbc/r1p0v3/

obj-$(CONFIG_SND_SOC_SPRD_VAUDIO)	+= dai/vaudio/
obj-$(CONFIG_SND_SOC_SPRD_I2S)		+= dai/i2s/
obj-$(CONFIG_SND_SOC_SPRD_CODEC_SC2723)	+= codec/sprd/sc2723/
obj-$(CONFIG_SND_SOC_SPRD_CODEC_SC2731)	+= codec/sprd/sc2731/
obj-$(CONFIG_SND_SOC_SPRD_CODEC_SC2721)	+= codec/sprd/sc2721/

# Machines
obj-$(CONFIG_SND_SOC_SPRD_R1P0V3_SC2721) 	+= vbc-rxpx-codec-sc27xx.o
obj-$(CONFIG_SND_SOC_SPRD_R1P0V3_SC2731) 	+= vbc-rxpx-codec-sc27xx.o
obj-$(CONFIG_SND_SOC_SPRD_R2P0_SC2723)		+= vbc-rxpx-codec-sc27xx.o
obj-$(CONFIG_SND_SOC_SPRD_R3P0_SC2731)		+= vbc-rxpx-codec-sc27xx.o
obj-$(CONFIG_SND_SOC_SPRD_R3P0_RT5659)		+= vbc-r3p0-codec-rt5659.o

snd-soc-i2s-dummy-objs         		:= i2s-r0p0-dummy-codec.o
obj-$(CONFIG_SND_SOC_SPRD_I2S_DUMMY)    += snd-soc-i2s-dummy.o
