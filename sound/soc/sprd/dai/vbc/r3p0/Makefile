# SPRD VBC Support
SPRD_SOUND_TREE := sound/soc/sprd/
SPRD_AUDIO_ASSISTANT_TREE := drivers/misc/sprd_audio/
SPRD_MCDT_TREE := drivers/misc/mcdt/
ccflags-y += -I$(SPRD_AUDIO_ASSISTANT_TREE)/audiosipc \
	     -I$(SPRD_SOUND_TREE)/dai \
	     -I$(SPRD_SOUND_TREE)/platform \
	     -I$(SPRD_SOUND_TREE)/dai/vbc/ \
	     -I$(SPRD_MCDT_TREE) \
	     -I$(SPRD_SOUND_TREE)/codec/sprd/sc2731/

snd-soc-sprd-vbc-r3p0-objs       := vbc-dai.o vbc-utils.o vbc-phy-r3p0.o vbc-codec.o
obj-$(CONFIG_SND_SOC_SPRD_VBC_R3P0)    += snd-soc-sprd-vbc-r3p0.o
