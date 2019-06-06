dtb-$(CONFIG_ARCH_SPRD) += sp7731e-1h20-native.dtb
dtbo-$(CONFIG_ARCH_SPRD) += sp7731e-1h20-overlay.dtbo


dtstree		:= $(srctree)/$(src)
dtb-$(CONFIG_OF_ALL_DTBS) := $(patsubst $(dtstree)/%.dts,%.dtb, $(wildcard $(dtstree)/*.dts))

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES)) $(dtbo-y)
else
DTB_LIST := $(dtb-y) $(dtbo-y)
endif

targets += dtbs dtbs_install
targets += $(DTB_LIST)

always		:= $(DTB_LIST)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
