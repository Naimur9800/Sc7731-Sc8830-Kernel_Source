ifneq ($(ZTE_BOARD_NAME),)
	dts-dirs += $(ZTE_BOARD_NAME)
else
dtb-$(CONFIG_ARCH_SPRD) += sc9836-openphone.dtb \
	sp9850s-1h10-native.dtb \
	sp9850s-2h10-native.dtb \
	sp9860g-1h10-native.dtb \
	sp9860g-1h10-al.dtb \
	sp9860g-2h10-native.dtb \
	sp9860g-1h10-3g.dtb \
	sp9860g-1h10-3gfhd.dtb \
	ss_whale2_j7-native.dtb \
	sp9850kh-smt-1h10.dtb \
	sp9850kh-smt-1h10-a53only.dtb \
	sp9850kh-smt-1h10-singlethread.dtb \
	sp9850kh-smt-1h10-isolatecpu.dtb \
	sp9850kh-smt-2c20.dtb \
	sp9850kh-smt-fpga.dtb \
	sp9850kh-smt-vdk.dtb \
	sp9850kh-smt-zebu.dtb \
	sp9862-vdk.dtb \
	sp9862-haps.dtb \
	sp9835_haps.dtb \
	sp9855a-vdk.dtb \
	sp9855a-haps.dtb \
	sp9863a-1h10-native.dtb\
	sp9863a-10c10-native.dtb\
	sp9863a-2h10-native.dtb\
	sp9863a-3h10-native.dtb\
	sp9863a-1h10-go-native.dtb\
	sp9863a-3h10-gofu-native.dtb\
	sp9832e-1h10-native.dtb\
	sp9832e-10c20-native.dtb\
	sp9850e-1h10-native.dtb\
	ud710-haps.dtb\
	ums510-haps.dtb

dtbo-$(CONFIG_ARCH_SPRD) += \
	sp9863a-1h10-overlay.dtbo \
	sp9863a-10c10-overlay.dtbo \
	sp9863a-2h10-overlay.dtbo \
	sp9863a-3h10-overlay.dtbo \
	sp9863a-1h10-go-overlay.dtbo \
	sp9863a-3h10-gofu-overlay.dtbo \
	sp9832e-1h10-overlay.dtbo \
	sp9832e-10c20-overlay.dtbo \
	sp9832e-2h10-overlay.dtbo
endif

always		:= $(dtb-y) $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
