ifeq ($(strip $(TARGET_BOARD_DDR_1G)),true)
kernel_prepear_CMD := kernel/prepear.sh $(TARGET_PRODUCT) $(KERNEL_DEFCONFIG) $(TARGET_BOARD_DDR_1G)
else
kernel_prepear_CMD := kernel/prepear.sh $(TARGET_PRODUCT) $(KERNEL_DEFCONFIG) false
endif
KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL
KERNEL_CONFIG := $(KERNEL_OUT)/.config
KERNEL_MODULES_OUT := $(TARGET_OUT)/lib/modules

ifeq ($(strip $(KERNEL_UBOOT_USE_ARCH_ARM64)),true)
TRUE_ARCH := arm64
TRUE_CROSS_COMPILE := aarch64-linux-android-
else
TRUE_ARCH := arm
TRUE_CROSS_COMPILE := arm-eabi-
endif

ifeq ($(USES_UNCOMPRESSED_KERNEL),true)
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/$(TRUE_ARCH)/boot/Image
else
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/$(TRUE_ARCH)/boot/zImage
endif

.PHONY : build-kernel
build-kernel:
	@echo "====build-kernel stub===="


$(KERNEL_OUT): FORCE
	@echo "==== Copy Kernel files "$(TARGET_PRODUCT)" "$(KERNEL_DEFCONFIG)" "$(TARGET_BOARD_DDR_1G)"... ===="
	$(shell $(kernel_prepear_CMD))
	@echo "==== Start Kernel Compiling ... ===="


$(KERNEL_CONFIG): kernel/arch/$(TRUE_ARCH)/configs/$(KERNEL_DEFCONFIG)
	echo "KERNEL_OUT = $KERNEL_OUT,  KERNEL_DEFCONFIG = KERNEL_DEFCONFIG"
	mkdir -p $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(TRUE_ARCH) CROSS_COMPILE=$(TRUE_CROSS_COMPILE) $(KERNEL_DEFCONFIG)

ifeq ($(TARGET_BUILD_VARIANT),user)
USER_CONFIG := $(TARGET_OUT)/dummy
TARGET_DEVICE_USER_CONFIG := $(PLATDIR)/user_diff_config
TARGET_DEVICE_CUSTOM_CONFIG := device/sprd/$(TARGET_DEVICE)/ProjectConfig.mk

$(USER_CONFIG) : $(KERNEL_CONFIG)
	$(info $(shell ./kernel/scripts/sprd_custom_config_kernel.sh $(KERNEL_CONFIG) $(TARGET_DEVICE_CUSTOM_CONFIG)))
	$(info $(shell ./kernel/scripts/sprd_create_user_config.sh $(KERNEL_CONFIG) $(TARGET_DEVICE_USER_CONFIG)))
else
USER_CONFIG  := $(TARGET_OUT)/dummy
TARGET_DEVICE_CUSTOM_CONFIG := device/sprd/$(TARGET_DEVICE)/ProjectConfig.mk
$(USER_CONFIG) : $(KERNEL_CONFIG)
	$(info $(shell ./kernel/scripts/sprd_custom_config_kernel.sh $(KERNEL_CONFIG) $(TARGET_DEVICE_CUSTOM_CONFIG)))
endif

$(TARGET_PREBUILT_KERNEL) : $(KERNEL_OUT) $(USER_CONFIG)|$(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(TRUE_ARCH) CROSS_COMPILE=$(TRUE_CROSS_COMPILE) headers_install
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(TRUE_ARCH) CROSS_COMPILE=$(TRUE_CROSS_COMPILE) -j4
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(TRUE_ARCH) CROSS_COMPILE=$(TRUE_CROSS_COMPILE) modules
	@-mkdir -p $(KERNEL_MODULES_OUT)
	@-find $(KERNEL_OUT) -name *.ko | xargs -I{} cp {} $(KERNEL_MODULES_OUT)

kernelheader:
	mkdir -p $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(TRUE_ARCH) CROSS_COMPILE=$(TRUE_CROSS_COMPILE) headers_install
