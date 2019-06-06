obj-$(CONFIG_SPRD_MEMDISK)	+= sprd_memdisk.o
obj-$(CONFIG_SPRD_WAKEUPEN_INTC)	+= sprd-wakeupen-intc.o
obj-$(CONFIG_SPRD_VIBRATOR)     += sprd_vibrator.o
ifdef CONFIG_SOC_PIKE2
	obj-$(CONFIG_ARM_SPRD_CPUIDLE)  += sprd_light_sleep_pike2.o
else
	obj-$(CONFIG_ARM_SPRD_CPUIDLE)  += sprd_light_sleep.o
endif
obj-$(CONFIG_SPRD_IOMMU)	+= iommu/
# obj-$(CONFIG_SDIODEV)		+= sdiodev/
obj-$(CONFIG_SPRD_DEBUG)	+= debug/
obj-y	+= performance/

ifdef CONFIG_SOC_SHARKLJ1
	obj-$(CONFIG_ARM64_SPRD_DEEPSLEEP)   += pm.o pm-sharklj1.o pm-debug-sharklj1.o
else
	obj-$(CONFIG_ARM64_SPRD_DEEPSLEEP)   += pm.o pm-arm64.o pm_debug_arm64.o
endif
obj-$(CONFIG_ARM32_SPRD_DEEPSLEEP)   += pm.o pm-arm.o pm_debug_arm.o standby-arm.o
obj-$(CONFIG_SPRD_DEEPSLEEP_WITH_PSCI)  += psci.o
obj-$(CONFIG_SPRD_DEEPSLEEP_SHARKLE)  += pm-sharkle.o
obj-$(CONFIG_SPRD_DEEPSLEEP_PIKE2)  += pm-pike2.o
obj-$(CONFIG_SPRD_DEEPSLEEP_SHARKL3)  += pm-sharkl3.o
obj-$(CONFIG_SPRD_ARM32_SMP_WITH_PSCI)  += sprd_psci_smp.o
obj-$(CONFIG_SPRD_SYSDUMP)      += debug/sysdump/
obj-$(CONFIG_SPRD_DMC_DRV)      += debug/dmc_drv/
ifdef CONFIG_ARM_CPUIDLE
	obj-$(CONFIG_SPRD_ARM32_SMP_WITH_PSCI) += sprd_aarch32_cpuidle.o
endif
