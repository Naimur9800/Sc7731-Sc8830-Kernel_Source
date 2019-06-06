ifeq ($(CONFIG_OF),y)
dtb-$(CONFIG_ARCH_SPRD) += sp9861-fpga.dtb \
	sp9861-vdk.dtb \
	sp9861-vdk-vmm.dtb \
	sp9861-vdk-4g-single.dtb \
	sp9861-vdk-4g-noninterleave.dtb \
	sp9861e-1h10.dtb \
	sp9861e-1h10-vmm.dtb \
	sp9861e-3h10.dtb \
	sp9861e-3h10-vmm.dtb \
	sp9861e-2h10.dtb \
	sp9861e-2h10-2705.dtb \
	sp9861e-1h11.dtb \
	sp9861e-1h11-vmm.dtb \
	sp9861e-2h10-vmm.dtb \
	sp9861e-2h10-2705-vmm.dtb \
	sp9861g-1h10-emmc.dtb \
	sp9853i-vdk.dtb \
	sp9853i-vdk-4g.dtb \
	sp9853i-fpga.dtb \
	sp9853i-vdk-vmm.dtb \
	sp9853i-fpga-vmm.dtb \
	sp9853i-1h10.dtb \
	sp9853i-9c10.dtb \
	sp9853i-10c10.dtb \
	sp9853i-2h10.dtb \
	sp9853i-3c10.dtb \
	sp9853i-1h10-vmm.dtb \
	sp9853i-9c10-vmm.dtb \
	sp9853i-10c10-vmm.dtb \
	sp9853i-2h10-vmm.dtb \
	sp9853i-3c10-vmm.dtb

targets += dtbs
targets += $(dtb-y)
endif

# *.dtb used to be generated in the directory above. Clean out the
# old build results so people don't accidentally use them.
dtbs: $(addprefix $(obj)/, $(dtb-y))
	$(Q)rm -f $(obj)/../*.dtb

clean-files := *.dtb