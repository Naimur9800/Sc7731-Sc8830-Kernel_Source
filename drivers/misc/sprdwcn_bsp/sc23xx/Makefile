subdir-ccflags-y := \
	-Idrivers/misc/sprdwcn_bsp/sc23xx/include \
	-Idrivers/misc/sprdwcn_bsp/sc23xx/boot \
	-Idrivers/misc/sprdwcn_bsp/sc23xx/platform \
	-Idrivers/misc/sprdwcn_bsp/sc23xx/platform/rf

ifdef CONFIG_SC23XX
#
# Support sprdwcn_bsp build-in.
#
obj-$(CONFIG_SC23XX) += wcn_bus.o
obj-$(CONFIG_SDIOHAL) += sdio/
obj-$(CONFIG_WCN_SIPC) += sipc/
obj-$(CONFIG_WCN_SLP) += sleep/
obj-$(CONFIG_WCN_BOOT) += platform/
obj-$(CONFIG_WCN_PCIE) += pcie/
obj-$(CONFIG_SC2342_INTEG) += boot/
obj-$(CONFIG_MARLIN3_GNSS) += gnss/
obj-$(CONFIG_WCN_PLATFORM) += platform/

else
#
# Support sprdwcn_bsp ko module loading.
# This else branch is reserved for debuging.
#
obj-m := wcnbsp.o
wcnbsp-$(CONFIG_SC23XX) += wcn_bus.o
wcnbsp-$(CONFIG_SDIOHAL) += sdio/sdiohal_main.o \
			    sdio/sdiohal_common.o \
			    sdio/sdiohal_tx.o \
			    sdio/sdiohal_rx.o \
			    sdio/sdio_v3.o
ifdef CONFIG_SDIOHAL
wcnbsp-$(CONFIG_DEBUG_FS) += sdio/sdiohal_ctl.o
endif
wcnbsp-$(CONFIG_WCN_SIPC) += sipc/wcn_sipc.o
wcnbsp-$(CONFIG_WCN_SLP) += sleep/sdio_int.o \
			    sleep/slp_mgr.o \
			    sleep/slp_sdio.o \
			    sleep/slp_test.o
wcnbsp-$(CONFIG_WCN_BOOT) += platform/wcn_boot.o \
			     platform/wcn_dump.o \
			     platform/bufring.o \
			     platform/wcn_txrx.o \
			     platform/wcn_op.o \
			     platform/rf/rf.o \
			     platform/wcn_parn_parser.o \
			     platform/rdc_debug.o \
			     platform/wcn_log.o \
			     platform/wcn_misc.o \
			     platform/wcn_procfs.o \
			     platform/mem_pd_mgr.o
wcnbsp-$(CONFIG_SC2342_INTEG) += boot/wcn_integrate_dev.o \
				 boot/wcn_integrate_boot.o \
				 boot/wcn_integrate.o
wcnbsp-$(CONFIG_WCN_PLATFORM) += platform/wcn_parn_parser.o \
				 platform/wcn_procfs.o \
				 platform/wcn_txrx.o \
				 platform/rdc_debug.o \
				 platform/wcn_log.o \
				 platform/wcn_misc.o \
				 platform/rf/rf.o \
				 platform/bufring.o
ifdef CONFIG_WCN_PLATFORM
wcnbsp-$(CONFIG_SC2342_INTEG) += platform/wcn_dump_integrate.o
endif

endif
