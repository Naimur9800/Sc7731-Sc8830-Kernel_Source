obj-$(CONFIG_MARLIN2) := marlin.o

marlin-c-objs += marlin_core.o
marlin-c-objs += slp_mgr.o
marlin-c-objs += mdbg_main.o
marlin-c-objs += mdbg_ring.o
marlin-c-objs += mdbg_common.o
marlin-c-objs += wcn_op.o
marlin-c-objs += rf/rf.o
marlin-c-objs += wcn_parn_parser.o
marlin-c-objs += wcn_debug.o
marlin-c-objs += wcn_misc.o
marlin-c-objs += wcn_bus.o
marlin-c-objs += sdio_v2.o

marlin-objs += $(marlin-c-objs)

#INTEGRATED_MARLIN2_GE2
obj-$(CONFIG_INTEGRATED_MARLIN2_GE2) := integrated_marlin2_ge2.o

integrated_marlin2_ge2-c-objs += wcn_integrate_platform.o wcn_parn_parser.o \
				 gnss/gnss_common_ctl.o \
				 gnss/gnss_pmnotify_ctl.o

integrated_marlin2_ge2-c-objs += mdbg_main.o \
                                 mdbg_common.o \
                                 mdbg_ring.o \
                                 mdbg_sipc.o \
                                 wcn_debug.o \
                                 wcn_misc.o \
                                 gnss/gnss_dbg.o \
                                 gnss/gnss_dump.o \
                                 rf/rf.o

integrated_marlin2_ge2-objs += $(integrated_marlin2_ge2-c-objs)

