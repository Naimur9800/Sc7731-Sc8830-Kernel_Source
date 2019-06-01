
ifeq ($(CONFIG_MACH_VIVALTO),y)
obj-y += flash_vivalto.o
else
ifeq ($(CONFIG_MACH_KANAS_W),y)
obj-y += flash_kanas_w.o
else
ifeq ($(CONFIG_MACH_SP8730SEA),y)
obj-y += flash_sgm3140.o
else
ifeq ($(CONFIG_MACH_SP7731GEA_HD),y)
obj-y	+= flash_ktd267.o
else
obj-y += flash_common.o
endif
endif
endif
endif
