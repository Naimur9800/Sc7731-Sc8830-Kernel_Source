# Makefile for Discretix CC63P Linux device driver

MODULE_NAME = dx_cc63p

DX_ROM_VERSION:=0x03000009U
DX_CC_SRAM_SIZE:=4096

obj-$(CONFIG_CRYPTO_DEV_DX_CC63P) := $(MODULE_NAME).o
$(MODULE_NAME)-objs := dx_driver.o dx_sysfs.o dx_buffer_mgr.o dx_request_mgr.o dx_cipher.o dx_hash.o dx_aead.o dx_ivgen.o dx_sram_mgr.o dx_pm.o dx_pm_ext.o

EXTRA_CFLAGS += -DDX_CC_REE #-DCOMP_IN_WQ
EXTRA_CFLAGS += -DDX_ROM_VERSION=$(DX_ROM_VERSION)
ifneq ($(DX_CC_SRAM_SIZE),)
EXTRA_CFLAGS += -DDX_CC_SRAM_SIZE=$(DX_CC_SRAM_SIZE)
endif

