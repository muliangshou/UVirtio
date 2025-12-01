SRC_FILES := system_MIMXRT1052.c fsl_cache.c fsl_clock.c fsl_common.c pin_mux.c clock_config.c

ifeq ($(CONFIG_BSP_USING_SDIO),y)
  SRC_FILES += fsl_usdhc.c
endif

include $(KERNEL_ROOT)/compiler.mk
