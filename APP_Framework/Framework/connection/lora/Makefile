include $(KERNEL_ROOT)/.config
ifeq ($(CONFIG_ADD_NUTTX_FETURES),y)
    include $(APPDIR)/Make.defs
    CSRCS += adapter_lora.c
    include $(APPDIR)/Application.mk

endif

ifeq ($(CONFIG_ADD_XIZI_FETURES),y)
    SRC_FILES := adapter_lora.c

    ifeq ($(CONFIG_ADAPTER_SX1278),y)
        SRC_DIR += sx1278
    endif

    ifeq ($(CONFIG_ADAPTER_E220),y)
        SRC_DIR += e220
    endif
    include $(KERNEL_ROOT)/compiler.mk

endif
