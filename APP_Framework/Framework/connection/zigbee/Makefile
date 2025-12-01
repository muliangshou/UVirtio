include $(KERNEL_ROOT)/.config
ifeq ($(CONFIG_ADD_NUTTX_FETURES),y)
    include $(APPDIR)/Make.defs
    CSRCS += adapter_zigbee.c
    include $(APPDIR)/Application.mk

endif

ifeq ($(CONFIG_ADD_XIZI_FETURES),y)
    SRC_FILES := adapter_zigbee.c

    ifeq ($(CONFIG_ADAPTER_E18),y)
        SRC_DIR += e18
    endif

    include $(KERNEL_ROOT)/compiler.mk

endif
