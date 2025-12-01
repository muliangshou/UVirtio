include $(KERNEL_ROOT)/.config
ifeq ($(CONFIG_ADD_NUTTX_FETURES),y)
    include $(APPDIR)/Make.defs
    CSRCS += adapter.c adapter_agent.c
    include $(APPDIR)/Application.mk

endif

ifeq ($(CONFIG_ADD_XIZI_FETURES),y)
    SRC_FILES := adapter.c adapter_agent.c

    ifeq ($(CONFIG_CONNECTION_INDUSTRIAL_ETHERNET),y)
        SRC_DIR += industrial_ethernet
    endif

    ifeq ($(CONFIG_CONNECTION_INDUSTRIAL_FIELDBUS),y)
        SRC_DIR += industrial_fieldbus
    endif

    ifeq ($(CONFIG_CONNECTION_INDUSTRIAL_WLAN),y)
        SRC_DIR += industrial_wlan
    endif

    ifeq ($(CONFIG_CONNECTION_ADAPTER_LORA),y)
        SRC_DIR += lora
    endif

    ifeq ($(CONFIG_CONNECTION_ADAPTER_4G),y)
        SRC_DIR += 4g
    endif

    ifeq ($(CONFIG_CONNECTION_ADAPTER_NB),y)
        SRC_DIR += nbiot
    endif

    ifeq ($(CONFIG_CONNECTION_ADAPTER_WIFI),y)
        SRC_DIR += wifi
    endif

    ifeq ($(CONFIG_CONNECTION_ADAPTER_ETHERNET),y)
        SRC_DIR += ethernet
    endif

    ifeq ($(CONFIG_CONNECTION_ADAPTER_BLUETOOTH),y)
        SRC_DIR += bluetooth
    endif

    ifeq ($(CONFIG_CONNECTION_ADAPTER_ZIGBEE),y)
        SRC_DIR += zigbee
    endif

    ifeq ($(CONFIG_CONNECTION_ADAPTER_5G),y)
        SRC_DIR += 5g
    endif

    include $(KERNEL_ROOT)/compiler.mk

endif