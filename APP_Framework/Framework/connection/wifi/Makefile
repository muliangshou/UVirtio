SRC_FILES := adapter_wifi.c

ifeq ($(CONFIG_ADAPTER_HFA21_WIFI),y)
	SRC_DIR += hfa21_wifi
endif

ifeq ($(CONFIG_ADAPTER_ESP07S_WIFI),y)
	SRC_DIR += esp07s_wifi
endif

include $(KERNEL_ROOT)/compiler.mk
