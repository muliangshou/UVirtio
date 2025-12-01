SRC_DIR :=

ifeq ($(CONFIG_RESOURCES_LWIP),y)

ifeq ($(CONFIG_USING_CONTROL_PLC_OPCUA), y)
	SRC_DIR += opcua
endif

ifeq ($(CONFIG_USING_CONTROL_PLC_SOCKET), y)
	SRC_DIR += socket
endif

endif

SRC_FILES += interoperability.c

include $(KERNEL_ROOT)/compiler.mk
