SRC_DIR :=  transform_layer 

ifeq ($(CONFIG_SUPPORT_SENSOR_FRAMEWORK),y)
	SRC_DIR += sensor
endif

ifeq ($(CONFIG_SUPPORT_CONNECTION_FRAMEWORK),y)
	SRC_DIR += connection
endif

ifeq ($(CONFIG_SUPPORT_KNOWING_FRAMEWORK),y)
	SRC_DIR += knowing
endif

ifeq ($(CONFIG_SUPPORT_CONTROL_FRAMEWORK),y)
	SRC_DIR += control
endif

ifeq ($(CONFIG_CRYPTO),y)
	SRC_DIR += security
endif

include $(KERNEL_ROOT)/compiler.mk

