# SRC_DIR := lvgl

ifeq ($(CONFIG_APP_SELECT_NEWLIB),y)
	ifeq ($(CONFIG_SEPARATE_COMPILE),y)
		SRC_DIR += app_newlib
	endif	
endif

ifeq ($(CONFIG_LIB_LV),y)
	SRC_DIR += lvgl
endif

ifeq ($(CONFIG_LIB_USING_CJSON),y)
	SRC_DIR += cJSON
endif

include $(KERNEL_ROOT)/compiler.mk
