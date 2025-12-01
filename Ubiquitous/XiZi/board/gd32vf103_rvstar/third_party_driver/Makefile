SRC_FILES := system_gd32vf103.c
SRC_DIR := 

ifeq ($(CONFIG_BSP_USING_GPIO),y)
  SRC_DIR += gpio
endif

ifeq ($(CONFIG_BSP_USING_SYSCLOCK),y)
  SRC_DIR += sys_clock
endif

ifeq ($(CONFIG_BSP_USING_UART),y)
  SRC_DIR += uart
endif


include $(KERNEL_ROOT)/compiler.mk
