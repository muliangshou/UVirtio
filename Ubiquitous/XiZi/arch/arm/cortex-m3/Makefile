ifeq ($(CONFIG_BOARD_CORTEX_M3_EVB),y)
SRC_FILES := boot.c interrupt_vector_evb.S
endif

ifeq ($(CONFIG_BOARD_STM32F103_NANO),y)
SRC_FILES := boot.S	interrupt_vector_nano.S
endif

SRC_FILES += interrupt.c

include $(KERNEL_ROOT)/compiler.mk
