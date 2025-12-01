# The following three platforms support compatiable instructions.

ifeq ($(CONFIG_BOARD_CORTEX_M3_EVB),y)
SRC_DIR := shared
SRC_DIR += cortex-m3
endif

ifeq ($(CONFIG_BOARD_STM32F103_NANO),y)
SRC_DIR := shared
SRC_DIR += cortex-m3
endif

ifeq ($(CONFIG_BOARD_STM32F407_EVB),y)
SRC_DIR := shared
SRC_DIR += cortex-m4
endif

ifeq ($(CONFIG_BOARD_CORTEX_M4_EVB),y)
SRC_DIR := shared
SRC_DIR += cortex-m4
endif

ifeq ($(CONFIG_BOARD_CORTEX_M7_EVB),y)
SRC_DIR := shared
SRC_DIR += cortex-m7
endif

# cortex-m0 is ARMv6-m

ifeq ($(CONFIG_BOARD_CORTEX_M0_EVB),y)
SRC_DIR += cortex-m0
endif

ifeq ($(CONFIG_BOARD_NUVOTON_M2354),y)
SRC_DIR += cortex-m23
endif

include $(KERNEL_ROOT)/compiler.mk
