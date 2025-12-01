SRC_DIR :=  shared

ifeq ($(CONFIG_BOARD_FE310_EVB),y)
SRC_DIR +=fe310
endif

ifeq ($(CONFIG_BOARD_K210_EVB),y)
SRC_DIR +=k210
endif

ifeq ($(CONFIG_BOARD_MAIX_GO_EVB),y)
SRC_DIR +=k210
endif

ifeq ($(CONFIG_BOARD_AIIT_RISCV_EVB),y)
SRC_DIR +=k210
endif

ifeq ($(CONFIG_BOARD_GAPUINO),y)
SRC_DIR +=gap8
endif

ifeq ($(CONFIG_BOARD_GD32VF103RVSTAR),y)
SRC_DIR +=gd32vf103_rvstar
endif

ifeq ($(CONFIG_BOARD_RV32M1_VEGA),y)
SRC_DIR +=rv32m1_vega
endif

include $(KERNEL_ROOT)/compiler.mk
