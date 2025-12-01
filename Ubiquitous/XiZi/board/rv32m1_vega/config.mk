
export CFLAGS :=  -march=rv32imac -mabi=ilp32 -fno-builtin -fno-exceptions -ffunction-sections  -O0 -ggdb -Werror
export AFLAGS :=  -c -march=rv32imac -mabi=ilp32 -x assembler-with-cpp -ggdb
export LFLAGS :=  -march=rv32imac -mabi=ilp32 -nostartfiles -Wl,--gc-sections,-Map=XiZi_rv32m1_vega.map,-cref,-u,Reset_Handler -T $(BSP_ROOT)/link.lds

export CXXFLAGS :=  -march=rv32imac -mabi=ilp32 -fno-builtin -fno-exceptions -ffunction-sections  -O0 -ggdb -Werror

export CROSS_COMPILE ?=/opt/gnu-mcu-eclipse/riscv-none-gcc/8.2.0-2.1-20190425-1021/bin/riscv-none-embed-

export DEFINES := -DHAVE_CCONFIG_H -DHAVE_SIGINFO

export ARCH = risc-v
export MCU =  RV32M1_VEGA



