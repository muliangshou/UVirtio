# ARCHCFLAGS = -fno-builtin
# ARCHCXXFLAGS = -fno-builtin -fno-exceptions -fcheck-new -fno-rtti
# ARCHPICFLAGS = -fpic -msingle-pic-base -mpic-register=r10

# ARCHOPTIMIZATION = -g
# ASARCHCPUFLAGS += -Wa,-g

#  ARCHCPUFLAGS = -march=rv32imcxgap8 -mPE=8 -mFC=1 -D__riscv__ -D__pulp__ -D__GAP8__ -fno-common -ffunction-sections -fdata-sections -fstrict-volatile-bitfields

# export CFLAGS := $(ARCHCFLAGS)  $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) -O0 -g3 -gdwarf-2

# export AFLAGS :=  $(CFLAGS) -D__ASSEMBLY__ $(ASARCHCPUFLAGS)

# export LFLAGS := -march=rv32imcxgap8 -mPE=8 -mFC=1 -nostartfiles -Wl,--gc-sections,-Map=XiUOS_gap8.map,-cref,-u,reset_handler -T $(BSP_ROOT)/link.lds

# export APPLFLAGS :=  -march=rv32imcxgap8 -mPE=8 -mFC=1  -nostartfiles -Wl,--gc-sections,-Map=XiUOS_app.map,-cref,-u, -T $(BSP_ROOT)/link_userspace.lds

# export CXXFLAGS :=  $(ARCHCXXFLAGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) 

# export CROSS_COMPILE ?=/opt/gap_riscv_toolchain/bin/riscv32-unknown-elf-


export CFLAGS := -mcmodel=medany -march=rv32imac -mabi=ilp32 -fno-common -ffunction-sections -fdata-sections -fstrict-volatile-bitfields -O0 -ggdb -fgnu89-inline -Werror
export AFLAGS := -c -mcmodel=medany -march=rv32imac -mabi=ilp32 -x assembler-with-cpp -ggdb
export LFLAGS := -mcmodel=medany -march=rv32imac -mabi=ilp32 -nostartfiles -Wl,--gc-sections,-Map=XiZi_gap8.map,-cref,-u,_start -T $(BSP_ROOT)/link.lds

export APPLFLAGS := -mcmodel=medany -march=rv32imac -mabi=ilp32 -nostartfiles -Wl,--gc-sections,-Map=XiZi_app.map,-cref,-u, -T $(BSP_ROOT)/link_userspace.lds

export CXXFLAGS := -mcmodel=medany -march=rv32imac -mabi=ilp32 -fno-common -ffunction-sections -fdata-sections -fstrict-volatile-bitfields -O0 -ggdb -Werror

export CROSS_COMPILE ?=/opt/gnu-mcu-eclipse/riscv-none-gcc/8.2.0-2.1-20190425-1021/bin/riscv-none-embed-

export DEFINES := -DHAVE_CCONFIG_H -DHAVE_SIGINFO

export ARCH = risc-v
export MCU =  GAP8


