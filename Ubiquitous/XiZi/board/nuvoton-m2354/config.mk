export CROSS_COMPILE ?= /opt/gcc-arm-none-eabi-6-2017-q1-update/bin/arm-none-eabi-

export CFLAGS := -mcpu=cortex-m23 -mthumb  -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g -fgnu89-inline -Wa,-mimplicit-it=thumb 
export AFLAGS := -c -mcpu=cortex-m23 -mthumb -ffunction-sections -fdata-sections -x assembler-with-cpp -Wa,-mimplicit-it=thumb  -gdwarf-2
export LFLAGS := -mcpu=cortex-m23 -mthumb -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=XiZi_nuvoton_m2354.map,-cref,-u,Reset_Handler -T $(BSP_ROOT)/link.lds
export CXXFLAGS := -mcpu=cortex-m23 -mthumb -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g 

export DEFINES := -DHAVE_CCONFIG_H  

export ARCH = arm
export MCU = cortex-m23
