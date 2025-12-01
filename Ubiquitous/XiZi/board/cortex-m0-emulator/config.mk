export CROSS_COMPILE ?=/usr/bin/arm-none-eabi-

export CFLAGS := -mcpu=cortex-m0 -mthumb -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g -fgnu89-inline -Wa,-mimplicit-it=thumb 
export AFLAGS := -c -mcpu=cortex-m0 -mthumb -ffunction-sections -fdata-sections -x assembler-with-cpp -Wa,-mimplicit-it=thumb  -gdwarf-2
export LFLAGS := -mcpu=cortex-m0 -mthumb -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=XiZi_cortex-m0-emulator.map,-cref,-u,Reset_Handler -T $(BSP_ROOT)/link.lds
export CXXFLAGS := -mcpu=cortex-m0 -mthumb -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g 

export APPLFLAGS := -mcpu=cortex-m0 -mthumb -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=XiZi_app.map,-cref,-u, -T $(BSP_ROOT)/link_userspace.lds


export DEFINES := -DHAVE_CCONFIG_H -g

export ARCH = arm
export MCU = cortex-m0
