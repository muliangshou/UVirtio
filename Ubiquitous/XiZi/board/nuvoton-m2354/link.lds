/*
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

/* Linker script to configure memory regions. */
MEMORY
{
    flash (rx) : ORIGIN = 0x00000000, LENGTH = 1024K   /* 1024K flash */
    sram (rw) : ORIGIN = 0x20000000, LENGTH = 256K    /* 256K sram */
}
ENTRY(Reset_Handler)
_system_stack_size = 0x2000;

SECTIONS
{    
    .text :
    {        
        . = ALIGN(4);
        KEEP(*(.isr_vector))            /* Startup code */
        . = ALIGN(4);
        *(.text)                        /* remaining code */
        *(.text.*)                      /* remaining code */
        *(.rodata)                      /* read-only data (constants) */
        *(.rodata*)

        /* section information for shell */
        . = ALIGN(4);
        _shell_command_start = .;
        KEEP (*(shellCommand))
        _shell_command_end = .;
        . = ALIGN(4);
      
        __isrtbl_idx_start = .;
        KEEP(*(.isrtbl.idx))
        __isrtbl_start = .;
        KEEP(*(.isrtbl))
        __isrtbl_end = .;
        . = ALIGN(4);

        PROVIDE(g_service_table_start = ABSOLUTE(.));
        KEEP(*(.g_service_table))
        PROVIDE(g_service_table_end = ABSOLUTE(.));

        PROVIDE(_etext =  ABSOLUTE(.));

        . = ALIGN(4);
        _etext = .;
    } > flash = 0

    /* .ARM.exidx is sorted, so has to go in its own output section.  */
    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)

        /* This is used by the startup in order to initialize the .data section */
        _sidata = .;
    } > flash
    __exidx_end = .;

    .stack : 
    {
        . = . + _system_stack_size;
        . = ALIGN(4);
        _sp = .;
    } > sram

    /* .data section which is used for initialized data */
    .data : AT (_sidata)
    {
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data section */
        _sdata = . ;

        *(.data)
        *(.data.*)

        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data section */
        _edata = . ;
    } > sram

    __bss_start = .;
    .bss :
    {
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss section */
        _sbss = .;

        *(.bss)
        *(.bss.*)
        *(COMMON)

        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss section */
        _ebss = . ;
    } > sram
    __bss_end = .;

    _end = .;

}
