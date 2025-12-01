/*
 * linker script for STM32F10x with GNU ld
 */

/* Program Entry, set to mark it as "used" and avoid gc */
MEMORY
{
    flash (rx) : ORIGIN = 0x08000000, LENGTH = 128k /* 128KB flash */
    sram (rw) : ORIGIN = 0x20000000, LENGTH =  20k /* 20K sram */
}
OUTPUT_ARCH(arm)

ENTRY(Reset_Handler)
_system_stack_size = 0x200;

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
        *(.glue_7)
        *(.glue_7t)

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

        _etext = .;
    } > flash = 0

    /* .ARM.exidx is sorted, so has to go in its own output section.  */
    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)

        /* This is used by the startup in order to initialize the .data secion */
        _sidata = .;
    } > flash
    __exidx_end = .;

    /* .data section which is used for initialized data */

    .data : AT (_sidata)
    {
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        _sdata = . ;

        *(.data)
        *(.data.*)
        
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        _edata = . ;
    } >sram

    __bss_start = .;
    .bss :
    {
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _sbss = .;

        *(.bss)
        *(.bss.*)
        *(COMMON)

        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _ebss = . ;
    } > sram
    __bss_end = .;

    _end = .;

   .stack ORIGIN(sram) + LENGTH(sram) - _system_stack_size :
    {
    PROVIDE( _heap_end = . );
    . = _system_stack_size;
    PROVIDE( _sp = . );
    } >sram 

}
