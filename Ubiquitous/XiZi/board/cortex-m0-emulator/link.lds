
MEMORY {
    flash (rx) : ORIGIN = 0x00000000, LENGTH = 0x40000
    sram (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x4000
}

OUTPUT_FORMAT ("elf32-littlearm")
__SYSTEM_STACKSIZE__ = 0x400;

ENTRY(Reset_Handler)
SECTIONS {
    .text : {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
        *(.text*)
        *(.rodata*)

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

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        _sidata = .;
    } > flash
    __exidx_end = .;

    .data : AT (_sidata) {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > sram

    .bss : {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .; 
    } > sram
    __bss_end = .;
    __end = .;

   .stack ORIGIN(sram) + LENGTH(sram) - __SYSTEM_STACKSIZE__ :
  {
    PROVIDE( _heap_end = . );
    . = __SYSTEM_STACKSIZE__;
    PROVIDE( _sp = . );
  } >sram 
}
