ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 128K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .text : {
        KEEP(*(.isr_vector))
        *(.text*)
    } > FLASH

    .rodata : {
        *(.rodata*)
    } > FLASH

    _sidata = LOADADDR(.data);

    .data : {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM AT> FLASH

    .bss : {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    . = ALIGN(4);
    _end = .;
    PROVIDE(end = .);
}