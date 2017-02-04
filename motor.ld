
_min_stack_size = 256;

MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512k
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128k
}

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

/* "PROVIDE" let the variable can be redefined in program.
 * otherwise, linker will report "redefined" error.
 */
PROVIDE(_stack_limit = _estack - _min_stack_size);

SECTIONS
{
    /* load other .text in SRAM */
    .text : 
    {
        KEEP(*(.isr_vector))
       _text = .;
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(4);
        _etext = .;
    } >FLASH
    _sitext = LOADADDR(.text);

    /*.data _etext : AT (LOADADDR(.text) + SIZEOF(.text))*/
    .data :
    {
        _sdata = .;
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        _edata = .;
    } >SRAM AT>FLASH
    _sidata = LOADADDR(.data);

    .bss (NOLOAD) :
    {
        _sbss = .;
        *(.bss)
        . = ALIGN(4);
        _ebss = .;
    } >SRAM

    _end = .;
}
