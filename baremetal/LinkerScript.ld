MEMORY
{
    RAM(xrw)    : ORIGIN = 0x20000000,   LENGTH = 20K
    FLASH(rx)   : ORIGIN = 0x8000000,   LENGTH = 64K
}

SECTIONS
{
   .text : {
        *(.isr_vector*)
        *(.text*)
    } > FLASH
   
   .bss  : {
        *(.bss*)
    } > RAM
}
