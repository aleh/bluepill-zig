/* STM32F103xx, found in Maple mini aka "Blue Pill" boards. */

MEMORY {
    flash (rx)  : o = 0x08000000, l = 128K
    sram (rw)   : o = 0x20000000, l = 20K
}

SECTIONS {
    .text : {
        /* The initial value of SP, past the end of RAM. */
        LONG(ORIGIN(sram) + LENGTH(sram))
        /* Reset vector. */
        LONG(_start)
        /* We should put a bunch of other vectors here, but since none are used yet we can use the space. */
        /* So now goes our code. */
        *(.text)
        /* Then read-only data. */
        *(.rodata.*) 
        *(.rodata) 
    } >flash

    .bss : { 
        *(.bss) 
    } >sram

    /DISCARD/ : { 
        /* I don't want to keep sections needed only when printing stack traces. */
        *(.ARM.*)
    }
}
