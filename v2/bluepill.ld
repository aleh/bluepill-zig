/* STM32F103xx, found in Maple mini aka "Blue Pill" boards. */

MEMORY {
	flash (rx)  : o = 0x08000000, l = 128K
	sram (rw)   : o = 0x20000000, l = 20K
}

SECTIONS {
	.text : {
		/* The first word contains the value for SP to have after reset. */
		LONG(ORIGIN(sram) + LENGTH(sram))
        
		/* Then goes the Reset vector. */
		LONG(_start);
        
        /* We don't use any interrupt vectors before SysTick, so let's just fill.  */
        FILL(0); . = ADDR(.text) + 0x003C;
        
        LONG(_SysTickInt);
        
		/* Other vectors follow, but since we are not using them we can just start our code earlier. */
        
		*(.text)
        
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

