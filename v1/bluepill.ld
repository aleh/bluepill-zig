/* STM32F103xx, found in Maple mini aka "Blue Pill" boards. */

MEMORY {
	flash (rx)	: o = 0x08000000, l = 128K
	sram (rw)	: o = 0x20000000, l = 20K
}

SECTIONS {
	/DISCARD/ : { 
		/* I don't want to keep sections needed only when printing stack traces. */
		*(.ARM.*) 
	}

	.text : {
		/* The first word contains the value for SP to have after reset. */
		LONG(ORIGIN(sram) + LENGTH(sram))
		/* Then goes the Reset vector. */
		LONG(_start);
		/* We should put a bunch of other vectors here, but since none are used yet we can use the space. */
		*(.text)
		*(.rodata.*) 
		*(.rodata) 
	} >flash

	.bss : { 
		*(.bss) 
	} >sram
}

