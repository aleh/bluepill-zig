# Bare Metal Zig on STM32

This is about using [Zig](https://ziglang.org) alone to directly program boards based on STM32F103xx MCU, such as "Blue
Pill" clones of Maple Mini.

## Requirements

- A Blue Pill board or similar, see [here](https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html) for general and
  physical info.
  
- [Zig](https://ziglang.org/download/) to build our examples. (I used version 0.13.0 here.)

- [ST-Link Tools](https://github.com/stlink-org/stlink) to flash them.

## Docs

- [Blue Pill Schematic](https://stm32-base.org/assets/pdf/boards/original-schematic-STM32F103C8T6-Blue_Pill.pdf).

- [STM32 Cortex®-M3 Programming Manual](https://www.st.com/resource/en/programming_manual/pm0056-stm32f10xxx20xxx21xxxl1xxxx-cortexm3-programming-manual-stmicroelectronics.pdf) for general info on Cortex-M3.

- [STM32F10x Reference Manual](https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf) to know how to program all available peripherals.

- [STM32F103x8 Datasheet](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf) to know what exactly is available
  in our MCU as the above reference manual describes the whole family.

- [Info on Linker Scripts](http://web.mit.edu/rhel-doc/3/rhel-ld-en-3/scripts.html) to be able to describe memory layout to the linker.

## Getting Started

One of the cool things about Zig is that (thanks to LLVM) it can compile for many architectures out of the box and yet
manages to keep its MacOS package size under 50MB without any external dependencies!

Zig expects its `-target` command line switch to be a dash-separated triple identifying architecture, operating system
and ABI (application binary interface) of the target system. See the whole list by running:

    zig targets | less

Our architecture is ARM, we don't have any OS and we don't care about any particular ABI, so we are going to use
`arm-freestanding-none` as our `-target`.

Another command line switch, `-mcpu`, should be used to specify the processor to generate the code for. STM32F103xx is
based on Cortext-M3, so consulting the list of supported targets the most logical choice seemed to be `cortex_m3`. I am
getting build errors when using one however (something about an instruction in `IT` block), so I decided to step back
to `cortex_m23` that seems to be a subset of Cortext-M3 instruction-wise, missing exactly the `IT` instruction. (I
don't expect the compiler to use "TrustZone" security extensions. We could also use `cortex_m0` to be 100% sure it
won't generate something unsupported.)

Before we go to the remaining compiler settings let's type in some code first. Let's make the classic "Blink" example toggling the on-board LED (see `v0/main.zig`). It's not going to be nice, but we'll improve it later.

### Version 0

Looking at the Blue Pill Schematic we see that the LED is attached to PC13 of the MCU which can be controlled via port 13 of GPIO bank C. GPIO and the corresponding registers are described in Chapter 9 of the Reference Manual, while the base address of bank C, `0x4001_1000`, can be found in the STM32F103x8 Datasheet. 

Before we can use the GPIO bank C however we need to enable it via one of the Reset and Clock Control (RCC) registers (base `0x4002_1000`), see chapter 7.3. GPIO bank C is controlled by bit 4 of `RCC_APB2ENR`, offset `0x18`:

```zig
reg(0x4002_1000 + 0x18).* |= 1 << 4;
```

Where `reg()` is a simple wrapper that gets us a `volatile` pointer, which we need when working with memory mapped registers for Zig's optimizer to not try removing or reorder our reads/writes.

```zig
fn reg(comptime address: u32) *volatile u32 {
    return @ptrFromInt(address);
}
```

Before we can start toggling port 13 however we need to configure it as output via `GPIOx_CRH` register (offset `0x04`, see chapter 9.2.2). Each nibble in this register is responsible for configuration of ports 8-15:

```zig
reg(0x4001_1000 + 0x04).* |= 0b01_10 << (4 * (13 - 8)); // Relying on the reset value being 0b01_00.
```

We'll be using `GPIOx_BSRR` register (offset `0x10` from the base, see chapter 9.2.5) to control the output state of our port. Setting bits 0-15 here sets the output on ports 0-15 to 1, while setting bits 16-32 *resets* the output on the same ports. (This register is more convenient than more traditional `GPIOx_ODR`, because there is no need to read its current state to modify a single bit.)

```zig
const GPIOC_BSRR = reg(0x4001_1000 + 0x10);
while (true) {
    GPIOC_BSRR.* = 1 << (13 + 16);
    delay(50);
    GPIOC_BSRR.* = 1 << 13;
    delay(950);
}
```

To implement `delay()` we'll just do something in a long loop. (We'll return to better implementation in the next version of the example.)

```zig
fn delay_ticks(ticks: u32) void {
    var i = ticks;
    while (i > 0) {
        // Reading any location to prevent the loop from being optimized out.
        _ = reg(0x2000_0000).*;
        i -= 1;
    }
}
```

To calculate the number of ticks we need to iterate to get a millisecond delay we need to know that after reset our CPU runs approximately at 8MHz and that every iteration in the above loop takes 6 CPU cycles (more on this below):

```zig
fn delay(comptime ms: u32) void {
    delay_ticks(ms * 8_000 / 6);
}
```

### Linker Script

OK, now when we have a basic program (see `v0/main.zig`) we can try to compile it:

    zig build-exe -target arm-freestanding-none -mcpu cortex_m23 -O ReleaseSmall -femit-asm main.zig

The extra `-femit-asm` flag makes Zig produce assembly output which is handy when examining our code. For example, this is how we can calculate how many CPU clock cycles `delay_ticks()` spends per tick (comments added by me using info on [this page](https://developer.arm.com/documentation/ddi0337/e/Instruction-Timing/Processor-instruction-timings)):

```asm
main.delay_ticks:           ; r0 contains the number of ticks already.
	movs	r1, #1
	lsls	r1, r1, #29     ; (1 << 29) is this 0x20000000 address we are reading from below.
.LBB1_1:
	cbz	r0, .LBB1_3         ; 1 cycle, branch not taken. (Jump out of the loop if tick counter in r0 is zero.)
	ldr	r2, [r1]            ; 2 cycles. (Our fake read.)
	subs	r0, r0, #1      ; 1 cycle. (Decrement the tick counter in r0.)
	b	.LBB1_1             ; 2 cycles, branch is taken. (Repeat the loop.)
.LBB1_3:
	bx	lr                  ; Return from the function.
```

Speaking of assembly, we could also disassemble the output file directly with `objdump -d main`, but it might be harder to see what's going on, here is the same `delay_ticks()` function:

    20130: 2101         	movs	r1, #1
    20132: 0749         	lsls	r1, r1, #29
    20134: b110         	cbz	r0, 0x2013c <.text+0x50> @ imm = #4
    20136: 680a         	ldr	r2, [r1]
    20138: 1e40         	subs	r0, r0, #1
    2013a: e7fb         	b	0x20134 <.text+0x48>    @ imm = #-10
    2013c: 4770         	bx	lr

Another thing that we can see with `objdump` is that our code starts at address `000200ec` which is quite wrong for our MCU where flash memory begins at `0x08000000`:

    main:	file format elf32-littlearm

    Disassembly of section .text:

    000200ec <.text>:
       200ec: 480c         	ldr	r0, [pc, #48]           @ 0x20120 <.text+0x34>
       200ee: 6801         	ldr	r1, [r0]
       200f0: 2210         	movs	r2, #16
    ...

Well, this is logical because Zig does not really know much about our MCU. We need to help it by writing a "linker script". The official documentation on the topic mentioned above is easy to read and actual scripts are fairly self-explanatory.

The first thing we do in our script (`v0/bluepill.ld`) is describing relevant memory regions, which is quite simple in our case as we have 128K of flash memory starting at `0x08000000` and 20K of RAM starting at `0x20000000` (see chapter 4 in the Datasheet):

    MEMORY {
    	flash (rx)	: o = 0x08000000, l = 128K
    	sram (rw)	: o = 0x20000000, l = 20K
    }

(The names of the regions can be arbitrary here, the linker does not know what the "flash" is.)

The next part of the script tells what should be placed into the flash memory:

    SECTIONS {
    	.text : {
            ...
    	} >flash

We cannot tell it to begin filling with the code from the start as the first word has to be the value of the main stack pointer (MSP), as per chapter 2.1.2 of the Programming Manual:

> On reset, the processor loads the MSP with the value from address 0x00000000.

(The address is from the start of the flash, `0x08000000` in our case.)

Next go interrupt vectors (see table 63 in the Reference Manual) of which we are only interested in the first one, Reset, as we don't use interrupts just yet:

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

Next we tell that our writable data (static variables) is expected in RAM. We don't have such variables in our simple example yet, but that'll be handy later.

	.bss : { 
		*(.bss) 
	} >sram

And finally we exclude a few extra code segments that otherwise would increase the size of our binary:

	/DISCARD/ : { 
		/* I don't want to keep sections needed only when printing stack traces. */
		*(.ARM.*)
	}

Let's compile using our linker script now:

    zig build-exe -target arm-freestanding-none -mcpu cortex_m23 -femit-asm -O ReleaseSmall --script bluepill.ld main.zig

Disassembling with `objdump` shows that the addresses are correct now:

    main:	file format elf32-littlearm

    Disassembly of section .text:

    08000000 <.text>:
     8000000: 20005000     	andhs	r5, r0, r0
     8000004: 08000009     	stmdaeq	r0, {r0, r3}
     8000008: 6801480c     	stmdavs	r1, {r2, r3, r11, lr}
    ...

The first word appears to be the desired stack pointer just beyond the RAM followed by the Reset vector pointing to the next word. The number is even to indicate Thumb mode. Let's add `--mcpu=cortex-m23` to force Thumb mode:

    08000000 <.text>:
     8000000: 5000         	str	r0, [r0, r0]
     8000002: 2000         	movs	r0, #0
     8000004: 0009         	movs	r1, r1
     8000006: 0800         	lsrs	r0, r0, #32
     8000008: 480c         	ldr	r0, [pc, #48]           @ 0x800003c <.text+0x3c>
     800000a: 6801         	ldr	r1, [r0]
     800000c: 2210         	movs	r2, #16
     ...
     
OK, now the part starting at `0x8000008` looks like the code in our `.s` file.

### Flashing

We'll be using `st-flash` utility which expects either a raw binary with the starting address passes separately or an Intel hex file that already contains addresses. Let's use the latter by converting our build to `.hex` with Zig:

    zig objcopy -O hex main main.hex

Flashing is then as simple as:

    st-flash --reset --format ihex write main.hex 

You'll see something like this and your LED will hopefully start blinking every second:

    st-flash 1.8.0
    2024-08-18T22:01:38 INFO common.c: STM32F1xx_MD: 20 KiB SRAM, 128 KiB flash in at least 1 KiB pages.
    2024-08-18T22:01:38 INFO common_flash.c: Attempting to write 90 (0x5a) bytes to stm32 address: 134217728 (0x8000000)
    -> Flash page at 0x8000000 erased (size: 0x400)
    2024-08-18T22:01:38 INFO flash_loader.c: Starting Flash write for VL/F0/F3/F1_XL
    2024-08-18T22:01:38 INFO flash_loader.c: Successfully loaded flash loader in sram
    2024-08-18T22:01:38 INFO flash_loader.c: Clear DFSR
      1/1   pages written
    2024-08-18T22:01:38 INFO common_flash.c: Starting verification of write complete
    2024-08-18T22:01:38 INFO common_flash.c: Flash written and verified! jolly good!

Also, as you can see our code is just 90 bytes, which is quite nice given all the required setup instructions.

### V1

Now let's improve the example showing some power of Zig.

```zig

export fn _start() noreturn {
    const bankC = GPIOBank(.C);
    bankC.init();

    const led = bankC.port(13);
    led.setOutput(.openDrain, .max2MHz);

    while (true) {
        led.reset();
        delay(50);
        led.set();
        delay(950);
    }
}
```

The `GPIOBank` is an abstraction that is more readable, more reusable (we can use all banks/ports) but does not add any overhead. Our program is 94 bytes, which is just 4 bytes larger only because we are not relying on the reset values when writing to `GPIOC_CRH` as we want to change pin configuration at runtime.

### V2

Here we are adding `SysTick` timer for nicer `delay()` along with `USART` to say the actual `Hello`.

---
