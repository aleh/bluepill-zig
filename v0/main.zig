// V0: a small but ugly version of a LED blinking example for STM32F103-based Maple mini (aka "Blue Pill") boards.

export fn _start() noreturn {

    // Enable clock for GPIO bank C via RCC_APB2ENR register.
    reg(0x4002_1000 + 0x18).* |= 1 << 4;

    // Configure GPIO C port 13 for slow output by setting CNF13[1:0] and MODE13[1:0] bits at once:
    // - CNF13[1:0]:  "01: General purpose output Open-drain".
    // - MODE13[1:0]: "10: Output mode, max speed 2 MHz".
    reg(0x4001_1000 + 0x04).* |= 0b01_10 << (4 * (13 - 8)); // Relying on the reset value being 0b01_00.

    // Setting bits 0-15 of this reg sets ports 0-15 correspondingly, while setting bits 16-31 resets them.
    const GPIOC_BSRR = reg(0x4001_1000 + 0x10);

    while (true) {
        GPIOC_BSRR.* = 1 << (13 + 16);
        delay(50);
        GPIOC_BSRR.* = 1 << 13;
        delay(950);
    }
}

fn delay(comptime ms: u32) void {
    // The CPU runs at 8MHz now and one tick of the loop is about 6 cycles. (Check the output .s file and count using
    // <https://developer.arm.com/documentation/ddi0337/e/Instruction-Timing/Processor-instruction-timings>.)
    delay_ticks(ms * 8_000 / 6);
}

fn delay_ticks(ticks: u32) void {
    var i = ticks;
    while (i > 0) {
        // Reading any location to prevent the loop from being optimized out.
        _ = reg(0x2000_0000).*;
        i -= 1;
    }
}

/// A volatile pointer to location with the given address, to make register declarations less noisy.
fn reg(comptime address: u32) *volatile u32 {
    return @ptrFromInt(address);
}
