// V1: a self-contained LED blinking example for STM32F103-based Maple mini (aka "Blue Pill") boards.

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

/// A basic wrapper for an STM32 GPIO bank: can enable the corresponding clock and vend all the "ports".
pub fn GPIOBank(comptime bank: GPIOBankIndex) type {
    return struct {
        pub fn init() void {
            // We need to enable clock for the bank via Reset and Clock Control registers.
            const APB2ENR: *volatile u32 = @ptrFromInt(0x4002_1000 + 0x18);
            APB2ENR.* |= 1 << (@intFromEnum(bank) + 2);
        }

        /// A wrapper for a single GPIO pin/port in this bank.
        pub fn port(comptime pin: u4) type {
            return struct {
                fn reg(comptime offset: u32) *volatile u32 {
                    return @ptrFromInt(switch (bank) {
                        .A => 0x4001_0800,
                        .B => 0x4001_0C00,
                        .C => 0x4001_1000,
                        .D => 0x4001_1400,
                        .E => 0x4001_1800,
                    } + offset);
                }

                fn setModeBits(comptime bits: u32) void {
                    const CRx = reg(if (pin >= 8) 0x04 else 0x00);
                    const shift = 4 * @as(u8, if (pin >= 8) pin - 8 else pin);
                    CRx.* = CRx.* & ~(@as(u32, 0xF) << shift) | (bits << shift);
                }

                pub const OutputSpeed = enum(u32) { max10MHz = 0b01, max2MHz = 0b10, max50MHz = 0b11 };

                pub const OutputMode = enum(u32) { pushPull = 0b00_00, openDrain = 0b01_00 };

                /// Configures the port as output with the given settings.
                pub fn setOutput(comptime mode: OutputMode, comptime speed: OutputSpeed) void {
                    setModeBits(@intFromEnum(mode) | @intFromEnum(speed));
                }

                const BSRR = reg(0x10);

                pub fn set() void {
                    BSRR.* = 1 << pin;
                }

                pub fn reset() void {
                    BSRR.* = 1 << (@as(u8, pin) + 16);
                }
            };
        }
    };
}

pub const GPIOBankIndex = enum(u32) { A, B, C, D, E };

fn delay(comptime ms: u32) void {
    // The CPU runs at 8MHz now and one tick of the loop is about 6 cycles. (Check the output .s file and count using
    // <https://developer.arm.com/documentation/ddi0337/e/Instruction-Timing/Processor-instruction-timings>.)
    delay_ticks(ms * 8_000 / 6);
}

fn delay_ticks(ticks: u32) void {
    var i = ticks;
    while (i > 0) {
        // Reading any location to prevent the loop from being optimized out.
        _ = @as(*volatile u32, @ptrFromInt(0x2000_0000)).*;
        i -= 1;
    }
}
