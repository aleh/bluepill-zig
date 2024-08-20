/// Helps defining register pointers starting from the given base addresss.
pub fn RegisterSet(comptime base: u32) type {
    return struct {
        /// A pointer to the register at `offset` from the set's `base` interpreted as `u32` or a packed struct.
        /// When using packed structs watch out for registers allowing Word-access only!
        pub fn at(comptime offset: u32, comptime reg_type: type) *volatile reg_type {
            const valid = switch (@typeInfo(reg_type)) {
                .Struct => |s| switch (s.layout) {
                    .@"packed" => s.backing_integer == u32,
                    else => false,
                },
                .Int => |i| i.bits == 32,
                else => false,
            };
            if (!valid) {
                @compileError("Expected `reg_type` to be u32 or a packed struct backed by u32");
            }
            return @ptrFromInt(base + offset);
        }
    };
}

pub const USARTIndex = enum { usart1, usart2, usart3 };

pub const GPIOBank = enum(u32) { A, B, C, D, E };

pub fn GPIO(comptime rcc: type, comptime bank: GPIOBank) type {
    return struct {
        /// It's safe to call multiple times.
        pub fn init() void {
            rcc.enableGPIO(bank);
        }

        const BankType = @This();

        pub fn port(comptime pin: u4) type {
            return struct {
                /// The bank this port is part of.
                pub const Bank = BankType;

                const regs = RegisterSet(switch (bank) {
                    .A => 0x4001_0800,
                    .B => 0x4001_0C00,
                    .C => 0x4001_1000,
                    .D => 0x4001_1400,
                    .E => 0x4001_1800,
                });

                const OutputSpeed = enum(u32) { max10MHz = 0b01, max2MHz = 0b10, max50MHz = 0b11 };

                // All modes are in the form CNFx:MODEx already.

                const OutputMode = enum(u32) { pushPull = 0b00_00, openDrain = 0b01_00 };

                const AlternativeMode = enum(u32) { pushPull = 0b10_00, openDrain = 0b11_00 };

                const InputMode = enum(u32) {
                    analog = 0b00_00,
                    floating = 0b01_00,
                    /// Use set() to pull up and reset() to pull down.
                    pulled = 0b10_00,
                };

                fn setModeBits(comptime bits: u32) void {
                    const CRx = regs.at(if (pin >= 8) 0x04 else 0x00, u32);
                    const shift = 4 * @as(u8, if (pin >= 8) pin - 8 else pin);
                    CRx.* = CRx.* & ~(@as(u32, 0xF) << shift) | (bits << shift);
                }

                pub fn setOutput(comptime mode: OutputMode, comptime speed: OutputSpeed) void {
                    setModeBits(@intFromEnum(mode) | @intFromEnum(speed));
                }

                pub fn setAlternative(comptime mode: AlternativeMode, comptime speed: OutputSpeed) void {
                    setModeBits(@intFromEnum(mode) | @intFromEnum(speed));
                }

                pub fn setInput(comptime mode: InputMode) void {
                    setModeBits(@intFromEnum(mode));
                }

                // This register supports Word access only, so we cannot use a packed struct here.
                const BSRR = regs.at(0x10, u32);

                pub fn set() void {
                    BSRR.* = 1 << pin;
                }

                pub fn reset() void {
                    BSRR.* = 1 << (@as(u5, pin) + 16);
                }
            };
        }
    };
}

pub inline fn wait_for_interrupt() void {
    asm volatile ("wfi");
}
