export fn _start() noreturn {
    const rcc = RCC(.internalRC);
    rcc.init();

    const sysTick = SysTick(rcc.SYSCLK, 50);
    sysTick.init();

    const led = GPIO(rcc, .C).port(13);
    led.Bank.init();
    led.setOutput(.openDrain, .max2MHz);

    const usart = USART(rcc, .usart1);
    usart.init(115200);

    while (true) {
        led.reset();
        sysTick.delay(50);
        led.set();
        sysTick.delay(950);
        usart.write_string("Hello\n");
    }
}

/// Helps defining register pointers.
pub fn RegisterSet(comptime base: u32) type {
    return struct {
        pub fn raw_ptr(comptime offset: u32) *volatile u32 {
            return @ptrFromInt(base + offset);
        }

        pub fn struct_ptr(comptime offset: u32, comptime PackedStruct: type) *volatile PackedStruct {
            switch (@typeInfo(PackedStruct)) {
                .Struct => |s| {
                    switch (s.layout) {
                        .@"packed" => {},
                        else => {
                            @compileError("A *packed* struct is expected");
                        },
                    }
                },
                else => {
                    @compileError("A packed *struct* is expected");
                },
            }
            return @ptrFromInt(base + offset);
        }
    };
}

const RCCConfig = union(enum) {
    /// HSI: internal RC oscillator, 8MHz.
    internalRC: void,
    /// HSE: external crystal without PLL, 8MHz on the Blue Pill.
    external: void,
    /// PLL: use external crystal and multiply its frequency.
    PLL: void,
};

/// Wraps the Reset and Clock Control registers.
pub fn RCC(comptime config: RCCConfig) type {
    const regs = RegisterSet(0x4002_1000);
    const CR = regs.struct_ptr(0x00, packed struct(u32) {
        reserved1: u16,
        /// Enable high-speed external clock.
        HSEON: bool,
        /// High-speed external clock is ready now.
        HSERDY: bool,
        reserved2: u14,
    });
    const ClockSource = enum(u2) {
        /// Internal RC.
        HSI = 0b00,
        /// External crystal.
        HSE = 0b01,
        PLL = 0b11,
    };
    const CFGR = regs.struct_ptr(0x04, packed struct(u32) {
        // The clock source we want to switch to.
        SW: ClockSource,
        // The actual clock source used now.
        SWS: ClockSource,
        reserved: u28,
    });
    //~ const AHBENR = regs.raw_ptr(0x14);
    const APB2RSTR = regs.raw_ptr(0x0C);
    //~ const APB1RSTR = regs.raw_ptr(0x10);
    const APB2ENR = regs.raw_ptr(0x18);
    const APB1ENR = regs.raw_ptr(0x1C);

    return struct {
        /// Frequency of the CPU itself. Note that different peripherals use different clock sources.
        pub const SYSCLK = switch (config) {
            .internalRC => 8_000_000,
            .external => 8_000_000,
            .PLL => {
                @compileError("PLL is not supported yet");
            },
        };

        inline fn setClockSource(clockSource: ClockSource) void {
            // Select it for the system clock and wait for the switch to happen.
            CFGR.SW = clockSource;
            while (CFGR.SWS != clockSource) {}
        }

        pub fn init() void {
            switch (config) {
                .internalRC => {
                    // Nothing to do, it's default after a reset.
                },
                .external => {
                    // Turn the high-speed external clock on and wait for it to become ready.
                    CR.HSEON = true;
                    // We don't have to wait for HSERDY as this will be done by the CPU when switching the clock source.
                    while (!CR.HSERDY) {}

                    setClockSource(.HSE);
                },
                .PLL => {
                    @compileError("PLL is not supported yet");
                },
            }
        }

        pub fn resetGPIO(comptime bank: GPIOBank) void {
            const mask: u32 = 1 << (@intFromEnum(bank) + 2);
            APB2RSTR.* |= mask;
            APB2RSTR.* &= ~mask;
        }

        pub fn enableGPIO(comptime bank: GPIOBank) void {
            APB2ENR.* |= 1 << (@intFromEnum(bank) + 2);
        }

        pub fn enableUSART(comptime index: USARTIndex) void {
            switch (index) {
                .usart1 => {
                    APB2ENR.* |= 1 << 14;
                },
                .usart2 => {
                    APB1ENR.* |= 1 << 17;
                },
                .usart3 => {
                    APB1ENR.* |= 1 << 18;
                },
            }
            // Alternate function clock?
            // const AFIOEN = 0;
            // APB2ENR.* |= 1 << AFIOEN;
        }
    };
}

const GPIOBank = enum(u32) { A, B, C, D, E };

pub fn GPIO(comptime rcc: type, comptime bank: GPIOBank) type {
    return struct {
        /// It's safe to call multiple times.
        pub fn init() void {
            rcc.enableGPIO(bank);
        }

        pub fn port(comptime pin: u4) type {
            return struct {
                /// The bank this port is part of.
                pub const Bank = GPIO(rcc, bank);

                fn reg(comptime offset: u32) *volatile u32 {
                    const base = switch (bank) {
                        .A => 0x4001_0800,
                        .B => 0x4001_0C00,
                        .C => 0x4001_1000,
                        .D => 0x4001_1400,
                        .E => 0x4001_1800,
                    };
                    return @ptrFromInt(base + offset);
                }

                fn setModeBits(comptime bits: u32) void {
                    const CRx = reg(if (pin >= 8) 0x04 else 0x00);
                    const shift = 4 * @as(u8, if (pin >= 8) pin - 8 else pin);
                    CRx.* = CRx.* & ~(@as(u32, 0xF) << shift) | (bits << shift);
                }

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

                pub fn setOutput(comptime mode: OutputMode, comptime speed: OutputSpeed) void {
                    setModeBits(@intFromEnum(mode) | @intFromEnum(speed));
                }

                pub fn setAlternative(comptime mode: AlternativeMode, comptime speed: OutputSpeed) void {
                    setModeBits(@intFromEnum(mode) | @intFromEnum(speed));
                }

                pub fn setInput(comptime mode: InputMode) void {
                    setModeBits(@intFromEnum(mode));
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

inline fn wait_for_interrupt() void {
    asm volatile ("wfi");
}

const USARTIndex = enum { usart1, usart2, usart3 };

pub fn USART(comptime rcc: type, comptime index: USARTIndex) type {
    return struct {
        fn reg(comptime offset: u32) *volatile u32 {
            return @ptrFromInt(switch (index) {
                .usart1 => 0x4001_3800,
                .usart2 => 0x4000_4400,
                .usart3 => 0x4000_4800,
            } + offset);
        }

        const SR = reg(0x00);
        const DR = reg(0x04);
        const BRR = reg(0x08);
        const CR1 = reg(0x0C);
        const CR2 = reg(0x10);
        const CR3 = reg(0x14);

        pub fn init(comptime baud_rate: u32) void {
            const rx = switch (index) {
                .usart1 => GPIO(rcc, .A).port(10),
                .usart2 => GPIO(rcc, .A).port(3),
                .usart3 => GPIO(rcc, .B).port(11),
            };
            const tx = switch (index) {
                .usart1 => GPIO(rcc, .A).port(9),
                .usart2 => GPIO(rcc, .A).port(2),
                .usart3 => GPIO(rcc, .B).port(10),
            };
            rx.Bank.init();
            if (tx.Bank != rx.Bank) {
                tx.Bank.init();
            }
            rx.setInput(.floating);
            tx.setAlternative(.pushPull, .max10MHz);

            rcc.enableUSART(index);

            const clock = rcc.SYSCLK; // TODO: use peripheral clock 1 or 2.

            // Set the baud rate divider, a 12.4 fixed point value.
            const divider: u32 = clock / baud_rate;
            if (divider > 0xFFFF) {
                @compileError("The baud rate divider must fit 16 bits");
            }
            if ((@as(f64, clock) / divider - baud_rate) / baud_rate >= 0.01) {
                @compileError("The actual baud rate is going to be too much off");
            }
            BRR.* = divider;

            const UE = 13;
            const RE = 2;
            const TE = 3;
            CR1.* = (1 << UE) | (1 << TE) | (1 << RE);
        }

        pub fn write_byte(byte: u8) void {
            DR.* = byte;
            // Wait for the transmit data register to become empty.
            const TXE = 7;
            while ((SR.* & (1 << TXE)) == 0) {}
        }

        pub fn write_string(string: []const u8) void {
            for (string) |byte| {
                if (byte == 0) break;
                write_byte(byte);
            }
        }
    };
}

/// Very simple wrapper for the SysTick timer.
pub fn SysTick(comptime cpuFreq: u32, comptime msPerTick: u32) type {
    return struct {
        export fn _SysTickInt() callconv(.C) void {
            counter += 1;
        }

        /// The total tick counter we increment on every interrupt.
        var counter: u32 = undefined;

        fn reg(comptime offset: u32) *volatile u32 {
            return @ptrFromInt(0xE000_E010 + offset);
        }

        const STK_CTRL = reg(0);
        const STK_LOAD = reg(4);
        //~ const STK_VAL = reg(8);
        //~ const STK_CALIB = reg(12)

        pub fn init() void {
            counter = 0;
            // We want a tick interrupt every msPerTick milliseconds.
            // The value of the counter is derecemented 8th cycle of the CPU clock.
            STK_LOAD.* = (cpuFreq / 8) * msPerTick / 1000;
            STK_CTRL.* = (0 << 2) // Use CPU clock divided by 8.
            | (1 << 1) // With interrupts.
            | (1 << 0); // Enable the counter.
        }

        pub fn value() u32 {
            return @as(*volatile u32, &counter).*;
        }

        fn delay_ticks(ticks: u32) void {
            const start = value();
            while (value() - start < ticks) {
                wait_for_interrupt();
            }
        }

        pub fn delay(comptime ms: u32) void {
            delay_ticks((ms + msPerTick - 1) / msPerTick);
        }
    };
}
