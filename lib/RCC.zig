const common = @import("common.zig");

/// Wraps the Reset and Clock Control registers.
pub fn RCC(comptime config: RCCConfig) type {
    const regs = common.RegisterSet(0x4002_1000);

    const CR = regs.at(0x00, packed struct(u32) {
        _r1: u16,
        /// Enable high-speed external clock.
        HSEON: bool,
        /// High-speed external clock is ready.
        HSERDY: bool,
        _r2: u14,
    });

    const ClockSource = enum(u2) {
        /// Internal RC.
        HSI = 0b00,
        /// External crystal.
        HSE = 0b01,
        PLL = 0b11,
    };
    const CFGR = regs.at(0x04, packed struct(u32) {
        // The clock source we want to switch to.
        SW: ClockSource,
        // The actual clock source used now.
        SWS: ClockSource,
        _r1: u28,
    });

    const APB2ENR = regs.at(0x18, u32);
    const APB1ENR = regs.at(0x1C, u32);

    return struct {
        /// Frequency of the CPU itself. Note that different peripherals use different clock sources.
        pub const SYSCLK = switch (config) {
            .internalRC => 8_000_000,
            .external => 8_000_000,
            .PLL => {
                @compileError("PLL is not supported yet");
            },
        };

        fn setClockSource(comptime clockSource: ClockSource) void {
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
                    // Turn the high-speed external clock.
                    CR.HSEON = true;
                    // We don't have to wait for it to become ready as this is done by the CPU when switching the source.
                    //~ while (!CR.HSERDY) {}
                    setClockSource(.HSE);
                },
                .PLL => {
                    @compileError("PLL is not supported yet");
                },
            }
        }

        pub fn enableGPIO(comptime bank: common.GPIOBank) void {
            APB2ENR.* |= 1 << (@intFromEnum(bank) + 2);
        }

        pub fn enableUSART(comptime index: common.USARTIndex) void {
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
