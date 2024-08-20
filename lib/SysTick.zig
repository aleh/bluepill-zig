const common = @import("common.zig");

/// Very simple wrapper for the SysTick timer.
pub fn SysTick(comptime cpuFreq: u32, comptime msPerTick: u32) type {
    return struct {
        /// The total tick counter we increment on every interrupt.
        var counter: u32 = undefined;

        export fn SysTick_Vector() void {
            counter = counter +% 1;
        }

        // See chapter 4.5 of the Programming manual.
        const regs = common.RegisterSet(0xE000_E010);

        const STK_CTRL = regs.at(0, packed struct(u32) {
            /// Counter enable.
            ENABLE: bool,
            TICKINT: bool,
            CLKSOURCE: enum(u1) {
                /// AHB/8.
                AHB_8 = 0,
                /// Processor clock (AHB).
                AHB = 1,
            },
            _r1: u13 = 0,
            COUNTFLAG: bool = false,
            _r2: u15 = 0,
        });

        const STK_LOAD = regs.at(4, u32);
        //~ const STK_VAL = reg(8);
        //~ const STK_CALIB = reg(12)

        pub fn init() void {
            counter = 0;
            // We want a tick interrupt every msPerTick milliseconds.
            // The value of the counter is decremented every 8th cycle of the CPU clock.
            STK_LOAD.* = (cpuFreq / 8) * msPerTick / 1000; // TODO: minus one?
            STK_CTRL.* = .{
                .ENABLE = true,
                .TICKINT = true,
                .CLKSOURCE = .AHB_8,
            };
        }

        pub fn value() u32 {
            return @as(*volatile u32, &counter).*;
        }

        fn delay_ticks(ticks: u32) void {
            const start = value();
            while (value() - start < ticks) {
                common.wait_for_interrupt();
            }
        }

        pub fn delay(comptime ms: u32) void {
            delay_ticks((ms + msPerTick - 1) / msPerTick);
        }
    };
}
