const common = @import("common.zig");

/// Very simple wrapper for the SysTick timer.
pub fn SysTick(comptime cpuFreq: u32, comptime msPerTick: u32) type {
    return struct {
        export fn SysTick_Vector() void {
            total_ms +%= msPerTick;
        }

        var total_ms: u32 = undefined;

        /// The total milliseconds since initialization.
        pub inline fn milliseconds() u32 {
            return @as(*volatile u32, &total_ms).*;
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

        pub fn init() void {
            total_ms = 0;

            // We want a tick interrupt every msPerTick milliseconds.
            // The value of the counter is decremented every cycle of the CPU clock.
            // It should be one cycle less to account for one cycle between 0 and reloading.
            const load = @as(u64, cpuFreq) * msPerTick / 1000 - 1;
            if (!(1 <= load and load <= 0xFFFFFF)) {
                @compileError("Invalid SysTick load value for the given msPerTick and CPU frequency");
            }
            STK_LOAD.* = load;

            STK_CTRL.* = .{
                .ENABLE = true,
                .TICKINT = true,
                .CLKSOURCE = .AHB,
            };
        }

        pub fn delay(ms: u32) void {
            const start = milliseconds();
            while (milliseconds() - start < ms) {
                common.wait_for_interrupt();
            }
        }
    };
}
