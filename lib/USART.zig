const common = @import("common.zig");
const GPIO = common.GPIO;
const USARTIndex = common.USARTIndex;

pub fn USART(comptime rcc: type, comptime index: USARTIndex) type {
    return struct {
        const regs = common.RegisterSet(switch (index) {
            .usart1 => 0x4001_3800,
            .usart2 => 0x4000_4400,
            .usart3 => 0x4000_4800,
        });

        const SR = regs.at(0x00, packed struct(u32) {
            _r1: u7,
            /// Transmit data register empty.
            TXE: bool,
            _r2: u24,
        });

        const DR = regs.at(0x04, u32);

        const BRR = regs.at(0x08, u32);

        const CR1 = regs.at(0x0C, packed struct(u32) {
            _r1: u2 = 0,
            /// Receiver enable.
            RE: bool,
            /// Transmitter enable.
            TE: bool,
            _r2: u9 = 0,
            /// USART enable.
            UE: bool,
            _r3: u18 = 0,
        });

        const CR2 = regs.at(0x10, u32);

        const CR3 = regs.at(0x14, u32);

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

            // Go!
            CR1.* = .{
                .UE = true,
                .TE = true,
                .RE = true,
            };
        }

        pub fn write_byte(byte: u8) void {
            DR.* = byte;
            while (!SR.TXE) {}
        }

        pub fn write_string(string: []const u8) void {
            for (string) |byte| {
                if (byte == 0) break;
                write_byte(byte);
            }
        }
    };
}
