// V2: a self-contained LED blinking example with SysTick timer and USART for STM32F103-based Maple mini (aka "Blue Pill") boards.

const std = @import("std");
const z41 = @import("z41");

export fn _start() noreturn {
    const rcc = z41.RCC(.internalRC);
    rcc.init();

    const SysTick = z41.SysTick(rcc.SYSCLK, 50);
    SysTick.init();

    const led = z41.GPIO(rcc, .C).port(13);
    led.Bank.init();
    led.setOutput(.openDrain, .max2MHz);

    const usart = z41.USART(rcc, .usart1);
    usart.init(115200);

    usart.writeBytes("\nHello! It's V2\n\n");

    while (true) {
        led.reset();
        SysTick.delay(50);

        led.set();
        SysTick.delay(950);

        try usart.writer.print("\rUptime: {}s", .{SysTick.milliseconds() / 1000});
    }
}
