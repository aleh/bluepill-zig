// V2: a self-contained LED blinking example with SysTick timer and USART for STM32F103-based Maple mini (aka "Blue Pill") boards.

const z41 = @import("z41");

export fn _start() noreturn {
    const rcc = z41.RCC(.internalRC);
    rcc.init();

    const sysTick = z41.SysTick(rcc.SYSCLK, 50);
    sysTick.init();

    const led = z41.GPIO(rcc, .C).port(13);
    led.Bank.init();
    led.setOutput(.openDrain, .max2MHz);

    const usart = z41.USART(rcc, .usart1);
    usart.init(115200);

    while (true) {
        led.reset();
        sysTick.delay(50);
        led.set();
        sysTick.delay(950);
        usart.write_string("Hello\n");
    }
}
