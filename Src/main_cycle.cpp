#include <stm32h7xx.h>
#include <stdint.h>
#include <stdio.h>

#include "utils.hpp"
#include "lcd.hpp"

volatile bool init_done = false;

SMR12864 lcd;

void main_loop()
{
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    printf("Hello, I am H730 working at %ld MHz\n", SystemCoreClock / 1000 / 1000);

    lcd.reset();
    lcd.locate(0, 0);
    lcd.printf("Hello %ld MHz", SystemCoreClock / 1000 / 1000);
    lcd.locate(1, 0);
    lcd.set_fontsize(16);
    lcd.printf("Hello\n");
    lcd.locate(3, 0);
    lcd.set_fontsize(32);
    lcd.printf("Hello\n");

    init_done = true;
    while (1) {
    }
}

void callback_1ms()
{
    if (!init_done) {
        return;
    }
}

extern "C" {
/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
    callback_1ms();
    HAL_IncTick();
}
}
