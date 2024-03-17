#include <stm32h7xx.h>
#include <stdint.h>
#include <stdio.h>

#include "utils.hpp"

volatile bool init_done = false;

void main_loop()
{
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    printf("Hello, I am H730 working at %ld MHz\n", SystemCoreClock / 1000 / 1000);

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
