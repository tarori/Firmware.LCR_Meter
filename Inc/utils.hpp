#pragma once
#include <stm32h7xx.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "adc.h"

#define FIELD_GET(mask, reg) (((reg) & (mask)) >> (__builtin_ffsll(mask) - 1))
#define __ALIGN_MASK(x, mask) ((x) & ~(mask))
#define ALIGN(x, a) __ALIGN_MASK(x, (typeof(x))(a)-1)
#define PI (3.1415926535897932384626433832795028841971)

static inline void delay_us(uint32_t us)
{
    uint32_t old_val = SysTick->VAL;
    uint32_t end_val = us * (SystemCoreClock / 1000 / 1000);
    uint32_t elapsed = 0;
    while (elapsed < end_val) {
        uint32_t new_val = SysTick->VAL;
        if (new_val < old_val) {
            elapsed += old_val - new_val;
        } else {
            elapsed += old_val + SysTick->LOAD + 1 - new_val;
        }
        old_val = new_val;
    }
}

static inline void delay_ms(uint32_t ms)
{
    delay_us(1000 * ms);
}

static inline uint32_t dma_get_last_index(ADC_HandleTypeDef* hadc, uint32_t buf_size)
{
    return (2 * buf_size - 1 - __HAL_DMA_GET_COUNTER(hadc->DMA_Handle)) % buf_size;
}

static inline float my_fast_sin(double x)
{
    float in = fmod(x, 2 * PI);

    return sinf(in);
}

static inline float my_fast_cos(double x)
{
    float in = fmod(x, 2 * PI);

    return cosf(in);
}

class ScopedLock
{
public:
    ScopedLock()
    {
        __disable_irq();
        if (lock_ref_count++) {
            printf("WARN: interrupt lock is already held\n");
        }
    }

    ~ScopedLock()
    {
        if (!(--lock_ref_count)) {
            __enable_irq();
        }
    }

private:
    static volatile uint32_t lock_ref_count;
};

inline volatile uint32_t ScopedLock::lock_ref_count = 0;
