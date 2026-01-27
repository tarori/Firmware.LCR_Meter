#pragma once
#include <stm32h7xx.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "adc.h"
#include "math_android.hpp"

#define ARM_MATH_CM7
#include "arm_math.h"

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

#define FIELD_GET(mask, reg) (((reg) & (mask)) >> (__builtin_ffsll(mask) - 1))
#define __ALIGN_MASK(x, mask) ((x) & ~(mask))
#define ALIGN(x, a) __ALIGN_MASK(x, (typeof(x))(a) - 1)

static inline void delay_us(uint64_t us)
{
    uint32_t old_val = SysTick->VAL;
    uint64_t end_val = us * (SystemCoreClock / 1000 / 1000);
    uint64_t elapsed = 0;
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

static inline double my_fmod(double x, double y)
{
    int32_t k = (float)x / (float)y;
    return x - k * y;
}

static inline float my_fast_sin(double x)
{
    float in = my_fmod(x, 2 * M_PI);
    return sinf(in);
}

static inline float my_fast_cos(double x)
{
    float in = my_fmod(x, 2 * M_PI);
    return cosf(in);
}

static inline void my_fast_sincos(double x, float* sin_out, float* cos_out)
{
    float in = my_fmod(x, 2 * M_PI);
    sincosf(in, sin_out, cos_out);
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

inline uint32_t delaymeas_start_cnt;

static inline void delaymeas_start()
{
    delaymeas_start_cnt = TIM23->CNT;
}

static inline void delaymeas_end()
{
    uint32_t end_cnt = TIM23->CNT;
    double elapsed_ms = (uint32_t)(end_cnt - delaymeas_start_cnt) * 1000.0 / SystemCoreClock * 2;
    printf("Elapsed: %.5f ms\n", elapsed_ms);
}
