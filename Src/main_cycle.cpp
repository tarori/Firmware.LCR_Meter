#include <stm32h7xx.h>
#include "adc.h"
#include "dac.h"
#include "tim.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <cstring>
#include <math.h>
#include <algorithm>

#include "utils.hpp"
#include "lcd.hpp"

volatile bool init_done = false;

// DAC
constexpr uint32_t dac_dma_buf_len = 4000;
uint16_t dac_dma_buffer[dac_dma_buf_len];
uint32_t dac_sampling_freq = 1000000;

// ADC
constexpr uint32_t adc_dma_buf_len = 256;
uint16_t adc1_dma_buffer[adc_dma_buf_len];
uint16_t adc2_dma_buffer[adc_dma_buf_len];
constexpr uint32_t adc_data_buf_len = 4000;
uint16_t adc1_data_buffer[adc_data_buf_len];
uint16_t adc2_data_buffer[adc_data_buf_len];
uint32_t adc_sampling_freq = 1000000;

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

    delay_ms(10);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_dma_buffer, adc_dma_buf_len);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_dma_buffer, adc_dma_buf_len);
    MODIFY_REG(((DMA_Stream_TypeDef*)hadc1.DMA_Handle->Instance)->CR, DMA_IT_TC | DMA_IT_HT, 0);
    MODIFY_REG(((DMA_Stream_TypeDef*)hadc2.DMA_Handle->Instance)->CR, DMA_IT_TC | DMA_IT_HT, 0);

    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_dma_buffer, dac_dma_buf_len, DAC_ALIGN_12B_R);
    MODIFY_REG(((DMA_Stream_TypeDef*)hdac1.DMA_Handle1->Instance)->CR, DMA_IT_TC | DMA_IT_HT, 0);

    HAL_TIM_Base_Start(&htim15);  // 1MHz TIM

    init_done = true;
    while (1) {
        printf("Hi %d, %d\n", adc1_dma_buffer[0], adc2_dma_buffer[0]);
        delay_ms(500);
    }
}


void measure_voltage_current()
{
    uint16_t dma_current_ptr = dma_get_last_index(&hadc1, adc_dma_buf_len);
    uint16_t dma_next_read = dma_current_ptr;
    uint32_t write_ptr = 0;
    while (write_ptr < adc_data_buf_len) {
        while (dma_next_read == dma_current_ptr) {
            dma_current_ptr = dma_get_last_index(&hadc1, adc_dma_buf_len);
        }

        adc1_data_buffer[write_ptr] = adc1_dma_buffer[dma_next_read];
        adc1_data_buffer[write_ptr] = adc2_dma_buffer[dma_next_read];
        dma_next_read = (dma_next_read + 1) % adc_dma_buf_len;
        ++write_ptr;
    }
}

void set_dac_output(int freq, int amp)
{
    for (uint32_t i = 0; i < dac_dma_buf_len; ++i) {
        dac_dma_buffer[i] = 2084 + amp * sinf(2 * PI * i * freq / dac_sampling_freq);
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

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_dac1_ch1;

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc2);
    printf("DMA1_Stream0_IRQHandler\n");
}

/**
 * @brief This function handles DMA1 stream1 global interrupt.
 */
void DMA1_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
    printf("DMA1_Stream1_IRQHandler\n");
}

/**
 * @brief This function handles DMA1 stream2 global interrupt.
 */
void DMA1_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_dac1_ch1);
    printf("DMA1_Stream2_IRQHandler\n");
}
}
