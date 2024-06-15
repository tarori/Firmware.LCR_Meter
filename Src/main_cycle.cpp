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
#include "complex.hpp"

volatile bool init_done = false;
volatile bool button1_pushed = false;
volatile bool button2_pushed = false;

enum LCR_ID_IV {
    LCR_ID_I = 0,
    LCR_ID_V = 1
};

// DAC
constexpr uint32_t dac_dma_buf_len = 10000;
__attribute__((section(".RAM_DMA"))) uint16_t dac_dma_buffer[dac_dma_buf_len];
float dac_sampling_freq = 5.0e+6;

// ADC
constexpr uint32_t adc_dma_buf_len = 256;
__attribute__((section(".RAM_DMA"))) uint16_t adc_dma_buffer[2][adc_dma_buf_len];
constexpr uint32_t adc_data_buf_len = 24000;
__attribute__((section(".RAM_DATA"))) uint16_t adc_data_buffer[2][adc_data_buf_len];
float adc_sampling_freq = 120e+6 / 125;

constexpr int freq_list_length = 13;

struct Settings {
    int freq_list[freq_list_length] = {40, 120, 400, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000};

    float adc_ratio = -1.000f;
    float adc_delay_err = 1.1e-9f;

    float short_resistance = 0.0f;
    float short_inductance = 0.0e-6f;
    float open_resistance = 1.0e+24f;
    float open_capacitance = 0.375e-12f;

    Complex pga_v_gain_table[freq_list_length][4] = {
        // 1, 2.9608, 10.067, 30.2
        {{1.0000, 0.0000}, {2.9617, 0.0000}, {10.0642, 0.0004}, {30.1768, 0.0044}},
        {{1.0000, 0.0000}, {2.9617, -0.0000}, {10.0641, 0.0003}, {30.1768, 0.0021}},
        {{1.0000, 0.0000}, {2.9617, -0.0000}, {10.0646, -0.0002}, {30.1800, -0.0009}},
        {{1.0000, 0.0000}, {2.9616, -0.0000}, {10.0643, -0.0008}, {30.1784, -0.0081}},
        {{1.0000, 0.0000}, {2.9616, -0.0001}, {10.0646, -0.0021}, {30.1813, -0.0181}},
        {{1.0000, 0.0000}, {2.9616, -0.0001}, {10.0646, -0.0056}, {30.1825, -0.0487}},
        {{1.0000, 0.0000}, {2.9616, -0.0002}, {10.0646, -0.0116}, {30.1818, -0.0990}},
        {{1.0000, 0.0000}, {2.9615, -0.0005}, {10.0652, -0.0232}, {30.1839, -0.2016}},
        {{1.0000, 0.0000}, {2.9614, -0.0014}, {10.0638, -0.0597}, {30.1721, -0.5162}},
        {{1.0000, 0.0000}, {2.9613, -0.0024}, {10.0616, -0.1186}, {30.1364, -1.0180}},
        {{1.0000, 0.0000}, {2.9615, -0.0047}, {10.0593, -0.2369}, {30.0637, -2.0504}},
        {{1.0000, 0.0000}, {2.9618, -0.0114}, {10.0253, -0.5872}, {29.3079, -4.9519}},
        {{1.0000, 0.0000}, {2.9631, -0.0241}, {9.9172, -1.1581}, {27.0945, -9.0999}}};

    Complex pga_i_gain_table[freq_list_length][4] = {
        // 1, 2.9608, 10.067, 30.2
        {{1.0000, 0.0000}, {2.9604, 0.0000}, {10.0630, 0.0003}, {30.1343, 0.0038}},
        {{1.0000, 0.0000}, {2.9604, 0.0000}, {10.0629, 0.0001}, {30.1364, 0.0026}},
        {{1.0000, 0.0000}, {2.9604, -0.0000}, {10.0632, -0.0003}, {30.1385, -0.0026}},
        {{1.0000, 0.0000}, {2.9604, -0.0000}, {10.0631, -0.0009}, {30.1395, -0.0092}},
        {{1.0000, 0.0000}, {2.9604, -0.0000}, {10.0630, -0.0022}, {30.1396, -0.0188}},
        {{1.0000, 0.0000}, {2.9604, -0.0001}, {10.0632, -0.0057}, {30.1402, -0.0494}},
        {{1.0000, 0.0000}, {2.9604, -0.0002}, {10.0632, -0.0111}, {30.1388, -0.0987}},
        {{1.0000, 0.0000}, {2.9604, -0.0004}, {10.0630, -0.0227}, {30.1330, -0.1966}},
        {{1.0000, 0.0000}, {2.9604, -0.0011}, {10.0633, -0.0580}, {30.1385, -0.4958}},
        {{1.0000, 0.0000}, {2.9606, -0.0024}, {10.0624, -0.1164}, {30.1122, -0.9963}},
        {{1.0000, 0.0000}, {2.9608, -0.0045}, {10.0567, -0.2312}, {29.9995, -1.9847}},
        {{1.0000, 0.0000}, {2.9618, -0.0125}, {10.0291, -0.5822}, {29.3432, -4.8829}},
        {{1.0000, 0.0000}, {2.9653, -0.0257}, {9.9333, -1.1577}, {27.1517, -8.9789}}};

    float tia_res_table[4] = {20, 100, 1000, 20000};
    float tia_cap_table[4] = {0e-12, 0e-12, 0e-12, 2e-12};

    uint32_t adc1_linearity_cal[ADC_LINEAR_CALIB_REG_COUNT] = {0x20080601, 0x2017fe01, 0x2007ea04, 0x2027f5fe, 0x1fe801fe, 0x0209};
    uint32_t adc2_linearity_cal[ADC_LINEAR_CALIB_REG_COUNT] = {0x1fe7fdff, 0x20180dff, 0x20180603, 0x20280e00, 0x1ff7f9ff, 0x0200};
} settings;

SMR12864 lcd;

float set_dac_output(int freq, float v_rms);
void measure_voltage_current();
void measure_short_voltage_current();
bool adc_is_clipping(LCR_ID_IV id, bool strict);
void pga_calibration();
void adc_calibration();
void pga_set_gain(LCR_ID_IV id, int gain_id);
void tia_set_gain(int gain_id);
void coupling_set_dc(bool cur, bool pot);
struct Complex calc_fourier(LCR_ID_IV id, int freq);
void set_dac_bw(int freq);
void adc_calibration_dump();

void main_loop()
{
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    printf("Hello, I am H730 working at %ld MHz\n", SystemCoreClock / 1000 / 1000);

    tia_set_gain(0);
    coupling_set_dc(true, true);
    delay_ms(100);

    // adc_calibration_dump();

    lcd.reset();
    delay_ms(200);
    lcd.locate(0, 0);
    lcd.printf("Hello, I'm working\nat %ld MHz", SystemCoreClock / 1000 / 1000);
    lcd.locate(2, 0);
    lcd.set_fontsize(16);
    lcd.printf("Hoge");
    lcd.locate(4, 0);
    lcd.set_fontsize(32);
    lcd.printf("Piyo");

    delay_ms(10);
    int hal_state = HAL_OK;
    hal_state |= HAL_ADCEx_LinearCalibration_SetValue(&hadc1, settings.adc1_linearity_cal);
    hal_state |= HAL_ADCEx_LinearCalibration_SetValue(&hadc2, settings.adc2_linearity_cal);
    hal_state |= HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    hal_state |= HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    hal_state |= HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    hal_state |= HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer[LCR_ID_I], adc_dma_buf_len);
    hal_state |= HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_dma_buffer[LCR_ID_V], adc_dma_buf_len);
    MODIFY_REG(((DMA_Stream_TypeDef*)hadc1.DMA_Handle->Instance)->CR, DMA_IT_TC | DMA_IT_HT, 0);
    MODIFY_REG(((DMA_Stream_TypeDef*)hadc2.DMA_Handle->Instance)->CR, DMA_IT_TC | DMA_IT_HT, 0);

    if (hal_state != HAL_OK) {
        printf("ADC initialize error\n");
        lcd.cls();
        lcd.printf("ADC initialize error");
        while (1) {
        }
    }

    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_dma_buffer, dac_dma_buf_len, DAC_ALIGN_12B_R);
    MODIFY_REG(((DMA_Stream_TypeDef*)hdac1.DMA_Handle1->Instance)->CR, DMA_IT_TC | DMA_IT_HT, 0);

    delay_ms(10);

    HAL_TIM_Base_Start(&htim15);                     // TIM for ADC
    HAL_TIM_Base_Start(&htim7);                      // TIM for DAC
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  // Encoder 1
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // Encoder 2
    TIM3->CNT = INT16_MAX;
    TIM4->CNT = INT16_MAX;

    int freq = 0;
    int freq_id = 9;  // 100kHz Default
    bool dac_changed = true;
    TIM4->CNT -= 4 * ((TIM4->CNT / 4 - freq_id) % freq_list_length);
    float v_rms = 0.5f;
    bool dc_couple = true;

    init_done = true;
    while (1) {
        int freq_id_new = (TIM4->CNT / 4) % freq_list_length;
        if (freq_id != freq_id_new) {
            dac_changed = true;
            freq_id = freq_id_new;
        }

        if (button2_pushed) {
            button2_pushed = false;
            dc_couple = !dc_couple;
            coupling_set_dc(dc_couple, dc_couple);
            dac_changed = true;
        }

        if (dac_changed) {
            freq = settings.freq_list[freq_id];

            v_rms = set_dac_output(freq, v_rms);
            if (freq < 10 * 1000) {
                // delay_ms(100);
            }
            // delay_ms(100);
        }

        while (0) {
            // adc_calibration();
            pga_calibration();
            delay_ms(5000);
        }

        int tia_gain_id = 3;
        int pga_v_gain_id = 0;
        int pga_i_gain_id = 0;
        pga_set_gain(LCR_ID_I, pga_i_gain_id);
        pga_set_gain(LCR_ID_V, pga_v_gain_id);

        while (1) {
            tia_set_gain(tia_gain_id);
            if (freq > 2000) {
                delay_ms(10);
                measure_short_voltage_current();
            } else {
                delay_ms(50);
                measure_voltage_current();
            }

            if (adc_is_clipping(LCR_ID_I, freq > 200000) && tia_gain_id > 0) {
                --tia_gain_id;
            } else {
                break;
            }
        }

        pga_v_gain_id = 3;
        pga_i_gain_id = 3;
        while (1) {
            pga_set_gain(LCR_ID_I, pga_i_gain_id);
            pga_set_gain(LCR_ID_V, pga_v_gain_id);

            if (freq > 2000) {
                delay_ms(1);
                measure_short_voltage_current();
            } else {
                delay_ms(10);
                measure_voltage_current();
            }

            if (adc_is_clipping(LCR_ID_V, false) && pga_v_gain_id > 0) {
                pga_v_gain_id--;
                continue;
            } else if (adc_is_clipping(LCR_ID_I, false) && pga_i_gain_id > 0) {
                pga_i_gain_id--;
                continue;
            } else {
                break;
            }
        }

        Complex voltage;
        Complex current;

        int measurement_cycle = 1;
        Complex impedance_list[measurement_cycle];

        for (int i = 0; i < measurement_cycle; ++i) {
            measure_voltage_current();
            voltage = calc_fourier(LCR_ID_V, freq);
            current = calc_fourier(LCR_ID_I, freq);

            Complex tia_conductance = Complex{1 / settings.tia_res_table[tia_gain_id], 2 * (float)PI * freq * settings.tia_cap_table[tia_gain_id]};

            current = current * tia_conductance;

            voltage = voltage / settings.pga_v_gain_table[freq_id][pga_v_gain_id];
            current = current / settings.pga_i_gain_table[freq_id][pga_i_gain_id];

            impedance_list[i] = voltage / current;
        }

        Complex impedance = mid(impedance_list, measurement_cycle);

        printf("V: %.4f, I: %4f, Z: %4f, TIA: %d, PGA: %d, %d\n", voltage.abs, current.abs, impedance.abs, tia_gain_id, pga_v_gain_id, pga_i_gain_id);

        float omega = 2 * PI * freq;

        float open_capacitance = settings.open_capacitance;
        impedance = impedance - Complex(settings.short_resistance, omega * settings.short_inductance);
        Complex conductance = Complex(1.0f) / impedance;
        conductance = conductance - Complex(1.0f / settings.open_resistance, omega * open_capacitance);
        impedance = Complex(1.0f) / conductance;

        bool sp_mode = true;  // series
        if (impedance.abs > 1.2e+3) {
            sp_mode = false;  // parallel
        }

        float resistance = sp_mode ? impedance.real : 1.0f / conductance.real;
        float inductance = sp_mode ? impedance.im / omega : -1.0f / conductance.im / omega;
        float capacitance = sp_mode ? -1.0f / (impedance.im * omega) : conductance.im / omega;

        if (abs(capacitance) > 1.0e-8) {
            printf("R: %.4fOhm, L: %.4fuH, C: %.4fuF, Z: %.4fOhm\n", resistance, inductance * 1.0e+6, capacitance * 1.0e+6, impedance.abs);
        } else {
            printf("R: %.4fOhm, L: %.4fuH, C: %.4fpF, Z: %.4fOhm\n", resistance, inductance * 1.0e+6, capacitance * 1.0e+12, impedance.abs);
        }

        lcd.cls();
        lcd.printf("%5d%s %3.1fVrms %s", freq < 1000 ? freq : freq / 1000, freq < 1000 ? "Hz" : "kHz", v_rms, dc_couple ? "DC" : "AC");

        lcd.locate(1, 6);
        lcd.printf("%s Z:", sp_mode ? "Serial" : "Parallel");
        if (impedance.abs > 1e+9) {
            lcd.printf(" ---- $");
        } else if (impedance.abs > 1e+6) {
            lcd.printf("%6.2fM$", impedance.abs / 1e+6);
        } else if (impedance.abs > 1e+5) {
            lcd.printf("%6.4fM$", impedance.abs / 1e+6);
        } else if (impedance.abs > 1e+4) {
            lcd.printf("%6.2fk$", impedance.abs / 1e+3);
        } else if (impedance.abs > 1e+2) {
            lcd.printf("%6.1f $", impedance.abs);
        } else if (impedance.abs > 1e+1) {
            lcd.printf("%6.3f $", impedance.abs);
        } else if (impedance.abs > 1) {
            lcd.printf("%6.2f $", impedance.abs);
        } else {
            lcd.printf("%6.1fm$", impedance.abs * 1000);
        }

        lcd.locate(2, 6);
        lcd.printf("TIA:%d  PGA:(%d,%d)", tia_gain_id, pga_v_gain_id, pga_i_gain_id);

        lcd.locate(3, 6);
        lcd.set_fontsize(16);

        if (capacitance > 0) {
            lcd.printf("%s", sp_mode ? "Cs" : "Cp");
            if (capacitance > 1.0e-1) {
                lcd.printf(" ---- uF");
            } else if (capacitance > 1.0e-3) {
                lcd.printf("%7.0fuF", capacitance * 1.0e+6);
            } else if (capacitance > 1.0e-4) {
                lcd.printf("%6.1fuF", capacitance * 1.0e+6);
            } else if (capacitance > 1.0e-6) {
                lcd.printf("%6.2fuF", capacitance * 1.0e+6);
            } else if (capacitance > 1.0e-7) {
                lcd.printf("%6.4fuF", capacitance * 1.0e+6);
            } else if (capacitance > 1.0e-8) {
                lcd.printf("%6.2fnF", capacitance * 1.0e+9);
            } else if (capacitance > 1.0e-9) {
                lcd.printf("%6.2fpF", capacitance * 1.0e+12);
            } else if (capacitance > 1.0e-11) {
                lcd.printf("%6.2fpF", capacitance * 1.0e+12);
            } else if (capacitance > 1.0e-12 || freq < 100000) {
                lcd.printf("%6.3fpF", capacitance * 1.0e+12);
            } else {
                lcd.printf("%6.1ffF", capacitance * 1.0e+15);
            }
        }

        if (inductance > 0) {
            lcd.printf("%s", sp_mode ? "Ls" : "Lp");
            if (inductance > 1.0e-1) {
                lcd.printf(" ---- mH");
            } else if (inductance > 1.0e-3) {
                lcd.printf("%6.3fmH", inductance * 1.0e+3);
            } else if (inductance > 1.0e-3) {
                lcd.printf("%6.1fuH", inductance * 1.0e+6);
            } else if (inductance > 1.0e-4) {
                lcd.printf("%6.2fuH", inductance * 1.0e+6);
            } else if (inductance > 1.0e-6 || freq < 100000) {
                lcd.printf("%6.3fuH", inductance * 1.0e+6);
            } else {
                lcd.printf("%6.1fnH", inductance * 1.0e+9);
            }
        }

        lcd.locate(5, 6);
        lcd.printf("%s", sp_mode ? "Rs" : "Rp");
        if (resistance < -0.1 || resistance > 1e+7) {
            lcd.printf(" ---- $");
        } else if (resistance > 1e+6) {
            lcd.printf("%5.3fM$", resistance / 1e+6);
        } else if (resistance > 1e+5) {
            lcd.printf("%5.4fM$", resistance / 1e+6);
        } else if (resistance > 1e+4) {
            lcd.printf("%5.2fk$", resistance / 1e+3);
        } else if (resistance > 1e+3) {
            lcd.printf("%5.1f $", resistance);
        } else if (resistance > 1e+1) {
            lcd.printf("%5.2f $", resistance);
        } else if (resistance > 1) {
            lcd.printf("%5.4f $", resistance);
        } else {
            lcd.printf("%5.1fm$", resistance * 1000);
        }

        if (1) {
            lcd.locate(7, 12);
            lcd.set_fontsize(8);
            lcd.printf("Q = ");
            float q = abs(impedance.im / impedance.real);
            if (q > 1e+2) {
                lcd.printf("%.1f", q);
            } else if (q > 1e+1) {
                lcd.printf("%.2f", q);
            } else {
                lcd.printf("%5.3f", q);
            }

            float theta_deg = 360.0f / (2 * PI) * atan2(abs(impedance.im), abs(impedance.real));
            lcd.printf(" %5.2fdeg", theta_deg);
        }

        if (button1_pushed) {
            settings.open_capacitance = settings.open_capacitance + capacitance;
            button1_pushed = false;
        }
    }
}

void measure_voltage_current()
{
    ScopedLock lock;
    uint16_t dma_current_ptr = dma_get_last_index(&hadc1, adc_dma_buf_len);
    uint16_t dma_next_read = dma_current_ptr;
    uint32_t write_ptr = 0;
    while (write_ptr < adc_data_buf_len) {
        while (dma_next_read == dma_current_ptr) {
            dma_current_ptr = dma_get_last_index(&hadc1, adc_dma_buf_len);
        }

        adc_data_buffer[LCR_ID_I][write_ptr] = adc_dma_buffer[LCR_ID_I][dma_next_read];
        adc_data_buffer[LCR_ID_V][write_ptr] = adc_dma_buffer[LCR_ID_V][dma_next_read];
        dma_next_read = (dma_next_read + 1) % adc_dma_buf_len;
        ++write_ptr;
    }
}

void measure_short_voltage_current()
{
    ScopedLock lock;
    uint16_t dma_current_ptr = dma_get_last_index(&hadc1, adc_dma_buf_len);
    uint16_t dma_next_read = dma_current_ptr;
    uint32_t write_ptr = 0;
    while (write_ptr < adc_data_buf_len / 10) {
        while (dma_next_read == dma_current_ptr) {
            dma_current_ptr = dma_get_last_index(&hadc1, adc_dma_buf_len);
        }

        adc_data_buffer[LCR_ID_I][write_ptr] = adc_dma_buffer[LCR_ID_I][dma_next_read];
        adc_data_buffer[LCR_ID_V][write_ptr] = adc_dma_buffer[LCR_ID_V][dma_next_read];
        dma_next_read = (dma_next_read + 1) % adc_dma_buf_len;
        ++write_ptr;
    }

    while (write_ptr < adc_data_buf_len) {
        adc_data_buffer[LCR_ID_I][write_ptr] = UINT16_MAX / 2;
        adc_data_buffer[LCR_ID_V][write_ptr] = UINT16_MAX / 2;
        ++write_ptr;
    }
}

void adc_calibration_dump()
{
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(hadc1.Instance), ADC_CLOCK_ASYNC_DIV4);
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(hadc2.Instance), ADC_CLOCK_ASYNC_DIV4);
    delay_ms(1000);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);
    uint32_t cal_data[ADC_LINEAR_CALIB_REG_COUNT];
    HAL_ADCEx_LinearCalibration_GetValue(&hadc1, cal_data);
    printf("ADC1: {0x%04lx, 0x%04lx, 0x%04lx, 0x%04lx, 0x%04lx, 0x%04lx}\n",
        cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4], cal_data[5]);
    HAL_ADCEx_LinearCalibration_GetValue(&hadc2, cal_data);
    printf("ADC2: {0x%04lx, 0x%04lx, 0x%04lx, 0x%04lx, 0x%04lx, 0x%04lx}\n",
        cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4], cal_data[5]);
    while (1) {
    }
}

struct Complex
calc_fourier(LCR_ID_IV id, int freq)
{
    double real_sum = 0;
    float ratio = (id == LCR_ID_V) ? 1 : settings.adc_ratio;
    float delay = (id == LCR_ID_V) ? 0 : settings.adc_delay_err;
    for (uint32_t i = 0; i < adc_data_buf_len; ++i) {
        float cos_val = my_fast_cos(2 * PI * freq * (delay + i / (double)adc_sampling_freq));
        real_sum += adc_data_buffer[id][i] * cos_val / adc_data_buf_len;
    }

    double im_sum = 0;
    for (uint32_t i = 0; i < adc_data_buf_len; ++i) {
        float sin_val = -my_fast_sin(2 * PI * freq * (delay + i / (double)adc_sampling_freq));
        im_sum += adc_data_buffer[id][i] * sin_val / adc_data_buf_len;
    }

    return Complex(ratio * real_sum, ratio * im_sum);
}

float set_dac_output(int freq, float v_rms)
{
    float k = sqrt(1.0f + 6e-12 * freq * freq);
    v_rms *= k;
    if (v_rms > 1.4f) {
        v_rms = 1.4f;
    }
    for (uint32_t i = 0; i < dac_dma_buf_len; ++i) {
        float dither = (rand() % 4) - 1.5f;
        dac_dma_buffer[i] = 2250 + 1285.0f * v_rms * my_fast_sin(2 * PI * i * freq / (double)dac_sampling_freq) + dither;
    }

    set_dac_bw(freq);

    if (freq < 1000) {
        TIM7->ARR = 600 - 1;
        dac_sampling_freq = 400e+3;
    } else {
        TIM7->ARR = 48 - 1;
        dac_sampling_freq = 5e+6;
    }

    return v_rms / k;
}

bool adc_is_clipping(LCR_ID_IV id, bool strict)
{
    uint16_t min_val = *std::min_element(adc_data_buffer[id], adc_data_buffer[id] + adc_data_buf_len);
    uint16_t max_val = *std::max_element(adc_data_buffer[id], adc_data_buffer[id] + adc_data_buf_len);
    if (strict) {
        return min_val < UINT16_MAX * 0.3f || max_val > UINT16_MAX * 0.7f;
    } else {
        return min_val < UINT16_MAX * 0.2f || max_val > UINT16_MAX * 0.8f;
    }
}

void set_iv_mux_sw(bool sw1, bool sw2)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, sw1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, sw2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


void coupling_set_dc(bool cur, bool pot)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, cur ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, pot ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


void adc_calibration()
{
    tia_set_gain(0);
    int freq_id = 9;
    int freq = settings.freq_list[freq_id];
    set_dac_output(freq, 1.0f);
    tia_set_gain(0);
    pga_set_gain(LCR_ID_V, 0);
    pga_set_gain(LCR_ID_I, 0);
    set_iv_mux_sw(true, false);
    delay_ms(100);

    measure_voltage_current();
    if (adc_is_clipping(LCR_ID_I, false) || adc_is_clipping(LCR_ID_V, false)) {
        printf("Warn: ADC is clipping\n");
    }
    Complex voltage = calc_fourier(LCR_ID_V, freq);
    Complex current = calc_fourier(LCR_ID_I, freq);
    Complex ratio = current / voltage;
    float delay_s = -atan2(ratio.im, ratio.real) / 2 / PI / freq;
    printf("ADC Cal Ratio: %f, Delay %fns, Complex: %f+%fi\n", ratio.abs, delay_s * 1.0e+9, ratio.real, ratio.im);
}

void pga_calibration()
{
    tia_set_gain(0);
    for (int freq_id = 0; freq_id < freq_list_length; ++freq_id) {
        int freq = settings.freq_list[freq_id];
        int pga_v_gain_id = 0;
        int pga_i_gain_id = 0;
        printf("{");
        while (1) {
            float v_rms = std::min(1.0f / settings.pga_v_gain_table[freq_id][pga_v_gain_id].abs, 1.0f / settings.pga_i_gain_table[freq_id][pga_i_gain_id].abs);
            set_dac_output(freq, v_rms);
            pga_set_gain(LCR_ID_V, pga_v_gain_id);
            pga_set_gain(LCR_ID_I, pga_i_gain_id);
            set_iv_mux_sw(true, false);
            delay_ms(100);

            int measurement_cycle = 16;
            Complex ratio_list[measurement_cycle];

            for (int i = 0; i < measurement_cycle; ++i) {
                measure_voltage_current();
                if (adc_is_clipping(LCR_ID_I, false)
                    || adc_is_clipping(LCR_ID_V, false)) {
                    printf("Warn: ADC is clipping\n");
                }

                Complex voltage = calc_fourier(LCR_ID_V, freq);
                Complex current = calc_fourier(LCR_ID_I, freq);
                // ratio_list[i] = voltage / current;
                ratio_list[i] = current / voltage;
            }

            Complex ratio = mid(ratio_list, measurement_cycle);

            // printf("%dkHz, %d/%d, Ratio: %.5f + %.5fi = |%.6f|\n", freq / 1000, pga_v_gain_id, pga_i_gain_id, ratio.real, ratio.im, ratio.abs);
            printf("{%.4f,%.4f}", ratio.real, ratio.im);
            pga_i_gain_id++;
            if (pga_v_gain_id >= 4 || pga_i_gain_id >= 4) {
                break;
            } else {
                printf(",");
            }
        }
        printf("},\n");
    }
    printf("PGA Calibration Done!\n");
}

void pga_set_gain(LCR_ID_IV id, int gain_id)
{
    if (gain_id >= 4) {
        printf("PGA Gain Error\n");
        return;
    }
    if (id == LCR_ID_I) {
        switch (gain_id) {
        case 0:
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
            break;
        case 2:

            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
            break;
        }
    } else {
        switch (gain_id) {
        case 0:
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
            break;
        }
    }
}

void tia_set_gain(int gain_id)
{
    if (gain_id >= 4) {
        printf("PGA Gain Error\n");
        return;
    }
    switch (gain_id) {
    case 0:
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
        break;
    case 1:
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
        break;
    case 2:
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
        break;
    case 3:
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
        break;
    }
}


void set_dac_bw(int freq)
{
    int bw_id;
    if (freq > 500000) {
        bw_id = 3;
    } else if (freq > 100000) {
        bw_id = 2;
    } else if (freq > 20000) {
        bw_id = 1;
    } else {
        bw_id = 0;
    }

    switch (bw_id) {
    case 0:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
        break;
    case 1:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
        break;
    case 2:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
        break;
    case 3:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
        break;
    default:
        printf("DAC BW Error\n");
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
 * @brief This function handles EXTI line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    button1_pushed = true;
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    button2_pushed = true;
}

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
