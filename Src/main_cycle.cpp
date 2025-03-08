#include <stm32h7xx.h>
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "rng.h"

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
volatile bool button3_pushed = true;

enum LCR_ID_IV {
    LCR_ID_I = 0,
    LCR_ID_V = 1
};

// DAC
constexpr uint32_t dac_dma_buf_len = 5000;
__attribute__((section(".RAM_DMA"))) uint16_t dac_dma_buffer[2][dac_dma_buf_len];
double dac_sampling_freq = 5.0e+6;

// ADC
constexpr uint32_t adc_dma_buf_len = 256;
__attribute__((section(".RAM_DMA"))) uint16_t adc_dma_buffer[2][adc_dma_buf_len];
constexpr uint32_t adc_data_buf_len = 24000;
__attribute__((section(".RAM_DATA"))) uint16_t adc_data_buffer[2][adc_data_buf_len];
double adc_sampling_freq = 120e+6 / 125;

bool read_button3()
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET;
}

constexpr int freq_list_length = 13;
constexpr int pga_list_length = 4;
constexpr int tia_list_length = 4;

struct Settings {
    int freq_list[freq_list_length] = {40, 120, 400, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000};

    double adc_ratio = -1.000;
    double adc_delay_err = 1.1e-9;

    double short_resistance = 0.0;
    double short_inductance = 0.0e-6;
    double open_resistance = 1.0e+24;
    double open_capacitance = 0.096e-12;

    int pga_gain_disp[pga_list_length] = {1, 5, 20, 50};

    Complex pga_v_gain_table[freq_list_length][pga_list_length] = {
        // 1, 4.75, 17.944, 53.833
        {{1.0000, 0.0000}, {4.7474, 0.0001}, {17.9108, 0.0033}, {53.7776, 0.0289}},
        {{1.0000, 0.0000}, {4.7475, -0.0001}, {17.9145, 0.0005}, {53.7983, 0.0096}},
        {{1.0000, 0.0000}, {4.7475, -0.0000}, {17.9148, -0.0014}, {53.8045, -0.0064}},
        {{1.0000, 0.0000}, {4.7474, -0.0002}, {17.9132, -0.0035}, {53.7945, -0.0338}},
        {{1.0000, 0.0000}, {4.7475, -0.0005}, {17.9141, -0.0081}, {53.8097, -0.0707}},
        {{1.0000, 0.0000}, {4.7475, -0.0013}, {17.9141, -0.0193}, {53.8065, -0.1767}},
        {{1.0000, 0.0000}, {4.7474, -0.0026}, {17.9134, -0.0393}, {53.8035, -0.3428}},
        {{1.0000, 0.0000}, {4.7475, -0.0054}, {17.9118, -0.0786}, {53.7713, -0.6595}},
        {{1.0000, 0.0000}, {4.7474, -0.0131}, {17.9102, -0.1972}, {53.7157, -1.6623}},
        {{1.0000, 0.0000}, {4.7472, -0.0263}, {17.9063, -0.3903}, {53.6282, -3.2888}},
        {{1.0000, 0.0000}, {4.7474, -0.0527}, {17.8812, -0.7962}, {53.1584, -6.8398}},
        {{0.9999, 0.0000}, {4.7430, -0.1307}, {17.6975, -1.9468}, {49.3114, -15.1200}},
        {{0.9995, 0.0000}, {4.7295, -0.2612}, {17.1298, -3.7096}, {39.0496, -24.3760}}};

    Complex pga_i_gain_table[freq_list_length][pga_list_length] = {
        // 1, 4.75, 17.944, 53.833
        {{1.0000, 0.0000}, {4.7490, 0.0001}, {17.9269, 0.0024}, {53.7743, 0.0381}},
        {{1.0000, 0.0000}, {4.7490, 0.0001}, {17.9280, 0.0009}, {53.7881, 0.0132}},
        {{1.0000, 0.0000}, {4.7491, -0.0001}, {17.9284, -0.0006}, {53.7922, -0.0058}},
        {{1.0000, 0.0000}, {4.7492, -0.0002}, {17.9291, -0.0025}, {53.7930, -0.0192}},
        {{1.0000, 0.0000}, {4.7491, -0.0004}, {17.9293, -0.0067}, {53.7963, -0.0531}},
        {{1.0000, 0.0000}, {4.7491, -0.0012}, {17.9301, -0.0189}, {53.8021, -0.1573}},
        {{1.0000, 0.0000}, {4.7491, -0.0025}, {17.9308, -0.0382}, {53.8092, -0.3155}},
        {{1.0000, 0.0000}, {4.7490, -0.0052}, {17.9308, -0.0767}, {53.8085, -0.6546}},
        {{1.0000, 0.0000}, {4.7491, -0.0132}, {17.9295, -0.1916}, {53.7920, -1.5704}},
        {{1.0000, 0.0000}, {4.7491, -0.0264}, {17.9233, -0.3864}, {53.6334, -3.2511}},
        {{1.0000, 0.0000}, {4.7487, -0.0520}, {17.9023, -0.7639}, {52.8879, -6.4359}},
        {{1.0000, 0.0000}, {4.7453, -0.1311}, {17.7157, -1.8983}, {49.1967, -15.2804}},
        {{1.0000, 0.0000}, {4.7344, -0.2618}, {17.0819, -3.6977}, {39.1677, -23.4002}}};

    double tia_res_table[tia_list_length] = {20, 100, 1000, 20000};
    double tia_cap_table[tia_list_length] = {-50e-12, 11.5e-12, 10.2e-12, 1.4e-12};

    uint32_t adc1_linearity_cal[ADC_LINEAR_CALIB_REG_COUNT] = {0x20080e02, 0x20080600, 0x2007fdfe, 0x200805ff, 0x20181200, 0x01ff};
    uint32_t adc2_linearity_cal[ADC_LINEAR_CALIB_REG_COUNT] = {0x2017fdff, 0x20380200, 0x1ff809fd, 0x1ff80601, 0x201811ff, 0x01fd};
} settings;

SMR12864 lcd;

bool initialize_adc();
bool initialize_dac_tim();
double set_dac_output(int freq, double v_rms);
void measure_voltage_current(bool is_short);
bool adc_is_clipping(LCR_ID_IV id, bool strict);
double read_battery_voltage();
void pga_calibration();
void adc_calibration();
void pga_set_gain(LCR_ID_IV id, int gain_id);
void tia_set_gain(int gain_id);
void coupling_set_dc(bool cur, bool pot);
struct Complex calc_fourier(LCR_ID_IV id, int freq);
void set_dac_bw(int freq);
void adc_calibration_dump();
void adc_data_dump();
void set_backlight(bool state);

void main_loop()
{
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    printf("Hello, I am H730 working at %ld MHz\n", SystemCoreClock / 1000 / 1000);

    tia_set_gain(0);
    coupling_set_dc(true, true);
    delay_ms(100);
    set_backlight(false);

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

    if (initialize_adc()) {
        printf("ADC initialize error\n");
        lcd.cls();
        lcd.printf("ADC initialize error");
        while (1) {
        }
    }

    if (initialize_dac_tim()) {
        printf("DAC initialize error\n");
        lcd.cls();
        lcd.printf("DAC initialize error");
        while (1) {
        }
    }

    int freq = 0;
    int freq_id = 9;  // 100kHz Default
    bool dac_changed = true;
    bool dc_couple = true;
    TIM4->CNT -= 4 * ((TIM4->CNT / 4 - freq_id) % freq_list_length);
    double v_rms = 1.0;
    TIM3->CNT = INT16_MAX + 4 * (v_rms / 0.1);

    init_done = true;
    while (1) {
        /* Signal source setup */
        int freq_id_new = (TIM4->CNT / 4) % freq_list_length;
        if (freq_id != freq_id_new) {
            dac_changed = true;
            freq_id = freq_id_new;
        }

        v_rms = (((int32_t)TIM3->CNT - INT16_MAX) / 4) * 0.1;
        if (v_rms <= 0) {
            v_rms = 0.05;
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
        }

        double battery_voltage = read_battery_voltage();

        while (0) {
            // adc_calibration();
            pga_calibration();
            // delay_ms(5000);
        }

        /* Auto ranging */
        int tia_gain_id = tia_list_length - 1;
        int pga_v_gain_id = 0;
        int pga_i_gain_id = 0;
        pga_set_gain(LCR_ID_I, pga_i_gain_id);
        pga_set_gain(LCR_ID_V, pga_v_gain_id);

        while (1) {
            tia_set_gain(tia_gain_id);
            if (freq > 2000) {
                delay_ms(10);
                measure_voltage_current(true);
            } else {
                delay_ms(50);
                measure_voltage_current(false);
            }

            if (adc_is_clipping(LCR_ID_I, freq > 200000) && tia_gain_id > 0) {
                --tia_gain_id;
            } else {
                break;
            }
        }

        pga_v_gain_id = pga_list_length - 1;
        pga_i_gain_id = pga_list_length - 1;
        while (1) {
            pga_set_gain(LCR_ID_I, pga_i_gain_id);
            pga_set_gain(LCR_ID_V, pga_v_gain_id);

            if (freq > 2000) {
                delay_ms(1);
                measure_voltage_current(true);
            } else {
                delay_ms(10);
                measure_voltage_current(false);
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

        // adc_data_dump();

        /* Real measurement */
        measure_voltage_current(false);
        Complex voltage = calc_fourier(LCR_ID_V, freq);
        Complex current = calc_fourier(LCR_ID_I, freq);

        Complex tia_conductance = Complex{1 / settings.tia_res_table[tia_gain_id], 2 * (double)PI * freq * settings.tia_cap_table[tia_gain_id]};

        voltage = voltage / settings.pga_v_gain_table[freq_id][pga_v_gain_id];
        current = (current * tia_conductance) / settings.pga_i_gain_table[freq_id][pga_i_gain_id];
        Complex impedance = voltage / current;

        /* Probe compensation */
        double omega = 2 * M_PI * freq;
        if (button3_pushed) {
            button3_pushed = false;
            if (impedance.abs < 10.0) {
                // Short Compensation
                double inductance = impedance.im / omega;
                double resistance = impedance.real;
                settings.short_inductance = inductance;
                settings.short_resistance = resistance;
            } else if (impedance.abs > 10000.0) {
                // Open Compensation
                Complex conductance = Complex(1.0) / impedance;
                double capacitance = conductance.im / omega;
                double resistance = 1.0 / conductance.real;
                settings.open_capacitance = capacitance;
                settings.open_resistance = resistance;
            } else {
                // Error
            }
        }

        /* Calculation*/
        impedance = impedance - Complex(settings.short_resistance, omega * settings.short_inductance);
        Complex conductance = Complex(1.0) / impedance;
        conductance = conductance - Complex(1.0 / settings.open_resistance, omega * settings.open_capacitance);
        impedance = Complex(1.0) / conductance;

        bool sp_mode = true;  // series
        if (impedance.abs > 1.2e+3) {
            sp_mode = false;  // parallel
        } else if (impedance.abs > 50 && impedance.real * 1.2 > impedance.abs) {
            sp_mode = false;  // parallel;
        }

        if (freq == 120 && impedance.abs > 1e+1) {
            sp_mode = false;
        }

        double resistance = sp_mode ? impedance.real : 1.0 / conductance.real;
        double inductance = sp_mode ? impedance.im / omega : -1.0 / conductance.im / omega;
        double capacitance = sp_mode ? -1.0 / (impedance.im * omega) : conductance.im / omega;

        if (abs(capacitance) > 1.0e-8) {
            printf("R: %.4fOhm, L: %.4fuH, C: %.4fuF, Z: %.4fOhm, BAT: %.3fV\n", resistance, inductance * 1.0e+6, capacitance * 1.0e+6, impedance.abs, battery_voltage);
        } else {
            printf("R: %.4fOhm, L: %.4fuH, C: %.4fpF, Z: %.4fOhm, BAT: %.3fV\n", resistance, inductance * 1.0e+6, capacitance * 1.0e+12, impedance.abs, battery_voltage);
        }

        lcd.cls();
        lcd.printf("%5d%s %4.2fVrms %s", freq < 1000 ? freq : freq / 1000, freq < 1000 ? "Hz" : "kHz", v_rms, dc_couple ? "DC" : "AC");

        lcd.locate(1, 6);
        lcd.printf("B:%3.1fV  Z:", battery_voltage);
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
        lcd.printf("TIA:%d PGA:[x%d,x%d]", tia_gain_id, settings.pga_gain_disp[pga_i_gain_id], settings.pga_gain_disp[pga_v_gain_id]);

        lcd.locate(3, 6);
        lcd.set_fontsize(16);

        if (capacitance > -10e-15 && capacitance < 1.0e-1) {
            lcd.printf("%s", sp_mode ? "Cs" : "Cp");
            if (capacitance > 1.0e-3) {
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

        if (inductance > 0 && inductance < 1.0e-1) {
            lcd.printf("%s", sp_mode ? "Ls" : "Lp");
            if (inductance > 1.0e-3) {
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
        if (resistance < -0.1 || resistance > 1e+9) {
            lcd.printf(" ---- $");
        } else if (resistance > 1e+8) {
            lcd.printf("%6.1fM$", resistance / 1e+6);
        } else if (resistance > 1e+7) {
            lcd.printf("%5.2fM$", resistance / 1e+6);
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
            double q = abs(impedance.im / impedance.real);
            if (q > 1e+2) {
                lcd.printf("%.1f", q);
            } else if (q > 1e+1) {
                lcd.printf("%.2f", q);
            } else {
                lcd.printf("%5.3f", q);
            }

            double theta_deg = 360.0 / (2 * PI) * atan2(abs(impedance.im), abs(impedance.real));
            lcd.printf(" %5.2fdeg", theta_deg);
        }
    }
}

void measure_voltage_current(bool is_short)
{
    ScopedLock lock;
    uint16_t dma_current_ptr = dma_get_last_index(&hadc1, adc_dma_buf_len);
    uint16_t dma_next_read = dma_current_ptr;
    uint32_t write_ptr = 0;
    while (write_ptr < (is_short ? adc_data_buf_len / 10 : adc_data_buf_len)) {
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

bool initialize_adc()
{
    int hal_state = HAL_OK;
    // hal_state |= HAL_ADCEx_LinearCalibration_SetValue(&hadc1, settings.adc1_linearity_cal);
    // hal_state |= HAL_ADCEx_LinearCalibration_SetValue(&hadc2, settings.adc2_linearity_cal);
    hal_state |= HAL_ADCEx_LinearCalibration_FactorLoad(&hadc1);
    hal_state |= HAL_ADCEx_LinearCalibration_FactorLoad(&hadc2);
    // hal_state |= HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);
    // hal_state |= HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);
    hal_state |= HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    hal_state |= HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    hal_state |= HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    /* Initialize DMA */
    hal_state |= HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer[LCR_ID_I], adc_dma_buf_len);
    hal_state |= HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_dma_buffer[LCR_ID_V], adc_dma_buf_len);
    CLEAR_BIT(((DMA_Stream_TypeDef*)hadc1.DMA_Handle->Instance)->CR, DMA_IT_TC | DMA_IT_HT);
    CLEAR_BIT(((DMA_Stream_TypeDef*)hadc2.DMA_Handle->Instance)->CR, DMA_IT_TC | DMA_IT_HT);

    /* Stop DMA and ADC*/
    CLEAR_BIT(((DMA_Stream_TypeDef*)hadc1.DMA_Handle->Instance)->CR, DMA_SxCR_EN);
    CLEAR_BIT(((DMA_Stream_TypeDef*)hadc2.DMA_Handle->Instance)->CR, DMA_SxCR_EN);

    SET_BIT(hadc1.Instance->CR, ADC_CR_ADSTP);
    while (READ_BIT(hadc1.Instance->CR, ADC_CR_ADSTP)) {
    }
    SET_BIT(hadc2.Instance->CR, ADC_CR_ADSTP);
    while (READ_BIT(hadc2.Instance->CR, ADC_CR_ADSTP)) {
    }

    /* Configure ADC synchronization */
    SET_BIT(ADC12_COMMON->CCR, ADC_DUALMODE_REGSIMULT);

    /* Restart DMA and ADC */
    ((DMA_Stream_TypeDef*)hadc1.DMA_Handle->Instance)->NDTR = adc_dma_buf_len;
    ((DMA_Stream_TypeDef*)hadc2.DMA_Handle->Instance)->NDTR = adc_dma_buf_len;
    SET_BIT(((DMA_Stream_TypeDef*)hadc1.DMA_Handle->Instance)->CR, DMA_SxCR_EN);
    SET_BIT(((DMA_Stream_TypeDef*)hadc2.DMA_Handle->Instance)->CR, DMA_SxCR_EN);
    SET_BIT(hadc1.Instance->CR, ADC_CR_ADSTART);

    return hal_state != HAL_OK;
}

bool initialize_dac_tim()
{
    int hal_state = HAL_OK;
    hal_state |= HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_dma_buffer[0], dac_dma_buf_len, DAC_ALIGN_12B_R);
    hal_state |= HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)dac_dma_buffer[1], dac_dma_buf_len, DAC_ALIGN_12B_R);
    CLEAR_BIT(((DMA_Stream_TypeDef*)hdac1.DMA_Handle1->Instance)->CR, DMA_IT_TC | DMA_IT_HT);
    CLEAR_BIT(((DMA_Stream_TypeDef*)hdac1.DMA_Handle2->Instance)->CR, DMA_IT_TC | DMA_IT_HT);

    delay_ms(10);

    hal_state |= HAL_TIM_Base_Start(&htim15);                     // TIM for ADC
    hal_state |= HAL_TIM_Base_Start(&htim7);                      // TIM for DAC
    hal_state |= HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  // Encoder 1
    hal_state |= HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // Encoder 2
    TIM3->CNT = INT16_MAX;
    TIM4->CNT = INT16_MAX;

    return hal_state != HAL_OK;
}

void adc_calibration_dump()
{
    // LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(hadc1.Instance), ADC_CLOCK_ASYNC_DIV4);
    // LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(hadc2.Instance), ADC_CLOCK_ASYNC_DIV4);
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
    double ratio = (id == LCR_ID_V) ? 1 : settings.adc_ratio;
    double delay = (id == LCR_ID_V) ? 0 : settings.adc_delay_err;
    for (uint32_t i = 0; i < adc_data_buf_len; ++i) {
        double cos_val = my_fast_cos(2 * M_PI * freq * (delay + i / (double)adc_sampling_freq));
        real_sum += adc_data_buffer[id][i] * cos_val / adc_data_buf_len;
    }

    double im_sum = 0;
    for (uint32_t i = 0; i < adc_data_buf_len; ++i) {
        double sin_val = -my_fast_sin(2 * M_PI * freq * (delay + i / (double)adc_sampling_freq));
        im_sum += adc_data_buffer[id][i] * sin_val / adc_data_buf_len;
    }

    return Complex(ratio * real_sum, ratio * im_sum);
}

double set_dac_output(int freq, double v_rms)
{
    set_dac_bw(freq);

    if (freq < 1000) {
        TIM7->ARR = 1200 - 1;
        dac_sampling_freq = 200e+3;
    } else {
        TIM7->ARR = 48 - 1;
        dac_sampling_freq = 5e+6;
    }

    double k = sqrt(1.0 + 2.7e-12 * freq * freq);
    v_rms *= k;
    if (v_rms > 1.5) {
        v_rms = 1.5;
    }

    uint32_t rand_seed = 0;
    HAL_RNG_GenerateRandomNumber(&hrng, &rand_seed);
    srand(rand_seed);

    /* Constant dither */
    for (uint32_t i = 0; i < dac_dma_buf_len; ++i) {
        double dither = ((rand() % 16384) - 8192) / 16384.0;
        double dac_code_ideal = 2043.0 + 1173.0 * v_rms * my_fast_sin(2 * M_PI * i * freq / (double)dac_sampling_freq) + dither;
        dac_dma_buffer[0][i] = dac_code_ideal;
        dac_dma_buffer[1][i] = dac_code_ideal + 0.5;
    }

    /* Dither shaping */
    /*
    double dac_error_integ = 0;
    for (int32_t i = -1000; i < (int32_t)dac_dma_buf_len; ++i) {
        double dither = ((rand() % 16384) - 8192) / 8192.0;
        double dac_code_ideal = 2043.5 + 1173.0 * v_rms * my_fast_sin(2 * M_PI * i * freq / (double)dac_sampling_freq);
        double dac_code_float = dac_code_ideal + dither - 0.5 * dac_error_integ;
        uint32_t dac_code = dac_code_float;
        double dac_error = dac_code - dac_code_ideal;
        dac_error_integ += dac_error;
        if (i >= 0) {
            dac_dma_buffer[0][i] = dac_code;
        }
    }
    */

    return v_rms / k;
}

bool adc_is_clipping(LCR_ID_IV id, bool strict)
{
    std::pair<uint16_t*, uint16_t*> minmax = std::minmax_element(adc_data_buffer[id], adc_data_buffer[id] + adc_data_buf_len);
    if (strict) {
        return *minmax.first < UINT16_MAX * 0.3 || *minmax.second > UINT16_MAX * 0.7;
    } else {
        return *minmax.first < UINT16_MAX * 0.2 || *minmax.second > UINT16_MAX * 0.8;
    }
}

void set_iv_mux_sw(bool sw1, bool sw2)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, sw1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, sw2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void set_backlight(bool state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void coupling_set_dc(bool cur, bool pot)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, cur ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, pot ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

double read_battery_voltage()
{
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 100);
    uint16_t adc_val = HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);
    return 3.3 * 2.0 / 4.0 * adc_val / 4096.0;
}


void adc_calibration()
{
    tia_set_gain(0);
    int freq_id = 9;
    int freq = settings.freq_list[freq_id];
    set_dac_output(freq, 1.0);
    tia_set_gain(0);
    pga_set_gain(LCR_ID_V, 0);
    pga_set_gain(LCR_ID_I, 0);
    set_iv_mux_sw(true, false);
    delay_ms(100);

    measure_voltage_current(false);
    if (adc_is_clipping(LCR_ID_I, false) || adc_is_clipping(LCR_ID_V, false)) {
        printf("Warn: ADC is clipping\n");
    }
    Complex voltage = calc_fourier(LCR_ID_V, freq);
    Complex current = calc_fourier(LCR_ID_I, freq);
    Complex ratio = current / voltage;
    double delay_s = -atan2(ratio.im, ratio.real) / (2 * M_PI * freq);
    printf("ADC Cal Ratio: %f, Delay %fns, Complex: %f+%fi\n", ratio.abs, delay_s * 1.0e+9, ratio.real, ratio.im);
}

void pga_calibration()
{
    tia_set_gain(0);
    bool is_target_v = true;
    for (int freq_id = 0; freq_id < freq_list_length; ++freq_id) {
        int freq = settings.freq_list[freq_id];
        int pga_v_gain_id = 0;
        int pga_i_gain_id = 0;
        printf("{");
        while (1) {
            pga_set_gain(LCR_ID_V, pga_v_gain_id);
            pga_set_gain(LCR_ID_I, pga_i_gain_id);
            set_iv_mux_sw(true, false);
            double v_rms = 1.0;
            while (1) {
                set_dac_output(freq, v_rms);
                measure_voltage_current(false);
                if (adc_is_clipping(LCR_ID_I, false)
                    || adc_is_clipping(LCR_ID_V, false)) {
                    v_rms *= 0.8;
                } else {
                    break;
                }
            }
            delay_ms(100);

            int measurement_cycle = 16;
            Complex ratio_list[measurement_cycle];

            for (int i = 0; i < measurement_cycle; ++i) {
                measure_voltage_current(false);
                if (adc_is_clipping(LCR_ID_I, true)
                    || adc_is_clipping(LCR_ID_V, true)) {
                    printf("Warn: ADC is clipping\n");
                }

                Complex voltage = calc_fourier(LCR_ID_V, freq);
                Complex current = calc_fourier(LCR_ID_I, freq);

                ratio_list[i] = is_target_v ? (voltage / current) : (current / voltage);
            }

            Complex ratio = mid(ratio_list, measurement_cycle);

            // printf("%dkHz, %d/%d, Ratio: %.5f + %.5fi = |%.6f|\n", freq / 1000, pga_v_gain_id, pga_i_gain_id, ratio.real, ratio.im, ratio.abs);
            printf("{%.4f,%.4f}", ratio.real, ratio.im);
            is_target_v ? pga_v_gain_id++ : pga_i_gain_id++;
            if (pga_v_gain_id >= pga_list_length || pga_i_gain_id >= pga_list_length) {
                break;
            } else {
                printf(",");
            }
        }
        printf("},\n");
    }
    printf("PGA Calibration Done!\n");
}

void pga_calibration_new()
{
    printf("Insert 1kOhm resistor\n");
    tia_set_gain(2);
    bool is_target_v = true;
    Complex ratio_list[pga_list_length];
    for (int freq_id = 0; freq_id < freq_list_length; ++freq_id) {
        int freq = settings.freq_list[freq_id];
        int pga_v_gain_id = 0;
        int pga_i_gain_id = 0;
        printf("{");
        for (int target_gain_id = 0; target_gain_id < pga_list_length; ++target_gain_id) {
            if (is_target_v) {
                pga_v_gain_id = target_gain_id;
                pga_i_gain_id = 2;
            } else {
                pga_i_gain_id = target_gain_id;
                pga_v_gain_id = 2;
            }

            pga_set_gain(LCR_ID_V, pga_v_gain_id);
            pga_set_gain(LCR_ID_I, pga_i_gain_id);

            double v_rms = 1.0;
            while (1) {
                set_dac_output(freq, v_rms);
                measure_voltage_current(false);
                if (adc_is_clipping(LCR_ID_I, false)
                    || adc_is_clipping(LCR_ID_V, false)) {
                    v_rms *= 0.8;
                } else {
                    break;
                }
            }
            delay_ms(100);

            int measurement_cycle = 16;
            Complex ratio_avg_list[measurement_cycle];

            for (int i = 0; i < measurement_cycle; ++i) {
                measure_voltage_current(false);
                if (adc_is_clipping(LCR_ID_I, false)
                    || adc_is_clipping(LCR_ID_V, false)) {
                    printf("Warn: ADC is clipping\n");
                }

                Complex voltage = calc_fourier(LCR_ID_V, freq);
                Complex current = calc_fourier(LCR_ID_I, freq);

                if (voltage.abs < 0.1 || current.abs < 0.1) {
                    printf("Warn: ADC input is very small %.4f, %.4f\n", voltage.abs, current.abs);
                }

                ratio_avg_list[i] = is_target_v ? (voltage / current) : (current / voltage);
            }

            ratio_list[target_gain_id] = mid(ratio_avg_list, measurement_cycle);
            Complex ratio_rel = ratio_list[target_gain_id] / ratio_list[0];

            // printf("%dkHz, %d/%d, Ratio: %.5f + %.5fi = |%.6f|\n", freq / 1000, pga_v_gain_id, pga_i_gain_id, ratio.real, ratio.im, ratio.abs);
            printf("{%.4f,%.4f}", ratio_rel.real, ratio_rel.im);
            if (target_gain_id + 1 < pga_list_length) {
                printf(",");
            }
        }
        printf("},\n");
    }
    printf("PGA Calibration Done!\n");
}

void adc_data_dump()
{
    measure_voltage_current(false);
    for (uint32_t i = 0; i < adc_data_buf_len; ++i) {
        printf("%d, %d\n", adc_data_buffer[LCR_ID_I][i], adc_data_buffer[LCR_ID_V][i]);
    }
    while (1) {
    }
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

    static bool button3_prev = false;
    if (read_button3()) {
        if (button3_prev == false) {
            button3_prev = true;
            button3_pushed = true;
        }
    } else {
        button3_prev = false;
    }
}

extern "C" {

int _read(int file, char* ptr, int len)
{
    (void)file;

    HAL_UART_Receive(&huart8, (uint8_t*)ptr, len, 0xFFFF);

    /*
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        *ptr++ = __io_getchar();
    }
    */
    return len;
}

int _write(int file, char* ptr, int len)
{
    (void)file;

    HAL_UART_Transmit(&huart8, (uint8_t*)ptr, len, 0xFFFF);

    /*
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        __io_putchar(*ptr++);
    }
    */
    return len;
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
extern DMA_HandleTypeDef hdma_dac1_ch2;

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

/**
 * @brief This function handles DMA1 stream3 global interrupt.
 */
void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_dac1_ch2);
}
}
