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
volatile bool button3_pushed = false;
volatile bool button4_pushed = false;

enum LCR_ID_IV {
    LCR_ID_I = 0,
    LCR_ID_V = 1
};

// DAC
constexpr uint32_t dac_dma_buf_len = 40000;
__attribute__((section(".RAM_DMA"))) uint16_t dac_dma_buffer[dac_dma_buf_len];
double dac_sampling_freq = 0;

// ADC
constexpr uint32_t adc_data_buf_len = 27600;
__attribute__((section(".RAM_DATA"))) uint16_t adc_data_buffer[2][adc_data_buf_len];
constexpr double adc_sampling_freq = 12e+6 * 23 / 250;

constexpr int freq_list_length = 13;
constexpr int pga_list_length = 6;
constexpr int tia_list_length = 4;
constexpr int dac_resolution = 12;

struct Settings {
    int freq_list[freq_list_length] = {40, 120, 400, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000};

    double adc_ratio = -1.000;
    double adc_delay_err = 1.1e-9;

    double short_resistance = 0.0;
    double short_inductance = 0.0e-6;
    double open_resistance = 1.0e+24;
    double open_capacitance = 0.000e-12;

    int pga_gain_disp[pga_list_length] = {1, 2, 6, 13, 42, 135};

    Complex pga_v_gain_table[freq_list_length][pga_list_length] = {
        {{1.0000, 0.0000}, {1.9999, 0.0002}, {6.7982, 0.0003}, {13.1151, 0.0014}, {44.5825, 0.0022}, {133.7893, 0.0136}},
        {{1.0000, 0.0000}, {2.0001, 0.0000}, {6.7991, -0.0001}, {13.1170, -0.0001}, {44.5897, -0.0016}, {133.8128, -0.0070}},
        {{1.0000, 0.0000}, {2.0001, -0.0000}, {6.7989, -0.0004}, {13.1167, -0.0010}, {44.5886, -0.0049}, {133.8116, -0.0226}},
        {{1.0000, 0.0000}, {2.0001, -0.0001}, {6.7989, -0.0007}, {13.1168, -0.0021}, {44.5885, -0.0101}, {133.8104, -0.0516}},
        {{1.0000, 0.0000}, {2.0001, 0.0000}, {6.7991, -0.0012}, {13.1171, -0.0035}, {44.5899, -0.0200}, {133.8129, -0.1019}},
        {{1.0000, 0.0000}, {2.0001, 0.0000}, {6.7990, -0.0033}, {13.1170, -0.0086}, {44.5892, -0.0508}, {133.8128, -0.2633}},
        {{1.0000, 0.0000}, {2.0001, -0.0000}, {6.7991, -0.0064}, {13.1170, -0.0179}, {44.5902, -0.1014}, {133.8148, -0.5270}},
        {{1.0000, 0.0000}, {2.0000, -0.0000}, {6.7989, -0.0126}, {13.1165, -0.0352}, {44.5878, -0.2017}, {133.8046, -1.0509}},
        {{1.0000, 0.0000}, {2.0001, -0.0002}, {6.7989, -0.0317}, {13.1167, -0.0886}, {44.5863, -0.5053}, {133.7774, -2.6288}},
        {{1.0000, 0.0000}, {2.0001, -0.0004}, {6.7986, -0.0635}, {13.1152, -0.1777}, {44.5745, -1.0117}, {133.6597, -5.2600}},
        {{1.0000, 0.0000}, {2.0001, -0.0006}, {6.7961, -0.1267}, {13.1088, -0.3542}, {44.5192, -2.0214}, {133.1709, -10.4916}},
        {{1.0000, 0.0000}, {2.0004, -0.0014}, {6.7821, -0.3153}, {13.0629, -0.8835}, {44.1532, -5.0238}, {129.8579, -25.7727}},
        {{1.0000, 0.0000}, {2.0012, -0.0027}, {6.7334, -0.6248}, {12.9008, -1.7546}, {42.8721, -9.8723}, {118.7880, -48.5016}}};

    Complex pga_i_gain_table[freq_list_length][pga_list_length] = {
        {{1.0000, 0.0000}, {1.9989, 0.0000}, {6.7970, 0.0001}, {13.0975, 0.0000}, {44.5366, -0.0001}, {133.5888, 0.0053}},
        {{1.0000, 0.0000}, {1.9988, 0.0000}, {6.7964, -0.0001}, {13.0975, -0.0001}, {44.5349, -0.0012}, {133.5899, -0.0102}},
        {{1.0000, 0.0000}, {1.9989, -0.0000}, {6.7969, -0.0002}, {13.0978, -0.0010}, {44.5377, -0.0041}, {133.5980, -0.0242}},
        {{1.0000, 0.0000}, {1.9989, -0.0000}, {6.7972, -0.0008}, {13.0984, -0.0017}, {44.5397, -0.0106}, {133.6031, -0.0523}},
        {{1.0000, 0.0000}, {1.9989, -0.0000}, {6.7968, -0.0013}, {13.0979, -0.0037}, {44.5364, -0.0213}, {133.5945, -0.1078}},
        {{1.0000, 0.0000}, {1.9988, -0.0000}, {6.7967, -0.0032}, {13.0976, -0.0090}, {44.5360, -0.0514}, {133.5917, -0.2619}},
        {{1.0000, 0.0000}, {1.9988, -0.0001}, {6.7965, -0.0065}, {13.0968, -0.0180}, {44.5331, -0.1025}, {133.5807, -0.5226}},
        {{1.0000, 0.0000}, {1.9989, 0.0000}, {6.7969, -0.0124}, {13.0975, -0.0352}, {44.5361, -0.2011}, {133.5888, -1.0381}},
        {{1.0000, 0.0000}, {1.9989, -0.0002}, {6.7964, -0.0315}, {13.0974, -0.0890}, {44.5313, -0.5050}, {133.5544, -2.6043}},
        {{1.0000, 0.0000}, {1.9989, -0.0002}, {6.7961, -0.0623}, {13.0952, -0.1771}, {44.5169, -1.0047}, {133.4291, -5.1885}},
        {{1.0000, 0.0000}, {1.9988, -0.0005}, {6.7937, -0.1244}, {13.0879, -0.3544}, {44.4627, -2.0079}, {132.9583, -10.3493}},
        {{1.0000, 0.0000}, {1.9989, -0.0011}, {6.7795, -0.3103}, {13.0413, -0.8824}, {44.0974, -4.9936}, {129.7168, -25.4383}},
        {{1.0000, 0.0000}, {2.0001, -0.0023}, {6.7330, -0.6158}, {12.8803, -1.7572}, {42.8285, -9.8320}, {118.9147, -48.0196}}};

    Complex ac_couple_gain[freq_list_length] = {{0.999456, 0.020259}, {0.999912, 0.006790}, {0.999979, 0.002048}, {0.999997, 0.000820}, {0.999995, 0.000409}, {1.000004, 0.000172}, {1.000008, 0.000076}, {1.000017, 0.000047}, {1.000003, 0.000011}, {0.999997, -0.000000}, {0.999974, 0.000001}, {1.000013, 0.000006}, {0.999992, 0.000008}};

    double tia_res_table[tia_list_length] = {20, 100, 1000, 10000};
    double tia_cap_table[tia_list_length] = {0e-12, 10e-12, 10e-12, 10e-12};

    uint32_t adc1_linearity_cal[ADC_LINEAR_CALIB_REG_COUNT] = {0x20080e02, 0x20180201, 0x2017f9fe, 0x200801ff, 0x20081200, 0x01ff};
    uint32_t adc2_linearity_cal[ADC_LINEAR_CALIB_REG_COUNT] = {0x200801ff, 0x20280200, 0x1ff805ff, 0x201809ff, 0x201809ff, 0x01ff};
} settings;

SMR12864 lcd;

bool initialize_adc();
bool initialize_dac_tim();
double set_dac_output(int freq, double vdac_pp);
void measure_voltage_current(bool is_short);
bool adc_is_clipping(LCR_ID_IV id, bool strict);
double read_battery_voltage();
void pga_calibration();
void dac_calibration();
void ac_couple_calibration();
void pga_set_gain(LCR_ID_IV id, int gain_id);
void pga_set_gain_sub(LCR_ID_IV iv_id, int stage_id, int gain_id);
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
        while (1);
    }

    if (initialize_dac_tim()) {
        printf("DAC initialize error\n");
        lcd.cls();
        lcd.printf("DAC initialize error");
        while (1);
    }

    int freq = 0;
    int freq_id = 9;  // 100kHz Default
    int vdac_id = 0;
    bool dac_changed = true;
    bool dc_couple = true;
    bool lcd_backlight = false;
    TIM4->CNT -= 4 * ((TIM4->CNT / 4 - freq_id) % freq_list_length);
    double v_dac = 2.0;
    TIM3->CNT = INT16_MAX + 4 * (v_dac / 0.1);

    init_done = true;
    while (1) {
        /* Signal source setup */
        int freq_id_new = (TIM4->CNT / 4) % freq_list_length;
        if (freq_id != freq_id_new) {
            dac_changed = true;
        }

        int vdac_id_new = ((int32_t)TIM3->CNT - INT16_MAX) / 4;
        if (vdac_id_new != vdac_id) {
            dac_changed = true;
        }

        if (dac_changed) {
            freq_id = freq_id_new;
            v_dac = vdac_id_new * 0.1;
            if (v_dac <= 0) {
                v_dac = 0.1;
            }
            vdac_id = vdac_id_new;
        }

        if (button2_pushed) {
            button2_pushed = false;
            dc_couple = !dc_couple;
            coupling_set_dc(dc_couple, dc_couple);
            dac_changed = true;
        }

        if (button3_pushed) {
            button3_pushed = false;
            lcd_backlight = !lcd_backlight;
            set_backlight(lcd_backlight);
        }

        if (dac_changed) {
            freq = settings.freq_list[freq_id];
            delaymeas_start();
            v_dac = set_dac_output(freq, v_dac);
            delaymeas_end();
            dac_changed = false;
        }

        double battery_voltage = read_battery_voltage();

        while (0) {
            // adc_calibration();
            // dac_calibration();
            // pga_calibration();
            ac_couple_calibration();
            delay_ms(5000);
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

            if (adc_is_clipping(LCR_ID_I, true) && tia_gain_id > 0) {
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
        if (dc_couple == false) {
            voltage = voltage / settings.ac_couple_gain[freq_id];
        }
        current = (current * tia_conductance) / settings.pga_i_gain_table[freq_id][pga_i_gain_id];
        Complex impedance = voltage / current;

        /* Probe compensation */
        double omega = 2 * M_PI * freq;
        if (button1_pushed) {
            button1_pushed = false;
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
            printf("R: %.4f Ohm, L: %.4f uH, C: %.4f uF, Z: %.4f Ohm, BAT: %.3f V\n",
                resistance, inductance * 1.0e+6, capacitance * 1.0e+6, impedance.abs, battery_voltage);
        } else {
            printf("R: %.4f Ohm, L: %.4f uH, C: %.5f pF, Z: %.4f Ohm, BAT: %.3f V\n",
                resistance, inductance * 1.0e+6, capacitance * 1.0e+12, impedance.abs, battery_voltage);
        }

        lcd.cls();
        lcd.printf("%5d%s %4.2fVpp %s", freq < 1000 ? freq : freq / 1000, freq < 1000 ? "Hz" : "kHz", v_dac, dc_couple ? "DC" : "AC");

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
        lcd.printf("TIA:%d PGA:Ix%d,Vx%d", tia_gain_id, settings.pga_gain_disp[pga_i_gain_id], settings.pga_gain_disp[pga_v_gain_id]);

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
            lcd.printf(" %6.3fdeg", theta_deg);
        }
    }
}

void measure_voltage_current(bool is_short)
{
    ScopedLock lock;
    uint32_t write_ptr = 0;

    // Dummy read
    for (uint32_t i = 0; i < 10; ++i) {
        while (!READ_BIT(ADC1->ISR, ADC_ISR_EOC));
        ADC1->ISR = ADC_ISR_EOC | ADC_ISR_OVR;
        ADC2->ISR = ADC_ISR_EOC | ADC_ISR_OVR;
    }

    // Read conversion results
    while (write_ptr < (is_short ? adc_data_buf_len / 10 : adc_data_buf_len)) {
        while (!READ_BIT(ADC1->ISR, ADC_ISR_EOC));

        uint32_t val = ADC12_COMMON->CDR;
        adc_data_buffer[LCR_ID_I][write_ptr] = val & 0xFFFF;
        adc_data_buffer[LCR_ID_V][write_ptr] = val >> 16;
        ++write_ptr;

        ADC1->ISR = ADC_ISR_EOC;
    }

    // Overrun check
    if (READ_BIT(ADC1->ISR, ADC_ISR_OVR)) {
        printf("ADC overrun, 0x%lx\n", ADC12_COMMON->CSR);
        lcd.cls();
        lcd.printf("ADC overrun");
        while (1);
    }

    // Fill unused buffer
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
    // hal_state |= HAL_ADCEx_LinearCalibration_FactorLoad(&hadc1);
    // hal_state |= HAL_ADCEx_LinearCalibration_FactorLoad(&hadc2);
    hal_state |= HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);
    hal_state |= HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);
    // hal_state |= HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    // hal_state |= HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    hal_state |= HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    // adc_calibration_dump();

    /* Start conversion */
    MODIFY_REG(ADC12_COMMON->CCR, ADC_CCR_DAMDF, ADC_DUALMODEDATAFORMAT_32_10_BITS);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_Start(&hadc1);

    return hal_state != HAL_OK;
}

bool initialize_dac_tim()
{
    int hal_state = HAL_OK;
    hal_state |= HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)&dac_dma_buffer[0], dac_dma_buf_len, DAC_ALIGN_12B_R);
    CLEAR_BIT(((DMA_Stream_TypeDef*)hdac1.DMA_Handle1->Instance)->CR, DMA_IT_TC | DMA_IT_HT);

    delay_ms(10);

    hal_state |= HAL_TIM_Base_Start(&htim23);                     // TIM for counter
    hal_state |= HAL_TIM_Base_Start(&htim7);                      // TIM for DAC
    hal_state |= HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  // Encoder 1
    hal_state |= HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // Encoder 2
    TIM3->CNT = INT16_MAX;
    TIM4->CNT = INT16_MAX;

    return hal_state != HAL_OK;
}

void adc_calibration_dump()
{
    {
        uint32_t cal_data[ADC_LINEAR_CALIB_REG_COUNT];
        HAL_ADCEx_LinearCalibration_GetValue(&hadc1, cal_data);
        printf("ADC1: {0x%04lx, 0x%04lx, 0x%04lx, 0x%04lx, 0x%04lx, 0x%04lx}\n",
            cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4], cal_data[5]);
        HAL_ADCEx_LinearCalibration_GetValue(&hadc2, cal_data);
        printf("ADC2: {0x%04lx, 0x%04lx, 0x%04lx, 0x%04lx, 0x%04lx, 0x%04lx}\n",
            cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4], cal_data[5]);
    }
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
    while (1);
}

Complex calc_fourier(LCR_ID_IV id, int freq)
{
    double real_sum = 0;
    double im_sum = 0;
    double ratio = (id == LCR_ID_V) ? 1 : settings.adc_ratio;
    double delay = (id == LCR_ID_V) ? 0 : settings.adc_delay_err;
    double adc_sampling_period = 1.0 / adc_sampling_freq;
    for (uint32_t i = 0; i < adc_data_buf_len; ++i) {
        double sin_cos_phase = (2 * M_PI * freq) * (delay + i * adc_sampling_period);
        double cos_val = my_fast_cos(sin_cos_phase);
        double sin_val = -my_fast_sin(sin_cos_phase);
        real_sum += adc_data_buffer[id][i] * cos_val;
        im_sum += adc_data_buffer[id][i] * sin_val;
    }

    return Complex(ratio * real_sum / adc_data_buf_len, ratio * im_sum / adc_data_buf_len);
}

double set_dac_output(int freq, double vdac_pp)
{
    set_dac_bw(freq);
    if (SystemCoreClock != 120000000) {
        printf("Warn: check clock settings\n");
    }
    if (freq < 1000) {
        TIM7->ARR = 300 - 1;
        dac_sampling_freq = 400e+3;
    } else {
        TIM7->ARR = 24 - 1;
        dac_sampling_freq = 5e+6;
    }

    double k = sqrt(1.0 + 2.7e-12 * freq * freq);
    vdac_pp *= k;
    if (vdac_pp > 3.0) {
        vdac_pp = 3.0;
    }

    // uint32_t rand_seed = 0;
    // HAL_RNG_GenerateRandomNumber(&hrng, &rand_seed);
    // srand(rand_seed);
    // srand(rand());

    /* No dither */
    for (uint32_t i = 0; i < dac_dma_buf_len; ++i) {
        double dac_code_ideal = 2043.0 + 419.0 * vdac_pp * my_fast_sin(2 * M_PI * i * freq / (double)dac_sampling_freq);
        dac_dma_buffer[i] = dac_code_ideal;
    }

    /* Constant dither */
    /*
    for (uint32_t i = 0; i < dac_dma_buf_len; ++i) {
        double dither = ((rand() % 16384) - 8192) / 16384.0;
        double dac_code_ideal = 2034.0 + 1173.0 * v_rms * my_fast_sin(2 * M_PI * i * freq / (double)dac_sampling_freq) + dither;
        dac_dma_buffer[i] = dac_code_ideal;
    }
    */
    // SCB_CleanDCache();

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
            dac_dma_buffer[i] = dac_code;
        }
    }
    */

    return vdac_pp / k;
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

void set_backlight(bool state)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void coupling_set_dc(bool cur, bool pot)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, cur ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, pot ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

double read_battery_voltage()
{
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 100);
    uint16_t adc_val = HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);
    return 3.3 * 2.0 / 4.0 * adc_val / 4096.0;
}

void dac_calibration()
{
    tia_set_gain(0);
    int freq_id = 3;  // 1kHz
    // int freq_id = 9;  // 100kHz
    int freq = settings.freq_list[freq_id];
    // set_dac_output(freq, 1.5);
    set_dac_output(freq, 2.0);
    tia_set_gain(0);
    pga_set_gain(LCR_ID_V, 0);
    pga_set_gain(LCR_ID_I, 0);
    delay_ms(100);

    measure_voltage_current(false);
    if (adc_is_clipping(LCR_ID_I, false) || adc_is_clipping(LCR_ID_V, false)) {
        //printf("Warn: ADC is clipping\n");
    }

    adc_data_dump();

    // Calculate distortion
    for (int dist_order = 1; dist_order <= 5; dist_order++) {
        double real_avg = 0;
        for (uint32_t i = 0; i < adc_data_buf_len; ++i) {
            double cos_val = my_fast_cos(2 * M_PI * dist_order * freq * i / (double)adc_sampling_freq);
            real_avg += adc_data_buffer[LCR_ID_V][i] * cos_val / adc_data_buf_len;
        }

        double im_avg = 0;
        for (uint32_t i = 0; i < adc_data_buf_len; ++i) {
            double sin_val = -my_fast_sin(2 * M_PI * dist_order * freq * i / (double)adc_sampling_freq);
            im_avg += adc_data_buffer[LCR_ID_V][i] * sin_val / adc_data_buf_len;
        }

        double abs_avg = sqrt(real_avg * real_avg + im_avg * im_avg);

        printf("%drd distortion: %.2f [%.2f, %.2f]\n", dist_order, abs_avg, real_avg, im_avg);
    }
}

static_assert(pga_list_length == 6);

void pga_calibration()
{
    printf("Insert 1kOhm resistor\n");
    bool is_target_v = true;
    printf("PGA calibration for %c\n", is_target_v ? 'V' : 'I');
    for (int freq_id = 0; freq_id < freq_list_length; ++freq_id) {
        int freq = settings.freq_list[freq_id];

        Complex measured_gain[2][3] = {};

        for (int target_stage_id = 0; target_stage_id < 2; target_stage_id++) {
            // Source setup
            int max_gain_id = (target_stage_id == 0) ? 1 : 3;
            int mid_gain_id = (target_stage_id == 0) ? 0 : 2;
            tia_set_gain(2);
            double v_pp = 1.0;
            while (1) {
                set_dac_output(freq, v_pp);
                pga_set_gain(LCR_ID_V, 0);  // release vref clampling
                pga_set_gain(LCR_ID_I, 0);
                delay_ms(10);
                pga_set_gain_sub(LCR_ID_V, target_stage_id, is_target_v ? max_gain_id : mid_gain_id);
                pga_set_gain_sub(LCR_ID_I, target_stage_id, is_target_v ? mid_gain_id : max_gain_id);
                delay_ms(10);
                measure_voltage_current(false);
                if (adc_is_clipping(LCR_ID_I, true)
                    || adc_is_clipping(LCR_ID_V, true)) {
                    v_pp *= 0.9;
                    if (v_pp < 0.04) {
                        printf("\nSource level not converged, clipping %c\n", adc_is_clipping(LCR_ID_I, true) ? 'I' : 'V');
                        break;
                    }
                } else {
                    break;
                }
            }
            delay_ms(100);

            // Measurement
            Complex ratio_list[4] = {};

            for (int target_gain_id = 0; target_gain_id <= max_gain_id; ++target_gain_id) {

                int measurement_cycle = 16;
                Complex ratio_avg_temp[measurement_cycle];

                pga_set_gain_sub(LCR_ID_V, target_stage_id, is_target_v ? target_gain_id : mid_gain_id);
                pga_set_gain_sub(LCR_ID_I, target_stage_id, is_target_v ? mid_gain_id : target_gain_id);
                delay_ms(100);

                for (int i = 0; i < measurement_cycle; ++i) {
                    // printf("%c", target_stage_id ? '#' : '.');
                    measure_voltage_current(false);
                    if (adc_is_clipping(LCR_ID_I, false)
                        || adc_is_clipping(LCR_ID_V, false)) {
                        printf("Warn: ADC is clipping\n");
                    }

                    Complex voltage = calc_fourier(LCR_ID_V, freq);
                    Complex current = calc_fourier(LCR_ID_I, freq);

                    if (voltage.abs < 0.02 * 8192 || current.abs < 0.02 * 8192) {
                        printf("\nWarn: ADC input is very small: [V,I] %.4f, %.4f\n", voltage.abs, current.abs);
                    }

                    ratio_avg_temp[i] = is_target_v ? (voltage / current) : (current / voltage);
                }
                ratio_list[target_gain_id] = mid(ratio_avg_temp, measurement_cycle);

                if (target_gain_id == 0) {
                    measured_gain[target_stage_id][target_gain_id] = {1.0, 0};
                } else {
                    measured_gain[target_stage_id][target_gain_id] = ratio_list[target_gain_id] / ratio_list[0];
                }
            }
        }

        printf("{");
        for (int gain_id = 0; gain_id < pga_list_length; ++gain_id) {
            // Gain calculation
            Complex pga_gain = {};
            switch (gain_id) {
            case 0:
                pga_gain = measured_gain[0][0] * measured_gain[1][0];
                break;
            case 1:
                pga_gain = measured_gain[0][0] * measured_gain[1][1];
                break;
            case 2:
                pga_gain = measured_gain[0][0] * measured_gain[1][2];
                break;
            case 3:
                pga_gain = measured_gain[0][1] * measured_gain[1][1];
                break;
            case 4:
                pga_gain = measured_gain[0][1] * measured_gain[1][2];
                break;
            case 5:
                pga_gain = measured_gain[0][1] * measured_gain[1][3];
                break;
            default:
                printf("PGA calibration unexpected\n");
                break;
            }

            printf("{%.4f,%.4f}", pga_gain.real, pga_gain.im);
            if (gain_id != pga_list_length - 1) {
                printf(",");
            }
        }
        printf("},\n");
    }
    printf("PGA Calibration Done!\n");
}

void ac_couple_calibration()
{
    printf("Insert 1kOhm resistor\n");
    pga_set_gain(LCR_ID_V, 0);
    pga_set_gain(LCR_ID_I, 0);
    tia_set_gain(2);
    printf("{");
    for (int freq_id = 0; freq_id < freq_list_length; ++freq_id) {
        int freq = settings.freq_list[freq_id];

        double v_pp = 1.0;
        set_dac_output(freq, v_pp);
        coupling_set_dc(true, true);  // DC couple
        delay_ms(200);

        int measurement_cycle = 4;
        Complex ratio_list[measurement_cycle];

        for (int i = 0; i < measurement_cycle; ++i) {
            measure_voltage_current(false);
            if (adc_is_clipping(LCR_ID_I, false)
                || adc_is_clipping(LCR_ID_V, false)) {
                printf("Warn: ADC is clipping\n");
            }
            Complex voltage = calc_fourier(LCR_ID_V, freq);
            Complex current = calc_fourier(LCR_ID_I, freq);
            ratio_list[i] = voltage / current;
            if (voltage.abs < 0.1 * 8192 || current.abs < 0.1 * 8192) {
                printf("\nWarn: ADC input is very small: [V,I] %.4f, %.4f\n", voltage.abs, current.abs);
            }
        }
        Complex ratio_dc = mid(ratio_list, measurement_cycle);

        coupling_set_dc(true, false);  // AC couple
        delay_ms(100);
        for (int i = 0; i < measurement_cycle; ++i) {
            measure_voltage_current(false);
            if (adc_is_clipping(LCR_ID_I, false)
                || adc_is_clipping(LCR_ID_V, false)) {
                printf("\nWarn: ADC is clipping\n");
            }
            Complex voltage = calc_fourier(LCR_ID_V, freq);
            Complex current = calc_fourier(LCR_ID_I, freq);
            ratio_list[i] = voltage / current;
            if (voltage.abs < 0.2 * 8192 || current.abs < 0.2 * 8192) {
                printf("\nWarn: ADC input is very small: [V,I] %.4f, %.4f\n", voltage.abs, current.abs);
            }
        }

        Complex ratio_ac = mid(ratio_list, measurement_cycle);

        Complex ratio = ratio_ac / ratio_dc;

        printf("{%.6f,%.6f}", ratio.real, ratio.im);
        if (freq_id < freq_list_length - 1) {
            printf(",");
        }
    }
    printf("}\n");
    printf("AC Calibration Done!\n");
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

void pga_set_gain_sub(LCR_ID_IV iv_id, int stage_id, int gain_id)
{
    if (iv_id == LCR_ID_I) {
        if (stage_id == 0) {
            switch (gain_id) {
            case 0:  // 1x1
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
                break;
            case 1:  // 1x2
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
                break;
            default:
                printf("PGA sub gain error\n");
                return;
            }
        } else if (stage_id == 1) {
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
            default:
                printf("PGA sub gain error\n");
                return;
            }
        } else {
            printf("PGA sub gain error\n");
        }

    } else {
        if (stage_id == 0) {
            switch (gain_id) {
            case 0:  // 1x1
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
                break;
            case 1:  // 1x2
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
                break;
            default:
                printf("PGA sub gain error\n");
                return;
            }
        } else if (stage_id == 1) {
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
            default:
                printf("PGA sub gain error\n");
                return;
            }
        } else {
            printf("PGA sub gain error\n");
        }
    }
}

void pga_set_gain(LCR_ID_IV iv_id, int gain_id)
{
    switch (gain_id) {
    case 0:  // 1x1
        pga_set_gain_sub(iv_id, 0, 0);
        pga_set_gain_sub(iv_id, 1, 0);
        break;
    case 1:  // 1x2
        pga_set_gain_sub(iv_id, 0, 0);
        pga_set_gain_sub(iv_id, 1, 1);
        break;
    case 2:  // 1x6.1
        pga_set_gain_sub(iv_id, 0, 0);
        pga_set_gain_sub(iv_id, 1, 2);
        break;
    case 3:  // 6.6x2
        pga_set_gain_sub(iv_id, 0, 1);
        pga_set_gain_sub(iv_id, 1, 1);
        break;
    case 4:  // 6.6x6.1
        pga_set_gain_sub(iv_id, 0, 1);
        pga_set_gain_sub(iv_id, 1, 2);
        break;
    case 5:  // 6.6x20.4
        pga_set_gain_sub(iv_id, 0, 1);
        pga_set_gain_sub(iv_id, 1, 3);
        break;
    default:
        printf("PGA gain error\n");
        return;
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
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
        break;
    case 1:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
        break;
    case 2:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
        break;
    case 3:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
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
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        break;
    case 1:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        break;
    case 2:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        break;
    case 3:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        break;
    default:
        printf("DAC BW Error\n");
    }
}


bool button1_prev = false;
bool button2_prev = false;
bool button3_prev = false;
bool button4_prev = false;

void update_button()
{
    bool button1_state = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET);
    if (button1_state && !button1_prev) {
        button1_pushed = true;
    }
    button1_prev = button1_state;

    bool button2_state = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET);
    if (button2_state && !button2_prev) {
        button2_pushed = true;
    }
    button2_prev = button2_state;

    bool button3_state = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET);
    if (button3_state && !button3_prev) {
        button3_pushed = true;
    }
    button3_prev = button3_state;

    bool button4_state = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_RESET);
    if (button4_state && !button4_prev) {
        button4_pushed = true;
    }
    button4_prev = button4_state;
}

void callback_1ms()
{
    if (!init_done) {
        return;
    }

    update_button();
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

    // HAL_UART_Transmit(&huart8, (uint8_t*)ptr, len, 0xFFFF);

    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        while (!READ_BIT(huart8.Instance->ISR, UART_FLAG_TXE));
        huart8.Instance->TDR = ptr[DataIdx];
    }
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

extern DMA_HandleTypeDef hdma_dac1_ch1;

/**
 * @brief This function handles DMA1 stream2 global interrupt.
 */
void DMA1_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_dac1_ch1);
    printf("DMA1_Stream2_IRQHandler\n");
}
}
