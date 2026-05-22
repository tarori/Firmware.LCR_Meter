#include <stm32h7xx.h>
#include "adc.h"
#include "dac.h"
#include "tim.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <cstring>
#include <cmath>
#include <algorithm>

#include "utils.hpp"
#include "app_shared.hpp"

namespace
{

enum LCR_ID_IV {
    LCR_ID_I = 0,
    LCR_ID_V = 1
};

constexpr uint32_t dac_dma_buf_len = 60000;
__attribute__((section(".RAM_DMA"))) uint16_t dac_dma_buffer[dac_dma_buf_len];
double dac_sampling_freq = 0.0;

constexpr uint32_t adc_data_buf_len_nom = 31200;
constexpr uint32_t adc_data_buf_len_120 = 26000;
__attribute__((section(".RAM_DATA"))) uint16_t adc_data_buffer[2][adc_data_buf_len_nom];
constexpr double adc_clock_freq = 24e+6 * 221 / 100;
constexpr double adc_sampling_freq = adc_clock_freq / 17 / 2;
static_assert(abs(adc_data_buf_len_nom / adc_sampling_freq - 1 / 50.0) < 1e-6, "ADC total time must be 20ms");
static_assert(abs(adc_data_buf_len_120 / adc_sampling_freq - 1 / 60.0) < 1e-6, "ADC total time must be 16.6ms");
uint32_t adc_sample_count = adc_data_buf_len_nom;

constexpr int freq_list_length = 13;
constexpr int pga_list_length = 6;
constexpr int tia_list_length = 4;
constexpr int dac_resolution = 12;

struct CalibrationData {
    int freq_list[freq_list_length] = {50, 120, 400, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000};

    double adc_ratio = -1.000;
    double adc_delay_err = 1.1e-9;


    int pga_gain_disp[pga_list_length] = {1, 2, 6, 13, 42, 135};

    Complex pga_v_gain_table[freq_list_length][pga_list_length] = {
        {{1.0000, 0.0000}, {1.9992, 0.0000}, {6.7959, 0.0003}, {13.1114, 0.0006}, {44.5686, 0.0034}, {133.7471, 0.0075}},
        {{1.0000, 0.0000}, {1.9993, 0.0001}, {6.7961, 0.0005}, {13.1120, 0.0007}, {44.5700, 0.0037}, {133.7546, 0.0076}},
        {{1.0000, 0.0000}, {1.9994, 0.0001}, {6.7964, 0.0001}, {13.1123, 0.0002}, {44.5726, 0.0000}, {133.7631, -0.0180}},
        {{1.0000, 0.0000}, {1.9995, 0.0001}, {6.7971, -0.0005}, {13.1131, -0.0010}, {44.5776, -0.0083}, {133.7770, -0.0660}},
        {{1.0000, 0.0000}, {1.9994, 0.0001}, {6.7970, -0.0015}, {13.1129, -0.0028}, {44.5773, -0.0209}, {133.7768, -0.1458}},
        {{1.0000, 0.0000}, {1.9995, -0.0000}, {6.7977, -0.0048}, {13.1139, -0.0089}, {44.5820, -0.0604}, {133.7911, -0.3874}},
        {{1.0000, 0.0000}, {1.9996, -0.0002}, {6.7979, -0.0101}, {13.1144, -0.0183}, {44.5836, -0.1238}, {133.7912, -0.7835}},
        {{1.0000, 0.0000}, {1.9996, -0.0005}, {6.7980, -0.0207}, {13.1148, -0.0386}, {44.5859, -0.2553}, {133.7944, -1.5970}},
        {{1.0000, 0.0000}, {1.9997, -0.0013}, {6.7980, -0.0524}, {13.1152, -0.0954}, {44.5834, -0.6391}, {133.7143, -3.9861}},
        {{1.0000, 0.0000}, {1.9997, -0.0027}, {6.7968, -0.1053}, {13.1134, -0.1933}, {44.5639, -1.2861}, {133.4269, -7.9892}},
        {{1.0000, 0.0000}, {1.9998, -0.0057}, {6.7918, -0.2114}, {13.1066, -0.3889}, {44.4787, -2.5808}, {132.2371, -15.9092}},
        {{1.0000, 0.0000}, {2.0000, -0.0139}, {6.7554, -0.5243}, {13.0555, -0.9678}, {43.8886, -6.3860}, {124.3408, -37.8716}},
        {{1.0000, 0.0000}, {2.0010, -0.0278}, {6.6305, -1.0273}, {12.8771, -1.9214}, {41.8563, -12.3962}, {101.1411, -64.6685}}};

    Complex pga_i_gain_table[freq_list_length][pga_list_length] = {
        {{1.0000, 0.0000}, {1.9992, -0.0000}, {6.7987, 0.0001}, {13.1012, -0.0001}, {44.5528, 0.0002}, {133.6576, -0.0029}},
        {{1.0000, 0.0000}, {1.9992, 0.0000}, {6.7989, 0.0003}, {13.1015, 0.0006}, {44.5553, 0.0027}, {133.6666, 0.0041}},
        {{1.0000, 0.0000}, {1.9994, 0.0000}, {6.7995, -0.0000}, {13.1030, -0.0002}, {44.5600, -0.0019}, {133.6783, -0.0230}},
        {{1.0000, 0.0000}, {1.9994, 0.0001}, {6.7997, -0.0007}, {13.1031, -0.0013}, {44.5617, -0.0103}, {133.6838, -0.0739}},
        {{1.0000, 0.0000}, {1.9994, -0.0000}, {6.7997, -0.0020}, {13.1032, -0.0032}, {44.5624, -0.0240}, {133.6869, -0.1582}},
        {{1.0000, 0.0000}, {1.9994, -0.0001}, {6.7997, -0.0053}, {13.1033, -0.0095}, {44.5622, -0.0642}, {133.6843, -0.4066}},
        {{1.0000, 0.0000}, {1.9994, -0.0003}, {6.7996, -0.0110}, {13.1037, -0.0196}, {44.5631, -0.1312}, {133.6834, -0.8179}},
        {{1.0000, 0.0000}, {1.9994, -0.0006}, {6.7998, -0.0217}, {13.1031, -0.0385}, {44.5623, -0.2612}, {133.6736, -1.6424}},
        {{1.0000, 0.0000}, {1.9994, -0.0015}, {6.7995, -0.0544}, {13.1025, -0.0965}, {44.5555, -0.6515}, {133.5832, -4.0936}},
        {{1.0000, 0.0000}, {1.9994, -0.0029}, {6.7979, -0.1087}, {13.0994, -0.1932}, {44.5289, -1.3043}, {133.2546, -8.1804}},
        {{1.0000, 0.0000}, {1.9994, -0.0059}, {6.7921, -0.2173}, {13.0924, -0.3874}, {44.4418, -2.6069}, {132.0124, -16.2364}},
        {{1.0000, 0.0000}, {1.9998, -0.0146}, {6.7542, -0.5394}, {13.0449, -0.9665}, {43.8452, -6.4623}, {123.7687, -38.5963}},
        {{1.0000, 0.0000}, {2.0009, -0.0297}, {6.6220, -1.0574}, {12.8669, -1.9254}, {41.7512, -12.5526}, {99.6988, -65.4497}}};

    Complex ac_couple_gain[freq_list_length] = {{0.999456, 0.020259}, {0.999912, 0.006790}, {0.999979, 0.002048}, {0.999997, 0.000820}, {0.999995, 0.000409}, {1.000004, 0.000172}, {1.000008, 0.000076}, {1.000017, 0.000047}, {1.000003, 0.000011}, {0.999997, -0.000000}, {0.999974, 0.000001}, {1.000013, 0.000006}, {0.999992, 0.000008}};

    double tia_res_table[tia_list_length] = {20, 100, 1000, 10000};
    double tia_cap_table[tia_list_length] = {0e-12, 10e-12, 10e-12, 10e-12};

    uint32_t adc1_linearity_cal[ADC_LINEAR_CALIB_REG_COUNT] = {0x20080e02, 0x20180201, 0x2017f9fe, 0x200801ff, 0x20081200, 0x01ff};
    uint32_t adc2_linearity_cal[ADC_LINEAR_CALIB_REG_COUNT] = {0x200801ff, 0x20280200, 0x1ff805ff, 0x201809ff, 0x201809ff, 0x01ff};
};

const CalibrationData calibration{};

struct CalibrationInterpolation {
    int lower_id;
    int upper_id;
    double ratio;
};

double clamp_unit(double value)
{
    if (value < 0.0) {
        return 0.0;
    }
    if (value > 1.0) {
        return 1.0;
    }
    return value;
}

double log_frequency_ratio(int freq, int lower_freq, int upper_freq)
{
    if (lower_freq == upper_freq) {
        return 0.0;
    }

    double ratio = (log((double)freq) - log((double)lower_freq))
                   / (log((double)upper_freq) - log((double)lower_freq));
    return clamp_unit(ratio);
}

CalibrationInterpolation get_calibration_interpolation(int freq)
{
    if (freq <= calibration.freq_list[0]) {
        return {0, 0, 0.0};
    }

    for (int i = 0; i < freq_list_length - 1; ++i) {
        int lower_freq = calibration.freq_list[i];
        int upper_freq = calibration.freq_list[i + 1];

        if (freq == lower_freq) {
            return {i, i, 0.0};
        }
        if (freq == upper_freq) {
            return {i + 1, i + 1, 0.0};
        }
        if (freq < upper_freq) {
            return {i, i + 1, log_frequency_ratio(freq, lower_freq, upper_freq)};
        }
    }

    return {freq_list_length - 1, freq_list_length - 1, 0.0};
}

Complex interpolate_complex_gain(Complex lower, Complex upper, double ratio)
{
    if (ratio <= 0.0) {
        return lower;
    }
    if (ratio >= 1.0) {
        return upper;
    }

    if (lower.abs <= 0.0 || upper.abs <= 0.0) {
        return Complex(lower.real + (upper.real - lower.real) * ratio,
            lower.im + (upper.im - lower.im) * ratio);
    }

    double lower_phase = atan2(lower.im, lower.real);
    double upper_phase = atan2(upper.im, upper.real);
    double phase_diff = upper_phase - lower_phase;
    while (phase_diff > M_PI) {
        phase_diff -= 2.0 * M_PI;
    }
    while (phase_diff < -M_PI) {
        phase_diff += 2.0 * M_PI;
    }

    double gain_abs = exp(log(lower.abs) + (log(upper.abs) - log(lower.abs)) * ratio);
    double phase = lower_phase + phase_diff * ratio;
    return Complex(gain_abs * cos(phase), gain_abs * sin(phase));
}

Complex get_pga_v_gain(int freq, int gain_id)
{
    CalibrationInterpolation interp = get_calibration_interpolation(freq);
    return interpolate_complex_gain(calibration.pga_v_gain_table[interp.lower_id][gain_id],
        calibration.pga_v_gain_table[interp.upper_id][gain_id],
        interp.ratio);
}

Complex get_pga_i_gain(int freq, int gain_id)
{
    CalibrationInterpolation interp = get_calibration_interpolation(freq);
    return interpolate_complex_gain(calibration.pga_i_gain_table[interp.lower_id][gain_id],
        calibration.pga_i_gain_table[interp.upper_id][gain_id],
        interp.ratio);
}

Complex get_ac_couple_gain(int freq)
{
    CalibrationInterpolation interp = get_calibration_interpolation(freq);
    return interpolate_complex_gain(calibration.ac_couple_gain[interp.lower_id],
        calibration.ac_couple_gain[interp.upper_id],
        interp.ratio);
}

}  // namespace


void measure_voltage_current(bool is_short);
bool adc_is_clipping(LCR_ID_IV id, bool strict);
Complex calc_DFT(LCR_ID_IV id, int freq);
void set_adc_freq(int freq);
double set_dac_output(int freq, double vdac_pp, double vdac_dc);
void set_dac_bw(int freq);
void pga_set_gain(LCR_ID_IV id, int gain_id);
void pga_set_gain_sub(LCR_ID_IV iv_id, int stage_id, int gain_id);
void adc_calibration_dump();
void adc_data_dump();
void update_probe_compensation(ProbeCompensation& probe, const Complex& impedance, double omega, uint32_t button_events);
MeasurementResult apply_probe_compensation_and_calculate(Complex impedance, int freq, const ProbeCompensation& probe);

int get_pga_gain_display(int gain_id)
{
    if (gain_id < 0 || gain_id >= pga_list_length) {
        return 0;
    }
    return calibration.pga_gain_disp[gain_id];
}

int get_calibration_freq_count()
{
    return freq_list_length;
}

int get_calibration_freq(int freq_id)
{
    if (freq_id < 0) {
        return calibration.freq_list[0];
    }
    if (freq_id >= freq_list_length) {
        return calibration.freq_list[freq_list_length - 1];
    }
    return calibration.freq_list[freq_id];
}

int get_nearest_calibration_freq_id(int freq)
{
    int best_id = 0;
    int best_error = abs(freq - calibration.freq_list[0]);

    for (int i = 1; i < freq_list_length; ++i) {
        int error = abs(freq - calibration.freq_list[i]);
        if (error < best_error) {
            best_error = error;
            best_id = i;
        }
    }
    return best_id;
}

void update_signal_source(MainLoopState& state)
{
    if (!state.dac_changed) {
        return;
    }

    state.freq_id = get_calibration_freq_id(state.freq);
    // delaymeas_start();
    state.vdac_pp = set_dac_output(state.freq, state.vdac_pp, state.vdac_dc);
    // delaymeas_end();
    state.dac_changed = false;
}

RangeState auto_range(int freq)
{
    RangeState range{};
    range.tia_gain_id = tia_list_length - 1;
    range.pga_v_gain_id = 0;
    range.pga_i_gain_id = 0;
    pga_set_gain(LCR_ID_I, range.pga_i_gain_id);
    pga_set_gain(LCR_ID_V, range.pga_v_gain_id);

    while (1) {
        tia_set_gain(range.tia_gain_id);
        if (freq > 2000) {
            delay_ms(10);
            measure_voltage_current(true);
        } else {
            delay_ms(50);
            measure_voltage_current(false);
        }

        if (adc_is_clipping(LCR_ID_I, true) && range.tia_gain_id > 0) {
            --range.tia_gain_id;
        } else {
            break;
        }
    }

    range.pga_v_gain_id = pga_list_length - 1;
    range.pga_i_gain_id = pga_list_length - 1;
    while (1) {
        pga_set_gain(LCR_ID_I, range.pga_i_gain_id);
        pga_set_gain(LCR_ID_V, range.pga_v_gain_id);

        if (freq > 2000) {
            delay_ms(1);
            measure_voltage_current(true);
        } else {
            delay_ms(10);
            measure_voltage_current(false);
        }

        if (adc_is_clipping(LCR_ID_V, false) && range.pga_v_gain_id > 0) {
            range.pga_v_gain_id--;
            continue;
        } else if (adc_is_clipping(LCR_ID_I, false) && range.pga_i_gain_id > 0) {
            range.pga_i_gain_id--;
            continue;
        } else {
            break;
        }
    }

    return range;
}

MeasurementResult measure_and_calculate(MainLoopState& state, const MeasurementRequest& request, const RangeState& range, uint32_t button_events)
{
    // adc_data_dump();

    measure_voltage_current(false);
    Complex voltage = calc_DFT(LCR_ID_V, request.freq);
    Complex current = calc_DFT(LCR_ID_I, request.freq);

    Complex tia_conductance = Complex{1 / calibration.tia_res_table[range.tia_gain_id], 2 * (double)PI * request.freq * calibration.tia_cap_table[range.tia_gain_id]};

    voltage = voltage / get_pga_v_gain(request.freq, range.pga_v_gain_id);
    if (request.dc_couple == false) {
        voltage = voltage / get_ac_couple_gain(request.freq);
    }
    current = (current * tia_conductance) / get_pga_i_gain(request.freq, range.pga_i_gain_id);
    Complex impedance = voltage / current;

    double omega = 2 * M_PI * request.freq;
    update_probe_compensation(state.probe, impedance, omega, button_events);
    return apply_probe_compensation_and_calculate(impedance, request.freq, state.probe);
}

void update_probe_compensation(ProbeCompensation& probe, const Complex& impedance, double omega, uint32_t button_events)
{
    if ((button_events & BUTTON_EVENT_COMPENSATE) == 0) {
        return;
    }

    if (impedance.abs < 10.0) {
        // Short Compensation
        probe.short_inductance = impedance.im / omega;
        probe.short_resistance = impedance.real;
    } else if (impedance.abs > 10000.0) {
        // Open Compensation
        Complex conductance = Complex(1.0) / impedance;
        probe.open_capacitance = conductance.im / omega;
        probe.open_resistance = 1.0 / conductance.real;
    } else {
        // Error: fixture is neither sufficiently short nor sufficiently open.
    }
}

MeasurementResult apply_probe_compensation_and_calculate(Complex impedance, int freq, const ProbeCompensation& probe)
{
    MeasurementResult result{};
    double omega = 2 * M_PI * freq;

    impedance = impedance - Complex(probe.short_resistance, omega * probe.short_inductance);
    Complex conductance = Complex(1.0) / impedance;
    conductance = conductance - Complex(1.0 / probe.open_resistance, omega * probe.open_capacitance);
    impedance = Complex(1.0) / conductance;

    result.impedance = impedance;
    result.conductance = conductance;
    result.sp_mode = true;  // series
    if (result.impedance.abs > 1.2e+3) {
        result.sp_mode = false;  // parallel
    } else if (result.impedance.abs > 50 && result.impedance.real * 1.2 > result.impedance.abs) {
        result.sp_mode = false;  // parallel;
    }

    if (freq == 120 && result.impedance.abs > 1e+1) {
        result.sp_mode = false;
    }

    result.resistance = result.sp_mode ? result.impedance.real : 1.0 / result.conductance.real;
    result.inductance = result.sp_mode ? result.impedance.im / omega : -1.0 / result.conductance.im / omega;
    result.capacitance = result.sp_mode ? -1.0 / (result.impedance.im * omega) : result.conductance.im / omega;

    return result;
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
    while (write_ptr < (is_short ? adc_sample_count / 10 : adc_sample_count)) {
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
        display_error("ADC overrun");
        while (1);
    }

    // Fill unused buffer
    while (write_ptr < adc_sample_count) {
        adc_data_buffer[LCR_ID_I][write_ptr] = UINT16_MAX / 2;
        adc_data_buffer[LCR_ID_V][write_ptr] = UINT16_MAX / 2;
        ++write_ptr;
    }
}

bool initialize_adc()
{
    int hal_state = HAL_OK;
    // hal_state |= HAL_ADCEx_LinearCalibration_SetValue(&hadc1, calibration.adc1_linearity_cal);
    // hal_state |= HAL_ADCEx_LinearCalibration_SetValue(&hadc2, calibration.adc2_linearity_cal);
    // hal_state |= HAL_ADCEx_LinearCalibration_FactorLoad(&hadc1);
    // hal_state |= HAL_ADCEx_LinearCalibration_FactorLoad(&hadc2);
    hal_state |= HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);
    hal_state |= HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);
    // hal_state |= HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    // hal_state |= HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    hal_state |= HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    // adc_calibration_dump();
    printf("ADC CR: 0x%04lx, 0x%04lx\n", ADC1->CR, ADC2->CR);

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

Complex calc_DFT(LCR_ID_IV id, int freq)
{
    double real_sum = 0;
    double im_sum = 0;
    double ratio = (id == LCR_ID_V) ? 1 : calibration.adc_ratio;
    double delay = (id == LCR_ID_V) ? 0 : calibration.adc_delay_err;
    double adc_sampling_period = 1.0 / adc_sampling_freq;
    for (uint32_t i = 0; i < adc_sample_count; ++i) {
        double sin_cos_phase = (2 * M_PI * freq) * (delay + i * adc_sampling_period);
        float sin_val, cos_val;
        my_fast_sincos(sin_cos_phase, &sin_val, &cos_val);
        real_sum += adc_data_buffer[id][i] * cos_val;
        im_sum += -adc_data_buffer[id][i] * sin_val;
    }

    return Complex(ratio * real_sum / adc_sample_count, ratio * im_sum / adc_sample_count);
}

double set_dac_output(int freq, double vdac_pp, double vdac_dc)
{
    set_dac_bw(freq);
    set_adc_freq(freq);

    if (SystemCoreClock != 120000000) {
        printf("Warn: check clock settings\n");
    }
    if (freq < 1000) {
        TIM7->ARR = 200 - 1;
        dac_sampling_freq = 600e+3;
    } else {
        TIM7->ARR = 24 - 1;
        dac_sampling_freq = 5e+6;
    }

    double bandwidth_ratio = sqrt(1.0 + 2.7e-12 * freq * freq);
    vdac_pp *= bandwidth_ratio;
    double vdac_over_ratio = (vdac_pp / 2 + abs(vdac_dc)) / 2.1;
    if (vdac_over_ratio > 1) {
        vdac_pp /= vdac_over_ratio;
        vdac_dc /= vdac_over_ratio;
    }

    vdac_pp = round(vdac_pp * 1000) / 1000.0;
    vdac_dc = round(vdac_dc * 1000) / 1000.0;

    // uint32_t rand_seed = 0;
    // HAL_RNG_GenerateRandomNumber(&hrng, &rand_seed);
    // srand(rand_seed);
    // srand(rand());

    /* No dither */
    for (uint32_t i = 0; i < dac_dma_buf_len; ++i) {
        double dac_code_ideal = 2043.0 + 838.0 * vdac_dc + 419.0 * vdac_pp * my_fast_sin(2 * M_PI * i * freq / (double)dac_sampling_freq);
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

    return vdac_pp / bandwidth_ratio;
}

void set_adc_freq(int freq)
{
    if (freq == 120) {
        adc_sample_count = adc_data_buf_len_120;
    } else {
        adc_sample_count = adc_data_buf_len_nom;
    }
}

bool adc_is_clipping(LCR_ID_IV id, bool strict)
{
    std::pair<uint16_t*, uint16_t*> minmax = std::minmax_element(adc_data_buffer[id], adc_data_buffer[id] + adc_sample_count);
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
    int freq = calibration.freq_list[freq_id];
    // set_dac_output(freq, 1.5);
    set_dac_output(freq, 2.0, 0.0);
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
        for (uint32_t i = 0; i < adc_sample_count; ++i) {
            double cos_val = my_fast_cos(2 * M_PI * dist_order * freq * i / (double)adc_sampling_freq);
            real_avg += adc_data_buffer[LCR_ID_V][i] * cos_val / adc_sample_count;
        }

        double im_avg = 0;
        for (uint32_t i = 0; i < adc_sample_count; ++i) {
            double sin_val = -my_fast_sin(2 * M_PI * dist_order * freq * i / (double)adc_sampling_freq);
            im_avg += adc_data_buffer[LCR_ID_V][i] * sin_val / adc_sample_count;
        }

        double abs_avg = sqrt(real_avg * real_avg + im_avg * im_avg);

        printf("%drd distortion: %.2f [%.2f, %.2f]\n", dist_order, abs_avg, real_avg, im_avg);
    }
}

void pga_calibration()
{
    printf("Insert 1kOhm resistor\n");
    bool is_target_v = false;
    printf("PGA calibration for %c\n", is_target_v ? 'V' : 'I');
    for (int freq_id = 0; freq_id < freq_list_length; ++freq_id) {
        int freq = calibration.freq_list[freq_id];

        Complex measured_gain[2][3] = {};

        for (int target_stage_id = 0; target_stage_id < 2; target_stage_id++) {
            // Source setup
            int max_gain_id = (target_stage_id == 0) ? 1 : 3;
            int mid_gain_id = (target_stage_id == 0) ? 0 : 2;
            tia_set_gain(2);
            double v_pp = 1.0;
            while (1) {
                set_dac_output(freq, v_pp, 0);
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

                    Complex voltage = calc_DFT(LCR_ID_V, freq);
                    Complex current = calc_DFT(LCR_ID_I, freq);

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
        int freq = calibration.freq_list[freq_id];

        double v_pp = 1.0;
        set_dac_output(freq, v_pp, 0);
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
            Complex voltage = calc_DFT(LCR_ID_V, freq);
            Complex current = calc_DFT(LCR_ID_I, freq);
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
            Complex voltage = calc_DFT(LCR_ID_V, freq);
            Complex current = calc_DFT(LCR_ID_I, freq);
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
    for (uint32_t i = 0; i < adc_sample_count; ++i) {
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

int get_calibration_freq_id(int freq)
{
    CalibrationInterpolation interp = get_calibration_interpolation(freq);
    return interp.lower_id;
}
