#include <stm32h7xx.h>
#include "dac.h"
#include "tim.h"
#include "usart.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <climits>

#include "utils.hpp"
#include "app_shared.hpp"

namespace
{

struct ButtonState {
    bool previous[4] = {};
    volatile uint32_t pending_events = 0;
};

struct FrequencyStepRange {
    int first_freq_hz;
    int last_freq_hz;
    int step_hz;
};

// AllFrequencies mode.  To add another step range, append one entry here.
constexpr FrequencyStepRange measurement_frequency_ranges[] = {
    {50, 1000, 50},
    {1250, 9750, 250},
    {10000, 99500, 500},
    {100000, 1000000, 5000},
};

// Frequencies with exact ADC sampling aliases are removed from the selectable
// list.  Fs_ADC = 1.56 MHz, so 780 kHz is Nyquist.  The lower points are
// Fs_ADC/N where low-order harmonics can fold exactly onto the fundamental.
constexpr int avoided_measurement_frequencies[] = {
    260000,  // Fs/6: 5th harmonic folds to the fundamental
    312000,  // Fs/5: 4th harmonic folds to the fundamental; not selected by the current grid
    390000,  // Fs/4: 3rd harmonic folds to the fundamental
    520000,  // Fs/3: 2nd harmonic folds to the fundamental
    780000,  // Fs/2: Nyquist; quadrature information is lost
};

volatile bool button_scan_enabled = false;
ButtonState buttons;

}  // namespace

MainLoopState initialize_main_loop_state();
void initialize_main_hardware();
void apply_front_panel_input(MainLoopState& state, uint32_t button_events);
void set_measurement_frequency(MainLoopState& state, int freq);
void set_vdac_step(MainLoopState& state, int vdac_step);
void update_dc_bias_offset(MainLoopState& state);
void initialize_encoder_positions(const MainLoopState& state);
void initialize_frequency_encoder_position(const MainLoopState& state);
int read_frequency_encoder(const MainLoopState& state);
int read_frequency_encoder_step();
int read_vdac_encoder_step();
int encoder_step_to_selected_frequency(const MainLoopState& state, int step_id);
int selected_frequency_to_encoder_step(const MainLoopState& state, int freq);
int all_frequency_step_to_freq(int step_id);
int freq_to_all_frequency_step(int freq);
int all_frequency_step_count();
int count_frequency_steps_in_range(const FrequencyStepRange& range);
bool is_avoided_measurement_frequency(int freq);
int calibration_frequency_step_to_freq(int step_id);
int freq_to_calibration_frequency_step(int freq);
void toggle_frequency_selection_mode(MainLoopState& state);
void latch_button_event(int index, bool is_pressed, uint32_t event_mask);
bool read_compensate_button_pressed();
bool read_coupling_button_pressed();
bool read_backlight_button_pressed();
bool read_frequency_mode_button_pressed();
void initialize_button_state();
void update_button();
uint32_t consume_button_events();
void callback_1ms();

void main_loop()
{
    initialize_main_hardware();
    MainLoopState state = initialize_main_loop_state();

    if (read_frequency_mode_button_pressed()) {
        measure_frequency_sweep(state, 50, 1000000);
    }

    initialize_button_state();
    button_scan_enabled = true;
    while (1) {
        uint32_t button_events = consume_button_events();

        apply_front_panel_input(state, button_events);
        update_signal_source(state);

        double battery_voltage = read_battery_voltage();

#if 0
        // Temporary calibration entry point. Enable one of these while generating calibration data.
        // adc_calibration();
        // dac_calibration();
        pga_calibration();
        // ac_couple_calibration();
        delay_ms(5000);
#endif

        // measure_frequency_sweep(state, 50, 1000000);

        MeasurementRequest request{state.freq, state.freq_id, state.dc_couple};
        RangeState range = auto_range(request.freq);
        MeasurementResult result = measure_and_calculate(state, request, range, button_events);

        display_measurement_result(state, range, result, battery_voltage);
    }
}

void initialize_main_hardware()
{
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    printf("Hello, I am H730 working at %ld MHz\n", SystemCoreClock / 1000 / 1000);

    tia_set_gain(0);
    coupling_set_dc(true, true);
    delay_ms(100);
    set_backlight(false);

    initialize_display();

    if (initialize_adc()) {
        printf("ADC initialize error\n");
        display_error("ADC initialize error");
        while (1);
    }

    if (initialize_dac_tim()) {
        printf("DAC initialize error\n");
        display_error("DAC initialize error");
        while (1);
    }
}


void measure_frequency_sweep(MainLoopState& state, int start_freq, int stop_freq)
{
    if (start_freq > stop_freq) {
        int tmp = start_freq;
        start_freq = stop_freq;
        stop_freq = tmp;
    }

    printf("Frequency sweep: %d Hz to %d Hz\n", start_freq, stop_freq);

    FrequencySelectionMode saved_freq_selection_mode = state.freq_selection_mode;
    int saved_freq = state.freq;

    state.freq_selection_mode = FrequencySelectionMode::AllFrequencies;
    int start_step = freq_to_all_frequency_step(start_freq);
    int stop_step = freq_to_all_frequency_step(stop_freq);

    for (int step = start_step; step <= stop_step; ++step) {
        set_measurement_frequency(state, all_frequency_step_to_freq(step));
        update_signal_source(state);

        double battery_voltage = read_battery_voltage();
        MeasurementRequest request{state.freq, state.freq_id, state.dc_couple};
        RangeState range = auto_range(request.freq);
        MeasurementResult result = measure_and_calculate(state, request, range, 0);

        display_measurement_result(state, range, result, battery_voltage);
    }

    state.freq_selection_mode = saved_freq_selection_mode;
    set_measurement_frequency(state, saved_freq);
    initialize_frequency_encoder_position(state);
    update_signal_source(state);

    printf("Frequency sweep done\n");

    while (1);
}

MainLoopState initialize_main_loop_state()
{
    MainLoopState state{};
    set_measurement_frequency(state, 100000);  // 100 kHz default
    set_vdac_step(state, 20);                  // 2.0 Vpp default
    state.dc_couple = true;
    state.dc_bias = false;
    state.lcd_backlight = false;
    state.freq_selection_mode = FrequencySelectionMode::CalibrationOnly;
    update_dc_bias_offset(state);

    initialize_encoder_positions(state);
    return state;
}

void apply_front_panel_input(MainLoopState& state, uint32_t button_events)
{
    if (button_events & BUTTON_EVENT_FREQ_MODE) {
        toggle_frequency_selection_mode(state);
    }

    set_measurement_frequency(state, read_frequency_encoder(state));
    set_vdac_step(state, read_vdac_encoder_step());

    if (button_events & BUTTON_EVENT_COUPLING) {
        state.dc_couple = !state.dc_couple;
        coupling_set_dc(state.dc_couple, state.dc_couple);
        state.dac_changed = true;
    }

    if (button_events & BUTTON_EVENT_BACKLIGHT) {
        state.lcd_backlight = !state.lcd_backlight;
        set_backlight(state.lcd_backlight);
    }

    if (button_events & BUTTON_EVENT_DC_BIAS) {
        state.dc_bias = !state.dc_bias;
        update_dc_bias_offset(state);
        state.dac_changed = true;
    }
}

void set_measurement_frequency(MainLoopState& state, int freq)
{
    int normalized_freq = encoder_step_to_selected_frequency(state, selected_frequency_to_encoder_step(state, freq));
    if (state.freq == normalized_freq) {
        return;
    }

    state.freq = normalized_freq;
    state.freq_id = get_calibration_freq_id(state.freq);
    state.dac_changed = true;
}

void set_vdac_step(MainLoopState& state, int vdac_step)
{
    if (vdac_step < 1) {
        vdac_step = 1;
    }

    if (state.vdac_id == vdac_step) {
        return;
    }

    state.vdac_id = vdac_step;
    state.vdac_pp = state.vdac_id * 0.1;
    update_dc_bias_offset(state);
    state.dac_changed = true;
}

void update_dc_bias_offset(MainLoopState& state)
{
    state.vdac_dc = state.dc_bias ? state.vdac_pp / 2.0 : 0.0;
}

void initialize_encoder_positions(const MainLoopState& state)
{
    initialize_frequency_encoder_position(state);

    // TIM3 selects DAC output level. INT16_MAX is used as the center offset so the
    // encoder can move below/above the initial position without immediate underflow.
    TIM3->CNT = INT16_MAX + 4 * state.vdac_id;
}

void initialize_frequency_encoder_position(const MainLoopState& state)
{
    // TIM4 selects measurement frequency. One encoder detent corresponds to 4 timer counts.
    TIM4->CNT = 4 * selected_frequency_to_encoder_step(state, state.freq);
}

int read_frequency_encoder(const MainLoopState& state)
{
    return encoder_step_to_selected_frequency(state, read_frequency_encoder_step());
}

int read_frequency_encoder_step()
{
    return (int32_t)TIM4->CNT / 4;
}

int read_vdac_encoder_step()
{
    return ((int32_t)TIM3->CNT - INT16_MAX) / 4;
}

int encoder_step_to_selected_frequency(const MainLoopState& state, int step_id)
{
    if (state.freq_selection_mode == FrequencySelectionMode::CalibrationOnly) {
        return calibration_frequency_step_to_freq(step_id);
    }
    return all_frequency_step_to_freq(step_id);
}

int selected_frequency_to_encoder_step(const MainLoopState& state, int freq)
{
    if (state.freq_selection_mode == FrequencySelectionMode::CalibrationOnly) {
        return freq_to_calibration_frequency_step(freq);
    }
    return freq_to_all_frequency_step(freq);
}

int all_frequency_step_to_freq(int step_id)
{
    int step_count = all_frequency_step_count();
    step_id %= step_count;
    if (step_id < 0) {
        step_id += step_count;
    }

    for (const FrequencyStepRange& range : measurement_frequency_ranges) {
        for (int freq = range.first_freq_hz; freq <= range.last_freq_hz; freq += range.step_hz) {
            if (is_avoided_measurement_frequency(freq)) {
                continue;
            }

            if (step_id == 0) {
                return freq;
            }
            --step_id;
        }
    }

    return measurement_frequency_ranges[0].first_freq_hz;
}

int freq_to_all_frequency_step(int freq)
{
    int step_count = all_frequency_step_count();
    int best_step = 0;
    int best_error = abs(freq - all_frequency_step_to_freq(0));

    for (int step = 1; step < step_count; ++step) {
        int error = abs(freq - all_frequency_step_to_freq(step));
        if (error < best_error) {
            best_error = error;
            best_step = step;
        }
    }

    return best_step;
}

int all_frequency_step_count()
{
    int total = 0;
    for (const FrequencyStepRange& range : measurement_frequency_ranges) {
        total += count_frequency_steps_in_range(range);
    }
    return total;
}

int count_frequency_steps_in_range(const FrequencyStepRange& range)
{
    int count = 0;
    for (int freq = range.first_freq_hz; freq <= range.last_freq_hz; freq += range.step_hz) {
        if (!is_avoided_measurement_frequency(freq)) {
            ++count;
        }
    }
    return count;
}

bool is_avoided_measurement_frequency(int freq)
{
    for (int avoided_freq : avoided_measurement_frequencies) {
        if (freq == avoided_freq) {
            return true;
        }
    }
    return false;
}

int calibration_frequency_step_to_freq(int step_id)
{
    int count = get_calibration_freq_count();
    step_id %= count;
    if (step_id < 0) {
        step_id += count;
    }
    return get_calibration_freq(step_id);
}

int freq_to_calibration_frequency_step(int freq)
{
    return get_nearest_calibration_freq_id(freq);
}

void toggle_frequency_selection_mode(MainLoopState& state)
{
    state.freq_selection_mode = (state.freq_selection_mode == FrequencySelectionMode::CalibrationOnly)
                                    ? FrequencySelectionMode::AllFrequencies
                                    : FrequencySelectionMode::CalibrationOnly;

    set_measurement_frequency(state, state.freq);
    initialize_frequency_encoder_position(state);
}

void latch_button_event(int index, bool is_pressed, uint32_t event_mask)
{
    if (is_pressed && !buttons.previous[index]) {
        buttons.pending_events |= event_mask;
    }
    buttons.previous[index] = is_pressed;
}

bool read_compensate_button_pressed()
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET;
}

bool read_coupling_button_pressed()
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET;
}

bool read_backlight_button_pressed()
{
    return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET;
}

bool read_frequency_mode_button_pressed()
{
    return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_RESET;
}

void initialize_button_state()
{
    ScopedLock lock;
    buttons.previous[0] = read_compensate_button_pressed();
    buttons.previous[1] = read_coupling_button_pressed();
    buttons.previous[2] = read_backlight_button_pressed();
    buttons.previous[3] = read_frequency_mode_button_pressed();
    buttons.pending_events = 0;
}

void update_button()
{
    latch_button_event(0, read_compensate_button_pressed(), BUTTON_EVENT_COMPENSATE);
    latch_button_event(1, read_coupling_button_pressed(), BUTTON_EVENT_COUPLING);
    latch_button_event(2, read_backlight_button_pressed(), BUTTON_EVENT_BACKLIGHT);
    latch_button_event(3, read_frequency_mode_button_pressed(), BUTTON_EVENT_FREQ_MODE);
}

uint32_t consume_button_events()
{
    ScopedLock lock;
    uint32_t events = buttons.pending_events;
    buttons.pending_events = 0;
    return events;
}

void callback_1ms()
{
    if (!button_scan_enabled) {
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
