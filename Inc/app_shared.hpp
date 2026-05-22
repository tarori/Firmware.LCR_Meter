#pragma once

#include <stdint.h>
#include "complex.hpp"

enum ButtonEvent : uint32_t {
    BUTTON_EVENT_COMPENSATE = 1u << 0,
    BUTTON_EVENT_COUPLING = 1u << 1,
    BUTTON_EVENT_BACKLIGHT = 1u << 2,
    BUTTON_EVENT_FREQ_MODE = 1u << 3,
    BUTTON_EVENT_DC_BIAS = 1u << 4
};

enum class FrequencySelectionMode {
    CalibrationOnly,
    AllFrequencies
};

struct ProbeCompensation {
    double short_resistance = 0.0;
    double short_inductance = 0.0e-6;
    double open_resistance = 1.0e+24;
    double open_capacitance = 0.000e-12;
};

struct MainLoopState {
    int freq = 0;
    int freq_id = 0;
    int vdac_id = 20;
    bool dac_changed = true;
    bool dc_couple = true;
    bool dc_bias = false;
    bool lcd_backlight = false;
    FrequencySelectionMode freq_selection_mode = FrequencySelectionMode::CalibrationOnly;
    double vdac_pp = 2.0;
    double vdac_dc = 0.0;
    ProbeCompensation probe;
};

// One measurement transaction. A sweep command can generate this repeatedly for each frequency.
struct MeasurementRequest {
    int freq = 100000;
    int freq_id = 0;
    bool dc_couple = true;
};

struct RangeState {
    int tia_gain_id = 0;
    int pga_v_gain_id = 0;
    int pga_i_gain_id = 0;
};

struct MeasurementResult {
    Complex impedance;
    Complex conductance;
    bool sp_mode = true;
    double resistance = 0.0;
    double inductance = 0.0;
    double capacitance = 0.0;
};

// measurement_signal_source.cpp
bool initialize_adc();
bool initialize_dac_tim();
void update_signal_source(MainLoopState& state);
RangeState auto_range(int freq);
MeasurementResult measure_and_calculate(MainLoopState& state, const MeasurementRequest& request, const RangeState& range, uint32_t button_events);
void measure_frequency_sweep(MainLoopState& state, int start_freq, int stop_freq);
double read_battery_voltage();
int get_calibration_freq_count();
int get_calibration_freq(int freq_id);
int get_calibration_freq_id(int freq);
int get_nearest_calibration_freq_id(int freq);
int get_pga_gain_display(int gain_id);
void pga_calibration();
void dac_calibration();
void ac_couple_calibration();
void tia_set_gain(int gain_id);
void coupling_set_dc(bool cur, bool pot);
void set_backlight(bool state);

// display_lcr.cpp
void initialize_display();
void display_error(const char* message);
void display_measurement_result(const MainLoopState& state, const RangeState& range, const MeasurementResult& result, double battery_voltage);
