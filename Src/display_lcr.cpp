#include <stm32h7xx.h>

#include <stdio.h>
#include <cmath>

#include "utils.hpp"
#include "lcd.hpp"
#include "app_shared.hpp"

namespace
{
SMR12864 lcd;
}

void initialize_display()
{
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
}

void display_error(const char* message)
{
    lcd.cls();
    lcd.printf("%s", message);
}

void display_measurement_result(const MainLoopState& state, const RangeState& range, const MeasurementResult& result, double battery_voltage)
{
    if (abs(result.capacitance) > 1.0e-8) {
        printf("Freq: %d Hz, R: %.4f Ohm, L: %.4f uH, C: %.4f uF, Z: %.4f Ohm, BAT: %.3f V\n",
            state.freq, result.resistance, result.inductance * 1.0e+6, result.capacitance * 1.0e+6, result.impedance.abs, battery_voltage);
    } else {
        printf("Freq: %d Hz, R: %.4f Ohm, L: %.4f uH, C: %.5f pF, Z: %.4f Ohm, BAT: %.3f V\n",
            state.freq, result.resistance, result.inductance * 1.0e+6, result.capacitance * 1.0e+12, result.impedance.abs, battery_voltage);
    }


    lcd.cls();

    if (state.freq < 1000) {
        lcd.printf("%4dHz", state.freq);
    } else if (state.freq < 10000) {
        lcd.printf("%4.2fk", state.freq / 1000.0);
    } else if (state.freq < 100000) {
        lcd.printf("%4.1fk", state.freq / 1000.0);
    } else if (state.freq < 1000000) {
        lcd.printf("%4dk", state.freq / 1000);
    } else {
        lcd.printf("%4.1fM", state.freq / 1000000.0);
    }

    lcd.printf(" %4.2fVpp %s %s",
        state.vdac_pp,
        state.dc_couple ? (state.dc_bias ? "DCPB" : "DC") : "AC",
        state.freq_selection_mode == FrequencySelectionMode::CalibrationOnly ? "CAL" : "ALL");

    lcd.locate(1, 6);
    lcd.printf("B:%3.1fV  Z:", battery_voltage);
    if (result.impedance.abs > 1e+11) {
        lcd.printf(" ---- $");
    } else if (result.impedance.abs > 1e+9) {
        lcd.printf("%6.2fG$", result.impedance.abs / 1e+9);
    } else if (result.impedance.abs > 1e+8) {
        lcd.printf("%6.0fM$", result.impedance.abs / 1e+6);
    } else if (result.impedance.abs > 1e+6) {
        lcd.printf("%6.2fM$", result.impedance.abs / 1e+6);
    } else if (result.impedance.abs > 1e+5) {
        lcd.printf("%6.4fM$", result.impedance.abs / 1e+6);
    } else if (result.impedance.abs > 1e+4) {
        lcd.printf("%6.2fk$", result.impedance.abs / 1e+3);
    } else if (result.impedance.abs > 1e+2) {
        lcd.printf("%6.1f $", result.impedance.abs);
    } else if (result.impedance.abs > 1e+1) {
        lcd.printf("%6.3f $", result.impedance.abs);
    } else if (result.impedance.abs > 1) {
        lcd.printf("%6.2f $", result.impedance.abs);
    } else {
        lcd.printf("%6.1fm$", result.impedance.abs * 1000);
    }

    lcd.locate(2, 6);
    lcd.printf("TIA:%d PGA:Ix%d,Vx%d", range.tia_gain_id, get_pga_gain_display(range.pga_i_gain_id), get_pga_gain_display(range.pga_v_gain_id));

    lcd.locate(3, 6);
    lcd.set_fontsize(16);
    if (result.capacitance > -10e-15 && result.capacitance < 1.0e-1) {
        lcd.printf("%s", result.sp_mode ? "Cs" : "Cp");
        if (result.capacitance > 1.0e-3) {
            lcd.printf("%7.0fuF", result.capacitance * 1.0e+6);
        } else if (result.capacitance > 1.0e-4) {
            lcd.printf("%6.1fuF", result.capacitance * 1.0e+6);
        } else if (result.capacitance > 1.0e-6) {
            lcd.printf("%6.2fuF", result.capacitance * 1.0e+6);
        } else if (result.capacitance > 1.0e-7) {
            lcd.printf("%6.4fuF", result.capacitance * 1.0e+6);
        } else if (result.capacitance > 1.0e-8) {
            lcd.printf("%6.2fnF", result.capacitance * 1.0e+9);
        } else if (result.capacitance > 1.0e-9) {
            lcd.printf("%6.2fpF", result.capacitance * 1.0e+12);
        } else if (result.capacitance > 1.0e-11) {
            lcd.printf("%6.2fpF", result.capacitance * 1.0e+12);
        } else if (result.capacitance > 1.0e-12 || state.freq < 100000) {
            lcd.printf("%6.3fpF", result.capacitance * 1.0e+12);
        } else {
            lcd.printf("%6.1ffF", result.capacitance * 1.0e+15);
        }
    }

    if (result.inductance > 0 && result.inductance < 1.0e-1) {
        lcd.printf("%s", result.sp_mode ? "Ls" : "Lp");
        if (result.inductance > 1.0e-2) {
            lcd.printf("%6.3fmH", result.inductance * 1.0e+3);
        } else if (result.inductance > 1.0e-3) {
            lcd.printf("%6.1fuH", result.inductance * 1.0e+6);
        } else if (result.inductance > 1.0e-4) {
            lcd.printf("%6.2fuH", result.inductance * 1.0e+6);
        } else if (result.inductance > 1.0e-6 || state.freq < 100000) {
            lcd.printf("%6.3fuH", result.inductance * 1.0e+6);
        } else {
            lcd.printf("%6.1fnH", result.inductance * 1.0e+9);
        }
    }

    lcd.locate(5, 6);
    lcd.printf("%s", result.sp_mode ? "Rs" : "Rp");
    if (result.resistance < -0.1 || result.resistance > 1e+9) {
        lcd.printf(" ---- $");
    } else if (result.resistance > 1e+8) {
        lcd.printf("%6.1fM$", result.resistance / 1e+6);
    } else if (result.resistance > 1e+7) {
        lcd.printf("%5.2fM$", result.resistance / 1e+6);
    } else if (result.resistance > 1e+6) {
        lcd.printf("%5.3fM$", result.resistance / 1e+6);
    } else if (result.resistance > 1e+5) {
        lcd.printf("%5.4fM$", result.resistance / 1e+6);
    } else if (result.resistance > 1e+4) {
        lcd.printf("%5.2fk$", result.resistance / 1e+3);
    } else if (result.resistance > 1e+3) {
        lcd.printf("%5.1f $", result.resistance);
    } else if (result.resistance > 1e+1) {
        lcd.printf("%5.2f $", result.resistance);
    } else if (result.resistance > 1) {
        lcd.printf("%5.4f $", result.resistance);
    } else {
        lcd.printf("%5.1fm$", result.resistance * 1000);
    }

    lcd.locate(7, 12);
    lcd.set_fontsize(8);
    lcd.printf("Q = ");
    double q = abs(result.impedance.im / result.impedance.real);
    if (q > 1e+2) {
        lcd.printf("%.1f", q);
    } else if (q > 1e+1) {
        lcd.printf("%.2f", q);
    } else {
        lcd.printf("%5.3f", q);
    }

    double theta_deg = 360.0 / (2 * PI) * atan2(abs(result.impedance.im), abs(result.impedance.real));
    lcd.printf(" %6.3fdeg", theta_deg);
}
