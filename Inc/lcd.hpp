#pragma once

#include <stdint.h>
#include <stdarg.h>

class SMR12864
{
public:
    static const uint32_t lcd_width = 128;
    static const uint32_t chip_width = lcd_width / 2;
    static const uint32_t lcd_height = 64;
    static const bool chip_swap = true;
    uint32_t pos_x;
    uint32_t pos_y;
    uint32_t font_size;

    SMR12864();
    void reset();
    void cls();
    void locate(uint32_t x, uint32_t y);
    void set_fontsize(uint32_t s);
    void write_pixel(uint32_t addr_x, uint32_t addr_y, const uint8_t* data, uint32_t len);
    int putc(uint8_t ch);
    int printf(const char* format, ...);

private:
    void set_bus(uint8_t val);
    uint8_t get_bus();
    void set_bus_mode(bool is_out);
    void set_cs(uint8_t chip_id, bool state);
    void set_rw(bool state);
    void set_e(bool state);
    void set_rs(bool state);
    void set_rst(bool state);
    void write_command(uint8_t chip_id, uint8_t data);
    void write_data(uint8_t chip_id, uint8_t data);
    uint8_t read_status(uint8_t chip_id);
};
