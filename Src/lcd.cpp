#include "lcd.hpp"
#include "utils.hpp"
#include "gpio.h"

void SMR12864::set_bus(uint8_t val)
{
    GPIOD->BSRR = val;
    GPIOD->BSRR = ((~val) & 0xFF) << 16;
}

void SMR12864::set_cs(uint8_t chip_id, bool state)
{
    if (chip_id == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void SMR12864::set_rw(bool state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SMR12864::set_rs(bool state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SMR12864::set_e(bool state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SMR12864::set_rst(bool state)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SMR12864::locate(uint32_t x, uint32_t y)
{
    pos_x = x;
    pos_y = y;
}

void SMR12864::write_pixel(uint32_t addr_x, uint32_t addr_y, const uint8_t* data, uint32_t len)
{
    if (addr_x >= lcd_height || addr_y >= lcd_width) {
        ::printf("Error in write_pixel\n");
        return;
    }

    // truncate at LCD boundary
    if (addr_y + len >= lcd_width) {
        len = lcd_width - addr_y;
    }

    // process chip boundary
    uint32_t len_rest = 0;
    if (addr_y < chip_width && addr_y + len >= chip_width) {
        len_rest = len - (chip_width - addr_y);
        len = chip_width - addr_y;
    }

    // Address set
    uint8_t chip_id = (addr_y >= chip_width);
    uint8_t command_data = 0xB8 | (addr_x && 0x07);
    write_command(chip_id, command_data);
    command_data = 0x40 | (addr_y & 0x3F);
    write_command(chip_id, command_data);

    // Pixel write
    for (uint32_t i = 0; i < len; ++i) {
        write_data(chip_id, data[i]);
    }

    if (len_rest > 0) {
        write_pixel(addr_x, chip_width, data + len, len_rest);
    }
}

extern const uint8_t lcd_font8_map1[96][5];       // ASCII
extern const uint8_t lcd_font8_map2[64][5];       // Katakana
extern const uint8_t lcd_font16_map1[96][2][12];  // ASCII
extern const uint8_t lcd_font32_map1[96][4][24];  // ASCII

int SMR12864::putc(uint8_t ch)
{
    if (font_size == 8) {
        if (0x20 <= ch && ch <= 0x7F) {
            write_pixel(pos_x, pos_y, lcd_font8_map1[ch - 0x20], 5);
            pos_y += 6;
            return ch;
        } else if (0xA0 <= ch && ch <= 0xDF) {
            write_pixel(pos_x, pos_y, lcd_font8_map2[ch - 0xA0], 5);
            pos_y += 6;
            return ch;
        }
    } else if (font_size == 16) {
        if (0x20 <= ch && ch <= 0x7F) {
            write_pixel(pos_x, pos_y + 0, lcd_font16_map1[ch - 0x20][0], 12);
            write_pixel(pos_x, pos_y + 1, lcd_font16_map1[ch - 0x20][1], 12);
            pos_y += 12;
            return ch;
        }
    } else if (font_size == 32) {
        if (0x20 <= ch && ch <= 0x7F) {
            write_pixel(pos_x, pos_y + 0, lcd_font32_map1[ch - 0x20][0], 24);
            write_pixel(pos_x, pos_y + 1, lcd_font32_map1[ch - 0x20][1], 24);
            write_pixel(pos_x, pos_y + 2, lcd_font32_map1[ch - 0x20][2], 24);
            write_pixel(pos_x, pos_y + 3, lcd_font32_map1[ch - 0x20][3], 24);
            pos_y += 24;
            return ch;
        }
    }
    return -1;
}

int SMR12864::printf(const char* format, ...)
{
    char buff[lcd_width];
    va_list args;
    va_start(args, format);
    int count = vsnprintf(buff, sizeof(buff), format, args);
    va_end(args);
    for (int i = 0; i < count; ++i) {
        this->putc(buff[i]);
    }
    return count;
}

void SMR12864::set_fontsize(uint32_t s)
{
    font_size = s;
}

SMR12864::SMR12864()
{
}

void SMR12864::reset()
{
    set_cs(0, false);
    set_cs(1, false);
    set_e(true);
    set_rw(false);
    set_rs(false);
    set_bus(0);

    set_rst(false);
    delay_ms(1);
    set_rst(true);
    delay_ms(30);

    write_command(0, 0xc0);  // set display start line
    write_command(1, 0xc0);
    write_command(0, 0x3f);  // set display on
    write_command(1, 0x3f);
}

void SMR12864::write_command(uint8_t chip_id, uint8_t data)
{
    if (chip_id == 0) {
        set_cs(0, true);
        set_cs(1, false);
    } else {
        set_cs(0, false);
        set_cs(1, true);
    }
    set_rs(false);
    set_e(false);
    set_bus(data);

    delay_us(1);
    set_e(true);
    delay_us(1);
}

void SMR12864::write_data(uint8_t chip_id, uint8_t data)
{
    if (chip_id == 0) {
        set_cs(0, true);
        set_cs(1, false);
    } else {
        set_cs(0, false);
        set_cs(1, true);
    }
    set_rs(true);
    set_e(false);
    set_bus(data);

    delay_us(1);
    set_e(true);
    delay_us(1);
}
