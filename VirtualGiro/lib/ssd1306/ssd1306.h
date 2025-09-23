#ifndef SSD1306_H
#define SSD1306_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ssd1306_font.h"
#include <stdbool.h>
#include <string.h>

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_ADDR 0x3C   // Endereço I2C padrão do SSD1306

typedef struct {
    i2c_inst_t *i2c;
    uint8_t sda, scl;
    uint8_t buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
} ssd1306_t;

void ssd1306_init(ssd1306_t *dev, i2c_inst_t *i2c, uint sda, uint scl);
void ssd1306_clear(ssd1306_t *dev);
void ssd1306_show(ssd1306_t *dev);
void ssd1306_draw_pixel(ssd1306_t *dev, uint8_t x, uint8_t y, bool color);
void ssd1306_draw_char(ssd1306_t *dev, uint8_t x, uint8_t y, char c);
void ssd1306_draw_string(ssd1306_t *dev, uint8_t x, uint8_t y, const char *str);

#endif
