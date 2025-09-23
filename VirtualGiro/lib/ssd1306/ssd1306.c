#include "ssd1306.h"

static void ssd1306_command(ssd1306_t *dev, uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    i2c_write_blocking(dev->i2c, SSD1306_ADDR, buf, 2, false);
}

void ssd1306_init(ssd1306_t *dev, i2c_inst_t *i2c, uint sda, uint scl) {
    dev->i2c = i2c;
    dev->sda = sda;
    dev->scl = scl;
    memset(dev->buffer, 0, sizeof(dev->buffer));

    i2c_init(dev->i2c, 400000);
    gpio_set_function(dev->sda, GPIO_FUNC_I2C);
    gpio_set_function(dev->scl, GPIO_FUNC_I2C);
    gpio_pull_up(dev->sda);
    gpio_pull_up(dev->scl);
    sleep_ms(100);

    ssd1306_command(dev, 0xAE);
    ssd1306_command(dev, 0xD5); ssd1306_command(dev, 0x80);
    ssd1306_command(dev, 0xA8); ssd1306_command(dev, SSD1306_HEIGHT - 1);
    ssd1306_command(dev, 0xD3); ssd1306_command(dev, 0x00);
    ssd1306_command(dev, 0x40);
    ssd1306_command(dev, 0x8D); ssd1306_command(dev, 0x14);
    ssd1306_command(dev, 0x20); ssd1306_command(dev, 0x00);
    ssd1306_command(dev, 0xA1);
    ssd1306_command(dev, 0xC8);
    ssd1306_command(dev, 0xDA); ssd1306_command(dev, 0x12);
    ssd1306_command(dev, 0x81); ssd1306_command(dev, 0xCF);
    ssd1306_command(dev, 0xD9); ssd1306_command(dev, 0xF1);
    ssd1306_command(dev, 0xDB); ssd1306_command(dev, 0x40);
    ssd1306_command(dev, 0xA4);
    ssd1306_command(dev, 0xA6);
    ssd1306_command(dev, 0xAF);
}

void ssd1306_clear(ssd1306_t *dev) {
    memset(dev->buffer, 0, sizeof(dev->buffer));
}

void ssd1306_show(ssd1306_t *dev) {
    for (uint8_t page = 0; page < SSD1306_HEIGHT / 8; page++) {
        ssd1306_command(dev, 0xB0 + page);
        ssd1306_command(dev, 0x00);
        ssd1306_command(dev, 0x10);

        uint8_t data[SSD1306_WIDTH + 1];
        data[0] = 0x40;
        memcpy(&data[1], &dev->buffer[page * SSD1306_WIDTH], SSD1306_WIDTH);
        i2c_write_blocking(dev->i2c, SSD1306_ADDR, data, SSD1306_WIDTH + 1, false);
    }
}

void ssd1306_draw_pixel(ssd1306_t *dev, uint8_t x, uint8_t y, bool color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    if (color)
        dev->buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    else
        dev->buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
}

void ssd1306_draw_char(ssd1306_t *dev, uint8_t x, uint8_t y, char c) {
    if (c < 32 || c > 127) return;
    const uint8_t *ch = font6x8[c - 32];
    for (int i = 0; i < 6; i++) {
        uint8_t line = ch[i];
        for (int j = 0; j < 8; j++) {
            if (x + i < SSD1306_WIDTH && y + j < SSD1306_HEIGHT) {
                ssd1306_draw_pixel(dev, x + i, y + j, line & (1 << j));
            }
        }
    }
}

void ssd1306_draw_string(ssd1306_t *dev, uint8_t x, uint8_t y, const char *str) {
    while (*str) {
        ssd1306_draw_char(dev, x, y, *str++);
        x += 6;
        if (x + 6 > SSD1306_WIDTH) {
            x = 0;
            y += 8;
        }
        if (y + 8 > SSD1306_HEIGHT) break;
    }
}
