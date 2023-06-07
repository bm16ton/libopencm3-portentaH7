
/* Author, Copyright: Oleg Borodin <onborodin@gmail.com> 2018 */

#include "../tft_stm32_spi.h"

#ifndef CONSOLE_H_ITU
#define CONSOLE_H_ITU

typedef struct basefontbm {
    uint8_t width;
    uint8_t height;
    uint8_t start;
    uint8_t length;
    const uint8_t * bitmap;
} font_t;

typedef struct consolebm {
    uint8_t width;
    uint8_t height;
    uint8_t line;
    uint8_t row;
    uint16_t xmax;
    uint16_t ymax;
    uint16_t xshift;
    uint16_t yshift;
    font_t *font;
    uint8_t *buffer;
    uint16_t buffer_len;
} console_t;

#define LCD_TFTHEIGHT   480
#define LCD_TFTWIDTH    320
#define CONSOLE_WIDTH   (480/8)
#define CONSOLE_HEIGHT  (LCD_TFTWIDTH/14)

extern console_t consolebm;

void _console_setup(console_t *console, uint16_t xmax, uint16_t ymax, font_t *font);
void console_setup(void);
void console_render_char(console_t *console, uint8_t line, uint8_t row);
void console_render(console_t *console);
void console_shift(console_t *console);
void console_putc(console_t *console, uint8_t c);
int console_puts(console_t *console, uint8_t *str);
void console_render_xychar(console_t *console, uint8_t line, uint8_t row, uint8_t c);
void console_xyputc(console_t *console, uint16_t line, uint16_t row, uint8_t c);
int console_xyputs(console_t *console, uint16_t line, uint16_t row, uint8_t *str);
void lcd_draw_char(uint16_t xbase, uint16_t ybase, font_t *font, uint8_t c);
uint16_t lcd_rgb2c(uint32_t rgb);
#endif
