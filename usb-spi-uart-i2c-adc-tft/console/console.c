
/* Author, Copyright: Oleg Borodin <onborodin@gmail.com> 2018 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "../fonts/font_fixedsys_mono_24.h"
//#include "../fonts/lcd-debug-black.h"
#include "../tft_stm32_spi.h"
#include "console.h"
#include "font8x14.h"

font_t basefontbm = {
    .width = 8,
    .height = 14,
    .start = 32,
    .length = 0x5F,
    .bitmap = basefont_bitmap
};

//uint8_t console_buffer[CONSOLE_WIDTH * CONSOLE_HEIGHT];

console_t consolebm = {
    .width = CONSOLE_WIDTH,
    .height = CONSOLE_HEIGHT,
    .line = 0,
    .row = 0,
    .xmax = 127,
    .ymax = 127,
    .xshift = 0,
    .yshift = 0,
    .font = &basefontbm,
//    .buffer = console_buffer,
    .buffer_len = CONSOLE_WIDTH * CONSOLE_HEIGHT + 1
};

void _console_setup(console_t *console, uint16_t xmax, uint16_t ymax, font_t *font) {

    console->xmax = xmax;
    console->ymax = ymax;
    console->row = 0;
    console->line = 0;
    console->xshift = 0;
    console->yshift = 0;
    console->font = font;
    console->width = ymax/font->width;
    console->height = xmax/font->height;
    console->buffer_len = console->width * console->height + 1;
    console->buffer = malloc(console->buffer_len);
}

void console_setup(void) {
    _console_setup(&consolebm, LCD_TFTWIDTH, LCD_TFTHEIGHT, &basefontbm);
}

void console_render_char(console_t *console, uint8_t line, uint8_t row) {
    uint8_t c = console->buffer[(console->width * line) + row];
    lcd_draw_char(
        (console->xmax - (console->font->height * (line + 1))) + console->yshift,
        (console->font->width * row) + console->xshift,
        console->font, c);
}

uint16_t lcd_rgb2c(uint32_t rgb) {
#if 1
    return (uint16_t)
        (((rgb & 0xF80000) >> 19) |
         ((rgb & 0x00FC00 ) >> 5) |
         ((rgb & 0x0000F8) << 8));
#else
    return (uint16_t)
        (((rgb & 0xF80000) >> 11) |
         ((rgb & 0x00FC00 ) >> 5) |
         ((rgb & 0x0000F8) >> 3));
#endif
}


void lcd_draw_char(uint16_t xbase, uint16_t ybase, font_t *font, uint8_t c) {
    if (c < font->start || c > (font->start + font->length))
        c = ' ';
    c = c - font->start;
    _st_write_command_16bit(ST7789_MADCTL);
    _st_write_data_16bit(ST7789_MADCTL_MX | ST7789_MADCTL_BGR);

    uint16_t fg = lcd_rgb2c(ST_COLOR_WHITE);
    uint16_t bg = lcd_rgb2c(ST_COLOR_BLACK);

    st_set_address_window(xbase, ybase, xbase + font->height - 1, ybase + font->width - 1);

    _st_write_command_16bit(ST7789_RAMWR);

    for (uint8_t w = font->width; w > 0; w--) {
        for (uint8_t h = font->height; h > 0; h--) {
            if ((font->bitmap[(c) * font->height + (h - 1)]) & (1 << (w - 1)))
                _st_write_data_16bit(0xEFEF);
            else
                _st_write_data_16bit(0x0000);
        }
    }
}


void console_render(console_t *console) {
#if 0
    for (uint8_t line = console->height; line > 0; --line) {
        for (uint8_t row = console->width; row > 0; --row) {
            console_render_char(console, line - 1, row - 1);
        }
    }
#endif
//dma_start(console->buffer, console->buffer_len);

    for (uint8_t line = 0; line < console->height; line++) {
        for (uint8_t row = 0; row < console->width; row++) {
            console_render_char(console, line, row);
        }
    }

}

void console_shift(console_t *console) {
    uint16_t i = 0;
    uint16_t pos = console->width * (console->line - 1) + console->row;
    uint16_t end = console->width * console->height;

    while (i < (end - console->width)) {
        console->buffer[i] = console->buffer[i + console->width];
        i++;
    }
    while (i < end) {
        console->buffer[i] = ' ';
        i++;
    }

    if (console->line > 0)
        console->line--;
    else
        console->line = 0;
}

void console_putc(console_t *console, uint8_t c) {

    if ((console->row + 1) > console->width) {
        console->line++;
        console->row = 0;
    }

    if (c == '\n') {
        console->line++;
        console->row = 0;
    }

    if (console->line >= console->height) {
        console_shift(console);
        console_render(console);
    }
    console->buffer[(console->line * console->width) + console->row] = c;
    console_render_char(console, console->line, console->row);
    console->row++;
}

int console_puts(console_t *console, uint8_t *str) {
    uint8_t i = 0;
    while (str[i] != 0) {
        console_putc(console, str[i]);
        i++;
    }
//    st_draw_bitmap_nodma(385, 95, &LCDDEBUGBLACK);
//    st_draw_string(385, 95, "LCDDBG", menu1clr, &font_fixedsys_mono_24);
//	st_draw_string(385, 125, "LCDI2C", menu2clr, &font_fixedsys_mono_24);
//    dma_start((void *)console->buffer, i);
    return i;
}

void console_render_xychar(console_t *console, uint8_t line, uint8_t row, uint8_t c) {
    lcd_draw_char(
        (console->xmax - (console->font->height * (line + 1))) + console->yshift,
        (console->font->width * row) + console->xshift,
        console->font, c);
}

void console_xyputc(console_t *console, uint16_t line, uint16_t row, uint8_t c) {
    if (row < console->width && line < console->height) {
        console_render_xychar(console, line, row, c);
    }
}

int console_xyputs(console_t *console, uint16_t line, uint16_t row, uint8_t *str) {
    uint8_t i = 0;
    while (str[i] != 0 && row < console->width && line < console->height) {
        console_render_xychar(console, line, row, str[i]);
        i++;
        row++;
    }
    return i;
}

/* EOF */
