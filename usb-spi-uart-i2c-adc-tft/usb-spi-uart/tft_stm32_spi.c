/*
MIT License

Copyright (c) 2020 Avra Mitra

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
//#include "timing_stm32.h"
#include "tft_stm32_spi.h"
#include "fonts/font_fixedsys_mono_24.h"
//#include "fonts/pic.h"
//#include "fonts/16ton.h"
//#include "fonts/ext-firm.h"
#include "fonts/bitmap_typedefs.h"
#include <math.h>
#include "fonts/font_ubuntu_48.h"
#include "ILI9486_Defines.h"
#include "cdc.h"
#include <libopencm3/stm32/h7/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/dmamux.h>
#include <libopencm3/stm32/memorymap.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "dcache.h"
#include <libopencm3/cm3/mpu.h>
#include "vars.h"
#include "debugen.h"
//#include "mpu_armv7.h"
//#define ST_BUFFER_SIZE_BYTES	256
//TFT width and height default global variables
uint16_t st_tftwidth = 480;
uint16_t st_tftheight = 320;
#define ST_BUFFER_SIZE_BYTES	1024
//static uint16_t ST_BUFFER_SIZE_BYTES = 10;
#define ILI9163C_A (480 * 320)
#define DMA_BUFFER \
    __attribute__((aligned (4)))   __attribute__((section(".ram2")))

DMA_BUFFER __attribute__((aligned (4))) uint16_t* FRAMEBUFFER[8192];
//FRAMEBUFFER[1024] = {0};
//__attribute__((aligned (16))) uint16_t FRAMEBUFFER[ILI9163C_A];
//uint16_t FRAMEBUFFER2[100];
uint16_t spot = 0;
uint64_t twaitm = 34;
void dma_start(void *tfttx, size_t data_size);
void tftdma(void);
void dma_setup(void);

//void st_draw_pixel2(uint16_t x, uint16_t y, uint16_t color);

/**
 * Set an area for drawing on the display with start row,col and end row,col.
 * User don't need to call it usually, call it only before some functions who don't call it by default.
 * @param x1 start column address.
 * @param y1 start row address.
 * @param x2 end column address.
 * @param y2 end row address.
 */

void st_set_address_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	_st_write_command_16bit(TFT_CASET);

#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_ACTIVE;
#endif
//	ST_DC_DAT;
	_st_write_data_16bit((uint8_t)(x1 >> 8));
	_st_write_data_16bit((uint8_t)x1);
	_st_write_data_16bit((uint8_t)(x2 >> 8));
	_st_write_data_16bit((uint8_t)x2);


	_st_write_command_16bit(ST7789_RASET);
#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_ACTIVE;
#endif
//	ST_DC_DAT;
	_st_write_data_16bit((uint8_t)(y1 >> 8));
	_st_write_data_16bit((uint8_t)y1);
	_st_write_data_16bit((uint8_t)(y2 >> 8));
	_st_write_data_16bit((uint8_t)y2);

	_st_write_command_16bit(ST7789_RAMWR);
}



/*
 * Render a character glyph on the display. Called by `_st_draw_string_main()`
 * User need NOT call it
 */
void _st_render_glyph(uint16_t x, uint16_t y, uint16_t fore_color, uint16_t back_color, const tImage *glyph, uint8_t is_bg)
{
	uint16_t width = 0, height = 0;

	width = glyph->width;
	height = glyph->height;

	uint16_t temp_x = x;
	uint16_t temp_y = y;

	uint8_t mask = 0x80;
	uint8_t bit_counter = 0;

	const uint8_t *glyph_data_ptr = (const uint8_t *)(glyph->data);
	uint8_t glyph_data = 0;

	// font bitmaps are stored in column major order (scanned from left-to-right, not the conventional top-to-bottom)
	// as font glyphs have heigher height than width, this scanning saves some storage.
	// So, we also render in left-to-right manner.

	// Along x axis (width)
	for (int i = 0; i < width; i++)
	{
		// Along y axis (height)
		for (int j = 0; j < height; j++)
		{

			// load new data only when previous byte (or word, depends on glyph->dataSize) is completely traversed by the mask
			// bit_counter = 0 means glyph_data is completely traversed by the mask
			if (bit_counter == 0)
			{
				glyph_data = *glyph_data_ptr++;
				bit_counter = glyph->dataSize;
			}
			// Decrement bit counter
			bit_counter--;

			//If pixel is blank
			if (glyph_data & mask)
			{
				//Has background color (not transparent bg)
				if (is_bg)
				{
					st_draw_pixel(temp_x, temp_y, back_color);
				}
			}

			//if pixel is not blank
			else
			{
				st_draw_pixel(temp_x, temp_y, fore_color);
			}

			glyph_data <<= 1;
			temp_y++;
		}

		//New col starts. So, row is set to initial value and col is increased by one
		temp_y = y;
		temp_x++;

		//Reset the bit counter cause we're moving to next column, so we start with a new byte
		bit_counter = 0;
	}
}



/**
 * Renders a string by drawing each character glyph from the passed string.
 * Called by `st_draw_string()` and `st_draw_string_withbg()`.
 * Text is wrapped automatically if it hits the screen boundary.
 * x_padding and y_padding defines horizontal and vertical distance (in px) between two characters
 * is_bg=1 : Text will habe background color,   is_bg=0 : Text will have transparent background
 * User need NOT call it.
 */

void _st_draw_string_main(uint16_t x, uint16_t y, char *str, uint16_t fore_color, uint16_t back_color, const tFont *font, uint8_t is_bg)
{
	uint16_t x_temp = x;
	uint16_t y_temp = y;

	uint8_t x_padding = 0;
	uint8_t y_padding = 0;
	const tImage *img = NULL;
	uint16_t width = 0, height = 0;



	while (*str)
	{
		if (*str == '\n')
		{
			x_temp = x;					//go to first col
			y_temp += (font->chars[0].image->height + y_padding);	//go to next row (row height = height of space)
		}

		else if (*str == '\t')
		{
			x_temp += 4 * (font->chars[0].image->height + y_padding);	//Skip 4 spaces (width = width of space)
		}
		else
		{
			for (uint8_t i = 0; i < font->length; i++)
			{
				if (font->chars[i].code == *str)
				{
					img = font->chars[i].image;
					break;
				}
			}
			// No glyph (img) found, so return from this function
			if (img == NULL)
			{
				return;
			}

			width = img->width;
			height = img->height;

			if(y_temp + (height + y_padding) > st_tftheight - 1)	//not enough space available at the bottom
				return;
			if (x_temp + (width + x_padding) > st_tftwidth - 1)	//not enough space available at the right side
			{
				x_temp = x;					//go to first col
				y_temp += (height + y_padding);	//go to next row
			}


			if (is_bg)
				_st_render_glyph(x_temp, y_temp, fore_color, back_color, img, 1);
			else
				_st_render_glyph(x_temp, y_temp, fore_color, back_color, img, 0);
			x_temp += (width + x_padding);		//next char position
		}


		str++;
	}
}


/**
 * Draws a character at a given position, fore color, back color.
 * @param x Start col address
 * @param y Start row address
 * @param character the ASCII character to be drawn
 * @param fore_color foreground color
 * @param back_color background color
 * @param font Pointer to the font of the character
 * @param is_bg Defines if character has background or not (transparent)
 */
void st_draw_char(uint16_t x, uint16_t y, char character, uint16_t fore_color, uint16_t back_color, const tFont *font, uint8_t is_bg)
{
	const tImage *img = NULL;
	for (uint8_t i = 0; i < font->length; i++)
	{
		if (font->chars[i].code == character)
		{
			img = font->chars[i].image;
			break;
		}
	}
	// No glyph (img) found, so return from this function
	if (img == NULL)
	{
		return;
	}

	if (is_bg)
		_st_render_glyph(x, y, fore_color, back_color, img, 1);
	else
		_st_render_glyph(x, y, fore_color, back_color, img, 0);
}


/**
 * Draws a string on the display with `font` and `color` at given position.
 * Background of this string is transparent
 * @param x Start col address
 * @param y Start y address
 * @param str pointer to the string to be drawn
 * @param color 16-bit RGB565 color of the string
 * @param font Pointer to the font of the string
 */
void st_draw_string(uint16_t x, uint16_t y, char *str, uint16_t color, const tFont *font)
{
	_st_draw_string_main(x, y, str, color, 0, font, 0);
}


/**
 * Draws a string on the display with `font`, `fore_color`, and `back_color` at given position.
 * The string has background color
 * @param x Start col address
 * @param y Start y address
 * @param str pointer to the string to be drawn
 * @param foe_color 16-bit RGB565 color of the string
 * @param back_color 16-bit RGB565 color of the string's background
 * @param font Pointer to the font of the string
 */
void st_draw_string_withbg(uint16_t x, uint16_t y, char *str, uint16_t fore_color, uint16_t back_color, const tFont *font)
{
	_st_draw_string_main(x, y, str, fore_color, back_color, font, 1);
}


/**
 * Draw a bitmap image on the display
 * @param x Start col address
 * @param y Start row address
 * @param bitmap Pointer to the image data to be drawn
 */

void st_draw_bitmap(uint16_t x, uint16_t y, const tImage *bitmap)
{
	uint16_t width = 0, height = 0;
	width = bitmap->width;
	height = bitmap->height;
	st_set_address_window(x, y, x + width-1, y + height-1);

#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_ACTIVE;
#endif
//	ST_DC_DAT;

#ifdef ST_USE_SPI_DMA
		uint32_t bytes_to_write = width * height * 2;
		uint16_t transfer_size = ST_BUFFER_SIZE_BYTES;
		uint32_t src_start_address = 0;

		while (bytes_to_write)
		{
			transfer_size = (bytes_to_write < transfer_size) ? bytes_to_write : transfer_size;
			dma_start((void *)(&bitmap->data[src_start_address]), transfer_size);
			src_start_address += ST_BUFFER_SIZE_BYTES;
			bytes_to_write -= transfer_size;
		}

#else

		uint32_t total_pixels = width * height;
		for (uint16_t pixels = 0; pixels < total_pixels; pixels++)
		{
			my_spi_send(SPI5, (uint8_t)(bitmap->data[2*pixels]));
			my_spi_send(SPI5, (uint8_t)(bitmap->data[2*pixels + 1]));
		}

	#endif

	#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_IDLE;
	#endif
}

void st_draw_bitmap_nodma(uint16_t x, uint16_t y, const tImage *bitmap)
{
	uint16_t width = 0, height = 0;
	width = bitmap->width;
	height = bitmap->height;
	st_set_address_window(x, y, x + width-1, y + height-1);

	#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_ACTIVE;
	#endif
	ST_DC_DAT;

		uint32_t total_pixels = width * height;
		for (uint16_t pixels = 0; pixels < total_pixels; pixels++)
		{
			my_spi_send(SPI5, (uint8_t)(bitmap->data[2*pixels]));
			my_spi_send(SPI5, (uint8_t)(bitmap->data[2*pixels + 1]));
		}

st_set_address_window(0, 0, st_tftwidth - 1, st_tftheight - 1);

	#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_IDLE;
	#endif
}

/**
 * Fills `len` number of pixels with `color`.
 * Call st_set_address_window() before calling this function.
 * @param color 16-bit RGB565 color value
 * @param len 32-bit number of pixels
 */

int poormansmutex = 0;
//__attribute__((aligned (32)))  uint8_t disp_buffer[0x48];


void st_fill_color(uint16_t color, uint32_t len)
{
#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_ACTIVE;
#endif
	uint8_t color_high = color >> 8;
	uint8_t color_low = color;

	#ifdef ST_USE_SPI_DMA
		uint8_t disp_buffer[ST_BUFFER_SIZE_BYTES];
		for (uint16_t i = 0; i <= (ST_BUFFER_SIZE_BYTES - 1); i = i+2)
		{
			disp_buffer[i] = color_high;
			disp_buffer[i + 1] = color_low;
		}

		// len is pixel count. But each pixel is 2 bytes. So, multiply by 2
		uint32_t bytes_to_write = len * 2;
		uint16_t transfer_size = ST_BUFFER_SIZE_BYTES;
		while (bytes_to_write)
		{
		while (poormansmutex == 1) {
		;
		}
			transfer_size = (bytes_to_write < transfer_size) ? bytes_to_write : transfer_size;
			dma_start(disp_buffer, transfer_size);
			bytes_to_write -= transfer_size;
		}

#else
		/*
		* Here, macros are directly called (instead of inline functions) for performance increase
		*/
		uint16_t blocks = (uint16_t)(len / 64); // 64 pixels/block
		uint8_t  pass_count;

		// Write first pixel
		my_spi_send(SPI5, color_high); my_spi_send(SPI5, color_low);
		len--;

		while(blocks--)
		{
			pass_count = 16;
			while(pass_count--)
			{
				my_spi_send(SPI5, color_high); my_spi_send(SPI5, color_low); 	my_spi_send(SPI5, color_high); my_spi_send(SPI5, color_low); //2
				my_spi_send(SPI5, color_high); my_spi_send(SPI5, color_low); 	my_spi_send(SPI5, color_high); my_spi_send(SPI5, color_low); //4
			}
		}
		pass_count = len & 63;
		while (pass_count--)
		{
			// write here the remaining data
			my_spi_send(SPI5, color_high); my_spi_send(SPI5, color_low);
		}

#endif

#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_IDLE;
#endif
}

void st_fill_color_nodma(uint16_t color, uint32_t len)
{
	#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_ACTIVE;
	#endif
	ST_DC_DAT;
	uint8_t color_high = color >> 8;
	uint8_t color_low = color;
		uint16_t blocks = (uint16_t)(len / 64); // 64 pixels/block
		uint8_t  pass_count;

		// Write first pixel
		my_spi_send(SPI5, color_high); my_spi_send(SPI5, color_low);
		len--;

		while(blocks--)
		{
			pass_count = 16;
			while(pass_count--)
			{
				my_spi_send(SPI5, color_high); my_spi_send(SPI5, color_low); 	my_spi_send(SPI5, color_high); my_spi_send(SPI5, color_low); //2
				my_spi_send(SPI5, color_high); my_spi_send(SPI5, color_low); 	my_spi_send(SPI5, color_high); my_spi_send(SPI5, color_low); //4
			}
		}
		pass_count = len & 63;
		while (pass_count--)
		{
			// write here the remaining data
			my_spi_send(SPI5, color_high); my_spi_send(SPI5, color_low);
		}
	#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_IDLE;
    #endif
}
/**
 * Fills `len` number of pixels with `color`.
 * Call st_set_address_window() before calling this function.
 * @param color_arr pointer to uint8_t array. Each 16-bit color is seperated into two 8-bit `high` and `low` components
 * @param bytes 32-bit number of bytes in the array (= no. of pixels x2)
 */

void st_fill_color_array(uint8_t *color_arr, uint32_t bytes)
{
#ifdef ST_RELEASE_WHEN_IDLE
	ST_CS_ACTIVE;
#endif
//	ST_DC_DAT;

#ifdef ST_USE_SPI_DMA

		// len is pixel count. But each pixel is 2 bytes. So, multiply by 2
		uint32_t bytes_to_write = bytes;
		uint16_t transfer_size = ST_BUFFER_SIZE_BYTES;
		while (bytes_to_write)
		{
			transfer_size = (bytes_to_write < transfer_size) ? bytes_to_write : transfer_size;
			dma_start(color_arr, transfer_size);
			bytes_to_write -= transfer_size;
		}

#else
		/*
		* Here, macros are directly called (instead of inline functions) for performance increase
*/
		uint32_t len = bytes / 2;
		uint16_t blocks = (uint16_t)(len / 64); // 64 pixels/block
		uint8_t  pass_count;

		// Write first pixel
		my_spi_send(SPI5, *color_arr); my_spi_send(SPI5, *(++color_arr));
		--len;

		while(blocks--)
		{
			pass_count = 16;
			while(pass_count--)
			{
				my_spi_send(SPI5, *(++color_arr)); my_spi_send(SPI5, *(++color_arr)); 	my_spi_send(SPI5, *(++color_arr)); my_spi_send(SPI5, *(++color_arr)); //2
				my_spi_send(SPI5, *(++color_arr)); my_spi_send(SPI5, *(++color_arr)); 	my_spi_send(SPI5, *(++color_arr)); my_spi_send(SPI5, *(++color_arr)); //4
			}
		}
		pass_count = len & 63;
		while (pass_count--)
		{
			// write here the remaining data
			my_spi_send(SPI5, *(++color_arr)); my_spi_send(SPI5, *(++color_arr));
		}

#endif

#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_IDLE;
#endif
}


/**
 * Fills a rectangular area with `color`.
 * Before filling, performs area bound checking
 * @param x Start col address
 * @param y Start row address
 * @param w Width of rectangle
 * @param h Height of rectangle
 * @param color 16-bit RGB565 color
 */
void st_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	if (x >= st_tftwidth || y >= st_tftheight || w == 0 || h == 0)
		return;
	if (x + w - 1 >= st_tftwidth)
		w = st_tftwidth - x;
	if (y + h - 1 >= st_tftheight)
		h = st_tftheight - y;

	st_set_address_window(x, y, x + w - 1, y + h - 1);
	st_fill_color(color, (uint32_t)w * (uint32_t)h);
}

void st_fill_rect_nodma(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	if (x >= st_tftwidth || y >= st_tftheight || w == 0 || h == 0)
		return;
	if (x + w - 1 >= st_tftwidth)
		w = st_tftwidth - x;
	if (y + h - 1 >= st_tftheight)
		h = st_tftheight - y;

	st_set_address_window(x, y, x + w - 1, y + h - 1);
	st_fill_color_nodma(color, (uint32_t)w * (uint32_t)h);
}
/*
 * Same as `st_fill_rect()` but does not do bound checking, so it's slightly faster
 */
void st_fill_rect_fast(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t color)
{
	st_set_address_window(x1, y1, x1 + w - 1, y1 + h - 1);
	st_fill_color(color, (uint32_t)w * (uint32_t)h);
}


/**
 * Fill the entire display (screen) with `color`
 * @param color 16-bit RGB565 color
 */
void st_fill_screen(uint16_t color)
{
	st_set_address_window(0, 0, st_tftwidth - 1, st_tftheight - 1);
	st_fill_color(color, (uint32_t)st_tftwidth * (uint32_t)st_tftheight);
}

void st_fill_screen_nodma(uint16_t color)
{
	st_set_address_window(0, 0, st_tftwidth - 1, st_tftheight - 1);
	st_fill_color_nodma(color, (uint32_t)st_tftwidth * (uint32_t)st_tftheight);
}

/**
 * Draw a rectangle
*/
void st_draw_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	// Perform bound checking
	if (x >= st_tftwidth || y >= st_tftheight || w == 0 || h == 0)
		return;
	if (x + w - 1 >= st_tftwidth)
		w = st_tftwidth - x;
	if (y + h - 1 >= st_tftheight)
		h = st_tftheight - y;

	_st_draw_fast_h_line(x, y, x+w-1, 1, color);
	_st_draw_fast_h_line(x, y+h, x+w-1, 1, color);
	_st_draw_fast_v_line(x, y, y+h-1, 1, color);
	_st_draw_fast_v_line(x+w, y, y+h-1, 1, color);
}

/*
 * Called by st_draw_line().
 * User need not call it
 */
void _st_plot_line_low(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t width, uint16_t color)
{
	int16_t dx = x1 - x0;
	int16_t dy = y1 - y0;
	int8_t yi = 1;
	uint8_t pixels_per_point = width * width;	//no of pixels making a point. if line width is 1, this var is 1. if 2, this var is 4 and so on
	uint8_t color_high = (uint8_t)(color >> 8);
	uint8_t color_low = (uint8_t)color;
	if (dy < 0)
	{
		yi = -1;
		dy = -dy;
	}

	int16_t D = 2*dy - dx;
	uint16_t y = y0;
	uint16_t x = x0;

	while (x <= x1)
	{
		st_set_address_window(x, y, x+width-1, y+width-1);
		//Drawing all the pixels of a single point

#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_ACTIVE;
#endif
//		ST_DC_DAT;
		for (uint8_t pixel_cnt = 0; pixel_cnt < pixels_per_point; pixel_cnt++)
		{
			my_spi_send(SPI5, color_high);
			my_spi_send(SPI5, color_low);
		}
#ifdef ST_RELEASE_WHEN_IDLE
	    ST_CS_IDLE;
#endif

		if (D > 0)
		{
			y = y + yi;
			D = D - 2*dx;
		}
		D = D + 2*dy;
		x++;
	}
}


/*
 * Called by st_draw_line().
 * User need not call it
 */
void _st_plot_line_high(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t width, uint16_t color)
{
	int16_t dx = x1 - x0;
	int16_t dy = y1 - y0;
	int8_t xi = 1;
	uint8_t pixels_per_point = width * width;	//no of pixels making a point. if line width is 1, this var is 1. if 2, this var is 4 and so on
	uint8_t color_high = (uint8_t)(color >> 8);
	uint8_t color_low = (uint8_t)color;

	if (dx < 0)
	{
		xi = -1;
		dx = -dx;
	}

	int16_t D = 2*dx - dy;
	uint16_t y = y0;
	uint16_t x = x0;

	while (y <= y1)
	{
		st_set_address_window(x, y, x+width-1, y+width-1);
		//Drawing all the pixels of a single point

#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_ACTIVE;
#endif
//		ST_DC_DAT;
		for (uint8_t pixel_cnt = 0; pixel_cnt < pixels_per_point; pixel_cnt++)
		{
			my_spi_send(SPI5, color_high);
			my_spi_send(SPI5, color_low);
		}
#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_IDLE;
#endif

		if (D > 0)
		{
			x = x + xi;
			D = D - 2*dy;
		}
		D = D + 2*dx;
		y++;
	}
}

/**
 * Draw a line from (x0,y0) to (x1,y1) with `width` and `color`.
 * @param x0 start column address.
 * @param y0 start row address.
 * @param x1 end column address.
 * @param y1 end row address.
 * @param width width or thickness of the line
 * @param color 16-bit RGB565 color of the line
 */
void st_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t width, uint16_t color)
{
	/*
	* Brehensen's algorithm is used.
	* Not necessarily start points has to be less than end points.
	*/

	if (x0 == x1)	//vertical line
	{
		_st_draw_fast_v_line(x0, y0, y1, width, color);
	}
	else if (y0 == y1)		//horizontal line
	{
		_st_draw_fast_h_line(x0, y0, x1, width, color);
	}

	else
	{
		if (abs(y1 - y0) < abs(x1 - x0))
		{
			if (x0 > x1)
				_st_plot_line_low(x1, y1, x0, y0, width, color);
			else
				_st_plot_line_low(x0, y0, x1, y1, width, color);
		}

		else
		{
			if (y0 > y1)
				_st_plot_line_high(x1, y1, x0, y0, width, color);
			else
				_st_plot_line_high(x0, y0, x1, y1, width, color) ;
		}
	}

}


/*
 * Called by st_draw_line().
 * User need not call it
 */
void _st_draw_fast_h_line(uint16_t x0, uint16_t y0, uint16_t x1, uint8_t width, uint16_t color)
{
	/*
	* Draw a horizontal line very fast
	*/
	if (x0 < x1) {
		st_set_address_window(x0, y0, x1, y0+width-1);	//as it's horizontal line, y1=y0.. must be.
	} else {
		st_set_address_window(x1, y0, x0, y0+width-1);
		if (lcddma == 1) {
	st_fill_color(color, (uint32_t)width * (uint32_t)abs(x1 - x0 + 1));
	    } else {
	        st_fill_color_nodma(color, (uint32_t)width * (uint32_t)abs(x1 - x0 + 1));
	    }
	}
}


/*
 * Called by st_draw_line().
 * User need not call it
 */
void _st_draw_fast_v_line(uint16_t x0, uint16_t y0, uint16_t y1, uint8_t width, uint16_t color)
{
	/*
	* Draw a vertical line very fast
	*/
	if (y0 < y1) {
		st_set_address_window(x0, y0, x0+width-1, y1);	//as it's vertical line, x1=x0.. must be.
	} else {
		st_set_address_window(x0, y1, x0+width-1, y0);
		if (lcddma == 1) {
	st_fill_color(color, (uint32_t)width * (uint32_t)abs(y1 - y0 + 1));
	    } else {
	        st_fill_color_nodma(color, (uint32_t)width * (uint32_t)abs(y1 - y0 + 1));
	    }
    }
}


/**
 * Draw a pixel at a given position with `color`
 * @param x Start col address
 * @param y Start row address
 */
void st_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
	/*
	* Why?: This function is mainly added in the driver so that  ui libraries can use it.
	* example: LittlevGL requires user to supply a function that can draw pixel
	*/

	st_set_address_window(x, y, x, y);
#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_ACTIVE;
#endif
//	ST_DC_DAT;
	my_spi_send(SPI5, (uint8_t)(color >> 8));
	my_spi_send(SPI5, (uint8_t)color);

#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_IDLE;
#endif
}
/*
void st_draw_pixel2(uint16_t x, uint16_t y, uint16_t color)
{

	st_set_address_window(x, y, x, y);
#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_ACTIVE;
#endif
//	ST_DC_DAT;
//	my_spi_send(SPI5, (uint8_t)(color >> 8));
//	my_spi_send(SPI5, (uint8_t)color);
    FRAMEBUFFER[spot] = ((uint8_t)(color >> 8)) ;
    FRAMEBUFFER[spot] = ((uint8_t)color);
    printf("spot = %d\r\n", spot);
#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_IDLE;
#endif
}
*/

void gfx_drawLine(int16_t x0, int16_t y0,
			    int16_t x1, int16_t y1,
			    uint16_t color)
{
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}
    spot = 0;
	for (; x0 <= x1; x0++) {
		if (steep) {
			st_draw_pixel(y0, x0, color);
		} else {
			st_draw_pixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
    spot = spot + 1;
	}
//printf("spot sending spot = %d\r\n", spot);
//	memset(FRAMEBUFFER, 0x00, spot);
//dma_start(FRAMEBUFFER2, spot);
//memset(FRAMEBUFFER2, 0x00, spot);
spot = 0;
}

/* Draw a circle outline */
void st_draw_circle(int16_t x0, int16_t y0, int16_t r,
		    uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	st_draw_pixel(x0  , y0+r, color);
	st_draw_pixel(x0  , y0-r, color);
	st_draw_pixel(x0+r, y0  , color);
	st_draw_pixel(x0-r, y0  , color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		st_draw_pixel(x0 + x, y0 + y, color);
		st_draw_pixel(x0 - x, y0 + y, color);
		st_draw_pixel(x0 + x, y0 - y, color);
		st_draw_pixel(x0 - x, y0 - y, color);
		st_draw_pixel(x0 + y, y0 + x, color);
		st_draw_pixel(x0 - y, y0 + x, color);
		st_draw_pixel(x0 + y, y0 - x, color);
		st_draw_pixel(x0 - y, y0 - x, color);
	}
}

void st_draw_circle_helper(int16_t x0, int16_t y0,
			  int16_t r, uint8_t cornername, uint16_t color)
{
	int16_t f     = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x     = 0;
	int16_t y     = r;

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f     += ddF_y;
		}
		x++;
		ddF_x += 2;
		f     += ddF_x;
		if (cornername & 0x4) {
			st_draw_pixel(x0 + x, y0 + y, color);
			st_draw_pixel(x0 + y, y0 + x, color);
		}
		if (cornername & 0x2) {
			st_draw_pixel(x0 + x, y0 - y, color);
			st_draw_pixel(x0 + y, y0 - x, color);
		}
		if (cornername & 0x8) {
			st_draw_pixel(x0 - y, y0 + x, color);
			st_draw_pixel(x0 - x, y0 + y, color);
		}
		if (cornername & 0x1) {
			st_draw_pixel(x0 - y, y0 - x, color);
			st_draw_pixel(x0 - x, y0 - y, color);
		}
	}
}

void st_fill_circle(int16_t x0, int16_t y0, int16_t r,
		    uint16_t color)
{
	gfx_drawFastVLine(x0, y0 - r, 2*r+1, color);
	st_fill_circle_helper(x0, y0, r, 3, 0, color);
}

/* Used to do circles and roundrects */
void st_fill_circle_helper(int16_t x0, int16_t y0, int16_t r,
			  uint8_t cornername, int16_t delta, uint16_t color)
{
	int16_t f     = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x     = 0;
	int16_t y     = r;

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f     += ddF_y;
		}
		x++;
		ddF_x += 2;
		f     += ddF_x;

		if (cornername & 0x1) {
			gfx_drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
			gfx_drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
		}
		if (cornername & 0x2) {
			gfx_drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
			gfx_drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
		}
	}
}

/* Draw a rounded rectangle */
void st_draw_round_rect(int16_t x, int16_t y, int16_t w,
		       int16_t h, int16_t r, uint16_t color)
{
	/* smarter version */
	gfx_drawFastHLine(x + r    , y        , w - 2 * r, color); /* Top */
	gfx_drawFastHLine(x + r    , y + h - 1, w - 2 * r, color); /* Bottom */
	gfx_drawFastVLine(x        , y + r    , h - 2 * r, color); /* Left */
	gfx_drawFastVLine(x + w - 1, y + r    , h - 2 * r, color); /* Right */
	/* draw four corners */
	st_draw_circle_helper(x + r        , y + r        , r, 1, color);
	st_draw_circle_helper(x + w - r - 1, y + r        , r, 2, color);
	st_draw_circle_helper(x + w - r - 1, y + h - r - 1, r, 4, color);
	st_draw_circle_helper(x + r        , y + h - r - 1, r, 8, color);
}

/* Fill a rounded rectangle */
void st_fill_round_rect(int16_t x, int16_t y, int16_t w,
		       int16_t h, int16_t r, uint16_t color) {
	/* smarter version */
	st_fill_rect(x + r, y, w - 2 * r, h, color);

	/* draw four corners */
	st_fill_circle_helper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1, color);
	st_fill_circle_helper(x + r        , y + r, r, 2, h - 2 * r - 1, color);
}

/* Draw a triangle */
void st_draw_triangle(int16_t x0, int16_t y0,
		      int16_t x1, int16_t y1,
		      int16_t x2, int16_t y2, uint16_t color)
{
	gfx_drawLine(x0, y0, x1, y1, color);
	gfx_drawLine(x1, y1, x2, y2, color);
	gfx_drawLine(x2, y2, x0, y0, color);
}

/* Fill a triangle */
void st_fill_triangle(int16_t x0, int16_t y0,
		      int16_t x1, int16_t y1,
		      int16_t x2, int16_t y2, uint16_t color)
{
	int16_t a, b, y, last;

	/* Sort coordinates by Y order (y2 >= y1 >= y0) */
	if (y0 > y1) {
		swap(y0, y1); swap(x0, x1);
	}
	if (y1 > y2) {
		swap(y2, y1); swap(x2, x1);
	}
	if (y0 > y1) {
		swap(y0, y1); swap(x0, x1);
	}

	/* Handle awkward all-on-same-line case as its own thing */
	if (y0 == y2) {
		a = b = x0;
		if (x1 < a) {
			a = x1;
		} else if (x1 > b) {
			b = x1;
		}
		if (x2 < a) {
			a = x2;
		} else if (x2 > b) {
			b = x2;
		}
		gfx_drawFastHLine(a, y0, b - a + 1, color);
		return;
	}

	int16_t
	dx01 = x1 - x0,
	dy01 = y1 - y0,
	dx02 = x2 - x0,
	dy02 = y2 - y0,
	dx12 = x2 - x1,
	dy12 = y2 - y1,
	sa   = 0,
	sb   = 0;

	/* For upper part of triangle, find scanline crossings for segments
	 * 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
	 * is included here (and second loop will be skipped, avoiding a /0
	 * error there), otherwise scanline y1 is skipped here and handled
	 * in the second loop...which also avoids a /0 error here if y0=y1
	 * (flat-topped triangle).
	 */
	if (y1 == y2) {
		last = y1;   /* Include y1 scanline */
	} else {
		last = y1 - 1; /* Skip it */
	}

	for (y = y0; y <= last; y++) {
		a   = x0 + sa / dy01;
		b   = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;
		/* longhand:
		   a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
		   b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		   */
		if (a > b) {
			swap(a, b);
		}
		gfx_drawFastHLine(a, y, b - a + 1, color);
	}

	/* For lower part of triangle, find scanline crossings for segments
	 * 0-2 and 1-2.  This loop is skipped if y1=y2.
	 */
	sa = dx12 * (y - y1);
	sb = dx02 * (y - y0);
	for (; y <= y2; y++) {
		a   = x1 + sa / dy12;
		b   = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		/* longhand:
		   a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
		   b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		   */
		if (a > b) {
			swap(a, b);
		}
		gfx_drawFastHLine(a, y, b - a + 1, color);
	}
}

// used for circles and rounded corners
void gfx_drawFastVLine(int16_t x, int16_t y,
		       int16_t h, uint16_t color)
{
	gfx_drawLine(x, y, x, y + h - 1, color);
}

// used for circles and rounded corners
void gfx_drawFastHLine(int16_t x, int16_t y,
		       int16_t w, uint16_t color)
{
	gfx_drawLine(x, y, x + w - 1, y, color);
}


/**
 * Rotate the display clockwise or anti-clockwie set by `rotation`
 * @param rotation Type of rotation. Supported values 0, 1, 2, 3
 */
void st_rotate_display(uint8_t rotation)
{
	/*
	* 	(uint8_t)rotation :	Rotation Type
	* 					0 : Potrait
	* 					1 : landscape 1
	* 					2 : Potrait 2
	* 					3 : Landscape 2
	*/
	// Set max rotation value to 4
	rotation = rotation % 4;
	_st_write_command_16bit(ST7789_MADCTL);		//Memory Access Control
	switch (rotation)
	{
		case 0:
			_st_write_data_16bit(TFT_MAD_BGR);	// Default
			st_tftheight = 320;
			st_tftwidth = 480;
			break;
		case 1:
			_st_write_data_16bit(TFT_MAD_MX | TFT_MAD_MY | TFT_MAD_BGR);
			st_tftheight = 320;
			st_tftwidth = 480;
			break;
		case 2:
			_st_write_data_16bit(TFT_MAD_MY | TFT_MAD_MV | TFT_MAD_BGR);
			st_tftheight = 320;
			st_tftwidth = 480;
//			st_set_address_window( 0, 120,  240,  240);
			break;
		case 3:
			_st_write_data_16bit(TFT_MAD_MX | TFT_MAD_MV | TFT_MAD_BGR);
			st_tftheight = 320;
			st_tftwidth = 480;
			break;
	}
}

#define d2r(d) ((d) * 6.2831853 / 360.0)

void planets(void)
{
    int p1, p2, p3, pp1, pp2, pp3;
	p1 = 0;
	p2 = 45;
	p3 = 90;
	pp1 = 0;
	pp2 = 45;
	pp3 = 90;
	st_fill_screen(ST_COLOR_RED);
	st_fill_circle(120, 160, 40, ST_COLOR_YELLOW);
	while (1) {

//	    st_fill_rect(1, 40, 270, 235, ST_COLOR_RED);

//	st_fill_screen(ST_COLOR_RED);
//		   st_draw_string(15, 36, "PLANETS!", ST_COLOR_BLACK, &font_fixedsys_mono_24);
//		st_fill_circle(120, 160, 40, ST_COLOR_YELLOW);
//		st_draw_circle(120, 160, 55, ST_COLOR_LIGHTGREY);
//		st_draw_circle(120, 160, 75, ST_COLOR_LIGHTGREY);
//		st_draw_circle(120, 160, 100, ST_COLOR_LIGHTGREY);
//		st_draw_bitmap(1, 1, &extfirm);

		pp1 = (p1 - 3) % 360;
		pp2 = (p2 - 2) % 360;
		pp3 = (p3 - 1) % 360;

        st_fill_circle(120 + (sin(d2r(pp1)) * 55),
			       160 + (cos(d2r(pp1)) * 55), 5, ST_COLOR_RED);
		st_fill_circle(120 + (sin(d2r(pp2)) * 75),
			       160 + (cos(d2r(pp2)) * 75), 10, ST_COLOR_RED);
		st_fill_circle(120 + (sin(d2r(pp3)) * 100),
			       160 + (cos(d2r(pp3)) * 100), 8, ST_COLOR_RED);

		st_fill_circle(120 + (sin(d2r(p1)) * 55),
			       160 + (cos(d2r(p1)) * 55), 5, ST_COLOR_PURPLE);
		st_fill_circle(120 + (sin(d2r(p2)) * 75),
			       160 + (cos(d2r(p2)) * 75), 10, ST_COLOR_BLACK);
		st_fill_circle(120 + (sin(d2r(p3)) * 100),
			       160 + (cos(d2r(p3)) * 100), 8, ST_COLOR_DARKGREEN);
		p1 = (p1 + 3) % 360;
		p2 = (p2 + 2) % 360;
		p3 = (p3 + 1) % 360;
	}
}



//int poormansmutex = 0;

void dma1_str1_isr(void) {
 dma_clear_interrupt_flags(DMA1, DMA_STREAM1);
    while(!(SPI_SR(SPI5) & SPI_SR_TXP));
    while( !( SPI_SR(SPI5) & SPI_SR_TXC)) {
    ;
    }
for (unsigned i = 0; i < 150; i++)
	  {
		__asm__("nop");
	  }
    spi_disable_tx_dma(SPI5);
    dma_disable_stream(DMA1, DMA_STREAM1);
#ifdef ST_RELEASE_WHEN_IDLE
        ST_CS_IDLE;
#endif
poormansmutex = 0;
}

void dma_just_send(size_t data_size);

void dma_just_send(size_t data_size) {
 dma_disable(DMA1, DMA_CHANNEL1);
spi_disable_tx_dma(SPI5);
 spi_disable(SPI5);
	dma_set_memory_address(DMA1, DMA_STREAM1, (uint32_t) FRAMEBUFFER);
	dma_set_number_of_data(DMA1, DMA_STREAM1, data_size);
    spi_enable(SPI5);
//printf( "dma_get_number_of_data = %d\r\n", dma_get_number_of_data(DMA1, DMA_CHANNEL1));
SPI_CR1(SPI5) |= SPI_CR1_CSTART;
//        __DSB();
		__asm__("nop");
spi_enable_tx_dma(SPI5);
dma_enable(DMA1, DMA_CHANNEL1);
dma_clear_interrupt_flags(DMA1, DMA_STREAM1);
}

void dma_start(void *tfttx, size_t data_size) {
uint64_t  twait = (data_size * twaitm);
 dma_disable(DMA1, DMA_CHANNEL1);
spi_disable_tx_dma(SPI5);
 spi_disable(SPI5);
// dma_clear_interrupt_flags(DMA1, DMA_STREAM1);
    while(poormansmutex == 1) {
    ;
    }

//uint32_t* dmaram  = ( uint32_t* )0x30020000;
poormansmutex = 1;
memcpy(FRAMEBUFFER, (void *)tfttx, data_size);
/*
printf( "DMARAM[0]: 0x%08lX (Word)\r\n", dmaram[ 0 ] );
printf( "DMARAM[1]: 0x%08lX (Word)\r\n", dmaram[ 1 ] );
printf( "DMARAM[2]: 0x%08lX (Word)\r\n", dmaram[ 2 ] );
printf( "DMARAM[3]: 0x%08lX (Word)\r\n", dmaram[ 3 ] );
printf( "DMARAM[4]: 0x%08lX (Word)\r\n", dmaram[ 4 ] );
printf( "DMARAM[5]: 0x%08lX (Word)\r\n", dmaram[ 5 ] );
printf( "DMARAM[6]: 0x%08lX (Word)\r\n", dmaram[ 6 ] );
printf( "DMARAM[7]: 0x%08lX (Word)\r\n", dmaram[ 7 ] );
printf( "DMARAM[8]: 0x%08lX (Word)\r\n", dmaram[ 8 ] );
printf( "DMARAM[9]: 0x%08lX (Word)\r\n", dmaram[ 9 ] );
printf( "DMARAM[10]: 0x%08lX (Word)\r\n", dmaram[ 10 ] );
printf( "DMARAM[11]: 0x%08lX (Word)\r\n", dmaram[ 11 ] );
printf( "DMARAM[12]: 0x%08lX (Word)\r\n", dmaram[ 12 ] );
printf( "DMARAM[13]: 0x%08lX (Word)\r\n", dmaram[ 13 ] );
printf( "DMARAM[14]: 0x%08lX (Word)\r\n", dmaram[ 14 ] );
printf( "DMARAM[15]: 0x%08lX (Word)\r\n", dmaram[ 15 ] );
printf( "DMARAM[16]: 0x%08lX (Word)\r\n", dmaram[ 16 ] );
printf( "DMARAM[17]: 0x%08lX (Word)\r\n", dmaram[ 17 ] );
printf( "DMARAM[18]: 0x%08lX (Word)\r\n", dmaram[ 18 ] );
printf( "DMARAM[19]: 0x%08lX (Word)\r\n", dmaram[ 19 ] );
printf( "DMARAM[20]: 0x%08lX (Word)\r\n", dmaram[ 20 ] );
printf( "DMARAM[21]: 0x%08lX (Word)\r\n", dmaram[ 21 ] );
printf( "DMARAM[22]: 0x%08lX (Word)\r\n", dmaram[ 22 ] );
printf( "DMARAM[23]: 0x%08lX (Word)\r\n", dmaram[ 23 ] );
printf( "DMARAM[24]: 0x%08lX (Word)\r\n", dmaram[ 24 ] );
printf( "DMARAM[38]: 0x%08lX (Word)\r\n", dmaram[ 38 ] );
printf( "DMARAM[44]: 0x%08lX (Word)\r\n", dmaram[ 44 ] );
printf( "DMARAM[60]: 0x%08lX (Word)\r\n", dmaram[ 60 ] );
printf( "DMARAM[70]: 0x%08lX (Word)\r\n", dmaram[ 70 ] );
printf( "DMARAM[75]: 0x%08lX (Word)\r\n", dmaram[ 75 ] );
printf( "DMARAM[120]: 0x%08lX (Word)\r\n", dmaram[ 120 ] );
*/
//SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)FRAMEBUFFER) & ~(uint32_t)0x1F), ILI9163C_A+32);
////    SCB_CleanInvalidateDCache();
	dma_set_memory_address(DMA1, DMA_STREAM1, (uint32_t) FRAMEBUFFER);
	dma_set_number_of_data(DMA1, DMA_STREAM1, data_size);
    spi_enable(SPI5);
    __DSB();
    /*
    if (data_size < 129) {
    twait = 4500;
    } else if (data_size < 257) {
    twait = 8900;
    } else if (data_size < 513) {
    twait = 17000;
    } else if (data_size < 769) {
    twait = 24500;
    } else {
    twait = 34400;
    }*/

//printf( "dma_get_number_of_data = %d\r\n", dma_get_number_of_data(DMA1, DMA_CHANNEL1));
SPI_CR1(SPI5) |= SPI_CR1_CSTART;
//        __DSB();
		__asm__("nop");
spi_enable_tx_dma(SPI5);

dma_enable(DMA1, DMA_CHANNEL1);
//dma_clear_interrupt_flags(DMA1, DMA_STREAM1);
for (unsigned i = 0; i < twait; i++)   //prev value 49400
	  {
		__asm__("nop");
	  }

//dma_status("just b4 in tx");
	if (DMA_LISR(DMA1) & DMA_LISR16_CFEIF) {
		DMA_LIFCR(DMA1) |= DMA_LIFCRC6_CFEIF;
		}
}

#define HWREG(x)    (*((volatile uint32_t *)(x)))

void dma_setup(void) {
    // Note - manipulating the memory address/size of the DMA controller cannot
    // be done while the channel is enabled. Ensure any previous transfer has
    // completed and the channel is disabled before you start another transfer.
    // Tell the DMA controller to start reading memory data from this address
    int temp;
//    memset(FRAMEBUFFER, 0x7BE0, 4096);

     rcc_periph_clock_enable(RCC_SYSCFG);
    rcc_periph_clock_enable(RCC_DMA1);
    nvic_enable_irq(NVIC_DMA1_STR1_IRQ);
    nvic_set_priority(NVIC_DMA1_STR1_IRQ, 48);
    dma_stream_reset(DMA1, DMA_STREAM1);
    dma_set_priority_level(DMA1, DMA_STREAM1, DMA_SxCR_PL_VERY_HIGH);
    dma_disable_direct_mode(DMA1, DMA_STREAM1);
    dma_disable_mburst(DMA1, DMA_STREAM1);
    dma_enable_pburst(DMA1, DMA_STREAM1);
    dma_disable_peripheral_increment_mode(DMA1, DMA_STREAM1);
    dma_set_memory_size(DMA1, DMA_STREAM1, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM1, DMA_SxCR_PSIZE_8BIT);
    dma_set_as_flow_controller(DMA1, DMA_STREAM1);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM1);
	dma_set_transfer_mode(DMA1, DMA_STREAM1, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
	dma_set_peripheral_address(DMA1, DMA_STREAM1, (uint32_t) &SPI_TXDR(SPI5));
	dma_set_memory_address(DMA1, DMA_STREAM1, (uint32_t)0x0000);
	dma_set_number_of_data(DMA1, DMA_STREAM1, (uint16_t)0x00);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM1);
	dma_set_fifo_threshold(DMA1, DMA_STREAM1, DMA_THRESHOLD_QUARTER);
	dma_enable_bufferable_transfers(DMA1, DMA_STREAM1);

//dma_disable(DMA1, DMA_CHANNEL1);
    _st_fixed_delay();
//spi_disable(SPI5);
_st_fixed_delay();
/*	for (unsigned i = 0; i < 250; i++)
	  {
		__asm__("nop");
	  }  */
	  dmamux_set_dma_channel_request(DMAMUX1, 1, 86);
    ST_DC_DAT;

    #ifdef ST_RELEASE_WHEN_IDLE
	ST_CS_ACTIVE;
	#endif
}


void spi_init(void)
{
	rcc_periph_clock_enable(RCC_SPI5);
     rcc_periph_clock_enable(RCC_SYSCFG);
    rcc_periph_clock_enable(RCC_DMA1);
	gpio_mode_setup(GPIOJ, GPIO_MODE_AF, GPIO_PUPD_NONE,
        GPIO10
        |  GPIO11
    );

    gpio_set_output_options(GPIOJ, GPIO_OTYPE_PP,
				GPIO_OSPEED_100MHZ, GPIO10 | GPIO11);

	gpio_set_af(GPIOJ, GPIO_AF5,
       GPIO10
        |  GPIO11
    );

	gpio_mode_setup(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE,
        GPIO6
    );

	gpio_set_output_options(GPIOH, GPIO_OTYPE_PP,
				GPIO_OSPEED_100MHZ, GPIO6);

    gpio_set_af(GPIOH, GPIO_AF5,
       GPIO6
    );

//hardware nss
	gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE,
        GPIO6
    );

	gpio_set_output_options(GPIOF, GPIO_OTYPE_PP,
				GPIO_OSPEED_100MHZ, GPIO6);

    gpio_set_af(GPIOF, GPIO_AF5,
       GPIO6
    );
  // tp_cs
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP,
				GPIO_OSPEED_100MHZ, GPIO7);
    gpio_clear(GPIOC, GPIO7);
for (unsigned i = 0; i < 50; i++)
	  {
		__asm__("nop");
	  }
	   gpio_set(GPIOC, GPIO7);
 //gpio cs
    gpio_mode_setup(GPIOJ, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);
    gpio_set_output_options(GPIOJ, GPIO_OTYPE_PP,
				GPIO_OSPEED_100MHZ, GPIO7);
    gpio_clear(GPIOJ, GPIO7);


 //gpio dc
    gpio_mode_setup(GPIOH, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);
    gpio_set_output_options(GPIOH, GPIO_OTYPE_PP,
				GPIO_OSPEED_100MHZ, GPIO15);
    gpio_clear(GPIOH, GPIO15);

    _st_fixed_delay();
    spi_reset(SPI5);
	SPI_IFCR(SPI5)|=SPI_IER_MODFIE;


//	spi_enable_crc(SPI5);
	spi_enable_software_slave_management(SPI5);

	spi_disable_ss_output(SPI5);
	spi_set_full_duplex_mode(SPI5);
//spi_set_bidirectional_transmit_only_mode(SPI5);
	spi_set_transfer_size(SPI5, 0x0);
	spi_send_msb_first(SPI5);

	spi_set_nss_high(SPI5);
//spi_enable_tx_dma(RCC_SPI5);
	spi_set_baudrate_prescaler(SPI5, SPI_CFG1_MBR_CLK_DIV_2);
	spi_set_data_size(SPI5,SPI_CFG1_DSIZE_8BIT);
		spi_set_master_mode(SPI5);

		SPI_CR2(SPI5) =0;
		SPI_CFG2(SPI5) |= SPI_CFG2_AFCNTR | SPI_CFG2_SSOE;


	spi_enable(SPI5);
	SPI_CR1(SPI5) |= SPI_CR1_CSTART;
    spi_enable_tx_dma(SPI5);
	_st_fixed_delay();
//	gpio_clear(GPIOF, GPIO6);
}


/**
 * Initialize the display driver
 */
void st_init()
{
spi_status(SPI5, "before dma init");
        dma_setup();
        uint64_t val1;
        uint64_t val2;
        uint64_t val3;
        uint64_t val4;
        uint64_t sum1;
        uint64_t sum2;
//  tftdma();
  //Serial.println("Init_DMA");
//  DMA_Config ();
        spi_init();
        spi_status(SPI5, "just after spi init");
//        dma_setup();
/*        spi_disable(SPI5);
        _st_fixed_delay();
        spi_set_baudrate_prescaler(SPI5, SPI_CFG1_MBR_CLK_DIV_8);
        _st_fixed_delay();
        spi_enable(SPI5);
        */
#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_ACTIVE;
#endif
	// Hardwae reset is not mandatory if software rest is done
#ifdef ST_HAS_RST
		ST_RST_ACTIVE;
		_st_fixed_delay();
		ST_RST_IDLE;
#endif
#ifdef ST_RELEASE_WHEN_IDLE
		ST_CS_ACTIVE;
#endif
    _st_fixed_delay();
//    spi_set_baudrate_prescaler(SPI5, SPI_CR1_BR_FPCLK_DIV_16);
//    gpio_clear(GPIOD, GPIO7);
//    delay(120);
//    gpio_set(GPIOD, GPIO7);
//    delay(150);
    _st_write_command_16bit(0x01); // SW reset

//    _st_write_command_16bit(0x01); // SW reset
    _st_fixed_delay();
    _st_write_command_16bit(0x11); // Sleep out, also SW reset
    _st_fixed_delay();

    _st_write_command_16bit(0x3A);

      _st_write_data_16bit(0x55);           // 16 bit colour interface


    _st_write_command_16bit(0xC2);
    _st_write_data_16bit(0x44);

    _st_write_command_16bit(0xC5);
    _st_write_data_16bit(0x00);
    _st_write_data_16bit(0x00);
    _st_write_data_16bit(0x00);
    _st_write_data_16bit(0x00);

    _st_write_command_16bit(0xE0);
    _st_write_data_16bit(0x0F);
    _st_write_data_16bit(0x1F);
    _st_write_data_16bit(0x1C);
    _st_write_data_16bit(0x0C);
    _st_write_data_16bit(0x0F);
    _st_write_data_16bit(0x08);
    _st_write_data_16bit(0x48);
    _st_write_data_16bit(0x98);
    _st_write_data_16bit(0x37);
    _st_write_data_16bit(0x0A);
    _st_write_data_16bit(0x13);
    _st_write_data_16bit(0x04);
    _st_write_data_16bit(0x11);
    _st_write_data_16bit(0x0D);
    _st_write_data_16bit(0x00);

    _st_write_command_16bit(0xE1);
    _st_write_data_16bit(0x0F);
    _st_write_data_16bit(0x32);
    _st_write_data_16bit(0x2E);
    _st_write_data_16bit(0x0B);
    _st_write_data_16bit(0x0D);
    _st_write_data_16bit(0x05);
    _st_write_data_16bit(0x47);
    _st_write_data_16bit(0x75);
    _st_write_data_16bit(0x37);
    _st_write_data_16bit(0x06);
    _st_write_data_16bit(0x10);
    _st_write_data_16bit(0x03);
    _st_write_data_16bit(0x24);
    _st_write_data_16bit(0x20);
    _st_write_data_16bit(0x00);

    _st_write_command_16bit(TFT_INVOFF);

    _st_write_command_16bit(0x36);
    _st_write_data_16bit(0x28);
    _st_write_data_16bit(0x00);
    _st_write_command_16bit(0x29);                     // display on
    ST_DC_DAT;
    _st_fixed_delay();
    /*
        spi_disable(SPI5);
        _st_fixed_delay();
        spi_set_baudrate_prescaler(SPI5, SPI_CFG1_MBR_CLK_DIV_2);
        _st_fixed_delay();
        spi_enable(SPI5);
*/
//  st_fill_screen(ST_COLOR_RED);
    spi_status(SPI5, "end of start commands");

  _st_fixed_delay();

  /*
  st_draw_line(1, 10, 10, 20, 4, ST_COLOR_YELLOW);
//  st_draw_line(25, 70, 35, 80, 4, ST_COLOR_YELLOW);
//  st_draw_line(35, 80, 30, 20, 4, ST_COLOR_YELLOW);
   st_draw_string(50, 50, "16TON'S OF H7", ST_COLOR_BLACK, &font_fixedsys_mono_24);
//   st_fill_rect_fast(100, 110, 20, 20, ST_COLOR_YELLOW);
//  gfx_drawLine(10, 20, 25, 70, ST_COLOR_YELLOW);
//  gfx_drawLine(25, 70, 35, 80, ST_COLOR_YELLOW);
 // gfx_drawLine(35, 80, 30, 20, ST_COLOR_YELLOW);  _st_fixed_delay();
//st_draw_bitmap(1, 110, &b16ton);
	st_fill_round_rect(60, 60, 100, 100, 5, ST_COLOR_WHITE);
//	st_draw_round_rect(10, 10, 220, 220, 5, ST_COLOR_RED);
	st_fill_circle(20, 50, 10, ST_COLOR_BLUE);
//	st_fill_circle(120, 250, 10, ST_COLOR_GREEN);
*/

//st_fill_color(0x1784, 10);

//SPI_CR1(SPI5) |= SPI_CR1_CSTART;



 //   st_fill_screen_nodma(ST_COLOR_YELLOW);
//    st_fill_round_rect(60, 60, 100, 100, 5, ST_COLOR_WHITE);
//    for ( size_t i = 0; i < ILI9163C_A; ++i ) {
//      FRAMEBUFFER[ i ] = color;
//      while((SPI_SR(SPI5) & SPI_SR_RXWNE) || ( (SPI_SR(SPI5) & (SPI_SR_RXPLVL_MASK<<SPI_SR_RXPLVL_SHIFT)) !=0 ) )
//SPI_RXDR(SPI5);
//    }
 //   _st_fixed_delay();
//    _st_fixed_delay();
 //   _st_fixed_delay();
//    printf("printf \r\n
// twaitm = (64);
//st_fill_screen(ST_COLOR_YELLOW);
 twaitm = (34);
 __DSB();
 //   _st_fixed_delay();
 //   _st_fixed_delay();
//    _st_fixed_delay();
//dma_just_send(4096);
 //   _st_fixed_delay();
    _st_fixed_delay();
}

void _st_fixed_delay(void)
{
	for (uint16_t i = 0; i < 5000; i++)
		__asm__("nop");
}



