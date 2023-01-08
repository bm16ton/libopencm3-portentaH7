/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/adc.h>

#include <stdio.h>
#include "usb-gadget0.h"

#define ER_DEBUG
#ifdef ER_DEBUG
#define ER_DPRINTF(fmt, ...) \
	do { printf(fmt, ## __VA_ARGS__); } while (0)
#else
#define ER_DPRINTF(fmt, ...) \
	do { } while (0)
#endif


static volatile uint32_t millis_count;
void adc_init(void);

int main(void)
{
    SCB_VTOR = (uint32_t) 0x08000000;
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
	/* LED to indicate boot process */
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
	gpio_set(GPIOC, GPIO1);

	rcc_periph_clock_enable(RCC_GPIOA);
	/*
	 * Vile hack to reenumerate, physically _drag_ d+ low.
	 * do NOT do this if you're board has proper usb pull up control!
	 * (need at least 2.5us to trigger usb disconnect)
	 
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	*/
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
		      GPIO11 | GPIO12);
	gpio_clear(GPIOA, GPIO11 | GPIO12);
	for (unsigned int i = 0; i < 1100000; i++) {
		__asm__("nop");
	}



	rcc_periph_clock_enable(RCC_OTGFS);
    systick_init();
    adc_init();

	usbd_device *usbd_dev = gadget0_init(&st_usbfs_v1_usb_driver,
					     "stm32f103-generic");

	ER_DPRINTF("bootup complete\n");
	gpio_set(GPIOC, GPIO1);
	while (1) {
		gadget0_run(usbd_dev);
	}

}

void adc_init()
{
    rcc_periph_clock_enable(RCC_ADC1);
    adc_power_off(ADC1);

    // configure for regular single conversion
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_right_aligned(ADC1);

    // power up
    adc_power_on(ADC1);
    delay(100);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);

    // configure A0 as ADC channel
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, 0);
//    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, 1);
    uint8_t channels[] = {ADC_CHANNEL0};
    adc_set_regular_sequence(ADC1, 1, channels);
}



uint32_t millis()
{
    return millis_count;
}

void delay(uint32_t ms)
{
    int32_t target_time = millis_count + ms;
    while (target_time - (int32_t)millis_count > 0)
        ;
}

void systick_init()
{
    // Initialize SysTick
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_set_reload(rcc_ahb_frequency / 8 / 1000 - 1);

    // Enable and start
    systick_interrupt_enable();
    systick_counter_enable();
}

// System tick timer interrupt handler
void sys_tick_handler()
{
    millis_count++;
}

// Takes a single sample

