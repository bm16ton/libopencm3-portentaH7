/*
 * Part of librfm3 (a utility library built on librfn and libopencm3)
 *
 * Copyright (C) 2014 Daniel Thompson <daniel@redfelineninja.org.uk>
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

#include "i2c_ctx.h"

#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "regdump.h"
#include "../cdc.h"
#include "util.h"
#include "i2cusb.h"
#include "../vars.h"

#define I2C_WRITE			0
#define I2C_READ            1


uint16_t i2caddr;

#define D(x) { #x, x }
static const regdump_desc_t i2c_isr_desc[] = { { "I2C_ISR", 0 },
					       D(I2C_ISR_ALERT),
					       D(I2C_ISR_TIMEOUT),
					       D(I2C_ISR_PECERR),
					       D(I2C_ISR_OVR),
					       D(I2C_ISR_NACKF),
					       D(I2C_ISR_ARLO),
					       D(I2C_ISR_BERR),
					       D(I2C_ISR_STOPF),
					       D(I2C_ISR_TC),
					       D(I2C_ISR_ADDR),
					       { NULL, 0 } };
#undef D

#define D(x) { #x, x }
static const regdump_desc_t i2c_cr1_desc[] = { { "I2C_CR1", 0 },
					       D(I2C_CR1_TXIE),
					       D(I2C_CR1_RXIE),
					       D(I2C_CR1_SBC),
					       { NULL, 0 } };
#undef D

void i2c_ctx_init(i2c_ctx_t *c, uint32_t pi2c)
{
	memset(c, 0, sizeof(*c));

	c->i2c = pi2c;
	c->timeout = time_now() + 100000;
}

void i2c_ctx_reset(i2c_ctx_t *c)
{
    (void)c;


    	switch (c->i2c) {
	case I2C1:
        rcc_periph_clock_enable(RCC_I2C1);
		break;
	case I2C2:
        rcc_periph_clock_enable(RCC_I2C2);
		break;
	case I2C3:
        rcc_periph_clock_enable(RCC_I2C3);
		break;
	}

	gpio_mode_setup(GPIOH, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO7 | GPIO8);
	gpio_set_output_options(GPIOH, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO7 | GPIO8);
	gpio_set_af(GPIOH, GPIO_AF4, GPIO7 | GPIO8);
	i2c_peripheral_disable(c->i2c);
	i2c_reset(c->i2c);
    i2c_disable_analog_filter(c->i2c);
	i2c_set_digital_filter(c->i2c, 0);
//	i2c_set_speed(c->i2c, i2c_speed_fm_400k, 8);
//    enable_i2c_fmp_port(I2C3);
if (i2cspeed == 1) {
    i2c_set_speed(c->i2c, i2c_speed_sm_100k, 8);
} else if (i2cspeed == 2) {
    i2c_set_speed(c->i2c, i2c_speed_fm_400k, 8);
} else if (i2cspeed == 3) {
    i2c_set_speed(c->i2c, i2c_speed_fmp_1m, 16);
}
	i2c_set_7bit_addr_mode(c->i2c);
	i2c_peripheral_enable(c->i2c);
	i2c_set_own_7bit_slave_address(c->i2c, 0x00);

	 for (uint32_t loop = 0; loop < 150; ++loop) {
        __asm__("nop");
    }

}

pt_state_t i2c_ctx_start(i2c_ctx_t *c, uint16_t addr, uint16_t size, int dir)
{

	PT_BEGIN(&c->leaf);
	i2c_clear_nack(c->i2c);
	i2c_clear_stop(c->i2c);
	i2c_set_7bit_address(c->i2c, addr);
	if (dir == 1) {
		i2c_set_read_transfer_dir(c->i2c);
	} else {
		i2c_set_write_transfer_dir(c->i2c);
	}
	i2c_set_bytes_to_transfer(c->i2c, size);
	i2c_disable_autoend(c->i2c);
	i2c_send_start(c->i2c);

    while (i2c_is_start(c->i2c)) {
        ;
	}

    if (i2c_nack(c->i2c) == 0) {
        ;   //todo i dunno something
    } else {
        i2c_send_stop(c->i2c);
        i2c_clear_stop(c->i2c);
        i2c_clear_nack(c->i2c);
        c->err = EIO;
    }
	PT_END();
}

pt_state_t i2c_ctx_senddata(i2c_ctx_t *c, uint8_t *data, uint16_t size)
{

	PT_BEGIN(&c->leaf);
	if (size != 0) {
    int i = 0;
	while (size--) {
	if (i2c_stop_detected(c->i2c)) {
	    printf("i2c send stop detected\r\n");
		/* Clear potential stop detection */
		i2c_clear_stop(c->i2c);
	}
	if (i2c_nack(c->i2c)) {
		/* Stop transaction on nack */
		printf("i2c nack detected stop\r\n");
		i2c_clear_nack(c->i2c);
		i2c_send_stop(c->i2c);
		c->err = EIO;
	}

	i2c_send_data(c->i2c, *data++);
	printf("sent data = 0x%04X\r\n", (uint8_t)data[i]);  //perfect delay will replace with register stuff
	i++;
    }
 }
    while (!i2c_transfer_complete(c->i2c));

	PT_END();
}

pt_state_t i2c_ctx_getdata(i2c_ctx_t *c, uint8_t *data, uint16_t size)
{
	PT_BEGIN(&c->leaf);
	 if (size != 0) {

		for (size_t i = 0; i < size; i++) {
			while (i2c_received_data(c->i2c) == 0);
			data[i] = i2c_get_data(c->i2c);
			for (uint32_t loop = 0; loop < 550; ++loop) {
                __asm__("nop");
                }
			printf("get data = 0x%04X\r\n", data[i]);  //simply handy
		}

    }

	PT_END();
}

pt_state_t i2c_ctx_stop(i2c_ctx_t *c)
{
	PT_BEGIN(&c->leaf);

	if (i2c_nack(c->i2c)) {
		i2c_clear_nack(c->i2c);
		i2c_send_stop(c->i2c);
	} else {
	    i2c_send_stop(c->i2c);
    }

	PT_END();
}

