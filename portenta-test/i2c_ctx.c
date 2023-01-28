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

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "regdump.h"
//#include <librfn/time.h>
#include "util.h"
#include "i2cusb.h"

#define I2C_WRITE			0
#define I2C_READ            1
//void i2c_send_7bit_address(uint32_t i2c, uint8_t slave, uint8_t readwrite);

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

static bool i2c_ctx_is_timed_out(i2c_ctx_t *c)
{
	if (cyclecmp32(time_now(), c->timeout) > 0) {
		if (c->verbose) {
			printf("I2C TRANSACTION TIMED OUT\n");
			regdump(I2C_ISR(c->i2c), i2c_isr_desc);
			regdump(I2C_CR1(c->i2c), i2c_cr1_desc);
		}

		return true;
	}

	return false;
}

void i2c_ctx_init(i2c_ctx_t *c, uint32_t pi2c)
{
	memset(c, 0, sizeof(*c));

	c->i2c = pi2c;
	c->timeout = time_now() + 100000;
}

void i2c_ctx_reset(i2c_ctx_t *c)
{
    (void)c;
	rcc_periph_clock_enable(RCC_I2C3);

 //   printf("i2c b4 reset \r\n");
	i2c_reset(I2C3);
	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOH, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO7 | GPIO8);
	gpio_set_output_options(GPIOH, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO7 | GPIO8);
	gpio_set_af(GPIOH, GPIO_AF4, GPIO7 | GPIO8);
	i2c_peripheral_disable(I2C3);
	i2c_reset(I2C3);
//	i2c_set_prescaler(I2C3, 8);
	//configure ANFOFF DNF[3:0] in CR1
//	i2c_enable_analog_filter(I2C3);
    i2c_disable_analog_filter(I2C3);
	i2c_set_digital_filter(I2C3, 0);
	/* HSI is at 8Mhz */
	i2c_set_speed(I2C3, i2c_speed_fm_400k, 8);
	//configure No-Stretch CR1 (only relevant in slave mode)
//	i2c_enable_stretching(I2C3);
	//addressing mode
	i2c_set_7bit_addr_mode(I2C3);
	i2c_peripheral_enable(I2C3);
	i2c_set_own_7bit_slave_address(I2C3, 0x00);
	
	        for (uint32_t loop = 0; loop < 1500; ++loop) {
    __asm__("nop");
  } 
}

pt_state_t i2c_ctx_start(i2c_ctx_t *c)
{
	PT_BEGIN(&c->leaf);

	i2c_send_start(c->i2c);

	while (!i2c_ctx_is_timed_out(c) &&
	       !((I2C_CR2(c->i2c) & I2C_CR2_START) &
		 (I2C_ISR(c->i2c) | (I2C_ISR_BUSY))))
		PT_YIELD();

	if (!(I2C_CR2(c->i2c) & I2C_CR2_START)) {
		i2c_ctx_reset(c);
		c->err = EIO;
	}

	PT_END();
}
/*
void i2c_send_7bit_address(uint32_t i2c, uint8_t slave, uint8_t readwrite)
{
	I2C_TXDR(i2c) = (uint8_t)((slave << 1) | readwrite);
}
*/
pt_state_t i2c_ctx_sendaddr(i2c_ctx_t *c, uint16_t addr,
				   uint8_t bytes_to_read)
{
	PT_BEGIN(&c->leaf);

	c->bytes_remaining = bytes_to_read;
    i2c_set_7bit_address(c->i2c, addr);
	i2c_send_7bit_address(c->i2c, addr, !!bytes_to_read);

	while (!i2c_ctx_is_timed_out(c) &&		// bad
	       !(I2C_ISR(c->i2c) & I2C_ISR_NACKF) &&	// bad
	       !(I2C_ISR(c->i2c) & I2C_ISR_ADDR))	// good
		PT_YIELD();

	if (!(I2C_ISR(c->i2c) & I2C_ISR_ADDR)) {
		i2c_ctx_reset(c);
		c->err = EIO;
	}

	/* If we are only reading one byte we must get ready to NACK the
	 * final byte.
	 */
	if (c->bytes_remaining == 1) {
		I2C_CR2(c->i2c) |= I2C_CR2_NACK;
        i2c_disable_ack(c->i2c);
	    i2c_nack(c->i2c);
	} else if (c->bytes_remaining >= 2) {
	    i2c_enable_ack(c->i2c);
		I2C_CR2(c->i2c) &= ~I2C_CR2_NACK;
    }
	/* Read sequence has side effect or clearing I2C_CR1_ADDR */
	uint32_t reg32 __attribute__((unused));
	reg32 = I2C_CR2(c->i2c);

	if (c->bytes_remaining == 1)
		i2c_send_stop(c->i2c);

	PT_END();
}

pt_state_t i2c_ctx_senddata(i2c_ctx_t *c, uint8_t data)
{
	PT_BEGIN(&c->leaf);

	i2c_send_data(c->i2c, data);

	while (!i2c_ctx_is_timed_out(c) && !(I2C_ISR(c->i2c) & I2C_ISR_TC))
		PT_YIELD();

	if (!(I2C_ISR(c->i2c) & I2C_ISR_TC)) {
		i2c_ctx_reset(c);
		c->err = EIO;
	}

	PT_END();
}

pt_state_t i2c_ctx_getdata(i2c_ctx_t *c, uint8_t *data)
{
	PT_BEGIN(&c->leaf);

	while (!i2c_ctx_is_timed_out(c) && !(I2C_ISR(c->i2c) & I2C_ISR_RXNE))
		PT_YIELD();

	if (!(I2C_ISR(c->i2c) & I2C_ISR_RXNE)) {
		i2c_ctx_reset(c);
		c->err = EIO;
		PT_EXIT();
	}

	/* Need to NACK the final byte and STOP the transaction */
	if (--c->bytes_remaining == 1) {
	    i2c_disable_ack(c->i2c);
	    i2c_nack(c->i2c);
		I2C_CR2(c->i2c) |= I2C_CR2_NACK;
		i2c_send_stop(c->i2c);
	}

	*data = i2c_get_data(c->i2c);

	PT_END();
}

pt_state_t i2c_ctx_stop(i2c_ctx_t *c)
{
	PT_BEGIN(&c->leaf);

	while (!i2c_ctx_is_timed_out(c) &&
	       !(I2C_ISR(c->i2c) & (I2C_ISR_TC | I2C_ISR_TXE)))
		PT_YIELD();

	if (!(I2C_ISR(c->i2c) & (I2C_ISR_TC | I2C_ISR_TXE))) {
		i2c_ctx_reset(c);
		c->err = EIO;
		PT_EXIT();
	}

	i2c_send_stop(c->i2c);

	/* TODO: is it safe to just drop out of this */

	PT_END();
}

pt_state_t i2c_ctx_detect(i2c_ctx_t *c, i2c_device_map_t *map)
{
	PT_BEGIN(&c->pt);

	memset(map, 0, sizeof(*map));

	for (c->i = 0; c->i < 0x80; c->i++) {
		c->timeout = time_now() + 10000;
		c->err = 0;

		PT_SPAWN(&c->leaf, i2c_ctx_start(c));
		if (c->err)
			continue;

		PT_SPAWN(&c->leaf, i2c_ctx_sendaddr(c, c->i, I2C_WRITE));
		if (c->err)
			continue;

		PT_SPAWN(&c->leaf, i2c_ctx_stop(c));
		if (c->err)
			continue;

		map->devices[c->i / 16] |= 1 << (c->i % 16);
	}

	PT_END();
}

pt_state_t i2c_ctx_setreg(i2c_ctx_t *c, uint16_t addr, uint16_t reg,
				 uint8_t val)
{
	PT_BEGIN(&c->pt);

	PT_SPAWN(&c->leaf, i2c_ctx_start(c));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_sendaddr(c, addr, I2C_WRITE));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_senddata(c, reg));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_senddata(c, val));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_stop(c));

	PT_END();
}

pt_state_t i2c_ctx_getreg(i2c_ctx_t *c, uint16_t addr, uint16_t reg,
				 uint8_t *val)
{
	PT_BEGIN(&c->pt);

	PT_SPAWN(&c->leaf, i2c_ctx_start(c));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_sendaddr(c, addr, I2C_WRITE));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_senddata(c, reg));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_start(c));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_sendaddr(c, addr, I2C_READ));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_getdata(c, val));

	/* For reads STOP is generated automatically by sendaddr and/or
	 * getdata
	 */

	PT_END();
}

pt_state_t i2c_ctx_write(i2c_ctx_t *c, uint16_t addr, uint8_t *data,
				uint8_t len)
{
	PT_BEGIN(&c->pt);

	PT_SPAWN(&c->leaf, i2c_ctx_start(c));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_sendaddr(c, addr, I2C_WRITE));
	PT_EXIT_ON(c->err);

	for (c->i = 0; c->i < len; c->i++) {
		PT_SPAWN(&c->leaf, i2c_ctx_senddata(c, data[c->i]));
		PT_EXIT_ON(c->err);
	}

	PT_SPAWN(&c->leaf, i2c_ctx_stop(c));

	PT_END();
}
