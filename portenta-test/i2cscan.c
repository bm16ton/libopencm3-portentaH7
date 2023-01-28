


#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include "i2c.h"
#include "test_i2c.h"

#define I2C_WRITE           0
#define I2C_READ            1

void bensdelay(void);

void bensdelay(void) {
	for (unsigned i = 0; i < 120; i++)
	  {
		__asm__("nop");
	  }
}

void i2c_deinit(uint32_t i2c)
{
	i2c_send_stop(i2c);
	i2c_reset(i2c);
	i2c_peripheral_disable(i2c); /* disable i2c during setup */

}

uint8_t i2c_start(uint32_t i2c, uint8_t address, uint8_t mode)
{
i2c_set_7bit_address(i2c, address);
	i2c_send_start(i2c);

    bensdelay();
	i2c_send_7bit_address(i2c, address, mode);
	int timeout = 20000;
	/* Waiting for address is transferred. */
    while (!(I2C_ISR(i2c) & I2C_ISR_TC)) {
		if (timeout > 0) {
			timeout--;
		} else {
			return 1;
		}
	}

	/* Cleaning ADDR condition sequence. */
//	I2C_ISR(i2c) &= ~I2C_ICR_ADDRCF;
	
	uint32_t temp = I2C_ISR(i2c) & I2C_ICR_ADDRCF;
	(void) temp; /* unused */
	uint32_t reg32 = I2C_ISR(i2c);
	(void) reg32; /* unused */

	
	
	return 0;
}

/* unused untested */
uint8_t i2c_write(uint32_t i2c, uint8_t address, uint8_t reg,
	uint8_t data)
{
	i2c_start(i2c, address, I2C_WRITE);

	i2c_send_data(i2c, reg);

	while (!(I2C_ISR(i2c) & I2C_ISR_TC));
	i2c_send_data(i2c, data);

	while (!(I2C_ISR(i2c) & I2C_ISR_TC));

	i2c_send_stop(i2c);

	return 0;
}


int i2c_read(uint32_t i2c, uint8_t address, uint8_t reg)
{
	uint32_t timeout = 20000;
	while (I2C_ISR(i2c) & I2C_ISR_BUSY); 
	
	if (i2c_start(i2c, address, I2C_WRITE)) {
		return 0;
	}
	
	i2c_send_data(i2c, reg);
	timeout = 20000;
	while (I2C_ISR(i2c) & I2C_ISR_BUSY) {
		if (timeout > 0) {
			timeout--;
		} else {
			return -1;
		}
	}

	i2c_start(i2c, address, I2C_READ);
	i2c_send_stop(i2c);

//	while (!(I2C_ISR(i2c) & I2C_ISR_RXNE));
//printf(" rxne value = %d\r\n", i2c_received_data(i2c));

	int result = (int)i2c_get_data(i2c);

//	i2c_clear_nack(i2c);

    bensdelay();
	i2c_send_stop(i2c);

	return result;
}

void i2cscan(uint32_t i2c) {
	printf("\r\nstarting i2c scan.\r\n");

	int i, j;
	for (i = 1; i < 0x80; i++) {
		i2c_setup();
        bensdelay();
		for (j = 0; j < 0x100; j++) {
			int data; // = 0;
			data = i2c_read(i2c, i, j);
			if (data == 0) {
			break;
			}
			if (data == -1) {
				
					printf("device on address 0x%02X : reg = 0x%02X with data == 0x%02X\r\n", i, j, data);
			break;	
			} else {
				printf("unknown error at addr = 0x%02X, reg = %d, data = %d \r\n", data, i, j);
				break;
			}
			//i2c_send_stop(I2C3);
		}
	    i2c_deinit(i2c);
		bensdelay();
	}
	printf("scan ended!\r\n");
	}
