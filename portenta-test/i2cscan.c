


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


void print_bits(size_t const size, void const *const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;

    printf("ret=0x");
    for (i=size-1;i>=0;i--)
    {
        for (j=7;j>=0;j--)
        {
            byte = b[i] & (1<<j);
            byte >>= j;
            printf("%u", byte);
        }
    }
    printf("\r\n");
}

void i2c_deinit(void)
{
	i2c_send_stop(I2C3);
    printf("i2c_deinit");
	i2c_reset(I2C3);
	i2c_peripheral_disable(I2C3); /* disable i2c during setup */

}

uint8_t i2c_start(uint32_t i2c, uint8_t address, uint8_t mode)
{
	i2c_send_start(i2c);
printf("nack = %d \r\n", i2c_nack(I2C3));
	/* Wait for master mode selected */
//	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
//		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
//    while(i2c_busy(i2c)) {
//    while ((I2C_ISR(i2c)) & I2C_ISR_BUSY) {
printf("i2c_start b4 busy check\r\n");
//while (I2C_ISR(I2C3) & I2C_ISR_BUSY) {
//    ;
 //   }
//bensdelay();
printf("i2c_send_7bit_address\r\n");
	i2c_send_7bit_address(i2c, address, mode);
printf("after i2c_send_7bit_address\r\n");
	int timeout = 20000;
	/* Waiting for address is transferred. */
//	while (!(I2C_ISR(i2c) & I2C_ISR_ADDR)) {
    while (!(I2C_ISR(i2c) & I2C_ISR_TC)) {
		if (timeout > 0) {
			timeout--;
		} else {
			return 1;
		}
	}

printf("just cleaning address left\r\n");
	/* Cleaning ADDR condition sequence. */
//	I2C_ISR(i2c) &= ~I2C_ICR_ADDRCF;
	
	uint32_t temp = I2C_ISR(i2c) & I2C_ICR_ADDRCF;
	(void) temp;
	uint32_t reg32 = I2C_ISR(i2c);
	(void) reg32; /* unused */

	return 0;
}

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
	printf("1 in i2c_read. before busy check\r\n");
	while (I2C_ISR(i2c) & I2C_ISR_BUSY); // {
 //   bensdelay();
//    while (!(I2C_ISR(i2c) & I2C_ISR_TC));
	printf("2 in i2c_read. after busy check\r\n");
//		if (timeout > 0) {
//			timeout--;
//		} else {
//			return -1;
//		}
//	}
    printf("3 in i2c_read. before i2c_start\r\n");
	if (i2c_start(i2c, address, I2C_WRITE)) {
	    printf("returning 0 ");
	    printf("nack = %d \r\n", i2c_nack(I2C3));
		return 0;
	}
	i2c_send_data(i2c, reg);
    printf("4 in i2c_read.after i2c_start and send data\r\n");
	timeout = 20000;
	printf("5 in i2c_read. before tx complete while\r\n");
	while (!(I2C_ISR(i2c) & I2C_ISR_TC)) {
		if (timeout > 0) {
			timeout--;
		} else {
			return -1;
		}
	}
    printf("6 in i2c_read. after tx complete while\r\n");
	i2c_start(i2c, address, I2C_READ);
    printf("6.5\r\n");
	i2c_send_stop(i2c);
    printf("7 in i2c_read. after send stop\r\n");
//	while (!(I2C_ISR(i2c) & I2C_ISR_RXNE));
    printf("8 in i2c_read. after RXNE\r\n");
	int result = (int)i2c_get_data(i2c);
    printf("9 in i2c_read. after i2c_get_data\r\n");
	I2C_ICR(i2c) &= ~I2C_ICR_NACKCF;
	printf("10 in i2c_read. after I2C_ICR_NACKCF\r\n");
//	msleep(50);
    bensdelay();
	  
	i2c_send_stop(i2c);
    printf("11 in i2c_read. after i2c_send_stop\r\n");
	return result;
}

void i2cscan(void) {
	printf("\r\nstarting i2c scan.\r\n");

	int i, j;
	for (i = 1; i < 0x80; i++) {
		i2c_setup();
//		msleep(50);
        bensdelay();
		for (j = 0; j < 0x09; j++) {
			int data; // = 0;
//			printf("b4 i2c_read.\r\n");
			data = i2c_read(I2C3, i, j);
			printf("data = %d, i = %d, j = %d \r\n", data, i, j);
//			printf("after i2c_read.\r\n");
			if (data > -1) {
				if (data) {
					printf("device on address 0x%02X : reg = 0x%02X with data == 0x%02X\r\n", i, j, data);
				}
			} else {
				printf("errerr!!\r\n");
				break;
			//return;
			}
			//i2c_send_stop(I2C3);
		}
	//	i2c_deinit();
		bensdelay();
//		msleep(50);
	}
	printf("scan ended!\r\n");
	}
