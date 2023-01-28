#include <errno.h>
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include "test_i2c.h"
#include <string.h>
#include <unistd.h>

void i2c_setup(void)
{
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
//	printf("i2c clock = %ld \r\n", rcc_get_i2c_clk_freq(I2C3));
}

void i2ctest(void) {
uint8_t temp;
   uint8_t cmdWrite1[] = { 0x1, 0x19, 0x0 };
   i2c_transfer7(I2C3, 0x77, cmdWrite1, sizeof(cmdWrite1), NULL, 0);
      for (uint32_t loop = 0; loop < 200; ++loop) {
    __asm__("nop");
  } 
  printf("i2c after 1st send  \r\n");
        for (uint32_t loop = 0; loop < 200; ++loop) {
    __asm__("nop");
  } 
   i2c_transfer7(I2C3, 0x77, NULL, 0, &temp, 1);
      for (uint32_t loop = 0; loop < 200; ++loop) {
    __asm__("nop");
  } 
  i2c_transfer7(I2C3, 0x77, NULL, 0, &temp, 1);
      for (uint32_t loop = 0; loop < 200; ++loop) {
    __asm__("nop");
  } 
   uint8_t cmdWrite2[] = { 0x1, 25, 0x0 };
   i2c_transfer7(I2C3, 0x77, cmdWrite2, sizeof(cmdWrite2), &temp, 1);
      for (uint32_t loop = 0; loop < 200; ++loop) {
    __asm__("nop");
  } 
   i2c_transfer7(I2C3, 0x77, NULL, 0, &temp, 1);
      for (uint32_t loop = 0; loop < 200; ++loop) {
    __asm__("nop");
  } 
}
