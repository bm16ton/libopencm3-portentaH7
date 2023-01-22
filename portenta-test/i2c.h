
#ifndef __I2C_H
#define __I2C_H

void i2cscan(void);
void print_bits(size_t const size, void const *const ptr);
uint8_t i2c_start(uint32_t i2c, uint8_t address, uint8_t mode);
void i2c_deinit(void);
uint8_t i2c_write(uint32_t i2c, uint8_t address, uint8_t reg,
	uint8_t data);
int i2c_read(uint32_t i2c, uint8_t address, uint8_t reg);


#endif
