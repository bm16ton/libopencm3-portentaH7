#ifndef _LIBOPENCM3_QSPI2_H
#define _LIBOPENCM3_QSPI2_H

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern uint8_t val[8];
void quad_setup(void);

void quad_read(uint32_t addr, size_t len, uint8_t *dest);
void quad_write(uint32_t addr, size_t len, uint8_t *dest);
void quad_erase(uint32_t addr);
void quad_map(void);

#endif
