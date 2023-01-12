#ifndef _LIBOPENCM3_QSPI_H
#define _LIBOPENCM3_QSPI_H

#include <string.h>
#include <stdio.h>
#include <stdlib.h>


int qspiinit( void );


void qspi_wen( uint8_t dw );
void qspi_reg_wait( uint8_t reg, uint32_t msk,
                    uint32_t mat, uint8_t dw );
void qspi_erase_sector( uint32_t snum );
void qspi_write_word( uint32_t addr, uint32_t data );

#endif
