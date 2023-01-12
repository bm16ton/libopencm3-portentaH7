
#ifndef CDC_H
#define CDC_H

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

void SysTick_IRQn_handler( void );
void delay_ms( uint32_t ms );
int sdramsetup( void );
int ramtest(void);
extern volatile uint32_t systick;

int _write(int file, char *ptr, int len);

uint32_t SysTick_Config(uint32_t ticks);

#endif
