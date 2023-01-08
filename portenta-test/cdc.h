
#ifndef LIBOPENCM3_CDC_H
#define LIBOPENCM3_CDC_H

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

void SysTick_IRQn_handler( void );
void delay_ms( uint32_t ms );
int sdramsetup( void );
int ramtest(void);
uint32_t SysTick_Config(uint32_t ticks);
int _write(int file, char *ptr, int len);

uint32_t SysTick_Config(uint32_t ticks)
{
//  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
//  {
//    return (1UL);                                                   /* Reload value impossible */
//  }
(void)ticks;
systick_set_reload(400000000-1);
//  SysTick->LOAD  = (uint32_t)(ticks - 1UL);                         /* set reload register */
//  NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
 // SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
  	systick_counter_enable();
  	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  		systick_interrupt_enable();
//  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
//                   SysTick_CTRL_TICKINT_Msk   |
//                   SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */
  return (0UL);                                                     /* Function successful */
}


#endif
