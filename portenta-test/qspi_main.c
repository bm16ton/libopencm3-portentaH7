#include "qspi.h"
#include "cdc.h"
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/quadspi.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>


static inline __attribute__((always_inline)) void __WFI(void)
{
	__asm volatile ("wfi");
}

/*
// Reset handler: set the stack pointer and branch to main().
__attribute__( ( naked ) ) void reset_handler( void ) {
  // Set the stack pointer to the 'end of stack' value.
  __asm__( "LDR r0, =_estack\n\t"
           "MOV sp, r0" );
  // Branch to main().
  __asm__( "B main" );
}
*/
//SysTick interrupt handler.

// Simple blocking millisecond delay method.
void delay_ms( uint32_t ms ) {
  // Calculate the 'end of delay' tick value, then wait for it.
  uint32_t next = systick + ms;
  while ( systick < next ) { __WFI(); }
}

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

/**
 * Main program.
 */
int qspiinit( void ) {
  // TODO: Enable SRAM1, SRAM2, SRAM3.
  // Enable GPIOB, GPIOF, QSPI, UART4
    rcc_periph_clock_enable(RCC_QSPI);
    rcc_periph_clock_enable(RCC_CRC);

	/* PF6, PF7, and PF10 are Alt Function 9, PF8 and PF9 are Alt Function 10 */
	gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
						GPIO7 | GPIO10);
	gpio_set_output_options(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
						GPIO7 | GPIO10);
	gpio_set_af(GPIOF, GPIO_AF9, GPIO7 | GPIO10);

	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
						GPIO11 | GPIO12 | GPIO13);
	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
						GPIO11 | GPIO12 | GPIO13);
	gpio_set_af(GPIOD, GPIO_AF9, GPIO11 | GPIO12 | GPIO13);

	gpio_mode_setup(GPIOG, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
						GPIO6);
	gpio_set_output_options(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
						GPIO6);					
	gpio_set_af(GPIOG, GPIO_AF10, GPIO6);

  // Copy initialized data from .sidata (Flash) to .data (RAM)
//  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
//  memcpy( &_sitcm, &_siitcm, ( ( void* )&_eitcm - ( void* )&_sitcm ) );
//  memcpy( &_sdtcm, &_sidtcm, ( ( void* )&_edtcm - ( void* )&_sdtcm ) );
  // Clear the .bss section in RAM.
//  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );


  
  // Enable floating-point unit.
  SCB_CPACR      |=  ( 0xF << 20 );

  printf( "Configure QSPI...\r\n" );

  // QSPI peripheral setup.
  // Flash size: 16MiB = 2^(23+1) bytes.
  QUADSPI_DCR    |=  ( 23 << QUADSPI_DCR_FSIZE_SHIFT );
  // Set 24-bit addressing.
  QUADSPI_CCR    |=  ( 2 << QUADSPI_CCR_ADSIZE_SHIFT );
  // Set clock prescaler to 240 / ( 3 + 1 ) = 60MHz.
  QUADSPI_CR     |=  ( ( 3 << QUADSPI_CR_PRESCALE_SHIFT ) |
                        QUADSPI_CR_SSHIFT );
    printf( "Configure QSPI...after shifts \r\n" );
  // Send 'enable writes' command.
  qspi_wen( 1 );
printf( "Configure QSPI...after wen \r\n" );
  // Wait for the 'write enable latch' to be set.
  qspi_reg_wait( 0x05, 0x03, 0x02, 1 );
printf( "Configure QSPI...after reg wait \r\n" );
  // Send 'write volatile status register 2' to 0x02.
  // Clear instruction, mode and transaction phases.
  QUADSPI_CCR  &= ~( QUADSPI_CCR_INST_MASK | QUADSPI_CCR_FMODE_MASK |
                      QUADSPI_CCR_IMODE_MASK | QUADSPI_CCR_DMODE_MASK |
                      QUADSPI_CCR_ADMODE_MASK );
  printf( "Configure QSPI...after QUADSPI_CCR masks \r\n" );
  // Set 1-wire instruction and data modes, and auto-polling mode.
  QUADSPI_CCR  |=  ( ( 1 << QUADSPI_CCR_IMODE_SHIFT ) |
                      ( 1 << QUADSPI_CCR_DMODE_SHIFT ) );
                      
  printf( "Configure QSPI...after QUADSPI_CCR shifts \r\n" );
  // Enable the peripheral.
  QUADSPI_CR   |=  ( QUADSPI_CR_EN );
  printf( "Configure QSPI...after enable \r\n" );
  // Send information.
  QUADSPI_CCR  |=  ( 0x31 << QUADSPI_CCR_INST_SHIFT );
  QUADSPI_DR    =  ( 0x02 );
  printf( "Configure QSPI...after send \r\n" );
  // Wait for the transaction to finish.
//  while ( QUADSPI_SR & QUADSPI_SR_BUSY ) {};
  printf( "Configure QSPI...after QUADSPI_SR_BUSY \r\n" );
  // Disable the peripheral.
  QUADSPI_CR   &= ~( QUADSPI_CR_EN );
printf( "Configure QSPI...after QUADSPI disable \r\n" );
  // Wait for the register write to finish.
  qspi_reg_wait( 0x05, 0x01, 0x00, 1 );
printf( "Configure QSPI...after 2nd reg wait \r\n" );
  // Make sure that QSPI mode is enabled.
  qspi_reg_wait( 0x35, 0x02, 0x02, 1 );
printf( "Configure QSPI...after 3rdnd reg wait \r\n" );
  printf( "Done.\r\n" );

  // Test external Flash sector erase.
  //printf( "Erase sector...\r\n" );
  //qspi_erase_sector( 0 );
  //printf( "Done.\r\n" );

  // Test external Flash writes.
/*  printf( "Write words...\r\n" );
  qspi_write_word( 0x90000000, 0x01234567 );
  qspi_write_word( 0x90000002, 0x89ABCDEF );
  qspi_write_word( 0x90000004, 0xCABAFABA );
  printf( "Done.\r\n" );
*/
    printf( "Flash reads...\r\n" );
  // Test external Flash reads.
  QUADSPI_CCR  &= ~( QUADSPI_CCR_INST_MASK | QUADSPI_CCR_FMODE_MASK |
                      QUADSPI_CCR_IMODE_MASK | QUADSPI_CCR_DMODE_MASK |
                      QUADSPI_CCR_ADMODE_MASK );
  QUADSPI_CCR |= ( 3 << QUADSPI_CCR_FMODE_SHIFT |
                    3 << QUADSPI_CCR_ADMODE_SHIFT |
                    3 << QUADSPI_CCR_DMODE_SHIFT |
                    1 << QUADSPI_CCR_IMODE_SHIFT |
                    0xEB << QUADSPI_CCR_INST_SHIFT |
                    31 << QUADSPI_CCR_DCYC_SHIFT );
  QUADSPI_CR  |=  ( QUADSPI_CR_EN );

  // Add a dummy cycle; if memory-mapped access is attempted
  // immediately after enabling the peripheral, it seems to fail.
  // I'm not sure why, but adding one nop instruction seems to fix it.

  delay_ms( 1 );

  // Test reading values from memory-mapped Flash.
  int val = *( ( uint32_t* ) 0x90000000 );
  printf( "QSPI[0]: 0x%08X\r\n", val );
  val = *( ( uint32_t* ) 0x90000002 );
  printf( "QSPI[2]: 0x%08X\r\n", val );
  val = *( ( uint32_t* ) 0x90000004 );
  printf( "QSPI[4]: 0x%08X\r\n", val );
  val = *( ( uint32_t* ) 0x90000008 );
  printf( "QSPI[8]: 0x%08X\r\n", val );
  val = *( ( uint32_t* ) 0x9000000D );
  printf( "QSPI[13]: 0x%08X\r\n", val );
  val = *( ( uint32_t* ) 0x90000100 );
  printf( "QSPI[+]: 0x%08X\r\n", val );
  val = *( ( uint32_t* ) 0x90000000 );
  printf( "QSPI[0]: 0x%08X\r\n", val );

return 0;
}
