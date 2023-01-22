#include "qspi2.h"
#include "cdc.h"
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/quadspi.h>
#include <libopencm3/cm3/scb.h>

void quad_setup(void)
{
/* the usual enable and setup pins for qspi
*/
rcc_periph_clock_enable(RCC_QSPI);
rcc_periph_reset_pulse(RST_QSPI);

	gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
						GPIO7 | GPIO10);
	gpio_set_output_options(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
						GPIO7 | GPIO10);
	gpio_set_af(GPIOF, GPIO_AF9, GPIO7 | GPIO10);

	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE,
						GPIO11 | GPIO12 | GPIO13);
	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
						GPIO11 | GPIO12 | GPIO13);
	gpio_set_af(GPIOD, GPIO_AF9, GPIO11 | GPIO12 | GPIO13);

	gpio_mode_setup(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE,
						GPIO6);
	gpio_set_output_options(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
						GPIO6);					
	gpio_set_af(GPIOG, GPIO_AF10, GPIO6);

QUADSPI_CR= 80<<QUADSPI_CR_PRESCALE_SHIFT   //was 80
	    | 3 << QUADSPI_CR_FTHRES_SHIFT
	    |    QUADSPI_CR_EN ;
QUADSPI_DCR = 23<<QUADSPI_DCR_FSIZE_SHIFT
		| 7 << QUADSPI_DCR_CSHT_SHIFT ;
QUADSPI_ABR =0;
QUADSPI_DLR =16;
QUADSPI_AR =0;


QUADSPI_CCR=0x00000166;             // enable reset 66  Page 70 in the datasheet
while( (QUADSPI_SR & 32) == 32 );
QUADSPI_CCR=0x00000199;            // soft reset  Page 70 
while( (QUADSPI_SR & 32) == 32 );
while(!(QUADSPI_SR & 0x04));
while(!(QUADSPI_SR & 0x02)); /* Wait for TCF flag to be set */
QUADSPI_CCR=0x00000138;            // enable quadspi      Page 68
while( (QUADSPI_SR & 32) == 32 );
while(!(QUADSPI_SR & 0x02)); /* Wait for TCF flag to be set */
/* W25Q64 is now in the qspi mode
*/
}

void quad_map(void)
{
/* Page 31 in the data sheet
following keep in mind :
we are sending command in 4 bit mode, we are sending address in 4 bit mode, we have 2 dummy cycle in 4 bit mode and we receive data in 4 bit mode.
the Address len acording datasheet 24bit aka 3 bytes
and we'd like to map the flash to memory.

datasheet of h7:
FMODE : 3 Memory map
DCYC: 2 dummy cycles
ABSIZE: 32Bit
ABMODE: No alternative bytes
ADSIZE: 24 Bit according Flashdevice
ADMODE: Using 4 Lines for sending address
IMODE: Send command via 4 Lines
DMODE: Receive data via 4 lines

Command is Fast Read (0Bh) in QPI Mode Page 31

*/

QUADSPI_CCR = 3 << QUADSPI_CCR_FMODE_SHIFT
		| 2 << QUADSPI_CCR_DCYC_SHIFT
		| 0 << QUADSPI_CCR_ABMODE_SHIFT
		| 3 << QUADSPI_CCR_ABSIZE_SHIFT
		| 3 << QUADSPI_CCR_ADMODE_SHIFT
		| 2 << QUADSPI_CCR_ADSIZE_SHIFT
		| 3 << QUADSPI_CCR_IMODE_SHIFT
		| 3 << QUADSPI_CCR_DMODE_SHIFT
	//	| QUADSPI_CCR_SIOO
	//	| QUADSPI_CCR_DDRM
  | 0x0B ;                    //  quad fast read in qspi 
while( (QUADSPI_SR & 32) == 32 );

}

void quad_read(uint32_t addr, size_t len, uint8_t *dest)
{

/* Same as Above on MAP, difference is
FMODE is READ
DLR is set to len-1
AR to flash Adress
*/

QUADSPI_CR &= ~  QUADSPI_CR_EN;
QUADSPI_DLR = len -1 ;
//QUADSPI_DLR = len  ;
QUADSPI_CCR = 1 << QUADSPI_CCR_FMODE_SHIFT
		| 2 << QUADSPI_CCR_DCYC_SHIFT
		| 0 << QUADSPI_CCR_ABMODE_SHIFT
		| 3 << QUADSPI_CCR_ABSIZE_SHIFT
		| 3 << QUADSPI_CCR_ADMODE_SHIFT
		| 2 << QUADSPI_CCR_ADSIZE_SHIFT
		| 3 << QUADSPI_CCR_IMODE_SHIFT
		| 3 << QUADSPI_CCR_DMODE_SHIFT
	//	| QUADSPI_CCR_SIOO
	//	| QUADSPI_CCR_DDRM
  | 0x0B ;                    //  quad fast read in qspi
	QUADSPI_CR |=   QUADSPI_CR_EN;
QUADSPI_ABR =0;
QUADSPI_AR = addr;

while(len)
  {
	//	while(!(QUADSPI_SR & 0x02));
	 while((QUADSPI_SR & 32) != 32);
	*(uint32_t *)dest = QUADSPI_DR;
	dest+=4;
    len-=4;

  }

QUADSPI_FCR=0x1b;          //CLEAR CTFC Flag
QUADSPI_CR &= ~  QUADSPI_CR_EN;

}

void quad_write(uint32_t addr, size_t len, uint8_t *dest)
{

QUADSPI_FCR=2;          //CLEAR CTFC Flag
    QUADSPI_CR |=   QUADSPI_CR_EN;
	QUADSPI_CCR = 0 << QUADSPI_CCR_FMODE_SHIFT
		| 0 << QUADSPI_CCR_DCYC_SHIFT
		| 0 << QUADSPI_CCR_ABMODE_SHIFT
		| 0 << QUADSPI_CCR_ABSIZE_SHIFT
		| 0 << QUADSPI_CCR_ADMODE_SHIFT
		| 2 << QUADSPI_CCR_ADSIZE_SHIFT
		| 3 << QUADSPI_CCR_IMODE_SHIFT
		| 0 << QUADSPI_CCR_DMODE_SHIFT
	//	| QUADSPI_CCR_SIOO
	//	| QUADSPI_CCR_DDRM
  | 0x06 ;                    //  quad write enable in qspi 



 	while( (QUADSPI_SR & 32) == 32 );
	
	
		QUADSPI_FCR=2;          //CLEAR CTFC Flag
		QUADSPI_CR &= ~  QUADSPI_CR_EN;

QUADSPI_DLR = len -1 ;
QUADSPI_CCR = 0 << QUADSPI_CCR_FMODE_SHIFT
| 0 << QUADSPI_CCR_DCYC_SHIFT
| 0 << QUADSPI_CCR_ABMODE_SHIFT
| 3 << QUADSPI_CCR_ABSIZE_SHIFT
| 3 << QUADSPI_CCR_ADMODE_SHIFT
| 2 << QUADSPI_CCR_ADSIZE_SHIFT
| 3 << QUADSPI_CCR_IMODE_SHIFT
| 3 << QUADSPI_CCR_DMODE_SHIFT
// | QUADSPI_CCR_SIOO
// | QUADSPI_CCR_DDRM
| 0x02 ; // quad write page in qspi
QUADSPI_CR |= QUADSPI_CR_EN;
QUADSPI_ABR =0;
QUADSPI_AR = addr;

while(len)
  {
	while(!(QUADSPI_SR & 0x04));
    QUADSPI_DR = *(uint32_t *)dest;
	dest+=4;
    len-=4;

  }

QUADSPI_FCR=2;          //CLEAR CTFC Flag
	QUADSPI_CR &= ~  QUADSPI_CR_EN;

}

void quad_erase(uint32_t addr)
{

QUADSPI_FCR=2;          //CLEAR CTFC Flag
    QUADSPI_CR |=   QUADSPI_CR_EN;
	QUADSPI_CCR = 0 << QUADSPI_CCR_FMODE_SHIFT
		| 0 << QUADSPI_CCR_DCYC_SHIFT
		| 0 << QUADSPI_CCR_ABMODE_SHIFT
		| 0 << QUADSPI_CCR_ABSIZE_SHIFT
		| 0 << QUADSPI_CCR_ADMODE_SHIFT
		| 2 << QUADSPI_CCR_ADSIZE_SHIFT
		| 3 << QUADSPI_CCR_IMODE_SHIFT
		| 0 << QUADSPI_CCR_DMODE_SHIFT
	//	| QUADSPI_CCR_SIOO
	//	| QUADSPI_CCR_DDRM
  | 0x06 ;                    //  quad write enable in qspi 



 	while( (QUADSPI_SR & 32) == 32 );
	
	
		QUADSPI_FCR=2;          //CLEAR CTFC Flag
		QUADSPI_CR &= ~  QUADSPI_CR_EN;

		QUADSPI_CR |=   QUADSPI_CR_EN;
		QUADSPI_CCR = 0 << QUADSPI_CCR_FMODE_SHIFT
		| 0 << QUADSPI_CCR_DCYC_SHIFT
		| 0 << QUADSPI_CCR_ABMODE_SHIFT
		| 0 << QUADSPI_CCR_ABSIZE_SHIFT
		| 3 << QUADSPI_CCR_ADMODE_SHIFT
		| 2 << QUADSPI_CCR_ADSIZE_SHIFT
		| 3 << QUADSPI_CCR_IMODE_SHIFT
		| 0 << QUADSPI_CCR_DMODE_SHIFT
	//	| QUADSPI_CCR_SIOO
	//	| QUADSPI_CCR_DDRM
  | 0x20 ;                    //  quad erase in qspi 	

		//	QUADSPI_ABR =0;
QUADSPI_AR = addr;

	while( (QUADSPI_SR & 32) == 32 );

    	QUADSPI_CR &= ~  QUADSPI_CR_EN;
		QUADSPI_FCR=2;          //CLEAR CTFC Flag
		
	QUADSPI_CR |=   QUADSPI_CR_EN;
QUADSPI_CCR = 0 << QUADSPI_CCR_FMODE_SHIFT
		| 0 << QUADSPI_CCR_DCYC_SHIFT
		| 0 << QUADSPI_CCR_ABMODE_SHIFT
		| 0 << QUADSPI_CCR_ABSIZE_SHIFT
		| 0 << QUADSPI_CCR_ADMODE_SHIFT
		| 2 << QUADSPI_CCR_ADSIZE_SHIFT
		| 3 << QUADSPI_CCR_IMODE_SHIFT
		| 0 << QUADSPI_CCR_DMODE_SHIFT
	//	| QUADSPI_CCR_SIOO
	//	| QUADSPI_CCR_DDRM
  | 0x04 ;                    //  quad write disable in qspi 
		
while( (QUADSPI_SR & 32) == 32 );
		QUADSPI_FCR=2;          //CLEAR CTFC Flag
QUADSPI_CR &= ~  QUADSPI_CR_EN;

}
