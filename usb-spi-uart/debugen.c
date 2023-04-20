
#include <stdlib.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/dmamux.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/usart.h>
#include <stdio.h>
#include "debugen.h"
#include "cdc.h"
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
//silly functions I added to check all plls, seems like this probly exists in lopencm3 but I didnt see it
void printplls(void) {
printf("pll1p = %ld\r\n", rcc_get_pll1_clock('p'));
printf("pll1q = %ld\r\n", rcc_get_pll1_clock('q'));
printf("pll1r = %ld\r\n", rcc_get_pll1_clock('r'));
printf("pll2p = %ld\r\n", rcc_get_pll2_clock('p'));
printf("pll2q = %ld\r\n", rcc_get_pll2_clock('q'));
printf("pll2r = %ld\r\n", rcc_get_pll2_clock('r'));
printf("pll3p = %ld\r\n", rcc_get_pll3_clock('p'));
printf("pll3q = %ld\r\n", rcc_get_pll3_clock('q'));
printf("pll3r = %ld\r\n", rcc_get_pll3_clock('r'));
}

void dma_status(char *m)
{
//	int stmp;

    (void)m;
//	printf(" Status: ");
//	stmp = DMA1;

	if (DMA_HISR(DMA1) & DMA_HISR5_TCIE) {
		printf("tx complete HIGH irq 5, \r\n");
	}
	if (DMA_HISR(DMA1) & DMA_HISR11_TCIE) {
		printf("tx complete HIGH irq 11 ,\r\n ");
	}
	if (DMA_HISR(DMA1) & DMA_HISR21_TCIE) {
		printf("tx complete HIGH irq 21 , \r\n");
	}
	if (DMA_HISR(DMA1) & DMA_HISR27_TCIE) {
		printf("tx complete HIGH irq 27 , \r\n");
	}
	if (DMA_HISR(DMA1) & DMA_HISR3_TEIF) {
		printf("transfer error HIGH interrupt 3 , \r\n");
	}
	if (DMA_HISR(DMA1) & DMA_HISR9_TEIF) {
		printf("transfer error HIGH interrupt 9 , \r\n");
	}
	if (DMA_HISR(DMA1) & DMA_HISR19_TEIF) {
		printf("transfer error HIGH interrupt 19 , \r\n");
	}
	if (DMA_HISR(DMA1) & DMA_HISR25_TEIF) {
		printf("transfer error HIGH interrupt 25 , \r\n");
	}
	if (DMA_HISR(DMA1) & DMA_HISR0_CFEIF) {
		printf("FIFO error HIGH interrupt flag 0, \r\n");
	}
	if (DMA_HISR(DMA1) & DMA_HISR6_CFEIF) {
		printf("FIFO error HIGH interrupt flag 6, \r\n");
	}
	if (DMA_HISR(DMA1) & DMA_HISR16_CFEIF) {
		printf("FIFO error HIGH interrupt flag 16,\r\n ");
	}
	if (DMA_HISR(DMA1) & DMA_HISR22_CFEIF) {
		printf("FIFO error HIGH interrupt flag 22, \r\n");
	}
	if (DMA_LISR(DMA1) & DMA_LISR5_TCIE) {
		printf("tx complete LOW irq 5, \r\n");
	}
	if (DMA_LISR(DMA1) & DMA_LISR11_TCIE) {
		printf("tx complete LOW irq 11 ,\r\n ");
	}
	if (DMA_LISR(DMA1) & DMA_LISR21_TCIE) {
		printf("tx complete LOW irq 21 , \r\n");
	}
	if (DMA_LISR(DMA1) & DMA_LISR27_TCIE) {
		printf("tx complete LOW irq 27 , \r\n");
	}
	if (DMA_LISR(DMA1) & DMA_LISR3_TEIF) {
		printf("transfer error LOW interrupt 3 , \r\n");
	}
	if (DMA_LISR(DMA1) & DMA_LISR9_TEIF) {
		printf("transfer error LOW interrupt 9 , \r\n");
	}
	if (DMA_LISR(DMA1) & DMA_LISR19_TEIF) {
		printf("transfer error LOW interrupt 19 , \r\n");
	}
	if (DMA_LISR(DMA1) & DMA_LISR25_TEIF) {
		printf("transfer error LOW interrupt 25 ,\r\n ");
	}
	if (DMA_LISR(DMA1) & DMA_LISR0_CFEIF) {
		printf("FIFO error LOW interrupt flag 0, \r\n");
	}
	if (DMA_LISR(DMA1) & DMA_LISR6_CFEIF) {
		printf("FIFO error LOW interrupt flag 6, \r\n");
	}
	if (DMA_LISR(DMA1) & DMA_LISR16_CFEIF) {
		printf("FIFO error LOW interrupt flag 16, \r\n");
	}
	if (DMA_LISR(DMA1) & DMA_LISR22_CFEIF) {
		printf("FIFO error LOW interrupt flag 22, \r\n");
	}
//	printf("\r\n");
}


void spi_status(unsigned int spibase, char *m)
{
	uint16_t stmp;

	printf(m);
	printf(" Status: ");
	stmp = SPI_SR(spibase);
	if (stmp & SPI_SR_TXC) {
		printf("TXC, TX COMPLETE BUS IDLE ");
	}
	if (stmp & SPI_SR_RXWNE) {
		printf("RXWNE, ");
	}
	if (stmp & SPI_SR_EOT) {
		printf("EOT, ");
	}
	if (stmp & SPI_SR_OVR) {
		printf("OVERRUN, REC FULL, ");
	}
	if (stmp & SPI_SR_MODF) {
		printf("MODE FAULT, ");
	}
	if (stmp & SPI_SR_CRCE) {
		printf("CRCE, ");
	}
	if (stmp & SPI_SR_UDR) {
		printf("UNDERRUN, ");
	}
	if (stmp & SPI_SR_RXP) {
		printf("COMPLETE DATA PKT READY FOR RX, ");
	}
	if (stmp & SPI_SR_TXP) {
		printf("WRITE FINISHED, ");
	}
	if (stmp & SPI_SR_DXP) {
		printf("BOTH TX/RX EVENTS PENDING, ");
	}
	if (stmp & SPI_SR_SUSP) {
		printf("MASTER SUSPENDED, ");
	}
	printf("\r\n");
}

//GOOD CANIDATE FOR THE DEBUG HEADER
int ramtest(void) {
	  // Test external RAM reads and writes.
  uint32_t* sdram  = ( uint32_t* )0x60000000;
  uint16_t* sdramh = ( uint16_t* )0x60000000;
  uint8_t*  sdramb = ( uint8_t*  )0x60000000;
  printf( "RAM[0]: 0x%08lX (Uninitialized)\r\n", sdram[ 0 ] );
  sdram[ 0 ] = 0x01234567;
  sdram[ 1 ] = 0x89ABCDEF;
  printf( "RAM[0]: 0x%02X (Byte)\r\n", sdramb[ 0 ] );
  printf( "RAM[0]: 0x%04X (Halfword)\r\n", sdramh[ 0 ] );
  printf( "RAM[0]: 0x%08lX (Word)\r\n", sdram[ 0 ] );
  printf( "RAM[4]: 0x%08lX\r\n", sdram[ 1 ] );
  printf( "RAM[0]: 0x%02X (Byte)\r\n", sdramb[ 0 ] );
    sdram[ 0 ] = 0x01234567;
  sdram[ 1 ] = 0x89ABCDEF;
    printf( "RAM[0]: 0x%02X (Byte)\r\n", sdramb[ 0 ] );
  printf( "RAM[0]: 0x%04X (Halfword)\r\n", sdramh[ 0 ] );
  printf( "RAM[0]: 0x%08lX (Word)\r\n", sdram[ 0 ] );
  printf( "RAM[4]: 0x%08lX\r\n", sdram[ 1 ] );
  printf( "RAM[0]: 0x%02X (Byte)\r\n", sdramb[ 0 ] );

  return 0; // lol
}

void usbdebug(void)
{
printplls();
spi_status(SPI2, "usb debug spi2");
spi_status(SPI5, "usb debug spi5");
dma_status("dma status");
printplls();
ramtest();
	printf("spi2 clock freq  %ld \r\n", rcc_get_spi_clk_freq(SPI2));
	printf("spi5 clock freq  %ld \r\n", rcc_get_spi_clk_freq(SPI5));
    printf("qspi clock freq  %ld \r\n", rcc_get_qspi_clk_freq(RCC_QSPI));
	printf("fmc clock freq  %ld \r\n", rcc_get_fmc_clk_freq(RCC_FMC));
	printf("usart1 clock = %ld \r\n", rcc_get_usart_clk_freq(USART1));
	printf("cpu clock = %ld \r\n", rcc_get_bus_clk_freq(RCC_CPUCLK));
	printf("RCC_PERCLK = %ld \r\n", rcc_get_bus_clk_freq(RCC_PERCLK));
	printf("sysclock = %ld \r\n", rcc_get_bus_clk_freq(RCC_SYSCLK));
	printf("m4 core clock = %ld \r\n", rcc_get_core2_clk_freq());
	printf("I2C3 clock = %ld \r\n", rcc_get_i2c_clk_freq(I2C3));
}
