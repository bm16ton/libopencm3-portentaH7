



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


void debug(void)
{
printplls();
put_status("usb debug spi");
ramtest();
	printf("spi2 clock freq  %ld \r\n", rcc_get_spi_clk_freq(SPI2));
    printf("qspi clock freq  %ld \r\n", rcc_get_qspi_clk_freq(RCC_QSPI));
	printf("fmc clock freq  %ld \r\n", rcc_get_fmc_clk_freq(RCC_FMC));
	printf("usart1 clock = %ld \r\n", rcc_get_usart_clk_freq(USART1));
	printf("cpu clock = %ld \r\n", rcc_get_bus_clk_freq(RCC_CPUCLK));
	printf("RCC_PERCLK = %ld \r\n", rcc_get_bus_clk_freq(RCC_PERCLK));
	printf("sysclock = %ld \r\n", rcc_get_bus_clk_freq(RCC_SYSCLK));
	printf("m4 core clock = %ld \r\n", rcc_get_core2_clk_freq());
	printf("I2C3 clock = %ld \r\n", rcc_get_i2c_clk_freq(I2C3));
}




void put_status(char *m)
{
	uint16_t stmp;

	printf(m);
	printf(" Status: ");
	stmp = SPI_SR(SPI5);
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

