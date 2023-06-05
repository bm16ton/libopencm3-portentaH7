/* Author, Copyright: Oleg Borodin <onborodin@gmail.com> 2018 */

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/dma.h>
#include "tft_stm32_spi.h"
#include <stdlib.h>
#include "xpt2046.h"
#include <sys/types.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include "cdc.h"
#include "bulkspi.h"
#define Z_THRESHOLD     300
#define Z_THRESHOLD_INT	75
#define MSEC_THRESHOLD  7
/*
void ts_spi_setup(void) {

    gpio_mode_setup(TS_SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    TS_SCL_PIN | TS_CS_PIN | TS_MO_PIN | TS_MI_PIN);
    gpio_set_af(TS_SPI_PORT, GPIO_AF5, TS_SCL_PIN | TS_CS_PIN | TS_MO_PIN | TS_MI_PIN);
    gpio_set_output_options(TS_SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                            TS_MO_PIN | TS_SCL_PIN | TS_CS_PIN);

    spi_reset(TS_SPI);

    spi_disable(TS_SPI);
    spi_init_master(TS_SPI,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_128,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

    spi_set_full_duplex_mode(TS_SPI);
    spi_disable_software_slave_management(TS_SPI);
    spi_set_nss_high(TS_SPI);
    spi_set_master_mode(TS_SPI);
    spi_enable_ss_output(TS_SPI);
    spi_disable_crc(TS_SPI);

    spi_enable(TS_SPI);
}
*/

void my_spi_flush82(void);

int16_t threshholdv(void) {
int16_t z;
int16_t z1;
int16_t z2;

ts_get_data16(0xB1);
z1 = ts_get_data16(0xC1) >> 3;
z = z1 + 4095;
z2 = ts_get_data16(0x91) >> 3;
z -= z2;

return z;

}

void spifast(void);
void spislow(void);

void spislow(void) {
dma_disable(DMA1, DMA_CHANNEL1);
    spi_disable_tx_dma(TS_SPI);
    spi_disable(TS_SPI);
    for (unsigned i = 0; i < 350; i++)
	  {
		__asm__("nop");
	  }

    spi_set_baudrate_prescaler(TS_SPI, SPI_CFG1_MBR_CLK_DIV_256);
    spi_enable(TS_SPI);
    SPI_CR1(TS_SPI) |= SPI_CR1_CSTART;
    spi_enable_tx_dma(TS_SPI);
    dma_enable(DMA1, DMA_CHANNEL1);
    for (unsigned i = 0; i < 350; i++)
	  {
		__asm__("nop");
	  }

    spi_enable(TS_SPI);

    for (unsigned i = 0; i < 350; i++)
	  {
		__asm__("nop");
	  }
}

void spifast(void) {
    my_spi_flush82();
    dma_disable(DMA1, DMA_CHANNEL1);
    spi_disable_tx_dma(TS_SPI);
    spi_disable(TS_SPI);
    for (unsigned i = 0; i < 350; i++)
	  {
		__asm__("nop");
	  }

    spi_set_baudrate_prescaler(TS_SPI, SPI_CFG1_MBR_CLK_DIV_2);
    spi_enable(TS_SPI);
    SPI_CR1(TS_SPI) |= SPI_CR1_CSTART;
    spi_enable_tx_dma(TS_SPI);
    dma_enable(DMA1, DMA_CHANNEL1);
    for (unsigned i = 0; i < 350; i++)
	  {
		__asm__("nop");
	  }
}

void tscson(void) {
    TFT_CS_IDLE;
    TS_CS_ACTIVE;
}

void tscsoff(void) {
    TS_CS_IDLE;
    TFT_CS_ACTIVE;
//    gpio_clear(GPIOA, GPIO15);
}

void throwaway(uint8_t command) {
my_spi_send(TS_SPI, command);
}

void spitx(unsigned char d);

void spitx(unsigned char d) {
SPI_CR1(TS_SPI) |= SPI_CR1_CSTART;
while(!(SPI_SR(TS_SPI) & SPI_SR_TXP));
SPI_TXDR(TS_SPI)=d;
while( !( SPI_SR(TS_SPI) & SPI_SR_TXC));
}


void my_spi_send2(unsigned char d);

uint16_t tsbuf[64] = {0};
int spot2 = 0;

void my_spi_flush82(void)
{
while((SPI_SR(TS_SPI) & SPI_SR_RXWNE) || ( (SPI_SR(TS_SPI) & (SPI_SR_RXPLVL_MASK<<SPI_SR_RXPLVL_SHIFT)) !=0 ) )

tsbuf[spot2] = SPI_RXDR(TS_SPI);
printf("flush = %d\n", tsbuf[spot2]);
spot2 = spot2 + 1;
}

void my_spi_send2(unsigned char d)
{
SPI_CR1(TS_SPI) |= SPI_CR1_CSTART;
while(!(SPI_SR(TS_SPI) & SPI_SR_TXP));
SPI_TXDR8(TS_SPI)=d;
while( !( SPI_SR(TS_SPI) & SPI_SR_TXC));
my_spi_flush82();
}



uint16_t ts_get_data16(uint8_t command) {

//tscson();
//TS_CS_ACTIVE;
/*
            while (!(SPI_SR(TS_SPI) & SPI_SR_TXP));  //todo reading/writing should be one op
            my_spi_send8(TS_SPI, command);
            while (!(SPI_SR(TS_SPI) & SPI_SR_TXP)) {
                ;
            }
            my_spi_send8(TS_SPI, 0x00);
            while (!(SPI_SR(TS_SPI) & SPI_SR_TXP)) {
                ;
            }
		    uint8_t res1 = my_spi_flush(TS_SPI);
		    my_spi_send8(TS_SPI, 0x00);
            while (!(SPI_SR(TS_SPI) & SPI_SR_TXP)) {
                ;
            }
		    uint8_t res2 = my_spi_flush(TS_SPI);
*/

    my_spi_send2(command);
        for (unsigned i = 0; i < 400; i++)
	  {
		__asm__("nop");
	  }

    my_spi_send2(0x00);
    uint16_t res1 = (uint16_t)tsbuf[spot2 - 1];
//printf("res1 = %d\r", res1);

    my_spi_send2(0x00);
    uint16_t res2 = (uint16_t)tsbuf[spot2 - 1];
//    printf("res2 = %d\r", res2);
    spot2 = 0;
//TS_CS_IDLE;

    return ((res1 << 8) | (res2 && 0xFF)) >> 4;
}


uint16_t ts_get_x_raw(void) {
    int16_t res = 0;
    spislow();
//    tscson();
    for (uint8_t i = 0; i < TS_EVAL_COUNT; i++) {
        res += ts_get_data16(TS_COMM_X_SPOS);
    }
        spifast();
//    tscsoff();
//    TFT_CS_ACTIVE;
    return res / TS_EVAL_COUNT;
}

uint16_t ts_get_y_raw(void) {
    int16_t res = 0;
    spislow();
//    tscson();
    for (uint8_t i = 0; i < TS_EVAL_COUNT; i++) {
        res += ts_get_data16(TS_COMM_Y_SPOS);
    }
    spifast();
 //   tscsoff();
 //   TFT_CS_ACTIVE;
    return res / TS_EVAL_COUNT;
}

uint16_t ts_get_x(void) {
spislow();
//    tscson();
    uint16_t res = ts_get_x_raw();

    if (res >= TS_X_MAX_EDGE)
        return TS_X_SCREEN_MAX;
    if (res <= TS_X_MIN_EDGE)
        return TS_X_SCREEN_MIN;

    res = (TS_X_SCREEN_MAX * (res - TS_X_MIN_EDGE)) / (TS_X_MAX_EDGE - TS_X_MIN_EDGE);
        spifast();
 //   tscsoff();
//    TFT_CS_ACTIVE;
    return TS_X_SCREEN_MAX - res;

}

uint16_t ts_get_y(void) {
spislow();
//    tscson();
    uint16_t res = ts_get_y_raw();

    if (res >= TS_Y_MAX_EDGE)
        return TS_Y_SCREEN_MIN;
    if (res <= TS_Y_MIN_EDGE)
        return TS_Y_SCREEN_MIN;

    res = (TS_Y_SCREEN_MAX * (res - TS_Y_MIN_EDGE)) / (TS_Y_MAX_EDGE - TS_Y_MIN_EDGE);
        spifast();
//    tscsoff();
//    TFT_CS_ACTIVE;
    return res;
}

uint16_t ts_get_z1_raw(void) {
spislow();
//    tscson();
    uint16_t res = 0;
    for (uint8_t i = 0; i < TS_EVAL_COUNT; i++) {
        res += ts_get_data16(TS_COMM_Z1_POS);
    }
        spifast();
 //   tscsoff();
//    TFT_CS_ACTIVE;
    return res / TS_EVAL_COUNT;
}

uint16_t ts_get_z2_raw(void) {
spislow();
//    tscson();
    uint16_t res = 0;
    for (uint8_t i = 0; i < TS_EVAL_COUNT; i++) {
        res += ts_get_data16(TS_COMM_Z2_POS);
    }
        spifast();
//    tscsoff();
//    TFT_CS_ACTIVE;
    return res / TS_EVAL_COUNT;
}

/* EOF */
