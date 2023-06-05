#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include "timing_stm32.h"
#include "st7789_stm32_spi.h"
#include "fonts/font_fixedsys_mono_24.h"
#include "fonts/font_ubuntu_48.h"
#include "ILI9486_Defines.h"
#include "platform.h"

#include "fonts/bitmap_typedefs.h"
#include "adclive.h"
#include "pos.h"

int liveadc = 0;
static void adc_dma2_setup(void);
uint32_t adc_liveonce(void);
uint32_t adc_liveonce2(void);
volatile static uint16_t adc_res[32];

void adc_livestart(void)
{
    static uint8_t channel_seq[16];
    rcc_periph_clock_enable(RCC_DMA2);
    adc_dma2_setup();
	rcc_periph_clock_enable(RCC_ADC1);

    st_fill_screen(ST_COLOR_RED);
//	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
//	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO6);
	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_ADC2);
	adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
	adc_power_off(ADC1);
//	adc_power_off(ADC2);

    adc_enable_scan_mode(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
    adc_disable_discontinuous_mode_regular(ADC1);
        adc_enable_external_trigger_regular(ADC1, ADC_CR2_SWSTART, ADC_CR2_EXTEN_DISABLED);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28CYC);
	adc_set_resolution(ADC1, 12);
    channel_seq[0] = ADC_CHANNEL4;
    channel_seq[1] = ADC_CHANNEL6;

    adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);

    adc_set_regular_sequence(ADC1, 1, channel_seq);
    adc_enable_dma(ADC1);
    adc_set_dma_continue(ADC1);
	adc_power_on(ADC1);
	adc_start_conversion_regular(ADC1);
//	adc_power_on(ADC2);
//	adc_set_regular_sequence(ADC1, 1, channels);
//	adc_start_conversion_regular(ADC1);
//    adc_res[0] = adc_read_regular(ADC1);
/*   adc_power_off(ADC1);
    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
    adc_enable_scan_mode(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
//    adc_disable_discontinuous_mode_regular(ADC1);
//    adc_enable_external_trigger_regular(ADC1, ADC_CR2_SWSTART, ADC_CR2_EXTEN_DISABLED);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28CYC);
    adc_set_resolution(ADC1, 12);
    uint8_t channels[] = {ADC_CHANNEL2};
    adc_set_multi_mode(ADC_CCR_MULTI_DUAL_INTERLEAVED);
    //ADC_CCR_MULTI_INDEPENDENT
    //ADC_CCR_MULTI_TRIPLE_INTERLEAVED
    adc_set_regular_sequence(ADC1, 1, channels);
    adc_enable_dma(ADC1);
    adc_set_dma_continue(ADC1);
    adc_power_on(ADC1);
*/
//    adc_start_conversion_regular(ADC1);
     st_set_address_window(1, 1, 320, 480);
     st_fill_screen_nodma(ST_COLOR_RED);
    
    put_status("after adclive fillscreen");
//    adc_liveonce();
}

static void adc_dma2_setup(void) {

    dma_stream_reset(DMA2, DMA_STREAM0);
    dma_set_priority(DMA2, DMA_STREAM0, DMA_SxCR_PL_LOW);

    dma_set_memory_size(DMA2, DMA_STREAM0, DMA_SxCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA2, DMA_STREAM0, DMA_SxCR_PSIZE_16BIT);

    dma_enable_memory_increment_mode(DMA2, DMA_STREAM0);
    dma_enable_circular_mode(DMA2, DMA_STREAM0);

    dma_set_transfer_mode(DMA2, DMA_STREAM0, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_peripheral_address(DMA2, DMA_STREAM0, (uint32_t) & ADC_DR(ADC1));
    dma_set_memory_address(DMA2, DMA_STREAM0, (uint32_t) &adc_res[0]);
    dma_set_number_of_data(DMA2, DMA_STREAM0, 2);
dma_set_dma_flow_control(DMA2, DMA_STREAM0);

    dma_channel_select(DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_0);

    //nvic_enable_irq(NVIC_DMA2_STREAM4_IRQ);
    //dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM4);
    dma_enable_stream(DMA2, DMA_STREAM0);
}

void dma2_stream0_isr(void) {
    //dma_clear_interrupt_flags(DMA2, DMA_CHANNEL4, DMA_IFCR_CGIF1);
}


uint16_t adcolor = 0xFFE0;
uint16_t adcolor2 = 0x001F;
static char ret[] = "0.00 ";
static char ret2[] = "0.00 ";

 static int lastxcord;
 static int lastycord;
 static int curxcord;
 static int curycord;

 static int lastxcord2;
 static int lastycord2;
 static int curxcord2;
 static int curycord2;
 
void adclive(void) {
/**
 * Draw a line from (x0,y0) to (x1,y1) with `width` and `color`.
 * @param x0 start column address.
 * @param y0 start row address.
 * @param x1 end column address.
 * @param y1 end row address.
 * @param width width or thickness of the line
 * @param color 16-bit RGB565 color of the line
 */
    timer_disable_irq(TIM2, NVIC_TIM2_IRQ); 
 
    while (SPI_SR(SPI3) & SPI_SR_BSY) {
        ;
    }


 while (1) {

    curycord = adc_liveonce();
    curycord2 = (adc_liveonce2() + 20);
    printf("curycord = %d\r\n", curycord);
    printf("curycord2 = %d\r\n", curycord2);
    curxcord = lastxcord + 13;
    curxcord2 = lastxcord2 + 13;


//    printf("curxcord = %d\r\n", curxcord);
    if (curxcord >= 490) {
    curxcord = 1;
    lastxcord = 1;
    if (adcolor == 0xFFE0)
        {
        adcolor = 0x001F;
        } else {
        adcolor = 0xFFE0;
        }
    }
      
    if (curxcord2 >= 490) {
    curxcord2 = 1;
    lastxcord2 = 1;
    if (adcolor2 == 0x001F)
        {
        adcolor2 = 0xFFE0;
        } else {
        adcolor2 = 0x001F;
        }
    }
 
    while (SPI_SR(SPI3) & SPI_SR_BSY) {
        ;
    }

   gfx_drawLine(lastxcord, lastycord, curxcord, curycord, adcolor);
// st_draw_line(lastxcord, lastycord, curxcord, curycord, 7, adcolor);
    gfx_drawLine(lastxcord2, lastycord2, curxcord2, curycord2, adcolor2);

//st_draw_line(lastxcord2, lastycord2, curxcord2, curycord2, 7, adcolor2);
    st_fill_rect_fast(1, 5, 320, 25, ILI9486_RED);  //105  50

    st_draw_string(105, 5, ret, ST_COLOR_DARKGREEN, &font_fixedsys_mono_24);
//    st_fill_rect_fast(1, 5, 320, 25, ILI9486_RED); //165    50
    st_draw_string(165, 5, ret2, ST_COLOR_DARKGREEN, &font_fixedsys_mono_24);
  
    put_status("after draw lines/strings");
   
    lastxcord = curxcord;
    lastycord = curycord;

    lastxcord2 = curxcord2;
    lastycord2 = curycord2;

  }
} 


uint32_t adc_liveonce(void)
{

	unsigned value;
	uint32_t read1[6];
    

//	for (unsigned i = 0; i < 20; i++)
//	  {
//		__asm__("nop");
//	  }

	value = adc_res[0];
	
//    value = adc_res[0];
	value *= 3379; /* 3.3 * 1024 == 3379.2 */
	value += 104858; /* round, 0.05V * 2 ^ 21 == 104857.6 */
	ret[0] = (value >> 21) + '0';
	value &= (1 << 21) - 1;
	value *= 10;
	ret[2] = (value >> 21) + '0';
    read1[0] = ret[0];
    read1[1] = ret[1];
    read1[2] = ret[2];
    read1[3] = ret[3];
    read1[4] = ret[4];
    read1[5] = ret[5];
    read1[6] = ret[6];
    printf("ret %s\r\n", (char*)ret);
    
    delay(500);

return *read1;
}

uint32_t adc_liveonce2(void)
{
unsigned value2;
uint32_t read2[6];

value2 = adc_res[1];

    value2 *= 3379; /* 3.3 * 1024 == 3379.2 */
	value2 += 104858; /* round, 0.05V * 2 ^ 21 == 104857.6 */
	ret2[0] = (value2 >> 21) + '0';
	value2 &= (1 << 21) - 1;
	value2 *= 10;
	ret2[2] = (value2 >> 21) + '0';
    read2[0] = ret2[0];
    read2[1] = ret2[1];
    read2[2] = ret2[2];
    read2[3] = ret2[3];
    read2[4] = ret2[4];
    read2[5] = ret2[5];
    read2[6] = ret2[6];
    printf("ret2 %s\r\n", (char*)ret2);
    delay(500);
    
    return *read2;
}
