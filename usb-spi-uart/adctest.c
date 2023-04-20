#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dwc/otg_common.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include "adctest.h"
#include "tft_stm32_spi.h"
#include "fonts/font_fixedsys_mono_24.h"
#include "fonts/font_fixedsys_mono_16.h"
#include "fonts/font_ubuntu_48.h"
#include "usb.h"
#include "fonts/bitmap_typedefs.h"
//#include "adclive.h"
#include "usb-2-i2c/pos.h"
#include "ILI9486_Defines.h"
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/dmamux.h>

int liveadc = 0;
volatile static uint16_t adc_res[32];
bool is_configured = true;
static int configured;
int humanreadable = 1;
#define SAMPLES_PER_MESSAGE 50
int num_samples = 0;
uint32_t samples[SAMPLES_PER_MESSAGE] __attribute__ ((aligned(4)));
//static void send_samples(uint8_t ep);
static void adc_once(void);
static char ret2[] = "0.0V ";
uint8_t ADC_CHANNEL2 = 18;
const unsigned char CHAN4[] = { 18, 3 };
uint8_t ADC_CHANNEL3 = 3;
const unsigned char CHAN3[] = { 3 };

static enum usbd_request_return_codes adc_control_request(usbd_device *dev,
	struct usb_setup_data *req,
	uint8_t **buf,
	uint16_t *len,
	usbd_control_complete_callback *complete)
{
	(void) dev;
	(void) complete;
	(void) buf;
//    (void) req;
    (void) len;

   if (req->bRequest == 249)
     {
        if ( req->wIndex == 0 )
			{
            adcswitch(0);
			return USBD_REQ_HANDLED;
			}
        if ( req->wIndex == 1 )
			{
			adcswitch(1);
			return USBD_REQ_HANDLED;
			}
	 }
	return USBD_REQ_NEXT_CALLBACK;
}

void adcswitch(int onoff) {
adc_power_off(ADC1);

for (unsigned i = 0; i < 30; i++)
	  {
		samples[i] = '0';
	  }

num_samples = 0;
printf("fuuuck\r\n");
humanreadable = onoff;
printf("humanreadable = %d\r\n", humanreadable);
adc_start();
}

void adc_start(void)
{
//static uint8_t channel_seq[16];
rcc_periph_clock_enable(RCC_ADC12);
rcc_periph_clock_enable(RCC_GPIOC);
rcc_periph_clock_enable(RCC_GPIOA);
adc_power_off(ADC1);
gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);
gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO6);
;
   adc_enable_dual_mode(ADC1);
    adc_set_dual_mode_type(ADC1, ADC_CCR_INDEPENDENT);
 //   adc_set_dual_mode_data_format(ADC1, ADC_CCR_DAMDF_NONE);
    adc_set_continuous_conversion_mode(ADC1);
    adc_enable_dma_circular_mode(ADC1);
//    adc_disable_discontinuous_mode_regular(ADC1);
        adc_set_right_aligned(ADC1);
//    adc_enable_external_trigger_regular(ADC1, ADC_CR2_SWSTART, ADC_CR2_EXTEN_DISABLED);
    enable_pa1so(0);
	adc_set_clock_param(ADC1, ADC_CCR_CKMODE_DIV2, ADC_CCR_PRESC_2);
	adc_set_sample_time(ADC1, ADC_CHANNEL2, ADC_SMPR_SMP_2DOT5CYC);
	adc_set_sample_time(ADC1, ADC_CHANNEL3, ADC_SMPR_SMP_2DOT5CYC);
	adc_set_resolution(ADC1, 12);
//	channel_seq[0] = ADC_CHANNEL2;
//    channel_seq[1] = ADC_CHANNEL3;
    adc_set_dma_dmngt_circ(ADC1);
	adc_set_pcsel(ADC1, 2, CHAN4);
	adc_calibrate(ADC1);
//	uint8_t channels[] = {ADC_CHANNEL2};
//	adc_set_multi_mode(ADC_CCR_MULTI_DUAL_INTERLEAVED);
    adc_enable_regulator(ADC1);
	for (uint32_t loop = 0; loop < 100; ++loop) {
        __asm__("nop");
    }
	adc_power_on(ADC1);
	adc_enable_dma(ADC1);
//	adc_set_regular_sequence(ADC1, 2, channel_seq);
//	adc_start_conversion_regular(ADC1);
/*
    adc_set_continuous_conversion_mode(ADC1);
//    adc_disable_discontinuous_mode_regular(ADC1);
//    adc_enable_external_trigger_regular(ADC1, ADC_CR2_SWSTART, ADC_CR2_EXTEN_DISABLED);
    adc_set_right_aligned(ADC1);
    uint8_t channels[] = {CHAN4};
  */
uint8_t channels[] = {ADC_CHANNEL2, ADC_CHANNEL3 };
    adc_set_regular_sequence(ADC1, 2, channels);
    adc_start_conversion_regular(ADC1);

adc_start_conversion_regular(ADC1);

     st_set_address_window(1, 1, 320, 480);
     st_fill_screen_nodma(ST_COLOR_RED);
}

void adc_dma_setup(void) {

    nvic_enable_irq(NVIC_DMA1_STR4_IRQ);
    nvic_set_priority(NVIC_DMA1_STR4_IRQ, 5);
    dma_stream_reset(DMA1, DMA_STREAM4);
    dma_set_priority_level(DMA1, DMA_STREAM4, DMA_SxCR_PL_LOW);

    dma_set_memory_size(DMA1, DMA_STREAM4, DMA_SxCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM4, DMA_SxCR_PSIZE_16BIT);
    dma_circ_enable(DMA1, DMA_STREAM4, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM4);
dma_enable_peripheral_increment_mode(DMA1, DMA_STREAM4);
dma_set_as_flow_controller(DMA1, DMA_STREAM4);
    dma_set_transfer_mode(DMA1, DMA_STREAM4, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_peripheral_address(DMA1, DMA_STREAM4, (uint32_t) & ADC_CDR(ADC1));
    dma_set_memory_address(DMA1, DMA_STREAM4, (uint32_t) &adc_res[0]);
    dma_set_number_of_data(DMA1, DMA_STREAM4, 32);
dma_set_as_flow_controller(DMA1, DMA_STREAM4);
	dma_set_fifo_threshold(DMA1, DMA_STREAM4, DMA_THRESHOLD_QUARTER);
	dma_enable_bufferable_transfers(DMA1, DMA_STREAM4);

dmamux_set_dma_channel_request(DMAMUX1, 4, 9);
dma_enable(DMA1, DMA_STREAM4);
    dma_enable_stream(DMA1, DMA_STREAM4);

}

void dma1_str4_isr(void) {
   dma_clear_interrupt_flags(DMA1, DMA_STREAM4);

}

uint16_t adcolor = 0xFFE0;
uint16_t adcolor2 = 0x001F;
static char ret[] = "0.00 ";
int oldret;
int oldret2;

 static int lastxcord;
 static int lastycord;
 static int curxcord;
 static int curycord;
 int adclivestop = 0;
 static int lastxcord2;
 static int lastycord2;
 static int curxcord2;
 static int curycord2;
static int bootrun = 0;
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

    if (adclivestop == 1) {
    return;
    }

    while(!(SPI_SR(SPI5) & SPI_SR_TXP));
    while( !( SPI_SR(SPI5) & SPI_SR_TXC)) {
    ;
    }
//    st_draw_string_withbg(1, 16, "3.3", ILI9486_BLACK, ILI9486_RED, &font_fixedsys_mono_16);
//st_draw_line(17, 42, 480, 42, 1, ILI9486_BLACK);
//st_draw_line(17, 65, 480, 65, 1, ILI9486_BLACK);
//st_draw_string_withbg(1, 30, "1.6", ILI9486_BLACK, ILI9486_RED, &font_fixedsys_mono_16);
st_draw_string_withbg(105, 5, "0.00", ST_COLOR_DARKGREEN, ILI9486_RED, &font_fixedsys_mono_16);
st_draw_string_withbg(165, 5, "0.00", ST_COLOR_DARKGREEN, ILI9486_RED, &font_fixedsys_mono_16);
 while (1) {

    curycord = 115 - adc_liveonce();
    curycord2 = 135 - adc_liveonce2();
//    printf("curycord = %d\r\n", curycord);
//    printf("curycord2 = %d\r\n", curycord2);
    curxcord = lastxcord + 13;
    curxcord2 = lastxcord2 + 13;


//    printf("curxcord = %d\r\n", curxcord);
    if (curxcord >= 490) {
    curxcord = 1;
    lastxcord = 1;
    if (bootrun == 0) {
    st_fill_rect_fast(1, 1, 33, 90, ILI9486_RED);

    bootrun = 1;
    }

    st_fill_rect_fast(1, 45, 480, 60, ILI9486_RED);
    st_fill_rect_fast(1, 118, 480, 25, ILI9486_RED);

//    if (adcolor == 0xFFE0)
//        {
        adcolor = 0x001F;
//        } else {
//        adcolor = 0xFFE0;
 //       }
    }

    if (curxcord2 >= 490) {
    curxcord2 = 1;
    lastxcord2 = 1;
//    if (adcolor2 == 0x001F)
//        {
        adcolor2 = 0xFFE0;
//        } else {
//        adcolor2 = 0x001F;
//        }
    }

    while(!(SPI_SR(SPI5) & SPI_SR_TXP));
    while( !( SPI_SR(SPI5) & SPI_SR_TXC)) {
    ;
    }

    printf("curycord = %d\r", curycord);

    if ((85 - curycord) <= 15) {
        adcolor = 0x001F;
    } else if ((85 - curycord) <= 30) {
        adcolor = 0xF81F;
    } else if ((85 - curycord) <= 45) {
        adcolor = 0xFFE0;
    } else if ((85 - curycord) <= 68) {
       adcolor =  0xFFFF;
    }
    st_draw_line(lastxcord, lastycord, curxcord, curycord, 2, adcolor);
    st_draw_line(lastxcord2, lastycord2, curxcord2, curycord2, 2, adcolor2);
    int poo = atoi(ret);
    int poo2 = atoi(ret2);
     if (oldret != poo) {
    st_draw_string_withbg(105, 5, ret, ST_COLOR_DARKGREEN, ILI9486_RED, &font_fixedsys_mono_16);
    oldret = poo;
    } else if (oldret2 != poo2) {
    st_draw_string_withbg(165, 5, ret2, ST_COLOR_DARKGREEN, ILI9486_RED, &font_fixedsys_mono_16);

    oldret2 = poo2;
    }

    lastxcord = curxcord;
    lastycord = curycord;

    lastxcord2 = curxcord2;
    lastycord2 = curycord2;

  }
}

unsigned adc_liveonce(void)
{
	unsigned value;
    unsigned poop3;

	value = adc_res[0];
printf("value = %d\r",value);
poop3 = (value / 31);
	value *= 3379; /* 3.3 * 1024 == 3379.2 */
	value += 104858; /* round, 0.05V * 2 ^ 21 == 104857.6 */
	ret[0] = (value >> 21) + '0';
	value &= (1 << 21) - 1;
	value *= 10;
	ret[2] = (value >> 21) + '0';

// poop3 = atoi(ret);

	for (uint32_t loop = 0; loop < 1111500; ++loop) {
        __asm__("nop");
    }
    printf("poop3 = %d\r", poop3);
return poop3 * 2;
}

uint32_t adc_liveonce2(void)
{
    unsigned value2;
    uint32_t poop4;
    value2 = adc_res[17];

    value2 *= 3379; /* 3.3 * 1024 == 3379.2 */
	value2 += 104858; /* round, 0.05V * 2 ^ 21 == 104857.6 */
	ret2[0] = (value2 >> 21) + '0';
	value2 &= (1 << 21) - 1;
	value2 *= 10;
	ret2[2] = (value2 >> 21) + '0';
    poop4 = atoi(ret2);

	for (uint32_t loop = 0; loop < 1111500; ++loop) {
        __asm__("nop");
    }
    return poop4 * 10;
}

void adc_always(void)
{
    static char ret3[] = "0.0V ";
    static char oldret69[] = "0.0V ";
	uint8_t channels[] = { ADC_CHANNEL2, };
	uint32_t value;

	adc_set_regular_sequence(ADC1, 1, channels);
	adc_start_conversion_regular(ADC1);
	while (!(adc_eoc(ADC1)))
	;
    for (unsigned p = 0; p < 330; p++)
	  {
		__asm__("nop");
	  }
	value = adc_read_regular(ADC1);
	value *= 3379; /* 3.3 * 1024 == 3379.2 */
	value += 104858; /* round, 0.05V * 2 ^ 21 == 104857.6 */
	ret3[0] = (value >> 21) + '0';
	value &= (1 << 21) - 1;
	value *= 10;
	ret3[2] = (value >> 21) + '0';
    strcpy(ret2, ret3);
//st_draw_string(50, 50, oldret, ST_COLOR_YELLOW, &font_ubuntu_48);
if (strcmp(ret2, oldret69) != 0) {
 st_fill_rect(52, 55, 62, 50, ST_COLOR_YELLOW);
st_draw_string(50, 50, ret2, ST_COLOR_BLACK, &font_ubuntu_48);
//    for (unsigned p = 0; p < 92660000; p++)   //  alright with red bg 92660000
//	  {
//		__asm__("nop");
//	  }

 strcpy(oldret69, ret3);
    }
}

static void adc_once(void)
{
    static char ret3[] = "0.0V ";

    if (humanreadable == 0) {
    for(int i=0; i < SAMPLES_PER_MESSAGE; i++) {
//	uint8_t channels[] = { ADC_CHANNEL2, };
	uint32_t value;

//	adc_set_regular_sequence(ADC1, 1, channels);
//	adc_start_conversion_regular(ADC1);
//	while (!(adc_eoc(ADC1)))
//	;

//    samples[num_samples++] = adc_read_regular(ADC1);
//	value = adc_read_regular(ADC1);
	value = adc_res[0];
	samples[num_samples++] = (uint32_t)value;
//	printf("samples %d %ld\r", i, samples[i]);
	    }
	} else {
//	uint8_t channels[] = { ADC_CHANNEL2, };
	uint32_t value;

//	adc_set_regular_sequence(ADC1, 1, channels);
//	adc_start_conversion_regular(ADC1);
//	while (!(adc_eoc(ADC1)))
//	;
    value = adc_res[0];
	value = adc_read_regular(ADC1);
	value *= 3379; /* 3.3 * 1024 == 3379.2 */
	value += 104858; /* round, 0.05V * 2 ^ 21 == 104857.6 */
	ret3[0] = (value >> 21) + '0';
	value &= (1 << 21) - 1;
	value *= 10;
	ret3[2] = (value >> 21) + '0';

    strcpy(ret2, ret3);
    }
    return;
}

static void adc_ss_out_cb(usbd_device *dev, uint8_t ep)
{
	(void) ep;
	(void)dev;
	uint16_t x;
	/* TODO - if you're really keen, perf test this. tiva implies it matters */
	/* char buf[64] __attribute__ ((aligned(4))); */
	uint8_t buf[BULK_EP_MAXPACKET + 1] __attribute__ ((aligned(2)));
	uint8_t *dest;
	dest = buf;

	x = usbd_ep_read_packet(dev, ep, dest, BULK_EP_MAXPACKET);
    if (x) {
        ;
        }
}



static void adc_ss_in_cb(usbd_device *dev, uint8_t ep)
{
	(void) dev;
//	uint8_t buf[BULK_EP_MAXPACKET + 1] __attribute__ ((aligned(2)));
// usbd_ep_nak_set(dev, ep, 1);
    adc_once();

    if (humanreadable == 0) {
    uint16_t x = usbd_ep_write_packet(dev, ep,&samples, sizeof(samples));
 //   while (usbd_ep_write_packet(dev, ep, &samples, sizeof(samples)) == 0);

    if (x != BULK_EP_MAXPACKET) {
	;
	}
//	samples[0] = 0;
    } else {
    uint16_t x = usbd_ep_write_packet(dev, ep,&ret2, 5);
//    while (usbd_ep_write_packet(dev, ep, &ret2, sizeof(&ret2)) == 0);


    if (x != BULK_EP_MAXPACKET) {
	;
	}

	}
	if (num_samples >= SAMPLES_PER_MESSAGE) {
    num_samples = 0;
    }

// usbd_ep_nak_set(dev, ep, 0);

}

void usbadc_set_config(usbd_device *dev, uint16_t wValue)
{
//	(void)wValue;
    configured = wValue;

		usbd_ep_setup(dev, 0x06, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			adc_ss_out_cb);
		usbd_ep_setup(dev, 0x86, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			adc_ss_in_cb);
		usbd_register_control_callback(
			dev,
			USB_REQ_TYPE_VENDOR,
			USB_REQ_TYPE_TYPE,
			adc_control_request);
		/* Prime source for IN data. */
		adc_ss_in_cb(dev, 0x86);
}
