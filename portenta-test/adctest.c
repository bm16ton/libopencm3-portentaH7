#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include "adctest.h"
#include "st7789_stm32_spi.h"
#include "fonts/font_fixedsys_mono_24.h"
#include "fonts/font_ubuntu_48.h"
#include "usb.h"

bool is_configured = true;
static int configured;
int humanreadable = 1;
#define SAMPLES_PER_MESSAGE 30
int num_samples = 0;
uint32_t samples[SAMPLES_PER_MESSAGE] __attribute__ ((aligned(2)));
//static void send_samples(uint8_t ep);
static void adc_once(void);

uint8_t ADC_CHANNEL2 = 12;
const unsigned char CHAN4[] = { 12 };

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

static char ret2[] = "0.0V ";

void adcswitch(int onoff) {
adc_power_off(ADC1);
num_samples = 0;
printf("fuuuck\r\n");
humanreadable = onoff;
adc_start();
}

void adc_start(void)
{
//	rcc_periph_clock_enable(RCC_ADC12);


//	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
	gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);
//	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_ADC12);
//	adc_set_clk_prescale(ADC_CCR_PRESC_2);
    adc_power_off(ADC1);
	adc_set_clock_param(ADC1, ADC_CCR_CKMODE_DIV2, ADC_CCR_PRESC_2);
	

//	adc_disable_scan_mode(ADC1);

	adc_set_sample_time(ADC1, ADC_CHANNEL2, ADC_SMPR_SMP_2DOT5CYC);
	adc_set_resolution(ADC1, 12);
	adc_enable_regulator(ADC1);
	adc_set_pcsel(ADC1, 1, CHAN4);
	adc_calibrate(ADC1);
//	uint8_t channels[] = {ADC_CHANNEL2};
//	adc_set_multi_mode(ADC_CCR_MULTI_DUAL_INTERLEAVED);
	adc_power_on(ADC1);
/*
    adc_enable_scan_mode(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
//    adc_disable_discontinuous_mode_regular(ADC1);
//    adc_enable_external_trigger_regular(ADC1, ADC_CR2_SWSTART, ADC_CR2_EXTEN_DISABLED);
    adc_set_right_aligned(ADC1);
    uint8_t channels[] = {ADC_CHANNEL2};
    //ADC_CCR_MULTI_INDEPENDENT
    //ADC_CCR_MULTI_TRIPLE_INTERLEAVED
    adc_set_regular_sequence(ADC1, 1, channels);
    adc_start_conversion_regular(ADC1);
    */
}


static void adc_once(void)
{
    static char ret[] = "0.0V ";
    if (humanreadable == 0) {
    for(int i=0; i < SAMPLES_PER_MESSAGE; i++) {
	uint8_t channels[] = { ADC_CHANNEL2, };
	uint32_t value;

	adc_set_regular_sequence(ADC1, 1, channels);
	adc_start_conversion_regular(ADC1);
	while (!(adc_eoc(ADC1)))
	;

	value = adc_read_regular(ADC1);
	samples[num_samples++] = (uint32_t)value;
	}
	} else {
		uint8_t channels[] = { ADC_CHANNEL2, };
	uint32_t value;

	adc_set_regular_sequence(ADC1, 1, channels);
	adc_start_conversion_regular(ADC1);
	while (!(adc_eoc(ADC1)))
	;
	
	value = adc_read_regular(ADC1);
//    value = adc_res[0];
	value *= 3379; /* 3.3 * 1024 == 3379.2 */
	value += 104858; /* round, 0.05V * 2 ^ 21 == 104857.6 */
	ret[0] = (value >> 21) + '0';
	value &= (1 << 21) - 1;
	value *= 10;
	ret[2] = (value >> 21) + '0';

    strcpy(ret2, ret);
//    printf("adc_res 1 and 2 %4lu  %4lu \n", adc_res[0], adc_res[1]);
//   st_fill_rect(1, 110, 74, 60, ST_COLOR_YELLOW);
//   st_draw_string(10, 65, "Target IO", ST_COLOR_DARKGREEN, &font_ubuntu_48);
//	st_draw_string(10, 110, ret, ST_COLOR_RED, &font_ubuntu_48);
	
	
    samples[num_samples++] = (uint32_t)ret;
    }
    if (num_samples == SAMPLES_PER_MESSAGE) {
//        send_samples(ep);
 //       num_samples = 0;
    return;
    }
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
//	uint8_t *src;


    adc_once();

    if (humanreadable == 0) {
    uint16_t x = usbd_ep_write_packet(dev, ep,&samples, sizeof(samples));
    if (x != BULK_EP_MAXPACKET) {
	;
	}
    } else {
    uint16_t x = usbd_ep_write_packet(dev, ep,&ret2, sizeof(ret2));
    if (x != BULK_EP_MAXPACKET) {
	;
	}
	}
	if (num_samples == SAMPLES_PER_MESSAGE) {
//        send_samples(ep);
    num_samples = 0;
    }

	//assert(x == sizeof(buf));

}

void usbadc_set_config(usbd_device *dev, uint16_t wValue)
{
//	(void)wValue;
    configured = wValue;

		usbd_ep_setup(dev, 0x03, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			adc_ss_out_cb);
		usbd_ep_setup(dev, 0x83, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			adc_ss_in_cb);
		usbd_register_control_callback(
			dev,
			USB_REQ_TYPE_VENDOR,
			USB_REQ_TYPE_TYPE,
			adc_control_request);
		/* Prime source for IN data. */
		adc_ss_in_cb(dev, 0x83);
}
