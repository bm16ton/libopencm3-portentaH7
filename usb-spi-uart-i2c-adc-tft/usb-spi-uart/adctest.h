#ifndef ADCTEST_H
#define ADCTEST_H

#include <libopencm3/stm32/common/adc_common_v2.h>
#include <libopencm3/stm32/common/adc_common_v2_multi.h>
extern int adclivestop;
unsigned adc_liveonce(void);
uint32_t adc_liveonce2(void);
void adcswitch(int onoff);
void adc_start(void);
void usbadc_set_config(usbd_device *dev, uint16_t wValue);
void adc_always(void);
void adc_livestart(void);
void adclive(void);
extern int liveadc;
void adc_dma_setup(void);
#endif
