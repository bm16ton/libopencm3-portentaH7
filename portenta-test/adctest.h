#ifndef ADCTEST_H
#define ADCTEST_H

#include <libopencm3/stm32/common/adc_common_v2.h>
#include <libopencm3/stm32/common/adc_common_v2_multi.h>

void adcswitch(int onoff);
void adc_start(void);
void usbadc_set_config(usbd_device *dev, uint16_t wValue);



#endif
