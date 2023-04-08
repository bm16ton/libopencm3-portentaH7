#ifndef _I2CUSB_H
#define _I2CUSB_H

#include <libopencm3/usb/usbd.h>
#include <stdint.h>

//extern usbd_device *usbd_dev;
uint64_t time64_now(void);
uint32_t time_now(void);
void usb_set_config(usbd_device *usbd_dev, uint16_t wValue);
//void i2c_init(void);

#endif
