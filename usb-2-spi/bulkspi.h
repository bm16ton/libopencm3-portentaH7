#ifndef BULKSPI_H
#define BULKSPI_H

#include <libopencm3/usb/usbd.h>

void spi_start(void);
void usbspi_set_config(usbd_device *dev, uint16_t wValue);
void my_spi_flush(unsigned long spi, uint32_t i);
void my_spi_send8(unsigned long spi,unsigned char d);
void usb_set_config(usbd_device *usbd_dev, uint16_t wValue);

#endif
