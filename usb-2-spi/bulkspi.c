#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include "st7789_stm32_spi.h"
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include "cdc.h"
#include "usb.h"
#include "bulkspi.h"

#define CMD_ECHO       0
#define CMD_CS         1
#define CMD_SET_DELAY  2
#define CMD_GET_STATUS 3
#define CMD_TX         64
#define CMD_RX         65
#define CMD_TXRX       6
#define CMD_HZ         67
#define CMD_MODE       68
#define CMD_BPW        69
#define CMD_LSB        160
#define CMD_SPI_IO     4
#define CMD_SPI_BEGIN  1  
#define CMD_SPI_END    2 


#define STATUS_IDLE	   0
uint8_t spibuf[64];
static int configured;



uint8_t spibuf512[512];
uint32_t spirxlen = 0;

void recusb_sendspi(usbd_device *dev, uint32_t len);
void recspi_sendusb(usbd_device *dev, uint32_t len);

static uint8_t status = STATUS_IDLE;

unsigned char my_spi_read8(unsigned long spi);

void upd_spi_mode(int mode);

void upd_spi_bpw(int bpw);

void upd_spi_lsb(int lsb);

unsigned char my_spi_read8(unsigned long spi)
{
unsigned char d;
//SPI_CR1(spi) |= SPI_CR1_CSTART;
//while(!(SPI_SR(spi) & SPI_SR_RXP));
d = spi_read(spi);
//while( !( SPI_SR(spi) & SPI_SR_RXWNE));
//my_spi_flush(spi);
return d;
}

void my_spi_flush(unsigned long spi, uint32_t i)
{

while((SPI_SR(spi) & SPI_SR_RXWNE) || ( (SPI_SR(spi) & (SPI_SR_RXPLVL_MASK<<SPI_SR_RXPLVL_SHIFT)) !=0 ) )

spibuf512[i] = SPI_RXDR(spi);
//printf("flush = %d\r\n", spibuf512[i]);
}

void my_spi_send8(unsigned long spi,unsigned char d)
{
SPI_CR1(spi) |= SPI_CR1_CSTART;
while(!(SPI_SR(spi) & SPI_SR_TXP));
SPI_TXDR8(spi)=d;
while( !( SPI_SR(spi) & SPI_SR_TXC));
}


void upd_spi_div(int spihz);
int is_first_closest(int hz);

static int nums[9] = {9999,4900,2400,1150,615,302,146,77,38};

int is_first_closest(int hz) {
    int clock = rcc_get_spi_clk_freq(SPI2);
    int divis = 2;
    int ii = 0;
    float result = 000000000;
    for (divis = 2; divis < 513; divis = divis * 2) {
    result = (clock / ((double)divis));
    nums[ii] = ((result / 10000) - 1);
    ii++;
    }
    for (int i = 0; i < 9; ++i) {
        if (hz > nums[i]) {
        printf("clock set to %d\r\n", (nums[i] * 10000));
            return i;
        }
    }
    return 1;
}

void upd_spi_div(int spihz) {
    int result;
    spi_disable(SPI2);
    result = is_first_closest(spihz);
    spi_set_baudrate_prescaler(SPI2, result);
    printf("spi divider set to %d\r\n", result);
    spi_enable(SPI2);
}

void upd_spi_mode(int mode) {
    spi_disable(SPI2);
    spi_set_standard_mode(SPI2, mode);
    spi_enable(SPI2);
}

void upd_spi_bpw(int bpw) {
    spi_disable(SPI2);
    spi_set_data_size(SPI2, bpw);
    spi_enable(SPI2);
}

void upd_spi_lsb(int lsb) {
    spi_disable(SPI2);
    if (lsb == 0x08) {
        spi_send_lsb_first(SPI2);
        } else {
        spi_send_msb_first(SPI2);
        }
    spi_enable(SPI2);
}

static enum usbd_request_return_codes spi_control_request(
    usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
    uint16_t *len,
    void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{   
    (void)len;
    (void) buf;
	(void)usbd_dev;
	(void)complete;
    static uint8_t reply_buf[64];
    static uint8_t test[64];
	uint8_t cmd = req->bRequest;
	uint8_t cscmd = req->wIndex;
	uint8_t size = req->wValue;
	uint8_t spidir = req->wLength;
	uint8_t nsize = size / 8;
	
	switch (req->bRequest) {

    case CMD_BPW:
        upd_spi_bpw(req->wValue - 1);
        return USBD_REQ_HANDLED;
        
    case CMD_MODE:
        upd_spi_mode(req->wValue);
        return USBD_REQ_HANDLED;
   
    case CMD_HZ:
        upd_spi_div(req->wValue);
        printf("changing spi speed value from usb = %d\r\n", req->wValue);
        return USBD_REQ_HANDLED;
        
	case CMD_TX:
	    spirxlen = size;
//        ST_CS_IDLE;
		return USBD_REQ_HANDLED;

	case CMD_RX:
        if (cmd == CMD_RX) {
            ST_CS_ACTIVE;
 //           printf("recspi_sendusb size = %d\r\n", size);
            recspi_sendusb(usbd_dev, size);
	        ST_CS_IDLE;
	    return USBD_REQ_HANDLED;
        } 
		return USBD_REQ_HANDLED;

	default:
		break;

	}

	return USBD_REQ_NEXT_CALLBACK;
}

void recusb_sendspi(usbd_device *dev, uint32_t len) {
	uint8_t buf[BULK_EP_MAXPACKET + 1] = {0}; // __attribute__ ((aligned(2)));
	uint8_t *dest;
    int x;

	x = usbd_ep_read_packet(dev, 0x02, buf, BULK_EP_MAXPACKET);
    if (x) {
        ;
        }
    
    for (uint16_t i = 0; i < len; i++)
        {
            my_spi_send8(SPI2, buf[i]);
//            put_status("send loop");
            my_spi_flush(SPI2, i);
//            printf("spi send just sent = %d\r\n", buf[i]);
	    }
}

void recspi_sendusb(usbd_device *dev, uint32_t len) {
	uint16_t x;
    uint8_t buf[BULK_EP_MAXPACKET + 1] __attribute__ ((aligned(2)));

    memcpy(buf, &spibuf512, len);
//    for (uint16_t i = 0; i < len; i++)
//        {
//            printf("values sending to usb buf = %d\r\n", buf[i]);
//	    }
    x = usbd_ep_write_packet(dev, 0x82, buf, len);
    if (x != BULK_EP_MAXPACKET) {
	;
	}
}

void spi_ss_out_cb(usbd_device *dev, uint8_t ep);
void spi_ss_in_cb(usbd_device *dev, uint8_t ep);

void spi_ss_out_cb(usbd_device *dev, uint8_t ep)
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
    
        for (uint16_t i = 0; i < spirxlen; i++)
        {
            ST_CS_ACTIVE;
            my_spi_send8(SPI2, dest[i]);
//            put_status("send callback");
            my_spi_flush(SPI2, i);
//            printf("spi send just sent = %d\r\n", dest[i]);
            ST_CS_IDLE;
	    }
}

void spi_ss_in_cb(usbd_device *dev, uint8_t ep)
{
	(void) dev;
	(void) ep;

}

void usbspi_set_config(usbd_device *dev, uint16_t wValue)
{
    configured = wValue;

		usbd_ep_setup(dev, 0x02, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			spi_ss_out_cb);
		usbd_ep_setup(dev, 0x82, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			spi_ss_in_cb);
		usbd_register_control_callback(
			dev,
			USB_REQ_TYPE_VENDOR,
			USB_REQ_TYPE_TYPE,
			spi_control_request);
		/* Prime source for IN data. */
		spi_ss_in_cb(dev, 0x82);
}
