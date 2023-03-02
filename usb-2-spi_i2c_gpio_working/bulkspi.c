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
//#include "st7789_stm32_spi.h"
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
#define CMD_TXZEROS    63
#define SPION          74
#define SPIOFF         75
#define CMD_RX         65
#define CMD_TXRX       66
#define CMD_HZ         67
#define CMD_MODE       68
#define CMD_BPW        69
#define CMD_FRST_TX    70
#define CMD_CSON       71
#define CMD_CSOFF      72
#define CMD_LSB        160
#define CMD_SPI_IO     4
#define CMD_SPI_BEGIN  1  
#define CMD_SPI_END    2 

#define STATUS_IDLE	   0
uint8_t spibuf[64];
static int configured;

uint8_t spibuf512[1024] = {0x00};

int diffr = 0;
//void recusb_sendspi(uint16_t len);
int recspi_sendusb(usbd_device *dev, uint16_t len);

static uint8_t status = STATUS_IDLE;

unsigned char my_spi_read8(unsigned long spi);

void upd_spi_mode(int mode);

void upd_spi_bpw(int bpw);

void upd_spi_lsb(int lsb);

unsigned char my_spi_read8(unsigned long spi)
{
unsigned char d;
d = spi_read(spi);
return d;
}

int firstboot = 1;
int offset = 0;

uint8_t my_spi_flush(unsigned long spi)
{
uint8_t temp = 0;
while((SPI_SR(spi) & SPI_SR_RXWNE) || ( (SPI_SR(spi) & (SPI_SR_RXPLVL_MASK<<SPI_SR_RXPLVL_SHIFT)) !=0 ) )

temp = SPI_RXDR8(spi);
return temp;
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

static int nums[9] = {9996,4900,2400,1150,615,302,146,77,38};

int is_first_closest(int hz) {
    int clock = rcc_get_spi_clk_freq(SPI2);
    int divis = 2;
    int ii = 0;
    float result = 000000000;
    for (divis = 2; divis < 513; divis = divis * 2) {
    result = (clock / ((double)divis));
    nums[ii] = ((result / 10000) - 2);
    ii++;
    }
    for (int i = 0; i < 9; ++i) {
        if (hz > nums[i]) {
        printf("clock set to %d\r\n", (nums[i]  + 2)* 10000);
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
//	(void)usbd_dev;
	(void)complete;
    static uint8_t reply_buf[64];
    static uint8_t test[64];
	uint8_t cmd = req->bRequest;
	uint8_t cscmd = req->wIndex;
	uint8_t size = req->wValue;
	uint8_t spidir = req->wLength;
	uint8_t nsize = size / 8;
	uint8_t dest2[] = {00};
	int ret;
	int actsz;
	static int p = 0;
	uint8_t temp;
	
	switch (req->bRequest) {

    case CMD_BPW:
        upd_spi_bpw(req->wValue - 1);
        return USBD_REQ_HANDLED;
        
    case CMD_MODE:
        upd_spi_mode(req->wValue);
        return USBD_REQ_HANDLED;
   
    case CMD_HZ:
        upd_spi_div(req->wValue);
        return USBD_REQ_HANDLED;

    case CMD_CSON:
        gpio_clear(GPIOA, GPIO8);
       for (unsigned i = 0; i < 80; i++)
        {
        __asm__("nop");
        }
        return USBD_REQ_HANDLED;
        
    case CMD_CSOFF:
        gpio_set(GPIOA, GPIO8);
       for (unsigned i = 0; i < 80; i++)
        {
        __asm__("nop");
        }
        return USBD_REQ_HANDLED;
        
    case CMD_TXZEROS:
	    dest2[0] = 0x00;
	    gpio_clear(GPIOA, GPIO8);
	    for (uint16_t i = 0; i < req->wValue; i++)
        {
            gpio_clear(GPIOA, GPIO8);
            __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
            while (!(SPI_SR(SPI2) & SPI_SR_TXP)) {
            ;
            }
           	my_spi_send8(SPI2, dest2[0]);
           	while (!(SPI_SR(SPI2) & SPI_SR_TXP)) {
            ;
            }
		    spibuf512[i] = my_spi_flush(SPI2);
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    gpio_set(GPIOA, GPIO8);
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
	    }
        gpio_set(GPIOA, GPIO8);
		return USBD_REQ_HANDLED;

    case CMD_FRST_TX:
        dest2[0] = 0x00;
       my_spi_send8(SPI2, dest2[0]);
        temp = spi_read8(SPI2);
        if (temp) {
        ;
        }
        return USBD_REQ_HANDLED;
        
	case CMD_RX:
        ret = recspi_sendusb(usbd_dev, (uint16_t)req->wValue);
        if (ret) {
        ;
        }
	    offset = 0;
		return USBD_REQ_HANDLED;
		
	case SPION:
		spi_enable(SPI2);
	    SPI_CR1(SPI2) |= SPI_CR1_CSTART;
	    printf("enabling spi\r\n");
	    return USBD_REQ_HANDLED;

	case SPIOFF:
		spi_disable(SPI2);
	    printf("disabling spi\r\n");
	    return USBD_REQ_HANDLED;
	default:
		break;

	}

	return USBD_REQ_NEXT_CALLBACK;
}

int recspi_sendusb(usbd_device *dev, uint16_t len) {
	uint16_t x;
    uint8_t buf[BULK_EP_MAXPACKET + 1] __attribute__ ((aligned(2)));

    memcpy(buf, spibuf512, len);
    x = usbd_ep_write_packet(dev, 0x83, buf, len);
    if (x != BULK_EP_MAXPACKET) {
	;
	}
	return 0;
}

void spi_ss_out_cb(usbd_device *dev, uint8_t ep);
void spi_ss_in_cb(usbd_device *dev, uint8_t ep);


void spi_ss_out_cb(usbd_device *dev, uint8_t ep)
{
	uint16_t x;
	uint8_t buf[BULK_EP_MAXPACKET + 1] __attribute__ ((aligned(2)));
	uint8_t *dest;
	uint8_t tempy;
	dest = buf;
	
	x = usbd_ep_read_packet(dev, ep, dest, BULK_EP_MAXPACKET);
    if (x) {
        ;
        }

    usbd_ep_nak_set(dev, ep, 1);
        if (x > 0) {
        for (uint16_t i = 0; i < x; i++)
        {
            while (!(SPI_SR(SPI2) & SPI_SR_TXP));
            my_spi_send8(SPI2, dest[i]);
            
            while (!(SPI_SR(SPI2) & SPI_SR_TXP)) {
                ;
            }
		    spibuf512[i] = my_spi_flush(SPI2);
		        }
		     }
	usbd_ep_nak_set(dev, ep, 0);

}

void spi_ss_in_cb(usbd_device *dev, uint8_t ep)
{
	(void) dev;
	(void) ep;
 	uint16_t x;
 	uint16_t t;
}

void usbspi_set_config(usbd_device *dev, uint16_t wValue)
{
    configured = wValue;

		usbd_ep_setup(dev, 0x03, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			spi_ss_out_cb);
		usbd_ep_setup(dev, 0x83, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			spi_ss_in_cb);
		usbd_register_control_callback(
			dev,
			USB_REQ_TYPE_VENDOR,
			USB_REQ_TYPE_TYPE,
			spi_control_request);
		
		spi_ss_in_cb(dev, 0x83);
}
